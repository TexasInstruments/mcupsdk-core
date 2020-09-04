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
 *  \file stw_mcanTestcases.h
 *
 *  \brief This file defines the test cases for mcan UT.
 */

#ifndef TEST_MCAN_TEST_CASES_H_
#define TEST_MCAN_TEST_CASES_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "test_mcan.h"
#include "test_mcanTestCasesConfig.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define MCAN_NUM_TESTCASES               (sizeof (gMcanTestcaseParams) / sizeof (st_mcanTestcaseParams_t))
#define AVV_TEST_ENABLE                  (0U)
#define MCAN_MANUAL_TEST_ENABLE          (0U)
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** \brief Defines the various mcan test cases. */
st_mcanTestcaseParams_t gMcanTestcaseParams[] =
{
    /* enableTest, testCaseId,
     * *reqId,
     * *testCaseName,
     * *userInfo, *disableReason
     * cpuID,
     * mcanConfigParams,
     * printEnable,
     * testType,
     */
    {
        TEST_ENABLE, 1234U,
        "None",
        "MCAN: CAN FD Mode with Bit Rate Switching ON And Bitrate 1MBps/5MBps",
        "None", "None",
        "1.Sent message shall match with received message. 2.Tx Event message marker shall match with sent message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[1U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[2U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1235U,
        "None",
        "MCAN: Internal Loopback, High Priority And Bitrate 1MBps/2.5MBps",
        "None", "None",
        "Sent message shall match with received message with High Priority.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[1U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[2U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[7U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[5U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1236U,
        "None",
        "MCAN: Internal Loopback, High Priority And Bitrate 250Kbps/5MBps",
        "None", "None",
        "Sent message shall match with received message with High Priority.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[2U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[2U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[7U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[5U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1237U,
        "None",
        "MCAN: Internal Loopback, High Priority And Bitrate 125Kbps/5MBps",
        "None", "None",
        "Sent message shall match with received message with High Priority.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[3U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[2U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[7U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[5U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1239U,
        "None",
        "MCAN: CAN FD Mode with Bit Rate Switching ON Extended Id Test",
        "None", "None",
        "1.Sent message shall match with received message. 2.Tx Event message marker shall match with sent message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[1U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[1U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1240U,
        "None",
        "MCU MCAN 0: Classic CAN Mode",
        "None", "None",
        "Sent message shall match with received message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[0U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            2U, /* tx message number */
            1U, /* standard ID message filter number */
            1U, /* extended ID message filter number */
            &canTxMSG[3U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[2U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_1)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1242U,
        "None",
        "MCU MCAN 0: Classic CAN Mode",
        "None", "None",
        "Sent message shall match with received message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[0U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            2U, /* tx message number */
            1U, /* standard ID message filter number */
            1U, /* extended ID message filter number */
            &canTxMSG[3U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[2U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_1)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1243U,
        "None",
        "MCAN: Tx Mixed Config. With Buffer and Queue",
        "None", "None",
        "Sent messages shall match with received message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[2U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[1U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            2U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[8U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1244U,
        "None",
        "MCAN: Tx Mixed Config. With Buffer and FIFO",
        "None", "None",
        "Sent messages shall match with received message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[2U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            2U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[8U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1245U,
        "None",
        "MCAN: CAN FD Mode with Bit Rate Switching ON And MAX TX Buffer Test",
        "None", "None",
        "1.Sent message shall match with received message. 2.Tx Event message marker shall match with sent message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[1U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[2U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1246U,
        "None",
        "MCAN: CAN FD Mode with Bit Rate Switching ON And RX Buffer Test",
        "None", "None",
        "1.Sent message shall match with received message. 2.Tx Event message marker shall match with sent message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[1U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[2U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1247U,
        "None",
        "MCAN: CAN FD Mode with Bit Rate Switching OFF, RX FIFO 0 Test",
        "None", "None",
        "Sent message shall match with received message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[0U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            3U, /* tx message number */
            1U, /* standard ID message filter number */
            1U, /* extended ID message filter number */
            &canTxMSG[5U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[3U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1248U,
        "None",
        "MCAN: Rx FIFO 1 Test mode",
        "None", "None",
        "Sent message shall match with received message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[1U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[2U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            0U, /* standard ID message filter number */
            0U, /* extended ID message filter number */
            &canTxMSG[15U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1249U,
        "None",
        "MCAN: Rx FIFO 0 Message Lost Test",
        "In this TC, no filters will be configured and all incoming messages will be stored into FIFO(by accepting non-matching frame) for later comparison.", "None",
        "1. In Blocking Mode, after FIFO(FIFO1) full, new message shall be rejected. 2. In Overwrite Mode, after FIFO(FIFO0) full, new message shall be overwritten on oldest message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[1U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[2U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            0U, /* standard ID message filter number */
            0U, /* extended ID message filter number */
            &canTxMSG[17U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1250U,
        "None",
        "MCAN: Rx FIFO 0 Message Lost Test",
        "In this TC, no filters will be configured and all incoming messages will be stored into FIFO(by accepting non-matching frame) for later comparison.", "None",
        "1. In Blocking Mode, after FIFO(FIFO1) full, new message shall be rejected. 2. In Overwrite Mode, after FIFO(FIFO0) full, new message shall be overwritten on oldest message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[1U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[2U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            0U, /* standard ID message filter number */
            0U, /* extended ID message filter number */
            &canTxMSG[15U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1251U,
        "None",
        "MCAN: CAN FD Mode with Bit Rate Switching ON And MAX TX Buffer Test",
        "None", "None",
        "1.Sent message shall match with received message. 2.Tx Event message marker shall match with sent message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[1U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[2U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 2022U,
        "None",
        "MCAN: Tx and Rx Throughput Standard ID",
        "None", "None",
        "Sent messages shall match with received message and measured frames per seconds should be 85% of the theoretical.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1000U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[2U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[18U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[7U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 2023U,
        "None",
        "MCAN: Tx and Rx Throughput Extended ID",
        "None", "None",
        "Sent messages shall match with received message and measured frames per seconds should be 85% of the theoretical.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1000U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[2U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[19U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[3U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1252U,
        "None",
        "MCAN: High Priority Messages",
        "None", "None",
        "Sent message shall match with received message with High Priority.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[2U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[7U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[5U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1254U,
        "None",
        "MCAN: Message Cancel Test",
        "None", "None",
        "MCAN shall cancel pending messages.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[2U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[0U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1255U,
        "None",
        "MCAN: LOOPBACK Automatic Retransmission Test",
        "None", "None",
        "1.Sent message shall match with received message. 2.Tx Event message marker shall match with sent message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[1U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[2U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1258U,
        "None",
        "MCAN: LOOPBACK Transmitter Delay Compensation Test",
        "None", "None",
        "1.Sent message shall match with received message. 2.Tx Event message marker shall match with sent message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[1U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[2U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1260U,
        "None",
        "MCU MCAN 0: Classic CAN Mode",
        "None", "None",
        "Sent message shall match with received message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[0U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            2U, /* tx message number */
            1U, /* standard ID message filter number */
            1U, /* extended ID message filter number */
            &canTxMSG[3U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[2U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_1)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1262U,
        "None",
        "MCAN: Revision ID Test",
        "None", "None",
        "Sent message shall match with received message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[0U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            2U, /* tx message number */
            1U, /* standard ID message filter number */
            1U, /* extended ID message filter number */
            &canTxMSG[3U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[2U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_1)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1263U,
        "None",
        "MCAN: Pin State",
        "None", "None",
        "Configured PAD values shall match with read values.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[2U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[2U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1264U,
        "None",
        "MCAN: LOOPBACK Endianess Test",
        "None", "None",
        "1.Sent message shall match with received message. 2.Tx Event message marker shall match with sent message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[1U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[2U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1265U,
        "None",
        "MCAN: Rx FIFO Block Mode/Overwrite mode",
        "In this TC, no filters will be configured and all incoming messages will be stored into FIFO(by accepting non-matching frame) for later comparison.", "None",
        "1. In Blocking Mode, after FIFO(FIFO1) full, new message shall be rejected. 2. In Overwrite Mode, after FIFO(FIFO0) full, new message shall be overwritten on oldest message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[1U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[2U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            0U, /* standard ID message filter number */
            0U, /* extended ID message filter number */
            &canTxMSG[15U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1266U,
        "None",
        "MCAN: Acceptance Filter Range Filter Test",
        "None", "None",
        "1.Sent message shall match with received message. 2.Tx Event message marker shall match with sent message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[1U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[2U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1267U,
        "None",
        "MCAN: CAN FD Mode with Bit Rate Switching ON, DUAL FILTER ID Test",
        "None", "None",
        "1.Sent message shall match with received message. 2.Tx Event message marker shall match with sent message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[1U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[2U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[6U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1268U,
        "None",
        "MCAN: Internal Loopback, Classic BitMask Filter Test 1MBps/2.5MBps",
        "None", "None",
        "Sent message shall match with received message with High Priority.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[1U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[2U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[7U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[5U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1633U,
        "None",
        "MCAN: External Time-Stamp Code Coverage Improvement Test",
        "None", "None",
        "1.Sent message shall match with received message.2.Tx Event message marker shall match with sent message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[3U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            1U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[2U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1634,
        "None",
        "MCAN: Time Stamp Counter Reset Code Coverage Improvement Test",
        "None", "None",
        "Time Stamp Counter shall get reset after calling TS Reset API.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[1U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[2U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1269U,
        "None",
        "MCAN: CAN FD Mode with Bit Rate Switching ON",
        "None", "None",
        "1.Sent message shall match with received message. 2.Tx Event message marker shall match with sent message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[1U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[2U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1261U,
        "None",
        "MCAN: Clock Stop Request",
        "None", "None",
        "MCAN shall ack back the clock stop request.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[2U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[2U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1635U,
        "None",
        "MCAN: Code Coverage Enhancement Test, Error Test",
        "None", "None",
        "None",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[4U], /* mcan module bit timing parameters */
            &canFDInitParams[1U], /* mcan module initialization parameters */
            &canFDConfigParams[5U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[3U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[2U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1636U,
        "None",
        "MCAN: Code Coverage Enhancement Test, Error Test",
        "None", "None",
        "None",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[5U], /* mcan module bit timing parameters */
            &canFDInitParams[1U], /* mcan module initialization parameters */
            &canFDConfigParams[5U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[3U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[2U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 4104U,
        "None",
        "MCAN: Tx and Rx Acceptance Filter Test with SFID as 0xFFFFFFFF",
        "None", "None",
        "Sent messages shall match with received message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[2U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[20U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[8U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
#if (MCAN_MANUAL_TEST_ENABLE == 1U)
    {
        TEST_ENABLE, 1259U,
        "Test Setup: Connect MCAN HIGH and MCAN LOW Pins to PCAN Tool",
        "MCAN: CAN FD Bus Monitor Test",
        "None", "None",
        "1.Sent message from PCAN shall match with received message.",
        IPU,
        {
            (MCAN_TEST_TYPE_EXTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[7U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[2U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1256U,
        "Test Setup: Short MCAN_HIGH and MCAN_LOW Pins",
        "MCAN: CAN FD Error Passive Test",
        "None", "None",
        "1.Error Passive Status Should Occur.",
        IPU,
        {
            (MCAN_TEST_TYPE_EXTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[1U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[2U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1257U,
        "Test Setup: Short MCAN_HIGH and MCAN_LOW Pins",
        "MCAN: CAN FD Bus Off Test",
        "None", "None",
        "1.BUS Off Should Occur.",
        IPU,
        {
            (MCAN_TEST_TYPE_EXTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[1U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[2U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
#endif

#if (AVV_TEST_ENABLE == 1U)
    {
        TEST_ENABLE, 21U,
        "None",
        "MCAN: CAN FD Mode latency measure",
        "None", "None",
        "1.Access MCAN RAM and Registers measure latency.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            2U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[1U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[2U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE,22U,
        "None",
        "MCAN: Throughput=>EXT ID",
        "None", "None",
        "Sent messages shall match with received message and measured frames per seconds should be 85% of the theoretical.",
        IPU,
        {
#if defined (SOC_AM65XX) || defined (SOC_J721E) || defined (SOC_AM64X)
            (MCAN_TEST_TYPE_B2B), /* testType */
#else
            (MCAN_TEST_TYPE_B2B), /* testType */
#endif
            6000U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[0U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[1U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_PERFORMANCE),
    },
    {
        TEST_ENABLE, 23U,
        "None",
        "MCAN: Tx and Rx Throughput",
        "None", "None",
        "Sent messages shall match with received message and measured frames per seconds should be 85% of the theoretical.",
        IPU,
        {
#if defined (SOC_AM65XX) || defined (SOC_J721E) || defined (SOC_AM64X)
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
#else
            (MCAN_TEST_TYPE_B2B), /* testType */
#endif
            60U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[2U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[1U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_PERFORMANCE),
    },
    {
        TEST_ENABLE, 24U,
        "None",
        "MCAN: Message Arbitration",
        "In this TC, no filters will be configured and all incoming messages will be stored into FIFO(by accepting non-matching frame) for later comparison.", "None",
        "1.Sent message shall match with received message. 2.Tx Event message marker shall match with sent message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[1U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            4U, /* tx message number */
            0U, /* standard ID message filter number */
            0U, /* extended ID message filter number */
            &canTxMSG[11U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 25U,
        "None",
        "MCAN: Rx FIFO Block Mode/Overwrite mode",
        "In this TC, no filters will be configured and all incoming messages will be stored into FIFO(by accepting non-matching frame) for later comparison.", "None",
        "1. In Blocking Mode, after FIFO(FIFO1) full, new message shall be rejected. 2. In Overwrite Mode, after FIFO(FIFO0) full, new message shall be overwritten on oldest message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[1U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[2U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            0U, /* standard ID message filter number */
            0U, /* extended ID message filter number */
            &canTxMSG[15U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 26U,
        "None",
        "MCAN: Tx and Rx Throughput: Classic CAN",
        "None", "None",
        "Sent messages shall match with received message and measured frames per seconds should be 85% of the theoretical.",
        IPU,
        {
#if defined (SOC_AM65XX) || defined (SOC_J721E) || defined (SOC_AM64X)
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
#else
            (MCAN_TEST_TYPE_B2B), /* testType */
#endif
            60U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[2U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            0U, /* extended ID message filter number */
            &canTxMSG[16U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[1U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_PERFORMANCE),
    },
    {
        TEST_ENABLE, 27U,
        "None",
        "MCAN: DMA Events - Classic CAN",
        "None", "None",
        "Sent messages shall match with received message. ",
        IPU,
        {
#if defined (SOC_AM65XX) || defined (SOC_J721E) || defined (SOC_AM64X)
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
#else
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
#endif
            100U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[1U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[16U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[1U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND & ~MCAN_INTR_SRC_BIT_ERR_UNCORRECTED & ~MCAN_INTR_SRC_BIT_ERR_CORRECTED),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_FULL),
    },
    {
        TEST_ENABLE, 99U,
        "None",
        "User defined test",
        "None", "None",
        "None",
        IPU,
        {
            (MCAN_TEST_TYPE_B2B), /* testType */
            2U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[0U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[0U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 100U,
        "None",
        "User defined test",
        "None", "None",
        "None",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            2U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[0U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            2U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[0U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1000U,
        "None",
        "MCAN: CAN FD Mode with Bit Rate Switching ON",
        "None", "None",
        "1.Sent message shall match with received message. 2.Tx Event message marker shall match with sent message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            2U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[1U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[2U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1001U,
        "None",
        "MCAN: Classic CAN Mode",
        "None", "None",
        "Sent message shall match with received message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[0U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            2U, /* tx message number */
            1U, /* standard ID message filter number */
            1U, /* extended ID message filter number */
            &canTxMSG[3U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[2U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1002U,
        "None",
        "MCAN: CAN FD Mode with Bit Rate Switching OFF",
        "None", "None",
        "Sent message shall match with received message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[0U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            3U, /* tx message number */
            1U, /* standard ID message filter number */
            1U, /* extended ID message filter number */
            &canTxMSG[5U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[3U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1003U,
        "None",
        "MCAN: External Time-Stamp",
        "None", "None",
        "1.Sent message shall match with received message.2.Tx Event message marker shall match with sent message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[3U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            1U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[2U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1004U,
        "None",
        "MCAN: High Priority Messages",
        "None", "None",
        "Sent message shall match with received message with High Priority.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[2U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[7U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[5U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1005U,
        "None",
        "MCAN: Internal Loopback",
        "None", "None",
        "Sent message shall match with received message with High Priority.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[2U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[0U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            1U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[7U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[5U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
    {
        TEST_ENABLE, 1006U,
        "None",
        "MCAN: Tx Mixed Config. With Buffer and FIFO",
        "None", "None",
        "Sent messages shall match with received message.",
        IPU,
        {
            (MCAN_TEST_TYPE_INTERNAL_LOOBACK), /* testType */
            1U, /* iteration count for tx */
            &canFDBitTimings[0U], /* mcan module bit timing parameters */
            &canFDInitParams[0U], /* mcan module initialization parameters */
            &canFDConfigParams[2U], /* mcan module configuration parameters */
            &canFDRAMConfigParams[1U], /* mcan module MSG RAM configuration parameters */
            &canFDECCConfigParams[0U], /* mcan module ECC configuration parameters */
            &canFDECCErrForceConfigParams[0U], /* mcan module ECC Error Force parameters */
            2U, /* tx message number */
            2U, /* standard ID message filter number */
            2U, /* extended ID message filter number */
            &canTxMSG[8U], /* tx Buffer elements/Tx message */
            &canSTDIDFilter[0U], /* standard message ID filters */
            &canEXTIDFilter[0U],  /* extended message ID filters */
            (MCAN_INTR_MASK_ALL & ~MCAN_INTR_SRC_RES_ADDR_ACCESS & ~MCAN_INTR_SRC_TIMESTAMP_WRAPAROUND),  /* Interrupt Enable/Disable Mask */
            (MCAN_INTR_MASK_ALL),  /* Interrupt Line Select Mask */
            (MCAN_INTR_LINE_NUM_0)  /* Interrupt Line Select */
        },
        PRINT_ENABLE,
        (ST_TT_SANITY),
    },
#endif
};

uint32_t bitTimingsListSize = (sizeof(canFDBitTimings) / sizeof(MCAN_BitTimingParams));

#endif /* #ifndef ST_MCAN_TEST_CASES_H_ */
