/*
 * Copyright (C) 2021 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TEST_FSI_COMMON_H
#define TEST_FSI_COMMON_H

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define FSI_TASK_PRIORITY   (8U)
#define FSI_TASK_STACK_SIZE (4U * 1024U)
/* CRC Value is calculated based on the TX pattern */
#define FSI_APP_TX_PATTERN_USER_CRC_VALUE (0x41U)
/* Configuring Frame - can be between 1-16U */
#define FSI_APP_FRAME_DATA_WORD_SIZE (16U)
/* 0x0U for 1 lane and 0x1U for two lane */
#define FSI_APP_N_LANES (FSI_DATA_WIDTH_1_LANE)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */
/** \brief Typedef for test case type function pointer */
typedef void (*FsiTestTestCaseFxnPtr)(void *args);

typedef struct FSI_TxTestParams_s {

    uint32_t baseAddr;
    uint32_t frameDataSize;
    uint32_t prescaleVal;
    uint32_t userData;
    uint32_t numLane;
    uint32_t frameDataTag;
    uint32_t frameType;
    uint32_t intrLine;
    uint32_t intrNum;
    uint32_t intrEvt;
    uint32_t userDefinedCrc;
    uint32_t testEccFlag;
    uint32_t delayLineCtrlTest;
    uint32_t rxFrameWDTest;
    uint32_t rxPingWDTest;
    uint32_t udataFilterTest;
    uint32_t rxTriggerTest;
    SemaphoreP_Object taskDoneSemaphoreObj;
} FSI_TxTestParams;

typedef struct FSI_RxTestParams_s {

    uint32_t baseAddr;
    uint32_t frameDataSize;
    uint32_t prescaleVal;
    uint32_t numLane;
    uint32_t frameType;
    uint32_t intrLine;
    uint32_t intrNum;
    uint32_t intrEvt;
    uint32_t userDefinedCrc;
    uint32_t testEccFlag;
    uint32_t delayLineCtrlTest;
    uint32_t rxFrameWDTest;
    uint32_t rxPingWDTest;
    uint32_t udataFilterTest;
    uint32_t userData;
    uint32_t rxTriggerTest;
    SemaphoreP_Object taskDoneSemaphoreObj;
} FSI_RxTestParams;

typedef struct FSI_MainTestParams_s {

    FsiTestTestCaseFxnPtr testCaseTxFxnPtr;
    FsiTestTestCaseFxnPtr testCaseRxFxnPtr;
    FSI_RxTestParams      *rxTestParams;
    FSI_TxTestParams      *txTestParams;
} FSI_MainTestParams;

#ifdef __cplusplus
}
#endif

#endif

/** @} */
