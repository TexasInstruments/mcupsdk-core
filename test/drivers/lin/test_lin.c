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
 */

/* Disable assert */

#ifdef DebugP_ASSERT_ENABLED
#undef DebugP_ASSERT_ENABLED
#endif /* DebugP_ASSERT_ENABLED */

#define DebugP_ASSERT_ENABLED 0

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <unity.h>
#include <drivers/lin.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define LIN_RAM_BASE  0x70071000

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*******************  TEST SUITE 1 ************************************/
static void LIN_setLINModeCheck(void *args);
static void LIN_setMaximumBaudRateCheck(void *args);
static void LIN_setMessageFilteringCheck(void *args);
static void LIN_enableParityCheck(void *args);
static void LIN_disableParityCheck(void *args);
static void LIN_setIDByteCheck(void *args);
static void LIN_setIDResponderTaskCheck(void *args);
static void LIN_sendWakeupSignalCheck(void *args);
static void LIN_enterSleepCheck(void *args);
static void LIN_sendChecksumCheck(void *args);
static void LIN_setFrameLengthCheck(void *args);
static void LIN_setCommModeCheck(void *args);

/*******************  TEST SUITE 2 ************************************/
static void LIN_isSCISpaceAvailableCheck(void *args);
static void LIN_readSCICharNonBlockingCheck(void *args);
static void LIN_readSCICharBlockingCheck(void *args);
static void LIN_writeSCICharNonBlockingCheck(void *args);
static void LIN_writeSCICharBlockingCheck(void *args);
static void LIN_enableSCIInterruptCheck(void *args);
static void LIN_disableSCIInterruptCheck(void *args);
static void LIN_clearSCIInterruptStatusCheck(void *args);
static void LIN_setSCIInterruptLevel0Check(void *args);
static void LIN_setSCIInterruptLevel1Check(void *args);
static void LIN_isSCIReceiverIdleCheck(void *args);
static void LIN_getSCITxFrameTypeCheck(void *args);
static void LIN_getSCIRxFrameTypeCheck(void *args);
static void LIN_isSCIBreakDetectedCheck(void *args);
static void LIN_enableModuleCheck(void *args);
static void LIN_disableModuleCheck(void *args);
static void LIN_setBaudRatePrescalerCheck(void *args);

/*******************  TEST SUITE 3 ************************************/
static void LIN_enableDataTransmitterCheck(void *args);
static void LIN_disableDataTransmitterCheck(void *args);
static void LIN_enableDataReceiverCheck(void *args);
static void LIN_disableDataReceiverCheck(void *args);
static void LIN_performSoftwareResetCheck(void *args);
static void LIN_enterSoftwareResetCheck(void *args);
static void LIN_exitSoftwareResetCheck(void *args);
static void LIN_isBusBusyCheck(void *args);
static void LIN_isTxBufferEmptyCheck(void *args);
static void LIN_enableExtLoopbackCheck(void *args);
static void LIN_disableExtLoopbackCheck(void *args);
static void LIN_enableIntLoopbackCheck(void *args);
static void LIN_getInterruptStatusCheck(void *args);
static void LIN_getInterruptLevelCheck(void *args);
static void LIN_getInterruptLine0OffsetCheck(void *args);
static void LIN_getInterruptLine1OffsetCheck(void *args);
static void LIN_enableMultibufferModeCheck(void *args);
static void LIN_disableMultibufferModeCheck(void *args);
static void LIN_setTransmitDelayCheck(void *args);
static void LIN_setPinSampleMaskCheck(void *args);

/*******************  TEST SUITE 4 ************************************/
static void LIN_setDebugSuspendModeCheck(void *args);
static void LIN_setCheckSumCheck(void *args);
static void LIN_enableGlobalInterruptCheck(void *args);
static void LIN_disableGlobalInterruptCheck(void *args);
static void LIN_sendDataCheck(void *args);
static void LIN_generateParityIDCheck(void *args);

/*******************  END OF STATIC FUNCTIONS ************************************/
extern void Unit_setupTxMask(uint16_t mask);
extern void Unit_setupRxMask(uint16_t mask);
extern void Unit_setupRxID(uint32_t ID);
extern void Unit_setupClearIntStatus();
extern void Unit_setupInterruptLevel(uint32_t ints);
extern void Unit_setupInterrupts(uint32_t flags);
extern void Unit_setupModuleErrors(uint32_t errors);
extern void Unit_initSCIModule();
extern void Unit_enableSCIParity(void);
extern void Unit_sendSCIDataNonBlock(uint16_t data);
extern void Unit_sendSCIDataBlock(uint16_t data);
extern void Unit_enableSCIErrors(uint16_t errors);
extern void Unit_enableExtLoopback(void);
extern void Unit_setupSCIInterrupts(uint32_t flags);
extern void Unit_setupSCIInterruptLevel(uint32_t ints);
extern void Unit_setupInterruptLine0Offset(uint16_t value);
extern void Unit_setupInterruptLine1Offset(uint16_t value);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static uint16_t myTxData[8]={1,2,3,4,5,6,7,8};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_main(void *args)
{
    /* Open drivers */
    Drivers_open();
    Board_driversOpen();

    UNITY_BEGIN();

    LIN_initModule(CONFIG_LIN1_BASE_ADDR);

    RUN_TEST(LIN_setLINModeCheck,  8038, NULL);
    RUN_TEST(LIN_setMaximumBaudRateCheck,  8038, NULL);
    RUN_TEST(LIN_setMessageFilteringCheck,  8038, NULL);
    RUN_TEST(LIN_enableParityCheck,  8038, NULL);
    RUN_TEST(LIN_disableParityCheck,  8038, NULL);
    RUN_TEST(LIN_setIDByteCheck,  8038, NULL);
    RUN_TEST(LIN_setIDResponderTaskCheck,  8038, NULL);
    RUN_TEST(LIN_sendWakeupSignalCheck,  8038, NULL);
    RUN_TEST(LIN_enterSleepCheck,  8038, NULL);
    RUN_TEST(LIN_sendChecksumCheck,  8038, NULL);
    RUN_TEST(LIN_setFrameLengthCheck,  8038, NULL);
    RUN_TEST(LIN_setCommModeCheck,  8038, NULL);

    RUN_TEST(LIN_isSCISpaceAvailableCheck,  8038, NULL);
    RUN_TEST(LIN_readSCICharNonBlockingCheck,  8038, NULL);
    RUN_TEST(LIN_readSCICharBlockingCheck,  8038, NULL);
    RUN_TEST(LIN_writeSCICharNonBlockingCheck,  8038, NULL);
    RUN_TEST(LIN_writeSCICharBlockingCheck,  8038, NULL);
    RUN_TEST(LIN_enableSCIInterruptCheck,  8038, NULL);
    RUN_TEST(LIN_disableSCIInterruptCheck,  8038, NULL);
    RUN_TEST(LIN_clearSCIInterruptStatusCheck,  8038, NULL);
    RUN_TEST(LIN_setSCIInterruptLevel0Check,  8038, NULL);
    RUN_TEST(LIN_setSCIInterruptLevel1Check,  8038, NULL);
    //RUN_TEST(LIN_isSCIReceiverIdleCheck,  8038, NULL);
    RUN_TEST(LIN_getSCITxFrameTypeCheck,  8038, NULL);
    RUN_TEST(LIN_getSCIRxFrameTypeCheck,  8038, NULL);
    RUN_TEST(LIN_isSCIBreakDetectedCheck,  8038, NULL);
    RUN_TEST(LIN_enableModuleCheck,  8038, NULL);
    RUN_TEST(LIN_disableModuleCheck,  8038, NULL);
    RUN_TEST(LIN_setBaudRatePrescalerCheck,  8038, NULL);

    RUN_TEST(LIN_enableDataTransmitterCheck,  8038, NULL);
    RUN_TEST(LIN_disableDataTransmitterCheck,  8038, NULL);
    RUN_TEST(LIN_enableDataReceiverCheck,  8038, NULL);
    RUN_TEST(LIN_disableDataReceiverCheck,  8038, NULL);
    RUN_TEST(LIN_performSoftwareResetCheck,  8038, NULL);
    RUN_TEST(LIN_enterSoftwareResetCheck,  8038, NULL);
    RUN_TEST(LIN_exitSoftwareResetCheck,  8038, NULL);
    RUN_TEST(LIN_isBusBusyCheck,  8038, NULL);
    RUN_TEST(LIN_isTxBufferEmptyCheck,  8038, NULL);
    RUN_TEST(LIN_enableExtLoopbackCheck,  8038, NULL);
    RUN_TEST(LIN_disableExtLoopbackCheck,  8038, NULL);
    RUN_TEST(LIN_enableIntLoopbackCheck,  8038, NULL);
    RUN_TEST(LIN_getInterruptStatusCheck,  8038, NULL);
    RUN_TEST(LIN_getInterruptLevelCheck,  8038, NULL);
    RUN_TEST(LIN_getInterruptLine0OffsetCheck,  8038, NULL);
    RUN_TEST(LIN_getInterruptLine1OffsetCheck,  8038, NULL);
    RUN_TEST(LIN_enableMultibufferModeCheck,  8038, NULL);
    RUN_TEST(LIN_disableMultibufferModeCheck,  8038, NULL);
    RUN_TEST(LIN_setTransmitDelayCheck,  8038, NULL);
    RUN_TEST(LIN_setPinSampleMaskCheck,  8038, NULL);

    RUN_TEST(LIN_setDebugSuspendModeCheck,  8038, NULL);
    RUN_TEST(LIN_setCheckSumCheck,  8038, NULL);
    RUN_TEST(LIN_enableGlobalInterruptCheck,  8038, NULL);
    RUN_TEST(LIN_disableGlobalInterruptCheck,  8038, NULL);
    RUN_TEST(LIN_sendDataCheck,  8038, NULL);
    RUN_TEST(LIN_generateParityIDCheck,  8038, NULL);

    UNITY_END();

    /* Close drivers */
    Board_driversClose();
    Drivers_close();

    return;
}

/* Unity framework required information */
void setUp(void)
{
}

void tearDown(void)
{
}

/* Testcase 1 - Check the LIN_setLINMode API */
static void LIN_setLINModeCheck(void *args)
{
    LIN_setLINMode(CONFIG_LIN1_BASE_ADDR, LIN_MODE_LIN_RESPONDER);

    /* Check if the value was written correctly */
    TEST_ASSERT_NOT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR1)
        & CSL_LIN_SCIGCR1_CLK_MASTER_MASK), CSL_LIN_SCIGCR1_CLK_MASTER_MASK, "Responder Mode Check failed.");

    LIN_setLINMode(CONFIG_LIN1_BASE_ADDR, LIN_MODE_LIN_COMMANDER);

    /* Check if the value was written correctly */
    /* (HWREGH(base + LIN_O_SCIGCR1) & LIN_SCIGCR1_CLK_MASTER) */
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR1)
        & CSL_LIN_SCIGCR1_CLK_MASTER_MASK), CSL_LIN_SCIGCR1_CLK_MASTER_MASK, "Commander Mode Check failed.");
}

/* Testcase 2 - Check the LIN_setMaximumBaudRate API */
static void LIN_setMaximumBaudRateCheck(void *args)
{
    LIN_setMaximumBaudRate(CONFIG_LIN1_BASE_ADDR, 120000000U);

    /* Check if the value was written correctly */
    /* HWREGH(base + LIN_O_MBRSR) */
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_MBRSR)
        & CSL_LIN_MBRSR_MBR_MASK)>>CSL_LIN_MBRSR_MBR_SHIFT, 6000U, "Maximum Baud Rate Check failed.");
}

/* Testcase 3 - Check the LIN_setMessageFiltering API */
static void LIN_setMessageFilteringCheck(void *args)
{
    LIN_setMessageFiltering(CONFIG_LIN1_BASE_ADDR, LIN_MSG_FILTER_IDBYTE);

    /* Check if the value was written correctly */
    /* (HWREGH(base + LIN_O_SCIGCR1) & LIN_SCIGCR1_HGENCTRL) */
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR1)
        & CSL_LIN_SCIGCR1_HGENCTRL_MASK), 0U, "MSG Filter check failed.");

    LIN_setMessageFiltering(CONFIG_LIN1_BASE_ADDR, LIN_MSG_FILTER_IDRESPONDER);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR1)
        & CSL_LIN_SCIGCR1_HGENCTRL_MASK), CSL_LIN_SCIGCR1_HGENCTRL_MASK, "MSG Filter check failed.");
}

/* Testcase 4 - Check the LIN_enableParity API */
static void LIN_enableParityCheck(void *args)
{
    LIN_enableParity(CONFIG_LIN1_BASE_ADDR);

    /* Check if the value was written correctly */
    /*(HWREGH(base + LIN_O_SCIGCR1) & LIN_SCIGCR1_PARITYENA) */
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR1)
        & CSL_LIN_SCIGCR1_PARITYENA_MASK), CSL_LIN_SCIGCR1_PARITYENA_MASK, "Parity check failed.");
}

/* Testcase 5 - Check the LIN_disableParity API */
static void LIN_disableParityCheck(void *args)
{
    LIN_disableParity(CONFIG_LIN1_BASE_ADDR);

    /* Check if the value was written correctly */
    /*(HWREGH(base + LIN_O_SCIGCR1) & LIN_SCIGCR1_PARITYENA) */
    TEST_ASSERT_NOT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR1)
        & CSL_LIN_SCIGCR1_PARITYENA_MASK), CSL_LIN_SCIGCR1_PARITYENA_MASK, "Parity check failed.");
}

/* Testcase 6 - Check the LIN_setIDByte API */
static void LIN_setIDByteCheck(void *args)
{
    LIN_setIDByte(CONFIG_LIN1_BASE_ADDR, 0xFAU);

    /* Check if the value was written correctly */
    /*(HWREGH(base + LIN_O_ID) & LIN_ID_IDBYTE_M)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_LINID)
                                        & CSL_LIN_LINID_IDBYTE_MASK), 0xFAU, "Set ID Byte check failed.");

    LIN_setIDByte(CONFIG_LIN1_BASE_ADDR, 0x5FU);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_LINID)
                                        & CSL_LIN_LINID_IDBYTE_MASK), 0x5FU, "Set ID Byte check failed.");

}

/* Testcase 7 - Check the LIN_setIDResponderTask API */
static void LIN_setIDResponderTaskCheck(void *args)
{
    LIN_setIDResponderTask(CONFIG_LIN1_BASE_ADDR, 0xAAU);

    /* Check if the value was written correctly */
    /*((HWREGH(base + LIN_O_ID) & LIN_ID_IDRESPONDERTASKBYTE_M) >> LIN_ID_IDRESPONDERTASKBYTE_S)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_LINID)
                                        & CSL_LIN_LINID_IDSLAVETASKBYTE_MASK)>>CSL_LIN_LINID_IDSLAVETASKBYTE_SHIFT, 0xAAU, "Set ID Responder check failed.");

    LIN_setIDResponderTask(CONFIG_LIN1_BASE_ADDR, 0x05U);

    /* Check if the value was written correctly */
    /*((HWREGH(base + LIN_O_ID) & LIN_ID_IDRESPONDERTASKBYTE_M) >> LIN_ID_IDTASKBYTE_S)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_LINID)
                                        & CSL_LIN_LINID_IDSLAVETASKBYTE_MASK)>>CSL_LIN_LINID_IDSLAVETASKBYTE_SHIFT, 0x05U, "Set ID Responder check failed.");
}

/* Testcase 8 - Check the LIN_sendWakeupSignal API */
static void LIN_sendWakeupSignalCheck(void *args)
{
    LIN_sendWakeupSignal(CONFIG_LIN1_BASE_ADDR);

    /* Check if the value was written correctly */
    /*(HWREGH(base + LIN_O_SCIGCR2) & LIN_SCIGCR2_GENWU)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR2)
                                        & CSL_LIN_SCIGCR2_GENWU_MASK), CSL_LIN_SCIGCR2_GENWU_MASK, "sendWakeupSignalCheck failed.");
}

/* Testcase 9 - Check the LIN_enterSleep API */
static void LIN_enterSleepCheck(void *args)
{
    LIN_enterSleep(CONFIG_LIN1_BASE_ADDR);

    /* Check if the value was written correctly */
    /*(HWREGH(base + LIN_O_SCIGCR2) & LIN_SCIGCR2_POWERDOWN)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR2)
                                        & CSL_LIN_SCIGCR2_POWERDOWN_MASK), CSL_LIN_SCIGCR2_POWERDOWN_MASK, "Enter Sleep failed.");
}

/* Testcase 10 - Check the LIN_sendChecksum API */
static void LIN_sendChecksumCheck(void *args)
{
    LIN_sendChecksum(CONFIG_LIN1_BASE_ADDR);

    /* Check if the value was written correctly */
    /*(HWREG_BP(base + LIN_O_SCIGCR2) & LIN_SCIGCR2_SC)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR2)
                                        & CSL_LIN_SCIGCR2_SC_MASK), CSL_LIN_SCIGCR2_SC_MASK, "Send Checksum check failed.");
}

/* Testcase 11 - Check the LIN_triggerChecksumCompare API */
static void LIN_triggerChecksumCompareCheck(void *args)
{
    LIN_triggerChecksumCompare(CONFIG_LIN1_BASE_ADDR);

    /* Check if the value was written correctly */
    /*(HWREG_BP(base + LIN_O_SCIGCR2) & LIN_SCIGCR2_CC)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR2)
                                        & CSL_LIN_SCIGCR2_CC_MASK), CSL_LIN_SCIGCR2_CC_MASK, "Send Checksum Compare check failed.");
}

/* Testcase 12 - Check the LIN_isTxReady API */
static void LIN_isTxReadyCheck(void *args)
{
    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE((LIN_isTxReady(CONFIG_LIN1_BASE_ADDR)), CSL_TRUE, "Tx Ready Check failed.");
}

/* Testcase 13 - Check the LIN_setFrameLength API */
static void LIN_setFrameLengthCheck(void *args)
{
    LIN_setFrameLength(CONFIG_LIN1_BASE_ADDR, 0x1U);

    /* Check if the value was written correctly */
    /*((HWREG_BP(base + LIN_O_SCIFORMAT) & LIN_SCIFORMAT_LENGTH_M) >> LIN_SCIFORMAT_LENGTH_S)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE(((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIFORMAT)
                                        & CSL_LIN_SCIFORMAT_LENGTH_MASK) >> CSL_LIN_SCIFORMAT_LENGTH_SHIFT), 0x0U, "Sent Frame Length failed.");

    LIN_setFrameLength(CONFIG_LIN1_BASE_ADDR, 0x4U);

    /* Check if the value was written correctly */
    /*((HWREG_BP(base + LIN_O_SCIFORMAT) & LIN_SCIFORMAT_LENGTH_M) >> LIN_SCIFORMAT_LENGTH_S)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE(((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIFORMAT)
                                        & CSL_LIN_SCIFORMAT_LENGTH_MASK) >> CSL_LIN_SCIFORMAT_LENGTH_SHIFT), 0x3U, "Sent Frame Length failed.");

    LIN_setFrameLength(CONFIG_LIN1_BASE_ADDR, 0x8U);

    /* Check if the value was written correctly */
    /*((HWREG_BP(base + LIN_O_SCIFORMAT) & LIN_SCIFORMAT_LENGTH_M) >> LIN_SCIFORMAT_LENGTH_S)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE(((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIFORMAT)
                                        & CSL_LIN_SCIFORMAT_LENGTH_MASK) >> CSL_LIN_SCIFORMAT_LENGTH_SHIFT), 0x7U, "Sent Frame Length failed.");
}

static void LIN_setCommModeCheck(void *args)
{
    LIN_setCommMode(CONFIG_LIN1_BASE_ADDR, LIN_COMM_LIN_USELENGTHVAL);

    /* Check if the value was written correctly */
    /*(HWREGH(base + LIN_O_SCIGCR1) & LIN_SCIGCR1_COMMMODE)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE(((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR1)
                                        & CSL_LIN_SCIGCR1_COMMMODE_MASK) >> CSL_LIN_SCIGCR1_COMMMODE_SHIFT), LIN_COMM_LIN_USELENGTHVAL, "LIN_setCommMode check failed.");

    LIN_setCommMode(CONFIG_LIN1_BASE_ADDR, LIN_COMM_LIN_ID4ID5LENCTL);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE(((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR1)
                                        & CSL_LIN_SCIGCR1_COMMMODE_MASK) >> CSL_LIN_SCIGCR1_COMMMODE_SHIFT), LIN_COMM_LIN_ID4ID5LENCTL, "LIN_setCommMode check failed.");
}

static void LIN_isSCISpaceAvailableCheck(void *args)
{
    Unit_initSCIModule();

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE(LIN_isSCISpaceAvailable(CONFIG_LIN1_BASE_ADDR),
                                        CSL_TRUE, "LIN_isSCISpaceAvailable check failed.");
}

static void LIN_readSCICharNonBlockingCheck(void *args)
{
    Unit_sendSCIDataNonBlock(0xAAU);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE(LIN_readSCICharNonBlocking(LIN_RAM_BASE, CSL_TRUE),
                                        0xAAU, "LIN_readSCICharNonBlocking check failed.");

    Unit_sendSCIDataNonBlock(0x55U);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE(LIN_readSCICharNonBlocking(LIN_RAM_BASE, CSL_FALSE),
                                        0x55U, "LIN_readSCICharNonBlocking check failed.");
}

static void LIN_readSCICharBlockingCheck(void *args)
{
    Unit_sendSCIDataBlock(0xAAU);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE(LIN_readSCICharBlocking(LIN_RAM_BASE, CSL_TRUE),
                                        0xAAU, "LIN_readSCICharNonBlocking check failed.");

    Unit_sendSCIDataBlock(0x55U);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE(LIN_readSCICharBlocking(LIN_RAM_BASE, CSL_FALSE),
                                        0x55U, "LIN_readSCICharNonBlocking check failed.");
}

static void LIN_writeSCICharNonBlockingCheck(void *args)
{
    Unit_initSCIModule();

    LIN_writeSCICharNonBlocking(CONFIG_LIN1_BASE_ADDR, 0xAFU);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCITD)
                                        & CSL_LIN_SCITD_TD_MASK), 0xAFU, "LIN_writeSCICharNonBlocking check failed.");
}

static void LIN_writeSCICharBlockingCheck(void *args)
{
    Unit_initSCIModule();

    LIN_writeSCICharBlocking(CONFIG_LIN1_BASE_ADDR, 0xCCU);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCITD)
                                        & CSL_LIN_SCITD_TD_MASK), 0xCCU, "LIN_writeSCICharNonBlocking check failed.");
}

static void LIN_enableSCIInterruptCheck(void *args)
{
    Unit_initSCIModule();

    LIN_enableSCIInterrupt(CONFIG_LIN1_BASE_ADDR, LIN_SCI_INT_ALL);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE(HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCISETINT),
                                        LIN_SCI_INT_ALL, "LIN_enableSCIInterrupt check failed.");

    LIN_enableSCIInterrupt(CONFIG_LIN1_BASE_ADDR, LIN_SCI_INT_RX);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE(HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCISETINT) & LIN_SCI_INT_RX,
                                        LIN_SCI_INT_RX, "LIN_enableSCIInterrupt check failed.");
}

static void LIN_disableSCIInterruptCheck(void *args)
{
    Unit_initSCIModule();
    Unit_setupSCIInterrupts(LIN_SCI_INT_TX);

    LIN_disableSCIInterrupt(CONFIG_LIN1_BASE_ADDR, LIN_SCI_INT_TX);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE(HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCISETINT) & LIN_SCI_INT_TX,
                                        0x0, "LIN_disableSCIInterrupt check failed.");

    Unit_initSCIModule();
    Unit_setupSCIInterrupts(LIN_SCI_INT_OVERRUN);

    LIN_disableSCIInterrupt(CONFIG_LIN1_BASE_ADDR, LIN_SCI_INT_OVERRUN);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE(HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCISETINT) & LIN_SCI_INT_OVERRUN,
                                        0x0, "LIN_disableSCIInterrupt check failed.");
}

static void LIN_clearSCIInterruptStatusCheck(void *args)
{
    Unit_initSCIModule();
    HW_WR_REG32_RAW(LIN_RAM_BASE + CSL_LIN_SCIFLR, 0);

    LIN_clearSCIInterruptStatus(LIN_RAM_BASE, LIN_SCI_INT_BREAK);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE(HW_RD_REG32_RAW(LIN_RAM_BASE + CSL_LIN_SCIFLR),
                                        LIN_SCI_INT_BREAK, "LIN_clearSCIInterruptStatus check failed.");

    Unit_initSCIModule();
    HW_WR_REG32_RAW(LIN_RAM_BASE + CSL_LIN_SCIFLR, 0);

    LIN_clearSCIInterruptStatus(LIN_RAM_BASE, LIN_SCI_INT_ALL);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE(HW_RD_REG32_RAW(LIN_RAM_BASE + CSL_LIN_SCIFLR),
                                        LIN_SCI_INT_ALL, "LIN_clearSCIInterruptStatus check failed.");
}

static void LIN_setSCIInterruptLevel0Check(void *args)
{
    Unit_initSCIModule();
    Unit_setupSCIInterruptLevel((LIN_SCI_INT_BREAK | LIN_SCI_INT_OVERRUN));

    LIN_setSCIInterruptLevel0(CONFIG_LIN1_BASE_ADDR, (LIN_SCI_INT_BREAK | LIN_SCI_INT_OVERRUN));

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE(HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCISETINTLVL),
                                        0x0U, "LIN_setSCIInterruptLevel0 check failed.");

    Unit_setupSCIInterruptLevel((LIN_SCI_INT_TX | LIN_SCI_INT_RX));
    LIN_setSCIInterruptLevel0(CONFIG_LIN1_BASE_ADDR, (LIN_SCI_INT_TX | LIN_SCI_INT_RX));

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE(HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCISETINTLVL),
                                        0x0U, "LIN_setSCIInterruptLevel0 check failed.");
}

static void LIN_setSCIInterruptLevel1Check(void *args)
{
    Unit_initSCIModule();

    LIN_setSCIInterruptLevel1(CONFIG_LIN1_BASE_ADDR, (LIN_SCI_INT_BREAK | LIN_SCI_INT_OVERRUN));

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE(HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCISETINTLVL),
                                        (LIN_SCI_INT_BREAK | LIN_SCI_INT_OVERRUN), "LIN_setSCIInterruptLevel1 check failed.");

    LIN_setSCIInterruptLevel1(CONFIG_LIN1_BASE_ADDR, (LIN_SCI_INT_TX | LIN_SCI_INT_RX));

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE(HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCISETINTLVL) & (LIN_SCI_INT_TX | LIN_SCI_INT_RX),
                                        (LIN_SCI_INT_TX | LIN_SCI_INT_RX), "LIN_setSCIInterruptLevel1 check failed.");
}

static void LIN_isSCIReceiverIdleCheck(void *args)
{
    Unit_initSCIModule();

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE(LIN_isSCIReceiverIdle(CONFIG_LIN1_BASE_ADDR),
                                        CSL_TRUE, "LIN_isSCIReceiverIdle check failed.");
}

static void LIN_getSCITxFrameTypeCheck(void *args)
{
    Unit_initSCIModule();

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE(LIN_getSCITxFrameType(CONFIG_LIN1_BASE_ADDR),
                                        CSL_FALSE, "LIN_getSCITxFrameType check failed.");
}

static void LIN_getSCIRxFrameTypeCheck(void *args)
{
    Unit_initSCIModule();

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE(LIN_getSCIRxFrameType(CONFIG_LIN1_BASE_ADDR),
                                        CSL_FALSE, "LIN_getSCIRxFrameType check failed.");
}

static void LIN_isSCIBreakDetectedCheck(void *args)
{
    Unit_initSCIModule();

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE(LIN_isSCIBreakDetected(CONFIG_LIN1_BASE_ADDR),
                                        CSL_FALSE, "LIN_isSCIBreakDetected check failed.");
}

static void LIN_enableModuleCheck(void *args)
{
    LIN_initModule(CONFIG_LIN1_BASE_ADDR);

    LIN_enableModule(CONFIG_LIN1_BASE_ADDR);

    /* Check if the value was written correctly */
    /*(HWREGH(base + LIN_O_SCIGCR0) & LIN_SCIGCR0_RESET)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE(HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR0) & (CSL_LIN_SCIGCR0_RESET_MASK),
                                        (CSL_LIN_SCIGCR0_RESET_MASK), "LIN_enableModule check failed.");

    /*HWREGH(base + LIN_O_SCIPIO0)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE(HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIPIO0) & (CSL_LIN_SCIPIO0_RXFUNC_MASK | CSL_LIN_SCIPIO0_TXFUNC_MASK),
                                        (CSL_LIN_SCIPIO0_RXFUNC_MASK | CSL_LIN_SCIPIO0_TXFUNC_MASK), "LIN_enableModule check failed.");
}

static void LIN_disableModuleCheck(void *args)
{
    LIN_disableModule(CONFIG_LIN1_BASE_ADDR);

    /* Check if the value was written correctly */
    /*(HWREGH(base + LIN_O_SCIGCR0) & LIN_SCIGCR0_RESET)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE(HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR0) & (CSL_LIN_SCIGCR0_RESET_MASK),
                                        (0x0U), "LIN_disableModule check failed.");

    /*HWREGH(base + LIN_O_SCIPIO0)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE(HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIPIO0) & (CSL_LIN_SCIPIO0_RXFUNC_MASK | CSL_LIN_SCIPIO0_TXFUNC_MASK),
                                        (0x0U), "LIN_disableModule check failed.");
}

static void LIN_setBaudRatePrescalerCheck(void *args)
{
    LIN_initModule(CONFIG_LIN1_BASE_ADDR);

    LIN_setBaudRatePrescaler(CONFIG_LIN1_BASE_ADDR, 0xABABCU, 0x3U);

    /* Check if the value was written correctly */
    /*HWREG_BP(base + LIN_O_BRSR)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE(HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_BRSR),
                                        (0x30ABABCU), "LIN_setBaudRatePrescaler check failed.");

    LIN_setBaudRatePrescaler(CONFIG_LIN1_BASE_ADDR, 0x555AAU, 0x1U);

    /* Check if the value was written correctly */
    /*HWREG_BP(base + LIN_O_BRSR)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE(HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_BRSR),
                                        (0x10555AAU), "LIN_setBaudRatePrescaler check failed.");
}

static void LIN_setDebugSuspendModeCheck(void *args)
{
    LIN_initModule(CONFIG_LIN1_BASE_ADDR);

    LIN_setDebugSuspendMode(CONFIG_LIN1_BASE_ADDR, LIN_DEBUG_FROZEN);

    /* Check if the value was written correctly */
    /*(HWREG_BP(base + LIN_O_SCIGCR1) & LIN_SCIGCR1_CONT)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_CONT_MASK),
                                        (0x0U), "LIN_setDebugSuspendMode check failed.");

    LIN_setDebugSuspendMode(CONFIG_LIN1_BASE_ADDR, LIN_DEBUG_COMPLETE);

    /* Check if the value was written correctly */
    /*(HWREG_BP(base + LIN_O_SCIGCR1) & LIN_SCIGCR1_CONT)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_CONT_MASK),
                                        (CSL_LIN_SCIGCR1_CONT_MASK), "LIN_setDebugSuspendMode check failed.");
}

static void LIN_setCheckSumCheck(void *args)
{
    LIN_initModule(CONFIG_LIN1_BASE_ADDR);

    LIN_setChecksumType(CONFIG_LIN1_BASE_ADDR, LIN_CHECKSUM_CLASSIC);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_CTYPE_MASK),
                                        (0x0U), "LIN_setDebugSuspendMode check failed.");

    LIN_setChecksumType(CONFIG_LIN1_BASE_ADDR, LIN_CHECKSUM_ENHANCED);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_CTYPE_MASK),
                                        (CSL_LIN_SCIGCR1_CTYPE_MASK), "LIN_setDebugSuspendMode check failed.");
}

static void LIN_enableGlobalInterruptCheck(void *args)
{
    LIN_initModule(CONFIG_LIN1_BASE_ADDR);
    LIN_disableGlobalInterrupt(CONFIG_LIN1_BASE_ADDR, LIN_INTERRUPT_LINE0);
    LIN_disableGlobalInterrupt(CONFIG_LIN1_BASE_ADDR, LIN_INTERRUPT_LINE1);

    LIN_enableGlobalInterrupt(CONFIG_LIN1_BASE_ADDR, LIN_INTERRUPT_LINE0);

    /* Check if the value was written correctly */
    /*(HWREGH(base + LIN_O_GLB_INT_EN) & 0x3)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_LIN_GLB_INT_EN) & 0x3),
                                        (CSL_LIN_LIN_GLB_INT_FLG_INT0_FLG_MASK << LIN_INTERRUPT_LINE0), "LIN_enableGlobalInterrupt check failed.");

    LIN_disableGlobalInterrupt(CONFIG_LIN1_BASE_ADDR, LIN_INTERRUPT_LINE0);

    LIN_enableGlobalInterrupt(CONFIG_LIN1_BASE_ADDR, LIN_INTERRUPT_LINE1);

    /*(HWREGH(base + LIN_O_GLB_INT_EN) & 0x3)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_LIN_GLB_INT_EN) & 0x3),
                                        (CSL_LIN_LIN_GLB_INT_FLG_INT0_FLG_MASK << LIN_INTERRUPT_LINE1), "LIN_enableGlobalInterrupt check failed.");

    LIN_disableGlobalInterrupt(CONFIG_LIN1_BASE_ADDR, LIN_INTERRUPT_LINE1);
}

static void LIN_disableGlobalInterruptCheck(void *args)
{
    LIN_disableGlobalInterrupt(CONFIG_LIN1_BASE_ADDR, LIN_INTERRUPT_LINE0);

    /* Check if the value was written correctly */
    /*(HWREGH(base + LIN_O_GLB_INT_EN) & 0x3)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_LIN_GLB_INT_EN) & 0x3),
                                        (0x0U), "LIN_enableGlobalInterrupt check failed.");

    LIN_disableGlobalInterrupt(CONFIG_LIN1_BASE_ADDR, LIN_INTERRUPT_LINE1);

    /* Check if the value was written correctly */
    /*(HWREGH(base + LIN_O_GLB_INT_EN) & 0x3)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_LIN_GLB_INT_EN) & 0x3),
                                        (0x0U), "LIN_enableGlobalInterrupt check failed.");
}

static void LIN_sendDataCheck(void *args)
{
    LIN_initModule(CONFIG_LIN1_BASE_ADDR);
    HW_WR_REG32_RAW((CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIFORMAT), HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIFORMAT)|((uint32_t)(7U) << CSL_LIN_SCIFORMAT_LENGTH_SHIFT));

    LIN_sendData(CONFIG_LIN1_BASE_ADDR, &myTxData[0U]);

    /* Check if the value was written correctly */
    /*HWREG_BP(base + LIN_O_TD0)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_LINTD0)),
                                        (0x1020304UL), "LIN_sendData check failed.");

    /* Check if the value was written correctly */
    /*HWREG_BP(base + LIN_O_TD1)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_LINTD1)),
                                        (0x5060708UL), "LIN_sendData check failed.");
}

static void LIN_generateParityIDCheck(void *args)
{
    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE(LIN_generateParityID(0x2FU),
                                        (0x6FU), "LIN_generateParityIDCheck check failed.");

    TEST_ASSERT_EQUAL_INT32_MESSAGE(LIN_generateParityID(0xE5U),
                                        (0xE5U), "LIN_generateParityIDCheck check failed.");
}

static void LIN_enableDataTransmitterCheck(void *args)
{
    LIN_initModule(CONFIG_LIN1_BASE_ADDR);

    LIN_enableDataTransmitter(CONFIG_LIN1_BASE_ADDR);

    /* Check if the value was written correctly */
    /*(HWREG_BP(base + LIN_O_SCIGCR1) & LIN_SCIGCR1_TXENA)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_TXENA_MASK),
                                        CSL_LIN_SCIGCR1_TXENA_MASK, "LIN_enableDataTransmitter check failed.");
}

static void LIN_disableDataTransmitterCheck(void *args)
{
    LIN_disableDataTransmitter(CONFIG_LIN1_BASE_ADDR);

    /* Check if the value was written correctly */
    /*(HWREG_BP(base + LIN_O_SCIGCR1) & LIN_SCIGCR1_TXENA)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_TXENA_MASK),
                                        0x0U, "LIN_disableDataTransmitter check failed.");
}

static void LIN_enableDataReceiverCheck(void *args)
{
    LIN_initModule(CONFIG_LIN1_BASE_ADDR);

    LIN_enableDataReceiver(CONFIG_LIN1_BASE_ADDR);

    /* Check if the value was written correctly */
    /*(HWREG_BP(base + LIN_O_SCIGCR1) & LIN_SCIGCR1_RXENA)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_RXENA_MASK),
                                        CSL_LIN_SCIGCR1_RXENA_MASK, "LIN_enableDataReceiver check failed.");
}

static void LIN_disableDataReceiverCheck(void *args)
{
    LIN_disableDataReceiver(CONFIG_LIN1_BASE_ADDR);

    /* Check if the value was written correctly */
    /*(HWREG_BP(base + LIN_O_SCIGCR1) & LIN_SCIGCR1_RXENA)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_RXENA_MASK),
                                        (0x0U), "LIN_enableDataReceiver check failed.");
}

static void LIN_performSoftwareResetCheck(void *args)
{
    LIN_initModule(CONFIG_LIN1_BASE_ADDR);

    LIN_performSoftwareReset(CONFIG_LIN1_BASE_ADDR);

    /* Check if the value was written correctly */
    /*(HWREGH(base + LIN_O_SCIGCR1) & LIN_SCIGCR1_SWNRST)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_SWNRST_MASK),
                                        (CSL_LIN_SCIGCR1_SWNRST_MASK), "LIN_enableDataReceiver check failed.");
}

static void LIN_enterSoftwareResetCheck(void *args)
{
    LIN_initModule(CONFIG_LIN1_BASE_ADDR);

    LIN_enterSoftwareReset(CONFIG_LIN1_BASE_ADDR);

    /* Check if the value was written correctly */
    /*(HWREGH(base + LIN_O_SCIGCR1) & LIN_SCIGCR1_SWNRST)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_SWNRST_MASK),
                                        (0x0U), "LIN_enterSoftwareReset check failed.");
}

static void LIN_exitSoftwareResetCheck(void *args)
{
    LIN_initModule(CONFIG_LIN1_BASE_ADDR);

    LIN_exitSoftwareReset(CONFIG_LIN1_BASE_ADDR);

    /* Check if the value was written correctly */
    /*(HWREGH(base + LIN_O_SCIGCR1) & LIN_SCIGCR1_SWNRST)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_SWNRST_MASK),
                                        (CSL_LIN_SCIGCR1_SWNRST_MASK), "LIN_exitSoftwareReset check failed.");
}

static void LIN_isBusBusyCheck(void *args)
{
    LIN_initModule(CONFIG_LIN1_BASE_ADDR);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE((LIN_isBusBusy(CONFIG_LIN1_BASE_ADDR)),
                                        (CSL_FALSE), "LIN_isBusBusyCheck check failed.");
}

static void LIN_isTxBufferEmptyCheck(void *args)
{
    LIN_initModule(CONFIG_LIN1_BASE_ADDR);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE((LIN_isTxBufferEmpty(CONFIG_LIN1_BASE_ADDR)),
                                        (CSL_TRUE), "LIN_isTxBufferEmpty check failed.");
}

static void LIN_enableExtLoopbackCheck(void *args)
{
    LIN_initModule(CONFIG_LIN1_BASE_ADDR);

    LIN_enableExtLoopback(CONFIG_LIN1_BASE_ADDR, LIN_LOOPBACK_DIGITAL, LIN_ANALOG_LOOP_NONE);

    /* Check if the value was written correctly */
    /*HWREGH(base + LIN_O_IODFTCTRL) & 0x3*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_IODFTCTRL) & 0x3U),
                                        ((LIN_LOOPBACK_DIGITAL << CSL_LIN_IODFTCTRL_LPBENA_SHIFT) | LIN_ANALOG_LOOP_NONE), "LIN_enableExtLoopback check failed.");

    LIN_initModule(CONFIG_LIN1_BASE_ADDR);

    LIN_enableExtLoopback(CONFIG_LIN1_BASE_ADDR, LIN_LOOPBACK_ANALOG, LIN_ANALOG_LOOP_TX);

    /* Check if the value was written correctly */
    /*HWREGH(base + LIN_O_IODFTCTRL) & 0x3*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_IODFTCTRL) & 0x3U),
                                        ((LIN_LOOPBACK_ANALOG << CSL_LIN_IODFTCTRL_LPBENA_SHIFT) | LIN_ANALOG_LOOP_TX), "LIN_enableExtLoopback check failed.");

    LIN_initModule(CONFIG_LIN1_BASE_ADDR);

    LIN_enableExtLoopback(CONFIG_LIN1_BASE_ADDR, LIN_LOOPBACK_ANALOG, LIN_ANALOG_LOOP_RX);

    /* Check if the value was written correctly */
    /*HWREGH(base + LIN_O_IODFTCTRL) & 0x3*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_IODFTCTRL) & 0x3U),
                                        ((LIN_LOOPBACK_ANALOG << CSL_LIN_IODFTCTRL_LPBENA_SHIFT) | LIN_ANALOG_LOOP_RX), "LIN_enableExtLoopback check failed.");
}

static void LIN_disableExtLoopbackCheck(void *args)
{
    Unit_enableExtLoopback();

    LIN_disableExtLoopback(CONFIG_LIN1_BASE_ADDR);

    /* Check if the value was written correctly */
    /*HWREGH(base + LIN_O_IODFTCTRL) & 0x3*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_IODFTCTRL) & 0x3U),
                                        (0x0U), "LIN_disableExtLoopback check failed.");
}

static void LIN_enableIntLoopbackCheck(void *args)
{
    LIN_initModule(CONFIG_LIN1_BASE_ADDR);

    LIN_enableIntLoopback(CONFIG_LIN1_BASE_ADDR);

    /* Check if the value was written correctly */
    /*(HWREG_BP(base + LIN_O_SCIGCR1) & LIN_SCIGCR1_LOOPBACK)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LOOPBACK_MASK),
                                        (CSL_LIN_SCIGCR1_LOOPBACK_MASK), "LIN_enableIntLoopback check failed.");
}

static void LIN_disableIntLoopbackCheck(void *args)
{
    LIN_initModule(CONFIG_LIN1_BASE_ADDR);

    LIN_disableIntLoopback(CONFIG_LIN1_BASE_ADDR);

    /* Check if the value was written correctly */
    /*(HWREG_BP(base + LIN_O_SCIGCR1) & LIN_SCIGCR1_LOOPBACK)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_LOOPBACK_MASK),
                                        (0x0U), "LIN_disableIntLoopback check failed.");
}

static void LIN_getInterruptStatusCheck(void *args)
{
    LIN_initModule(CONFIG_LIN1_BASE_ADDR);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE((LIN_getInterruptStatus(CONFIG_LIN1_BASE_ADDR)),
                                        (0x900U), "LIN_getInterruptStatus check failed.");
}

static void LIN_getInterruptLevelCheck(void *args)
{
    Unit_setupInterruptLevel(LIN_INT_ALL);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE((LIN_getInterruptLevel(CONFIG_LIN1_BASE_ADDR)),
                                        (LIN_INT_ALL), "LIN_getInterruptLevel check failed.");
}

static void LIN_getInterruptLine0OffsetCheck(void *args)
{
    Unit_setupInterruptLine0Offset(0x15U);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE((LIN_getInterruptLine0Offset(LIN_RAM_BASE)),
                                        (0x15U), "LIN_getInterruptLine0Offset check failed.");
}

static void LIN_getInterruptLine1OffsetCheck(void *args)
{
    Unit_setupInterruptLine1Offset(0x1CU);

    /* Check if the value was written correctly */
    TEST_ASSERT_EQUAL_INT32_MESSAGE((LIN_getInterruptLine1Offset(LIN_RAM_BASE)),
                                        (0x1CU), "LIN_getInterruptLine1Offset check failed.");
}

static void LIN_enableMultibufferModeCheck(void *args)
{
    LIN_initModule(CONFIG_LIN1_BASE_ADDR);

    LIN_enableMultibufferMode(CONFIG_LIN1_BASE_ADDR);

    /* Check if the value was written correctly */
    /*(HWREGH(base + LIN_O_SCIGCR1) & LIN_SCIGCR1_MBUFMODE)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_MBUFMODE_MASK),
                                        (CSL_LIN_SCIGCR1_MBUFMODE_MASK), "LIN_enableMultibufferMode check failed.");
}

static void LIN_disableMultibufferModeCheck(void *args)
{
    LIN_initModule(CONFIG_LIN1_BASE_ADDR);

    LIN_disableMultibufferMode(CONFIG_LIN1_BASE_ADDR);

    /* Check if the value was written correctly */
    /*(HWREGH(base + LIN_O_SCIGCR1) & LIN_SCIGCR1_MBUFMODE)*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_SCIGCR1) & CSL_LIN_SCIGCR1_MBUFMODE_MASK),
                                        (0x0U), "LIN_disableMultibufferMode check failed.");
}

static void LIN_setTransmitDelayCheck(void *args)
{
    LIN_initModule(CONFIG_LIN1_BASE_ADDR);

    LIN_setTransmitDelay(CONFIG_LIN1_BASE_ADDR, 0x0U);

    /* Check if the value was written correctly */
    /*(HWREG_BP(base + LIN_O_IODFTCTRL) & (0x3 << LIN_IODFTCTRL_TXSHIFT_S))*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_IODFTCTRL) & (0x7 << CSL_LIN_IODFTCTRL_TXSHIFT_SHIFT)),
                                        (0x0U), "LIN_setTransmitDelay check failed.");

    LIN_initModule(CONFIG_LIN1_BASE_ADDR);

    LIN_setTransmitDelay(CONFIG_LIN1_BASE_ADDR, 0x7);

    /* Check if the value was written correctly */
    /*(HWREG_BP(base + LIN_O_IODFTCTRL) & (0x3 << LIN_IODFTCTRL_TXSHIFT_S))*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_IODFTCTRL) & (0x7 << CSL_LIN_IODFTCTRL_TXSHIFT_SHIFT)),
                                        (0x7 << CSL_LIN_IODFTCTRL_TXSHIFT_SHIFT), "LIN_setTransmitDelay check failed.");
}

static void LIN_setPinSampleMaskCheck(void *args)
{
    LIN_initModule(CONFIG_LIN1_BASE_ADDR);

    LIN_setPinSampleMask(CONFIG_LIN1_BASE_ADDR, LIN_PINMASK_NONE);

    /* Check if the value was written correctly */
    /*(HWREG_BP(base + LIN_O_IODFTCTRL) & (0x3 << LIN_IODFTCTRL_PINSAMPLEMASK_S))*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_IODFTCTRL) & (0x3 << CSL_LIN_IODFTCTRL_PINSAMPLEMASK_SHIFT)),
                                        (LIN_PINMASK_NONE << CSL_LIN_IODFTCTRL_PINSAMPLEMASK_SHIFT), "LIN_setPinSampleMask check failed.");

    LIN_setPinSampleMask(CONFIG_LIN1_BASE_ADDR, LIN_PINMASK_CENTER_2SCLK);

    /* Check if the value was written correctly */
    /*(HWREG_BP(base + LIN_O_IODFTCTRL) & (0x3 << LIN_IODFTCTRL_PINSAMPLEMASK_S))*/
    TEST_ASSERT_EQUAL_INT32_MESSAGE((HW_RD_REG32_RAW(CONFIG_LIN1_BASE_ADDR + CSL_LIN_IODFTCTRL) & (0x3 << CSL_LIN_IODFTCTRL_PINSAMPLEMASK_SHIFT)),
                                        (LIN_PINMASK_CENTER_2SCLK << CSL_LIN_IODFTCTRL_PINSAMPLEMASK_SHIFT), "LIN_setPinSampleMask check failed.");
}
