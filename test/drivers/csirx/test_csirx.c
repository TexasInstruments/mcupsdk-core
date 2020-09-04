/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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

#include <unity.h>
#include <string.h>
#include <kernel/dpl/CacheP.h>
#include "ti_drivers_open_close.h"

/* number of frames to capture */
#define TEST_NUM_FRAMES_TO_CAPTURE  (10U)

/* max buffer size for ping and ping */
#define TEST_FRAME_LINES_MAX    (4U)
#define TEST_BYTES_PER_LINE_MAX (128U)
#define TEST_BUF_SIZE_MAX       (TEST_FRAME_LINES_MAX*TEST_BYTES_PER_LINE_MAX)

/* ping pong buffer, MUST be placed in a memory accessible to both CPU and CSIRX HW */
uint8_t gTestBuf[2][TEST_BUF_SIZE_MAX] __attribute__((aligned(64), section(".bss.dss_l3")));

/* structure to count interrupts and use it to wait for completion */
typedef struct {

    volatile uint32_t frameEndIntr;
    volatile uint32_t numLinesIntr;

} TEST_ContextIntrCount;

/* interrupt counter per context */
TEST_ContextIntrCount  gTestContextIntrCount[CSIRX_CONTEXTS_MAX];

/* clear interrupt counters */
void test_csirxClearIntrCount()
{
    uint16_t i;

    for(i=0; i<CSIRX_CONTEXTS_MAX; i++)
    {
        memset(&gTestContextIntrCount[i], 0, sizeof(TEST_ContextIntrCount) );
    }
}

/* interrupt callback, we simply count the interrupts in the callback */
void test_csirxCommonCallback(CSIRX_Handle handle, void *arg, struct CSIRX_CommonIntr_s *irq)
{
    uint16_t i;
    CSIRX_ContextIntr contextIntrStatus;

    for(i=0; i<CSIRX_CONTEXTS_MAX; i++)
    {
        if(irq->isContextIntr[i])
        {
            CSIRX_contextGetPendingIntr(handle, i, &contextIntrStatus);
            CSIRX_contextClearAllIntr(handle, i);

            if(contextIntrStatus.isNumLines)
            {
                gTestContextIntrCount[i].numLinesIntr++;
            }
            if(contextIntrStatus.isFrameEndCodeDetect)
            {
                gTestContextIntrCount[i].frameEndIntr++;
            }
        }
    }
}

/* clear capture buf so that we know we have really captured something */
void test_clearBuf(uint16_t bufId)
{
    if(bufId < 2)
    {
        memset(gTestBuf[bufId], 0, TEST_BUF_SIZE_MAX);
        CacheP_wbInv(gTestBuf[bufId], TEST_BUF_SIZE_MAX, CacheP_TYPE_ALL);
    }
}

/* capture frame with buffer switch at frame boundary
   - each iteration generates one frame of test data in debug mode
   - we check if we have received the correct amount of data
   - we check if we have received the correct number of interrupts
 */
void test_csirxFrameMode(void *args)
{
    uint32_t i, j;
    uint32_t contextId = 0;
    uint32_t numBytesPerLine = TEST_BYTES_PER_LINE_MAX;
    uint32_t numLines = TEST_FRAME_LINES_MAX;
    uint32_t numFrames = TEST_NUM_FRAMES_TO_CAPTURE;
    CSIRX_Handle csirxHandle = gCsirxHandle[CONFIG_CSIRX0];
    CSIRX_ContextConfig *contextConfig = &gConfigCsirx0ContextConfig[contextId];

    test_clearBuf(0);
    test_clearBuf(1);

    CSIRX_contextSetPingPongAddress(csirxHandle, contextId, (uint32_t)gTestBuf[0], (uint32_t)gTestBuf[1] );
    CSIRX_contextEnable(csirxHandle, contextId);

    CSIRX_debugModeEnable(csirxHandle);

    /* generate and capture one frame of data, after each iteration we switch the buffer from ping to ping and so on */
    for(i=0; i < numFrames; i++)
    {
        test_csirxClearIntrCount();

        CSIRX_debugModeGenerateFrames(csirxHandle,
                    contextConfig->format,
                    contextConfig->virtualChannelId,
                    1,
                    numLines,
                    numBytesPerLine
                );

        /* wait for one frame to be captured */
        while( gTestContextIntrCount[contextId].frameEndIntr != 1 )
        {

        }

        for(j=0; j<numLines; j++)
        {
            /* check 1st pixel of a line is non-zero */
            TEST_ASSERT_NOT_EQUAL_UINT8( 0, gTestBuf[i%2] [numBytesPerLine*(j)] );
            /* check last pixel of a line is non-zero */
            TEST_ASSERT_NOT_EQUAL_UINT8( 0, gTestBuf[i%2] [numBytesPerLine*(j) + numBytesPerLine - 1] );
        }

        /* clear captured buffer */
        test_clearBuf(i%2);
    }

    CSIRX_debugModeDisable(csirxHandle);
    CSIRX_contextDisable(csirxHandle, contextId);
}

/* capture frame with buffer switch at 'n' line boundary
   - each iteration generates one frame of test data in debug mode
   - we check if we have received the correct amount of data in both ping and ping buffer
   - each buffer will received n/2 lines of data
   - we check if we have received the correct number of interrupts
 */
void test_csirxLineMode(void *args)
{
    uint32_t i, j;
    uint32_t contextId = 1;
    uint32_t numBytesPerLine = TEST_BYTES_PER_LINE_MAX;
    uint32_t numLines = TEST_FRAME_LINES_MAX;
    uint32_t numFrames = TEST_NUM_FRAMES_TO_CAPTURE;
    CSIRX_Handle csirxHandle = gCsirxHandle[CONFIG_CSIRX0];
    CSIRX_ContextConfig *contextConfig = &gConfigCsirx0ContextConfig[contextId];

    test_clearBuf(0);
    test_clearBuf(1);

    CSIRX_contextSetPingPongAddress(csirxHandle, contextId, (uint32_t)gTestBuf[0], (uint32_t)gTestBuf[1] );
    CSIRX_contextEnable(csirxHandle, contextId);

    CSIRX_debugModeEnable(csirxHandle);

    /* generate one frame of data, here the buffer will switch from ping to pong after half the number of lines
       in a frame are captured */
    for(i=0; i < numFrames; i++)
    {
        test_csirxClearIntrCount();

        CSIRX_debugModeGenerateFrames(csirxHandle,
                    contextConfig->format,
                    contextConfig->virtualChannelId,
                    1,
                    numLines,
                    numBytesPerLine
                );

        while( gTestContextIntrCount[contextId].numLinesIntr != 2 )
        {

        }

        for(j=0; j<numLines/2; j++)
        {
            /* check 1st pixel of a line is non-zero */
            TEST_ASSERT_NOT_EQUAL_UINT8( 0, gTestBuf[0] [numBytesPerLine*(j)] );
            /* check last pixel of a line is non-zero */
            TEST_ASSERT_NOT_EQUAL_UINT8( 0, gTestBuf[0] [numBytesPerLine*(j) + numBytesPerLine - 1] );

            /* check 1st pixel of a line is non-zero */
            TEST_ASSERT_NOT_EQUAL_UINT8( 0, gTestBuf[1] [numBytesPerLine*(j)] );
            /* check last pixel of a line is non-zero */
            TEST_ASSERT_NOT_EQUAL_UINT8( 0, gTestBuf[1] [numBytesPerLine*(j) + numBytesPerLine - 1] );
        }

        /* clear captured buffer */
        test_clearBuf(0);
        test_clearBuf(1);
    }

    CSIRX_debugModeDisable(csirxHandle);
    CSIRX_contextDisable(csirxHandle, contextId);
}

void test_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();

    UNITY_BEGIN();

    RUN_TEST(test_csirxFrameMode, 1880, NULL);
    RUN_TEST(test_csirxLineMode, 1881, NULL);

    UNITY_END();

    Drivers_close();
}

void setUp(void)
{
}

void tearDown(void)
{
}
