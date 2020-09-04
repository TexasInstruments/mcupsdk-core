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

#include <string.h>
#include <kernel/dpl/CacheP.h>
#include "ti_drivers_open_close.h"

/*
 * In this example, we capture frames using the CSIRX interface in internal debug or test mode.
 * In this mode, there is no external sensor attached. The CSIRX HW has a internal test mode which is used
 * to construct and send a CSIRX frame to the CSIRX HW. The CSI HW captures the frame using its HW logic + DMA
 * and outputs to the frame to memory buffer that is specified.
 *
 * Here the CSIRX HW is setup using SysConfig. Two DMA contexts are created.
 * - Context 0, to switch DMA buffers when a complete frame is captured in the DMA buffer, also referred to as frame mode switching.
 * - Context 1, to switch DMA buffers when a 2 lines are captured in the DMA buffer, , also referred to as line mode switching.
 *
 * Line mode switching is used when the frame is large and cannot be buffered in the limited internal memory of the SOC.
 *
 * We show both the buffer switching examples in two different function calls in the example below.
 */

/* number of frames to capture */
#define APP_NUM_FRAMES_TO_CAPTURE  (10U)

/* max buffer size for ping and ping */
#define APP_FRAME_LINES_MAX    (4U)
#define APP_BYTES_PER_LINE_MAX (128U)
#define APP_BUF_SIZE_MAX       (APP_FRAME_LINES_MAX*APP_BYTES_PER_LINE_MAX)

/* ping pong buffer, MUST be placed in a memory accessible to both CPU and CSIRX HW */
uint8_t gAppBuf[2][APP_BUF_SIZE_MAX] __attribute__((aligned(64), section(".bss.dss_l3")));

/* structure to count interrupts and use it to wait for completion */
typedef struct {

    volatile uint32_t frameEndIntr;
    volatile uint32_t numLinesIntr;

} App_ContextIntrCount;

/* interrupt counter per context */
App_ContextIntrCount  gAppContextIntrCount[CSIRX_CONTEXTS_MAX];

/* clear interrupt counters */
void App_csirxClearIntrCount(void)
{
    uint16_t i;

    for(i=0; i<CSIRX_CONTEXTS_MAX; i++)
    {
        memset(&gAppContextIntrCount[i], 0, sizeof(App_ContextIntrCount) );
    }
}

/* interrupt callback, we simply count the interrupts in the callback */
void App_csirxCommonCallback(CSIRX_Handle handle, void *arg, struct CSIRX_CommonIntr_s *irq)
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
                gAppContextIntrCount[i].numLinesIntr++;
            }
            if(contextIntrStatus.isFrameEndCodeDetect)
            {
                gAppContextIntrCount[i].frameEndIntr++;
            }
        }
    }
}

/* clear capture buf so that we know we have really captured something */
void App_clearBuf(uint16_t bufId)
{
    if(bufId < 2)
    {
        memset(gAppBuf[bufId], 0, APP_BUF_SIZE_MAX);
        CacheP_wbInv(gAppBuf[bufId], APP_BUF_SIZE_MAX, CacheP_TYPE_ALL);
    }
}

/* capture frame with buffer switch at frame boundary
   - each iteration generates one frame of test data in debug mode
   - we check if we have received the correct number of interrupts
 */
void App_csirxFrameMode(void)
{
    uint32_t i;
    uint32_t contextId = 0;
    uint32_t numBytesPerLine = APP_BYTES_PER_LINE_MAX;
    uint32_t numLines = APP_FRAME_LINES_MAX;
    uint32_t numFrames = APP_NUM_FRAMES_TO_CAPTURE;
    CSIRX_Handle csirxHandle = gCsirxHandle[CONFIG_CSIRX0];
    CSIRX_ContextConfig *contextConfig = &gConfigCsirx0ContextConfig[contextId];

    App_clearBuf(0);
    App_clearBuf(1);

    CSIRX_contextSetPingPongAddress(csirxHandle, contextId, (uint32_t)gAppBuf[0], (uint32_t)gAppBuf[1] );
    CSIRX_contextEnable(csirxHandle, contextId);

    CSIRX_debugModeEnable(csirxHandle);

    /* generate and capture one frame of data, after each iteration we switch the buffer from ping to ping and so on */
    for(i=0; i < numFrames; i++)
    {
        App_csirxClearIntrCount();

        CSIRX_debugModeGenerateFrames(csirxHandle,
                    contextConfig->format,
                    contextConfig->virtualChannelId,
                    1,
                    numLines,
                    numBytesPerLine
                );

        /* wait for one frame to be captured */
        while( gAppContextIntrCount[contextId].frameEndIntr != 1 )
        {

        }

        /* Data is captured in buffer gAppBuf[i%2] */

        /* clear captured buffer */
        App_clearBuf(i%2);
    }

    CSIRX_debugModeDisable(csirxHandle);
    CSIRX_contextDisable(csirxHandle, contextId);
}

/* capture frame with buffer switch at 'n' line boundary
   - each iteration generates one frame of test data in debug mode
   - each buffer will receive 2 lines of data
   - we check if we have received the correct number of interrupts
 */
void App_csirxLineMode(void)
{
    uint32_t i;
    uint32_t contextId = 1;
    uint32_t numBytesPerLine = APP_BYTES_PER_LINE_MAX;
    uint32_t numLines = APP_FRAME_LINES_MAX;
    uint32_t numFrames = APP_NUM_FRAMES_TO_CAPTURE;
    CSIRX_Handle csirxHandle = gCsirxHandle[CONFIG_CSIRX0];
    CSIRX_ContextConfig *contextConfig = &gConfigCsirx0ContextConfig[contextId];

    App_clearBuf(0);
    App_clearBuf(1);

    CSIRX_contextSetPingPongAddress(csirxHandle, contextId, (uint32_t)gAppBuf[0], (uint32_t)gAppBuf[1] );
    CSIRX_contextEnable(csirxHandle, contextId);

    CSIRX_debugModeEnable(csirxHandle);

    /* generate one frame of data, here the buffer will switch from ping to pong after half the number of lines
       in a frame are captured */
    for(i=0; i < numFrames; i++)
    {
        App_csirxClearIntrCount();

        CSIRX_debugModeGenerateFrames(csirxHandle,
                    contextConfig->format,
                    contextConfig->virtualChannelId,
                    1,
                    numLines,
                    numBytesPerLine
                );

        while( gAppContextIntrCount[contextId].numLinesIntr != 2 )
        {

        }

        /* gAppBuf[0] and gAppBuf[1] contain captured data */

        /* clear captured buffer */
        App_clearBuf(0);
        App_clearBuf(1);
    }

    CSIRX_debugModeDisable(csirxHandle);
    CSIRX_contextDisable(csirxHandle, contextId);
}

void csirx_internal_capture_main(void *args)
{
    /* Open all drivers including CSIRX driver, which is setup via SysConfig */
    Drivers_open();

    DebugP_log("Frame switching mode capture ... starting\r\n");
    App_csirxFrameMode();
    DebugP_log("Frame switching mode capture ... done !!!\r\n");

    DebugP_log("Line switching mode capture ... starting\r\n");
    App_csirxLineMode();
    DebugP_log("Line switching mode capture ... done !!!\r\n");

    DebugP_log("All tests have passed !!!\r\n");

    /* close all drivers including CSIRX */
    Drivers_close();
}

