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
 */

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#define GPTIMER_EXAMPLE_LOOP_WAIT_TIME  100000U
#define GPTIMER_MATCH_DETECTION_COUNT   10U

uint32_t matchCount = 0U;
uint32_t updateFlag = 0;

/* Compare Match Callback Function */
void CompareMatchCallbackUser(GPTIMER_Handle handle)
{
    /* Counter match happened with Compare Value */
    matchCount++;
    updateFlag++;

    /* Set Counter Value Back to 0 */
    GPTIMER_setCount(handle, 0U);
}

void gp_timer_compare_match_callback_main(void *args)
{
    Drivers_open();
    Board_driversOpen();

    DebugP_log("GP Timer Compare Match Test Started ...\r\n");

    /* Start the Timer */
    GPTIMER_start(gGpTimerHandle[0]);

    while(TRUE) {
        /* Wait for 100ms*/
        ClockP_usleep(GPTIMER_EXAMPLE_LOOP_WAIT_TIME);

        /* Check Update Flag for match count update detection */
        if(updateFlag != 0U) {
            DebugP_log("Compare Match Flag set, matchCount: %d \r\n", matchCount);
            /* Set Update Flag to 0 */
            updateFlag = 0U;
            /* Exit Loop after 10 detections */
            if(matchCount == GPTIMER_MATCH_DETECTION_COUNT) {
                break;
            }
        }
    }

    /* Stop the Counter */
    GPTIMER_stop(gGpTimerHandle[0]);

    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}
