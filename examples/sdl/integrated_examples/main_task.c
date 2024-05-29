/*
 *  Copyright (C) 2023-2024 Texas Instruments Incorporated
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

#include <stdlib.h>
#include <dpl_interface.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>

/* platform and drivers headers */
#include "ti_drivers_config.h"
#include "ti_board_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Freertos headers */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/* our example headers */
#include "example.h"
#include "sdlexample.h"

/* State of the example - see example.h for values */
uint32_t exampleState;

/* stats for SDL */
volatile struct s_sdlstats  sdlstats;

/* handle for mcan task */
extern TaskHandle_t gMcanTask;

/* A software timer used to schedule periodic diagnsotics */
static TimerHandle_t xDiagTimer;

/* used to denote whether this is a normal shutdown or due to error */
static bool bNormalShutdown = true;

/*
** void diagTimerCallback( TimerHandle_t xTimer )
**
** This callback is made by FreeRTOS whne our diagnostics timer expires. It is periodic so
** we do not need to reset it.
*/
void diagTimerCallback( TimerHandle_t xTimer )
 {
    /* signal main task we need to run diagnostics */
     sdlstats.runDiagnostics = true;
     return;
 }

/*
** void main_task(void *args)
**
** This is our task in the example and controls execution flow. It also
**  monitors for SDL.
*/
void main_task(void *args)
{
  int32_t   sdlResult;

  /* this is set to true by the periodic diagnostics timer */
  sdlstats.runDiagnostics = false;

  while (FOREVER)
  {
    /* Process based on state */
    switch (exampleState)
    {
      case  EXAMPLE_STATE_FREERUN:
            /* check for an ESM event */
            if (sdlstats.esmcb != ESMCB_NONE)
            {
              /* ESM event ocurred */
              DebugP_log("[MAIN] ESM Event: ESMCB %d, instType %d, intType %d,  grpChannel %d, index %d, intSrc %d \r\n",
                          sdlstats.esmcb, sdlstats.esm.esmInst, sdlstats.esm.esmIntrType, sdlstats.esm.grpChannel, sdlstats.esm.index, sdlstats.esm.intSrc);
              exampleState = EXAMPLE_STATE_BLOCK;
            }
            /* Check to run diagnostics - signalled by diag timer */
            else
            if (sdlstats.runDiagnostics)
            {
              /*  time to run dagnostics */
              exampleState = EXAMPLE_STATE_RUNDIAGS;
            }
            /* check to see if we have sent enough data  and then shutdown if yes */
            else
            if (mcanstats.txcount >= EXAMPLE_MAX_DATA_TO_SEND)
            {
              exampleState = EXAMPLE_STATE_SHUTDOWN;
            }
            else
            {
              /* give tasks a chance to run */
              vTaskDelay(10);
            }
            break;

    case  EXAMPLE_STATE_RUNDIAGS:
          /* we will perform diagnostics  */
          DebugP_log("[MAIN] Running diagnostics... \r\n");
          sdlResult = performDiags(NULL);
          if (sdlResult == SDL_PASS)
          {
              /* we are good to run */
              DebugP_log("[MAIN] Diagnostics Complete. \r\n");
              exampleState = EXAMPLE_STATE_FREERUN;
          }
          else
          {
              /* failed, transition to blocked */
              exampleState = EXAMPLE_STATE_BLOCK;
          }
          /* Clear diagnostics flag pass or fail */
          sdlstats.runDiagnostics = false;
          break;

      case  EXAMPLE_STATE_STARTUP_DONE:
            /* Open drivers to open the UART driver for console */
            Drivers_open();
            Board_driversOpen();
            DebugP_log("[MAIN] Start up diagnostics (STC,PBIST,CCM) passed. \r\n");
            /* configure MCAN interface and set in loopback */
            DebugP_log("[MAIN] MCAN set to Loopback Mode.\r\n");
            McanConfigureDriver(NULL);
            /* start a periodic timer to run safety diagnostics */
            xDiagTimer = xTimerCreate
                                (
                                /* Just a text name, not used by the RTOS kernel. */
                                "DiagTimer",
                                /* The timer period in ticks */
                                DIAG_TIMER_VALUE,
                                /* timer will re-load when it expires */
                                pdTRUE,
                                /* The ID is used to store a count of the
                                   number of times the timer has expired, which
                                  is initialised to 0. */
                                ( void * ) 0,
                                /* our callback for hwen the timer expires */
                                diagTimerCallback
                                );
            if (xDiagTimer != NULL)
            {
              if (xTimerStart(xDiagTimer, 500) == pdPASS)
              {
                  DebugP_log("[MAIN] Diagnostic check timer set to %d ticks.\r\n",DIAG_TIMER_VALUE);
              }
              else
              {
                  DebugP_log("[MAIN] Could not start diag timer.\r\n");
              }
            }
            else
            {
                DebugP_log("[MAIN] Could not create diag timer.\r\n");
            }

            /* set the system to run */
            DebugP_log("[MAIN] System set to FREERUN.\r\n");
            DebugP_log("[MAIN] MCAN task priority = %d \r\n", MCAN_TASK_PRI);
            DebugP_log("[MAIN] MAIN task priority = %d \r\n", MAIN_TASK_PRI);
            exampleState = EXAMPLE_STATE_FREERUN;
            break;

      case  EXAMPLE_STATE_BLOCK:
            DebugP_log("[MAIN] Example entered BLOCKED state based on a safety check.\r\n");
            /* handle the ESM event based on what it is. In this example we block and then shutdown */
            if (sdlstats.esm.intSrc == ESM_INT_MCAN0_ECC_CORRECTABLE)
            {
              sdlstats.eccCorrectedCount++;
            }
            /* toggle safety LED */
            SDL_ESM_setNError(SDL_ESM_INST_MAIN_ESM0);
            /* we could decide to do some actions here other than shutting down */
            bNormalShutdown = false;
            exampleState = EXAMPLE_STATE_SHUTDOWN;
            break;

      case  EXAMPLE_STATE_SHUTDOWN:
            /* This example is done */
            DebugP_log("[MAIN] Shutting down. \r\n");
            DebugP_log("[MAIN] Example is done.  \r\n");
            if (bNormalShutdown)
            {
              DebugP_log("All tests have passed \r\n");
            }
            else
            {
              DebugP_log("Some tests have failed \r\n");
            }
            /* delete FreeRTOS tasks and resources */
            xTimerStop(xDiagTimer,0);
            vTaskDelete(gMcanTask);
            /* delay for uart prints */
            for (sdlResult = 0; sdlResult < 120000; sdlResult++);
            Board_driversClose();
            Drivers_close();
            Board_deinit();
            System_deinit();
            while (FOREVER);
            break;

      default:
            /* should not happen, act defensively */
            DebugP_log("[MAIN] Unknown state = %d. We will block. \r\n", exampleState);
            exampleState = EXAMPLE_STATE_BLOCK;
            break;
    }
  }
}
