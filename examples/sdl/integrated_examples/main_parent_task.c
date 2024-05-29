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
#include "ti_drivers_config.h"
#include "ti_board_config.h"
#include "FreeRTOS.h"
#include "task.h"
/* for checking STC completion */
#include <sdl/sdl_stc.h>
#include "example.h"
#include "sdlexample.h"


#define MAIN_TASK_SIZE (16384U/sizeof(configSTACK_DEPTH_TYPE))
StackType_t gMcanTaskStack[MAIN_TASK_SIZE] __attribute__((aligned(32)));
StackType_t gMainTaskStack[MAIN_TASK_SIZE] __attribute__((aligned(32)));

/* MCAN driver task */
StaticTask_t gMcanTaskObj;
TaskHandle_t gMcanTask;

/* Main Task */
StaticTask_t gMainTaskObj;
TaskHandle_t gMainTask;

int main_parent(void)
{

  /* -------------------------------------------------------------------------------------*/
  /*  We cannot use the Uart for logging until FreeRTOS starts                            */
  /*  (therefore the start up diagnostics will not print anything)                        */
  /* -------------------------------------------------------------------------------------*/
  exampleState = EXAMPLE_STATE_STARTUP;

  /* -------------------------------------------------------------------------------------*/
  /*  Run start up diagnostics with SDL                                                   */
  /* -------------------------------------------------------------------------------------*/

  /* initialize for diagnsotics mode */
  sdlstats.startupErrCode       = STARTUPDIAGS_OK;
  sdlstats.esmcb                = ESMCB_NONE;
  sdlstats.stcResult[0]         = SDL_EFAIL;
  sdlstats.stcResult[1]         = SDL_EFAIL;
  sdlstats.eccCorrectedCount    = 0;
  sdlstats.esmcb                = ESMCB_NONE;
  sdlstats.runDiagnostics       = true;
  sdlstats.diagRunning          = DIAGRUNNING_NONE;

  /* Configure DPL */
  if (SDL_EXAMPLE_dplInit() != SDL_PASS)
  {
    /* failed */
    sdlstats.startupErrCode |= STARTUPDIAGS_DPLERR;
  }

  /* Configure ESM  for when we run the system and set up our callbacks */
  if (ESM_init () != SDL_PASS)
  {
    sdlstats.startupErrCode |= STARTUPDIAGS_ESMERR;
  }

  /* Run self Test Check */
  STC_run(NULL);
  if ( (sdlstats.stcResult[0] != SDL_STC_COMPLETED_SUCCESS) || (sdlstats.stcResult[1] != SDL_STC_COMPLETED_SUCCESS) )
  {
    sdlstats.startupErrCode |= STARTUPDIAGS_STCERR;
  }

  /* Run PBIST on Core 0 */
  if (pbist_run(NULL) != SDL_PASS)
  {
    /* failed */
    sdlstats.startupErrCode |= STARTUPDIAGS_PBISTERR;
  }

  /* Run CCM test on both R51 & R50*/
  if (CCM_test(NULL) != SDL_PASS)
  {
    /* failed */
    sdlstats.startupErrCode |= STARTUPDIAGS_CCMERR;
  }

  /* if one or more start up diags failed dont load */
  if (sdlstats.startupErrCode != STARTUPDIAGS_OK)
  {
    /* toggle safety LED */
    SDL_ESM_setNError(SDL_ESM_INST_MAIN_ESM0);
    while (FOREVER);
  }

  /* clear diagnostics mode */
  sdlstats.runDiagnostics = false;
  sdlstats.esmcb          = ESMCB_NONE;


  /* -------------------------------------------------------------------------------------*/
  /*  Create our FreeRTOS tasks                                                           */
  /* -------------------------------------------------------------------------------------*/

  /* Our MCAN polling task */
  gMcanTask = xTaskCreateStatic( mcan_task,      /* Pointer to the function that implements the task. */
                                "mcan_task",     /* Text name for the task.  This is to facilitate debugging only. */
                                MAIN_TASK_SIZE,  /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                NULL,            /* We are not using the task parameter. */
                                MCAN_TASK_PRI,   /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                gMcanTaskStack,  /* pointer to stack base */
                                &gMcanTaskObj ); /* pointer to statically allocated task object memory */

 if (gMcanTask == NULL)
  {
    /* task failed to start - toggle LED. Ideally this never should fail as the resources should be allocated  */
    SDL_ESM_setNError(SDL_ESM_INST_MAIN_ESM0);
    while (FOREVER);
  }

  /* Main task */
  gMainTask = xTaskCreateStatic( main_task,        /* Pointer to the function that implements the task. */
                                "main_task",      /* Text name for the task.  This is to facilitate debugging only. */
                                MAIN_TASK_SIZE,   /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                NULL,             /* We are not using the task parameter. */
                                MAIN_TASK_PRI,    /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                gMainTaskStack,   /* pointer to stack base */
                                &gMainTaskObj );  /* pointer to statically allocated task object memory */
  if (gMainTask == NULL)
  {
    /* task failed to start - toggle LED. Ideally this should never fail as the resources should be allocated  */
    vTaskDelete(gMcanTask);
    SDL_ESM_setNError(SDL_ESM_INST_MAIN_ESM0);
    while (FOREVER);
  }

  /* -------------------------------------------------------------------------------------*/
  /*  Start the example running                                                           */
  /* -------------------------------------------------------------------------------------*/

  /* start up is complete */
  exampleState = EXAMPLE_STATE_STARTUP_DONE;

  /* Start the scheduler to start the tasks executing. */
  vTaskStartScheduler();

  /* The following line should never be reached because vTaskStartScheduler()
  will only return if there was not enough FreeRTOS heap memory available to
  create the Idle and (if configured) Timer tasks.  Heap management, and
  techniques for trapping heap exhaustion, are described in the book text. */
  while (FOREVER);
}
