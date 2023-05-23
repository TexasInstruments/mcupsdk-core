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
#include <stdlib.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/nortos/dpl/c66/HwiP_c66.h>
#include <kernel/safertos/dpl/c66/HwiP_safertos.h>
#include "ti_drivers_config.h"
#include "ti_board_config.h"
#include "SafeRTOS_API.h"


/* SafeRTOS inspects the vector table to ensure
 * the necessary handlers have been installed. */
#define configSTACK_CHECK_MARGIN            ( 0U )
#define configSYSTEM_STACK_SIZE             ( 0x2000U )

/* timer related config */
#define configTIMER_TASK_PRIORITY           ( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH            ( 16 )

/* The user configuration for the idle task. */
#define configIDLE_TASK_STACK_SIZE          ( configMINIMAL_STACK_SIZE )

/* The user configuration for the timer task. */
#define configTIMER_TASK_STACK_SIZE          ( configMINIMAL_STACK_SIZE )

/* The user configuration for the timer module. */
#define configTIMER_CMD_QUEUE_BUFFER_SIZE   ( ( configTIMER_QUEUE_LENGTH * sizeof( timerQueueMessageType ) ) + safertosapiQUEUE_OVERHEAD_BYTES )

static portInt8Type acIdleTaskStack[ configIDLE_TASK_STACK_SIZE ] = { 0 };

/* Declare the stack for the timer task, it cannot be done in the timer
 * module as the syntax for alignment is port specific. Also the callback
 * functions are executed in the timer task and their complexity/stack
 * requirements are application specific. */
static portInt8Type acTimerTaskStack[ configTIMER_TASK_STACK_SIZE ] = { 0 };

/* The buffer for the timer command queue. */
static portInt8Type acTimerCommandQueueBuffer[ configTIMER_CMD_QUEUE_BUFFER_SIZE ] = { 0 };

/* The structure passed to xTaskInitializeScheduler() to configure the kernel
 * with the application defined constants and call back functions. */
xPORT_INIT_PARAMETERS gSafertosPortInit =
{
    configSYSTICK_CLOCK_HZ,                 /* ulTimerClockHz */
    configTICK_RATE_HZ,                     /* ulTickRateHz */

    /* Hook Functions */
    ( void * ) NULL,  /* pvSvcHookFunction */

    /* System Stack parameters */
    configSTACK_CHECK_MARGIN,               /* uxAdditionalStackCheckMarginBytes */

    /* Idle Task parameters */
    acIdleTaskStack,                        /* pcIdleTaskStackBuffer */
    configIDLE_TASK_STACK_SIZE,             /* uxIdleTaskStackSizeBytes */
    safertosapiUNPRIVILEGED_TASK,           /* The idle hook will not be executed in privileged mode. */
    NULL,                                   /* pvIdleTaskTLSObject */

    /* Timer feature initialisation parameters */
    configTIMER_TASK_PRIORITY,              /* uxTimerTaskPriority */
    configTIMER_TASK_STACK_SIZE,            /* uxTimerTaskStackSize */
    acTimerTaskStack,                       /* pcTimerTaskStackBuffer */
    configTIMER_QUEUE_LENGTH,               /* uxTimerCommandQueueLength */
    configTIMER_CMD_QUEUE_BUFFER_SIZE,      /* uxTimerCommandQueueBufferSize */
    acTimerCommandQueueBuffer,              /* pcTimerCommandQueueBuffer */
};

#define MAIN_TASK_PRI  (configMAX_PRIORITIES-2)

#define MAIN_TASK_SIZE (16384U)
portInt8Type gMainTaskStack[MAIN_TASK_SIZE] __attribute__((aligned(32)));

xTCB gMainTaskObj;
portTaskHandleType gMainTask;

void hello_world_main(void *args);

void safertos_main(void *args)
{
    hello_world_main(NULL);

    xTaskDelete(NULL);
}

static void SafeRTOS_init( void )
{
    portBaseType xInitSchedResult;

    memset(acIdleTaskStack, 0xFF, sizeof(acIdleTaskStack));
    memset(acTimerTaskStack, 0xFF, sizeof(acTimerTaskStack));
    memset(acTimerCommandQueueBuffer, 0xFF, sizeof(acTimerCommandQueueBuffer));

    /* Initialise the kernel by passing in a pointer to the xPORT_INIT_PARAMETERS structure
     * and return the resulting error code. */
    xInitSchedResult = xTaskInitializeScheduler( &gSafertosPortInit );
    DebugP_assertNoLog((xInitSchedResult == pdPASS));

    /* Register Privilege handler hook. */
    HwiP_registerRaisePrivilegeHandlerHook(&HwiP_portRaisePrivilege);

    /* Register restore Privilege handler hook. */
    HwiP_registerRestorePrivilegeHandlerHook(&HwiP_portRestorePrivilege);

    return;
}

extern void DebugP_logZoneInit(void);

int main(void)
{
    xTaskParameters xTaskPParams;
    portBaseType xCreateResult;

    /* init SOC specific modules */
    SafeRTOS_init();
    DebugP_logZoneInit();   /* Create log semaphore ahead of all module init */
    System_init();
    Board_init();

    xTaskPParams.pvTaskCode         = safertos_main;
    xTaskPParams.pcTaskName         = "safertos_main";
    xTaskPParams.pxTCB              = &gMainTaskObj;
    xTaskPParams.pcStackBuffer      = (portInt8Type *) gMainTaskStack;
    xTaskPParams.uxStackDepthBytes  = MAIN_TASK_SIZE;
    xTaskPParams.pvParameters       = NULL;
    xTaskPParams.uxPriority         = MAIN_TASK_PRI;
    xTaskPParams.pvObject           = NULL;  /* Thread Local Storage not used. */

    /* This task is created at highest priority, it should create more tasks and then delete itself */
    xCreateResult = xTaskCreate(&xTaskPParams, &gMainTask);
    DebugP_assert(xCreateResult == pdPASS);
    DebugP_assert(gMainTask != NULL);

    /* Start the scheduler to start the tasks executing. */
    xTaskStartScheduler();

    /* The following line should never be reached because vTaskStartScheduler()
    will only return if there was not enough SafeRTOS heap memory available to
    create the Idle and (if configured) Timer tasks.  Heap management, and
    techniques for trapping heap exhaustion, are described in the book text. */
    DebugP_assertNoLog(0);

    return 0;
}
