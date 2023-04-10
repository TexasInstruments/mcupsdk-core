/*!
* \file main.c
*
* \brief
* IO-Link Master main/init funtions
*
* \author
* KUNBUS GmbH
*
* \date
* 2021-05-19
*
* \copyright
* Copyright (c) 2021, KUNBUS GmbH<br /><br />
* All rights reserved.<br />
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:<br />
* <ol>
* <li>Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.</li>
* <li>Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.</li>
* <li>Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.</li>
* </ol>
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <osal.h>
#include <hwal.h>
#include <pru.h>
#include <pru_IOLink.h>

#include <stdio.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/SystemP.h>

#include "FreeRTOS.h"
#include "task.h"

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_config.h"
#include "ti_board_open_close.h"

#include <IOLM_SMI.h>
#include <IOLM_Sitara_Version.h>
#include "IOLM_Port_Utils.h"
#include "IOLM_Phy.h"
#include "IOLinkPort/IOLM_Port_Sitara_soc.h"
#include "IOLinkPort/IOLM_Port_LEDTask.h"
#include "IOLM_Port_smiExample.h"

#define IOLM_MAIN_PRU_INSTANCE              (0)

#define IOLM_MAIN_TASK_SIZE_DIVIDER         sizeof(configSTACK_DEPTH_TYPE)
#define IOLM_MAIN_TASK_STACK_TYPE           StackType_t

#define IOLM_MAIN_STARTUP_TASK_SIZE         (0x4000U / IOLM_MAIN_TASK_SIZE_DIVIDER)
#define IOLM_MAIN_LOOP_TASK_SIZE            (0x2000U / IOLM_MAIN_TASK_SIZE_DIVIDER)
#define IOLM_MAIN_LED_TASK_SIZE             (0x2000U / IOLM_MAIN_TASK_SIZE_DIVIDER)
#define IOLM_EXAMPLE_TASK_SIZE              (0x2000U / IOLM_MAIN_TASK_SIZE_DIVIDER)

static IOLM_MAIN_TASK_STACK_TYPE    IOLM_startupTaskStack_s[IOLM_MAIN_STARTUP_TASK_SIZE] \
                                        __attribute__((aligned(32), section(".threadstack"))) = {0};
static IOLM_MAIN_TASK_STACK_TYPE    IOLM_mainTaskStack_s[IOLM_MAIN_LOOP_TASK_SIZE] \
                                        __attribute__((aligned(32), section(".threadstack"))) = {0};
static IOLM_MAIN_TASK_STACK_TYPE    IOLM_ledTaskStack_s[IOLM_MAIN_LED_TASK_SIZE] \
                                        __attribute__((aligned(32), section(".threadstack"))) = {0};
static IOLM_MAIN_TASK_STACK_TYPE    IOLM_exampleTaskStack_s[IOLM_EXAMPLE_TASK_SIZE] \
                                        __attribute__((aligned(32), section(".threadstack"))) = {0};

static void* IOLM_pMainTaskHandle_s;
static void* PRU_IOL_pLedTaskHandle_s;
static void* IOLM_pExampleTaskHandle_s;

OSAL_SCHED_SSignalHandle_t* pMainLoopRequested_g = NULL;

/*!
 *  \brief
 *  Initialization of FreeRTOS and SysCfg board init.
 */
static void IOLM_MAIN_sysInit()
{
    System_init();
    Board_init();
}

/*!
 *  \brief
 *  Initialization of board specific hardware.
 *
 *  \return     error                           as uint32_t
 *  \retval     #OSAL_NO_ERROR                  Success.
 *  \retval     #else                           Something went wrong.
 *
 */
uint32_t IOLM_MAIN_boardInit()
{
    static const IOLM_SPhyGeneric*          pPhyStackCallbacks[IOLM_PORT_COUNT];
    uint8_t     port;
    uint32_t    error = OSAL_eERR_NOERROR;

    Drivers_open();
    Board_driversOpen();

    error = HWAL_init();
    if (error != OSAL_eERR_NOERROR)
    {
        goto laExit;
    }

    /* Init lower levels */

    for (port = 0; port < IOLM_PORT_COUNT; port++)
    {
        pPhyStackCallbacks[port]     = (IOLM_SPhyGeneric *) &IOLM_SOC_phyPortCfgPru_g[port];
    }
    
    IOLM_SPI_init();
    IOLM_SOC_init();
    PRU_IOLM_init(IOLM_MAIN_PRU_INSTANCE);

    IOLM_Phy_Init(pPhyStackCallbacks);

laExit:
    return error;
}

/*!
 *  \brief
 *  Task for the main example loop.
 */
void IOLM_MAIN_exampleStart()
{
    IOLM_pExampleTaskHandle_s = OSAL_SCHED_startTask((OSAL_SCHED_CBTask_t)IOLM_EXMPL_mainLoop
                                                     ,NULL
                                                     ,OSAL_TASK_ePRIO_19
                                                     ,(uint8_t*)IOLM_exampleTaskStack_s
                                                     ,sizeof(IOLM_exampleTaskStack_s)
                                                     ,OSAL_OS_START_TASK_FLG_NONE
                                                     ,"Example Task");

    if (NULL == IOLM_pExampleTaskHandle_s)
    {
        OSAL_error(__FILE__, __LINE__, OSAL_eERR_INVALIDSTATE, true, 1,
                   "Creating Example Task failed.\r\n");
    }
}

/*!
 *  \brief
 *  This callback function is called by the stack requires a mainloop run.
 *  This is used in combination with operating systems
 */
void IOLM_MAIN_cbMainLoopRequest(void)
{
    OSAL_postSignal(pMainLoopRequested_g);
}

/*!
 *  \brief
 *  Task for the main application loop.
 */
void OSAL_FUNC_NORETURN IOLM_MAIN_loop()
{
    while (1)
    {
        IOLM_SMI_vRun();
        OSAL_waitSignal(pMainLoopRequested_g, 2);
    }
}

/*!
 *  \brief
 *  Initialize stack and demo application.
 */
void IOLM_MAIN_init()
{
    uint8_t portNumber;

    pMainLoopRequested_g = OSAL_createSignal("IOLM_mainLoopRequest");

    /* IO Link Master stack and example init */
    IOLM_EXMPL_init();

    for (portNumber = 0; portNumber < IOLM_PORT_COUNT; portNumber++)
    {
        IOLM_SOC_setPower(IOLM_MAIN_PRU_INSTANCE, portNumber, true);
    }

    /* Create a task for the IO-Link main execution */
    IOLM_MAIN_exampleStart();
}

/*!
 *  \brief
 *  Startup task to init board and create following tasks.
 */
void IOLM_MAIN_startupTask()
{
    uint32_t    error = OSAL_eERR_NOERROR;

    error = IOLM_MAIN_boardInit();

    OSAL_printf("IO-Link Sitara example application version: %s\n", IOLM_LIB_getVersion());

    if (error == OSAL_eERR_NOERROR)
    {

        /* Create a task for the IO-Link status LED management */
        PRU_IOL_pLedTaskHandle_s = OSAL_SCHED_startTask((OSAL_SCHED_CBTask_t)IOLM_LED_switchingTask
                                                        ,NULL
                                                        ,OSAL_TASK_ePRIO_IOL_LED
                                                        ,(uint8_t*)IOLM_ledTaskStack_s
                                                        ,sizeof(IOLM_ledTaskStack_s)
                                                        ,OSAL_OS_START_TASK_FLG_NONE
                                                        ,"LED Task");
        if (NULL == PRU_IOL_pLedTaskHandle_s)
        {
            OSAL_error(__FILE__, __LINE__, OSAL_eERR_INVALIDSTATE, true, 1,
                       "Creating LED Task failed.\r\n");
        }
        IOLM_MAIN_init();

        /* Create a task for the IO-Link main execution */
        IOLM_pMainTaskHandle_s = OSAL_SCHED_startTask((OSAL_SCHED_CBTask_t)IOLM_MAIN_loop
                                                     ,NULL
                                                     ,OSAL_TASK_ePRIO_IOL_Main
                                                     ,(uint8_t*)IOLM_mainTaskStack_s
                                                     ,sizeof(IOLM_mainTaskStack_s)
                                                     ,OSAL_OS_START_TASK_FLG_NONE
                                                     ,"Main Task");

        if (NULL == IOLM_pMainTaskHandle_s)
        {
            OSAL_error(__FILE__, __LINE__, OSAL_eERR_INVALIDSTATE, true, 1,
                       "Creating Main Task failed.\r\n");
        }

        for (;;)
        {
            OSAL_SCHED_sleep(10000);
        }
    }
    else
    {
        OSAL_error(__FILE__, __LINE__, OSAL_eERR_INVALIDSTATE, true, 1,
                   "Board init failed.\r\n");
    }

    Drivers_close();
    vTaskDelete(NULL);
}

/*!
 *  \brief
 *  Main entry point.
 *
 *  \param[in]  argc                            Number of arguments.
 *  \param[in]  argv                            Array of arguments.
 *
 *  \return     error                           as int
 *  \retval     #OSAL_NO_ERROR                  Success.
 *  \retval     #else                           Something went wrong.
 *
 */
int main(int argc, char* argv[])
{
    int                                 error = OSAL_NO_ERROR;
    TaskP_Object                        pStartupTaskHandle;
    TaskP_Params                        startupTaskParams;

    IOLM_MAIN_sysInit();
    OSAL_init();
    OSAL_TIMER_set100usecTickSupport(true);
    OSAL_registerPrintOut(NULL, IOLM_Utils_logPrint);

    OSAL_printfSuppress(true);

    TaskP_Params_init(&startupTaskParams);

    startupTaskParams.name          = "Startup Task";
    startupTaskParams.stackSize     = sizeof(IOLM_startupTaskStack_s);
    startupTaskParams.stack         = (uint8_t*)IOLM_startupTaskStack_s;
    startupTaskParams.priority      = OSAL_TASK_ePRIO_Idle;
    startupTaskParams.taskMain      = (TaskP_FxnMain)IOLM_MAIN_startupTask;
    startupTaskParams.args          = NULL;

    error = TaskP_construct(&pStartupTaskHandle, &startupTaskParams);

    if (error != SystemP_SUCCESS)
    {
        OSAL_error(__FILE__, __LINE__, OSAL_eERR_INVALIDSTATE, true, 1,
                   "Creating Startup Task failed.\r\n");
    }

    OSAL_startOs();
    OSAL_error(__FILE__, __LINE__, OSAL_eERR_INVALIDSTATE, true, 1, "OS startup failed.\r\n");

    return error;
}

