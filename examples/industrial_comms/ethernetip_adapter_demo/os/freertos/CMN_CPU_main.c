/*!
 *  \file CMN_CPU_main.c
 *
 *  \brief
 *  Common CPU load analyze support.
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2022-06-08
 *
 *  \copyright
 *  Copyright (c) 2021, KUNBUS GmbH<br /><br />
 *  All rights reserved.<br />
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:<br />
 *  <ol>
 *  <li>Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.</li>
 *  <li>Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.</li>
 *  <li>Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.</li>
 *  </ol>
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include <FreeRTOS.h>
#include <task.h>

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/TaskP.h>

#include <osal.h>

#include <CMN_CPU_api.h>

#define CMN_CPU_OUTPUT_MAX_LINE_SIZE   100

#if (defined CPU_LOAD_MONITOR) && (1==CPU_LOAD_MONITOR)

#include <CMN_CPU_intern.h>

#define CMN_CPU_ANALYZE_STACK_SIZE_BYTE     2048
#define CMN_CPU_ANALYZE_STACK_SIZE          (CMN_CPU_ANALYZE_STACK_SIZE_BYTE/sizeof(configSTACK_DEPTH_TYPE))

static void*         cpuAnalyzeTaskHandle_s = NULL;
static StackType_t   cpuAnalyzeTaskStack_s[CMN_CPU_ANALYZE_STACK_SIZE] __attribute__((aligned(32), section(".threadstack"))) = {0};

static CMN_CPU_API_SData_t  data_s;

static void          CMN_CPU_loadTask       (void *pArg_p);

static TaskP_Object* CMN_CPU_mcuAddTask     (TaskHandle_t pTaskHandle_p);
static TaskP_Object* CMN_CPU_mcuFindTask    (TaskHandle_t pTaskHandle_p);

static uint32_t      CMN_CPU_taskQuickFind  (TaskP_Object* pTaskHandle_p);
static uint32_t      CMN_CPU_taskNameFind   (const char* name);
static uint32_t      CMN_CPU_taskGetFreePos (void);

#endif  // CPU_LOAD_MONITOR == 1

static void          CMN_CPU_output         (CMN_CPU_API_EOutput_t out_p, const char * __restrict pFormat_p, ...);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Starts CPU load monitoring task.
 *
 *  \details
 *  CPU load monitoring needs to be by compilation switch "CPU_LOAD_MONITOR=1" enabled.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pParams_p  CPU load monitoring parameters
 */
void CMN_CPU_API_startMonitor (CMN_CPU_API_SParams_t* pParams_p)
{
#if (defined CPU_LOAD_MONITOR) && (1==CPU_LOAD_MONITOR)

    OSAL_MEMORY_memset(&data_s, 0, sizeof(CMN_CPU_API_SData_t));

    strncpy(data_s.cpu.name, "CPU", sizeof(CMN_CPU_API_LOAD_NAME_MAX_CHARS));

    // Start writing the flash in a thread so that the main task is not blocked
    cpuAnalyzeTaskHandle_s = OSAL_SCHED_startTask (CMN_CPU_loadTask,
                                                   pParams_p,
                                                   pParams_p->taskPrio,
                                                   (uint8_t*) cpuAnalyzeTaskStack_s,
                                                   CMN_CPU_ANALYZE_STACK_SIZE_BYTE,
                                                   OSAL_OS_START_TASK_FLG_NONE,
                                                   "CpuAnalyze");
#endif
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Provides CPU load data.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \return     Pointer to CMN_CPU_API_SData_t as CPU load data.
 *
 */
CMN_CPU_API_SData_t* CMN_CPU_API_getData (void)
{
#if (defined CPU_LOAD_MONITOR) && (1==CPU_LOAD_MONITOR)
    return &data_s;
#else
    return NULL;
#endif
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Provides report of CPU load data.
 *
 */
void CMN_CPU_API_generateReport (CMN_CPU_API_EOutput_t out_p)
{
    uint32_t i = 0;

    CMN_CPU_API_SData_t* pData = CMN_CPU_API_getData();

    if ( (NULL                  != pData) ||
         (CMN_CPU_API_eOUT_NONE != out_p) )
    {
        CMN_CPU_output(out_p, " LOAD: %s  = %2d.%2d %%\r\n", pData->cpu.name, pData->cpu.cpuLoad/100, pData->cpu.cpuLoad%100 );

        for (i = 0; i < CMN_CPU_API_MAX_TASKS_NUM; i++)
        {
            if (NULL != pData->tasks[i].taskHandle)
            {
                CMN_CPU_output(out_p, " LOAD: %s = %2d.%2d %%\r\n", pData->tasks[i].name, pData->tasks[i].cpuLoad/100, pData->tasks[i].cpuLoad%100 );
            }
        }

        CMN_CPU_output(out_p, "----------------------------------------------------\r\n");
    }
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Used to print out the string to specific peripheral.
 *
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  out_p     Defines print out peripheral (UART, Console, ...)
 *  \param[in]  pFormat_p Format string
 *  \param[in]  ...       Parameter list
 *
 */
static void CMN_CPU_output (CMN_CPU_API_EOutput_t out_p, const char * __restrict pFormat_p, ...)
{
    char output[CMN_CPU_OUTPUT_MAX_LINE_SIZE];

    OSAL_MEMORY_memset(output, 0, sizeof(output));

    va_list argptr;
    va_start (argptr, pFormat_p);

    vsnprintf(output, sizeof(output), pFormat_p, argptr);

    va_end(argptr);

    switch(out_p)
    {
        case CMN_CPU_API_eOUT_OSAL_PRINTF:
            OSAL_printf("%s", output);
            break;
        case CMN_CPU_API_eOUT_DEBUGP:
            DebugP_log("%s", output);
            break;
        default:
            break;
    }
}

#if (defined CPU_LOAD_MONITOR) && (1==CPU_LOAD_MONITOR)

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Used to add FreeRTOS task to MCU+SDK CPU load monitoring register.
 *
 *  \details
 *  Use only in case when task is not already present in CPU load register.
 *  Based on FreeRTOS task handle the MCU+SDK task handle is created and
 *  stored to CPU load monitoring register.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pTaskHandle_p     FreeRTOS task handle
 *
 *  \return     Pointer to MCU+SDK task handle as TaskP_Object pointer created from FreeRTOS handle.
 *
 *  \retval     NULL              Failed to add FreeRTOS task to CPU load register.
 *  \retval     other             FreeRTOS task added to CPU load register succesfully.
 *
 */
static TaskP_Object* CMN_CPU_mcuAddTask(TaskHandle_t pTaskHandle_p)
{
    TaskP_Object* pTaskHandle    = NULL;
    uint32_t      schedularState = taskSCHEDULER_NOT_STARTED;
    uint32_t      i              = 0;
    bool          added          = false;

    if (pTaskHandle_p == NULL)
    {
        return NULL;
    }

    pTaskHandle = OSAL_MEMORY_calloc(sizeof(TaskP_Object), 1);

    if (pTaskHandle == NULL)
    {
        OSAL_printf("Func: %s, Line: %lu: Memory allocation of %lu bytes failed.\r\n", __func__, __LINE__, sizeof(TaskP_Object));
        return NULL;
    }

    ((TaskP_Struct *) pTaskHandle)->lastRunTime = 0;
    ((TaskP_Struct *) pTaskHandle)->accRunTime  = 0;
    ((TaskP_Struct *) pTaskHandle)->taskHndl    = pTaskHandle_p;

    schedularState = xTaskGetSchedulerState();

    if (schedularState != taskSCHEDULER_NOT_STARTED)
    {
        vTaskSuspendAll();
    }

    for(i = 0; i < CMN_CPU_API_MAX_TASKS_NUM; i++)
    {
        if(gTaskP_ctrl.taskRegistry[i]==NULL)
        {
            gTaskP_ctrl.taskRegistry[i] = (TaskP_Struct *) pTaskHandle;
            added = true;
            break;
        }
    }

    if(schedularState != taskSCHEDULER_NOT_STARTED)
    {
        xTaskResumeAll();
    }

    if (added == false)
    {
        OSAL_MEMORY_free(pTaskHandle);
        pTaskHandle = NULL;
    }

    return pTaskHandle;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Tries to find specific task defined by FreeRTOS task handle in MCU+SDK CPU load register.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pTaskHandle_p     FreeRTOS task handle
 *
 *  \return     Pointer to MCU+SDK task handle as TaskP_Object pointer when found.
 *
 *  \retval     NULL              Task was not found.
 *  \retval     other             Task already present in CPU load register.
 *
 */
static TaskP_Object* CMN_CPU_mcuFindTask(TaskHandle_t pTaskHandle_p)
{
    TaskP_Object* pTask = NULL;
    uint32_t      i = 0;

    for(i = 0; i < CMN_CPU_API_MAX_TASKS_NUM; i++)
    {
        if(gTaskP_ctrl.taskRegistry[i] != NULL)
        {
            if (gTaskP_ctrl.taskRegistry[i]->taskHndl == pTaskHandle_p)
            {
                pTask = (TaskP_Object*) gTaskP_ctrl.taskRegistry[i];
                break;
            }
        }
    }

    if (pTask == NULL)
    {
        if (pTaskHandle_p != NULL)
        {
            pTask = CMN_CPU_mcuAddTask(pTaskHandle_p);
        }
    }

    return pTask;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Tries to find specific task in existing output data.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pTaskObject_p     MCU task object pointer
 *
 *  \return     Output data index when found in output data.
 *
 *  \retval     -1               Task was not found in output data array.
 *  \retval     other            Index to the array of output data.
 *
 */
static uint32_t CMN_CPU_taskQuickFind (TaskP_Object* pTaskObject_p)
{
    uint32_t i     = 0;
    uint32_t index = -1;

    for (i = 0; i < CMN_CPU_API_MAX_TASKS_NUM; i++)
    {
        if (data_s.tasks[i].taskHandle == pTaskObject_p)
        {
            index = i;
            break;
        }
    }

    return index;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Tries to find specific task in existing output data based on string.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pTaskName_p      Name of task
 *
 *  \return     Output data index when found in output data.
 *
 *  \retval     -1               Task was not found in output data array.
 *  \retval     other            Index to the array of output data.
 *
 */
static uint32_t CMN_CPU_taskNameFind (const char* name)
{
    uint32_t i     = 0;
    uint32_t index = -1;

    for (i = 0; i < CMN_CPU_API_MAX_TASKS_NUM; i++)
    {
        if (data_s.tasks[i].taskHandle != NULL)
        {
            if (0 == strncmp(data_s.tasks[i].name, name, CMN_CPU_API_LOAD_NAME_MAX_CHARS))
            {
                index = i;
                break;
            }
        }
    }

    return index;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Get first available position in output data.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \return     Output data index when found free place in output data.
 *
 *  \retval     -1               Free place was not found in output data array.
 *  \retval     other            Index to the array of output data.
 *
 */
static uint32_t CMN_CPU_taskGetFreePos (void)
{
    uint32_t i     = 0;
    uint32_t index = -1;

    for (i = 0; i < CMN_CPU_API_MAX_TASKS_NUM; i++)
    {
        if (data_s.tasks[i].taskHandle == NULL)
        {
            index = i;
            break;
        }
    }

    return index;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  CPU load monitoring task.
 *
 *  \details
 *  Checks list of all running tasks on side of FreeRTOS, compares the list with CPU load register
 *  on side of MCU+SDK and extends the CPU load register with missing task instead of IDLE task.
 *  At the end makes printout on specific peripheral.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pArg_p     pointer to task parameters
 *
 */
static void CMN_CPU_loadTask (void *pArg_p)
{
    CMN_CPU_API_SParams_t* pParams;
    TaskP_Object*          pTask;
    TaskP_Load             taskLoad;
    TaskStatus_t*          pxStatusArray;
    uint32_t               cpuLoad;
    uint32_t               statusArraySize;
    uint32_t               totalRuntime;
    uint32_t               i, k;

    pParams = (CMN_CPU_API_SParams_t*) pArg_p;

    for (;;)
    {
        TaskP_loadResetAll();

        OSAL_SCHED_sleep(1000);

        statusArraySize = uxTaskGetNumberOfTasks();

        pxStatusArray = OSAL_MEMORY_calloc(statusArraySize * sizeof(TaskStatus_t), 1);

        if (pxStatusArray == NULL)
        {
            OSAL_printf("Func: %s, Line: %lu: Memory allocation of %lu bytes failed.\r\n", __func__, __LINE__, statusArraySize * sizeof(TaskStatus_t));
            break;
        }

        uxTaskGetSystemState(pxStatusArray, statusArraySize, &totalRuntime);

        cpuLoad = TaskP_loadGetTotalCpuLoad();

        data_s.cpu.cpuLoad = cpuLoad;

        for (i = 0; i < CMN_CPU_API_MAX_TASKS_NUM; i++)
        {
            data_s.tasks[i].exists = false;
        }

        for (i = 0; i < statusArraySize; i++)
        {
            pTask = CMN_CPU_mcuFindTask(pxStatusArray[i].xHandle);

            if (pTask != NULL)
            {
                TaskP_loadGet(pTask, &taskLoad);

                k = CMN_CPU_taskQuickFind(pTask);

                if (-1 != k)
                {
                    data_s.tasks[k].cpuLoad = taskLoad.cpuLoad;
                    data_s.tasks[k].exists  = true;
                    continue;
                }

                k = CMN_CPU_taskNameFind(taskLoad.name);

                if (-1 != k)
                {
                    data_s.tasks[k].taskHandle = pTask;
                    data_s.tasks[k].cpuLoad    = taskLoad.cpuLoad;
                    data_s.tasks[k].exists     = true;
                    continue;
                }

                k = CMN_CPU_taskGetFreePos();

                if (-1 != k)
                {
                    data_s.tasksNum++;

                    data_s.tasks[k].taskHandle = pTask;
                    data_s.tasks[k].cpuLoad    = taskLoad.cpuLoad;
                    data_s.tasks[k].exists     = true;

                    strncpy(data_s.tasks[k].name, taskLoad.name, CMN_CPU_API_LOAD_NAME_MAX_CHARS);
                    continue;
                }
            }
        }

        for (i = 0; i < CMN_CPU_API_MAX_TASKS_NUM; i++)
        {
            if (false == data_s.tasks[i].exists)
            {
                data_s.tasks[i].cpuLoad = 0;
            }
        }

        OSAL_MEMORY_free(pxStatusArray);

        CMN_CPU_API_generateReport(pParams->output);
    }
}

#endif  // CPU_LOAD_MONITOR == 1
