/*
 *  Copyright (C) 2018-2023 Texas Instruments Incorporated
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

#include <stdbool.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <FreeRTOS.h>
#include <task.h>

void vApplicationLoadHook(void);

#define TaskP_LOAD_CPU_LOAD_SCALE   (10000U)
#define TaskP_REGISTRY_MAX_ENTRIES  (32u)
#define TaskP_STACK_SIZE_MIN        (128U)

#ifdef SMP_FREERTOS
typedef struct {

    TaskP_Object *taskRegistry[TaskP_REGISTRY_MAX_ENTRIES];
    uint32_t lastTotalTime;
    uint64_t accTotalTime;
    uint32_t idleTsk1LastRunTime;
    uint64_t idleTsk1AccRunTime;
    uint32_t idleTsk2LastRunTime;
    uint64_t idleTsk2AccRunTime;

} TaskP_Ctrl;
#else
typedef struct {

    TaskP_Object *taskRegistry[TaskP_REGISTRY_MAX_ENTRIES];
    uint32_t lastTotalTime;
    uint64_t accTotalTime;
    uint32_t idleTskLastRunTime;
    uint64_t idleTskAccRunTime;

} TaskP_Ctrl;
#endif


TaskP_Ctrl gTaskP_ctrl;

static void TaskP_assertParams(TaskP_Object *obj, const TaskP_Params *params, bool isStackAlignToSize)
{
    DebugP_assert(params != NULL);
    DebugP_assert(obj != NULL);

    DebugP_assert(params->stackSize >= TaskP_STACK_SIZE_MIN);
    DebugP_assert((params->stackSize & (sizeof(configSTACK_DEPTH_TYPE) - 1U)) == 0U);
    DebugP_assert(params->stack != NULL);

    if (isStackAlignToSize) 
    {
        /* Task stack should be aligned to task size to add as an MPU region. */
        DebugP_assert(((uintptr_t)params->stack & (params->stackSize - 1U)) == 0U);
    }
    else
    {
        DebugP_assert(((uintptr_t)params->stack & (sizeof(configSTACK_DEPTH_TYPE) - 1U)) == 0U);
    }

    DebugP_assert(params->taskMain != NULL);
}

static uint32_t TaskP_adjustPriority(const TaskP_Params *params)
{
    uint32_t priority = params->priority;

    /* if priority is out of range, adjust to bring it in range */
    priority = (priority > TaskP_PRIORITY_HIGHEST) ? TaskP_PRIORITY_HIGHEST : priority;
    priority = (priority <= TaskP_PRIORITY_LOWEST) ? TaskP_PRIORITY_LOWEST : priority;

    return priority;
}

static void TaskP_addToRegistry(TaskP_Object *task)
{
    uint32_t i;
    BaseType_t schedularState;

    task->lastRunTime = 0U;
    task->accRunTime  = 0U;

    schedularState = xTaskGetSchedulerState();
    if(schedularState != taskSCHEDULER_NOT_STARTED)
    {
        vTaskSuspendAll();
    }

    for(i=0; i<TaskP_REGISTRY_MAX_ENTRIES; i++)
    {
        if(gTaskP_ctrl.taskRegistry[i]==NULL)
        {
            gTaskP_ctrl.taskRegistry[i] = task;
            break;
        }
    }

    if(schedularState != taskSCHEDULER_NOT_STARTED)
    {
        xTaskResumeAll();
    }
}

static void TaskP_removeFromRegistry(TaskP_Object *task)
{
    uint32_t i;
    BaseType_t schedularState;

    schedularState = xTaskGetSchedulerState();
    if(schedularState != taskSCHEDULER_NOT_STARTED)
    {
        vTaskSuspendAll();
    }

    for(i=0; i<TaskP_REGISTRY_MAX_ENTRIES; i++)
    {
        if(gTaskP_ctrl.taskRegistry[i]==task)
        {
            gTaskP_ctrl.taskRegistry[i] = NULL;
            break;
        }
    }

    if(schedularState != taskSCHEDULER_NOT_STARTED)
    {
        xTaskResumeAll();
    }
}

static uint32_t TaskP_calcCounterDiff(uint32_t cur, uint32_t last)
{
    uint32_t delta;

    if(cur >= last)
    {
        delta = cur - last;
    }
    else
    {
        delta = (  0xFFFFFFFFU - last ) + cur;
    }
    return delta;
}

static uint32_t TaskP_calcCpuLoad(uint64_t taskTime, uint64_t totalTime)
{
    uint32_t cpuLoad;

    cpuLoad = (taskTime * TaskP_LOAD_CPU_LOAD_SCALE) / totalTime;
    if( cpuLoad > TaskP_LOAD_CPU_LOAD_SCALE)
    {
        cpuLoad = TaskP_LOAD_CPU_LOAD_SCALE;
    }
    return cpuLoad;
}

static void TaskP_updateLoad(TaskP_Object *taskObj)
{
#ifndef SMP_FREERTOS
#if (configGENERATE_RUN_TIME_STATS == 1)
    uint32_t runTimeCounter = ulTaskGetRunTimeCounter(taskObj->taskHndl);

    uint32_t delta = TaskP_calcCounterDiff(runTimeCounter, taskObj->lastRunTime);

    taskObj->accRunTime += delta;
    taskObj->lastRunTime = runTimeCounter;
#endif
#else
    uint32_t delta;
    TaskStatus_t taskStatus;

    vTaskGetInfo(taskObj->taskHndl, &taskStatus, pdFALSE, eReady);

    delta = TaskP_calcCounterDiff(taskStatus.ulRunTimeCounter, taskObj->lastRunTime);

    taskObj->accRunTime += delta;
    taskObj->lastRunTime = taskStatus.ulRunTimeCounter;
#endif
}

static void TaskP_updateIdleTaskLoad()
{
#ifndef SMP_FREERTOS
#if (configGENERATE_RUN_TIME_STATS == 1)
    uint32_t runTimeCounter = ulTaskGetIdleRunTimeCounter();
    
    uint32_t delta = TaskP_calcCounterDiff(runTimeCounter, gTaskP_ctrl.idleTskLastRunTime);

    gTaskP_ctrl.idleTskAccRunTime += delta;
    gTaskP_ctrl.idleTskLastRunTime = runTimeCounter;
#endif
#else
    uint32_t delta;
    TaskStatus_t taskStatus;
    TaskHandle_t* idleTskHndl = xTaskGetIdleTaskHandle();

    if(idleTskHndl != NULL)
    {
        vTaskGetInfo(idleTskHndl[0], &taskStatus, pdFALSE, eReady);

        delta = TaskP_calcCounterDiff(taskStatus.ulRunTimeCounter, gTaskP_ctrl.idleTsk1LastRunTime);

        gTaskP_ctrl.idleTsk1AccRunTime += delta;
        gTaskP_ctrl.idleTsk1LastRunTime = taskStatus.ulRunTimeCounter;

        vTaskGetInfo(idleTskHndl[1], &taskStatus, pdFALSE, eReady);

        delta += TaskP_calcCounterDiff(taskStatus.ulRunTimeCounter, gTaskP_ctrl.idleTsk2LastRunTime);

        gTaskP_ctrl.idleTsk2AccRunTime += delta;
        gTaskP_ctrl.idleTsk2LastRunTime = taskStatus.ulRunTimeCounter;
    }
#endif
}

static void TaskP_updateTotalRunTime()
{
    uint32_t curTotalTime = portGET_RUN_TIME_COUNTER_VALUE();
    uint32_t delta = TaskP_calcCounterDiff(curTotalTime, gTaskP_ctrl.lastTotalTime);
    
    gTaskP_ctrl.accTotalTime += delta;
    gTaskP_ctrl.lastTotalTime = curTotalTime;
}

void TaskP_Params_init(TaskP_Params *params)
{
    params->name = "Task (DPL)";
    params->stackSize = 0;
    params->stack = NULL;
    params->priority = ((TaskP_PRIORITY_HIGHEST - TaskP_PRIORITY_LOWEST) /(uint32_t) 2);
    params->args = NULL;
    params->taskMain = NULL;
#ifdef SMP_FREERTOS
    params->coreAffinity = (uintptr_t)(~0);
#endif
}

int32_t TaskP_construct(TaskP_Object *taskObj, const TaskP_Params *params)
{
    int32_t      status   = SystemP_SUCCESS;
    uint32_t     priority;

    TaskP_assertParams(taskObj, params, false);

    priority = TaskP_adjustPriority(params);

#if (portUSING_MPU_WRAPPERS == 1)
    /* Create privileged tasks with TaskP_construct */
    priority |= portPRIVILEGE_BIT; /* A privileged task is created by performing a bitwise OR of 
                                    * portPRIVILEGE_BIT with the task priority */
#endif

    TaskP_addToRegistry(taskObj);

    taskObj->taskHndl = xTaskCreateStatic( params->taskMain, /* Pointer to the function that implements the task. */
                                  params->name,              /* Text name for the task.  This is to facilitate debugging only. */
                                  params->stackSize/(sizeof(configSTACK_DEPTH_TYPE)),  /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                  params->args,       /* task specific args */
                                  priority,           /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                  (StackType_t*)params->stack,      /* pointer to stack base */
                                  &taskObj->taskTcb); /* pointer to statically allocated task object memory */
    if(taskObj->taskHndl == NULL)
    {
        status = SystemP_FAILURE;
    }

#ifdef SMP_FREERTOS
    /* Set core affinity in case of SMP FreeRTOS */
    vTaskCoreAffinitySet(taskObj->taskHndl, params->coreAffinity);
#endif

    return status;
}

#if ( portUSING_MPU_WRAPPERS == 1 )
#if defined(__ARM_ARCH_7R__) 
void TaskP_ParamsRestricted_init(TaskP_ParamsRestricted *params)
{
    /* Initialize standard task parameters */
    TaskP_Params_init(&params->params);

    /* Initialize MPU region configurations */
    for (uint32_t i = 0; i < portNUM_CONFIGURABLE_REGIONS; i++)
    {
        params->regionConfig[i].baseAddr  = 0U;
        params->regionConfig[i].sizeBytes = 0U;
        MpuP_RegionAttrs_init(&params->regionConfig[i].attrs);
    }
}

int32_t TaskP_constructRestricted(TaskP_Object *taskObj, const TaskP_ParamsRestricted *params)
{
    BaseType_t         xResult;
    int32_t            status      = SystemP_SUCCESS;
    const TaskP_Params *taskParams = &params->params;

    TaskP_assertParams(taskObj, taskParams, true);

    xTaskParameters xTaskParams = {
        .pvTaskCode     = taskParams->taskMain,
        .pcName         = taskParams->name,
        .usStackDepth   = taskParams->stackSize / sizeof(configSTACK_DEPTH_TYPE),
        .pvParameters   = taskParams->args,
        .uxPriority     = TaskP_adjustPriority(taskParams),
        .puxStackBuffer = (StackType_t *)taskParams->stack,
        .pxTaskBuffer   = &taskObj->taskTcb,
    };

    /* Convert TaskP_MpuRegionConfig to MemoryRegion_t */
    for (uint32_t i = 0; i < portNUM_CONFIGURABLE_REGIONS; i++)
    {
        uint32_t MpuP_getAttrs(const MpuP_RegionAttrs *region);
        
        xTaskParams.xRegions[i].pvBaseAddress   = (void *)params->regionConfig[i].baseAddr;
        xTaskParams.xRegions[i].ulLengthInBytes = params->regionConfig[i].sizeBytes;
        xTaskParams.xRegions[i].ulParameters    = MpuP_getAttrs(&params->regionConfig[i].attrs);
    }

    TaskP_addToRegistry(taskObj);

    xResult = xTaskCreateRestrictedStatic(&xTaskParams, &taskObj->taskHndl);
    if ((xResult != pdPASS) || (taskObj->taskHndl == NULL))
    {
        status = SystemP_FAILURE;
    }

    return status;
}
#endif /* #if ( portUSING_MPU_WRAPPERS == 1 ) */
#endif /* #if defined(__ARM_ARCH_7R__) */

void TaskP_destruct(TaskP_Object *taskObj)
{
    if(taskObj && taskObj->taskHndl)
    {
        vTaskDelete(taskObj->taskHndl);
        taskObj->taskHndl = NULL;
    }
    if(taskObj != NULL)
    {
        TaskP_removeFromRegistry(taskObj);
    }
}

void* TaskP_getHndl(TaskP_Object *taskObj)
{
    return  (void*)taskObj->taskHndl;
}

void TaskP_yield(void)
{
    taskYIELD();
}

void TaskP_exit(void)
{
#if ( portUSING_MPU_WRAPPERS == 1 )
    if (xPortIsPrivileged() == pdFALSE)
    {
        /** Un-privileged tasks can't delete itself. 
         * Instead a privileged task should delete the same.
         * Hence move to blocked state. */
        vTaskSuspend(NULL);
    }
    else 
#endif
    {
        vTaskDelete(NULL);
    }
}

void TaskP_loadGet(TaskP_Object *taskObj, TaskP_Load *taskLoad)
{
    if(taskObj->taskHndl == xTaskGetCurrentTaskHandle())
    {
        /* Perform a force yield for the kernel to update the current task run time counter value. */
        taskYIELD();
    }

    vTaskSuspendAll();

    TaskP_updateLoad(taskObj);
    TaskP_updateTotalRunTime();

    taskLoad->runTime = taskObj->accRunTime;
    taskLoad->totalTime  = gTaskP_ctrl.accTotalTime;
    taskLoad->cpuLoad = TaskP_calcCpuLoad(taskObj->accRunTime, gTaskP_ctrl.accTotalTime);
    taskLoad->name = pcTaskGetName(taskObj->taskHndl);

    (void)xTaskResumeAll();
}

uint32_t TaskP_loadGetTotalCpuLoad(void)
{
    uint32_t cpuLoad;

    vTaskSuspendAll();

    /* This is need to get upto date IDLE task statistics, since the next update window could be some time away */
    TaskP_updateIdleTaskLoad();
    TaskP_updateTotalRunTime();

    #ifdef SMP_FREERTOS
    cpuLoad = TaskP_LOAD_CPU_LOAD_SCALE - TaskP_calcCpuLoad(gTaskP_ctrl.idleTsk1AccRunTime + gTaskP_ctrl.idleTsk2AccRunTime, gTaskP_ctrl.accTotalTime);
    #else
    cpuLoad = TaskP_LOAD_CPU_LOAD_SCALE - TaskP_calcCpuLoad(gTaskP_ctrl.idleTskAccRunTime, gTaskP_ctrl.accTotalTime);
    #endif

    (void)xTaskResumeAll();

    return cpuLoad;
}

void TaskP_loadResetAll(void)
{
    TaskP_Object *taskObj;
    uint32_t i;

    /* Perform a force yield for the kernel to update the current task run time counter value. */
    taskYIELD();
    /** This is need to store upto date value in `taskObj->lastRunTime` and `gTaskP_ctrl.idleTskLastRunTime`,
     *  since the next update window will use the difference b/w these values and latest counter value to 
     *  increment the accumulated run time. */
    TaskP_loadUpdateAll();

    vTaskSuspendAll();

    for(i=0; i<TaskP_REGISTRY_MAX_ENTRIES; i++)
    {
        if(gTaskP_ctrl.taskRegistry[i]!=NULL)
        {
            taskObj = gTaskP_ctrl.taskRegistry[i];

            taskObj->accRunTime = 0;
        }
    }

#ifdef SMP_FREERTOS
    gTaskP_ctrl.idleTsk1AccRunTime = 0;
    gTaskP_ctrl.idleTsk2AccRunTime = 0;
    gTaskP_ctrl.accTotalTime = 0;
#else
    gTaskP_ctrl.idleTskAccRunTime = 0;
    gTaskP_ctrl.accTotalTime = 0;
#endif

    (void)xTaskResumeAll();
}

void TaskP_loadUpdateAll(void)
{
    TaskP_Object *taskObj;
    uint32_t i;

    vTaskSuspendAll();

    for(i=0; i<TaskP_REGISTRY_MAX_ENTRIES; i++)
    {
        taskObj = gTaskP_ctrl.taskRegistry[i];

        if(taskObj != NULL)
        {
            TaskP_updateLoad(taskObj);
        }
    }

    TaskP_updateIdleTaskLoad();
    TaskP_updateTotalRunTime();
    
    (void)xTaskResumeAll();
}

void vApplicationLoadHook(void)
{
    static uint64_t lastUpdateTime = 0;
    uint64_t curUpdateTime = ClockP_getTimeUsec();

    if( (curUpdateTime > lastUpdateTime) && ((curUpdateTime - lastUpdateTime) > (TaskP_LOAD_UPDATE_WINDOW_MSEC*1000u )) )
    {
        TaskP_loadUpdateAll();
        lastUpdateTime = curUpdateTime;
    }
}
