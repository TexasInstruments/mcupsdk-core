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

#ifndef TASKP_H
#define TASKP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <kernel/dpl/SystemP.h>
#if defined(__ARM_ARCH_7R__)
#include <kernel/dpl/MpuP_armv7.h>
#endif

/**
 * \defgroup KERNEL_DPL_TASK APIs for Task
 * \ingroup KERNEL_DPL
 *
 * For more details and example usage, see \ref KERNEL_DPL_TASK_PAGE
 *
 * @{
 */

/**
 * \brief Entry point to the task
 */
typedef void (*TaskP_FxnMain)(void *args);

/**
 * \brief Value to be used for lowest priority task
 */
#define TaskP_PRIORITY_LOWEST       (0u)

/**
 * \brief Value to be used for highest priority task
 */
#define TaskP_PRIORITY_HIGHEST      (31u)

/**
 * \brief The update rate at which TaskP_loadUpdateAll() is called
 */
#define TaskP_LOAD_UPDATE_WINDOW_MSEC   (500u)


/**
 * \brief Task load statistics
 */
typedef struct {

    const char  *name;     /**< Name of the task */
    uint64_t runTime;   /**< Amount of time the task has run, units of usec */
    uint64_t totalTime; /**< Amount of time that has elapsed so far, units of usec */
    uint32_t cpuLoad;   /**< CPU load in units of percentage with 2 decimal point precision, i.e 1234 means 12.34% */

} TaskP_Load;

/**
 * \brief Opaque task object used with the task APIs
 */

#if defined (OS_FREERTOS) || defined (OS_FREERTOS_SMP) || defined (OS_FREERTOS_MPU)

#include <FreeRTOS.h>
#include <task.h>

typedef struct TaskP_Object_ {

    StaticTask_t taskTcb;
    TaskHandle_t taskHndl;
    uint32_t     lastRunTime;
    uint64_t     accRunTime;

} TaskP_Object;

#elif defined (OS_SAFERTOS)

#include <SafeRTOS.h>
#include <task.h>

typedef struct TaskP_Object_ {

    xTCB                taskTcb;
    portTaskHandleType  taskHndl;
    uint32_t            lastRunTime;
    uint64_t            accRunTime;

} TaskP_Object;

#elif defined (OS_NORTOS)

/** Add a dummy `TaskP_Object` since `TaskP.h` is included by drivers which will be built with NORTOS as well.
 * `TaskP_yield` API is implemented for NORTOS. Usage of other APIs with NORTOS will result in linking errors.
 */
typedef struct TaskP_Object_ {

    uint32_t rsv;

} TaskP_Object;

#else 

#error "Define OS_FREERTOS or OS_FREERTOS_SMP or OS_FREERTOS_MPU or OS_SAFERTOS or OS_NORTOS"

#endif

/**
 * \brief Parameters passed during \ref TaskP_construct
 */
typedef struct TaskP_Params_ {

    const char   *name;         /**< Pointer to task name */
    uint32_t      stackSize;    /**< Size of stack in units of bytes */
    uint8_t      *stack;        /**< Pointer to stack memory, MUST be aligned based on CPU architecture, typically atleast 32b on 32b systems */
    uint32_t      priority;     /**< Task priority, MUST be between \ref TaskP_PRIORITY_LOWEST and TaskP_PRIORITY_HIGHEST */
    void         *args;         /**< User arguments that are passed back as parameter to task main */
    TaskP_FxnMain taskMain;     /**< Entry point function to the task */
    uintptr_t     coreAffinity; /**< Core affinity for the task (Applicable in case of SMP only)*/

} TaskP_Params;

#if defined(__ARM_ARCH_7R__)
/**
 * \brief Task specific MPU regions config
 * 
 * This is used with FreeRTOS MPU port only.
 */
typedef struct TaskP_MpuRegionConfig_ {
    uint32_t baseAddr;      /**< Region start address, MUST aligned to region size */
    uint32_t sizeBytes;     /**< Region size in bytes */
    /** \brief Region attributes, see \ref MpuP_RegionAttrs
     * 
     *  \note Following params are not supported with the FreeRTOS MemoryRegion_t 
     *        and hence won't take effect:-
     *        1. isEnable              
     *           - Will be enabled if valid \p sizeBytes is provided
     *        2. subregionDisableMask 
     *           - All sub-regions will be enabled
     */
    MpuP_RegionAttrs attrs; 
} TaskP_MpuRegionConfig;

/**
 * \brief Parameters passed during \ref TaskP_constructRestricted
 * 
 * This is used with FreeRTOS MPU port only.
 */
typedef struct TaskP_ParamsRestricted_ {
    TaskP_Params          params;           /**< Standard task parameters */
    TaskP_MpuRegionConfig regionConfig[7U]; /**< MPU region configurations */
    // TODO: Replace with below after TaskP refactor to include FreeRTOS headers and structs directly.
    // TaskP_MpuRegionConfig regionConfig[portNUM_CONFIGURABLE_REGIONS]; /**< MPU region configurations */
} TaskP_ParamsRestricted;
#endif

/**
 * \brief Set default values to TaskP_Params
 *
 * Strongly recommended to be called before seting values in TaskP_Params
 *
 * \param params [out] parameter structure to set to default
 */
void TaskP_Params_init(TaskP_Params *params);

/**
 * \brief Create a task object
 * 
 * \note With FreeRTOS MPU port, only privileged tasks can be created with this API.
 *       To create un-privileged tasks use \p TaskP_constructRestricted
 *
 * \param obj [out] Created object
 * \param params [in] Task create parameters
 *
 * \return \ref SystemP_SUCCESS on success, \ref SystemP_FAILURE on error
 */
int32_t TaskP_construct(TaskP_Object *obj, const TaskP_Params *params);

#if defined(__ARM_ARCH_7R__)
/**
 * \brief Set default values to TaskP_ParamsRestricted
 * 
 * \note This API is supported only with FreeRTOS MPU Port
 *
 * Strongly recommended to be called before setting values in TaskP_ParamsRestricted
 *
 * \param params [out] parameter structure to set to default
 */
void TaskP_ParamsRestricted_init(TaskP_ParamsRestricted *params);

/**
 * \brief Create a restricted task object (un-privileged task)
 * 
 * \note This API is supported only with FreeRTOS MPU Port
 * 
 * \param obj [out] Created object
 * \param params [in] Task create parameters
 *
 * \return \ref SystemP_SUCCESS on success, \ref SystemP_FAILURE on error
 */
int32_t TaskP_constructRestricted(TaskP_Object *obj, const TaskP_ParamsRestricted *params);
#endif

/**
 * \brief Cleanup, delete, destruct a task object
 *
 * \param obj [in] task object
 */
void TaskP_destruct(TaskP_Object *obj);


/**
 * \brief Return OS defined task handle
 *
 * \param obj [in] task object
 *
 * \return OS specific task handle
 */
void* TaskP_getHndl(TaskP_Object *obj);

/**
 * \brief Yield current task
 */
void TaskP_yield(void);

/**
 * \brief Exit current task
 *
 * In FreeRTOS, task cannot simply return from a function. It needs to call vTaskDelete(NULL) instead.
 * To keep the task exit portable, call this function when a task wants to terminate itself.
 *
 * \note With FreeRTOS MPU port, an unprivileged task can't delete itself. 
 *       Instead a privileged task should delete the same using \ref TaskP_destruct.
 *       Hence this API will suspend the current task.
 */
void TaskP_exit(void);

/**
 * \brief Get task load
 *
 * \param obj [out] Created object
 * \param taskLoad [out] Task load statistics
 */
void TaskP_loadGet(TaskP_Object *obj, TaskP_Load *taskLoad);

/**
 * \brief Updated task load statistics
 *
 * This updates task load statistics for all tasks created with TaskP_construct().
 * This also updates idle task load.
 *
 * This function is called, every \ref TaskP_LOAD_UPDATE_WINDOW_MSEC msecs within the IDLE task.
 * It is important that idle task get to run atleast once every \ref TaskP_LOAD_UPDATE_WINDOW_MSEC msecs
 * for the load statistics to be correct
 */
void TaskP_loadUpdateAll(void);

/**
 * \brief Reset task load statistics
 *
 * Until load statistics is reset the load statistics keep getting accumulated
 */
void TaskP_loadResetAll(void);


/**
 * \brief Get total CPU load including all task and ISR execution time
 *
 * \return CPU load in units of percentage with 2 decimal point precision, i.e 1234 means 12.34%
 */
uint32_t TaskP_loadGetTotalCpuLoad(void);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* TASKP_H */
