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


#ifndef TI_FREERTOS_CONFIG_H
#define TI_FREERTOS_CONFIG_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <kernel/dpl/DebugP.h>

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE AND IN THE
 * FreeRTOS REFERENCE MANUAL.
 *----------------------------------------------------------*/

/* Keep below as 1 if the most optimized task switching latency is needed.
 * This disables tracing, logging, assert and other error checks.
 * So unless every last cycle of task switching is important leave this as 0.
 *
 * This is not a FreeRTOS defined config and is defined by TI to quickly switch
 * between optimized and not-so-optimized config
 */
#define configOPTIMIZE_FOR_LATENCY              (0)

#define configUSE_PREEMPTION					(1)
#define configUSE_PORT_OPTIMISED_TASK_SELECTION	(0)
#define configNUM_CORES                         (2)
#define configRUN_MULTIPLE_PRIORITIES           (1)
#define configUSE_CORE_AFFINITY                 (1)
#define configUSE_TICKLESS_IDLE                 (0)
#define configUSE_IDLE_HOOK                     (1)  /* when 1, make sure to implement void vApplicationIdleHook(void) as the hook function */
#define configUSE_MINIMAL_IDLE_HOOK             (1)  /* when 1, make sure to implement void vApplicationMinimalIdleHook(void) as the hook function */
#define configUSE_MALLOC_FAILED_HOOK            (0)
#define configUSE_DAEMON_TASK_STARTUP_HOOK      (0)
#define configUSE_TICK_HOOK                     (0)
#define configCPU_CLOCK_HZ                      (0) /* NOT USED in TI ports */
#define configSYSTICK_CLOCK_HZ                  (0) /* NOT USED in TI ports */
#define configTICK_RATE_HZ                      (1000)
#define configMAX_PRIORITIES                    (32)
#define configMINIMAL_STACK_SIZE                (1024) /* in units of configSTACK_DEPTH_TYPE, not bytes */
#define configMAX_TASK_NAME_LEN                 (32)
#define configUSE_TRACE_FACILITY                (1)
#if (configOPTIMIZE_FOR_LATENCY==0)
#define configUSE_STATS_FORMATTING_FUNCTIONS    (1)
#else
#define configUSE_STATS_FORMATTING_FUNCTIONS    (0)
#endif
#define configUSE_16_BIT_TICKS                  (0)
#define configIDLE_SHOULD_YIELD                 (1)
#define configUSE_TASK_NOTIFICATIONS            (1)
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   (1)
#define configUSE_MUTEXES                       (1)
#define configUSE_RECURSIVE_MUTEXES             (1)
#define configUSE_COUNTING_SEMAPHORES           (1)
#define configUSE_ALTERNATIVE_API               (0)

/* when = 1, Need to provied below,
 *    void vApplicationStackOverflowHook( TaskHandle_t xTask,
 *           char *pcTaskName );
 */
#if (configOPTIMIZE_FOR_LATENCY==0)
#define configCHECK_FOR_STACK_OVERFLOW          (1)
#else
#define configCHECK_FOR_STACK_OVERFLOW          (0)
#endif
#define configQUEUE_REGISTRY_SIZE               (32)
#define configUSE_QUEUE_SETS                    (0)
#define configUSE_TIME_SLICING                  (0) /* keep as 0 to get same functionality as SysBIOS6 */
#define configUSE_NEWLIB_REENTRANT              (0)
#define configENABLE_BACKWARD_COMPATIBILITY     (1)
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS (4)
#define configSTACK_DEPTH_TYPE                  UBaseType_t
#define configMESSAGE_BUFFER_LENGTH_TYPE        size_t
#define configSUPPORT_STATIC_ALLOCATION         (1) /* when = 1, need to provide below,
                                                     *
                                                     * void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                                     *      StackType_t **ppxTimerTaskStackBuffer,
                                                     *      uint32_t *pulTimerTaskStackSize );
                                                     *
                                                     * void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                                     *      StackType_t **ppxIdleTaskStackBuffer,
                                                     *      uint32_t *pulIdleTaskStackSize );
                                                     */
#define configSUPPORT_DYNAMIC_ALLOCATION        (1)
#define configTOTAL_HEAP_SIZE                   (0xF80000) /* not used when heap_3.c is the selected heap */
#define configAPPLICATION_ALLOCATED_HEAP        (0)

/* run-time stats config */
#if (configOPTIMIZE_FOR_LATENCY==0)
#define configGENERATE_RUN_TIME_STATS           (1)
#else
#define configGENERATE_RUN_TIME_STATS           (0)
#endif
void vPortConfigTimerForRunTimeStats();
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() vPortConfigTimerForRunTimeStats()
uint32_t uiPortGetRunTimeCounterValue();
#define portGET_RUN_TIME_COUNTER_VALUE()        uiPortGetRunTimeCounterValue()

/* co-routine related config */
#define configUSE_CO_ROUTINES                   (0)
#define configMAX_CO_ROUTINE_PRIORITIES         (0)

/* timer related config */
#define configUSE_TIMERS                        (1)
#define configTIMER_TASK_PRIORITY               (configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH                (16)
#define configTIMER_TASK_STACK_DEPTH            (256)

/* used in M4F, not applicable in R5F, C66x, A53 */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    (0xE0U)
#define configKERNEL_INTERRUPT_PRIORITY         (configMAX_SYSCALL_INTERRUPT_PRIORITY)
#define configMAX_API_CALL_INTERRUPT_PRIORITY   (configMAX_SYSCALL_INTERRUPT_PRIORITY)

#if (configOPTIMIZE_FOR_LATENCY==0)
#define configASSERT(x)                        DebugP_assert( (uint32_t)(x))
#endif

/* MPU aware FreeRTOS, not supported as TI */
#define configINCLUDE_APPLICATION_DEFINED_PRIVILEGED_FUNCTIONS  (0)
#define configTOTAL_MPU_REGIONS                                 (0)
#undef  configTEX_S_C_B_FLASH
#undef  configTEX_S_C_B_SRAM
#define configENFORCE_SYSTEM_CALLS_FROM_KERNEL_ONLY             (0)

/* defines need for FreeRTOS+POSIX*/
#define configUSE_POSIX_ERRNO           (1)
#define configUSE_APPLICATION_TASK_TAG  (1)

/* include specific functions */
#define INCLUDE_vTaskDelete             (1)
#define INCLUDE_vTaskDelay              (1)
#define INCLUDE_vTaskSuspend            (1)
#define INCLUDE_xTimerDelete            (1)
#define INCLUDE_vSemaphoreDelete        (1)
#define INCLUDE_xTimerPendFunctionCall  (1)
#define INCLUDE_xTaskGetIdleTaskHandle  (1)

#ifdef __cplusplus
}
#endif

#endif /* TI_FREERTOS_CONFIG_H */
