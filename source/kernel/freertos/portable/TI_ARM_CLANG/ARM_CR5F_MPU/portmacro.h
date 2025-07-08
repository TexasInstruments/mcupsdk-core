/*
 * FreeRTOS Kernel V11.1.0
 * Copyright (C) 2024 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */
/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
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

#ifndef PORTMACRO_H
#define PORTMACRO_H

/*-----------------------------------------------------------
 * Port specific definitions.
 *
 * Functions, Defines, and Structs for use in the ARM_CR5F_MPU FreeRTOS-Port
 * 
 * The settings in this file configure FreeRTOS correctly for the given
 * hardware and compiler. These settings should not be altered.
 */

#ifdef __cplusplus
extern "C" {
#endif

/* Include stdint for integer types of specific bit widths. */
#include <stdint.h>

#include <kernel/dpl/HwiP.h>
#include <kernel/nortos/dpl/r5/HwiP_armv7r_vim.h>

#include <FreeRTOSConfig.h>

/* ------------------------------ FreeRTOS Config Check ------------------------------ */
//MPU_ONLY_START
#ifndef configSYSTEM_CALL_STACK_SIZE
    #error "Define configSYSTEM_CALL_STACK_SIZE to a length, in bytes, " \
            "to use when an unprivileged task makes a FreeRTOS Kernel call. "
#endif /* configSYSTEM_CALL_STACK_SIZE */

#if( configUSE_MPU_WRAPPERS_V1 == 1 )
    #error This port is usable with MPU wrappers V2 only.
#endif /* configUSE_MPU_WRAPPERS_V1 */

#if( configCHECK_FOR_STACK_OVERFLOW == 1 )
    #error "This port cannot use configCHECK_FOR_STACK_OVERFLOW v1, \
            since it checks for stack pointer being out of bounds using  \
            `pxCurrentTCB->pxTopOfStack`. But this port set this value to  \
            `TCB -> xMPU_SETTINGS -> ulContext[]` since task context is  \
            stored / restored from here.  \
            Hence instead set configCHECK_FOR_STACK_OVERFLOW to v2."
#endif /* configUSE_MPU_WRAPPERS_V1 */

#include "portmacro_asm.h"
//MPU_ONLY_STOP

/* ------------------------------ Port Type Definitions ------------------------------ */

#if( configUSE_PORT_OPTIMISED_TASK_SELECTION == 1 )

    /* Check the configuration. */
    #if( configMAX_PRIORITIES > 32 )
        #error "configUSE_PORT_OPTIMISED_TASK_SELECTION can only be set to 1 when " \
                "configMAX_PRIORITIES is less than or equal to 32. " \
                "It is very rare that a system requires more than 10 to 15 difference " \
                "priorities as tasks that share a priority will time slice."
    #endif /* ( configMAX_PRIORITIES > 32 ) */

    /* Mark that a task of the given priority is ready. */
    #define portRECORD_READY_PRIORITY( uxPriority, uxTopReadyPriority ) \
        ( uxTopReadyPriority ) |= ( 1UL << ( uxPriority ) )

    /* Mark that a task of the given priority is no longer ready. */
    #define portRESET_READY_PRIORITY( uxPriority, uxTopReadyPriority ) \
        ( uxTopReadyPriority ) &= ~( 1UL << ( uxPriority ) )

    /* Determine the highest priority ready task's priority. */
    #define portGET_HIGHEST_PRIORITY( uxTopPriority, uxTopReadyPriority ) \
        ( uxTopPriority ) = ( 31UL - __builtin_clz( ( uxTopReadyPriority ) ) )

#endif /* configUSE_PORT_OPTIMISED_TASK_SELECTION */

/** Type definitions. */
/* Data type used to represent a stack word. */
typedef uint32_t StackType_t;

/* Signed data type equal to the data word operating size of the CPU. */
typedef int32_t BaseType_t;

/* Unsigned data type equal to the data word operating size of the CPU. */
typedef uint32_t UBaseType_t;

/** Data type used for the FreeRTOS Tick Counter.
 *
 * Note: Using 32-bit tick type on a 32-bit architecture ensures that reads of
 * the tick count do not need to be guarded with a critical section. */
typedef uint32_t TickType_t;

/* Marks the direction the stack grows on the targeted CPU. */
#define portSTACK_GROWTH   ( -1 )

/* Specifies stack pointer alignment requirements of the target CPU. */
#define portBYTE_ALIGNMENT 8U

/** Task function prototype macro as described on FreeRTOS.org.
 *
 * Note: This is not required for this port but included in case common demo
 * code uses it.
 */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) \
    void vFunction( void * pvParameters )

/** Task function prototype macro as described on FreeRTOS.org.
 *
 * Note: This is not required for this port but included in case common demo
 * code uses it. */
#define portTASK_FUNCTION( vFunction, pvParameters ) \
    void vFunction( void * pvParameters )

/* The no-op ARM assembly instruction. */
#define portNOP()                                    __asm volatile( "NOP" )

/* The inline compiler label. */
#define portINLINE                                   __inline

/* The always inline compiler label. */
#define portFORCE_INLINE    inline __attribute__( ( always_inline ) )

/* The memory access synchronization barrier. */
#define portMEMORY_BARRIER()                         __asm volatile( "" ::: "memory" )

/* Ensure a symbol isn't removed from the compilation unit. */
#define portDONT_DISCARD                             __attribute__( ( used ) )

/* The number of miliseconds between system ticks. */
#define portTICK_PERIOD_MS                           ( ( TickType_t ) 1000UL / configTICK_RATE_HZ )

/* The largest possible delay value for any FreeRTOS API. */
#define portMAX_DELAY                                ( TickType_t ) 0xFFFFFFFFUL

/* ----------------------------- Port Assembly Functions ----------------------------- */

//MPU_DIFF_START
// MPU port performs SVC call with a specific index
void vPortYield( void );

#define portYIELD() vPortYield()
//MPU_DIFF_STOP

/* -------------------------------------------------------------------------------------- */

void vPortYeildFromISR( uint32_t x );

#define portYIELD_FROM_ISR( x ) vPortYeildFromISR( x )
#define portEND_SWITCHING_ISR( x ) vPortYeildFromISR( x )

extern volatile UBaseType_t ulPortInterruptNesting;
#define portASSERT_IF_IN_ISR() configASSERT( ulPortInterruptNesting == 0 )

/* Critical section control */
extern void vTaskEnterCritical( void );

#define portENTER_CRITICAL() vTaskEnterCritical()

extern void vTaskExitCritical( void );

#define portEXIT_CRITICAL() vTaskExitCritical()

#if (configUSE_INTERRUPT_PRIORITY_BASED_CRITICAL_SECTIONS == 1)

    #define portDISABLE_INTERRUPTS()                \
                (void)HwiP_setVimIrqPriMaskAtomic( configMAX_SYSCALL_INTERRUPT_PRIORITY )

    #define portENABLE_INTERRUPTS()                 \
                (void)HwiP_setVimIrqPriMaskNonAtomic( HwiP_MAX_PRIORITY )

    #define portSET_INTERRUPT_MASK_FROM_ISR()       \
                HwiP_setVimIrqPriMaskAtomic( configMAX_SYSCALL_INTERRUPT_PRIORITY )

    #define portCLEAR_INTERRUPT_MASK_FROM_ISR( x )  \
                HwiP_restoreVimIrqPriMask( x )

#else /* #if configUSE_INTERRUPT_PRIORITY_BASED_CRITICAL_SECTIONS == 1 */

    #define portDISABLE_INTERRUPTS()                  __asm__ volatile ( "CPSID	i" ::: "cc" )

    #define portENABLE_INTERRUPTS()                   __asm__ volatile ( "CPSIE	i" ::: "cc" )

    #define portSET_INTERRUPT_MASK_FROM_ISR()         HwiP_disable()

    #define portCLEAR_INTERRUPT_MASK_FROM_ISR( x )    HwiP_restore( x )

#endif /* #else configUSE_INTERRUPT_PRIORITY_BASED_CRITICAL_SECTIONS == 1 */

/* Run time stats utilities. */
void vPortConfigTimerForRunTimeStats();

#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() vPortConfigTimerForRunTimeStats()

uint32_t uiPortGetRunTimeCounterValue();

#define portGET_RUN_TIME_COUNTER_VALUE() uiPortGetRunTimeCounterValue()

void vPortStartFirstTask( void );

/* Architecture specific optimisations. */
#ifdef configASSERT

    void vPortValidateInterruptPriority( void );

    #define portASSERT_IF_INTERRUPT_PRIORITY_INVALID()    vPortValidateInterruptPriority()

#endif /* configASSERT */

//MPU_ONLY_START
/* Exit from a FreeRTOS System Call.*/
void vPortSystemCallExit( void );

/** Checks whether or not the processor is privileged.
 *
 * Note: The processor privilege level is determined by checking the
 * mode bits [4:0] of the Current Program Status Register (CPSR).
 *
 * Returns pdTRUE, if the processer is privileged, pdFALSE otherwise.
 */
BaseType_t xPortIsPrivileged( void );

#define portIS_PRIVILEGED() xPortIsPrivileged()

/** Checks whether or not a task is privileged.
 *
 * Note: A task's privilege level is associated with the task and is different from
 * the processor's privilege level returned by xPortIsPrivileged. For example,
 * the processor is privileged when an unprivileged task executes a system call.
 *
 * Returns pdTRUE if the task is privileged, pdFALSE otherwise.
 */
BaseType_t xPortIsTaskPrivileged( void );

#define portIS_TASK_PRIVILEGED() xPortIsTaskPrivileged()

/* --------------------------------- MPU Definitions --------------------------------- */

/* Mark that this port utilizes the onboard ARM MPU. */
#define portUSING_MPU_WRAPPERS     1

/** Used to mark if a task should be created as a privileged task.
 *
 * Note: A privileged task is created by performing a bitwise OR of this value and
 * the task priority. For example, to create a privileged task at priority 2, the
 * uxPriority parameter should be set to ( 2 | portPRIVILEGE_BIT ).
 */
#define portPRIVILEGE_BIT          ( 0x80000000UL )

/* Size of an Access Control List (ACL) entry in bits. */
#define portACL_ENTRY_SIZE_BITS    ( 32UL )

/** Structure to hold the MPU Register Values.
 *
 * Note: The ordering of this struct MUST be in sync with the ordering in
 * portRESTORE_CONTEXT.
 */
typedef struct MPU_REGION_REGISTERS
{
    uint32_t ulRegionSize;        /* Information for MPU Region Size and Enable Register. */
    uint32_t ulRegionAttribute;   /* Information for MPU Region Access Control Register. */
    uint32_t ulRegionBaseAddress; /* Information for MPU Region Base Address Register. */
} xMPU_REGION_REGISTERS;

/** Structure to hold per-task System Call Stack information.
 *
 * Note: The ordering of this structure MUST be in sync with the assembly code
 * of the port.
 */
typedef struct SYSTEM_CALL_STACK_INFO
{
    uint32_t * pulTaskStackPointer;              /* Stack Pointer of the task when it made a FreeRTOS System Call. */
    uint32_t * pulLinkRegisterAtSystemCallEntry; /* Link Register of the task when it made a FreeRTOS System Call. */
    uint32_t * pulSystemCallStackPointer;        /* Stack Pointer to use for executing a FreeRTOS System Call. */
    uint32_t * pulSystemCallExitAddress;         /* System call exit address. */
    uint32_t ulSystemCallStackBuffer[ configSYSTEM_CALL_STACK_SIZE ]; /* Buffer to be used as stack when performing a FreeRTOS System Call. */
} xSYSTEM_CALL_STACK_INFO;

/** Per-Task MPU settings structure stored in the TCB.
 *
 * Note: The ordering of this structure MUST be in sync with the assembly code
 * of the port.
 */
typedef struct MPU_SETTINGS
{
    xMPU_REGION_REGISTERS xRegion[ portTOTAL_NUM_REGIONS_IN_TCB ];
    uint32_t ulTaskFlags;
    xSYSTEM_CALL_STACK_INFO xSystemCallStackInfo;
    uint32_t ulContext[ CONTEXT_SIZE ]; /* Buffer used to store task context. */

    #if( configENABLE_ACCESS_CONTROL_LIST == 1 )
        uint32_t ulAccessControlList[ ( configPROTECTED_KERNEL_OBJECT_POOL_SIZE
                                        / portACL_ENTRY_SIZE_BITS )
                                      + 1UL ];
    #endif
} xMPU_SETTINGS;
//MPU_ONLY_STOP

#ifdef __cplusplus
} /* extern C */
#endif

#endif /* PORTMACRO_H */
