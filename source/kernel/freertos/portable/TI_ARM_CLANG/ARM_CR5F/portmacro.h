/*
 * FreeRTOS Kernel V10.4.1
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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
 * 1 tab == 4 spaces!
 */
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

#ifndef PORTMACRO_H
    #define PORTMACRO_H

    #ifdef __cplusplus
        extern "C" {
    #endif

#include <kernel/dpl/HwiP.h>
#include <kernel/nortos/dpl/r5/HwiP_armv7r_vim.h>

/*-----------------------------------------------------------
 * Port specific definitions.
 *
 * The settings in this file configure FreeRTOS correctly for the given hardware
 * and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

/* Macro to enable priority based interrupt masking using
   configMAX_SYSCALL_INTERRUPT_PRIORITY in critical section.
   This feature is NOT supported on AM273x & must be disabled */
// #define EN_MAX_SYSCALL_INTR_PRI_CRIT_SECTION

#ifdef EN_MAX_SYSCALL_INTR_PRI_CRIT_SECTION
#define PRIO_MASK       ((((uint32_t)0x1<<(configMAX_SYSCALL_INTERRUPT_PRIORITY))-1))
#define MAX_PRIO_MASK   ((((uint32_t)0x1<<(HwiP_MAX_PRIORITY))-1))
#endif

/* Type definitions. */
    #define portCHAR         char
    #define portFLOAT        float
    #define portDOUBLE       double
    #define portLONG         long
    #define portSHORT        short
    #define portSTACK_TYPE   uint32_t
    #define portBASE_TYPE    long

    typedef portSTACK_TYPE   StackType_t;
    typedef long             BaseType_t;
    typedef unsigned long    UBaseType_t;

    typedef uint32_t         TickType_t;
    #define portMAX_DELAY    ( TickType_t ) 0xffffffffUL

/* Hardware specifics. */
    #define portSTACK_GROWTH      ( -1 )
    #define portBYTE_ALIGNMENT    8

/* Timer related define's */
    #define portTICK_PERIOD_MS	( ( TickType_t ) 1000 / configTICK_RATE_HZ )

/* Task switch utilities. */
    extern void vPortYeildFromISR( uint32_t x );
    #define portYIELD_FROM_ISR( x )    vPortYeildFromISR( x )
    #define portEND_SWITCHING_ISR( x ) vPortYeildFromISR( x )
    #define portYIELD()                __asm__ volatile ( "SWI 0" ::: "memory" )

/* Critical section control */
    #define portCRITICAL_NESTING_IN_TCB					(1)

    extern void vTaskEnterCritical( void );
    extern void vTaskExitCritical( void );
    #define portENTER_CRITICAL()		vTaskEnterCritical()
    #define portEXIT_CRITICAL()			vTaskExitCritical()
    #define portDISABLE_INTERRUPTS()                  vDisableInterruptPriority()
    #define portENABLE_INTERRUPTS()                   vEnableInterruptPriority()
    #define portSET_INTERRUPT_MASK_FROM_ISR()         HwiP_disable()
    #define portCLEAR_INTERRUPT_MASK_FROM_ISR( x )    HwiP_restore( x )

/* Task function macros as described on the FreeRTOS.org WEB site.  These are
 * not required for this port but included in case common demo code that uses these
 * macros is used. */
    #define portTASK_FUNCTION_PROTO( vFunction, pvParameters )    void vFunction( void * pvParameters )
    #define portTASK_FUNCTION( vFunction, pvParameters )          void vFunction( void * pvParameters )

/* Any task that uses the floating point unit MUST call vPortTaskUsesFPU()
 * before any floating point instructions are executed. */
    extern void vPortTaskUsesFPU( void );
    #define portTASK_USES_FLOATING_POINT()    vPortTaskUsesFPU()

    #if configUSE_PORT_OPTIMISED_TASK_SELECTION == 1

        /* Store/clear the ready priorities in a bit map. */
        #define portRECORD_READY_PRIORITY( uxPriority, uxReadyPriorities )    ( uxReadyPriorities ) |= ( 1UL << ( uxPriority ) )
        #define portRESET_READY_PRIORITY( uxPriority, uxReadyPriorities )     ( uxReadyPriorities ) &= ~( 1UL << ( uxPriority ) )

        #define portGET_HIGHEST_PRIORITY( uxTopPriority, uxReadyPriorities )    do {(uxReadyPriorities) ? (uxTopPriority = ( 31 - __builtin_clz( (uxReadyPriorities) ) )) : (uxTopPriority = 0); } while(0)

    #endif /* configUSE_PORT_OPTIMISED_TASK_SELECTION */

/* Architecture specific optimisations. */
    #ifdef configASSERT
        void vPortValidateInterruptPriority( void );
        #define portASSERT_IF_INTERRUPT_PRIORITY_INVALID()    vPortValidateInterruptPriority()
    #endif /* configASSERT */


    #ifdef __cplusplus
        } /* extern C */
    #endif

    #define portMEMORY_BARRIER()    __asm__ volatile ( "" ::: "memory" )

/* Inline definition */
    #define portINLINE __inline

#ifndef portFORCE_INLINE
    #define portFORCE_INLINE    inline __attribute__( ( always_inline ) )
#endif

/* Disable all IRQ interrupts or upto certain priority  if EN_MAX_SYSCALL_INTR_PRI_CRIT_SECTION is enabled */
portFORCE_INLINE static void vDisableInterruptPriority( void )
{
#ifdef EN_MAX_SYSCALL_INTR_PRI_CRIT_SECTION

    uint32_t*   ptrVimIrqPrimask    = (uint32_t* )(gHwiConfig.intcBaseAddr + VIM_IRQPRIMASK);

    *(ptrVimIrqPrimask) = (uint32_t)PRIO_MASK;
#else
    __asm__ volatile ( "CPSID	i" ::: "cc" );
#endif
}

/* Enable all IRQ interrupts  */
portFORCE_INLINE static void vEnableInterruptPriority( void )
{
#ifdef EN_MAX_SYSCALL_INTR_PRI_CRIT_SECTION

    uint32_t*   ptrVimIrqPrimask    = (uint32_t* )(gHwiConfig.intcBaseAddr + VIM_IRQPRIMASK);

    *(ptrVimIrqPrimask) = (uint32_t)MAX_PRIO_MASK;
#else
    __asm__ volatile ( "CPSIE	i" ::: "cc" );
#endif
}

#endif /* PORTMACRO_H */
