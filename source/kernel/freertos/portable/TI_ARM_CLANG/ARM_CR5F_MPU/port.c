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

/* Standard includes. */
#include <stdint.h>

//MPU_ONLY_START
/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
 * all the API functions to use the MPU wrappers. That should only be done when
 * task.h is included from an application file. */
#ifndef MPU_WRAPPERS_INCLUDED_FROM_API_FILE
    #define MPU_WRAPPERS_INCLUDED_FROM_API_FILE
#endif /* MPU_WRAPPERS_INCLUDED_FROM_API_FILE */
//MPU_ONLY_STOP

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "portmacro.h"
#include "task.h"
#include "mpu_syscall_numbers.h"

#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/MpuP_armv7.h>
#include <kernel/dpl/CycleCounterP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/nortos/dpl/r5/HwiP_armv7r_vim.h>

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/* Let the user override the pre-loading of the initial LR with the address of
 * prvTaskExitError() in case it messes up unwinding of the stack in the
 * debugger. */
#ifdef configTASK_RETURN_ADDRESS
    #define portTASK_RETURN_ADDRESS    configTASK_RETURN_ADDRESS
#else
    #define portTASK_RETURN_ADDRESS    prvTaskExitError
#endif

/* Bit in Current Program Status Register (CPSR) to indicate that CPU is
 * in Thumb State. */
#define portTHUMB_MODE_BIT      ( ( StackType_t ) 0x20 )

/* Bitmask to check if an address is of Thumb Code. */
#define portTHUMB_MODE_ADDRESS  ( 0x01UL )

// portINITIAL_SPSR is not needed for MPU port. 
// This is instead set to SYS_MODE or USER_MODE based on privileged task flag

//TODO: Check how to handle FPU

/* Max value that fits in a uint32_t type. */
#define portUINT32_MAX    ( ~( ( uint32_t ) 0 ) )

/* Check if adding a and b will result in overflow. */
#define portADD_UINT32_WILL_OVERFLOW( a, b )    ( ( a ) > ( portUINT32_MAX - ( b ) ) )
/* ----------------------------------------------------------------------------------- */

/* Set to 1 to pend a context switch from an ISR. */
PRIVILEGED_DATA volatile UBaseType_t ulPortYieldRequired = pdFALSE;

/* Interrupt nesting depth, used to count the number of interrupts to unwind. */
PRIVILEGED_DATA volatile UBaseType_t ulPortInterruptNesting = 0UL;

/** Variable to track whether or not the scheduler has been started.
 *
 * This is the port specific version of the xSchedulerRunning in tasks.c.
 */
PRIVILEGED_DATA static BaseType_t prvPortSchedulerRunning = pdFALSE;

/* -------------------------- Private Function Declarations -------------------------- */

//MPU_ONLY_START
/** Determine if the given MPU region settings authorizes the requested
 * access to the given buffer.
 * 
 * Returns pdTRUE if MPU region settings authorizes the requested access to the
 * given buffer, pdFALSE otherwise.
 */
PRIVILEGED_FUNCTION static BaseType_t prvMPURegionAuthorizesBuffer( const xMPU_REGION_REGISTERS * xTaskMPURegion,
                                                                    const uint32_t ulBufferStart,
                                                                    const uint32_t ulBufferLength,
                                                                    const uint32_t ulAccessRequested );

/** Determine the smallest MPU Region Size Encoding for the given MPU
 * region size in bytes.
 *
 * Returns the smallest MPU Region Size Encoding for the given MPU region size.
 */
PRIVILEGED_FUNCTION static uint32_t prvGetMPURegionSizeEncoding( uint32_t ulActualMPURegionSize );

/** Decode the given MPU Region Size Encoding to MPU region size in bytes.
 *
 * Returns MPU region size in bytes for the given Encoded MPU region size.
 */
PRIVILEGED_FUNCTION static uint32_t prvGetMPURegionSizeDecode( uint32_t ulRegionSizeEncoded );
//MPU_ONLY_STOP

/* ----------------------------------------------------------------------------------- */

void prvTaskExitError( void )
{
    /* A function that implements a task must not exit or attempt to return to
     * its caller as there is nothing to return to. If a task wants to exit it
     * should instead call vTaskDelete( NULL ).
     *
     * Artificially force an assert() to be triggered if configASSERT() is
     * defined, then stop here so application writers can catch the error. */
    DebugP_assertNoLog(0);
}

/* ----------------------------------------------------------------------------------- */

//MPU_DIFF_START
// - `xRunPrivileged` and `xMPUSettings` is only for MPU port
// - MPU port stores task context in `TCB -> xMPUSettings -> ulContext[]`,
//   wheres non-MPU port stores in stack
/** Setup a FreeRTOS task's initial context.
 *
 * Returns the location where to restore the task's context from.
 */
/* PRIVILEGED_FUNCTION */
StackType_t * pxPortInitialiseStack( StackType_t * pxTopOfStack,
                                     TaskFunction_t pxCode,
                                     void * pvParameters,
                                     BaseType_t xRunPrivileged,
                                     xMPU_SETTINGS * xMPUSettings )
{
    /* Setup the initial context of the task. The context is set exactly as
     * expected by the portRESTORE_CONTEXT() macro. */
    UBaseType_t ulIndex = CONTEXT_SIZE - 1U;

    xSYSTEM_CALL_STACK_INFO * xSysCallInfo = NULL;

    if( xRunPrivileged == pdTRUE )
    {
        xMPUSettings->ulTaskFlags |= portTASK_IS_PRIVILEGED_FLAG;
        /* Current Program Status Register (CPSR). */
        xMPUSettings->ulContext[ ulIndex ] = SYS_MODE;
    }
    else
    {
        xMPUSettings->ulTaskFlags &= ( ~portTASK_IS_PRIVILEGED_FLAG );
        /* Current Program Status Register (CPSR). */
        xMPUSettings->ulContext[ ulIndex ] = USER_MODE;
    }

    if( ( ( uint32_t ) pxCode & portTHUMB_MODE_ADDRESS ) != 0x0UL )
    {
        /* The task will cause the processor to start in THUMB state, set the
         * Thumb state bit in the CPSR. */
        xMPUSettings->ulContext[ ulIndex ] |= portTHUMB_MODE_BIT;
    }

    ulIndex--;

    xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) pxCode; /* PC. */
    ulIndex--;

    xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) portTASK_RETURN_ADDRESS; /* LR. */
    ulIndex--;

    xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) pxTopOfStack; /* SP. */
    ulIndex--;

    /* General Purpose Registers. */
    xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0x12121212; /* R12. */
    ulIndex--;
    xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0x11111111; /* R11. */
    ulIndex--;
    xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0x10101010; /* R10. */
    ulIndex--;
    xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0x09090909; /* R9. */
    ulIndex--;
    xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0x08080808; /* R8. */
    ulIndex--;
    xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0x07070707; /* R7. */
    ulIndex--;
    xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0x06060606; /* R6. */
    ulIndex--;
    xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0x05050505; /* R5. */
    ulIndex--;
    xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0x04040404; /* R4. */
    ulIndex--;
    xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0x03030303; /* R3. */
    ulIndex--;
    xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0x02020202; /* R2. */
    ulIndex--;
    xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0x01010101; /* R1. */
    ulIndex--;
    xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) pvParameters; /* R0. */
    ulIndex--;

    #if( portENABLE_FPU == 1 )
    {
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD0000015; /* S31. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD1500000; /* S30. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD0000014; /* S29. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD1400000; /* S28. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD0000013; /* S27. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD1300000; /* S26. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD0000012; /* S25. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD1200000; /* S24. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD0000011; /* S23. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD1100000; /* S22. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD0000010; /* S21. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD1000000; /* S20. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD0000009; /* S19. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD9000000; /* S18. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD0000008; /* S17. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD8000000; /* S16. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD0000007; /* S15. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD7000000; /* S14. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD0000006; /* S13. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD6000000; /* S12. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD0000005; /* S11. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD5000000; /* S10. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD0000004; /* S9. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD4000000; /* S8. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD0000003; /* S7. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD3000000; /* S6. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD0000002; /* S5. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD2000000; /* S4. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD0000001; /* S3. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD1000000; /* S2. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD0000000; /* S1. */
        ulIndex--;
        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0xD0000000; /* S0. */
        ulIndex--;

        xMPUSettings->ulContext[ ulIndex ] = ( StackType_t ) 0x00000000; /* FPSR. */
    }
    #endif /* portENABLE_FPU */

    /* Ensure that the system call stack is double word aligned. */
    xSysCallInfo = &( xMPUSettings->xSystemCallStackInfo );
    xSysCallInfo->pulSystemCallStackPointer = &( xSysCallInfo->ulSystemCallStackBuffer[ configSYSTEM_CALL_STACK_SIZE - 1U ] );
    xSysCallInfo->pulSystemCallStackPointer = ( uint32_t * ) ( ( ( uint32_t ) ( xSysCallInfo->pulSystemCallStackPointer ) ) &
                                                               ( ( uint32_t ) ( ~( portBYTE_ALIGNMENT_MASK ) ) ) );

    /* This is not NULL only for the duration of a system call. */
    xSysCallInfo->pulTaskStackPointer = NULL;

    /* Set the System Call to return to vPortSystemCallExit. */
    xSysCallInfo->pulSystemCallExitAddress = ( uint32_t * ) ( &vPortSystemCallExit );

    /* Return the address where this task's context should be restored from. */
    return &( xMPUSettings->ulContext[ ulIndex ] );
}
//MPU_DIFF_STOP

/* ----------------------------------------------------------------------------------- */

//MPU_ONLY_START
/** Store a FreeRTOS task's MPU settings in its TCB.
 *
 * - xMPUSettings The MPU settings in TCB.
 * - xRegions The updated MPU settings requested by the task.
 * - pxBottomOfStack The base address of the task's Stack.
 * - ulStackDepth The length of the task's stack.
 */
/* PRIVILEGED_FUNCTION */
void vPortStoreTaskMPUSettings( xMPU_SETTINGS * xMPUSettings,
                                const struct xMEMORY_REGION * const xRegions,
                                StackType_t * pxBottomOfStack,
                                uint32_t ulStackDepth )
{
    uint32_t ulIndex = 0x0;
    uint32_t ulRegionIndex = 0x0;
    uint32_t ulRegionLength;
    uint32_t ulRegionLengthEncoded;
    uint32_t ulRegionLengthDecoded;

    /** Task stack is added to task specific MPU region only if user has provided various MPU regions
     *  for the task. If the user has not provided any MPU region for the task, then task stack would 
     *  also be skipped. This is to avoid task stack alignment issue with task size to add as an MPU
     *  region. */
    if( xRegions != NULL )
    {
        /* This function is called automatically when the task is created - in
         * which case the stack region parameters will be valid. At all other
         * times the stack parameters will not be valid and it is assumed that the
         * stack region has already been configured. */
        if( ulStackDepth != 0x0UL )
        {
            ulRegionLengthEncoded = prvGetMPURegionSizeEncoding( ulStackDepth * ( uint32_t ) sizeof( StackType_t ) );

            /* MPU region base address must be aligned to the region size
             * boundary. */
            ulRegionLengthDecoded = prvGetMPURegionSizeDecode( ulRegionLengthEncoded );
            configASSERT( ( ( uint32_t ) pxBottomOfStack % ( ulRegionLengthDecoded ) ) == 0U );

            xMPUSettings->xRegion[ ulIndex ].ulRegionBaseAddress = ( uint32_t ) pxBottomOfStack;
            xMPUSettings->xRegion[ ulIndex ].ulRegionSize = ( ulRegionLengthEncoded |
                                                                portMPU_REGION_ENABLE );
            xMPUSettings->xRegion[ ulIndex ].ulRegionAttribute = ( portMPU_REGION_PRIV_RW_USER_RW_NOEXEC |
                                                                    portMPU_REGION_NORMAL_OIWBWA_NONSHARED );
            ulIndex++;
        }

        while( ulIndex < portTOTAL_NUM_REGIONS_IN_TCB)
        {
            ulRegionLength = xRegions[ ulRegionIndex ].ulLengthInBytes;
            /* If a length has been provided, the region is in use. */
            if( ulRegionLength > 0UL )
            {
                ulRegionLengthEncoded = prvGetMPURegionSizeEncoding( ulRegionLength );

                /* MPU region base address must be aligned to the region size boundary. */
                ulRegionLengthDecoded = prvGetMPURegionSizeDecode( ulRegionLengthEncoded );
                configASSERT( ( ( ( uint32_t ) xRegions[ ulRegionIndex ].pvBaseAddress ) % ( ulRegionLengthDecoded ) ) == 0UL );

                xMPUSettings->xRegion[ ulIndex ].ulRegionBaseAddress = ( uint32_t ) xRegions[ ulRegionIndex ].pvBaseAddress;
                xMPUSettings->xRegion[ ulIndex ].ulRegionSize = ( ulRegionLengthEncoded |
                                                                  portMPU_REGION_ENABLE );
                xMPUSettings->xRegion[ ulIndex ].ulRegionAttribute = xRegions[ ulRegionIndex ].ulParameters;
            }
            else
            {
                xMPUSettings->xRegion[ ulIndex ].ulRegionBaseAddress = 0x0UL;
                xMPUSettings->xRegion[ ulIndex ].ulRegionSize = 0x0UL;
                xMPUSettings->xRegion[ ulIndex ].ulRegionAttribute = 0x0UL;
            }
    
            ulIndex++;
            ulRegionIndex++;
        }
    } 
    else 
    {
        /* MPU Settings is zero'd out in the TCB before this function is called.
         * We, therefore, do not need to explicitly zero out unused MPU regions
         * in xMPUSettings. */
    }
}

/* ----------------------------------------------------------------------------------- */

/* PRIVILEGED_FUNCTION */
BaseType_t xPortIsTaskPrivileged( void )
{
    BaseType_t xTaskIsPrivileged = pdFALSE;

    /* Calling task's MPU settings. */
    const xMPU_SETTINGS * xTaskMpuSettings = xTaskGetMPUSettings( NULL );

    if( ( xTaskMpuSettings->ulTaskFlags & portTASK_IS_PRIVILEGED_FLAG ) == portTASK_IS_PRIVILEGED_FLAG )
    {
        xTaskIsPrivileged = pdTRUE;
    }

    return xTaskIsPrivileged;
}
//MPU_ONLY_STOP

/* ----------------------------------------------------------------------------------- */

/* PRIVILEGED_FUNCTION */
BaseType_t xPortStartScheduler( void )
{
    /* Interrupts are turned off in the CPU itself to ensure tick does
     * not execute while the scheduler is being started.  Interrupts are
     * automatically turned back on in the CPU when the first task starts
     * executing.
     */
    __asm__ volatile ( "CPSID	i" ::: "cc" );
    /* Set a initial PRI mask for max priority, so that when the first task starts it will enable IRQ and
     * the primask is setup to enable all interrupts */
    (void)HwiP_setVimIrqPriMaskAtomic( HwiP_MAX_PRIORITY );

    /* MPU is already enabled with static regions by the DPL config */

    CycleCounterP_enableUserAccess();

    prvPortSchedulerRunning = pdTRUE;

    /* Load the context of the first task. */
    vPortStartFirstTask();

    /* Will only get here if vTaskStartScheduler() was called with the CPU in
     * a non-privileged mode or the binary point register was not set to its lowest
     * possible value. prvTaskExitError() is referenced to prevent a compiler
     * warning about it being defined but not referenced in the case that the user
     * defines their own exit address. */
    ( void ) prvTaskExitError();

    return pdFALSE;
}

/* ----------------------------------------------------------------------------------- */

void vPortYeildFromISR( uint32_t xSwitchRequired )
{
    if( xSwitchRequired != pdFALSE )
    {
        ulPortYieldRequired = pdTRUE;
    }
}

/* ----------------------------------------------------------------------------------- */

void vPortTimerTickHandler()
{
    /* Enter critical section from ISR */
    uint32_t key = taskENTER_CRITICAL_FROM_ISR();

    if( prvPortSchedulerRunning == pdTRUE )
    {
        /* Increment the RTOS tick. */
        if( xTaskIncrementTick() != pdFALSE )
        {
            ulPortYieldRequired = pdTRUE;
        }
    }
    taskEXIT_CRITICAL_FROM_ISR( key );
}

/* ----------------------------------------------------------------------------------- */

/* initialize high resolution timer for CPU and task load calculation */
void vPortConfigTimerForRunTimeStats()
{
    /* we assume clock is initialized before the schedular is started */
}

/* ----------------------------------------------------------------------------------- */

/* return current counter value of high speed counter in units of usecs */
uint32_t uiPortGetRunTimeCounterValue()
{
    uint64_t timeInUsecs = ClockP_getTimeUsec();

    /* note, there is no overflow protection for this 32b value in FreeRTOS
     *
     * This value will overflow in
     * ((0xFFFFFFFF)/(1000000*60)) minutes ~ 71 minutes
     *
     * We call vApplicationLoadHook() in idle loop to accumulate the task load into a 64b value.
     * The implementation of vApplicationLoadHook() is in source\kernel\freertos\dpl\common\TaskP_freertos.c
     */
    return (uint32_t)(timeInUsecs);
}

/* ----------------------------------------------------------------------------------- */

/* This is used to make sure we are using the FreeRTOS API from within a valid interrupt priority level */
void vPortValidateInterruptPriority()
{
    #if (configUSE_INTERRUPT_PRIORITY_BASED_CRITICAL_SECTIONS == 1)
    /*
     * The following assertion will fail if a service routine (ISR) for
     * an interrupt that has been assigned a priority above
     * configMAX_SYSCALL_INTERRUPT_PRIORITY calls an ISR safe FreeRTOS API
     * function.  ISR safe FreeRTOS API functions must *only* be called
     * from interrupts that have been assigned a priority at or below
     * configMAX_SYSCALL_INTERRUPT_PRIORITY.
     *
     * Numerically low interrupt priority numbers represent logically high
     * interrupt priorities, therefore the priority of the interrupt must
     * be set to a value equal to or numerically *higher* than
     * configMAX_SYSCALL_INTERRUPT_PRIORITY.
     *
     * FreeRTOS maintains separate thread and ISR API functions to ensure
     * interrupt entry is as fast and simple as possible.
     */
    configASSERT( HwiP_getActivePriority() >= ( uint32_t ) configMAX_SYSCALL_INTERRUPT_PRIORITY );
    #endif /* (configUSE_INTERRUPT_PRIORITY_BASED_CRITICAL_SECTIONS == 1) */
}

/* ----------------------------------------------------------------------------------- */

//MPU_ONLY_START
/* PRIVILEGED_FUNCTION */
static uint32_t prvGetMPURegionSizeEncoding( uint32_t ulActualMPURegionSize )
{
    uint32_t ulRegionSize, ulReturnValue = 4U;

    /* 32 bytes is the smallest valid region for Cortex R5 CPUs. */
    for( ulRegionSize = 0x20UL; ulReturnValue < 0x1FUL; ( ulRegionSize <<= 1UL ) )
    {
        if( ulActualMPURegionSize <= ulRegionSize )
        {
            break;
        }
        else
        {
            ulReturnValue++;
        }
    }

    /* Shift the code by one before returning so it can be written directly
     * into the the correct bit position of the attribute register. */
    return ulReturnValue << 1UL;
}

static uint32_t prvGetMPURegionSizeDecode( uint32_t ulRegionSizeEncoded )
{
    return ( 2UL << ( ulRegionSizeEncoded >> 1UL ) );
}

/* ----------------------------------------------------------------------------------- */

/* PRIVILEGED_FUNCTION */
static BaseType_t prvMPURegionAuthorizesBuffer( const xMPU_REGION_REGISTERS * xTaskMPURegion,
                                                const uint32_t ulBufferStart,
                                                const uint32_t ulBufferLength,
                                                const uint32_t ulAccessRequested )
{
    BaseType_t xAccessGranted = pdFALSE;
    uint32_t ulBufferEnd;
    uint32_t ulMPURegionLength;
    uint32_t ulMPURegionStart;
    uint32_t ulMPURegionEnd;
    uint32_t ulMPURegionAccessPermissions;

    if( portADD_UINT32_WILL_OVERFLOW( ulBufferStart, ( ulBufferLength - 1UL ) ) == pdFALSE )
    {
        ulBufferEnd = ulBufferStart + ulBufferLength - 1UL;
        ulMPURegionLength = prvGetMPURegionSizeDecode( xTaskMPURegion->ulRegionSize );
        ulMPURegionStart = xTaskMPURegion->ulRegionBaseAddress;
        ulMPURegionEnd = ulMPURegionStart + ulMPURegionLength - 1UL;

        if( ( ulBufferStart >= ulMPURegionStart ) &&
            ( ulBufferEnd <= ulMPURegionEnd ) &&
            ( ulBufferStart <= ulBufferEnd ) )
        {
            ulMPURegionAccessPermissions = xTaskMPURegion->ulRegionAttribute & portMPU_REGION_AP_BITMASK;

            if( ulAccessRequested == tskMPU_READ_PERMISSION ) /* RO. */
            {
                if( ( ulMPURegionAccessPermissions == portMPU_REGION_PRIV_RW_USER_RO ) ||
                    ( ulMPURegionAccessPermissions == portMPU_REGION_PRIV_RO_USER_RO ) ||
                    ( ulMPURegionAccessPermissions == portMPU_REGION_PRIV_RW_USER_RW ) )
                {
                    xAccessGranted = pdTRUE;
                }
            }
            else if( ( ulAccessRequested & tskMPU_WRITE_PERMISSION ) != 0UL ) /* W or RW. */
            {
                if( ulMPURegionAccessPermissions == portMPU_REGION_PRIV_RW_USER_RW )
                {
                    xAccessGranted = pdTRUE;
                }
            }
        }
    }

    return xAccessGranted;
}

/* ----------------------------------------------------------------------------------- */

/* PRIVILEGED_FUNCTION */
BaseType_t xPortIsAuthorizedToAccessBuffer( const void * pvBuffer,
                                            uint32_t ulBufferLength,
                                            uint32_t ulAccessRequested )
{
    BaseType_t xAccessGranted = pdFALSE;
    uint32_t ulRegionIndex;
    xMPU_SETTINGS * xTaskMPUSettings = NULL;

    if( prvPortSchedulerRunning == pdFALSE )
    {
        /* Grant access to all the memory before the scheduler is started. It is
         * necessary because there is no task running yet and therefore, we
         * cannot use the permissions of any task. */
        xAccessGranted = pdTRUE;
    }
    else
    {
        /* Calling task's MPU settings. */
        xTaskMPUSettings = xTaskGetMPUSettings( NULL );

        if( ( xTaskMPUSettings->ulTaskFlags & portTASK_IS_PRIVILEGED_FLAG ) == portTASK_IS_PRIVILEGED_FLAG )
        {
            /* Privileged tasks have access to all the memory. */
            xAccessGranted = pdTRUE;
        }
        else
        {
            for( ulRegionIndex = 0x0UL; ulRegionIndex < portTOTAL_NUM_REGIONS_IN_TCB; ulRegionIndex++ )
            {
                xAccessGranted = prvMPURegionAuthorizesBuffer( &( xTaskMPUSettings->xRegion[ ulRegionIndex ] ),
                                                               ( uint32_t ) pvBuffer,
                                                               ulBufferLength,
                                                               ulAccessRequested );

                if( xAccessGranted == pdTRUE )
                {
                    break;
                }
            }
        }
    }
    if( xAccessGranted == pdFALSE )
    {
        /* Check for static MPU regions set by DPL config */
        MpuP_RegionPerm xPerm = MpuP_RP_R;
        
        if( ( ulAccessRequested & tskMPU_WRITE_PERMISSION ) != 0UL )
        {
            xPerm = MpuP_RP_RW;
        }

        if( MpuP_isUserAuthorizedToAccessMemory( ( uint32_t ) pvBuffer, ulBufferLength, xPerm ) == 1U )
        {
            xAccessGranted = pdTRUE;
        }
    }

    return xAccessGranted;
}

/* ----------------------------------------------------------------------------------- */

#if( configENABLE_ACCESS_CONTROL_LIST == 1 )

/* PRIVILEGED_FUNCTION */
BaseType_t xPortIsAuthorizedToAccessKernelObject( int32_t lInternalIndexOfKernelObject )
{
    uint32_t ulAccessControlListEntryIndex, ulAccessControlListEntryBit;
    BaseType_t xAccessGranted = pdFALSE;
    const xMPU_SETTINGS * xTaskMpuSettings;

    if( prvPortSchedulerRunning == pdFALSE )
    {
        /* Grant access to all the kernel objects before the scheduler
         * is started. It is necessary because there is no task running
         * yet and therefore, we cannot use the permissions of any
         * task. */
        xAccessGranted = pdTRUE;
    }
    else
    {
        /* Calling task's MPU settings. */
        xTaskMpuSettings = xTaskGetMPUSettings( NULL );

        if( ( xTaskMpuSettings->ulTaskFlags & portTASK_IS_PRIVILEGED_FLAG ) == portTASK_IS_PRIVILEGED_FLAG )
        {
            xAccessGranted = pdTRUE;
        }
        else
        {
            ulAccessControlListEntryIndex = ( ( uint32_t ) lInternalIndexOfKernelObject
                                              / portACL_ENTRY_SIZE_BITS );
            ulAccessControlListEntryBit = ( ( uint32_t ) lInternalIndexOfKernelObject
                                            % portACL_ENTRY_SIZE_BITS );
                                            
            if( ( ( xTaskMpuSettings->ulAccessControlList[ ulAccessControlListEntryIndex ] ) &
                  ( 1U << ulAccessControlListEntryBit ) ) != 0UL )
            {
                xAccessGranted = pdTRUE;
            }
        }
    }

    return xAccessGranted;
}

#else

/* PRIVILEGED_FUNCTION */
BaseType_t xPortIsAuthorizedToAccessKernelObject( int32_t lInternalIndexOfKernelObject )
{
    ( void ) lInternalIndexOfKernelObject;

    /* If Access Control List feature is not used, all the tasks have
     * access to all the kernel objects. */
    return pdTRUE;
}

#endif /* #if ( configENABLE_ACCESS_CONTROL_LIST == 1 ) */

/* ----------------------------------------------------------------------------------- */

#if( configENABLE_ACCESS_CONTROL_LIST == 1 )

/* PRIVILEGED_FUNCTION */
void vPortGrantAccessToKernelObject( TaskHandle_t xInternalTaskHandle,
                                     int32_t lInternalIndexOfKernelObject )
{
    uint32_t ulAccessControlListEntryIndex, ulAccessControlListEntryBit;
    xMPU_SETTINGS * xTaskMpuSettings;

    ulAccessControlListEntryIndex = ( ( uint32_t ) lInternalIndexOfKernelObject
                                      / portACL_ENTRY_SIZE_BITS );
    ulAccessControlListEntryBit = ( ( uint32_t ) lInternalIndexOfKernelObject
                                    % portACL_ENTRY_SIZE_BITS );

    xTaskMpuSettings = xTaskGetMPUSettings( xInternalTaskHandle );

    xTaskMpuSettings->ulAccessControlList[ ulAccessControlListEntryIndex ] |= ( 1U << ulAccessControlListEntryBit );
}

#endif /* #if ( configENABLE_ACCESS_CONTROL_LIST == 1 ) */

/* ----------------------------------------------------------------------------------- */

#if( configENABLE_ACCESS_CONTROL_LIST == 1 )

/* PRIVILEGED_FUNCTION */
void vPortRevokeAccessToKernelObject( TaskHandle_t xInternalTaskHandle,
                                      int32_t lInternalIndexOfKernelObject )
{
    uint32_t ulAccessControlListEntryIndex, ulAccessControlListEntryBit;
    xMPU_SETTINGS * xTaskMpuSettings;

    ulAccessControlListEntryIndex = ( ( uint32_t ) lInternalIndexOfKernelObject
                                      / portACL_ENTRY_SIZE_BITS );
    ulAccessControlListEntryBit = ( ( uint32_t ) lInternalIndexOfKernelObject
                                    % portACL_ENTRY_SIZE_BITS );

    xTaskMpuSettings = xTaskGetMPUSettings( xInternalTaskHandle );

    xTaskMpuSettings->ulAccessControlList[ ulAccessControlListEntryIndex ] &= ~( 1U << ulAccessControlListEntryBit );
}

#endif /* #if ( configENABLE_ACCESS_CONTROL_LIST == 1 ) */

/* ----------------------------------------------------------------------------------- */

void vPortEndScheduler( void )
{
    /* nothing to do */
}

/* ----------------------------------------------------------------------------------- */

/* configCHECK_FOR_STACK_OVERFLOW is set to 1, so the application must provide an
 * implementation of vApplicationStackOverflowHook()
 */
void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    char * pcTaskName )
{
    DebugP_logError("[FreeRTOS] Stack overflow detected for task [%s]", pcTaskName);
    DebugP_assertNoLog(0);
}

PRIVILEGED_DATA static StaticTask_t xIdleTaskTCB;
PRIVILEGED_DATA static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ]
    __attribute__( ( aligned( configMINIMAL_STACK_SIZE * 0x4U ) ) );
/* configSUPPORT_STATIC_ALLOCATION is set to 1, so the application must provide an
 * implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
 * used by the Idle task.
 */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    configSTACK_DEPTH_TYPE *pulIdleTaskStackSize )
{
    /* Pass out a pointer to the StaticTask_t structure in which the Idle task’s
     * state will be stored.
     */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task’s stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configMINIMAL_STACK_SIZE is specified in words, not bytes.
     */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

PRIVILEGED_DATA static StaticTask_t xTimerTaskTCB;
PRIVILEGED_DATA static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ]
    __attribute__( ( aligned( configTIMER_TASK_STACK_DEPTH * 0x4U ) ) );
/* configSUPPORT_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
 * application must provide an implementation of vApplicationGetTimerTaskMemory()
 * to provide the memory that is used by the Timer service task.
 */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     configSTACK_DEPTH_TYPE *pulTimerTaskStackSize )
{
    /* Pass out a pointer to the StaticTask_t structure in which the Timer
     * task’s state will be stored.
     */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task’s stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configTIMER_TASK_STACK_DEPTH is specified in words, not bytes.
     */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

/* This function is called when configUSE_IDLE_HOOK is 1 in FreeRTOSConfig.h */
void vApplicationIdleHook( void )
{
    void vApplicationLoadHook();

    vApplicationLoadHook();

    __asm__ __volatile__ ("wfi"   "\n\t": : : "memory");
}
