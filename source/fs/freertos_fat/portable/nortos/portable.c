/*
 * FreeRTOS+FAT V2.3.3
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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
 * Copyright (C) 2021 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  \file portable.c
 *
 *  \brief NO-RTOS portable source file for FreeRTOS includes in FreeRTOS+FAT module.
 *
 */

#include "FreeRTOS.h"
#include "task.h"

#include "ff_headers.h"

void *pvTaskGetThreadLocalStoragePointer(TaskHandle_t xTaskToQuery, BaseType_t xIndex)
{
	return 0U;
}

void *vTaskSetThreadLocalStoragePointer(TaskHandle_t xTaskToSet, BaseType_t xIndex, void *pvValue)
{
	return 0U;	
}

void vTaskSuspendAll(void)
{

}

void xTaskResumeAll(void)
{
	
}

BaseType_t FF_TrySemaphore( void * pxSemaphore, uint32_t ulTime_ms )
{
    return 1;
}

void FF_PendSemaphore( void * pxSemaphore )
{
}

void FF_ReleaseSemaphore( void * pxSemaphore )
{
}

void FF_Sleep( uint32_t ulTime_ms )
{
}

void FF_DeleteEvents( FF_IOManager_t * pxIOManager )
{
}

BaseType_t FF_CreateEvents( FF_IOManager_t * pxIOManager )
{
	return pdTRUE;
}

void FF_LockDirectory( FF_IOManager_t *pxIOManager )
{
}

void FF_UnlockDirectory( FF_IOManager_t *pxIOManager )
{
}

int FF_Has_Lock( FF_IOManager_t *pxIOManager, uint32_t aBits )
{
    return pdTRUE;
}

void FF_Assert_Lock( FF_IOManager_t *pxIOManager, uint32_t aBits )
{
}

void FF_LockFAT( FF_IOManager_t *pxIOManager )
{
}

void FF_UnlockFAT( FF_IOManager_t *pxIOManager )
{
}

BaseType_t FF_BufferWait( FF_IOManager_t * pxIOManager, uint32_t xWaitMS )
{
    return pdTRUE;
}

void FF_BufferProceed( FF_IOManager_t *pxIOManager )
{
}
