/*!
 *  \file CMN_mem.c
 *
 *  \brief
 *  Dynamic memory allocation trace.
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2022-10-27
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

#if (defined CMN_MEM_TRACE) && (1==CMN_MEM_TRACE)

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>

#include <osal.h>
#include <osal_error.h>

#include <CMN_mem.h>

static uint32_t cmn_mem_message_id_s;
static uint32_t cmn_mem_overall_allocated_s;

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  OSAL callback function of OSAL_MEMORY_calloc function.
 *
 *  \detail
 *  CMN_MEM_traceInit needs to be called to register this callback function.
 *
 *  <!-- Parameters and return values: -->
 *  \param[in]  pContext_p     Context pointer.
 *  \param[in]  nmemb_p        Memory allocation unit size in bytes.
 *  \param[in]  size_p         Object size in memory allocation units.
 *  \param[in]  pPtr_p         Pointer to dynamically allocated memory.
 *
 *
 */
static void CMN_MEM_traceCalloc(void* pContext_p, size_t nmemb_p, size_t size_p, void* pPtr_p)
{
    uint32_t* ptrSize   = NULL;
    uint32_t  allocSize = 0;

    ptrSize = (uint32_t*)((uint8_t*) pPtr_p - CMN_MEM_DESCRIPTOR_POSITION);
    allocSize = ((*ptrSize) & CMN_MEM_DESCRIPTOR_SIZE_MASK) + CMN_MEM_DESCRIPTOR_POSITION;

    cmn_mem_overall_allocated_s += allocSize;

    OSAL_printf("%10d. Calloc: ptr: 0x%8X, len: %5d. Allocated size: %5d. Overall allocated: %d.\r\n", cmn_mem_message_id_s++, pPtr_p, nmemb_p*size_p, allocSize, cmn_mem_overall_allocated_s);
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  OSAL callback function of OSAL_MEMORY_free function.
 *
 *  \detail
 *  CMN_MEM_traceInit needs to be called to register this callback function.
 *
 *  <!-- Parameters and return values: -->
 *  \param[in]  pContext_p     Context pointer.
 *  \param[in]  pPtr_p         Pointer to dynamically allocated memory.
 *
 *
 */
static void CMN_MEM_traceFree (void *pContext_p, void* pPtr_p)
{
    uint32_t* ptrSize   = NULL;
    uint32_t  allocSize = 0;
    bool      allocated = false;

    ptrSize   = (uint32_t*)((uint8_t*) pPtr_p - CMN_MEM_DESCRIPTOR_POSITION);
    allocSize = ((*ptrSize) & CMN_MEM_DESCRIPTOR_SIZE_MASK) + CMN_MEM_DESCRIPTOR_POSITION;
    allocated = (bool)((*ptrSize) & CMN_MEM_DESCRIPTOR_USED_FLAG_MASK);

    if (allocated == true)
    {
        cmn_mem_overall_allocated_s -= allocSize;

        OSAL_printf("%10d. Free:   ptr: 0x%8X, len: %5d. Allocated size: %5d. Overall allocated: %d.\r\n", cmn_mem_message_id_s++, pPtr_p, 0, allocSize, cmn_mem_overall_allocated_s);
    }
    else
    {
        OSAL_printf("%10d. WARN:   ptr: 0x%8X already deallocated.\r\n", cmn_mem_message_id_s++, pPtr_p);
    }
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Registers OSAL_MEMORY_calloc and OSAL_MEMORY_free functions to be able trace dynamic memory allocation.
 *
 *  \detail
 *  CMN_MEM_traceCalloc and CMN_MEM_traceFree functions need to be defined.
 *
 */
void CMN_MEM_traceInit (void)
{
    cmn_mem_message_id_s = 0;
    cmn_mem_overall_allocated_s = 0;

    OSAL_MEMORY_traceCallocRegister (CMN_MEM_traceCalloc, NULL);
    OSAL_MEMORY_traceFreeRegister (CMN_MEM_traceFree, NULL);
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Checks HEAP memory for status and printouts the report.
 *
 *  \detail
 *  Use in case when OSAL_MEMORY_calloc will return NULL or in other strange circumstances.
 *
 *  <!-- Parameters and return values: -->
 *  \param[in]  heapStart_p    HEAP start address.
 *  \param[in]  size_p         HEAP size.
 *
 */
void CMN_MEM_traceHeapCheck (uint32_t heapStart_p, uint32_t size_p)
{
    uint32_t  ptr       = heapStart_p + CMN_MEM_DESCRIPTOR_POSITION;
    uint32_t* ptrSize   = NULL;
    uint32_t  allocSize = 0;
    bool      allocated = false;

    OSAL_printf("\r\n");
    OSAL_printf("****************************************************************************\r\n");
    OSAL_printf("*                               HEAP check                                 *\r\n");
    OSAL_printf("****************************************************************************\r\n");
    OSAL_printf("\r\n");

    while((ptr - heapStart_p) < size_p)
    {
        ptrSize   = (uint32_t*)((uint8_t*) ptr - CMN_MEM_DESCRIPTOR_POSITION);
        allocSize = ((*ptrSize) & CMN_MEM_DESCRIPTOR_SIZE_MASK) + CMN_MEM_DESCRIPTOR_POSITION;
        allocated = (bool)((*ptrSize) & CMN_MEM_DESCRIPTOR_USED_FLAG_MASK);

        OSAL_printf("%10d. ptr: 0x%8X, len: %6d. allocated: %s.\r\n", cmn_mem_message_id_s++, ptr, allocSize, allocated ? "yes" : "no");

        ptr += allocSize;

        if ((ptr - CMN_MEM_DESCRIPTOR_POSITION) > heapStart_p + size_p)
        {
            OSAL_printf("\r\n");
            OSAL_printf("%10d. HEAP overflow error.\r\n", cmn_mem_message_id_s++);
            break;
        }
    }

    OSAL_printf("\r\n");
    OSAL_printf("****************************************************************************\r\n");
}
#endif // (defined CMN_MEM_TRACE) && (1==CMN_MEM_TRACE)
