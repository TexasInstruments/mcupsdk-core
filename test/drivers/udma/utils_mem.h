/*
 *  Copyright (c) Texas Instruments Incorporated 2018
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

/**
 *  \file utils_mem.h
 *
 *  \brief Memory allocator API.
 */

#ifndef UTILS_MEM_H_
#define UTILS_MEM_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define UTILS_MEM_HEAP_ID_MSMC      (0U)
#define UTILS_MEM_HEAP_ID_DDR       (1U)
/* L2 for CC6x, L2 for C7x, DDR for A72, R5 OCMC (MSRAM) */
#define UTILS_MEM_HEAP_ID_INTERNAL  (2U)
#define UTILS_MEM_HEAP_ID_OSPI      (3U)
#define UTILS_MEM_HEAP_NUM          (4U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  struct Utils_MemHeapStatus
 *  \brief He.
 */
typedef struct
{
    uint32_t freeSysHeapSize;
    /**< Current system heap free space. */
    uint32_t freeBufHeapSize[UTILS_MEM_HEAP_NUM];
    /**< Current buffer heap free space. */
} Utils_MemHeapStatus;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief One time system init of memory allocator
 *
 *  Should be called by application before using allocate APIs
 *
 *  \return 0 on sucess, else failure
 */
int32_t Utils_memInit(void);

/**
 *  \brief One time system de-init of memory allocator
 *
 *  Should be called by application at system de-init
 *
 *  \return 0 on sucess, else failure
 */
int32_t Utils_memDeInit(void);

/**
 *  \brief Allocate memory from memory pool
 *
 *  \param heapId [IN] Heap ID
 *  \param size   [IN] size in bytes
 *  \param align  [IN] alignment in bytes
 *
 *  \return NULL or error, else memory pointer
 */
void *Utils_memAlloc(uint32_t heapId, uint32_t size, uint32_t align);

/**
 *  \brief Free previously allocate memory pointer
 *
 *  \param heapId   [IN] Heap ID
 *  \param addr     [IN] memory pointer to free
 *  \param size     [IN] size of memory pointed to by the memory pointer
 *
 *  \return 0 on sucess, else failure
 */
int32_t Utils_memFree(uint32_t heapId, void *addr, uint32_t size);

/**
 *  \brief Control if allocated buffer needs to be cleared to 0
 *
 *  By default allocated buffer will not be cleared to 0
 *
 *  \param enable   [IN] TRUE: clear allocated buffer,
 *                       FALSE: do not clear allocated buffer
 *
 *  \return 0 on sucess, else failure
 */
int32_t Utils_memClearOnAlloc(Bool enable);

/**
 *  Utils_memGetHeapStat
 *  \brief Returns the current status (free size) of the various heaps used.
 *
 *  This function could be called before calling the module init function or
 *  after module deinit function.
 *
 *  \param heapStat [OUT]   Status filled by the function.
 *
 */
void Utils_memGetHeapStat(Utils_MemHeapStatus *heapStat);

/**
 *  Utils_memCheckHeapStat
 *  \brief Checks the current status of each heap with the value passed.
 *
 *  This returns an error if the values of each of the heap doesn't match.
 *  This function could be called before calling the module init function or
 *  after module deinit function.
 *
 *  \param heapStat [IN]    Older status to be compared with the current status.
 *
 *  \return                 Returns 0 if the heap sizes match else return
 *                          failure.
 */
int32_t Utils_memCheckHeapStat(const Utils_MemHeapStatus *heapStat);

/**
 *  \brief Returns the system heap free memory (in bytes)
 */
uint32_t Utils_memGetSystemHeapFreeSpace(void);

/**
 *  \brief Returns the buffer heap free memory (in bytes)
 *  \param heapId [IN] Heap ID
 */
uint32_t Utils_memGetBufferHeapFreeSpace(uint32_t heapId);

#ifdef __cplusplus
}
#endif

#endif  /* #define UTILS_MEM_H_ */
