/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

#ifndef __FOTAAgent__H_
#define __FOTAAgent__H_

/* FOTA Agent Middleware */

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include <stdlib.h>
#include <string.h>
#include <drivers/flsopskd.h>
#include <middleware/tiELFuParser/tielfup32.h>
#include <drivers/spinlock.h>

#define CHUNK_SIZE 4096
#define LOCK_NUM0   (0U)
#define LOCK_NUM1   (1U)

typedef struct
{
    uint8_t chunk[CHUNK_SIZE];
    uint32_t chunkOffset;
    uint32_t currWriteOffset;
    uint32_t prevWriteOffset;
    uint32_t flashBaseOffset;
    uint32_t isXip;
    ELFUP_ELFPH pht[20];
    ELFUP_Handle elfuph;
    FLSOPSKD_handle FLSOPSKDhandle;
} FOTAAgent_handle;

/**
 * @brief Initilize the FOTA Agent and the driver 
 * 
 * @param pHandle Pointer to handle
 * @return int32_t SystemP_SUCCESS if everything okay
 */
int32_t FOTAAgent_init(FOTAAgent_handle *pHandle);

/**
 * @brief Starts Fota write operation.
 * 
 * @param baseAddr Flash Base Address
 * @param wrOffset Flash Write Offset
 * @param isXip variable to check whether file is Xip
 */
void FOTAAgent_writeStart(FOTAAgent_handle *pHandle,uint32_t baseAddr,uint32_t wrOffset,uint32_t isXip);

/**
 * @brief Sends write scheduling request to hardware IP. 
 *
 * This function will schedule Writes in Chunks   
 * 
 * @param pHandle handle
 * @param buf pointer to source buffer
 * @param size size of the buffer to write
 * @return SystemP_SUCCESS if everything okay
 */
int32_t FOTAAgent_writeUpdate(FOTAAgent_handle *pHandle,uint8_t *buf,uint32_t size);

/**
 * @brief End Fota Write operation. 
 * 
 * @param pHandle handle
 * @return SystemP_SUCCESS if everything okay
 */
int32_t FOTAAgent_writeEnd(FOTAAgent_handle *pHandle);


#ifdef __cplusplus
}
#endif

#endif /* __FOTAAgent__H_ */