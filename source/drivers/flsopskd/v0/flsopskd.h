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

#ifndef __FLSOPSKD__H__
#define __FLSOPSKD__H__

#include <stdio.h>
#include <stdbool.h>
#include <kernel/dpl/DebugP.h>
#include <stdlib.h>
#include <string.h>
#include <drivers/hw_include/cslr_flsopskd.h>
#include "fota_fw_arr.h"


#define FLASH_PAGE_SIZE (256U)
#define FLASH_SECTOR_SIZE (128*1024U)

/**
 * FLSOPSKD stands for **FL**ash **OP**erations Scheduler (SKD)
 */
#define  CSLR_R5FSS0_CORE1_FOTA_WR_COMPL         (214U)

extern volatile uint32_t maxSingleSectorEraseTime;
extern volatile uint32_t maxSinglePageProgramTime;

typedef struct __tag__flash_operation_scheduler_handle
{
    int8_t pollEnable; /**< Enable polling mode */
} FLSOPSKD_handle;

/**
 * @brief Initilize the IP and the driver 
 * 
 * By default polling mode is enabled. 
 * 
 * @param pHandle Pointer to handle
 * @return int32_t SystemP_SUCCESS if everything okay
 */
int32_t FLSOPSKD_Init(FLSOPSKD_handle *pHandle);

/**
 * @brief Sends write scheduling request to hardware IP. 
 *
 * This function will send write request to 8051 IP.  
 * 
 * @param pHandle handle
 * @param destAddr Flash address to which to write this buffer
 * @param pSrcBuffer pointer to source buffer
 * @param wrSize size of the buffer to write
 * @return SystemP_SUCCESS if everything okay
 */
int32_t FLSOPSKD_Write(FLSOPSKD_handle *pHandle, uint32_t destAddr, uint8_t * pSrcBuffer, uint32_t wrSize);

/**
 * @brief Send erase scheduling request.
 * 
 * This API will erase a block of the flash.
 * 
 * @param pHandle handle
 * @param offset block offset.
 * @return SystemP_SUCCESS if everything okay
 */
int32_t FLSOPSKD_Erase(FLSOPSKD_handle *pHandle, uint32_t offset);

/**
 * @brief Request to to read data from flash.
 * 
 * Reads the data from srcAddress to destArry and the 
 * size that is written 
 * 
 * @param pHandle handle
 * @param srcAddress source address 
 * @param destArry destination array
 * @param bytesToRead bytes to read
 * @return SystemP_SUCCESS if everything okay
 */
int32_t FLSOPSKD_STIGRead(FLSOPSKD_handle *pHandle, uint32_t srcAddress, uint8_t *destArry, size_t bytesToRead);

/**
 * @brief Get 8051 Firmware stack
 * 
 * @param pHandle handle
 * @param pVersion pointer to memory to save version info.
 * @return int32_t 
 */
int32_t FLSOPSKD_Get8051Version(FLSOPSKD_handle *pHandle, volatile uint32_t *pVersion);

int32_t FLSOPSKD_GetSectorSize(FLSOPSKD_handle *pHandle, volatile uint32_t *pSecSize);

#endif