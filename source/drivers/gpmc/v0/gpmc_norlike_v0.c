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

/**
 *  \file gpmc_norlike_v0.c
 *
 *  \brief File containing GPMC norlike Driver APIs implementation for version V0.
 *
 */

#include <string.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/gpmc.h>
#include <drivers/elm.h>
#include <drivers/gpmc/v0/dma/gpmc_dma.h>
#include "gpmc_priv_v0.h"

#define NORLIKE_BUSWIDTH_8BITS               (0U)
#define NORLIKE_BUSWIDTH_16BITS              (1U)
#define NORLIKE_BUSWIDTH_32BITS              (2U)

uint8_t *GPMC_norMakeAddr(uint8_t busWidth,uint32_t blkAddr,uint32_t offset)
{
    uint32_t addr;

    if(busWidth == NORLIKE_BUSWIDTH_8BITS)
    {
        addr = blkAddr + offset;
    }
    else
    {
        addr = blkAddr + (offset << 1);
    }
    return ((uint8_t *) addr);
}

void GPMC_norMakeCmd(uint8_t busWidth, uint32_t cmd, void *cmdBuf)
{
    uint32_t i;
    uint8_t *cmdPtr = (uint8_t *)cmdBuf;

    for (i = (1 << busWidth); i > 0; i--)
    {
        *cmdPtr = (i & ((1 << busWidth) - 1)) ? 0x00 : cmd;
        cmdPtr++;
    }
}

int32_t GPMC_norWriteData(GPMC_Handle handle,uint32_t offset,
                                         uint8_t *buf, uint32_t len)

{
    int32_t status = SystemP_FAILURE;
    uint32_t size = len;

    if(handle != NULL)
    {
        const GPMC_HwAttrs *hwAttrs = ((GPMC_Config*)handle)->attrs;
        GPMC_Object *obj = ((GPMC_Config*)handle)->object;
        uint32_t devSize = obj->params.devSize;
        uint32_t baseAddress = hwAttrs->dataBaseAddr;

        if(devSize == CSL_GPMC_CONFIG1_DEVICESIZE_EIGHTBITS)
        {
            volatile uint8_t *pSrc = (volatile uint8_t *)buf;
            volatile uint8_t *pDst = (volatile uint8_t *)(offset + baseAddress);
            while (size != 0U)
            {
                *pDst = *pSrc;
                pSrc++;
                pDst++;
                size--;
            }
        }
        else
        {
            volatile uint16_t *pSrc = (volatile uint16_t *)buf;
            volatile uint16_t *pDst = (volatile uint16_t *)(offset + baseAddress);
            while (size != 0U)
            {
                *pDst = *pSrc;
                pSrc++;
                pDst++;
                if(size==1)
                {
                    size--;
                }
                else
                {
                    size -= 2;
                }
            }
        }

        status = SystemP_SUCCESS;
    }

    return status;

}

int32_t GPMC_norReadData(GPMC_Handle handle, uint32_t offset,
                                  uint8_t *buf, uint32_t len)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t size =  len;

    if(handle != NULL)
    {
        const GPMC_HwAttrs *hwAttrs = ((GPMC_Config*)handle)->attrs;
        GPMC_Object *obj = ((GPMC_Config*)handle)->object;
        uint32_t devSize = obj->params.devSize;
        uint32_t baseAddress = hwAttrs->dataBaseAddr;

        if(devSize == CSL_GPMC_CONFIG1_DEVICESIZE_EIGHTBITS)
        {
            volatile uint8_t *pDst = (volatile uint8_t *)buf;
            volatile uint8_t *pSrc = (volatile uint8_t *)(offset + baseAddress);
            while (size != 0U)
            {
                *pDst = *pSrc;
                pSrc++;
                pDst++;
                size--;
            }
        }
        else
        {
            volatile uint16_t *pSrc = (volatile uint16_t *)(offset+baseAddress);
            volatile uint16_t *pDst = (volatile uint16_t *)buf;
            uint32_t  remain = size & 0x1;

            if (remain != 0U)
            {
                size = size - remain + 2U;
            }

            while (size != 0U)
            {
                *pDst = *pSrc;
                pSrc++;
                pDst++;
                size -= 2U;
            }
        }
    }

    else{
        status = SystemP_FAILURE;
    }

    return status;
}