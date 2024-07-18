/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

#include <board/flash.h>

extern Flash_Config gFlashConfig[];
extern uint32_t gFlashConfigNum;

Flash_Attrs *Flash_getAttrs(uint32_t instanceId)
{
    Flash_Attrs *attrs = NULL;
    Flash_Config *config = NULL;

    if(instanceId < gFlashConfigNum)
    {
        config = &gFlashConfig[instanceId];
        attrs = config->attrs;
    }
    return attrs;
}

Flash_Handle Flash_open(uint32_t instanceId, Flash_Params *params)
{
    Flash_Config *config = NULL;

    if(instanceId < gFlashConfigNum)
    {
        config = &gFlashConfig[instanceId];
        if(config->fxns && config->fxns->openFxn)
        {
            int32_t status;

            status = config->fxns->openFxn(config, params);
            if(status != SystemP_SUCCESS)
            {
                config = NULL;
            }
        }
    }
    return config;
}

void Flash_close(Flash_Handle handle)
{
    Flash_Config *config = (Flash_Config*)handle;

    if(config && config->fxns && config->fxns->closeFxn)
    {
        config->fxns->closeFxn(config);
    }
}

Flash_Handle Flash_getHandle(uint32_t instanceId)
{
    Flash_Config *config = NULL;

    if(instanceId < gFlashConfigNum)
    {
        config = &gFlashConfig[instanceId];
    }

    return config;
}

int32_t Flash_read(Flash_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len)
{
    Flash_Config *config = (Flash_Config*)handle;
    int32_t status = SystemP_FAILURE;

    if(config && config->fxns && config->fxns->readFxn)
    {
        offset += config->rwOffset;
        status = config->fxns->readFxn(config, offset, buf, len);
    }
    return status;
}

int32_t Flash_write(Flash_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len)
{
    Flash_Config *config = (Flash_Config*)handle;
    int32_t status = SystemP_FAILURE;

    if(config && config->fxns && config->fxns->writeFxn)
    {
        offset += config->rwOffset;
        status = config->fxns->writeFxn(config, offset, buf, len);
    }
    return status;
}

int32_t Flash_eraseBlk(Flash_Handle handle, uint32_t blockNum)
{
    Flash_Config *config = (Flash_Config*)handle;
    int32_t status = SystemP_FAILURE;

    if(config && config->fxns && config->fxns->eraseFxn)
    {
        status = config->fxns->eraseFxn(config, blockNum);
    }
    return status;
}

int32_t Flash_eraseSector(Flash_Handle handle, uint32_t sectorNum)
{
    Flash_Config *config = (Flash_Config*)handle;
    int32_t status = SystemP_FAILURE;

    if(config && config->fxns && config->fxns->eraseFxn)
    {
        status = config->fxns->eraseSectorFxn(config, sectorNum);
    }
    return status;
}

int32_t Flash_reset(Flash_Handle handle)
{
    Flash_Config *config = (Flash_Config*)handle;
    int32_t status = SystemP_FAILURE;

    if(config && config->fxns && config->fxns->resetFxn)
    {
        status = config->fxns->resetFxn(config);
    }
    return status;
}

int32_t Flash_blkPageToOffset(Flash_Handle handle, uint32_t *offset, uint32_t block, uint32_t page)
{
    Flash_Config *config = (Flash_Config*)handle;
    int32_t status = SystemP_FAILURE;

    if(config && offset)
    {
        uint32_t blockCount = config->attrs->blockCount;
        uint32_t pageSize   = config->attrs->pageSize;
        uint32_t pageCount  = config->attrs->pageCount;

        status = SystemP_SUCCESS;
        *offset = 0;

        if( block > blockCount || page > pageCount)
        {
            status = SystemP_FAILURE;
        }
        if(status == SystemP_SUCCESS)
        {
            *offset =   (block * (pageCount * pageSize)) + (page * pageSize);
        }
    }
    return status;
}

int32_t Flash_offsetToBlkPage(Flash_Handle handle, uint32_t  offset, uint32_t *block, uint32_t *page)
{
    Flash_Config *config = (Flash_Config*)handle;
    int32_t status = SystemP_FAILURE;

    if(config && block && page)
    {
        uint32_t blockCount = config->attrs->blockCount;
        uint32_t pageSize   = config->attrs->pageSize;
        uint32_t pageCount  = config->attrs->pageCount;
        uint32_t blockSize  = config->attrs->blockSize;
        uint32_t leftover;

        status = SystemP_SUCCESS;

        offset += config->rwOffset;

        *block 	  = offset / blockSize;
        leftover  = offset % blockSize;
        *page 	  = leftover / pageSize;
        if (leftover % pageSize)
        {
            /* All writes must be page aligned for now */
            status = SystemP_FAILURE;
        }
        if (*block > blockCount || *page > pageCount)
        {
            /* beyond limits for this flash */
            status = SystemP_FAILURE;
        }
    }
    return status;
}

int32_t Flash_sectorPageToOffset(Flash_Handle handle, uint32_t *offset, uint32_t sector, uint32_t page)
{
    Flash_Config *config = (Flash_Config*)handle;
    int32_t status = SystemP_FAILURE;

    if(config && offset)
    {
        uint32_t sectorCount = config->attrs->sectorCount;
        uint32_t pageSize    = config->attrs->pageSize;
        uint32_t pageCount   = config->attrs->pageCount;

        status = SystemP_SUCCESS;
        *offset = 0;

        if( sector > sectorCount || page > pageCount)
        {
            status = SystemP_FAILURE;
        }
        if(status == SystemP_SUCCESS)
        {
            *offset =   (sector * (pageCount * pageSize)) + (page * pageSize);
        }
    }
    return status;
}

int32_t Flash_offsetToSectorPage(Flash_Handle handle, uint32_t  offset, uint32_t *sector, uint32_t *page)
{
    Flash_Config *config = (Flash_Config*)handle;
    int32_t status = SystemP_FAILURE;

    if(config && sector && page)
    {
        uint32_t sectorCount = config->attrs->sectorCount;
        uint32_t pageSize    = config->attrs->pageSize;
        uint32_t sectorSize  = config->attrs->sectorSize;
        uint32_t pageCount   = sectorSize / pageSize;
        uint32_t leftover;

        status = SystemP_SUCCESS;

        offset += config->rwOffset;

        *sector   = offset / sectorSize;
        leftover  = offset % sectorSize;
        *page 	  = leftover / pageSize;
        if (leftover % pageSize)
        {
            /* All writes must be page aligned for now */
            status = SystemP_FAILURE;
        }
        if (*sector > sectorCount || *page > pageCount)
        {
            /* beyond limits for this flash */
            status = SystemP_FAILURE;
        }
    }
    return status;
}

uint32_t Flash_getPhyTuningOffset(Flash_Handle handle)
{
    Flash_Config *config = (Flash_Config*)handle;
    uint32_t offset = 0xFFFFFFFFU;

    if(config)
    {
        offset = config->attrs->phyTuningOffset;
    }

    return offset;
}

int32_t Flash_enableDacMode(Flash_Handle handle)
{
    Flash_Config *config = (Flash_Config*)handle;
    int32_t status = SystemP_FAILURE;

    if(config && config->fxns && config->fxns->enableDacModeFxn)
    {
        status = config->fxns->enableDacModeFxn(config);
    }
    return status;
}

int32_t Flash_disableDacMode(Flash_Handle handle)
{
    Flash_Config *config = (Flash_Config*)handle;
    int32_t status = SystemP_FAILURE;

    if(config && config->fxns && config->fxns->disableDacModeFxn)
    {
        status = config->fxns->disableDacModeFxn(config);
    }
    return status;
}