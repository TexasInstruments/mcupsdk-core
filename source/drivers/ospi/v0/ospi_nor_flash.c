/*
 *  Copyright (C) 2021-24 Texas Instruments Incorporated
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
 *  \file ospi_nor_flash.c
 *
 *  \brief File containing generic NOR xSPI flash driver APIs.
 *
 */

#include <drivers/ospi.h>
#include <drivers/ospi/v0/lld/ospi_lld.h>

void OSPI_norFlashSetCmds(uint8_t rdCmd, uint8_t wrCmd, uint8_t eraseCmd)
{
    OSPI_lld_norFlashSetCmds(rdCmd,wrCmd,eraseCmd);
}

int32_t OSPI_norFlashInit1s1s1s(OSPI_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;
    if(NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status = OSPI_lld_norFlashInit1s1s1s(hOspi);
    }

    return status;
}

int32_t OSPI_norFlashReadId(OSPI_Handle handle, uint32_t *manufacturerId, uint32_t *deviceId)
{
    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;
    if(NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status = OSPI_lld_norFlashReadId(hOspi,manufacturerId,deviceId);
    }
    return status;
}

int32_t OSPI_norFlashWrite(OSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len)
{
    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;
    if(NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status = OSPI_lld_norFlashWrite(hOspi,offset,buf,len);
    }
   
    return status;
}

int32_t OSPI_norFlashRead(OSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len)
{
    int32_t status = SystemP_SUCCESS;

    OSPILLD_Handle hOspi;
    if(NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status = OSPI_lld_norFlashRead(hOspi,offset,buf,len);
    }
   
    return status;
}

int32_t OSPI_norFlashReadSfdp(OSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len)
{
    int32_t status = SystemP_SUCCESS;

    OSPILLD_Handle hOspi;
    if(NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status = OSPI_lld_norFlashReadSfdp(hOspi,offset,buf,len);
    }

    return status;
}

int32_t OSPI_norFlashErase(OSPI_Handle handle, uint32_t address)
{
    int32_t status = SystemP_SUCCESS;

    OSPILLD_Handle hOspi;
    if(NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status = OSPI_lld_norFlashErase(hOspi,address);
    }

    return status;
}

