
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


#include "fss.h"

#define HWREG(x)                                                               \
        (*((volatile uint32_t *)(x)))

#define MSS_OSPI_BOOT_CONFIG_MASK              (0x840ul)
#define MSS_OSPI_BOOT_CONFIG_SEG               (0x844ul)

int32_t FSS_addressBitMask(FSS_Handle handle, uint32_t bitMask, uint8_t segment)
{
    int32_t ret = SystemP_FAILURE;

    if(handle != NULL)
    {
        FSS_Config *config = (FSS_Config *)handle;
        if(bitMask != 0)
        {
            HWREG(config->ipBaseAddress + MSS_OSPI_BOOT_CONFIG_MASK) = ~(bitMask >> 12);
            HWREG(config->ipBaseAddress + MSS_OSPI_BOOT_CONFIG_SEG) = (segment * (bitMask + 1)) >> 12;
        }
        else
        {
            HWREG(config->ipBaseAddress + MSS_OSPI_BOOT_CONFIG_MASK) = 0;
            HWREG(config->ipBaseAddress + MSS_OSPI_BOOT_CONFIG_SEG) = 0;
        }
        ret = SystemP_SUCCESS;
    }

    return ret;
}

int32_t FSS_selectRegionA(FSS_Handle handle)
{
    int32_t ret = SystemP_FAILURE;
    if(handle != NULL)
    {
        FSS_Config *config = (FSS_Config *)handle;
        if((config->extFlashSize & (config->extFlashSize - 1)) == 0)
        {
            ret = FSS_addressBitMask(handle, (config->extFlashSize / 2) - 1, 0);
        }
    }
    return ret;
}

int32_t FSS_selectRegionB(FSS_Handle handle)
{
    int32_t ret = SystemP_FAILURE;
    if(handle != NULL)
    {
        FSS_Config *config = (FSS_Config *)handle;
        if((config->extFlashSize & (config->extFlashSize - 1)) == 0)
        {
            ret = FSS_addressBitMask(handle, (config->extFlashSize / 2) - 1, 1);
        }
    }
    return ret;
}

int32_t FSS_disableAddressRemap(FSS_Handle handle)
{
    return FSS_addressBitMask(handle, 0, 0);
}
