/*
 *  Copyright (C) 2021-2023 Texas Instruments Incorporated
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

#include <string.h>
#include "psram_gpmc.h"

#define GPMC_DATA_BASE_ADDRESS 0x68000000

Psram_Fxns gPsramGpmcFxns = {
    .openFxn = Psram_gpmcOpen,
    .closeFxn = Psram_gpmcClose,
    .readFxn = Psram_gpmcRead,
    .writeFxn = Psram_gpmcWrite,
};

int32_t Psram_gpmcOpen(Psram_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    Psram_Attrs *attrs = config->attrs;

    GPMC_Handle gpmcHandle = GPMC_getHandle(attrs->driverInstance);

    if(gpmcHandle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        /* Set device type for GPMC peripherial. NANDLIKE or NORLIKE*/
        status +=GPMC_setDeviceType(gpmcHandle);
    }

    if(status == SystemP_SUCCESS)
    {
        /* Set device width.*/
        status += GPMC_setDeviceSize(gpmcHandle);
    }

    if(status == SystemP_SUCCESS)
    {
        /* Configure GPMC timing parameters for PSRAM. */
        status += GPMC_configureTimingParametersPsram(gpmcHandle);
    }

    return status;

}

int32_t Psram_gpmcWrite (Psram_Config *config, uint32_t offset,
                                    uint8_t *buf, uint32_t len)

{
    int32_t status = SystemP_FAILURE;
    uint32_t size =  len;

    /* Input parameter validation. */
    if(config != NULL)
    {
        if(offset + len > config->attrs->psramSize)
        {
            status = SystemP_FAILURE;
        }
        uint32_t baseAddress = GPMC_DATA_BASE_ADDRESS;

        volatile uint16_t *pSrc = (volatile uint16_t *)buf;
        volatile uint16_t *pDst = (volatile uint16_t *)(offset + baseAddress);

        while (size != 0U)
        {
            *pDst = *pSrc;
            pSrc++;
            pDst++;
            size -= 2;
        }

        status = SystemP_SUCCESS;
    }

    return status;
}

int32_t Psram_gpmcRead (Psram_Config *config, uint32_t offset, uint8_t *buf, uint32_t len)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t size =  len;

    if(config != NULL)
    {

        if(offset + len > config->attrs->psramSize)
        {
            status = SystemP_FAILURE;
        }
        uint32_t baseAddress = GPMC_DATA_BASE_ADDRESS;

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

    return status;
}

void Psram_gpmcClose(Psram_Config *config)
{
    return;
}