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

#include <string.h>
#include "psram_gpmc.h"

#define GPMC_DATA_BASE_ADDRESS 0x68000000

static int32_t Psram_gpmcOpen(Ram_Config *config);
static void Psram_gpmcClose(Ram_Config *config);
static int32_t Psram_gpmcWrite (Ram_Config *config, uint32_t offset,
                                    uint8_t *buf, uint32_t len);
static int32_t Psram_gpmcRead (Ram_Config *config, uint32_t offset, uint8_t *buf, uint32_t len);

Ram_Fxns gPsramGpmcFxns = {
    .openFxn = Psram_gpmcOpen,
    .closeFxn = Psram_gpmcClose,
    .readFxn = Psram_gpmcRead,
    .writeFxn = Psram_gpmcWrite,
};

static int32_t Psram_gpmcOpen(Ram_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    Ram_Attrs *attrs = config->attrs;
    Ram_GpmcPsramObject *obj = (Ram_GpmcPsramObject*)(config->object);

    obj->gpmcHandle = GPMC_getHandle(attrs->driverInstance);

    if(obj->gpmcHandle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        /* Set device type for GPMC peripherial. NANDLIKE or NORLIKE*/
        status +=GPMC_setDeviceType(obj->gpmcHandle);
    }

    if(status == SystemP_SUCCESS)
    {
        /* Set device width.*/
        status += GPMC_setDeviceSize(obj->gpmcHandle);
    }

    if(status == SystemP_SUCCESS)
    {
        /* Configure GPMC timing parameters for PSRAM. */
        status += GPMC_configureTimingParameters(obj->gpmcHandle);
    }

    return status;

}

static int32_t Psram_gpmcWrite (Ram_Config *config, uint32_t offset,
                                    uint8_t *buf, uint32_t len)

{
    int32_t status = SystemP_FAILURE;

    /* Input parameter validation. */
    if(config != NULL)
    {
        if(offset + len > config->attrs->ramSize)
        {
            status = SystemP_FAILURE;
        }
        Ram_GpmcPsramObject *obj = (Ram_GpmcPsramObject*)(config->object);

        status = GPMC_norWriteData(obj->gpmcHandle,offset,buf,len);
    }

    return status;
}

static int32_t Psram_gpmcRead (Ram_Config *config, uint32_t offset, uint8_t *buf, uint32_t len)
{
    int32_t status = SystemP_SUCCESS;

    if(config != NULL)
    {

        if(offset + len > config->attrs->ramSize)
        {
            status = SystemP_FAILURE;
        }

        Ram_GpmcPsramObject *obj = (Ram_GpmcPsramObject*)(config->object);

        status = GPMC_norReadData(obj->gpmcHandle,offset,buf,len);
    }

    return status;
}

static void Psram_gpmcClose(Ram_Config *config)
{
    return;
}