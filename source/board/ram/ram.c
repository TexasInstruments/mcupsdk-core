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

#include <board/ram.h>
#include <stddef.h>

extern Ram_Config gRamConfig[];
extern uint32_t gRamConfigNum;

Ram_Attrs *Ram_getAttrs(uint32_t instanceId)
{
    Ram_Attrs *attrs = NULL;
    Ram_Config *config = NULL;

    if(instanceId < gRamConfigNum)
    {
        config = &gRamConfig[instanceId];
        attrs = config->attrs;
    }
    return attrs;
}

Ram_Handle Ram_open(uint32_t instanceId, Ram_Params *params)
{
    Ram_Config *config = NULL;

    if(instanceId < gRamConfigNum)
    {
        config = &gRamConfig[instanceId];
        if(config->fxns && config->fxns->openFxn)
        {
            int32_t status;

            status = config->fxns->openFxn(config);
            if(status != SystemP_SUCCESS)
            {
                config = NULL;
            }
        }
    }
    return config;
}

int32_t Ram_write(Ram_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len)
{
    Ram_Config *config = (Ram_Config*)handle;
    int32_t status = SystemP_FAILURE;

    if(config && config->fxns && config->fxns->writeFxn)
    {
        status = config->fxns->writeFxn(config, offset, buf, len);
    }
    return status;
}

int32_t Ram_read(Ram_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len)
{
    Ram_Config *config = (Ram_Config*)handle;
    int32_t status = SystemP_FAILURE;

    if(config && config->fxns && config->fxns->readFxn)
    {
        status = config->fxns->readFxn(config, offset, buf, len);
    }
    return status;
}

void Ram_close(Ram_Handle handle)
{
    Ram_Config *config = (Ram_Config*)handle;

    if(config && config->fxns && config->fxns->closeFxn)
    {
        config->fxns->closeFxn(config);
    }
}