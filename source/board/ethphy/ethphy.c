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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <board/ethphy.h>
#include <drivers/hw_include/csl_types.h>

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern ETHPHY_Config gEthPhyConfig[];
extern uint32_t gEthPhyConfigNum;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

ETHPHY_Handle ETHPHY_open(uint32_t instanceId, const ETHPHY_Params *params)
{
    ETHPHY_Config *config = NULL;
    int32_t status = SystemP_FAILURE;

    if(instanceId < gEthPhyConfigNum)
    {
        config = &gEthPhyConfig[instanceId];
        if((config->fxns) && (config->fxns->openFxn))
        {
            status = config->fxns->openFxn(config, params);
            if(status != SystemP_SUCCESS)
            {
                config = NULL;
            }
        }
    }
    return config;
}

void ETHPHY_close(ETHPHY_Handle handle)
{
    ETHPHY_Config *config = (ETHPHY_Config*)handle;

    if(config && config->fxns && config->fxns->closeFxn)
    {
        config->fxns->closeFxn(config);
    }
}

int32_t ETHPHY_command(ETHPHY_Handle handle,
                       uint32_t command,
                       void *data,
                       uint32_t dataSize)
{
    ETHPHY_Config *config = (ETHPHY_Config*)handle;
    int32_t status = SystemP_FAILURE;

    if(config && config->fxns && config->fxns->commandFxn)
    {
        status = config->fxns->commandFxn(config, command, data, dataSize);
    }
    return status;
}

const ETHPHY_Attrs *ETHPHY_getAttrs(uint32_t instanceId)
{
    ETHPHY_Attrs   *attrs = NULL;
    ETHPHY_Config  *config = NULL;

    if(instanceId < gEthPhyConfigNum)
    {
        config = &gEthPhyConfig[instanceId];
        attrs = config->attrs;
    }

    return attrs;
}

void ETHPHY_Params_init(ETHPHY_Params *params)
{
    memset(params, 0, sizeof(ETHPHY_Params));
}
