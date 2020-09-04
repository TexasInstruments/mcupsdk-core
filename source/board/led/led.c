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

#include <board/led.h>
#include <drivers/hw_include/csl_types.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern LED_Config gLedConfig[];
extern uint32_t gLedConfigNum;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

LED_Handle LED_open(uint32_t instanceId, const LED_Params *params)
{
    LED_Config *config = NULL;

    if(instanceId < gLedConfigNum)
    {
        config = &gLedConfig[instanceId];
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

    return (config);
}

void LED_close(LED_Handle handle)
{
    LED_Config *config = (LED_Config *) handle;

    if(config && config->fxns && config->fxns->closeFxn)
    {
        config->fxns->closeFxn(config);
    }

    return;
}

int32_t LED_on(LED_Handle handle, uint32_t index)
{
    int32_t         status = SystemP_FAILURE;
    LED_Config  *config = (LED_Config *) handle;

    if(config && config->fxns && config->fxns->onFxn)
    {
        status = config->fxns->onFxn(config, index);
    }

    return (status);
}

int32_t LED_off(LED_Handle handle, uint32_t index)
{
    int32_t         status = SystemP_FAILURE;
    LED_Config  *config = (LED_Config *) handle;

    if(config && config->fxns && config->fxns->offFxn)
    {
        status = config->fxns->offFxn(config, index);
    }

    return (status);
}

int32_t LED_setMask(LED_Handle handle, uint32_t mask)
{
    int32_t         status = SystemP_FAILURE;
    LED_Config  *config = (LED_Config *) handle;

    if(config && config->fxns && config->fxns->setMaskFxn)
    {
        status = config->fxns->setMaskFxn(config, mask);
    }

    return (status);
}

const LED_Attrs *LED_getAttrs(uint32_t instanceId)
{
    LED_Attrs   *attrs = NULL;
    LED_Config  *config = NULL;

    if(instanceId < gLedConfigNum)
    {
        config = &gLedConfig[instanceId];
        attrs = config->attrs;
    }

    return (attrs);
}

void LED_Params_init(LED_Params *params)
{
    if(NULL != params)
    {
        params->gpioBaseAddr    = 0U;
        params->gpioPinNum      = 0U;
        params->i2cInstance     = 0U;
        params->i2cAddress      = 0x60U;
    }

    return;
}
