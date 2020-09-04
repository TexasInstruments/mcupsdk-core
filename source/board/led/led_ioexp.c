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
#include <board/led/led_ioexp.h>
#include <board/ioexp/ioexp_tca6424.h>
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

LED_Attrs gLedAttrs_Ioexp =
{
    .numLedPerGroup = 1U,
};

LED_Fxns gLedFxns_Ioexp =
{
    .openFxn    = LED_ioexpOpen,
    .closeFxn   = LED_ioexpClose,
    .onFxn      = LED_ioexpOn,
    .offFxn     = LED_ioexpOff,
    .setMaskFxn = NULL,
};

static TCA6424_Config  gTCA6424_Config;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t LED_ioexpOpen(LED_Config *config, const LED_Params *params)
{
    int32_t         status = SystemP_SUCCESS;
    LED_Object     *object;
    TCA6424_Params  tca6424Params;

    if((NULL == config) || (NULL == params))
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        object = (LED_Object *) config->object;
        object->gpioBaseAddr = 0U;  /* Not used */
        object->gpioPinNum   = 0U;  /* Not used */
        object->i2cInstance  = params->i2cInstance;
        object->i2cAddress   = params->i2cAddress;
        object->ioIndex      = params->ioIndex;

        TCA6424_Params_init(&tca6424Params);
        tca6424Params.i2cInstance = params->i2cInstance;
        tca6424Params.i2cAddress = params->i2cAddress;
        status = TCA6424_open(&gTCA6424_Config, &tca6424Params);
        if(status == SystemP_SUCCESS)
        {
            /* Set output to low before config so that LED start with Off state */
            status = TCA6424_setOutput(
                         &gTCA6424_Config,
                         object->ioIndex,
                         TCA6424_OUT_STATE_LOW);

            /* Configure as output  */
            status += TCA6424_config(
                          &gTCA6424_Config,
                          params->ioIndex,
                          TCA6424_MODE_OUTPUT);
        }
    }

    return (status);
}

void LED_ioexpClose(LED_Config *config)
{
    int32_t         status = SystemP_SUCCESS;

    if(NULL == config)
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        TCA6424_close(&gTCA6424_Config);
    }

    return;
}

int32_t LED_ioexpOn(LED_Config *config, uint32_t index)
{
    int32_t         status = SystemP_SUCCESS;
    LED_Object     *object;
    LED_Attrs      *attrs;

    if(NULL == config)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        object      = (LED_Object *) config->object;
        attrs       = config->attrs;
        /* Validate index */
        if(index >= attrs->numLedPerGroup)
        {
            status = SystemP_FAILURE;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        status = TCA6424_setOutput(
                     &gTCA6424_Config,
                     object->ioIndex,
                     TCA6424_OUT_STATE_HIGH);
    }

    return (status);
}

int32_t LED_ioexpOff(LED_Config *config, uint32_t index)
{
    int32_t         status = SystemP_SUCCESS;
    LED_Object     *object;
    LED_Attrs      *attrs;

    if(NULL == config)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        object      = (LED_Object *) config->object;
        attrs       = config->attrs;
        /* Validate index */
        if(index >= attrs->numLedPerGroup)
        {
            status = SystemP_FAILURE;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        status = TCA6424_setOutput(
                     &gTCA6424_Config,
                     object->ioIndex,
                     TCA6424_OUT_STATE_LOW);
    }

    return (status);
}
