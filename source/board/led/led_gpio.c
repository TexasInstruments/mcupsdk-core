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
#include <board/led/led_gpio.h>
#include <kernel/dpl/AddrTranslateP.h>
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

LED_Attrs gLedAttrs_GPIO =
{
    .numLedPerGroup = 1U,
};

LED_Fxns gLedFxns_GPIO =
{
    .openFxn    = LED_gpioOpen,
    .closeFxn   = LED_gpioClose,
    .onFxn      = LED_gpioOn,
    .offFxn     = LED_gpioOff,
    .setMaskFxn = NULL,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t LED_gpioOpen(LED_Config *config, const LED_Params *params)
{
    int32_t         status = SystemP_SUCCESS;
    LED_Object     *object;

    if((NULL == config) || (NULL == params))
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        object = (LED_Object *) config->object;
        /* Get address after translation translate */
        object->gpioBaseAddr =
            (uint32_t) AddrTranslateP_getLocalAddr(params->gpioBaseAddr);
        object->gpioPinNum  = params->gpioPinNum;
        object->i2cInstance  = 0U;  /* Not used */
        object->i2cAddress   = 0U;  /* Not used */
        object->i2cHandle    = NULL;/* Not used */

        /* Set GPIO direction */
        GPIO_setDirMode(
            object->gpioBaseAddr, object->gpioPinNum, GPIO_DIRECTION_OUTPUT);
    }

    return (status);
}

void LED_gpioClose(LED_Config *config)
{
    /* Nothing to do */
    return;
}

int32_t LED_gpioOn(LED_Config *config, uint32_t index)
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
        GPIO_pinWriteHigh(object->gpioBaseAddr, object->gpioPinNum);
    }

    return (status);
}

int32_t LED_gpioOff(LED_Config *config, uint32_t index)
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
        GPIO_pinWriteLow(object->gpioBaseAddr, object->gpioPinNum);
    }

    return (status);
}
