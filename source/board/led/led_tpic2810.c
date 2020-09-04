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
#include <board/led/led_tpic2810.h>
#include <drivers/hw_include/csl_types.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Sub address command as per TPIC2810 datasheet */
#define TPIC2810_CMD_RD                 (0x11U)
#define TPIC2810_CMD_WR_SHIFT_REG       (0x11U)
#define TPIC2810_CMD_LOAD_OUTPUT        (0x22U)
#define TPIC2810_CMD_WR_IMMEDIATE       (0x44U)

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

LED_Attrs gLedAttrs_TPIC2810 =
{
    .numLedPerGroup = 8U,
};

LED_Fxns gLedFxns_TPIC2810 =
{
    .openFxn    = LED_tpic2810Open,
    .closeFxn   = LED_tpic2810Close,
    .onFxn      = LED_tpic2810On,
    .offFxn     = LED_tpic2810Off,
    .setMaskFxn = LED_tpic2810SetMask,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t LED_tpic2810Open(LED_Config *config, const LED_Params *params)
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
        object->gpioBaseAddr = 0U;  /* Not used */
        object->gpioPinNum   = 0U;  /* Not used */
        object->i2cInstance  = params->i2cInstance;
        object->i2cAddress   = params->i2cAddress;
        object->i2cHandle      = I2C_getHandle(object->i2cInstance);
        if(NULL == object->i2cHandle)
        {
            status = SystemP_FAILURE;
        }
    }

    return (status);
}

void LED_tpic2810Close(LED_Config *config)
{
    int32_t         status = SystemP_SUCCESS;
    LED_Object     *object;

    if(NULL == config)
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        object = (LED_Object *) config->object;

        /* I2C Driver will be closed outside flash */
        object->i2cHandle = NULL;
    }

    return;
}

int32_t LED_tpic2810On(LED_Config *config, uint32_t index)
{
    int32_t         status = SystemP_SUCCESS;
    LED_Object     *object;
    LED_Attrs      *attrs;
    uint8_t         rdData, wrData[2U];
    I2C_Transaction i2cTransaction;

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
        /* Set read command */
        wrData[0U] = TPIC2810_CMD_RD;
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.writeBuf     = &wrData[0U];
        i2cTransaction.writeCount   = 1U;
        i2cTransaction.slaveAddress = object->i2cAddress;
        status = I2C_transfer(object->i2cHandle, &i2cTransaction);
    }

    if(status == SystemP_SUCCESS)
    {
        /* Read current state */
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.readBuf      = &rdData;
        i2cTransaction.readCount    = 1U;
        i2cTransaction.slaveAddress = object->i2cAddress;
        status = I2C_transfer(object->i2cHandle, &i2cTransaction);
    }

    if(status == SystemP_SUCCESS)
    {
        /* Set output to ON */
        wrData[0U] = TPIC2810_CMD_WR_IMMEDIATE;
        wrData[1U] = rdData | ((uint8_t)1U << index);
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.writeBuf     = &wrData[0U];
        i2cTransaction.writeCount   = 2U;
        i2cTransaction.slaveAddress = object->i2cAddress;
        status = I2C_transfer(object->i2cHandle, &i2cTransaction);
    }

    return (status);
}

int32_t LED_tpic2810Off(LED_Config *config, uint32_t index)
{
    int32_t         status = SystemP_SUCCESS;
    LED_Object     *object;
    LED_Attrs      *attrs;
    uint8_t         rdData, wrData[2U];
    I2C_Transaction i2cTransaction;

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
        /* Set read command */
        wrData[0U] = TPIC2810_CMD_RD;
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.writeBuf     = &wrData[0U];
        i2cTransaction.writeCount   = 1U;
        i2cTransaction.slaveAddress = object->i2cAddress;
        status = I2C_transfer(object->i2cHandle, &i2cTransaction);
    }

    if(status == SystemP_SUCCESS)
    {
        /* Read current state */
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.readBuf      = &rdData;
        i2cTransaction.readCount    = 1U;
        i2cTransaction.slaveAddress = object->i2cAddress;
        status = I2C_transfer(object->i2cHandle, &i2cTransaction);
    }

    if(status == SystemP_SUCCESS)
    {
        /* Set output to OFF */
        wrData[0U] = TPIC2810_CMD_WR_IMMEDIATE;
        wrData[1U] = rdData & ~((uint8_t)1U << index);
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.writeBuf     = &wrData[0U];
        i2cTransaction.writeCount   = 2U;
        i2cTransaction.slaveAddress = object->i2cAddress;
        status = I2C_transfer(object->i2cHandle, &i2cTransaction);
    }

    return (status);
}

int32_t LED_tpic2810SetMask(LED_Config *config, uint32_t mask)
{
    int32_t         status = SystemP_SUCCESS;
    LED_Object     *object;

    uint8_t         wrData[2U];
    I2C_Transaction i2cTransaction;

    if(NULL == config)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        object      = (LED_Object *) config->object;


        /* Set mask */
        wrData[0U] = TPIC2810_CMD_WR_IMMEDIATE;
        wrData[1U] = (uint8_t) (mask & 0xFFU);
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.writeBuf     = &wrData[0U];
        i2cTransaction.writeCount   = 2U;
        i2cTransaction.slaveAddress = object->i2cAddress;
        status = I2C_transfer(object->i2cHandle, &i2cTransaction);
    }

    return (status);
}
