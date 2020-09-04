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

#include <board/ioexp/ioexp_tca6424.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define TCA6424_REG_INPUT_PORT_0        (0x00U)
#define TCA6424_REG_INPUT_PORT_1        (0x01U)
#define TCA6424_REG_INPUT_PORT_2        (0x02U)
#define TCA6424_REG_OUTPUT_PORT_0       (0x04U)
#define TCA6424_REG_OUTPUT_PORT_1       (0x05U)
#define TCA6424_REG_OUTPUT_PORT_2       (0x06U)
#define TCA6424_REG_POL_INV_PORT_0      (0x08U)
#define TCA6424_REG_POL_INV_PORT_1      (0x09U)
#define TCA6424_REG_POL_INV_PORT_2      (0x0AU)
#define TCA6424_REG_CONFIG_PORT_0       (0x0CU)
#define TCA6424_REG_CONFIG_PORT_1       (0x0DU)
#define TCA6424_REG_CONFIG_PORT_2       (0x0EU)

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

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t TCA6424_open(TCA6424_Config *config, const TCA6424_Params *params)
{
    int32_t         status = SystemP_SUCCESS;

    if((NULL == config) || (NULL == params))
    {
        status = SystemP_FAILURE;
    }
    else
    {
        config->params.i2cInstance = params->i2cInstance;
        config->params.i2cAddress  = params->i2cAddress;
        config->lock               = NULL;
        config->i2cHandle          = I2C_getHandle(config->params.i2cInstance);
        if(NULL == config->i2cHandle)
        {
            status = SystemP_FAILURE;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        SemaphoreP_constructMutex(&config->lockObj);
        config->lock = &config->lockObj;
        TCA6424_getAttrs(config, &config->attrs);
    }

    return (status);
}

void TCA6424_close(TCA6424_Config *config)
{

    if(NULL == config)
    {
    }
    else
    {
        /* I2C Driver will be closed outside flash */
        config->i2cHandle = NULL;
        if(NULL != config->lock)
        {
            SemaphoreP_destruct(&config->lockObj);
            config->lock = NULL;
        }
    }

    return;
}

int32_t TCA6424_config(TCA6424_Config *config, uint32_t ioIndex, uint32_t mode)
{
    int32_t         status = SystemP_SUCCESS;
    I2C_Transaction i2cTransaction;
    uint32_t        port, portPin, i2cAddress;
    uint8_t         buffer[2U] = {0};

    if(NULL == config)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        /* Validate input IO number */
        if(ioIndex >= config->attrs.numIo)
        {
            status = SystemP_FAILURE;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        /* Each port contains 8 IOs */
        port        = ioIndex >> 3U;        /* /8 gives port */
        portPin     = ioIndex & 0x07U;      /* %8 gives pin within port */
        i2cAddress  = config->params.i2cAddress;

        SemaphoreP_pend(&config->lockObj, SystemP_WAIT_FOREVER);

        /* Set config register address - needed for next read */
        I2C_Transaction_init(&i2cTransaction);
        buffer[0] = TCA6424_REG_CONFIG_PORT_0 + port;
        i2cTransaction.writeBuf     = buffer;
        i2cTransaction.writeCount   = 1U;
        i2cTransaction.slaveAddress = i2cAddress;
        status += I2C_transfer(config->i2cHandle, &i2cTransaction);

        /* Read config register value */
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.readBuf      = buffer;
        i2cTransaction.readCount    = 1;
        i2cTransaction.slaveAddress = i2cAddress;
        status += I2C_transfer(config->i2cHandle, &i2cTransaction);

        /* Set output or input mode to particular IO pin - read/modify/write */
        I2C_Transaction_init(&i2cTransaction);
        if(TCA6424_MODE_INPUT == mode)
        {
            buffer[1] = buffer[0] | (0x01 << portPin);
        }
        else
        {
            buffer[1] = buffer[0] & ~(0x01 << portPin);
        }
        buffer[0] = TCA6424_REG_CONFIG_PORT_0 + port;
        i2cTransaction.writeBuf     = buffer;
        i2cTransaction.writeCount   = 2;
        i2cTransaction.slaveAddress = i2cAddress;
        status += I2C_transfer(config->i2cHandle, &i2cTransaction);

        SemaphoreP_post(&config->lockObj);
    }

    return (status);
}

int32_t TCA6424_setOutput(TCA6424_Config *config, uint32_t ioIndex, uint32_t state)
{
    int32_t         status = SystemP_SUCCESS;
    I2C_Transaction i2cTransaction;
    uint32_t        port, portPin, i2cAddress;
    uint8_t         buffer[2U] = {0};

    if(NULL == config)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        /* Validate input IO number */
        if(ioIndex >= config->attrs.numIo)
        {
            status = SystemP_FAILURE;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        /* Each port contains 8 IOs */
        port        = ioIndex >> 3U;        /* /8 gives port */
        portPin     = ioIndex & 0x07U;      /* %8 gives pin within port */
        i2cAddress  = config->params.i2cAddress;

        SemaphoreP_pend(&config->lockObj, SystemP_WAIT_FOREVER);

        /* Set output prt register address - needed for next read */
        I2C_Transaction_init(&i2cTransaction);
        buffer[0] = TCA6424_REG_OUTPUT_PORT_0 + port;
        i2cTransaction.writeBuf     = buffer;
        i2cTransaction.writeCount   = 1U;
        i2cTransaction.slaveAddress = i2cAddress;
        status += I2C_transfer(config->i2cHandle, &i2cTransaction);

        /* Read config register value */
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.readBuf      = buffer;
        i2cTransaction.readCount    = 1;
        i2cTransaction.slaveAddress = i2cAddress;
        status += I2C_transfer(config->i2cHandle, &i2cTransaction);

        /* Set output or input mode to particular IO pin - read/modify/write */
        I2C_Transaction_init(&i2cTransaction);
        if(TCA6424_OUT_STATE_HIGH == state)
        {
            buffer[1] = buffer[0] | (0x01 << portPin);
        }
        else
        {
            buffer[1] = buffer[0] & ~(0x01 << portPin);
        }
        buffer[0] = TCA6424_REG_OUTPUT_PORT_0 + port;
        i2cTransaction.writeBuf     = buffer;
        i2cTransaction.writeCount   = 2;
        i2cTransaction.slaveAddress = i2cAddress;
        status += I2C_transfer(config->i2cHandle, &i2cTransaction);

        SemaphoreP_post(&config->lockObj);
    }

    return (status);
}

void TCA6424_getAttrs(TCA6424_Config *config, TCA6424_Attrs *attrs)
{
    if(NULL != attrs)
    {
        attrs->numIo = 24U;
    }

    return;
}

void TCA6424_Params_init(TCA6424_Params *params)
{
    if(NULL != params)
    {
        params->i2cInstance = 0U;
        params->i2cAddress  = 0x22U;
    }

    return;
}
