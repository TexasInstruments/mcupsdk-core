/*
 *  Copyright (C) 2026 Texas Instruments Incorporated
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

/**
 * \file hdc3020.c
 * \brief HDC3020 Temperature/Humidity Sensor Driver Implementation
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <kernel/dpl/ClockP.h>
#include "hdc3020.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Conversion times in milliseconds for each LPM */
static const uint8_t gTida010997Hdc3020ConversionTime[4] = {
    13U,  /* LPM0 */
    8U,   /* LPM1 */
    5U,   /* LPM2 */
    4U    /* LPM3 */
};

/* Command codes for different trigger modes */
static const uint16_t gTida010997Hdc3020TriggerCmd[4] = {
    0x2400U,  /* LPM0 */
    0x240BU,  /* LPM1 */
    0x2416U,  /* LPM2 */
    0x24FFU   /* LPM3 */
};

/* Command codes for different auto modes and rates */
/* [rate][lpm] */
static const uint16_t gTida010997Hdc3020AutoCmd[5][4] = {
    /* LPM0,    LPM1,    LPM2,    LPM3 */
    { 0x2032U, 0x2024U, 0x202FU, 0x20FFU },  /* 0.5Hz */
    { 0x2130U, 0x2126U, 0x212DU, 0x21FFU },  /* 1Hz */
    { 0x2236U, 0x2220U, 0x222BU, 0x22FFU },  /* 2Hz */
    { 0x2334U, 0x2322U, 0x2329U, 0x23FFU },  /* 4Hz */
    { 0x2737U, 0x2721U, 0x272AU, 0x27FFU }   /* 10Hz */
};

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static I2C_Handle                  gTida010997Hdc3020I2cHandle = NULL;
static HDC3020_Config_t gTida010997Hdc3020Config;
static bool                        gTida010997Hdc3020IsInitialized = false;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 * \brief Send a 16-bit command to the HDC3020
 */
static int32_t HDC3020_sendCommand(uint16_t cmd)
{
    int32_t         status;
    I2C_Transaction i2cTrans;
    uint8_t         txBuf[2];

    if (gTida010997Hdc3020I2cHandle == NULL)
    {
        return HDC3020_ERR_NOT_INITIALIZED;
    }

    txBuf[0] = (uint8_t)((cmd >> 8U) & 0xFFU);  /* MSB */
    txBuf[1] = (uint8_t)(cmd & 0xFFU);          /* LSB */

    I2C_Transaction_init(&i2cTrans);
    i2cTrans.writeBuf      = txBuf;
    i2cTrans.writeCount    = 2U;
    i2cTrans.readBuf       = NULL;
    i2cTrans.readCount     = 0U;
    i2cTrans.targetAddress = HDC3020_I2C_ADDR;

    status = I2C_transfer(gTida010997Hdc3020I2cHandle, &i2cTrans);

    return (status == SystemP_SUCCESS) ? HDC3020_OK : HDC3020_ERR_I2C;
}

uint8_t HDC3020_calculateCrc(const uint8_t *data, uint8_t length)
{
    uint8_t crc        = 0xFFU;  /* Initial value */
    uint8_t polynomial = 0x31U;  /* x^8 + x^5 + x^4 + 1 */
    uint8_t i, j;

    for (i = 0U; i < length; i++)
    {
        crc ^= data[i];
        for (j = 0U; j < 8U; j++)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = (uint8_t)((crc << 1U) ^ polynomial);
            }
            else
            {
                crc = (uint8_t)(crc << 1U);
            }
        }
    }

    return crc;
}

int32_t HDC3020_init(I2C_Handle i2cHandle, const HDC3020_Config_t *config)
{
    int32_t status;

    if ((i2cHandle == NULL) || (config == NULL))
    {
        return HDC3020_ERR_I2C;
    }

    gTida010997Hdc3020I2cHandle = i2cHandle;
    (void)memcpy(&gTida010997Hdc3020Config, config, sizeof(HDC3020_Config_t));

    /* Probe sensor */
    status = I2C_probe(gTida010997Hdc3020I2cHandle, HDC3020_I2C_ADDR);
    if (status != SystemP_SUCCESS)
    {
        return HDC3020_ERR_I2C;
    }

    /* Reset the sensor */
    status = HDC3020_reset();
    if (status != HDC3020_OK)
    {
        return status;
    }

    /* Wait for reset to complete (5ms) */
    ClockP_usleep(5000U);

    /* Configure the mode */
    status = HDC3020_setMode(config->mode, config->lpm, config->rate);
    if (status == HDC3020_OK)
    {
        gTida010997Hdc3020IsInitialized = true;
    }

    return status;
}

int32_t HDC3020_reset(void)
{
    return HDC3020_sendCommand(HDC3020_CMD_SOFT_RESET);
}

int32_t HDC3020_setMode(HDC3020_Mode_t mode, HDC3020_Lpm_t lpm, HDC3020_Rate_t rate)
{
    int32_t status = HDC3020_OK;

    gTida010997Hdc3020Config.mode = mode;
    gTida010997Hdc3020Config.lpm  = lpm;

    if (mode == HDC3020_MODE_AUTO_MEASUREMENT)
    {
        gTida010997Hdc3020Config.rate = rate;
        status = HDC3020_sendCommand(gTida010997Hdc3020AutoCmd[rate][lpm]);
    }
    /* For trigger-on-demand mode, no command needed until measurement */

    return status;
}

int32_t HDC3020_triggerMeasurement(HDC3020_Lpm_t lpm)
{
    int32_t status;

    if (gTida010997Hdc3020Config.mode != HDC3020_MODE_TRIGGER_ON_DEMAND)
    {
        /* Exit auto mode first */
        status = HDC3020_sendCommand(HDC3020_CMD_EXIT_AUTO_MODE);
        if (status != HDC3020_OK)
        {
            return status;
        }

        /* Wait before sending trigger command */
        ClockP_usleep(5000U);

        gTida010997Hdc3020Config.mode = HDC3020_MODE_TRIGGER_ON_DEMAND;
    }

    return HDC3020_sendCommand(gTida010997Hdc3020TriggerCmd[lpm]);
}

int32_t HDC3020_getData(HDC3020_Data_t *data, HDC3020_Sensor_t sensorType)
{
    int32_t         status;
    I2C_Transaction i2cTrans;
    uint8_t         cmd[2];
    uint8_t         rxBuf[6];
    uint8_t         tempData[2];
    uint8_t         rhData[2];
    uint32_t        readCount;

    if (data == NULL)
    {
        return HDC3020_ERR_I2C;
    }

    if (!gTida010997Hdc3020IsInitialized)
    {
        return HDC3020_ERR_NOT_INITIALIZED;
    }

    data->valid = false;

    /* Select the appropriate command based on sensor type */
    if ((sensorType == HDC3020_SENSOR_BOTH) || (sensorType == HDC3020_SENSOR_TEMPERATURE))
    {
        cmd[0] = 0xE0U;
        cmd[1] = 0x00U;  /* Read both temperature and humidity */
        readCount = 6U;  /* T_MSB, T_LSB, T_CRC, RH_MSB, RH_LSB, RH_CRC */
    }
    else
    {
        cmd[0] = 0xE0U;
        cmd[1] = 0x01U;  /* Read humidity only */
        readCount = 3U;  /* RH_MSB, RH_LSB, RH_CRC */
    }

    /* In trigger-on-demand mode, trigger measurement first */
    if (gTida010997Hdc3020Config.mode == HDC3020_MODE_TRIGGER_ON_DEMAND)
    {
        status = HDC3020_triggerMeasurement(gTida010997Hdc3020Config.lpm);
        if (status != HDC3020_OK)
        {
            return status;
        }

        /* Wait for conversion to complete */
        ClockP_usleep((uint32_t)gTida010997Hdc3020ConversionTime[gTida010997Hdc3020Config.lpm] * 1000U);
    }

    /* Combined write-read transaction (uses repeated start condition) */
    I2C_Transaction_init(&i2cTrans);
    i2cTrans.writeBuf      = cmd;
    i2cTrans.writeCount    = 2U;
    i2cTrans.readBuf       = rxBuf;
    i2cTrans.readCount     = readCount;
    i2cTrans.targetAddress = HDC3020_I2C_ADDR;

    status = I2C_transfer(gTida010997Hdc3020I2cHandle, &i2cTrans);
    if (status != SystemP_SUCCESS)
    {
        return HDC3020_ERR_I2C;
    }

    /* Parse and verify CRC */
    if ((sensorType == HDC3020_SENSOR_BOTH) || (sensorType == HDC3020_SENSOR_TEMPERATURE))
    {
        /* Verify temperature CRC */
        tempData[0] = rxBuf[0];
        tempData[1] = rxBuf[1];
        if (HDC3020_calculateCrc(tempData, 2U) != rxBuf[2])
        {
            return HDC3020_ERR_CRC;
        }

        /* Verify humidity CRC */
        rhData[0] = rxBuf[3];
        rhData[1] = rxBuf[4];
        if (HDC3020_calculateCrc(rhData, 2U) != rxBuf[5])
        {
            return HDC3020_ERR_CRC;
        }

        data->temperature = ((uint16_t)rxBuf[0] << 8U) | (uint16_t)rxBuf[1];
        data->humidity    = ((uint16_t)rxBuf[3] << 8U) | (uint16_t)rxBuf[4];

        /* Check for invalid readings */
        if ((data->temperature == 0U) || (data->humidity == 0xFFFFU))
        {
            return HDC3020_ERR_INVALID_DATA;
        }
    }
    else
    {
        /* Verify humidity CRC */
        rhData[0] = rxBuf[0];
        rhData[1] = rxBuf[1];
        if (HDC3020_calculateCrc(rhData, 2U) != rxBuf[2])
        {
            return HDC3020_ERR_CRC;
        }

        data->temperature = 0U;
        data->humidity    = ((uint16_t)rxBuf[0] << 8U) | (uint16_t)rxBuf[1];

        /* Check for invalid reading */
        if (data->humidity == 0xFFFFU)
        {
            return HDC3020_ERR_INVALID_DATA;
        }
    }

    data->valid = true;

    return HDC3020_OK;
}

float HDC3020_convertTemperature(uint16_t raw)
{
    /* Formula: T = -45 + 175 * (raw / 65535) */
    return -45.0f + 175.0f * ((float)raw / 65535.0f);
}

float HDC3020_convertHumidity(uint16_t raw)
{
    /* Formula: RH = 100 * (raw / 65535) */
    return 100.0f * ((float)raw / 65535.0f);
}
