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
 * \file bmi270.c
 * \brief BMI270 6-Axis IMU Driver Implementation
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>
#include "bmi270.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* BMI270 Register Addresses */
#define BMI270_REG_CHIP_ID         (0x00U)
#define BMI270_REG_ERR_REG         (0x02U)
#define BMI270_REG_STATUS          (0x03U)
#define BMI270_REG_DATA_0          (0x04U)
#define BMI270_REG_DATA_8          (0x0CU)  /* Accel X LSB */
#define BMI270_REG_DATA_14         (0x12U)  /* Gyro X LSB */
#define BMI270_REG_SENSORTIME_0    (0x18U)
#define BMI270_REG_EVENT           (0x1BU)
#define BMI270_REG_INTERNAL_STATUS (0x21U)
#define BMI270_REG_ACC_CONF        (0x40U)
#define BMI270_REG_ACC_RANGE       (0x41U)
#define BMI270_REG_GYR_CONF        (0x42U)
#define BMI270_REG_GYR_RANGE       (0x43U)
#define BMI270_REG_INIT_CTRL       (0x59U)
#define BMI270_REG_INIT_ADDR_0     (0x5BU)  /* Firmware load address LSB */
#define BMI270_REG_INIT_ADDR_1     (0x5CU)  /* Firmware load address MSB */
#define BMI270_REG_INIT_DATA       (0x5EU)
#define BMI270_REG_PWR_CONF        (0x7CU)
#define BMI270_REG_PWR_CTRL        (0x7DU)
#define BMI270_REG_CMD             (0x7EU)

/* BMI270 Commands */
#define BMI270_CMD_SOFT_RESET (0xB6U)

/* BMI270 Internal Status Register Bits */
#define BMI270_INTERNAL_STATUS_INIT_OK (0x01U)

/* BMI270 Power Control Register Bits */
#define BMI270_PWR_CTRL_AUX_EN  (0x01U)
#define BMI270_PWR_CTRL_GYR_EN  (0x02U)
#define BMI270_PWR_CTRL_ACC_EN  (0x04U)
#define BMI270_PWR_CTRL_TEMP_EN (0x08U)

/* SPI Read/Write Masks */
#define BMI270_SPI_READ_MASK  (0x80U)
#define BMI270_SPI_WRITE_MASK (0x7FU)

/* Number of dummy bytes in SPI read */
#define BMI270_SPI_DUMMY_BYTES (2U)

/* Configuration constants */
#define BMI270_RESET_DELAY_MS       (10U)
#define BMI270_POWER_CONF_DELAY_MS  (1U)
#define BMI270_INIT_POLL_TIMEOUT_MS (150U)  /* BMI270 can take up to 150ms to initialize */
#define BMI270_SPI_SWITCH_DELAY_MS  (10U)
#define BMI270_FIRMWARE_CHUNK_SIZE  (256U)

/* Gravity constant for acceleration conversion */
#define BMI270_GRAVITY_M_S2 (9.80665f)

/* External reference to firmware data */
extern const uint8_t gTida010997Bmi270Firmware[BMI270_FIRMWARE_SIZE];

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static MCSPI_Handle                gTida010997Bmi270SpiHandle = NULL;
static BMI270_Config_t  gTida010997Bmi270Config;
static bool                        gTida010997Bmi270IsInitialized = false;

/* ========================================================================== */
/*                          Function Prototypes                               */
/* ========================================================================== */

static int32_t BMI270_readRegister(uint8_t regAddr, uint8_t *buffer, uint32_t length);
static int32_t BMI270_writeRegister(uint8_t regAddr, const uint8_t *data, uint32_t length);
static int32_t BMI270_switchToSpiMode(void);
static int32_t BMI270_uploadFirmware(void);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static int32_t BMI270_readRegister(uint8_t regAddr, uint8_t *buffer, uint32_t length)
{
    int32_t           status;
    MCSPI_Transaction spiTrans;
    uint8_t           txBuffer[258U];  /* Max: 1 addr + 1 dummy + 256 data */
    uint8_t           rxBuffer[258U];
    uint32_t          totalLength = length + BMI270_SPI_DUMMY_BYTES;
    uint32_t          i;

    if (gTida010997Bmi270SpiHandle == NULL)
    {
        return BMI270_ERR_NOT_INITIALIZED;
    }

    /* Check length does not exceed buffer size */
    if (totalLength > sizeof(txBuffer))
    {
        return BMI270_ERR_INVALID_DATA;
    }

    /* Set read bit (MSB = 1) */
    txBuffer[0U] = regAddr | BMI270_SPI_READ_MASK;

    /* Fill rest with dummy bytes */
    for (i = 1U; i < totalLength; i++)
    {
        txBuffer[i] = 0x00U;
    }

    /* Setup SPI transaction */
    MCSPI_Transaction_init(&spiTrans);
    spiTrans.channel  = 0U;
    spiTrans.count    = totalLength;
    spiTrans.txBuf    = (void *)txBuffer;
    spiTrans.rxBuf    = (void *)rxBuffer;
    spiTrans.args     = NULL;
    spiTrans.csDisable = TRUE;

    /* Perform SPI transfer */
    status = MCSPI_transfer(gTida010997Bmi270SpiHandle, &spiTrans);
    if (status != SystemP_SUCCESS)
    {
        return BMI270_ERR_SPI;
    }

    /* Copy data from RX buffer (skip dummy bytes) */
    for (i = 0U; i < length; i++)
    {
        buffer[i] = rxBuffer[i + BMI270_SPI_DUMMY_BYTES];
    }

    return BMI270_OK;
}

static int32_t BMI270_writeRegister(uint8_t regAddr, const uint8_t *data, uint32_t length)
{
    int32_t           status;
    MCSPI_Transaction spiTrans;
    uint8_t           txBuffer[258U];  /* Max: 1 addr + 256 data + 1 spare */
    uint8_t           rxBuffer[258U];
    uint32_t          totalLength = length + 1U;
    uint32_t          i;

    if (gTida010997Bmi270SpiHandle == NULL)
    {
        return BMI270_ERR_NOT_INITIALIZED;
    }

    /* Check length does not exceed buffer size */
    if (totalLength > sizeof(txBuffer))
    {
        return BMI270_ERR_INVALID_DATA;
    }

    /* Clear write bit (MSB = 0) */
    txBuffer[0U] = regAddr & BMI270_SPI_WRITE_MASK;

    /* Copy data */
    for (i = 0U; i < length; i++)
    {
        txBuffer[i + 1U] = data[i];
    }

    /* Setup SPI transaction */
    MCSPI_Transaction_init(&spiTrans);
    spiTrans.channel  = 0U;
    spiTrans.count    = totalLength;
    spiTrans.txBuf    = (void *)txBuffer;
    spiTrans.rxBuf    = (void *)rxBuffer;
    spiTrans.args     = NULL;
    spiTrans.csDisable = TRUE;

    /* Perform SPI transfer */
    status = MCSPI_transfer(gTida010997Bmi270SpiHandle, &spiTrans);
    if (status != SystemP_SUCCESS)
    {
        return BMI270_ERR_SPI;
    }

    return BMI270_OK;
}

static int32_t BMI270_switchToSpiMode(void)
{
    uint8_t dummy = 0U;

    /* Perform a dummy read to switch from I2C to SPI mode */
    return BMI270_readRegister(BMI270_REG_CHIP_ID, &dummy, 1U);
}

static int32_t BMI270_uploadFirmware(void)
{
    int32_t  status   = BMI270_OK;
    uint8_t  regValue = 0U;
    uint32_t offset;
    uint32_t chunkSize;
    uint32_t pollCount;
    bool     initOk = false;

    DebugP_log("[BMI270] Starting firmware upload (%u bytes)...\r\n", BMI270_FIRMWARE_SIZE);

    /* Disable advanced power save */
    regValue = 0x00U;
    status   = BMI270_writeRegister(BMI270_REG_PWR_CONF, &regValue, 1U);
    if (status != BMI270_OK)
    {
        DebugP_log("[BMI270] Failed to write PWR_CONF register\r\n");
        return status;
    }

    /* Wait for power configuration to settle */
    ClockP_usleep(BMI270_POWER_CONF_DELAY_MS * 1000U);

    /* Clear INIT_CTRL register - prepare for config load */
    regValue = 0x00U;
    status   = BMI270_writeRegister(BMI270_REG_INIT_CTRL, &regValue, 1U);
    if (status != BMI270_OK)
    {
        DebugP_log("[BMI270] Failed to clear INIT_CTRL register\r\n");
        return status;
    }

    /* Upload firmware in chunks */
    for (offset = 0U;
         (offset < BMI270_FIRMWARE_SIZE) && (status == BMI270_OK);
         offset += BMI270_FIRMWARE_CHUNK_SIZE)
    {
        /* Set INIT_ADDR to current word address (offset / 2) */
        uint16_t wordAddr = (uint16_t)(offset / 2U);
        regValue = (uint8_t)(wordAddr & 0x0FU);
        status = BMI270_writeRegister(BMI270_REG_INIT_ADDR_0, &regValue, 1U);
        if (status != BMI270_OK)
        {
            DebugP_log("[BMI270] Failed to write INIT_ADDR_0 at offset %u\r\n", offset);
            return status;
        }

        regValue = (uint8_t)((wordAddr >> 4U) & 0xFFU);
        status = BMI270_writeRegister(BMI270_REG_INIT_ADDR_1, &regValue, 1U);
        if (status != BMI270_OK)
        {
            DebugP_log("[BMI270] Failed to write INIT_ADDR_1 at offset %u\r\n", offset);
            return status;
        }

        chunkSize = BMI270_FIRMWARE_CHUNK_SIZE;
        if ((offset + chunkSize) > BMI270_FIRMWARE_SIZE)
        {
            chunkSize = BMI270_FIRMWARE_SIZE - offset;
        }

        status = BMI270_writeRegister(BMI270_REG_INIT_DATA,
                                                  &gTida010997Bmi270Firmware[offset],
                                                  chunkSize);
        if (status != BMI270_OK)
        {
            DebugP_log("[BMI270] Firmware upload failed at offset %u\r\n", offset);
            return status;
        }
    }
    DebugP_log("[BMI270] Firmware data uploaded successfully\r\n");

    /* Set INIT_CTRL to 0x01 to complete initialization */
    regValue = 0x01U;
    status   = BMI270_writeRegister(BMI270_REG_INIT_CTRL, &regValue, 1U);
    if (status != BMI270_OK)
    {
        DebugP_log("[BMI270] Failed to set INIT_CTRL to start init\r\n");
        return status;
    }

    /* Poll INTERNAL_STATUS register for init_ok bit */
    DebugP_log("[BMI270] Waiting for init_ok status...\r\n");
    for (pollCount = 0U;
         (pollCount < BMI270_INIT_POLL_TIMEOUT_MS) && (status == BMI270_OK) && (!initOk);
         pollCount++)
    {
        status = BMI270_readRegister(BMI270_REG_INTERNAL_STATUS, &regValue, 1U);

        if (status == BMI270_OK)
        {
            /* Check if init_ok bit is set */
            if ((regValue & BMI270_INTERNAL_STATUS_INIT_OK) != 0U)
            {
                initOk = true;
                DebugP_log("[BMI270] Init OK! (INTERNAL_STATUS=0x%02X after %u ms)\r\n", regValue, pollCount);
            }
            else
            {
                ClockP_usleep(1000U);  /* 1ms delay */
            }
        }
    }

    /* Check if timeout occurred */
    if ((status == BMI270_OK) && (!initOk))
    {
        DebugP_log("[BMI270] Timeout waiting for init_ok (INTERNAL_STATUS=0x%02X)\r\n", regValue);
        status = BMI270_ERR_TIMEOUT;
    }

    return status;
}

int32_t BMI270_init(MCSPI_Handle spiHandle, const BMI270_Config_t *config)
{
    int32_t status   = BMI270_OK;
    uint8_t chipId   = 0U;
    uint8_t regValue = 0U;

    if ((spiHandle == NULL) || (config == NULL))
    {
        return BMI270_ERR_SPI;
    }

    /* Store configuration */
    gTida010997Bmi270SpiHandle = spiHandle;
    (void)memcpy(&gTida010997Bmi270Config, config, sizeof(BMI270_Config_t));

    gTida010997Bmi270IsInitialized = false;

    /* Switch to SPI mode (dummy read) */
    status = BMI270_switchToSpiMode();

    if (status == BMI270_OK)
    {
        /* Wait for SPI mode switch to complete */
        ClockP_usleep(BMI270_SPI_SWITCH_DELAY_MS * 1000U);

        /* Perform soft reset */
        status = BMI270_reset();
    }

    if (status == BMI270_OK)
    {
        /* Wait for reset to complete */
        ClockP_usleep(BMI270_RESET_DELAY_MS * 1000U);

        /* Switch to SPI mode again after reset */
        status = BMI270_switchToSpiMode();
    }

    if (status == BMI270_OK)
    {
        /* Wait for SPI mode switch to complete */
        ClockP_usleep(BMI270_SPI_SWITCH_DELAY_MS * 1000U);

        /* Verify chip ID */
        status = BMI270_getChipId(&chipId);
    }

    if (status == BMI270_OK)
    {
        DebugP_log("[BMI270] Chip ID read: 0x%02X (expected 0x%02X)\r\n", chipId, BMI270_CHIP_ID);
        if (chipId != BMI270_CHIP_ID)
        {
            if (chipId == 0x00U)
            {
                DebugP_log("[BMI270] ChipID=0x00: No response. Check MISO connection or sensor power.\r\n");
            }
            else if (chipId == 0xFFU)
            {
                DebugP_log("[BMI270] ChipID=0xFF: MISO line high. Check wiring or sensor not present.\r\n");
            }
            status = BMI270_ERR_INVALID_CHIP_ID;
        }
    }

    if (status == BMI270_OK)
    {
        /* Upload firmware */
        status = BMI270_uploadFirmware();
        if (status != BMI270_OK)
        {
            status = BMI270_ERR_INIT_FAILED;
        }
    }

    /* Configure accelerometer if enabled */
    if ((status == BMI270_OK) &&
        ((config->sensorEnable == BMI270_SENSOR_ACCEL_ONLY) ||
         (config->sensorEnable == BMI270_SENSOR_BOTH)))
    {
        /* Configure ACC_CONF register */
        regValue = (uint8_t)config->accelOdr |
                   ((uint8_t)config->accelBwp << 4U) |
                   (config->accelFilterPerf ? (1U << 7U) : 0U);
        status = BMI270_writeRegister(BMI270_REG_ACC_CONF, &regValue, 1U);

        if (status == BMI270_OK)
        {
            /* Configure ACC_RANGE register */
            regValue = (uint8_t)config->accelRange;
            status   = BMI270_writeRegister(BMI270_REG_ACC_RANGE, &regValue, 1U);
        }
    }

    /* Configure gyroscope if enabled */
    if ((status == BMI270_OK) &&
        ((config->sensorEnable == BMI270_SENSOR_GYRO_ONLY) ||
         (config->sensorEnable == BMI270_SENSOR_BOTH)))
    {
        /* Configure GYR_CONF register */
        regValue = (uint8_t)config->gyroOdr |
                   ((uint8_t)config->gyroBwp << 4U) |
                   (config->gyroNoisePerf ? (1U << 6U) : 0U) |
                   (config->gyroFilterPerf ? (1U << 7U) : 0U);
        status = BMI270_writeRegister(BMI270_REG_GYR_CONF, &regValue, 1U);

        if (status == BMI270_OK)
        {
            /* Configure GYR_RANGE register */
            regValue = (uint8_t)config->gyroRange;
            status   = BMI270_writeRegister(BMI270_REG_GYR_RANGE, &regValue, 1U);
        }
    }

    if (status == BMI270_OK)
    {
        /* Enable sensors via PWR_CTRL register */
        status = BMI270_setSensorEnable(config->sensorEnable);
    }

    if (status == BMI270_OK)
    {
        /* Set power mode via PWR_CONF register */
        status = BMI270_setPowerMode(config->powerMode);
    }

    if (status == BMI270_OK)
    {
        gTida010997Bmi270IsInitialized = true;
    }

    return status;
}

int32_t BMI270_reset(void)
{
    uint8_t command = BMI270_CMD_SOFT_RESET;

    return BMI270_writeRegister(BMI270_REG_CMD, &command, 1U);
}

int32_t BMI270_getAccelData(BMI270_AccelData_t *data)
{
    int32_t status;
    uint8_t rawData[6U];

    if (data == NULL)
    {
        return BMI270_ERR_INVALID_DATA;
    }

    /* Initialize validity flag */
    data->valid = false;

    /* Check initialization */
    if (!gTida010997Bmi270IsInitialized)
    {
        return BMI270_ERR_NOT_INITIALIZED;
    }

    /* Check sensor is enabled */
    if ((gTida010997Bmi270Config.sensorEnable != BMI270_SENSOR_ACCEL_ONLY) &&
        (gTida010997Bmi270Config.sensorEnable != BMI270_SENSOR_BOTH))
    {
        return BMI270_ERR_SENSOR_DISABLED;
    }

    /* Read 6 bytes of accelerometer data */
    status = BMI270_readRegister(BMI270_REG_DATA_8, rawData, 6U);

    if (status == BMI270_OK)
    {
        /* Parse little-endian signed 16-bit data */
        data->x     = (int16_t)(((uint16_t)rawData[1U] << 8U) | (uint16_t)rawData[0U]);
        data->y     = (int16_t)(((uint16_t)rawData[3U] << 8U) | (uint16_t)rawData[2U]);
        data->z     = (int16_t)(((uint16_t)rawData[5U] << 8U) | (uint16_t)rawData[4U]);
        data->valid = true;
    }

    return status;
}

int32_t BMI270_getGyroData(BMI270_GyroData_t *data)
{
    int32_t status;
    uint8_t rawData[6U];

    if (data == NULL)
    {
        return BMI270_ERR_INVALID_DATA;
    }

    /* Initialize validity flag */
    data->valid = false;

    /* Check initialization */
    if (!gTida010997Bmi270IsInitialized)
    {
        return BMI270_ERR_NOT_INITIALIZED;
    }

    /* Check sensor is enabled */
    if ((gTida010997Bmi270Config.sensorEnable != BMI270_SENSOR_GYRO_ONLY) &&
        (gTida010997Bmi270Config.sensorEnable != BMI270_SENSOR_BOTH))
    {
        return BMI270_ERR_SENSOR_DISABLED;
    }

    /* Read 6 bytes of gyroscope data */
    status = BMI270_readRegister(BMI270_REG_DATA_14, rawData, 6U);

    if (status == BMI270_OK)
    {
        /* Parse little-endian signed 16-bit data */
        data->x     = (int16_t)(((uint16_t)rawData[1U] << 8U) | (uint16_t)rawData[0U]);
        data->y     = (int16_t)(((uint16_t)rawData[3U] << 8U) | (uint16_t)rawData[2U]);
        data->z     = (int16_t)(((uint16_t)rawData[5U] << 8U) | (uint16_t)rawData[4U]);
        data->valid = true;
    }

    return status;
}

int32_t BMI270_getImuData(BMI270_ImuData_t *data)
{
    int32_t status;
    uint8_t rawData[12U];

    if (data == NULL)
    {
        return BMI270_ERR_INVALID_DATA;
    }

    /* Initialize validity flags */
    data->accel.valid = false;
    data->gyro.valid  = false;

    /* Check initialization */
    if (!gTida010997Bmi270IsInitialized)
    {
        return BMI270_ERR_NOT_INITIALIZED;
    }

    /* Read 12 bytes (accel + gyro data) */
    status = BMI270_readRegister(BMI270_REG_DATA_8, rawData, 12U);

    if (status == BMI270_OK)
    {
        /* Parse accelerometer data (bytes 0-5) */
        if ((gTida010997Bmi270Config.sensorEnable == BMI270_SENSOR_ACCEL_ONLY) ||
            (gTida010997Bmi270Config.sensorEnable == BMI270_SENSOR_BOTH))
        {
            data->accel.x     = (int16_t)(((uint16_t)rawData[1U] << 8U) | (uint16_t)rawData[0U]);
            data->accel.y     = (int16_t)(((uint16_t)rawData[3U] << 8U) | (uint16_t)rawData[2U]);
            data->accel.z     = (int16_t)(((uint16_t)rawData[5U] << 8U) | (uint16_t)rawData[4U]);
            data->accel.valid = true;
        }

        /* Parse gyroscope data (bytes 6-11) */
        if ((gTida010997Bmi270Config.sensorEnable == BMI270_SENSOR_GYRO_ONLY) ||
            (gTida010997Bmi270Config.sensorEnable == BMI270_SENSOR_BOTH))
        {
            data->gyro.x     = (int16_t)(((uint16_t)rawData[7U] << 8U) | (uint16_t)rawData[6U]);
            data->gyro.y     = (int16_t)(((uint16_t)rawData[9U] << 8U) | (uint16_t)rawData[8U]);
            data->gyro.z     = (int16_t)(((uint16_t)rawData[11U] << 8U) | (uint16_t)rawData[10U]);
            data->gyro.valid = true;
        }
    }

    return status;
}

int32_t BMI270_setPowerMode(BMI270_PowerMode_t mode)
{
    uint8_t regValue;

    (void)mode;  /* Reserved for future use */

    /* For now, we only support normal mode (advanced power save disabled) */
    /* PWR_CONF = 0x00 disables advanced power save */
    regValue = 0x00U;

    return BMI270_writeRegister(BMI270_REG_PWR_CONF, &regValue, 1U);
}

int32_t BMI270_setSensorEnable(BMI270_SensorEnable_t sensorEnable)
{
    uint8_t regValue = 0x00U;

    /* Build PWR_CTRL register value */
    switch (sensorEnable)
    {
        case BMI270_SENSOR_ACCEL_ONLY:
            regValue = BMI270_PWR_CTRL_ACC_EN;
            break;

        case BMI270_SENSOR_GYRO_ONLY:
            regValue = BMI270_PWR_CTRL_GYR_EN;
            break;

        case BMI270_SENSOR_BOTH:
            regValue = BMI270_PWR_CTRL_ACC_EN | BMI270_PWR_CTRL_GYR_EN;
            break;

        case BMI270_SENSOR_NONE:
        default:
            regValue = 0x00U;
            break;
    }

    return BMI270_writeRegister(BMI270_REG_PWR_CTRL, &regValue, 1U);
}

int32_t BMI270_getChipId(uint8_t *chipId)
{
    if (chipId == NULL)
    {
        return BMI270_ERR_INVALID_DATA;
    }

    return BMI270_readRegister(BMI270_REG_CHIP_ID, chipId, 1U);
}

float BMI270_convertAccel(int16_t raw, BMI270_AccelRange_t range)
{
    float scale;

    /* LSB to g conversion factors */
    switch (range)
    {
        case BMI270_ACCEL_RANGE_2G:
            scale = 2.0f / 32768.0f;
            break;
        case BMI270_ACCEL_RANGE_4G:
            scale = 4.0f / 32768.0f;
            break;
        case BMI270_ACCEL_RANGE_8G:
            scale = 8.0f / 32768.0f;
            break;
        case BMI270_ACCEL_RANGE_16G:
            scale = 16.0f / 32768.0f;
            break;
        default:
            scale = 2.0f / 32768.0f;
            break;
    }

    /* Convert to m/s^2 */
    return ((float)raw * scale * BMI270_GRAVITY_M_S2);
}

float BMI270_convertGyro(int16_t raw, BMI270_GyroRange_t range)
{
    float scale;

    /* LSB to degrees/s conversion factors */
    switch (range)
    {
        case BMI270_GYRO_RANGE_2000DPS:
            scale = 2000.0f / 32768.0f;
            break;
        case BMI270_GYRO_RANGE_1000DPS:
            scale = 1000.0f / 32768.0f;
            break;
        case BMI270_GYRO_RANGE_500DPS:
            scale = 500.0f / 32768.0f;
            break;
        case BMI270_GYRO_RANGE_250DPS:
            scale = 250.0f / 32768.0f;
            break;
        case BMI270_GYRO_RANGE_125DPS:
            scale = 125.0f / 32768.0f;
            break;
        default:
            scale = 2000.0f / 32768.0f;
            break;
    }

    return ((float)raw * scale);
}
