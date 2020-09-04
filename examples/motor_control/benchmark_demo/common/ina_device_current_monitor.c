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

#include <stdint.h>
#include "ina_device_current_monitor.h"
#include "ti_drivers_open_close.h"

uint32_t gAppPowerStats[NUM_OF_INA_DEVICES];
uint32_t gAppBusVoltageStats[NUM_OF_INA_DEVICES];
uint32_t gAppShuntVoltageStats[NUM_OF_INA_DEVICES];
uint32_t gAppCurrentStats[NUM_OF_INA_DEVICES];

#ifndef APP_ENABLE_INA_CURRENT_MONITOR

int32_t current_monitor_init()
{
    /* nothing to do */
    return 0;
}

int32_t current_monitor_run()
{
    /* nothing to do */
    return 0;
}

#endif

#ifdef APP_ENABLE_INA_CURRENT_MONITOR

/* INA Device Register Address Offsets */
#define CONFIGURATION_REG_ADDR_OFFSET     (0x00U)
#define SHUNT_VOLTAGE_REG_ADDR_OFFSET     (0x01U)
#define BUS_VOLTAGE_REG_ADDR_OFFSET       (0x02U)
#define POWER_REG_ADDR_OFFSET             (0x03U)
#define CURRENT_REG_ADDR_OFFSET           (0x04U)
#define CALIBRATION_REG_ADDR_OFFSET       (0x05U)

/* INA Device Register masks */
#define SHUNT_VOLTAGE_REG_MASK            (0x7fffU)
#define BUS_VOLTAGE_REG_MASK              (0x7fffU)
#define POWER_REG_MASK                    (0xffffU)
#define CURRENT_REG_MASK                  (0x7fffU)
#define SIGNED_REG_MASK                   (0x8000U)

/* INA Device Constant values */
#define SHUNT_MICROVOLT_CONSTANT          (float)(2.50) // constant 2.5  uV/bit
#define BUS_MILIVOLT_CONSTANT             (float)(1.25) // constant 1.25 mV/bit
#define POWER_MICROWATT_CONSTANT          (float)(25.0) // constant 25 * current_LSB
#define INA_DEFAULT_CONFIG_VAL            (uint16_t)(0x4737U)
#define INA_CALIBRATION_CONSTANT          (uint32_t)(5120000U)

/*
 *  Structure defining INA Device calibration.
 */
typedef struct CalibrationParams
{
    uint16_t shuntMiliOhms;
    uint16_t lsbMicroAmps;
} calParams_t;

/*
 * Structure defining INA Device configuration.
 */
typedef struct inaCfgObj
{
    char deviceID[20];
    calParams_t inaCalParams;
    uint8_t slaveAddr;
} inaCfgObj_t;

inaCfgObj_t inaDevice[NUM_OF_INA_DEVICES] =
        { { "VDD_CORE",    { 2, 100 }, 0x40 },
          { "VDDAR_CORE",  { 10, 20 }, 0x41 },
          { "VDDS_DDR",    { 10, 20 }, 0x46 },
          { "SoC_DVDD1V8", { 10, 20 }, 0x4B },
          { "SoC_DVDD3V3", { 10, 20 }, 0x4C },
          { "SoC_AVDD1V8", { 10, 20 }, 0x4E }
        };

static int32_t INA_read_register(I2C_Handle handle, uint8_t slaveAddress,
                                uint8_t regAddr, uint16_t *regData);
static int32_t INA_set_default_config(I2C_Handle handle, inaCfgObj_t *inaDevice);
static int32_t INA_set_calibration(I2C_Handle handle, inaCfgObj_t *inaDevice);
static int32_t INA_read_shunt_microvolts(I2C_Handle handle,
                                        inaCfgObj_t *inaDevice,
                                        float *shuntVoltage);
static int32_t INA_read_bus_milivolts(I2C_Handle handle, inaCfgObj_t *inaDevice,
                                     float *busVoltage);
static int32_t INA_read_bus_microwatts(I2C_Handle handle, inaCfgObj_t *inaDevice,
                                      float *powerMeasured);
static int32_t INA_read_bus_microamps(I2C_Handle handle, inaCfgObj_t *inaDevice,
                                     float *currentMeasured);

int32_t current_monitor_run()
{
    int32_t ret = SystemP_SUCCESS;
    float retData = 0;
    uint32_t index;

    for (index = 0; index < NUM_OF_INA_DEVICES; index++)
    {

        ret = INA_read_bus_microwatts(gI2cHandle[CONFIG_I2C_INA_DEVICE],
                                      &inaDevice[index], &retData);
        if (ret != SystemP_SUCCESS)
        {
            break;
        }
        else
        {
            gAppPowerStats[index] = (uint32_t ) retData / 1000;
        }

        ret = INA_read_bus_milivolts(gI2cHandle[CONFIG_I2C_INA_DEVICE], &inaDevice[index],
                                             &retData);
        if (ret != SystemP_SUCCESS)
        {
            break;
        }
        else
        {
            gAppBusVoltageStats[index] = (uint32_t ) retData;
        }

        ret = INA_read_bus_microamps(gI2cHandle[CONFIG_I2C_INA_DEVICE], &inaDevice[index],
                                     &retData);
        if (ret != SystemP_SUCCESS)
        {
            break;
        }
        else
        {
            gAppCurrentStats[index] = (uint32_t ) retData / 1000;
        }

    }

    return SystemP_SUCCESS;
}

int32_t current_monitor_init()
{
    int32_t ret = SystemP_SUCCESS;
    uint32_t index;

    for (index = 0; index < NUM_OF_INA_DEVICES; index++)
    {
        ret = INA_set_default_config(gI2cHandle[CONFIG_I2C_INA_DEVICE],
                                     &inaDevice[index]);
        if (ret != SystemP_SUCCESS)
        {
            break;
        }

        ret = INA_set_calibration(gI2cHandle[CONFIG_I2C_INA_DEVICE], &inaDevice[index]);

        if (ret != SystemP_SUCCESS)
        {
            break;
        }
    }

    return SystemP_SUCCESS;
}

static int32_t INA_read_register(I2C_Handle handle, uint8_t slaveAddress,
                                uint8_t regAddr, uint16_t *regData)
{
    int32_t ret = SystemP_SUCCESS;
    uint8_t rx[2];
    I2C_Transaction transaction;

    /* Initializes the I2C transaction structure with default values */
    I2C_Transaction_init(&transaction);

    transaction.slaveAddress = slaveAddress;
    transaction.writeBuf = &regAddr;
    transaction.writeCount = 1;
    transaction.readBuf = NULL;
    transaction.readCount = 0;

    ret = I2C_transfer(handle, &transaction);
    if (ret != I2C_STS_SUCCESS)
    {
        ret = SystemP_FAILURE;
        return ret;
    }

    transaction.writeBuf = NULL;
    transaction.writeCount = 0;
    transaction.readBuf = &rx[0];
    transaction.readCount = 2;

    ret = I2C_transfer(handle, &transaction);
    if (ret != I2C_STS_SUCCESS)
    {
        ret = SystemP_FAILURE;
        return ret;
    }
    else
    {
        ret = SystemP_SUCCESS;
    }

    /* Note:- Slave device responds with MSB first for the read sequence sent */
    *regData = ((((uint16_t) rx[0]) << 8) | ((uint16_t) rx[1]));

    return ret;
}

static int32_t INA_write_register(I2C_Handle handle, uint8_t slaveAddress,
                                 uint8_t regAddr, uint16_t regData)
{
    int32_t ret = SystemP_SUCCESS;
    uint8_t tx[3];
    I2C_Transaction transaction;

    /* Set default transaction parameters */
    I2C_Transaction_init(&transaction);

    transaction.slaveAddress = slaveAddress;
    transaction.writeBuf = &tx[0];
    transaction.writeCount = 3;
    transaction.readBuf = NULL;
    transaction.readCount = 0;

    tx[0] = regAddr;

    /* MSB of 16-bit data should be sent first followed by the LSB */
    tx[1] = (uint8_t) ((regData & 0xFF00) >> 8);
    tx[2] = (uint8_t) (regData & 0x00FF);

    ret = I2C_transfer(handle, &transaction);
    if (ret != I2C_STS_SUCCESS)
    {
        ret = SystemP_FAILURE;
    }
    else
    {
        ret = SystemP_SUCCESS;
    }

    return ret;
}

static int32_t INA_set_default_config(I2C_Handle handle, inaCfgObj_t *inaDevice)
{
    int32_t ret = SystemP_SUCCESS;

    ret = INA_write_register(handle, inaDevice->slaveAddr,
                             CONFIGURATION_REG_ADDR_OFFSET,
                             INA_DEFAULT_CONFIG_VAL);

    return ret;
}

static int32_t INA_set_calibration(I2C_Handle handle, inaCfgObj_t *inaDevice)
{
    int32_t ret = SystemP_SUCCESS;
    calParams_t *calParams = &inaDevice->inaCalParams;
    uint32_t calibration = INA_CALIBRATION_CONSTANT
            / ((uint32_t) (calParams->shuntMiliOhms * calParams->lsbMicroAmps));

    if (calibration > INT16_MAX)
    {
        ret = SystemP_FAILURE;
    }
    else
    {
        ret = INA_write_register(handle, inaDevice->slaveAddr,
                                 CALIBRATION_REG_ADDR_OFFSET,
                                 (uint16_t) calibration);
    }

    return ret;
}

static int32_t INA_read_bus_microwatts(I2C_Handle handle, inaCfgObj_t *inaDevice,
                                      float *powerMeasured)
{
    int32_t ret = SystemP_SUCCESS;
    uint16_t readRegData = 0;
    calParams_t *calParams = &inaDevice->inaCalParams;

    ret = INA_read_register(handle, inaDevice->slaveAddr, POWER_REG_ADDR_OFFSET,
                            &readRegData);
    if (ret != SystemP_SUCCESS)
    {
        return ret;
    }
    else
    {
        *powerMeasured = (readRegData & POWER_REG_MASK)
                * calParams->lsbMicroAmps * POWER_MICROWATT_CONSTANT;
    }

    return ret;
}

static int32_t INA_read_shunt_microvolts(I2C_Handle handle,
                                        inaCfgObj_t *inaDevice,
                                        float *shuntVoltage)
{
    int32_t ret = SystemP_SUCCESS;
    float sign = 1;
    uint16_t readRegData = 0;

    ret = INA_read_register(handle, inaDevice->slaveAddr,
                            SHUNT_VOLTAGE_REG_ADDR_OFFSET,
                            &readRegData);
    if (ret == SystemP_SUCCESS)
    {
        if (readRegData & SIGNED_REG_MASK)
        {
            readRegData = (~readRegData) + 1;
            sign = -1;
        }
        else
        {
            *shuntVoltage = (readRegData & SHUNT_VOLTAGE_REG_MASK)
                    * SHUNT_MICROVOLT_CONSTANT * sign;
        }
    }

    return ret;
}

static int32_t INA_read_bus_milivolts(I2C_Handle handle, inaCfgObj_t *inaDevice,
                                     float *busVoltage)
{
    int32_t ret = SystemP_SUCCESS;
    uint16_t readRegData = 0;

    ret = INA_read_register(handle, inaDevice->slaveAddr,
                            BUS_VOLTAGE_REG_ADDR_OFFSET,
                            &readRegData);
    if (ret == SystemP_SUCCESS)
    {
        *busVoltage = (readRegData & BUS_VOLTAGE_REG_MASK)
                * BUS_MILIVOLT_CONSTANT;
    }

    return ret;
}

static int32_t INA_read_bus_microamps(I2C_Handle handle, inaCfgObj_t *inaDevice,
                                     float *currentMeasured)
{
    int32_t ret = SystemP_SUCCESS;
    float sign = 1;
    uint16_t readRegData = 0;
    calParams_t *calParams = &inaDevice->inaCalParams;

    ret = INA_read_register(handle, inaDevice->slaveAddr,
                            CURRENT_REG_ADDR_OFFSET,
                            &readRegData);
    if (ret == SystemP_SUCCESS)
    {
        if (readRegData & SIGNED_REG_MASK)
        {
            readRegData = (~readRegData) + 1;
            sign = -1;
        }
        else
        {
            *currentMeasured = (readRegData & CURRENT_REG_MASK)
                    * calParams->lsbMicroAmps * sign;
        }
    }

    return ret;
}

#endif /* APP_ENABLE_INA_CURRENT_MONITOR */
