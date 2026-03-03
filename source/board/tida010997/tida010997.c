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

#include "tida010997.h"
#include "humidity_temperature/hdc3020.h"
#include "imu/bmi270.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t TIDA010997_HDC3020_init(TIDA010997_Sensor_Instance_t *instance,
                                              const TIDA010997_Sensor_InitParams_t *params);
static int32_t TIDA010997_HDC3020_deInit(TIDA010997_Sensor_Instance_t *instance);
static int32_t TIDA010997_HDC3020_trigger(TIDA010997_Sensor_Instance_t *instance);
static int32_t TIDA010997_HDC3020_getData(TIDA010997_Sensor_Instance_t *instance,
                                                 TIDA010997_Sensor_Data_t *data,
                                                 TIDA010997_Data_Type_t dataType);
static int32_t TIDA010997_HDC3020_getAllData(TIDA010997_Sensor_Instance_t *instance,
                                                    TIDA010997_Sensor_Data_t *dataArray,
                                                    uint32_t *dataCount);

static int32_t TIDA010997_BMI270_init(TIDA010997_Sensor_Instance_t *instance,
                                             const TIDA010997_Sensor_InitParams_t *params);
static int32_t TIDA010997_BMI270_deInit(TIDA010997_Sensor_Instance_t *instance);
static int32_t TIDA010997_BMI270_trigger(TIDA010997_Sensor_Instance_t *instance);
static int32_t TIDA010997_BMI270_getData(TIDA010997_Sensor_Instance_t *instance,
                                                TIDA010997_Sensor_Data_t *data,
                                                TIDA010997_Data_Type_t dataType);
static int32_t TIDA010997_BMI270_getAllData(TIDA010997_Sensor_Instance_t *instance,
                                                   TIDA010997_Sensor_Data_t *dataArray,
                                                   uint32_t *dataCount);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t TIDA010997_Sensor_initParamsDefault(TIDA010997_Sensor_InitParams_t *params,
                                            TIDA010997_Sensor_Type_t sensorType)
{
    int32_t status = TIDA010997_SENSOR_OK;

    if (params == NULL)
    {
        status = TIDA010997_SENSOR_ERR_INVALID_PARAMS;
    }
    else if (sensorType >= TIDA010997_SENSOR_TYPE_MAX)
    {
        status = TIDA010997_SENSOR_ERR_UNSUPPORTED_TYPE;
    }
    else
    {
        params->sensorType = sensorType;
        params->i2cHandle  = NULL;
        params->spiHandle  = NULL;

        if (sensorType == TIDA010997_SENSOR_TYPE_HDC3020)
        {
            params->config.hdc3020.lowPowerMode    = 0U;  /* LPM_0 */
            params->config.hdc3020.measurementRate = 1U;  /* 1 Hz */
            params->config.hdc3020.autoMeasurement = false;
        }
        else if (sensorType == TIDA010997_SENSOR_TYPE_BMI270)
        {
            params->config.bmi270.accelRange  = 0U;    /* 2g */
            params->config.bmi270.gyroRange   = 0U;    /* 2000 dps */
            params->config.bmi270.accelOdr    = 0x08U; /* 100 Hz */
            params->config.bmi270.gyroOdr     = 0x08U; /* 100 Hz */
            params->config.bmi270.accelEnable = true;
            params->config.bmi270.gyroEnable  = true;
        }
    }

    return status;
}

int32_t TIDA010997_Sensor_init(TIDA010997_Sensor_Instance_t *instance,
                               const TIDA010997_Sensor_InitParams_t *params)
{
    int32_t status = TIDA010997_SENSOR_OK;

    if (instance == NULL)
    {
        status = TIDA010997_SENSOR_ERR_INVALID_INSTANCE;
    }
    else if (params == NULL)
    {
        status = TIDA010997_SENSOR_ERR_INVALID_PARAMS;
    }
    else if (params->sensorType >= TIDA010997_SENSOR_TYPE_MAX)
    {
        status = TIDA010997_SENSOR_ERR_UNSUPPORTED_TYPE;
    }
    else
    {
        /* Initialize instance fields */
        instance->sensorType   = params->sensorType;
        instance->i2cHandle    = params->i2cHandle;
        instance->spiHandle    = params->spiHandle;
        instance->privateData  = NULL;
        instance->currentState = TIDA010997_SENSOR_STATE_UNCONFIGURED;

        /* Call device-specific init */
        switch (params->sensorType)
        {
            case TIDA010997_SENSOR_TYPE_HDC3020:
                status = TIDA010997_HDC3020_init(instance, params);
                break;

            case TIDA010997_SENSOR_TYPE_BMI270:
                status = TIDA010997_BMI270_init(instance, params);
                break;

            default:
                status = TIDA010997_SENSOR_ERR_UNSUPPORTED_TYPE;
                break;
        }

        if (status == TIDA010997_SENSOR_OK)
        {
            instance->currentState = TIDA010997_SENSOR_STATE_READY;
        }
    }

    return status;
}

int32_t TIDA010997_Sensor_deInit(TIDA010997_Sensor_Instance_t *instance)
{
    int32_t status = TIDA010997_SENSOR_OK;

    if (instance == NULL)
    {
        status = TIDA010997_SENSOR_ERR_INVALID_INSTANCE;
    }
    else
    {
        switch (instance->sensorType)
        {
            case TIDA010997_SENSOR_TYPE_HDC3020:
                status = TIDA010997_HDC3020_deInit(instance);
                break;

            case TIDA010997_SENSOR_TYPE_BMI270:
                status = TIDA010997_BMI270_deInit(instance);
                break;

            default:
                status = TIDA010997_SENSOR_ERR_UNSUPPORTED_TYPE;
                break;
        }

        instance->currentState = TIDA010997_SENSOR_STATE_UNCONFIGURED;
        instance->privateData  = NULL;
    }

    return status;
}

int32_t TIDA010997_Sensor_triggerMeasurement(TIDA010997_Sensor_Instance_t *instance)
{
    int32_t status = TIDA010997_SENSOR_OK;

    if (instance == NULL)
    {
        status = TIDA010997_SENSOR_ERR_INVALID_INSTANCE;
    }
    else if (instance->currentState != TIDA010997_SENSOR_STATE_READY)
    {
        status = TIDA010997_SENSOR_ERR_NOT_READY;
    }
    else
    {
        switch (instance->sensorType)
        {
            case TIDA010997_SENSOR_TYPE_HDC3020:
                status = TIDA010997_HDC3020_trigger(instance);
                break;

            case TIDA010997_SENSOR_TYPE_BMI270:
                status = TIDA010997_BMI270_trigger(instance);
                break;

            default:
                status = TIDA010997_SENSOR_ERR_UNSUPPORTED_TYPE;
                break;
        }
    }

    return status;
}

int32_t TIDA010997_Sensor_getData(TIDA010997_Sensor_Instance_t *instance,
                                  TIDA010997_Sensor_Data_t *data,
                                  TIDA010997_Data_Type_t dataType)
{
    int32_t status = TIDA010997_SENSOR_OK;

    if (instance == NULL)
    {
        status = TIDA010997_SENSOR_ERR_INVALID_INSTANCE;
    }
    else if (data == NULL)
    {
        status = TIDA010997_SENSOR_ERR_INVALID_PARAMS;
    }
    else if (instance->currentState != TIDA010997_SENSOR_STATE_READY)
    {
        status = TIDA010997_SENSOR_ERR_NOT_READY;
    }
    else if (dataType >= TIDA010997_DATA_MAX)
    {
        status = TIDA010997_SENSOR_ERR_INVALID_PARAMS;
    }
    else
    {
        /* Initialize data as invalid */
        data->dataType    = dataType;
        data->rawValue    = 0;
        data->scaledValue = 0;
        data->isValid     = false;

        switch (instance->sensorType)
        {
            case TIDA010997_SENSOR_TYPE_HDC3020:
                status = TIDA010997_HDC3020_getData(instance, data, dataType);
                break;

            case TIDA010997_SENSOR_TYPE_BMI270:
                status = TIDA010997_BMI270_getData(instance, data, dataType);
                break;

            default:
                status = TIDA010997_SENSOR_ERR_UNSUPPORTED_TYPE;
                break;
        }
    }

    return status;
}

int32_t TIDA010997_Sensor_getAllData(TIDA010997_Sensor_Instance_t *instance,
                                     TIDA010997_Sensor_Data_t *dataArray,
                                     uint32_t *dataCount)
{
    int32_t status = TIDA010997_SENSOR_OK;

    if (instance == NULL)
    {
        status = TIDA010997_SENSOR_ERR_INVALID_INSTANCE;
    }
    else if ((dataArray == NULL) || (dataCount == NULL))
    {
        status = TIDA010997_SENSOR_ERR_INVALID_PARAMS;
    }
    else if (instance->currentState != TIDA010997_SENSOR_STATE_READY)
    {
        status = TIDA010997_SENSOR_ERR_NOT_READY;
    }
    else if (*dataCount == 0U)
    {
        status = TIDA010997_SENSOR_ERR_INVALID_PARAMS;
    }
    else
    {
        switch (instance->sensorType)
        {
            case TIDA010997_SENSOR_TYPE_HDC3020:
                status = TIDA010997_HDC3020_getAllData(instance, dataArray, dataCount);
                break;

            case TIDA010997_SENSOR_TYPE_BMI270:
                status = TIDA010997_BMI270_getAllData(instance, dataArray, dataCount);
                break;

            default:
                status = TIDA010997_SENSOR_ERR_UNSUPPORTED_TYPE;
                break;
        }
    }

    return status;
}

int32_t TIDA010997_Sensor_getType(const TIDA010997_Sensor_Instance_t *instance,
                                  TIDA010997_Sensor_Type_t *type)
{
    int32_t status = TIDA010997_SENSOR_OK;

    if (instance == NULL)
    {
        status = TIDA010997_SENSOR_ERR_INVALID_INSTANCE;
    }
    else if (type == NULL)
    {
        status = TIDA010997_SENSOR_ERR_INVALID_PARAMS;
    }
    else
    {
        *type = instance->sensorType;
    }

    return status;
}

int32_t TIDA010997_Sensor_getState(const TIDA010997_Sensor_Instance_t *instance,
                                   TIDA010997_Sensor_State_t *state)
{
    int32_t status = TIDA010997_SENSOR_OK;

    if (instance == NULL)
    {
        status = TIDA010997_SENSOR_ERR_INVALID_INSTANCE;
    }
    else if (state == NULL)
    {
        status = TIDA010997_SENSOR_ERR_INVALID_PARAMS;
    }
    else
    {
        *state = instance->currentState;
    }

    return status;
}

/* ========================================================================== */
/*                       HDC3020 Wrapper Functions                            */
/* ========================================================================== */

static int32_t TIDA010997_HDC3020_init(TIDA010997_Sensor_Instance_t *instance,
                                              const TIDA010997_Sensor_InitParams_t *params)
{
    int32_t status;
    HDC3020_Config_t hdcConfig;

    if (params->i2cHandle == NULL)
    {
        return TIDA010997_SENSOR_ERR_INVALID_PARAMS;
    }

    /* Map generic config to HDC3020-specific config */
    hdcConfig.lpm  = (HDC3020_Lpm_t)params->config.hdc3020.lowPowerMode;
    hdcConfig.rate = (HDC3020_Rate_t)params->config.hdc3020.measurementRate;
    hdcConfig.mode = params->config.hdc3020.autoMeasurement ?
                     HDC3020_MODE_AUTO_MEASUREMENT :
                     HDC3020_MODE_TRIGGER_ON_DEMAND;

    status = HDC3020_init(params->i2cHandle, &hdcConfig);

    /* Map HDC3020 error codes to generic error codes */
    if (status == HDC3020_OK)
    {
        status = TIDA010997_SENSOR_OK;
    }
    else if (status == HDC3020_ERR_I2C)
    {
        status = TIDA010997_SENSOR_ERR_COMM;
    }
    else if (status == HDC3020_ERR_CRC)
    {
        status = TIDA010997_SENSOR_ERR_CRC;
    }
    else
    {
        status = TIDA010997_SENSOR_ERR_INIT_FAILED;
    }

    (void)instance;  /* Instance not used - HDC3020 uses global state */

    return status;
}

static int32_t TIDA010997_HDC3020_deInit(TIDA010997_Sensor_Instance_t *instance)
{
    int32_t status;

    status = HDC3020_reset();

    if (status == HDC3020_OK)
    {
        status = TIDA010997_SENSOR_OK;
    }
    else
    {
        status = TIDA010997_SENSOR_ERR_COMM;
    }

    (void)instance;

    return status;
}

static int32_t TIDA010997_HDC3020_trigger(TIDA010997_Sensor_Instance_t *instance)
{
    int32_t status;

    status = HDC3020_triggerMeasurement(HDC3020_LPM_0);

    if (status == HDC3020_OK)
    {
        status = TIDA010997_SENSOR_OK;
    }
    else
    {
        status = TIDA010997_SENSOR_ERR_COMM;
    }

    (void)instance;

    return status;
}

static int32_t TIDA010997_HDC3020_getData(TIDA010997_Sensor_Instance_t *instance,
                                                 TIDA010997_Sensor_Data_t *data,
                                                 TIDA010997_Data_Type_t dataType)
{
    int32_t status;
    HDC3020_Data_t hdcData;
    HDC3020_Sensor_t sensorType;

    /* Only temperature and humidity are supported for HDC3020 */
    if (dataType == TIDA010997_DATA_TEMPERATURE)
    {
        sensorType = HDC3020_SENSOR_TEMPERATURE;
    }
    else if (dataType == TIDA010997_DATA_HUMIDITY)
    {
        sensorType = HDC3020_SENSOR_HUMIDITY;
    }
    else
    {
        return TIDA010997_SENSOR_ERR_INVALID_PARAMS;
    }

    status = HDC3020_getData(&hdcData, sensorType);

    if (status == HDC3020_OK)
    {
        if (hdcData.valid)
        {
            if (dataType == TIDA010997_DATA_TEMPERATURE)
            {
                data->rawValue    = (int32_t)hdcData.temperature;
                /* Temperature: scaledValue = -45000 + (175000 * raw) / 65535 (milli-degrees C) */
                /* Use 64-bit arithmetic to avoid overflow */
                data->scaledValue = -45000 + (int32_t)(((uint64_t)175000U * (uint64_t)hdcData.temperature) / 65535U);
            }
            else
            {
                data->rawValue    = (int32_t)hdcData.humidity;
                /* Humidity: scaledValue = (100000 * raw) / 65535 (milli-percent RH) */
                /* Use 64-bit arithmetic to avoid overflow */
                data->scaledValue = (int32_t)(((uint64_t)100000U * (uint64_t)hdcData.humidity) / 65535U);
            }
            data->isValid = true;
            status = TIDA010997_SENSOR_OK;
        }
        else
        {
            status = TIDA010997_SENSOR_ERR_INVALID_DATA;
        }
    }
    else if (status == HDC3020_ERR_I2C)
    {
        status = TIDA010997_SENSOR_ERR_COMM;
    }
    else if (status == HDC3020_ERR_CRC)
    {
        status = TIDA010997_SENSOR_ERR_CRC;
    }
    else
    {
        status = TIDA010997_SENSOR_ERR_INVALID_DATA;
    }

    (void)instance;

    return status;
}

static int32_t TIDA010997_HDC3020_getAllData(TIDA010997_Sensor_Instance_t *instance,
                                                    TIDA010997_Sensor_Data_t *dataArray,
                                                    uint32_t *dataCount)
{
    int32_t status;
    HDC3020_Data_t hdcData;
    uint32_t maxCount = *dataCount;
    uint32_t actualCount = 0U;

    status = HDC3020_getData(&hdcData, HDC3020_SENSOR_BOTH);

    if (status == HDC3020_OK && hdcData.valid)
    {
        /* Add temperature data */
        if (actualCount < maxCount)
        {
            dataArray[actualCount].dataType    = TIDA010997_DATA_TEMPERATURE;
            dataArray[actualCount].rawValue    = (int32_t)hdcData.temperature;
            /* Use 64-bit arithmetic to avoid overflow */
            dataArray[actualCount].scaledValue = -45000 + (int32_t)(((uint64_t)175000U * (uint64_t)hdcData.temperature) / 65535U);
            dataArray[actualCount].isValid     = true;
            actualCount++;
        }

        /* Add humidity data */
        if (actualCount < maxCount)
        {
            dataArray[actualCount].dataType    = TIDA010997_DATA_HUMIDITY;
            dataArray[actualCount].rawValue    = (int32_t)hdcData.humidity;
            /* Use 64-bit arithmetic to avoid overflow */
            dataArray[actualCount].scaledValue = (int32_t)(((uint64_t)100000U * (uint64_t)hdcData.humidity) / 65535U);
            dataArray[actualCount].isValid     = true;
            actualCount++;
        }

        *dataCount = actualCount;
        status = TIDA010997_SENSOR_OK;
    }
    else if (status == HDC3020_ERR_I2C)
    {
        status = TIDA010997_SENSOR_ERR_COMM;
    }
    else if (status == HDC3020_ERR_CRC)
    {
        status = TIDA010997_SENSOR_ERR_CRC;
    }
    else
    {
        status = TIDA010997_SENSOR_ERR_INVALID_DATA;
    }

    (void)instance;

    return status;
}

/* ========================================================================== */
/*                       BMI270 Wrapper Functions                             */
/* ========================================================================== */

static int32_t TIDA010997_BMI270_init(TIDA010997_Sensor_Instance_t *instance,
                                             const TIDA010997_Sensor_InitParams_t *params)
{
    int32_t status;
    BMI270_Config_t bmiConfig;

    if (params->spiHandle == NULL)
    {
        return TIDA010997_SENSOR_ERR_INVALID_PARAMS;
    }

    /* Map generic config to BMI270-specific config */
    bmiConfig.powerMode    = BMI270_POWER_MODE_NORMAL;
    bmiConfig.accelRange   = (BMI270_AccelRange_t)params->config.bmi270.accelRange;
    bmiConfig.gyroRange    = (BMI270_GyroRange_t)params->config.bmi270.gyroRange;
    bmiConfig.accelOdr     = (BMI270_Odr_t)params->config.bmi270.accelOdr;
    bmiConfig.gyroOdr      = (BMI270_Odr_t)params->config.bmi270.gyroOdr;
    bmiConfig.accelBwp     = BMI270_BWP_NORMAL;
    bmiConfig.gyroBwp      = BMI270_BWP_NORMAL;
    bmiConfig.accelFilterPerf = true;
    bmiConfig.gyroFilterPerf  = true;
    bmiConfig.gyroNoisePerf   = true;

    if (params->config.bmi270.accelEnable && params->config.bmi270.gyroEnable)
    {
        bmiConfig.sensorEnable = BMI270_SENSOR_BOTH;
    }
    else if (params->config.bmi270.accelEnable)
    {
        bmiConfig.sensorEnable = BMI270_SENSOR_ACCEL_ONLY;
    }
    else if (params->config.bmi270.gyroEnable)
    {
        bmiConfig.sensorEnable = BMI270_SENSOR_GYRO_ONLY;
    }
    else
    {
        bmiConfig.sensorEnable = BMI270_SENSOR_NONE;
    }

    status = BMI270_init(params->spiHandle, &bmiConfig);

    /* Map BMI270 error codes to generic error codes */
    if (status == BMI270_OK)
    {
        status = TIDA010997_SENSOR_OK;
    }
    else if (status == BMI270_ERR_SPI)
    {
        status = TIDA010997_SENSOR_ERR_COMM;
    }
    else if (status == BMI270_ERR_TIMEOUT)
    {
        status = TIDA010997_SENSOR_ERR_TIMEOUT;
    }
    else
    {
        status = TIDA010997_SENSOR_ERR_INIT_FAILED;
    }

    (void)instance;  /* Instance not used - BMI270 uses global state */

    return status;
}

static int32_t TIDA010997_BMI270_deInit(TIDA010997_Sensor_Instance_t *instance)
{
    int32_t status;

    status = BMI270_reset();

    if (status == BMI270_OK)
    {
        status = TIDA010997_SENSOR_OK;
    }
    else
    {
        status = TIDA010997_SENSOR_ERR_COMM;
    }

    (void)instance;

    return status;
}

static int32_t TIDA010997_BMI270_trigger(TIDA010997_Sensor_Instance_t *instance)
{
    /* BMI270 operates in continuous mode, no trigger needed */
    (void)instance;
    return TIDA010997_SENSOR_OK;
}

static int32_t TIDA010997_BMI270_getData(TIDA010997_Sensor_Instance_t *instance,
                                                TIDA010997_Sensor_Data_t *data,
                                                TIDA010997_Data_Type_t dataType)
{
    int32_t status = TIDA010997_SENSOR_OK;
    BMI270_AccelData_t accelData;
    BMI270_GyroData_t gyroData;

    /* Only accel/gyro data types are supported for BMI270 */
    if (dataType == TIDA010997_DATA_ACCEL_X ||
        dataType == TIDA010997_DATA_ACCEL_Y ||
        dataType == TIDA010997_DATA_ACCEL_Z)
    {
        status = BMI270_getAccelData(&accelData);
        if (status == BMI270_OK && accelData.valid)
        {
            int16_t rawVal = 0;
            if (dataType == TIDA010997_DATA_ACCEL_X)
            {
                rawVal = accelData.x;
            }
            else if (dataType == TIDA010997_DATA_ACCEL_Y)
            {
                rawVal = accelData.y;
            }
            else
            {
                rawVal = accelData.z;
            }

            data->rawValue = (int32_t)rawVal;
            /* Convert to milli-g (assuming 2g range: 1 LSB = 2000/32768 mg) */
            data->scaledValue = (int32_t)(((int32_t)rawVal * 2000) / 32768);
            data->isValid = true;
            status = TIDA010997_SENSOR_OK;
        }
        else
        {
            status = (status == BMI270_ERR_SPI) ?
                     TIDA010997_SENSOR_ERR_COMM : TIDA010997_SENSOR_ERR_INVALID_DATA;
        }
    }
    else if (dataType == TIDA010997_DATA_GYRO_X ||
             dataType == TIDA010997_DATA_GYRO_Y ||
             dataType == TIDA010997_DATA_GYRO_Z)
    {
        status = BMI270_getGyroData(&gyroData);
        if (status == BMI270_OK && gyroData.valid)
        {
            int16_t rawVal = 0;
            if (dataType == TIDA010997_DATA_GYRO_X)
            {
                rawVal = gyroData.x;
            }
            else if (dataType == TIDA010997_DATA_GYRO_Y)
            {
                rawVal = gyroData.y;
            }
            else
            {
                rawVal = gyroData.z;
            }

            data->rawValue = (int32_t)rawVal;
            /* Convert to milli-dps (assuming 2000dps range: 1 LSB = 2000000/32768 mdps) */
            data->scaledValue = (int32_t)(((int32_t)rawVal * 2000000) / 32768);
            data->isValid = true;
            status = TIDA010997_SENSOR_OK;
        }
        else
        {
            status = (status == BMI270_ERR_SPI) ?
                     TIDA010997_SENSOR_ERR_COMM : TIDA010997_SENSOR_ERR_INVALID_DATA;
        }
    }
    else
    {
        status = TIDA010997_SENSOR_ERR_INVALID_PARAMS;
    }

    (void)instance;

    return status;
}

static int32_t TIDA010997_BMI270_getAllData(TIDA010997_Sensor_Instance_t *instance,
                                                   TIDA010997_Sensor_Data_t *dataArray,
                                                   uint32_t *dataCount)
{
    int32_t status;
    BMI270_ImuData_t imuData;
    uint32_t maxCount = *dataCount;
    uint32_t actualCount = 0U;

    status = BMI270_getImuData(&imuData);

    if (status == BMI270_OK)
    {
        /* Add accel X */
        if (actualCount < maxCount && imuData.accel.valid)
        {
            dataArray[actualCount].dataType    = TIDA010997_DATA_ACCEL_X;
            dataArray[actualCount].rawValue    = (int32_t)imuData.accel.x;
            dataArray[actualCount].scaledValue = (int32_t)(((int32_t)imuData.accel.x * 2000) / 32768);
            dataArray[actualCount].isValid     = true;
            actualCount++;
        }
        /* Add accel Y */
        if (actualCount < maxCount && imuData.accel.valid)
        {
            dataArray[actualCount].dataType    = TIDA010997_DATA_ACCEL_Y;
            dataArray[actualCount].rawValue    = (int32_t)imuData.accel.y;
            dataArray[actualCount].scaledValue = (int32_t)(((int32_t)imuData.accel.y * 2000) / 32768);
            dataArray[actualCount].isValid     = true;
            actualCount++;
        }
        /* Add accel Z */
        if (actualCount < maxCount && imuData.accel.valid)
        {
            dataArray[actualCount].dataType    = TIDA010997_DATA_ACCEL_Z;
            dataArray[actualCount].rawValue    = (int32_t)imuData.accel.z;
            dataArray[actualCount].scaledValue = (int32_t)(((int32_t)imuData.accel.z * 2000) / 32768);
            dataArray[actualCount].isValid     = true;
            actualCount++;
        }
        /* Add gyro X */
        if (actualCount < maxCount && imuData.gyro.valid)
        {
            dataArray[actualCount].dataType    = TIDA010997_DATA_GYRO_X;
            dataArray[actualCount].rawValue    = (int32_t)imuData.gyro.x;
            dataArray[actualCount].scaledValue = (int32_t)(((int32_t)imuData.gyro.x * 2000000) / 32768);
            dataArray[actualCount].isValid     = true;
            actualCount++;
        }
        /* Add gyro Y */
        if (actualCount < maxCount && imuData.gyro.valid)
        {
            dataArray[actualCount].dataType    = TIDA010997_DATA_GYRO_Y;
            dataArray[actualCount].rawValue    = (int32_t)imuData.gyro.y;
            dataArray[actualCount].scaledValue = (int32_t)(((int32_t)imuData.gyro.y * 2000000) / 32768);
            dataArray[actualCount].isValid     = true;
            actualCount++;
        }
        /* Add gyro Z */
        if (actualCount < maxCount && imuData.gyro.valid)
        {
            dataArray[actualCount].dataType    = TIDA010997_DATA_GYRO_Z;
            dataArray[actualCount].rawValue    = (int32_t)imuData.gyro.z;
            dataArray[actualCount].scaledValue = (int32_t)(((int32_t)imuData.gyro.z * 2000000) / 32768);
            dataArray[actualCount].isValid     = true;
            actualCount++;
        }

        *dataCount = actualCount;
        status = TIDA010997_SENSOR_OK;
    }
    else if (status == BMI270_ERR_SPI)
    {
        status = TIDA010997_SENSOR_ERR_COMM;
    }
    else
    {
        status = TIDA010997_SENSOR_ERR_INVALID_DATA;
    }

    (void)instance;

    return status;
}
