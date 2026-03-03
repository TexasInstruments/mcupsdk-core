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
 * \defgroup BOARD_TIDA010997_MODULE APIs for TIDA-010997 Booster Pack
 * \ingroup BOARD_MODULE
 *
 * This module provides a generic sensor abstraction layer for the TIDA-010997
 * sensor booster pack. The API allows uniform access to all sensors through
 * a common interface.
 *
 * The booster pack contains the following sensors:
 * - **HDC3020**: Temperature and humidity sensor (I2C interface)
 * - **BMI270**: 6-axis IMU - accelerometer + gyroscope (SPI interface)
 *
 * @{
 */

/**
 * \file tida010997.h
 * \brief TIDA-010997 Booster Pack Sensor HAL
 */

#ifndef TIDA010997_H_
#define TIDA010997_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>
#include <drivers/i2c.h>
#include <drivers/mcspi.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * \anchor TIDA010997_Sensor_Error
 * \name Sensor HAL Error Codes
 * @{
 */
#define TIDA010997_SENSOR_OK                    (0)   /**< Operation successful */
#define TIDA010997_SENSOR_ERR_INVALID_INSTANCE  (-1)  /**< Invalid instance pointer */
#define TIDA010997_SENSOR_ERR_INVALID_PARAMS    (-2)  /**< Invalid parameters */
#define TIDA010997_SENSOR_ERR_UNSUPPORTED_TYPE  (-3)  /**< Unsupported sensor type */
#define TIDA010997_SENSOR_ERR_COMM              (-4)  /**< Communication error (I2C/SPI) */
#define TIDA010997_SENSOR_ERR_NOT_READY         (-5)  /**< Sensor not ready */
#define TIDA010997_SENSOR_ERR_INVALID_DATA      (-6)  /**< Invalid data received */
#define TIDA010997_SENSOR_ERR_CRC               (-7)  /**< CRC verification failed */
#define TIDA010997_SENSOR_ERR_TIMEOUT           (-8)  /**< Operation timed out */
#define TIDA010997_SENSOR_ERR_INIT_FAILED       (-9)  /**< Initialization failed */
/** @} */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 * \brief Sensor instance states
 */
typedef enum
{
    TIDA010997_SENSOR_STATE_UNCONFIGURED = 0U,  /**< Instance not initialized */
    TIDA010997_SENSOR_STATE_READY        = 1U,  /**< Instance initialized and ready */
    TIDA010997_SENSOR_STATE_BUSY         = 2U   /**< Measurement in progress */
} TIDA010997_Sensor_State_t;

/**
 * \brief Supported sensor types
 */
typedef enum
{
    TIDA010997_SENSOR_TYPE_HDC3020 = 0U,  /**< HDC3020 Temperature/Humidity sensor */
    TIDA010997_SENSOR_TYPE_BMI270,        /**< BMI270 6-axis IMU */
    TIDA010997_SENSOR_TYPE_MAX
} TIDA010997_Sensor_Type_t;

/**
 * \brief Sensor data types
 */
typedef enum
{
    TIDA010997_DATA_TEMPERATURE = 0U,  /**< Temperature data (milli-degrees C) */
    TIDA010997_DATA_HUMIDITY,          /**< Humidity data (milli-percent RH) */
    TIDA010997_DATA_ACCEL_X,           /**< Accelerometer X-axis (milli-g) */
    TIDA010997_DATA_ACCEL_Y,           /**< Accelerometer Y-axis (milli-g) */
    TIDA010997_DATA_ACCEL_Z,           /**< Accelerometer Z-axis (milli-g) */
    TIDA010997_DATA_GYRO_X,            /**< Gyroscope X-axis (milli-dps) */
    TIDA010997_DATA_GYRO_Y,            /**< Gyroscope Y-axis (milli-dps) */
    TIDA010997_DATA_GYRO_Z,            /**< Gyroscope Z-axis (milli-dps) */
    TIDA010997_DATA_MAX
} TIDA010997_Data_Type_t;

/**
 * \brief Generic sensor data structure
 */
typedef struct
{
    TIDA010997_Data_Type_t dataType;     /**< Type of data */
    int32_t                rawValue;     /**< Raw sensor value */
    int32_t                scaledValue;  /**< Scaled value in milli-units */
    bool                   isValid;      /**< Validity flag */
} TIDA010997_Sensor_Data_t;

/**
 * \brief HDC3020 specific configuration
 */
typedef struct
{
    uint8_t  lowPowerMode;      /**< Low power mode (0-3) */
    uint8_t  measurementRate;   /**< Measurement rate for auto mode */
    bool     autoMeasurement;   /**< Enable auto measurement mode */
} TIDA010997_HDC3020_Cfg_t;

/**
 * \brief BMI270 specific configuration
 */
typedef struct
{
    uint8_t  accelRange;        /**< Accelerometer range (0=2g, 1=4g, 2=8g, 3=16g) */
    uint8_t  gyroRange;         /**< Gyroscope range (0=2000dps, 1=1000dps, etc.) */
    uint8_t  accelOdr;          /**< Accelerometer output data rate */
    uint8_t  gyroOdr;           /**< Gyroscope output data rate */
    bool     accelEnable;       /**< Enable accelerometer */
    bool     gyroEnable;        /**< Enable gyroscope */
} TIDA010997_BMI270_Cfg_t;

/**
 * \brief Sensor initialization parameters
 */
typedef struct
{
    TIDA010997_Sensor_Type_t  sensorType;     /**< Type of sensor */
    I2C_Handle                i2cHandle;      /**< I2C handle (for HDC3020) */
    MCSPI_Handle              spiHandle;      /**< SPI handle (for BMI270) */
    union
    {
        TIDA010997_HDC3020_Cfg_t hdc3020;     /**< HDC3020 configuration */
        TIDA010997_BMI270_Cfg_t  bmi270;      /**< BMI270 configuration */
    } config;
} TIDA010997_Sensor_InitParams_t;

/**
 * \brief Forward declaration of sensor instance
 */
typedef struct TIDA010997_Sensor_Instance_Tag TIDA010997_Sensor_Instance_t;

/**
 * \brief Sensor instance structure
 */
struct TIDA010997_Sensor_Instance_Tag
{
    TIDA010997_Sensor_State_t  currentState;   /**< Current state */
    TIDA010997_Sensor_Type_t   sensorType;     /**< Sensor type */
    I2C_Handle                 i2cHandle;      /**< I2C handle */
    MCSPI_Handle               spiHandle;      /**< SPI handle */
    void                      *privateData;    /**< Device-specific data */
};

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * \brief Set default initialization parameters
 *
 * \param[out] params     Pointer to initialization parameters
 * \param[in]  sensorType Sensor type to set defaults for
 *
 * \return TIDA010997_SENSOR_OK on success, error code otherwise
 */
int32_t TIDA010997_Sensor_initParamsDefault(TIDA010997_Sensor_InitParams_t *params,
                                            TIDA010997_Sensor_Type_t sensorType);

/**
 * \brief Initialize a sensor instance
 *
 * \param[out] instance  Pointer to sensor instance
 * \param[in]  params    Pointer to initialization parameters
 *
 * \return TIDA010997_SENSOR_OK on success, error code otherwise
 */
int32_t TIDA010997_Sensor_init(TIDA010997_Sensor_Instance_t *instance,
                               const TIDA010997_Sensor_InitParams_t *params);

/**
 * \brief De-initialize a sensor instance
 *
 * \param[in,out] instance  Pointer to sensor instance
 *
 * \return TIDA010997_SENSOR_OK on success, error code otherwise
 */
int32_t TIDA010997_Sensor_deInit(TIDA010997_Sensor_Instance_t *instance);

/**
 * \brief Trigger a measurement
 *
 * For on-demand mode sensors, triggers a new measurement.
 * For continuous mode sensors, this is a no-op.
 *
 * \param[in,out] instance  Pointer to sensor instance
 *
 * \return TIDA010997_SENSOR_OK on success, error code otherwise
 */
int32_t TIDA010997_Sensor_triggerMeasurement(TIDA010997_Sensor_Instance_t *instance);

/**
 * \brief Get a single data value from the sensor
 *
 * \param[in,out] instance  Pointer to sensor instance
 * \param[out]    data      Pointer to data structure
 * \param[in]     dataType  Type of data to retrieve
 *
 * \return TIDA010997_SENSOR_OK on success, error code otherwise
 */
int32_t TIDA010997_Sensor_getData(TIDA010997_Sensor_Instance_t *instance,
                                  TIDA010997_Sensor_Data_t *data,
                                  TIDA010997_Data_Type_t dataType);

/**
 * \brief Get all available data from the sensor
 *
 * \param[in,out] instance   Pointer to sensor instance
 * \param[out]    dataArray  Pointer to array of data structures
 * \param[in,out] dataCount  Input: max count, Output: actual count
 *
 * \return TIDA010997_SENSOR_OK on success, error code otherwise
 */
int32_t TIDA010997_Sensor_getAllData(TIDA010997_Sensor_Instance_t *instance,
                                     TIDA010997_Sensor_Data_t *dataArray,
                                     uint32_t *dataCount);

/**
 * \brief Get the sensor type
 *
 * \param[in]  instance  Pointer to sensor instance
 * \param[out] type      Pointer to store sensor type
 *
 * \return TIDA010997_SENSOR_OK on success, error code otherwise
 */
int32_t TIDA010997_Sensor_getType(const TIDA010997_Sensor_Instance_t *instance,
                                  TIDA010997_Sensor_Type_t *type);

/**
 * \brief Get the current state
 *
 * \param[in]  instance  Pointer to sensor instance
 * \param[out] state     Pointer to store current state
 *
 * \return TIDA010997_SENSOR_OK on success, error code otherwise
 */
int32_t TIDA010997_Sensor_getState(const TIDA010997_Sensor_Instance_t *instance,
                                   TIDA010997_Sensor_State_t *state);

#ifdef __cplusplus
}
#endif

#endif /* TIDA010997_H_ */

/** @} */
