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
 * \defgroup BOARD_HDC3020 HDC3020 Temperature/Humidity Sensor Driver
 * \ingroup BOARD_TIDA010997_MODULE
 *
 * HDC3020 is a high-accuracy digital temperature and humidity sensor.
 * This driver interfaces via I2C to configure and read measurements.
 *
 * ## Features
 * - Temperature measurement: -40 to +125 degrees C
 * - Humidity measurement: 0 to 100% RH
 * - Multiple low power modes for power optimization
 * - Auto measurement mode with configurable rates (0.5Hz to 10Hz)
 * - Trigger-on-demand mode for single measurements
 * - CRC-8 validation for data integrity
 *
 * @{
 */

/**
 * \file hdc3020.h
 * \brief HDC3020 Temperature/Humidity Sensor Driver for TIDA-010997
 */

#ifndef HDC3020_H_
#define HDC3020_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>
#include <drivers/i2c.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief HDC3020 I2C address (fixed for TIDA-010997 booster pack) */
#define HDC3020_I2C_ADDR            (0x46U)

/** \brief HDC3020 command codes */
#define HDC3020_CMD_SOFT_RESET      (0x30A2U)
#define HDC3020_CMD_READ_BOTH       (0xE000U)
#define HDC3020_CMD_READ_RH_ONLY    (0xE001U)
#define HDC3020_CMD_EXIT_AUTO_MODE  (0x3093U)

/**
 * \anchor HDC3020_Error
 * \name HDC3020 Error Codes
 * @{
 */
#define HDC3020_OK                  (0)   /**< Operation successful */
#define HDC3020_ERR_I2C             (-1)  /**< I2C communication error */
#define HDC3020_ERR_CRC             (-2)  /**< CRC verification failed */
#define HDC3020_ERR_INVALID_DATA    (-3)  /**< Invalid measurement data */
#define HDC3020_ERR_NOT_INITIALIZED (-4)  /**< Driver not initialized */
/** @} */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 * \brief HDC3020 low power modes
 *
 * Lower power modes reduce accuracy but decrease conversion time.
 */
typedef enum
{
    HDC3020_LPM_0 = 0,  /**< Low Power Mode 0: Lowest noise, 12.5ms conversion */
    HDC3020_LPM_1 = 1,  /**< Low Power Mode 1: Medium, 7.5ms conversion */
    HDC3020_LPM_2 = 2,  /**< Low Power Mode 2: Medium, 5.0ms conversion */
    HDC3020_LPM_3 = 3   /**< Low Power Mode 3: Lowest power, 3.7ms conversion */
} HDC3020_Lpm_t;

/**
 * \brief HDC3020 measurement rates for auto measurement mode
 */
typedef enum
{
    HDC3020_RATE_0_5HZ = 0,  /**< 0.5 measurements per second */
    HDC3020_RATE_1HZ   = 1,  /**< 1 measurement per second */
    HDC3020_RATE_2HZ   = 2,  /**< 2 measurements per second */
    HDC3020_RATE_4HZ   = 3,  /**< 4 measurements per second */
    HDC3020_RATE_10HZ  = 4   /**< 10 measurements per second */
} HDC3020_Rate_t;

/**
 * \brief HDC3020 measurement mode
 */
typedef enum
{
    HDC3020_MODE_TRIGGER_ON_DEMAND = 0,  /**< Single measurement on command */
    HDC3020_MODE_AUTO_MEASUREMENT  = 1   /**< Continuous measurements at specified rate */
} HDC3020_Mode_t;

/**
 * \brief HDC3020 sensor type selection
 */
typedef enum
{
    HDC3020_SENSOR_BOTH = 0,     /**< Both temperature and humidity sensors */
    HDC3020_SENSOR_TEMPERATURE,  /**< Temperature sensor only */
    HDC3020_SENSOR_HUMIDITY      /**< Humidity sensor only */
} HDC3020_Sensor_t;

/**
 * \brief HDC3020 measurement data structure
 */
typedef struct
{
    uint16_t temperature;  /**< Raw temperature value (16-bit) */
    uint16_t humidity;     /**< Raw humidity value (16-bit) */
    bool     valid;        /**< Flag indicating if the data is valid */
} HDC3020_Data_t;

/**
 * \brief HDC3020 configuration structure
 */
typedef struct
{
    HDC3020_Mode_t mode;  /**< Measurement mode */
    HDC3020_Lpm_t  lpm;   /**< Low power mode */
    HDC3020_Rate_t rate;  /**< Measurement rate (for auto mode) */
} HDC3020_Config_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * \brief Initialize the HDC3020 sensor
 *
 * This function initializes the HDC3020 sensor with the specified configuration.
 * It performs sensor detection, soft reset, and mode configuration.
 *
 * \param[in] i2cHandle  MCU+SDK I2C handle (must be opened before calling)
 * \param[in] config     Pointer to configuration structure
 *
 * \return HDC3020_OK on success, error code otherwise
 */
int32_t HDC3020_init(I2C_Handle i2cHandle, const HDC3020_Config_t *config);

/**
 * \brief Reset the HDC3020 sensor
 *
 * Performs a soft reset of the sensor. After reset, the sensor must be
 * reinitialized using HDC3020_init().
 *
 * \return HDC3020_OK on success, error code otherwise
 */
int32_t HDC3020_reset(void);

/**
 * \brief Get data from the selected sensor(s)
 *
 * In trigger-on-demand mode, this function triggers a measurement and waits
 * for conversion to complete. In auto measurement mode, it reads the latest
 * available measurement.
 *
 * \param[out] data        Pointer to data structure to store the readings
 * \param[in]  sensorType  Which sensor to read (temperature, humidity, or both)
 *
 * \return HDC3020_OK on success, error code otherwise
 */
int32_t HDC3020_getData(HDC3020_Data_t *data, HDC3020_Sensor_t sensorType);

/**
 * \brief Trigger a single measurement (only in trigger-on-demand mode)
 *
 * If currently in auto measurement mode, this function will exit auto mode
 * first before triggering a single measurement.
 *
 * \param[in] lpm  Low power mode to use for the measurement
 *
 * \return HDC3020_OK on success, error code otherwise
 */
int32_t HDC3020_triggerMeasurement(HDC3020_Lpm_t lpm);

/**
 * \brief Set the measurement mode
 *
 * \param[in] mode  Measurement mode
 * \param[in] lpm   Low power mode
 * \param[in] rate  Measurement rate (for auto mode)
 *
 * \return HDC3020_OK on success, error code otherwise
 */
int32_t HDC3020_setMode(HDC3020_Mode_t mode, HDC3020_Lpm_t lpm, HDC3020_Rate_t rate);

/**
 * \brief Convert raw temperature value to Celsius
 *
 * Uses the formula: T = -45 + 175 * (raw / 65535)
 *
 * \param[in] raw  Raw 16-bit temperature value from sensor
 *
 * \return Temperature in degrees Celsius
 */
float HDC3020_convertTemperature(uint16_t raw);

/**
 * \brief Convert raw humidity value to relative humidity percentage
 *
 * Uses the formula: RH = 100 * (raw / 65535)
 *
 * \param[in] raw  Raw 16-bit humidity value from sensor
 *
 * \return Relative humidity in percent (0-100)
 */
float HDC3020_convertHumidity(uint16_t raw);

/**
 * \brief Calculate CRC-8 checksum for HDC3020 communication
 *
 * Uses polynomial x^8 + x^5 + x^4 + 1 (0x31) with initial value 0xFF.
 *
 * \param[in] data    Pointer to data bytes
 * \param[in] length  Number of bytes
 *
 * \return Calculated CRC value
 */
uint8_t HDC3020_calculateCrc(const uint8_t *data, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif /* HDC3020_H_ */

/** @} */
