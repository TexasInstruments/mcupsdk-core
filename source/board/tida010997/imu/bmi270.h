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
 * \defgroup BOARD_BMI270 BMI270 6-Axis IMU Driver
 * \ingroup BOARD_TIDA010997_MODULE
 *
 * BMI270 is a 6-axis Inertial Measurement Unit (IMU) with integrated
 * 3-axis accelerometer and 3-axis gyroscope. The driver interfaces via SPI.
 *
 * ## Features
 * - 3-axis accelerometer (±2g to ±16g range)
 * - 3-axis gyroscope (±125°/s to ±2000°/s range)
 * - Configurable output data rates (25Hz to 1600Hz)
 * - Multiple power modes for power optimization
 * - 8KB firmware upload during initialization
 *
 * @{
 */

/**
 * \file bmi270.h
 * \brief BMI270 6-Axis IMU Driver for TIDA-010997
 */

#ifndef BMI270_H_
#define BMI270_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>
#include <drivers/mcspi.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief BMI270 expected chip ID value */
#define BMI270_CHIP_ID              (0x24U)

/** \brief BMI270 firmware size in bytes */
#define BMI270_FIRMWARE_SIZE        (8192U)

/**
 * \anchor BMI270_Error
 * \name BMI270 Error Codes
 * @{
 */
#define BMI270_OK                   (0)   /**< Operation successful */
#define BMI270_ERR_SPI              (-1)  /**< SPI communication error */
#define BMI270_ERR_INVALID_CHIP_ID  (-2)  /**< Chip ID mismatch */
#define BMI270_ERR_INIT_FAILED      (-3)  /**< Initialization failed */
#define BMI270_ERR_TIMEOUT          (-4)  /**< Operation timeout */
#define BMI270_ERR_INVALID_DATA     (-5)  /**< Invalid data received */
#define BMI270_ERR_SENSOR_DISABLED  (-6)  /**< Sensor not enabled */
#define BMI270_ERR_NOT_INITIALIZED  (-7)  /**< Driver not initialized */
/** @} */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 * \brief BMI270 power modes
 */
typedef enum
{
    BMI270_POWER_MODE_SUSPEND = 0U,  /**< All sensors suspended, lowest power */
    BMI270_POWER_MODE_NORMAL,        /**< Normal operation mode */
    BMI270_POWER_MODE_LOW_POWER,     /**< Low power mode */
    BMI270_POWER_MODE_PERFORMANCE    /**< High performance mode */
} BMI270_PowerMode_t;

/**
 * \brief BMI270 sensor enable configuration
 */
typedef enum
{
    BMI270_SENSOR_NONE = 0U,   /**< No sensors enabled */
    BMI270_SENSOR_ACCEL_ONLY,  /**< Accelerometer only */
    BMI270_SENSOR_GYRO_ONLY,   /**< Gyroscope only */
    BMI270_SENSOR_BOTH         /**< Both sensors enabled */
} BMI270_SensorEnable_t;

/**
 * \brief BMI270 accelerometer measurement range
 */
typedef enum
{
    BMI270_ACCEL_RANGE_2G  = 0x00U,  /**< ±2g range */
    BMI270_ACCEL_RANGE_4G  = 0x01U,  /**< ±4g range */
    BMI270_ACCEL_RANGE_8G  = 0x02U,  /**< ±8g range */
    BMI270_ACCEL_RANGE_16G = 0x03U   /**< ±16g range */
} BMI270_AccelRange_t;

/**
 * \brief BMI270 gyroscope measurement range
 */
typedef enum
{
    BMI270_GYRO_RANGE_2000DPS = 0x00U,  /**< ±2000°/s range */
    BMI270_GYRO_RANGE_1000DPS = 0x01U,  /**< ±1000°/s range */
    BMI270_GYRO_RANGE_500DPS  = 0x02U,  /**< ±500°/s range */
    BMI270_GYRO_RANGE_250DPS  = 0x03U,  /**< ±250°/s range */
    BMI270_GYRO_RANGE_125DPS  = 0x04U   /**< ±125°/s range */
} BMI270_GyroRange_t;

/**
 * \brief BMI270 output data rate (ODR) configuration
 */
typedef enum
{
    BMI270_ODR_25HZ   = 0x06U,  /**< 25 Hz output rate */
    BMI270_ODR_50HZ   = 0x07U,  /**< 50 Hz output rate */
    BMI270_ODR_100HZ  = 0x08U,  /**< 100 Hz output rate */
    BMI270_ODR_200HZ  = 0x09U,  /**< 200 Hz output rate */
    BMI270_ODR_400HZ  = 0x0AU,  /**< 400 Hz output rate */
    BMI270_ODR_800HZ  = 0x0BU,  /**< 800 Hz output rate */
    BMI270_ODR_1600HZ = 0x0CU   /**< 1600 Hz output rate */
} BMI270_Odr_t;

/**
 * \brief BMI270 bandwidth parameter (filter configuration)
 */
typedef enum
{
    BMI270_BWP_OSR4   = 0x00U,  /**< OSR4 averaging mode */
    BMI270_BWP_OSR2   = 0x01U,  /**< OSR2 averaging mode */
    BMI270_BWP_NORMAL = 0x02U   /**< Normal filter mode */
} BMI270_Bwp_t;

/**
 * \brief BMI270 configuration structure
 */
typedef struct
{
    BMI270_PowerMode_t    powerMode;       /**< Power mode selection */
    BMI270_SensorEnable_t sensorEnable;    /**< Sensor enable configuration */
    BMI270_AccelRange_t   accelRange;      /**< Accelerometer range */
    BMI270_GyroRange_t    gyroRange;       /**< Gyroscope range */
    BMI270_Odr_t          accelOdr;        /**< Accelerometer ODR */
    BMI270_Odr_t          gyroOdr;         /**< Gyroscope ODR */
    BMI270_Bwp_t          accelBwp;        /**< Accelerometer bandwidth */
    BMI270_Bwp_t          gyroBwp;         /**< Gyroscope bandwidth */
    bool                             accelFilterPerf; /**< Accel filter performance mode */
    bool                             gyroFilterPerf;  /**< Gyro filter performance mode */
    bool                             gyroNoisePerf;   /**< Gyro noise performance mode */
} BMI270_Config_t;

/**
 * \brief BMI270 accelerometer data structure
 */
typedef struct
{
    int16_t x;      /**< X-axis acceleration (raw) */
    int16_t y;      /**< Y-axis acceleration (raw) */
    int16_t z;      /**< Z-axis acceleration (raw) */
    bool    valid;  /**< Data validity flag */
} BMI270_AccelData_t;

/**
 * \brief BMI270 gyroscope data structure
 */
typedef struct
{
    int16_t x;      /**< X-axis angular velocity (raw) */
    int16_t y;      /**< Y-axis angular velocity (raw) */
    int16_t z;      /**< Z-axis angular velocity (raw) */
    bool    valid;  /**< Data validity flag */
} BMI270_GyroData_t;

/**
 * \brief BMI270 combined IMU data structure
 */
typedef struct
{
    BMI270_AccelData_t accel;  /**< Accelerometer data */
    BMI270_GyroData_t  gyro;   /**< Gyroscope data */
} BMI270_ImuData_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * \brief Initialize the BMI270 sensor
 *
 * Performs complete initialization sequence including:
 * - SPI mode switching
 * - Soft reset
 * - Chip ID verification
 * - Firmware upload (8KB)
 * - Sensor configuration
 *
 * \param[in] spiHandle   MCU+SDK MCSPI handle (must be opened before calling)
 * \param[in] config      Pointer to configuration structure
 *
 * \return BMI270_OK on success, error code otherwise
 */
int32_t BMI270_init(MCSPI_Handle spiHandle, const BMI270_Config_t *config);

/**
 * \brief Reset the BMI270 sensor
 *
 * Performs soft reset of the device. After reset, the device must be
 * reinitialized using BMI270_init().
 *
 * \return BMI270_OK on success, error code otherwise
 */
int32_t BMI270_reset(void);

/**
 * \brief Read accelerometer data
 *
 * Reads 3-axis accelerometer data from the device.
 * Sensor must be enabled (ACCEL_ONLY or BOTH) before reading.
 *
 * \param[out] data  Pointer to accelerometer data structure
 *
 * \return BMI270_OK on success, error code otherwise
 */
int32_t BMI270_getAccelData(BMI270_AccelData_t *data);

/**
 * \brief Read gyroscope data
 *
 * Reads 3-axis gyroscope data from the device.
 * Sensor must be enabled (GYRO_ONLY or BOTH) before reading.
 *
 * \param[out] data  Pointer to gyroscope data structure
 *
 * \return BMI270_OK on success, error code otherwise
 */
int32_t BMI270_getGyroData(BMI270_GyroData_t *data);

/**
 * \brief Read combined IMU data
 *
 * Reads both accelerometer and gyroscope data in a single operation.
 * More efficient than reading sensors individually.
 *
 * \param[out] data  Pointer to IMU data structure
 *
 * \return BMI270_OK on success, error code otherwise
 */
int32_t BMI270_getImuData(BMI270_ImuData_t *data);

/**
 * \brief Set power mode
 *
 * Changes the device power mode. Must be called after initialization.
 *
 * \param[in] mode  Power mode to set
 *
 * \return BMI270_OK on success, error code otherwise
 */
int32_t BMI270_setPowerMode(BMI270_PowerMode_t mode);

/**
 * \brief Set sensor enable configuration
 *
 * Enables or disables accelerometer and/or gyroscope.
 *
 * \param[in] sensorEnable  Sensor enable configuration
 *
 * \return BMI270_OK on success, error code otherwise
 */
int32_t BMI270_setSensorEnable(BMI270_SensorEnable_t sensorEnable);

/**
 * \brief Read chip ID
 *
 * Reads the device chip ID for verification. Expected value is 0x24.
 *
 * \param[out] chipId  Pointer to store chip ID
 *
 * \return BMI270_OK on success, error code otherwise
 */
int32_t BMI270_getChipId(uint8_t *chipId);

/**
 * \brief Convert raw accelerometer value to m/s^2
 *
 * \param[in] raw    Raw 16-bit accelerometer value
 * \param[in] range  Accelerometer range setting
 *
 * \return Acceleration in m/s^2
 */
float BMI270_convertAccel(int16_t raw, BMI270_AccelRange_t range);

/**
 * \brief Convert raw gyroscope value to degrees/s
 *
 * \param[in] raw    Raw 16-bit gyroscope value
 * \param[in] range  Gyroscope range setting
 *
 * \return Angular velocity in degrees/s
 */
float BMI270_convertGyro(int16_t raw, BMI270_GyroRange_t range);

#ifdef __cplusplus
}
#endif

#endif /* BMI270_H_ */

/** @} */
