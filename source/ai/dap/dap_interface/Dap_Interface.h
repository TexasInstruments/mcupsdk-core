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
 * \defgroup DAP_INTERFACE DAP Interface Layer
 * \ingroup DAP_AM263X
 *
 * @{
 */

#ifndef DAP_INTERFACE_H
#define DAP_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/*                 Standard Type Definitions                                  */
/* ========================================================================== */

typedef uint8_t     uint8;
typedef uint16_t    uint16;
typedef uint32_t    uint32;
typedef int8_t      sint8;
typedef int16_t     sint16;
typedef int32_t     sint32;

#ifndef TRUE
#define TRUE        (1U)
#endif

#ifndef FALSE
#define FALSE       (0U)
#endif

/* ========================================================================== */
/*                 Public Definitions and Macros                              */
/* ========================================================================== */

/** \brief Maximum number of sensors supported */
#define DAP_INTERFACE_MAX_SENSORS           (8U)

/** \brief Maximum number of properties supported */
#define DAP_INTERFACE_MAX_PROPERTIES        (16U)

/** \brief Maximum number of models supported */
#define DAP_INTERFACE_MAX_MODELS            (4U)

/** \brief Maximum number of inference values supported */
#define DAP_INTERFACE_MAX_INF_VALUES        (8U)

/** \brief Maximum length of sensor/property name */
#define DAP_INTERFACE_MAX_NAME_LEN_BYTES    (32U)

/**
 * \anchor Dap_Interface_DeviceConfig
 * \name DAP Device Configuration Macros
 *
 * Compile-time device configuration. These macros define device capabilities
 * that are reported to the host during the Get Capabilities command.
 *
 * @{
 */

/** \brief API version number */
#define DAP_INTERFACE_API_VERSION           (1U)

/** \brief SDK major version number */
#define DAP_INTERFACE_SDK_MAJOR_VERSION     (1U)

/** \brief SDK minor version number */
#define DAP_INTERFACE_SDK_MINOR_VERSION     (1U)

/** \brief Device name string */
#define DAP_INTERFACE_DEVICE_NAME           "AM263x"

/** \brief CRC capability enabled (1U/0U) */
#define DAP_INTERFACE_CRC_ENABLED           (0U)

/** \brief Bidirectional communication enabled (1U/0U) */
#define DAP_INTERFACE_BIDIRECTIONAL_ENABLED (0U)

/** \brief Big endian data format enabled (1U/0U) */
#define DAP_INTERFACE_BIG_ENDIAN_ENABLED    (1U)

/** \brief Model upload capability enabled (1U/0U) */
#define DAP_INTERFACE_UPLOAD_ENABLED        (0U)

/** \brief On-device inference capability enabled (1U/0U) */
#define DAP_INTERFACE_INFERENCE_ENABLED     (0U)

/** \brief Attach sequencing metadata when streaming (1U/0U) */
#define DAP_INTERFACE_USE_SEQUENCE_HEADERS  (1U)

/** @} */

/* ========================================================================== */
/*                 Public Typedefs                                            */
/* ========================================================================== */

/**
 * \brief Sensor type enumeration
 *
 * Defines the type of sensor for data interpretation.
 */
typedef enum
{
    DAP_SENSOR_TYPE_IMAGE     = 0x01U,  /**< Image sensor */
    DAP_SENSOR_TYPE_AUDIO     = 0x02U,  /**< Audio sensor */
    DAP_SENSOR_TYPE_SCALAR    = 0x03U,  /**< Scalar sensor (single value) */
    DAP_SENSOR_TYPE_VECTOR    = 0x04U,  /**< Vector sensor (multiple values) */
    DAP_SENSOR_TYPE_ARRAY     = 0x05U,  /**< Array sensor */
    DAP_SENSOR_TYPE_ARC_FAULT = 0x06U   /**< Arc fault sensor */
} Dap_Interface_SensorTypeType;

/**
 * \brief Data format enumeration
 *
 * Defines the data format for sensor data and properties.
 */
typedef enum
{
    DAP_DATA_FORMAT_INT8    = 0x01U,    /**< 8-bit signed integer */
    DAP_DATA_FORMAT_INT16   = 0x02U,    /**< 16-bit signed integer */
    DAP_DATA_FORMAT_INT32   = 0x03U,    /**< 32-bit signed integer */
    DAP_DATA_FORMAT_UINT8   = 0x04U,    /**< 8-bit unsigned integer */
    DAP_DATA_FORMAT_UINT16  = 0x05U,    /**< 16-bit unsigned integer */
    DAP_DATA_FORMAT_UINT32  = 0x06U,    /**< 32-bit unsigned integer */
    DAP_DATA_FORMAT_FLOAT16 = 0x07U,    /**< 16-bit float */
    DAP_DATA_FORMAT_FLOAT32 = 0x08U,    /**< 32-bit float */
    DAP_DATA_FORMAT_FLOAT64 = 0x09U,    /**< 64-bit float */
    DAP_DATA_FORMAT_JPEG    = 0x0AU     /**< JPEG image */
} Dap_Interface_DataFormatType;


/**
 * \brief Property value union
 *
 * Holds the value of a property in the appropriate type.
 */
typedef union
{
    uint8  U8;      /**< 8-bit unsigned value */
    uint16 U16;     /**< 16-bit unsigned value */
    uint32 U32;     /**< 32-bit unsigned value */
    sint8  S8;      /**< 8-bit signed value */
    sint16 S16;     /**< 16-bit signed value */
    sint32 S32;     /**< 32-bit signed value */
} Dap_Interface_PropertyValueType;

/**
 * \brief Property information structure
 *
 * Contains metadata and value for a device property.
 */
typedef struct
{
    /** \brief Property name (null-terminated string) */
    const char                      *NamePtr;
    /** \brief Data type of the property */
    Dap_Interface_DataFormatType     Type;
    /** \brief Current value of the property */
    Dap_Interface_PropertyValueType  Value;
} Dap_Interface_PropertyInfoType;

/**
 * \brief Inference value information structure
 *
 * Contains metadata about an inference value for reporting to the host.
 */
typedef struct
{
    /** \brief Inference value name (null-terminated string) */
    const char                      *NamePtr;
    /** \brief Data format for this inference value */
    Dap_Interface_DataFormatType     Format;
} Dap_Interface_InfValueInfoType;

/* ========================================================================== */
/*                 Public Functions                                           */
/* ========================================================================== */


#ifdef __cplusplus
}
#endif

#endif /* DAP_INTERFACE_H */

/** @} */