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
 * \defgroup DAP_TYPES DAP Shared Types
 * \ingroup DAP_AM263X
 *
 * Shared type definitions for the DAP component.
 * This header contains all types shared between DAP layers to avoid
 * circular include dependencies.
 *
 * @{
 */

#ifndef DAP_TYPES_H
#define DAP_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <drivers/uart.h>
#include "dap_interface/Dap_Interface.h"

/* ========================================================================== */
/*                 Standard Type Definitions                                  */
/* ========================================================================== */

typedef uint8_t     uint8;
typedef uint16_t    uint16;
typedef uint32_t    uint32;
typedef int8_t      sint8;
typedef int16_t     sint16;
typedef int32_t     sint32;
typedef bool        boolean;

#ifndef TRUE
#define TRUE        (1U)
#endif

#ifndef FALSE
#define FALSE       (0U)
#endif

#ifndef NULL_PTR
#define NULL_PTR    ((void *)0)
#endif

#ifndef STD_ON
#define STD_ON      (1U)
#endif

#ifndef STD_OFF
#define STD_OFF     (0U)
#endif

/** \brief Error check configuration - set to STD_ON to enable runtime checks */
#ifndef DAP_CFG_ERROR_CHECK
#define DAP_CFG_ERROR_CHECK     STD_OFF
#endif

/** \brief TX wait for complete configuration - set to STD_ON to wait for TX completion */
#ifndef DAP_CFG_TX_WAIT_FOR_COMPLETE
#define DAP_CFG_TX_WAIT_FOR_COMPLETE    STD_ON
#endif

/** \brief TX ready timeout counter */
#ifndef DAP_CFG_TX_READY_TIMEOUT
#define DAP_CFG_TX_READY_TIMEOUT        (100000U)
#endif

/* ========================================================================== */
/*                 DAP Link Layer Definitions and Macros                      */
/* ========================================================================== */

/**
 * \anchor Dap_Link_BufferConfig
 * \name DAP Link buffer configuration
 * @{
 */

/** \brief Default buffer size in bytes */
#define DAP_LINK_BUFFER_SIZE_BYTES          (500U)

/** \brief DMA receive chunk size in bytes */
#define DAP_LINK_DMA_RX_SIZE_BYTES          (68U)

/** \brief Interrupt receive chunk - Initial size in bytes */
#define DAP_LINK_INT_INITIAL_RX_SIZE_BYTES  (3U)

/** \brief Buffer alignment for DMA operations */
#define DAP_LINK_BUFFER_ALIGN_BYTES         (4U)

/** \brief Link timeout error */
#define DAP_LINK_ERROR_TIMEOUT              (8U)

/** @} */

/**
 * \anchor Dap_Core_FrameOffsets
 * \name DAP Frame structure byte offsets
 * @{
 */

/** \brief Offset of start byte in frame */
#define DAP_FRAME_OFFSET_START              (0U)

/** \brief Offset of command/response byte in frame */
#define DAP_FRAME_OFFSET_CMD                (1U)

/** \brief Offset of payload length field in frame */
#define DAP_FRAME_OFFSET_LENGTH             (2U)

/** \brief Minimum frame size (start + cmd + len(1) + end) */
#define DAP_FRAME_MIN_SIZE_BYTES            (4U)

/** \brief Frame header size without payload (start + cmd) */
#define DAP_FRAME_HEADER_SIZE_BYTES         (2U)

/** \brief Frame trailer size */
#define DAP_FRAME_TRAILER_SIZE_BYTES        (1U)

/** \brief Minimum bytes needed to decode payload length (start + cmd + len) */
#define DAP_FRAME_MIN_HEADER_DECODE_BYTES   (3U)

/** \brief Fixed bytes in frame (header + trailer) */
#define DAP_FRAME_FIXED_SIZE_BYTES          (DAP_FRAME_HEADER_SIZE_BYTES + DAP_FRAME_TRAILER_SIZE_BYTES)

/** \brief Maximum size of length field */
#define DAP_FRAME_MAX_LENGTH_SIZE_BYTES     (3U)

/** @} */

/**
 * \anchor Dap_Link_ErrorType
 * \name DAP Link layer error codes
 * @{
 */

/** \brief No error */
#define DAP_LINK_ERROR_NONE                 (0U)

/** \brief Invalid instance pointer */
#define DAP_LINK_ERROR_INVALID_INSTANCE     (1U)

/** \brief Invalid parameters */
#define DAP_LINK_ERROR_INVALID_PARAMS       (2U)

/** \brief Hardware busy */
#define DAP_LINK_ERROR_HW_BUSY              (3U)

/** \brief Buffer overflow */
#define DAP_LINK_ERROR_BUFFER_OVERFLOW      (4U)

/** \brief Not initialized */
#define DAP_LINK_ERROR_NOT_INITIALIZED      (5U)

/** \brief Hardware initialization failed */
#define DAP_LINK_ERROR_HW_INIT              (6U)

/** \brief Unsupported operation mode */
#define DAP_LINK_ERROR_UNSUPPORTED_MODE     (7U)

/** @} */

/* ========================================================================== */
/*                 DAP Core Layer Definitions and Macros                      */
/* ========================================================================== */

/**
 * \anchor Dap_Core_FrameConstants
 * \name DAP Frame structure constants
 * @{
 */

/** \brief Frame start byte */
#define DAP_FRAME_START_BYTE            (0xEDU)

/** \brief Frame end byte (when CRC disabled) */
#define DAP_FRAME_END_BYTE              (0x9EU)

/** \brief CRC initial value */
#define DAP_CRC_INIT_VALUE              (0x66U)

/** @} */

/**
 * \anchor Dap_Core_PayloadLength
 * \name Payload length encoding constants
 * @{
 */

/** \brief 1-byte payload length minimum */
#define DAP_PAYLOAD_1B_MIN              (0U)

/** \brief 1-byte payload length maximum */
#define DAP_PAYLOAD_1B_MAX              (127U)

/** \brief 2-byte payload length minimum */
#define DAP_PAYLOAD_2B_MIN              (128U)

/** \brief 2-byte payload length maximum */
#define DAP_PAYLOAD_2B_MAX              (16383U)

/** \brief 3-byte payload length minimum */
#define DAP_PAYLOAD_3B_MIN              (16384U)

/** \brief 3-byte payload length maximum */
#define DAP_PAYLOAD_3B_MAX              (4194303U)

/** \brief 2-byte payload length offset */
#define DAP_PAYLOAD_2B_OFFSET           (0x8000U)

/** \brief 3-byte payload length offset */
#define DAP_PAYLOAD_3B_OFFSET           (0xC00000U)

/** @} */

/**
 * \anchor Dap_Core_CommandCodes
 * \name DAP Command codes
 * @{
 */

/** \brief Get capabilities command */
#define DAP_CMD_GET_CAPABILITIES        (0x01U)

/** \brief List sensors command */
#define DAP_CMD_LIST_SENSORS            (0x02U)

/** \brief Configure pipeline command */
#define DAP_CMD_CONFIGURE_PIPELINE      (0x03U)

/** \brief List models command */
#define DAP_CMD_LIST_MODELS             (0x04U)

/** \brief Remove model command (not supported) */
#define DAP_CMD_REMOVE_MODEL            (0x05U)

/** \brief Start model upload command (not supported) */
#define DAP_CMD_START_MODEL_UPLOAD      (0x06U)

/** \brief End model upload command (not supported) */
#define DAP_CMD_END_MODEL_UPLOAD        (0x07U)

/** \brief Start streaming command */
#define DAP_CMD_START_STREAMING         (0x08U)

/** \brief Stop streaming command */
#define DAP_CMD_STOP_STREAMING          (0x09U)

/** \brief List inferencing values command */
#define DAP_CMD_LIST_INF_VALUES         (0x0AU)

/** \brief Read property command */
#define DAP_CMD_READ_PROPERTY           (0x0CU)

/** \brief Write property command */
#define DAP_CMD_WRITE_PROPERTY          (0x0DU)

/** \brief List properties command */
#define DAP_CMD_LIST_PROPERTIES         (0x0EU)

/** \brief Send data command */
#define DAP_CMD_SEND_DATA               (0x10U)

/** @} */

/**
 * \anchor Dap_Core_ResponseCodes
 * \name DAP Response codes
 * @{
 */

/** \brief Error response */
#define DAP_RESP_ERROR                  (0x00U)

/** \brief Get capabilities response */
#define DAP_RESP_GET_CAPABILITIES       (0x01U)

/** \brief List sensors response */
#define DAP_RESP_LIST_SENSORS           (0x02U)

/** \brief Configure pipeline response */
#define DAP_RESP_CONFIGURE_PIPELINE     (0x03U)

/** \brief List models response */
#define DAP_RESP_LIST_MODELS            (0x04U)

/** \brief Remove model response */
#define DAP_RESP_REMOVE_MODEL           (0x05U)

/** \brief Start model upload response */
#define DAP_RESP_START_MODEL_UPLOAD     (0x06U)

/** \brief End model upload response */
#define DAP_RESP_END_MODEL_UPLOAD       (0x07U)

/** \brief Start streaming response */
#define DAP_RESP_START_STREAMING        (0x08U)

/** \brief Stop streaming response */
#define DAP_RESP_STOP_STREAMING         (0x09U)

/** \brief List inferencing values response */
#define DAP_RESP_LIST_INF_VALUES        (0x0AU)

/** \brief Read property response */
#define DAP_RESP_READ_PROPERTY          (0x0CU)

/** \brief Write property response */
#define DAP_RESP_WRITE_PROPERTY         (0x0DU)

/** \brief List properties response */
#define DAP_RESP_LIST_PROPERTIES        (0x0EU)

/** \brief Receive data response */
#define DAP_RESP_RECEIVE_DATA           (0x10U)

/** @} */

/**
 * \anchor Dap_Core_DataChannels
 * \name DAP Data channel identifiers
 * @{
 */

/** \brief Sensor signal data channel */
#define DAP_CHANNEL_SENSOR_SIGNAL       (0x01U)

/** \brief Inference signal data channel */
#define DAP_CHANNEL_INF_SIGNAL          (0x03U)

/** \brief Inference result data channel */
#define DAP_CHANNEL_INF_RESULT          (0x04U)

/** \brief Inference value data channel */
#define DAP_CHANNEL_INF_VALUE           (0x05U)

/** \brief Inference log data channel */
#define DAP_CHANNEL_INF_LOG             (0x06U)

/** @} */

/**
 * \anchor Dap_Core_ErrorCodes
 * \name DAP Core error codes
 * @{
 */

/** \brief No error */
#define DAP_CORE_ERROR_NONE             (0U)

/** \brief Invalid instance pointer */
#define DAP_CORE_ERROR_INVALID_INSTANCE (1U)

/** \brief Invalid parameters */
#define DAP_CORE_ERROR_INVALID_PARAMS   (2U)

/** \brief Invalid start byte */
#define DAP_CORE_ERROR_INVALID_START_BYTE (3U)

/** \brief Invalid end byte */
#define DAP_CORE_ERROR_INVALID_END_BYTE (4U)

/** \brief Invalid CRC */
#define DAP_CORE_ERROR_INVALID_CRC      (5U)

/** \brief Invalid command */
#define DAP_CORE_ERROR_INVALID_COMMAND  (6U)

/** \brief Invalid payload */
#define DAP_CORE_ERROR_INVALID_PAYLOAD  (7U)

/** \brief Buffer overflow */
#define DAP_CORE_ERROR_BUFFER_OVERFLOW  (8U)

/** \brief Generic frame error */
#define DAP_CORE_ERROR_MALFORMED_FRAME  (9U)

/** \brief Invalid DAP state error */
#define DAP_CORE_ERROR_INVALID_STATE    (10U)

/** \brief Link layer error */
#define DAP_CORE_ERROR_LINK             (11U)

/** @} */

/**
 * \anchor Dap_Core_ProtocolErrors
 * \name DAP Protocol error numbers (sent to host)
 * @{
 */

/** \brief Unsupported command error */
#define DAP_PROTOCOL_ERROR_UNSUPPORTED_CMD    (0x01U)

/** \brief Unknown command error */
#define DAP_PROTOCOL_ERROR_UNKNOWN_CMD        (0x02U)

/** \brief Invalid start byte */
#define DAP_PROTOCOL_ERROR_INVALID_START_BYTE (0x03U)

/** \brief Invalid end byte */
#define DAP_PROTOCOL_ERROR_INVALID_END_BYTE   (0x04U)

/** \brief Invalid payload error */
#define DAP_PROTOCOL_ERROR_INVALID_PAYLOAD    (0x05U)

/** \brief Invalid CRC error */
#define DAP_PROTOCOL_ERROR_INVALID_CRC        (0x06U)

/** @} */

/* ========================================================================== */
/*                 DAP Module Definitions and Macros                          */
/* ========================================================================== */

/**
 * \anchor Dap_ErrorType
 * \name DAP module error codes
 *
 * All APIs in this module return below error codes
 *
 * @{
 */

/** \brief No error */
#define DAP_ERROR_NONE                      (0)

/** \brief Invalid instance pointer */
#define DAP_ERROR_INVALID_INSTANCE          (1)

/** \brief Invalid parameters */
#define DAP_ERROR_INVALID_PARAMS            (2)

/** \brief Not initialized */
#define DAP_ERROR_NOT_INITIALIZED           (3)

/** \brief Frame start byte incorrect */
#define DAP_ERROR_INVALID_START_BYTE        (10)

/** \brief Frame end byte incorrect */
#define DAP_ERROR_INVALID_END_BYTE          (11)

/** \brief Frame CRC mismatch */
#define DAP_ERROR_INVALID_CRC               (12)

/** \brief Frame payload malformed */
#define DAP_ERROR_INVALID_PAYLOAD           (13)

/** \brief Buffer overflow */
#define DAP_ERROR_BUFFER_OVERFLOW           (14)

/** \brief Hardware busy */
#define DAP_ERROR_HW_BUSY                   (20)

/** \brief Transfer timeout */
#define DAP_ERROR_LINK_TIMEOUT              (21)

/** \brief Unsupported operation mode */
#define DAP_ERROR_UNSUPPORTED_MODE          (22)

/** \brief Streaming not active */
#define DAP_ERROR_NOT_STREAMING             (30)

/** \brief Stream not started - header not sent yet */
#define DAP_ERROR_STREAM_NOT_STARTED        (31)

/** \brief Invalid sensor index */
#define DAP_ERROR_INVALID_SENSOR_INDEX      (32)

/** @} */

/* ========================================================================== */
/*                 DAP Core Layer Typedefs                                    */
/* ========================================================================== */
/* ========================================================================== */
/*                 DAP Link Layer Typedefs                                    */
/* ========================================================================== */

/**
 * \brief DAP Link operation mode
 *
 * DMA mode is the primary operation mode. Interrupt mode support
 * is reserved for future implementation.
 */
typedef enum
{
    DAP_LINK_MODE_INTERRUPT,    /**< Interrupt mode operation */
    DAP_LINK_MODE_DMA,          /**< DMA mode operation */
    DAP_LINK_MODE_MAX           /**< Maximum enum value for bounds checking */
} Dap_Link_ModeType;

/**
 * \brief DAP Link Tx state
 */
typedef enum
{
    DAP_LINK_TX_STATE_IDLE,         /**< Idle, ready for transmission */
    DAP_LINK_TX_STATE_TRANSMITTING, /**< Transmitting data */
    DAP_LINK_TX_STATE_COMPLETE      /**< Transmission complete */
} Dap_Link_TxStateType;

/**
 * \brief DAP Link Rx state
 */
typedef enum
{
    DAP_LINK_RX_STATE_IDLE,         /**< Idle, waiting for data */
    DAP_LINK_RX_STATE_BUFFERING,    /**< Waiting to receive a complete frame */
    DAP_LINK_RX_STATE_FRAME_READY   /**< Frame ready for processing */
} Dap_Link_RxStateType;

/**
 * \brief DAP Frame information structure
 */
typedef struct
{
    /** \brief Data buffer */
    uint8                   Buffer[DAP_LINK_BUFFER_SIZE_BYTES] __attribute__((aligned(DAP_LINK_BUFFER_ALIGN_BYTES)));
    /** \brief Current position in buffer */
    uint32                  Ptr;
    /** \brief Total frame length */
    uint32                  Len;
} Dap_Link_Frame;

/**
 * \brief DAP Link initialization parameters
 */
typedef struct
{
    /** \brief UART instance index from SysConfig (e.g., CONFIG_UART0) */
    uint32                      UartInstanceIndex;
    /** \brief Link Tx operation mode */
    Dap_Link_ModeType           LinkTxMode;
    /** \brief Link Rx operation mode */
    Dap_Link_ModeType           LinkRxMode;
} Dap_Link_InitParamsType;

/* Forward declaration for cross-reference */
struct Dap_Link_InstanceTag;

/**
 * \brief DAP Link Tx instance structure
 */
typedef struct
{
    /** \brief Pointer to parent link instance */
    struct Dap_Link_InstanceTag *ParentInstancePtr;
    /** \brief Operation mode */
    Dap_Link_ModeType           Mode;
    /** \brief Transmit frame */
    Dap_Link_Frame              Frame;
    /** \brief Transmit state */
    volatile Dap_Link_TxStateType State;
} Dap_Link_TxInstanceType;

/**
 * \brief DAP Link Rx instance structure
 */
typedef struct
{
    /** \brief Pointer to parent link instance */
    struct Dap_Link_InstanceTag *ParentInstancePtr;
    /** \brief Operation mode */
    Dap_Link_ModeType           Mode;
    /** \brief Receive frame */
    Dap_Link_Frame              Frame;
    /** \brief Receive state */
    volatile Dap_Link_RxStateType State;
    /** \brief Expected frame length (0 if unknown) - legacy field */
    uint32                      FrameLength;
    /** \brief Expected total frame length after header decoded */
    uint32                      ExpectedFrameLen;
    /** \brief TRUE when payload length has been decoded from header */
    boolean                     HeaderDecoded;
} Dap_Link_RxInstanceType;


/**
 * \brief DAP Link instance structure (contains both Tx and Rx)
 */
typedef struct Dap_Link_InstanceTag
{
    /** \brief UART handle from MCU+SDK driver */
    UART_Handle             UartHandle;
    /** \brief UART transaction for Tx operations */
    UART_Transaction        TxTransaction;
    /** \brief UART transaction for Rx operations */
    UART_Transaction        RxTransaction;
    /** \brief Transmit instance */
    Dap_Link_TxInstanceType TxInstance;
    /** \brief Receive instance */
    Dap_Link_RxInstanceType RxInstance;
    /** \brief Initialization flag */
    boolean                 IsInitialized;
    /** \brief Last frame error encountered (stores DAP_CORE_ERROR_* codes) */
    uint32                  LastFrameError;
    /** \brief Last protocol error code sent to host */
    volatile uint32         LastProtocolError;
    /** \brief CRC enabled flag */
    boolean                 IsCrcEnabled;
} Dap_Link_InstanceType;

/* ========================================================================== */
/*                 DAP Module Typedefs                                        */
/* ========================================================================== */

/**
 * \brief Pipeline mode enumeration
 *
 * Defines the operating mode for the data pipeline.
 */
typedef enum
{
    DAP_PIPELINE_MODE_UNINITIALIZED    = 0x00U,  /**< Pipeline not yet initialized  */
    DAP_PIPELINE_MODE_DATA_ACQUISITION = 0x01U,  /**< Data acquisition mode */
    DAP_PIPELINE_MODE_SENSOR_INFERENCE = 0x02U,  /**< Sensor inference mode */
    DAP_PIPELINE_MODE_HOST_INFERENCE   = 0x03U,  /**< Host inference mode */
    DAP_PIPELINE_MODE_LOOPBACK         = 0x04U   /**< Loopback mode */
} Dap_PipelineModeType;

/**
 * \brief Pipeline configuration structure
 *
 * Contains the current pipeline configuration state.
 */
typedef struct
{
    /** \brief Pipeline operating mode */
    Dap_PipelineModeType Mode;
    /** \brief Selected model index (0 if no model) */
    uint8                ModelIndex;
    /** \brief Array of configured sensor indices */
    uint8                SensorIndex[DAP_INTERFACE_MAX_SENSORS];
    /** \brief Number of configured sensors */
    uint8                SensorCount;
} Dap_PipelineConfigType;

/**
 * \brief DAP interface configuration structure
 *
 * Contains all application-provided sensor, model, property, and inference value data.
 */
typedef struct
{
    /** \brief Array of sensor JSON strings (pre-formatted) */
    const char                               *SensorList[DAP_INTERFACE_MAX_SENSORS];
    /** \brief Number of sensors */
    uint8                                     SensorCount;

    /** \brief Array of model JSON strings (pre-formatted) */
    const char                               *ModelList[DAP_INTERFACE_MAX_MODELS];
    /** \brief Number of models */
    uint8                                     ModelCount;

    /** \brief Array of property information pointers */
    Dap_Interface_PropertyInfoType           *PropertyList[DAP_INTERFACE_MAX_PROPERTIES];
    /** \brief Number of properties */
    uint8                                     PropertyCount;

    /** \brief Array of inference value information pointers */
    const Dap_Interface_InfValueInfoType     *InfValueList[DAP_INTERFACE_MAX_INF_VALUES];
    /** \brief Number of inference values */
    uint8                                     InfValueCount;
} Dap_InterfaceConfigType;

/**
 * \brief Streaming context structure
 *
 * Maintains state for multi-sensor continuous streaming mode.
 */
typedef struct
{
    /** \brief Per-sensor packet sequence number **/
    uint32           SequenceNumber[DAP_INTERFACE_MAX_SENSORS];
    /** \brief Total samples to stream */
    uint32           TotalSampleCount;
    /** \brief Samples streamed so far */
    uint32           CurrentSampleCount;
    /** \brief TRUE after stream header has been transmitted */
    boolean          HeaderSent;
    /** \brief TRUE when streaming is active */
    volatile boolean IsActive;
} Dap_StreamingContextType;

/**
 * \brief DAP initialization parameters
 */
typedef struct
{
    /** \brief Link hardware configuration */
    Dap_Link_InitParamsType         LinkParams;
    /** \brief Interface configuration (sensors, models, properties, inf values) */
    const Dap_InterfaceConfigType  *InterfaceConfigPtr;
} Dap_InitParamsType;

/**
 * \brief DAP instance structure
 */
typedef struct Dap_InstanceTag
{
    /** \brief Link layer instance */
    Dap_Link_InstanceType               LinkInstance;
    /** \brief Pipeline configuration */
    Dap_PipelineConfigType              PipelineConfig;
    /** \brief Interface configuration (sensors, models, properties, inf values) */
    Dap_InterfaceConfigType             InterfaceConfig;
    /** \brief Streaming context for multi-sensor continuous streaming */
    Dap_StreamingContextType            StreamingContext;
    /** \brief Initialization flag */
    boolean                             IsInitialized;
} Dap_InstanceType;

#ifdef __cplusplus
}
#endif

#endif /* DAP_TYPES_H */

/** @} */