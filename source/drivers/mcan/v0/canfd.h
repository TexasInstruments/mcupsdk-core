/*
 * Copyright (C) 2024 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  \defgroup DRV_CANFD_MODULE APIs for CANFD
 *  \ingroup  DRV_MODULE
 *  @{
 *  \file  canfd.h
 * 
 *  \brief
 *      This is the header file for the CANFD driver which exposes the
 *      data structures and exported API which can be used by the
 *      applications to use the CANFD driver.
 *
 *  The CANFD driver provides functionality of transferring data between CANFD peripherals.
 *  This driver does not interpret any of the data sent to or received from using this peripheral.
 *
 *
 *  The CANFD header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/canfd/canfd.h>
 *   @endcode
 *
 *  ## Initializing the driver #
 *  The CANFD Driver needs to be initialized once across the System. This is
 *  done using the #CANFD_open. None of the CANFD API's can be used without invoking
 *  this API.
 *
 *  Once the CANFD Driver has been initialized; the bit timing can be configured using #CANFD_configBitTime.
 *  This APIs can be called multiple times to reconfigure bit timings.
 *
 * ## Creating the message objects #
 *  Message objects are used to transmit or receive data over the CANFD peripheral. A message object is created
 *  using #CANFD_createMsgObject.
 *
 * ## Sending and receiving data #
 *  Data is transmitted using the #CANFD_write. The application will be notified when the transmit is
 *  complete if it has enabled dataInterruptEnable and registered a callback function appDataCallBack when
 *  initializing the CANFD driver.
 *
 *  If the receive interrupts are enabled using dataInterruptEnable field and a callback function appCallBack
 *  has been registered when initializing the CANFD driver, the driver notifies the application
 *  when the data has arrived. The application needs to call the #CANFD_read function to read the received data.

 *
 * ## Error and status handling #
 *  The application can monitor the ECC error, Bus off error and Protocol Errors by enabling the error interrupts errInterruptEnable.
 *  The driver will call the registered callback function appErrCallBack to indicate which error fields caused the interrupt.
 *  It is up to the application to take appropriate action.
 *
 * ## Get/Set Options #
 *  Helper APIs to get and set various statistics, error counters, ECC diagnostics, power down have been provided.
 *  Refer to \ref CANFD_Option for more information.
 *
 * ## Limitation #
 *  The CANFD driver does not support the DMA or power down.
 *
 * @{
 */

#ifndef CANFD__H_
#define CANFD__H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <drivers/mcan/v0/mcan.h>
#include <drivers/hw_include/cslr_soc.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/mcan/v0/dma/canfd_dma.h>

/** \brief A handle that is returned from a #CANFD_open() call */
typedef struct CANFD_Config_s *CANFD_Handle;

/** \brief Externally defined driver configuration array size */
extern uint32_t             gCANFDConfigNum;

/** @name Return status
 *
 * @{
 *
 * \brief Return status when the API execution was successful
 */
#define MCAN_STATUS_SUCCESS         ((int32_t)0)

/**
 * \brief Return status when the API execution was not successful due to a failure
 */
#define MCAN_STATUS_FAILURE         ((int32_t)-1)
/** @} */

/**
 *  \anchor CANFD_TransferMode
 *  \name Transfer Mode
 *
 *  This determines whether the driver operates synchronously or asynchronously
 *
 *  In #CANFD_TRANSFER_MODE_BLOCKING mode #CANFD_write() blocks code
 *  execution until the transaction has completed
 *
 *  In #CANFD_TRANSFER_MODE_CALLBACK #CANFD_write() does not block code
 *  execution and instead calls a #CANFD_TransferCallbackFxn callback function 
 *  when the transaction has completed
 *
 *  @{
 */
/**
 *  \brief #CANFD_write() blocks execution. This mode can only be used
 *  when called within a Task context
 */
#define CANFD_TRANSFER_MODE_BLOCKING    (0U)
/**
 *  \brief #CANFD_write() does not block code execution and will call a
 *  #CANFD_TransferCallbackFxn. This mode can be used in a Task, Swi, or Hwi context
 */
#define CANFD_TRANSFER_MODE_CALLBACK    (1U)
/** @} */

/**
 *  \anchor CANFD_OperatingMode
 *  \name Operating Mode
 *
 *  Values used to determine the CANFD driver operation.
 *
 *  @{
 */
#define CANFD_OPER_MODE_POLLED                 (0U)
#define CANFD_OPER_MODE_INTERRUPT              (1U)
#define CANFD_OPER_MODE_DMA                    (2U)
/** @} */

/**
 *  @{
 * \brief The section has a list of all the data structures which are exposed
 *  to the application
 *
 */

/*! \brief  Standard ID Filter Element Size */
#define MCAN_MSG_RAM_STD_ELEM_SIZE          (1U)

/*! \brief  Extended ID Filter Element Size */
#define MCAN_MSG_RAM_EXT_ELEM_SIZE          (2U)

/*! \brief  MCAN Header size in Bytes */
#define MCAN_MSG_HEADER_SIZE                (8U)

/*! \brief  MCAN Frame type FD */
#define MCAN_FRAME_TYPE_FD                  (1U)

/*! \brief  Tx/Rx Element Size.
 * 18 words = 18 * 4 = 72 bytes: 8 bytes of header and 64 bytes of data */
#define MCAN_MSG_RAM_TX_RX_ELEM_SIZE        (18U)

/*! \brief  Message Identifier Masks */
#define XTD_MSGID_MASK                      (0x1fffffffU)
#define STD_MSGID_MASK                      (0x7ffU)
#define STD_MSGID_SHIFT                     (18U)

/*! \brief  Maximum payload supported by CAN-FD protocol in bytes. */
#define MCAN_MAX_PAYLOAD_BYTES              (64U)

/*! \brief  Maximum number of Rx buffers. */
#define MCAN_MAX_RX_BUFFERS                 (64U)

/*! \brief  Maximum number of Tx buffers. */
#define MCAN_MAX_TX_BUFFERS                 (32U)

/*! \brief Macro to get the size of an array */
#define CANFD_UTILS_ARRAYSIZE(x)  (sizeof(x) / sizeof(x[0]))

/*! \brief Get the index of the given element within an array */
#define CANFD_UTILS_GETARRAYINDEX(member, array)   (member - &array[0])

/*! \brief Macro to determine if a member is part of an array */
#define CANFD_UTILS_ARRAYISMEMBER(member, array)                              \
    (((((uint32)member - (uint32) & array[0]) % sizeof(array[0])) == 0)       \
     && (member >= &array[0])                                                 \
     && (CANFD_UTILS_GETARRAYINDEX(member, array) < CANFD_UTILS_ARRAYSIZE(array)))

/**
 * \brief  Defines all the interrupt are enabled.
 */
#define MCAN_INTR_MASK       ((uint32_t)MCAN_INTR_SRC_RX_FIFO0_NEW_MSG | \
                              (uint32_t)MCAN_INTR_SRC_RX_FIFO0_MSG_LOST |  \
                              (uint32_t)MCAN_INTR_SRC_RX_FIFO1_NEW_MSG | \
                              (uint32_t)MCAN_INTR_SRC_TRANS_COMPLETE |  \
                              (uint32_t)MCAN_INTR_SRC_TRANS_CANCEL_FINISH |  \
                              (uint32_t)MCAN_INTR_SRC_DEDICATED_RX_BUFF_MSG |  \
                              (uint32_t)MCAN_INTR_SRC_PROTOCOL_ERR_ARB |  \
                              (uint32_t)MCAN_INTR_SRC_PROTOCOL_ERR_DATA |  \
                              (uint32_t)MCAN_INTR_SRC_RX_FIFO1_MSG_LOST | \
                              (uint32_t)MCAN_INTR_SRC_BUS_OFF_STATUS)

/*! \brief completion type for Tx in dma mode - Intermediate completion */
#define CANFD_DMA_TX_COMPLETION_INTERMEDIATE        (1U)
/*! \brief completion type for Tx in dma mode - Final completion */
#define CANFD_DMA_TX_COMPLETION_FINAL               (2U)
/*! \brief completion type for Rx in dma mode - Intermediate completion */
#define CANFD_DMA_RX_COMPLETION_INTERMEDIATE        (1U)
/*! \brief completion type for Rx in dma mode - Final completion */
#define CANFD_DMA_RX_COMPLETION_FINAL               (2U)

/*! \brief Maximum data to DLC mapping supported. Refer \ref MCAN_DataLengthSize */
#define CANFD_MAX_DLC_MAPPING                        (16U)
/** @} */

/*!
 * @{
 *  \anchor   CANFD_DriverState
 *  \name     CANFD Driver State
 *  
 *  \brief    This macros defines the values used to represent the CANFD driver state
 *  
 */

/*! CANFD controller not initialized */
#define CANFD_DRIVER_STATE_UNINIT        (0U)
/*! CANFD controller started */
#define CANFD_DRIVER_STATE_STARTED       (1U)
/*! CANFD controller stopped */
#define CANFD_DRIVER_STATE_STOPPED       (2U)
/*! CANFD controller in sleep mode */
#define CCANFD_DRIVER_STATE_SLEEP        (3U)
/** @} */

/**
 *  \anchor   CANFD_MCANElemSize
 *  \name     CANFD Element Size
 * 
 *  \brief    This enumeration defines the MCAN FIFO/Buffer element Size
 */
typedef enum CANFD_MCANElemSize_t
{
    /*! 8 byte data field */
    CANFD_MCANElemSize_8BYTES = 0U,

    /*! 12 byte data field */
    CANFD_MCANElemSize_12BYTES = 1U,

    /*! 16 byte data field */
    CANFD_MCANElemSize_16BYTES = 2U,

    /*! 20 byte data field */
    CANFD_MCANElemSize_20BYTES = 3U,

    /*! 24 byte data field */
    CANFD_MCANElemSize_24BYTES = 4U,

    /*! 32 byte data field */
    CANFD_MCANElemSize_32BYTES = 5U,

    /*! 48 byte data field */
    CANFD_MCANElemSize_48BYTES = 6U,

    /*! 64 byte data field */
    CANFD_MCANElemSize_64BYTES = 7U
}CANFD_MCANElemSize;

/*! 
 *
 *  \anchor   CANFD_MCANOperationMode
 *  \name     CANFD Operation Mode
 * 
 *  \brief    Enumerates the values used to represent the MCAN mode of operation
 */
typedef enum CANFD_MCANOperationMode_t
{
    /*! MCAN normal mode */
    CANFD_MCANOperationMode_NORMAL = 0U,

    /*! MCAN SW initialization mode */
    CANFD_MCANOperationMode_SW_INIT = 1U
}CANFD_MCANOperationMode;

/*!
 *  \anchor   CANFD_Direction
 *  \name     CANFD Direction
 * 
 *  \brief    This enumeration defines the values used to set the direction
 *            of message object
 */
typedef enum CANFD_Direction_t
{
    /*! Message object used to receive data */
    CANFD_Direction_RX,

    /*! Message object used to transmit data */
    CANFD_Direction_TX
} CANFD_Direction;

/*!
 *  \anchor   CANFD_MCANXidType
 *  \name     CANFD Identifier Type
 * 
 *  \brief    This enumeration defines the values used to represent the 
 *            CAN Identifier Type
 */
typedef enum CANFD_MCANXidType_t
{
    /*! 11bit MCAN Identifier */
    CANFD_MCANXidType_11_BIT,

    /*! 29bit MCAN Identifier */
    CANFD_MCANXidType_29_BIT
} CANFD_MCANXidType;

/*!
 *  \anchor    CANFD_MCANFrameType
 *  \name      CANFD Frame Type
 * 
 *  \brief    This enumeration defines the CAN frame type
 */
typedef enum CANFD_MCANFrameType_t
{
    /*! Classic Frame */
    CANFD_MCANFrameType_CLASSIC,

    /*! FD Frame */
    CANFD_MCANFrameType_FD
} CANFD_MCANFrameType;

/**
 *  \anchor CANFD_MCANTimeOutSelect
 *  \name   CANFD Time out select
 * 
 *  \brief    This enumeration defines the MCAN timeout counter configuration
 */
typedef enum CANFD_MCANTimeOutSelect_t
{
    /*! Continuous operation Mode */
    CANFD_MCANTimeOutSelect_CONT = 0U,

    /*! Timeout controlled by Tx Event FIFO */
    CANFD_MCANTimeOutSelect_TX_EVENT_FIFO = 1U,

    /*! Timeout controlled by Rx FIFO 0 */
    CANFD_MCANTimeOutSelect_RX_FIFO0 = 2U,

    /*! Timeout controlled by Rx FIFO 1 */
    CANFD_MCANTimeOutSelect_RX_FIFO1 = 3U
}CANFD_MCANTimeOutSelect;

/**
 *  \anchor CANFD_MCANECCErrType
 *  \name   CANFD ECC error type
 *  @{
 */
/**
 *  \brief    This enumeration defines the MCAN ECC Error Types
 */
typedef uint32_t CANFD_MCANECCErrType;

#define CANFD_MCAN_ECC_ERR_TYPE_SEC                   (0U)
/**< ECC Single Error Correction */
#define CANFD_MCAN_ECC_ERR_TYPE_DED                   (1U)
/**< ECC Single Error Detection */
/** @} */

/**
 *  \anchor CANFD_MCANLoopBackMode
 *  \name   CANFD Loopback mode
 *
 *  \brief    This enumeration defines the MCAN Loopback mode
 */
typedef enum CANFD_MCANLoopBackMode_t
{
    /*! This mode can be used for hot self-test and this mode will not affect bus state. */
    CANFD_MCANLoopBackMode_INTERNAL = 0U,

    /*! In this mode, MCAN the MCAN treats its own transmitted messages 
     *  as received messages and stores them (if they pass acceptance filtering) 
     *  into an Rx Buffer or an Rx FIFO. This mode will affect bus state.
     */
    CANFD_MCANLoopBackMode_EXTERNAL = 1U
}CANFD_MCANLoopBackMode;

/**
 *  \anchor CANFD_MCANCommState
 *  \name   CANFD Communication State
 *
 *  \brief    This enumeration defines the MCAN's communication state
 * 
 */
typedef enum CANFD_MCANCommState_t
{
    /*! MCAN is synchronizing on CANFD communication */
    CANFD_MCANCommState_SYNCHRONIZING = 0U,

    /*! MCAN is neither receiver nor transmitter */
    CANFD_MCANCommState_IDLE = 1U,

    /*! MCAN is operating as receiver */
    CANFD_MCANCommState_RECEIVER = 2U,

    /*! MCAN is operating as transmitter */
    CANFD_MCANCommState_TRANSMITTER = 3U
}CANFD_MCANCommState;

/**
 *  \anchor CANFD_MCANErrCode
 *  \name   CANFD Error Code
 *
 *  \brief    This enumeration defines the  MCAN's Error Code
 */
typedef enum CANFD_MCANErrCode_t
{
    /*! No error occurred since LEC has been reset by
     *  successful reception or transmission.
     */
    CANFD_MCANErrCode_NO_ERROR = 0U,

    /*! More than 5 equal bits in a sequence have occurred in a part of
     *  a received message where this is not allowed.
     */
    CANFD_MCANErrCode_STUFF_ERROR = 1U,

    /*! A fixed format part of a received frame has the wrong format. */
    CANFD_MCANErrCode_FORM_ERROR = 2U,

    /*! The message transmitted by the MCAN was not acknowledged by another node. */
    CANFD_MCANErrCode_ACK_ERROR = 3U,

    /*! During the transmission of a message (with the exception of
     *  the arbitration field), the device wanted to send a
     *  recessive level (bit of logical value 1),
     *  but the monitored bus value was dominant.
     */
    CANFD_MCANErrCode_BIT1_ERROR = 4U,

    /*! During the transmission of a message (or acknowledge bit,
     *  or active error flag, or overload flag), the device wanted to send
     *  a dominant level (data or identifier bit logical value 0),
     *  but the monitored bus value was recessive. During Bus_Off recovery
     *  this status is set each time a sequence of 11 recessive bits has been
     *  monitored. This enables the CPU to monitor the proceeding of
     *  the Bus_Off recovery sequence (indicating the bus is not stuck at
     *  dominant or continuously disturbed).
     */
    CANFD_MCANErrCode_BIT0_ERROR = 5U,

    /*! The CRC check sum of a received message was incorrect.
     *   The CRC of an incoming message does not match with the
     *   CRC calculated from the received data.
     */
    CANFD_MCANErrCode_CRC_ERROR = 6U,

    /*! Any read access to the Protocol Status Register re-initializes the
     *   LEC to 7. When the LEC shows the value 7, no CANFD bus event was 
     *   detected since the last CPU read access to the Protocol Status Register.
     */
    CANFD_MCANErrCode_NO_CHANGE = 7U
}CANFD_MCANErrCode;

/**
 *  \anchor CANFD_Reason
 *  \name   CANFD reason
 *
 *  \brief  This enumeration describes a list of all the reasons for which
 *          the driver will invoke application callback functions.   
 */
typedef enum CANFD_Reason_t
{
    /**
     * \brief  Data has been received and the application is required to 
     *         read and process the data.
     */
    CANFD_Reason_RX                         = 0x1,

    /**
     * \brief  Data has been succesfully transmitted.
     */
    CANFD_Reason_TX_COMPLETION              = 0x2,

    /**
     * \brief  Data transmission is succesfully canceled.
     */
    CANFD_Reason_TX_CANCELED                = 0x3,

    /**
     * \brief  Data has been succesfully transmitted.
     */
    CANFD_Reason_ECC_ERROR                  = 0x4,

    /**
     * \brief  Bus Off condition detected.
     */
    CANFD_Reason_BUSOFF                     = 0x5,

    /**
     * \brief  Protocol error in data phase detected.
     */
    CANFD_Reason_PROTOCOL_ERR_DATA_PHASE    = 0x6,

    /**
     * \brief  Protocol error in arbitration phase detected.
     */
    CANFD_Reason_PROTOCOL_ERR_ARB_PHASE     = 0x7,
    
    /**
     * \brief  Rx FIFO 0 Message Lost.
     */
    CANFD_Reason_SRC_RX_FIFO0_MSG_LOST     = 0x8,

    /**
     * \brief  Rx FIFO 1 Message Lost.
     */
    CANFD_Reason_SRC_RX_FIFO1_MSG_LOST     = 0x9
}CANFD_Reason;

/*!
 *  \anchor CANFD_Option
 *  \name   CANFD Option
 *
 *  \brief    This enumeration defines the values used to represent the GET/SET options
 *
 *  @sa
 *  CANFD_OptionTLV 
 */
typedef enum CANFD_Option_t
{
    /*! Used to get the MCAN Tx and Rx error counters
     * @sa CANFD_getOptions
     *
     * NOTE: The length in the TLV should be sizeof(CANFD_MCANErrCntStatus) 
     *       for this option.
     */
    CANFD_Option_MCAN_ERROR_COUNTER,

    /*! Used to get the MCAN protocol status
     * @sa CANFD_getOptions
     *
     * NOTE: The length in the TLV should be sizeof(CANFD_MCANProtocolStatus) 
     *       for this option.
     */
    CANFD_Option_MCAN_PROTOCOL_STATUS,

    /*! Used to get the MCAN message object software maintained statistics
     * @sa CANFD_getOptions
     *
     * NOTE: The length in the TLV should be sizeof(CANFD_MCANMsgObjectStats)
     *       for this option.
     *       Application must fill in the message object handle for which the 
     *       statistics is requested.
     *
     */
    CANFD_Option_MCAN_MSG_OBJECT_STATS,

    /*! Used to put the MCAN module in init or operational state
     * @sa CANFD_setOptions
     *
     * NOTE: The length in the TLV should be 1 byte for this option.
     * Valid values: Refer to (CANFD_MCANOperationMode)
     */
    CANFD_Option_MCAN_MODE,

    /*! Used to enable or disable internal/external loopback mode
     * @sa CANFD_setOptions
     *
     * NOTE: The length in the TLV should be sizeof(CANFD_MCANLoopbackCfgParams)
     *       for this option.
     *
     */
    CANFD_Option_MCAN_LOOPBACK,

    /*! Used to request a local power down or wakeup from a local power down
     * @sa CANFD_setOptions
     *
     * NOTE: The length in the TLV should be 1 byte for this option.
     * Valid values are
     *  1   -   MCAN Sleep
     *  0   -   MCAN Wakeup
     *
     */
    CANFD_Option_MCAN_POWER_DOWN
} CANFD_Option;

/**
 *  \anchor CANFD_MCANLoopbackCfgParams
 *  \name   CANFD Loopabck configuration parameters
 *
 * \brief  Data structure defines the MCAN Loopback parameters.
 */
typedef struct CANFD_MCANLoopbackCfgParams_t
{
    /*! Enable or disable loopback mode
     * Valid values are
     *  0   -   Disable
     *  1   -   Enable
     */
    uint32_t                enable;

    /*! Loopback mode: Internal or External */
    CANFD_MCANLoopBackMode  mode;
}CANFD_MCANLoopbackCfgParams;

/**
 *  \anchor CANFD_MCANBitTimingParams
 *  \name   CANFD Bit Timimg Parameters
 *
 * \brief   Data structure defines the parameters for bit timing calculation.
 *          Bit timing related to data phase will be valid only in case where
 *          MCAN is put in CANFD mode and will be '0' otherwise.
 */
typedef struct CANFD_MCANBitTimingParams_t
{
    /*! Nominal Baud Rate Pre-scaler */
    uint32_t        nomBrp;

    /*! NominalProp Segment value */
    uint32_t        nomPropSeg;

    /*! NominalPhase Segment1 value */
    uint32_t        nomPseg1;

    /*! NominalPhase Segment2 value */
    uint32_t        nomPseg2;

    /*! Nominal (Re)Synchronization Jump Width */
    uint32_t        nomSjw;

    /*! Nominal Baud Rate Pre-scaler */
    uint32_t        dataBrp;

    /*! NominalProp Segment value */
    uint32_t        dataPropSeg;

    /*! NominalPhase Segment1 value */
    uint32_t        dataPseg1;

    /*! NominalPhase Segment2 value */
    uint32_t        dataPseg2;

    /*! Nominal (Re)Synchronization Jump Width */
    uint32_t        dataSjw;
}CANFD_MCANBitTimingParams;

/**
 *  \anchor CANFD_MCANTdcConfig
 *  \name   CANFD TDC Config
 *
 * \brief  Data structure defines the MCAN Transmitter Delay Compensation parameters.
 */
typedef struct CANFD_MCANTdcConfig_t
{
    /*! Transmitter Delay Compensation Filter Window Length
     *   Range: [0x0-0x7F]
     */
    uint32_t        tdcf;

    /*! Transmitter Delay Compensation Offset
     *   Range: [0x0-0x7F]
     */
    uint32_t        tdco;
}CANFD_MCANTdcConfig;

/**
 *  \anchor CANFD_MCANGlobalFiltConfig
 *  \name   CANFD Global Filter Config
 *
 * \brief   Data structure defines the MCAN Global Filter Configuration parameters.
 */
typedef struct CANFD_MCANGlobalFiltConfig_t
{
    /*! Reject Remote Frames Extended
     *   0 = Filter remote frames with 29-bit extended IDs
     *   1 = Reject all remote frames with 29-bit extended IDs
     */
    uint32_t        rrfe;

    /*! Reject Remote Frames Standard
     *   0 = Filter remote frames with 11-bit standard IDs
     *   1 = Reject all remote frames with 11-bit standard IDs
     */
    uint32_t        rrfs;

    /*! Accept Non-matching Frames Extended
     *   0 = Accept in Rx FIFO 0
     *   1 = Accept in Rx FIFO 1
     *   others = Reject
     */
    uint32_t        anfe;

    /*! Accept Non-matching Frames Standard
     *   0 = Accept in Rx FIFO 0
     *   1 = Accept in Rx FIFO 1
     *   others = Reject
     */
    uint32_t        anfs;
}CANFD_MCANGlobalFiltConfig;

/**
 *  \anchor CANFD_MCANMsgRAMCfgParams
 *  \name   CANFD Message RAM Config
 *
 * \brief   Data structure defines the MCAN Message RAM Configuration Parameters.
 *          Message RAM can contain following sections:
 *          Standard ID filters, Extended ID filters, TX FIFO(or TX Q),
 *          TX Buffers, TX EventFIFO, RX FIFO0, RX FIFO1, RX Buffer.
 *          Note: If particular section in the RAM is not used then it's size
 *          should be initialized to '0'
 *          (Number of buffers in case of Tx/Rx buffer).
 */
typedef struct CANFD_MCANMsgRAMCfgParams_t
{
    /*! List Size: Standard ID
     *   0 = No standard Message ID filter
     *   1-127 = Number of standard Message ID filter elements
     *   others = Values greater than 128 are interpreted as 128
     */
    uint32_t            lss;

    /*! List Size: Extended ID
     *   0 = No standard Message ID filter
     *   1-64 = Number of standard Message ID filter elements
     *   others = Values greater than 64 are interpreted as 64
     */
    uint32_t            lse;

    /*! Number of Dedicated Transmit Buffers
     *   0 = No Dedicated Tx Buffers
     *   1-32 = Number of Dedicated Tx Buffers
     *   others = Values greater than 32 are interpreted as 32
     */
    uint32_t            txBufNum;

    /*! Transmit FIFO/Queue Size
     *   0 = No Tx FIFO/Queue
     *   1-32 = Number of Tx Buffers used for Tx FIFO/Queue
     *   others = Values greater than 32 are interpreted as 32
     */
    uint32_t            txFIFOSize;

    /*! Tx FIFO/Queue Mode
     *   0 = Tx FIFO operation
     *   1 = Tx Queue operation
     */
    uint32_t            txBufMode;

    /*! Event FIFO Size
     *   0 = Tx Event FIFO disabled
     *   1-32 = Number of Tx Event FIFO elements
     *   others = Values greater than 32 are interpreted as 32
     */
    uint32_t            txEventFIFOSize;

    /*! Tx Event FIFO Watermark
     *   0 = Watermark interrupt disabled
     *   1-32 = Level for Tx Event FIFO watermark interrupt
     *   others = Watermark interrupt disabled
     */
    uint32_t            txEventFIFOWaterMark;

    /*! Rx FIFO0 Size
     *   0 = No Rx FIFO
     *   1-64 = Number of Rx FIFO elements
     *   others = Values greater than 64 are interpreted as 64
     */
    uint32_t            rxFIFO0size;

    /*! Rx FIFO0 Watermark
     *   0 = Watermark interrupt disabled
     *   1-63 = Level for Rx FIFO 0 watermark interrupt
     *   others = Watermark interrupt disabled
     */
    uint32_t            rxFIFO0waterMark;

    /*! Rx FIFO0 Operation Mode
     *   0 = FIFO blocking mode
     *   1 = FIFO overwrite mode
     */
    uint32_t            rxFIFO0OpMode;

    /*! Rx FIFO1 Size
     *   0 = No Rx FIFO
     *   1-64 = Number of Rx FIFO elements
     *   others = Values greater than 64 are interpreted as 64
     */
    uint32_t            rxFIFO1size;

    /*! Rx FIFO1 Watermark
     *   0 = Watermark interrupt disabled
     *   1-63 = Level for Rx FIFO 1 watermark interrupt
     *   others = Watermark interrupt disabled
     */
    uint32_t            rxFIFO1waterMark;

    /*! Rx FIFO1 Operation Mode
     *   0 = FIFO blocking mode
     *   1 = FIFO overwrite mode
     */
    uint32_t            rxFIFO1OpMode;
}CANFD_MCANMsgRAMCfgParams;

/**
 *  \anchor CANFD_MCANECCConfigParams
 *  \name   CANFD ECC Config Parameters
 * 
 * \brief   Data structure defines the MCAN ECC configuration parameters.
 */
typedef struct CANFD_MCANECCConfigParams_t
{
    /*! Enable/disable ECC
     *   0 = Disable ECC
     *   1 = Enable ECC
     */
    uint32_t            enable;

    /*! Enable/disable ECC Check
     *   0 = Disable ECC Check
     *   1 = Enable ECC Check
     */
    uint32_t            enableChk;

    /*! Enable/disable Read Modify Write operation
     *   0 = Disable Read Modify Write operation
     *   1 = Enable Read Modify Write operation
     */
    uint32_t            enableRdModWr;
}CANFD_MCANECCConfigParams;

/**
 *  \anchor CANFD_MCANErrCntStatus
 *  \name   CANFD Error Counter Status
 * 
 * \brief   Data structure defines the MCAN error logging counters status.
 */
typedef struct CANFD_MCANErrCntStatus_t
{
    /*! Transmit Error Counter */
    uint32_t            transErrLogCnt;

    /*! Receive Error Counter */
    uint32_t            recErrCnt;

    /*! Receive Error Passive
     *   0 = The Receive Error Counter is below the error passive level(128)
     *   1 = The Receive Error Counter has reached the error passive level(128)
     */
    uint32_t            rpStatus;

    /*! CAN Error Logging */
    uint32_t            canErrLogCnt;
}CANFD_MCANErrCntStatus;

/**
 *  \anchor CANFD_MCANProtocolStatus
 *  \name   CANFD Protocol Status
 * 
 * \brief   Data structure defines the MCAN protocol status.
 */
typedef struct CANFD_MCANProtocolStatus_t
{
    /*! Last Error Code
     *   Refer enum \ref CANFD_MCANErrCode
     */
    uint32_t                lastErrCode;

    /*! Activity - Monitors the module's CAN communication state.
     *   refer enum \ref CANFD_MCANCommState
     */
    uint32_t                act;

    /*! Error Passive
     *   0 = The M_CAN is in the Error_Active state
     *   1 = The M_CAN is in the Error_Passive state
     */
    uint32_t                errPassive;

    /*! Warning Status
     *   0 = Both error counters are below the Error_Warning limit of 96
     *   1 = At least one of error counter has reached the Error_Warning
     *       limit of 96
     */
    uint32_t                warningStatus;

    /*! Bus_Off Status
     *   0 = The M_CAN is not Bus_Off
     *   1 = The M_CAN is in Bus_Off state
     */
    uint32_t                busOffStatus;

    /*! Data Phase Last Error Code
     *   Refer enum \ref CANFD_MCANErrCode
     */
    uint32_t                dlec;

    /*! ESI flag of last received CAN FD Message
     *   0 = Last received CAN FD message did not have its ESI flag set
     *   1 = Last received CAN FD message had its ESI flag set
     */
    uint32_t                resi;

    /*! BRS flag of last received CAN FD Message
     *   0 = Last received CAN FD message did not have its BRS flag set
     *   1 = TLast received CAN FD message had its BRS flag set
     */
    uint32_t                rbrs;

    /*! Received a CAN FD Message
     *   0 = Since this bit was reset by the CPU, no CAN FD message has been
     *       received
     *   1 = Message in CAN FD format with FDF flag set has been received
     */
    uint32_t                rfdf;

    /*! Protocol Exception Event
     *   0 = No protocol exception event occurred since last read access
     *   1 = Protocol exception event occurred
     */
    uint32_t                pxe;

    /*! Transmitter Delay Compensation Value */
    uint32_t                tdcv;
}CANFD_MCANProtocolStatus;

/**
 *  \anchor CANFD_MCANECCErrForceParams
 *  \name   CANFD ECC Force Param
 * 
 * \brief   Data structure defines the ECC Error forcing.
 */
typedef struct CANFD_MCANECCErrForceParams_t
{
    /*! Error type to be forced
     *   Refer enum \ref CANFD_MCANECCErrType.
     */
    uint32_t                errType;

    /*! Row address where error needs to be applied. */
    uint32_t                rowNum;

    /*! Column/Data bit that needs to be flipped when
     *   force_sec or force_ded is set
     */
    uint32_t                bit1;

    /*! Data bit that needs to be flipped when force_ded is set */
    uint32_t                bit2;

    /*! Force Error once
     *   1: The error will inject an error to the specified row only once
     */
    uint32_t                errOnce;

    /*! Force error on the next RAM read */
    uint32_t                errForce;
}CANFD_MCANECCErrForceParams;

/**
 *  \anchor CANFD_MCANECCErrStatus
 *  \name   CANFD ECC Error Status
 * 
 * @br
 * \brief   Data structure defines the ECC Error Status.
 */
typedef struct CANFD_MCANECCErrStatus_t
{
    /*! Single Bit Error Status
     *   0 = No Single Bit Error pending
     *   1 = Single Bit Error pending
     */
    uint32_t                secErr;

    /*! Double Bit Error Status
     *   0 = No Double Bit Error pending
     *   1 = Double Bit Error pending
     */
    uint32_t                dedErr;

    /*! Indicates the row/address where the single or double bit
     *   error occurred.
     */
    uint32_t                row;

    /*! Indicates the bit position in the ram data that is in error
     */
    uint32_t                bit1;

    /*! Indicates the bit position in the ram data that is in error
     *   Valid only in case of DED.
     */
    uint32_t                bit2;
}CANFD_MCANECCErrStatus;

/*!
 *  \anchor CANFD_ErrStatusResp
 *  \name   CANFD ECC Error Status response
 * 
 * \brief
 *  Response structure definition for Error and status information.
 */
typedef union CANFD_ErrStatusResp_t
{
    /*! ECC Error Status. */
    CANFD_MCANECCErrStatus      eccErrStatus;

    /*! Protocol Status. */
    CANFD_MCANProtocolStatus    protocolStatus;
} CANFD_ErrStatusResp;


/*!
 *  \anchor CANFD_Attrs
 *  \name   CANFD Attributes
 * 
 * \brief
 *  CANFD instance attributes - used during init time
 */
typedef struct CANFD_Attrs_s
{
    /* Peripheral base address */
    uint32_t                baseAddr;

    /**< Peripheral interrupt number */
    uint32_t                intrNum0;

    /**< Peripheral interrupt number */
    uint32_t                intrNum1;

    /**< Driver operating mode. Polling, DMA, interrupt */
    uint32_t                operMode;

    /**< Interrupt priority */
    uint8_t                 intrPriority;

    /**< Driver operating mode. Polling, DMA, interrupt */
    uint32_t                OptionTLVtype;

    /* MCAN Loopback parameters. */
    CANFD_MCANLoopbackCfgParams    CANFDMcanloopbackParams;

    /* parameters for bit timing calculation. */
    CANFD_MCANBitTimingParams      CANFDMcanBitTimingParams;

    /**<  Tx message storage type. Refer enum #MCAN_MemType. */
    uint32_t         txMemType;

    /**<  Rx message storage type. Refer enum #MCAN_MemType. */
    uint32_t         rxMemType;

    /**
     * \brief    void pointer to #MCAN_ExtMsgIDFilterElement or 
     *           #MCAN_StdMsgIDFilterElement structure. It will point 
     *            to any of the two structure based on fdMode. 
     */
    void                *filterConfig;
} CANFD_Attrs;

/**
 *  \anchor CANFD_OptionTLV
 *  \name   CANFD Option TLV
 * 
 * \brief
 *  Options TLV data structure
 *
 * @details
 *  Specifies the option type, length, value.
 */
typedef struct CANFD_OptionTLV_t
{
    /**
     * \brief   Option Name
     */
    CANFD_Option            type;

    /**
     * \brief   Option Length
     */
    int32_t                 length;

    /**
     * \brief   Option Value
     */
    void*                   value;
}CANFD_OptionTLV;

/**
 *  \brief  The definition of a callback function used by the CANFD driver
 *  when used in #CANFD_TRANSFER_MODE_CALLBACK
 *
 *  \param args*           void pointer
 *  \param reason          Cause of the interrupt which prompted the callback.
 */
typedef void (*CANFD_TransferCallbackFxn) (void *args, CANFD_Reason reason);

/**
 *  \brief  The definition of a callback function used by the CANFD driver
 *  when used in #CANFD_TRANSFER_MODE_CALLBACK in case of error
 *
 *  \param args*           void pointer
 *  \param reason          Cause of the interrupt which prompted the callback.
 *  \param errStatusResp   Erros status Response
 */
typedef void (*CANFD_ErrorCallbackFxn) (void *args, CANFD_Reason reason, 
                          CANFD_ErrStatusResp* errStatusResp);


/**
 *  \anchor CANFD_OpenParams
 *  \name   CANFD open params
 *  
 * \brief CANFD Parameters. Data structure defines the MCAN initialization parameters.
 *
 *  CANFD Parameters are used to with the #CANFD_open() call.
 */
typedef struct CANFD_OpenParams_t
{
    /*! Blocking or Callback mode. Refer \ref CANFD_TransferMode
     */
    uint32_t                transferMode;

    /**< Transfer Callback function pointer */
    CANFD_TransferCallbackFxn       transferCallbackFxn;

    /**< Error Callback function pointer */
    CANFD_ErrorCallbackFxn           errorCallbackFxn;

    /*! FD Operation Enable
     *   0 = FD operation disabled
     *   1 = FD operation enabled
     */
    uint32_t                    fdMode;

    /*! Bit Rate Switch Enable
     *   This is valid only when opMode = 1.
     *   0 = Bit rate switching for transmissions disabled
     *   1 = Bit rate switching for transmissions enabled
     */
    uint32_t                    brsEnable;

    /*! Transmit Pause
     *   0 = Transmit pause disabled
     *   1 = Transmit pause enabled
     */
    uint32_t                    txpEnable;

    /*! FEdge Filtering during Bus Integration
     *   0 = Edge filtering disabled
     *   1 = Two consecutive dominant tq required to detect an edge for
     *       hard synchronization
     */
    uint32_t                    efbi;

    /*! Protocol Exception Handling Disable
     *   0 = Protocol exception handling enabled
     *   1 = Protocol exception handling disabled
     */
    uint32_t                    pxhddisable;

    /*! Disable Automatic Retransmission
     *   0 = Automatic retransmission of messages not transmitted successfully
     *       enabled
     *   1 = Automatic retransmission disabled
     */
    uint32_t                    darEnable;

    /*! Wakeup Request Enable
     *   0 = Wakeup request is disabled
     *   1 = Wakeup request is enabled
     */
    uint32_t                    wkupReqEnable;

    /*! Auto-Wakeup Enable
     *   0 = Auto-Wakeup is disabled
     *   1 = Auto-Wakeup is enabled
     */
    uint32_t                    autoWkupEnable;

    /*! Emulation/Debug Suspend Enable
     *   0 = Emulation/Debug Suspend is disabled
     *   1 = Emulation/Debug Suspend is enabled
     */
    uint32_t                    emulationEnable;

    /*! Emulation/Debug Suspend Fast Ack Enable
     *   0 = Emulation/Debug Suspend does not wait for idle/immediate effect
     *   1 = Emulation/Debug Suspend waits for idle/graceful stop
     */
    uint32_t                    emulationFAck;

    /*! Clock Stop Fast Ack Enable
     *   0 = Clock Stop does not wait for idle/immediate effect
     *   1 = Clock Stop waits for idle/graceful stop
     */
    uint32_t                    clkStopFAck;

    /*! Start value of the Message RAM Watchdog Counter
     *   Range:[0x0-0xFF]
     */
    uint32_t                    wdcPreload;

    /*! Transmitter Delay Compensation Enable
     *   0 = Transmitter Delay Compensation is disabled
     *   1 = Transmitter Delay Compensation is enabled
     */
    uint32_t                    tdcEnable;

    /*! Transmitter Delay Compensation parameters.
     *   Refer struct \ref CANFD_MCANTdcConfig.
     */
    CANFD_MCANTdcConfig         tdcConfig;

    /*! Bus Monitoring Mode
     *   0 = Bus Monitoring Mode is disabled
     *   1 = Bus Monitoring Mode is enabled
     */
    uint32_t                    monEnable;

    /*! Restricted Operation Mode
     *   0 = Normal CAN operation
     *   1 = Restricted Operation Mode active
     *   This mode should not be combined with test modes.
     */
    uint32_t                    asmEnable;

    /*! Timestamp Counter Prescaler.
     *   Range:[0x0-0xF]
     */
    uint32_t                    tsPrescalar;

    /*! Timeout source selection.
     *   00b: Timestamp counter value always 0x0000
     *   01b: Timestamp counter value incremented according to tsPrescalar
     *   10b: External timestamp counter value used
     *   11b: Same as 00b
     */
    uint32_t                    tsSelect;

    /*! Time-out counter source select.
     *   Refer enum \ref CANFD_MCANTimeOutSelect.
     */
    CANFD_MCANTimeOutSelect     timeoutSelect;

    /*! Start value of the Timeout Counter (down-counter).
     *   The Timeout Counter is decremented in multiples of CAN bit times [1-16]
     *   depending on the configuration of the tsPrescalar.
     *   Range: [0x0-0xFFFF]
     */
    uint32_t                    timeoutPreload;

    /*! Timeout Counter Enable
     *   0 - Timeout Counter is disabled
     *   1 - Timeout Counter is enabled
     */
    uint32_t                    timeoutCntEnable;

    /*! Global Filter Configuration parameters.
     *    Refer struct \ref CANFD_MCANGlobalFiltConfig.
     */
    CANFD_MCANGlobalFiltConfig  filterConfig;

    /*! Message RAM Configuration parameters.
     *    Refer struct \ref CANFD_MCANMsgRAMCfgParams.
     */
    CANFD_MCANMsgRAMCfgParams   msgRAMConfig;

    /*! ECC Configuration parameters.
     *    Refer struct \ref CANFD_MCANECCConfigParams.
     */
    CANFD_MCANECCConfigParams   eccConfig;

    /*! Enable/Disable error/status interrupts
     * Note: Must be enabled to receive error and status interrupts. */
    uint32_t                    errInterruptEnable;

    /*! Enable/Disable data interrupts.
     * Note: Must be enabled to receive transmit complete and data receive interrupts. */
    uint32_t                    dataInterruptEnable;
}CANFD_OpenParams;

/**
 *  \anchor CANFD_Object
 *  \name   CANFD Object
 * 
 * \brief
 *  CANFD Master Control Block
 *
 * @details
 *  The structure describes the CANFD Driver and is used to hold the relevant
 *  information with respect to the CANFD module.
 */
typedef struct CANFD_Object_t
{
    /*!
     * \brief   Instance handle to which this object belongs.
     */
    CANFD_Handle            handle;

    /*!
     * \brief   Base address of the register address space to be used.
     */
    uint32_t                regBaseAddress;

    /**
     * \brief   CANFD driver internal state
     */
    uint32_t                state;

    /**
     * \brief   CANFD driver init parameters
     */
    CANFD_OpenParams               *openParams;

    /**
     * \brief   Data Length to DLC mapping
     */
    uint8_t                         mcanDataSize[CANFD_MAX_DLC_MAPPING];

    /**
     * \brief   Message object handle book keeping
     */
    struct CANFD_MessageObject_t*  msgObjectHandle[MCAN_MAX_MSG_OBJECTS];

    /**
     * \brief   Tx Mapping to message handles for transmit post processing
     */
    struct CANFD_MessageObject_t*   txMapping[MCAN_MAX_TX_MSG_OBJECTS];

    /**
     * \brief   Rx Mapping to message handles for Rx processing
     */
    struct CANFD_MessageObject_t*   rxMapping[MCAN_MAX_RX_MSG_OBJECTS];

    /**
     * \brief   Number of error and status interrupts received
     */
    uint32_t                        errStatusInterrupts;

    /**
     * \brief   Number of interrupts received for message Tx or Rx
     */
    uint32_t                        interrupts;

    /**
     * \brief   Number of ECC interrupts received
     */
    uint32_t                        eccInterrupts;

    /**
     * \brief   Number of Bus-Off interrupts received
     */
    uint32_t                        busOffInterrupts;

    /**
     * \brief   Number of Protocol error in data phase interrupts received
     */
    uint32_t                        protoDataErrInterrupts;

    /**
     * \brief   Number of Protocol error in arbitration phase interrupts received
     */
    uint32_t                        protoArbErrInterrupts;

    /**
     * \brief   Tx Status of the message object
     */
    uint8_t                         txStatus[MCAN_MAX_TX_MSG_OBJECTS];

    /**
     * \brief   Flag to toggle the usage of FIFO 0 and FIFO 1. Valid values are 0 and 1.
     */
    uint32_t                        useFifoNum;

    /**
     * \brief   Buffer used to read message RAM.
     */
    MCAN_RxBufElement               rxBuffElem;

    /**
     * \brief   Dma driver handle.
     */
    CANFD_DmaHandle             canfdDmaHandle;

    /**
     * \brief   Pointer to Dma channel configuration.
     */
    CANFD_DmaChConfig           canfdDmaChCfg;

    /**
     * \brief   Pointer to be used by application to store miscellaneous data.
     */
    void*                          args;

    /**< Read Transfer Sync Sempahore - to sync between transfer completion ISR
     *   and task */
    void                   *readTransferSem;

    /**< Read Transfer Sync Sempahore - to sync between transfer completion ISR
     *   and task. Transfer Sync Sempahore object */
    SemaphoreP_Object       readTransferSemObj;

    /**< Write Transfer Sync Sempahore - to sync between transfer completion ISR
     *   and task */
    void                   *writeTransferSem;

    /**< Write Transfer Sync Sempahore - to sync between transfer completion ISR
     *   and task. Transfer Sync Sempahore object */
    SemaphoreP_Object       writeTransferSemObj;

    /**< Interrupt handle for controller ISR */
    void                   *hwiHandle;

    /**< Interrupt object */
    HwiP_Object             hwiObj0;

    /**< Interrupt object */
    HwiP_Object             hwiObj1;

    /* Revision ID and Core Release Info */
    MCAN_RevisionId         revId;

}CANFD_Object;

/**
 *  \anchor CANFD_Config
 *  \name   CANFD Config
 * 
 *  \brief CANFD global configuration array
 *
 *  This structure needs to be defined before calling #CANFD_init() and it must
 *  not be changed by user thereafter.
 *
 *  The last entry of the array should be a NULL entry which demarks the end
 *  of the array.
 */
typedef struct CANFD_Config_s
{
    CANFD_Attrs      *attrs;
    /**< Pointer to driver specific attributes */
    CANFD_Object     *object;
    /**< Pointer to driver specific data object */
} CANFD_Config;

/**
 *  \anchor CANFD_DmaMsgConfig
 *  \name   CANFD DMA Message Config
 * 
 * \brief
 *  CANFD DMA message configuration used for Tx
 */
typedef struct CANFD_DmaMsgConfig_s
{
    /**
     * \brief   data length in Bytes for every message.
     */
    uint32_t        dataLengthPerMsg;
    /**
     * \brief   Number of messages.
     */
    uint32_t        numMsgs;
    /**
     * \brief   pointer to the data buffer. This should a 2d array of uint8[numMsgs][dataLengthPerMsg]
     */
    const void      *data;
    /**
     * \brief   Used by driver to store current message count out of the numMsgs already processed.
     */
    uint32_t        currentMsgNum;
}CANFD_DmaMsgConfig;

/**
 *  \anchor CANFD_DmaRxBuf
 *  \name   CANFD DMA RX Buffer
 * 
 * \brief
 *  CANFD Rx Buffer used in DMA mode
 *
 * @details
 *  This structure defines the Rx buffer format used in DMA mode.
 *  This matches with the Rx Buffer format in message ram. The Dma will be
 *  configured to read the full Rx buffer with header from message ram.
 */
typedef struct CANFD_DmaRxBuf_s
{
    /* Header word 1 in Rx buffer in message ram */
    uint32_t        header1;
    /* Header word 2 in Rx buffer in message ram */
    uint32_t        header2;
    /* Data Bytes Rx buffer in message ram */
    uint8_t         data[MCAN_MAX_RX_BUFFERS];
}CANFD_DmaRxBuf;

/**
 *  \anchor CANFD_MessageObject
 *  \name   CANFD Message Object
 *
 * \brief
 *  CAN message object block
 *
 * @details
 *  The structure defines the message object
 */
typedef struct CANFD_MessageObject_t
{
    /**
     * \brief   Starting range of the Message Id to which the configuration belongs.
     *          For Tx and single Message Id objects the startMsgId = endMsgId.
     */
    uint32_t                startMsgId;

    /**
     * \brief   Ending range of the Message Id to which the configuration belongs
     *          For Tx and single Message Id objects the startMsgId = endMsgId.
     */
    uint32_t                endMsgId;

    /**
     * \brief   Pointer to the CANFD driver object
     */
    CANFD_Config*           canfdHandle;

    /**
     * \brief   Message object direction.
     */
    CANFD_Direction         direction;

    /**
     * \brief   Message object type.
     */
    CANFD_MCANXidType       msgIdType;

    /**
     * \brief   Allocated message object number
     */
    uint32_t                messageObjNum;

    /**
     * \brief   Data Length used by application for transmission and reception.
     *          Valid values: 1 to 64 bytes.
     */
    uint32_t                dataLength;

    /**
     * \brief   Tx buffer number used to send data
     */
    uint32_t                txElement;

    /**
     * \brief   Rx buffer number used to receive data
     */
    uint32_t                rxElement;

    /**
     * \brief   Part of message ram to accessed by this message object. 
     *          Refer enum #MCAN_MemType.
     */
    MCAN_MemType            txMemType;

    /**
     * \brief   Part of message ram to accessed by this message object. 
     *          Refer enum #MCAN_MemType.
     */
    MCAN_MemType            rxMemType;

    /**
     * \brief   FIFO Num (MCAN_RX_FIFO_NUM_0/MCAN_RX_FIFO_NUM_1).
     */
    uint32_t               rxFifoNum;

    /**
     * \brief   Number of interrupts received
     */
    uint32_t                interruptsRxed;

    /**
     * \brief   Number of messages processed
     */
    uint32_t                messageProcessed;

    /**
     * \brief   Dma Event number allocated for this message object
     */
    uint32_t                dmaEventNo;
    /**
     * \brief   Dma message configuration
     */
    CANFD_DmaMsgConfig      dmaMsgConfig;

    /**
     * \brief   Pointer to be used by application to store rx buffer.
     */
    void                   *args;

    /**
     * \brief    void pointer to #MCAN_ExtMsgIDFilterElement or 
     *           #MCAN_StdMsgIDFilterElement structure. It will point  
     *           to any of the two structure based on fdMode. 
     */
    void                *filterConfig;
}CANFD_MessageObject;

/*!
 *  \anchor CANFD_MessageObjectHandle
 *  \name   CANFD Message Handle
 *
 *  \brief      CANFD message object handle returned by the CANFD_createMsgObject() API call.
 */
typedef CANFD_MessageObject* CANFD_MsgObjHandle;

/*!
 *  \anchor CANFD_MCANMsgObjectStats
 *  \name   CANFD Message Object Status
 * 
 *  \brief    Data structure defines the software maintained message object statistics.
 */
typedef struct CANFD_MCANMsgObjectStats_t
{
    /*! Message Object Handle for which the statistics is requested */
    CANFD_MsgObjHandle      handle;

    /*! Message Object direction */
    uint32_t                direction;

    /*! Starting range of the Message Id to which the configuration belongs.
     * For Tx and single Message Id objects the 
     * startMsgIdentifier = endMsgIdentifier 
     */
    uint32_t                startMsgIdentifier;

    /*! Ending range of the Message Id to which the configuration belongs
     *  For Tx and single Message Id objects the 
     *  startMsgIdentifier = endMsgIdentifier. 
     */
    uint32_t                endMsgIdentifier;

    /*! Number of interrupts received */
    uint32_t                interruptsRxed;

    /*! Number of messages processed */
    uint32_t                messageProcessed;
} CANFD_MCANMsgObjectStats;

/**
 *  \anchor CANFD_DmaConfig
 *  \name   CANFD Dma Config 
 * 
 * \brief CANFD DMA Configuration, these are filled by SysCfg based on the
 *        DMA driver(EDMA/UDMA) that is selected
 */



/**
 *  \brief  This function initializes each driver instance object and create a driver lock.
 */
void CANFD_init(void);

/**
 *  \brief  This function de-initializes each driver instance object and delete a driver lock.
 */
void CANFD_deinit(void);

/**
 *  \brief  This function opens a given CANFD peripheral
 *
 *  \pre    CANFD has been initialized using #CANFD_init()
 *
 *  \param  index       Index of config to use in the *CANFD_Config* array
 *  \param  openPrms    Pointer to open parameters. If NULL is passed, then
 *                      default values will be used
 *
 *  \return A #CANFD_Handle on success or a NULL on an error or if it has been
 *          opened already
 */
CANFD_Handle CANFD_open(uint32_t index, CANFD_OpenParams *openPrms);

/**
 *  \brief  Function to close a CANFD peripheral specified by the CANFD handle
 *
 *  \pre    #CANFD_open() has to be called first
 *
 *  \param  handle      #CANFD_Handle returned from #CANFD_open()
 *
 */
void CANFD_close(CANFD_Handle handle);

/**
 *  \brief
 *      Function configures the bit time parameters for the CANFD module.
 *
 *  \param  handle
 *      Handle to the CANFD Driver
 *  \param  bitTimeParams
 *      Bit time configuration parameters
 *
 *  \return SystemP_SUCCESS on success, else failure
 * 
 */
int32_t CANFD_configBitTime(CANFD_Handle handle, const CANFD_MCANBitTimingParams* bitTimeParams);

/**
 *  \brief
 *      Function configures the receive or transmit message object.
 *      It also enables Tx completion and Tx cancelation interrupts.
 *      The callback function will be invoked on data transmit complete
 *      for transmit message objects OR upon receiving data for receive
 *      message objects. The application MUST then call CANFD_read() API 
 *      to process the received data.
 *
 *  \param  handle
 *      Handle to the CANFD Driver
 *  \param  ptrCanMsgObj
 *      Pointer to Message Object configuration parameters
 *  \return
 *      Success -   Handle to the message object.
 *  \return
 *      Error   -   NULL
 */
int32_t CANFD_createMsgObject(CANFD_Handle handle, 
                              CANFD_MessageObject* ptrCanMsgObj);

/**
 *   \brief
 *      Function configures a receive message objects for a range of message identifiers.
 *      It also enables Rx interrupts.
 *      The callback function will be invoked upon receiving data for receive message objects.
 *      The application MUST then call CANFD_read() API to process the received data.
 *
 *  \param  handle
 *      Handle to the CANFD Driver
 *  \param  ptrCanMsgObj
 *      Pointer to Message Object configuration parameters
 *  \return
 *      Success -   Handle to the message object.
 *  \return
 *      Error   -   NULL
 */
int32_t CANFD_createRxRangeMsgObject(CANFD_Handle handle, 
                                     CANFD_MessageObject* ptrCanMsgObj);

/**
 *   \brief
 *      Function deletes a message object.
 *
 *  \param  handle
 *      Handle to the message object
 *
 *  \return SystemP_SUCCESS on success, else failure
 * 
 */
int32_t CANFD_deleteMsgObject(CANFD_MsgObjHandle handle);

/**
 *  \brief
 *      Function used by the application to transmit data.
 *
 *  \param  handle
 *      Handle to the message object
 *  \param  id
 *      Message Identifier
 *  \param  frameType
 *      Frame type - Classic or FD
 *  \param  numMsgs
 *      Number of msgs to be transmitted Only applicale in DMA Mode.
 *  \param  data
 *      Data to be transmitted
 *
 *  \return SystemP_SUCCESS on success, else failure
 * 
 */
int32_t CANFD_write(CANFD_MsgObjHandle handle, uint32_t id, 
                    CANFD_MCANFrameType frameType, uint32_t numMsgs, 
                    const uint8_t* data);

/**
 *   \brief
 *      Function used by the application to cancel a pending data transmit.
 *
 *  \param  handle
 *      Handle to the message object
 *
 *  \return SystemP_SUCCESS on success, else failure
 * 
 */
int32_t CANFD_writeCancel(CANFD_MsgObjHandle handle);

/**
 *   \brief
 *      Function used by the application to initiate transmit data in dma mode.
 *      DMA mode is recommended to be used when multiple msgs needs to be transmitted.
 *      This will transmit first msg and configure the dma to copy subsequent msgs in message ram.
 *      Once first msg transfer is completed the CANFD_dmaTxCompletionCallback function is called.
 *      Application needs to call the API CANFD_writeDmaTriggerNext to trigger transmission
 *      of subsequent msgs.
 *
 *  \param  handle
 *      Handle to the message object
 *  \param  id
 *      Message Identifier
 *  \param  frameType
 *      Frame type - Classic or FD
 *  \param  numMsgs
 *      Number of msgs to be transmitted
 *  \param  data
 *      Data to be transmitted. This should be a 2d array of uint8[numMsgs][dataLengthPerMsg]
 *
 *  \return SystemP_SUCCESS on success, else failure 
 * 
 */
int32_t CANFD_writeDma(CANFD_MsgObjHandle handle, uint32_t id, 
                       CANFD_MCANFrameType frameType, 
                       uint32_t numMsgs, const void* data);

/**
 *   \brief
 *      Function used by the application to start transmission of next msg in dma mode.
 *      Transfer should be initiated using the API CANFD_writeDma before calling this function.
 *      This should be called after the previous transfer is completed and CANFD_dmaTxCompletionCallback
 *      is called.
 *
 *  \param  handle
 *      Handle to the message object
 *
 *  \return SystemP_SUCCESS on success, else failure
 * 
 */
int32_t CANFD_writeDmaTriggerNext(CANFD_MsgObjHandle handle);

uint32_t CANFD_getFilterEventConfig(uint32_t eventNum);

/**
 *   \brief
 *      Function is used by the application to get the CAN message from message
 *      RAM using a receive message object.
 *      NOTE: This API must ONLY be called from the callback context.
 *      Function is used by the application to get the CAN message from message RAM using a receive message object.
 *
 *  \param  rxMsgHandle
 *      Handle to the message object
 *  \param  numMsgs
 *      Number of message count. Applicale only for DMA Mode, for other modes, provide 0.
 *  \param  data
 *      Pointer to the receiver's buffer, where read data will be stored.
 *  \return SystemP_SUCCESS on success, else failure
 * 
 */
int32_t CANFD_read(CANFD_MsgObjHandle rxMsgHandle, uint32_t numMsgs, uint8_t* data);

/**
 *   \brief
 *      Function is used by the application to configure reading the received msgs from message ram.
 *      This API will configure the DMA to copy msg from MCAN message ram to application buffer.
 *      For every message received the CANFD_dmaRxCompletionCallback will be called by driver by pointing
 *      to the latest msg in the data buffer.
 *
 *  \param  handle
 *      Handle to the message object
 *  \param  data
 *      Array of CANFD_DmaRxBuf to receive the data from message ram. This should be an array
 *      of type CANFD_DmaRxBuf and length numDmaRxBuf
 *  \param numMsgs        
 *       Number of messages to transmit
 *  \return SystemP_SUCCESS on success, else failure
 * 
 */
int32_t CANFD_readDmaConfig(CANFD_MsgObjHandle handle, const void* data, uint32_t numMsgs);

/**
 *   \brief
 *      Function is used by the application to get the error and status information from the driver.
 *
 *  \param  handle
 *      Handle to the CANFD Driver
 *  \param ptrOptInfo
 *      Option info in TLV format which is populated with the requested information
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t CANFD_getOptions(CANFD_Handle handle, const CANFD_OptionTLV* ptrOptInfo);

/**
 *   \brief Function is used by the application to configure the driver options.
 *
 *  \param  handle
 *      Handle to the CANFD Driver
 *  \param ptrOptInfo
 *      Option info in TLV format which is used to configure the driver
 * 
 *  \return SystemP_SUCCESS on success, else failure
 */
int32_t CANFD_setOptions(CANFD_Handle handle, const CANFD_OptionTLV* ptrOptInfo);

/**
 *   \brief  The function is the registered interrupt 0 ISR for the CANFD Driver.
 */
void CANFD_int0Isr (void *args);

/**
 *  \brief The function is the registered interrupt 1 ISR for the CANFD Driver.
 */
void CANFD_int1Isr (void *args);

/**
 * \brief
 * Application specified callback function which is invoked
 * by the CANFD driver once transmit is complete or data has been received for the
 * specified message object.
 *
 *  \param  args
 *      void pointer basically Message object handle for which the 
 *      callback function is invoked.
 *  \param  reason
 *      Cause of the interrupt which prompted the callback.
 */
void CANFD_transferCallBack(void *args, CANFD_Reason reason);

/**
 * \brief
 * Application specified callback function which is invoked
 * by the CANFD driver on error or status change.
 *
 *  \param  handle
 *      Handle to the CANFD Driver
 *  \param  reason
 *      Cause of the interrupt which prompted the callback.
 *  \param  errStatusResp
 *      Response structure populated with the value of the fields that caused
 *      the error or status interrupt. Processing of this structure is dependent
 *      on the callback reason.
 */
void CANFD_errStatusCallBack(CANFD_Handle handle, CANFD_Reason reason, 
                             CANFD_ErrStatusResp* errStatusResp);

/**
 * \brief API to open an CANFD DMA channel
 *
 *  This API will open a DMA Channel using the appropriate DMA driver callbacks and
 *  the registered via Sysconfig
 *
 * \param canfdHandle    CANFD Handle
 * \param dmaChCfg       CANFD Channel config
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CANFD_dmaOpen(CANFD_Handle canfdHandle, CANFD_DmaChConfig dmaChCfg);

/**
 * \brief API to close an CANFD DMA channel
 *
 * \param canfdHandle   CANFD handle returned from \ref CANFD_open
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CANFD_dmaClose(CANFD_Handle canfdHandle);

/**
 * \brief API to configure dma for the Tx message object. Called from the 
 *        API CANFD_createMsgObject.
 *
 * \param ptrCanFdObj   pointer to the CANFD driver object
 * \param ptrCanMsgObj  pointer to the CANFD message object
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CANFD_createDmaTxMsgObject(const CANFD_Object *ptrCanFdObj, 
                                   CANFD_MessageObject* ptrCanMsgObj);

/**
 * \brief API to delete dma configuration for the Tx message object. Called 
 *        from the API CANFD_deleteMsgObject.
 *
 * \param ptrCanFdObj   pointer to the CANFD driver object
 * \param ptrCanMsgObj  pointer to the CANFD message object
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CANFD_deleteDmaTxMsgObject(const CANFD_Object *ptrCanFdObj, 
                            const CANFD_MessageObject* ptrCanMsgObj);

/**
 * \brief API to enable dma event transfer for the Tx. Called from the API CANFD_writeDma.
 *
 * \param ptrCanFdObj       pointer to the CANFD driver object
 * \param ptrCanMsgObj      pointer to the CANFD message object
 * \param dataLengthPerMsg  data length in bytes for each message
 * \param numMsgs           Number f messages to transmit
 * \param data              pointer to the data buffer to transmit
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CANFD_configureDmaTx(const CANFD_Object *ptrCanFdObj, 
                    CANFD_MessageObject* ptrCanMsgObj, uint32_t dataLengthPerMsg, 
                    uint32_t numMsgs, const void* data);

/**
 * \brief API to disbale dma event transfer for the Tx to cancel the transfer.
 *
 * \param ptrCanFdObj       pointer to the CANFD driver object
 * \param ptrCanMsgObj      pointer to the CANFD message object
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CANFD_cancelDmaTx(const CANFD_Object *ptrCanFdObj, 
                const CANFD_MessageObject* ptrCanMsgObj);

/**
 * \brief Callback function for the Tx completion.
 *        This is called for each message in the array of msgs provided in 
 *        CANFD_configureDmaTx. Data will point to the current transmitted message.
 *        For intermediate message transfer completion the completionType will be 
 *        set to CANFD_DMA_TX_COMPLETION_INTERMEDIATE for last message transfer 
 *        completion the completionType will be set to CANFD_DMA_TX_COMPLETION_FINAL
 *
 * \param ptrCanMsgObj    pointer to the CANFD message object
 * \param data            pointer to current completed message
 * \param completionType  specifie completion type for the callback
 *
 */
void CANFD_dmaTxCompletionCallback(CANFD_MessageObject* ptrCanMsgObj, 
                const void *data, uint32_t completionType);

/**
 * \brief API to configure dma for the Rx message object. Called from
 *        the CANFD_createMsgObject.
 *
 * \param ptrCanFdObj   pointer to the CANFD driver object
 * \param ptrCanMsgObj  pointer to the CANFD message object
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CANFD_createDmaRxMsgObject(const CANFD_Object *ptrCanFdObj, 
                CANFD_MessageObject* ptrCanMsgObj);

/**
 * \brief API to delete dma configuration for the Rx message object. Called 
 *        from the CANFD_deleteMsgObject.
 *
 * \param ptrCanFdObj   pointer to the CANFD driver object
 * \param ptrCanMsgObj  pointer to the CANFD message object
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CANFD_deleteDmaRxMsgObject(const CANFD_Object *ptrCanFdObj, 
                                   const CANFD_MessageObject* ptrCanMsgObj);

/**
 * \brief API to enable dma event transfer for the Rx. Called from the API CANFD_readDma.
 *
 * \param ptrCanFdObj       pointer to the CANFD driver object
 * \param ptrCanMsgObj      pointer to the CANFD message object
 * \param dataLengthPerMsg  data length in bytes for each message
 * \param numMsgs           Number f messages to transmit
 * \param data              pointer to the data buffer to transmit
 *
 */
int32_t CANFD_configureDmaRx(const CANFD_Object *ptrCanFdObj, 
                             CANFD_MessageObject* ptrCanMsgObj, 
                             uint32_t dataLengthPerMsg, uint32_t numMsgs, 
                             const void* data);

/**
 * \brief Callback function for the Rx completion.
 *        This is called for each message in the array of msgs provided in 
 *        CANFD_configureDmaRx. Data will point to the current received message.
 *        For intermediate message transfer completion the completionType will be 
 *        set to CANFD_DMA_RX_COMPLETION_INTERMEDIATE for last message transfer 
 *        completion the completionType will be set to CANFD_DMA_RX_COMPLETION_FINAL
 *
 * \param ptrCanMsgObj    pointer to the CANFD message object
 * \param data            pointer to current recieved message
 * \param completionType  specifie completion type for the callback
 *
 */
void CANFD_dmaRxCompletionCallback(CANFD_MessageObject* ptrCanMsgObj, const void *data, 
                                   uint32_t completionType);


/**
 * \brief Function to validate the data Size.
 *
 * \param dataSize  CANFD data Size
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t CANFD_isDataSizeValid(uint32_t dataSize);

/* ========================================================================== */
/*                          Advance Functions                                 */
/* ========================================================================== */

/**
 * \brief   This function return endianness value of MCAN module.
 *
 * \param   canfdHandle   #CANFD_Handle returned from #CANFD_open()
 *
 * \retval  val           Endianness value. (0x87654321)
 */
int32_t CANFD_getEndianVal(CANFD_Handle canfdHandle);

/**
 * \brief   This API is used get the MCAN revision ID.
 *
 * \param   canfdHandle   #CANFD_Handle returned from #CANFD_open()
 * 
 */
void CANFD_getRevisionId(CANFD_Handle canfdHandle);

/**
 * \brief   This API add clock stop request for MCAN module to put it in
 *          power down mode.
 *
 * \param   canfdHandle     #CANFD_Handle returned from #CANFD_open()
 * \param   enable          Add CLock Stop Request.
 *                          Adds Clock Clock stop Request is TRUE otherwise
 *                          removes it.
 */
void CANFD_addClockStopRequest(CANFD_Handle canfdHandle, uint32_t enable);

/**
 * \brief   This API get clock stop acknowledgement for MCAN module.
 *          It return whether MCAN is power down mode or not.
 *
 * \param   canfdHandle     #CANFD_Handle returned from #CANFD_open()
 * \return  status          Return Clock Stop Acknowledgement status.
 *                          Return '1' if M_CAN is set in power down mode else
 *                          returns '0'.
 */
uint32_t CANFD_clockStopReq(CANFD_Handle canfdHandle);

/**
 * \brief   This API will return clock stop acknowledgement
 *          for MCAN module.
 *
 * \param   canfdHandle     #CANFD_Handle returned from #CANFD_open()
 * \return  ack             Clock Stop Acknowledge
 *                          0= No clock stop acknowledged
 *                          1= M_CAN may be set in power down
 */
uint32_t CANFD_getClkStopAck(CANFD_Handle canfdHandle);

/**
 * \brief   This API will return Rx pin state of MCAN module.
 * 
 * \param   canfdHandle     #CANFD_Handle returned from #CANFD_open()
 * \return  state           MCAN Rx Pin State.
 *                          0= The CAN bus is dominant
 *                          1= The CAN bus is recessive
 */
uint32_t CANFD_getRxPinState(CANFD_Handle canfdHandle);

/**
 * \brief   This API will set Tx pin state of MCAN module.
 *
 * \param   canfdHandle     #CANFD_Handle returned from #CANFD_open()
 * \param   state           MCAN Tx Pin State.
 *                          00= Reset value
 *                          01= Sample Point can be monitored at tx pin
 *                          10= The CAN bus is dominant
 *                          11= The CAN bus is recessive
 *                          other= It will treated as 11.
 */
void CANFD_setTxPinState(CANFD_Handle canfdHandle, uint32_t state);

/**
 * \brief   This API will return Tx pin state of MCAN module.
 *
 * \param   canfdHandle     #CANFD_Handle returned from #CANFD_open()
 * \return  state           MCAN Tx Pin State.
 *                          00= Reset value
 *                          01= Sample Point can be monitored at tx pin
 *                          10= The CAN bus is dominant
 *                          11= The CAN bus is recessive
 */
uint32_t CANFD_getTxPinState(CANFD_Handle canfdHandle);

/**
 * \brief   This API will get the configured bit timings for MCAN module.
 *
 * \param   canfdHandle     #CANFD_Handle returned from #CANFD_open()
 */
void CANFD_getBitTime(CANFD_Handle canfdHandle);

/**
 * \brief   This API will return current timestamp counter value.
 *
 * \param   canfdHandle    #CANFD_Handle returned from #CANFD_open()
 *
 * \return  val            Current Timestamp counter value.
 */
uint32_t CANFD_getTSCounterVal(CANFD_Handle canfdHandle);

/**
 * \brief   This API will reset timestamp counter value.
 *
 * \param   canfdHandle    #CANFD_Handle returned from #CANFD_open()
 *
 */
void CANFD_resetTSCounter(CANFD_Handle canfdHandle);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef CANFD__H_ */

/** @} */
