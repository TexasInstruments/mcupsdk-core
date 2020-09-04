/*
 * Copyright (C) 2021 Texas Instruments Incorporated
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
 *  \defgroup DRV_CBUFF_MODULE APIs for CBUFF
 *  \ingroup DRV_MODULE
 *
 *  The CBUFF (Common Buffer Controller) is responsible for the transfer
 *  of data from multiple sources like ADCBUFF, Chirp Parameters (CP),
 *  Chirp Quality (CQ) or any other source to the LVDS Tx Module.
 *  The application initializes the CBUFF driver by calling CBUFF_init()
 *  and then it is ready to open CBUFF instance with the Session
 *  configuration (either HW/SW) provided by application.
 *
 *  The default driver library fully supports the following data modes:-
 *    - Interleaved
 *    - Non-Interleaved
 *
 *  @{
 */

/**
 *  \file v0/cbuff.h
 *
 *  \brief This file contains the prototypes of the APIs present in the
 *         device abstraction layer file of CBUFF.
 *         This also contains some related macros.
 */


#ifndef CBUFF_V0_H_
#define CBUFF_V0_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdbool.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/hw_include/cslr_cbuff.h>
#include <drivers/edma.h>
#include <drivers/adcbuf.h>
#include <drivers/hw_include/hw_types.h>


/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/** \brief CBUFF driver error base */
#define CBUFF_ERRNO_BASE                    (-3300)

/**
 * \brief   Maximum number of user supplied data buffers which can be configured
 * and sent over the interface
 */
#define CBUFF_MAX_USER_BUFFER               (3U)


/**
 *  \defgroup CBUFF_ERROR_CODES Error Codes
 *  \ingroup DRV_CBUFF_MODULE
 *
 * @{
 */

/*! \brief    No Error. */
#define CBUFF_STATUS_SUCCESS            ((int32_t)0)
/**
 * \brief   Error Code: Invalid argument
 */
#define CBUFF_EINVAL                    (CBUFF_ERRNO_BASE-1)

/**
 * \brief   Error Code: Out of memory
 */
#define CBUFF_ENOMEM                    (CBUFF_ERRNO_BASE-2)

/**
 * \brief   Error Code: Operation cannot be implemented because the CBUFF driver
 * is in use
 */
#define CBUFF_EINUSE                    (CBUFF_ERRNO_BASE-3)

/**
 * \brief   Error Code: Operation cannot be implemented because the CBUFF driver
 * is not supporting this currently
 */
#define CBUFF_ENOTSUP                   (CBUFF_ERRNO_BASE-4)

/**
 * \brief   Error Code: The application failed to provide the EDMA resources
 * which are required by the CBUFF driver to support the application use case
 */
#define CBUFF_EDMA_FAIL                 (CBUFF_ERRNO_BASE-5)

/**
 * \brief   Error Code: Limit exceeded
 */
#define CBUFF_ELIMIT                    (CBUFF_ERRNO_BASE-6)

/** @} */ /* end defgroup CBUFF_ERROR_CODES */

/**
 * \brief   Maximum number of sessions which can be supported
 * by the CBUFF Driver.
 *
 * *NOTE*: This is limited to the number of DMA channels available
 * to the CBUFF IP.
 */
#define CBUFF_MAX_NUM_SESSION                       (7U)

/**
 * \brief   Maximum number of EDMA channels which can be used by the
 * CBUFF driver.
 */
#define CBUFF_EDMA_MAX_NUM_CHANNELS                 (32U)

/**
 * \brief   Maximum number of linked list entries which can be tracked
 * by the driver.
 *
 * *NOTE*: This is limited to the number Linked list entries available
 * to the CBUFF IP.
 */
#define CBUFF_MAX_LINKED_LIST_SIZE                  (32U)

/**
 * \brief   Read threshold configured in the Linked List Entries. This is
 * used by the CBUFF 128bit FIFO
 *
 * *NOTE*: This is the recommended value as per the Application Note
 */
#define CBUFF_LL_READ_THRESHOLD                     (0x4U)

/**
 * \brief   Write threshold configured in the Linked List Entries. This is
 * used by the CBUFF 128bit FIFO
 *
 * *NOTE*: This is the recommended value as per the Application Note
 */
#define CBUFF_LL_WRITE_THRESHOLD                    (0x40U)

/**
 * \brief   Maximum transfer size in CBUFF Units
 *
 * *NOTE*: This is the maximum size of the transfer as per the CBUFF IP
 */
#define CBUFF_MAX_TRANSFER_SIZE_CBUFF_UNITS         (0x3FFFU)

/**
 * \brief   Minimum transfer size in CBUFF Units
 *
 * *NOTE*: This is the minimum frame size which can be transferred by the
 * CBUFF IP. The Frame size is defined to the sum of the transfer size of
 * all the linked list entries
 *
 * The CBUFF FIFO Width for Read Threshold of 4 will imply that 4*128bits
 * i.e. 64 bytes or 32CBUFF Units
 */
#define CBUFF_MIN_TRANSFER_SIZE_CBUFF_UNITS         ((CBUFF_LL_READ_THRESHOLD * 16U) / 2U)

/* ========================================================================== */
/*                             typedefs                                       */
/* ========================================================================== */

/*!
 *  \brief      A handle that is returned from a CBUFF_open() call.
 */
typedef void*   CBUFF_Handle;

/*!
 *  \brief      A handle that is returned from a CBUFF Session.
 */
typedef void*   CBUFF_SessionHandle;


/**
 * \brief
 *  High Speed Interface
 *
 * \details
 *  Describes the high speed interface which is to be used
 *  by the CBUFF to send out the data
 */
typedef uint32_t CBUFF_Interface;

#define CBUFF_Interface_MDO       ((uint32_t) 1) /*! The CBUFF driver will send out the packets using the MDO/Aurora Interface */
#define CBUFF_Interface_LVDS      ((uint32_t) 2) /*! The CBUFF driver will send out the packets using the LVDS Interface */


/**
 * \brief
 *  CBUFF Command
 *
 * \details
 *  Describes commands which are used to get/set information from the CBUFF Driver
 *
 *  CBUFF_control
 */
typedef uint32_t CBUFF_Command;

#define CBUFF_Command_GET_CBUFF_STATS       ((uint32_t) 1) /*! get the statistics associated with the CBUFF driver */
#define CBUFF_Command_CLEAR_CBUFF_STATS     ((uint32_t) 2) /*! clear the statistics associated with the CBUFF driver */
#define CBUFF_Command_GET_ACTIVE_SESSION    ((uint32_t) 3) /*! Get the current active session in the CBUFF */


/**
 * \brief
 *  Output Data Format
 *
 * \details
 *   Describes the LVDS output data format.
 */
typedef uint32_t CBUFF_OutputDataFmt;

#define CBUFF_OutputDataFmt_12bit     ((uint32_t) 0) /*! 12bit output format */
#define CBUFF_OutputDataFmt_14bit     ((uint32_t) 1) /*! 14bit output format */
#define CBUFF_OutputDataFmt_16bit     ((uint32_t) 2) /*! 16bit output format */


/**
 * \brief
 *  Data Format
 *
 * \details
 *  Describes the data format.
 */
typedef uint32_t CBUFF_DataFmt;

#define CBUFF_DataFmt_ADC_DATA           ((uint32_t) 0) /*! Only ADC Data is to sent out */
#define CBUFF_DataFmt_CP_ADC             ((uint32_t) 1) /*! Chirp Parameters + ADC Data */
#define CBUFF_DataFmt_ADC_CP             ((uint32_t) 2) /*! ADC Data + Chirp Parameters */
#define CBUFF_DataFmt_CP_ADC_CQ          ((uint32_t) 3) /*! Chirp Parameters + ADC Data + Chirp Quality */
#define CBUFF_DataFmt_ADC_USER           ((uint32_t) 4) /*! ADC + User Data */
#define CBUFF_DataFmt_CP_ADC_CQ_USER     ((uint32_t) 5) /*! Chirp Parameters + ADC Data + Chirp Quality + User Data */
#define CBUFF_DataFmt_MAX                ((uint32_t) 6) /*! Maximum Data format value. */


/**
 * \brief
 *  Data Type
 *
 * \details
 *   Describes the data type
 */
typedef uint32_t CBUFF_DataType;

#define CBUFF_DataType_REAL                   ((uint32_t) 0) /*! Real Data */
#define CBUFF_DataType_COMPLEX                ((uint32_t) 1) /*! Complex Data */


/**
 * \brief
 *  CBUFF Operational Mode
 *
 * \details
 *  Describes the operational mode for the CBUFF
 */
typedef uint32_t CBUFF_OperationalMode;

#define CBUFF_OperationalMode_CHIRP                   ((uint32_t) 1) /*! The CBUFF sends out the data on the High speed interface based on the number of chirps. */
#define CBUFF_OperationalMode_CONTINUOUS              ((uint32_t) 2) /*! The CBUFF sends out the data on the High speed interface based on the threshold. This is only supported on LVDS. */

/**
 * \brief
 *  LVDS Lane Format Map
 *
 * \details
 *  Describes the LVDS Lane Format maps which are available
 *  and which need to be used.
 */
typedef uint32_t CBUFF_LVDSLaneFmtMap;

#define CBUFF_LVDSLaneFmtMapLANEx_FMT_0_y             ((uint32_t) 0) /*! Use the LVDS Lane Format-0. */
#define CBUFF_LVDSLaneFmtMapLANEx_FMT_1_y             ((uint32_t) 1) /*!  Use the LVDS Lane Format-1. */


/**
 * \brief
 *  Data Storage Mode
 *
 * \details
 *  Describes the storage mode in which the data is stored
 *  in the ADC Buffer.
 */
typedef uint32_t CBUFF_DataMode;

#define CBUFF_DataMode_INTERLEAVED             ((uint32_t) 0) /*! Data is stored in interleaved mode. */
#define CBUFF_DataMode_NON_INTERLEAVED         ((uint32_t) 1) /*! Data is stored in non-interleaved mode. This implies that all the samples of a receive channel are grouped together. */


/**
 * \brief
 *  Session Execution Mode
 *
 * \details
 *  Describes the execution mode for the sessions.
 *  Sessions can be automatically triggered by the hardware *or* these
 *  need to be triggered manually by the CBUFF Driver.
 */
typedef uint32_t CBUFF_SessionExecuteMode;

#define CBUFF_SessionExecuteMode_HW             ((uint32_t) 0) /*! CBUFF Transfers are triggered by the hardware. */
#define CBUFF_SessionExecuteMode_SW             ((uint32_t) 1) /*! CBUFF Transfers are triggered by the driver. */


/*!
 *  \brief  CBUFF Linked List Type
 *
 *  Each linked list in the CBUFF holds information about one of
 *  the following types
 */
typedef uint32_t CBUFF_LLType;

/*!
 *  \brief  CBUFF_LLType type of data transfer.
 */
#define CBUFF_LLType_UNUSED       ((uint32_t) 0) /*! CBUFF Linked List Entry is not being used */
#define CBUFF_LLType_ADCDATA      ((uint32_t) 1) /*! CBUFF Linked List Entry holds ADC Data */
#define CBUFF_LLType_CQ           ((uint32_t) 2) /*! CBUFF Linked List Entry holds Chirp Quality (CQ) */
#define CBUFF_LLType_CP           ((uint32_t) 3) /*! CBUFF Linked List Entry holds Chirp Parameters (CP) */
#define CBUFF_LLType_USER         ((uint32_t) 4) /*! CBUFF Linked List Entry holds a user buffer */
#define CBUFF_LLType_HEADER       ((uint32_t) 5) /*! CBUFF Linked List Entry holds a header */


/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/**
 * \brief
 *  CBUFF Statistics
 *
 * \details
 *  The structure describes the CBUFF statistics which can be used
 *  to determine the behavior of the CBUFF module.
 */
typedef struct CBUFF_Stats_t
{
    /**
     * \brief   Number of frame done interrupts received: This is available
     * only in the following cases:
     *  (a) Enable debug mode
     *  (b) Sessions with the Frame done callback
     * If either of the above cases is not met the value of this is always 0.
     *
     * @sa CBUFF_InitCfg_t::enableDebugMode
     * @sa CBUFF_SessionCfg_t::frameDoneCallbackFxn
     */
    uint32_t    numFrameDone;

    /**
     * \brief   Number of chirp done interrupts received:  This is available
     * only in the following cases:
     *  (a) Enable debug mode
     *  (b) Sessions with the Frame done callback
     * If either of the above cases is not met the value of this is always 0.
     *
     * @sa CBUFF_InitCfg_t::enableDebugMode
     * @sa CBUFF_SessionCfg_t::frameDoneCallbackFxn
     */
    uint32_t    numChirpDone;

    /**
     * \brief   Number of error interrupts received. This is always available
     */
    uint32_t    numErrorInterrupts;

    /**
     * \brief   Flag which indicated if a frame start error was detected.
     * This is always available
     */
    uint8_t     frameStartError;

    /**
     * \brief   Flag which indicated if a chirp error was detected.
     * This is always available
     */
    uint8_t     chirpError;
}CBUFF_Stats;

/**
 * \brief
 *  CBUFF Buffer configuration
 *
 * \details
 *  This is a generic data structure which is exposed to the application
 *  to pass buffer configuration to the drivers. This is used to populate
 *  the User Buffers and Headers which can be streamed out via CBUFF to
 *  the selected High speed interface.
 */
typedef struct CBUFF_BufferCfg_t
{
    /**
     * \brief   Size of the Buffer: This can be set to 0 to indicate
     * that no buffer is specified. The size is specified in CBUFF units
     * i.e. 16 bits.
     */
    uint16_t    size;

    /**
     * \brief   Address of the Buffer:
     * Please be aware that the address should be in a memory range
     * which is accessible by the EDMA.
     */
    uint32_t    address;
}CBUFF_BufferCfg;

/**
 * \brief
 *  CBUFF Linked List parameters
 *
 * \details
 *  The structure contains the parameters which are used to program and setup
 *  the CBUFF linked list entries.
 */
typedef struct CBUFF_LinkListParams_t
{
    /**
     * \brief   Type of data being stored in the linked list entry
     */
    CBUFF_LLType    type;

    /**
     * \brief   CRC Enable: Only ADC Buffers have CRC and these need to be
     * verified
     */
    uint8_t         crcEnable;

    /**
     * \brief   Long packet header enable: This is to be 1 for the first
     * entry in the group.
     */
    uint8_t         lpHdrEn;

    /**
     * \brief   Output data format:
     *  00 - 16bit
     *  01 - 14bit
     *  10 - 12bit
     */
    uint8_t         outputDataFmt;

    /**
     * \brief   Transfer size
     */
    uint16_t        transferSize;

    /**
     * \brief   Cumulative Transfer Size for all linked entries.
     * This is applicable only if the HSI is CSI
     */
    uint16_t        totalTransferSize;

    /**
     * \brief   96 byte transfer mode
     */
    uint8_t         align96;

    /**
     * \brief   Data Format mapping
     */
    uint8_t         dataFmtMap;

    /**
     * \brief   Virtual channel
     */
    uint8_t         vcNum;

    /**
     * \brief   Horizontal sync start
     */
    uint8_t         hsyncStart;

    /**
     * \brief   Horizontal sync end
     */
    uint8_t         hsyncEnd;

    /**
     * \brief   Long Packet Header Value
     */
    uint32_t        lpHeaderValue;

    /**
     * \brief   Thresholds to be configured
     */
    uint32_t        threshold;
}CBUFF_LinkListParams;

/** \brief CBUFF instance attributes - used during init time */
typedef struct
{
    /*
     * SOC configuration
     */
    uint32_t            baseAddr;
    /**< Peripheral base address */
    uint32_t            fifoBaseAddr;
    /**< CBUFF FIFO base address */
    uint32_t            adcBufBaseAddr;
    /**< ADCBUF base address */
    uint32_t            maxLVDSLanesSupported;
    /**< Max LVDS Lines supported */
    uint32_t            errorIntrNum;
    /**< Error Interrupt Number */
    uint32_t            intrNum;
    /**< Interrupt Number */
    uint32_t            chirpModeStartIndex;
    /**< Chirp mode start Index */
    uint32_t            chirpModeEndIndex;
    /**< Chirp mode End Index */
    uint32_t            cpSingleChirpInterleavedAddr[SOC_ADCBUF_NUM_RX_CHANNEL];
    /**< Single Chirp Interleaved Base address of the Chirp Parameter: */
    uint32_t            cpSingleChirpNonInterleavedAddr[SOC_ADCBUF_NUM_RX_CHANNEL];
    /**< Single Chirp Non-Interleaved Base address of the Chirp Parameter: */
    uint32_t            cpMultipleChirpNonInterleavedAddr[8];
    /**< Multiple Chirp Non-Interleaved Base address of the Chirp Parameter: */
    uint32_t            cbuffChannelId[CBUFF_MAX_NUM_SESSION];
    /**< CBUFF EDMA Physical Channel Identifier: */
} CBUFF_Attrs;

/* ========================================================================== */
/*                         Structures                                         */
/* ========================================================================== */

/**
 * \brief
 *  CBUFF LVDS Initialization configuration
 *
 * \details
 *  The structure describes the configuration which is required to configure
 *  the LVDS
 */
typedef struct CBUFF_LVDSCfg_t
{
    /**
     * \brief   Enable/Disable CRC on LVDS
     */
    uint8_t                 crcEnable;

    /**
     * \brief   LVDS Lane configuration: The bit mask here is used to indicate
     * the active LVDS lanes i.e. Bit 0 implies Lane-0, Bit 1 implies Lanel-1 etc
     *
     * The number of LVDS lanes is platform specific and is defined in the CBUFF
     * platform file.
     */
    uint8_t                 lvdsLaneEnable;

    /**
     * \brief   Set the flag to 1 to indicate that the MSB is sent first or LSB
     */
    uint8_t                 msbFirst;

    /**
     * \brief   Set the flag to 1 for DDR Clock Mode and 0 for SDR
     */
    uint8_t                 ddrClockMode;

    /**
     * \brief   Set the flag to 1 for DDR Mode Clock Mux and 0 for SDR Mode Clock Mux
     */
    uint8_t                 ddrClockModeMux;

    /**
     * \brief   LVDS Lane Format:
     */
    CBUFF_LVDSLaneFmtMap    laneFormat;
}CBUFF_LVDSCfg;

/**
 * \brief
 *  CBUFF EDMA channel resource configuration
 *
 * \details
 *  The structure describes the EDMA channel resources which are needed by the
 *  CBUFF driver in order to stream out the data over the high speed interface.
 */
typedef struct CBUFF_EDMAChannelCfg_t
{
    /**
     * \brief   EDMA Chain Channels Identifier
     */
    uint32_t     chainChannelsId;

    /**
     * \brief   EDMA Shadow link channels Identifier
     */
    uint32_t    shadowLinkChannelsId;
}CBUFF_EDMAChannelCfg;

/**
 * \brief
 *  CBUFF EDMA Information block
 *
 * \details
 *  The structure describes the EDMA informational block which is passed
 *  to the application. The information here requires to be used by the
 *  application before performing an EDMA channel allocation.
 */
typedef struct CBUFF_EDMAInfo_t
{
    /**
     * \brief   EDMA Instance handle: This is the configuration which was passed
     * by the application. Ensure that the EDMA channels are allocated from the
     * the specified EDMA instance.
     */
    EDMA_Handle     edmaHandle;

    /**
     * \brief   Flag which if set indicates that this is the first EDMA channel which
     * is being allocated. The first EDMA channel allocation is a special case explained
     * below.
     */
    bool            isFirstEDMAChannel;

    /**
     * \brief   DMA Number: There are multiple DMA in the CBUFF driver. There exists
     * a mapping between the CBUFF DMA number and the corresponding hardwired EDMA channel.
     *
     * Please be aware that the CBUFF EDMA transfers are kicked in automatically by the
     * hardware. The AWR294x has a special CBUFF EDMA Physical channel which has been allocated
     * for this purpose and which should not be used for any other reason. Each DMA Number is
     * associated with a corresponding EDMA channel.
     *
     *  DMA Number   | EDMA Physical Channel(s)
     *  -------------|-----------------------
     *    18         | EDMA_DSS_TPCC_A_EVT_CBUFF_DMA_REQ0, EDMA_DSS_TPCC_B_EVT_CBUFF_DMA_REQ0, EDMA_DSS_TPCC_C_EVT_CBUFF_DMA_REQ0
     *    19         | EDMA_DSS_TPCC_A_EVT_CBUFF_DMA_REQ1, EDMA_DSS_TPCC_B_EVT_CBUFF_DMA_REQ1, EDMA_DSS_TPCC_C_EVT_CBUFF_DMA_REQ1
     *    20         | EDMA_DSS_TPCC_A_EVT_CBUFF_DMA_REQ2, EDMA_DSS_TPCC_B_EVT_CBUFF_DMA_REQ2, EDMA_DSS_TPCC_C_EVT_CBUFF_DMA_REQ2
     *    21         | EDMA_DSS_TPCC_A_EVT_CBUFF_DMA_REQ3, EDMA_DSS_TPCC_B_EVT_CBUFF_DMA_REQ3, EDMA_DSS_TPCC_C_EVT_CBUFF_DMA_REQ3
     *    22         | EDMA_DSS_TPCC_A_EVT_CBUFF_DMA_REQ4, EDMA_DSS_TPCC_B_EVT_CBUFF_DMA_REQ4, EDMA_DSS_TPCC_C_EVT_CBUFF_DMA_REQ4
     *    23         | EDMA_DSS_TPCC_A_EVT_CBUFF_DMA_REQ5, EDMA_DSS_TPCC_B_EVT_CBUFF_DMA_REQ5, EDMA_DSS_TPCC_C_EVT_CBUFF_DMA_REQ5
     *    24         | EDMA_DSS_TPCC_A_EVT_CBUFF_DMA_REQ6, EDMA_DSS_TPCC_B_EVT_CBUFF_DMA_REQ6, EDMA_DSS_TPCC_C_EVT_CBUFF_DMA_REQ6
     *
     *
     *  Please ensure that the *first* EDMA channel for the specified DMA number has to be
     *  from the table above. There is no restriction on subsequent EDMA channel allocations.
     *  This table needs to be enforced if the 'isFirstEDMAChannel' is set to true.
     *
     *  *NOTE*: The CBUFF driver will fail the session creation if the above table is not
     *  enforced.
     */
    uint32_t         dmaNum;
}CBUFF_EDMAInfo;

/**
*  @b Description
*  @n
*      This is the function which is registered with the CBUFF driver and is invoked
*      by the driver whenever it needs to allocate and use an EDMA channel.
*
*  @param[in]  ptrEDMAInfo
*      Pointer to the EDMA informational block which needs to be used by
*      the application to perform the EDMA channel allocation
*  @param[out] ptrEDMAChannelCfg
*      Pointer to the EDMA Channel configuration which is to be populated
*
*  @retval
*      Success -   0
*  @retval
*      Error   -   <0 [Implies that the application is unable to allocate a channel]
*
* @b NOTE:
*  The EDMA transfer completion codes are identical to the CBUFF Physical & chain channel
*  resources therefore these transfer completion codes are considered to be reserved
*  for CBUFF Usage.
*/
typedef int32_t (*CBUFF_EDMAChannelAllocateFxn) (CBUFF_EDMAInfo* ptrEDMAInfo, CBUFF_EDMAChannelCfg* ptrEDMAChannelCfg);

/**
 *  @b Description
 *  @n
 *      This is the function which is registered with the CBUFF driver and is invoked
 *      by the driver whenever it needs to free an allocated EDMA channel.
 *
 *  @param[in] ptrEDMAChannelCfg
 *      Pointer to the EDMA chnanel configuration which is to be released
 *
 *  @retval
 *      Not applicable
 */
typedef void (*CBUFF_EDMAChannelFreeFxn) (CBUFF_EDMAChannelCfg* ptrEDMAChannelCfg);

/**
 *  @b Description
 *  @n
 *      Sessions can register for Frame Done interrupts. This callback function
 *      is invoked if the session is currently active and the CBUFF Driver receives
 *      a frame done interrupt. Application should use this to activate another
 *      session. Switching between active sessions while the frame is not done can
 *      lead to unexpected results.
 *
 *  @param[in] sessionHandle
 *      Session Handle which was active for which the Frame done interrupt was
 *      received
 *
 *  @retval
 *      Not applicable
 */
typedef void (*CBUFF_FrameDoneCallbackFxn) (CBUFF_SessionHandle sessionHandle);

/**
 * \brief
 *  CBUFF Hardware Triggered Session configuration
 *
 * \details
 *  The structure describes the configuration required to be specified if
 *  the session is created in hardware triggered mode.
 */
typedef struct CBUFF_HwSessionCfg_t
{
        /**
     * \brief   ADCBUF Driver Handle: Ensure that the ADC Channels are
     * enabled and configured.
     */
    ADCBuf_Handle           adcBufHandle;

    /**
     * \brief   Data Format: This is used to describe the format of the data
     * which is being sent out via the HSI.
     */
    CBUFF_DataFmt           dataFormat;

    /**
     * \brief   ADC Buffer data storage mode: Interleaved or Non-Interleaved
     * This is only used if the data format is configured to send out ADC Data
     */
    CBUFF_DataMode          dataMode;

    /**
     * \brief   Operational mode for the driver:
     */
    CBUFF_OperationalMode   opMode;

    /**
     * \brief This field is described as follows:-
     *
     *  - Chirp Mode     : Number of chirps per frame
     *  - Continuous Mode: This field is ignored
     */
    uint32_t                numChirpsPerFrame;

    /**
     * \brief This field is described as follows:-
     *  - Single Chirp Mode : Set this to 1
     *  - Multi-Chirp  Mode : Set this between 2 to 8
     *  - Continuous Mode   : Set this to 0
     *
     * *NOTE*: Multi-Chirp mode is possible only on the XWR16xx/XWR18xx/XWR68xx.
     *         But default CBUFF library disables this on these devices.
     *         In order to include support for multi-chirp mode, populate the multi-chirp table
     *         as instructed in the platform/cbuff_\<device\>.c file.
     */
    uint32_t                chirpMode;

    /**
     * \brief This field is described as follows:-
     *
     *  - Chirp Mode      : This is the number of ADC samples per chirp per channel
     *  - Continuous Mode : This is the number of samples per channel which is configured
     *                      in the ADC Buffer
     *
     * This is only used if the data format is configured to send out ADC data.
     */
    uint16_t                numADCSamples;

    /**
     * \brief   Chirp Quality Size: The size is specified in CBUFF units
     * If the size is set to 0; CQx is ignored.
     */
    uint16_t                cqSize[ADCBufMMWave_CQType_MAX_CQ];

    /**
     * \brief   User supplied data buffers which can be transmitted over the interface
     */
    CBUFF_BufferCfg         userBufferInfo[CBUFF_MAX_USER_BUFFER];
}CBUFF_HwSessionCfg;

/**
 * \brief
 *  CBUFF Software Triggered Session configuration
 *
 * \details
 *  The structure describes the configuration required to be specified if
 *  the session is created in software triggered mode.
 */
typedef struct CBUFF_SwSessionCfg_t
{
    /**
     * \brief   User supplied data buffers which can be transmitted over the interface
     */
    CBUFF_BufferCfg     userBufferInfo[CBUFF_MAX_USER_BUFFER];
}CBUFF_SwSessionCfg;

/**
 * \brief
 *  CBUFF configuration
 *
 * \details
 *  The structure describes the configuration which is required to configure
 *  the CBUFF Driver.
 */
typedef struct CBUFF_SessionCfg_t
{
    /**
     * \brief   Session execution mode: Hardware or Software triggered
     */
    CBUFF_SessionExecuteMode        executionMode;

    /**
     * \brief   This is the callback function which is triggered once the frame has
     * been sent over the HSI. This can be set to NULL if the application only uses
     * a single session. But if multiple sessions are being used application should
     * register this callback function in order to switch from one session to another
     *
     * *NOTE*: Applications are responsible for ensuring that the switch between the
     * sessions is done between the inter-frame boundaries. Failure to enforce this
     * will result in unpredictable behavior.
     */
    CBUFF_FrameDoneCallbackFxn      frameDoneCallbackFxn;

    /**
     * \brief   EDMA Instance Handle: The session will allocate EDMA channels
     */
    EDMA_Handle                     edmaHandle;

    /**
     * \brief   Application provided EDMA Channel allocation function.
     */
    CBUFF_EDMAChannelAllocateFxn    allocateEDMAChannelFxn;

    /**
     * \brief   Application provided EDMA Channel free function.
     */
    CBUFF_EDMAChannelFreeFxn        freeEDMAChannelFxn;

    /**
     * \brief   Type of Data: Real or Complex which is going to be streamed out
     * via the CBUFF High speed interface
     */
    CBUFF_DataType                  dataType;

    /**
     * \brief   This is the header which needs to be added to the stream. If the
     * size in the header is set to 0 then no header will be appended.
     */
    CBUFF_BufferCfg                 header;

    union
    {
        /**
         * \brief   Configuration used if the session is executing in hardware
         * trigerred mode
         */
        CBUFF_HwSessionCfg      hwCfg;

        /**
         * \brief   Configuration used if the session is executing in software
         * trigerred mode
         */
        CBUFF_SwSessionCfg      swCfg;
    }u;
}CBUFF_SessionCfg;

/**
 * \brief
 *  CBUFF Initialization Configuration
 *
 * \details
 *  The structure describes the configuration which is required to initialize
 *  the CBUFF Driver.
 */
typedef struct CBUFF_InitCfg_t
{

    /**
     * \brief   This is used to specify the LVDS/CSI2 output format.
     */
    CBUFF_OutputDataFmt             outputDataFmt;

    /**
     * \brief   Enable/Disable the ECC in the CBUFF module
     */
    uint8_t                         enableECC;

    /**
     * \brief   In order to ensure data integrity of data transfer from ADC buffer to CBUFF
     * there is a CRC computed and this checked for integrity in the hardware at both the
     * source (ADC Buffer) and destination (CBUFF)
     */
    uint8_t                         crcEnable;

    /**
     * \brief   This is the maximum number of sessions which can be supported
     * for the CBUFF Instance. This value needs to be >= 1 and < CBUFF_MAX_NUM_SESSION.
     * Any other value will result in an invalid argument error.
     */
    uint8_t                         maxSessions;

    /**
     * \brief   This is a flag which if set to true will register the ISR to track
     * Frame Start/Done and Chirp done.
     *
     * *NOTE*: Enable the debug mode will allow the statistics to increment and can
     * be useful to ensure that the streaming is working. But this can overwhelm the
     * system with a large number of interrupts.
     *
     * Certain interrupt such as the Frame Done interrupts are required to be registered
     * if multiple sessions are created. Switching from one session to another can
     * only be done with the intra-frame boundary.
     */
    bool                            enableDebugMode;

    /**
     * \brief   The interface over which the CBUFF module will send out the data
     */
    CBUFF_Interface                 interface;

	/**
	 * \brief   LVDS Initialization configuration: This needs to be specified
	 * if the interface is configured to be LVDS.
	 */
	CBUFF_LVDSCfg                   lvdsCfg;
}CBUFF_InitCfg;

/**
 * \brief
 *  CBUFF EDMA Tracking Entry
 *
 * \details
 *  The structure holds information about the EDMA channel which has been
 *  added to correspond to the linked list entry to DMA the data into the
 *  CBUFF.
 */
typedef struct CBUFF_EDMATrackingEntry_t
{
    /**
     * \brief   EDMA Channel Configuration allocated by the application
     * and which is used to program the CBUFF Transfers
     */
    CBUFF_EDMAChannelCfg    cbuffEDMAChannelCfg;

    /**
     * \brief   Source address of the EDMA Transfer
     */
    uint32_t                srcAddress;

    /**
     * \brief   Size of the data being transferred by the EDMA. This is stored
     * in CBUFF Units.
     */
    uint32_t                transferSize;
}CBUFF_EDMATrackingEntry;

/**
 * \brief
 *  CBUFF Session
 *
 * \details
 *  The structure is used to track the relevant configuration and run time
 *  information for each session in the CBUFF driver
 */
typedef struct CBUFF_Session_t
{
    /**
     * \brief   Status flag which indicates if the session is active or not.
     */
    bool                            isValid;

    /**
     * \brief   This is the back pointer to the CBUFF Driver MCB
     */
    struct CBUFF_Driver_t*          ptrDriverMCB;

    /**
     * \brief   Configuration used to create the session.
     */
    CBUFF_SessionCfg                sessionCfg;

    /**
     * \brief   DMA number associated with the session.
     */
    uint8_t                         dmaNum;

    /**
     * \brief   Number of active ADC Channels
     */
    uint8_t                         numActiveADCChannels;

    /**
     * \brief   Number of ADC Sample per chirp * [Complex(2) *or* Real(1)]
     */
    uint16_t                        adcTransferSize;

    /**
     * \brief   ADC Receive Channel Address: This is computed from the ADCBUF
     * driver if the session is operating in HW Triggered mode.
     */
    uint32_t                        rxChannelAddress[SOC_ADCBUF_NUM_RX_CHANNEL];

    /**
     * \brief   Chirp Quality Buffer Information:
     *  While sending CQ it is possible to only send CQ1 or CQ2 or both. In
     *  order to handle this the following array is used:-
     *
     * Only CQ1:
     *  cqBuffer[0] = CQ1;
     *
     * Only CQ2:
     *  cqBuffer[0] = CQ2;
     *
     * Both
     *  cqBuffer[0] = CQ1;
     *  cqBuffer[1] = CQ2;
     */
    CBUFF_BufferCfg                 cqBuffer[ADCBufMMWave_CQType_MAX_CQ];

    /**
     * \brief   Number of CQ Detected: This works in conjuction with the
     * CQ Buffer described above.
     */
    uint8_t                         numCQ;

    /**
     * \brief   Chirp Parameter Transfer Size: Sum total of the size of all
     * valid chirps
     */
    uint16_t                        cpTotalTransferSize;

    /**
     * \brief   Link Index which tracks the linked list entries being added
     */
    uint8_t                         linkListIndex;

    /**
     * \brief   Linked List Tracker used to keep track of the CBUFF Linked List
     * entries which have been added
     */
    CBUFF_LinkListParams            linkedListTracker[CBUFF_MAX_LINKED_LIST_SIZE];

    /**
     * \brief   Counter which tracks the EDMA channels
     */
    uint8_t                         edmaChannelCount;

    /**
     * \brief   Tracks all the EDMA entries which have been added
     */
    CBUFF_EDMATrackingEntry         edmaTrackingEntry[CBUFF_EDMA_MAX_NUM_CHANNELS];

    /**
     * \brief This is the number of chirps per frame.
     * - Hardware Triggered Execution Mode
     *   This is derived from the Session H/W configuration
     *      - Chirp Mode     : Number of chirps per frame
     *      - Continuous Mode: This field is ignored
     * - Software Triggered Execution Mode
     *   This is always set to 1.
     */
    uint32_t                        numChirpsPerFrame;

    /**
     * \brief   Number of frame done interrupts received
     */
    volatile uint32_t               numFrameDone;

    /**
     * \brief   Number of chirp done interrupts received
     */
    volatile uint32_t               numChirpDone;
}CBUFF_Session;

/**
 * \brief
 *  CBUFF Driver
 *
 * \details
 *  The structure is used to track the relevant configuration and run time
 *  information for the CBUFF Driver.
 */
typedef struct CBUFF_Driver_t
{
    /**
     * \brief   Initialization configuration used to setup the driver.
     * This is valid once the driver has been INITIALIZED
     */
    CBUFF_InitCfg                   initCfg;

    /**
     * \brief   CBUFF register space
     */
    CSL_CbuffRegs*                   ptrCBUFFReg;

    /**
     * \brief   High speed interface function table to be used by the CBUFF Driver
     */
    struct CBUFF_InterfaceFxn_t*    interfaceFxn;

    /**
     * \brief   Registered interrupt handler for the CBUFF Module.
     */
    HwiP_Object                     hwiISRHandle;

    /**
     * \brief   Registered interrupt handler for the CBUFF Module.
     */
    HwiP_Object                     hwiErrorISRHandle;

    /**
     * \brief   This is the table which tracks all the sessions which can be created by
     * the CBUFF Driver.
     */
    CBUFF_Session*                  ptrSessionTable;

    /**
     * \brief   This is the pointer to the currently active session. This can be NULL
     * to indicate that no session is currently active.
     */
    CBUFF_Session*                  ptrCurrentActiveSession;

    /**
     * \brief   Number of active LVDS lanes configured
     */
    uint8_t                         numActiveLVDSLanes;

    /**
     * \brief HWA Hardware related params
     */
    CBUFF_Attrs          const       *hwAttrs;

    /**
     * \brief   Total number of frame done interrupts received
     */
    volatile uint32_t               totalNumFrameDone;

    /**
     * \brief   Total number of chirp done interrupts received
     */
    volatile uint32_t               totalNumChirpDone;

    /**
     * \brief   Number of interrupts received when there is no active session
     */
    uint32_t                        totalNumNonActiveSessionInterrupts;

    /**
     * \brief   Total number of error interrupts received
     */
    volatile uint32_t               totalNumErrorInterrupts;
}CBUFF_Object;

/**
 * \brief
 *  This is the function invoked by the CBUFF driver to initialize the high
 *  speed interface.
 *
 *  @param[in]  ptrDriverMCB
 *      Pointer to the CBUFF Driver MCB
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
typedef int32_t (*CBUFF_initFxn) (CBUFF_Object* ptrDriverMCB, int32_t* errCode);

/**
 * \brief
 *  This is the function invoked by the CBUFF driver to deinitialize and shutdown
 *  the high speed interface
 *
 *  @param[in]  ptrDriverMCB
 *      Pointer to the CBUFF Driver MCB
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
typedef int32_t (*CBUFF_deinitFxn) (CBUFF_Object* ptrDriverMCB, int32_t* errCode);

/**
 * \brief
 *  This is the function invoked by the CBUFF driver to open the high
 *  speed interface for the specific session
 *
 *  @param[in]  ptrSession
 *      Pointer to the session
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
typedef int32_t (*CBUFF_openFxn) (CBUFF_Session* ptrSession, int32_t* errCode);

/**
 * \brief
 *  This is the function invoked by the CBUFF driver to close the high
 *  speed interface for the specific session
 *
 *  @param[in]  ptrSession
 *      Pointer to the session
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
typedef int32_t (*CBUFF_closeFxn) (CBUFF_Session* ptrSession, int32_t* errCode);

/**
 * \brief
 *  This is the function prototype invoked by the CBUFF Driver to initialize the
 *  linked list parameters for each high speed interface
 *
 *  @param[in]  ptrSession
 *      Pointer to the CBUFF Session
 *  @param[out] ptrLinkListParams
 *      Pointer to the linked list configuration populated by the API
 *
 *  @retval
 *      Not applicable
 */
typedef void (*CBUFF_initLinkListParamsFxn) (CBUFF_Session* ptrSession, CBUFF_LinkListParams* ptrLinkListParams);

/**
 * \brief
 *  This is the function prototype invoked by the CBUFF Driver to finalize the linked
 *  list parameters for the high speed interface. Once the parameters have been finalized
 *  they can be written to the CBUFF IP.
 *
 *  @param[in]  ptrSession
 *      Pointer to the CBUFF Session
 *  @param[in] ptrLinkListParams
 *      Pointer to the linked list configuration
 *
 *  @retval
 *      Not applicable
 */
typedef void (*CBUFF_finalizeLinkListParamsFxn) (CBUFF_Session* ptrSession, CBUFF_LinkListParams* ptrLinkListParams);

/**
 * \brief
 *  CBUFF Interface Function
 *
 * \details
 *  This is the call table which encapsulates the CBUFF Driver from the high
 *  speed interface.
 */
typedef struct CBUFF_InterfaceFxn_t
{
    /**
     * \brief   High speed interface for which the interface functions are defined
     */
    CBUFF_Interface                 interface;

    /**
     * \brief   This is the function which is used to initialize the high speed
     * interface
     */
    CBUFF_initFxn                   initFxn;

    /**
     * \brief   This is the function which is used to deinitialize and shutdown
     * the high speed interface
     */
    CBUFF_deinitFxn                 deinitFxn;

    /**
     * \brief   This is the function which is used to open and setup the high
     * speed interface
     */
    CBUFF_openFxn                   openFxn;

    /**
     * \brief   This is the function which is used to close the high speed interface
     */
    CBUFF_closeFxn                  closeFxn;

    /**
     * \brief   This is the function which is used to initialize the linked list
     * parameters to the default values for the high speed interface
     */
    CBUFF_initLinkListParamsFxn     initLinkListParamsFxn;

    /**
     * \brief   This is the function which is used to finalize and setup the linked
     * list parameters after which they can be wriiten to the CBUFF IP.
     */
    CBUFF_finalizeLinkListParamsFxn finalizeLinkListParamsFxn;
}CBUFF_InterfaceFxn;

/**
 * \brief
 *  This is the function prototype invoked by the CBUFF Driver to setup the
 *  transfers in the CBUFF IP. This will handle the configuration of the
 *  linked list as well as the EDMA Transfers
 *
 *  @param[in]  ptrDriverMCB
 *      Pointer to the CBUFF Driver
 *  @param[in]  ptrSession
 *      Pointer to the CBUFF Session
 *  @param[out] errCode
 *      Error code populated on error
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
typedef int32_t (*CBUFF_setupTransferFxn) (CBUFF_Object* ptrDriverMCB, CBUFF_Session* ptrSession, int32_t* errCode);



/** @} */ /* end defgroup CBUFF_DRIVER_EXTERNAL_DATA_STRUCTURE */

/** \brief Externally defined driver configuration array */
extern CBUFF_Attrs        gCbuffAttrs[];
/** \brief Externally defined driver object */
extern CBUFF_Object       gCbuffObject[];
/** \brief Externally defined driver object pointer */
extern CBUFF_Object      *gCbuffObjectPtr[];
/** \brief Externally defined driver configuration array size */
extern uint32_t           gCbuffConfigNum;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \defgroup CBUFF_DRIVER_EXTERNAL_FUNCTION       CBUFF Driver External Functions
 *  \ingroup DRV_CBUFF_MODULE
 *  \brief The section has a list of all the exported API which the applications need to
 *   invoke in order to use the driver
 * @{
 */

/**
 *  \brief Description
 *  \n
 *      This function initializes the CBUFF module. This function must be called
 *      before any other functions are called.
 *
 */
void CBUFF_init(void);


/**
 *  \brief Description
 *  \n
 *      The function Configures CBUFF peripheral with user provided configuration.
 *
 *  \param[in]  ptrInitCfg      A pointer to CBUFF_InitCfg structure for initialization
 *
 *  \param[out]  errCode
 *       Pointer to an error code populated by the driver.
 *
 *  \return Success     - Handle to the CBUFF Driver
 *          Error       - NULL
 */
CBUFF_Handle CBUFF_open (CBUFF_InitCfg* ptrInitCfg, int32_t* errCode);


/**
 *  \brief Description
 *  \n
 *     The function creates a session with the specified configuration.
 *
 *  \param[in]  cbuffHandle
 *       Handle to the CBUFF driver.
 *
 *  \param[in]  ptrSessionCfg
 *       Pointer to the session configuration.
 *
 *  \param[out] errCode
 *       Pointer to an error code populated by the driver.
 *
 *  \return Success     - Handle to the session
 *          Error       - NULL
 */
CBUFF_SessionHandle CBUFF_createSession (CBUFF_Handle cbuffHandle, CBUFF_SessionCfg* ptrSessionCfg, int32_t* errCode);


/**
 *  \brief Description
 *  \n
 *     The function deletes the specific session.
 *
 *  \param[in]  sessionHandle
 *       Handle to the CBUFF session.
 *
 *  \param[out] errCode
 *       Pointer to an error code populated by the driver.
 *
 *  \return Success     - \ref CBUFF_STATUS_SUCCESS
 *          Error       - \ref CBUFF_ERROR_CODES
 */
int32_t CBUFF_close(CBUFF_SessionHandle sessionHandle, int32_t* errCode);

/**
 *  \brief Description
 *  \n
 *     The function is used to deinitialize and shutdown the CBUFF driver.
 *
 *  \param[in]  cBuffHandle
 *       Handle to the CBUFF instance obtained through call to \ref CBUFF_open.
 *
 *  \param[out] errCode
 *       Pointer to an error code populated by the driver.
 *
 *  \return Success     - \ref CBUFF_STATUS_SUCCESS
 *          Error       - \ref CBUFF_ERROR_CODES
 */
int32_t CBUFF_deinit (CBUFF_Handle cBuffHandle, int32_t* errCode);

/**
 *  \brief Description
 *  \n
 *     The function is used to get/set information from the CBUFF Driver
 *
 *  \param[in]  cBuffHandle
 *       Handle to the CBUFF instance obtained through call to \ref CBUFF_open.
 *
 *  \param[in]  cBuffHandle
 *      Handle to the driver
 *  \param[in]  cmd
 *      CBUFF command
 *  \param[in]  arg
 *      Command specified pointer to the argument
 *  \param[in]  argLen
 *      Length of the argument
 *  \param[out] errCode
 *       Pointer to an error code populated by the driver.
 *
 *  \return Success     - \ref CBUFF_STATUS_SUCCESS
 *          Error       - \ref CBUFF_ERROR_CODES
 */
int32_t CBUFF_control(CBUFF_Handle cBuffHandle, CBUFF_Command cmd, void* arg, uint32_t argLen, int32_t* errCode);

/**
 *  \brief Description
 *  \n
 *       The function activates the specific CBUFF session.
 *
 *  \param[in]  sessionHandle
 *       Handle to the session to be activated \ref CBUFF_createSession.
 *
 *  \param[out] errCode
 *       Pointer to an error code populated by the driver.
 *
 *  \return Success     - \ref CBUFF_STATUS_SUCCESS
 *          Error       - \ref CBUFF_ERROR_CODES
 */
int32_t CBUFF_activateSession (CBUFF_SessionHandle sessionHandle, int32_t* errCode);

/**
 *  \brief Description
 *  \n
 *       The function deactivates the specific CBUFF session.
 *
 *  \param[in]  sessionHandle
 *       Handle to the session to be deactivated \ref CBUFF_createSession.
 *
 *  \param[out] errCode
 *       Pointer to an error code populated by the driver.
 *
 *  \return Success     - \ref CBUFF_STATUS_SUCCESS
 *          Error       - \ref CBUFF_ERROR_CODES
 */
int32_t CBUFF_deactivateSession (CBUFF_SessionHandle sessionHandle, int32_t* errCode);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef CBUFF_V0_H_ */

/** @} */
