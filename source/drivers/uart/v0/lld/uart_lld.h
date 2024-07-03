/*
 * Copyright (C) 2021-2023 Texas Instruments Incorporated
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
 *  \defgroup DRV_UART_LLD_MODULE APIs for UART
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the UART.
 *
 *  @{
 */

/**
 *  \file v0/lld/uart_lld.h
 *
 *  \brief This file contains the prototype of UART driver APIs
 */

#ifndef UART_LLD_H_
#define UART_LLD_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/cslr_uart.h>
#include <drivers/hw_include/hw_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/** \brief A handle that is returned from a #UART_open() call */
typedef void *UART_DmaHandle;

typedef void *UART_DmaChConfig;

/** \brief Uart FIFO Size */
#define UART_FIFO_SIZE                  (64U)

/**
 *  \brief Timeout in ms used for TX FIFO empty at the time of delete. Three
 *  seconds is more than sufficient to transfer 64 bytes (FIFO size) at the
 *  lowest baud rate of 2400.
 */
#define UART_TRANSMITEMPTY_TRIALCOUNT   (3000U)

/** \brief Count Value to check error in the recieved byte  */
#define UART_ERROR_COUNT            (0x00FFFFFFU)

/**
 *  \anchor UART_TransferStatus
 *  \name Transfer Status Code
 *
 *  Status codes that are set by the UART driver
 *
 *  @{
 */
/** \brief Transaction success */
#define UART_STATUS_SUCCESS                 ((int32_t)0)
/** \brief Transaction failure */
#define UART_STATUS_FAILURE                 ((int32_t)-1)
/** \brief Transaction success */
#define UART_TRANSFER_STATUS_SUCCESS         ((int32_t)0)
/** \brief Transaction failure */
#define UART_TRANSFER_STATUS_FAILURE         ((int32_t)-1)

/** \brief Value to use when needing a
 * timeout of infinity or wait forver
 * until resource is available
 */
#define UART_WAIT_FOREVER                   ((int32_t)-1)
/** \brief Return status when the API
 * execution
 * failed due invalid parameters */
#define UART_INVALID_PARAM                  ((int32_t)-3)
/**
 * \brief Return status when the API
 * execution failed due to driver busy
 */
#define UART_TRANSFER_BUSY                  ((int32_t)-4)
/**
 * \brief Return status when the API
 *  execution was not successful due to
 *  a time out
 */
#define UART_TRANSFER_TIMEOUT               ((int32_t)-2)
/**
 * \brief Return status when the API
 * execution failed due to invalid LLD
 * state
 */
#define UART_TRANSFER_INVALID_STATE        ((int32_t)-5)
/** \brief Time out error */
#define UART_TRANSFER_STATUS_TIMEOUT         (1U)
/** \brief Break condition error */
#define UART_TRANSFER_STATUS_ERROR_BI        (2U)
/** \brief Framing error */
#define UART_TRANSFER_STATUS_ERROR_FE        (3U)
/** \brief Parity error */
#define UART_TRANSFER_STATUS_ERROR_PE        (4U)
/** \brief Overrun error */
#define UART_TRANSFER_STATUS_ERROR_OE        (5U)
/** \brief Cancelled */
#define UART_TRANSFER_STATUS_CANCELLED       (6U)
/** \brief Transaction started */
#define UART_TRANSFER_STATUS_STARTED         (7U)
/** \brief Read timeout error */
#define UART_TRANSFER_STATUS_READ_TIMEOUT    (8U)
/** \brief UART is currently in use */
#define UART_TRANSFER_STATUS_ERROR_INUSE     (9U)
/** \brief Other errors */
#define UART_TRANSFER_STATUS_ERROR_OTH       (10U)
/** @} */

/**
 *  \anchor UART_ReadReturnMode
 *  \name UART Read length
 *
 *  This enumeration defines the return modes for UART_read().
 *
 *  #UART_READ_RETURN_MODE_FULL unblocks or performs a callback when the read
 *  buffer has been filled with the number of bytes passed to #UART_read().
 *  #UART_READ_RETURN_MODE_PARTIAL unblocks or performs a callback whenever a
 *  read timeout error occurs on the UART peripheral.
 *  The read timeout occurs if the read FIFO is non-empty and no new
 *  data has been received for a specific device/baudrate dependent number of
 *  clock cycles.  This mode can be used when the exact number of bytes to
 *  be read is not known.
 *
 *  @{
 */
/**
 *  \brief Unblock/callback when buffer is full.
 */
#define UART_READ_RETURN_MODE_FULL        (0U)
/**
 *  \brief Unblock/callback when no new data comes in.
 */
#define UART_READ_RETURN_MODE_PARTIAL     (1U)
/** @} */

/**
 *  \anchor UART_DataLength
 *  \name UART data length
 *
 *  Note: The values should not be changed since it represents the
 *        actual register configuration values used to configure the UART
 *  @{
 */
#define UART_LEN_5                      (0U)
#define UART_LEN_6                      (1U)
#define UART_LEN_7                      (2U)
#define UART_LEN_8                      (3U)
/** @} */

/**
 *  \anchor UART Status
 *  \name Transfer Status Code
 *
 *  Status codes that are set by the UART driver
 *
 *  @{
 */
#define UART_STATE_RESET (0U)
#define UART_STATE_READY (1U)
#define UART_STATE_BUSY  (2U)
#define UART_STATE_ERROR (3U)
/** @} */

/**
 * \brief NOT_IN_USE macro to highlight unused parameters
 */
#define UART_NOT_IN_USE(x) (void) 0
/**
 *  \anchor UART_StopBits
 *  \name UART stop bits
 *
 *  Note: The values should not be changed since it represents the
 *        actual register configuration values used to configure the UART
 *  @{
 */
#define UART_STOPBITS_1                 (0U)
#define UART_STOPBITS_2                 (1U)
#define UART_STOPBITS_1P5               (UART_STOPBITS_1)
/** @} */

/**
 *  \anchor UART_Parity
 *  \name UART Parity
 *
 *  Note: The values should not be changed since it represents the
 *        actual register configuration values used to configure the UART
 *  @{
 */
#define UART_PARITY_NONE                (0x00U)
#define UART_PARITY_ODD                 (0x01U)
#define UART_PARITY_EVEN                (0x03U)
#define UART_PARITY_FORCED0             (0x07U)
#define UART_PARITY_FORCED1             (0x05U)
/** @} */

/**
 *  \anchor UART_FlowControlType
 *  \name UART Flow Control Type
 *
 *  Note: The values should not be changed since it represents the
 *        actual register configuration values used to configure the UART
 *  @{
 */
#define UART_FCTYPE_NONE                (0x00U)
#define UART_FCTYPE_HW                  (0x02U)
/** @} */

/**
 *  \anchor UART_FlowControlParamsRx
 *  \name UART Flow Control Params for RX
 *
 *  Note: The values should not be changed since it represents the
 *        actual register configuration values used to configure the UART
 *  @{
 */
#define UART_FCPARAM_RXNONE             (0x00U)
#define UART_FCPARAM_RXXONXOFF_2        (0x01U)
#define UART_FCPARAM_RXXONXOFF_1        (0x02U)
#define UART_FCPARAM_RXXONXOFF_12       (0x03U)
#define UART_FCPARAM_AUTO_RTS           (0x40U)
/** @} */

/**
 *  \anchor UART_FlowControlParamsTx
 *  \name UART Flow Control Params for TX
 *
 *  Note: The values should not be changed since it represents the
 *        actual register configuration values used to configure the UART
 *  @{
 */
#define UART_FCPARAM_TXNONE             (0x00U)
#define UART_FCPARAM_TXXONXOFF_2        (0x04U)
#define UART_FCPARAM_TXXONXOFF_1        (0x08U)
#define UART_FCPARAM_TXXONXOFF_12       (0x0CU)
#define UART_FCPARAM_AUTO_CTS           (0x80U)
/** @} */

/**
 *  \anchor UART_RxTrigLvl
 *  \name UART RX trigger level
 *
 *  Note: The values should not be changed since it represents the
 *        actual register configuration values used to configure the UART
 *  @{
 */
#define UART_RXTRIGLVL_1                (1U)
#define UART_RXTRIGLVL_8                (8U)
#define UART_RXTRIGLVL_16               (16U)
#define UART_RXTRIGLVL_56               (56U)
#define UART_RXTRIGLVL_60               (60U)
/** @} */

/**
 *  \anchor UART_TxTrigLvl
 *  \name UART TX trigger level
 *
 *  Note: The values should not be changed since it represents the
 *        actual register configuration values used to configure the UART
 *  @{
 */
#define UART_TXTRIGLVL_1                (1U)
#define UART_TXTRIGLVL_8                (8U)
#define UART_TXTRIGLVL_16               (16U)
#define UART_TXTRIGLVL_32               (32U)
#define UART_TXTRIGLVL_56               (56U)
/** @} */

/**
 *  \anchor UART_OperMode
 *  \name UART Operational Mode
 *
 *  Note: The values should not be changed since it represents the
 *        actual register configuration values used to configure the UART
 *  @{
 */
#define UART_OPER_MODE_16X              (0U)
#define UART_OPER_MODE_SIR              (1U)
#define UART_OPER_MODE_16X_AUTO_BAUD    (2U)
#define UART_OPER_MODE_13X              (3U)
#define UART_OPER_MODE_MIR              (4U)
#define UART_OPER_MODE_FIR              (5U)
#define UART_OPER_MODE_CIR              (6U)
#define UART_OPER_MODE_DISABLED         (7U)
/** @} */

/**
 *  \anchor UART_TXFIFO
 *  \name Values indicating the filled status of TX FIFO
 *
 *  Note: The values should not be changed since it represents the
 *        actual register configuration values used to configure the UART
 *  @{
 */

#define UART_TX_FIFO_NOT_FULL               ( \
        UART_SSR_TX_FIFO_FULL_TX_FIFO_FULL_VALUE_0)
#define UART_TX_FIFO_FULL                   ( \
        UART_SSR_TX_FIFO_FULL_TX_FIFO_FULL_VALUE_1)
/** @} */

/**
 *  \anchor UART_IntrSources
**  \name Values related to status of Interrupt souces.
* @{
*/

/** \brief Values pertaining to status of UART Interrupt sources. */

#define UART_INTID_MODEM_STAT               (UART_IIR_IT_TYPE_IT_TYPE_VALUE_0 \
                                             <<                               \
                                             UART_IIR_IT_TYPE_SHIFT)
#define UART_INTID_TX_THRES_REACH           (UART_IIR_IT_TYPE_IT_TYPE_VALUE_1 \
                                             <<                               \
                                             UART_IIR_IT_TYPE_SHIFT)
#define UART_INTID_RX_THRES_REACH           (UART_IIR_IT_TYPE_IT_TYPE_VALUE_2 \
                                             <<                               \
                                             UART_IIR_IT_TYPE_SHIFT)
#define UART_INTID_RX_LINE_STAT_ERROR       (UART_IIR_IT_TYPE_IT_TYPE_VALUE_3 \
                                             <<                               \
                                             UART_IIR_IT_TYPE_SHIFT)
#define UART_INTID_CHAR_TIMEOUT             (UART_IIR_IT_TYPE_IT_TYPE_VALUE_6 \
                                             <<                               \
                                             UART_IIR_IT_TYPE_SHIFT)
#define UART_INTID_XOFF_SPEC_CHAR_DETECT    (UART_IIR_IT_TYPE_IT_TYPE_VALUE_8 \
                                             <<                               \
                                             UART_IIR_IT_TYPE_SHIFT)
#define UART_INTID_MODEM_SIG_STATE_CHANGE   (UART_IIR_IT_TYPE_IT_TYPE_VALUE_10 \
                                             <<                                \
                                             UART_IIR_IT_TYPE_SHIFT)

/** \brief Values indicating the UART Interrupt pending status. */
#define UART_INTR_PENDING                   (0U)
#define UART_N0_INTR_PENDING                (1U)
/** @} */

/**
** \name Values related to enabling/disabling of Interrupts.
* @{
*/

/** \brief Values for enabling/disabling the interrupts of UART. */

#define UART_INTR_CTS                       (UART_IER_CTS_IT_MASK)
#define UART_INTR_RTS                       (UART_IER_RTS_IT_MASK)
#define UART_INTR_XOFF                      (UART_IER_XOFF_IT_MASK)
#define UART_INTR_SLEEPMODE                 (UART_IER_SLEEP_MODE_MASK)
#define UART_INTR_MODEM_STAT                (UART_IER_MODEM_STS_IT_MASK)
#define UART_INTR_LINE_STAT                 (UART_IER_LINE_STS_IT_MASK)
#define UART_INTR_THR                       (UART_IER_THR_IT_MASK)
#define UART_INTR_RHR_CTI                   (UART_IER_RHR_IT_MASK)

#define UART_INTR2_RX_EMPTY                 (UART_IER2_EN_RXFIFO_EMPTY_MASK)
#define UART_INTR2_TX_EMPTY                 (UART_IER2_EN_TXFIFO_EMPTY_MASK)
/** @} */

/**
** \name Values related to Line Status information.
* @{
*/

/** \brief Values pertaining to UART Line Status information. */

#define UART_FIFO_PE_FE_BI_DETECTED         (UART_LSR_RX_FIFO_STS_MASK)
#define UART_BREAK_DETECTED_ERROR           (UART_LSR_RX_BI_MASK)
#define UART_FRAMING_ERROR                  (UART_LSR_RX_FE_MASK)
#define UART_PARITY_ERROR                   (UART_LSR_RX_PE_MASK)
#define UART_OVERRUN_ERROR                  (UART_LSR_RX_OE_MASK)
/** @} */

/**
** \name Values related to Register Mode Configuration.
* @{
*/

/** \brief Values to be used while switching between register
 * configuration modes. */
#define UART_REG_CONFIG_MODE_A          ((uint32_t) 0x0080)
#define UART_REG_CONFIG_MODE_B          ((uint32_t) 0x00BF)
#define UART_REG_OPERATIONAL_MODE       ((uint32_t) 0x007F)
/** @} */

/**
 *  \anchor UART_ConfigMode
 *  \name UART Configration Mode
 *
 *  This determines whether the driver configuration mode like Polled,
 *  Interrupt, Dma used for the transfer function
 *
 *  @{
 */
#define UART_CONFIG_MODE_POLLED       (0x00U)
#define UART_CONFIG_MODE_INTERRUPT    (0x01U)
#define UART_CONFIG_MODE_USER_INTR    (0x02U)
#define UART_CONFIG_MODE_DMA          (0x03U)
#define UART_CONFIG_MODE_POLLED_WITH_COUNTER (0x04U)
/** @} */

#define UART_STATE_RX_DISABLED        (0x0U)
#define UART_STATE_RX_ENABLED         (0x1U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */
typedef struct UART_ExtendedParams_s
{
     void                   *args;
    /**< [IN] Argument to be passed to the callback function */
} UART_ExtendedParams;

/**
 *  \brief Data structure used with #UART_read() and #UART_write()
 *
 */
typedef struct
{
    void                   *buf;
    /**< [IN] void * to a buffer with data to be transferred .
     *   This parameter can't be NULL */
    uint32_t                count;
    /**< [IN/OUT] Number of bytes for this transaction.
      *  This is input incase of read/write call and on API return
      *  this represents number of bytes actually read by the API */
    uint32_t                timeout;
    /**< Timeout for this transaction in units of system ticks */
    uint32_t                status;
    /**< [OUT] \ref UART_TransferStatus code */
    void                   *args;
    /**< [IN] Argument to be passed to the callback function */
} UART_Transaction;

typedef uint32_t (*UART_clockGet) (void);
typedef uint32_t (*UART_clockUsecToTick) (uint64_t usecs);

/**
 *  \brief  The definition of a read complete callback function used by
 *  the UART driver when used in #UART_TRANSFER_MODE_CALLBACK
 *
 *  \param void*          void pointer
 */
typedef void (*UART_readCompCallbackFxn) (void *hUart);

/**
 *  \brief  The definition of a write complete callback function used by
 *  the UART driver when used in #UART_TRANSFER_MODE_CALLBACK
 *
 *  \param void*          void pointer
 */
typedef void (*UART_writeCompCallbackFxn) (void *hUart);

/**
 *  \brief  The definition of a error callback function used by the UART driver
 *  when used in #UART_TRANSFER_MODE_CALLBACK
 *
 *  \param void*          void pointer
 */
typedef void (*UART_errorCallbackFxn) (void *hUart);

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */
/**
 *  \brief UART driver initialization object
 */
typedef struct
{
    /*
     * SOC configuration
     */
    uint32_t                inputClkFreq;
    /**< Module input clock frequency */
    uint32_t                baudRate;
    /**< Baud rate for UART */
    uint32_t                dataLength;
    /**< Data length for UART. Refer \ref UART_DataLength */
    uint32_t                stopBits;
    /**< Stop bits for UART. Refer \ref UART_StopBits */
    uint32_t                parityType;
    /**< Parity bit type for UART. Refer \ref UART_Parity */
    uint32_t                readReturnMode;
    /**< Receive return mode  Refer \ref UART_ReadReturnMode */
    uint32_t                hwFlowControl;
    /** < Enable HW Flow Control */
    uint32_t                hwFlowControlThr;
    /**< Hardware flow Control threshold, greater than or equal to the
         RX FIFO trigger level \ref UART_RxTrigLvl */
    /*
     * Driver configuration
     */
    uint32_t                transferMode;
   /**< Transfer mode \ref UART_ConfigMode */
    uint32_t                intrNum;
    /**< Peripheral interrupt number */
    uint8_t                 intrPriority;
    /**< Interrupt priority */

    /*
     * UART configuration
     */
    uint32_t                operMode;
    /**< Receive return mode readMode */
    uint32_t                readMode;
    /**< Receive return mode writeMode */
    uint32_t                writeMode;
    /**< Refer \ref UART_OperMode for valid values */
    uint32_t                rxTrigLvl;
    /**< Refer \ref UART_RxTrigLvl for valid values */
    uint32_t                txTrigLvl;
    /**< Refer \ref UART_TxTrigLvl for valid values */
    uint32_t                rxEvtNum;
    /**< DMA Event number used for UART Rx */
    uint32_t                txEvtNum;
    /**< DMA Event number used for UART Tx */
    UART_DmaHandle         uartDmaHandle;
    /**< DMA Handle */
    UART_DmaChConfig       dmaChCfg;
    /**< DMA Configuration for this instance. */
    uint32_t               timeGuardVal;
    /* timeguard feature by UART*/
    UART_clockGet          clockP_get;
    /* ClockP_get  API */
    UART_clockUsecToTick           clockP_usecToTick;
    /* clock usec to tick */
    UART_readCompCallbackFxn        readCompleteCallbackFxn;
    /**< Read Callback function pointer */
    UART_writeCompCallbackFxn       writeCompleteCallbackFxn;
    /**< Write Callback function pointer */
    UART_errorCallbackFxn           errorCallbackFxn;
    /**< Error callback function */
} UARTLLD_InitObject, *UARTLLD_InitHandle;

/**
 *  \brief UART driver object
 */
typedef struct
{
    uint32_t                baseAddr;
    /**< Peripheral base address */
    UARTLLD_InitHandle        hUartInit;
    /**< [IN] Initialization parameters of UART instance */

    /*
     * UART write variables
     */
    const void             *writeBuf;
    /**< Buffer data pointer */
    uint32_t                writeCount;
    /**< Number of Chars sent */
    uint32_t                writeSizeRemaining;
    /**< Chars remaining in buffer */

    /*
     * UART receive variables
     */
    void                   *readBuf;
    /**< Buffer data pointer */
    uint32_t                readCount;
    /**< Number of Chars read */
    uint32_t                readSizeRemaining;
    /**< Chars remaining in buffer */
    uint32_t                rxTimeoutCnt;
    /**< Receive timeout error count */
    uint32_t                readErrorCnt;
    /**< Line status error count */

    /*
     * UART ransaction status variables
     */
    UART_Transaction        readTrans;
    /**< Pointer to the current read transaction */
    UART_Transaction        writeTrans;
    /**< Pointer to the current write transaction */
    uint32_t                currIntMask;

    /*
     * State variables
     */
    uint32_t                state;
    /**< Flag to indicate whether the instance is opened already */
    void                *readTransferMutex;
    /**< Read Transfer Sync Sempahore - to signal transfer completion */
    void                *writeTransferMutex;
    /**< Write Transfer Sync Sempahore - to signal transfer completion */

    void*                   args;
    /**< Pointer to be used by application to store miscellaneous data.*/
    uint64_t                lineStatusTimeout;
    /**< Variable to hold the line status timeout in ticks */
} UARTLLD_Object, *UARTLLD_Handle;

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

uint32_t UART_operatingModeSelect(uint32_t baseAddr, uint32_t modeFlag);
uint32_t UART_spaceAvail(uint32_t baseAddr);
uint32_t UART_IsTxRxFifoEmpty(uint32_t baseAddr);
int32_t UART_writeCancelNoCB(UARTLLD_Handle hUart);
int32_t UART_readCancelNoCB(UARTLLD_Handle hUart);

/* ========================================================================== */
/*                       Advanced Function Declarations                       */
/* ========================================================================== */

/**
 *  \brief  Function to get base address of UART instance of a particular
 *          handle.
 *
 *  \param  handle      #UARTLLD_Handle returned from #UART_open()
 *
 *  \sa     #UART_open
 */
uint32_t UART_getBaseAddr_lld(UARTLLD_Handle handle);

/**
 *  \brief  Function to enable loopback mode. This function is for internal use.
 *          Not recommended for customers to use.
 *
 *  \param  baseAddr    Memory address of the UART instance being used.
 *
 *  \sa     #UART_open
 */
void UART_enableLoopbackMode(uint32_t baseAddr);

/**
 *  \brief  Function to disable loopback mode. This function is for internal use.
 *          Not recommended for customers to use.
 *
 *  \param  baseAddr    Memory address of the UART instance being used.
 *
 *  \sa     #UART_open
 */
void UART_disableLoopbackMode(uint32_t baseAddr);

/**
 *  \brief  This API Initializes the UART instance
 *
 *  \param  hUart      Handle to the UART instance used
 *
 *  \return #SystemP_SUCCESS if successful; else error on failure
 *
 */
int32_t UART_lld_init(UARTLLD_Handle hUart);

/**
 *  \brief  This API Initializes the UART instance in DMA mode
 *
 *  \param  hUart      Handle to the UART instance used
 *
 *  \return #SystemP_SUCCESS if successful; else error on failure
 *
 */
int32_t UART_lld_initDma(UARTLLD_Handle hUart);

/**
 *  \brief  This API De-Initializes the UART instance
 *
 *  \param  hUart      Handle to the UART instance used
 *
 *  \return #SystemP_SUCCESS if successful; else error on failure
 *
 */
int32_t UART_lld_deInit(UARTLLD_Handle hUart);

/**
 *  \brief  This API De-Initializes the UART instance in DMA mode
 *
 *  \param  hUart      Handle to the UART instance used
 *
 *  \return #SystemP_SUCCESS if successful; else error on failure
 *
 */
int32_t UART_lld_deInitDma(UARTLLD_Handle hUart);

/**
 *  \brief  This API writes data to the UART instance in Polling mode.
 *
 *  \param  hUart           Handle to the UART instance used
 *  \param  txBuf           Pointer to write data buffer
 *  \param  size            Data size to be transferred
 *  \param  timeout         Write timeout value
 *  \param  extendedParams  Structure to hold the extended parameters
 *
 *  \return #SystemP_SUCCESS if successful; else error on failure
 *
 */
int32_t UART_lld_write(UARTLLD_Handle hUart, void * txBuf, uint32_t size, uint32_t timeout,
                       const UART_ExtendedParams *extendedParams);

/**
 *  \brief  This API writes data to the UART instance in Interrupt mode.
 *
 *  \param  hUart           Handle to the UART instance used
 *  \param  txBuf           Pointer to write data buffer
 *  \param  size            Data size to be transferred
 *  \param  extendedParams  Structure to hold the extended parameters
 *
 *  \return #SystemP_SUCCESS if successful; else error on failure
 *
 */
int32_t UART_lld_writeIntr(UARTLLD_Handle hUart, void * txBuf, uint32_t size,
                           const UART_ExtendedParams *extendedParams);

/**
 *  \brief  This API writes data to the UART instance in DMA mode.
 *
 *  \param  hUart           Handle to the UART instance used
 *  \param  txBuf           Pointer to write data buffer
 *  \param  size            Data size to be transferred
 *  \param  extendedParams  Structure to hold the extended parameters
 *
 *  \return #SystemP_SUCCESS if successful; else error on failure
 *
 */
int32_t UART_lld_writeDma(UARTLLD_Handle hUart, void * txBuf, uint32_t size,
                          const UART_ExtendedParams *extendedParams);

/**
 *  \brief  This API writes data to the UART instance in Polling mode.
 *
 *  \param  hUart           Handle to the UART instance used
 *  \param  rxBuf           Pointer to read data buffer
 *  \param  size            Data size to be transferred
 *  \param  timeout         Write timeout value
 *  \param  extendedParams  Structure to hold the extended parameters
 *
 *  \return #SystemP_SUCCESS if successful; else error on failure
 *
 */
int32_t UART_lld_read(UARTLLD_Handle hUart, void * rxBuf, uint32_t size, uint32_t timeout,
                      const UART_ExtendedParams *extendedParams);

/**
 *  \brief  This API writes data to the UART instance in Polling mode.
 *
 *  \param  hUart           Handle to the UART instance used
 *  \param  rxBuf           Pointer to read data buffer
 *  \param  size            Data size to be transferred
 *  \param  timeout         Write timeout value
 *  \param  extendedParams  Structure to hold the extended parameters
 *
 *  \return #SystemP_SUCCESS if successful; else error on failure
 *
 */
int32_t UART_lld_readWithCounter(UARTLLD_Handle hUart, void * rxBuf, uint32_t size, uint32_t timeout,
                      const UART_ExtendedParams *extendedParams);

/**
 *  \brief  This API writes data to the UART instance in Interrupt mode.
 *
 *  \param  hUart           Handle to the UART instance used
 *  \param  rxBuf           Pointer to read data buffer
 *  \param  size            Data size to be transferred
 *  \param  extendedParams  Structure to hold the extended parameters
 *
 *  \return #SystemP_SUCCESS if successful; else error on failure
 *
 */
int32_t UART_lld_readIntr(UARTLLD_Handle hUart, void * rxBuf, uint32_t size,
                         const UART_ExtendedParams *extendedParams);

/**
 *  \brief  This API writes data to the UART instance in DMA mode.
 *
 *  \param  hUart           Handle to the UART instance used
 *  \param  rxBuf           Pointer to read data buffer
 *  \param  size            Data size to be transferred
 *  \param  extendedParams  Structure to hold the extended parameters
 *
 *  \return #SystemP_SUCCESS if successful; else error on failure
 *
 */
int32_t UART_lld_readDma(UARTLLD_Handle hUart, void * rxBuf, uint32_t size,
                        const UART_ExtendedParams *extendedParams);

/**
 *  \brief  This API cancels current UART Write
 *
 *  \param  hUart   Handle to the UART instance used
 *  \param  trans   Pointer to #UART_Transaction structure
 *
 *  \return #SystemP_SUCCESS if successful; else error on failure
 *
 */
int32_t UART_lld_writeCancel(UARTLLD_Handle hUart, UART_Transaction *trans);

/**
 *  \brief  This API cancels current UART Read
 *
 *  \param  hUart   Handle to the UART instance used
 *  \param  trans   Pointer to #UART_Transaction structure
 *
 *  \return #SystemP_SUCCESS if successful; else error on failure
 *
 */
int32_t UART_lld_readCancel(UARTLLD_Handle hUart, UART_Transaction *trans);

/**
 *  \brief  Function to flush a TX FIFO of peripheral specified by the UART handle
 *
 *  \param  hUart      Handle to the UART instance used
 *
 *  \return #SystemP_SUCCESS if successful; else error on failure
 */
int32_t UART_lld_flushTxFifo(UARTLLD_Handle hUart);

/**
 *  \brief  Function to check various error conditions in uart
 *
 *  \param  hUart       Handle to the UART instance used
 *
 *  \return #SystemP_SUCCESS if successful; else error on failure
 */
int32_t UART_procLineStatusErr(UARTLLD_Handle hUart);

/**
 *  \brief   Function to enable/disable UART RX.
 *
 *  \param  hUart      Handle to the UART instance used
 *  \param  state      Enable/Disable UART interrupt
 *
 *  \return #SystemP_SUCCESS if successful; else error on failure
 */
int32_t UART_lld_setRxState(UARTLLD_Handle hUart, uint32_t state);

/**
 *  \brief  This is the UART ISR and can be used as IRQ handler.
 *
 *  \param  args      Argument to the ISR.
 *
 */
void UART_lld_controllerIsr(void* args);

/**
 * \brief   This API disables the specified interrupts in the UART mode of
 *          operation.
 *
 * \param   baseAddr   Memory address of the UART instance being used.
 * \param   intrFlag   Bit mask value of the bits corresponding to Interrupt
 *                    Enable Register(IER). This specifies the UART interrupts
 *                    to be disabled.
 *
 *  'intrFlag' can take one or a combination of the following macros:
 *  - UART_INTR_CTS - to disable Clear-To-Send interrupt,
 *  - UART_INTR_RTS - to disable Request-To-Send interrupt,
 *  - UART_INTR_XOFF - to disable XOFF interrupt,
 *  - UART_INTR_SLEEPMODE - to disable Sleep Mode,
 *  - UART_INTR_MODEM_STAT - to disable Modem Status interrupt,
 *  - UART_INTR_LINE_STAT - to disable Line Status interrupt,
 *  - UART_INTR_THR - to disable Transmitter Holding Register Empty interrupt,
 *  - UART_INTR_RHR_CTI - to disable Receiver Data available interrupt and
 *                       Character timeout indication interrupt.
 *
 * \note  The note section of UART_intrEnable() also applies to this API.
 *
 * \sa      #UART_open
 */
void UART_intrDisable(uint32_t baseAddr, uint32_t intrFlag);

/**
 * \brief   This API disables the specified interrupts in the UART mode of
 *          operation  for IER2
 *
 * \param   baseAddr  Memory address of the UART instance being used.
 * \param   intrFlag   Bit mask value of the bits corresponding to Interrupt
 *                    Enable Register(IER2). This specifies the UART interrupts
 *                    to be disabled.
 *
 *  'intrFlag' can take one or a combination of the following macros:
 *  - UART_INTR2_RX_EMPTY - to enable receive FIFO empty interrupt
 *  - UART_INTR2_TX_EMPTY - to enable TX FIFO empty interrupt
 *
 * \note  The note section of UART_intr2Enable() also applies to this API.
 *
 * \sa      #UART_open
 */
void UART_intr2Disable(uint32_t baseAddr, uint32_t intrFlag);

/**
 * \brief  This API checks if the RX FIFO (or RHR in non-FIFO mode) has atleast
 *         one byte of data to be read.
 *
 * \param  baseAddr    Memory address of the UART instance being used.
 *
 *
 * \return  TRUE - if there is atleast one data byte present in the RX FIFO
 *          (or RHR in non-FIFO mode)\n
 *          FALSE - if there are no data bytes present in the RX FIFO(or RHR
 *           in non-FIFO mode)\n
 *
 * \sa      #UART_open
 */
uint32_t UART_checkCharsAvailInFifo(uint32_t baseAddr);

/**
 * \brief  This API reads the line status register value.
 *
 * \param  baseAddr     Memory address of the UART instance being used.
 *
 *
 * \return This returns the line status register value.
 *
 * \sa      #UART_open
 */
uint32_t UART_readLineStatus(uint32_t baseAddr);

/**
 * \brief   This API reads the data present at the top of the RX FIFO, that
 *          is, the data in the Receive Holding Register(RHR). However
 *          before reading the data from RHR, it checks for RX error.
 *
 * \param   baseAddr     Memory address of the UART instance being used.
 * \param   readBuf      Pointer to the byte buffer to be read from RHR register.
 *
 *
 * \return  The data read from the RHR.
 *
 * \sa      #UART_open
 */
uint8_t UART_getCharFifo(uint32_t baseAddr, uint8_t *readBuf);

/**
 * \brief   This API writes a byte to the Transmitter FIFO without checking for
 *          the emptiness of the Transmitter FIFO or the Transmitter Shift
 *          Register(TSR).
 *
 * \param   baseAddr    Memory address of the UART instance being used.
 * \param   byteTx      The byte to be transmitted by the UART.
 *
 * \note    Unlike the APIs UARTCharPut() or UARTCharPutNonBlocking(), this
 *          API does not check for the emptiness of the TX FIFO or TSR. This
 *          API is ideal for use in FIFO mode of operation where the 64-byte
 *          TX FIFO has to be written with successive bytes. If transmit
 *          interrupt is enabled, it provides a mechanism to control the
 *          writes to the TX FIFO.
 *
 * \sa      #UART_open
 */
void UART_putChar(uint32_t baseAddr, uint8_t byteTx);

/**
 * \brief   This API enables the specified interrupts in the UART mode of
 *          operation.
 *
 * \param   baseAddr   Memory address of the UART instance being used.
 * \param   intrFlag   Bit mask value of the bits corresponding to Interrupt
 *                    Enable Register(IER). This specifies the UART interrupts
 *                    to be enabled.
 *
 *  'intrFlag' can take one or a combination of the following macros:
 *  - UART_INTR_CTS - to enable Clear-To-Send interrupt,
 *  - UART_INTR_RTS - to enable Request-To-Send interrupt,
 *  - UART_INTR_XOFF - to enable XOFF interrupt,
 *  - UART_INTR_SLEEPMODE - to enable Sleep Mode,
 *  - UART_INTR_MODEM_STAT - to enable Modem Status interrupt,
 *  - UART_INTR_LINE_STAT - to enable Line Status interrupt,
 *  - UART_INTR_THR - to enable Transmitter Holding Register Empty interrupt,
 *  - UART_INTR_RHR_CTI - to enable Receiver Data available interrupt and
 *                       Character timeout indication interrupt.
 *
 * \note    This API modifies the contents of UART Interrupt Enable Register
 *          (IER). Modifying the bits IER[7:4] requires that EFR[4] be set.
 *          This API does the needful before it accesses IER.
 *          Moreover, this API should be called when UART is operating in
 *          UART 16x Mode, UART 13x Mode or UART 16x Auto-baud mode.\n
 *
 * \sa      #UART_open
 */
void UART_intrEnable(uint32_t baseAddr, uint32_t intrFlag);

/**
 * \brief    This API reads a byte from the Receiver Buffer Register
 *           (RBR). It checks once if any character is ready to be read.
 *
 * \param    baseAddr     Memory address of the UART instance being used.
 *
 * \param    pChar        Pointer to the byte variable which saves the byte
 *                        read from RBR if there is any char ready to be read
 *
 * \return   If the RX FIFO(or RHR) was found to have atleast one byte of
 *           data, then this API returns TRUE. Else it returns FALSE.
 *
 * \sa      #UART_open
 */
uint32_t UART_getChar(uint32_t baseAddr, uint8_t *pChar);

/**
 * \brief  This API determines the UART Interrupt Status 2.
 *
 * \param  baseAddr   Memory address of the UART instance being used.
 *
 *
 * \return This returns one or a combination of the following macros:
 *         - UART_INTR2_RX_EMPTY - to enable receive FIFO empty interrupt\n
 *         - UART_INTR2_TX_EMPTY - to enable TX FIFO empty interrupt\n
 *
 * \sa      #UART_open
 */
uint32_t UART_getIntr2Status(uint32_t baseAddr);

/**
 * \brief  This API determines the UART Interrupt Status.
 *
 * \param  baseAddr   Memory address of the UART instance being used.
 *
 *
 * \return This returns one or a combination of the following macros:
 *         - UART_INTID_MODEM_STAT - indicating the occurence of a Modem Status
 *           interrupt\n
 *         - UART_INTID_TX_THRES_REACH - indicating that the TX FIFO Threshold
 *           number of bytes can be written to the TX FIFO.
 *         - UART_INTID_RX_THRES_REACH - indicating that the RX FIFO has
 *           reached its programmed Trigger Level\n
 *         - UART_INTID_RX_LINE_STAT_ERROR - indicating the occurence of a
 *           receiver Line Status error\n
 *         - UART_INTID_CHAR_TIMEOUT - indicating the occurence of a Receiver
 *           Timeout\n
 *         - UART_INTID_XOFF_SPEC_CHAR_DETECT - indicating the detection of XOFF
 *           or a Special character\n
 *         - UART_INTID_MODEM_SIG_STATE_CHANGE - indicating that atleast one of
 *           the Modem signals among CTSn, RTSn and DSRn have changed states
 *           from active(low) to inactive(high)\n
 *
 * \sa      #UART_open
 */
uint32_t UART_getIntrIdentityStatus(uint32_t baseAddr);

/**
 * \brief   This API enables the specified interrupts in the UART mode of
 *          operation for IER2
 *
 * \param   baseAddr  Memory address of the UART instance being used.
 * \param   intrFlag   Bit mask value of the bits corresponding to Interrupt
 *                    Enable Register(IER2). This specifies the UART interrupts
 *                    to be enabled.
 *
 *          'intrFlag' can take one or a combination of the following macros:
 *          - UART_INTR2_RX_EMPTY - to enable receive FIFO empty interrupt
 *          - UART_INTR2_TX_EMPTY - to enable TX FIFO empty interrupt
 *
 * \note    This API modifies the contents of UART Interrupt Enable Register 2
 *          (IER2).
 *
 * \sa      #UART_open
 */
void UART_intr2Enable(uint32_t baseAddr, uint32_t intrFlag);

/**
 *  \brief  Function to initialize the #UART_Transaction struct to its defaults
 *
 *  \param  trans       Pointer to #UART_Transaction structure for
 *                      initialization
 */
void UART_lld_Transaction_init(UART_Transaction *trans);

/**
 *  \brief  Function to de-initialize the #UART_Transaction struct to its defaults
 *
 *  \param  trans       Pointer to #UART_Transaction structure for
 *                      initialization
 */
void UART_lld_Transaction_deInit(UART_Transaction *trans);

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef UART_V0_LLD_H_ */

/** @} */
