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
 *  \defgroup DRV_UART_MODULE APIs for UART
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the UART.
 *
 *  @{
 */

/**
 *  \file v0/uart.h
 *
 *  \brief This file contains the prototype of UART driver APIs
 */

#ifndef UART_V0_H_
#define UART_V0_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/cslr_uart.h>
#include <drivers/hw_include/hw_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

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

/** \brief A handle that is returned from a #UART_open() call */
typedef void *UART_Handle;

/**
 *  \anchor UART_TransferStatus
 *  \name Transfer Status Code
 *
 *  Status codes that are set by the UART driver
 *
 *  @{
 */
/** \brief Transaction success */
#define UART_TRANSFER_STATUS_SUCCESS         (0U)
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
 *  \anchor UART_TransferMode
 *  \name Transfer Mode
 *
 *  This determines whether the driver operates synchronously or asynchronously
 *
 *  In #UART_TRANSFER_MODE_BLOCKING mode #UART_read() and #UART_write() blocks
 *  code execution until the transaction has completed
 *
 *  In #UART_TRANSFER_MODE_CALLBACK #UART_read() and #UART_write() does not
 *  block code execution and instead calls a #UART_CallbackFxn callback
 *  function when the transaction has completed
 *
 *  @{
 */
/**
 *  \brief UART read/write APIs blocks execution. This mode can only be used
 *  when called within a Task context
 */
#define UART_TRANSFER_MODE_BLOCKING     (0U)
/**
 *  \brief UART read/write APIs does not block code execution and will call a
 *  #UART_CallbackFxn. This mode can be used in a Task, Swi, or Hwi context
 */
#define UART_TRANSFER_MODE_CALLBACK     (1U)
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
 *  \anchor UART_StopBits
 *  \name UART stop bits
 *
 *  Note: The values should not be changed since it represents the
 *        actual register configuration values used to configure the UART
 *  @{
 */
#define UART_STOPBITS_1                 (0U)
#define UART_STOPBITS_2                 (1U)
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
/** @} */
/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

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

/**
 *  \brief  The definition of a callback function used by the UART driver
 *  when used in #UART_TRANSFER_MODE_CALLBACK
 *
 *  \param handle          UART_Handle
 *  \param transaction*    Pointer to a #UART_Transaction
 */
typedef void (*UART_CallbackFxn) (UART_Handle handle,
                                  UART_Transaction *transaction);

/**
 *  \brief UART Parameters
 *
 *  UART Parameters are used to with the #UART_open() call. Default values for
 *  these parameters are set using #UART_Params_init().
 *
 *  If NULL is passed for the parameters, #UART_open() uses default parameters.
 *
 *  \sa #UART_Params_init()
 */
typedef struct
{
    uint32_t                baudRate;
    /**< Baud rate for UART */
    uint32_t                dataLength;
    /**< Data length for UART. Refer \ref UART_DataLength */
    uint32_t                stopBits;
    /**< Stop bits for UART. Refer \ref UART_StopBits */
    uint32_t                parityType;
    /**< Parity bit type for UART. Refer \ref UART_Parity */
    uint32_t                readMode;
    /**< Read blocking or Callback mode. Refer \ref UART_TransferMode */
    uint32_t                readReturnMode;
    /**< Receive return mode  Refer \ref UART_ReadReturnMode */
    uint32_t                writeMode;
    /**< Write blocking or Callback mode. Refer \ref UART_TransferMode */
    UART_CallbackFxn        readCallbackFxn;
    /**< Read callback function pointer */
    UART_CallbackFxn        writeCallbackFxn;
    /**< Write callback function pointer */
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
    uint32_t                skipIntrReg;
    /**< Skips Driver registering interrupt */
    int32_t                 uartDmaIndex;
    /**< Index of DMA instance used by UART Driver. This index will be set by SysCfg according to the DMA driver chosen.
     * The UART driver uses this index to do an \ref UART_dmaOpen inside the \ref UART_open if the DMA mode is enabled
     */

    /*
     * UART configuration
     */
    uint32_t                operMode;
    /**< Refer \ref UART_OperMode for valid values */
    uint32_t                rxTrigLvl;
    /**< Refer \ref UART_RxTrigLvl for valid values */
    uint32_t                txTrigLvl;
    /**< Refer \ref UART_TxTrigLvl for valid values */
    uint32_t                rxEvtNum;
    /**< DMA Event number used for UART Rx */
    uint32_t                txEvtNum;
    /**< DMA Event number used for UART Tx */
} UART_Params;

/** \brief UART instance attributes - used during init time */
typedef struct
{
    /*
     * SOC configuration
     */
    uint32_t                baseAddr;
    /**< Peripheral base address */
    uint32_t                inputClkFreq;
    /**< Module input clock frequency */
} UART_Attrs;

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/**
 *  \brief UART driver object
 */
typedef struct
{
    /*
     * User parameters
     */
    UART_Handle             handle;
    /**< Instance handle to which this object belongs */
    UART_Params             prms;
    /**< Open parameter as provided by user */
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
    UART_Transaction       *readTrans;
    /**< Pointer to the current read transaction */
    UART_Transaction       *writeTrans;
    /**< Pointer to the current write transaction */
    /*
     * State variables
     */
    uint32_t                isOpen;
    /**< Flag to indicate whether the instance is opened already */
    void                   *lock;
    /**< Instance lock - to protect across transfers */
    SemaphoreP_Object       lockObj;
    /**< Driver lock object */
    void                   *readTransferSem;
    /**< Read Transfer Sync Sempahore - to sync between transfer completion ISR
     *   and task */
    SemaphoreP_Object       readTransferSemObj;
    /**< Read Transfer Sync Sempahore object */
    void                   *writeTransferSem;
    /**< Write Transfer Sync Sempahore - to sync between transfer completion ISR
     *   and task */
    SemaphoreP_Object       writeTransferSemObj;
    /**< Write Transfer Sync Sempahore object */
    void                   *hwiHandle;
    /**< Interrupt handle for master ISR */
    HwiP_Object             hwiObj;
    /**< Interrupt object */
    void* uartDmaHandle;
    /**< Pointer to current transaction struct */
} UART_Object;

/**
 *  \brief UART global configuration array
 *
 *  This structure needs to be defined before calling #UART_init() and it must
 *  not be changed by user thereafter.
 */
typedef struct
{
    UART_Attrs       *attrs;
    /**< Pointer to driver specific attributes */
    UART_Object      *object;
    /**< Pointer to driver specific data object */
} UART_Config;

/** \brief Externally defined driver configuration array */
extern UART_Config gUartConfig[];
/** \brief Externally defined driver configuration array size */
extern uint32_t    gUartConfigNum;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief  This function initializes the UART module
 */
void UART_init(void);

/**
 *  \brief  This function de-initializes the UART module
 */
void UART_deinit(void);

/**
 *  \brief  This function opens a given UART peripheral
 *
 *  \pre    UART controller has been initialized using #UART_init()
 *
 *  \param  index       Index of config to use in the *UART_Config* array
 *  \param  prms        Pointer to open parameters. If NULL is passed, then
 *                      default values will be used
 *
 *  \return A #UART_Handle on success or a NULL on an error or if it has been
 *          opened already
 *
 *  \sa     #UART_init()
 *  \sa     #UART_close()
 *  \sa     #UART_Params_init
 */
UART_Handle UART_open(uint32_t index, const UART_Params *prms);

/**
 *  \brief  Function to close a UART peripheral specified by the UART handle
 *
 *  \pre    #UART_open() has to be called first
 *
 *  \param  handle      #UART_Handle returned from #UART_open()
 *
 *  \sa     #UART_open()
 */
void UART_close(UART_Handle handle);

/**
 *  \brief  Function to perform UART write operation
 *
 *  In #UART_TRANSFER_MODE_BLOCKING, #UART_write() will block task
 *  execution until the transaction has completed or a timeout has occurred.
 *
 *  In #UART_TRANSFER_MODE_CALLBACK, #UART_write() does not block
 *  task execution, but calls a #UART_CallbackFxn once the transfer
 *  has finished. This makes #UART_write() safe to be used within a Task,
 *  software or hardware interrupt context.
 *
 *  In interrupt mode, #UART_write() does not wait until tx fifo is empty.
 *  Application needs to call #UART_flushTxFifo() to ensure write is completed.
 *  i.e. data is out from the FIFO and shift registers.
 *
 *  From calling #UART_write() until transfer completion, the
 *  #UART_Transaction structure must stay persistent and must not be altered
 *  by application code.
 *  It is also forbidden to modify the content of the #UART_Transaction.buf
 *  during a transaction, even though the physical transfer might not have
 *  started yet. Doing this can result in data corruption.
 *
 *  \param  handle      #UART_Handle returned from #UART_open()
 *  \param  trans       Pointer to a #UART_Transaction. All of the fields
 *                      within transaction except #UART_Transaction.count and
 *                      #UART_Transaction.status are WO (write-only) unless
 *                      otherwise noted in the driver implementations. If a
 *                      transaction timeout has occurred,
 *                      #UART_Transaction.count will contain the number of
 *                      bytes that were transferred. Neither is it allowed to
 *                      modify the transaction object nor the content of
 *                      #UART_Transaction.buf until the transfer
 *                      has completed
 *
 *  \return #SystemP_SUCCESS if started successfully; else error on failure
 *
 *  \sa     #UART_open
 */
int32_t UART_write(UART_Handle handle, UART_Transaction *trans);

/**
 *  \brief  Function to perform UART read operation
 *
 *  In #UART_TRANSFER_MODE_BLOCKING, #UART_read() will block task
 *  execution until the transaction has completed or a timeout has occurred.
 *
 *  In #UART_TRANSFER_MODE_CALLBACK, #UART_read() does not block
 *  task execution, but calls a #UART_CallbackFxn once the transfer
 *  has finished. This makes #UART_read() safe to be used within a Task,
 *  software or hardware interrupt context.
 *
 *  From calling #UART_read() until transfer completion, the
 *  #UART_Transaction structure must stay persistent and must not be altered
 *  by application code.
 *  It is also forbidden to modify the content of the #UART_Transaction.buf
 *  during a transaction, even though the physical transfer might not have
 *  started yet. Doing this can result in data corruption.
 *
 *  \param  handle      #UART_Handle returned from #UART_open()
 *  \param  trans       Pointer to a #UART_Transaction. All of the fields
 *                      within transaction except #UART_Transaction.count and
 *                      #UART_Transaction.status are WO (write-only) unless
 *                      otherwise noted in the driver implementations. If a
 *                      transaction timeout has occurred,
 *                      #UART_Transaction.count will contain the number of
 *                      bytes that were transferred. Neither is it allowed to
 *                      modify the transaction object nor the content of
 *                      #UART_Transaction.buf until the transfer
 *                      has completed
 *
 *  \return #SystemP_SUCCESS if started successfully; else error on failure
 *
 *  \sa     #UART_open
 */
int32_t UART_read(UART_Handle handle, UART_Transaction *trans);

/**
 *  \brief  Function to perform UART canceling of current write transaction.
 *
 *  In #UART_TRANSFER_MODE_CALLBACK, #UART_writeCancel() does not block
 *  task execution, but calls a #UART_CallbackFxn once the cancel
 *  has finished. This makes #UART_writeCancel() safe to be used within a Task,
 *  software or hardware interrupt context.
 *
 *  From calling #UART_writeCancel() until cancel completion, the
 *  #UART_Transaction structure must stay persistent and must not be altered
 *  by application code.
 *  It is also forbidden to modify the content of the #UART_Transaction.buf
 *  during a transaction, even though the physical transfer might not have
 *  started yet. Doing this can result in data corruption.
 *
 *  \param  handle      #UART_Handle returned from #UART_open()
 *  \param  trans       Pointer to a #UART_Transaction. All of the fields
 *                      within transaction except #UART_Transaction.count and
 *                      #UART_Transaction.status are WO (write-only) unless
 *                      otherwise noted in the driver implementations. If a
 *                      transaction timeout has occurred,
 *                      #UART_Transaction.count will contain the number of
 *                      bytes that were transferred. Neither is it allowed to
 *                      modify the transaction object nor the content of
 *                      #UART_Transaction.buf until the transfer
 *                      has completed
 *
 *  \return #SystemP_SUCCESS if started successfully; else error on failure
 *
 *  \sa     #UART_open
 */
int32_t UART_writeCancel(UART_Handle handle, UART_Transaction *trans);

/**
 *  \brief  Function to perform UART canceling of current read transaction
 *
 *  In #UART_TRANSFER_MODE_CALLBACK, #UART_readCancel() does not block
 *  task execution, but calls a #UART_CallbackFxn once the cancel
 *  has finished. This makes #UART_writeCancel() safe to be used within a Task,
 *  software or hardware interrupt context.
 *
 *  From calling #UART_readCancel() until cancel completion, the
 *  #UART_Transaction structure must stay persistent and must not be altered
 *  by application code.
 *  It is also forbidden to modify the content of the #UART_Transaction.buf
 *  during a transaction, even though the physical transfer might not have
 *  started yet. Doing this can result in data corruption.
 *
 *  \param  handle      #UART_Handle returned from #UART_open()
 *  \param  trans       Pointer to a #UART_Transaction. All of the fields
 *                      within transaction except #UART_Transaction.count and
 *                      #UART_Transaction.status are WO (write-only) unless
 *                      otherwise noted in the driver implementations. If a
 *                      transaction timeout has occurred,
 *                      #UART_Transaction.count will contain the number of
 *                      bytes that were transferred. Neither is it allowed to
 *                      modify the transaction object nor the content of
 *                      #UART_Transaction.buf until the transfer
 *                      has completed
 *
 *  \return #SystemP_SUCCESS if started successfully; else error on failure
 *
 *  \sa     #UART_open
 */
int32_t UART_readCancel(UART_Handle handle, UART_Transaction *trans);

/**
 *  \brief  Function to return a open'ed UART handle given a UART instance index
 *
 *  \param  index       Index of config to use in the *UART_Config* array
 *
 *  \return A #UART_Handle on success or a NULL on an error or if the instance
 *            index has  NOT been opened yet
 */
UART_Handle UART_getHandle(uint32_t index);

/**
 *  \brief  Function to flush a TX FIFO of peripheral specified by the UART handle
 *
 *  \pre    #UART_open() has to be called first
 *
 *  \param  handle      #UART_Handle returned from #UART_open()
 *
 *  \sa     #UART_open()
 */
void UART_flushTxFifo(UART_Handle handle);

/**
 *  \brief  Function to initialize the #UART_Params struct to its defaults
 *
 *  \param  prms        Pointer to #UART_Params structure for initialization
 */
static inline void UART_Params_init(UART_Params *prms);

/**
 *  \brief  Function to initialize the #UART_Transaction struct to its defaults
 *
 *  \param  trans       Pointer to #UART_Transaction structure for
 *                      initialization
 */
static inline void UART_Transaction_init(UART_Transaction *trans);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

static inline void UART_Params_init(UART_Params *prms)
{
    if(prms != NULL)
    {
        prms->baudRate           = 115200U;
        prms->dataLength         = UART_LEN_8;
        prms->stopBits           = UART_STOPBITS_1;
        prms->parityType         = UART_PARITY_NONE;
        prms->readMode           = UART_TRANSFER_MODE_BLOCKING;
        prms->readReturnMode     = UART_READ_RETURN_MODE_FULL;
        prms->writeMode          = UART_TRANSFER_MODE_BLOCKING;
        prms->readCallbackFxn    = NULL;
        prms->writeCallbackFxn   = NULL;
        prms->hwFlowControl      = FALSE;
        prms->hwFlowControlThr   = UART_RXTRIGLVL_16;
        prms->intrNum            = 0xFFFF;
        prms->transferMode       = UART_CONFIG_MODE_INTERRUPT;
        prms->intrPriority       = 4U;
        prms->skipIntrReg        = FALSE;
        prms->uartDmaIndex       = -1;
        prms->operMode           = UART_OPER_MODE_16X;
        prms->rxTrigLvl          = UART_RXTRIGLVL_8;
        prms->txTrigLvl          = UART_TXTRIGLVL_32;
    }
}

static inline void UART_Transaction_init(UART_Transaction *trans)
{
    if(trans != NULL)
    {
        trans->buf              = NULL;
        trans->count            = 0U;
        trans->timeout          = SystemP_WAIT_FOREVER;
        trans->status           = UART_TRANSFER_STATUS_SUCCESS;
        trans->args             = NULL;
    }
}

/* ========================================================================== */
/*                       Advanced Function Declarations                       */
/* ========================================================================== */
/**
 *  \brief  Function to get base address of UART instance of a particular
 *          handle.
 *
 *  \param  handle      #UART_Handle returned from #UART_open()
 *
 *  \sa     #UART_open
 */
uint32_t UART_getBaseAddr(UART_Handle handle);

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
static inline void UART_putChar(uint32_t baseAddr, uint8_t byteTx);

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
static inline uint32_t UART_getChar(uint32_t baseAddr, uint8_t *pChar);

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
static inline void UART_intrEnable(uint32_t baseAddr, uint32_t intrFlag);

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
static inline void UART_intrDisable(uint32_t baseAddr, uint32_t intrFlag);

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
static inline void UART_intr2Enable(uint32_t baseAddr, uint32_t intrFlag);

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
static inline void UART_intr2Disable(uint32_t baseAddr, uint32_t intrFlag);

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
static inline uint32_t UART_getIntrIdentityStatus(uint32_t baseAddr);

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
static inline uint32_t UART_getIntr2Status(uint32_t baseAddr);

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
static inline uint32_t UART_checkCharsAvailInFifo(uint32_t baseAddr);

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
static inline uint32_t UART_readLineStatus(uint32_t baseAddr);

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
static inline uint8_t UART_getCharFifo(uint32_t baseAddr, uint8_t *readBuf);
/* ========================================================================== */
/*                       Advanced Function Definitions                        */
/* ========================================================================== */
static inline void UART_putChar(uint32_t baseAddr, uint8_t byteTx)
{
    /* Write the byte to the Transmit Holding Register(or TX FIFO). */
    HW_WR_REG32(baseAddr + UART_THR, (uint32_t) byteTx);
}

static inline uint32_t UART_getChar(uint32_t baseAddr, uint8_t *pChar)
{
    uint32_t lcrRegValue = 0U;
    uint32_t retVal      = FALSE;

    /* Preserving the current value of LCR. */
    lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);

    /* Switching to Register Operational Mode of operation. */
    HW_WR_REG32(baseAddr + UART_LCR, HW_RD_REG32(baseAddr + UART_LCR)
                            & 0x7FU);

    /* Checking if the RX FIFO(or RHR) has atleast one byte of data. */
    if ((uint32_t) UART_LSR_RX_FIFO_E_RX_FIFO_E_VALUE_0 !=
        (HW_RD_REG32(baseAddr + UART_LSR) &
         UART_LSR_RX_FIFO_E_MASK))
    {
        uint32_t tempRetVal = HW_RD_REG32(baseAddr + UART_RHR);
        *pChar = (uint8_t)tempRetVal;
        retVal = TRUE;
    }

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    return retVal;
}

static inline void UART_intrEnable(uint32_t baseAddr, uint32_t intrFlag)
{
    uint32_t enhanFnBitVal = 0U;
    uint32_t lcrRegValue   = 0U;

    /* Switch to mode B only when the upper 4 bits of IER needs to be changed */
    if ((intrFlag & 0xF0U) > 0U)
    {
        /* Preserving the current value of LCR. */
        lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);
        /* Switching to Register Configuration Mode B. */
        HW_WR_REG32(baseAddr + UART_LCR, UART_REG_CONFIG_MODE_B & 0xFFU);

        /* Collecting the current value of EFR[4] and later setting it. */
        enhanFnBitVal = HW_RD_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN);

        HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN,
                      UART_EFR_ENHANCED_EN_ENHANCED_EN_U_VALUE_1);

        /* Restoring the value of LCR. */
        HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

        /* Preserving the current value of LCR. */
        lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);

        /* Switching to Register Operational Mode of operation. */
        HW_WR_REG32(baseAddr + UART_LCR, HW_RD_REG32(baseAddr + UART_LCR)
                                & 0x7FU);

        /*
        ** It is suggested that the System Interrupts for UART in the
        ** Interrupt Controller are enabled after enabling the peripheral
        ** interrupts of the UART using this API. If done otherwise, there
        ** is a risk of LCR value not getting restored and illicit characters
        ** transmitted or received from/to the UART. The situation is explained
        ** below.
        ** The scene is that the system interrupt for UART is already enabled
        ** and the current API is invoked. On enabling the interrupts
        ** corresponding to IER[7:4] bits below, if any of those interrupt
        ** conditions already existed, there is a possibility that the control
        ** goes to Interrupt Service Routine (ISR) without executing the
        ** remaining statements in this API. Executing the remaining statements
        ** is critical in that the LCR value is restored in them.
        ** However, there seems to be no risk in this API for enabling
        ** interrupts corresponding to IER[3:0] because it is done at the end
        ** and no statements follow that.
        */

        /************* ATOMIC STATEMENTS START *************************/

        /* Programming the bits IER[7:4]. */
        HW_WR_REG32(baseAddr + UART_IER, intrFlag & 0xF0U);

        /* Restoring the value of LCR. */
        HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

        /* Preserving the current value of LCR. */
        lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);
        /* Switching to Register Configuration Mode B. */
        HW_WR_REG32(baseAddr + UART_LCR, UART_REG_CONFIG_MODE_B & 0xFFU);

        /* Restoring the value of EFR[4] to its original value. */
        HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN, enhanFnBitVal);

        /* Restoring the value of LCR. */
        HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

        /************** ATOMIC STATEMENTS END *************************/
    }

    /* Programming the bits IER[3:0]. */
    HW_WR_REG32(baseAddr + UART_IER, HW_RD_REG32(baseAddr + UART_IER) |
                (intrFlag & 0x0FU));
}

static inline void UART_intrDisable(uint32_t baseAddr, uint32_t intrFlag)
{
    uint32_t enhanFnBitVal;
    uint32_t lcrRegValue;

    /* Switch to mode B only when the upper 4 bits of IER needs to be changed */
    if((intrFlag & 0xF0U) > 0U)
    {
        /* Preserving the current value of LCR. */
        lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);
        /* Switching to Register Configuration Mode B. */
        HW_WR_REG32(baseAddr + UART_LCR, UART_REG_CONFIG_MODE_B & 0xFFU);

        /* Collecting the current value of EFR[4] and later setting it. */
        enhanFnBitVal = HW_RD_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN);

        HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN,
                      UART_EFR_ENHANCED_EN_ENHANCED_EN_U_VALUE_1);

        /* Restoring the value of LCR. */
        HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);
    }

    /* Preserving the current value of LCR. */
    lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);

    /* Switching to Register Operational Mode of operation. */
    HW_WR_REG32(baseAddr + UART_LCR, HW_RD_REG32(baseAddr + UART_LCR)
                            & 0x7FU);

    HW_WR_REG32(baseAddr + UART_IER, HW_RD_REG32(baseAddr + UART_IER) &
                ~(intrFlag & 0xFFU));

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    /* Switch to mode B only when the upper 4 bits of IER needs to be changed */
    if((intrFlag & 0xF0U) > 0U)
    {
        /* Preserving the current value of LCR. */
        lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);
        /* Switching to Register Configuration Mode B. */
        HW_WR_REG32(baseAddr + UART_LCR, UART_REG_CONFIG_MODE_B & 0xFFU);

        /* Restoring the value of EFR[4] to its original value. */
        HW_WR_FIELD32(baseAddr + UART_EFR, UART_EFR_ENHANCED_EN, enhanFnBitVal);

        /* Restoring the value of LCR. */
        HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);
    }
}

static inline void UART_intr2Enable(uint32_t baseAddr, uint32_t intrFlag)
{
    /* Programming the bits IER2[1:0]. */
    HW_WR_REG32(baseAddr + UART_IER2, HW_RD_REG32(baseAddr + UART_IER2) |
                (intrFlag & 0x03U));
}

static inline void UART_intr2Disable(uint32_t baseAddr, uint32_t intrFlag)
{
    HW_WR_REG32(baseAddr + UART_IER2, HW_RD_REG32(baseAddr + UART_IER2) &
                ~(intrFlag & 0x3U));
}

static inline uint32_t UART_getIntrIdentityStatus(uint32_t baseAddr)
{
    uint32_t lcrRegValue = 0U;
    uint32_t retVal      = 0U;

    /* Preserving the current value of LCR. */
    lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);

    /* Switching to Register Operational Mode of operation. */
    HW_WR_REG32(baseAddr + UART_LCR, HW_RD_REG32(baseAddr + UART_LCR)
                            & 0x7FU);

    retVal = HW_RD_REG32(baseAddr + UART_IIR) & UART_IIR_IT_TYPE_MASK;

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    return retVal;
}

static inline uint32_t UART_getIntr2Status(uint32_t baseAddr)
{
    uint32_t retVal = 0U;

    retVal = HW_RD_REG32(baseAddr + UART_ISR2) &
        (UART_IER2_EN_RXFIFO_EMPTY_MASK | UART_IER2_EN_TXFIFO_EMPTY_MASK);

    return retVal;
}

static inline uint32_t UART_checkCharsAvailInFifo(uint32_t baseAddr)
{
    uint32_t lcrRegValue = 0;
    uint32_t retVal      = FALSE;

    /* Preserving the current value of LCR. */
    lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);

    /* Switching to Register Operational Mode of operation. */
    HW_WR_REG32(baseAddr + UART_LCR, HW_RD_REG32(baseAddr + UART_LCR)
                            & 0x7FU);

    /* Checking if the RHR(or RX FIFO) has atleast one byte to be read. */
    if ((uint32_t) UART_LSR_RX_FIFO_E_RX_FIFO_E_VALUE_0 !=
        (HW_RD_REG32(baseAddr + UART_LSR) &
         UART_LSR_RX_FIFO_E_MASK))
    {
        retVal = (uint32_t) TRUE;
    }

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    return retVal;
}

static inline uint32_t UART_readLineStatus(uint32_t baseAddr)
{
    uint32_t lcrRegValue = 0U;
    uint32_t retVal      = 0U;

    /* Preserving the current value of LCR. */
    lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);

    /* Switching to Register Operational Mode of operation. */
    HW_WR_REG32(baseAddr + UART_LCR, HW_RD_REG32(baseAddr + UART_LCR)
                            & 0x7FU);

    retVal = HW_RD_REG32(baseAddr + UART_LSR);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    return retVal;
}

static inline uint8_t UART_getCharFifo(uint32_t baseAddr, uint8_t *readBuf)
{
    uint8_t           readByte = 0;
    uint32_t          waitCount = UART_ERROR_COUNT;
    uint32_t          errorVal;
    uint32_t          lcrRegValue = 0;

    /* Preserving the current value of LCR. */
    lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);

    /* Switching to Register Operational Mode of operation. */
    HW_WR_REG32(baseAddr + UART_LCR, HW_RD_REG32(baseAddr + UART_LCR)
                            & 0x7FU);

    /* Read Rx Error Status */
    errorVal = HW_RD_REG32(baseAddr + UART_LSR) &
             (UART_LSR_RX_FIFO_STS_MASK |
              UART_LSR_RX_BI_MASK |
              UART_LSR_RX_FE_MASK |
              UART_LSR_RX_PE_MASK |
              UART_LSR_RX_OE_MASK);

    /* Restoring the value of LCR. */
    HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);

    /* Read and throw Erroneous bytes from RxFIFO */
    while ((UART_LSR_RX_FIFO_STS_MASK |
            UART_LSR_RX_BI_MASK |
            UART_LSR_RX_FE_MASK |
            UART_LSR_RX_PE_MASK |
            UART_LSR_RX_OE_MASK) == errorVal)
    {
        readByte = HW_RD_REG32(baseAddr + UART_RHR);
        waitCount--;
        if (0U == waitCount)
        {
            break;
        }

        /* Preserving the current value of LCR. */
        lcrRegValue = HW_RD_REG32(baseAddr + UART_LCR);

        /* Switching to Register Operational Mode of operation. */
        HW_WR_REG32(baseAddr + UART_LCR, HW_RD_REG32(baseAddr + UART_LCR)
                                & 0x7FU);

        /* Read Rx Error Status */
        errorVal = HW_RD_REG32(baseAddr + UART_LSR) &
                 (UART_LSR_RX_FIFO_STS_MASK |
                  UART_LSR_RX_BI_MASK |
                  UART_LSR_RX_FE_MASK |
                  UART_LSR_RX_PE_MASK |
                  UART_LSR_RX_OE_MASK);

        /* Restoring the value of LCR. */
        HW_WR_REG32(baseAddr + UART_LCR, lcrRegValue);
    }

    /* Read non-erroneous byte from RxFIFO */
    readByte = HW_RD_REG32(baseAddr + UART_RHR);

    return readByte;
}

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef UART_V0_H_ */

/** @} */
