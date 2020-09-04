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
 *  \file v1/uart_sci.h
 *
 *  \brief This file contains the prototype of UART driver APIs
 */

#ifndef UART_SCI_H_
#define UART_SCI_H_

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
#include <drivers/edma.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

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
 *  \anchor UART_DataLength
 *  \name UART data length
 *
 *  Note: The values should not be changed since it represents the
 *        actual register configuration values used to configure the UART
 *  @{
 */
#define UART_LEN_1                      (0U)
#define UART_LEN_2                      (1U)
#define UART_LEN_3                      (2U)
#define UART_LEN_4                      (3U)
#define UART_LEN_5                      (4U)
#define UART_LEN_6                      (5U)
#define UART_LEN_7                      (6U)
#define UART_LEN_8                      (7U)
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
 *  \param UART_Handle          UART_Handle
 *  \param UART_Transaction*    Pointer to a #UART_Transaction
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
    uint32_t                writeMode;
    /**< Write blocking or Callback mode. Refer \ref UART_TransferMode */
    UART_CallbackFxn        readCallbackFxn;
    /**< Read callback function pointer */
    UART_CallbackFxn        writeCallbackFxn;
    /**< Write callback function pointer */
    /*
     * Driver configuration
     */
    uint32_t                transferMode;
    /**< Transfer mode */
    uint32_t                intrNum;
    /**< Peripheral interrupt number */
    uint8_t                 intrPriority;
    /**< Interrupt priority */
    uint32_t                edmaInst;
    /**< EDMA instance used for QSPI transfer */
    uint32_t                rxEvtNum;
    /**< EDMA Event number used for UART Rx */
    uint32_t                txEvtNum;
    /**< EDMA Event number used for UART Tx */
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

/**
 *  \brief UART EDMA Parameters
 *
 *  Used to store the EDMA parameters allocated for UART transfer.
 *
 */
typedef struct
{
    uint32_t edmaTcc;
    /**< EDMA TCC used for UART transfer */
    uint32_t edmaChId;
    /**< EDMA Channel used for UART transfer */
    uint32_t edmaParam;
    /**< EDMA Param ID used for UART transfer */
    uint32_t edmaRegionId;
    /**< EDMA Region used for UART transfer */
    uint32_t edmaBaseAddr;
    /**< EDMA Base address used for UART transfer */
    uint32_t isIntEnabled;
    /**< EDMA Interrupt enabled status */
    Edma_IntrObject edmaIntrObj;
    /**< EDMA Interrupt object */
} UART_EdmaParams;


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
    UART_EdmaParams         rxEdmaParams;
    /**< EDMA parameters for the UART Rx */
    UART_EdmaParams         txEdmaParams;
    /**< EDMA parameters for the UART Tx */
    EDMA_Handle             uartEdmaHandle;

    CSL_sciRegs            *pSCIRegs;
    /**< Pointer to register overlay */
    uint8_t                 shiftJustification;
    /**< This is the number of bits the read data needs to be shifted. This is
     *   calculated using the data length. When data of fewer than eight bits
     *   in length is received, it is left-justified in SCIRD and padded with
     *   trailing zeros. Data read from the SCIRD should be shifted by software
     *   to make the received data rightjustified. */
} UART_Object;

/**
 *  \brief UART global configuration array
 *
 *  This structure needs to be defined before calling #UART_init() and it must
 *  not be changed by user thereafter.
 *
 *  The last entry of the array should be a NULL entry which demarks the end
 *  of the array.
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
        prms->writeMode          = UART_TRANSFER_MODE_BLOCKING;
        prms->readCallbackFxn    = NULL;
        prms->writeCallbackFxn   = NULL;
        prms->transferMode       = UART_CONFIG_MODE_INTERRUPT;
        prms->intrNum            = 210U;
        prms->intrPriority       = 4U;
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

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef UART_SCI_H_ */

/** @} */
