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
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/cslr_uart.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/uart/v0/lld/uart_lld.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */


/** \brief A handle that is returned from a #UART_open() call */
typedef void *UART_Handle;

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
     * The UART driver uses this index to do an \ref UART_lld_initDma inside the \ref UART_open if the DMA mode is enabled
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
    uint32_t                timeGuardVal;
    /* timeguard feature by UART*/
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
    /**< Interrupt handle for controller ISR */
    HwiP_Object             hwiObj;
    /**< Interrupt object */
    void* uartDmaHandle;
    /**< Pointer to current transaction struct */
    UARTLLD_Object          uartLld_object;
    UARTLLD_Handle          uartLld_handle;
    /**< [IN] Initialization parameters of UART instance */
    UARTLLD_InitHandle      uartLld_initHandle;
    UARTLLD_InitObject      uartLld_initObject;
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
 *  \brief  Function to initialize the #UART_Transaction struct to its defaults
 *
 *  \param  trans       Pointer to #UART_Transaction structure for
 *                      initialization
 */
void UART_Transaction_init(UART_Transaction *trans);

/**
 *  \brief  Function to initialize the #UART_Params struct to its defaults
 *
 *  \param  prms        Pointer to #UART_Params structure for initialization
 */
void UART_Params_init(UART_Params *prms);

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef UART_V0_H_ */

/** @} */
