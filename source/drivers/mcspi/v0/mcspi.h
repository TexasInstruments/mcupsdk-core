/*
 *  Copyright (C) 2021-23 Texas Instruments Incorporated
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
 *  \defgroup DRV_MCSPI_MODULE APIs for MCSPI
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the MCSPI module. The APIs
 *  can be used by other drivers to get access to MCSPI and also by
 *  application to initiate data transfer operation.
 *
 *  @{
 */

/**
 *  \file v0/mcspi.h
 *
 *  \brief MCSPI Driver API/interface file.
 */

#ifndef MCSPI_H_
#define MCSPI_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/mcspi/v0/lld/mcspi_lld.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief A handle that is returned from a #MCSPI_open() call */
typedef void *MCSPI_Handle;

/**
 *  \anchor MCSPI_TransferMode
 *  \name Transfer Mode
 *
 *  This determines whether the driver operates synchronously or asynchronously
 *
 *  In #MCSPI_TRANSFER_MODE_BLOCKING mode #MCSPI_transfer() blocks code
 *  execution until the transaction has completed
 *
 *  In #MCSPI_TRANSFER_MODE_CALLBACK #MCSPI_transfer() does not block code
 *  execution and instead calls a #MCSPI_CallbackFxn callback function when the
 *  transaction has completed
 *
 *  @{
 */
/**
 *  \brief #MCSPI_transfer() blocks execution. This mode can only be used
 *  when called within a Task context
 */
#define MCSPI_TRANSFER_MODE_BLOCKING    (0U)
/**
 *  \brief #MCSPI_transfer() does not block code execution and will call a
 *  #MCSPI_CallbackFxn. This mode can be used in a Task, Swi, or Hwi context
 */
#define MCSPI_TRANSFER_MODE_CALLBACK    (1U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief  The definition of a callback function used by the SPI driver
 *  when used in #MCSPI_TRANSFER_MODE_CALLBACK
 *
 *  \param MCSPI_Handle          MCSPI_Handle
 *  \param MCSPI_Transaction*    Pointer to a #MCSPI_Transaction
 */
typedef void (*MCSPI_CallbackFxn) (MCSPI_Handle handle,
                                   MCSPI_Transaction *transaction);

/**
 *  \brief MCSPI Parameters
 *
 *  MCSPI Parameters are used to with the #MCSPI_open() call. Default values for
 *  these parameters are set using #MCSPI_OpenParams_init().
 *
 *  If NULL is passed for the parameters, #MCSPI_open() uses default parameters.
 *
 *  \sa #MCSPI_OpenParams_init()
 */
typedef struct
{
    uint32_t                transferMode;
    /**< Blocking or Callback mode. Refer \ref MCSPI_TransferMode */
    uint32_t                transferTimeout;
    /**< Transfer timeout in system ticks */
    MCSPI_CallbackFxn       transferCallbackFxn;
    /**< Callback function pointer */
    uint32_t                msMode;
    /**< Controller or Peripheral mode. Refer \ref MCSPI_MsMode */
    int32_t                 mcspiDmaIndex;
    /**< Index of DMA instance used by MCSPI Driver. This index will be set by SysCfg according to the DMA driver chosen.
     * The MCSPI driver uses this index to do an \ref MCSPI_lld_initDma inside the \ref MCSPI_open if the DMA mode is enabled
     */
} MCSPI_OpenParams;

/** \brief MCSPI instance attributes - used during init time */
typedef struct
{
    /*
     * SOC configuration
     */
    uint32_t                baseAddr;
    /**< Peripheral base address */
    uint32_t                inputClkFreq;
    /**< Module input clock frequency */

    /*
     * Driver configuration
     */
    uint32_t                intrNum;
    /**< Peripheral interrupt number */
    uint32_t                operMode;
    /**< Driver operating mode */
    uint8_t                 intrPriority;
    /**< Interrupt priority */

    /*
     * MCSPI instance configuration - common across all channels
     */
    uint32_t                chMode;
    /**< Channel mode: Single or multi channel. Refer \ref MCSPI_ChMode  */
    uint32_t                pinMode;
    /**< Pin mode. Refer \ref MCSPI_PinMode */
    uint32_t                initDelay;
    /**< Initial SPI delay for first transfer. Refer \ref MCSPI_InitDelay */
    uint32_t                multiWordAccess;
    /**< Flag to enable/disable turbo mode */
} MCSPI_Attrs;

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/**
 *  \brief MCSPI driver object
 */
typedef struct
{
    /*
     * User parameters
     */
    MCSPI_Handle            handle;
    /**< Instance handle to which this object belongs */
    MCSPI_OpenParams        openPrms;
    /**< Open parameter as provided by user */
    uint32_t                baseAddr;
    /**< Peripheral base address - CPU view */
    MCSPI_ChObject          chObj[MCSPI_MAX_NUM_CHANNELS];
    /**< Channel object */

    /*
     * State variables
     */
    uint32_t                isOpen;
    /**< Flag to indicate whether the instance is opened already */
    void                   *transferSem;
    /**< Transfer Sync Sempahore - to sync between transfer completion ISR
     *   and task */
    SemaphoreP_Object       transferSemObj;
    /**< Transfer Sync Sempahore object */
    void                   *hwiHandle;
    /**< Interrupt handle for controller ISR */
    HwiP_Object             hwiObj;
    /**< Interrupt object */

    MCSPI_Transaction      *transaction;
    /**< Pointer to transaction */
    void                   *mcspiDmaHandle;
    /**< DMA Handle */
    MCSPILLD_Object        mcspiLldObject;
    MCSPILLD_Handle        mcspiLldHandle;
    MCSPILLD_InitObject    mcspiLldInitObj;
 /*  MCSPI driver object for LLD */
} MCSPI_Object;

/**
 *  \brief MCSPI global configuration array
 *
 *  This structure needs to be defined before calling #MCSPI_init() and it must
 *  not be changed by user thereafter.
 *
 *  The last entry of the array should be a NULL entry which demarks the end
 *  of the array.
 */
typedef struct
{
    const MCSPI_Attrs      *attrs;
    /**< Pointer to driver specific attributes */
    MCSPI_Object           *object;
    /**< Pointer to driver specific data object */
} MCSPI_Config;

/** \brief Externally defined driver configuration array */
extern MCSPI_Config         gMcspiConfig[];
/** \brief Externally defined driver configuration array size */
extern uint32_t             gMcspiConfigNum;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief  This function initializes the MCSPI module
 */
void MCSPI_init(void);

/**
 *  \brief  This function de-initializes the MCSPI module
 */
void MCSPI_deinit(void);

/**
 *  \brief  This function opens a given MCSPI peripheral
 *
 *  \pre    MCSPI controller has been initialized using #MCSPI_init()
 *
 *  \param  index       Index of config to use in the *MCSPI_Config* array
 *  \param  openPrms    Pointer to open parameters. If NULL is passed, then
 *                      default values will be used
 *
 *  \return A #MCSPI_Handle on success or a NULL on an error or if it has been
 *          opened already
 *
 *  \sa     #MCSPI_init()
 *  \sa     #MCSPI_close()
 *  \sa     #MCSPI_OpenParams_init
 */
MCSPI_Handle MCSPI_open(uint32_t index, const MCSPI_OpenParams *openPrms);

/**
 *  \brief  Function to close a MCSPI peripheral specified by the MCSPI handle
 *
 *  \pre    #MCSPI_open() has to be called first
 *
 *  \param  handle      #MCSPI_Handle returned from #MCSPI_open()
 *
 *  \sa     #MCSPI_open()
 */
void MCSPI_close(MCSPI_Handle handle);

/**
 *  \brief  Function to return a open'ed MCSPI handle given a MCSPI instance index
 *
 *  \param  index       Index of config to use in the *MCSPI_Config* array
 *
 *  \return A #MCSPI_Handle on success or a NULL on an error or if the instance
 *            index has  NOT been opened yet
 */
MCSPI_Handle MCSPI_getHandle(uint32_t index);

/**
 *  \brief  Function to configure a MCSPI channel
 *
 *  \param  handle      #MCSPI_Handle returned from #MCSPI_open()
 *  \param  chCfg       Pointer to #MCSPI_ChConfig. This parameter can't be NULL
 *
 *  \return #SystemP_SUCCESS if successful; else error on failure
 *
 *  \sa     #MCSPI_open
 *  \sa     #MCSPI_ChConfig_init
 */

int32_t MCSPI_chConfig(MCSPI_Handle handle, const MCSPI_ChConfig *chCfg);

/**
 *  \brief  Function to configure a DMA of a channel
 *
 *  \param  handle      #MCSPI_Handle returned from #MCSPI_open()
 *  \param  chCfg       Pointer to #MCSPI_ChConfig. This parameter can't be NULL
 *  \param  dmaChCfg    Pointer to \ref MCSPI_DmaChConfig. This parameter can't be NULL
 *
 *  \return #SystemP_SUCCESS if successful; else error on failure
 *
 *  \sa     #MCSPI_open
 *  \sa     #MCSPI_ChConfig_init
 */
int32_t MCSPI_dmaChConfig(MCSPI_Handle handle,
                          const MCSPI_ChConfig *chCfg,
                          const MCSPI_DmaChConfig *dmaChCfg);

/**
 *  \brief  Function to perform MCSPI transactions
 *
 *  If the MCSPI is in #MCSPI_MS_MODE_CONTROLLER mode, it will immediately start the
 *  transaction. If the MCSPI is in #MCSPI_MS_MODE_PERIPHERAL mode, it prepares the
 *  driver for a transaction with a MCSPI controller device. The device will then
 *  wait until the controller begins the transfer.
 *
 *  In #MCSPI_TRANSFER_MODE_BLOCKING, #MCSPI_transfer() will block task
 *  execution until the transaction has completed or a timeout has occurred.
 *
 *  In #MCSPI_TRANSFER_MODE_CALLBACK, #MCSPI_transfer() does not block
 *  task execution, but calls a #MCSPI_CallbackFxn once the transfer
 *  has finished. This makes #MCSPI_transfer() safe to be used within a Task,
 *  software or hardware interrupt context.
 *
 *  From calling #MCSPI_transfer() until transfer completion, the
 *  #MCSPI_Transaction structure must stay persistent and must not be altered
 *  by application code.
 *  It is also forbidden to modify the content of the #MCSPI_Transaction.txBuf
 *  during a transaction, even though the physical transfer might not have
 *  started yet. Doing this can result in data corruption. This is especially
 *  important for peripheral operations where #MCSPI_transfer() might be called a
 *  long time before the actual data transfer begins.
 *
 *  \param  handle      #MCSPI_Handle returned from #MCSPI_open()
 *  \param  transaction Pointer to a #MCSPI_Transaction. All of the fields
 *                      within transaction except #MCSPI_Transaction.count and
 *                      #MCSPI_Transaction.status are WO (write-only) unless
 *                      otherwise noted in the driver implementations. If a
 *                      transaction timeout has occurred,
 *                      #MCSPI_Transaction.count will contain the number of
 *                      frames that were transferred. Neither is it allowed to
 *                      modify the transaction object nor the content of
 *                      #MCSPI_Transaction.txBuf until the transfer
 *                      has completed
 *
 *  \return #SystemP_SUCCESS if started successfully; else error on failure
 *
 *  \sa     #MCSPI_open
 *  \sa     #MCSPI_transferCancel
 */
int32_t MCSPI_transfer(MCSPI_Handle handle, MCSPI_Transaction *transaction);

/**
 *  \brief  Function to cancel MCSPI transactions on channel of a
 *          SPI peripheral specified by the MCSPI handle
 *
 *  In #MCSPI_TRANSFER_MODE_BLOCKING, #MCSPI_transferCancel has no effect.
 *
 *  In #MCSPI_TRANSFER_MODE_CALLBACK, #MCSPI_transferCancel() will stop an
 *  MCSPI transfer if one is in progress.
 *  If a transaction was in progress, its callback function will be called
 *  in context from which this API is called from. The #MCSPI_CallbackFxn
 *  function can determine if the transaction was successful or not by reading
 *  the \ref MCSPI_TransferStatus status value in the #MCSPI_Transaction
 *  structure.
 *
 *  \param  handle      #MCSPI_Handle returned from #MCSPI_open()
 *
 *  \sa     #MCSPI_open
 *  \sa     #MCSPI_transfer
 */
int32_t MCSPI_transferCancel(MCSPI_Handle handle);

/**
 *  \brief  Function to initialize the #MCSPI_OpenParams struct to its defaults
 *
 *  \param  openPrms    Pointer to #MCSPI_OpenParams structure for
 *                      initialization
 */
static inline void MCSPI_OpenParams_init(MCSPI_OpenParams *openPrms);

/**
 *  \brief  Function to initialize the #MCSPI_ChConfig struct to its defaults
 *
 *  \param  chConfig    Pointer to #MCSPI_ChConfig structure for
 *                      initialization
 */
static inline void MCSPI_ChConfig_init(MCSPI_ChConfig *chConfig);

/**
 *  \brief  Function to initialize the #MCSPI_Transaction struct to its defaults
 *
 *  \param  trans       Pointer to #MCSPI_Transaction structure for
 *                      initialization
 */
static inline void MCSPI_Transaction_init(MCSPI_Transaction *trans);

/**
 *  \brief  Function to re-configure Effective FIFO Words.
 *
 *  \param  handle          #MCSPI_Handle returned from #MCSPI_open()
 *  \param  chNum           Channel used for communication.
 *  \param  numWordsRxTx    Number of words to transfer
 *
 *  \return #SystemP_SUCCESS if successful; else error on failure
 *
 *  \sa     #MCSPI_open
 */
int32_t MCSPI_reConfigFifo(MCSPI_Handle handle, uint32_t chNum, uint32_t numWordsRxTx);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

static inline void MCSPI_OpenParams_init(MCSPI_OpenParams *openPrms)
{
    if(openPrms != NULL)
    {
        openPrms->transferMode        = MCSPI_TRANSFER_MODE_BLOCKING;
        openPrms->transferCallbackFxn = NULL;
        openPrms->msMode              = MCSPI_MS_MODE_CONTROLLER;
        openPrms->transferTimeout     = SystemP_WAIT_FOREVER;
    }
}

static inline void MCSPI_ChConfig_init(MCSPI_ChConfig *chConfig)
{
    if(chConfig != NULL)
    {
        chConfig->chNum             = MCSPI_CHANNEL_0;
        chConfig->frameFormat       = MCSPI_FF_POL0_PHA0;
        chConfig->bitRate           = 1000000U;
        chConfig->csPolarity        = MCSPI_CS_POL_LOW;
        chConfig->trMode            = MCSPI_TR_MODE_TX_RX;
        chConfig->inputSelect       = MCSPI_IS_D1;
        chConfig->dpe0              = MCSPI_DPE_ENABLE;
        chConfig->dpe1              = MCSPI_DPE_DISABLE;
        chConfig->slvCsSelect       = MCSPI_SLV_CS_SELECT_0;
        chConfig->startBitEnable    = FALSE;
        chConfig->startBitPolarity  = MCSPI_SB_POL_LOW;
        chConfig->csIdleTime        = MCSPI_TCS0_0_CLK;
        chConfig->defaultTxData     = 0x00000000U;
        chConfig->txFifoTrigLvl     = 16;
        chConfig->rxFifoTrigLvl     = 16;
    }
}

static inline void MCSPI_Transaction_init(MCSPI_Transaction *trans)
{
    if(trans != NULL)
    {
        trans->channel   = 0U;
        trans->csDisable = TRUE;
        trans->dataSize  = 8U;
        trans->count     = 0U;
        trans->txBuf     = NULL;
        trans->rxBuf     = NULL;
        trans->args      = NULL;
        trans->status    = MCSPI_TRANSFER_COMPLETED;
    }
}

/* ========================================================================== */
/*                       Advanced Macros & Typedefs                           */
/* ========================================================================== */


/* ========================================================================== */
/*                       Advanced Function Declarations                       */
/* ========================================================================== */
/**
 *  \brief  Function to get base address of MCSPI instance of a particular
 *          handle.
 *
 *  \param  handle      #MCSPI_Handle returned from #MCSPI_open()
 *
 *  \sa     #MCSPI_open
 */
uint32_t MCSPI_getBaseAddr(MCSPI_Handle handle);

/* ========================================================================== */
/*                       Advanced Function Definitions                        */
/* ========================================================================== */

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef MCSPI_H_ */

/** @} */
/** @} */
