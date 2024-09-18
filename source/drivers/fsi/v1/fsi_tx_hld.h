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
 *  \defgroup DRV_FSI_TX_MODULE APIs for FSI_Tx
 *  \ingroup  DRV_MODULE
 *  @{
 *  \file  fsi_tx_hld.h
 * 
 *  \brief
 *      This is the header file for the FSI Tx driver which exposes the
 *      data structures and exported API which can be used by the
 *      applications to use the FSI driver.
 *
 *  The FSI Tx driver provides functionality of transferring data between FSI Tx and FSI
 *  TX peripherals.This driver does not interpret any of the data sent to or received
 *  from using this peripheral.
 *
 *
 *  The FSI Tx header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/fsi_tx/fsi_tx_hld.h>
 *   @endcode
 *
 *  ## Initializing the driver #
 *  The FSI Tx Driver needs to be initialized once across the System. This is
 *  done using the #FSI_Tx_open. None of the FSI Tx API's can be used without invoking
 *  this API.
 *
 * @{
 */

#ifndef FSI_Tx__H_
#define FSI_Tx__H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <drivers/fsi/v1/fsi.h>
#include <drivers/fsi/v1/fsi_tx.h>
#include <drivers/hw_include/cslr_soc.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>

/** \brief A handle that is returned from a #FSI_Tx_open() call */
typedef void *FSI_Tx_Handle;

/** \brief Externally defined driver configuration array size */
extern uint32_t             gFsiTxConfigNum;

static SemaphoreP_Object gFsiDmaTxSemObject;

/** @name Return status
 *
 * @{
 *
 * \brief Return status when the API execution was successful
 */
#define FSI_TX_STATUS_SUCCESS         ((int32_t)0)

/**
 * \brief Return status when the API execution was not successful due to a failure
 */
#define FSI_TX_STATUS_FAILURE         ((int32_t)-1)
/** @} */

/**
 *  \anchor FSI_Tx_TransferMode
 *  \name Transfer Mode
 *
 *  This determines whether the driver operates synchronously or asynchronously
 *
 *  In #FSI_TRANSFER_MODE_BLOCKING mode #FSI_write() blocks code
 *  execution until the transaction has completed
 *
 *  In #FSI_TRANSFER_MODE_CALLBACK #FSI_write() does not block code
 *  execution and instead calls a #FSI_TransferCallbackFxn callback function when the
 *  transaction has completed
 *
 *  @{
 */
/**
 *  \brief #FSI_write() blocks execution. This mode can only be used
 *  when called within a Task context
 */
#define FSI_TX_TRANSFER_MODE_BLOCKING    (0U)
/**
 *  \brief #FSI_write() does not block code execution and will call a
 *  #FSI_TransferCallbackFxn. This mode can be used in a Task, Swi, or Hwi context
 */
#define FSI_TX_TRANSFER_MODE_CALLBACK    (1U)
/** @} */

/**
 *  \anchor FSI_OperatingMode
 *  \name Operating Mode
 *
 *  Values used to determine the FSI driver operation.
 *
 *  @{
 */
#define FSI_TX_OPER_MODE_POLLED                 (0U)
#define FSI_TX_OPER_MODE_INTERRUPT              (1U)
#define FSI_TX_OPER_MODE_DMA                    (2U)


/** \brief A handle that is returned from a #UART_open() call */
typedef void *FSI_Tx_DmaHandle;

typedef void *FSI_Tx_DmaChConfig;

/**
 *  \brief  The definition of a callback function used by the FSI Tx driver
 *  when used in #FSI_TX_TRANSFER_MODE_CALLBACK
 *
 *  \param handle          FSI_TX Handle
 *  \param transaction*    Pointer to a #FSI_Tx_Transaction
 */
typedef void (*FSI_Tx_TransferCallbackFxn) (FSI_Tx_Handle handle);

/**
 *  \brief  The definition of a error callback function used by the FSI Tx driver
 *  when used in #FSI_TX_TRANSFER_MODE_CALLBACK
 *
 *  \param handle          FSI_TX Handle
 */
typedef void (*FSI_Tx_ErrorCallbackFxn) (FSI_Tx_Handle handle);
/**
 *  \anchor FSI_Tx_Params
 *  \name   FSI_Tx params
 *
 * \brief FSI_Tx Parameters. Data structure defines the FSI Tx initialization parameters.
 *
 *  FSI_Tx Parameters are used to with the #FSI_Tx_open() call.
 */
typedef struct FSI_Tx_Params_t
{
    /*! Blocking or Callback mode. Refer \ref FSI_Tx_TransferMode
     */
    uint32_t                transferMode;

    /**< Transfer Callback function pointer */
    FSI_Tx_TransferCallbackFxn       transferCallbackFxn;

    /**< Error Callback function pointer */
    FSI_Tx_ErrorCallbackFxn           errorCallbackFxn;

}FSI_Tx_Params;

/*!
 *  \anchor FSI_Tx_Attrs
 *  \name   FSI_Tx Attributes
 * 
 * \brief
 *  FSI_Tx instance attributes - used during init time
 */
typedef struct FSI_Tx_Attrs_s
{
    /* Peripheral base address */
    uint32_t                baseAddr;

    /**< Peripheral interrupt number */
    uint32_t                intrNum;

    /**< Driver operating mode. Polling, DMA, interrupt */
    uint32_t                operMode;

    /**< Interrupt priority */
    uint8_t                 intrPriority;

} FSI_Tx_Attrs;

/**
 *  \anchor FSI_Tx_Object
 *  \name   FSI_Tx Object
 * 
 * \brief
 *  FSI_Tx Master Control Block
 *
 * @details
 *  The structure describes the FSI_Tx Driver and is used to hold the relevant
 *  information with respect to the FSI_Tx module.
 */
typedef struct FSI_Tx_Object_t
{
    /*!
     * \brief   Instance handle to which this object belongs.
     */
    FSI_Tx_Handle            handle;

    /**
     * \brief   FSI_Tx driver init parameters
     */
    FSI_Tx_Params               *params;

    /**
     * \brief   Number of interrupts received for message Tx
     */
    uint32_t                        interrupts;

    /**
     * \brief   Dma driver handle.
     */
    FSI_Tx_DmaHandle             fsiTxDmaHandle;

    /**
     * \brief   Pointer to Dma channel configuration.
     */
    FSI_Tx_DmaChConfig           fsiTxDmaChCfg;

    /**
     * \brief   Pointer to be used by application to store miscellaneous data.
     */
    void*                          args;

    /**< Write Transfer Sync Sempahore - to sync between transfer completion ISR
     *   and task */
    void                   *writeTransferSem;

    /**< Write Transfer Sync Sempahore - to sync between transfer completion ISR
     *   and task. Transfer Sync Sempahore object */
    SemaphoreP_Object       writeTransferSemObj;

    /**< Interrupt handle for controller ISR */
    void                   *hwiHandle;

    /**< Interrupt object */
    HwiP_Object             hwiObj;

}FSI_Tx_Object;

/**
 *  \anchor FSI_Tx_Config
 *  \name   FSI_Tx Config
 * 
 *  \brief FSI_Tx global configuration array
 *
 *  This structure needs to be defined before calling #FSI_Tx_init() and it must
 *  not be changed by user thereafter.
 *
 *  The last entry of the array should be a NULL entry which demarks the end
 *  of the array.
 */
typedef struct FSI_Tx_Config_s
{
    FSI_Tx_Attrs      *attrs;
    /**< Pointer to driver specific attributes */
    FSI_Tx_Object     *object;
    /**< Pointer to driver specific data object */
} FSI_Tx_Config;

/** @} */

/**
 *  \brief  This function initializes each driver instance object and create a driver lock.
 */
void FSI_Tx_init(void);

/**
 *  \brief  This function de-initializes each driver instance object and delete a driver lock.
 */
void FSI_Tx_deinit(void);

/**
 *  \brief  This function opens a given FSI_Tx peripheral
 *
 *  \pre    FSI_Tx has been initialized using #FSI_Tx_init()
 *
 *  \param  index       Index of config to use in the *FSI_Tx_Config* array
 *  \param  openPrms    Pointer to open parameters. If NULL is passed, then
 *                      default values will be used
 *
 *  \return A #FSI_Tx_Handle on success or a NULL on an error or if it has been
 *          opened already
 */
FSI_Tx_Handle FSI_Tx_open(uint32_t index, FSI_Tx_Params *openPrms);

/**
 *  \brief  Function to close a FSI_Tx peripheral specified by the FSI_Tx handle
 *
 *  \pre    #FSI_Tx_open() has to be called first
 *
 *  \param  handle      #FSI_Tx_Handle returned from #FSI_Tx_open()
 *
 */
void FSI_Tx_close(FSI_Tx_Handle handle);

int32_t FSI_Tx_edmaIntrInit(FSI_Tx_Object *FsiTxObj, uint32_t tccAlloc);

int32_t FSI_Tx_hld(FSI_Tx_Handle handle, uint16_t *txBufData, uint16_t *txBufTagAndUserData, 
                    uint16_t dataSize, uint16_t bufIdx);

int32_t FSI_Tx_Intr(FSI_Tx_Handle handle, uint16_t *txBufData, uint16_t *txBufTagAndUserData, 
                    uint16_t dataSize, uint16_t bufIdx);

int32_t FSI_Tx_Dma(FSI_Tx_Handle handle, uint16_t *txBufData, uint16_t *txBufTagAndUserData, 
                    uint16_t dataSize, uint16_t bufIdx);

int32_t FSI_Tx_Poll(FSI_Tx_Handle handle, uint16_t *txBufData, uint16_t *txBufTagAndUserData, 
                    uint16_t dataSize, uint16_t bufIdx);

int32_t FSI_Tx_dmaOpen(FSI_Tx_Handle handle, FSI_Tx_DmaChConfig dmaChCfg);

int32_t FSI_Tx_edmaChInit(const FSI_Tx_Object *FsiTxObj, uint32_t edmaEventNo,
                          uint32_t *edmaParam, uint32_t *edmaTccAlloc);

int32_t FSI_Tx_configureDma(const FSI_Tx_Object *fsiTxObj, uint32_t *dmaCh,
                void *src, void *dst, uint32_t *tcc, uint32_t *param, uint32_t regionId,
                uint32_t aCnt, uint32_t bCnt, uint32_t cCnt, uint32_t srcBIdx, uint32_t destBIdx,
                uint32_t srcCIdx, uint32_t destCIdx, uint32_t triggerMode);

void FSI_Tx_DmaCompletionCallback(void *args);

void FSI_Tx_pendDmaCompletion();

/**
 *  \brief  This is the FSI TX ISR and can be used as IRQ handler.
 *
 *  \param  args      Argument to the ISR.
 *
 */
void FSI_Tx_Isr(void* args);
#ifdef __cplusplus
}
#endif

#endif