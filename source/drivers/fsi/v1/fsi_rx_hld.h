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
 *  \defgroup DRV_FSI_RX_MODULE APIs for FSI_RX
 *  \ingroup  DRV_MODULE
 *  @{
 *  \file  fsi_rx_hld.h
 * 
 *  \brief
 *      This is the header file for the FSI Rx driver which exposes the
 *      data structures and exported API which can be used by the
 *      applications to use the FSI driver.
 *
 *  The FSI Rx driver provides functionality of transferring data between FSI Rx and FSI
 *  Rx peripherals.This driver does not interpret any of the data sent to or received
 *  from using this peripheral.
 *
 *
 *  The FSI Rx header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/fsi_rx/fsi_rx_hld.h>
 *   @endcode
 *
 *  ## Initializing the driver #
 *  The FSI Rx Driver needs to be initialized once across the System. This is
 *  done using the #FSI_Rx_open. None of the FSI Rx API's can be used without invoking
 *  this API.
 *
 * @{
 */

#ifndef FSI_Rx__H_
#define FSI_Rx__H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <drivers/fsi/v1/fsi.h>
#include <drivers/fsi/v1/fsi_rx.h>
#include <drivers/hw_include/cslr_soc.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>

/** \brief A handle that is returned from a #FSI_Rx_open() call */
typedef void *FSI_Rx_Handle;

/** \brief A handle that is returned from a #UART_open() call */
typedef void *FSI_Rx_DmaHandle;

typedef void *FSI_Rx_DmaChConfig;

/** \brief Externally defined driver configuration array size */
extern uint32_t             gFsiRxConfigNum;

static SemaphoreP_Object gFsiDmaRxSemObject;

/**
 *  \anchor FSI_OperatingMode
 *  \name Operating Mode
 *
 *  Values used to determine the FSI driver operation.
 *
 *  @{
 */
#define FSI_RX_OPER_MODE_POLLED                 (0U)
#define FSI_RX_OPER_MODE_INTERRUPT              (1U)
#define FSI_RX_OPER_MODE_DMA                    (2U)

/** @name Return status
 *
 * @{
 *
 * \brief Return status when the API execution was successful
 */
#define FSI_RX_STATUS_SUCCESS         ((int32_t)0)

/**
 * \brief Return status when the API execution was not successful due to a failure
 */
#define FSI_RX_STATUS_FAILURE         ((int32_t)-1)
/** @} */

/**
 *  \anchor FSI_Rx_TransferMode
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
#define FSI_RX_TRANSFER_MODE_BLOCKING    (0U)

/**
 *  \brief  The definition of a callback function used by the FSI Rx driver
 *  when used in #FSI_RX_TRANSFER_MODE_CALLBACK
 *
 *  \param handle          FSI_Rx Handle
 *  \param transaction*    Pointer to a #FSI_RX_Transaction
 */
typedef void (*FSI_Rx_TransferCallbackFxn) (FSI_Rx_Handle handle);

/**
 *  \brief  The definition of a error callback function used by the FSI Rx driver
 *  when used in #FSI_Rx_TRANSFER_MODE_CALLBACK
 *
 *  \param handle          FSI_Rx Handle
 */
typedef void (*FSI_Rx_ErrorCallbackFxn) (FSI_Rx_Handle handle);
/**
 *  \anchor FSI_Rx_Params
 *  \name   FSI_Rx params
 *  
 * \brief FSI_Rx Parameters. Data structure defines the FSI Rx initialization parameters.
 *
 *  FSI_Rx Parameters are used to with the #FSI_Rx_open() call.
 */
typedef struct FSI_Rx_Params_t
{
    /*! Blocking or Callback mode. Refer \ref FSI_Rx_TransferMode
     */
    uint32_t                transferMode;

    /**< Transfer Callback function pointer */
    FSI_Rx_TransferCallbackFxn       transferCallbackFxn;

    /**< Error Callback function pointer */
    FSI_Rx_ErrorCallbackFxn           errorCallbackFxn;

}FSI_Rx_Params;

/*!
 *  \anchor FSI_Rx_Attrs
 *  \name   FSI_Rx Attributes
 * 
 * \brief
 *  FSI_Rx instance attributes - used during init time
 */
typedef struct FSI_Rx_Attrs_s
{
    /* Peripheral base address */
    uint32_t                baseAddr;

    /**< Peripheral interrupt number */
    uint32_t                intrNum;

    /**< Driver operating mode. Polling, DMA, interrupt */
    uint32_t                operMode;

    /**< Interrupt priority */
    uint8_t                 intrPriority;

} FSI_Rx_Attrs;

/**
 *  \anchor FSI_Rx_Object
 *  \name   FSI_Rx Object
 * 
 * \brief
 *  FSI_Rx Master Control Block
 *
 * @details
 *  The structure describes the FSI_Rx Driver and is used to hold the relevant
 *  information with respect to the FSI_Rx module.
 */
typedef struct FSI_Rx_Object_t
{
    /*!
     * \brief   Instance handle to which this object belongs.
     */
    FSI_Rx_Handle            handle;

    /**
     * \brief   FSI_Rx driver init parameters
     */
    FSI_Rx_Params               *params;

    /**
     * \brief   Number of interrupts received for message Rx or Rx
     */
    uint32_t                        interrupts;

    /**
     * \brief   Dma driver handle.
     */
    FSI_Rx_DmaHandle             fsiRxDmaHandle;

    /**
     * \brief   Pointer to Dma channel configuration.
     */
    FSI_Rx_DmaChConfig           fsiRxDmaChCfg;

    /**
     * \brief   Pointer to be used by application to store miscellaneous data.
     */
    void*                          args;

    /**< Write Transfer Sync Sempahore - to sync between transfer completion ISR
     *   and task */
    void                   *readTransferSem;

    /**< Write Transfer Sync Sempahore - to sync between transfer completion ISR
     *   and task. Transfer Sync Sempahore object */
    SemaphoreP_Object       readTransferSemObj;

    /**< Interrupt handle for controller ISR */
    void                   *hwiHandle;

    /**< Interrupt object */
    HwiP_Object             hwiObj;

}FSI_Rx_Object;

/**
 *  \anchor FSI_Rx_Config
 *  \name   FSI_Rx Config
 * 
 *  \brief FSI_Rx global configuration array
 *
 *  This structure needs to be defined before calling #FSI_Rx_init() and it must
 *  not be changed by user thereafter.
 *
 *  The last entry of the array should be a NULL entry which demarks the end
 *  of the array.
 */
typedef struct FSI_Rx_Config_s
{
    FSI_Rx_Attrs      *attrs;
    /**< Pointer to driver specific attributes */
    FSI_Rx_Object     *object;
    /**< Pointer to driver specific data object */
} FSI_Rx_Config;

/** @} */

/**
 *  \brief  This function initializes each driver instance object and create a driver lock.
 */
void FSI_Rx_init(void);

/**
 *  \brief  This function de-initializes each driver instance object and delete a driver lock.
 */
void FSI_Rx_deinit(void);

/**
 *  \brief  This function opens a given FSI_Rx peripheral
 *
 *  \pre    FSI_Rx has been initialized using #FSI_Rx_init()
 *
 *  \param  index       Index of config to use in the *FSI_Rx_Config* array
 *  \param  openPrms    Pointer to open parameters. If NULL is passed, then
 *                      default values will be used
 *
 *  \return A #FSI_Rx_Handle on success or a NULL on an error or if it has been
 *          opened already
 */
FSI_Rx_Handle FSI_Rx_open(uint32_t index, FSI_Rx_Params *openPrms);

/**
 *  \brief  Function to close a FSI_Rx peripheral specified by the FSI_Rx handle
 *
 *  \pre    #FSI_Rx_open() has to be called first
 *
 *  \param  handle      #FSI_Rx_Handle returned from #FSI_Rx_open()
 *
 */
void FSI_Rx_close(FSI_Rx_Handle handle);

int32_t FSI_Rx_edmaIntrInit(FSI_Rx_Object *FsiTxObj, uint32_t tccAlloc);

int32_t FSI_Rx_hld(FSI_Rx_Handle handle, uint16_t *rxBufData, uint16_t *rxBufTagAndUserData,
                    uint16_t dataSize, uint16_t bufIdx);

int32_t FSI_Rx_Intr(FSI_Rx_Handle handle, uint16_t *rxBufData, uint16_t *rxBufTagAndUserData,
                    uint16_t dataSize, uint16_t bufIdx);

int32_t FSI_Rx_Dma(FSI_Rx_Handle handle, uint16_t *rxBufData, uint16_t *rxBufTagAndUserData,
                    uint16_t dataSize, uint16_t bufIdx);

int32_t FSI_Rx_Poll(FSI_Rx_Handle handle, uint16_t *rxBufData, uint16_t *rxBufTagAndUserData,
                    uint16_t dataSize, uint16_t bufIdx);

int32_t FSI_Rx_edmaChInit(const FSI_Rx_Object *fsiRxObj, uint32_t edmaEventNo,
                          uint32_t *edmaParam, uint32_t *edmaTccAlloc);

int32_t FSI_Rx_dmaOpen(FSI_Rx_Handle fsiRxHandle, FSI_Rx_DmaChConfig dmaChCfg);

int32_t FSI_Rx_configureDma(const FSI_Rx_Object *fsiRxObj, uint32_t *dmaCh,
                void *src, void *dst, uint32_t *tcc, uint32_t *param, uint32_t regionId,
                uint32_t aCnt, uint32_t bCnt, uint32_t cCnt, uint32_t srcBIdx, uint32_t destBIdx,
                uint32_t srcCIdx, uint32_t destCIdx, uint32_t triggerMode);

void FSI_Rx_DmaCompletionCallback(void *args);

void FSI_Rx_pendDmaCompletion();

/**
 *  \brief #FSI_write() does not block code execution and will call a
 *  #FSI_TransferCallbackFxn. This mode can be used in a Task, Swi, or Hwi context
 */int32_t FSI_Rx_edmaIntrInit(FSI_Rx_Object *FsiRxObj, uint32_t tccAlloc);
/**
 *  \brief  This is the FSI RX ISR and can be used as IRQ handler.
 *
 *  \param  args      Argument to the ISR.
 *
 */
void FSI_Rx_Isr(void* args);

#ifdef __cplusplus
}
#endif

#endif