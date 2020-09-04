/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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
 *  \defgroup DRV_MCASP_MODULE APIs for MCASP
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the MCASP module. The APIs
 *  can be used by other drivers to get access to MCASP and also by
 *  application to initiate data transfer operation.
 *
 *  @{
 */

/**
 *  \file v0/mcasp.h
 *
 *  \brief MCASP Driver API/interface file.
 */

#ifndef MCASP_H_
#define MCASP_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/mcasp.h>
#include <stdint.h>
#include <stdbool.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/QueueP.h>
#include <drivers/hw_include/cslr_mcasp.h>
#include <drivers/edma.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief A handle that is returned from a #MCASP_open() call */
typedef void *MCASP_Handle;

/** \brief The time to try (in Msec) before the GBLCTL register setting timeouts
   if the setting/resetting is done in a context other than a task this will
   be used as a retry count rather than the MSec timeout
*/
#define MCASP_GBLCTL_TIMEOUT  (30000U)

/** \brief The time to try (in Msec) to check the XDTATA/RDATA flags in status
   register. if the setting/resetting is done in a context other than a task
   this will be used as a retry count rather than the MSec timeout
*/
#define MCASP_DATA_TIMEOUT  (30000U)

/** \brief McASP macros for enable/reset */
#define MCASP_RESET                             ((uint32_t) 0x00U)
#define MCASP_ENABLE                            ((uint32_t) 0x01U)
#define MCASP_DISABLE                           ((uint32_t) 0x00U)

/*
** McASP Register Offset
*/
#define MCASP_REG_OFFSET                         ((uint32_t)0x4U)

/**
 *  \anchor MCASP_TransferStatus
 *  \name Transfer Status Code
 *
 *  Status codes that are set by the MCASP driver
 *
 *  @{
 */
/** \brief I/O completed successfully */
#define MCASP_TRANSFER_STATUS_COMPLETED        (0U)
/** \brief I/O queued and pending */
#define MCASP_TRANSFER_STATUS_STARTED          (1U)
/** \brief I/O cancelled. Returned by incomplete read or write requests */
#define MCASP_TRANSFER_STATUS_CANCELLED        (2U)
/** \brief Generic failure condition */
#define MCASP_TRANSFER_STATUS_FAILED           (3U)
/** \brief I/O timeout occurred */
#define MCASP_TRANSFER_STATUS_TIMEOUT          (4U)
/** @} */

/**
 *  \anchor MCASP_TransferMode
 *  \name McASP Transfer Mode
 *
 *  @{
 */
/**
 *  \brief MCASP read/write APIs blocks execution. This mode can only be used
 *  when called within a Task context
 */
#define MCASP_TRANSFER_MODE_POLLING       (0U)
/**
 *  \brief MCASP read/write APIs does not block code execution and will call a
 *  #MCASP_TxCallbackFxn or #MCASP_RxCallbackFxn.
 *  This mode can be used in a Task, Swi, or Hwi context
 */
#define MCASP_TRANSFER_MODE_INTERRUPT     (1U)
/**
 *  \brief MCASP read/write APIs does not block code execution and will use DMA
 *  for transfers
 */
#define MCASP_TRANSFER_MODE_DMA           (2U)
/** @} */

/**
 *  \anchor MCASP_OperMode
 *  \name McASP Operating Mode
 *
 *  OPerating Modes that are supported by the MCASP driver
 *
 *  @{
 */
/** \brief I/O completed successfully */
#define MCASP_OPMODE_MASTER             (0U)
/** \brief I/O queued and pending */
#define MCASP_OPMODE_SLAVE              (1U)
/** @} */

/**
 *  \anchor MCASP_ChannelDirection
 *  \name MCASP Channel Direction
 *
 *   the channel modes supported by Mcasp
 *
 *  @{
 */
/** \brief MCASP Channel will receive data */
#define MCASP_CHANNEL_INPUT             (0x0001U)
/** \brief MCASP Channel will transmit data */
#define MCASP_CHANNEL_OUTPUT            (0x0002U)
/** \brief McASP channel transmits & recieves data */
#define MCASP_CHANNEL_INOUT             (MCASP_CHANNEL_INPUT | MCASP_CHANNEL_OUTPUT)
/** @} */

/**
 *  \anchor MCASP_DriverState
 *  \name MCASP Driver State
 *
 *   Mcasp driver state enums used to track the driver state
 *
 *  @{
 */
#define MCASP_DRIVER_STATE_DELETED          (0U)
#define MCASP_DRIVER_STATE_CREATED          (1U)
#define MCASP_DRIVER_STATE_INITIALIZED      (2U)
#define MCASP_DRIVER_STATE_OPENED           (3U)
#define MCASP_DRIVER_STATE_CLOSED           (4U)
#define MCASP_DRIVER_STATE_DEINITIALIZED    (5U)
#define MCASP_DRIVER_STATE_POWERED_DOWN     (6U)
#define MCASP_DRIVER_STATE_PWRM_SUSPEND     (7U)
/** @} */

/**
 *  \anchor MCASP_SerializerStatus
 *  \name MCASP Serializer Status
 *
 *  \brief Enumeration for serializer status
 *
 *  @{
 */
#define MCASP_SERIALIZER_STATUS_FREE        (0U)
#define MCASP_SERIALIZER_STATUS_XMT         (1U)
#define MCASP_SERIALIZER_STATUS_RCV         (2U)
/** @} */

/**
 *  \anchor MCASP_ChannelMode
 *  \name MCASP Channel Mode
 *
 *  \brief Enumeration for channel mode
 *
 *  @{
 */
#define MCASP_CHANNEL_MODE_FREE             (0U)
#define MCASP_CHANNEL_MODE_XMT_DIT          (1U)
#define MCASP_CHANNEL_MODE_XMT_TDM          (2U)
#define MCASP_CHANNEL_MODE_RCV              (3U)
/** @} */

/**
 *  \anchor MCASP_WordSelect
 *  \name MCASP Word Select
 *
 *  \brief Enumerated constant for selecting MSB/LSB word in the slot bits
 *
 *  @{
 */
#define MCASP_WORD_SELECT_LSW               (0U)
#define MCASP_WORD_SELECT_MSW               (1U)
/** @} */

/**
 *  \anchor MCASP_BufferFormats
 *  \name Enumerated constants to specify the supported buffer formats.
 *
 *  Interleaved and non-interleaved is standard format, this enumeration
 *  captures the standard and custom data formats.
 *
 *  @{
 */
/**
 *  \brief This is used for transfer of data on a single serializer with
 *  multiple slots.please note that the slot data is not interleaved in this
 *  format. TDM with single serializer and slots > 1 uses this format.
 */
#define MCASP_AUDBUFF_FORMAT_1SER_MULTISLOT_NON_INTERLEAVED            (0U)
/**
 *  \brief This is used for transfer of data on a single serializer with
 *  multiple slots.please note that the slot data is interleaved in this format.
 *  TDM with single serializer and slots > 1 uses this format.
 */
#define MCASP_AUDBUFF_FORMAT_1SER_MULTISLOT_INTERLEAVED                (1U)
/**
 *  \brief This is used for transfer of data with  multiple serializers and
 *  also multiple slots enabled.please note that the serializer data is
 *  interleaved in this format. The slot data is also interleaved Refer to the
 *  user guide to view the sample data format
 */
#define MCASP_AUDBUFF_FORMAT_MULTISER_MULTISLOT_SEMI_INTERLEAVED_1     (2U)
/**
 *  \brief This is used for transfer of data with  multiple serializers and
 *  also multiple slots enabled.please note that the serializer data is NOT
 *  interleaved in this format. The slot data is interleaved. Refer to the user
 *  guide to view the sample data format.
 */
#define MCASP_AUDBUFF_FORMAT_MULTISER_MULTISLOT_SEMI_INTERLEAVED_2     (3U)
/** @} */

/**
 *  \anchor MCASP_TransmitState
 *  \name Transmit State for McASP transfer
 *
 *  Enumeration for McASP transmit state
 *
 *  @{
 */
#define MCASP_TRANSMIT_STATE_TX_RESET               (0U)
#define MCASP_TRANSMIT_STATE_TX_FLUSH               (1U)
#define MCASP_TRANSMIT_STATE_LOAD_INIT_BUFFER       (2U)
#define MCASP_TRANSMIT_STATE_RELEASE_FROM_RESET     (3U)
#define MCASP_TRANSMIT_STATE_WAIT_EVENT             (4U)
#define MCASP_TRANSMIT_STATE_PROCESS_EVENT          (5U)
#define MCASP_TRANSMIT_STATE_LOAD_ACTIVE_BUFFER     (6U)
#define MCASP_TRANSMIT_STATE_DONE                   (7U)
#define MCASP_TRANSMIT_STATE_SPIN_IDLE              (8U)
#define MCASP_TRANSMIT_STATE_ERROR                  (9U)
#define MCASP_TRANSMIT_STATE_EXIT                   (10U)
/** @} */

/**
 *  \anchor MCASP_ReceiveState
 *  \name Receive State for McASP transfer
 *
 *  Enumeration for McASP receive state
 *
 *  @{
 */
#define MCASP_RECEIVE_STATE_RX_RESET               (0U)
#define MCASP_RECEIVE_STATE_RX_FLUSH               (1U)
#define MCASP_RECEIVE_STATE_RELEASE_FROM_RESET     (2U)
#define MCASP_RECEIVE_STATE_WAIT_EVENT             (3U)
#define MCASP_RECEIVE_STATE_PROCESS_EVENT          (4U)
#define MCASP_RECEIVE_STATE_DONE                   (5U)
#define MCASP_RECEIVE_STATE_SPIN_IDLE              (6U)
#define MCASP_RECEIVE_STATE_ERROR                  (7U)
#define MCASP_RECEIVE_STATE_EXIT                   (8U)
/** @} */

/* ========================================================================== */
/*                             Structure Definitions                          */
/* ========================================================================== */

/**
 *  \brief Data structure used with transfer call
 *
 */
typedef struct
{
    QueueP_Elem qElem;
    /**< [IN] This should be the first parameter.
     *    this is the queue element used by driver. */
    void *buf;
    /**< [IN] void * to a buffer with data to be transferred .
     *   This parameter can't be NULL */
    uint32_t count;
    /**< [IN/OUT] Number of bytes for this transaction.
      *  This is input incase of read/write call and on API return
      *  this represents number of bytes actually read by the API */
    uint32_t timeout;
    /**< [IN] Timeout for this transaction in units of system ticks */
    int32_t status;
    /**< [OUT] \ref MCASP_TransferStatus code */
    void *args;
    /**< [IN] Argument to be passed to the callback function */
} MCASP_Transaction;

/**
 *  \brief  The definition of a callback function used by the MCASP driver
 *  when used in Callback Mode
 *
 *  \param handle          MCASP_Handle
 *  \param transaction*    Pointer to a #MCASP_Transaction
 */
typedef void (*MCASP_TxCallbackFxn) (MCASP_Handle handle,
                                  MCASP_Transaction *transaction);

typedef void (*MCASP_RxCallbackFxn) (MCASP_Handle handle,
                                  MCASP_Transaction *transaction);

/**
 *  \brief Hardware setup data clock structure
 */
typedef struct
{
    uint32_t aClk;
    /**< Clock details ACLK(R/X)CTL */
    uint32_t hiClk;
    /**< High clock details AHCLK(R/X)CTL */
    uint32_t clkChk;
    /**< Configures RX/TX CLK fail detect */
} MCASP_ClockConfig;

/**
 *  \brief Hardware setup global structure
 */
typedef struct
{
    uint32_t pfunc;
    /**< Pin function register */
    uint32_t pdir;
    /**< Pin direction register */
    uint32_t gblCtl;
    /**< Global control register - GBLCTL */
    uint32_t ditCtl;
    /**< whether McASP operates in DIT mode*/
    uint32_t dlbCtl;
    /**< Digital loopback mode setup */
    uint32_t amute;
    /**< Mute control register - AMUTE */
    uint32_t serSetup[16u];
    /**< Setup serializer control register */
} MCASP_GlobalConfig;

/**
 *  \brief Hardware fifo setup structure
 */
typedef struct
{
    uint32_t fifoCtl;
    /**<  FIFO control register */
    uint32_t fifoStatus;
    /**<  FIFO status register (read only) */
}MCASP_FifoConfig;

/**
 *  \brief Hardware setup data structure
 */
typedef struct
{
    uint32_t mask;
    /**< To mask or not to mask - R/XMASK */
    uint32_t fmt;
    /**< Format details as per  - R/XFMT */
    uint32_t frSyncCtl;
    /**< Configure the rcv/xmt frame sync */
    uint32_t tdm;
    /**< Specifies which TDM slots are active */
    uint32_t intCtl;
    /**< Controls generation of interrupts */
    uint32_t stat;
    /**< Status register (controls writable fields of STAT register)-R/XSTAT */
    uint32_t evtCtl;
    /**< Event control register - R/XEVTCTL */
    MCASP_ClockConfig clk;
    /**< Clock settings for rcv/xmt */
    MCASP_FifoConfig fifoCfg;
    /**< Clock settings for rcv/xmt */
} MCASP_DataConfig;

/**
 *  \brief Hardware setup structure
 */
typedef struct
{
    MCASP_GlobalConfig gbl;
    /**< Value to be loaded in module setup regs */
    MCASP_DataConfig rx;
    /**< Receiver settings */
    MCASP_DataConfig tx;
    /**< Transmitter settings */
} MCASP_HwConfig;

/**
 *  \brief MCASP Parameters
 *
 *  MCASP Parameters are used to with the #MCASP_open() call. Default values for
 *  these parameters are set using #MCASP_openParamsInit().
 *
 *  If NULL is passed for the parameters, #MCASP_open() uses default parameters.
 *
 *  \sa #MCASP_openParamsInit()
 */
typedef struct
{
    int32_t edmaInst;
    /**< EDMA instance used for MCASP transfer */
    uint32_t transferMode;
    /**< Polling, Blocking or Callback mode. */
    uint8_t txBufferFormat;
    /**< Audio buffer format for app tx buffer */
    uint8_t rxBufferFormat;
    /**< Audio buffer format for app rx buffer */
    uint8_t txSerUsedCount;
    /**< Number of allocated transmit serializers */
    uint8_t rxSerUsedCount;
    /**< Number of allocated receive serializers */
    uint8_t *txSerUsedArray;
    /**< POinter to the array of allocated transmit serializer indices */
    uint8_t *rxSerUsedArray;
    /**< POinter to the array of allocated receive serializer indices */
    uint8_t txSlotCount;
    /**< Number of slots for trasnmit operation */
    uint8_t rxSlotCount;
    /**< Number of slots for receive operation */
    MCASP_TxCallbackFxn        txCallbackFxn;
    /**< Read callback function pointer */
    MCASP_RxCallbackFxn        rxCallbackFxn;
    /**< Write callback function pointer */
    uint32_t txLoopjobEnable;
    /**< Flag to enable loopjob for transmit */
    uint8_t *txLoopjobBuf;
    /**< Loopjob buffer address for transmit */
    uint32_t txLoopjobBufLength;
    /**< Loopjob buffer length for transmit */
    uint32_t rxLoopjobEnable;
    /**< Flag to enable loopjob for receive */
    uint8_t *rxLoopjobBuf;
    /**< Loopjob buffer address for receive */
    uint32_t rxLoopjobBufLength;
    /**< Loopjob buffer length for receive */
} MCASP_OpenParams;

/**
 *  \brief McASP Interrupt structures
 */
typedef struct
{
    uint32_t intrNum;
    /**< Receive interrupt number */
    uint8_t intrPriority;
    /**< Interrupt priority */
} MCASP_HwIntConfig;

/**
 *  \brief McASP Transfer Data structure stored in driver object
 *
 */
typedef struct
{
    uint32_t inProgress;
    /**< Flag to indicate if the transfer is ongoing */
    uint32_t state;
    /**< MCASP Transfer State */
    int32_t status;
    /**< MCASP_Transfer Status */
    uint32_t count;
    /**< MCASP Transfer count */
    uint8_t slotCount;
    /**<  number of slots in transaction */
    uint8_t slotIndex;
    /**<  current slot index in ongoing transfer */
    uint8_t frameCount;
    /**<  number of frames in transaction */
    uint8_t frameIndex;
    /**< current frame index in ongoing transfer */
    uint8_t serCount;
    /**< Number of allocated serializers */
    uint8_t *serArray;
    /**< Pointer to the array of allocated serializer indices */
    uint8_t bufferFormat;
    /**< Audio buffer format for app buffer */
    MCASP_Transaction *transaction;
    /**< Pointer to current transaction struct */
    MCASP_TxCallbackFxn cbFxn;
    /**< callback function provided by the app to be called by DMA or Host */
    /**< Audio buffer format for app buffer */
    MCASP_Transaction txnLoopjob;
    /**< transaction struct object for loopjob buffer */
    uint32_t loopjobEnable;
    /**< Flag to enable loopjob. when enabled loopjob buffer is transmitted if application fails to load buffers.
     *   If disabled minimum of 2 buffers need to be submitted before start and atleast one buffer should be queued before each ISR call. */
} MCASP_TransferObj;

/**
 *  \brief MCASP edma configuration object
 */
typedef struct
{
    uint32_t baseAddr;
    /**< EDMA instance base address */
    uint32_t regionId;
    /**< EDMA region Id */
    uint32_t chId;
    /**< DMA Channel Number. This should be the MCASP Event number */
    uint32_t paramId;
    /**< EDMA parameter ram Id */
    uint32_t tccId;
    /**< EDMA transfer completion code */
    uint32_t linkParamId;
    /**< EDMA link parameter ram Id */
    Edma_IntrObject intrObj;
}MCASP_EdmaConfig;

/**
 *  \brief MCASP driver object
 */
typedef struct
{
    /*
     * User params
     */
    MCASP_Handle handle;
    /**< Instance handle */
    uint16_t instNum;
    /**< Instance number in port */
    uint32_t drvState;
    /**< stores the current state of the driver */
    uint32_t transferMode;
    /**< Polling, Blocking or Callback mode. */
    int32_t edmaInst;
    /**< EDMA instance used for MCASP transfer */
    MCASP_TransferObj XmtObj;
    /**< Holds transmit channel to the McASP. */
    MCASP_TransferObj RcvObj;
    /**< Holds receive channel to the McASP. */

    /*
     * State variables
     */
    uint32_t isOpen;
    /**< Flag to indicate if the instance is already open */
    uint32_t isTxStarted;
    /**< Flag to indicate if the Tx is started */
    uint32_t isRxStarted;
    /**< Flag to indicate if the Rx is started */
    SemaphoreP_Object lockObj;
    /**< Driver lock object */
    SemaphoreP_Object transferSemObj;
    /**< Transfer Sync Semaphore object */
    HwiP_Object hwiObjTx;
    /**< Transmit Interrupt object */
    HwiP_Object hwiObjRx;
    /**< Receive Interrupt object */
    MCASP_EdmaConfig xmtDmaObj;
    /**< transmit dma object */
    MCASP_EdmaConfig rcvDmaObj;
    /**< receive dma object */
    /**< number of slots used by the mcasp */
    QueueP_Object reqQueueObjTx, curentQueueObjTx, reqQueueObjRx, curentQueueObjRx;
    /**< Queue Obj to store the application buffers */
    QueueP_Handle reqQueueHandleTx, curentQueueHandleTx, reqQueueHandleRx, curentQueueHandleRx;
    /**< Queue handle used for storing the application buffers */
} MCASP_Object;

/** \brief MCASP instance attributes - used during init time */
typedef struct
{
    /*
     * SOC configuration
     */
    uintptr_t baseAddr;
    /**< Peripheral base address */
    uintptr_t dataBaseAddr;
    /**< Peripheral data port base address */
    uint16_t numOfSerializers;
    /**< Number of serializers */
    uint16_t serStatus[16];
    /**< Holds status information for both the serializers */
    MCASP_HwConfig hwCfg;
    /**< Register information for initialising the Mcasp hardware. */
    MCASP_HwIntConfig intCfgTx;
    /**< Transmit interrupt configuration */
    MCASP_HwIntConfig intCfgRx;
    /**< Receive interrupt configuration */
    uint32_t edmaChTx;
    /**< edma event for tx */
    uint32_t edmaChRx;
    /**< edma event for rx */
    uint32_t txSlotSize;
    /**< slot size for transmission */
    uint32_t rxSlotSize;
    /**< slot size for reception */
} MCASP_Attrs;

typedef struct
{
    const MCASP_Attrs *attrs;
    /**< Pointer to driver specific hardware attributes */
    MCASP_Object *object;
    /**< Pointer to driver specific data object */
} MCASP_Config;

/** \brief Externally defined driver configuration array */
extern MCASP_Config gMcaspConfig[];
/** \brief Externally defined driver configuration array size */
extern uint32_t gMcaspConfigNum;


/* ========================================================================== */
/*                          Function Declarations */
/* ========================================================================== */

/**
 *  \brief  This function initializes the MCASP module
 */
void MCASP_init(void);

/**
 *  \brief  This function de-initializes the MCASP module
 */
void MCASP_deinit(void);

/**
 *  \brief  Function to initialize the #MCASP_OpenParams struct to its defaults
 *
 *  \param  openPrms    Pointer to #MCASP_OpenParams structure for
 *                      initialization
 */
static inline void MCASP_openParamsInit(MCASP_OpenParams *openPrms);

/**
 *  \brief  This function opens a given MCASP peripheral
 *
 *  \pre    MCASP controller has been initialized using #MCASP_init()
 *
 *  \param  index       Index of config to use in the *MCASP_Config* array
 *  \param  openParams  Pointer to parameters to open the driver with
 *
 *  \return A #MCASP_Handle on success or a NULL on an error or if it has been
 *          opened already
 *
 *  \sa     #MCASP_init()
 *  \sa     #MCASP_close()
 */
MCASP_Handle MCASP_open(uint32_t index, const MCASP_OpenParams *openParams);

/**
 *  \brief  Function to close a MCASP peripheral specified by the MCASP handle
 *
 *  \pre    #MCASP_open() has to be called first
 *
 *  \param  handle      #MCASP_Handle returned from #MCASP_open()
 *
 *  \sa     #MCASP_open()
 */
void MCASP_close(MCASP_Handle handle);

/**
 *  \brief  This function returns the handle of an open MCASP Instance from the
 *          instance index
 *
 *  \pre    MCASP controller has been opened using #MCASP_open()
 *
 *  \param  index Index of config to use in the *MCASP_Config* array
 *
 *  \return A #MCASP_Handle if it has been opened already or NULL otherwise
 *
 *  \sa     #MCASP_init()
 *  \sa     #MCASP_open()
 */
MCASP_Handle MCASP_getHandle(uint32_t index);

/**
 * \brief   Function to submit the buffer to McASP driver for transmission.
 *          Transaction object is held by the driver till it is returned in the callback function.
 *          It is recommended not to allocate the transaction object in stack to avoid corruption.
 *
 * \param handle MCASP_Handle
 * \param txn* Pointer to a #MCASP_Transaction
 *
 * \return  Success/Failure for configuration
 *
 */
int32_t MCASP_submitTx(MCASP_Handle handle, MCASP_Transaction *txn);

/**
 * \brief   Function to submit the buffer to McASP driver for reception.
 *          Transaction object is held by the driver till it is returned in the callback function.
 *          It is recommended not to allocate the transaction object in stack to avoid corruption.
 *
 * \param handle MCASP_Handle
 * \param txn* Pointer to a #MCASP_Transaction
 *
 * \return  Success/Failure for configuration
 *
 */
int32_t MCASP_submitRx(MCASP_Handle handle, MCASP_Transaction *txn);

/**
 * \brief   Function to withdraw the buffer submitted to McASP driver for transmission.
 *          This should be called after the MCASP_stopTransferTx.
 *          These buffers are not transmitted.
 *
 * \param handle MCASP_Handle
 *
 * \return Pointer to a #MCASP_Transaction. NULL if no more buffers are there with driver.
 *
 */
MCASP_Transaction* MCASP_withdrawTx(MCASP_Handle handle);

/**
 * \brief   Function to withdraw the buffer submitted to McASP driver for reception.
 *          This should be called after the MCASP_stopTransferRx.
 *          These buffers are not transmitted.
 *
 * \param handle MCASP_Handle
 *
 * \return Pointer to a #MCASP_Transaction. NULL if no more buffers are there with driver.
 *
 */
MCASP_Transaction* MCASP_withdrawRx(MCASP_Handle handle);

/**
 * \brief   Function to start McASP transmission.
 *
 * \param handle MCASP_Handle
 *
 * \return  Success/Failure for configuration
 *
 */
int32_t MCASP_startTransferTx(MCASP_Handle handle);

/**
 * \brief   Function to start McASP reception.
 *
 * \param handle MCASP_Handle
 *
 * \return  Success/Failure for configuration
 *
 */
int32_t MCASP_startTransferRx(MCASP_Handle handle);

/**
 * \brief   Function to stop McASP transmission.
 *
 * \param handle MCASP_Handle
 *
 * \return  Success/Failure for configuration
 *
 */
int32_t MCASP_stopTransferTx(MCASP_Handle handle);

/**
 * \brief   Function to stop McASP reception.
 *
 * \param handle MCASP_Handle
 *
 * \return  Success/Failure for configuration
 *
 */
int32_t MCASP_stopTransferRx(MCASP_Handle handle);

/* ========================================================================== */
/*                       Static Function Definitions */
/* ========================================================================== */

static inline void MCASP_openParamsInit(MCASP_OpenParams *openPrms)
{
    if (openPrms != NULL)
    {
        // Set Default values
    }
}

#ifdef __cplusplus
}
#endif

#endif /* #ifndef MCASP_H_ */

/** @} */
