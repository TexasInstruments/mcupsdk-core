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
 *  \defgroup DRV_MIBSPI_MODULE APIs for MIBSPI
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the MIBSPI module. The APIs
 *  can be used by other drivers to get access to MIBSPI and also by
 *  application to initiate data transfer operation.
 *
 *  @{
 */

/**
 *  \file v0/mibspi.h
 *
 *  \brief MIBSPI Driver API/interface file.
 */

#ifndef MIBSPI_H_
#define MIBSPI_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/hw_include/cslr_mibspi.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/edma.h>
#include <drivers/mibspi/v0/soc/mibspi_soc.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief A handle that is returned from a #MIBSPI_open() call */
typedef void *MIBSPI_Handle;

/** \brief MibSPI RAM buffer mode */
#define MIBSPI_RAM_BUFFER_MODE              (7U)

/** \brief Transfer group used in slave mode */
#define MIBSPI_SLAVEMODE_TRANS_GROUP        (0U)

/** \brief MIBSPI interrupt level */
#define MIBSPI_INT_LEVEL                    (1U)

/**
 *  @defgroup MIBSPI_FEATURE MIBSPI Feature defines
 *  @ingroup  MIBSPI_DRIVER_INTERNAL_DATA_STRUCTURE
 *  \brief    MIBSPI_FEATURE_* macros indicate presence of a particular features in
 *  SPI instance on the given SoC. Used internally in driver
 *  @{
 */
/** \brief MIBSPI Parallel mode operation Enable */
#define MIBSPI_FEATURE_PARALLEL_MODE_ENA        (0x1U << 0)
/** \brief MIBSPI Parallel mode operation Disable */
#define MIBSPI_FEATURE_PARALLEL_MODE_DIS        (0U)
/** \brief MIBSPI Modulo mode operation Enable */
#define MIBSPI_FEATURE_MODULO_MODE_ENA          (0x1U << 1)
/** \brief MIBSPI Modulo mode operation Disable */
#define MIBSPI_FEATURE_MODULO_MODE_DIS          (0U)
/** \brief MIBSPI SPIENA pin Enable */
#define MIBSPI_FEATURE_SPIENA_PIN_ENA           (0x1U << 2)
/** \brief MIBSPI SPIENA pin Disable */
#define MIBSPI_FEATURE_SPIENA_PIN_DIS           (0U)
/** @} */

/**
 *  \brief    Maximum element in MibSPI RAM.
 *            For 8bit data size, the maximum transfer size is 64 bytes
 *            For 16bit data size, the maximum transfer size is 128 bytes
 */
#define MIBSPI_RAM_MAX_ELEM          (128U)

/** \brief Max number of slaves supported when MibSPI is configured as master */
#define MIBSPI_SLAVE_MAX             (3U)

/** \brief Max number of DMA REQ lines supported by MIBSPI */
#define MIBSPI_DMA_REQLINE_MAX       (3U)

/** \brief Maximum CS supported for the device */
#define MIBSPI_MAX_CS                (MIBSPI_SLAVE_MAX)

/** \brief Max number of transport group */
#define MIBSPI_NUM_TRANS_GROUP       (8U)

/** \brief Max number of DMA group */
#define MIBSPI_NUM_DMA_GROUP         (8U)

/** \brief CS configuration value when none of the CS is activated */
#define MIBSPI_CS_NONE               (0xFFU)

/* ========================================================================== */
/*                           enums                                            */
/* ========================================================================== */

/** \brief Enum used to track status of EDMA callbacks internally in driver */
enum MIBSPI_EdmaCBFlag_e
{
    MIBSPI_NONE_EDMA_CALLBACK_OCCURED  =   0x0U,
    /**< Flag to indicate that Transmission EDMA callback has occured         */
    MIBSPI_TX_EDMA_CALLBACK_OCCURED    =   0x1U,
    /**< Flag to indicate that Transmission EDMA callback has occured         */
    MIBSPI_RX_EDMA_CALLBACK_OCCURED    =   0x2U,
    /**< Flag to indicate that Reception EDMA callback has occured            */
};

/** \brief  SPI_v3 transfer error code */
enum MIBSPI_XferErr_e {
    MIBSPI_XFER_ERR_NONE,   /* No transfer error */
    MIBSPI_XFER_ERR_RXOR,   /* Receiver overrun error */
    MIBSPI_XFER_ERR_BE,     /* Bit error */
    MIBSPI_XFER_ERR_TIMEOUT /* Transfer timeout error */
};

/** \brief      Status codes that are set by the SPI driver. */
typedef enum {
    MIBSPI_TRANSFER_COMPLETED = 0,
    MIBSPI_TRANSFER_STARTED,
    MIBSPI_TRANSFER_CANCELED,
    MIBSPI_TRANSFER_FAILED,
    MIBSPI_TRANSFER_CSN_DEASSERT,
    MIBSPI_TRANSFER_TIMEOUT
} MIBSPI_Status;

/** \brief Definitions for MIBSPI Pins Operation Mode. */
typedef enum {
    MIBSPI_PINMODE_3PIN     = 0,      /**< 3 Pin operation Mode */
    MIBSPI_PINMODE_4PIN_CS  = 1       /**< 4 Pin operation Mode with CS signal */
} MIBSPI_PinMode;

/**  \brief Definitions for MIBSPI Data shift format. */
typedef enum {
    MIBSPI_MSB_FIRST = 0,    /**< MSB shift out first */
    MIBSPI_LSB_FIRST = 1     /**< LSB shift out first */
} MIBSPI_DataShiftFmt;

/** \brief  Definitions for  MIBSPI loopback modes. */
typedef enum {
    MIBSPI_LOOPBK_DIGITAL = 0,    /**< MIBSPI digital loopback  mode */
    MIBSPI_LOOPBK_ANALOG  = 1,    /**< MIBSPI analog loopback  mode */
    MIBSPI_LOOPBK_NONE    = 2,    /**< MIBSPI loopback  disable */
} MIBSPI_LoopBackType;

/** \brief Definitions for various SPI modes of operation. */
typedef enum {
    MIBSPI_MASTER      = 0,    /**< SPI in master mode */
    MIBSPI_SLAVE       = 1     /**< SPI in slave mode */
} MIBSPI_Mode;

/**  \brief Definitions for various SPI data frame formats. */
typedef enum {
    MIBSPI_POL0_PHA0   = 0,    /**< SPI mode Polarity 0 Phase 0 */
    MIBSPI_POL0_PHA1   = 1,    /**< SPI mode Polarity 0 Phase 1 */
    MIBSPI_POL1_PHA0   = 2,    /**< SPI mode Polarity 1 Phase 0 */
    MIBSPI_POL1_PHA1   = 3,    /**< SPI mode Polarity 1 Phase 1 */
} MIBSPI_FrameFormat;

/**
 *  \brief
 *
 *  MIBSPI transfer mode determines the whether the MIBSPI controller operates
 *  synchronously or asynchronously. In #MIBSPI_MODE_BLOCKING mode MIBSPI_transfer()
 *  blocks code execution until the MIBSPI transaction has completed. In
 *  #MIBSPI_MODE_CALLBACK MIBSPI_transfer() does not block code execution and instead
 *  calls a #MIBSPI_CallbackFxn callback function when the transaction has
 *  completed.
 */
typedef enum {
    /**
     * MIBSPI_transfer() blocks execution. This mode can only be used when called
     * within a Task context
     */
    MIBSPI_MODE_BLOCKING,
    /**
     * MIBSPI_transfer() does not block code execution and will call a
     * #MIBSPI_CallbackFxn. This mode can be used in a Task, Swi, or Hwi context.
     */
    MIBSPI_MODE_CALLBACK
} MIBSPI_TransferMode;

/**  \brief Definitions for DMA controller channel type. */
typedef enum {
    MIBSPI_DMACTRL_CH_TX,   /**< Configure DMA channel as transmit only */
    MIBSPI_DMACTRL_CH_RX,   /**< Configure DMA channel as receive only */
    MIBSPI_DMACTRL_CH_BOTH, /**< Configure DMA channel as transmit and receive */
} MIBSPI_DmaCtrlChType;

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 * \brief
 *  SPI Driver Statistics
 *
 *  The structure is used to store driver statistics
 */
typedef struct
{
    /** \brief   Number of data length error interrupts */
    uint32_t    dlenErr;
    /** \brief   Number of timeout interrupts */
    uint32_t    timeout;
    /** \brief   Number of Parity Error interrupts */
    uint32_t    parErr;
    /** \brief   Number of De-synchronization of slave device interrupts */
    uint32_t    desync;
    /** \brief   Number of mismatch of internal transmit data and transmittted data error interrupts */
    uint32_t    bitErr;
    /** \brief   Number of RX Overrun Error Interrupts */
    uint32_t    rxOvrnErr;
    /** \brief   Number of RX Full Interrupts */
    uint32_t    rxFull;
    /** \brief   Number of TX Empty Interrupts */
    uint32_t    txEmpty;
    /** \brief   Number of TG Complete interrupts */
    uint32_t    tgComplete[MIBSPI_NUM_TRANS_GROUP];
}MIBSPI_HW_Stats;

/**  \brief MIBSPI_Stats data structure is used with SPI_getStats() to get driver statistics. */
typedef struct
{
    uint32_t   dlenErr;     /**<Number of data length error */
    uint32_t   timeout;     /**<Number of timeout */
    uint32_t   parErr;      /**<Number of Parity error */
    uint32_t   desync;      /**<Number of De-synchronization of slave device - master only */
    uint32_t   bitErr;      /**<Number of data mismatch transmit data error - master only */
    uint32_t   rxOvrnErr;   /**<Number of RX Overrun Error */
} MIBSPI_Stats;

/**
 *  \brief MIBSPI slave profile Parameters
 *
 *  When MIBSPI is configured as master mode, it may connect to multiple slaves. This data structure captures
 *  the the configurations for remote slaves.
 */
typedef struct
{
    uint8_t             chipSelect;   /**< CS0-CS7 signal number from 0 -7 */
    uint8_t             ramBufLen;    /**< RAM Length in bytes that used by the slave */
    uint32_t            dmaReqLine;   /**< DMA request line to be used for slave */
} MIBSPI_SlaveProfile;

/**
 *  \brief MIBSPI slave mode Parameters
 *
 *  MIBSPI slave Parameters are used with the MIBSPI_open() call when mode is set to MIBSPI_SLAVE.
 *  Default values for these parameters are set using MIBSPI_Params_init().
 *
 *  @sa         MIBSPI_Params_init()
 */
typedef struct
{
    uint32_t            dmaReqLine;    /**< DMA request line to be used for Slave */
    uint8_t             chipSelect;    /**< CS0-CS7 signal number from 0 -7 */
} MIBSPI_SlaveModeParams;

/**
 * \brief
 *  SPI Driver Info for Master
 *
 * @details
 *  The structure is used to store the driver info for Master.
 */
typedef struct
{
    /** \brief   Number of DMA Recieve Interrupts */
    uint32_t    rxDmaIntCnt;
    /** \brief   Number of DMA Transmit Interrupts */
    uint32_t    txDmaIntCnt;
} MIBSPI_DriverDmaInfo;

/**
 * \brief
 *  MIBSPI Driver DMA request line tie up for the SoC
 *
 *  The structure is used to store the hardware specific configuration for DMA request lines.
 */
typedef struct
{
    /** \brief   TX DMA Request Line number */
    uint8_t     txDmaReqLine;
    /** \brief   RX DMA Request Line number  */
    uint8_t     rxDmaReqLine;
}MIBSPI_DMAReqlineCfg;

/**
 * \brief
 *  MIBSPI Driver DMA module Address info definition
 *
 *  Data structure used to exchange address info between the MIBSPI driver and
 *  the SoC specific DMA engine implementation
 */
typedef struct
{
    uintptr_t saddr; /**< Source address of transfer. Used by DMA engine so should be SoC system address */
    uintptr_t daddr; /**< Destination address of transfer. Used by DMA engine so should be SoC system address */
} MIBSPI_DMAXferAddrInfo;

/**
 * \brief
 *  MIBSPI Driver DMA module Transfer size definition
 *
 *  Data structure used to exchange size of transaction between the MIBSPI driver and
 *  the SoC specific DMA engine implementation
 *
 *  @ingroup MIBSPI_DRIVER_INTERNAL_DATA_STRUCTURE
 *
 */
typedef struct
{
    uint16_t elemSize; /**< Element Size of transfer in bytes */
    uint16_t elemCnt;  /**< Number of elements */
    uint16_t frameCnt; /**< Number of frames of elemCnt */
} MIBSPI_DMAXferSizeInfo;

/**
 * \brief
 *  MIBSPI Driver DMA module Transfer Info definition
 *
 *  Data structure used to exchange transaction info between the MIBSPI driver and
 *  the SoC specific DMA engine implementation
 */
typedef struct
{
    MIBSPI_DMAXferAddrInfo tx;   /**< Transation Transmit info */
    MIBSPI_DMAXferAddrInfo rx;   /**< Transation Receive info */
    MIBSPI_DMAXferSizeInfo size; /**< Transation size info */
    uint32_t  dmaReqLine;        /**< DMA request line to be used for the transaction */
} MIBSPI_DMAXferInfo;

/**
 *  \brief
 *  MIBSPI_Transaction data structure is used with MIBSPI_transfer(). It indicates
 *  how many MIBSPI_FrameFormat frames are sent and received from the buffers
 *  pointed to txBuf and rxBuf.
 */
typedef struct
{
    /* User input (write-only) fields */
    uint32_t   count;      /**< Number of frames for this transaction */
    void      *txBuf;      /**< void * to a buffer with data to be transmitted */
    void      *rxBuf;      /**< void * to a buffer to receive data */
    void      *arg;        /**< Argument to be passed to the callback function */
    /* User output (read-only) fields */
    MIBSPI_Status status;     /**< Status code set by SPI_transfer */
    uint8_t   slaveIndex; /**< Index of the slave enabled for this transfer */
} MIBSPI_Transaction;

/**
 * \brief
 *  MIBSPI Driver DMA module Tranaction state definition
 *
 *  Data structure used to exchange transaction state info between the MIBSPI driver and
 *  the SoC specific DMA engine implementation
 */
typedef struct
{
    volatile enum MIBSPI_EdmaCBFlag_e edmaCbCheck;    /**< EDMA call back check */
    enum MIBSPI_XferErr_e             transferErr;    /**< SPI_v3 transfer error codes
                                                           refer to #MIBSPI_XferErr_e */
    MIBSPI_Transaction               *transaction;    /**< Transaction data structure
                                                       refer to #MIBSPI_Transaction */
    volatile uint16_t                 remainSize;     /**< Remaining size after after first transfer */
    volatile uint16_t                 dataLength;     /**< Remaining data length after first transfer */
    volatile uint16_t                 dataSizeInBytes;/**< Remaining data size in bytes after first transfer */
} MIBSPI_TransactionState;

/**
 *  \brief      The definition of a callback function used by the SPI driver
 *              when used in #MIBSPI_MODE_CALLBACK
 *
 *  @param      handle          MIBSPI_Handle
 *  @param      transaction*    MIBSPI_Transaction*
 */
typedef void        (*MIBSPI_CallbackFxn) (MIBSPI_Handle handle,
                                           MIBSPI_Transaction *transaction);

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/**
 *  \brief MIBSPI master Parameters
 *
 *  MIBSPI master Parameters are used  with the MIBSPI_open() call when mode is set to SPI_MASTER.
 *  Default values for these parameters are set using MIBSPI_Params_init().
 *
 *  @sa         MIBSPI_Params_init()
 */
typedef struct
{
    uint32_t            bitRate;                        /**< SPI bit rate in Hz */
    uint8_t             numSlaves;                      /**< Number of slaves connected to SPI master */
    uint8_t             t2cDelay;                       /**< Transmit end to chip select inactive delay */
    uint8_t             c2tDelay;                       /**< Chip select active to transmit start delay */
    uint8_t             wDelay;                         /**< Delay in between transmissions */
    MIBSPI_SlaveProfile    slaveProf[MIBSPI_SLAVE_MAX]; /**< Slave profile for each slave */
} MIBSPI_MasterModeParams;

/**
 *  \brief MIBSPI EDMA Parameters
 *
 *  Used to store the EDMA parameters allocated for MIBPI transfer.
 *
 */
typedef struct
{
    uint32_t edmaTcc;
    /**< EDMA TCC used for MIBSPI transfer */
    uint32_t edmaChId;
    /**< EDMA Channel used for MIBSPI transfer */
    uint32_t edmaParam;
    /**< EDMA Param ID used for MIBSPI transfer */
} MIBSPI_EDMAChParams;

/**
 *  \brief MIBSPI Parameters
 *
 *  MIBSPI Parameters are used with the MIBSPI_open() call. Default values for
 *  these parameters are set using MIBSPI_Params_init().
 *
 *  @sa         MIBSPI_Params_init()
 */
typedef struct
{
    MIBSPI_Mode            mode;                /**< Master or Slave mode */
    MIBSPI_TransferMode    transferMode;        /**< Blocking or Callback mode */
    Bool                   iCountSupport;       /**< Support for multi icount in one transfer
                                                     to achieve high throughput, this
                                                     is only supported in blocking mode */
    MIBSPI_CallbackFxn     transferCallbackFxn; /**< Callback function pointer */
    uint32_t               dataSize;            /**< SPI data frame size in bits */
    MIBSPI_FrameFormat     frameFormat;         /**< SPI frame format */
    union
    {
        MIBSPI_MasterModeParams  masterParams;  /**< Configuration dedicated to master mode,
                                                     refer to #MIBSPI_MasterModeParams */
        MIBSPI_SlaveModeParams   slaveParams;   /**< Configuration dedicated to slave mode,
                                                     refer to #MIBSPI_SlaveModeParams */
    }u;                                         /**< union to hold either master mode or
                                                     slave mode params */
    MIBSPI_PinMode         pinMode;             /**< Pin operation Mode, refer to
                                                     #MIBSPI_PinMode */
    MIBSPI_DataShiftFmt    shiftFormat;         /**< SPI Data shift format, refer to
                                                     #MIBSPI_DataShiftFmt */
    uint8_t                dmaEnable;           /**< DMA mode enable. When Dma mode is disable,
                                                     SPI driver is operated in polling mode, there
                                                     is while(1) loop waiting for hardware to finish */
    int32_t                edmaInst;            /**< EDMA instance */
    void                   *dmaHandle;         /**< EDMA handle */
    uint8_t                eccEnable;           /**< ECC mode enable */
    uint8_t                csHold;              /**< CS Hold enable */
    uint16_t               txDummyValue;        /**< 16 bit value transmitted when no TX data is provided for SPI_tranfer() */
    uint32_t               transferTimeout;     /*!< Transfer timeout in system ticks
                                                     (Not supported with all implementations */
    Bool                   compatibilityMode;   /**< Enable Compatibility mode operation of MIBSPI.
                                                     MIBSPI RAM will be disabled */
    uint16_t               edmaLinkParamId;     /**< Link param id.
                                                     \brief  EDMA link channel id
                                                     Needs to be set only if compatibility mode is TRUE
                                                     The MIBSPI requries an additional param to terminate Tx transfer.
                                                     This should be from the pool of free param sets  */
} MIBSPI_OpenParams;

/**
 * \brief
 *  MIBSPI Driver HW configuration
 *
 *  The structure is used to store the hardware specific configuration which is
 *  passed to MIBSPI driver instance
 */
typedef struct
{
    /** \brief mibspi instance ID */
    uint32_t                 mibspiInstId;
    /** \brief   Base address of the MibSpi register address space to be used. */
    CSL_mss_spiRegs         *ptrSpiRegBase;
    /** \brief   Base address of the MibSpi ram address space to be used. */
    CSL_mibspiRam           *ptrMibSpiRam;
    /** \brief   MIBSPI clock source frequency in Hz, It will be used to calculate prescaler for Master mode */
    uint32_t                 clockSrcFreq;
    /** \brief   Interrupt Number for INT0 */
    uint32_t                 interrupt0Num;
    /** \brief   Interrupt Number for INT1 */
    uint32_t                 interrupt1Num;
    /** \brief Size of MIBSPI RAM in this SoC for this MIBSPI instance  */
    uint32_t                 mibspiRamSize;
    /** \brief Number of transfer groups for this MIBSPI instance  */
    uint32_t                 numTransferGroups;
    /**
     * \brief Number of parallel mode pins supported for this MIBSPI instance
     *
     * If MIBSPI_FEATURE_PARALLEL_MODE is not enabled for this MIBSPI instance
     * this value is dont care and will be set to 0
     */
    uint32_t                 numParallelModePins;
    /**
     * \brief Optional feature that are supported for this MIBSPI instance
     *        This is a bitmap comprising of defines @ref MIBSPI_FEATURE
     *        Bit being set indicates presence of the features for the MIBSPI
     *        instance
     */
    uint32_t                 featureBitMap;
    /** \brief Number of independent DMA request lines supported by MIBSPI */
    uint32_t                 numDmaReqLines;
    /** \brief   MIBSPI DMA reqline definition */
    MIBSPI_DMAReqlineCfg     dmaReqlineCfg[MIBSPI_DMA_REQLINE_MAX];
} MIBSPI_Attrs;

/**
 *  \brief      SPI Driver Object
 *
 *  The structure is used to store the SPI driver internal variables.
 *  The application must not access any member variables of this structure!
 *
 */
typedef struct
{
    /** \brief   SPI driver handle.  */
    MIBSPI_Handle             mibspiHandle;
    /** \brief   Pointer to MibSpi driver Hardware Configuration */
    const MIBSPI_Attrs        *ptrHwCfg;
    /** \brief   MibSpi driver parameters */
    MIBSPI_OpenParams          params;
    /** \brief   DMA information used in transfer for remote peers */
    MIBSPI_DriverDmaInfo       dmaInfo[MIBSPI_SLAVE_MAX];
    /** \brief   MibSPI mode RAM offset start settings for remote peers */
    uint8_t                    rambufStart[MIBSPI_SLAVE_MAX];
    /** \brief   MibSPI mode RAM offset end settings for remote peers */
    uint8_t                    rambufEnd[MIBSPI_SLAVE_MAX];
    /**
     * \brief   Rx Scratch buffer, used as scratch buffer to dump received
     *          data from SPI transfer when application does not provide
     *          receive buffer.
     */
    uint16_t                   rxScratchBuffer;
    /**
     * \brief   Tx Scratch buffer, used when TX data is not provided for SPI_transfer()
     *          The driver transmits txDummyValue from #MIBSPI_OpenParams
     */
    uint16_t                   txScratchBuffer;
     /*
     * State variables
     */
    uint32_t                   isOpen;
    /**< Flag to indicate whether the instance is opened already */
    void                      *transferSem;
    /**< Transfer Sync Sempahore - to sync between transfer completion ISR
     *   and task */
    SemaphoreP_Object          transferSemObj;
    /**< Transfer Sync Sempahore object */
    void                      *hwiHandle;
    /**< Interrupt handle for master ISR */
    HwiP_Object                hwiObj;
    /**< Interrupt object */

    /** \brief   MibSpi driver stats. */
    MIBSPI_HW_Stats     hwStats;
    /** \brief  State info on current transaction */
    MIBSPI_TransactionState transactionState;
    /**\brief EDMA Base address used for MIBSPI transfer */
    uint32_t                edmaBaseAddr;
    /**\brief EDMA Region used for MIBSPI transfer */
    uint32_t                edmaRegionId;
    /** \brief EDMA Tx channel parameters allocated for MIBSPI */
    MIBSPI_EDMAChParams     edmaParamsTx[MIBSPI_DMA_REQLINE_MAX];
    /** \brief EDMA Rx channel parameters allocated for MIBSPI */
    MIBSPI_EDMAChParams     edmaParamsRx[MIBSPI_DMA_REQLINE_MAX];
    /** \brief EDMA TX intr object */
    Edma_IntrObject         intrTxObj;
    /** \brief EDMA RX intr object */
    Edma_IntrObject         intrRxObj;
    /** \brief EDMA Dummy intr object */
    Edma_IntrObject         intrDummyObj;
} MIBSPI_Object;

/**
 *  \brief MIBSPI Instance configuration
 *
 *  The MIBSPI_Config structure contains a set of pointers to MIBSPI instance
 *  object and the SoC info associated with the specific MIBSPI instance
 *
 */
typedef struct
{
    const MIBSPI_Attrs      *attrs;
    /**< Pointer to driver specific attributes */
    MIBSPI_Object           *object;
    /**< Pointer to driver specific data object */
} MIBSPI_Config;

/** \brief Externally defined driver configuration array */
extern MIBSPI_Config         gMibspiConfig[];
/** \brief Externally defined driver configuration array size */
extern uint32_t              gMibspiConfigNum;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief  This function initializes the MIBSPI module.
 *
 */
void MIBSPI_init(void);

/**
 *  \brief  This function de-initializes the MIBSPI module
 */
void MIBSPI_deinit(void);

/**
 *  \brief  This function opens a given instance of the MIBSPI peripheral.
 *
 *  @pre    MIBSPI controller has been initialized using #MIBSPI_init()
 *
 *  @param  index         Id of the SPI instance.
 *
 *  @param  params        Pointer to an parameter block, if NULL it will use
 *                        default values.
 *
 *  @return A MIBSPI_Handle on success or a NULL on an error or if it has been
 *          opened already.
 *
 *  @sa     MIBSPI_init()
 *  @sa     MIBSPI_close()
 *  @sa     MIBSPI_Params_init()
 */
MIBSPI_Handle MIBSPI_open(uint32_t index, MIBSPI_OpenParams *params);

/**
 *  \brief  Function to close a instance of a MIBSPI peripheral specified by the MIBSPI handle
 *
 *  @pre    MIBSPI_open() has to be called first.
 *
 *  @param  handle  #MIBSPI_Handle returned from #MIBSPI_open()
 *
 *  @sa     MIBSPI_open()
 */
void MIBSPI_close(MIBSPI_Handle handle);

/**
 *  \brief  Function to perform MIBSPI transactions on a instance of
 *          a MIBSPI peripheral specified by the MIBSPI handle for a specific slave.
 *
 *  If the MIBSPI is in #MIBSPI_MASTER mode, it will immediately start the
 *  transaction. If the MIBSPI is in #MIBSPI_SLAVE mode, it prepares itself for a
 *  transaction with a SPI master.
 *
 *  In #MIBSPI_MODE_BLOCKING, MIBSPI_transfer will block task execution until the
 *  transaction has completed.
 *
 *  In #MIBSPI_MODE_CALLBACK, MIBSPI_transfer() does not block task execution and
 *  calls a #MIBSPI_CallbackFxn. This makes the MIBSPI_tranfer() safe to be used
 *  within a Task, Swi, or Hwi context. The #MIBSPI_Transaction structure must
 *  stay persistent until the MIBSPI_transfer function has completed!
 *
 *  @param  handle         #MIBSPI_Handle
 *
 *  @param  transaction    A pointer to a MIBSPI_Transaction. All of the fields within
 *                         transaction except MIBSPI_Transaction.count and
 *                         MIBSPI_Transaction.status are WO (write-only) unless
 *                         otherwise noted in the driver implementations. If a
 *                         transaction timeout has occured, MIBSPI_Transaction.count
 *                         will contain the number of frames that were transferred.
 *
 *  @return true if started successfully; else false
 *
 *  @sa     MIBSPI_open
 *  @sa     MIBSPI_transferCancel
 */
int32_t MIBSPI_transfer(MIBSPI_Handle handle, MIBSPI_Transaction *transaction);

/**
 *  \brief  Function to cancel MIBSPI transactions on instance of a
 *          MIBSPI peripheral specified by the MIBSPI handle.
 *
 *
 *  In #MIBSPI_MODE_BLOCKING, MIBSPI_transferCancel has no effect.
 *
 *  In #MIBSPI_MODE_CALLBACK, MIBSPI_transferCancel() will stop an SPI transfer if
 *  if one is in progress.
 *  If a transaction was in progress, its callback function will be called
 *  in context from which this API is called from. The #MIBSPI_CallbackFxn
 *  function can determine if the transaction was successful or not by reading
 *  the #MIBSPI_Status status value in the #MIBSPI_Transaction structure.
 *
 *  @param  handle      #MIBSPI_Handle
 *
 *  @sa     MIBSPI_open
 *  @sa     MIBSPI_transfer
 */
void MIBSPI_transferCancel(MIBSPI_Handle handle);

/**
 *  \brief  This function enables the Loopback mode for self test. Loopback is SPI master only feature.
 *
 *   @param[in] handle          #MIBSPI_Handle
 *   @param[in] loopbacktype     Digital or Analog
 *
 *  @return On success returns SystemP_SUCCESS. Negative values indicate
 *          unsuccessful operations.
 */
 int32_t MIBSPI_enableLoopback(MIBSPI_Handle handle,
                               MIBSPI_LoopBackType loopbacktype);

 /**
 *  \brief  This function disable the Loopback mode.
 *
 *   @param[in] handle          #MIBSPI_Handle
 *
 *  @return On success returns SystemP_SUCCESS. Negative values indicate
 *          unsuccessful operations.
 */
 int32_t MIBSPI_disableLoopback(MIBSPI_Handle handle);

 /**
 *  \brief This function sets the Polarity and phase for MibSpi as requested.
 *
 *   @param[in] handle           #MIBSPI_Handle MibSpiSPI
 *   @param[in] clockFmt          MIBSPI functional mode (clock/polarity)
 *
 *
 *  @return On success returns SystemP_SUCCESS. Negative values indicate
 *          unsuccessful operations.
 */
 int32_t MIBSPI_setClockPhasePolarity(MIBSPI_Handle handle,
                                      uint8_t clockFmt);

 /**
 *  \brief  Get SPI driver internal stats
 *
 *   @param[in] handle           SPI device handle
 *   @param[in] ptrStats         pointer to SPI driver stats structure
 *
 *  @return On success returns SystemP_SUCCESS. Negative values indicate
 *          unsuccessful operations.
 *
 *  @sa     MIBSPI_open
 *  @sa     MIBSPI_transfer
 */
 int32_t MIBSPI_getDrvStats(MIBSPI_Handle handle,
                            MIBSPI_Stats *ptrStats);

/**
 *  \brief  Function invoked by the SoC DMA implementation to the driver on
 *          DMA transfer completion
 *
 *  @param  mibspiHandle  A MIBSPI_Handle
 *
 */
void MIBSPI_dmaDoneCb(MIBSPI_Handle mibspiHandle);

/**
 *  \brief  Function to initialize the #MIBSPI_OpenParams struct to its defaults
 *
 *  \param  openPrms    Pointer to #MIBSPI_OpenParams structure for
 *                      initialization
 */
static inline void MIBSPI_Params_init(MIBSPI_OpenParams *openPrms);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */
static inline void MIBSPI_Params_init(MIBSPI_OpenParams *openPrms)
{
    if (openPrms != NULL)
    {
        openPrms->mode                  = MIBSPI_MASTER;
        openPrms->dataSize              = 16U;
        openPrms->frameFormat           = MIBSPI_POL0_PHA0;
        openPrms->transferMode          = MIBSPI_MODE_BLOCKING;
        openPrms->iCountSupport         = FALSE;
        openPrms->transferCallbackFxn   = NULL;
        openPrms->pinMode               = MIBSPI_PINMODE_4PIN_CS;
        openPrms->shiftFormat           = MIBSPI_MSB_FIRST;
        openPrms->dmaEnable             = (uint8_t)1U;
        openPrms->eccEnable             = (uint8_t)0U;
        openPrms->csHold                = (uint8_t)0U;
        openPrms->txDummyValue          = (uint16_t)0xFFFFU;
        openPrms->transferTimeout       = 0xFFFFFFFFU;
        openPrms->compatibilityMode     = FALSE;

        /* Initlaize master mode params */
        openPrms->u.masterParams.bitRate   = 5000000U;
        openPrms->u.masterParams.numSlaves = 1;
        openPrms->u.masterParams.t2cDelay  = 0;
        openPrms->u.masterParams.c2tDelay  = 0;
        openPrms->u.masterParams.wDelay    = 0;
        openPrms->u.masterParams.slaveProf[0].chipSelect = (uint8_t)0U;
        openPrms->u.masterParams.slaveProf[0].ramBufLen = (uint8_t)MIBSPI_RAM_MAX_ELEM;
        openPrms->u.masterParams.slaveProf[0].dmaReqLine = 0U;
    }
}

#ifdef __cplusplus
}
#endif

#endif /* #ifndef MIBSPI_H_ */

/** @} */
