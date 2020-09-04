/*
 *  Copyright (C) 2021-22 Texas Instruments Incorporated
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
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/hw_include/cslr_mcspi.h>
#include <drivers/hw_include/cslr.h>

#if defined (DMA_VERSION_MCSPI_UDMA)
#include <drivers/mcspi/v0/dma/udma/mcspi_dma_udma.h>
#endif

#if defined (DMA_VERSION_MCSPI_EDMA)
#include <drivers/mcspi/v0/dma/edma/mcspi_dma_edma.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief A handle that is returned from a #MCSPI_open() call */
typedef void *MCSPI_Handle;

/**
 *  \anchor MCSPI_ChannelId
 *  \name Channel Id
 *
 *  Values used to determine the channel number used for McSPI
 *  communication. This determines which Chip Select (CS) line to use
 *
 *  @{
 */
#define MCSPI_CHANNEL_0                 (0U)
#define MCSPI_CHANNEL_1                 (1U)
#define MCSPI_CHANNEL_2                 (2U)
#define MCSPI_CHANNEL_3                 (3U)
/** @} */

/**
 *  \anchor MCSPI_OperatingMode
 *  \name Operating Mode
 *
 *  Values used to determine the McSPI driver operation.
 *
 *  @{
 */
#define MCSPI_OPER_MODE_POLLED                 (0U)
#define MCSPI_OPER_MODE_INTERRUPT              (1U)
#define MCSPI_OPER_MODE_DMA                    (2U)
/** @} */

/** \brief Max number of channels/Chip Select (CS) supported */
#define MCSPI_MAX_NUM_CHANNELS          (4U)

/**
 *  \anchor MCSPI_TransferStatus
 *  \name Transfer Status Code
 *
 *  Status codes that are set by the MCSPI driver
 *
 *  @{
 */
#define MCSPI_TRANSFER_COMPLETED        (0U)
#define MCSPI_TRANSFER_STARTED          (1U)
#define MCSPI_TRANSFER_CANCELLED        (2U)
#define MCSPI_TRANSFER_FAILED           (3U)
#define MCSPI_TRANSFER_CSN_DEASSERT     (4U)
#define MCSPI_TRANSFER_TIMEOUT          (5U)
/** @} */

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
/** @} */

/**
 *  \anchor MCSPI_MsMode
 *  \name Modes of Operation
 *
 *  Definitions for various MCSPI modes of operation
 *
 *  The MCSPI driver operates in both master and SPI slave modes.
 *  Logically, the implementation is identical, however the difference
 *  between these two modes is driven by hardware. The default mode is
 *  #MCSPI_MS_MODE_MASTER, but can be set to slave mode by setting
 *  #MCSPI_OpenParams.msMode to #MCSPI_MS_MODE_SLAVE in the parameters passed to
 *  #MCSPI_open().
 *
 *  @{
 */
/** \brief The module generates the clock and CS */
#define MCSPI_MS_MODE_MASTER            (CSL_MCSPI_MODULCTRL_MS_MASTER)
/** \brief The module receives the clock and CS */
#define MCSPI_MS_MODE_SLAVE             (CSL_MCSPI_MODULCTRL_MS_SLAVE)
/** @} */

/**
 *  \anchor MCSPI_FrameFormat
 *  \name Frame Format
 *
 *  Definitions for various SPI data frame formats
 *
 *  POL0 = SPICLK is held low during the INACTIVE state
 *  POL1 = SPICLK is held high during the INACTIVE state
 *
 *  PHA0 = Data are latched on odd-numbered edges of SPICLK
 *  PHA1 = Data are latched on even-numbered edges of SPICLK
 *
 *  @{
 */
#define MCSPI_FF_POL0_PHA0              (0U)
#define MCSPI_FF_POL0_PHA1              (1U)
#define MCSPI_FF_POL1_PHA0              (2U)
#define MCSPI_FF_POL1_PHA1              (3U)
/** @} */

/**
 *  \anchor MCSPI_CsPolarity
 *  \name Chip-select Polarity
 *
 *  Type for SPI Chip Select Polarity and Clock Idle Level
 *
 *  @{
 */
/** \brief SPIEN (CS) is held high during the ACTIVE state */
#define MCSPI_CS_POL_HIGH               (CSL_MCSPI_CH0CONF_EPOL_ACTIVEHIGH)
/** \brief SPIEN (CS) is held low during the ACTIVE state */
#define MCSPI_CS_POL_LOW                (CSL_MCSPI_CH0CONF_EPOL_ACTIVELOW)
/** @} */

/**
 *  \anchor MCSPI_TrMode
 *  \name Transmit-Receive Modes
 *
 *  @{
 */
#define MCSPI_TR_MODE_TX_RX             (CSL_MCSPI_CH0CONF_TRM_TRANSRECEI)
#define MCSPI_TR_MODE_RX_ONLY           (CSL_MCSPI_CH0CONF_TRM_RECEIVONLY)
#define MCSPI_TR_MODE_TX_ONLY           (CSL_MCSPI_CH0CONF_TRM_TRANSONLY)
/** @} */

/**
 *  \anchor MCSPI_InputSelect
 *  \name Input Select
 *
 *  @{
 */
/** \brief Data line 0 (SPIDAT[0]) selected for reception */
#define MCSPI_IS_D0                     (CSL_MCSPI_CH0CONF_IS_LINE0)
/** \brief Data line 1 (SPIDAT[1]) selected for reception */
#define MCSPI_IS_D1                     (CSL_MCSPI_CH0CONF_IS_LINE1)
/** @} */

/**
 *  \anchor MCSPI_TxEnable
 *  \name Transmission Enable for Data Line
 *
 *  @{
 */
/** \brief Data line selected for transmission */
#define MCSPI_DPE_ENABLE                (CSL_MCSPI_CH0CONF_DPE0_ENABLED)
/** \brief No transmission on Data Line */
#define MCSPI_DPE_DISABLE               (CSL_MCSPI_CH0CONF_DPE0_DISABLED)
/** @} */

/**
 *  \anchor MCSPI_SlvCsSelect
 *  \name Slave Chip-select Signal Select
 *
 *  @{
 */
#define MCSPI_SLV_CS_SELECT_0           (CSL_MCSPI_CH0CONF_SPIENSLV_SPIEN0)
#define MCSPI_SLV_CS_SELECT_1           (CSL_MCSPI_CH0CONF_SPIENSLV_SPIEN1)
#define MCSPI_SLV_CS_SELECT_2           (CSL_MCSPI_CH0CONF_SPIENSLV_SPIEN2)
#define MCSPI_SLV_CS_SELECT_3           (CSL_MCSPI_CH0CONF_SPIENSLV_SPIEN3)
/** @} */

/**
 *  \anchor MCSPI_SbPolarity
 *  \name Start-bit Polarity
 *
 *  @{
 */
/** \brief SStart-bit polarity is held to 1 during MCSPI transfer */
#define MCSPI_SB_POL_HIGH               (CSL_MCSPI_CH0CONF_SBPOL_HIGHLEVEL)
/** \brief Start-bit polarity is held to 0 during MCSPI transfer */
#define MCSPI_SB_POL_LOW                (CSL_MCSPI_CH0CONF_SBPOL_LOWLEVEL)
/** @} */

/**
 *  \anchor MCSPI_CsIdleTime
 *  \name Chip-select Idle Time
 *
 *  Values used to configure the chip select time control (TCS)
 *
 *  @{
 */
/** \brief 0.5 clock cycles delay */
#define MCSPI_TCS0_0_CLK                (CSL_MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY)
/** \brief 1.5 clock cycles delay */
#define MCSPI_TCS0_1_CLK                (CSL_MCSPI_CH0CONF_TCS0_ONECYCLEDLY)
/** \brief 2.5 clock cycles delay */
#define MCSPI_TCS0_2_CLK                (CSL_MCSPI_CH0CONF_TCS0_TWOCYCLEDLY)
/** \brief 3.5 clock cycles delay */
#define MCSPI_TCS0_3_CLK                (CSL_MCSPI_CH0CONF_TCS0_THREECYCLEDLY)
/** @} */

/**
 *  \anchor MCSPI_ChMode
 *  \name Channel Mode
 *
 *  @{
 */
/**
 *  \brief Only one channel will be used in master mode. This should be used
 *  when CS is used in forced enable mode.
 */
#define MCSPI_CH_MODE_SINGLE            (CSL_MCSPI_MODULCTRL_SINGLE_SINGLE)
/** \brief More than one channel will be used in master mode */
#define MCSPI_CH_MODE_MULTI             (CSL_MCSPI_MODULCTRL_SINGLE_MULTI)
/** @} */

/**
 *  \anchor MCSPI_PinMode
 *  \name Pin Mode
 *
 *  @{
 */
/**
 *  \brief SPIEN (CS) is not used. In this mode all related options to
 *  chip-select have no meaning.
 */
#define MCSPI_PINMODE_3PIN              (CSL_MCSPI_MODULCTRL_PIN34_3PINMODE)
#define MCSPI_PINMODE_4PIN              (CSL_MCSPI_MODULCTRL_PIN34_4PINMODE)
/** @} */

/**
 *  \anchor MCSPI_InitDelay
 *  \name Init Delay
 *
 *  Values used to enable initial delay for first transfer
 *
 *  @{
 */
/** \brief No delay */
#define MCSPI_INITDLY_0                 (CSL_MCSPI_MODULCTRL_INITDLY_NODELAY)
/** \brief 4 SPI bus clock delays */
#define MCSPI_INITDLY_4                 (CSL_MCSPI_MODULCTRL_INITDLY_4CLKDLY)
/** \brief 8 SPI bus clock delays */
#define MCSPI_INITDLY_8                 (CSL_MCSPI_MODULCTRL_INITDLY_8CLKDLY)
/** \brief 16 SPI bus clock delays */
#define MCSPI_INITDLY_16                (CSL_MCSPI_MODULCTRL_INITDLY_16CLKDLY)
/** \brief 32 SPI bus clock delays */
#define MCSPI_INITDLY_32                (CSL_MCSPI_MODULCTRL_INITDLY_32CLKDLY)
/** @} */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief Data structure used with #MCSPI_transfer()
 *
 *  It indicates how many \ref MCSPI_FrameFormat frames are sent and received
 *  from the buffers pointed to txBuf and rxBuf.
 *  The args variable is an user-definable argument which gets passed to the
 *  #MCSPI_CallbackFxn when the SPI driver is in #MCSPI_TRANSFER_MODE_CALLBACK.
 */
typedef struct
{
    uint32_t                channel;
    /**< [IN] Channel number (chip select) to use.
      *  Valid value from 0 to (#MCSPI_MAX_NUM_CHANNELS - 1) */
    uint32_t                csDisable;
    /**< [IN] TRUE/FALSE to disable CS(chip select)
      * If it is set to TRUE, CS is de-asseted automatically at the end of the
      * transfer. If user wants to chain more transfers under one CS pulse,
      * user needs to set it to FALSE for each transfer and for the last
      * transfer, user needs to set to TRUE to de-assert CS */
    uint32_t                dataSize;
    /**< [IN] MCSPI data frame size in bits - valid values: 4 bits to 32 bits
     *
     *   The dataSize value determines the element types of txBuf and rxBuf.
     *   If the dataSize is from 4 to 8 bits, the driver assumes the
     *   data buffers are of type uint8_t (unsigned char).
     *   If the dataSize is from 8 to 16 bits, the driver assumes the
     *   data buffers are of type uint16_t (unsigned short).
     *   If the dataSize is greater than 16 bits, the driver assumes the
     *   data buffers are uint32_t (unsigned long).
     */
    uint32_t                count;
    /**< [IN] Number of frames for this transaction. This should in word size
     *   length and not in bytes */
    void                   *txBuf;
    /**< [IN] void * to a buffer with data to be transmitted.
     *
     *   If txBuf is NULL, the driver sends MCSPI frames with all data set to
     *   the default value specified by #MCSPI_ChConfig.defaultTxData.
     *
     *   The size of the buffer should be count * #MCSPI_Transaction.dataSize
     *   in bytes rounded to nearest byte boundary.
     *   For example if #MCSPI_Transaction.dataSize is 12 bits, then size of
     *   buffers should be count * 2 bytes.
     */
    void                   *rxBuf;
    /**< [IN] void * to a buffer to receive data.
     *
     *   If rxBuf is NULL, the driver discards all MCSPI frames received.
     *
     *   The size of the buffer is similar to txBuf as explained above.
     */
    void                   *args;
    /**< [IN] Argument to be passed to the callback function */
    uint32_t                status;
    /**< [OUT] \ref MCSPI_TransferStatus code set by #MCSPI_transfer() */
} MCSPI_Transaction;

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
    /**< Master or Slave mode. Refer \ref MCSPI_MsMode */
    int32_t                 mcspiDmaIndex;
    /**< Index of DMA instance used by MCSPI Driver. This index will be set by SysCfg according to the DMA driver chosen.
     * The MCSPI driver uses this index to do an \ref MCSPI_dmaOpen inside the \ref MCSPI_open if the DMA mode is enabled
     */
} MCSPI_OpenParams;

/**
 *  \brief MCSPI configuration parameters for the channel
 *
 *  MCSPI channel parameters used with the #MCSPI_chConfig() call.
 *  Default values for these parameters are set using MCSPI_ChConfig_init().
 *
 *  If NULL is passed for the parameters, #MCSPI_chConfig() uses default
 *  parameters.
 *
 *  \sa #MCSPI_ChConfig_init()
 */
typedef struct
{
    uint32_t                chNum;
    /**< Channel number. Refer \ref MCSPI_ChannelId */
    uint32_t                frameFormat;
    /**< MCSPI frame format. Refer \ref MCSPI_FrameFormat */
    uint32_t                bitRate;
    /**< MCSPI bit rate in Hz */
    uint32_t                csPolarity;
    /**< Polarity of the chip select signal. Refer \ref MCSPI_CsPolarity */
    uint32_t                trMode;
    /**< Channel transmit/receive mode. Refer \ref MCSPI_TrMode */
    uint32_t                inputSelect;
    /**< Input Select - D0 or D1. Refer \ref MCSPI_InputSelect */
    uint32_t                dpe0;
    /**< Transmission enable/disable for D0. Refer \ref MCSPI_TxEnable */
    uint32_t                dpe1;
    /**< Transmission enable/disable for D1. Refer \ref MCSPI_TxEnable */
    uint32_t                slvCsSelect;
    /**< MCSPI slave select signal detection. Applicable for Channel 0 and
     *   in slave mode only. Refer \ref MCSPI_SlvCsSelect */
    uint32_t                startBitEnable;
    /**< Start bit D/CX added before SPI transfer. Polarity is defined by
     *   start bit level (below) */
    uint32_t                startBitPolarity;
    /**< Start-bit polarity used when startBitEnable is TRUE
     *   Refer \ref MCSPI_SbPolarity */
    uint32_t                csIdleTime;
    /**< Chip select time control. Refer \ref MCSPI_CsIdleTime.
     *   This is applicable only in master mode */
    uint32_t                defaultTxData;
    /**< Default TX data to use when NULL pointer is provided for TX buffer.
     *   The actual data that is transmitted depends on the dataSize field */
    uint32_t                txFifoTrigLvl;
    /**< TX FIFO trigger level in bytes */
    uint32_t                rxFifoTrigLvl;
    /**< RX FIFO trigger level in bytes */
} MCSPI_ChConfig;

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
} MCSPI_Attrs;

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/**
 *  \brief MCSPI channel object
 */
typedef struct
{
    /*
     * User parameters
     */
    MCSPI_ChConfig          chCfg;
    /**< Channel configuration as provided by user */

    /*
     * State variables
     */
    uint32_t                isOpen;
    /**< Flag to indicate whether the instance is opened already */
    uint32_t                csDisable;
    /**< Flag to indicate disable chip select */
    uint32_t                csEnable;
    /**< Flag to indicate enable chip select */
    const uint8_t          *curTxBufPtr;
    /**< Current TX buffer pointer */
    uint8_t                *curRxBufPtr;
    /**< Current RX buffer pointer */
    uint32_t                curTxWords;
    /**< Number of words transmitted. We need seperate counters for TX/RX
     *   because when FIFO in enabled, TX writes happen in advance where as
     *   RX will happen on actual received data. */
    uint32_t                curRxWords;
    /**< Number of words received */

    /*
     * MCSPI derived variables
     */
    uint8_t                 bufWidthShift;
    /**< Width of buffer in bytes - used for accessing the TX/RX buffer.
     *   When dataWidth <= 8,           bufWidth = uint8_t  (1 byte - 0 shift)
     *   When dataWidth > 8  && <= 16,  bufWidth = uint16_t (2 bytes - 1 shift)
     *   When dataWidth > 16 && <= 32,  bufWidth = uint32_t (4 bytes - 2 shift)
     */
    uint32_t                dataWidthBitMask;
    /**< Data width mask depending on SPI word size */
    uint32_t                effTxFifoDepth;
    /**< Effective TX FIFO depth in words - depends on dataWidth */
    uint32_t                effRxFifoDepth;
    /**< Effective RX FIFO depth in words - depends on dataWidth */
    uint32_t                intrMask;
    /**< Interrupt mask to be used for enabling / checking interrupts. */
    MCSPI_DmaChConfig       dmaChCfg;
    /**< Channel Config Register Value. */
    uint32_t                chConfRegVal;
    /**< Channel Control Register Value. */
    uint32_t                chCtrlRegVal;
    /**< SYST Register Value. */
    uint32_t                systRegVal;
} MCSPI_ChObject;

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
    /**< Interrupt handle for master ISR */
    HwiP_Object             hwiObj;
    /**< Interrupt object */

    MCSPI_Transaction      *currTransaction;
    /**< Pointer to current transaction */
    void                   *mcspiDmaHandle;
    /**< DMA Handle */
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
 *  If the MCSPI is in #MCSPI_MS_MODE_MASTER mode, it will immediately start the
 *  transaction. If the MCSPI is in #MCSPI_MS_MODE_SLAVE mode, it prepares the
 *  driver for a transaction with a MCSPI master device. The device will then
 *  wait until the master begins the transfer.
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
 *  important for slave operations where #MCSPI_transfer() might be called a
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
 *  MCSPI transfer if if one is in progress.
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
/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

static inline void MCSPI_OpenParams_init(MCSPI_OpenParams *openPrms)
{
    if(openPrms != NULL)
    {
        openPrms->transferMode        = MCSPI_TRANSFER_MODE_BLOCKING;
        openPrms->transferTimeout     = SystemP_WAIT_FOREVER;
        openPrms->transferCallbackFxn = NULL;
        openPrms->msMode              = MCSPI_MS_MODE_MASTER;
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
/** \brief Total length of FIFO for both TX/RX */
#define MCSPI_FIFO_LENGTH           (64U)
/**
 * \brief McSPI peripheral Rx FIFO is enabled
 */
#define MCSPI_RX_FIFO_ENABLE         ((uint32_t) CSL_MCSPI_CH0CONF_FFER_FFENABLED \
                                      <<                                      \
                                      CSL_MCSPI_CH0CONF_FFER_SHIFT)

/**
 * \brief McSPI peripheral Rx FIFO is disabled
 */
#define MCSPI_RX_FIFO_DISABLE        ((uint32_t) CSL_MCSPI_CH0CONF_FFER_FFDISABLED \
                                      << CSL_MCSPI_CH0CONF_FFER_SHIFT)

/**
 * \brief McSPI peripheral Tx FIFO is enabled
 */
#define MCSPI_TX_FIFO_ENABLE         ((uint32_t) CSL_MCSPI_CH0CONF_FFEW_FFENABLED \
                                      << CSL_MCSPI_CH0CONF_FFEW_SHIFT)

/**
 * \brief McSPI peripheral Tx FIFO is disabled
 */
#define MCSPI_TX_FIFO_DISABLE        ((uint32_t) CSL_MCSPI_CH0CONF_FFEW_FFDISABLED \
                                      << CSL_MCSPI_CH0CONF_FFEW_SHIFT)
/**
 *  \brief McSPI Register Offset for MCSPI_CHxCONF, MCSPI_CHxSTAT,
 *  MCSPI_CHxCTRL, MCSPI_TXx and MCSPI_RXx register set.
 */
#define MCSPI_REG_OFFSET            (0x14U)
/** \brief Base address of McSPI_CHCONF(x) */
#define MCSPI_CHCONF(x)             ((uint32_t) CSL_MCSPI_CH0CONF +            \
                                     (uint32_t) ((uint32_t) MCSPI_REG_OFFSET * \
                                                 (uint32_t) (x)))
/** \brief Base address of McSPI_CHSTAT(x) */
#define MCSPI_CHSTAT(x)             ((uint32_t) CSL_MCSPI_CH0STAT +            \
                                     (uint32_t) ((uint32_t) MCSPI_REG_OFFSET * \
                                                 (uint32_t) (x)))
/** \brief Base address of McSPI_CHCTRL(x) */
#define MCSPI_CHCTRL(x)             ((uint32_t) CSL_MCSPI_CH0CTRL +            \
                                     (uint32_t) ((uint32_t) MCSPI_REG_OFFSET * \
                                                 (uint32_t) (x)))
/** \brief Base address of McSPI_CHTX(x) */
#define MCSPI_CHTX(x)               ((uint32_t) CSL_MCSPI_TX0 +                \
                                     (uint32_t) ((uint32_t) MCSPI_REG_OFFSET * \
                                                 (uint32_t) (x)))
/** \brief Base address of McSPI_CHRX(x) */
#define MCSPI_CHRX(x)               ((uint32_t) CSL_MCSPI_RX0 +                \
                                     (uint32_t) ((uint32_t) MCSPI_REG_OFFSET * \
                                                 (uint32_t) (x)))

#define MCSPI_CLKD_MASK             (0x0FU)

/** \brief Bit mask to clear all status bits */
#define MCSPI_IRQSTATUS_CLEAR_ALL   (CSL_MCSPI_IRQSTATUS_EOW_MASK |            \
                                     CSL_MCSPI_IRQSTATUS_WKS_MASK |            \
                                     CSL_MCSPI_IRQSTATUS_RX3_FULL_MASK |       \
                                     CSL_MCSPI_IRQSTATUS_TX3_UNDERFLOW_MASK |  \
                                     CSL_MCSPI_IRQSTATUS_TX3_EMPTY_MASK |      \
                                     CSL_MCSPI_IRQSTATUS_RX2_FULL_MASK |       \
                                     CSL_MCSPI_IRQSTATUS_TX2_UNDERFLOW_MASK |  \
                                     CSL_MCSPI_IRQSTATUS_TX2_EMPTY_MASK |      \
                                     CSL_MCSPI_IRQSTATUS_RX1_FULL_MASK |       \
                                     CSL_MCSPI_IRQSTATUS_TX1_UNDERFLOW_MASK |  \
                                     CSL_MCSPI_IRQSTATUS_TX1_EMPTY_MASK |      \
                                     CSL_MCSPI_IRQSTATUS_RX0_OVERFLOW_MASK |   \
                                     CSL_MCSPI_IRQSTATUS_RX0_FULL_MASK |       \
                                     CSL_MCSPI_IRQSTATUS_TX0_UNDERFLOW_MASK |  \
                                     CSL_MCSPI_IRQSTATUS_TX0_EMPTY_MASK)

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
int32_t MCSPI_reConfigFifo(MCSPI_Handle handle,
                           uint32_t chNum,
                           uint32_t numWordsRxTx);

/**
 * \brief  This API will return the buffer width in bytes based on dataSize.
 *
 *
 * \param  dataSize         MCSPI data frame size in bits - valid values: 4 bits to 32 bits
 *
 * \return bufWidthShift    Width of buffer in bytes - used for accessing the TX/RX buffer.
 *                          When dataWidth <= 8,          (1 byte - 0 shift)
 *                          When dataWidth > 8  && <= 16, (2 bytes - 1 shift)
 *                          When dataWidth > 16 && <= 32, (4 bytes - 2 shift)
 *
 *
 *  \sa    #MCSPI_open
 **/
static inline uint32_t MCSPI_getBufWidthShift(uint32_t dataSize);

/**
 * \brief  This API will return the status of the McSPI channel currently in
 *         use.
 *
 * \param  baseAddr        Memory Address of the McSPI instance used.
 * \param  chNum           Channel used for communication.\n
 *
 *         'chNum' can take the following values.\n
 *         MCSPI_CHANNEL_n - Channel n is used for communication.\n
 *
 * \return This API will return the status of the McSPI channel status
 *         register.
 *         User can use the following macros to check the status \n
 *         MCSPI_CH_STAT_RXS_FULL - Receiver register is full \n
 *         MCSPI_CH_STAT_TXS_EMPTY - Transmitter register is full \n
 *         MCSPI_CH_STAT_EOT - End of transfer status \n
 *         MCSPI_CH_TXFFE - FIFO transmit buffer empty status \n
 *         MCSPI_CH_TXFFF - FIFO transmit buffer full status \n
 *         MCSPI_CH_RXFFE - FIFO receive buffer empty status \n
 *         MCSPI_CH_RXFFF - FIFO receive buffer full status \n
 *
 *  \sa    #MCSPI_open
 **/
static inline uint32_t MCSPI_readChStatusReg(uint32_t baseAddr, uint32_t chNum);

/**
 * \brief This API returns Channel control register value.
 *
 * \param  baseAddr       Memory Address of the McSPI instance used.
 * \param  chNum          Channel number of the McSPI instance used.
 *
 * \return Channel control register value.
 *
 *  \sa    #MCSPI_open
 **/
static inline uint32_t MCSPI_readChCtrlReg(uint32_t baseAddr, uint32_t chNum);

/**
 * \brief This API sets Channel control register value.
 *
 * \param  baseAddr       Memory Address of the McSPI instance used.
 * \param  chNum          Channel number of the McSPI instance used.
 * \param  regVal         register value to set in channel control register.
 *
 *  \sa    #MCSPI_open
 **/
static inline void MCSPI_writeChCtrlReg(uint32_t baseAddr, uint32_t chNum,
                       uint32_t regVal);

/**
 * \brief This API returns Channel Config register value.
 *
 * \param  baseAddr       Memory Address of the McSPI instance used.
 * \param  chNum          Channel number of the McSPI instance used.
 *
 * \return Channel Config register value.
 *
 *  \sa    #MCSPI_open
 **/
static inline uint32_t MCSPI_readChConf(uint32_t baseAddr, uint32_t chNum);

/**
 * \brief This API sets Channel Config register value.
 *
 * \param  baseAddr       Memory Address of the McSPI instance used.
 * \param  chNum          Channel number of the McSPI instance used.
 * \param  regVal         register value to set in channel Config register.
 *
 *  \sa    #MCSPI_open
 **/
static inline void MCSPI_writeChConfReg(uint32_t baseAddr, uint32_t chNum,
                       uint32_t regVal);

/**
 * \brief  This API will put the data on to the McSPI Channel
 *         transmit register.
 *
 * \param  baseAddr        Memory Address of the McSPI instance used.
 * \param  txData          32 bit data sent by the user which is put on
 *                         to the MCSPI_TX register.
 * \param  chNum           Channel number of the McSPI instance used.\n
 *
 *         'chNum' can take the following values.\n
 *         MCSPI_CHANNEL_n - Channel n is used for communication.\n
 *
 *         For chNum n can range from 0-3.\n
 *
 *  \sa    #MCSPI_open
 **/
static inline void MCSPI_writeTxDataReg(uint32_t baseAddr,
                       uint32_t txData,
                       uint32_t chNum);

/**
 * \brief  This API will enable/disable the Tx FIFOs of McSPI peripheral.
 *
 * \param  baseAddr        Memory Address of the McSPI instance used.
 * \param  chNum           Channel number of the McSPI instance used.\n
 * \param  enableFlag      Flag to enable/diable FIFO transmit mode.
 *
 *         'enableFlag' can take the following values.\n
 *         MCSPI_TX_FIFO_ENABLE - Enables the receiver FIFO of McSPI.\n
 *         MCSPI_TX_FIFO_DISABLE - Disables the receiver FIFO of McSPI.\n
 *
 *         'chNum' can take the following values.\n
 *         MCSPI_CHANNEL_n - Channel n is used for communication.\n
 *
 *         For chNum n can range from 0-3.\n
 *
 * \note:  Enabling FIFO is restricted to only 1 channel.
 *  \sa    #MCSPI_open
 **/
static inline void MCSPI_enableTxFIFO(uint32_t baseAddr, uint32_t chNum,
                               uint32_t enableFlag);

/**
 * \brief  This API will enable/disable the Rx FIFOs of McSPI peripheral.
 *
 * \param  baseAddr        Memory Address of the McSPI instance used.
 * \param  chNum           Channel number of the McSPI instance used.\n
 * \param  enableFlag      Flag to enable/diable FIFO receive mode.
 *
 *         'enableFlag' can take the following values.\n
 *         MCSPI_RX_FIFO_ENABLE - Enables the receiver FIFO of McSPI.\n
 *         MCSPI_RX_FIFO_DISABLE - Disables the receiver FIFO of McSPI.\n
 *
 *         'chNum' can take the following values.\n
 *         MCSPI_CHANNEL_n - Channel n is used for communication.\n
 *
 *         For chNum n can range from 0-3.\n
 *
 * \note:  Enabling FIFO is restricted to only 1 channel.
 *  \sa    #MCSPI_open
 **/
static inline void MCSPI_enableRxFIFO(uint32_t baseAddr, uint32_t chNum,
                               uint32_t enableFlag);

/**
 * \brief  This API will return the data present in the MCSPI_RX register.
 *
 * \param  baseAddr        Memory Address of the McSPI instance used.
 * \param  chNum           Channel number of the McSPI instance used.
 *
 *         'chNum' can take the following values.\n
 *         MCSPI_CHANNEL_n - Channel n is used for communication.\n
 *
 *         For chNum n can range from 0-3.\n
 *
 *  \sa    #MCSPI_open
 *
 * \return This API will return the data received in the MCSPI_RX register.
 **/
static inline uint32_t MCSPI_readRxDataReg(uint32_t baseAddr,
                                           uint32_t chNum);

/**
 * \brief  This API will set the data width in the channel config register.
 *
 * \param  baseAddr        Memory Address of the McSPI instance used.
 * \param  chNum           Channel number of the McSPI instance used.
 * \param  dataWidth        MCSPI data frame width in bits.
 *
 *         'chNum' can take the following values.\n
 *         MCSPI_CHANNEL_n - Channel n is used for communication.\n
 *
 *         For chNum n can range from 0-3.\n
 *         For dataWidth valid values: 4 bits to 32 bits
 *
 *  \sa    #MCSPI_open
 *
 **/
static inline void MCSPI_setDataWidth(uint32_t baseAddr, uint32_t chNum,
                                         uint32_t dataWidth);

/* ========================================================================== */
/*                       Advanced Function Definitions                        */
/* ========================================================================== */
static inline uint32_t MCSPI_getBufWidthShift(uint32_t dataSize)
{
    uint32_t bufWidthShift = 0U;

    if(dataSize <= 8U)
    {
        bufWidthShift = 0U;
    }
    else if(dataSize <= 16U)
    {
        bufWidthShift = 1U;
    }
    else
    {
        bufWidthShift = 2U;
    }

    return bufWidthShift;
}

static inline uint32_t MCSPI_readChStatusReg(uint32_t baseAddr, uint32_t chNum)
{
    /* Return the status from MCSPI_CHSTAT register. */
    return (CSL_REG32_RD(baseAddr + MCSPI_CHSTAT(chNum)));
}

static inline uint32_t MCSPI_readChCtrlReg(uint32_t baseAddr, uint32_t chNum)
{
    return CSL_REG32_RD(baseAddr + MCSPI_CHCTRL(chNum));
}

static inline void MCSPI_writeChCtrlReg(uint32_t baseAddr, uint32_t chNum,
                        uint32_t regVal)
{
    CSL_REG32_WR(baseAddr + MCSPI_CHCTRL(chNum), regVal);
}

static inline uint32_t MCSPI_readChConf(uint32_t baseAddr, uint32_t chNum)
{
    return CSL_REG32_RD(baseAddr + MCSPI_CHCONF(chNum));
}

static inline void MCSPI_writeChConfReg(uint32_t baseAddr, uint32_t chNum,
                        uint32_t regVal)
{
    CSL_REG32_WR(baseAddr + MCSPI_CHCONF(chNum), regVal);
}

static inline void MCSPI_writeTxDataReg(uint32_t baseAddr,
                       uint32_t txData,
                       uint32_t chNum)
{
    /* Load the MCSPI_TX register with the data to be transmitted */
    CSL_REG32_WR(baseAddr + MCSPI_CHTX(chNum), txData);
}

static inline void MCSPI_enableTxFIFO(uint32_t baseAddr,
                               uint32_t chNum,
                               uint32_t enableFlag)
{
    /* Set the FFEW field with user sent value. */
    CSL_REG32_FINS(
        baseAddr + MCSPI_CHCONF(chNum),
        MCSPI_CH0CONF_FFEW,
        enableFlag >> CSL_MCSPI_CH0CONF_FFEW_SHIFT);
}

static inline void MCSPI_enableRxFIFO(uint32_t baseAddr,
                               uint32_t chNum,
                               uint32_t enableFlag)
{
    /* Set the FFER field with the user sent value. */
    CSL_REG32_FINS(
        baseAddr + MCSPI_CHCONF(chNum),
        MCSPI_CH0CONF_FFER,
        enableFlag >> CSL_MCSPI_CH0CONF_FFER_SHIFT);
}

static inline uint32_t MCSPI_readRxDataReg(uint32_t baseAddr, uint32_t chNum)
{
    /* Return the data present in the MCSPI_RX register. */
    return (CSL_REG32_RD(baseAddr + MCSPI_CHRX(chNum)));
}

static inline void MCSPI_setDataWidth(uint32_t baseAddr, uint32_t chNum,
                                         uint32_t dataWidth)
{
    uint32_t regVal;

    regVal = CSL_REG32_RD(baseAddr + MCSPI_CHCONF(chNum));
    CSL_FINS(regVal, MCSPI_CH0CONF_WL, (dataWidth - 1U));
    CSL_REG32_WR(baseAddr + MCSPI_CHCONF(chNum), regVal);
}

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef MCSPI_H_ */

/** @} */
