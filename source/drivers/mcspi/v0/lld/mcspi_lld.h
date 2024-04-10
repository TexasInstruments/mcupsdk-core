/*
 *  Copyright (C) 2023-24 Texas Instruments Incorporated
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
 *  \defgroup DRV_MCSPI_LLD_MODULE APIs for MCSPI LLD
 *  \ingroup  DRV_MODULE
 *
 *  This module contains APIs to program and use the MCSPI LLD module. The APIs
 *  can be used by other drivers to get access to MCSPI and also by
 *  application to initiate data transfer operation.
 *
 *  @{
 */

/**
 *  \file v0/lld/mcspi_lld.h
 *
 *  \brief MCSPI LLD Driver API/interface file.
 */

#ifndef MCSPI_LLD_H_
#define MCSPI_LLD_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdbool.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/hw_include/cslr_mcspi.h>
#include <drivers/hw_include/cslr.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*  pointer for DMA Handle  */
typedef void *MCSPI_DmaHandle;
/*  pointer for DMA Channel Config */
typedef void *MCSPI_DmaChConfig;

/* function pointer to get clock ticks */
typedef uint32_t (*MCSPI_clockGet) (void);


/** @name Return status
 */
/**@{*/
/**
 * \brief Return status when the API execution was successful
 */
#define MCSPI_STATUS_SUCCESS         ((int32_t)0)

/**
 * \brief Return status when the API execution was not successful due to a failure
 */
#define MCSPI_STATUS_FAILURE         ((int32_t)-1)

/**
 * \brief Return status when the API execution was not successful due to a time out
 */
#define MCSPI_TIMEOUT                ((int32_t)-2)

/**
 * \brief Return status when the API execution failed due invalid parameters
 */
#define MCSPI_INVALID_PARAM          ((int32_t)-3)

/**
 * \brief Return status when the API execution failed due to driver busy
 */
#define MCSPI_STATUS_BUSY            ((int32_t)-4)

/**
 * \brief Return status when the API execution failed due to invalid state
 */
#define MCSPI_INVALID_STATE          ((int32_t)-5)

/**@}*/

/** @name Timeout values
 */
/**@{*/
/**
 * \brief Value to use when needing a timeout of zero or NO timeout, return immediately on resource not available.
 */
#define MCSPI_NO_WAIT                ((uint32_t)0)

/**
 * \brief Value to use when needing a timeout of infinity or wait forver until resource is available
 */
#define MCSPI_WAIT_FOREVER           ((uint32_t)-1)
/**@}*/

/**@{*/
/** @name MCSPI Driver states
 */
/**
 * \brief MCSPI driver is in Reset State prior to driver init and post driver deinit
 */
#define MCSPI_STATE_RESET            ((uint32_t)0U)

/**
 * \brief MCSPI driver accepts runtime APIs only Ready State, otherwise return error
 */
#define MCSPI_STATE_READY            ((uint32_t)1U)

/**
 * \brief MCSPI driver is busy performing operation with peripherals, return error when APIs are invoked
 */
#define MCSPI_STATE_BUSY             ((uint32_t)2U)

/**
 * \brief MCSPI driver ran into error, returns error for all APIs other than deinit in this state
 */
#define MCSPI_STATE_ERROR            ((uint32_t)3U)
/**@}*/

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
#define MCSPI_TRANSFER_COMPLETED        ((int32_t)0U)
#define MCSPI_TRANSFER_STARTED          ((int32_t)1U)
#define MCSPI_TRANSFER_CANCELLED        ((int32_t)2U)
#define MCSPI_TRANSFER_FAILED           ((int32_t)3U)
#define MCSPI_TRANSFER_CSN_DEASSERT     ((int32_t)4U)
#define MCSPI_TRANSFER_TIMEOUT          ((int32_t)5U)
/** @} */

/**
 *  \anchor MCSPI_MsMode
 *  \name Modes of Operation
 *
 *  Definitions for various MCSPI modes of operation
 *
 *  The MCSPI driver operates in both controller and SPI peripheral modes.
 *  Logically, the implementation is identical, however the difference
 *  between these two modes is driven by hardware. The default mode is
 *  #MCSPI_MS_MODE_CONTROLLER, but can be set to peripheral mode by setting
 *  #MCSPI_OpenParams.msMode to #MCSPI_MS_MODE_PERIPHERAL in the parameters passed to
 *  #MCSPI_open().
 *
 *  @{
 */
/** \brief The module generates the clock and CS */
#define MCSPI_MS_MODE_CONTROLLER        (CSL_MCSPI_MODULCTRL_MS_MASTER)
/** \brief The module receives the clock and CS */
#define MCSPI_MS_MODE_PERIPHERAL        (CSL_MCSPI_MODULCTRL_MS_SLAVE)
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
 *  \name Peripheral Chip-select Signal Select
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
 *  \brief Only one channel will be used in controller mode. This should be used
 *  when CS is used in forced enable mode.
 */
#define MCSPI_CH_MODE_SINGLE            (CSL_MCSPI_MODULCTRL_SINGLE_SINGLE)
/** \brief More than one channel will be used in controller mode */
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

/** \brief McSPI error macro's*/
#define MCSPI_ERROR_TX_UNDERFLOW    (0x00000001U)
#define MCSPI_ERROR_RX_OVERFLOW     (0x00000002U)

/** Maximuum Clock Divider supported */
#define MCSPI_MAX_CLK_DIVIDER_SUPPORTED   (4096U)

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
    uint32_t                timeout;
    /**< Timeout for this transaction in units of system ticks */
    uint32_t                status;
    /**< [OUT] \ref MCSPI_TransferStatus code set by #MCSPI_transfer() */
} MCSPI_Transaction;

/**
 *  \brief Data structure used with #MCSPI_lld_read(), #MCSPI_lld_readIntr(),
 *         #MCSPI_lld_readDma(), #MCSPI_lld_write(), #MCSPI_lld_writeIntr,
 *         #MCSPI_lld_writeDma, #MCSPI_lld_readWrite(), #MCSPI_lld_readWriteIntr(),
 *         #MCSPI_lld_readWriteDma().
 *         MCSPI ExtendedParams to be used in case, one's don't want to use
 *         default parameters, else pass NULL.
 *
 */
typedef struct MCSPI_ExtendedParams_s
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
    uint32_t               dataSize;
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
    void                   *args;
    /**< [IN] Argument to be passed to the callback function */
} MCSPI_ExtendedParams;

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
    /**< MCSPI peripheral select signal detection. Applicable for Channel 0 and
     *   in peripheral mode only. Refer \ref MCSPI_SlvCsSelect */
    uint32_t                startBitEnable;
    /**< Start bit D/CX added before SPI transfer. Polarity is defined by
     *   start bit level (below) */
    uint32_t                startBitPolarity;
    /**< Start-bit polarity used when startBitEnable is TRUE
     *   Refer \ref MCSPI_SbPolarity */
    uint32_t                turboEnable;
    /**< Enable Turbo Mode */
    uint32_t                csIdleTime;
    /**< Chip select time control. Refer \ref MCSPI_CsIdleTime.
     *   This is applicable only in controller mode */
    uint32_t                defaultTxData;
    /**< Default TX data to use when NULL pointer is provided for TX buffer.
     *   The actual data that is transmitted depends on the dataSize field */
    uint32_t                txFifoTrigLvl;
    /**< TX FIFO trigger level in bytes */
    uint32_t                rxFifoTrigLvl;
    /**< RX FIFO trigger level in bytes */
} MCSPI_ChConfig;

/* ========================================================================== */
/*                      Function pointers Declarations                        */
/* ========================================================================== */

/**
 *  \brief  The definition of a transfer completion callback function used by
 *  the SPI driver when used in #MCSPI_TRANSFER_MODE_CALLBACK
 *
 *  \param void*          void pointer
 *  \param transferStatus transfer Status
 */
typedef void (*MCSPI_transferCallbackFxn) (void *args, uint32_t transferStatus);

/**
 *  \brief  The definition of a error callback function used by the SPI driver
 *  when used in #MCSPI_TRANSFER_MODE_CALLBACK
 *
 *  \param void*          void pointer
 *  \param transferStatus transfer Status
 */
typedef void (*MCSPI_errorCallbackFxn) (void *args, uint32_t transferStatus);

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
    MCSPI_ChConfig  *chCfg;
    /**< Channel configuration as provided by user */
    uint32_t        dmaChConfigNum;
    /**< Index of dmaChConfig in DMA Config Array */

    /*
     * State variables
     */
    uint32_t                isOpen;
    /**< Flag to indicate whether the instance is opened already */
    uint32_t                csDisable;
    /**< Flag to indicate disable chip select */
    uint32_t                csEnable;
    /**< Flag to indicate enable chip select */
    uint8_t                *curTxBufPtr;
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
    /**< DMA Configuration for each channel. */
    uint32_t                chConfRegVal;
    /**< Channel Control Register Value. */
    uint32_t                chCtrlRegVal;
    /**< SYST Register Value. */
    uint32_t                systRegVal;
} MCSPI_ChObject;

/**
 *  \brief MCSPI driver initialization object
 */
typedef struct
{
    uint32_t                            inputClkFreq;
    /**< Module input clock frequency */
    uint32_t                            intrNum;
    /**< Peripheral interrupt number */
    uint32_t                            operMode;
    /**< Driver operating mode */
    uint8_t                             intrPriority;
    /**< Interrupt priority */
    uint32_t                            chMode;
    /**< Channel mode: Single or multi channel. Refer \ref MCSPI_ChMode  */
    uint32_t                            pinMode;
    /**< Pin mode. Refer \ref MCSPI_PinMode */
    uint32_t                            initDelay;
    /**< Initial SPI delay for first transfer. Refer \ref MCSPI_InitDelay */
    uint32_t                            multiWordAccess;
    /**< Flag to enable/disable multi word access */
    uint32_t                            msMode;
    /**< Controller or Peripheral mode. Refer \ref MCSPI_MsMode */
    uint32_t                            chEnabled[MCSPI_MAX_NUM_CHANNELS];
    /**< Enable/Disable Flag for all McSPI channels */
    MCSPI_ChObject                      chObj[MCSPI_MAX_NUM_CHANNELS];
    /**< Channel object */
    MCSPI_DmaHandle                     mcspiDmaHandle;
    /**< DMA Handle */
    MCSPI_clockGet                      clockP_get;
    /* clock usec to tick */
    MCSPI_transferCallbackFxn           transferCallbackFxn;
    /**< Callback function pointer */
    MCSPI_errorCallbackFxn              errorCallbackFxn;
    /**< Callback function pointer */
} MCSPILLD_InitObject, *MCSPILLD_InitHandle;

/**
 *  \brief MCSPI driver object
 */
typedef struct
{
    uint32_t                baseAddr;
    /**< Peripheral base address */

    /*
     * User parameters
     */
    uint32_t                 state;
    /**< Driver state variable */
    void                    *transferMutex;
    /**< Transfer Sync Sempahore - to signal transfer completion */
    MCSPILLD_InitHandle      hMcspiInit;
    /**< [IN] Initialization parameters of McSPI instance */
    uint32_t                 errorFlag;
    /**< Variable to store different McSPI errors */

    /*
     * Transfer parameters
     */
    uint32_t                transferChannel;
    /**< [IN] Channel number (chip select) to use for transfers */
    uint32_t                transferCsDisable;
    /**< [IN] TRUE/FALSE to disable CS(chip select) for transfers */
    uint32_t                transferDataSize;
    /**< [IN] MCSPI data frame size in bits - valid values: 4 bits to 32 bits */
    MCSPI_Transaction       transaction;
    /**< Pointer to current transaction */
    void*                   args;
    /**< Pointer to be used by application to store miscellaneous data.*/
} MCSPILLD_Object, *MCSPILLD_Handle;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Low level HW functions */
void MCSPI_reset(uint32_t baseAddr);
void MCSPI_clearAllIrqStatus(uint32_t baseAddr);
void MCSPI_stop(MCSPILLD_Handle hMcspi, MCSPI_ChObject *chObj, uint32_t chNum);
void MCSPI_setChDataSize(uint32_t baseAddr, MCSPI_ChObject *chObj,
                         uint32_t dataSize, uint32_t csDisable);

static inline void MCSPI_intrStatusClear(const MCSPI_ChObject *chObj,
                                         uint32_t baseAddr, uint32_t intFlags)
{
    /* Clear the SSB bit in the MCSPI_SYST register. */
    CSL_REG32_WR(baseAddr + CSL_MCSPI_SYST, chObj->systRegVal);
    /* Clear the interrupt status. */
    CSL_REG32_WR(baseAddr + CSL_MCSPI_IRQSTATUS, intFlags);
}

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief  This API Initializes the McSPI instance
 *
 *  \param  hMcspi      Handle to the McSPI instance used
 *
 *  \return #MCSPI_STATUS_SUCCESS if successful; else error on failure
 *
 */
int32_t MCSPI_lld_init(MCSPILLD_Handle hMcspi);

/**
 *  \brief  This API Initializes the McSPI instance in DMA Mode
 *
 *  \param  hMcspi      Handle to the McSPI instance used
 *
 *  \return #MCSPI_STATUS_SUCCESS if successful; else error on failure
 *
 */
int32_t MCSPI_lld_initDma(MCSPILLD_Handle hMcspi);
/**
 *  \brief  This API De-Initializes the McSPI instance
 *
 *  \param  hMcspi      Handle to the McSPI instance used
 *
 *  \return #MCSPI_STATUS_SUCCESS if successful; else error on failure
 *
 */
int32_t MCSPI_lld_deInit(MCSPILLD_Handle hMcspi);

/**
 *  \brief  This API De-Initializes the McSPI instance in DMA mode
 *
 *  \param  hMcspi      Handle to the McSPI instance used
 *
 *  \return #MCSPI_STATUS_SUCCESS if successful; else error on failure
 *
 */
int32_t MCSPI_lld_deInitDma(MCSPILLD_Handle hMcspi);

/**
 *  \brief  This API writes data to the McSPI instance in Polling mode.
 *
 *  \param  hMcspi           Handle to the McSPI instance used
 *  \param  txBuf            Pointer to write data buffer
 *  \param  count            Number of frames for this transaction
 *  \param  timeout          Write timeout value
 *  \param  extendedParams   Pointer to structure that contains the transfer data.
 *
 *  \return #MCSPI_STATUS_SUCCESS if successful; else error on failure
 *
 */
int32_t MCSPI_lld_write(MCSPILLD_Handle hMcspi, void *txBuf, uint32_t count, uint32_t timeout,
                        const MCSPI_ExtendedParams *extendedParams);

/**
 *  \brief  This API writes data to the McSPI instance in Interrupt mode.
 *
 *  \param  hMcspi           Handle to the McSPI instance used
 *  \param  txBuf            Pointer to write data buffer
 *  \param  count            Number of frames for this transaction
 *  \param  timeout          write timeout value
 *  \param  extendedParams   Pointer to structure that contains the transfer data.
 *
 *  \return #MCSPI_STATUS_SUCCESS if successful; else error on failure
 *
 */
int32_t MCSPI_lld_writeIntr(MCSPILLD_Handle hMcspi, void *txBuf, uint32_t count, uint32_t timeout,
                            const MCSPI_ExtendedParams *extendedParams);

/**
 *  \brief  This API writes data to the McSPI instance in DMA mode.
 *
 *  \param  hMcspi           Handle to the McSPI instance used
 *  \param  txBuf            Pointer to write data buffer
 *  \param  count            Number of frames for this transaction
 *  \param  timeout          write timeout value
 *  \param  extendedParams   Pointer to structure that contains the transfer data.
 *
 *  \return #MCSPI_STATUS_SUCCESS if successful; else error on failure
 *
 */
int32_t MCSPI_lld_writeDma(MCSPILLD_Handle hMcspi, void * txBuf, uint32_t count,
                           uint32_t timeout, const MCSPI_ExtendedParams *extendedParams);

/**
 *  \brief  This API reads data from the McSPI instance in Polling mode.
 *
 *  \param  hMcspi           Handle to the McSPI instance used
 *  \param  rxBuf            Pointer to Read data buffer
 *  \param  count            Number of frames for this transaction
 *  \param  timeout          Read timeout value
 *  \param  extendedParams   Pointer to structure that contains the transfer data.
 *
 *  \return #MCSPI_STATUS_SUCCESS if successful; else error on failure
 *
 */
int32_t MCSPI_lld_read(MCSPILLD_Handle hMcspi, void * rxBuf, uint32_t count, uint32_t timeout,
                       const MCSPI_ExtendedParams *extendedParams);

/**
 *  \brief  This API reads data from the McSPI instance in Interrupt mode.
 *
 *  \param  hMcspi           Handle to the McSPI instance used
 *  \param  rxBuf            Pointer to Read data buffer
 *  \param  count            Number of frames for this transaction
 *  \param  timeout          Read timeout value
 *  \param  extendedParams   Pointer to structure that contains the transfer data.
 *
 *  \return #MCSPI_STATUS_SUCCESS if successful; else error on failure
 *
 */
int32_t MCSPI_lld_readIntr(MCSPILLD_Handle hMcspi, void * rxBuf, uint32_t count, uint32_t timeout,
                           const MCSPI_ExtendedParams *extendedParams);

/**
 *  \brief  This API reads data from the McSPI instance in DMA mode.
 *
 *  \param  hMcspi           Handle to the McSPI instance used
 *  \param  rxBuf            Pointer to Read data buffer
 *  \param  count            Number of frames for this transaction
 *  \param  timeout          Read timeout value
 *  \param  extendedParams   Pointer to structure that contains the transfer data.
 * extendedParams
 *
 *  \return #MCSPI_STATUS_SUCCESS if successful; else error on failure
 *
 */
int32_t MCSPI_lld_readDma(MCSPILLD_Handle hMcspi, void * rxBuf, uint32_t count,
                          uint32_t timeout, const MCSPI_ExtendedParams *extendedParams);
/**
 *  \brief  This API reads writes data from the McSPI instance in polling mode.
 *
 *  \param  hMcspi           Handle to the McSPI instance used
 *  \param  txBuf            Pointer to write data buffer
 *  \param  rxBuf            Pointer to Read data buffer
 *  \param  count            Number of frames for this transaction
 *  \param  timeout          Read write timeout value
 *  \param  extendedParams   Pointer to structure that contains the transfer data.
 *
 *  \return #MCSPI_STATUS_SUCCESS if successful; else error on failure
 */
int32_t MCSPI_lld_readWrite(MCSPILLD_Handle hMcspi, void *txBuf, void *rxBuf, uint32_t count,
                            uint32_t timeout, const MCSPI_ExtendedParams *extendedParams);
/**
 *  \brief  This API reads writes data from the McSPI instance in Interrupt mode.
 *
 *  \param  hMcspi           Handle to the McSPI instance used
 *  \param  txBuf            Pointer to write data buffer
 *  \param  rxBuf            Pointer to Read data buffer
 *  \param  count            Number of frames for this transaction
 *  \param  timeout          Read write timeout value
 *  \param  extendedParams   Pointer to structure that contains the transfer data.
 *
 *  \return #MCSPI_STATUS_SUCCESS if successful; else error on failure
 */
int32_t MCSPI_lld_readWriteIntr(MCSPILLD_Handle hMcspi, void *txBuf, void *rxBuf, uint32_t count,
                                uint32_t timeout, const MCSPI_ExtendedParams *extendedParams);
/**
 *  \brief  This API reads writes data from the McSPI instance in DMA mode.
 *
 *  \param  hMcspi           Handle to the McSPI instance used
 *  \param  txBuf            Pointer to write data buffer
 *  \param  rxBuf            Pointer to Read data buffer
 *  \param  count            Number of frames for this transaction
 *  \param  timeout          Read write timeout value
 *  \param  extendedParams   Pointer to structure that contains the transfer data.
 *
 *  \return #MCSPI_STATUS_SUCCESS if successful; else error on failure
 */
int32_t MCSPI_lld_readWriteDma(MCSPILLD_Handle hMcspi, void *txBuf, void *rxBuf, uint32_t count,
                               uint32_t timeout, const MCSPI_ExtendedParams *extendedParams);

/**
 *  \brief  This API cancels current McSPI transfer
 *
 *  \param  hMcspi          Handle to the McSPI instance used
 *
 *  \return Mcspi Status    Status of current mcspi transaction
 *
 */
int32_t MCSPI_lld_readWriteCancel(MCSPILLD_Handle hMcspi);

/**
 *  \brief  This API cancels current McSPI transfer in DMA mode
 *
 *  \param  hMcspi          Handle to the McSPI instance used
 *
 *  \return Mcspi Status    Status of current mcspi transaction
 *
 */
int32_t MCSPI_lld_readWriteDmaCancel(MCSPILLD_Handle hMcspi);

/**
 *  \brief  This API transfers data from the McSPI instance in Polling mode.
 *
 *  \param  hMcspi          Handle to the McSPI instance used
 *  \param  transaction     Pointer to structure that contains the transfer data.
 *
 *  \return #MCSPI_STATUS_SUCCESS if successful; else error on failure
 *
 */
int32_t MCSPI_lld_transfer(MCSPILLD_Handle hMcspi, MCSPI_Transaction *transaction);

/**
 *  \brief  This API reads data from the McSPI instance in Interrupt mode.
 *
 *  \param  hMcspi          Handle to the McSPI instance used
 *  \param  transaction     Pointer to structure that contains the transfer data.
 *
 *  \return #MCSPI_STATUS_SUCCESS if successful; else error on failure
 *
 */
int32_t MCSPI_lld_transferIntr(MCSPILLD_Handle hMcspi, MCSPI_Transaction *transaction);

/**
 *  \brief  This API reads data from the McSPI instance in DMA mode.
 *
 *  \param  hMcspi          Handle to the McSPI instance used
 *  \param  transaction     Pointer to structure that contains the transfer data.
 *
 *  \return #MCSPI_STATUS_SUCCESS if successful; else error on failure
 *
 */
int32_t MCSPI_lld_transferDma(MCSPILLD_Handle hMcspi, MCSPI_Transaction *transaction);

/**
 *  \brief  This is the McSPI Controller ISR and can be used as IRQ handler in Controller mode.
 *
 *  \param  args      Argument to the ISR.
 *
 */
void MCSPI_lld_controllerIsr(void* args);

/**
 *  \brief  This is the McSPI Peripheral ISR and can be used as IRQ handler in Peripheral mode.
 *
 *  \param  args      Argument to the ISR.
 *
 */
void MCSPI_lld_peripheralIsr(void* args);

/**
 *  \brief  This API returns the driver state.
 *
 *  \param  hMcspi           Handle to the McSPI instance used
 *
 */
int32_t MCSPI_lld_getState(MCSPILLD_Handle hMcspi);

/**
 *  \brief  This API cancels current McSPI transfer
 *
 *  \param  hMcspi          Handle to the McSPI instance used
 *
 *  \return #MCSPI_STATUS_SUCCESS if successful; else error on failure
 *
 */
int32_t MCSPI_lld_transferCancel(MCSPILLD_Handle hMcspi);

/**
 *  \brief  This API cancels current McSPI transfer in DMA mode
 *
 *  \param  hMcspi          Handle to the McSPI instance used
 *
 *  \return #MCSPI_STATUS_SUCCESS if successful; else error on failure
 *
 */
int32_t MCSPI_lld_transferDmaCancel(MCSPILLD_Handle hMcspi);

/**
 *  \brief  Function to initialize the #MCSPI_ChConfig struct to its defaults
 *
 *  \param  chConfig    Pointer to #MCSPI_ChConfig structure for
 *                      initialization
 */
static inline void MCSPI_lld_ChConfig_init(MCSPI_ChConfig *chConfig);

/**
 *  \brief  Function to initialize the #MCSPI_Transaction struct to its defaults
 *
 *  \param  trans       Pointer to #MCSPI_Transaction structure for
 *                      initialization
 */
static inline void MCSPI_lld_Transaction_init(MCSPI_Transaction *trans);

/**
 *  \brief  Function to get base address of MCSPI instance of a particular
 *          handle.
 *
 *  \param  handle      #MCSPILLD_Handle returned from #MCSPI_open()
 *
 *  \sa     #MCSPI_open
 */
uint32_t MCSPI_lld_getBaseAddr(MCSPILLD_Handle handle);

/**
 *  \brief  Function to re-configure Effective FIFO Words.
 *
 *  \param  handle          #MCSPILLD_Handle returned from #MCSPI_open()
 *  \param  chNum           Channel used for communication.
 *  \param  numWordsRxTx    Number of words to transfer
 *
 *  \return #MCSPI_STATUS_SUCCESS if successful; else error on failure
 *
 *  \sa     #MCSPI_open
 */
int32_t MCSPI_lld_reConfigFifo(MCSPILLD_Handle handle,
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
static inline uint8_t MCSPI_getBufWidthShift(uint32_t dataSize);

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
/*                       Static Function Definitions                          */
/* ========================================================================== */

static inline void MCSPI_lld_ChConfig_init(MCSPI_ChConfig *chConfig)
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
    }
}

static inline void MCSPI_lld_Transaction_init(MCSPI_Transaction *trans)
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
        trans->timeout   = MCSPI_WAIT_FOREVER;
    }
}

static inline uint8_t MCSPI_getBufWidthShift(uint32_t dataSize)
{
    uint8_t bufWidthShift = 0U;

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

#ifdef __cplusplus
}
#endif

#endif /* #ifndef MCSPI_LLD_H_ */

/** @} */
