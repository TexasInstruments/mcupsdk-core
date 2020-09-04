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
 *  \defgroup DRV_FSI_TX_MODULE APIs for FSI TX
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the FSI TX module.
 *
 *  @{
 */

/**
 *  \file   v0/fsi_tx.h
 *
 *  \brief  Header file containing various enumerations, structure definitions and function
 *  declarations for the FSI TX IP.
 */

#ifndef FSI_TX_H
#define FSI_TX_H


#if defined (SOC_AM64X) || defined (SOC_AM243X)
#define FSI_TX_BUFFER_ACCESS_64BIT
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/hw_include/cslr_fsi_tx.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 *  \anchor FSI_TxEnum
 *  \name FSI TX Enum type
 *  @{
 */
/**
 * \brief This enumerator defines the types of possible FSI TX events
 *
 *  Values that can be passed to APIs to enable/disable interrupts and
 *  also to set/get/clear event status on FSI TX operation.
 *
 *  There are 4 supported interrupts related to TX events, all are
 *  available as event status as well excecpt 4th one.
 *    1) frame transmission done
 *    2) transmit buffer is underrun
 *    3) transmit buffer is overrun
 *    4) ping counter timeout
 *
 *  Ping frame transmission upon hardware trigger (ping watchdog or
 *  external trigger) is shown as event status.
 *
 *
 */
typedef uint32_t FSI_TxEnumType;

#define FSI_TX_EVT_FRAME_DONE       ((uint16_t)0x1U)
    /** TX frame done event */
#define FSI_TX_EVT_BUF_UNDERRUN     ((uint16_t)0x2U)
    /** TX buffer underrun event */
#define FSI_TX_EVT_BUF_OVERRUN      ((uint16_t)0x4U)
    /** TX buffer overrun event */
#define FSI_TX_EVT_PING_TIMEOUT     ((uint16_t)0x8U)
    /** TX ping timeout event */
#define FSI_TX_EVT_PING_HW_TRIG     ((uint16_t)0x8U)
    /** TX ping hardware trigger event */

/**
 * \brief Mask of all TX Event types
 */
#define FSI_TX_EVTMASK              ((uint16_t)0xFU)

/**
 * \brief maximum number of external input for triggering frame-transmission
 */
#define FSI_TX_MAX_NUM_EXT_TRIGGERS ((uint16_t)0x40U)

/**
 * \brief Shifts needed to control FSI TX interrupt generation on INT2
 */
#define FSI_TX_INT2_CTRL_S          ((uint16_t)0x8U)
/** @} */

/**
 *  \anchor FSI_TxSubmoduleInReset
 *  \name FSI TX submodules can be in reset
 *  @{
 */
/**
 * \brief  TX submodules that can be reset with reset APIs
 */
typedef uint32_t FSI_TxSubmoduleInReset;

#define FSI_TX_MASTER_CORE_RESET             ((uint32_t)0x0U)
/**< Reset entire TX Module */
#define FSI_TX_CLOCK_RESET                   ((uint32_t)0x1U)
/**< Reset only TX clock */
#define FSI_TX_PING_TIMEOUT_CNT_RESET        ((uint32_t)0x2U)
/**< Reset ping timeout counter */
/** @} */

/**
 *  \anchor FSI_TxStartMode
 *  \name FSI TX start mode
 *  @{
 */
/**
 * \brief  Start Mode for TX frame transmission (i.e. how transmission will start)
 */
typedef uint32_t FSI_TxStartMode;

#define FSI_TX_START_FRAME_CTRL              ((uint32_t)0x0U)
/**< SW write of START bit in TX_PKT_CTRL register */
#define FSI_TX_START_EXT_TRIG                ((uint32_t)0x1U)
/**< Rising edge on external trigger */
#define FSI_TX_START_FRAME_CTRL_OR_UDATA_TAG ((uint32_t)0x2U)
/**< Either SW write of START bit or Frame completion */
/** @} */

/**
 *  \anchor FSI_TxClkSel
 *  \name FSI TX input clock select
 *  @{
 */
/**
 * \brief  FSI TX  input clock select
 */
typedef uint32_t FSI_TxClkSel;

#define FSI_TX_CLK_SEL0                   ((uint32_t)0x0U)
/**< Input clock select 0 */
#define FSI_TX_CLK_SEL1                   ((uint32_t)0x1U)
/**< Input clock select 1 */
/** @} */


/**
 *  \anchor FSI_FrameTag
 *  \name FSI frame tag
 *  @{
 */
/**
 * \brief  FSI frame tag values
 *
 * 4 bit field inside FSI frame is available to set tag value (0-15)
 */
typedef uint32_t FSI_FrameTag;

#define FSI_FRAME_TAG0              ((uint32_t)0x0U)
/**< Frame tag value 0 */
#define FSI_FRAME_TAG1              ((uint32_t)0x1U)
/**< Frame tag value 1 */
#define FSI_FRAME_TAG2              ((uint32_t)0x2U)
/**< Frame tag value 2 */
#define FSI_FRAME_TAG3              ((uint32_t)0x3U)
/**< Frame tag value 3 */
#define FSI_FRAME_TAG4              ((uint32_t)0x4U)
/**< Frame tag value 4 */
#define FSI_FRAME_TAG5              ((uint32_t)0x5U)
/**< Frame tag value 5 */
#define FSI_FRAME_TAG6              ((uint32_t)0x6U)
/**< Frame tag value 6 */
#define FSI_FRAME_TAG7              ((uint32_t)0x7U)
/**< Frame tag value 7 */
#define FSI_FRAME_TAG8              ((uint32_t)0x8U)
/**< Frame tag value 8 */
#define FSI_FRAME_TAG9              ((uint32_t)0x9U)
/**< Frame tag value 9 */
#define FSI_FRAME_TAG10             ((uint32_t)0xAU)
/**< Frame tag value 10 */
#define FSI_FRAME_TAG11             ((uint32_t)0xBU)
/**< Frame tag value 11 */
#define FSI_FRAME_TAG12             ((uint32_t)0xCU)
/**< Frame tag value 12 */
#define FSI_FRAME_TAG13             ((uint32_t)0xDU)
/**< Frame tag value 13 */
#define FSI_FRAME_TAG14             ((uint32_t)0xEU)
/**< Frame tag value 14 */
#define FSI_FRAME_TAG15             ((uint32_t)0xFU)
/**< Frame tag value 15 */
/** @} */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * \brief   This API sends FLUSH pattern
 *
 * FLUSH pattern (toggle data lines followed by toggle on clocks)
 * should be sent only when FSI TX is not under SOFT_RESET and the
 * clock to the transmit core has been turned ON.
 *
 * \param   base       [IN]  Base address of the FSI TX module
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base addr parameter
 */
int32_t FSI_sendTxFlush(uint32_t base);

/**
 * \brief   This API stops FLUSH pattern transmission
 *
 * Transmission of FLUSH pattern should be stopped before starting
 * sending frames. Generally during initilization a pair of send/stop
 * APIs for FLUSH pattern is called to clear data/clock lines.
 *
 * \param   base       [IN]  Base address of the FSI TX module.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base addr parameter
 */
int32_t FSI_stopTxFlush(uint32_t base);

/**
 * \brief   This API selects PLL clock as source for clock dividers
 *
 * \param   base       [IN]  Base address of the FSI TX module.
 * \param   clkSel     [IN]  Input PLL clock select to the FSI TX module
 *                           refer #FSI_TxClkSel.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base addr parameter
 */
int32_t FSI_selectTxPLLClock(uint32_t base, FSI_TxClkSel clkSel);

/**
 * \brief   This API sets clock division prescalar and enables the transmit clock
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 * \param   preScaleValue [IN]  prescale value used to generate transmit clock, it
 *                              defines the division value of /2,/3,/4, etc. of PLL CLK
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_enableTxClock(uint32_t base, uint16_t preScaleValue);

/**
 * \brief   This API disables transmit clock
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_disableTxClock(uint32_t base);

/**
 * \brief   This API sets Data width for transmission
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 * \param   dataWidth     [IN]  Data lines used for TX operation
 *                              refer #FSI_DataWidth.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_setTxDataWidth(uint32_t base, FSI_DataWidth dataWidth);

/**
 * \brief   This API enables SPI compatible mode
 *
 * @n This API is only applicable when communicating with a SPI interface
 *
 * FSI supports a compatibility mode in order to communicate with
 * legacy peripherals like SPI. Only the 16-bit mode of SPI will
 * be supported. All the frame structures, CRC checks and will be
 * identical to the normal FSI frames.
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_enableTxSPIMode(uint32_t base);

/**
 * \brief   This API disables SPI compatible mode
 *
 * @n This API is only applicable when communicating with a SPI interface
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_disableTxSPIMode(uint32_t base);

/**
 * \brief   This API sets start mode for any frame transmission
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 * \param   txStartMode   [IN]  Refer #FSI_TxStartMode
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_setTxStartMode(uint32_t base, FSI_TxStartMode txStartMode);

/**
 * \brief   This API sets HW/SW initiated TX ping timeout mode
 *
 * \param   base            [IN]  Base address of the FSI TX module.
 * \param   pingTimeoutMode [IN]  Refer #FSI_PingTimeoutMode
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_setTxPingTimeoutMode(uint32_t base, FSI_PingTimeoutMode pingTimeoutMode);

/**
 * \brief   This API sets a particular external input to trigger transmission
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 * \param   extInputNum   [IN]  external input number, from 0 to 31
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_setTxExtFrameTrigger(uint32_t base, uint16_t extInputNum);

/**
 * \brief   This API enables CRC value of a data frame to be forced to zero
 *
 * CRC value of the data frame will be forced to 0 whenever there is
 * a transmission and buffer over-run or under-run condition happens.
 * The idea is to force a corruption of the CRC since the data is not
 * guaranteed to be reliable
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_enableTxCRCForceError(uint32_t base);

/**
 * \brief   This API disables forcing of CRC value of a data frame to zero
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_disableTxCRCForceError(uint32_t base);

/**
 * \brief   This API select between 16-bit and 32-bit ECC computation for FSI TX
 *
 * \param   base            [IN]  Base address of the FSI TX module.
 * \param   eccComputeWidth [IN]  Refer #FSI_ECCComputeWidth
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_setTxECCComputeWidth(uint32_t base, FSI_ECCComputeWidth eccComputeWidth);

/**
 * \brief   This API sets frame type for transmission
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 * \param   frameType     [IN]  Refer #FSI_FrameType
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_setTxFrameType(uint32_t base, FSI_FrameType frameType);

/**
 * \brief   This API sets the frame size if frame type is user/software defined frame
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 * \param   nWords        [IN]  number of data words in a software defined frame
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_setTxSoftwareFrameSize(uint32_t base, uint16_t nWords);

/**
 * \brief   This API starts transmitting frames
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_startTxTransmit(uint32_t base);

/**
 * \brief   This API sets frame tag for transmission
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 * \param   frameTag      [IN]  refer #FSI_FrameTag
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_setTxFrameTag(uint32_t base, FSI_FrameTag frameTag);

/**
 * \brief   This API sets user defined data for transmission
 *          It is an extra data field (8 bit) apart from regular data
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 * \param   userDefData   [IN]  8 bit user defined data value
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_setTxUserDefinedData(uint32_t base, uint16_t userDefData);

/**
 * \brief   This API sets the value for transmit buffer pointer at desired location
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 * \param   bufPtrOff     [IN]  4 bit offset pointer in TX buffer where
 *                              transmitter will pick the data
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_setTxBufferPtr(uint32_t base, uint16_t bufPtrOff);

/**
 * \brief   This API gets current buffer pointer locationn
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 * \param   pBufPtrLoc    [OUT] Pointer to current buffer pointer location
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_getTxBufferPtr(uint32_t base, uint16_t *pBufPtrLoc);

/**
 * \brief   This API gets valid number of data words present
 *          in buffer which have not been transmitted yet
 *
 * @n there could be lag due to synchronization hence value is
 *    accurate only when no current transmission is happening
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 * \param   pWordCnt      [OUT] Pointer to number of data words
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_getTxWordCount(uint32_t base, uint16_t *pWordCnt);

/**
 * \brief   This API enables ping timer logic and once set time elapses
 *          it sends signal to transmitter to send ping frame
 *
 * @n there could be lag due to synchronization hence value is
 *    accurate only when no current transmission is happening
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 * \param   refValue      [IN]  32 bit reference value for ping time-out counter
 * \param   pingFrameTag  [IN]  4 bit tag value for ping time-out counter
 *                              refer #FSI_FrameTag
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_enableTxPingTimer(uint32_t     base,
                                     uint32_t     refValue,
                                     FSI_FrameTag pingFrameTag);

/**
 * \brief   This API sets the ping tag value, used by either timeout counter
 *          initiated PING frame transfer or by external ping trigger input
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 * \param   frameTag      [IN]  4 bit tag value for ping time-out counter
 *                              refer #FSI_FrameTag
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_setTxPingTag(uint32_t base, FSI_FrameTag frameTag);

/**
 * \brief   This API disables ping timer logic
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_disableTxPingTimer(uint32_t base);

/**
 * \brief   This API enables external trigger to transmit a ping frame
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 * \param   extTrigSel    [IN]  5 bit value which selects among 32 external inputs
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_enableTxExtPingTrigger(uint32_t base, uint16_t extTrigSel);

/**
 * \brief   This API disables external trigger logic
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_disableTxExtPingTrigger(uint32_t base);

/**
 * \brief   This API gets current value of ping timeout logic counter
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 * \param   pPingToCnt    [OUT] Pointer to current ping timeout counter
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_getTxCurrentPingTimeoutCounter(uint32_t  base,
                                                  uint32_t *pPingToCnt);

/**
 * \brief   This API locks the control of all transmit control registers, once
 *          locked further writes will not take effect until system reset occurs
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_lockTxCtrl(uint32_t base);

/**
 * \brief   This API gets current status of all the error flags
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 * \param   pEvtFlags     [OUT] Pointer to the status of error event flags, each
 *                              bit of integer is associated with one error flag.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_getTxEventStatus(uint32_t base, uint16_t *pEvtFlags);

/**
 * \brief   This API enables user to set TX error flags
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 * \param   evtFlags      [IN]  event and error flags to be set
 *
 * @n Writing a 1 to this bit position will cause the corresponding bit
 *    in TX_EVT_ERR_STATUS register to get set. The purpose of this
 *    register is to allow software to simulate the effect of the event
 *    and test the associated software/ISR.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_forceTxEvents(uint32_t base, uint16_t evtFlags);

/**
 * \brief   This API enables user to clear TX error flags
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 * \param   evtFlags      [IN]  event and error flags to be cleared
 *
 * @n Writing a 1 to this bit position will cause the corresponding bit
 *    in TX_EVT_ERR_STATUS register to get cleared to 0.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_clearTxEvents(uint32_t base, uint16_t evtFlags);

/**
 * \brief   This API sets the CRC value to be picked transmission if
 *          transmission is configured to use user defined SW CRC
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 * \param   userCRCValue  [IN]  user defined CRC value
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_enableTxUserCRC(uint32_t base, uint16_t userCRCValue);

/**
 * \brief   This API disables user defined CRC value, the
 *          transmitted CRC value is computed by hardware
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_disableTxUserCRC(uint32_t base);

/**
 * \brief   This API sets data for ECC logic computaion
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 * \param   data          [IN]  data value for which ECC needs to be computed
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_setTxECCdata(uint32_t base, uint32_t data);

/**
 * \brief   This API gets ECC value evaluated for 16/32 bit data
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 * \param   pEccVal       [OUT] Pointer to the ECC value for input data.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_getTxECCValue(uint32_t base, uint16_t *pEccVal);

/**
 * \brief   This API enables user to generate interrupt on
 *          occurrence of FSI_TxEventList events
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 * \param   intNum        [IN]  Type of interrupt to be generated, INT1 or INT2
 * \param   intFlags      [IN]  TX events on which interrupt should be generated
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_enableTxInterrupt(uint32_t         base,
                                     FSI_InterruptNum intNum,
                                     uint16_t         intFlags);

/**
 * \brief   This API enables user to disable generation interrupt
 *          on occurrence of FSI TX events
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 * \param   intNum        [IN]  Type of interrupt to be generated, INT1 or INT2
 * \param   intFlags      [IN]  TX events on which interrupt generation
 *                              has to be disabled
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_disableTxInterrupt(uint32_t         base,
                                      FSI_InterruptNum intNum,
                                      uint16_t         intFlags);

/**
 * \brief   This API gets address of TX data buffer
 *
 * @n Data buffer is consisting of 16 words from offset- 0x40 to 0x4e
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 * \param   pBufAddr      [OUT] Pointer to the TX data buffer address
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_getTxBufferAddress(uint32_t base, uint32_t *pBufAddr);

/**
 * \brief   This API resets clock or ping timeout counter or entire TX module
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 * \param   submodule     [IN]  name of submodule which is supposed to be reset
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_resetTxModule(uint32_t base, FSI_TxSubmoduleInReset submodule);

/**
 * \brief   This API clears reset on clock or ping timeout counter or entire TX module
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 * \param   submodule     [IN]  name of submodule to be brought out of reset
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_clearTxModuleReset(uint32_t base, FSI_TxSubmoduleInReset submodule);

/**
 * \brief   This API writes data in FSI TX buffer
 *
 * \param   base          [IN]  Base address of the FSI TX module.
 * \param   pArray        [IN]  Address of the array of words to be transmitted.
 * \param   length        [IN]  Length is the number of words in the array to be
 *                              transmitted.
 * \param   bufOffset     [IN]  Offset in TX buffer where data will be written,
 *                              offset is a 16-bit aligned address
 *
 * @note Data Overwrite protection is implemented in this function by ensuring
 *       not more than 16 words are written and also wrap around case is taken
 *       care when more words need to be written if last write happens at
 *       maximum offset in TX buffer
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_writeTxBuffer(uint32_t        base,
                          const uint16_t *pArray,
                          uint16_t        length,
                          uint16_t        bufOffset);

/**
 * \brief   This API initializes FSI TX module
 *
 * @n Software based initialization of the FSI transmitter IP. This is typically
 *    needed only once during initialization or if the module needs to be
 *    reset due to an underrun condition that occurred during operation.
 *
 * \param   base          [IN]  Base address of the FSI TX module
 * \param   prescalar     [IN]  User configurable clock divider for PLL input clock
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_performTxInitialization(uint32_t base, uint16_t prescalar);

/**
 * \brief   This API sends Flush pattern sequence
 *
 * @n Flush pattern sequence sent by a FSI transmit module will bring the
 *    FSI receive module out of reset so that it will then be ready to
 *    receive subsequent frames.
 *
 * \param   base          [IN]  Base address of the FSI TX module
 * \param   prescalar     [IN]  user configurable clock divider for PLL input clock
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_executeTxFlushSequence(uint32_t base, uint16_t prescalar);

#ifdef __cplusplus
}
#endif

#endif

/** @} */
