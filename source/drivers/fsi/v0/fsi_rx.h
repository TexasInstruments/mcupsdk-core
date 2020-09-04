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
 *  \defgroup DRV_FSI_RX_MODULE APIs for FSI RX
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the FSI RX module.
 *
 *  @{
 */

/**
 *  \file   v0/fsi_rx.h
 *
 *  \brief  Header file containing various enumerations, structure definitions and function
 *  declarations for the FSI RX IP.
 */

#ifndef FSI_RX_H
#define FSI_RX_H

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/hw_include/cslr_fsi_rx.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 *  \anchor FSI_RxEnum
 *  \name FSI RX Enum type
 *  @{
 */
/**
 * \brief This enumerator defines the types of possible FSI RX events
 *
 *  Values that can be passed to APIs to enable/disable interrupts and
 *  also to set/get/clear event status on FSI RX operation.
 *
 *  There are 12 supported interrupts related to RX events,
 *  all are available as event status as well.
 *      1)  ping watchdog times out
 *      2)  frame watchdog times out
 *      3)  mismatch between hardware computed CRC and received CRC.This
 *          status should be ignored if user chooses SW CRC computation
 *      4)  invalid Frame type detected
 *      5)  invalid EndofFrame bit-pattern
 *      6)  buffer Overrun in RX buffer
 *      7)  received frame without errors
 *      8)  software reads empty RX buffer
 *      9)  received error frame
 *      10) received ping frame
 *      11) software didn't clear FRAME_DONE flag after receiving new
 *          frame
 *      12) received data frame
 */
typedef uint32_t FSI_RxEnumType;

#define FSI_RX_EVT_PING_WD_TIMEOUT  ((uint16_t)0x0001U)
    /** RX ping watchdog times out event */
#define FSI_RX_EVT_FRAME_WD_TIMEOUT ((uint16_t)0x0002U)
    /** RX frame watchdog times out event */
#define FSI_RX_EVT_CRC_ERR          ((uint16_t)0x0004U)
    /** RX frame CRC error event */
#define FSI_RX_EVT_TYPE_ERR         ((uint16_t)0x0008U)
    /** RX frame invalid Frame type event */
#define FSI_RX_EVT_EOF_ERR          ((uint16_t)0x0010U)
    /** RX frame invalid End of Frame event */
#define FSI_RX_EVT_OVERRUN          ((uint16_t)0x0020U)
    /** RX frame buffer overrun event */
#define FSI_RX_EVT_FRAME_DONE       ((uint16_t)0x0040U)
    /** RX frame done without errors event */
#define FSI_RX_EVT_UNDERRUN         ((uint16_t)0x0080U)
    /** Software reads empty RX buffer event */
#define FSI_RX_EVT_ERR_FRAME        ((uint16_t)0x0100U)
    /** RX error frame event */
#define FSI_RX_EVT_PING_FRAME       ((uint16_t)0x0200U)
    /** RX ping frame event */
#define FSI_RX_EVT_FRAME_OVERRUN    ((uint16_t)0x0400U)
    /** Software didn't clear FRAME_DONE flag
     *  after receiving new frame event */
#define FSI_RX_EVT_DATA_FRAME       ((uint16_t)0x0800U)
    /** RX data frame event */

/**
 * \brief  Mask of all RX Events, ORing all event defines
 */
#define FSI_RX_EVTMASK              ((uint16_t)0x0FFFU)

/**
 * \brief  Maximum value in RX delay line tap control
 */
#define FSI_RX_MAX_DELAY_LINE_VAL   ((uint16_t)0x001FU)
/** @} */

/**
 *  \anchor FSI_RxSubmoduleInReset
 *  \name FSI RX submodues that can be reset
 *  @{
 */
/**
 * \brief  List of RX modules that can be reset, can be used with reset APIs
 *
 * @n Three submodules can be reset-
 *      1) RX master core
 *      2) frame watchdog counter
 *      3) ping watchdog counter
 */
typedef uint32_t FSI_RxSubmoduleInReset;

#define FSI_RX_MASTER_CORE_RESET    ((uint32_t)0x0U)
/**< RX master core reset */
#define FSI_RX_FRAME_WD_CNT_RESET   ((uint32_t)0x1U)
/**< RX frame watchdog counter reset*/
#define FSI_RX_PING_WD_CNT_RESET    ((uint32_t)0x2U)
/**< RX ping watchdog counter reset*/
/** @} */

/**
 *  \anchor FSI_RxDelayTapType
 *  \name FSI RX delay tap type
 *  @{
 */
/**
 * \brief  Available RX lines for delay tap selection
 *
 * @n Delay tapping can be done on 3 lines:
 *      1) RXCLK
 *      2) RXD0
 *      3) RXD1
 */
typedef uint32_t FSI_RxDelayTapType;

#define FSI_RX_DELAY_CLK            ((uint32_t)0x0U)
/**< RX CLK line delay tap */
#define FSI_RX_DELAY_D0             ((uint32_t)0x1U)
/**< RX D0 line delay tap */
#define FSI_RX_DELAY_D1             ((uint32_t)0x2U)
/**< RX D1 line delay tap */
/** @} */

/**
 *  \anchor FSI_ExtFrameTriggerSrc
 *  \name FSI external frame trigger source
 *  @{
 */
/**
 * \brief  Indexes of available EPWM SOC triggers
 *
 * @n There are 16 ePWM SOC events as external triggers for FSI frame
 *    transfers. Indexes 0:7 and 24:31 are reserved out of total 32
 *    muxed external triggers.
 */
typedef uint32_t FSI_ExtFrameTriggerSrc;

#define FSI_EXT_TRIGSRC_EPWM1_SOCA  ((uint32_t)0x08U)
/**< FSI external trigger source for ePWM1 SOCA */
#define FSI_EXT_TRIGSRC_EPWM1_SOCB  ((uint32_t)0x09U)
/**< FSI external trigger source for ePWM1 SOCB */
#define FSI_EXT_TRIGSRC_EPWM2_SOCA  ((uint32_t)0x0AU)
/**< FSI external trigger source for ePWM2 SOCA */
#define FSI_EXT_TRIGSRC_EPWM2_SOCB  ((uint32_t)0x0BU)
/**< FSI external trigger source for ePWM2 SOCB */
#define FSI_EXT_TRIGSRC_EPWM3_SOCA  ((uint32_t)0x0CU)
/**< FSI external trigger source for ePWM3 SOCA */
#define FSI_EXT_TRIGSRC_EPWM3_SOCB  ((uint32_t)0x0DU)
/**< FSI external trigger source for ePWM3 SOCB */
#define FSI_EXT_TRIGSRC_EPWM4_SOCA  ((uint32_t)0x0EU)
/**< FSI external trigger source for ePWM4 SOCA */
#define FSI_EXT_TRIGSRC_EPWM4_SOCB  ((uint32_t)0x0FU)
/**< FSI external trigger source for ePWM4 SOCB */
#define FSI_EXT_TRIGSRC_EPWM5_SOCA  ((uint32_t)0x10U)
/**< FSI external trigger source for ePWM5 SOCA */
#define FSI_EXT_TRIGSRC_EPWM5_SOCB  ((uint32_t)0x11U)
/**< FSI external trigger source for ePWM5 SOCB */
#define FSI_EXT_TRIGSRC_EPWM6_SOCA  ((uint32_t)0x12U)
/**< FSI external trigger source for ePWM6 SOCA */
#define FSI_EXT_TRIGSRC_EPWM6_SOCB  ((uint32_t)0x13U)
/**< FSI external trigger source for ePWM6 SOCB */
#define FSI_EXT_TRIGSRC_EPWM7_SOCA  ((uint32_t)0x14U)
/**< FSI external trigger source for ePWM7 SOCA */
#define FSI_EXT_TRIGSRC_EPWM7_SOCB  ((uint32_t)0x15U)
/**< FSI external trigger source for ePWM7 SOCB */
#define FSI_EXT_TRIGSRC_EPWM8_SOCA  ((uint32_t)0x16U)
/**< FSI external trigger source for ePWM8 SOCA */
#define FSI_EXT_TRIGSRC_EPWM8_SOCB  ((uint32_t)0x17U)
/**< FSI external trigger source for ePWM8 SOCB */
/** @} */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * \brief   This API enables internal loopback where mux will select internal
 *          pins coming from RX module instead of what comes from pins
 *
 * \param   base          [IN]  Base address of the FSI RX module.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_enableRxInternalLoopback(uint32_t base);

/**
 * \brief   This API disables internal loopback where mux will
 *          not use internal pins coming from RX module
 *
 * \param   base          [IN]  Base address of the FSI RX module.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_disableRxInternalLoopback(uint32_t base);

/**
 * \brief   This API enables SPI clock paring, receive clock is selected
 *          from the internal port coming from RX module
 *
 * @n This API is only applicable when communicating with a SPI interface
 *
 * \param   base          [IN]  Base address of the FSI RX module.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_enableRxSPIPairing(uint32_t base);

/**
 * \brief   This API disables SPI clock paring, selects regular
 *          receive clock coming from the pins
 *
 * @n This API is only applicable when communicating with a SPI interface
 *
 * \param   base          [IN]  Base address of the FSI RX module.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_disableRxSPIPairing(uint32_t base);

/**
 * \brief   This API selects number of data lines used for receiving
 *
 * \param   base          [IN]  Base address of the FSI RX module.
 * \param   dataWidth     [IN]  Data lines used for RX operation
 *                              refer #FSI_DataWidth.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_setRxDataWidth(uint32_t base, FSI_DataWidth dataWidth);

/**
 * \brief   This API enables SPI compatible mode in FSI RX
 *
 * @n This API is only applicable when communicating with a SPI interface
 *
 * \param   base          [IN]  Base address of the FSI RX module.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_enableRxSPIMode(uint32_t base);

/**
 * \brief   This API disables SPI compatible mode in FSI RX
 *
 * @n This API is only applicable when communicating with a SPI interface
 *
 * \param   base          [IN]  Base address of the FSI RX module.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_disableRxSPIMode(uint32_t base);

/**
 * \brief   This API sets the RX frame size if frame type is user/software
 *          defined frame
 *
 * \param   base          [IN]  Base address of the FSI RX module.
 * \param   nWords        [IN]  number of data words in a software defined frame.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_setRxSoftwareFrameSize(uint32_t base, uint16_t nWords);

/**
 * \brief   This API select between 16-bit and 32-bit ECC computation for FSI RX
 *
 * \param   base            [IN]  Base address of the FSI RX module.
 * \param   eccComputeWidth [IN]  Refer #FSI_ECCComputeWidth
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_setRxECCComputeWidth(uint32_t base, FSI_ECCComputeWidth eccComputeWidth);

/**
 * \brief   This API sets HW/SW initiated RX ping timeout mode
 *
 * \param   base            [IN]  Base address of the FSI RX module.
 * \param   pingTimeoutMode [IN]  Refer #FSI_PingTimeoutMode
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_setRxPingTimeoutMode(uint32_t base, FSI_PingTimeoutMode pingTimeoutMode);

/**
 * \brief   This API gets frame type received in the last successful frame
 *
 * \param   base          [IN]  Base address of the FSI RX module.
 * \param   pFrameType    [OUT] Pointer to the RX frame type.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_getRxFrameType(uint32_t base, FSI_FrameType *pFrameType);

/**
 * \brief   This API gets frame tag received for the last successful frame
 *
 * \param   base          [IN]  Base address of the FSI RX module.
 * \param   pFrameTag     [OUT] Pointer to the RX frame tag.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_getRxFrameTag(uint32_t base, uint16_t *pFrameTag);

/**
 * \brief   This API gets User-Data (8-bit) field for received data frame.
 *
 * \param   base          [IN]  Base address of the FSI RX module.
 * \param   pUserData     [OUT] Pointer to the user data value.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_getRxUserDefinedData(uint32_t base, uint16_t *pUserData);

/**
 * \brief   This API gets current status of all the event/error flags
 *
 * \param   base          [IN]  Base address of the FSI RX module
 * \param   pEvtFlags     [OUT] Pointer to status of event/error flags, each bit
 *                              of integer is associated with one error flag
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_getRxEventStatus(uint32_t base, uint16_t *pEvtFlags);


/**
 * \brief   This API enables user to set RX event/error flags
 *
 * \param   base          [IN]  Base address of the FSI RX module
 * \param   evtFlags      [IN]  event/error flags to be set
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_forceRxEvents(uint32_t base, uint16_t evtFlags);

/**
 * \brief   This API enables user to clear RX event/error flags
 *
 * \param   base          [IN]  Base address of the FSI RX module
 * \param   evtFlags      [IN]  event/error flags to be cleared
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_clearRxEvents(uint32_t base, uint16_t evtFlags);

/**
 * \brief   This API gets CRC value received in data frame/frame
 *
 * \param   base          [IN]  Base address of the FSI RX module
 * \param   pCrcVal       [OUT] Pointer to CRC value received in data frame
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_getRxReceivedCRC(uint32_t base, uint16_t *pCrcVal);

/**
 * \brief   This API gets CRC value computed for data received
 *
 * \param   base          [IN]  Base address of the FSI RX module
 * \param   pCrcVal       [OUT] Pointer to CRC value computed on received data
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_getRxComputedCRC(uint32_t base, uint16_t *pCrcVal);

/**
 * \brief   This API sets the value for receive buffer pointer at desired location
 *
 * \param   base          [IN]  Base address of the FSI RX module
 * \param   bufPtrOff     [IN]  4 bit offset pointer in RX buffer from where
 *                              received data will be read
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_setRxBufferPtr(uint32_t base, uint16_t bufPtrOff);

/**
 * \brief   This API gets current buffer pointer location
 *
 * @n There could be lag due to synchronization, hence value is accurate
 *    only when no current reception is happening
 *
 * \param   base          [IN]  Base address of the FSI RX module
 * \param   pBufPtrLoc    [OUT] Pointer to current buffer pointer location
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_getRxBufferPtr(uint32_t base, uint16_t *pBufPtrLoc);

/**
 * \brief   This API gets valid number of data words present in buffer
 *          which have not been read out yet
 *
 * @n There could be lag due to synchronization, hence value is accurate
 *    only when no current reception is happening
 *
 * \param   base          [IN]  Base address of the FSI RX module
 * \param   pWordCnt      [OUT] Pointer to number of data words present in buffer
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_getRxWordCount(uint32_t base, uint16_t *pWordCnt);

/**
 * \brief   This API enables the frame watchdog counter logic to count
 *          every time it start to receive a frame
 *
 * \param   base          [IN]  Base address of the FSI RX module
 * \param   wdRef         [IN]  reference value for frame watchdog time-out counter
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_enableRxFrameWatchdog(uint32_t base,uint32_t wdRef);

/**
 * \brief   This API disables the frame watchdog counter logic
 *
 * \param   base          [IN]  Base address of the FSI RX module
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_disableRxFrameWatchdog(uint32_t base);

/**
 * \brief   This API gets current value of frame watchdog counter
 *
 * \param   base          [IN]  Base address of the FSI RX module
 * \param   pWdCnt        [OUT] Pointer to current value of frame watchdog counter
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_getRxFrameWatchdogCounter(uint32_t base, uint32_t *pWdCnt);

/**
 * \brief   This API enables the ping watchdog counter logic and once the set
 *          time elapses it will indicate ping watchdog time-out has occurred
 *
 * \param   base          [IN]  Base address of the FSI RX module
 * \param   wdRef         [IN]  reference value for ping watchdog time-out counter
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_enableRxPingWatchdog(uint32_t base, uint32_t wdRef);

/**
 * \brief   This API disables the ping watchdog counter logic
 *
 * \param   base          [IN]  Base address of the FSI RX module
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_disableRxPingWatchdog(uint32_t base);

/**
 * \brief   This API gets current value of ping watchdog counter
 *
 * \param   base          [IN]  Base address of the FSI RX module
 * \param   pWdCnt        [OUT] Pointer to current value of ping watchdog counter
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_getRxPingWatchdogCounter(uint32_t base, uint32_t *pWdCnt);

/**
 * \brief   This API gets the value of tag received for last ping frame
 *
 * \param   base          [IN]  Base address of the FSI RX module
 * \param   pPingTag      [OUT] Pointer to ping frame tag value
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_getRxPingTag(uint32_t base, uint16_t *pPingTag);

/**
 * \brief   This API locks the control of all receive control registers,
 *          once locked further writes will not take effect until system
 *          reset occurs
 *
 * \param   base          [IN]  Base address of the FSI RX module
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_lockRxCtrl(uint32_t base);

/**
 * \brief   This API sets RX ECC data on which ECC (SEC-DED) computaion logic runs
 *
 * \param   base          [IN]  Base address of the FSI RX module
 * \param   rxECCdata     [IN]  RX ECC data for ECC logic
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_setRxECCData(uint32_t base, uint32_t rxECCdata);

/**
 * \brief   This API sets received ECC value on which ECC (SEC-DED) computaion logic runs
 *
 * \param   base          [IN]  Base address of the FSI RX module
 * \param   rxECCvalue    [IN]  Received ECC value in a data frame
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_setRxReceivedECCValue(uint32_t base, uint16_t rxECCvalue);

/**
 * \brief   This API gets ECC corrected data
 *
 * \param   base          [IN]  Base address of the FSI RX module
 * \param   pEccData      [OUT] Pointer to 32 bit ECC corrected data
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_getRxECCCorrectedData(uint32_t base, uint32_t *pEccData);

/**
 * \brief   This API gets ECC Log details
 *
 * \param   base          [IN]  Base address of the FSI RX module
 * \param   pEccLog       [OUT] Pointer to ECC Log value (8 bit)
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_getRxECCLog(uint32_t base, uint16_t *pEccLog);

/**
 * \brief   This API enables user to generate interrupt on occurrence of RX events
 *
 * \param   base          [IN]  Base address of the FSI RX module
 * \param   intNum        [IN]  Type of interrupt to be generated, INT1 or INT2,
 *                              refer #FSI_InterruptNum
 * \param   intFlags      [IN]  events on which interrupt should be generated.
 *                              Each bit will represent one event, bits for the
 *                              events on which user want to generate interrupt
 *                              will be set others remain clear
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_enableRxInterrupt(uint32_t         base,
                              FSI_InterruptNum intNum,
                              uint16_t         intFlags);

/**
 * \brief   This API enables user to disable interrupt generation on RX events
 *
 * \param   base          [IN]  Base address of the FSI RX module
 * \param   intNum        [IN]  Type of interrupt to be generated, INT1 or INT2,
 *                              refer #FSI_InterruptNum
 * \param   intFlags      [IN]  events on which interrupt generation
 *                              has to be disabled.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_disableRxInterrupt(uint32_t         base,
                               FSI_InterruptNum intNum,
                               uint16_t         intFlags);

/**
 * \brief   This API gets address of RX data buffer
 *
 * @n Data buffer is consisting of 16 words from offset- 0x40 to 0x4e
 *
 * \param   base          [IN]  Base address of the FSI RX module
 * \param   pAddr         [OUT] Pointer to RX data buffer address
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_getRxBufferAddress(uint32_t base, uint32_t *pAddr);

/**
 * \brief   This API resets frame watchdog, ping watchdog or entire RX module
 *
 * \param   base          [IN]  Base address of the FSI RX module
 * \param   submodule     [IN]  submodule which is supposed to be reset
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_resetRxModule(uint32_t base, FSI_RxSubmoduleInReset submodule);

/**
 * \brief   This API clears resets on frame watchdog, ping watchdog or entire RX module
 *
 * \param   base          [IN]  Base address of the FSI RX module
 * \param   submodule     [IN]  submodule which is to be brought out of reset
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_clearRxModuleReset(uint32_t base, FSI_RxSubmoduleInReset submodule);

/**
 * \brief   This API reads data from FSI RX buffer
 *
 * \param   base          [IN]  Base address of the FSI RX module
 * \param   pArray        [IN]  Pointer to the array of words to be received
 * \param   length        [IN]  Number of words in the array to be received
 * \param   bufOffset     [IN]  Offset in RX buffer where data will be read,
 *                              offset is 16-bit aligned address
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_readRxBuffer(uint32_t  base,
                         uint16_t *pArray,
                         uint16_t  length,
                         uint16_t  bufOffset);

/**
 * \brief   This API adds delay for selected RX tap line
 *
 * \param   base          [IN]  Base address of the FSI RX module
 * \param   delayTapType  [IN]  RX line for which delay needs to be added
 *                              it can be either RXCLK,RXD0 or RXD1
 * \param   tapValue      [IN]  5 bit value of the amount of delay to be added
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_configRxDelayLine(uint32_t           base,
                              FSI_RxDelayTapType delayTapType,
                              uint16_t           tapValue);

/**
 * \brief   This API initializes FSI RX module
 *
 * @n Software based initialization of the FSI receiver module.This is
 *    typically needed only once during initialization. However, if there are
 *    framing errors in the received frames, then the receive module needs
 *    to be  reset so that subsequent frames/packets can be handled fresh
 *
 *
 * \param   base          [IN]  Base address of the FSI RX module
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 */
int32_t FSI_performRxInitialization(uint32_t base);

#ifdef __cplusplus
}
#endif

#endif

/** @} */
