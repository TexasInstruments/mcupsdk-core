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
 *  \defgroup DRV_EQEP_MODULE APIs for EQEP
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the EQEP module.
 *
 *  @{
 */

/**
 *  \file v0/eqep.h
 *
 *  \brief EQEP Driver API/interface file.
 *
 */

#ifndef EQEP_V0_H_
#define EQEP_V0_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/cslr_eqep.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */
/**
 *  \anchor Eqep_DecoderConfig_t
 *  \name EQEP Decoder Configuration
 *  @{
 */
/** \brief Values that can be passed to #EQEP_setDecoderConfig() as the
 * config parameter.
 */
/** \brief Operation Mode */
#define EQEP_CONFIG_QUADRATURE      (0x0000U)
/** \brief Quadrature-clock Direction-count mode */
#define EQEP_CONFIG_CLOCK_DIR       (0x4000U)
/** \brief Up-count mode, QDIR = 1 */
#define EQEP_CONFIG_UP_COUNT        (0x8000U)
/** \brief Down-count mode, QDIR = 0 */
#define EQEP_CONFIG_DOWN_COUNT      (0xC000U)
/** \brief Resolution Count rising and falling edge */
#define EQEP_CONFIG_2X_RESOLUTION   (0x0000U)
/** \brief Resolution Count rising edge only */
#define EQEP_CONFIG_1X_RESOLUTION   (0x0800U)
/** \brief Do not swap QEPA and QEPB */
#define EQEP_CONFIG_NO_SWAP         (0x0000U)
/** \brief Swap QEPA and QEPB */
#define EQEP_CONFIG_SWAP            (0x0400U)
/** @} */

/**
 *  \anchor Eqep_CompareConfig_t
 *  \name EQEP Compare Configuration
 *  @{
 */
/** \brief Values that can be passed to #EQEP_setCompareConfig() as the
 * config parameter.
 */
/** \brief Sync pulse pin, Disable sync output */
#define EQEP_COMPARE_NO_SYNC_OUT      (0x0000U)
/** \brief Sync output on index pin */
#define EQEP_COMPARE_IDX_SYNC_OUT     (0x2000U)
/** \brief Sync output on strobe pin */
#define EQEP_COMPARE_STROBE_SYNC_OUT  (0x3000U)
/** \brief Disable shadow of QPOSCMP */
#define EQEP_COMPARE_NO_SHADOW        (0x0000U)
/** \brief Load on QPOSCNT = 0 */
#define EQEP_COMPARE_LOAD_ON_ZERO     (0x8000U)
/** \brief Load on QPOSCNT = QPOSCMP */
#define EQEP_COMPARE_LOAD_ON_MATCH    (0xC000U)
/** @} */

/**
 *  \anchor Eqep_IntrSrc_t
 *  \name EQEP Interrupt Sources
 *  @{
 */
/** \brief Values that can be passed to #EQEP_enableInterrupt(),
 * #EQEP_disableInterrupt(), and #EQEP_clearInterruptStatus() as the
 * intFlags parameter and returned by #EQEP_clearInterruptStatus().
 */
/** \brief Global interrupt flag */
#define EQEP_INT_GLOBAL               (0x0001U)
/** \brief Position counter error */
#define EQEP_INT_POS_CNT_ERROR        (0x0002U)
/** \brief Quadrature phase error */
#define EQEP_INT_PHASE_ERROR          (0x0004U)
/** \brief Quadrature direction change */
#define EQEP_INT_DIR_CHANGE           (0x0008U)
/** \brief Watchdog time-out */
#define EQEP_INT_WATCHDOG             (0x0010U)
/** \brief Position counter underflow */
#define EQEP_INT_UNDERFLOW            (0x0020U)
/** \brief Position counter overflow */
#define EQEP_INT_OVERFLOW             (0x0040U)
/** \brief Position-compare ready */
#define EQEP_INT_POS_COMP_READY       (0x0080U)
/** \brief Position-compare match */
#define EQEP_INT_POS_COMP_MATCH       (0x0100U)
/** \brief Strobe event latch */
#define EQEP_INT_STROBE_EVNT_LATCH    (0x0200U)
/** \brief Index event latch */
#define EQEP_INT_INDEX_EVNT_LATCH     (0x0400U)
/** \brief Unit time-out */
#define EQEP_INT_UNIT_TIME_OUT        (0x0800U)
/** \brief QMA error */
#define EQEP_INT_QMA_ERROR            (0x1000U)
/** @} */

/**
 *  \anchor Eqep_ModuleStatus_t
 *  \name EQEP Get Module Status
 *  @{
 */
/** \brief Values that can be returned by #EQEP_getStatus().
 */
/** \brief Unit position event detected */
#define EQEP_STS_UNIT_POS_EVNT        (0x0080U)
/** \brief Direction was clockwise on first index event */
#define EQEP_STS_DIR_ON_1ST_IDX       (0x0040U)
/** \brief Direction is CW (forward) */
#define EQEP_STS_DIR_FLAG             (0x0020U)
/** \brief Direction was CW on index */
#define EQEP_STS_DIR_LATCH            (0x0010U)
/** \brief Capture timer overflow */
#define EQEP_STS_CAP_OVRFLW_ERROR     (0x0008U)
/** \brief Direction changed between position capture events */
#define EQEP_STS_CAP_DIR_ERROR        (0x0004U)
/** \brief First index pulse occurred */
#define EQEP_STS_1ST_IDX_FLAG         (0x0002U)
/** \brief Position counter error */
#define EQEP_STS_POS_CNT_ERROR        (0x0001U)
/** @} */

/**
 *  \anchor Eqep_LatchMode_t
 *  \name EQEP Set Latch Mode
 *  @{
 */
/** \brief Values that can be passed to #EQEP_setLatchMode() as the latchMode parameter.
 */
/** \brief Position counter latch event */
#define EQEP_LATCH_CNT_READ_BY_CPU    (0x0000U)
/** \brief On position counter read */
#define EQEP_LATCH_UNIT_TIME_OUT      (0x0004U)
/** \brief On unit time-out event */
/** \brief Strobe position counter latch event On rising edge of strobe */
#define EQEP_LATCH_RISING_STROBE      (0x0000U)
/** \brief On rising edge when clockwise, on falling when counter clockwise */
#define EQEP_LATCH_EDGE_DIR_STROBE    (0x0040U)
/** \brief Index position counter latch event */
#define EQEP_LATCH_RISING_INDEX       (0x0010U)
/** \brief On rising edge of index */
#define EQEP_LATCH_FALLING_INDEX      (0x0020U)
/** \brief On falling edge of index */
#define EQEP_LATCH_SW_INDEX_MARKER    (0x0030U)
/** \brief On software index marker */
/** @} */

/**
 *  \anchor Eqep_InitMode_t
 *  \name EQEP Set Position Init Mode
 *  @{
 */
/** \brief Values that can be passed to #EQEP_setPositionInitMode() as the initMode parameter.
 */
/** \brief Action is disabled */
#define EQEP_INIT_DO_NOTHING          (0x0000U)
/** \brief Strobe events On rising edge of strobe */
#define EQEP_INIT_RISING_STROBE       (0x0800U)
/** \brief Strobe events On rising edge when clockwise, on falling when counter clockwise */
#define EQEP_INIT_EDGE_DIR_STROBE     (0x0C00U)
/** \brief Index events, On rising edge of index */
#define EQEP_INIT_RISING_INDEX        (0x0200U)
/** \brief Index events, On falling edge of index */
#define EQEP_INIT_FALLING_INDEX       (0x0300U)
/** @} */

/**
 *  \anchor Eqep_getDirection_t
 *  \name EQEP get direction
 *  @{
 */
/** \brief Values that can be return from #EQEP_getDirection() .
 */
/** \brief Moving in the forward direction */
#define EQEP_MOD_FORWARD_DIR          (1)
/** \brief Moving in the reverse direction */
#define EQEP_MOD_BACKWARD_DIR         (-1)
/** @} */

/**
 *  \anchor Eqep_PositionResetMode_t
 *  \name EQEP Position Reset Mode
 *  @{
 */
/**
 * \brief  Enum Values that can be passed to #EQEP_setPositionCounterConfig() as the mode parameter.
 */
typedef uint32_t eqepPositionResetMode_t;
/** \brief Reset position on index pulse */
#define EQEP_POSITION_RESET_IDX                     (0x0000)
/** \brief Reset position on maximum position */
#define EQEP_POSITION_RESET_MAX_POS                 (0x1000)
/** \brief Reset position on the first index pulse */
#define EQEP_POSITION_RESET_1ST_IDX                 (0x2000)
/** \brief Reset position on a unit time event */
#define EQEP_POSITION_RESET_UNIT_TIME_OUT           (0x3000)
/** @} */

/**
 *  \anchor Eqep_CapClkPrescale_t
 *  \name EQEP CapPrescalar parameter
 *  @{
 */
/**
 * \brief  Enum Values that can be passed to #EQEP_setCaptureConfig() as the capPrescale parameter.
 *
 *         CAPCLK is the capture timer clock frequency.
 */
typedef uint32_t eqepCapClkPrescale_t;
/** \brief CAPCLK = SYSCLKOUT/1 */
#define EQEP_CAPTURE_CLK_DIV_1                      (0x00)
/** \brief CAPCLK = SYSCLKOUT/2 */
#define EQEP_CAPTURE_CLK_DIV_2                      (0x10)
/** \brief CAPCLK = SYSCLKOUT/4 */
#define EQEP_CAPTURE_CLK_DIV_4                      (0x20)
/** \brief CAPCLK = SYSCLKOUT/8 */
#define EQEP_CAPTURE_CLK_DIV_8                      (0x30)
/** \brief CAPCLK = SYSCLKOUT/16 */
#define EQEP_CAPTURE_CLK_DIV_16                     (0x40)
/** \brief CAPCLK = SYSCLKOUT/32 */
#define EQEP_CAPTURE_CLK_DIV_32                     (0x50)
/** \brief CAPCLK = SYSCLKOUT/64 */
#define EQEP_CAPTURE_CLK_DIV_64                     (0x60)
/** \brief CAPCLK = SYSCLKOUT/128 */
#define EQEP_CAPTURE_CLK_DIV_128                    (0x070)
/** @} */

/**
 *  \anchor Eqep_UpEvntPrescale_t
 *  \name EQEP unit position event frequency
 *  @{
 */
/**
 * \brief  Enum Values that can be passed to #EQEP_setCaptureConfig() as the  evntPrescaleparameter.
 *
 *         UPEVNT is the unit position event frequency.
 */
typedef uint32_t eqeqUpEvntPrescale_t;
/** \brief UPEVNT = QCLK/1 */
#define EQEP_UNIT_POS_EVNT_DIV_1                    (0)
/** \brief UPEVNT = QCLK/2 */
#define EQEP_UNIT_POS_EVNT_DIV_2                    (1)
/** \brief UPEVNT = QCLK/4 */
#define EQEP_UNIT_POS_EVNT_DIV_4                    (2)
/** \brief UPEVNT = QCLK/8 */
#define EQEP_UNIT_POS_EVNT_DIV_8                    (3)
/** \brief UPEVNT = QCLK/16 */
#define EQEP_UNIT_POS_EVNT_DIV_16                   (4)
/** \brief UPEVNT = QCLK/32 */
#define EQEP_UNIT_POS_EVNT_DIV_32                   (5)
/** \brief UPEVNT = QCLK/64 */
#define EQEP_UNIT_POS_EVNT_DIV_64                   (6)
/** \brief UPEVNT = QCLK/128 */
#define EQEP_UNIT_POS_EVNT_DIV_128                  (7)
/** \brief UPEVNT = QCLK/256 */
#define EQEP_UNIT_POS_EVNT_DIV_256                  (8)
/** \brief UPEVNT = QCLK/512 */
#define EQEP_UNIT_POS_EVNT_DIV_512                  (9)
/** \brief UPEVNT = QCLK/1024 */
#define EQEP_UNIT_POS_EVNT_DIV_1024                 (10)
/** \brief UPEVNT = QCLK/2048 */
#define EQEP_UNIT_POS_EVNT_DIV_2048                 (11)
/** @} */

/**
 *  \anchor Eqep_StrobeSource_t
 *  \name EQEP strobeSrc parameter
 *  @{
 */
/**
 * \brief  Enum Values that can be passed to #EQEP_setStrobeSource() as the  strobeSrc parameter.
 *
 */
typedef uint32_t eqepStrobeSource_t;
/** \brief Strobe signal comes from GPIO */
#define EQEP_STROBE_FROM_GPIO                       (0)
/** \brief Strobe signal is OR'd with ADCSOCA */
#define EQEP_STROBE_OR_ADCSOCA                      (2)
/** \brief Strobe signal is OR'd with ADCSOCB */
#define EQEP_STROBE_OR_ADCSOCB                      (3)
/** @} */

/**
 *  \anchor Eqep_QmaMode_t
 *  \name EQEP qmaMode parameter
 *  @{
 */
/**
 * \brief  Enum Values that can be passed to #EQEP_setQMAModuleMode() as the  qmaMode parameter.
 *
 */
typedef uint32_t eqepQmaMode_t;
/** \brief QMA module is bypassed */
#define EQEP_QMA_MODE_BYPASS                        (0)
/** \brief QMA mode-1 operation is selected */
#define EQEP_QMA_MODE_1                             (1)
/** \brief QMA mode-2 operation is selected */
#define EQEP_QMA_MODE_2                             (2)
/** @} */

/**
 *  \anchor Eqep_EmulationMode_t
 *  \name EQEP emuMode parameter
 *  @{
 */
/**
 * \brief  Enum Values that can be passed to #EQEP_setEmulationMode() as the emuMode parameter.
 *
 */
typedef uint32_t eqepEmulationMode_t;
/** \brief Counters stop immediately */
#define EQEP_EMULATIONMODE_STOPIMMEDIATELY          (0)
/** \brief Counters stop at period rollover */
#define EQEP_EMULATIONMODE_STOPATROLLOVER           (1)
/** \brief Counter unaffected by suspend */
#define EQEP_EMULATIONMODE_RUNFREE                  (2)
/** @} */

/**
 *  \anchor Eqep_Direction_t
 *  \name EQEP direction
 *  @{
 */
/** \brief Moving in the anticlockwise direction */
#define EQEP_DIR_ANTI_CLOCKWISE   (CSL_EQEP_QEPSTS_TYPE1_QDF_VAL_QDF_COUNTERCLK)
/** \brief Moving in the clockwise direction */
#define EQEP_DIR_CLOCKWISE        (CSL_EQEP_QEPSTS_TYPE1_QDF_VAL_QDF_CLK)
/** @} */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief    Enables the eQEP module.
 *
 * \param baseAddr is the base address of the eQEP module.
 *
 * This function enables operation of the enhanced quadrature encoder pulse
 * (eQEP) module.  The module must be configured before it is enabled.
 *
 */
void EQEP_enableModule(uint32_t baseAddr);

/**
 * \brief    Disables the eQEP module.
 *
 * \param baseAddr is the base address of the enhanced quadrature encoder pulse
 * (eQEP) module
 *
 * This function disables operation of the eQEP module.
 *
 */
void EQEP_disableModule(uint32_t baseAddr);

/**
 * \brief    Configures eQEP module's quadrature decoder unit.
 *
 * \param baseAddr is the base address of the eQEP module.
 * \param config is the configuration for the eQEP module decoder unit.
 *
 * This function configures the operation of the eQEP module's quadrature decoder
 * unit.  The config parameter provides the configuration of the decoder
 * and is the logical OR of several values:
 *
 * - #EQEP_CONFIG_2X_RESOLUTION or #EQEP_CONFIG_1X_RESOLUTION specify
 *   if both rising and falling edges should be counted or just rising edges.
 * - #EQEP_CONFIG_QUADRATURE, #EQEP_CONFIG_CLOCK_DIR,
 *   #EQEP_CONFIG_UP_COUNT, or #EQEP_CONFIG_DOWN_COUNT specify if
 *   quadrature signals are being provided on QEPA and QEPB, if a direction
 *   signal and a clock are being provided, or if the direction should be
 *   hard-wired for a single direction with QEPA used for input.
 * - #EQEP_CONFIG_NO_SWAP or #EQEP_CONFIG_SWAP to specify if the
 *   signals provided on QEPA and QEPB should be swapped before being
 *   processed.
 *   - \ref Eqep_DecoderConfig_t.
 *
 */
void EQEP_setDecoderConfig(uint32_t baseAddr, uint16_t config);

/**
 * \brief    Configures eQEP module position counter unit.
 *
 * \param baseAddr is the base address of the eQEP module.
 * \param mode is the configuration for the eQEP module position counter.
 * \param maxPosition specifies the maximum position value.
 *
 * This function configures the operation of the eQEP module position
 * counter.  The mode parameter determines the event on which the position
 * counter gets reset. It should be passed one of the following values:
 * #EQEP_POSITION_RESET_IDX, #EQEP_POSITION_RESET_MAX_POS,
 * #EQEP_POSITION_RESET_1ST_IDX, or #EQEP_POSITION_RESET_UNIT_TIME_OUT.
 * - \ref Eqep_PositionResetMode_t.
 *
 * maxPosition is the maximum value of the position counter and is
 * the value used to reset the position capture when moving in the reverse
 * (negative) direction.
 *
 */
void EQEP_setPositionCounterConfig(uint32_t baseAddr,
                                   eqepPositionResetMode_t mode,
                                   uint32_t maxPosition);

/**
 * \brief    Gets the current encoder position.
 *
 * \param baseAddr is the base address of the eQEP module.
 *
 * This function returns the current position of the encoder.  Depending upon
 * the configuration of the encoder, and the incident of an index pulse, this
 * value may or may not contain the expected data (that is, if in reset on
 * index mode, if an index pulse has not been encountered, the position
 * counter is not yet aligned with the index pulse).
 *
 * \return The current position of the encoder.
 */
uint32_t EQEP_getPosition(uint32_t baseAddr);

/**
 * \brief    Sets the current encoder position.
 *
 * \param baseAddr is the base address of the eQEP module.
 * \param position is the new position for the encoder.
 *
 * This function sets the current position of the encoder; the encoder
 * position is then measured relative to this value.
 *
 */
void EQEP_setPosition(uint32_t baseAddr, uint32_t position);

/**
 * \brief    Gets the current direction of rotation.
 *
 * \param baseAddr is the base address of the eQEP module.
 *
 * This function returns the current direction of rotation.  In this case,
 * current means the most recently detected direction of the encoder; it may
 * not be presently moving but this is the direction it last moved before it
 * stopped.
 * - \ref Eqep_getDirection_t.
 *
 * \return  1 if moving in the forward direction
 *          -1 if moving in the reverse direction.
 */
int32_t EQEP_getDirection(uint32_t baseAddr);

/**
 * \brief    Enables individual eQEP module interrupt sources.
 *
 * \param baseAddr is the base address of the eQEP module.
 * \param intFlags is a bit mask of the interrupt sources to be enabled.
 *
 * This function enables eQEP module interrupt sources. The  intFlags
 * parameter can be any of the following values OR'd together:
 * - #EQEP_INT_POS_CNT_ERROR - Position counter error
 * - #EQEP_INT_PHASE_ERROR - Quadrature phase error
 * - #EQEP_INT_DIR_CHANGE - Quadrature direction change
 * - #EQEP_INT_WATCHDOG - Watchdog time-out
 * - #EQEP_INT_UNDERFLOW - Position counter underflow
 * - #EQEP_INT_OVERFLOW - Position counter overflow
 * - #EQEP_INT_POS_COMP_READY - Position-compare ready
 * - #EQEP_INT_POS_COMP_MATCH - Position-compare match
 * - #EQEP_INT_STROBE_EVNT_LATCH - Strobe event latch
 * - #EQEP_INT_INDEX_EVNT_LATCH - Index event latch
 * - #EQEP_INT_UNIT_TIME_OUT - Unit time-out
 * - #EQEP_INT_QMA_ERROR - QMA error
 *
 * - \ref Eqep_IntrSrc_t.
 *
 */
void EQEP_enableInterrupt(uint32_t baseAddr, uint16_t intFlags);

/**
 * \brief    Disables individual eQEP module interrupt sources.
 *
 * \param baseAddr is the base address of the eQEP module.
 * \param intFlags is a bit mask of the interrupt sources to be disabled.
 *
 * This function disables eQEP module interrupt sources. The  intFlags
 * parameter can be any of the following values OR'd together:
 * - #EQEP_INT_POS_CNT_ERROR - Position counter error
 * - #EQEP_INT_PHASE_ERROR - Quadrature phase error
 * - #EQEP_INT_DIR_CHANGE - Quadrature direction change
 * - #EQEP_INT_WATCHDOG - Watchdog time-out
 * - #EQEP_INT_UNDERFLOW - Position counter underflow
 * - #EQEP_INT_OVERFLOW - Position counter overflow
 * - #EQEP_INT_POS_COMP_READY - Position-compare ready
 * - #EQEP_INT_POS_COMP_MATCH - Position-compare match
 * - #EQEP_INT_STROBE_EVNT_LATCH - Strobe event latch
 * - #EQEP_INT_INDEX_EVNT_LATCH - Index event latch
 * - #EQEP_INT_UNIT_TIME_OUT - Unit time-out
 * - #EQEP_INT_QMA_ERROR - QMA error
 *
 * - \ref Eqep_IntrSrc_t.
 *
 */
void EQEP_disableInterrupt(uint32_t baseAddr, uint16_t intFlags);

/**
 * \brief    Get Enabled interrupts of eQEP module.
 *
 * \param baseAddr is the base address of the eQEP module.
 *
 * \return the current enabled interrupts, enumerated as a bit field of
 * the following values:
 * - #EQEP_INT_GLOBAL - Global interrupt flag
 * - #EQEP_INT_POS_CNT_ERROR - Position counter error
 * - #EQEP_INT_PHASE_ERROR - Quadrature phase error
 * - #EQEP_INT_DIR_CHANGE - Quadrature direction change
 * - #EQEP_INT_WATCHDOG - Watchdog time-out
 * - #EQEP_INT_UNDERFLOW - Position counter underflow
 * - #EQEP_INT_OVERFLOW - Position counter overflow
 * - #EQEP_INT_POS_COMP_READY - Position-compare ready
 * - #EQEP_INT_POS_COMP_MATCH - Position-compare match
 * - #EQEP_INT_STROBE_EVNT_LATCH - Strobe event latch
 * - #EQEP_INT_INDEX_EVNT_LATCH - Index event latch
 * - #EQEP_INT_UNIT_TIME_OUT - Unit time-out
 * - #EQEP_INT_QMA_ERROR - QMA error
 *
 * - \ref Eqep_IntrSrc_t.
 *
 */
uint16_t EQEP_getEnabledInterrupt(uint32_t baseAddr);

/**
 * \brief    Gets the current interrupt status.
 *
 * \param baseAddr is the base address of the eQEP module.
 *
 * This function returns the interrupt status for the eQEP module
 * module.
 *
 * \return the current interrupt status, enumerated as a bit field of
 * the following values:
 * - #EQEP_INT_GLOBAL - Global interrupt flag
 * - #EQEP_INT_POS_CNT_ERROR - Position counter error
 * - #EQEP_INT_PHASE_ERROR - Quadrature phase error
 * - #EQEP_INT_DIR_CHANGE - Quadrature direction change
 * - #EQEP_INT_WATCHDOG - Watchdog time-out
 * - #EQEP_INT_UNDERFLOW - Position counter underflow
 * - #EQEP_INT_OVERFLOW - Position counter overflow
 * - #EQEP_INT_POS_COMP_READY - Position-compare ready
 * - #EQEP_INT_POS_COMP_MATCH - Position-compare match
 * - #EQEP_INT_STROBE_EVNT_LATCH - Strobe event latch
 * - #EQEP_INT_INDEX_EVNT_LATCH - Index event latch
 * - #EQEP_INT_UNIT_TIME_OUT - Unit time-out
 * - #EQEP_INT_QMA_ERROR - QMA error
 *
 * - \ref Eqep_IntrSrc_t.
 *
 */
uint16_t EQEP_getInterruptStatus(uint32_t baseAddr);

/**
 * \brief    Clears eQEP module interrupt sources.
 *
 * \param baseAddr is the base address of the eQEP module.
 * \param intFlags is a bit mask of the interrupt sources to be cleared.
 *
 * This function clears eQEP module interrupt flags. The  intFlags
 * parameter can be any of the following values OR'd together:
 * - #EQEP_INT_GLOBAL - Global interrupt flag
 * - #EQEP_INT_POS_CNT_ERROR - Position counter error
 * - #EQEP_INT_PHASE_ERROR - Quadrature phase error
 * - #EQEP_INT_DIR_CHANGE - Quadrature direction change
 * - #EQEP_INT_WATCHDOG - Watchdog time-out
 * - #EQEP_INT_UNDERFLOW - Position counter underflow
 * - #EQEP_INT_OVERFLOW - Position counter overflow
 * - #EQEP_INT_POS_COMP_READY - Position-compare ready
 * - #EQEP_INT_POS_COMP_MATCH - Position-compare match
 * - #EQEP_INT_STROBE_EVNT_LATCH - Strobe event latch
 * - #EQEP_INT_INDEX_EVNT_LATCH - Index event latch
 * - #EQEP_INT_UNIT_TIME_OUT - Unit time-out
 * - #EQEP_INT_QMA_ERROR - QMA error
 *
 * - \ref Eqep_IntrSrc_t.
 *
 * Note that the #EQEP_INT_GLOBAL value is the global interrupt flag. In
 * order to get any further eQEP interrupts, this flag must be cleared.
 *
 */
void EQEP_clearInterruptStatus(uint32_t baseAddr, uint16_t intFlags);

/**
 * \brief    Forces individual eQEP module interrupts.
 *
 * \param baseAddr is the base address of the eQEP module.
 * \param intFlags is a bit mask of the interrupt sources to be forced.
 *
 * This function forces eQEP module interrupt flags. The  intFlags
 * parameter can be any of the following values OR'd together:
 * - #EQEP_INT_POS_CNT_ERROR
 * - #EQEP_INT_PHASE_ERROR
 * - #EQEP_INT_DIR_CHANGE
 * - #EQEP_INT_WATCHDOG
 * - #EQEP_INT_UNDERFLOW
 * - #EQEP_INT_OVERFLOW
 * - #EQEP_INT_POS_COMP_READY
 * - #EQEP_INT_POS_COMP_MATCH
 * - #EQEP_INT_STROBE_EVNT_LATCH
 * - #EQEP_INT_INDEX_EVNT_LATCH
 * - #EQEP_INT_UNIT_TIME_OUT
 * - #EQEP_INT_QMA_ERROR
 *
 * - \ref Eqep_IntrSrc_t.
 *
 */
void EQEP_forceInterrupt(uint32_t baseAddr, uint16_t intFlags);

/**
 * \brief    Gets the encoder error indicator.
 *
 * \param baseAddr is the base address of the eQEP module.
 *
 * This function returns the error indicator for the eQEP module.  It is an
 * error for both of the signals of the quadrature input to change at the same
 * time.
 *
 * \return true if an error has occurred otherwise false
 */
Bool EQEP_isErrorSet(uint32_t baseAddr);

/**
 * \brief    Returns content of the eQEP module status register
 *
 * \param baseAddr is the base address of the eQEP module.
 *
 * This function returns the contents of the status register.  The value it
 * returns is an OR of the following values:
 *
 * - #EQEP_STS_UNIT_POS_EVNT - Unit position event detected
 * - #EQEP_STS_DIR_ON_1ST_IDX - If set, clockwise rotation (forward
 *   movement) occurred on the first index event
 * - #EQEP_STS_DIR_FLAG - If set, movement is clockwise rotation
 * - #EQEP_STS_DIR_LATCH - If set, clockwise rotation occurred on last
 *   index event marker
 * - #EQEP_STS_CAP_OVRFLW_ERROR - Overflow occurred in eQEP capture timer
 * - #EQEP_STS_CAP_DIR_ERROR - Direction change occurred between position
 *   capture events
 * - #EQEP_STS_1ST_IDX_FLAG - Set by the occurrence of the first index
 *   pulse
 * - #EQEP_STS_POS_CNT_ERROR - Position counter error occurred
 *
 * - \ref Eqep_ModuleStatus_t.
 *
 * \return the value of the QEP status register
 */
uint32_t EQEP_getStatus(uint32_t baseAddr);

/**
 * \brief    Clears selected fields of the eQEP module status register
 *
 * \param baseAddr is the base address of the eQEP module.
 * \param statusFlags is the bit mask of the status flags to be cleared.
 *
 * This function clears the status register fields indicated by
 *  statusFlags. The  statusFlags parameter is the logical OR of any of
 * the following:
 *
 * - #EQEP_STS_UNIT_POS_EVNT - Unit position event detected
 * - #EQEP_STS_CAP_OVRFLW_ERROR - Overflow occurred in eQEP capture timer
 * - #EQEP_STS_CAP_DIR_ERROR - Direction change occurred between position
 *   capture events
 * - #EQEP_STS_1ST_IDX_FLAG - Set by the occurrence of the first index
 *   pulse
 *
 * - \ref Eqep_ModuleStatus_t.
 *
 * \note Only the above status fields can be cleared. All others are
 * read-only, non-sticky fields.
 *
 */
void EQEP_clearStatus(uint32_t baseAddr, uint16_t statusFlags);

/**
 * \brief    Configures eQEP module edge-capture unit.
 *
 * \param baseAddr is the base address of the eQEP module.
 * \param capPrescale is the prescaler setting of the eQEP capture timer clk.
 * \param evntPrescale is the prescaler setting of the unit position event
 * frequency.
 *
 * This function configures the operation of the eQEP module edge-capture
 * unit.  The  capPrescale parameter provides the configuration of the eQEP
 * capture timer clock rate. It determines by which power of 2 between 1 and
 * 128 inclusive SYSCLKOUT is divided. The macros for this parameter are in
 * the format of EQEP_CAPTURE_CLK_DIV_X, where X is the divide value. For
 * example, #EQEP_CAPTURE_CLK_DIV_32 will give a capture timer clock
 * frequency that is SYSCLKOUT/32.
 *
 * The  evntPrescale parameter determines how frequently a unit position
 * event occurs. The macro that can be passed this parameter is in the format
 * EQEP_UNIT_POS_EVNT_DIV_X, where X is the number of quadrature clock
 * periods between unit position events. For example,
 * #EQEP_UNIT_POS_EVNT_DIV_16 will result in a unit position event
 * frequency of QCLK/16.
 *
 * - \ref Eqep_CapClkPrescale_t and - \ref Eqep_UpEvntPrescale_t.
 *
 */
void EQEP_setCaptureConfig(uint32_t baseAddr,
                           eqepCapClkPrescale_t capPrescale,
                           eqeqUpEvntPrescale_t evntPrescale);

/**
 * \brief    Enables the eQEP module edge-capture unit.
 *
 * \param baseAddr is the base address of the eQEP module.
 *
 * This function enables operation of the eQEP module's edge-capture unit.
 *
 */
void EQEP_enableCapture(uint32_t baseAddr);

/**
 * \brief    Disables the eQEP module edge-capture unit.
 *
 * \param baseAddr is the base address of the eQEP module.
 *
 * This function disables operation of the eQEP module's edge-capture unit.
 *
 */
void EQEP_disableCapture(uint32_t baseAddr);

/**
 * \brief    Gets the encoder capture period.
 *
 * \param baseAddr is the base address of the eQEP module.
 *
 * This function returns the period count value between the last successive
 * eQEP position events.
 *
 * \return The period count value between the last successive position events
 */
uint32_t EQEP_getCapturePeriod(uint32_t baseAddr);

/**
 * \brief    Gets the encoder capture timer value.
 *
 * \param baseAddr is the base address of the eQEP module.
 *
 * This function returns the time base for the edge capture unit.
 *
 * \return The capture timer value
 */
uint32_t EQEP_getCaptureTimer(uint32_t baseAddr);

/**
 * \brief    Enables the eQEP module position-compare unit.
 *
 * \param baseAddr is the base address of the eQEP module.
 *
 * This function enables operation of the eQEP module's position-compare unit.
 *
 */
void EQEP_enableCompare(uint32_t baseAddr);

/**
 * \brief    Disables the eQEP module position-compare unit.
 *
 * \param baseAddr is the base address of the eQEP module.
 *
 * This function disables operation of the eQEP module's position-compare
 * unit.
 *
 */
void EQEP_disableCompare(uint32_t baseAddr);

/**
 * \brief    Configures the position-compare unit's sync output pulse width.
 *
 * \param baseAddr is the base address of the eQEP module.
 * \param cycles is the width of the pulse that can be generated on a
 * position-compare event.  It is in units of 4 SYSCLKOUT cycles.
 *
 * This function configures the width of the sync output pulse.  The width of
 * the pulse will be  cycles * 4 * the width of a SYSCLKOUT cycle.  The
 * maximum width is 4096 * 4 * SYSCLKOUT cycles.
 *
 * \return  CSL_PASS = success
 *          CSL_EBADARGS = Invalid base address parameter
 *
 */
int32_t EQEP_setComparePulseWidth(uint32_t baseAddr, uint16_t cycles);

/**
 * \brief    Enables the eQEP module unit timer.
 *
 * \param baseAddr is the base address of the eQEP module.
 * \param period is period value at which a unit time-out interrupt is set.
 *
 * This function enables operation of the eQEP module's peripheral unit timer.
 * The unit timer is clocked by SYSCLKOUT and will set the unit time-out
 * interrupt when it matches the value specified by  period.
 *
 */
void EQEP_enableUnitTimer(uint32_t baseAddr, uint32_t period);

/**
 * \brief    Disables the eQEP module unit timer.
 *
 * \param baseAddr is the base address of the eQEP module.
 *
 * This function disables operation of the eQEP module's peripheral
 * unit timer.
 *
 */
void EQEP_disableUnitTimer(uint32_t baseAddr);

/**
 * \brief    Enables the eQEP module watchdog timer.
 *
 * \param baseAddr is the base address of the eQEP module.
 * \param period is watchdog period value at which a time-out will occur if
 * no quadrature-clock event is detected.
 *
 * This function enables operation of the eQEP module's peripheral watchdog
 * timer.
 *
 * \note When selecting  period, note that the watchdog timer is clocked
 * from SYSCLKOUT/64.
 *
 */
void EQEP_enableWatchdog(uint32_t baseAddr, uint16_t period);

/**
 * \brief    Disables the eQEP module watchdog timer.
 *
 * \param baseAddr is the base address of the eQEP module.
 *
 * This function disables operation of the eQEP module's peripheral watchdog
 * timer.
 *
 */
void EQEP_disableWatchdog(uint32_t baseAddr);

/**
 * \brief    Sets the eQEP module watchdog timer value.
 *
 * \param baseAddr is the base address of the eQEP module.
 * \param value is the value to be written to the watchdog timer.
 *
 * This function sets the eQEP module's watchdog timer value.
 *
 */
void EQEP_setWatchdogTimerValue(uint32_t baseAddr, uint16_t value);

/**
 * \brief    Gets the eQEP module watchdog timer value.
 *
 * \param baseAddr is the base address of the eQEP module.
 *
 * \return Returns the current watchdog timer value.
 */
uint32_t EQEP_getWatchdogTimerValue(uint32_t baseAddr);

/**
 * \brief    Configures the mode in which the position counter is initialized.
 *
 * \param baseAddr is the base address of the eQEP module.
 * \param initMode is the configuration for initializing the position count.
 * See below for a description of this parameter.
 *
 * This function configures the events on which the position count can be
 * initialized.  The  initMode parameter provides the mode as either
 * #EQEP_INIT_DO_NOTHING (no action configured) or one of the following
 * strobe events, index events, or a logical OR of both a strobe event and an
 * index event.
 *
 * - #EQEP_INIT_RISING_STROBE or #EQEP_INIT_EDGE_DIR_STROBE specify
 *   which strobe event will initialize the position counter.
 * - #EQEP_INIT_RISING_INDEX or #EQEP_INIT_FALLING_INDEX specify
 *   which index event will initialize the position counter.
 *
 * - \ref Eqep_InitMode_t.
 *
 * Use #EQEP_setSWPositionInit() to cause a software initialization and
 * #EQEP_setInitialPosition() to set the value that gets loaded into the
 * position counter upon initialization.
 *
 */
void EQEP_setPositionInitMode(uint32_t baseAddr, uint16_t initMode);

/**
 * \brief    Sets the software initialization of the encoder position counter.
 *
 * \param baseAddr is the base address of the eQEP module.
 * \param initialize is a flag to specify if software initialization of the
 * position counter is enabled.
 *
 * This function does a software initialization of the position counter when
 * the  initialize parameter is true. When false, the QEPCTL[SWI] bit
 * is cleared and no action is taken.
 *
 * The init value to be loaded into the position counter can be set with
 * #EQEP_setInitialPosition().  Additional initialization causes can be
 * configured with #EQEP_setPositionInitMode().
 *
 */
void EQEP_setSWPositionInit(uint32_t baseAddr, Bool initialize);

/**
 * \brief     Sets the init value for the encoder position counter.
 *
 * \param baseAddr is the base address of the eQEP module.
 * \param position is the value to be written to the position counter upon.
 * initialization.
 *
 * This function sets the init value for position of the encoder. See
 * #EQEP_setPositionInitMode() to set the initialization cause or
 * #EQEP_setSWPositionInit() to cause a software initialization.
 *
 */
void EQEP_setInitialPosition(uint32_t baseAddr, uint32_t position);

/**
 * \brief    Configures the quadrature modes in which the position count can be latched.
 *
 * \param baseAddr is the base address of the eQEP module.
 * \param latchMode is the configuration for latching of the position count
 * and several other registers.  See below for a description of this
 * parameter.
 *
 * This function configures the events on which the position count and several
 * other registers can be latched.  The  latchMode parameter provides the
 * mode as the logical OR of several values.
 *
 * - #EQEP_LATCH_CNT_READ_BY_CPU or #EQEP_LATCH_UNIT_TIME_OUT specify
 *   the event that latches the position counter.  This latch register can be
 *   read using #EQEP_getPositionLatch(). The capture timer and capture
 *   period are also latched based on this setting, and can be read using
 *   #EQEP_getCaptureTimerLatch() and #EQEP_getCapturePeriodLatch().
 * - #EQEP_LATCH_RISING_STROBE or #EQEP_LATCH_EDGE_DIR_STROBE
 *   specify which strobe event will latch the position counter into the
 *   strobe position latch register.  This register can be read with
 *   #EQEP_getStrobePositionLatch().
 * - #EQEP_LATCH_RISING_INDEX, #EQEP_LATCH_FALLING_INDEX, or
 *   #EQEP_LATCH_SW_INDEX_MARKER specify which index event will latch the
 *   position counter into the index position latch register.  This register
 *   can be read with #EQEP_getIndexPositionLatch().
 *
 * - \ref Eqep_LatchMode_t.
 *
 */
void EQEP_setLatchMode(uint32_t baseAddr, uint32_t latchMode);

/**
 * \brief    Gets the encoder position that was latched on an index event.
 *
 * \param baseAddr is the base address of the eQEP module.
 *
 * This function returns the value in the index position latch register. The
 * position counter is latched into this register on either a rising index
 * edge, a falling index edge, or a software index marker. This is configured
 * using #EQEP_setLatchMode().
 *
 * \return The position count latched on an index event.
 */
uint32_t EQEP_getIndexPositionLatch(uint32_t baseAddr);

/**
 * \brief    Gets the encoder position that was latched on a strobe event.
 *
 * \param baseAddr is the base address of the eQEP module.
 *
 * This function returns the value in the strobe position latch register. The
 * position counter can be configured to be latched into this register on
 * rising strobe edges only or on rising strobe edges while moving clockwise
 * and falling strobe edges while moving counter-clockwise. This is configured
 * using #EQEP_setLatchMode().
 *
 * \return The position count latched on a strobe event.
 */
uint32_t EQEP_getStrobePositionLatch(uint32_t baseAddr);

/**
 * \brief    Gets the encoder position that was latched on a unit time-out event.
 *
 * \param baseAddr is the base address of the eQEP module.
 *
 * This function returns the value in the position latch register. The
 * position counter is latched into this register either on a unit time-out
 * event.
 *
 * \return The position count latch register value.
 */
uint32_t EQEP_getPositionLatch(uint32_t baseAddr);

/**
 * \brief    Gets the encoder capture timer latch.
 *
 * \param baseAddr is the base address of the eQEP module.
 *
 * This function returns the value in the capture timer latch register.  The
 * capture timer value is latched into this register either on a unit time-out
 * event or upon the CPU reading the eQEP position counter. This is configured
 * using #EQEP_setLatchMode().
 *
 * \return The edge-capture timer latch value.
 */
uint32_t EQEP_getCaptureTimerLatch(uint32_t baseAddr);

/**
 * \brief    Gets the encoder capture period latch.
 *
 * \param baseAddr is the base address of the eQEP module.
 *
 * This function returns the value in the capture period latch register.  The
 * capture period value is latched into this register either on a unit
 * time-out event or upon the CPU reading the eQEP position counter. This is
 * configured using #EQEP_setLatchMode().
 *
 * \return The edge-capture period latch value.
 */
uint32_t EQEP_getCapturePeriodLatch(uint32_t baseAddr);

/**
 * \brief    Set the quadrature mode adapter (QMA) module mode
 *
 * \param baseAddr is the base address of the eQEP module.
 * \param qmaMode is the mode in which the QMA module will operate.
 *
 * This function sets the quadrature mode adapter module mode. The possible
 * modes are passed to the function through the  qmaMode parameter which
 * can take the values #EQEP_QMA_MODE_BYPASS, #EQEP_QMA_MODE_1, or
 * #EQEP_QMA_MODE_2.
 *
 * - \ref Eqep_QmaMode_t.
 *
 * To use the QMA module, you must first put the eQEP module into
 * direction-count mode (#EQEP_CONFIG_CLOCK_DIR) using #EQEP_setDecoderConfig().
 *
 */
void EQEP_setQMAModuleMode(uint32_t baseAddr, eqepQmaMode_t qmaMode);

/**
 * \brief    Set the strobe input source of the eQEP module.
 *
 * \param baseAddr is the base address of the eQEP module.
 * \param strobeSrc is the source of the strobe signal.
 *
 * This function sets the source of the eQEP module's strobe signal. The
 * possible values of the  strobeSrc parameter are
 * - #EQEP_STROBE_FROM_GPIO - The strobe is used as-is after passing through
 *   the polarity select logic.
 * - #EQEP_STROBE_OR_ADCSOCA - The strobe is OR'd with the ADCSOCA signal
 *   after passing through the polarity select logic.
 * - #EQEP_STROBE_OR_ADCSOCB - The strobe is OR'd with the ADCSOCB signal
 *   after passing through the polarity select logic.
 *
 * - \ref Eqep_StrobeSource_t.
 *
 */
void EQEP_setStrobeSource(uint32_t baseAddr, eqepStrobeSource_t strobeSrc);

/**
 * \brief    Set the emulation mode of the eQEP module.
 *
 * \param baseAddr is the base address of the eQEP module.
 * \param emuMode is the mode operation upon an emulation suspend.
 *
 * This function sets the eQEP module's emulation mode. This mode determines
 * how the timers are affected by an emulation suspend. Valid values for the
 *  emuMode parameter are the following:
 *
 * - #EQEP_EMULATIONMODE_STOPIMMEDIATELY - The position counter, watchdog
 *   counter, unit timer, and capture timer all stop immediately.
 * - #EQEP_EMULATIONMODE_STOPATROLLOVER - The position counter, watchdog
 *   counter, unit timer all count until period rollover. The capture timer
 *   counts until the next unit period event.
 * - #EQEP_EMULATIONMODE_RUNFREE - The position counter, watchdog counter,
 *   unit timer, and capture timer are all unaffected by an emulation suspend.
 *
 */
void EQEP_setEmulationMode(uint32_t baseAddr, eqepEmulationMode_t emuMode);

/**
 * \brief    Configures eQEP module position-compare unit.
 *
 * \param baseAddr is the base address of the eQEP module.
 * \param config is the configuration for the eQEP module
 * position-compare unit.  See below for a description of this parameter.
 * \param compareValue is the value to which the position count value is
 * compared for a position-compare event.
 * \param cycles is the width of the pulse that can be generated on a
 * position-compare event.  It is in units of 4 SYSCLKOUT cycles.
 *
 * This function configures the operation of the eQEP module position-compare
 * unit.  The  config parameter provides the configuration of the
 * position-compare unit and is the logical OR of several values:
 *
 * - #EQEP_COMPARE_NO_SYNC_OUT, #EQEP_COMPARE_IDX_SYNC_OUT, or
 *   #EQEP_COMPARE_STROBE_SYNC_OUT specify if there is a sync output pulse
 *   and which pin should be used.
 * - #EQEP_COMPARE_NO_SHADOW, #EQEP_COMPARE_LOAD_ON_ZERO, or
 *   #EQEP_COMPARE_LOAD_ON_MATCH specify if a shadow is enabled and when
 *   should the load should occur--QPOSCNT = 0 or QPOSCNT = QPOSCOMP.
 *
 * The  cycles is used to select the width of the sync output pulse. The
 * width of the resulting pulse will be  cycles * 4 * the width of a
 * SYSCLKOUT cycle. The maximum width is 4096 * 4 * SYSCLKOUT cycles.
 *
 * - \ref Eqep_CompareConfig_t.
 *
 * \note You can set the sync pulse width independently using the
 * #EQEP_setComparePulseWidth() function.
 *
 */
void EQEP_setCompareConfig(uint32_t baseAddr, uint16_t config,
                           uint32_t compareValue, uint16_t cycles);

/**
 * \brief    Sets the polarity of the eQEP module's input signals.
 *
 * \param baseAddr is the base address of the eQEP module.
 * \param invertQEPA is the flag to negate the QEPA input.
 * \param invertQEPB is the flag to negate the QEPA input.
 * \param invertIndex is the flag to negate the index input.
 * \param invertStrobe is the flag to negate the strobe input.
 *
 * This function configures the polarity of the inputs to the eQEP module. To
 * negate the polarity of any of the input signals, pass true into its
 * corresponding parameter in this function. Pass false to leave it as-is.
 *
 */
void EQEP_setInputPolarity(uint32_t baseAddr, Bool invertQEPA, Bool invertQEPB,
                           Bool invertIndex, Bool invertStrobe);

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef EQEP_V0_H_ */

/** @} */
