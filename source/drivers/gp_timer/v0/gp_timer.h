/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
 *  \defgroup DRV_GPTIMER_MODULE APIs for GPTIMER
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the GPTIMER modules.
 *
 *  @{
 */

/**
 *  \file v0/gp_timer.h
 *
 *  \brief GPTIMER Driver API/interface file.
 */

#ifndef GP_TIMER_H_
#define GP_TIMER_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdbool.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * \anchor GptimerConfigModes
 * \name MACROS for the possible Configurations of GPTIMER.
 * @{
 */

/** \brief Free Running Mode */
#define GPTIMER_MODE_CONFIG_FREE_RUN                ((uint8_t) 0U)
/** \brief Input Capture Mode */
#define GPTIMER_MODE_CONFIG_INPUT_CAPTURE           ((uint8_t) 1U)
/** \brief Output Compare Mode */
#define GPTIMER_MODE_CONFIG_OUTPUT_COMPARE          ((uint8_t) 2U)
/** \brief PWM Generation Mode */
#define GPTIMER_MODE_CONFIG_PWM_GEN                 ((uint8_t) 3U)

/** @} */

/**
 * \anchor GptimerInputCaptureModes
 * \name MACROS for the possible Configurations of GPTIMER input Capture Mode.
 * @{
 */

/** \brief Capture Single Event */
#define GPTIMER_INPUT_CAPTURE_MODE_SINGLE           ((uint32_t) 0U)
/** \brief Capture Two Events */
#define GPTIMER_INPUT_CAPTURE_MODE_SECOND           ((uint32_t) 1U)

/** @} */

/**
 * \anchor GptimerInputCaptureEventModes
 * \name MACROS for the possible Configurations of GPTIMER input Capture event.
 * @{
 */

/** \brief No Event Capture */
#define GPTIMER_INPUT_CAPTURE_EVENT_NO_CAPTURE      ((uint32_t) 0U)
/** \brief Capture Event on Rising Edge */
#define GPTIMER_INPUT_CAPTURE_EVENT_RISING          ((uint32_t) 1U)
/** \brief Capture Event on Falling Edge */
#define GPTIMER_INPUT_CAPTURE_EVENT_FALLING         ((uint32_t) 2U)
/** \brief Capture Event on both rising and falling edge */
#define GPTIMER_INPUT_CAPTURE_EVENT_EDGE            ((uint32_t) 3U)

/** @} */

/**
 * \anchor GptimerPWMOutputTrigModes
 * \name MACROS for the possible Configurations of GPTIMER PWM output Trigger.
 * @{
 */

/** \brief PWM Uses No Trigger */
#define GPTIMER_PWM_OUT_NO_TRIGGER                  ((uint32_t) 0U)
/** \brief PWM uses overflow as a Trigger */
#define GPTIMER_PWM_OUT_OVERFLOW_TRIGGER            ((uint32_t) 1U)
/** \brief PWM uses overflow and compare match as a trigger */
#define GPTIMER_PWM_OUT_OVERFLOW_MATCH_TRIGGER      ((uint32_t) 2U)

/** @} */

/**
 * \anchor GptimerPWMOutputPinDefaultVal
 * \name MACROS for the possible Configurations of PWM output Pin Default Val.
 * @{
 */

/** \brief Pin Default value 0 */
#define GPTIMER_PWM_OUT_PIN_DEFAULT_0               ((uint32_t) 0U)
/** \brief Pin Default value 0 */
#define GPTIMER_PWM_OUT_PIN_DEFAULT_1               ((uint32_t) 1U)

/** @} */




/**
 * \anchor GptimerPWMOutputModulationType
 * \name MACROS for the possible Configurations of PWM output Pin Modulation
 *       Type.
 * @{
 */

/** \brief Pin Modulation type Pulse */
#define GPTIMER_PWM_OUT_PIN_MODULATION_PULSE        ((uint32_t) 0U)
/** \brief Pin Modulation type Toggle */
#define GPTIMER_PWM_OUT_PIN_MODULATION_TOGGLE       ((uint32_t) 1U)

/** @} */

/**
 * \anchor GptimerIrqMask
 * \name MACROS for the possible IRQ Status Mask.
 * @{
 */

#define TIMER_IRQ_TCAR_IT_FLAG_MASK                 (uint32_t)0x04U
#define TIMER_IRQ_OVF_IT_FLAG_MASK                  (uint32_t)0x02U
#define TIMER_IRQ_MAT_IT_FLAG_MASK                  (uint32_t)0x01U

/** @} */

/* ========================================================================== */
/*                          Structure Declarations                            */
/* ========================================================================== */

/**
 *  \brief A handle that is returned from a GPTIMER_open() call.
 */
typedef struct GPTIMER_Config_s     *GPTIMER_Handle;

/* ========================================================================== */
/*                      Function pointers Declarations                        */
/* ========================================================================== */

/**
 *  \brief  GPTIMER Overflow Callback Function
 *
 *  User definable callback function prototype. The GPTIMER driver will call the
 *  defined function and pass in the GPTIMER driver's handle when an overflow
 *  happens.
 *
 *  The callback will be called if the overflow interrupt is enabled.
 *
 *  \param  handle              GPTIMER_Handle
 */
typedef void (*GPTIMER_OverflowCallbackFxn) (GPTIMER_Handle handle);

/**
 *  \brief  GPTIMER Compare Match Callback Function
 *
 *  User definable callback function prototype. The GPTIMER driver will call the
 *  defined function and pass in the GPTIMER driver's handle when a compare
 *  match happens.
 *
 *  The Callback will be called if the Compare Match interrupt is enabled.
 *
 *  \param  handle              GPTIMER_Handle
 */
typedef void (*GPTIMER_CompareMatchCallbackFxn) (GPTIMER_Handle handle);

/**
 *  \brief  GPTIMER Capture Callback Function
 *
 *  User definable callback function prototype. The GPTIMER driver will call the
 *  defined function and pass in the GPTIMER driver's handle when a capture
 *  event happens.
 *
 *  The Callback will be called if the Capture interrupt is enabled.
 *
 *  \param  handle              GPTIMER_Handle
 */
typedef void (*GPTIMER_CaptureCallbackFxn) (GPTIMER_Handle handle);

/* ========================================================================== */
/*                          Structure Declarations                            */
/* ========================================================================== */

/**
 *  \brief  GPTIMER Hardware attributes
 */
typedef struct GPTIMER_HwAttrs_s {

/** TIMER Peripheral base address */
    uint32_t                    baseAddr;
/** TIMER Peripheral interrupt vector */
    uint32_t                    intNum;
/** TIMER Peripheral event id*/
    uint16_t                    eventId;
/** TIMER input input clock */
    uint32_t                    inputClk;
/** Enable Interrupt */
    bool                        enableIntr;
/** Interrupt Priority */
    uint8_t                     intrPriority;

} GPTIMER_HwAttrs;

/**
 *  \brief  GPTIMER Parameters
 *
 *  GPTIMER parameters are used to with the GPTIMER_open() call. Default values
 *  for these parameters are set using GPTIMER_Params_init().
 */
typedef struct GPTIMER_Params_s {

/** TIMER input clock Presealer Enable */
    bool                        enablePrescaler;
/** TIMER Counter Prescaler */
    uint32_t                    cntPrescaler;
/** TIMER OneShotMode/AutoReload Mode */
    bool                        oneShotMode;
/** TIMER Counter Reload val for Auto Reload mode */
    uint32_t                    cntReloadVal;
/** TIMER Overflow Mask Count */
    uint32_t                    overflowMaskCount;
/** TIMER Counter initial Value */
    uint32_t                    counterInitVal;

} GPTIMER_Params;

/**
 *  \brief  GPTIMER Compare Mode Configuration Parameters
 */
typedef struct GPTIMER_Compare_Config_s
{
/** TIMER Counter Comapare Value */
    uint32_t                    cntCompareValComp;

} GPTIMER_Compare_Config;

/**
 *  \brief  GPTIMER Capture Mode Configuration Parameters
 */
typedef struct GPTIMER_Capture_Config_s
{
/** TIMER input Capture Mode Select \ref GptimerInputCaptureModes */
    uint32_t                    captureMode;
/** TIMER input Capture Mode Select \ref GptimerInputCaptureEventModes */
    uint32_t                    captureEventMode;

} GPTIMER_Capture_Config;

/**
 *  \brief  GPTIMER PWM Generation Mode Configuration Parameters
 */
typedef struct GPTIMER_PWM_Config_s
{
/** TIMER PWM Output Trigger mode \ref GptimerPWMOutputTrigModes */
    uint32_t                    trigOutputPWMMode;
/** TIMER PWM Output PIN default Value \ref GptimerPWMOutputPinDefaultVal */
    uint32_t                    defaultPWMOutSetting;
/** TIMER PWM timer Compare Value */
    uint32_t                    cntCompareValPWM;
/** TIMER PWM modulation type \ref GptimerPWMOutputModulationType */
    uint32_t                    outputModulationType;

} GPTIMER_PWM_Config;

/**
 *  \brief GPTIMER driver object
 */
typedef struct GPTIMER_Object_s
{
/** Grants exclusive access to this GPTIMER Instance */
    SemaphoreP_Object           mutex;
/** Hwi object */
    HwiP_Object                 hwiObj;
/** Flag to indicate whether the instance is opened already */
    bool                        isOpen;
/** GPTIMER open parameters as provided by user */
    GPTIMER_Params              gptimerParams;

/**
 * @name Timer Mode Configuration Parameters
 */

/** Current Timer Configured Mode \ref GptimerConfigModes */
    uint8_t                     timerConfigMode;
/** Comapare Mode Configuration */
    GPTIMER_Compare_Config      compareConfig;
/** Capture Mode Configuration */
    GPTIMER_Capture_Config      captureConfig;
/** PWM Mode Configuration */
    GPTIMER_PWM_Config          pwmConfig;

/**
 * @name Timer Interrupt Callback Functions
 */

/** Callback Function Pointer for overflow Interrupt */
    GPTIMER_OverflowCallbackFxn         overflowCallbackFunction;
/** Callback Function Pointer for Compare Match Interrupt */
    GPTIMER_CompareMatchCallbackFxn     compareMatchCallbackFunction;
/** Callback Function Pointer for Capture Interrupt */
    GPTIMER_CaptureCallbackFxn          captureCallbackFunction;

} GPTIMER_Object;

/**
 *  \brief GPTIMER Global Configuration
 *
 *  The GPTIMER_Config structure contains a set of pointers used to characterise
 *  the GPTIMER driver implementation.
 *
 *  This structure needs to be defined before calling GPTIMER_init() and it must
 *  not be changed thereafter.
 */
typedef struct GPTIMER_Config_s {
/** Pointer to a driver specific data object */
    GPTIMER_Object                      *object;
/** Pointer to a driver specific hardware attributes structure */
    GPTIMER_HwAttrs const               *hwAttrs;
} GPTIMER_Config;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief Initialize the GPTIMER module.
 */
void GPTIMER_init(void);

/**
 *  \brief De-initialize the GPTIMER module.
 */
void GPTIMER_deinit(void);

/**
 *  \brief Function to set default values of GPTIMER_Params in params
 *
 *  \param params           [IN] pointer to the structure to be initialized
 */
void GPTIMER_Params_init(GPTIMER_Params *params);

/**
 *  \brief Open the GPTIMER at index idx with parameters params
 *
 *  \param idx              [IN] Index of GPTIMER to open in global config
 *  \param params           [IN] GPTIMER_Params values to use for opening
 *
 *  \return GPTIMER_Handle
 */
GPTIMER_Handle GPTIMER_open(uint32_t idx, const GPTIMER_Params *params);

/**
 *  \brief Function to close the GPTIMER Peripheral specified by the handle
 *
 *  \pre #GPTIMER_open() has to be called first
 *
 *  \param handle           [IN] #GPTIMER_Handle returned from GPTIMER_open()
 *
 *  \sa #GPTIMER_open()
 */
void GPTIMER_close(GPTIMER_Handle handle);

/**
 * \brief Start the Timer
 *
 * \param handle            [IN] #GPTIMER_Handle returned from GPTIMER_open()
 */
void GPTIMER_start(GPTIMER_Handle handle);

/**
 * \brief Stop the TImer
 *
 * \param handle            [IN] #GPTIMER_Handle returned from GPTIMER_open()
 */
void GPTIMER_stop(GPTIMER_Handle handle);

/**
 * \brief Get timer counter value
 *
 * \param handle            [IN] #GPTIMER_Handle returned from GPTIMER_open()
 *
 * \return Current Counter value
 */
uint32_t GPTIMER_getCount(GPTIMER_Handle handle);

/**
 * \brief Set timer counter value
 *
 * \param handle            [IN] #GPTIMER_Handle returned from GPTIMER_open()
 * \param value             [IN] Counter value to be set
 *
 */
void GPTIMER_setCount(GPTIMER_Handle handle, uint32_t value);

/**
 * \brief Set timer compare value
 *
 * \param handle            [IN] #GPTIMER_Handle returned from GPTIMER_open()
 * \param value             [IN] Compare value to be set
 *
 */
void GPTIMER_setCompareVal(GPTIMER_Handle handle, uint32_t value);

/**
 * \brief Get Timer Capture Value 1
 *
 * \param handle            [IN] #GPTIMER_Handle returned from GPTIMER_open()
 *
 * \return Capture Value 1
 */
uint32_t GPTIMER_getTimerCaptureVal1(GPTIMER_Handle handle);

/**
 * \brief Get Timer Capture Value 2
 *
 * \param handle            [IN] #GPTIMER_Handle returned from GPTIMER_open()
 *
 * \return Capture Value 2
 */
uint32_t GPTIMER_getTimerCaptureVal2(GPTIMER_Handle handle);

/**
 * \brief Change Timer Configuration
 *
 * \param handle            [IN] #GPTIMER_Handle returned from GPTIMER_open()
 * \param timerConfigMode   [IN] Timer Config Mode \ref GptimerConfigModes
 * \param config            [IN] Pointer to the respective Configuration
 *                               Structure.
 *
 * \return #SystemP_SUCCESS if successful; else error on failure
 */
int32_t GPTIMER_setTimerConfigMode( GPTIMER_Handle handle,
                                    uint32_t timerConfigMode, void *config);

/**
 * \brief Update Callback Functions
 *
 * \param handle            [IN] #GPTIMER_Handle returned from GPTIMER_open()
 * \param overflowCbFxn     [IN] Overflow Callback
 * \param compMatchCbFxn    [IN] Compare Match Callback
 * \param captureCbFxn      [IN] Capture Callback
 *
 */
void GPTIMER_setCallbackFxn (GPTIMER_Handle handle,
                            GPTIMER_OverflowCallbackFxn overflowCbFxn,
                            GPTIMER_CompareMatchCallbackFxn compMatchCbFxn,
                            GPTIMER_CaptureCallbackFxn captureCbFxn);

/**
 * \brief Get IRQ status
 *
 * \param handle            [IN] #GPTIMER_Handle returned from GPTIMER_open()
 *
 */
uint32_t GPTIMER_getIRQStatus(GPTIMER_Handle handle);

/**
 * \brief Clear IRQ status bit
 *
 * \param handle            [IN] #GPTIMER_Handle returned from GPTIMER_open()
 * \param irqMask           [IN] IRQ Mask \ref GptimerIrqMask
 *
 */
void GPTIMER_clearIRQStatus(GPTIMER_Handle handle, uint32_t irqMask);

/**
 * \brief Enable IRQ Status, Corresponding status bit will be set in case of
 *        an event. Interrupt and callback will be called if interrupt is
 *        registered.
 *
 * \param handle            [IN] #GPTIMER_Handle returned from GPTIMER_open()
 * \param irqMask           [IN] IRQ Mask \ref GptimerIrqMask
 *
 */
void GPTIMER_enableInterruptStatus(GPTIMER_Handle handle, uint32_t irqMask);

/**
 * \brief Disable IRQ Status.
 *
 * \param handle            [IN] #GPTIMER_Handle returned from GPTIMER_open()
 * \param irqMask           [IN] IRQ Mask \ref GptimerIrqMask
 *
 */
void GPTIMER_disableInterruptStatus(GPTIMER_Handle handle, uint32_t irqMask);

#ifdef __cplusplus
}
#endif
#endif /* #ifndef GP_TIMER_H_ */
/** @} */
