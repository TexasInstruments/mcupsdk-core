
#include <stdio.h>
//! [include]
#include <drivers/gp_timer.h>
//! [include]

#define CONFIG_GPTIMER0     (0U)
extern GPTIMER_Object       gGpTimerObjects[];
GPTIMER_Handle              gGpTimerHandle;

GPTIMER_Compare_Config      compareConfig;
GPTIMER_Capture_Config      inputCaptureConfig;
GPTIMER_PWM_Config          pwmConfig;

/* Semaphores to be posted in callbacks */
SemaphoreP_Object overflowSemObj;
SemaphoreP_Object compareMatchSemObj;
SemaphoreP_Object captureSemObj;

void open(void)
{
//! [open]
    GPTIMER_Params         params;

    /* Initialize parameters */
    GPTIMER_Params_init(&params);

    params.enablePrescaler = false,
    params.cntPrescaler = 0,
    params.oneShotMode = false,
    params.cntReloadVal = 0,
    params.overflowMaskCount = 0,
    params.counterInitVal = 0,

    /* Update object Parameters mode settings */
    gGpTimerObjects[CONFIG_GPTIMER0].timerConfigMode = GPTIMER_MODE_CONFIG_FREE_RUN;
    gGpTimerObjects[CONFIG_GPTIMER0].overflowCallbackFunction = NULL,
    gGpTimerObjects[CONFIG_GPTIMER0].compareMatchCallbackFunction = NULL,
    gGpTimerObjects[CONFIG_GPTIMER0].captureCallbackFunction = NULL,

    gGpTimerHandle = GPTIMER_open(CONFIG_GPTIMER0, &params);
    DebugP_assert(gGpTimerHandle != NULL);
//! [open]
}

void close(void)
{
//! [close]
    GPTIMER_close(gGpTimerHandle);
//! [close]
}

//! [free_run_no_interrupt]

void gptimer_free_run_no_interrupt(void)
{

    uint32_t conifgMode;
    conifgMode = GPTIMER_MODE_CONFIG_FREE_RUN;

    GPTIMER_setTimerConfigMode(gGpTimerHandle, conifgMode,
                               (void *)NULL);
    /* Start the Timer */
    GPTIMER_start(gGpTimerHandle);

}

//! [free_run_no_interrupt]

//! [free_run_interrupt]

void overflowCallback(GPTIMER_Handle handle)
{
    /* Post Semaphore */
    SemaphoreP_post(&overflowSemObj);
}

void gptimer_free_run_interrupt(void)
{

    uint32_t conifgMode;
    conifgMode = GPTIMER_MODE_CONFIG_FREE_RUN;

    SemaphoreP_constructBinary(&overflowSemObj, 0);

    GPTIMER_setTimerConfigMode(gGpTimerHandle, conifgMode,
                               (void *)NULL);
    /* Add Callback */
    GPTIMER_setCallbackFxn(gGpTimerHandle, overflowCallback, NULL, NULL);
    /* Start the Timer */
    GPTIMER_start(gGpTimerHandle);
    /* Pend Semaphore */
    SemaphoreP_pend(&overflowSemObj, SystemP_WAIT_FOREVER);

    SemaphoreP_destruct(&overflowSemObj);

}

//! [free_run_interrupt]


//! [output_compare_interrupt]

void compareMatchCallback(GPTIMER_Handle handle)
{
    /* Post Semaphore */
    SemaphoreP_post(&compareMatchSemObj);
}

void gptimer_output_compare_interrupt(void)
{
    SemaphoreP_constructBinary(&compareMatchSemObj, 0);

    uint32_t conifgMode;

    conifgMode = GPTIMER_MODE_CONFIG_OUTPUT_COMPARE;
    compareConfig.cntCompareValComp = (0x017D7840U);

    GPTIMER_setTimerConfigMode(gGpTimerHandle, conifgMode,
                               (void *)(&compareConfig));
    /* Add Callback */
    GPTIMER_setCallbackFxn(gGpTimerHandle, NULL, compareMatchCallback, NULL);
    /* Start the TImer */
    GPTIMER_start(gGpTimerHandle);
    /* Pend Semaphore */
    SemaphoreP_pend(&compareMatchSemObj, SystemP_WAIT_FOREVER);

    SemaphoreP_destruct(&compareMatchSemObj);
}

//! [output_compare_interrupt]




//! [input_capture_interrupt]

void CaptureCallbackUser(GPTIMER_Handle handle)
{
    /* Post Semaphore */
    SemaphoreP_post(&captureSemObj);
}

void gptimer_input_capture_interrupt(void)
{
    SemaphoreP_constructBinary(&captureSemObj, 0);

    uint32_t conifgMode;

    conifgMode = GPTIMER_MODE_CONFIG_INPUT_CAPTURE;
    inputCaptureConfig.captureMode = GPTIMER_INPUT_CAPTURE_MODE_SECOND;
    inputCaptureConfig.captureEventMode = GPTIMER_INPUT_CAPTURE_EVENT_EDGE;

    GPTIMER_setTimerConfigMode(gGpTimerHandle, conifgMode,
                              (void *)(&inputCaptureConfig));
    /* Add Callback */
    GPTIMER_setCallbackFxn(gGpTimerHandle, NULL, NULL, CaptureCallbackUser);
    /* Start the Timer */
    GPTIMER_start(gGpTimerHandle);
    /* Pend Semaphore */
    SemaphoreP_pend(&captureSemObj, SystemP_WAIT_FOREVER);

    SemaphoreP_destruct(&captureSemObj);
}

//! [input_capture_interrupt]

//! [pwm_gen]

void gptimer_pwm_gen(void)
{

    uint32_t conifgMode;

    conifgMode = GPTIMER_MODE_CONFIG_PWM_GEN;

    pwmConfig.trigOutputPWMMode = GPTIMER_PWM_OUT_OVERFLOW_MATCH_TRIGGER,
    pwmConfig.defaultPWMOutSetting = GPTIMER_PWM_OUT_PIN_DEFAULT_0,
    pwmConfig.cntCompareValPWM = 4294954795,
    pwmConfig.outputModulationType = GPTIMER_PWM_OUT_PIN_MODULATION_TOGGLE,

    GPTIMER_setTimerConfigMode(gGpTimerHandle, conifgMode,
                              (void *)(&pwmConfig));

    /* Start the Timer */
    GPTIMER_start(gGpTimerHandle);

}

//! [pwm_gen]

