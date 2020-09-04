
#include <stdio.h>
#include <string.h>
//! [include]
#include <drivers/hwa.h>
//! [include]
#include <kernel/dpl/DebugP.h>

HWA_Handle              gHwaHandle;

void open(void)
{
//! [open]
    int32_t             status = SystemP_SUCCESS;
    HWA_CommonConfig    commonCfg;
    HWA_ParamConfig     paramCfg;
    uint32_t            paramIdx = 0U;  /* Can be any paramset as per need */

    /* Open HWA driver */
    gHwaHandle = HWA_open(0, NULL, &status);
    DebugP_assert(gHwaHandle == NULL);

    /* Init Common Params */
    memset(&commonCfg, 0, sizeof(commonCfg));
    commonCfg.configMask = HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG |
                            HWA_COMMONCONFIG_MASK_TWIDDITHERENABLE |
                            HWA_COMMONCONFIG_MASK_LFSRSEED;
    commonCfg.fftConfig.twidDitherEnable = 1U;
    commonCfg.fftConfig.lfsrSeed = 11U;
    commonCfg.paramStartIdx = paramIdx;
    commonCfg.paramStopIdx = paramIdx;
    commonCfg.numLoops = (commonCfg.paramStopIdx - commonCfg.paramStartIdx) + 1U;
    status = HWA_configCommon(gHwaHandle, &commonCfg);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Init param set */
    memset(&paramCfg , 0, sizeof(paramCfg));
    paramCfg.triggerMode = HWA_TRIG_MODE_DMA;
    paramCfg.accelMode = HWA_ACCELMODE_FFT;
    /* ... Add init all other parameters as per need */
    status = HWA_configParamSet(gHwaHandle, paramIdx, &paramCfg, NULL);
    DebugP_assert(status == SystemP_SUCCESS);
//! [open]
}

void close(void)
{
//! [close]
    int32_t status;

    status = HWA_close(gHwaHandle);
    DebugP_assert(status == SystemP_SUCCESS);
//! [close]
}

void process(void)
{
//! [process]
    int32_t     status;
    uint32_t    dmaCh = 0U; /* Can be any DMA channel as per need */

    /* Enable HWA */
    status = HWA_enable(gHwaHandle, 1U);
    DebugP_assert(status == SystemP_SUCCESS);
    status = HWA_reset(gHwaHandle);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Trigger and wait */
    status = HWA_setDMA2ACCManualTrig(gHwaHandle, dmaCh);
    DebugP_assert(status == SystemP_SUCCESS);
    /* Wait for Done Callback */

    /* Disable HWA */
    status = HWA_enable(gHwaHandle, 0U);
    DebugP_assert(status == SystemP_SUCCESS);
//! [process]
}
