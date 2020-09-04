/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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

#include <stdint.h>
#include <math.h>
#include <drivers/epwm.h>
#include "epwm_drv_aux.h"
#include "epwm_dc.h"

/* Min / max output amplitude.
   Waveform amplitude values beyond these thresholds are saturated. */
#define AMP_MAX (  1.0f )
#define AMP_MIN ( -1.0f )

/* Compute Duty Cycle & CMPx given amplitude & EPWM period */
static void computeCmpx(
    float       amp,
    uint32_t    epwmPrdVal,
    float       *pEpwmDutyCycle,
    uint16_t    *pEpwmCmpVal
);

/* Initialize EPWM */
Epwm_Handle epwmInit(
    EPwmCfgPrms_t *pEpwmCfgPrms, 
    EPwmObj_t *pEpwmObj
)
{
    Epwm_Handle hEpwm;              /* EPWM handle */
    uint32_t epwmBaseAddr;          /* EPWM base address */
    uint32_t epwmTbFreq;            /* EPWM timebase clock */
    uint32_t epwmOutFreq;           /* EPWM output frequency */
    uint32_t epwmTbCounterDir;      /* EPWM TB counter direction */
    uint32_t epwmPrdVal;
    float epwmTbCntVal_f;
    uint32_t epwmTbCntVal;
    float epwmCmpAVal_f;
    uint32_t epwmCmpAVal;

    if ((pEpwmCfgPrms == NULL) || (pEpwmObj == NULL))
    {
        return NULL;
    }
    
    /* Get configuration parameters */
    epwmBaseAddr = pEpwmCfgPrms->epwmBaseAddr;   
    epwmOutFreq = pEpwmCfgPrms->epwmOutFreq;
    epwmTbCounterDir = pEpwmCfgPrms->epwmTbCounterDir;
    
    /* Configure Time Base submodule, clock dividers */
    writeTbClkDiv(epwmBaseAddr, pEpwmCfgPrms->hspClkDiv, pEpwmCfgPrms->clkDiv);

    /* Calculate TB frequency */
    if (pEpwmCfgPrms->hspClkDiv != 0)
    {
        epwmTbFreq = pEpwmCfgPrms->epwmFclkFreq / (2*pEpwmCfgPrms->hspClkDiv * (1<<pEpwmCfgPrms->clkDiv));
    }
    else
    {
        epwmTbFreq = pEpwmCfgPrms->epwmFclkFreq / (1 * (1<<pEpwmCfgPrms->clkDiv));
    }

    /* Configure Time Base Period, immediate load */
    tbPwmFreqCfg(epwmBaseAddr, epwmTbFreq, epwmOutFreq,
        epwmTbCounterDir, EPWM_SHADOW_REG_CTRL_DISABLE, &epwmPrdVal);
    /* Configure Time Base Period, shadow load mode */
    tbPwmFreqCfg(epwmBaseAddr, epwmTbFreq, epwmOutFreq,
        epwmTbCounterDir, EPWM_SHADOW_REG_CTRL_ENABLE, &epwmPrdVal);

    /* Configure TB Sync In Mode */
    if (pEpwmCfgPrms->cfgTbSyncIn == FALSE)
    {
        EPWM_tbSyncDisable(epwmBaseAddr);
    }
    else
    {
        epwmTbCntVal_f = epwmPrdVal * pEpwmCfgPrms->tbPhsValue;
        epwmTbCntVal = (uint32_t)roundf(epwmTbCntVal_f);
        EPWM_tbSyncEnable(epwmBaseAddr, epwmTbCntVal, pEpwmCfgPrms->tbSyncInCounterDir);
    }
    
    /* Configure TB Sync Out Mode */
    if (pEpwmCfgPrms->cfgTbSyncOut == FALSE)
    {
        EPWM_tbSetSyncOutMode(epwmBaseAddr, EPWM_TB_SYNC_OUT_EVT_DISABLE );
    }
    else
    {
        EPWM_tbSetSyncOutMode(epwmBaseAddr, pEpwmCfgPrms->tbSyncOutMode);
    }
    
    /* Configure emulation mode */
    EPWM_tbSetEmulationMode(epwmBaseAddr, EPWM_TB_EMU_MODE_FREE_RUN);

    /*
     *  Compute COMPA value - this determines the duty cycle
     *  COMPA = (PRD - ((DC * PRD) / 100)
     */
    epwmCmpAVal_f =  (float)epwmPrdVal;
    epwmCmpAVal_f -= (pEpwmCfgPrms->epwmDutyCycle * epwmPrdVal) / 100.0;
    epwmCmpAVal_f = roundf(epwmCmpAVal_f);
    epwmCmpAVal = (uint32_t)epwmCmpAVal_f;
    
    /* Configure counter compare submodule */
    EPWM_counterComparatorCfg(epwmBaseAddr, EPWM_CC_CMP_A,
        epwmCmpAVal, EPWM_SHADOW_REG_CTRL_ENABLE,
            EPWM_CC_CMP_LOAD_MODE_CNT_EQ_ZERO, TRUE);

    /* Configure Action Qualifier Submodule */
    EPWM_aqActionOnOutputCfg(epwmBaseAddr, EPWM_OUTPUT_CH_A, &pEpwmCfgPrms->aqCfg);

    /* Configure Dead Band Submodule */
    if (pEpwmCfgPrms->cfgDb == TRUE)
    {
        EPWM_deadbandCfg(epwmBaseAddr, &pEpwmCfgPrms->dbCfg);
    }
    else 
    {
        EPWM_deadbandBypass(epwmBaseAddr);
    }

    /* Configure Chopper Submodule */
    EPWM_chopperEnable(epwmBaseAddr, FALSE);

    /* Configure trip zone Submodule */
    EPWM_tzTripEventDisable(epwmBaseAddr, EPWM_TZ_EVENT_ONE_SHOT, 0U);
    EPWM_tzTripEventDisable(epwmBaseAddr, EPWM_TZ_EVENT_CYCLE_BY_CYCLE, 0U);

    /* Configure event trigger Submodule */
    if (pEpwmCfgPrms->cfgEt == TRUE)
    {
        EPWM_etIntrCfg(epwmBaseAddr, pEpwmCfgPrms->intSel, pEpwmCfgPrms->intPrd);
        EPWM_etIntrEnable(epwmBaseAddr);
    }

    /* Init PWM object */
    hEpwm = (Epwm_Handle)pEpwmObj;
    hEpwm->epwmCfgPrms = *pEpwmCfgPrms;
    hEpwm->epwmPrdVal = epwmPrdVal;
    
    return hEpwm;
}

/* Update EPWM outputs */
int32_t epwmUpdateOut(
    Epwm_Handle hEpwm,
    float amp
)
{
    float dcVal;        /* EPWM duty cycle value */
    uint16_t cmpVal;    /* EPWM CMP value */
    
    if (hEpwm == NULL)
    {
        return EPWM_DC_INV_PRMS;
    }
    
    /* Compute next Duty Cycle and CMP values */
    computeCmpx(amp, hEpwm->epwmPrdVal, &dcVal, &cmpVal);
        
    /* Write next CMPA value */
    writeCmpA(hEpwm->epwmCfgPrms.epwmBaseAddr, cmpVal);

    return EPWM_DC_SOK;
}

/* Compute Duty Cycle & CMPx given amplitude & EPWM period */
static void computeCmpx(
    float       amp,
    uint32_t    epwmPrdVal,
    float       *pEpwmDutyCycle,
    uint16_t    *pEpwmCmpVal
)
{
    float dc_f;
    float cmp_f;
    uint16_t cmp;

    if ((pEpwmDutyCycle == NULL) || (pEpwmCmpVal == NULL))
    {
        return;
    }

    if (amp >= AMP_MAX)
    {
        /* 100% duty cycle */
        dc_f = 1.0;
    }
    else if (amp <= AMP_MIN)
    {
        /* 0% duty cycle */
        dc_f = 0.0;
    }
    else
    {
        /* compute Duty Cycle */
        dc_f = 0.5*(amp + 1.0);
    }

    /* compute CMPx */
    cmp_f = (1.0 - dc_f)*epwmPrdVal; /* up-down count */
    cmp = (uint16_t)roundf(cmp_f);

    *pEpwmDutyCycle = dc_f;
    *pEpwmCmpVal = cmp;
}
