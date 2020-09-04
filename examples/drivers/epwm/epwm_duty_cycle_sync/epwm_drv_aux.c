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
#include <drivers/epwm.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/csl_types.h>
#include "epwm_drv_aux.h"

#include <math.h>

/* Configure PWM Time base counter Frequency/Period */
void tbPwmFreqCfg(
    uint32_t baseAddr,
    uint32_t tbClk,
    uint32_t pwmFreq,
    uint32_t counterDir,
    uint32_t enableShadowWrite, 
    uint32_t *pPeriodCount
)
{
    uint32_t tbPeriodCount;
    float tbPeriodCount_f;
    uint32_t regVal = 0U;

    regVal = HW_RD_REG16(baseAddr + PWMSS_EPWM_TBCTL);
    HW_SET_FIELD32(regVal, PWMSS_EPWM_TBCTL_PRDLD, enableShadowWrite);
    HW_SET_FIELD32(regVal, PWMSS_EPWM_TBCTL_CTRMODE, counterDir);
    HW_WR_REG16((baseAddr + PWMSS_EPWM_TBCTL),
        (uint16_t)regVal);

    /* compute period using floating point */
    tbPeriodCount_f = (float)tbClk / pwmFreq;
    if (EPWM_TB_COUNTER_DIR_UP_DOWN == counterDir)
    {
        tbPeriodCount_f = tbPeriodCount_f / 2.0;
    }
    tbPeriodCount_f = roundf(tbPeriodCount_f);
    tbPeriodCount = (uint32_t)tbPeriodCount_f;

    regVal = (counterDir == EPWM_TB_COUNTER_DIR_UP_DOWN) ? 
        tbPeriodCount : tbPeriodCount-1;
    HW_WR_REG16((baseAddr + PWMSS_EPWM_TBPRD),
        (uint16_t)regVal);
        
    *pPeriodCount = tbPeriodCount;
}
