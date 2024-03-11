/******************************************************************************
 * Copyright (c) 2024 Texas Instruments Incorporated - http://www.ti.com
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
 *
 *****************************************************************************/

/**
 *   @file    pmic_low_iq_timer.h
 *
 *   @brief   This file contains the default MACRO's and function definitions
 * for PMIC Low IQ Timer state configuration
 *
 */

#ifndef PMIC_PMIC_LOW_IQ_TIMER_H_
#define PMIC_PMIC_LOW_IQ_TIMER_H_

/* ==========================================================================*/
/*                             Include Files                                 */
/* ==========================================================================*/

#include "pmic_core.h"

#include "pmic_core_priv.h"

#include "pmic_io_priv.h"

#include "pmic_types.h"

/* ========================================================================== */
/*                          Function Prototypess */
/* ========================================================================== */

int32_t Pmic_SetTimerConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t tmrData);
int32_t Pmic_GetTimerConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t * tmrConfigData);
int32_t Pmic_SetTimerPrescale(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t tmrPSData);
int32_t Pmic_GetTimerPrescale(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t * tmrPSData);
int32_t Pmic_TimerClear(Pmic_CoreHandle_t * pPmicCoreHandle);
int32_t Pmic_SetTimerCounter0(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t tmrcntData0);
int32_t Pmic_SetTimerCounter1(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t tmrcntData1);
int32_t Pmic_SetTimerCounter2(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t tmrcntData2);
int32_t Pmic_GetTimerCounter0(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t * tmrcntData0);
int32_t Pmic_GetTimerCounter1(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t * tmrcntData1);
int32_t Pmic_GetTimerCounter2(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t * tmrcntData2);
int32_t Pmic_GetLPWake0(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t * tmrlpwakeData);
int32_t Pmic_GetLPWake1(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t * tmrlpwakeData);
int32_t Pmic_GetLPWake2(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t * tmrlpwakeData);
int32_t Pmic_SetLPWake0(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t tmrlpwakeData);
int32_t Pmic_SetLPWake1(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t tmrlpwakeData);
int32_t Pmic_SetLPWake2(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t tmrlpwakeData);

#endif /* PMIC_PMIC_LOW_IQ_TIMER_H_ */