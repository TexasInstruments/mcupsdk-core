/*
 * VTM Example Application
 *
 * Voltage and Thermal Monitor (VTM) Example Application
 *
 *  Copyright (c) 2024-2025 Texas Instruments Incorporated
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
 *  \file main.c
 *
 *  \brief This file contains functions that provide main function
 *         for the Voltage and Thermal Monitor (VTM) application.
 */

#include <stdio.h>
#include <kernel/dpl/ClockP.h>
#include <sdl/sdl_esm.h>
#include <stdint.h>
#include <stdbool.h>
#include <sdl/include/sdl_types.h>
#include <dpl_interface.h>
#include <sdl/esm/v2/v2_0/sdl_esm_priv.h>
#include <sdl/sdl_vtm.h>
#include <sdl/dpl/sdl_dpl.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

SDL_VTM_configTs SDL_VTM_configTempSense =
{
    0U,     /* TS0 Shut*/
    0U,     /* TS0 Alert*/
    72000,//58000,  /* TS0 Hot*/
    8000,//52000,  /* TS0 Cold*/
    58000,  /* TSA0 Alert Hot*/
    52000,  /* TS0 Alert Cold*/
    0U,
    0U,
    58000,
    52000,
    58000,
    52000
};

volatile uint8_t SDL_tempExceedHot=0U;
volatile uint8_t SDL_tempBelowCold=0U;
volatile uint8_t SDL_tempLowThresholdIntr=0U;
volatile uint8_t SDL_vtmEsmError = 0U;

int32_t VTM_clear(void)
{
    int32_t alert_th_hot, alert_th_cold;
    int32_t retValue;
    uint32_t temp0;

    /* Reverse the condition so that ESM is not generated. */
    retValue = SDL_VTM_getTemp(SDL_VTM_INSTANCE_TS_0, &temp0);
    if(retValue == 0)
    {
        alert_th_hot =   temp0 + 3000;
        alert_th_cold =  temp0 - 3000;
        SDL_VTM_setTShutTemp(SDL_VTM_INSTANCE_TS_0, alert_th_hot, alert_th_cold);
    }

    SDL_vtmEsmError = 1U;

    return retValue;
}


void SDL_TS_hiInterruptHandler()
{
    SDL_VTM_Stat_val SDL_VTM_Stat_value;
    SDL_VTM_getSensorStatus(&SDL_VTM_Stat_value);

       if(SDL_VTM_Stat_value.s0HotEvent == 1U)
       {
           SDL_VTM_setClearInterrupts(SDL_VTM_INSTANCE_TS_0, SDL_VTM_MASK_HOT, 0, SDL_VTM_MASK_LOW_TH);
           SDL_tempExceedHot = 1U;
       }

       if(SDL_VTM_Stat_value.s0ColdEvent == 1U)
       {
           SDL_VTM_setClearInterrupts(SDL_VTM_INSTANCE_TS_0, 0, 0, 0);
           SDL_tempBelowCold = 1U;
           SDL_DPL_disableInterrupt(SDL_R5FSS0_CORE0_INTR_TSENSE_H);
       }

       if(SDL_VTM_Stat_value.s1HotEvent == 1U)
       {
           SDL_VTM_setClearInterrupts(SDL_VTM_INSTANCE_TS_1, SDL_VTM_MASK_HOT, 0, SDL_VTM_MASK_LOW_TH);
       }

       if(SDL_VTM_Stat_value.s1ColdEvent == 1U)
       {
           SDL_VTM_setClearInterrupts(SDL_VTM_INSTANCE_TS_1, 0, SDL_VTM_MASK_COLD, SDL_VTM_MASK_LOW_TH);
       }
}

void SDL_TS_loInterruptHandler()
{
    SDL_VTM_Stat_val SDL_VTM_Stat_value;
    SDL_VTM_getSensorStatus(&SDL_VTM_Stat_value);
    if(SDL_VTM_Stat_value.s0LowThresholdEvent == 1U)
    {
        SDL_VTM_setClearInterrupts(SDL_VTM_INSTANCE_TS_0, SDL_VTM_MASK_HOT, SDL_VTM_MASK_COLD, SDL_VTM_MASK_LOW_TH);
        SDL_tempLowThresholdIntr = 1U;
    }
    SDL_DPL_disableInterrupt(SDL_R5FSS0_CORE0_INTR_TSENSE_L);
}
/*****************************************************************************
 * This is the main function for the Voltage and Thermal Monitor (VTM) example
 * application.
 * It runs through 2 test cases to demonstrate usage of the VTM modules
 * for receiving errors and controlling the error pin.
 */
int32_t VTM_test(void)
{
    int32_t retValue;
    uint32_t temp0, temp1, temp2, temp3;
    int32_t alert_th_hot, alert_th_cold;
    SDL_VTM_Stat_val SDL_VTM_Stat_value;
    void (*pTSenseHInterruptHandler)(void *);
    void  (*pTSenseLInterruptHandler)(void *);
    SDL_DPL_HwipParams intrParams;
    pSDL_DPL_HwipHandle SDL_TS_HiHwiPHandle, SDL_TS_LoHwiPHandle;

    /* Register Interrupt Handlers */
    pTSenseHInterruptHandler = &SDL_TS_hiInterruptHandler;
    pTSenseLInterruptHandler = &SDL_TS_loInterruptHandler;

    intrParams.intNum      = SDL_R5FSS0_CORE0_INTR_TSENSE_H;
    intrParams.callback    = (*pTSenseHInterruptHandler);
    SDL_DPL_registerInterrupt(&intrParams, &SDL_TS_HiHwiPHandle);

    intrParams.intNum      = SDL_R5FSS0_CORE0_INTR_TSENSE_L;
    intrParams.callback    = (*pTSenseLInterruptHandler);
    SDL_DPL_registerInterrupt(&intrParams, &SDL_TS_LoHwiPHandle);

    /* Enable Interrupts */
    SDL_DPL_enableInterrupt(SDL_R5FSS0_CORE0_INTR_TSENSE_H);
    SDL_DPL_enableInterrupt(SDL_R5FSS0_CORE0_INTR_TSENSE_L);

    /* UC1 - Receive Hot and Cold Interrupt. */
    SDL_VTM_setClearInterrupts(SDL_VTM_INSTANCE_TS_0, 0, SDL_VTM_MASK_COLD, SDL_VTM_MASK_LOW_TH);
    
    retValue = SDL_VTM_initTs(&SDL_VTM_configTempSense);
    SDL_VTM_enableTs(SDL_VTM_SENSOR_SEL0, 0);
    SDL_VTM_enableTc();
    ClockP_usleep(9000);
    SDL_VTM_getTemp(SDL_VTM_INSTANCE_TS_0, &temp0);
    SDL_VTM_getTemp(SDL_VTM_INSTANCE_TS_1, &temp1);
    SDL_VTM_getTemp(SDL_VTM_INSTANCE_TS_2, &temp2);
    SDL_VTM_getTemp(SDL_VTM_INSTANCE_TS_3, &temp3);

    alert_th_hot = temp0 - 3000; // temperatube in mc 
    alert_th_cold = temp0 + 3000; // temperatube in mc
    /* Device temperature is 0x40 => 70000mc */
    SDL_VTM_setAlertTemp(SDL_VTM_INSTANCE_TS_0, alert_th_hot, alert_th_cold);

    SDL_VTM_getSensorStatus(&SDL_VTM_Stat_value);

    while(SDL_tempExceedHot != 1U);
    SDL_VTM_getTemp(SDL_VTM_INSTANCE_TS_0, &temp0);
    
    while(SDL_tempBelowCold != 1U);
    
    ClockP_usleep(9000);
    /* UC2 - Receive Low threshold Breach Interrupt and  Hot Interrupt. */
    SDL_VTM_setClearInterrupts(SDL_VTM_INSTANCE_TS_0, SDL_VTM_MASK_HOT, SDL_VTM_MASK_COLD, 0);
    ClockP_usleep(9000);
    retValue = SDL_VTM_initTs(&SDL_VTM_configTempSense);
    ClockP_usleep(9000);
    SDL_VTM_enableTs(SDL_VTM_SENSOR_SEL0, 0);
    ClockP_usleep(9000);
    SDL_VTM_enableTc();
    ClockP_usleep(9000);
    SDL_VTM_getTemp(SDL_VTM_INSTANCE_TS_0, &temp0);
    ClockP_usleep(9000);
    alert_th_hot = 0; // temperatube in mc
    alert_th_cold = temp0 - 3000; // temperatube in mc
    /* Device temperature is 0x40 => 70000mc */
    /* Configure cold alert temperature so that low threshold interrupt is generated. */
    SDL_VTM_setAlertTemp(SDL_VTM_INSTANCE_TS_0, alert_th_hot, alert_th_cold);

    while(SDL_tempLowThresholdIntr != 1U);
    SDL_VTM_getTemp(SDL_VTM_INSTANCE_TS_0, &temp0);
    SDL_VTM_disableTc();
    alert_th_hot = temp0 - 3000; // temperatube in mc
    alert_th_cold = temp0 + 3000; // temperatube in mc
    /* Device temperature is 0x40 => 70000mc */
    /* Configure hot alert temperature so that hot interrupt is generated. */
    SDL_VTM_setAlertTemp(SDL_VTM_INSTANCE_TS_0, alert_th_hot, alert_th_cold);
    SDL_VTM_setClearInterrupts(SDL_VTM_INSTANCE_TS_0, 0, SDL_VTM_MASK_COLD, SDL_VTM_MASK_LOW_TH);
    SDL_VTM_enableTc();

    while(SDL_tempExceedHot != 1U);

    ClockP_usleep(9000);
    /* UC3 - ESM Interrupt and Warm Reset Generation. */
    SDL_VTM_setClearInterrupts(SDL_VTM_INSTANCE_TS_0, SDL_VTM_MASK_HOT, SDL_VTM_MASK_COLD, SDL_VTM_MASK_LOW_TH);
    alert_th_hot = temp0 - 3000; // temperatube in mc
    alert_th_cold = temp0 + 3000; // temperatube in mc
    retValue = SDL_VTM_setTShutTemp(SDL_VTM_INSTANCE_TS_0, alert_th_hot, alert_th_cold);
    retValue = SDL_VTM_initTs(&SDL_VTM_configTempSense);
    SDL_VTM_enableTs(SDL_VTM_SENSOR_SEL0, 0);
    SDL_VTM_enableTc();
    SDL_VTM_getTemp(SDL_VTM_INSTANCE_TS_0, &temp0);

    while(SDL_vtmEsmError != 1U);
    SDL_VTM_getTemp(SDL_VTM_INSTANCE_TS_0, &temp0);

    return retValue;
}

/* Nothing past this point */