/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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

#include <kernel/safertos/dpl/common/ClockP_safertos_priv.h>
#include <kernel/dpl/TimerP.h>

/* Converts a time in milliseconds to a time in ticks.  This macro can be
 * overridden by a macro of the same name defined in FreeRTOSConfig.h in case the
 * definition here is not suitable for your application. */
#ifndef pdMS_TO_TICKS
    #define pdMS_TO_TICKS( xTimeInMs )    ( ( portTickType ) ( ( ( portTickType ) ( xTimeInMs ) * ( portTickType ) configTICK_RATE_HZ ) / ( portTickType ) 1000U ) )
#endif

void ClockP_timerClearOverflowInt(uint32_t timerBaseAddr)
{
    TimerP_clearOverflowInt(timerBaseAddr);
}

uint32_t ClockP_getTimerCount(uint32_t timerBaseAddr)
{
    return TimerP_getCount(timerBaseAddr);
}

void ClockP_init(void)
{
    /* These MUST not be 0 */
    DebugP_assert( gClockConfig.timerInputPreScaler != 0);
    DebugP_assert( gClockConfig.timerInputClkHz != 0);
    DebugP_assert( gClockConfig.usecPerTick != 0);
    DebugP_assert( gClockConfig.timerBaseAddr != 0);

    /* init internal data structure */
    gClockCtrl.ticks = 0;
    gClockCtrl.usecPerTick = gClockConfig.usecPerTick;
    gClockCtrl.timerBaseAddr = gClockConfig.timerBaseAddr;
}

/* Tick timer setup using TI MCU+ timers and interrupts. */
void vApplicationSetupTickInterruptHook( portUInt32Type ulTimerClockHz,
                                         portUInt32Type ulTickRateHz )
{
    TimerP_Params xTimerParams;
    HwiP_Params timerHwiParams;
    HwiP_Object timerHwiObj;

    TimerP_Params_init( &xTimerParams );
    xTimerParams.inputClkHz = ulTimerClockHz;
    xTimerParams.periodInUsec = ( 1000000UL / ulTickRateHz );
    TimerP_setup( gClockCtrl.timerBaseAddr, &xTimerParams );

    /* setup ISR and enable it */
    HwiP_Params_init( &timerHwiParams );
    timerHwiParams.intNum = gClockConfig.timerHwiIntNum;
    timerHwiParams.callback = ClockP_timerTickIsr;
    HwiP_construct( &timerHwiObj, &timerHwiParams);

    /* start the tick timer */
    TimerP_start( gClockCtrl.timerBaseAddr );
}
