/*
 *  Copyright (C) 2018-2023 Texas Instruments Incorporated
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
#include "SysTickTimerP.h"
#include <drivers/hw_include/cslr.h>

/* SysTick timer implementation for clock tick */
#define SYST_BASE   (0xE000E010u)
#define SYST_CSR    (volatile uint32_t *)((SYST_BASE) + 0x00u)
#define SYST_RVR    (volatile uint32_t *)((SYST_BASE) + 0x04u)
#define SYST_CVR    (volatile uint32_t *)((SYST_BASE) + 0x08u)

void SysTickTimerP_Params_init(TimerP_Params *params)
{
    params->inputPreScaler = 1; /* NOT USED */
    params->inputClkHz = 200*1000000;
    params->periodInUsec = 1000;
    params->periodInNsec = 0;
    params->oneshotMode = 0;
    params->enableOverflowInt = 1;
}

void SysTickTimerP_setup(TimerP_Params *params)
{
    volatile uint32_t *addr;
    uint32_t ctrlVal;
    uint32_t countVal, reloadVal;
    uint64_t timeInNsec, timerCycles;

    /* There is no pre-scaler support for SysTick and its value is ignored */
    DebugP_assert( params->inputClkHz != 0U);
    DebugP_assert( params->periodInUsec != 0U);
    /* usec period MUST divide 1sec in integer units */
    DebugP_assert( (1000000u % params->periodInUsec) == 0U );

    /* stop timer and clear pending interrupts */
    SysTickTimerP_stop();

    timeInNsec = (uint64_t)params->periodInNsec;
    if(timeInNsec == 0U)
    {
        timeInNsec = (uint64_t)params->periodInUsec*1000U;
    }

    timerCycles =  ( (uint64_t)params->inputClkHz * timeInNsec ) / 1000000000U;

    /* if timerCycles > 32b then we cannot give accurate timing */
    DebugP_assert( timerCycles <= 0xFFFFFFFFU );

    /* calculate count and reload value register value */
    /* For generating a mutishot timer with period of N processor cycles,
       a reload count of N-1 is used */
    countVal = timerCycles - 1U;

    /* keep reload value as 0, later if is auto-reload is enabled, it will be set a value > 0 */
    reloadVal = 0;

    /* calculate control register value, keep timer disabled */
    ctrlVal = 0;
    /* select clock source as CPU clock */
    ctrlVal |= (1u << 2u);
    /* enable/disable interrupts */
    if((params->enableOverflowInt) != 0U)
    {
        /* enable interrupt */
        ctrlVal |= (1u << 1u);
    }

    if(params->oneshotMode==0U)
    {
        /* autoreload timer */
        reloadVal = countVal;
    }

    /* set timer control value */
    addr = SYST_CSR;
    *addr = ctrlVal;

    /* set reload value */
    addr = SYST_RVR;
    *addr = reloadVal;

    /* set count value */
    addr = SYST_CVR;
    *addr = countVal;

}

/* base address not used since, address is fixed for SysTick in M4F */
void SysTickTimerP_start(void)
{
    volatile uint32_t *addr = SYST_CSR;

    /* start timer */
    *addr |= ((uint32_t)0x1 << 0);
}

/* base address not used since, address is fixed for SysTick in M4F */
void SysTickTimerP_stop(void)
{
    volatile uint32_t *addr = SYST_CSR;

    /* stop timer */
    *addr &= ~(0x1U << 0);
}

/* base address not used since, address is fixed for SysTick in M4F */
uint32_t SysTickTimerP_getCount(void)
{
    /* return 0xFFFFFFFF - value, since ClockP assumes in this format to calculate current time */
    return (0xFFFFFFFFu - CSL_REG32_RD(SYST_CVR));

}

/* base address not used since, address is fixed for SysTick in M4F */
uint32_t SysTickTimerP_getReloadCount(void)
{
    /* return 0xFFFFFFFF - value, since ClockP assumes in this format to calculate current time */
    return (0xFFFFFFFFu - CSL_REG32_RD(SYST_RVR));
}

/* base address not used since, address is fixed for SysTick in M4F */
uint32_t SysTickTimerP_isOverflowed(void)
{
    volatile uint32_t *addr = SYST_CSR;

    return ((*addr >> 16) & 0x1U);
}