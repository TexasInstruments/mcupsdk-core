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

#include <kernel/dpl/TimerP.h>

/* GP timer implementation for clock tick */

#define TIMER_IRQ_EOI           (0x20u)
#define TIMER_IRQ_STATUS_RAW    (0x24u)
#define TIMER_IRQ_STATUS        (0x28u)
#define TIMER_IRQ_INT_ENABLE    (0x2Cu)
#define TIMER_IRQ_INT_DISABLE   (0x30u)
#define TIMER_TCLR              (0x38u)
#define TIMER_TCRR              (0x3cu)
#define TIMER_TLDR              (0x40u)

#define TIMER_OVF_INT_SHIFT     (0x1)

void TimerP_Params_init(TimerP_Params *params)
{
    params->inputPreScaler = 1;
    params->inputClkHz = 25*1000000;
    params->periodInUsec = 1000;
    params->periodInNsec = 0; /* periodInUsec is used when periodInNsec is 0 */
    params->oneshotMode = 0;
    params->enableOverflowInt = 1;
    params->enableDmaTrigger = 0; /* NOT USED */
}

void TimerP_setup(uint32_t baseAddr, TimerP_Params *params)
{
    volatile uint32_t *addr;
    uint32_t ctrlVal;
    uint32_t countVal, reloadVal;
    uint64_t timeInNsec, inputClkHz, timerCycles;

    DebugP_assert( baseAddr!=0);
    DebugP_assert( params->inputPreScaler != 0);
    DebugP_assert( params->inputClkHz != 0);
    DebugP_assert( params->periodInUsec != 0 || params->periodInNsec != 0 );
    /* pre scaler MUST be <= 256 */
    DebugP_assert( params->inputPreScaler <= 256);
    /* pre scaler MUST divide input clock in integer units */
    DebugP_assert( (params->inputClkHz % params->inputPreScaler) == 0);

    /* stop timer and clear pending interrupts */
    TimerP_stop(baseAddr);
    TimerP_clearOverflowInt(baseAddr);

    timeInNsec = (uint64_t)params->periodInNsec;
    if(timeInNsec == 0)
    {
        timeInNsec = params->periodInUsec*1000U;
    }

    inputClkHz = params->inputClkHz / params->inputPreScaler;
    timerCycles =  ( inputClkHz * timeInNsec ) / 1000000000U;

    /* if timerCycles > 32b then we cannot give accurate timing */
    DebugP_assert( timerCycles < 0xFFFFFFFFU );

    /* calculate count and reload value register value */
    countVal = 0xFFFFFFFFu - timerCycles - 1;

    /* keep reload value as 0, later if is auto-reload is enabled, it will be set a value > 0 */
    reloadVal = 0;

    /* calculate control register value, keep timer disabled */
    ctrlVal = 0;
    if(params->inputPreScaler>1)
    {
        uint32_t preScaleVal;

        for(preScaleVal=8; preScaleVal>=1; preScaleVal--)
        {
            if( (params->inputPreScaler & (0x1 << preScaleVal)) != 0 )
                break;
        }

        /* enable pre-scaler */
        ctrlVal |= (0x1 << 5);
        /* set pre-scaler value */
        ctrlVal |= ( ((preScaleVal - 1) & 0x7) << 2);
    }
    if(params->oneshotMode==0)
    {
        /* autoreload timer */
        ctrlVal |= (0x1 << 1);
        reloadVal = countVal;
    }

    /* set timer control value */
    addr = (volatile uint32_t *)(baseAddr + TIMER_TCLR);
    *addr = ctrlVal;

    /* set timer count value */
    addr = (volatile uint32_t *)(baseAddr + TIMER_TCRR);
    *addr = countVal;

    /* set reload value */
    addr = (volatile uint32_t *)(baseAddr + TIMER_TLDR);
    *addr = reloadVal;

    /* enable/disable interrupts */
    if(params->enableOverflowInt)
    {
        /* enable interrupt */
        addr = (volatile uint32_t *)(baseAddr + TIMER_IRQ_INT_ENABLE);
        *addr = (0x1 << TIMER_OVF_INT_SHIFT);
    }
    else
    {
        /* disable interrupt */
        addr = (volatile uint32_t *)(baseAddr + TIMER_IRQ_INT_DISABLE);
        *addr = (0x1 << TIMER_OVF_INT_SHIFT);
    }
}

void TimerP_start(uint32_t baseAddr)
{
    volatile uint32_t *addr = (uint32_t *)(baseAddr + TIMER_TCLR);

    /* start timer */
    *addr |= (0x1 << 0);
}

void TimerP_stop(uint32_t baseAddr)
{
    volatile uint32_t *addr = (volatile uint32_t *)(baseAddr + TIMER_TCLR);

    /* stop timer */
    *addr &= ~(0x1 << 0);
}

uint32_t TimerP_getCount(uint32_t baseAddr)
{
    volatile uint32_t *addr = (volatile uint32_t *)(baseAddr + TIMER_TCRR);

    return *addr;
}

uint32_t TimerP_getReloadCount(uint32_t baseAddr)
{
    volatile uint32_t *addr = (volatile uint32_t *)(baseAddr + TIMER_TLDR);

    return *addr;
}

void TimerP_clearOverflowInt(uint32_t baseAddr)
{
    volatile uint32_t *addr;
    uint32_t value = (0x1 << TIMER_OVF_INT_SHIFT);

    /* clear status for overflow interrupt */
    addr = (volatile uint32_t *)(baseAddr + TIMER_IRQ_STATUS);
    *addr = value;

    /* [MCUSDK-177] read back and make sure interrupt was indeed cleared, if not clear it again
     */
    if(*addr & value)
        *addr = value;

    #if 0 /* should not be used for level interrupts */
    /* apply SW EOI */
    addr = (volatile uint32_t *)(baseAddr + TIMER_IRQ_EOI);
    *addr = 0;
    #endif

}

uint32_t TimerP_isOverflowed(uint32_t baseAddr)
{
    uint32_t val;

    /* get status for overflow interrupt */
    val = *(volatile uint32_t *)(baseAddr + TIMER_IRQ_STATUS_RAW);

    return ((val >> TIMER_OVF_INT_SHIFT) & 0x1);
}
