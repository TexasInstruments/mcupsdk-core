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

/* RTI timer implementation for clock tick */

#define RTI_RTIGCTRL                                                (0x0U)
#define RTI_RTICOMPCTRL                                             (0xcU)
#define RTI_RTIFRC0                                                 (0x10U)
#define RTI_RTIUC0                                                  (0x14U)
#define RTI_RTICPUC0                                                (0x18U)
#define RTI_RTICOMP0                                                (0x50U)
#define RTI_RTIUDCP0                                                (0x54U)
#define RTI_RTISETINT                                               (0x80U)
#define RTI_RTICLEARINT                                             (0x84U)
#define RTI_RTIINTFLAG                                              (0x88U)

void TimerP_Params_init(TimerP_Params *params)
{
    params->inputPreScaler = 1; /* NOT USED */
    params->inputClkHz = 25*1000000;
    params->periodInUsec = 1000;
    params->periodInNsec = 0;
    params->oneshotMode = 0; /* NOT USED, always periodic timer */
    params->enableOverflowInt = 1;
    params->enableDmaTrigger = 0;
}

void TimerP_setup(uint32_t baseAddr, TimerP_Params *params)
{
    volatile uint32_t *addr;
    uint32_t reloadVal;
    uint64_t timeInNsec, timerCycles;

    DebugP_assert( baseAddr!=0);
    /* pre scaler MUST be 1 for RTI when used as tick timer */
    DebugP_assert( params->inputPreScaler == 1);
    DebugP_assert( params->inputClkHz != 0);
    DebugP_assert( params->periodInUsec != 0 || params->periodInNsec != 0 );

    /* stop timer and clear pending interrupts */
    TimerP_stop(baseAddr);
    TimerP_clearOverflowInt(baseAddr);

    timeInNsec = (uint64_t)params->periodInNsec;
    if(timeInNsec == 0)
    {
        timeInNsec = params->periodInUsec*1000U;
    }

    timerCycles =  ( (uint64_t)params->inputClkHz * timeInNsec ) / 1000000000U;

    /* if timerCycles > 32b then we cannot give accurate timing */
    DebugP_assert( timerCycles <= 0xFFFFFFFFU );

    /* calculate count and reload value register value */
    reloadVal = timerCycles;

    /* reset up counter (UC) value  0 */
    addr = (volatile uint32_t *)(baseAddr + RTI_RTIUC0);
    *addr = 0;

    /* reset free running counter (FRC) to 0 */
    addr = (volatile uint32_t *)(baseAddr + RTI_RTIFRC0);
    *addr = 0;

    /* set reload value in compare upcounter, when this value is reach FRC is incremented by 1 */
    addr = (volatile uint32_t *)(baseAddr + RTI_RTICPUC0);
    *addr = reloadVal;

    /* Program compare 0 (COMP0) to trigger interrupt when FRC increments by 1 */
    addr = (volatile uint32_t *)(baseAddr + RTI_RTICOMP0);
    *addr = 1;

    /* Add 1 to COMP0 so we will again trigger a interrupt when each time FRC increments by 1 */
    addr = (volatile uint32_t *)(baseAddr + RTI_RTIUDCP0);
    *addr = 1;

    /* Select counter block 0 */
    addr = (volatile uint32_t *)(baseAddr + RTI_RTICOMPCTRL);
    *addr = 0;

    /* enable/disable interrupts */
    if(params->enableOverflowInt)
    {
        /* enable interrupt */
        addr = (volatile uint32_t *)(baseAddr + RTI_RTISETINT);
        *addr = (0x1);
    }
    else
    {
        /* disable interrupt */
        addr = (volatile uint32_t *)(baseAddr + RTI_RTICLEARINT);
        *addr = (0x1);
    }

    /* enable/disable interrupts */
    if(params->enableDmaTrigger)
    {
        /* enable interrupt */
        addr = (volatile uint32_t *)(baseAddr + RTI_RTISETINT);
        *addr = (0x1 << 8);
    }
    else
    {
        /* disable interrupt */
        addr = (volatile uint32_t *)(baseAddr + RTI_RTICLEARINT);
        *addr = (0x1 << 8);;
    }
}

void TimerP_start(uint32_t baseAddr)
{
    volatile uint32_t *addr = (uint32_t *)(baseAddr + RTI_RTIGCTRL);

    /* start timer */
    *addr |= (0x1 << 0);
}

void TimerP_stop(uint32_t baseAddr)
{
    volatile uint32_t *addr = (volatile uint32_t *)(baseAddr + RTI_RTIGCTRL);

    /* stop timer */
    *addr &= ~(0x1 << 0);
}

uint32_t TimerP_getCount(uint32_t baseAddr)
{
    volatile uint32_t *uc_addr = (volatile uint32_t *)(baseAddr + RTI_RTIUC0);
    volatile uint32_t *frc_addr = (volatile uint32_t *)(baseAddr + RTI_RTIFRC0);
    volatile uint32_t *cpuc_addr = (volatile uint32_t *)(baseAddr + RTI_RTICPUC0);
    volatile uint32_t frc;

    /* RTI_RTIUC0 will be only updated by a previous read of free running counter 0 (RTIFRC0). This
     * method of updating effectively gives a 64-bit read of both counters, without having the problem
     * of a counter being updated between two consecutive reads on up counter 0 (RTIUC0) and free
     * running counter 0 (RTIFRC0).
     * A read of this counter returns the value of the counter at the time RTIFRC0 was read.
     */

    frc = *frc_addr;
    frc++; /* dummy increment so that compipler does not optimized this out */

    /* return 0xFFFFFFFF - value, since ClockP assumes in this format to calculate current time */
    return 0xFFFFFFFFu - (*cpuc_addr - *uc_addr) - 1UL;
}

uint32_t TimerP_getReloadCount(uint32_t baseAddr)
{
    volatile uint32_t *cpuc_addr = (volatile uint32_t *)(baseAddr + RTI_RTICPUC0);

    /* return 0xFFFFFFFF - value, since ClockP assumes in this format to calculate current time */
    return 0xFFFFFFFFu - (*cpuc_addr) - 1UL;
}

void TimerP_clearOverflowInt(uint32_t baseAddr)
{
    volatile uint32_t *addr;

    /* clear status for interrupt */
    addr = (volatile uint32_t *)(baseAddr + RTI_RTIINTFLAG);
    *addr = (0x1);

}

uint32_t TimerP_isOverflowed(uint32_t baseAddr)
{
    uint32_t val;

    /* get status for interrupt */
    val = *(volatile uint32_t *)(baseAddr + RTI_RTIINTFLAG);

    return ((val) & 0x1);
}
