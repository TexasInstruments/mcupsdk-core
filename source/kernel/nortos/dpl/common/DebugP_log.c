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

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/HwiP.h>

/* This is needed in r5f since assert can be called before MPU init */
#if defined(__ARM_ARCH_7R__)
#define BOOT_SECTION __attribute__((section(".text.boot")))
#else
#define BOOT_SECTION
#endif

volatile uint32_t gDebugLogZone = DebugP_LOG_ZONE_ALWAYS_ON;

uint32_t DebugP_logZoneEnable(uint32_t logZoneMask)
{
    uint32_t oldZoneMask;
    uintptr_t oldIntState;

    oldIntState = HwiP_disable();

    oldZoneMask = gDebugLogZone;
    gDebugLogZone = gDebugLogZone | (logZoneMask);

    HwiP_restore(oldIntState);

    return oldZoneMask;
}

uint32_t DebugP_logZoneDisable(uint32_t logZoneMask)
{
    uint32_t oldZoneMask;
    uintptr_t oldIntState;

    oldIntState = HwiP_disable();

    oldZoneMask = gDebugLogZone;
    gDebugLogZone = gDebugLogZone & ~(logZoneMask);

    HwiP_restore(oldIntState);

    return oldZoneMask;
}

void DebugP_logZoneRestore(uint32_t logZoneMask)
{
    uintptr_t oldIntState;

    oldIntState = HwiP_disable();

    gDebugLogZone = logZoneMask;

    HwiP_restore(oldIntState);
}

void _DebugP_assert(int expression, const char *file, const char *function, int line, const char *expressionString)
{
    if(expression==0)
    {
        volatile uint32_t assert_loop = 1;
        uint64_t curTime = ClockP_getTimeUsec();

        DebugP_log("ASSERT: %d.%ds: %s:%s:%d: %s failed !!!\r\n",
            (uint32_t)(curTime/1000000U),
            (uint32_t)(curTime%1000000U),
            file, function, line,
            expressionString
            );

        HwiP_disable();
        while(assert_loop)
        {
            /* loop forver */
        }
    }
}

void BOOT_SECTION _DebugP_assertNoLog(int expression)
{
    if(expression==0)
    {
        volatile uint32_t assert_loop = 1;

        HwiP_disable();
        while(assert_loop)
        {
            /* loop forver */
        }
    }
}

void DebugP_logChar(char a)
{
    DebugP_log("%c", a);
}

