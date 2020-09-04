/**
 *  \file     cps.c
 *
 *  \brief    Platform specific functions to support CDN drivers
 *
 *  \copyright Copyright (C) 2019 Texas Instruments Incorporated -
 *             http://www.ti.com/
 */

/*
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
 */
#include <drivers/hw_include/cslr.h>
#include <cps.h>
#include <string.h>

#include "kernel/dpl/ClockP.h"
#include "kernel/dpl/CacheP.h"

#define _weak __attribute__((weak))


/* see cps.h */
_weak uint32_t CPS_UncachedRead32(volatile uint32_t* address) {
    return CSL_REG_RD(address);
}

uint32_t CPS_ReadReg32(volatile uint32_t* address)
{
    return CSL_REG_RD(address);
}


/* see cps.h */
_weak void CPS_UncachedWrite32(volatile uint32_t* address, uint32_t value) {
    CSL_REG_WR(address, value);
}


/* see cps.h */
_weak void CPS_CacheInvalidate(void* address, size_t size, uintptr_t devInfo) {
    (void) address;
    (void) size;
    (void) devInfo;

    CacheP_inv(address, size, CacheP_TYPE_ALL);
}

/* see cps.h */
_weak void CPS_CacheFlush(void* address, size_t size, uintptr_t devInfo) {
    (void) address;
    (void) size;
    (void) devInfo;

    CacheP_wbInv(address, size, CacheP_TYPE_ALL);
}

/* see cps.h */
/* we don't have ns resolution delay. And since Cadence driver
   is delaying 1000ns, so we are fine doing ns/1000 (i.e Cadence driver is
   not actually using nanosecond resolution */
extern void osalTimerDelayUs(uint32_t us);

_weak void CPS_DelayNs(uint32_t ns)
{
    ClockP_usleep(ns/1000);
}

static inline void CPS_RegWrite(volatile uint32_t* reg, uint32_t value)
{
    CSL_REG_WR(reg, value);
}

void CPS_WriteReg32(volatile uint32_t* address, uint32_t value)
{
    CSL_REG_WR(address, value);
}


#define LOW_32(x)   ((uint32_t)((uint64_t)x & (uint64_t)0xFFFFFFFF))
#define HI_32(x)    ((uint32_t)((uint64_t)x >> 32))
static inline void CPS_RegWrite64(volatile uint64_t* reg, uint64_t value)
{
    uintptr_t regAddr;

    regAddr = (uintptr_t)(reg);

    /* we are little endian system */
    CSL_REG_WR((uint32_t*)regAddr, LOW_32(value));
    CSL_REG_WR((uint32_t*)(regAddr+4), HI_32(value));
}


void CPS_UncachedWrite64(volatile uint64_t* address, uint64_t value)
{
    CPS_RegWrite64(address, value);
}



void CPS_MemoryBarrier(void)
{
    __asm__ __volatile__  ("    dmb"  "\n\t": : : "memory");
}


