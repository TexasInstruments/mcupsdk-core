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

#ifndef CACHEP_ARMV8_H
#define CACHEP_ARMV8_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/CacheP.h>

/* Wait for the 'Drain write buffer' to complete */
void CacheP_wait(void);

/* Invalidates all entries in the instruction Cache */
void CacheP_invL1pAll(void);

/* Disabel L1 Data cache */
void CacheP_disableL1D(void);

/* Disable L1 Instruction Cache */
void CacheP_disableL1P(void);

/* Enable L1 Data Cache */
void CacheP_enableL1D(void);

/* Enable L1 instruction cache */
void CacheP_enableL1P(void);

/* Invalidate range of L1 instruction cache */
void CacheP_invL1p(uintptr_t blockPtr, uint32_t byteCnt);

/* Invalidate range of L1 data cache */
void CacheP_invL1d(uintptr_t blockPtr, uint32_t byteCnt);

#if defined (SMP_FREERTOS)
/* Enable cache coherency between cores */
void CacheP_enableSMP(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* CACHEP_ARMV8_H */