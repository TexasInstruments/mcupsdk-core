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

#include "CacheP_armv8.h"

void CacheP_disable(uint32_t type)
{
    uint32_t enabled;
    uintptr_t key;

    enabled = CacheP_getEnabled();

    if (enabled & (type & (CacheP_TYPE_L1D | CacheP_TYPE_L2)))
    {
        /* Disable Interurpts */
        key = HwiP_disable();
        /* Disable L1D and L2 Cache */
        CacheP_disableL1D();
        /* Ensure data cache written back and disabled */
        CacheP_wait();
        /* Re-enable interrupts */
        HwiP_restore(key);

    }

    if (enabled & (type & CacheP_TYPE_L1P)) {
        CacheP_disableL1P();             /* Disable ICache */
    }
}


void CacheP_enable(uint32_t type)
{
    uint32_t disabled;

#if defined (SMP_FREERTOS)
    /* Set SMPEN flag when running SMP FreeRTOS */
    CacheP_enableSMP();
#endif

    /* only enable caches that are currently disabled */
    disabled = ~(CacheP_getEnabled());

    if (disabled & (type & (CacheP_TYPE_L1D | CacheP_TYPE_L2))) {
        CacheP_enableL1D();              /* Enable L1D and L2 Cache */
    }

    if (disabled & (type & CacheP_TYPE_L1P)) {
        CacheP_enableL1P();              /* Enable ICache */
    }
}

void CacheP_inv(void *blockPtr, uint32_t byteCnt, uint32_t type)
{
    if (type & (CacheP_TYPE_L1P | CacheP_TYPE_L2P))
    {
        CacheP_invL1p((uintptr_t)blockPtr, byteCnt);
    }

    if (type & (CacheP_TYPE_L1D | CacheP_TYPE_L2D))
    {
        CacheP_invL1d((uintptr_t)blockPtr, byteCnt);
    }
}