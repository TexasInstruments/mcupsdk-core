/*
 * Copyright (C) 2021 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  \file   spinlock.c
 *
 *  \brief  Low lever APIs performing hardware register writes and reads for
 *          SPINLOCK IP version 0.
 *
 *   This file contains the hardware register write/read APIs for SPINLOCK.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/spinlock.h>
#include <drivers/hw_include/cslr.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define CSL_SPINLOCK_LOCK_REG_TAKEN_FREE                        (0x00000000U)
#define CSL_SPINLOCK_LOCK_REG_TAKEN_BUSY                        (0x00000001U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

uint32_t Spinlock_getNumLocks(uint32_t baseAddr)
{
    uint32_t         numLocks;
    CSL_spinlockRegs *pSpinlockRegs = (CSL_spinlockRegs *)((uintptr_t)baseAddr);

    numLocks = CSL_REG32_FEXT(&pSpinlockRegs->SYSTATUS, SPINLOCK_SYSTATUS_NUMLOCKS);
    numLocks <<= 5U;        /* # numlock is 32 * field value read */

    return (numLocks);
}

int32_t Spinlock_lock(uint32_t baseAddr, uint32_t lockNumber)
{
    int32_t         lockStatus = SystemP_FAILURE;
    CSL_spinlockRegs *pSpinlockRegs = (CSL_spinlockRegs *)((uintptr_t)baseAddr);

    if(lockNumber < Spinlock_getNumLocks(baseAddr))
    {
        lockStatus = CSL_REG32_RD(&pSpinlockRegs->LOCK_REG[lockNumber]);
    }

    return (lockStatus);
}

void Spinlock_unlock(uint32_t baseAddr, uint32_t lockNumber)
{
    CSL_spinlockRegs *pSpinlockRegs = (CSL_spinlockRegs *)((uintptr_t)baseAddr);

    if(lockNumber < Spinlock_getNumLocks(baseAddr))
    {
        CSL_REG32_WR(&pSpinlockRegs->LOCK_REG[lockNumber], CSL_SPINLOCK_LOCK_REG_TAKEN_FREE);
    }

    return;
}
