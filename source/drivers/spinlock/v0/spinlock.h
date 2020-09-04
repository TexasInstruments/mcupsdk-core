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
 *  \defgroup DRV_SPINLOCK_MODULE APIs for SPINLOCK
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the SPINLOCK module.
 *
 *  @{
 */

/**
 *  \file v0/spinlock.h
 *
 *  \brief SPINLOCK Driver API/interface file.
 *
 */

#ifndef SPINLOCK_V0_H_
#define SPINLOCK_V0_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/hw_include/cslr_spinlock.h>

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/** \brief Flag to indicate spinlock is free */
#define SPINLOCK_LOCK_STATUS_FREE       (0U)
/** \brief Flag to indicate spinlock is in use */
#define SPINLOCK_LOCK_STATUS_INUSE      (1U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief   This API provides returns the number of locks supported
 *
 *  \param   baseAddr       Memory address of SPINLOCK module
 *
 *  \return  Number of locks in an instance
 */
uint32_t Spinlock_getNumLocks(uint32_t baseAddr);

/**
 *  \brief   This API performs the Read operation for Lock status of the
 *           SPINLOCK_LOCK_REG, in order to acquire a Spinlock
 *
 *  \param   baseAddr       Memory address of SPINLOCK module
 *  \param   lockNumber     Lock number in Spinlock module that should be
 *                          acquired
 *
 *  \return  Status of the Lock Register
 *           Lock state :
 *           #SPINLOCK_LOCK_STATUS_FREE: Lock was previously NOT Taken (Free).
 *                                       The requester is granted the lock
 *           #SPINLOCK_LOCK_STATUS_INUSE: Lock was previously Taken (Not Free).
 *                                        The requester is not granted the
 *                                        lock and must retry
 *           #SystemP_FAILURE: lockNumber is invalid (out of range)
 */
int32_t Spinlock_lock(uint32_t baseAddr, uint32_t lockNumber);

/**
 *  \brief   This API performs the write operation for Lock status of the
 *           SPINLOCK_LOCK_REG, in order to Free the lock
 *           If lockNumber is invalid (out of range), no operation is performed
 *           Lock state :
 *              Write 0x0: Set the lock to Not Taken(Free)
 *              Write 0x1: No update to the lock value
 *
 *  \param   baseAddr       Memory address of SPINLOCK module
 *  \param   lockNumber     Lock number in Spinlock module that should be
 *                          released
 */
void Spinlock_unlock(uint32_t baseAddr, uint32_t lockNumber);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif  /* #ifndef SPINLOCK_V0_H_ */

#endif

/** @} */
