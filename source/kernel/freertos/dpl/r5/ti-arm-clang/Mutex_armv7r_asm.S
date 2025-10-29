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


.text

.equ locked, 0x1
.equ unlocked, 0x0

//==============================================================================
//void lock_mutex(void * mutex);
//uint32_t try_lock_mutex(void * mutex);
//void unlock_mutex(void * mutex);
//==============================================================================
    .global lock_mutex
    .global try_lock_mutex
    .global unlock_mutex
    .global lock_mutex_wait
    .global lock_mutex_locked

.macro wait_for_update
    wfi                     // Indicate opportunity to enter low-power state
.endm

.macro signal_update
                            // No software signalling operation
.endm

// lock_mutex
lock_mutex:
    mov     r1, #1
lock_mutex_retry:
    ldrex   r2, [r0]
    cmp     r2, r1          // Test if mutex is locked or unlocked
    beq     lock_mutex_wait           // If locked - wait for it to be released, from 2
    strexne r2, r1, [r0]    // Not locked, attempt to lock it
    cmpne   r2, #1          // Check if Store-Exclusive failed
    beq     lock_mutex_retry           // Failed - retry from 1
    // Lock acquired
    dmb                     // Required before accessing protected resource
    bx      lr

lock_mutex_wait:            // Take appropriate action while waiting for mutex to become unlocked
    wait_for_update
    b       lock_mutex_retry           // Retry from 1(lock_mutex_retry)

// try_lock_mutex
try_lock_mutex:
    mov     r1, #1
    ldrex   r2, [r0]
    cmp     r2, r1          // Test if mutex is locked or unlocked
    beq     lock_mutex_locked           // If locked, return 1
    strexne r2, r1, [r0]    // Not locked, attempt to lock it
    // Lock acquired
    dmb                     // Required before accessing protected resource
    mov     r0, #0
    bx      lr
lock_mutex_locked:            // Take appropriate action while waiting for mutex to become unlocked
    mov     r0, #1
    bx      lr           // return with mutex locked

// unlock_mutex
unlock_mutex:
    mov     r1, #0
    dmb                     // Required before releasing protected resource
    str     r1, [r0]        // Unlock mutex
    signal_update
    bx      lr

        .end

