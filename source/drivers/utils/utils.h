/*
 * Copyright (C) 2026 Texas Instruments Incorporated
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

#ifndef UTILS_H_
#define UTILS_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#if defined(__C7504__) || defined(__C7524__)
#include <c7x.h>
#endif

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  This function copy data from the source to destination using
 *  uintptr_t pointer for the aligned bytes and 8-bit pointer
 *  for unaligned bytes.
 */
void Utils_memcpyWord(uint8_t *source, uint8_t *destination, uint32_t length);

/**
 * \brief Data and Instruction barrier
 *
 * Assembly code for data and instruction barrier
 */
void Utils_dataAndInstructionBarrier(void);

/**
 * \brief This function copy data from the source to destination using
 *  uintptr_t pointer for the unaligned source
 *
 * \param destination Destination address
 * \param source Source address
 * \param size Length of data to be copied
 *
 */
void Utils_memcopySourceUnalingned(void *destination, const volatile void *source,
                                   uint32_t size);
/**
 * \brief This function copy data from the source to destination using
 *  uintptr_t pointer for the unaligned destination
 *
 * \param destination Destination address
 * \param source Source address
 * \param size Length of data to be copied
 *
 */
void Utils_memcopyDestinationUnalingned(void *destination, const volatile void *source,
                                        uint32_t size);

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef UTILS_H_ */

/** @} */
