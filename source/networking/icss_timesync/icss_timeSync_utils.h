/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

#ifndef ICSS_TIMESYNC_UTILS_H_
#define ICSS_TIMESYNC_UTILS_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */



/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
 *  \addtogroup NETWORKING_ICSS_TIMESYNC_MODULE
 *  @{
 */

/**
* \brief Add a word to the packet stream
* \param src pointer to the packet stream
* \param word the word which is to be added
*/
void TimeSync_addWord(uint8_t *src, uint32_t word);

/**
* \brief Add a half word to the packet stream
* \param src pointer to the packet stream
* \param halfWord the 16 bit word which is to be added
*/
void TimeSync_addHalfWord(volatile uint8_t *src, uint16_t halfWord);

/**
* \brief Compute checksum used in IP/UDP packets for a given stream
* \param packet pointer to the packet stream
* \param len length of stream
* \return Checksum computed
*/
uint32_t TimeSync_calcChecksum(uint8_t *packet, uint16_t len);

/**
* \brief Compute checksum for IP Header and modify in place
* \param packet pointer to the packet stream
*/
void TimeSync_calcIPChecksum(uint8_t *packet);

/**
* \brief Convert specified number of bytes in source from big endian bytes to little
* endian and vice versa.
* Assumption :
* 1. Source and Destination are different
* 2. Memory is properly allocated (!Function does not check for memory overrun)
* 3. Number of bytes is even
* \param src pointer to source byte stream
* \param dst pointer to destination byte stream
* \param numBytes number of bytes to convert and copy
*/
void TimeSync_convEndianess(volatile void *src, volatile void *dst, uint8_t numBytes);

/**
* \brief Takes in a 6 byte reverse byte endian source and puts it in an 64 bit double word with
* correct endianness. This function is specific to PTP
* Assumption :
* 1. Source and Destination are different
* 2. Memory is properly allocated (!Function does not check for memory overrun)
* \param src pointer to source byte stream
* \param dst pointer to destination byte stream
*/
void TimeSync_convEnd6to8(volatile void *src, void *dst);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef ICSS_TIMESYNC_UTILS_H_ */
