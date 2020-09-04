/*
 *  Copyright (c) Texas Instruments Incorporated 2022
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

/*!
 * \file timeSync_tools.h
 *
 * \brief This file contains the interface to helper functions
 *        of timeSync driver
 */

/*!
 * \addtogroup TIMESYNC_HAL_API
 * @{
 */

#ifndef TIMESYNC_TOOLS_H
#define TIMESYNC_TOOLS_H

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/** Macro to check if bit at given bit position is set */
#define TIMESYNC_IS_BIT_SET(val, n)        (((val) & (1U << (n))) != 0U)

/**Starting offset for an IP header in TCP/IP packet*/
#define TIMESYNC_START_OF_IP_HEADER        (14)

/**Starting offset for checksum in IP header in TCP/IP packet*/
#define TIMESYNC_START_OF_IP_CHECKSUM      (TIMESYNC_START_OF_IP_HEADER + 10U)

/**Starting offset for UDP header in TCP/IP packet*/
#define TIMESYNC_START_OF_UDP_HEADER       (34U)

/**standard size for IP header in TCP/IP packet*/
#define TIMESYNC_DEFAULT_IP_HEADER_SIZE    (20U)

/**Compare MAC ID's. Uses memcmp internally */
#define TIMESYNC_COMPARE_MAC(x, y)         (memcmp((x), (y), 6U) == 0)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None*/

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief Add a word to the packet stream
 * @param src  Pointer to the packet stream
 * @param word Word which is to be added
 * @return None
 */
void TimeSync_addWord(uint8_t *src,
                      uint32_t word);

/**
 * @brief Add a half word to the packet stream
 * @param src      Pointer to the packet stream
 * @param halfWord 16-bit word which is to be added
 * @return None
 */
void TimeSync_addHalfWord(volatile uint8_t *src,
                          uint16_t halfWord);

/**
 * @brief Convert specified number of bytes in source from big endian bytes to little
 * endian and vice versa.
 * Assumption :
 * 1. Source and Destination are different
 * 2. Memory is properly allocated (!Function does not check for memory overrun)
 * 3. Number of bytes is even
 * @param src      Pointer to source byte stream
 * @param dst      Pointer to destination byte stream
 * @param numBytes Number of bytes to convert and copy
 * @return None
 */
void TimeSync_convEndianess(volatile void *src,
                            volatile void *dst,
                            uint8_t numBytes);

/**
 * @brief Compute checksum used in IP/UDP packets for a given stream
 * @param packet Pointer to the packet stream
 * @param len    Length of stream
 * @return Checksum computed
 */
uint32_t TimeSync_calcChecksum(uint8_t *packet,
                               uint16_t len);

/**
 * @brief Compute checksum for IP Header and modify in place
 * @param packet Pointer to the packet stream
 * @return None
 */
void TimeSync_calcIpChecksum(uint8_t *packet);

/**
 * @brief get a halfword from the network stream, convert it to little endian format and return it
 * assumption halfword is present in big endian byte format and starts from zero offset
 * @param byte Pointer to byte stream
 * @return halfword halfword in little endian format
 */
uint16_t TimeSync_convBigEndianToLittleEndianHalfWord(uint8_t *byte);

/**
 * @brief Takes in a 6 byte reverse byte endian source and puts it in an 64 bit double word with
 * correct endianness. This function is specific to PTP
 * Assumption :
 * 1. Source and Destination are different
 * 2. Memory is properly allocated (!Function does not check for memory overrun)
 * @param src Pointer to source byte stream
 * @param dst Pointer to destination byte stream
 * @return None
 */
void TimeSync_convEnd6to8(volatile void *src,
                          void *dst);

#endif /* TIMESYNC_TOOLS_H */
