/* =============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2022
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
 *
 */

#ifndef CSLR_DTHE_H
#define CSLR_DTHE_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>
/**
 *  CSL DTHE Register Layer
 *
 *  Structure type to access the DTHE Header file.
 */
typedef struct
{
    volatile uint32_t S_EIP_CONFIG;         /**< Offset: 0x0000 */
    volatile uint32_t RESERVED0[3];         /**< Offset: 0x0004 */
} CSL_DTHECfgRegs;

typedef struct
{
    volatile uint32_t EIP57T_IMST;        /**< Offset: 0x0010 */
    volatile uint32_t EIP57T_IRIS;        /**< Offset: 0x0014 */
    volatile uint32_t EIP57T_IMIS;        /**< Offset: 0x0018 */
    volatile uint32_t EIP57T_ICIS;        /**< Offset: 0x001c */
    volatile uint32_t EIP36T_IMST;        /**< Offset: 0x0020 */
    volatile uint32_t EIP36T_IRIS;        /**< Offset: 0x0024 */
    volatile uint32_t EIP36T_IMIS;        /**< Offset: 0x0028 */
    volatile uint32_t EIP36T_ICIS;        /**< Offset: 0x002c */
    volatile uint32_t EIP16T_IMST;        /**< Offset: 0x0030 */
    volatile uint32_t EIP16T_IRIS;        /**< Offset: 0x0034 */
    volatile uint32_t EIP16T_IMIS;        /**< Offset: 0x0038 */
    volatile uint32_t EIP16T_ICIS;        /**< Offset: 0x003c */
    volatile uint32_t RESERVED1[112];       /**< Offset: 0x0040 */
    volatile uint32_t EIP_CGCFG;          /**< Offset: 0x0200 */
    volatile uint32_t EIP_CGREQ;          /**< Offset: 0x0204 */
    volatile uint32_t RESERVED2[126];       /**< Offset: 0x0208 */
    volatile uint32_t CRC_CTRL;           /**< Offset: 0x0400 */
    volatile uint32_t RESERVED3[3];         /**< Offset: 0x0404 */
    volatile uint32_t CRC_SEED;           /**< Offset: 0x0410 */
    volatile uint32_t CRC_DIN;            /**< Offset: 0x0414 */
    volatile uint32_t CRC_RSLT_PP;        /**< Offset: 0x0418 */
    volatile uint32_t RESERVED4[185];       /**< Offset: 0x041c */
    /* Only accessible for public context*/
    volatile uint32_t RAND_KEY0;          /**< Offset: 0x0700 */
    volatile uint32_t RAND_KEY1;          /**< Offset: 0x0704 */
    volatile uint32_t RAND_KEY2;          /**< Offset: 0x0708 */
    volatile uint32_t RAND_KEY3;          /**< Offset: 0x070c */
} CSL_DTHERegs;

#ifdef __cplusplus
}
#endif
#endif  /* CSLR_DTHE_H */
