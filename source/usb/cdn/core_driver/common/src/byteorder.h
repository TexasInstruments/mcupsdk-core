/******************************************************************************
*
* Copyright (C) 2012-2021 Cadence Design Systems, Inc.
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* 3. Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
* byteorder.h
* File contains Endian handling
******************************************************************************/

#ifndef BYTEORDER_H
#define BYTEORDER_H

#ifdef __cplusplus
extern "C"
{
#endif


#if !defined(CPU_LITTLE_ENDIAN) && !defined(CPU_BIG_ENDIAN)
#define CPU_LITTLE_ENDIAN
#endif
/* XHCI is LE order so we don't need to have BE macros */

#include "cdn_stdtypes.h"

/* parasoft-begin-suppress MISRA2012-DIR-4_9-4 "Do not define function-like macro, DRV-4346" */
#define swap64(x) ((uint64_t) ( \
                       (((uint64_t) (x) & 0x00000000000000FFULL) << 56U) | \
                       (((uint64_t) (x) & 0x000000000000FF00ULL) << 40U) | \
                       (((uint64_t) (x) & 0x0000000000FF0000ULL) << 24U) | \
                       (((uint64_t) (x) & 0x00000000FF000000ULL) << 8U) | \
                       (((uint64_t) (x) & 0x000000FF00000000ULL) >> 8U) | \
                       (((uint64_t) (x) & 0x0000FF0000000000ULL) >> 24U) | \
                       (((uint64_t) (x) & 0x00FF000000000000ULL) >> 40U) | \
                       (((uint64_t) (x) & 0xFF00000000000000ULL) >> 56U)))

#define swap32(x) ((uint32_t) ( \
                       (((uint32_t) (x) & 0x000000FFUL) << 24U) | \
                       (((uint32_t) (x) & 0x0000FF00UL) << 8U) | \
                       (((uint32_t) (x) & 0x00FF0000UL) >> 8U) | \
                       (((uint32_t) (x) & 0xFF000000UL) >> 24U)))

#define swap16(x) ((uint16_t) ( \
                       (((uint16_t) (x) & 0x00FFU) << 8U) | \
                       (((uint16_t) (x) & 0xFF00U) >> 8U)))

#ifdef CPU_BIG_ENDIAN
#define cpuToLe64(x) swap64(x)
#define cpuToLe32(x) swap32(x)
#define cpuToLe16(x) swap16(x)
#define le64ToCpu(x) swap64(x)
#define le32ToCpu(x) swap32(x)
#define le16ToCpu(x) swap16(x)
#define cpuToBe64(x) (x)
#define cpuToBe32(x) (x)
#define cpuToBe16(x) (x)
#define be64ToCpu(x) (x)
#define be32ToCpu(x) (x)
#define be16ToCpu(x) (x)
#else
#define cpuToLe64(x) (x)
#define cpuToLe32(x) (x)
#define cpuToLe16(x) (x)
#define le64ToCpu(x) (x)
#define le32ToCpu(x) (x)
#define le16ToCpu(x) (x)
#define cpuToBe64(x) swap64(x)
#define cpuToBe32(x) swap32(x)
#define cpuToBe16(x) swap16(x)
#define be64ToCpu(x) swap64(x)
#define be32ToCpu(x) swap32(x)
#define be16ToCpu(x) swap16(x)
#endif

#ifdef PLATFORM_64_BIT
#define leXToCpu le64ToCpu
#else
#define leXToCpu le32ToCpu
#endif
/* parasoft-end-suppress MISRA2012-DIR-4_9-4 "Do not define function-like macro" */

#ifdef __cplusplus
}
#endif

#endif /* BYTEORDER_H */

