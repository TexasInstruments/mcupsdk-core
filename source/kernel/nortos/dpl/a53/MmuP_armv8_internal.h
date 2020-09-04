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

#ifndef MMUP_ARMV8_INTERNAL_H
#define MMUP_ARMV8_INTERNAL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <kernel/dpl/MmuP_armv8.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/DebugP.h>
#include "common_armv8.h"
#include <drivers/hw_include/tistdtypes.h>

#define MMUP_PADDR_MASK          0x0000FFFFFFFFFFFF
#define MMUP_GRANULE_SIZE        MMUP_GRANULE_SIZE_4KB
#define MMUP_TABLE_ARRAY_LEN    16
#define MMUP_TABLE_LEN          512

#define MMUP_GRANULE_SIZE_BITS  12
#define MMUP_INDEX_BITS         MMUP_GRANULE_SIZE_BITS - 3
#define MMUP_TABLE_OFFSET_L3    MMUP_GRANULE_SIZE_BITS
#define MMUP_TABLE_OFFSET_L2    MMUP_TABLE_OFFSET_L3 + MMUP_INDEX_BITS
#define MMUP_TABLE_OFFSET_L1    MMUP_TABLE_OFFSET_L2 + MMUP_INDEX_BITS
#define MMUP_TABLE_OFFSET_L0    MMUP_TABLE_OFFSET_L1 + MMUP_INDEX_BITS
#define MMUP_INDEX_MASK         0x1FF   /* ((1 << MMUP_INDEX_BITS) - 1) */

#define MMUP_PA_SIZE_ENCODING   0x5

#define MMUP_OUTER_SHAREABLE    0x2000
#define MMUP_OUTER_CACHEABLE    0x0400
#define MMUP_INNER_CACHEABLE    0x0100

#define MMUP_PA_MAX_WIDTH            48

/* Memory attribute 0.
 *
 *  Default is memory with non-gathering, non-reordering and no early write
 *  acknowledegement property. */
#define MmuP_MAIR0 0x00

/* Memory attribute 1
 *
 *  Default is memory with non-gathering, non-reordering and early write
 *  acknowledegement property. */
#define MmuP_MAIR1 0x04


/* Memory attribute 2
 *
 *  Default is memory with non-gathering, reordering and early write
 *  acknowledegement property. */
#define MmuP_MAIR2 0x08


/* Memory attribute 3
 *
 *  Default is memory with gathering, reordering and early write
 *  acknowledegement property. */
#define MmuP_MAIR3 0x0C


/*  Memory attribute 4
 *
 *  Default is normal inner & outer non-cacheable memory. */
#define MmuP_MAIR4 0x44


/* Memory attribute 5
 *
 *  Default is normal outer non-cacheable, inner write-back cacheable
 *  non-transient memory.*/
#define MmuP_MAIR5 0x4F


/* Memory attribute 6
 *
 *  Default is normal outer & inner write-through cacheable non-transient
 *  memory. */
#define MmuP_MAIR6 0xBB


/* Memory attribute 7
 *
 *  Default is normal outer and inner write-back cacheable non-transient
 *  memory. */
#define MmuP_MAIR7 0xFF

/* GranuleSize */
typedef enum MmuP_GranuleSize_ {
    MMUP_GRANULE_SIZE_4KB = 0x1000,
    MMUP_GRANULE_SIZE_16KB = 0x4000,
    MMUP_GRANULE_SIZE_64KB = 0x10000
} MmuP_GranuleSize;

typedef struct MmuP_Info_ {
    uint64_t indexMask;
    uint32_t tableLength;
    uint8_t  tableOffset[4];
    uint8_t  granuleSizeBits;
    uint8_t  indexBits;
    uint8_t  noLevel0Table;
} MmuP_Info;

typedef enum MmuP_DescriptorType_ {
    MmuP_DescriptorType_INVALID0 = 0,   /*! Virtual address is unmapped     */
    MmuP_DescriptorType_BLOCK = 1,      /*! Block descriptor                */
    MmuP_DescriptorType_INVALID1 = 2,   /*! Virtual address is unmapped     */
    MmuP_DescriptorType_TABLE = 3       /*! Next-level table address        */
} MmuP_DescriptorType;

/* Initialize MMU */
void MmuP_enableI(void);

/* Disables MMU */
void MmuP_disableI(void);

/* Write to TCR */
void MmuP_setTCR(uint64_t tcr);

/* Set Translation Table Base address */
void MmuP_setTableBase(uintptr_t *table_addr);

/* Write attr to MAIRn register */
void MmuP_setMAIRAsm(uint8_t idx, uint8_t attr);

/* Invalidate TLB */
void MmuP_tlbInvAll(void);


#ifdef __cplusplus
}
#endif

#endif /* MMUP_ARMV8_INTERNAL_H */