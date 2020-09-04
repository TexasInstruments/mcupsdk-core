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

#include "MmuP_armv8_internal.h"

__attribute__((aligned(65536))) uintptr_t *gMmuLevel1Table;
__attribute__((aligned(65536))) uint64_t gMmuTableArray[MMUP_TABLE_LEN * MMUP_TABLE_ARRAY_LEN];
__attribute__((aligned(65536))) uint64_t gMmuTableArraySlot;

const MmuP_Info gMmuInfo = {
    MMUP_INDEX_MASK,
    MMUP_TABLE_LEN,
    {
        MMUP_TABLE_OFFSET_L0,
        MMUP_TABLE_OFFSET_L1,
        MMUP_TABLE_OFFSET_L2,
        MMUP_TABLE_OFFSET_L3
    },
    MMUP_GRANULE_SIZE_BITS,
    MMUP_INDEX_BITS,
    0
};

extern MmuP_Config gMmuConfig;
extern MmuP_RegionConfig gMmuRegionConfig[];

static void MmuP_addBlockEntry(uint8_t level, uintptr_t *tablePtr, uint16_t tableIdx,
    uintptr_t paddr, MmuP_MapAttrs *mapAttrs)
{
    uint64_t desc;

    if (level == 3)
    {
        desc = 0x3;
    }
    else
    {
        desc = MmuP_DescriptorType_BLOCK;
    }

    desc |= ((uint64_t)(mapAttrs->attrIndx & 0x7) << 2) |
            ((uint64_t)(0x1 << 5)) |
            ((uint64_t)(mapAttrs->accessPerm & 0x3) << 6) |
            ((uint64_t)(mapAttrs->shareable & 0x3) << 8) |
            ((uint64_t)(0x1) << 10) |  /* access flag */
            ((uint64_t)(!(mapAttrs->global) & 0x1) << 11) |
            ((uint64_t)(paddr & ~((1UL << gMmuInfo.tableOffset[level]) - 1))) |
            ((uint64_t)(!(mapAttrs->privExecute) & 0x1) << 53) |
            ((uint64_t)(!(mapAttrs->userExecute) & 0x1) << 54);

    tablePtr[tableIdx] = desc;
}


static void MmuP_freeTable(uint64_t *table)
{
    *table = gMmuTableArraySlot;
    gMmuTableArraySlot = (table - gMmuTableArray) / (MMUP_GRANULE_SIZE >> 3);
}


static void MmuP_readBlockEntry(uint8_t level, uint64_t *tablePtr, uint16_t tableIdx,
    uint64_t *paddr, MmuP_MapAttrs *mapAttrs)
{
    uint64_t desc;

    desc = tablePtr[tableIdx];

    mapAttrs->attrIndx = (desc >> 2) & 0x7;
    mapAttrs->accessPerm = (desc >> 6) & 0x3;
    mapAttrs->shareable = (desc >> 8) & 0x3;
    mapAttrs->global = !((desc >> 11) & 0x1);
    mapAttrs->privExecute = !((desc >> 53) & 0x1);
    mapAttrs->userExecute = !((desc >> 54) & 0x1);

    *paddr = desc & (uint64_t)MMUP_PADDR_MASK &
        ~((1UL << gMmuInfo.tableOffset[level]) - 1);
}

static uintptr_t* MmuP_allocTable()
{
    uint64_t *table;
    uint32_t i, tableLen = (MMUP_GRANULE_SIZE >> 3);

    if (gMmuTableArraySlot == (~0))
    {
        return (NULL);
    }

    table = &gMmuTableArray[tableLen * gMmuTableArraySlot];
    gMmuTableArraySlot = *table;

    /* Zero-out level 1 table */
    for (i = 0; i < tableLen; i++)
    {
        /*
         * Use (i << 2) instead of 0 to init table, in order to force
         * compiler to not use memset() as an optimization
         */
        table[i] = (i << 2);
    }

    return (table);
}


static uint64_t* MmuP_addTableEntry(uint64_t *tablePtr, UInt16 tableIdx,
    MmuP_MapAttrs *mapAttrs)
{
    uint64_t desc, *newTable;

    newTable = MmuP_allocTable();
    if (newTable == NULL)
    {
        return (NULL);
    }

    desc = ((uint64_t)MmuP_DescriptorType_TABLE & 0x3) |
           ((uint64_t)newTable & ~(MMUP_GRANULE_SIZE - 1));
    tablePtr[tableIdx] = desc;

    return (newTable);
}


static uint8_t MmuP_tableWalk(uint8_t level, uintptr_t *tablePtr, uintptr_t *vaddr, uintptr_t *paddr,
    uint32_t *size, MmuP_MapAttrs *mapAttrs)
{
    uint64_t desc;
    uint32_t blockSize;
    uint16_t tableIdx;
    uint8_t  blockTranslation;
    uint8_t  retStatus;
    uint64_t *nextLevelTablePtr;

    blockTranslation = 1;
    blockSize = 1UL << gMmuInfo.tableOffset[level];
    if (level == 0)
    {
        blockTranslation = 0;
    }

    tableIdx = (*vaddr >> gMmuInfo.tableOffset[level]) &
        gMmuInfo.indexMask;

    while ((*size != 0) && (tableIdx < gMmuInfo.tableLength))
    {
        desc = tablePtr[tableIdx];

        if (((desc & 0x3) == MmuP_DescriptorType_TABLE) && (level != 3))
        {
            if ((blockTranslation == 1) && (*size >= blockSize) &&
                (*vaddr && (blockSize - 1) == 0))
            {
                MmuP_addBlockEntry(level, tablePtr, tableIdx, *paddr, mapAttrs);
                *size = *size - blockSize;
                *vaddr = *vaddr + blockSize;
                *paddr = *paddr + blockSize;
                MmuP_freeTable((uint64_t *)(MMUP_PADDR_MASK & desc &
                    ~(uint64_t)(MMUP_GRANULE_SIZE - 1)));
            }
            else
            {
                nextLevelTablePtr = (uint64_t *)(MMUP_PADDR_MASK & desc &
                    ~(uint64_t)(MMUP_GRANULE_SIZE - 1));
                retStatus = MmuP_tableWalk(level + 1, nextLevelTablePtr,
                    vaddr, paddr, size, mapAttrs);
                if (!retStatus)
                {
                    return 0;
                }
            }
        }
        else if (((desc & 0x3) != MmuP_DescriptorType_TABLE) || (level == 3))
        {
            if ((level == 0) || ((level < 3) && (*size < blockSize)) ||
                ((*size >= blockSize) && ((*vaddr & (blockSize - 1)) != 0)))
            {
                uintptr_t vaddrCopy = (*vaddr & (~(blockSize - 1)));
                uintptr_t paddrCopy;
                MmuP_MapAttrs mapAttrsCopy;
                uint32_t sizeCopy = blockSize;

                if ((desc & 0x3) == MmuP_DescriptorType_BLOCK)
                {
                    MmuP_readBlockEntry(level, tablePtr, tableIdx, &paddrCopy,
                            &mapAttrsCopy);
                }

                nextLevelTablePtr =
                        MmuP_addTableEntry(tablePtr, tableIdx, mapAttrs);
                if (nextLevelTablePtr == NULL)
                {
                    return 0;
                }

                if ((desc & 0x3) == MmuP_DescriptorType_BLOCK)
                {
                    /*
                    * If old entry is a block entry, a new table entry is
                    * added and merged with the old block entry.
                    */
                    MmuP_tableWalk(level + 1, nextLevelTablePtr, &vaddrCopy,
                        &paddrCopy, &sizeCopy, &mapAttrsCopy);
                }

                retStatus = MmuP_tableWalk(level + 1, nextLevelTablePtr,
                        vaddr, paddr, size, mapAttrs);
                if (!retStatus)
                {
                    return 0;
                }
            }
            else if ((blockTranslation == 1) && (*size >= blockSize))
            {
                MmuP_addBlockEntry(level, tablePtr, tableIdx, *paddr, mapAttrs);
                *size = *size - blockSize;
                *vaddr = *vaddr + blockSize;
                *paddr = *paddr + blockSize;
            }
        }
        tableIdx++;
    }
    return 1;
}

static void MmuP_setConfig()
{
	uint32_t i;
	int32_t status;

	for(i = 0; i < gMmuConfig.numRegions; i++)
	{
		status = MmuP_map(gMmuRegionConfig[i].vaddr, gMmuRegionConfig[i].paddr,
						gMmuRegionConfig[i].size, &gMmuRegionConfig[i].attr);

        DebugP_assertNoLog(status == SystemP_SUCCESS);
	}
	return;
}

void MmuP_MapAttrs_init(MmuP_MapAttrs *attrs)
{
    DebugP_assertNoLog( attrs != NULL );

    attrs->accessPerm = MMUP_ACCESS_PERM_PRIV_RW_USER_NONE;
    attrs->privExecute = 1;
    attrs->userExecute = 0;
    attrs->shareable = MMUP_SHARABLE_OUTER;
    attrs->attrIndx = MMUP_ATTRINDX_MAIR0;
    attrs->global = 1;
}

void MmuP_enable()
{
    uintptr_t key;
    uint32_t type;

    /* if MMU is already enabled then just return */
    if (MmuP_isEnabled())
    {
        return;
    }

    key = HwiP_disable();

    type = CacheP_getEnabled();

    if (type & CacheP_TYPE_ALLP)
    {
        /* disable Instruction cache */
        CacheP_disable(CacheP_TYPE_ALLP);
    }

    /* Invalidate entire TLB */
    MmuP_tlbInvAll();

    /* enables the MMU */
    MmuP_enableI();

    /* set cache back to initial settings */
    CacheP_enable(type);

    HwiP_restore(key);
}

void MmuP_disable(void)
{
    uint32_t type;
    uintptr_t key;

    /* if MMU is alreay disabled, just return */
    if (!(MmuP_isEnabled())) {
        return;
    }

    key = HwiP_disable();

    /* get the current enabled bits */
    type = CacheP_getEnabled();

    if (type & CacheP_TYPE_L1D)
    {
        /* disable the L1 data cache */
        CacheP_disable(CacheP_TYPE_L1D);
    }

    if (type & CacheP_TYPE_L1P)
    {
        /* disable L1P cache */
        CacheP_disable(CacheP_TYPE_L1P);
    }

    /* disables the MMU */
    MmuP_disableI();

    /* set cache back to initial settings */
    CacheP_enable(type);

    HwiP_restore(key);
}

int32_t MmuP_map(uint64_t vaddr, uint64_t paddr, uint32_t size, MmuP_MapAttrs *mapAttrs)
{
    uint32_t key;
    uint8_t enabled, retStatus;
    int32_t status = SystemP_SUCCESS;

    /* Assert that mapAttrs != NULL */
    DebugP_assertNoLog( mapAttrs != NULL );

    /* Range check on vaddr and paddr */
    DebugP_assertNoLog(vaddr <= MMUP_PADDR_MASK);
    DebugP_assertNoLog(paddr <= MMUP_PADDR_MASK);

    /* Alignment check on vaddr, paddr & size */
    DebugP_assertNoLog((vaddr & (MMUP_GRANULE_SIZE - 1)) == 0);
    DebugP_assertNoLog((paddr & (MMUP_GRANULE_SIZE - 1)) == 0);
    DebugP_assertNoLog((size & (MMUP_GRANULE_SIZE - 1)) == 0);

    key = HwiP_disable();

    /* determine the current state of the MMU */
    enabled = MmuP_isEnabled();

    /* disable the MMU (if already disabled, does nothing) */
    MmuP_disable();

    if (gMmuInfo.noLevel0Table)
    {
        retStatus = MmuP_tableWalk(1, gMmuLevel1Table, &vaddr, &paddr,
            &size, mapAttrs);
    }
    else
    {
        retStatus = MmuP_tableWalk(0, gMmuLevel1Table, &vaddr, &paddr,
            &size, mapAttrs);
    }

    Armv8_dsbSy();

    if (enabled)
    {
        /* if Mmu was enabled, then re-enable it */
        MmuP_enable();
    }

    HwiP_restore(key);

    if(retStatus==1)
    {
        status = SystemP_SUCCESS;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return (status);
}

void MmuP_init()
{
    uint64_t tcr = 0;
    uint32_t i, tableLen = gMmuInfo.tableLength;

    /* Initialize MAIR */
    MmuP_setMAIRAsm(0, MmuP_MAIR0);
    MmuP_setMAIRAsm(1, MmuP_MAIR1);
    MmuP_setMAIRAsm(2, MmuP_MAIR2);
    MmuP_setMAIRAsm(3, MmuP_MAIR3);
    MmuP_setMAIRAsm(4, MmuP_MAIR4);
    MmuP_setMAIRAsm(5, MmuP_MAIR5);
    MmuP_setMAIRAsm(6, MmuP_MAIR6);
    MmuP_setMAIRAsm(7, MmuP_MAIR7);

    tcr = MMUP_GRANULE_SIZE;

    tcr = tcr | ((uint64_t)MMUP_PA_SIZE_ENCODING << 32) | MMUP_SHARABLE_OUTER |
          MMUP_OUTER_CACHEABLE | MMUP_INNER_CACHEABLE | (64 - MMUP_PA_MAX_WIDTH);

    MmuP_setTCR(tcr);

    /*
     *  When running in SMP mode, the MMU table init should be done
     *  by core 0 (master) only.
     *
     *  If not running in SMP mode, Core_getId() always returns 0,
     *  and the below init code will be run.
     */
    if (Armv8_getCoreId() == 0)
    {
        /* Initialize table array */
        for (i = 0; i < MMUP_TABLE_ARRAY_LEN; i++)
        {
            gMmuTableArray[tableLen * i] = i + 1;
        }

        gMmuTableArray[tableLen * (i - 1)] = (~0);
        gMmuTableArraySlot = 0;

        /* Allocate level1 Table */
        gMmuLevel1Table = (uintptr_t *)MmuP_allocTable();
    }

    /* Install MMU translation tables */
    MmuP_setTableBase(gMmuLevel1Table);

    /*
     * Call init function. This function is part of the application and will
     * add MMU mappings. If in SMP mode, core 0 has already done this.
     */
    if (Armv8_getCoreId() == 0)
    {
        MmuP_setConfig();
    }

    /* Invalidate entire TLB */
    MmuP_tlbInvAll();

    if (gMmuConfig.enableMmu == 1)
    {
        /* Enable MMU */
        MmuP_enableI();
    }
}