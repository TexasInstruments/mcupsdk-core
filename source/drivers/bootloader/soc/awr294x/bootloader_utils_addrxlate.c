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

#include <drivers/bootloader.h>

typedef enum Utils_mmapMemId_types
{
    UTILS_MEMID_CR5A_TCMA_RAM,
    UTILS_MEMID_CR5A_TCMB_RAM,
    UTILS_MEMID_CR5B_TCMA_RAM,
    UTILS_MEMID_CR5B_TCMB_RAM,
    UTILS_MEMID_DSS_L2,
    UTILS_MEMID_DSS_L1D,
    UTILS_MEMID_DSS_L1P,
    UTILS_MEMID_MSS_L2,
    UTILS_MEMID_BSS_SHARED_RAM,
    UTILS_MEMID_BSS_TCMA_RAM,
    UTILS_MEMID_BSS_TCMB,
    UTILS_MEMID_BSS_STATIC_MEM,
    UTILS_MEMID_UNKNOWN
}Utils_mmapMemId_e;

typedef struct Utils_mmapSegmentEntry_s
{
    uint32_t baseAddress;
    uint32_t length;
    Utils_mmapMemId_e memId;
} Utils_mmapSegmentEntry;


static const Utils_mmapSegmentEntry gMemSegmentTblVirtCR5A[] =
{
    {
        .baseAddress = 0x00000000,
        .length      = (64 * 1024U),
        .memId       = UTILS_MEMID_CR5A_TCMA_RAM,
    },
    {
        .baseAddress = 0x00080000,
        .length      = (64 * 1024U),
        .memId       = UTILS_MEMID_CR5A_TCMB_RAM,
    },
    {
        .baseAddress = 0x10200000,
        .length      = (960 * 1024U),
        .memId       = UTILS_MEMID_MSS_L2,
    },
};

static const Utils_mmapSegmentEntry gMemSegmentTblVirtCR5B[] =
{
    {
        .baseAddress = 0x00000000,
        .length      = (32 * 1024U),
        .memId       = UTILS_MEMID_CR5B_TCMA_RAM,
    },
    {
        .baseAddress = 0x00080000,
        .length      = (32 * 1024U),
        .memId       = UTILS_MEMID_CR5B_TCMB_RAM,
    },
    {
        .baseAddress = 0x10200000,
        .length      = (960 * 1024U),
        .memId       = UTILS_MEMID_MSS_L2,
    },
};


static const Utils_mmapSegmentEntry gMemSegmentTblVirtDSP[] =
{
    {
        .baseAddress = 0x00E00000,
        .length      = (32 * 1024),
        .memId       = UTILS_MEMID_DSS_L1P,
    },
    {
        .baseAddress = 0x00F00000,
        .length      = (32 * 1024),
        .memId       = UTILS_MEMID_DSS_L1D,
    },
    {
        .baseAddress = 0x00800000,
        .length      = (384 * 1024U),
        .memId       = UTILS_MEMID_DSS_L2,
    },
};

static const Utils_mmapSegmentEntry gMemSegmentTblVirtBSS[] =
{
    {
        .baseAddress = 0x00000000,
        .length      = (256 * 1024),
        .memId       = UTILS_MEMID_BSS_SHARED_RAM,
    },
    {
        .baseAddress = 0x00080000,
        .length      = (32 * 1024),
        .memId       = UTILS_MEMID_BSS_TCMA_RAM,
    },
    {
        .baseAddress = 0x01000000,
        .length      = (64 * 1024),
        .memId       = UTILS_MEMID_BSS_TCMB,
    },
    {
        .baseAddress = 0x04020000,
        .length      = (16 * 1024),
        .memId       = UTILS_MEMID_BSS_STATIC_MEM,
    },
};

static const uint32_t gSelfCoreMemAddress[] =
{
    [UTILS_MEMID_CR5A_TCMA_RAM] = 0x00000000,
    [UTILS_MEMID_CR5A_TCMB_RAM] = 0x00080000,
    [UTILS_MEMID_CR5B_TCMA_RAM] = 0x00008000,
    [UTILS_MEMID_CR5B_TCMB_RAM] = 0x00088000,
    [UTILS_MEMID_DSS_L2]        = 0x80800000,
    [UTILS_MEMID_DSS_L1D]       = 0x80F00000,
    [UTILS_MEMID_DSS_L1P]       = 0x80E00000,
    [UTILS_MEMID_MSS_L2]        = 0x10200000,
    [UTILS_MEMID_BSS_SHARED_RAM]= 0xA0000000,
    [UTILS_MEMID_BSS_TCMA_RAM]  = 0xA0080000,
    [UTILS_MEMID_BSS_TCMB]      = 0xA1000000,
    [UTILS_MEMID_BSS_STATIC_MEM]= 0xA4020000,
};


#define UTILS_ARRAYSIZE(x)  (sizeof(x)/sizeof(x[0]))

static Utils_mmapMemId_e Utils_getMemId(uint32_t addr, const Utils_mmapSegmentEntry *memTbl, uint32_t numEntries, uint32_t *addrOffset)
{
    uint_fast8_t i;
    Utils_mmapMemId_e memId;

    for (i = 0; i < numEntries; i++)
    {
        if ((addr >= memTbl[i].baseAddress) && (addr < (memTbl[i].baseAddress + memTbl[i].length)))
        {
            break;
        }
    }
    if (i < numEntries)
    {
        memId = memTbl[i].memId;
        *addrOffset = (addr - memTbl[i].baseAddress);
    }
    else
    {
        memId = UTILS_MEMID_UNKNOWN;
    }
    return memId;
}

static void Utils_getVirtAddrTblInfo(Utils_mmapSegmentEntry const **tblBase, uint32_t *tblLen, uint32_t coreId)
{
    switch(coreId)
    {
        case CSL_CORE_ID_R5FSS0_0:
            *tblBase = &gMemSegmentTblVirtCR5A[0];
            *tblLen = UTILS_ARRAYSIZE(gMemSegmentTblVirtCR5A);
            break;
        case CSL_CORE_ID_R5FSS0_1:
            *tblBase = &gMemSegmentTblVirtCR5B[0];
            *tblLen = UTILS_ARRAYSIZE(gMemSegmentTblVirtCR5B);
            break;
        case CSL_CORE_ID_C66SS0:
            *tblBase = &gMemSegmentTblVirtDSP[0];
            *tblLen = UTILS_ARRAYSIZE(gMemSegmentTblVirtDSP);
            break;
        case CSL_CORE_ID_RSS_R4:
            *tblBase = &gMemSegmentTblVirtBSS[0];
            *tblLen = UTILS_ARRAYSIZE(gMemSegmentTblVirtBSS);
            break;
    }
}

uint32_t Bootloader_socTranslateSectionAddr(uint32_t cslCoreId, uint32_t addr)
{
    Utils_mmapMemId_e memId;
    Utils_mmapSegmentEntry const *tblBase;
    uint32_t tblLen;
    uint32_t physAddr;
    uint32_t addrOffset;

    Utils_getVirtAddrTblInfo(&tblBase, &tblLen, cslCoreId);
    memId = Utils_getMemId(addr, tblBase, tblLen, &addrOffset);
    if (UTILS_MEMID_UNKNOWN == memId)
    {
        physAddr = addr;
    }
    else
    {
        physAddr = gSelfCoreMemAddress[memId] + addrOffset;
    }
    return physAddr;
}
