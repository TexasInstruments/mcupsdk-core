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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/DebugP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define L1PCFG                  ((volatile uint32_t *) 0x01840020U)
#define L1PCC                   ((volatile uint32_t *) 0x01840024U)
#define L1PIBAR                 ((volatile uint32_t *) 0x01844020U)
#define L1PIWC                  ((volatile uint32_t *) 0x01844024U)
#define L1PINV                  ((volatile uint32_t *) 0x01845028U)
#define L1PCC_OPER_MASK         (0x00000007U)
#define L1PCC_OPER_FREEZE       (0x00000001U)
#define L1PCC_OPER_NORMAL       (0x00000000U)
#define L1PCC_POPER_MASK        (0x00070000U)
#define L1PCC_POPER_FREEZE      (0x00010000U)
#define L1PCC_POPER_NORMAL      (0x00000000U)

#define L1DCFG                  ((volatile uint32_t *) 0x01840040U)
#define L1DCC                   ((volatile uint32_t *) 0x01840044U)
#define L1DWIBAR                ((volatile uint32_t *) 0x01844030U)
#define L1DWIWC                 ((volatile uint32_t *) 0x01844034U)
#define L1DWBAR                 ((volatile uint32_t *) 0x01844040U)
#define L1DWWC                  ((volatile uint32_t *) 0x01844044U)
#define L1DIBAR                 ((volatile uint32_t *) 0x01844048U)
#define L1DIWC                  ((volatile uint32_t *) 0x0184404CU)
#define L1DWB                   ((volatile uint32_t *) 0x01845040U)
#define L1DWBINV                ((volatile uint32_t *) 0x01845044U)
#define L1DINV                  ((volatile uint32_t *) 0x01845048U)
#define L1DCC_OPER_MASK         (0x00000007U)
#define L1DCC_OPER_FREEZE       (0x00000001U)
#define L1DCC_OPER_NORMAL       (0x00000000U)
#define L1DCC_POPER_MASK        (0x00070000U)
#define L1DCC_POPER_FREEZE      (0x00010000U)
#define L1DCC_POPER_NORMAL      (0x00000000U)

#define L2CFG                   ((volatile uint32_t *) 0x01840000U)
#define L2WBAR                  ((volatile uint32_t *) 0x01844000U)
#define L2WWC                   ((volatile uint32_t *) 0x01844004U)
#define L2WIBAR                 ((volatile uint32_t *) 0x01844010U)
#define L2WIWC                  ((volatile uint32_t *) 0x01844014U)
#define L2IBAR                  ((volatile uint32_t *) 0x01844018U)
#define L2IWC                   ((volatile uint32_t *) 0x0184401CU)
#define L2WB                    ((volatile uint32_t *) 0x01845000U)
#define L2WBINV                 ((volatile uint32_t *) 0x01845004U)
#define L2INV                   ((volatile uint32_t *) 0x01845008U)
#define L2CFG_L2CC_MASK         (0x00000018U)
#define L2CFG_L2CC_ENABLE       (0x00000000U)
#define L2CFG_L2CC_FREEZE       (0x00000008U)
#define L2CFG_L2CC_BYPASS       (0x00000010U)
#define L2CFG_MODE_MASK         (0x00000007U)

#define MAR                     ((volatile uint32_t *) 0x01848000U)
#define MAR_PFX_MASK            (0x00000008U)

#define XPFCMD                  ((volatile uint32_t *) 0x08000300U)

/* Max word count per cache operations */
#define MAXWC                   (0xFF00U)

/* This parameter is used to break up large blocks into multiple
 * small blocks which are done atomically.  Each block of the
 * specified size waits for the cache operation to finish before
 * starting the next block. Setting this size to 0, means the
 * cache operations are not done atomically.
 */
#define CacheP_ATOMIC_BLOCKSIZE (1024U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void CacheP_all(volatile uint32_t *cacheReg);
static inline void CacheP_wait(void);
static void CacheP_block(void *addr, uint32_t size, volatile uint32_t *barReg);
static void CacheP_invPrefetchBuffer(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void CacheP_init(void)
{
    uint32_t            i;
    CacheP_MarRegion   *mar;

    /* Set the default cache size */
    CacheP_setSize(&gCacheSize);

    /* Mar setup */
    for(i = 0U; i < gCacheMarRegionNum; i++)
    {
        mar = &gCacheMarRegion[i];
        CacheP_setMar(mar->baseAddr, mar->size, mar->value);
    }

    return;
}

void CacheP_enable(uint32_t type)
{
    volatile uint32_t   regVal;
    CacheP_Size        *size = &gCacheSize;

    if((type & CacheP_TYPE_L1P)!=0U)
    {
        CacheP_setSize(size);
    }

    if((type & CacheP_TYPE_L1D)!=0U)
    {
        CacheP_setSize(size);
    }

    if((type & CacheP_TYPE_L2)!=0U)
    {
        /* set the L2 mode to normal */
        regVal = *L2CFG;
        *L2CFG = (regVal & ~L2CFG_L2CC_MASK) | (L2CFG_L2CC_ENABLE);
        regVal = *L2CFG;
    }

    return;
}

void CacheP_disable(uint32_t type)
{
    CacheP_Size size;

    /* Only L1 is supported, reject any other type
     * To disable L2 use CacheP_setSize() and/or CacheP_setMar() */
    DebugP_assertNoLog((type & ~CacheP_TYPE_L1) == 0U);

    CacheP_getSize(&size);

    if((type & CacheP_TYPE_L1P)!=0U)
    {
        size.l1pSize = CacheP_L1Size_0K;
        CacheP_setSize(&size);
    }

    if((type & CacheP_TYPE_L1D)!=0U)
    {
        size.l1dSize = CacheP_L1Size_0K;
        CacheP_setSize(&size);
    }

    return;
}

uint32_t CacheP_getEnabled(void)
{
    uint32_t        type = 0;
    CacheP_Size     size;

    CacheP_getSize(&size);
    if(size.l1pSize > CacheP_L1Size_0K)
    {
        type |= CacheP_TYPE_L1P;
    }
    if(size.l1dSize > CacheP_L1Size_0K)
    {
        type |= CacheP_TYPE_L1D;
    }
    if(size.l2Size > CacheP_L2Size_0K)
    {
        type |= CacheP_TYPE_L2D;
    }

    return (type);
}

void CacheP_wbAll(uint32_t type)
{
    if((type & CacheP_TYPE_L2D)!=0U)
    {
        /* Perform a global write back. There is no effect on L1P cache. All cache
         * lines are left valid in L1D cache and the data in L1D cache is written
         * back L2 or external. All cache lines are left valid in L2 cache and the
         * data in L2 cache is written back to external */
        CacheP_all(L2WB);
    }
    else
    {
        if((type & CacheP_TYPE_L1D)!=0U)
        {
            /* L1D alone set without L2 */
            CacheP_all(L1DWB);
        }
        else
        {
            /* Do nothing - there is no Wb for L1P */
        }
    }

    return;
}

void CacheP_wbInvAll(uint32_t type)
{
    if((type & CacheP_TYPE_L2D)!=0U)
    {
        /* Performs a global write back and invalidate. All cache lines are
         * invalidated in L1P cache. All cache lines are written back to L2 or
         * or external then invalidated in L1D cache. All cache lines are
         * written back to external and then invalidated in L2 cache */
        CacheP_all(L2WBINV);
    }
    else
    {
        if((type & CacheP_TYPE_L1D)!=0U)
        {
            /* L1D set without L2 */
            CacheP_all(L1DWBINV);
        }
        if((type & CacheP_TYPE_L1P)!=0U)
        {
            /* L1P set without L2 - L1P has only invalidate */
            CacheP_all(L1PINV);
        }
    }

    return;
}

void CacheP_wb(void *addr, uint32_t size, uint32_t type)
{
    /*
     * Writes back the range of memory within the specified starting address
     * and byte count.  The range of addresses operated on gets quantized to
     * whole cache lines in each cache.  There is no effect on L1P cache.
     * All cache lines within the range are left valid in L1D cache and the data
     * within the range in L1D cache will be written back to L2 or external.
     * All cache lines within the range are left valid in L2 cache and the data
     * within the range in L2 cache will be written back to external
     */
    CacheP_block(addr, size, L2WBAR);

    return;
}

void CacheP_inv(void *addr, uint32_t size, uint32_t type)
{
    /*
     * Invalidate the range of memory within the specified starting address and
     * byte count. The range of addresses operated on gets quantized to whole
     * cache lines in each cache. All cache lines in range are invalidated in L1P
     * cache. All cache lines in range are invalidated in L1D cache.
     * All cache lines in range are invaliated in L2 cache
     */
    CacheP_block(addr, size, L2IBAR);

    return;
}

void CacheP_wbInv(void *addr, uint32_t size, uint32_t type)
{
    /*
     * Writes back and invalidates the range of memory within the specified
     * starting address and byte count. The range of addresses operated on gets
     * quantized to whole cache lines in each cache. All cache lines within range
     * are invalidated in L1P cache. All cache lines within the range are
     * written back to L2 or external and then invalidated in L1D cache
     * All cache lines within the range are written back to external and then
     * invalidated in L2 cache.
     */
    CacheP_block(addr, size, L2WIBAR);

    return;
}

void CacheP_setSize(const CacheP_Size *size)
{
    uintptr_t   key;

    key = HwiP_disable();

    /* Set size of L2 cache
     * Read back CFG, this stalls CPU until the mode change completes */
    *L2CFG = size->l2Size;
    *L2CFG;

    /* Set size of L1D/L1P cache.
     * Read back CFG, this stalls CPU until the mode change completes */
    *L1DCFG = size->l1dSize;
    *L1PCFG = size->l1pSize;
    *L1DCFG;
    *L1PCFG;

    HwiP_restore(key);

    return;
}

void CacheP_getSize(CacheP_Size *size)
{
    uint32_t    tmpSize;

    /* Return CacheP_L2Size_1024K if register value is equal or greater than
     * CacheP_L2Size_1024K. This is the largest size defined by architecture */
    tmpSize = *L2CFG & L2CFG_MODE_MASK;
    size->l2Size = tmpSize;

    /* Its possible the register value is greater than 4 in which case
     * we simply return 4 since values greater than 4 is equivalent to 4 */
    tmpSize = *L1DCFG;
    size->l1dSize = tmpSize;

    /* Its possible the register value is greater than 4 in which case
     * we simply return 4 since values greater than 4 is equivalent to 4. */
    tmpSize = *L1PCFG;
    size->l1pSize = tmpSize;

    return;
}

void CacheP_setMar(void *baseAddr, uint32_t size, uint32_t value)
{
    uint32_t maxAddr;
    uint32_t firstMar, lastMar;
    uint32_t marNum;
    volatile uint32_t *marBase = MAR;

    /* caculate the maximum address */
    maxAddr = ((uint32_t) baseAddr) + (size - 1U);

    /* range of MAR's that need to be modified */
    firstMar = ((uint32_t) baseAddr) >> 24U;
    lastMar = ((uint32_t) maxAddr) >> 24U;

    /* write back invalidate all cached entries */
    CacheP_wbInvAll(CacheP_TYPE_ALLD);

    /* loop through the number of MAR registers affecting the address range */
    for(marNum = firstMar; marNum <= lastMar; marNum++)
    {
        /* set the MAR registers to the specified value */
        marBase[marNum] = value;
    }

    return;
}

uint32_t CacheP_getMar(void *baseAddr)
{
    uint32_t marNum;
    volatile uint32_t *marBase = MAR;

    /* MAR register number can be obtained by right shifting the
     * base address by 24 bits (upper 8 bits correspond to the MAR number) */
    marNum = ((uint32_t) baseAddr) >> 24U;

    /* return the value of the MAR register */
    return (marBase[marNum]);
}

static void CacheP_all(volatile uint32_t *cacheReg)
{
    uintptr_t   key;

    key = HwiP_disable();

    /* wait for any previous cache operation to complete */
    while(*L2WWC != 0U)
    {
        /* open a window for interrupts */
        HwiP_restore(key);
        key = HwiP_disable();
    }

    /* perform global write back of cache */
    *cacheReg = 1;

    HwiP_restore(key);

    /* wait until cache operation completes */
    while((*cacheReg)!=0U)
    {
        /* Wait */
    }

    return;
}

/*
 *  Wait for the L2 count to complete.  This function needs only to wait
 *  for L2 word count since all block cache operations in BIOS are done
 *  through the L2 registers and all global cache operations must already
 *  wait until the operation completes. Note: Its sufficient to wait
 *  on one of the L2 count registers since all 3 count registers are
 *  mirrors of one another and map to the same bits.
 */
static inline void CacheP_wait(void)
{
    /* Stall CPU while memory system is busy */
    _mfence();

    /* do a 2nd mfence as per single mfence silicon errata */
    _mfence();

    return;
}

static void CacheP_block(void *addr, uint32_t size, volatile uint32_t *barReg)
{
    uint32_t            maxAddr, marNum;
    uint32_t            firstMar, lastMar;
    uint32_t            wordCnt, incCnt, currWordCnt;
    uintptr_t           key;
    uint32_t            alignAddr;
    volatile uint32_t  *bar;
    volatile uint32_t  *wc;
    volatile uint32_t  *marBase;

    /* Get the base address and word count register.
     * wc is one word after bar on c66x cache */
    bar = barReg;
    wc = bar + 1U;

    /* word align the base address */
    alignAddr = ((uint32_t) addr & ~((uint32_t) 3U));

    /* convert from byte to word since cache operation takes words */
    wordCnt = (size + 3U + ((uint32_t) addr - alignAddr)) >> 2U;

    /* determine the increment count */
    if(CacheP_ATOMIC_BLOCKSIZE!=0U)
    {
        incCnt = CacheP_ATOMIC_BLOCKSIZE;
    }
    else
    {
        incCnt = MAXWC;
    }

    /* loop until word count is zero or less */
    while(wordCnt > 0U)
    {
        key = HwiP_disable();

        /* wait for any previous cache operation to complete */
        while(*L2WWC != 0U)
        {
            /* open a window for interrupts */
            HwiP_restore(key);
            key = HwiP_disable();
        }

        /* set the word address and number of words to invalidate */
        *bar = alignAddr;
        currWordCnt = (wordCnt > incCnt) ? incCnt : wordCnt;
        *wc = currWordCnt;

        /*
         *  Silicon errata sprz331a Advisory 14.
         *  Due to c66 silicon bug [see SDOCM00076053] spin with
         *  interrupts disabled here if atomicBlockSize != 0.
         *  CacheP_wait() is doing 2 mfences so no need to spin for 16 NOPs
         */
        if(CacheP_ATOMIC_BLOCKSIZE != 0U)
        {
            CacheP_wait();
        }

        HwiP_restore(key);

        /*
         * reduce word count by the increment count and
         * increase base address by increment count.
         */
        wordCnt -= currWordCnt;
        alignAddr += (currWordCnt * sizeof(uint32_t));
    }

    /* invalidate prefetch buffer if necessary */
    if((barReg == L2WIBAR) || (barReg == L2IBAR))
    {
        /* set the marBase addr */
        marBase = MAR;

        /* caculate the maximum address */
        maxAddr = (uint32_t) addr + (size - 1U);

        /* range of MAR's that need to be modified */
        firstMar = (uint32_t) addr >> 24U;
        lastMar = (uint32_t) maxAddr >> 24U;

        /* loop through the number of MAR registers */
        for(marNum = firstMar; marNum <= lastMar; marNum++)
        {
            /* if prefetch bit enabled, invalidate prefetch buffer */
            if((marBase[marNum] & MAR_PFX_MASK)!=0U)
            {
                CacheP_invPrefetchBuffer();
                break;
            }
        }
    }

    /* Only wait here if atomicBlockSize is 0.
     * When atomicBlockSize != 0, the wait already happens above */
    if(CacheP_ATOMIC_BLOCKSIZE == 0U)
    {
        CacheP_wait();
    }

    return;
}

static void CacheP_invPrefetchBuffer(void)
{
    volatile uint32_t *xpfcmd;

    xpfcmd = XPFCMD;
    xpfcmd[0] = 1;

    return;
}
