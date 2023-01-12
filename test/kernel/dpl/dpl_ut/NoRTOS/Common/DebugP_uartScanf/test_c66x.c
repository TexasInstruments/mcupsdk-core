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

#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <unity.h>
#include <drivers/hw_include/cslr_soc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Allocate more than cache size */
#define ARR_SIZE                        (80U * 1024U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void test_cache(void *args);

static void test_setvalue(volatile uint32_t *addr, uint32_t value, uint32_t length);
static uint32_t test_get_l2_sys_addr(uint32_t localAddr);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static uint8_t gCacheTestLine[ARR_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_c66x(void)
{
    RUN_TEST(test_cache, 893, NULL);

    return;
}

static void test_cache(void *args)
{
    uint32_t            expected, type, marValue, temp;
    volatile uint32_t  *testBuf;
    volatile uint32_t  *sysPtr;
    volatile uint32_t  *l3Ptr;

    testBuf = (volatile uint32_t *) &gCacheTestLine[0];
    sysPtr  = (volatile uint32_t *) test_get_l2_sys_addr((uint32_t) testBuf);
    l3Ptr   = (volatile uint32_t *) CSL_DSP_L3_U_BASE;

    /*
     * Various Cache API Tests
     */
    /* Check default - as set in Syscfg */
    type = CacheP_getEnabled();
    TEST_ASSERT_EQUAL_UINT32(CacheP_TYPE_L1P | CacheP_TYPE_L1D | CacheP_TYPE_L2D, type);
    /* Invalidate and Writeback all */
    CacheP_wbAll(CacheP_TYPE_L1D);
    CacheP_wbAll(CacheP_TYPE_ALL);
    CacheP_wbInvAll(CacheP_TYPE_L1P);
    CacheP_wbInvAll(CacheP_TYPE_L1D);
    CacheP_wbInvAll(CacheP_TYPE_ALL);
    /* Disable cache and check */
    CacheP_disable(CacheP_TYPE_L1P);
    CacheP_disable(CacheP_TYPE_L1D);
    type = CacheP_getEnabled();
    TEST_ASSERT_EQUAL_UINT32(CacheP_TYPE_L2D, type);    /* L2 is never disabled - only MAR controls the cache settings */
    /* Enable cache again and check */
    CacheP_enable(CacheP_TYPE_L1P);
    CacheP_enable(CacheP_TYPE_L1D);
    CacheP_enable(CacheP_TYPE_L2);
    CacheP_enable(CacheP_TYPE_L1P | CacheP_TYPE_L1D | CacheP_TYPE_L2);
    type = CacheP_getEnabled();
    TEST_ASSERT_EQUAL_UINT32(CacheP_TYPE_L1P | CacheP_TYPE_L1D | CacheP_TYPE_L2D, type);

    /*
     * Test to check if a L2 data is getting cached in L1D as expected
     */
    /* Invalidate Cache before start of test */
    CacheP_wbInvAll(CacheP_TYPE_ALLD);

    /* Test read allocate */
    test_setvalue(testBuf, 0xA5A5A5A5U, ARR_SIZE);

    /* Test Writeback: write to local address and check system address */
    test_setvalue(testBuf, 0x55555555U, ARR_SIZE);
    CacheP_wb((void *) testBuf, ARR_SIZE, CacheP_TYPE_ALLD);
    expected = 0x55555555U;
    TEST_ASSERT_EACH_EQUAL_UINT32(expected, sysPtr, (ARR_SIZE / sizeof(uint32_t)));

    /* Test invalidate: write to system address and check local address */
    test_setvalue(sysPtr, 0xAAAAAAAAU, ARR_SIZE);
    CacheP_inv((void *) testBuf, ARR_SIZE, CacheP_TYPE_ALLD);
    expected = 0xAAAAAAAAU;
    TEST_ASSERT_EACH_EQUAL_UINT32(expected, testBuf, (ARR_SIZE / sizeof(uint32_t)));

    /*
     * Test to check if a L3 data is getting cached in L1D/L2 as expected
     */
    /* Invalidate Cache before start of test */
    CacheP_wbInvAll(CacheP_TYPE_ALLD);
    temp = *l3Ptr;              /* Read allocate */
    *l3Ptr = 0xA5A5A5A5U;       /* Check through CCS if this is cached */
    CacheP_wb((void *)l3Ptr, 4, CacheP_TYPE_ALLD);
    /* Check through CCS if the data is reflected in L3 */
    *l3Ptr = temp;              /* To supress warning */

    /*
     * Test MAR: Through SysCfg DSS M3 is cached. Check that
     */
    marValue = CacheP_getMar((void *) CSL_DSP_L3_U_BASE);
    TEST_ASSERT_EQUAL_UINT32(CacheP_MarMask_PC, marValue);

    return;
}

static void test_setvalue(volatile uint32_t *addr, uint32_t value, uint32_t length)
{
    uint32_t i;
    volatile uint32_t temp;

    for(i = 0U; i < (length/ sizeof (uint32_t)); i++)
    {
        addr[i] = value;
        temp = addr[i]; /* Read to allocate */
    }

    return;
}

static uint32_t test_get_l2_sys_addr(uint32_t localAddr)
{
    uint32_t sysAddr;

    sysAddr = (localAddr & (CSL_DSP_L2_U_BASE - 1)) | CSL_DSS_L2_U_BASE;

    return (sysAddr);
}
