/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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

#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include <drivers/esm.h>
#include <drivers/ddr.h>

/*******************************************************************************
This example simulates a 1b and 2b ECC error in DDR ECC enabled region.
The error events are routed to the Safety domain M4F through the MCU ESM.
    * Main ESM has the error events corresponding to DDR 1b ECC and DDR 2b ECC
      enabled
        - 1b ECC error --> Low priority, Enable error pin
        - 2b ECC error --> High priority, Enable error pin
    * MCU ESM has the error events corresponding to MAIN ESM high priority
      error event and MAIN ESM low priorty error events enabled
        - MAIN ESM low priority error - Low priority, error pin not enabled
        - MAIN ESM high priority error - High priority, error pin enabled

The M4 on receiving the error event will signal R5 via IPC to take the
required action.

The single bit ECC errors will be corrected by the DDR, but the dual bit ECC
errors is only detected and is not corrected.
User could do a warm reset or do necessaryy corrective action to resolve the
dual bit ECC errors
*******************************************************************************/

#define DDR_START_ADDR (0x80000000u)

/* Memory block for which ECC is calculated (256 Bytes) */
#define DDR_EMIF_ECC_MEM_BLOCK_SIZE       0x100
/* ECC data size per block (32 Bytes) */
#define DDR_EMIF_ECC_DATA_SIZE_PER_BLOCK  0x20

#define DDR_ECC_TEST_ADDR               (DDR_START_ADDR + DDR_ECC_REGION0_START \
                                        + DDR_EMIF_ECC_MEM_BLOCK_SIZE)

/* esm_lvl_event for DDR single error */
#define DDR_ECC_AGGR0_SEC_ERR_EVENT    (CSLR_ESM0_ESM_LVL_EVENT_DDR16SS0_DDRSS_DRAM_ECC_CORR_ERR_LVL_0)
/* esm_lvl_event for DDR double error */
#define DDR_ECC_AGGR0_DED_ERR_EVENT    (CSLR_ESM0_ESM_LVL_EVENT_DDR16SS0_DDRSS_DRAM_ECC_UNCORR_ERR_LVL_0)

#define ESM_HIGH_PRIORITY_INTR_NUM  (168U)

volatile uint32_t gSecTestPass;
volatile uint32_t gDedTestPass;

volatile uint32_t *gTest_Addr = NULL;
volatile uint32_t testVal;

uint32_t gClientId1b = 4;
uint32_t gClientId2b = 5;

volatile CSL_esmRegs* esmMainRegs __attribute__ ((used)) = (CSL_esmRegs *)0x420000;

volatile CSL_esmRegs* esmMcuRegs __attribute__ ((used)) = (CSL_esmRegs *)0x4100000;

uintptr_t DDRGetTranslatedAddress (uintptr_t memAddress)
{
    uint32_t memIndex;
    uintptr_t translatedMemAddr;

    memIndex = (memAddress - DDR_START_ADDR)/DDR_EMIF_ECC_MEM_BLOCK_SIZE;

    if ((memIndex & 0x1u) == 0)
    {
        translatedMemAddr = memAddress + ((memIndex)*DDR_EMIF_ECC_DATA_SIZE_PER_BLOCK);
    }
    else
    {
        translatedMemAddr = memAddress + ((memIndex+1u)*DDR_EMIF_ECC_DATA_SIZE_PER_BLOCK);
    }
    return  translatedMemAddr;
}

/* Handler for single bit ECC error */
void DDR_secHandler (void *args)
{
    int32_t status = SystemP_SUCCESS;
    DDR_ECCErrorInfo ECCErrorInfo;

    /* Read ECC registers and double check address */
    status = DDR_getECCErrorInfo (&ECCErrorInfo);

    if (status == SystemP_SUCCESS)
    {
        if ((ECCErrorInfo.singlebitErrorAddress & (~0x7u))
            == ((DDR_ECC_TEST_ADDR- DDR_START_ADDR) & (~0x7u)))
        {
            gSecTestPass = TRUE;
        }

        /* Clear Specific ECC error */
        status = DDR_clearECCError (DDR_ECC_1B_ERROR);
    }
}

/* Handler for double bit ECC error */
void DDR_dedHandler (void *args)
{
    int32_t status = SystemP_SUCCESS;
    DDR_ECCErrorInfo ECCErrorInfo;
    volatile uint32_t *translatedMemPtr;

    status = DDR_getECCErrorInfo (&ECCErrorInfo);

    if (status == SystemP_SUCCESS)
    {
        if ((ECCErrorInfo.doublebitErrorAddress & (~0x7u))
            == ((DDR_ECC_TEST_ADDR - DDR_START_ADDR) & (~0x7u)))
        {
            gDedTestPass = TRUE;

            /* This section corrects the ECC error simulated */
            /* In a real application the user must take necessary corrective action */
            /******************************************************************/

            /* Disable Inline ECC */
            DDR_enableInlineECC(0);

            translatedMemPtr = (volatile uint32_t *)(DDRGetTranslatedAddress ((uintptr_t)gTest_Addr));

            /* Now replace location with original value as 2b errors are not corrected */
            *(translatedMemPtr) = testVal;

            /* Write back any pending writes */
            CacheP_wbInv ((void *)translatedMemPtr, 4, CacheP_TYPE_ALL);

            /* Enable back ECC */
            DDR_enableInlineECC (1);

            /******************************************************************/
        }

        /* Clear specific error */
        status = DDR_clearECCError (DDR_ECC_2B_ERROR);
    }

}

int32_t DDR_secErrTest (void)
{
    int32_t status = SystemP_SUCCESS;
    volatile uint32_t testVal2;
    volatile uint32_t *translatedMemPtr;
    uint32_t waitCount = 0;

    gSecTestPass = FALSE;

    /* Clear any residual ECC errors */
    DDR_clearECCError (DDR_ECC_ERR_ALL);

    /* Inject error */
    gTest_Addr = (uint32_t *) (DDR_ECC_TEST_ADDR);

    /* Write back any pending writes */
    CacheP_wbInv ((void *)gTest_Addr, 4, CacheP_TYPE_ALL);

    /* Read value from test location */
    testVal = gTest_Addr[0];

    /* Flip one bit to introduce error */
    testVal2       = testVal ^ 0x00010000u;

    /* Calculate translated address */
    translatedMemPtr = (volatile uint32_t *)(DDRGetTranslatedAddress ((uintptr_t)gTest_Addr));

    /* Generating a 1b ECC error */
    /* NOTE: The following section should NOT be useed in actual application */
    /* ================================================================================ */

    /* Temporarily disable ECC */
    DDR_enableInlineECC (0);

    /* Now corrupt the value */
    *(translatedMemPtr) = testVal2;
    CacheP_wbInv ((void *)translatedMemPtr, 4, CacheP_TYPE_ALL);

    /* Enable back ECC */
    DDR_enableInlineECC (1);

    /* ================================================================================ */

    /* Invalidate cache */
    CacheP_inv ((void *)gTest_Addr, 4, CacheP_TYPE_ALL);

    /* Read value to trigger error */
    testVal2 = gTest_Addr[0];

    DebugP_log ("Waiting on Single bit Error Correction Interrupt...\r\n");

    while ((gSecTestPass == FALSE) && (waitCount++ < 100u))
    {
        ClockP_usleep(10);
    }

    if (gSecTestPass == TRUE)
    {
        DebugP_log ("1b ECC error detected and corrected\r\n");
        status = SystemP_SUCCESS;
    }
    else
    {
        DebugP_logError ("1b Inline ECC test failed timedout ...\r\n");
    }

    /* Restore original value */
    gTest_Addr[0] = testVal;

    /* Write back any pending writes */
    CacheP_wbInv ((void *)gTest_Addr, 4, CacheP_TYPE_ALL);

    return status;
}

int32_t DDR_dedErrTest (void)
{
    int32_t status = SystemP_SUCCESS;
    volatile uint32_t testVal;
    volatile uint32_t testVal2;
    volatile uint32_t *translatedMemPtr;
    uint32_t waitCount = 0;

    gDedTestPass = FALSE;

    /* Clear any residual ECC errors */
    DDR_clearECCError (DDR_ECC_ERR_ALL);

    gTest_Addr = (uint32_t *) (DDR_ECC_TEST_ADDR);

    CacheP_wbInv ((void *)gTest_Addr, 4, CacheP_TYPE_ALL);
    /* Read reference value */
    testVal       = gTest_Addr[0];
    /* flip 2 bits */
    testVal2       = testVal ^ 0x00101000u;
    /* Calculate translated address */
    translatedMemPtr = (volatile uint32_t *)(DDRGetTranslatedAddress ((uintptr_t)gTest_Addr));

    /* Generating a 2b ECC error */
    /* NOTE: The following section should NOT be useed in actual application */
    /* ================================================================================ */

    /* Temporarily disable ECC */
    DDR_enableInlineECC (0);

    /* Now corrupt the value */
    *(translatedMemPtr) = testVal2;

    /* Make sure the values are written back */
    CacheP_wbInv ((void *)translatedMemPtr, 4, CacheP_TYPE_ALL);

    /* Enable back ECC */
    DDR_enableInlineECC (1);

    /* ================================================================================ */

    /* Invalidate cache */
    CacheP_inv ((void *)gTest_Addr, 4, CacheP_TYPE_ALL);

    /* Read value to trigger error */
    testVal2 = gTest_Addr[0];

    DebugP_log ("Waiting on Dual bit error detection Interrupt...\r\n");

    while ((gDedTestPass == FALSE) && (waitCount++ < 100u))
    {
        ClockP_usleep (100);
    }

    if (gDedTestPass == TRUE)
    {
        DebugP_log ("2b ECC error detected\r\n");
        status = SystemP_SUCCESS;
    }
    else
    {
        DebugP_logError ("2b Inline ECC Test failed timedout ...\r\n");
        status = SystemP_FAILURE;
    }

    /* Restore original value */
    gTest_Addr[0] = testVal;

    return status;
}

void ipc_notify_handler1bECC (uint32_t remoteCoreId, uint16_t localClientId,
                                uint32_t msgValue, int32_t crcStatus, void *args)
{
    if (remoteCoreId == CSL_CORE_ID_M4FSS0_0)
    {
        if (localClientId == gClientId1b)
        {
            DDR_secHandler (NULL);
        }
    }
}

void ipc_notify_handler2bECC (uint32_t remoteCoreId, uint16_t localClientId,
                                uint32_t msgValue, int32_t crcStatus, void *args)
{
    if (remoteCoreId == CSL_CORE_ID_M4FSS0_0)
    {
        if (localClientId == gClientId2b)
        {
            DDR_dedHandler (NULL);
        }
    }
}

void ddr_ecc_test_main (void *args)
{
    int32_t status;

    /* Open drivers to open the UART driver for console */
    Drivers_open ();
    Board_driversOpen ();

    gSecTestPass = FALSE;
    gDedTestPass = FALSE;

    status = IpcNotify_registerClient(gClientId1b, ipc_notify_handler1bECC, NULL);

    DebugP_assert (status == SystemP_SUCCESS);

    status = IpcNotify_registerClient(gClientId2b, ipc_notify_handler2bECC, NULL);

    DebugP_assert (status == SystemP_SUCCESS);

    HwiP_enableInt (ESM_HIGH_PRIORITY_INTR_NUM);

    /* Wait for all cores to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    status = DDR_secErrTest ();

    if (status == SystemP_SUCCESS)
    {
        status = DDR_dedErrTest ();
    }

    if (status == SystemP_SUCCESS)
    {
        DebugP_log ("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_logError ("Some tests have failed\r\n");
    }

    Board_driversClose ();
    Drivers_close ();
}
