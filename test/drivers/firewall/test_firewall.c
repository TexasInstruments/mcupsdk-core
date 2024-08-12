/*
 * Copyright (C) 2022-23 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <inttypes.h>
#include <drivers/soc.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/firewall.h>
#include <unity.h>
#include "ti_drivers_open_close.h"

/*
 *  Pre firewall configuration using SysConfig during Drivers_Open()
 *  +--------------+--------+------------+------------+------------------------------+
 *  | Fwl Id       | Region | Start Addr | End Addr   | Privilege & Permission       |
 *  +--------------+--------+------------+------------+------------------------------+
 *  | 35 (SA2UL    | 1      | 0x40901000 | 0x40901FFF | Everyone                     |
 *  | 17 (MSRAM_5) | 0      | 0x70140000 | 0x7017FFFF | Everyone                     |
 *  | 17 (MSRAM_5) | 1      | 0x70141000 | 0x70141FFF | R50_0 - Allow, R5_0_1 - Deny |
 *  | 17 (MSRAM_5) | 2      | 0x70142000 | 0x70142FFF | R50_0-Deny, R5_0_1-Allow     |
 *  | 17 (MSRAM_5) | 3      | 0x70143000 | 0x70143FFF | Block All                    |
 *  +--------------+--------+------------+------------+------------------------------+
 */

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define TEST_DATA                       (0xABABABAB)
#define TEST_MEM_SA2UL_MMRA_REGION_1    (0x40901000)
#define TEST_MEM_MSRAM_BANK_5_REGION_0  (0x701400FF)
#define TEST_MEM_MSRAM_BANK_5_REGION_1  (0x701410FF)
#define TEST_MEM_MSRAM_BANK_5_REGION_2  (0x701420FF)
#define TEST_MEM_MSRAM_BANK_5_REGION_3  (0x701430FF)
#define TEST_MEM_MSRAM_BLOCKED_REGION_1 (0x701FC000)
#define TEST_MEM_MSRAM_BLOCKED_REGION_2 (0x701FC010)

#define TEST_DATA_ABORT_EXCEP_COUNT     (2U)

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* Test cases */
void test_firewall_api(void *args);
void test_firewall_positive(void *args);
void test_firewall_negative(void *args);

/* Strong declaration of user defined data abort exception */
extern void HwiP_user_data_abort_handler_c(DFSR dfsr,ADFSR adfsr,volatile uint32_t dfar,volatile uint32_t address,volatile uint32_t spsr);

/* client ID that is used to receive messages in Any to Any test */
uint32_t gClientId = 4u;
/* Flag that is used to check data abort exception*/
uint32_t gDataAbortRecived = 0;
/* variable that is used to read data from address */
uint32_t data = 0;
/* Main core that checks the test pass/fail */
uint32_t gMainCoreId = CSL_CORE_ID_R5FSS0_0;

/* semaphore's used to indicate a main core has finished all message exchanges */
SemaphoreP_Object gMainDoneSem;
/* semaphore used to indicate a remote core has finished all message xchange */
SemaphoreP_Object gRemoteDoneSem;

/* Strong definition of user defined data abort exception. This function will be called incase of any data abort */
void HwiP_user_data_abort_handler_c(DFSR dfsr,ADFSR adfsr,volatile uint32_t dfar,volatile uint32_t address,volatile uint32_t spsr){
    gDataAbortRecived++;
}

void test_firewall_msg_handler_main_core(uint32_t remoteCoreId, uint16_t localClientId, uint32_t msgValue, int32_t crcStatus, void *args)
{
    SemaphoreP_post(&gMainDoneSem);
}

void test_firewall_msg_handler_remote_core(uint32_t remoteCoreId, uint16_t localClientId, uint32_t msgValue, int32_t crcStatus, void *args)
{
    SemaphoreP_post(&gRemoteDoneSem);
}

void test_firewall_api(void *args)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t fwlId = 18; // MSRAM BANK 4

    Firewall_Handle handle = NULL;

    Firewall_RegionCfg region =
        {
            .regionIndex = 1,
            .control = FWL_CONTROL_ENABLE | FWL_CONTROL_CACHE_MODE,
            .permissions[0] = 0xC5FFFF,
            .permissions[1] = 0xC5FFFF,
            .permissions[2] = 0xC5FFFF,
            .startAddr = 0x70100000,
            .endAddr = 0x7013FFFF,
        };

    Firewall_Attrs attrs =
        {
            .firewallId = 18,
            .totalRegions = 4,
            .regionInfo = &region,
            .initRegions = 1,
        };

    Firewall_Config config =
        {
            .attrs = NULL,
            .object = NULL,
        };

    DebugP_log("Firewall API Test Started !! \r\n");

    /*  ----------  Firewall API's Positive Sanity Test  ----------- */

    status = Firewall_configureSingleRegion(fwlId, &region);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    /*  ----------  Firewall API's Negative Sanity Test  ----------- */

    status = Firewall_configureSingleRegion(fwlId, (Firewall_RegionCfg *)NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, status);

    region.regionIndex = 12;
    fwlId = 36;
    status = Firewall_configureSingleRegion(fwlId, &region);
    TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, status);

    status = Firewall_configureRegion(handle);
    TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, status);

    handle = (Firewall_Handle)&config;
    status = Firewall_configureRegion(handle);
    TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, status);

    config.attrs = &attrs;
    attrs.firewallId = 36;
    attrs.totalRegions = 17;
    status = Firewall_configureRegion(handle);
    TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, status);
}

void test_firewall_positive(void *args)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t gRemoteCoreId = (uint32_t)args;
    uint32_t msgValue = 0;

    DebugP_log("Firewall Positive Test Started !! \r\n");

    /*  ----------  Testing positive scenerios  ----------- */

    /*Test MSRAM Region 0 Access */
    CSL_REG32_WR((uint32_t *)TEST_MEM_MSRAM_BANK_5_REGION_0, (uint32_t)TEST_DATA);
    CacheP_inv((uint32_t *)TEST_MEM_MSRAM_BANK_5_REGION_0, CacheP_CACHELINE_ALIGNMENT, CacheP_TYPE_ALL);
    data = CSL_REG32_RD((uint32_t *)TEST_MEM_MSRAM_BANK_5_REGION_0);
    if (TEST_DATA != data)
    {
        status = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    data = 0;
    /*Test MSRAM Region 1 Access */
    CSL_REG32_WR((uint32_t *)TEST_MEM_MSRAM_BANK_5_REGION_1, (uint32_t)TEST_DATA);
    CacheP_inv((uint32_t *)TEST_MEM_MSRAM_BANK_5_REGION_1, CacheP_CACHELINE_ALIGNMENT, CacheP_TYPE_ALL);
    data = CSL_REG32_RD((uint32_t *)TEST_MEM_MSRAM_BANK_5_REGION_1);
    if (TEST_DATA != data)
    {
        status = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    /* send message to other core's, wait for message to be put in HW FIFO */
    status = IpcNotify_sendMsg(gRemoteCoreId, gClientId, msgValue, 1);
    DebugP_assert(status == SystemP_SUCCESS);

    /* wait for messages from remote core */
    SemaphoreP_pend(&gMainDoneSem, SystemP_WAIT_FOREVER);
}

void test_firewall_negative(void *args)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t gRemoteCoreId = (uint32_t)args;
    uint32_t msgValue = 0;

    DebugP_log("Firewall Negative Test Started !! \r\n");

    /*  ----------  Testing negative scenerios  ----------- */

    /* Bank 5 Firewall only configured on HS devices */
    if(SOC_isHsDevice())
    {
        /*Test MSRAM Region 1 Access */
        data = CSL_REG32_RD((uint32_t *)TEST_MEM_MSRAM_BANK_5_REGION_2);
        /*Test MSRAM Region 3 Access */
        data = CSL_REG32_RD((uint32_t *)TEST_MEM_MSRAM_BANK_5_REGION_3);
    }
    else
    {
        /* Checking MSRAM blocked region by sysfw on GP devices */
        data = CSL_REG32_RD((uint32_t *)TEST_MEM_MSRAM_BLOCKED_REGION_1);
        /*Test MSRAM Region 3 Access */
        data = CSL_REG32_RD((uint32_t *)TEST_MEM_MSRAM_BLOCKED_REGION_2);

    }

    if (gDataAbortRecived != TEST_DATA_ABORT_EXCEP_COUNT)
    {
        status = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    /* send message to other core's, wait for message to be put in HW FIFO */
    status = IpcNotify_sendMsg(gRemoteCoreId, gClientId, msgValue, 1);
    DebugP_assert(status == SystemP_SUCCESS);

    /* wait for messages from remote core */
    SemaphoreP_pend(&gMainDoneSem, SystemP_WAIT_FOREVER);
}

/* This code executes on remote core, i.e not on main core */
void test_firewall_remote_core_start()
{
    int32_t status = SystemP_SUCCESS;
    uint32_t msgValue = 0;

    SemaphoreP_constructBinary(&gRemoteDoneSem, 0);

    /* register a handler to receive messages */
    status = IpcNotify_registerClient(gClientId, test_firewall_msg_handler_remote_core, NULL);
    DebugP_assert(status == SystemP_SUCCESS);

    /* wait for all cores to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    /* wait for all messages from main core */
    SemaphoreP_pend(&gRemoteDoneSem, SystemP_WAIT_FOREVER);

    /*  ----------  Testing positive scenerios  ----------- */

    /*Test MSRAM Region 0 Access */
    CSL_REG32_WR((uint32_t *)TEST_MEM_MSRAM_BANK_5_REGION_0, (uint32_t)TEST_DATA);
    CacheP_inv((uint32_t *)TEST_MEM_MSRAM_BANK_5_REGION_0, CacheP_CACHELINE_ALIGNMENT, CacheP_TYPE_ALL);
    data = CSL_REG32_RD((uint32_t *)TEST_MEM_MSRAM_BANK_5_REGION_0);
    if (TEST_DATA != data)
    {
        status = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    data = 0;
    /*Test MSRAM Region 1 Access */
    CSL_REG32_WR((uint32_t *)TEST_MEM_MSRAM_BANK_5_REGION_2, (uint32_t)TEST_DATA);
    CacheP_inv((uint32_t *)TEST_MEM_MSRAM_BANK_5_REGION_2, CacheP_CACHELINE_ALIGNMENT, CacheP_TYPE_ALL);
    data = CSL_REG32_RD((uint32_t *)TEST_MEM_MSRAM_BANK_5_REGION_2);
    if (TEST_DATA != data)
    {
        status = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    /* send message to main core's, wait for message to be put in HW FIFO */
    status = IpcNotify_sendMsg(gMainCoreId, gClientId, msgValue, 1);
    DebugP_assert(status == SystemP_SUCCESS);

    /* wait for all messages from main core */
    SemaphoreP_pend(&gRemoteDoneSem, SystemP_WAIT_FOREVER);

    /*  ----------  Testing negative scenerios  ----------- */

    /* Bank 5 Firewall only configured on HS devices */
    if(SOC_isHsDevice())
    {
        /*Test MSRAM Region 1 Access */
        data = CSL_REG32_RD((uint32_t *)TEST_MEM_MSRAM_BANK_5_REGION_1);
        /*Test MSRAM Region 3 Access */
        data = CSL_REG32_RD((uint32_t *)TEST_MEM_MSRAM_BANK_5_REGION_3);
    }
    else
    {
        /* Checking MSRAM blocked region by sysfw on GP devices */
        data = CSL_REG32_RD((uint32_t *)TEST_MEM_MSRAM_BLOCKED_REGION_1);
        /*Test MSRAM Region 3 Access */
        data = CSL_REG32_RD((uint32_t *)TEST_MEM_MSRAM_BLOCKED_REGION_2);

    }

    if (gDataAbortRecived != TEST_DATA_ABORT_EXCEP_COUNT)
    {
        status = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    /* send message to other core's, wait for message to be put in HW FIFO */
    status = IpcNotify_sendMsg(gMainCoreId, gClientId, msgValue, 1);
    DebugP_assert(status == SystemP_SUCCESS);
}

/* This code executes on main core, i.e not on remote core */
void test_firewall_main_core_start()
{
    int32_t status;

    UNITY_BEGIN();

    /* create done semaphore */
    SemaphoreP_constructBinary(&gMainDoneSem, 0);

    /* register a handler to receive ACK messages, also pass semaphore handle as a arg */
    status = IpcNotify_registerClient(gClientId, test_firewall_msg_handler_main_core, NULL);
    DebugP_assert(status == SystemP_SUCCESS);

    /* wait for all cores to be ready */
    IpcNotify_syncAll(SystemP_WAIT_FOREVER);

    RUN_TEST(test_firewall_api, 9478, NULL);
    RUN_TEST(test_firewall_positive, 9479, (void *)CSL_CORE_ID_R5FSS0_1);
    RUN_TEST(test_firewall_negative, 9619, (void *)CSL_CORE_ID_R5FSS0_1);

    UNITY_END();
}

void test_main(void *args)
{
    Drivers_open();

    if (IpcNotify_getSelfCoreId() == gMainCoreId)
    {
        test_firewall_main_core_start();
    }
    else
    {
        test_firewall_remote_core_start();
    }

    /* We dont close drivers to let the UART driver remain open and flush any pending messages to console */
    // Drivers_close();
}

void setUp(void)
{
}

void tearDown(void)
{
}
