/*
 * Copyright (C) 2021 Texas Instruments Incorporated
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unity.h>
#include <drivers/sciclient.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include "ti_drivers_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* Testcases */
static void test_sciclient_version(void *args);
static void test_sciclient_timeout(void *args);
static void test_sciclient_invalid_params(void *args);
static void test_sciclient_rm_ir_output(void *args);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    UNITY_BEGIN();

    RUN_TEST(test_sciclient_version,  171, NULL);
    RUN_TEST(test_sciclient_timeout,  172, NULL);
    RUN_TEST(test_sciclient_invalid_params, 173, NULL);
    RUN_TEST(test_sciclient_rm_ir_output, 4048, NULL);

    UNITY_END();
    Drivers_close();

    return;
}

/*
 * Unity framework required functions
 */
void setUp(void)
{
}

void tearDown(void)
{
}

/*
 * Testcases
 */

static void test_sciclient_version(void *args)
{
    int32_t retVal = SystemP_SUCCESS;
    struct tisci_msg_version_req request;
    const Sciclient_ReqPrm_t      reqPrm =
    {
        TISCI_MSG_VERSION,
        TISCI_MSG_FLAG_AOP,
        (uint8_t *) &request,
        sizeof(request),
        SystemP_WAIT_FOREVER
    };

    struct tisci_msg_version_resp response;
    Sciclient_RespPrm_t           respPrm =
    {
        0,
        (uint8_t *) &response,
        sizeof (response)
    };

    retVal = Sciclient_service(&reqPrm, &respPrm);

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
    TEST_ASSERT_EQUAL_UINT32(TISCI_MSG_FLAG_ACK, respPrm.flags);

    DebugP_log("DMSC Firmware Version %s\r\nFirmware revision 0x%x\r\nABI revision %d.%d\r\n",
                            (char *) response.str,
                            response.version,
                            response.abi_major,
                            response.abi_minor);
}

static void test_sciclient_timeout(void *args)
{
    int32_t retVal = SystemP_SUCCESS;

    struct tisci_msg_version_req request;
    const Sciclient_ReqPrm_t      reqPrm =
    {
        TISCI_MSG_VERSION,
        TISCI_MSG_FLAG_AOP,
        (uint8_t *) &request,
        sizeof(request),
        (uint32_t)0x0A
    };

    struct tisci_msg_version_resp response;
    Sciclient_RespPrm_t           respPrm =
    {
        0,
        (uint8_t *) &response,
        sizeof (response)
    };

    retVal = Sciclient_service(&reqPrm, &respPrm);

    TEST_ASSERT_EQUAL_INT32(SystemP_TIMEOUT, retVal);
}

static void test_sciclient_invalid_params(void *args)
{
    int32_t retVal = SystemP_SUCCESS;

    struct tisci_msg_version_req request;
    const Sciclient_ReqPrm_t reqPrm_badTxSize =
    {
        TISCI_MSG_VERSION,
        TISCI_MSG_FLAG_AOP,
        (uint8_t*) &request,
        100,
        SystemP_WAIT_FOREVER
    };

    const Sciclient_ReqPrm_t reqPrm_badPayload =
    {
        TISCI_MSG_VERSION,
        TISCI_MSG_FLAG_AOP,
        NULL,
        0,
        SystemP_WAIT_FOREVER
    };

    const Sciclient_ReqPrm_t reqPrm_good =
    {
        TISCI_MSG_VERSION,
        TISCI_MSG_FLAG_AOP,
        (uint8_t*)&request,
        0,
        SystemP_WAIT_FOREVER
    };

    struct tisci_msg_version_resp response;
    Sciclient_RespPrm_t respPrm_badRxsize =
    {
        0,
        (uint8_t *) &response,
        100
    };

    Sciclient_RespPrm_t respPrm_good =
    {
        0,
        (uint8_t *) &response,
        sizeof (response)
    };

    retVal = Sciclient_service(NULL, &respPrm_good);
    TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, retVal);

    retVal = Sciclient_service(&reqPrm_badTxSize, &respPrm_good);
    TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, retVal);

    retVal = Sciclient_service(&reqPrm_badPayload, &respPrm_good);
    TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, retVal);

    retVal = Sciclient_service(&reqPrm_good, NULL);
    TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, retVal);

    retVal = Sciclient_service(&reqPrm_good, &respPrm_badRxsize);
    TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, retVal);
}

static void test_sciclient_rm_ir_output(void *args)
{
    int32_t retVal = SystemP_SUCCESS;
    uint16_t irId, outp;
    uint32_t i;
#if defined (SOC_AM62X)
    uint16_t validIrDevIds[4] = {
        TISCI_DEV_CMP_EVENT_INTROUTER0,
        TISCI_DEV_MAIN_GPIOMUX_INTROUTER0,
        TISCI_DEV_WKUP_MCU_GPIOMUX_INTROUTER0,
        TISCI_DEV_TIMESYNC_EVENT_ROUTER0,
    };
#else
    uint16_t validIrDevIds[4] = {
        TISCI_DEV_CMP_EVENT_INTROUTER0,
        TISCI_DEV_MAIN_GPIOMUX_INTROUTER0,
        TISCI_DEV_MCU_MCU_GPIOMUX_INTROUTER0,
        TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    };
#endif
    uint16_t freeOutp[4] = { 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF };
    uint16_t maxCheckOutp = 50;

    /* First check with invalid IR instance, valid output number */
    irId = 0xFFFF;
    outp = 0; /* All IRs will have atlease one output :) */

    retVal = Sciclient_rmIrOutpIsFree(irId, outp);
    TEST_ASSERT_NOT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    /* Now check for invalid output numbers in all valid IRs */
    outp = 0xFFFF; /* No IR will have 65535 outputs !! */
    for(i = 0; i < (sizeof(validIrDevIds)/sizeof(validIrDevIds[0])); i++)
    {
        retVal = Sciclient_rmIrOutpIsFree(validIrDevIds[i], outp);
        TEST_ASSERT_NOT_EQUAL_INT32(SystemP_SUCCESS, retVal);
    }

    /* Now check for valid values. */
    /* Assume around 50 outputs for each router, exit the loop after finding first free outp */
    for(i = 0; i < 4; i++)
    {
        for(outp = 0; outp < maxCheckOutp; outp++)
        {
            retVal = Sciclient_rmIrOutpIsFree(validIrDevIds[i], outp);
            if(retVal == SystemP_SUCCESS)
            {
                freeOutp[i] = outp;
                DebugP_log("Found free output for IR %d at %d !\r\n", validIrDevIds[i], outp);
                break;
            }
        }
    }

    for(i = 0; i < 4; i++)
    {
        TEST_ASSERT_LESS_THAN_UINT16(maxCheckOutp, freeOutp[i]);
    }
}