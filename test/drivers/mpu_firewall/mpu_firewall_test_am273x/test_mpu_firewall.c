/*
 * Copyright (C) 2023 Texas Instruments Incorporated
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

#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <unity.h>
#include <drivers/mpu_firewall.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/hw_include/cslr_soc.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* ========================================================================== */
/*                          Test Functions Declaration                        */
/* ========================================================================== */

void test_MPU_FIREWALL_getRegion(void* args);
void test_region_bound_check(void* args);
void test_region_access(void* args);

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define JIRA_ID_TEST_MPU_FIREWALL_GETREGION           (11282U)
#define JIRA_ID_TEST_REGION_BOUND_CHECK               (11283U)
#define JIRA_ID_TEST_REGION_ACCESS                    (11284U)
#define CSL_HSM_SEC_ROM_U_BASE                  (0x20010000ul)
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

uint8_t gflag = 0 ;
/* Flag that is used to check data abort exception*/
uint32_t gDataAbortReceived = 0;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* Strong definition of user defined data abort exception. This function will be called incase of any data abort */
void HwiP_user_data_abort_handler_c(DFSR dfsr,ADFSR adfsr,volatile uint32_t dfar,volatile uint32_t address,volatile uint32_t spsr)
{
    gDataAbortReceived++;
}

void test_main(void *args)
{
    Drivers_open();
    Board_driversOpen();

    UNITY_BEGIN();

    RUN_TEST(test_MPU_FIREWALL_getRegion, JIRA_ID_TEST_MPU_FIREWALL_GETREGION, NULL);
    RUN_TEST(test_region_bound_check, JIRA_ID_TEST_REGION_BOUND_CHECK, NULL);
    RUN_TEST(test_region_access, JIRA_ID_TEST_REGION_ACCESS,NULL);

    UNITY_END();

    Board_driversClose();
    Drivers_close();

    return;
}

/* Unity framework required information */
void setUp(void)
{
}

void tearDown(void)
{
}

void test_MPU_FIREWALL_getRegion(void* args)
{

    Fwl_Return_t status;
    int32_t testStatus = SystemP_FAILURE ;
    MPU_FIREWALL_RegionParams mpuParams;

    /* Checks MPU_FIREWALL_getRegion API for MSS_PCRA fwl and region 0 */
    mpuParams.id = CSL_FW_MSS_PCRA_ID;
    mpuParams.regionNumber = 0U;
    status = MPU_FIREWALL_getRegion(&mpuParams);
    if(status == FWL_DRV_RETURN_SUCCESS)
    {
        if(mpuParams.startAddress == (uint32_t)0x2F7A800 && mpuParams.endAddress == (uint32_t)0x2F7ABFF)
        {
            testStatus = SystemP_SUCCESS;
        }
        else
        {
            testStatus = SystemP_FAILURE ;
        }

        if(testStatus == SystemP_SUCCESS)
        {
            /* 0000 0000 0010 == 0x2
            PRIV_ID_HSMM4 -> 1U*/
            if(mpuParams.aidConfig == (uint32_t) 0x2)
            {
                testStatus = SystemP_SUCCESS;
            }
            else
            {
                testStatus = SystemP_FAILURE;
            }
        }
    }
    else
    {
        testStatus = SystemP_FAILURE ;
    }

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
}

void test_region_bound_check(void* args)
{
    Fwl_Return_t status_1, status_2;
    int32_t testStatus = SystemP_FAILURE ;
    MPU_FIREWALL_RegionParams mpuParams;

    /* Checks MPU_FIREWALL_getRegion API for DTHE_SLV fwl and region 0 */
    mpuParams.id = CSL_FW_HSM_DTHE_ID;
    mpuParams.regionNumber = CSL_FW_HSM_DTHE_NUM_REGION + 1U;
    status_1 = MPU_FIREWALL_getRegion(&mpuParams);

    mpuParams.regionNumber = -1U;
    status_2 = MPU_FIREWALL_getRegion(&mpuParams);

    if(status_1 == status_2 && status_1 == FWL_DRV_RETURN_FAILURE)
    {
        testStatus = SystemP_SUCCESS;
    }
    else{
        testStatus = SystemP_FAILURE;
    }

    TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, testStatus);
}

__attribute__((optnone)) void test_region_access(void* args)
{
    int32_t testStatus = SystemP_FAILURE ;

    /* secure context of HSM_PKA is read/write protected */
    /*Read HSM_PKA */
    CSL_REG32_RD((uint32_t *) CSL_HSM_PKA_U_BASE);
    /*Write HSM_PKA */
    CSL_REG32_WR((uint32_t *) 0xCE011FF0, 0x2U);

    /*Read HSM_TRNG */
    CSL_REG32_RD((uint32_t *) CSL_HSM_TRNG_U_BASE);
    /*Write HSM_TRNG */
    CSL_REG32_WR((uint32_t *) CSL_HSM_TRNG_U_BASE + 0x70, 0xF00);

    /*Read HSM_PKA_RAM */
    CSL_REG32_RD((uint32_t *) CSL_HSM_PKA_RAM_U_BASE);
    /*Write HSM_PKA_RAM */
    CSL_REG32_WR((uint32_t *) 0xCE014004U, 0x2U);

    /*Read TOP_EFUSE_FARM Instruction register */
    CSL_REG32_RD((uint32_t *) CSL_TOP_EFUSE_FARM_U_BASE);

    /*Read MPU_HSM Revision register */
    CSL_REG32_RD((uint32_t *) CSL_MPU_HSM_BASE);

    /*Write MPU_HSM PROGRAMMABLE_1_START_ADDRESS register */
    CSL_REG32_WR((uint32_t *) CSL_MPU_HSM_BASE + 0x200U, 0x2U);

    /*Read MPU_DTHE Revision register */
    CSL_REG32_RD((uint32_t *) CSL_MPU_DTHE_U_BASE);

    /*Write MPU_DTHE PROGRAMMABLE_1_START_ADDRESS register */
    CSL_REG32_WR((uint32_t *) CSL_MPU_DTHE_U_BASE + 0x200U, 0x2U);

    /*Read HSM ROM  */
    CSL_REG32_RD((uint32_t *) CSL_HSM_ROM_U_BASE);

    /*Read HSM SEC ROM  */
    CSL_REG32_RD((uint32_t *) CSL_HSM_SEC_ROM_U_BASE);

    /*Read HSM RAM */
    CSL_REG32_RD((uint32_t *) CSL_HSM_RAM_U_BASE);

    /*Write HSM RAM */
    CSL_REG32_WR((uint32_t *) CSL_HSM_RAM_U_BASE, 0x2U);

    /* Read SEC MGR */
    CSL_REG32_RD((uint32_t *) CSL_HSM_SEC_MGR_U_BASE);

    /*Write SEC MGR  */
    CSL_REG32_WR((uint32_t *) CSL_HSM_SEC_MGR_U_BASE, 0x2U);

    if(gDataAbortReceived == 17U)
    {
        testStatus = SystemP_SUCCESS;
    }

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
}
