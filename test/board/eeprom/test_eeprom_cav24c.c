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

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unity.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define APP_BUF_SIZE                    (256U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void test_eeprom_attributes(void *args);
static void test_eeprom_readwrite(void *args);
static int32_t eeprom_writeVerify(EEPROM_Handle handle,
                                  uint32_t offset,
                                  uint32_t size);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Allocate as uint32_t so that write pattern can be more than 256 so that we
 * can avoid any wrap around mistakes */
uint32_t gReadBuffer[APP_BUF_SIZE / sizeof(uint32_t)];
uint32_t gWriteBuffer[APP_BUF_SIZE / sizeof(uint32_t)];

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_main(void *args)
{
    int32_t status = SystemP_SUCCESS;

    UNITY_BEGIN();

    /* Open I2C and other drivers */
    Drivers_open();

    /* Open EEPROM_I2C drivers with I2C instance as input */
    status = Board_driversOpen();
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    RUN_TEST(test_eeprom_attributes, 244, NULL);
    RUN_TEST(test_eeprom_readwrite,  245, NULL);

    UNITY_END();

    /* Close I2C and other drivers */
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

/* Testcases */
static void test_eeprom_attributes(void *args)
{
    const EEPROM_Attrs *attr;

    attr = EEPROM_getAttrs(CONFIG_EEPROM0);
    TEST_ASSERT_NOT_NULL(attr);

    DebugP_log("[EEPROM] Attributes Size       : %d\r\n", attr->size);
    DebugP_log("[EEPROM] Attributes Page Count : %d\r\n", attr->pageCount);
    DebugP_log("[EEPROM] Attributes Page Size  : %d\r\n", attr->pageSize);

    return;
}

static void test_eeprom_readwrite(void *args)
{
    int32_t         status;
    uint32_t        startOffset;
    EEPROM_Handle   eepromHandle = gEepromHandle[CONFIG_EEPROM0];

    /* Offset aligned to page boundary */
    startOffset = 48;  /* 48: Not to corrupt board ID */
    status = eeprom_writeVerify(eepromHandle, startOffset, 1);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    status = eeprom_writeVerify(eepromHandle, startOffset, 4);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
	status = eeprom_writeVerify(eepromHandle, startOffset, 16);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    status = eeprom_writeVerify(eepromHandle, startOffset, 32);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
	status = eeprom_writeVerify(eepromHandle, startOffset, 128);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    /* Offset not aligned to page boundary */
    startOffset = 48 + 8;  /* 48: Not to corrupt board ID */
    status = eeprom_writeVerify(eepromHandle, startOffset, 1);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    status = eeprom_writeVerify(eepromHandle, startOffset, 4);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
	status = eeprom_writeVerify(eepromHandle, startOffset, 16);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    status = eeprom_writeVerify(eepromHandle, startOffset, 32);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
	status = eeprom_writeVerify(eepromHandle, startOffset, 128);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    return;
}

static int32_t eeprom_writeVerify(EEPROM_Handle handle,
                                  uint32_t offset,
                                  uint32_t size)
{
    int32_t     status;
    uint32_t    i;
    uint8_t    *rdPtr, *wrPtr;

    DebugP_log("[EEPROM] Write/read verify test with offset: %d, size:%d ...\r\n", offset, size);

    /* Memset write and read buffer */
    rdPtr = (uint8_t *) &gReadBuffer[0U];
    wrPtr = (uint8_t *) &gWriteBuffer[0U];
    for(i = 0U; i < (APP_BUF_SIZE / sizeof(uint32_t)); i++)
    {
        gReadBuffer[i] = 0U;
        gWriteBuffer[i] = i;
    }

    /* Write to EEPROM */
    status = EEPROM_write(handle, offset, wrPtr, size);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    /* Read from EEPROM */
    status = EEPROM_read(handle, offset, rdPtr, size);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    /* compare write and read buffer */
    TEST_ASSERT_EQUAL_UINT8_ARRAY(wrPtr, rdPtr, size);

    return (status);
}
