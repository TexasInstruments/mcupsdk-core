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
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include "ti_drivers_open_close.h"
#include "ti_eclipse_threadx_open_close.h"
#include "ti_eclipse_threadx_config.h"
#include <filex_mmcsd.h>
#include <fx_api.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define TEST_MMCSD_FILE_LINE_CNT         (100U)

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

static void test_filex_file_read_write(void *args);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_main(ULONG args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    EclipseThreadx_open();
    UNITY_BEGIN();

    for (size_t k = 0u; k < FILEX_NUM_INSTANCES; k++) {
        RUN_TEST(test_filex_file_read_write, 20000, &gt_media[k]);
    }

    UNITY_END();
    EclipseThreadx_close();
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
 * Test cases
 */
static void test_filex_file_read_write(void *args)
{
    FX_MEDIA *p_media;
    char *fileData = "THIS IS A TEST FILE TO TEST SD CARD FILE IO\n";
    int cmp;
    uint32_t i;
    char buf[100];
    UINT status;
    ULONG actual_sz;
    FX_FILE file;

    p_media = (FX_MEDIA *)args;

    /* Create a new file 'test.bin'. */
    status = fx_file_create(p_media, "test.bin");
    TEST_ASSERT_EQUAL_INT32(status, FX_SUCCESS);

    /* Open the file for writing. */
    status = fx_file_open(p_media, &file, "test.bin", FX_OPEN_FOR_WRITE);
    TEST_ASSERT_EQUAL_INT32(status, FX_SUCCESS);

    /* Write file data `TEST_MMCSD_FILE_LINE_CNT` times */
    uint32_t fileDataLen = strlen(fileData)+1;
    for(i = 0u; i < TEST_MMCSD_FILE_LINE_CNT; i++) {
        // Write the message.
        status = fx_file_write(&file, fileData, fileDataLen);
        TEST_ASSERT_EQUAL_INT32(status, FX_SUCCESS);
    }

    /* Close the file. */
    status = fx_file_close(&file);
    DebugP_assert(status == FX_SUCCESS);

    /* Re-open now for reading */
    status = fx_file_open(p_media, &file, "test.bin", FX_OPEN_FOR_READ);
    TEST_ASSERT_EQUAL_INT32(status, FX_SUCCESS);

    /* Now read the lines one by one and check with fileData */
    cmp = 0;
    for(i = 0U; i < TEST_MMCSD_FILE_LINE_CNT; i++) {

        status = fx_file_read(&file, &buf[0], fileDataLen, &actual_sz);
        TEST_ASSERT_EQUAL_INT32(status, FX_SUCCESS);

        cmp |= strncmp(fileData, &buf[0], fileDataLen);
        if(cmp != 0) {
            break;
        }
    }

    /* Close the file. */
    status = fx_file_close(&file);
    TEST_ASSERT_EQUAL_INT32(status, FX_SUCCESS);

    /* Delete file */
    status = fx_file_delete(p_media, "test.bin");
    TEST_ASSERT_EQUAL_INT32(status, FX_SUCCESS);

    TEST_ASSERT_EQUAL_INT(cmp, 0);
}