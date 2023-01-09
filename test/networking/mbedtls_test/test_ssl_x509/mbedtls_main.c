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

#include "utils/mbedtls_test_utils.h"

int* mbedtls_test_ssl();
int* mbedtls_test_x509();

/* Unity framework required information */
void setUp(void)
{
}

void tearDown(void)
{
}

void mbedtls_unity_prints(int* mbedtls_test_results)
{
    int total_tests = mbedtls_test_results[0];
    int total_fail = mbedtls_test_results[1];
    int total_skipped = mbedtls_test_results[2];
    char unity_buffer[50];
    sprintf(unity_buffer, "total tests: %d", total_tests);
    TEST_MESSAGE(unity_buffer);
    if(total_fail>0)
    {
        sprintf(unity_buffer, "total tests failed: %d", total_fail);
        TEST_FAIL_MESSAGE(unity_buffer);
    }
    sprintf(unity_buffer, "total tests skipped: %d", total_skipped);
    TEST_MESSAGE(unity_buffer);
}

void mbedtls_test_x509_wrapper(void *args)
{
    mbedtls_unity_prints(mbedtls_test_x509());
}

void mbedtls_test_ssl_wrapper(void *args)
{
    mbedtls_unity_prints(mbedtls_test_ssl());
}

int mbedtls_test_main()
{
    Drivers_open();
    Board_driversOpen();
    UNITY_BEGIN();
    RUN_TEST((mbedtls_test_ssl_wrapper), 8584, NULL);
    RUN_TEST((mbedtls_test_x509_wrapper), 8584, NULL);
    UNITY_END();
    Drivers_close();
    Board_driversClose();
    return 0;
}
