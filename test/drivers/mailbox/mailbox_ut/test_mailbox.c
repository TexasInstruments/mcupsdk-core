/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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
#include <string.h>
#include <unity.h>
#include <drivers/mailbox/v0/mailbox.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_open_close.h"

static const uint16_t gMessage_r5fss0_0[11] =
{
    0x1234, 0x4321, 0x0448, 0x0012,
    0x0000, 0x0000, 0x0001, 0xFBA4,
    0x0220, 0x0004, 0xBA39
};

static const uint16_t gMessage_c66ss0[11] =
{
    0x1234, 0x4321, 0x044A, 0x0012,
    0x0000, 0x0000, 0x0001, 0xFBA2,
    0x0220, 0x0004, 0x77B1
};

static const uint16_t gMessage1_r5fss0_0[22] =
{
    0x1234, 0x4321, 0x0448, 0x0012,
    0x0000, 0x0000, 0x0001, 0xFBA4,
    0x0220, 0x0004, 0xBA39, 0x1235,
    0x5432, 0x0248, 0x0112, 0x1000,
    0x0000, 0x0001, 0x01A4, 0x02A0,
    0x0004, 0xBA39
};

static const uint16_t gMessage1_c66ss0[22] =
{
    0x1234, 0x4321, 0x044A, 0x0012,
    0x0000, 0x0000, 0x0001, 0xFBA2,
    0x0220, 0x0004, 0x77B1, 0x1234,
    0x4321, 0x044A, 0x0012, 0x0000,
    0x0000, 0x0001, 0xFBA2, 0x0220,
    0x0004, 0x77B1
};

static uint32_t gMessageResponse[32];

void test_pos_main(void *args);
void test_neg_main(void *args);

void Mailbox_Callback(uint32_t remoteCoreId,void * args)
{
    int32_t status;
    status = Mailbox_read(
                    CSL_CORE_ID_RSS_R4,
                    (uint8_t*)gMessageResponse,
                    sizeof(gMessageResponse),
                    SystemP_WAIT_FOREVER
                );
        DebugP_assert(status==SystemP_SUCCESS);

        status = Mailbox_readDone(CSL_CORE_ID_RSS_R4);
        DebugP_assert(status==SystemP_SUCCESS);
}
void test_mailbox(void *args)
{
    int32_t status;
    uint8_t *buffer = NULL;
    uint32_t size = 0;
    uint32_t iterations = 10, i;

    if(IpcNotify_getSelfCoreId()==CSL_CORE_ID_R5FSS0_0)
    {
        buffer = (uint8_t*)gMessage1_r5fss0_0;
        size = sizeof(gMessage1_r5fss0_0);
    }
    if(IpcNotify_getSelfCoreId()==CSL_CORE_ID_C66SS0)
    {
        buffer = (uint8_t*)gMessage1_c66ss0;
        size = sizeof(gMessage1_c66ss0);
    }

    for(i=0; i<iterations; i++)
    {
        status = Mailbox_write(
                    CSL_CORE_ID_RSS_R4,
                    buffer,
                    size,
                    SystemP_WAIT_FOREVER
                );
        DebugP_assert(status==SystemP_SUCCESS);

        status = Mailbox_read(
                    CSL_CORE_ID_RSS_R4,
                    (uint8_t*)gMessageResponse,
                    sizeof(gMessageResponse),
                    SystemP_WAIT_FOREVER
                );
        DebugP_assert(status==SystemP_SUCCESS);

        status = Mailbox_readDone(CSL_CORE_ID_RSS_R4);
        DebugP_assert(status==SystemP_SUCCESS);
    }
}

void test_multipleRead(void *args)
{
    int32_t status;
    uint8_t *buffer = NULL;
    uint32_t size = 0;
    uint32_t iterations = 10, i;

    if(IpcNotify_getSelfCoreId()==CSL_CORE_ID_R5FSS0_0)
    {
        buffer = (uint8_t*)gMessage_r5fss0_0;
        size = sizeof(gMessage_r5fss0_0);
    }
    if(IpcNotify_getSelfCoreId()==CSL_CORE_ID_C66SS0)
    {
        buffer = (uint8_t*)gMessage_c66ss0;
        size = sizeof(gMessage_c66ss0);
    }

    for(i=0; i<iterations; i++)
    {
        status = Mailbox_write(
                    CSL_CORE_ID_RSS_R4,
                    buffer,
                    size,
                    SystemP_WAIT_FOREVER
                );
        DebugP_assert(status==SystemP_SUCCESS);

        status = Mailbox_read(
                    CSL_CORE_ID_RSS_R4,
                    (uint8_t*)gMessageResponse,
                    6U,
                    SystemP_WAIT_FOREVER
                );
        DebugP_assert(status==SystemP_SUCCESS);

        status = Mailbox_read(
                    CSL_CORE_ID_RSS_R4,
                    (uint8_t*)gMessageResponse,
                    5U,
                    SystemP_WAIT_FOREVER
                );
        DebugP_assert(status==SystemP_SUCCESS);

        status = Mailbox_readDone(CSL_CORE_ID_RSS_R4);
        DebugP_assert(status==SystemP_SUCCESS);
    }
}

void test_setReadCallback(void *args)
{
    int32_t status;
    uint8_t *buffer = NULL;
    uint32_t size = 0;
    uint32_t iterations = 10, i;

    if(IpcNotify_getSelfCoreId()==CSL_CORE_ID_R5FSS0_0)
    {
        buffer = (uint8_t*)gMessage_r5fss0_0;
        size = sizeof(gMessage_r5fss0_0);
    }
    if(IpcNotify_getSelfCoreId()==CSL_CORE_ID_C66SS0)
    {
        buffer = (uint8_t*)gMessage_c66ss0;
        size = sizeof(gMessage_c66ss0);
    }

    Mailbox_setReadCallback(Mailbox_Callback, NULL);
    for(i=0; i<iterations; i++)
    {
        status = Mailbox_write(
                    CSL_CORE_ID_RSS_R4,
                    buffer,
                    size,
                    SystemP_WAIT_FOREVER
                );
        DebugP_assert(status==SystemP_SUCCESS);


    }
}
void test_mailboxVarSize(void *args)
{
    int32_t status;
    uint8_t *buffer = NULL;
    uint32_t size = 0;
    uint32_t iterations = 20, i;

    if(IpcNotify_getSelfCoreId()==CSL_CORE_ID_R5FSS0_0)
    {
        buffer = (uint8_t*)gMessage1_r5fss0_0;
        size = sizeof(gMessage1_r5fss0_0);
    }
    if(IpcNotify_getSelfCoreId()==CSL_CORE_ID_C66SS0)
    {
        buffer = (uint8_t*)gMessage1_c66ss0;
        size = sizeof(gMessage1_c66ss0);
    }

    for(i=0; i<iterations; i++)
    {
        status = Mailbox_write(
                    CSL_CORE_ID_RSS_R4,
                    buffer,
                    size,
                    SystemP_WAIT_FOREVER
                );
        DebugP_assert(status==SystemP_SUCCESS);

        status = Mailbox_read(
                    CSL_CORE_ID_RSS_R4,
                    (uint8_t*)gMessageResponse,
                    sizeof(gMessageResponse),
                    SystemP_WAIT_FOREVER
                );
        DebugP_assert(status==SystemP_SUCCESS);

        status = Mailbox_readDone(CSL_CORE_ID_RSS_R4);
        DebugP_assert(status==SystemP_SUCCESS);
    }
}

void setUp(void)
{
}

void tearDown(void)
{
}

void test_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();

    UNITY_BEGIN();

    RUN_TEST(test_mailbox,2000,NULL);
    RUN_TEST(test_multipleRead,9026,NULL);
    RUN_TEST(test_mailboxVarSize,9027,NULL);
    RUN_TEST(test_setReadCallback,8959,NULL);


    UNITY_END();

    Drivers_close();

    test_pos_main(NULL);
    test_neg_main(NULL);
}
