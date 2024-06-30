/*
 *  Copyright (C) 2018-2024 Texas Instruments Incorporated
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
#include "ti_eclipse_threadx_config.h"
#include "ti_eclipse_threadx_open_close.h"
#include <fx_api.h>

#define MESSAGE  "Hello world!"

void filex_hello_world_main(ULONG args)
{
    int32_t res;
    UINT status;
    FX_FILE file;
    ULONG actual_sz;
    uint8_t t_buf[sizeof(MESSAGE)];

    Drivers_open();
    
    res = Board_driversOpen();
    DebugP_assert(res == SystemP_SUCCESS);

    res = EclipseThreadx_open();
    DebugP_assert(res == SystemP_SUCCESS);

    DebugP_log("[FILEX HELLO WORLD] Hello world example started ...\r\n");

    // Create a new file 'test.bin'.
    status = fx_file_create(&gt_media[FILEX0], "test.bin");
    DebugP_assert((status == FX_SUCCESS) || (status == FX_ALREADY_CREATED));

    // Open the file for writing.
    status = fx_file_open(&gt_media[FILEX0], &file, "test.bin", FX_OPEN_FOR_WRITE);
    DebugP_assert(status == FX_SUCCESS);

    // Clear the whole file (in case it already exists and contains data).
    status = fx_file_truncate(&file, 0);
    DebugP_assert((status == FX_SUCCESS) || (status == FX_ALREADY_CREATED));

    // Write the message.
    status = fx_file_write(&file, MESSAGE, sizeof(MESSAGE));
    DebugP_assert(status == FX_SUCCESS);

    // Close the file.
    status = fx_file_close(&file);
    DebugP_assert(status == FX_SUCCESS);

    // Re-open the file, this time for reading.
    status = fx_file_open(&gt_media[FILEX0], &file, "test.bin", FX_OPEN_FOR_READ);
    DebugP_assert(status == FX_SUCCESS);

    // Read the previously written message.
    status = fx_file_read(&file, &t_buf[0], sizeof(MESSAGE), &actual_sz);
    DebugP_assert(status == FX_SUCCESS);

    // Close the file.
    status = fx_file_close(&file);
    DebugP_assert(status == FX_SUCCESS);

    // Make sure that the read-back message matches the original message.
    if(strcmp((const char *)&t_buf[0], MESSAGE) != 0)
    {
        DebugP_log("Some tests have failed!!\r\n");
    }
    else
    {
        DebugP_log("All tests have passed!!\r\n");
    }

    res = EclipseThreadx_close();
    DebugP_assert(res == SystemP_SUCCESS);

    Board_driversClose();
    Drivers_close();
}