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

#include <string.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_drivers_config.h"
#include <kernel/dpl/DebugP.h>
#include <drivers/soc.h>
#include <drivers/pcie.h>


/******** Buffer *******/
#define BUF_SIZE 60

uint32_t src_buf[BUF_SIZE]  __attribute__((aligned(4096)));
uint32_t dst_buf[BUF_SIZE]  __attribute__((aligned(4096)));

/* Initialize buffer */
void Pcie_bufInit(void)
{
    int i = 0;

    for (i = 0; i < BUF_SIZE; i++)
    {
        src_buf[i] = 0xC0DEC0DE;
    }

    for (i = 0; i < BUF_SIZE; i++)
    {
        dst_buf[i] = 0xFFFFFFFF;
    }

    return;
}

void pcie_buf_transfer_rc_main (void *args)
{
    int32_t status = SystemP_SUCCESS;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("Device in RC mode\r\n");

    /* Initialize buffer */
    Pcie_bufInit();

    /* Get EP Device Id and Vendor Id */
    {
        uint32_t devId;
        uint32_t vndId;

        status = Pcie_getVendorId(gPcieHandle[CONFIG_PCIE0], PCIE_LOCATION_REMOTE, &vndId, &devId);

        if (SystemP_SUCCESS == status)
        {
            DebugP_log ("Endpoint Device ID: %XX\r\n", devId);
            DebugP_log ("Endpoint Vendor ID: %XX\r\n", vndId);
        }
        else
        {
            DebugP_logError ("Unable to read Endpoint Device & Vendor ID\r\n");
        }

    }

    /* Initiate buffer transfer */
    {

        void *transBufAddr = (void *)(CONFIG_PCIE0_OB_REGION0_LOWER);

        memcpy(transBufAddr, src_buf, BUF_SIZE * sizeof(uint32_t));

        CacheP_wbInv(transBufAddr, sizeof(uint32_t) * BUF_SIZE, CacheP_TYPE_ALL);

    }

    do
    {
        CacheP_inv(dst_buf, BUF_SIZE * sizeof(uint32_t), CacheP_TYPE_ALL);
    }while (dst_buf[BUF_SIZE - 1] != 0xC0DEC0DEu);


    for (int i = 0; i < BUF_SIZE; i++)
    {
        if(dst_buf[i] != 0xC0DEC0DEu)
        {
            status = SystemP_FAILURE;
            break;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_logError("Data transfer mismatch\r\n");
    }

    Board_driversClose();
    Drivers_close();

    return;
}
