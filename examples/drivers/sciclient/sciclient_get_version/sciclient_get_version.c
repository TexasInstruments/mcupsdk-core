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

#include <inttypes.h>
#include <drivers/sciclient.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

void sciclient_get_version_main(void *args)
{
    int32_t         retVal = SystemP_SUCCESS;

    /* Open drivers to open UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /* Check for the SYSFW version by sending a request */
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
    DebugP_assert(SystemP_SUCCESS == retVal && respPrm.flags == TISCI_MSG_FLAG_ACK);

    DebugP_log("DMSC Firmware Version %s\r\n",(char *) response.str);
    DebugP_log("Firmware revision 0x%x\r\n", response.version);
    DebugP_log("ABI revision %d.%d\r\n",response.abi_major,response.abi_minor);

    /* Do an ABI check */
    retVal = Sciclient_abiCheck();
    DebugP_assert(SystemP_SUCCESS == retVal);

    {
        /* Fetch the CPU clock */
        uint64_t clkRate = 0;

        retVal = Sciclient_pmGetModuleClkFreq(
                    Sciclient_getSelfDevIdCore(),
                    TISCI_DEV_R5FSS0_CORE0_CPU_CLK,
                    &clkRate,
                    SystemP_WAIT_FOREVER);
        DebugP_assert(SystemP_SUCCESS == retVal);

        DebugP_log("[SCICLIENT] CPU clock frequency = %" PRId64 " Hz \r\n", clkRate);
    }

    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}
