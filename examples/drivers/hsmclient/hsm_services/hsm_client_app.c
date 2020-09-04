/*
 *  Copyright (C) 2018-2022 Texas Instruments Incorporated
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
#include <kernel/dpl/CycleCounterP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/hsmclient.h>
#include <drivers/sipc_notify.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/dpl/SystemP.h>
#include <string.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/soc.h>

#define APP_CLIENT_ID                  (0x02)
#define PARSED_VER_SIZE                (0X96)

/* Demo Application code on R5 */
void HsmClientApp_start(void)
{
    /* loop through and request get version from HSM */
    /* also calculate the time spent doing get version */
    int32_t status ;
    HsmClient_t client ;
    uint8_t *uid = malloc(HSM_UID_SIZE) ;
    char parsedVer[PARSED_VER_SIZE];
    memset(parsedVer, '\0' , PARSED_VER_SIZE);
    HsmVer_t *hsmVer = malloc(sizeof(HsmVer_t)) ;
    uint32_t startCycleCount, endCycleCount;
    const uint32_t cpuMHz = SOC_getSelfCpuClk()/1000000;

    status = HsmClient_register(&client,APP_CLIENT_ID);
    DebugP_assert(status == SystemP_SUCCESS);

    CycleCounterP_reset();
    DebugP_log("\r\n [HSM CLIENT] Sending UID Request to HSM Server..");

    startCycleCount = CycleCounterP_getCount32();

    /* Send Request for UID to HSM Server */
    status = HsmClient_getUID(&client, (uint8_t *)uid, SystemP_WAIT_FOREVER);
    endCycleCount = CycleCounterP_getCount32();

    DebugP_assert(status == SystemP_SUCCESS);
    DebugP_log("\r\n [HSM CLIENT] Device UID Suceessfully retrived from the HSM Server.");
    DebugP_log("\r\n\r\n [HSM CLIENT_PROFILE] Time taken by GetUID Request : %dus\r\n", ((endCycleCount - startCycleCount)/cpuMHz));

    /* print UID */
    DebugP_log("\r\n [HSM CLIENT] Device UID is : ");
    for(uint8_t i = 0; i<HSM_UID_SIZE; i++)
    {
        DebugP_log("%02X", uid[i]);
    }

    /* Send Request for TIFS-MCU version to HSM Server */
    status = HsmClient_getVersion(&client,hsmVer,SystemP_WAIT_FOREVER);
    DebugP_assert(status == SystemP_SUCCESS);

    /* print version */
    DebugP_log("[HSM CLIENT] TIFS-MCU 64bit version string = 0x00%llx\r\n",hsmVer->HsmrtVer);
    DebugP_log("\r\n [HSM CLIENT] TIFS-MCU Information");
    status = HsmClient_parseVersion(hsmVer, parsedVer);
    DebugP_log("%s", parsedVer);
}
