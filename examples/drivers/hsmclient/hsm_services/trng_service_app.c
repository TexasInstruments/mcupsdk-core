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
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/SystemP.h>
#include <security/security_common/drivers/hsmclient/hsmclient.h>
#include <security/security_common/drivers/secure_ipc_notify/sipc_notify.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/dpl/SystemP.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/hw_include/cslr_soc.h>

#define APP_CLIENT_ID (0x02)
#define SEED_SIZE_IN_DWORDS (12U)

/* Demo Application code on R5 */
void HsmRngApp_start(HsmClient_t *client)
{
    /* loop through and request random number from HSM */
    /* also calculate the time spent doing the generation */
    int32_t status;
    uint32_t length = 16,startCycleCount, endCycleCount;
    uint32_t val[4];
    const uint32_t cpuMHz = SOC_getSelfCpuClk()/1000000;
    /* struct instance used for sending get random num request */
    RNGReq_t getRNG;

    /* initialize parameters of the getRNG struct */
    uint32_t RngDrbgSeed[12] = {0x949db311, 0x1b53c4bf, 0x1d6cb9de, 0x75c85f23,
                                 0xfe6bfe37, 0xae1c6462, 0x9e45f958, 0x62493581,
                                 0x8b5df32b, 0x7bc94d49, 0xa8e69e31, 0x9237ca9f};
    getRNG.DRBGMode = 0x5A;
    getRNG.seedSizeInDWords = SEED_SIZE_IN_DWORDS;
    getRNG.seedValue = (uint32_t *)&RngDrbgSeed;
    getRNG.resultLength = length;
    getRNG.resultPtr = (uint8_t *)val;

    CycleCounterP_reset();

    startCycleCount = CycleCounterP_getCount32();
    status = HsmClient_getRandomNum(client, &getRNG);
    endCycleCount = CycleCounterP_getCount32();

    DebugP_assert(status == SystemP_SUCCESS);

    /* print the random numbers generated */
    for (int i = 0; i < length / 4; i++)
    {
        DebugP_log("RNG output word -- 0x%X\r\n", val[i]);
    }

    DebugP_log("\r\n\r\n [HSM CLIENT_PROFILE] Time taken by get RNG request : %dus\r\n", ((endCycleCount - startCycleCount)/cpuMHz));
}
