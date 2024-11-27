/*
 *  Copyright (C) 2022-23 Texas Instruments Incorporated
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

#include <stdlib.h>
#include "kernel/dpl/AddrTranslateP.h"
#include "ti_drivers_config.h"
#include "ti_board_config.h"

extern int __TI_ATRegion0_src_addr;
extern int __TI_ATRegion0_trg_addr;
extern int __TI_ATRegion0_region_sz;
extern int __TI_ATRegion1_src_addr;
extern int __TI_ATRegion1_trg_addr;
extern int __TI_ATRegion1_region_sz;
extern int __TI_ATRegion2_src_addr;
extern int __TI_ATRegion2_trg_addr;
extern int __TI_ATRegion2_region_sz;

void ipc_notify_echo_main(void *args);
void __attribute__((do_not_share)) AddrTranslateP_init (AddrTranslateP_Params *params);
void __attribute__((do_not_share)) AddrTranslateP_Params_init (AddrTranslateP_Params *params) ;

int AppStart(void)
{
    System_init();
    Board_init();

    ipc_notify_echo_main(NULL);

    Board_deinit();
    System_deinit();

    return 0;
}

__attribute__((do_not_share)) int main(void)
{

    AddrTranslateP_Params params;
    AddrTranslateP_RegionConfig region[3];

    AddrTranslateP_Params_init(&params);

    if((uint32_t)(&__TI_ATRegion0_region_sz) > 0)
    {
        params.numRegions++;
        region[0].size = 0;
        uint32_t actualSize = (uint32_t)(&__TI_ATRegion0_region_sz);
        region[0].localAddr = (uint32_t)&__TI_ATRegion0_src_addr;
        region[0].systemAddr = (uint32_t)&__TI_ATRegion0_trg_addr;
        for(uint32_t sz = 1; sz < actualSize; region[0].size++)
        {
            sz = sz << 1;
        }
    }

    if((uint32_t)(&__TI_ATRegion1_region_sz) > 0)
    {
        params.numRegions++;
        region[1].size = 0;
        region[1].localAddr = (uint32_t)&__TI_ATRegion1_src_addr;
        region[1].systemAddr = (uint32_t)&__TI_ATRegion1_trg_addr;
        for(uint32_t sz = 1; sz < (uint32_t)(&__TI_ATRegion1_region_sz); sz <<= 1, region[1].size++);
    }

    if((uint32_t)(&__TI_ATRegion2_region_sz) > 0)
    {
        params.numRegions++;
        region[2].size = 0;
        region[2].localAddr = (uint32_t)&__TI_ATRegion2_src_addr;
        region[2].systemAddr = (uint32_t)&__TI_ATRegion2_trg_addr;
        for(uint32_t sz = 1; sz < (uint32_t)(&__TI_ATRegion2_region_sz); sz <<= 1, region[2].size++);
    }

    params.ratBaseAddr = CSL_RL2_REGS_R5SS1_CORE0_U_BASE + CSL_RL2_OF_R5FSS0_CORE0_RAT_CTL(0) - 0x20;
    params.regionConfig = region;
    AddrTranslateP_init(&params);

    return AppStart();
}