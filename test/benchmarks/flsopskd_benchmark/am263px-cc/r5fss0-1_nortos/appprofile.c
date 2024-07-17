/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

#include <stdint.h>
#include <string.h>
#include <assert.h>

#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/DebugP.h>




#include "csl_arm_r5_pmu.h"

#include "appprofile.h"

#define PMU_EVENT_COUNTER_1 (CSL_ARM_R5_PMU_EVENT_TYPE_ICACHE_STALL)
#define PMU_EVENT_COUNTER_2 (CSL_ARM_R5_PMU_EVENT_TYPE_I_X)
#define PMU_EVENT_COUNTER_3 (CSL_ARM_R5_PMU_EVENT_TYPE_ICACHE_MISS)



#define UTILS_ARRAYSIZE(x) sizeof(x)/sizeof (x[0U])
#define UTILS_ALIGN(x,align)  ((((x) + ((align) - 1))/(align)) * (align))


void config_pmu(void)
{

    CSL_armR5PmuCfg(0, 0 ,1);
    CSL_armR5PmuEnableAllCntrs(1);
    int num_cnt = CSL_armR5PmuGetNumCntrs();

    DebugP_assert(num_cnt == 3);
    CSL_armR5PmuCfgCntr(0, PMU_EVENT_COUNTER_1);
    CSL_armR5PmuCfgCntr(1, PMU_EVENT_COUNTER_2);
    CSL_armR5PmuCfgCntr(2, PMU_EVENT_COUNTER_3);

    CSL_armR5PmuEnableCntrOverflowIntr(0, 0);
    CSL_armR5PmuEnableCntrOverflowIntr(1, 0);
    CSL_armR5PmuEnableCntrOverflowIntr(2, 0);
    CSL_armR5PmuResetCntrs();
    CSL_armR5PmuEnableCntr(0, 1);
    CSL_armR5PmuEnableCntr(1, 1);
    CSL_armR5PmuEnableCntr(2, 1);
}












