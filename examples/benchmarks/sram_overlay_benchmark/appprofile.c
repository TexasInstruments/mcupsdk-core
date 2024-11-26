#include <stdint.h>
#include <string.h>
#include <assert.h>

#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/DebugP.h>

#include "csl_arm_r5_pmu.h"

#include "appprofile.h"

#define PMU_EVENT_COUNTER_1 (CSL_ARM_R5_PMU_EVENT_TYPE_I_X)
#define PMU_EVENT_COUNTER_2 (CSL_ARM_R5_PMU_EVENT_TYPE_ICACHE_MISS)
#define PMU_EVENT_COUNTER_3 (CSL_ARM_R5_PMU_EVENT_TYPE_ICACHE_ACCESS)

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