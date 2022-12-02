
//! [include]
#include <stdio.h>
#include <drivers/pmu.h>
//! [include]

//! [config]
PMU_EventCfg gPmuEventCfg[3] = 
{
    {
        .name = "ICache Miss",
        .type = CSL_ARM_R5_PMU_EVENT_TYPE_ICACHE_MISS,
    },
    {
        .name = "DCache Access",
        .type = CSL_ARM_R5_PMU_EVENT_TYPE_DCACHE_ACCESS,
    },
    {
        .name = "DCache Miss",
        .type = CSL_ARM_R5_PMU_EVENT_TYPE_DCACHE_MISS,
    },
};

PMU_Config gPmuConfig = 
{
    .bCycleCounter = 1,
    .numEventCounters = 3U,
    .eventCounters = gPmuEventCfg,
};
//! [config]

void init(void)
{
//! [init]
    PMU_init(&gPmuConfig);
//! [init]
}

void profile(void)
{
//! [profile]
    PMU_profileStart("Block1");
    /* Code Block 1 */
    uint32_t i, j, k = 0;
    uint32_t arr[100] = { 0U };
    for(i = 0; i < 100; i++)
    {
        for(j = 0; j < 100; j++)
        {
            arr[k] = i + j;
            k = (k+1) % 100;
        }
    }
    PMU_profileEnd("Block1");
//! [profile]
//! [print_entry]
    PMU_profilePrintEntry("Block1");
//! [print_entry]
//! [print_dump]
    PMU_profilePrint();
//! [print_dump]
}
