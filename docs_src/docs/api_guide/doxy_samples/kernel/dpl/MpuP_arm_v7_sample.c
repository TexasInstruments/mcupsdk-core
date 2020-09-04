
//! [include]
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/MpuP_armv7.h>
//! [include]

void samples()
{
//! [mpu]
    #define MAX_REGIONS (16u) /* Max regions on a R5F */

    MpuP_RegionAttrs regionParams;
    uint32_t regionId = 0;

    /* recommend to disable Cache and MPU before setting up MPU regions */
    CacheP_disable(CacheP_TYPE_ALL);
    MpuP_disable();

    /* make all 4G as strongly ordered, non-cacheable */
    MpuP_RegionAttrs_init(&regionParams);
    regionParams.isEnable = 1;
    regionParams.isCacheable = 0;
    regionParams.isBufferable = 0;
    regionParams.isSharable = 1;
    regionParams.isExecuteNever = 1; /* dont allow code execution */
    regionParams.tex = 0;
    regionParams.accessPerm = MpuP_AP_S_RW;
    MpuP_setRegion(regionId, (void*)0x00000000, MpuP_RegionSize_4G, &regionParams);
    regionId++;

    /* make ATCM as cacheable */
    MpuP_RegionAttrs_init(&regionParams);
    regionParams.isEnable = 1;
    regionParams.isCacheable = 1;
    regionParams.isBufferable = 1;
    regionParams.isSharable = 0;
    regionParams.isExecuteNever = 0; /* allow code execution */
    regionParams.tex = 1;
    regionParams.accessPerm = MpuP_AP_S_RW;
    MpuP_setRegion(regionId, (void*)0x00000000, MpuP_RegionSize_32K, &regionParams);
    regionId++;

    /* make memory region as strongly ordered, non-cacheable */
    MpuP_RegionAttrs_init(&regionParams);
    regionParams.isEnable = 1;
    regionParams.isCacheable = 0;
    regionParams.isBufferable = 0;
    regionParams.isSharable = 1;
    regionParams.isExecuteNever = 1; /* dont allow code execution */
    regionParams.tex = 0;
    regionParams.accessPerm = MpuP_AP_S_RW;
    MpuP_setRegion(regionId, (void*)0x02000000, MpuP_RegionSize_32K, &regionParams);
    regionId++;

    if(regionId >= MAX_REGIONS)
    {
        while(1) { ; }
        /* Typically MPU setup happens very early in the boot sequence,
         * at this point DebugP_assert(), DebugP_log() are not available
         */
    }

    /* enable Cache and MPU after setting up MPU regions */
    MpuP_enable();
    CacheP_enable(CacheP_TYPE_ALL);


//! [mpu]
}