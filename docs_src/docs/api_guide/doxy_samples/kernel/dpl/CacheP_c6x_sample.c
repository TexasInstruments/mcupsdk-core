
//! [include]
#include <stdio.h>
#include <kernel/dpl/CacheP_c6x.h>

//! [include]



void samples()
{
{
//! [setsize]
    CacheP_Size size;

    size.l1pSize = CacheP_L1Size_32K;
    size.l1dSize = CacheP_L1Size_32K;
    size.l2Size = CacheP_L2Size_64K;

    CacheP_setSize(&size);
//! [setsize]
}

{
//! [getsize]
    char *l1Size[] = { "0K", "4K", "8K", "16K", "32K" };
    char *l2Size[] = { "0K", "32K", "64K", "128K", "256K", "512K", "1024K" };

    CacheP_Size size;

    CacheP_getSize(&size);

    DebugP_log("Cache is size l1p = %s, l1d = %s, l2 = %s",
        l1Size[size.l1pSize],
        l1Size[size.l1dSize],
        l2Size[size.l2Size]
        );
//! [getsize]
}
{
//! [setmar]
    #define CACHE_REGION_ADDR 0x80000000
    #define CACHE_REGION_SIZE 0x00100000

    #define NON_CACHE_REGION_ADDR 0x80000000
    #define NON_CACHE_REGION_SIZE 0x00100000

    CacheP_setMar((void*)CACHE_REGION_ADDR, CACHE_REGION_SIZE, CacheP_MarMask_PC | CacheP_MarMask_PFX);
    CacheP_setMar((void*)NON_CACHE_REGION_ADDR, NON_CACHE_REGION_SIZE, 0);
//! [setmar]
}
}