
//! [include]
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/MmuP_armv8.h>
//! [include]

void sample()
{
//! [mmu]

    MmuP_MapAttrs mapAttrs;

    /* recommend to disable Cache and MMU before setting up MMU regions */
    CacheP_disable(CacheP_TYPE_ALL);
    MmuP_disable();

    /* make 4G region as privileged executable, outer shareable */
    MmuP_MapAttrs_init(&mapAttrs);
    mapAttrs.accessPerm = MMUP_ACCESS_PERM_PRIV_RW_USER_NONE;
    mapAttrs.privExecute = 1;
    mapAttrs.userExecute = 0;
    mapAttrs.shareable = MMUP_SHARABLE_OUTER,
    mapAttrs.attrIndx = MMUP_ATTRINDX_MAIR0;
    mapAttrs.global = 1;
    MmuP_map(0x0u, 0x0u, 0x80000000u, &mapAttrs);

    /* make memory region as outer and inner writeback cacheable */
    MmuP_MapAttrs_init(&mapAttrs);
    mapAttrs.accessPerm = MMUP_ACCESS_PERM_PRIV_RW_USER_NONE;
    mapAttrs.privExecute = 1;
    mapAttrs.userExecute = 0;
    mapAttrs.shareable = MMUP_SHARABLE_OUTER,
    mapAttrs.attrIndx = MMUP_ATTRINDX_MAIR7;
    mapAttrs.global = 1;
    MmuP_map(0x80000000u, 0x80000000u, 0x80000000u, &mapAttrs);

    /* enable Cache and MPU after setting up MPU regions */
    MmuP_enable();
    CacheP_enable(CacheP_TYPE_ALL);

//! [mmu]
}