/*
 *  Copyright (C) 2018-2023 Texas Instruments Incorporated
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


#include <kernel/dpl/MpuP_armv7.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/HwiP.h>

#define MPU_SECTION __attribute__((section(".text.mpu")))

/* Max possible regions in ARMv7-M CPU */
#define MpuP_MAX_REGIONS    (16u)

#define MPU_BASE        (0xE000ED90u)
#define MPU_CTRL        (volatile uint32_t *)((MPU_BASE) + 0x04u)
#define MPU_RNR         (volatile uint32_t *)((MPU_BASE) + 0x08u)
#define MPU_RBAR        (volatile uint32_t *)((MPU_BASE) + 0x0Cu)
#define MPU_RASR        (volatile uint32_t *)((MPU_BASE) + 0x10u)


/* these are defined as part of SysConfig */
extern MpuP_Config gMpuConfig;
extern MpuP_RegionConfig gMpuRegionConfig[];


static uint32_t MPU_SECTION MpuP_getAttrsAndSize(MpuP_RegionAttrs *region, uint32_t size)
{
    uint32_t regionAttrs =
          ((uint32_t)(region->isExecuteNever & (uint32_t)0x1) << 28)
        | ((uint32_t)(region->accessPerm     & (uint32_t)0x7) << 24)
        | ((uint32_t)(region->tex            & (uint32_t)0x7) << 19)
        | ((uint32_t)(region->isSharable     & (uint32_t)0x1) << 18)
        | ((uint32_t)(region->isCacheable    & (uint32_t)0x1) << 17)
        | ((uint32_t)(region->isBufferable   & (uint32_t)0x1) << 16)
        | ((uint32_t)(region->subregionDisableMask & (uint32_t)0xFF) << 8)
        | ((uint32_t)(size & (uint32_t)0x1F)                  << 1)
        | ((uint32_t)(region->isEnable       & (uint32_t)0x1) << 0)
        ;

    return regionAttrs;
}

void MPU_SECTION MpuP_RegionAttrs_init(MpuP_RegionAttrs *region)
{
    region->isExecuteNever = 0;
    region->accessPerm     = MpuP_AP_S_RW_U_R;
    region->tex            = 0;
    region->isSharable     = 1;
    region->isCacheable    = 0;
    region->isBufferable   = 0;
    region->isEnable       = 0;
    region->subregionDisableMask = 0;
}

void MPU_SECTION MpuP_setRegion(uint32_t regionNum, void * addr, uint32_t size, MpuP_RegionAttrs *attrs)
{
    uint32_t baseAddress, regionAndSizeAttrs;
    uint32_t enabled;
    uintptr_t key;
    uint32_t setSize = size;

    DebugP_assertNoLog( regionNum < MpuP_MAX_REGIONS);

    /* size 5b field */
    setSize = (setSize & 0x1FU);

    /* align base address to region size */
    baseAddress = ((uint32_t)addr & ~( (1U<<((uint64_t)setSize+1U))-1U ));

    /* get region attribute mask */
    regionAndSizeAttrs = MpuP_getAttrsAndSize(attrs, setSize);

    enabled = MpuP_isEnable();

    /* disable the MPU (if already disabled, does nothing) */
    MpuP_disable();

    key = HwiP_disable();

    *MPU_RNR  = regionNum;
    *MPU_RBAR = (baseAddress & (0xFFFFFFE0u));
    *MPU_RASR = regionAndSizeAttrs;

    HwiP_restore(key);

    if ((enabled) != 0U) {
        MpuP_enable();
    }
}

void MPU_SECTION MpuP_enable(void)
{
    if((MpuP_isEnable()) == 0U)
    {
        uint32_t value;
        uintptr_t key;

        key = HwiP_disable();

        value = 0;
        if ((gMpuConfig.enableBackgroundRegion) != 0U) {
            value |= (1u << 2u);  /* PRIVDEFENA, 0: Disables the default memory map,
                                   *             1: enable default memory map for non mapped regions
                                   */
        }
        value |= (1u << 1u);  /* HFNMIENA, 0: disable MPU for fault handlers, 1: enable MPU for fault handlers */
        value |= (1u << 0u);  /* 0: MPU disable, 1: MPU enable */

        *MPU_CTRL = value;

        __asm__ __volatile__  (" dsb" "\n\t": : : "memory");
        __asm__ __volatile__  (" isb" "\n\t": : : "memory");

        HwiP_restore(key);
    }
}

void MPU_SECTION MpuP_disable(void)
{
    if((MpuP_isEnable()) == 1U)
    {
        uintptr_t key;

        key = HwiP_disable();

        __asm__ __volatile__ (" dsb""\n\t": : : "memory");

        *MPU_CTRL = 0; /* Disable MPU */

        HwiP_restore(key);
    }
}

uint32_t MPU_SECTION MpuP_isEnable(void)
{
    return (*MPU_CTRL & 0x1U);
}

void MPU_SECTION MpuP_init(void)
{
    uint32_t i;

    if ((MpuP_isEnable()) != 0U) {
        MpuP_disable();
    }

    DebugP_assertNoLog( gMpuConfig.numRegions < MpuP_MAX_REGIONS);

    /*
     * Initialize MPU regions
     */
    for (i = 0; i < gMpuConfig.numRegions; i++)
    {
        MpuP_setRegion(i,
                (void*)gMpuRegionConfig[i].baseAddr,
                gMpuRegionConfig[i].size,
                &gMpuRegionConfig[i].attrs
        );
    }

    if ((gMpuConfig.enableMpu) != 0U) {
        MpuP_enable();
    }
}
