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
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>

#define MPU_SECTION __attribute__((section(".text.mpu")))

/* Max possible regions in ARMv7-R CPU */
#define MpuP_MAX_REGIONS    (16u)

/* APIs defined in MpuP_armv7r_asm.s */
#ifdef __cplusplus
extern "C" {
#endif
void MpuP_disableAsm(void);
void MpuP_enableAsm(void);
uint32_t MpuP_isEnableAsm(void);
void MpuP_disableBRAsm(void);
void MpuP_enableBRAsm(void);
void MpuP_setRegionAsm(uint32_t regionId, uint32_t regionBaseAddr,
              uint32_t sizeAndEnble, uint32_t regionAttrs);
#ifdef __cplusplus
}
#endif

/* these are defined as part of SysConfig */
extern MpuP_Config gMpuConfig;
extern MpuP_RegionConfig gMpuRegionConfig[4u];


static uint32_t MPU_SECTION MpuP_getAttrs(MpuP_RegionAttrs *region)
{
    uint32_t regionAttrs =
          ((uint32_t)(region->isExecuteNever & (uint32_t)0x1) << (uint32_t)12)
        | ((uint32_t)(region->accessPerm     & (uint32_t)0x7) << (uint32_t)8)
        | ((uint32_t)(region->tex            & (uint32_t)0x7) << (uint32_t)3)
        | ((uint32_t)(region->isSharable     & (uint32_t)0x1) << (uint32_t)2)
        | ((uint32_t)(region->isCacheable    & (uint32_t)0x1) << (uint32_t)1)
        | ((uint32_t)(region->isBufferable   & (uint32_t)0x1) << (uint32_t)0);

    return regionAttrs;
}

void MPU_SECTION MpuP_RegionAttrs_init(MpuP_RegionAttrs *region)
{
    region->isExecuteNever = 0;
    region->accessPerm     = (uint8_t)MpuP_AP_S_RW_U_R;
    region->tex            = 0;
    region->isSharable     = 1;
    region->isCacheable    = 0;
    region->isBufferable   = 0;
    region->isEnable       = 0;
    region->subregionDisableMask = 0;
}

void MPU_SECTION MpuP_setRegion(uint32_t regionNum, void * addr, uint32_t size, MpuP_RegionAttrs *attrs)
{
    uint32_t baseAddress, sizeAndEnable, regionAttrs;
    uint32_t enabled;
    uintptr_t key;
    uint32_t value = size;

    DebugP_assertNoLog( regionNum < MpuP_MAX_REGIONS);

    /* size 5b field */
    value = (value & (uint32_t)0x1F);

    /* If N is the value in size field, the region size is 2N+1 bytes. */
    sizeAndEnable = ((uint32_t)(attrs->subregionDisableMask & (uint32_t)0xFF) << (uint32_t)8)
                  | ((uint32_t)(value            & (uint32_t)0x1F) << (uint32_t)1)
                  | ((uint32_t)(attrs->isEnable &  (uint32_t)0x1) << (uint32_t)0);

    /* align base address to region size */
    baseAddress = ((uint32_t)addr & ~( (1U <<((uint64_t)value+1U))-1U ));

    /* get region attribute mask */
    regionAttrs = MpuP_getAttrs(attrs);

    enabled = MpuP_isEnable();

    /* disable the MPU (if already disabled, does nothing) */
    MpuP_disable();

    key = HwiP_disable();

    MpuP_setRegionAsm(regionNum, baseAddress, sizeAndEnable, regionAttrs);

    HwiP_restore(key);

    if (enabled != 0U) {
        MpuP_enable();
    }
}

void MPU_SECTION MpuP_enable(void)
{
    if(MpuP_isEnable()==(uint32_t) 0U)
    {
        uint32_t type;
        uintptr_t key;

        key = HwiP_disable();

        /* get the current enabled bits */
        type = (uint32_t)CacheP_getEnabled();

        if (type & CacheP_TYPE_L1) {
            CacheP_disable(CacheP_TYPE_L1);
        }

        MpuP_enableAsm();

        /* set cache back to initial settings */
        CacheP_enable(type);

        __asm__  __volatile__ (" dsb" "\n\t": : : "memory");
        __asm__  __volatile__ (" isb" "\n\t": : : "memory");

        HwiP_restore(key);
    }
}

void MPU_SECTION MpuP_disable(void)
{
    if(MpuP_isEnable()!=0U)
    {
        uint32_t type;
        uintptr_t key;

        key = HwiP_disable();

        /* get the current enabled bits */
        type = CacheP_getEnabled();

        /* disable all enabled caches */
        CacheP_disable(type);

        __asm__ __volatile__ (" dsb" "\n\t": : : "memory");

        MpuP_disableAsm();

        /* set cache back to initial settings */
        CacheP_enable(type);

        HwiP_restore(key);
    }
}

uint32_t MPU_SECTION MpuP_isEnable(void)
{
    return MpuP_isEnableAsm();
}

void MPU_SECTION MpuP_init(void)
{
    uint32_t i;

    if (MpuP_isEnable()!=0U) {
        MpuP_disable();
    }

    MpuP_disableBRAsm();

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

    if (gMpuConfig.enableBackgroundRegion!=0U) {
        MpuP_enableBRAsm();
    }

    if (gMpuConfig.enableMpu!=0U) {
        MpuP_enable();
    }
}
