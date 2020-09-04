/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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

#ifndef MMUP_ARMV8_H
#define MMUP_ARMV8_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <kernel/dpl/SystemP.h>

/**
 * \defgroup KERNEL_DPL_MMU_ARMV8 APIs for MMU for ARMv8 (ARM A53)
 * \ingroup KERNEL_DPL
 *
 * For more details and example usage, see \ref KERNEL_DPL_MMU_ARMV8_PAGE
 *
 * @{
 */

/**
 * \brief Enum's to represent different types of access permissions that are possible for a given MMU region
 */
typedef enum MmuP_AccessPerm_ {
    MMUP_ACCESS_PERM_PRIV_RW_USER_NONE = 0x0, /**< RW Permission in privileged mode only */
    MMUP_ACCESS_PERM_PRIV_RW_USER_RW = 0x1, /**< RW Permission in privileged and user mode */
    MMUP_ACCESS_PERM_PRIV_RO_USER_NONE = 0x2, /**< RD Permission for privileged mode and access bllocked for user mode */
    MMUP_ACCESS_PERM_PRIV_RO_USER_RO = 0x3 /**< All access are blocked */
} MmuP_AccessPerm;

/**
 * \brief Enum's to represent MMU region sharable  status
 */
typedef enum MmuP_Shareable_ {
    MMUP_SHARABLE_NONE = 0x0, /**< Not shareable */
    MMUP_SHARABLE_OUTER = 0x2, /**< Outer shareable */
    MMUP_SHARABLE_INNER = 0x3 /**< Inner shareable */
} MmuP_Shareable;

/**
 * \brief Enum's to represent MMU attribute index
 */
typedef enum MmuP_AttrIndx_ {
    MMUP_ATTRINDX_MAIR0 = 0,
    MMUP_ATTRINDX_MAIR1,
    MMUP_ATTRINDX_MAIR2,
    MMUP_ATTRINDX_MAIR3,
    MMUP_ATTRINDX_MAIR4,
    MMUP_ATTRINDX_MAIR5,
    MMUP_ATTRINDX_MAIR6,
    MMUP_ATTRINDX_MAIR7
} MmuP_AttrIndx;

/**
 * \brief Attribute's to apply for a MMU region
 *
 * \note AP, UXN, PXN, SH, Indx, G bits
 *  together control the access permissions, execution permission and if region must be fully cached or non-cached
 */
typedef struct MmuP_MapAttrs_ {
    MmuP_AccessPerm accessPerm; /**< RW Permission for Privileged and user mode */
    uint8_t privExecute; /**< 1: Privileged executable region, 0: Privileged non executable region */
    uint8_t userExecute; /**< 1: User executable region, 0: User non executable region */
    MmuP_Shareable shareable; /**< Shareable status for the region */
    MmuP_AttrIndx attrIndx; /**< Region attribute index */
    uint8_t global; /**< 1: Global, 0: Non-global */
} MmuP_MapAttrs;

/**
 * \brief MMU config structure, this used by SysConfig and not to be used by end-users directly
 */
typedef struct MmuP_Config_ {
    uint32_t numRegions;    /**< Number of regions to configure */
    uint8_t enableMmu;      /**< 0: keep MMU disabled, 1: enable MMU */
} MmuP_Config;

/**
 * \brief Region config structure, this used by SysConfig and not to be used by end-users directly
 */
typedef struct MmuP_RegionConfig_ {
    uint64_t vaddr;     /**< Virtual address, MUST aligned to granule size */
    uint64_t paddr;     /**< Physical address, MUST aligned to granule size */
    uint32_t size;      /**< Region size, MUST be aligned to granule size */
    MmuP_MapAttrs attr; /**< Region attributes, see \ref MmuP_MapAttrs */
} MmuP_RegionConfig;

/**
 * \brief Set default values to MmuP_MapAttrs
 *
 * Strongly recommended to be called before seting values in MmuP_MapAttrs
 *
 * \param attrs [out] parameter structure to set to default
 */
void MmuP_MapAttrs_init(MmuP_MapAttrs *attrs);

/**
 * \brief Enable MMU sub-system
 */
void MmuP_enable();

/**
 * \brief Disable MMU sub-system
 */
void MmuP_disable(void);

/**
 * \brief Check if MMU sub-system is enabled
 *
 * \return 0: MMU sub-system is disabled, 1: MMU sub-system is enabled
 */
uint32_t MmuP_isEnabled();

/* Memory map */

/**
 * \brief Setup a region in the MMU
 *
 * \note Recommended to disable MMU and disable cache before setting up MMU regions
 *
 * \param vaddr [in] virtual address of region to setup, MUST aligned to granule size
 * \param paddr [in] physical address of region to setup, MUST aligned to granule size
 * \param size [in] region size, MUST aligned to granule size
 * \param mapAttrs [in] map attrs, see \ref MmuP_MapAttrs
 */
int32_t MmuP_map(uintptr_t vaddr, uintptr_t paddr, uint32_t size, MmuP_MapAttrs *mapAttrs);

/**
 * \brief Initialize MMU sub-system, called by SysConfig, not to be called by end users
 *
 */
void MmuP_init();

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* MMUP_ARMV8_H */