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

#ifndef CACHEP_C6X_H
#define CACHEP_C6X_H


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <kernel/dpl/CacheP.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * \defgroup KERNEL_DPL_CACHE_C6X APIs for Cache in C6x
 * \ingroup KERNEL_DPL
 *
 * For more details and example usage, see \ref KERNEL_DPL_CACHE_C6X_PAGE
 *
 * @{
 */

/**
 *  \anchor CacheP_MarMask
 *  \name MAR mask values
 *  @{
 */
/**
 * \brief Permit copies field enables/disables the cacheability of the affected
 *        address range.
 *
 *      0 - Memory range not cacheable
 *      1 - Memory range cacheable
 */
#define CacheP_MarMask_PC               (0x00000001U)
/**
 * \brief Enables/disables the prefetchability of the affected address range.
 *        The L2 memory controller uses this bit to convey to the XMC whether
 *        a given address range is prefetchable.
 *
 *      0 - Memory range not prefetchable
 *      1 - Memory range prefetchable
 */
#define CacheP_MarMask_PFX              (0x00000008U)
/** @} */

/**
 *  \anchor CacheP_L1Size
 *  \name L1 cache size values
 *  @{
 */
/** \brief Amount of cache is 0K, Amount of SRAM is 32K */
#define CacheP_L1Size_0K                (0U)
/** \brief Amount of cache is 4K, Amount of SRAM is 28K */
#define CacheP_L1Size_4K                (1U)
/** \brief Amount of cache is 8K, Amount of SRAM is 24K */
#define CacheP_L1Size_8K                (2U)
/** \brief Amount of cache is 16K, Amount of SRAM is 16K */
#define CacheP_L1Size_16K               (3U)
/** \brief Amount of cache is 32K, Amount of SRAM is 0K */
#define CacheP_L1Size_32K               (4U)
/** @} */

/**
 *  \anchor CacheP_L2Size
 *  \name L2 cache size values
 *  @{
 */
/** \brief L2 is all SRAM */
#define CacheP_L2Size_0K                (0U)
/** \brief Amount of cache is 32K */
#define CacheP_L2Size_32K               (1U)
/** \brief Amount of cache is 64K */
#define CacheP_L2Size_64K               (2U)
/** \brief Amount of cache is 128K */
#define CacheP_L2Size_128K              (3U)
/** \brief Amount of cache is 256K */
#define CacheP_L2Size_256K              (4U)
/** \brief Amount of cache is 512K */
#define CacheP_L2Size_512K              (5U)
/** \brief Amount of cache is 1024K */
#define CacheP_L2Size_1024K             (6U)
/** @} */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/**
 * \brief Structure to describe cache size
 */
typedef struct CacheP_Size_
{
    uint32_t l1pSize;
    /**< L1 Program cache size, \ref Cache_L1Size */
    uint32_t l1dSize;
    /**< L1 Data data size, \ref Cache_L1Size */
    uint32_t l2Size;
    /**< L2 cache size, \ref CacheP_L2Size */
} CacheP_Size;

/**
 * \brief Structure to describe MAR settings for a region
 */
typedef struct CacheP_MarRegion_
{
    void       *baseAddr;
    /**< Region start address. Recommended to be 16MB aligned */
    uint32_t    size;
    /**< Region size in bytes. Recommended to be multiple of 16MB aligned */
    uint32_t    value;
    /**< Bit mask of \ref CacheP_MarMask */
} CacheP_MarRegion;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/** \brief Externally defined Cache Size configuration */
extern CacheP_Size      gCacheSize;
/** \brief Externally defined Cache Mar Region configuration array */
extern CacheP_MarRegion gCacheMarRegion[];
/** \brief Externally defined Cache Mar Region configuration array size */
extern uint32_t         gCacheMarRegionNum;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * \brief Set cache size
 *
 * \param size [in] cache size
 */
void CacheP_setSize(const CacheP_Size *size);

/**
 * \brief Get cache size
 *
 * \param size [out] cache size
 */
void CacheP_getSize(CacheP_Size *size);

/**
 * \brief Set MAR attribute for a memory range
 *
 * \note The API will set the MAR attribute for the range,
 *    start_addr = floor(baseAddr, 16MB) .. end_addr = ceil(baseAddr+size, 16MB)
 *
 * \param baseAddr [in] Region start address. Recommended to be 16MB aligned
 * \param size     [in] Region size in bytes. Recommended to be multiple of 16MB aligned
 * \param value    [in] Bit mask of \ref CacheP_MarMask
 */
void CacheP_setMar(void *baseAddr, uint32_t size, uint32_t value);

/**
 * \brief Get MAR attribute for a region of 16MB
 *
 * \note The API will make the region start addr as
 *    start_addr = floor(baseAddr, 16MB)
 *
 * \param baseAddr [in] region start address, recommended to be 16MB aligned
 *
 * \return MAR attribute for this 16MB region, see \ref CacheP_MarMask
 */
uint32_t CacheP_getMar(void *baseAddr);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* CACHEP_C6X_H */
