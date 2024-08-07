/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

#ifndef IG__RL2_H__
#define IG__RL2_H__

#include <stdint.h>
#include <drivers/hw_include/cslr_soc.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief RL2 Write Hit Interrupt
 */
#define RL2_INTERRUPT_WRITE_HIT ((RL2_Interrupt)(0))
/**
 * @brief RL2 Write Error Interrupt
 */
#define RL2_INTERRUPT_WRITE_ERROR ((RL2_Interrupt)(1))

/**
 * @brief Define RL2 cache size of 8K
 */
#define RL2_CACHESIZE_8K     ((RL2_CacheSize)(0))
/**
 * @brief Define RL2 cache size of 16K
 */
#define RL2_CACHESIZE_16K    ((RL2_CacheSize)(1))

/**
 * @brief Define RL2 cache size of 32K
 */
#define RL2_CACHESIZE_32K    ((RL2_CacheSize)(2))

/**
 * @brief Define RL2 cache size of 64K
 */
#define RL2_CACHESIZE_64K    ((RL2_CacheSize)(3))

/**
 * @brief Define RL2 cache size of 128K
 */
#define RL2_CACHESIZE_128K   ((RL2_CacheSize)(4))

/**
 * @brief Define RL2 cache size of 256K
 */
#define RL2_CACHESIZE_256K   ((RL2_CacheSize)(5))

/**
 * @brief No error or success
 *
 */
#define RL2_API_STS_SUCCESS                    ((RL2_API_STS_t)(0))
/**
 * @brief value was passed that would force the driver to write to reserved bit of a register
 *
 */
#define RL2_API_STS_RESERVED_BIT_WRITE           ((RL2_API_STS_t)(1U<<0U))
/**
 * @brief egions address and length were configured such that they conflicts with the other RL2 configurations
 *
 */
#define RL2_API_STS_WRONG_REGION_CALCULATION    ((RL2_API_STS_t)(1U<<1U))
/**
 * @brief regions address and length were configured such that they are overlapping
 *
 */
#define RL2_API_STS_REGIONS_OVERLAPPING         ((RL2_API_STS_t)(1U<<2U))
/**
 * @brief cannot configure RL2 due to some reason, e.g. pointer to tConfigRl2Cache is NULL
 *
 */
#define RL2_API_STS_CANNOT_CONFIGURE            ((RL2_API_STS_t)(1U<<3U))
/**
 * @brief cache size configured to some value which is not known
 *
 */
#define RL2_API_STS_UNKNOWN_CACHE_SIZE          ((RL2_API_STS_t)(1U<<4U))
/**
 * @brief unknown Interrupt number was given
 *
 */
#define RL2_API_STS_UNKNOWN_INTERRUPT            ((RL2_API_STS_t)(1U<<5U))
/**
 * @brief RL2 Interrupts
 * @see RL2_Interrupt_Writehit
 * @see RL2_Interrupt_WriteError
 */
typedef uint32_t RL2_Interrupt;
/**
 * @brief RL2 cache sizes
 * @see RL2_CACHESIZE_8K
 * @see RL2_CACHESIZE_16K
 * @see RL2_CACHESIZE_32K
 * @see RL2_CACHESIZE_64K
 * @see RL2_CACHESIZE_128K
 * @see RL2_CACHESIZE_256K
 */
typedef uint32_t RL2_CacheSize;
/**
 * @brief status of RL2 API function
 * @see RL2_API_STS_SUCCESS
 * @see RL2_API_STS_RESERVED_BIT_WRITE
 * @see RL2_API_STS_WRONG_REGION_CALCULATION
 * @see RL2_API_STS_REGIONS_OVERLAPPING
 * @see RL2_API_STS_CANNOT_CONFIGURE
 * @see RL2_API_STS_UNKNOWN_CACHE_SIZE
 * @see RL2_API_STS_UNKNOWN_INTERRUPT
 */
typedef uint32_t RL2_API_STS_t;

/**
 * @brief Struct to configure RL2
 *
 */
typedef struct
{
    uint32_t rangeStart;    ///< XIP flash address start
    uint32_t rangeEnd;      ///< XIP flash address end
    RL2_CacheSize cacheSize; ///< min 8KB max 256KB (dual)
    uint32_t l2Sram0Base;   ///< base address of remote region 0
    uint32_t l2Sram0Len;    ///< (remote region 0 length)/64
    uint32_t l2Sram1Base;   ///< base address of remote region 1
    uint32_t l2Sram1Len;    ///< (remote region 1 length)/64
    uint32_t l2Sram2Base;   ///< base address of remote region 2
    uint32_t l2Sram2Len;    ///< (remote region 2 length)/64
    uint32_t baseAddress;   ///< baseaddress of the ip
}RL2_Params;

/**
 * @brief configure RL2
 *
 * @param config [in] pointer to RL2_Params struct object
 * @return RL2_API_STS_t
 * @see RL2_API_STS_t
 * @see RL2_Params
 */
RL2_API_STS_t RL2_configure(RL2_Params *config);

/**
 * @brief Init RL2_Params struct with default values
 *
 * @param config [inout] pointer to RL2_Params object which need to be initialized
 * @return RL2_API_STS_t
 */
RL2_API_STS_t RL2_initparams(RL2_Params * config);


/**
 * @brief Enable RL2
 *
 * @param config [inout] pointer to RL2_Params object
 * @return RL2_API_STS_t
 */
RL2_API_STS_t RL2_enable(RL2_Params * config);


/**
 * @brief disable RL2
 *
 * @param config [inout] pointer to RL2_Params object
 * @return RL2_API_STS_t
 */
RL2_API_STS_t RL2_disable(RL2_Params * config);

/**
 * @brief set RL2 related interrupts
 *
 * @param config pointer to the RL2_Params
 * @param intr interrupt which needs to be set
 * @return RL2_API_STS_t
 * @see RL2_Params
 */
RL2_API_STS_t RL2_setInterrupt(RL2_Params * config, RL2_Interrupt intr);

/**
 * @brief clear RL2 related Interrupts
 *
 * @param config pointer to the RL2_Params
 * @param intr interrupt which needs to be set
 * @return int
 */
RL2_API_STS_t RL2_clearInterrupt(RL2_Params * config, RL2_Interrupt intr);

/**
 * @brief return IRQ mask.
 *
 * Refer to TRM for bit wise position of the returned value.
 *
 * @param config [in] RL2_Params
 * @param status [out] poninter to the uint32_t variable in which the status will be stored.
 * @return RL2_API_STS_t
 */
RL2_API_STS_t RL2_readIRQMask(RL2_Params * config, uint32_t * status);

/**
 * @brief reutrn RL2 IRQ Status register value.
 *
 * Refer to TRM for bit wise position of the returned value.
 *
 * @param config [in] RL2_Params
 * @param status [out] poninter to the uint32_t variable in which the status will be stored.
 * @return RL2_API_STS_t
 */
RL2_API_STS_t RL2_readIRQStatus(RL2_Params * config, uint32_t * status);

/**
 * @brief get cache miss count.
 *
 * @param config [in] pointer to RL2_Params struct.
 * @param miss [out] pointer to variable to write cache miss to.
 * @return RL2_API_STS_t
 */
RL2_API_STS_t RL2_getCacheMiss(RL2_Params * config, uint32_t * miss);

/**
 * @brief get miss count.
 *
 * @param config [in] pointer to RL2_Params struct.
 * @param hits [out] pointer to variable to wirte cache hits.
 * @return RL2_API_STS_t
 */
RL2_API_STS_t RL2_getCacheHits(RL2_Params * config, uint32_t * hits);


#ifdef __cplusplus
}
#endif

#endif //IG__RL2_H__