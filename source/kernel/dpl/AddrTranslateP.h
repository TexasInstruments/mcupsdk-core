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

#ifndef ADDR_TRANSLATEP_H
#define ADDR_TRANSLATEP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <kernel/dpl/SystemP.h>

/**
 * \defgroup KERNEL_DPL_ADDR_TRANSLATE APIs for Region based address translation (RAT) module
 * \ingroup KERNEL_DPL
 *
 * For more details and example usage, see \ref KERNEL_DPL_ADDR_TRANSLATE_PAGE
 *
 * @{
 */

/**
 * \brief Maximum regions in the address translate module
 */
#define AddrTranslateP_MAX_REGIONS      (16u)

/**
 * \brief Enum's to represent different possible region size for the address translate module
 */
typedef enum AddrTranslateP_RegionSize_e {
    AddrTranslateP_RegionSize_1 = 0x0,
    AddrTranslateP_RegionSize_2,
    AddrTranslateP_RegionSize_4,
    AddrTranslateP_RegionSize_8,
    AddrTranslateP_RegionSize_16,
    AddrTranslateP_RegionSize_32,
    AddrTranslateP_RegionSize_64,
    AddrTranslateP_RegionSize_128,
    AddrTranslateP_RegionSize_256,
    AddrTranslateP_RegionSize_512,
    AddrTranslateP_RegionSize_1K,
    AddrTranslateP_RegionSize_2K,
    AddrTranslateP_RegionSize_4K,
    AddrTranslateP_RegionSize_8K,
    AddrTranslateP_RegionSize_16K,
    AddrTranslateP_RegionSize_32K,
    AddrTranslateP_RegionSize_64K,
    AddrTranslateP_RegionSize_128K,
    AddrTranslateP_RegionSize_256K,
    AddrTranslateP_RegionSize_512K,
    AddrTranslateP_RegionSize_1M,
    AddrTranslateP_RegionSize_2M,
    AddrTranslateP_RegionSize_4M,
    AddrTranslateP_RegionSize_8M,
    AddrTranslateP_RegionSize_16M,
    AddrTranslateP_RegionSize_32M,
    AddrTranslateP_RegionSize_64M,
    AddrTranslateP_RegionSize_128M,
    AddrTranslateP_RegionSize_256M,
    AddrTranslateP_RegionSize_512M,
    AddrTranslateP_RegionSize_1G,
    AddrTranslateP_RegionSize_2G,
    AddrTranslateP_RegionSize_4G
} AddrTranslateP_RegionSize;

/**
 * \brief Region config structure, this is used by SysConfig and not to be used by end-users directly
 */
typedef struct AddrTranslateP_RegionConfig_ {

    uint64_t systemAddr; /**< translated 48b system addr as seen by the SOC, MUST align to region size */
    uint32_t localAddr;  /**< region start address as seen by the CPU, MUST align to region size */
    uint32_t size;       /**< region size, see \ref AddrTranslateP_RegionSize */

} AddrTranslateP_RegionConfig;

/**
 * \brief Parameters for \ref AddrTranslateP_init, this is used by SysConfig and not to be used by end-users directly
 */
typedef struct AddrTranslateP_Params_s {

    uint32_t numRegions;             /**< Number of regions to configure */
    uint32_t ratBaseAddr;            /**< Base address of the RAT HW module */
    AddrTranslateP_RegionConfig *regionConfig; /**< Pointer to array of region config,
                                                 * number of array element MUST be >= numRegions
                                                 */
} AddrTranslateP_Params;


/**
 * \brief Set default value for address translate parameters
 *
 * \param   params  [out] Structure initialized with default parameters
 */
void AddrTranslateP_Params_init(AddrTranslateP_Params *params);

/**
 * \brief Initialize Address translate sub-system, called by SysConfig, not to be called by end users
 *
 * \param   params  [in] Initialization parameters
 */
void AddrTranslateP_init(AddrTranslateP_Params *params);

/**
 * \brief Translate from 48b system address to a CPU address as seen via the RAT module
 *
 * \note If no mapping is found then lower 32b are returned as the local address,
 *       i.e no translation is done for 32b address, and address truncation for > 32b input addresses.
 * \note All drivers MUST call this API to translate peripheral MMR base addresses to CPU visible base addr.
 *       It is recommended to call this once to get the base address during driver module init.
 *
 * \param systemAddr    [in] 48b system address or SOC view address
 *
 * \return void * CPU view or local address
 */
void *AddrTranslateP_getLocalAddr(uint64_t systemAddr);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ADDR_TRANSLATEP_H */
