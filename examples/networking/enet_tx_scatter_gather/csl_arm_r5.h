    /**
 * @file  csl_arm_r5.h
 *
 * @brief
 *  Header file containing various enumerations, structure definitions and function
 *  declarations for the ARM R5 IP.
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2017, Texas Instruments, Inc.
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
 *
*/
#ifndef CSL_ARM_R5_H_
#define CSL_ARM_R5_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
#define CSL_R5_MPIDR_AFF0_MASK                                   (0x000000FFU)
#define CSL_R5_MPIDR_AFF0_SHIFT                                  (0U)
#define CSL_R5_MPIDR_AFF1_MASK                                   (0x0000FF00U)
#define CSL_R5_MPIDR_AFF1_SHIFT                                  (8U)
#define CSL_R5_MPIDR_AFF2_MASK                                   (0x00FF0000U)
#define CSL_R5_MPIDR_AFF2_SHIFT                                  (16U)
#define CSL_R5_MPIDR_MULEXT_MASK                                 (0xC0000000U)
#define CSL_R5_MPIDR_MULEXT_SHIFT                                (30U)

/** ===========================================================================
 *
 * @defgroup CSL_ARM_R5_API R5 API
 * @ingroup CSL_ARM_R5_API
 *
 * ============================================================================
 */
/**
@defgroup CSL_ARM_R5_DATASTRUCT  R5 Data Structures
@ingroup CSL_ARM_R5_API
*/
/**
@defgroup CSL_ARM_R5_FUNCTION  R5 Functions
@ingroup CSL_ARM_R5_API
*/
/**
@defgroup CSL_ARM_R5_ENUM R5 Enumerated Data Types
@ingroup CSL_ARM_R5_API
*/

/** ===========================================================================
 *  @addtogroup CSL_ARM_R5_ENUM
    @{
 * ============================================================================
 */

/**
 *  \anchor CSL_ArmR5ClusterGroupID
 *  \name Pulsar/R5 Cluster Group IDs
 *
 *  @{
 */
/** \brief Pulsar/R5 Cluster Group ID0 */
#define CSL_ARM_R5_CLUSTER_GROUP_ID_0                 ((uint32_t) 0x00U)
/** \brief Pulsar/R5 Cluster Group ID1 */
#define CSL_ARM_R5_CLUSTER_GROUP_ID_1                 ((uint32_t) 0x01U)
/** \brief Pulsar/R5 Cluster Group ID2 */
#define CSL_ARM_R5_CLUSTER_GROUP_ID_2                 ((uint32_t) 0x02U)
/* @} */

/**
 *  \anchor CSL_ArmR5CPUID
 *  \name R5 Core IDs
 *
 *  @{
 */
/** \brief R5 Core ID0 */
#define CSL_ARM_R5_CPU_ID_0                          ((uint32_t) 0x00U)
/** \brief R5 Core ID1 */
#define CSL_ARM_R5_CPU_ID_1                          ((uint32_t) 0x01U)
/* @} */

/**
 *  \anchor CSL_ArmR5Mode
 *  \name R5 Mode
 *
 *  @{
 */
/** \brief R5 Mode */
#define CSL_ARM_R5_SYS_MODE                         ((uint32_t) 0x1FU)
/* @} */


/**
 * \brief  Structure containing the Exception Handlers.
 *         If application does not want register an exception handler then
 *         below handlers can be assigned to 'NULL' value.
 *         In such case, default handler will be used which have
 *         the infinite loop.
 */
typedef struct
{
    uint32_t cpuID;
    /**< CPU/core ID within cluster
     *   Refer #CSL_ArmR5CPUID
     */
    uint32_t grpId;
    /**< Group Id of the cluster
     *   Refer #CSL_ArmR5ClusterGroupID
     */
    uint32_t multiprocessingExt;
    /**< Prefetch Abort exception handler */
}CSL_ArmR5CPUInfo;

/* @} */

/**
 *  \addtogroup CSL_ARM_R5_DATASTRUCT
 *  @{
 */

/* @} */

/**
 *  \addtogroup CSL_ARM_R5_FUNCTION
 *  @{
 */

/**
 *  \brief Executes DSB instruction
 */
void CSL_armR5Dsb(void);

/**
 *  \brief Enable/disable the Floating Point Unit (FPU)
 *
 *  This function is used to enable or disable the FPU.
 *
 *  \param enable   [IN]    0=FPU is disabled, otherwise FPU is enabled
 *
 *  \return None
 */
void CSL_armR5FpuEnable( uint32_t enable );

/**
 *  \brief Enable/disable FIQ interrupt generation
 *
 *  This function is used to enable or disable FIQ interrupt generation.
 *
 *  \param enable   [IN]    0=Interrupt generation is disabled, otherwise
 *                          it is enabled
 *
 *  \return None
 */
void CSL_armR5IntrEnableFiq( uint32_t enable );

/**
 *  \brief Enable/disable IRQ interrupt generation
 *
 *  This function is used to enable or disable IRQ interrupt generation.
 *
 *  \param enable   [IN]    0=Interrupt generation is disabled, otherwise
 *                          it is enabled
 *
 *  \return None
 */
void CSL_armR5IntrEnableIrq( uint32_t enable );

/**
 *  \brief Enable/disable VIC
 *
 *  This function is used to enable or disable the VIC.
 *
 *  \param enable   [IN]    0=VIC is disabled, otherwise
 *                          it is enabled
 *
 *  \return None
 */
void CSL_armR5IntrEnableVic( uint32_t enable );

/**
 *  \brief Enable/disable instruction caches
 *
 *  This function is used to enable or disable instruction caches.
 *
 *  \param enable   [IN]    0=Instruction caches are disabled, otherwise
 *                          they are enabled
 *
 *  \return None
 */
void CSL_armR5CacheEnableICache( uint32_t enable );

/**
 *  \brief Enable/disable data caches
 *
 *  This function is used to enable or disable data caches.
 *
 *  \param enable   [IN]    0=Data cache are disabled, otherwise
 *                          they are enabled
 *
 *  \return None
 */
void CSL_armR5CacheEnableDCache( uint32_t enable );

/**
 *  \brief Enable/disable instruction and data caches
 *
 *  This function is used to enable or disable instruction and data caches.
 *
 *  \param enable   [IN]    0=All caches are disabled, otherwise
 *                          they are enabled
 *
 *  \return None
 */
void CSL_armR5CacheEnableAllCache( uint32_t enable );

/**
 *  \brief Invalidate all instruction caches
 *
 *  This function is used to invalidate all instruction cache.
 *
 *  \param None
 *
 *  \return None
 */
void CSL_armR5CacheInvalidateAllIcache( void );

/**
 *  \brief Invalidate all data caches
 *
 *  This function is used to invalidate all data caches.
 *
 *  \param None
 *
 *  \return None
 */
void CSL_armR5CacheInvalidateAllDcache( void );

/**
 *  \brief Invalidate all instruction and data caches
 *
 *  This function is used to invalidate all instruction and data caches.
 *
 *  \param None
 *
 *  \return None
 */
void CSL_armR5CacheInvalidateAllCache( void );

/**
 *  \brief Invalidate an instruction cache line by MVA
 *
 *  This function is used to invalidate an instruction cache Line by MVA.
 *
 *  \param address  [IN]    Modified virtual address
 *
 *  \return None
 */
void CSL_armR5CacheInvalidateIcacheMva( uint32_t address );

/**
 *  \brief Invalidate a data cache line by MVA
 *
 *  This function is used to invalidate a data cache Line by MVA.
 *
 *  \param address  [IN]    Modified virtual address
 *
 *  \return None
 */
void CSL_armR5CacheInvalidateDcacheMva( uint32_t address );

/**
 *  \brief Invalidate a data cache line by set and way
 *
 *  This function is used to invalidate a data cache line by set and way.
 *
 *  \param set      [IN]    Indicates the cache set to invalidate
 *  \param way      [IN]    Indicates the cache way to invalidate
 *
 *  \return None
 */
void CSL_armR5CacheInvalidateDcacheSetWay( uint32_t set, uint32_t way );

/**
 *  \brief Clean a data cache line by MVA
 *
 *  This function is used to clean a data cache Line by MVA.
 *
 *  \param address  [IN]    Modified virtual address
 *
 *  \return None
 */
void CSL_armR5CacheCleanDcacheMva( uint32_t address );

/**
 *  \brief Clean a data cache line by set and way
 *
 *  This function is used to clean a data cache line by set and way.
 *
 *  \param set      [IN]    Indicates the cache set to clean
 *  \param way      [IN]    Indicates the cache way to clean
 *
 *  \return None
 */
void CSL_armR5CacheCleanDcacheSetWay( uint32_t set, uint32_t way );

/**
 *  \brief Clean and invalidate a data cache line by MVA
 *
 *  This function is used to clean and invalidate a data cache Line by MVA.
 *
 *  \param address  [IN]    Modified virtual address
 *
 *  \return None
 */
void CSL_armR5CacheCleanInvalidateDcacheMva( uint32_t address );

/**
 *  \brief Clean and invalidate a data cache line by set and way
 *
 *  This function is used to clean and invalidate a data cache line by set and
 *  way.
 *
 *  \param set      [IN]    Indicates the cache set to clean and invalidate
 *  \param way      [IN]    Indicates the cache way to clean and invalidate
 *
 *  \return None
 */
void CSL_armR5CacheCleanInvalidateDcacheSetWay( uint32_t set, uint32_t way );

/**
 *  \brief Enable/disable force write-through (WT) for write-back (WB) regions
 *
 *  This function is used to enable or disable force write-through (WT) for
 *  write-back (WB) regions.
 *
 *  \param enable   [IN]    0=No forcing of WT, otherwise WT forced for WB
 *                          regions
 *
 *  \return None
 */
void CSL_armR5CacheEnableForceWrThru( uint32_t enable );

/**
 *  \brief Disable ECC (parity) checking on cache rams
 *
 *  This function is used to disable ECC (parity) checking on cache rams.
 *
 *  \param None
 *
 *  \return None
 */
void CSL_armR5CacheDisableEcc( void );

/**
 *  \brief Enable AXI slave access to cache RAM
 *
 *  This function is used to enable AXI slave access to cache RAM.
 *
 *  \param None
 *
 *  \return None
 */
void CSL_armR5CacheEnableAxiAccess( void );

/**
 *  \brief Get the instruction cache line size
 *
 *  This function is used to get the instruction cache line size for MCU.
 *  Implementation of this API/code is use-case specific.
 *
 *  \param None
 *
 *  \return the instruction cache line size in bytes
 */
uint32_t CSL_armR5CacheGetIcacheLineSize( void );

/**
 *  \brief Get the data cache line size
 *
 *  This function is used to get the data cache line size for MCU.
 *  Implementation of this API/code is use-case specific.
 *
 *  \param None
 *
 *  \return the data cache line size in bytes
 */
uint32_t CSL_armR5CacheGetDcacheLineSize( void );

/**
 *  \brief Get the cluster group and CPU ID for current R5 Core
 *
 *  \param cpuInfo          Pointer to CPU info structure
 *                          Refer struct #CSL_ArmR5CPUInfo
 *
 *  \return None
 * Please NOTE that this function has to be called in privileged mode only
 */
void CSL_armR5GetCpuID( CSL_ArmR5CPUInfo *cpuInfo );

/**
 *  \brief Get the contents fo MPIDR register
 *
 *  \param None
 *
 *  \return                 contents of MPIDR register
 * Please NOTE that this function has to be called in privileged mode only
 */
uint32_t CSL_armR5ReadMpidrReg( void );

/**
 *  \brief Get the CPSR register value
 *
 *  This function is used to get the current value of CPSR value.
 *
 *  \param None
 *
 *  \return value           CPSR register value
 */
uintptr_t CSL_armR5GetCpsrRegVal( void );

/**
 *  \brief This function will disable IRQ and FIQ in single instruction
 *
 *  \param None
 *
 *  \return None
 */
void CSL_armR5DisableIrqFiq( void );

/**
 *  \brief This function will enable IRQ and FIQ in single instruction
 *
 *  \param cookie           CPSR value/ values returned by Intc_SystemDisable()
 *
 *  \return None
 */
uintptr_t CSL_armR5EnableIrqFiq( uint32_t cookie );

/* @} */

#ifdef __cplusplus
}
#endif  /* extern "C" */

#endif  /* end of CSL_ARM_R5_H_ definition */
