    /**
 * @file  csl_arm_r5_pmu.h
 *
 * @brief
 *  Header file containing various enumerations, structure definitions and function
 *  declarations for the ARM R5 PMU IP.
 *  ============================================================================
 *  @n   (C) Copyright 2019, Texas Instruments, Inc.
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
#ifndef CSL_ARM_R5_PMU_H
#define CSL_ARM_R5_PMU_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 
 *
 * @defgroup CSL_ARM_R5_PMU_API R5 PMU API
 *
 * Provides the APIs to configure/read Performance Monitor Unit counters
 * 
 */

/**
@defgroup CSL_ARM_R5_PMU_FUNCTION  R5 PMU Functions
@ingroup CSL_ARM_R5_PMU_API
*/
/**
@defgroup CSL_ARM_R5_PMU_ENUM R5 PMU Enumerated Data Types
@ingroup CSL_ARM_R5_PMU_API
*/

/**
@defgroup CSL_ARM_R5_PMU_MACROS R5 PMU Macros
@ingroup CSL_ARM_R5_PMU_API
*/

/**
 *  @addtogroup CSL_ARM_R5_PMU_ENUM
    @{
 *
 */


/**
 * @brief This enumerator defines PMU event types
 *
 *
 */
typedef enum
{
    CSL_ARM_R5_PMU_EVENT_TYPE_SWINC                     = 0,
    CSL_ARM_R5_PMU_EVENT_TYPE_ICACHE_MISS               = 0x01U,
    CSL_ARM_R5_PMU_EVENT_TYPE_DCACHE_MISS               = 0x03U,
    CSL_ARM_R5_PMU_EVENT_TYPE_DCACHE_ACCESS             = 0x04U,
    CSL_ARM_R5_PMU_EVENT_TYPE_D_RD                      = 0x06U,
    CSL_ARM_R5_PMU_EVENT_TYPE_D_WR                      = 0x07U,
    CSL_ARM_R5_PMU_EVENT_TYPE_I_X                       = 0x08U,
    CSL_ARM_R5_PMU_EVENT_TYPE_PI_X                      = 0x5EU,
    CSL_ARM_R5_PMU_EVENT_TYPE_EXCEPTION                 = 0x09U,
    CSL_ARM_R5_PMU_EVENT_TYPE_EXCEPTION_RET             = 0x0AU,
    CSL_ARM_R5_PMU_EVENT_TYPE_CID_CHANGE                = 0x0BU,
    CSL_ARM_R5_PMU_EVENT_TYPE_SW_PC                     = 0x0CU,
    CSL_ARM_R5_PMU_EVENT_TYPE_B_IMMEDIATE               = 0x0DU,
    CSL_ARM_R5_PMU_EVENT_TYPE_PROC_RET                  = 0x0EU,
    CSL_ARM_R5_PMU_EVENT_TYPE_UNALIGNED_ACCESS          = 0x0FU,
    CSL_ARM_R5_PMU_EVENT_TYPE_BRANCH_TAKEN              = 0x10U,
    CSL_ARM_R5_PMU_EVENT_TYPE_BRANCH_PRED               = 0x12U,
    CSL_ARM_R5_PMU_EVENT_TYPE_ICACHE_STALL              = 0x40U,
    CSL_ARM_R5_PMU_EVENT_TYPE_DCACHE_STALL              = 0x41U,
    CSL_ARM_R5_PMU_EVENT_TYPE_DCACHE_WB                 = 0x42U,
    CSL_ARM_R5_PMU_EVENT_TYPE_MEM_REQ                   = 0x43U,
    CSL_ARM_R5_PMU_EVENT_TYPE_LSU_BUSY_STALL            = 0x44U,
    CSL_ARM_R5_PMU_EVENT_TYPE_SB_DRAIN                  = 0x45U,
    CSL_ARM_R5_PMU_EVENT_TYPE_FIQ_DISABLED_CYCLES       = 0x46U,
    CSL_ARM_R5_PMU_EVENT_TYPE_IRQ_DISABLED_CYCLES       = 0x47U,
    CSL_ARM_R5_PMU_EVENT_TYPE_ETMEXTOUTM0               = 0x48U,
    CSL_ARM_R5_PMU_EVENT_TYPE_ETMEXTOUTM1               = 0x49U,
    CSL_ARM_R5_PMU_EVENT_TYPE_ICACHE_TAG_CECC           = 0x4AU,
    CSL_ARM_R5_PMU_EVENT_TYPE_ICACHE_DATA_CECC          = 0x4BU,
    CSL_ARM_R5_PMU_EVENT_TYPE_DCACHE_TAG_CECC           = 0x4CU,
    CSL_ARM_R5_PMU_EVENT_TYPE_DCACHE_DATA_CECC          = 0x4DU,
    CSL_ARM_R5_PMU_EVENT_TYPE_TCM_FECC_PF               = 0x4EU,
    CSL_ARM_R5_PMU_EVENT_TYPE_TCM_FECC_LS               = 0x4FU,
    CSL_ARM_R5_PMU_EVENT_TYPE_SB_MERGE                  = 0x50U,
    CSL_ARM_R5_PMU_EVENT_TYPE_LSU_SB_STALL              = 0x51U,
    CSL_ARM_R5_PMU_EVENT_TYPE_LSU_QF_STALL              = 0x52U,
    CSL_ARM_R5_PMU_EVENT_TYPE_INT_DIV                   = 0x53U,
    CSL_ARM_R5_PMU_EVENT_TYPE_INT_DIV_STALL             = 0x54U,
    CSL_ARM_R5_PMU_EVENT_TYPE_PLD_LINEFILL              = 0x55U,
    CSL_ARM_R5_PMU_EVENT_TYPE_PLD_NO_LINEFILL           = 0x56U,
    CSL_ARM_R5_PMU_EVENT_TYPE_NONCACHEABLE_ACCESS       = 0x57U,
    CSL_ARM_R5_PMU_EVENT_TYPE_ICACHE_ACCESS             = 0x58U,
    CSL_ARM_R5_PMU_EVENT_TYPE_SB_ATTR                   = 0x59U,
    CSL_ARM_R5_PMU_EVENT_TYPE_DUAL_ISSUE_CASE_A         = 0x5AU,
    CSL_ARM_R5_PMU_EVENT_TYPE_DUAL_ISSUE_CASE_B         = 0x5BU,
    CSL_ARM_R5_PMU_EVENT_TYPE_DUAL_ISSUE_CASE_OTHER     = 0x5CU,
    CSL_ARM_R5_PMU_EVENT_TYPE_DOUBLE_FP                 = 0x5DU,
    CSL_ARM_R5_PMU_EVENT_TYPE_DCACHE_DATA_FECC          = 0x60U,
    CSL_ARM_R5_PMU_EVENT_TYPE_DCACHE_TAG_FECC           = 0x61U,
    CSL_ARM_R5_PMU_EVENT_TYPE_LIVELOCK                  = 0x62U,
    CSL_ARM_R5_PMU_EVENT_TYPE_ATCM_MB_ECC               = 0x64U,
    CSL_ARM_R5_PMU_EVENT_TYPE_B0TCM_MB_ECC              = 0x65U,
    CSL_ARM_R5_PMU_EVENT_TYPE_B1TCM_MB_ECC              = 0x66U,
    CSL_ARM_R5_PMU_EVENT_TYPE_ATCM_SB_ECC               = 0x67U,
    CSL_ARM_R5_PMU_EVENT_TYPE_B0TCM_SB_ECC              = 0x68U,
    CSL_ARM_R5_PMU_EVENT_TYPE_B1TCM_SB_ECC              = 0x69U,
    CSL_ARM_R5_PMU_EVENT_TYPE_TCM_CECC_LS               = 0x6AU,
    CSL_ARM_R5_PMU_EVENT_TYPE_TCM_CECC_PF               = 0x6BU,
    CSL_ARM_R5_PMU_EVENT_TYPE_TCM_FECC_AXI              = 0x6CU,
    CSL_ARM_R5_PMU_EVENT_TYPE_TCM_CECC_AXI              = 0x6DU,
    CSL_ARM_R5_PMU_EVENT_TYPE_CORRECTABLE_EVENTS        = 0x6EU,
    CSL_ARM_R5_PMU_EVENT_TYPE_FATAL_EVENTS              = 0x6FU,
    CSL_ARM_R5_PMU_EVENT_TYPE_CORRECTABLE_BUS_FAULTS    = 0x70U,
    CSL_ARM_R5_PMU_EVENT_TYPE_FATAL_BUS_FAULTS          = 0x71U,
    CSL_ARM_R5_PMU_EVENT_TYPE_ACP_DCACHE_ACCESS         = 0x72U,
    CSL_ARM_R5_PMU_EVENT_TYPE_ACP_DCACHE_INVALIDATE     = 0x73U,
    CSL_ARM_R5_PMU_EVENT_TYPE_CYCLE_CNT                 = 0xFFU
} CSL_ArmR5PmuEventType;


/* @} */

/**
 *  \addtogroup CSL_ARM_R5_PMU_MACROS
 *  @{
 */
/** PMU Cycle count number */
#define CSL_ARM_R5_PMU_CYCLE_COUNTER_NUM                                           (31U)

/** PMU_CFG_CNTR_EVENT_TYPE */
#define CSL_ARM_R5_PMU_CFG_CNTR_EVENT_TYPE_MASK                                    (0x000000FFU)
#define CSL_ARM_R5_PMU_CFG_CNTR_EVENT_TYPE_SHIFT                                   (0x00000000U)
#define CSL_ARM_R5_PMU_CFG_CNTR_EVENT_TYPE_RESETVAL                                (0x00000000U)
#define CSL_ARM_R5_PMU_CFG_CNTR_EVENT_TYPE_MAX                                     (0x000000FFU)

/* @} */

/**
 *  \addtogroup CSL_ARM_R5_PMU_FUNCTION
 *  @{
 */


/**
 *  \brief Configure the Performance Management Unit (PMU)
 *
 *  Requirement: REQ_TAG(PDK-6048)
 *  Design: did_csl_core_pmu_configure
 *
 *  This function is used to configure the PMU.
 *
 *  The value arguments provided to this function are directly written into
 *  the corresponding R5 system register. As a result, the values must be
 *  constructed per the expected register format.
 *
 *  See the "MPU memory region programming registers" section of the ARM Cortex
 *  R5 TRM for more information.
 *
 *  \param cycleCntDiv      [IN]    Cycle count divider: 0=Counts every
 *                                  processor clock cycle, otherwise counts
 *                                  every 64th processor clock cycle
 *  \param exportEvents     [IN]    Export of the events to the event bus for an
 *                                  external monitoring block: 0=Export disabled,
 *                                  otherwise export is enabled.
 *  \param userEnable       [IN]    User mode access to performance monitor and
 *                                  validation registers: 0=disabled, otherwise
 *                                  enabled.
 *
 *  \return None
 */
extern void CSL_armR5PmuCfg( uint32_t cycleCntDiv, uint32_t exportEvents, uint32_t userEnable );

/**
 *  \brief Enable/disable all PMU counters
 *
 *
 *  Requirement: REQ_TAG(PDK-6048)
 *  Design: did_csl_core_pmu_configure
 *
 *  This function is used to enable or disable all of the PMU counters.
 *  Note that to enable a specific counter, both this function and the
 *  #CSL_armR5PmuEnableCntr function must be called.
 *
 *  \param enable   [IN]    0=All counters are disabled, otherwise
 *                          they are enabled
 *
 *  \return None
 */
extern void CSL_armR5PmuEnableAllCntrs( uint32_t enable );

/**
 *  \brief Get the number of PMU counters supported
 *
 *  Requirement: REQ_TAG(PDK-6048)
 *  Design: did_csl_core_pmu_read
 *
 *  This function is used to get the number of PMU counters supported.
 *
 *
 *  \return The number of PMU counters supported
 */
extern uint32_t CSL_armR5PmuGetNumCntrs( void );

/**
 *  \brief Configure a PMU counter
 *
 *  Requirement: REQ_TAG(PDK-6048)
 *  Design: did_csl_core_pmu_configure
 *
 *  This function is used to configure a PMU counter.
 *
 *  \param cntrNum      [IN]    Counter number (0..(#CSL_armR5PmuGetNumCntrs()-1))
 *  \param eventType    [IN]    Event type to count. See #CSL_ArmR5PmuEventType
 *                              for available event types.
 *
 *  \return None
 */
extern void CSL_armR5PmuCfgCntr( uint32_t cntrNum, CSL_ArmR5PmuEventType eventType );

/**
 *  \brief Enable/disable overflow interrupt generation for a PMU counter
 *
 *  Requirement: REQ_TAG(PDK-6048)
 *  Design: did_csl_core_pmu_configure
 *
 *  This function is used to enable or disable overflow interrupt generation for the specified PMU counter.
 *
 *  \param cntrNum      [IN]    Counter number (0..(#CSL_armR5PmuGetNumCntrs()-1))
 *                              or CSL_ARM_R5_PMU_CYCLE_COUNTER_NUM)
 *  \param enable       [IN]    0=Overflow interrupt generation is disabled,
 *                              otherwise it is enabled
 *
 *  \return None
 */
extern void CSL_armR5PmuEnableCntrOverflowIntr( uint32_t cntrNum, uint32_t enable );

/**
 *  \brief Enable/disable a PMU counter
 *
 *  Requirement: REQ_TAG(PDK-6048)
 *  Design: did_csl_core_pmu_configure
 *
 *  This function is used to enable or disable the specified PMU counter.
 *  Note that to enable a specific counter, both this function and the
 *  #CSL_armR5PmuEnableAllCntrs function must be called.
 *
 *  \param cntrNum      [IN]    Counter number (0..(CSL_armR5PmuGetNumCntrs()-1))
 *                              or CSL_ARM_R5_PMU_CYCLE_COUNTER_NUM)
 *  \param enable       [IN]    0=Disable counter, otherwise it is enabled
 *
 *  \return None
 */
extern void CSL_armR5PmuEnableCntr( uint32_t cntrNum, uint32_t enable );

/**
 *  \brief Read a PMU counter
 *
 *  Requirement: REQ_TAG(PDK-6048)
 *  Design: did_csl_core_pmu_read
 *
 *  This function is used to read the specified PMU counter.
 *
 *  \param cntrNum      [IN]    Counter number (0..(CSL_armR5PmuGetNumCntrs()-1))
 *                              or CSL_ARM_R5_PMU_CYCLE_COUNTER_NUM)
 *
 *  \return The current count associated with the specified PMU counter
 */
extern uint32_t CSL_armR5PmuReadCntr( uint32_t cntrNum );

/**
 *  \brief Set a PMU counter
 *
 *  Requirement: REQ_TAG(PDK-6026)
 *  Design: did_csl_core_pmu_set
 *
 *  This function is used to set the specified PMU counter.
 *
 *  \param cntrNum      [IN]    Counter number (0..(CSL_armR5PmuGetNumCntrs()-1))
 *                              or CSL_ARM_R5_PMU_CYCLE_COUNTER_NUM)
 *  \param cntrVal      [IN]    Counter Value to be set
 *
 *  \return None
 */
extern uint32_t CSL_armR5PmuSetCntr( uint32_t cntrNum, uint32_t cntrVal);

/**
 *  \brief Read the overflow status for all of the counters
 *
 *  Requirement: REQ_TAG(PDK-6048)
 *  Design: did_csl_core_pmu_read
 *
 *  This function is used to read the overflow status for all of the counters.
 *  A bit-mask is returned where bits set to '1' indicate overflow occurred for
 *  the corresponding counter. See the 'Overflow Flag Status Register'
 *  description in the ARM R5 TRM for the format of this bitmask.
 *
 *  \return the value of PMOVSR register
 */
extern uint32_t CSL_armR5PmuReadCntrOverflowStatus( void );

/**
 *  \brief Clear the overflow flag for the specified counter(s)
 *
 *  Requirement: REQ_TAG(PDK-6048)
 *  Design: did_csl_core_pmu_configure
 *
 *  This function is used to clear the overflow flag for the specified
 *  counter(s). The cntrMask argument is a bit-mask where bits set to '1'
 *  indicate which counter(s)' overflow flag to clear. See the 'Overflow
 *  Flag Status Register' description in the ARM R5 TRM for the format of
 *  this bitmask.
 *
 *  \param cntrMask     [IN]    Bit-mask indicating which counter(s)' overflow
 *                      flag to clear
 *
 *  \return None
 */
extern void CSL_armR5PmuClearCntrOverflowStatus( uint32_t cntrMask );

/**
 *  \brief Reset the cycle counter to zero
 *
 *  Requirement: REQ_TAG(PDK-6048)
 *  Design: did_csl_core_pmu_configure
 *
 *  This function resets the cycle counter to zero.
 *
 *
 *  \return None
 */
extern void CSL_armR5PmuResetCycleCnt( void );

/**
 *  \brief Reset all counters to zero
 *
 *  Requirement: REQ_TAG(PDK-6048)
 *  Design: did_csl_core_pmu_configure
 *
 *  This function resets all event counters to zero.
 *
 *
 *  \return None
 */
extern void CSL_armR5PmuResetCntrs( void );

/* @} */

#ifdef __cplusplus
}
#endif  /* extern "C" */

#endif  /* end of CSL_ARM_R5_H_ definition */
