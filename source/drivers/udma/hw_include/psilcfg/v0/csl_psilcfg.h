/* =============================================================================
 *  Copyright (c) 2024 Texas Instruments Incorporated
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

/**
 *  \file   csl_psilcfg.h
 *
 *  \brief  This is the CSL header file for the Packet Streaming Interface
 *          Link (PSI-L) configuration CSL-FL.
 */
#ifndef CSL_PSILCFG_H_
#define CSL_PSILCFG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <drivers/udma/hw_include/cslr_psilcfg.h>

/** ===========================================================================
 *
 * @defgroup CSL_PSILCFG_API Packet Streaming Interface Link (PSI-L) configuration CSL-FL API
 *
 * ============================================================================
 */
/**
@defgroup CSL_PSILCFG_DATASTRUCT  PSILCFG Data Structures
@ingroup CSL_PSILCFG_API
*/
/**
@defgroup CSL_PSILCFG_FUNCTION  PSILCFG Functions
@ingroup CSL_PSILCFG_API
*/
/**
@defgroup CSL_PSILCFG_ENUM PSILCFG Enumerated Data Types
@ingroup CSL_PSILCFG_API
*/

/**
 *  \addtogroup CSL_PSILCFG_ENUM
 *  @{
 */

/** ---------------------------------------------------------------------------
 * @brief PSI-L Configuration Registers
 *
 *  \anchor CSL_PsilCfgReg
 *  \name PSI-L Configuration Registers
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef uint32_t CSL_PsilCfgReg;

/**< Peer thread ID register (implemented for src threads only) */
#define CSL_PSILCFG_REG_PEER_THREAD_ID          ((uint32_t) 0U)
/**< Peer credit register (implemented for src threads only) */
#define CSL_PSILCFG_REG_PEER_CREDIT             ((uint32_t) 0x001U)
/**< Enable register */
#define CSL_PSILCFG_REG_ENABLE                  ((uint32_t) 0x002U)
/**< Capabilities register */
#define CSL_PSILCFG_REG_CAPABILITIES            ((uint32_t) 0x040U)
/**< Static TR register */
#define CSL_PSILCFG_REG_STATIC_TR               ((uint32_t) 0x400U)
/**< Static TR Z register */
#define CSL_PSILCFG_REG_STATIC_TR_Z             ((uint32_t) 0x401U)
/**< Byte Count register */
#define CSL_PSILCFG_REG_BYTE_COUNT              ((uint32_t) 0x404U)
/**< AASRC Tx/Rx FIFO configuration register */
#define CSL_PSILCFG_REG_AASRC_FIFO_CFG          ((uint32_t) 0x405U)
/**< AASRC Tx/Rx order table 0 register */
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE0      ((uint32_t) 0x406U)
/**< AASRC Tx/Rx order table 1 register */
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE1      ((uint32_t) 0x407U)
/**< Realtime enable register */
#define CSL_PSILCFG_REG_RT_ENABLE               ((uint32_t) 0x408U)
/**< Local to global event translation registers
   Note that these 16 PSIL registers (N=0..15) are only applicable for
   source thread 0 on PDMA instance pdma_main1 on the AM654x SOC. */
#define CSL_PSILCFG_REG_LOCAL_TO_GLOBAL_MAPPING_E(N)        ((uint32_t)0x4000U + (uint32_t)(N))
/* @} */

/* PSILCFG_REG_PEER_THREAD_ID */
#define CSL_PSILCFG_REG_PEER_THREAD_ID_THREAD_ID_SHIFT      (0)
#define CSL_PSILCFG_REG_PEER_THREAD_ID_THREAD_ID_MASK       ((uint32_t)0xFFFFU<<CSL_PSILCFG_REG_PEER_THREAD_ID_THREAD_ID_SHIFT)
#define CSL_PSILCFG_REG_PEER_THREAD_ID_THREAD_WIDTH_SHIFT   (24U)
#define CSL_PSILCFG_REG_PEER_THREAD_ID_THREAD_WIDTH_MASK    ((uint32_t)0x1FU<<CSL_PSILCFG_REG_PEER_THREAD_ID_THREAD_WIDTH_SHIFT)
#define CSL_PSILCFG_REG_PEER_THREAD_ID_THREAD_PRI_SHIFT     (29U)
#define CSL_PSILCFG_REG_PEER_THREAD_ID_THREAD_PRI_MASK      ((uint32_t)0x7U<<CSL_PSILCFG_REG_PEER_THREAD_ID_THREAD_PRI_SHIFT)

/* PSILCFG_REG_PEER_CREDIT */
#define CSL_PSILCFG_REG_PEER_CREDIT_CNT_SHIFT               (0)
#define CSL_PSILCFG_REG_PEER_CREDIT_CNT_MASK                ((uint32_t)0xFFU<<CSL_PSILCFG_REG_PEER_CREDIT_CNT_SHIFT)

/* PSILCFG_REG_ENABLE */
#define CSL_PSILCFG_REG_ENABLE_ENABLE_SHIFT                 (31U)
#define CSL_PSILCFG_REG_ENABLE_ENABLE_MASK                  ((uint32_t)0x01U<<CSL_PSILCFG_REG_ENABLE_ENABLE_SHIFT)
#define CSL_PSILCFG_REG_ENABLE_TEARDOWN_SHIFT               (30U)
#define CSL_PSILCFG_REG_ENABLE_TEARDOWN_MASK                ((uint32_t)0x01U<<CSL_PSILCFG_REG_ENABLE_TEARDOWN_SHIFT)

/* PSILCFG_REG_CAPABILITIES */
#define CSL_PSILCFG_REG_CAPABILITIES_CREDIT_CNT_SHIFT       (0)
#define CSL_PSILCFG_REG_CAPABILITIES_CREDIT_CNT_MASK        ((uint32_t)0xFFU<<CSL_PSILCFG_REG_CAPABILITIES_CREDIT_CNT_SHIFT)
#define CSL_PSILCFG_REG_CAPABILITIES_THREAD_WIDTH_SHIFT     (24U)
#define CSL_PSILCFG_REG_CAPABILITIES_THREAD_WIDTH_MASK      ((uint32_t)0x1FU<<CSL_PSILCFG_REG_CAPABILITIES_THREAD_WIDTH_SHIFT)

/* PSILCFG_REG_STATIC_TR */
#define CSL_PSILCFG_REG_STATIC_TR_X_SHIFT                   (24U)
#define CSL_PSILCFG_REG_STATIC_TR_X_MASK                    (((uint32_t)0x0007U) << CSL_PSILCFG_REG_STATIC_TR_X_SHIFT)
#define CSL_PSILCFG_REG_STATIC_TR_Y_SHIFT                   (0U)
#define CSL_PSILCFG_REG_STATIC_TR_Y_MASK                    (((uint32_t)0x0FFFU) << CSL_PSILCFG_REG_STATIC_TR_Y_SHIFT)
#define CSL_PSILCFG_REG_STATIC_TR_Z_SHIFT                   (0U)
#define CSL_PSILCFG_REG_STATIC_TR_Z_MASK                    (((uint32_t)0x0FFFU) << CSL_PSILCFG_REG_STATIC_TR_Z_SHIFT)

/* PSILCFG_REG_RT_ENABLE */
#define CSL_PSILCFG_REG_RT_ENABLE_IDLE_SHIFT                (1U)
#define CSL_PSILCFG_REG_RT_ENABLE_IDLE_MASK                 ((uint32_t)0x01U<<CSL_PSILCFG_REG_RT_ENABLE_IDLE_SHIFT)
#define CSL_PSILCFG_REG_RT_ENABLE_FLUSH_SHIFT               (28U)
#define CSL_PSILCFG_REG_RT_ENABLE_FLUSH_MASK                ((uint32_t)0x01U << CSL_PSILCFG_REG_RT_ENABLE_FLUSH_SHIFT)
#define CSL_PSILCFG_REG_RT_ENABLE_PAUSE_SHIFT               (29U)
#define CSL_PSILCFG_REG_RT_ENABLE_PAUSE_MASK                ((uint32_t)0x01U<<CSL_PSILCFG_REG_RT_ENABLE_PAUSE_SHIFT)
#define CSL_PSILCFG_REG_RT_ENABLE_TDOWN_SHIFT               (30U)
#define CSL_PSILCFG_REG_RT_ENABLE_TDOWN_MASK                ((uint32_t)0x01U<<CSL_PSILCFG_REG_RT_ENABLE_TDOWN_SHIFT)
#define CSL_PSILCFG_REG_RT_ENABLE_ENABLE_SHIFT              (31U)
#define CSL_PSILCFG_REG_RT_ENABLE_ENABLE_MASK               ((uint32_t)0x01U<<CSL_PSILCFG_REG_RT_ENABLE_ENABLE_SHIFT)

/* PSILCFG_REG_AASRC_FIFO_CFG */
#define CSL_PSILCFG_REG_AASRC_FIFO_CFG_DMA_REQ_MASK_SHIFT   (0U)
#define CSL_PSILCFG_REG_AASRC_FIFO_CFG_DMA_REQ_MASK_MASK    ((uint32_t)0x0FFFFU<<CSL_PSILCFG_REG_AASRC_FIFO_CFG_DMA_REQ_MASK_SHIFT)
#define CSL_PSILCFG_REG_AASRC_FIFO_CFG_FIRST_SLOT_SHIFT     (16U)
#define CSL_PSILCFG_REG_AASRC_FIFO_CFG_FIRST_SLOT_MASK      ((uint32_t)0x0FU<<CSL_PSILCFG_REG_AASRC_FIFO_CFG_FIRST_SLOT_SHIFT)
#define CSL_PSILCFG_REG_AASRC_FIFO_CFG_LAST_SLOT_SHIFT      (20U)
#define CSL_PSILCFG_REG_AASRC_FIFO_CFG_LAST_SLOT_MASK       ((uint32_t)0x0FU<<CSL_PSILCFG_REG_AASRC_FIFO_CFG_LAST_SLOT_SHIFT)
#define CSL_PSILCFG_REG_AASRC_FIFO_CFG_DMA_REQ_RESET_SHIFT  (30U)
#define CSL_PSILCFG_REG_AASRC_FIFO_CFG_DMA_REQ_RESET_MASK   ((uint32_t)0x01U<<CSL_PSILCFG_REG_AASRC_FIFO_CFG_DMA_REQ_RESET_SHIFT)
#define CSL_PSILCFG_REG_AASRC_FIFO_CFG_GROUP_MODE_SHIFT     (31U)
#define CSL_PSILCFG_REG_AASRC_FIFO_CFG_GROUP_MODE_MASK      ((uint32_t)0x01U<<CSL_PSILCFG_REG_AASRC_FIFO_CFG_GROUP_MODE_SHIFT)

/* PSILCFG_REG_AASRC_ORDER_TABLE0 */
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE0_ENTRY0_SHIFT     (0U)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE0_ENTRY0_MASK      ((uint32_t)0x0FU<<CSL_PSILCFG_REG_AASRC_ORDER_TABLE0_ENTRY0_SHIFT)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE0_ENTRY1_SHIFT     (4U)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE0_ENTRY1_MASK      ((uint32_t)0x0FU<<CSL_PSILCFG_REG_AASRC_ORDER_TABLE0_ENTRY1_SHIFT)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE0_ENTRY2_SHIFT     (8U)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE0_ENTRY2_MASK      ((uint32_t)0x0FU<<CSL_PSILCFG_REG_AASRC_ORDER_TABLE0_ENTRY2_SHIFT)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE0_ENTRY3_SHIFT     (12U)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE0_ENTRY3_MASK      ((uint32_t)0x0FU<<CSL_PSILCFG_REG_AASRC_ORDER_TABLE0_ENTRY3_SHIFT)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE0_ENTRY4_SHIFT     (16U)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE0_ENTRY4_MASK      ((uint32_t)0x0FU<<CSL_PSILCFG_REG_AASRC_ORDER_TABLE0_ENTRY4_SHIFT)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE0_ENTRY5_SHIFT     (20U)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE0_ENTRY5_MASK      ((uint32_t)0x0FU<<CSL_PSILCFG_REG_AASRC_ORDER_TABLE0_ENTRY5_SHIFT)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE0_ENTRY6_SHIFT     (24U)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE0_ENTRY6_MASK      ((uint32_t)0x0FU<<CSL_PSILCFG_REG_AASRC_ORDER_TABLE0_ENTRY6_SHIFT)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE0_ENTRY7_SHIFT     (28U)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE0_ENTRY7_MASK      ((uint32_t)0x0FU<<CSL_PSILCFG_REG_AASRC_ORDER_TABLE0_ENTRY7_SHIFT)

/* PSILCFG_REG_AASRC_ORDER_TABLE1 */
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE1_ENTRY8_SHIFT     (0U)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE1_ENTRY8_MASK      ((uint32_t)0x0FU<<CSL_PSILCFG_REG_AASRC_ORDER_TABLE1_ENTRY8_SHIFT)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE1_ENTRY9_SHIFT     (4U)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE1_ENTRY9_MASK      ((uint32_t)0x0FU<<CSL_PSILCFG_REG_AASRC_ORDER_TABLE1_ENTRY9_SHIFT)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE1_ENTRY10_SHIFT    (8U)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE1_ENTRY10_MASK     ((uint32_t)0x0FU<<CSL_PSILCFG_REG_AASRC_ORDER_TABLE1_ENTRY10_SHIFT)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE1_ENTRY11_SHIFT    (12U)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE1_ENTRY11_MASK     ((uint32_t)0x0FU<<CSL_PSILCFG_REG_AASRC_ORDER_TABLE1_ENTRY11_SHIFT)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE1_ENTRY12_SHIFT    (16U)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE1_ENTRY12_MASK     ((uint32_t)0x0FU<<CSL_PSILCFG_REG_AASRC_ORDER_TABLE1_ENTRY12_SHIFT)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE1_ENTRY13_SHIFT    (20U)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE1_ENTRY13_MASK     ((uint32_t)0x0FU<<CSL_PSILCFG_REG_AASRC_ORDER_TABLE1_ENTRY13_SHIFT)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE1_ENTRY14_SHIFT    (24U)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE1_ENTRY14_MASK     ((uint32_t)0x0FU<<CSL_PSILCFG_REG_AASRC_ORDER_TABLE1_ENTRY14_SHIFT)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE1_ENTRY15_SHIFT    (28U)
#define CSL_PSILCFG_REG_AASRC_ORDER_TABLE1_ENTRY15_MASK     ((uint32_t)0x0FU<<CSL_PSILCFG_REG_AASRC_ORDER_TABLE1_ENTRY15_SHIFT)

/*  PSILCFG_REG_LOCAL_TO_GLOBAL_MAPPING_E */
#define CSL_PSILCFG_REG_LOCAL_TO_GLOBAL_MAPPING_E_INDEX_SHIFT       (0U)
#define CSL_PSILCFG_REG_LOCAL_TO_GLOBAL_MAPPING_E_INDEX_MASK        ((uint32_t)0xFFFFU<<CSL_PSILCFG_REG_LOCAL_TO_GLOBAL_MAPPING_E_INDEX_SHIFT)
#define CSL_PSILCFG_REG_LOCAL_TO_GLOBAL_MAPPING_E_TYPE_SHIFT        (31U)
#define CSL_PSILCFG_REG_LOCAL_TO_GLOBAL_MAPPING_E_TYPE_MASK         ((uint32_t)0x1U<<CSL_PSILCFG_REG_LOCAL_TO_GLOBAL_MAPPING_E_TYPE_SHIFT)
#define CSL_PSILCFG_REG_LOCAL_TO_GLOBAL_MAPPING_E_TYPE_PULSE        ((uint32_t) 0U)
#define CSL_PSILCFG_REG_LOCAL_TO_GLOBAL_MAPPING_E_TYPE_RISING_EDGE  ((uint32_t) 1U)

/* @} */

/**
 *  \addtogroup CSL_PSILCFG_FUNCTION
 *  @{
 */

/**
 *  \brief Return revision of the PSILCFG module.
 *
 *  This function returns the contents of the PSILCFG revision register.
 *  Consult the PSILCFG module documentation for a description of the
 *  contents of the revision register.
 *
 *  \param pRegs    [IN]    Pointer to the CSL_psilcfgRegs register structure
 *
 *  \return The 32-bit revision register is returned.
 */
extern uint32_t CSL_psilcfgGetRevision( const CSL_psilcfgRegs *pRegs );

/**
 *  \brief Write a value to a PSI-L periperal using the PSI-L configuration
 *  proxy.
 *
 *  This function submits a write command to the PSI-L periperal specified by
 *  the thread 'threadId' at address specified by 'regId'. It then waits for the
 *  write command to complete before returning.
 *
 *  Note: This command will return false if the PSI-L peripheral specified by
 *  the thread 'threadId' is invalid, the regId is invalid, or the peripheral
 *  is powered down or is in reset.
 *
 *  \param pRegs    [IN]    Pointer to the CSL_psilcfgRegs register structure
 *  \param threadId [IN]    Thread identifying the PSI-L peripheral to write to
 *  \param regId    [IN]    Register of the PSI-L peripheral to write to. See
 *                          \ref CSL_PsilCfgReg.
 *  \param data     [IN]    Data to write
 *
 *  \return true  = The write command completed successfully
 *          false = The write command timed out. Check the threadId and regId
 *                  parameters to make sure they are valid and that the
 *                  targeted PSI-L peripheral is powered up and unreset.
 */
extern bool CSL_psilcfgWrite( const CSL_psilcfgRegs *pRegs, uint32_t threadId, uint32_t regId, uint32_t data );

/**
 *  \brief Read a value from a PSI-L periperal using the PSI-L configuration
 *  proxy.
 *
 *  This function submits a read command to the PSI-L periperal specified by
 *  the thread 'threadId' at address specified by 'regId', waits for the read
 *  command to complete, then reads and returns the data value in pData.
 *
 *  Note: This command will return false if the PSI-L peripheral specified by
 *  the thread 'threadId' is invalid, the regId is invalid, or the peripheral
 *  is powered down or is in reset.
 *
 *  \param pRegs    [IN]    Pointer to the CSL_psilcfgRegs register structure
 *  \param threadId [IN]    Thread identifying the PSI-L peripheral to write to
 *  \param regId    [IN]    Register of the PSI-L peripheral to read from. See
 *                          \ref CSL_PsilCfgReg.
 *  \param pData    [OUT]   A pointer where to store the 32-bit data value read
 *
 *  \return true  = The read command completed successfully
 *          false = The read command timed out. Check the threadId and regId
 *                  parameters to make sure they are valid and that the
 *                  targeted PSI-L peripheral is powered up and unreset.
 */
extern bool CSL_psilcfgRead( const CSL_psilcfgRegs *pRegs, uint32_t threadId, uint32_t regId, uint32_t *pData );

/**
 *  \brief Enable or disable a thread
 *
 *  This function enables or disables the specified thread.
 *
 *  Software should only disable a thread if the thread is to be reset before
 *  being re-enabled. Otherwise, a teardown command #CSL_psilcfgTeardownThread
 *  should be used to gracefully disable a thread.
 *
 *  \param pRegs    [IN]    Pointer to the CSL_psilcfgRegs register structure
 *  \param threadId [IN]    Thread identifying the PSI-L peripheral
 *  \param bEnable  [IN]    0 = thread is disabled, 1 = thread is enabled
 *
 *  \return true  = the command was successfully executed
 *          false = the command was not submitted because a prior command was
 *                  still busy
 */
extern bool CSL_psilcfgSetThreadEnable( const CSL_psilcfgRegs *pRegs, uint32_t threadId, bool bEnable );

/**
 *  \brief Enable or disable a thread via the realtime register
 *
 *  This function enables or disables the specified thread via the realtime
 *  register.
 *
 *  Software should only disable a thread if the thread is to be reset before
 *  being re-enabled. Otherwise, a teardown command #CSL_psilcfgTeardownThread
 *  should be used to gracefully disable a thread.
 *
 *  \param pRegs    [IN]    Pointer to the CSL_psilcfgRegs register structure
 *  \param threadId [IN]    Thread identifying the PSI-L peripheral
 *  \param bEnable  [IN]    0 = thread is disabled, 1 = thread is enabled
 *
 *  \return true  = the command was successfully executed
 *          false = the command was not submitted because a prior command was
 *                  still busy
 */
extern bool CSL_psilcfgSetThreadRealtimeEnable( const CSL_psilcfgRegs *pRegs, uint32_t threadId, bool bEnable );

/**
 *  \brief Teardown a thread
 *
 *  This function tears down the specified thread.
 *
 *  For a source thread:
 *
 *      A teardown command will stop transferring data on a boundary which is
 *      appropriate for the type of attached peripheral and clear and mask any
 *      peripheral specific functionality (DMA event counters, etc.).
 *
 *      After stopping data transfer, the source thread sends a 'NULL data'
 *      teardown message to the destination thread.
 *
 *      Once the thread teardown is complete and ready to be reused, the enable
 *      bit is cleared.
 *
 *  For a destination thread:
 *
 *      To perform a destination thread teardown, it is recommended that the
 *      teardown command be set in the source thread and it will automatically
 *      propagate to destination thread with the normal flow of peripheral data
 *      via the tdown bit.
 *
 *      As a result, if a destination thread is specified in this function, then
 *      nothing is done and false is returned.
 *
 *  \param pRegs    [IN]    Pointer to the CSL_psilcfgRegs register structure
 *  \param threadId [IN]    Thread identifying the PSI-L peripheral
 *
 *  \return true  = the command was successfully executed
 *          false = the command was not submitted because a prior command was
 *                  still busy, or a destination thread was specified
 */
extern bool CSL_psilcfgTeardownThread( const CSL_psilcfgRegs *pRegs, uint32_t threadId );

/**
 *  \brief Clear a thread's teardown and flush bits
 *
 *  This function clears the teardown and flush bits in the specified thread's
 *  PSIL realtime enable register.
 *
 *  Software can call this function following a thread teardown via the
 *  #CSL_psilcfgTeardownThread function to clear the thread's teardown and
 *  flush bits prior to re-enabling this thread again.
 *
 *  \param pRegs    [IN]    Pointer to the CSL_psilcfgRegs register structure
 *  \param threadId [IN]    Thread identifying the PSI-L peripheral
 *
 *  \return true  = the command was successfully executed
 *          false = the command was not submitted because a prior command was
 *                  still busy
 */
extern bool CSL_psilcfgClrTeardown( const CSL_psilcfgRegs *pRegs, uint32_t threadId );

/**
 *  \brief Flush data from a destination thread
 *
 *  This function flushes data in the specified destination thread.
 *
 *  This function only operates with destination threads. The flush command
 *  causes all destination thread data to be discarded instead of being
 *  written to the peripheral.
 *
 *  The flush command should be called only when a thread fails to complete
 *  its teardown procedure normally, because a peripheral is no longer
 *  functioning or because some other factor is causing a deadlock beyond
 *  the PSI-L interface.
 *
 *  \param pRegs    [IN]    Pointer to the CSL_psilcfgRegs register structure
 *  \param threadId [IN]    Thread identifying the PSI-L peripheral
 *
 *  \return true  = the command was successfully executed
 *          false = the command was not submitted because a prior command was
 *                  still busy, or the specified thread is not a destinaition
 *                  thread
 */
extern bool CSL_psilcfgFlushThread( const CSL_psilcfgRegs *pRegs, uint32_t threadId );

/**
 *  \brief Pause or un-pause a thread
 *
 *  This function pauses or un-pauses the specified thread.
 *
 *  While paused, data transfers will no longer occur but other application
 *  specific actions may still occur (DMA event increments, etc.).
 *
 *  \param pRegs    [IN]    Pointer to the CSL_psilcfgRegs register structure
 *  \param threadId [IN]    Thread identifying the PSI-L peripheral
 *  \param bPause   [IN]    0 = thread is un-paused, 1 = thread is paused
 *
 *  \return true  = the command was successfully executed
 *          false = the command was not submitted because a prior command was
 *                  still busy
 */
extern bool CSL_psilcfgSetThreadPause( const CSL_psilcfgRegs *pRegs, uint32_t threadId, bool bPause );

/**
 *  \brief Create a route through the PSI-L switch
 *
 *  This function creates a route through the PSI-L switch between the
 *  specified source and destination thread ID's. It reads the thread width and
 *  credit count from the destination thread and sets the source thread to use
 *  these values. It then enables the credit passing functionality for both
 *  threads.
 *
 *  In addition, this function also enables both threads via their psil real-time
 *  enable configuration register.
 *
 *  srcThreadId must not have bit 15 set to 1. If it does, this function does
 *  nothing and false is returned.
 *
 *  This function forces bit 15 of dstThreadId to 1.
 *
 *  \param pRegs        [IN]    Pointer to the CSL_psilcfgRegs register structure
 *  \param srcThreadId  [IN]    Thread identifying the source PSI-L peripheral
 *  \param dstThreadId  [IN]    Thread identifying the destination PSI-L peripheral
 *
 *  \return true if successful, false if 1) bit 15 is set in srcThreadId, or
 *          2) a transaction timeout occurs
 */
extern bool CSL_psilcfgCreateRoute( const CSL_psilcfgRegs *pRegs, uint32_t srcThreadId, uint32_t dstThreadId );

/**
 *  \brief Create a link through the PSI-L switch
 *
 *  This function creates a link through the PSI-L switch between the
 *  specified source and destination thread ID's. It reads the thread width and
 *  credit count from the destination thread and sets the source thread to use
 *  these values. It then enables the credit passing functionality for both
 *  threads.
 *
 *  Unlike the CSL_psilcfgCreateRoute() function, this function does not
 *  enable either thread via their real-time enable configuration register.
 *  Software must use the udmap CSL-FL CSL_udmapEnableLink() function to
 *  accomplish this.
 *
 *  srcThreadId must not have bit 15 set to 1. If it does, this function does
 *  nothing and false is returned.
 *
 *  This function forces bit 15 of dstThreadId to 1.
 *
 *  \param pRegs        [IN]    Pointer to the CSL_psilcfgRegs register structure
 *  \param srcThreadId  [IN]    Thread identifying the source PSI-L peripheral
 *  \param dstThreadId  [IN]    Thread identifying the destination PSI-L peripheral
 *
 *  \return true if successful, false if 1) bit 15 is set in srcThreadId, or
 *          2) a transaction timeout occurs
 */
extern bool CSL_psilcfgCreateLink( const CSL_psilcfgRegs *pRegs, uint32_t srcThreadId, uint32_t dstThreadId );

/**
 *  \brief Determine if a disabled or paused thread is idle
 *
 *  This function returns the idle status of the specified thread.
 *
 *  A thread will return a true idle status under the following conditions:
 *  o For destination threads: the thread is disabled and is also idle (no active transactions)
 *  o For source threads: the thread is paused or disabled and is also idle (no active transactions)
 *
 *  \param pRegs    [IN]    Pointer to the CSL_psilcfgRegs register structure
 *  \param threadId [IN]    Thread identifying the PSI-L peripheral
 *
 *  \return true  = the thread is idle
 *          false = the thread is not idle
 */
extern bool CSL_psilcfgIsThreadIdle( const CSL_psilcfgRegs *pRegs, uint32_t threadId );

/* @} */

#ifdef __cplusplus
}
#endif

#endif
