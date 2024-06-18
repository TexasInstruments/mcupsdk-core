/*
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
 */

/**
 *  \file udma_soc.h
 *
 *  \brief UDMA Low Level Driver AM65xx SOC specific file.
 */

#ifndef UDMA_SOC_H_
#define UDMA_SOC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/hw_include/csl_ringacc.h>
#include <drivers/udma/hw_include/csl_psilcfg.h>
#include <drivers/udma/hw_include/csl_proxy.h>
#include <drivers/udma/include/udma_proxy.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  \anchor Udma_InstanceIdSoc
 *  \name UDMA Instance ID specific to SOC
 *
 *  UDMA instance ID - Main/MCU NAVSS
 *
 *  @{
 */
/** \brief Main NAVSS UDMA instance */
#define UDMA_INST_ID_MAIN_0             (UDMA_INST_ID_0)
/** \brief MCU NAVSS UDMA instance */
#define UDMA_INST_ID_MCU_0              (UDMA_INST_ID_1)
/** \brief Start of UDMA instance */
#define UDMA_INST_ID_START              (UDMA_INST_ID_0)
/** \brief Maximum number of UDMA instance */
#define UDMA_INST_ID_MAX                (UDMA_INST_ID_1)
/** \brief Total number of UDMA instances */
#define UDMA_NUM_INST_ID                (UDMA_INST_ID_MAX - UDMA_INST_ID_START + 1U)
/** @} */

 /**
 *  \anchor Udma_SocCfg
 *  \name UDMA SOC Configuration
 *
 *  UDMA Soc Cfg - Flags to indicate the presnce of various SOC specific modules.
 *
 *  @{
 */
/** \brief Flag to indicate UDMAP module is present or not in the SOC*/
#define UDMA_SOC_CFG_UDMAP_PRESENT               (1U)

/** \brief Flag to indicate LCDMA module is present or not in the SOC*/
#define UDMA_SOC_CFG_LCDMA_PRESENT               (0U)

/** \brief Flag to indicate Proxy is present or not in the SOC*/
#define UDMA_SOC_CFG_PROXY_PRESENT               (1U)

/** \brief Flag to indicate Clec is present or not in the SOC*/
#define UDMA_SOC_CFG_CLEC_PRESENT                (1U)

/** \brief Flag to indicate Normal RA is present or not in the SOC*/
#define UDMA_SOC_CFG_RA_NORMAL_PRESENT           (1U)

/** \brief Flag to indicate LCDMA RA is present or not in the SOC*/
#define UDMA_SOC_CFG_RA_LCDMA_PRESENT            (0U)

/** \brief Flag to indicate Ring Monitor is present or not in the SOC*/
#define UDMA_SOC_CFG_RING_MON_PRESENT            (1U)

/** \brief Flag to indicate the SOC needs ring reset workaround */
#define UDMA_SOC_CFG_APPLY_RING_WORKAROUND		 (1U)
/** @} */

 /**
 *  \anchor Udma_TxChFdepth
 *  \name UDMA Tx Channels FDEPTH
 *
 *  UDMA Tx Ch Fdepth - Fdepth of various types of channels present in the SOC.
 *
 *  @{
 */
/** \brief Tx Ultra High Capacity Channel FDEPTH*/
#define UDMA_TX_UHC_CHANS_FDEPTH         (CSL_NAVSS_UDMAP_TX_UHC_CHANS_FDEPTH)
/** \brief Tx High Capacity Channel FDEPTH*/
#define UDMA_TX_HC_CHANS_FDEPTH 		 (CSL_NAVSS_UDMAP_TX_HC_CHANS_FDEPTH)
/** \brief Tx Normal Channel FDEPTH*/
#define UDMA_TX_CHANS_FDEPTH 			 (CSL_NAVSS_UDMAP_TX_CHANS_FDEPTH)
/** @} */

/**
 *  \anchor Udma_RingAccAselEndpointSoc
 *  \name UDMA Ringacc address select (asel) endpoint
 *
 *  List of all valid address select (asel) endpoints in the SOC.
 *
 *  @{
 */
/** \brief Physical address (normal) */
#define UDMA_RINGACC_ASEL_ENDPOINT_PHYSADDR          (0U)
/** @} */


/** \brief Invalid Ring Mode*/
#define UDMA_RING_MODE_INVALID          (CSL_RINGACC_RING_MODE_INVALID)

/** \brief Number of Mapped TX Group */
#define UDMA_NUM_MAPPED_TX_GROUP     (0U)
/**
 *  \anchor Udma_MappedTxGrpSoc
 *  \name Mapped TX Group specific to a SOC
 *
 *  List of all mapped TX groups present in the SOC.
 *
 *  @{
 */
/* No mapped TX channels/rings in AM65XX */
/** @} */

/** \brief Number of Mapped RX Group */
#define UDMA_NUM_MAPPED_RX_GROUP     (0U)
/**
 *  \anchor Udma_MappedRxGrpSoc
 *  \name Mapped RX Group specific to a SOC
 *
 *  List of all mapped RX groups present in the SOC.
 *
 *  @{
 */
/* No mapped RX channels/rings in AM65XX */
/** @} */

/** \brief Number of UTC instance */
#define UDMA_NUM_UTC_INSTANCE           (CSL_NAVSS_UTC_CNT)

/**
 *  \anchor Udma_UtcIdSoc
 *  \name UTC ID specific to a SOC
 *
 *  List of all UTC's present in the SOC.
 *
 *  @{
 */
#define UDMA_UTC_ID_MSMC_DRU0           (UDMA_UTC_ID0)
/** @} */

/** \brief External start channel of DRU0 UTC */
#define UDMA_UTC_START_CH_DRU0          (0U)
/** \brief Number of channels present in DRU0 UTC */
#define UDMA_UTC_NUM_CH_DRU0            (CSL_PSILCFG_NAVSS_MAIN_MSMC0_PSILS_THREAD_CNT)
/** \brief Start thread ID of DRU0 UTC */
#define UDMA_UTC_START_THREAD_ID_DRU0   (CSL_PSILCFG_NAVSS_MAIN_MSMC0_PSILD_THREAD_OFFSET)
/** \brief DRU0 UTC baseaddress */
#define UDMA_UTC_BASE_DRU0              (CSL_COMPUTE_CLUSTER0_DRU_BASE)

/**
 *  \anchor Udma_CoreId
 *  \name Core ID specific to a SOC
 *
 *  List of all cores present in the SOC.
 *
 *  @{
 */
/*
 * Locally used core ID to define default RM configuration.
 * Not to be used by caller
 */
/* Main domain cores */
#define UDMA_CORE_ID_MPU1_0             (0U)
#define UDMA_NUM_MAIN_CORE              (1U)
/* MCU domain cores - Note: This should be after all main domain cores */
#define UDMA_CORE_ID_MCU1_0             (UDMA_NUM_MAIN_CORE + 0U)
#define UDMA_CORE_ID_MCU1_1             (UDMA_NUM_MAIN_CORE + 1U)
#define UDMA_NUM_MCU_CORE               (2U)
/* Total number of cores */
#define UDMA_NUM_CORE                   (UDMA_NUM_MAIN_CORE + UDMA_NUM_MCU_CORE)
/** @} */

/**
 *  \anchor Udma_DruSubmitCoreId
 *  \name DRU core ID register to use for direct TR submission.
 *   Each CPU should have a unique submit register to avoid corrupting
 *   submit word when SW is running from multiple CPU at the same time
 *
 *  List of all DRU cores ID to use for all the CPUs present in the SOC.
 *
 *  @{
 */
#define UDMA_DRU_CORE_ID_MPU1_0         (CSL_DRU_CORE_ID_0)
#define UDMA_DRU_CORE_ID_MCU1_0         (CSL_DRU_CORE_ID_1)
#define UDMA_DRU_CORE_ID_MCU1_1         (CSL_DRU_CORE_ID_2)
/** @} */

/**
 *  \anchor Udma_RmResId
 *  \name UDMA Resources ID
 *
 *  List of all UDMA Resources Id's.
 *
 *  @{
 */
/** \brief Ultra High Capacity TX and Block Copy Channels */
#define UDMA_RM_RES_ID_TX_UHC                   (0U)
/** \brief High Capacity TX and Block Copy Channels */
#define UDMA_RM_RES_ID_TX_HC                    (1U)
/** \brief Normal Capacity TX and Block Copy Channels */
#define UDMA_RM_RES_ID_TX                       (2U)
/** \brief Ultra High Capacity RX Channels */
#define UDMA_RM_RES_ID_RX_UHC                   (3U)
/** \brief High Capacity RX Channels */
#define UDMA_RM_RES_ID_RX_HC                    (4U)
/** \brief Normal Capacity RX Channels */
#define UDMA_RM_RES_ID_RX                       (5U)
/** \brief UTC - Extended Channels (MSMC_DRU) */
#define UDMA_RM_RES_ID_UTC                      (6U)
/** \brief Free Flows */
#define UDMA_RM_RES_ID_RX_FLOW                  (7U)
/** \brief Free Rings */
#define UDMA_RM_RES_ID_RING                     (8U)
/** \brief Global Event */
#define UDMA_RM_RES_ID_GLOBAL_EVENT             (9U)
/** \brief Virtual Interrupts */
#define UDMA_RM_RES_ID_VINTR                    (10U)
/** \brief Interrupt Router Interrupts */
#define UDMA_RM_RES_ID_IR_INTR                  (11U)
/** \brief Proxy */
#define UDMA_RM_RES_ID_PROXY                    (12U)
/** \brief Ring Monitors */
#define UDMA_RM_RES_ID_RING_MON                 (13U)
/** \brief Total number of resources */
#define UDMA_RM_NUM_RES                         (14U)
/** @} */

/** \brief Total number of shared resources -
 *  Global_Event/IR Intr/VINT */
#define UDMA_RM_NUM_SHARED_RES                  (3U)
/** \brief Maximum no.of instances to split a shared resource.
 *  This should be max(UDMA_NUM_CORE,UDMA_NUM_INST_ID) */
#define UDMA_RM_SHARED_RES_MAX_INST             (UDMA_NUM_CORE)

/**
 *  \anchor Udma_PsilCh
 *  \name PSIL Channels
 *
 *  List of all PSIL channels across MCU and main domains
 *
 *  @{
 */

/**
 *  \anchor Udma_PsilChMain
 *  \name Main PSIL Channels
 *
 *  List of all Main PSIL channels and the corresponding counts
 *
 *  @{
 */
#define UDMA_PSIL_CH_MAIN_SAUL0_TX          (CSL_PSILCFG_NAVSS_MAIN_SAUL0_PSILD_THREAD_OFFSET)
#define UDMA_PSIL_CH_MAIN_ICSS_G0_TX        (CSL_PSILCFG_NAVSS_MAIN_ICSS_G0_PSILD_THREAD_OFFSET)
#define UDMA_PSIL_CH_MAIN_ICSS_G1_TX        (CSL_PSILCFG_NAVSS_MAIN_ICSS_G1_PSILD_THREAD_OFFSET)
#define UDMA_PSIL_CH_MAIN_ICSS_G2_TX        (CSL_PSILCFG_NAVSS_MAIN_ICSS_G2_PSILD_THREAD_OFFSET)
#define UDMA_PSIL_CH_MAIN_CAL0_TX           (CSL_PSILCFG_NAVSS_MAIN_CAL0_PSILD_THREAD_OFFSET)

#define UDMA_PSIL_CH_MAIN_SAUL0_RX          (CSL_PSILCFG_NAVSS_MAIN_SAUL0_PSILS_THREAD_OFFSET)
#define UDMA_PSIL_CH_MAIN_ICSS_G0_RX        (CSL_PSILCFG_NAVSS_MAIN_ICSS_G0_PSILS_THREAD_OFFSET)
#define UDMA_PSIL_CH_MAIN_ICSS_G1_RX        (CSL_PSILCFG_NAVSS_MAIN_ICSS_G1_PSILS_THREAD_OFFSET)
#define UDMA_PSIL_CH_MAIN_ICSS_G2_RX        (CSL_PSILCFG_NAVSS_MAIN_ICSS_G2_PSILS_THREAD_OFFSET)
#define UDMA_PSIL_CH_MAIN_CAL0_RX           (CSL_PSILCFG_NAVSS_MAIN_CAL0_PSILS_THREAD_OFFSET)

#define UDMA_PSIL_CH_MAIN_SAUL0_TX_CNT      (CSL_PSILCFG_NAVSS_MAIN_SAUL0_PSILD_THREAD_CNT)
#define UDMA_PSIL_CH_MAIN_ICSS_G0_TX_CNT    (CSL_PSILCFG_NAVSS_MAIN_ICSS_G0_PSILD_THREAD_CNT)
#define UDMA_PSIL_CH_MAIN_ICSS_G1_TX_CNT    (CSL_PSILCFG_NAVSS_MAIN_ICSS_G1_PSILD_THREAD_CNT)
#define UDMA_PSIL_CH_MAIN_ICSS_G2_TX_CNT    (CSL_PSILCFG_NAVSS_MAIN_ICSS_G2_PSILD_THREAD_CNT)
#define UDMA_PSIL_CH_MAIN_CAL0_TX_CNT       (CSL_PSILCFG_NAVSS_MAIN_CAL0_PSILD_THREAD_CNT)

#define UDMA_PSIL_CH_MAIN_SAUL0_RX_CNT      (CSL_PSILCFG_NAVSS_MAIN_SAUL0_PSILS_THREAD_CNT)
#define UDMA_PSIL_CH_MAIN_ICSS_G0_RX_CNT    (CSL_PSILCFG_NAVSS_MAIN_ICSS_G0_PSILS_THREAD_CNT)
#define UDMA_PSIL_CH_MAIN_ICSS_G1_RX_CNT    (CSL_PSILCFG_NAVSS_MAIN_ICSS_G1_PSILS_THREAD_CNT)
#define UDMA_PSIL_CH_MAIN_ICSS_G2_RX_CNT    (CSL_PSILCFG_NAVSS_MAIN_ICSS_G2_PSILS_THREAD_CNT)
#define UDMA_PSIL_CH_MAIN_CAL0_RX_CNT       (CSL_PSILCFG_NAVSS_MAIN_CAL0_PSILS_THREAD_CNT)
/** @} */

/**
 *  \anchor Udma_PsilChMcu
 *  \name Mcu PSIL Channels
 *
 *  List of all Mcu PSIL channels and the corresponding counts
 *
 *  @{
 */
#define UDMA_PSIL_CH_MCU_CPSW0_TX           (CSL_PSILCFG_NAVSS_MCU_CPSW0_PSILD_THREAD_OFFSET)
#define UDMA_PSIL_CH_MCU_CPSW0_RX           (CSL_PSILCFG_NAVSS_MCU_CPSW0_PSILS_THREAD_OFFSET)

#define UDMA_PSIL_CH_MCU_CPSW0_TX_CNT       (CSL_PSILCFG_NAVSS_MCU_CPSW0_PSILD_THREAD_CNT)
#define UDMA_PSIL_CH_MCU_CPSW0_RX_CNT       (CSL_PSILCFG_NAVSS_MCU_CPSW0_PSILS_THREAD_CNT)
/** @} */

/** @} */

/**
 *  \anchor Udma_PdmaCh
 *  \name PDMA Channels
 *
 *  List of all PDMA channels across MCU and main domains
 *
 *  @{
 */

/**
 *  \anchor Udma_PdmaChMainTx
 *  \name Main TX PDMA Channels
 *
 *  List of all Main PDMA TX channels
 *
 *  @{
 */
/*
 * PDMA Main McASP TX Channels
 */
#define UDMA_PDMA_CH_MAIN_MCASP0_TX     (CSL_PDMA_CH_MAIN_MCASP0_CH0_TX)
#define UDMA_PDMA_CH_MAIN_MCASP1_TX     (CSL_PDMA_CH_MAIN_MCASP1_CH0_TX)
#define UDMA_PDMA_CH_MAIN_MCASP2_TX     (CSL_PDMA_CH_MAIN_MCASP2_CH0_TX)
/*
 * PDMA Main McSPI TX Channels
 */
#define UDMA_PDMA_CH_MAIN_MCSPI0_CH0_TX (CSL_PDMA_CH_MAIN_MCSPI0_CH0_TX)
#define UDMA_PDMA_CH_MAIN_MCSPI0_CH1_TX (CSL_PDMA_CH_MAIN_MCSPI0_CH1_TX)
#define UDMA_PDMA_CH_MAIN_MCSPI0_CH2_TX (CSL_PDMA_CH_MAIN_MCSPI0_CH2_TX)
#define UDMA_PDMA_CH_MAIN_MCSPI0_CH3_TX (CSL_PDMA_CH_MAIN_MCSPI0_CH3_TX)
#define UDMA_PDMA_CH_MAIN_MCSPI1_CH0_TX (CSL_PDMA_CH_MAIN_MCSPI1_CH0_TX)
#define UDMA_PDMA_CH_MAIN_MCSPI1_CH1_TX (CSL_PDMA_CH_MAIN_MCSPI1_CH1_TX)
#define UDMA_PDMA_CH_MAIN_MCSPI1_CH2_TX (CSL_PDMA_CH_MAIN_MCSPI1_CH2_TX)
#define UDMA_PDMA_CH_MAIN_MCSPI1_CH3_TX (CSL_PDMA_CH_MAIN_MCSPI1_CH3_TX)
#define UDMA_PDMA_CH_MAIN_MCSPI2_CH0_TX (CSL_PDMA_CH_MAIN_MCSPI2_CH0_TX)
#define UDMA_PDMA_CH_MAIN_MCSPI2_CH1_TX (CSL_PDMA_CH_MAIN_MCSPI2_CH1_TX)
#define UDMA_PDMA_CH_MAIN_MCSPI2_CH2_TX (CSL_PDMA_CH_MAIN_MCSPI2_CH2_TX)
#define UDMA_PDMA_CH_MAIN_MCSPI2_CH3_TX (CSL_PDMA_CH_MAIN_MCSPI2_CH3_TX)
#define UDMA_PDMA_CH_MAIN_MCSPI3_CH0_TX (CSL_PDMA_CH_MAIN_MCSPI3_CH0_TX)
#define UDMA_PDMA_CH_MAIN_MCSPI3_CH1_TX (CSL_PDMA_CH_MAIN_MCSPI3_CH1_TX)
#define UDMA_PDMA_CH_MAIN_MCSPI3_CH2_TX (CSL_PDMA_CH_MAIN_MCSPI3_CH2_TX)
#define UDMA_PDMA_CH_MAIN_MCSPI3_CH3_TX (CSL_PDMA_CH_MAIN_MCSPI3_CH3_TX)
#define UDMA_PDMA_CH_MAIN_MCSPI4_CH0_TX (CSL_PDMA_CH_MAIN_MCSPI4_CH0_TX)
#define UDMA_PDMA_CH_MAIN_MCSPI4_CH1_TX (CSL_PDMA_CH_MAIN_MCSPI4_CH1_TX)
#define UDMA_PDMA_CH_MAIN_MCSPI4_CH2_TX (CSL_PDMA_CH_MAIN_MCSPI4_CH2_TX)
#define UDMA_PDMA_CH_MAIN_MCSPI4_CH3_TX (CSL_PDMA_CH_MAIN_MCSPI4_CH3_TX)
/*
 * PDMA Main UART TX Channels
 */
#define UDMA_PDMA_CH_MAIN_UART0_TX      (CSL_PDMA_CH_MAIN_UART0_CH0_TX)
#define UDMA_PDMA_CH_MAIN_UART1_TX      (CSL_PDMA_CH_MAIN_UART1_CH0_TX)
#define UDMA_PDMA_CH_MAIN_UART2_TX      (CSL_PDMA_CH_MAIN_UART2_CH0_TX)
/** @} */

/**
 *  \anchor Udma_PdmaChMcuTx
 *  \name MCU TX PDMA Channels
 *
 *  List of all MCU PDMA TX channels
 *
 *  @{
 */
/*
 * PDMA MCU McSPI TX Channels
 */
#define UDMA_PDMA_CH_MCU_MCSPI0_CH0_TX  (CSL_PDMA_CH_MCU_MCSPI0_CH0_TX)
#define UDMA_PDMA_CH_MCU_MCSPI0_CH1_TX  (CSL_PDMA_CH_MCU_MCSPI0_CH1_TX)
#define UDMA_PDMA_CH_MCU_MCSPI0_CH2_TX  (CSL_PDMA_CH_MCU_MCSPI0_CH2_TX)
#define UDMA_PDMA_CH_MCU_MCSPI0_CH3_TX  (CSL_PDMA_CH_MCU_MCSPI0_CH3_TX)
#define UDMA_PDMA_CH_MCU_MCSPI1_CH0_TX  (CSL_PDMA_CH_MCU_MCSPI1_CH0_TX)
#define UDMA_PDMA_CH_MCU_MCSPI1_CH1_TX  (CSL_PDMA_CH_MCU_MCSPI1_CH1_TX)
#define UDMA_PDMA_CH_MCU_MCSPI1_CH2_TX  (CSL_PDMA_CH_MCU_MCSPI1_CH2_TX)
#define UDMA_PDMA_CH_MCU_MCSPI1_CH3_TX  (CSL_PDMA_CH_MCU_MCSPI1_CH3_TX)
#define UDMA_PDMA_CH_MCU_MCSPI2_CH0_TX  (CSL_PDMA_CH_MCU_MCSPI2_CH0_TX)
#define UDMA_PDMA_CH_MCU_MCSPI2_CH1_TX  (CSL_PDMA_CH_MCU_MCSPI2_CH1_TX)
#define UDMA_PDMA_CH_MCU_MCSPI2_CH2_TX  (CSL_PDMA_CH_MCU_MCSPI2_CH2_TX)
#define UDMA_PDMA_CH_MCU_MCSPI2_CH3_TX  (CSL_PDMA_CH_MCU_MCSPI2_CH3_TX)
/*
 * PDMA MCU MCAN TX Channels
 */
#define UDMA_PDMA_CH_MCU_MCAN0_CH0_TX   (CSL_PDMA_CH_MCU_MCAN0_CH0_TX)
#define UDMA_PDMA_CH_MCU_MCAN0_CH1_TX   (CSL_PDMA_CH_MCU_MCAN0_CH1_TX)
#define UDMA_PDMA_CH_MCU_MCAN0_CH2_TX   (CSL_PDMA_CH_MCU_MCAN0_CH2_TX)
#define UDMA_PDMA_CH_MCU_MCAN1_CH0_TX   (CSL_PDMA_CH_MCU_MCAN1_CH0_TX)
#define UDMA_PDMA_CH_MCU_MCAN1_CH1_TX   (CSL_PDMA_CH_MCU_MCAN1_CH1_TX)
#define UDMA_PDMA_CH_MCU_MCAN1_CH2_TX   (CSL_PDMA_CH_MCU_MCAN1_CH2_TX)
/*
 * PDMA MCU UART TX Channels
 */
#define UDMA_PDMA_CH_MCU_UART0_TX       (CSL_PDMA_CH_MCU_UART0_CH0_TX)
/** @} */

/**
 *  \anchor Udma_PdmaChMainRx
 *  \name Main RX PDMA Channels
 *
 *  List of all Main PDMA RX channels
 *
 *  @{
 */
/*
 * PDMA Main McASP RX Channels
 */
#define UDMA_PDMA_CH_MAIN_MCASP0_RX     (CSL_PDMA_CH_MAIN_MCASP0_CH0_RX)
#define UDMA_PDMA_CH_MAIN_MCASP1_RX     (CSL_PDMA_CH_MAIN_MCASP1_CH0_RX)
#define UDMA_PDMA_CH_MAIN_MCASP2_RX     (CSL_PDMA_CH_MAIN_MCASP2_CH0_RX)
/*
 * PDMA Main McSPI RX Channels
 */
#define UDMA_PDMA_CH_MAIN_MCSPI0_CH0_RX (CSL_PDMA_CH_MAIN_MCSPI0_CH0_RX)
#define UDMA_PDMA_CH_MAIN_MCSPI0_CH1_RX (CSL_PDMA_CH_MAIN_MCSPI0_CH1_RX)
#define UDMA_PDMA_CH_MAIN_MCSPI0_CH2_RX (CSL_PDMA_CH_MAIN_MCSPI0_CH2_RX)
#define UDMA_PDMA_CH_MAIN_MCSPI0_CH3_RX (CSL_PDMA_CH_MAIN_MCSPI0_CH3_RX)
#define UDMA_PDMA_CH_MAIN_MCSPI1_CH0_RX (CSL_PDMA_CH_MAIN_MCSPI1_CH0_RX)
#define UDMA_PDMA_CH_MAIN_MCSPI1_CH1_RX (CSL_PDMA_CH_MAIN_MCSPI1_CH1_RX)
#define UDMA_PDMA_CH_MAIN_MCSPI1_CH2_RX (CSL_PDMA_CH_MAIN_MCSPI1_CH2_RX)
#define UDMA_PDMA_CH_MAIN_MCSPI1_CH3_RX (CSL_PDMA_CH_MAIN_MCSPI1_CH3_RX)
#define UDMA_PDMA_CH_MAIN_MCSPI2_CH0_RX (CSL_PDMA_CH_MAIN_MCSPI2_CH0_RX)
#define UDMA_PDMA_CH_MAIN_MCSPI2_CH1_RX (CSL_PDMA_CH_MAIN_MCSPI2_CH1_RX)
#define UDMA_PDMA_CH_MAIN_MCSPI2_CH2_RX (CSL_PDMA_CH_MAIN_MCSPI2_CH2_RX)
#define UDMA_PDMA_CH_MAIN_MCSPI2_CH3_RX (CSL_PDMA_CH_MAIN_MCSPI2_CH3_RX)
#define UDMA_PDMA_CH_MAIN_MCSPI3_CH0_RX (CSL_PDMA_CH_MAIN_MCSPI3_CH0_RX)
#define UDMA_PDMA_CH_MAIN_MCSPI3_CH1_RX (CSL_PDMA_CH_MAIN_MCSPI3_CH1_RX)
#define UDMA_PDMA_CH_MAIN_MCSPI3_CH2_RX (CSL_PDMA_CH_MAIN_MCSPI3_CH2_RX)
#define UDMA_PDMA_CH_MAIN_MCSPI3_CH3_RX (CSL_PDMA_CH_MAIN_MCSPI3_CH3_RX)
#define UDMA_PDMA_CH_MAIN_MCSPI4_CH0_RX (CSL_PDMA_CH_MAIN_MCSPI4_CH0_RX)
#define UDMA_PDMA_CH_MAIN_MCSPI4_CH1_RX (CSL_PDMA_CH_MAIN_MCSPI4_CH1_RX)
#define UDMA_PDMA_CH_MAIN_MCSPI4_CH2_RX (CSL_PDMA_CH_MAIN_MCSPI4_CH2_RX)
#define UDMA_PDMA_CH_MAIN_MCSPI4_CH3_RX (CSL_PDMA_CH_MAIN_MCSPI4_CH3_RX)
/*
 * PDMA Main UART RX Channels
 */
#define UDMA_PDMA_CH_MAIN_UART0_RX      (CSL_PDMA_CH_MAIN_UART0_CH0_RX)
#define UDMA_PDMA_CH_MAIN_UART1_RX      (CSL_PDMA_CH_MAIN_UART1_CH0_RX)
#define UDMA_PDMA_CH_MAIN_UART2_RX      (CSL_PDMA_CH_MAIN_UART2_CH0_RX)
/** @} */

/**
 *  \anchor Udma_PdmaChMcuRx
 *  \name MCU RX PDMA Channels
 *
 *  List of all MCU PDMA RX channels
 *
 *  @{
 */
/*
 * PDMA MCU ADC RX Channels
 */
#define UDMA_PDMA_CH_MCU_ADC0_CH0_RX    (CSL_PDMA_CH_MCU_ADC0_CH0_RX)
#define UDMA_PDMA_CH_MCU_ADC0_CH1_RX    (CSL_PDMA_CH_MCU_ADC0_CH1_RX)
#define UDMA_PDMA_CH_MCU_ADC1_CH0_RX    (CSL_PDMA_CH_MCU_ADC1_CH0_RX)
#define UDMA_PDMA_CH_MCU_ADC1_CH1_RX    (CSL_PDMA_CH_MCU_ADC1_CH1_RX)
/*
 * PDMA MCU McSPI RX Channels
 */
#define UDMA_PDMA_CH_MCU_MCSPI0_CH0_RX  (CSL_PDMA_CH_MCU_MCSPI0_CH0_RX)
#define UDMA_PDMA_CH_MCU_MCSPI0_CH1_RX  (CSL_PDMA_CH_MCU_MCSPI0_CH1_RX)
#define UDMA_PDMA_CH_MCU_MCSPI0_CH2_RX  (CSL_PDMA_CH_MCU_MCSPI0_CH2_RX)
#define UDMA_PDMA_CH_MCU_MCSPI0_CH3_RX  (CSL_PDMA_CH_MCU_MCSPI0_CH3_RX)
#define UDMA_PDMA_CH_MCU_MCSPI1_CH0_RX  (CSL_PDMA_CH_MCU_MCSPI1_CH0_RX)
#define UDMA_PDMA_CH_MCU_MCSPI1_CH1_RX  (CSL_PDMA_CH_MCU_MCSPI1_CH1_RX)
#define UDMA_PDMA_CH_MCU_MCSPI1_CH2_RX  (CSL_PDMA_CH_MCU_MCSPI1_CH2_RX)
#define UDMA_PDMA_CH_MCU_MCSPI1_CH3_RX  (CSL_PDMA_CH_MCU_MCSPI1_CH3_RX)
#define UDMA_PDMA_CH_MCU_MCSPI2_CH0_RX  (CSL_PDMA_CH_MCU_MCSPI2_CH0_RX)
#define UDMA_PDMA_CH_MCU_MCSPI2_CH1_RX  (CSL_PDMA_CH_MCU_MCSPI2_CH1_RX)
#define UDMA_PDMA_CH_MCU_MCSPI2_CH2_RX  (CSL_PDMA_CH_MCU_MCSPI2_CH2_RX)
#define UDMA_PDMA_CH_MCU_MCSPI2_CH3_RX  (CSL_PDMA_CH_MCU_MCSPI2_CH3_RX)
/*
 * PDMA MCU MCAN RX Channels
 */
#define UDMA_PDMA_CH_MCU_MCAN0_CH0_RX   (CSL_PDMA_CH_MCU_MCAN0_CH0_RX)
#define UDMA_PDMA_CH_MCU_MCAN0_CH1_RX   (CSL_PDMA_CH_MCU_MCAN0_CH1_RX)
#define UDMA_PDMA_CH_MCU_MCAN0_CH2_RX   (CSL_PDMA_CH_MCU_MCAN0_CH2_RX)
#define UDMA_PDMA_CH_MCU_MCAN1_CH0_RX   (CSL_PDMA_CH_MCU_MCAN1_CH0_RX)
#define UDMA_PDMA_CH_MCU_MCAN1_CH1_RX   (CSL_PDMA_CH_MCU_MCAN1_CH1_RX)
#define UDMA_PDMA_CH_MCU_MCAN1_CH2_RX   (CSL_PDMA_CH_MCU_MCAN1_CH2_RX)
/*
 * PDMA MCU UART RX Channels
 */
#define UDMA_PDMA_CH_MCU_UART0_RX       (CSL_PDMA_CH_MCU_UART0_CH0_RX)
/** @} */

/** @} */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 *  \brief Returns TRUE if the memory is cache coherent
 *
 *  \return TRUE/FALSE
 */
uint32_t Udma_isCacheCoherent(void);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef UDMA_SOC_H_ */
