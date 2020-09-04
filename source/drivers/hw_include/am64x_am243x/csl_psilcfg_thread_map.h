/* =============================================================================
 *   Copyright (c) 2019 Texas Instruments Incorporated
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

#ifndef CSL_PSILCFG_THREAD_MAP_H_
#define CSL_PSILCFG_THREAD_MAP_H_

#ifdef __cplusplus
extern "C"
{
#endif


/*-----------------------------------------------------------------------------
 *  The following nomenclature is used in this file:
 *
 *  o #define CSL_PSILCFG_DMSS_SAUL0_PSILS_THREAD_OFFSET      (0x4000U)
 *                                             -
 *      "S" means a thread source (from the perspective of navss)
 *
 *  o #define CSL_PSILCFG_DMSS_SAUL0_PSILD_THREAD_CNT         (2U)
 *                                             -
 *      "D" means a thread destination (from the perspective of navss)
 *
 *  o #define CSL_PSILCFG_DMSS_SAUL0_PSILS_THREAD_CNT         (4U)
 *                                               ----------
 *      "THREAD_CNT" is the thread count for the specified thread
 *      source or destination
 *
 *  o #define CSL_PSILCFG_DMSS_SAUL0_PSILD_THREAD_OFFSET      (CSL_PSILCFG_DEST_THREAD_OFFSET | CSL_PSILCFG_DMSS_SAUL0_PSILS_THREAD_OFFSET)
 *                                               -------------
 *      "THREAD_OFFSET" is the starting thread # for the specified thread
 *      source or destination of the PSI-L endpoint
 *---------------------------------------------------------------------------*/

/** \brief Destination thread offset */
#define CSL_PSILCFG_DEST_THREAD_OFFSET      (0x8000U)

/**
 *  \anchor CSL_PsilcfgThreadMap
 *  \name PSIL thread map for AM64X dmss
 *
 *  Map of all PSIL threads
 *
 *  @{
 */
/*=============================================================================
 * navss_main thread map
 *===========================================================================*/
/*-----------------------------------------------------------------------------
 * PSILCFG0_CFGSTRM PSIL Map
 *---------------------------------------------------------------------------*/
#define CSL_PSILCFG_DMSS_PSILCFG0_CFGSTRM_PSILS_THREAD_OFFSET             (0x0000U)
#define CSL_PSILCFG_DMSS_PSILCFG0_CFGSTRM_PSILS_THREAD_CNT                (1U)
#define CSL_PSILCFG_DMSS_PSILCFG0_CFGSTRM_PSILD_THREAD_OFFSET             (CSL_PSILCFG_DEST_THREAD_OFFSET | CSL_PSILCFG_DMSS_PSILCFG0_CFGSTRM_PSILS_THREAD_OFFSET)
#define CSL_PSILCFG_DMSS_PSILCFG0_CFGSTRM_PSILD_THREAD_CNT                (0U)
/*-----------------------------------------------------------------------------
 * PKTDMA_CFGSTRM PSIL Map
 *---------------------------------------------------------------------------*/
#define CSL_PSILCFG_DMSS_PKTDMA_CFGSTRM_PSILS_THREAD_OFFSET               (0x0020U)
#define CSL_PSILCFG_DMSS_PKTDMA_CFGSTRM_PSILS_THREAD_CNT                  (1U)
#define CSL_PSILCFG_DMSS_PKTDMA_CFGSTRM_PSILD_THREAD_OFFSET               (CSL_PSILCFG_DEST_THREAD_OFFSET | CSL_PSILCFG_DMSS_PKTDMA_CFGSTRM_PSILS_THREAD_OFFSET)
#define CSL_PSILCFG_DMSS_PKTDMA_CFGSTRM_PSILD_THREAD_CNT                  (0U)
/*-----------------------------------------------------------------------------
 * BCDMA_CFGSTRM PSIL Map
 *---------------------------------------------------------------------------*/
#define CSL_PSILCFG_DMSS_BCDMA_CFGSTRM_PSILS_THREAD_OFFSET                (0x0021U)
#define CSL_PSILCFG_DMSS_BCDMA_CFGSTRM_PSILS_THREAD_CNT                   (1U)
#define CSL_PSILCFG_DMSS_BCDMA_CFGSTRM_PSILD_THREAD_OFFSET                (CSL_PSILCFG_DEST_THREAD_OFFSET | CSL_PSILCFG_DMSS_BCDMA_CFGSTRM_PSILS_THREAD_OFFSET)
#define CSL_PSILCFG_DMSS_BCDMA_CFGSTRM_PSILD_THREAD_CNT                   (0U)
/*-----------------------------------------------------------------------------
 * PKTDMA_STRM PSIL Map
 *---------------------------------------------------------------------------*/
#define CSL_PSILCFG_DMSS_PKTDMA_STRM_PSILS_THREAD_OFFSET                  (0x1000U)
#define CSL_PSILCFG_DMSS_PKTDMA_STRM_PSILS_THREAD_CNT                     (35U)
#define CSL_PSILCFG_DMSS_PKTDMA_STRM_PSILD_THREAD_OFFSET                  (CSL_PSILCFG_DEST_THREAD_OFFSET | CSL_PSILCFG_DMSS_PKTDMA_STRM_PSILS_THREAD_OFFSET)
#define CSL_PSILCFG_DMSS_PKTDMA_STRM_PSILD_THREAD_CNT                     (29U)
/*-----------------------------------------------------------------------------
 * BCDMA_STRM PSIL Map
 *---------------------------------------------------------------------------*/
#define CSL_PSILCFG_DMSS_BCDMA_STRM_PSILS_THREAD_OFFSET                   (0x2000U)
#define CSL_PSILCFG_DMSS_BCDMA_STRM_PSILS_THREAD_CNT                      (20U)
#define CSL_PSILCFG_DMSS_BCDMA_STRM_PSILD_THREAD_OFFSET                   (CSL_PSILCFG_DEST_THREAD_OFFSET | CSL_PSILCFG_DMSS_BCDMA_STRM_PSILS_THREAD_OFFSET)
#define CSL_PSILCFG_DMSS_BCDMA_STRM_PSILD_THREAD_CNT                      (20U)
/*-----------------------------------------------------------------------------
 * SAUL0 PSIL Map
 *---------------------------------------------------------------------------*/
#define CSL_PSILCFG_DMSS_SAUL0_PSILS_THREAD_OFFSET                        (0x4000U)
#define CSL_PSILCFG_DMSS_SAUL0_PSILS_THREAD_CNT                           (4U)
#define CSL_PSILCFG_DMSS_SAUL0_PSILD_THREAD_OFFSET                        (CSL_PSILCFG_DEST_THREAD_OFFSET | CSL_PSILCFG_DMSS_SAUL0_PSILS_THREAD_OFFSET)
#define CSL_PSILCFG_DMSS_SAUL0_PSILD_THREAD_CNT                           (2U)
/*-----------------------------------------------------------------------------
 * ICSS_G0 PSIL Map
 *---------------------------------------------------------------------------*/
#define CSL_PSILCFG_DMSS_ICSS_G0_PSILS_THREAD_OFFSET                      (0x4100U)
#define CSL_PSILCFG_DMSS_ICSS_G0_PSILS_THREAD_CNT                         (5U)
#define CSL_PSILCFG_DMSS_ICSS_G0_PSILD_THREAD_OFFSET                      (CSL_PSILCFG_DEST_THREAD_OFFSET | CSL_PSILCFG_DMSS_ICSS_G0_PSILS_THREAD_OFFSET)
#define CSL_PSILCFG_DMSS_ICSS_G0_PSILD_THREAD_CNT                         (9U)
/*-----------------------------------------------------------------------------
 * ICSS_G1 PSIL Map
 *---------------------------------------------------------------------------*/
#define CSL_PSILCFG_DMSS_ICSS_G1_PSILS_THREAD_OFFSET                      (0x4200U)
#define CSL_PSILCFG_DMSS_ICSS_G1_PSILS_THREAD_CNT                         (5U)
#define CSL_PSILCFG_DMSS_ICSS_G1_PSILD_THREAD_OFFSET                      (CSL_PSILCFG_DEST_THREAD_OFFSET | CSL_PSILCFG_DMSS_ICSS_G1_PSILS_THREAD_OFFSET)
#define CSL_PSILCFG_DMSS_ICSS_G1_PSILD_THREAD_CNT                         (9U)
/*-----------------------------------------------------------------------------
 * PDMA_MAIN0 PSIL Map
 *---------------------------------------------------------------------------*/
#define CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILS_THREAD_OFFSET                   (0x4300U)
#define CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILS_THREAD_CNT                      (18U)
#define CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILD_THREAD_OFFSET                   (CSL_PSILCFG_DEST_THREAD_OFFSET | CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILS_THREAD_OFFSET)
#define CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILD_THREAD_CNT                      (18U)
/*-----------------------------------------------------------------------------
 * PDMA_MAIN1 PSIL Map
 *---------------------------------------------------------------------------*/
#define CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILS_THREAD_OFFSET                   (0x4400U)
#define CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILS_THREAD_CNT                      (17U)
#define CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILD_THREAD_OFFSET                   (CSL_PSILCFG_DEST_THREAD_OFFSET | CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILS_THREAD_OFFSET)
#define CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILD_THREAD_CNT                      (15U)
/*-----------------------------------------------------------------------------
 * CPSW2 PSIL Map
 *---------------------------------------------------------------------------*/
#define CSL_PSILCFG_DMSS_CPSW2_PSILS_THREAD_OFFSET                        (0x4500U)
#define CSL_PSILCFG_DMSS_CPSW2_PSILS_THREAD_CNT                           (1U)
#define CSL_PSILCFG_DMSS_CPSW2_PSILD_THREAD_OFFSET                        (CSL_PSILCFG_DEST_THREAD_OFFSET | CSL_PSILCFG_DMSS_CPSW2_PSILS_THREAD_OFFSET)
#define CSL_PSILCFG_DMSS_CPSW2_PSILD_THREAD_CNT                           (8U)

/* @} */

/**
 *  \anchor CSL_PdmaCh
 *  \name PDMA Channels
 *
 *  List of all PDMA channels across dmss domain
 *
 *  @{
 */

/**
 *  \anchor CSL_PdmaChMain0Tx
 *  \name Main0 TX PDMA Channels
 *
 *  List of all Main0 PDMA TX channels
 *
 *  @{
 */

/*
 * PDMA MAIN0 MCSPI TX Channels
 */
#define CSL_PDMA_CH_MAIN0_MCSPI0_CH0_TX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILD_THREAD_OFFSET + 0U)
#define CSL_PDMA_CH_MAIN0_MCSPI0_CH1_TX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILD_THREAD_OFFSET + 1U)
#define CSL_PDMA_CH_MAIN0_MCSPI0_CH2_TX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILD_THREAD_OFFSET + 2U)
#define CSL_PDMA_CH_MAIN0_MCSPI0_CH3_TX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILD_THREAD_OFFSET + 3U)
#define CSL_PDMA_CH_MAIN0_MCSPI1_CH0_TX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILD_THREAD_OFFSET + 4U)
#define CSL_PDMA_CH_MAIN0_MCSPI1_CH1_TX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILD_THREAD_OFFSET + 5U)
#define CSL_PDMA_CH_MAIN0_MCSPI1_CH2_TX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILD_THREAD_OFFSET + 6U)
#define CSL_PDMA_CH_MAIN0_MCSPI1_CH3_TX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILD_THREAD_OFFSET + 7U)
#define CSL_PDMA_CH_MAIN0_MCSPI2_CH0_TX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILD_THREAD_OFFSET + 8U)
#define CSL_PDMA_CH_MAIN0_MCSPI2_CH1_TX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILD_THREAD_OFFSET + 9U)
#define CSL_PDMA_CH_MAIN0_MCSPI2_CH2_TX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILD_THREAD_OFFSET + 10U)
#define CSL_PDMA_CH_MAIN0_MCSPI2_CH3_TX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILD_THREAD_OFFSET + 11U)
#define CSL_PDMA_CH_MAIN0_MCSPI3_CH0_TX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILD_THREAD_OFFSET + 12U)
#define CSL_PDMA_CH_MAIN0_MCSPI3_CH1_TX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILD_THREAD_OFFSET + 13U)
#define CSL_PDMA_CH_MAIN0_MCSPI3_CH2_TX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILD_THREAD_OFFSET + 14U)
#define CSL_PDMA_CH_MAIN0_MCSPI3_CH3_TX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILD_THREAD_OFFSET + 15U)
/*
 * PDMA MAIN0 UART TX Channels
 */
#define CSL_PDMA_CH_MAIN0_UART0_CH0_TX       (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILD_THREAD_OFFSET + 16U)
#define CSL_PDMA_CH_MAIN0_UART1_CH0_TX       (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILD_THREAD_OFFSET + 17U)

/* @} */

/**
 *  \anchor CSL_PdmaChMain0Rx
 *  \name Main0 RX PDMA Channels
 *
 *  List of all Main0 PDMA RX channels
 *
 *  @{
 */

/*
 * PDMA MAIN0 MCSPI RX Channels
 */
#define CSL_PDMA_CH_MAIN0_MCSPI0_CH0_RX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILS_THREAD_OFFSET + 0U)
#define CSL_PDMA_CH_MAIN0_MCSPI0_CH1_RX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILS_THREAD_OFFSET + 1U)
#define CSL_PDMA_CH_MAIN0_MCSPI0_CH2_RX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILS_THREAD_OFFSET + 2U)
#define CSL_PDMA_CH_MAIN0_MCSPI0_CH3_RX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILS_THREAD_OFFSET + 3U)
#define CSL_PDMA_CH_MAIN0_MCSPI1_CH0_RX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILS_THREAD_OFFSET + 4U)
#define CSL_PDMA_CH_MAIN0_MCSPI1_CH1_RX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILS_THREAD_OFFSET + 5U)
#define CSL_PDMA_CH_MAIN0_MCSPI1_CH2_RX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILS_THREAD_OFFSET + 6U)
#define CSL_PDMA_CH_MAIN0_MCSPI1_CH3_RX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILS_THREAD_OFFSET + 7U)
#define CSL_PDMA_CH_MAIN0_MCSPI2_CH0_RX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILS_THREAD_OFFSET + 8U)
#define CSL_PDMA_CH_MAIN0_MCSPI2_CH1_RX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILS_THREAD_OFFSET + 9U)
#define CSL_PDMA_CH_MAIN0_MCSPI2_CH2_RX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILS_THREAD_OFFSET + 10U)
#define CSL_PDMA_CH_MAIN0_MCSPI2_CH3_RX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILS_THREAD_OFFSET + 11U)
#define CSL_PDMA_CH_MAIN0_MCSPI3_CH0_RX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILS_THREAD_OFFSET + 12U)
#define CSL_PDMA_CH_MAIN0_MCSPI3_CH1_RX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILS_THREAD_OFFSET + 13U)
#define CSL_PDMA_CH_MAIN0_MCSPI3_CH2_RX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILS_THREAD_OFFSET + 14U)
#define CSL_PDMA_CH_MAIN0_MCSPI3_CH3_RX      (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILS_THREAD_OFFSET + 15U)
/*
 * PDMA MAIN0 UART RX Channels
 */
#define CSL_PDMA_CH_MAIN0_UART0_CH0_RX       (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILS_THREAD_OFFSET + 16U)
#define CSL_PDMA_CH_MAIN0_UART1_CH0_RX       (CSL_PSILCFG_DMSS_PDMA_MAIN0_PSILS_THREAD_OFFSET + 17U)

/* @} */

/**
 *  \anchor CSL_PdmaChMain1Tx
 *  \name Main1 TX PDMA Channels
 *
 *  List of all Main1 PDMA TX channels
 *
 *  @{
 */

/*
 * PDMA MAIN1 MCSPI TX Channels
 */
#define CSL_PDMA_CH_MAIN1_MCSPI4_CH0_TX      (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILD_THREAD_OFFSET + 0U)
#define CSL_PDMA_CH_MAIN1_MCSPI4_CH1_TX      (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILD_THREAD_OFFSET + 1U)
#define CSL_PDMA_CH_MAIN1_MCSPI4_CH2_TX      (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILD_THREAD_OFFSET + 2U)
#define CSL_PDMA_CH_MAIN1_MCSPI4_CH3_TX      (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILD_THREAD_OFFSET + 3U)
/*
 * PDMA MAIN1 UART TX Channels
 */
#define CSL_PDMA_CH_MAIN1_UART2_CH0_TX       (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILD_THREAD_OFFSET + 4U)
#define CSL_PDMA_CH_MAIN1_UART3_CH0_TX       (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILD_THREAD_OFFSET + 5U)
#define CSL_PDMA_CH_MAIN1_UART4_CH0_TX       (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILD_THREAD_OFFSET + 6U)
#define CSL_PDMA_CH_MAIN1_UART5_CH0_TX       (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILD_THREAD_OFFSET + 7U)
#define CSL_PDMA_CH_MAIN1_UART6_CH0_TX       (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILD_THREAD_OFFSET + 8U)
/*
 * PDMA MAIN1 MCAN TX Channels
 */
#define CSL_PDMA_CH_MAIN1_MCAN0_CH0_TX       (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILD_THREAD_OFFSET + 9U)
#define CSL_PDMA_CH_MAIN1_MCAN0_CH1_TX       (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILD_THREAD_OFFSET + 10U)
#define CSL_PDMA_CH_MAIN1_MCAN0_CH2_TX       (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILD_THREAD_OFFSET + 11U)
#define CSL_PDMA_CH_MAIN1_MCAN1_CH0_TX       (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILD_THREAD_OFFSET + 12U)
#define CSL_PDMA_CH_MAIN1_MCAN1_CH1_TX       (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILD_THREAD_OFFSET + 13U)
#define CSL_PDMA_CH_MAIN1_MCAN1_CH2_TX       (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILD_THREAD_OFFSET + 14U)

/* @} */

/**
 *  \anchor CSL_PdmaChMain1Rx
 *  \name Main1 RX PDMA Channels
 *
 *  List of all Main1 PDMA RX channels
 *
 *  @{
 */

/*
 * PDMA MAIN1 MCSPI RX Channels
 */
#define CSL_PDMA_CH_MAIN1_MCSPI4_CH0_RX      (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILS_THREAD_OFFSET + 0U)
#define CSL_PDMA_CH_MAIN1_MCSPI4_CH1_RX      (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILS_THREAD_OFFSET + 1U)
#define CSL_PDMA_CH_MAIN1_MCSPI4_CH2_RX      (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILS_THREAD_OFFSET + 2U)
#define CSL_PDMA_CH_MAIN1_MCSPI4_CH3_RX      (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILS_THREAD_OFFSET + 3U)
/*
 * PDMA MAIN1 UART RX Channels
 */
#define CSL_PDMA_CH_MAIN1_UART2_CH0_RX       (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILS_THREAD_OFFSET + 4U)
#define CSL_PDMA_CH_MAIN1_UART3_CH0_RX       (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILS_THREAD_OFFSET + 5U)
#define CSL_PDMA_CH_MAIN1_UART4_CH0_RX       (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILS_THREAD_OFFSET + 6U)
#define CSL_PDMA_CH_MAIN1_UART5_CH0_RX       (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILS_THREAD_OFFSET + 7U)
#define CSL_PDMA_CH_MAIN1_UART6_CH0_RX       (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILS_THREAD_OFFSET + 8U)
/*
 * PDMA MAIN1 MCAN RX Channels
 */
#define CSL_PDMA_CH_MAIN1_MCAN0_CH0_RX       (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILS_THREAD_OFFSET + 9U)
#define CSL_PDMA_CH_MAIN1_MCAN0_CH1_RX       (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILS_THREAD_OFFSET + 10U)
#define CSL_PDMA_CH_MAIN1_MCAN0_CH2_RX       (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILS_THREAD_OFFSET + 11U)
#define CSL_PDMA_CH_MAIN1_MCAN1_CH0_RX       (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILS_THREAD_OFFSET + 12U)
#define CSL_PDMA_CH_MAIN1_MCAN1_CH1_RX       (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILS_THREAD_OFFSET + 13U)
#define CSL_PDMA_CH_MAIN1_MCAN1_CH2_RX       (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILS_THREAD_OFFSET + 14U)
/*
 * PDMA MAIN1 ADC RX Channels
 */
#define CSL_PDMA_CH_MAIN1_ADC0_CH0_RX        (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILS_THREAD_OFFSET + 15U)
#define CSL_PDMA_CH_MAIN1_ADC0_CH1_RX        (CSL_PSILCFG_DMSS_PDMA_MAIN1_PSILS_THREAD_OFFSET + 16U)

/* @} */

/* @} */


#ifdef __cplusplus
}
#endif

#endif
