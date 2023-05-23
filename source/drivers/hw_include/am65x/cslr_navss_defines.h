/********************************************************************
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

#ifndef CSLR_NAVSS_DEFINES_H_
#define CSLR_NAVSS_DEFINES_H_

/**
 *  \anchor CSL_Navss
 *  \name Configuration parameters for navss
 *
 *  List of configuration parameters for navss_main and navss_mcu
 *
 *  @{
 */

/**
 *  \anchor CSL_NavssUdmap
 *  \name Udmap configuration parameters
 *
 *  List of udmap module configuration parameters for navss_main and navss_mcu
 *
 *  @{
 */
#define CSL_NAVSS_MAIN_UDMAP_NUM_TX_CHANS                                 ((uint32_t) 120U)
#define CSL_NAVSS_MAIN_UDMAP_NUM_RX_CHANS                                 ((uint32_t) 150U)
#define CSL_NAVSS_MAIN_UDMAP_NUM_TX_HC_CHANS                              ((uint32_t) 8U)
#define CSL_NAVSS_MAIN_UDMAP_NUM_RX_HC_CHANS                              ((uint32_t) 8U)
#define CSL_NAVSS_MAIN_UDMAP_NUM_EXT_CHANS                                ((uint32_t) 32U)
#define CSL_NAVSS_MAIN_UDMAP_NUM_SECURE_CHANS                             ((uint32_t) 50U)
#define CSL_NAVSS_MAIN_UDMAP_NUM_RX_FLOWS                                 ((uint32_t) 300U)

#define CSL_NAVSS_MCU_UDMAP_NUM_TX_CHANS                                  ((uint32_t) 48U)
#define CSL_NAVSS_MCU_UDMAP_NUM_RX_CHANS                                  ((uint32_t) 48U)
#define CSL_NAVSS_MCU_UDMAP_NUM_TX_HC_CHANS                               ((uint32_t) 2U)
#define CSL_NAVSS_MCU_UDMAP_NUM_RX_HC_CHANS                               ((uint32_t) 2U)
#define CSL_NAVSS_MCU_UDMAP_NUM_EXT_CHANS                                 ((uint32_t) 0U)
#define CSL_NAVSS_MCU_UDMAP_NUM_SECURE_CHANS                              ((uint32_t) 30U)
#define CSL_NAVSS_MCU_UDMAP_NUM_RX_FLOWS                                  ((uint32_t) 96U)

#define CSL_NAVSS_UDMAP_TX_CHANS_FDEPTH                                   ((uint32_t) 128U)
#define CSL_NAVSS_UDMAP_TX_HC_CHANS_FDEPTH                                ((uint32_t) 1024U)
/* There is no UHC in this SOC. Set same as HC */
#define CSL_NAVSS_UDMAP_TX_UHC_CHANS_FDEPTH                               ((uint32_t) 1024U)
/* @} */

/**
 *  \anchor CSL_NavssProxy
 *  \name Proxy configuration parameters
 *
 *  List of Proxy module configuration parameters for navss_main and navss_mcu
 *
 *  @{
 */
#define CSL_NAVSS_MAIN_PROXY_NUM_PROXIES                                  ((uint32_t) 64U)
#define CSL_NAVSS_MAIN_PROXY_BUFFER_SIZE_BYTES                            ((uint32_t) 512U)
#define CSL_NAVSS_MAIN_PROXY_NUM_TARGETS                                  ((uint32_t) 1U)
#define CSL_NAVSS_MAIN_PROXY_TARGET_NUM_RINGACC0                          ((uint32_t) 0U)
#define CSL_NAVSS_MAIN_PROXY_TARGET_RINGACC0_ADDR                         ((uintptr_t) 0x000038000000U)
#define CSL_NAVSS_MAIN_PROXY_TARGET_RINGACC0_NUM_CHANNELS                 ((uint32_t) 818U)
#define CSL_NAVSS_MAIN_PROXY_TARGET_RINGACC0_NUM_CHANNEL_SIZE_BYTES       ((uint32_t) 4096U)

#define CSL_NAVSS_MCU_PROXY_NUM_PROXIES                                   ((uint32_t) 64U)
#define CSL_NAVSS_MCU_PROXY_BUFFER_SIZE_BYTES                             ((uint32_t) 512U)
#define CSL_NAVSS_MCU_PROXY_NUM_TARGETS                                   ((uint32_t) 1U)
#define CSL_NAVSS_MCU_PROXY_TARGET_NUM_RINGACC0                           ((uint32_t) 0U)
#define CSL_NAVSS_MCU_PROXY_TARGET_RINGACC0_ADDR                          ((uintptr_t) 0x00002b000000U)
#define CSL_NAVSS_MCU_PROXY_TARGET_RINGACC0_NUM_CHANNELS                  ((uint32_t) 286U)
#define CSL_NAVSS_MCU_PROXY_TARGET_RINGACC0_NUM_CHANNEL_SIZE_BYTES        ((uint32_t) 4096U)
/* @} */

/**
 *  \anchor CSL_NavssSecProxy
 *  \name Secure Proxy configuration parameters
 *
 *  List of Secure Proxy module configuration parameters for navss_main and navss_mcu
 *
 *  @{
 */
#define CSL_NAVSS_MAIN_SEC_PROXY_NUM_PROXIES                              ((uint32_t) 160U)
#define CSL_NAVSS_MAIN_SEC_PROXY_MSG_SIZE_BYTES                           ((uint32_t) 64)
#define CSL_NAVSS_MAIN_SEC_PROXY_NUM_TARGETS                              ((uint32_t) 1U)
#define CSL_NAVSS_MAIN_SEC_PROXY_TARGET_RINGACC0_ADDR                     ((uintptr_t) 0x000038300000U)
#define CSL_NAVSS_MAIN_SEC_PROXY_TARGET_RINGACC0_NUM_CHANNELS             ((uint32_t) 50U)
#define CSL_NAVSS_MAIN_SEC_PROXY_TARGET_RINGACC0_NUM_CHANNEL_SIZE_BYTES   ((uint32_t) 4096U)

#define CSL_NAVSS_MCU_SEC_PROXY_NUM_PROXIES                               ((uint32_t) 90U)
#define CSL_NAVSS_MCU_SEC_PROXY_MSG_SIZE_BYTES                            ((uint32_t) 64)
#define CSL_NAVSS_MCU_SEC_PROXY_NUM_TARGETS                               ((uint32_t) 1U)
#define CSL_NAVSS_MCU_SEC_PROXY_TARGET_RINGACC0_ADDR                      ((uintptr_t) 0x00002b100000U)
#define CSL_NAVSS_MCU_SEC_PROXY_TARGET_RINGACC0_NUM_CHANNELS              ((uint32_t) 30U)
#define CSL_NAVSS_MCU_SEC_PROXY_TARGET_RINGACC0_NUM_CHANNEL_SIZE_BYTES    ((uint32_t) 4096U)
/* @} */

/**
 *  \anchor CSL_NavssRingacc
 *  \name Ring Accelerator configuration parameters
 *
 *  List of Ring Accelerator module configuration parameters for navss_main and navss_mcu
 *
 *  @{
 */
#define CSL_NAVSS_MAIN_RINGACC_RING_CNT                                   ((uint32_t) 818U)
#define CSL_NAVSS_MAIN_RINGACC_NUM_MONITORS                               ((uint32_t) 32U)
#define CSL_NAVSS_MAIN_RINGACC_IS_TRACE_SUPPORTED                         ((uint32_t) 1U)

#define CSL_NAVSS_MCU_RINGACC_RING_CNT                                    ((uint32_t) 286U)
#define CSL_NAVSS_MCU_RINGACC_NUM_MONITORS                                ((uint32_t) 32U)
#define CSL_NAVSS_MCU_RINGACC_IS_TRACE_SUPPORTED                          ((uint32_t) 1U)
/* @} */

/**
 *  \anchor CSL_NavssGlobalEventMap
 *  \name NAVSS Global Event map
 *
 *  List of global event offsets
 *
 *  @{
 */
#define CSL_NAVSS_GEM_MAIN_UDMA_INTA0_SEVI_OFFSET                         ((uint32_t) 0x0000U)
#define CSL_NAVSS_GEM_MCU_UDMA_INTA0_SEVI_OFFSET                          ((uint32_t) 0x4000U)
#define CSL_NAVSS_GEM_DMSC_INTA_SEVI_OFFSET                               ((uint32_t) 0x4800U)
#define CSL_NAVSS_GEM_MODSS_INTA0_SEVI_OFFSET                             ((uint32_t) 0x5000U)
#define CSL_NAVSS_GEM_MODSS_INTA1_SEVI_OFFSET                             ((uint32_t) 0x5800U)
#define CSL_NAVSS_GEM_MAIN_UDMA_INTA0_MEVI_OFFSET                         ((uint32_t) 0x8000U)
#define CSL_NAVSS_GEM_MCU_UDMA_INTA0_MEVI_OFFSET                          ((uint32_t) 0x8800U)
#define CSL_NAVSS_GEM_MAIN_UDMA_INTA0_GEVI_OFFSET                         ((uint32_t) 0x9000U)
#define CSL_NAVSS_GEM_MCU_UDMA_INTA0_GEVI_OFFSET                          ((uint32_t) 0x9C00U)
#define CSL_NAVSS_GEM_PDMA_MCU0_LEVI_OFFSET                               ((uint32_t) 0xA000U)
#define CSL_NAVSS_GEM_PDMA_MCU1_LEVI_OFFSET                               ((uint32_t) 0xA080U)
#define CSL_NAVSS_GEM_PDMA_MAIN0_LEVI_OFFSET                              ((uint32_t) 0xA400U)
#define CSL_NAVSS_GEM_PDMA_MAIN1_LEVI_OFFSET                              ((uint32_t) 0xA480U)
#define CSL_NAVSS_GEM_PDMA_DEBUG_LEVI_OFFSET                              ((uint32_t) 0xA500U)
#define CSL_NAVSS_GEM_MAIN_MCRC_LEVI_OFFSET                               ((uint32_t) 0xA800U)
#define CSL_NAVSS_GEM_MCU_MCRC_LEVI_OFFSET                                ((uint32_t) 0xA880U)
#define CSL_NAVSS_GEM_MAIN_UDMA_TRIGGER_OFFSET                            ((uint32_t) 0xC000U)
#define CSL_NAVSS_GEM_MCU_UDMA_TRIGGER_OFFSET                             ((uint32_t) 0xDC00U)
#define CSL_NAVSS_GEM_MSMC_DRU_OFFSET                                     ((uint32_t) 0xF000U)
/* @} */

/**
 *  \anchor CSL_NavssUtc
 *  \name UTC configuration parameters
 *
 *  List of UTC module configuration parameters
 *
 *  @{
 */
#define CSL_NAVSS_UTC_CNT                                                 (0x0001U)
#define CSL_NAVSS_UTC_MSMC_DRU_QUEUE_CNT                                  ((uint32_t) 0x0005U)
/* @} */

/* @} */

#endif
