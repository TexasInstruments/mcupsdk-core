/********************************************************************
 * Copyright (C) 2013-2016 Texas Instruments Incorporated.
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
#ifndef CSLR_CPDMA_H_
#define CSLR_CPDMA_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>


/**************************************************************************
* Register Overlay Structure for interrupt
**************************************************************************/
typedef struct {
    volatile Uint32 TX_INTSTAT_RAW;
    volatile Uint32 TX_INTSTAT_MASKED;
    volatile Uint32 TX_INTMASK_SET;
    volatile Uint32 TX_INTMASK_CLEAR;
    volatile Uint32 DMA_IN_VECTOR;
    volatile Uint32 DMA_EOI_VECTOR;
    volatile Uint8  RSVD0[8];
    volatile Uint32 RX_INTSTAT_RAW;
    volatile Uint32 RX_INTSTAT_MASKED;
    volatile Uint32 RX_INTMASK_SET;
    volatile Uint32 RX_INTMASK_CLEAR;
    volatile Uint32 DMA_INTSTAT_RAW;
    volatile Uint32 DMA_INTSTAT_MASKED;
    volatile Uint32 DMA_INTMASK_SET;
    volatile Uint32 DMA_INTMASK_CLEAR;
    volatile Uint32 RX_PENDTHRESH[8];
    volatile Uint32 RX_FREEBUFFER[8];
} CSL_CpdmaInterruptRegs;


/**************************************************************************
* Register Overlay Structure for sram
**************************************************************************/
typedef struct {
    volatile Uint32 TX_HDP[8];
    volatile Uint32 RX_HDP[8];
    volatile Uint32 TX_CP[8];
    volatile Uint32 RX_CP[8];
} CSL_CpdmaSramRegs;


/**************************************************************************
* Register Overlay Structure
**************************************************************************/
typedef struct {
    volatile Uint32 TX_IDVER;
    volatile Uint32 TX_CONTROL;
    volatile Uint32 TX_TEARDOWN;
    volatile Uint8  RSVD0[4];
    volatile Uint32 RX_IDVER;
    volatile Uint32 RX_CONTROL;
    volatile Uint32 RX_TEARDOWN;
    volatile Uint32 SOFT_RESET;
    volatile Uint32 DMACONTROL;
    volatile Uint32 DMASTATUS;
    volatile Uint32 RX_BUFFER_OFFSET;
    volatile Uint32 EMCONTROL;
    volatile Uint32 TX_PRI_RATE[8];
    volatile Uint8  RSVD1[48];
    CSL_CpdmaInterruptRegs	INTERRUPT;
    volatile Uint8  RSVD2[256];
    CSL_CpdmaSramRegs	SRAM;
} CSL_CpdmaRegs;




/**************************************************************************
* Register Macros
**************************************************************************/

/* TX_IDVER */
#define CSL_CPDMA_TX_IDVER                                      ((uint32_t)(0x0U))

/* TX_CONTROL */
#define CSL_CPDMA_TX_CONTROL                                    ((uint32_t)(0x4U))

/* TX_TEARDOWN */
#define CSL_CPDMA_TX_TEARDOWN                                   ((uint32_t)(0x8U))

/* RX_IDVER */
#define CSL_CPDMA_RX_IDVER                                      ((uint32_t)(0x10U))

/* RX_CONTROL */
#define CSL_CPDMA_RX_CONTROL                                    ((uint32_t)(0x14U))

/* RX_TEARDOWN */
#define CSL_CPDMA_RX_TEARDOWN                                   ((uint32_t)(0x18U))

/* SOFT_RESET */
#define CSL_CPDMA_SOFT_RESET                                    ((uint32_t)(0x1CU))

/* DMACONTROL */
#define CSL_CPDMA_DMACONTROL                                    ((uint32_t)(0x20U))

/* DMASTATUS */
#define CSL_CPDMA_DMASTATUS                                     ((uint32_t)(0x24U))

/* RX_BUFFER_OFFSET */
#define CSL_CPDMA_RX_BUFFER_OFFSET                              ((uint32_t)(0x28U))

/* EMCONTROL */
#define CSL_CPDMA_EMCONTROL                                     ((uint32_t)(0x2CU))

/* TX_PRI_RATE */
#define CSL_CPDMA_TX_PRI_RATE(i)                                ((uint32_t)0x30U + ((i) * ((uint32_t)(0x4U))))

/* TX_INTSTAT_RAW */
#define CSL_CPDMA_TX_INTSTAT_RAW                                ((uint32_t)(0x80U))

/* TX_INTSTAT_MASKED */
#define CSL_CPDMA_TX_INTSTAT_MASKED                             ((uint32_t)(0x84U))

/* TX_INTMASK_SET */
#define CSL_CPDMA_TX_INTMASK_SET                                ((uint32_t)(0x88U))

/* TX_INTMASK_CLEAR */
#define CSL_CPDMA_TX_INTMASK_CLEAR                              ((uint32_t)(0x8CU))

/* DMA_IN_VECTOR */
#define CSL_CPDMA_DMA_IN_VECTOR                                 ((uint32_t)(0x90U))

/* DMA_EOI_VECTOR */
#define CSL_CPDMA_DMA_EOI_VECTOR                                ((uint32_t)(0x94U))

/* RX_INTSTAT_RAW */
#define CSL_CPDMA_RX_INTSTAT_RAW                                ((uint32_t)(0xA0U))

/* RX_INTSTAT_MASKED */
#define CSL_CPDMA_RX_INTSTAT_MASKED                             ((uint32_t)(0xA4U))

/* RX_INTMASK_SET */
#define CSL_CPDMA_RX_INTMASK_SET                                ((uint32_t)(0xA8U))

/* RX_INTMASK_CLEAR */
#define CSL_CPDMA_RX_INTMASK_CLEAR                              ((uint32_t)(0xACU))

/* DMA_INTSTAT_RAW */
#define CSL_CPDMA_DMA_INTSTAT_RAW                               ((uint32_t)(0xB0U))

/* DMA_INTSTAT_MASKED */
#define CSL_CPDMA_DMA_INTSTAT_MASKED                            ((uint32_t)(0xB4U))

/* DMA_INTMASK_SET */
#define CSL_CPDMA_DMA_INTMASK_SET                               ((uint32_t)(0xB8U))

/* DMA_INTMASK_CLEAR */
#define CSL_CPDMA_DMA_INTMASK_CLEAR                             ((uint32_t)(0xBCU))

/* RX_PENDTHRESH */
#define CSL_CPDMA_RX_PENDTHRESH(i)                              ((uint32_t)0xC0U + ((i) * ((uint32_t)(0x4U))))

/* RX_FREEBUFFER */
#define CSL_CPDMA_RX_FREEBUFFER(i)                              ((uint32_t)0xE0U + ((i) * ((uint32_t)(0x4U))))

/* TX_HDP */
#define CSL_CPDMA_TX_HDP(i)                                     ((uint32_t)0x200U + ((i) * ((uint32_t)(0x4U))))

/* RX_HDP */
#define CSL_CPDMA_RX_HDP(i)                                     ((uint32_t)0x220U + ((i) * ((uint32_t)(0x4U))))

/* TX_CP */
#define CSL_CPDMA_TX_CP(i)                                      ((uint32_t)0x240U + ((i) * ((uint32_t)(0x4U))))

/* RX_CP */
#define CSL_CPDMA_RX_CP(i)                                      ((uint32_t)0x260U + ((i) * ((uint32_t)(0x4U))))


/**************************************************************************
* Field Definition Macros
**************************************************************************/

/* TX_IDVER */

#define CSL_CPDMA_TX_IDVER_TX_MINOR_VER_MASK                    ((uint32_t)(0x000000FFU))
#define CSL_CPDMA_TX_IDVER_TX_MINOR_VER_SHIFT                   ((uint32_t)(0U))
#define CSL_CPDMA_TX_IDVER_TX_MINOR_VER_RESETVAL                ((uint32_t)(0x00000000U))
#define CSL_CPDMA_TX_IDVER_TX_MINOR_VER_MAX                     ((uint32_t)(0x000000ffU))

#define CSL_CPDMA_TX_IDVER_TX_MAJOR_VER_MASK                    ((uint32_t)(0x0000FF00U))
#define CSL_CPDMA_TX_IDVER_TX_MAJOR_VER_SHIFT                   ((uint32_t)(8U))
#define CSL_CPDMA_TX_IDVER_TX_MAJOR_VER_RESETVAL                ((uint32_t)(0x00000000U))
#define CSL_CPDMA_TX_IDVER_TX_MAJOR_VER_MAX                     ((uint32_t)(0x000000ffU))

#define CSL_CPDMA_TX_IDVER_TX_IDENT_MASK                        ((uint32_t)(0xFFFF0000U))
#define CSL_CPDMA_TX_IDVER_TX_IDENT_SHIFT                       ((uint32_t)(16U))
#define CSL_CPDMA_TX_IDVER_TX_IDENT_RESETVAL                    ((uint32_t)(0x00000000U))
#define CSL_CPDMA_TX_IDVER_TX_IDENT_MAX                         ((uint32_t)(0x0000ffffU))

#define CSL_CPDMA_TX_IDVER_RESETVAL                             ((uint32_t)(0x00000000U))

/* TX_CONTROL */

#define CSL_CPDMA_TX_CONTROL_TX_EN_MASK                         ((uint32_t)(0x00000001U))
#define CSL_CPDMA_TX_CONTROL_TX_EN_SHIFT                        ((uint32_t)(0U))
#define CSL_CPDMA_TX_CONTROL_TX_EN_RESETVAL                     ((uint32_t)(0x00000000U))
#define CSL_CPDMA_TX_CONTROL_TX_EN_MAX                          ((uint32_t)(0x00000001U))

#define CSL_CPDMA_TX_CONTROL_RESETVAL                           ((uint32_t)(0x00000000U))

/* TX_TEARDOWN */

#define CSL_CPDMA_TX_TEARDOWN_TX_TDN_CH_MASK                    ((uint32_t)(0x00000007U))
#define CSL_CPDMA_TX_TEARDOWN_TX_TDN_CH_SHIFT                   ((uint32_t)(0U))
#define CSL_CPDMA_TX_TEARDOWN_TX_TDN_CH_RESETVAL                ((uint32_t)(0x00000000U))
#define CSL_CPDMA_TX_TEARDOWN_TX_TDN_CH_MAX                     ((uint32_t)(0x00000007U))

#define CSL_CPDMA_TX_TEARDOWN_TX_TDN_RDY_MASK                   ((uint32_t)(0x80000000U))
#define CSL_CPDMA_TX_TEARDOWN_TX_TDN_RDY_SHIFT                  ((uint32_t)(31U))
#define CSL_CPDMA_TX_TEARDOWN_TX_TDN_RDY_RESETVAL               ((uint32_t)(0x00000000U))
#define CSL_CPDMA_TX_TEARDOWN_TX_TDN_RDY_MAX                    ((uint32_t)(0x00000001U))

#define CSL_CPDMA_TX_TEARDOWN_RESETVAL                          ((uint32_t)(0x00000000U))

/* RX_IDVER */

#define CSL_CPDMA_RX_IDVER_RX_MINOR_VER_MASK                    ((uint32_t)(0x000000FFU))
#define CSL_CPDMA_RX_IDVER_RX_MINOR_VER_SHIFT                   ((uint32_t)(0U))
#define CSL_CPDMA_RX_IDVER_RX_MINOR_VER_RESETVAL                ((uint32_t)(0x00000000U))
#define CSL_CPDMA_RX_IDVER_RX_MINOR_VER_MAX                     ((uint32_t)(0x000000ffU))

#define CSL_CPDMA_RX_IDVER_RX_MAJOR_VER_MASK                    ((uint32_t)(0x0000FF00U))
#define CSL_CPDMA_RX_IDVER_RX_MAJOR_VER_SHIFT                   ((uint32_t)(8U))
#define CSL_CPDMA_RX_IDVER_RX_MAJOR_VER_RESETVAL                ((uint32_t)(0x00000000U))
#define CSL_CPDMA_RX_IDVER_RX_MAJOR_VER_MAX                     ((uint32_t)(0x000000ffU))

#define CSL_CPDMA_RX_IDVER_RX_IDENT_MASK                        ((uint32_t)(0xFFFF0000U))
#define CSL_CPDMA_RX_IDVER_RX_IDENT_SHIFT                       ((uint32_t)(16U))
#define CSL_CPDMA_RX_IDVER_RX_IDENT_RESETVAL                    ((uint32_t)(0x00000000U))
#define CSL_CPDMA_RX_IDVER_RX_IDENT_MAX                         ((uint32_t)(0x0000ffffU))

#define CSL_CPDMA_RX_IDVER_RESETVAL                             ((uint32_t)(0x00000000U))

/* RX_CONTROL */

#define CSL_CPDMA_RX_CONTROL_RX_EN_MASK                         ((uint32_t)(0x00000001U))
#define CSL_CPDMA_RX_CONTROL_RX_EN_SHIFT                        ((uint32_t)(0U))
#define CSL_CPDMA_RX_CONTROL_RX_EN_RESETVAL                     ((uint32_t)(0x00000000U))
#define CSL_CPDMA_RX_CONTROL_RX_EN_MAX                          ((uint32_t)(0x00000001U))

#define CSL_CPDMA_RX_CONTROL_RESETVAL                           ((uint32_t)(0x00000000U))

/* RX_TEARDOWN */

#define CSL_CPDMA_RX_TEARDOWN_RX_TDN_CH_MASK                    ((uint32_t)(0x00000007U))
#define CSL_CPDMA_RX_TEARDOWN_RX_TDN_CH_SHIFT                   ((uint32_t)(0U))
#define CSL_CPDMA_RX_TEARDOWN_RX_TDN_CH_RESETVAL                ((uint32_t)(0x00000000U))
#define CSL_CPDMA_RX_TEARDOWN_RX_TDN_CH_MAX                     ((uint32_t)(0x00000007U))

#define CSL_CPDMA_RX_TEARDOWN_RX_TDN_RDY_MASK                   ((uint32_t)(0x80000000U))
#define CSL_CPDMA_RX_TEARDOWN_RX_TDN_RDY_SHIFT                  ((uint32_t)(31U))
#define CSL_CPDMA_RX_TEARDOWN_RX_TDN_RDY_RESETVAL               ((uint32_t)(0x00000000U))
#define CSL_CPDMA_RX_TEARDOWN_RX_TDN_RDY_MAX                    ((uint32_t)(0x00000001U))

#define CSL_CPDMA_RX_TEARDOWN_RESETVAL                          ((uint32_t)(0x00000000U))

/* SOFT_RESET */

#define CSL_CPDMA_SOFT_RESET_SOFT_RESET_MASK                    ((uint32_t)(0x00000001U))
#define CSL_CPDMA_SOFT_RESET_SOFT_RESET_SHIFT                   ((uint32_t)(0U))
#define CSL_CPDMA_SOFT_RESET_SOFT_RESET_RESETVAL                ((uint32_t)(0x00000000U))
#define CSL_CPDMA_SOFT_RESET_SOFT_RESET_MAX                     ((uint32_t)(0x00000001U))

#define CSL_CPDMA_SOFT_RESET_RESETVAL                           ((uint32_t)(0x00000000U))

/* DMACONTROL */

#define CSL_CPDMA_DMACONTROL_TX_PTYPE_MASK                      ((uint32_t)(0x00000001U))
#define CSL_CPDMA_DMACONTROL_TX_PTYPE_SHIFT                     ((uint32_t)(0U))
#define CSL_CPDMA_DMACONTROL_TX_PTYPE_RESETVAL                  ((uint32_t)(0x00000000U))
#define CSL_CPDMA_DMACONTROL_TX_PTYPE_MAX                       ((uint32_t)(0x00000001U))

#define CSL_CPDMA_DMACONTROL_RX_OWNERSHIP_MASK                  ((uint32_t)(0x00000002U))
#define CSL_CPDMA_DMACONTROL_RX_OWNERSHIP_SHIFT                 ((uint32_t)(1U))
#define CSL_CPDMA_DMACONTROL_RX_OWNERSHIP_RESETVAL              ((uint32_t)(0x00000000U))
#define CSL_CPDMA_DMACONTROL_RX_OWNERSHIP_MAX                   ((uint32_t)(0x00000001U))

#define CSL_CPDMA_DMACONTROL_RX_OFFLEN_BLOCK_MASK               ((uint32_t)(0x00000004U))
#define CSL_CPDMA_DMACONTROL_RX_OFFLEN_BLOCK_SHIFT              ((uint32_t)(2U))
#define CSL_CPDMA_DMACONTROL_RX_OFFLEN_BLOCK_RESETVAL           ((uint32_t)(0x00000000U))
#define CSL_CPDMA_DMACONTROL_RX_OFFLEN_BLOCK_MAX                ((uint32_t)(0x00000001U))

#define CSL_CPDMA_DMACONTROL_CMD_IDLE_MASK                      ((uint32_t)(0x00000008U))
#define CSL_CPDMA_DMACONTROL_CMD_IDLE_SHIFT                     ((uint32_t)(3U))
#define CSL_CPDMA_DMACONTROL_CMD_IDLE_RESETVAL                  ((uint32_t)(0x00000000U))
#define CSL_CPDMA_DMACONTROL_CMD_IDLE_MAX                       ((uint32_t)(0x00000001U))

#define CSL_CPDMA_DMACONTROL_RX_CEF_MASK                        ((uint32_t)(0x00000010U))
#define CSL_CPDMA_DMACONTROL_RX_CEF_SHIFT                       ((uint32_t)(4U))
#define CSL_CPDMA_DMACONTROL_RX_CEF_RESETVAL                    ((uint32_t)(0x00000000U))
#define CSL_CPDMA_DMACONTROL_RX_CEF_MAX                         ((uint32_t)(0x00000001U))

#define CSL_CPDMA_DMACONTROL_RX_VLAN_ENCAP_MASK                 ((uint32_t)(0x00000020U))
#define CSL_CPDMA_DMACONTROL_RX_VLAN_ENCAP_SHIFT                ((uint32_t)(5U))
#define CSL_CPDMA_DMACONTROL_RX_VLAN_ENCAP_RESETVAL             ((uint32_t)(0x00000000U))
#define CSL_CPDMA_DMACONTROL_RX_VLAN_ENCAP_MAX                  ((uint32_t)(0x00000001U))

#define CSL_CPDMA_DMACONTROL_RX_TS_ENCAP_MASK                   ((uint32_t)(0x00000040U))
#define CSL_CPDMA_DMACONTROL_RX_TS_ENCAP_SHIFT                  ((uint32_t)(6U))
#define CSL_CPDMA_DMACONTROL_RX_TS_ENCAP_RESETVAL               ((uint32_t)(0x00000000U))
#define CSL_CPDMA_DMACONTROL_RX_TS_ENCAP_MAX                    ((uint32_t)(0x00000001U))

#define CSL_CPDMA_DMACONTROL_TX_RLIM_MASK                       ((uint32_t)(0x0000FF00U))
#define CSL_CPDMA_DMACONTROL_TX_RLIM_SHIFT                      ((uint32_t)(8U))
#define CSL_CPDMA_DMACONTROL_TX_RLIM_RESETVAL                   ((uint32_t)(0x00000000U))
#define CSL_CPDMA_DMACONTROL_TX_RLIM_MAX                        ((uint32_t)(0x000000ffU))

#define CSL_CPDMA_DMACONTROL_RESETVAL                           ((uint32_t)(0x00000000U))

/* DMASTATUS */

#define CSL_CPDMA_DMASTATUS_RX_ERR_CH_MASK                      ((uint32_t)(0x00000700U))
#define CSL_CPDMA_DMASTATUS_RX_ERR_CH_SHIFT                     ((uint32_t)(8U))
#define CSL_CPDMA_DMASTATUS_RX_ERR_CH_RESETVAL                  ((uint32_t)(0x00000000U))
#define CSL_CPDMA_DMASTATUS_RX_ERR_CH_MAX                       ((uint32_t)(0x00000007U))

#define CSL_CPDMA_DMASTATUS_RX_HOST_ERR_CODE_MASK               ((uint32_t)(0x0000F000U))
#define CSL_CPDMA_DMASTATUS_RX_HOST_ERR_CODE_SHIFT              ((uint32_t)(12U))
#define CSL_CPDMA_DMASTATUS_RX_HOST_ERR_CODE_RESETVAL           ((uint32_t)(0x00000000U))
#define CSL_CPDMA_DMASTATUS_RX_HOST_ERR_CODE_MAX                ((uint32_t)(0x0000000fU))

#define CSL_CPDMA_DMASTATUS_TX_ERR_CH_MASK                      ((uint32_t)(0x00070000U))
#define CSL_CPDMA_DMASTATUS_TX_ERR_CH_SHIFT                     ((uint32_t)(16U))
#define CSL_CPDMA_DMASTATUS_TX_ERR_CH_RESETVAL                  ((uint32_t)(0x00000000U))
#define CSL_CPDMA_DMASTATUS_TX_ERR_CH_MAX                       ((uint32_t)(0x00000007U))

#define CSL_CPDMA_DMASTATUS_TX_HOST_ERR_CODE_MASK               ((uint32_t)(0x00F00000U))
#define CSL_CPDMA_DMASTATUS_TX_HOST_ERR_CODE_SHIFT              ((uint32_t)(20U))
#define CSL_CPDMA_DMASTATUS_TX_HOST_ERR_CODE_RESETVAL           ((uint32_t)(0x00000000U))
#define CSL_CPDMA_DMASTATUS_TX_HOST_ERR_CODE_MAX                ((uint32_t)(0x0000000fU))

#define CSL_CPDMA_DMASTATUS_IDLE_MASK                           ((uint32_t)(0x80000000U))
#define CSL_CPDMA_DMASTATUS_IDLE_SHIFT                          ((uint32_t)(31U))
#define CSL_CPDMA_DMASTATUS_IDLE_RESETVAL                       ((uint32_t)(0x00000000U))
#define CSL_CPDMA_DMASTATUS_IDLE_MAX                            ((uint32_t)(0x00000001U))

#define CSL_CPDMA_DMASTATUS_RESETVAL                            ((uint32_t)(0x00000000U))

/* RX_BUFFER_OFFSET */

#define CSL_CPDMA_RX_BUFFER_OFFSET_RX_BUFFER_OFFSET_MASK        ((uint32_t)(0x0000FFFFU))
#define CSL_CPDMA_RX_BUFFER_OFFSET_RX_BUFFER_OFFSET_SHIFT       ((uint32_t)(0U))
#define CSL_CPDMA_RX_BUFFER_OFFSET_RX_BUFFER_OFFSET_RESETVAL    ((uint32_t)(0x00000000U))
#define CSL_CPDMA_RX_BUFFER_OFFSET_RX_BUFFER_OFFSET_MAX         ((uint32_t)(0x0000ffffU))

#define CSL_CPDMA_RX_BUFFER_OFFSET_RESETVAL                     ((uint32_t)(0x00000000U))

/* EMCONTROL */

#define CSL_CPDMA_EMCONTROL_FREE_MASK                           ((uint32_t)(0x00000001U))
#define CSL_CPDMA_EMCONTROL_FREE_SHIFT                          ((uint32_t)(0U))
#define CSL_CPDMA_EMCONTROL_FREE_RESETVAL                       ((uint32_t)(0x00000000U))
#define CSL_CPDMA_EMCONTROL_FREE_MAX                            ((uint32_t)(0x00000001U))

#define CSL_CPDMA_EMCONTROL_SOFT_MASK                           ((uint32_t)(0x00000002U))
#define CSL_CPDMA_EMCONTROL_SOFT_SHIFT                          ((uint32_t)(1U))
#define CSL_CPDMA_EMCONTROL_SOFT_RESETVAL                       ((uint32_t)(0x00000000U))
#define CSL_CPDMA_EMCONTROL_SOFT_MAX                            ((uint32_t)(0x00000001U))

#define CSL_CPDMA_EMCONTROL_RESETVAL                            ((uint32_t)(0x00000000U))

/* TX_PRI_RATE */

#define CSL_CPDMA_TX_PRI_RATE_PRI_SEND_CNT_MASK                 ((uint32_t)(0x00003FFFU))
#define CSL_CPDMA_TX_PRI_RATE_PRI_SEND_CNT_SHIFT                ((uint32_t)(0U))
#define CSL_CPDMA_TX_PRI_RATE_PRI_SEND_CNT_RESETVAL             ((uint32_t)(0x00000000U))
#define CSL_CPDMA_TX_PRI_RATE_PRI_SEND_CNT_MAX                  ((uint32_t)(0x00003fffU))

#define CSL_CPDMA_TX_PRI_RATE_PRI_IDLE_CNT_MASK                 ((uint32_t)(0x3FFF0000U))
#define CSL_CPDMA_TX_PRI_RATE_PRI_IDLE_CNT_SHIFT                ((uint32_t)(16U))
#define CSL_CPDMA_TX_PRI_RATE_PRI_IDLE_CNT_RESETVAL             ((uint32_t)(0x00000000U))
#define CSL_CPDMA_TX_PRI_RATE_PRI_IDLE_CNT_MAX                  ((uint32_t)(0x00003fffU))

#define CSL_CPDMA_TX_PRI_RATE_RESETVAL                          ((uint32_t)(0x00000000U))

/* TX_INTSTAT_RAW */

#define CSL_CPDMA_TX_INTSTAT_RAW_TX_PEND_MASK                   ((uint32_t)(0x000000FFU))
#define CSL_CPDMA_TX_INTSTAT_RAW_TX_PEND_SHIFT                  ((uint32_t)(0U))
#define CSL_CPDMA_TX_INTSTAT_RAW_TX_PEND_RESETVAL               ((uint32_t)(0x00000000U))
#define CSL_CPDMA_TX_INTSTAT_RAW_TX_PEND_MAX                    ((uint32_t)(0x000000ffU))

#define CSL_CPDMA_TX_INTSTAT_RAW_RESETVAL                       ((uint32_t)(0x00000000U))

/* TX_INTSTAT_MASKED */

#define CSL_CPDMA_TX_INTSTAT_MASKED_TX_PEND_MASK                ((uint32_t)(0x000000FFU))
#define CSL_CPDMA_TX_INTSTAT_MASKED_TX_PEND_SHIFT               ((uint32_t)(0U))
#define CSL_CPDMA_TX_INTSTAT_MASKED_TX_PEND_RESETVAL            ((uint32_t)(0x00000000U))
#define CSL_CPDMA_TX_INTSTAT_MASKED_TX_PEND_MAX                 ((uint32_t)(0x000000ffU))

#define CSL_CPDMA_TX_INTSTAT_MASKED_RESETVAL                    ((uint32_t)(0x00000000U))

/* TX_INTMASK_SET */

#define CSL_CPDMA_TX_INTMASK_SET_TX_MASK_MASK                   ((uint32_t)(0x000000FFU))
#define CSL_CPDMA_TX_INTMASK_SET_TX_MASK_SHIFT                  ((uint32_t)(0U))
#define CSL_CPDMA_TX_INTMASK_SET_TX_MASK_RESETVAL               ((uint32_t)(0x00000000U))
#define CSL_CPDMA_TX_INTMASK_SET_TX_MASK_MAX                    ((uint32_t)(0x000000ffU))

#define CSL_CPDMA_TX_INTMASK_SET_RESETVAL                       ((uint32_t)(0x00000000U))

/* TX_INTMASK_CLEAR */

#define CSL_CPDMA_TX_INTMASK_CLEAR_TX_MASK_MASK                 ((uint32_t)(0x000000FFU))
#define CSL_CPDMA_TX_INTMASK_CLEAR_TX_MASK_SHIFT                ((uint32_t)(0U))
#define CSL_CPDMA_TX_INTMASK_CLEAR_TX_MASK_RESETVAL             ((uint32_t)(0x00000000U))
#define CSL_CPDMA_TX_INTMASK_CLEAR_TX_MASK_MAX                  ((uint32_t)(0x000000ffU))

#define CSL_CPDMA_TX_INTMASK_CLEAR_RESETVAL                     ((uint32_t)(0x00000000U))

/* DMA_IN_VECTOR */

#define CSL_CPDMA_DMA_IN_VECTOR_DMA_IN_VECTOR_MASK              ((uint32_t)(0xFFFFFFFFU))
#define CSL_CPDMA_DMA_IN_VECTOR_DMA_IN_VECTOR_SHIFT             ((uint32_t)(0U))
#define CSL_CPDMA_DMA_IN_VECTOR_DMA_IN_VECTOR_RESETVAL          ((uint32_t)(0x00000000U))
#define CSL_CPDMA_DMA_IN_VECTOR_DMA_IN_VECTOR_MAX               ((uint32_t)(0xffffffffU))

#define CSL_CPDMA_DMA_IN_VECTOR_RESETVAL                        ((uint32_t)(0x00000000U))

/* DMA_EOI_VECTOR */

#define CSL_CPDMA_DMA_EOI_VECTOR_DMA_EOI_VECTOR_MASK            ((uint32_t)(0x0000001FU))
#define CSL_CPDMA_DMA_EOI_VECTOR_DMA_EOI_VECTOR_SHIFT           ((uint32_t)(0U))
#define CSL_CPDMA_DMA_EOI_VECTOR_DMA_EOI_VECTOR_RESETVAL        ((uint32_t)(0x00000000U))
#define CSL_CPDMA_DMA_EOI_VECTOR_DMA_EOI_VECTOR_MAX             ((uint32_t)(0x0000001fU))

#define CSL_CPDMA_DMA_EOI_VECTOR_RESETVAL                       ((uint32_t)(0x00000000U))

/* RX_INTSTAT_RAW */

#define CSL_CPDMA_RX_INTSTAT_RAW_RX_PEND_MASK                   ((uint32_t)(0x000000FFU))
#define CSL_CPDMA_RX_INTSTAT_RAW_RX_PEND_SHIFT                  ((uint32_t)(0U))
#define CSL_CPDMA_RX_INTSTAT_RAW_RX_PEND_RESETVAL               ((uint32_t)(0x00000000U))
#define CSL_CPDMA_RX_INTSTAT_RAW_RX_PEND_MAX                    ((uint32_t)(0x000000ffU))

#define CSL_CPDMA_RX_INTSTAT_RAW_RX_THRESH_PEND_MASK            ((uint32_t)(0x0000FF00U))
#define CSL_CPDMA_RX_INTSTAT_RAW_RX_THRESH_PEND_SHIFT           ((uint32_t)(8U))
#define CSL_CPDMA_RX_INTSTAT_RAW_RX_THRESH_PEND_RESETVAL        ((uint32_t)(0x00000000U))
#define CSL_CPDMA_RX_INTSTAT_RAW_RX_THRESH_PEND_MAX             ((uint32_t)(0x000000ffU))

#define CSL_CPDMA_RX_INTSTAT_RAW_RESETVAL                       ((uint32_t)(0x00000000U))

/* RX_INTSTAT_MASKED */

#define CSL_CPDMA_RX_INTSTAT_MASKED_RX_PEND_MASK                ((uint32_t)(0x000000FFU))
#define CSL_CPDMA_RX_INTSTAT_MASKED_RX_PEND_SHIFT               ((uint32_t)(0U))
#define CSL_CPDMA_RX_INTSTAT_MASKED_RX_PEND_RESETVAL            ((uint32_t)(0x00000000U))
#define CSL_CPDMA_RX_INTSTAT_MASKED_RX_PEND_MAX                 ((uint32_t)(0x000000ffU))

#define CSL_CPDMA_RX_INTSTAT_MASKED_RX_THRESH_PEND_MASK         ((uint32_t)(0x0000FF00U))
#define CSL_CPDMA_RX_INTSTAT_MASKED_RX_THRESH_PEND_SHIFT        ((uint32_t)(8U))
#define CSL_CPDMA_RX_INTSTAT_MASKED_RX_THRESH_PEND_RESETVAL     ((uint32_t)(0x00000000U))
#define CSL_CPDMA_RX_INTSTAT_MASKED_RX_THRESH_PEND_MAX          ((uint32_t)(0x000000ffU))

#define CSL_CPDMA_RX_INTSTAT_MASKED_RESETVAL                    ((uint32_t)(0x00000000U))

/* RX_INTMASK_SET */

#define CSL_CPDMA_RX_INTMASK_SET_RX_PEND_MASK_MASK              ((uint32_t)(0x000000FFU))
#define CSL_CPDMA_RX_INTMASK_SET_RX_PEND_MASK_SHIFT             ((uint32_t)(0U))
#define CSL_CPDMA_RX_INTMASK_SET_RX_PEND_MASK_RESETVAL          ((uint32_t)(0x00000000U))
#define CSL_CPDMA_RX_INTMASK_SET_RX_PEND_MASK_MAX               ((uint32_t)(0x000000ffU))

#define CSL_CPDMA_RX_INTMASK_SET_RX_THRESH_PEND_MASK_MASK       ((uint32_t)(0x0000FF00U))
#define CSL_CPDMA_RX_INTMASK_SET_RX_THRESH_PEND_MASK_SHIFT      ((uint32_t)(8U))
#define CSL_CPDMA_RX_INTMASK_SET_RX_THRESH_PEND_MASK_RESETVAL   ((uint32_t)(0x00000000U))
#define CSL_CPDMA_RX_INTMASK_SET_RX_THRESH_PEND_MASK_MAX        ((uint32_t)(0x000000ffU))

#define CSL_CPDMA_RX_INTMASK_SET_RESETVAL                       ((uint32_t)(0x00000000U))

/* RX_INTMASK_CLEAR */

#define CSL_CPDMA_RX_INTMASK_CLEAR_RX_PEND_MASK_MASK            ((uint32_t)(0x000000FFU))
#define CSL_CPDMA_RX_INTMASK_CLEAR_RX_PEND_MASK_SHIFT           ((uint32_t)(0U))
#define CSL_CPDMA_RX_INTMASK_CLEAR_RX_PEND_MASK_RESETVAL        ((uint32_t)(0x00000000U))
#define CSL_CPDMA_RX_INTMASK_CLEAR_RX_PEND_MASK_MAX             ((uint32_t)(0x000000ffU))

#define CSL_CPDMA_RX_INTMASK_CLEAR_RX_THRESH_PEND_MASK_MASK     ((uint32_t)(0x0000FF00U))
#define CSL_CPDMA_RX_INTMASK_CLEAR_RX_THRESH_PEND_MASK_SHIFT    ((uint32_t)(8U))
#define CSL_CPDMA_RX_INTMASK_CLEAR_RX_THRESH_PEND_MASK_RESETVAL  ((uint32_t)(0x00000000U))
#define CSL_CPDMA_RX_INTMASK_CLEAR_RX_THRESH_PEND_MASK_MAX      ((uint32_t)(0x000000ffU))

#define CSL_CPDMA_RX_INTMASK_CLEAR_RESETVAL                     ((uint32_t)(0x00000000U))

/* DMA_INTSTAT_RAW */

#define CSL_CPDMA_DMA_INTSTAT_RAW_STAT_PEND_MASK                ((uint32_t)(0x00000001U))
#define CSL_CPDMA_DMA_INTSTAT_RAW_STAT_PEND_SHIFT               ((uint32_t)(0U))
#define CSL_CPDMA_DMA_INTSTAT_RAW_STAT_PEND_RESETVAL            ((uint32_t)(0x00000000U))
#define CSL_CPDMA_DMA_INTSTAT_RAW_STAT_PEND_MAX                 ((uint32_t)(0x00000001U))

#define CSL_CPDMA_DMA_INTSTAT_RAW_HOST_PEND_MASK                ((uint32_t)(0x00000002U))
#define CSL_CPDMA_DMA_INTSTAT_RAW_HOST_PEND_SHIFT               ((uint32_t)(1U))
#define CSL_CPDMA_DMA_INTSTAT_RAW_HOST_PEND_RESETVAL            ((uint32_t)(0x00000000U))
#define CSL_CPDMA_DMA_INTSTAT_RAW_HOST_PEND_MAX                 ((uint32_t)(0x00000001U))

#define CSL_CPDMA_DMA_INTSTAT_RAW_RESETVAL                      ((uint32_t)(0x00000000U))

/* DMA_INTSTAT_MASKED */

#define CSL_CPDMA_DMA_INTSTAT_MASKED_STAT_PEND_MASK             ((uint32_t)(0x00000001U))
#define CSL_CPDMA_DMA_INTSTAT_MASKED_STAT_PEND_SHIFT            ((uint32_t)(0U))
#define CSL_CPDMA_DMA_INTSTAT_MASKED_STAT_PEND_RESETVAL         ((uint32_t)(0x00000000U))
#define CSL_CPDMA_DMA_INTSTAT_MASKED_STAT_PEND_MAX              ((uint32_t)(0x00000001U))

#define CSL_CPDMA_DMA_INTSTAT_MASKED_HOST_PEND_MASK             ((uint32_t)(0x00000002U))
#define CSL_CPDMA_DMA_INTSTAT_MASKED_HOST_PEND_SHIFT            ((uint32_t)(1U))
#define CSL_CPDMA_DMA_INTSTAT_MASKED_HOST_PEND_RESETVAL         ((uint32_t)(0x00000000U))
#define CSL_CPDMA_DMA_INTSTAT_MASKED_HOST_PEND_MAX              ((uint32_t)(0x00000001U))

#define CSL_CPDMA_DMA_INTSTAT_MASKED_RESETVAL                   ((uint32_t)(0x00000000U))

/* DMA_INTMASK_SET */

#define CSL_CPDMA_DMA_INTMASK_SET_STAT_INT_MASK_MASK            ((uint32_t)(0x00000001U))
#define CSL_CPDMA_DMA_INTMASK_SET_STAT_INT_MASK_SHIFT           ((uint32_t)(0U))
#define CSL_CPDMA_DMA_INTMASK_SET_STAT_INT_MASK_RESETVAL        ((uint32_t)(0x00000000U))
#define CSL_CPDMA_DMA_INTMASK_SET_STAT_INT_MASK_MAX             ((uint32_t)(0x00000001U))

#define CSL_CPDMA_DMA_INTMASK_SET_HOST_ERR_INT_MASK_MASK        ((uint32_t)(0x00000002U))
#define CSL_CPDMA_DMA_INTMASK_SET_HOST_ERR_INT_MASK_SHIFT       ((uint32_t)(1U))
#define CSL_CPDMA_DMA_INTMASK_SET_HOST_ERR_INT_MASK_RESETVAL    ((uint32_t)(0x00000000U))
#define CSL_CPDMA_DMA_INTMASK_SET_HOST_ERR_INT_MASK_MAX         ((uint32_t)(0x00000001U))

#define CSL_CPDMA_DMA_INTMASK_SET_RESETVAL                      ((uint32_t)(0x00000000U))

/* DMA_INTMASK_CLEAR */

#define CSL_CPDMA_DMA_INTMASK_CLEAR_STAT_INT_MASK_MASK          ((uint32_t)(0x00000001U))
#define CSL_CPDMA_DMA_INTMASK_CLEAR_STAT_INT_MASK_SHIFT         ((uint32_t)(0U))
#define CSL_CPDMA_DMA_INTMASK_CLEAR_STAT_INT_MASK_RESETVAL      ((uint32_t)(0x00000000U))
#define CSL_CPDMA_DMA_INTMASK_CLEAR_STAT_INT_MASK_MAX           ((uint32_t)(0x00000001U))

#define CSL_CPDMA_DMA_INTMASK_CLEAR_HOST_ERR_INT_MASK_MASK      ((uint32_t)(0x00000002U))
#define CSL_CPDMA_DMA_INTMASK_CLEAR_HOST_ERR_INT_MASK_SHIFT     ((uint32_t)(1U))
#define CSL_CPDMA_DMA_INTMASK_CLEAR_HOST_ERR_INT_MASK_RESETVAL  ((uint32_t)(0x00000000U))
#define CSL_CPDMA_DMA_INTMASK_CLEAR_HOST_ERR_INT_MASK_MAX       ((uint32_t)(0x00000001U))

#define CSL_CPDMA_DMA_INTMASK_CLEAR_RESETVAL                    ((uint32_t)(0x00000000U))

/* RX_PENDTHRESH */

#define CSL_CPDMA_RX_PENDTHRESH_RX_PENDTHRESH_MASK              ((uint32_t)(0x000000FFU))
#define CSL_CPDMA_RX_PENDTHRESH_RX_PENDTHRESH_SHIFT             ((uint32_t)(0U))
#define CSL_CPDMA_RX_PENDTHRESH_RX_PENDTHRESH_RESETVAL          ((uint32_t)(0x00000000U))
#define CSL_CPDMA_RX_PENDTHRESH_RX_PENDTHRESH_MAX               ((uint32_t)(0x000000ffU))

#define CSL_CPDMA_RX_PENDTHRESH_RESETVAL                        ((uint32_t)(0x00000000U))

/* RX_FREEBUFFER */

#define CSL_CPDMA_RX_FREEBUFFER_RX_FREEBUFFER_MASK              ((uint32_t)(0x0000FFFFU))
#define CSL_CPDMA_RX_FREEBUFFER_RX_FREEBUFFER_SHIFT             ((uint32_t)(0U))
#define CSL_CPDMA_RX_FREEBUFFER_RX_FREEBUFFER_RESETVAL          ((uint32_t)(0x00000000U))
#define CSL_CPDMA_RX_FREEBUFFER_RX_FREEBUFFER_MAX               ((uint32_t)(0x0000ffffU))

#define CSL_CPDMA_RX_FREEBUFFER_RESETVAL                        ((uint32_t)(0x00000000U))

/* TX_HDP */

#define CSL_CPDMA_TX_HDP_TX_HDP_MASK                            ((uint32_t)(0xFFFFFFFFU))
#define CSL_CPDMA_TX_HDP_TX_HDP_SHIFT                           ((uint32_t)(0U))
#define CSL_CPDMA_TX_HDP_TX_HDP_RESETVAL                        ((uint32_t)(0x00000000U))
#define CSL_CPDMA_TX_HDP_TX_HDP_MAX                             ((uint32_t)(0xffffffffU))

#define CSL_CPDMA_TX_HDP_RESETVAL                               ((uint32_t)(0x00000000U))

/* RX_HDP */

#define CSL_CPDMA_RX_HDP_RX_HDP_MASK                            ((uint32_t)(0xFFFFFFFFU))
#define CSL_CPDMA_RX_HDP_RX_HDP_SHIFT                           ((uint32_t)(0U))
#define CSL_CPDMA_RX_HDP_RX_HDP_RESETVAL                        ((uint32_t)(0x00000000U))
#define CSL_CPDMA_RX_HDP_RX_HDP_MAX                             ((uint32_t)(0xffffffffU))

#define CSL_CPDMA_RX_HDP_RESETVAL                               ((uint32_t)(0x00000000U))

/* TX_CP */

#define CSL_CPDMA_TX_CP_TX_CP_MASK                              ((uint32_t)(0xFFFFFFFFU))
#define CSL_CPDMA_TX_CP_TX_CP_SHIFT                             ((uint32_t)(0U))
#define CSL_CPDMA_TX_CP_TX_CP_RESETVAL                          ((uint32_t)(0x00000000U))
#define CSL_CPDMA_TX_CP_TX_CP_MAX                               ((uint32_t)(0xffffffffU))

#define CSL_CPDMA_TX_CP_RESETVAL                                ((uint32_t)(0x00000000U))

/* RX_CP */

#define CSL_CPDMA_RX_CP_RX_CP_MASK                              ((uint32_t)(0xFFFFFFFFU))
#define CSL_CPDMA_RX_CP_RX_CP_SHIFT                             ((uint32_t)(0U))
#define CSL_CPDMA_RX_CP_RX_CP_RESETVAL                          ((uint32_t)(0x00000000U))
#define CSL_CPDMA_RX_CP_RX_CP_MAX                               ((uint32_t)(0xffffffffU))

#define CSL_CPDMA_RX_CP_RESETVAL                                ((uint32_t)(0x00000000U))

#ifdef __cplusplus
}
#endif
#endif
