/********************************************************************
 * Copyright (C) 2017 Texas Instruments Incorporated.
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
 *  Name        : cslr_ringacc.h
*/
#ifndef CSLR_RINGACC_H_
#define CSLR_RINGACC_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  : Ring Accelerator Ring ISC Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t CONTROL;                   /* ISC a Region b Control Register */
    volatile uint32_t CONTROL2;                  /* ISC a Region b Control Register 2 */
    volatile uint8_t  Resv_32[24];
} CSL_ringacc_iscRegs_ep;


typedef struct {
    CSL_ringacc_iscRegs_ep EP[1024];
} CSL_ringacc_iscRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_RINGACC_ISC_EP_CONTROL(EP)                                         (0x00000000U+((EP)*0x20U))
#define CSL_RINGACC_ISC_EP_CONTROL2(EP)                                        (0x00000004U+((EP)*0x20U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* CONTROL */

#define CSL_RINGACC_ISC_EP_CONTROL_ENABLE_MASK                                 (0x0000000FU)
#define CSL_RINGACC_ISC_EP_CONTROL_ENABLE_SHIFT                                (0x00000000U)
#define CSL_RINGACC_ISC_EP_CONTROL_ENABLE_MAX                                  (0x0000000FU)

#define CSL_RINGACC_ISC_EP_CONTROL_LOCK_MASK                                   (0x00000010U)
#define CSL_RINGACC_ISC_EP_CONTROL_LOCK_SHIFT                                  (0x00000004U)
#define CSL_RINGACC_ISC_EP_CONTROL_LOCK_MAX                                    (0x00000001U)

#define CSL_RINGACC_ISC_EP_CONTROL_PRIV_ID_MASK                                (0x0000FF00U)
#define CSL_RINGACC_ISC_EP_CONTROL_PRIV_ID_SHIFT                               (0x00000008U)
#define CSL_RINGACC_ISC_EP_CONTROL_PRIV_ID_MAX                                 (0x000000FFU)

#define CSL_RINGACC_ISC_EP_CONTROL_SEC_MASK                                    (0x000F0000U)
#define CSL_RINGACC_ISC_EP_CONTROL_SEC_SHIFT                                   (0x00000010U)
#define CSL_RINGACC_ISC_EP_CONTROL_SEC_MAX                                     (0x0000000FU)

#define CSL_RINGACC_ISC_EP_CONTROL_NONSEC_MASK                                 (0x00100000U)
#define CSL_RINGACC_ISC_EP_CONTROL_NONSEC_SHIFT                                (0x00000014U)
#define CSL_RINGACC_ISC_EP_CONTROL_NONSEC_MAX                                  (0x00000001U)

#define CSL_RINGACC_ISC_EP_CONTROL_PASS_MASK                                   (0x00200000U)
#define CSL_RINGACC_ISC_EP_CONTROL_PASS_SHIFT                                  (0x00000015U)
#define CSL_RINGACC_ISC_EP_CONTROL_PASS_MAX                                    (0x00000001U)

#define CSL_RINGACC_ISC_EP_CONTROL_PRIV_MASK                                   (0x03000000U)
#define CSL_RINGACC_ISC_EP_CONTROL_PRIV_SHIFT                                  (0x00000018U)
#define CSL_RINGACC_ISC_EP_CONTROL_PRIV_MAX                                    (0x00000003U)

#define CSL_RINGACC_ISC_EP_CONTROL_NOPRIV_MASK                                 (0x0C000000U)
#define CSL_RINGACC_ISC_EP_CONTROL_NOPRIV_SHIFT                                (0x0000001AU)
#define CSL_RINGACC_ISC_EP_CONTROL_NOPRIV_MAX                                  (0x00000003U)

/* CONTROL2 */

#define CSL_RINGACC_ISC_EP_CONTROL2_PASS_V_MASK                                (0x80000000U)
#define CSL_RINGACC_ISC_EP_CONTROL2_PASS_V_SHIFT                               (0x0000001FU)
#define CSL_RINGACC_ISC_EP_CONTROL2_PASS_V_MAX                                 (0x00000001U)

#define CSL_RINGACC_ISC_EP_CONTROL2_VIRTID_MASK                                (0x0FFF0000U)
#define CSL_RINGACC_ISC_EP_CONTROL2_VIRTID_SHIFT                               (0x00000010U)
#define CSL_RINGACC_ISC_EP_CONTROL2_VIRTID_MAX                                 (0x00000FFFU)

/**************************************************************************
* Hardware Region  : Ring Accelerator Global Control / Status Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t REVISION;                  /* Revision Register */
    volatile uint8_t  Resv_16[12];
    volatile uint32_t TRACE_CTL;                 /* Trace Control Register */
    volatile uint8_t  Resv_32[12];
    volatile uint32_t OVRFLOW;                   /* Overflow Queue Register */
    volatile uint8_t  Resv_64[28];
    volatile uint32_t ERROR_EVT;                 /* Error Event Register */
    volatile uint32_t ERROR_LOG;                 /* Error Log Register */
} CSL_ringacc_gcfgRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_RINGACC_GCFG_REVISION                                              (0x00000000U)
#define CSL_RINGACC_GCFG_TRACE_CTL                                             (0x00000010U)
#define CSL_RINGACC_GCFG_OVRFLOW                                               (0x00000020U)
#define CSL_RINGACC_GCFG_ERROR_EVT                                             (0x00000040U)
#define CSL_RINGACC_GCFG_ERROR_LOG                                             (0x00000044U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* REVISION */

#define CSL_RINGACC_GCFG_REVISION_MODID_MASK                                   (0xFFFF0000U)
#define CSL_RINGACC_GCFG_REVISION_MODID_SHIFT                                  (0x00000010U)
#define CSL_RINGACC_GCFG_REVISION_MODID_MAX                                    (0x0000FFFFU)

#define CSL_RINGACC_GCFG_REVISION_REVRTL_MASK                                  (0x0000F800U)
#define CSL_RINGACC_GCFG_REVISION_REVRTL_SHIFT                                 (0x0000000BU)
#define CSL_RINGACC_GCFG_REVISION_REVRTL_MAX                                   (0x0000001FU)

#define CSL_RINGACC_GCFG_REVISION_REVMAJ_MASK                                  (0x00000700U)
#define CSL_RINGACC_GCFG_REVISION_REVMAJ_SHIFT                                 (0x00000008U)
#define CSL_RINGACC_GCFG_REVISION_REVMAJ_MAX                                   (0x00000007U)

#define CSL_RINGACC_GCFG_REVISION_CUSTOM_MASK                                  (0x000000C0U)
#define CSL_RINGACC_GCFG_REVISION_CUSTOM_SHIFT                                 (0x00000006U)
#define CSL_RINGACC_GCFG_REVISION_CUSTOM_MAX                                   (0x00000003U)

#define CSL_RINGACC_GCFG_REVISION_REVMIN_MASK                                  (0x0000003FU)
#define CSL_RINGACC_GCFG_REVISION_REVMIN_SHIFT                                 (0x00000000U)
#define CSL_RINGACC_GCFG_REVISION_REVMIN_MAX                                   (0x0000003FU)

/* TRACE_CTL */

#define CSL_RINGACC_GCFG_TRACE_CTL_EN_MASK                                     (0x80000000U)
#define CSL_RINGACC_GCFG_TRACE_CTL_EN_SHIFT                                    (0x0000001FU)
#define CSL_RINGACC_GCFG_TRACE_CTL_EN_MAX                                      (0x00000001U)

#define CSL_RINGACC_GCFG_TRACE_CTL_ALL_QUEUES_MASK                             (0x40000000U)
#define CSL_RINGACC_GCFG_TRACE_CTL_ALL_QUEUES_SHIFT                            (0x0000001EU)
#define CSL_RINGACC_GCFG_TRACE_CTL_ALL_QUEUES_MAX                              (0x00000001U)

#define CSL_RINGACC_GCFG_TRACE_CTL_MSG_MASK                                    (0x20000000U)
#define CSL_RINGACC_GCFG_TRACE_CTL_MSG_SHIFT                                   (0x0000001DU)
#define CSL_RINGACC_GCFG_TRACE_CTL_MSG_MAX                                     (0x00000001U)

#define CSL_RINGACC_GCFG_TRACE_CTL_QUEUE_MASK                                  (0x0000FFFFU)
#define CSL_RINGACC_GCFG_TRACE_CTL_QUEUE_SHIFT                                 (0x00000000U)
#define CSL_RINGACC_GCFG_TRACE_CTL_QUEUE_MAX                                   (0x0000FFFFU)

/* OVRFLOW */

#define CSL_RINGACC_GCFG_OVRFLOW_QUEUE_MASK                                    (0x0000FFFFU)
#define CSL_RINGACC_GCFG_OVRFLOW_QUEUE_SHIFT                                   (0x00000000U)
#define CSL_RINGACC_GCFG_OVRFLOW_QUEUE_MAX                                     (0x0000FFFFU)

/* ERROR_EVT */

#define CSL_RINGACC_GCFG_ERROR_EVT_EVT_MASK                                    (0x0000FFFFU)
#define CSL_RINGACC_GCFG_ERROR_EVT_EVT_SHIFT                                   (0x00000000U)
#define CSL_RINGACC_GCFG_ERROR_EVT_EVT_MAX                                     (0x0000FFFFU)

/* ERROR_LOG */

#define CSL_RINGACC_GCFG_ERROR_LOG_PUSH_MASK                                   (0x80000000U)
#define CSL_RINGACC_GCFG_ERROR_LOG_PUSH_SHIFT                                  (0x0000001FU)
#define CSL_RINGACC_GCFG_ERROR_LOG_PUSH_MAX                                    (0x00000001U)

#define CSL_RINGACC_GCFG_ERROR_LOG_QUEUE_MASK                                  (0x0000FFFFU)
#define CSL_RINGACC_GCFG_ERROR_LOG_QUEUE_SHIFT                                 (0x00000000U)
#define CSL_RINGACC_GCFG_ERROR_LOG_QUEUE_MAX                                   (0x0000FFFFU)

/**************************************************************************
* Hardware Region  : Ring Accelerator Ring Control / Status Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint8_t  Resv_64[64];
    volatile uint32_t BA_LO;                     /* Ring Base Address Lo Register */
    volatile uint32_t BA_HI;                     /* Ring Base Address Hi Register */
    volatile uint32_t SIZE;                      /* Ring Size Register */
    volatile uint32_t EVENT;                     /* Ring Event Register */
    volatile uint32_t ORDERID;                   /* Ring OrderID Register */
    volatile uint8_t  Resv_256[172];
} CSL_ringacc_cfgRegs_RING;


typedef struct {
    CSL_ringacc_cfgRegs_RING RING[1024];
} CSL_ringacc_cfgRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_RINGACC_CFG_RING_BA_LO(RING)                                       (0x00000040U+((RING)*0x100U))
#define CSL_RINGACC_CFG_RING_BA_HI(RING)                                       (0x00000044U+((RING)*0x100U))
#define CSL_RINGACC_CFG_RING_SIZE(RING)                                        (0x00000048U+((RING)*0x100U))
#define CSL_RINGACC_CFG_RING_EVENT(RING)                                       (0x0000004CU+((RING)*0x100U))
#define CSL_RINGACC_CFG_RING_ORDERID(RING)                                     (0x00000050U+((RING)*0x100U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* BA_LO */

#define CSL_RINGACC_CFG_RING_BA_LO_ADDR_LO_MASK                                (0xFFFFFFFFU)
#define CSL_RINGACC_CFG_RING_BA_LO_ADDR_LO_SHIFT                               (0x00000000U)
#define CSL_RINGACC_CFG_RING_BA_LO_ADDR_LO_MAX                                 (0xFFFFFFFFU)

/* BA_HI */

#define CSL_RINGACC_CFG_RING_BA_HI_ADDR_HI_MASK                                (0x0000FFFFU)
#define CSL_RINGACC_CFG_RING_BA_HI_ADDR_HI_SHIFT                               (0x00000000U)
#define CSL_RINGACC_CFG_RING_BA_HI_ADDR_HI_MAX                                 (0x0000FFFFU)

/* SIZE */

#define CSL_RINGACC_CFG_RING_SIZE_QMODE_MASK                                   (0xC0000000U)
#define CSL_RINGACC_CFG_RING_SIZE_QMODE_SHIFT                                  (0x0000001EU)
#define CSL_RINGACC_CFG_RING_SIZE_QMODE_MAX                                    (0x00000003U)

#define CSL_RINGACC_CFG_RING_SIZE_QMODE_VAL_RING_MODE                          (0x0U)
#define CSL_RINGACC_CFG_RING_SIZE_QMODE_VAL_MESSAGE_MODE                       (0x1U)
#define CSL_RINGACC_CFG_RING_SIZE_QMODE_VAL_CREDENTIALS_MODE                   (0x2U)
#define CSL_RINGACC_CFG_RING_SIZE_QMODE_VAL_QM_MODE                            (0x3U)

#define CSL_RINGACC_CFG_RING_SIZE_ELSIZE_MASK                                  (0x07000000U)
#define CSL_RINGACC_CFG_RING_SIZE_ELSIZE_SHIFT                                 (0x00000018U)
#define CSL_RINGACC_CFG_RING_SIZE_ELSIZE_MAX                                   (0x00000007U)

#define CSL_RINGACC_CFG_RING_SIZE_ELCNT_MASK                                   (0x000FFFFFU)
#define CSL_RINGACC_CFG_RING_SIZE_ELCNT_SHIFT                                  (0x00000000U)
#define CSL_RINGACC_CFG_RING_SIZE_ELCNT_MAX                                    (0x000FFFFFU)

/* EVENT */

#define CSL_RINGACC_CFG_RING_EVENT_EVENT_MASK                                  (0x0000FFFFU)
#define CSL_RINGACC_CFG_RING_EVENT_EVENT_SHIFT                                 (0x00000000U)
#define CSL_RINGACC_CFG_RING_EVENT_EVENT_MAX                                   (0x0000FFFFU)

/* ORDERID */

#define CSL_RINGACC_CFG_RING_ORDERID_REPLACE_MASK                              (0x00000010U)
#define CSL_RINGACC_CFG_RING_ORDERID_REPLACE_SHIFT                             (0x00000004U)
#define CSL_RINGACC_CFG_RING_ORDERID_REPLACE_MAX                               (0x00000001U)

#define CSL_RINGACC_CFG_RING_ORDERID_ORDERID_MASK                              (0x0000000FU)
#define CSL_RINGACC_CFG_RING_ORDERID_ORDERID_SHIFT                             (0x00000000U)
#define CSL_RINGACC_CFG_RING_ORDERID_ORDERID_MAX                               (0x0000000FU)

/**************************************************************************
* Hardware Region  : Ring Accelerator Control / Status Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint8_t  Resv_16[16];
    volatile uint32_t DB;                        /* Realtime Ring N Doorbell Register */
    volatile uint8_t  Resv_24[4];
    volatile uint32_t OCC;                       /* Realtime Ring N Occupancy Register */
    volatile uint32_t INDX;                      /* Realtime Ring N Current Index Register */
    volatile uint32_t HWOCC;                     /* Realtime Ring N Hardware Occupancy Register */
    volatile uint32_t HWINDX;                    /* Realtime Ring N Current Index Register */
    volatile uint8_t  Resv_4096[4056];
} CSL_ringacc_rtRegs_RINGRT;


typedef struct {
    CSL_ringacc_rtRegs_RINGRT RINGRT[1024];
} CSL_ringacc_rtRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_RINGACC_RT_RINGRT_DB(RINGRT)                                       (0x00000010U+((RINGRT)*0x1000U))
#define CSL_RINGACC_RT_RINGRT_OCC(RINGRT)                                      (0x00000018U+((RINGRT)*0x1000U))
#define CSL_RINGACC_RT_RINGRT_INDX(RINGRT)                                     (0x0000001CU+((RINGRT)*0x1000U))
#define CSL_RINGACC_RT_RINGRT_HWOCC(RINGRT)                                    (0x00000020U+((RINGRT)*0x1000U))
#define CSL_RINGACC_RT_RINGRT_HWINDX(RINGRT)                                   (0x00000024U+((RINGRT)*0x1000U))

/**************************************************************************
* 64K Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint8_t  Resv_16[16];
    volatile uint32_t DB;                        /* Realtime Ring N Doorbell Register */
    volatile uint8_t  Resv_24[4];
    volatile uint32_t OCC;                       /* Realtime Ring N Occupancy Register */
    volatile uint32_t INDX;                      /* Realtime Ring N Current Index Register */
    volatile uint32_t HWOCC;                     /* Realtime Ring N Hardware Occupancy Register */
    volatile uint32_t HWINDX;                    /* Realtime Ring N Current Index Register */
    volatile uint8_t  Resv_65536[65496];
} CSL_a64_ringacc_rtRegs_RINGRT;


typedef struct {
    CSL_a64_ringacc_rtRegs_RINGRT RINGRT[1024];
} CSL_a64_ringacc_rtRegs;


/**************************************************************************
* 64K Register Macros
**************************************************************************/

#define CSL_A64_RINGACC_RT_RINGRT_DB(RINGRT)                                   (0x00000010U+((RINGRT)*0x10000U))
#define CSL_A64_RINGACC_RT_RINGRT_OCC(RINGRT)                                  (0x00000018U+((RINGRT)*0x10000U))
#define CSL_A64_RINGACC_RT_RINGRT_INDX(RINGRT)                                 (0x0000001CU+((RINGRT)*0x10000U))
#define CSL_A64_RINGACC_RT_RINGRT_HWOCC(RINGRT)                                (0x00000020U+((RINGRT)*0x10000U))
#define CSL_A64_RINGACC_RT_RINGRT_HWINDX(RINGRT)                               (0x00000024U+((RINGRT)*0x10000U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* DB */

#define CSL_RINGACC_RT_RINGRT_DB_CNT_MASK                                      (0x000000FFU)
#define CSL_RINGACC_RT_RINGRT_DB_CNT_SHIFT                                     (0x00000000U)
#define CSL_RINGACC_RT_RINGRT_DB_CNT_MAX                                       (0x000000FFU)

/* OCC */

#define CSL_RINGACC_RT_RINGRT_OCC_CNT_MASK                                     (0x001FFFFFU)
#define CSL_RINGACC_RT_RINGRT_OCC_CNT_SHIFT                                    (0x00000000U)
#define CSL_RINGACC_RT_RINGRT_OCC_CNT_MAX                                      (0x001FFFFFU)

/* INDX */

#define CSL_RINGACC_RT_RINGRT_INDX_IDX_MASK                                    (0x000FFFFFU)
#define CSL_RINGACC_RT_RINGRT_INDX_IDX_SHIFT                                   (0x00000000U)
#define CSL_RINGACC_RT_RINGRT_INDX_IDX_MAX                                     (0x000FFFFFU)

/* HWOCC */

#define CSL_RINGACC_RT_RINGRT_HWOCC_CNT_MASK                                   (0x001FFFFFU)
#define CSL_RINGACC_RT_RINGRT_HWOCC_CNT_SHIFT                                  (0x00000000U)
#define CSL_RINGACC_RT_RINGRT_HWOCC_CNT_MAX                                    (0x001FFFFFU)

/* HWINDX */

#define CSL_RINGACC_RT_RINGRT_HWINDX_IDX_MASK                                  (0x000FFFFFU)
#define CSL_RINGACC_RT_RINGRT_HWINDX_IDX_SHIFT                                 (0x00000000U)
#define CSL_RINGACC_RT_RINGRT_HWINDX_IDX_MAX                                   (0x000FFFFFU)

/**************************************************************************
* Hardware Region  : Ring Accelerator Monitor Control / Status Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t CONTROL;                   /* Monitor Control Register */
    volatile uint32_t QUEUE;                     /* Monitor Queue Register */
    volatile uint32_t DATA0;                     /* Monitor Data Register */
    volatile uint32_t DATA1;                     /* Monitor Data Register */
    volatile uint8_t  Resv_4096[4080];
} CSL_ringacc_monitorRegs_mon;


typedef struct {
    CSL_ringacc_monitorRegs_mon MON[64];
} CSL_ringacc_monitorRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_RINGACC_MONITOR_MON_CONTROL(MON)                                   (0x00000000U+((MON)*0x1000U))
#define CSL_RINGACC_MONITOR_MON_QUEUE(MON)                                     (0x00000004U+((MON)*0x1000U))
#define CSL_RINGACC_MONITOR_MON_DATA0(MON)                                     (0x00000008U+((MON)*0x1000U))
#define CSL_RINGACC_MONITOR_MON_DATA1(MON)                                     (0x0000000CU+((MON)*0x1000U))

/**************************************************************************
* 64K Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t CONTROL;                   /* Monitor Control Register */
    volatile uint32_t QUEUE;                     /* Monitor Queue Register */
    volatile uint32_t DATA0;                     /* Monitor Data Register */
    volatile uint32_t DATA1;                     /* Monitor Data Register */
    volatile uint8_t  Resv_65536[65520];
} CSL_a64_ringacc_monitorRegs_mon;


typedef struct {
    CSL_a64_ringacc_monitorRegs_mon MON[64];
} CSL_a64_ringacc_monitorRegs;


/**************************************************************************
* 64K Register Macros
**************************************************************************/

#define CSL_A64_RINGACC_MONITOR_MON_CONTROL(MON)                               (0x00000000U+((MON)*0x10000U))
#define CSL_A64_RINGACC_MONITOR_MON_QUEUE(MON)                                 (0x00000004U+((MON)*0x10000U))
#define CSL_A64_RINGACC_MONITOR_MON_DATA0(MON)                                 (0x00000008U+((MON)*0x10000U))
#define CSL_A64_RINGACC_MONITOR_MON_DATA1(MON)                                 (0x0000000CU+((MON)*0x10000U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* CONTROL */

#define CSL_RINGACC_MONITOR_MON_CONTROL_EVT_MASK                               (0xFFFF0000U)
#define CSL_RINGACC_MONITOR_MON_CONTROL_EVT_SHIFT                              (0x00000010U)
#define CSL_RINGACC_MONITOR_MON_CONTROL_EVT_MAX                                (0x0000FFFFU)

#define CSL_RINGACC_MONITOR_MON_CONTROL_SOURCE_MASK                            (0x00000F00U)
#define CSL_RINGACC_MONITOR_MON_CONTROL_SOURCE_SHIFT                           (0x00000008U)
#define CSL_RINGACC_MONITOR_MON_CONTROL_SOURCE_MAX                             (0x0000000FU)

#define CSL_RINGACC_MONITOR_MON_CONTROL_MODE_MASK                              (0x00000007U)
#define CSL_RINGACC_MONITOR_MON_CONTROL_MODE_SHIFT                             (0x00000000U)
#define CSL_RINGACC_MONITOR_MON_CONTROL_MODE_MAX                               (0x00000007U)

/* QUEUE */

#define CSL_RINGACC_MONITOR_MON_QUEUE_VAL_MASK                                 (0x0000FFFFU)
#define CSL_RINGACC_MONITOR_MON_QUEUE_VAL_SHIFT                                (0x00000000U)
#define CSL_RINGACC_MONITOR_MON_QUEUE_VAL_MAX                                  (0x0000FFFFU)

/* DATA0 */

#define CSL_RINGACC_MONITOR_MON_DATA0_VAL_MASK                                 (0xFFFFFFFFU)
#define CSL_RINGACC_MONITOR_MON_DATA0_VAL_SHIFT                                (0x00000000U)
#define CSL_RINGACC_MONITOR_MON_DATA0_VAL_MAX                                  (0xFFFFFFFFU)

/* DATA1 */

#define CSL_RINGACC_MONITOR_MON_DATA1_VAL_MASK                                 (0xFFFFFFFFU)
#define CSL_RINGACC_MONITOR_MON_DATA1_VAL_SHIFT                                (0x00000000U)
#define CSL_RINGACC_MONITOR_MON_DATA1_VAL_MAX                                  (0xFFFFFFFFU)

/**************************************************************************
* Hardware Region  : Ring Accelerator Queues Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t RINGHEADDATA[128];         /* Ring Head Entry Data Registers */
    volatile uint32_t RINGTAILDATA[128];         /* Ring Tail Entry Data Registers */
    volatile uint32_t PEEKHEADDATA[128];         /* Ring Peek Head Entry Data Registers */
    volatile uint32_t PEEKTAILDATA[128];         /* Ring Peek Tail Entry Data Registers */
    volatile uint8_t  Resv_4096[2048];
} CSL_ringacc_fifosRegs_FIFO;


typedef struct {
    CSL_ringacc_fifosRegs_FIFO FIFO[1024];
} CSL_ringacc_fifosRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_RINGACC_FIFOS_FIFO_RINGHEADDATA(FIFO,RINGHEADDATA)                 (0x00000000U+((FIFO)*0x1000U)+((RINGHEADDATA)*0x4U))
#define CSL_RINGACC_FIFOS_FIFO_RINGTAILDATA(FIFO,RINGTAILDATA)                 (0x00000200U+((FIFO)*0x1000U)+((RINGTAILDATA)*0x4U))
#define CSL_RINGACC_FIFOS_FIFO_PEEKHEADDATA(FIFO,PEEKHEADDATA)                 (0x00000400U+((FIFO)*0x1000U)+((PEEKHEADDATA)*0x4U))
#define CSL_RINGACC_FIFOS_FIFO_PEEKTAILDATA(FIFO,PEEKTAILDATA)                 (0x00000600U+((FIFO)*0x1000U)+((PEEKTAILDATA)*0x4U))

/**************************************************************************
* 64K Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t RINGHEADDATA[128];         /* Ring Head Entry Data Registers */
    volatile uint32_t RINGTAILDATA[128];         /* Ring Tail Entry Data Registers */
    volatile uint32_t PEEKHEADDATA[128];         /* Ring Peek Head Entry Data Registers */
    volatile uint32_t PEEKTAILDATA[128];         /* Ring Peek Tail Entry Data Registers */
    volatile uint8_t  Resv_65536[63488];
} CSL_a64_ringacc_fifosRegs_FIFO;


typedef struct {
    CSL_a64_ringacc_fifosRegs_FIFO FIFO[1024];
} CSL_a64_ringacc_fifosRegs;


/**************************************************************************
* 64K Register Macros
**************************************************************************/

#define CSL_A64_RINGACC_FIFOS_FIFO_RINGHEADDATA(FIFO,RINGHEADDATA)             (0x00000000U+((FIFO)*0x10000U)+((RINGHEADDATA)*0x4U))
#define CSL_A64_RINGACC_FIFOS_FIFO_RINGTAILDATA(FIFO,RINGTAILDATA)             (0x00000200U+((FIFO)*0x10000U)+((RINGTAILDATA)*0x4U))
#define CSL_A64_RINGACC_FIFOS_FIFO_PEEKHEADDATA(FIFO,PEEKHEADDATA)             (0x00000400U+((FIFO)*0x10000U)+((PEEKHEADDATA)*0x4U))
#define CSL_A64_RINGACC_FIFOS_FIFO_PEEKTAILDATA(FIFO,PEEKTAILDATA)             (0x00000600U+((FIFO)*0x10000U)+((PEEKTAILDATA)*0x4U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* RINGHEADDATA */

#define CSL_RINGACC_FIFOS_FIFO_RINGHEADDATA_DATA_MASK                          (0xFFFFFFFFU)
#define CSL_RINGACC_FIFOS_FIFO_RINGHEADDATA_DATA_SHIFT                         (0x00000000U)
#define CSL_RINGACC_FIFOS_FIFO_RINGHEADDATA_DATA_MAX                           (0xFFFFFFFFU)

/* RINGTAILDATA */

#define CSL_RINGACC_FIFOS_FIFO_RINGTAILDATA_DATA_MASK                          (0xFFFFFFFFU)
#define CSL_RINGACC_FIFOS_FIFO_RINGTAILDATA_DATA_SHIFT                         (0x00000000U)
#define CSL_RINGACC_FIFOS_FIFO_RINGTAILDATA_DATA_MAX                           (0xFFFFFFFFU)

/* PEEKHEADDATA */

#define CSL_RINGACC_FIFOS_FIFO_PEEKHEADDATA_DATA_MASK                          (0xFFFFFFFFU)
#define CSL_RINGACC_FIFOS_FIFO_PEEKHEADDATA_DATA_SHIFT                         (0x00000000U)
#define CSL_RINGACC_FIFOS_FIFO_PEEKHEADDATA_DATA_MAX                           (0xFFFFFFFFU)

/* PEEKTAILDATA */

#define CSL_RINGACC_FIFOS_FIFO_PEEKTAILDATA_DATA_MASK                          (0xFFFFFFFFU)
#define CSL_RINGACC_FIFOS_FIFO_PEEKTAILDATA_DATA_SHIFT                         (0x00000000U)
#define CSL_RINGACC_FIFOS_FIFO_PEEKTAILDATA_DATA_MAX                           (0xFFFFFFFFU)

#ifdef __cplusplus
}
#endif
#endif
