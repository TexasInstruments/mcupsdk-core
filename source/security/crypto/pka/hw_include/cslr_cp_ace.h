/********************************************************************
 * Copyright (C) 2017-2022 Texas Instruments Incorporated.
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
 *  Name        : cslr_cp_ace.h
*/
#ifndef CSLR_CP_ACE_H_
#define CSLR_CP_ACE_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <drivers/hw_include/cslr.h>
#include <stdint.h>
#include <security/crypto/pka/hw_include/cslr_eip_29t2_ram.h>

/**************************************************************************
* Module Base Offset Values
**************************************************************************/

#define CSL_CP_ACE_ECC_AGGR_REGS_BASE                                           (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_REGS_BASE                                        (0x00020000U)
#define CSL_CP_ACE_REGS_BASE                                                    (0x00000000U)
#define CSL_CP_ACE_RNG_REGS_BASE                                                (0x00010000U)
#define CSL_CP_ACE_UPDATES_REGS_BASE                                            (0x00001000U)


/**************************************************************************
* Hardware Region  : Global Control Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PID;                  /* Version and Identification Register */
    volatile uint32_t EFUSE_EN;              /* Efuse Enable */
    volatile uint32_t CMD_STATUS;             /* Engine Status Readable Register */
    volatile uint8_t  Resv_20[8];
    volatile uint32_t CDMA_FLOWID;               /* CPPI FlowID */
    volatile uint8_t  Resv_28[4];
    volatile uint32_t CDMA_ENG_ID;            /* Default engine id */
    volatile uint8_t  Resv_256[224];
    volatile uint32_t CTXCACH_CTRL;              /* context cache control */
    volatile uint8_t  Resv_264[4];
    volatile uint32_t CTXCACH_SC_ID;              /* context cache SCID */
    volatile uint32_t CTXCACH_MISSCNT;           /* Cache miss counter */
} CSL_Cp_aceMmrRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_CP_ACE_PID                                              (0x00000000U)
#define CSL_CP_ACE_EFUSE_EN                                         (0x00000004U)
#define CSL_CP_ACE_CMD_STATUS                                       (0x00000008U)
#define CSL_CP_ACE_CDMA_FLOWID                                      (0x00000014U)
#define CSL_CP_ACE_CDMA_ENG_ID                                      (0x0000001CU)
#define CSL_CP_ACE_CTXCACH_CTRL                                     (0x00000100U)
#define CSL_CP_ACE_CTXCACH_SC_ID                                    (0x00000108U)
#define CSL_CP_ACE_CTXCACH_MISSCNT                                   (0x0000010CU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_CP_ACE_PID_MINOR_MASK                                 (0x0000003FU)
#define CSL_CP_ACE_PID_MINOR_SHIFT                                (0x00000000U)
#define CSL_CP_ACE_PID_MINOR_MAX                                  (0x0000003FU)

#define CSL_CP_ACE_PID_CUSTOM_MASK                                 (0x000000C0U)
#define CSL_CP_ACE_PID_CUSTOM_SHIFT                                (0x00000006U)
#define CSL_CP_ACE_PID_CUSTOM_MAX                                  (0x00000003U)

#define CSL_CP_ACE_PID_MAJOR_MASK                                 (0x00000700U)
#define CSL_CP_ACE_PID_MAJOR_SHIFT                                (0x00000008U)
#define CSL_CP_ACE_PID_MAJOR_MAX                                  (0x00000007U)

#define CSL_CP_ACE_PID_RTL_MASK                                 (0x0000F800U)
#define CSL_CP_ACE_PID_RTL_SHIFT                                (0x0000000BU)
#define CSL_CP_ACE_PID_RTL_MAX                                  (0x0000001FU)

#define CSL_CP_ACE_PID_FUNC_MASK                                  (0x0FFF0000U)
#define CSL_CP_ACE_PID_FUNC_SHIFT                                 (0x00000010U)
#define CSL_CP_ACE_PID_FUNC_MAX                                   (0x00000FFFU)

#define CSL_CP_ACE_PID_BU_MASK                                     (0x30000000U)
#define CSL_CP_ACE_PID_BU_SHIFT                                    (0x0000001CU)
#define CSL_CP_ACE_PID_BU_MAX                                      (0x00000003U)

#define CSL_CP_ACE_PID_SCHEME_MASK                                 (0xC0000000U)
#define CSL_CP_ACE_PID_SCHEME_SHIFT                                (0x0000001EU)
#define CSL_CP_ACE_PID_SCHEME_MAX                                  (0x00000003U)

/* EFUSE_EN */

#define CSL_CP_ACE_EFUSE_EN_ENABLE_MASK                           (0x00000007U)
#define CSL_CP_ACE_EFUSE_EN_ENABLE_SHIFT                          (0x00000000U)
#define CSL_CP_ACE_EFUSE_EN_ENABLE_MAX                            (0x00000007U)

/* CMD_STATUS */

#define CSL_CP_ACE_CMD_STATUS_ENCSS_EN_MASK                          (0x00000001U)
#define CSL_CP_ACE_CMD_STATUS_ENCSS_EN_SHIFT                         (0x00000000U)
#define CSL_CP_ACE_CMD_STATUS_ENCSS_EN_MAX                           (0x00000001U)

#define CSL_CP_ACE_CMD_STATUS_AUTHSS_EN_MASK                         (0x00000002U)
#define CSL_CP_ACE_CMD_STATUS_AUTHSS_EN_SHIFT                        (0x00000001U)
#define CSL_CP_ACE_CMD_STATUS_AUTHSS_EN_MAX                          (0x00000001U)

#define CSL_CP_ACE_CMD_STATUS_TRNG_EN_MASK                           (0x00000008U)
#define CSL_CP_ACE_CMD_STATUS_TRNG_EN_SHIFT                          (0x00000003U)
#define CSL_CP_ACE_CMD_STATUS_TRNG_EN_MAX                            (0x00000001U)

#define CSL_CP_ACE_CMD_STATUS_PKA_EN_MASK                            (0x00000010U)
#define CSL_CP_ACE_CMD_STATUS_PKA_EN_SHIFT                           (0x00000004U)
#define CSL_CP_ACE_CMD_STATUS_PKA_EN_MAX                             (0x00000001U)

#define CSL_CP_ACE_CMD_STATUS_CTXCACH_EN_MASK                        (0x00000080U)
#define CSL_CP_ACE_CMD_STATUS_CTXCACH_EN_SHIFT                       (0x00000007U)
#define CSL_CP_ACE_CMD_STATUS_CTXCACH_EN_MAX                         (0x00000001U)

#define CSL_CP_ACE_CMD_STATUS_CDMA_IN_PORT_EN_MASK                   (0x00000200U)
#define CSL_CP_ACE_CMD_STATUS_CDMA_IN_PORT_EN_SHIFT                  (0x00000009U)
#define CSL_CP_ACE_CMD_STATUS_CDMA_IN_PORT_EN_MAX                    (0x00000001U)

#define CSL_CP_ACE_CMD_STATUS_CDMA_OUT_PORT_EN_MASK                  (0x00000800U)
#define CSL_CP_ACE_CMD_STATUS_CDMA_OUT_PORT_EN_SHIFT                 (0x0000000BU)
#define CSL_CP_ACE_CMD_STATUS_CDMA_OUT_PORT_EN_MAX                   (0x00000001U)

#define CSL_CP_ACE_CMD_STATUS_ENCSS_BUSY_MASK                        (0x00010000U)
#define CSL_CP_ACE_CMD_STATUS_ENCSS_BUSY_SHIFT                       (0x00000010U)
#define CSL_CP_ACE_CMD_STATUS_ENCSS_BUSY_MAX                         (0x00000001U)

#define CSL_CP_ACE_CMD_STATUS_AUTHSS_BUSY_MASK                       (0x00020000U)
#define CSL_CP_ACE_CMD_STATUS_AUTHSS_BUSY_SHIFT                      (0x00000011U)
#define CSL_CP_ACE_CMD_STATUS_AUTHSS_BUSY_MAX                        (0x00000001U)

#define CSL_CP_ACE_CMD_STATUS_TRNG_BUSY_MASK                         (0x00080000U)
#define CSL_CP_ACE_CMD_STATUS_TRNG_BUSY_SHIFT                        (0x00000013U)
#define CSL_CP_ACE_CMD_STATUS_TRNG_BUSY_MAX                          (0x00000001U)

#define CSL_CP_ACE_CMD_STATUS_PKA_BUSY_MASK                          (0x00100000U)
#define CSL_CP_ACE_CMD_STATUS_PKA_BUSY_SHIFT                         (0x00000014U)
#define CSL_CP_ACE_CMD_STATUS_PKA_BUSY_MAX                           (0x00000001U)

#define CSL_CP_ACE_CMD_STATUS_CTXCACH_BUSY_MASK                      (0x00800000U)
#define CSL_CP_ACE_CMD_STATUS_CTXCACH_BUSY_SHIFT                     (0x00000017U)
#define CSL_CP_ACE_CMD_STATUS_CTXCACH_BUSY_MAX                       (0x00000001U)

#define CSL_CP_ACE_CMD_STATUS_CDMA_IN_PORT_BUSY_MASK                 (0x02000000U)
#define CSL_CP_ACE_CMD_STATUS_CDMA_IN_PORT_BUSY_SHIFT                (0x00000019U)
#define CSL_CP_ACE_CMD_STATUS_CDMA_IN_PORT_BUSY_MAX                  (0x00000001U)

#define CSL_CP_ACE_CMD_STATUS_CDMA_OUT_PORT_BUSY_MASK                (0x08000000U)
#define CSL_CP_ACE_CMD_STATUS_CDMA_OUT_PORT_BUSY_SHIFT               (0x0000001BU)
#define CSL_CP_ACE_CMD_STATUS_CDMA_OUT_PORT_BUSY_MAX                 (0x00000001U)

/* CDMA_FLOWID */

#define CSL_CP_ACE_CDMA_FLOWID_CDMA_FLOWID_MASK                         (0x000000FFU)
#define CSL_CP_ACE_CDMA_FLOWID_CDMA_FLOWID_SHIFT                        (0x00000000U)
#define CSL_CP_ACE_CDMA_FLOWID_CDMA_FLOWID_MAX                          (0x000000FFU)

/* CDMA_ENG_ID */

#define CSL_CP_ACE_CDMA_ENG_ID_CDMA_ENG_ID_MASK                   (0x0000001FU)
#define CSL_CP_ACE_CDMA_ENG_ID_CDMA_ENG_ID_SHIFT                  (0x00000000U)
#define CSL_CP_ACE_CDMA_ENG_ID_CDMA_ENG_ID_MAX                    (0x0000001FU)

/* CTXCACH_CTRL */

#define CSL_CP_ACE_CTXCACH_CTRL_AUTO_FETCH_EN_MASK                      (0x00000001U)
#define CSL_CP_ACE_CTXCACH_CTRL_AUTO_FETCH_EN_SHIFT                     (0x00000000U)
#define CSL_CP_ACE_CTXCACH_CTRL_AUTO_FETCH_EN_MAX                       (0x00000001U)

#define CSL_CP_ACE_CTXCACH_CTRL_CLR_CACHE_TABLE_MASK                    (0x00000002U)
#define CSL_CP_ACE_CTXCACH_CTRL_CLR_CACHE_TABLE_SHIFT                   (0x00000001U)
#define CSL_CP_ACE_CTXCACH_CTRL_CLR_CACHE_TABLE_MAX                     (0x00000001U)

#define CSL_CP_ACE_CTXCACH_CTRL_CDMA_PORT_EN_MASK                       (0x00000008U)
#define CSL_CP_ACE_CTXCACH_CTRL_CDMA_PORT_EN_SHIFT                      (0x00000003U)
#define CSL_CP_ACE_CTXCACH_CTRL_CDMA_PORT_EN_MAX                        (0x00000001U)

#define CSL_CP_ACE_CTXCACH_CTRL_CLR_STATS_MASK                          (0x00000010U)
#define CSL_CP_ACE_CTXCACH_CTRL_CLR_STATS_SHIFT                         (0x00000004U)
#define CSL_CP_ACE_CTXCACH_CTRL_CLR_STATS_MAX                           (0x00000001U)

#define CSL_CP_ACE_CTXCACH_CTRL_CTX_CNT_MASK                            (0x7F000000U)
#define CSL_CP_ACE_CTXCACH_CTRL_CTX_CNT_SHIFT                           (0x00000018U)
#define CSL_CP_ACE_CTXCACH_CTRL_CTX_CNT_MAX                             (0x0000007FU)

#define CSL_CP_ACE_CTXCACH_CTRL_BUSY_MASK                           (0x80000000U)
#define CSL_CP_ACE_CTXCACH_CTRL_BUSY_SHIFT                          (0x0000001FU)
#define CSL_CP_ACE_CTXCACH_CTRL_BUSY_MAX                            (0x00000001U)

/* CTXCACH_SC_ID */

#define CSL_CP_ACE_CTXCACH_SC_ID_SC_ID_MASK                               (0x0000FFFFU)
#define CSL_CP_ACE_CTXCACH_SC_ID_SC_ID_SHIFT                              (0x00000000U)
#define CSL_CP_ACE_CTXCACH_SC_ID_SC_ID_MAX                                (0x0000FFFFU)

#define CSL_CP_ACE_CTXCACH_SC_ID_SC_FETCH_EVICT_MASK                     (0x00010000U)
#define CSL_CP_ACE_CTXCACH_SC_ID_SC_FETCH_EVICT_SHIFT                    (0x00000010U)
#define CSL_CP_ACE_CTXCACH_SC_ID_SC_FETCH_EVICT_MAX                      (0x00000001U)

#define CSL_CP_ACE_CTXCACH_SC_ID_SC_TEAR_MASK                            (0x00020000U)
#define CSL_CP_ACE_CTXCACH_SC_ID_SC_TEAR_SHIFT                           (0x00000011U)
#define CSL_CP_ACE_CTXCACH_SC_ID_SC_TEAR_MAX                             (0x00000001U)

#define CSL_CP_ACE_CTXCACH_SC_ID_GO_MASK                                 (0x00080000U)
#define CSL_CP_ACE_CTXCACH_SC_ID_GO_SHIFT                                (0x00000013U)
#define CSL_CP_ACE_CTXCACH_SC_ID_GO_MAX                                  (0x00000001U)

#define CSL_CP_ACE_CTXCACH_SC_ID_SC_RAMIDX_MASK                          (0x0FF00000U)
#define CSL_CP_ACE_CTXCACH_SC_ID_SC_RAMIDX_SHIFT                         (0x00000014U)
#define CSL_CP_ACE_CTXCACH_SC_ID_SC_RAMIDX_MAX                           (0x000000FFU)

#define CSL_CP_ACE_CTXCACH_SC_ID_SC_ERRCODE_MASK                         (0x70000000U)
#define CSL_CP_ACE_CTXCACH_SC_ID_SC_ERRCODE_SHIFT                        (0x0000001CU)
#define CSL_CP_ACE_CTXCACH_SC_ID_SC_ERRCODE_MAX                          (0x00000007U)

#define CSL_CP_ACE_CTXCACH_SC_ID_DONE_MASK                            (0x80000000U)
#define CSL_CP_ACE_CTXCACH_SC_ID_DONE_SHIFT                           (0x0000001FU)
#define CSL_CP_ACE_CTXCACH_SC_ID_DONE_MAX                             (0x00000001U)

/* CTXCACH_MISSCNT */

#define CSL_CP_ACE_CTXCACH_MISSCNT_CTX_MISSCNT_MASK                     (0xFFFFFFFFU)
#define CSL_CP_ACE_CTXCACH_MISSCNT_CTX_MISSCNT_SHIFT                    (0x00000000U)
#define CSL_CP_ACE_CTXCACH_MISSCNT_CTX_MISSCNT_MAX                      (0xFFFFFFFFU)

/**************************************************************************
* Hardware Region  : MMR region that is protected by firewall at the SoC CBASS
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t ENGINE_ENABLE;             /* Engine Enable */
    volatile uint8_t  Resv_16[12];
    volatile uint32_t SCPTR_PROMOTE_LOW_RANGE_L;   /* SCPTR Promote Lower Limit */
    volatile uint32_t SCPTR_PROMOTE_LOW_RANGE_H;   /* SCPTR Promote Lower Limit */
    volatile uint32_t SCPTR_PROMOTE_HI_RANGE_L;   /* SCPTR Promote Upper Limit */
    volatile uint32_t SCPTR_PROMOTE_HI_RANGE_H;   /* SCPTR Promote Upper Limit */
    volatile uint32_t EXCEPTION_LOGGING_CONTROL;   /* Exception Logging Control Register */
    volatile uint32_t EXCEPTION_LOGGING_HEADER0;   /* Exception Logging Header 0 Register */
    volatile uint32_t EXCEPTION_LOGGING_HEADER1;   /* Exception Logging Header 1 Register */
    volatile uint32_t EXCEPTION_LOGGING_DATA0;   /* Exception Logging Data 0 Register */
    volatile uint32_t EXCEPTION_LOGGING_DATA1;   /* Exception Logging Data 1 Register */
    volatile uint32_t EXCEPTION_LOGGING_DATA2;   /* Exception Logging Data 2 Register */
    volatile uint32_t EXCEPTION_LOGGING_DATA3;   /* Exception Logging Data 3 Register */
    volatile uint8_t  Resv_64[4];
    volatile uint32_t EXCEPTION_PEND_SET;        /* Exception Logging Pending Set Register */
    volatile uint32_t EXCEPTION_PEND_CLEAR;      /* Exception Logging Pending Clear Register */
    volatile uint8_t  Resv_80[8];
    volatile uint32_t TRNG_INTR_SET;             /* TRNG Interrupt Set Register */
    volatile uint32_t TRNG_INTR_CLEAR;           /* TRNG Interrupt Clear Register */
    volatile uint8_t  Resv_96[8];
    volatile uint32_t PKA_INTR_SET;              /* PKA Interrupt Set Register */
    volatile uint32_t PKA_INTR_CLEAR;            /* PKA Interrupt Clear Register */
    volatile uint8_t  Resv_256[152];
    volatile uint32_t KEK[8];
    volatile uint32_t KEK_LOCK;
    volatile uint32_t DKEK_PRIVID;
} CSL_Cp_aceUpdatesRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_CP_ACE_UPDATES_ENGINE_ENABLE                                        (0x00000000U)
#define CSL_CP_ACE_UPDATES_SCPTR_PROMOTE_LOW_RANGE_L                            (0x00000010U)
#define CSL_CP_ACE_UPDATES_SCPTR_PROMOTE_LOW_RANGE_H                            (0x00000014U)
#define CSL_CP_ACE_UPDATES_SCPTR_PROMOTE_HI_RANGE_L                             (0x00000018U)
#define CSL_CP_ACE_UPDATES_SCPTR_PROMOTE_HI_RANGE_H                             (0x0000001CU)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_CONTROL                            (0x00000020U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_HEADER0                            (0x00000024U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_HEADER1                            (0x00000028U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA0                              (0x0000002CU)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA1                              (0x00000030U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA2                              (0x00000034U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA3                              (0x00000038U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_PEND_SET                                   (0x00000040U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_PEND_CLEAR                                 (0x00000044U)
#define CSL_CP_ACE_UPDATES_TRNG_INTR_SET                                        (0x00000050U)
#define CSL_CP_ACE_UPDATES_TRNG_INTR_CLEAR                                      (0x00000054U)
#define CSL_CP_ACE_UPDATES_PKA_INTR_SET                                         (0x00000060U)
#define CSL_CP_ACE_UPDATES_PKA_INTR_CLEAR                                       (0x00000064U)
#define CSL_CP_ACE_UPDATES_KEK(N)                                   (0x00000100U+((N)*0x4U))
#define CSL_CP_ACE_UPDATES_KEK_LOCK                                             (0x00000120U)
#define CSL_CP_ACE_UPDATES_DKEK_PRIVID                                          (0x00000124U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* ENGINE_ENABLE */

#define CSL_CP_ACE_UPDATES_ENGINE_ENABLE_ENCSS_EN_MASK                          (0x00000001U)
#define CSL_CP_ACE_UPDATES_ENGINE_ENABLE_ENCSS_EN_SHIFT                         (0x00000000U)
#define CSL_CP_ACE_UPDATES_ENGINE_ENABLE_ENCSS_EN_MAX                           (0x00000001U)

#define CSL_CP_ACE_UPDATES_ENGINE_ENABLE_AUTHSS_EN_MASK                         (0x00000002U)
#define CSL_CP_ACE_UPDATES_ENGINE_ENABLE_AUTHSS_EN_SHIFT                        (0x00000001U)
#define CSL_CP_ACE_UPDATES_ENGINE_ENABLE_AUTHSS_EN_MAX                          (0x00000001U)

#define CSL_CP_ACE_UPDATES_ENGINE_ENABLE_TRNG_EN_MASK                           (0x00000008U)
#define CSL_CP_ACE_UPDATES_ENGINE_ENABLE_TRNG_EN_SHIFT                          (0x00000003U)
#define CSL_CP_ACE_UPDATES_ENGINE_ENABLE_TRNG_EN_MAX                            (0x00000001U)

#define CSL_CP_ACE_UPDATES_ENGINE_ENABLE_PKA_EN_MASK                            (0x00000010U)
#define CSL_CP_ACE_UPDATES_ENGINE_ENABLE_PKA_EN_SHIFT                           (0x00000004U)
#define CSL_CP_ACE_UPDATES_ENGINE_ENABLE_PKA_EN_MAX                             (0x00000001U)

#define CSL_CP_ACE_UPDATES_ENGINE_ENABLE_CTX_EN_MASK                            (0x00000080U)
#define CSL_CP_ACE_UPDATES_ENGINE_ENABLE_CTX_EN_SHIFT                           (0x00000007U)
#define CSL_CP_ACE_UPDATES_ENGINE_ENABLE_CTX_EN_MAX                             (0x00000001U)

#define CSL_CP_ACE_UPDATES_ENGINE_ENABLE_CDMA_IN_EN_MASK                        (0x00000200U)
#define CSL_CP_ACE_UPDATES_ENGINE_ENABLE_CDMA_IN_EN_SHIFT                       (0x00000009U)
#define CSL_CP_ACE_UPDATES_ENGINE_ENABLE_CDMA_IN_EN_MAX                         (0x00000001U)

#define CSL_CP_ACE_UPDATES_ENGINE_ENABLE_CDMA_OUT_EN_MASK                       (0x00000800U)
#define CSL_CP_ACE_UPDATES_ENGINE_ENABLE_CDMA_OUT_EN_SHIFT                      (0x0000000BU)
#define CSL_CP_ACE_UPDATES_ENGINE_ENABLE_CDMA_OUT_EN_MAX                        (0x00000001U)

/* SCPTR_PROMOTE_LOW_RANGE_L */

#define CSL_CP_ACE_UPDATES_SCPTR_PROMOTE_LOW_RANGE_L_BIT_31_0_MASK              (0xFFFFFFFFU)
#define CSL_CP_ACE_UPDATES_SCPTR_PROMOTE_LOW_RANGE_L_BIT_31_0_SHIFT             (0x00000000U)
#define CSL_CP_ACE_UPDATES_SCPTR_PROMOTE_LOW_RANGE_L_BIT_31_0_MAX               (0xFFFFFFFFU)

/* SCPTR_PROMOTE_LOW_RANGE_H */

#define CSL_CP_ACE_UPDATES_SCPTR_PROMOTE_LOW_RANGE_H_BIT_47_32_MASK             (0x0000FFFFU)
#define CSL_CP_ACE_UPDATES_SCPTR_PROMOTE_LOW_RANGE_H_BIT_47_32_SHIFT            (0x00000000U)
#define CSL_CP_ACE_UPDATES_SCPTR_PROMOTE_LOW_RANGE_H_BIT_47_32_MAX              (0x0000FFFFU)

/* SCPTR_PROMOTE_HI_RANGE_L */

#define CSL_CP_ACE_UPDATES_SCPTR_PROMOTE_HI_RANGE_L_BIT_31_0_MASK               (0xFFFFFFFFU)
#define CSL_CP_ACE_UPDATES_SCPTR_PROMOTE_HI_RANGE_L_BIT_31_0_SHIFT              (0x00000000U)
#define CSL_CP_ACE_UPDATES_SCPTR_PROMOTE_HI_RANGE_L_BIT_31_0_MAX                (0xFFFFFFFFU)

/* SCPTR_PROMOTE_HI_RANGE_H */

#define CSL_CP_ACE_UPDATES_SCPTR_PROMOTE_HI_RANGE_H_BIT_47_32_MASK              (0x0000FFFFU)
#define CSL_CP_ACE_UPDATES_SCPTR_PROMOTE_HI_RANGE_H_BIT_47_32_SHIFT             (0x00000000U)
#define CSL_CP_ACE_UPDATES_SCPTR_PROMOTE_HI_RANGE_H_BIT_47_32_MAX               (0x0000FFFFU)

/* EXCEPTION_LOGGING_CONTROL */

#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_CONTROL_DISABLE_LOG_MASK           (0x00000001U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_CONTROL_DISABLE_LOG_SHIFT          (0x00000000U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_CONTROL_DISABLE_LOG_MAX            (0x00000001U)

#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_CONTROL_DISABLE_PEND_MASK          (0x00000002U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_CONTROL_DISABLE_PEND_SHIFT         (0x00000001U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_CONTROL_DISABLE_PEND_MAX           (0x00000001U)

/* EXCEPTION_LOGGING_HEADER0 */

#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_HEADER0_SRC_ID_MASK                (0x00FFFF00U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_HEADER0_SRC_ID_SHIFT               (0x00000008U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_HEADER0_SRC_ID_MAX                 (0x0000FFFFU)

#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_HEADER0_TYPE_LOG_MASK              (0xFF000000U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_HEADER0_TYPE_LOG_SHIFT             (0x00000018U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_HEADER0_TYPE_LOG_MAX               (0x000000FFU)

/* EXCEPTION_LOGGING_HEADER1 */

#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_HEADER1_CODE_MASK                  (0x00FF0000U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_HEADER1_CODE_SHIFT                 (0x00000010U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_HEADER1_CODE_MAX                   (0x000000FFU)

#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_HEADER1_GROUP_MASK                 (0xFF000000U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_HEADER1_GROUP_SHIFT                (0x00000018U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_HEADER1_GROUP_MAX                  (0x000000FFU)

/* EXCEPTION_LOGGING_DATA0 */

#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA0_ADDR_L_MASK                  (0xFFFFFFFFU)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA0_ADDR_L_SHIFT                 (0x00000000U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA0_ADDR_L_MAX                   (0xFFFFFFFFU)

/* EXCEPTION_LOGGING_DATA1 */

#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA1_ADDR_H_MASK                  (0x0000FFFFU)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA1_ADDR_H_SHIFT                 (0x00000000U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA1_ADDR_H_MAX                   (0x0000FFFFU)

/* EXCEPTION_LOGGING_DATA2 */

#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA2_SECURE_MASK                  (0x00000001U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA2_SECURE_SHIFT                 (0x00000000U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA2_SECURE_MAX                   (0x00000001U)

#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA2_PROMOTE_MASK                 (0x00000020U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA2_PROMOTE_SHIFT                (0x00000005U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA2_PROMOTE_MAX                  (0x00000001U)

#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA2_DEMOTE_MASK                  (0x00000040U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA2_DEMOTE_SHIFT                 (0x00000006U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA2_DEMOTE_MAX                   (0x00000001U)

#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA2_ALLOWNS_MASK                 (0x00000080U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA2_ALLOWNS_SHIFT                (0x00000007U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA2_ALLOWNS_MAX                  (0x00000001U)

#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA2_PRIV_MASK                    (0x00000300U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA2_PRIV_SHIFT                   (0x00000008U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA2_PRIV_MAX                     (0x00000003U)

#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA2_PRIV_ID_MASK                 (0x00FF0000U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA2_PRIV_ID_SHIFT                (0x00000010U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA2_PRIV_ID_MAX                  (0x000000FFU)

/* EXCEPTION_LOGGING_DATA3 */

#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA3_SECURE_MASK                  (0x00000001U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA3_SECURE_SHIFT                 (0x00000000U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA3_SECURE_MAX                   (0x00000001U)

#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA3_PROMOTE_MASK                 (0x00000020U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA3_PROMOTE_SHIFT                (0x00000005U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA3_PROMOTE_MAX                  (0x00000001U)

#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA3_DEMOTE_MASK                  (0x00000040U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA3_DEMOTE_SHIFT                 (0x00000006U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA3_DEMOTE_MAX                   (0x00000001U)

#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA3_ALLOWNS_MASK                 (0x00000080U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA3_ALLOWNS_SHIFT                (0x00000007U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA3_ALLOWNS_MAX                  (0x00000001U)

#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA3_PRIV_MASK                    (0x00000300U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA3_PRIV_SHIFT                   (0x00000008U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA3_PRIV_MAX                     (0x00000003U)

#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA3_PRIV_ID_MASK                 (0x00FF0000U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA3_PRIV_ID_SHIFT                (0x00000010U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_LOGGING_DATA3_PRIV_ID_MAX                  (0x000000FFU)

/* EXCEPTION_PEND_SET */

#define CSL_CP_ACE_UPDATES_EXCEPTION_PEND_SET_PEND_SET_MASK                     (0x00000001U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_PEND_SET_PEND_SET_SHIFT                    (0x00000000U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_PEND_SET_PEND_SET_MAX                      (0x00000001U)

/* EXCEPTION_PEND_CLEAR */

#define CSL_CP_ACE_UPDATES_EXCEPTION_PEND_CLEAR_PEND_CLR_MASK                   (0x00000001U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_PEND_CLEAR_PEND_CLR_SHIFT                  (0x00000000U)
#define CSL_CP_ACE_UPDATES_EXCEPTION_PEND_CLEAR_PEND_CLR_MAX                    (0x00000001U)

/* TRNG_INTR_SET */

#define CSL_CP_ACE_UPDATES_TRNG_INTR_SET_TRNG_SET_MASK                          (0x00000001U)
#define CSL_CP_ACE_UPDATES_TRNG_INTR_SET_TRNG_SET_SHIFT                         (0x00000000U)
#define CSL_CP_ACE_UPDATES_TRNG_INTR_SET_TRNG_SET_MAX                           (0x00000001U)

/* TRNG_INTR_CLEAR */

#define CSL_CP_ACE_UPDATES_TRNG_INTR_CLEAR_TRNG_CLR_MASK                        (0x00000001U)
#define CSL_CP_ACE_UPDATES_TRNG_INTR_CLEAR_TRNG_CLR_SHIFT                       (0x00000000U)
#define CSL_CP_ACE_UPDATES_TRNG_INTR_CLEAR_TRNG_CLR_MAX                         (0x00000001U)

/* PKA_INTR_SET */

#define CSL_CP_ACE_UPDATES_PKA_INTR_SET_PKA_SET_MASK                            (0x00000001U)
#define CSL_CP_ACE_UPDATES_PKA_INTR_SET_PKA_SET_SHIFT                           (0x00000000U)
#define CSL_CP_ACE_UPDATES_PKA_INTR_SET_PKA_SET_MAX                             (0x00000001U)

/* PKA_INTR_CLEAR */

#define CSL_CP_ACE_UPDATES_PKA_INTR_CLEAR_PKA_CLR_MASK                          (0x00000001U)
#define CSL_CP_ACE_UPDATES_PKA_INTR_CLEAR_PKA_CLR_SHIFT                         (0x00000000U)
#define CSL_CP_ACE_UPDATES_PKA_INTR_CLEAR_PKA_CLR_MAX                           (0x00000001U)

/* KEK */

#define CSL_CP_ACE_UPDATES_KEK_KEYS_MASK			(0xFFFFFFFFU)
#define CSL_CP_ACE_UPDATES_KEK_KEYS_SHIFT			(0x00000000U)
#define CSL_CP_ACE_UPDATES_KEK_KEYS_MAX			(0xFFFFFFFFU)

/* KEK_LOCK */

#define CSL_CP_ACE_UPDATES_KEK_LOCK_LOCK_MASK                                   (0x00000001U)
#define CSL_CP_ACE_UPDATES_KEK_LOCK_LOCK_SHIFT                                  (0x00000000U)
#define CSL_CP_ACE_UPDATES_KEK_LOCK_LOCK_MAX                                    (0x00000001U)

/* DKEK_PRIVID */

#define CSL_CP_ACE_UPDATES_DKEK_PRIVID_PRIVID3_MASK                             (0xFF000000U)
#define CSL_CP_ACE_UPDATES_DKEK_PRIVID_PRIVID3_SHIFT                            (0x00000018U)
#define CSL_CP_ACE_UPDATES_DKEK_PRIVID_PRIVID3_MAX                              (0x000000FFU)

#define CSL_CP_ACE_UPDATES_DKEK_PRIVID_PRIVID2_MASK                             (0x00FF0000U)
#define CSL_CP_ACE_UPDATES_DKEK_PRIVID_PRIVID2_SHIFT                            (0x00000010U)
#define CSL_CP_ACE_UPDATES_DKEK_PRIVID_PRIVID2_MAX                              (0x000000FFU)

#define CSL_CP_ACE_UPDATES_DKEK_PRIVID_PRIVID1_MASK                             (0x0000FF00U)
#define CSL_CP_ACE_UPDATES_DKEK_PRIVID_PRIVID1_SHIFT                            (0x00000008U)
#define CSL_CP_ACE_UPDATES_DKEK_PRIVID_PRIVID1_MAX                              (0x000000FFU)

#define CSL_CP_ACE_UPDATES_DKEK_PRIVID_PRIVID0_MASK                             (0x000000FFU)
#define CSL_CP_ACE_UPDATES_DKEK_PRIVID_PRIVID0_SHIFT                            (0x00000000U)
#define CSL_CP_ACE_UPDATES_DKEK_PRIVID_PRIVID0_MAX                              (0x000000FFU)

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t TRNG_INPUT_0;
    volatile uint32_t TRNG_INPUT_1;
    volatile uint32_t TRNG_INPUT_2;
    volatile uint32_t TRNG_INPUT_3;
    volatile uint32_t TRNG_STATUS;
    volatile uint32_t TRNG_CONTROL;
    volatile uint32_t TRNG_CONFIG;
    volatile uint32_t TRNG_ALARMCNT;
    volatile uint32_t TRNG_FROENABLE;
    volatile uint32_t TRNG_FRODETUNE;
    volatile uint32_t TRNG_ALARMMASK;
    volatile uint32_t TRNG_ALARMSTOP;
    volatile uint32_t TRNG_RAW_L;
    volatile uint32_t TRNG_RAW_H;
    volatile uint32_t TRNG_SPB_TESTS;
    volatile uint32_t TRNG_COUNT;
    volatile uint32_t TRNG_PS_AI[12];
    volatile uint32_t TRNG_TEST;
    volatile uint32_t TRNG_BLOCKCNT;
    volatile uint32_t TRNG_OPTIONS;
    volatile uint32_t TRNG_EIP_REV;
} CSL_Cp_aceTrngRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_CP_ACE_TRNG_INPUT_0                                             (0x00000000U)
#define CSL_CP_ACE_TRNG_INPUT_1                                             (0x00000004U)
#define CSL_CP_ACE_TRNG_INPUT_2                                             (0x00000008U)
#define CSL_CP_ACE_TRNG_INPUT_3                                             (0x0000000CU)
#define CSL_CP_ACE_TRNG_STATUS                                              (0x00000010U)
#define CSL_CP_ACE_TRNG_INTACK_SECURE_MODE                                  (0x00000010U)
#define CSL_CP_ACE_TRNG_INTACK                                              (0x00000010U)
#define CSL_CP_ACE_TRNG_CONTROL                                             (0x00000014U)
#define CSL_CP_ACE_TRNG_CONFIG                                              (0x00000018U)
#define CSL_CP_ACE_TRNG_ALARMCNT                                            (0x0000001CU)
#define CSL_CP_ACE_TRNG_FROENABLE                                           (0x00000020U)
#define CSL_CP_ACE_TRNG_FRODETUNE                                           (0x00000024U)
#define CSL_CP_ACE_TRNG_ALARMMASK                                           (0x00000028U)
#define CSL_CP_ACE_TRNG_ALARMSTOP                                           (0x0000002CU)
#define CSL_CP_ACE_TRNG_RAW_L                                               (0x00000030U)
#define CSL_CP_ACE_TRNG_RAW_H                                               (0x00000034U)
#define CSL_CP_ACE_TRNG_SPB_TESTS                                           (0x00000038U)
#define CSL_CP_ACE_TRNG_COUNT                                               (0x0000003CU)
#define CSL_CP_ACE_TRNG_PS_AI_0                                             (0x00000040U)
#define CSL_CP_ACE_TRNG_PS_AI_1                                             (0x00000044U)
#define CSL_CP_ACE_TRNG_PS_AI_2                                             (0x00000048U)
#define CSL_CP_ACE_TRNG_PS_AI_3                                             (0x0000004CU)
#define CSL_CP_ACE_TRNG_PS_AI_4                                             (0x00000050U)
#define CSL_CP_ACE_TRNG_PS_AI_5                                             (0x00000054U)
#define CSL_CP_ACE_TRNG_PS_AI_6                                             (0x00000058U)
#define CSL_CP_ACE_TRNG_PS_AI_7                                             (0x0000005CU)
#define CSL_CP_ACE_TRNG_PS_AI_8                                             (0x00000060U)
#define CSL_CP_ACE_TRNG_PS_AI_9                                             (0x00000064U)
#define CSL_CP_ACE_TRNG_PS_AI_10                                            (0x00000068U)
#define CSL_CP_ACE_TRNG_PS_AI_11                                            (0x0000006CU)
#define CSL_CP_ACE_TRNG_TEST                                                (0x00000070U)
#define CSL_CP_ACE_TRNG_BLOCKCNT                                            (0x00000074U)
#define CSL_CP_ACE_TRNG_OPTIONS                                             (0x00000078U)
#define CSL_CP_ACE_TRNG_EIP_REV                                             (0x0000007CU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* TRNG_INPUT_0 */

#define CSL_CP_ACE_TRNG_INPUT_0_TRNG_INPUT_0_MASK                           (0xFFFFFFFFU)
#define CSL_CP_ACE_TRNG_INPUT_0_TRNG_INPUT_0_SHIFT                          (0x00000000U)
#define CSL_CP_ACE_TRNG_INPUT_0_TRNG_INPUT_0_MAX                            (0xFFFFFFFFU)

/* TRNG_INPUT_1 */

#define CSL_CP_ACE_TRNG_INPUT_1_TRNG_INPUT_1_MASK                           (0xFFFFFFFFU)
#define CSL_CP_ACE_TRNG_INPUT_1_TRNG_INPUT_1_SHIFT                          (0x00000000U)
#define CSL_CP_ACE_TRNG_INPUT_1_TRNG_INPUT_1_MAX                            (0xFFFFFFFFU)

/* TRNG_INPUT_2 */

#define CSL_CP_ACE_TRNG_INPUT_2_TRNG_INPUT_2_MASK                           (0xFFFFFFFFU)
#define CSL_CP_ACE_TRNG_INPUT_2_TRNG_INPUT_2_SHIFT                          (0x00000000U)
#define CSL_CP_ACE_TRNG_INPUT_2_TRNG_INPUT_2_MAX                            (0xFFFFFFFFU)

/* TRNG_INPUT_3 */

#define CSL_CP_ACE_TRNG_INPUT_3_TRNG_INPUT_3_MASK                           (0xFFFFFFFFU)
#define CSL_CP_ACE_TRNG_INPUT_3_TRNG_INPUT_3_SHIFT                          (0x00000000U)
#define CSL_CP_ACE_TRNG_INPUT_3_TRNG_INPUT_3_MAX                            (0xFFFFFFFFU)

/* TRNG_STATUS */

#define CSL_CP_ACE_TRNG_STATUS_READY_MASK                                   (0x00000001U)
#define CSL_CP_ACE_TRNG_STATUS_READY_SHIFT                                  (0x00000000U)
#define CSL_CP_ACE_TRNG_STATUS_READY_MAX                                    (0x00000001U)

#define CSL_CP_ACE_TRNG_STATUS_SHUTDOWN_OFLO_MASK                           (0x00000002U)
#define CSL_CP_ACE_TRNG_STATUS_SHUTDOWN_OFLO_SHIFT                          (0x00000001U)
#define CSL_CP_ACE_TRNG_STATUS_SHUTDOWN_OFLO_MAX                            (0x00000001U)

#define CSL_CP_ACE_TRNG_STATUS_STUCK_OUT_MASK                               (0x00000004U)
#define CSL_CP_ACE_TRNG_STATUS_STUCK_OUT_SHIFT                              (0x00000002U)
#define CSL_CP_ACE_TRNG_STATUS_STUCK_OUT_MAX                                (0x00000001U)

#define CSL_CP_ACE_TRNG_STATUS_NOISE_FAIL_MASK                              (0x00000008U)
#define CSL_CP_ACE_TRNG_STATUS_NOISE_FAIL_SHIFT                             (0x00000003U)
#define CSL_CP_ACE_TRNG_STATUS_NOISE_FAIL_MAX                               (0x00000001U)

#define CSL_CP_ACE_TRNG_STATUS_RUN_FAIL_MASK                                (0x00000010U)
#define CSL_CP_ACE_TRNG_STATUS_RUN_FAIL_SHIFT                               (0x00000004U)
#define CSL_CP_ACE_TRNG_STATUS_RUN_FAIL_MAX                                 (0x00000001U)

#define CSL_CP_ACE_TRNG_STATUS_LONG_RUN_FAIL_MASK                           (0x00000020U)
#define CSL_CP_ACE_TRNG_STATUS_LONG_RUN_FAIL_SHIFT                          (0x00000005U)
#define CSL_CP_ACE_TRNG_STATUS_LONG_RUN_FAIL_MAX                            (0x00000001U)

#define CSL_CP_ACE_TRNG_STATUS_POKER_FAIL_MASK                              (0x00000040U)
#define CSL_CP_ACE_TRNG_STATUS_POKER_FAIL_SHIFT                             (0x00000006U)
#define CSL_CP_ACE_TRNG_STATUS_POKER_FAIL_MAX                               (0x00000001U)

#define CSL_CP_ACE_TRNG_STATUS_MONOBIT_FAIL_MASK                            (0x00000080U)
#define CSL_CP_ACE_TRNG_STATUS_MONOBIT_FAIL_SHIFT                           (0x00000007U)
#define CSL_CP_ACE_TRNG_STATUS_MONOBIT_FAIL_MAX                             (0x00000001U)

#define CSL_CP_ACE_TRNG_STATUS_TEST_READY_MASK                              (0x00000100U)
#define CSL_CP_ACE_TRNG_STATUS_TEST_READY_SHIFT                             (0x00000008U)
#define CSL_CP_ACE_TRNG_STATUS_TEST_READY_MAX                               (0x00000001U)

#define CSL_CP_ACE_TRNG_STATUS_STUCK_NRBG_MASK                              (0x00000200U)
#define CSL_CP_ACE_TRNG_STATUS_STUCK_NRBG_SHIFT                             (0x00000009U)
#define CSL_CP_ACE_TRNG_STATUS_STUCK_NRBG_MAX                               (0x00000001U)

#define CSL_CP_ACE_TRNG_STATUS_RESEED_AI_MASK                               (0x00000400U)
#define CSL_CP_ACE_TRNG_STATUS_RESEED_AI_SHIFT                              (0x0000000AU)
#define CSL_CP_ACE_TRNG_STATUS_RESEED_AI_MAX                                (0x00000001U)

#define CSL_CP_ACE_TRNG_STATUS_RESERVED_0_MASK                              (0x00001800U)
#define CSL_CP_ACE_TRNG_STATUS_RESERVED_0_SHIFT                             (0x0000000BU)
#define CSL_CP_ACE_TRNG_STATUS_RESERVED_0_MAX                               (0x00000003U)

#define CSL_CP_ACE_TRNG_STATUS_REPCNT_FAIL_MASK                             (0x00002000U)
#define CSL_CP_ACE_TRNG_STATUS_REPCNT_FAIL_SHIFT                            (0x0000000DU)
#define CSL_CP_ACE_TRNG_STATUS_REPCNT_FAIL_MAX                              (0x00000001U)

#define CSL_CP_ACE_TRNG_STATUS_APROP_FAIL_MASK                              (0x00004000U)
#define CSL_CP_ACE_TRNG_STATUS_APROP_FAIL_SHIFT                             (0x0000000EU)
#define CSL_CP_ACE_TRNG_STATUS_APROP_FAIL_MAX                               (0x00000001U)

#define CSL_CP_ACE_TRNG_STATUS_TEST_STUCK_OUT_MASK                          (0x00008000U)
#define CSL_CP_ACE_TRNG_STATUS_TEST_STUCK_OUT_SHIFT                         (0x0000000FU)
#define CSL_CP_ACE_TRNG_STATUS_TEST_STUCK_OUT_MAX                           (0x00000001U)

#define CSL_CP_ACE_TRNG_STATUS_BLOCKS_AVAILABLE_MASK                        (0x00FF0000U)
#define CSL_CP_ACE_TRNG_STATUS_BLOCKS_AVAILABLE_SHIFT                       (0x00000010U)
#define CSL_CP_ACE_TRNG_STATUS_BLOCKS_AVAILABLE_MAX                         (0x000000FFU)

#define CSL_CP_ACE_TRNG_STATUS_BLOCKS_THRESH_MASK                           (0x7F000000U)
#define CSL_CP_ACE_TRNG_STATUS_BLOCKS_THRESH_SHIFT                          (0x00000018U)
#define CSL_CP_ACE_TRNG_STATUS_BLOCKS_THRESH_MAX                            (0x0000007FU)

#define CSL_CP_ACE_TRNG_STATUS_NEED_CLOCK_MASK                              (0x80000000U)
#define CSL_CP_ACE_TRNG_STATUS_NEED_CLOCK_SHIFT                             (0x0000001FU)
#define CSL_CP_ACE_TRNG_STATUS_NEED_CLOCK_MAX                               (0x00000001U)

/* TRNG_INTACK_SECURE_MODE */

#define CSL_CP_ACE_TRNG_INTACK_SECURE_MODE_READY_MASK                       (0x00007FFFU)
#define CSL_CP_ACE_TRNG_INTACK_SECURE_MODE_READY_SHIFT                      (0x00000000U)
#define CSL_CP_ACE_TRNG_INTACK_SECURE_MODE_READY_MAX                        (0x00007FFFU)

/* TRNG_INTACK */

#define CSL_CP_ACE_TRNG_INTACK_READY_ACK_MASK                               (0x00000001U)
#define CSL_CP_ACE_TRNG_INTACK_READY_ACK_SHIFT                              (0x00000000U)
#define CSL_CP_ACE_TRNG_INTACK_READY_ACK_MAX                                (0x00000001U)

#define CSL_CP_ACE_TRNG_INTACK_SHUTDOWN_OFLO_ACK_MASK                       (0x00000002U)
#define CSL_CP_ACE_TRNG_INTACK_SHUTDOWN_OFLO_ACK_SHIFT                      (0x00000001U)
#define CSL_CP_ACE_TRNG_INTACK_SHUTDOWN_OFLO_ACK_MAX                        (0x00000001U)

#define CSL_CP_ACE_TRNG_INTACK_STUCK_OUT_ACK_MASK                           (0x00000004U)
#define CSL_CP_ACE_TRNG_INTACK_STUCK_OUT_ACK_SHIFT                          (0x00000002U)
#define CSL_CP_ACE_TRNG_INTACK_STUCK_OUT_ACK_MAX                            (0x00000001U)

#define CSL_CP_ACE_TRNG_INTACK_NOISE_FAIL_ACK_MASK                          (0x00000008U)
#define CSL_CP_ACE_TRNG_INTACK_NOISE_FAIL_ACK_SHIFT                         (0x00000003U)
#define CSL_CP_ACE_TRNG_INTACK_NOISE_FAIL_ACK_MAX                           (0x00000001U)

#define CSL_CP_ACE_TRNG_INTACK_RUN_FAIL_ACK_MASK                            (0x00000010U)
#define CSL_CP_ACE_TRNG_INTACK_RUN_FAIL_ACK_SHIFT                           (0x00000004U)
#define CSL_CP_ACE_TRNG_INTACK_RUN_FAIL_ACK_MAX                             (0x00000001U)

#define CSL_CP_ACE_TRNG_INTACK_LONG_RUN_FAIL_ACK_MASK                       (0x00000020U)
#define CSL_CP_ACE_TRNG_INTACK_LONG_RUN_FAIL_ACK_SHIFT                      (0x00000005U)
#define CSL_CP_ACE_TRNG_INTACK_LONG_RUN_FAIL_ACK_MAX                        (0x00000001U)

#define CSL_CP_ACE_TRNG_INTACK_POKER_FAIL_ACK_MASK                          (0x00000040U)
#define CSL_CP_ACE_TRNG_INTACK_POKER_FAIL_ACK_SHIFT                         (0x00000006U)
#define CSL_CP_ACE_TRNG_INTACK_POKER_FAIL_ACK_MAX                           (0x00000001U)

#define CSL_CP_ACE_TRNG_INTACK_MONOBIT_FAIL_ACK_MASK                        (0x00000080U)
#define CSL_CP_ACE_TRNG_INTACK_MONOBIT_FAIL_ACK_SHIFT                       (0x00000007U)
#define CSL_CP_ACE_TRNG_INTACK_MONOBIT_FAIL_ACK_MAX                         (0x00000001U)

#define CSL_CP_ACE_TRNG_INTACK_TEST_READY_ACK_MASK                          (0x00000100U)
#define CSL_CP_ACE_TRNG_INTACK_TEST_READY_ACK_SHIFT                         (0x00000008U)
#define CSL_CP_ACE_TRNG_INTACK_TEST_READY_ACK_MAX                           (0x00000001U)

#define CSL_CP_ACE_TRNG_INTACK_STUCK_NRBG_ACK_MASK                          (0x00000200U)
#define CSL_CP_ACE_TRNG_INTACK_STUCK_NRBG_ACK_SHIFT                         (0x00000009U)
#define CSL_CP_ACE_TRNG_INTACK_STUCK_NRBG_ACK_MAX                           (0x00000001U)

#define CSL_CP_ACE_TRNG_INTACK_STUCK_OUT_ACK2_MASK                          (0x00000400U)
#define CSL_CP_ACE_TRNG_INTACK_STUCK_OUT_ACK2_SHIFT                         (0x0000000AU)
#define CSL_CP_ACE_TRNG_INTACK_STUCK_OUT_ACK2_MAX                           (0x00000001U)

#define CSL_CP_ACE_TRNG_INTACK_OPEN_READ_GATE2_MASK                         (0x00001000U)
#define CSL_CP_ACE_TRNG_INTACK_OPEN_READ_GATE2_SHIFT                        (0x0000000CU)
#define CSL_CP_ACE_TRNG_INTACK_OPEN_READ_GATE2_MAX                          (0x00000001U)

#define CSL_CP_ACE_TRNG_INTACK_REPCNT_FAIL_ACK_MASK                         (0x00002000U)
#define CSL_CP_ACE_TRNG_INTACK_REPCNT_FAIL_ACK_SHIFT                        (0x0000000DU)
#define CSL_CP_ACE_TRNG_INTACK_REPCNT_FAIL_ACK_MAX                          (0x00000001U)

#define CSL_CP_ACE_TRNG_INTACK_APROP_FAIL_ACK_MASK                          (0x00004000U)
#define CSL_CP_ACE_TRNG_INTACK_APROP_FAIL_ACK_SHIFT                         (0x0000000EU)
#define CSL_CP_ACE_TRNG_INTACK_APROP_FAIL_ACK_MAX                           (0x00000001U)

#define CSL_CP_ACE_TRNG_INTACK_TEST_STUCK_OUT_MASK                          (0x00008000U)
#define CSL_CP_ACE_TRNG_INTACK_TEST_STUCK_OUT_SHIFT                         (0x0000000FU)
#define CSL_CP_ACE_TRNG_INTACK_TEST_STUCK_OUT_MAX                           (0x00000001U)

#define CSL_CP_ACE_TRNG_INTACK_BLOCKS_THRESH_MASK                           (0x7F000000U)
#define CSL_CP_ACE_TRNG_INTACK_BLOCKS_THRESH_SHIFT                          (0x00000018U)
#define CSL_CP_ACE_TRNG_INTACK_BLOCKS_THRESH_MAX                            (0x0000007FU)

#define CSL_CP_ACE_TRNG_INTACK_LOAD_THRESH_MASK                             (0x80000000U)
#define CSL_CP_ACE_TRNG_INTACK_LOAD_THRESH_SHIFT                            (0x0000001FU)
#define CSL_CP_ACE_TRNG_INTACK_LOAD_THRESH_MAX                              (0x00000001U)

/* TRNG_CONTROL */

#define CSL_CP_ACE_TRNG_CONTROL_READY_MASK_MASK                             (0x00000001U)
#define CSL_CP_ACE_TRNG_CONTROL_READY_MASK_SHIFT                            (0x00000000U)
#define CSL_CP_ACE_TRNG_CONTROL_READY_MASK_MAX                              (0x00000001U)

#define CSL_CP_ACE_TRNG_CONTROL_SHUTDOWN_OFLO_MASK_MASK                     (0x00000002U)
#define CSL_CP_ACE_TRNG_CONTROL_SHUTDOWN_OFLO_MASK_SHIFT                    (0x00000001U)
#define CSL_CP_ACE_TRNG_CONTROL_SHUTDOWN_OFLO_MASK_MAX                      (0x00000001U)

#define CSL_CP_ACE_TRNG_CONTROL_STUCK_OUT_MASK_MASK                         (0x00000004U)
#define CSL_CP_ACE_TRNG_CONTROL_STUCK_OUT_MASK_SHIFT                        (0x00000002U)
#define CSL_CP_ACE_TRNG_CONTROL_STUCK_OUT_MASK_MAX                          (0x00000001U)

#define CSL_CP_ACE_TRNG_CONTROL_NOISE_FAIL_MASK_MASK                        (0x00000008U)
#define CSL_CP_ACE_TRNG_CONTROL_NOISE_FAIL_MASK_SHIFT                       (0x00000003U)
#define CSL_CP_ACE_TRNG_CONTROL_NOISE_FAIL_MASK_MAX                         (0x00000001U)

#define CSL_CP_ACE_TRNG_CONTROL_RUN_FAIL_MASK_MASK                          (0x00000010U)
#define CSL_CP_ACE_TRNG_CONTROL_RUN_FAIL_MASK_SHIFT                         (0x00000004U)
#define CSL_CP_ACE_TRNG_CONTROL_RUN_FAIL_MASK_MAX                           (0x00000001U)

#define CSL_CP_ACE_TRNG_CONTROL_LONG_RUN_FAIL_MASK_MASK                     (0x00000020U)
#define CSL_CP_ACE_TRNG_CONTROL_LONG_RUN_FAIL_MASK_SHIFT                    (0x00000005U)
#define CSL_CP_ACE_TRNG_CONTROL_LONG_RUN_FAIL_MASK_MAX                      (0x00000001U)

#define CSL_CP_ACE_TRNG_CONTROL_POKER_FAIL_MASK_MASK                        (0x00000040U)
#define CSL_CP_ACE_TRNG_CONTROL_POKER_FAIL_MASK_SHIFT                       (0x00000006U)
#define CSL_CP_ACE_TRNG_CONTROL_POKER_FAIL_MASK_MAX                         (0x00000001U)

#define CSL_CP_ACE_TRNG_CONTROL_MONOBIT_FAIL_MASK_MASK                      (0x00000080U)
#define CSL_CP_ACE_TRNG_CONTROL_MONOBIT_FAIL_MASK_SHIFT                     (0x00000007U)
#define CSL_CP_ACE_TRNG_CONTROL_MONOBIT_FAIL_MASK_MAX                       (0x00000001U)

#define CSL_CP_ACE_TRNG_CONTROL_TEST_MODE_MASK                              (0x00000100U)
#define CSL_CP_ACE_TRNG_CONTROL_TEST_MODE_SHIFT                             (0x00000008U)
#define CSL_CP_ACE_TRNG_CONTROL_TEST_MODE_MAX                               (0x00000001U)

#define CSL_CP_ACE_TRNG_CONTROL_STUCK_NRBG_MASK_MASK                        (0x00000200U)
#define CSL_CP_ACE_TRNG_CONTROL_STUCK_NRBG_MASK_SHIFT                       (0x00000009U)
#define CSL_CP_ACE_TRNG_CONTROL_STUCK_NRBG_MASK_MAX                         (0x00000001U)

#define CSL_CP_ACE_TRNG_CONTROL_ENABLE_TRNG_MASK                            (0x00000400U)
#define CSL_CP_ACE_TRNG_CONTROL_ENABLE_TRNG_SHIFT                           (0x0000000AU)
#define CSL_CP_ACE_TRNG_CONTROL_ENABLE_TRNG_MAX                             (0x00000001U)

#define CSL_CP_ACE_TRNG_CONTROL_NO_WHITENING_MASK                           (0x00000800U)
#define CSL_CP_ACE_TRNG_CONTROL_NO_WHITENING_SHIFT                          (0x0000000BU)
#define CSL_CP_ACE_TRNG_CONTROL_NO_WHITENING_MAX                            (0x00000001U)

#define CSL_CP_ACE_TRNG_CONTROL_DRBG_EN_MASK                                (0x00001000U)
#define CSL_CP_ACE_TRNG_CONTROL_DRBG_EN_SHIFT                               (0x0000000CU)
#define CSL_CP_ACE_TRNG_CONTROL_DRBG_EN_MAX                                 (0x00000001U)

#define CSL_CP_ACE_TRNG_CONTROL_REPCNT_FAIL_MASK_MASK                       (0x00002000U)
#define CSL_CP_ACE_TRNG_CONTROL_REPCNT_FAIL_MASK_SHIFT                      (0x0000000DU)
#define CSL_CP_ACE_TRNG_CONTROL_REPCNT_FAIL_MASK_MAX                        (0x00000001U)

#define CSL_CP_ACE_TRNG_CONTROL_APROP_FAIL_MASK_MASK                        (0x00004000U)
#define CSL_CP_ACE_TRNG_CONTROL_APROP_FAIL_MASK_SHIFT                       (0x0000000EU)
#define CSL_CP_ACE_TRNG_CONTROL_APROP_FAIL_MASK_MAX                         (0x00000001U)

#define CSL_CP_ACE_TRNG_CONTROL_RE_SEED_MASK                                (0x00008000U)
#define CSL_CP_ACE_TRNG_CONTROL_RE_SEED_SHIFT                               (0x0000000FU)
#define CSL_CP_ACE_TRNG_CONTROL_RE_SEED_MAX                                 (0x00000001U)

#define CSL_CP_ACE_TRNG_CONTROL_REQUEST_DATA_MASK                           (0x00010000U)
#define CSL_CP_ACE_TRNG_CONTROL_REQUEST_DATA_SHIFT                          (0x00000010U)
#define CSL_CP_ACE_TRNG_CONTROL_REQUEST_DATA_MAX                            (0x00000001U)

#define CSL_CP_ACE_TRNG_CONTROL_REQUEST_HOLD_MASK                           (0x00020000U)
#define CSL_CP_ACE_TRNG_CONTROL_REQUEST_HOLD_SHIFT                          (0x00000011U)
#define CSL_CP_ACE_TRNG_CONTROL_REQUEST_HOLD_MAX                            (0x00000001U)

#define CSL_CP_ACE_TRNG_CONTROL_RESERVED_0_MASK                             (0x000C0000U)
#define CSL_CP_ACE_TRNG_CONTROL_RESERVED_0_SHIFT                            (0x00000012U)
#define CSL_CP_ACE_TRNG_CONTROL_RESERVED_0_MAX                              (0x00000003U)

#define CSL_CP_ACE_TRNG_CONTROL_DATA_BLOCKS_MASK                            (0xFFF00000U)
#define CSL_CP_ACE_TRNG_CONTROL_DATA_BLOCKS_SHIFT                           (0x00000014U)
#define CSL_CP_ACE_TRNG_CONTROL_DATA_BLOCKS_MAX                             (0x00000FFFU)

/* TRNG_CONFIG */

#define CSL_CP_ACE_TRNG_CONFIG_NOISE_BLOCKS_MASK                            (0x0000001FU)
#define CSL_CP_ACE_TRNG_CONFIG_NOISE_BLOCKS_SHIFT                           (0x00000000U)
#define CSL_CP_ACE_TRNG_CONFIG_NOISE_BLOCKS_MAX                             (0x0000001FU)

#define CSL_CP_ACE_TRNG_CONFIG_USE_STARTUP_BITS_MASK                        (0x00000020U)
#define CSL_CP_ACE_TRNG_CONFIG_USE_STARTUP_BITS_SHIFT                       (0x00000005U)
#define CSL_CP_ACE_TRNG_CONFIG_USE_STARTUP_BITS_MAX                         (0x00000001U)

#define CSL_CP_ACE_TRNG_CONFIG_SCALE_MASK                                   (0x000000C0U)
#define CSL_CP_ACE_TRNG_CONFIG_SCALE_SHIFT                                  (0x00000006U)
#define CSL_CP_ACE_TRNG_CONFIG_SCALE_MAX                                    (0x00000003U)

#define CSL_CP_ACE_TRNG_CONFIG_SAMPLE_DIV_MASK                              (0x00000F00U)
#define CSL_CP_ACE_TRNG_CONFIG_SAMPLE_DIV_SHIFT                             (0x00000008U)
#define CSL_CP_ACE_TRNG_CONFIG_SAMPLE_DIV_MAX                               (0x0000000FU)

#define CSL_CP_ACE_TRNG_CONFIG_READ_TIMEOUT_MASK                            (0x0000F000U)
#define CSL_CP_ACE_TRNG_CONFIG_READ_TIMEOUT_SHIFT                           (0x0000000CU)
#define CSL_CP_ACE_TRNG_CONFIG_READ_TIMEOUT_MAX                             (0x0000000FU)

#define CSL_CP_ACE_TRNG_CONFIG_SAMPLE_CYCLES_MASK                           (0xFFFF0000U)
#define CSL_CP_ACE_TRNG_CONFIG_SAMPLE_CYCLES_SHIFT                          (0x00000010U)
#define CSL_CP_ACE_TRNG_CONFIG_SAMPLE_CYCLES_MAX                            (0x0000FFFFU)

/* New field macros specific for TRNG version 2.4.1 and later */
#define CSL_CP_ACE_TRNG_CONFIG_NOISE_BLOCKS_2_4_1_MASK                      (0x0000001FU)
#define CSL_CP_ACE_TRNG_CONFIG_NOISE_BLOCKS_2_4_1_SHIFT                     (0x00000000U)
#define CSL_CP_ACE_TRNG_CONFIG_NOISE_BLOCKS_2_4_1_MAX                       (0x0000001FU)

#define CSL_CP_ACE_TRNG_CONFIG_USE_STARTUP_BITS_2_4_1_MASK                  (0x00000020U)
#define CSL_CP_ACE_TRNG_CONFIG_USE_STARTUP_BITS_2_4_1_SHIFT                 (0x00000005U)
#define CSL_CP_ACE_TRNG_CONFIG_USE_STARTUP_BITS_2_4_1_MAX                   (0x00000001U)

/* TRNG_ALARMCNT */

#define CSL_CP_ACE_TRNG_ALARMCNT_ALARM_THRESHOLD_MASK                       (0x000000FFU)
#define CSL_CP_ACE_TRNG_ALARMCNT_ALARM_THRESHOLD_SHIFT                      (0x00000000U)
#define CSL_CP_ACE_TRNG_ALARMCNT_ALARM_THRESHOLD_MAX                        (0x000000FFU)

#define CSL_CP_ACE_TRNG_ALARMCNT_RESERVED_0_MASK                            (0x00007F00U)
#define CSL_CP_ACE_TRNG_ALARMCNT_RESERVED_0_SHIFT                           (0x00000008U)
#define CSL_CP_ACE_TRNG_ALARMCNT_RESERVED_0_MAX                             (0x0000007FU)

#define CSL_CP_ACE_TRNG_ALARMCNT_STALL_RUN_POKER_MASK                       (0x00008000U)
#define CSL_CP_ACE_TRNG_ALARMCNT_STALL_RUN_POKER_SHIFT                      (0x0000000FU)
#define CSL_CP_ACE_TRNG_ALARMCNT_STALL_RUN_POKER_MAX                        (0x00000001U)

#define CSL_CP_ACE_TRNG_ALARMCNT_SHUTDOWN_THRESHOLD_MASK                    (0x001F0000U)
#define CSL_CP_ACE_TRNG_ALARMCNT_SHUTDOWN_THRESHOLD_SHIFT                   (0x00000010U)
#define CSL_CP_ACE_TRNG_ALARMCNT_SHUTDOWN_THRESHOLD_MAX                     (0x0000001FU)

#define CSL_CP_ACE_TRNG_ALARMCNT_RESERVED_1_MASK                            (0x00600000U)
#define CSL_CP_ACE_TRNG_ALARMCNT_RESERVED_1_SHIFT                           (0x00000015U)
#define CSL_CP_ACE_TRNG_ALARMCNT_RESERVED_1_MAX                             (0x00000003U)

#define CSL_CP_ACE_TRNG_ALARMCNT_SHUTDOWN_FATAL_MASK                        (0x00800000U)
#define CSL_CP_ACE_TRNG_ALARMCNT_SHUTDOWN_FATAL_SHIFT                       (0x00000017U)
#define CSL_CP_ACE_TRNG_ALARMCNT_SHUTDOWN_FATAL_MAX                         (0x00000001U)

#define CSL_CP_ACE_TRNG_ALARMCNT_SHUTDOWN_COUNT_MASK                        (0x3F000000U)
#define CSL_CP_ACE_TRNG_ALARMCNT_SHUTDOWN_COUNT_SHIFT                       (0x00000018U)
#define CSL_CP_ACE_TRNG_ALARMCNT_SHUTDOWN_COUNT_MAX                         (0x0000003FU)

#define CSL_CP_ACE_TRNG_ALARMCNT_RESERVED_2_MASK                            (0xC0000000U)
#define CSL_CP_ACE_TRNG_ALARMCNT_RESERVED_2_SHIFT                           (0x0000001EU)
#define CSL_CP_ACE_TRNG_ALARMCNT_RESERVED_2_MAX                             (0x00000003U)

/* TRNG_FROENABLE */

#define CSL_CP_ACE_TRNG_FROENABLE_TRNG_FROENABLE_MASK                       (0x000000FFU)
#define CSL_CP_ACE_TRNG_FROENABLE_TRNG_FROENABLE_SHIFT                      (0x00000000U)
#define CSL_CP_ACE_TRNG_FROENABLE_TRNG_FROENABLE_MAX                        (0x000000FFU)

/* TRNG_FRODETUNE */

#define CSL_CP_ACE_TRNG_FRODETUNE_TRNG_FRODETUNE_MASK                       (0x000000FFU)
#define CSL_CP_ACE_TRNG_FRODETUNE_TRNG_FRODETUNE_SHIFT                      (0x00000000U)
#define CSL_CP_ACE_TRNG_FRODETUNE_TRNG_FRODETUNE_MAX                        (0x000000FFU)

/* TRNG_ALARMMASK */

#define CSL_CP_ACE_TRNG_ALARMMASK_TRNG_ALARMMASK_MASK                       (0x000000FFU)
#define CSL_CP_ACE_TRNG_ALARMMASK_TRNG_ALARMMASK_SHIFT                      (0x00000000U)
#define CSL_CP_ACE_TRNG_ALARMMASK_TRNG_ALARMMASK_MAX                        (0x000000FFU)

/* TRNG_ALARMSTOP */

#define CSL_CP_ACE_TRNG_ALARMSTOP_TRNG_ALARMSTOP_MASK                       (0x000000FFU)
#define CSL_CP_ACE_TRNG_ALARMSTOP_TRNG_ALARMSTOP_SHIFT                      (0x00000000U)
#define CSL_CP_ACE_TRNG_ALARMSTOP_TRNG_ALARMSTOP_MAX                        (0x000000FFU)

/* TRNG_RAW_L */

#define CSL_CP_ACE_TRNG_RAW_L_TRNG_RAW_L_MASK                               (0xFFFFFFFFU)
#define CSL_CP_ACE_TRNG_RAW_L_TRNG_RAW_L_SHIFT                              (0x00000000U)
#define CSL_CP_ACE_TRNG_RAW_L_TRNG_RAW_L_MAX                                (0xFFFFFFFFU)

/* TRNG_RAW_H */

#define CSL_CP_ACE_TRNG_RAW_H_TRNG_RAW_H_MASK                               (0xFFFFFFFFU)
#define CSL_CP_ACE_TRNG_RAW_H_TRNG_RAW_H_SHIFT                              (0x00000000U)
#define CSL_CP_ACE_TRNG_RAW_H_TRNG_RAW_H_MAX                                (0xFFFFFFFFU)

/* TRNG_SPB_TESTS */

#define CSL_CP_ACE_TRNG_SPB_TESTS_REPCNT_CUTOFF_MASK                        (0x0000003FU)
#define CSL_CP_ACE_TRNG_SPB_TESTS_REPCNT_CUTOFF_SHIFT                       (0x00000000U)
#define CSL_CP_ACE_TRNG_SPB_TESTS_REPCNT_CUTOFF_MAX                         (0x0000003FU)

#define CSL_CP_ACE_TRNG_SPB_TESTS_RESERVED_0_MASK                           (0x000000C0U)
#define CSL_CP_ACE_TRNG_SPB_TESTS_RESERVED_0_SHIFT                          (0x00000006U)
#define CSL_CP_ACE_TRNG_SPB_TESTS_RESERVED_0_MAX                            (0x00000003U)

#define CSL_CP_ACE_TRNG_SPB_TESTS_APROP_64_CUTOFF_MASK                      (0x00003F00U)
#define CSL_CP_ACE_TRNG_SPB_TESTS_APROP_64_CUTOFF_SHIFT                     (0x00000008U)
#define CSL_CP_ACE_TRNG_SPB_TESTS_APROP_64_CUTOFF_MAX                       (0x0000003FU)

#define CSL_CP_ACE_TRNG_SPB_TESTS_RESERVED_1_MASK                           (0x0000C000U)
#define CSL_CP_ACE_TRNG_SPB_TESTS_RESERVED_1_SHIFT                          (0x0000000EU)
#define CSL_CP_ACE_TRNG_SPB_TESTS_RESERVED_1_MAX                            (0x00000003U)

#define CSL_CP_ACE_TRNG_SPB_TESTS_APROP_512_CUTOFF_MASK                     (0x01FF0000U)
#define CSL_CP_ACE_TRNG_SPB_TESTS_APROP_512_CUTOFF_SHIFT                    (0x00000010U)
#define CSL_CP_ACE_TRNG_SPB_TESTS_APROP_512_CUTOFF_MAX                      (0x000001FFU)

#define CSL_CP_ACE_TRNG_SPB_TESTS_RESERVED_2_MASK                           (0x0E000000U)
#define CSL_CP_ACE_TRNG_SPB_TESTS_RESERVED_2_SHIFT                          (0x00000019U)
#define CSL_CP_ACE_TRNG_SPB_TESTS_RESERVED_2_MAX                            (0x00000007U)

#define CSL_CP_ACE_TRNG_SPB_TESTS_SHOW_COUNTERS_MASK                        (0x10000000U)
#define CSL_CP_ACE_TRNG_SPB_TESTS_SHOW_COUNTERS_SHIFT                       (0x0000001CU)
#define CSL_CP_ACE_TRNG_SPB_TESTS_SHOW_COUNTERS_MAX                         (0x00000001U)

#define CSL_CP_ACE_TRNG_SPB_TESTS_SHOW_VALUES_MASK                          (0x20000000U)
#define CSL_CP_ACE_TRNG_SPB_TESTS_SHOW_VALUES_SHIFT                         (0x0000001DU)
#define CSL_CP_ACE_TRNG_SPB_TESTS_SHOW_VALUES_MAX                           (0x00000001U)

#define CSL_CP_ACE_TRNG_SPB_TESTS_APROP_64_FAIL_MASK                        (0x40000000U)
#define CSL_CP_ACE_TRNG_SPB_TESTS_APROP_64_FAIL_SHIFT                       (0x0000001EU)
#define CSL_CP_ACE_TRNG_SPB_TESTS_APROP_64_FAIL_MAX                         (0x00000001U)

#define CSL_CP_ACE_TRNG_SPB_TESTS_APROP_512_FAIL_MASK                       (0x80000000U)
#define CSL_CP_ACE_TRNG_SPB_TESTS_APROP_512_FAIL_SHIFT                      (0x0000001FU)
#define CSL_CP_ACE_TRNG_SPB_TESTS_APROP_512_FAIL_MAX                        (0x00000001U)

/* TRNG_COUNT */

#define CSL_CP_ACE_TRNG_COUNT_SAMPLE_CYC_CNT_MASK                           (0x0000FFFFU)
#define CSL_CP_ACE_TRNG_COUNT_SAMPLE_CYC_CNT_SHIFT                          (0x00000000U)
#define CSL_CP_ACE_TRNG_COUNT_SAMPLE_CYC_CNT_MAX                            (0x0000FFFFU)

#define CSL_CP_ACE_TRNG_COUNT_NOISE_BLK_CNT_MASK                            (0x001F0000U)
#define CSL_CP_ACE_TRNG_COUNT_NOISE_BLK_CNT_SHIFT                           (0x00000010U)
#define CSL_CP_ACE_TRNG_COUNT_NOISE_BLK_CNT_MAX                             (0x0000001FU)

#define CSL_CP_ACE_TRNG_COUNT_RESERVED_0_MASK                               (0x00E00000U)
#define CSL_CP_ACE_TRNG_COUNT_RESERVED_0_SHIFT                              (0x00000015U)
#define CSL_CP_ACE_TRNG_COUNT_RESERVED_0_MAX                                (0x00000007U)

#define CSL_CP_ACE_TRNG_COUNT_SAMPLE_CYC_EXT_MASK                           (0x3F000000U)
#define CSL_CP_ACE_TRNG_COUNT_SAMPLE_CYC_EXT_SHIFT                          (0x00000018U)
#define CSL_CP_ACE_TRNG_COUNT_SAMPLE_CYC_EXT_MAX                            (0x0000003FU)

#define CSL_CP_ACE_TRNG_COUNT_RESERVED_1_MASK                               (0xC0000000U)
#define CSL_CP_ACE_TRNG_COUNT_RESERVED_1_SHIFT                              (0x0000001EU)
#define CSL_CP_ACE_TRNG_COUNT_RESERVED_1_MAX                                (0x00000003U)

/* New field macros specific for TRNG version 2.4.1 and later */
#define CSL_CP_ACE_TRNG_COUNT_NOISE_BLK_CNT_2_4_1_MASK                      (0x001F0000U)
#define CSL_CP_ACE_TRNG_COUNT_NOISE_BLK_CNT_2_4_1_SHIFT                     (0x00000010U)
#define CSL_CP_ACE_TRNG_COUNT_NOISE_BLK_CNT_2_4_1_MAX                       (0x0000001FU)

#define CSL_CP_ACE_TRNG_COUNT_RESERVED_0_2_4_1_MASK                         (0x00E00000U)
#define CSL_CP_ACE_TRNG_COUNT_RESERVED_0_2_4_1_SHIFT                        (0x00000015U)
#define CSL_CP_ACE_TRNG_COUNT_RESERVED_0_2_4_1_MAX                          (0x00000007U)

/* TRNG_PS_AI_0 */

#define CSL_CP_ACE_TRNG_PS_AI_0_VECTOR_MASK                                 (0xFFFFFFFFU)
#define CSL_CP_ACE_TRNG_PS_AI_0_VECTOR_SHIFT                                (0x00000000U)
#define CSL_CP_ACE_TRNG_PS_AI_0_VECTOR_MAX                                  (0xFFFFFFFFU)

/* TRNG_PS_AI_1 */

#define CSL_CP_ACE_TRNG_PS_AI_1_VECTOR_MASK                                 (0xFFFFFFFFU)
#define CSL_CP_ACE_TRNG_PS_AI_1_VECTOR_SHIFT                                (0x00000000U)
#define CSL_CP_ACE_TRNG_PS_AI_1_VECTOR_MAX                                  (0xFFFFFFFFU)

/* TRNG_PS_AI_2 */

#define CSL_CP_ACE_TRNG_PS_AI_2_VECTOR_MASK                                 (0xFFFFFFFFU)
#define CSL_CP_ACE_TRNG_PS_AI_2_VECTOR_SHIFT                                (0x00000000U)
#define CSL_CP_ACE_TRNG_PS_AI_2_VECTOR_MAX                                  (0xFFFFFFFFU)

/* TRNG_PS_AI_3 */

#define CSL_CP_ACE_TRNG_PS_AI_3_VECTOR_MASK                                 (0xFFFFFFFFU)
#define CSL_CP_ACE_TRNG_PS_AI_3_VECTOR_SHIFT                                (0x00000000U)
#define CSL_CP_ACE_TRNG_PS_AI_3_VECTOR_MAX                                  (0xFFFFFFFFU)

/* TRNG_PS_AI_4 */

#define CSL_CP_ACE_TRNG_PS_AI_4_VECTOR_MASK                                 (0xFFFFFFFFU)
#define CSL_CP_ACE_TRNG_PS_AI_4_VECTOR_SHIFT                                (0x00000000U)
#define CSL_CP_ACE_TRNG_PS_AI_4_VECTOR_MAX                                  (0xFFFFFFFFU)

/* TRNG_PS_AI_5 */

#define CSL_CP_ACE_TRNG_PS_AI_5_VECTOR_MASK                                 (0xFFFFFFFFU)
#define CSL_CP_ACE_TRNG_PS_AI_5_VECTOR_SHIFT                                (0x00000000U)
#define CSL_CP_ACE_TRNG_PS_AI_5_VECTOR_MAX                                  (0xFFFFFFFFU)

/* TRNG_PS_AI_6 */

#define CSL_CP_ACE_TRNG_PS_AI_6_VECTOR_MASK                                 (0xFFFFFFFFU)
#define CSL_CP_ACE_TRNG_PS_AI_6_VECTOR_SHIFT                                (0x00000000U)
#define CSL_CP_ACE_TRNG_PS_AI_6_VECTOR_MAX                                  (0xFFFFFFFFU)

/* TRNG_PS_AI_7 */

#define CSL_CP_ACE_TRNG_PS_AI_7_VECTOR_MASK                                 (0xFFFFFFFFU)
#define CSL_CP_ACE_TRNG_PS_AI_7_VECTOR_SHIFT                                (0x00000000U)
#define CSL_CP_ACE_TRNG_PS_AI_7_VECTOR_MAX                                  (0xFFFFFFFFU)

/* TRNG_PS_AI_8 */

#define CSL_CP_ACE_TRNG_PS_AI_8_VECTOR_MASK                                 (0xFFFFFFFFU)
#define CSL_CP_ACE_TRNG_PS_AI_8_VECTOR_SHIFT                                (0x00000000U)
#define CSL_CP_ACE_TRNG_PS_AI_8_VECTOR_MAX                                  (0xFFFFFFFFU)

/* TRNG_PS_AI_9 */

#define CSL_CP_ACE_TRNG_PS_AI_9_VECTOR_MASK                                 (0xFFFFFFFFU)
#define CSL_CP_ACE_TRNG_PS_AI_9_VECTOR_SHIFT                                (0x00000000U)
#define CSL_CP_ACE_TRNG_PS_AI_9_VECTOR_MAX                                  (0xFFFFFFFFU)

/* TRNG_PS_AI_10 */

#define CSL_CP_ACE_TRNG_PS_AI_10_VECTOR_MASK                                (0xFFFFFFFFU)
#define CSL_CP_ACE_TRNG_PS_AI_10_VECTOR_SHIFT                               (0x00000000U)
#define CSL_CP_ACE_TRNG_PS_AI_10_VECTOR_MAX                                 (0xFFFFFFFFU)

/* TRNG_PS_AI_11 */

#define CSL_CP_ACE_TRNG_PS_AI_11_VECTOR_MASK                                (0xFFFFFFFFU)
#define CSL_CP_ACE_TRNG_PS_AI_11_VECTOR_SHIFT                               (0x00000000U)
#define CSL_CP_ACE_TRNG_PS_AI_11_VECTOR_MAX                                 (0xFFFFFFFFU)

/* TRNG_TEST */

#define CSL_CP_ACE_TRNG_TEST_TEST_EN_OUT_MASK                               (0x00000001U)
#define CSL_CP_ACE_TRNG_TEST_TEST_EN_OUT_SHIFT                              (0x00000000U)
#define CSL_CP_ACE_TRNG_TEST_TEST_EN_OUT_MAX                                (0x00000001U)

#define CSL_CP_ACE_TRNG_TEST_TEST_PATT_FR_MASK                              (0x00000002U)
#define CSL_CP_ACE_TRNG_TEST_TEST_PATT_FR_SHIFT                             (0x00000001U)
#define CSL_CP_ACE_TRNG_TEST_TEST_PATT_FR_MAX                               (0x00000001U)

#define CSL_CP_ACE_TRNG_TEST_TEST_PATT_DET_MASK                             (0x00000004U)
#define CSL_CP_ACE_TRNG_TEST_TEST_PATT_DET_SHIFT                            (0x00000002U)
#define CSL_CP_ACE_TRNG_TEST_TEST_PATT_DET_MAX                              (0x00000001U)

#define CSL_CP_ACE_TRNG_TEST_TEST_SHIFTREG_MASK                             (0x00000008U)
#define CSL_CP_ACE_TRNG_TEST_TEST_SHIFTREG_SHIFT                            (0x00000003U)
#define CSL_CP_ACE_TRNG_TEST_TEST_SHIFTREG_MAX                              (0x00000001U)

#define CSL_CP_ACE_TRNG_TEST_CONT_POKER_MASK                                (0x00000010U)
#define CSL_CP_ACE_TRNG_TEST_CONT_POKER_SHIFT                               (0x00000004U)
#define CSL_CP_ACE_TRNG_TEST_CONT_POKER_MAX                                 (0x00000001U)

#define CSL_CP_ACE_TRNG_TEST_TEST_KNOWN_NOISE_MASK                          (0x00000020U)
#define CSL_CP_ACE_TRNG_TEST_TEST_KNOWN_NOISE_SHIFT                         (0x00000005U)
#define CSL_CP_ACE_TRNG_TEST_TEST_KNOWN_NOISE_MAX                           (0x00000001U)

#define CSL_CP_ACE_TRNG_TEST_TEST_AES_256_MASK                              (0x00000040U)
#define CSL_CP_ACE_TRNG_TEST_TEST_AES_256_SHIFT                             (0x00000006U)
#define CSL_CP_ACE_TRNG_TEST_TEST_AES_256_MAX                               (0x00000001U)

#define CSL_CP_ACE_TRNG_TEST_TEST_SP_800_90_MASK                            (0x00000080U)
#define CSL_CP_ACE_TRNG_TEST_TEST_SP_800_90_SHIFT                           (0x00000007U)
#define CSL_CP_ACE_TRNG_TEST_TEST_SP_800_90_MAX                             (0x00000001U)

#define CSL_CP_ACE_TRNG_TEST_TEST_SELECT_MASK                               (0x00001F00U)
#define CSL_CP_ACE_TRNG_TEST_TEST_SELECT_SHIFT                              (0x00000008U)
#define CSL_CP_ACE_TRNG_TEST_TEST_SELECT_MAX                                (0x0000001FU)

#define CSL_CP_ACE_TRNG_TEST_TEST_NOISE_MASK                                (0x00002000U)
#define CSL_CP_ACE_TRNG_TEST_TEST_NOISE_SHIFT                               (0x0000000DU)
#define CSL_CP_ACE_TRNG_TEST_TEST_NOISE_MAX                                 (0x00000001U)

#define CSL_CP_ACE_TRNG_TEST_TEST_SPB_MASK                                  (0x00004000U)
#define CSL_CP_ACE_TRNG_TEST_TEST_SPB_SHIFT                                 (0x0000000EU)
#define CSL_CP_ACE_TRNG_TEST_TEST_SPB_MAX                                   (0x00000001U)

#define CSL_CP_ACE_TRNG_TEST_RESERVED_0_MASK                                (0x00008000U)
#define CSL_CP_ACE_TRNG_TEST_RESERVED_0_SHIFT                               (0x0000000FU)
#define CSL_CP_ACE_TRNG_TEST_RESERVED_0_MAX                                 (0x00000001U)

#define CSL_CP_ACE_TRNG_TEST_TEST_PATTERN_MASK                              (0x0FFF0000U)
#define CSL_CP_ACE_TRNG_TEST_TEST_PATTERN_SHIFT                             (0x00000010U)
#define CSL_CP_ACE_TRNG_TEST_TEST_PATTERN_MAX                               (0x00000FFFU)

#define CSL_CP_ACE_TRNG_TEST_FRO_TESTIN2_NOT_MASK                           (0x10000000U)
#define CSL_CP_ACE_TRNG_TEST_FRO_TESTIN2_NOT_SHIFT                          (0x0000001CU)
#define CSL_CP_ACE_TRNG_TEST_FRO_TESTIN2_NOT_MAX                            (0x00000001U)

#define CSL_CP_ACE_TRNG_TEST_FRO_TESTIN3_MASK                               (0x20000000U)
#define CSL_CP_ACE_TRNG_TEST_FRO_TESTIN3_SHIFT                              (0x0000001DU)
#define CSL_CP_ACE_TRNG_TEST_FRO_TESTIN3_MAX                                (0x00000001U)

#define CSL_CP_ACE_TRNG_TEST_FRO_TESTIN4_MASK                               (0x40000000U)
#define CSL_CP_ACE_TRNG_TEST_FRO_TESTIN4_SHIFT                              (0x0000001EU)
#define CSL_CP_ACE_TRNG_TEST_FRO_TESTIN4_MAX                                (0x00000001U)

#define CSL_CP_ACE_TRNG_TEST_TEST_IRQ_MASK                                  (0x80000000U)
#define CSL_CP_ACE_TRNG_TEST_TEST_IRQ_SHIFT                                 (0x0000001FU)
#define CSL_CP_ACE_TRNG_TEST_TEST_IRQ_MAX                                   (0x00000001U)

/* TRNG_BLOCKCNT */

#define CSL_CP_ACE_TRNG_BLOCKCNT_RESERVED_0_MASK                            (0x0000000FU)
#define CSL_CP_ACE_TRNG_BLOCKCNT_RESERVED_0_SHIFT                           (0x00000000U)
#define CSL_CP_ACE_TRNG_BLOCKCNT_RESERVED_0_MAX                             (0x0000000FU)

#define CSL_CP_ACE_TRNG_BLOCKCNT_BLOCK_COUNT_MASK                           (0xFFFFFFF0U)
#define CSL_CP_ACE_TRNG_BLOCKCNT_BLOCK_COUNT_SHIFT                          (0x00000004U)
#define CSL_CP_ACE_TRNG_BLOCKCNT_BLOCK_COUNT_MAX                            (0x0FFFFFFFU)

/* TRNG_OPTIONS */

#define CSL_CP_ACE_TRNG_OPTIONS_POST_PROCESSOR_MASK                         (0x00000007U)
#define CSL_CP_ACE_TRNG_OPTIONS_POST_PROCESSOR_SHIFT                        (0x00000000U)
#define CSL_CP_ACE_TRNG_OPTIONS_POST_PROCESSOR_MAX                          (0x00000007U)

#define CSL_CP_ACE_TRNG_OPTIONS_RESERVED_0_MASK                             (0x00000038U)
#define CSL_CP_ACE_TRNG_OPTIONS_RESERVED_0_SHIFT                            (0x00000003U)
#define CSL_CP_ACE_TRNG_OPTIONS_RESERVED_0_MAX                              (0x00000007U)

#define CSL_CP_ACE_TRNG_OPTIONS_NR_OF_FROS_MASK                             (0x00000FC0U)
#define CSL_CP_ACE_TRNG_OPTIONS_NR_OF_FROS_SHIFT                            (0x00000006U)
#define CSL_CP_ACE_TRNG_OPTIONS_NR_OF_FROS_MAX                              (0x0000003FU)

#define CSL_CP_ACE_TRNG_OPTIONS_BUFFER_SIZE_MASK                            (0x00007000U)
#define CSL_CP_ACE_TRNG_OPTIONS_BUFFER_SIZE_SHIFT                           (0x0000000CU)
#define CSL_CP_ACE_TRNG_OPTIONS_BUFFER_SIZE_MAX                             (0x00000007U)

#define CSL_CP_ACE_TRNG_OPTIONS_RESERVED_1_MASK                             (0x00008000U)
#define CSL_CP_ACE_TRNG_OPTIONS_RESERVED_1_SHIFT                            (0x0000000FU)
#define CSL_CP_ACE_TRNG_OPTIONS_RESERVED_1_MAX                              (0x00000001U)

#define CSL_CP_ACE_TRNG_OPTIONS_PR_TEST_MASK                                (0x00010000U)
#define CSL_CP_ACE_TRNG_OPTIONS_PR_TEST_SHIFT                               (0x00000010U)
#define CSL_CP_ACE_TRNG_OPTIONS_PR_TEST_MAX                                 (0x00000001U)

#define CSL_CP_ACE_TRNG_OPTIONS_CONDITIONER_MASK                            (0x00060000U)
#define CSL_CP_ACE_TRNG_OPTIONS_CONDITIONER_SHIFT                           (0x00000011U)
#define CSL_CP_ACE_TRNG_OPTIONS_CONDITIONER_MAX                             (0x00000003U)

#define CSL_CP_ACE_TRNG_OPTIONS_DETUNING_OPTION_MASK                        (0x00080000U)
#define CSL_CP_ACE_TRNG_OPTIONS_DETUNING_OPTION_SHIFT                       (0x00000013U)
#define CSL_CP_ACE_TRNG_OPTIONS_DETUNING_OPTION_MAX                         (0x00000001U)

#define CSL_CP_ACE_TRNG_OPTIONS_RESERVED_2_MASK                             (0x00100000U)
#define CSL_CP_ACE_TRNG_OPTIONS_RESERVED_2_SHIFT                            (0x00000014U)
#define CSL_CP_ACE_TRNG_OPTIONS_RESERVED_2_MAX                              (0x00000001U)

#define CSL_CP_ACE_TRNG_OPTIONS_APROP_512_MASK                              (0x00200000U)
#define CSL_CP_ACE_TRNG_OPTIONS_APROP_512_SHIFT                             (0x00000015U)
#define CSL_CP_ACE_TRNG_OPTIONS_APROP_512_MAX                               (0x00000001U)

#define CSL_CP_ACE_TRNG_OPTIONS_RESERVED_3_MASK                             (0x00400000U)
#define CSL_CP_ACE_TRNG_OPTIONS_RESERVED_3_SHIFT                            (0x00000016U)
#define CSL_CP_ACE_TRNG_OPTIONS_RESERVED_3_MAX                              (0x00000001U)

#define CSL_CP_ACE_TRNG_OPTIONS_AUTO_DETUNE_MASK                            (0x00800000U)
#define CSL_CP_ACE_TRNG_OPTIONS_AUTO_DETUNE_SHIFT                           (0x00000017U)
#define CSL_CP_ACE_TRNG_OPTIONS_AUTO_DETUNE_MAX                             (0x00000001U)

#define CSL_CP_ACE_TRNG_OPTIONS_DETUNE_COUNT_MASK                           (0xFF000000U)
#define CSL_CP_ACE_TRNG_OPTIONS_DETUNE_COUNT_SHIFT                          (0x00000018U)
#define CSL_CP_ACE_TRNG_OPTIONS_DETUNE_COUNT_MAX                            (0x000000FFU)

/* TRNG_EIP_REV */

#define CSL_CP_ACE_TRNG_EIP_REV_BASIS_EIP_NUMBER_MASK                       (0x000000FFU)
#define CSL_CP_ACE_TRNG_EIP_REV_BASIS_EIP_NUMBER_SHIFT                      (0x00000000U)
#define CSL_CP_ACE_TRNG_EIP_REV_BASIS_EIP_NUMBER_MAX                        (0x000000FFU)

#define CSL_CP_ACE_TRNG_EIP_REV_COMPLEMENT_OF_BASIC_EIP_NUMBER_MASK         (0x0000FF00U)
#define CSL_CP_ACE_TRNG_EIP_REV_COMPLEMENT_OF_BASIC_EIP_NUMBER_SHIFT        (0x00000008U)
#define CSL_CP_ACE_TRNG_EIP_REV_COMPLEMENT_OF_BASIC_EIP_NUMBER_MAX          (0x000000FFU)

#define CSL_CP_ACE_TRNG_EIP_REV_HW_PATCH_LEVEL_MASK                         (0x000F0000U)
#define CSL_CP_ACE_TRNG_EIP_REV_HW_PATCH_LEVEL_SHIFT                        (0x00000010U)
#define CSL_CP_ACE_TRNG_EIP_REV_HW_PATCH_LEVEL_MAX                          (0x0000000FU)

#define CSL_CP_ACE_TRNG_EIP_REV_MINOR_HW_REVISION_MASK                      (0x00F00000U)
#define CSL_CP_ACE_TRNG_EIP_REV_MINOR_HW_REVISION_SHIFT                     (0x00000014U)
#define CSL_CP_ACE_TRNG_EIP_REV_MINOR_HW_REVISION_MAX                       (0x0000000FU)

#define CSL_CP_ACE_TRNG_EIP_REV_MAJOR_HW_REVISION_MASK                      (0x0F000000U)
#define CSL_CP_ACE_TRNG_EIP_REV_MAJOR_HW_REVISION_SHIFT                     (0x00000018U)
#define CSL_CP_ACE_TRNG_EIP_REV_MAJOR_HW_REVISION_MAX                       (0x0000000FU)

#define CSL_CP_ACE_TRNG_EIP_REV_RESERVED_0_MASK                             (0xF0000000U)
#define CSL_CP_ACE_TRNG_EIP_REV_RESERVED_0_SHIFT                            (0x0000001CU)
#define CSL_CP_ACE_TRNG_EIP_REV_RESERVED_0_MAX                              (0x0000000FU)

/**************************************************************************
* Hardware Region  : SA2_UL PKA control and status registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PKA_APTR;
    volatile uint32_t PKA_BPTR;
    volatile uint32_t PKA_CPTR;
    volatile uint32_t PKA_DPTR;
    volatile uint32_t PKA_ALENGTH;
    volatile uint32_t PKA_BLENGTH;
    volatile uint32_t PKA_SHIFT;
    volatile uint32_t PKA_FUNCTION;
    volatile uint32_t PKA_COMPARE;
    volatile uint32_t PKA_MSW;
    volatile uint32_t PKA_DIVMSW;
    volatile uint8_t  Resv_64[20];
    volatile uint32_t LNME1_STATUS;
    volatile uint32_t LNME1_CONTROL;
    volatile uint8_t  Resv_96[24];
    volatile uint32_t LNME1_NBASE;
    volatile uint32_t LNME1_XBASE;
    volatile uint32_t LNME1_YBASE;
    volatile uint32_t LNME1_BBASE;
    volatile uint32_t LNME1_NACC;
    volatile uint32_t LNME1_NZERO;
    volatile uint8_t  Resv_128[8];
    volatile uint32_t LNME0_STATUS;
    volatile uint32_t LNME0_CONTROL;
    volatile uint32_t LNME_DATAPATH;
    volatile uint8_t  Resv_144[4];
    volatile uint32_t LNME_FAST_CTRL;
    volatile uint32_t LNME_FAST_STRT;
    volatile uint32_t LNME_FAST_MMM;
    volatile uint8_t  Resv_160[4];
    volatile uint32_t LNME0_NBASE;
    volatile uint32_t LNME0_XBASE;
    volatile uint32_t LNME0_YBASE;
    volatile uint32_t LNME0_BBASE;
    volatile uint32_t LNME0_NACC;
    volatile uint32_t LNME0_NZERO;
    volatile uint8_t  Resv_200[16];
    volatile uint32_t PKA_SEQ_CTRL;
    volatile uint8_t  Resv_244[40];
    volatile uint32_t PKA_OPTIONS;
    volatile uint32_t PKA_SW_REV;
    volatile uint32_t PKA_REVISION;
    volatile uint8_t  Resv_1024[768];
    volatile uint32_t GF2M_OPERAND_A_0;
    volatile uint32_t GF2M_OPERAND_A_1;
    volatile uint32_t GF2M_OPERAND_A_2;
    volatile uint32_t GF2M_OPERAND_A_3;
    volatile uint32_t GF2M_OPERAND_A_4;
    volatile uint32_t GF2M_OPERAND_A_5;
    volatile uint32_t GF2M_OPERAND_A_6;
    volatile uint32_t GF2M_OPERAND_A_7;
    volatile uint32_t GF2M_OPERAND_A_8;
    volatile uint32_t GF2M_OPERAND_A_9;
    volatile uint32_t GF2M_OPERAND_A_10;
    volatile uint32_t GF2M_OPERAND_A_11;
    volatile uint32_t GF2M_OPERAND_A_12;
    volatile uint32_t GF2M_OPERAND_A_13;
    volatile uint32_t GF2M_OPERAND_A_14;
    volatile uint32_t GF2M_OPERAND_A_15;
    volatile uint32_t GF2M_OPERAND_A_16;
    volatile uint32_t GF2M_OPERAND_A_17;
    volatile uint8_t  Resv_1152[56];
    volatile uint32_t GF2M_OPERAND_B_0;
    volatile uint32_t GF2M_OPERAND_B_1;
    volatile uint32_t GF2M_OPERAND_B_2;
    volatile uint32_t GF2M_OPERAND_B_3;
    volatile uint32_t GF2M_OPERAND_B_4;
    volatile uint32_t GF2M_OPERAND_B_5;
    volatile uint32_t GF2M_OPERAND_B_6;
    volatile uint32_t GF2M_OPERAND_B_7;
    volatile uint32_t GF2M_OPERAND_B_8;
    volatile uint32_t GF2M_OPERAND_B_9;
    volatile uint32_t GF2M_OPERAND_B_10;
    volatile uint32_t GF2M_OPERAND_B_11;
    volatile uint32_t GF2M_OPERAND_B_12;
    volatile uint32_t GF2M_OPERAND_B_13;
    volatile uint32_t GF2M_OPERAND_B_14;
    volatile uint32_t GF2M_OPERAND_B_15;
    volatile uint32_t GF2M_OPERAND_B_16;
    volatile uint32_t GF2M_OPERAND_B_17;
    volatile uint8_t  Resv_1280[56];
    volatile uint32_t GF2M_OPERAND_C_0;
    volatile uint32_t GF2M_OPERAND_C_1;
    volatile uint32_t GF2M_OPERAND_C_2;
    volatile uint32_t GF2M_OPERAND_C_3;
    volatile uint32_t GF2M_OPERAND_C_4;
    volatile uint32_t GF2M_OPERAND_C_5;
    volatile uint32_t GF2M_OPERAND_C_6;
    volatile uint32_t GF2M_OPERAND_C_7;
    volatile uint32_t GF2M_OPERAND_C_8;
    volatile uint32_t GF2M_OPERAND_C_9;
    volatile uint32_t GF2M_OPERAND_C_10;
    volatile uint32_t GF2M_OPERAND_C_11;
    volatile uint32_t GF2M_OPERAND_C_12;
    volatile uint32_t GF2M_OPERAND_C_13;
    volatile uint32_t GF2M_OPERAND_C_14;
    volatile uint32_t GF2M_OPERAND_C_15;
    volatile uint32_t GF2M_OPERAND_C_16;
    volatile uint32_t GF2M_OPERAND_C_17;
    volatile uint8_t  Resv_1408[56];
    volatile uint32_t GF2M_OPERAND_D_0;
    volatile uint32_t GF2M_OPERAND_D_1;
    volatile uint32_t GF2M_OPERAND_D_2;
    volatile uint32_t GF2M_OPERAND_D_3;
    volatile uint32_t GF2M_OPERAND_D_4;
    volatile uint32_t GF2M_OPERAND_D_5;
    volatile uint32_t GF2M_OPERAND_D_6;
    volatile uint32_t GF2M_OPERAND_D_7;
    volatile uint32_t GF2M_OPERAND_D_8;
    volatile uint32_t GF2M_OPERAND_D_9;
    volatile uint32_t GF2M_OPERAND_D_10;
    volatile uint32_t GF2M_OPERAND_D_11;
    volatile uint32_t GF2M_OPERAND_D_12;
    volatile uint32_t GF2M_OPERAND_D_13;
    volatile uint32_t GF2M_OPERAND_D_14;
    volatile uint32_t GF2M_OPERAND_D_15;
    volatile uint32_t GF2M_OPERAND_D_16;
    volatile uint32_t GF2M_OPERAND_D_17;
    volatile uint8_t  Resv_1536[56];
    volatile uint32_t GF2M_POLYNOMIAL_0;
    volatile uint32_t GF2M_POLYNOMIAL_1;
    volatile uint32_t GF2M_POLYNOMIAL_2;
    volatile uint32_t GF2M_POLYNOMIAL_3;
    volatile uint32_t GF2M_POLYNOMIAL_4;
    volatile uint32_t GF2M_POLYNOMIAL_5;
    volatile uint32_t GF2M_POLYNOMIAL_6;
    volatile uint32_t GF2M_POLYNOMIAL_7;
    volatile uint32_t GF2M_POLYNOMIAL_8;
    volatile uint32_t GF2M_POLYNOMIAL_9;
    volatile uint32_t GF2M_POLYNOMIAL_10;
    volatile uint32_t GF2M_POLYNOMIAL_11;
    volatile uint32_t GF2M_POLYNOMIAL_12;
    volatile uint32_t GF2M_POLYNOMIAL_13;
    volatile uint32_t GF2M_POLYNOMIAL_14;
    volatile uint32_t GF2M_POLYNOMIAL_15;
    volatile uint32_t GF2M_POLYNOMIAL_16;
    volatile uint32_t GF2M_POLYNOMIAL_17;
    volatile uint8_t  Resv_1792[184];
    volatile uint32_t GF2M_CMD;
    volatile uint32_t GF2M_STAT;
    volatile uint32_t GF2M_FIELDSIZE;
    volatile uint8_t  Resv_2040[236];
    volatile uint32_t GF2M_OPTIONS;
    volatile uint32_t GF2M_VERSION;
    volatile uint8_t  Resv_8160[6112];
    volatile uint32_t PKA_REV;
    volatile uint8_t  Resv_8168[4];
    volatile uint32_t PKA_CLK_CTRL;
    volatile uint8_t  Resv_8176[4];
    volatile uint32_t PKA_SYSCONFIG;
    volatile uint32_t PKA_SYSSTATUS;
    union {
        volatile uint32_t PKA_IRQSTATUS;
        volatile uint32_t PKA_IRQCLR;
    } u0;
    volatile uint32_t PKA_IRQENABLE;
} CSL_Cp_ace_pka_eip29t2Regs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_CP_ACE_PKA_EIP29T2_PKA_APTR                                         (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_BPTR                                         (0x00000004U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_CPTR                                         (0x00000008U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_DPTR                                         (0x0000000CU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_ALENGTH                                      (0x00000010U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_BLENGTH                                      (0x00000014U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_SHIFT                                        (0x00000018U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION                                     (0x0000001CU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_COMPARE                                      (0x00000020U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_MSW                                          (0x00000024U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_DIVMSW                                       (0x00000028U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_STATUS                                     (0x00000040U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_CONTROL                                    (0x00000044U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_NBASE                                      (0x00000060U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_XBASE                                      (0x00000064U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_YBASE                                      (0x00000068U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_BBASE                                      (0x0000006CU)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_NACC                                       (0x00000070U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_NZERO                                      (0x00000074U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_STATUS                                     (0x00000080U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_CONTROL                                    (0x00000084U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_DATAPATH                                    (0x00000088U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_CTRL                                   (0x00000090U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_STRT                                   (0x00000094U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_MMM                                    (0x00000098U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_NBASE                                      (0x000000A0U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_XBASE                                      (0x000000A4U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_YBASE                                      (0x000000A8U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_BBASE                                      (0x000000ACU)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_NACC                                       (0x000000B0U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_NZERO                                      (0x000000B4U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_SEQ_CTRL                                     (0x000000C8U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS                                      (0x000000F4U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_SW_REV                                       (0x000000F8U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_REVISION                                     (0x000000FCU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_0                                 (0x00000400U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_1                                 (0x00000404U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_2                                 (0x00000408U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_3                                 (0x0000040CU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_4                                 (0x00000410U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_5                                 (0x00000414U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_6                                 (0x00000418U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_7                                 (0x0000041CU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_8                                 (0x00000420U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_9                                 (0x00000424U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_10                                (0x00000428U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_11                                (0x0000042CU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_12                                (0x00000430U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_13                                (0x00000434U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_14                                (0x00000438U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_15                                (0x0000043CU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_16                                (0x00000440U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_17                                (0x00000444U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_0                                 (0x00000480U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_1                                 (0x00000484U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_2                                 (0x00000488U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_3                                 (0x0000048CU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_4                                 (0x00000490U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_5                                 (0x00000494U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_6                                 (0x00000498U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_7                                 (0x0000049CU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_8                                 (0x000004A0U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_9                                 (0x000004A4U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_10                                (0x000004A8U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_11                                (0x000004ACU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_12                                (0x000004B0U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_13                                (0x000004B4U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_14                                (0x000004B8U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_15                                (0x000004BCU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_16                                (0x000004C0U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_17                                (0x000004C4U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_0                                 (0x00000500U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_1                                 (0x00000504U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_2                                 (0x00000508U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_3                                 (0x0000050CU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_4                                 (0x00000510U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_5                                 (0x00000514U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_6                                 (0x00000518U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_7                                 (0x0000051CU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_8                                 (0x00000520U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_9                                 (0x00000524U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_10                                (0x00000528U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_11                                (0x0000052CU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_12                                (0x00000530U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_13                                (0x00000534U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_14                                (0x00000538U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_15                                (0x0000053CU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_16                                (0x00000540U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_17                                (0x00000544U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_0                                 (0x00000580U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_1                                 (0x00000584U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_2                                 (0x00000588U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_3                                 (0x0000058CU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_4                                 (0x00000590U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_5                                 (0x00000594U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_6                                 (0x00000598U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_7                                 (0x0000059CU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_8                                 (0x000005A0U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_9                                 (0x000005A4U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_10                                (0x000005A8U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_11                                (0x000005ACU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_12                                (0x000005B0U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_13                                (0x000005B4U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_14                                (0x000005B8U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_15                                (0x000005BCU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_16                                (0x000005C0U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_17                                (0x000005C4U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_0                                (0x00000600U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_1                                (0x00000604U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_2                                (0x00000608U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_3                                (0x0000060CU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_4                                (0x00000610U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_5                                (0x00000614U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_6                                (0x00000618U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_7                                (0x0000061CU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_8                                (0x00000620U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_9                                (0x00000624U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_10                               (0x00000628U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_11                               (0x0000062CU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_12                               (0x00000630U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_13                               (0x00000634U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_14                               (0x00000638U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_15                               (0x0000063CU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_16                               (0x00000640U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_17                               (0x00000644U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_CMD                                         (0x00000700U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_STAT                                        (0x00000704U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_FIELDSIZE                                   (0x00000708U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPTIONS                                     (0x000007F8U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_VERSION                                     (0x000007FCU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_REV                                          (0x00001FE0U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_CLK_CTRL                                     (0x00001FE8U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_SYSCONFIG                                    (0x00001FF0U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_SYSSTATUS                                    (0x00001FF4U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQSTATUS                                    (0x00001FF8U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQCLR                                       (0x00001FF8U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQENABLE                                    (0x00001FFCU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PKA_APTR */

#define CSL_CP_ACE_PKA_EIP29T2_PKA_APTR_APTR_MASK                               (0x000007FFU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_APTR_APTR_SHIFT                              (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_APTR_APTR_MAX                                (0x000007FFU)

/* PKA_BPTR */

#define CSL_CP_ACE_PKA_EIP29T2_PKA_BPTR_BPTR_MASK                               (0x000007FFU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_BPTR_BPTR_SHIFT                              (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_BPTR_BPTR_MAX                                (0x000007FFU)

/* PKA_CPTR */

#define CSL_CP_ACE_PKA_EIP29T2_PKA_CPTR_CPTR_MASK                               (0x000007FFU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_CPTR_CPTR_SHIFT                              (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_CPTR_CPTR_MAX                                (0x000007FFU)

/* PKA_DPTR */

#define CSL_CP_ACE_PKA_EIP29T2_PKA_DPTR_DPTR_MASK                               (0x000007FFU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_DPTR_DPTR_SHIFT                              (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_DPTR_DPTR_MAX                                (0x000007FFU)

/* PKA_ALENGTH */

#define CSL_CP_ACE_PKA_EIP29T2_PKA_ALENGTH_ALENGTH_MASK                         (0x000001FFU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_ALENGTH_ALENGTH_SHIFT                        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_ALENGTH_ALENGTH_MAX                          (0x000001FFU)

/* PKA_BLENGTH */

#define CSL_CP_ACE_PKA_EIP29T2_PKA_BLENGTH_BLENGTH_MASK                         (0x000001FFU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_BLENGTH_BLENGTH_SHIFT                        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_BLENGTH_BLENGTH_MAX                          (0x000001FFU)

/* PKA_SHIFT */

#define CSL_CP_ACE_PKA_EIP29T2_PKA_SHIFT_BITS_TO_SHIFT_MASK                     (0x0000001FU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_SHIFT_BITS_TO_SHIFT_SHIFT                    (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_SHIFT_BITS_TO_SHIFT_MAX                      (0x0000001FU)

/* PKA_FUNCTION */

#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_MULTIPLY_MASK                       (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_MULTIPLY_SHIFT                      (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_MULTIPLY_MAX                        (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_ADDSUB_MASK                         (0x00000002U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_ADDSUB_SHIFT                        (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_ADDSUB_MAX                          (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_MS_ONE_MASK                         (0x00000008U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_MS_ONE_SHIFT                        (0x00000003U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_MS_ONE_MAX                          (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_ADD_MASK                            (0x00000010U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_ADD_SHIFT                           (0x00000004U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_ADD_MAX                             (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_SUBTRACT_MASK                       (0x00000020U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_SUBTRACT_SHIFT                      (0x00000005U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_SUBTRACT_MAX                        (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_RSHIFT_MASK                         (0x00000040U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_RSHIFT_SHIFT                        (0x00000006U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_RSHIFT_MAX                          (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_LSHIFT_MASK                         (0x00000080U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_LSHIFT_SHIFT                        (0x00000007U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_LSHIFT_MAX                          (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_DIVIDE_MASK                         (0x00000100U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_DIVIDE_SHIFT                        (0x00000008U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_DIVIDE_MAX                          (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_MODULO_MASK                         (0x00000200U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_MODULO_SHIFT                        (0x00000009U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_MODULO_MAX                          (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_COMPARE_MASK                        (0x00000400U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_COMPARE_SHIFT                       (0x0000000AU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_COMPARE_MAX                         (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_COPY_MASK                           (0x00000800U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_COPY_SHIFT                          (0x0000000BU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_COPY_MAX                            (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_SEQ_OP_MASK                         (0x00007000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_SEQ_OP_SHIFT                        (0x0000000CU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_SEQ_OP_MAX                          (0x00000007U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_RUN_MASK                            (0x00008000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_RUN_SHIFT                           (0x0000000FU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_RUN_MAX                             (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_STALL_RESULT_MASK                   (0x01000000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_STALL_RESULT_SHIFT                  (0x00000018U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_FUNCTION_STALL_RESULT_MAX                    (0x00000001U)

/* PKA_COMPARE */

#define CSL_CP_ACE_PKA_EIP29T2_PKA_COMPARE_A_EQUAL_B_MASK                       (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_COMPARE_A_EQUAL_B_SHIFT                      (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_COMPARE_A_EQUAL_B_MAX                        (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_COMPARE_A_LESS_THAN_B_MASK                   (0x00000002U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_COMPARE_A_LESS_THAN_B_SHIFT                  (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_COMPARE_A_LESS_THAN_B_MAX                    (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_COMPARE_A_GREATER_THAN_B_MASK                (0x00000004U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_COMPARE_A_GREATER_THAN_B_SHIFT               (0x00000002U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_COMPARE_A_GREATER_THAN_B_MAX                 (0x00000001U)

/* PKA_MSW */

#define CSL_CP_ACE_PKA_EIP29T2_PKA_MSW_MSW_ADDRESS_MASK                         (0x000007FFU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_MSW_MSW_ADDRESS_SHIFT                        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_MSW_MSW_ADDRESS_MAX                          (0x000007FFU)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_MSW_RESULT_IS_ZERO_MASK                      (0x00008000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_MSW_RESULT_IS_ZERO_SHIFT                     (0x0000000FU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_MSW_RESULT_IS_ZERO_MAX                       (0x00000001U)

/* PKA_DIVMSW */

#define CSL_CP_ACE_PKA_EIP29T2_PKA_DIVMSW_MSW_ADDRESS_MASK                      (0x000007FFU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_DIVMSW_MSW_ADDRESS_SHIFT                     (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_DIVMSW_MSW_ADDRESS_MAX                       (0x000007FFU)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_DIVMSW_RESULT_IS_ZERO_MASK                   (0x00008000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_DIVMSW_RESULT_IS_ZERO_SHIFT                  (0x0000000FU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_DIVMSW_RESULT_IS_ZERO_MAX                    (0x00000001U)

/* LNME1_STATUS */

#define CSL_CP_ACE_PKA_EIP29T2_LNME1_STATUS_OVERFLOW_MASK                       (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_STATUS_OVERFLOW_SHIFT                      (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_STATUS_OVERFLOW_MAX                        (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME1_STATUS_MMM_BUSY_MASK                       (0x00000002U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_STATUS_MMM_BUSY_SHIFT                      (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_STATUS_MMM_BUSY_MAX                        (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME1_STATUS_CMD_ERROR_MASK                      (0x00000004U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_STATUS_CMD_ERROR_SHIFT                     (0x00000002U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_STATUS_CMD_ERROR_MAX                       (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME1_STATUS_RESULT_ZERO_MASK                    (0x00000008U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_STATUS_RESULT_ZERO_SHIFT                   (0x00000003U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_STATUS_RESULT_ZERO_MAX                     (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME1_STATUS_STICKY_OFLO_MASK                    (0x00000010U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_STATUS_STICKY_OFLO_SHIFT                   (0x00000004U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_STATUS_STICKY_OFLO_MAX                     (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME1_STATUS_STICKY_ZERO_MASK                    (0x00000020U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_STATUS_STICKY_ZERO_SHIFT                   (0x00000005U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_STATUS_STICKY_ZERO_MAX                     (0x00000001U)

/* LNME1_CONTROL */

#define CSL_CP_ACE_PKA_EIP29T2_LNME1_CONTROL_MMM_CMD_MASK                       (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_CONTROL_MMM_CMD_SHIFT                      (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_CONTROL_MMM_CMD_MAX                        (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME1_CONTROL_MMMNEXT_CMD_MASK                   (0x00000002U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_CONTROL_MMMNEXT_CMD_SHIFT                  (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_CONTROL_MMMNEXT_CMD_MAX                    (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME1_CONTROL_EXP_CMD_MASK                       (0x00000004U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_CONTROL_EXP_CMD_SHIFT                      (0x00000002U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_CONTROL_EXP_CMD_MAX                        (0x00000001U)

/* LNME1_NBASE */

#define CSL_CP_ACE_PKA_EIP29T2_LNME1_NBASE_ZERO_MASK                            (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_NBASE_ZERO_SHIFT                           (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_NBASE_ZERO_MAX                             (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME1_NBASE_NBASE_MASK                           (0x000007FEU)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_NBASE_NBASE_SHIFT                          (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_NBASE_NBASE_MAX                            (0x000003FFU)

#define CSL_CP_ACE_PKA_EIP29T2_LNME1_NBASE_NYDIGITS_MASK                        (0x03FF0000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_NBASE_NYDIGITS_SHIFT                       (0x00000010U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_NBASE_NYDIGITS_MAX                         (0x000003FFU)

/* LNME1_XBASE */

#define CSL_CP_ACE_PKA_EIP29T2_LNME1_XBASE_ZERO_MASK                            (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_XBASE_ZERO_SHIFT                           (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_XBASE_ZERO_MAX                             (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME1_XBASE_XBASE_MASK                           (0x000007FEU)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_XBASE_XBASE_SHIFT                          (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_XBASE_XBASE_MAX                            (0x000003FFU)

#define CSL_CP_ACE_PKA_EIP29T2_LNME1_XBASE_XDIGITS_MASK                         (0x03FF0000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_XBASE_XDIGITS_SHIFT                        (0x00000010U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_XBASE_XDIGITS_MAX                          (0x000003FFU)

/* LNME1_YBASE */

#define CSL_CP_ACE_PKA_EIP29T2_LNME1_YBASE_ZERO_MASK                            (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_YBASE_ZERO_SHIFT                           (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_YBASE_ZERO_MAX                             (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME1_YBASE_YBASE_MASK                           (0x000007FEU)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_YBASE_YBASE_SHIFT                          (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_YBASE_YBASE_MAX                            (0x000003FFU)

#define CSL_CP_ACE_PKA_EIP29T2_LNME1_YBASE_NPASSES_MASK                         (0x00FF0000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_YBASE_NPASSES_SHIFT                        (0x00000010U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_YBASE_NPASSES_MAX                          (0x000000FFU)

/* LNME1_BBASE */

#define CSL_CP_ACE_PKA_EIP29T2_LNME1_BBASE_ZERO_MASK                            (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_BBASE_ZERO_SHIFT                           (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_BBASE_ZERO_MAX                             (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME1_BBASE_BBASE_MASK                           (0x000007FEU)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_BBASE_BBASE_SHIFT                          (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_BBASE_BBASE_MAX                            (0x000003FFU)

#define CSL_CP_ACE_PKA_EIP29T2_LNME1_BBASE_BCNTR_MASK                           (0x7FFF0000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_BBASE_BCNTR_SHIFT                          (0x00000010U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_BBASE_BCNTR_MAX                            (0x00007FFFU)

/* LNME1_NACC */

#define CSL_CP_ACE_PKA_EIP29T2_LNME1_NACC_NACC_MASK                             (0x000000FFU)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_NACC_NACC_SHIFT                            (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_NACC_NACC_MAX                              (0x000000FFU)

#define CSL_CP_ACE_PKA_EIP29T2_LNME1_NACC_NACC_BUSY_MASK                        (0x00000100U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_NACC_NACC_BUSY_SHIFT                       (0x00000008U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_NACC_NACC_BUSY_MAX                         (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME1_NACC_EXPARRAY_MASK                         (0x001F0000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_NACC_EXPARRAY_SHIFT                        (0x00000010U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_NACC_EXPARRAY_MAX                          (0x0000001FU)

/* LNME1_NZERO */

#define CSL_CP_ACE_PKA_EIP29T2_LNME1_NZERO_NZERO_MASK                           (0x000000FFU)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_NZERO_NZERO_SHIFT                          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME1_NZERO_NZERO_MAX                            (0x000000FFU)

/* LNME0_STATUS */

#define CSL_CP_ACE_PKA_EIP29T2_LNME0_STATUS_OVERFLOW_MASK                       (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_STATUS_OVERFLOW_SHIFT                      (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_STATUS_OVERFLOW_MAX                        (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME0_STATUS_MMM_BUSY_MASK                       (0x00000002U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_STATUS_MMM_BUSY_SHIFT                      (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_STATUS_MMM_BUSY_MAX                        (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME0_STATUS_CMD_ERROR_MASK                      (0x00000004U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_STATUS_CMD_ERROR_SHIFT                     (0x00000002U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_STATUS_CMD_ERROR_MAX                       (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME0_STATUS_RESULT_ZERO_MASK                    (0x00000008U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_STATUS_RESULT_ZERO_SHIFT                   (0x00000003U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_STATUS_RESULT_ZERO_MAX                     (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME0_STATUS_STICKY_OFLO_MASK                    (0x00000010U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_STATUS_STICKY_OFLO_SHIFT                   (0x00000004U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_STATUS_STICKY_OFLO_MAX                     (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME0_STATUS_STICKY_ZERO_MASK                    (0x00000020U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_STATUS_STICKY_ZERO_SHIFT                   (0x00000005U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_STATUS_STICKY_ZERO_MAX                     (0x00000001U)

/* LNME0_CONTROL */

#define CSL_CP_ACE_PKA_EIP29T2_LNME0_CONTROL_MMM_CMD_MASK                       (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_CONTROL_MMM_CMD_SHIFT                      (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_CONTROL_MMM_CMD_MAX                        (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME0_CONTROL_MMMNEXT_CMD_MASK                   (0x00000002U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_CONTROL_MMMNEXT_CMD_SHIFT                  (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_CONTROL_MMMNEXT_CMD_MAX                    (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME0_CONTROL_EXP_CMD_MASK                       (0x00000004U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_CONTROL_EXP_CMD_SHIFT                      (0x00000002U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_CONTROL_EXP_CMD_MAX                        (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME0_CONTROL_RESET_CMD_MASK                     (0x00000020U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_CONTROL_RESET_CMD_SHIFT                    (0x00000005U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_CONTROL_RESET_CMD_MAX                      (0x00000001U)

/* LNME_DATAPATH */

#define CSL_CP_ACE_PKA_EIP29T2_LNME_DATAPATH_BYPASS_0_MASK                      (0x00000007U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_DATAPATH_BYPASS_0_SHIFT                     (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_DATAPATH_BYPASS_0_MAX                       (0x00000007U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME_DATAPATH_BYPASS_1_MASK                      (0x00000700U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_DATAPATH_BYPASS_1_SHIFT                     (0x00000008U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_DATAPATH_BYPASS_1_MAX                       (0x00000007U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME_DATAPATH_LINKUP_MASK                        (0x00008000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_DATAPATH_LINKUP_SHIFT                       (0x0000000FU)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_DATAPATH_LINKUP_MAX                         (0x00000001U)

/* LNME_FAST_CTRL */

#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_CTRL_XOR_CTRL_MASK                     (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_CTRL_XOR_CTRL_SHIFT                    (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_CTRL_XOR_CTRL_MAX                      (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_CTRL_IGNORED_MASK                      (0x0000FFFEU)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_CTRL_IGNORED_SHIFT                     (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_CTRL_IGNORED_MAX                       (0x00007FFFU)

#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_CTRL_CMP_MASK_MASK                     (0x001F0000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_CTRL_CMP_MASK_SHIFT                    (0x00000010U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_CTRL_CMP_MASK_MAX                      (0x0000001FU)

#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_CTRL_CMP_VALUE_MASK                    (0x03E00000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_CTRL_CMP_VALUE_SHIFT                   (0x00000015U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_CTRL_CMP_VALUE_MAX                     (0x0000001FU)

#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_CTRL_XOR_VALUE_MASK                    (0x7C000000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_CTRL_XOR_VALUE_SHIFT                   (0x0000001AU)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_CTRL_XOR_VALUE_MAX                     (0x0000001FU)

#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_CTRL_UPDATE_MASK                       (0x80000000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_CTRL_UPDATE_SHIFT                      (0x0000001FU)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_CTRL_UPDATE_MAX                        (0x00000001U)

/* LNME_FAST_STRT */

#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_STRT_PKCP_REQUEST_MASK                 (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_STRT_PKCP_REQUEST_SHIFT                (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_STRT_PKCP_REQUEST_MAX                  (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_STRT_LNME0_REQUEST_MASK                (0x00000002U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_STRT_LNME0_REQUEST_SHIFT               (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_STRT_LNME0_REQUEST_MAX                 (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_STRT_LNME1_REQUEST_MASK                (0x00000004U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_STRT_LNME1_REQUEST_SHIFT               (0x00000002U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_STRT_LNME1_REQUEST_MAX                 (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_STRT_ADDSUB_A_MASK                     (0x001F0000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_STRT_ADDSUB_A_SHIFT                    (0x00000010U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_STRT_ADDSUB_A_MAX                      (0x0000001FU)

#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_STRT_ADDSUB_BC_MASK                    (0x03E00000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_STRT_ADDSUB_BC_SHIFT                   (0x00000015U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_STRT_ADDSUB_BC_MAX                     (0x0000001FU)

#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_STRT_ADDSUB_D_MASK                     (0x7C000000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_STRT_ADDSUB_D_SHIFT                    (0x0000001AU)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_STRT_ADDSUB_D_MAX                      (0x0000001FU)

#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_STRT_REQUEST_SUB_MASK                  (0x80000000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_STRT_REQUEST_SUB_SHIFT                 (0x0000001FU)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_STRT_REQUEST_SUB_MAX                   (0x00000001U)

/* LNME_FAST_MMM */

#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_MMM_XINDEX_0_MASK                      (0x0000001FU)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_MMM_XINDEX_0_SHIFT                     (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_MMM_XINDEX_0_MAX                       (0x0000001FU)

#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_MMM_YINDEX_0_MASK                      (0x000003E0U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_MMM_YINDEX_0_SHIFT                     (0x00000005U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_MMM_YINDEX_0_MAX                       (0x0000001FU)

#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_MMM_RINDEX_0_MASK                      (0x00007C00U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_MMM_RINDEX_0_SHIFT                     (0x0000000AU)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_MMM_RINDEX_0_MAX                       (0x0000001FU)

#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_MMM_STICKY_0_MASK                      (0x00008000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_MMM_STICKY_0_SHIFT                     (0x0000000FU)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_MMM_STICKY_0_MAX                       (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_MMM_XINDEX_1_MASK                      (0x001F0000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_MMM_XINDEX_1_SHIFT                     (0x00000010U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_MMM_XINDEX_1_MAX                       (0x0000001FU)

#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_MMM_YINDEX_1_MASK                      (0x03E00000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_MMM_YINDEX_1_SHIFT                     (0x00000015U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_MMM_YINDEX_1_MAX                       (0x0000001FU)

#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_MMM_RINDEX_1_MASK                      (0x7C000000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_MMM_RINDEX_1_SHIFT                     (0x0000001AU)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_MMM_RINDEX_1_MAX                       (0x0000001FU)

#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_MMM_STICKY_1_MASK                      (0x80000000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_MMM_STICKY_1_SHIFT                     (0x0000001FU)
#define CSL_CP_ACE_PKA_EIP29T2_LNME_FAST_MMM_STICKY_1_MAX                       (0x00000001U)

/* LNME0_NBASE */

#define CSL_CP_ACE_PKA_EIP29T2_LNME0_NBASE_ZERO_MASK                            (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_NBASE_ZERO_SHIFT                           (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_NBASE_ZERO_MAX                             (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME0_NBASE_NBASE_MASK                           (0x000007FEU)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_NBASE_NBASE_SHIFT                          (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_NBASE_NBASE_MAX                            (0x000003FFU)

#define CSL_CP_ACE_PKA_EIP29T2_LNME0_NBASE_NYDIGITS_MASK                        (0x03FF0000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_NBASE_NYDIGITS_SHIFT                       (0x00000010U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_NBASE_NYDIGITS_MAX                         (0x000003FFU)

/* LNME0_XBASE */

#define CSL_CP_ACE_PKA_EIP29T2_LNME0_XBASE_ZERO_MASK                            (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_XBASE_ZERO_SHIFT                           (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_XBASE_ZERO_MAX                             (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME0_XBASE_XBASE_MASK                           (0x000007FEU)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_XBASE_XBASE_SHIFT                          (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_XBASE_XBASE_MAX                            (0x000003FFU)

#define CSL_CP_ACE_PKA_EIP29T2_LNME0_XBASE_XDIGITS_MASK                         (0x03FF0000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_XBASE_XDIGITS_SHIFT                        (0x00000010U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_XBASE_XDIGITS_MAX                          (0x000003FFU)

/* LNME0_YBASE */

#define CSL_CP_ACE_PKA_EIP29T2_LNME0_YBASE_ZERO_MASK                            (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_YBASE_ZERO_SHIFT                           (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_YBASE_ZERO_MAX                             (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME0_YBASE_YBASE_MASK                           (0x000007FEU)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_YBASE_YBASE_SHIFT                          (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_YBASE_YBASE_MAX                            (0x000003FFU)

#define CSL_CP_ACE_PKA_EIP29T2_LNME0_YBASE_NPASSES_MASK                         (0x00FF0000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_YBASE_NPASSES_SHIFT                        (0x00000010U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_YBASE_NPASSES_MAX                          (0x000000FFU)

/* LNME0_BBASE */

#define CSL_CP_ACE_PKA_EIP29T2_LNME0_BBASE_ZERO_MASK                            (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_BBASE_ZERO_SHIFT                           (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_BBASE_ZERO_MAX                             (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME0_BBASE_BBASE_MASK                           (0x000007FEU)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_BBASE_BBASE_SHIFT                          (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_BBASE_BBASE_MAX                            (0x000003FFU)

#define CSL_CP_ACE_PKA_EIP29T2_LNME0_BBASE_BCNTR_MASK                           (0x7FFF0000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_BBASE_BCNTR_SHIFT                          (0x00000010U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_BBASE_BCNTR_MAX                            (0x00007FFFU)

/* LNME0_NACC */

#define CSL_CP_ACE_PKA_EIP29T2_LNME0_NACC_NACC_MASK                             (0x000000FFU)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_NACC_NACC_SHIFT                            (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_NACC_NACC_MAX                              (0x000000FFU)

#define CSL_CP_ACE_PKA_EIP29T2_LNME0_NACC_NACC_BUSY_MASK                        (0x00000100U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_NACC_NACC_BUSY_SHIFT                       (0x00000008U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_NACC_NACC_BUSY_MAX                         (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_LNME0_NACC_EXPARRAY_MASK                         (0x001F0000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_NACC_EXPARRAY_SHIFT                        (0x00000010U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_NACC_EXPARRAY_MAX                          (0x0000001FU)

/* LNME0_NZERO */

#define CSL_CP_ACE_PKA_EIP29T2_LNME0_NZERO_NZERO_MASK                           (0x000000FFU)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_NZERO_NZERO_SHIFT                          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_LNME0_NZERO_NZERO_MAX                            (0x000000FFU)

/* PKA_SEQ_CTRL */

#define CSL_CP_ACE_PKA_EIP29T2_PKA_SEQ_CTRL_SW_TRIGGERS_MASK                    (0x000000FFU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_SEQ_CTRL_SW_TRIGGERS_SHIFT                   (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_SEQ_CTRL_SW_TRIGGERS_MAX                     (0x000000FFU)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_SEQ_CTRL_SEQ_STATUS_MASK                     (0x0000FF00U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_SEQ_CTRL_SEQ_STATUS_SHIFT                    (0x00000008U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_SEQ_CTRL_SEQ_STATUS_MAX                      (0x000000FFU)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_SEQ_CTRL_RESET_MASK                          (0x80000000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_SEQ_CTRL_RESET_SHIFT                         (0x0000001FU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_SEQ_CTRL_RESET_MAX                           (0x00000001U)

/* PKA_OPTIONS */

#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_PKCP_CONFIG_MASK                     (0x00000003U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_PKCP_CONFIG_SHIFT                    (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_PKCP_CONFIG_MAX                      (0x00000003U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_LNME_CONFIG_MASK                     (0x0000001CU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_LNME_CONFIG_SHIFT                    (0x00000002U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_LNME_CONFIG_MAX                      (0x00000007U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_SEQ_CONFIG_MASK                      (0x00000060U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_SEQ_CONFIG_SHIFT                     (0x00000005U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_SEQ_CONFIG_MAX                       (0x00000003U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_PROGRAM_RAM_MASK                     (0x00000080U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_PROGRAM_RAM_SHIFT                    (0x00000007U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_PROGRAM_RAM_MAX                      (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_PROT_OPTION_MASK                     (0x00000700U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_PROT_OPTION_SHIFT                    (0x00000008U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_PROT_OPTION_MAX                      (0x00000007U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_INTERRUPT_MASK_MASK                  (0x00000800U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_INTERRUPT_MASK_SHIFT                 (0x0000000BU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_INTERRUPT_MASK_MAX                   (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_MMM3A_MASK                           (0x00001000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_MMM3A_SHIFT                          (0x0000000CU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_MMM3A_MAX                            (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_ZEROIZATION_MASK                     (0x00002000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_ZEROIZATION_SHIFT                    (0x0000000DU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_ZEROIZATION_MAX                      (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_LNME_BYPASS_MASK                     (0x00004000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_LNME_BYPASS_SHIFT                    (0x0000000EU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_LNME_BYPASS_MAX                      (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_LNME_PES_MASK                        (0x003F0000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_LNME_PES_SHIFT                       (0x00000010U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_LNME_PES_MAX                         (0x0000003FU)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_GF2M_CONFIG_MASK                     (0x00C00000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_GF2M_CONFIG_SHIFT                    (0x00000016U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_GF2M_CONFIG_MAX                      (0x00000003U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_LNME_FIFO_DEPT_MASK                  (0xFF000000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_LNME_FIFO_DEPT_SHIFT                 (0x00000018U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_OPTIONS_LNME_FIFO_DEPT_MAX                   (0x000000FFU)

/* PKA_SW_REV */

#define CSL_CP_ACE_PKA_EIP29T2_PKA_SW_REV_SW_PATCH_LEVEL_MASK                   (0x000F0000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_SW_REV_SW_PATCH_LEVEL_SHIFT                  (0x00000010U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_SW_REV_SW_PATCH_LEVEL_MAX                    (0x0000000FU)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_SW_REV_SW_MINOR_REV_MASK                     (0x00F00000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_SW_REV_SW_MINOR_REV_SHIFT                    (0x00000014U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_SW_REV_SW_MINOR_REV_MAX                      (0x0000000FU)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_SW_REV_SW_MAJOR_REV_MASK                     (0x0F000000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_SW_REV_SW_MAJOR_REV_SHIFT                    (0x00000018U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_SW_REV_SW_MAJOR_REV_MAX                      (0x0000000FU)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_SW_REV_SW_CAPABILITIES_MASK                  (0xF0000000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_SW_REV_SW_CAPABILITIES_SHIFT                 (0x0000001CU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_SW_REV_SW_CAPABILITIES_MAX                   (0x0000000FU)

/* PKA_REVISION */

#define CSL_CP_ACE_PKA_EIP29T2_PKA_REVISION_EIP_NR_MASK                         (0x000000FFU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_REVISION_EIP_NR_SHIFT                        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_REVISION_EIP_NR_MAX                          (0x000000FFU)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_REVISION_EIP_NR_COMPL_MASK                   (0x0000FF00U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_REVISION_EIP_NR_COMPL_SHIFT                  (0x00000008U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_REVISION_EIP_NR_COMPL_MAX                    (0x000000FFU)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_REVISION_HW_PATCH_LEVEL_MASK                 (0x000F0000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_REVISION_HW_PATCH_LEVEL_SHIFT                (0x00000010U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_REVISION_HW_PATCH_LEVEL_MAX                  (0x0000000FU)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_REVISION_HW_MINOR_REV_MASK                   (0x00F00000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_REVISION_HW_MINOR_REV_SHIFT                  (0x00000014U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_REVISION_HW_MINOR_REV_MAX                    (0x0000000FU)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_REVISION_HW_MAJOR_REV_MASK                   (0x0F000000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_REVISION_HW_MAJOR_REV_SHIFT                  (0x00000018U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_REVISION_HW_MAJOR_REV_MAX                    (0x0000000FU)

/* GF2M_OPERAND_A_0 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_0_GF2M_OPERAND_A_0_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_0_GF2M_OPERAND_A_0_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_0_GF2M_OPERAND_A_0_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_A_1 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_1_GF2M_OPERAND_A_1_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_1_GF2M_OPERAND_A_1_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_1_GF2M_OPERAND_A_1_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_A_2 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_2_GF2M_OPERAND_A_2_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_2_GF2M_OPERAND_A_2_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_2_GF2M_OPERAND_A_2_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_A_3 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_3_GF2M_OPERAND_A_3_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_3_GF2M_OPERAND_A_3_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_3_GF2M_OPERAND_A_3_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_A_4 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_4_GF2M_OPERAND_A_4_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_4_GF2M_OPERAND_A_4_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_4_GF2M_OPERAND_A_4_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_A_5 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_5_GF2M_OPERAND_A_5_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_5_GF2M_OPERAND_A_5_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_5_GF2M_OPERAND_A_5_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_A_6 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_6_GF2M_OPERAND_A_6_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_6_GF2M_OPERAND_A_6_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_6_GF2M_OPERAND_A_6_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_A_7 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_7_GF2M_OPERAND_A_7_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_7_GF2M_OPERAND_A_7_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_7_GF2M_OPERAND_A_7_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_A_8 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_8_GF2M_OPERAND_A_8_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_8_GF2M_OPERAND_A_8_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_8_GF2M_OPERAND_A_8_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_A_9 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_9_GF2M_OPERAND_A_9_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_9_GF2M_OPERAND_A_9_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_9_GF2M_OPERAND_A_9_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_A_10 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_10_GF2M_OPERAND_A_10_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_10_GF2M_OPERAND_A_10_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_10_GF2M_OPERAND_A_10_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_A_11 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_11_GF2M_OPERAND_A_11_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_11_GF2M_OPERAND_A_11_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_11_GF2M_OPERAND_A_11_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_A_12 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_12_GF2M_OPERAND_A_12_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_12_GF2M_OPERAND_A_12_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_12_GF2M_OPERAND_A_12_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_A_13 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_13_GF2M_OPERAND_A_13_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_13_GF2M_OPERAND_A_13_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_13_GF2M_OPERAND_A_13_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_A_14 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_14_GF2M_OPERAND_A_14_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_14_GF2M_OPERAND_A_14_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_14_GF2M_OPERAND_A_14_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_A_15 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_15_GF2M_OPERAND_A_15_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_15_GF2M_OPERAND_A_15_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_15_GF2M_OPERAND_A_15_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_A_16 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_16_GF2M_OPERAND_A_16_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_16_GF2M_OPERAND_A_16_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_16_GF2M_OPERAND_A_16_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_A_17 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_17_OPERAND_A_MASK                 (0x0FFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_17_OPERAND_A_SHIFT                (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_A_17_OPERAND_A_MAX                  (0x0FFFFFFFU)

/* GF2M_OPERAND_B_0 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_0_GF2M_OPERAND_B_0_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_0_GF2M_OPERAND_B_0_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_0_GF2M_OPERAND_B_0_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_B_1 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_1_GF2M_OPERAND_B_1_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_1_GF2M_OPERAND_B_1_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_1_GF2M_OPERAND_B_1_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_B_2 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_2_GF2M_OPERAND_B_2_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_2_GF2M_OPERAND_B_2_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_2_GF2M_OPERAND_B_2_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_B_3 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_3_GF2M_OPERAND_B_3_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_3_GF2M_OPERAND_B_3_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_3_GF2M_OPERAND_B_3_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_B_4 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_4_GF2M_OPERAND_B_4_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_4_GF2M_OPERAND_B_4_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_4_GF2M_OPERAND_B_4_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_B_5 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_5_GF2M_OPERAND_B_5_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_5_GF2M_OPERAND_B_5_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_5_GF2M_OPERAND_B_5_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_B_6 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_6_GF2M_OPERAND_B_6_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_6_GF2M_OPERAND_B_6_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_6_GF2M_OPERAND_B_6_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_B_7 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_7_GF2M_OPERAND_B_7_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_7_GF2M_OPERAND_B_7_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_7_GF2M_OPERAND_B_7_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_B_8 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_8_GF2M_OPERAND_B_8_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_8_GF2M_OPERAND_B_8_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_8_GF2M_OPERAND_B_8_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_B_9 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_9_GF2M_OPERAND_B_9_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_9_GF2M_OPERAND_B_9_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_9_GF2M_OPERAND_B_9_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_B_10 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_10_GF2M_OPERAND_B_10_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_10_GF2M_OPERAND_B_10_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_10_GF2M_OPERAND_B_10_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_B_11 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_11_GF2M_OPERAND_B_11_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_11_GF2M_OPERAND_B_11_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_11_GF2M_OPERAND_B_11_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_B_12 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_12_GF2M_OPERAND_B_12_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_12_GF2M_OPERAND_B_12_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_12_GF2M_OPERAND_B_12_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_B_13 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_13_GF2M_OPERAND_B_13_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_13_GF2M_OPERAND_B_13_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_13_GF2M_OPERAND_B_13_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_B_14 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_14_GF2M_OPERAND_B_14_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_14_GF2M_OPERAND_B_14_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_14_GF2M_OPERAND_B_14_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_B_15 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_15_GF2M_OPERAND_B_15_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_15_GF2M_OPERAND_B_15_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_15_GF2M_OPERAND_B_15_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_B_16 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_16_GF2M_OPERAND_B_16_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_16_GF2M_OPERAND_B_16_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_16_GF2M_OPERAND_B_16_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_B_17 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_17_OPERAND_B_MASK                 (0x0FFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_17_OPERAND_B_SHIFT                (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_B_17_OPERAND_B_MAX                  (0x0FFFFFFFU)

/* GF2M_OPERAND_C_0 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_0_GF2M_OPERAND_C_0_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_0_GF2M_OPERAND_C_0_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_0_GF2M_OPERAND_C_0_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_C_1 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_1_GF2M_OPERAND_C_1_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_1_GF2M_OPERAND_C_1_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_1_GF2M_OPERAND_C_1_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_C_2 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_2_GF2M_OPERAND_C_2_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_2_GF2M_OPERAND_C_2_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_2_GF2M_OPERAND_C_2_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_C_3 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_3_GF2M_OPERAND_C_3_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_3_GF2M_OPERAND_C_3_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_3_GF2M_OPERAND_C_3_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_C_4 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_4_GF2M_OPERAND_C_4_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_4_GF2M_OPERAND_C_4_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_4_GF2M_OPERAND_C_4_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_C_5 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_5_GF2M_OPERAND_C_5_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_5_GF2M_OPERAND_C_5_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_5_GF2M_OPERAND_C_5_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_C_6 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_6_GF2M_OPERAND_C_6_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_6_GF2M_OPERAND_C_6_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_6_GF2M_OPERAND_C_6_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_C_7 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_7_GF2M_OPERAND_C_7_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_7_GF2M_OPERAND_C_7_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_7_GF2M_OPERAND_C_7_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_C_8 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_8_GF2M_OPERAND_C_8_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_8_GF2M_OPERAND_C_8_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_8_GF2M_OPERAND_C_8_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_C_9 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_9_GF2M_OPERAND_C_9_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_9_GF2M_OPERAND_C_9_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_9_GF2M_OPERAND_C_9_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_C_10 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_10_GF2M_OPERAND_C_10_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_10_GF2M_OPERAND_C_10_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_10_GF2M_OPERAND_C_10_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_C_11 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_11_GF2M_OPERAND_C_11_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_11_GF2M_OPERAND_C_11_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_11_GF2M_OPERAND_C_11_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_C_12 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_12_GF2M_OPERAND_C_12_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_12_GF2M_OPERAND_C_12_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_12_GF2M_OPERAND_C_12_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_C_13 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_13_GF2M_OPERAND_C_13_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_13_GF2M_OPERAND_C_13_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_13_GF2M_OPERAND_C_13_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_C_14 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_14_GF2M_OPERAND_C_14_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_14_GF2M_OPERAND_C_14_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_14_GF2M_OPERAND_C_14_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_C_15 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_15_GF2M_OPERAND_C_15_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_15_GF2M_OPERAND_C_15_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_15_GF2M_OPERAND_C_15_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_C_16 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_16_GF2M_OPERAND_C_16_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_16_GF2M_OPERAND_C_16_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_16_GF2M_OPERAND_C_16_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_C_17 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_17_OPERAND_C_MASK                 (0x0FFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_17_OPERAND_C_SHIFT                (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_C_17_OPERAND_C_MAX                  (0x0FFFFFFFU)

/* GF2M_OPERAND_D_0 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_0_GF2M_OPERAND_D_0_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_0_GF2M_OPERAND_D_0_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_0_GF2M_OPERAND_D_0_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_D_1 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_1_GF2M_OPERAND_D_1_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_1_GF2M_OPERAND_D_1_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_1_GF2M_OPERAND_D_1_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_D_2 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_2_GF2M_OPERAND_D_2_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_2_GF2M_OPERAND_D_2_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_2_GF2M_OPERAND_D_2_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_D_3 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_3_GF2M_OPERAND_D_3_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_3_GF2M_OPERAND_D_3_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_3_GF2M_OPERAND_D_3_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_D_4 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_4_GF2M_OPERAND_D_4_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_4_GF2M_OPERAND_D_4_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_4_GF2M_OPERAND_D_4_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_D_5 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_5_GF2M_OPERAND_D_5_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_5_GF2M_OPERAND_D_5_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_5_GF2M_OPERAND_D_5_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_D_6 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_6_GF2M_OPERAND_D_6_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_6_GF2M_OPERAND_D_6_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_6_GF2M_OPERAND_D_6_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_D_7 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_7_GF2M_OPERAND_D_7_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_7_GF2M_OPERAND_D_7_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_7_GF2M_OPERAND_D_7_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_D_8 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_8_GF2M_OPERAND_D_8_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_8_GF2M_OPERAND_D_8_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_8_GF2M_OPERAND_D_8_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_D_9 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_9_GF2M_OPERAND_D_9_MASK           (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_9_GF2M_OPERAND_D_9_SHIFT          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_9_GF2M_OPERAND_D_9_MAX            (0xFFFFFFFFU)

/* GF2M_OPERAND_D_10 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_10_GF2M_OPERAND_D_10_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_10_GF2M_OPERAND_D_10_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_10_GF2M_OPERAND_D_10_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_D_11 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_11_GF2M_OPERAND_D_11_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_11_GF2M_OPERAND_D_11_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_11_GF2M_OPERAND_D_11_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_D_12 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_12_GF2M_OPERAND_D_12_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_12_GF2M_OPERAND_D_12_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_12_GF2M_OPERAND_D_12_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_D_13 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_13_GF2M_OPERAND_D_13_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_13_GF2M_OPERAND_D_13_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_13_GF2M_OPERAND_D_13_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_D_14 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_14_GF2M_OPERAND_D_14_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_14_GF2M_OPERAND_D_14_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_14_GF2M_OPERAND_D_14_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_D_15 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_15_GF2M_OPERAND_D_15_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_15_GF2M_OPERAND_D_15_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_15_GF2M_OPERAND_D_15_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_D_16 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_16_GF2M_OPERAND_D_16_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_16_GF2M_OPERAND_D_16_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_16_GF2M_OPERAND_D_16_MAX          (0xFFFFFFFFU)

/* GF2M_OPERAND_D_17 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_17_OPERAND_D_MASK                 (0x0FFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_17_OPERAND_D_SHIFT                (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPERAND_D_17_OPERAND_D_MAX                  (0x0FFFFFFFU)

/* GF2M_POLYNOMIAL_0 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_0_GF2M_POLYNOMIAL_0_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_0_GF2M_POLYNOMIAL_0_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_0_GF2M_POLYNOMIAL_0_MAX          (0xFFFFFFFFU)

/* GF2M_POLYNOMIAL_1 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_1_GF2M_POLYNOMIAL_1_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_1_GF2M_POLYNOMIAL_1_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_1_GF2M_POLYNOMIAL_1_MAX          (0xFFFFFFFFU)

/* GF2M_POLYNOMIAL_2 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_2_GF2M_POLYNOMIAL_2_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_2_GF2M_POLYNOMIAL_2_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_2_GF2M_POLYNOMIAL_2_MAX          (0xFFFFFFFFU)

/* GF2M_POLYNOMIAL_3 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_3_GF2M_POLYNOMIAL_3_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_3_GF2M_POLYNOMIAL_3_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_3_GF2M_POLYNOMIAL_3_MAX          (0xFFFFFFFFU)

/* GF2M_POLYNOMIAL_4 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_4_GF2M_POLYNOMIAL_4_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_4_GF2M_POLYNOMIAL_4_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_4_GF2M_POLYNOMIAL_4_MAX          (0xFFFFFFFFU)

/* GF2M_POLYNOMIAL_5 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_5_GF2M_POLYNOMIAL_5_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_5_GF2M_POLYNOMIAL_5_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_5_GF2M_POLYNOMIAL_5_MAX          (0xFFFFFFFFU)

/* GF2M_POLYNOMIAL_6 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_6_GF2M_POLYNOMIAL_6_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_6_GF2M_POLYNOMIAL_6_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_6_GF2M_POLYNOMIAL_6_MAX          (0xFFFFFFFFU)

/* GF2M_POLYNOMIAL_7 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_7_GF2M_POLYNOMIAL_7_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_7_GF2M_POLYNOMIAL_7_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_7_GF2M_POLYNOMIAL_7_MAX          (0xFFFFFFFFU)

/* GF2M_POLYNOMIAL_8 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_8_GF2M_POLYNOMIAL_8_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_8_GF2M_POLYNOMIAL_8_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_8_GF2M_POLYNOMIAL_8_MAX          (0xFFFFFFFFU)

/* GF2M_POLYNOMIAL_9 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_9_GF2M_POLYNOMIAL_9_MASK         (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_9_GF2M_POLYNOMIAL_9_SHIFT        (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_9_GF2M_POLYNOMIAL_9_MAX          (0xFFFFFFFFU)

/* GF2M_POLYNOMIAL_10 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_10_GF2M_POLYNOMIAL_10_MASK       (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_10_GF2M_POLYNOMIAL_10_SHIFT      (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_10_GF2M_POLYNOMIAL_10_MAX        (0xFFFFFFFFU)

/* GF2M_POLYNOMIAL_11 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_11_GF2M_POLYNOMIAL_11_MASK       (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_11_GF2M_POLYNOMIAL_11_SHIFT      (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_11_GF2M_POLYNOMIAL_11_MAX        (0xFFFFFFFFU)

/* GF2M_POLYNOMIAL_12 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_12_GF2M_POLYNOMIAL_12_MASK       (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_12_GF2M_POLYNOMIAL_12_SHIFT      (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_12_GF2M_POLYNOMIAL_12_MAX        (0xFFFFFFFFU)

/* GF2M_POLYNOMIAL_13 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_13_GF2M_POLYNOMIAL_13_MASK       (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_13_GF2M_POLYNOMIAL_13_SHIFT      (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_13_GF2M_POLYNOMIAL_13_MAX        (0xFFFFFFFFU)

/* GF2M_POLYNOMIAL_14 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_14_GF2M_POLYNOMIAL_14_MASK       (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_14_GF2M_POLYNOMIAL_14_SHIFT      (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_14_GF2M_POLYNOMIAL_14_MAX        (0xFFFFFFFFU)

/* GF2M_POLYNOMIAL_15 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_15_GF2M_POLYNOMIAL_15_MASK       (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_15_GF2M_POLYNOMIAL_15_SHIFT      (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_15_GF2M_POLYNOMIAL_15_MAX        (0xFFFFFFFFU)

/* GF2M_POLYNOMIAL_16 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_16_GF2M_POLYNOMIAL_16_MASK       (0xFFFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_16_GF2M_POLYNOMIAL_16_SHIFT      (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_16_GF2M_POLYNOMIAL_16_MAX        (0xFFFFFFFFU)

/* GF2M_POLYNOMIAL_17 */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_17_POLYNOMIAL_MASK               (0x0FFFFFFFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_17_POLYNOMIAL_SHIFT              (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_POLYNOMIAL_17_POLYNOMIAL_MAX                (0x0FFFFFFFU)

/* GF2M_CMD */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_CMD_CMD_SUBMIT_CMD_BUF_FULL_MASK            (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_CMD_CMD_SUBMIT_CMD_BUF_FULL_SHIFT           (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_CMD_CMD_SUBMIT_CMD_BUF_FULL_MAX             (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_CMD_OPCODE_MASK                             (0x0000000EU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_CMD_OPCODE_SHIFT                            (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_CMD_OPCODE_MAX                              (0x00000007U)

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_CMD_SRC0_MASK                               (0x00000180U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_CMD_SRC0_SHIFT                              (0x00000007U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_CMD_SRC0_MAX                                (0x00000003U)

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_CMD_SRC1_MASK                               (0x00000C00U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_CMD_SRC1_SHIFT                              (0x0000000AU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_CMD_SRC1_MAX                                (0x00000003U)

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_CMD_TGT_MASK                                (0x00006000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_CMD_TGT_SHIFT                               (0x0000000DU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_CMD_TGT_MAX                                 (0x00000003U)

/* GF2M_STAT */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_STAT_MSB_PTR_MASK                           (0x0000001FU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_STAT_MSB_PTR_SHIFT                          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_STAT_MSB_PTR_MAX                            (0x0000001FU)

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_STAT_NO_MSB_MASK                            (0x00000020U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_STAT_NO_MSB_SHIFT                           (0x00000005U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_STAT_NO_MSB_MAX                             (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_STAT_CMD_ERR_MASK                           (0x00008000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_STAT_CMD_ERR_SHIFT                          (0x0000000FU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_STAT_CMD_ERR_MAX                            (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_STAT_SHIFT_VALUE_MASK                       (0x03FF0000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_STAT_SHIFT_VALUE_SHIFT                      (0x00000010U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_STAT_SHIFT_VALUE_MAX                        (0x000003FFU)

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_STAT_BUSY_MASK                              (0x80000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_STAT_BUSY_SHIFT                             (0x0000001FU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_STAT_BUSY_MAX                               (0x00000001U)

/* GF2M_FIELDSIZE */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_FIELDSIZE_FIELD_SIZE_MASK                   (0x000000FFU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_FIELDSIZE_FIELD_SIZE_SHIFT                  (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_FIELDSIZE_FIELD_SIZE_MAX                    (0x000000FFU)

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_FIELDSIZE_OP_SHIFT_MASK                     (0x00030000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_FIELDSIZE_OP_SHIFT_SHIFT                    (0x00000010U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_FIELDSIZE_OP_SHIFT_MAX                      (0x00000003U)

/* GF2M_OPTIONS */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPTIONS_OPERANDS_MASK                       (0x0000000FU)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPTIONS_OPERANDS_SHIFT                      (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPTIONS_OPERANDS_MAX                        (0x0000000FU)

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPTIONS_MUL_DEPTH_MASK                      (0x000000F0U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPTIONS_MUL_DEPTH_SHIFT                     (0x00000004U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPTIONS_MUL_DEPTH_MAX                       (0x0000000FU)

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPTIONS_OPERAND_SIZE_MASK                   (0x0FFF0000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPTIONS_OPERAND_SIZE_SHIFT                  (0x00000010U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_OPTIONS_OPERAND_SIZE_MAX                    (0x00000FFFU)

/* GF2M_VERSION */

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_VERSION_PATCH_LEVEL_MASK                    (0x000F0000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_VERSION_PATCH_LEVEL_SHIFT                   (0x00000010U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_VERSION_PATCH_LEVEL_MAX                     (0x0000000FU)

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_VERSION_MINOR_VERSION_MASK                  (0x00F00000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_VERSION_MINOR_VERSION_SHIFT                 (0x00000014U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_VERSION_MINOR_VERSION_MAX                   (0x0000000FU)

#define CSL_CP_ACE_PKA_EIP29T2_GF2M_VERSION_MAJOR_VERSION_MASK                  (0x0F000000U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_VERSION_MAJOR_VERSION_SHIFT                 (0x00000018U)
#define CSL_CP_ACE_PKA_EIP29T2_GF2M_VERSION_MAJOR_VERSION_MAX                   (0x0000000FU)

/* PKA_REV */

#define CSL_CP_ACE_PKA_EIP29T2_PKA_REV_REV_MINOR_MASK                           (0x0000000FU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_REV_REV_MINOR_SHIFT                          (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_REV_REV_MINOR_MAX                            (0x0000000FU)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_REV_REV_MAJOR_MASK                           (0x000000F0U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_REV_REV_MAJOR_SHIFT                          (0x00000004U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_REV_REV_MAJOR_MAX                            (0x0000000FU)

/* PKA_CLK_CTRL */

#define CSL_CP_ACE_PKA_EIP29T2_PKA_CLK_CTRL_CLK_FORCE_ON_MASK                   (0x0000007FU)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_CLK_CTRL_CLK_FORCE_ON_SHIFT                  (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_CLK_CTRL_CLK_FORCE_ON_MAX                    (0x0000007FU)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_CLK_CTRL_CLK_FORCE_OFF_MASK                  (0x00007F00U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_CLK_CTRL_CLK_FORCE_OFF_SHIFT                 (0x00000008U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_CLK_CTRL_CLK_FORCE_OFF_MAX                   (0x0000007FU)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_CLK_CTRL_CLK_EN_STATUS_MASK                  (0x007F0000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_CLK_CTRL_CLK_EN_STATUS_SHIFT                 (0x00000010U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_CLK_CTRL_CLK_EN_STATUS_MAX                   (0x0000007FU)

/* PKA_SYSCONFIG */

#define CSL_CP_ACE_PKA_EIP29T2_PKA_SYSCONFIG_SOFTRESET_MASK                     (0x00000002U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_SYSCONFIG_SOFTRESET_SHIFT                    (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_SYSCONFIG_SOFTRESET_MAX                      (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_SYSCONFIG_IDLEMODE_MASK                      (0x00000030U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_SYSCONFIG_IDLEMODE_SHIFT                     (0x00000004U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_SYSCONFIG_IDLEMODE_MAX                       (0x00000003U)

/* PKA_SYSSTATUS */

#define CSL_CP_ACE_PKA_EIP29T2_PKA_SYSSTATUS_RESETDONE_MASK                     (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_SYSSTATUS_RESETDONE_SHIFT                    (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_SYSSTATUS_RESETDONE_MAX                      (0x00000001U)

/* PKA_IRQSTATUS */

#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQSTATUS_PKAIRQSTAT_MASK                    (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQSTATUS_PKAIRQSTAT_SHIFT                   (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQSTATUS_PKAIRQSTAT_MAX                     (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQSTATUS_LNMEIRQSTAT_MASK                   (0x00000002U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQSTATUS_LNMEIRQSTAT_SHIFT                  (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQSTATUS_LNMEIRQSTAT_MAX                    (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQSTATUS_GF2MIRQSTAT_MASK                   (0x00000004U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQSTATUS_GF2MIRQSTAT_SHIFT                  (0x00000002U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQSTATUS_GF2MIRQSTAT_MAX                    (0x00000001U)

/* PKA_IRQCLR */

#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQCLR_PKAIRQCLR_MASK                        (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQCLR_PKAIRQCLR_SHIFT                       (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQCLR_PKAIRQCLR_MAX                         (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQCLR_LNMEIRQCLR_MASK                       (0x00000002U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQCLR_LNMEIRQCLR_SHIFT                      (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQCLR_LNMEIRQCLR_MAX                        (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQCLR_GF2MIRQCLR_MASK                       (0x00000004U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQCLR_GF2MIRQCLR_SHIFT                      (0x00000002U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQCLR_GF2MIRQCLR_MAX                        (0x00000001U)

/* PKA_IRQENABLE */

#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQENABLE_PKAIRQEN_MASK                      (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQENABLE_PKAIRQEN_SHIFT                     (0x00000000U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQENABLE_PKAIRQEN_MAX                       (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQENABLE_LNMEIRQEN_MASK                     (0x00000002U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQENABLE_LNMEIRQEN_SHIFT                    (0x00000001U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQENABLE_LNMEIRQEN_MAX                      (0x00000001U)

#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQENABLE_GF2MIRQEN_MASK                     (0x00000004U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQENABLE_GF2MIRQEN_SHIFT                    (0x00000002U)
#define CSL_CP_ACE_PKA_EIP29T2_PKA_IRQENABLE_GF2MIRQEN_MAX                      (0x00000001U)

/**************************************************************************
* Register Overlay Structure
**************************************************************************/
typedef struct {
    CSL_Cp_aceMmrRegs        MMR;
    volatile uint8_t         RSVD0[3824];
    CSL_Cp_aceUpdatesRegs    UPDATES;
    volatile uint8_t         RSVD1[61144];
    CSL_Cp_aceTrngRegs       TRNG;
    volatile uint8_t         RSVD2[65408];
    CSL_Eip_29t2_ramRegs     PKA;
} CSL_Cp_aceRegs;

#ifdef __cplusplus
}
#endif
#endif /* CSLR_CP_ACE_H_ */
