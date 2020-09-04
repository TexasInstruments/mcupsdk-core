/********************************************************************
 * Copyright (C) 2019 Texas Instruments Incorporated.
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
 *  Name        : cslr_sec_proxy.h
*/
#ifndef CSLR_SEC_PROXY_H_
#define CSLR_SEC_PROXY_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  : Sec Proxy Config
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PID;                       /* Revision Register */
    volatile uint32_t CONFIG;                    /* Config Register */
    volatile uint8_t  Resv_20[12];
    volatile uint32_t GLB_EVT;                   /* Global Event Register */
} CSL_sec_proxyRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_SEC_PROXY_PID                                                      (0x00000000U)
#define CSL_SEC_PROXY_CONFIG                                                   (0x00000004U)
#define CSL_SEC_PROXY_GLB_EVT                                                  (0x00000014U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_SEC_PROXY_PID_MINOR_MASK                                           (0x0000003FU)
#define CSL_SEC_PROXY_PID_MINOR_SHIFT                                          (0x00000000U)
#define CSL_SEC_PROXY_PID_MINOR_MAX                                            (0x0000003FU)

#define CSL_SEC_PROXY_PID_CUSTOM_MASK                                          (0x000000C0U)
#define CSL_SEC_PROXY_PID_CUSTOM_SHIFT                                         (0x00000006U)
#define CSL_SEC_PROXY_PID_CUSTOM_MAX                                           (0x00000003U)

#define CSL_SEC_PROXY_PID_MAJOR_MASK                                           (0x00000700U)
#define CSL_SEC_PROXY_PID_MAJOR_SHIFT                                          (0x00000008U)
#define CSL_SEC_PROXY_PID_MAJOR_MAX                                            (0x00000007U)

#define CSL_SEC_PROXY_PID_RTL_MASK                                             (0x0000F800U)
#define CSL_SEC_PROXY_PID_RTL_SHIFT                                            (0x0000000BU)
#define CSL_SEC_PROXY_PID_RTL_MAX                                              (0x0000001FU)

#define CSL_SEC_PROXY_PID_FUNC_MASK                                            (0x0FFF0000U)
#define CSL_SEC_PROXY_PID_FUNC_SHIFT                                           (0x00000010U)
#define CSL_SEC_PROXY_PID_FUNC_MAX                                             (0x00000FFFU)

#define CSL_SEC_PROXY_PID_BU_MASK                                              (0x30000000U)
#define CSL_SEC_PROXY_PID_BU_SHIFT                                             (0x0000001CU)
#define CSL_SEC_PROXY_PID_BU_MAX                                               (0x00000003U)

#define CSL_SEC_PROXY_PID_SCHEME_MASK                                          (0xC0000000U)
#define CSL_SEC_PROXY_PID_SCHEME_SHIFT                                         (0x0000001EU)
#define CSL_SEC_PROXY_PID_SCHEME_MAX                                           (0x00000003U)

/* CONFIG */

#define CSL_SEC_PROXY_CONFIG_MSG_SIZE_MASK                                     (0xFFFF0000U)
#define CSL_SEC_PROXY_CONFIG_MSG_SIZE_SHIFT                                    (0x00000010U)
#define CSL_SEC_PROXY_CONFIG_MSG_SIZE_MAX                                      (0x0000FFFFU)

#define CSL_SEC_PROXY_CONFIG_THREADS_MASK                                      (0x0000FFFFU)
#define CSL_SEC_PROXY_CONFIG_THREADS_SHIFT                                     (0x00000000U)
#define CSL_SEC_PROXY_CONFIG_THREADS_MAX                                       (0x0000FFFFU)

/* GLB_EVT */

#define CSL_SEC_PROXY_GLB_EVT_ERR_EVENT_MASK                                   (0x0000FFFFU)
#define CSL_SEC_PROXY_GLB_EVT_ERR_EVENT_SHIFT                                  (0x00000000U)
#define CSL_SEC_PROXY_GLB_EVT_ERR_EVENT_MAX                                    (0x0000FFFFU)

/**************************************************************************
* Hardware Region  : Sec Proxy Secure Config
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t CTL;                       /* Control Register */
    volatile uint32_t EVT_MAP;                   /* Event Map Register */
    volatile uint32_t DST;                       /* Destination Register */
    volatile uint8_t  Resv_4096[4084];
} CSL_sec_proxy_scfgRegs_thread;


typedef struct {
    volatile uint32_t BUFFER_L;                  /* Buffer Register */
    volatile uint32_t BUFFER_H;                  /* Buffer Register */
    volatile uint32_t TARGET_L;                  /* Target Register */
    volatile uint32_t TARGET_H;                  /* Target Register */
    volatile uint32_t ORDERID;                   /* Buffer OrderID Register */
    volatile uint8_t  Resv_4096[4076];
    CSL_sec_proxy_scfgRegs_thread THREAD[1024];
} CSL_sec_proxy_scfgRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_SEC_PROXY_SCFG_BUFFER_L                                            (0x00000000U)
#define CSL_SEC_PROXY_SCFG_BUFFER_H                                            (0x00000004U)
#define CSL_SEC_PROXY_SCFG_TARGET_L                                            (0x00000008U)
#define CSL_SEC_PROXY_SCFG_TARGET_H                                            (0x0000000CU)
#define CSL_SEC_PROXY_SCFG_ORDERID                                             (0x00000010U)
#define CSL_SEC_PROXY_SCFG_THREAD_CTL(THREAD)                                  (0x00001000U+((THREAD)*0x1000U))
#define CSL_SEC_PROXY_SCFG_THREAD_EVT_MAP(THREAD)                              (0x00001004U+((THREAD)*0x1000U))
#define CSL_SEC_PROXY_SCFG_THREAD_DST(THREAD)                                  (0x00001008U+((THREAD)*0x1000U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* CTL */

#define CSL_SEC_PROXY_SCFG_THREAD_CTL_DIR_MASK                                 (0x80000000U)
#define CSL_SEC_PROXY_SCFG_THREAD_CTL_DIR_SHIFT                                (0x0000001FU)
#define CSL_SEC_PROXY_SCFG_THREAD_CTL_DIR_MAX                                  (0x00000001U)

#define CSL_SEC_PROXY_SCFG_THREAD_CTL_MAX_CNT_MASK                             (0x00FF0000U)
#define CSL_SEC_PROXY_SCFG_THREAD_CTL_MAX_CNT_SHIFT                            (0x00000010U)
#define CSL_SEC_PROXY_SCFG_THREAD_CTL_MAX_CNT_MAX                              (0x000000FFU)

#define CSL_SEC_PROXY_SCFG_THREAD_CTL_QUEUE_MASK                               (0x0000FFFFU)
#define CSL_SEC_PROXY_SCFG_THREAD_CTL_QUEUE_SHIFT                              (0x00000000U)
#define CSL_SEC_PROXY_SCFG_THREAD_CTL_QUEUE_MAX                                (0x0000FFFFU)

/* EVT_MAP */

#define CSL_SEC_PROXY_SCFG_THREAD_EVT_MAP_ERR_EVT_MASK                         (0xFFFF0000U)
#define CSL_SEC_PROXY_SCFG_THREAD_EVT_MAP_ERR_EVT_SHIFT                        (0x00000010U)
#define CSL_SEC_PROXY_SCFG_THREAD_EVT_MAP_ERR_EVT_MAX                          (0x0000FFFFU)

#define CSL_SEC_PROXY_SCFG_THREAD_EVT_MAP_THR_EVT_MASK                         (0x0000FFFFU)
#define CSL_SEC_PROXY_SCFG_THREAD_EVT_MAP_THR_EVT_SHIFT                        (0x00000000U)
#define CSL_SEC_PROXY_SCFG_THREAD_EVT_MAP_THR_EVT_MAX                          (0x0000FFFFU)

/* DST */

#define CSL_SEC_PROXY_SCFG_THREAD_DST_THREAD_MASK                              (0x0000FFFFU)
#define CSL_SEC_PROXY_SCFG_THREAD_DST_THREAD_SHIFT                             (0x00000000U)
#define CSL_SEC_PROXY_SCFG_THREAD_DST_THREAD_MAX                               (0x0000FFFFU)

/* BUFFER_L */

#define CSL_SEC_PROXY_SCFG_BUFFER_L_BASE_L_MASK                                (0xFFFFFFFFU)
#define CSL_SEC_PROXY_SCFG_BUFFER_L_BASE_L_SHIFT                               (0x00000000U)
#define CSL_SEC_PROXY_SCFG_BUFFER_L_BASE_L_MAX                                 (0xFFFFFFFFU)

/* BUFFER_H */

#define CSL_SEC_PROXY_SCFG_BUFFER_H_BASE_H_MASK                                (0x0000FFFFU)
#define CSL_SEC_PROXY_SCFG_BUFFER_H_BASE_H_SHIFT                               (0x00000000U)
#define CSL_SEC_PROXY_SCFG_BUFFER_H_BASE_H_MAX                                 (0x0000FFFFU)

/* TARGET_L */

#define CSL_SEC_PROXY_SCFG_TARGET_L_BASE_L_MASK                                (0xFFFFFFFFU)
#define CSL_SEC_PROXY_SCFG_TARGET_L_BASE_L_SHIFT                               (0x00000000U)
#define CSL_SEC_PROXY_SCFG_TARGET_L_BASE_L_MAX                                 (0xFFFFFFFFU)

/* TARGET_H */

#define CSL_SEC_PROXY_SCFG_TARGET_H_BASE_H_MASK                                (0x0000FFFFU)
#define CSL_SEC_PROXY_SCFG_TARGET_H_BASE_H_SHIFT                               (0x00000000U)
#define CSL_SEC_PROXY_SCFG_TARGET_H_BASE_H_MAX                                 (0x0000FFFFU)

/* ORDERID */

#define CSL_SEC_PROXY_SCFG_ORDERID_REPLACE_MASK                                (0x00000010U)
#define CSL_SEC_PROXY_SCFG_ORDERID_REPLACE_SHIFT                               (0x00000004U)
#define CSL_SEC_PROXY_SCFG_ORDERID_REPLACE_MAX                                 (0x00000001U)

#define CSL_SEC_PROXY_SCFG_ORDERID_ORDERID_MASK                                (0x0000000FU)
#define CSL_SEC_PROXY_SCFG_ORDERID_ORDERID_SHIFT                               (0x00000000U)
#define CSL_SEC_PROXY_SCFG_ORDERID_ORDERID_MAX                                 (0x0000000FU)

/**************************************************************************
* Hardware Region  : Sec Proxy Realtime
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t STATUS;                    /* Status Register */
    volatile uint32_t THR;                       /* Threshold Register */
    volatile uint8_t  Resv_4096[4088];
} CSL_sec_proxy_rtRegs_thread;


typedef struct {
    CSL_sec_proxy_rtRegs_thread THREAD[1024];
} CSL_sec_proxy_rtRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_SEC_PROXY_RT_THREAD_STATUS(THREAD)                                 (0x00000000U+((THREAD)*0x1000U))
#define CSL_SEC_PROXY_RT_THREAD_THR(THREAD)                                    (0x00000004U+((THREAD)*0x1000U))

/**************************************************************************
* 64K Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t STATUS;                    /* Status Register */
    volatile uint32_t THR;                       /* Threshold Register */
    volatile uint8_t  Resv_65536[65528];
} CSL_a64_sec_proxy_rtRegs_thread;


typedef struct {
    CSL_a64_sec_proxy_rtRegs_thread THREAD[1024];
} CSL_a64_sec_proxy_rtRegs;


/**************************************************************************
* 64K Register Macros
**************************************************************************/

#define CSL_A64_SEC_PROXY_RT_THREAD_STATUS(THREAD)                             (0x00000000U+((THREAD)*0x10000U))
#define CSL_A64_SEC_PROXY_RT_THREAD_THR(THREAD)                                (0x00000004U+((THREAD)*0x10000U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* STATUS */

#define CSL_SEC_PROXY_RT_THREAD_STATUS_ERROR_MASK                              (0x80000000U)
#define CSL_SEC_PROXY_RT_THREAD_STATUS_ERROR_SHIFT                             (0x0000001FU)
#define CSL_SEC_PROXY_RT_THREAD_STATUS_ERROR_MAX                               (0x00000001U)

#define CSL_SEC_PROXY_RT_THREAD_STATUS_DIR_MASK                                (0x40000000U)
#define CSL_SEC_PROXY_RT_THREAD_STATUS_DIR_SHIFT                               (0x0000001EU)
#define CSL_SEC_PROXY_RT_THREAD_STATUS_DIR_MAX                                 (0x00000001U)

#define CSL_SEC_PROXY_RT_THREAD_STATUS_MAX_CNT_MASK                            (0x00FF0000U)
#define CSL_SEC_PROXY_RT_THREAD_STATUS_MAX_CNT_SHIFT                           (0x00000010U)
#define CSL_SEC_PROXY_RT_THREAD_STATUS_MAX_CNT_MAX                             (0x000000FFU)

#define CSL_SEC_PROXY_RT_THREAD_STATUS_CUR_CNT_MASK                            (0x000000FFU)
#define CSL_SEC_PROXY_RT_THREAD_STATUS_CUR_CNT_SHIFT                           (0x00000000U)
#define CSL_SEC_PROXY_RT_THREAD_STATUS_CUR_CNT_MAX                             (0x000000FFU)

/* THR */

#define CSL_SEC_PROXY_RT_THREAD_THR_THR_CNT_MASK                               (0x000000FFU)
#define CSL_SEC_PROXY_RT_THREAD_THR_THR_CNT_SHIFT                              (0x00000000U)
#define CSL_SEC_PROXY_RT_THREAD_THR_THR_CNT_MAX                                (0x000000FFU)

#ifdef __cplusplus
}
#endif
#endif
