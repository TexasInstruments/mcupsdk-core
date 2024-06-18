/********************************************************************
 * Copyright (C) 2024 Texas Instruments Incorporated
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
 *  Name        : cslr_proxy.h
*/
#ifndef CSLR_PROXY_H_
#define CSLR_PROXY_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  : Proxy Global Config
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PID;                       /* Revision Register */
    volatile uint32_t CONFIG;                    /* Config Register */
    volatile uint8_t  Resv_20[12];
    volatile uint32_t GLB_EVT;                   /* Global Event Register */
} CSL_proxyRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_PROXY_PID                                                          (0x00000000U)
#define CSL_PROXY_CONFIG                                                       (0x00000004U)
#define CSL_PROXY_GLB_EVT                                                      (0x00000014U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_PROXY_PID_MINOR_MASK                                               (0x0000003FU)
#define CSL_PROXY_PID_MINOR_SHIFT                                              (0x00000000U)
#define CSL_PROXY_PID_MINOR_MAX                                                (0x0000003FU)

#define CSL_PROXY_PID_CUSTOM_MASK                                              (0x000000C0U)
#define CSL_PROXY_PID_CUSTOM_SHIFT                                             (0x00000006U)
#define CSL_PROXY_PID_CUSTOM_MAX                                               (0x00000003U)

#define CSL_PROXY_PID_MAJOR_MASK                                               (0x00000700U)
#define CSL_PROXY_PID_MAJOR_SHIFT                                              (0x00000008U)
#define CSL_PROXY_PID_MAJOR_MAX                                                (0x00000007U)

#define CSL_PROXY_PID_RTL_MASK                                                 (0x0000F800U)
#define CSL_PROXY_PID_RTL_SHIFT                                                (0x0000000BU)
#define CSL_PROXY_PID_RTL_MAX                                                  (0x0000001FU)

#define CSL_PROXY_PID_FUNC_MASK                                                (0x0FFF0000U)
#define CSL_PROXY_PID_FUNC_SHIFT                                               (0x00000010U)
#define CSL_PROXY_PID_FUNC_MAX                                                 (0x00000FFFU)

#define CSL_PROXY_PID_BU_MASK                                                  (0x30000000U)
#define CSL_PROXY_PID_BU_SHIFT                                                 (0x0000001CU)
#define CSL_PROXY_PID_BU_MAX                                                   (0x00000003U)

#define CSL_PROXY_PID_SCHEME_MASK                                              (0xC0000000U)
#define CSL_PROXY_PID_SCHEME_SHIFT                                             (0x0000001EU)
#define CSL_PROXY_PID_SCHEME_MAX                                               (0x00000003U)

/* CONFIG */

#define CSL_PROXY_CONFIG_THREADS_MASK                                          (0x0000FFFFU)
#define CSL_PROXY_CONFIG_THREADS_SHIFT                                         (0x00000000U)
#define CSL_PROXY_CONFIG_THREADS_MAX                                           (0x0000FFFFU)

/* GLB_EVT */

#define CSL_PROXY_GLB_EVT_ERR_EVENT_MASK                                       (0x0000FFFFU)
#define CSL_PROXY_GLB_EVT_ERR_EVENT_SHIFT                                      (0x00000000U)
#define CSL_PROXY_GLB_EVT_ERR_EVENT_MAX                                        (0x0000FFFFU)

/**************************************************************************
* Hardware Region  : Proxy Config
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t EVT_REG;                   /* Proxy Event Register */
    volatile uint8_t  Resv_4096[4092];
} CSL_proxy_cfgRegs_PROXY;


typedef struct {
    CSL_proxy_cfgRegs_PROXY PROXY[1024];
} CSL_proxy_cfgRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_PROXY_CFG_PROXY_EVT_REG(PROXY)                                     (0x00000000U+((PROXY)*0x1000U))

/**************************************************************************
* 64K Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t EVT_REG;                   /* Proxy Event Register */
    volatile uint8_t  Resv_65536[65532];
} CSL_a64_proxy_cfgRegs_PROXY;


typedef struct {
    CSL_a64_proxy_cfgRegs_PROXY PROXY[1024];
} CSL_a64_proxy_cfgRegs;


/**************************************************************************
* 64K Register Macros
**************************************************************************/

#define CSL_A64_PROXY_CFG_PROXY_EVT_REG(PROXY)                                 (0x00000000U+((PROXY)*0x10000U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* EVT_REG */

#define CSL_PROXY_CFG_PROXY_EVT_REG_ERR_EVENT_MASK                             (0x0000FFFFU)
#define CSL_PROXY_CFG_PROXY_EVT_REG_ERR_EVENT_SHIFT                            (0x00000000U)
#define CSL_PROXY_CFG_PROXY_EVT_REG_ERR_EVENT_MAX                              (0x0000FFFFU)

/**************************************************************************
* Hardware Region  : Proxy Buffer RAM Debug
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t DATA[131072];              /* Proxy Buffer Register */
} CSL_proxy_bufferRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_PROXY_BUFFER_DATA(DATA)                                            (0x00000000U+((DATA)*0x4U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* DATA */

#define CSL_PROXY_BUFFER_DATA_VAL_MASK                                         (0xFFFFFFFFU)
#define CSL_PROXY_BUFFER_DATA_VAL_SHIFT                                        (0x00000000U)
#define CSL_PROXY_BUFFER_DATA_VAL_MAX                                          (0xFFFFFFFFU)

/**************************************************************************
* Hardware Region  : Proxy Datapath
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t CTL;                       /* Proxy Control Register */
    volatile uint32_t STATUS;                    /* Proxy Status Register */
    volatile uint8_t  Resv_512[504];
    volatile uint32_t DATA[128];                 /* Proxy Data Register */
    volatile uint8_t  Resv_4096[3072];
} CSL_proxy_target0Regs_PROXY;


typedef struct {
    CSL_proxy_target0Regs_PROXY PROXY[1024];
} CSL_proxy_target0Regs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_PROXY_TARGET0_PROXY_CTL(PROXY)                                     (0x00000000U+((PROXY)*0x1000U))
#define CSL_PROXY_TARGET0_PROXY_STATUS(PROXY)                                  (0x00000004U+((PROXY)*0x1000U))
#define CSL_PROXY_TARGET0_PROXY_DATA(PROXY,DATA)                               (0x00000200U+((PROXY)*0x1000U)+((DATA)*0x4U))

/**************************************************************************
* 64K Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t CTL;                       /* Proxy Control Register */
    volatile uint32_t STATUS;                    /* Proxy Status Register */
    volatile uint8_t  Resv_512[504];
    volatile uint32_t DATA[128];                 /* Proxy Data Register */
    volatile uint8_t  Resv_65536[64512];
} CSL_a64_proxy_target0Regs_PROXY;


typedef struct {
    CSL_a64_proxy_target0Regs_PROXY PROXY[1024];
} CSL_a64_proxy_target0Regs;


/**************************************************************************
* 64K Register Macros
**************************************************************************/

#define CSL_A64_PROXY_TARGET0_PROXY_CTL(PROXY)                                 (0x00000000U+((PROXY)*0x10000U))
#define CSL_A64_PROXY_TARGET0_PROXY_STATUS(PROXY)                              (0x00000004U+((PROXY)*0x10000U))
#define CSL_A64_PROXY_TARGET0_PROXY_DATA(PROXY,DATA)                           (0x00000200U+((PROXY)*0x10000U)+((DATA)*0x4U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* CTL */

#define CSL_PROXY_TARGET0_PROXY_CTL_ELSIZE_MASK                                (0x07000000U)
#define CSL_PROXY_TARGET0_PROXY_CTL_ELSIZE_SHIFT                               (0x00000018U)
#define CSL_PROXY_TARGET0_PROXY_CTL_ELSIZE_MAX                                 (0x00000007U)

#define CSL_PROXY_TARGET0_PROXY_CTL_MODE_MASK                                  (0x00030000U)
#define CSL_PROXY_TARGET0_PROXY_CTL_MODE_SHIFT                                 (0x00000010U)
#define CSL_PROXY_TARGET0_PROXY_CTL_MODE_MAX                                   (0x00000003U)

#define CSL_PROXY_TARGET0_PROXY_CTL_MODE_VAL_HEAD                              (0x0U)
#define CSL_PROXY_TARGET0_PROXY_CTL_MODE_VAL_TAIL                              (0x1U)
#define CSL_PROXY_TARGET0_PROXY_CTL_MODE_VAL_PEEK_HEAD                         (0x2U)
#define CSL_PROXY_TARGET0_PROXY_CTL_MODE_VAL_PEEK_TAIL                         (0x3U)

#define CSL_PROXY_TARGET0_PROXY_CTL_QUEUE_MASK                                 (0x0000FFFFU)
#define CSL_PROXY_TARGET0_PROXY_CTL_QUEUE_SHIFT                                (0x00000000U)
#define CSL_PROXY_TARGET0_PROXY_CTL_QUEUE_MAX                                  (0x0000FFFFU)

/* STATUS */

#define CSL_PROXY_TARGET0_PROXY_STATUS_ERROR_MASK                              (0x80000000U)
#define CSL_PROXY_TARGET0_PROXY_STATUS_ERROR_SHIFT                             (0x0000001FU)
#define CSL_PROXY_TARGET0_PROXY_STATUS_ERROR_MAX                               (0x00000001U)

/* DATA */

#define CSL_PROXY_TARGET0_PROXY_DATA_VAL_MASK                                  (0xFFFFFFFFU)
#define CSL_PROXY_TARGET0_PROXY_DATA_VAL_SHIFT                                 (0x00000000U)
#define CSL_PROXY_TARGET0_PROXY_DATA_VAL_MAX                                   (0xFFFFFFFFU)

#ifdef __cplusplus
}
#endif
#endif
