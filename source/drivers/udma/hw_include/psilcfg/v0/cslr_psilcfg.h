/********************************************************************
 * Copyright (C) 2024 Texas Instruments Incorporated.
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
 *  Name        : cslr_psilcfg.h
*/
#ifndef CSLR_PSILCFG_H_
#define CSLR_PSILCFG_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  : PSI-L Configuration Proxy Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t REVISION;                  /* Revision Register */
    volatile uint8_t  Resv_16[12];
    volatile uint32_t PSIL_TO;                   /* PSI-L Proxy Timeout Register */
    volatile uint8_t  Resv_256[236];
    volatile uint32_t CMDA;                      /* PSI-L Configuration Proxy Command Register A */
    volatile uint32_t CMDB;                      /* PSI-L Configuration Proxy Command Register B */
    volatile uint32_t WDATA;                     /* PSI-L Configuration Proxy Write Data Register */
    volatile uint8_t  Resv_320[52];
    volatile uint32_t RDATA;                     /* PSI-L Configuration Proxy Read Data Register */
} CSL_psilcfgRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_PSILCFG_REVISION                                                   (0x00000000U)
#define CSL_PSILCFG_PSIL_TO                                                    (0x00000010U)
#define CSL_PSILCFG_CMDA                                                       (0x00000100U)
#define CSL_PSILCFG_CMDB                                                       (0x00000104U)
#define CSL_PSILCFG_WDATA                                                      (0x00000108U)
#define CSL_PSILCFG_RDATA                                                      (0x00000140U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* REVISION */

#define CSL_PSILCFG_REVISION_MODID_MASK                                        (0xFFFF0000U)
#define CSL_PSILCFG_REVISION_MODID_SHIFT                                       (0x00000010U)
#define CSL_PSILCFG_REVISION_MODID_MAX                                         (0x0000FFFFU)

#define CSL_PSILCFG_REVISION_REVRTL_MASK                                       (0x0000F800U)
#define CSL_PSILCFG_REVISION_REVRTL_SHIFT                                      (0x0000000BU)
#define CSL_PSILCFG_REVISION_REVRTL_MAX                                        (0x0000001FU)

#define CSL_PSILCFG_REVISION_REVMAJ_MASK                                       (0x00000700U)
#define CSL_PSILCFG_REVISION_REVMAJ_SHIFT                                      (0x00000008U)
#define CSL_PSILCFG_REVISION_REVMAJ_MAX                                        (0x00000007U)

#define CSL_PSILCFG_REVISION_CUSTOM_MASK                                       (0x000000C0U)
#define CSL_PSILCFG_REVISION_CUSTOM_SHIFT                                      (0x00000006U)
#define CSL_PSILCFG_REVISION_CUSTOM_MAX                                        (0x00000003U)

#define CSL_PSILCFG_REVISION_REVMIN_MASK                                       (0x0000003FU)
#define CSL_PSILCFG_REVISION_REVMIN_SHIFT                                      (0x00000000U)
#define CSL_PSILCFG_REVISION_REVMIN_MAX                                        (0x0000003FU)

/* PSIL_TO */

#define CSL_PSILCFG_PSIL_TO_TOUT_MASK                                          (0x80000000U)
#define CSL_PSILCFG_PSIL_TO_TOUT_SHIFT                                         (0x0000001FU)
#define CSL_PSILCFG_PSIL_TO_TOUT_MAX                                           (0x00000001U)

#define CSL_PSILCFG_PSIL_TO_TOUT_CNT_MASK                                      (0x0000FFFFU)
#define CSL_PSILCFG_PSIL_TO_TOUT_CNT_SHIFT                                     (0x00000000U)
#define CSL_PSILCFG_PSIL_TO_TOUT_CNT_MAX                                       (0x0000FFFFU)

/* CMDA */

#define CSL_PSILCFG_CMDA_BUSY_MASK                                             (0x80000000U)
#define CSL_PSILCFG_CMDA_BUSY_SHIFT                                            (0x0000001FU)
#define CSL_PSILCFG_CMDA_BUSY_MAX                                              (0x00000001U)

#define CSL_PSILCFG_CMDA_BUSY_VAL_FALSE                                        (0x0U)
#define CSL_PSILCFG_CMDA_BUSY_VAL_TRUE                                         (0x1U)

#define CSL_PSILCFG_CMDA_DIR_MASK                                              (0x40000000U)
#define CSL_PSILCFG_CMDA_DIR_SHIFT                                             (0x0000001EU)
#define CSL_PSILCFG_CMDA_DIR_MAX                                               (0x00000001U)

#define CSL_PSILCFG_CMDA_DIR_VAL_WRITE                                         (0x0U)
#define CSL_PSILCFG_CMDA_DIR_VAL_READ                                          (0x1U)

#define CSL_PSILCFG_CMDA_TO_MASK                                               (0x20000000U)
#define CSL_PSILCFG_CMDA_TO_SHIFT                                              (0x0000001DU)
#define CSL_PSILCFG_CMDA_TO_MAX                                                (0x00000001U)

#define CSL_PSILCFG_CMDA_TO_VAL_WRITE                                          (0x0U)
#define CSL_PSILCFG_CMDA_TO_VAL_READ                                           (0x1U)

#define CSL_PSILCFG_CMDA_THREAD_ID_MASK                                        (0x0000FFFFU)
#define CSL_PSILCFG_CMDA_THREAD_ID_SHIFT                                       (0x00000000U)
#define CSL_PSILCFG_CMDA_THREAD_ID_MAX                                         (0x0000FFFFU)

/* CMDB */

#define CSL_PSILCFG_CMDB_BYTEN_MASK                                            (0xF0000000U)
#define CSL_PSILCFG_CMDB_BYTEN_SHIFT                                           (0x0000001CU)
#define CSL_PSILCFG_CMDB_BYTEN_MAX                                             (0x0000000FU)

#define CSL_PSILCFG_CMDB_ADDRESS_MASK                                          (0x0000FFFFU)
#define CSL_PSILCFG_CMDB_ADDRESS_SHIFT                                         (0x00000000U)
#define CSL_PSILCFG_CMDB_ADDRESS_MAX                                           (0x0000FFFFU)

#define CSL_PSILCFG_CMDB_ADDRESS_VAL_PEER_THREAD_ID                            (0x0U)
#define CSL_PSILCFG_CMDB_ADDRESS_VAL_PEER_CREDIT                               (0x1U)
#define CSL_PSILCFG_CMDB_ADDRESS_VAL_ENABLE                                    (0x2U)
#define CSL_PSILCFG_CMDB_ADDRESS_VAL_CAPABILITIES                              (0x40U)
#define CSL_PSILCFG_CMDB_ADDRESS_VAL_STATIC_TR                                 (0x400U)

/* WDATA */

#define CSL_PSILCFG_WDATA_WDATA_MASK                                           (0xFFFFFFFFU)
#define CSL_PSILCFG_WDATA_WDATA_SHIFT                                          (0x00000000U)
#define CSL_PSILCFG_WDATA_WDATA_MAX                                            (0xFFFFFFFFU)

/* RDATA */

#define CSL_PSILCFG_RDATA_RDATA_MASK                                           (0xFFFFFFFFU)
#define CSL_PSILCFG_RDATA_RDATA_SHIFT                                          (0x00000000U)
#define CSL_PSILCFG_RDATA_RDATA_MAX                                            (0xFFFFFFFFU)

#ifdef __cplusplus
}
#endif
#endif
