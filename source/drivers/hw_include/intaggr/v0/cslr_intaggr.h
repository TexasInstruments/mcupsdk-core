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
 *  Name        : cslr_intaggr.h
*/
#ifndef CSLR_INTAGGR_H_
#define CSLR_INTAGGR_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  : Interrupt Aggregator Global Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint64_t REVISION;                  /* Revision Register */
    volatile uint64_t INTCAP;                    /* Interrupt Capabilities */
    volatile uint64_t AUXCAP;                    /* Auxiliary Capabilities */
} CSL_intaggr_cfgRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_INTAGGR_CFG_REVISION                                               (0x00000000U)
#define CSL_INTAGGR_CFG_INTCAP                                                 (0x00000008U)
#define CSL_INTAGGR_CFG_AUXCAP                                                 (0x00000010U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* REVISION */

#define CSL_INTAGGR_CFG_REVISION_MODID_MASK                                    (0x00000000FFFF0000U)
#define CSL_INTAGGR_CFG_REVISION_MODID_SHIFT                                   (0x0000000000000010U)
#define CSL_INTAGGR_CFG_REVISION_MODID_MAX                                     (0x000000000000FFFFU)

#define CSL_INTAGGR_CFG_REVISION_REVRTL_MASK                                   (0x000000000000F800U)
#define CSL_INTAGGR_CFG_REVISION_REVRTL_SHIFT                                  (0x000000000000000BU)
#define CSL_INTAGGR_CFG_REVISION_REVRTL_MAX                                    (0x000000000000001FU)

#define CSL_INTAGGR_CFG_REVISION_REVMAJ_MASK                                   (0x0000000000000700U)
#define CSL_INTAGGR_CFG_REVISION_REVMAJ_SHIFT                                  (0x0000000000000008U)
#define CSL_INTAGGR_CFG_REVISION_REVMAJ_MAX                                    (0x0000000000000007U)

#define CSL_INTAGGR_CFG_REVISION_CUSTOM_MASK                                   (0x00000000000000C0U)
#define CSL_INTAGGR_CFG_REVISION_CUSTOM_SHIFT                                  (0x0000000000000006U)
#define CSL_INTAGGR_CFG_REVISION_CUSTOM_MAX                                    (0x0000000000000003U)

#define CSL_INTAGGR_CFG_REVISION_REVMIN_MASK                                   (0x000000000000003FU)
#define CSL_INTAGGR_CFG_REVISION_REVMIN_SHIFT                                  (0x0000000000000000U)
#define CSL_INTAGGR_CFG_REVISION_REVMIN_MAX                                    (0x000000000000003FU)

/* INTCAP */

#define CSL_INTAGGR_CFG_INTCAP_VINTR_CNT_MASK                                  (0x00000000FFFF0000U)
#define CSL_INTAGGR_CFG_INTCAP_VINTR_CNT_SHIFT                                 (0x0000000000000010U)
#define CSL_INTAGGR_CFG_INTCAP_VINTR_CNT_MAX                                   (0x000000000000FFFFU)

#define CSL_INTAGGR_CFG_INTCAP_SEVT_CNT_MASK                                   (0x000000000000FFFFU)
#define CSL_INTAGGR_CFG_INTCAP_SEVT_CNT_SHIFT                                  (0x0000000000000000U)
#define CSL_INTAGGR_CFG_INTCAP_SEVT_CNT_MAX                                    (0x000000000000FFFFU)

/* AUXCAP */

#define CSL_INTAGGR_CFG_AUXCAP_UNMAP_CNT_MASK                                  (0xFFFF000000000000U)
#define CSL_INTAGGR_CFG_AUXCAP_UNMAP_CNT_SHIFT                                 (0x0000000000000030U)
#define CSL_INTAGGR_CFG_AUXCAP_UNMAP_CNT_MAX                                   (0x000000000000FFFFU)

#define CSL_INTAGGR_CFG_AUXCAP_MEVI_CNT_MASK                                   (0x0000FFFF00000000U)
#define CSL_INTAGGR_CFG_AUXCAP_MEVI_CNT_SHIFT                                  (0x0000000000000020U)
#define CSL_INTAGGR_CFG_AUXCAP_MEVI_CNT_MAX                                    (0x000000000000FFFFU)

#define CSL_INTAGGR_CFG_AUXCAP_LEVI_CNT_MASK                                   (0x00000000FFFF0000U)
#define CSL_INTAGGR_CFG_AUXCAP_LEVI_CNT_SHIFT                                  (0x0000000000000010U)
#define CSL_INTAGGR_CFG_AUXCAP_LEVI_CNT_MAX                                    (0x000000000000FFFFU)

#define CSL_INTAGGR_CFG_AUXCAP_GEVI_CNT_MASK                                   (0x000000000000FFFFU)
#define CSL_INTAGGR_CFG_AUXCAP_GEVI_CNT_SHIFT                                  (0x0000000000000000U)
#define CSL_INTAGGR_CFG_AUXCAP_GEVI_CNT_MAX                                    (0x000000000000FFFFU)

/**************************************************************************
* Hardware Region  : Event to Interrupt Mapping Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint64_t IMAP;                      /* Interrupt Mapping Register */
} CSL_intaggr_imapRegs_gevi;


typedef struct {
    CSL_intaggr_imapRegs_gevi GEVI[8192];
} CSL_intaggr_imapRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_INTAGGR_IMAP_GEVI_IMAP(GEVI)                                       (0x00000000U+((GEVI)*0x8U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* IMAP */

#define CSL_INTAGGR_IMAP_GEVI_IMAP_REGNUM_MASK                                 (0x000000000001FF00U)
#define CSL_INTAGGR_IMAP_GEVI_IMAP_REGNUM_SHIFT                                (0x0000000000000008U)
#define CSL_INTAGGR_IMAP_GEVI_IMAP_REGNUM_MAX                                  (0x00000000000001FFU)

#define CSL_INTAGGR_IMAP_GEVI_IMAP_BITNUM_MASK                                 (0x000000000000003FU)
#define CSL_INTAGGR_IMAP_GEVI_IMAP_BITNUM_SHIFT                                (0x0000000000000000U)
#define CSL_INTAGGR_IMAP_GEVI_IMAP_BITNUM_MAX                                  (0x000000000000003FU)

/**************************************************************************
* Hardware Region  : Interrupt Control / Status Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint64_t ENABLE_SET;                /* Interrupt Enable Set Register */
    volatile uint64_t ENABLE_CLEAR;              /* Interrupt Enable Clear Register */
    volatile uint64_t STATUS_SET;                /* Interrupt Status Set Register */
    volatile uint64_t STATUS_CLEAR;              /* Interrupt Status Clear Register */
    volatile uint64_t STATUSM;                   /* Interrupt Masked Status Register */
    volatile uint8_t  Resv_4096[4056];
} CSL_intaggr_intrRegs_vint;


typedef struct {
    CSL_intaggr_intrRegs_vint VINT[512];
} CSL_intaggr_intrRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_INTAGGR_INTR_VINT_ENABLE_SET(VINT)                                 (0x00000000U+((VINT)*0x1000U))
#define CSL_INTAGGR_INTR_VINT_ENABLE_CLEAR(VINT)                               (0x00000008U+((VINT)*0x1000U))
#define CSL_INTAGGR_INTR_VINT_STATUS_SET(VINT)                                 (0x00000010U+((VINT)*0x1000U))
#define CSL_INTAGGR_INTR_VINT_STATUS_CLEAR(VINT)                               (0x00000018U+((VINT)*0x1000U))
#define CSL_INTAGGR_INTR_VINT_STATUSM(VINT)                                    (0x00000020U+((VINT)*0x1000U))

/**************************************************************************
* 64K Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint64_t ENABLE_SET;                /* Interrupt Enable Set Register */
    volatile uint64_t ENABLE_CLEAR;              /* Interrupt Enable Clear Register */
    volatile uint64_t STATUS_SET;                /* Interrupt Status Set Register */
    volatile uint64_t STATUS_CLEAR;              /* Interrupt Status Clear Register */
    volatile uint64_t STATUSM;                   /* Interrupt Masked Status Register */
    volatile uint8_t  Resv_65536[65496];
} CSL_a64_intaggr_intrRegs_vint;


typedef struct {
    CSL_a64_intaggr_intrRegs_vint VINT[512];
} CSL_a64_intaggr_intrRegs;


/**************************************************************************
* 64K Register Macros
**************************************************************************/

#define CSL_A64_INTAGGR_INTR_VINT_ENABLE_SET(VINT)                             (0x00000000U+((VINT)*0x10000U))
#define CSL_A64_INTAGGR_INTR_VINT_ENABLE_CLEAR(VINT)                           (0x00000008U+((VINT)*0x10000U))
#define CSL_A64_INTAGGR_INTR_VINT_STATUS_SET(VINT)                             (0x00000010U+((VINT)*0x10000U))
#define CSL_A64_INTAGGR_INTR_VINT_STATUS_CLEAR(VINT)                           (0x00000018U+((VINT)*0x10000U))
#define CSL_A64_INTAGGR_INTR_VINT_STATUSM(VINT)                                (0x00000020U+((VINT)*0x10000U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* ENABLE_SET */

#define CSL_INTAGGR_INTR_VINT_ENABLE_SET_ENABLE_MASK                           (0xFFFFFFFFFFFFFFFFU)
#define CSL_INTAGGR_INTR_VINT_ENABLE_SET_ENABLE_SHIFT                          (0x0000000000000000U)
#define CSL_INTAGGR_INTR_VINT_ENABLE_SET_ENABLE_MAX                            (0xFFFFFFFFFFFFFFFFU)

/* ENABLE_CLEAR */

#define CSL_INTAGGR_INTR_VINT_ENABLE_CLEAR_ENABLE_MASK                         (0xFFFFFFFFFFFFFFFFU)
#define CSL_INTAGGR_INTR_VINT_ENABLE_CLEAR_ENABLE_SHIFT                        (0x0000000000000000U)
#define CSL_INTAGGR_INTR_VINT_ENABLE_CLEAR_ENABLE_MAX                          (0xFFFFFFFFFFFFFFFFU)

/* STATUS_SET */

#define CSL_INTAGGR_INTR_VINT_STATUS_SET_STATUS_MASK                           (0xFFFFFFFFFFFFFFFFU)
#define CSL_INTAGGR_INTR_VINT_STATUS_SET_STATUS_SHIFT                          (0x0000000000000000U)
#define CSL_INTAGGR_INTR_VINT_STATUS_SET_STATUS_MAX                            (0xFFFFFFFFFFFFFFFFU)

/* STATUS_CLEAR */

#define CSL_INTAGGR_INTR_VINT_STATUS_CLEAR_STATUS_MASK                         (0xFFFFFFFFFFFFFFFFU)
#define CSL_INTAGGR_INTR_VINT_STATUS_CLEAR_STATUS_SHIFT                        (0x0000000000000000U)
#define CSL_INTAGGR_INTR_VINT_STATUS_CLEAR_STATUS_MAX                          (0xFFFFFFFFFFFFFFFFU)

/* STATUSM */

#define CSL_INTAGGR_INTR_VINT_STATUSM_STATUS_MASK                              (0xFFFFFFFFFFFFFFFFU)
#define CSL_INTAGGR_INTR_VINT_STATUSM_STATUS_SHIFT                             (0x0000000000000000U)
#define CSL_INTAGGR_INTR_VINT_STATUSM_STATUS_MAX                               (0xFFFFFFFFFFFFFFFFU)

/**************************************************************************
* Hardware Region  : Local to Global Event Mapping Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint64_t MAP;                       /* Local to global event mapping */
    volatile uint8_t  Resv_32[24];
} CSL_intaggr_l2gRegs_levi;


typedef struct {
    CSL_intaggr_l2gRegs_levi LEVI[512];
} CSL_intaggr_l2gRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_INTAGGR_L2G_LEVI_MAP(LEVI)                                         (0x00000000U+((LEVI)*0x20U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* MAP */

#define CSL_INTAGGR_L2G_LEVI_MAP_MODE_MASK                                     (0x0000000080000000U)
#define CSL_INTAGGR_L2G_LEVI_MAP_MODE_SHIFT                                    (0x000000000000001FU)
#define CSL_INTAGGR_L2G_LEVI_MAP_MODE_MAX                                      (0x0000000000000001U)

#define CSL_INTAGGR_L2G_LEVI_MAP_GEVIDX_MASK                                   (0x000000000000FFFFU)
#define CSL_INTAGGR_L2G_LEVI_MAP_GEVIDX_SHIFT                                  (0x0000000000000000U)
#define CSL_INTAGGR_L2G_LEVI_MAP_GEVIDX_MAX                                    (0x000000000000FFFFU)

/**************************************************************************
* Hardware Region  : Global Event Multicast Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint64_t MCMAP;                     /* Multicast event mapping */
    volatile uint8_t  Resv_32[24];
} CSL_intaggr_mcastRegs_gevi;


typedef struct {
    CSL_intaggr_mcastRegs_gevi GEVI[512];
} CSL_intaggr_mcastRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_INTAGGR_MCAST_GEVI_MCMAP(GEVI)                                     (0x00000000U+((GEVI)*0x20U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* MCMAP */

#define CSL_INTAGGR_MCAST_GEVI_MCMAP_IRQMODE1_MASK                             (0x8000000000000000U)
#define CSL_INTAGGR_MCAST_GEVI_MCMAP_IRQMODE1_SHIFT                            (0x000000000000003FU)
#define CSL_INTAGGR_MCAST_GEVI_MCMAP_IRQMODE1_MAX                              (0x0000000000000001U)

#define CSL_INTAGGR_MCAST_GEVI_MCMAP_GEVIDX1_MASK                              (0x0000FFFF00000000U)
#define CSL_INTAGGR_MCAST_GEVI_MCMAP_GEVIDX1_SHIFT                             (0x0000000000000020U)
#define CSL_INTAGGR_MCAST_GEVI_MCMAP_GEVIDX1_MAX                               (0x000000000000FFFFU)

#define CSL_INTAGGR_MCAST_GEVI_MCMAP_IRQMODE0_MASK                             (0x0000000080000000U)
#define CSL_INTAGGR_MCAST_GEVI_MCMAP_IRQMODE0_SHIFT                            (0x000000000000001FU)
#define CSL_INTAGGR_MCAST_GEVI_MCMAP_IRQMODE0_MAX                              (0x0000000000000001U)

#define CSL_INTAGGR_MCAST_GEVI_MCMAP_GEVIDX0_MASK                              (0x000000000000FFFFU)
#define CSL_INTAGGR_MCAST_GEVI_MCMAP_GEVIDX0_SHIFT                             (0x0000000000000000U)
#define CSL_INTAGGR_MCAST_GEVI_MCMAP_GEVIDX0_MAX                               (0x000000000000FFFFU)

/**************************************************************************
* Hardware Region  : Global Event Count Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint64_t MAP;                       /* Gobal Event Mapping Register */
    volatile uint8_t  Resv_32[24];
} CSL_intaggr_gcntcfgRegs_gevi;


typedef struct {
    CSL_intaggr_gcntcfgRegs_gevi GEVI[512];
} CSL_intaggr_gcntcfgRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_INTAGGR_GCNTCFG_GEVI_MAP(GEVI)                                     (0x00000000U+((GEVI)*0x20U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* MAP */

#define CSL_INTAGGR_GCNTCFG_GEVI_MAP_IRQMODE_MASK                              (0x0000000080000000U)
#define CSL_INTAGGR_GCNTCFG_GEVI_MAP_IRQMODE_SHIFT                             (0x000000000000001FU)
#define CSL_INTAGGR_GCNTCFG_GEVI_MAP_IRQMODE_MAX                               (0x0000000000000001U)

#define CSL_INTAGGR_GCNTCFG_GEVI_MAP_GEVIDX_MASK                               (0x000000000000FFFFU)
#define CSL_INTAGGR_GCNTCFG_GEVI_MAP_GEVIDX_SHIFT                              (0x0000000000000000U)
#define CSL_INTAGGR_GCNTCFG_GEVI_MAP_GEVIDX_MAX                                (0x000000000000FFFFU)

/**************************************************************************
* Hardware Region  : Global Event Count Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint64_t COUNT;                     /* ETL Count Register */
    volatile uint8_t  Resv_4096[4088];
} CSL_intaggr_gcntrtiRegs_gevi;


typedef struct {
    CSL_intaggr_gcntrtiRegs_gevi GEVI[512];
} CSL_intaggr_gcntrtiRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_INTAGGR_GCNTRTI_GEVI_COUNT(GEVI)                                   (0x00000000U+((GEVI)*0x1000U))

/**************************************************************************
* 64K Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint64_t COUNT;                     /* ETL Count Register */
    volatile uint8_t  Resv_65536[65528];
} CSL_a64_intaggr_gcntrtiRegs_gevi;


typedef struct {
    CSL_a64_intaggr_gcntrtiRegs_gevi GEVI[512];
} CSL_a64_intaggr_gcntrtiRegs;


/**************************************************************************
* 64K Register Macros
**************************************************************************/

#define CSL_A64_INTAGGR_GCNTRTI_GEVI_COUNT(GEVI)                               (0x00000000U+((GEVI)*0x10000U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* COUNT */

#define CSL_INTAGGR_GCNTRTI_GEVI_COUNT_CCNT_MASK                               (0x00000000FFFFFFFFU)
#define CSL_INTAGGR_GCNTRTI_GEVI_COUNT_CCNT_SHIFT                              (0x0000000000000000U)
#define CSL_INTAGGR_GCNTRTI_GEVI_COUNT_CCNT_MAX                                (0x00000000FFFFFFFFU)

/**************************************************************************
* Hardware Region  : Unmapped Event to Interrupt Mapping Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint64_t MAP;                       /* Unmapped Event Mapping Register */
} CSL_intaggr_unmapRegs_unmap;


typedef struct {
    CSL_intaggr_unmapRegs_unmap UNMAP[65536];
} CSL_intaggr_unmapRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_INTAGGR_UNMAP_UNMAP_MAP(UNMAP)                                     (0x00000000U+((UNMAP)*0x8U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* MAP */

#define CSL_INTAGGR_UNMAP_UNMAP_MAP_IRQMODE_MASK                               (0x0000000080000000U)
#define CSL_INTAGGR_UNMAP_UNMAP_MAP_IRQMODE_SHIFT                              (0x000000000000001FU)
#define CSL_INTAGGR_UNMAP_UNMAP_MAP_IRQMODE_MAX                                (0x0000000000000001U)

#define CSL_INTAGGR_UNMAP_UNMAP_MAP_MAPIDX_MASK                                (0x000000000000FFFFU)
#define CSL_INTAGGR_UNMAP_UNMAP_MAP_MAPIDX_SHIFT                               (0x0000000000000000U)
#define CSL_INTAGGR_UNMAP_UNMAP_MAP_MAPIDX_MAX                                 (0x000000000000FFFFU)

#ifdef __cplusplus
}
#endif
#endif
