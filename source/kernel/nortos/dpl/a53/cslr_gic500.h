/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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

#ifndef CSLR_GIC500_H
#define CSLR_GIC500_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/**************************************************************************
* Hardware Region  : GICD
**************************************************************************/

/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
   volatile uint32_t                         LOWER;
   volatile uint32_t                         UPPER;
} CSL_gic500_gicdRegs_irouter;

/************************************************************************
* Register Macros
**************************************************************************/

#define CSL_GIC500_GICD_IROUTER_LOWER(IROUTER)                        ( 0x0U +  (0x6100U + ((IROUTER)*0x8U)) )
#define CSL_GIC500_GICD_IROUTER_UPPER(IROUTER)                        ( 0x0U +  (0x6100U + ((IROUTER)*0x8U)) + 0x4U )

/************************************************************************
* Field Definition Macros
**************************************************************************/

/* LOWER */

#define CSL_GIC500_GICD_IROUTER_LOWER_A0_MASK                              (0xffU)
#define CSL_GIC500_GICD_IROUTER_LOWER_A0_SHIFT                             (0x0U)
#define CSL_GIC500_GICD_IROUTER_LOWER_A0_RESETVAL                          (0x0U)

#define CSL_GIC500_GICD_IROUTER_LOWER_A1_MASK                              (0xff00U)
#define CSL_GIC500_GICD_IROUTER_LOWER_A1_SHIFT                             (0x8U)
#define CSL_GIC500_GICD_IROUTER_LOWER_A1_RESETVAL                          (0x0U)

#define CSL_GIC500_GICD_IROUTER_LOWER_IRM_MASK                             (0x80000000U)
#define CSL_GIC500_GICD_IROUTER_LOWER_IRM_SHIFT                            (0x1fU)
#define CSL_GIC500_GICD_IROUTER_LOWER_IRM_RESETVAL                         (0x0U)

#define CSL_GIC500_GICD_IROUTER_LOWER_A2_MASK                              (0xff0000U)
#define CSL_GIC500_GICD_IROUTER_LOWER_A2_SHIFT                             (0x10U)
#define CSL_GIC500_GICD_IROUTER_LOWER_A2_RESETVAL                          (0x0U)

#define CSL_GIC500_GICD_IROUTER_LOWER_RESETVAL                             0x0
/* UPPER */

#define CSL_GIC500_GICD_IROUTER_UPPER_A3_MASK                              (0xffU)
#define CSL_GIC500_GICD_IROUTER_UPPER_A3_SHIFT                             (0x0U)
#define CSL_GIC500_GICD_IROUTER_UPPER_A3_RESETVAL                          (0x0U)

#define CSL_GIC500_GICD_IROUTER_UPPER_RESETVAL                             0x0
/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
   volatile uint32_t                         CTLR;
   volatile uint32_t                         TYPER;
   volatile uint32_t                         IIDR;
   volatile uint8_t                          Rsvd0[4];
   volatile uint32_t                         STATUSR;
   volatile uint8_t                          Rsvd1[44];
   volatile uint32_t                         SETSPI_NSR;
   volatile uint8_t                          Rsvd2[4];
   volatile uint32_t                         CLRSPI_NSR;
   volatile uint8_t                          Rsvd3[4];
   volatile uint32_t                         SETSPI_SR;
   volatile uint8_t                          Rsvd4[4];
   volatile uint32_t                         CLRSPI_SR;
   volatile uint8_t                          Rsvd5[36];
   volatile uint32_t                         IGROUPR_SGI_PPI;
   volatile uint32_t                         IGROUPR_SPI[30];
   volatile uint8_t                          Rsvd6[4];
   volatile uint32_t                         ISENABLER_SGI_PPI;
   volatile uint32_t                         ISENABLER_SPI[30];
   volatile uint8_t                          Rsvd7[4];
   volatile uint32_t                         ICENABLER_SGI_PPI;
   volatile uint32_t                         ICENABLER_SPI[30];
   volatile uint8_t                          Rsvd8[4];
   volatile uint32_t                         ISPENDR_SGI_PPI;
   volatile uint32_t                         ISPENDR_SPI[30];
   volatile uint8_t                          Rsvd9[4];
   volatile uint32_t                         ICPENDR_SGI_PPI;
   volatile uint32_t                         ICPENDR_SPI[30];
   volatile uint8_t                          Rsvd10[4];
   volatile uint32_t                         ISACTIVER_SGI_PPI;
   volatile uint32_t                         ISACTIVER_SPI[30];
   volatile uint8_t                          Rsvd11[4];
   volatile uint32_t                         ICACTIVER_SGI_PPI;
   volatile uint32_t                         ICACTIVER_SPI[30];
   volatile uint8_t                          Rsvd12[4];
   volatile uint32_t                         IPRIORITYR_SGI_PPI[8];
   volatile uint32_t                         IPRIORITYR_SPI[240];
   volatile uint8_t                          Rsvd13[32];
   volatile uint32_t                         ITARGETSR_SGI_PPI[8];
   volatile uint32_t                         ITARGETSR_SPI[240];
   volatile uint8_t                          Rsvd14[32];
   volatile uint32_t                         ICFGR_SGI_PPI[2];
   volatile uint32_t                         ICFGR_SPI[60];
   volatile uint8_t                          Rsvd15[8];
   volatile uint32_t                         IGRPMODR_SGI_PPI;
   volatile uint32_t                         IGRPMODR_SPI[30];
   volatile uint8_t                          Rsvd16[132];
   volatile uint32_t                         NSACR[62];
   volatile uint8_t                          Rsvd17[8];
   volatile uint32_t                         SGIR;
   volatile uint8_t                          Rsvd18[12];
   volatile uint32_t                         CPENDSGIR[4];
   volatile uint32_t                         SPENDSGIR[4];
   volatile uint8_t                          Rsvd19[20944];
   CSL_gic500_gicdRegs_irouter               IROUTER[960];
   volatile uint8_t                          Rsvd20[16640];
   volatile uint32_t                         ESTATUSR;
   volatile uint32_t                         ERRTESTR;
   volatile uint8_t                          Rsvd21[124];
   volatile uint32_t                         SPISR[30];
   volatile uint8_t                          Rsvd22[16084];
   volatile uint32_t                         PIDR4;
   volatile uint32_t                         PIDR5;
   volatile uint32_t                         PIDR6;
   volatile uint32_t                         PIDR7;
   volatile uint32_t                         PIDR0;
   volatile uint32_t                         PIDR1;
   volatile uint32_t                         PIDR2;
   volatile uint32_t                         PIDR3;
   volatile uint32_t                         CIDR0;
   volatile uint32_t                         CIDR1;
   volatile uint32_t                         CIDR2;
   volatile uint32_t                         CIDR3;
} CSL_gic500_gicdRegs;

/************************************************************************
* Register Macros
**************************************************************************/

#define CSL_GIC500_GICD_CTLR                                          ( 0x0U )
#define CSL_GIC500_GICD_TYPER                                         ( 0x0U + 0x4U )
#define CSL_GIC500_GICD_IIDR                                          ( 0x0U + 0x8U )
#define CSL_GIC500_GICD_STATUSR                                       ( 0x0U + 0x10U )
#define CSL_GIC500_GICD_SETSPI_NSR                                    ( 0x0U + 0x40U )
#define CSL_GIC500_GICD_CLRSPI_NSR                                    ( 0x0U + 0x48U )
#define CSL_GIC500_GICD_SETSPI_SR                                     ( 0x0U + 0x50U )
#define CSL_GIC500_GICD_CLRSPI_SR                                     ( 0x0U + 0x58U )
#define CSL_GIC500_GICD_IGROUPR_SGI_PPI                               ( 0x0U + 0x80U )
#define CSL_GIC500_GICD_IGROUPR_SPI(IDX)                              ( 0x0U +  (0x84U + ((IDX)*0x4U)) )
#define CSL_GIC500_GICD_ISENABLER_SGI_PPI                             ( 0x0U + 0x100U )
#define CSL_GIC500_GICD_ISENABLER_SPI(IDX)                            ( 0x0U +  (0x104U + ((IDX)*0x4U)) )
#define CSL_GIC500_GICD_ICENABLER_SGI_PPI                             ( 0x0U + 0x180U )
#define CSL_GIC500_GICD_ICENABLER_SPI(IDX)                            ( 0x0U +  (0x184U + ((IDX)*0x4U)) )
#define CSL_GIC500_GICD_ISPENDR_SGI_PPI                               ( 0x0U + 0x200U )
#define CSL_GIC500_GICD_ISPENDR_SPI(IDX)                              ( 0x0U +  (0x204U + ((IDX)*0x4U)) )
#define CSL_GIC500_GICD_ICPENDR_SGI_PPI                               ( 0x0U + 0x280U )
#define CSL_GIC500_GICD_ICPENDR_SPI(IDX)                              ( 0x0U +  (0x284U + ((IDX)*0x4U)) )
#define CSL_GIC500_GICD_ISACTIVER_SGI_PPI                             ( 0x0U + 0x300U )
#define CSL_GIC500_GICD_ISACTIVER_SPI(IDX)                            ( 0x0U +  (0x304U + ((IDX)*0x4U)) )
#define CSL_GIC500_GICD_ICACTIVER_SGI_PPI                             ( 0x0U + 0x380U )
#define CSL_GIC500_GICD_ICACTIVER_SPI(IDX)                            ( 0x0U +  (0x384U + ((IDX)*0x4U)) )
#define CSL_GIC500_GICD_IPRIORITYR_SGI_PPI(IDX)                       ( 0x0U +  (0x400U + ((IDX)*0x4U)) )
#define CSL_GIC500_GICD_IPRIORITYR_SPI(IDX)                           ( 0x0U +  (0x420U + ((IDX)*0x4U)) )
#define CSL_GIC500_GICD_ITARGETSR_SGI_PPI(IDX)                        ( 0x0U +  (0x800U + ((IDX)*0x4U)) )
#define CSL_GIC500_GICD_ITARGETSR_SPI(IDX)                            ( 0x0U +  (0x820U + ((IDX)*0x4U)) )
#define CSL_GIC500_GICD_ICFGR_SGI_PPI(IDX)                            ( 0x0U +  (0xc00U + ((IDX)*0x4U)) )
#define CSL_GIC500_GICD_ICFGR_SPI(IDX)                                ( 0x0U +  (0xc08U + ((IDX)*0x4U)) )
#define CSL_GIC500_GICD_IGRPMODR_SGI_PPI                              ( 0x0U + 0xd00U )
#define CSL_GIC500_GICD_IGRPMODR_SPI(IDX)                             ( 0x0U +  (0xd04U + ((IDX)*0x4U)) )
#define CSL_GIC500_GICD_NSACR(IDX)                                    ( 0x0U +  (0xe00U + ((IDX)*0x4U)) )
#define CSL_GIC500_GICD_SGIR                                          ( 0x0U + 0xf00U )
#define CSL_GIC500_GICD_CPENDSGIR(IDX)                                ( 0x0U +  (0xf10U + ((IDX)*0x4U)) )
#define CSL_GIC500_GICD_SPENDSGIR(IDX)                                ( 0x0U +  (0xf20U + ((IDX)*0x4U)) )
#define CSL_GIC500_GICD_ESTATUSR                                      ( 0x0U + 0xc000U )
#define CSL_GIC500_GICD_ERRTESTR                                      ( 0x0U + 0xc004U )
#define CSL_GIC500_GICD_SPISR(IDX)                                    ( 0x0U +  (0xc084U + ((IDX)*0x4U)) )
#define CSL_GIC500_GICD_PIDR4                                         ( 0x0U + 0xffd0U )
#define CSL_GIC500_GICD_PIDR5                                         ( 0x0U + 0xffd4U )
#define CSL_GIC500_GICD_PIDR6                                         ( 0x0U + 0xffd8U )
#define CSL_GIC500_GICD_PIDR7                                         ( 0x0U + 0xffdcU )
#define CSL_GIC500_GICD_PIDR0                                         ( 0x0U + 0xffe0U )
#define CSL_GIC500_GICD_PIDR1                                         ( 0x0U + 0xffe4U )
#define CSL_GIC500_GICD_PIDR2                                         ( 0x0U + 0xffe8U )
#define CSL_GIC500_GICD_PIDR3                                         ( 0x0U + 0xffecU )
#define CSL_GIC500_GICD_CIDR0                                         ( 0x0U + 0xfff0U )
#define CSL_GIC500_GICD_CIDR1                                         ( 0x0U + 0xfff4U )
#define CSL_GIC500_GICD_CIDR2                                         ( 0x0U + 0xfff8U )
#define CSL_GIC500_GICD_CIDR3                                         ( 0x0U + 0xfffcU )

/************************************************************************
* Field Definition Macros
**************************************************************************/

/* CTLR */

#define CSL_GIC500_GICD_CTLR_ENABLEGRP0_MASK                               (0x1U)
#define CSL_GIC500_GICD_CTLR_ENABLEGRP0_SHIFT                              (0x0U)
#define CSL_GIC500_GICD_CTLR_ENABLEGRP0_RESETVAL                           (0x0U)

#define CSL_GIC500_GICD_CTLR_ENABLEGRP1_NS_MASK                            (0x2U)
#define CSL_GIC500_GICD_CTLR_ENABLEGRP1_NS_SHIFT                           (0x1U)
#define CSL_GIC500_GICD_CTLR_ENABLEGRP1_NS_RESETVAL                        (0x0U)

#define CSL_GIC500_GICD_CTLR_ENABLEGRP1_S_MASK                             (0x4U)
#define CSL_GIC500_GICD_CTLR_ENABLEGRP1_S_SHIFT                            (0x2U)
#define CSL_GIC500_GICD_CTLR_ENABLEGRP1_S_RESETVAL                         (0x0U)

#define CSL_GIC500_GICD_CTLR_ARE_MASK                                      (0x10U)
#define CSL_GIC500_GICD_CTLR_ARE_SHIFT                                     (0x4U)
#define CSL_GIC500_GICD_CTLR_ARE_RESETVAL                                  (0x1U)

#define CSL_GIC500_GICD_CTLR_ARE_NS_MASK                                   (0x20U)
#define CSL_GIC500_GICD_CTLR_ARE_NS_SHIFT                                  (0x5U)
#define CSL_GIC500_GICD_CTLR_ARE_NS_RESETVAL                               (0x1U)

#define CSL_GIC500_GICD_CTLR_DS_MASK                                       (0x40U)
#define CSL_GIC500_GICD_CTLR_DS_SHIFT                                      (0x6U)
#define CSL_GIC500_GICD_CTLR_DS_RESETVAL                                   (0x0U)

#define CSL_GIC500_GICD_CTLR_RWP_MASK                                      (0x80000000U)
#define CSL_GIC500_GICD_CTLR_RWP_SHIFT                                     (0x1fU)
#define CSL_GIC500_GICD_CTLR_RWP_RESETVAL                                  (0x0U)

#define CSL_GIC500_GICD_CTLR_EINWF_MASK                                    (0x80U)
#define CSL_GIC500_GICD_CTLR_EINWF_SHIFT                                   (0x7U)
#define CSL_GIC500_GICD_CTLR_EINWF_RESETVAL                                (0x0U)

#define CSL_GIC500_GICD_CTLR_RESETVAL                                      0x30
/* TYPER */

#define CSL_GIC500_GICD_TYPER_ITLINESNUMBER_MASK                           (0x1fU)
#define CSL_GIC500_GICD_TYPER_ITLINESNUMBER_SHIFT                          (0x0U)
#define CSL_GIC500_GICD_TYPER_ITLINESNUMBER_RESETVAL                       (0x1eU)

#define CSL_GIC500_GICD_TYPER_CPUNUMBER_MASK                               (0xe0U)
#define CSL_GIC500_GICD_TYPER_CPUNUMBER_SHIFT                              (0x5U)
#define CSL_GIC500_GICD_TYPER_CPUNUMBER_RESETVAL                           (0x3U)

#define CSL_GIC500_GICD_TYPER_SECURITYEXTN_MASK                            (0x400U)
#define CSL_GIC500_GICD_TYPER_SECURITYEXTN_SHIFT                           (0xaU)
#define CSL_GIC500_GICD_TYPER_SECURITYEXTN_RESETVAL                        (0x1U)

#define CSL_GIC500_GICD_TYPER_LSPI_MASK                                    (0xf800U)
#define CSL_GIC500_GICD_TYPER_LSPI_SHIFT                                   (0xbU)
#define CSL_GIC500_GICD_TYPER_LSPI_RESETVAL                                (0x0U)

#define CSL_GIC500_GICD_TYPER_MBIS_MASK                                    (0x10000U)
#define CSL_GIC500_GICD_TYPER_MBIS_SHIFT                                   (0x10U)
#define CSL_GIC500_GICD_TYPER_MBIS_RESETVAL                                (0x1U)

#define CSL_GIC500_GICD_TYPER_LPIS_MASK                                    (0x20000U)
#define CSL_GIC500_GICD_TYPER_LPIS_SHIFT                                   (0x11U)
#define CSL_GIC500_GICD_TYPER_LPIS_RESETVAL                                (0x1U)

#define CSL_GIC500_GICD_TYPER_DVIS_MASK                                    (0x40000U)
#define CSL_GIC500_GICD_TYPER_DVIS_SHIFT                                   (0x12U)
#define CSL_GIC500_GICD_TYPER_DVIS_RESETVAL                                (0x0U)

#define CSL_GIC500_GICD_TYPER_IDBITS_MASK                                  (0xf80000U)
#define CSL_GIC500_GICD_TYPER_IDBITS_SHIFT                                 (0x13U)
#define CSL_GIC500_GICD_TYPER_IDBITS_RESETVAL                              (0xfU)

#define CSL_GIC500_GICD_TYPER_A3V_MASK                                     (0x1000000U)
#define CSL_GIC500_GICD_TYPER_A3V_SHIFT                                    (0x18U)
#define CSL_GIC500_GICD_TYPER_A3V_RESETVAL                                 (0x0U)

#define CSL_GIC500_GICD_TYPER_RESETVAL                                     0x7b047e
/* IIDR */

#define CSL_GIC500_GICD_IIDR_IMPLEMENTER_MASK                              (0xfffU)
#define CSL_GIC500_GICD_IIDR_IMPLEMENTER_SHIFT                             (0x0U)
#define CSL_GIC500_GICD_IIDR_IMPLEMENTER_RESETVAL                          (0x43bU)

#define CSL_GIC500_GICD_IIDR_REVISION_MASK                                 (0xf000U)
#define CSL_GIC500_GICD_IIDR_REVISION_SHIFT                                (0xcU)
#define CSL_GIC500_GICD_IIDR_REVISION_RESETVAL                             (0x1U)

#define CSL_GIC500_GICD_IIDR_VARIANT_MASK                                  (0xf0000U)
#define CSL_GIC500_GICD_IIDR_VARIANT_SHIFT                                 (0x10U)
#define CSL_GIC500_GICD_IIDR_VARIANT_RESETVAL                              (0x1U)

#define CSL_GIC500_GICD_IIDR_PRODUCTID_MASK                                (0xff000000U)
#define CSL_GIC500_GICD_IIDR_PRODUCTID_SHIFT                               (0x18U)
#define CSL_GIC500_GICD_IIDR_PRODUCTID_RESETVAL                            (0x0U)

#define CSL_GIC500_GICD_IIDR_RESETVAL                                      0x1143b
/* STATUSR */

#define CSL_GIC500_GICD_STATUSR_RSVD_MASK                                  (0xfffffff0U)
#define CSL_GIC500_GICD_STATUSR_RSVD_SHIFT                                 (0x4U)
#define CSL_GIC500_GICD_STATUSR_RSVD_RESETVAL                              (0x0U)

#define CSL_GIC500_GICD_STATUSR_RRD_MASK                                   (0x1U)
#define CSL_GIC500_GICD_STATUSR_RRD_SHIFT                                  (0x0U)
#define CSL_GIC500_GICD_STATUSR_RRD_RESETVAL                               (0x0U)

#define CSL_GIC500_GICD_STATUSR_WRD_MASK                                   (0x2U)
#define CSL_GIC500_GICD_STATUSR_WRD_SHIFT                                  (0x1U)
#define CSL_GIC500_GICD_STATUSR_WRD_RESETVAL                               (0x0U)

#define CSL_GIC500_GICD_STATUSR_RWOD_MASK                                  (0x4U)
#define CSL_GIC500_GICD_STATUSR_RWOD_SHIFT                                 (0x2U)
#define CSL_GIC500_GICD_STATUSR_RWOD_RESETVAL                              (0x0U)

#define CSL_GIC500_GICD_STATUSR_WROD_MASK                                  (0x8U)
#define CSL_GIC500_GICD_STATUSR_WROD_SHIFT                                 (0x3U)
#define CSL_GIC500_GICD_STATUSR_WROD_RESETVAL                              (0x0U)

#define CSL_GIC500_GICD_STATUSR_RESETVAL                                   0x0
/* SETSPI_NSR */

#define CSL_GIC500_GICD_SETSPI_NSR_SPI_ID_MASK                             (0x3ffU)
#define CSL_GIC500_GICD_SETSPI_NSR_SPI_ID_SHIFT                            (0x0U)
#define CSL_GIC500_GICD_SETSPI_NSR_SPI_ID_RESETVAL                         (0x0U)

#define CSL_GIC500_GICD_SETSPI_NSR_RESETVAL                                0x0
/* CLRSPI_NSR */

#define CSL_GIC500_GICD_CLRSPI_NSR_SPI_ID_MASK                             (0x3ffU)
#define CSL_GIC500_GICD_CLRSPI_NSR_SPI_ID_SHIFT                            (0x0U)
#define CSL_GIC500_GICD_CLRSPI_NSR_SPI_ID_RESETVAL                         (0x0U)

#define CSL_GIC500_GICD_CLRSPI_NSR_RESETVAL                                0x0
/* SETSPI_SR */

#define CSL_GIC500_GICD_SETSPI_SR_SPI_ID_MASK                              (0x3ffU)
#define CSL_GIC500_GICD_SETSPI_SR_SPI_ID_SHIFT                             (0x0U)
#define CSL_GIC500_GICD_SETSPI_SR_SPI_ID_RESETVAL                          (0x0U)

#define CSL_GIC500_GICD_SETSPI_SR_RESETVAL                                 0x0
/* CLRSPI_SR */

#define CSL_GIC500_GICD_CLRSPI_SR_SPI_ID_MASK                              (0x3ffU)
#define CSL_GIC500_GICD_CLRSPI_SR_SPI_ID_SHIFT                             (0x0U)
#define CSL_GIC500_GICD_CLRSPI_SR_SPI_ID_RESETVAL                          (0x0U)

#define CSL_GIC500_GICD_CLRSPI_SR_RESETVAL                                 0x0
/* IGROUPR_SGI_PPI */

#define CSL_GIC500_GICD_IGROUPR_SGI_PPI_GROUP_MASK                         (0xffffffffU)
#define CSL_GIC500_GICD_IGROUPR_SGI_PPI_GROUP_SHIFT                        (0x0U)
#define CSL_GIC500_GICD_IGROUPR_SGI_PPI_GROUP_RESETVAL                     (0x0U)

#define CSL_GIC500_GICD_IGROUPR_SGI_PPI_RESETVAL                           0x0
/* IGROUPR_SPI */

#define CSL_GIC500_GICD_IGROUPR_SPI_GROUP_MASK                             (0xffffffffU)
#define CSL_GIC500_GICD_IGROUPR_SPI_GROUP_SHIFT                            (0x0U)
#define CSL_GIC500_GICD_IGROUPR_SPI_GROUP_RESETVAL                         (0x0U)

#define CSL_GIC500_GICD_IGROUPR_SPI_RESETVAL                               0x0
/* ISENABLER_SGI_PPI */

#define CSL_GIC500_GICD_ISENABLER_SGI_PPI_ENABLE_MASK                      (0xffffffffU)
#define CSL_GIC500_GICD_ISENABLER_SGI_PPI_ENABLE_SHIFT                     (0x0U)
#define CSL_GIC500_GICD_ISENABLER_SGI_PPI_ENABLE_RESETVAL                  (0x0U)

#define CSL_GIC500_GICD_ISENABLER_SGI_PPI_RESETVAL                         0x0
/* ISENABLER_SPI */

#define CSL_GIC500_GICD_ISENABLER_SPI_ENABLE_MASK                          (0xffffffffU)
#define CSL_GIC500_GICD_ISENABLER_SPI_ENABLE_SHIFT                         (0x0U)
#define CSL_GIC500_GICD_ISENABLER_SPI_ENABLE_RESETVAL                      (0x0U)

#define CSL_GIC500_GICD_ISENABLER_SPI_RESETVAL                             0x0
/* ICENABLER_SGI_PPI */

#define CSL_GIC500_GICD_ICENABLER_SGI_PPI_ENABLE_MASK                      (0xffffffffU)
#define CSL_GIC500_GICD_ICENABLER_SGI_PPI_ENABLE_SHIFT                     (0x0U)
#define CSL_GIC500_GICD_ICENABLER_SGI_PPI_ENABLE_RESETVAL                  (0x0U)

#define CSL_GIC500_GICD_ICENABLER_SGI_PPI_RESETVAL                         0x0
/* ICENABLER_SPI */

#define CSL_GIC500_GICD_ICENABLER_SPI_ENABLE_MASK                          (0xffffffffU)
#define CSL_GIC500_GICD_ICENABLER_SPI_ENABLE_SHIFT                         (0x0U)
#define CSL_GIC500_GICD_ICENABLER_SPI_ENABLE_RESETVAL                      (0x0U)

#define CSL_GIC500_GICD_ICENABLER_SPI_RESETVAL                             0x0
/* ISPENDR_SGI_PPI */

#define CSL_GIC500_GICD_ISPENDR_SGI_PPI_PEND_MASK                          (0xffffffffU)
#define CSL_GIC500_GICD_ISPENDR_SGI_PPI_PEND_SHIFT                         (0x0U)
#define CSL_GIC500_GICD_ISPENDR_SGI_PPI_PEND_RESETVAL                      (0x0U)

#define CSL_GIC500_GICD_ISPENDR_SGI_PPI_RESETVAL                           0x0
/* ISPENDR_SPI */

#define CSL_GIC500_GICD_ISPENDR_SPI_PEND_MASK                              (0xffffffffU)
#define CSL_GIC500_GICD_ISPENDR_SPI_PEND_SHIFT                             (0x0U)
#define CSL_GIC500_GICD_ISPENDR_SPI_PEND_RESETVAL                          (0x0U)

#define CSL_GIC500_GICD_ISPENDR_SPI_RESETVAL                               0x0
/* ICPENDR_SGI_PPI */

#define CSL_GIC500_GICD_ICPENDR_SGI_PPI_PEND_MASK                          (0xffffffffU)
#define CSL_GIC500_GICD_ICPENDR_SGI_PPI_PEND_SHIFT                         (0x0U)
#define CSL_GIC500_GICD_ICPENDR_SGI_PPI_PEND_RESETVAL                      (0x0U)

#define CSL_GIC500_GICD_ICPENDR_SGI_PPI_RESETVAL                           0x0
/* ICPENDR_SPI */

#define CSL_GIC500_GICD_ICPENDR_SPI_PEND_MASK                              (0xffffffffU)
#define CSL_GIC500_GICD_ICPENDR_SPI_PEND_SHIFT                             (0x0U)
#define CSL_GIC500_GICD_ICPENDR_SPI_PEND_RESETVAL                          (0x0U)

#define CSL_GIC500_GICD_ICPENDR_SPI_RESETVAL                               0x0
/* ISACTIVER_SGI_PPI */

#define CSL_GIC500_GICD_ISACTIVER_SGI_PPI_SET_MASK                         (0xffffffffU)
#define CSL_GIC500_GICD_ISACTIVER_SGI_PPI_SET_SHIFT                        (0x0U)
#define CSL_GIC500_GICD_ISACTIVER_SGI_PPI_SET_RESETVAL                     (0x0U)

#define CSL_GIC500_GICD_ISACTIVER_SGI_PPI_RESETVAL                         0x0
/* ISACTIVER_SPI */

#define CSL_GIC500_GICD_ISACTIVER_SPI_SET_MASK                             (0xffffffffU)
#define CSL_GIC500_GICD_ISACTIVER_SPI_SET_SHIFT                            (0x0U)
#define CSL_GIC500_GICD_ISACTIVER_SPI_SET_RESETVAL                         (0x0U)

#define CSL_GIC500_GICD_ISACTIVER_SPI_RESETVAL                             0x0
/* ICACTIVER_SGI_PPI */

#define CSL_GIC500_GICD_ICACTIVER_SGI_PPI_SET_MASK                         (0xffffffffU)
#define CSL_GIC500_GICD_ICACTIVER_SGI_PPI_SET_SHIFT                        (0x0U)
#define CSL_GIC500_GICD_ICACTIVER_SGI_PPI_SET_RESETVAL                     (0x0U)

#define CSL_GIC500_GICD_ICACTIVER_SGI_PPI_RESETVAL                         0x0
/* ICACTIVER_SPI */

#define CSL_GIC500_GICD_ICACTIVER_SPI_SET_MASK                             (0xffffffffU)
#define CSL_GIC500_GICD_ICACTIVER_SPI_SET_SHIFT                            (0x0U)
#define CSL_GIC500_GICD_ICACTIVER_SPI_SET_RESETVAL                         (0x0U)

#define CSL_GIC500_GICD_ICACTIVER_SPI_RESETVAL                             0x0
/* IPRIORITYR_SGI_PPI */

#define CSL_GIC500_GICD_IPRIORITYR_SGI_PPI_PRIO_MASK                       (0xffffffffU)
#define CSL_GIC500_GICD_IPRIORITYR_SGI_PPI_PRIO_SHIFT                      (0x0U)
#define CSL_GIC500_GICD_IPRIORITYR_SGI_PPI_PRIO_RESETVAL                   (0x0U)

#define CSL_GIC500_GICD_IPRIORITYR_SGI_PPI_RESETVAL                        0x0
/* IPRIORITYR_SPI */

#define CSL_GIC500_GICD_IPRIORITYR_SPI_PRIO_MASK                           (0xffffffffU)
#define CSL_GIC500_GICD_IPRIORITYR_SPI_PRIO_SHIFT                          (0x0U)
#define CSL_GIC500_GICD_IPRIORITYR_SPI_PRIO_RESETVAL                       (0x0U)

#define CSL_GIC500_GICD_IPRIORITYR_SPI_RESETVAL                            0x0
/* ITARGETSR_SGI_PPI */

#define CSL_GIC500_GICD_ITARGETSR_SGI_PPI_CPU_ID_MASK                      (0xffffffffU)
#define CSL_GIC500_GICD_ITARGETSR_SGI_PPI_CPU_ID_SHIFT                     (0x0U)
#define CSL_GIC500_GICD_ITARGETSR_SGI_PPI_CPU_ID_RESETVAL                  (0x0U)

#define CSL_GIC500_GICD_ITARGETSR_SGI_PPI_RESETVAL                         0x0
/* ITARGETSR_SPI */

#define CSL_GIC500_GICD_ITARGETSR_SPI_CPU_ID_MASK                          (0xffffffffU)
#define CSL_GIC500_GICD_ITARGETSR_SPI_CPU_ID_SHIFT                         (0x0U)
#define CSL_GIC500_GICD_ITARGETSR_SPI_CPU_ID_RESETVAL                      (0x0U)

#define CSL_GIC500_GICD_ITARGETSR_SPI_RESETVAL                             0x0
/* ICFGR_SGI_PPI */

#define CSL_GIC500_GICD_ICFGR_SGI_PPI_INT_TYPE_MASK                        (0xffffffffU)
#define CSL_GIC500_GICD_ICFGR_SGI_PPI_INT_TYPE_SHIFT                       (0x0U)
#define CSL_GIC500_GICD_ICFGR_SGI_PPI_INT_TYPE_RESETVAL                    (0xaaaaaaaaU)

#define CSL_GIC500_GICD_ICFGR_SGI_PPI_RESETVAL                             0xaaaaaaaa
/* ICFGR_SPI */

#define CSL_GIC500_GICD_ICFGR_SPI_INT_TYPE_MASK                            (0xffffffffU)
#define CSL_GIC500_GICD_ICFGR_SPI_INT_TYPE_SHIFT                           (0x0U)
#define CSL_GIC500_GICD_ICFGR_SPI_INT_TYPE_RESETVAL                        (0xaaaaaaaaU)

#define CSL_GIC500_GICD_ICFGR_SPI_RESETVAL                                 0xaaaaaaaa
/* IGRPMODR_SGI_PPI */

#define CSL_GIC500_GICD_IGRPMODR_SGI_PPI_GROUP_MASK                        (0xffffffffU)
#define CSL_GIC500_GICD_IGRPMODR_SGI_PPI_GROUP_SHIFT                       (0x0U)
#define CSL_GIC500_GICD_IGRPMODR_SGI_PPI_GROUP_RESETVAL                    (0x0U)

#define CSL_GIC500_GICD_IGRPMODR_SGI_PPI_RESETVAL                          0x0
/* IGRPMODR_SPI */

#define CSL_GIC500_GICD_IGRPMODR_SPI_GROUP_MASK                            (0xffffffffU)
#define CSL_GIC500_GICD_IGRPMODR_SPI_GROUP_SHIFT                           (0x0U)
#define CSL_GIC500_GICD_IGRPMODR_SPI_GROUP_RESETVAL                        (0x0U)

#define CSL_GIC500_GICD_IGRPMODR_SPI_RESETVAL                              0x0
/* NSACR */

#define CSL_GIC500_GICD_NSACR_RESETVAL                                     0x0
/* SGIR */

#define CSL_GIC500_GICD_SGIR_SGIINTID_MASK                                 (0xfU)
#define CSL_GIC500_GICD_SGIR_SGIINTID_SHIFT                                (0x0U)
#define CSL_GIC500_GICD_SGIR_SGIINTID_RESETVAL                             (0x0U)

#define CSL_GIC500_GICD_SGIR_NSATT_MASK                                    (0x8000U)
#define CSL_GIC500_GICD_SGIR_NSATT_SHIFT                                   (0xfU)
#define CSL_GIC500_GICD_SGIR_NSATT_RESETVAL                                (0x0U)

#define CSL_GIC500_GICD_SGIR_CPUTARGETLIST_MASK                            (0xff0000U)
#define CSL_GIC500_GICD_SGIR_CPUTARGETLIST_SHIFT                           (0x10U)
#define CSL_GIC500_GICD_SGIR_CPUTARGETLIST_RESETVAL                        (0x0U)

#define CSL_GIC500_GICD_SGIR_TARGETLISTFILTER_MASK                         (0x3000000U)
#define CSL_GIC500_GICD_SGIR_TARGETLISTFILTER_SHIFT                        (0x18U)
#define CSL_GIC500_GICD_SGIR_TARGETLISTFILTER_RESETVAL                     (0x0U)

#define CSL_GIC500_GICD_SGIR_RESETVAL                                      0x0
/* CPENDSGIR */

#define CSL_GIC500_GICD_CPENDSGIR_RESETVAL                                 0x0
/* SPENDSGIR */

#define CSL_GIC500_GICD_SPENDSGIR_RESETVAL                                 0x0
/* ESTATUSR */

#define CSL_GIC500_GICD_ESTATUSR_SWRP_MASK                                 (0x80000000U)
#define CSL_GIC500_GICD_ESTATUSR_SWRP_SHIFT                                (0x1fU)
#define CSL_GIC500_GICD_ESTATUSR_SWRP_RESETVAL                             (0x0U)

#define CSL_GIC500_GICD_ESTATUSR_RESETVAL                                  0x0
/* ERRTESTR */

#define CSL_GIC500_GICD_ERRTESTR_ECC_FATAL_MASK                            (0x1U)
#define CSL_GIC500_GICD_ERRTESTR_ECC_FATAL_SHIFT                           (0x0U)
#define CSL_GIC500_GICD_ERRTESTR_ECC_FATAL_RESETVAL                        (0x0U)

#define CSL_GIC500_GICD_ERRTESTR_AXIM_ERR_MASK                             (0x2U)
#define CSL_GIC500_GICD_ERRTESTR_AXIM_ERR_SHIFT                            (0x1U)
#define CSL_GIC500_GICD_ERRTESTR_AXIM_ERR_RESETVAL                         (0x0U)

#define CSL_GIC500_GICD_ERRTESTR_RESETVAL                                  0x0
/* SPISR */

#define CSL_GIC500_GICD_SPISR_RESETVAL                                     0x0
/* PIDR4 */

#define CSL_GIC500_GICD_PIDR4_RESETVAL                                     0x44
/* PIDR5 */

#define CSL_GIC500_GICD_PIDR5_RESETVAL                                     0x0
/* PIDR6 */

#define CSL_GIC500_GICD_PIDR6_RESETVAL                                     0x0
/* PIDR7 */

#define CSL_GIC500_GICD_PIDR7_RESETVAL                                     0x0
/* PIDR0 */

#define CSL_GIC500_GICD_PIDR0_RESETVAL                                     0x92
/* PIDR1 */

#define CSL_GIC500_GICD_PIDR1_RESETVAL                                     0xb4
/* PIDR2 */

#define CSL_GIC500_GICD_PIDR2_RESETVAL                                     0x3b
/* PIDR3 */

#define CSL_GIC500_GICD_PIDR3_RESETVAL                                     0x0
/* CIDR0 */

#define CSL_GIC500_GICD_CIDR0_RESETVAL                                     0xd
/* CIDR1 */

#define CSL_GIC500_GICD_CIDR1_RESETVAL                                     0xf0
/* CIDR2 */

#define CSL_GIC500_GICD_CIDR2_RESETVAL                                     0x5
/* CIDR3 */

#define CSL_GIC500_GICD_CIDR3_RESETVAL                                     0xb1
/**************************************************************************
* Hardware Region  : MSG
**************************************************************************/

/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
   volatile uint8_t                          Rsvd0[64];
   volatile uint32_t                         SETSPI_NSR;
   volatile uint8_t                          Rsvd1[4];
   volatile uint32_t                         CLRSPI_NSR;
   volatile uint8_t                          Rsvd2[4];
   volatile uint32_t                         SETSPI_SR;
   volatile uint8_t                          Rsvd3[4];
   volatile uint32_t                         CLRSPI_SR;
} CSL_gic500_msgRegs;

/************************************************************************
* Register Macros
**************************************************************************/

#define CSL_GIC500_MSG_SETSPI_NSR                                     ( 0x10000U + 0x40U )
#define CSL_GIC500_MSG_CLRSPI_NSR                                     ( 0x10000U + 0x48U )
#define CSL_GIC500_MSG_SETSPI_SR                                      ( 0x10000U + 0x50U )
#define CSL_GIC500_MSG_CLRSPI_SR                                      ( 0x10000U + 0x58U )

/************************************************************************
* Field Definition Macros
**************************************************************************/

/* SETSPI_NSR */

#define CSL_GIC500_MSG_SETSPI_NSR_SPI_ID_MASK                              (0x3ffU)
#define CSL_GIC500_MSG_SETSPI_NSR_SPI_ID_SHIFT                             (0x0U)
#define CSL_GIC500_MSG_SETSPI_NSR_SPI_ID_RESETVAL                          (0x0U)

#define CSL_GIC500_MSG_SETSPI_NSR_RESETVAL                                 0x0
/* CLRSPI_NSR */

#define CSL_GIC500_MSG_CLRSPI_NSR_SPI_ID_MASK                              (0x3ffU)
#define CSL_GIC500_MSG_CLRSPI_NSR_SPI_ID_SHIFT                             (0x0U)
#define CSL_GIC500_MSG_CLRSPI_NSR_SPI_ID_RESETVAL                          (0x0U)

#define CSL_GIC500_MSG_CLRSPI_NSR_RESETVAL                                 0x0
/* SETSPI_SR */

#define CSL_GIC500_MSG_SETSPI_SR_SPI_ID_MASK                               (0x3ffU)
#define CSL_GIC500_MSG_SETSPI_SR_SPI_ID_SHIFT                              (0x0U)
#define CSL_GIC500_MSG_SETSPI_SR_SPI_ID_RESETVAL                           (0x0U)

#define CSL_GIC500_MSG_SETSPI_SR_RESETVAL                                  0x0
/* CLRSPI_SR */

#define CSL_GIC500_MSG_CLRSPI_SR_SPI_ID_MASK                               (0x3ffU)
#define CSL_GIC500_MSG_CLRSPI_SR_SPI_ID_SHIFT                              (0x0U)
#define CSL_GIC500_MSG_CLRSPI_SR_SPI_ID_RESETVAL                           (0x0U)

#define CSL_GIC500_MSG_CLRSPI_SR_RESETVAL                                  0x0
/**************************************************************************
* Hardware Region  : ITS
**************************************************************************/

/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
   volatile uint32_t                         CTLR;
   volatile uint32_t                         IIDR;
   volatile uint32_t                         TYPER_LOWER;
   volatile uint32_t                         TYPER_UPPER;
   volatile uint8_t                          Rsvd0[112];
   volatile uint32_t                         CBASER_LOWER;
   volatile uint32_t                         CBASER_UPPER;
   volatile uint32_t                         CWRITER_LOWER;
   volatile uint32_t                         CWRITER_UPPER;
   volatile uint32_t                         CREADR_LOWER;
   volatile uint32_t                         CREADR_UPPER;
   volatile uint8_t                          Rsvd1[104];
   volatile uint32_t                         BASER0_LOWER;
   volatile uint32_t                         BASER0_UPPER;
   volatile uint8_t                          Rsvd2[48888];
   volatile uint32_t                         TRKCTLR;
   volatile uint32_t                         TRKR;
   volatile uint32_t                         TRKDIDR;
   volatile uint32_t                         TRKPIDR;
   volatile uint32_t                         TRKVIDR;
   volatile uint32_t                         TRKTGTR;
   volatile uint32_t                         TRKICR;
   volatile uint32_t                         TRKLCR;
   volatile uint8_t                          Rsvd3[16304];
   volatile uint32_t                         PIDR4;
   volatile uint32_t                         PIDR5;
   volatile uint32_t                         PIDR6;
   volatile uint32_t                         PIDR7;
   volatile uint32_t                         PIDR0;
   volatile uint32_t                         PIDR1;
   volatile uint32_t                         PIDR2;
   volatile uint32_t                         PIDR3;
   volatile uint32_t                         CIDR0;
   volatile uint32_t                         CIDR1;
   volatile uint32_t                         CIDR2;
   volatile uint32_t                         CIDR3;
} CSL_gic500_itsRegs;

/************************************************************************
* Register Macros
**************************************************************************/

#define CSL_GIC500_ITS_CTLR                                           ( 0x20000U )
#define CSL_GIC500_ITS_IIDR                                           ( 0x20000U + 0x4U )
#define CSL_GIC500_ITS_TYPER_LOWER                                    ( 0x20000U + 0x8U )
#define CSL_GIC500_ITS_TYPER_UPPER                                    ( 0x20000U + 0xcU )
#define CSL_GIC500_ITS_CBASER_LOWER                                   ( 0x20000U + 0x80U )
#define CSL_GIC500_ITS_CBASER_UPPER                                   ( 0x20000U + 0x84U )
#define CSL_GIC500_ITS_CWRITER_LOWER                                  ( 0x20000U + 0x88U )
#define CSL_GIC500_ITS_CWRITER_UPPER                                  ( 0x20000U + 0x8cU )
#define CSL_GIC500_ITS_CREADR_LOWER                                   ( 0x20000U + 0x90U )
#define CSL_GIC500_ITS_CREADR_UPPER                                   ( 0x20000U + 0x94U )
#define CSL_GIC500_ITS_BASER0_LOWER                                   ( 0x20000U + 0x100U )
#define CSL_GIC500_ITS_BASER0_UPPER                                   ( 0x20000U + 0x104U )
#define CSL_GIC500_ITS_TRKCTLR                                        ( 0x20000U + 0xc000U )
#define CSL_GIC500_ITS_TRKR                                           ( 0x20000U + 0xc004U )
#define CSL_GIC500_ITS_TRKDIDR                                        ( 0x20000U + 0xc008U )
#define CSL_GIC500_ITS_TRKPIDR                                        ( 0x20000U + 0xc00cU )
#define CSL_GIC500_ITS_TRKVIDR                                        ( 0x20000U + 0xc010U )
#define CSL_GIC500_ITS_TRKTGTR                                        ( 0x20000U + 0xc014U )
#define CSL_GIC500_ITS_TRKICR                                         ( 0x20000U + 0xc018U )
#define CSL_GIC500_ITS_TRKLCR                                         ( 0x20000U + 0xc01cU )
#define CSL_GIC500_ITS_PIDR4                                          ( 0x20000U + 0xffd0U )
#define CSL_GIC500_ITS_PIDR5                                          ( 0x20000U + 0xffd4U )
#define CSL_GIC500_ITS_PIDR6                                          ( 0x20000U + 0xffd8U )
#define CSL_GIC500_ITS_PIDR7                                          ( 0x20000U + 0xffdcU )
#define CSL_GIC500_ITS_PIDR0                                          ( 0x20000U + 0xffe0U )
#define CSL_GIC500_ITS_PIDR1                                          ( 0x20000U + 0xffe4U )
#define CSL_GIC500_ITS_PIDR2                                          ( 0x20000U + 0xffe8U )
#define CSL_GIC500_ITS_PIDR3                                          ( 0x20000U + 0xffecU )
#define CSL_GIC500_ITS_CIDR0                                          ( 0x20000U + 0xfff0U )
#define CSL_GIC500_ITS_CIDR1                                          ( 0x20000U + 0xfff4U )
#define CSL_GIC500_ITS_CIDR2                                          ( 0x20000U + 0xfff8U )
#define CSL_GIC500_ITS_CIDR3                                          ( 0x20000U + 0xfffcU )

/************************************************************************
* Field Definition Macros
**************************************************************************/

/* CTLR */

#define CSL_GIC500_ITS_CTLR_ENABLED_MASK                                   (0x1U)
#define CSL_GIC500_ITS_CTLR_ENABLED_SHIFT                                  (0x0U)
#define CSL_GIC500_ITS_CTLR_ENABLED_RESETVAL                               (0x0U)

#define CSL_GIC500_ITS_CTLR_QUIESCENT_MASK                                 (0x80000000U)
#define CSL_GIC500_ITS_CTLR_QUIESCENT_SHIFT                                (0x1fU)
#define CSL_GIC500_ITS_CTLR_QUIESCENT_RESETVAL                             (0x1U)

#define CSL_GIC500_ITS_CTLR_RESETVAL                                       0x80000000
/* IIDR */

#define CSL_GIC500_ITS_IIDR_IMPLEMENTER_MASK                               (0xfffU)
#define CSL_GIC500_ITS_IIDR_IMPLEMENTER_SHIFT                              (0x0U)
#define CSL_GIC500_ITS_IIDR_IMPLEMENTER_RESETVAL                           (0x43bU)

#define CSL_GIC500_ITS_IIDR_REVISION_MASK                                  (0xf000U)
#define CSL_GIC500_ITS_IIDR_REVISION_SHIFT                                 (0xcU)
#define CSL_GIC500_ITS_IIDR_REVISION_RESETVAL                              (0x1U)

#define CSL_GIC500_ITS_IIDR_VARIANT_MASK                                   (0xf0000U)
#define CSL_GIC500_ITS_IIDR_VARIANT_SHIFT                                  (0x10U)
#define CSL_GIC500_ITS_IIDR_VARIANT_RESETVAL                               (0x1U)

#define CSL_GIC500_ITS_IIDR_PRODUCTID_MASK                                 (0xff000000U)
#define CSL_GIC500_ITS_IIDR_PRODUCTID_SHIFT                                (0x18U)
#define CSL_GIC500_ITS_IIDR_PRODUCTID_RESETVAL                             (0x0U)

#define CSL_GIC500_ITS_IIDR_RESETVAL                                       0x1143b
/* TYPER_LOWER */

#define CSL_GIC500_ITS_TYPER_LOWER_PLPIS_MASK                              (0x1U)
#define CSL_GIC500_ITS_TYPER_LOWER_PLPIS_SHIFT                             (0x0U)
#define CSL_GIC500_ITS_TYPER_LOWER_PLPIS_RESETVAL                          (0x1U)

#define CSL_GIC500_ITS_TYPER_LOWER_VLPIS_MASK                              (0x2U)
#define CSL_GIC500_ITS_TYPER_LOWER_VLPIS_SHIFT                             (0x1U)
#define CSL_GIC500_ITS_TYPER_LOWER_VLPIS_RESETVAL                          (0x0U)

#define CSL_GIC500_ITS_TYPER_LOWER_DISTRIBUTED_MASK                        (0x8U)
#define CSL_GIC500_ITS_TYPER_LOWER_DISTRIBUTED_SHIFT                       (0x3U)
#define CSL_GIC500_ITS_TYPER_LOWER_DISTRIBUTED_RESETVAL                    (0x0U)

#define CSL_GIC500_ITS_TYPER_LOWER_ITT_ENTRY_SIZE_MASK                     (0xf0U)
#define CSL_GIC500_ITS_TYPER_LOWER_ITT_ENTRY_SIZE_SHIFT                    (0x4U)
#define CSL_GIC500_ITS_TYPER_LOWER_ITT_ENTRY_SIZE_RESETVAL                 (0x3U)

#define CSL_GIC500_ITS_TYPER_LOWER_IDBITS_MASK                             (0x1f00U)
#define CSL_GIC500_ITS_TYPER_LOWER_IDBITS_SHIFT                            (0x8U)
#define CSL_GIC500_ITS_TYPER_LOWER_IDBITS_RESETVAL                         (0xfU)

#define CSL_GIC500_ITS_TYPER_LOWER_DEVBITS_MASK                            (0x3e000U)
#define CSL_GIC500_ITS_TYPER_LOWER_DEVBITS_SHIFT                           (0xdU)
#define CSL_GIC500_ITS_TYPER_LOWER_DEVBITS_RESETVAL                        (0x14U)

#define CSL_GIC500_ITS_TYPER_LOWER_PTA_MASK                                (0x80000U)
#define CSL_GIC500_ITS_TYPER_LOWER_PTA_SHIFT                               (0x13U)
#define CSL_GIC500_ITS_TYPER_LOWER_PTA_RESETVAL                            (0x0U)

#define CSL_GIC500_ITS_TYPER_LOWER_HCC_MASK                                (0xff000000U)
#define CSL_GIC500_ITS_TYPER_LOWER_HCC_SHIFT                               (0x18U)
#define CSL_GIC500_ITS_TYPER_LOWER_HCC_RESETVAL                            (0x5U)

#define CSL_GIC500_ITS_TYPER_LOWER_RESETVAL                                0x5028f31
/* TYPER_UPPER */

#define CSL_GIC500_ITS_TYPER_UPPER_RESETVAL                                0x0
/* CBASER_LOWER */

#define CSL_GIC500_ITS_CBASER_LOWER_SIZE_MASK                              (0xffU)
#define CSL_GIC500_ITS_CBASER_LOWER_SIZE_SHIFT                             (0x0U)
#define CSL_GIC500_ITS_CBASER_LOWER_SIZE_RESETVAL                          (0x0U)

#define CSL_GIC500_ITS_CBASER_LOWER__PHYSICAL_ADDRESS__31_12__MASK         (0xfffff000U)
#define CSL_GIC500_ITS_CBASER_LOWER__PHYSICAL_ADDRESS__31_12__SHIFT        (0xcU)
#define CSL_GIC500_ITS_CBASER_LOWER__PHYSICAL_ADDRESS__31_12__RESETVAL     (0x0U)

#define CSL_GIC500_ITS_CBASER_LOWER_RESETVAL                               0x0
/* CBASER_UPPER */

#define CSL_GIC500_ITS_CBASER_UPPER__PHYSICAL_ADDRESS__47_32__MASK         (0xffffU)
#define CSL_GIC500_ITS_CBASER_UPPER__PHYSICAL_ADDRESS__47_32__SHIFT        (0x0U)
#define CSL_GIC500_ITS_CBASER_UPPER__PHYSICAL_ADDRESS__47_32__RESETVAL     (0x0U)

#define CSL_GIC500_ITS_CBASER_UPPER_CACHEABILITY_MASK                      (0x38000000U)
#define CSL_GIC500_ITS_CBASER_UPPER_CACHEABILITY_SHIFT                     (0x1bU)
#define CSL_GIC500_ITS_CBASER_UPPER_CACHEABILITY_RESETVAL                  (0x0U)

#define CSL_GIC500_ITS_CBASER_UPPER_VALID_MASK                             (0x80000000U)
#define CSL_GIC500_ITS_CBASER_UPPER_VALID_SHIFT                            (0x1fU)
#define CSL_GIC500_ITS_CBASER_UPPER_VALID_RESETVAL                         (0x0U)

#define CSL_GIC500_ITS_CBASER_UPPER_RESETVAL                               0x0
/* CWRITER_LOWER */

#define CSL_GIC500_ITS_CWRITER_LOWER_OFFSET_MASK                           (0xfffe0U)
#define CSL_GIC500_ITS_CWRITER_LOWER_OFFSET_SHIFT                          (0x5U)
#define CSL_GIC500_ITS_CWRITER_LOWER_OFFSET_RESETVAL                       (0x0U)

#define CSL_GIC500_ITS_CWRITER_LOWER_RESETVAL                              0x0
/* CWRITER_UPPER */

#define CSL_GIC500_ITS_CWRITER_UPPER_RESETVAL                              0x0
/* CREADR_LOWER */

#define CSL_GIC500_ITS_CREADR_LOWER_OFFSET_MASK                            (0xfffe0U)
#define CSL_GIC500_ITS_CREADR_LOWER_OFFSET_SHIFT                           (0x5U)
#define CSL_GIC500_ITS_CREADR_LOWER_OFFSET_RESETVAL                        (0x0U)

#define CSL_GIC500_ITS_CREADR_LOWER_RESETVAL                               0x0
/* CREADR_UPPER */

#define CSL_GIC500_ITS_CREADR_UPPER_RESETVAL                               0x0
/* BASER0_LOWER */

#define CSL_GIC500_ITS_BASER0_LOWER_SIZE_MASK                              (0xffU)
#define CSL_GIC500_ITS_BASER0_LOWER_SIZE_SHIFT                             (0x0U)
#define CSL_GIC500_ITS_BASER0_LOWER_SIZE_RESETVAL                          (0x0U)

#define CSL_GIC500_ITS_BASER0_LOWER_PAGE_SIZE_MASK                         (0x300U)
#define CSL_GIC500_ITS_BASER0_LOWER_PAGE_SIZE_SHIFT                        (0x8U)
#define CSL_GIC500_ITS_BASER0_LOWER_PAGE_SIZE_RESETVAL                     (0x0U)

#define CSL_GIC500_ITS_BASER0_LOWER__PHYSICAL_ADDRESS__31_12__MASK         (0xfffff000U)
#define CSL_GIC500_ITS_BASER0_LOWER__PHYSICAL_ADDRESS__31_12__SHIFT        (0xcU)
#define CSL_GIC500_ITS_BASER0_LOWER__PHYSICAL_ADDRESS__31_12__RESETVAL     (0x0U)

#define CSL_GIC500_ITS_BASER0_LOWER_RESETVAL                               0x0
/* BASER0_UPPER */

#define CSL_GIC500_ITS_BASER0_UPPER__PHYSICAL_ADDRESS__47_32__MASK         (0xffffU)
#define CSL_GIC500_ITS_BASER0_UPPER__PHYSICAL_ADDRESS__47_32__SHIFT        (0x0U)
#define CSL_GIC500_ITS_BASER0_UPPER__PHYSICAL_ADDRESS__47_32__RESETVAL     (0x0U)

#define CSL_GIC500_ITS_BASER0_UPPER_ENTRY_SIZE_MASK                        (0xff0000U)
#define CSL_GIC500_ITS_BASER0_UPPER_ENTRY_SIZE_SHIFT                       (0x10U)
#define CSL_GIC500_ITS_BASER0_UPPER_ENTRY_SIZE_RESETVAL                    (0x7U)

#define CSL_GIC500_ITS_BASER0_UPPER_TYPE_MASK                              (0x7000000U)
#define CSL_GIC500_ITS_BASER0_UPPER_TYPE_SHIFT                             (0x18U)
#define CSL_GIC500_ITS_BASER0_UPPER_TYPE_RESETVAL                          (0x1U)

#define CSL_GIC500_ITS_BASER0_UPPER_RESETVAL                               0x1070000
/* TRKCTLR */

#define CSL_GIC500_ITS_TRKCTLR_CACHE_COUNT_RESET_MASK                      (0x1U)
#define CSL_GIC500_ITS_TRKCTLR_CACHE_COUNT_RESET_SHIFT                     (0x0U)
#define CSL_GIC500_ITS_TRKCTLR_CACHE_COUNT_RESET_RESETVAL                  (0x0U)

#define CSL_GIC500_ITS_TRKCTLR_ITS_TRACK_MASK                              (0x2U)
#define CSL_GIC500_ITS_TRKCTLR_ITS_TRACK_SHIFT                             (0x1U)
#define CSL_GIC500_ITS_TRKCTLR_ITS_TRACK_RESETVAL                          (0x0U)

#define CSL_GIC500_ITS_TRKCTLR_RESETVAL                                    0x0
/* TRKR */

#define CSL_GIC500_ITS_TRKR_DID_OUT_OF_RANGE_MASK                          (0x1U)
#define CSL_GIC500_ITS_TRKR_DID_OUT_OF_RANGE_SHIFT                         (0x0U)
#define CSL_GIC500_ITS_TRKR_DID_OUT_OF_RANGE_RESETVAL                      (0x0U)

#define CSL_GIC500_ITS_TRKR_DID_UNMAPPED_MASK                              (0x2U)
#define CSL_GIC500_ITS_TRKR_DID_UNMAPPED_SHIFT                             (0x1U)
#define CSL_GIC500_ITS_TRKR_DID_UNMAPPED_RESETVAL                          (0x0U)

#define CSL_GIC500_ITS_TRKR_INPUT_ID_OUT_OF_RANGE_MASK                     (0x4U)
#define CSL_GIC500_ITS_TRKR_INPUT_ID_OUT_OF_RANGE_SHIFT                    (0x2U)
#define CSL_GIC500_ITS_TRKR_INPUT_ID_OUT_OF_RANGE_RESETVAL                 (0x0U)

#define CSL_GIC500_ITS_TRKR_DID_ID_NOT_MAPPED_MASK                         (0x8U)
#define CSL_GIC500_ITS_TRKR_DID_ID_NOT_MAPPED_SHIFT                        (0x3U)
#define CSL_GIC500_ITS_TRKR_DID_ID_NOT_MAPPED_RESETVAL                     (0x0U)

#define CSL_GIC500_ITS_TRKR_TARGET_CPU_OUT_OF_RANGE_MASK                   (0x10U)
#define CSL_GIC500_ITS_TRKR_TARGET_CPU_OUT_OF_RANGE_SHIFT                  (0x4U)
#define CSL_GIC500_ITS_TRKR_TARGET_CPU_OUT_OF_RANGE_RESETVAL               (0x0U)

#define CSL_GIC500_ITS_TRKR_TRANSLATED_ID_OUT_OF_RANGE_MASK                (0x20U)
#define CSL_GIC500_ITS_TRKR_TRANSLATED_ID_OUT_OF_RANGE_SHIFT               (0x5U)
#define CSL_GIC500_ITS_TRKR_TRANSLATED_ID_OUT_OF_RANGE_RESETVAL            (0x0U)

#define CSL_GIC500_ITS_TRKR_RESETVAL                                       0x0
/* TRKDIDR */

#define CSL_GIC500_ITS_TRKDIDR_DID_MASK                                    (0xfffffU)
#define CSL_GIC500_ITS_TRKDIDR_DID_SHIFT                                   (0x0U)
#define CSL_GIC500_ITS_TRKDIDR_DID_RESETVAL                                (0x0U)

#define CSL_GIC500_ITS_TRKDIDR_RESETVAL                                    0x0
/* TRKPIDR */

#define CSL_GIC500_ITS_TRKPIDR_TRANSLATED_ID_MASK                          (0xffffU)
#define CSL_GIC500_ITS_TRKPIDR_TRANSLATED_ID_SHIFT                         (0x0U)
#define CSL_GIC500_ITS_TRKPIDR_TRANSLATED_ID_RESETVAL                      (0x0U)

#define CSL_GIC500_ITS_TRKPIDR_RESETVAL                                    0x0
/* TRKVIDR */

#define CSL_GIC500_ITS_TRKVIDR_INPUT_ID_MASK                               (0xffffU)
#define CSL_GIC500_ITS_TRKVIDR_INPUT_ID_SHIFT                              (0x0U)
#define CSL_GIC500_ITS_TRKVIDR_INPUT_ID_RESETVAL                           (0x0U)

#define CSL_GIC500_ITS_TRKVIDR_RESETVAL                                    0x0
/* TRKTGTR */

#define CSL_GIC500_ITS_TRKTGTR_TARGET_CPU_MASK                             (0x7fU)
#define CSL_GIC500_ITS_TRKTGTR_TARGET_CPU_SHIFT                            (0x0U)
#define CSL_GIC500_ITS_TRKTGTR_TARGET_CPU_RESETVAL                         (0x0U)

#define CSL_GIC500_ITS_TRKTGTR_RESETVAL                                    0x0
/* TRKICR */

#define CSL_GIC500_ITS_TRKICR_ITE_CACHE_MISSES_MASK                        (0xffffU)
#define CSL_GIC500_ITS_TRKICR_ITE_CACHE_MISSES_SHIFT                       (0x0U)
#define CSL_GIC500_ITS_TRKICR_ITE_CACHE_MISSES_RESETVAL                    (0x0U)

#define CSL_GIC500_ITS_TRKICR_ITE_CACHE_HITS_MASK                          (0xffff0000U)
#define CSL_GIC500_ITS_TRKICR_ITE_CACHE_HITS_SHIFT                         (0x10U)
#define CSL_GIC500_ITS_TRKICR_ITE_CACHE_HITS_RESETVAL                      (0x0U)

#define CSL_GIC500_ITS_TRKICR_RESETVAL                                     0x0
/* TRKLCR */

#define CSL_GIC500_ITS_TRKLCR_LPI_CACHE_MISSES_MASK                        (0xffffU)
#define CSL_GIC500_ITS_TRKLCR_LPI_CACHE_MISSES_SHIFT                       (0x0U)
#define CSL_GIC500_ITS_TRKLCR_LPI_CACHE_MISSES_RESETVAL                    (0x0U)

#define CSL_GIC500_ITS_TRKLCR_LPI_CACHE_HITS_MASK                          (0xffff0000U)
#define CSL_GIC500_ITS_TRKLCR_LPI_CACHE_HITS_SHIFT                         (0x10U)
#define CSL_GIC500_ITS_TRKLCR_LPI_CACHE_HITS_RESETVAL                      (0x0U)

#define CSL_GIC500_ITS_TRKLCR_RESETVAL                                     0x0
/* PIDR4 */

#define CSL_GIC500_ITS_PIDR4_RESETVAL                                      0x44
/* PIDR5 */

#define CSL_GIC500_ITS_PIDR5_RESETVAL                                      0x0
/* PIDR6 */

#define CSL_GIC500_ITS_PIDR6_RESETVAL                                      0x0
/* PIDR7 */

#define CSL_GIC500_ITS_PIDR7_RESETVAL                                      0x0
/* PIDR0 */

#define CSL_GIC500_ITS_PIDR0_RESETVAL                                      0x94
/* PIDR1 */

#define CSL_GIC500_ITS_PIDR1_RESETVAL                                      0xb4
/* PIDR2 */

#define CSL_GIC500_ITS_PIDR2_RESETVAL                                      0x3b
/* PIDR3 */

#define CSL_GIC500_ITS_PIDR3_RESETVAL                                      0x0
/* CIDR0 */

#define CSL_GIC500_ITS_CIDR0_RESETVAL                                      0xd
/* CIDR1 */

#define CSL_GIC500_ITS_CIDR1_RESETVAL                                      0xf0
/* CIDR2 */

#define CSL_GIC500_ITS_CIDR2_RESETVAL                                      0x5
/* CIDR3 */

#define CSL_GIC500_ITS_CIDR3_RESETVAL                                      0xb1
/**************************************************************************
* Hardware Region  : GICR
**************************************************************************/

/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
   volatile uint32_t                         CTLR;
   volatile uint32_t                         IIDR;
   volatile uint32_t                         TYPER_LOWER;
   volatile uint32_t                         TYPER_UPPER;
   volatile uint32_t                         STATUSR;
   volatile uint32_t                         WAKER;
   volatile uint8_t                          Rsvd0[40];
   volatile uint32_t                         SETLPIR_LOWER;
   volatile uint32_t                         SETLPIR_UPPER;
   volatile uint32_t                         CLRLPIR_LOWER;
   volatile uint32_t                         CLRLPIR_UPPER;
   volatile uint8_t                          Rsvd1[32];
   volatile uint32_t                         PROPBASER_LOWER;
   volatile uint32_t                         PROPBASER_UPPER;
   volatile uint32_t                         PENDBASER_LOWER;
   volatile uint32_t                         PENDBASER_UPPER;
   volatile uint8_t                          Rsvd2[32];
   volatile uint32_t                         INVLPIR_LOWER;
   volatile uint32_t                         INVLPIR_UPPER;
   volatile uint8_t                          Rsvd3[8];
   volatile uint32_t                         INVALLR_LOWER;
   volatile uint32_t                         INVALLR_UPPER;
   volatile uint8_t                          Rsvd4[8];
   volatile uint32_t                         SYNCR;
   volatile uint8_t                          Rsvd5[65292];
   volatile uint32_t                         PIDR4;
   volatile uint32_t                         PIDR5;
   volatile uint32_t                         PIDR6;
   volatile uint32_t                         PIDR7;
   volatile uint32_t                         PIDR0;
   volatile uint32_t                         PIDR1;
   volatile uint32_t                         PIDR2;
   volatile uint32_t                         PIDR3;
   volatile uint32_t                         CIDR0;
   volatile uint32_t                         CIDR1;
   volatile uint32_t                         CIDR2;
   volatile uint32_t                         CIDR3;
} CSL_gic500_gicrRegs_core_control;

/************************************************************************
* Register Macros
**************************************************************************/

#define CSL_GIC500_GICR_CORE_CONTROL_OFFSET     (0x40000U)
#define CSL_GIC500_GICR_CORE_SGI_PPI_OFFSET     (0x50000U)

#define CSL_GIC500_GICR_CORE_CONTROL_CTLR(CORE)                       ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) )
#define CSL_GIC500_GICR_CORE_CONTROL_IIDR(CORE)                       ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0x4U )
#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_LOWER(CORE)                ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0x8U )
#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_UPPER(CORE)                ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0xcU )
#define CSL_GIC500_GICR_CORE_CONTROL_STATUSR(CORE)                    ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0x10U )
#define CSL_GIC500_GICR_CORE_CONTROL_WAKER(CORE)                      ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0x14U )
#define CSL_GIC500_GICR_CORE_CONTROL_SETLPIR_LOWER(CORE)              ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0x40U )
#define CSL_GIC500_GICR_CORE_CONTROL_SETLPIR_UPPER(CORE)              ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0x44U )
#define CSL_GIC500_GICR_CORE_CONTROL_CLRLPIR_LOWER(CORE)              ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0x48U )
#define CSL_GIC500_GICR_CORE_CONTROL_CLRLPIR_UPPER(CORE)              ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0x4cU )
#define CSL_GIC500_GICR_CORE_CONTROL_PROPBASER_LOWER(CORE)            ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0x70U )
#define CSL_GIC500_GICR_CORE_CONTROL_PROPBASER_UPPER(CORE)            ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0x74U )
#define CSL_GIC500_GICR_CORE_CONTROL_PENDBASER_LOWER(CORE)            ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0x78U )
#define CSL_GIC500_GICR_CORE_CONTROL_PENDBASER_UPPER(CORE)            ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0x7cU )
#define CSL_GIC500_GICR_CORE_CONTROL_INVLPIR_LOWER(CORE)              ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0xa0U )
#define CSL_GIC500_GICR_CORE_CONTROL_INVLPIR_UPPER(CORE)              ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0xa4U )
#define CSL_GIC500_GICR_CORE_CONTROL_INVALLR_LOWER(CORE)              ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0xb0U )
#define CSL_GIC500_GICR_CORE_CONTROL_INVALLR_UPPER(CORE)              ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0xb4U )
#define CSL_GIC500_GICR_CORE_CONTROL_SYNCR(CORE)                      ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0xc0U )
#define CSL_GIC500_GICR_CORE_CONTROL_PIDR4(CORE)                      ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0xffd0U )
#define CSL_GIC500_GICR_CORE_CONTROL_PIDR5(CORE)                      ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0xffd4U )
#define CSL_GIC500_GICR_CORE_CONTROL_PIDR6(CORE)                      ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0xffd8U )
#define CSL_GIC500_GICR_CORE_CONTROL_PIDR7(CORE)                      ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0xffdcU )
#define CSL_GIC500_GICR_CORE_CONTROL_PIDR0(CORE)                      ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0xffe0U )
#define CSL_GIC500_GICR_CORE_CONTROL_PIDR1(CORE)                      ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0xffe4U )
#define CSL_GIC500_GICR_CORE_CONTROL_PIDR2(CORE)                      ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0xffe8U )
#define CSL_GIC500_GICR_CORE_CONTROL_PIDR3(CORE)                      ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0xffecU )
#define CSL_GIC500_GICR_CORE_CONTROL_CIDR0(CORE)                      ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0xfff0U )
#define CSL_GIC500_GICR_CORE_CONTROL_CIDR1(CORE)                      ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0xfff4U )
#define CSL_GIC500_GICR_CORE_CONTROL_CIDR2(CORE)                      ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0xfff8U )
#define CSL_GIC500_GICR_CORE_CONTROL_CIDR3(CORE)                      ( CSL_GIC500_GICR_CORE_CONTROL_OFFSET +  ((CORE)*0x20000U) + 0xfffcU )

/************************************************************************
* Field Definition Macros
**************************************************************************/

/* CTLR */

#define CSL_GIC500_GICR_CORE_CONTROL_CTLR_REGISTER_WRITE_PENDING_MASK      (0x8U)
#define CSL_GIC500_GICR_CORE_CONTROL_CTLR_REGISTER_WRITE_PENDING_SHIFT     (0x3U)
#define CSL_GIC500_GICR_CORE_CONTROL_CTLR_REGISTER_WRITE_PENDING_RESETVAL  (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_CTLR_UPSTREAM_WRITE_PENDING_MASK      (0x80000000U)
#define CSL_GIC500_GICR_CORE_CONTROL_CTLR_UPSTREAM_WRITE_PENDING_SHIFT     (0x1fU)
#define CSL_GIC500_GICR_CORE_CONTROL_CTLR_UPSTREAM_WRITE_PENDING_RESETVAL  (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_CTLR_RESETVAL                         0x0
/* IIDR */

#define CSL_GIC500_GICR_CORE_CONTROL_IIDR_IMPLEMENTER_MASK                 (0xfffU)
#define CSL_GIC500_GICR_CORE_CONTROL_IIDR_IMPLEMENTER_SHIFT                (0x0U)
#define CSL_GIC500_GICR_CORE_CONTROL_IIDR_IMPLEMENTER_RESETVAL             (0x43bU)

#define CSL_GIC500_GICR_CORE_CONTROL_IIDR_REVISION_MASK                    (0xf000U)
#define CSL_GIC500_GICR_CORE_CONTROL_IIDR_REVISION_SHIFT                   (0xcU)
#define CSL_GIC500_GICR_CORE_CONTROL_IIDR_REVISION_RESETVAL                (0x1U)

#define CSL_GIC500_GICR_CORE_CONTROL_IIDR_VARIANT_MASK                     (0xf0000U)
#define CSL_GIC500_GICR_CORE_CONTROL_IIDR_VARIANT_SHIFT                    (0x10U)
#define CSL_GIC500_GICR_CORE_CONTROL_IIDR_VARIANT_RESETVAL                 (0x1U)

#define CSL_GIC500_GICR_CORE_CONTROL_IIDR_PRODUCTID_MASK                   (0xff000000U)
#define CSL_GIC500_GICR_CORE_CONTROL_IIDR_PRODUCTID_SHIFT                  (0x18U)
#define CSL_GIC500_GICR_CORE_CONTROL_IIDR_PRODUCTID_RESETVAL               (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_IIDR_RESETVAL                         0x1143b
/* TYPER_LOWER */

#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_LOWER_PLPIS_MASK                (0x1U)
#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_LOWER_PLPIS_SHIFT               (0x0U)
#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_LOWER_PLPIS_RESETVAL            (0x1U)

#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_LOWER_VLPIS_MASK                (0x2U)
#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_LOWER_VLPIS_SHIFT               (0x1U)
#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_LOWER_VLPIS_RESETVAL            (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_LOWER_DISTRIBUTED_MASK          (0x8U)
#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_LOWER_DISTRIBUTED_SHIFT         (0x3U)
#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_LOWER_DISTRIBUTED_RESETVAL      (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_LOWER_LAST_MASK                 (0x10U)
#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_LOWER_LAST_SHIFT                (0x4U)
#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_LOWER_LAST_RESETVAL             (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_LOWER_PROCESSOR_NUMBER_MASK     (0xffff00U)
#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_LOWER_PROCESSOR_NUMBER_SHIFT    (0x8U)
#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_LOWER_PROCESSOR_NUMBER_RESETVAL (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_LOWER_RESETVAL                  0x1
/* TYPER_UPPER */

#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_UPPER_A0_MASK                   (0xffU)
#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_UPPER_A0_SHIFT                  (0x0U)
#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_UPPER_A0_RESETVAL               (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_UPPER_A1_MASK                   (0xff00U)
#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_UPPER_A1_SHIFT                  (0x8U)
#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_UPPER_A1_RESETVAL               (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_UPPER_A2_MASK                   (0xff0000U)
#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_UPPER_A2_SHIFT                  (0x10U)
#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_UPPER_A2_RESETVAL               (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_UPPER_A3_MASK                   (0xff000000U)
#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_UPPER_A3_SHIFT                  (0x18U)
#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_UPPER_A3_RESETVAL               (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_TYPER_UPPER_RESETVAL                  0x0
/* STATUSR */

#define CSL_GIC500_GICR_CORE_CONTROL_STATUSR_RSVD_MASK                     (0xfffffff0U)
#define CSL_GIC500_GICR_CORE_CONTROL_STATUSR_RSVD_SHIFT                    (0x4U)
#define CSL_GIC500_GICR_CORE_CONTROL_STATUSR_RSVD_RESETVAL                 (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_STATUSR_RRD_MASK                      (0x1U)
#define CSL_GIC500_GICR_CORE_CONTROL_STATUSR_RRD_SHIFT                     (0x0U)
#define CSL_GIC500_GICR_CORE_CONTROL_STATUSR_RRD_RESETVAL                  (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_STATUSR_WRD_MASK                      (0x2U)
#define CSL_GIC500_GICR_CORE_CONTROL_STATUSR_WRD_SHIFT                     (0x1U)
#define CSL_GIC500_GICR_CORE_CONTROL_STATUSR_WRD_RESETVAL                  (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_STATUSR_RWOD_MASK                     (0x4U)
#define CSL_GIC500_GICR_CORE_CONTROL_STATUSR_RWOD_SHIFT                    (0x2U)
#define CSL_GIC500_GICR_CORE_CONTROL_STATUSR_RWOD_RESETVAL                 (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_STATUSR_WROD_MASK                     (0x8U)
#define CSL_GIC500_GICR_CORE_CONTROL_STATUSR_WROD_SHIFT                    (0x3U)
#define CSL_GIC500_GICR_CORE_CONTROL_STATUSR_WROD_RESETVAL                 (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_STATUSR_RESETVAL                      0x0
/* WAKER */

#define CSL_GIC500_GICR_CORE_CONTROL_WAKER_SLEEP_MASK                      (0x1U)
#define CSL_GIC500_GICR_CORE_CONTROL_WAKER_SLEEP_SHIFT                     (0x0U)
#define CSL_GIC500_GICR_CORE_CONTROL_WAKER_SLEEP_RESETVAL                  (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_WAKER_PROCESSORSLEEP_MASK             (0x2U)
#define CSL_GIC500_GICR_CORE_CONTROL_WAKER_PROCESSORSLEEP_SHIFT            (0x1U)
#define CSL_GIC500_GICR_CORE_CONTROL_WAKER_PROCESSORSLEEP_RESETVAL         (0x1U)

#define CSL_GIC500_GICR_CORE_CONTROL_WAKER_CHILDRENASLEEP_MASK             (0x4U)
#define CSL_GIC500_GICR_CORE_CONTROL_WAKER_CHILDRENASLEEP_SHIFT            (0x2U)
#define CSL_GIC500_GICR_CORE_CONTROL_WAKER_CHILDRENASLEEP_RESETVAL         (0x1U)

#define CSL_GIC500_GICR_CORE_CONTROL_WAKER_QUIESCENT_MASK                  (0x80000000U)
#define CSL_GIC500_GICR_CORE_CONTROL_WAKER_QUIESCENT_SHIFT                 (0x1fU)
#define CSL_GIC500_GICR_CORE_CONTROL_WAKER_QUIESCENT_RESETVAL              (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_WAKER_RESETVAL                        0x6
/* SETLPIR_LOWER */

#define CSL_GIC500_GICR_CORE_CONTROL_SETLPIR_LOWER_PHYSICAL_ID_MASK        (0xffffffffU)
#define CSL_GIC500_GICR_CORE_CONTROL_SETLPIR_LOWER_PHYSICAL_ID_SHIFT       (0x0U)
#define CSL_GIC500_GICR_CORE_CONTROL_SETLPIR_LOWER_PHYSICAL_ID_RESETVAL    (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_SETLPIR_LOWER_RESETVAL                0x0
/* SETLPIR_UPPER */

#define CSL_GIC500_GICR_CORE_CONTROL_SETLPIR_UPPER_RESETVAL                0x0
/* CLRLPIR_LOWER */

#define CSL_GIC500_GICR_CORE_CONTROL_CLRLPIR_LOWER_PHYSICAL_ID_MASK        (0xffffffffU)
#define CSL_GIC500_GICR_CORE_CONTROL_CLRLPIR_LOWER_PHYSICAL_ID_SHIFT       (0x0U)
#define CSL_GIC500_GICR_CORE_CONTROL_CLRLPIR_LOWER_PHYSICAL_ID_RESETVAL    (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_CLRLPIR_LOWER_RESETVAL                0x0
/* CLRLPIR_UPPER */

#define CSL_GIC500_GICR_CORE_CONTROL_CLRLPIR_UPPER_RESETVAL                0x0
/* PROPBASER_LOWER */

#define CSL_GIC500_GICR_CORE_CONTROL_PROPBASER_LOWER_IDBITS_MASK           (0x1fU)
#define CSL_GIC500_GICR_CORE_CONTROL_PROPBASER_LOWER_IDBITS_SHIFT          (0x0U)
#define CSL_GIC500_GICR_CORE_CONTROL_PROPBASER_LOWER_IDBITS_RESETVAL       (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_PROPBASER_LOWER_CACHEABILITY_MASK     (0x380U)
#define CSL_GIC500_GICR_CORE_CONTROL_PROPBASER_LOWER_CACHEABILITY_SHIFT    (0x7U)
#define CSL_GIC500_GICR_CORE_CONTROL_PROPBASER_LOWER_CACHEABILITY_RESETVAL (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_PROPBASER_LOWER__PHYSICAL_ADDRESS__31_12__MASK (0xfffff000U)
#define CSL_GIC500_GICR_CORE_CONTROL_PROPBASER_LOWER__PHYSICAL_ADDRESS__31_12__SHIFT (0xcU)
#define CSL_GIC500_GICR_CORE_CONTROL_PROPBASER_LOWER__PHYSICAL_ADDRESS__31_12__RESETVAL (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_PROPBASER_LOWER_RESETVAL              0x0
/* PROPBASER_UPPER */

#define CSL_GIC500_GICR_CORE_CONTROL_PROPBASER_UPPER__PHYSICAL_ADDRESS__47_32__MASK (0xffffU)
#define CSL_GIC500_GICR_CORE_CONTROL_PROPBASER_UPPER__PHYSICAL_ADDRESS__47_32__SHIFT (0x0U)
#define CSL_GIC500_GICR_CORE_CONTROL_PROPBASER_UPPER__PHYSICAL_ADDRESS__47_32__RESETVAL (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_PROPBASER_UPPER_RESETVAL              0x0
/* PENDBASER_LOWER */

#define CSL_GIC500_GICR_CORE_CONTROL_PENDBASER_LOWER_CACHEABILITY_MASK     (0x380U)
#define CSL_GIC500_GICR_CORE_CONTROL_PENDBASER_LOWER_CACHEABILITY_SHIFT    (0x7U)
#define CSL_GIC500_GICR_CORE_CONTROL_PENDBASER_LOWER_CACHEABILITY_RESETVAL (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_PENDBASER_LOWER__PHYSICAL_ADDRESS__31_16__MASK (0xffff0000U)
#define CSL_GIC500_GICR_CORE_CONTROL_PENDBASER_LOWER__PHYSICAL_ADDRESS__31_16__SHIFT (0x10U)
#define CSL_GIC500_GICR_CORE_CONTROL_PENDBASER_LOWER__PHYSICAL_ADDRESS__31_16__RESETVAL (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_PENDBASER_LOWER_RESETVAL              0x0
/* PENDBASER_UPPER */

#define CSL_GIC500_GICR_CORE_CONTROL_PENDBASER_UPPER__PHYSICAL_ADDRESS__47_32__MASK (0xffffU)
#define CSL_GIC500_GICR_CORE_CONTROL_PENDBASER_UPPER__PHYSICAL_ADDRESS__47_32__SHIFT (0x0U)
#define CSL_GIC500_GICR_CORE_CONTROL_PENDBASER_UPPER__PHYSICAL_ADDRESS__47_32__RESETVAL (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_PENDBASER_UPPER_PENDING_TABLE_ZERO_MASK (0x40000000U)
#define CSL_GIC500_GICR_CORE_CONTROL_PENDBASER_UPPER_PENDING_TABLE_ZERO_SHIFT (0x1eU)
#define CSL_GIC500_GICR_CORE_CONTROL_PENDBASER_UPPER_PENDING_TABLE_ZERO_RESETVAL (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_PENDBASER_UPPER_RESETVAL              0x0
/* INVLPIR_LOWER */

#define CSL_GIC500_GICR_CORE_CONTROL_INVLPIR_LOWER_PHYSICAL_ID_MASK        (0xffffffffU)
#define CSL_GIC500_GICR_CORE_CONTROL_INVLPIR_LOWER_PHYSICAL_ID_SHIFT       (0x0U)
#define CSL_GIC500_GICR_CORE_CONTROL_INVLPIR_LOWER_PHYSICAL_ID_RESETVAL    (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_INVLPIR_LOWER_RESETVAL                0x0
/* INVLPIR_UPPER */

#define CSL_GIC500_GICR_CORE_CONTROL_INVLPIR_UPPER_RESETVAL                0x0
/* INVALLR_LOWER */

#define CSL_GIC500_GICR_CORE_CONTROL_INVALLR_LOWER_RESETVAL                0x0
/* INVALLR_UPPER */

#define CSL_GIC500_GICR_CORE_CONTROL_INVALLR_UPPER_RESETVAL                0x0
/* SYNCR */

#define CSL_GIC500_GICR_CORE_CONTROL_SYNCR_BUSY_MASK                       (0x1U)
#define CSL_GIC500_GICR_CORE_CONTROL_SYNCR_BUSY_SHIFT                      (0x0U)
#define CSL_GIC500_GICR_CORE_CONTROL_SYNCR_BUSY_RESETVAL                   (0x0U)

#define CSL_GIC500_GICR_CORE_CONTROL_SYNCR_RESETVAL                        0x0
/* PIDR4 */

#define CSL_GIC500_GICR_CORE_CONTROL_PIDR4_RESETVAL                        0x44
/* PIDR5 */

#define CSL_GIC500_GICR_CORE_CONTROL_PIDR5_RESETVAL                        0x0
/* PIDR6 */

#define CSL_GIC500_GICR_CORE_CONTROL_PIDR6_RESETVAL                        0x0
/* PIDR7 */

#define CSL_GIC500_GICR_CORE_CONTROL_PIDR7_RESETVAL                        0x0
/* PIDR0 */

#define CSL_GIC500_GICR_CORE_CONTROL_PIDR0_RESETVAL                        0x93
/* PIDR1 */

#define CSL_GIC500_GICR_CORE_CONTROL_PIDR1_RESETVAL                        0xb4
/* PIDR2 */

#define CSL_GIC500_GICR_CORE_CONTROL_PIDR2_RESETVAL                        0x3b
/* PIDR3 */

#define CSL_GIC500_GICR_CORE_CONTROL_PIDR3_RESETVAL                        0x0
/* CIDR0 */

#define CSL_GIC500_GICR_CORE_CONTROL_CIDR0_RESETVAL                        0xd
/* CIDR1 */

#define CSL_GIC500_GICR_CORE_CONTROL_CIDR1_RESETVAL                        0xf0
/* CIDR2 */

#define CSL_GIC500_GICR_CORE_CONTROL_CIDR2_RESETVAL                        0x5
/* CIDR3 */

#define CSL_GIC500_GICR_CORE_CONTROL_CIDR3_RESETVAL                        0xb1
/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
   volatile uint8_t                          Rsvd0[128];
   volatile uint32_t                         IGROUPR_SGI_PPI;
   volatile uint8_t                          Rsvd1[124];
   volatile uint32_t                         ISENABLER0;
   volatile uint8_t                          Rsvd2[124];
   volatile uint32_t                         ICENABLER0;
   volatile uint8_t                          Rsvd3[124];
   volatile uint32_t                         ISPENDR0;
   volatile uint8_t                          Rsvd4[124];
   volatile uint32_t                         ICPENDR0;
   volatile uint8_t                          Rsvd5[124];
   volatile uint32_t                         ISACTIVER0;
   volatile uint8_t                          Rsvd6[124];
   volatile uint32_t                         ICACTIVER0;
   volatile uint8_t                          Rsvd7[124];
   volatile uint32_t                         IPRIORITYR[8];
   volatile uint8_t                          Rsvd8[2016];
   volatile uint32_t                         ICFGR0;
   volatile uint32_t                         ICFGR1;
   volatile uint8_t                          Rsvd9[248];
   volatile uint32_t                         IGRPMODR_SGI_PPI;
   volatile uint8_t                          Rsvd10[252];
   volatile uint32_t                         NSACR;
   volatile uint8_t                          Rsvd11[45564];
   volatile uint32_t                         MISCSTATUSR;
   volatile uint8_t                          Rsvd12[124];
   volatile uint32_t                         PPISR;
   volatile uint8_t                          Rsvd13[16252];
} CSL_gic500_gicrRegs_core_sgi_ppi;

/************************************************************************
* Register Macros
**************************************************************************/

#define CSL_GIC500_GICR_CORE_SGI_PPI_IGROUPR_SGI_PPI(CORE)            ( CSL_GIC500_GICR_CORE_SGI_PPI_OFFSET +  ((CORE)*0x20000U) + 0x80U )
#define CSL_GIC500_GICR_CORE_SGI_PPI_ISENABLER0(CORE)                 ( CSL_GIC500_GICR_CORE_SGI_PPI_OFFSET +  ((CORE)*0x20000U) + 0x100U )
#define CSL_GIC500_GICR_CORE_SGI_PPI_ICENABLER0(CORE)                 ( CSL_GIC500_GICR_CORE_SGI_PPI_OFFSET +  ((CORE)*0x20000U) + 0x180U )
#define CSL_GIC500_GICR_CORE_SGI_PPI_ISPENDR0(CORE)                   ( CSL_GIC500_GICR_CORE_SGI_PPI_OFFSET +  ((CORE)*0x20000U) + 0x200U )
#define CSL_GIC500_GICR_CORE_SGI_PPI_ICPENDR0(CORE)                   ( CSL_GIC500_GICR_CORE_SGI_PPI_OFFSET +  ((CORE)*0x20000U) + 0x280U )
#define CSL_GIC500_GICR_CORE_SGI_PPI_ISACTIVER0(CORE)                 ( CSL_GIC500_GICR_CORE_SGI_PPI_OFFSET +  ((CORE)*0x20000U) + 0x300U )
#define CSL_GIC500_GICR_CORE_SGI_PPI_ICACTIVER0(CORE)                 ( CSL_GIC500_GICR_CORE_SGI_PPI_OFFSET +  ((CORE)*0x20000U) + 0x380U )
#define CSL_GIC500_GICR_CORE_SGI_PPI_IPRIORITYR(CORE, IDX)            ( CSL_GIC500_GICR_CORE_SGI_PPI_OFFSET +  ((CORE)*0x20000U)+ (0x400U + ((IDX)*0x4U)) )
#define CSL_GIC500_GICR_CORE_SGI_PPI_ICFGR0(CORE)                     ( CSL_GIC500_GICR_CORE_SGI_PPI_OFFSET +  ((CORE)*0x20000U) + 0xc00U )
#define CSL_GIC500_GICR_CORE_SGI_PPI_ICFGR1(CORE)                     ( CSL_GIC500_GICR_CORE_SGI_PPI_OFFSET +  ((CORE)*0x20000U) + 0xc04U )
#define CSL_GIC500_GICR_CORE_SGI_PPI_IGRPMODR_SGI_PPI(CORE)           ( CSL_GIC500_GICR_CORE_SGI_PPI_OFFSET +  ((CORE)*0x20000U) + 0xd00U )
#define CSL_GIC500_GICR_CORE_SGI_PPI_NSACR(CORE)                      ( CSL_GIC500_GICR_CORE_SGI_PPI_OFFSET +  ((CORE)*0x20000U) + 0xe00U )
#define CSL_GIC500_GICR_CORE_SGI_PPI_MISCSTATUSR(CORE)                ( CSL_GIC500_GICR_CORE_SGI_PPI_OFFSET +  ((CORE)*0x20000U) + 0xc000U )
#define CSL_GIC500_GICR_CORE_SGI_PPI_PPISR(CORE)                      ( CSL_GIC500_GICR_CORE_SGI_PPI_OFFSET +  ((CORE)*0x20000U) + 0xc080U )

/************************************************************************
* Field Definition Macros
**************************************************************************/

/* IGROUPR_SGI_PPI */

#define CSL_GIC500_GICR_CORE_SGI_PPI_IGROUPR_SGI_PPI_GROUP_MASK            (0xffffffffU)
#define CSL_GIC500_GICR_CORE_SGI_PPI_IGROUPR_SGI_PPI_GROUP_SHIFT           (0x0U)
#define CSL_GIC500_GICR_CORE_SGI_PPI_IGROUPR_SGI_PPI_GROUP_RESETVAL        (0x0U)

#define CSL_GIC500_GICR_CORE_SGI_PPI_IGROUPR_SGI_PPI_RESETVAL              0x0
/* ISENABLER0 */

#define CSL_GIC500_GICR_CORE_SGI_PPI_ISENABLER0_ENABLE_MASK                (0xffffffffU)
#define CSL_GIC500_GICR_CORE_SGI_PPI_ISENABLER0_ENABLE_SHIFT               (0x0U)
#define CSL_GIC500_GICR_CORE_SGI_PPI_ISENABLER0_ENABLE_RESETVAL            (0x0U)

#define CSL_GIC500_GICR_CORE_SGI_PPI_ISENABLER0_RESETVAL                   0x0
/* ICENABLER0 */

#define CSL_GIC500_GICR_CORE_SGI_PPI_ICENABLER0_ENABLE_MASK                (0xffffffffU)
#define CSL_GIC500_GICR_CORE_SGI_PPI_ICENABLER0_ENABLE_SHIFT               (0x0U)
#define CSL_GIC500_GICR_CORE_SGI_PPI_ICENABLER0_ENABLE_RESETVAL            (0x0U)

#define CSL_GIC500_GICR_CORE_SGI_PPI_ICENABLER0_RESETVAL                   0x0
/* ISPENDR0 */

#define CSL_GIC500_GICR_CORE_SGI_PPI_ISPENDR0_PEND_MASK                    (0xffffffffU)
#define CSL_GIC500_GICR_CORE_SGI_PPI_ISPENDR0_PEND_SHIFT                   (0x0U)
#define CSL_GIC500_GICR_CORE_SGI_PPI_ISPENDR0_PEND_RESETVAL                (0x0U)

#define CSL_GIC500_GICR_CORE_SGI_PPI_ISPENDR0_RESETVAL                     0x0
/* ICPENDR0 */

#define CSL_GIC500_GICR_CORE_SGI_PPI_ICPENDR0_PEND_MASK                    (0xffffffffU)
#define CSL_GIC500_GICR_CORE_SGI_PPI_ICPENDR0_PEND_SHIFT                   (0x0U)
#define CSL_GIC500_GICR_CORE_SGI_PPI_ICPENDR0_PEND_RESETVAL                (0x0U)

#define CSL_GIC500_GICR_CORE_SGI_PPI_ICPENDR0_RESETVAL                     0x0
/* ISACTIVER0 */

#define CSL_GIC500_GICR_CORE_SGI_PPI_ISACTIVER0_SET_MASK                   (0xffffffffU)
#define CSL_GIC500_GICR_CORE_SGI_PPI_ISACTIVER0_SET_SHIFT                  (0x0U)
#define CSL_GIC500_GICR_CORE_SGI_PPI_ISACTIVER0_SET_RESETVAL               (0x0U)

#define CSL_GIC500_GICR_CORE_SGI_PPI_ISACTIVER0_RESETVAL                   0x0
/* ICACTIVER0 */

#define CSL_GIC500_GICR_CORE_SGI_PPI_ICACTIVER0_SET_MASK                   (0xffffffffU)
#define CSL_GIC500_GICR_CORE_SGI_PPI_ICACTIVER0_SET_SHIFT                  (0x0U)
#define CSL_GIC500_GICR_CORE_SGI_PPI_ICACTIVER0_SET_RESETVAL               (0x0U)

#define CSL_GIC500_GICR_CORE_SGI_PPI_ICACTIVER0_RESETVAL                   0x0
/* IPRIORITYR */

#define CSL_GIC500_GICR_CORE_SGI_PPI_IPRIORITYR_PRIO_MASK                  (0xffffffffU)
#define CSL_GIC500_GICR_CORE_SGI_PPI_IPRIORITYR_PRIO_SHIFT                 (0x0U)
#define CSL_GIC500_GICR_CORE_SGI_PPI_IPRIORITYR_PRIO_RESETVAL              (0x0U)

#define CSL_GIC500_GICR_CORE_SGI_PPI_IPRIORITYR_RESETVAL                   0x0
/* ICFGR0 */

#define CSL_GIC500_GICR_CORE_SGI_PPI_ICFGR0_INT_TYPE_MASK                  (0xffffffffU)
#define CSL_GIC500_GICR_CORE_SGI_PPI_ICFGR0_INT_TYPE_SHIFT                 (0x0U)
#define CSL_GIC500_GICR_CORE_SGI_PPI_ICFGR0_INT_TYPE_RESETVAL              (0xc0U)

#define CSL_GIC500_GICR_CORE_SGI_PPI_ICFGR0_RESETVAL                       0xc0
/* ICFGR1 */

#define CSL_GIC500_GICR_CORE_SGI_PPI_ICFGR1_INT_TYPE_MASK                  (0xffffffffU)
#define CSL_GIC500_GICR_CORE_SGI_PPI_ICFGR1_INT_TYPE_SHIFT                 (0x0U)
#define CSL_GIC500_GICR_CORE_SGI_PPI_ICFGR1_INT_TYPE_RESETVAL              (0x0U)

#define CSL_GIC500_GICR_CORE_SGI_PPI_ICFGR1_RESETVAL                       0x0
/* IGRPMODR_SGI_PPI */

#define CSL_GIC500_GICR_CORE_SGI_PPI_IGRPMODR_SGI_PPI_GROUP_MASK           (0xffffffffU)
#define CSL_GIC500_GICR_CORE_SGI_PPI_IGRPMODR_SGI_PPI_GROUP_SHIFT          (0x0U)
#define CSL_GIC500_GICR_CORE_SGI_PPI_IGRPMODR_SGI_PPI_GROUP_RESETVAL       (0x0U)

#define CSL_GIC500_GICR_CORE_SGI_PPI_IGRPMODR_SGI_PPI_RESETVAL             0x0
/* NSACR */

#define CSL_GIC500_GICR_CORE_SGI_PPI_NSACR_RESETVAL                        0x0
/* MISCSTATUSR */

#define CSL_GIC500_GICR_CORE_SGI_PPI_MISCSTATUSR_ENABLEGRP0_MASK           (0x1U)
#define CSL_GIC500_GICR_CORE_SGI_PPI_MISCSTATUSR_ENABLEGRP0_SHIFT          (0x0U)
#define CSL_GIC500_GICR_CORE_SGI_PPI_MISCSTATUSR_ENABLEGRP0_RESETVAL       (0x0U)

#define CSL_GIC500_GICR_CORE_SGI_PPI_MISCSTATUSR_ENABLEGRP1_NS_MASK        (0x2U)
#define CSL_GIC500_GICR_CORE_SGI_PPI_MISCSTATUSR_ENABLEGRP1_NS_SHIFT       (0x1U)
#define CSL_GIC500_GICR_CORE_SGI_PPI_MISCSTATUSR_ENABLEGRP1_NS_RESETVAL    (0x0U)

#define CSL_GIC500_GICR_CORE_SGI_PPI_MISCSTATUSR_ENABLEGRP1_S_MASK         (0x4U)
#define CSL_GIC500_GICR_CORE_SGI_PPI_MISCSTATUSR_ENABLEGRP1_S_SHIFT        (0x2U)
#define CSL_GIC500_GICR_CORE_SGI_PPI_MISCSTATUSR_ENABLEGRP1_S_RESETVAL     (0x0U)

#define CSL_GIC500_GICR_CORE_SGI_PPI_MISCSTATUSR_CPU_ACTIVE_MASK           (0x80000000U)
#define CSL_GIC500_GICR_CORE_SGI_PPI_MISCSTATUSR_CPU_ACTIVE_SHIFT          (0x1fU)
#define CSL_GIC500_GICR_CORE_SGI_PPI_MISCSTATUSR_CPU_ACTIVE_RESETVAL       (0x0U)

#define CSL_GIC500_GICR_CORE_SGI_PPI_MISCSTATUSR_RESETVAL                  0x0
/* PPISR */

#define CSL_GIC500_GICR_CORE_SGI_PPI_PPISR_PPI_STATUS_MASK                 (0xffff0000U)
#define CSL_GIC500_GICR_CORE_SGI_PPI_PPISR_PPI_STATUS_SHIFT                (0x10U)
#define CSL_GIC500_GICR_CORE_SGI_PPI_PPISR_PPI_STATUS_RESETVAL             (0x0U)

#define CSL_GIC500_GICR_CORE_SGI_PPI_PPISR_RESETVAL                        0x0
/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
   CSL_gic500_gicrRegs_core_control          CONTROL;
   CSL_gic500_gicrRegs_core_sgi_ppi          SGI_PPI;
} CSL_gic500_gicrRegs_core;


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
   CSL_gic500_gicrRegs_core                  CORE[4];
} CSL_gic500_gicrRegs;


/**************************************************************************
* Hardware Region  : Region0
**************************************************************************/

typedef struct {
   CSL_gic500_gicdRegs                       GICD;
   CSL_gic500_msgRegs                        MSG;
   CSL_gic500_itsRegs                        ITS;
   volatile uint8_t                          Rsvd0[327680];
   CSL_gic500_gicrRegs                       GICR;
} CSL_gic500_region0Regs;

/**************************************************************************
* Hardware Region  : TRANSLATER
**************************************************************************/

/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
   volatile uint8_t                          Rsvd0[64];
   volatile uint32_t                         TRANSLATER[1048576];
} CSL_gic500_translaterRegs;

/************************************************************************
* Register Macros
**************************************************************************/

#define CSL_GIC500_TRANSLATER_TRANSLATER(IDX)                         ( 0x0U +  (0x40U + ((IDX)*0x4U)) )

/************************************************************************
* Field Definition Macros
**************************************************************************/

/* TRANSLATER */

#define CSL_GIC500_TRANSLATER_TRANSLATER_INPUT_ID_MASK                     (0xffffffffU)
#define CSL_GIC500_TRANSLATER_TRANSLATER_INPUT_ID_SHIFT                    (0x0U)
#define CSL_GIC500_TRANSLATER_TRANSLATER_INPUT_ID_RESETVAL                 (0x0U)

#define CSL_GIC500_TRANSLATER_TRANSLATER_RESETVAL                          0x0

#ifdef __cplusplus
}
#endif
#endif /* CSLR_GIC500_H */
