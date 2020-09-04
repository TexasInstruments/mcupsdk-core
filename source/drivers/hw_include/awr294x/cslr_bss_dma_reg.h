/********************************************************************
 * Copyright (C) 2021 Texas Instruments Incorporated.
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
 *  Name        : cslr_bss_dma_reg.h
*/
#ifndef CSLR_BSS_DMA_REG_H_
#define CSLR_BSS_DMA_REG_H_

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t UDMA_STAT;
    volatile uint32_t UDMA_CFG;
    volatile uint32_t UDMA_CTLBASE;
    volatile uint32_t UDMA_ALTBASE;
    volatile uint32_t UDMA_WAITSTAT;
    volatile uint32_t UDMA_SWREQ;
    volatile uint32_t UDMA_USEBURSTSET;
    volatile uint32_t UDMA_USEBURSTCLR;
    volatile uint32_t UDMA_REQMASKSET;
    volatile uint32_t UDMA_REQMASKCLR;
    volatile uint32_t UDMA_ENASET;
    volatile uint32_t UDMA_ENACLR;
    volatile uint32_t UDMA_ALTSET;
    volatile uint32_t UDMA_ALTCLR;
    volatile uint32_t UDMA_PRIOSET;
    volatile uint32_t UDMA_PRIOCLR;
    volatile uint8_t  Resv_76[12];
    volatile uint32_t UDMA_ERRCLR;
    volatile uint8_t  Resv_256[176];
    volatile uint32_t UDMA_CHASGN;
    volatile uint8_t  Resv_272[12];
    volatile uint32_t UDMA_CHMAP0;
    volatile uint32_t UDMA_CHMAP1;
    volatile uint32_t UDMA_CHMAP2;
    volatile uint32_t UDMA_CHMAP3;
} CSL_bss_dma_regRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_BSS_DMA_REG_UDMA_STAT                                              (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CFG                                               (0x00000004U)
#define CSL_BSS_DMA_REG_UDMA_CTLBASE                                           (0x00000008U)
#define CSL_BSS_DMA_REG_UDMA_ALTBASE                                           (0x0000000CU)
#define CSL_BSS_DMA_REG_UDMA_WAITSTAT                                          (0x00000010U)
#define CSL_BSS_DMA_REG_UDMA_SWREQ                                             (0x00000014U)
#define CSL_BSS_DMA_REG_UDMA_USEBURSTSET                                       (0x00000018U)
#define CSL_BSS_DMA_REG_UDMA_USEBURSTCLR                                       (0x0000001CU)
#define CSL_BSS_DMA_REG_UDMA_REQMASKSET                                        (0x00000020U)
#define CSL_BSS_DMA_REG_UDMA_REQMASKCLR                                        (0x00000024U)
#define CSL_BSS_DMA_REG_UDMA_ENASET                                            (0x00000028U)
#define CSL_BSS_DMA_REG_UDMA_ENACLR                                            (0x0000002CU)
#define CSL_BSS_DMA_REG_UDMA_ALTSET                                            (0x00000030U)
#define CSL_BSS_DMA_REG_UDMA_ALTCLR                                            (0x00000034U)
#define CSL_BSS_DMA_REG_UDMA_PRIOSET                                           (0x00000038U)
#define CSL_BSS_DMA_REG_UDMA_PRIOCLR                                           (0x0000003CU)
#define CSL_BSS_DMA_REG_UDMA_ERRCLR                                            (0x0000004CU)
#define CSL_BSS_DMA_REG_UDMA_CHASGN                                            (0x00000100U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP0                                            (0x00000110U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP1                                            (0x00000114U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP2                                            (0x00000118U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP3                                            (0x0000011CU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* UDMA_STAT */

#define CSL_BSS_DMA_REG_UDMA_STAT_MASTEN_MASK                                  (0x00000001U)
#define CSL_BSS_DMA_REG_UDMA_STAT_MASTEN_SHIFT                                 (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_STAT_MASTEN_RESETVAL                              (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_STAT_MASTEN_MAX                                   (0x00000001U)

#define CSL_BSS_DMA_REG_UDMA_STAT_NU1_MASK                                     (0x0000000EU)
#define CSL_BSS_DMA_REG_UDMA_STAT_NU1_SHIFT                                    (0x00000001U)
#define CSL_BSS_DMA_REG_UDMA_STAT_NU1_RESETVAL                                 (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_STAT_NU1_MAX                                      (0x00000007U)

#define CSL_BSS_DMA_REG_UDMA_STAT_STATE_MASK                                   (0x000000F0U)
#define CSL_BSS_DMA_REG_UDMA_STAT_STATE_SHIFT                                  (0x00000004U)
#define CSL_BSS_DMA_REG_UDMA_STAT_STATE_RESETVAL                               (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_STAT_STATE_MAX                                    (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_STAT_NU2_MASK                                     (0x0000FF00U)
#define CSL_BSS_DMA_REG_UDMA_STAT_NU2_SHIFT                                    (0x00000008U)
#define CSL_BSS_DMA_REG_UDMA_STAT_NU2_RESETVAL                                 (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_STAT_NU2_MAX                                      (0x000000FFU)

#define CSL_BSS_DMA_REG_UDMA_STAT_DMACHANS_MASK                                (0x001F0000U)
#define CSL_BSS_DMA_REG_UDMA_STAT_DMACHANS_SHIFT                               (0x00000010U)
#define CSL_BSS_DMA_REG_UDMA_STAT_DMACHANS_RESETVAL                            (0x0000001FU)
#define CSL_BSS_DMA_REG_UDMA_STAT_DMACHANS_MAX                                 (0x0000001FU)

#define CSL_BSS_DMA_REG_UDMA_STAT_NU3_MASK                                     (0xFFE00000U)
#define CSL_BSS_DMA_REG_UDMA_STAT_NU3_SHIFT                                    (0x00000015U)
#define CSL_BSS_DMA_REG_UDMA_STAT_NU3_RESETVAL                                 (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_STAT_NU3_MAX                                      (0x000007FFU)

#define CSL_BSS_DMA_REG_UDMA_STAT_RESETVAL                                     (0x001F0000U)

/* UDMA_CFG */

#define CSL_BSS_DMA_REG_UDMA_CFG_MASTEN_MASK                                   (0x00000001U)
#define CSL_BSS_DMA_REG_UDMA_CFG_MASTEN_SHIFT                                  (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CFG_MASTEN_RESETVAL                               (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CFG_MASTEN_MAX                                    (0x00000001U)

#define CSL_BSS_DMA_REG_UDMA_CFG_NU1_MASK                                      (0xFFFFFFFEU)
#define CSL_BSS_DMA_REG_UDMA_CFG_NU1_SHIFT                                     (0x00000001U)
#define CSL_BSS_DMA_REG_UDMA_CFG_NU1_RESETVAL                                  (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CFG_NU1_MAX                                       (0x7FFFFFFFU)

#define CSL_BSS_DMA_REG_UDMA_CFG_RESETVAL                                      (0x00000000U)

/* UDMA_CTLBASE */

#define CSL_BSS_DMA_REG_UDMA_CTLBASE_NU1_MASK                                  (0x000003FFU)
#define CSL_BSS_DMA_REG_UDMA_CTLBASE_NU1_SHIFT                                 (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CTLBASE_NU1_RESETVAL                              (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CTLBASE_NU1_MAX                                   (0x000003FFU)

#define CSL_BSS_DMA_REG_UDMA_CTLBASE_ADDR_MASK                                 (0xFFFFFC00U)
#define CSL_BSS_DMA_REG_UDMA_CTLBASE_ADDR_SHIFT                                (0x0000000AU)
#define CSL_BSS_DMA_REG_UDMA_CTLBASE_ADDR_RESETVAL                             (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CTLBASE_ADDR_MAX                                  (0x003FFFFFU)

#define CSL_BSS_DMA_REG_UDMA_CTLBASE_RESETVAL                                  (0x00000000U)

/* UDMA_ALTBASE */

#define CSL_BSS_DMA_REG_UDMA_ALTBASE_ADDR_MASK                                 (0xFFFFFFFFU)
#define CSL_BSS_DMA_REG_UDMA_ALTBASE_ADDR_SHIFT                                (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_ALTBASE_ADDR_RESETVAL                             (0x00000200U)
#define CSL_BSS_DMA_REG_UDMA_ALTBASE_ADDR_MAX                                  (0xFFFFFFFFU)

#define CSL_BSS_DMA_REG_UDMA_ALTBASE_RESETVAL                                  (0x00000200U)

/* UDMA_WAITSTAT */

#define CSL_BSS_DMA_REG_UDMA_WAITSTAT_WAITREQ_N_MASK                           (0xFFFFFFFFU)
#define CSL_BSS_DMA_REG_UDMA_WAITSTAT_WAITREQ_N_SHIFT                          (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_WAITSTAT_WAITREQ_N_RESETVAL                       (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_WAITSTAT_WAITREQ_N_MAX                            (0xFFFFFFFFU)

#define CSL_BSS_DMA_REG_UDMA_WAITSTAT_RESETVAL                                 (0x00000000U)

/* UDMA_SWREQ */

#define CSL_BSS_DMA_REG_UDMA_SWREQ_SWREQ_N_MASK                                (0xFFFFFFFFU)
#define CSL_BSS_DMA_REG_UDMA_SWREQ_SWREQ_N_SHIFT                               (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_SWREQ_SWREQ_N_RESETVAL                            (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_SWREQ_SWREQ_N_MAX                                 (0xFFFFFFFFU)

#define CSL_BSS_DMA_REG_UDMA_SWREQ_RESETVAL                                    (0x00000000U)

/* UDMA_USEBURSTSET */

#define CSL_BSS_DMA_REG_UDMA_USEBURSTSET_SET_N_MASK                            (0xFFFFFFFFU)
#define CSL_BSS_DMA_REG_UDMA_USEBURSTSET_SET_N_SHIFT                           (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_USEBURSTSET_SET_N_RESETVAL                        (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_USEBURSTSET_SET_N_MAX                             (0xFFFFFFFFU)

#define CSL_BSS_DMA_REG_UDMA_USEBURSTSET_RESETVAL                              (0x00000000U)

/* UDMA_USEBURSTCLR */

#define CSL_BSS_DMA_REG_UDMA_USEBURSTCLR_CLR_N_MASK                            (0xFFFFFFFFU)
#define CSL_BSS_DMA_REG_UDMA_USEBURSTCLR_CLR_N_SHIFT                           (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_USEBURSTCLR_CLR_N_RESETVAL                        (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_USEBURSTCLR_CLR_N_MAX                             (0xFFFFFFFFU)

#define CSL_BSS_DMA_REG_UDMA_USEBURSTCLR_RESETVAL                              (0x00000000U)

/* UDMA_REQMASKSET */

#define CSL_BSS_DMA_REG_UDMA_REQMASKSET_SET_N_MASK                             (0xFFFFFFFFU)
#define CSL_BSS_DMA_REG_UDMA_REQMASKSET_SET_N_SHIFT                            (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_REQMASKSET_SET_N_RESETVAL                         (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_REQMASKSET_SET_N_MAX                              (0xFFFFFFFFU)

#define CSL_BSS_DMA_REG_UDMA_REQMASKSET_RESETVAL                               (0x00000000U)

/* UDMA_REQMASKCLR */

#define CSL_BSS_DMA_REG_UDMA_REQMASKCLR_CLR_N_MASK                             (0xFFFFFFFFU)
#define CSL_BSS_DMA_REG_UDMA_REQMASKCLR_CLR_N_SHIFT                            (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_REQMASKCLR_CLR_N_RESETVAL                         (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_REQMASKCLR_CLR_N_MAX                              (0xFFFFFFFFU)

#define CSL_BSS_DMA_REG_UDMA_REQMASKCLR_RESETVAL                               (0x00000000U)

/* UDMA_ENASET */

#define CSL_BSS_DMA_REG_UDMA_ENASET_SET_N_MASK                                 (0xFFFFFFFFU)
#define CSL_BSS_DMA_REG_UDMA_ENASET_SET_N_SHIFT                                (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_ENASET_SET_N_RESETVAL                             (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_ENASET_SET_N_MAX                                  (0xFFFFFFFFU)

#define CSL_BSS_DMA_REG_UDMA_ENASET_RESETVAL                                   (0x00000000U)

/* UDMA_ENACLR */

#define CSL_BSS_DMA_REG_UDMA_ENACLR_CLR_N_MASK                                 (0xFFFFFFFFU)
#define CSL_BSS_DMA_REG_UDMA_ENACLR_CLR_N_SHIFT                                (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_ENACLR_CLR_N_RESETVAL                             (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_ENACLR_CLR_N_MAX                                  (0xFFFFFFFFU)

#define CSL_BSS_DMA_REG_UDMA_ENACLR_RESETVAL                                   (0x00000000U)

/* UDMA_ALTSET */

#define CSL_BSS_DMA_REG_UDMA_ALTSET_SET_N_MASK                                 (0xFFFFFFFFU)
#define CSL_BSS_DMA_REG_UDMA_ALTSET_SET_N_SHIFT                                (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_ALTSET_SET_N_RESETVAL                             (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_ALTSET_SET_N_MAX                                  (0xFFFFFFFFU)

#define CSL_BSS_DMA_REG_UDMA_ALTSET_RESETVAL                                   (0x00000000U)

/* UDMA_ALTCLR */

#define CSL_BSS_DMA_REG_UDMA_ALTCLR_CLR_N_MASK                                 (0xFFFFFFFFU)
#define CSL_BSS_DMA_REG_UDMA_ALTCLR_CLR_N_SHIFT                                (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_ALTCLR_CLR_N_RESETVAL                             (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_ALTCLR_CLR_N_MAX                                  (0xFFFFFFFFU)

#define CSL_BSS_DMA_REG_UDMA_ALTCLR_RESETVAL                                   (0x00000000U)

/* UDMA_PRIOSET */

#define CSL_BSS_DMA_REG_UDMA_PRIOSET_SET_N_MASK                                (0xFFFFFFFFU)
#define CSL_BSS_DMA_REG_UDMA_PRIOSET_SET_N_SHIFT                               (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_PRIOSET_SET_N_RESETVAL                            (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_PRIOSET_SET_N_MAX                                 (0xFFFFFFFFU)

#define CSL_BSS_DMA_REG_UDMA_PRIOSET_RESETVAL                                  (0x00000000U)

/* UDMA_PRIOCLR */

#define CSL_BSS_DMA_REG_UDMA_PRIOCLR_CLR_N_MASK                                (0xFFFFFFFFU)
#define CSL_BSS_DMA_REG_UDMA_PRIOCLR_CLR_N_SHIFT                               (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_PRIOCLR_CLR_N_RESETVAL                            (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_PRIOCLR_CLR_N_MAX                                 (0xFFFFFFFFU)

#define CSL_BSS_DMA_REG_UDMA_PRIOCLR_RESETVAL                                  (0x00000000U)

/* UDMA_ERRCLR */

#define CSL_BSS_DMA_REG_UDMA_ERRCLR_ERRCLR_MASK                                (0x00000001U)
#define CSL_BSS_DMA_REG_UDMA_ERRCLR_ERRCLR_SHIFT                               (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_ERRCLR_ERRCLR_RESETVAL                            (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_ERRCLR_ERRCLR_MAX                                 (0x00000001U)

#define CSL_BSS_DMA_REG_UDMA_ERRCLR_NU1_MASK                                   (0xFFFFFFFEU)
#define CSL_BSS_DMA_REG_UDMA_ERRCLR_NU1_SHIFT                                  (0x00000001U)
#define CSL_BSS_DMA_REG_UDMA_ERRCLR_NU1_RESETVAL                               (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_ERRCLR_NU1_MAX                                    (0x7FFFFFFFU)

#define CSL_BSS_DMA_REG_UDMA_ERRCLR_RESETVAL                                   (0x00000000U)

/* UDMA_CHASGN */

#define CSL_BSS_DMA_REG_UDMA_CHASGN_CHASGN_MASK                                (0xFFFFFFFFU)
#define CSL_BSS_DMA_REG_UDMA_CHASGN_CHASGN_SHIFT                               (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHASGN_CHASGN_RESETVAL                            (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHASGN_CHASGN_MAX                                 (0xFFFFFFFFU)

#define CSL_BSS_DMA_REG_UDMA_CHASGN_RESETVAL                                   (0x00000000U)

/* UDMA_CHMAP0 */

#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH0SEL_N_MASK                              (0x0000000FU)
#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH0SEL_N_SHIFT                             (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH0SEL_N_RESETVAL                          (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH0SEL_N_MAX                               (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH1SEL_N_MASK                              (0x000000F0U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH1SEL_N_SHIFT                             (0x00000004U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH1SEL_N_RESETVAL                          (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH1SEL_N_MAX                               (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH2SEL_N_MASK                              (0x00000F00U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH2SEL_N_SHIFT                             (0x00000008U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH2SEL_N_RESETVAL                          (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH2SEL_N_MAX                               (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH3SEL_N_MASK                              (0x0000F000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH3SEL_N_SHIFT                             (0x0000000CU)
#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH3SEL_N_RESETVAL                          (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH3SEL_N_MAX                               (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH4SEL_N_MASK                              (0x000F0000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH4SEL_N_SHIFT                             (0x00000010U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH4SEL_N_RESETVAL                          (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH4SEL_N_MAX                               (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH5SEL_N_MASK                              (0x00F00000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH5SEL_N_SHIFT                             (0x00000014U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH5SEL_N_RESETVAL                          (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH5SEL_N_MAX                               (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH6SEL_N_MASK                              (0x0F000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH6SEL_N_SHIFT                             (0x00000018U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH6SEL_N_RESETVAL                          (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH6SEL_N_MAX                               (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH7SEL_N_MASK                              (0xF0000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH7SEL_N_SHIFT                             (0x0000001CU)
#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH7SEL_N_RESETVAL                          (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP0_CH7SEL_N_MAX                               (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP0_RESETVAL                                   (0x00000000U)

/* UDMA_CHMAP1 */

#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH8SEL_N_MASK                              (0x0000000FU)
#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH8SEL_N_SHIFT                             (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH8SEL_N_RESETVAL                          (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH8SEL_N_MAX                               (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH9SEL_N_MASK                              (0x000000F0U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH9SEL_N_SHIFT                             (0x00000004U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH9SEL_N_RESETVAL                          (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH9SEL_N_MAX                               (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH10SEL_N_MASK                             (0x00000F00U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH10SEL_N_SHIFT                            (0x00000008U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH10SEL_N_RESETVAL                         (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH10SEL_N_MAX                              (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH11SEL_N_MASK                             (0x0000F000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH11SEL_N_SHIFT                            (0x0000000CU)
#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH11SEL_N_RESETVAL                         (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH11SEL_N_MAX                              (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH12SEL_N_MASK                             (0x000F0000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH12SEL_N_SHIFT                            (0x00000010U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH12SEL_N_RESETVAL                         (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH12SEL_N_MAX                              (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH13SEL_N_MASK                             (0x00F00000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH13SEL_N_SHIFT                            (0x00000014U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH13SEL_N_RESETVAL                         (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH13SEL_N_MAX                              (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH14SEL_N_MASK                             (0x0F000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH14SEL_N_SHIFT                            (0x00000018U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH14SEL_N_RESETVAL                         (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH14SEL_N_MAX                              (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH15SEL_N_MASK                             (0xF0000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH15SEL_N_SHIFT                            (0x0000001CU)
#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH15SEL_N_RESETVAL                         (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP1_CH15SEL_N_MAX                              (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP1_RESETVAL                                   (0x00000000U)

/* UDMA_CHMAP2 */

#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH16SEL_N_MASK                             (0x0000000FU)
#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH16SEL_N_SHIFT                            (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH16SEL_N_RESETVAL                         (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH16SEL_N_MAX                              (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH17SEL_N_MASK                             (0x000000F0U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH17SEL_N_SHIFT                            (0x00000004U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH17SEL_N_RESETVAL                         (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH17SEL_N_MAX                              (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH18SEL_N_MASK                             (0x00000F00U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH18SEL_N_SHIFT                            (0x00000008U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH18SEL_N_RESETVAL                         (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH18SEL_N_MAX                              (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH19SEL_N_MASK                             (0x0000F000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH19SEL_N_SHIFT                            (0x0000000CU)
#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH19SEL_N_RESETVAL                         (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH19SEL_N_MAX                              (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH20SEL_N_MASK                             (0x000F0000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH20SEL_N_SHIFT                            (0x00000010U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH20SEL_N_RESETVAL                         (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH20SEL_N_MAX                              (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH21SEL_N_MASK                             (0x00F00000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH21SEL_N_SHIFT                            (0x00000014U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH21SEL_N_RESETVAL                         (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH21SEL_N_MAX                              (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH22SEL_N_MASK                             (0x0F000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH22SEL_N_SHIFT                            (0x00000018U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH22SEL_N_RESETVAL                         (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH22SEL_N_MAX                              (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH23SEL_N_MASK                             (0xF0000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH23SEL_N_SHIFT                            (0x0000001CU)
#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH23SEL_N_RESETVAL                         (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP2_CH23SEL_N_MAX                              (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP2_RESETVAL                                   (0x00000000U)

/* UDMA_CHMAP3 */

#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH24SEL_N_MASK                             (0x0000000FU)
#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH24SEL_N_SHIFT                            (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH24SEL_N_RESETVAL                         (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH24SEL_N_MAX                              (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH25SEL_N_MASK                             (0x000000F0U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH25SEL_N_SHIFT                            (0x00000004U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH25SEL_N_RESETVAL                         (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH25SEL_N_MAX                              (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH26SEL_N_MASK                             (0x00000F00U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH26SEL_N_SHIFT                            (0x00000008U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH26SEL_N_RESETVAL                         (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH26SEL_N_MAX                              (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH27SEL_N_MASK                             (0x0000F000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH27SEL_N_SHIFT                            (0x0000000CU)
#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH27SEL_N_RESETVAL                         (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH27SEL_N_MAX                              (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH28SEL_N_MASK                             (0x000F0000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH28SEL_N_SHIFT                            (0x00000010U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH28SEL_N_RESETVAL                         (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH28SEL_N_MAX                              (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH29SEL_N_MASK                             (0x00F00000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH29SEL_N_SHIFT                            (0x00000014U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH29SEL_N_RESETVAL                         (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH29SEL_N_MAX                              (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH30SEL_N_MASK                             (0x0F000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH30SEL_N_SHIFT                            (0x00000018U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH30SEL_N_RESETVAL                         (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH30SEL_N_MAX                              (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH31SEL_N_MASK                             (0xF0000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH31SEL_N_SHIFT                            (0x0000001CU)
#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH31SEL_N_RESETVAL                         (0x00000000U)
#define CSL_BSS_DMA_REG_UDMA_CHMAP3_CH31SEL_N_MAX                              (0x0000000FU)

#define CSL_BSS_DMA_REG_UDMA_CHMAP3_RESETVAL                                   (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
