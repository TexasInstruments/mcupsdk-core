/********************************************************************
 * Copyright (C) 2013-2017 Texas Instruments Incorporated.
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
#ifndef CSLR_ICSSCFG_H
#define CSLR_ICSSCFG_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>


/**************************************************************************
* Register Overlay Structure for __ALL__
**************************************************************************/
typedef struct {
    volatile Uint32 REVID;
    volatile Uint32 SYSCFG;
    volatile Uint32 GPCFG0;
    volatile Uint32 GPCFG1;
    volatile Uint32 CGR;
    volatile Uint32 ISRP;
    volatile Uint32 ISP;
    volatile Uint32 IESP;
    volatile Uint32 IECP;
    volatile Uint32 SCRP;
    volatile Uint32 PMAO;
    volatile Uint32 MII_RT;
    volatile Uint32 IEPCLK;
    volatile Uint32 SPP;
    volatile Uint8  RSVD0[8];
    volatile Uint32 PIN_MX;
    volatile Uint32 SDPRU0CLKDIV;
    volatile Uint32 SDPRU0CLKSELREGISTER0;
    volatile Uint32 SDPRU0SAMPLESIZEREGISTER0;
    volatile Uint32 SDPRU0CLKSELREGISTER1;
    volatile Uint32 SDPRU0SAMPLESIZEREGISTER1;
    volatile Uint32 SDPRU0CLKSELREGISTER2;
    volatile Uint32 SDPRU0SAMPLESIZEREGISTER2;
    volatile Uint32 SDPRU0CLKSELREGISTER3;
    volatile Uint32 SDPRU0SAMPLESIZEREGISTER3;
    volatile Uint32 SDPRU0CLKSELREGISTER4;
    volatile Uint32 SDPRU0SAMPLESIZEREGISTER4;
    volatile Uint32 SDPRU0CLKSELREGISTER5;
    volatile Uint32 SDPRU0SAMPLESIZEREGISTER5;
    volatile Uint32 SDPRU0CLKSELREGISTER6;
    volatile Uint32 SDPRU0SAMPLESIZEREGISTER6;
    volatile Uint32 SDPRU0CLKSELREGISTER7;
    volatile Uint32 SDPRU0SAMPLESIZEREGISTER7;
    volatile Uint32 SDPRU0CLKSELREGISTER8;
    volatile Uint32 SDPRU0SAMPLESIZEREGISTER8;
    volatile Uint32 SDPRU1CLKDIVREGISTER;
    volatile Uint32 SDPRU1CLKSELREGISTER0;
    volatile Uint32 SDPRU1SAMPLESIZEREGISTER0;
    volatile Uint32 SDPRU1CLKSELREGISTER1;
    volatile Uint32 SDPRU1SAMPLESIZEREGISTER1;
    volatile Uint32 SDPRU1CLKSELREGISTER2;
    volatile Uint32 SDPRU1SAMPLESIZEREGISTER2;
    volatile Uint32 SDPRU1CLKSELREGISTER3;
    volatile Uint32 SDPRU1SAMPLESIZEREGISTER3;
    volatile Uint32 SDPRU1CLKSELREGISTER4;
    volatile Uint32 SDPRU1SAMPLESIZEREGISTER4;
    volatile Uint32 SDPRU1CLKSELREGISTER5;
    volatile Uint32 SDPRU1SAMPLESIZEREGISTER5;
    volatile Uint32 SDPRU1CLKSELREGISTER6;
    volatile Uint32 SDPRU1SAMPLESIZEREGISTER6;
    volatile Uint32 SDPRU1CLKSELREGISTER7;
    volatile Uint32 SDPRU1SAMPLESIZEREGISTER7;
    volatile Uint32 SDPRU1CLKSELREGISTER8;
    volatile Uint32 SDPRU1SAMPLESIZEREGISTER8;
    volatile Uint8  RSVD1[4];
    volatile Uint32 EDPRU0RXCFGREGISTER;
    volatile Uint32 EDPRU0TXCFGREGISTER;
    volatile Uint32 EDPRU0CH0CFG0REGISTER;
    volatile Uint32 EDPRU0CH0CFG1REGISTER;
    volatile Uint32 EDPRU0CH1CFG0REGISTER;
    volatile Uint32 EDPRU0CH1CFG1REGISTER;
    volatile Uint32 EDPRU0CH2CFG0REGISTER;
    volatile Uint32 EDPRU0CH2CFG1REGISTER;
    volatile Uint32 EDPRU1RXCFGREGISTER;
    volatile Uint32 EDPRU1TXCFGREGISTER;
    volatile Uint32 EDPRU1CH0CFG0REGISTER;
    volatile Uint32 EDPRU1CH0CFG1REGISTER;
    volatile Uint32 EDPRU1CH1CFG0REGISTER;
    volatile Uint32 EDPRU1CH1CFG1REGISTER;
    volatile Uint32 EDPRU1CH2CFG0REGISTER;
    volatile Uint32 EDPRU1CH2CFG1REGISTER;
    volatile Uint8  RSVD2[852];
} CSL_IcssCfgRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

/* HL08 REVID REGISTER */
#define CSL_ICSSCFG_REVID                                       (0x0U)

/* SYSCFG */
#define CSL_ICSSCFG_SYSCFG                                      (0x4U)

/* GP CFG0 */
#define CSL_ICSSCFG_GPCFG0                                      (0x8U)

/* GP CFG1 */
#define CSL_ICSSCFG_GPCFG1                                      (0xCU)

/* CLOCK GATING */
#define CSL_ICSSCFG_CGR                                         (0x10U)

/* IRQSTATUS_RAW_PARITY */
#define CSL_ICSSCFG_ISRP                                        (0x14U)

/* IRQSTATUS_PARITY */
#define CSL_ICSSCFG_ISP                                         (0x18U)

/* IRQENABLE_SET_PARITY */
#define CSL_ICSSCFG_IESP                                        (0x1CU)

/* IRQENABLE_CLR_PARITY */
#define CSL_ICSSCFG_IECP                                        (0x20U)

/* SCR PRIORITY CFG */
#define CSL_ICSSCFG_SCRP                                        (0x24U)

/* PRU MASTER OCP ADDRESSOFFSET */
#define CSL_ICSSCFG_PMAO                                        (0x28U)

/* MII_RT EVENT ENABLE */
#define CSL_ICSSCFG_MII_RT                                      (0x2CU)

/* IEP CLOCKSOURCE */
#define CSL_ICSSCFG_IEPCLK                                      (0x30U)

/* SCRATCH PAD PRIORITY AND CONFIG */
#define CSL_ICSSCFG_SPP                                         (0x34U)

/* PIN MUXSEL */
#define CSL_ICSSCFG_PIN_MX                                      (0x40U)

/* SDPRU0CLKDIV */
#define CSL_ICSSCFG_SDPRU0CLKDIV                                (0x44U)

/* SDPRU0CLKSELREGISTER0 */
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER0                       (0x48U)

/* SDPRU0SAMPLESIZEREGISTER0 */
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER0                   (0x4CU)

/* SDPRU0CLKSELREGISTER1 */
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER1                       (0x50U)

/* SDPRU0SAMPLESIZEREGISTER1 */
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER1                   (0x54U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER2                       (0x58U)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER2                   (0x5CU)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER3                       (0x60U)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER3                   (0x64U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER4                       (0x68U)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER4                   (0x6CU)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER5                       (0x70U)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER5                   (0x74U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER6                       (0x78U)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER6                   (0x7CU)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER7                       (0x80U)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER7                   (0x84U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER8                       (0x88U)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER8                   (0x8CU)
#define CSL_ICSSCFG_SDPRU1CLKDIVREGISTER                        (0x90U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER0                       (0x94U)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER0                   (0x98U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER1                       (0x9CU)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER1                   (0xA0U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER2                       (0xA4U)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER2                   (0xA8U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER3                       (0xACU)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER3                   (0xB0U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER4                       (0xB4U)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER4                   (0xB8U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER5                       (0xBCU)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER5                   (0xC0U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER6                       (0xC4U)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER6                   (0xC8U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER7                       (0xCCU)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER7                   (0xD0U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER8                       (0xD4U)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER8                   (0xD8U)
#define CSL_ICSSCFG_EDPRU0RXCFGREGISTER                         (0xE0U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER                         (0xE4U)
#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER                       (0xE8U)
#define CSL_ICSSCFG_EDPRU0CH0CFG1REGISTER                       (0xECU)
#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER                       (0xF0U)
#define CSL_ICSSCFG_EDPRU0CH1CFG1REGISTER                       (0xF4U)
#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER                       (0xF8U)
#define CSL_ICSSCFG_EDPRU0CH2CFG1REGISTER                       (0xFCU)
#define CSL_ICSSCFG_EDPRU1RXCFGREGISTER                         (0x100U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER                         (0x104U)
#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER                       (0x108U)
#define CSL_ICSSCFG_EDPRU1CH0CFG1REGISTER                       (0x10CU)
#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER                       (0x110U)
#define CSL_ICSSCFG_EDPRU1CH1CFG1REGISTER                       (0x114U)
#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER                       (0x118U)
#define CSL_ICSSCFG_EDPRU1CH2CFG1REGISTER                       (0x11CU)


/**************************************************************************
* Field Definition Macros
**************************************************************************/

/* REVID */

#define CSL_ICSSCFG_REVID_REVID_MASK                            (0xFFFFFFFFU)
#define CSL_ICSSCFG_REVID_REVID_SHIFT                           (0U)
#define CSL_ICSSCFG_REVID_REVID_RESETVAL                        (0x470000a2U)
#define CSL_ICSSCFG_REVID_REVID_MAX                             (0xffffffffU)

#define CSL_ICSSCFG_REVID_RESETVAL                              (0x470000a2U)

/* SYSCFG */

#define CSL_ICSSCFG_SYSCFG_IDLE_MODE_MASK                       (0x00000003U)
#define CSL_ICSSCFG_SYSCFG_IDLE_MODE_SHIFT                      (0U)
#define CSL_ICSSCFG_SYSCFG_IDLE_MODE_RESETVAL                   (0x00000002U)
#define CSL_ICSSCFG_SYSCFG_IDLE_MODE_MAX                        (0x00000003U)

#define CSL_ICSSCFG_SYSCFG_STANDBY_MODE_MASK                    (0x0000000CU)
#define CSL_ICSSCFG_SYSCFG_STANDBY_MODE_SHIFT                   (2U)
#define CSL_ICSSCFG_SYSCFG_STANDBY_MODE_RESETVAL                (0x00000002U)
#define CSL_ICSSCFG_SYSCFG_STANDBY_MODE_MAX                     (0x00000003U)

#define CSL_ICSSCFG_SYSCFG_STANDBY_INIT_MASK                    (0x00000010U)
#define CSL_ICSSCFG_SYSCFG_STANDBY_INIT_SHIFT                   (4U)
#define CSL_ICSSCFG_SYSCFG_STANDBY_INIT_RESETVAL                (0x00000001U)
#define CSL_ICSSCFG_SYSCFG_STANDBY_INIT_MAX                     (0x00000001U)

#define CSL_ICSSCFG_SYSCFG_SUB_MWAIT_MASK                       (0x00000020U)
#define CSL_ICSSCFG_SYSCFG_SUB_MWAIT_SHIFT                      (5U)
#define CSL_ICSSCFG_SYSCFG_SUB_MWAIT_RESETVAL                   (0x00000000U)
#define CSL_ICSSCFG_SYSCFG_SUB_MWAIT_MAX                        (0x00000001U)

#define CSL_ICSSCFG_SYSCFG_RESETVAL                             (0x0000001aU)

/* GPCFG0 */

#define CSL_ICSSCFG_GPCFG0_PRU0_GPI_MODE_MASK                   (0x00000003U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GPI_MODE_SHIFT                  (0U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GPI_MODE_RESETVAL               (0x00000000U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GPI_MODE_MAX                    (0x00000003U)

#define CSL_ICSSCFG_GPCFG0_PRU0_GPI_CLK_MODE_MASK               (0x00000004U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GPI_CLK_MODE_SHIFT              (2U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GPI_CLK_MODE_RESETVAL           (0x00000000U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GPI_CLK_MODE_MAX                (0x00000001U)

#define CSL_ICSSCFG_GPCFG0_PRU0_GPI_DIV0_MASK                   (0x000000F8U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GPI_DIV0_SHIFT                  (3U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GPI_DIV0_RESETVAL               (0x00000000U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GPI_DIV0_MAX                    (0x0000001fU)

#define CSL_ICSSCFG_GPCFG0_PRU0_GPI_DIV1_MASK                   (0x00001F00U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GPI_DIV1_SHIFT                  (8U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GPI_DIV1_RESETVAL               (0x00000000U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GPI_DIV1_MAX                    (0x0000001fU)

#define CSL_ICSSCFG_GPCFG0_PRU0_GPI_SB_MASK                     (0x00002000U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GPI_SB_SHIFT                    (13U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GPI_SB_RESETVAL                 (0x00000000U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GPI_SB_MAX                      (0x00000001U)

#define CSL_ICSSCFG_GPCFG0_PRU0_GPO_MODE_MASK                   (0x00004000U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GPO_MODE_SHIFT                  (14U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GPO_MODE_RESETVAL               (0x00000000U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GPO_MODE_MAX                    (0x00000001U)

#define CSL_ICSSCFG_GPCFG0_PRU0_GPO_DIV0_MASK                   (0x000F8000U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GPO_DIV0_SHIFT                  (15U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GPO_DIV0_RESETVAL               (0x00000000U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GPO_DIV0_MAX                    (0x0000001fU)

#define CSL_ICSSCFG_GPCFG0_PRU0_GPO_DIV1_MASK                   (0x01F00000U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GPO_DIV1_SHIFT                  (20U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GPO_DIV1_RESETVAL               (0x00000000U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GPO_DIV1_MAX                    (0x0000001fU)

#define CSL_ICSSCFG_GPCFG0_PRU0_GPO_SH_SEL_MASK                 (0x02000000U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GPO_SH_SEL_SHIFT                (25U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GPO_SH_SEL_RESETVAL             (0x00000000U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GPO_SH_SEL_MAX                  (0x00000001U)

#ifndef CSL_MODIFICATION
#define CSL_ICSSCFG_GPCFG0_PRU0_GP_MUX_SEL_MASK                 (0x3C000000U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GP_MUX_SEL_SHIFT                (26U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GP_MUX_SEL_RESETVAL             (0x00000000U)
#define CSL_ICSSCFG_GPCFG0_PRU0_GP_MUX_SEL_MAX                  (0x00000004U)
#endif

#define CSL_ICSSCFG_GPCFG0_RESETVAL                             (0x00000000U)

/* GPCFG1 */

#define CSL_ICSSCFG_GPCFG1_PRU1_GPI_MODE_MASK                   (0x00000003U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GPI_MODE_SHIFT                  (0U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GPI_MODE_RESETVAL               (0x00000000U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GPI_MODE_MAX                    (0x00000003U)

#define CSL_ICSSCFG_GPCFG1_PRU1_GPI_CLK_MODE_MASK               (0x00000004U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GPI_CLK_MODE_SHIFT              (2U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GPI_CLK_MODE_RESETVAL           (0x00000000U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GPI_CLK_MODE_MAX                (0x00000001U)

#define CSL_ICSSCFG_GPCFG1_PRU1_GPI_DIV0_MASK                   (0x000000F8U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GPI_DIV0_SHIFT                  (3U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GPI_DIV0_RESETVAL               (0x00000000U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GPI_DIV0_MAX                    (0x0000001fU)

#define CSL_ICSSCFG_GPCFG1_PRU1_GPI_DIV1_MASK                   (0x00001F00U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GPI_DIV1_SHIFT                  (8U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GPI_DIV1_RESETVAL               (0x00000000U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GPI_DIV1_MAX                    (0x0000001fU)

#define CSL_ICSSCFG_GPCFG1_PRU1_GPI_SB_MASK                     (0x00002000U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GPI_SB_SHIFT                    (13U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GPI_SB_RESETVAL                 (0x00000000U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GPI_SB_MAX                      (0x00000001U)

#define CSL_ICSSCFG_GPCFG1_PRU1_GPO_MODE_MASK                   (0x00004000U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GPO_MODE_SHIFT                  (14U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GPO_MODE_RESETVAL               (0x00000000U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GPO_MODE_MAX                    (0x00000001U)

#define CSL_ICSSCFG_GPCFG1_PRU1_GPO_DIV0_MASK                   (0x000F8000U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GPO_DIV0_SHIFT                  (15U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GPO_DIV0_RESETVAL               (0x00000000U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GPO_DIV0_MAX                    (0x0000001fU)

#define CSL_ICSSCFG_GPCFG1_PRU1_GPO_DIV1_MASK                   (0x01F00000U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GPO_DIV1_SHIFT                  (20U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GPO_DIV1_RESETVAL               (0x00000000U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GPO_DIV1_MAX                    (0x0000001fU)

#define CSL_ICSSCFG_GPCFG1_PRU1_GPO_SH_SEL_MASK                 (0x02000000U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GPO_SH_SEL_SHIFT                (25U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GPO_SH_SEL_RESETVAL             (0x00000000U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GPO_SH_SEL_MAX                  (0x00000001U)

#ifndef CSL_MODIFICATION
#define CSL_ICSSCFG_GPCFG1_PRU1_GP_MUX_SEL_MASK                 (0x3C000000U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GP_MUX_SEL_SHIFT                (26U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GP_MUX_SEL_RESETVAL             (0x00000000U)
#define CSL_ICSSCFG_GPCFG1_PRU1_GP_MUX_SEL_MAX                  (0x00000004U)
#endif

#define CSL_ICSSCFG_GPCFG1_RESETVAL                             (0x00000000U)

/* CGR */

#define CSL_ICSSCFG_CGR_PRU0_CLK_STOP_REQ_MASK                  (0x00000001U)
#define CSL_ICSSCFG_CGR_PRU0_CLK_STOP_REQ_SHIFT                 (0U)
#define CSL_ICSSCFG_CGR_PRU0_CLK_STOP_REQ_RESETVAL              (0x00000000U)
#define CSL_ICSSCFG_CGR_PRU0_CLK_STOP_REQ_MAX                   (0x00000001U)

#define CSL_ICSSCFG_CGR_PRU0_CLK_STOP_ACK_MASK                  (0x00000002U)
#define CSL_ICSSCFG_CGR_PRU0_CLK_STOP_ACK_SHIFT                 (1U)
#define CSL_ICSSCFG_CGR_PRU0_CLK_STOP_ACK_RESETVAL              (0x00000000U)
#define CSL_ICSSCFG_CGR_PRU0_CLK_STOP_ACK_MAX                   (0x00000001U)

#define CSL_ICSSCFG_CGR_PRU0_CLK_EN_MASK                        (0x00000004U)
#define CSL_ICSSCFG_CGR_PRU0_CLK_EN_SHIFT                       (2U)
#define CSL_ICSSCFG_CGR_PRU0_CLK_EN_RESETVAL                    (0x00000001U)
#define CSL_ICSSCFG_CGR_PRU0_CLK_EN_MAX                         (0x00000001U)

#define CSL_ICSSCFG_CGR_PRU1_CLK_STOP_REQ_MASK                  (0x00000008U)
#define CSL_ICSSCFG_CGR_PRU1_CLK_STOP_REQ_SHIFT                 (3U)
#define CSL_ICSSCFG_CGR_PRU1_CLK_STOP_REQ_RESETVAL              (0x00000000U)
#define CSL_ICSSCFG_CGR_PRU1_CLK_STOP_REQ_MAX                   (0x00000001U)

#define CSL_ICSSCFG_CGR_PRU1_CLK_STOP_ACK_MASK                  (0x00000010U)
#define CSL_ICSSCFG_CGR_PRU1_CLK_STOP_ACK_SHIFT                 (4U)
#define CSL_ICSSCFG_CGR_PRU1_CLK_STOP_ACK_RESETVAL              (0x00000000U)
#define CSL_ICSSCFG_CGR_PRU1_CLK_STOP_ACK_MAX                   (0x00000001U)

#define CSL_ICSSCFG_CGR_PRU1_CLK_EN_MASK                        (0x00000020U)
#define CSL_ICSSCFG_CGR_PRU1_CLK_EN_SHIFT                       (5U)
#define CSL_ICSSCFG_CGR_PRU1_CLK_EN_RESETVAL                    (0x00000001U)
#define CSL_ICSSCFG_CGR_PRU1_CLK_EN_MAX                         (0x00000001U)

#define CSL_ICSSCFG_CGR_INTC_CLK_STOP_REQ_MASK                  (0x00000040U)
#define CSL_ICSSCFG_CGR_INTC_CLK_STOP_REQ_SHIFT                 (6U)
#define CSL_ICSSCFG_CGR_INTC_CLK_STOP_REQ_RESETVAL              (0x00000000U)
#define CSL_ICSSCFG_CGR_INTC_CLK_STOP_REQ_MAX                   (0x00000001U)

#define CSL_ICSSCFG_CGR_INTC_CLK_STOP_ACK_MASK                  (0x00000080U)
#define CSL_ICSSCFG_CGR_INTC_CLK_STOP_ACK_SHIFT                 (7U)
#define CSL_ICSSCFG_CGR_INTC_CLK_STOP_ACK_RESETVAL              (0x00000000U)
#define CSL_ICSSCFG_CGR_INTC_CLK_STOP_ACK_MAX                   (0x00000001U)

#define CSL_ICSSCFG_CGR_INTC_CLK_EN_MASK                        (0x00000100U)
#define CSL_ICSSCFG_CGR_INTC_CLK_EN_SHIFT                       (8U)
#define CSL_ICSSCFG_CGR_INTC_CLK_EN_RESETVAL                    (0x00000001U)
#define CSL_ICSSCFG_CGR_INTC_CLK_EN_MAX                         (0x00000001U)

#define CSL_ICSSCFG_CGR_UART_CLK_STOP_REQ_MASK                  (0x00000200U)
#define CSL_ICSSCFG_CGR_UART_CLK_STOP_REQ_SHIFT                 (9U)
#define CSL_ICSSCFG_CGR_UART_CLK_STOP_REQ_RESETVAL              (0x00000000U)
#define CSL_ICSSCFG_CGR_UART_CLK_STOP_REQ_MAX                   (0x00000001U)

#define CSL_ICSSCFG_CGR_UART_CLK_STOP_ACK_MASK                  (0x00000400U)
#define CSL_ICSSCFG_CGR_UART_CLK_STOP_ACK_SHIFT                 (10U)
#define CSL_ICSSCFG_CGR_UART_CLK_STOP_ACK_RESETVAL              (0x00000000U)
#define CSL_ICSSCFG_CGR_UART_CLK_STOP_ACK_MAX                   (0x00000001U)

#define CSL_ICSSCFG_CGR_UART_CLK_EN_MASK                        (0x00000800U)
#define CSL_ICSSCFG_CGR_UART_CLK_EN_SHIFT                       (11U)
#define CSL_ICSSCFG_CGR_UART_CLK_EN_RESETVAL                    (0x00000001U)
#define CSL_ICSSCFG_CGR_UART_CLK_EN_MAX                         (0x00000001U)

#define CSL_ICSSCFG_CGR_ECAP_CLK_STOP_REQ_MASK                  (0x00001000U)
#define CSL_ICSSCFG_CGR_ECAP_CLK_STOP_REQ_SHIFT                 (12U)
#define CSL_ICSSCFG_CGR_ECAP_CLK_STOP_REQ_RESETVAL              (0x00000000U)
#define CSL_ICSSCFG_CGR_ECAP_CLK_STOP_REQ_MAX                   (0x00000001U)

#define CSL_ICSSCFG_CGR_ECAP_CLK_STOP_ACK_MASK                  (0x00002000U)
#define CSL_ICSSCFG_CGR_ECAP_CLK_STOP_ACK_SHIFT                 (13U)
#define CSL_ICSSCFG_CGR_ECAP_CLK_STOP_ACK_RESETVAL              (0x00000000U)
#define CSL_ICSSCFG_CGR_ECAP_CLK_STOP_ACK_MAX                   (0x00000001U)

#define CSL_ICSSCFG_CGR_ECAP_CLK_EN_MASK                        (0x00004000U)
#define CSL_ICSSCFG_CGR_ECAP_CLK_EN_SHIFT                       (14U)
#define CSL_ICSSCFG_CGR_ECAP_CLK_EN_RESETVAL                    (0x00000001U)
#define CSL_ICSSCFG_CGR_ECAP_CLK_EN_MAX                         (0x00000001U)

#define CSL_ICSSCFG_CGR_IEP_CLK_STOP_REQ_MASK                   (0x00008000U)
#define CSL_ICSSCFG_CGR_IEP_CLK_STOP_REQ_SHIFT                  (15U)
#define CSL_ICSSCFG_CGR_IEP_CLK_STOP_REQ_RESETVAL               (0x00000000U)
#define CSL_ICSSCFG_CGR_IEP_CLK_STOP_REQ_MAX                    (0x00000001U)

#define CSL_ICSSCFG_CGR_IEP_CLK_STOP_ACK_MASK                   (0x00010000U)
#define CSL_ICSSCFG_CGR_IEP_CLK_STOP_ACK_SHIFT                  (16U)
#define CSL_ICSSCFG_CGR_IEP_CLK_STOP_ACK_RESETVAL               (0x00000000U)
#define CSL_ICSSCFG_CGR_IEP_CLK_STOP_ACK_MAX                    (0x00000001U)

#define CSL_ICSSCFG_CGR_IEP_CLK_EN_MASK                         (0x00020000U)
#define CSL_ICSSCFG_CGR_IEP_CLK_EN_SHIFT                        (17U)
#define CSL_ICSSCFG_CGR_IEP_CLK_EN_RESETVAL                     (0x00000001U)
#define CSL_ICSSCFG_CGR_IEP_CLK_EN_MAX                          (0x00000001U)

#define CSL_ICSSCFG_CGR_RESETVAL                                (0x00024924U)

/* ISRP */

#define CSL_ICSSCFG_ISRP_PRU0_IMEM_PE_RAW_MASK                  (0x0000000FU)
#define CSL_ICSSCFG_ISRP_PRU0_IMEM_PE_RAW_SHIFT                 (0U)
#define CSL_ICSSCFG_ISRP_PRU0_IMEM_PE_RAW_RESETVAL              (0x00000000U)
#define CSL_ICSSCFG_ISRP_PRU0_IMEM_PE_RAW_MAX                   (0x0000000fU)

#define CSL_ICSSCFG_ISRP_PRU0_DMEM_PE_RAW_MASK                  (0x000000F0U)
#define CSL_ICSSCFG_ISRP_PRU0_DMEM_PE_RAW_SHIFT                 (4U)
#define CSL_ICSSCFG_ISRP_PRU0_DMEM_PE_RAW_RESETVAL              (0x00000000U)
#define CSL_ICSSCFG_ISRP_PRU0_DMEM_PE_RAW_MAX                   (0x0000000fU)

#define CSL_ICSSCFG_ISRP_PRU1_IMEM_PE_RAW_MASK                  (0x00000F00U)
#define CSL_ICSSCFG_ISRP_PRU1_IMEM_PE_RAW_SHIFT                 (8U)
#define CSL_ICSSCFG_ISRP_PRU1_IMEM_PE_RAW_RESETVAL              (0x00000000U)
#define CSL_ICSSCFG_ISRP_PRU1_IMEM_PE_RAW_MAX                   (0x0000000fU)

#define CSL_ICSSCFG_ISRP_PRU1_DMEM_PE_RAW_MASK                  (0x0000F000U)
#define CSL_ICSSCFG_ISRP_PRU1_DMEM_PE_RAW_SHIFT                 (12U)
#define CSL_ICSSCFG_ISRP_PRU1_DMEM_PE_RAW_RESETVAL              (0x00000000U)
#define CSL_ICSSCFG_ISRP_PRU1_DMEM_PE_RAW_MAX                   (0x0000000fU)

#define CSL_ICSSCFG_ISRP_RAM_PE_RAW_MASK                        (0x000F0000U)
#define CSL_ICSSCFG_ISRP_RAM_PE_RAW_SHIFT                       (16U)
#define CSL_ICSSCFG_ISRP_RAM_PE_RAW_RESETVAL                    (0x00000000U)
#define CSL_ICSSCFG_ISRP_RAM_PE_RAW_MAX                         (0x0000000fU)

#define CSL_ICSSCFG_ISRP_RESETVAL                               (0x00000000U)

/* ISP */

#define CSL_ICSSCFG_ISP_PRU0_IMEM_PE_MASK                       (0x0000000FU)
#define CSL_ICSSCFG_ISP_PRU0_IMEM_PE_SHIFT                      (0U)
#define CSL_ICSSCFG_ISP_PRU0_IMEM_PE_RESETVAL                   (0x00000000U)
#define CSL_ICSSCFG_ISP_PRU0_IMEM_PE_MAX                        (0x0000000fU)

#define CSL_ICSSCFG_ISP_PRU0_DMEM_PE_MASK                       (0x000000F0U)
#define CSL_ICSSCFG_ISP_PRU0_DMEM_PE_SHIFT                      (4U)
#define CSL_ICSSCFG_ISP_PRU0_DMEM_PE_RESETVAL                   (0x00000000U)
#define CSL_ICSSCFG_ISP_PRU0_DMEM_PE_MAX                        (0x0000000fU)

#define CSL_ICSSCFG_ISP_PRU1_IMEM_PE_MASK                       (0x00000F00U)
#define CSL_ICSSCFG_ISP_PRU1_IMEM_PE_SHIFT                      (8U)
#define CSL_ICSSCFG_ISP_PRU1_IMEM_PE_RESETVAL                   (0x00000000U)
#define CSL_ICSSCFG_ISP_PRU1_IMEM_PE_MAX                        (0x0000000fU)

#define CSL_ICSSCFG_ISP_PRU1_DMEM_PE_MASK                       (0x0000F000U)
#define CSL_ICSSCFG_ISP_PRU1_DMEM_PE_SHIFT                      (12U)
#define CSL_ICSSCFG_ISP_PRU1_DMEM_PE_RESETVAL                   (0x00000000U)
#define CSL_ICSSCFG_ISP_PRU1_DMEM_PE_MAX                        (0x0000000fU)

#define CSL_ICSSCFG_ISP_RAM_PE_MASK                             (0x000F0000U)
#define CSL_ICSSCFG_ISP_RAM_PE_SHIFT                            (16U)
#define CSL_ICSSCFG_ISP_RAM_PE_RESETVAL                         (0x00000000U)
#define CSL_ICSSCFG_ISP_RAM_PE_MAX                              (0x0000000fU)

#define CSL_ICSSCFG_ISP_RESETVAL                                (0x00000000U)

/* IESP */

#define CSL_ICSSCFG_IESP_PRU0_IMEM_PE_SET_MASK                  (0x0000000FU)
#define CSL_ICSSCFG_IESP_PRU0_IMEM_PE_SET_SHIFT                 (0U)
#define CSL_ICSSCFG_IESP_PRU0_IMEM_PE_SET_RESETVAL              (0x00000000U)
#define CSL_ICSSCFG_IESP_PRU0_IMEM_PE_SET_MAX                   (0x0000000fU)

#define CSL_ICSSCFG_IESP_PRU0_DMEM_PE_SET_MASK                  (0x000000F0U)
#define CSL_ICSSCFG_IESP_PRU0_DMEM_PE_SET_SHIFT                 (4U)
#define CSL_ICSSCFG_IESP_PRU0_DMEM_PE_SET_RESETVAL              (0x00000000U)
#define CSL_ICSSCFG_IESP_PRU0_DMEM_PE_SET_MAX                   (0x0000000fU)

#define CSL_ICSSCFG_IESP_PRU1_IMEM_PE_SET_MASK                  (0x00000F00U)
#define CSL_ICSSCFG_IESP_PRU1_IMEM_PE_SET_SHIFT                 (8U)
#define CSL_ICSSCFG_IESP_PRU1_IMEM_PE_SET_RESETVAL              (0x00000000U)
#define CSL_ICSSCFG_IESP_PRU1_IMEM_PE_SET_MAX                   (0x0000000fU)

#define CSL_ICSSCFG_IESP_PRU1_DMEM_PE_SET_MASK                  (0x0000F000U)
#define CSL_ICSSCFG_IESP_PRU1_DMEM_PE_SET_SHIFT                 (12U)
#define CSL_ICSSCFG_IESP_PRU1_DMEM_PE_SET_RESETVAL              (0x00000000U)
#define CSL_ICSSCFG_IESP_PRU1_DMEM_PE_SET_MAX                   (0x0000000fU)

#define CSL_ICSSCFG_IESP_RAM_PE_SET_MASK                        (0x000F0000U)
#define CSL_ICSSCFG_IESP_RAM_PE_SET_SHIFT                       (16U)
#define CSL_ICSSCFG_IESP_RAM_PE_SET_RESETVAL                    (0x00000000U)
#define CSL_ICSSCFG_IESP_RAM_PE_SET_MAX                         (0x0000000fU)

#define CSL_ICSSCFG_IESP_RESETVAL                               (0x00000000U)

/* IECP */

#define CSL_ICSSCFG_IECP_PRU0_IMEM_PE_CLR_MASK                  (0x0000000FU)
#define CSL_ICSSCFG_IECP_PRU0_IMEM_PE_CLR_SHIFT                 (0U)
#define CSL_ICSSCFG_IECP_PRU0_IMEM_PE_CLR_RESETVAL              (0x00000000U)
#define CSL_ICSSCFG_IECP_PRU0_IMEM_PE_CLR_MAX                   (0x0000000fU)

#define CSL_ICSSCFG_IECP_PRU0_DMEM_PE_CLR_MASK                  (0x000000F0U)
#define CSL_ICSSCFG_IECP_PRU0_DMEM_PE_CLR_SHIFT                 (4U)
#define CSL_ICSSCFG_IECP_PRU0_DMEM_PE_CLR_RESETVAL              (0x00000000U)
#define CSL_ICSSCFG_IECP_PRU0_DMEM_PE_CLR_MAX                   (0x0000000fU)

#define CSL_ICSSCFG_IECP_PRU1_IMEM_PE_CLR_MASK                  (0x00000F00U)
#define CSL_ICSSCFG_IECP_PRU1_IMEM_PE_CLR_SHIFT                 (8U)
#define CSL_ICSSCFG_IECP_PRU1_IMEM_PE_CLR_RESETVAL              (0x00000000U)
#define CSL_ICSSCFG_IECP_PRU1_IMEM_PE_CLR_MAX                   (0x0000000fU)

#define CSL_ICSSCFG_IECP_PRU1_DMEM_PE_CLR_MASK                  (0x0000F000U)
#define CSL_ICSSCFG_IECP_PRU1_DMEM_PE_CLR_SHIFT                 (12U)
#define CSL_ICSSCFG_IECP_PRU1_DMEM_PE_CLR_RESETVAL              (0x00000000U)
#define CSL_ICSSCFG_IECP_PRU1_DMEM_PE_CLR_MAX                   (0x0000000fU)

#define CSL_ICSSCFG_IECP_RESETVAL                               (0x00000000U)

/* SCRP */

#define CSL_ICSSCFG_SCRP_SCRP_1_MASK                            (0x00000003U)
#define CSL_ICSSCFG_SCRP_SCRP_1_SHIFT                           (0U)
#define CSL_ICSSCFG_SCRP_SCRP_1_RESETVAL                        (0x00000003U)
#define CSL_ICSSCFG_SCRP_SCRP_1_MAX                             (0x00000003U)

#define CSL_ICSSCFG_SCRP_SCRP_2_MASK                            (0x0000000CU)
#define CSL_ICSSCFG_SCRP_SCRP_2_SHIFT                           (2U)
#define CSL_ICSSCFG_SCRP_SCRP_2_RESETVAL                        (0x00000002U)
#define CSL_ICSSCFG_SCRP_SCRP_2_MAX                             (0x00000003U)

#define CSL_ICSSCFG_SCRP_SCRP_3_MASK                            (0x00000030U)
#define CSL_ICSSCFG_SCRP_SCRP_3_SHIFT                           (4U)
#define CSL_ICSSCFG_SCRP_SCRP_3_RESETVAL                        (0x00000000U)
#define CSL_ICSSCFG_SCRP_SCRP_3_MAX                             (0x00000003U)

#define CSL_ICSSCFG_SCRP_SCRP_4_MASK                            (0x000000C0U)
#define CSL_ICSSCFG_SCRP_SCRP_4_SHIFT                           (6U)
#define CSL_ICSSCFG_SCRP_SCRP_4_RESETVAL                        (0x00000001U)
#define CSL_ICSSCFG_SCRP_SCRP_4_MAX                             (0x00000003U)

#define CSL_ICSSCFG_SCRP_SCRP_5_MASK                            (0x00000100U)
#define CSL_ICSSCFG_SCRP_SCRP_5_SHIFT                           (8U)
#define CSL_ICSSCFG_SCRP_SCRP_5_RESETVAL                        (0x00000000U)
#define CSL_ICSSCFG_SCRP_SCRP_5_MAX                             (0x00000001U)

#define CSL_ICSSCFG_SCRP_SCRP_6_MASK                            (0x00000200U)
#define CSL_ICSSCFG_SCRP_SCRP_6_SHIFT                           (9U)
#define CSL_ICSSCFG_SCRP_SCRP_6_RESETVAL                        (0x00000000U)
#define CSL_ICSSCFG_SCRP_SCRP_6_MAX                             (0x00000001U)

#define CSL_ICSSCFG_SCRP_SCRP_7_MASK                            (0x00000400U)
#define CSL_ICSSCFG_SCRP_SCRP_7_SHIFT                           (10U)
#define CSL_ICSSCFG_SCRP_SCRP_7_RESETVAL                        (0x00000000U)
#define CSL_ICSSCFG_SCRP_SCRP_7_MAX                             (0x00000001U)

#define CSL_ICSSCFG_SCRP_SCRP_8_MASK                            (0x00000800U)
#define CSL_ICSSCFG_SCRP_SCRP_8_SHIFT                           (11U)
#define CSL_ICSSCFG_SCRP_SCRP_8_RESETVAL                        (0x00000001U)
#define CSL_ICSSCFG_SCRP_SCRP_8_MAX                             (0x00000001U)

#define CSL_ICSSCFG_SCRP_SCRP_9_MASK                            (0x00001000U)
#define CSL_ICSSCFG_SCRP_SCRP_9_SHIFT                           (12U)
#define CSL_ICSSCFG_SCRP_SCRP_9_RESETVAL                        (0x00000000U)
#define CSL_ICSSCFG_SCRP_SCRP_9_MAX                             (0x00000001U)

#define CSL_ICSSCFG_SCRP_SCRP_10_MASK                           (0x00002000U)
#define CSL_ICSSCFG_SCRP_SCRP_10_SHIFT                          (13U)
#define CSL_ICSSCFG_SCRP_SCRP_10_RESETVAL                       (0x00000001U)
#define CSL_ICSSCFG_SCRP_SCRP_10_MAX                            (0x00000001U)

#define CSL_ICSSCFG_SCRP_SCRP_11_MASK                           (0x00004000U)
#define CSL_ICSSCFG_SCRP_SCRP_11_SHIFT                          (14U)
#define CSL_ICSSCFG_SCRP_SCRP_11_RESETVAL                       (0x00000000U)
#define CSL_ICSSCFG_SCRP_SCRP_11_MAX                            (0x00000001U)

#define CSL_ICSSCFG_SCRP_SCRP_12_MASK                           (0x00008000U)
#define CSL_ICSSCFG_SCRP_SCRP_12_SHIFT                          (15U)
#define CSL_ICSSCFG_SCRP_SCRP_12_RESETVAL                       (0x00000001U)
#define CSL_ICSSCFG_SCRP_SCRP_12_MAX                            (0x00000001U)

#define CSL_ICSSCFG_SCRP_SCRP_13_MASK                           (0x00010000U)
#define CSL_ICSSCFG_SCRP_SCRP_13_SHIFT                          (16U)
#define CSL_ICSSCFG_SCRP_SCRP_13_RESETVAL                       (0x00000000U)
#define CSL_ICSSCFG_SCRP_SCRP_13_MAX                            (0x00000001U)

#define CSL_ICSSCFG_SCRP_SCRP_14_MASK                           (0x00020000U)
#define CSL_ICSSCFG_SCRP_SCRP_14_SHIFT                          (17U)
#define CSL_ICSSCFG_SCRP_SCRP_14_RESETVAL                       (0x00000001U)
#define CSL_ICSSCFG_SCRP_SCRP_14_MAX                            (0x00000001U)

#define CSL_ICSSCFG_SCRP_SCRP_15_MASK                           (0x00040000U)
#define CSL_ICSSCFG_SCRP_SCRP_15_SHIFT                          (18U)
#define CSL_ICSSCFG_SCRP_SCRP_15_RESETVAL                       (0x00000000U)
#define CSL_ICSSCFG_SCRP_SCRP_15_MAX                            (0x00000001U)

#define CSL_ICSSCFG_SCRP_SCRP_16_MASK                           (0x00080000U)
#define CSL_ICSSCFG_SCRP_SCRP_16_SHIFT                          (19U)
#define CSL_ICSSCFG_SCRP_SCRP_16_RESETVAL                       (0x00000001U)
#define CSL_ICSSCFG_SCRP_SCRP_16_MAX                            (0x00000001U)

#define CSL_ICSSCFG_SCRP_SCRP_17_MASK                           (0x00100000U)
#define CSL_ICSSCFG_SCRP_SCRP_17_SHIFT                          (20U)
#define CSL_ICSSCFG_SCRP_SCRP_17_RESETVAL                       (0x00000000U)
#define CSL_ICSSCFG_SCRP_SCRP_17_MAX                            (0x00000001U)

#define CSL_ICSSCFG_SCRP_SCRP_18_MASK                           (0x00200000U)
#define CSL_ICSSCFG_SCRP_SCRP_18_SHIFT                          (21U)
#define CSL_ICSSCFG_SCRP_SCRP_18_RESETVAL                       (0x00000000U)
#define CSL_ICSSCFG_SCRP_SCRP_18_MAX                            (0x00000001U)

#define CSL_ICSSCFG_SCRP_RESETVAL                               (0x000aa84bU)

/* PMAO */

#define CSL_ICSSCFG_PMAO_PMAO_PRU0_MASK                         (0x00000001U)
#define CSL_ICSSCFG_PMAO_PMAO_PRU0_SHIFT                        (0U)
#define CSL_ICSSCFG_PMAO_PMAO_PRU0_RESETVAL                     (0x00000000U)
#define CSL_ICSSCFG_PMAO_PMAO_PRU0_MAX                          (0x00000001U)

#define CSL_ICSSCFG_PMAO_PMAO_PRU1_MASK                         (0x00000002U)
#define CSL_ICSSCFG_PMAO_PMAO_PRU1_SHIFT                        (1U)
#define CSL_ICSSCFG_PMAO_PMAO_PRU1_RESETVAL                     (0x00000000U)
#define CSL_ICSSCFG_PMAO_PMAO_PRU1_MAX                          (0x00000001U)

#define CSL_ICSSCFG_PMAO_RESETVAL                               (0x00000000U)

/* MII_RT */

#define CSL_ICSSCFG_MII_RT_MII_RT_EVENT_EN_MASK                 (0x00000001U)
#define CSL_ICSSCFG_MII_RT_MII_RT_EVENT_EN_SHIFT                (0U)
#define CSL_ICSSCFG_MII_RT_MII_RT_EVENT_EN_RESETVAL             (0x00000001U)
#define CSL_ICSSCFG_MII_RT_MII_RT_EVENT_EN_MAX                  (0x00000001U)

#define CSL_ICSSCFG_MII_RT_RESETVAL                             (0x00000001U)

/* IEPCLK */

#define CSL_ICSSCFG_IEPCLK_OCP_EN_MASK                          (0x00000001U)
#define CSL_ICSSCFG_IEPCLK_OCP_EN_SHIFT                         (0U)
#define CSL_ICSSCFG_IEPCLK_OCP_EN_RESETVAL                      (0x00000000U)
#define CSL_ICSSCFG_IEPCLK_OCP_EN_MAX                           (0x00000001U)

#define CSL_ICSSCFG_IEPCLK_RESETVAL                             (0x00000000U)

/* SPP */

#define CSL_ICSSCFG_SPP_PRU1_PAD_HP_EN_MASK                     (0x00000001U)
#define CSL_ICSSCFG_SPP_PRU1_PAD_HP_EN_SHIFT                    (0U)
#define CSL_ICSSCFG_SPP_PRU1_PAD_HP_EN_RESETVAL                 (0x00000000U)
#define CSL_ICSSCFG_SPP_PRU1_PAD_HP_EN_MAX                      (0x00000001U)

#define CSL_ICSSCFG_SPP_XFR_SHIFT_EN_MASK                       (0x00000002U)
#define CSL_ICSSCFG_SPP_XFR_SHIFT_EN_SHIFT                      (1U)
#define CSL_ICSSCFG_SPP_XFR_SHIFT_EN_RESETVAL                   (0x00000000U)
#define CSL_ICSSCFG_SPP_XFR_SHIFT_EN_MAX                        (0x00000001U)

#define CSL_ICSSCFG_SPP_RESETVAL                                (0x00000000U)

/* PIN_MX */

#define CSL_ICSSCFG_PIN_MX_PIN_MUX_SEL_MASK                     (0x000000FFU)
#define CSL_ICSSCFG_PIN_MX_PIN_MUX_SEL_SHIFT                    (0U)
#define CSL_ICSSCFG_PIN_MX_PIN_MUX_SEL_RESETVAL                 (0x00000000U)
#define CSL_ICSSCFG_PIN_MX_PIN_MUX_SEL_MAX                      (0x000000ffU)

#define CSL_ICSSCFG_PIN_MX_PWM0_REMAP_EN_MASK                   (0x00000100U)
#define CSL_ICSSCFG_PIN_MX_PWM0_REMAP_EN_SHIFT                  (8U)
#define CSL_ICSSCFG_PIN_MX_PWM0_REMAP_EN_RESETVAL               (0x00000000U)
#define CSL_ICSSCFG_PIN_MX_PWM0_REMAP_EN_MAX                    (0x00000001U)

#define CSL_ICSSCFG_PIN_MX_PWM3_REMAP_EN_MASK                   (0x00000200U)
#define CSL_ICSSCFG_PIN_MX_PWM3_REMAP_EN_SHIFT                  (9U)
#define CSL_ICSSCFG_PIN_MX_PWM3_REMAP_EN_RESETVAL               (0x00000000U)
#define CSL_ICSSCFG_PIN_MX_PWM3_REMAP_EN_MAX                    (0x00000001U)

#define CSL_ICSSCFG_PIN_MX_RESETVAL                             (0x00000000U)

/* SDPRU0CLKDIV */

#define CSL_ICSSCFG_SDPRU0CLKDIV_PRU0_SD_DIVFACTOR_MASK         (0x0000000FU)
#define CSL_ICSSCFG_SDPRU0CLKDIV_PRU0_SD_DIVFACTOR_SHIFT        (0U)
#define CSL_ICSSCFG_SDPRU0CLKDIV_PRU0_SD_DIVFACTOR_RESETVAL     (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKDIV_PRU0_SD_DIVFACTOR_MAX          (0x0000000fU)

#define CSL_ICSSCFG_SDPRU0CLKDIV_PRU0_SD_DIVFACTOR_FRAC_MASK    (0x00000010U)
#define CSL_ICSSCFG_SDPRU0CLKDIV_PRU0_SD_DIVFACTOR_FRAC_SHIFT   (4U)
#define CSL_ICSSCFG_SDPRU0CLKDIV_PRU0_SD_DIVFACTOR_FRAC_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKDIV_PRU0_SD_DIVFACTOR_FRAC_MAX     (0x00000001U)

#define CSL_ICSSCFG_SDPRU0CLKDIV_RESETVAL                       (0x00000000U)

/* SDPRU0CLKSELREGISTER0 */

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER0_PRU0_SD_CLK_SEL0_MASK  (0x00000003U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER0_PRU0_SD_CLK_SEL0_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER0_PRU0_SD_CLK_SEL0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER0_PRU0_SD_CLK_SEL0_MAX  (0x00000003U)

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER0_PRU0_SD_CLK_INV0_MASK  (0x00000004U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER0_PRU0_SD_CLK_INV0_SHIFT  (2U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER0_PRU0_SD_CLK_INV0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER0_PRU0_SD_CLK_INV0_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER0_PRU0_SD_ACC2_SEL0_MASK  (0x00000010U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER0_PRU0_SD_ACC2_SEL0_SHIFT  (4U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER0_PRU0_SD_ACC2_SEL0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER0_PRU0_SD_ACC2_SEL0_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER0_RESETVAL              (0x00000000U)

/* SDPRU0SAMPLESIZEREGISTER0 */

#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER0_PRU0_SD_SAMPLE_SIZE0_MASK  (0x000000FFU)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER0_PRU0_SD_SAMPLE_SIZE0_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER0_PRU0_SD_SAMPLE_SIZE0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER0_PRU0_SD_SAMPLE_SIZE0_MAX  (0x000000ffU)

#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER0_RESETVAL          (0x00000000U)

/* SDPRU0CLKSELREGISTER1 */

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER1_PRU0_SD_CLK_SEL1_MASK  (0x00000003U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER1_PRU0_SD_CLK_SEL1_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER1_PRU0_SD_CLK_SEL1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER1_PRU0_SD_CLK_SEL1_MAX  (0x00000003U)

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER1_PRU0_SD_CLK_INV1_MASK  (0x00000004U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER1_PRU0_SD_CLK_INV1_SHIFT  (2U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER1_PRU0_SD_CLK_INV1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER1_PRU0_SD_CLK_INV1_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER1_PRU0_SD_ACC2_SEL1_MASK  (0x00000010U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER1_PRU0_SD_ACC2_SEL1_SHIFT  (4U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER1_PRU0_SD_ACC2_SEL1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER1_PRU0_SD_ACC2_SEL1_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER1_RESETVAL              (0x00000000U)

/* SDPRU0SAMPLESIZEREGISTER1 */

#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER1_PRU0_SD_SAMPLE_SIZE1_MASK  (0x000000FFU)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER1_PRU0_SD_SAMPLE_SIZE1_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER1_PRU0_SD_SAMPLE_SIZE1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER1_PRU0_SD_SAMPLE_SIZE1_MAX  (0x000000ffU)

#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER1_RESETVAL          (0x00000000U)

/* SDPRU0CLKSELREGISTER2 */

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER2_PRU0_SD_CLK_SEL2_MASK  (0x00000003U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER2_PRU0_SD_CLK_SEL2_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER2_PRU0_SD_CLK_SEL2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER2_PRU0_SD_CLK_SEL2_MAX  (0x00000003U)

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER2_PRU0_SD_CLK_INV2_MASK  (0x00000004U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER2_PRU0_SD_CLK_INV2_SHIFT  (2U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER2_PRU0_SD_CLK_INV2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER2_PRU0_SD_CLK_INV2_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER2_PRU0_SD_ACC2_SEL2_MASK  (0x00000010U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER2_PRU0_SD_ACC2_SEL2_SHIFT  (4U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER2_PRU0_SD_ACC2_SEL2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER2_PRU0_SD_ACC2_SEL2_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER2_RESETVAL              (0x00000000U)

/* SDPRU0SAMPLESIZEREGISTER2 */

#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER2_PRU0_SD_SAMPLE_SIZE2_MASK  (0x000000FFU)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER2_PRU0_SD_SAMPLE_SIZE2_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER2_PRU0_SD_SAMPLE_SIZE2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER2_PRU0_SD_SAMPLE_SIZE2_MAX  (0x000000ffU)

#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER2_RESETVAL          (0x00000000U)

/* SDPRU0CLKSELREGISTER3 */

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER3_PRU0_SD_CLK_SEL3_MASK  (0x00000003U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER3_PRU0_SD_CLK_SEL3_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER3_PRU0_SD_CLK_SEL3_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER3_PRU0_SD_CLK_SEL3_MAX  (0x00000003U)

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER3_PRU0_SD_CLK_INV3_MASK  (0x00000004U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER3_PRU0_SD_CLK_INV3_SHIFT  (2U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER3_PRU0_SD_CLK_INV3_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER3_PRU0_SD_CLK_INV3_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER3_PRU0_SD_ACC2_SEL3_MASK  (0x00000010U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER3_PRU0_SD_ACC2_SEL3_SHIFT  (4U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER3_PRU0_SD_ACC2_SEL3_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER3_PRU0_SD_ACC2_SEL3_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER3_RESETVAL              (0x00000000U)

/* SDPRU0SAMPLESIZEREGISTER3 */

#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER3_PRU0_SD_SAMPLE_SIZE3_MASK  (0x000000FFU)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER3_PRU0_SD_SAMPLE_SIZE3_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER3_PRU0_SD_SAMPLE_SIZE3_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER3_PRU0_SD_SAMPLE_SIZE3_MAX  (0x000000ffU)

#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER3_RESETVAL          (0x00000000U)

/* SDPRU0CLKSELREGISTER4 */

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER4_PRU0_SD_CLK_SEL4_MASK  (0x00000003U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER4_PRU0_SD_CLK_SEL4_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER4_PRU0_SD_CLK_SEL4_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER4_PRU0_SD_CLK_SEL4_MAX  (0x00000003U)

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER4_PRU0_SD_CLK_INV4_MASK  (0x00000004U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER4_PRU0_SD_CLK_INV4_SHIFT  (2U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER4_PRU0_SD_CLK_INV4_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER4_PRU0_SD_CLK_INV4_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER4_PRU0_SD_ACC2_SEL4_MASK  (0x00000010U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER4_PRU0_SD_ACC2_SEL4_SHIFT  (4U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER4_PRU0_SD_ACC2_SEL4_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER4_PRU0_SD_ACC2_SEL4_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER4_RESETVAL              (0x00000000U)

/* SDPRU0SAMPLESIZEREGISTER4 */

#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER4_PRU0_SD_SAMPLE_SIZE4_MASK  (0x000000FFU)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER4_PRU0_SD_SAMPLE_SIZE4_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER4_PRU0_SD_SAMPLE_SIZE4_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER4_PRU0_SD_SAMPLE_SIZE4_MAX  (0x000000ffU)

#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER4_RESETVAL          (0x00000000U)

/* SDPRU0CLKSELREGISTER5 */

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER5_PRU0_SD_CLK_SEL5_MASK  (0x00000003U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER5_PRU0_SD_CLK_SEL5_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER5_PRU0_SD_CLK_SEL5_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER5_PRU0_SD_CLK_SEL5_MAX  (0x00000003U)

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER5_PRU0_SD_CLK_INV5_MASK  (0x00000004U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER5_PRU0_SD_CLK_INV5_SHIFT  (2U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER5_PRU0_SD_CLK_INV5_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER5_PRU0_SD_CLK_INV5_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER5_PRU0_SD_ACC2_SEL5_MASK  (0x00000010U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER5_PRU0_SD_ACC2_SEL5_SHIFT  (4U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER5_PRU0_SD_ACC2_SEL5_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER5_PRU0_SD_ACC2_SEL5_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER5_RESETVAL              (0x00000000U)

/* SDPRU0SAMPLESIZEREGISTER5 */

#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER5_PRU0_SD_SAMPLE_SIZE5_MASK  (0x000000FFU)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER5_PRU0_SD_SAMPLE_SIZE5_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER5_PRU0_SD_SAMPLE_SIZE5_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER5_PRU0_SD_SAMPLE_SIZE5_MAX  (0x000000ffU)

#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER5_RESETVAL          (0x00000000U)

/* SDPRU0CLKSELREGISTER6 */

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER6_PRU0_SD_CLK_SEL6_MASK  (0x00000003U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER6_PRU0_SD_CLK_SEL6_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER6_PRU0_SD_CLK_SEL6_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER6_PRU0_SD_CLK_SEL6_MAX  (0x00000003U)

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER6_PRU0_SD_CLK_INV6_MASK  (0x00000004U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER6_PRU0_SD_CLK_INV6_SHIFT  (2U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER6_PRU0_SD_CLK_INV6_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER6_PRU0_SD_CLK_INV6_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER6_PRU0_SD_ACC2_SEL6_MASK  (0x00000010U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER6_PRU0_SD_ACC2_SEL6_SHIFT  (4U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER6_PRU0_SD_ACC2_SEL6_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER6_PRU0_SD_ACC2_SEL6_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER6_RESETVAL              (0x00000000U)

/* SDPRU0SAMPLESIZEREGISTER6 */

#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER6_PRU0_SD_SAMPLE_SIZE6_MASK  (0x000000FFU)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER6_PRU0_SD_SAMPLE_SIZE6_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER6_PRU0_SD_SAMPLE_SIZE6_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER6_PRU0_SD_SAMPLE_SIZE6_MAX  (0x000000ffU)

#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER6_RESETVAL          (0x00000000U)

/* SDPRU0CLKSELREGISTER7 */

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER7_PRU0_SD_CLK_SEL7_MASK  (0x00000003U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER7_PRU0_SD_CLK_SEL7_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER7_PRU0_SD_CLK_SEL7_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER7_PRU0_SD_CLK_SEL7_MAX  (0x00000003U)

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER7_PRU0_SD_CLK_INV7_MASK  (0x00000004U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER7_PRU0_SD_CLK_INV7_SHIFT  (2U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER7_PRU0_SD_CLK_INV7_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER7_PRU0_SD_CLK_INV7_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER7_PRU0_SD_ACC2_SEL7_MASK  (0x00000010U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER7_PRU0_SD_ACC2_SEL7_SHIFT  (4U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER7_PRU0_SD_ACC2_SEL7_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER7_PRU0_SD_ACC2_SEL7_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER7_RESETVAL              (0x00000000U)

/* SDPRU0SAMPLESIZEREGISTER7 */

#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER7_PRU0_SD_SAMPLE_SIZE7_MASK  (0x000000FFU)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER7_PRU0_SD_SAMPLE_SIZE7_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER7_PRU0_SD_SAMPLE_SIZE7_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER7_PRU0_SD_SAMPLE_SIZE7_MAX  (0x000000ffU)

#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER7_RESETVAL          (0x00000000U)

/* SDPRU0CLKSELREGISTER8 */

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER8_PRU0_SD_CLK_SEL8_MASK  (0x00000003U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER8_PRU0_SD_CLK_SEL8_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER8_PRU0_SD_CLK_SEL8_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER8_PRU0_SD_CLK_SEL8_MAX  (0x00000003U)

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER8_PRU0_SD_CLK_INV8_MASK  (0x00000004U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER8_PRU0_SD_CLK_INV8_SHIFT  (2U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER8_PRU0_SD_CLK_INV8_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER8_PRU0_SD_CLK_INV8_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER8_PRU0_SD_ACC2_SEL8_MASK  (0x00000010U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER8_PRU0_SD_ACC2_SEL8_SHIFT  (4U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER8_PRU0_SD_ACC2_SEL8_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER8_PRU0_SD_ACC2_SEL8_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU0CLKSELREGISTER8_RESETVAL              (0x00000000U)

/* SDPRU0SAMPLESIZEREGISTER8 */

#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER8_PRU0_SD_SAMPLE_SIZE8_MASK  (0x000000FFU)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER8_PRU0_SD_SAMPLE_SIZE8_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER8_PRU0_SD_SAMPLE_SIZE8_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER8_PRU0_SD_SAMPLE_SIZE8_MAX  (0x000000ffU)

#define CSL_ICSSCFG_SDPRU0SAMPLESIZEREGISTER8_RESETVAL          (0x00000000U)

/* SDPRU1CLKDIVREGISTER */

#define CSL_ICSSCFG_SDPRU1CLKDIVREGISTER_PRU1_SD_DIVFACTOR_MASK  (0x0000000FU)
#define CSL_ICSSCFG_SDPRU1CLKDIVREGISTER_PRU1_SD_DIVFACTOR_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU1CLKDIVREGISTER_PRU1_SD_DIVFACTOR_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKDIVREGISTER_PRU1_SD_DIVFACTOR_MAX  (0x0000000fU)

#define CSL_ICSSCFG_SDPRU1CLKDIVREGISTER_PRU1_SD_DIVFACTOR_FRAC_MASK  (0x00000010U)
#define CSL_ICSSCFG_SDPRU1CLKDIVREGISTER_PRU1_SD_DIVFACTOR_FRAC_SHIFT  (4U)
#define CSL_ICSSCFG_SDPRU1CLKDIVREGISTER_PRU1_SD_DIVFACTOR_FRAC_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKDIVREGISTER_PRU1_SD_DIVFACTOR_FRAC_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU1CLKDIVREGISTER_RESETVAL               (0x00000000U)

/* SDPRU1CLKSELREGISTER0 */

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER0_PRU1_SD_CLK_SEL0_MASK  (0x00000003U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER0_PRU1_SD_CLK_SEL0_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER0_PRU1_SD_CLK_SEL0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER0_PRU1_SD_CLK_SEL0_MAX  (0x00000003U)

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER0_PRU1_SD_CLK_INV0_MASK  (0x00000004U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER0_PRU1_SD_CLK_INV0_SHIFT  (2U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER0_PRU1_SD_CLK_INV0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER0_PRU1_SD_CLK_INV0_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER0_PRU1_SD_ACC2_SEL0_MASK  (0x00000010U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER0_PRU1_SD_ACC2_SEL0_SHIFT  (4U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER0_PRU1_SD_ACC2_SEL0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER0_PRU1_SD_ACC2_SEL0_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER0_RESETVAL              (0x00000000U)

/* SDPRU1SAMPLESIZEREGISTER0 */

#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER0_PRU1_SD_SAMPLE_SIZE0_MASK  (0x000000FFU)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER0_PRU1_SD_SAMPLE_SIZE0_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER0_PRU1_SD_SAMPLE_SIZE0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER0_PRU1_SD_SAMPLE_SIZE0_MAX  (0x000000ffU)

#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER0_RESETVAL          (0x00000000U)

/* SDPRU1CLKSELREGISTER1 */

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER1_PRU1_SD_CLK_SEL1_MASK  (0x00000003U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER1_PRU1_SD_CLK_SEL1_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER1_PRU1_SD_CLK_SEL1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER1_PRU1_SD_CLK_SEL1_MAX  (0x00000003U)

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER1_PRU1_SD_CLK_INV1_MASK  (0x00000004U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER1_PRU1_SD_CLK_INV1_SHIFT  (2U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER1_PRU1_SD_CLK_INV1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER1_PRU1_SD_CLK_INV1_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER1_PRU1_SD_ACC2_SEL1_MASK  (0x00000010U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER1_PRU1_SD_ACC2_SEL1_SHIFT  (4U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER1_PRU1_SD_ACC2_SEL1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER1_PRU1_SD_ACC2_SEL1_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER1_RESETVAL              (0x00000000U)

/* SDPRU1SAMPLESIZEREGISTER1 */

#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER1_PRU1_SD_SAMPLE_SIZE1_MASK  (0x000000FFU)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER1_PRU1_SD_SAMPLE_SIZE1_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER1_PRU1_SD_SAMPLE_SIZE1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER1_PRU1_SD_SAMPLE_SIZE1_MAX  (0x000000ffU)

#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER1_RESETVAL          (0x00000000U)

/* SDPRU1CLKSELREGISTER2 */

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER2_PRU1_SD_CLK_SEL2_MASK  (0x00000003U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER2_PRU1_SD_CLK_SEL2_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER2_PRU1_SD_CLK_SEL2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER2_PRU1_SD_CLK_SEL2_MAX  (0x00000003U)

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER2_PRU1_SD_CLK_INV2_MASK  (0x00000004U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER2_PRU1_SD_CLK_INV2_SHIFT  (2U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER2_PRU1_SD_CLK_INV2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER2_PRU1_SD_CLK_INV2_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER2_PRU1_SD_ACC2_SEL2_MASK  (0x00000010U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER2_PRU1_SD_ACC2_SEL2_SHIFT  (4U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER2_PRU1_SD_ACC2_SEL2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER2_PRU1_SD_ACC2_SEL2_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER2_RESETVAL              (0x00000000U)

/* SDPRU1SAMPLESIZEREGISTER2 */

#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER2_PRU1_SD_SAMPLE_SIZE2_MASK  (0x000000FFU)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER2_PRU1_SD_SAMPLE_SIZE2_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER2_PRU1_SD_SAMPLE_SIZE2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER2_PRU1_SD_SAMPLE_SIZE2_MAX  (0x000000ffU)

#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER2_RESETVAL          (0x00000000U)

/* SDPRU1CLKSELREGISTER3 */

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER3_PRU1_SD_CLK_SEL3_MASK  (0x00000003U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER3_PRU1_SD_CLK_SEL3_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER3_PRU1_SD_CLK_SEL3_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER3_PRU1_SD_CLK_SEL3_MAX  (0x00000003U)

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER3_PRU1_SD_CLK_INV3_MASK  (0x00000004U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER3_PRU1_SD_CLK_INV3_SHIFT  (2U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER3_PRU1_SD_CLK_INV3_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER3_PRU1_SD_CLK_INV3_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER3_PRU1_SD_ACC2_SEL3_MASK  (0x00000010U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER3_PRU1_SD_ACC2_SEL3_SHIFT  (4U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER3_PRU1_SD_ACC2_SEL3_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER3_PRU1_SD_ACC2_SEL3_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER3_RESETVAL              (0x00000000U)

/* SDPRU1SAMPLESIZEREGISTER3 */

#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER3_PRU1_SD_SAMPLE_SIZE3_MASK  (0x000000FFU)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER3_PRU1_SD_SAMPLE_SIZE3_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER3_PRU1_SD_SAMPLE_SIZE3_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER3_PRU1_SD_SAMPLE_SIZE3_MAX  (0x000000ffU)

#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER3_RESETVAL          (0x00000000U)

/* SDPRU1CLKSELREGISTER4 */

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER4_PRU1_SD_CLK_SEL4_MASK  (0x00000003U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER4_PRU1_SD_CLK_SEL4_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER4_PRU1_SD_CLK_SEL4_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER4_PRU1_SD_CLK_SEL4_MAX  (0x00000003U)

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER4_PRU1_SD_CLK_INV4_MASK  (0x00000004U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER4_PRU1_SD_CLK_INV4_SHIFT  (2U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER4_PRU1_SD_CLK_INV4_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER4_PRU1_SD_CLK_INV4_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER4_PRU1_SD_ACC2_SEL4_MASK  (0x00000010U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER4_PRU1_SD_ACC2_SEL4_SHIFT  (4U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER4_PRU1_SD_ACC2_SEL4_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER4_PRU1_SD_ACC2_SEL4_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER4_RESETVAL              (0x00000000U)

/* SDPRU1SAMPLESIZEREGISTER4 */

#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER4_PRU1_SD_SAMPLE_SIZE4_MASK  (0x000000FFU)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER4_PRU1_SD_SAMPLE_SIZE4_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER4_PRU1_SD_SAMPLE_SIZE4_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER4_PRU1_SD_SAMPLE_SIZE4_MAX  (0x000000ffU)

#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER4_RESETVAL          (0x00000000U)

/* SDPRU1CLKSELREGISTER5 */

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER5_PRU1_SD_CLK_SEL5_MASK  (0x00000003U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER5_PRU1_SD_CLK_SEL5_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER5_PRU1_SD_CLK_SEL5_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER5_PRU1_SD_CLK_SEL5_MAX  (0x00000003U)

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER5_PRU1_SD_CLK_INV5_MASK  (0x00000004U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER5_PRU1_SD_CLK_INV5_SHIFT  (2U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER5_PRU1_SD_CLK_INV5_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER5_PRU1_SD_CLK_INV5_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER5_PRU1_SD_ACC2_SEL5_MASK  (0x00000010U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER5_PRU1_SD_ACC2_SEL5_SHIFT  (4U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER5_PRU1_SD_ACC2_SEL5_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER5_PRU1_SD_ACC2_SEL5_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER5_RESETVAL              (0x00000000U)

/* SDPRU1SAMPLESIZEREGISTER5 */

#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER5_PRU1_SD_SAMPLE_SIZE5_MASK  (0x000000FFU)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER5_PRU1_SD_SAMPLE_SIZE5_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER5_PRU1_SD_SAMPLE_SIZE5_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER5_PRU1_SD_SAMPLE_SIZE5_MAX  (0x000000ffU)

#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER5_RESETVAL          (0x00000000U)

/* SDPRU1CLKSELREGISTER6 */

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER6_PRU1_SD_CLK_SEL6_MASK  (0x00000003U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER6_PRU1_SD_CLK_SEL6_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER6_PRU1_SD_CLK_SEL6_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER6_PRU1_SD_CLK_SEL6_MAX  (0x00000003U)

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER6_PRU1_SD_CLK_INV6_MASK  (0x00000004U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER6_PRU1_SD_CLK_INV6_SHIFT  (2U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER6_PRU1_SD_CLK_INV6_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER6_PRU1_SD_CLK_INV6_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER6_PRU1_SD_ACC2_SEL6_MASK  (0x00000010U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER6_PRU1_SD_ACC2_SEL6_SHIFT  (4U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER6_PRU1_SD_ACC2_SEL6_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER6_PRU1_SD_ACC2_SEL6_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER6_RESETVAL              (0x00000000U)

/* SDPRU1SAMPLESIZEREGISTER6 */

#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER6_PRU1_SD_SAMPLE_SIZE6_MASK  (0x000000FFU)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER6_PRU1_SD_SAMPLE_SIZE6_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER6_PRU1_SD_SAMPLE_SIZE6_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER6_PRU1_SD_SAMPLE_SIZE6_MAX  (0x000000ffU)

#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER6_RESETVAL          (0x00000000U)

/* SDPRU1CLKSELREGISTER7 */

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER7_PRU1_SD_CLK_SEL7_MASK  (0x00000003U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER7_PRU1_SD_CLK_SEL7_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER7_PRU1_SD_CLK_SEL7_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER7_PRU1_SD_CLK_SEL7_MAX  (0x00000003U)

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER7_PRU1_SD_CLK_INV7_MASK  (0x00000004U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER7_PRU1_SD_CLK_INV7_SHIFT  (2U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER7_PRU1_SD_CLK_INV7_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER7_PRU1_SD_CLK_INV7_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER7_PRU1_SD_ACC2_SEL7_MASK  (0x00000010U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER7_PRU1_SD_ACC2_SEL7_SHIFT  (4U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER7_PRU1_SD_ACC2_SEL7_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER7_PRU1_SD_ACC2_SEL7_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER7_RESETVAL              (0x00000000U)

/* SDPRU1SAMPLESIZEREGISTER7 */

#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER7_PRU1_SD_SAMPLE_SIZE7_MASK  (0x000000FFU)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER7_PRU1_SD_SAMPLE_SIZE7_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER7_PRU1_SD_SAMPLE_SIZE7_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER7_PRU1_SD_SAMPLE_SIZE7_MAX  (0x000000ffU)

#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER7_RESETVAL          (0x00000000U)

/* SDPRU1CLKSELREGISTER8 */

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER8_PRU1_SD_CLK_SEL8_MASK  (0x00000003U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER8_PRU1_SD_CLK_SEL8_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER8_PRU1_SD_CLK_SEL8_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER8_PRU1_SD_CLK_SEL8_MAX  (0x00000003U)

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER8_PRU1_SD_CLK_INV8_MASK  (0x00000004U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER8_PRU1_SD_CLK_INV8_SHIFT  (2U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER8_PRU1_SD_CLK_INV8_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER8_PRU1_SD_CLK_INV8_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER8_PRU1_SD_ACC2_SEL8_MASK  (0x00000010U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER8_PRU1_SD_ACC2_SEL8_SHIFT  (4U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER8_PRU1_SD_ACC2_SEL8_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER8_PRU1_SD_ACC2_SEL8_MAX  (0x00000001U)

#define CSL_ICSSCFG_SDPRU1CLKSELREGISTER8_RESETVAL              (0x00000000U)

/* SDPRU1SAMPLESIZEREGISTER8 */

#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER8_PRU1_SD_SAMPLE_SIZE8_MASK  (0x000000FFU)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER8_PRU1_SD_SAMPLE_SIZE8_SHIFT  (0U)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER8_PRU1_SD_SAMPLE_SIZE8_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER8_PRU1_SD_SAMPLE_SIZE8_MAX  (0x000000ffU)

#define CSL_ICSSCFG_SDPRU1SAMPLESIZEREGISTER8_RESETVAL          (0x00000000U)

/* EDPRU0RXCFGREGISTER */

#define CSL_ICSSCFG_EDPRU0RXCFGREGISTER_PRU0_ED_RX_SAMPLE_SIZE_MASK  (0x00000007U)
#define CSL_ICSSCFG_EDPRU0RXCFGREGISTER_PRU0_ED_RX_SAMPLE_SIZE_SHIFT  (0U)
#define CSL_ICSSCFG_EDPRU0RXCFGREGISTER_PRU0_ED_RX_SAMPLE_SIZE_RESETVAL  (0x00000007U)
#define CSL_ICSSCFG_EDPRU0RXCFGREGISTER_PRU0_ED_RX_SAMPLE_SIZE_MAX  (0x00000007U)

#define CSL_ICSSCFG_EDPRU0RXCFGREGISTER_PRU0_ED_RX_CLK_SEL_MASK  (0x00000010U)
#define CSL_ICSSCFG_EDPRU0RXCFGREGISTER_PRU0_ED_RX_CLK_SEL_SHIFT  (4U)
#define CSL_ICSSCFG_EDPRU0RXCFGREGISTER_PRU0_ED_RX_CLK_SEL_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0RXCFGREGISTER_PRU0_ED_RX_CLK_SEL_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU0RXCFGREGISTER_PRU0_ED_RX_DIV_FACTOR_FRAC_MASK  (0x00008000U)
#define CSL_ICSSCFG_EDPRU0RXCFGREGISTER_PRU0_ED_RX_DIV_FACTOR_FRAC_SHIFT  (15U)
#define CSL_ICSSCFG_EDPRU0RXCFGREGISTER_PRU0_ED_RX_DIV_FACTOR_FRAC_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0RXCFGREGISTER_PRU0_ED_RX_DIV_FACTOR_FRAC_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU0RXCFGREGISTER_PRU0_ED_RX_DIV_FACTOR_MASK  (0xFFFF0000U)
#define CSL_ICSSCFG_EDPRU0RXCFGREGISTER_PRU0_ED_RX_DIV_FACTOR_SHIFT  (16U)
#define CSL_ICSSCFG_EDPRU0RXCFGREGISTER_PRU0_ED_RX_DIV_FACTOR_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0RXCFGREGISTER_PRU0_ED_RX_DIV_FACTOR_MAX  (0x0000ffffU)

#define CSL_ICSSCFG_EDPRU0RXCFGREGISTER_RESETVAL                (0x00000007U)

/* EDPRU0TXCFGREGISTER */

#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ED_TX_CLK_SEL_MASK  (0x00000010U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ED_TX_CLK_SEL_SHIFT  (4U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ED_TX_CLK_SEL_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ED_TX_CLK_SEL_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ED_BUSY_0_MASK     (0x00000020U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ED_BUSY_0_SHIFT    (5U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ED_BUSY_0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ED_BUSY_0_MAX      (0x00000001U)

#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ED_BUSY_1_MASK     (0x00000040U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ED_BUSY_1_SHIFT    (6U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ED_BUSY_1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ED_BUSY_1_MAX      (0x00000001U)

#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ED_BUSY_2_MASK     (0x00000080U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ED_BUSY_2_SHIFT    (7U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ED_BUSY_2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ED_BUSY_2_MAX      (0x00000001U)

#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ENDAT0_CLK_SYNC_MASK  (0x00000100U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ENDAT0_CLK_SYNC_SHIFT  (8U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ENDAT0_CLK_SYNC_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ENDAT0_CLK_SYNC_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ENDAT1_CLK_SYNC_MASK  (0x00000200U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ENDAT1_CLK_SYNC_SHIFT  (9U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ENDAT1_CLK_SYNC_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ENDAT1_CLK_SYNC_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ENDAT2_CLK_SYNC_MASK  (0x00000400U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ENDAT2_CLK_SYNC_SHIFT  (10U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ENDAT2_CLK_SYNC_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ENDAT2_CLK_SYNC_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ED_TX_DIV_FACTOR_FRAC_MASK  (0x00008000U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ED_TX_DIV_FACTOR_FRAC_SHIFT  (15U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ED_TX_DIV_FACTOR_FRAC_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ED_TX_DIV_FACTOR_FRAC_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ED_TX_DIV_FACTOR_MASK  (0xFFFF0000U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ED_TX_DIV_FACTOR_SHIFT  (16U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ED_TX_DIV_FACTOR_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_PRU0_ED_TX_DIV_FACTOR_MAX  (0x0000ffffU)

#define CSL_ICSSCFG_EDPRU0TXCFGREGISTER_RESETVAL                (0x00000000U)

/* EDPRU0CH0CFG0REGISTER */

#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_TX_WDLY0_MASK  (0x000007FFU)
#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_TX_WDLY0_SHIFT  (0U)
#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_TX_WDLY0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_TX_WDLY0_MAX  (0x000007ffU)

#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_TX_FRAME_SIZE0_MASK  (0x0000F800U)
#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_TX_FRAME_SIZE0_SHIFT  (11U)
#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_TX_FRAME_SIZE0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_TX_FRAME_SIZE0_MAX  (0x0000001fU)

#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_RX_FRAME_SIZE0_MASK  (0x0FFF0000U)
#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_RX_FRAME_SIZE0_SHIFT  (16U)
#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_RX_FRAME_SIZE0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_RX_FRAME_SIZE0_MAX  (0x00000fffU)

#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_RX_SNOOP0_MASK  (0x10000000U)
#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_RX_SNOOP0_SHIFT  (28U)
#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_RX_SNOOP0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_RX_SNOOP0_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_CLK_OUT_OVR_EN0_MASK  (0x20000000U)
#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_CLK_OUT_OVR_EN0_SHIFT  (29U)
#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_CLK_OUT_OVR_EN0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_CLK_OUT_OVR_EN0_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_SW_CLK_OUT0_MASK  (0x40000000U)
#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_SW_CLK_OUT0_SHIFT  (30U)
#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_SW_CLK_OUT0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_SW_CLK_OUT0_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_TX_FIFO_SWAP_BITS0_MASK  (0x80000000U)
#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_TX_FIFO_SWAP_BITS0_SHIFT  (31U)
#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_TX_FIFO_SWAP_BITS0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_PRU0_ED_TX_FIFO_SWAP_BITS0_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU0CH0CFG0REGISTER_RESETVAL              (0x00000000U)

/* EDPRU0CH0CFG1REGISTER */

#define CSL_ICSSCFG_EDPRU0CH0CFG1REGISTER_PRU0_ED_TST_DELAY_COUNTER0_MASK  (0x0000FFFFU)
#define CSL_ICSSCFG_EDPRU0CH0CFG1REGISTER_PRU0_ED_TST_DELAY_COUNTER0_SHIFT  (0U)
#define CSL_ICSSCFG_EDPRU0CH0CFG1REGISTER_PRU0_ED_TST_DELAY_COUNTER0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0CH0CFG1REGISTER_PRU0_ED_TST_DELAY_COUNTER0_MAX  (0x0000ffffU)

#define CSL_ICSSCFG_EDPRU0CH0CFG1REGISTER_PRU0_ED_RX_EN_COUNTER0_MASK  (0xFFFF0000U)
#define CSL_ICSSCFG_EDPRU0CH0CFG1REGISTER_PRU0_ED_RX_EN_COUNTER0_SHIFT  (16U)
#define CSL_ICSSCFG_EDPRU0CH0CFG1REGISTER_PRU0_ED_RX_EN_COUNTER0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0CH0CFG1REGISTER_PRU0_ED_RX_EN_COUNTER0_MAX  (0x0000ffffU)

#define CSL_ICSSCFG_EDPRU0CH0CFG1REGISTER_RESETVAL              (0x00000000U)

/* EDPRU0CH1CFG0REGISTER */

#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_TX_WDLY1_MASK  (0x000007FFU)
#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_TX_WDLY1_SHIFT  (0U)
#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_TX_WDLY1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_TX_WDLY1_MAX  (0x000007ffU)

#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_TX_FRAME_SIZE1_MASK  (0x0000F800U)
#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_TX_FRAME_SIZE1_SHIFT  (11U)
#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_TX_FRAME_SIZE1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_TX_FRAME_SIZE1_MAX  (0x0000001fU)

#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_RX_FRAME_SIZE1_MASK  (0x0FFF0000U)
#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_RX_FRAME_SIZE1_SHIFT  (16U)
#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_RX_FRAME_SIZE1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_RX_FRAME_SIZE1_MAX  (0x00000fffU)

#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_RX_SNOOP1_MASK  (0x10000000U)
#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_RX_SNOOP1_SHIFT  (28U)
#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_RX_SNOOP1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_RX_SNOOP1_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_CLK_OUT_OVR_EN1_MASK  (0x20000000U)
#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_CLK_OUT_OVR_EN1_SHIFT  (29U)
#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_CLK_OUT_OVR_EN1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_CLK_OUT_OVR_EN1_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_SW_CLK_OUT1_MASK  (0x40000000U)
#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_SW_CLK_OUT1_SHIFT  (30U)
#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_SW_CLK_OUT1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_SW_CLK_OUT1_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_TX_FIFO_SWAP_BITS1_MASK  (0x80000000U)
#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_TX_FIFO_SWAP_BITS1_SHIFT  (31U)
#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_TX_FIFO_SWAP_BITS1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_PRU0_ED_TX_FIFO_SWAP_BITS1_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU0CH1CFG0REGISTER_RESETVAL              (0x00000000U)

/* EDPRU0CH1CFG1REGISTER */

#define CSL_ICSSCFG_EDPRU0CH1CFG1REGISTER_PRU0_ED_TST_DELAY_COUNTER1_MASK  (0x0000FFFFU)
#define CSL_ICSSCFG_EDPRU0CH1CFG1REGISTER_PRU0_ED_TST_DELAY_COUNTER1_SHIFT  (0U)
#define CSL_ICSSCFG_EDPRU0CH1CFG1REGISTER_PRU0_ED_TST_DELAY_COUNTER1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0CH1CFG1REGISTER_PRU0_ED_TST_DELAY_COUNTER1_MAX  (0x0000ffffU)

#define CSL_ICSSCFG_EDPRU0CH1CFG1REGISTER_PRU0_ED_RX_EN_COUNTER1_MASK  (0xFFFF0000U)
#define CSL_ICSSCFG_EDPRU0CH1CFG1REGISTER_PRU0_ED_RX_EN_COUNTER1_SHIFT  (16U)
#define CSL_ICSSCFG_EDPRU0CH1CFG1REGISTER_PRU0_ED_RX_EN_COUNTER1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0CH1CFG1REGISTER_PRU0_ED_RX_EN_COUNTER1_MAX  (0x0000ffffU)

#define CSL_ICSSCFG_EDPRU0CH1CFG1REGISTER_RESETVAL              (0x00000000U)

/* EDPRU0CH2CFG0REGISTER */

#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_TX_WDLY2_MASK  (0x000007FFU)
#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_TX_WDLY2_SHIFT  (0U)
#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_TX_WDLY2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_TX_WDLY2_MAX  (0x000007ffU)

#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_TX_FRAME_SIZE2_MASK  (0x0000F800U)
#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_TX_FRAME_SIZE2_SHIFT  (11U)
#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_TX_FRAME_SIZE2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_TX_FRAME_SIZE2_MAX  (0x0000001fU)

#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_RX_FRAME_SIZE2_MASK  (0x0FFF0000U)
#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_RX_FRAME_SIZE2_SHIFT  (16U)
#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_RX_FRAME_SIZE2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_RX_FRAME_SIZE2_MAX  (0x00000fffU)

#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_RX_SNOOP2_MASK  (0x10000000U)
#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_RX_SNOOP2_SHIFT  (28U)
#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_RX_SNOOP2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_RX_SNOOP2_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_CLK_OUT_OVR_EN2_MASK  (0x20000000U)
#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_CLK_OUT_OVR_EN2_SHIFT  (29U)
#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_CLK_OUT_OVR_EN2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_CLK_OUT_OVR_EN2_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_SW_CLK_OUT2_MASK  (0x40000000U)
#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_SW_CLK_OUT2_SHIFT  (30U)
#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_SW_CLK_OUT2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_SW_CLK_OUT2_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_TX_FIFO_SWAP_BITS2_MASK  (0x80000000U)
#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_TX_FIFO_SWAP_BITS2_SHIFT  (31U)
#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_TX_FIFO_SWAP_BITS2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_PRU0_ED_TX_FIFO_SWAP_BITS2_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU0CH2CFG0REGISTER_RESETVAL              (0x00000000U)

/* EDPRU0CH2CFG1REGISTER */

#define CSL_ICSSCFG_EDPRU0CH2CFG1REGISTER_PRU0_ED_TST_DELAY_COUNTER2_MASK  (0x0000FFFFU)
#define CSL_ICSSCFG_EDPRU0CH2CFG1REGISTER_PRU0_ED_TST_DELAY_COUNTER2_SHIFT  (0U)
#define CSL_ICSSCFG_EDPRU0CH2CFG1REGISTER_PRU0_ED_TST_DELAY_COUNTER2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0CH2CFG1REGISTER_PRU0_ED_TST_DELAY_COUNTER2_MAX  (0x0000ffffU)

#define CSL_ICSSCFG_EDPRU0CH2CFG1REGISTER_PRU0_ED_RX_EN_COUNTER2_MASK  (0xFFFF0000U)
#define CSL_ICSSCFG_EDPRU0CH2CFG1REGISTER_PRU0_ED_RX_EN_COUNTER2_SHIFT  (16U)
#define CSL_ICSSCFG_EDPRU0CH2CFG1REGISTER_PRU0_ED_RX_EN_COUNTER2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU0CH2CFG1REGISTER_PRU0_ED_RX_EN_COUNTER2_MAX  (0x0000ffffU)

#define CSL_ICSSCFG_EDPRU0CH2CFG1REGISTER_RESETVAL              (0x00000000U)

/* EDPRU1RXCFGREGISTER */

#define CSL_ICSSCFG_EDPRU1RXCFGREGISTER_PRU1_ED_RX_SAMPLE_SIZE_MASK  (0x00000007U)
#define CSL_ICSSCFG_EDPRU1RXCFGREGISTER_PRU1_ED_RX_SAMPLE_SIZE_SHIFT  (0U)
#define CSL_ICSSCFG_EDPRU1RXCFGREGISTER_PRU1_ED_RX_SAMPLE_SIZE_RESETVAL  (0x00000007U)
#define CSL_ICSSCFG_EDPRU1RXCFGREGISTER_PRU1_ED_RX_SAMPLE_SIZE_MAX  (0x00000007U)

#define CSL_ICSSCFG_EDPRU1RXCFGREGISTER_PRU1_ED_RX_CLK_SEL_MASK  (0x00000010U)
#define CSL_ICSSCFG_EDPRU1RXCFGREGISTER_PRU1_ED_RX_CLK_SEL_SHIFT  (4U)
#define CSL_ICSSCFG_EDPRU1RXCFGREGISTER_PRU1_ED_RX_CLK_SEL_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1RXCFGREGISTER_PRU1_ED_RX_CLK_SEL_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU1RXCFGREGISTER_PRU1_ED_RX_DIV_FACTOR_FRAC_MASK  (0x00008000U)
#define CSL_ICSSCFG_EDPRU1RXCFGREGISTER_PRU1_ED_RX_DIV_FACTOR_FRAC_SHIFT  (15U)
#define CSL_ICSSCFG_EDPRU1RXCFGREGISTER_PRU1_ED_RX_DIV_FACTOR_FRAC_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1RXCFGREGISTER_PRU1_ED_RX_DIV_FACTOR_FRAC_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU1RXCFGREGISTER_PRU1_ED_RX_DIV_FACTOR_MASK  (0xFFFF0000U)
#define CSL_ICSSCFG_EDPRU1RXCFGREGISTER_PRU1_ED_RX_DIV_FACTOR_SHIFT  (16U)
#define CSL_ICSSCFG_EDPRU1RXCFGREGISTER_PRU1_ED_RX_DIV_FACTOR_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1RXCFGREGISTER_PRU1_ED_RX_DIV_FACTOR_MAX  (0x0000ffffU)

#define CSL_ICSSCFG_EDPRU1RXCFGREGISTER_RESETVAL                (0x00000007U)

/* EDPRU1TXCFGREGISTER */

#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ED_TX_CLK_SEL_MASK  (0x00000010U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ED_TX_CLK_SEL_SHIFT  (4U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ED_TX_CLK_SEL_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ED_TX_CLK_SEL_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ED_BUSY_0_MASK     (0x00000020U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ED_BUSY_0_SHIFT    (5U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ED_BUSY_0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ED_BUSY_0_MAX      (0x00000001U)

#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ED_BUSY_1_MASK     (0x00000040U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ED_BUSY_1_SHIFT    (6U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ED_BUSY_1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ED_BUSY_1_MAX      (0x00000001U)

#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ED_BUSY_2_MASK     (0x00000080U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ED_BUSY_2_SHIFT    (7U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ED_BUSY_2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ED_BUSY_2_MAX      (0x00000001U)

#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ENDAT0_CLK_SYNC_MASK  (0x00000100U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ENDAT0_CLK_SYNC_SHIFT  (8U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ENDAT0_CLK_SYNC_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ENDAT0_CLK_SYNC_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ENDAT1_CLK_SYNC_MASK  (0x00000200U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ENDAT1_CLK_SYNC_SHIFT  (9U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ENDAT1_CLK_SYNC_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ENDAT1_CLK_SYNC_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ENDAT2_CLK_SYNC_MASK  (0x00000400U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ENDAT2_CLK_SYNC_SHIFT  (10U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ENDAT2_CLK_SYNC_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ENDAT2_CLK_SYNC_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ED_TX_DIV_FACTOR_FRAC_MASK  (0x00008000U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ED_TX_DIV_FACTOR_FRAC_SHIFT  (15U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ED_TX_DIV_FACTOR_FRAC_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ED_TX_DIV_FACTOR_FRAC_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ED_TX_DIV_FACTOR_MASK  (0xFFFF0000U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ED_TX_DIV_FACTOR_SHIFT  (16U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ED_TX_DIV_FACTOR_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_PRU1_ED_TX_DIV_FACTOR_MAX  (0x0000ffffU)

#define CSL_ICSSCFG_EDPRU1TXCFGREGISTER_RESETVAL                (0x00000000U)

/* EDPRU1CH0CFG0REGISTER */

#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_TX_WDLY0_MASK  (0x000007FFU)
#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_TX_WDLY0_SHIFT  (0U)
#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_TX_WDLY0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_TX_WDLY0_MAX  (0x000007ffU)

#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_TX_FRAME_SIZE0_MASK  (0x0000F800U)
#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_TX_FRAME_SIZE0_SHIFT  (11U)
#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_TX_FRAME_SIZE0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_TX_FRAME_SIZE0_MAX  (0x0000001fU)

#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_RX_FRAME_SIZE0_MASK  (0x0FFF0000U)
#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_RX_FRAME_SIZE0_SHIFT  (16U)
#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_RX_FRAME_SIZE0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_RX_FRAME_SIZE0_MAX  (0x00000fffU)

#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_RX_SNOOP0_MASK  (0x10000000U)
#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_RX_SNOOP0_SHIFT  (28U)
#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_RX_SNOOP0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_RX_SNOOP0_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_CLK_OUT_OVR_EN0_MASK  (0x20000000U)
#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_CLK_OUT_OVR_EN0_SHIFT  (29U)
#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_CLK_OUT_OVR_EN0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_CLK_OUT_OVR_EN0_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_SW_CLK_OUT0_MASK  (0x40000000U)
#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_SW_CLK_OUT0_SHIFT  (30U)
#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_SW_CLK_OUT0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_SW_CLK_OUT0_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_TX_FIFO_SWAP_BITS0_MASK  (0x80000000U)
#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_TX_FIFO_SWAP_BITS0_SHIFT  (31U)
#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_TX_FIFO_SWAP_BITS0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_PRU1_ED_TX_FIFO_SWAP_BITS0_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU1CH0CFG0REGISTER_RESETVAL              (0x00000000U)

/* EDPRU1CH0CFG1REGISTER */

#define CSL_ICSSCFG_EDPRU1CH0CFG1REGISTER_PRU1_ED_TST_DELAY_COUNTER0_MASK  (0x0000FFFFU)
#define CSL_ICSSCFG_EDPRU1CH0CFG1REGISTER_PRU1_ED_TST_DELAY_COUNTER0_SHIFT  (0U)
#define CSL_ICSSCFG_EDPRU1CH0CFG1REGISTER_PRU1_ED_TST_DELAY_COUNTER0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1CH0CFG1REGISTER_PRU1_ED_TST_DELAY_COUNTER0_MAX  (0x0000ffffU)

#define CSL_ICSSCFG_EDPRU1CH0CFG1REGISTER_PRU1_ED_RX_EN_COUNTER0_MASK  (0xFFFF0000U)
#define CSL_ICSSCFG_EDPRU1CH0CFG1REGISTER_PRU1_ED_RX_EN_COUNTER0_SHIFT  (16U)
#define CSL_ICSSCFG_EDPRU1CH0CFG1REGISTER_PRU1_ED_RX_EN_COUNTER0_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1CH0CFG1REGISTER_PRU1_ED_RX_EN_COUNTER0_MAX  (0x0000ffffU)

#define CSL_ICSSCFG_EDPRU1CH0CFG1REGISTER_RESETVAL              (0x00000000U)

/* EDPRU1CH1CFG0REGISTER */

#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_TX_WDLY1_MASK  (0x000007FFU)
#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_TX_WDLY1_SHIFT  (0U)
#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_TX_WDLY1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_TX_WDLY1_MAX  (0x000007ffU)

#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_TX_FRAME_SIZE1_MASK  (0x0000F800U)
#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_TX_FRAME_SIZE1_SHIFT  (11U)
#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_TX_FRAME_SIZE1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_TX_FRAME_SIZE1_MAX  (0x0000001fU)

#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_RX_FRAME_SIZE1_MASK  (0x0FFF0000U)
#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_RX_FRAME_SIZE1_SHIFT  (16U)
#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_RX_FRAME_SIZE1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_RX_FRAME_SIZE1_MAX  (0x00000fffU)

#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_RX_SNOOP1_MASK  (0x10000000U)
#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_RX_SNOOP1_SHIFT  (28U)
#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_RX_SNOOP1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_RX_SNOOP1_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_CLK_OUT_OVR_EN1_MASK  (0x20000000U)
#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_CLK_OUT_OVR_EN1_SHIFT  (29U)
#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_CLK_OUT_OVR_EN1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_CLK_OUT_OVR_EN1_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_SW_CLK_OUT1_MASK  (0x40000000U)
#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_SW_CLK_OUT1_SHIFT  (30U)
#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_SW_CLK_OUT1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_SW_CLK_OUT1_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_TX_FIFO_SWAP_BITS1_MASK  (0x80000000U)
#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_TX_FIFO_SWAP_BITS1_SHIFT  (31U)
#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_TX_FIFO_SWAP_BITS1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_PRU1_ED_TX_FIFO_SWAP_BITS1_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU1CH1CFG0REGISTER_RESETVAL              (0x00000000U)

/* EDPRU1CH1CFG1REGISTER */

#define CSL_ICSSCFG_EDPRU1CH1CFG1REGISTER_PRU1_ED_TST_DELAY_COUNTER1_MASK  (0x0000FFFFU)
#define CSL_ICSSCFG_EDPRU1CH1CFG1REGISTER_PRU1_ED_TST_DELAY_COUNTER1_SHIFT  (0U)
#define CSL_ICSSCFG_EDPRU1CH1CFG1REGISTER_PRU1_ED_TST_DELAY_COUNTER1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1CH1CFG1REGISTER_PRU1_ED_TST_DELAY_COUNTER1_MAX  (0x0000ffffU)

#define CSL_ICSSCFG_EDPRU1CH1CFG1REGISTER_PRU1_ED_RX_EN_COUNTER1_MASK  (0xFFFF0000U)
#define CSL_ICSSCFG_EDPRU1CH1CFG1REGISTER_PRU1_ED_RX_EN_COUNTER1_SHIFT  (16U)
#define CSL_ICSSCFG_EDPRU1CH1CFG1REGISTER_PRU1_ED_RX_EN_COUNTER1_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1CH1CFG1REGISTER_PRU1_ED_RX_EN_COUNTER1_MAX  (0x0000ffffU)

#define CSL_ICSSCFG_EDPRU1CH1CFG1REGISTER_RESETVAL              (0x00000000U)

/* EDPRU1CH2CFG0REGISTER */

#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_TX_WDLY2_MASK  (0x000007FFU)
#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_TX_WDLY2_SHIFT  (0U)
#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_TX_WDLY2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_TX_WDLY2_MAX  (0x000007ffU)

#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_TX_FRAME_SIZE2_MASK  (0x0000F800U)
#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_TX_FRAME_SIZE2_SHIFT  (11U)
#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_TX_FRAME_SIZE2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_TX_FRAME_SIZE2_MAX  (0x0000001fU)

#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_RX_FRAME_SIZE2_MASK  (0x0FFF0000U)
#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_RX_FRAME_SIZE2_SHIFT  (16U)
#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_RX_FRAME_SIZE2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_RX_FRAME_SIZE2_MAX  (0x00000fffU)

#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_RX_SNOOP2_MASK  (0x10000000U)
#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_RX_SNOOP2_SHIFT  (28U)
#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_RX_SNOOP2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_RX_SNOOP2_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_CLK_OUT_OVR_EN2_MASK  (0x20000000U)
#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_CLK_OUT_OVR_EN2_SHIFT  (29U)
#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_CLK_OUT_OVR_EN2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_CLK_OUT_OVR_EN2_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_SW_CLK_OUT2_MASK  (0x40000000U)
#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_SW_CLK_OUT2_SHIFT  (30U)
#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_SW_CLK_OUT2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_SW_CLK_OUT2_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_TX_FIFO_SWAP_BITS2_MASK  (0x80000000U)
#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_TX_FIFO_SWAP_BITS2_SHIFT  (31U)
#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_TX_FIFO_SWAP_BITS2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_PRU1_ED_TX_FIFO_SWAP_BITS2_MAX  (0x00000001U)

#define CSL_ICSSCFG_EDPRU1CH2CFG0REGISTER_RESETVAL              (0x00000000U)

/* EDPRU1CH2CFG1REGISTER */

#define CSL_ICSSCFG_EDPRU1CH2CFG1REGISTER_PRU1_ED_TST_DELAY_COUNTER2_MASK  (0x0000FFFFU)
#define CSL_ICSSCFG_EDPRU1CH2CFG1REGISTER_PRU1_ED_TST_DELAY_COUNTER2_SHIFT  (0U)
#define CSL_ICSSCFG_EDPRU1CH2CFG1REGISTER_PRU1_ED_TST_DELAY_COUNTER2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1CH2CFG1REGISTER_PRU1_ED_TST_DELAY_COUNTER2_MAX  (0x0000ffffU)

#define CSL_ICSSCFG_EDPRU1CH2CFG1REGISTER_PRU1_ED_RX_EN_COUNTER2_MASK  (0xFFFF0000U)
#define CSL_ICSSCFG_EDPRU1CH2CFG1REGISTER_PRU1_ED_RX_EN_COUNTER2_SHIFT  (16U)
#define CSL_ICSSCFG_EDPRU1CH2CFG1REGISTER_PRU1_ED_RX_EN_COUNTER2_RESETVAL  (0x00000000U)
#define CSL_ICSSCFG_EDPRU1CH2CFG1REGISTER_PRU1_ED_RX_EN_COUNTER2_MAX  (0x0000ffffU)

#define CSL_ICSSCFG_EDPRU1CH2CFG1REGISTER_RESETVAL              (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
