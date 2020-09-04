/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

#ifndef CSLR_MCASP_H
#define CSLR_MCASP_H

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************\
* AFIFO Data-IN Register Overlay Structure (buffer block)
\**************************************************************************/
typedef struct  {
    volatile uint32_t DATA_PORT;
    volatile uint32_t RSVD_04;
    volatile uint32_t RSVD_08;
    volatile uint32_t RSVD_0C;
} CSL_AdataRegs;

/**************************************************************************\
* Overlay structure typedef definition
\**************************************************************************/
typedef volatile CSL_AdataRegs         *CSL_AdataRegsOvly;

/**************************************************************************\
* AFIFO MMR Register Overlay Structure (buffer block)
\**************************************************************************/
typedef struct  {
    volatile uint32_t REVID;
    volatile uint32_t RSVD_04;
    volatile uint32_t RSVD_08;
    volatile uint32_t RSVD_0C;
    volatile uint32_t WFIFOCTL;
    volatile uint32_t WFIFOSTS;
    volatile uint32_t RFIFOCTL;
    volatile uint32_t RFIFOSTS;
} CSL_AfifoRegs;

/**************************************************************************\
* Overlay structure typedef definition
\**************************************************************************/
typedef volatile CSL_AfifoRegs         *CSL_AfifoRegsOvly;

/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure for ALL
\**************************************************************************/
typedef struct  {
    volatile uint32_t REVID;
    volatile uint32_t PWRIDLESYSCONFIG;
    volatile uint8_t  RSVD0[8];
    volatile uint32_t PFUNC;
    volatile uint32_t PDIR;
    volatile uint32_t PDOUT;
    volatile uint32_t PDIN;
    volatile uint32_t PDCLR;
    volatile uint8_t  RSVD1[32];
    volatile uint32_t GBLCTL;
    volatile uint32_t AMUTE;
    volatile uint32_t DLBCTL;
    volatile uint32_t DITCTL;
    volatile uint8_t  RSVD2[12];
    volatile uint32_t RGBLCTL;
    volatile uint32_t RMASK;
    volatile uint32_t RFMT;
    volatile uint32_t AFSRCTL;
    volatile uint32_t ACLKRCTL;
    volatile uint32_t AHCLKRCTL;
    volatile uint32_t RTDM;
    volatile uint32_t RINTCTL;
    volatile uint32_t RSTAT;
    volatile uint32_t RSLOT;
    volatile uint32_t RCLKCHK;
    volatile uint32_t REVTCTL;
    volatile uint8_t  RSVD3[16];
    volatile uint32_t XGBLCTL;
    volatile uint32_t XMASK;
    volatile uint32_t XFMT;
    volatile uint32_t AFSXCTL;
    volatile uint32_t ACLKXCTL;
    volatile uint32_t AHCLKXCTL;
    volatile uint32_t XTDM;
    volatile uint32_t XINTCTL;
    volatile uint32_t XSTAT;
    volatile uint32_t XSLOT;
    volatile uint32_t XCLKCHK;
    volatile uint32_t XEVTCTL;
    volatile uint8_t  RSVD4[48];
    volatile uint32_t DITCSRA0;
    volatile uint32_t DITCSRA1;
    volatile uint32_t DITCSRA2;
    volatile uint32_t DITCSRA3;
    volatile uint32_t DITCSRA4;
    volatile uint32_t DITCSRA5;
    volatile uint32_t DITCSRB0;
    volatile uint32_t DITCSRB1;
    volatile uint32_t DITCSRB2;
    volatile uint32_t DITCSRB3;
    volatile uint32_t DITCSRB4;
    volatile uint32_t DITCSRB5;
    volatile uint32_t DITUDRA0;
    volatile uint32_t DITUDRA1;
    volatile uint32_t DITUDRA2;
    volatile uint32_t DITUDRA3;
    volatile uint32_t DITUDRA4;
    volatile uint32_t DITUDRA5;
    volatile uint32_t DITUDRB0;
    volatile uint32_t DITUDRB1;
    volatile uint32_t DITUDRB2;
    volatile uint32_t DITUDRB3;
    volatile uint32_t DITUDRB4;
    volatile uint32_t DITUDRB5;
    volatile uint8_t  RSVD5[32];
    volatile uint32_t SRCTL0;
    volatile uint32_t SRCTL1;
    volatile uint32_t SRCTL2;
    volatile uint32_t SRCTL3;
    volatile uint32_t SRCTL4;
    volatile uint32_t SRCTL5;
    volatile uint32_t SRCTL6;
    volatile uint32_t SRCTL7;
    volatile uint32_t SRCTL8;
    volatile uint32_t SRCTL9;
    volatile uint32_t SRCTL10;
    volatile uint32_t SRCTL11;
    volatile uint32_t SRCTL12;
    volatile uint32_t SRCTL13;
    volatile uint32_t SRCTL14;
    volatile uint32_t SRCTL15;
    volatile uint8_t  RSVD6[64];
    volatile uint32_t XBUF0;
    volatile uint32_t XBUF1;
    volatile uint32_t XBUF2;
    volatile uint32_t XBUF3;
    volatile uint32_t XBUF4;
    volatile uint32_t XBUF5;
    volatile uint32_t XBUF6;
    volatile uint32_t XBUF7;
    volatile uint32_t XBUF8;
    volatile uint32_t XBUF9;
    volatile uint32_t XBUF10;
    volatile uint32_t XBUF11;
    volatile uint32_t XBUF12;
    volatile uint32_t XBUF13;
    volatile uint32_t XBUF14;
    volatile uint32_t XBUF15;
    volatile uint8_t  RSVD7[64];
    volatile uint32_t RBUF0;
    volatile uint32_t RBUF1;
    volatile uint32_t RBUF2;
    volatile uint32_t RBUF3;
    volatile uint32_t RBUF4;
    volatile uint32_t RBUF5;
    volatile uint32_t RBUF6;
    volatile uint32_t RBUF7;
    volatile uint32_t RBUF8;
    volatile uint32_t RBUF9;
    volatile uint32_t RBUF10;
    volatile uint32_t RBUF11;
    volatile uint32_t RBUF12;
    volatile uint32_t RBUF13;
    volatile uint32_t RBUF14;
    volatile uint32_t RBUF15;
    volatile uint8_t  RSVD8[3392];
    volatile uint32_t WFIFOCTL;
    volatile uint32_t WFIFOSTS;
    volatile uint32_t RFIFOCTL;
    volatile uint32_t RFIFOSTS;
    volatile uint8_t  RSVD9[12304];
} CSL_McaspRegs;

/**************************************************************************\
* Overlay structure typedef definition
\**************************************************************************/
typedef volatile CSL_McaspRegs  *CSL_McaspRegsOvly;
/**************************************************************************
* Register Macros
**************************************************************************/
#define CSL_MCASP_REV                                           ((uint32_t)0x0u)
#define CSL_MCASP_PWRIDLESYSCONFIG                              ((uint32_t)0x4u)
#define CSL_MCASP_PFUNC                                         ((uint32_t)0x10u)
#define CSL_MCASP_PDIR                                          ((uint32_t)0x14u)
#define CSL_MCASP_PDOUT                                         ((uint32_t)0x18u)
#define CSL_MCASP_PDIN                                          ((uint32_t)0x1Cu)
#define CSL_MCASP_PDCLR                                         ((uint32_t)0x20u)
#define CSL_MCASP_GBLCTL                                        ((uint32_t)0x44u)
#define CSL_MCASP_AMUTE                                         ((uint32_t)0x48u)
#define CSL_MCASP_DLBCTL                                        ((uint32_t)0x4Cu)
#define CSL_MCASP_DITCTL                                        ((uint32_t)0x50u)
#define CSL_MCASP_RGBLCTL                                       ((uint32_t)0x60u)
#define CSL_MCASP_RMASK                                         ((uint32_t)0x64u)
#define CSL_MCASP_RFMT                                          ((uint32_t)0x68u)
#define CSL_MCASP_AFSRCTL                                       ((uint32_t)0x6Cu)
#define CSL_MCASP_ACLKRCTL                                      ((uint32_t)0x70u)
#define CSL_MCASP_AHCLKRCTL                                     ((uint32_t)0x74u)
#define CSL_MCASP_RTDM                                          ((uint32_t)0x78u)
#define CSL_MCASP_RINTCTL                                       ((uint32_t)0x7Cu)
#define CSL_MCASP_RSTAT                                         ((uint32_t)0x80u)
#define CSL_MCASP_RSLOT                                         ((uint32_t)0x84u)
#define CSL_MCASP_RCLKCHK                                       ((uint32_t)0x88u)
#define CSL_MCASP_REVTCTL                                       ((uint32_t)0x8Cu)
#define CSL_MCASP_XGBLCTL                                       ((uint32_t)0xA0u)
#define CSL_MCASP_XMASK                                         ((uint32_t)0xA4u)
#define CSL_MCASP_XFMT                                          ((uint32_t)0xA8u)
#define CSL_MCASP_AFSXCTL                                       ((uint32_t)0xACu)
#define CSL_MCASP_ACLKXCTL                                      ((uint32_t)0xB0u)
#define CSL_MCASP_AHCLKXCTL                                     ((uint32_t)0xB4u)
#define CSL_MCASP_XTDM                                          ((uint32_t)0xB8u)
#define CSL_MCASP_XINTCTL                                       ((uint32_t)0xBCu)
#define CSL_MCASP_XSTAT                                         ((uint32_t)0xC0u)
#define CSL_MCASP_XSLOT                                         ((uint32_t)0xC4u)
#define CSL_MCASP_XCLKCHK                                       ((uint32_t)0xC8u)
#define CSL_MCASP_XEVTCTL                                       ((uint32_t)0xCCu)
#define CSL_MCASP_DITCSRA0                                      ((uint32_t)0x100u)
#define CSL_MCASP_DITCSRA1                                      ((uint32_t)0x104u)
#define CSL_MCASP_DITCSRA2                                      ((uint32_t)0x108u)
#define CSL_MCASP_DITCSRA3                                      ((uint32_t)0x10Cu)
#define CSL_MCASP_DITCSRA4                                      ((uint32_t)0x110u)
#define CSL_MCASP_DITCSRA5                                      ((uint32_t)0x114u)
#define CSL_MCASP_DITCSRB0                                      ((uint32_t)0x118u)
#define CSL_MCASP_DITCSRB1                                      ((uint32_t)0x11Cu)
#define CSL_MCASP_DITCSRB2                                      ((uint32_t)0x120u)
#define CSL_MCASP_DITCSRB3                                      ((uint32_t)0x124u)
#define CSL_MCASP_DITCSRB4                                      ((uint32_t)0x128u)
#define CSL_MCASP_DITCSRB5                                      ((uint32_t)0x12Cu)
#define CSL_MCASP_DITUDRA0                                      ((uint32_t)0x130u)
#define CSL_MCASP_DITUDRA1                                      ((uint32_t)0x134u)
#define CSL_MCASP_DITUDRA2                                      ((uint32_t)0x138u)
#define CSL_MCASP_DITUDRA3                                      ((uint32_t)0x13Cu)
#define CSL_MCASP_DITUDRA4                                      ((uint32_t)0x140u)
#define CSL_MCASP_DITUDRA5                                      ((uint32_t)0x144u)
#define CSL_MCASP_DITUDRB0                                      ((uint32_t)0x148u)
#define CSL_MCASP_DITUDRB1                                      ((uint32_t)0x14Cu)
#define CSL_MCASP_DITUDRB2                                      ((uint32_t)0x150u)
#define CSL_MCASP_DITUDRB3                                      ((uint32_t)0x154u)
#define CSL_MCASP_DITUDRB4                                      ((uint32_t)0x158u)
#define CSL_MCASP_DITUDRB5                                      ((uint32_t)0x15Cu)
#define CSL_MCASP_SRCTL0                                        ((uint32_t)0x180u)
#define CSL_MCASP_SRCTL1                                        ((uint32_t)0x184u)
#define CSL_MCASP_SRCTL2                                        ((uint32_t)0x188u)
#define CSL_MCASP_SRCTL3                                        ((uint32_t)0x18Cu)
#define CSL_MCASP_SRCTL4                                        ((uint32_t)0x190u)
#define CSL_MCASP_SRCTL5                                        ((uint32_t)0x194u)
#define CSL_MCASP_SRCTL6                                        ((uint32_t)0x198u)
#define CSL_MCASP_SRCTL7                                        ((uint32_t)0x19Cu)
#define CSL_MCASP_SRCTL8                                        ((uint32_t)0x1A0u)
#define CSL_MCASP_SRCTL9                                        ((uint32_t)0x1A4u)
#define CSL_MCASP_SRCTL10                                       ((uint32_t)0x1A8u)
#define CSL_MCASP_SRCTL11                                       ((uint32_t)0x1ACu)
#define CSL_MCASP_SRCTL12                                       ((uint32_t)0x1B0u)
#define CSL_MCASP_SRCTL13                                       ((uint32_t)0x1B4u)
#define CSL_MCASP_SRCTL14                                       ((uint32_t)0x1B8u)
#define CSL_MCASP_SRCTL15                                       ((uint32_t)0x1BCu)
#define CSL_MCASP_XBUF0                                         ((uint32_t)0x200u)
#define CSL_MCASP_XBUF1                                         ((uint32_t)0x204u)
#define CSL_MCASP_XBUF2                                         ((uint32_t)0x208u)
#define CSL_MCASP_XBUF3                                         ((uint32_t)0x20Cu)
#define CSL_MCASP_XBUF4                                         ((uint32_t)0x210u)
#define CSL_MCASP_XBUF5                                         ((uint32_t)0x214u)
#define CSL_MCASP_XBUF6                                         ((uint32_t)0x218u)
#define CSL_MCASP_XBUF7                                         ((uint32_t)0x21Cu)
#define CSL_MCASP_XBUF8                                         ((uint32_t)0x220u)
#define CSL_MCASP_XBUF9                                         ((uint32_t)0x224u)
#define CSL_MCASP_XBUF10                                        ((uint32_t)0x228u)
#define CSL_MCASP_XBUF11                                        ((uint32_t)0x22Cu)
#define CSL_MCASP_XBUF12                                        ((uint32_t)0x230u)
#define CSL_MCASP_XBUF13                                        ((uint32_t)0x234u)
#define CSL_MCASP_XBUF14                                        ((uint32_t)0x238u)
#define CSL_MCASP_XBUF15                                        ((uint32_t)0x23Cu)
#define CSL_MCASP_RBUF0                                         ((uint32_t)0x280u)
#define CSL_MCASP_RBUF1                                         ((uint32_t)0x284u)
#define CSL_MCASP_RBUF2                                         ((uint32_t)0x288u)
#define CSL_MCASP_RBUF3                                         ((uint32_t)0x28Cu)
#define CSL_MCASP_RBUF4                                         ((uint32_t)0x290u)
#define CSL_MCASP_RBUF5                                         ((uint32_t)0x294u)
#define CSL_MCASP_RBUF6                                         ((uint32_t)0x298u)
#define CSL_MCASP_RBUF7                                         ((uint32_t)0x29Cu)
#define CSL_MCASP_RBUF8                                         ((uint32_t)0x2A0u)
#define CSL_MCASP_RBUF9                                         ((uint32_t)0x2A4u)
#define CSL_MCASP_RBUF10                                        ((uint32_t)0x2A8u)
#define CSL_MCASP_RBUF11                                        ((uint32_t)0x2ACu)
#define CSL_MCASP_RBUF12                                        ((uint32_t)0x2B0u)
#define CSL_MCASP_RBUF13                                        ((uint32_t)0x2B4u)
#define CSL_MCASP_RBUF14                                        ((uint32_t)0x2B8u)
#define CSL_MCASP_RBUF15                                        ((uint32_t)0x2BCu)
#define CSL_MCASP_WFIFOCTL                                      ((uint32_t)0x1000u)
#define CSL_MCASP_WFIFOSTS                                      ((uint32_t)0x1004u)
#define CSL_MCASP_RFIFOCTL                                      ((uint32_t)0x1008u)
#define CSL_MCASP_RFIFOSTS                                      ((uint32_t)0x100Cu)


/**************************************************************************
* Field Definition Macros
**************************************************************************/

/* REVID */

#define CSL_MCASP_REVID_REV_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_REVID_REV_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_REVID_REV_RESETVAL ((uint32_t)0x44300A02u)

#define CSL_MCASP_REVID_RESETVAL ((uint32_t)0x44300A02u)

/* PWRIDLESYSCONFIG */

#define CSL_MCASP_PWRIDLESYSCONFIG_IDLEMODE_SHIFT               ((uint32_t)0u)
#define CSL_MCASP_PWRIDLESYSCONFIG_IDLEMODE_MASK                ((uint32_t)0x00000003u)
#define CSL_MCASP_PWRIDLESYSCONFIG_IDLEMODE_RESETVAL            ((uint32_t)0x00000002u)
#define CSL_MCASP_PWRIDLESYSCONFIG_IDLEMODE_FORCEIDLE           ((uint32_t)0x00000000u)
#define CSL_MCASP_PWRIDLESYSCONFIG_IDLEMODE_NOIDLE              ((uint32_t)0x00000001u)
#define CSL_MCASP_PWRIDLESYSCONFIG_IDLEMODE_SMARTIDLE           ((uint32_t)0x00000002u)
#define CSL_MCASP_PWRIDLESYSCONFIG_IDLEMODE_RSVD                ((uint32_t)0x00000003u)

#define CSL_MCASP_PWRIDLESYSCONFIG_OTHER_SHIFT                  ((uint32_t)2u)
#define CSL_MCASP_PWRIDLESYSCONFIG_OTHER_MASK                   ((uint32_t)0x0000003Cu)
#define CSL_MCASP_PWRIDLESYSCONFIG_OTHER_RESETVAL               ((uint32_t)0x00000000u)
#define CSL_MCASP_PWRIDLESYSCONFIG_OTHER_MAX                    ((uint32_t)0x0000000fu)

#define CSL_MCASP_PWRIDLESYSCONFIG_RESETVAL                     ((uint32_t)0x00000002u)

/* PFUNC */

#define CSL_MCASP_PFUNC_AFSR_MASK ((uint32_t)0x80000000u)
#define CSL_MCASP_PFUNC_AFSR_SHIFT ((uint32_t)0x0000001Fu)
#define CSL_MCASP_PFUNC_AFSR_RESETVAL ((uint32_t)0x00000000u)
/*----AFSR Tokens----*/
#define CSL_MCASP_PFUNC_AFSR_MCASP ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AFSR_GPIO ((uint32_t)0x00000001u)

#define CSL_MCASP_PFUNC_AHCLKR_MASK ((uint32_t)0x40000000u)
#define CSL_MCASP_PFUNC_AHCLKR_SHIFT ((uint32_t)0x0000001Eu)
#define CSL_MCASP_PFUNC_AHCLKR_RESETVAL ((uint32_t)0x00000000u)
/*----AHCLKR Tokens----*/
#define CSL_MCASP_PFUNC_AHCLKR_MCASP ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AHCLKR_GPIO ((uint32_t)0x00000001u)

#define CSL_MCASP_PFUNC_ACLKR_MASK ((uint32_t)0x20000000u)
#define CSL_MCASP_PFUNC_ACLKR_SHIFT ((uint32_t)0x0000001Du)
#define CSL_MCASP_PFUNC_ACLKR_RESETVAL ((uint32_t)0x00000000u)
/*----ACLKR Tokens----*/
#define CSL_MCASP_PFUNC_ACLKR_MCASP ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_ACLKR_GPIO ((uint32_t)0x00000001u)

#define CSL_MCASP_PFUNC_AFSX_MASK ((uint32_t)0x10000000u)
#define CSL_MCASP_PFUNC_AFSX_SHIFT ((uint32_t)0x0000001Cu)
#define CSL_MCASP_PFUNC_AFSX_RESETVAL ((uint32_t)0x00000000u)
/*----AFSX Tokens----*/
#define CSL_MCASP_PFUNC_AFSX_MCASP ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AFSX_GPIO ((uint32_t)0x00000001u)

#define CSL_MCASP_PFUNC_AHCLKX_MASK ((uint32_t)0x08000000u)
#define CSL_MCASP_PFUNC_AHCLKX_SHIFT ((uint32_t)0x0000001Bu)
#define CSL_MCASP_PFUNC_AHCLKX_RESETVAL ((uint32_t)0x00000000u)
/*----AHCLKX Tokens----*/
#define CSL_MCASP_PFUNC_AHCLKX_MCASP ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AHCLKX_GPIO ((uint32_t)0x00000001u)

#define CSL_MCASP_PFUNC_ACLKX_MASK ((uint32_t)0x04000000u)
#define CSL_MCASP_PFUNC_ACLKX_SHIFT ((uint32_t)0x0000001Au)
#define CSL_MCASP_PFUNC_ACLKX_RESETVAL ((uint32_t)0x00000000u)
/*----ACLKX Tokens----*/
#define CSL_MCASP_PFUNC_ACLKX_MCASP ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_ACLKX_GPIO ((uint32_t)0x00000001u)

#define CSL_MCASP_PFUNC_AMUTE_MASK ((uint32_t)0x02000000u)
#define CSL_MCASP_PFUNC_AMUTE_SHIFT ((uint32_t)0x00000019u)
#define CSL_MCASP_PFUNC_AMUTE_RESETVAL ((uint32_t)0x00000000u)
/*----AMUTE Tokens----*/
#define CSL_MCASP_PFUNC_AMUTE_MCASP ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AMUTE_GPIO ((uint32_t)0x00000001u)

#define CSL_MCASP_PFUNC_AXR15_MASK ((uint32_t)0x00008000u)
#define CSL_MCASP_PFUNC_AXR15_SHIFT ((uint32_t)0x0000000Fu)
#define CSL_MCASP_PFUNC_AXR15_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AXR15_MCASP ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AXR15_GPIO ((uint32_t)0x00000001u)

#define CSL_MCASP_PFUNC_AXR14_MASK ((uint32_t)0x00004000u)
#define CSL_MCASP_PFUNC_AXR14_SHIFT ((uint32_t)0x0000000Eu)
#define CSL_MCASP_PFUNC_AXR14_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AXR14_MCASP ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AXR14_GPIO ((uint32_t)0x00000001u)

#define CSL_MCASP_PFUNC_AXR13_MASK ((uint32_t)0x00002000u)
#define CSL_MCASP_PFUNC_AXR13_SHIFT ((uint32_t)0x0000000Du)
#define CSL_MCASP_PFUNC_AXR13_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AXR13_MCASP ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AXR13_GPIO ((uint32_t)0x00000001u)

#define CSL_MCASP_PFUNC_AXR12_MASK ((uint32_t)0x00001000u)
#define CSL_MCASP_PFUNC_AXR12_SHIFT ((uint32_t)0x0000000Cu)
#define CSL_MCASP_PFUNC_AXR12_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AXR12_MCASP ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AXR12_GPIO ((uint32_t)0x00000001u)

#define CSL_MCASP_PFUNC_AXR11_MASK ((uint32_t)0x00000800u)
#define CSL_MCASP_PFUNC_AXR11_SHIFT ((uint32_t)0x0000000Bu)
#define CSL_MCASP_PFUNC_AXR11_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AXR11_MCASP ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AXR11_GPIO ((uint32_t)0x00000001u)

#define CSL_MCASP_PFUNC_AXR10_MASK ((uint32_t)0x00000400u)
#define CSL_MCASP_PFUNC_AXR10_SHIFT ((uint32_t)0x0000000Au)
#define CSL_MCASP_PFUNC_AXR10_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AXR10_MCASP ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AXR10_GPIO ((uint32_t)0x00000001u)
#define CSL_MCASP_PFUNC_AXR9_MASK ((uint32_t)0x00000200u)
#define CSL_MCASP_PFUNC_AXR9_SHIFT ((uint32_t)0x00000009u)
#define CSL_MCASP_PFUNC_AXR9_RESETVAL ((uint32_t)0x00000000u)
/*----AXR9 Tokens----*/
#define CSL_MCASP_PFUNC_AXR9_MCASP ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AXR9_GPIO ((uint32_t)0x00000001u)

#define CSL_MCASP_PFUNC_AXR8_MASK ((uint32_t)0x00000100u)
#define CSL_MCASP_PFUNC_AXR8_SHIFT ((uint32_t)0x00000008u)
#define CSL_MCASP_PFUNC_AXR8_RESETVAL ((uint32_t)0x00000000u)
/*----AXR8 Tokens----*/
#define CSL_MCASP_PFUNC_AXR8_MCASP ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AXR8_GPIO ((uint32_t)0x00000001u)

#define CSL_MCASP_PFUNC_AXR7_MASK ((uint32_t)0x00000080u)
#define CSL_MCASP_PFUNC_AXR7_SHIFT ((uint32_t)0x00000007u)
#define CSL_MCASP_PFUNC_AXR7_RESETVAL ((uint32_t)0x00000000u)
/*----AXR7 Tokens----*/
#define CSL_MCASP_PFUNC_AXR7_MCASP ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AXR7_GPIO ((uint32_t)0x00000001u)

#define CSL_MCASP_PFUNC_AXR6_MASK ((uint32_t)0x00000040u)
#define CSL_MCASP_PFUNC_AXR6_SHIFT ((uint32_t)0x00000006u)
#define CSL_MCASP_PFUNC_AXR6_RESETVAL ((uint32_t)0x00000000u)
/*----AXR6 Tokens----*/
#define CSL_MCASP_PFUNC_AXR6_MCASP ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AXR6_GPIO ((uint32_t)0x00000001u)

#define CSL_MCASP_PFUNC_AXR5_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_PFUNC_AXR5_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_PFUNC_AXR5_RESETVAL ((uint32_t)0x00000000u)
/*----AXR5 Tokens----*/
#define CSL_MCASP_PFUNC_AXR5_MCASP ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AXR5_GPIO ((uint32_t)0x00000001u)

#define CSL_MCASP_PFUNC_AXR4_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_PFUNC_AXR4_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_PFUNC_AXR4_RESETVAL ((uint32_t)0x00000000u)
/*----AXR4 Tokens----*/
#define CSL_MCASP_PFUNC_AXR4_MCASP ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AXR4_GPIO ((uint32_t)0x00000001u)

#define CSL_MCASP_PFUNC_AXR3_MASK ((uint32_t)0x00000008u)
#define CSL_MCASP_PFUNC_AXR3_SHIFT ((uint32_t)0x00000003u)
#define CSL_MCASP_PFUNC_AXR3_RESETVAL ((uint32_t)0x00000000u)
/*----AXR3 Tokens----*/
#define CSL_MCASP_PFUNC_AXR3_MCASP ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AXR3_GPIO ((uint32_t)0x00000001u)

#define CSL_MCASP_PFUNC_AXR2_MASK ((uint32_t)0x00000004u)
#define CSL_MCASP_PFUNC_AXR2_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_PFUNC_AXR2_RESETVAL ((uint32_t)0x00000000u)
/*----AXR2 Tokens----*/
#define CSL_MCASP_PFUNC_AXR2_MCASP ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AXR2_GPIO ((uint32_t)0x00000001u)

#define CSL_MCASP_PFUNC_AXR1_MASK ((uint32_t)0x00000002u)
#define CSL_MCASP_PFUNC_AXR1_SHIFT ((uint32_t)0x00000001u)
#define CSL_MCASP_PFUNC_AXR1_RESETVAL ((uint32_t)0x00000000u)
/*----AXR1 Tokens----*/
#define CSL_MCASP_PFUNC_AXR1_MCASP ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AXR1_GPIO ((uint32_t)0x00000001u)

#define CSL_MCASP_PFUNC_AXR0_MASK ((uint32_t)0x00000001u)
#define CSL_MCASP_PFUNC_AXR0_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AXR0_RESETVAL ((uint32_t)0x00000000u)
/*----AXR0 Tokens----*/
#define CSL_MCASP_PFUNC_AXR0_MCASP ((uint32_t)0x00000000u)
#define CSL_MCASP_PFUNC_AXR0_GPIO ((uint32_t)0x00000001u)

#define CSL_MCASP_PFUNC_RESETVAL ((uint32_t)0x00000000u)

/* PDIR */

#define CSL_MCASP_PDIR_AFSR_MASK ((uint32_t)0x80000000u)
#define CSL_MCASP_PDIR_AFSR_SHIFT ((uint32_t)0x0000001Fu)
#define CSL_MCASP_PDIR_AFSR_RESETVAL ((uint32_t)0x00000000u)
/*----AFSR Tokens----*/
#define CSL_MCASP_PDIR_AFSR_INPUT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AFSR_OUTPUT ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIR_AHCLKR_MASK ((uint32_t)0x40000000u)
#define CSL_MCASP_PDIR_AHCLKR_SHIFT ((uint32_t)0x0000001Eu)
#define CSL_MCASP_PDIR_AHCLKR_RESETVAL ((uint32_t)0x00000000u)
/*----AHCLKR Tokens----*/
#define CSL_MCASP_PDIR_AHCLKR_INPUT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AHCLKR_OUTPUT ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIR_ACLKR_MASK ((uint32_t)0x20000000u)
#define CSL_MCASP_PDIR_ACLKR_SHIFT ((uint32_t)0x0000001Du)
#define CSL_MCASP_PDIR_ACLKR_RESETVAL ((uint32_t)0x00000000u)
/*----ACLKR Tokens----*/
#define CSL_MCASP_PDIR_ACLKR_INPUT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_ACLKR_OUTPUT ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIR_AFSX_MASK ((uint32_t)0x10000000u)
#define CSL_MCASP_PDIR_AFSX_SHIFT ((uint32_t)0x0000001Cu)
#define CSL_MCASP_PDIR_AFSX_RESETVAL ((uint32_t)0x00000000u)
/*----AFSX Tokens----*/
#define CSL_MCASP_PDIR_AFSX_INPUT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AFSX_OUTPUT ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIR_AHCLKX_MASK ((uint32_t)0x08000000u)
#define CSL_MCASP_PDIR_AHCLKX_SHIFT ((uint32_t)0x0000001Bu)
#define CSL_MCASP_PDIR_AHCLKX_RESETVAL ((uint32_t)0x00000000u)
/*----AHCLKX Tokens----*/
#define CSL_MCASP_PDIR_AHCLKX_INPUT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AHCLKX_OUTPUT ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIR_ACLKX_MASK ((uint32_t)0x04000000u)
#define CSL_MCASP_PDIR_ACLKX_SHIFT ((uint32_t)0x0000001Au)
#define CSL_MCASP_PDIR_ACLKX_RESETVAL ((uint32_t)0x00000000u)
/*----ACLKX Tokens----*/
#define CSL_MCASP_PDIR_ACLKX_INPUT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_ACLKX_OUTPUT ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIR_AMUTE_MASK ((uint32_t)0x02000000u)
#define CSL_MCASP_PDIR_AMUTE_SHIFT ((uint32_t)0x00000019u)
#define CSL_MCASP_PDIR_AMUTE_RESETVAL ((uint32_t)0x00000000u)
/*----AMUTE Tokens----*/
#define CSL_MCASP_PDIR_AMUTE_INPUT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AMUTE_OUTPUT ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIR_AXR15_MASK ((uint32_t)0x00008000u)
#define CSL_MCASP_PDIR_AXR15_SHIFT ((uint32_t)0x0000000Fu)
#define CSL_MCASP_PDIR_AXR15_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AXR15_INPUT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AXR15_OUTPUT ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIR_AXR14_MASK ((uint32_t)0x00004000u)
#define CSL_MCASP_PDIR_AXR14_SHIFT ((uint32_t)0x0000000Eu)
#define CSL_MCASP_PDIR_AXR14_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AXR14_INPUT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AXR14_OUTPUT ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIR_AXR13_MASK ((uint32_t)0x00002000u)
#define CSL_MCASP_PDIR_AXR13_SHIFT ((uint32_t)0x0000000Du)
#define CSL_MCASP_PDIR_AXR13_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AXR13_INPUT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AXR13_OUTPUT ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIR_AXR12_MASK ((uint32_t)0x00001000u)
#define CSL_MCASP_PDIR_AXR12_SHIFT ((uint32_t)0x0000000Cu)
#define CSL_MCASP_PDIR_AXR12_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AXR12_INPUT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AXR12_OUTPUT ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIR_AXR11_MASK ((uint32_t)0x00000800u)
#define CSL_MCASP_PDIR_AXR11_SHIFT ((uint32_t)0x0000000Bu)
#define CSL_MCASP_PDIR_AXR11_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AXR11_INPUT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AXR11_OUTPUT ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIR_AXR10_MASK ((uint32_t)0x00000400u)
#define CSL_MCASP_PDIR_AXR10_SHIFT ((uint32_t)0x0000000Au)
#define CSL_MCASP_PDIR_AXR10_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AXR10_INPUT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AXR10_OUTPUT ((uint32_t)0x00000001u)
#define CSL_MCASP_PDIR_AXR9_MASK ((uint32_t)0x00000200u)
#define CSL_MCASP_PDIR_AXR9_SHIFT ((uint32_t)0x00000009u)
#define CSL_MCASP_PDIR_AXR9_RESETVAL ((uint32_t)0x00000000u)
/*----AXR9 Tokens----*/
#define CSL_MCASP_PDIR_AXR9_INPUT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AXR9_OUTPUT ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIR_AXR8_MASK ((uint32_t)0x00000100u)
#define CSL_MCASP_PDIR_AXR8_SHIFT ((uint32_t)0x00000008u)
#define CSL_MCASP_PDIR_AXR8_RESETVAL ((uint32_t)0x00000000u)
/*----AXR8 Tokens----*/
#define CSL_MCASP_PDIR_AXR8_INPUT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AXR8_OUTPUT ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIR_AXR7_MASK ((uint32_t)0x00000080u)
#define CSL_MCASP_PDIR_AXR7_SHIFT ((uint32_t)0x00000007u)
#define CSL_MCASP_PDIR_AXR7_RESETVAL ((uint32_t)0x00000000u)
/*----AXR7 Tokens----*/
#define CSL_MCASP_PDIR_AXR7_INPUT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AXR7_OUTPUT ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIR_AXR6_MASK ((uint32_t)0x00000040u)
#define CSL_MCASP_PDIR_AXR6_SHIFT ((uint32_t)0x00000006u)
#define CSL_MCASP_PDIR_AXR6_RESETVAL ((uint32_t)0x00000000u)
/*----AXR6 Tokens----*/
#define CSL_MCASP_PDIR_AXR6_INPUT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AXR6_OUTPUT ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIR_AXR5_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_PDIR_AXR5_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_PDIR_AXR5_RESETVAL ((uint32_t)0x00000000u)
/*----AXR5 Tokens----*/
#define CSL_MCASP_PDIR_AXR5_INPUT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AXR5_OUTPUT ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIR_AXR4_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_PDIR_AXR4_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_PDIR_AXR4_RESETVAL ((uint32_t)0x00000000u)
/*----AXR4 Tokens----*/
#define CSL_MCASP_PDIR_AXR4_INPUT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AXR4_OUTPUT ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIR_AXR3_MASK ((uint32_t)0x00000008u)
#define CSL_MCASP_PDIR_AXR3_SHIFT ((uint32_t)0x00000003u)
#define CSL_MCASP_PDIR_AXR3_RESETVAL ((uint32_t)0x00000000u)
/*----AXR3 Tokens----*/
#define CSL_MCASP_PDIR_AXR3_INPUT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AXR3_OUTPUT ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIR_AXR2_MASK ((uint32_t)0x00000004u)
#define CSL_MCASP_PDIR_AXR2_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_PDIR_AXR2_RESETVAL ((uint32_t)0x00000000u)
/*----AXR2 Tokens----*/
#define CSL_MCASP_PDIR_AXR2_INPUT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AXR2_OUTPUT ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIR_AXR1_MASK ((uint32_t)0x00000002u)
#define CSL_MCASP_PDIR_AXR1_SHIFT ((uint32_t)0x00000001u)
#define CSL_MCASP_PDIR_AXR1_RESETVAL ((uint32_t)0x00000000u)
/*----AXR1 Tokens----*/
#define CSL_MCASP_PDIR_AXR1_INPUT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AXR1_OUTPUT ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIR_AXR0_MASK ((uint32_t)0x00000001u)
#define CSL_MCASP_PDIR_AXR0_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AXR0_RESETVAL ((uint32_t)0x00000000u)
/*----AXR0 Tokens----*/
#define CSL_MCASP_PDIR_AXR0_INPUT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIR_AXR0_OUTPUT ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIR_RESETVAL ((uint32_t)0x00000000u)

/* PDOUT */

#define CSL_MCASP_PDOUT_AFSR_MASK ((uint32_t)0x80000000u)
#define CSL_MCASP_PDOUT_AFSR_SHIFT ((uint32_t)0x0000001Fu)
#define CSL_MCASP_PDOUT_AFSR_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AFSR_DRV0 ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AFSR_DRV1 ((uint32_t)0x00000001u)

#define CSL_MCASP_PDOUT_AHCLKR_MASK ((uint32_t)0x40000000u)
#define CSL_MCASP_PDOUT_AHCLKR_SHIFT ((uint32_t)0x0000001Eu)
#define CSL_MCASP_PDOUT_AHCLKR_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AHCLKR_DRV0 ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AHCLKR_DRV1 ((uint32_t)0x00000001u)

#define CSL_MCASP_PDOUT_ACLKR_MASK ((uint32_t)0x20000000u)
#define CSL_MCASP_PDOUT_ACLKR_SHIFT ((uint32_t)0x0000001Du)
#define CSL_MCASP_PDOUT_ACLKR_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_ACLKR_DRV0 ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_ACLKR_DRV1 ((uint32_t)0x00000001u)

#define CSL_MCASP_PDOUT_AFSX_MASK ((uint32_t)0x10000000u)
#define CSL_MCASP_PDOUT_AFSX_SHIFT ((uint32_t)0x0000001Cu)
#define CSL_MCASP_PDOUT_AFSX_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AFSX_DRV0 ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AFSX_DRV1 ((uint32_t)0x00000001u)

#define CSL_MCASP_PDOUT_AHCLKX_MASK ((uint32_t)0x08000000u)
#define CSL_MCASP_PDOUT_AHCLKX_SHIFT ((uint32_t)0x0000001Bu)
#define CSL_MCASP_PDOUT_AHCLKX_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AHCLKX_DRV0 ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AHCLKX_DRV1 ((uint32_t)0x00000001u)

#define CSL_MCASP_PDOUT_ACLKX_MASK ((uint32_t)0x04000000u)
#define CSL_MCASP_PDOUT_ACLKX_SHIFT ((uint32_t)0x0000001Au)
#define CSL_MCASP_PDOUT_ACLKX_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_ACLKX_DRV0 ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_ACLKX_DRV1 ((uint32_t)0x00000001u)

#define CSL_MCASP_PDOUT_AMUTE_MASK ((uint32_t)0x02000000u)
#define CSL_MCASP_PDOUT_AMUTE_SHIFT ((uint32_t)0x00000019u)
#define CSL_MCASP_PDOUT_AMUTE_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AMUTE_DRV0 ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AMUTE_DRV1 ((uint32_t)0x00000001u)


#define CSL_MCASP_PDOUT_AXR15_MASK ((uint32_t)0x00008000u)
#define CSL_MCASP_PDOUT_AXR15_SHIFT ((uint32_t)0x0000000Fu)
#define CSL_MCASP_PDOUT_AXR15_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR15_DRV0 ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR15_DRV1 ((uint32_t)0x00000001u)

#define CSL_MCASP_PDOUT_AXR14_MASK ((uint32_t)0x00004000u)
#define CSL_MCASP_PDOUT_AXR14_SHIFT ((uint32_t)0x0000000Eu)
#define CSL_MCASP_PDOUT_AXR14_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR14_DRV0 ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR14_DRV1 ((uint32_t)0x00000001u)

#define CSL_MCASP_PDOUT_AXR13_MASK ((uint32_t)0x00002000u)
#define CSL_MCASP_PDOUT_AXR13_SHIFT ((uint32_t)0x0000000Du)
#define CSL_MCASP_PDOUT_AXR13_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR13_DRV0 ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR13_DRV1 ((uint32_t)0x00000001u)

#define CSL_MCASP_PDOUT_AXR12_MASK ((uint32_t)0x00001000u)
#define CSL_MCASP_PDOUT_AXR12_SHIFT ((uint32_t)0x0000000Cu)
#define CSL_MCASP_PDOUT_AXR12_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR12_DRV0 ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR12_DRV1 ((uint32_t)0x00000001u)

#define CSL_MCASP_PDOUT_AXR11_MASK ((uint32_t)0x00000800u)
#define CSL_MCASP_PDOUT_AXR11_SHIFT ((uint32_t)0x0000000Bu)
#define CSL_MCASP_PDOUT_AXR11_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR11_DRV0 ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR11_DRV1 ((uint32_t)0x00000001u)

#define CSL_MCASP_PDOUT_AXR10_MASK ((uint32_t)0x00000400u)
#define CSL_MCASP_PDOUT_AXR10_SHIFT ((uint32_t)0x0000000Au)
#define CSL_MCASP_PDOUT_AXR10_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR10_DRV0 ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR10_DRV1 ((uint32_t)0x00000001u)

#define CSL_MCASP_PDOUT_AXR9_MASK ((uint32_t)0x00000200u)
#define CSL_MCASP_PDOUT_AXR9_SHIFT ((uint32_t)0x00000009u)
#define CSL_MCASP_PDOUT_AXR9_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR9_DRV0 ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR9_DRV1 ((uint32_t)0x00000001u)

#define CSL_MCASP_PDOUT_AXR8_MASK ((uint32_t)0x00000100u)
#define CSL_MCASP_PDOUT_AXR8_SHIFT ((uint32_t)0x00000008u)
#define CSL_MCASP_PDOUT_AXR8_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR8_DRV0 ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR8_DRV1 ((uint32_t)0x00000001u)

#define CSL_MCASP_PDOUT_AXR7_MASK ((uint32_t)0x00000080u)
#define CSL_MCASP_PDOUT_AXR7_SHIFT ((uint32_t)0x00000007u)
#define CSL_MCASP_PDOUT_AXR7_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR7_DRV0 ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR7_DRV1 ((uint32_t)0x00000001u)

#define CSL_MCASP_PDOUT_AXR6_MASK ((uint32_t)0x00000040u)
#define CSL_MCASP_PDOUT_AXR6_SHIFT ((uint32_t)0x00000006u)
#define CSL_MCASP_PDOUT_AXR6_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR6_DRV0 ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR6_DRV1 ((uint32_t)0x00000001u)

#define CSL_MCASP_PDOUT_AXR5_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_PDOUT_AXR5_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_PDOUT_AXR5_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR5_DRV0 ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR5_DRV1 ((uint32_t)0x00000001u)

#define CSL_MCASP_PDOUT_AXR4_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_PDOUT_AXR4_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_PDOUT_AXR4_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR4_DRV0 ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR4_DRV1 ((uint32_t)0x00000001u)

#define CSL_MCASP_PDOUT_AXR3_MASK ((uint32_t)0x00000008u)
#define CSL_MCASP_PDOUT_AXR3_SHIFT ((uint32_t)0x00000003u)
#define CSL_MCASP_PDOUT_AXR3_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR3_DRV0 ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR3_DRV1 ((uint32_t)0x00000001u)

#define CSL_MCASP_PDOUT_AXR2_MASK ((uint32_t)0x00000004u)
#define CSL_MCASP_PDOUT_AXR2_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_PDOUT_AXR2_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR2_DRV0 ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR2_DRV1 ((uint32_t)0x00000001u)

#define CSL_MCASP_PDOUT_AXR1_MASK ((uint32_t)0x00000002u)
#define CSL_MCASP_PDOUT_AXR1_SHIFT ((uint32_t)0x00000001u)
#define CSL_MCASP_PDOUT_AXR1_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR1_DRV0 ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR1_DRV1 ((uint32_t)0x00000001u)

#define CSL_MCASP_PDOUT_AXR0_MASK ((uint32_t)0x00000001u)
#define CSL_MCASP_PDOUT_AXR0_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR0_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR0_DRV0 ((uint32_t)0x00000000u)
#define CSL_MCASP_PDOUT_AXR0_DRV1 ((uint32_t)0x00000001u)

#define CSL_MCASP_PDOUT_RESETVAL ((uint32_t)0x00000000u)

/* PDIN */

#define CSL_MCASP_PDIN_AFSR_MASK ((uint32_t)0x80000000u)
#define CSL_MCASP_PDIN_AFSR_SHIFT ((uint32_t)0x0000001Fu)
#define CSL_MCASP_PDIN_AFSR_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AFSR_LOW ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AFSR_HIGH ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIN_AHCLKR_MASK ((uint32_t)0x40000000u)
#define CSL_MCASP_PDIN_AHCLKR_SHIFT ((uint32_t)0x0000001Eu)
#define CSL_MCASP_PDIN_AHCLKR_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AHCLKR_LOW ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AHCLKR_HIGH ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIN_ACLKR_MASK ((uint32_t)0x20000000u)
#define CSL_MCASP_PDIN_ACLKR_SHIFT ((uint32_t)0x0000001Du)
#define CSL_MCASP_PDIN_ACLKR_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_ACLKR_LOW ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_ACLKR_HIGH ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIN_AFSX_MASK ((uint32_t)0x10000000u)
#define CSL_MCASP_PDIN_AFSX_SHIFT ((uint32_t)0x0000001Cu)
#define CSL_MCASP_PDIN_AFSX_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AFSX_LOW ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AFSX_HIGH ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIN_AHCLKX_MASK ((uint32_t)0x08000000u)
#define CSL_MCASP_PDIN_AHCLKX_SHIFT ((uint32_t)0x0000001Bu)
#define CSL_MCASP_PDIN_AHCLKX_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AHCLKX_LOW ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AHCLKX_HIGH ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIN_ACLKX_MASK ((uint32_t)0x04000000u)
#define CSL_MCASP_PDIN_ACLKX_SHIFT ((uint32_t)0x0000001Au)
#define CSL_MCASP_PDIN_ACLKX_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_ACLKX_LOW ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_ACLKX_HIGH ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIN_AMUTE_MASK ((uint32_t)0x02000000u)
#define CSL_MCASP_PDIN_AMUTE_SHIFT ((uint32_t)0x00000019u)
#define CSL_MCASP_PDIN_AMUTE_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AMUTE_LOW ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AMUTE_HIGH ((uint32_t)0x00000001u)


#define CSL_MCASP_PDIN_AXR15_MASK ((uint32_t)0x00008000u)
#define CSL_MCASP_PDIN_AXR15_SHIFT ((uint32_t)0x0000000Fu)
#define CSL_MCASP_PDIN_AXR15_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR15_LOW ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR15_HIGH ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIN_AXR14_MASK ((uint32_t)0x00004000u)
#define CSL_MCASP_PDIN_AXR14_SHIFT ((uint32_t)0x0000000Eu)
#define CSL_MCASP_PDIN_AXR14_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR14_LOW ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR14_HIGH ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIN_AXR13_MASK ((uint32_t)0x00002000u)
#define CSL_MCASP_PDIN_AXR13_SHIFT ((uint32_t)0x0000000Du)
#define CSL_MCASP_PDIN_AXR13_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR13_LOW ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR13_HIGH ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIN_AXR12_MASK ((uint32_t)0x00001000u)
#define CSL_MCASP_PDIN_AXR12_SHIFT ((uint32_t)0x0000000Cu)
#define CSL_MCASP_PDIN_AXR12_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR12_LOW ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR12_HIGH ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIN_AXR11_MASK ((uint32_t)0x00000800u)
#define CSL_MCASP_PDIN_AXR11_SHIFT ((uint32_t)0x0000000Bu)
#define CSL_MCASP_PDIN_AXR11_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR11_LOW ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR11_HIGH ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIN_AXR10_MASK ((uint32_t)0x00000400u)
#define CSL_MCASP_PDIN_AXR10_SHIFT ((uint32_t)0x0000000Au)
#define CSL_MCASP_PDIN_AXR10_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR10_LOW ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR10_HIGH ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIN_AXR9_MASK ((uint32_t)0x00000200u)
#define CSL_MCASP_PDIN_AXR9_SHIFT ((uint32_t)0x00000009u)
#define CSL_MCASP_PDIN_AXR9_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR9_LOW ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR9_HIGH ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIN_AXR8_MASK ((uint32_t)0x00000100u)
#define CSL_MCASP_PDIN_AXR8_SHIFT ((uint32_t)0x00000008u)
#define CSL_MCASP_PDIN_AXR8_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR8_LOW ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR8_HIGH ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIN_AXR7_MASK ((uint32_t)0x00000080u)
#define CSL_MCASP_PDIN_AXR7_SHIFT ((uint32_t)0x00000007u)
#define CSL_MCASP_PDIN_AXR7_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR7_LOW ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR7_HIGH ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIN_AXR6_MASK ((uint32_t)0x00000040u)
#define CSL_MCASP_PDIN_AXR6_SHIFT ((uint32_t)0x00000006u)
#define CSL_MCASP_PDIN_AXR6_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR6_LOW ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR6_HIGH ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIN_AXR5_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_PDIN_AXR5_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_PDIN_AXR5_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR5_LOW ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR5_HIGH ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIN_AXR4_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_PDIN_AXR4_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_PDIN_AXR4_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR4_LOW ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR4_HIGH ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIN_AXR3_MASK ((uint32_t)0x00000008u)
#define CSL_MCASP_PDIN_AXR3_SHIFT ((uint32_t)0x00000003u)
#define CSL_MCASP_PDIN_AXR3_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR3_LOW ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR3_HIGH ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIN_AXR2_MASK ((uint32_t)0x00000004u)
#define CSL_MCASP_PDIN_AXR2_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_PDIN_AXR2_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR2_LOW ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR2_HIGH ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIN_AXR1_MASK ((uint32_t)0x00000002u)
#define CSL_MCASP_PDIN_AXR1_SHIFT ((uint32_t)0x00000001u)
#define CSL_MCASP_PDIN_AXR1_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR1_LOW ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR1_HIGH ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIN_AXR0_MASK ((uint32_t)0x00000001u)
#define CSL_MCASP_PDIN_AXR0_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR0_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR0_LOW ((uint32_t)0x00000000u)
#define CSL_MCASP_PDIN_AXR0_HIGH ((uint32_t)0x00000001u)

#define CSL_MCASP_PDIN_RESETVAL ((uint32_t)0x00000000u)

/* PDSET */

#define CSL_MCASP_PDSET_AFSR_MASK ((uint32_t)0x80000000u)
#define CSL_MCASP_PDSET_AFSR_SHIFT ((uint32_t)0x0000001Fu)
#define CSL_MCASP_PDSET_AFSR_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AFSR_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AFSR_SET ((uint32_t)0x00000001u)

#define CSL_MCASP_PDSET_AHCLKR_MASK ((uint32_t)0x40000000u)
#define CSL_MCASP_PDSET_AHCLKR_SHIFT ((uint32_t)0x0000001Eu)
#define CSL_MCASP_PDSET_AHCLKR_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AHCLKR_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AHCLKR_SET ((uint32_t)0x00000001u)

#define CSL_MCASP_PDSET_ACLKR_MASK ((uint32_t)0x20000000u)
#define CSL_MCASP_PDSET_ACLKR_SHIFT ((uint32_t)0x0000001Du)
#define CSL_MCASP_PDSET_ACLKR_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_ACLKR_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_ACLKR_SET ((uint32_t)0x00000001u)

#define CSL_MCASP_PDSET_AFSX_MASK ((uint32_t)0x10000000u)
#define CSL_MCASP_PDSET_AFSX_SHIFT ((uint32_t)0x0000001Cu)
#define CSL_MCASP_PDSET_AFSX_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AFSX_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AFSX_SET ((uint32_t)0x00000001u)

#define CSL_MCASP_PDSET_AHCLKX_MASK ((uint32_t)0x08000000u)
#define CSL_MCASP_PDSET_AHCLKX_SHIFT ((uint32_t)0x0000001Bu)
#define CSL_MCASP_PDSET_AHCLKX_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AHCLKX_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AHCLKX_SET ((uint32_t)0x00000001u)

#define CSL_MCASP_PDSET_ACLKX_MASK ((uint32_t)0x04000000u)
#define CSL_MCASP_PDSET_ACLKX_SHIFT ((uint32_t)0x0000001Au)
#define CSL_MCASP_PDSET_ACLKX_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_ACLKX_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_ACLKX_SET ((uint32_t)0x00000001u)

#define CSL_MCASP_PDSET_AMUTE_MASK ((uint32_t)0x02000000u)
#define CSL_MCASP_PDSET_AMUTE_SHIFT ((uint32_t)0x00000019u)
#define CSL_MCASP_PDSET_AMUTE_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AMUTE_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AMUTE_SET ((uint32_t)0x00000001u)


#define CSL_MCASP_PDSET_AXR15_MASK ((uint32_t)0x00008000u)
#define CSL_MCASP_PDSET_AXR15_SHIFT ((uint32_t)0x0000000Fu)
#define CSL_MCASP_PDSET_AXR15_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR15_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR15_SET ((uint32_t)0x00000001u)

#define CSL_MCASP_PDSET_AXR14_MASK ((uint32_t)0x00004000u)
#define CSL_MCASP_PDSET_AXR14_SHIFT ((uint32_t)0x0000000Eu)
#define CSL_MCASP_PDSET_AXR14_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR14_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR14_SET ((uint32_t)0x00000001u)

#define CSL_MCASP_PDSET_AXR13_MASK ((uint32_t)0x00002000u)
#define CSL_MCASP_PDSET_AXR13_SHIFT ((uint32_t)0x0000000Du)
#define CSL_MCASP_PDSET_AXR13_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR13_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR13_SET ((uint32_t)0x00000001u)

#define CSL_MCASP_PDSET_AXR12_MASK ((uint32_t)0x00001000u)
#define CSL_MCASP_PDSET_AXR12_SHIFT ((uint32_t)0x0000000Cu)
#define CSL_MCASP_PDSET_AXR12_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR12_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR12_SET ((uint32_t)0x00000001u)

#define CSL_MCASP_PDSET_AXR11_MASK ((uint32_t)0x00000800u)
#define CSL_MCASP_PDSET_AXR11_SHIFT ((uint32_t)0x0000000Bu)
#define CSL_MCASP_PDSET_AXR11_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR11_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR11_SET ((uint32_t)0x00000001u)

#define CSL_MCASP_PDSET_AXR10_MASK ((uint32_t)0x00000400u)
#define CSL_MCASP_PDSET_AXR10_SHIFT ((uint32_t)0x0000000Au)
#define CSL_MCASP_PDSET_AXR10_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR10_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR10_SET ((uint32_t)0x00000001u)

#define CSL_MCASP_PDSET_AXR9_MASK ((uint32_t)0x00000200u)
#define CSL_MCASP_PDSET_AXR9_SHIFT ((uint32_t)0x00000009u)
#define CSL_MCASP_PDSET_AXR9_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR9_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR9_SET ((uint32_t)0x00000001u)

#define CSL_MCASP_PDSET_AXR8_MASK ((uint32_t)0x00000100u)
#define CSL_MCASP_PDSET_AXR8_SHIFT ((uint32_t)0x00000008u)
#define CSL_MCASP_PDSET_AXR8_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR8_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR8_SET ((uint32_t)0x00000001u)

#define CSL_MCASP_PDSET_AXR7_MASK ((uint32_t)0x00000080u)
#define CSL_MCASP_PDSET_AXR7_SHIFT ((uint32_t)0x00000007u)
#define CSL_MCASP_PDSET_AXR7_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR7_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR7_SET ((uint32_t)0x00000001u)

#define CSL_MCASP_PDSET_AXR6_MASK ((uint32_t)0x00000040u)
#define CSL_MCASP_PDSET_AXR6_SHIFT ((uint32_t)0x00000006u)
#define CSL_MCASP_PDSET_AXR6_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR6_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR6_SET ((uint32_t)0x00000001u)

#define CSL_MCASP_PDSET_AXR5_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_PDSET_AXR5_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_PDSET_AXR5_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR5_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR5_SET ((uint32_t)0x00000001u)

#define CSL_MCASP_PDSET_AXR4_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_PDSET_AXR4_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_PDSET_AXR4_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR4_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR4_SET ((uint32_t)0x00000001u)

#define CSL_MCASP_PDSET_AXR3_MASK ((uint32_t)0x00000008u)
#define CSL_MCASP_PDSET_AXR3_SHIFT ((uint32_t)0x00000003u)
#define CSL_MCASP_PDSET_AXR3_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR3_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR3_SET ((uint32_t)0x00000001u)

#define CSL_MCASP_PDSET_AXR2_MASK ((uint32_t)0x00000004u)
#define CSL_MCASP_PDSET_AXR2_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_PDSET_AXR2_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR2_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR2_SET ((uint32_t)0x00000001u)

#define CSL_MCASP_PDSET_AXR1_MASK ((uint32_t)0x00000002u)
#define CSL_MCASP_PDSET_AXR1_SHIFT ((uint32_t)0x00000001u)
#define CSL_MCASP_PDSET_AXR1_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR1_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR1_SET ((uint32_t)0x00000001u)

#define CSL_MCASP_PDSET_AXR0_MASK ((uint32_t)0x00000001u)
#define CSL_MCASP_PDSET_AXR0_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR0_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR0_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDSET_AXR0_SET ((uint32_t)0x00000001u)

#define CSL_MCASP_PDSET_RESETVAL ((uint32_t)0x00000000u)

/* PDCLR */

#define CSL_MCASP_PDCLR_AFSR_MASK ((uint32_t)0x80000000u)
#define CSL_MCASP_PDCLR_AFSR_SHIFT ((uint32_t)0x0000001Fu)
#define CSL_MCASP_PDCLR_AFSR_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AFSR_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AFSR_CLEAR ((uint32_t)0x00000001u)

#define CSL_MCASP_PDCLR_AHCLKR_MASK ((uint32_t)0x40000000u)
#define CSL_MCASP_PDCLR_AHCLKR_SHIFT ((uint32_t)0x0000001Eu)
#define CSL_MCASP_PDCLR_AHCLKR_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AHCLKR_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AHCLKR_CLEAR ((uint32_t)0x00000001u)

#define CSL_MCASP_PDCLR_ACLKR_MASK ((uint32_t)0x20000000u)
#define CSL_MCASP_PDCLR_ACLKR_SHIFT ((uint32_t)0x0000001Du)
#define CSL_MCASP_PDCLR_ACLKR_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_ACLKR_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_ACLKR_CLEAR ((uint32_t)0x00000001u)

#define CSL_MCASP_PDCLR_AFSX_MASK ((uint32_t)0x10000000u)
#define CSL_MCASP_PDCLR_AFSX_SHIFT ((uint32_t)0x0000001Cu)
#define CSL_MCASP_PDCLR_AFSX_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AFSX_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AFSX_CLEAR ((uint32_t)0x00000001u)

#define CSL_MCASP_PDCLR_AHCLKX_MASK ((uint32_t)0x08000000u)
#define CSL_MCASP_PDCLR_AHCLKX_SHIFT ((uint32_t)0x0000001Bu)
#define CSL_MCASP_PDCLR_AHCLKX_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AHCLKX_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AHCLKX_CLEAR ((uint32_t)0x00000001u)

#define CSL_MCASP_PDCLR_ACLKX_MASK ((uint32_t)0x04000000u)
#define CSL_MCASP_PDCLR_ACLKX_SHIFT ((uint32_t)0x0000001Au)
#define CSL_MCASP_PDCLR_ACLKX_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_ACLKX_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_ACLKX_CLEAR ((uint32_t)0x00000001u)

#define CSL_MCASP_PDCLR_AMUTE_MASK ((uint32_t)0x02000000u)
#define CSL_MCASP_PDCLR_AMUTE_SHIFT ((uint32_t)0x00000019u)
#define CSL_MCASP_PDCLR_AMUTE_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AMUTE_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AMUTE_CLEAR ((uint32_t)0x00000001u)


#define CSL_MCASP_PDCLR_AXR15_MASK ((uint32_t)0x00008000u)
#define CSL_MCASP_PDCLR_AXR15_SHIFT ((uint32_t)0x0000000Fu)
#define CSL_MCASP_PDCLR_AXR15_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR15_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR15_CLEAR ((uint32_t)0x00000001u)

#define CSL_MCASP_PDCLR_AXR14_MASK ((uint32_t)0x00004000u)
#define CSL_MCASP_PDCLR_AXR14_SHIFT ((uint32_t)0x0000000Eu)
#define CSL_MCASP_PDCLR_AXR14_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR14_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR14_CLEAR ((uint32_t)0x00000001u)

#define CSL_MCASP_PDCLR_AXR13_MASK ((uint32_t)0x00002000u)
#define CSL_MCASP_PDCLR_AXR13_SHIFT ((uint32_t)0x0000000Du)
#define CSL_MCASP_PDCLR_AXR13_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR13_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR13_CLEAR ((uint32_t)0x00000001u)

#define CSL_MCASP_PDCLR_AXR12_MASK ((uint32_t)0x00001000u)
#define CSL_MCASP_PDCLR_AXR12_SHIFT ((uint32_t)0x0000000Cu)
#define CSL_MCASP_PDCLR_AXR12_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR12_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR12_CLEAR ((uint32_t)0x00000001u)

#define CSL_MCASP_PDCLR_AXR11_MASK ((uint32_t)0x00000800u)
#define CSL_MCASP_PDCLR_AXR11_SHIFT ((uint32_t)0x0000000Bu)
#define CSL_MCASP_PDCLR_AXR11_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR11_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR11_CLEAR ((uint32_t)0x00000001u)

#define CSL_MCASP_PDCLR_AXR10_MASK ((uint32_t)0x00000400u)
#define CSL_MCASP_PDCLR_AXR10_SHIFT ((uint32_t)0x0000000Au)
#define CSL_MCASP_PDCLR_AXR10_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR10_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR10_CLEAR ((uint32_t)0x00000001u)

#define CSL_MCASP_PDCLR_AXR9_MASK ((uint32_t)0x00000200u)
#define CSL_MCASP_PDCLR_AXR9_SHIFT ((uint32_t)0x00000009u)
#define CSL_MCASP_PDCLR_AXR9_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR9_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR9_CLEAR ((uint32_t)0x00000001u)

#define CSL_MCASP_PDCLR_AXR8_MASK ((uint32_t)0x00000100u)
#define CSL_MCASP_PDCLR_AXR8_SHIFT ((uint32_t)0x00000008u)
#define CSL_MCASP_PDCLR_AXR8_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR8_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR8_CLEAR ((uint32_t)0x00000001u)

#define CSL_MCASP_PDCLR_AXR7_MASK ((uint32_t)0x00000080u)
#define CSL_MCASP_PDCLR_AXR7_SHIFT ((uint32_t)0x00000007u)
#define CSL_MCASP_PDCLR_AXR7_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR7_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR7_CLEAR ((uint32_t)0x00000001u)

#define CSL_MCASP_PDCLR_AXR6_MASK ((uint32_t)0x00000040u)
#define CSL_MCASP_PDCLR_AXR6_SHIFT ((uint32_t)0x00000006u)
#define CSL_MCASP_PDCLR_AXR6_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR6_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR6_CLEAR ((uint32_t)0x00000001u)

#define CSL_MCASP_PDCLR_AXR5_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_PDCLR_AXR5_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_PDCLR_AXR5_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR5_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR5_CLEAR ((uint32_t)0x00000001u)

#define CSL_MCASP_PDCLR_AXR4_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_PDCLR_AXR4_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_PDCLR_AXR4_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR4_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR4_CLEAR ((uint32_t)0x00000001u)

#define CSL_MCASP_PDCLR_AXR3_MASK ((uint32_t)0x00000008u)
#define CSL_MCASP_PDCLR_AXR3_SHIFT ((uint32_t)0x00000003u)
#define CSL_MCASP_PDCLR_AXR3_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR3_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR3_CLEAR ((uint32_t)0x00000001u)

#define CSL_MCASP_PDCLR_AXR2_MASK ((uint32_t)0x00000004u)
#define CSL_MCASP_PDCLR_AXR2_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_PDCLR_AXR2_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR2_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR2_CLEAR ((uint32_t)0x00000001u)

#define CSL_MCASP_PDCLR_AXR1_MASK ((uint32_t)0x00000002u)
#define CSL_MCASP_PDCLR_AXR1_SHIFT ((uint32_t)0x00000001u)
#define CSL_MCASP_PDCLR_AXR1_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR1_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR1_CLEAR ((uint32_t)0x00000001u)

#define CSL_MCASP_PDCLR_AXR0_MASK ((uint32_t)0x00000001u)
#define CSL_MCASP_PDCLR_AXR0_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR0_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR0_NOEFFECT ((uint32_t)0x00000000u)
#define CSL_MCASP_PDCLR_AXR0_CLEAR ((uint32_t)0x00000001u)

#define CSL_MCASP_PDCLR_RESETVAL ((uint32_t)0x00000000u)
/* GBLCTL */


#define CSL_MCASP_GBLCTL_XFRST_MASK ((uint32_t)0x00001000u)
#define CSL_MCASP_GBLCTL_XFRST_SHIFT ((uint32_t)0x0000000Cu)
#define CSL_MCASP_GBLCTL_XFRST_RESETVAL ((uint32_t)0x00000000u)
/*----XFRST Tokens----*/
#define CSL_MCASP_GBLCTL_XFRST_RESET ((uint32_t)0x00000000u)
#define CSL_MCASP_GBLCTL_XFRST_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_GBLCTL_XSMRST_MASK ((uint32_t)0x00000800u)
#define CSL_MCASP_GBLCTL_XSMRST_SHIFT ((uint32_t)0x0000000Bu)
#define CSL_MCASP_GBLCTL_XSMRST_RESETVAL ((uint32_t)0x00000000u)
/*----XSMRST Tokens----*/
#define CSL_MCASP_GBLCTL_XSMRST_RESET ((uint32_t)0x00000000u)
#define CSL_MCASP_GBLCTL_XSMRST_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_GBLCTL_XSRCLR_MASK ((uint32_t)0x00000400u)
#define CSL_MCASP_GBLCTL_XSRCLR_SHIFT ((uint32_t)0x0000000Au)
#define CSL_MCASP_GBLCTL_XSRCLR_RESETVAL ((uint32_t)0x00000000u)
/*----XSRCLR Tokens----*/
#define CSL_MCASP_GBLCTL_XSRCLR_CLEAR ((uint32_t)0x00000000u)
#define CSL_MCASP_GBLCTL_XSRCLR_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_GBLCTL_XHCLKRST_MASK ((uint32_t)0x00000200u)
#define CSL_MCASP_GBLCTL_XHCLKRST_SHIFT ((uint32_t)0x00000009u)
#define CSL_MCASP_GBLCTL_XHCLKRST_RESETVAL ((uint32_t)0x00000000u)
/*----XHCLKRST Tokens----*/
#define CSL_MCASP_GBLCTL_XHCLKRST_RESET ((uint32_t)0x00000000u)
#define CSL_MCASP_GBLCTL_XHCLKRST_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_GBLCTL_XCLKRST_MASK ((uint32_t)0x00000100u)
#define CSL_MCASP_GBLCTL_XCLKRST_SHIFT ((uint32_t)0x00000008u)
#define CSL_MCASP_GBLCTL_XCLKRST_RESETVAL ((uint32_t)0x00000000u)
/*----XCLKRST Tokens----*/
#define CSL_MCASP_GBLCTL_XCLKRST_RESET ((uint32_t)0x00000000u)
#define CSL_MCASP_GBLCTL_XCLKRST_ACTIVE ((uint32_t)0x00000001u)


#define CSL_MCASP_GBLCTL_RFRST_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_GBLCTL_RFRST_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_GBLCTL_RFRST_RESETVAL ((uint32_t)0x00000000u)
/*----RFRST Tokens----*/
#define CSL_MCASP_GBLCTL_RFRST_RESET ((uint32_t)0x00000000u)
#define CSL_MCASP_GBLCTL_RFRST_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_GBLCTL_RSMRST_MASK ((uint32_t)0x00000008u)
#define CSL_MCASP_GBLCTL_RSMRST_SHIFT ((uint32_t)0x00000003u)
#define CSL_MCASP_GBLCTL_RSMRST_RESETVAL ((uint32_t)0x00000000u)
/*----RSMRST Tokens----*/
#define CSL_MCASP_GBLCTL_RSMRST_RESET ((uint32_t)0x00000000u)
#define CSL_MCASP_GBLCTL_RSMRST_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_GBLCTL_RSRCLR_MASK ((uint32_t)0x00000004u)
#define CSL_MCASP_GBLCTL_RSRCLR_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_GBLCTL_RSRCLR_RESETVAL ((uint32_t)0x00000000u)
/*----RSRCLR Tokens----*/
#define CSL_MCASP_GBLCTL_RSRCLR_CLEAR ((uint32_t)0x00000000u)
#define CSL_MCASP_GBLCTL_RSRCLR_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_GBLCTL_RHCLKRST_MASK ((uint32_t)0x00000002u)
#define CSL_MCASP_GBLCTL_RHCLKRST_SHIFT ((uint32_t)0x00000001u)
#define CSL_MCASP_GBLCTL_RHCLKRST_RESETVAL ((uint32_t)0x00000000u)
/*----RHCLKRST Tokens----*/
#define CSL_MCASP_GBLCTL_RHCLKRST_RESET ((uint32_t)0x00000000u)
#define CSL_MCASP_GBLCTL_RHCLKRST_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_GBLCTL_RCLKRST_MASK ((uint32_t)0x00000001u)
#define CSL_MCASP_GBLCTL_RCLKRST_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_GBLCTL_RCLKRST_RESETVAL ((uint32_t)0x00000000u)
/*----RCLKRST Tokens----*/
#define CSL_MCASP_GBLCTL_RCLKRST_RESET ((uint32_t)0x00000000u)
#define CSL_MCASP_GBLCTL_RCLKRST_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_GBLCTL_RESETVAL ((uint32_t)0x00000000u)

/* AMUTE */


#define CSL_MCASP_AMUTE_XDMAERR_MASK ((uint32_t)0x00001000u)
#define CSL_MCASP_AMUTE_XDMAERR_SHIFT ((uint32_t)0x0000000Cu)
#define CSL_MCASP_AMUTE_XDMAERR_RESETVAL ((uint32_t)0x00000000u)
/*----XDMAERR Tokens----*/
#define CSL_MCASP_AMUTE_XDMAERR_DISABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_AMUTE_XDMAERR_ENABLE ((uint32_t)0x00000001u)

#define CSL_MCASP_AMUTE_RDMAERR_MASK ((uint32_t)0x00000800u)
#define CSL_MCASP_AMUTE_RDMAERR_SHIFT ((uint32_t)0x0000000Bu)
#define CSL_MCASP_AMUTE_RDMAERR_RESETVAL ((uint32_t)0x00000000u)
/*----RDMAERR Tokens----*/
#define CSL_MCASP_AMUTE_RDMAERR_DISABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_AMUTE_RDMAERR_ENABLE ((uint32_t)0x00000001u)

#define CSL_MCASP_AMUTE_XCKFAIL_MASK ((uint32_t)0x00000400u)
#define CSL_MCASP_AMUTE_XCKFAIL_SHIFT ((uint32_t)0x0000000Au)
#define CSL_MCASP_AMUTE_XCKFAIL_RESETVAL ((uint32_t)0x00000000u)
/*----XCKFAIL Tokens----*/
#define CSL_MCASP_AMUTE_XCKFAIL_DISABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_AMUTE_XCKFAIL_ENABLE ((uint32_t)0x00000001u)

#define CSL_MCASP_AMUTE_RCKFAIL_MASK ((uint32_t)0x00000200u)
#define CSL_MCASP_AMUTE_RCKFAIL_SHIFT ((uint32_t)0x00000009u)
#define CSL_MCASP_AMUTE_RCKFAIL_RESETVAL ((uint32_t)0x00000000u)
/*----RCKFAIL Tokens----*/
#define CSL_MCASP_AMUTE_RCKFAIL_DISABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_AMUTE_RCKFAIL_ENABLE ((uint32_t)0x00000001u)

#define CSL_MCASP_AMUTE_XSYNCERR_MASK ((uint32_t)0x00000100u)
#define CSL_MCASP_AMUTE_XSYNCERR_SHIFT ((uint32_t)0x00000008u)
#define CSL_MCASP_AMUTE_XSYNCERR_RESETVAL ((uint32_t)0x00000000u)
/*----XSYNCERR Tokens----*/
#define CSL_MCASP_AMUTE_XSYNCERR_DISABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_AMUTE_XSYNCERR_ENABLE ((uint32_t)0x00000001u)

#define CSL_MCASP_AMUTE_RSYNCERR_MASK ((uint32_t)0x00000080u)
#define CSL_MCASP_AMUTE_RSYNCERR_SHIFT ((uint32_t)0x00000007u)
#define CSL_MCASP_AMUTE_RSYNCERR_RESETVAL ((uint32_t)0x00000000u)
/*----RSYNCERR Tokens----*/
#define CSL_MCASP_AMUTE_RSYNCERR_DISABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_AMUTE_RSYNCERR_ENABLE ((uint32_t)0x00000001u)

#define CSL_MCASP_AMUTE_XUNDRN_MASK ((uint32_t)0x00000040u)
#define CSL_MCASP_AMUTE_XUNDRN_SHIFT ((uint32_t)0x00000006u)
#define CSL_MCASP_AMUTE_XUNDRN_RESETVAL ((uint32_t)0x00000000u)
/*----XUNDRN Tokens----*/
#define CSL_MCASP_AMUTE_XUNDRN_DISABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_AMUTE_XUNDRN_ENABLE ((uint32_t)0x00000001u)

#define CSL_MCASP_AMUTE_ROVRN_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_AMUTE_ROVRN_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_AMUTE_ROVRN_RESETVAL ((uint32_t)0x00000000u)
/*----ROVRN Tokens----*/
#define CSL_MCASP_AMUTE_ROVRN_DISABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_AMUTE_ROVRN_ENABLE ((uint32_t)0x00000001u)

#define CSL_MCASP_AMUTE_INSTAT_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_AMUTE_INSTAT_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_AMUTE_INSTAT_RESETVAL ((uint32_t)0x00000000u)
/*----INSTAT Tokens----*/
#define CSL_MCASP_AMUTE_INSTAT_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_AMUTE_INSTAT_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_AMUTE_INEN_MASK ((uint32_t)0x00000008u)
#define CSL_MCASP_AMUTE_INEN_SHIFT ((uint32_t)0x00000003u)
#define CSL_MCASP_AMUTE_INEN_RESETVAL ((uint32_t)0x00000000u)
/*----INEN Tokens----*/
#define CSL_MCASP_AMUTE_INEN_DISABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_AMUTE_INEN_ENABLE ((uint32_t)0x00000001u)

#define CSL_MCASP_AMUTE_INPOL_MASK ((uint32_t)0x00000004u)
#define CSL_MCASP_AMUTE_INPOL_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_AMUTE_INPOL_RESETVAL ((uint32_t)0x00000000u)
/*----INPOL Tokens----*/
#define CSL_MCASP_AMUTE_INPOL_ACTHIGH ((uint32_t)0x00000000u)
#define CSL_MCASP_AMUTE_INPOL_ACTLOW ((uint32_t)0x00000001u)

#define CSL_MCASP_AMUTE_MUTEN_MASK ((uint32_t)0x00000003u)
#define CSL_MCASP_AMUTE_MUTEN_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_AMUTE_MUTEN_RESETVAL ((uint32_t)0x00000000u)
/*----MUTEN Tokens----*/
#define CSL_MCASP_AMUTE_MUTEN_DISABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_AMUTE_MUTEN_ERRHIGH ((uint32_t)0x00000001u)
#define CSL_MCASP_AMUTE_MUTEN_ERRLOW ((uint32_t)0x00000002u)
#define CSL_MCASP_AMUTE_MUTEN_RESV ((uint32_t)0x00000003u)

#define CSL_MCASP_AMUTE_RESETVAL ((uint32_t)0x00000000u)

/* DLBCTL */


#define CSL_MCASP_DLBCTL_IOLBEN_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_DLBCTL_IOLBEN_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_DLBCTL_IOLBEN_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_DLBCTL_IOLBEN_DISABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_DLBCTL_IOLBEN_ENABLE ((uint32_t)0x00000001u)
#define CSL_MCASP_DLBCTL_MODE_MASK ((uint32_t)0x0000000Cu)
#define CSL_MCASP_DLBCTL_MODE_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_DLBCTL_MODE_RESETVAL ((uint32_t)0x00000000u)
/*----MODE Tokens----*/
#define CSL_MCASP_DLBCTL_MODE_RESERVED ((uint32_t)0x00000000u)
#define CSL_MCASP_DLBCTL_MODE_XMTCLK ((uint32_t)0x00000001u)
#define CSL_MCASP_DLBCTL_MODE_RESERVED1 ((uint32_t)0x00000002u)
#define CSL_MCASP_DLBCTL_MODE_RESERVED2 ((uint32_t)0x00000003u)

#define CSL_MCASP_DLBCTL_ORD_MASK ((uint32_t)0x00000002u)
#define CSL_MCASP_DLBCTL_ORD_SHIFT ((uint32_t)0x00000001u)
#define CSL_MCASP_DLBCTL_ORD_RESETVAL ((uint32_t)0x00000000u)
/*----ORD Tokens----*/
#define CSL_MCASP_DLBCTL_ORD_XMTODD ((uint32_t)0x00000000u)
#define CSL_MCASP_DLBCTL_ORD_XMTEVEN ((uint32_t)0x00000001u)

#define CSL_MCASP_DLBCTL_DLBEN_MASK ((uint32_t)0x00000001u)
#define CSL_MCASP_DLBCTL_DLBEN_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_DLBCTL_DLBEN_RESETVAL ((uint32_t)0x00000000u)
/*----DLBEN Tokens----*/
#define CSL_MCASP_DLBCTL_DLBEN_DISABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_DLBCTL_DLBEN_ENABLE ((uint32_t)0x00000001u)

#define CSL_MCASP_DLBCTL_RESETVAL ((uint32_t)0x00000000u)

/* DITCTL */


#define CSL_MCASP_DITCTL_VB_MASK ((uint32_t)0x00000008u)
#define CSL_MCASP_DITCTL_VB_SHIFT ((uint32_t)0x00000003u)
#define CSL_MCASP_DITCTL_VB_RESETVAL ((uint32_t)0x00000000u)
/*----VB Tokens----*/
#define CSL_MCASP_DITCTL_VB_ZERO ((uint32_t)0x00000000u)
#define CSL_MCASP_DITCTL_VB_ONE ((uint32_t)0x00000001u)

#define CSL_MCASP_DITCTL_VA_MASK ((uint32_t)0x00000004u)
#define CSL_MCASP_DITCTL_VA_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_DITCTL_VA_RESETVAL ((uint32_t)0x00000000u)
/*----VA Tokens----*/
#define CSL_MCASP_DITCTL_VA_ZERO ((uint32_t)0x00000000u)
#define CSL_MCASP_DITCTL_VA_ONE ((uint32_t)0x00000001u)


#define CSL_MCASP_DITCTL_DITEN_MASK ((uint32_t)0x00000001u)
#define CSL_MCASP_DITCTL_DITEN_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_DITCTL_DITEN_RESETVAL ((uint32_t)0x00000000u)
/*----DITEN Tokens----*/
#define CSL_MCASP_DITCTL_DITEN_DISABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_DITCTL_DITEN_ENABLE ((uint32_t)0x00000001u)

#define CSL_MCASP_DITCTL_RESETVAL ((uint32_t)0x00000000u)

/* RGBLCTL */


#define CSL_MCASP_RGBLCTL_XFRST_MASK ((uint32_t)0x00001000u)
#define CSL_MCASP_RGBLCTL_XFRST_SHIFT ((uint32_t)0x0000000Cu)
#define CSL_MCASP_RGBLCTL_XFRST_RESETVAL ((uint32_t)0x00000000u)
/*----XFRST Tokens----*/
#define CSL_MCASP_RGBLCTL_XFRST_RESET ((uint32_t)0x00000000u)
#define CSL_MCASP_RGBLCTL_XFRST_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RGBLCTL_XSMRST_MASK ((uint32_t)0x00000800u)
#define CSL_MCASP_RGBLCTL_XSMRST_SHIFT ((uint32_t)0x0000000Bu)
#define CSL_MCASP_RGBLCTL_XSMRST_RESETVAL ((uint32_t)0x00000000u)
/*----XSMRST Tokens----*/
#define CSL_MCASP_RGBLCTL_XSMRST_RESET ((uint32_t)0x00000000u)
#define CSL_MCASP_RGBLCTL_XSMRST_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RGBLCTL_XSRCLR_MASK ((uint32_t)0x00000400u)
#define CSL_MCASP_RGBLCTL_XSRCLR_SHIFT ((uint32_t)0x0000000Au)
#define CSL_MCASP_RGBLCTL_XSRCLR_RESETVAL ((uint32_t)0x00000000u)
/*----XSRCLR Tokens----*/
#define CSL_MCASP_RGBLCTL_XSRCLR_CLEAR ((uint32_t)0x00000000u)
#define CSL_MCASP_RGBLCTL_XSRCLR_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RGBLCTL_XHCLKRST_MASK ((uint32_t)0x00000200u)
#define CSL_MCASP_RGBLCTL_XHCLKRST_SHIFT ((uint32_t)0x00000009u)
#define CSL_MCASP_RGBLCTL_XHCLKRST_RESETVAL ((uint32_t)0x00000000u)
/*----XHCLKRST Tokens----*/
#define CSL_MCASP_RGBLCTL_XHCLKRST_RESET ((uint32_t)0x00000000u)
#define CSL_MCASP_RGBLCTL_XHCLKRST_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RGBLCTL_XCLKRST_MASK ((uint32_t)0x00000100u)
#define CSL_MCASP_RGBLCTL_XCLKRST_SHIFT ((uint32_t)0x00000008u)
#define CSL_MCASP_RGBLCTL_XCLKRST_RESETVAL ((uint32_t)0x00000000u)
/*----XCLKRST Tokens----*/
#define CSL_MCASP_RGBLCTL_XCLKRST_RESET ((uint32_t)0x00000000u)
#define CSL_MCASP_RGBLCTL_XCLKRST_ACTIVE ((uint32_t)0x00000001u)


#define CSL_MCASP_RGBLCTL_RFRST_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_RGBLCTL_RFRST_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_RGBLCTL_RFRST_RESETVAL ((uint32_t)0x00000000u)
/*----RFRST Tokens----*/
#define CSL_MCASP_RGBLCTL_RFRST_RESET ((uint32_t)0x00000000u)
#define CSL_MCASP_RGBLCTL_RFRST_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RGBLCTL_RSMRST_MASK ((uint32_t)0x00000008u)
#define CSL_MCASP_RGBLCTL_RSMRST_SHIFT ((uint32_t)0x00000003u)
#define CSL_MCASP_RGBLCTL_RSMRST_RESETVAL ((uint32_t)0x00000000u)
/*----RSMRST Tokens----*/
#define CSL_MCASP_RGBLCTL_RSMRST_RESET ((uint32_t)0x00000000u)
#define CSL_MCASP_RGBLCTL_RSMRST_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RGBLCTL_RSRCLR_MASK ((uint32_t)0x00000004u)
#define CSL_MCASP_RGBLCTL_RSRCLR_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_RGBLCTL_RSRCLR_RESETVAL ((uint32_t)0x00000000u)
/*----RSRCLR Tokens----*/
#define CSL_MCASP_RGBLCTL_RSRCLR_CLEAR ((uint32_t)0x00000000u)
#define CSL_MCASP_RGBLCTL_RSRCLR_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RGBLCTL_RHCLKRST_MASK ((uint32_t)0x00000002u)
#define CSL_MCASP_RGBLCTL_RHCLKRST_SHIFT ((uint32_t)0x00000001u)
#define CSL_MCASP_RGBLCTL_RHCLKRST_RESETVAL ((uint32_t)0x00000000u)
/*----RHCLKRST Tokens----*/
#define CSL_MCASP_RGBLCTL_RHCLKRST_RESET ((uint32_t)0x00000000u)
#define CSL_MCASP_RGBLCTL_RHCLKRST_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RGBLCTL_RCLKRST_MASK ((uint32_t)0x00000001u)
#define CSL_MCASP_RGBLCTL_RCLKRST_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_RGBLCTL_RCLKRST_RESETVAL ((uint32_t)0x00000000u)
/*----RCLKRST Tokens----*/
#define CSL_MCASP_RGBLCTL_RCLKRST_RESET ((uint32_t)0x00000000u)
#define CSL_MCASP_RGBLCTL_RCLKRST_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RGBLCTL_RESETVAL ((uint32_t)0x00000000u)

/* RMASK */

#define CSL_MCASP_RMASK_RMASK31_MASK ((uint32_t)0x80000000u)
#define CSL_MCASP_RMASK_RMASK31_SHIFT ((uint32_t)0x0000001Fu)
#define CSL_MCASP_RMASK_RMASK31_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK31_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK31_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK30_MASK ((uint32_t)0x40000000u)
#define CSL_MCASP_RMASK_RMASK30_SHIFT ((uint32_t)0x0000001Eu)
#define CSL_MCASP_RMASK_RMASK30_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK30_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK30_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK29_MASK ((uint32_t)0x20000000u)
#define CSL_MCASP_RMASK_RMASK29_SHIFT ((uint32_t)0x0000001Du)
#define CSL_MCASP_RMASK_RMASK29_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK29_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK29_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK28_MASK ((uint32_t)0x10000000u)
#define CSL_MCASP_RMASK_RMASK28_SHIFT ((uint32_t)0x0000001Cu)
#define CSL_MCASP_RMASK_RMASK28_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK28_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK28_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK27_MASK ((uint32_t)0x08000000u)
#define CSL_MCASP_RMASK_RMASK27_SHIFT ((uint32_t)0x0000001Bu)
#define CSL_MCASP_RMASK_RMASK27_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK27_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK27_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK26_MASK ((uint32_t)0x04000000u)
#define CSL_MCASP_RMASK_RMASK26_SHIFT ((uint32_t)0x0000001Au)
#define CSL_MCASP_RMASK_RMASK26_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK26_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK26_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK25_MASK ((uint32_t)0x02000000u)
#define CSL_MCASP_RMASK_RMASK25_SHIFT ((uint32_t)0x00000019u)
#define CSL_MCASP_RMASK_RMASK25_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK25_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK25_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK24_MASK ((uint32_t)0x01000000u)
#define CSL_MCASP_RMASK_RMASK24_SHIFT ((uint32_t)0x00000018u)
#define CSL_MCASP_RMASK_RMASK24_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK24_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK24_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK23_MASK ((uint32_t)0x00800000u)
#define CSL_MCASP_RMASK_RMASK23_SHIFT ((uint32_t)0x00000017u)
#define CSL_MCASP_RMASK_RMASK23_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK23_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK23_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK22_MASK ((uint32_t)0x00400000u)
#define CSL_MCASP_RMASK_RMASK22_SHIFT ((uint32_t)0x00000016u)
#define CSL_MCASP_RMASK_RMASK22_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK22_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK22_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK21_MASK ((uint32_t)0x00200000u)
#define CSL_MCASP_RMASK_RMASK21_SHIFT ((uint32_t)0x00000015u)
#define CSL_MCASP_RMASK_RMASK21_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK21_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK21_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK20_MASK ((uint32_t)0x00100000u)
#define CSL_MCASP_RMASK_RMASK20_SHIFT ((uint32_t)0x00000014u)
#define CSL_MCASP_RMASK_RMASK20_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK20_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK20_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK19_MASK ((uint32_t)0x00080000u)
#define CSL_MCASP_RMASK_RMASK19_SHIFT ((uint32_t)0x00000013u)
#define CSL_MCASP_RMASK_RMASK19_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK19_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK19_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK18_MASK ((uint32_t)0x00040000u)
#define CSL_MCASP_RMASK_RMASK18_SHIFT ((uint32_t)0x00000012u)
#define CSL_MCASP_RMASK_RMASK18_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK18_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK18_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK17_MASK ((uint32_t)0x00020000u)
#define CSL_MCASP_RMASK_RMASK17_SHIFT ((uint32_t)0x00000011u)
#define CSL_MCASP_RMASK_RMASK17_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK17_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK17_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK16_MASK ((uint32_t)0x00010000u)
#define CSL_MCASP_RMASK_RMASK16_SHIFT ((uint32_t)0x00000010u)
#define CSL_MCASP_RMASK_RMASK16_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK16_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK16_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK15_MASK ((uint32_t)0x00008000u)
#define CSL_MCASP_RMASK_RMASK15_SHIFT ((uint32_t)0x0000000Fu)
#define CSL_MCASP_RMASK_RMASK15_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK15_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK15_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK14_MASK ((uint32_t)0x00004000u)
#define CSL_MCASP_RMASK_RMASK14_SHIFT ((uint32_t)0x0000000Eu)
#define CSL_MCASP_RMASK_RMASK14_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK14_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK14_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK13_MASK ((uint32_t)0x00002000u)
#define CSL_MCASP_RMASK_RMASK13_SHIFT ((uint32_t)0x0000000Du)
#define CSL_MCASP_RMASK_RMASK13_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK13_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK13_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK12_MASK ((uint32_t)0x00001000u)
#define CSL_MCASP_RMASK_RMASK12_SHIFT ((uint32_t)0x0000000Cu)
#define CSL_MCASP_RMASK_RMASK12_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK12_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK12_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK11_MASK ((uint32_t)0x00000800u)
#define CSL_MCASP_RMASK_RMASK11_SHIFT ((uint32_t)0x0000000Bu)
#define CSL_MCASP_RMASK_RMASK11_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK11_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK11_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK10_MASK ((uint32_t)0x00000400u)
#define CSL_MCASP_RMASK_RMASK10_SHIFT ((uint32_t)0x0000000Au)
#define CSL_MCASP_RMASK_RMASK10_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK10_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK10_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK9_MASK ((uint32_t)0x00000200u)
#define CSL_MCASP_RMASK_RMASK9_SHIFT ((uint32_t)0x00000009u)
#define CSL_MCASP_RMASK_RMASK9_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK9_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK9_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK8_MASK ((uint32_t)0x00000100u)
#define CSL_MCASP_RMASK_RMASK8_SHIFT ((uint32_t)0x00000008u)
#define CSL_MCASP_RMASK_RMASK8_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK8_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK8_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK7_MASK ((uint32_t)0x00000080u)
#define CSL_MCASP_RMASK_RMASK7_SHIFT ((uint32_t)0x00000007u)
#define CSL_MCASP_RMASK_RMASK7_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK7_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK7_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK6_MASK ((uint32_t)0x00000040u)
#define CSL_MCASP_RMASK_RMASK6_SHIFT ((uint32_t)0x00000006u)
#define CSL_MCASP_RMASK_RMASK6_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK6_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK6_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK5_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_RMASK_RMASK5_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_RMASK_RMASK5_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK5_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK5_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK4_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_RMASK_RMASK4_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_RMASK_RMASK4_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK4_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK4_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK3_MASK ((uint32_t)0x00000008u)
#define CSL_MCASP_RMASK_RMASK3_SHIFT ((uint32_t)0x00000003u)
#define CSL_MCASP_RMASK_RMASK3_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK3_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK3_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK2_MASK ((uint32_t)0x00000004u)
#define CSL_MCASP_RMASK_RMASK2_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_RMASK_RMASK2_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK2_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK2_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK1_MASK ((uint32_t)0x00000002u)
#define CSL_MCASP_RMASK_RMASK1_SHIFT ((uint32_t)0x00000001u)
#define CSL_MCASP_RMASK_RMASK1_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK1_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK1_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RMASK0_MASK ((uint32_t)0x00000001u)
#define CSL_MCASP_RMASK_RMASK0_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK0_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK0_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_RMASK_RMASK0_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_RMASK_RESETVAL ((uint32_t)0x00000000u)

/* RFMT */


#define CSL_MCASP_RFMT_RDATDLY_MASK ((uint32_t)0x00030000u)
#define CSL_MCASP_RFMT_RDATDLY_SHIFT ((uint32_t)0x00000010u)
#define CSL_MCASP_RFMT_RDATDLY_RESETVAL ((uint32_t)0x00000000u)
/*----RDATDLY Tokens----*/
#define CSL_MCASP_RFMT_RDATDLY_0BIT ((uint32_t)0x00000000u)
#define CSL_MCASP_RFMT_RDATDLY_1BIT ((uint32_t)0x00000001u)
#define CSL_MCASP_RFMT_RDATDLY_2BIT ((uint32_t)0x00000002u)
#define CSL_MCASP_RFMT_RDATDLY_RESV ((uint32_t)0x00000003u)

#define CSL_MCASP_RFMT_RRVRS_MASK ((uint32_t)0x00008000u)
#define CSL_MCASP_RFMT_RRVRS_SHIFT ((uint32_t)0x0000000Fu)
#define CSL_MCASP_RFMT_RRVRS_RESETVAL ((uint32_t)0x00000000u)
/*----RRVRS Tokens----*/
#define CSL_MCASP_RFMT_RRVRS_LSBFIRST ((uint32_t)0x00000000u)
#define CSL_MCASP_RFMT_RRVRS_MSBFIRST ((uint32_t)0x00000001u)

#define CSL_MCASP_RFMT_RPAD_MASK ((uint32_t)0x00006000u)
#define CSL_MCASP_RFMT_RPAD_SHIFT ((uint32_t)0x0000000Du)
#define CSL_MCASP_RFMT_RPAD_RESETVAL ((uint32_t)0x00000000u)
/*----RPAD Tokens----*/
#define CSL_MCASP_RFMT_RPAD_ZERO ((uint32_t)0x00000000u)
#define CSL_MCASP_RFMT_RPAD_ONE ((uint32_t)0x00000001u)
#define CSL_MCASP_RFMT_RPAD_RPBIT ((uint32_t)0x00000002u)
#define CSL_MCASP_RFMT_RPAD_RESV ((uint32_t)0x00000003u)

#define CSL_MCASP_RFMT_RPBIT_MASK ((uint32_t)0x00001F00u)
#define CSL_MCASP_RFMT_RPBIT_SHIFT ((uint32_t)0x00000008u)
#define CSL_MCASP_RFMT_RPBIT_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_RFMT_RSSZ_MASK ((uint32_t)0x000000F0u)
#define CSL_MCASP_RFMT_RSSZ_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_RFMT_RSSZ_RESETVAL ((uint32_t)0x00000000u)
/*----RSSZ Tokens----*/
#define CSL_MCASP_RFMT_RSSZ_RSV ((uint32_t)0x00000000u)
#define CSL_MCASP_RFMT_RSSZ_RSV1 ((uint32_t)0x00000001u)
#define CSL_MCASP_RFMT_RSSZ_RSV2 ((uint32_t)0x00000002u)
#define CSL_MCASP_RFMT_RSSZ_8BITS ((uint32_t)0x00000003u)
#define CSL_MCASP_RFMT_RSSZ_RSV3 ((uint32_t)0x00000004u)
#define CSL_MCASP_RFMT_RSSZ_12BITS ((uint32_t)0x00000005u)
#define CSL_MCASP_RFMT_RSSZ_RSV4 ((uint32_t)0x00000006u)
#define CSL_MCASP_RFMT_RSSZ_16BITS ((uint32_t)0x00000007u)
#define CSL_MCASP_RFMT_RSSZ_RSV5 ((uint32_t)0x00000008u)
#define CSL_MCASP_RFMT_RSSZ_20BITS ((uint32_t)0x00000009u)
#define CSL_MCASP_RFMT_RSSZ_RSV6 ((uint32_t)0x0000000Au)
#define CSL_MCASP_RFMT_RSSZ_24BITS ((uint32_t)0x0000000Bu)
#define CSL_MCASP_RFMT_RSSZ_RSV7 ((uint32_t)0x0000000Cu)
#define CSL_MCASP_RFMT_RSSZ_28BITS ((uint32_t)0x0000000Du)
#define CSL_MCASP_RFMT_RSSZ_RSV8 ((uint32_t)0x0000000Eu)
#define CSL_MCASP_RFMT_RSSZ_32BITS ((uint32_t)0x0000000Fu)

#define CSL_MCASP_RFMT_RBUSEL_MASK ((uint32_t)0x00000008u)
#define CSL_MCASP_RFMT_RBUSEL_SHIFT ((uint32_t)0x00000003u)
#define CSL_MCASP_RFMT_RBUSEL_RESETVAL ((uint32_t)0x00000000u)
/*----RBUSEL Tokens----*/
#define CSL_MCASP_RFMT_RBUSEL_VBUSP ((uint32_t)0x00000000u)
#define CSL_MCASP_RFMT_RBUSEL_VBUS ((uint32_t)0x00000001u)

#define CSL_MCASP_RFMT_RROT_MASK ((uint32_t)0x00000007u)
#define CSL_MCASP_RFMT_RROT_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_RFMT_RROT_RESETVAL ((uint32_t)0x00000000u)
/*----RROT Tokens----*/
#define CSL_MCASP_RFMT_RROT_NONE ((uint32_t)0x00000000u)
#define CSL_MCASP_RFMT_RROT_4BITS ((uint32_t)0x00000001u)
#define CSL_MCASP_RFMT_RROT_8BITS ((uint32_t)0x00000002u)
#define CSL_MCASP_RFMT_RROT_12BITS ((uint32_t)0x00000003u)
#define CSL_MCASP_RFMT_RROT_16BITS ((uint32_t)0x00000004u)
#define CSL_MCASP_RFMT_RROT_20BITS ((uint32_t)0x00000005u)
#define CSL_MCASP_RFMT_RROT_24BITS ((uint32_t)0x00000006u)
#define CSL_MCASP_RFMT_RROT_28BITS ((uint32_t)0x00000007u)

#define CSL_MCASP_RFMT_RESETVAL ((uint32_t)0x00000000u)

/* AFSRCTL */


#define CSL_MCASP_AFSRCTL_RMOD_MASK ((uint32_t)0x0000FF80u)
#define CSL_MCASP_AFSRCTL_RMOD_SHIFT ((uint32_t)0x00000007u)
#define CSL_MCASP_AFSRCTL_RMOD_RESETVAL ((uint32_t)0x00000000u)
/*----RMOD Tokens----*/
#define CSL_MCASP_AFSRCTL_RMOD_BURST ((uint32_t)0x00000000u)
#define CSL_MCASP_AFSRCTL_RMOD_I2S ((uint32_t)0x00000002u)


#define CSL_MCASP_AFSRCTL_FRWID_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_AFSRCTL_FRWID_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_AFSRCTL_FRWID_RESETVAL ((uint32_t)0x00000000u)
/*----FRWID Tokens----*/
#define CSL_MCASP_AFSRCTL_FRWID_BIT ((uint32_t)0x00000000u)
#define CSL_MCASP_AFSRCTL_FRWID_WORD ((uint32_t)0x00000001u)


#define CSL_MCASP_AFSRCTL_FSRM_MASK ((uint32_t)0x00000002u)
#define CSL_MCASP_AFSRCTL_FSRM_SHIFT ((uint32_t)0x00000001u)
#define CSL_MCASP_AFSRCTL_FSRM_RESETVAL ((uint32_t)0x00000000u)
/*----FSRM Tokens----*/
#define CSL_MCASP_AFSRCTL_FSRM_EXTERNAL ((uint32_t)0x00000000u)
#define CSL_MCASP_AFSRCTL_FSRM_INTERNAL ((uint32_t)0x00000001u)

#define CSL_MCASP_AFSRCTL_FSRP_MASK ((uint32_t)0x00000001u)
#define CSL_MCASP_AFSRCTL_FSRP_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_AFSRCTL_FSRP_RESETVAL ((uint32_t)0x00000000u)
/*----FSRP Tokens----*/
#define CSL_MCASP_AFSRCTL_FSRP_RISINGEDGE ((uint32_t)0x00000000u)
#define CSL_MCASP_AFSRCTL_FSRP_FALLINGEDGE ((uint32_t)0x00000001u)

#define CSL_MCASP_AFSRCTL_RESETVAL ((uint32_t)0x00000000u)

/* ACLKRCTL */


#define CSL_MCASP_ACLKRCTL_CLKRP_MASK ((uint32_t)0x00000080u)
#define CSL_MCASP_ACLKRCTL_CLKRP_SHIFT ((uint32_t)0x00000007u)
#define CSL_MCASP_ACLKRCTL_CLKRP_RESETVAL ((uint32_t)0x00000000u)
/*----CLKRP Tokens----*/
#define CSL_MCASP_ACLKRCTL_CLKRP_FALLINGEDGE ((uint32_t)0x00000000u)
#define CSL_MCASP_ACLKRCTL_CLKRP_RISINGEDGE ((uint32_t)0x00000001u)


#define CSL_MCASP_ACLKRCTL_CLKRM_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_ACLKRCTL_CLKRM_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_ACLKRCTL_CLKRM_RESETVAL ((uint32_t)0x00000001u)
/*----CLKRM Tokens----*/
#define CSL_MCASP_ACLKRCTL_CLKRM_EXTERNAL ((uint32_t)0x00000000u)
#define CSL_MCASP_ACLKRCTL_CLKRM_INTERNAL ((uint32_t)0x00000001u)

#define CSL_MCASP_ACLKRCTL_CLKRDIV_MASK ((uint32_t)0x0000001Fu)
#define CSL_MCASP_ACLKRCTL_CLKRDIV_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_ACLKRCTL_CLKRDIV_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_ACLKRCTL_RESETVAL ((uint32_t)0x00000020u)

/* AHCLKRCTL */


#define CSL_MCASP_AHCLKRCTL_HCLKRM_MASK ((uint32_t)0x00008000u)
#define CSL_MCASP_AHCLKRCTL_HCLKRM_SHIFT ((uint32_t)0x0000000Fu)
#define CSL_MCASP_AHCLKRCTL_HCLKRM_RESETVAL ((uint32_t)0x00000001u)
/*----HCLKRM Tokens----*/
#define CSL_MCASP_AHCLKRCTL_HCLKRM_EXTERNAL ((uint32_t)0x00000000u)
#define CSL_MCASP_AHCLKRCTL_HCLKRM_INTERNAL ((uint32_t)0x00000001u)

#define CSL_MCASP_AHCLKRCTL_HCLKRP_MASK ((uint32_t)0x00004000u)
#define CSL_MCASP_AHCLKRCTL_HCLKRP_SHIFT ((uint32_t)0x0000000Eu)
#define CSL_MCASP_AHCLKRCTL_HCLKRP_RESETVAL ((uint32_t)0x00000000u)
/*----HCLKRP Tokens----*/
#define CSL_MCASP_AHCLKRCTL_HCLKRP_NOTINVERTED ((uint32_t)0x00000000u)
#define CSL_MCASP_AHCLKRCTL_HCLKRP_INVERTED ((uint32_t)0x00000001u)


#define CSL_MCASP_AHCLKRCTL_HCLKRDIV_MASK ((uint32_t)0x00000FFFu)
#define CSL_MCASP_AHCLKRCTL_HCLKRDIV_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_AHCLKRCTL_HCLKRDIV_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_AHCLKRCTL_RESETVAL ((uint32_t)0x00008000u)

/* RTDM */

#define CSL_MCASP_RTDM_RTDMS31_MASK ((uint32_t)0x80000000u)
#define CSL_MCASP_RTDM_RTDMS31_SHIFT ((uint32_t)0x0000001Fu)
#define CSL_MCASP_RTDM_RTDMS31_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS31_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS31_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS30_MASK ((uint32_t)0x40000000u)
#define CSL_MCASP_RTDM_RTDMS30_SHIFT ((uint32_t)0x0000001Eu)
#define CSL_MCASP_RTDM_RTDMS30_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS30_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS30_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS29_MASK ((uint32_t)0x20000000u)
#define CSL_MCASP_RTDM_RTDMS29_SHIFT ((uint32_t)0x0000001Du)
#define CSL_MCASP_RTDM_RTDMS29_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS29_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS29_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS28_MASK ((uint32_t)0x10000000u)
#define CSL_MCASP_RTDM_RTDMS28_SHIFT ((uint32_t)0x0000001Cu)
#define CSL_MCASP_RTDM_RTDMS28_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS28_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS28_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS27_MASK ((uint32_t)0x08000000u)
#define CSL_MCASP_RTDM_RTDMS27_SHIFT ((uint32_t)0x0000001Bu)
#define CSL_MCASP_RTDM_RTDMS27_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS27_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS27_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS26_MASK ((uint32_t)0x04000000u)
#define CSL_MCASP_RTDM_RTDMS26_SHIFT ((uint32_t)0x0000001Au)
#define CSL_MCASP_RTDM_RTDMS26_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS26_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS26_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS25_MASK ((uint32_t)0x02000000u)
#define CSL_MCASP_RTDM_RTDMS25_SHIFT ((uint32_t)0x00000019u)
#define CSL_MCASP_RTDM_RTDMS25_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS25_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS25_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS24_MASK ((uint32_t)0x01000000u)
#define CSL_MCASP_RTDM_RTDMS24_SHIFT ((uint32_t)0x00000018u)
#define CSL_MCASP_RTDM_RTDMS24_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS24_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS24_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS23_MASK ((uint32_t)0x00800000u)
#define CSL_MCASP_RTDM_RTDMS23_SHIFT ((uint32_t)0x00000017u)
#define CSL_MCASP_RTDM_RTDMS23_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS23_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS23_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS22_MASK ((uint32_t)0x00400000u)
#define CSL_MCASP_RTDM_RTDMS22_SHIFT ((uint32_t)0x00000016u)
#define CSL_MCASP_RTDM_RTDMS22_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS22_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS22_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS21_MASK ((uint32_t)0x00200000u)
#define CSL_MCASP_RTDM_RTDMS21_SHIFT ((uint32_t)0x00000015u)
#define CSL_MCASP_RTDM_RTDMS21_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS21_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS21_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS20_MASK ((uint32_t)0x00100000u)
#define CSL_MCASP_RTDM_RTDMS20_SHIFT ((uint32_t)0x00000014u)
#define CSL_MCASP_RTDM_RTDMS20_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS20_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS20_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS19_MASK ((uint32_t)0x00080000u)
#define CSL_MCASP_RTDM_RTDMS19_SHIFT ((uint32_t)0x00000013u)
#define CSL_MCASP_RTDM_RTDMS19_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS19_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS19_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS18_MASK ((uint32_t)0x00040000u)
#define CSL_MCASP_RTDM_RTDMS18_SHIFT ((uint32_t)0x00000012u)
#define CSL_MCASP_RTDM_RTDMS18_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS18_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS18_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS17_MASK ((uint32_t)0x00020000u)
#define CSL_MCASP_RTDM_RTDMS17_SHIFT ((uint32_t)0x00000011u)
#define CSL_MCASP_RTDM_RTDMS17_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS17_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS17_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS16_MASK ((uint32_t)0x00010000u)
#define CSL_MCASP_RTDM_RTDMS16_SHIFT ((uint32_t)0x00000010u)
#define CSL_MCASP_RTDM_RTDMS16_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS16_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS16_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS15_MASK ((uint32_t)0x00008000u)
#define CSL_MCASP_RTDM_RTDMS15_SHIFT ((uint32_t)0x0000000Fu)
#define CSL_MCASP_RTDM_RTDMS15_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS15_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS15_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS14_MASK ((uint32_t)0x00004000u)
#define CSL_MCASP_RTDM_RTDMS14_SHIFT ((uint32_t)0x0000000Eu)
#define CSL_MCASP_RTDM_RTDMS14_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS14_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS14_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS13_MASK ((uint32_t)0x00002000u)
#define CSL_MCASP_RTDM_RTDMS13_SHIFT ((uint32_t)0x0000000Du)
#define CSL_MCASP_RTDM_RTDMS13_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS13_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS13_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS12_MASK ((uint32_t)0x00001000u)
#define CSL_MCASP_RTDM_RTDMS12_SHIFT ((uint32_t)0x0000000Cu)
#define CSL_MCASP_RTDM_RTDMS12_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS12_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS12_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS11_MASK ((uint32_t)0x00000800u)
#define CSL_MCASP_RTDM_RTDMS11_SHIFT ((uint32_t)0x0000000Bu)
#define CSL_MCASP_RTDM_RTDMS11_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS11_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS11_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS10_MASK ((uint32_t)0x00000400u)
#define CSL_MCASP_RTDM_RTDMS10_SHIFT ((uint32_t)0x0000000Au)
#define CSL_MCASP_RTDM_RTDMS10_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS10_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS10_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS9_MASK ((uint32_t)0x00000200u)
#define CSL_MCASP_RTDM_RTDMS9_SHIFT ((uint32_t)0x00000009u)
#define CSL_MCASP_RTDM_RTDMS9_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS9_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS9_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS8_MASK ((uint32_t)0x00000100u)
#define CSL_MCASP_RTDM_RTDMS8_SHIFT ((uint32_t)0x00000008u)
#define CSL_MCASP_RTDM_RTDMS8_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS8_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS8_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS7_MASK ((uint32_t)0x00000080u)
#define CSL_MCASP_RTDM_RTDMS7_SHIFT ((uint32_t)0x00000007u)
#define CSL_MCASP_RTDM_RTDMS7_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS7_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS7_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS6_MASK ((uint32_t)0x00000040u)
#define CSL_MCASP_RTDM_RTDMS6_SHIFT ((uint32_t)0x00000006u)
#define CSL_MCASP_RTDM_RTDMS6_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS6_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS6_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS5_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_RTDM_RTDMS5_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_RTDM_RTDMS5_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS5_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS5_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS4_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_RTDM_RTDMS4_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_RTDM_RTDMS4_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS4_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS4_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS3_MASK ((uint32_t)0x00000008u)
#define CSL_MCASP_RTDM_RTDMS3_SHIFT ((uint32_t)0x00000003u)
#define CSL_MCASP_RTDM_RTDMS3_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS3_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS3_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS2_MASK ((uint32_t)0x00000004u)
#define CSL_MCASP_RTDM_RTDMS2_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_RTDM_RTDMS2_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS2_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS2_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS1_MASK ((uint32_t)0x00000002u)
#define CSL_MCASP_RTDM_RTDMS1_SHIFT ((uint32_t)0x00000001u)
#define CSL_MCASP_RTDM_RTDMS1_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS1_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS1_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RTDMS0_MASK ((uint32_t)0x00000001u)
#define CSL_MCASP_RTDM_RTDMS0_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS0_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS0_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_RTDM_RTDMS0_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_RTDM_RESETVAL ((uint32_t)0x00000000u)

/* RINTCTL */


#define CSL_MCASP_RINTCTL_RSTAFRM_MASK ((uint32_t)0x00000080u)
#define CSL_MCASP_RINTCTL_RSTAFRM_SHIFT ((uint32_t)0x00000007u)
#define CSL_MCASP_RINTCTL_RSTAFRM_RESETVAL ((uint32_t)0x00000000u)
/*----RSTAFRM Tokens----*/
#define CSL_MCASP_RINTCTL_RSTAFRM_DISABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_RINTCTL_RSTAFRM_ENABLE ((uint32_t)0x00000001u)


#define CSL_MCASP_RINTCTL_RDATA_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_RINTCTL_RDATA_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_RINTCTL_RDATA_RESETVAL ((uint32_t)0x00000000u)
/*----RDATA Tokens----*/
#define CSL_MCASP_RINTCTL_RDATA_DISABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_RINTCTL_RDATA_ENABLE ((uint32_t)0x00000001u)

#define CSL_MCASP_RINTCTL_RLAST_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_RINTCTL_RLAST_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_RINTCTL_RLAST_RESETVAL ((uint32_t)0x00000000u)
/*----RLAST Tokens----*/
#define CSL_MCASP_RINTCTL_RLAST_DISABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_RINTCTL_RLAST_ENABLE ((uint32_t)0x00000001u)

#define CSL_MCASP_RINTCTL_RDMAERR_MASK ((uint32_t)0x00000008u)
#define CSL_MCASP_RINTCTL_RDMAERR_SHIFT ((uint32_t)0x00000003u)
#define CSL_MCASP_RINTCTL_RDMAERR_RESETVAL ((uint32_t)0x00000000u)
/*----RDMAERR Tokens----*/
#define CSL_MCASP_RINTCTL_RDMAERR_DISABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_RINTCTL_RDMAERR_ENABLE ((uint32_t)0x00000001u)

#define CSL_MCASP_RINTCTL_RCKFAIL_MASK ((uint32_t)0x00000004u)
#define CSL_MCASP_RINTCTL_RCKFAIL_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_RINTCTL_RCKFAIL_RESETVAL ((uint32_t)0x00000000u)
/*----RCKFAIL Tokens----*/
#define CSL_MCASP_RINTCTL_RCKFAIL_DISABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_RINTCTL_RCKFAIL_ENABLE ((uint32_t)0x00000001u)

#define CSL_MCASP_RINTCTL_RSYNCERR_MASK ((uint32_t)0x00000002u)
#define CSL_MCASP_RINTCTL_RSYNCERR_SHIFT ((uint32_t)0x00000001u)
#define CSL_MCASP_RINTCTL_RSYNCERR_RESETVAL ((uint32_t)0x00000000u)
/*----RSYNCERR Tokens----*/
#define CSL_MCASP_RINTCTL_RSYNCERR_DISABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_RINTCTL_RSYNCERR_ENABLE ((uint32_t)0x00000001u)

#define CSL_MCASP_RINTCTL_ROVRN_MASK ((uint32_t)0x00000001u)
#define CSL_MCASP_RINTCTL_ROVRN_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_RINTCTL_ROVRN_RESETVAL ((uint32_t)0x00000000u)
/*----ROVRN Tokens----*/
#define CSL_MCASP_RINTCTL_ROVRN_DISABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_RINTCTL_ROVRN_ENABLE ((uint32_t)0x00000001u)

#define CSL_MCASP_RINTCTL_RESETVAL ((uint32_t)0x00000000u)

/* RSTAT */


#define CSL_MCASP_RSTAT_RERR_MASK ((uint32_t)0x00000100u)
#define CSL_MCASP_RSTAT_RERR_SHIFT ((uint32_t)0x00000008u)
#define CSL_MCASP_RSTAT_RERR_RESETVAL ((uint32_t)0x00000000u)
/*----RERR Tokens----*/
#define CSL_MCASP_RSTAT_RERR_NOERROR ((uint32_t)0x00000000u)
#define CSL_MCASP_RSTAT_RERR_ERROR ((uint32_t)0x00000001u)

#define CSL_MCASP_RSTAT_RDMAERR_MASK ((uint32_t)0x00000080u)
#define CSL_MCASP_RSTAT_RDMAERR_SHIFT ((uint32_t)0x00000007u)
#define CSL_MCASP_RSTAT_RDMAERR_RESETVAL ((uint32_t)0x00000000u)
/*----RDMAERR Tokens----*/
#define CSL_MCASP_RSTAT_RDMAERR_NO_ERROR ((uint32_t)0x00000000u)
#define CSL_MCASP_RSTAT_RDMAERR_ERROR ((uint32_t)0x00000001u)

#define CSL_MCASP_RSTAT_RSTAFRM_MASK ((uint32_t)0x00000040u)
#define CSL_MCASP_RSTAT_RSTAFRM_SHIFT ((uint32_t)0x00000006u)
#define CSL_MCASP_RSTAT_RSTAFRM_RESETVAL ((uint32_t)0x00000000u)
/*----RSTAFRM Tokens----*/
#define CSL_MCASP_RSTAT_RSTAFRM_NO ((uint32_t)0x00000000u)
#define CSL_MCASP_RSTAT_RSTAFRM_YES ((uint32_t)0x00000001u)

#define CSL_MCASP_RSTAT_RDATA_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_RSTAT_RDATA_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_RSTAT_RDATA_RESETVAL ((uint32_t)0x00000000u)
/*----RDATA Tokens----*/
#define CSL_MCASP_RSTAT_RDATA_NO ((uint32_t)0x00000000u)
#define CSL_MCASP_RSTAT_RDATA_YES ((uint32_t)0x00000001u)

#define CSL_MCASP_RSTAT_RLAST_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_RSTAT_RLAST_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_RSTAT_RLAST_RESETVAL ((uint32_t)0x00000000u)
/*----RLAST Tokens----*/
#define CSL_MCASP_RSTAT_RLAST_NO ((uint32_t)0x00000000u)
#define CSL_MCASP_RSTAT_RLAST_YES ((uint32_t)0x00000001u)

#define CSL_MCASP_RSTAT_RTDMSLOT_MASK ((uint32_t)0x00000008u)
#define CSL_MCASP_RSTAT_RTDMSLOT_SHIFT ((uint32_t)0x00000003u)
#define CSL_MCASP_RSTAT_RTDMSLOT_RESETVAL ((uint32_t)0x00000000u)
/*----RTDMSLOT Tokens----*/
#define CSL_MCASP_RSTAT_RTDMSLOT_ODD ((uint32_t)0x00000000u)
#define CSL_MCASP_RSTAT_RTDMSLOT_EVEN ((uint32_t)0x00000001u)

#define CSL_MCASP_RSTAT_RCKFAIL_MASK ((uint32_t)0x00000004u)
#define CSL_MCASP_RSTAT_RCKFAIL_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_RSTAT_RCKFAIL_RESETVAL ((uint32_t)0x00000000u)
/*----RCKFAIL Tokens----*/
#define CSL_MCASP_RSTAT_RCKFAIL_NO ((uint32_t)0x00000000u)
#define CSL_MCASP_RSTAT_RCKFAIL_YES ((uint32_t)0x00000001u)

#define CSL_MCASP_RSTAT_RSYNCERR_MASK ((uint32_t)0x00000002u)
#define CSL_MCASP_RSTAT_RSYNCERR_SHIFT ((uint32_t)0x00000001u)
#define CSL_MCASP_RSTAT_RSYNCERR_RESETVAL ((uint32_t)0x00000000u)
/*----RSYNCERR Tokens----*/
#define CSL_MCASP_RSTAT_RSYNCERR_NO ((uint32_t)0x00000000u)
#define CSL_MCASP_RSTAT_RSYNCERR_YES ((uint32_t)0x00000001u)

#define CSL_MCASP_RSTAT_ROVRN_MASK ((uint32_t)0x00000001u)
#define CSL_MCASP_RSTAT_ROVRN_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_RSTAT_ROVRN_RESETVAL ((uint32_t)0x00000000u)
/*----ROVRN Tokens----*/
#define CSL_MCASP_RSTAT_ROVRN_NO ((uint32_t)0x00000000u)
#define CSL_MCASP_RSTAT_ROVRN_YES ((uint32_t)0x00000001u)

#define CSL_MCASP_RSTAT_RESETVAL ((uint32_t)0x00000000u)

/* RSLOT */


#define CSL_MCASP_RSLOT_RSLOTCNT_MASK ((uint32_t)0x000003FFu)
#define CSL_MCASP_RSLOT_RSLOTCNT_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_RSLOT_RSLOTCNT_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_RSLOT_RESETVAL ((uint32_t)0x00000000u)

/* RCLKCHK */

#define CSL_MCASP_RCLKCHK_RCNT_MASK ((uint32_t)0xFF000000u)
#define CSL_MCASP_RCLKCHK_RCNT_SHIFT ((uint32_t)0x00000018u)
#define CSL_MCASP_RCLKCHK_RCNT_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_RCLKCHK_RMAX_MASK ((uint32_t)0x00FF0000u)
#define CSL_MCASP_RCLKCHK_RMAX_SHIFT ((uint32_t)0x00000010u)
#define CSL_MCASP_RCLKCHK_RMAX_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_RCLKCHK_RMIN_MASK ((uint32_t)0x0000FF00u)
#define CSL_MCASP_RCLKCHK_RMIN_SHIFT ((uint32_t)0x00000008u)
#define CSL_MCASP_RCLKCHK_RMIN_RESETVAL ((uint32_t)0x00000000u)


#define CSL_MCASP_RCLKCHK_RPS_MASK ((uint32_t)0x0000000Fu)
#define CSL_MCASP_RCLKCHK_RPS_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_RCLKCHK_RPS_RESETVAL ((uint32_t)0x00000000u)
/*----RPS Tokens----*/
#define CSL_MCASP_RCLKCHK_RPS_DIVBY1 ((uint32_t)0x00000000u)
#define CSL_MCASP_RCLKCHK_RPS_DIVBY2 ((uint32_t)0x00000001u)
#define CSL_MCASP_RCLKCHK_RPS_DIVBY4 ((uint32_t)0x00000002u)
#define CSL_MCASP_RCLKCHK_RPS_DIVBY8 ((uint32_t)0x00000003u)
#define CSL_MCASP_RCLKCHK_RPS_DIVBY16 ((uint32_t)0x00000004u)
#define CSL_MCASP_RCLKCHK_RPS_DIVBY32 ((uint32_t)0x00000005u)
#define CSL_MCASP_RCLKCHK_RPS_DIVBY64 ((uint32_t)0x00000006u)
#define CSL_MCASP_RCLKCHK_RPS_DIVBY128 ((uint32_t)0x00000007u)
#define CSL_MCASP_RCLKCHK_RPS_DIVBY256 ((uint32_t)0x00000008u)

#define CSL_MCASP_RCLKCHK_RESETVAL ((uint32_t)0x00000000u)

/* REVTCTL */


#define CSL_MCASP_REVTCTL_RDATDMA_MASK ((uint32_t)0x00000001u)
#define CSL_MCASP_REVTCTL_RDATDMA_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_REVTCTL_RDATDMA_RESETVAL ((uint32_t)0x00000000u)
/*----RDATDMA Tokens----*/
#define CSL_MCASP_REVTCTL_RDATDMA_ENABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_REVTCTL_RDATDMA_RSV ((uint32_t)0x00000001u)

#define CSL_MCASP_REVTCTL_RESETVAL ((uint32_t)0x00000000u)

/* XGBLCTL */


#define CSL_MCASP_XGBLCTL_XFRST_MASK ((uint32_t)0x00001000u)
#define CSL_MCASP_XGBLCTL_XFRST_SHIFT ((uint32_t)0x0000000Cu)
#define CSL_MCASP_XGBLCTL_XFRST_RESETVAL ((uint32_t)0x00000000u)
/*----XFRST Tokens----*/
#define CSL_MCASP_XGBLCTL_XFRST_RESET ((uint32_t)0x00000000u)
#define CSL_MCASP_XGBLCTL_XFRST_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XGBLCTL_XSMRST_MASK ((uint32_t)0x00000800u)
#define CSL_MCASP_XGBLCTL_XSMRST_SHIFT ((uint32_t)0x0000000Bu)
#define CSL_MCASP_XGBLCTL_XSMRST_RESETVAL ((uint32_t)0x00000000u)
/*----XSMRST Tokens----*/
#define CSL_MCASP_XGBLCTL_XSMRST_RESET ((uint32_t)0x00000000u)
#define CSL_MCASP_XGBLCTL_XSMRST_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XGBLCTL_XSRCLR_MASK ((uint32_t)0x00000400u)
#define CSL_MCASP_XGBLCTL_XSRCLR_SHIFT ((uint32_t)0x0000000Au)
#define CSL_MCASP_XGBLCTL_XSRCLR_RESETVAL ((uint32_t)0x00000000u)
/*----XSRCLR Tokens----*/
#define CSL_MCASP_XGBLCTL_XSRCLR_CLEAR ((uint32_t)0x00000000u)
#define CSL_MCASP_XGBLCTL_XSRCLR_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XGBLCTL_XHCLKRST_MASK ((uint32_t)0x00000200u)
#define CSL_MCASP_XGBLCTL_XHCLKRST_SHIFT ((uint32_t)0x00000009u)
#define CSL_MCASP_XGBLCTL_XHCLKRST_RESETVAL ((uint32_t)0x00000000u)
/*----XHCLKRST Tokens----*/
#define CSL_MCASP_XGBLCTL_XHCLKRST_RESET ((uint32_t)0x00000000u)
#define CSL_MCASP_XGBLCTL_XHCLKRST_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XGBLCTL_XCLKRST_MASK ((uint32_t)0x00000100u)
#define CSL_MCASP_XGBLCTL_XCLKRST_SHIFT ((uint32_t)0x00000008u)
#define CSL_MCASP_XGBLCTL_XCLKRST_RESETVAL ((uint32_t)0x00000000u)
/*----XCLKRST Tokens----*/
#define CSL_MCASP_XGBLCTL_XCLKRST_RESET ((uint32_t)0x00000000u)
#define CSL_MCASP_XGBLCTL_XCLKRST_ACTIVE ((uint32_t)0x00000001u)


#define CSL_MCASP_XGBLCTL_RFRST_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_XGBLCTL_RFRST_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_XGBLCTL_RFRST_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XGBLCTL_RFRST_RESET ((uint32_t)0x00000000u)
#define CSL_MCASP_XGBLCTL_RFRST_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XGBLCTL_RSMRST_MASK ((uint32_t)0x00000008u)
#define CSL_MCASP_XGBLCTL_RSMRST_SHIFT ((uint32_t)0x00000003u)
#define CSL_MCASP_XGBLCTL_RSMRST_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XGBLCTL_RSMRST_RESET ((uint32_t)0x00000000u)
#define CSL_MCASP_XGBLCTL_RSMRST_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XGBLCTL_RSRCLKR_MASK ((uint32_t)0x00000004u)
#define CSL_MCASP_XGBLCTL_RSRCLKR_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_XGBLCTL_RSRCLKR_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XGBLCTL_RSRCLKR_CLEAR ((uint32_t)0x00000000u)
#define CSL_MCASP_XGBLCTL_RSRCLKR_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XGBLCTL_RHCLKRST_MASK ((uint32_t)0x00000002u)
#define CSL_MCASP_XGBLCTL_RHCLKRST_SHIFT ((uint32_t)0x00000001u)
#define CSL_MCASP_XGBLCTL_RHCLKRST_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XGBLCTL_RHCLKRST_RESET ((uint32_t)0x00000000u)
#define CSL_MCASP_XGBLCTL_RHCLKRST_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XGBLCTL_RCLKRST_MASK ((uint32_t)0x00000001u)
#define CSL_MCASP_XGBLCTL_RCLKRST_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_XGBLCTL_RCLKRST_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XGBLCTL_RCLKRST_RESET ((uint32_t)0x00000000u)
#define CSL_MCASP_XGBLCTL_RCLKRST_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XGBLCTL_RESETVAL ((uint32_t)0x00000000u)

/* XMASK */

#define CSL_MCASP_XMASK_XMASK31_MASK ((uint32_t)0x80000000u)
#define CSL_MCASP_XMASK_XMASK31_SHIFT ((uint32_t)0x0000001Fu)
#define CSL_MCASP_XMASK_XMASK31_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK31_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK31_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK30_MASK ((uint32_t)0x40000000u)
#define CSL_MCASP_XMASK_XMASK30_SHIFT ((uint32_t)0x0000001Eu)
#define CSL_MCASP_XMASK_XMASK30_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK30_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK30_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK29_MASK ((uint32_t)0x20000000u)
#define CSL_MCASP_XMASK_XMASK29_SHIFT ((uint32_t)0x0000001Du)
#define CSL_MCASP_XMASK_XMASK29_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK29_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK29_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK28_MASK ((uint32_t)0x10000000u)
#define CSL_MCASP_XMASK_XMASK28_SHIFT ((uint32_t)0x0000001Cu)
#define CSL_MCASP_XMASK_XMASK28_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK28_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK28_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK27_MASK ((uint32_t)0x08000000u)
#define CSL_MCASP_XMASK_XMASK27_SHIFT ((uint32_t)0x0000001Bu)
#define CSL_MCASP_XMASK_XMASK27_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK27_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK27_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK26_MASK ((uint32_t)0x04000000u)
#define CSL_MCASP_XMASK_XMASK26_SHIFT ((uint32_t)0x0000001Au)
#define CSL_MCASP_XMASK_XMASK26_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK26_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK26_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK25_MASK ((uint32_t)0x02000000u)
#define CSL_MCASP_XMASK_XMASK25_SHIFT ((uint32_t)0x00000019u)
#define CSL_MCASP_XMASK_XMASK25_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK25_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK25_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK24_MASK ((uint32_t)0x01000000u)
#define CSL_MCASP_XMASK_XMASK24_SHIFT ((uint32_t)0x00000018u)
#define CSL_MCASP_XMASK_XMASK24_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK24_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK24_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK23_MASK ((uint32_t)0x00800000u)
#define CSL_MCASP_XMASK_XMASK23_SHIFT ((uint32_t)0x00000017u)
#define CSL_MCASP_XMASK_XMASK23_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK23_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK23_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK22_MASK ((uint32_t)0x00400000u)
#define CSL_MCASP_XMASK_XMASK22_SHIFT ((uint32_t)0x00000016u)
#define CSL_MCASP_XMASK_XMASK22_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK22_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK22_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK21_MASK ((uint32_t)0x00200000u)
#define CSL_MCASP_XMASK_XMASK21_SHIFT ((uint32_t)0x00000015u)
#define CSL_MCASP_XMASK_XMASK21_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK21_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK21_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK20_MASK ((uint32_t)0x00100000u)
#define CSL_MCASP_XMASK_XMASK20_SHIFT ((uint32_t)0x00000014u)
#define CSL_MCASP_XMASK_XMASK20_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK20_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK20_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK19_MASK ((uint32_t)0x00080000u)
#define CSL_MCASP_XMASK_XMASK19_SHIFT ((uint32_t)0x00000013u)
#define CSL_MCASP_XMASK_XMASK19_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK19_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK19_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK18_MASK ((uint32_t)0x00040000u)
#define CSL_MCASP_XMASK_XMASK18_SHIFT ((uint32_t)0x00000012u)
#define CSL_MCASP_XMASK_XMASK18_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK18_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK18_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK17_MASK ((uint32_t)0x00020000u)
#define CSL_MCASP_XMASK_XMASK17_SHIFT ((uint32_t)0x00000011u)
#define CSL_MCASP_XMASK_XMASK17_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK17_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK17_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK16_MASK ((uint32_t)0x00010000u)
#define CSL_MCASP_XMASK_XMASK16_SHIFT ((uint32_t)0x00000010u)
#define CSL_MCASP_XMASK_XMASK16_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK16_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK16_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK15_MASK ((uint32_t)0x00008000u)
#define CSL_MCASP_XMASK_XMASK15_SHIFT ((uint32_t)0x0000000Fu)
#define CSL_MCASP_XMASK_XMASK15_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK15_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK15_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK14_MASK ((uint32_t)0x00004000u)
#define CSL_MCASP_XMASK_XMASK14_SHIFT ((uint32_t)0x0000000Eu)
#define CSL_MCASP_XMASK_XMASK14_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK14_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK14_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK13_MASK ((uint32_t)0x00002000u)
#define CSL_MCASP_XMASK_XMASK13_SHIFT ((uint32_t)0x0000000Du)
#define CSL_MCASP_XMASK_XMASK13_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK13_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK13_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK12_MASK ((uint32_t)0x00001000u)
#define CSL_MCASP_XMASK_XMASK12_SHIFT ((uint32_t)0x0000000Cu)
#define CSL_MCASP_XMASK_XMASK12_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK12_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK12_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK11_MASK ((uint32_t)0x00000800u)
#define CSL_MCASP_XMASK_XMASK11_SHIFT ((uint32_t)0x0000000Bu)
#define CSL_MCASP_XMASK_XMASK11_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK11_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK11_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK10_MASK ((uint32_t)0x00000400u)
#define CSL_MCASP_XMASK_XMASK10_SHIFT ((uint32_t)0x0000000Au)
#define CSL_MCASP_XMASK_XMASK10_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK10_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK10_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK9_MASK ((uint32_t)0x00000200u)
#define CSL_MCASP_XMASK_XMASK9_SHIFT ((uint32_t)0x00000009u)
#define CSL_MCASP_XMASK_XMASK9_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK9_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK9_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK8_MASK ((uint32_t)0x00000100u)
#define CSL_MCASP_XMASK_XMASK8_SHIFT ((uint32_t)0x00000008u)
#define CSL_MCASP_XMASK_XMASK8_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK8_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK8_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK7_MASK ((uint32_t)0x00000080u)
#define CSL_MCASP_XMASK_XMASK7_SHIFT ((uint32_t)0x00000007u)
#define CSL_MCASP_XMASK_XMASK7_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK7_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK7_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK6_MASK ((uint32_t)0x00000040u)
#define CSL_MCASP_XMASK_XMASK6_SHIFT ((uint32_t)0x00000006u)
#define CSL_MCASP_XMASK_XMASK6_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK6_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK6_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK5_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_XMASK_XMASK5_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_XMASK_XMASK5_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK5_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK5_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK4_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_XMASK_XMASK4_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_XMASK_XMASK4_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK4_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK4_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK3_MASK ((uint32_t)0x00000008u)
#define CSL_MCASP_XMASK_XMASK3_SHIFT ((uint32_t)0x00000003u)
#define CSL_MCASP_XMASK_XMASK3_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK3_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK3_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK2_MASK ((uint32_t)0x00000004u)
#define CSL_MCASP_XMASK_XMASK2_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_XMASK_XMASK2_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK2_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK2_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK1_MASK ((uint32_t)0x00000002u)
#define CSL_MCASP_XMASK_XMASK1_SHIFT ((uint32_t)0x00000001u)
#define CSL_MCASP_XMASK_XMASK1_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK1_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK1_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_XMASK0_MASK ((uint32_t)0x00000001u)
#define CSL_MCASP_XMASK_XMASK0_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK0_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK0_USEMASK ((uint32_t)0x00000000u)
#define CSL_MCASP_XMASK_XMASK0_NOMASK ((uint32_t)0x00000001u)

#define CSL_MCASP_XMASK_RESETVAL ((uint32_t)0x00000000u)

/* XFMT */


#define CSL_MCASP_XFMT_XDATDLY_MASK ((uint32_t)0x00030000u)
#define CSL_MCASP_XFMT_XDATDLY_SHIFT ((uint32_t)0x00000010u)
#define CSL_MCASP_XFMT_XDATDLY_RESETVAL ((uint32_t)0x00000000u)
/*----XDATDLY Tokens----*/
#define CSL_MCASP_XFMT_XDATDLY_0BIT ((uint32_t)0x00000000u)
#define CSL_MCASP_XFMT_XDATDLY_1BIT ((uint32_t)0x00000001u)
#define CSL_MCASP_XFMT_XDATDLY_2BIT ((uint32_t)0x00000002u)

#define CSL_MCASP_XFMT_XRVRS_MASK ((uint32_t)0x00008000u)
#define CSL_MCASP_XFMT_XRVRS_SHIFT ((uint32_t)0x0000000Fu)
#define CSL_MCASP_XFMT_XRVRS_RESETVAL ((uint32_t)0x00000000u)
/*----XRVRS Tokens----*/
#define CSL_MCASP_XFMT_XRVRS_LSBFIRST ((uint32_t)0x00000000u)
#define CSL_MCASP_XFMT_XRVRS_MSBFIRST ((uint32_t)0x00000001u)

#define CSL_MCASP_XFMT_XPAD_MASK ((uint32_t)0x00006000u)
#define CSL_MCASP_XFMT_XPAD_SHIFT ((uint32_t)0x0000000Du)
#define CSL_MCASP_XFMT_XPAD_RESETVAL ((uint32_t)0x00000000u)
/*----XPAD Tokens----*/
#define CSL_MCASP_XFMT_XPAD_ZERO ((uint32_t)0x00000000u)
#define CSL_MCASP_XFMT_XPAD_ONE ((uint32_t)0x00000001u)
#define CSL_MCASP_XFMT_XPAD_XPBIT ((uint32_t)0x00000002u)

#define CSL_MCASP_XFMT_XPBIT_MASK ((uint32_t)0x00001F00u)
#define CSL_MCASP_XFMT_XPBIT_SHIFT ((uint32_t)0x00000008u)
#define CSL_MCASP_XFMT_XPBIT_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_XFMT_XSSZ_MASK ((uint32_t)0x000000F0u)
#define CSL_MCASP_XFMT_XSSZ_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_XFMT_XSSZ_RESETVAL ((uint32_t)0x00000000u)
/*----XSSZ Tokens----*/
#define CSL_MCASP_XFMT_XSSZ_RSV ((uint32_t)0x00000000u)
#define CSL_MCASP_XFMT_XSSZ_RSV1 ((uint32_t)0x00000001u)
#define CSL_MCASP_XFMT_XSSZ_RSV2 ((uint32_t)0x00000002u)
#define CSL_MCASP_XFMT_XSSZ_8BITS ((uint32_t)0x00000003u)
#define CSL_MCASP_XFMT_XSSZ_RSV3 ((uint32_t)0x00000004u)
#define CSL_MCASP_XFMT_XSSZ_12BITS ((uint32_t)0x00000005u)
#define CSL_MCASP_XFMT_XSSZ_RSV4 ((uint32_t)0x00000006u)
#define CSL_MCASP_XFMT_XSSZ_16BITS ((uint32_t)0x00000007u)
#define CSL_MCASP_XFMT_XSSZ_RSV5 ((uint32_t)0x00000008u)
#define CSL_MCASP_XFMT_XSSZ_20BITS ((uint32_t)0x00000009u)
#define CSL_MCASP_XFMT_XSSZ_RSV6 ((uint32_t)0x0000000Au)
#define CSL_MCASP_XFMT_XSSZ_24BITS ((uint32_t)0x0000000Bu)
#define CSL_MCASP_XFMT_XSSZ_RSV7 ((uint32_t)0x0000000Cu)
#define CSL_MCASP_XFMT_XSSZ_28BITS ((uint32_t)0x0000000Du)
#define CSL_MCASP_XFMT_XSSZ_RSV8 ((uint32_t)0x0000000Eu)
#define CSL_MCASP_XFMT_XSSZ_32BITS ((uint32_t)0x0000000Fu)

#define CSL_MCASP_XFMT_XBUSEL_MASK ((uint32_t)0x00000008u)
#define CSL_MCASP_XFMT_XBUSEL_SHIFT ((uint32_t)0x00000003u)
#define CSL_MCASP_XFMT_XBUSEL_RESETVAL ((uint32_t)0x00000000u)
/*----XBUSEL Tokens----*/
#define CSL_MCASP_XFMT_XBUSEL_VBUSP ((uint32_t)0x00000000u)
#define CSL_MCASP_XFMT_XBUSEL_VBUS ((uint32_t)0x00000001u)

#define CSL_MCASP_XFMT_XROT_MASK ((uint32_t)0x00000007u)
#define CSL_MCASP_XFMT_XROT_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_XFMT_XROT_RESETVAL ((uint32_t)0x00000000u)
/*----XROT Tokens----*/
#define CSL_MCASP_XFMT_XROT_NONE ((uint32_t)0x00000000u)
#define CSL_MCASP_XFMT_XROT_4BITS ((uint32_t)0x00000001u)
#define CSL_MCASP_XFMT_XROT_8BITS ((uint32_t)0x00000002u)
#define CSL_MCASP_XFMT_XROT_12BITS ((uint32_t)0x00000003u)
#define CSL_MCASP_XFMT_XROT_16BITS ((uint32_t)0x00000004u)
#define CSL_MCASP_XFMT_XROT_20BITS ((uint32_t)0x00000005u)
#define CSL_MCASP_XFMT_XROT_24BITS ((uint32_t)0x00000006u)
#define CSL_MCASP_XFMT_XROT_28BITS ((uint32_t)0x00000007u)

#define CSL_MCASP_XFMT_RESETVAL ((uint32_t)0x00000000u)

/* AFSXCTL */


#define CSL_MCASP_AFSXCTL_XMOD_MASK ((uint32_t)0x0000FF80u)
#define CSL_MCASP_AFSXCTL_XMOD_SHIFT ((uint32_t)0x00000007u)
#define CSL_MCASP_AFSXCTL_XMOD_RESETVAL ((uint32_t)0x00000000u)
/*----XMOD Tokens----*/
#define CSL_MCASP_AFSXCTL_XMOD_BURST ((uint32_t)0x00000000u)
#define CSL_MCASP_AFSXCTL_XMOD_I2S ((uint32_t)0x00000002u)


#define CSL_MCASP_AFSXCTL_FXWID_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_AFSXCTL_FXWID_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_AFSXCTL_FXWID_RESETVAL ((uint32_t)0x00000000u)
/*----FXWID Tokens----*/
#define CSL_MCASP_AFSXCTL_FXWID_BIT ((uint32_t)0x00000000u)
#define CSL_MCASP_AFSXCTL_FXWID_WORD ((uint32_t)0x00000001u)


#define CSL_MCASP_AFSXCTL_FSXM_MASK ((uint32_t)0x00000002u)
#define CSL_MCASP_AFSXCTL_FSXM_SHIFT ((uint32_t)0x00000001u)
#define CSL_MCASP_AFSXCTL_FSXM_RESETVAL ((uint32_t)0x00000000u)
/*----FSXM Tokens----*/
#define CSL_MCASP_AFSXCTL_FSXM_EXTERNAL ((uint32_t)0x00000000u)
#define CSL_MCASP_AFSXCTL_FSXM_INTERNAL ((uint32_t)0x00000001u)

#define CSL_MCASP_AFSXCTL_FSXP_MASK ((uint32_t)0x00000001u)
#define CSL_MCASP_AFSXCTL_FSXP_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_AFSXCTL_FSXP_RESETVAL ((uint32_t)0x00000000u)
/*----FSXP Tokens----*/
#define CSL_MCASP_AFSXCTL_FSXP_RISINGEDGE ((uint32_t)0x00000000u)
#define CSL_MCASP_AFSXCTL_FSXP_FALLINGEDGE ((uint32_t)0x00000001u)

#define CSL_MCASP_AFSXCTL_RESETVAL ((uint32_t)0x00000000u)

/* ACLKXCTL */


#define CSL_MCASP_ACLKXCTL_CLKXP_MASK ((uint32_t)0x00000080u)
#define CSL_MCASP_ACLKXCTL_CLKXP_SHIFT ((uint32_t)0x00000007u)
#define CSL_MCASP_ACLKXCTL_CLKXP_RESETVAL ((uint32_t)0x00000000u)
/*----CLKXP Tokens----*/
#define CSL_MCASP_ACLKXCTL_CLKXP_RISINGEDGE ((uint32_t)0x00000000u)
#define CSL_MCASP_ACLKXCTL_CLKXP_FALLINGEDGE ((uint32_t)0x00000001u)

#define CSL_MCASP_ACLKXCTL_ASYNC_MASK ((uint32_t)0x00000040u)
#define CSL_MCASP_ACLKXCTL_ASYNC_SHIFT ((uint32_t)0x00000006u)
#define CSL_MCASP_ACLKXCTL_ASYNC_RESETVAL ((uint32_t)0x00000001u)
/*----ASYNC Tokens----*/
#define CSL_MCASP_ACLKXCTL_ASYNC_SYNC ((uint32_t)0x00000000u)
#define CSL_MCASP_ACLKXCTL_ASYNC_ASYNC ((uint32_t)0x00000001u)

#define CSL_MCASP_ACLKXCTL_CLKXM_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_ACLKXCTL_CLKXM_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_ACLKXCTL_CLKXM_RESETVAL ((uint32_t)0x00000001u)
/*----CLKXM Tokens----*/
#define CSL_MCASP_ACLKXCTL_CLKXM_EXTERNAL ((uint32_t)0x00000000u)
#define CSL_MCASP_ACLKXCTL_CLKXM_INTERNAL ((uint32_t)0x00000001u)

#define CSL_MCASP_ACLKXCTL_CLKXDIV_MASK ((uint32_t)0x0000001Fu)
#define CSL_MCASP_ACLKXCTL_CLKXDIV_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_ACLKXCTL_CLKXDIV_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_ACLKXCTL_RESETVAL ((uint32_t)0x00000060u)

/* AHCLKXCTL */


#define CSL_MCASP_AHCLKXCTL_HCLKXM_MASK ((uint32_t)0x00008000u)
#define CSL_MCASP_AHCLKXCTL_HCLKXM_SHIFT ((uint32_t)0x0000000Fu)
#define CSL_MCASP_AHCLKXCTL_HCLKXM_RESETVAL ((uint32_t)0x00000001u)
/*----HCLKXM Tokens----*/
#define CSL_MCASP_AHCLKXCTL_HCLKXM_EXTERNAL ((uint32_t)0x00000000u)
#define CSL_MCASP_AHCLKXCTL_HCLKXM_INTERNAL ((uint32_t)0x00000001u)

#define CSL_MCASP_AHCLKXCTL_HCLKXP_MASK ((uint32_t)0x00004000u)
#define CSL_MCASP_AHCLKXCTL_HCLKXP_SHIFT ((uint32_t)0x0000000Eu)
#define CSL_MCASP_AHCLKXCTL_HCLKXP_RESETVAL ((uint32_t)0x00000000u)
/*----HCLKXP Tokens----*/
#define CSL_MCASP_AHCLKXCTL_HCLKXP_NOTINVERTED ((uint32_t)0x00000000u)
#define CSL_MCASP_AHCLKXCTL_HCLKXP_INVERTED ((uint32_t)0x00000001u)


#define CSL_MCASP_AHCLKXCTL_HCLKXDIV_MASK ((uint32_t)0x00000FFFu)
#define CSL_MCASP_AHCLKXCTL_HCLKXDIV_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_AHCLKXCTL_HCLKXDIV_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_AHCLKXCTL_RESETVAL ((uint32_t)0x00008000u)

/* XTDM */

#define CSL_MCASP_XTDM_XTDMS31_MASK ((uint32_t)0x80000000u)
#define CSL_MCASP_XTDM_XTDMS31_SHIFT ((uint32_t)0x0000001Fu)
#define CSL_MCASP_XTDM_XTDMS31_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS31_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS31_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS30_MASK ((uint32_t)0x40000000u)
#define CSL_MCASP_XTDM_XTDMS30_SHIFT ((uint32_t)0x0000001Eu)
#define CSL_MCASP_XTDM_XTDMS30_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS30_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS30_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS29_MASK ((uint32_t)0x20000000u)
#define CSL_MCASP_XTDM_XTDMS29_SHIFT ((uint32_t)0x0000001Du)
#define CSL_MCASP_XTDM_XTDMS29_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS29_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS29_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS28_MASK ((uint32_t)0x10000000u)
#define CSL_MCASP_XTDM_XTDMS28_SHIFT ((uint32_t)0x0000001Cu)
#define CSL_MCASP_XTDM_XTDMS28_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS28_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS28_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS27_MASK ((uint32_t)0x08000000u)
#define CSL_MCASP_XTDM_XTDMS27_SHIFT ((uint32_t)0x0000001Bu)
#define CSL_MCASP_XTDM_XTDMS27_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS27_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS27_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS26_MASK ((uint32_t)0x04000000u)
#define CSL_MCASP_XTDM_XTDMS26_SHIFT ((uint32_t)0x0000001Au)
#define CSL_MCASP_XTDM_XTDMS26_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS26_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS26_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS25_MASK ((uint32_t)0x02000000u)
#define CSL_MCASP_XTDM_XTDMS25_SHIFT ((uint32_t)0x00000019u)
#define CSL_MCASP_XTDM_XTDMS25_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS25_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS25_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS24_MASK ((uint32_t)0x01000000u)
#define CSL_MCASP_XTDM_XTDMS24_SHIFT ((uint32_t)0x00000018u)
#define CSL_MCASP_XTDM_XTDMS24_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS24_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS24_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS23_MASK ((uint32_t)0x00800000u)
#define CSL_MCASP_XTDM_XTDMS23_SHIFT ((uint32_t)0x00000017u)
#define CSL_MCASP_XTDM_XTDMS23_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS23_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS23_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS22_MASK ((uint32_t)0x00400000u)
#define CSL_MCASP_XTDM_XTDMS22_SHIFT ((uint32_t)0x00000016u)
#define CSL_MCASP_XTDM_XTDMS22_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS22_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS22_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS21_MASK ((uint32_t)0x00200000u)
#define CSL_MCASP_XTDM_XTDMS21_SHIFT ((uint32_t)0x00000015u)
#define CSL_MCASP_XTDM_XTDMS21_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS21_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS21_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS20_MASK ((uint32_t)0x00100000u)
#define CSL_MCASP_XTDM_XTDMS20_SHIFT ((uint32_t)0x00000014u)
#define CSL_MCASP_XTDM_XTDMS20_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS20_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS20_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS19_MASK ((uint32_t)0x00080000u)
#define CSL_MCASP_XTDM_XTDMS19_SHIFT ((uint32_t)0x00000013u)
#define CSL_MCASP_XTDM_XTDMS19_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS19_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS19_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS18_MASK ((uint32_t)0x00040000u)
#define CSL_MCASP_XTDM_XTDMS18_SHIFT ((uint32_t)0x00000012u)
#define CSL_MCASP_XTDM_XTDMS18_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS18_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS18_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS17_MASK ((uint32_t)0x00020000u)
#define CSL_MCASP_XTDM_XTDMS17_SHIFT ((uint32_t)0x00000011u)
#define CSL_MCASP_XTDM_XTDMS17_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS17_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS17_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS16_MASK ((uint32_t)0x00010000u)
#define CSL_MCASP_XTDM_XTDMS16_SHIFT ((uint32_t)0x00000010u)
#define CSL_MCASP_XTDM_XTDMS16_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS16_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS16_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS15_MASK ((uint32_t)0x00008000u)
#define CSL_MCASP_XTDM_XTDMS15_SHIFT ((uint32_t)0x0000000Fu)
#define CSL_MCASP_XTDM_XTDMS15_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS15_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS15_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS14_MASK ((uint32_t)0x00004000u)
#define CSL_MCASP_XTDM_XTDMS14_SHIFT ((uint32_t)0x0000000Eu)
#define CSL_MCASP_XTDM_XTDMS14_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS14_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS14_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS13_MASK ((uint32_t)0x00002000u)
#define CSL_MCASP_XTDM_XTDMS13_SHIFT ((uint32_t)0x0000000Du)
#define CSL_MCASP_XTDM_XTDMS13_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS13_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS13_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS12_MASK ((uint32_t)0x00001000u)
#define CSL_MCASP_XTDM_XTDMS12_SHIFT ((uint32_t)0x0000000Cu)
#define CSL_MCASP_XTDM_XTDMS12_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS12_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS12_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS11_MASK ((uint32_t)0x00000800u)
#define CSL_MCASP_XTDM_XTDMS11_SHIFT ((uint32_t)0x0000000Bu)
#define CSL_MCASP_XTDM_XTDMS11_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS11_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS11_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS10_MASK ((uint32_t)0x00000400u)
#define CSL_MCASP_XTDM_XTDMS10_SHIFT ((uint32_t)0x0000000Au)
#define CSL_MCASP_XTDM_XTDMS10_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS10_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS10_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS9_MASK ((uint32_t)0x00000200u)
#define CSL_MCASP_XTDM_XTDMS9_SHIFT ((uint32_t)0x00000009u)
#define CSL_MCASP_XTDM_XTDMS9_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS9_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS9_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS8_MASK ((uint32_t)0x00000100u)
#define CSL_MCASP_XTDM_XTDMS8_SHIFT ((uint32_t)0x00000008u)
#define CSL_MCASP_XTDM_XTDMS8_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS8_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS8_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS7_MASK ((uint32_t)0x00000080u)
#define CSL_MCASP_XTDM_XTDMS7_SHIFT ((uint32_t)0x00000007u)
#define CSL_MCASP_XTDM_XTDMS7_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS7_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS7_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS6_MASK ((uint32_t)0x00000040u)
#define CSL_MCASP_XTDM_XTDMS6_SHIFT ((uint32_t)0x00000006u)
#define CSL_MCASP_XTDM_XTDMS6_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS6_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS6_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS5_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_XTDM_XTDMS5_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_XTDM_XTDMS5_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS5_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS5_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS4_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_XTDM_XTDMS4_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_XTDM_XTDMS4_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS4_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS4_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS3_MASK ((uint32_t)0x00000008u)
#define CSL_MCASP_XTDM_XTDMS3_SHIFT ((uint32_t)0x00000003u)
#define CSL_MCASP_XTDM_XTDMS3_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS3_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS3_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS2_MASK ((uint32_t)0x00000004u)
#define CSL_MCASP_XTDM_XTDMS2_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_XTDM_XTDMS2_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS2_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS2_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS1_MASK ((uint32_t)0x00000002u)
#define CSL_MCASP_XTDM_XTDMS1_SHIFT ((uint32_t)0x00000001u)
#define CSL_MCASP_XTDM_XTDMS1_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS1_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS1_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_XTDMS0_MASK ((uint32_t)0x00000001u)
#define CSL_MCASP_XTDM_XTDMS0_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS0_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS0_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_XTDM_XTDMS0_ACTIVE ((uint32_t)0x00000001u)

#define CSL_MCASP_XTDM_RESETVAL ((uint32_t)0x00000000u)

/* XINTCTL */


#define CSL_MCASP_XINTCTL_XSTAFRM_MASK ((uint32_t)0x00000080u)
#define CSL_MCASP_XINTCTL_XSTAFRM_SHIFT ((uint32_t)0x00000007u)
#define CSL_MCASP_XINTCTL_XSTAFRM_RESETVAL ((uint32_t)0x00000000u)
/*----XSTAFRM Tokens----*/
#define CSL_MCASP_XINTCTL_XSTAFRM_DISABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_XINTCTL_XSTAFRM_ENABLE ((uint32_t)0x00000001u)


#define CSL_MCASP_XINTCTL_XDATA_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_XINTCTL_XDATA_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_XINTCTL_XDATA_RESETVAL ((uint32_t)0x00000000u)
/*----XDATA Tokens----*/
#define CSL_MCASP_XINTCTL_XDATA_DISABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_XINTCTL_XDATA_ENABLE ((uint32_t)0x00000001u)

#define CSL_MCASP_XINTCTL_XLAST_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_XINTCTL_XLAST_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_XINTCTL_XLAST_RESETVAL ((uint32_t)0x00000000u)
/*----XLAST Tokens----*/
#define CSL_MCASP_XINTCTL_XLAST_DISABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_XINTCTL_XLAST_ENABLE ((uint32_t)0x00000001u)

#define CSL_MCASP_XINTCTL_XDMAERR_MASK ((uint32_t)0x00000008u)
#define CSL_MCASP_XINTCTL_XDMAERR_SHIFT ((uint32_t)0x00000003u)
#define CSL_MCASP_XINTCTL_XDMAERR_RESETVAL ((uint32_t)0x00000000u)
/*----XDMAERR Tokens----*/
#define CSL_MCASP_XINTCTL_XDMAERR_DISABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_XINTCTL_XDMAERR_ENABLE ((uint32_t)0x00000001u)

#define CSL_MCASP_XINTCTL_XCKFAIL_MASK ((uint32_t)0x00000004u)
#define CSL_MCASP_XINTCTL_XCKFAIL_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_XINTCTL_XCKFAIL_RESETVAL ((uint32_t)0x00000000u)
/*----XCKFAIL Tokens----*/
#define CSL_MCASP_XINTCTL_XCKFAIL_DISABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_XINTCTL_XCKFAIL_ENABLE ((uint32_t)0x00000001u)

#define CSL_MCASP_XINTCTL_XSYNCERR_MASK ((uint32_t)0x00000002u)
#define CSL_MCASP_XINTCTL_XSYNCERR_SHIFT ((uint32_t)0x00000001u)
#define CSL_MCASP_XINTCTL_XSYNCERR_RESETVAL ((uint32_t)0x00000000u)
/*----XSYNCERR Tokens----*/
#define CSL_MCASP_XINTCTL_XSYNCERR_DISABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_XINTCTL_XSYNCERR_ENABLE ((uint32_t)0x00000001u)

#define CSL_MCASP_XINTCTL_XUNDRN_MASK ((uint32_t)0x00000001u)
#define CSL_MCASP_XINTCTL_XUNDRN_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_XINTCTL_XUNDRN_RESETVAL ((uint32_t)0x00000000u)
/*----XUNDRN Tokens----*/
#define CSL_MCASP_XINTCTL_XUNDRN_DISABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_XINTCTL_XUNDRN_ENABLE ((uint32_t)0x00000001u)

#define CSL_MCASP_XINTCTL_RESETVAL ((uint32_t)0x00000000u)

/* XSTAT */


#define CSL_MCASP_XSTAT_XERR_MASK ((uint32_t)0x00000100u)
#define CSL_MCASP_XSTAT_XERR_SHIFT ((uint32_t)0x00000008u)
#define CSL_MCASP_XSTAT_XERR_RESETVAL ((uint32_t)0x00000000u)
/*----XERR Tokens----*/
#define CSL_MCASP_XSTAT_XERR_NOERROR ((uint32_t)0x00000000u)
#define CSL_MCASP_XSTAT_XERR_ERROR ((uint32_t)0x00000001u)

#define CSL_MCASP_XSTAT_XDMAERR_MASK ((uint32_t)0x00000080u)
#define CSL_MCASP_XSTAT_XDMAERR_SHIFT ((uint32_t)0x00000007u)
#define CSL_MCASP_XSTAT_XDMAERR_RESETVAL ((uint32_t)0x00000000u)
/*----XDMAERR Tokens----*/
#define CSL_MCASP_XSTAT_XDMAERR_NOERROR ((uint32_t)0x00000000u)
#define CSL_MCASP_XSTAT_XDMAERR_ERROR ((uint32_t)0x00000001u)

#define CSL_MCASP_XSTAT_XSTAFRM_MASK ((uint32_t)0x00000040u)
#define CSL_MCASP_XSTAT_XSTAFRM_SHIFT ((uint32_t)0x00000006u)
#define CSL_MCASP_XSTAT_XSTAFRM_RESETVAL ((uint32_t)0x00000000u)
/*----XSTAFRM Tokens----*/
#define CSL_MCASP_XSTAT_XSTAFRM_NO ((uint32_t)0x00000000u)
#define CSL_MCASP_XSTAT_XSTAFRM_YES ((uint32_t)0x00000001u)

#define CSL_MCASP_XSTAT_XDATA_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_XSTAT_XDATA_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_XSTAT_XDATA_RESETVAL ((uint32_t)0x00000000u)
/*----XDATA Tokens----*/
#define CSL_MCASP_XSTAT_XDATA_NO ((uint32_t)0x00000000u)
#define CSL_MCASP_XSTAT_XDATA_YES ((uint32_t)0x00000001u)

#define CSL_MCASP_XSTAT_XLAST_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_XSTAT_XLAST_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_XSTAT_XLAST_RESETVAL ((uint32_t)0x00000000u)
/*----XLAST Tokens----*/
#define CSL_MCASP_XSTAT_XLAST_NO ((uint32_t)0x00000000u)
#define CSL_MCASP_XSTAT_XLAST_YES ((uint32_t)0x00000001u)

#define CSL_MCASP_XSTAT_XTDMSLOT_MASK ((uint32_t)0x00000008u)
#define CSL_MCASP_XSTAT_XTDMSLOT_SHIFT ((uint32_t)0x00000003u)
#define CSL_MCASP_XSTAT_XTDMSLOT_RESETVAL ((uint32_t)0x00000000u)
/*----XTDMSLOT Tokens----*/
#define CSL_MCASP_XSTAT_XTDMSLOT_ODD ((uint32_t)0x00000000u)
#define CSL_MCASP_XSTAT_XTDMSLOT_EVEN ((uint32_t)0x00000001u)

#define CSL_MCASP_XSTAT_XCKFAIL_MASK ((uint32_t)0x00000004u)
#define CSL_MCASP_XSTAT_XCKFAIL_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_XSTAT_XCKFAIL_RESETVAL ((uint32_t)0x00000000u)
/*----XCKFAIL Tokens----*/
#define CSL_MCASP_XSTAT_XCKFAIL_NO ((uint32_t)0x00000000u)
#define CSL_MCASP_XSTAT_XCKFAIL_YES ((uint32_t)0x00000001u)

#define CSL_MCASP_XSTAT_XSYNCERR_MASK ((uint32_t)0x00000002u)
#define CSL_MCASP_XSTAT_XSYNCERR_SHIFT ((uint32_t)0x00000001u)
#define CSL_MCASP_XSTAT_XSYNCERR_RESETVAL ((uint32_t)0x00000000u)
/*----XSYNCERR Tokens----*/
#define CSL_MCASP_XSTAT_XSYNCERR_NO ((uint32_t)0x00000000u)
#define CSL_MCASP_XSTAT_XSYNCERR_YES ((uint32_t)0x00000001u)

#define CSL_MCASP_XSTAT_XUNDRN_MASK ((uint32_t)0x00000001u)
#define CSL_MCASP_XSTAT_XUNDRN_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_XSTAT_XUNDRN_RESETVAL ((uint32_t)0x00000000u)
/*----XUNDRN Tokens----*/
#define CSL_MCASP_XSTAT_XUNDRN_NO ((uint32_t)0x00000000u)
#define CSL_MCASP_XSTAT_XUNDRN_YES ((uint32_t)0x00000001u)

#define CSL_MCASP_XSTAT_RESETVAL ((uint32_t)0x00000000u)

/* XSLOT */


#define CSL_MCASP_XSLOT_XSLOTCNT_MASK ((uint32_t)0x000003FFu)
#define CSL_MCASP_XSLOT_XSLOTCNT_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_XSLOT_XSLOTCNT_RESETVAL ((uint32_t)0x0000017Fu)

#define CSL_MCASP_XSLOT_RESETVAL ((uint32_t)0x0000017Fu)

/* XCLKCHK */

#define CSL_MCASP_XCLKCHK_XCNT_MASK ((uint32_t)0xFF000000u)
#define CSL_MCASP_XCLKCHK_XCNT_SHIFT ((uint32_t)0x00000018u)
#define CSL_MCASP_XCLKCHK_XCNT_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_XCLKCHK_XMAX_MASK ((uint32_t)0x00FF0000u)
#define CSL_MCASP_XCLKCHK_XMAX_SHIFT ((uint32_t)0x00000010u)
#define CSL_MCASP_XCLKCHK_XMAX_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_XCLKCHK_XMIN_MASK ((uint32_t)0x0000FF00u)
#define CSL_MCASP_XCLKCHK_XMIN_SHIFT ((uint32_t)0x00000008u)
#define CSL_MCASP_XCLKCHK_XMIN_RESETVAL ((uint32_t)0x00000000u)



#define CSL_MCASP_XCLKCHK_XPS_MASK ((uint32_t)0x0000000Fu)
#define CSL_MCASP_XCLKCHK_XPS_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_XCLKCHK_XPS_RESETVAL ((uint32_t)0x00000000u)
/*----XPS Tokens----*/
#define CSL_MCASP_XCLKCHK_XPS_DIVBY1 ((uint32_t)0x00000000u)
#define CSL_MCASP_XCLKCHK_XPS_DIVBY2 ((uint32_t)0x00000001u)
#define CSL_MCASP_XCLKCHK_XPS_DIVBY4 ((uint32_t)0x00000002u)
#define CSL_MCASP_XCLKCHK_XPS_DIVBY8 ((uint32_t)0x00000003u)
#define CSL_MCASP_XCLKCHK_XPS_DIVBY16 ((uint32_t)0x00000004u)
#define CSL_MCASP_XCLKCHK_XPS_DIVBY32 ((uint32_t)0x00000005u)
#define CSL_MCASP_XCLKCHK_XPS_DIVBY64 ((uint32_t)0x00000006u)
#define CSL_MCASP_XCLKCHK_XPS_DIVBY128 ((uint32_t)0x00000007u)
#define CSL_MCASP_XCLKCHK_XPS_DIVBY256 ((uint32_t)0x00000008u)

#define CSL_MCASP_XCLKCHK_RESETVAL ((uint32_t)0x00000000u)

/* XEVTCTL */


#define CSL_MCASP_XEVTCTL_XDATDMA_MASK ((uint32_t)0x00000001u)
#define CSL_MCASP_XEVTCTL_XDATDMA_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_XEVTCTL_XDATDMA_RESETVAL ((uint32_t)0x00000000u)
/*----XDATDMA Tokens----*/
#define CSL_MCASP_XEVTCTL_XDATDMA_ENABLE ((uint32_t)0x00000000u)
#define CSL_MCASP_XEVTCTL_XDATDMA_RSV ((uint32_t)0x00000001u)

#define CSL_MCASP_XEVTCTL_RESETVAL ((uint32_t)0x00000000u)

/* DITCSRA0 */

#define CSL_MCASP_DITCSRA0_DITCSRA0_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_DITCSRA0_DITCSRA0_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_DITCSRA0_DITCSRA0_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_DITCSRA0_RESETVAL ((uint32_t)0x00000000u)

/* DITCSRA1 */

#define CSL_MCASP_DITCSRA1_DITCSRA1_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_DITCSRA1_DITCSRA1_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_DITCSRA1_DITCSRA1_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_DITCSRA1_RESETVAL ((uint32_t)0x00000000u)

/* DITCSRA2 */

#define CSL_MCASP_DITCSRA2_DITCSRA2_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_DITCSRA2_DITCSRA2_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_DITCSRA2_DITCSRA2_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_DITCSRA2_RESETVAL ((uint32_t)0x00000000u)

/* DITCSRA3 */

#define CSL_MCASP_DITCSRA3_DITCSRA3_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_DITCSRA3_DITCSRA3_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_DITCSRA3_DITCSRA3_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_DITCSRA3_RESETVAL ((uint32_t)0x00000000u)

/* DITCSRA4 */

#define CSL_MCASP_DITCSRA4_DITCSRA4_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_DITCSRA4_DITCSRA4_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_DITCSRA4_DITCSRA4_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_DITCSRA4_RESETVAL ((uint32_t)0x00000000u)

/* DITCSRA5 */

#define CSL_MCASP_DITCSRA5_DITCSRA5_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_DITCSRA5_DITCSRA5_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_DITCSRA5_DITCSRA5_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_DITCSRA5_RESETVAL ((uint32_t)0x00000000u)

/* DITCSRB0 */

#define CSL_MCASP_DITCSRB0_DITCSRB0_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_DITCSRB0_DITCSRB0_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_DITCSRB0_DITCSRB0_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_DITCSRB0_RESETVAL ((uint32_t)0x00000000u)

/* DITCSRB1 */

#define CSL_MCASP_DITCSRB1_DITCSRB1_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_DITCSRB1_DITCSRB1_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_DITCSRB1_DITCSRB1_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_DITCSRB1_RESETVAL ((uint32_t)0x00000000u)

/* DITCSRB2 */

#define CSL_MCASP_DITCSRB2_DITCSRB2_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_DITCSRB2_DITCSRB2_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_DITCSRB2_DITCSRB2_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_DITCSRB2_RESETVAL ((uint32_t)0x00000000u)

/* DITCSRB3 */

#define CSL_MCASP_DITCSRB3_DITCSRB3_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_DITCSRB3_DITCSRB3_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_DITCSRB3_DITCSRB3_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_DITCSRB3_RESETVAL ((uint32_t)0x00000000u)

/* DITCSRB4 */

#define CSL_MCASP_DITCSRB4_DITCSRB4_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_DITCSRB4_DITCSRB4_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_DITCSRB4_DITCSRB4_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_DITCSRB4_RESETVAL ((uint32_t)0x00000000u)

/* DITCSRB5 */

#define CSL_MCASP_DITCSRB5_DITCSRB5_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_DITCSRB5_DITCSRB5_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_DITCSRB5_DITCSRB5_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_DITCSRB5_RESETVAL ((uint32_t)0x00000000u)

/* DITUDRA0 */

#define CSL_MCASP_DITUDRA0_DITUDRA0_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_DITUDRA0_DITUDRA0_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_DITUDRA0_DITUDRA0_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_DITUDRA0_RESETVAL ((uint32_t)0x00000000u)

/* DITUDRA1 */

#define CSL_MCASP_DITUDRA1_DITUDRA1_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_DITUDRA1_DITUDRA1_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_DITUDRA1_DITUDRA1_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_DITUDRA1_RESETVAL ((uint32_t)0x00000000u)

/* DITUDRA2 */

#define CSL_MCASP_DITUDRA2_DITUDRA2_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_DITUDRA2_DITUDRA2_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_DITUDRA2_DITUDRA2_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_DITUDRA2_RESETVAL ((uint32_t)0x00000000u)

/* DITUDRA3 */

#define CSL_MCASP_DITUDRA3_DITUDRA3_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_DITUDRA3_DITUDRA3_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_DITUDRA3_DITUDRA3_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_DITUDRA3_RESETVAL ((uint32_t)0x00000000u)

/* DITUDRA4 */

#define CSL_MCASP_DITUDRA4_DITUDRA4_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_DITUDRA4_DITUDRA4_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_DITUDRA4_DITUDRA4_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_DITUDRA4_RESETVAL ((uint32_t)0x00000000u)

/* DITUDRA5 */

#define CSL_MCASP_DITUDRA5_DITUDRA5_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_DITUDRA5_DITUDRA5_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_DITUDRA5_DITUDRA5_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_DITUDRA5_RESETVAL ((uint32_t)0x00000000u)

/* DITUDRB0 */

#define CSL_MCASP_DITUDRB0_DITUDRB0_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_DITUDRB0_DITUDRB0_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_DITUDRB0_DITUDRB0_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_DITUDRB0_RESETVAL ((uint32_t)0x00000000u)

/* DITUDRB1 */

#define CSL_MCASP_DITUDRB1_DITUDRB1_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_DITUDRB1_DITUDRB1_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_DITUDRB1_DITUDRB1_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_DITUDRB1_RESETVAL ((uint32_t)0x00000000u)

/* DITUDRB2 */

#define CSL_MCASP_DITUDRB2_DITUDRB2_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_DITUDRB2_DITUDRB2_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_DITUDRB2_DITUDRB2_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_DITUDRB2_RESETVAL ((uint32_t)0x00000000u)

/* DITUDRB3 */

#define CSL_MCASP_DITUDRB3_DITUDRB3_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_DITUDRB3_DITUDRB3_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_DITUDRB3_DITUDRB3_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_DITUDRB3_RESETVAL ((uint32_t)0x00000000u)

/* DITUDRB4 */

#define CSL_MCASP_DITUDRB4_DITUDRB4_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_DITUDRB4_DITUDRB4_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_DITUDRB4_DITUDRB4_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_DITUDRB4_RESETVAL ((uint32_t)0x00000000u)

/* DITUDRB5 */

#define CSL_MCASP_DITUDRB5_DITUDRB5_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_DITUDRB5_DITUDRB5_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_DITUDRB5_DITUDRB5_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_DITUDRB5_RESETVAL ((uint32_t)0x00000000u)

/* SRCTL0 */


#define CSL_MCASP_SRCTL0_RRDY_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_SRCTL0_RRDY_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_SRCTL0_RRDY_RESETVAL ((uint32_t)0x00000000u)
/*----RRDY Tokens----*/
#define CSL_MCASP_SRCTL0_RRDY_EMPTY ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL0_RRDY_DATA ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL0_XRDY_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_SRCTL0_XRDY_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_SRCTL0_XRDY_RESETVAL ((uint32_t)0x00000000u)
/*----XRDY Tokens----*/
#define CSL_MCASP_SRCTL0_XRDY_DATA ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL0_XRDY_EMPTY ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL0_DISMOD_MASK ((uint32_t)0x0000000Cu)
#define CSL_MCASP_SRCTL0_DISMOD_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL0_DISMOD_RESETVAL ((uint32_t)0x00000000u)
/*----DISMOD Tokens----*/
#define CSL_MCASP_SRCTL0_DISMOD_3STATE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL0_DISMOD_RSV ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL0_DISMOD_LOW ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL0_DISMOD_HIGH ((uint32_t)0x00000003u)

#define CSL_MCASP_SRCTL0_SRMOD_MASK ((uint32_t)0x00000003u)
#define CSL_MCASP_SRCTL0_SRMOD_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL0_SRMOD_RESETVAL ((uint32_t)0x00000000u)
/*----SRMOD Tokens----*/
#define CSL_MCASP_SRCTL0_SRMOD_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL0_SRMOD_XMT ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL0_SRMOD_RCV ((uint32_t)0x00000002u)

#define CSL_MCASP_SRCTL0_RESETVAL ((uint32_t)0x00000000u)

/* SRCTL1 */


#define CSL_MCASP_SRCTL1_RRDY_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_SRCTL1_RRDY_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_SRCTL1_RRDY_RESETVAL ((uint32_t)0x00000000u)
/*----RRDY Tokens----*/
#define CSL_MCASP_SRCTL1_RRDY_EMPTY ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL1_RRDY_DATA ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL1_XRDY_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_SRCTL1_XRDY_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_SRCTL1_XRDY_RESETVAL ((uint32_t)0x00000000u)
/*----XRDY Tokens----*/
#define CSL_MCASP_SRCTL1_XRDY_DATA ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL1_XRDY_EMPTY ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL1_DISMOD_MASK ((uint32_t)0x0000000Cu)
#define CSL_MCASP_SRCTL1_DISMOD_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL1_DISMOD_RESETVAL ((uint32_t)0x00000000u)
/*----DISMOD Tokens----*/
#define CSL_MCASP_SRCTL1_DISMOD_3STATE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL1_DISMOD_RSV ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL1_DISMOD_LOW ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL1_DISMOD_HIGH ((uint32_t)0x00000003u)

#define CSL_MCASP_SRCTL1_SRMOD_MASK ((uint32_t)0x00000003u)
#define CSL_MCASP_SRCTL1_SRMOD_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL1_SRMOD_RESETVAL ((uint32_t)0x00000000u)
/*----SRMOD Tokens----*/
#define CSL_MCASP_SRCTL1_SRMOD_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL1_SRMOD_XMT ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL1_SRMOD_RCV ((uint32_t)0x00000002u)

#define CSL_MCASP_SRCTL1_RESETVAL ((uint32_t)0x00000000u)

/* SRCTL2 */


#define CSL_MCASP_SRCTL2_RRDY_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_SRCTL2_RRDY_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_SRCTL2_RRDY_RESETVAL ((uint32_t)0x00000000u)
/*----RRDY Tokens----*/
#define CSL_MCASP_SRCTL2_RRDY_EMPTY ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL2_RRDY_DATA ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL2_XRDY_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_SRCTL2_XRDY_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_SRCTL2_XRDY_RESETVAL ((uint32_t)0x00000000u)
/*----XRDY Tokens----*/
#define CSL_MCASP_SRCTL2_XRDY_DATA ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL2_XRDY_EMPTY ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL2_DISMOD_MASK ((uint32_t)0x0000000Cu)
#define CSL_MCASP_SRCTL2_DISMOD_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL2_DISMOD_RESETVAL ((uint32_t)0x00000000u)
/*----DISMOD Tokens----*/
#define CSL_MCASP_SRCTL2_DISMOD_3STATE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL2_DISMOD_RSV ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL2_DISMOD_LOW ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL2_DISMOD_HIGH ((uint32_t)0x00000003u)

#define CSL_MCASP_SRCTL2_SRMOD_MASK ((uint32_t)0x00000003u)
#define CSL_MCASP_SRCTL2_SRMOD_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL2_SRMOD_RESETVAL ((uint32_t)0x00000000u)
/*----SRMOD Tokens----*/
#define CSL_MCASP_SRCTL2_SRMOD_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL2_SRMOD_XMT ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL2_SRMOD_RCV ((uint32_t)0x00000002u)

#define CSL_MCASP_SRCTL2_RESETVAL ((uint32_t)0x00000000u)

/* SRCTL3 */


#define CSL_MCASP_SRCTL3_RRDY_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_SRCTL3_RRDY_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_SRCTL3_RRDY_RESETVAL ((uint32_t)0x00000000u)
/*----RRDY Tokens----*/
#define CSL_MCASP_SRCTL3_RRDY_EMPTY ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL3_RRDY_DATA ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL3_XRDY_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_SRCTL3_XRDY_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_SRCTL3_XRDY_RESETVAL ((uint32_t)0x00000000u)
/*----XRDY Tokens----*/
#define CSL_MCASP_SRCTL3_XRDY_DATA ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL3_XRDY_EMPTY ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL3_DISMOD_MASK ((uint32_t)0x0000000Cu)
#define CSL_MCASP_SRCTL3_DISMOD_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL3_DISMOD_RESETVAL ((uint32_t)0x00000000u)
/*----DISMOD Tokens----*/
#define CSL_MCASP_SRCTL3_DISMOD_3STATE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL3_DISMOD_RSV ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL3_DISMOD_LOW ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL3_DISMOD_HIGH ((uint32_t)0x00000003u)

#define CSL_MCASP_SRCTL3_SRMOD_MASK ((uint32_t)0x00000003u)
#define CSL_MCASP_SRCTL3_SRMOD_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL3_SRMOD_RESETVAL ((uint32_t)0x00000000u)
/*----SRMOD Tokens----*/
#define CSL_MCASP_SRCTL3_SRMOD_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL3_SRMOD_XMT ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL3_SRMOD_RCV ((uint32_t)0x00000002u)

#define CSL_MCASP_SRCTL3_RESETVAL ((uint32_t)0x00000000u)

/* SRCTL4 */


#define CSL_MCASP_SRCTL4_RRDY_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_SRCTL4_RRDY_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_SRCTL4_RRDY_RESETVAL ((uint32_t)0x00000000u)
/*----RRDY Tokens----*/
#define CSL_MCASP_SRCTL4_RRDY_EMPTY ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL4_RRDY_DATA ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL4_XRDY_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_SRCTL4_XRDY_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_SRCTL4_XRDY_RESETVAL ((uint32_t)0x00000000u)
/*----XRDY Tokens----*/
#define CSL_MCASP_SRCTL4_XRDY_DATA ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL4_XRDY_EMPTY ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL4_DISMOD_MASK ((uint32_t)0x0000000Cu)
#define CSL_MCASP_SRCTL4_DISMOD_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL4_DISMOD_RESETVAL ((uint32_t)0x00000000u)
/*----DISMOD Tokens----*/
#define CSL_MCASP_SRCTL4_DISMOD_3STATE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL4_DISMOD_RSV ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL4_DISMOD_LOW ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL4_DISMOD_HIGH ((uint32_t)0x00000003u)

#define CSL_MCASP_SRCTL4_SRMOD_MASK ((uint32_t)0x00000003u)
#define CSL_MCASP_SRCTL4_SRMOD_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL4_SRMOD_RESETVAL ((uint32_t)0x00000000u)
/*----SRMOD Tokens----*/
#define CSL_MCASP_SRCTL4_SRMOD_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL4_SRMOD_XMT ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL4_SRMOD_RCV ((uint32_t)0x00000002u)

#define CSL_MCASP_SRCTL4_RESETVAL ((uint32_t)0x00000000u)

/* SRCTL5 */


#define CSL_MCASP_SRCTL5_RRDY_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_SRCTL5_RRDY_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_SRCTL5_RRDY_RESETVAL ((uint32_t)0x00000000u)
/*----RRDY Tokens----*/
#define CSL_MCASP_SRCTL5_RRDY_EMPTY ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL5_RRDY_DATA ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL5_XRDY_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_SRCTL5_XRDY_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_SRCTL5_XRDY_RESETVAL ((uint32_t)0x00000000u)
/*----XRDY Tokens----*/
#define CSL_MCASP_SRCTL5_XRDY_DATA ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL5_XRDY_EMPTY ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL5_DISMOD_MASK ((uint32_t)0x0000000Cu)
#define CSL_MCASP_SRCTL5_DISMOD_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL5_DISMOD_RESETVAL ((uint32_t)0x00000000u)
/*----DISMOD Tokens----*/
#define CSL_MCASP_SRCTL5_DISMOD_3STATE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL5_DISMOD_RSV ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL5_DISMOD_LOW ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL5_DISMOD_HIGH ((uint32_t)0x00000003u)

#define CSL_MCASP_SRCTL5_SRMOD_MASK ((uint32_t)0x00000003u)
#define CSL_MCASP_SRCTL5_SRMOD_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL5_SRMOD_RESETVAL ((uint32_t)0x00000000u)
/*----SRMOD Tokens----*/
#define CSL_MCASP_SRCTL5_SRMOD_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL5_SRMOD_XMT ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL5_SRMOD_RCV ((uint32_t)0x00000002u)

#define CSL_MCASP_SRCTL5_RESETVAL ((uint32_t)0x00000000u)

/* SRCTL6 */


#define CSL_MCASP_SRCTL6_RRDY_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_SRCTL6_RRDY_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_SRCTL6_RRDY_RESETVAL ((uint32_t)0x00000000u)
/*----RRDY Tokens----*/
#define CSL_MCASP_SRCTL6_RRDY_EMPTY ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL6_RRDY_DATA ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL6_XRDY_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_SRCTL6_XRDY_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_SRCTL6_XRDY_RESETVAL ((uint32_t)0x00000000u)
/*----XRDY Tokens----*/
#define CSL_MCASP_SRCTL6_XRDY_DATA ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL6_XRDY_EMPTY ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL6_DISMOD_MASK ((uint32_t)0x0000000Cu)
#define CSL_MCASP_SRCTL6_DISMOD_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL6_DISMOD_RESETVAL ((uint32_t)0x00000000u)
/*----DISMOD Tokens----*/
#define CSL_MCASP_SRCTL6_DISMOD_3STATE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL6_DISMOD_RSV ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL6_DISMOD_LOW ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL6_DISMOD_HIGH ((uint32_t)0x00000003u)

#define CSL_MCASP_SRCTL6_SRMOD_MASK ((uint32_t)0x00000003u)
#define CSL_MCASP_SRCTL6_SRMOD_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL6_SRMOD_RESETVAL ((uint32_t)0x00000000u)
/*----SRMOD Tokens----*/
#define CSL_MCASP_SRCTL6_SRMOD_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL6_SRMOD_XMT ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL6_SRMOD_RCV ((uint32_t)0x00000002u)

#define CSL_MCASP_SRCTL6_RESETVAL ((uint32_t)0x00000000u)

/* SRCTL7 */


#define CSL_MCASP_SRCTL7_RRDY_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_SRCTL7_RRDY_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_SRCTL7_RRDY_RESETVAL ((uint32_t)0x00000000u)
/*----RRDY Tokens----*/
#define CSL_MCASP_SRCTL7_RRDY_EMPTY ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL7_RRDY_DATA ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL7_XRDY_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_SRCTL7_XRDY_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_SRCTL7_XRDY_RESETVAL ((uint32_t)0x00000000u)
/*----XRDY Tokens----*/
#define CSL_MCASP_SRCTL7_XRDY_DATA ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL7_XRDY_EMPTY ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL7_DISMOD_MASK ((uint32_t)0x0000000Cu)
#define CSL_MCASP_SRCTL7_DISMOD_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL7_DISMOD_RESETVAL ((uint32_t)0x00000000u)
/*----DISMOD Tokens----*/
#define CSL_MCASP_SRCTL7_DISMOD_3STATE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL7_DISMOD_RSV ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL7_DISMOD_LOW ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL7_DISMOD_HIGH ((uint32_t)0x00000003u)

#define CSL_MCASP_SRCTL7_SRMOD_MASK ((uint32_t)0x00000003u)
#define CSL_MCASP_SRCTL7_SRMOD_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL7_SRMOD_RESETVAL ((uint32_t)0x00000000u)
/*----SRMOD Tokens----*/
#define CSL_MCASP_SRCTL7_SRMOD_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL7_SRMOD_XMT ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL7_SRMOD_RCV ((uint32_t)0x00000002u)

#define CSL_MCASP_SRCTL7_RESETVAL ((uint32_t)0x00000000u)

/* SRCTL8 */


#define CSL_MCASP_SRCTL8_RRDY_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_SRCTL8_RRDY_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_SRCTL8_RRDY_RESETVAL ((uint32_t)0x00000000u)
/*----RRDY Tokens----*/
#define CSL_MCASP_SRCTL8_RRDY_EMPTY ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL8_RRDY_DATA ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL8_XRDY_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_SRCTL8_XRDY_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_SRCTL8_XRDY_RESETVAL ((uint32_t)0x00000000u)
/*----XRDY Tokens----*/
#define CSL_MCASP_SRCTL8_XRDY_DATA ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL8_XRDY_EMPTY ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL8_DISMOD_MASK ((uint32_t)0x0000000Cu)
#define CSL_MCASP_SRCTL8_DISMOD_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL8_DISMOD_RESETVAL ((uint32_t)0x00000000u)
/*----DISMOD Tokens----*/
#define CSL_MCASP_SRCTL8_DISMOD_3STATE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL8_DISMOD_RSV ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL8_DISMOD_LOW ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL8_DISMOD_HIGH ((uint32_t)0x00000003u)

#define CSL_MCASP_SRCTL8_SRMOD_MASK ((uint32_t)0x00000003u)
#define CSL_MCASP_SRCTL8_SRMOD_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL8_SRMOD_RESETVAL ((uint32_t)0x00000000u)
/*----SRMOD Tokens----*/
#define CSL_MCASP_SRCTL8_SRMOD_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL8_SRMOD_XMT ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL8_SRMOD_RCV ((uint32_t)0x00000002u)

#define CSL_MCASP_SRCTL8_RESETVAL ((uint32_t)0x00000000u)

/* SRCTL9 */


#define CSL_MCASP_SRCTL9_RRDY_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_SRCTL9_RRDY_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_SRCTL9_RRDY_RESETVAL ((uint32_t)0x00000000u)
/*----RRDY Tokens----*/
#define CSL_MCASP_SRCTL9_RRDY_EMPTY ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL9_RRDY_DATA ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL9_XRDY_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_SRCTL9_XRDY_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_SRCTL9_XRDY_RESETVAL ((uint32_t)0x00000000u)
/*----XRDY Tokens----*/
#define CSL_MCASP_SRCTL9_XRDY_DATA ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL9_XRDY_EMPTY ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL9_DISMOD_MASK ((uint32_t)0x0000000Cu)
#define CSL_MCASP_SRCTL9_DISMOD_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL9_DISMOD_RESETVAL ((uint32_t)0x00000000u)
/*----DISMOD Tokens----*/
#define CSL_MCASP_SRCTL9_DISMOD_3STATE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL9_DISMOD_RSV ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL9_DISMOD_LOW ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL9_DISMOD_HIGH ((uint32_t)0x00000003u)

#define CSL_MCASP_SRCTL9_SRMOD_MASK ((uint32_t)0x00000003u)
#define CSL_MCASP_SRCTL9_SRMOD_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL9_SRMOD_RESETVAL ((uint32_t)0x00000000u)
/*----SRMOD Tokens----*/
#define CSL_MCASP_SRCTL9_SRMOD_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL9_SRMOD_XMT ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL9_SRMOD_RCV ((uint32_t)0x00000002u)

#define CSL_MCASP_SRCTL9_RESETVAL ((uint32_t)0x00000000u)
/* SRCTL10 */


#define CSL_MCASP_SRCTL10_RRDY_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_SRCTL10_RRDY_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_SRCTL10_RRDY_RESETVAL ((uint32_t)0x00000000u)
/*----RRDY Tokens----*/
#define CSL_MCASP_SRCTL10_RRDY_EMPTY ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL10_RRDY_DATA ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL10_XRDY_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_SRCTL10_XRDY_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_SRCTL10_XRDY_RESETVAL ((uint32_t)0x00000000u)
/*----XRDY Tokens----*/
#define CSL_MCASP_SRCTL10_XRDY_DATA ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL10_XRDY_EMPTY ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL10_DISMOD_MASK ((uint32_t)0x0000000Cu)
#define CSL_MCASP_SRCTL10_DISMOD_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL10_DISMOD_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL10_DISMOD_3STATE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL10_DISMOD_RSV ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL10_DISMOD_LOW ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL10_DISMOD_HIGH ((uint32_t)0x00000003u)

#define CSL_MCASP_SRCTL10_SRMOD_MASK ((uint32_t)0x00000003u)
#define CSL_MCASP_SRCTL10_SRMOD_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL10_SRMOD_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL10_SRMOD_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL10_SRMOD_XMT ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL10_SRMOD_RCV ((uint32_t)0x00000002u)

#define CSL_MCASP_SRCTL10_RESETVAL ((uint32_t)0x00000000u)

/* SRCTL11 */


#define CSL_MCASP_SRCTL11_RRDY_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_SRCTL11_RRDY_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_SRCTL11_RRDY_RESETVAL ((uint32_t)0x00000000u)
/*----RRDY Tokens----*/
#define CSL_MCASP_SRCTL11_RRDY_EMPTY ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL11_RRDY_DATA ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL11_XRDY_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_SRCTL11_XRDY_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_SRCTL11_XRDY_RESETVAL ((uint32_t)0x00000000u)
/*----XRDY Tokens----*/
#define CSL_MCASP_SRCTL11_XRDY_DATA ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL11_XRDY_EMPTY ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL11_DISMOD_MASK ((uint32_t)0x0000000Cu)
#define CSL_MCASP_SRCTL11_DISMOD_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL11_DISMOD_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL11_DISMOD_3STATE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL11_DISMOD_RSV ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL11_DISMOD_LOW ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL11_DISMOD_HIGH ((uint32_t)0x00000003u)

#define CSL_MCASP_SRCTL11_SRMOD_MASK ((uint32_t)0x00000003u)
#define CSL_MCASP_SRCTL11_SRMOD_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL11_SRMOD_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL11_SRMOD_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL11_SRMOD_XMT ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL11_SRMOD_RCV ((uint32_t)0x00000002u)

#define CSL_MCASP_SRCTL11_RESETVAL ((uint32_t)0x00000000u)

/* SRCTL12 */


#define CSL_MCASP_SRCTL12_RRDY_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_SRCTL12_RRDY_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_SRCTL12_RRDY_RESETVAL ((uint32_t)0x00000000u)
/*----RRDY Tokens----*/
#define CSL_MCASP_SRCTL12_RRDY_EMPTY ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL12_RRDY_DATA ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL12_XRDY_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_SRCTL12_XRDY_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_SRCTL12_XRDY_RESETVAL ((uint32_t)0x00000000u)
/*----XRDY Tokens----*/
#define CSL_MCASP_SRCTL12_XRDY_DATA ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL12_XRDY_EMPTY ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL12_DISMOD_MASK ((uint32_t)0x0000000Cu)
#define CSL_MCASP_SRCTL12_DISMOD_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL12_DISMOD_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL12_DISMOD_3STATE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL12_DISMOD_RSV ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL12_DISMOD_LOW ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL12_DISMOD_HIGH ((uint32_t)0x00000003u)

#define CSL_MCASP_SRCTL12_SRMOD_MASK ((uint32_t)0x00000003u)
#define CSL_MCASP_SRCTL12_SRMOD_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL12_SRMOD_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL12_SRMOD_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL12_SRMOD_XMT ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL12_SRMOD_RCV ((uint32_t)0x00000002u)

#define CSL_MCASP_SRCTL12_RESETVAL ((uint32_t)0x00000000u)

/* SRCTL13 */


#define CSL_MCASP_SRCTL13_RRDY_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_SRCTL13_RRDY_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_SRCTL13_RRDY_RESETVAL ((uint32_t)0x00000000u)
/*----RRDY Tokens----*/
#define CSL_MCASP_SRCTL13_RRDY_EMPTY ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL13_RRDY_DATA ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL13_XRDY_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_SRCTL13_XRDY_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_SRCTL13_XRDY_RESETVAL ((uint32_t)0x00000000u)
/*----XRDY Tokens----*/
#define CSL_MCASP_SRCTL13_XRDY_DATA ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL13_XRDY_EMPTY ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL13_DISMOD_MASK ((uint32_t)0x0000000Cu)
#define CSL_MCASP_SRCTL13_DISMOD_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL13_DISMOD_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL13_DISMOD_3STATE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL13_DISMOD_RSV ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL13_DISMOD_LOW ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL13_DISMOD_HIGH ((uint32_t)0x00000003u)

#define CSL_MCASP_SRCTL13_SRMOD_MASK ((uint32_t)0x00000003u)
#define CSL_MCASP_SRCTL13_SRMOD_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL13_SRMOD_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL13_SRMOD_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL13_SRMOD_XMT ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL13_SRMOD_RCV ((uint32_t)0x00000002u)

#define CSL_MCASP_SRCTL13_RESETVAL ((uint32_t)0x00000000u)

/* SRCTL14 */


#define CSL_MCASP_SRCTL14_RRDY_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_SRCTL14_RRDY_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_SRCTL14_RRDY_RESETVAL ((uint32_t)0x00000000u)
/*----RRDY Tokens----*/
#define CSL_MCASP_SRCTL14_RRDY_EMPTY ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL14_RRDY_DATA ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL14_XRDY_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_SRCTL14_XRDY_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_SRCTL14_XRDY_RESETVAL ((uint32_t)0x00000000u)
/*----XRDY Tokens----*/
#define CSL_MCASP_SRCTL14_XRDY_DATA ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL14_XRDY_EMPTY ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL14_DISMOD_MASK ((uint32_t)0x0000000Cu)
#define CSL_MCASP_SRCTL14_DISMOD_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL14_DISMOD_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL14_DISMOD_3STATE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL14_DISMOD_RSV ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL14_DISMOD_LOW ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL14_DISMOD_HIGH ((uint32_t)0x00000003u)

#define CSL_MCASP_SRCTL14_SRMOD_MASK ((uint32_t)0x00000003u)
#define CSL_MCASP_SRCTL14_SRMOD_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL14_SRMOD_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL14_SRMOD_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL14_SRMOD_XMT ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL14_SRMOD_RCV ((uint32_t)0x00000002u)

#define CSL_MCASP_SRCTL14_RESETVAL ((uint32_t)0x00000000u)

/* SRCTL15 */


#define CSL_MCASP_SRCTL15_RRDY_MASK ((uint32_t)0x00000020u)
#define CSL_MCASP_SRCTL15_RRDY_SHIFT ((uint32_t)0x00000005u)
#define CSL_MCASP_SRCTL15_RRDY_RESETVAL ((uint32_t)0x00000000u)
/*----RRDY Tokens----*/
#define CSL_MCASP_SRCTL15_RRDY_EMPTY ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL15_RRDY_DATA ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL15_XRDY_MASK ((uint32_t)0x00000010u)
#define CSL_MCASP_SRCTL15_XRDY_SHIFT ((uint32_t)0x00000004u)
#define CSL_MCASP_SRCTL15_XRDY_RESETVAL ((uint32_t)0x00000000u)
/*----XRDY Tokens----*/
#define CSL_MCASP_SRCTL15_XRDY_DATA ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL15_XRDY_EMPTY ((uint32_t)0x00000001u)

#define CSL_MCASP_SRCTL15_DISMOD_MASK ((uint32_t)0x0000000Cu)
#define CSL_MCASP_SRCTL15_DISMOD_SHIFT ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL15_DISMOD_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL15_DISMOD_3STATE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL15_DISMOD_RSV ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL15_DISMOD_LOW ((uint32_t)0x00000002u)
#define CSL_MCASP_SRCTL15_DISMOD_HIGH ((uint32_t)0x00000003u)

#define CSL_MCASP_SRCTL15_SRMOD_MASK ((uint32_t)0x00000003u)
#define CSL_MCASP_SRCTL15_SRMOD_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL15_SRMOD_RESETVAL ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL15_SRMOD_INACTIVE ((uint32_t)0x00000000u)
#define CSL_MCASP_SRCTL15_SRMOD_XMT ((uint32_t)0x00000001u)
#define CSL_MCASP_SRCTL15_SRMOD_RCV ((uint32_t)0x00000002u)

#define CSL_MCASP_SRCTL15_RESETVAL ((uint32_t)0x00000000u)

/* XBUF0 */

#define CSL_MCASP_XBUF0_XBUF0_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_XBUF0_XBUF0_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_XBUF0_XBUF0_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_XBUF0_RESETVAL ((uint32_t)0x00000000u)

/* XBUF1 */

#define CSL_MCASP_XBUF1_XBUF1_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_XBUF1_XBUF1_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_XBUF1_XBUF1_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_XBUF1_RESETVAL ((uint32_t)0x00000000u)

/* XBUF2 */

#define CSL_MCASP_XBUF2_XBUF2_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_XBUF2_XBUF2_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_XBUF2_XBUF2_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_XBUF2_RESETVAL ((uint32_t)0x00000000u)

/* XBUF3 */

#define CSL_MCASP_XBUF3_XBUF3_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_XBUF3_XBUF3_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_XBUF3_XBUF3_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_XBUF3_RESETVAL ((uint32_t)0x00000000u)

/* XBUF4 */

#define CSL_MCASP_XBUF4_XBUF4_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_XBUF4_XBUF4_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_XBUF4_XBUF4_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_XBUF4_RESETVAL ((uint32_t)0x00000000u)

/* XBUF5 */

#define CSL_MCASP_XBUF5_XBUF5_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_XBUF5_XBUF5_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_XBUF5_XBUF5_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_XBUF5_RESETVAL ((uint32_t)0x00000000u)

/* XBUF6 */

#define CSL_MCASP_XBUF6_XBUF6_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_XBUF6_XBUF6_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_XBUF6_XBUF6_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_XBUF6_RESETVAL ((uint32_t)0x00000000u)

/* XBUF7 */

#define CSL_MCASP_XBUF7_XBUF7_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_XBUF7_XBUF7_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_XBUF7_XBUF7_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_XBUF7_RESETVAL ((uint32_t)0x00000000u)

/* XBUF8 */

#define CSL_MCASP_XBUF8_XBUF8_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_XBUF8_XBUF8_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_XBUF8_XBUF8_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_XBUF8_RESETVAL ((uint32_t)0x00000000u)

/* XBUF9 */

#define CSL_MCASP_XBUF9_XBUF9_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_XBUF9_XBUF9_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_XBUF9_XBUF9_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_XBUF9_RESETVAL ((uint32_t)0x00000000u)

/* XBUF10 */

#define CSL_MCASP_XBUF10_XBUF10_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_XBUF10_XBUF10_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_XBUF10_XBUF10_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_XBUF10_RESETVAL ((uint32_t)0x00000000u)

/* XBUF11 */

#define CSL_MCASP_XBUF11_XBUF11_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_XBUF11_XBUF11_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_XBUF11_XBUF11_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_XBUF11_RESETVAL ((uint32_t)0x00000000u)

/* XBUF12 */

#define CSL_MCASP_XBUF12_XBUF12_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_XBUF12_XBUF12_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_XBUF12_XBUF12_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_XBUF12_RESETVAL ((uint32_t)0x00000000u)

/* XBUF13 */

#define CSL_MCASP_XBUF13_XBUF13_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_XBUF13_XBUF13_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_XBUF13_XBUF13_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_XBUF13_RESETVAL ((uint32_t)0x00000000u)

/* XBUF14 */

#define CSL_MCASP_XBUF14_XBUF14_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_XBUF14_XBUF14_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_XBUF14_XBUF14_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_XBUF14_RESETVAL ((uint32_t)0x00000000u)

/* XBUF15 */

#define CSL_MCASP_XBUF15_XBUF15_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_XBUF15_XBUF15_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_XBUF15_XBUF15_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_XBUF15_RESETVAL ((uint32_t)0x00000000u)
/* RBUF0 */

#define CSL_MCASP_RBUF0_RBUF0_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_RBUF0_RBUF0_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_RBUF0_RBUF0_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_RBUF0_RESETVAL ((uint32_t)0x00000000u)

/* RBUF1 */

#define CSL_MCASP_RBUF1_RBUF1_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_RBUF1_RBUF1_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_RBUF1_RBUF1_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_RBUF1_RESETVAL ((uint32_t)0x00000000u)

/* RBUF2 */

#define CSL_MCASP_RBUF2_RBUF2_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_RBUF2_RBUF2_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_RBUF2_RBUF2_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_RBUF2_RESETVAL ((uint32_t)0x00000000u)

/* RBUF3 */

#define CSL_MCASP_RBUF3_RBUF3_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_RBUF3_RBUF3_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_RBUF3_RBUF3_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_RBUF3_RESETVAL ((uint32_t)0x00000000u)

/* RBUF4 */

#define CSL_MCASP_RBUF4_RBUF4_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_RBUF4_RBUF4_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_RBUF4_RBUF4_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_RBUF4_RESETVAL ((uint32_t)0x00000000u)

/* RBUF5 */

#define CSL_MCASP_RBUF5_RBUF5_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_RBUF5_RBUF5_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_RBUF5_RBUF5_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_RBUF5_RESETVAL ((uint32_t)0x00000000u)

/* RBUF6 */

#define CSL_MCASP_RBUF6_RBUF6_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_RBUF6_RBUF6_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_RBUF6_RBUF6_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_RBUF6_RESETVAL ((uint32_t)0x00000000u)

/* RBUF7 */

#define CSL_MCASP_RBUF7_RBUF7_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_RBUF7_RBUF7_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_RBUF7_RBUF7_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_RBUF7_RESETVAL ((uint32_t)0x00000000u)

/* RBUF8 */

#define CSL_MCASP_RBUF8_RBUF8_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_RBUF8_RBUF8_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_RBUF8_RBUF8_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_RBUF8_RESETVAL ((uint32_t)0x00000000u)

/* RBUF9 */

#define CSL_MCASP_RBUF9_RBUF9_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_RBUF9_RBUF9_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_RBUF9_RBUF9_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_RBUF9_RESETVAL ((uint32_t)0x00000000u)

/* RBUF10 */

#define CSL_MCASP_RBUF10_RBUF10_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_RBUF10_RBUF10_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_RBUF10_RBUF10_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_RBUF10_RESETVAL ((uint32_t)0x00000000u)

/* RBUF11 */

#define CSL_MCASP_RBUF11_RBUF11_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_RBUF11_RBUF11_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_RBUF11_RBUF11_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_RBUF11_RESETVAL ((uint32_t)0x00000000u)

/* RBUF12 */

#define CSL_MCASP_RBUF12_RBUF12_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_RBUF12_RBUF12_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_RBUF12_RBUF12_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_RBUF12_RESETVAL ((uint32_t)0x00000000u)

/* RBUF13 */

#define CSL_MCASP_RBUF13_RBUF13_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_RBUF13_RBUF13_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_RBUF13_RBUF13_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_RBUF13_RESETVAL ((uint32_t)0x00000000u)

/* RBUF14 */

#define CSL_MCASP_RBUF14_RBUF14_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_RBUF14_RBUF14_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_RBUF14_RBUF14_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_RBUF14_RESETVAL ((uint32_t)0x00000000u)

/* RBUF15 */

#define CSL_MCASP_RBUF15_RBUF15_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_MCASP_RBUF15_RBUF15_SHIFT ((uint32_t)0x00000000u)
#define CSL_MCASP_RBUF15_RBUF15_RESETVAL ((uint32_t)0x00000000u)

#define CSL_MCASP_RBUF15_RESETVAL ((uint32_t)0x00000000u)

/**************************************************************************\
* Field Definition Macros (AFIFO)
\**************************************************************************/

#define CSL_AFIFO_REVID_REV_MASK ((uint32_t)0xFFFFFFFFu)
#define CSL_AFIFO_REVID_REV_SHIFT ((uint32_t)0x00000000u)
#define CSL_AFIFO_REVID_REV_RESETVAL ((uint32_t)0x44311100u)

#define CSL_AFIFO_REVID_RESETVAL ((uint32_t)0x44311100u)


/* WFIFOCTL */

#define CSL_MCASP_WFIFOCTL_WNUMDMA_SHIFT                        ((uint32_t)0u)
#define CSL_MCASP_WFIFOCTL_WNUMDMA_MASK                         ((uint32_t)0x000000FFu)
#define CSL_MCASP_WFIFOCTL_WNUMDMA_RESETVAL                     ((uint32_t)0x00000000u)
#define CSL_MCASP_WFIFOCTL_WNUMDMA_MAX                          ((uint32_t)0x000000ffu)

#define CSL_MCASP_WFIFOCTL_WNUMEVT_SHIFT                        ((uint32_t)8u)
#define CSL_MCASP_WFIFOCTL_WNUMEVT_MASK                         ((uint32_t)0x0000FF00u)
#define CSL_MCASP_WFIFOCTL_WNUMEVT_RESETVAL                     ((uint32_t)0x00000000u)
#define CSL_MCASP_WFIFOCTL_WNUMEVT_MAX                          ((uint32_t)0x000000ffu)

#define CSL_MCASP_WFIFOCTL_WENA_SHIFT                           ((uint32_t)16u)
#define CSL_MCASP_WFIFOCTL_WENA_MASK                            ((uint32_t)0x00010000u)
#define CSL_MCASP_WFIFOCTL_WENA_RESETVAL                        ((uint32_t)0x00000000u)
#define CSL_MCASP_WFIFOCTL_WENA_EN_1_0X0                        ((uint32_t)0x00000000u)
#define CSL_MCASP_WFIFOCTL_WENA_EN_2_0X1                        ((uint32_t)0x00010000u)

#define CSL_MCASP_WFIFOCTL_RESETVAL                             ((uint32_t)0x00000000u)

/* Backward compatibility Defines */
#define CSL_AFIFO_WFIFOCTL_WENA_MASK ((uint32_t)0x00010000u)
#define CSL_AFIFO_WFIFOCTL_WENA_SHIFT ((uint32_t)0x00000010u)
#define CSL_AFIFO_WFIFOCTL_WENA_RESETVAL ((uint32_t)0x00000000u)
#define CSL_AFIFO_WFIFOCTL_WENA_DISABLED ((uint32_t)0x00000000u)
#define CSL_AFIFO_WFIFOCTL_WENA_ENABLED ((uint32_t)0x00000001u)

#define CSL_AFIFO_WFIFOCTL_WNUMEVT_MASK ((uint32_t)0x0000FF00u)
#define CSL_AFIFO_WFIFOCTL_WNUMEVT_SHIFT ((uint32_t)0x00000008u)
#define CSL_AFIFO_WFIFOCTL_WNUMEVT_RESETVAL ((uint32_t)0x00000010u)
#define CSL_AFIFO_WFIFOCTL_WNUMEVT_0WORDS ((uint32_t)0x00000000u)
#define CSL_AFIFO_WFIFOCTL_WNUMEVT_4WORDS ((uint32_t)0x00000004u)
#define CSL_AFIFO_WFIFOCTL_WNUMEVT_8WORDS ((uint32_t)0x00000008u)
#define CSL_AFIFO_WFIFOCTL_WNUMEVT_12WORDS ((uint32_t)0x0000000Cu)
#define CSL_AFIFO_WFIFOCTL_WNUMEVT_16WORDS ((uint32_t)0x00000010u)
#define CSL_AFIFO_WFIFOCTL_WNUMEVT_20WORDS ((uint32_t)0x00000014u)
#define CSL_AFIFO_WFIFOCTL_WNUMEVT_24WORDS ((uint32_t)0x00000018u)
#define CSL_AFIFO_WFIFOCTL_WNUMEVT_28WORDS ((uint32_t)0x0000001Cu)
#define CSL_AFIFO_WFIFOCTL_WNUMEVT_32WORDS ((uint32_t)0x00000020u)
#define CSL_AFIFO_WFIFOCTL_WNUMEVT_36WORDS ((uint32_t)0x00000024u)
#define CSL_AFIFO_WFIFOCTL_WNUMEVT_40WORDS ((uint32_t)0x00000028u)
#define CSL_AFIFO_WFIFOCTL_WNUMEVT_44WORDS ((uint32_t)0x0000002Cu)
#define CSL_AFIFO_WFIFOCTL_WNUMEVT_48WORDS ((uint32_t)0x00000030u)
#define CSL_AFIFO_WFIFOCTL_WNUMEVT_52WORDS ((uint32_t)0x00000034u)
#define CSL_AFIFO_WFIFOCTL_WNUMEVT_56WORDS ((uint32_t)0x00000038u)
#define CSL_AFIFO_WFIFOCTL_WNUMEVT_60WORDS ((uint32_t)0x0000003Cu)
#define CSL_AFIFO_WFIFOCTL_WNUMEVT_64WORDS ((uint32_t)0x00000040u)

#define CSL_AFIFO_WFIFOCTL_WNUMDMA_MASK ((uint32_t)0x000000FFu)
#define CSL_AFIFO_WFIFOCTL_WNUMDMA_SHIFT ((uint32_t)0x00000000u)
#define CSL_AFIFO_WFIFOCTL_WNUMDMA_RESETVAL ((uint32_t)0x00000004u)
#define CSL_AFIFO_WFIFOCTL_WNUMDMA_0WORDS ((uint32_t)0x00000000u)
#define CSL_AFIFO_WFIFOCTL_WNUMDMA_1WORDS ((uint32_t)0x00000001u)
#define CSL_AFIFO_WFIFOCTL_WNUMDMA_2WORDS ((uint32_t)0x00000002u)
#define CSL_AFIFO_WFIFOCTL_WNUMDMA_3WORDS ((uint32_t)0x00000003u)
#define CSL_AFIFO_WFIFOCTL_WNUMDMA_4WORDS ((uint32_t)0x00000004u)
#define CSL_AFIFO_WFIFOCTL_WNUMDMA_5WORDS ((uint32_t)0x00000005u)
#define CSL_AFIFO_WFIFOCTL_WNUMDMA_6WORDS ((uint32_t)0x00000006u)
#define CSL_AFIFO_WFIFOCTL_WNUMDMA_7WORDS ((uint32_t)0x00000007u)
#define CSL_AFIFO_WFIFOCTL_WNUMDMA_8WORDS ((uint32_t)0x00000008u)
#define CSL_AFIFO_WFIFOCTL_WNUMDMA_9WORDS ((uint32_t)0x00000009u)
#define CSL_AFIFO_WFIFOCTL_WNUMDMA_10WORDS ((uint32_t)0x0000000Au)
#define CSL_AFIFO_WFIFOCTL_WNUMDMA_11WORDS ((uint32_t)0x0000000Bu)
#define CSL_AFIFO_WFIFOCTL_WNUMDMA_12WORDS ((uint32_t)0x0000000Cu)
#define CSL_AFIFO_WFIFOCTL_WNUMDMA_13WORDS ((uint32_t)0x0000000Du)
#define CSL_AFIFO_WFIFOCTL_WNUMDMA_14WORDS ((uint32_t)0x0000000Eu)
#define CSL_AFIFO_WFIFOCTL_WNUMDMA_15WORDS ((uint32_t)0x0000000Fu)
#define CSL_AFIFO_WFIFOCTL_WNUMDMA_16WORDS ((uint32_t)0x00000010u)

#define CSL_AFIFO_WFIFOCTL_RESETVAL ((uint32_t)0x00001004u)

/* WFIFOSTS */


#define CSL_AFIFO_WFIFOSTS_WLVL_MASK ((uint32_t)0x000000FFu)
#define CSL_AFIFO_WFIFOSTS_WLVL_SHIFT ((uint32_t)0x00000000u)
#define CSL_AFIFO_WFIFOSTS_WLVL_RESETVAL ((uint32_t)0x00000000u)

#define CSL_AFIFO_WFIFOSTS_RESETVAL ((uint32_t)0x00000000u)

/* RFIFOCTL */

#define CSL_MCASP_RFIFOCTL_RNUMDMA_SHIFT                        ((uint32_t)0u)
#define CSL_MCASP_RFIFOCTL_RNUMDMA_MASK                         ((uint32_t)0x000000FFu)
#define CSL_MCASP_RFIFOCTL_RNUMDMA_RESETVAL                     ((uint32_t)0x00000000u)
#define CSL_MCASP_RFIFOCTL_RNUMDMA_MAX                          ((uint32_t)0x000000ffu)

#define CSL_MCASP_RFIFOCTL_RNUMEVT_SHIFT                        ((uint32_t)8u)
#define CSL_MCASP_RFIFOCTL_RNUMEVT_MASK                         ((uint32_t)0x0000FF00u)
#define CSL_MCASP_RFIFOCTL_RNUMEVT_RESETVAL                     ((uint32_t)0x00000000u)
#define CSL_MCASP_RFIFOCTL_RNUMEVT_MAX                          ((uint32_t)0x000000ffu)

#define CSL_MCASP_RFIFOCTL_RENA_SHIFT                           ((uint32_t)16u)
#define CSL_MCASP_RFIFOCTL_RENA_MASK                            ((uint32_t)0x00010000u)
#define CSL_MCASP_RFIFOCTL_RENA_RESETVAL                        ((uint32_t)0x00000000u)
#define CSL_MCASP_RFIFOCTL_RENA_EN_1_0X0                        ((uint32_t)0x00000000u)
#define CSL_MCASP_RFIFOCTL_RENA_EN_2_0X1                        ((uint32_t)0x00010000u)

#define CSL_MCASP_RFIFOCTL_RESETVAL                             ((uint32_t)0x00000000u)


#define CSL_AFIFO_RFIFOCTL_RENA_MASK ((uint32_t)0x00010000u)
#define CSL_AFIFO_RFIFOCTL_RENA_SHIFT ((uint32_t)0x00000010u)
#define CSL_AFIFO_RFIFOCTL_RENA_RESETVAL ((uint32_t)0x00000000u)
#define CSL_AFIFO_RFIFOCTL_RENA_DISABLED ((uint32_t)0x00000000u)
#define CSL_AFIFO_RFIFOCTL_RENA_ENABLED ((uint32_t)0x00000001u)

#define CSL_AFIFO_RFIFOCTL_RNUMEVT_MASK ((uint32_t)0x0000FF00u)
#define CSL_AFIFO_RFIFOCTL_RNUMEVT_SHIFT ((uint32_t)0x00000008u)
#define CSL_AFIFO_RFIFOCTL_RNUMEVT_RESETVAL ((uint32_t)0x00000010u)
#define CSL_AFIFO_RFIFOCTL_RNUMEVT_0WORDS ((uint32_t)0x00000000u)
#define CSL_AFIFO_RFIFOCTL_RNUMEVT_4WORDS ((uint32_t)0x00000004u)
#define CSL_AFIFO_RFIFOCTL_RNUMEVT_8WORDS ((uint32_t)0x00000008u)
#define CSL_AFIFO_RFIFOCTL_RNUMEVT_12WORDS ((uint32_t)0x0000000Cu)
#define CSL_AFIFO_RFIFOCTL_RNUMEVT_16WORDS ((uint32_t)0x00000010u)
#define CSL_AFIFO_RFIFOCTL_RNUMEVT_20WORDS ((uint32_t)0x00000014u)
#define CSL_AFIFO_RFIFOCTL_RNUMEVT_24WORDS ((uint32_t)0x00000018u)
#define CSL_AFIFO_RFIFOCTL_RNUMEVT_28WORDS ((uint32_t)0x0000001Cu)
#define CSL_AFIFO_RFIFOCTL_RNUMEVT_32WORDS ((uint32_t)0x00000020u)
#define CSL_AFIFO_RFIFOCTL_RNUMEVT_36WORDS ((uint32_t)0x00000024u)
#define CSL_AFIFO_RFIFOCTL_RNUMEVT_40WORDS ((uint32_t)0x00000028u)
#define CSL_AFIFO_RFIFOCTL_RNUMEVT_44WORDS ((uint32_t)0x0000002Cu)
#define CSL_AFIFO_RFIFOCTL_RNUMEVT_48WORDS ((uint32_t)0x00000030u)
#define CSL_AFIFO_RFIFOCTL_RNUMEVT_52WORDS ((uint32_t)0x00000034u)
#define CSL_AFIFO_RFIFOCTL_RNUMEVT_56WORDS ((uint32_t)0x00000038u)
#define CSL_AFIFO_RFIFOCTL_RNUMEVT_60WORDS ((uint32_t)0x0000003Cu)
#define CSL_AFIFO_RFIFOCTL_RNUMEVT_64WORDS ((uint32_t)0x00000040u)

#define CSL_AFIFO_RFIFOCTL_RNUMDMA_MASK ((uint32_t)0x000000FFu)
#define CSL_AFIFO_RFIFOCTL_RNUMDMA_SHIFT ((uint32_t)0x00000000u)
#define CSL_AFIFO_RFIFOCTL_RNUMDMA_RESETVAL ((uint32_t)0x00000004u)
#define CSL_AFIFO_RFIFOCTL_RNUMDMA_0WORDS ((uint32_t)0x00000000u)
#define CSL_AFIFO_RFIFOCTL_RNUMDMA_1WORDS ((uint32_t)0x00000001u)
#define CSL_AFIFO_RFIFOCTL_RNUMDMA_2WORDS ((uint32_t)0x00000002u)
#define CSL_AFIFO_RFIFOCTL_RNUMDMA_3WORDS ((uint32_t)0x00000003u)
#define CSL_AFIFO_RFIFOCTL_RNUMDMA_4WORDS ((uint32_t)0x00000004u)
#define CSL_AFIFO_RFIFOCTL_RNUMDMA_5WORDS ((uint32_t)0x00000005u)
#define CSL_AFIFO_RFIFOCTL_RNUMDMA_6WORDS ((uint32_t)0x00000006u)
#define CSL_AFIFO_RFIFOCTL_RNUMDMA_7WORDS ((uint32_t)0x00000007u)
#define CSL_AFIFO_RFIFOCTL_RNUMDMA_8WORDS ((uint32_t)0x00000008u)
#define CSL_AFIFO_RFIFOCTL_RNUMDMA_9WORDS ((uint32_t)0x00000009u)
#define CSL_AFIFO_RFIFOCTL_RNUMDMA_10WORDS ((uint32_t)0x0000000Au)
#define CSL_AFIFO_RFIFOCTL_RNUMDMA_11WORDS ((uint32_t)0x0000000Bu)
#define CSL_AFIFO_RFIFOCTL_RNUMDMA_12WORDS ((uint32_t)0x0000000Cu)
#define CSL_AFIFO_RFIFOCTL_RNUMDMA_13WORDS ((uint32_t)0x0000000Du)
#define CSL_AFIFO_RFIFOCTL_RNUMDMA_14WORDS ((uint32_t)0x0000000Eu)
#define CSL_AFIFO_RFIFOCTL_RNUMDMA_15WORDS ((uint32_t)0x0000000Fu)
#define CSL_AFIFO_RFIFOCTL_RNUMDMA_16WORDS ((uint32_t)0x00000010u)

#define CSL_AFIFO_RFIFOCTL_RESETVAL ((uint32_t)0x00001004u)

/* RFIFOSTS */

#define CSL_MCASP_RFIFOSTS_RLVL_SHIFT                           ((uint32_t)0u)
#define CSL_MCASP_RFIFOSTS_RLVL_MASK                            ((uint32_t)0x000000FFu)
#define CSL_MCASP_RFIFOSTS_RLVL_RESETVAL                        ((uint32_t)0x00000000u)
#define CSL_MCASP_RFIFOSTS_RLVL_MAX                             ((uint32_t)0x000000ffu)

#define CSL_MCASP_RFIFOSTS_RESETVAL                             ((uint32_t)0x00000000u)

#define CSL_AFIFO_RFIFOSTS_RLVL_MASK ((uint32_t)0x000000FFu)
#define CSL_AFIFO_RFIFOSTS_RLVL_SHIFT ((uint32_t)0x00000000u)
#define CSL_AFIFO_RFIFOSTS_RLVL_RESETVAL ((uint32_t)0x00000000u)

#define CSL_AFIFO_RFIFOSTS_RESETVAL ((uint32_t)0x00000000u)

#ifdef __cplusplus
}
#endif
#endif
