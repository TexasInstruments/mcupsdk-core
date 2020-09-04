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
 *  Name        : cslr_sdfm.h
*/
#ifndef CSLR_SDFM_H_
#define CSLR_SDFM_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t SDIFLG;
    volatile uint32_t SDIFLGCLR;
    volatile uint16_t SDCTL;
    volatile uint8_t  Resv_12[2];
    volatile uint16_t SDMFILEN;
    volatile uint16_t SDSTATUS;
    volatile uint8_t  Resv_32[16];
    volatile uint16_t SDCTLPARM1;
    volatile uint16_t SDDFPARM1;
    volatile uint16_t SDDPARM1;
    volatile uint16_t SDFLT1CMPH1;
    volatile uint16_t SDFLT1CMPL1;
    volatile uint16_t SDCPARM1;
    volatile uint32_t SDDATA1;
    volatile uint32_t SDDATFIFO1;
    volatile uint16_t SDCDATA1;
    volatile uint16_t SDFLT1CMPH2;
    volatile uint16_t SDFLT1CMPHZ;
    volatile uint16_t SDFIFOCTL1;
    volatile uint16_t SDSYNC1;
    volatile uint16_t SDFLT1CMPL2;
    volatile uint16_t SDCTLPARM2;
    volatile uint16_t SDDFPARM2;
    volatile uint16_t SDDPARM2;
    volatile uint16_t SDFLT2CMPH1;
    volatile uint16_t SDFLT2CMPL1;
    volatile uint16_t SDCPARM2;
    volatile uint32_t SDDATA2;
    volatile uint32_t SDDATFIFO2;
    volatile uint16_t SDCDATA2;
    volatile uint16_t SDFLT2CMPH2;
    volatile uint16_t SDFLT2CMPHZ;
    volatile uint16_t SDFIFOCTL2;
    volatile uint16_t SDSYNC2;
    volatile uint16_t SDFLT2CMPL2;
    volatile uint16_t SDCTLPARM3;
    volatile uint16_t SDDFPARM3;
    volatile uint16_t SDDPARM3;
    volatile uint16_t SDFLT3CMPH1;
    volatile uint16_t SDFLT3CMPL1;
    volatile uint16_t SDCPARM3;
    volatile uint32_t SDDATA3;
    volatile uint32_t SDDATFIFO3;
    volatile uint16_t SDCDATA3;
    volatile uint16_t SDFLT3CMPH2;
    volatile uint16_t SDFLT3CMPHZ;
    volatile uint16_t SDFIFOCTL3;
    volatile uint16_t SDSYNC3;
    volatile uint16_t SDFLT3CMPL2;
    volatile uint16_t SDCTLPARM4;
    volatile uint16_t SDDFPARM4;
    volatile uint16_t SDDPARM4;
    volatile uint16_t SDFLT4CMPH1;
    volatile uint16_t SDFLT4CMPL1;
    volatile uint16_t SDCPARM4;
    volatile uint32_t SDDATA4;
    volatile uint32_t SDDATFIFO4;
    volatile uint16_t SDCDATA4;
    volatile uint16_t SDFLT4CMPH2;
    volatile uint16_t SDFLT4CMPHZ;
    volatile uint16_t SDFIFOCTL4;
    volatile uint16_t SDSYNC4;
    volatile uint16_t SDFLT4CMPL2;
    volatile uint8_t  Resv_192[32];
    volatile uint16_t SDCOMP1CTL;
    volatile uint16_t SDCOMP1EVT2FLTCTL;
    volatile uint16_t SDCOMP1EVT2FLTCLKCTL;
    volatile uint16_t SDCOMP1EVT1FLTCTL;
    volatile uint16_t SDCOMP1EVT1FLTCLKCTL;
    volatile uint8_t  Resv_206[4];
    volatile uint16_t SDCOMP1LOCK;
    volatile uint16_t SDCOMP2CTL;
    volatile uint16_t SDCOMP2EVT2FLTCTL;
    volatile uint16_t SDCOMP2EVT2FLTCLKCTL;
    volatile uint16_t SDCOMP2EVT1FLTCTL;
    volatile uint16_t SDCOMP2EVT1FLTCLKCTL;
    volatile uint8_t  Resv_222[4];
    volatile uint16_t SDCOMP2LOCK;
    volatile uint16_t SDCOMP3CTL;
    volatile uint16_t SDCOMP3EVT2FLTCTL;
    volatile uint16_t SDCOMP3EVT2FLTCLKCTL;
    volatile uint16_t SDCOMP3EVT1FLTCTL;
    volatile uint16_t SDCOMP3EVT1FLTCLKCTL;
    volatile uint8_t  Resv_238[4];
    volatile uint16_t SDCOMP3LOCK;
    volatile uint16_t SDCOMP4CTL;
    volatile uint16_t SDCOMP4EVT2FLTCTL;
    volatile uint16_t SDCOMP4EVT2FLTCLKCTL;
    volatile uint16_t SDCOMP4EVT1FLTCTL;
    volatile uint16_t SDCOMP4EVT1FLTCLKCTL;
    volatile uint8_t  Resv_254[4];
    volatile uint16_t SDCOMP4LOCK;
} CSL_sdfmRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_SDFM_SDIFLG                                                        (0x00000000U)
#define CSL_SDFM_SDIFLGCLR                                                     (0x00000004U)
#define CSL_SDFM_SDCTL                                                         (0x00000008U)
#define CSL_SDFM_SDMFILEN                                                      (0x0000000CU)
#define CSL_SDFM_SDSTATUS                                                      (0x0000000EU)
#define CSL_SDFM_SDCTLPARM1                                                    (0x00000020U)
#define CSL_SDFM_SDDFPARM1                                                     (0x00000022U)
#define CSL_SDFM_SDDPARM1                                                      (0x00000024U)
#define CSL_SDFM_SDFLT1CMPH1                                                   (0x00000026U)
#define CSL_SDFM_SDFLT1CMPL1                                                   (0x00000028U)
#define CSL_SDFM_SDCPARM1                                                      (0x0000002AU)
#define CSL_SDFM_SDDATA1                                                       (0x0000002CU)
#define CSL_SDFM_SDDATFIFO1                                                    (0x00000030U)
#define CSL_SDFM_SDCDATA1                                                      (0x00000034U)
#define CSL_SDFM_SDFLT1CMPH2                                                   (0x00000036U)
#define CSL_SDFM_SDFLT1CMPHZ                                                   (0x00000038U)
#define CSL_SDFM_SDFIFOCTL1                                                    (0x0000003AU)
#define CSL_SDFM_SDSYNC1                                                       (0x0000003CU)
#define CSL_SDFM_SDFLT1CMPL2                                                   (0x0000003EU)
#define CSL_SDFM_SDCTLPARM2                                                    (0x00000040U)
#define CSL_SDFM_SDDFPARM2                                                     (0x00000042U)
#define CSL_SDFM_SDDPARM2                                                      (0x00000044U)
#define CSL_SDFM_SDFLT2CMPH1                                                   (0x00000046U)
#define CSL_SDFM_SDFLT2CMPL1                                                   (0x00000048U)
#define CSL_SDFM_SDCPARM2                                                      (0x0000004AU)
#define CSL_SDFM_SDDATA2                                                       (0x0000004CU)
#define CSL_SDFM_SDDATFIFO2                                                    (0x00000050U)
#define CSL_SDFM_SDCDATA2                                                      (0x00000054U)
#define CSL_SDFM_SDFLT2CMPH2                                                   (0x00000056U)
#define CSL_SDFM_SDFLT2CMPHZ                                                   (0x00000058U)
#define CSL_SDFM_SDFIFOCTL2                                                    (0x0000005AU)
#define CSL_SDFM_SDSYNC2                                                       (0x0000005CU)
#define CSL_SDFM_SDFLT2CMPL2                                                   (0x0000005EU)
#define CSL_SDFM_SDCTLPARM3                                                    (0x00000060U)
#define CSL_SDFM_SDDFPARM3                                                     (0x00000062U)
#define CSL_SDFM_SDDPARM3                                                      (0x00000064U)
#define CSL_SDFM_SDFLT3CMPH1                                                   (0x00000066U)
#define CSL_SDFM_SDFLT3CMPL1                                                   (0x00000068U)
#define CSL_SDFM_SDCPARM3                                                      (0x0000006AU)
#define CSL_SDFM_SDDATA3                                                       (0x0000006CU)
#define CSL_SDFM_SDDATFIFO3                                                    (0x00000070U)
#define CSL_SDFM_SDCDATA3                                                      (0x00000074U)
#define CSL_SDFM_SDFLT3CMPH2                                                   (0x00000076U)
#define CSL_SDFM_SDFLT3CMPHZ                                                   (0x00000078U)
#define CSL_SDFM_SDFIFOCTL3                                                    (0x0000007AU)
#define CSL_SDFM_SDSYNC3                                                       (0x0000007CU)
#define CSL_SDFM_SDFLT3CMPL2                                                   (0x0000007EU)
#define CSL_SDFM_SDCTLPARM4                                                    (0x00000080U)
#define CSL_SDFM_SDDFPARM4                                                     (0x00000082U)
#define CSL_SDFM_SDDPARM4                                                      (0x00000084U)
#define CSL_SDFM_SDFLT4CMPH1                                                   (0x00000086U)
#define CSL_SDFM_SDFLT4CMPL1                                                   (0x00000088U)
#define CSL_SDFM_SDCPARM4                                                      (0x0000008AU)
#define CSL_SDFM_SDDATA4                                                       (0x0000008CU)
#define CSL_SDFM_SDDATFIFO4                                                    (0x00000090U)
#define CSL_SDFM_SDCDATA4                                                      (0x00000094U)
#define CSL_SDFM_SDFLT4CMPH2                                                   (0x00000096U)
#define CSL_SDFM_SDFLT4CMPHZ                                                   (0x00000098U)
#define CSL_SDFM_SDFIFOCTL4                                                    (0x0000009AU)
#define CSL_SDFM_SDSYNC4                                                       (0x0000009CU)
#define CSL_SDFM_SDFLT4CMPL2                                                   (0x0000009EU)
#define CSL_SDFM_SDCOMP1CTL                                                    (0x000000C0U)
#define CSL_SDFM_SDCOMP1EVT2FLTCTL                                             (0x000000C2U)
#define CSL_SDFM_SDCOMP1EVT2FLTCLKCTL                                          (0x000000C4U)
#define CSL_SDFM_SDCOMP1EVT1FLTCTL                                             (0x000000C6U)
#define CSL_SDFM_SDCOMP1EVT1FLTCLKCTL                                          (0x000000C8U)
#define CSL_SDFM_SDCOMP1LOCK                                                   (0x000000CEU)
#define CSL_SDFM_SDCOMP2CTL                                                    (0x000000D0U)
#define CSL_SDFM_SDCOMP2EVT2FLTCTL                                             (0x000000D2U)
#define CSL_SDFM_SDCOMP2EVT2FLTCLKCTL                                          (0x000000D4U)
#define CSL_SDFM_SDCOMP2EVT1FLTCTL                                             (0x000000D6U)
#define CSL_SDFM_SDCOMP2EVT1FLTCLKCTL                                          (0x000000D8U)
#define CSL_SDFM_SDCOMP2LOCK                                                   (0x000000DEU)
#define CSL_SDFM_SDCOMP3CTL                                                    (0x000000E0U)
#define CSL_SDFM_SDCOMP3EVT2FLTCTL                                             (0x000000E2U)
#define CSL_SDFM_SDCOMP3EVT2FLTCLKCTL                                          (0x000000E4U)
#define CSL_SDFM_SDCOMP3EVT1FLTCTL                                             (0x000000E6U)
#define CSL_SDFM_SDCOMP3EVT1FLTCLKCTL                                          (0x000000E8U)
#define CSL_SDFM_SDCOMP3LOCK                                                   (0x000000EEU)
#define CSL_SDFM_SDCOMP4CTL                                                    (0x000000F0U)
#define CSL_SDFM_SDCOMP4EVT2FLTCTL                                             (0x000000F2U)
#define CSL_SDFM_SDCOMP4EVT2FLTCLKCTL                                          (0x000000F4U)
#define CSL_SDFM_SDCOMP4EVT1FLTCTL                                             (0x000000F6U)
#define CSL_SDFM_SDCOMP4EVT1FLTCLKCTL                                          (0x000000F8U)
#define CSL_SDFM_SDCOMP4LOCK                                                   (0x000000FEU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* SDIFLG */

#define CSL_SDFM_SDIFLG_FLT1_FLG_CEVT1_MASK                                    (0x00000001U)
#define CSL_SDFM_SDIFLG_FLT1_FLG_CEVT1_SHIFT                                   (0x00000000U)
#define CSL_SDFM_SDIFLG_FLT1_FLG_CEVT1_RESETVAL                                (0x00000000U)
#define CSL_SDFM_SDIFLG_FLT1_FLG_CEVT1_MAX                                     (0x00000001U)

#define CSL_SDFM_SDIFLG_FLT1_FLG_CEVT2_MASK                                    (0x00000002U)
#define CSL_SDFM_SDIFLG_FLT1_FLG_CEVT2_SHIFT                                   (0x00000001U)
#define CSL_SDFM_SDIFLG_FLT1_FLG_CEVT2_RESETVAL                                (0x00000000U)
#define CSL_SDFM_SDIFLG_FLT1_FLG_CEVT2_MAX                                     (0x00000001U)

#define CSL_SDFM_SDIFLG_FLT2_FLG_CEVT1_MASK                                    (0x00000004U)
#define CSL_SDFM_SDIFLG_FLT2_FLG_CEVT1_SHIFT                                   (0x00000002U)
#define CSL_SDFM_SDIFLG_FLT2_FLG_CEVT1_RESETVAL                                (0x00000000U)
#define CSL_SDFM_SDIFLG_FLT2_FLG_CEVT1_MAX                                     (0x00000001U)

#define CSL_SDFM_SDIFLG_FLT2_FLG_CEVT2_MASK                                    (0x00000008U)
#define CSL_SDFM_SDIFLG_FLT2_FLG_CEVT2_SHIFT                                   (0x00000003U)
#define CSL_SDFM_SDIFLG_FLT2_FLG_CEVT2_RESETVAL                                (0x00000000U)
#define CSL_SDFM_SDIFLG_FLT2_FLG_CEVT2_MAX                                     (0x00000001U)

#define CSL_SDFM_SDIFLG_FLT3_FLG_CEVT1_MASK                                    (0x00000010U)
#define CSL_SDFM_SDIFLG_FLT3_FLG_CEVT1_SHIFT                                   (0x00000004U)
#define CSL_SDFM_SDIFLG_FLT3_FLG_CEVT1_RESETVAL                                (0x00000000U)
#define CSL_SDFM_SDIFLG_FLT3_FLG_CEVT1_MAX                                     (0x00000001U)

#define CSL_SDFM_SDIFLG_FLT3_FLG_CEVT2_MASK                                    (0x00000020U)
#define CSL_SDFM_SDIFLG_FLT3_FLG_CEVT2_SHIFT                                   (0x00000005U)
#define CSL_SDFM_SDIFLG_FLT3_FLG_CEVT2_RESETVAL                                (0x00000000U)
#define CSL_SDFM_SDIFLG_FLT3_FLG_CEVT2_MAX                                     (0x00000001U)

#define CSL_SDFM_SDIFLG_FLT4_FLG_CEVT1_MASK                                    (0x00000040U)
#define CSL_SDFM_SDIFLG_FLT4_FLG_CEVT1_SHIFT                                   (0x00000006U)
#define CSL_SDFM_SDIFLG_FLT4_FLG_CEVT1_RESETVAL                                (0x00000000U)
#define CSL_SDFM_SDIFLG_FLT4_FLG_CEVT1_MAX                                     (0x00000001U)

#define CSL_SDFM_SDIFLG_FLT4_FLG_CEVT2_MASK                                    (0x00000080U)
#define CSL_SDFM_SDIFLG_FLT4_FLG_CEVT2_SHIFT                                   (0x00000007U)
#define CSL_SDFM_SDIFLG_FLT4_FLG_CEVT2_RESETVAL                                (0x00000000U)
#define CSL_SDFM_SDIFLG_FLT4_FLG_CEVT2_MAX                                     (0x00000001U)

#define CSL_SDFM_SDIFLG_MF1_MASK                                               (0x00000100U)
#define CSL_SDFM_SDIFLG_MF1_SHIFT                                              (0x00000008U)
#define CSL_SDFM_SDIFLG_MF1_RESETVAL                                           (0x00000000U)
#define CSL_SDFM_SDIFLG_MF1_MAX                                                (0x00000001U)

#define CSL_SDFM_SDIFLG_MF2_MASK                                               (0x00000200U)
#define CSL_SDFM_SDIFLG_MF2_SHIFT                                              (0x00000009U)
#define CSL_SDFM_SDIFLG_MF2_RESETVAL                                           (0x00000000U)
#define CSL_SDFM_SDIFLG_MF2_MAX                                                (0x00000001U)

#define CSL_SDFM_SDIFLG_MF3_MASK                                               (0x00000400U)
#define CSL_SDFM_SDIFLG_MF3_SHIFT                                              (0x0000000AU)
#define CSL_SDFM_SDIFLG_MF3_RESETVAL                                           (0x00000000U)
#define CSL_SDFM_SDIFLG_MF3_MAX                                                (0x00000001U)

#define CSL_SDFM_SDIFLG_MF4_MASK                                               (0x00000800U)
#define CSL_SDFM_SDIFLG_MF4_SHIFT                                              (0x0000000BU)
#define CSL_SDFM_SDIFLG_MF4_RESETVAL                                           (0x00000000U)
#define CSL_SDFM_SDIFLG_MF4_MAX                                                (0x00000001U)

#define CSL_SDFM_SDIFLG_AF1_MASK                                               (0x00001000U)
#define CSL_SDFM_SDIFLG_AF1_SHIFT                                              (0x0000000CU)
#define CSL_SDFM_SDIFLG_AF1_RESETVAL                                           (0x00000000U)
#define CSL_SDFM_SDIFLG_AF1_MAX                                                (0x00000001U)

#define CSL_SDFM_SDIFLG_AF2_MASK                                               (0x00002000U)
#define CSL_SDFM_SDIFLG_AF2_SHIFT                                              (0x0000000DU)
#define CSL_SDFM_SDIFLG_AF2_RESETVAL                                           (0x00000000U)
#define CSL_SDFM_SDIFLG_AF2_MAX                                                (0x00000001U)

#define CSL_SDFM_SDIFLG_AF3_MASK                                               (0x00004000U)
#define CSL_SDFM_SDIFLG_AF3_SHIFT                                              (0x0000000EU)
#define CSL_SDFM_SDIFLG_AF3_RESETVAL                                           (0x00000000U)
#define CSL_SDFM_SDIFLG_AF3_MAX                                                (0x00000001U)

#define CSL_SDFM_SDIFLG_AF4_MASK                                               (0x00008000U)
#define CSL_SDFM_SDIFLG_AF4_SHIFT                                              (0x0000000FU)
#define CSL_SDFM_SDIFLG_AF4_RESETVAL                                           (0x00000000U)
#define CSL_SDFM_SDIFLG_AF4_MAX                                                (0x00000001U)

#define CSL_SDFM_SDIFLG_SDFFOVF1_MASK                                          (0x00010000U)
#define CSL_SDFM_SDIFLG_SDFFOVF1_SHIFT                                         (0x00000010U)
#define CSL_SDFM_SDIFLG_SDFFOVF1_RESETVAL                                      (0x00000000U)
#define CSL_SDFM_SDIFLG_SDFFOVF1_MAX                                           (0x00000001U)

#define CSL_SDFM_SDIFLG_SDFFOVF2_MASK                                          (0x00020000U)
#define CSL_SDFM_SDIFLG_SDFFOVF2_SHIFT                                         (0x00000011U)
#define CSL_SDFM_SDIFLG_SDFFOVF2_RESETVAL                                      (0x00000000U)
#define CSL_SDFM_SDIFLG_SDFFOVF2_MAX                                           (0x00000001U)

#define CSL_SDFM_SDIFLG_SDFFOVF3_MASK                                          (0x00040000U)
#define CSL_SDFM_SDIFLG_SDFFOVF3_SHIFT                                         (0x00000012U)
#define CSL_SDFM_SDIFLG_SDFFOVF3_RESETVAL                                      (0x00000000U)
#define CSL_SDFM_SDIFLG_SDFFOVF3_MAX                                           (0x00000001U)

#define CSL_SDFM_SDIFLG_SDFFOVF4_MASK                                          (0x00080000U)
#define CSL_SDFM_SDIFLG_SDFFOVF4_SHIFT                                         (0x00000013U)
#define CSL_SDFM_SDIFLG_SDFFOVF4_RESETVAL                                      (0x00000000U)
#define CSL_SDFM_SDIFLG_SDFFOVF4_MAX                                           (0x00000001U)

#define CSL_SDFM_SDIFLG_SDFFINT1_MASK                                          (0x00100000U)
#define CSL_SDFM_SDIFLG_SDFFINT1_SHIFT                                         (0x00000014U)
#define CSL_SDFM_SDIFLG_SDFFINT1_RESETVAL                                      (0x00000000U)
#define CSL_SDFM_SDIFLG_SDFFINT1_MAX                                           (0x00000001U)

#define CSL_SDFM_SDIFLG_SDFFINT2_MASK                                          (0x00200000U)
#define CSL_SDFM_SDIFLG_SDFFINT2_SHIFT                                         (0x00000015U)
#define CSL_SDFM_SDIFLG_SDFFINT2_RESETVAL                                      (0x00000000U)
#define CSL_SDFM_SDIFLG_SDFFINT2_MAX                                           (0x00000001U)

#define CSL_SDFM_SDIFLG_SDFFINT3_MASK                                          (0x00400000U)
#define CSL_SDFM_SDIFLG_SDFFINT3_SHIFT                                         (0x00000016U)
#define CSL_SDFM_SDIFLG_SDFFINT3_RESETVAL                                      (0x00000000U)
#define CSL_SDFM_SDIFLG_SDFFINT3_MAX                                           (0x00000001U)

#define CSL_SDFM_SDIFLG_SDFFINT4_MASK                                          (0x00800000U)
#define CSL_SDFM_SDIFLG_SDFFINT4_SHIFT                                         (0x00000017U)
#define CSL_SDFM_SDIFLG_SDFFINT4_RESETVAL                                      (0x00000000U)
#define CSL_SDFM_SDIFLG_SDFFINT4_MAX                                           (0x00000001U)

#define CSL_SDFM_SDIFLG_RESERVED_1_MASK                                        (0x7F000000U)
#define CSL_SDFM_SDIFLG_RESERVED_1_SHIFT                                       (0x00000018U)
#define CSL_SDFM_SDIFLG_RESERVED_1_RESETVAL                                    (0x00000000U)
#define CSL_SDFM_SDIFLG_RESERVED_1_MAX                                         (0x0000007FU)

#define CSL_SDFM_SDIFLG_MIF_MASK                                               (0x80000000U)
#define CSL_SDFM_SDIFLG_MIF_SHIFT                                              (0x0000001FU)
#define CSL_SDFM_SDIFLG_MIF_RESETVAL                                           (0x00000000U)
#define CSL_SDFM_SDIFLG_MIF_MAX                                                (0x00000001U)

#define CSL_SDFM_SDIFLG_RESETVAL                                               (0x00000000U)

/* SDIFLGCLR */

#define CSL_SDFM_SDIFLGCLR_FLT1_FLG_CEVT1_MASK                                 (0x00000001U)
#define CSL_SDFM_SDIFLGCLR_FLT1_FLG_CEVT1_SHIFT                                (0x00000000U)
#define CSL_SDFM_SDIFLGCLR_FLT1_FLG_CEVT1_RESETVAL                             (0x00000000U)
#define CSL_SDFM_SDIFLGCLR_FLT1_FLG_CEVT1_MAX                                  (0x00000001U)

#define CSL_SDFM_SDIFLGCLR_FLT1_FLG_CEVT2_MASK                                 (0x00000002U)
#define CSL_SDFM_SDIFLGCLR_FLT1_FLG_CEVT2_SHIFT                                (0x00000001U)
#define CSL_SDFM_SDIFLGCLR_FLT1_FLG_CEVT2_RESETVAL                             (0x00000000U)
#define CSL_SDFM_SDIFLGCLR_FLT1_FLG_CEVT2_MAX                                  (0x00000001U)

#define CSL_SDFM_SDIFLGCLR_FLT2_FLG_CEVT1_MASK                                 (0x00000004U)
#define CSL_SDFM_SDIFLGCLR_FLT2_FLG_CEVT1_SHIFT                                (0x00000002U)
#define CSL_SDFM_SDIFLGCLR_FLT2_FLG_CEVT1_RESETVAL                             (0x00000000U)
#define CSL_SDFM_SDIFLGCLR_FLT2_FLG_CEVT1_MAX                                  (0x00000001U)

#define CSL_SDFM_SDIFLGCLR_FLT2_FLG_CEVT2_MASK                                 (0x00000008U)
#define CSL_SDFM_SDIFLGCLR_FLT2_FLG_CEVT2_SHIFT                                (0x00000003U)
#define CSL_SDFM_SDIFLGCLR_FLT2_FLG_CEVT2_RESETVAL                             (0x00000000U)
#define CSL_SDFM_SDIFLGCLR_FLT2_FLG_CEVT2_MAX                                  (0x00000001U)

#define CSL_SDFM_SDIFLGCLR_FLT3_FLG_CEVT1_MASK                                 (0x00000010U)
#define CSL_SDFM_SDIFLGCLR_FLT3_FLG_CEVT1_SHIFT                                (0x00000004U)
#define CSL_SDFM_SDIFLGCLR_FLT3_FLG_CEVT1_RESETVAL                             (0x00000000U)
#define CSL_SDFM_SDIFLGCLR_FLT3_FLG_CEVT1_MAX                                  (0x00000001U)

#define CSL_SDFM_SDIFLGCLR_FLT3_FLG_CEVT2_MASK                                 (0x00000020U)
#define CSL_SDFM_SDIFLGCLR_FLT3_FLG_CEVT2_SHIFT                                (0x00000005U)
#define CSL_SDFM_SDIFLGCLR_FLT3_FLG_CEVT2_RESETVAL                             (0x00000000U)
#define CSL_SDFM_SDIFLGCLR_FLT3_FLG_CEVT2_MAX                                  (0x00000001U)

#define CSL_SDFM_SDIFLGCLR_FLT4_FLG_CEVT1_MASK                                 (0x00000040U)
#define CSL_SDFM_SDIFLGCLR_FLT4_FLG_CEVT1_SHIFT                                (0x00000006U)
#define CSL_SDFM_SDIFLGCLR_FLT4_FLG_CEVT1_RESETVAL                             (0x00000000U)
#define CSL_SDFM_SDIFLGCLR_FLT4_FLG_CEVT1_MAX                                  (0x00000001U)

#define CSL_SDFM_SDIFLGCLR_FLT4_FLG_CEVT2_MASK                                 (0x00000080U)
#define CSL_SDFM_SDIFLGCLR_FLT4_FLG_CEVT2_SHIFT                                (0x00000007U)
#define CSL_SDFM_SDIFLGCLR_FLT4_FLG_CEVT2_RESETVAL                             (0x00000000U)
#define CSL_SDFM_SDIFLGCLR_FLT4_FLG_CEVT2_MAX                                  (0x00000001U)

#define CSL_SDFM_SDIFLGCLR_MF1_MASK                                            (0x00000100U)
#define CSL_SDFM_SDIFLGCLR_MF1_SHIFT                                           (0x00000008U)
#define CSL_SDFM_SDIFLGCLR_MF1_RESETVAL                                        (0x00000000U)
#define CSL_SDFM_SDIFLGCLR_MF1_MAX                                             (0x00000001U)

#define CSL_SDFM_SDIFLGCLR_MF2_MASK                                            (0x00000200U)
#define CSL_SDFM_SDIFLGCLR_MF2_SHIFT                                           (0x00000009U)
#define CSL_SDFM_SDIFLGCLR_MF2_RESETVAL                                        (0x00000000U)
#define CSL_SDFM_SDIFLGCLR_MF2_MAX                                             (0x00000001U)

#define CSL_SDFM_SDIFLGCLR_MF3_MASK                                            (0x00000400U)
#define CSL_SDFM_SDIFLGCLR_MF3_SHIFT                                           (0x0000000AU)
#define CSL_SDFM_SDIFLGCLR_MF3_RESETVAL                                        (0x00000000U)
#define CSL_SDFM_SDIFLGCLR_MF3_MAX                                             (0x00000001U)

#define CSL_SDFM_SDIFLGCLR_MF4_MASK                                            (0x00000800U)
#define CSL_SDFM_SDIFLGCLR_MF4_SHIFT                                           (0x0000000BU)
#define CSL_SDFM_SDIFLGCLR_MF4_RESETVAL                                        (0x00000000U)
#define CSL_SDFM_SDIFLGCLR_MF4_MAX                                             (0x00000001U)

#define CSL_SDFM_SDIFLGCLR_AF1_MASK                                            (0x00001000U)
#define CSL_SDFM_SDIFLGCLR_AF1_SHIFT                                           (0x0000000CU)
#define CSL_SDFM_SDIFLGCLR_AF1_RESETVAL                                        (0x00000000U)
#define CSL_SDFM_SDIFLGCLR_AF1_MAX                                             (0x00000001U)

#define CSL_SDFM_SDIFLGCLR_AF2_MASK                                            (0x00002000U)
#define CSL_SDFM_SDIFLGCLR_AF2_SHIFT                                           (0x0000000DU)
#define CSL_SDFM_SDIFLGCLR_AF2_RESETVAL                                        (0x00000000U)
#define CSL_SDFM_SDIFLGCLR_AF2_MAX                                             (0x00000001U)

#define CSL_SDFM_SDIFLGCLR_AF3_MASK                                            (0x00004000U)
#define CSL_SDFM_SDIFLGCLR_AF3_SHIFT                                           (0x0000000EU)
#define CSL_SDFM_SDIFLGCLR_AF3_RESETVAL                                        (0x00000000U)
#define CSL_SDFM_SDIFLGCLR_AF3_MAX                                             (0x00000001U)

#define CSL_SDFM_SDIFLGCLR_AF4_MASK                                            (0x00008000U)
#define CSL_SDFM_SDIFLGCLR_AF4_SHIFT                                           (0x0000000FU)
#define CSL_SDFM_SDIFLGCLR_AF4_RESETVAL                                        (0x00000000U)
#define CSL_SDFM_SDIFLGCLR_AF4_MAX                                             (0x00000001U)

#define CSL_SDFM_SDIFLGCLR_SDFFOVF1_MASK                                       (0x00010000U)
#define CSL_SDFM_SDIFLGCLR_SDFFOVF1_SHIFT                                      (0x00000010U)
#define CSL_SDFM_SDIFLGCLR_SDFFOVF1_RESETVAL                                   (0x00000000U)
#define CSL_SDFM_SDIFLGCLR_SDFFOVF1_MAX                                        (0x00000001U)

#define CSL_SDFM_SDIFLGCLR_SDFFOVF2_MASK                                       (0x00020000U)
#define CSL_SDFM_SDIFLGCLR_SDFFOVF2_SHIFT                                      (0x00000011U)
#define CSL_SDFM_SDIFLGCLR_SDFFOVF2_RESETVAL                                   (0x00000000U)
#define CSL_SDFM_SDIFLGCLR_SDFFOVF2_MAX                                        (0x00000001U)

#define CSL_SDFM_SDIFLGCLR_SDFFOVF3_MASK                                       (0x00040000U)
#define CSL_SDFM_SDIFLGCLR_SDFFOVF3_SHIFT                                      (0x00000012U)
#define CSL_SDFM_SDIFLGCLR_SDFFOVF3_RESETVAL                                   (0x00000000U)
#define CSL_SDFM_SDIFLGCLR_SDFFOVF3_MAX                                        (0x00000001U)

#define CSL_SDFM_SDIFLGCLR_SDFFOVF4_MASK                                       (0x00080000U)
#define CSL_SDFM_SDIFLGCLR_SDFFOVF4_SHIFT                                      (0x00000013U)
#define CSL_SDFM_SDIFLGCLR_SDFFOVF4_RESETVAL                                   (0x00000000U)
#define CSL_SDFM_SDIFLGCLR_SDFFOVF4_MAX                                        (0x00000001U)

#define CSL_SDFM_SDIFLGCLR_SDFFINT1_MASK                                       (0x00100000U)
#define CSL_SDFM_SDIFLGCLR_SDFFINT1_SHIFT                                      (0x00000014U)
#define CSL_SDFM_SDIFLGCLR_SDFFINT1_RESETVAL                                   (0x00000000U)
#define CSL_SDFM_SDIFLGCLR_SDFFINT1_MAX                                        (0x00000001U)

#define CSL_SDFM_SDIFLGCLR_SDFFINT2_MASK                                       (0x00200000U)
#define CSL_SDFM_SDIFLGCLR_SDFFINT2_SHIFT                                      (0x00000015U)
#define CSL_SDFM_SDIFLGCLR_SDFFINT2_RESETVAL                                   (0x00000000U)
#define CSL_SDFM_SDIFLGCLR_SDFFINT2_MAX                                        (0x00000001U)

#define CSL_SDFM_SDIFLGCLR_SDFFINT3_MASK                                       (0x00400000U)
#define CSL_SDFM_SDIFLGCLR_SDFFINT3_SHIFT                                      (0x00000016U)
#define CSL_SDFM_SDIFLGCLR_SDFFINT3_RESETVAL                                   (0x00000000U)
#define CSL_SDFM_SDIFLGCLR_SDFFINT3_MAX                                        (0x00000001U)

#define CSL_SDFM_SDIFLGCLR_SDFFINT4_MASK                                       (0x00800000U)
#define CSL_SDFM_SDIFLGCLR_SDFFINT4_SHIFT                                      (0x00000017U)
#define CSL_SDFM_SDIFLGCLR_SDFFINT4_RESETVAL                                   (0x00000000U)
#define CSL_SDFM_SDIFLGCLR_SDFFINT4_MAX                                        (0x00000001U)

#define CSL_SDFM_SDIFLGCLR_RESERVED_1_MASK                                     (0x7F000000U)
#define CSL_SDFM_SDIFLGCLR_RESERVED_1_SHIFT                                    (0x00000018U)
#define CSL_SDFM_SDIFLGCLR_RESERVED_1_RESETVAL                                 (0x00000000U)
#define CSL_SDFM_SDIFLGCLR_RESERVED_1_MAX                                      (0x0000007FU)

#define CSL_SDFM_SDIFLGCLR_MIF_MASK                                            (0x80000000U)
#define CSL_SDFM_SDIFLGCLR_MIF_SHIFT                                           (0x0000001FU)
#define CSL_SDFM_SDIFLGCLR_MIF_RESETVAL                                        (0x00000000U)
#define CSL_SDFM_SDIFLGCLR_MIF_MAX                                             (0x00000001U)

#define CSL_SDFM_SDIFLGCLR_RESETVAL                                            (0x00000000U)

/* SDCTL */

#define CSL_SDFM_SDCTL_HZ1_MASK                                                (0x0001U)
#define CSL_SDFM_SDCTL_HZ1_SHIFT                                               (0x0000U)
#define CSL_SDFM_SDCTL_HZ1_RESETVAL                                            (0x0000U)
#define CSL_SDFM_SDCTL_HZ1_MAX                                                 (0x0001U)

#define CSL_SDFM_SDCTL_HZ2_MASK                                                (0x0002U)
#define CSL_SDFM_SDCTL_HZ2_SHIFT                                               (0x0001U)
#define CSL_SDFM_SDCTL_HZ2_RESETVAL                                            (0x0000U)
#define CSL_SDFM_SDCTL_HZ2_MAX                                                 (0x0001U)

#define CSL_SDFM_SDCTL_HZ3_MASK                                                (0x0004U)
#define CSL_SDFM_SDCTL_HZ3_SHIFT                                               (0x0002U)
#define CSL_SDFM_SDCTL_HZ3_RESETVAL                                            (0x0000U)
#define CSL_SDFM_SDCTL_HZ3_MAX                                                 (0x0001U)

#define CSL_SDFM_SDCTL_HZ4_MASK                                                (0x0008U)
#define CSL_SDFM_SDCTL_HZ4_SHIFT                                               (0x0003U)
#define CSL_SDFM_SDCTL_HZ4_RESETVAL                                            (0x0000U)
#define CSL_SDFM_SDCTL_HZ4_MAX                                                 (0x0001U)

#define CSL_SDFM_SDCTL_RESERVED_1_MASK                                         (0x1FF0U)
#define CSL_SDFM_SDCTL_RESERVED_1_SHIFT                                        (0x0004U)
#define CSL_SDFM_SDCTL_RESERVED_1_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDCTL_RESERVED_1_MAX                                          (0x01FFU)

#define CSL_SDFM_SDCTL_MIE_MASK                                                (0x2000U)
#define CSL_SDFM_SDCTL_MIE_SHIFT                                               (0x000DU)
#define CSL_SDFM_SDCTL_MIE_RESETVAL                                            (0x0000U)
#define CSL_SDFM_SDCTL_MIE_MAX                                                 (0x0001U)

#define CSL_SDFM_SDCTL_RESERVED_2_MASK                                         (0x4000U)
#define CSL_SDFM_SDCTL_RESERVED_2_SHIFT                                        (0x000EU)
#define CSL_SDFM_SDCTL_RESERVED_2_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDCTL_RESERVED_2_MAX                                          (0x0001U)

#define CSL_SDFM_SDCTL_RESERVED_3_MASK                                         (0x8000U)
#define CSL_SDFM_SDCTL_RESERVED_3_SHIFT                                        (0x000FU)
#define CSL_SDFM_SDCTL_RESERVED_3_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDCTL_RESERVED_3_MAX                                          (0x0001U)

#define CSL_SDFM_SDCTL_RESETVAL                                                (0x0000U)

/* SDMFILEN */

#define CSL_SDFM_SDMFILEN_RESERVED_1_MASK                                      (0x000FU)
#define CSL_SDFM_SDMFILEN_RESERVED_1_SHIFT                                     (0x0000U)
#define CSL_SDFM_SDMFILEN_RESERVED_1_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDMFILEN_RESERVED_1_MAX                                       (0x000FU)

#define CSL_SDFM_SDMFILEN_RESERVED_2_MASK                                      (0x0070U)
#define CSL_SDFM_SDMFILEN_RESERVED_2_SHIFT                                     (0x0004U)
#define CSL_SDFM_SDMFILEN_RESERVED_2_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDMFILEN_RESERVED_2_MAX                                       (0x0007U)

#define CSL_SDFM_SDMFILEN_RESERVED_3_MASK                                      (0x0180U)
#define CSL_SDFM_SDMFILEN_RESERVED_3_SHIFT                                     (0x0007U)
#define CSL_SDFM_SDMFILEN_RESERVED_3_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDMFILEN_RESERVED_3_MAX                                       (0x0003U)

#define CSL_SDFM_SDMFILEN_RESERVED_4_MASK                                      (0x0200U)
#define CSL_SDFM_SDMFILEN_RESERVED_4_SHIFT                                     (0x0009U)
#define CSL_SDFM_SDMFILEN_RESERVED_4_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDMFILEN_RESERVED_4_MAX                                       (0x0001U)

#define CSL_SDFM_SDMFILEN_RESERVED_5_MASK                                      (0x0400U)
#define CSL_SDFM_SDMFILEN_RESERVED_5_SHIFT                                     (0x000AU)
#define CSL_SDFM_SDMFILEN_RESERVED_5_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDMFILEN_RESERVED_5_MAX                                       (0x0001U)

#define CSL_SDFM_SDMFILEN_MFE_MASK                                             (0x0800U)
#define CSL_SDFM_SDMFILEN_MFE_SHIFT                                            (0x000BU)
#define CSL_SDFM_SDMFILEN_MFE_RESETVAL                                         (0x0000U)
#define CSL_SDFM_SDMFILEN_MFE_MAX                                              (0x0001U)

#define CSL_SDFM_SDMFILEN_RESERVED_6_MASK                                      (0x1000U)
#define CSL_SDFM_SDMFILEN_RESERVED_6_SHIFT                                     (0x000CU)
#define CSL_SDFM_SDMFILEN_RESERVED_6_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDMFILEN_RESERVED_6_MAX                                       (0x0001U)

#define CSL_SDFM_SDMFILEN_RESERVED_7_MASK                                      (0xE000U)
#define CSL_SDFM_SDMFILEN_RESERVED_7_SHIFT                                     (0x000DU)
#define CSL_SDFM_SDMFILEN_RESERVED_7_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDMFILEN_RESERVED_7_MAX                                       (0x0007U)

#define CSL_SDFM_SDMFILEN_RESETVAL                                             (0x0000U)

/* SDSTATUS */

#define CSL_SDFM_SDSTATUS_HZ1_MASK                                             (0x0001U)
#define CSL_SDFM_SDSTATUS_HZ1_SHIFT                                            (0x0000U)
#define CSL_SDFM_SDSTATUS_HZ1_RESETVAL                                         (0x0000U)
#define CSL_SDFM_SDSTATUS_HZ1_MAX                                              (0x0001U)

#define CSL_SDFM_SDSTATUS_HZ2_MASK                                             (0x0002U)
#define CSL_SDFM_SDSTATUS_HZ2_SHIFT                                            (0x0001U)
#define CSL_SDFM_SDSTATUS_HZ2_RESETVAL                                         (0x0000U)
#define CSL_SDFM_SDSTATUS_HZ2_MAX                                              (0x0001U)

#define CSL_SDFM_SDSTATUS_HZ3_MASK                                             (0x0004U)
#define CSL_SDFM_SDSTATUS_HZ3_SHIFT                                            (0x0002U)
#define CSL_SDFM_SDSTATUS_HZ3_RESETVAL                                         (0x0000U)
#define CSL_SDFM_SDSTATUS_HZ3_MAX                                              (0x0001U)

#define CSL_SDFM_SDSTATUS_HZ4_MASK                                             (0x0008U)
#define CSL_SDFM_SDSTATUS_HZ4_SHIFT                                            (0x0003U)
#define CSL_SDFM_SDSTATUS_HZ4_RESETVAL                                         (0x0000U)
#define CSL_SDFM_SDSTATUS_HZ4_MAX                                              (0x0001U)

#define CSL_SDFM_SDSTATUS_RESERVED_1_MASK                                      (0x00F0U)
#define CSL_SDFM_SDSTATUS_RESERVED_1_SHIFT                                     (0x0004U)
#define CSL_SDFM_SDSTATUS_RESERVED_1_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDSTATUS_RESERVED_1_MAX                                       (0x000FU)

#define CSL_SDFM_SDSTATUS_RESERVED_2_MASK                                      (0x0100U)
#define CSL_SDFM_SDSTATUS_RESERVED_2_SHIFT                                     (0x0008U)
#define CSL_SDFM_SDSTATUS_RESERVED_2_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDSTATUS_RESERVED_2_MAX                                       (0x0001U)

#define CSL_SDFM_SDSTATUS_RESERVED_3_MASK                                      (0x0200U)
#define CSL_SDFM_SDSTATUS_RESERVED_3_SHIFT                                     (0x0009U)
#define CSL_SDFM_SDSTATUS_RESERVED_3_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDSTATUS_RESERVED_3_MAX                                       (0x0001U)

#define CSL_SDFM_SDSTATUS_RESERVED_4_MASK                                      (0x0400U)
#define CSL_SDFM_SDSTATUS_RESERVED_4_SHIFT                                     (0x000AU)
#define CSL_SDFM_SDSTATUS_RESERVED_4_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDSTATUS_RESERVED_4_MAX                                       (0x0001U)

#define CSL_SDFM_SDSTATUS_RESERVED_5_MASK                                      (0x0800U)
#define CSL_SDFM_SDSTATUS_RESERVED_5_SHIFT                                     (0x000BU)
#define CSL_SDFM_SDSTATUS_RESERVED_5_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDSTATUS_RESERVED_5_MAX                                       (0x0001U)

#define CSL_SDFM_SDSTATUS_RESERVED_6_MASK                                      (0x1000U)
#define CSL_SDFM_SDSTATUS_RESERVED_6_SHIFT                                     (0x000CU)
#define CSL_SDFM_SDSTATUS_RESERVED_6_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDSTATUS_RESERVED_6_MAX                                       (0x0001U)

#define CSL_SDFM_SDSTATUS_RESERVED_7_MASK                                      (0x2000U)
#define CSL_SDFM_SDSTATUS_RESERVED_7_SHIFT                                     (0x000DU)
#define CSL_SDFM_SDSTATUS_RESERVED_7_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDSTATUS_RESERVED_7_MAX                                       (0x0001U)

#define CSL_SDFM_SDSTATUS_RESERVED_8_MASK                                      (0x4000U)
#define CSL_SDFM_SDSTATUS_RESERVED_8_SHIFT                                     (0x000EU)
#define CSL_SDFM_SDSTATUS_RESERVED_8_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDSTATUS_RESERVED_8_MAX                                       (0x0001U)

#define CSL_SDFM_SDSTATUS_RESERVED_9_MASK                                      (0x8000U)
#define CSL_SDFM_SDSTATUS_RESERVED_9_SHIFT                                     (0x000FU)
#define CSL_SDFM_SDSTATUS_RESERVED_9_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDSTATUS_RESERVED_9_MAX                                       (0x0001U)

#define CSL_SDFM_SDSTATUS_RESETVAL                                             (0x0000U)

/* SDCTLPARM1 */

#define CSL_SDFM_SDCTLPARM1_MOD_MASK                                           (0x0003U)
#define CSL_SDFM_SDCTLPARM1_MOD_SHIFT                                          (0x0000U)
#define CSL_SDFM_SDCTLPARM1_MOD_RESETVAL                                       (0x0000U)
#define CSL_SDFM_SDCTLPARM1_MOD_MAX                                            (0x0003U)

#define CSL_SDFM_SDCTLPARM1_RESERVED_1_MASK                                    (0x0004U)
#define CSL_SDFM_SDCTLPARM1_RESERVED_1_SHIFT                                   (0x0002U)
#define CSL_SDFM_SDCTLPARM1_RESERVED_1_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCTLPARM1_RESERVED_1_MAX                                     (0x0001U)

#define CSL_SDFM_SDCTLPARM1_SDCLKSEL_MASK                                      (0x0008U)
#define CSL_SDFM_SDCTLPARM1_SDCLKSEL_SHIFT                                     (0x0003U)
#define CSL_SDFM_SDCTLPARM1_SDCLKSEL_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDCTLPARM1_SDCLKSEL_MAX                                       (0x0001U)

#define CSL_SDFM_SDCTLPARM1_SDCLKSYNC_MASK                                     (0x0010U)
#define CSL_SDFM_SDCTLPARM1_SDCLKSYNC_SHIFT                                    (0x0004U)
#define CSL_SDFM_SDCTLPARM1_SDCLKSYNC_RESETVAL                                 (0x0000U)
#define CSL_SDFM_SDCTLPARM1_SDCLKSYNC_MAX                                      (0x0001U)

#define CSL_SDFM_SDCTLPARM1_RESERVED_2_MASK                                    (0x0020U)
#define CSL_SDFM_SDCTLPARM1_RESERVED_2_SHIFT                                   (0x0005U)
#define CSL_SDFM_SDCTLPARM1_RESERVED_2_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCTLPARM1_RESERVED_2_MAX                                     (0x0001U)

#define CSL_SDFM_SDCTLPARM1_SDDATASYNC_MASK                                    (0x0040U)
#define CSL_SDFM_SDCTLPARM1_SDDATASYNC_SHIFT                                   (0x0006U)
#define CSL_SDFM_SDCTLPARM1_SDDATASYNC_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCTLPARM1_SDDATASYNC_MAX                                     (0x0001U)

#define CSL_SDFM_SDCTLPARM1_RESERVED_3_MASK                                    (0x0080U)
#define CSL_SDFM_SDCTLPARM1_RESERVED_3_SHIFT                                   (0x0007U)
#define CSL_SDFM_SDCTLPARM1_RESERVED_3_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCTLPARM1_RESERVED_3_MAX                                     (0x0001U)

#define CSL_SDFM_SDCTLPARM1_RESERVED_4_MASK                                    (0xFF00U)
#define CSL_SDFM_SDCTLPARM1_RESERVED_4_SHIFT                                   (0x0008U)
#define CSL_SDFM_SDCTLPARM1_RESERVED_4_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCTLPARM1_RESERVED_4_MAX                                     (0x00FFU)

#define CSL_SDFM_SDCTLPARM1_RESETVAL                                           (0x0000U)

/* SDDFPARM1 */

#define CSL_SDFM_SDDFPARM1_DOSR_MASK                                           (0x00FFU)
#define CSL_SDFM_SDDFPARM1_DOSR_SHIFT                                          (0x0000U)
#define CSL_SDFM_SDDFPARM1_DOSR_RESETVAL                                       (0x0000U)
#define CSL_SDFM_SDDFPARM1_DOSR_MAX                                            (0x00FFU)

#define CSL_SDFM_SDDFPARM1_FEN_MASK                                            (0x0100U)
#define CSL_SDFM_SDDFPARM1_FEN_SHIFT                                           (0x0008U)
#define CSL_SDFM_SDDFPARM1_FEN_RESETVAL                                        (0x0000U)
#define CSL_SDFM_SDDFPARM1_FEN_MAX                                             (0x0001U)

#define CSL_SDFM_SDDFPARM1_AE_MASK                                             (0x0200U)
#define CSL_SDFM_SDDFPARM1_AE_SHIFT                                            (0x0009U)
#define CSL_SDFM_SDDFPARM1_AE_RESETVAL                                         (0x0000U)
#define CSL_SDFM_SDDFPARM1_AE_MAX                                              (0x0001U)

#define CSL_SDFM_SDDFPARM1_SST_MASK                                            (0x0C00U)
#define CSL_SDFM_SDDFPARM1_SST_SHIFT                                           (0x000AU)
#define CSL_SDFM_SDDFPARM1_SST_RESETVAL                                        (0x0000U)
#define CSL_SDFM_SDDFPARM1_SST_MAX                                             (0x0003U)

#define CSL_SDFM_SDDFPARM1_SDSYNCEN_MASK                                       (0x1000U)
#define CSL_SDFM_SDDFPARM1_SDSYNCEN_SHIFT                                      (0x000CU)
#define CSL_SDFM_SDDFPARM1_SDSYNCEN_RESETVAL                                   (0x0000U)
#define CSL_SDFM_SDDFPARM1_SDSYNCEN_MAX                                        (0x0001U)

#define CSL_SDFM_SDDFPARM1_RESERVED_1_MASK                                     (0xE000U)
#define CSL_SDFM_SDDFPARM1_RESERVED_1_SHIFT                                    (0x000DU)
#define CSL_SDFM_SDDFPARM1_RESERVED_1_RESETVAL                                 (0x0000U)
#define CSL_SDFM_SDDFPARM1_RESERVED_1_MAX                                      (0x0007U)

#define CSL_SDFM_SDDFPARM1_RESETVAL                                            (0x0000U)

/* SDDPARM1 */

#define CSL_SDFM_SDDPARM1_RESERVED_1_MASK                                      (0x03FFU)
#define CSL_SDFM_SDDPARM1_RESERVED_1_SHIFT                                     (0x0000U)
#define CSL_SDFM_SDDPARM1_RESERVED_1_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDDPARM1_RESERVED_1_MAX                                       (0x03FFU)

#define CSL_SDFM_SDDPARM1_DR_MASK                                              (0x0400U)
#define CSL_SDFM_SDDPARM1_DR_SHIFT                                             (0x000AU)
#define CSL_SDFM_SDDPARM1_DR_RESETVAL                                          (0x0000U)
#define CSL_SDFM_SDDPARM1_DR_MAX                                               (0x0001U)

#define CSL_SDFM_SDDPARM1_SH_MASK                                              (0xF800U)
#define CSL_SDFM_SDDPARM1_SH_SHIFT                                             (0x000BU)
#define CSL_SDFM_SDDPARM1_SH_RESETVAL                                          (0x0000U)
#define CSL_SDFM_SDDPARM1_SH_MAX                                               (0x001FU)

#define CSL_SDFM_SDDPARM1_RESETVAL                                             (0x0000U)

/* SDFLT1CMPH1 */

#define CSL_SDFM_SDFLT1CMPH1_HLT_MASK                                          (0x7FFFU)
#define CSL_SDFM_SDFLT1CMPH1_HLT_SHIFT                                         (0x0000U)
#define CSL_SDFM_SDFLT1CMPH1_HLT_RESETVAL                                      (0x7FFFU)
#define CSL_SDFM_SDFLT1CMPH1_HLT_MAX                                           (0x7FFFU)

#define CSL_SDFM_SDFLT1CMPH1_RESERVED_1_MASK                                   (0x8000U)
#define CSL_SDFM_SDFLT1CMPH1_RESERVED_1_SHIFT                                  (0x000FU)
#define CSL_SDFM_SDFLT1CMPH1_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDFLT1CMPH1_RESERVED_1_MAX                                    (0x0001U)

#define CSL_SDFM_SDFLT1CMPH1_RESETVAL                                          (0x7FFFU)

/* SDFLT1CMPL1 */

#define CSL_SDFM_SDFLT1CMPL1_LLT_MASK                                          (0x7FFFU)
#define CSL_SDFM_SDFLT1CMPL1_LLT_SHIFT                                         (0x0000U)
#define CSL_SDFM_SDFLT1CMPL1_LLT_RESETVAL                                      (0x0000U)
#define CSL_SDFM_SDFLT1CMPL1_LLT_MAX                                           (0x7FFFU)

#define CSL_SDFM_SDFLT1CMPL1_RESERVED_1_MASK                                   (0x8000U)
#define CSL_SDFM_SDFLT1CMPL1_RESERVED_1_SHIFT                                  (0x000FU)
#define CSL_SDFM_SDFLT1CMPL1_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDFLT1CMPL1_RESERVED_1_MAX                                    (0x0001U)

#define CSL_SDFM_SDFLT1CMPL1_RESETVAL                                          (0x0000U)

/* SDCPARM1 */

#define CSL_SDFM_SDCPARM1_COSR_MASK                                            (0x001FU)
#define CSL_SDFM_SDCPARM1_COSR_SHIFT                                           (0x0000U)
#define CSL_SDFM_SDCPARM1_COSR_RESETVAL                                        (0x0000U)
#define CSL_SDFM_SDCPARM1_COSR_MAX                                             (0x001FU)

#define CSL_SDFM_SDCPARM1_EN_CEVT1_MASK                                        (0x0020U)
#define CSL_SDFM_SDCPARM1_EN_CEVT1_SHIFT                                       (0x0005U)
#define CSL_SDFM_SDCPARM1_EN_CEVT1_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDCPARM1_EN_CEVT1_MAX                                         (0x0001U)

#define CSL_SDFM_SDCPARM1_EN_CEVT2_MASK                                        (0x0040U)
#define CSL_SDFM_SDCPARM1_EN_CEVT2_SHIFT                                       (0x0006U)
#define CSL_SDFM_SDCPARM1_EN_CEVT2_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDCPARM1_EN_CEVT2_MAX                                         (0x0001U)

#define CSL_SDFM_SDCPARM1_CS1_CS0_MASK                                         (0x0180U)
#define CSL_SDFM_SDCPARM1_CS1_CS0_SHIFT                                        (0x0007U)
#define CSL_SDFM_SDCPARM1_CS1_CS0_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDCPARM1_CS1_CS0_MAX                                          (0x0003U)

#define CSL_SDFM_SDCPARM1_MFIE_MASK                                            (0x0200U)
#define CSL_SDFM_SDCPARM1_MFIE_SHIFT                                           (0x0009U)
#define CSL_SDFM_SDCPARM1_MFIE_RESETVAL                                        (0x0000U)
#define CSL_SDFM_SDCPARM1_MFIE_MAX                                             (0x0001U)

#define CSL_SDFM_SDCPARM1_HZEN_MASK                                            (0x0400U)
#define CSL_SDFM_SDCPARM1_HZEN_SHIFT                                           (0x000AU)
#define CSL_SDFM_SDCPARM1_HZEN_RESETVAL                                        (0x0000U)
#define CSL_SDFM_SDCPARM1_HZEN_MAX                                             (0x0001U)

#define CSL_SDFM_SDCPARM1_CEVT1SEL_MASK                                        (0x1800U)
#define CSL_SDFM_SDCPARM1_CEVT1SEL_SHIFT                                       (0x000BU)
#define CSL_SDFM_SDCPARM1_CEVT1SEL_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDCPARM1_CEVT1SEL_MAX                                         (0x0003U)

#define CSL_SDFM_SDCPARM1_CEN_MASK                                             (0x2000U)
#define CSL_SDFM_SDCPARM1_CEN_SHIFT                                            (0x000DU)
#define CSL_SDFM_SDCPARM1_CEN_RESETVAL                                         (0x0000U)
#define CSL_SDFM_SDCPARM1_CEN_MAX                                              (0x0001U)

#define CSL_SDFM_SDCPARM1_CEVT2SEL_MASK                                        (0xC000U)
#define CSL_SDFM_SDCPARM1_CEVT2SEL_SHIFT                                       (0x000EU)
#define CSL_SDFM_SDCPARM1_CEVT2SEL_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDCPARM1_CEVT2SEL_MAX                                         (0x0003U)

#define CSL_SDFM_SDCPARM1_RESETVAL                                             (0x0000U)

/* SDDATA1 */

#define CSL_SDFM_SDDATA1_DATA16_MASK                                           (0x0000FFFFU)
#define CSL_SDFM_SDDATA1_DATA16_SHIFT                                          (0x00000000U)
#define CSL_SDFM_SDDATA1_DATA16_RESETVAL                                       (0x00000000U)
#define CSL_SDFM_SDDATA1_DATA16_MAX                                            (0x0000FFFFU)

#define CSL_SDFM_SDDATA1_DATA32HI_MASK                                         (0xFFFF0000U)
#define CSL_SDFM_SDDATA1_DATA32HI_SHIFT                                        (0x00000010U)
#define CSL_SDFM_SDDATA1_DATA32HI_RESETVAL                                     (0x00000000U)
#define CSL_SDFM_SDDATA1_DATA32HI_MAX                                          (0x0000FFFFU)

#define CSL_SDFM_SDDATA1_RESETVAL                                              (0x00000000U)

/* SDDATFIFO1 */

#define CSL_SDFM_SDDATFIFO1_DATA16_MASK                                        (0x0000FFFFU)
#define CSL_SDFM_SDDATFIFO1_DATA16_SHIFT                                       (0x00000000U)
#define CSL_SDFM_SDDATFIFO1_DATA16_RESETVAL                                    (0x00000000U)
#define CSL_SDFM_SDDATFIFO1_DATA16_MAX                                         (0x0000FFFFU)

#define CSL_SDFM_SDDATFIFO1_DATA32HI_MASK                                      (0xFFFF0000U)
#define CSL_SDFM_SDDATFIFO1_DATA32HI_SHIFT                                     (0x00000010U)
#define CSL_SDFM_SDDATFIFO1_DATA32HI_RESETVAL                                  (0x00000000U)
#define CSL_SDFM_SDDATFIFO1_DATA32HI_MAX                                       (0x0000FFFFU)

#define CSL_SDFM_SDDATFIFO1_RESETVAL                                           (0x00000000U)

/* SDCDATA1 */

#define CSL_SDFM_SDCDATA1_DATA16_MASK                                          (0xFFFFU)
#define CSL_SDFM_SDCDATA1_DATA16_SHIFT                                         (0x0000U)
#define CSL_SDFM_SDCDATA1_DATA16_RESETVAL                                      (0x0000U)
#define CSL_SDFM_SDCDATA1_DATA16_MAX                                           (0xFFFFU)

#define CSL_SDFM_SDCDATA1_RESETVAL                                             (0x0000U)

/* SDFLT1CMPH2 */

#define CSL_SDFM_SDFLT1CMPH2_HLT2_MASK                                         (0x7FFFU)
#define CSL_SDFM_SDFLT1CMPH2_HLT2_SHIFT                                        (0x0000U)
#define CSL_SDFM_SDFLT1CMPH2_HLT2_RESETVAL                                     (0x7FFFU)
#define CSL_SDFM_SDFLT1CMPH2_HLT2_MAX                                          (0x7FFFU)

#define CSL_SDFM_SDFLT1CMPH2_RESERVED_1_MASK                                   (0x8000U)
#define CSL_SDFM_SDFLT1CMPH2_RESERVED_1_SHIFT                                  (0x000FU)
#define CSL_SDFM_SDFLT1CMPH2_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDFLT1CMPH2_RESERVED_1_MAX                                    (0x0001U)

#define CSL_SDFM_SDFLT1CMPH2_RESETVAL                                          (0x7FFFU)

/* SDFLT1CMPHZ */

#define CSL_SDFM_SDFLT1CMPHZ_HLTZ_MASK                                         (0x7FFFU)
#define CSL_SDFM_SDFLT1CMPHZ_HLTZ_SHIFT                                        (0x0000U)
#define CSL_SDFM_SDFLT1CMPHZ_HLTZ_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDFLT1CMPHZ_HLTZ_MAX                                          (0x7FFFU)

#define CSL_SDFM_SDFLT1CMPHZ_RESERVED_1_MASK                                   (0x8000U)
#define CSL_SDFM_SDFLT1CMPHZ_RESERVED_1_SHIFT                                  (0x000FU)
#define CSL_SDFM_SDFLT1CMPHZ_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDFLT1CMPHZ_RESERVED_1_MAX                                    (0x0001U)

#define CSL_SDFM_SDFLT1CMPHZ_RESETVAL                                          (0x0000U)

/* SDFIFOCTL1 */

#define CSL_SDFM_SDFIFOCTL1_SDFFIL_MASK                                        (0x001FU)
#define CSL_SDFM_SDFIFOCTL1_SDFFIL_SHIFT                                       (0x0000U)
#define CSL_SDFM_SDFIFOCTL1_SDFFIL_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDFIFOCTL1_SDFFIL_MAX                                         (0x001FU)

#define CSL_SDFM_SDFIFOCTL1_RESERVED_1_MASK                                    (0x0020U)
#define CSL_SDFM_SDFIFOCTL1_RESERVED_1_SHIFT                                   (0x0005U)
#define CSL_SDFM_SDFIFOCTL1_RESERVED_1_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDFIFOCTL1_RESERVED_1_MAX                                     (0x0001U)

#define CSL_SDFM_SDFIFOCTL1_SDFFST_MASK                                        (0x07C0U)
#define CSL_SDFM_SDFIFOCTL1_SDFFST_SHIFT                                       (0x0006U)
#define CSL_SDFM_SDFIFOCTL1_SDFFST_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDFIFOCTL1_SDFFST_MAX                                         (0x001FU)

#define CSL_SDFM_SDFIFOCTL1_RESERVED_2_MASK                                    (0x0800U)
#define CSL_SDFM_SDFIFOCTL1_RESERVED_2_SHIFT                                   (0x000BU)
#define CSL_SDFM_SDFIFOCTL1_RESERVED_2_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDFIFOCTL1_RESERVED_2_MAX                                     (0x0001U)

#define CSL_SDFM_SDFIFOCTL1_FFIEN_MASK                                         (0x1000U)
#define CSL_SDFM_SDFIFOCTL1_FFIEN_SHIFT                                        (0x000CU)
#define CSL_SDFM_SDFIFOCTL1_FFIEN_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDFIFOCTL1_FFIEN_MAX                                          (0x0001U)

#define CSL_SDFM_SDFIFOCTL1_FFEN_MASK                                          (0x2000U)
#define CSL_SDFM_SDFIFOCTL1_FFEN_SHIFT                                         (0x000DU)
#define CSL_SDFM_SDFIFOCTL1_FFEN_RESETVAL                                      (0x0000U)
#define CSL_SDFM_SDFIFOCTL1_FFEN_MAX                                           (0x0001U)

#define CSL_SDFM_SDFIFOCTL1_DRINTSEL_MASK                                      (0x4000U)
#define CSL_SDFM_SDFIFOCTL1_DRINTSEL_SHIFT                                     (0x000EU)
#define CSL_SDFM_SDFIFOCTL1_DRINTSEL_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDFIFOCTL1_DRINTSEL_MAX                                       (0x0001U)

#define CSL_SDFM_SDFIFOCTL1_OVFIEN_MASK                                        (0x8000U)
#define CSL_SDFM_SDFIFOCTL1_OVFIEN_SHIFT                                       (0x000FU)
#define CSL_SDFM_SDFIFOCTL1_OVFIEN_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDFIFOCTL1_OVFIEN_MAX                                         (0x0001U)

#define CSL_SDFM_SDFIFOCTL1_RESETVAL                                           (0x0000U)

/* SDSYNC1 */

#define CSL_SDFM_SDSYNC1_SYNCSEL_MASK                                          (0x003FU)
#define CSL_SDFM_SDSYNC1_SYNCSEL_SHIFT                                         (0x0000U)
#define CSL_SDFM_SDSYNC1_SYNCSEL_RESETVAL                                      (0x0000U)
#define CSL_SDFM_SDSYNC1_SYNCSEL_MAX                                           (0x003FU)

#define CSL_SDFM_SDSYNC1_WTSYNCEN_MASK                                         (0x0040U)
#define CSL_SDFM_SDSYNC1_WTSYNCEN_SHIFT                                        (0x0006U)
#define CSL_SDFM_SDSYNC1_WTSYNCEN_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDSYNC1_WTSYNCEN_MAX                                          (0x0001U)

#define CSL_SDFM_SDSYNC1_WTSYNFLG_MASK                                         (0x0080U)
#define CSL_SDFM_SDSYNC1_WTSYNFLG_SHIFT                                        (0x0007U)
#define CSL_SDFM_SDSYNC1_WTSYNFLG_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDSYNC1_WTSYNFLG_MAX                                          (0x0001U)

#define CSL_SDFM_SDSYNC1_WTSYNCLR_MASK                                         (0x0100U)
#define CSL_SDFM_SDSYNC1_WTSYNCLR_SHIFT                                        (0x0008U)
#define CSL_SDFM_SDSYNC1_WTSYNCLR_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDSYNC1_WTSYNCLR_MAX                                          (0x0001U)

#define CSL_SDFM_SDSYNC1_FFSYNCCLREN_MASK                                      (0x0200U)
#define CSL_SDFM_SDSYNC1_FFSYNCCLREN_SHIFT                                     (0x0009U)
#define CSL_SDFM_SDSYNC1_FFSYNCCLREN_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDSYNC1_FFSYNCCLREN_MAX                                       (0x0001U)

#define CSL_SDFM_SDSYNC1_WTSCLREN_MASK                                         (0x0400U)
#define CSL_SDFM_SDSYNC1_WTSCLREN_SHIFT                                        (0x000AU)
#define CSL_SDFM_SDSYNC1_WTSCLREN_RESETVAL                                     (0x0001U)
#define CSL_SDFM_SDSYNC1_WTSCLREN_MAX                                          (0x0001U)

#define CSL_SDFM_SDSYNC1_RESERVED_1_MASK                                       (0xF800U)
#define CSL_SDFM_SDSYNC1_RESERVED_1_SHIFT                                      (0x000BU)
#define CSL_SDFM_SDSYNC1_RESERVED_1_RESETVAL                                   (0x0000U)
#define CSL_SDFM_SDSYNC1_RESERVED_1_MAX                                        (0x001FU)

#define CSL_SDFM_SDSYNC1_RESETVAL                                              (0x0400U)

/* SDFLT1CMPL2 */

#define CSL_SDFM_SDFLT1CMPL2_LLT2_MASK                                         (0x7FFFU)
#define CSL_SDFM_SDFLT1CMPL2_LLT2_SHIFT                                        (0x0000U)
#define CSL_SDFM_SDFLT1CMPL2_LLT2_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDFLT1CMPL2_LLT2_MAX                                          (0x7FFFU)

#define CSL_SDFM_SDFLT1CMPL2_RESERVED_1_MASK                                   (0x8000U)
#define CSL_SDFM_SDFLT1CMPL2_RESERVED_1_SHIFT                                  (0x000FU)
#define CSL_SDFM_SDFLT1CMPL2_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDFLT1CMPL2_RESERVED_1_MAX                                    (0x0001U)

#define CSL_SDFM_SDFLT1CMPL2_RESETVAL                                          (0x0000U)

/* SDCTLPARM2 */

#define CSL_SDFM_SDCTLPARM2_MOD_MASK                                           (0x0003U)
#define CSL_SDFM_SDCTLPARM2_MOD_SHIFT                                          (0x0000U)
#define CSL_SDFM_SDCTLPARM2_MOD_RESETVAL                                       (0x0000U)
#define CSL_SDFM_SDCTLPARM2_MOD_MAX                                            (0x0003U)

#define CSL_SDFM_SDCTLPARM2_RESERVED_1_MASK                                    (0x0004U)
#define CSL_SDFM_SDCTLPARM2_RESERVED_1_SHIFT                                   (0x0002U)
#define CSL_SDFM_SDCTLPARM2_RESERVED_1_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCTLPARM2_RESERVED_1_MAX                                     (0x0001U)

#define CSL_SDFM_SDCTLPARM2_SDCLKSEL_MASK                                      (0x0008U)
#define CSL_SDFM_SDCTLPARM2_SDCLKSEL_SHIFT                                     (0x0003U)
#define CSL_SDFM_SDCTLPARM2_SDCLKSEL_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDCTLPARM2_SDCLKSEL_MAX                                       (0x0001U)

#define CSL_SDFM_SDCTLPARM2_SDCLKSYNC_MASK                                     (0x0010U)
#define CSL_SDFM_SDCTLPARM2_SDCLKSYNC_SHIFT                                    (0x0004U)
#define CSL_SDFM_SDCTLPARM2_SDCLKSYNC_RESETVAL                                 (0x0000U)
#define CSL_SDFM_SDCTLPARM2_SDCLKSYNC_MAX                                      (0x0001U)

#define CSL_SDFM_SDCTLPARM2_RESERVED_2_MASK                                    (0x0020U)
#define CSL_SDFM_SDCTLPARM2_RESERVED_2_SHIFT                                   (0x0005U)
#define CSL_SDFM_SDCTLPARM2_RESERVED_2_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCTLPARM2_RESERVED_2_MAX                                     (0x0001U)

#define CSL_SDFM_SDCTLPARM2_SDDATASYNC_MASK                                    (0x0040U)
#define CSL_SDFM_SDCTLPARM2_SDDATASYNC_SHIFT                                   (0x0006U)
#define CSL_SDFM_SDCTLPARM2_SDDATASYNC_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCTLPARM2_SDDATASYNC_MAX                                     (0x0001U)

#define CSL_SDFM_SDCTLPARM2_RESERVED_3_MASK                                    (0x0080U)
#define CSL_SDFM_SDCTLPARM2_RESERVED_3_SHIFT                                   (0x0007U)
#define CSL_SDFM_SDCTLPARM2_RESERVED_3_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCTLPARM2_RESERVED_3_MAX                                     (0x0001U)

#define CSL_SDFM_SDCTLPARM2_RESERVED_4_MASK                                    (0xFF00U)
#define CSL_SDFM_SDCTLPARM2_RESERVED_4_SHIFT                                   (0x0008U)
#define CSL_SDFM_SDCTLPARM2_RESERVED_4_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCTLPARM2_RESERVED_4_MAX                                     (0x00FFU)

#define CSL_SDFM_SDCTLPARM2_RESETVAL                                           (0x0000U)

/* SDDFPARM2 */

#define CSL_SDFM_SDDFPARM2_DOSR_MASK                                           (0x00FFU)
#define CSL_SDFM_SDDFPARM2_DOSR_SHIFT                                          (0x0000U)
#define CSL_SDFM_SDDFPARM2_DOSR_RESETVAL                                       (0x0000U)
#define CSL_SDFM_SDDFPARM2_DOSR_MAX                                            (0x00FFU)

#define CSL_SDFM_SDDFPARM2_FEN_MASK                                            (0x0100U)
#define CSL_SDFM_SDDFPARM2_FEN_SHIFT                                           (0x0008U)
#define CSL_SDFM_SDDFPARM2_FEN_RESETVAL                                        (0x0000U)
#define CSL_SDFM_SDDFPARM2_FEN_MAX                                             (0x0001U)

#define CSL_SDFM_SDDFPARM2_AE_MASK                                             (0x0200U)
#define CSL_SDFM_SDDFPARM2_AE_SHIFT                                            (0x0009U)
#define CSL_SDFM_SDDFPARM2_AE_RESETVAL                                         (0x0000U)
#define CSL_SDFM_SDDFPARM2_AE_MAX                                              (0x0001U)

#define CSL_SDFM_SDDFPARM2_SST_MASK                                            (0x0C00U)
#define CSL_SDFM_SDDFPARM2_SST_SHIFT                                           (0x000AU)
#define CSL_SDFM_SDDFPARM2_SST_RESETVAL                                        (0x0000U)
#define CSL_SDFM_SDDFPARM2_SST_MAX                                             (0x0003U)

#define CSL_SDFM_SDDFPARM2_SDSYNCEN_MASK                                       (0x1000U)
#define CSL_SDFM_SDDFPARM2_SDSYNCEN_SHIFT                                      (0x000CU)
#define CSL_SDFM_SDDFPARM2_SDSYNCEN_RESETVAL                                   (0x0000U)
#define CSL_SDFM_SDDFPARM2_SDSYNCEN_MAX                                        (0x0001U)

#define CSL_SDFM_SDDFPARM2_RESERVED_1_MASK                                     (0xE000U)
#define CSL_SDFM_SDDFPARM2_RESERVED_1_SHIFT                                    (0x000DU)
#define CSL_SDFM_SDDFPARM2_RESERVED_1_RESETVAL                                 (0x0000U)
#define CSL_SDFM_SDDFPARM2_RESERVED_1_MAX                                      (0x0007U)

#define CSL_SDFM_SDDFPARM2_RESETVAL                                            (0x0000U)

/* SDDPARM2 */

#define CSL_SDFM_SDDPARM2_RESERVED_1_MASK                                      (0x03FFU)
#define CSL_SDFM_SDDPARM2_RESERVED_1_SHIFT                                     (0x0000U)
#define CSL_SDFM_SDDPARM2_RESERVED_1_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDDPARM2_RESERVED_1_MAX                                       (0x03FFU)

#define CSL_SDFM_SDDPARM2_DR_MASK                                              (0x0400U)
#define CSL_SDFM_SDDPARM2_DR_SHIFT                                             (0x000AU)
#define CSL_SDFM_SDDPARM2_DR_RESETVAL                                          (0x0000U)
#define CSL_SDFM_SDDPARM2_DR_MAX                                               (0x0001U)

#define CSL_SDFM_SDDPARM2_SH_MASK                                              (0xF800U)
#define CSL_SDFM_SDDPARM2_SH_SHIFT                                             (0x000BU)
#define CSL_SDFM_SDDPARM2_SH_RESETVAL                                          (0x0000U)
#define CSL_SDFM_SDDPARM2_SH_MAX                                               (0x001FU)

#define CSL_SDFM_SDDPARM2_RESETVAL                                             (0x0000U)

/* SDFLT2CMPH1 */

#define CSL_SDFM_SDFLT2CMPH1_HLT_MASK                                          (0x7FFFU)
#define CSL_SDFM_SDFLT2CMPH1_HLT_SHIFT                                         (0x0000U)
#define CSL_SDFM_SDFLT2CMPH1_HLT_RESETVAL                                      (0x7FFFU)
#define CSL_SDFM_SDFLT2CMPH1_HLT_MAX                                           (0x7FFFU)

#define CSL_SDFM_SDFLT2CMPH1_RESERVED_1_MASK                                   (0x8000U)
#define CSL_SDFM_SDFLT2CMPH1_RESERVED_1_SHIFT                                  (0x000FU)
#define CSL_SDFM_SDFLT2CMPH1_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDFLT2CMPH1_RESERVED_1_MAX                                    (0x0001U)

#define CSL_SDFM_SDFLT2CMPH1_RESETVAL                                          (0x7FFFU)

/* SDFLT2CMPL1 */

#define CSL_SDFM_SDFLT2CMPL1_LLT_MASK                                          (0x7FFFU)
#define CSL_SDFM_SDFLT2CMPL1_LLT_SHIFT                                         (0x0000U)
#define CSL_SDFM_SDFLT2CMPL1_LLT_RESETVAL                                      (0x0000U)
#define CSL_SDFM_SDFLT2CMPL1_LLT_MAX                                           (0x7FFFU)

#define CSL_SDFM_SDFLT2CMPL1_RESERVED_1_MASK                                   (0x8000U)
#define CSL_SDFM_SDFLT2CMPL1_RESERVED_1_SHIFT                                  (0x000FU)
#define CSL_SDFM_SDFLT2CMPL1_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDFLT2CMPL1_RESERVED_1_MAX                                    (0x0001U)

#define CSL_SDFM_SDFLT2CMPL1_RESETVAL                                          (0x0000U)

/* SDCPARM2 */

#define CSL_SDFM_SDCPARM2_COSR_MASK                                            (0x001FU)
#define CSL_SDFM_SDCPARM2_COSR_SHIFT                                           (0x0000U)
#define CSL_SDFM_SDCPARM2_COSR_RESETVAL                                        (0x0000U)
#define CSL_SDFM_SDCPARM2_COSR_MAX                                             (0x001FU)

#define CSL_SDFM_SDCPARM2_EN_CEVT1_MASK                                        (0x0020U)
#define CSL_SDFM_SDCPARM2_EN_CEVT1_SHIFT                                       (0x0005U)
#define CSL_SDFM_SDCPARM2_EN_CEVT1_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDCPARM2_EN_CEVT1_MAX                                         (0x0001U)

#define CSL_SDFM_SDCPARM2_EN_CEVT2_MASK                                        (0x0040U)
#define CSL_SDFM_SDCPARM2_EN_CEVT2_SHIFT                                       (0x0006U)
#define CSL_SDFM_SDCPARM2_EN_CEVT2_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDCPARM2_EN_CEVT2_MAX                                         (0x0001U)

#define CSL_SDFM_SDCPARM2_CS1_CS0_MASK                                         (0x0180U)
#define CSL_SDFM_SDCPARM2_CS1_CS0_SHIFT                                        (0x0007U)
#define CSL_SDFM_SDCPARM2_CS1_CS0_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDCPARM2_CS1_CS0_MAX                                          (0x0003U)

#define CSL_SDFM_SDCPARM2_MFIE_MASK                                            (0x0200U)
#define CSL_SDFM_SDCPARM2_MFIE_SHIFT                                           (0x0009U)
#define CSL_SDFM_SDCPARM2_MFIE_RESETVAL                                        (0x0000U)
#define CSL_SDFM_SDCPARM2_MFIE_MAX                                             (0x0001U)

#define CSL_SDFM_SDCPARM2_HZEN_MASK                                            (0x0400U)
#define CSL_SDFM_SDCPARM2_HZEN_SHIFT                                           (0x000AU)
#define CSL_SDFM_SDCPARM2_HZEN_RESETVAL                                        (0x0000U)
#define CSL_SDFM_SDCPARM2_HZEN_MAX                                             (0x0001U)

#define CSL_SDFM_SDCPARM2_CEVT1SEL_MASK                                        (0x1800U)
#define CSL_SDFM_SDCPARM2_CEVT1SEL_SHIFT                                       (0x000BU)
#define CSL_SDFM_SDCPARM2_CEVT1SEL_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDCPARM2_CEVT1SEL_MAX                                         (0x0003U)

#define CSL_SDFM_SDCPARM2_CEN_MASK                                             (0x2000U)
#define CSL_SDFM_SDCPARM2_CEN_SHIFT                                            (0x000DU)
#define CSL_SDFM_SDCPARM2_CEN_RESETVAL                                         (0x0000U)
#define CSL_SDFM_SDCPARM2_CEN_MAX                                              (0x0001U)

#define CSL_SDFM_SDCPARM2_CEVT2SEL_MASK                                        (0xC000U)
#define CSL_SDFM_SDCPARM2_CEVT2SEL_SHIFT                                       (0x000EU)
#define CSL_SDFM_SDCPARM2_CEVT2SEL_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDCPARM2_CEVT2SEL_MAX                                         (0x0003U)

#define CSL_SDFM_SDCPARM2_RESETVAL                                             (0x0000U)

/* SDDATA2 */

#define CSL_SDFM_SDDATA2_DATA16_MASK                                           (0x0000FFFFU)
#define CSL_SDFM_SDDATA2_DATA16_SHIFT                                          (0x00000000U)
#define CSL_SDFM_SDDATA2_DATA16_RESETVAL                                       (0x00000000U)
#define CSL_SDFM_SDDATA2_DATA16_MAX                                            (0x0000FFFFU)

#define CSL_SDFM_SDDATA2_DATA32HI_MASK                                         (0xFFFF0000U)
#define CSL_SDFM_SDDATA2_DATA32HI_SHIFT                                        (0x00000010U)
#define CSL_SDFM_SDDATA2_DATA32HI_RESETVAL                                     (0x00000000U)
#define CSL_SDFM_SDDATA2_DATA32HI_MAX                                          (0x0000FFFFU)

#define CSL_SDFM_SDDATA2_RESETVAL                                              (0x00000000U)

/* SDDATFIFO2 */

#define CSL_SDFM_SDDATFIFO2_DATA16_MASK                                        (0x0000FFFFU)
#define CSL_SDFM_SDDATFIFO2_DATA16_SHIFT                                       (0x00000000U)
#define CSL_SDFM_SDDATFIFO2_DATA16_RESETVAL                                    (0x00000000U)
#define CSL_SDFM_SDDATFIFO2_DATA16_MAX                                         (0x0000FFFFU)

#define CSL_SDFM_SDDATFIFO2_DATA32HI_MASK                                      (0xFFFF0000U)
#define CSL_SDFM_SDDATFIFO2_DATA32HI_SHIFT                                     (0x00000010U)
#define CSL_SDFM_SDDATFIFO2_DATA32HI_RESETVAL                                  (0x00000000U)
#define CSL_SDFM_SDDATFIFO2_DATA32HI_MAX                                       (0x0000FFFFU)

#define CSL_SDFM_SDDATFIFO2_RESETVAL                                           (0x00000000U)

/* SDCDATA2 */

#define CSL_SDFM_SDCDATA2_DATA16_MASK                                          (0xFFFFU)
#define CSL_SDFM_SDCDATA2_DATA16_SHIFT                                         (0x0000U)
#define CSL_SDFM_SDCDATA2_DATA16_RESETVAL                                      (0x0000U)
#define CSL_SDFM_SDCDATA2_DATA16_MAX                                           (0xFFFFU)

#define CSL_SDFM_SDCDATA2_RESETVAL                                             (0x0000U)

/* SDFLT2CMPH2 */

#define CSL_SDFM_SDFLT2CMPH2_HLT2_MASK                                         (0x7FFFU)
#define CSL_SDFM_SDFLT2CMPH2_HLT2_SHIFT                                        (0x0000U)
#define CSL_SDFM_SDFLT2CMPH2_HLT2_RESETVAL                                     (0x7FFFU)
#define CSL_SDFM_SDFLT2CMPH2_HLT2_MAX                                          (0x7FFFU)

#define CSL_SDFM_SDFLT2CMPH2_RESERVED_1_MASK                                   (0x8000U)
#define CSL_SDFM_SDFLT2CMPH2_RESERVED_1_SHIFT                                  (0x000FU)
#define CSL_SDFM_SDFLT2CMPH2_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDFLT2CMPH2_RESERVED_1_MAX                                    (0x0001U)

#define CSL_SDFM_SDFLT2CMPH2_RESETVAL                                          (0x7FFFU)

/* SDFLT2CMPHZ */

#define CSL_SDFM_SDFLT2CMPHZ_HLTZ_MASK                                         (0x7FFFU)
#define CSL_SDFM_SDFLT2CMPHZ_HLTZ_SHIFT                                        (0x0000U)
#define CSL_SDFM_SDFLT2CMPHZ_HLTZ_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDFLT2CMPHZ_HLTZ_MAX                                          (0x7FFFU)

#define CSL_SDFM_SDFLT2CMPHZ_RESERVED_1_MASK                                   (0x8000U)
#define CSL_SDFM_SDFLT2CMPHZ_RESERVED_1_SHIFT                                  (0x000FU)
#define CSL_SDFM_SDFLT2CMPHZ_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDFLT2CMPHZ_RESERVED_1_MAX                                    (0x0001U)

#define CSL_SDFM_SDFLT2CMPHZ_RESETVAL                                          (0x0000U)

/* SDFIFOCTL2 */

#define CSL_SDFM_SDFIFOCTL2_SDFFIL_MASK                                        (0x001FU)
#define CSL_SDFM_SDFIFOCTL2_SDFFIL_SHIFT                                       (0x0000U)
#define CSL_SDFM_SDFIFOCTL2_SDFFIL_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDFIFOCTL2_SDFFIL_MAX                                         (0x001FU)

#define CSL_SDFM_SDFIFOCTL2_RESERVED_1_MASK                                    (0x0020U)
#define CSL_SDFM_SDFIFOCTL2_RESERVED_1_SHIFT                                   (0x0005U)
#define CSL_SDFM_SDFIFOCTL2_RESERVED_1_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDFIFOCTL2_RESERVED_1_MAX                                     (0x0001U)

#define CSL_SDFM_SDFIFOCTL2_SDFFST_MASK                                        (0x07C0U)
#define CSL_SDFM_SDFIFOCTL2_SDFFST_SHIFT                                       (0x0006U)
#define CSL_SDFM_SDFIFOCTL2_SDFFST_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDFIFOCTL2_SDFFST_MAX                                         (0x001FU)

#define CSL_SDFM_SDFIFOCTL2_RESERVED_2_MASK                                    (0x0800U)
#define CSL_SDFM_SDFIFOCTL2_RESERVED_2_SHIFT                                   (0x000BU)
#define CSL_SDFM_SDFIFOCTL2_RESERVED_2_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDFIFOCTL2_RESERVED_2_MAX                                     (0x0001U)

#define CSL_SDFM_SDFIFOCTL2_FFIEN_MASK                                         (0x1000U)
#define CSL_SDFM_SDFIFOCTL2_FFIEN_SHIFT                                        (0x000CU)
#define CSL_SDFM_SDFIFOCTL2_FFIEN_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDFIFOCTL2_FFIEN_MAX                                          (0x0001U)

#define CSL_SDFM_SDFIFOCTL2_FFEN_MASK                                          (0x2000U)
#define CSL_SDFM_SDFIFOCTL2_FFEN_SHIFT                                         (0x000DU)
#define CSL_SDFM_SDFIFOCTL2_FFEN_RESETVAL                                      (0x0000U)
#define CSL_SDFM_SDFIFOCTL2_FFEN_MAX                                           (0x0001U)

#define CSL_SDFM_SDFIFOCTL2_DRINTSEL_MASK                                      (0x4000U)
#define CSL_SDFM_SDFIFOCTL2_DRINTSEL_SHIFT                                     (0x000EU)
#define CSL_SDFM_SDFIFOCTL2_DRINTSEL_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDFIFOCTL2_DRINTSEL_MAX                                       (0x0001U)

#define CSL_SDFM_SDFIFOCTL2_OVFIEN_MASK                                        (0x8000U)
#define CSL_SDFM_SDFIFOCTL2_OVFIEN_SHIFT                                       (0x000FU)
#define CSL_SDFM_SDFIFOCTL2_OVFIEN_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDFIFOCTL2_OVFIEN_MAX                                         (0x0001U)

#define CSL_SDFM_SDFIFOCTL2_RESETVAL                                           (0x0000U)

/* SDSYNC2 */

#define CSL_SDFM_SDSYNC2_SYNCSEL_MASK                                          (0x003FU)
#define CSL_SDFM_SDSYNC2_SYNCSEL_SHIFT                                         (0x0000U)
#define CSL_SDFM_SDSYNC2_SYNCSEL_RESETVAL                                      (0x0000U)
#define CSL_SDFM_SDSYNC2_SYNCSEL_MAX                                           (0x003FU)

#define CSL_SDFM_SDSYNC2_WTSYNCEN_MASK                                         (0x0040U)
#define CSL_SDFM_SDSYNC2_WTSYNCEN_SHIFT                                        (0x0006U)
#define CSL_SDFM_SDSYNC2_WTSYNCEN_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDSYNC2_WTSYNCEN_MAX                                          (0x0001U)

#define CSL_SDFM_SDSYNC2_WTSYNFLG_MASK                                         (0x0080U)
#define CSL_SDFM_SDSYNC2_WTSYNFLG_SHIFT                                        (0x0007U)
#define CSL_SDFM_SDSYNC2_WTSYNFLG_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDSYNC2_WTSYNFLG_MAX                                          (0x0001U)

#define CSL_SDFM_SDSYNC2_WTSYNCLR_MASK                                         (0x0100U)
#define CSL_SDFM_SDSYNC2_WTSYNCLR_SHIFT                                        (0x0008U)
#define CSL_SDFM_SDSYNC2_WTSYNCLR_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDSYNC2_WTSYNCLR_MAX                                          (0x0001U)

#define CSL_SDFM_SDSYNC2_FFSYNCCLREN_MASK                                      (0x0200U)
#define CSL_SDFM_SDSYNC2_FFSYNCCLREN_SHIFT                                     (0x0009U)
#define CSL_SDFM_SDSYNC2_FFSYNCCLREN_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDSYNC2_FFSYNCCLREN_MAX                                       (0x0001U)

#define CSL_SDFM_SDSYNC2_WTSCLREN_MASK                                         (0x0400U)
#define CSL_SDFM_SDSYNC2_WTSCLREN_SHIFT                                        (0x000AU)
#define CSL_SDFM_SDSYNC2_WTSCLREN_RESETVAL                                     (0x0001U)
#define CSL_SDFM_SDSYNC2_WTSCLREN_MAX                                          (0x0001U)

#define CSL_SDFM_SDSYNC2_RESERVED_1_MASK                                       (0xF800U)
#define CSL_SDFM_SDSYNC2_RESERVED_1_SHIFT                                      (0x000BU)
#define CSL_SDFM_SDSYNC2_RESERVED_1_RESETVAL                                   (0x0000U)
#define CSL_SDFM_SDSYNC2_RESERVED_1_MAX                                        (0x001FU)

#define CSL_SDFM_SDSYNC2_RESETVAL                                              (0x0400U)

/* SDFLT2CMPL2 */

#define CSL_SDFM_SDFLT2CMPL2_LLT2_MASK                                         (0x7FFFU)
#define CSL_SDFM_SDFLT2CMPL2_LLT2_SHIFT                                        (0x0000U)
#define CSL_SDFM_SDFLT2CMPL2_LLT2_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDFLT2CMPL2_LLT2_MAX                                          (0x7FFFU)

#define CSL_SDFM_SDFLT2CMPL2_RESERVED_1_MASK                                   (0x8000U)
#define CSL_SDFM_SDFLT2CMPL2_RESERVED_1_SHIFT                                  (0x000FU)
#define CSL_SDFM_SDFLT2CMPL2_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDFLT2CMPL2_RESERVED_1_MAX                                    (0x0001U)

#define CSL_SDFM_SDFLT2CMPL2_RESETVAL                                          (0x0000U)

/* SDCTLPARM3 */

#define CSL_SDFM_SDCTLPARM3_MOD_MASK                                           (0x0003U)
#define CSL_SDFM_SDCTLPARM3_MOD_SHIFT                                          (0x0000U)
#define CSL_SDFM_SDCTLPARM3_MOD_RESETVAL                                       (0x0000U)
#define CSL_SDFM_SDCTLPARM3_MOD_MAX                                            (0x0003U)

#define CSL_SDFM_SDCTLPARM3_RESERVED_1_MASK                                    (0x0004U)
#define CSL_SDFM_SDCTLPARM3_RESERVED_1_SHIFT                                   (0x0002U)
#define CSL_SDFM_SDCTLPARM3_RESERVED_1_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCTLPARM3_RESERVED_1_MAX                                     (0x0001U)

#define CSL_SDFM_SDCTLPARM3_SDCLKSEL_MASK                                      (0x0008U)
#define CSL_SDFM_SDCTLPARM3_SDCLKSEL_SHIFT                                     (0x0003U)
#define CSL_SDFM_SDCTLPARM3_SDCLKSEL_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDCTLPARM3_SDCLKSEL_MAX                                       (0x0001U)

#define CSL_SDFM_SDCTLPARM3_SDCLKSYNC_MASK                                     (0x0010U)
#define CSL_SDFM_SDCTLPARM3_SDCLKSYNC_SHIFT                                    (0x0004U)
#define CSL_SDFM_SDCTLPARM3_SDCLKSYNC_RESETVAL                                 (0x0000U)
#define CSL_SDFM_SDCTLPARM3_SDCLKSYNC_MAX                                      (0x0001U)

#define CSL_SDFM_SDCTLPARM3_RESERVED_2_MASK                                    (0x0020U)
#define CSL_SDFM_SDCTLPARM3_RESERVED_2_SHIFT                                   (0x0005U)
#define CSL_SDFM_SDCTLPARM3_RESERVED_2_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCTLPARM3_RESERVED_2_MAX                                     (0x0001U)

#define CSL_SDFM_SDCTLPARM3_SDDATASYNC_MASK                                    (0x0040U)
#define CSL_SDFM_SDCTLPARM3_SDDATASYNC_SHIFT                                   (0x0006U)
#define CSL_SDFM_SDCTLPARM3_SDDATASYNC_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCTLPARM3_SDDATASYNC_MAX                                     (0x0001U)

#define CSL_SDFM_SDCTLPARM3_RESERVED_3_MASK                                    (0x0080U)
#define CSL_SDFM_SDCTLPARM3_RESERVED_3_SHIFT                                   (0x0007U)
#define CSL_SDFM_SDCTLPARM3_RESERVED_3_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCTLPARM3_RESERVED_3_MAX                                     (0x0001U)

#define CSL_SDFM_SDCTLPARM3_RESERVED_4_MASK                                    (0xFF00U)
#define CSL_SDFM_SDCTLPARM3_RESERVED_4_SHIFT                                   (0x0008U)
#define CSL_SDFM_SDCTLPARM3_RESERVED_4_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCTLPARM3_RESERVED_4_MAX                                     (0x00FFU)

#define CSL_SDFM_SDCTLPARM3_RESETVAL                                           (0x0000U)

/* SDDFPARM3 */

#define CSL_SDFM_SDDFPARM3_DOSR_MASK                                           (0x00FFU)
#define CSL_SDFM_SDDFPARM3_DOSR_SHIFT                                          (0x0000U)
#define CSL_SDFM_SDDFPARM3_DOSR_RESETVAL                                       (0x0000U)
#define CSL_SDFM_SDDFPARM3_DOSR_MAX                                            (0x00FFU)

#define CSL_SDFM_SDDFPARM3_FEN_MASK                                            (0x0100U)
#define CSL_SDFM_SDDFPARM3_FEN_SHIFT                                           (0x0008U)
#define CSL_SDFM_SDDFPARM3_FEN_RESETVAL                                        (0x0000U)
#define CSL_SDFM_SDDFPARM3_FEN_MAX                                             (0x0001U)

#define CSL_SDFM_SDDFPARM3_AE_MASK                                             (0x0200U)
#define CSL_SDFM_SDDFPARM3_AE_SHIFT                                            (0x0009U)
#define CSL_SDFM_SDDFPARM3_AE_RESETVAL                                         (0x0000U)
#define CSL_SDFM_SDDFPARM3_AE_MAX                                              (0x0001U)

#define CSL_SDFM_SDDFPARM3_SST_MASK                                            (0x0C00U)
#define CSL_SDFM_SDDFPARM3_SST_SHIFT                                           (0x000AU)
#define CSL_SDFM_SDDFPARM3_SST_RESETVAL                                        (0x0000U)
#define CSL_SDFM_SDDFPARM3_SST_MAX                                             (0x0003U)

#define CSL_SDFM_SDDFPARM3_SDSYNCEN_MASK                                       (0x1000U)
#define CSL_SDFM_SDDFPARM3_SDSYNCEN_SHIFT                                      (0x000CU)
#define CSL_SDFM_SDDFPARM3_SDSYNCEN_RESETVAL                                   (0x0000U)
#define CSL_SDFM_SDDFPARM3_SDSYNCEN_MAX                                        (0x0001U)

#define CSL_SDFM_SDDFPARM3_RESERVED_1_MASK                                     (0xE000U)
#define CSL_SDFM_SDDFPARM3_RESERVED_1_SHIFT                                    (0x000DU)
#define CSL_SDFM_SDDFPARM3_RESERVED_1_RESETVAL                                 (0x0000U)
#define CSL_SDFM_SDDFPARM3_RESERVED_1_MAX                                      (0x0007U)

#define CSL_SDFM_SDDFPARM3_RESETVAL                                            (0x0000U)

/* SDDPARM3 */

#define CSL_SDFM_SDDPARM3_RESERVED_1_MASK                                      (0x03FFU)
#define CSL_SDFM_SDDPARM3_RESERVED_1_SHIFT                                     (0x0000U)
#define CSL_SDFM_SDDPARM3_RESERVED_1_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDDPARM3_RESERVED_1_MAX                                       (0x03FFU)

#define CSL_SDFM_SDDPARM3_DR_MASK                                              (0x0400U)
#define CSL_SDFM_SDDPARM3_DR_SHIFT                                             (0x000AU)
#define CSL_SDFM_SDDPARM3_DR_RESETVAL                                          (0x0000U)
#define CSL_SDFM_SDDPARM3_DR_MAX                                               (0x0001U)

#define CSL_SDFM_SDDPARM3_SH_MASK                                              (0xF800U)
#define CSL_SDFM_SDDPARM3_SH_SHIFT                                             (0x000BU)
#define CSL_SDFM_SDDPARM3_SH_RESETVAL                                          (0x0000U)
#define CSL_SDFM_SDDPARM3_SH_MAX                                               (0x001FU)

#define CSL_SDFM_SDDPARM3_RESETVAL                                             (0x0000U)

/* SDFLT3CMPH1 */

#define CSL_SDFM_SDFLT3CMPH1_HLT_MASK                                          (0x7FFFU)
#define CSL_SDFM_SDFLT3CMPH1_HLT_SHIFT                                         (0x0000U)
#define CSL_SDFM_SDFLT3CMPH1_HLT_RESETVAL                                      (0x7FFFU)
#define CSL_SDFM_SDFLT3CMPH1_HLT_MAX                                           (0x7FFFU)

#define CSL_SDFM_SDFLT3CMPH1_RESERVED_1_MASK                                   (0x8000U)
#define CSL_SDFM_SDFLT3CMPH1_RESERVED_1_SHIFT                                  (0x000FU)
#define CSL_SDFM_SDFLT3CMPH1_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDFLT3CMPH1_RESERVED_1_MAX                                    (0x0001U)

#define CSL_SDFM_SDFLT3CMPH1_RESETVAL                                          (0x7FFFU)

/* SDFLT3CMPL1 */

#define CSL_SDFM_SDFLT3CMPL1_LLT_MASK                                          (0x7FFFU)
#define CSL_SDFM_SDFLT3CMPL1_LLT_SHIFT                                         (0x0000U)
#define CSL_SDFM_SDFLT3CMPL1_LLT_RESETVAL                                      (0x0000U)
#define CSL_SDFM_SDFLT3CMPL1_LLT_MAX                                           (0x7FFFU)

#define CSL_SDFM_SDFLT3CMPL1_RESERVED_1_MASK                                   (0x8000U)
#define CSL_SDFM_SDFLT3CMPL1_RESERVED_1_SHIFT                                  (0x000FU)
#define CSL_SDFM_SDFLT3CMPL1_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDFLT3CMPL1_RESERVED_1_MAX                                    (0x0001U)

#define CSL_SDFM_SDFLT3CMPL1_RESETVAL                                          (0x0000U)

/* SDCPARM3 */

#define CSL_SDFM_SDCPARM3_COSR_MASK                                            (0x001FU)
#define CSL_SDFM_SDCPARM3_COSR_SHIFT                                           (0x0000U)
#define CSL_SDFM_SDCPARM3_COSR_RESETVAL                                        (0x0000U)
#define CSL_SDFM_SDCPARM3_COSR_MAX                                             (0x001FU)

#define CSL_SDFM_SDCPARM3_EN_CEVT1_MASK                                        (0x0020U)
#define CSL_SDFM_SDCPARM3_EN_CEVT1_SHIFT                                       (0x0005U)
#define CSL_SDFM_SDCPARM3_EN_CEVT1_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDCPARM3_EN_CEVT1_MAX                                         (0x0001U)

#define CSL_SDFM_SDCPARM3_EN_CEVT2_MASK                                        (0x0040U)
#define CSL_SDFM_SDCPARM3_EN_CEVT2_SHIFT                                       (0x0006U)
#define CSL_SDFM_SDCPARM3_EN_CEVT2_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDCPARM3_EN_CEVT2_MAX                                         (0x0001U)

#define CSL_SDFM_SDCPARM3_CS1_CS0_MASK                                         (0x0180U)
#define CSL_SDFM_SDCPARM3_CS1_CS0_SHIFT                                        (0x0007U)
#define CSL_SDFM_SDCPARM3_CS1_CS0_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDCPARM3_CS1_CS0_MAX                                          (0x0003U)

#define CSL_SDFM_SDCPARM3_MFIE_MASK                                            (0x0200U)
#define CSL_SDFM_SDCPARM3_MFIE_SHIFT                                           (0x0009U)
#define CSL_SDFM_SDCPARM3_MFIE_RESETVAL                                        (0x0000U)
#define CSL_SDFM_SDCPARM3_MFIE_MAX                                             (0x0001U)

#define CSL_SDFM_SDCPARM3_HZEN_MASK                                            (0x0400U)
#define CSL_SDFM_SDCPARM3_HZEN_SHIFT                                           (0x000AU)
#define CSL_SDFM_SDCPARM3_HZEN_RESETVAL                                        (0x0000U)
#define CSL_SDFM_SDCPARM3_HZEN_MAX                                             (0x0001U)

#define CSL_SDFM_SDCPARM3_CEVT1SEL_MASK                                        (0x1800U)
#define CSL_SDFM_SDCPARM3_CEVT1SEL_SHIFT                                       (0x000BU)
#define CSL_SDFM_SDCPARM3_CEVT1SEL_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDCPARM3_CEVT1SEL_MAX                                         (0x0003U)

#define CSL_SDFM_SDCPARM3_CEN_MASK                                             (0x2000U)
#define CSL_SDFM_SDCPARM3_CEN_SHIFT                                            (0x000DU)
#define CSL_SDFM_SDCPARM3_CEN_RESETVAL                                         (0x0000U)
#define CSL_SDFM_SDCPARM3_CEN_MAX                                              (0x0001U)

#define CSL_SDFM_SDCPARM3_CEVT2SEL_MASK                                        (0xC000U)
#define CSL_SDFM_SDCPARM3_CEVT2SEL_SHIFT                                       (0x000EU)
#define CSL_SDFM_SDCPARM3_CEVT2SEL_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDCPARM3_CEVT2SEL_MAX                                         (0x0003U)

#define CSL_SDFM_SDCPARM3_RESETVAL                                             (0x0000U)

/* SDDATA3 */

#define CSL_SDFM_SDDATA3_DATA16_MASK                                           (0x0000FFFFU)
#define CSL_SDFM_SDDATA3_DATA16_SHIFT                                          (0x00000000U)
#define CSL_SDFM_SDDATA3_DATA16_RESETVAL                                       (0x00000000U)
#define CSL_SDFM_SDDATA3_DATA16_MAX                                            (0x0000FFFFU)

#define CSL_SDFM_SDDATA3_DATA32HI_MASK                                         (0xFFFF0000U)
#define CSL_SDFM_SDDATA3_DATA32HI_SHIFT                                        (0x00000010U)
#define CSL_SDFM_SDDATA3_DATA32HI_RESETVAL                                     (0x00000000U)
#define CSL_SDFM_SDDATA3_DATA32HI_MAX                                          (0x0000FFFFU)

#define CSL_SDFM_SDDATA3_RESETVAL                                              (0x00000000U)

/* SDDATFIFO3 */

#define CSL_SDFM_SDDATFIFO3_DATA16_MASK                                        (0x0000FFFFU)
#define CSL_SDFM_SDDATFIFO3_DATA16_SHIFT                                       (0x00000000U)
#define CSL_SDFM_SDDATFIFO3_DATA16_RESETVAL                                    (0x00000000U)
#define CSL_SDFM_SDDATFIFO3_DATA16_MAX                                         (0x0000FFFFU)

#define CSL_SDFM_SDDATFIFO3_DATA32HI_MASK                                      (0xFFFF0000U)
#define CSL_SDFM_SDDATFIFO3_DATA32HI_SHIFT                                     (0x00000010U)
#define CSL_SDFM_SDDATFIFO3_DATA32HI_RESETVAL                                  (0x00000000U)
#define CSL_SDFM_SDDATFIFO3_DATA32HI_MAX                                       (0x0000FFFFU)

#define CSL_SDFM_SDDATFIFO3_RESETVAL                                           (0x00000000U)

/* SDCDATA3 */

#define CSL_SDFM_SDCDATA3_DATA16_MASK                                          (0xFFFFU)
#define CSL_SDFM_SDCDATA3_DATA16_SHIFT                                         (0x0000U)
#define CSL_SDFM_SDCDATA3_DATA16_RESETVAL                                      (0x0000U)
#define CSL_SDFM_SDCDATA3_DATA16_MAX                                           (0xFFFFU)

#define CSL_SDFM_SDCDATA3_RESETVAL                                             (0x0000U)

/* SDFLT3CMPH2 */

#define CSL_SDFM_SDFLT3CMPH2_HLT2_MASK                                         (0x7FFFU)
#define CSL_SDFM_SDFLT3CMPH2_HLT2_SHIFT                                        (0x0000U)
#define CSL_SDFM_SDFLT3CMPH2_HLT2_RESETVAL                                     (0x7FFFU)
#define CSL_SDFM_SDFLT3CMPH2_HLT2_MAX                                          (0x7FFFU)

#define CSL_SDFM_SDFLT3CMPH2_RESERVED_1_MASK                                   (0x8000U)
#define CSL_SDFM_SDFLT3CMPH2_RESERVED_1_SHIFT                                  (0x000FU)
#define CSL_SDFM_SDFLT3CMPH2_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDFLT3CMPH2_RESERVED_1_MAX                                    (0x0001U)

#define CSL_SDFM_SDFLT3CMPH2_RESETVAL                                          (0x7FFFU)

/* SDFLT3CMPHZ */

#define CSL_SDFM_SDFLT3CMPHZ_HLTZ_MASK                                         (0x7FFFU)
#define CSL_SDFM_SDFLT3CMPHZ_HLTZ_SHIFT                                        (0x0000U)
#define CSL_SDFM_SDFLT3CMPHZ_HLTZ_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDFLT3CMPHZ_HLTZ_MAX                                          (0x7FFFU)

#define CSL_SDFM_SDFLT3CMPHZ_RESERVED_1_MASK                                   (0x8000U)
#define CSL_SDFM_SDFLT3CMPHZ_RESERVED_1_SHIFT                                  (0x000FU)
#define CSL_SDFM_SDFLT3CMPHZ_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDFLT3CMPHZ_RESERVED_1_MAX                                    (0x0001U)

#define CSL_SDFM_SDFLT3CMPHZ_RESETVAL                                          (0x0000U)

/* SDFIFOCTL3 */

#define CSL_SDFM_SDFIFOCTL3_SDFFIL_MASK                                        (0x001FU)
#define CSL_SDFM_SDFIFOCTL3_SDFFIL_SHIFT                                       (0x0000U)
#define CSL_SDFM_SDFIFOCTL3_SDFFIL_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDFIFOCTL3_SDFFIL_MAX                                         (0x001FU)

#define CSL_SDFM_SDFIFOCTL3_RESERVED_1_MASK                                    (0x0020U)
#define CSL_SDFM_SDFIFOCTL3_RESERVED_1_SHIFT                                   (0x0005U)
#define CSL_SDFM_SDFIFOCTL3_RESERVED_1_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDFIFOCTL3_RESERVED_1_MAX                                     (0x0001U)

#define CSL_SDFM_SDFIFOCTL3_SDFFST_MASK                                        (0x07C0U)
#define CSL_SDFM_SDFIFOCTL3_SDFFST_SHIFT                                       (0x0006U)
#define CSL_SDFM_SDFIFOCTL3_SDFFST_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDFIFOCTL3_SDFFST_MAX                                         (0x001FU)

#define CSL_SDFM_SDFIFOCTL3_RESERVED_2_MASK                                    (0x0800U)
#define CSL_SDFM_SDFIFOCTL3_RESERVED_2_SHIFT                                   (0x000BU)
#define CSL_SDFM_SDFIFOCTL3_RESERVED_2_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDFIFOCTL3_RESERVED_2_MAX                                     (0x0001U)

#define CSL_SDFM_SDFIFOCTL3_FFIEN_MASK                                         (0x1000U)
#define CSL_SDFM_SDFIFOCTL3_FFIEN_SHIFT                                        (0x000CU)
#define CSL_SDFM_SDFIFOCTL3_FFIEN_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDFIFOCTL3_FFIEN_MAX                                          (0x0001U)

#define CSL_SDFM_SDFIFOCTL3_FFEN_MASK                                          (0x2000U)
#define CSL_SDFM_SDFIFOCTL3_FFEN_SHIFT                                         (0x000DU)
#define CSL_SDFM_SDFIFOCTL3_FFEN_RESETVAL                                      (0x0000U)
#define CSL_SDFM_SDFIFOCTL3_FFEN_MAX                                           (0x0001U)

#define CSL_SDFM_SDFIFOCTL3_DRINTSEL_MASK                                      (0x4000U)
#define CSL_SDFM_SDFIFOCTL3_DRINTSEL_SHIFT                                     (0x000EU)
#define CSL_SDFM_SDFIFOCTL3_DRINTSEL_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDFIFOCTL3_DRINTSEL_MAX                                       (0x0001U)

#define CSL_SDFM_SDFIFOCTL3_OVFIEN_MASK                                        (0x8000U)
#define CSL_SDFM_SDFIFOCTL3_OVFIEN_SHIFT                                       (0x000FU)
#define CSL_SDFM_SDFIFOCTL3_OVFIEN_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDFIFOCTL3_OVFIEN_MAX                                         (0x0001U)

#define CSL_SDFM_SDFIFOCTL3_RESETVAL                                           (0x0000U)

/* SDSYNC3 */

#define CSL_SDFM_SDSYNC3_SYNCSEL_MASK                                          (0x003FU)
#define CSL_SDFM_SDSYNC3_SYNCSEL_SHIFT                                         (0x0000U)
#define CSL_SDFM_SDSYNC3_SYNCSEL_RESETVAL                                      (0x0000U)
#define CSL_SDFM_SDSYNC3_SYNCSEL_MAX                                           (0x003FU)

#define CSL_SDFM_SDSYNC3_WTSYNCEN_MASK                                         (0x0040U)
#define CSL_SDFM_SDSYNC3_WTSYNCEN_SHIFT                                        (0x0006U)
#define CSL_SDFM_SDSYNC3_WTSYNCEN_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDSYNC3_WTSYNCEN_MAX                                          (0x0001U)

#define CSL_SDFM_SDSYNC3_WTSYNFLG_MASK                                         (0x0080U)
#define CSL_SDFM_SDSYNC3_WTSYNFLG_SHIFT                                        (0x0007U)
#define CSL_SDFM_SDSYNC3_WTSYNFLG_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDSYNC3_WTSYNFLG_MAX                                          (0x0001U)

#define CSL_SDFM_SDSYNC3_WTSYNCLR_MASK                                         (0x0100U)
#define CSL_SDFM_SDSYNC3_WTSYNCLR_SHIFT                                        (0x0008U)
#define CSL_SDFM_SDSYNC3_WTSYNCLR_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDSYNC3_WTSYNCLR_MAX                                          (0x0001U)

#define CSL_SDFM_SDSYNC3_FFSYNCCLREN_MASK                                      (0x0200U)
#define CSL_SDFM_SDSYNC3_FFSYNCCLREN_SHIFT                                     (0x0009U)
#define CSL_SDFM_SDSYNC3_FFSYNCCLREN_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDSYNC3_FFSYNCCLREN_MAX                                       (0x0001U)

#define CSL_SDFM_SDSYNC3_WTSCLREN_MASK                                         (0x0400U)
#define CSL_SDFM_SDSYNC3_WTSCLREN_SHIFT                                        (0x000AU)
#define CSL_SDFM_SDSYNC3_WTSCLREN_RESETVAL                                     (0x0001U)
#define CSL_SDFM_SDSYNC3_WTSCLREN_MAX                                          (0x0001U)

#define CSL_SDFM_SDSYNC3_RESERVED_1_MASK                                       (0xF800U)
#define CSL_SDFM_SDSYNC3_RESERVED_1_SHIFT                                      (0x000BU)
#define CSL_SDFM_SDSYNC3_RESERVED_1_RESETVAL                                   (0x0000U)
#define CSL_SDFM_SDSYNC3_RESERVED_1_MAX                                        (0x001FU)

#define CSL_SDFM_SDSYNC3_RESETVAL                                              (0x0400U)

/* SDFLT3CMPL2 */

#define CSL_SDFM_SDFLT3CMPL2_LLT2_MASK                                         (0x7FFFU)
#define CSL_SDFM_SDFLT3CMPL2_LLT2_SHIFT                                        (0x0000U)
#define CSL_SDFM_SDFLT3CMPL2_LLT2_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDFLT3CMPL2_LLT2_MAX                                          (0x7FFFU)

#define CSL_SDFM_SDFLT3CMPL2_RESERVED_1_MASK                                   (0x8000U)
#define CSL_SDFM_SDFLT3CMPL2_RESERVED_1_SHIFT                                  (0x000FU)
#define CSL_SDFM_SDFLT3CMPL2_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDFLT3CMPL2_RESERVED_1_MAX                                    (0x0001U)

#define CSL_SDFM_SDFLT3CMPL2_RESETVAL                                          (0x0000U)

/* SDCTLPARM4 */

#define CSL_SDFM_SDCTLPARM4_MOD_MASK                                           (0x0003U)
#define CSL_SDFM_SDCTLPARM4_MOD_SHIFT                                          (0x0000U)
#define CSL_SDFM_SDCTLPARM4_MOD_RESETVAL                                       (0x0000U)
#define CSL_SDFM_SDCTLPARM4_MOD_MAX                                            (0x0003U)

#define CSL_SDFM_SDCTLPARM4_RESERVED_1_MASK                                    (0x0004U)
#define CSL_SDFM_SDCTLPARM4_RESERVED_1_SHIFT                                   (0x0002U)
#define CSL_SDFM_SDCTLPARM4_RESERVED_1_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCTLPARM4_RESERVED_1_MAX                                     (0x0001U)

#define CSL_SDFM_SDCTLPARM4_SDCLKSEL_MASK                                      (0x0008U)
#define CSL_SDFM_SDCTLPARM4_SDCLKSEL_SHIFT                                     (0x0003U)
#define CSL_SDFM_SDCTLPARM4_SDCLKSEL_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDCTLPARM4_SDCLKSEL_MAX                                       (0x0001U)

#define CSL_SDFM_SDCTLPARM4_SDCLKSYNC_MASK                                     (0x0010U)
#define CSL_SDFM_SDCTLPARM4_SDCLKSYNC_SHIFT                                    (0x0004U)
#define CSL_SDFM_SDCTLPARM4_SDCLKSYNC_RESETVAL                                 (0x0000U)
#define CSL_SDFM_SDCTLPARM4_SDCLKSYNC_MAX                                      (0x0001U)

#define CSL_SDFM_SDCTLPARM4_RESERVED_2_MASK                                    (0x0020U)
#define CSL_SDFM_SDCTLPARM4_RESERVED_2_SHIFT                                   (0x0005U)
#define CSL_SDFM_SDCTLPARM4_RESERVED_2_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCTLPARM4_RESERVED_2_MAX                                     (0x0001U)

#define CSL_SDFM_SDCTLPARM4_SDDATASYNC_MASK                                    (0x0040U)
#define CSL_SDFM_SDCTLPARM4_SDDATASYNC_SHIFT                                   (0x0006U)
#define CSL_SDFM_SDCTLPARM4_SDDATASYNC_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCTLPARM4_SDDATASYNC_MAX                                     (0x0001U)

#define CSL_SDFM_SDCTLPARM4_RESERVED_3_MASK                                    (0x0080U)
#define CSL_SDFM_SDCTLPARM4_RESERVED_3_SHIFT                                   (0x0007U)
#define CSL_SDFM_SDCTLPARM4_RESERVED_3_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCTLPARM4_RESERVED_3_MAX                                     (0x0001U)

#define CSL_SDFM_SDCTLPARM4_RESERVED_4_MASK                                    (0xFF00U)
#define CSL_SDFM_SDCTLPARM4_RESERVED_4_SHIFT                                   (0x0008U)
#define CSL_SDFM_SDCTLPARM4_RESERVED_4_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCTLPARM4_RESERVED_4_MAX                                     (0x00FFU)

#define CSL_SDFM_SDCTLPARM4_RESETVAL                                           (0x0000U)

/* SDDFPARM4 */

#define CSL_SDFM_SDDFPARM4_DOSR_MASK                                           (0x00FFU)
#define CSL_SDFM_SDDFPARM4_DOSR_SHIFT                                          (0x0000U)
#define CSL_SDFM_SDDFPARM4_DOSR_RESETVAL                                       (0x0000U)
#define CSL_SDFM_SDDFPARM4_DOSR_MAX                                            (0x00FFU)

#define CSL_SDFM_SDDFPARM4_FEN_MASK                                            (0x0100U)
#define CSL_SDFM_SDDFPARM4_FEN_SHIFT                                           (0x0008U)
#define CSL_SDFM_SDDFPARM4_FEN_RESETVAL                                        (0x0000U)
#define CSL_SDFM_SDDFPARM4_FEN_MAX                                             (0x0001U)

#define CSL_SDFM_SDDFPARM4_AE_MASK                                             (0x0200U)
#define CSL_SDFM_SDDFPARM4_AE_SHIFT                                            (0x0009U)
#define CSL_SDFM_SDDFPARM4_AE_RESETVAL                                         (0x0000U)
#define CSL_SDFM_SDDFPARM4_AE_MAX                                              (0x0001U)

#define CSL_SDFM_SDDFPARM4_SST_MASK                                            (0x0C00U)
#define CSL_SDFM_SDDFPARM4_SST_SHIFT                                           (0x000AU)
#define CSL_SDFM_SDDFPARM4_SST_RESETVAL                                        (0x0000U)
#define CSL_SDFM_SDDFPARM4_SST_MAX                                             (0x0003U)

#define CSL_SDFM_SDDFPARM4_SDSYNCEN_MASK                                       (0x1000U)
#define CSL_SDFM_SDDFPARM4_SDSYNCEN_SHIFT                                      (0x000CU)
#define CSL_SDFM_SDDFPARM4_SDSYNCEN_RESETVAL                                   (0x0000U)
#define CSL_SDFM_SDDFPARM4_SDSYNCEN_MAX                                        (0x0001U)

#define CSL_SDFM_SDDFPARM4_RESERVED_1_MASK                                     (0xE000U)
#define CSL_SDFM_SDDFPARM4_RESERVED_1_SHIFT                                    (0x000DU)
#define CSL_SDFM_SDDFPARM4_RESERVED_1_RESETVAL                                 (0x0000U)
#define CSL_SDFM_SDDFPARM4_RESERVED_1_MAX                                      (0x0007U)

#define CSL_SDFM_SDDFPARM4_RESETVAL                                            (0x0000U)

/* SDDPARM4 */

#define CSL_SDFM_SDDPARM4_RESERVED_1_MASK                                      (0x03FFU)
#define CSL_SDFM_SDDPARM4_RESERVED_1_SHIFT                                     (0x0000U)
#define CSL_SDFM_SDDPARM4_RESERVED_1_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDDPARM4_RESERVED_1_MAX                                       (0x03FFU)

#define CSL_SDFM_SDDPARM4_DR_MASK                                              (0x0400U)
#define CSL_SDFM_SDDPARM4_DR_SHIFT                                             (0x000AU)
#define CSL_SDFM_SDDPARM4_DR_RESETVAL                                          (0x0000U)
#define CSL_SDFM_SDDPARM4_DR_MAX                                               (0x0001U)

#define CSL_SDFM_SDDPARM4_SH_MASK                                              (0xF800U)
#define CSL_SDFM_SDDPARM4_SH_SHIFT                                             (0x000BU)
#define CSL_SDFM_SDDPARM4_SH_RESETVAL                                          (0x0000U)
#define CSL_SDFM_SDDPARM4_SH_MAX                                               (0x001FU)

#define CSL_SDFM_SDDPARM4_RESETVAL                                             (0x0000U)

/* SDFLT4CMPH1 */

#define CSL_SDFM_SDFLT4CMPH1_HLT_MASK                                          (0x7FFFU)
#define CSL_SDFM_SDFLT4CMPH1_HLT_SHIFT                                         (0x0000U)
#define CSL_SDFM_SDFLT4CMPH1_HLT_RESETVAL                                      (0x7FFFU)
#define CSL_SDFM_SDFLT4CMPH1_HLT_MAX                                           (0x7FFFU)

#define CSL_SDFM_SDFLT4CMPH1_RESERVED_1_MASK                                   (0x8000U)
#define CSL_SDFM_SDFLT4CMPH1_RESERVED_1_SHIFT                                  (0x000FU)
#define CSL_SDFM_SDFLT4CMPH1_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDFLT4CMPH1_RESERVED_1_MAX                                    (0x0001U)

#define CSL_SDFM_SDFLT4CMPH1_RESETVAL                                          (0x7FFFU)

/* SDFLT4CMPL1 */

#define CSL_SDFM_SDFLT4CMPL1_LLT_MASK                                          (0x7FFFU)
#define CSL_SDFM_SDFLT4CMPL1_LLT_SHIFT                                         (0x0000U)
#define CSL_SDFM_SDFLT4CMPL1_LLT_RESETVAL                                      (0x0000U)
#define CSL_SDFM_SDFLT4CMPL1_LLT_MAX                                           (0x7FFFU)

#define CSL_SDFM_SDFLT4CMPL1_RESERVED_1_MASK                                   (0x8000U)
#define CSL_SDFM_SDFLT4CMPL1_RESERVED_1_SHIFT                                  (0x000FU)
#define CSL_SDFM_SDFLT4CMPL1_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDFLT4CMPL1_RESERVED_1_MAX                                    (0x0001U)

#define CSL_SDFM_SDFLT4CMPL1_RESETVAL                                          (0x0000U)

/* SDCPARM4 */

#define CSL_SDFM_SDCPARM4_COSR_MASK                                            (0x001FU)
#define CSL_SDFM_SDCPARM4_COSR_SHIFT                                           (0x0000U)
#define CSL_SDFM_SDCPARM4_COSR_RESETVAL                                        (0x0000U)
#define CSL_SDFM_SDCPARM4_COSR_MAX                                             (0x001FU)

#define CSL_SDFM_SDCPARM4_EN_CEVT1_MASK                                        (0x0020U)
#define CSL_SDFM_SDCPARM4_EN_CEVT1_SHIFT                                       (0x0005U)
#define CSL_SDFM_SDCPARM4_EN_CEVT1_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDCPARM4_EN_CEVT1_MAX                                         (0x0001U)

#define CSL_SDFM_SDCPARM4_EN_CEVT2_MASK                                        (0x0040U)
#define CSL_SDFM_SDCPARM4_EN_CEVT2_SHIFT                                       (0x0006U)
#define CSL_SDFM_SDCPARM4_EN_CEVT2_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDCPARM4_EN_CEVT2_MAX                                         (0x0001U)

#define CSL_SDFM_SDCPARM4_CS1_CS0_MASK                                         (0x0180U)
#define CSL_SDFM_SDCPARM4_CS1_CS0_SHIFT                                        (0x0007U)
#define CSL_SDFM_SDCPARM4_CS1_CS0_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDCPARM4_CS1_CS0_MAX                                          (0x0003U)

#define CSL_SDFM_SDCPARM4_MFIE_MASK                                            (0x0200U)
#define CSL_SDFM_SDCPARM4_MFIE_SHIFT                                           (0x0009U)
#define CSL_SDFM_SDCPARM4_MFIE_RESETVAL                                        (0x0000U)
#define CSL_SDFM_SDCPARM4_MFIE_MAX                                             (0x0001U)

#define CSL_SDFM_SDCPARM4_HZEN_MASK                                            (0x0400U)
#define CSL_SDFM_SDCPARM4_HZEN_SHIFT                                           (0x000AU)
#define CSL_SDFM_SDCPARM4_HZEN_RESETVAL                                        (0x0000U)
#define CSL_SDFM_SDCPARM4_HZEN_MAX                                             (0x0001U)

#define CSL_SDFM_SDCPARM4_CEVT1SEL_MASK                                        (0x1800U)
#define CSL_SDFM_SDCPARM4_CEVT1SEL_SHIFT                                       (0x000BU)
#define CSL_SDFM_SDCPARM4_CEVT1SEL_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDCPARM4_CEVT1SEL_MAX                                         (0x0003U)

#define CSL_SDFM_SDCPARM4_CEN_MASK                                             (0x2000U)
#define CSL_SDFM_SDCPARM4_CEN_SHIFT                                            (0x000DU)
#define CSL_SDFM_SDCPARM4_CEN_RESETVAL                                         (0x0000U)
#define CSL_SDFM_SDCPARM4_CEN_MAX                                              (0x0001U)

#define CSL_SDFM_SDCPARM4_CEVT2SEL_MASK                                        (0xC000U)
#define CSL_SDFM_SDCPARM4_CEVT2SEL_SHIFT                                       (0x000EU)
#define CSL_SDFM_SDCPARM4_CEVT2SEL_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDCPARM4_CEVT2SEL_MAX                                         (0x0003U)

#define CSL_SDFM_SDCPARM4_RESETVAL                                             (0x0000U)

/* SDDATA4 */

#define CSL_SDFM_SDDATA4_DATA16_MASK                                           (0x0000FFFFU)
#define CSL_SDFM_SDDATA4_DATA16_SHIFT                                          (0x00000000U)
#define CSL_SDFM_SDDATA4_DATA16_RESETVAL                                       (0x00000000U)
#define CSL_SDFM_SDDATA4_DATA16_MAX                                            (0x0000FFFFU)

#define CSL_SDFM_SDDATA4_DATA32HI_MASK                                         (0xFFFF0000U)
#define CSL_SDFM_SDDATA4_DATA32HI_SHIFT                                        (0x00000010U)
#define CSL_SDFM_SDDATA4_DATA32HI_RESETVAL                                     (0x00000000U)
#define CSL_SDFM_SDDATA4_DATA32HI_MAX                                          (0x0000FFFFU)

#define CSL_SDFM_SDDATA4_RESETVAL                                              (0x00000000U)

/* SDDATFIFO4 */

#define CSL_SDFM_SDDATFIFO4_DATA16_MASK                                        (0x0000FFFFU)
#define CSL_SDFM_SDDATFIFO4_DATA16_SHIFT                                       (0x00000000U)
#define CSL_SDFM_SDDATFIFO4_DATA16_RESETVAL                                    (0x00000000U)
#define CSL_SDFM_SDDATFIFO4_DATA16_MAX                                         (0x0000FFFFU)

#define CSL_SDFM_SDDATFIFO4_DATA32HI_MASK                                      (0xFFFF0000U)
#define CSL_SDFM_SDDATFIFO4_DATA32HI_SHIFT                                     (0x00000010U)
#define CSL_SDFM_SDDATFIFO4_DATA32HI_RESETVAL                                  (0x00000000U)
#define CSL_SDFM_SDDATFIFO4_DATA32HI_MAX                                       (0x0000FFFFU)

#define CSL_SDFM_SDDATFIFO4_RESETVAL                                           (0x00000000U)

/* SDCDATA4 */

#define CSL_SDFM_SDCDATA4_DATA16_MASK                                          (0xFFFFU)
#define CSL_SDFM_SDCDATA4_DATA16_SHIFT                                         (0x0000U)
#define CSL_SDFM_SDCDATA4_DATA16_RESETVAL                                      (0x0000U)
#define CSL_SDFM_SDCDATA4_DATA16_MAX                                           (0xFFFFU)

#define CSL_SDFM_SDCDATA4_RESETVAL                                             (0x0000U)

/* SDFLT4CMPH2 */

#define CSL_SDFM_SDFLT4CMPH2_HLT2_MASK                                         (0x7FFFU)
#define CSL_SDFM_SDFLT4CMPH2_HLT2_SHIFT                                        (0x0000U)
#define CSL_SDFM_SDFLT4CMPH2_HLT2_RESETVAL                                     (0x7FFFU)
#define CSL_SDFM_SDFLT4CMPH2_HLT2_MAX                                          (0x7FFFU)

#define CSL_SDFM_SDFLT4CMPH2_RESERVED_1_MASK                                   (0x8000U)
#define CSL_SDFM_SDFLT4CMPH2_RESERVED_1_SHIFT                                  (0x000FU)
#define CSL_SDFM_SDFLT4CMPH2_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDFLT4CMPH2_RESERVED_1_MAX                                    (0x0001U)

#define CSL_SDFM_SDFLT4CMPH2_RESETVAL                                          (0x7FFFU)

/* SDFLT4CMPHZ */

#define CSL_SDFM_SDFLT4CMPHZ_HLTZ_MASK                                         (0x7FFFU)
#define CSL_SDFM_SDFLT4CMPHZ_HLTZ_SHIFT                                        (0x0000U)
#define CSL_SDFM_SDFLT4CMPHZ_HLTZ_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDFLT4CMPHZ_HLTZ_MAX                                          (0x7FFFU)

#define CSL_SDFM_SDFLT4CMPHZ_RESERVED_1_MASK                                   (0x8000U)
#define CSL_SDFM_SDFLT4CMPHZ_RESERVED_1_SHIFT                                  (0x000FU)
#define CSL_SDFM_SDFLT4CMPHZ_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDFLT4CMPHZ_RESERVED_1_MAX                                    (0x0001U)

#define CSL_SDFM_SDFLT4CMPHZ_RESETVAL                                          (0x0000U)

/* SDFIFOCTL4 */

#define CSL_SDFM_SDFIFOCTL4_SDFFIL_MASK                                        (0x001FU)
#define CSL_SDFM_SDFIFOCTL4_SDFFIL_SHIFT                                       (0x0000U)
#define CSL_SDFM_SDFIFOCTL4_SDFFIL_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDFIFOCTL4_SDFFIL_MAX                                         (0x001FU)

#define CSL_SDFM_SDFIFOCTL4_RESERVED_1_MASK                                    (0x0020U)
#define CSL_SDFM_SDFIFOCTL4_RESERVED_1_SHIFT                                   (0x0005U)
#define CSL_SDFM_SDFIFOCTL4_RESERVED_1_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDFIFOCTL4_RESERVED_1_MAX                                     (0x0001U)

#define CSL_SDFM_SDFIFOCTL4_SDFFST_MASK                                        (0x07C0U)
#define CSL_SDFM_SDFIFOCTL4_SDFFST_SHIFT                                       (0x0006U)
#define CSL_SDFM_SDFIFOCTL4_SDFFST_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDFIFOCTL4_SDFFST_MAX                                         (0x001FU)

#define CSL_SDFM_SDFIFOCTL4_RESERVED_2_MASK                                    (0x0800U)
#define CSL_SDFM_SDFIFOCTL4_RESERVED_2_SHIFT                                   (0x000BU)
#define CSL_SDFM_SDFIFOCTL4_RESERVED_2_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDFIFOCTL4_RESERVED_2_MAX                                     (0x0001U)

#define CSL_SDFM_SDFIFOCTL4_FFIEN_MASK                                         (0x1000U)
#define CSL_SDFM_SDFIFOCTL4_FFIEN_SHIFT                                        (0x000CU)
#define CSL_SDFM_SDFIFOCTL4_FFIEN_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDFIFOCTL4_FFIEN_MAX                                          (0x0001U)

#define CSL_SDFM_SDFIFOCTL4_FFEN_MASK                                          (0x2000U)
#define CSL_SDFM_SDFIFOCTL4_FFEN_SHIFT                                         (0x000DU)
#define CSL_SDFM_SDFIFOCTL4_FFEN_RESETVAL                                      (0x0000U)
#define CSL_SDFM_SDFIFOCTL4_FFEN_MAX                                           (0x0001U)

#define CSL_SDFM_SDFIFOCTL4_DRINTSEL_MASK                                      (0x4000U)
#define CSL_SDFM_SDFIFOCTL4_DRINTSEL_SHIFT                                     (0x000EU)
#define CSL_SDFM_SDFIFOCTL4_DRINTSEL_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDFIFOCTL4_DRINTSEL_MAX                                       (0x0001U)

#define CSL_SDFM_SDFIFOCTL4_OVFIEN_MASK                                        (0x8000U)
#define CSL_SDFM_SDFIFOCTL4_OVFIEN_SHIFT                                       (0x000FU)
#define CSL_SDFM_SDFIFOCTL4_OVFIEN_RESETVAL                                    (0x0000U)
#define CSL_SDFM_SDFIFOCTL4_OVFIEN_MAX                                         (0x0001U)

#define CSL_SDFM_SDFIFOCTL4_RESETVAL                                           (0x0000U)

/* SDSYNC4 */

#define CSL_SDFM_SDSYNC4_SYNCSEL_MASK                                          (0x003FU)
#define CSL_SDFM_SDSYNC4_SYNCSEL_SHIFT                                         (0x0000U)
#define CSL_SDFM_SDSYNC4_SYNCSEL_RESETVAL                                      (0x0000U)
#define CSL_SDFM_SDSYNC4_SYNCSEL_MAX                                           (0x003FU)

#define CSL_SDFM_SDSYNC4_WTSYNCEN_MASK                                         (0x0040U)
#define CSL_SDFM_SDSYNC4_WTSYNCEN_SHIFT                                        (0x0006U)
#define CSL_SDFM_SDSYNC4_WTSYNCEN_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDSYNC4_WTSYNCEN_MAX                                          (0x0001U)

#define CSL_SDFM_SDSYNC4_WTSYNFLG_MASK                                         (0x0080U)
#define CSL_SDFM_SDSYNC4_WTSYNFLG_SHIFT                                        (0x0007U)
#define CSL_SDFM_SDSYNC4_WTSYNFLG_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDSYNC4_WTSYNFLG_MAX                                          (0x0001U)

#define CSL_SDFM_SDSYNC4_WTSYNCLR_MASK                                         (0x0100U)
#define CSL_SDFM_SDSYNC4_WTSYNCLR_SHIFT                                        (0x0008U)
#define CSL_SDFM_SDSYNC4_WTSYNCLR_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDSYNC4_WTSYNCLR_MAX                                          (0x0001U)

#define CSL_SDFM_SDSYNC4_FFSYNCCLREN_MASK                                      (0x0200U)
#define CSL_SDFM_SDSYNC4_FFSYNCCLREN_SHIFT                                     (0x0009U)
#define CSL_SDFM_SDSYNC4_FFSYNCCLREN_RESETVAL                                  (0x0000U)
#define CSL_SDFM_SDSYNC4_FFSYNCCLREN_MAX                                       (0x0001U)

#define CSL_SDFM_SDSYNC4_WTSCLREN_MASK                                         (0x0400U)
#define CSL_SDFM_SDSYNC4_WTSCLREN_SHIFT                                        (0x000AU)
#define CSL_SDFM_SDSYNC4_WTSCLREN_RESETVAL                                     (0x0001U)
#define CSL_SDFM_SDSYNC4_WTSCLREN_MAX                                          (0x0001U)

#define CSL_SDFM_SDSYNC4_RESERVED_1_MASK                                       (0xF800U)
#define CSL_SDFM_SDSYNC4_RESERVED_1_SHIFT                                      (0x000BU)
#define CSL_SDFM_SDSYNC4_RESERVED_1_RESETVAL                                   (0x0000U)
#define CSL_SDFM_SDSYNC4_RESERVED_1_MAX                                        (0x001FU)

#define CSL_SDFM_SDSYNC4_RESETVAL                                              (0x0400U)

/* SDFLT4CMPL2 */

#define CSL_SDFM_SDFLT4CMPL2_LLT2_MASK                                         (0x7FFFU)
#define CSL_SDFM_SDFLT4CMPL2_LLT2_SHIFT                                        (0x0000U)
#define CSL_SDFM_SDFLT4CMPL2_LLT2_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDFLT4CMPL2_LLT2_MAX                                          (0x7FFFU)

#define CSL_SDFM_SDFLT4CMPL2_RESERVED_1_MASK                                   (0x8000U)
#define CSL_SDFM_SDFLT4CMPL2_RESERVED_1_SHIFT                                  (0x000FU)
#define CSL_SDFM_SDFLT4CMPL2_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDFLT4CMPL2_RESERVED_1_MAX                                    (0x0001U)

#define CSL_SDFM_SDFLT4CMPL2_RESETVAL                                          (0x0000U)

/* SDCOMP1CTL */

#define CSL_SDFM_SDCOMP1CTL_RESERVED_1_MASK                                    (0x0001U)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_1_SHIFT                                   (0x0000U)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_1_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_1_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP1CTL_RESERVED_2_MASK                                    (0x0002U)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_2_SHIFT                                   (0x0001U)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_2_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_2_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP1CTL_CEVT1DIGFILTSEL_MASK                               (0x000CU)
#define CSL_SDFM_SDCOMP1CTL_CEVT1DIGFILTSEL_SHIFT                              (0x0002U)
#define CSL_SDFM_SDCOMP1CTL_CEVT1DIGFILTSEL_RESETVAL                           (0x0000U)
#define CSL_SDFM_SDCOMP1CTL_CEVT1DIGFILTSEL_MAX                                (0x0003U)

#define CSL_SDFM_SDCOMP1CTL_RESERVED_3_MASK                                    (0x0030U)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_3_SHIFT                                   (0x0004U)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_3_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_3_MAX                                     (0x0003U)

#define CSL_SDFM_SDCOMP1CTL_RESERVED_4_MASK                                    (0x0040U)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_4_SHIFT                                   (0x0006U)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_4_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_4_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP1CTL_RESERVED_5_MASK                                    (0x0080U)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_5_SHIFT                                   (0x0007U)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_5_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_5_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP1CTL_RESERVED_6_MASK                                    (0x0100U)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_6_SHIFT                                   (0x0008U)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_6_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_6_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP1CTL_RESERVED_7_MASK                                    (0x0200U)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_7_SHIFT                                   (0x0009U)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_7_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_7_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP1CTL_CEVT2DIGFILTSEL_MASK                               (0x0C00U)
#define CSL_SDFM_SDCOMP1CTL_CEVT2DIGFILTSEL_SHIFT                              (0x000AU)
#define CSL_SDFM_SDCOMP1CTL_CEVT2DIGFILTSEL_RESETVAL                           (0x0000U)
#define CSL_SDFM_SDCOMP1CTL_CEVT2DIGFILTSEL_MAX                                (0x0003U)

#define CSL_SDFM_SDCOMP1CTL_RESERVED_8_MASK                                    (0x3000U)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_8_SHIFT                                   (0x000CU)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_8_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_8_MAX                                     (0x0003U)

#define CSL_SDFM_SDCOMP1CTL_RESERVED_9_MASK                                    (0x4000U)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_9_SHIFT                                   (0x000EU)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_9_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_9_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP1CTL_RESERVED_10_MASK                                   (0x8000U)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_10_SHIFT                                  (0x000FU)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_10_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDCOMP1CTL_RESERVED_10_MAX                                    (0x0001U)

#define CSL_SDFM_SDCOMP1CTL_RESETVAL                                           (0x0000U)

/* SDCOMP1EVT2FLTCTL */

#define CSL_SDFM_SDCOMP1EVT2FLTCTL_RESERVED_1_MASK                             (0x000FU)
#define CSL_SDFM_SDCOMP1EVT2FLTCTL_RESERVED_1_SHIFT                            (0x0000U)
#define CSL_SDFM_SDCOMP1EVT2FLTCTL_RESERVED_1_RESETVAL                         (0x0000U)
#define CSL_SDFM_SDCOMP1EVT2FLTCTL_RESERVED_1_MAX                              (0x000FU)

#define CSL_SDFM_SDCOMP1EVT2FLTCTL_SAMPWIN_MASK                                (0x01F0U)
#define CSL_SDFM_SDCOMP1EVT2FLTCTL_SAMPWIN_SHIFT                               (0x0004U)
#define CSL_SDFM_SDCOMP1EVT2FLTCTL_SAMPWIN_RESETVAL                            (0x0000U)
#define CSL_SDFM_SDCOMP1EVT2FLTCTL_SAMPWIN_MAX                                 (0x001FU)

#define CSL_SDFM_SDCOMP1EVT2FLTCTL_THRESH_MASK                                 (0x3E00U)
#define CSL_SDFM_SDCOMP1EVT2FLTCTL_THRESH_SHIFT                                (0x0009U)
#define CSL_SDFM_SDCOMP1EVT2FLTCTL_THRESH_RESETVAL                             (0x0000U)
#define CSL_SDFM_SDCOMP1EVT2FLTCTL_THRESH_MAX                                  (0x001FU)

#define CSL_SDFM_SDCOMP1EVT2FLTCTL_RESERVED_2_MASK                             (0x4000U)
#define CSL_SDFM_SDCOMP1EVT2FLTCTL_RESERVED_2_SHIFT                            (0x000EU)
#define CSL_SDFM_SDCOMP1EVT2FLTCTL_RESERVED_2_RESETVAL                         (0x0000U)
#define CSL_SDFM_SDCOMP1EVT2FLTCTL_RESERVED_2_MAX                              (0x0001U)

#define CSL_SDFM_SDCOMP1EVT2FLTCTL_FILINIT_MASK                                (0x8000U)
#define CSL_SDFM_SDCOMP1EVT2FLTCTL_FILINIT_SHIFT                               (0x000FU)
#define CSL_SDFM_SDCOMP1EVT2FLTCTL_FILINIT_RESETVAL                            (0x0000U)
#define CSL_SDFM_SDCOMP1EVT2FLTCTL_FILINIT_MAX                                 (0x0001U)

#define CSL_SDFM_SDCOMP1EVT2FLTCTL_RESETVAL                                    (0x0000U)

/* SDCOMP1EVT2FLTCLKCTL */

#define CSL_SDFM_SDCOMP1EVT2FLTCLKCTL_CLKPRESCALE_MASK                         (0x03FFU)
#define CSL_SDFM_SDCOMP1EVT2FLTCLKCTL_CLKPRESCALE_SHIFT                        (0x0000U)
#define CSL_SDFM_SDCOMP1EVT2FLTCLKCTL_CLKPRESCALE_RESETVAL                     (0x0000U)
#define CSL_SDFM_SDCOMP1EVT2FLTCLKCTL_CLKPRESCALE_MAX                          (0x03FFU)

#define CSL_SDFM_SDCOMP1EVT2FLTCLKCTL_RESERVED_1_MASK                          (0xFC00U)
#define CSL_SDFM_SDCOMP1EVT2FLTCLKCTL_RESERVED_1_SHIFT                         (0x000AU)
#define CSL_SDFM_SDCOMP1EVT2FLTCLKCTL_RESERVED_1_RESETVAL                      (0x0000U)
#define CSL_SDFM_SDCOMP1EVT2FLTCLKCTL_RESERVED_1_MAX                           (0x003FU)

#define CSL_SDFM_SDCOMP1EVT2FLTCLKCTL_RESETVAL                                 (0x0000U)

/* SDCOMP1EVT1FLTCTL */

#define CSL_SDFM_SDCOMP1EVT1FLTCTL_RESERVED_1_MASK                             (0x000FU)
#define CSL_SDFM_SDCOMP1EVT1FLTCTL_RESERVED_1_SHIFT                            (0x0000U)
#define CSL_SDFM_SDCOMP1EVT1FLTCTL_RESERVED_1_RESETVAL                         (0x0000U)
#define CSL_SDFM_SDCOMP1EVT1FLTCTL_RESERVED_1_MAX                              (0x000FU)

#define CSL_SDFM_SDCOMP1EVT1FLTCTL_SAMPWIN_MASK                                (0x01F0U)
#define CSL_SDFM_SDCOMP1EVT1FLTCTL_SAMPWIN_SHIFT                               (0x0004U)
#define CSL_SDFM_SDCOMP1EVT1FLTCTL_SAMPWIN_RESETVAL                            (0x0000U)
#define CSL_SDFM_SDCOMP1EVT1FLTCTL_SAMPWIN_MAX                                 (0x001FU)

#define CSL_SDFM_SDCOMP1EVT1FLTCTL_THRESH_MASK                                 (0x3E00U)
#define CSL_SDFM_SDCOMP1EVT1FLTCTL_THRESH_SHIFT                                (0x0009U)
#define CSL_SDFM_SDCOMP1EVT1FLTCTL_THRESH_RESETVAL                             (0x0000U)
#define CSL_SDFM_SDCOMP1EVT1FLTCTL_THRESH_MAX                                  (0x001FU)

#define CSL_SDFM_SDCOMP1EVT1FLTCTL_RESERVED_2_MASK                             (0x4000U)
#define CSL_SDFM_SDCOMP1EVT1FLTCTL_RESERVED_2_SHIFT                            (0x000EU)
#define CSL_SDFM_SDCOMP1EVT1FLTCTL_RESERVED_2_RESETVAL                         (0x0000U)
#define CSL_SDFM_SDCOMP1EVT1FLTCTL_RESERVED_2_MAX                              (0x0001U)

#define CSL_SDFM_SDCOMP1EVT1FLTCTL_FILINIT_MASK                                (0x8000U)
#define CSL_SDFM_SDCOMP1EVT1FLTCTL_FILINIT_SHIFT                               (0x000FU)
#define CSL_SDFM_SDCOMP1EVT1FLTCTL_FILINIT_RESETVAL                            (0x0000U)
#define CSL_SDFM_SDCOMP1EVT1FLTCTL_FILINIT_MAX                                 (0x0001U)

#define CSL_SDFM_SDCOMP1EVT1FLTCTL_RESETVAL                                    (0x0000U)

/* SDCOMP1EVT1FLTCLKCTL */

#define CSL_SDFM_SDCOMP1EVT1FLTCLKCTL_CLKPRESCALE_MASK                         (0x03FFU)
#define CSL_SDFM_SDCOMP1EVT1FLTCLKCTL_CLKPRESCALE_SHIFT                        (0x0000U)
#define CSL_SDFM_SDCOMP1EVT1FLTCLKCTL_CLKPRESCALE_RESETVAL                     (0x0000U)
#define CSL_SDFM_SDCOMP1EVT1FLTCLKCTL_CLKPRESCALE_MAX                          (0x03FFU)

#define CSL_SDFM_SDCOMP1EVT1FLTCLKCTL_RESERVED_1_MASK                          (0xFC00U)
#define CSL_SDFM_SDCOMP1EVT1FLTCLKCTL_RESERVED_1_SHIFT                         (0x000AU)
#define CSL_SDFM_SDCOMP1EVT1FLTCLKCTL_RESERVED_1_RESETVAL                      (0x0000U)
#define CSL_SDFM_SDCOMP1EVT1FLTCLKCTL_RESERVED_1_MAX                           (0x003FU)

#define CSL_SDFM_SDCOMP1EVT1FLTCLKCTL_RESETVAL                                 (0x0000U)

/* SDCOMP1LOCK */

#define CSL_SDFM_SDCOMP1LOCK_SDCOMP1CTL_MASK                                   (0x0001U)
#define CSL_SDFM_SDCOMP1LOCK_SDCOMP1CTL_SHIFT                                  (0x0000U)
#define CSL_SDFM_SDCOMP1LOCK_SDCOMP1CTL_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDCOMP1LOCK_SDCOMP1CTL_MAX                                    (0x0001U)

#define CSL_SDFM_SDCOMP1LOCK_RESERVED_1_MASK                                   (0x0002U)
#define CSL_SDFM_SDCOMP1LOCK_RESERVED_1_SHIFT                                  (0x0001U)
#define CSL_SDFM_SDCOMP1LOCK_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDCOMP1LOCK_RESERVED_1_MAX                                    (0x0001U)

#define CSL_SDFM_SDCOMP1LOCK_RESERVED_2_MASK                                   (0x0004U)
#define CSL_SDFM_SDCOMP1LOCK_RESERVED_2_SHIFT                                  (0x0002U)
#define CSL_SDFM_SDCOMP1LOCK_RESERVED_2_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDCOMP1LOCK_RESERVED_2_MAX                                    (0x0001U)

#define CSL_SDFM_SDCOMP1LOCK_COMP_MASK                                         (0x0008U)
#define CSL_SDFM_SDCOMP1LOCK_COMP_SHIFT                                        (0x0003U)
#define CSL_SDFM_SDCOMP1LOCK_COMP_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDCOMP1LOCK_COMP_MAX                                          (0x0001U)

#define CSL_SDFM_SDCOMP1LOCK_RESERVED_3_MASK                                   (0x0010U)
#define CSL_SDFM_SDCOMP1LOCK_RESERVED_3_SHIFT                                  (0x0004U)
#define CSL_SDFM_SDCOMP1LOCK_RESERVED_3_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDCOMP1LOCK_RESERVED_3_MAX                                    (0x0001U)

#define CSL_SDFM_SDCOMP1LOCK_RESERVED_4_MASK                                   (0xFFE0U)
#define CSL_SDFM_SDCOMP1LOCK_RESERVED_4_SHIFT                                  (0x0005U)
#define CSL_SDFM_SDCOMP1LOCK_RESERVED_4_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDCOMP1LOCK_RESERVED_4_MAX                                    (0x07FFU)

#define CSL_SDFM_SDCOMP1LOCK_RESETVAL                                          (0x0000U)

/* SDCOMP2CTL */

#define CSL_SDFM_SDCOMP2CTL_RESERVED_1_MASK                                    (0x0001U)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_1_SHIFT                                   (0x0000U)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_1_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_1_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP2CTL_RESERVED_2_MASK                                    (0x0002U)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_2_SHIFT                                   (0x0001U)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_2_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_2_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP2CTL_CEVT1DIGFILTSEL_MASK                               (0x000CU)
#define CSL_SDFM_SDCOMP2CTL_CEVT1DIGFILTSEL_SHIFT                              (0x0002U)
#define CSL_SDFM_SDCOMP2CTL_CEVT1DIGFILTSEL_RESETVAL                           (0x0000U)
#define CSL_SDFM_SDCOMP2CTL_CEVT1DIGFILTSEL_MAX                                (0x0003U)

#define CSL_SDFM_SDCOMP2CTL_RESERVED_3_MASK                                    (0x0030U)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_3_SHIFT                                   (0x0004U)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_3_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_3_MAX                                     (0x0003U)

#define CSL_SDFM_SDCOMP2CTL_RESERVED_4_MASK                                    (0x0040U)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_4_SHIFT                                   (0x0006U)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_4_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_4_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP2CTL_RESERVED_5_MASK                                    (0x0080U)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_5_SHIFT                                   (0x0007U)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_5_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_5_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP2CTL_RESERVED_6_MASK                                    (0x0100U)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_6_SHIFT                                   (0x0008U)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_6_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_6_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP2CTL_RESERVED_7_MASK                                    (0x0200U)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_7_SHIFT                                   (0x0009U)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_7_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_7_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP2CTL_CEVT2DIGFILTSEL_MASK                               (0x0C00U)
#define CSL_SDFM_SDCOMP2CTL_CEVT2DIGFILTSEL_SHIFT                              (0x000AU)
#define CSL_SDFM_SDCOMP2CTL_CEVT2DIGFILTSEL_RESETVAL                           (0x0000U)
#define CSL_SDFM_SDCOMP2CTL_CEVT2DIGFILTSEL_MAX                                (0x0003U)

#define CSL_SDFM_SDCOMP2CTL_RESERVED_8_MASK                                    (0x3000U)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_8_SHIFT                                   (0x000CU)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_8_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_8_MAX                                     (0x0003U)

#define CSL_SDFM_SDCOMP2CTL_RESERVED_9_MASK                                    (0x4000U)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_9_SHIFT                                   (0x000EU)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_9_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_9_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP2CTL_RESERVED_10_MASK                                   (0x8000U)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_10_SHIFT                                  (0x000FU)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_10_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDCOMP2CTL_RESERVED_10_MAX                                    (0x0001U)

#define CSL_SDFM_SDCOMP2CTL_RESETVAL                                           (0x0000U)

/* SDCOMP2EVT2FLTCTL */

#define CSL_SDFM_SDCOMP2EVT2FLTCTL_RESERVED_1_MASK                             (0x000FU)
#define CSL_SDFM_SDCOMP2EVT2FLTCTL_RESERVED_1_SHIFT                            (0x0000U)
#define CSL_SDFM_SDCOMP2EVT2FLTCTL_RESERVED_1_RESETVAL                         (0x0000U)
#define CSL_SDFM_SDCOMP2EVT2FLTCTL_RESERVED_1_MAX                              (0x000FU)

#define CSL_SDFM_SDCOMP2EVT2FLTCTL_SAMPWIN_MASK                                (0x01F0U)
#define CSL_SDFM_SDCOMP2EVT2FLTCTL_SAMPWIN_SHIFT                               (0x0004U)
#define CSL_SDFM_SDCOMP2EVT2FLTCTL_SAMPWIN_RESETVAL                            (0x0000U)
#define CSL_SDFM_SDCOMP2EVT2FLTCTL_SAMPWIN_MAX                                 (0x001FU)

#define CSL_SDFM_SDCOMP2EVT2FLTCTL_THRESH_MASK                                 (0x3E00U)
#define CSL_SDFM_SDCOMP2EVT2FLTCTL_THRESH_SHIFT                                (0x0009U)
#define CSL_SDFM_SDCOMP2EVT2FLTCTL_THRESH_RESETVAL                             (0x0000U)
#define CSL_SDFM_SDCOMP2EVT2FLTCTL_THRESH_MAX                                  (0x001FU)

#define CSL_SDFM_SDCOMP2EVT2FLTCTL_RESERVED_2_MASK                             (0x4000U)
#define CSL_SDFM_SDCOMP2EVT2FLTCTL_RESERVED_2_SHIFT                            (0x000EU)
#define CSL_SDFM_SDCOMP2EVT2FLTCTL_RESERVED_2_RESETVAL                         (0x0000U)
#define CSL_SDFM_SDCOMP2EVT2FLTCTL_RESERVED_2_MAX                              (0x0001U)

#define CSL_SDFM_SDCOMP2EVT2FLTCTL_FILINIT_MASK                                (0x8000U)
#define CSL_SDFM_SDCOMP2EVT2FLTCTL_FILINIT_SHIFT                               (0x000FU)
#define CSL_SDFM_SDCOMP2EVT2FLTCTL_FILINIT_RESETVAL                            (0x0000U)
#define CSL_SDFM_SDCOMP2EVT2FLTCTL_FILINIT_MAX                                 (0x0001U)

#define CSL_SDFM_SDCOMP2EVT2FLTCTL_RESETVAL                                    (0x0000U)

/* SDCOMP2EVT2FLTCLKCTL */

#define CSL_SDFM_SDCOMP2EVT2FLTCLKCTL_CLKPRESCALE_MASK                         (0x03FFU)
#define CSL_SDFM_SDCOMP2EVT2FLTCLKCTL_CLKPRESCALE_SHIFT                        (0x0000U)
#define CSL_SDFM_SDCOMP2EVT2FLTCLKCTL_CLKPRESCALE_RESETVAL                     (0x0000U)
#define CSL_SDFM_SDCOMP2EVT2FLTCLKCTL_CLKPRESCALE_MAX                          (0x03FFU)

#define CSL_SDFM_SDCOMP2EVT2FLTCLKCTL_RESERVED_1_MASK                          (0xFC00U)
#define CSL_SDFM_SDCOMP2EVT2FLTCLKCTL_RESERVED_1_SHIFT                         (0x000AU)
#define CSL_SDFM_SDCOMP2EVT2FLTCLKCTL_RESERVED_1_RESETVAL                      (0x0000U)
#define CSL_SDFM_SDCOMP2EVT2FLTCLKCTL_RESERVED_1_MAX                           (0x003FU)

#define CSL_SDFM_SDCOMP2EVT2FLTCLKCTL_RESETVAL                                 (0x0000U)

/* SDCOMP2EVT1FLTCTL */

#define CSL_SDFM_SDCOMP2EVT1FLTCTL_RESERVED_1_MASK                             (0x000FU)
#define CSL_SDFM_SDCOMP2EVT1FLTCTL_RESERVED_1_SHIFT                            (0x0000U)
#define CSL_SDFM_SDCOMP2EVT1FLTCTL_RESERVED_1_RESETVAL                         (0x0000U)
#define CSL_SDFM_SDCOMP2EVT1FLTCTL_RESERVED_1_MAX                              (0x000FU)

#define CSL_SDFM_SDCOMP2EVT1FLTCTL_SAMPWIN_MASK                                (0x01F0U)
#define CSL_SDFM_SDCOMP2EVT1FLTCTL_SAMPWIN_SHIFT                               (0x0004U)
#define CSL_SDFM_SDCOMP2EVT1FLTCTL_SAMPWIN_RESETVAL                            (0x0000U)
#define CSL_SDFM_SDCOMP2EVT1FLTCTL_SAMPWIN_MAX                                 (0x001FU)

#define CSL_SDFM_SDCOMP2EVT1FLTCTL_THRESH_MASK                                 (0x3E00U)
#define CSL_SDFM_SDCOMP2EVT1FLTCTL_THRESH_SHIFT                                (0x0009U)
#define CSL_SDFM_SDCOMP2EVT1FLTCTL_THRESH_RESETVAL                             (0x0000U)
#define CSL_SDFM_SDCOMP2EVT1FLTCTL_THRESH_MAX                                  (0x001FU)

#define CSL_SDFM_SDCOMP2EVT1FLTCTL_RESERVED_2_MASK                             (0x4000U)
#define CSL_SDFM_SDCOMP2EVT1FLTCTL_RESERVED_2_SHIFT                            (0x000EU)
#define CSL_SDFM_SDCOMP2EVT1FLTCTL_RESERVED_2_RESETVAL                         (0x0000U)
#define CSL_SDFM_SDCOMP2EVT1FLTCTL_RESERVED_2_MAX                              (0x0001U)

#define CSL_SDFM_SDCOMP2EVT1FLTCTL_FILINIT_MASK                                (0x8000U)
#define CSL_SDFM_SDCOMP2EVT1FLTCTL_FILINIT_SHIFT                               (0x000FU)
#define CSL_SDFM_SDCOMP2EVT1FLTCTL_FILINIT_RESETVAL                            (0x0000U)
#define CSL_SDFM_SDCOMP2EVT1FLTCTL_FILINIT_MAX                                 (0x0001U)

#define CSL_SDFM_SDCOMP2EVT1FLTCTL_RESETVAL                                    (0x0000U)

/* SDCOMP2EVT1FLTCLKCTL */

#define CSL_SDFM_SDCOMP2EVT1FLTCLKCTL_CLKPRESCALE_MASK                         (0x03FFU)
#define CSL_SDFM_SDCOMP2EVT1FLTCLKCTL_CLKPRESCALE_SHIFT                        (0x0000U)
#define CSL_SDFM_SDCOMP2EVT1FLTCLKCTL_CLKPRESCALE_RESETVAL                     (0x0000U)
#define CSL_SDFM_SDCOMP2EVT1FLTCLKCTL_CLKPRESCALE_MAX                          (0x03FFU)

#define CSL_SDFM_SDCOMP2EVT1FLTCLKCTL_RESERVED_1_MASK                          (0xFC00U)
#define CSL_SDFM_SDCOMP2EVT1FLTCLKCTL_RESERVED_1_SHIFT                         (0x000AU)
#define CSL_SDFM_SDCOMP2EVT1FLTCLKCTL_RESERVED_1_RESETVAL                      (0x0000U)
#define CSL_SDFM_SDCOMP2EVT1FLTCLKCTL_RESERVED_1_MAX                           (0x003FU)

#define CSL_SDFM_SDCOMP2EVT1FLTCLKCTL_RESETVAL                                 (0x0000U)

/* SDCOMP2LOCK */

#define CSL_SDFM_SDCOMP2LOCK_SDCOMP2CTL_MASK                                   (0x0001U)
#define CSL_SDFM_SDCOMP2LOCK_SDCOMP2CTL_SHIFT                                  (0x0000U)
#define CSL_SDFM_SDCOMP2LOCK_SDCOMP2CTL_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDCOMP2LOCK_SDCOMP2CTL_MAX                                    (0x0001U)

#define CSL_SDFM_SDCOMP2LOCK_RESERVED_1_MASK                                   (0x0002U)
#define CSL_SDFM_SDCOMP2LOCK_RESERVED_1_SHIFT                                  (0x0001U)
#define CSL_SDFM_SDCOMP2LOCK_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDCOMP2LOCK_RESERVED_1_MAX                                    (0x0001U)

#define CSL_SDFM_SDCOMP2LOCK_RESERVED_2_MASK                                   (0x0004U)
#define CSL_SDFM_SDCOMP2LOCK_RESERVED_2_SHIFT                                  (0x0002U)
#define CSL_SDFM_SDCOMP2LOCK_RESERVED_2_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDCOMP2LOCK_RESERVED_2_MAX                                    (0x0001U)

#define CSL_SDFM_SDCOMP2LOCK_COMP_MASK                                         (0x0008U)
#define CSL_SDFM_SDCOMP2LOCK_COMP_SHIFT                                        (0x0003U)
#define CSL_SDFM_SDCOMP2LOCK_COMP_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDCOMP2LOCK_COMP_MAX                                          (0x0001U)

#define CSL_SDFM_SDCOMP2LOCK_RESERVED_3_MASK                                   (0x0010U)
#define CSL_SDFM_SDCOMP2LOCK_RESERVED_3_SHIFT                                  (0x0004U)
#define CSL_SDFM_SDCOMP2LOCK_RESERVED_3_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDCOMP2LOCK_RESERVED_3_MAX                                    (0x0001U)

#define CSL_SDFM_SDCOMP2LOCK_RESERVED_4_MASK                                   (0xFFE0U)
#define CSL_SDFM_SDCOMP2LOCK_RESERVED_4_SHIFT                                  (0x0005U)
#define CSL_SDFM_SDCOMP2LOCK_RESERVED_4_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDCOMP2LOCK_RESERVED_4_MAX                                    (0x07FFU)

#define CSL_SDFM_SDCOMP2LOCK_RESETVAL                                          (0x0000U)

/* SDCOMP3CTL */

#define CSL_SDFM_SDCOMP3CTL_RESERVED_1_MASK                                    (0x0001U)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_1_SHIFT                                   (0x0000U)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_1_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_1_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP3CTL_RESERVED_2_MASK                                    (0x0002U)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_2_SHIFT                                   (0x0001U)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_2_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_2_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP3CTL_CEVT1DIGFILTSEL_MASK                               (0x000CU)
#define CSL_SDFM_SDCOMP3CTL_CEVT1DIGFILTSEL_SHIFT                              (0x0002U)
#define CSL_SDFM_SDCOMP3CTL_CEVT1DIGFILTSEL_RESETVAL                           (0x0000U)
#define CSL_SDFM_SDCOMP3CTL_CEVT1DIGFILTSEL_MAX                                (0x0003U)

#define CSL_SDFM_SDCOMP3CTL_RESERVED_3_MASK                                    (0x0030U)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_3_SHIFT                                   (0x0004U)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_3_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_3_MAX                                     (0x0003U)

#define CSL_SDFM_SDCOMP3CTL_RESERVED_4_MASK                                    (0x0040U)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_4_SHIFT                                   (0x0006U)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_4_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_4_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP3CTL_RESERVED_5_MASK                                    (0x0080U)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_5_SHIFT                                   (0x0007U)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_5_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_5_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP3CTL_RESERVED_6_MASK                                    (0x0100U)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_6_SHIFT                                   (0x0008U)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_6_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_6_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP3CTL_RESERVED_7_MASK                                    (0x0200U)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_7_SHIFT                                   (0x0009U)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_7_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_7_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP3CTL_CEVT2DIGFILTSEL_MASK                               (0x0C00U)
#define CSL_SDFM_SDCOMP3CTL_CEVT2DIGFILTSEL_SHIFT                              (0x000AU)
#define CSL_SDFM_SDCOMP3CTL_CEVT2DIGFILTSEL_RESETVAL                           (0x0000U)
#define CSL_SDFM_SDCOMP3CTL_CEVT2DIGFILTSEL_MAX                                (0x0003U)

#define CSL_SDFM_SDCOMP3CTL_RESERVED_8_MASK                                    (0x3000U)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_8_SHIFT                                   (0x000CU)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_8_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_8_MAX                                     (0x0003U)

#define CSL_SDFM_SDCOMP3CTL_RESERVED_9_MASK                                    (0x4000U)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_9_SHIFT                                   (0x000EU)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_9_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_9_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP3CTL_RESERVED_10_MASK                                   (0x8000U)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_10_SHIFT                                  (0x000FU)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_10_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDCOMP3CTL_RESERVED_10_MAX                                    (0x0001U)

#define CSL_SDFM_SDCOMP3CTL_RESETVAL                                           (0x0000U)

/* SDCOMP3EVT2FLTCTL */

#define CSL_SDFM_SDCOMP3EVT2FLTCTL_RESERVED_1_MASK                             (0x000FU)
#define CSL_SDFM_SDCOMP3EVT2FLTCTL_RESERVED_1_SHIFT                            (0x0000U)
#define CSL_SDFM_SDCOMP3EVT2FLTCTL_RESERVED_1_RESETVAL                         (0x0000U)
#define CSL_SDFM_SDCOMP3EVT2FLTCTL_RESERVED_1_MAX                              (0x000FU)

#define CSL_SDFM_SDCOMP3EVT2FLTCTL_SAMPWIN_MASK                                (0x01F0U)
#define CSL_SDFM_SDCOMP3EVT2FLTCTL_SAMPWIN_SHIFT                               (0x0004U)
#define CSL_SDFM_SDCOMP3EVT2FLTCTL_SAMPWIN_RESETVAL                            (0x0000U)
#define CSL_SDFM_SDCOMP3EVT2FLTCTL_SAMPWIN_MAX                                 (0x001FU)

#define CSL_SDFM_SDCOMP3EVT2FLTCTL_THRESH_MASK                                 (0x3E00U)
#define CSL_SDFM_SDCOMP3EVT2FLTCTL_THRESH_SHIFT                                (0x0009U)
#define CSL_SDFM_SDCOMP3EVT2FLTCTL_THRESH_RESETVAL                             (0x0000U)
#define CSL_SDFM_SDCOMP3EVT2FLTCTL_THRESH_MAX                                  (0x001FU)

#define CSL_SDFM_SDCOMP3EVT2FLTCTL_RESERVED_2_MASK                             (0x4000U)
#define CSL_SDFM_SDCOMP3EVT2FLTCTL_RESERVED_2_SHIFT                            (0x000EU)
#define CSL_SDFM_SDCOMP3EVT2FLTCTL_RESERVED_2_RESETVAL                         (0x0000U)
#define CSL_SDFM_SDCOMP3EVT2FLTCTL_RESERVED_2_MAX                              (0x0001U)

#define CSL_SDFM_SDCOMP3EVT2FLTCTL_FILINIT_MASK                                (0x8000U)
#define CSL_SDFM_SDCOMP3EVT2FLTCTL_FILINIT_SHIFT                               (0x000FU)
#define CSL_SDFM_SDCOMP3EVT2FLTCTL_FILINIT_RESETVAL                            (0x0000U)
#define CSL_SDFM_SDCOMP3EVT2FLTCTL_FILINIT_MAX                                 (0x0001U)

#define CSL_SDFM_SDCOMP3EVT2FLTCTL_RESETVAL                                    (0x0000U)

/* SDCOMP3EVT2FLTCLKCTL */

#define CSL_SDFM_SDCOMP3EVT2FLTCLKCTL_CLKPRESCALE_MASK                         (0x03FFU)
#define CSL_SDFM_SDCOMP3EVT2FLTCLKCTL_CLKPRESCALE_SHIFT                        (0x0000U)
#define CSL_SDFM_SDCOMP3EVT2FLTCLKCTL_CLKPRESCALE_RESETVAL                     (0x0000U)
#define CSL_SDFM_SDCOMP3EVT2FLTCLKCTL_CLKPRESCALE_MAX                          (0x03FFU)

#define CSL_SDFM_SDCOMP3EVT2FLTCLKCTL_RESERVED_1_MASK                          (0xFC00U)
#define CSL_SDFM_SDCOMP3EVT2FLTCLKCTL_RESERVED_1_SHIFT                         (0x000AU)
#define CSL_SDFM_SDCOMP3EVT2FLTCLKCTL_RESERVED_1_RESETVAL                      (0x0000U)
#define CSL_SDFM_SDCOMP3EVT2FLTCLKCTL_RESERVED_1_MAX                           (0x003FU)

#define CSL_SDFM_SDCOMP3EVT2FLTCLKCTL_RESETVAL                                 (0x0000U)

/* SDCOMP3EVT1FLTCTL */

#define CSL_SDFM_SDCOMP3EVT1FLTCTL_RESERVED_1_MASK                             (0x000FU)
#define CSL_SDFM_SDCOMP3EVT1FLTCTL_RESERVED_1_SHIFT                            (0x0000U)
#define CSL_SDFM_SDCOMP3EVT1FLTCTL_RESERVED_1_RESETVAL                         (0x0000U)
#define CSL_SDFM_SDCOMP3EVT1FLTCTL_RESERVED_1_MAX                              (0x000FU)

#define CSL_SDFM_SDCOMP3EVT1FLTCTL_SAMPWIN_MASK                                (0x01F0U)
#define CSL_SDFM_SDCOMP3EVT1FLTCTL_SAMPWIN_SHIFT                               (0x0004U)
#define CSL_SDFM_SDCOMP3EVT1FLTCTL_SAMPWIN_RESETVAL                            (0x0000U)
#define CSL_SDFM_SDCOMP3EVT1FLTCTL_SAMPWIN_MAX                                 (0x001FU)

#define CSL_SDFM_SDCOMP3EVT1FLTCTL_THRESH_MASK                                 (0x3E00U)
#define CSL_SDFM_SDCOMP3EVT1FLTCTL_THRESH_SHIFT                                (0x0009U)
#define CSL_SDFM_SDCOMP3EVT1FLTCTL_THRESH_RESETVAL                             (0x0000U)
#define CSL_SDFM_SDCOMP3EVT1FLTCTL_THRESH_MAX                                  (0x001FU)

#define CSL_SDFM_SDCOMP3EVT1FLTCTL_RESERVED_2_MASK                             (0x4000U)
#define CSL_SDFM_SDCOMP3EVT1FLTCTL_RESERVED_2_SHIFT                            (0x000EU)
#define CSL_SDFM_SDCOMP3EVT1FLTCTL_RESERVED_2_RESETVAL                         (0x0000U)
#define CSL_SDFM_SDCOMP3EVT1FLTCTL_RESERVED_2_MAX                              (0x0001U)

#define CSL_SDFM_SDCOMP3EVT1FLTCTL_FILINIT_MASK                                (0x8000U)
#define CSL_SDFM_SDCOMP3EVT1FLTCTL_FILINIT_SHIFT                               (0x000FU)
#define CSL_SDFM_SDCOMP3EVT1FLTCTL_FILINIT_RESETVAL                            (0x0000U)
#define CSL_SDFM_SDCOMP3EVT1FLTCTL_FILINIT_MAX                                 (0x0001U)

#define CSL_SDFM_SDCOMP3EVT1FLTCTL_RESETVAL                                    (0x0000U)

/* SDCOMP3EVT1FLTCLKCTL */

#define CSL_SDFM_SDCOMP3EVT1FLTCLKCTL_CLKPRESCALE_MASK                         (0x03FFU)
#define CSL_SDFM_SDCOMP3EVT1FLTCLKCTL_CLKPRESCALE_SHIFT                        (0x0000U)
#define CSL_SDFM_SDCOMP3EVT1FLTCLKCTL_CLKPRESCALE_RESETVAL                     (0x0000U)
#define CSL_SDFM_SDCOMP3EVT1FLTCLKCTL_CLKPRESCALE_MAX                          (0x03FFU)

#define CSL_SDFM_SDCOMP3EVT1FLTCLKCTL_RESERVED_1_MASK                          (0xFC00U)
#define CSL_SDFM_SDCOMP3EVT1FLTCLKCTL_RESERVED_1_SHIFT                         (0x000AU)
#define CSL_SDFM_SDCOMP3EVT1FLTCLKCTL_RESERVED_1_RESETVAL                      (0x0000U)
#define CSL_SDFM_SDCOMP3EVT1FLTCLKCTL_RESERVED_1_MAX                           (0x003FU)

#define CSL_SDFM_SDCOMP3EVT1FLTCLKCTL_RESETVAL                                 (0x0000U)

/* SDCOMP3LOCK */

#define CSL_SDFM_SDCOMP3LOCK_SDCOMP3CTL_MASK                                   (0x0001U)
#define CSL_SDFM_SDCOMP3LOCK_SDCOMP3CTL_SHIFT                                  (0x0000U)
#define CSL_SDFM_SDCOMP3LOCK_SDCOMP3CTL_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDCOMP3LOCK_SDCOMP3CTL_MAX                                    (0x0001U)

#define CSL_SDFM_SDCOMP3LOCK_RESERVED_1_MASK                                   (0x0002U)
#define CSL_SDFM_SDCOMP3LOCK_RESERVED_1_SHIFT                                  (0x0001U)
#define CSL_SDFM_SDCOMP3LOCK_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDCOMP3LOCK_RESERVED_1_MAX                                    (0x0001U)

#define CSL_SDFM_SDCOMP3LOCK_RESERVED_2_MASK                                   (0x0004U)
#define CSL_SDFM_SDCOMP3LOCK_RESERVED_2_SHIFT                                  (0x0002U)
#define CSL_SDFM_SDCOMP3LOCK_RESERVED_2_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDCOMP3LOCK_RESERVED_2_MAX                                    (0x0001U)

#define CSL_SDFM_SDCOMP3LOCK_COMP_MASK                                         (0x0008U)
#define CSL_SDFM_SDCOMP3LOCK_COMP_SHIFT                                        (0x0003U)
#define CSL_SDFM_SDCOMP3LOCK_COMP_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDCOMP3LOCK_COMP_MAX                                          (0x0001U)

#define CSL_SDFM_SDCOMP3LOCK_RESERVED_3_MASK                                   (0x0010U)
#define CSL_SDFM_SDCOMP3LOCK_RESERVED_3_SHIFT                                  (0x0004U)
#define CSL_SDFM_SDCOMP3LOCK_RESERVED_3_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDCOMP3LOCK_RESERVED_3_MAX                                    (0x0001U)

#define CSL_SDFM_SDCOMP3LOCK_RESERVED_4_MASK                                   (0xFFE0U)
#define CSL_SDFM_SDCOMP3LOCK_RESERVED_4_SHIFT                                  (0x0005U)
#define CSL_SDFM_SDCOMP3LOCK_RESERVED_4_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDCOMP3LOCK_RESERVED_4_MAX                                    (0x07FFU)

#define CSL_SDFM_SDCOMP3LOCK_RESETVAL                                          (0x0000U)

/* SDCOMP4CTL */

#define CSL_SDFM_SDCOMP4CTL_RESERVED_1_MASK                                    (0x0001U)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_1_SHIFT                                   (0x0000U)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_1_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_1_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP4CTL_RESERVED_2_MASK                                    (0x0002U)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_2_SHIFT                                   (0x0001U)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_2_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_2_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP4CTL_CEVT1DIGFILTSEL_MASK                               (0x000CU)
#define CSL_SDFM_SDCOMP4CTL_CEVT1DIGFILTSEL_SHIFT                              (0x0002U)
#define CSL_SDFM_SDCOMP4CTL_CEVT1DIGFILTSEL_RESETVAL                           (0x0000U)
#define CSL_SDFM_SDCOMP4CTL_CEVT1DIGFILTSEL_MAX                                (0x0003U)

#define CSL_SDFM_SDCOMP4CTL_RESERVED_3_MASK                                    (0x0030U)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_3_SHIFT                                   (0x0004U)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_3_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_3_MAX                                     (0x0003U)

#define CSL_SDFM_SDCOMP4CTL_RESERVED_4_MASK                                    (0x0040U)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_4_SHIFT                                   (0x0006U)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_4_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_4_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP4CTL_RESERVED_5_MASK                                    (0x0080U)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_5_SHIFT                                   (0x0007U)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_5_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_5_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP4CTL_RESERVED_6_MASK                                    (0x0100U)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_6_SHIFT                                   (0x0008U)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_6_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_6_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP4CTL_RESERVED_7_MASK                                    (0x0200U)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_7_SHIFT                                   (0x0009U)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_7_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_7_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP4CTL_CEVT2DIGFILTSEL_MASK                               (0x0C00U)
#define CSL_SDFM_SDCOMP4CTL_CEVT2DIGFILTSEL_SHIFT                              (0x000AU)
#define CSL_SDFM_SDCOMP4CTL_CEVT2DIGFILTSEL_RESETVAL                           (0x0000U)
#define CSL_SDFM_SDCOMP4CTL_CEVT2DIGFILTSEL_MAX                                (0x0003U)

#define CSL_SDFM_SDCOMP4CTL_RESERVED_8_MASK                                    (0x3000U)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_8_SHIFT                                   (0x000CU)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_8_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_8_MAX                                     (0x0003U)

#define CSL_SDFM_SDCOMP4CTL_RESERVED_9_MASK                                    (0x4000U)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_9_SHIFT                                   (0x000EU)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_9_RESETVAL                                (0x0000U)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_9_MAX                                     (0x0001U)

#define CSL_SDFM_SDCOMP4CTL_RESERVED_10_MASK                                   (0x8000U)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_10_SHIFT                                  (0x000FU)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_10_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDCOMP4CTL_RESERVED_10_MAX                                    (0x0001U)

#define CSL_SDFM_SDCOMP4CTL_RESETVAL                                           (0x0000U)

/* SDCOMP4EVT2FLTCTL */

#define CSL_SDFM_SDCOMP4EVT2FLTCTL_RESERVED_1_MASK                             (0x000FU)
#define CSL_SDFM_SDCOMP4EVT2FLTCTL_RESERVED_1_SHIFT                            (0x0000U)
#define CSL_SDFM_SDCOMP4EVT2FLTCTL_RESERVED_1_RESETVAL                         (0x0000U)
#define CSL_SDFM_SDCOMP4EVT2FLTCTL_RESERVED_1_MAX                              (0x000FU)

#define CSL_SDFM_SDCOMP4EVT2FLTCTL_SAMPWIN_MASK                                (0x01F0U)
#define CSL_SDFM_SDCOMP4EVT2FLTCTL_SAMPWIN_SHIFT                               (0x0004U)
#define CSL_SDFM_SDCOMP4EVT2FLTCTL_SAMPWIN_RESETVAL                            (0x0000U)
#define CSL_SDFM_SDCOMP4EVT2FLTCTL_SAMPWIN_MAX                                 (0x001FU)

#define CSL_SDFM_SDCOMP4EVT2FLTCTL_THRESH_MASK                                 (0x3E00U)
#define CSL_SDFM_SDCOMP4EVT2FLTCTL_THRESH_SHIFT                                (0x0009U)
#define CSL_SDFM_SDCOMP4EVT2FLTCTL_THRESH_RESETVAL                             (0x0000U)
#define CSL_SDFM_SDCOMP4EVT2FLTCTL_THRESH_MAX                                  (0x001FU)

#define CSL_SDFM_SDCOMP4EVT2FLTCTL_RESERVED_2_MASK                             (0x4000U)
#define CSL_SDFM_SDCOMP4EVT2FLTCTL_RESERVED_2_SHIFT                            (0x000EU)
#define CSL_SDFM_SDCOMP4EVT2FLTCTL_RESERVED_2_RESETVAL                         (0x0000U)
#define CSL_SDFM_SDCOMP4EVT2FLTCTL_RESERVED_2_MAX                              (0x0001U)

#define CSL_SDFM_SDCOMP4EVT2FLTCTL_FILINIT_MASK                                (0x8000U)
#define CSL_SDFM_SDCOMP4EVT2FLTCTL_FILINIT_SHIFT                               (0x000FU)
#define CSL_SDFM_SDCOMP4EVT2FLTCTL_FILINIT_RESETVAL                            (0x0000U)
#define CSL_SDFM_SDCOMP4EVT2FLTCTL_FILINIT_MAX                                 (0x0001U)

#define CSL_SDFM_SDCOMP4EVT2FLTCTL_RESETVAL                                    (0x0000U)

/* SDCOMP4EVT2FLTCLKCTL */

#define CSL_SDFM_SDCOMP4EVT2FLTCLKCTL_CLKPRESCALE_MASK                         (0x03FFU)
#define CSL_SDFM_SDCOMP4EVT2FLTCLKCTL_CLKPRESCALE_SHIFT                        (0x0000U)
#define CSL_SDFM_SDCOMP4EVT2FLTCLKCTL_CLKPRESCALE_RESETVAL                     (0x0000U)
#define CSL_SDFM_SDCOMP4EVT2FLTCLKCTL_CLKPRESCALE_MAX                          (0x03FFU)

#define CSL_SDFM_SDCOMP4EVT2FLTCLKCTL_RESERVED_1_MASK                          (0xFC00U)
#define CSL_SDFM_SDCOMP4EVT2FLTCLKCTL_RESERVED_1_SHIFT                         (0x000AU)
#define CSL_SDFM_SDCOMP4EVT2FLTCLKCTL_RESERVED_1_RESETVAL                      (0x0000U)
#define CSL_SDFM_SDCOMP4EVT2FLTCLKCTL_RESERVED_1_MAX                           (0x003FU)

#define CSL_SDFM_SDCOMP4EVT2FLTCLKCTL_RESETVAL                                 (0x0000U)

/* SDCOMP4EVT1FLTCTL */

#define CSL_SDFM_SDCOMP4EVT1FLTCTL_RESERVED_1_MASK                             (0x000FU)
#define CSL_SDFM_SDCOMP4EVT1FLTCTL_RESERVED_1_SHIFT                            (0x0000U)
#define CSL_SDFM_SDCOMP4EVT1FLTCTL_RESERVED_1_RESETVAL                         (0x0000U)
#define CSL_SDFM_SDCOMP4EVT1FLTCTL_RESERVED_1_MAX                              (0x000FU)

#define CSL_SDFM_SDCOMP4EVT1FLTCTL_SAMPWIN_MASK                                (0x01F0U)
#define CSL_SDFM_SDCOMP4EVT1FLTCTL_SAMPWIN_SHIFT                               (0x0004U)
#define CSL_SDFM_SDCOMP4EVT1FLTCTL_SAMPWIN_RESETVAL                            (0x0000U)
#define CSL_SDFM_SDCOMP4EVT1FLTCTL_SAMPWIN_MAX                                 (0x001FU)

#define CSL_SDFM_SDCOMP4EVT1FLTCTL_THRESH_MASK                                 (0x3E00U)
#define CSL_SDFM_SDCOMP4EVT1FLTCTL_THRESH_SHIFT                                (0x0009U)
#define CSL_SDFM_SDCOMP4EVT1FLTCTL_THRESH_RESETVAL                             (0x0000U)
#define CSL_SDFM_SDCOMP4EVT1FLTCTL_THRESH_MAX                                  (0x001FU)

#define CSL_SDFM_SDCOMP4EVT1FLTCTL_RESERVED_2_MASK                             (0x4000U)
#define CSL_SDFM_SDCOMP4EVT1FLTCTL_RESERVED_2_SHIFT                            (0x000EU)
#define CSL_SDFM_SDCOMP4EVT1FLTCTL_RESERVED_2_RESETVAL                         (0x0000U)
#define CSL_SDFM_SDCOMP4EVT1FLTCTL_RESERVED_2_MAX                              (0x0001U)

#define CSL_SDFM_SDCOMP4EVT1FLTCTL_FILINIT_MASK                                (0x8000U)
#define CSL_SDFM_SDCOMP4EVT1FLTCTL_FILINIT_SHIFT                               (0x000FU)
#define CSL_SDFM_SDCOMP4EVT1FLTCTL_FILINIT_RESETVAL                            (0x0000U)
#define CSL_SDFM_SDCOMP4EVT1FLTCTL_FILINIT_MAX                                 (0x0001U)

#define CSL_SDFM_SDCOMP4EVT1FLTCTL_RESETVAL                                    (0x0000U)

/* SDCOMP4EVT1FLTCLKCTL */

#define CSL_SDFM_SDCOMP4EVT1FLTCLKCTL_CLKPRESCALE_MASK                         (0x03FFU)
#define CSL_SDFM_SDCOMP4EVT1FLTCLKCTL_CLKPRESCALE_SHIFT                        (0x0000U)
#define CSL_SDFM_SDCOMP4EVT1FLTCLKCTL_CLKPRESCALE_RESETVAL                     (0x0000U)
#define CSL_SDFM_SDCOMP4EVT1FLTCLKCTL_CLKPRESCALE_MAX                          (0x03FFU)

#define CSL_SDFM_SDCOMP4EVT1FLTCLKCTL_RESERVED_1_MASK                          (0xFC00U)
#define CSL_SDFM_SDCOMP4EVT1FLTCLKCTL_RESERVED_1_SHIFT                         (0x000AU)
#define CSL_SDFM_SDCOMP4EVT1FLTCLKCTL_RESERVED_1_RESETVAL                      (0x0000U)
#define CSL_SDFM_SDCOMP4EVT1FLTCLKCTL_RESERVED_1_MAX                           (0x003FU)

#define CSL_SDFM_SDCOMP4EVT1FLTCLKCTL_RESETVAL                                 (0x0000U)

/* SDCOMP4LOCK */

#define CSL_SDFM_SDCOMP4LOCK_SDCOMP4CTL_MASK                                   (0x0001U)
#define CSL_SDFM_SDCOMP4LOCK_SDCOMP4CTL_SHIFT                                  (0x0000U)
#define CSL_SDFM_SDCOMP4LOCK_SDCOMP4CTL_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDCOMP4LOCK_SDCOMP4CTL_MAX                                    (0x0001U)

#define CSL_SDFM_SDCOMP4LOCK_RESERVED_1_MASK                                   (0x0002U)
#define CSL_SDFM_SDCOMP4LOCK_RESERVED_1_SHIFT                                  (0x0001U)
#define CSL_SDFM_SDCOMP4LOCK_RESERVED_1_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDCOMP4LOCK_RESERVED_1_MAX                                    (0x0001U)

#define CSL_SDFM_SDCOMP4LOCK_RESERVED_2_MASK                                   (0x0004U)
#define CSL_SDFM_SDCOMP4LOCK_RESERVED_2_SHIFT                                  (0x0002U)
#define CSL_SDFM_SDCOMP4LOCK_RESERVED_2_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDCOMP4LOCK_RESERVED_2_MAX                                    (0x0001U)

#define CSL_SDFM_SDCOMP4LOCK_COMP_MASK                                         (0x0008U)
#define CSL_SDFM_SDCOMP4LOCK_COMP_SHIFT                                        (0x0003U)
#define CSL_SDFM_SDCOMP4LOCK_COMP_RESETVAL                                     (0x0000U)
#define CSL_SDFM_SDCOMP4LOCK_COMP_MAX                                          (0x0001U)

#define CSL_SDFM_SDCOMP4LOCK_RESERVED_3_MASK                                   (0x0010U)
#define CSL_SDFM_SDCOMP4LOCK_RESERVED_3_SHIFT                                  (0x0004U)
#define CSL_SDFM_SDCOMP4LOCK_RESERVED_3_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDCOMP4LOCK_RESERVED_3_MAX                                    (0x0001U)

#define CSL_SDFM_SDCOMP4LOCK_RESERVED_4_MASK                                   (0xFFE0U)
#define CSL_SDFM_SDCOMP4LOCK_RESERVED_4_SHIFT                                  (0x0005U)
#define CSL_SDFM_SDCOMP4LOCK_RESERVED_4_RESETVAL                               (0x0000U)
#define CSL_SDFM_SDCOMP4LOCK_RESERVED_4_MAX                                    (0x07FFU)

#define CSL_SDFM_SDCOMP4LOCK_RESETVAL                                          (0x0000U)

#ifdef __cplusplus
}
#endif
#endif
