/********************************************************************
 * Copyright (C) 2020 Texas Instruments Incorporated.
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
 *  Name        : cslr_pbist.h
*/
#ifndef CSLR_PBIST_H_
#define CSLR_PBIST_H_

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
    volatile uint8_t  Resv_256[256];
    volatile uint32_t PBIST_A0;
    volatile uint32_t PBIST_A1;
    volatile uint32_t PBIST_A2;
    volatile uint32_t PBIST_A3;
    volatile uint32_t PBIST_L0;
    volatile uint32_t PBIST_L1;
    volatile uint32_t PBIST_L2;
    volatile uint32_t PBIST_L3;
    volatile uint32_t PBIST_DD10;
    volatile uint32_t PBIST_DE10;
    volatile uint8_t  Resv_304[8];
    volatile uint32_t PBIST_CA0;
    volatile uint32_t PBIST_CA1;
    volatile uint32_t PBIST_CA2;
    volatile uint32_t PBIST_CA3;
    volatile uint32_t PBIST_CL0;
    volatile uint32_t PBIST_CL1;
    volatile uint32_t PBIST_CL2;
    volatile uint32_t PBIST_CL3;
    volatile uint32_t PBIST_CI0;
    volatile uint32_t PBIST_CI1;
    volatile uint16_t PBIST_CI2;
    volatile uint8_t  Resv_348[2];
    volatile uint16_t PBIST_CI3;
    volatile uint8_t  Resv_352[2];
    volatile uint32_t PBIST_RAMT;
    volatile uint16_t PBIST_DLR;
    volatile uint8_t  Resv_360[2];
    volatile uint8_t PBIST_CMS;
    volatile uint8_t  Resv_364[3];
    volatile uint8_t PBIST_PC;
    volatile uint8_t  Resv_368[3];
    volatile uint32_t PBIST_SCR1;
    volatile uint32_t PBIST_SCR4;
    volatile uint32_t PBIST_CS;
    volatile uint8_t PBIST_FDLY;
    volatile uint8_t  Resv_384[3];
    volatile uint8_t PBIST_PACT;
    volatile uint8_t  Resv_388[3];
    volatile uint8_t PBIST_ID;
    volatile uint8_t  Resv_392[3];
    volatile uint32_t PBIST_OVR;
    volatile uint8_t  Resv_400[4];
    volatile uint8_t PBIST_FSFR0;
    volatile uint8_t  Resv_404[3];
    volatile uint8_t PBIST_FSFR1;
    volatile uint8_t  Resv_408[3];
    volatile uint8_t PBIST_FSRCR0;
    volatile uint8_t  Resv_412[3];
    volatile uint8_t PBIST_FSRCR1;
    volatile uint8_t  Resv_416[3];
    volatile uint32_t PBIST_FSRA0;
    volatile uint16_t PBIST_FSRA1;
    volatile uint8_t  Resv_424[2];
    volatile uint32_t PBIST_FSRDL0;
    volatile uint8_t  Resv_432[4];
    volatile uint32_t PBIST_FSRDL1;
    volatile uint32_t PBIST_MARGIN;
    volatile uint32_t PBIST_WRENZ;
    volatile uint32_t PBIST_PGS;
    volatile uint8_t PBIST_ROM;
    volatile uint8_t  Resv_452[3];
    volatile uint32_t PBIST_ALGO;
    volatile uint32_t PBIST_RINFOL;
    volatile uint32_t PBIST_RINFOU;
} CSL_pbistRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_PBIST_PBIST_A0                                                     (0x00000100U)
#define CSL_PBIST_PBIST_A1                                                     (0x00000104U)
#define CSL_PBIST_PBIST_A2                                                     (0x00000108U)
#define CSL_PBIST_PBIST_A3                                                     (0x0000010CU)
#define CSL_PBIST_PBIST_L0                                                     (0x00000110U)
#define CSL_PBIST_PBIST_L1                                                     (0x00000114U)
#define CSL_PBIST_PBIST_L2                                                     (0x00000118U)
#define CSL_PBIST_PBIST_L3                                                     (0x0000011CU)
#define CSL_PBIST_PBIST_DD10                                                   (0x00000120U)
#define CSL_PBIST_PBIST_DE10                                                   (0x00000124U)
#define CSL_PBIST_PBIST_CA0                                                    (0x00000130U)
#define CSL_PBIST_PBIST_CA1                                                    (0x00000134U)
#define CSL_PBIST_PBIST_CA2                                                    (0x00000138U)
#define CSL_PBIST_PBIST_CA3                                                    (0x0000013CU)
#define CSL_PBIST_PBIST_CL0                                                    (0x00000140U)
#define CSL_PBIST_PBIST_CL1                                                    (0x00000144U)
#define CSL_PBIST_PBIST_CL2                                                    (0x00000148U)
#define CSL_PBIST_PBIST_CL3                                                    (0x0000014CU)
#define CSL_PBIST_PBIST_CI0                                                    (0x00000150U)
#define CSL_PBIST_PBIST_CI1                                                    (0x00000154U)
#define CSL_PBIST_PBIST_CI2                                                    (0x00000158U)
#define CSL_PBIST_PBIST_CI3                                                    (0x0000015CU)
#define CSL_PBIST_PBIST_RAMT                                                   (0x00000160U)
#define CSL_PBIST_PBIST_DLR                                                    (0x00000164U)
#define CSL_PBIST_PBIST_CMS                                                    (0x00000168U)
#define CSL_PBIST_PBIST_PC                                                     (0x0000016CU)
#define CSL_PBIST_PBIST_SCR1                                                   (0x00000170U)
#define CSL_PBIST_PBIST_SCR4                                                   (0x00000174U)
#define CSL_PBIST_PBIST_CS                                                     (0x00000178U)
#define CSL_PBIST_PBIST_FDLY                                                   (0x0000017CU)
#define CSL_PBIST_PBIST_PACT                                                   (0x00000180U)
#define CSL_PBIST_PBIST_ID                                                     (0x00000184U)
#define CSL_PBIST_PBIST_OVR                                                    (0x00000188U)
#define CSL_PBIST_PBIST_FSFR0                                                  (0x00000190U)
#define CSL_PBIST_PBIST_FSFR1                                                  (0x00000194U)
#define CSL_PBIST_PBIST_FSRCR0                                                 (0x00000198U)
#define CSL_PBIST_PBIST_FSRCR1                                                 (0x0000019CU)
#define CSL_PBIST_PBIST_FSRA0                                                  (0x000001A0U)
#define CSL_PBIST_PBIST_FSRA1                                                  (0x000001A4U)
#define CSL_PBIST_PBIST_FSRDL0                                                 (0x000001A8U)
#define CSL_PBIST_PBIST_FSRDL1                                                 (0x000001B0U)
#define CSL_PBIST_PBIST_MARGIN                                                 (0x000001B4U)
#define CSL_PBIST_PBIST_WRENZ                                                  (0x000001B8U)
#define CSL_PBIST_PBIST_PGS                                                    (0x000001BCU)
#define CSL_PBIST_PBIST_ROM                                                    (0x000001C0U)
#define CSL_PBIST_PBIST_ALGO                                                   (0x000001C4U)
#define CSL_PBIST_PBIST_RINFOL                                                 (0x000001C8U)
#define CSL_PBIST_PBIST_RINFOU                                                 (0x000001CCU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PBIST_A0 */


/* PBIST_A1 */


/* PBIST_A2 */


/* PBIST_A3 */


/* PBIST_L0 */


/* PBIST_L1 */


/* PBIST_L2 */


/* PBIST_L3 */


/* PBIST_DD10 */


/* PBIST_DE10 */


/* PBIST_CA0 */


/* PBIST_CA1 */


/* PBIST_CA2 */


/* PBIST_CA3 */


/* PBIST_CL0 */


/* PBIST_CL1 */


/* PBIST_CL2 */


/* PBIST_CL3 */


/* PBIST_CI0 */


/* PBIST_CI1 */


/* PBIST_CI2 */

#define CSL_PBIST_PBIST_CI2_PBIST_CI2_MASK                                     (0xFFFFU)
#define CSL_PBIST_PBIST_CI2_PBIST_CI2_SHIFT                                    (0x0000U)
#define CSL_PBIST_PBIST_CI2_PBIST_CI2_RESETVAL                                 (0x0000U)
#define CSL_PBIST_PBIST_CI2_PBIST_CI2_MAX                                      (0xFFFFU)

#define CSL_PBIST_PBIST_CI2_RESETVAL                                           (0x0000U)

/* PBIST_CI3 */

#define CSL_PBIST_PBIST_CI3_PBIST_CI3_MASK                                     (0xFFFFU)
#define CSL_PBIST_PBIST_CI3_PBIST_CI3_SHIFT                                    (0x0000U)
#define CSL_PBIST_PBIST_CI3_PBIST_CI3_RESETVAL                                 (0x0000U)
#define CSL_PBIST_PBIST_CI3_PBIST_CI3_MAX                                      (0xFFFFU)

#define CSL_PBIST_PBIST_CI3_RESETVAL                                           (0x0000U)

/* PBIST_RAMT */

#define CSL_PBIST_PBIST_RAMT_RAM_MASK                                          (0x000000FFU)
#define CSL_PBIST_PBIST_RAMT_RAM_SHIFT                                         (0x00000000U)
#define CSL_PBIST_PBIST_RAMT_RAM_RESETVAL                                      (0x00000000U)
#define CSL_PBIST_PBIST_RAMT_RAM_MAX                                           (0x000000FFU)

#define CSL_PBIST_PBIST_RAMT_DWR_MASK                                          (0x0000FF00U)
#define CSL_PBIST_PBIST_RAMT_DWR_SHIFT                                         (0x00000008U)
#define CSL_PBIST_PBIST_RAMT_DWR_RESETVAL                                      (0x00000000U)
#define CSL_PBIST_PBIST_RAMT_DWR_MAX                                           (0x000000FFU)

#define CSL_PBIST_PBIST_RAMT_RDS_MASK                                          (0x00FF0000U)
#define CSL_PBIST_PBIST_RAMT_RDS_SHIFT                                         (0x00000010U)
#define CSL_PBIST_PBIST_RAMT_RDS_RESETVAL                                      (0x00000000U)
#define CSL_PBIST_PBIST_RAMT_RDS_MAX                                           (0x000000FFU)

#define CSL_PBIST_PBIST_RAMT_RGS_MASK                                          (0xFF000000U)
#define CSL_PBIST_PBIST_RAMT_RGS_SHIFT                                         (0x00000018U)
#define CSL_PBIST_PBIST_RAMT_RGS_RESETVAL                                      (0x00000000U)
#define CSL_PBIST_PBIST_RAMT_RGS_MAX                                           (0x000000FFU)

#define CSL_PBIST_PBIST_RAMT_RESETVAL                                          (0x00000000U)

/* PBIST_DLR */

#define CSL_PBIST_PBIST_DLR_DLR0_MASK                                          (0x00FFU)
#define CSL_PBIST_PBIST_DLR_DLR0_SHIFT                                         (0x0000U)
#define CSL_PBIST_PBIST_DLR_DLR0_RESETVAL                                      (0x0008U)
#define CSL_PBIST_PBIST_DLR_DLR0_MAX                                           (0x00FFU)

#define CSL_PBIST_PBIST_DLR_DLR1_MASK                                          (0xFF00U)
#define CSL_PBIST_PBIST_DLR_DLR1_SHIFT                                         (0x0008U)
#define CSL_PBIST_PBIST_DLR_DLR1_RESETVAL                                      (0x0002U)
#define CSL_PBIST_PBIST_DLR_DLR1_MAX                                           (0x00FFU)

#define CSL_PBIST_PBIST_DLR_RESETVAL                                           (0x0208U)

/* PBIST_CMS */

#define CSL_PBIST_PBIST_CMS_PBIST_CMS_MASK                                     (0x0FU)
#define CSL_PBIST_PBIST_CMS_PBIST_CMS_SHIFT                                    (0x00U)
#define CSL_PBIST_PBIST_CMS_PBIST_CMS_RESETVAL                                 (0x00U)
#define CSL_PBIST_PBIST_CMS_PBIST_CMS_MAX                                      (0x0FU)

#define CSL_PBIST_PBIST_CMS_RESETVAL                                           (0x00U)

/* PBIST_PC */

#define CSL_PBIST_PBIST_PC_PBIST_PC_MASK                                       (0x1FU)
#define CSL_PBIST_PBIST_PC_PBIST_PC_SHIFT                                      (0x00U)
#define CSL_PBIST_PBIST_PC_PBIST_PC_RESETVAL                                   (0x00U)
#define CSL_PBIST_PBIST_PC_PBIST_PC_MAX                                        (0x1FU)

#define CSL_PBIST_PBIST_PC_RESETVAL                                            (0x00U)

/* PBIST_SCR1 */

#define CSL_PBIST_PBIST_SCR1_SCR0_MASK                                         (0x000000FFU)
#define CSL_PBIST_PBIST_SCR1_SCR0_SHIFT                                        (0x00000000U)
#define CSL_PBIST_PBIST_SCR1_SCR0_RESETVAL                                     (0x00000010U)
#define CSL_PBIST_PBIST_SCR1_SCR0_MAX                                          (0x000000FFU)

#define CSL_PBIST_PBIST_SCR1_SCR1_MASK                                         (0x0000FF00U)
#define CSL_PBIST_PBIST_SCR1_SCR1_SHIFT                                        (0x00000008U)
#define CSL_PBIST_PBIST_SCR1_SCR1_RESETVAL                                     (0x00000032U)
#define CSL_PBIST_PBIST_SCR1_SCR1_MAX                                          (0x000000FFU)

#define CSL_PBIST_PBIST_SCR1_SCR2_MASK                                         (0x00FF0000U)
#define CSL_PBIST_PBIST_SCR1_SCR2_SHIFT                                        (0x00000010U)
#define CSL_PBIST_PBIST_SCR1_SCR2_RESETVAL                                     (0x00000054U)
#define CSL_PBIST_PBIST_SCR1_SCR2_MAX                                          (0x000000FFU)

#define CSL_PBIST_PBIST_SCR1_SCR3_MASK                                         (0xFF000000U)
#define CSL_PBIST_PBIST_SCR1_SCR3_SHIFT                                        (0x00000018U)
#define CSL_PBIST_PBIST_SCR1_SCR3_RESETVAL                                     (0x00000076U)
#define CSL_PBIST_PBIST_SCR1_SCR3_MAX                                          (0x000000FFU)

#define CSL_PBIST_PBIST_SCR1_RESETVAL                                          (0x76543210U)

/* PBIST_SCR4 */

#define CSL_PBIST_PBIST_SCR4_SCR4_MASK                                         (0x000000FFU)
#define CSL_PBIST_PBIST_SCR4_SCR4_SHIFT                                        (0x00000000U)
#define CSL_PBIST_PBIST_SCR4_SCR4_RESETVAL                                     (0x00000098U)
#define CSL_PBIST_PBIST_SCR4_SCR4_MAX                                          (0x000000FFU)

#define CSL_PBIST_PBIST_SCR4_SCR5_MASK                                         (0x0000FF00U)
#define CSL_PBIST_PBIST_SCR4_SCR5_SHIFT                                        (0x00000008U)
#define CSL_PBIST_PBIST_SCR4_SCR5_RESETVAL                                     (0x000000BAU)
#define CSL_PBIST_PBIST_SCR4_SCR5_MAX                                          (0x000000FFU)

#define CSL_PBIST_PBIST_SCR4_SCR6_MASK                                         (0x00FF0000U)
#define CSL_PBIST_PBIST_SCR4_SCR6_SHIFT                                        (0x00000010U)
#define CSL_PBIST_PBIST_SCR4_SCR6_RESETVAL                                     (0x000000DCU)
#define CSL_PBIST_PBIST_SCR4_SCR6_MAX                                          (0x000000FFU)

#define CSL_PBIST_PBIST_SCR4_SCR7_MASK                                         (0xFF000000U)
#define CSL_PBIST_PBIST_SCR4_SCR7_SHIFT                                        (0x00000018U)
#define CSL_PBIST_PBIST_SCR4_SCR7_RESETVAL                                     (0x000000FEU)
#define CSL_PBIST_PBIST_SCR4_SCR7_MAX                                          (0x000000FFU)

#define CSL_PBIST_PBIST_SCR4_RESETVAL                                          (0xFEDCBA98U)

/* PBIST_CS */

#define CSL_PBIST_PBIST_CS_CS0_MASK                                            (0x000000FFU)
#define CSL_PBIST_PBIST_CS_CS0_SHIFT                                           (0x00000000U)
#define CSL_PBIST_PBIST_CS_CS0_RESETVAL                                        (0x00000000U)
#define CSL_PBIST_PBIST_CS_CS0_MAX                                             (0x000000FFU)

#define CSL_PBIST_PBIST_CS_CS1_MASK                                            (0x0000FF00U)
#define CSL_PBIST_PBIST_CS_CS1_SHIFT                                           (0x00000008U)
#define CSL_PBIST_PBIST_CS_CS1_RESETVAL                                        (0x00000000U)
#define CSL_PBIST_PBIST_CS_CS1_MAX                                             (0x000000FFU)

#define CSL_PBIST_PBIST_CS_CS2_MASK                                            (0x00FF0000U)
#define CSL_PBIST_PBIST_CS_CS2_SHIFT                                           (0x00000010U)
#define CSL_PBIST_PBIST_CS_CS2_RESETVAL                                        (0x00000000U)
#define CSL_PBIST_PBIST_CS_CS2_MAX                                             (0x000000FFU)

#define CSL_PBIST_PBIST_CS_CS3_MASK                                            (0xFF000000U)
#define CSL_PBIST_PBIST_CS_CS3_SHIFT                                           (0x00000018U)
#define CSL_PBIST_PBIST_CS_CS3_RESETVAL                                        (0x00000000U)
#define CSL_PBIST_PBIST_CS_CS3_MAX                                             (0x000000FFU)

#define CSL_PBIST_PBIST_CS_RESETVAL                                            (0x00000000U)

/* PBIST_FDLY */

#define CSL_PBIST_PBIST_FDLY_PBIST_FDLY_MASK                                   (0xFFU)
#define CSL_PBIST_PBIST_FDLY_PBIST_FDLY_SHIFT                                  (0x00U)
#define CSL_PBIST_PBIST_FDLY_PBIST_FDLY_RESETVAL                               (0x48U)
#define CSL_PBIST_PBIST_FDLY_PBIST_FDLY_MAX                                    (0xFFU)

#define CSL_PBIST_PBIST_FDLY_RESETVAL                                          (0x48U)

/* PBIST_PACT */

#define CSL_PBIST_PBIST_PACT_PBIST_PACT_MASK                                   (0x01U)
#define CSL_PBIST_PBIST_PACT_PBIST_PACT_SHIFT                                  (0x00U)
#define CSL_PBIST_PBIST_PACT_PBIST_PACT_RESETVAL                               (0x00U)
#define CSL_PBIST_PBIST_PACT_PBIST_PACT_MAX                                    (0x01U)

#define CSL_PBIST_PBIST_PACT_RESETVAL                                          (0x00U)

/* PBIST_ID */

#define CSL_PBIST_PBIST_ID_PBIST_ID_MASK                                       (0x1FU)
#define CSL_PBIST_PBIST_ID_PBIST_ID_SHIFT                                      (0x00U)
#define CSL_PBIST_PBIST_ID_PBIST_ID_RESETVAL                                   (0x01U)
#define CSL_PBIST_PBIST_ID_PBIST_ID_MAX                                        (0x1FU)

#define CSL_PBIST_PBIST_ID_RESETVAL                                            (0x01U)

/* PBIST_OVR */


/* PBIST_FSFR0 */

#define CSL_PBIST_PBIST_FSFR0_PBIST_FSFR0_MASK                                 (0x01U)
#define CSL_PBIST_PBIST_FSFR0_PBIST_FSFR0_SHIFT                                (0x00U)
#define CSL_PBIST_PBIST_FSFR0_PBIST_FSFR0_RESETVAL                             (0x00U)
#define CSL_PBIST_PBIST_FSFR0_PBIST_FSFR0_MAX                                  (0x01U)

#define CSL_PBIST_PBIST_FSFR0_RESETVAL                                         (0x00U)

/* PBIST_FSFR1 */

#define CSL_PBIST_PBIST_FSFR1_PBIST_FSFR1_MASK                                 (0x01U)
#define CSL_PBIST_PBIST_FSFR1_PBIST_FSFR1_SHIFT                                (0x00U)
#define CSL_PBIST_PBIST_FSFR1_PBIST_FSFR1_RESETVAL                             (0x00U)
#define CSL_PBIST_PBIST_FSFR1_PBIST_FSFR1_MAX                                  (0x01U)

#define CSL_PBIST_PBIST_FSFR1_RESETVAL                                         (0x00U)

/* PBIST_FSRCR0 */

#define CSL_PBIST_PBIST_FSRCR0_PBIST_FSRCR0_MASK                               (0x0FU)
#define CSL_PBIST_PBIST_FSRCR0_PBIST_FSRCR0_SHIFT                              (0x00U)
#define CSL_PBIST_PBIST_FSRCR0_PBIST_FSRCR0_RESETVAL                           (0x00U)
#define CSL_PBIST_PBIST_FSRCR0_PBIST_FSRCR0_MAX                                (0x0FU)

#define CSL_PBIST_PBIST_FSRCR0_RESETVAL                                        (0x00U)

/* PBIST_FSRCR1 */

#define CSL_PBIST_PBIST_FSRCR1_PBIST_FSRCR1_MASK                               (0x0FU)
#define CSL_PBIST_PBIST_FSRCR1_PBIST_FSRCR1_SHIFT                              (0x00U)
#define CSL_PBIST_PBIST_FSRCR1_PBIST_FSRCR1_RESETVAL                           (0x00U)
#define CSL_PBIST_PBIST_FSRCR1_PBIST_FSRCR1_MAX                                (0x0FU)

#define CSL_PBIST_PBIST_FSRCR1_RESETVAL                                        (0x00U)

/* PBIST_FSRA0 */


/* PBIST_FSRA1 */

#define CSL_PBIST_PBIST_FSRA1_PBIST_FSRA1_MASK                                 (0xFFFFU)
#define CSL_PBIST_PBIST_FSRA1_PBIST_FSRA1_SHIFT                                (0x0000U)
#define CSL_PBIST_PBIST_FSRA1_PBIST_FSRA1_RESETVAL                             (0x0000U)
#define CSL_PBIST_PBIST_FSRA1_PBIST_FSRA1_MAX                                  (0xFFFFU)

#define CSL_PBIST_PBIST_FSRA1_RESETVAL                                         (0x0000U)

/* PBIST_FSRDL0 */

#define CSL_PBIST_PBIST_FSRDL0_PBIST_FSRDL0_MASK                               (0xFFFFFFFFU)
#define CSL_PBIST_PBIST_FSRDL0_PBIST_FSRDL0_SHIFT                              (0x00000000U)
#define CSL_PBIST_PBIST_FSRDL0_PBIST_FSRDL0_RESETVAL                           (0xAAAAAAAAU)
#define CSL_PBIST_PBIST_FSRDL0_PBIST_FSRDL0_MAX                                (0xFFFFFFFFU)

#define CSL_PBIST_PBIST_FSRDL0_RESETVAL                                        (0xAAAAAAAAU)

/* PBIST_FSRDL1 */

#define CSL_PBIST_PBIST_FSRDL1_PBIST_FSRDL1_MASK                               (0xFFFFFFFFU)
#define CSL_PBIST_PBIST_FSRDL1_PBIST_FSRDL1_SHIFT                              (0x00000000U)
#define CSL_PBIST_PBIST_FSRDL1_PBIST_FSRDL1_RESETVAL                           (0xAAAAAAAAU)
#define CSL_PBIST_PBIST_FSRDL1_PBIST_FSRDL1_MAX                                (0xFFFFFFFFU)

#define CSL_PBIST_PBIST_FSRDL1_RESETVAL                                        (0xAAAAAAAAU)

/* PBIST_MARGIN */


/* PBIST_WRENZ */


/* PBIST_PGS */


/* PBIST_ROM */

#define CSL_PBIST_PBIST_ROM_PBIST_ROM_MASK                                     (0x03U)
#define CSL_PBIST_PBIST_ROM_PBIST_ROM_SHIFT                                    (0x00U)
#define CSL_PBIST_PBIST_ROM_PBIST_ROM_RESETVAL                                 (0x03U)
#define CSL_PBIST_PBIST_ROM_PBIST_ROM_MAX                                      (0x03U)

#define CSL_PBIST_PBIST_ROM_RESETVAL                                           (0x03U)

/* PBIST_ALGO */

#define CSL_PBIST_PBIST_ALGO_ALGO0_MASK                                        (0x000000FFU)
#define CSL_PBIST_PBIST_ALGO_ALGO0_SHIFT                                       (0x00000000U)
#define CSL_PBIST_PBIST_ALGO_ALGO0_RESETVAL                                    (0x000000FFU)
#define CSL_PBIST_PBIST_ALGO_ALGO0_MAX                                         (0x000000FFU)

#define CSL_PBIST_PBIST_ALGO_ALGO1_MASK                                        (0x0000FF00U)
#define CSL_PBIST_PBIST_ALGO_ALGO1_SHIFT                                       (0x00000008U)
#define CSL_PBIST_PBIST_ALGO_ALGO1_RESETVAL                                    (0x000000FFU)
#define CSL_PBIST_PBIST_ALGO_ALGO1_MAX                                         (0x000000FFU)

#define CSL_PBIST_PBIST_ALGO_ALGO2_MASK                                        (0x00FF0000U)
#define CSL_PBIST_PBIST_ALGO_ALGO2_SHIFT                                       (0x00000010U)
#define CSL_PBIST_PBIST_ALGO_ALGO2_RESETVAL                                    (0x000000FFU)
#define CSL_PBIST_PBIST_ALGO_ALGO2_MAX                                         (0x000000FFU)

#define CSL_PBIST_PBIST_ALGO_ALGO3_MASK                                        (0xFF000000U)
#define CSL_PBIST_PBIST_ALGO_ALGO3_SHIFT                                       (0x00000018U)
#define CSL_PBIST_PBIST_ALGO_ALGO3_RESETVAL                                    (0x000000FFU)
#define CSL_PBIST_PBIST_ALGO_ALGO3_MAX                                         (0x000000FFU)

#define CSL_PBIST_PBIST_ALGO_RESETVAL                                          (0xFFFFFFFFU)

/* PBIST_RINFOL */

#define CSL_PBIST_PBIST_RINFOL_RINFOL0_MASK                                    (0x000000FFU)
#define CSL_PBIST_PBIST_RINFOL_RINFOL0_SHIFT                                   (0x00000000U)
#define CSL_PBIST_PBIST_RINFOL_RINFOL0_RESETVAL                                (0x000000FFU)
#define CSL_PBIST_PBIST_RINFOL_RINFOL0_MAX                                     (0x000000FFU)

#define CSL_PBIST_PBIST_RINFOL_RINFOL1_MASK                                    (0x0000FF00U)
#define CSL_PBIST_PBIST_RINFOL_RINFOL1_SHIFT                                   (0x00000008U)
#define CSL_PBIST_PBIST_RINFOL_RINFOL1_RESETVAL                                (0x000000FFU)
#define CSL_PBIST_PBIST_RINFOL_RINFOL1_MAX                                     (0x000000FFU)

#define CSL_PBIST_PBIST_RINFOL_RINFOL2_MASK                                    (0x00FF0000U)
#define CSL_PBIST_PBIST_RINFOL_RINFOL2_SHIFT                                   (0x00000010U)
#define CSL_PBIST_PBIST_RINFOL_RINFOL2_RESETVAL                                (0x000000FFU)
#define CSL_PBIST_PBIST_RINFOL_RINFOL2_MAX                                     (0x000000FFU)

#define CSL_PBIST_PBIST_RINFOL_RINFOL3_MASK                                    (0xFF000000U)
#define CSL_PBIST_PBIST_RINFOL_RINFOL3_SHIFT                                   (0x00000018U)
#define CSL_PBIST_PBIST_RINFOL_RINFOL3_RESETVAL                                (0x000000FFU)
#define CSL_PBIST_PBIST_RINFOL_RINFOL3_MAX                                     (0x000000FFU)

#define CSL_PBIST_PBIST_RINFOL_RESETVAL                                        (0xFFFFFFFFU)

/* PBIST_RINFOU */

#define CSL_PBIST_PBIST_RINFOU_RINFOU0_MASK                                    (0x000000FFU)
#define CSL_PBIST_PBIST_RINFOU_RINFOU0_SHIFT                                   (0x00000000U)
#define CSL_PBIST_PBIST_RINFOU_RINFOU0_RESETVAL                                (0x000000FFU)
#define CSL_PBIST_PBIST_RINFOU_RINFOU0_MAX                                     (0x000000FFU)

#define CSL_PBIST_PBIST_RINFOU_RINFOU1_MASK                                    (0x0000FF00U)
#define CSL_PBIST_PBIST_RINFOU_RINFOU1_SHIFT                                   (0x00000008U)
#define CSL_PBIST_PBIST_RINFOU_RINFOU1_RESETVAL                                (0x000000FFU)
#define CSL_PBIST_PBIST_RINFOU_RINFOU1_MAX                                     (0x000000FFU)

#define CSL_PBIST_PBIST_RINFOU_RINFOU2_MASK                                    (0x00FF0000U)
#define CSL_PBIST_PBIST_RINFOU_RINFOU2_SHIFT                                   (0x00000010U)
#define CSL_PBIST_PBIST_RINFOU_RINFOU2_RESETVAL                                (0x000000FFU)
#define CSL_PBIST_PBIST_RINFOU_RINFOU2_MAX                                     (0x000000FFU)

#define CSL_PBIST_PBIST_RINFOU_RINFOU3_MASK                                    (0xFF000000U)
#define CSL_PBIST_PBIST_RINFOU_RINFOU3_SHIFT                                   (0x00000018U)
#define CSL_PBIST_PBIST_RINFOU_RINFOU3_RESETVAL                                (0x000000FFU)
#define CSL_PBIST_PBIST_RINFOU_RINFOU3_MAX                                     (0x000000FFU)

#define CSL_PBIST_PBIST_RINFOU_RESETVAL                                        (0xFFFFFFFFU)

#ifdef __cplusplus
}
#endif
#endif
