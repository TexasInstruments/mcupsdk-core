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
 *  Name        : cslr_mss_mcrc.h
*/
#ifndef CSLR_MSS_MCRC_H_
#define CSLR_MSS_MCRC_H_

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
    volatile uint32_t CRC_CTRL0;
    volatile uint8_t  Resv_8[4];
    volatile uint32_t CRC_CTRL1;
    volatile uint8_t  Resv_16[4];
    volatile uint32_t CRC_CTRL2;
    volatile uint8_t  Resv_24[4];
    volatile uint32_t CRC_INTS;
    volatile uint8_t  Resv_32[4];
    volatile uint32_t CRC_INTR;
    volatile uint8_t  Resv_40[4];
    volatile uint32_t CRC_STATUS_REG;
    volatile uint8_t  Resv_48[4];
    volatile uint32_t CRC_INT_OFFSET_REG;
    volatile uint8_t  Resv_56[4];
    volatile uint32_t CRC_BUSY;
    volatile uint8_t  Resv_64[4];
    volatile uint32_t CRC_PCOUNT_REG1;
    volatile uint32_t CRC_SCOUNT_REG1;
    volatile uint32_t CRC_CURSEC_REG1;
    volatile uint32_t CRC_WDTOPLD1;
    volatile uint32_t CRC_BCTOPLD1;
    volatile uint8_t  Resv_96[12];
    volatile uint32_t PSA_SIGREGL1;
    volatile uint32_t PSA_SIGREGH1;
    volatile uint32_t CRC_REGL1;
    volatile uint32_t CRC_REGH1;
    volatile uint32_t PSA_SECSIGREGL1;
    volatile uint32_t PSA_SECSIGREGH1;
    volatile uint32_t RAW_DATAREGL1;
    volatile uint32_t RAW_DATAREGH1;
    volatile uint32_t CRC_PCOUNT_REG2;
    volatile uint32_t CRC_SCOUNT_REG2;
    volatile uint32_t CRC_CURSEC_REG2;
    volatile uint32_t CRC_WDTOPLD2;
    volatile uint32_t CRC_BCTOPLD2;
    volatile uint8_t  Resv_160[12];
    volatile uint32_t PSA_SIGREGL2;
    volatile uint32_t PSA_SIGREGH2;
    volatile uint32_t CRC_REGL2;
    volatile uint32_t CRC_REGH2;
    volatile uint32_t PSA_SECSIGREGL2;
    volatile uint32_t PSA_SECSIGREGH2;
    volatile uint32_t RAW_DATAREGL2;
    volatile uint32_t RAW_DATAREGH2;
    volatile uint32_t CRC_PCOUNT_REG3;
    volatile uint32_t CRC_SCOUNT_REG3;
    volatile uint32_t CRC_CURSEC_REG3;
    volatile uint32_t CRC_WDTOPLD3;
    volatile uint32_t CRC_BCTOPLD3;
    volatile uint8_t  Resv_224[12];
    volatile uint32_t PSA_SIGREGL3;
    volatile uint32_t PSA_SIGREGH3;
    volatile uint32_t CRC_REGL3;
    volatile uint32_t CRC_REGH3;
    volatile uint32_t PSA_SECSIGREGL3;
    volatile uint32_t PSA_SECSIGREGH3;
    volatile uint32_t RAW_DATAREGL3;
    volatile uint32_t RAW_DATAREGH3;
    volatile uint32_t CRC_PCOUNT_REG4;
    volatile uint32_t CRC_SCOUNT_REG4;
    volatile uint32_t CRC_CURSEC_REG4;
    volatile uint32_t CRC_WDTOPLD4;
    volatile uint32_t CRC_BCTOPLD4;
    volatile uint8_t  Resv_288[12];
    volatile uint32_t PSA_SIGREGL4;
    volatile uint32_t PSA_SIGREGH4;
    volatile uint32_t CRC_REGL4;
    volatile uint32_t CRC_REGH4;
    volatile uint32_t PSA_SECSIGREGL4;
    volatile uint32_t PSA_SECSIGREGH4;
    volatile uint32_t RAW_DATAREGL4;
    volatile uint32_t RAW_DATAREGH4;
    volatile uint32_t MCRC_BUS_SEL;
    volatile uint32_t MCRC_RESERVED;
} CSL_mss_mcrcRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_MSS_MCRC_CRC_CTRL0                                                 (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL1                                                 (0x00000008U)
#define CSL_MSS_MCRC_CRC_CTRL2                                                 (0x00000010U)
#define CSL_MSS_MCRC_CRC_INTS                                                  (0x00000018U)
#define CSL_MSS_MCRC_CRC_INTR                                                  (0x00000020U)
#define CSL_MSS_MCRC_CRC_STATUS_REG                                            (0x00000028U)
#define CSL_MSS_MCRC_CRC_INT_OFFSET_REG                                        (0x00000030U)
#define CSL_MSS_MCRC_CRC_BUSY                                                  (0x00000038U)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG1                                           (0x00000040U)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG1                                           (0x00000044U)
#define CSL_MSS_MCRC_CRC_CURSEC_REG1                                           (0x00000048U)
#define CSL_MSS_MCRC_CRC_WDTOPLD1                                              (0x0000004CU)
#define CSL_MSS_MCRC_CRC_BCTOPLD1                                              (0x00000050U)
#define CSL_MSS_MCRC_PSA_SIGREGL1                                              (0x00000060U)
#define CSL_MSS_MCRC_PSA_SIGREGH1                                              (0x00000064U)
#define CSL_MSS_MCRC_CRC_REGL1                                                 (0x00000068U)
#define CSL_MSS_MCRC_CRC_REGH1                                                 (0x0000006CU)
#define CSL_MSS_MCRC_PSA_SECSIGREGL1                                           (0x00000070U)
#define CSL_MSS_MCRC_PSA_SECSIGREGH1                                           (0x00000074U)
#define CSL_MSS_MCRC_RAW_DATAREGL1                                             (0x00000078U)
#define CSL_MSS_MCRC_RAW_DATAREGH1                                             (0x0000007CU)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG2                                           (0x00000080U)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG2                                           (0x00000084U)
#define CSL_MSS_MCRC_CRC_CURSEC_REG2                                           (0x00000088U)
#define CSL_MSS_MCRC_CRC_WDTOPLD2                                              (0x0000008CU)
#define CSL_MSS_MCRC_CRC_BCTOPLD2                                              (0x00000090U)
#define CSL_MSS_MCRC_PSA_SIGREGL2                                              (0x000000A0U)
#define CSL_MSS_MCRC_PSA_SIGREGH2                                              (0x000000A4U)
#define CSL_MSS_MCRC_CRC_REGL2                                                 (0x000000A8U)
#define CSL_MSS_MCRC_CRC_REGH2                                                 (0x000000ACU)
#define CSL_MSS_MCRC_PSA_SECSIGREGL2                                           (0x000000B0U)
#define CSL_MSS_MCRC_PSA_SECSIGREGH2                                           (0x000000B4U)
#define CSL_MSS_MCRC_RAW_DATAREGL2                                             (0x000000B8U)
#define CSL_MSS_MCRC_RAW_DATAREGH2                                             (0x000000BCU)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG3                                           (0x000000C0U)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG3                                           (0x000000C4U)
#define CSL_MSS_MCRC_CRC_CURSEC_REG3                                           (0x000000C8U)
#define CSL_MSS_MCRC_CRC_WDTOPLD3                                              (0x000000CCU)
#define CSL_MSS_MCRC_CRC_BCTOPLD3                                              (0x000000D0U)
#define CSL_MSS_MCRC_PSA_SIGREGL3                                              (0x000000E0U)
#define CSL_MSS_MCRC_PSA_SIGREGH3                                              (0x000000E4U)
#define CSL_MSS_MCRC_CRC_REGL3                                                 (0x000000E8U)
#define CSL_MSS_MCRC_CRC_REGH3                                                 (0x000000ECU)
#define CSL_MSS_MCRC_PSA_SECSIGREGL3                                           (0x000000F0U)
#define CSL_MSS_MCRC_PSA_SECSIGREGH3                                           (0x000000F4U)
#define CSL_MSS_MCRC_RAW_DATAREGL3                                             (0x000000F8U)
#define CSL_MSS_MCRC_RAW_DATAREGH3                                             (0x000000FCU)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG4                                           (0x00000100U)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG4                                           (0x00000104U)
#define CSL_MSS_MCRC_CRC_CURSEC_REG4                                           (0x00000108U)
#define CSL_MSS_MCRC_CRC_WDTOPLD4                                              (0x0000010CU)
#define CSL_MSS_MCRC_CRC_BCTOPLD4                                              (0x00000110U)
#define CSL_MSS_MCRC_PSA_SIGREGL4                                              (0x00000120U)
#define CSL_MSS_MCRC_PSA_SIGREGH4                                              (0x00000124U)
#define CSL_MSS_MCRC_CRC_REGL4                                                 (0x00000128U)
#define CSL_MSS_MCRC_CRC_REGH4                                                 (0x0000012CU)
#define CSL_MSS_MCRC_PSA_SECSIGREGL4                                           (0x00000130U)
#define CSL_MSS_MCRC_PSA_SECSIGREGH4                                           (0x00000134U)
#define CSL_MSS_MCRC_RAW_DATAREGL4                                             (0x00000138U)
#define CSL_MSS_MCRC_RAW_DATAREGH4                                             (0x0000013CU)
#define CSL_MSS_MCRC_MCRC_BUS_SEL                                              (0x00000140U)
#define CSL_MSS_MCRC_MCRC_RESERVED                                             (0x00000144U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* CRC_CTRL0 */

#define CSL_MSS_MCRC_CRC_CTRL0_CH1_PSA_SWREST_MASK                             (0x00000001U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH1_PSA_SWREST_SHIFT                            (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH1_PSA_SWREST_RESETVAL                         (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH1_PSA_SWREST_MAX                              (0x00000001U)

#define CSL_MSS_MCRC_CRC_CTRL0_CH1_DW_SEL_MASK                                 (0x00000006U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH1_DW_SEL_SHIFT                                (0x00000001U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH1_DW_SEL_RESETVAL                             (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH1_DW_SEL_MAX                                  (0x00000003U)

#define CSL_MSS_MCRC_CRC_CTRL0_CH1_CRC_SEL_MASK                                (0x00000018U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH1_CRC_SEL_SHIFT                               (0x00000003U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH1_CRC_SEL_RESETVAL                            (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH1_CRC_SEL_MAX                                 (0x00000003U)

#define CSL_MSS_MCRC_CRC_CTRL0_CH1_BIT_SWAP_MASK                               (0x00000020U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH1_BIT_SWAP_SHIFT                              (0x00000005U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH1_BIT_SWAP_RESETVAL                           (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH1_BIT_SWAP_MAX                                (0x00000001U)

#define CSL_MSS_MCRC_CRC_CTRL0_CH1_BYTE_SWAP_MASK                              (0x00000040U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH1_BYTE_SWAP_SHIFT                             (0x00000006U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH1_BYTE_SWAP_RESETVAL                          (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH1_BYTE_SWAP_MAX                               (0x00000001U)

#define CSL_MSS_MCRC_CRC_CTRL0_CH1_CRC_SEL2_MASK                               (0x00000080U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH1_CRC_SEL2_SHIFT                              (0x00000007U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH1_CRC_SEL2_RESETVAL                           (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH1_CRC_SEL2_MAX                                (0x00000001U)

#define CSL_MSS_MCRC_CRC_CTRL0_CH2_PSA_SWREST_MASK                             (0x00000100U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH2_PSA_SWREST_SHIFT                            (0x00000008U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH2_PSA_SWREST_RESETVAL                         (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH2_PSA_SWREST_MAX                              (0x00000001U)

#define CSL_MSS_MCRC_CRC_CTRL0_CH2_DW_SEL_MASK                                 (0x00000600U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH2_DW_SEL_SHIFT                                (0x00000009U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH2_DW_SEL_RESETVAL                             (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH2_DW_SEL_MAX                                  (0x00000003U)

#define CSL_MSS_MCRC_CRC_CTRL0_CH2_CRC_SEL_MASK                                (0x00001800U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH2_CRC_SEL_SHIFT                               (0x0000000BU)
#define CSL_MSS_MCRC_CRC_CTRL0_CH2_CRC_SEL_RESETVAL                            (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH2_CRC_SEL_MAX                                 (0x00000003U)

#define CSL_MSS_MCRC_CRC_CTRL0_CH2_BIT_SWAP_MASK                               (0x00002000U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH2_BIT_SWAP_SHIFT                              (0x0000000DU)
#define CSL_MSS_MCRC_CRC_CTRL0_CH2_BIT_SWAP_RESETVAL                           (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH2_BIT_SWAP_MAX                                (0x00000001U)

#define CSL_MSS_MCRC_CRC_CTRL0_CH2_BYTE_SWAP_MASK                              (0x00004000U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH2_BYTE_SWAP_SHIFT                             (0x0000000EU)
#define CSL_MSS_MCRC_CRC_CTRL0_CH2_BYTE_SWAP_RESETVAL                          (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH2_BYTE_SWAP_MAX                               (0x00000001U)

#define CSL_MSS_MCRC_CRC_CTRL0_CH2_CRC_SEL2_MASK                               (0x00008000U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH2_CRC_SEL2_SHIFT                              (0x0000000FU)
#define CSL_MSS_MCRC_CRC_CTRL0_CH2_CRC_SEL2_RESETVAL                           (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_CH2_CRC_SEL2_MAX                                (0x00000001U)

#define CSL_MSS_MCRC_CRC_CTRL0_NU1_MASK                                        (0x00010000U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU1_SHIFT                                       (0x00000010U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU1_RESETVAL                                    (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU1_MAX                                         (0x00000001U)

#define CSL_MSS_MCRC_CRC_CTRL0_NU2_MASK                                        (0x00060000U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU2_SHIFT                                       (0x00000011U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU2_RESETVAL                                    (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU2_MAX                                         (0x00000003U)

#define CSL_MSS_MCRC_CRC_CTRL0_NU3_MASK                                        (0x00180000U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU3_SHIFT                                       (0x00000013U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU3_RESETVAL                                    (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU3_MAX                                         (0x00000003U)

#define CSL_MSS_MCRC_CRC_CTRL0_NU4_MASK                                        (0x00200000U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU4_SHIFT                                       (0x00000015U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU4_RESETVAL                                    (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU4_MAX                                         (0x00000001U)

#define CSL_MSS_MCRC_CRC_CTRL0_NU5_MASK                                        (0x00400000U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU5_SHIFT                                       (0x00000016U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU5_RESETVAL                                    (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU5_MAX                                         (0x00000001U)

#define CSL_MSS_MCRC_CRC_CTRL0_NU6_MASK                                        (0x00800000U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU6_SHIFT                                       (0x00000017U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU6_RESETVAL                                    (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU6_MAX                                         (0x00000001U)

#define CSL_MSS_MCRC_CRC_CTRL0_NU7_MASK                                        (0x01000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU7_SHIFT                                       (0x00000018U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU7_RESETVAL                                    (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU7_MAX                                         (0x00000001U)

#define CSL_MSS_MCRC_CRC_CTRL0_NU8_MASK                                        (0x06000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU8_SHIFT                                       (0x00000019U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU8_RESETVAL                                    (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU8_MAX                                         (0x00000003U)

#define CSL_MSS_MCRC_CRC_CTRL0_NU9_MASK                                        (0x18000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU9_SHIFT                                       (0x0000001BU)
#define CSL_MSS_MCRC_CRC_CTRL0_NU9_RESETVAL                                    (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU9_MAX                                         (0x00000003U)

#define CSL_MSS_MCRC_CRC_CTRL0_NU10_MASK                                       (0x20000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU10_SHIFT                                      (0x0000001DU)
#define CSL_MSS_MCRC_CRC_CTRL0_NU10_RESETVAL                                   (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU10_MAX                                        (0x00000001U)

#define CSL_MSS_MCRC_CRC_CTRL0_NU11_MASK                                       (0x40000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU11_SHIFT                                      (0x0000001EU)
#define CSL_MSS_MCRC_CRC_CTRL0_NU11_RESETVAL                                   (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU11_MAX                                        (0x00000001U)

#define CSL_MSS_MCRC_CRC_CTRL0_NU12_MASK                                       (0x80000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU12_SHIFT                                      (0x0000001FU)
#define CSL_MSS_MCRC_CRC_CTRL0_NU12_RESETVAL                                   (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL0_NU12_MAX                                        (0x00000001U)

#define CSL_MSS_MCRC_CRC_CTRL0_RESETVAL                                        (0x00000000U)

/* CRC_CTRL1 */

#define CSL_MSS_MCRC_CRC_CTRL1_PWDN_MASK                                       (0x00000001U)
#define CSL_MSS_MCRC_CRC_CTRL1_PWDN_SHIFT                                      (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL1_PWDN_RESETVAL                                   (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL1_PWDN_MAX                                        (0x00000001U)

#define CSL_MSS_MCRC_CRC_CTRL1_RESERVED1_MASK                                  (0xFFFFFFFEU)
#define CSL_MSS_MCRC_CRC_CTRL1_RESERVED1_SHIFT                                 (0x00000001U)
#define CSL_MSS_MCRC_CRC_CTRL1_RESERVED1_RESETVAL                              (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL1_RESERVED1_MAX                                   (0x7FFFFFFFU)

#define CSL_MSS_MCRC_CRC_CTRL1_RESETVAL                                        (0x00000000U)

/* CRC_CTRL2 */

#define CSL_MSS_MCRC_CRC_CTRL2_CH1_MODE_MASK                                   (0x00000003U)
#define CSL_MSS_MCRC_CRC_CTRL2_CH1_MODE_SHIFT                                  (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL2_CH1_MODE_RESETVAL                               (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL2_CH1_MODE_MAX                                    (0x00000003U)

#define CSL_MSS_MCRC_CRC_CTRL2_RESERVED1_MASK                                  (0x0000000CU)
#define CSL_MSS_MCRC_CRC_CTRL2_RESERVED1_SHIFT                                 (0x00000002U)
#define CSL_MSS_MCRC_CRC_CTRL2_RESERVED1_RESETVAL                              (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL2_RESERVED1_MAX                                   (0x00000003U)

#define CSL_MSS_MCRC_CRC_CTRL2_CH1_TRACEEN_MASK                                (0x00000010U)
#define CSL_MSS_MCRC_CRC_CTRL2_CH1_TRACEEN_SHIFT                               (0x00000004U)
#define CSL_MSS_MCRC_CRC_CTRL2_CH1_TRACEEN_RESETVAL                            (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL2_CH1_TRACEEN_MAX                                 (0x00000001U)

#define CSL_MSS_MCRC_CRC_CTRL2_RESERVED2_MASK                                  (0x000000E0U)
#define CSL_MSS_MCRC_CRC_CTRL2_RESERVED2_SHIFT                                 (0x00000005U)
#define CSL_MSS_MCRC_CRC_CTRL2_RESERVED2_RESETVAL                              (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL2_RESERVED2_MAX                                   (0x00000007U)

#define CSL_MSS_MCRC_CRC_CTRL2_CH2_MODE_MASK                                   (0x00000300U)
#define CSL_MSS_MCRC_CRC_CTRL2_CH2_MODE_SHIFT                                  (0x00000008U)
#define CSL_MSS_MCRC_CRC_CTRL2_CH2_MODE_RESETVAL                               (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL2_CH2_MODE_MAX                                    (0x00000003U)

#define CSL_MSS_MCRC_CRC_CTRL2_RESERVED3_MASK                                  (0x0000FC00U)
#define CSL_MSS_MCRC_CRC_CTRL2_RESERVED3_SHIFT                                 (0x0000000AU)
#define CSL_MSS_MCRC_CRC_CTRL2_RESERVED3_RESETVAL                              (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL2_RESERVED3_MAX                                   (0x0000003FU)

#define CSL_MSS_MCRC_CRC_CTRL2_NU13_MASK                                       (0x00030000U)
#define CSL_MSS_MCRC_CRC_CTRL2_NU13_SHIFT                                      (0x00000010U)
#define CSL_MSS_MCRC_CRC_CTRL2_NU13_RESETVAL                                   (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL2_NU13_MAX                                        (0x00000003U)

#define CSL_MSS_MCRC_CRC_CTRL2_RESERVED4_MASK                                  (0x00FC0000U)
#define CSL_MSS_MCRC_CRC_CTRL2_RESERVED4_SHIFT                                 (0x00000012U)
#define CSL_MSS_MCRC_CRC_CTRL2_RESERVED4_RESETVAL                              (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL2_RESERVED4_MAX                                   (0x0000003FU)

#define CSL_MSS_MCRC_CRC_CTRL2_NU14_MASK                                       (0x03000000U)
#define CSL_MSS_MCRC_CRC_CTRL2_NU14_SHIFT                                      (0x00000018U)
#define CSL_MSS_MCRC_CRC_CTRL2_NU14_RESETVAL                                   (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL2_NU14_MAX                                        (0x00000003U)

#define CSL_MSS_MCRC_CRC_CTRL2_RESERVED5_MASK                                  (0xFC000000U)
#define CSL_MSS_MCRC_CRC_CTRL2_RESERVED5_SHIFT                                 (0x0000001AU)
#define CSL_MSS_MCRC_CRC_CTRL2_RESERVED5_RESETVAL                              (0x00000000U)
#define CSL_MSS_MCRC_CRC_CTRL2_RESERVED5_MAX                                   (0x0000003FU)

#define CSL_MSS_MCRC_CRC_CTRL2_RESETVAL                                        (0x00000000U)

/* CRC_INTS */

#define CSL_MSS_MCRC_CRC_INTS_RESERVED1_MASK                                   (0x00000001U)
#define CSL_MSS_MCRC_CRC_INTS_RESERVED1_SHIFT                                  (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTS_RESERVED1_RESETVAL                               (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTS_RESERVED1_MAX                                    (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTS_CH1_CRCFAILENS_MASK                              (0x00000002U)
#define CSL_MSS_MCRC_CRC_INTS_CH1_CRCFAILENS_SHIFT                             (0x00000001U)
#define CSL_MSS_MCRC_CRC_INTS_CH1_CRCFAILENS_RESETVAL                          (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTS_CH1_CRCFAILENS_MAX                               (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTS_CH1_OVERENS_MASK                                 (0x00000004U)
#define CSL_MSS_MCRC_CRC_INTS_CH1_OVERENS_SHIFT                                (0x00000002U)
#define CSL_MSS_MCRC_CRC_INTS_CH1_OVERENS_RESETVAL                             (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTS_CH1_OVERENS_MAX                                  (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTS_CH1_UNDERENS_MASK                                (0x00000008U)
#define CSL_MSS_MCRC_CRC_INTS_CH1_UNDERENS_SHIFT                               (0x00000003U)
#define CSL_MSS_MCRC_CRC_INTS_CH1_UNDERENS_RESETVAL                            (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTS_CH1_UNDERENS_MAX                                 (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTS_CH1_TIMEOUTENS_MASK                              (0x00000010U)
#define CSL_MSS_MCRC_CRC_INTS_CH1_TIMEOUTENS_SHIFT                             (0x00000004U)
#define CSL_MSS_MCRC_CRC_INTS_CH1_TIMEOUTENS_RESETVAL                          (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTS_CH1_TIMEOUTENS_MAX                               (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTS_RESERVED2_MASK                                   (0x000001E0U)
#define CSL_MSS_MCRC_CRC_INTS_RESERVED2_SHIFT                                  (0x00000005U)
#define CSL_MSS_MCRC_CRC_INTS_RESERVED2_RESETVAL                               (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTS_RESERVED2_MAX                                    (0x0000000FU)

#define CSL_MSS_MCRC_CRC_INTS_CH2_CRCFAILENS_MASK                              (0x00000200U)
#define CSL_MSS_MCRC_CRC_INTS_CH2_CRCFAILENS_SHIFT                             (0x00000009U)
#define CSL_MSS_MCRC_CRC_INTS_CH2_CRCFAILENS_RESETVAL                          (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTS_CH2_CRCFAILENS_MAX                               (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTS_CH2_OVERENS_MASK                                 (0x00000400U)
#define CSL_MSS_MCRC_CRC_INTS_CH2_OVERENS_SHIFT                                (0x0000000AU)
#define CSL_MSS_MCRC_CRC_INTS_CH2_OVERENS_RESETVAL                             (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTS_CH2_OVERENS_MAX                                  (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTS_CH2_UNDERENS_MASK                                (0x00000800U)
#define CSL_MSS_MCRC_CRC_INTS_CH2_UNDERENS_SHIFT                               (0x0000000BU)
#define CSL_MSS_MCRC_CRC_INTS_CH2_UNDERENS_RESETVAL                            (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTS_CH2_UNDERENS_MAX                                 (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTS_CH2_TIMEOUTENS_MASK                              (0x00001000U)
#define CSL_MSS_MCRC_CRC_INTS_CH2_TIMEOUTENS_SHIFT                             (0x0000000CU)
#define CSL_MSS_MCRC_CRC_INTS_CH2_TIMEOUTENS_RESETVAL                          (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTS_CH2_TIMEOUTENS_MAX                               (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTS_RESERVED3_MASK                                   (0x0001E000U)
#define CSL_MSS_MCRC_CRC_INTS_RESERVED3_SHIFT                                  (0x0000000DU)
#define CSL_MSS_MCRC_CRC_INTS_RESERVED3_RESETVAL                               (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTS_RESERVED3_MAX                                    (0x0000000FU)

#define CSL_MSS_MCRC_CRC_INTS_NU15_MASK                                        (0x00020000U)
#define CSL_MSS_MCRC_CRC_INTS_NU15_SHIFT                                       (0x00000011U)
#define CSL_MSS_MCRC_CRC_INTS_NU15_RESETVAL                                    (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTS_NU15_MAX                                         (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTS_NU16_MASK                                        (0x00040000U)
#define CSL_MSS_MCRC_CRC_INTS_NU16_SHIFT                                       (0x00000012U)
#define CSL_MSS_MCRC_CRC_INTS_NU16_RESETVAL                                    (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTS_NU16_MAX                                         (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTS_NU17_MASK                                        (0x00080000U)
#define CSL_MSS_MCRC_CRC_INTS_NU17_SHIFT                                       (0x00000013U)
#define CSL_MSS_MCRC_CRC_INTS_NU17_RESETVAL                                    (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTS_NU17_MAX                                         (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTS_NU18_MASK                                        (0x00100000U)
#define CSL_MSS_MCRC_CRC_INTS_NU18_SHIFT                                       (0x00000014U)
#define CSL_MSS_MCRC_CRC_INTS_NU18_RESETVAL                                    (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTS_NU18_MAX                                         (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTS_RESERVED4_MASK                                   (0x01E00000U)
#define CSL_MSS_MCRC_CRC_INTS_RESERVED4_SHIFT                                  (0x00000015U)
#define CSL_MSS_MCRC_CRC_INTS_RESERVED4_RESETVAL                               (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTS_RESERVED4_MAX                                    (0x0000000FU)

#define CSL_MSS_MCRC_CRC_INTS_NU19_MASK                                        (0x02000000U)
#define CSL_MSS_MCRC_CRC_INTS_NU19_SHIFT                                       (0x00000019U)
#define CSL_MSS_MCRC_CRC_INTS_NU19_RESETVAL                                    (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTS_NU19_MAX                                         (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTS_NU20_MASK                                        (0x04000000U)
#define CSL_MSS_MCRC_CRC_INTS_NU20_SHIFT                                       (0x0000001AU)
#define CSL_MSS_MCRC_CRC_INTS_NU20_RESETVAL                                    (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTS_NU20_MAX                                         (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTS_NU21_MASK                                        (0x08000000U)
#define CSL_MSS_MCRC_CRC_INTS_NU21_SHIFT                                       (0x0000001BU)
#define CSL_MSS_MCRC_CRC_INTS_NU21_RESETVAL                                    (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTS_NU21_MAX                                         (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTS_NU22_MASK                                        (0x10000000U)
#define CSL_MSS_MCRC_CRC_INTS_NU22_SHIFT                                       (0x0000001CU)
#define CSL_MSS_MCRC_CRC_INTS_NU22_RESETVAL                                    (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTS_NU22_MAX                                         (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTS_RESERVED5_MASK                                   (0xE0000000U)
#define CSL_MSS_MCRC_CRC_INTS_RESERVED5_SHIFT                                  (0x0000001DU)
#define CSL_MSS_MCRC_CRC_INTS_RESERVED5_RESETVAL                               (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTS_RESERVED5_MAX                                    (0x00000007U)

#define CSL_MSS_MCRC_CRC_INTS_RESETVAL                                         (0x00000000U)

/* CRC_INTR */

#define CSL_MSS_MCRC_CRC_INTR_RESERVED1_MASK                                   (0x00000001U)
#define CSL_MSS_MCRC_CRC_INTR_RESERVED1_SHIFT                                  (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTR_RESERVED1_RESETVAL                               (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTR_RESERVED1_MAX                                    (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTR_CH1_CRCFAILENR_MASK                              (0x00000002U)
#define CSL_MSS_MCRC_CRC_INTR_CH1_CRCFAILENR_SHIFT                             (0x00000001U)
#define CSL_MSS_MCRC_CRC_INTR_CH1_CRCFAILENR_RESETVAL                          (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTR_CH1_CRCFAILENR_MAX                               (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTR_CH1_OVERENR_MASK                                 (0x00000004U)
#define CSL_MSS_MCRC_CRC_INTR_CH1_OVERENR_SHIFT                                (0x00000002U)
#define CSL_MSS_MCRC_CRC_INTR_CH1_OVERENR_RESETVAL                             (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTR_CH1_OVERENR_MAX                                  (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTR_CH1_UNDERENR_MASK                                (0x00000008U)
#define CSL_MSS_MCRC_CRC_INTR_CH1_UNDERENR_SHIFT                               (0x00000003U)
#define CSL_MSS_MCRC_CRC_INTR_CH1_UNDERENR_RESETVAL                            (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTR_CH1_UNDERENR_MAX                                 (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTR_CH1_TIMEOUTENR_MASK                              (0x00000010U)
#define CSL_MSS_MCRC_CRC_INTR_CH1_TIMEOUTENR_SHIFT                             (0x00000004U)
#define CSL_MSS_MCRC_CRC_INTR_CH1_TIMEOUTENR_RESETVAL                          (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTR_CH1_TIMEOUTENR_MAX                               (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTR_RESERVED2_MASK                                   (0x000001E0U)
#define CSL_MSS_MCRC_CRC_INTR_RESERVED2_SHIFT                                  (0x00000005U)
#define CSL_MSS_MCRC_CRC_INTR_RESERVED2_RESETVAL                               (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTR_RESERVED2_MAX                                    (0x0000000FU)

#define CSL_MSS_MCRC_CRC_INTR_CH2_CRCFAILENR_MASK                              (0x00000200U)
#define CSL_MSS_MCRC_CRC_INTR_CH2_CRCFAILENR_SHIFT                             (0x00000009U)
#define CSL_MSS_MCRC_CRC_INTR_CH2_CRCFAILENR_RESETVAL                          (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTR_CH2_CRCFAILENR_MAX                               (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTR_CH2_OVERENR_MASK                                 (0x00000400U)
#define CSL_MSS_MCRC_CRC_INTR_CH2_OVERENR_SHIFT                                (0x0000000AU)
#define CSL_MSS_MCRC_CRC_INTR_CH2_OVERENR_RESETVAL                             (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTR_CH2_OVERENR_MAX                                  (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTR_CH2_UNDERENR_MASK                                (0x00000800U)
#define CSL_MSS_MCRC_CRC_INTR_CH2_UNDERENR_SHIFT                               (0x0000000BU)
#define CSL_MSS_MCRC_CRC_INTR_CH2_UNDERENR_RESETVAL                            (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTR_CH2_UNDERENR_MAX                                 (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTR_CH2_TIMEOUTENR_MASK                              (0x00001000U)
#define CSL_MSS_MCRC_CRC_INTR_CH2_TIMEOUTENR_SHIFT                             (0x0000000CU)
#define CSL_MSS_MCRC_CRC_INTR_CH2_TIMEOUTENR_RESETVAL                          (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTR_CH2_TIMEOUTENR_MAX                               (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTR_RESERVED3_MASK                                   (0x0001E000U)
#define CSL_MSS_MCRC_CRC_INTR_RESERVED3_SHIFT                                  (0x0000000DU)
#define CSL_MSS_MCRC_CRC_INTR_RESERVED3_RESETVAL                               (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTR_RESERVED3_MAX                                    (0x0000000FU)

#define CSL_MSS_MCRC_CRC_INTR_NU23_MASK                                        (0x00020000U)
#define CSL_MSS_MCRC_CRC_INTR_NU23_SHIFT                                       (0x00000011U)
#define CSL_MSS_MCRC_CRC_INTR_NU23_RESETVAL                                    (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTR_NU23_MAX                                         (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTR_NU24_MASK                                        (0x00040000U)
#define CSL_MSS_MCRC_CRC_INTR_NU24_SHIFT                                       (0x00000012U)
#define CSL_MSS_MCRC_CRC_INTR_NU24_RESETVAL                                    (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTR_NU24_MAX                                         (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTR_NU25_MASK                                        (0x00080000U)
#define CSL_MSS_MCRC_CRC_INTR_NU25_SHIFT                                       (0x00000013U)
#define CSL_MSS_MCRC_CRC_INTR_NU25_RESETVAL                                    (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTR_NU25_MAX                                         (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTR_NU26_MASK                                        (0x00100000U)
#define CSL_MSS_MCRC_CRC_INTR_NU26_SHIFT                                       (0x00000014U)
#define CSL_MSS_MCRC_CRC_INTR_NU26_RESETVAL                                    (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTR_NU26_MAX                                         (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTR_RESERVED4_MASK                                   (0x01E00000U)
#define CSL_MSS_MCRC_CRC_INTR_RESERVED4_SHIFT                                  (0x00000015U)
#define CSL_MSS_MCRC_CRC_INTR_RESERVED4_RESETVAL                               (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTR_RESERVED4_MAX                                    (0x0000000FU)

#define CSL_MSS_MCRC_CRC_INTR_NU27_MASK                                        (0x02000000U)
#define CSL_MSS_MCRC_CRC_INTR_NU27_SHIFT                                       (0x00000019U)
#define CSL_MSS_MCRC_CRC_INTR_NU27_RESETVAL                                    (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTR_NU27_MAX                                         (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTR_NU28_MASK                                        (0x04000000U)
#define CSL_MSS_MCRC_CRC_INTR_NU28_SHIFT                                       (0x0000001AU)
#define CSL_MSS_MCRC_CRC_INTR_NU28_RESETVAL                                    (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTR_NU28_MAX                                         (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTR_NU29_MASK                                        (0x08000000U)
#define CSL_MSS_MCRC_CRC_INTR_NU29_SHIFT                                       (0x0000001BU)
#define CSL_MSS_MCRC_CRC_INTR_NU29_RESETVAL                                    (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTR_NU29_MAX                                         (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTR_NU30_MASK                                        (0x10000000U)
#define CSL_MSS_MCRC_CRC_INTR_NU30_SHIFT                                       (0x0000001CU)
#define CSL_MSS_MCRC_CRC_INTR_NU30_RESETVAL                                    (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTR_NU30_MAX                                         (0x00000001U)

#define CSL_MSS_MCRC_CRC_INTR_RESERVED5_MASK                                   (0xE0000000U)
#define CSL_MSS_MCRC_CRC_INTR_RESERVED5_SHIFT                                  (0x0000001DU)
#define CSL_MSS_MCRC_CRC_INTR_RESERVED5_RESETVAL                               (0x00000000U)
#define CSL_MSS_MCRC_CRC_INTR_RESERVED5_MAX                                    (0x00000007U)

#define CSL_MSS_MCRC_CRC_INTR_RESETVAL                                         (0x00000000U)

/* CRC_STATUS_REG */

#define CSL_MSS_MCRC_CRC_STATUS_REG_RESERVED1_MASK                             (0x00000001U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_RESERVED1_SHIFT                            (0x00000000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_RESERVED1_RESETVAL                         (0x00000000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_RESERVED1_MAX                              (0x00000001U)

#define CSL_MSS_MCRC_CRC_STATUS_REG_CH1_CRCFAIL_MASK                           (0x00000002U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_CH1_CRCFAIL_SHIFT                          (0x00000001U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_CH1_CRCFAIL_RESETVAL                       (0x00000000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_CH1_CRCFAIL_MAX                            (0x00000001U)

#define CSL_MSS_MCRC_CRC_STATUS_REG_CH1_OVER_MASK                              (0x00000004U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_CH1_OVER_SHIFT                             (0x00000002U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_CH1_OVER_RESETVAL                          (0x00000000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_CH1_OVER_MAX                               (0x00000001U)

#define CSL_MSS_MCRC_CRC_STATUS_REG_CH1_UNDER_MASK                             (0x00000008U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_CH1_UNDER_SHIFT                            (0x00000003U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_CH1_UNDER_RESETVAL                         (0x00000000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_CH1_UNDER_MAX                              (0x00000001U)

#define CSL_MSS_MCRC_CRC_STATUS_REG_CH1_TIMEOUT_MASK                           (0x00000010U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_CH1_TIMEOUT_SHIFT                          (0x00000004U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_CH1_TIMEOUT_RESETVAL                       (0x00000000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_CH1_TIMEOUT_MAX                            (0x00000001U)

#define CSL_MSS_MCRC_CRC_STATUS_REG_RESERVED2_MASK                             (0x000001E0U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_RESERVED2_SHIFT                            (0x00000005U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_RESERVED2_RESETVAL                         (0x00000000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_RESERVED2_MAX                              (0x0000000FU)

#define CSL_MSS_MCRC_CRC_STATUS_REG_CH2_CRCFAIL_MASK                           (0x00000200U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_CH2_CRCFAIL_SHIFT                          (0x00000009U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_CH2_CRCFAIL_RESETVAL                       (0x00000000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_CH2_CRCFAIL_MAX                            (0x00000001U)

#define CSL_MSS_MCRC_CRC_STATUS_REG_CH2_OVER_MASK                              (0x00000400U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_CH2_OVER_SHIFT                             (0x0000000AU)
#define CSL_MSS_MCRC_CRC_STATUS_REG_CH2_OVER_RESETVAL                          (0x00000000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_CH2_OVER_MAX                               (0x00000001U)

#define CSL_MSS_MCRC_CRC_STATUS_REG_CH2_UNDER_MASK                             (0x00000800U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_CH2_UNDER_SHIFT                            (0x0000000BU)
#define CSL_MSS_MCRC_CRC_STATUS_REG_CH2_UNDER_RESETVAL                         (0x00000000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_CH2_UNDER_MAX                              (0x00000001U)

#define CSL_MSS_MCRC_CRC_STATUS_REG_CH2_TIMEOUT_MASK                           (0x00001000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_CH2_TIMEOUT_SHIFT                          (0x0000000CU)
#define CSL_MSS_MCRC_CRC_STATUS_REG_CH2_TIMEOUT_RESETVAL                       (0x00000000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_CH2_TIMEOUT_MAX                            (0x00000001U)

#define CSL_MSS_MCRC_CRC_STATUS_REG_RESERVED3_MASK                             (0x0001E000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_RESERVED3_SHIFT                            (0x0000000DU)
#define CSL_MSS_MCRC_CRC_STATUS_REG_RESERVED3_RESETVAL                         (0x00000000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_RESERVED3_MAX                              (0x0000000FU)

#define CSL_MSS_MCRC_CRC_STATUS_REG_NU31_MASK                                  (0x00020000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_NU31_SHIFT                                 (0x00000011U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_NU31_RESETVAL                              (0x00000000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_NU31_MAX                                   (0x00000001U)

#define CSL_MSS_MCRC_CRC_STATUS_REG_NU32_MASK                                  (0x00040000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_NU32_SHIFT                                 (0x00000012U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_NU32_RESETVAL                              (0x00000000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_NU32_MAX                                   (0x00000001U)

#define CSL_MSS_MCRC_CRC_STATUS_REG_NU33_MASK                                  (0x00080000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_NU33_SHIFT                                 (0x00000013U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_NU33_RESETVAL                              (0x00000000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_NU33_MAX                                   (0x00000001U)

#define CSL_MSS_MCRC_CRC_STATUS_REG_NU34_MASK                                  (0x00100000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_NU34_SHIFT                                 (0x00000014U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_NU34_RESETVAL                              (0x00000000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_NU34_MAX                                   (0x00000001U)

#define CSL_MSS_MCRC_CRC_STATUS_REG_RESERVED4_MASK                             (0x01E00000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_RESERVED4_SHIFT                            (0x00000015U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_RESERVED4_RESETVAL                         (0x00000000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_RESERVED4_MAX                              (0x0000000FU)

#define CSL_MSS_MCRC_CRC_STATUS_REG_NU35_MASK                                  (0x02000000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_NU35_SHIFT                                 (0x00000019U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_NU35_RESETVAL                              (0x00000000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_NU35_MAX                                   (0x00000001U)

#define CSL_MSS_MCRC_CRC_STATUS_REG_NU36_MASK                                  (0x04000000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_NU36_SHIFT                                 (0x0000001AU)
#define CSL_MSS_MCRC_CRC_STATUS_REG_NU36_RESETVAL                              (0x00000000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_NU36_MAX                                   (0x00000001U)

#define CSL_MSS_MCRC_CRC_STATUS_REG_NU37_MASK                                  (0x08000000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_NU37_SHIFT                                 (0x0000001BU)
#define CSL_MSS_MCRC_CRC_STATUS_REG_NU37_RESETVAL                              (0x00000000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_NU37_MAX                                   (0x00000001U)

#define CSL_MSS_MCRC_CRC_STATUS_REG_NU38_MASK                                  (0x10000000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_NU38_SHIFT                                 (0x0000001CU)
#define CSL_MSS_MCRC_CRC_STATUS_REG_NU38_RESETVAL                              (0x00000000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_NU38_MAX                                   (0x00000001U)

#define CSL_MSS_MCRC_CRC_STATUS_REG_RESERVED5_MASK                             (0xE0000000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_RESERVED5_SHIFT                            (0x0000001DU)
#define CSL_MSS_MCRC_CRC_STATUS_REG_RESERVED5_RESETVAL                         (0x00000000U)
#define CSL_MSS_MCRC_CRC_STATUS_REG_RESERVED5_MAX                              (0x00000007U)

#define CSL_MSS_MCRC_CRC_STATUS_REG_RESETVAL                                   (0x00000000U)

/* CRC_INT_OFFSET_REG */

#define CSL_MSS_MCRC_CRC_INT_OFFSET_REG_OFSTREG_MASK                           (0x000000FFU)
#define CSL_MSS_MCRC_CRC_INT_OFFSET_REG_OFSTREG_SHIFT                          (0x00000000U)
#define CSL_MSS_MCRC_CRC_INT_OFFSET_REG_OFSTREG_RESETVAL                       (0x00000000U)
#define CSL_MSS_MCRC_CRC_INT_OFFSET_REG_OFSTREG_MAX                            (0x000000FFU)

#define CSL_MSS_MCRC_CRC_INT_OFFSET_REG_RESERVED1_MASK                         (0xFFFFFF00U)
#define CSL_MSS_MCRC_CRC_INT_OFFSET_REG_RESERVED1_SHIFT                        (0x00000008U)
#define CSL_MSS_MCRC_CRC_INT_OFFSET_REG_RESERVED1_RESETVAL                     (0x00000000U)
#define CSL_MSS_MCRC_CRC_INT_OFFSET_REG_RESERVED1_MAX                          (0x00FFFFFFU)

#define CSL_MSS_MCRC_CRC_INT_OFFSET_REG_RESETVAL                               (0x00000000U)

/* CRC_BUSY */

#define CSL_MSS_MCRC_CRC_BUSY_CH1_BUSY_MASK                                    (0x00000001U)
#define CSL_MSS_MCRC_CRC_BUSY_CH1_BUSY_SHIFT                                   (0x00000000U)
#define CSL_MSS_MCRC_CRC_BUSY_CH1_BUSY_RESETVAL                                (0x00000000U)
#define CSL_MSS_MCRC_CRC_BUSY_CH1_BUSY_MAX                                     (0x00000001U)

#define CSL_MSS_MCRC_CRC_BUSY_RESERVED1_MASK                                   (0x000000FEU)
#define CSL_MSS_MCRC_CRC_BUSY_RESERVED1_SHIFT                                  (0x00000001U)
#define CSL_MSS_MCRC_CRC_BUSY_RESERVED1_RESETVAL                               (0x00000000U)
#define CSL_MSS_MCRC_CRC_BUSY_RESERVED1_MAX                                    (0x0000007FU)

#define CSL_MSS_MCRC_CRC_BUSY_CH2_BUSY_MASK                                    (0x00000100U)
#define CSL_MSS_MCRC_CRC_BUSY_CH2_BUSY_SHIFT                                   (0x00000008U)
#define CSL_MSS_MCRC_CRC_BUSY_CH2_BUSY_RESETVAL                                (0x00000000U)
#define CSL_MSS_MCRC_CRC_BUSY_CH2_BUSY_MAX                                     (0x00000001U)

#define CSL_MSS_MCRC_CRC_BUSY_RESERVED2_MASK                                   (0x0000FE00U)
#define CSL_MSS_MCRC_CRC_BUSY_RESERVED2_SHIFT                                  (0x00000009U)
#define CSL_MSS_MCRC_CRC_BUSY_RESERVED2_RESETVAL                               (0x00000000U)
#define CSL_MSS_MCRC_CRC_BUSY_RESERVED2_MAX                                    (0x0000007FU)

#define CSL_MSS_MCRC_CRC_BUSY_NU39_MASK                                        (0x00010000U)
#define CSL_MSS_MCRC_CRC_BUSY_NU39_SHIFT                                       (0x00000010U)
#define CSL_MSS_MCRC_CRC_BUSY_NU39_RESETVAL                                    (0x00000000U)
#define CSL_MSS_MCRC_CRC_BUSY_NU39_MAX                                         (0x00000001U)

#define CSL_MSS_MCRC_CRC_BUSY_RESERVED3_MASK                                   (0x00FE0000U)
#define CSL_MSS_MCRC_CRC_BUSY_RESERVED3_SHIFT                                  (0x00000011U)
#define CSL_MSS_MCRC_CRC_BUSY_RESERVED3_RESETVAL                               (0x00000000U)
#define CSL_MSS_MCRC_CRC_BUSY_RESERVED3_MAX                                    (0x0000007FU)

#define CSL_MSS_MCRC_CRC_BUSY_NU40_MASK                                        (0x01000000U)
#define CSL_MSS_MCRC_CRC_BUSY_NU40_SHIFT                                       (0x00000018U)
#define CSL_MSS_MCRC_CRC_BUSY_NU40_RESETVAL                                    (0x00000000U)
#define CSL_MSS_MCRC_CRC_BUSY_NU40_MAX                                         (0x00000001U)

#define CSL_MSS_MCRC_CRC_BUSY_RESERVED4_MASK                                   (0xFE000000U)
#define CSL_MSS_MCRC_CRC_BUSY_RESERVED4_SHIFT                                  (0x00000019U)
#define CSL_MSS_MCRC_CRC_BUSY_RESERVED4_RESETVAL                               (0x00000000U)
#define CSL_MSS_MCRC_CRC_BUSY_RESERVED4_MAX                                    (0x0000007FU)

#define CSL_MSS_MCRC_CRC_BUSY_RESETVAL                                         (0x00000000U)

/* CRC_PCOUNT_REG1 */

#define CSL_MSS_MCRC_CRC_PCOUNT_REG1_CRC_PAT_COUNT1_MASK                       (0x000FFFFFU)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG1_CRC_PAT_COUNT1_SHIFT                      (0x00000000U)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG1_CRC_PAT_COUNT1_RESETVAL                   (0x00000000U)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG1_CRC_PAT_COUNT1_MAX                        (0x000FFFFFU)

#define CSL_MSS_MCRC_CRC_PCOUNT_REG1_RESERVED1_MASK                            (0xFFF00000U)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG1_RESERVED1_SHIFT                           (0x00000014U)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG1_RESERVED1_RESETVAL                        (0x00000000U)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG1_RESERVED1_MAX                             (0x00000FFFU)

#define CSL_MSS_MCRC_CRC_PCOUNT_REG1_RESETVAL                                  (0x00000000U)

/* CRC_SCOUNT_REG1 */

#define CSL_MSS_MCRC_CRC_SCOUNT_REG1_CRC_SEC_COUNT1_MASK                       (0x0000FFFFU)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG1_CRC_SEC_COUNT1_SHIFT                      (0x00000000U)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG1_CRC_SEC_COUNT1_RESETVAL                   (0x00000000U)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG1_CRC_SEC_COUNT1_MAX                        (0x0000FFFFU)

#define CSL_MSS_MCRC_CRC_SCOUNT_REG1_RESERVED1_MASK                            (0xFFFF0000U)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG1_RESERVED1_SHIFT                           (0x00000010U)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG1_RESERVED1_RESETVAL                        (0x00000000U)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG1_RESERVED1_MAX                             (0x0000FFFFU)

#define CSL_MSS_MCRC_CRC_SCOUNT_REG1_RESETVAL                                  (0x00000000U)

/* CRC_CURSEC_REG1 */

#define CSL_MSS_MCRC_CRC_CURSEC_REG1_CRC_CURSEC1_MASK                          (0x0000FFFFU)
#define CSL_MSS_MCRC_CRC_CURSEC_REG1_CRC_CURSEC1_SHIFT                         (0x00000000U)
#define CSL_MSS_MCRC_CRC_CURSEC_REG1_CRC_CURSEC1_RESETVAL                      (0x00000000U)
#define CSL_MSS_MCRC_CRC_CURSEC_REG1_CRC_CURSEC1_MAX                           (0x0000FFFFU)

#define CSL_MSS_MCRC_CRC_CURSEC_REG1_RESERVED1_MASK                            (0xFFFF0000U)
#define CSL_MSS_MCRC_CRC_CURSEC_REG1_RESERVED1_SHIFT                           (0x00000010U)
#define CSL_MSS_MCRC_CRC_CURSEC_REG1_RESERVED1_RESETVAL                        (0x00000000U)
#define CSL_MSS_MCRC_CRC_CURSEC_REG1_RESERVED1_MAX                             (0x0000FFFFU)

#define CSL_MSS_MCRC_CRC_CURSEC_REG1_RESETVAL                                  (0x00000000U)

/* CRC_WDTOPLD1 */

#define CSL_MSS_MCRC_CRC_WDTOPLD1_CRC_WDTOPLD1_MASK                            (0x00FFFFFFU)
#define CSL_MSS_MCRC_CRC_WDTOPLD1_CRC_WDTOPLD1_SHIFT                           (0x00000000U)
#define CSL_MSS_MCRC_CRC_WDTOPLD1_CRC_WDTOPLD1_RESETVAL                        (0x00000000U)
#define CSL_MSS_MCRC_CRC_WDTOPLD1_CRC_WDTOPLD1_MAX                             (0x00FFFFFFU)

#define CSL_MSS_MCRC_CRC_WDTOPLD1_RESERVED1_MASK                               (0xFF000000U)
#define CSL_MSS_MCRC_CRC_WDTOPLD1_RESERVED1_SHIFT                              (0x00000018U)
#define CSL_MSS_MCRC_CRC_WDTOPLD1_RESERVED1_RESETVAL                           (0x00000000U)
#define CSL_MSS_MCRC_CRC_WDTOPLD1_RESERVED1_MAX                                (0x000000FFU)

#define CSL_MSS_MCRC_CRC_WDTOPLD1_RESETVAL                                     (0x00000000U)

/* CRC_BCTOPLD1 */

#define CSL_MSS_MCRC_CRC_BCTOPLD1_CRC_BCTOPLD1_MASK                            (0x00FFFFFFU)
#define CSL_MSS_MCRC_CRC_BCTOPLD1_CRC_BCTOPLD1_SHIFT                           (0x00000000U)
#define CSL_MSS_MCRC_CRC_BCTOPLD1_CRC_BCTOPLD1_RESETVAL                        (0x00000000U)
#define CSL_MSS_MCRC_CRC_BCTOPLD1_CRC_BCTOPLD1_MAX                             (0x00FFFFFFU)

#define CSL_MSS_MCRC_CRC_BCTOPLD1_RESERVED1_MASK                               (0xFF000000U)
#define CSL_MSS_MCRC_CRC_BCTOPLD1_RESERVED1_SHIFT                              (0x00000018U)
#define CSL_MSS_MCRC_CRC_BCTOPLD1_RESERVED1_RESETVAL                           (0x00000000U)
#define CSL_MSS_MCRC_CRC_BCTOPLD1_RESERVED1_MAX                                (0x000000FFU)

#define CSL_MSS_MCRC_CRC_BCTOPLD1_RESETVAL                                     (0x00000000U)

/* PSA_SIGREGL1 */

#define CSL_MSS_MCRC_PSA_SIGREGL1_PSASIG1_31_0_MASK                            (0xFFFFFFFFU)
#define CSL_MSS_MCRC_PSA_SIGREGL1_PSASIG1_31_0_SHIFT                           (0x00000000U)
#define CSL_MSS_MCRC_PSA_SIGREGL1_PSASIG1_31_0_RESETVAL                        (0x00000000U)
#define CSL_MSS_MCRC_PSA_SIGREGL1_PSASIG1_31_0_MAX                             (0xFFFFFFFFU)

#define CSL_MSS_MCRC_PSA_SIGREGL1_RESETVAL                                     (0x00000000U)

/* PSA_SIGREGH1 */

#define CSL_MSS_MCRC_PSA_SIGREGH1_PSA_SIG1_63_32_MASK                          (0xFFFFFFFFU)
#define CSL_MSS_MCRC_PSA_SIGREGH1_PSA_SIG1_63_32_SHIFT                         (0x00000000U)
#define CSL_MSS_MCRC_PSA_SIGREGH1_PSA_SIG1_63_32_RESETVAL                      (0x00000000U)
#define CSL_MSS_MCRC_PSA_SIGREGH1_PSA_SIG1_63_32_MAX                           (0xFFFFFFFFU)

#define CSL_MSS_MCRC_PSA_SIGREGH1_RESETVAL                                     (0x00000000U)

/* CRC_REGL1 */

#define CSL_MSS_MCRC_CRC_REGL1_CRC1_31_0_MASK                                  (0xFFFFFFFFU)
#define CSL_MSS_MCRC_CRC_REGL1_CRC1_31_0_SHIFT                                 (0x00000000U)
#define CSL_MSS_MCRC_CRC_REGL1_CRC1_31_0_RESETVAL                              (0x00000000U)
#define CSL_MSS_MCRC_CRC_REGL1_CRC1_31_0_MAX                                   (0xFFFFFFFFU)

#define CSL_MSS_MCRC_CRC_REGL1_RESETVAL                                        (0x00000000U)

/* CRC_REGH1 */

#define CSL_MSS_MCRC_CRC_REGH1_CRC1_63_32_MASK                                 (0xFFFFFFFFU)
#define CSL_MSS_MCRC_CRC_REGH1_CRC1_63_32_SHIFT                                (0x00000000U)
#define CSL_MSS_MCRC_CRC_REGH1_CRC1_63_32_RESETVAL                             (0x00000000U)
#define CSL_MSS_MCRC_CRC_REGH1_CRC1_63_32_MAX                                  (0xFFFFFFFFU)

#define CSL_MSS_MCRC_CRC_REGH1_RESETVAL                                        (0x00000000U)

/* PSA_SECSIGREGL1 */

#define CSL_MSS_MCRC_PSA_SECSIGREGL1_PSASECSIG1_31_0_MASK                      (0xFFFFFFFFU)
#define CSL_MSS_MCRC_PSA_SECSIGREGL1_PSASECSIG1_31_0_SHIFT                     (0x00000000U)
#define CSL_MSS_MCRC_PSA_SECSIGREGL1_PSASECSIG1_31_0_RESETVAL                  (0x00000000U)
#define CSL_MSS_MCRC_PSA_SECSIGREGL1_PSASECSIG1_31_0_MAX                       (0xFFFFFFFFU)

#define CSL_MSS_MCRC_PSA_SECSIGREGL1_RESETVAL                                  (0x00000000U)

/* PSA_SECSIGREGH1 */

#define CSL_MSS_MCRC_PSA_SECSIGREGH1_PSASECSIG1_63_32_MASK                     (0xFFFFFFFFU)
#define CSL_MSS_MCRC_PSA_SECSIGREGH1_PSASECSIG1_63_32_SHIFT                    (0x00000000U)
#define CSL_MSS_MCRC_PSA_SECSIGREGH1_PSASECSIG1_63_32_RESETVAL                 (0x00000000U)
#define CSL_MSS_MCRC_PSA_SECSIGREGH1_PSASECSIG1_63_32_MAX                      (0xFFFFFFFFU)

#define CSL_MSS_MCRC_PSA_SECSIGREGH1_RESETVAL                                  (0x00000000U)

/* RAW_DATAREGL1 */

#define CSL_MSS_MCRC_RAW_DATAREGL1_RAW_DATA1_31_0_MASK                         (0xFFFFFFFFU)
#define CSL_MSS_MCRC_RAW_DATAREGL1_RAW_DATA1_31_0_SHIFT                        (0x00000000U)
#define CSL_MSS_MCRC_RAW_DATAREGL1_RAW_DATA1_31_0_RESETVAL                     (0x00000000U)
#define CSL_MSS_MCRC_RAW_DATAREGL1_RAW_DATA1_31_0_MAX                          (0xFFFFFFFFU)

#define CSL_MSS_MCRC_RAW_DATAREGL1_RESETVAL                                    (0x00000000U)

/* RAW_DATAREGH1 */

#define CSL_MSS_MCRC_RAW_DATAREGH1_RAW_DATA1_63_32_MASK                        (0xFFFFFFFFU)
#define CSL_MSS_MCRC_RAW_DATAREGH1_RAW_DATA1_63_32_SHIFT                       (0x00000000U)
#define CSL_MSS_MCRC_RAW_DATAREGH1_RAW_DATA1_63_32_RESETVAL                    (0x00000000U)
#define CSL_MSS_MCRC_RAW_DATAREGH1_RAW_DATA1_63_32_MAX                         (0xFFFFFFFFU)

#define CSL_MSS_MCRC_RAW_DATAREGH1_RESETVAL                                    (0x00000000U)

/* CRC_PCOUNT_REG2 */

#define CSL_MSS_MCRC_CRC_PCOUNT_REG2_CRC_PAT_COUNT2_MASK                       (0x000FFFFFU)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG2_CRC_PAT_COUNT2_SHIFT                      (0x00000000U)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG2_CRC_PAT_COUNT2_RESETVAL                   (0x00000000U)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG2_CRC_PAT_COUNT2_MAX                        (0x000FFFFFU)

#define CSL_MSS_MCRC_CRC_PCOUNT_REG2_RESERVED1_MASK                            (0xFFF00000U)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG2_RESERVED1_SHIFT                           (0x00000014U)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG2_RESERVED1_RESETVAL                        (0x00000000U)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG2_RESERVED1_MAX                             (0x00000FFFU)

#define CSL_MSS_MCRC_CRC_PCOUNT_REG2_RESETVAL                                  (0x00000000U)

/* CRC_SCOUNT_REG2 */

#define CSL_MSS_MCRC_CRC_SCOUNT_REG2_CRC_SEC_COUNT2_MASK                       (0x0000FFFFU)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG2_CRC_SEC_COUNT2_SHIFT                      (0x00000000U)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG2_CRC_SEC_COUNT2_RESETVAL                   (0x00000000U)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG2_CRC_SEC_COUNT2_MAX                        (0x0000FFFFU)

#define CSL_MSS_MCRC_CRC_SCOUNT_REG2_RESERVED1_MASK                            (0xFFFF0000U)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG2_RESERVED1_SHIFT                           (0x00000010U)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG2_RESERVED1_RESETVAL                        (0x00000000U)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG2_RESERVED1_MAX                             (0x0000FFFFU)

#define CSL_MSS_MCRC_CRC_SCOUNT_REG2_RESETVAL                                  (0x00000000U)

/* CRC_CURSEC_REG2 */

#define CSL_MSS_MCRC_CRC_CURSEC_REG2_CRC_CURSEC2_MASK                          (0x0000FFFFU)
#define CSL_MSS_MCRC_CRC_CURSEC_REG2_CRC_CURSEC2_SHIFT                         (0x00000000U)
#define CSL_MSS_MCRC_CRC_CURSEC_REG2_CRC_CURSEC2_RESETVAL                      (0x00000000U)
#define CSL_MSS_MCRC_CRC_CURSEC_REG2_CRC_CURSEC2_MAX                           (0x0000FFFFU)

#define CSL_MSS_MCRC_CRC_CURSEC_REG2_RESERVED1_MASK                            (0xFFFF0000U)
#define CSL_MSS_MCRC_CRC_CURSEC_REG2_RESERVED1_SHIFT                           (0x00000010U)
#define CSL_MSS_MCRC_CRC_CURSEC_REG2_RESERVED1_RESETVAL                        (0x00000000U)
#define CSL_MSS_MCRC_CRC_CURSEC_REG2_RESERVED1_MAX                             (0x0000FFFFU)

#define CSL_MSS_MCRC_CRC_CURSEC_REG2_RESETVAL                                  (0x00000000U)

/* CRC_WDTOPLD2 */

#define CSL_MSS_MCRC_CRC_WDTOPLD2_CRC_WDTOPLD2_MASK                            (0x00FFFFFFU)
#define CSL_MSS_MCRC_CRC_WDTOPLD2_CRC_WDTOPLD2_SHIFT                           (0x00000000U)
#define CSL_MSS_MCRC_CRC_WDTOPLD2_CRC_WDTOPLD2_RESETVAL                        (0x00000000U)
#define CSL_MSS_MCRC_CRC_WDTOPLD2_CRC_WDTOPLD2_MAX                             (0x00FFFFFFU)

#define CSL_MSS_MCRC_CRC_WDTOPLD2_RESERVED1_MASK                               (0xFF000000U)
#define CSL_MSS_MCRC_CRC_WDTOPLD2_RESERVED1_SHIFT                              (0x00000018U)
#define CSL_MSS_MCRC_CRC_WDTOPLD2_RESERVED1_RESETVAL                           (0x00000000U)
#define CSL_MSS_MCRC_CRC_WDTOPLD2_RESERVED1_MAX                                (0x000000FFU)

#define CSL_MSS_MCRC_CRC_WDTOPLD2_RESETVAL                                     (0x00000000U)

/* CRC_BCTOPLD2 */

#define CSL_MSS_MCRC_CRC_BCTOPLD2_CRC_BCTOPLD2_MASK                            (0x00FFFFFFU)
#define CSL_MSS_MCRC_CRC_BCTOPLD2_CRC_BCTOPLD2_SHIFT                           (0x00000000U)
#define CSL_MSS_MCRC_CRC_BCTOPLD2_CRC_BCTOPLD2_RESETVAL                        (0x00000000U)
#define CSL_MSS_MCRC_CRC_BCTOPLD2_CRC_BCTOPLD2_MAX                             (0x00FFFFFFU)

#define CSL_MSS_MCRC_CRC_BCTOPLD2_RESERVED1_MASK                               (0xFF000000U)
#define CSL_MSS_MCRC_CRC_BCTOPLD2_RESERVED1_SHIFT                              (0x00000018U)
#define CSL_MSS_MCRC_CRC_BCTOPLD2_RESERVED1_RESETVAL                           (0x00000000U)
#define CSL_MSS_MCRC_CRC_BCTOPLD2_RESERVED1_MAX                                (0x000000FFU)

#define CSL_MSS_MCRC_CRC_BCTOPLD2_RESETVAL                                     (0x00000000U)

/* PSA_SIGREGL2 */

#define CSL_MSS_MCRC_PSA_SIGREGL2_PSASIG2_31_0_MASK                            (0xFFFFFFFFU)
#define CSL_MSS_MCRC_PSA_SIGREGL2_PSASIG2_31_0_SHIFT                           (0x00000000U)
#define CSL_MSS_MCRC_PSA_SIGREGL2_PSASIG2_31_0_RESETVAL                        (0x00000000U)
#define CSL_MSS_MCRC_PSA_SIGREGL2_PSASIG2_31_0_MAX                             (0xFFFFFFFFU)

#define CSL_MSS_MCRC_PSA_SIGREGL2_RESETVAL                                     (0x00000000U)

/* PSA_SIGREGH2 */

#define CSL_MSS_MCRC_PSA_SIGREGH2_PSA_SIG2_63_32_MASK                          (0xFFFFFFFFU)
#define CSL_MSS_MCRC_PSA_SIGREGH2_PSA_SIG2_63_32_SHIFT                         (0x00000000U)
#define CSL_MSS_MCRC_PSA_SIGREGH2_PSA_SIG2_63_32_RESETVAL                      (0x00000000U)
#define CSL_MSS_MCRC_PSA_SIGREGH2_PSA_SIG2_63_32_MAX                           (0xFFFFFFFFU)

#define CSL_MSS_MCRC_PSA_SIGREGH2_RESETVAL                                     (0x00000000U)

/* CRC_REGL2 */

#define CSL_MSS_MCRC_CRC_REGL2_CRC2_31_0_MASK                                  (0xFFFFFFFFU)
#define CSL_MSS_MCRC_CRC_REGL2_CRC2_31_0_SHIFT                                 (0x00000000U)
#define CSL_MSS_MCRC_CRC_REGL2_CRC2_31_0_RESETVAL                              (0x00000000U)
#define CSL_MSS_MCRC_CRC_REGL2_CRC2_31_0_MAX                                   (0xFFFFFFFFU)

#define CSL_MSS_MCRC_CRC_REGL2_RESETVAL                                        (0x00000000U)

/* CRC_REGH2 */

#define CSL_MSS_MCRC_CRC_REGH2_CRC2_63_32_MASK                                 (0xFFFFFFFFU)
#define CSL_MSS_MCRC_CRC_REGH2_CRC2_63_32_SHIFT                                (0x00000000U)
#define CSL_MSS_MCRC_CRC_REGH2_CRC2_63_32_RESETVAL                             (0x00000000U)
#define CSL_MSS_MCRC_CRC_REGH2_CRC2_63_32_MAX                                  (0xFFFFFFFFU)

#define CSL_MSS_MCRC_CRC_REGH2_RESETVAL                                        (0x00000000U)

/* PSA_SECSIGREGL2 */

#define CSL_MSS_MCRC_PSA_SECSIGREGL2_PSASECSIG2_31_0_MASK                      (0xFFFFFFFFU)
#define CSL_MSS_MCRC_PSA_SECSIGREGL2_PSASECSIG2_31_0_SHIFT                     (0x00000000U)
#define CSL_MSS_MCRC_PSA_SECSIGREGL2_PSASECSIG2_31_0_RESETVAL                  (0x00000000U)
#define CSL_MSS_MCRC_PSA_SECSIGREGL2_PSASECSIG2_31_0_MAX                       (0xFFFFFFFFU)

#define CSL_MSS_MCRC_PSA_SECSIGREGL2_RESETVAL                                  (0x00000000U)

/* PSA_SECSIGREGH2 */

#define CSL_MSS_MCRC_PSA_SECSIGREGH2_PSASECSIG2_63_32_MASK                     (0xFFFFFFFFU)
#define CSL_MSS_MCRC_PSA_SECSIGREGH2_PSASECSIG2_63_32_SHIFT                    (0x00000000U)
#define CSL_MSS_MCRC_PSA_SECSIGREGH2_PSASECSIG2_63_32_RESETVAL                 (0x00000000U)
#define CSL_MSS_MCRC_PSA_SECSIGREGH2_PSASECSIG2_63_32_MAX                      (0xFFFFFFFFU)

#define CSL_MSS_MCRC_PSA_SECSIGREGH2_RESETVAL                                  (0x00000000U)

/* RAW_DATAREGL2 */

#define CSL_MSS_MCRC_RAW_DATAREGL2_RAW_DATA2_31_0_MASK                         (0xFFFFFFFFU)
#define CSL_MSS_MCRC_RAW_DATAREGL2_RAW_DATA2_31_0_SHIFT                        (0x00000000U)
#define CSL_MSS_MCRC_RAW_DATAREGL2_RAW_DATA2_31_0_RESETVAL                     (0x00000000U)
#define CSL_MSS_MCRC_RAW_DATAREGL2_RAW_DATA2_31_0_MAX                          (0xFFFFFFFFU)

#define CSL_MSS_MCRC_RAW_DATAREGL2_RESETVAL                                    (0x00000000U)

/* RAW_DATAREGH2 */

#define CSL_MSS_MCRC_RAW_DATAREGH2_RAW_DATA2_63_32_MASK                        (0xFFFFFFFFU)
#define CSL_MSS_MCRC_RAW_DATAREGH2_RAW_DATA2_63_32_SHIFT                       (0x00000000U)
#define CSL_MSS_MCRC_RAW_DATAREGH2_RAW_DATA2_63_32_RESETVAL                    (0x00000000U)
#define CSL_MSS_MCRC_RAW_DATAREGH2_RAW_DATA2_63_32_MAX                         (0xFFFFFFFFU)

#define CSL_MSS_MCRC_RAW_DATAREGH2_RESETVAL                                    (0x00000000U)

/* CRC_PCOUNT_REG3 */

#define CSL_MSS_MCRC_CRC_PCOUNT_REG3_NU41_MASK                                 (0x000FFFFFU)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG3_NU41_SHIFT                                (0x00000000U)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG3_NU41_RESETVAL                             (0x00000000U)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG3_NU41_MAX                                  (0x000FFFFFU)

#define CSL_MSS_MCRC_CRC_PCOUNT_REG3_RESERVED1_MASK                            (0xFFF00000U)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG3_RESERVED1_SHIFT                           (0x00000014U)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG3_RESERVED1_RESETVAL                        (0x00000000U)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG3_RESERVED1_MAX                             (0x00000FFFU)

#define CSL_MSS_MCRC_CRC_PCOUNT_REG3_RESETVAL                                  (0x00000000U)

/* CRC_SCOUNT_REG3 */

#define CSL_MSS_MCRC_CRC_SCOUNT_REG3_NU42_MASK                                 (0x0000FFFFU)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG3_NU42_SHIFT                                (0x00000000U)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG3_NU42_RESETVAL                             (0x00000000U)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG3_NU42_MAX                                  (0x0000FFFFU)

#define CSL_MSS_MCRC_CRC_SCOUNT_REG3_RESERVED1_MASK                            (0xFFFF0000U)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG3_RESERVED1_SHIFT                           (0x00000010U)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG3_RESERVED1_RESETVAL                        (0x00000000U)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG3_RESERVED1_MAX                             (0x0000FFFFU)

#define CSL_MSS_MCRC_CRC_SCOUNT_REG3_RESETVAL                                  (0x00000000U)

/* CRC_CURSEC_REG3 */

#define CSL_MSS_MCRC_CRC_CURSEC_REG3_NU43_MASK                                 (0x0000FFFFU)
#define CSL_MSS_MCRC_CRC_CURSEC_REG3_NU43_SHIFT                                (0x00000000U)
#define CSL_MSS_MCRC_CRC_CURSEC_REG3_NU43_RESETVAL                             (0x00000000U)
#define CSL_MSS_MCRC_CRC_CURSEC_REG3_NU43_MAX                                  (0x0000FFFFU)

#define CSL_MSS_MCRC_CRC_CURSEC_REG3_RESERVED1_MASK                            (0xFFFF0000U)
#define CSL_MSS_MCRC_CRC_CURSEC_REG3_RESERVED1_SHIFT                           (0x00000010U)
#define CSL_MSS_MCRC_CRC_CURSEC_REG3_RESERVED1_RESETVAL                        (0x00000000U)
#define CSL_MSS_MCRC_CRC_CURSEC_REG3_RESERVED1_MAX                             (0x0000FFFFU)

#define CSL_MSS_MCRC_CRC_CURSEC_REG3_RESETVAL                                  (0x00000000U)

/* CRC_WDTOPLD3 */

#define CSL_MSS_MCRC_CRC_WDTOPLD3_NU44_MASK                                    (0x00FFFFFFU)
#define CSL_MSS_MCRC_CRC_WDTOPLD3_NU44_SHIFT                                   (0x00000000U)
#define CSL_MSS_MCRC_CRC_WDTOPLD3_NU44_RESETVAL                                (0x00000000U)
#define CSL_MSS_MCRC_CRC_WDTOPLD3_NU44_MAX                                     (0x00FFFFFFU)

#define CSL_MSS_MCRC_CRC_WDTOPLD3_RESERVED1_MASK                               (0xFF000000U)
#define CSL_MSS_MCRC_CRC_WDTOPLD3_RESERVED1_SHIFT                              (0x00000018U)
#define CSL_MSS_MCRC_CRC_WDTOPLD3_RESERVED1_RESETVAL                           (0x00000000U)
#define CSL_MSS_MCRC_CRC_WDTOPLD3_RESERVED1_MAX                                (0x000000FFU)

#define CSL_MSS_MCRC_CRC_WDTOPLD3_RESETVAL                                     (0x00000000U)

/* CRC_BCTOPLD3 */

#define CSL_MSS_MCRC_CRC_BCTOPLD3_NU45_MASK                                    (0x00FFFFFFU)
#define CSL_MSS_MCRC_CRC_BCTOPLD3_NU45_SHIFT                                   (0x00000000U)
#define CSL_MSS_MCRC_CRC_BCTOPLD3_NU45_RESETVAL                                (0x00000000U)
#define CSL_MSS_MCRC_CRC_BCTOPLD3_NU45_MAX                                     (0x00FFFFFFU)

#define CSL_MSS_MCRC_CRC_BCTOPLD3_RESERVED1_MASK                               (0xFF000000U)
#define CSL_MSS_MCRC_CRC_BCTOPLD3_RESERVED1_SHIFT                              (0x00000018U)
#define CSL_MSS_MCRC_CRC_BCTOPLD3_RESERVED1_RESETVAL                           (0x00000000U)
#define CSL_MSS_MCRC_CRC_BCTOPLD3_RESERVED1_MAX                                (0x000000FFU)

#define CSL_MSS_MCRC_CRC_BCTOPLD3_RESETVAL                                     (0x00000000U)

/* PSA_SIGREGL3 */

#define CSL_MSS_MCRC_PSA_SIGREGL3_NU46_MASK                                    (0xFFFFFFFFU)
#define CSL_MSS_MCRC_PSA_SIGREGL3_NU46_SHIFT                                   (0x00000000U)
#define CSL_MSS_MCRC_PSA_SIGREGL3_NU46_RESETVAL                                (0x00000000U)
#define CSL_MSS_MCRC_PSA_SIGREGL3_NU46_MAX                                     (0xFFFFFFFFU)

#define CSL_MSS_MCRC_PSA_SIGREGL3_RESETVAL                                     (0x00000000U)

/* PSA_SIGREGH3 */

#define CSL_MSS_MCRC_PSA_SIGREGH3_NU47_MASK                                    (0xFFFFFFFFU)
#define CSL_MSS_MCRC_PSA_SIGREGH3_NU47_SHIFT                                   (0x00000000U)
#define CSL_MSS_MCRC_PSA_SIGREGH3_NU47_RESETVAL                                (0x00000000U)
#define CSL_MSS_MCRC_PSA_SIGREGH3_NU47_MAX                                     (0xFFFFFFFFU)

#define CSL_MSS_MCRC_PSA_SIGREGH3_RESETVAL                                     (0x00000000U)

/* CRC_REGL3 */

#define CSL_MSS_MCRC_CRC_REGL3_NU48_MASK                                       (0xFFFFFFFFU)
#define CSL_MSS_MCRC_CRC_REGL3_NU48_SHIFT                                      (0x00000000U)
#define CSL_MSS_MCRC_CRC_REGL3_NU48_RESETVAL                                   (0x00000000U)
#define CSL_MSS_MCRC_CRC_REGL3_NU48_MAX                                        (0xFFFFFFFFU)

#define CSL_MSS_MCRC_CRC_REGL3_RESETVAL                                        (0x00000000U)

/* CRC_REGH3 */

#define CSL_MSS_MCRC_CRC_REGH3_NU49_MASK                                       (0xFFFFFFFFU)
#define CSL_MSS_MCRC_CRC_REGH3_NU49_SHIFT                                      (0x00000000U)
#define CSL_MSS_MCRC_CRC_REGH3_NU49_RESETVAL                                   (0x00000000U)
#define CSL_MSS_MCRC_CRC_REGH3_NU49_MAX                                        (0xFFFFFFFFU)

#define CSL_MSS_MCRC_CRC_REGH3_RESETVAL                                        (0x00000000U)

/* PSA_SECSIGREGL3 */

#define CSL_MSS_MCRC_PSA_SECSIGREGL3_NU50_MASK                                 (0xFFFFFFFFU)
#define CSL_MSS_MCRC_PSA_SECSIGREGL3_NU50_SHIFT                                (0x00000000U)
#define CSL_MSS_MCRC_PSA_SECSIGREGL3_NU50_RESETVAL                             (0x00000000U)
#define CSL_MSS_MCRC_PSA_SECSIGREGL3_NU50_MAX                                  (0xFFFFFFFFU)

#define CSL_MSS_MCRC_PSA_SECSIGREGL3_RESETVAL                                  (0x00000000U)

/* PSA_SECSIGREGH3 */

#define CSL_MSS_MCRC_PSA_SECSIGREGH3_NU51_MASK                                 (0xFFFFFFFFU)
#define CSL_MSS_MCRC_PSA_SECSIGREGH3_NU51_SHIFT                                (0x00000000U)
#define CSL_MSS_MCRC_PSA_SECSIGREGH3_NU51_RESETVAL                             (0x00000000U)
#define CSL_MSS_MCRC_PSA_SECSIGREGH3_NU51_MAX                                  (0xFFFFFFFFU)

#define CSL_MSS_MCRC_PSA_SECSIGREGH3_RESETVAL                                  (0x00000000U)

/* RAW_DATAREGL3 */

#define CSL_MSS_MCRC_RAW_DATAREGL3_NU52_MASK                                   (0xFFFFFFFFU)
#define CSL_MSS_MCRC_RAW_DATAREGL3_NU52_SHIFT                                  (0x00000000U)
#define CSL_MSS_MCRC_RAW_DATAREGL3_NU52_RESETVAL                               (0x00000000U)
#define CSL_MSS_MCRC_RAW_DATAREGL3_NU52_MAX                                    (0xFFFFFFFFU)

#define CSL_MSS_MCRC_RAW_DATAREGL3_RESETVAL                                    (0x00000000U)

/* RAW_DATAREGH3 */

#define CSL_MSS_MCRC_RAW_DATAREGH3_NU53_MASK                                   (0xFFFFFFFFU)
#define CSL_MSS_MCRC_RAW_DATAREGH3_NU53_SHIFT                                  (0x00000000U)
#define CSL_MSS_MCRC_RAW_DATAREGH3_NU53_RESETVAL                               (0x00000000U)
#define CSL_MSS_MCRC_RAW_DATAREGH3_NU53_MAX                                    (0xFFFFFFFFU)

#define CSL_MSS_MCRC_RAW_DATAREGH3_RESETVAL                                    (0x00000000U)

/* CRC_PCOUNT_REG4 */

#define CSL_MSS_MCRC_CRC_PCOUNT_REG4_NU54_MASK                                 (0x000FFFFFU)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG4_NU54_SHIFT                                (0x00000000U)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG4_NU54_RESETVAL                             (0x00000000U)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG4_NU54_MAX                                  (0x000FFFFFU)

#define CSL_MSS_MCRC_CRC_PCOUNT_REG4_RESERVED1_MASK                            (0xFFF00000U)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG4_RESERVED1_SHIFT                           (0x00000014U)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG4_RESERVED1_RESETVAL                        (0x00000000U)
#define CSL_MSS_MCRC_CRC_PCOUNT_REG4_RESERVED1_MAX                             (0x00000FFFU)

#define CSL_MSS_MCRC_CRC_PCOUNT_REG4_RESETVAL                                  (0x00000000U)

/* CRC_SCOUNT_REG4 */

#define CSL_MSS_MCRC_CRC_SCOUNT_REG4_NU55_MASK                                 (0x0000FFFFU)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG4_NU55_SHIFT                                (0x00000000U)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG4_NU55_RESETVAL                             (0x00000000U)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG4_NU55_MAX                                  (0x0000FFFFU)

#define CSL_MSS_MCRC_CRC_SCOUNT_REG4_RESERVED1_MASK                            (0xFFFF0000U)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG4_RESERVED1_SHIFT                           (0x00000010U)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG4_RESERVED1_RESETVAL                        (0x00000000U)
#define CSL_MSS_MCRC_CRC_SCOUNT_REG4_RESERVED1_MAX                             (0x0000FFFFU)

#define CSL_MSS_MCRC_CRC_SCOUNT_REG4_RESETVAL                                  (0x00000000U)

/* CRC_CURSEC_REG4 */

#define CSL_MSS_MCRC_CRC_CURSEC_REG4_NU56_MASK                                 (0x0000FFFFU)
#define CSL_MSS_MCRC_CRC_CURSEC_REG4_NU56_SHIFT                                (0x00000000U)
#define CSL_MSS_MCRC_CRC_CURSEC_REG4_NU56_RESETVAL                             (0x00000000U)
#define CSL_MSS_MCRC_CRC_CURSEC_REG4_NU56_MAX                                  (0x0000FFFFU)

#define CSL_MSS_MCRC_CRC_CURSEC_REG4_RESERVED1_MASK                            (0xFFFF0000U)
#define CSL_MSS_MCRC_CRC_CURSEC_REG4_RESERVED1_SHIFT                           (0x00000010U)
#define CSL_MSS_MCRC_CRC_CURSEC_REG4_RESERVED1_RESETVAL                        (0x00000000U)
#define CSL_MSS_MCRC_CRC_CURSEC_REG4_RESERVED1_MAX                             (0x0000FFFFU)

#define CSL_MSS_MCRC_CRC_CURSEC_REG4_RESETVAL                                  (0x00000000U)

/* CRC_WDTOPLD4 */

#define CSL_MSS_MCRC_CRC_WDTOPLD4_NU57_MASK                                    (0x00FFFFFFU)
#define CSL_MSS_MCRC_CRC_WDTOPLD4_NU57_SHIFT                                   (0x00000000U)
#define CSL_MSS_MCRC_CRC_WDTOPLD4_NU57_RESETVAL                                (0x00000000U)
#define CSL_MSS_MCRC_CRC_WDTOPLD4_NU57_MAX                                     (0x00FFFFFFU)

#define CSL_MSS_MCRC_CRC_WDTOPLD4_RESERVED1_MASK                               (0xFF000000U)
#define CSL_MSS_MCRC_CRC_WDTOPLD4_RESERVED1_SHIFT                              (0x00000018U)
#define CSL_MSS_MCRC_CRC_WDTOPLD4_RESERVED1_RESETVAL                           (0x00000000U)
#define CSL_MSS_MCRC_CRC_WDTOPLD4_RESERVED1_MAX                                (0x000000FFU)

#define CSL_MSS_MCRC_CRC_WDTOPLD4_RESETVAL                                     (0x00000000U)

/* CRC_BCTOPLD4 */

#define CSL_MSS_MCRC_CRC_BCTOPLD4_NU58_MASK                                    (0x00FFFFFFU)
#define CSL_MSS_MCRC_CRC_BCTOPLD4_NU58_SHIFT                                   (0x00000000U)
#define CSL_MSS_MCRC_CRC_BCTOPLD4_NU58_RESETVAL                                (0x00000000U)
#define CSL_MSS_MCRC_CRC_BCTOPLD4_NU58_MAX                                     (0x00FFFFFFU)

#define CSL_MSS_MCRC_CRC_BCTOPLD4_RESERVED1_MASK                               (0xFF000000U)
#define CSL_MSS_MCRC_CRC_BCTOPLD4_RESERVED1_SHIFT                              (0x00000018U)
#define CSL_MSS_MCRC_CRC_BCTOPLD4_RESERVED1_RESETVAL                           (0x00000000U)
#define CSL_MSS_MCRC_CRC_BCTOPLD4_RESERVED1_MAX                                (0x000000FFU)

#define CSL_MSS_MCRC_CRC_BCTOPLD4_RESETVAL                                     (0x00000000U)

/* PSA_SIGREGL4 */

#define CSL_MSS_MCRC_PSA_SIGREGL4_NU59_MASK                                    (0xFFFFFFFFU)
#define CSL_MSS_MCRC_PSA_SIGREGL4_NU59_SHIFT                                   (0x00000000U)
#define CSL_MSS_MCRC_PSA_SIGREGL4_NU59_RESETVAL                                (0x00000000U)
#define CSL_MSS_MCRC_PSA_SIGREGL4_NU59_MAX                                     (0xFFFFFFFFU)

#define CSL_MSS_MCRC_PSA_SIGREGL4_RESETVAL                                     (0x00000000U)

/* PSA_SIGREGH4 */

#define CSL_MSS_MCRC_PSA_SIGREGH4_NU60_MASK                                    (0xFFFFFFFFU)
#define CSL_MSS_MCRC_PSA_SIGREGH4_NU60_SHIFT                                   (0x00000000U)
#define CSL_MSS_MCRC_PSA_SIGREGH4_NU60_RESETVAL                                (0x00000000U)
#define CSL_MSS_MCRC_PSA_SIGREGH4_NU60_MAX                                     (0xFFFFFFFFU)

#define CSL_MSS_MCRC_PSA_SIGREGH4_RESETVAL                                     (0x00000000U)

/* CRC_REGL4 */

#define CSL_MSS_MCRC_CRC_REGL4_NU61_MASK                                       (0xFFFFFFFFU)
#define CSL_MSS_MCRC_CRC_REGL4_NU61_SHIFT                                      (0x00000000U)
#define CSL_MSS_MCRC_CRC_REGL4_NU61_RESETVAL                                   (0x00000000U)
#define CSL_MSS_MCRC_CRC_REGL4_NU61_MAX                                        (0xFFFFFFFFU)

#define CSL_MSS_MCRC_CRC_REGL4_RESETVAL                                        (0x00000000U)

/* CRC_REGH4 */

#define CSL_MSS_MCRC_CRC_REGH4_NU62_MASK                                       (0xFFFFFFFFU)
#define CSL_MSS_MCRC_CRC_REGH4_NU62_SHIFT                                      (0x00000000U)
#define CSL_MSS_MCRC_CRC_REGH4_NU62_RESETVAL                                   (0x00000000U)
#define CSL_MSS_MCRC_CRC_REGH4_NU62_MAX                                        (0xFFFFFFFFU)

#define CSL_MSS_MCRC_CRC_REGH4_RESETVAL                                        (0x00000000U)

/* PSA_SECSIGREGL4 */

#define CSL_MSS_MCRC_PSA_SECSIGREGL4_NU63_MASK                                 (0xFFFFFFFFU)
#define CSL_MSS_MCRC_PSA_SECSIGREGL4_NU63_SHIFT                                (0x00000000U)
#define CSL_MSS_MCRC_PSA_SECSIGREGL4_NU63_RESETVAL                             (0x00000000U)
#define CSL_MSS_MCRC_PSA_SECSIGREGL4_NU63_MAX                                  (0xFFFFFFFFU)

#define CSL_MSS_MCRC_PSA_SECSIGREGL4_RESETVAL                                  (0x00000000U)

/* PSA_SECSIGREGH4 */

#define CSL_MSS_MCRC_PSA_SECSIGREGH4_NU64_MASK                                 (0xFFFFFFFFU)
#define CSL_MSS_MCRC_PSA_SECSIGREGH4_NU64_SHIFT                                (0x00000000U)
#define CSL_MSS_MCRC_PSA_SECSIGREGH4_NU64_RESETVAL                             (0x00000000U)
#define CSL_MSS_MCRC_PSA_SECSIGREGH4_NU64_MAX                                  (0xFFFFFFFFU)

#define CSL_MSS_MCRC_PSA_SECSIGREGH4_RESETVAL                                  (0x00000000U)

/* RAW_DATAREGL4 */

#define CSL_MSS_MCRC_RAW_DATAREGL4_NU65_MASK                                   (0xFFFFFFFFU)
#define CSL_MSS_MCRC_RAW_DATAREGL4_NU65_SHIFT                                  (0x00000000U)
#define CSL_MSS_MCRC_RAW_DATAREGL4_NU65_RESETVAL                               (0x00000000U)
#define CSL_MSS_MCRC_RAW_DATAREGL4_NU65_MAX                                    (0xFFFFFFFFU)

#define CSL_MSS_MCRC_RAW_DATAREGL4_RESETVAL                                    (0x00000000U)

/* RAW_DATAREGH4 */

#define CSL_MSS_MCRC_RAW_DATAREGH4_NU66_MASK                                   (0xFFFFFFFFU)
#define CSL_MSS_MCRC_RAW_DATAREGH4_NU66_SHIFT                                  (0x00000000U)
#define CSL_MSS_MCRC_RAW_DATAREGH4_NU66_RESETVAL                               (0x00000000U)
#define CSL_MSS_MCRC_RAW_DATAREGH4_NU66_MAX                                    (0xFFFFFFFFU)

#define CSL_MSS_MCRC_RAW_DATAREGH4_RESETVAL                                    (0x00000000U)

/* MCRC_BUS_SEL */

#define CSL_MSS_MCRC_MCRC_BUS_SEL_ITCMEN_MASK                                  (0x00000001U)
#define CSL_MSS_MCRC_MCRC_BUS_SEL_ITCMEN_SHIFT                                 (0x00000000U)
#define CSL_MSS_MCRC_MCRC_BUS_SEL_ITCMEN_RESETVAL                              (0x00000001U)
#define CSL_MSS_MCRC_MCRC_BUS_SEL_ITCMEN_MAX                                   (0x00000001U)

#define CSL_MSS_MCRC_MCRC_BUS_SEL_DTCMEN_MASK                                  (0x00000002U)
#define CSL_MSS_MCRC_MCRC_BUS_SEL_DTCMEN_SHIFT                                 (0x00000001U)
#define CSL_MSS_MCRC_MCRC_BUS_SEL_DTCMEN_RESETVAL                              (0x00000001U)
#define CSL_MSS_MCRC_MCRC_BUS_SEL_DTCMEN_MAX                                   (0x00000001U)

#define CSL_MSS_MCRC_MCRC_BUS_SEL_MEN_MASK                                     (0x00000004U)
#define CSL_MSS_MCRC_MCRC_BUS_SEL_MEN_SHIFT                                    (0x00000002U)
#define CSL_MSS_MCRC_MCRC_BUS_SEL_MEN_RESETVAL                                 (0x00000001U)
#define CSL_MSS_MCRC_MCRC_BUS_SEL_MEN_MAX                                      (0x00000001U)

#define CSL_MSS_MCRC_MCRC_BUS_SEL_NU67_MASK                                    (0xFFFFFFF8U)
#define CSL_MSS_MCRC_MCRC_BUS_SEL_NU67_SHIFT                                   (0x00000003U)
#define CSL_MSS_MCRC_MCRC_BUS_SEL_NU67_RESETVAL                                (0x00000000U)
#define CSL_MSS_MCRC_MCRC_BUS_SEL_NU67_MAX                                     (0x1FFFFFFFU)

#define CSL_MSS_MCRC_MCRC_BUS_SEL_RESETVAL                                     (0x00000007U)

/* MCRC_RESERVED */

#define CSL_MSS_MCRC_MCRC_RESERVED_NU68_MASK                                   (0xFFFFFFFFU)
#define CSL_MSS_MCRC_MCRC_RESERVED_NU68_SHIFT                                  (0x00000000U)
#define CSL_MSS_MCRC_MCRC_RESERVED_NU68_RESETVAL                               (0x00000000U)
#define CSL_MSS_MCRC_MCRC_RESERVED_NU68_MAX                                    (0xFFFFFFFFU)

#define CSL_MSS_MCRC_MCRC_RESERVED_RESETVAL                                    (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
