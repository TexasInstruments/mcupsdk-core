/* =============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2022-2024
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

#ifndef SDL_MCRC_HW_H_
#define SDL_MCRC_HW_H_

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************************************
* Register Definitions
****************************************************************************************************/

#define SDL_MCRC_CTRL0                                                   (0x0U)
#define SDL_MCRC_CTRL1                                                   (0x8U)
#define SDL_MCRC_CTRL2                                                   (0x10U)
#define SDL_MCRC_INTS                                                    (0x18U)
#define SDL_MCRC_INTR                                                    (0x20U)
#define SDL_MCRC_STATUS                                                  (0x28U)
#define SDL_MCRC_INT_OFFSET_REG                                          (0x30U)
#define SDL_MCRC_BUSY                                                    (0x38U)
#define SDL_MCRC_PCOUNT_REG1                                             (0x40U)
#define SDL_MCRC_SCOUNT_REG1                                             (0x44U)
#define SDL_MCRC_CURSEC_REG1                                             (0x48U)
#define SDL_MCRC_WDTOPLD1                                                (0x4cU)
#define SDL_MCRC_BCTOPLD1                                                (0x50U)
#define SDL_MCRC_PSA_SIGREGL1                                            (0x60U)
#define SDL_MCRC_PSA_SIGREGH1                                            (0x64U)
#define SDL_MCRC_REGL1                                                   (0x68U)
#define SDL_MCRC_REGH1                                                   (0x6cU)
#define SDL_MCRC_PSA_SECSIGREGL1                                         (0x70U)
#define SDL_MCRC_PSA_SECSIGREGH1                                         (0x74U)
#define SDL_MCRC_RAW_DATAREGL1                                           (0x78U)
#define SDL_MCRC_RAW_DATAREGH1                                           (0x7cU)
#define SDL_MCRC_PCOUNT_REG2                                             (0x80U)
#define SDL_MCRC_SCOUNT_REG2                                             (0x84U)
#define SDL_MCRC_CURSEC_REG2                                             (0x88U)
#define SDL_MCRC_WDTOPLD2                                                (0x8cU)
#define SDL_MCRC_BCTOPLD2                                                (0x90U)
#define SDL_MCRC_PSA_SIGREGL2                                            (0xa0U)
#define SDL_MCRC_PSA_SIGREGH2                                            (0xa4U)
#define SDL_MCRC_REGL2                                                   (0xa8U)
#define SDL_MCRC_REGH2                                                   (0xacU)
#define SDL_MCRC_PSA_SECSIGREGL2                                         (0xb0U)
#define SDL_MCRC_PSA_SECSIGREGH2                                         (0xb4U)
#define SDL_MCRC_RAW_DATAREGL2                                           (0xb8U)
#define SDL_MCRC_RAW_DATAREGH2                                           (0xbcU)
#define SDL_MCRC_PCOUNT_REG3                                             (0xc0U)
#define SDL_MCRC_SCOUNT_REG3                                             (0xc4U)
#define SDL_MCRC_CURSEC_REG3                                             (0xc8U)
#define SDL_MCRC_WDTOPLD3                                                (0xccU)
#define SDL_MCRC_BCTOPLD3                                                (0xd0U)
#define SDL_MCRC_PSA_SIGREGL3                                            (0xe0U)
#define SDL_MCRC_PSA_SIGREGH3                                            (0xe4U)
#define SDL_MCRC_REGL3                                                   (0xe8U)
#define SDL_MCRC_REGH3                                                   (0xecU)
#define SDL_MCRC_PSA_SECSIGREGL3                                         (0xf0U)
#define SDL_MCRC_PSA_SECSIGREGH3                                         (0xf4U)
#define SDL_MCRC_RAW_DATAREGL3                                           (0xf8U)
#define SDL_MCRC_RAW_DATAREGH3                                           (0xfcU)
#define SDL_MCRC_PCOUNT_REG4                                             (0x100U)
#define SDL_MCRC_SCOUNT_REG4                                             (0x104U)
#define SDL_MCRC_CURSEC_REG4                                             (0x108U)
#define SDL_MCRC_WDTOPLD4                                                (0x10cU)
#define SDL_MCRC_BCTOPLD4                                                (0x110U)
#define SDL_MCRC_PSA_SIGREGL4                                            (0x120U)
#define SDL_MCRC_PSA_SIGREGH4                                            (0x124U)
#define SDL_MCRC_REGL4                                                   (0x128U)
#define SDL_MCRC_REGH4                                                   (0x12cU)
#define SDL_MCRC_PSA_SECSIGREGL4                                         (0x130U)
#define SDL_MCRC_PSA_SECSIGREGH4                                         (0x134U)
#define SDL_MCRC_RAW_DATAREGL4                                           (0x138U)
#define SDL_MCRC_RAW_DATAREGH4                                           (0x13cU)
#define SDL_MCRC_MCRC_BUS_SEL                                            (0x140U)

/****************************************************************************************************
* Field Definition Macros
****************************************************************************************************/

#define SDL_MCRC_CTRL0_CH1_PSA_SWRE_MASK                                                     (0x00000001U)
#define SDL_MCRC_CTRL0_CH1_PSA_SWRE_SHIFT                                                    (0x00000000U)
#define SDL_MCRC_CTRL0_CH1_PSA_SWRE_RESETVAL                                                 (0x00000000U)
#define SDL_MCRC_CTRL0_CH1_PSA_SWRE_MAX                                                      (0x00000001U)
#define SDL_MCRC_CTRL0_CH1_PSA_SWRE_ON                                                       (0x00000001U)
#define SDL_MCRC_CTRL0_CH1_PSA_SWRE_OFF                                                      (0x00000000U)

#define SDL_MCRC_CTRL0_CH1_DW_SEL_MASK                                                       (0x00000006U)
#define SDL_MCRC_CTRL0_CH1_DW_SEL_SHIFT                                                      (0x00000001U)
#define SDL_MCRC_CTRL0_CH1_DW_SEL_RESETVAL                                                   (0x00000000U)
#define SDL_MCRC_CTRL0_CH1_DW_SEL_MAX                                                        (0x00000003U)

#define SDL_MCRC_CTRL0_CH1_CRC_SEL_MASK                                                      (0x00000018U)
#define SDL_MCRC_CTRL0_CH1_CRC_SEL_SHIFT                                                     (0x00000003U)
#define SDL_MCRC_CTRL0_CH1_CRC_SEL_RESETVAL                                                  (0x00000000U)
#define SDL_MCRC_CTRL0_CH1_CRC_SEL_MAX                                                       (0x00000003U)

#define SDL_MCRC_CTRL0_CH1_BIT_SWAP_MASK                                                     (0x00000020U)
#define SDL_MCRC_CTRL0_CH1_BIT_SWAP_SHIFT                                                    (0x00000005U)
#define SDL_MCRC_CTRL0_CH1_BIT_SWAP_RESETVAL                                                 (0x00000000U)
#define SDL_MCRC_CTRL0_CH1_BIT_SWAP_MAX                                                      (0x00000001U)

#define SDL_MCRC_CTRL0_CH1_BYTE_SWAP_MASK                                                    (0x00000040U)
#define SDL_MCRC_CTRL0_CH1_BYTE_SWAP_SHIFT                                                   (0x00000006U)
#define SDL_MCRC_CTRL0_CH1_BYTE_SWAP_RESETVAL                                                (0x00000000U)
#define SDL_MCRC_CTRL0_CH1_BYTE_SWAP_MAX                                                     (0x00000001U)

#define SDL_MCRC_CTRL0_CH1_CRC_SEL2_MASK                                                     (0x00000080U)
#define SDL_MCRC_CTRL0_CH1_CRC_SEL2_SHIFT                                                    (0x00000007U)
#define SDL_MCRC_CTRL0_CH1_CRC_SEL2_RESETVAL                                                 (0x00000000U)
#define SDL_MCRC_CTRL0_CH1_CRC_SEL2_MAX                                                      (0x00000001U)

#define SDL_MCRC_CTRL0_CH2_PSA_SWRE_MASK                                                     (0x00000100U)
#define SDL_MCRC_CTRL0_CH2_PSA_SWRE_SHIFT                                                    (0x00000008U)
#define SDL_MCRC_CTRL0_CH2_PSA_SWRE_RESETVAL                                                 (0x00000000U)
#define SDL_MCRC_CTRL0_CH2_PSA_SWRE_MAX                                                      (0x00000001U)
#define SDL_MCRC_CTRL0_CH2_PSA_SWRE_ON                                                       (0x00000001U)
#define SDL_MCRC_CTRL0_CH2_PSA_SWRE_OFF                                                      (0x00000000U)

#define SDL_MCRC_CTRL0_CH2_DW_SEL_MASK                                                       (0x00000600U)
#define SDL_MCRC_CTRL0_CH2_DW_SEL_SHIFT                                                      (0x00000009U)
#define SDL_MCRC_CTRL0_CH2_DW_SEL_RESETVAL                                                   (0x00000000U)
#define SDL_MCRC_CTRL0_CH2_DW_SEL_MAX                                                        (0x00000003U)

#define SDL_MCRC_CTRL0_CH2_CRC_SEL_MASK                                                      (0x00001800U)
#define SDL_MCRC_CTRL0_CH2_CRC_SEL_SHIFT                                                     (0x0000000BU)
#define SDL_MCRC_CTRL0_CH2_CRC_SEL_RESETVAL                                                  (0x00000000U)
#define SDL_MCRC_CTRL0_CH2_CRC_SEL_MAX                                                       (0x00000003U)

#define SDL_MCRC_CTRL0_CH2_BIT_SWAP_MASK                                                     (0x00002000U)
#define SDL_MCRC_CTRL0_CH2_BIT_SWAP_SHIFT                                                    (0x0000000DU)
#define SDL_MCRC_CTRL0_CH2_BIT_SWAP_RESETVAL                                                 (0x00000000U)
#define SDL_MCRC_CTRL0_CH2_BIT_SWAP_MAX                                                      (0x00000001U)

#define SDL_MCRC_CTRL0_CH2_BYTE_SWAP_MASK                                                    (0x00004000U)
#define SDL_MCRC_CTRL0_CH2_BYTE_SWAP_SHIFT                                                   (0x0000000EU)
#define SDL_MCRC_CTRL0_CH2_BYTE_SWAP_RESETVAL                                                (0x00000000U)
#define SDL_MCRC_CTRL0_CH2_BYTE_SWAP_MAX                                                     (0x00000001U)

#define SDL_MCRC_CTRL0_CH2_CRC_SEL2_MASK                                                     (0x00008000U)
#define SDL_MCRC_CTRL0_CH2_CRC_SEL2_SHIFT                                                    (0x0000000FU)
#define SDL_MCRC_CTRL0_CH2_CRC_SEL2_RESETVAL                                                 (0x00000000U)
#define SDL_MCRC_CTRL0_CH2_CRC_SEL2_MAX                                                      (0x00000001U)

#define SDL_MCRC_CTRL0_CH3_PSA_SWRE_MASK                                                     (0x00010000U)
#define SDL_MCRC_CTRL0_CH3_PSA_SWRE_SHIFT                                                    (0x00000010U)
#define SDL_MCRC_CTRL0_CH3_PSA_SWRE_RESETVAL                                                 (0x00000000U)
#define SDL_MCRC_CTRL0_CH3_PSA_SWRE_MAX                                                      (0x00000001U)
#define SDL_MCRC_CTRL0_CH3_PSA_SWRE_ON                                                       (0x00000001U)
#define SDL_MCRC_CTRL0_CH3_PSA_SWRE_OFF                                                      (0x00000000U)

#define SDL_MCRC_CTRL0_CH3_DW_SEL_MASK                                                       (0x00060000U)
#define SDL_MCRC_CTRL0_CH3_DW_SEL_SHIFT                                                      (0x00000011U)
#define SDL_MCRC_CTRL0_CH3_DW_SEL_RESETVAL                                                   (0x00000000U)
#define SDL_MCRC_CTRL0_CH3_DW_SEL_MAX                                                        (0x00000003U)

#define SDL_MCRC_CTRL0_CH3_CRC_SEL_MASK                                                      (0x00180000U)
#define SDL_MCRC_CTRL0_CH3_CRC_SEL_SHIFT                                                     (0x00000013U)
#define SDL_MCRC_CTRL0_CH3_CRC_SEL_RESETVAL                                                  (0x00000000U)
#define SDL_MCRC_CTRL0_CH3_CRC_SEL_MAX                                                       (0x00000003U)

#define SDL_MCRC_CTRL0_CH3_BIT_SWAP_MASK                                                     (0x00200000U)
#define SDL_MCRC_CTRL0_CH3_BIT_SWAP_SHIFT                                                    (0x00000015U)
#define SDL_MCRC_CTRL0_CH3_BIT_SWAP_RESETVAL                                                 (0x00000000U)
#define SDL_MCRC_CTRL0_CH3_BIT_SWAP_MAX                                                      (0x00000001U)

#define SDL_MCRC_CTRL0_CH3_BYTE_SWAP_MASK                                                    (0x00400000U)
#define SDL_MCRC_CTRL0_CH3_BYTE_SWAP_SHIFT                                                   (0x00000016U)
#define SDL_MCRC_CTRL0_CH3_BYTE_SWAP_RESETVAL                                                (0x00000000U)
#define SDL_MCRC_CTRL0_CH3_BYTE_SWAP_MAX                                                     (0x00000001U)

#define SDL_MCRC_CTRL0_CH3_CRC_SEL2_MASK                                                     (0x00800000U)
#define SDL_MCRC_CTRL0_CH3_CRC_SEL2_SHIFT                                                    (0x00000017U)
#define SDL_MCRC_CTRL0_CH3_CRC_SEL2_RESETVAL                                                 (0x00000000U)
#define SDL_MCRC_CTRL0_CH3_CRC_SEL2_MAX                                                      (0x00000001U)

#define SDL_MCRC_CTRL0_CH4_PSA_SWRE_MASK                                                     (0x01000000U)
#define SDL_MCRC_CTRL0_CH4_PSA_SWRE_SHIFT                                                    (0x00000018U)
#define SDL_MCRC_CTRL0_CH4_PSA_SWRE_RESETVAL                                                 (0x00000000U)
#define SDL_MCRC_CTRL0_CH4_PSA_SWRE_MAX                                                      (0x00000001U)
#define SDL_MCRC_CTRL0_CH4_PSA_SWRE_ON                                                       (0x00000001U)
#define SDL_MCRC_CTRL0_CH4_PSA_SWRE_OFF                                                      (0x00000000U)

#define SDL_MCRC_CTRL0_CH4_DW_SEL_MASK                                                       (0x06000000U)
#define SDL_MCRC_CTRL0_CH4_DW_SEL_SHIFT                                                      (0x00000019U)
#define SDL_MCRC_CTRL0_CH4_DW_SEL_RESETVAL                                                   (0x00000000U)
#define SDL_MCRC_CTRL0_CH4_DW_SEL_MAX                                                        (0x00000003U)

#define SDL_MCRC_CTRL0_CH4_CRC_SEL_MASK                                                      (0x18000000U)
#define SDL_MCRC_CTRL0_CH4_CRC_SEL_SHIFT                                                     (0x0000001BU)
#define SDL_MCRC_CTRL0_CH4_CRC_SEL_RESETVAL                                                  (0x00000000U)
#define SDL_MCRC_CTRL0_CH4_CRC_SEL_MAX                                                       (0x00000003U)

#define SDL_MCRC_CTRL0_CH4_BIT_SWAP_MASK                                                     (0x20000000U)
#define SDL_MCRC_CTRL0_CH4_BIT_SWAP_SHIFT                                                    (0x0000001DU)
#define SDL_MCRC_CTRL0_CH4_BIT_SWAP_RESETVAL                                                 (0x00000000U)
#define SDL_MCRC_CTRL0_CH4_BIT_SWAP_MAX                                                      (0x00000001U)

#define SDL_MCRC_CTRL0_CH4_BYTE_SWAP_MASK                                                    (0x40000000U)
#define SDL_MCRC_CTRL0_CH4_BYTE_SWAP_SHIFT                                                   (0x0000001EU)
#define SDL_MCRC_CTRL0_CH4_BYTE_SWAP_RESETVAL                                                (0x00000000U)
#define SDL_MCRC_CTRL0_CH4_BYTE_SWAP_MAX                                                     (0x00000001U)

#define SDL_MCRC_CTRL0_CH4_CRC_SEL2_MASK                                                     (0x80000000U)
#define SDL_MCRC_CTRL0_CH4_CRC_SEL2_SHIFT                                                    (0x0000001FU)
#define SDL_MCRC_CTRL0_CH4_CRC_SEL2_RESETVAL                                                 (0x00000000U)
#define SDL_MCRC_CTRL0_CH4_CRC_SEL2_MAX                                                      (0x00000001U)

#define SDL_MCRC_CTRL0_RESETVAL                                                              (0x00000000U)

#define SDL_MCRC_CTRL1_PWDN_SHIFT                                                            (0U)
#define SDL_MCRC_CTRL1_PWDN_MASK                                                             (0x00000001U)

#define SDL_MCRC_CTRL2_CH1_MODE_SHIFT                                                        (0U)
#define SDL_MCRC_CTRL2_CH1_MODE_MASK                                                         (0x00000003U)
#define SDL_MCRC_CTRL2_CH1_MODE_DATA                                                         (0U)
#define SDL_MCRC_CTRL2_CH1_MODE_AUTO                                                         (1U)
#define SDL_MCRC_CTRL2_CH1_MODE_SEMICPU                                                      (2U)
#define SDL_MCRC_CTRL2_CH1_MODE_FULLCPU                                                      (3U)

#define SDL_MCRC_CTRL2_CH1_TRACEEN_SHIFT                                                     (4U)
#define SDL_MCRC_CTRL2_CH1_TRACEEN_MASK                                                      (0x00000010U)

#define SDL_MCRC_CTRL2_CH2_MODE_SHIFT                                                        (8U)
#define SDL_MCRC_CTRL2_CH2_MODE_MASK                                                         (0x00000300U)
#define SDL_MCRC_CTRL2_CH2_MODE_DATA                                                         (0U)
#define SDL_MCRC_CTRL2_CH2_MODE_AUTO                                                         (1U)
#define SDL_MCRC_CTRL2_CH2_MODE_SEMICPU                                                      (2U)
#define SDL_MCRC_CTRL2_CH2_MODE_FULLCPU                                                      (3U)

#define SDL_MCRC_CTRL2_CH3_MODE_SHIFT                                                        (16U)
#define SDL_MCRC_CTRL2_CH3_MODE_MASK                                                         (0x00030000U)
#define SDL_MCRC_CTRL2_CH3_MODE_DATA                                                         (0U)
#define SDL_MCRC_CTRL2_CH3_MODE_AUTO                                                         (1U)
#define SDL_MCRC_CTRL2_CH3_MODE_SEMICPU                                                      (2U)
#define SDL_MCRC_CTRL2_CH3_MODE_FULLCPU                                                      (3U)

#define SDL_MCRC_CTRL2_CH4_MODE_SHIFT                                                        (24U)
#define SDL_MCRC_CTRL2_CH4_MODE_MASK                                                         (0x03000000U)
#define SDL_MCRC_CTRL2_CH4_MODE_DATA                                                         (0U)
#define SDL_MCRC_CTRL2_CH4_MODE_AUTO                                                         (1U)
#define SDL_MCRC_CTRL2_CH4_MODE_SEMICPU                                                      (2U)
#define SDL_MCRC_CTRL2_CH4_MODE_FULLCPU                                                      (3U)

#define SDL_MCRC_INTS_CH1_CCITENS_SHIFT                                                      (0U)
#define SDL_MCRC_INTS_CH1_CCITENS_MASK                                                       (0x00000001U)

#define SDL_MCRC_INTS_CH1_FAILENS_SHIFT                                                      (1U)
#define SDL_MCRC_INTS_CH1_FAILENS_MASK                                                       (0x00000002U)

#define SDL_MCRC_INTS_CH1_OVERENS_SHIFT                                                      (2U)
#define SDL_MCRC_INTS_CH1_OVERENS_MASK                                                       (0x00000004U)

#define SDL_MCRC_INTS_CH1_UNDERENS_SHIFT                                                     (3U)
#define SDL_MCRC_INTS_CH1_UNDERENS_MASK                                                      (0x00000008U)

#define SDL_MCRC_INTS_CH1_TIME_OUT_ENS_SHIFT                                                 (4U)
#define SDL_MCRC_INTS_CH1_TIME_OUT_ENS_MASK                                                  (0x00000010U)

#define SDL_MCRC_INTS_CH2_CCITENS_SHIFT                                                      (8U)
#define SDL_MCRC_INTS_CH2_CCITENS_MASK                                                       (0x00000100U)

#define SDL_MCRC_INTS_CH2_FAILENS_SHIFT                                                      (9U)
#define SDL_MCRC_INTS_CH2_FAILENS_MASK                                                       (0x00000200U)

#define SDL_MCRC_INTS_CH2_OVERENS_SHIFT                                                      (10U)
#define SDL_MCRC_INTS_CH2_OVERENS_MASK                                                       (0x00000400U)

#define SDL_MCRC_INTS_CH2_UNDERENS_SHIFT                                                     (11U)
#define SDL_MCRC_INTS_CH2_UNDERENS_MASK                                                      (0x00000800U)

#define SDL_MCRC_INTS_CH2_TIME_OUT_ENS_SHIFT                                                 (12U)
#define SDL_MCRC_INTS_CH2_TIME_OUT_ENS_MASK                                                  (0x00001000U)

#define SDL_MCRC_INTS_CH3_CCITENS_SHIFT                                                      (16U)
#define SDL_MCRC_INTS_CH3_CCITENS_MASK                                                       (0x00010000U)

#define SDL_MCRC_INTS_CH3_FAILENS_SHIFT                                                      (17U)
#define SDL_MCRC_INTS_CH3_FAILENS_MASK                                                       (0x00020000U)

#define SDL_MCRC_INTS_CH3_OVERENS_SHIFT                                                      (18U)
#define SDL_MCRC_INTS_CH3_OVERENS_MASK                                                       (0x00040000U)

#define SDL_MCRC_INTS_CH3_UNDERENS_SHIFT                                                     (19U)
#define SDL_MCRC_INTS_CH3_UNDERENS_MASK                                                      (0x00080000U)

#define SDL_MCRC_INTS_CH3_TIME_OUT_ENS_SHIFT                                                 (20U)
#define SDL_MCRC_INTS_CH3_TIME_OUT_ENS_MASK                                                  (0x00100000U)

#define SDL_MCRC_INTS_CH4_CCITENS_SHIFT                                                      (24U)
#define SDL_MCRC_INTS_CH4_CCITENS_MASK                                                       (0x01000000U)

#define SDL_MCRC_INTS_CH4_FAILENS_SHIFT                                                      (25U)
#define SDL_MCRC_INTS_CH4_FAILENS_MASK                                                       (0x02000000U)

#define SDL_MCRC_INTS_CH4_OVERENS_SHIFT                                                      (26U)
#define SDL_MCRC_INTS_CH4_OVERENS_MASK                                                       (0x04000000U)

#define SDL_MCRC_INTS_CH4_UNDERENS_SHIFT                                                     (27U)
#define SDL_MCRC_INTS_CH4_UNDERENS_MASK                                                      (0x08000000U)

#define SDL_MCRC_INTS_CH4_TIME_OUT_ENS_SHIFT                                                 (28U)
#define SDL_MCRC_INTS_CH4_TIME_OUT_ENS_MASK                                                  (0x10000000U)

#define SDL_MCRC_INTS_CH2_CCITENS_SHIFT                                                      (8U)
#define SDL_MCRC_INTS_CH2_CCITENS_MASK                                                       (0x00000100U)

#define SDL_MCRC_INTS_CH3_CCITENS_SHIFT                                                      (16U)
#define SDL_MCRC_INTS_CH3_CCITENS_MASK                                                       (0x00010000U)

#define SDL_MCRC_INTS_CH4_CCITENS_SHIFT                                                      (24U)
#define SDL_MCRC_INTS_CH4_CCITENS_MASK                                                       (0x01000000U)

#define SDL_MCRC_INTR_CH1_FAILENR_SHIFT                                                      (1U)
#define SDL_MCRC_INTR_CH1_FAILENR_MASK                                                       (0x00000002U)

#define SDL_MCRC_INTR_CH1_OVERENR_SHIFT                                                      (2U)
#define SDL_MCRC_INTR_CH1_OVERENR_MASK                                                       (0x00000004U)

#define SDL_MCRC_INTR_CH1_UNDERENR_SHIFT                                                     (3U)
#define SDL_MCRC_INTR_CH1_UNDERENR_MASK                                                      (0x00000008U)

#define SDL_MCRC_INTR_CH1_TIME_OUT_ENR_SHIFT                                                 (4U)
#define SDL_MCRC_INTR_CH1_TIME_OUT_ENR_MASK                                                  (0x00000010U)

#define SDL_MCRC_INTR_CH2_FAILENR_SHIFT                                                      (9U)
#define SDL_MCRC_INTR_CH2_FAILENR_MASK                                                       (0x00000200U)

#define SDL_MCRC_INTR_CH2_OVERENR_SHIFT                                                      (10U)
#define SDL_MCRC_INTR_CH2_OVERENR_MASK                                                       (0x00000400U)

#define SDL_MCRC_INTR_CH2_UNDERENR_SHIFT                                                     (11U)
#define SDL_MCRC_INTR_CH2_UNDERENR_MASK                                                      (0x00000800U)

#define SDL_MCRC_INTR_CH2_TIME_OUT_ENR_SHIFT                                                 (12U)
#define SDL_MCRC_INTR_CH2_TIME_OUT_ENR_MASK                                                  (0x00001000U)

#define SDL_MCRC_INTR_CH3_FAILENR_SHIFT                                                      (17U)
#define SDL_MCRC_INTR_CH3_FAILENR_MASK                                                       (0x00020000U)

#define SDL_MCRC_INTR_CH3_OVERENR_SHIFT                                                      (18U)
#define SDL_MCRC_INTR_CH3_OVERENR_MASK                                                       (0x00040000U)

#define SDL_MCRC_INTR_CH3_UNDERENR_SHIFT                                                     (19U)
#define SDL_MCRC_INTR_CH3_UNDERENR_MASK                                                      (0x00080000U)

#define SDL_MCRC_INTR_CH3_TIME_OUT_ENR_SHIFT                                                 (20U)
#define SDL_MCRC_INTR_CH3_TIME_OUT_ENR_MASK                                                  (0x00100000U)

#define SDL_MCRC_INTR_CH4_FAILENR_SHIFT                                                      (25U)
#define SDL_MCRC_INTR_CH4_FAILENR_MASK                                                       (0x02000000U)

#define SDL_MCRC_INTR_CH4_OVERENR_SHIFT                                                      (26U)
#define SDL_MCRC_INTR_CH4_OVERENR_MASK                                                       (0x04000000U)

#define SDL_MCRC_INTR_CH4_UNDERENR_SHIFT                                                     (27U)
#define SDL_MCRC_INTR_CH4_UNDERENR_MASK                                                      (0x08000000U)

#define SDL_MCRC_INTR_CH4_TIME_OUT_ENR_SHIFT                                                 (28U)
#define SDL_MCRC_INTR_CH4_TIME_OUT_ENR_MASK                                                  (0x10000000U)

#define SDL_MCRC_INTR_CH1_CCITENR_SHIFT                                                      (0U)
#define SDL_MCRC_INTR_CH1_CCITENR_MASK                                                       (0x00000001U)

#define SDL_MCRC_INTR_CH2_CCITENR_SHIFT                                                      (8U)
#define SDL_MCRC_INTR_CH2_CCITENR_MASK                                                       (0x00000100U)

#define SDL_MCRC_INTR_CH3_CCITENR_SHIFT                                                      (16U)
#define SDL_MCRC_INTR_CH3_CCITENR_MASK                                                       (0x00010000U)

#define SDL_MCRC_INTR_CH4_CCITENR_SHIFT                                                      (24U)
#define SDL_MCRC_INTR_CH4_CCITENR_MASK                                                       (0x01000000U)

#define SDL_MCRC_STATUS_CH1_CCIT_SHIFT                                                       (0U)
#define SDL_MCRC_STATUS_CH1_CCIT_MASK                                                        (0x00000001U)

#define SDL_MCRC_STATUS_CH1_FAIL_SHIFT                                                       (1U)
#define SDL_MCRC_STATUS_CH1_FAIL_MASK                                                        (0x00000002U)

#define SDL_MCRC_STATUS_CH1_OVER_SHIFT                                                       (2U)
#define SDL_MCRC_STATUS_CH1_OVER_MASK                                                        (0x00000004U)

#define SDL_MCRC_STATUS_CH1_UNDER_SHIFT                                                      (3U)
#define SDL_MCRC_STATUS_CH1_UNDER_MASK                                                       (0x00000008U)

#define SDL_MCRC_STATUS_CH1_TIME_OUT_SHIFT                                                   (4U)
#define SDL_MCRC_STATUS_CH1_TIME_OUT_MASK                                                    (0x00000010U)

#define SDL_MCRC_STATUS_CH2_FAIL_SHIFT                                                       (9U)
#define SDL_MCRC_STATUS_CH2_FAIL_MASK                                                        (0x00000200U)

#define SDL_MCRC_STATUS_CH2_OVER_SHIFT                                                       (10U)
#define SDL_MCRC_STATUS_CH2_OVER_MASK                                                        (0x00000400U)

#define SDL_MCRC_STATUS_CH2_UNDER_SHIFT                                                      (11U)
#define SDL_MCRC_STATUS_CH2_UNDER_MASK                                                       (0x00000800U)

#define SDL_MCRC_STATUS_CH2_TIME_OUT_SHIFT                                                   (12U)
#define SDL_MCRC_STATUS_CH2_TIME_OUT_MASK                                                    (0x00001000U)

#define SDL_MCRC_STATUS_CH3_FAIL_SHIFT                                                       (17U)
#define SDL_MCRC_STATUS_CH3_FAIL_MASK                                                        (0x00020000U)

#define SDL_MCRC_STATUS_CH3_OVER_SHIFT                                                       (18U)
#define SDL_MCRC_STATUS_CH3_OVER_MASK                                                        (0x00040000U)

#define SDL_MCRC_STATUS_CH3_UNDER_SHIFT                                                      (19U)
#define SDL_MCRC_STATUS_CH3_UNDER_MASK                                                       (0x00080000U)

#define SDL_MCRC_STATUS_CH3_TIME_OUT_SHIFT                                                   (20U)
#define SDL_MCRC_STATUS_CH3_TIME_OUT_MASK                                                    (0x00100000U)

#define SDL_MCRC_STATUS_CH4_FAIL_SHIFT                                                       (25U)
#define SDL_MCRC_STATUS_CH4_FAIL_MASK                                                        (0x02000000U)

#define SDL_MCRC_STATUS_CH4_OVER_SHIFT                                                       (26U)
#define SDL_MCRC_STATUS_CH4_OVER_MASK                                                        (0x04000000U)

#define SDL_MCRC_STATUS_CH4_UNDER_SHIFT                                                      (27U)
#define SDL_MCRC_STATUS_CH4_UNDER_MASK                                                       (0x08000000U)

#define SDL_MCRC_STATUS_CH4_TIME_OUT_SHIFT                                                   (28U)
#define SDL_MCRC_STATUS_CH4_TIME_OUT_MASK                                                    (0x10000000U)

#define SDL_MCRC_STATUS_CH2_CCIT_SHIFT                                                       (8U)
#define SDL_MCRC_STATUS_CH2_CCIT_MASK                                                        (0x00000100U)

#define SDL_MCRC_STATUS_CH3_CCIT_SHIFT                                                       (16U)
#define SDL_MCRC_STATUS_CH3_CCIT_MASK                                                        (0x00010000U)

#define SDL_MCRC_STATUS_CH4_CCIT_SHIFT                                                       (24U)
#define SDL_MCRC_STATUS_CH4_CCIT_MASK                                                        (0x01000000U)

#define SDL_MCRC_INT_OFFSET_REG_SHIFT                                                        (0U)
#define SDL_MCRC_INT_OFFSET_REG_MASK                                                         (0x000000ffU)

#define SDL_MCRC_BUSY_CH1_SHIFT                                                              (0U)
#define SDL_MCRC_BUSY_CH1_MASK                                                               (0x00000001U)

#define SDL_MCRC_BUSY_CH2_SHIFT                                                              (8U)
#define SDL_MCRC_BUSY_CH2_MASK                                                               (0x00000100U)

#define SDL_MCRC_BUSY_CH3_SHIFT                                                              (16U)
#define SDL_MCRC_BUSY_CH3_MASK                                                               (0x00010000U)

#define SDL_MCRC_BUSY_CH4_SHIFT                                                              (24U)
#define SDL_MCRC_BUSY_CH4_MASK                                                               (0x01000000U)

#define SDL_MCRC_PCOUNT_REG1_PAT_COUNT1_SHIFT                                                (0U)
#define SDL_MCRC_PCOUNT_REG1_PAT_COUNT1_MASK                                                 (0x000fffffU)

#define SDL_MCRC_SCOUNT_REG1_SEC_COUNT1_SHIFT                                                (0U)
#define SDL_MCRC_SCOUNT_REG1_SEC_COUNT1_MASK                                                 (0x0000ffffU)

#define SDL_MCRC_CURSEC_REG1_CURSEC1_SHIFT                                                   (0U)
#define SDL_MCRC_CURSEC_REG1_CURSEC1_MASK                                                    (0x0000ffffU)

#define SDL_MCRC_WDTOPLD1_SHIFT                                                              (0U)
#define SDL_MCRC_WDTOPLD1_MASK                                                               (0x00ffffffU)

#define SDL_MCRC_BCTOPLD1_SHIFT                                                              (0U)
#define SDL_MCRC_BCTOPLD1_MASK                                                               (0x00ffffffU)

#define SDL_MCRC_PSA_SIGREGL1_PSASIG1_SHIFT                                                  (0U)
#define SDL_MCRC_PSA_SIGREGL1_PSASIG1_MASK                                                   (0xffffffffU)

#define SDL_MCRC_PSA_SIGREGH1_PSASIG1_63_32_SHIFT                                            (0U)
#define SDL_MCRC_PSA_SIGREGH1_PSASIG1_63_32_MASK                                             (0xffffffffU)

#define SDL_MCRC_REGL1_CRC1_SHIFT                                                            (0U)
#define SDL_MCRC_REGL1_CRC1_MASK                                                             (0xffffffffU)

#define SDL_MCRC_REGH1_CRC1_47_32_SHIFT                                                      (0U)
#define SDL_MCRC_REGH1_CRC1_47_32_MASK                                                       (0xffffffffU)

#define SDL_MCRC_PSA_SECSIGREGL1_PSASECSIG1_SHIFT                                            (0U)
#define SDL_MCRC_PSA_SECSIGREGL1_PSASECSIG1_MASK                                             (0xffffffffU)

#define SDL_MCRC_PSA_SECSIGREGH1_PSASECSIG1_SHIFT                                            (0U)
#define SDL_MCRC_PSA_SECSIGREGH1_PSASECSIG1_MASK                                             (0xffffffffU)

#define SDL_MCRC_RAW_DATAREGL1_DATA1_SHIFT                                                   (0U)
#define SDL_MCRC_RAW_DATAREGL1_DATA1_MASK                                                    (0xffffffffU)

#define SDL_MCRC_RAW_DATAREGH1_DATA1_47_32_SHIFT                                             (0U)
#define SDL_MCRC_RAW_DATAREGH1_DATA1_47_32_MASK                                              (0xffffffffU)

#define SDL_MCRC_PCOUNT_REG2_PAT_COUNT2_SHIFT                                                (0U)
#define SDL_MCRC_PCOUNT_REG2_PAT_COUNT2_MASK                                                 (0x000fffffU)

#define SDL_MCRC_SCOUNT_REG2_SEC_COUNT2_SHIFT                                                (0U)
#define SDL_MCRC_SCOUNT_REG2_SEC_COUNT2_MASK                                                 (0x0000ffffU)

#define SDL_MCRC_CURSEC_REG2_CURSEC2_SHIFT                                                   (0U)
#define SDL_MCRC_CURSEC_REG2_CURSEC2_MASK                                                    (0x0000ffffU)

#define SDL_MCRC_WDTOPLD2_SHIFT                                                              (0U)
#define SDL_MCRC_WDTOPLD2_MASK                                                               (0x00ffffffU)

#define SDL_MCRC_BCTOPLD2_SHIFT                                                              (0U)
#define SDL_MCRC_BCTOPLD2_MASK                                                               (0x00ffffffU)

#define SDL_MCRC_PSA_SIGREGL2_PSASIG2_SHIFT                                                  (0U)
#define SDL_MCRC_PSA_SIGREGL2_PSASIG2_MASK                                                   (0xffffffffU)

#define SDL_MCRC_PSA_SIGREGH2_PSASIG2_63_32_SHIFT                                            (0U)
#define SDL_MCRC_PSA_SIGREGH2_PSASIG2_63_32_MASK                                             (0xffffffffU)

#define SDL_MCRC_REGL2_CRC2_SHIFT                                                            (0U)
#define SDL_MCRC_REGL2_CRC2_MASK                                                             (0xffffffffU)

#define SDL_MCRC_REGH2_CRC2_63_32_SHIFT                                                      (0U)
#define SDL_MCRC_REGH2_CRC2_63_32_MASK                                                       (0xffffffffU)

#define SDL_MCRC_PSA_SECSIGREGL2_PSASECSIG2_SHIFT                                            (0U)
#define SDL_MCRC_PSA_SECSIGREGL2_PSASECSIG2_MASK                                             (0xffffffffU)

#define SDL_MCRC_PSA_SECSIGREGH2_PSASECSIG2_63_32_SHIFT                                      (0U)
#define SDL_MCRC_PSA_SECSIGREGH2_PSASECSIG2_63_32_MASK                                       (0xffffffffU)

#define SDL_MCRC_RAW_DATAREGL2_DATA2_SHIFT                                                   (0U)
#define SDL_MCRC_RAW_DATAREGL2_DATA2_MASK                                                    (0xffffffffU)

#define SDL_MCRC_RAW_DATAREGH2_DATA2_63_32_SHIFT                                             (0U)
#define SDL_MCRC_RAW_DATAREGH2_DATA2_63_32_MASK                                              (0xffffffffU)

#define SDL_MCRC_PCOUNT_REG3_PAT_COUNT3_SHIFT                                                (0U)
#define SDL_MCRC_PCOUNT_REG3_PAT_COUNT3_MASK                                                 (0x000fffffU)

#define SDL_MCRC_SCOUNT_REG3_SEC_COUNT3_SHIFT                                                (0U)
#define SDL_MCRC_SCOUNT_REG3_SEC_COUNT3_MASK                                                 (0x0000ffffU)

#define SDL_MCRC_CURSEC_REG3_CURSEC3_SHIFT                                                   (0U)
#define SDL_MCRC_CURSEC_REG3_CURSEC3_MASK                                                    (0x0000ffffU)

#define SDL_MCRC_WDTOPLD3_SHIFT                                                              (0U)
#define SDL_MCRC_WDTOPLD3_MASK                                                               (0x0000ffffU)

#define SDL_MCRC_BCTOPLD3_SHIFT                                                              (0U)
#define SDL_MCRC_BCTOPLD3_MASK                                                               (0x00ffffffU)

#define SDL_MCRC_PSA_SIGREGL3_PSASIG3_SHIFT                                                  (0U)
#define SDL_MCRC_PSA_SIGREGL3_PSASIG3_MASK                                                   (0xffffffffU)

#define SDL_MCRC_PSA_SIGREGH3_PSASIG3_63_32_SHIFT                                            (0U)
#define SDL_MCRC_PSA_SIGREGH3_PSASIG3_63_32_MASK                                             (0xffffffffU)

#define SDL_MCRC_REGL3_CRC3_SHIFT                                                            (0U)
#define SDL_MCRC_REGL3_CRC3_MASK                                                             (0xffffffffU)

#define SDL_MCRC_REGH3_CRC3_63_32_SHIFT                                                      (0U)
#define SDL_MCRC_REGH3_CRC3_63_32_MASK                                                       (0xffffffffU)

#define SDL_MCRC_PSA_SECSIGREGL3_PSASECSIG3_SHIFT                                            (0U)
#define SDL_MCRC_PSA_SECSIGREGL3_PSASECSIG3_MASK                                             (0xffffffffU)

#define SDL_MCRC_PSA_SECSIGREGH3_PSASECSIG3_63_32_SHIFT                                      (0U)
#define SDL_MCRC_PSA_SECSIGREGH3_PSASECSIG3_63_32_MASK                                       (0xffffffffU)

#define SDL_MCRC_RAW_DATAREGL3_DATA3_SHIFT                                                   (0U)
#define SDL_MCRC_RAW_DATAREGL3_DATA3_MASK                                                    (0xffffffffU)

#define SDL_MCRC_RAW_DATAREGH3_DATA3_63_32_SHIFT                                             (0U)
#define SDL_MCRC_RAW_DATAREGH3_DATA3_63_32_MASK                                              (0xffffffffU)

#define SDL_MCRC_PCOUNT_REG4_PAT_COUNT4_SHIFT                                                (0U)
#define SDL_MCRC_PCOUNT_REG4_PAT_COUNT4_MASK                                                 (0x000fffffU)

#define SDL_MCRC_SCOUNT_REG4_SEC_COUNT4_SHIFT                                                (0U)
#define SDL_MCRC_SCOUNT_REG4_SEC_COUNT4_MASK                                                 (0x0000ffffU)

#define SDL_MCRC_CURSEC_REG4_CURSEC4_SHIFT                                                   (0U)
#define SDL_MCRC_CURSEC_REG4_CURSEC4_MASK                                                    (0x0000ffffU)

#define SDL_MCRC_WDTOPLD4_SHIFT                                                              (0U)
#define SDL_MCRC_WDTOPLD4_MASK                                                               (0x00ffffffU)

#define SDL_MCRC_BCTOPLD4_SHIFT                                                              (0U)
#define SDL_MCRC_BCTOPLD4_MASK                                                               (0x00ffffffU)

#define SDL_MCRC_PSA_SIGREGL4_PSASIG4_SHIFT                                                  (0U)
#define SDL_MCRC_PSA_SIGREGL4_PSASIG4_MASK                                                   (0x00ffffffU)

#define SDL_MCRC_PSA_SIGREGH4_PSASIG4_63_32_SHIFT                                            (0U)
#define SDL_MCRC_PSA_SIGREGH4_PSASIG4_63_32_MASK                                             (0xffffffffU)

#define SDL_MCRC_REGL4_CRC4_SHIFT                                                            (0U)
#define SDL_MCRC_REGL4_CRC4_MASK                                                             (0xffffffffU)

#define SDL_MCRC_REGH4_CRC4_63_32_SHIFT                                                      (0U)
#define SDL_MCRC_REGH4_CRC4_63_32_MASK                                                       (0xffffffffU)

#define SDL_MCRC_PSA_SECSIGREGL4_PSASECSIG4_SHIFT                                            (0U)
#define SDL_MCRC_PSA_SECSIGREGL4_PSASECSIG4_MASK                                             (0xffffffffU)

#define SDL_MCRC_PSA_SECSIGREGH4_PSASECSIG4_63_32_SHIFT                                      (0U)
#define SDL_MCRC_PSA_SECSIGREGH4_PSASECSIG4_63_32_MASK                                       (0xffffffffU)

#define SDL_MCRC_RAW_DATAREGL4_DATA4_SHIFT                                                   (0U)
#define SDL_MCRC_RAW_DATAREGL4_DATA4_MASK                                                    (0xffffffffU)

#define SDL_MCRC_RAW_DATAREGH4_DATA4_63_32_SHIFT                                             (0U)
#define SDL_MCRC_RAW_DATAREGH4_DATA4_63_32_MASK                                              (0xffffffffU)

#define SDL_MCRC_MCRC_BUS_SEL_ITC_MEN_SHIFT                                                  (0U)
#define SDL_MCRC_MCRC_BUS_SEL_ITC_MEN_MASK                                                   (0x00000001U)

#define SDL_MCRC_MCRC_BUS_SEL_DTC_MEN_SHIFT                                                  (1U)
#define SDL_MCRC_MCRC_BUS_SEL_DTC_MEN_MASK                                                   (0x00000002U)

#define SDL_MCRC_MCRC_BUS_SEL_MEN_SHIFT                                                      (2U)
#define SDL_MCRC_MCRC_BUS_SEL_MEN_MASK                                                       (0x00000004U)

#ifdef __cplusplus
}
#endif
#endif  /* SDL_MCRC_HW_H_ */

