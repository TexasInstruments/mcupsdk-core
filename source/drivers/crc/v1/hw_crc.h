/*
 *  Copyright (C) 2021 Texas Instruments Incorporated.
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

#ifndef HW_CRC_H_
#define HW_CRC_H_

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************************************
* Register Definitions
****************************************************************************************************/

#define CRC_CTRL0                                                   (0x0U)
#define CRC_CTRL1                                                   (0x8U)
#define CRC_CTRL2                                                   (0x10U)
#define CRC_INTS                                                    (0x18U)
#define CRC_INTR                                                    (0x20U)
#define CRC_STATUS                                                  (0x28U)
#define CRC_INT_OFFSET_REG                                          (0x30U)
#define CRC_BUSY                                                    (0x38U)
#define CRC_PCOUNT_REG1                                             (0x40U)
#define CRC_SCOUNT_REG1                                             (0x44U)
#define CRC_CURSEC_REG1                                             (0x48U)
#define CRC_WDTOPLD1                                                (0x4cU)
#define CRC_BCTOPLD1                                                (0x50U)
#define CRC_PSA_SIGREGL1                                            (0x60U)
#define CRC_PSA_SIGREGH1                                            (0x64U)
#define CRC_REGL1                                                   (0x68U)
#define CRC_REGH1                                                   (0x6cU)
#define CRC_PSA_SECSIGREGL1                                         (0x70U)
#define CRC_PSA_SECSIGREGH1                                         (0x74U)
#define CRC_RAW_DATAREGL1                                           (0x78U)
#define CRC_RAW_DATAREGH1                                           (0x7cU)
#define CRC_PCOUNT_REG2                                             (0x80U)
#define CRC_SCOUNT_REG2                                             (0x84U)
#define CRC_CURSEC_REG2                                             (0x88U)
#define CRC_WDTOPLD2                                                (0x8cU)
#define CRC_BCTOPLD2                                                (0x90U)
#define CRC_PSA_SIGREGL2                                            (0xa0U)
#define CRC_PSA_SIGREGH2                                            (0xa4U)
#define CRC_REGL2                                                   (0xa8U)
#define CRC_REGH2                                                   (0xacU)
#define CRC_PSA_SECSIGREGL2                                         (0xb0U)
#define CRC_PSA_SECSIGREGH2                                         (0xb4U)
#define CRC_RAW_DATAREGL2                                           (0xb8U)
#define CRC_RAW_DATAREGH2                                           (0xbcU)
#define CRC_MCRC_BUS_SEL                                            (0x140U)

/****************************************************************************************************
* Field Definition Macros
****************************************************************************************************/

#define CRC_CTRL0_CH1_PSA_SWRE_SHIFT                                                    (0U)
#define CRC_CTRL0_CH1_PSA_SWRE_MASK                                                     (0x00000001U)
#define CRC_CTRL0_CH1_PSA_SWRE_ON                                                       (0x00000001U)
#define CRC_CTRL0_CH1_PSA_SWRE_OFF                                                      (0x00000000U)

#define CRC_CTRL0_CH1_DW_SEL_SHIFT                                                      (1U)
#define CRC_CTRL0_CH1_DW_SEL_MASK                                                       (0x00000006U)
#define CRC_CTRL0_CH1_DW_SEL_64BIT                                                      (0U)
#define CRC_CTRL0_CH1_DW_SEL_16BIT                                                      (1U)
#define CRC_CTRL0_CH1_DW_SEL_32BIT                                                      (2U)

#define CRC_CTRL0_CH1_CRC_SEL_SHIFT                                                     (3U)
#define CRC_CTRL0_CH1_CRC_SEL_MASK                                                      (0x00000018U)
#define CRC_CTRL0_CH1_CRC_SEL_64BIT                                                     (0U)
#define CRC_CTRL0_CH1_CRC_SEL_16BIT                                                     (1U)
#define CRC_CTRL0_CH1_CRC_SEL_32BIT                                                     (2U)

#define CRC_CTRL0_CH1_BIT_SWAP_SHIFT                                                    (5U)
#define CRC_CTRL0_CH1_BIT_SWAP_MASK                                                     (0x00000020U)
#define CRC_CTRL0_CH1_BIT_SWAP_MSB                                                      (0U)
#define CRC_CTRL0_CH1_BIT_SWAP_LSB                                                      (1U)

#define CRC_CTRL0_CH1_BTYE_SWAP_SHIFT                                                   (6U)
#define CRC_CTRL0_CH1_BTYE_SWAP_MASK                                                    (0x00000040U)
#define CRC_CTRL0_CH1_BTYE_SWAP_DISABLE                                                 (0U)
#define CRC_CTRL0_CH1_BTYE_SWAP_ENABLE                                                  (1U)

#define CRC_CTRL0_CH1_CRC_SEL_2_SHIFT                                                   (7U)
#define CRC_CTRL0_CH1_CRC_SEL_2_MASK                                                    (0x00000080U)

#define CRC_CTRL0_CH2_PSA_SWRE_SHIFT                                                    (8U)
#define CRC_CTRL0_CH2_PSA_SWRE_MASK                                                     (0x00000100U)
#define CRC_CTRL0_CH2_PSA_SWRE_ON                                                       (0x00000001U)
#define CRC_CTRL0_CH2_PSA_SWRE_OFF                                                      (0x00000000U)

#define CRC_CTRL0_CH2_DW_SEL_SHIFT                                                      (9U)
#define CRC_CTRL0_CH2_DW_SEL_MASK                                                       (0x00000600U)
#define CRC_CTRL0_CH2_DW_SEL_64BIT                                                      (0U)
#define CRC_CTRL0_CH2_DW_SEL_16BIT                                                      (1U)
#define CRC_CTRL0_CH2_DW_SEL_32BIT                                                      (2U)

#define CRC_CTRL0_CH2_CRC_SEL_SHIFT                                                     (11U)
#define CRC_CTRL0_CH2_CRC_SEL_MASK                                                      (0x00001800U)
#define CRC_CTRL0_CH2_CRC_SEL_64                                                        (0U)
#define CRC_CTRL0_CH2_CRC_SEL_16                                                        (1U)
#define CRC_CTRL0_CH2_CRC_SEL_32                                                        (2U)

#define CRC_CTRL0_CH2_BIT_SWAP_SHIFT                                                    (13U)
#define CRC_CTRL0_CH2_BIT_SWAP_MASK                                                     (0x00002000U)
#define CRC_CTRL0_CH2_BIT_SWAP_MSB                                                      (0U)
#define CRC_CTRL0_CH2_BIT_SWAP_LSB                                                      (1U)

#define CRC_CTRL0_CH2_BTYE_SWAP_SHIFT                                                   (14U)
#define CRC_CTRL0_CH2_BTYE_SWAP_MASK                                                    (0x00004000U)
#define CRC_CTRL0_CH2_BTYE_SWAP_DISABLE                                                 (0U)
#define CRC_CTRL0_CH2_BTYE_SWAP_ENABLE                                                  (1U)

#define CRC_CTRL0_CH2_CRC_SEL_2_SHIFT                                                   (15U)
#define CRC_CTRL0_CH2_CRC_SEL_2_MASK                                                    (0x00008000U)

#define CRC_CTRL1_PWDN_SHIFT                                                            (0U)
#define CRC_CTRL1_PWDN_MASK                                                             (0x00000001U)

#define CRC_CTRL2_CH1_MODE_SHIFT                                                        (0U)
#define CRC_CTRL2_CH1_MODE_MASK                                                         (0x00000003U)
#define CRC_CTRL2_CH1_MODE_DATA                                                         (0U)
#define CRC_CTRL2_CH1_MODE_AUTO                                                         (1U)
#define CRC_CTRL2_CH1_MODE_SEMICPU                                                      (2U)
#define CRC_CTRL2_CH1_MODE_FULLCPU                                                      (3U)

#define CRC_CTRL2_CH1_TRACEEN_SHIFT                                                     (4U)
#define CRC_CTRL2_CH1_TRACEEN_MASK                                                      (0x00000010U)

#define CRC_CTRL2_CH2_MODE_SHIFT                                                        (8U)
#define CRC_CTRL2_CH2_MODE_MASK                                                         (0x00000300U)
#define CRC_CTRL2_CH2_MODE_DATA                                                         (0U)
#define CRC_CTRL2_CH2_MODE_AUTO                                                         (1U)
#define CRC_CTRL2_CH2_MODE_SEMICPU                                                      (2U)
#define CRC_CTRL2_CH2_MODE_FULLCPU                                                      (3U)

#define CRC_INTS_CH1_FAILENS_SHIFT                                                      (1U)
#define CRC_INTS_CH1_FAILENS_MASK                                                       (0x00000002U)

#define CRC_INTS_CH1_OVERENS_SHIFT                                                      (2U)
#define CRC_INTS_CH1_OVERENS_MASK                                                       (0x00000004U)

#define CRC_INTS_CH1_UNDERENS_SHIFT                                                     (3U)
#define CRC_INTS_CH1_UNDERENS_MASK                                                      (0x00000008U)

#define CRC_INTS_CH1_TIME_OUT_ENS_SHIFT                                                 (4U)
#define CRC_INTS_CH1_TIME_OUT_ENS_MASK                                                  (0x00000010U)

#define CRC_INTS_CH2_FAILENS_SHIFT                                                      (9U)
#define CRC_INTS_CH2_FAILENS_MASK                                                       (0x00000200U)

#define CRC_INTS_CH2_OVERENS_SHIFT                                                      (10U)
#define CRC_INTS_CH2_OVERENS_MASK                                                       (0x00000400U)

#define CRC_INTS_CH2_UNDERENS_SHIFT                                                     (11U)
#define CRC_INTS_CH2_UNDERENS_MASK                                                      (0x00000800U)

#define CRC_INTS_CH2_TIME_OUT_ENS_SHIFT                                                 (12U)
#define CRC_INTS_CH2_TIME_OUT_ENS_MASK                                                  (0x00001000U)

#define CRC_INTR_CH1_FAILENR_SHIFT                                                      (1U)
#define CRC_INTR_CH1_FAILENR_MASK                                                       (0x00000002U)

#define CRC_INTR_CH1_OVERENR_SHIFT                                                      (2U)
#define CRC_INTR_CH1_OVERENR_MASK                                                       (0x00000004U)

#define CRC_INTR_CH1_UNDERENR_SHIFT                                                     (3U)
#define CRC_INTR_CH1_UNDERENR_MASK                                                      (0x00000008U)

#define CRC_INTR_CH1_TIME_OUT_ENR_SHIFT                                                 (4U)
#define CRC_INTR_CH1_TIME_OUT_ENR_MASK                                                  (0x00000010U)

#define CRC_INTR_CH2_FAILENR_SHIFT                                                      (9U)
#define CRC_INTR_CH2_FAILENR_MASK                                                       (0x00000200U)

#define CRC_INTR_CH2_OVERENR_SHIFT                                                      (10U)
#define CRC_INTR_CH2_OVERENR_MASK                                                       (0x00000400U)

#define CRC_INTR_CH2_UNDERENR_SHIFT                                                     (11U)
#define CRC_INTR_CH2_UNDERENR_MASK                                                      (0x00000800U)

#define CRC_INTR_CH2_TIME_OUT_ENR_SHIFT                                                 (12U)
#define CRC_INTR_CH2_TIME_OUT_ENR_MASK                                                  (0x00001000U)

#define CRC_STATUS_CH1_FAIL_SHIFT                                                       (1U)
#define CRC_STATUS_CH1_FAIL_MASK                                                        (0x00000002U)

#define CRC_STATUS_CH1_OVER_SHIFT                                                       (2U)
#define CRC_STATUS_CH1_OVER_MASK                                                        (0x00000004U)

#define CRC_STATUS_CH1_UNDER_SHIFT                                                      (3U)
#define CRC_STATUS_CH1_UNDER_MASK                                                       (0x00000008U)

#define CRC_STATUS_CH1_TIME_OUT_SHIFT                                                   (4U)
#define CRC_STATUS_CH1_TIME_OUT_MASK                                                    (0x00000010U)

#define CRC_STATUS_CH2_FAIL_SHIFT                                                       (9U)
#define CRC_STATUS_CH2_FAIL_MASK                                                        (0x00000200U)

#define CRC_STATUS_CH2_OVER_SHIFT                                                       (10U)
#define CRC_STATUS_CH2_OVER_MASK                                                        (0x00000400U)

#define CRC_STATUS_CH2_UNDER_SHIFT                                                      (11U)
#define CRC_STATUS_CH2_UNDER_MASK                                                       (0x00000800U)

#define CRC_STATUS_CH2_TIME_OUT_SHIFT                                                   (12U)
#define CRC_STATUS_CH2_TIME_OUT_MASK                                                    (0x00001000U)

#define CRC_INT_OFFSET_REG_SHIFT                                                        (0U)
#define CRC_INT_OFFSET_REG_MASK                                                         (0x000000ffU)

#define CRC_BUSY_CH1_SHIFT                                                              (0U)
#define CRC_BUSY_CH1_MASK                                                               (0x00000001U)

#define CRC_BUSY_CH2_SHIFT                                                              (8U)
#define CRC_BUSY_CH2_MASK                                                               (0x00000100U)

#define CRC_PCOUNT_REG1_PAT_COUNT1_SHIFT                                                (0U)
#define CRC_PCOUNT_REG1_PAT_COUNT1_MASK                                                 (0x000fffffU)

#define CRC_SCOUNT_REG1_SEC_COUNT1_SHIFT                                                (0U)
#define CRC_SCOUNT_REG1_SEC_COUNT1_MASK                                                 (0x0000ffffU)

#define CRC_CURSEC_REG1_CURSEC1_SHIFT                                                   (0U)
#define CRC_CURSEC_REG1_CURSEC1_MASK                                                    (0x0000ffffU)

#define CRC_WDTOPLD1_SHIFT                                                              (0U)
#define CRC_WDTOPLD1_MASK                                                               (0x00ffffffU)

#define CRC_BCTOPLD1_SHIFT                                                              (0U)
#define CRC_BCTOPLD1_MASK                                                               (0x00ffffffU)

#define CRC_PSA_SIGREGL1_PSASIG1_SHIFT                                                  (0U)
#define CRC_PSA_SIGREGL1_PSASIG1_MASK                                                   (0xffffffffU)

#define CRC_PSA_SIGREGH1_PSASIG1_63_32_SHIFT                                            (0U)
#define CRC_PSA_SIGREGH1_PSASIG1_63_32_MASK                                             (0xffffffffU)

#define CRC_REGL1_CRC1_SHIFT                                                            (0U)
#define CRC_REGL1_CRC1_MASK                                                             (0xffffffffU)

#define CRC_REGH1_CRC1_47_32_SHIFT                                                      (0U)
#define CRC_REGH1_CRC1_47_32_MASK                                                       (0xffffffffU)

#define CRC_PSA_SECSIGREGL1_PSASECSIG1_SHIFT                                            (0U)
#define CRC_PSA_SECSIGREGL1_PSASECSIG1_MASK                                             (0xffffffffU)

#define CRC_PSA_SECSIGREGH1_PSASECSIG1_SHIFT                                            (0U)
#define CRC_PSA_SECSIGREGH1_PSASECSIG1_MASK                                             (0xffffffffU)

#define CRC_RAW_DATAREGL1_DATA1_SHIFT                                                   (0U)
#define CRC_RAW_DATAREGL1_DATA1_MASK                                                    (0xffffffffU)

#define CRC_RAW_DATAREGH1_DATA1_47_32_SHIFT                                             (0U)
#define CRC_RAW_DATAREGH1_DATA1_47_32_MASK                                              (0xffffffffU)

#define CRC_PCOUNT_REG2_PAT_COUNT2_SHIFT                                                (0U)
#define CRC_PCOUNT_REG2_PAT_COUNT2_MASK                                                 (0x000fffffU)

#define CRC_SCOUNT_REG2_SEC_COUNT2_SHIFT                                                (0U)
#define CRC_SCOUNT_REG2_SEC_COUNT2_MASK                                                 (0x0000ffffU)

#define CRC_CURSEC_REG2_CURSEC2_SHIFT                                                   (0U)
#define CRC_CURSEC_REG2_CURSEC2_MASK                                                    (0x0000ffffU)

#define CRC_WDTOPLD2_SHIFT                                                              (0U)
#define CRC_WDTOPLD2_MASK                                                               (0x00ffffffU)

#define CRC_BCTOPLD2_SHIFT                                                              (0U)
#define CRC_BCTOPLD2_MASK                                                               (0x00ffffffU)

#define CRC_PSA_SIGREGL2_PSASIG2_SHIFT                                                  (0U)
#define CRC_PSA_SIGREGL2_PSASIG2_MASK                                                   (0xffffffffU)

#define CRC_PSA_SIGREGH2_PSASIG2_63_32_SHIFT                                            (0U)
#define CRC_PSA_SIGREGH2_PSASIG2_63_32_MASK                                             (0xffffffffU)

#define CRC_REGL2_CRC2_SHIFT                                                            (0U)
#define CRC_REGL2_CRC2_MASK                                                             (0xffffffffU)

#define CRC_REGH2_CRC2_63_32_SHIFT                                                      (0U)
#define CRC_REGH2_CRC2_63_32_MASK                                                       (0xffffffffU)

#define CRC_PSA_SECSIGREGL2_PSASECSIG2_SHIFT                                            (0U)
#define CRC_PSA_SECSIGREGL2_PSASECSIG2_MASK                                             (0xffffffffU)

#define CRC_PSA_SECSIGREGH2_PSASECSIG2_63_32_SHIFT                                      (0U)
#define CRC_PSA_SECSIGREGH2_PSASECSIG2_63_32_MASK                                       (0xffffffffU)

#define CRC_RAW_DATAREGL2_DATA2_SHIFT                                                   (0U)
#define CRC_RAW_DATAREGL2_DATA2_MASK                                                    (0xffffffffU)

#define CRC_RAW_DATAREGH2_DATA2_63_32_SHIFT                                             (0U)
#define CRC_RAW_DATAREGH2_DATA2_63_32_MASK                                              (0xffffffffU)

#define CRC_MCRC_BUS_SEL_ITC_MEN_SHIFT                                                  (0U)
#define CRC_MCRC_BUS_SEL_ITC_MEN_MASK                                                   (0x00000001U)

#define CRC_MCRC_BUS_SEL_DTC_MEN_SHIFT                                                  (1U)
#define CRC_MCRC_BUS_SEL_DTC_MEN_MASK                                                   (0x00000002U)

#define CRC_MCRC_BUS_SEL_MEN_SHIFT                                                      (2U)
#define CRC_MCRC_BUS_SEL_MEN_MASK                                                       (0x00000004U)

#ifdef __cplusplus
}
#endif
#endif  /* HW_CRC_H_ */

