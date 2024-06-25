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
 *  Name        : cslr_mss_mcan_ecc.h
*/
#ifndef CSLR_MSS_MCAN_ECC_H_
#define CSLR_MSS_MCAN_ECC_H_

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
    volatile uint32_t REV;
    volatile uint8_t  Resv_8[4];
    volatile uint32_t VECTOR;
    volatile uint32_t STAT;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t CTRL;
    volatile uint32_t ERR_CTRL1;
    volatile uint32_t ERR_CTRL2;
    volatile uint32_t ERR_STAT1;
    volatile uint32_t ERR_STAT2;
    volatile uint32_t ERR_STAT3;
    volatile uint8_t  Resv_60[16];
    volatile uint32_t SEC_EOI_REG;
    volatile uint32_t SEC_STATUS_REG0;
    volatile uint8_t  Resv_128[60];
    volatile uint32_t SEC_ENABLE_SET_REG0;
    volatile uint8_t  Resv_192[60];
    volatile uint32_t SEC_ENABLE_CLR_REG0;
    volatile uint8_t  Resv_316[120];
    volatile uint32_t DED_EOI_REG;
    volatile uint32_t DED_STATUS_REG0;
    volatile uint8_t  Resv_384[60];
    volatile uint32_t DED_ENABLE_SET_REG0;
    volatile uint8_t  Resv_448[60];
    volatile uint32_t DED_ENABLE_CLR_REG0;
    volatile uint8_t  Resv_512[60];
    volatile uint32_t AGGR_ENABLE_SET;
    volatile uint32_t AGGR_ENABLE_CLR;
    volatile uint32_t AGGR_STATUS_SET;
    volatile uint32_t AGGR_STATUS_CLR;
} CSL_mss_mcan_eccRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_MSS_MCAN_ECC_REV                                                   (0x00000000U)
#define CSL_MSS_MCAN_ECC_VECTOR                                                (0x00000008U)
#define CSL_MSS_MCAN_ECC_STAT                                                  (0x0000000CU)
#define CSL_MSS_MCAN_ECC_CTRL                                                  (0x00000014U)
#define CSL_MSS_MCAN_ECC_ERR_CTRL1                                             (0x00000018U)
#define CSL_MSS_MCAN_ECC_ERR_CTRL2                                             (0x0000001CU)
#define CSL_MSS_MCAN_ECC_ERR_STAT1                                             (0x00000020U)
#define CSL_MSS_MCAN_ECC_ERR_STAT2                                             (0x00000024U)
#define CSL_MSS_MCAN_ECC_ERR_STAT3                                             (0x00000028U)
#define CSL_MSS_MCAN_ECC_SEC_EOI_REG                                           (0x0000003CU)
#define CSL_MSS_MCAN_ECC_SEC_STATUS_REG0                                       (0x00000040U)
#define CSL_MSS_MCAN_ECC_SEC_ENABLE_SET_REG0                                   (0x00000080U)
#define CSL_MSS_MCAN_ECC_SEC_ENABLE_CLR_REG0                                   (0x000000C0U)
#define CSL_MSS_MCAN_ECC_DED_EOI_REG                                           (0x0000013CU)
#define CSL_MSS_MCAN_ECC_DED_STATUS_REG0                                       (0x00000140U)
#define CSL_MSS_MCAN_ECC_DED_ENABLE_SET_REG0                                   (0x00000180U)
#define CSL_MSS_MCAN_ECC_DED_ENABLE_CLR_REG0                                   (0x000001C0U)
#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_SET                                       (0x00000200U)
#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_CLR                                       (0x00000204U)
#define CSL_MSS_MCAN_ECC_AGGR_STATUS_SET                                       (0x00000208U)
#define CSL_MSS_MCAN_ECC_AGGR_STATUS_CLR                                       (0x0000020CU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* REV */

#define CSL_MSS_MCAN_ECC_REV_REVMIN_MASK                                       (0x0000003FU)
#define CSL_MSS_MCAN_ECC_REV_REVMIN_SHIFT                                      (0x00000000U)
#define CSL_MSS_MCAN_ECC_REV_REVMIN_RESETVAL                                   (0x00000000U)
#define CSL_MSS_MCAN_ECC_REV_REVMIN_MAX                                        (0x0000003FU)

#define CSL_MSS_MCAN_ECC_REV_CUSTOM_MASK                                       (0x000000C0U)
#define CSL_MSS_MCAN_ECC_REV_CUSTOM_SHIFT                                      (0x00000006U)
#define CSL_MSS_MCAN_ECC_REV_CUSTOM_RESETVAL                                   (0x00000000U)
#define CSL_MSS_MCAN_ECC_REV_CUSTOM_MAX                                        (0x00000003U)

#define CSL_MSS_MCAN_ECC_REV_REVMAJ_MASK                                       (0x00000700U)
#define CSL_MSS_MCAN_ECC_REV_REVMAJ_SHIFT                                      (0x00000008U)
#define CSL_MSS_MCAN_ECC_REV_REVMAJ_RESETVAL                                   (0x00000002U)
#define CSL_MSS_MCAN_ECC_REV_REVMAJ_MAX                                        (0x00000007U)

#define CSL_MSS_MCAN_ECC_REV_REVRTL_MASK                                       (0x0000F800U)
#define CSL_MSS_MCAN_ECC_REV_REVRTL_SHIFT                                      (0x0000000BU)
#define CSL_MSS_MCAN_ECC_REV_REVRTL_RESETVAL                                   (0x0000001DU)
#define CSL_MSS_MCAN_ECC_REV_REVRTL_MAX                                        (0x0000001FU)

#define CSL_MSS_MCAN_ECC_REV_MODULE_ID_MASK                                    (0x0FFF0000U)
#define CSL_MSS_MCAN_ECC_REV_MODULE_ID_SHIFT                                   (0x00000010U)
#define CSL_MSS_MCAN_ECC_REV_MODULE_ID_RESETVAL                                (0x000006A0U)
#define CSL_MSS_MCAN_ECC_REV_MODULE_ID_MAX                                     (0x00000FFFU)

#define CSL_MSS_MCAN_ECC_REV_BU_MASK                                           (0x30000000U)
#define CSL_MSS_MCAN_ECC_REV_BU_SHIFT                                          (0x0000001CU)
#define CSL_MSS_MCAN_ECC_REV_BU_RESETVAL                                       (0x00000002U)
#define CSL_MSS_MCAN_ECC_REV_BU_MAX                                            (0x00000003U)

#define CSL_MSS_MCAN_ECC_REV_SCHEME_MASK                                       (0xC0000000U)
#define CSL_MSS_MCAN_ECC_REV_SCHEME_SHIFT                                      (0x0000001EU)
#define CSL_MSS_MCAN_ECC_REV_SCHEME_RESETVAL                                   (0x00000001U)
#define CSL_MSS_MCAN_ECC_REV_SCHEME_MAX                                        (0x00000003U)

#define CSL_MSS_MCAN_ECC_REV_RESETVAL                                          (0x66A0EA00U)

/* VECTOR */

#define CSL_MSS_MCAN_ECC_VECTOR_ECC_VEC_MASK                                   (0x000007FFU)
#define CSL_MSS_MCAN_ECC_VECTOR_ECC_VEC_SHIFT                                  (0x00000000U)
#define CSL_MSS_MCAN_ECC_VECTOR_ECC_VEC_RESETVAL                               (0x00000000U)
#define CSL_MSS_MCAN_ECC_VECTOR_ECC_VEC_MAX                                    (0x000007FFU)

#define CSL_MSS_MCAN_ECC_VECTOR_NU0_MASK                                       (0x00007800U)
#define CSL_MSS_MCAN_ECC_VECTOR_NU0_SHIFT                                      (0x0000000BU)
#define CSL_MSS_MCAN_ECC_VECTOR_NU0_RESETVAL                                   (0x00000000U)
#define CSL_MSS_MCAN_ECC_VECTOR_NU0_MAX                                        (0x0000000FU)

#define CSL_MSS_MCAN_ECC_VECTOR_RD_SVBUS_MASK                                  (0x00008000U)
#define CSL_MSS_MCAN_ECC_VECTOR_RD_SVBUS_SHIFT                                 (0x0000000FU)
#define CSL_MSS_MCAN_ECC_VECTOR_RD_SVBUS_RESETVAL                              (0x00000000U)
#define CSL_MSS_MCAN_ECC_VECTOR_RD_SVBUS_MAX                                   (0x00000001U)

#define CSL_MSS_MCAN_ECC_VECTOR_RD_SVBUS_ADDR_MASK                             (0x00FF0000U)
#define CSL_MSS_MCAN_ECC_VECTOR_RD_SVBUS_ADDR_SHIFT                            (0x00000010U)
#define CSL_MSS_MCAN_ECC_VECTOR_RD_SVBUS_ADDR_RESETVAL                         (0x00000000U)
#define CSL_MSS_MCAN_ECC_VECTOR_RD_SVBUS_ADDR_MAX                              (0x000000FFU)

#define CSL_MSS_MCAN_ECC_VECTOR_RD_SVBUS_DONE_MASK                             (0x01000000U)
#define CSL_MSS_MCAN_ECC_VECTOR_RD_SVBUS_DONE_SHIFT                            (0x00000018U)
#define CSL_MSS_MCAN_ECC_VECTOR_RD_SVBUS_DONE_RESETVAL                         (0x00000000U)
#define CSL_MSS_MCAN_ECC_VECTOR_RD_SVBUS_DONE_MAX                              (0x00000001U)

#define CSL_MSS_MCAN_ECC_VECTOR_NU1_MASK                                       (0xFE000000U)
#define CSL_MSS_MCAN_ECC_VECTOR_NU1_SHIFT                                      (0x00000019U)
#define CSL_MSS_MCAN_ECC_VECTOR_NU1_RESETVAL                                   (0x00000000U)
#define CSL_MSS_MCAN_ECC_VECTOR_NU1_MAX                                        (0x0000007FU)

#define CSL_MSS_MCAN_ECC_VECTOR_RESETVAL                                       (0x00000000U)

/* STAT */

#define CSL_MSS_MCAN_ECC_STAT_NUM_RAMS_MASK                                    (0x000007FFU)
#define CSL_MSS_MCAN_ECC_STAT_NUM_RAMS_SHIFT                                   (0x00000000U)
#define CSL_MSS_MCAN_ECC_STAT_NUM_RAMS_RESETVAL                                (0x00000002U)
#define CSL_MSS_MCAN_ECC_STAT_NUM_RAMS_MAX                                     (0x000007FFU)

#define CSL_MSS_MCAN_ECC_STAT_NU2_MASK                                         (0xFFFFF800U)
#define CSL_MSS_MCAN_ECC_STAT_NU2_SHIFT                                        (0x0000000BU)
#define CSL_MSS_MCAN_ECC_STAT_NU2_RESETVAL                                     (0x00000000U)
#define CSL_MSS_MCAN_ECC_STAT_NU2_MAX                                          (0x001FFFFFU)

#define CSL_MSS_MCAN_ECC_STAT_RESETVAL                                         (0x00000002U)

/* CTRL */

#define CSL_MSS_MCAN_ECC_CTRL_ECC_EN_MASK                                      (0x00000001U)
#define CSL_MSS_MCAN_ECC_CTRL_ECC_EN_SHIFT                                     (0x00000000U)
#define CSL_MSS_MCAN_ECC_CTRL_ECC_EN_RESETVAL                                  (0x00000001U)
#define CSL_MSS_MCAN_ECC_CTRL_ECC_EN_MAX                                       (0x00000001U)

#define CSL_MSS_MCAN_ECC_CTRL_ECC_CHK_MASK                                     (0x00000002U)
#define CSL_MSS_MCAN_ECC_CTRL_ECC_CHK_SHIFT                                    (0x00000001U)
#define CSL_MSS_MCAN_ECC_CTRL_ECC_CHK_RESETVAL                                 (0x00000001U)
#define CSL_MSS_MCAN_ECC_CTRL_ECC_CHK_MAX                                      (0x00000001U)

#define CSL_MSS_MCAN_ECC_CTRL_EN_RMW_MASK                                      (0x00000004U)
#define CSL_MSS_MCAN_ECC_CTRL_EN_RMW_SHIFT                                     (0x00000002U)
#define CSL_MSS_MCAN_ECC_CTRL_EN_RMW_RESETVAL                                  (0x00000001U)
#define CSL_MSS_MCAN_ECC_CTRL_EN_RMW_MAX                                       (0x00000001U)

#define CSL_MSS_MCAN_ECC_CTRL_FORCE_SEC_MASK                                   (0x00000008U)
#define CSL_MSS_MCAN_ECC_CTRL_FORCE_SEC_SHIFT                                  (0x00000003U)
#define CSL_MSS_MCAN_ECC_CTRL_FORCE_SEC_RESETVAL                               (0x00000000U)
#define CSL_MSS_MCAN_ECC_CTRL_FORCE_SEC_MAX                                    (0x00000001U)

#define CSL_MSS_MCAN_ECC_CTRL_FORCE_DED_MASK                                   (0x00000010U)
#define CSL_MSS_MCAN_ECC_CTRL_FORCE_DED_SHIFT                                  (0x00000004U)
#define CSL_MSS_MCAN_ECC_CTRL_FORCE_DED_RESETVAL                               (0x00000000U)
#define CSL_MSS_MCAN_ECC_CTRL_FORCE_DED_MAX                                    (0x00000001U)

#define CSL_MSS_MCAN_ECC_CTRL_FORCE_N_ROW_MASK                                 (0x00000020U)
#define CSL_MSS_MCAN_ECC_CTRL_FORCE_N_ROW_SHIFT                                (0x00000005U)
#define CSL_MSS_MCAN_ECC_CTRL_FORCE_N_ROW_RESETVAL                             (0x00000000U)
#define CSL_MSS_MCAN_ECC_CTRL_FORCE_N_ROW_MAX                                  (0x00000001U)

#define CSL_MSS_MCAN_ECC_CTRL_ERROR_ONCE_MASK                                  (0x00000040U)
#define CSL_MSS_MCAN_ECC_CTRL_ERROR_ONCE_SHIFT                                 (0x00000006U)
#define CSL_MSS_MCAN_ECC_CTRL_ERROR_ONCE_RESETVAL                              (0x00000000U)
#define CSL_MSS_MCAN_ECC_CTRL_ERROR_ONCE_MAX                                   (0x00000001U)

#define CSL_MSS_MCAN_ECC_CTRL_CHECK_MASK                                       (0x00000100U)
#define CSL_MSS_MCAN_ECC_CTRL_CHECK_SHIFT                                      (0x00000008U)
#define CSL_MSS_MCAN_ECC_CTRL_CHECK_RESETVAL                                   (0x00000001U)
#define CSL_MSS_MCAN_ECC_CTRL_CHECK_MAX                                        (0x00000001U)

#define CSL_MSS_MCAN_ECC_CTRL_CHECK_MASK                                       (0x00000100U)
#define CSL_MSS_MCAN_ECC_CTRL_CHECK_SHIFT                                      (0x00000008U)
#define CSL_MSS_MCAN_ECC_CTRL_CHECK_RESETVAL                                   (0x00000001U)
#define CSL_MSS_MCAN_ECC_CTRL_CHECK_MAX                                        (0x00000001U)

#define CSL_MSS_MCAN_ECC_CTRL_NU3_MASK                                         (0xFFFFFE00U)
#define CSL_MSS_MCAN_ECC_CTRL_NU3_SHIFT                                        (0x00000009U)
#define CSL_MSS_MCAN_ECC_CTRL_NU3_RESETVAL                                     (0x00000000U)
#define CSL_MSS_MCAN_ECC_CTRL_NU3_MAX                                          (0x007FFFFFU)

#define CSL_MSS_MCAN_ECC_CTRL_RESETVAL                                         (0x00000107U)

/* ERR_CTRL1 */

#define CSL_MSS_MCAN_ECC_ERR_CTRL1_ECC_ROW_MASK                                (0xFFFFFFFFU)
#define CSL_MSS_MCAN_ECC_ERR_CTRL1_ECC_ROW_SHIFT                               (0x00000000U)
#define CSL_MSS_MCAN_ECC_ERR_CTRL1_ECC_ROW_RESETVAL                            (0x00000000U)
#define CSL_MSS_MCAN_ECC_ERR_CTRL1_ECC_ROW_MAX                                 (0xFFFFFFFFU)

#define CSL_MSS_MCAN_ECC_ERR_CTRL1_RESETVAL                                    (0x00000000U)

/* ERR_CTRL2 */

#define CSL_MSS_MCAN_ECC_ERR_CTRL2_ECC_BIT1_MASK                               (0x0000FFFFU)
#define CSL_MSS_MCAN_ECC_ERR_CTRL2_ECC_BIT1_SHIFT                              (0x00000000U)
#define CSL_MSS_MCAN_ECC_ERR_CTRL2_ECC_BIT1_RESETVAL                           (0x00000000U)
#define CSL_MSS_MCAN_ECC_ERR_CTRL2_ECC_BIT1_MAX                                (0x0000FFFFU)

#define CSL_MSS_MCAN_ECC_ERR_CTRL2_ECC_BIT2_MASK                               (0xFFFF0000U)
#define CSL_MSS_MCAN_ECC_ERR_CTRL2_ECC_BIT2_SHIFT                              (0x00000010U)
#define CSL_MSS_MCAN_ECC_ERR_CTRL2_ECC_BIT2_RESETVAL                           (0x00000000U)
#define CSL_MSS_MCAN_ECC_ERR_CTRL2_ECC_BIT2_MAX                                (0x0000FFFFU)

#define CSL_MSS_MCAN_ECC_ERR_CTRL2_RESETVAL                                    (0x00000000U)

/* ERR_STAT1 */

#define CSL_MSS_MCAN_ECC_ERR_STAT1_ECC_SEC_MASK                                (0x00000003U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_ECC_SEC_SHIFT                               (0x00000000U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_ECC_SEC_RESETVAL                            (0x00000000U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_ECC_SEC_MAX                                 (0x00000003U)

#define CSL_MSS_MCAN_ECC_ERR_STAT1_ECC_DED_MASK                                (0x0000000CU)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_ECC_DED_SHIFT                               (0x00000002U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_ECC_DED_RESETVAL                            (0x00000000U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_ECC_DED_MAX                                 (0x00000003U)

#define CSL_MSS_MCAN_ECC_ERR_STAT1_ECC_OTHER_MASK                              (0x00000010U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_ECC_OTHER_SHIFT                             (0x00000004U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_ECC_OTHER_RESETVAL                          (0x00000000U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_ECC_OTHER_MAX                               (0x00000001U)

#define CSL_MSS_MCAN_ECC_ERR_STAT1_ECC_PAR_MASK                                (0x00000060U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_ECC_PAR_SHIFT                               (0x00000005U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_ECC_PAR_RESETVAL                            (0x00000000U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_ECC_PAR_MAX                                 (0x00000003U)

#define CSL_MSS_MCAN_ECC_ERR_STAT1_ECC_CTRL_REG_MASK                           (0x00000080U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_ECC_CTRL_REG_SHIFT                          (0x00000007U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_ECC_CTRL_REG_RESETVAL                       (0x00000000U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_ECC_CTRL_REG_MAX                            (0x00000001U)

#define CSL_MSS_MCAN_ECC_ERR_STAT1_CLR_ECC_SEC_MASK                            (0x00000300U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_CLR_ECC_SEC_SHIFT                           (0x00000008U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_CLR_ECC_SEC_RESETVAL                        (0x00000000U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_CLR_ECC_SEC_MAX                             (0x00000003U)

#define CSL_MSS_MCAN_ECC_ERR_STAT1_CLR_ECC_DED_MASK                            (0x00000C00U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_CLR_ECC_DED_SHIFT                           (0x0000000AU)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_CLR_ECC_DED_RESETVAL                        (0x00000000U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_CLR_ECC_DED_MAX                             (0x00000003U)

#define CSL_MSS_MCAN_ECC_ERR_STAT1_CLR_ECC_OTHER_MASK                          (0x00001000U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_CLR_ECC_OTHER_SHIFT                         (0x0000000CU)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_CLR_ECC_OTHER_RESETVAL                      (0x00000000U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_CLR_ECC_OTHER_MAX                           (0x00000001U)

#define CSL_MSS_MCAN_ECC_ERR_STAT1_CLR_ECC_PAR_MASK                            (0x00006000U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_CLR_ECC_PAR_SHIFT                           (0x0000000DU)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_CLR_ECC_PAR_RESETVAL                        (0x00000000U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_CLR_ECC_PAR_MAX                             (0x00000003U)

#define CSL_MSS_MCAN_ECC_ERR_STAT1_CLR_ECC_CTRL_REG_MASK                       (0x00008000U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_CLR_ECC_CTRL_REG_SHIFT                      (0x0000000FU)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_CLR_ECC_CTRL_REG_RESETVAL                   (0x00000000U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_CLR_ECC_CTRL_REG_MAX                        (0x00000001U)

#define CSL_MSS_MCAN_ECC_ERR_STAT1_ECC_BIT1_STS_MASK                           (0xFFFF0000U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_ECC_BIT1_STS_SHIFT                          (0x00000010U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_ECC_BIT1_STS_RESETVAL                       (0x00000000U)
#define CSL_MSS_MCAN_ECC_ERR_STAT1_ECC_BIT1_STS_MAX                            (0x0000FFFFU)

#define CSL_MSS_MCAN_ECC_ERR_STAT1_RESETVAL                                    (0x00000000U)

/* ERR_STAT2 */

#define CSL_MSS_MCAN_ECC_ERR_STAT2_ECC_ROW_MASK                                (0xFFFFFFFFU)
#define CSL_MSS_MCAN_ECC_ERR_STAT2_ECC_ROW_SHIFT                               (0x00000000U)
#define CSL_MSS_MCAN_ECC_ERR_STAT2_ECC_ROW_RESETVAL                            (0x00000000U)
#define CSL_MSS_MCAN_ECC_ERR_STAT2_ECC_ROW_MAX                                 (0xFFFFFFFFU)

#define CSL_MSS_MCAN_ECC_ERR_STAT2_RESETVAL                                    (0x00000000U)

/* ERR_STAT3 */

#define CSL_MSS_MCAN_ECC_ERR_STAT3_NU4_MASK                                    (0x00000001U)
#define CSL_MSS_MCAN_ECC_ERR_STAT3_NU4_SHIFT                                   (0x00000000U)
#define CSL_MSS_MCAN_ECC_ERR_STAT3_NU4_RESETVAL                                (0x00000000U)
#define CSL_MSS_MCAN_ECC_ERR_STAT3_NU4_MAX                                     (0x00000001U)

#define CSL_MSS_MCAN_ECC_ERR_STAT3_TIMEOUT_PEND_MASK                           (0x00000002U)
#define CSL_MSS_MCAN_ECC_ERR_STAT3_TIMEOUT_PEND_SHIFT                          (0x00000001U)
#define CSL_MSS_MCAN_ECC_ERR_STAT3_TIMEOUT_PEND_RESETVAL                       (0x00000000U)
#define CSL_MSS_MCAN_ECC_ERR_STAT3_TIMEOUT_PEND_MAX                            (0x00000001U)

#define CSL_MSS_MCAN_ECC_ERR_STAT3_NU5_MASK                                    (0x000001FCU)
#define CSL_MSS_MCAN_ECC_ERR_STAT3_NU5_SHIFT                                   (0x00000002U)
#define CSL_MSS_MCAN_ECC_ERR_STAT3_NU5_RESETVAL                                (0x00000000U)
#define CSL_MSS_MCAN_ECC_ERR_STAT3_NU5_MAX                                     (0x0000007FU)

#define CSL_MSS_MCAN_ECC_ERR_STAT3_CLR_TIMEOUT_PEND_MASK                       (0x00000200U)
#define CSL_MSS_MCAN_ECC_ERR_STAT3_CLR_TIMEOUT_PEND_SHIFT                      (0x00000009U)
#define CSL_MSS_MCAN_ECC_ERR_STAT3_CLR_TIMEOUT_PEND_RESETVAL                   (0x00000000U)
#define CSL_MSS_MCAN_ECC_ERR_STAT3_CLR_TIMEOUT_PEND_MAX                        (0x00000001U)

#define CSL_MSS_MCAN_ECC_ERR_STAT3_NU6_MASK                                    (0xFFFFFC00U)
#define CSL_MSS_MCAN_ECC_ERR_STAT3_NU6_SHIFT                                   (0x0000000AU)
#define CSL_MSS_MCAN_ECC_ERR_STAT3_NU6_RESETVAL                                (0x00000000U)
#define CSL_MSS_MCAN_ECC_ERR_STAT3_NU6_MAX                                     (0x003FFFFFU)

#define CSL_MSS_MCAN_ECC_ERR_STAT3_RESETVAL                                    (0x00000000U)

/* SEC_EOI_REG */

#define CSL_MSS_MCAN_ECC_SEC_EOI_REG_SEC_EOI_WR_MASK                           (0x00000001U)
#define CSL_MSS_MCAN_ECC_SEC_EOI_REG_SEC_EOI_WR_SHIFT                          (0x00000000U)
#define CSL_MSS_MCAN_ECC_SEC_EOI_REG_SEC_EOI_WR_RESETVAL                       (0x00000000U)
#define CSL_MSS_MCAN_ECC_SEC_EOI_REG_SEC_EOI_WR_MAX                            (0x00000001U)

#define CSL_MSS_MCAN_ECC_SEC_EOI_REG_NU7_MASK                                  (0xFFFFFFFEU)
#define CSL_MSS_MCAN_ECC_SEC_EOI_REG_NU7_SHIFT                                 (0x00000001U)
#define CSL_MSS_MCAN_ECC_SEC_EOI_REG_NU7_RESETVAL                              (0x00000000U)
#define CSL_MSS_MCAN_ECC_SEC_EOI_REG_NU7_MAX                                   (0x7FFFFFFFU)

#define CSL_MSS_MCAN_ECC_SEC_EOI_REG_RESETVAL                                  (0x00000000U)

/* SEC_STATUS_REG0 */

#define CSL_MSS_MCAN_ECC_SEC_STATUS_REG0_SEC_PEND_MASK                         (0x00000001U)
#define CSL_MSS_MCAN_ECC_SEC_STATUS_REG0_SEC_PEND_SHIFT                        (0x00000000U)
#define CSL_MSS_MCAN_ECC_SEC_STATUS_REG0_SEC_PEND_RESETVAL                     (0x00000000U)
#define CSL_MSS_MCAN_ECC_SEC_STATUS_REG0_SEC_PEND_MAX                          (0x00000001U)

#define CSL_MSS_MCAN_ECC_SEC_STATUS_REG0_CTRL_EDC_VBUSS_PEND_MASK              (0x00000002U)
#define CSL_MSS_MCAN_ECC_SEC_STATUS_REG0_CTRL_EDC_VBUSS_PEND_SHIFT             (0x00000001U)
#define CSL_MSS_MCAN_ECC_SEC_STATUS_REG0_CTRL_EDC_VBUSS_PEND_RESETVAL          (0x00000000U)
#define CSL_MSS_MCAN_ECC_SEC_STATUS_REG0_CTRL_EDC_VBUSS_PEND_MAX               (0x00000001U)

#define CSL_MSS_MCAN_ECC_SEC_STATUS_REG0_NU8_MASK                              (0xFFFFFFFCU)
#define CSL_MSS_MCAN_ECC_SEC_STATUS_REG0_NU8_SHIFT                             (0x00000002U)
#define CSL_MSS_MCAN_ECC_SEC_STATUS_REG0_NU8_RESETVAL                          (0x00000000U)
#define CSL_MSS_MCAN_ECC_SEC_STATUS_REG0_NU8_MAX                               (0x3FFFFFFFU)

#define CSL_MSS_MCAN_ECC_SEC_STATUS_REG0_RESETVAL                              (0x00000000U)

/* SEC_ENABLE_SET_REG0 */

#define CSL_MSS_MCAN_ECC_SEC_ENABLE_SET_REG0_SEC_EN_SET_MASK                   (0x00000001U)
#define CSL_MSS_MCAN_ECC_SEC_ENABLE_SET_REG0_SEC_EN_SET_SHIFT                  (0x00000000U)
#define CSL_MSS_MCAN_ECC_SEC_ENABLE_SET_REG0_SEC_EN_SET_RESETVAL               (0x00000000U)
#define CSL_MSS_MCAN_ECC_SEC_ENABLE_SET_REG0_SEC_EN_SET_MAX                    (0x00000001U)

#define CSL_MSS_MCAN_ECC_SEC_ENABLE_SET_REG0_CTRL_EDC_VBUSS_ENABLE_SET_MASK    (0x00000002U)
#define CSL_MSS_MCAN_ECC_SEC_ENABLE_SET_REG0_CTRL_EDC_VBUSS_ENABLE_SET_SHIFT   (0x00000001U)
#define CSL_MSS_MCAN_ECC_SEC_ENABLE_SET_REG0_CTRL_EDC_VBUSS_ENABLE_SET_RESETVAL (0x00000000U)
#define CSL_MSS_MCAN_ECC_SEC_ENABLE_SET_REG0_CTRL_EDC_VBUSS_ENABLE_SET_MAX     (0x00000001U)

#define CSL_MSS_MCAN_ECC_SEC_ENABLE_SET_REG0_NU9_MASK                          (0xFFFFFFFCU)
#define CSL_MSS_MCAN_ECC_SEC_ENABLE_SET_REG0_NU9_SHIFT                         (0x00000002U)
#define CSL_MSS_MCAN_ECC_SEC_ENABLE_SET_REG0_NU9_RESETVAL                      (0x00000000U)
#define CSL_MSS_MCAN_ECC_SEC_ENABLE_SET_REG0_NU9_MAX                           (0x3FFFFFFFU)

#define CSL_MSS_MCAN_ECC_SEC_ENABLE_SET_REG0_RESETVAL                          (0x00000000U)

/* SEC_ENABLE_CLR_REG0 */

#define CSL_MSS_MCAN_ECC_SEC_ENABLE_CLR_REG0_SEC_EN_CLR_MASK                   (0x00000001U)
#define CSL_MSS_MCAN_ECC_SEC_ENABLE_CLR_REG0_SEC_EN_CLR_SHIFT                  (0x00000000U)
#define CSL_MSS_MCAN_ECC_SEC_ENABLE_CLR_REG0_SEC_EN_CLR_RESETVAL               (0x00000000U)
#define CSL_MSS_MCAN_ECC_SEC_ENABLE_CLR_REG0_SEC_EN_CLR_MAX                    (0x00000001U)

#define CSL_MSS_MCAN_ECC_SEC_ENABLE_CLR_REG0_CTRL_EDC_VBUSS_ENABLE_CLR_MASK    (0x00000002U)
#define CSL_MSS_MCAN_ECC_SEC_ENABLE_CLR_REG0_CTRL_EDC_VBUSS_ENABLE_CLR_SHIFT   (0x00000001U)
#define CSL_MSS_MCAN_ECC_SEC_ENABLE_CLR_REG0_CTRL_EDC_VBUSS_ENABLE_CLR_RESETVAL (0x00000000U)
#define CSL_MSS_MCAN_ECC_SEC_ENABLE_CLR_REG0_CTRL_EDC_VBUSS_ENABLE_CLR_MAX     (0x00000001U)

#define CSL_MSS_MCAN_ECC_SEC_ENABLE_CLR_REG0_NU10_MASK                         (0xFFFFFFFCU)
#define CSL_MSS_MCAN_ECC_SEC_ENABLE_CLR_REG0_NU10_SHIFT                        (0x00000002U)
#define CSL_MSS_MCAN_ECC_SEC_ENABLE_CLR_REG0_NU10_RESETVAL                     (0x00000000U)
#define CSL_MSS_MCAN_ECC_SEC_ENABLE_CLR_REG0_NU10_MAX                          (0x3FFFFFFFU)

#define CSL_MSS_MCAN_ECC_SEC_ENABLE_CLR_REG0_RESETVAL                          (0x00000000U)

/* DED_EOI_REG */

#define CSL_MSS_MCAN_ECC_DED_EOI_REG_DED_EOI_WR_MASK                           (0x00000001U)
#define CSL_MSS_MCAN_ECC_DED_EOI_REG_DED_EOI_WR_SHIFT                          (0x00000000U)
#define CSL_MSS_MCAN_ECC_DED_EOI_REG_DED_EOI_WR_RESETVAL                       (0x00000000U)
#define CSL_MSS_MCAN_ECC_DED_EOI_REG_DED_EOI_WR_MAX                            (0x00000001U)

#define CSL_MSS_MCAN_ECC_DED_EOI_REG_NU11_MASK                                 (0xFFFFFFFEU)
#define CSL_MSS_MCAN_ECC_DED_EOI_REG_NU11_SHIFT                                (0x00000001U)
#define CSL_MSS_MCAN_ECC_DED_EOI_REG_NU11_RESETVAL                             (0x00000000U)
#define CSL_MSS_MCAN_ECC_DED_EOI_REG_NU11_MAX                                  (0x7FFFFFFFU)

#define CSL_MSS_MCAN_ECC_DED_EOI_REG_RESETVAL                                  (0x00000000U)

/* DED_STATUS_REG0 */

#define CSL_MSS_MCAN_ECC_DED_STATUS_REG0_DED_PEND_MASK                         (0x00000001U)
#define CSL_MSS_MCAN_ECC_DED_STATUS_REG0_DED_PEND_SHIFT                        (0x00000000U)
#define CSL_MSS_MCAN_ECC_DED_STATUS_REG0_DED_PEND_RESETVAL                     (0x00000000U)
#define CSL_MSS_MCAN_ECC_DED_STATUS_REG0_DED_PEND_MAX                          (0x00000001U)

#define CSL_MSS_MCAN_ECC_DED_STATUS_REG0_CTRL_EDC_VBUSS_PEND_MASK              (0x00000002U)
#define CSL_MSS_MCAN_ECC_DED_STATUS_REG0_CTRL_EDC_VBUSS_PEND_SHIFT             (0x00000001U)
#define CSL_MSS_MCAN_ECC_DED_STATUS_REG0_CTRL_EDC_VBUSS_PEND_RESETVAL          (0x00000000U)
#define CSL_MSS_MCAN_ECC_DED_STATUS_REG0_CTRL_EDC_VBUSS_PEND_MAX               (0x00000001U)

#define CSL_MSS_MCAN_ECC_DED_STATUS_REG0_NU12_MASK                             (0xFFFFFFFCU)
#define CSL_MSS_MCAN_ECC_DED_STATUS_REG0_NU12_SHIFT                            (0x00000002U)
#define CSL_MSS_MCAN_ECC_DED_STATUS_REG0_NU12_RESETVAL                         (0x00000000U)
#define CSL_MSS_MCAN_ECC_DED_STATUS_REG0_NU12_MAX                              (0x3FFFFFFFU)

#define CSL_MSS_MCAN_ECC_DED_STATUS_REG0_RESETVAL                              (0x00000000U)

/* DED_ENABLE_SET_REG0 */

#define CSL_MSS_MCAN_ECC_DED_ENABLE_SET_REG0_DED_EN_SET_MASK                   (0x00000001U)
#define CSL_MSS_MCAN_ECC_DED_ENABLE_SET_REG0_DED_EN_SET_SHIFT                  (0x00000000U)
#define CSL_MSS_MCAN_ECC_DED_ENABLE_SET_REG0_DED_EN_SET_RESETVAL               (0x00000000U)
#define CSL_MSS_MCAN_ECC_DED_ENABLE_SET_REG0_DED_EN_SET_MAX                    (0x00000001U)

#define CSL_MSS_MCAN_ECC_DED_ENABLE_SET_REG0_CTRL_EDC_VBUSS_ENABLE_SET_MASK    (0x00000002U)
#define CSL_MSS_MCAN_ECC_DED_ENABLE_SET_REG0_CTRL_EDC_VBUSS_ENABLE_SET_SHIFT   (0x00000001U)
#define CSL_MSS_MCAN_ECC_DED_ENABLE_SET_REG0_CTRL_EDC_VBUSS_ENABLE_SET_RESETVAL (0x00000000U)
#define CSL_MSS_MCAN_ECC_DED_ENABLE_SET_REG0_CTRL_EDC_VBUSS_ENABLE_SET_MAX     (0x00000001U)

#define CSL_MSS_MCAN_ECC_DED_ENABLE_SET_REG0_NU13_MASK                         (0xFFFFFFFCU)
#define CSL_MSS_MCAN_ECC_DED_ENABLE_SET_REG0_NU13_SHIFT                        (0x00000002U)
#define CSL_MSS_MCAN_ECC_DED_ENABLE_SET_REG0_NU13_RESETVAL                     (0x00000000U)
#define CSL_MSS_MCAN_ECC_DED_ENABLE_SET_REG0_NU13_MAX                          (0x3FFFFFFFU)

#define CSL_MSS_MCAN_ECC_DED_ENABLE_SET_REG0_RESETVAL                          (0x00000000U)

/* DED_ENABLE_CLR_REG0 */

#define CSL_MSS_MCAN_ECC_DED_ENABLE_CLR_REG0_DED_EN_CLR_MASK                   (0x00000001U)
#define CSL_MSS_MCAN_ECC_DED_ENABLE_CLR_REG0_DED_EN_CLR_SHIFT                  (0x00000000U)
#define CSL_MSS_MCAN_ECC_DED_ENABLE_CLR_REG0_DED_EN_CLR_RESETVAL               (0x00000000U)
#define CSL_MSS_MCAN_ECC_DED_ENABLE_CLR_REG0_DED_EN_CLR_MAX                    (0x00000001U)

#define CSL_MSS_MCAN_ECC_DED_ENABLE_CLR_REG0_CTRL_EDC_VBUSS_ENABLE_CLR_MASK    (0x00000002U)
#define CSL_MSS_MCAN_ECC_DED_ENABLE_CLR_REG0_CTRL_EDC_VBUSS_ENABLE_CLR_SHIFT   (0x00000001U)
#define CSL_MSS_MCAN_ECC_DED_ENABLE_CLR_REG0_CTRL_EDC_VBUSS_ENABLE_CLR_RESETVAL (0x00000000U)
#define CSL_MSS_MCAN_ECC_DED_ENABLE_CLR_REG0_CTRL_EDC_VBUSS_ENABLE_CLR_MAX     (0x00000001U)

#define CSL_MSS_MCAN_ECC_DED_ENABLE_CLR_REG0_NU14_MASK                         (0xFFFFFFFCU)
#define CSL_MSS_MCAN_ECC_DED_ENABLE_CLR_REG0_NU14_SHIFT                        (0x00000002U)
#define CSL_MSS_MCAN_ECC_DED_ENABLE_CLR_REG0_NU14_RESETVAL                     (0x00000000U)
#define CSL_MSS_MCAN_ECC_DED_ENABLE_CLR_REG0_NU14_MAX                          (0x3FFFFFFFU)

#define CSL_MSS_MCAN_ECC_DED_ENABLE_CLR_REG0_RESETVAL                          (0x00000000U)

/* AGGR_ENABLE_SET */

#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_SET_PARITY_MASK                           (0x00000001U)
#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_SET_PARITY_SHIFT                          (0x00000000U)
#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_SET_PARITY_RESETVAL                       (0x00000000U)
#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_SET_PARITY_MAX                            (0x00000001U)

#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_SET_TIMEOUT_MASK                          (0x00000002U)
#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_SET_TIMEOUT_SHIFT                         (0x00000001U)
#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_SET_TIMEOUT_RESETVAL                      (0x00000000U)
#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_SET_TIMEOUT_MAX                           (0x00000001U)

#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_SET_NU15_MASK                             (0xFFFFFFFCU)
#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_SET_NU15_SHIFT                            (0x00000002U)
#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_SET_NU15_RESETVAL                         (0x00000000U)
#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_SET_NU15_MAX                              (0x3FFFFFFFU)

#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_SET_RESETVAL                              (0x00000000U)

/* AGGR_ENABLE_CLR */

#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_CLR_PARITY_MASK                           (0x00000001U)
#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_CLR_PARITY_SHIFT                          (0x00000000U)
#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_CLR_PARITY_RESETVAL                       (0x00000000U)
#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_CLR_PARITY_MAX                            (0x00000001U)

#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_CLR_TIMEOUT_MASK                          (0x00000002U)
#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_CLR_TIMEOUT_SHIFT                         (0x00000001U)
#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_CLR_TIMEOUT_RESETVAL                      (0x00000000U)
#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_CLR_TIMEOUT_MAX                           (0x00000001U)

#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_CLR_NU16_MASK                             (0xFFFFFFFCU)
#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_CLR_NU16_SHIFT                            (0x00000002U)
#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_CLR_NU16_RESETVAL                         (0x00000000U)
#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_CLR_NU16_MAX                              (0x3FFFFFFFU)

#define CSL_MSS_MCAN_ECC_AGGR_ENABLE_CLR_RESETVAL                              (0x00000000U)

/* AGGR_STATUS_SET */

#define CSL_MSS_MCAN_ECC_AGGR_STATUS_SET_PARITY_MASK                           (0x00000003U)
#define CSL_MSS_MCAN_ECC_AGGR_STATUS_SET_PARITY_SHIFT                          (0x00000000U)
#define CSL_MSS_MCAN_ECC_AGGR_STATUS_SET_PARITY_RESETVAL                       (0x00000000U)
#define CSL_MSS_MCAN_ECC_AGGR_STATUS_SET_PARITY_MAX                            (0x00000003U)

#define CSL_MSS_MCAN_ECC_AGGR_STATUS_SET_TIMEOUT_MASK                          (0x0000000CU)
#define CSL_MSS_MCAN_ECC_AGGR_STATUS_SET_TIMEOUT_SHIFT                         (0x00000002U)
#define CSL_MSS_MCAN_ECC_AGGR_STATUS_SET_TIMEOUT_RESETVAL                      (0x00000000U)
#define CSL_MSS_MCAN_ECC_AGGR_STATUS_SET_TIMEOUT_MAX                           (0x00000003U)

#define CSL_MSS_MCAN_ECC_AGGR_STATUS_SET_NU17_MASK                             (0xFFFFFFF0U)
#define CSL_MSS_MCAN_ECC_AGGR_STATUS_SET_NU17_SHIFT                            (0x00000004U)
#define CSL_MSS_MCAN_ECC_AGGR_STATUS_SET_NU17_RESETVAL                         (0x00000000U)
#define CSL_MSS_MCAN_ECC_AGGR_STATUS_SET_NU17_MAX                              (0x0FFFFFFFU)

#define CSL_MSS_MCAN_ECC_AGGR_STATUS_SET_RESETVAL                              (0x00000000U)

/* AGGR_STATUS_CLR */

#define CSL_MSS_MCAN_ECC_AGGR_STATUS_CLR_PARITY_MASK                           (0x00000003U)
#define CSL_MSS_MCAN_ECC_AGGR_STATUS_CLR_PARITY_SHIFT                          (0x00000000U)
#define CSL_MSS_MCAN_ECC_AGGR_STATUS_CLR_PARITY_RESETVAL                       (0x00000000U)
#define CSL_MSS_MCAN_ECC_AGGR_STATUS_CLR_PARITY_MAX                            (0x00000003U)

#define CSL_MSS_MCAN_ECC_AGGR_STATUS_CLR_TIMEOUT_MASK                          (0x0000000CU)
#define CSL_MSS_MCAN_ECC_AGGR_STATUS_CLR_TIMEOUT_SHIFT                         (0x00000002U)
#define CSL_MSS_MCAN_ECC_AGGR_STATUS_CLR_TIMEOUT_RESETVAL                      (0x00000000U)
#define CSL_MSS_MCAN_ECC_AGGR_STATUS_CLR_TIMEOUT_MAX                           (0x00000003U)

#define CSL_MSS_MCAN_ECC_AGGR_STATUS_CLR_NU18_MASK                             (0xFFFFFFF0U)
#define CSL_MSS_MCAN_ECC_AGGR_STATUS_CLR_NU18_SHIFT                            (0x00000004U)
#define CSL_MSS_MCAN_ECC_AGGR_STATUS_CLR_NU18_RESETVAL                         (0x00000000U)
#define CSL_MSS_MCAN_ECC_AGGR_STATUS_CLR_NU18_MAX                              (0x0FFFFFFFU)

#define CSL_MSS_MCAN_ECC_AGGR_STATUS_CLR_RESETVAL                              (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
