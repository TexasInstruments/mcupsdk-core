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

#ifndef CSLR_MDIO_H
#define CSLR_MDIO_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <drivers/hw_include/cslr.h>

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/


typedef struct {
    volatile uint32_t USER_ACCESS_REG;           /* user_access_reg */
    volatile uint32_t USER_PHY_SEL_REG;          /* user_phy_sel_reg */
} CSL_MdioUser_groupRegs;


typedef struct {
    volatile uint32_t VERSION_REG;          /* version_reg */
    volatile uint32_t CONTROL_REG;               /* control_reg */
    volatile uint32_t ALIVE_REG;                 /* alive_reg */
    volatile uint32_t LINK_REG;                  /* link_reg */
    volatile uint32_t LINK_INT_RAW_REG;          /* link_int_raw_reg */
    volatile uint32_t LINK_INT_MASKED_REG;       /* link_int_masked_reg */
    volatile uint32_t LINK_INT_MASK_SET_REG;     /* link_int_mask_set_reg */
    volatile uint32_t LINK_INT_MASK_CLEAR_REG;   /* link_int_mask_clear_reg */
    volatile uint32_t USER_INT_RAW_REG;          /* user_int_raw_reg */
    volatile uint32_t USER_INT_MASKED_REG;       /* user_int_masked_reg */
    volatile uint32_t USER_INT_MASK_SET_REG;     /* user_int_mask_set_reg */
    volatile uint32_t USER_INT_MASK_CLEAR_REG;   /* user_int_mask_clear_reg */
    volatile uint32_t MANUAL_IF_REG;             /* manual_if_reg */
    volatile uint32_t POLL_REG;                  /* poll_reg */
    volatile uint32_t POLL_EN_REG;               /* poll_reg */
    volatile uint32_t CLAUS45_REG;               /* poll_reg */
    volatile uint32_t USER_ADDR0_REG;            /* poll_reg */
    volatile uint32_t USER_ADDR1_REG;            /* poll_reg */
    volatile uint8_t  Resv_128[56];
    CSL_MdioUser_groupRegs USER_GROUP[2];
} CSL_MdioRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_MDIO_VERSION_REG                                              (0x00000000U)
#define CSL_MDIO_CONTROL_REG                                                   (0x00000004U)
#define CSL_MDIO_ALIVE_REG                                                     (0x00000008U)
#define CSL_MDIO_LINK_REG                                                      (0x0000000CU)
#define CSL_MDIO_LINK_INT_RAW_REG                                              (0x00000010U)
#define CSL_MDIO_LINK_INT_MASKED_REG                                           (0x00000014U)
#define CSL_MDIO_LINK_INT_MASK_SET_REG                                         (0x00000018U)
#define CSL_MDIO_LINK_INT_MASK_CLEAR_REG                                       (0x0000001CU)
#define CSL_MDIO_USER_INT_RAW_REG                                              (0x00000020U)
#define CSL_MDIO_USER_INT_MASKED_REG                                           (0x00000024U)
#define CSL_MDIO_USER_INT_MASK_SET_REG                                         (0x00000028U)
#define CSL_MDIO_USER_INT_MASK_CLEAR_REG                                       (0x0000002CU)
#define CSL_MDIO_MANUAL_IF_REG                                                 (0x00000030U)
#define CSL_MDIO_POLL_REG                                                      (0x00000034U)
#define CSL_MDIO_POLL_EN_REG                                                   (0x00000038U)
#define CSL_MDIO_CLAUS45_REG                                                   (0x0000003CU)
#define CSL_MDIO_USER_ADDR0_REG                                                (0x00000040U)
#define CSL_MDIO_USER_ADDR1_REG                                                (0x00000044U)
#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG(USER_GROUP)                        (0x00000080U+((USER_GROUP)*0x8U))
#define CSL_MDIO_USER_GROUP_USER_PHY_SEL_REG(USER_GROUP)                       (0x00000084U+((USER_GROUP)*0x8U))

#ifndef CSL_MODIFICATION
#define CSL_MDIO_USER_ACCESS_REG(n)                                            ((uint32_t)0x80U + ((n) * ((uint32_t)(0x8U))))
#define CSL_MDIO_USER_PHY_SEL_REG(n)                                           ((uint32_t)0x84U + ((n) * ((uint32_t)(0x8U))))
#endif

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* USER_ACCESS_REG */

#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_DATA_MASK                          (0x0000FFFFU)
#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_DATA_SHIFT                         (0x00000000U)
#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_DATA_RESETVAL                      (0x00000000U)
#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_DATA_MAX                           (0x0000FFFFU)

#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_PHYADR_MASK                        (0x001F0000U)
#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_PHYADR_SHIFT                       (0x00000010U)
#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_PHYADR_RESETVAL                    (0x00000000U)
#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_PHYADR_MAX                         (0x0000001FU)

#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_REGADR_MASK                        (0x03E00000U)
#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_REGADR_SHIFT                       (0x00000015U)
#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_REGADR_RESETVAL                    (0x00000000U)
#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_REGADR_MAX                         (0x0000001FU)

#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_ACK_MASK                           (0x20000000U)
#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_ACK_SHIFT                          (0x0000001DU)
#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_ACK_RESETVAL                       (0x00000000U)
#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_ACK_MAX                            (0x00000001U)

#ifndef CSL_MODIFICATION
#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_ACK_PASS                           ((uint32_t)(1U))
#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_ACK_FAIL                           ((uint32_t)(0U))
#endif

#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_WRITE_MASK                         (0x40000000U)
#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_WRITE_SHIFT                        (0x0000001EU)
#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_WRITE_RESETVAL                     (0x00000000U)
#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_WRITE_MAX                          (0x00000001U)

#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_MASK                            (0x80000000U)
#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_SHIFT                           (0x0000001FU)
#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_RESETVAL                        (0x00000000U)
#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_MAX                             (0x00000001U)

#ifndef CSL_MODIFICATION
#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_EN_0x1                          ((uint32_t)(1U))
#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_EN_0x0                          ((uint32_t)(0U))
#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_READ                               ((uint32_t)(0U))
#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_WRITE                              ((uint32_t)(1U))
#endif

#define CSL_MDIO_USER_GROUP_USER_ACCESS_REG_RESETVAL                           (0x00000000U)

/* USER_PHY_SEL_REG */

#define CSL_MDIO_USER_GROUP_USER_PHY_SEL_REG_PHYADR_MON_MASK                   (0x0000001FU)
#define CSL_MDIO_USER_GROUP_USER_PHY_SEL_REG_PHYADR_MON_SHIFT                  (0x00000000U)
#define CSL_MDIO_USER_GROUP_USER_PHY_SEL_REG_PHYADR_MON_RESETVAL               (0x00000000U)
#define CSL_MDIO_USER_GROUP_USER_PHY_SEL_REG_PHYADR_MON_MAX                    (0x0000001FU)

#define CSL_MDIO_USER_GROUP_USER_PHY_SEL_REG_LINKINT_ENABLE_MASK               (0x00000040U)
#define CSL_MDIO_USER_GROUP_USER_PHY_SEL_REG_LINKINT_ENABLE_SHIFT              (0x00000006U)
#define CSL_MDIO_USER_GROUP_USER_PHY_SEL_REG_LINKINT_ENABLE_RESETVAL           (0x00000000U)
#define CSL_MDIO_USER_GROUP_USER_PHY_SEL_REG_LINKINT_ENABLE_MAX                (0x00000001U)

#define CSL_MDIO_USER_GROUP_USER_PHY_SEL_REG_LINKSEL_MASK                      (0x00000080U)
#define CSL_MDIO_USER_GROUP_USER_PHY_SEL_REG_LINKSEL_SHIFT                     (0x00000007U)
#define CSL_MDIO_USER_GROUP_USER_PHY_SEL_REG_LINKSEL_RESETVAL                  (0x00000000U)
#define CSL_MDIO_USER_GROUP_USER_PHY_SEL_REG_LINKSEL_MAX                       (0x00000001U)

#define CSL_MDIO_USER_GROUP_USER_PHY_SEL_REG_RESETVAL                          (0x00000000U)

/* VERSION_REG */

#define CSL_MDIO_VERSION_REG_REVMINOR_MASK                                     (0x0000003FU)
#define CSL_MDIO_VERSION_REG_REVMINOR_SHIFT                                    (0x00000000U)
#define CSL_MDIO_VERSION_REG_REVMINOR_RESETVAL                                 (0x00000007U)
#define CSL_MDIO_VERSION_REG_REVMINOR_MAX                                      (0x0000003FU)

#define CSL_MDIO_VERSION_REG_REVMAJ_MASK                                       (0x00000700U)
#define CSL_MDIO_VERSION_REG_REVMAJ_SHIFT                                      (0x00000008U)
#define CSL_MDIO_VERSION_REG_REVMAJ_RESETVAL                                   (0x00000001U)
#define CSL_MDIO_VERSION_REG_REVMAJ_MAX                                        (0x00000007U)

#define CSL_MDIO_VERSION_REG_MODID_MASK                                        (0x0FFF0000U)
#define CSL_MDIO_VERSION_REG_MODID_SHIFT                                       (0x00000010U)
#define CSL_MDIO_VERSION_REG_MODID_RESETVAL                                    (0x00000007U)
#define CSL_MDIO_VERSION_REG_MODID_MAX                                         (0x00000FFFU)

#define CSL_MDIO_VERSION_REG_SCHEME_MASK                                       (0xC0000000U)
#define CSL_MDIO_VERSION_REG_SCHEME_SHIFT                                      (0x0000001EU)
#define CSL_MDIO_VERSION_REG_SCHEME_RESETVAL                                   (0x00000000U)
#define CSL_MDIO_VERSION_REG_SCHEME_MAX                                        (0x00000003U)

#define CSL_MDIO_VERSION_REG_BU_MASK                                           (0x30000000U)
#define CSL_MDIO_VERSION_REG_BU_SHIFT                                          (0x0000001CU)
#define CSL_MDIO_VERSION_REG_BU_RESETVAL                                       (0x00000000U)
#define CSL_MDIO_VERSION_REG_BU_MAX                                            (0x00000003U)

#define CSL_MDIO_VERSION_REG_REVRTL_MASK                                       (0x0000F800U)
#define CSL_MDIO_VERSION_REG_REVRTL_SHIFT                                      (0x0000000BU)
#define CSL_MDIO_VERSION_REG_REVRTL_RESETVAL                                   (0x00000001U)
#define CSL_MDIO_VERSION_REG_REVRTL_MAX                                        (0x0000001FU)

#define CSL_MDIO_VERSION_REG_CUSTOM_MASK                                       (0x000000C0U)
#define CSL_MDIO_VERSION_REG_CUSTOM_SHIFT                                      (0x00000006U)
#define CSL_MDIO_VERSION_REG_CUSTOM_RESETVAL                                   (0x00000000U)
#define CSL_MDIO_VERSION_REG_CUSTOM_MAX                                        (0x00000003U)

#define CSL_MDIO_VERSION_REG_RESETVAL                                          (0x00070907U)

/* CONTROL_REG */

#define CSL_MDIO_CONTROL_REG_CLKDIV_MASK                                       (0x0000FFFFU)
#define CSL_MDIO_CONTROL_REG_CLKDIV_SHIFT                                      (0x00000000U)
#define CSL_MDIO_CONTROL_REG_CLKDIV_RESETVAL                                   (0x000000FFU)
#define CSL_MDIO_CONTROL_REG_CLKDIV_MAX                                        (0x0000FFFFU)

#define CSL_MDIO_CONTROL_REG_INT_TEST_ENABLE_MASK                              (0x00020000U)
#define CSL_MDIO_CONTROL_REG_INT_TEST_ENABLE_SHIFT                             (0x00000011U)
#define CSL_MDIO_CONTROL_REG_INT_TEST_ENABLE_RESETVAL                          (0x00000000U)
#define CSL_MDIO_CONTROL_REG_INT_TEST_ENABLE_MAX                               (0x00000001U)

#define CSL_MDIO_CONTROL_REG_FAULT_DETECT_ENABLE_MASK                          (0x00040000U)
#define CSL_MDIO_CONTROL_REG_FAULT_DETECT_ENABLE_SHIFT                         (0x00000012U)
#define CSL_MDIO_CONTROL_REG_FAULT_DETECT_ENABLE_RESETVAL                      (0x00000000U)
#define CSL_MDIO_CONTROL_REG_FAULT_DETECT_ENABLE_MAX                           (0x00000001U)

#ifndef CSL_MODIFICATION
#define CSL_MDIO_CONTROL_REG_FAULT_DETECT_ENABLE_0X1                           ((uint32_t)(1U))
#endif
#define CSL_MDIO_CONTROL_REG_FAULT_MASK                                        (0x00080000U)
#define CSL_MDIO_CONTROL_REG_FAULT_SHIFT                                       (0x00000013U)
#define CSL_MDIO_CONTROL_REG_FAULT_RESETVAL                                    (0x00000000U)
#define CSL_MDIO_CONTROL_REG_FAULT_MAX                                         (0x00000001U)

#define CSL_MDIO_CONTROL_REG_PREAMBLE_MASK                                     (0x00100000U)
#define CSL_MDIO_CONTROL_REG_PREAMBLE_SHIFT                                    (0x00000014U)
#define CSL_MDIO_CONTROL_REG_PREAMBLE_RESETVAL                                 (0x00000000U)
#define CSL_MDIO_CONTROL_REG_PREAMBLE_MAX                                      (0x00000001U)
#ifndef CSL_MODIFICATION
#define CSL_MDIO_CONTROL_REG_PREAMBLE_EN_0X1                                   ((uint32_t)(1U))
#endif

#define CSL_MDIO_CONTROL_REG_HIGHEST_USER_CHANNEL_MASK                         (0x1F000000U)
#define CSL_MDIO_CONTROL_REG_HIGHEST_USER_CHANNEL_SHIFT                        (0x00000018U)
#define CSL_MDIO_CONTROL_REG_HIGHEST_USER_CHANNEL_RESETVAL                     (0x00000001U)
#define CSL_MDIO_CONTROL_REG_HIGHEST_USER_CHANNEL_MAX                          (0x0000001FU)

#ifndef CSL_MODIFICATION
#define CSL_MDIO_CONTROL_REG_ENABLE_NO                                         (0x00000000u)
#define CSL_MDIO_CONTROL_REG_ENABLE_YES                                        (0x00000001u)
#define CSL_MDIO_CONTROL_REG_ENABLE_0X1                                        ((uint32_t)(1U))
#endif

#define CSL_MDIO_CONTROL_REG_ENABLE_MASK                                       (0x40000000U)
#define CSL_MDIO_CONTROL_REG_ENABLE_SHIFT                                      (0x0000001EU)
#define CSL_MDIO_CONTROL_REG_ENABLE_RESETVAL                                   (0x00000000U)
#define CSL_MDIO_CONTROL_REG_ENABLE_MAX                                        (0x00000001U)

#define CSL_MDIO_CONTROL_REG_IDLE_MASK                                         (0x80000000U)
#define CSL_MDIO_CONTROL_REG_IDLE_SHIFT                                        (0x0000001FU)
#define CSL_MDIO_CONTROL_REG_IDLE_RESETVAL                                     (0x00000001U)
#define CSL_MDIO_CONTROL_REG_IDLE_MAX                                          (0x00000001U)

#define CSL_MDIO_CONTROL_REG_RESETVAL                                          (0x810000FFU)

/* ALIVE_REG */

#define CSL_MDIO_ALIVE_REG_ALIVE_MASK                                          (0xFFFFFFFFU)
#define CSL_MDIO_ALIVE_REG_ALIVE_SHIFT                                         (0x00000000U)
#define CSL_MDIO_ALIVE_REG_ALIVE_RESETVAL                                      (0x00000000U)
#define CSL_MDIO_ALIVE_REG_ALIVE_MAX                                           (0xFFFFFFFFU)

#define CSL_MDIO_ALIVE_REG_RESETVAL                                            (0x00000000U)

/* LINK_REG */

#define CSL_MDIO_LINK_REG_LINK_MASK                                            (0xFFFFFFFFU)
#define CSL_MDIO_LINK_REG_LINK_SHIFT                                           (0x00000000U)
#define CSL_MDIO_LINK_REG_LINK_RESETVAL                                        (0x00000000U)
#define CSL_MDIO_LINK_REG_LINK_MAX                                             (0xFFFFFFFFU)

#define CSL_MDIO_LINK_REG_RESETVAL                                             (0x00000000U)

/* LINK_INT_RAW_REG */

#define CSL_MDIO_LINK_INT_RAW_REG_LINKINTRAW_MASK                              (0x00000003U)
#define CSL_MDIO_LINK_INT_RAW_REG_LINKINTRAW_SHIFT                             (0x00000000U)
#define CSL_MDIO_LINK_INT_RAW_REG_LINKINTRAW_RESETVAL                          (0x00000000U)
#define CSL_MDIO_LINK_INT_RAW_REG_LINKINTRAW_MAX                               (0x00000003U)

#define CSL_MDIO_LINK_INT_RAW_REG_RESETVAL                                     (0x00000000U)

/* LINK_INT_MASKED_REG */

#define CSL_MDIO_LINK_INT_MASKED_REG_LINKINTMASKED_MASK                        (0x00000003U)
#define CSL_MDIO_LINK_INT_MASKED_REG_LINKINTMASKED_SHIFT                       (0x00000000U)
#define CSL_MDIO_LINK_INT_MASKED_REG_LINKINTMASKED_RESETVAL                    (0x00000000U)
#define CSL_MDIO_LINK_INT_MASKED_REG_LINKINTMASKED_MAX                         (0x00000003U)

#define CSL_MDIO_LINK_INT_MASKED_REG_RESETVAL                                  (0x00000000U)

/* LINK_INT_MASK_SET_REG */

#define CSL_MDIO_LINK_INT_MASK_SET_REG_LINKINTMASKSET_MASK                     (0x00000001U)
#define CSL_MDIO_LINK_INT_MASK_SET_REG_LINKINTMASKSET_SHIFT                    (0x00000000U)
#define CSL_MDIO_LINK_INT_MASK_SET_REG_LINKINTMASKSET_RESETVAL                 (0x00000000U)
#define CSL_MDIO_LINK_INT_MASK_SET_REG_LINKINTMASKSET_MAX                      (0x00000001U)

#define CSL_MDIO_LINK_INT_MASK_SET_REG_RESETVAL                                (0x00000000U)

/* LINK_INT_MASK_CLEAR_REG */

#define CSL_MDIO_LINK_INT_MASK_CLEAR_REG_LINKINTMASKCLR_MASK                   (0x00000001U)
#define CSL_MDIO_LINK_INT_MASK_CLEAR_REG_LINKINTMASKCLR_SHIFT                  (0x00000000U)
#define CSL_MDIO_LINK_INT_MASK_CLEAR_REG_LINKINTMASKCLR_RESETVAL               (0x00000000U)
#define CSL_MDIO_LINK_INT_MASK_CLEAR_REG_LINKINTMASKCLR_MAX                    (0x00000001U)

#define CSL_MDIO_LINK_INT_MASK_CLEAR_REG_RESETVAL                              (0x00000000U)

/* USER_INT_RAW_REG */

#define CSL_MDIO_USER_INT_RAW_REG_USERINTRAW_MASK                              (0x00000003U)
#define CSL_MDIO_USER_INT_RAW_REG_USERINTRAW_SHIFT                             (0x00000000U)
#define CSL_MDIO_USER_INT_RAW_REG_USERINTRAW_RESETVAL                          (0x00000000U)
#define CSL_MDIO_USER_INT_RAW_REG_USERINTRAW_MAX                               (0x00000003U)

#define CSL_MDIO_USER_INT_RAW_REG_RESETVAL                                     (0x00000000U)

/* USER_INT_MASKED_REG */

#define CSL_MDIO_USER_INT_MASKED_REG_USERINTMASKED_MASK                        (0x00000003U)
#define CSL_MDIO_USER_INT_MASKED_REG_USERINTMASKED_SHIFT                       (0x00000000U)
#define CSL_MDIO_USER_INT_MASKED_REG_USERINTMASKED_RESETVAL                    (0x00000000U)
#define CSL_MDIO_USER_INT_MASKED_REG_USERINTMASKED_MAX                         (0x00000003U)

#define CSL_MDIO_USER_INT_MASKED_REG_RESETVAL                                  (0x00000000U)

/* USER_INT_MASK_SET_REG */

#define CSL_MDIO_USER_INT_MASK_SET_REG_USERINTMASKSET_MASK                     (0x00000003U)
#define CSL_MDIO_USER_INT_MASK_SET_REG_USERINTMASKSET_SHIFT                    (0x00000000U)
#define CSL_MDIO_USER_INT_MASK_SET_REG_USERINTMASKSET_RESETVAL                 (0x00000000U)
#define CSL_MDIO_USER_INT_MASK_SET_REG_USERINTMASKSET_MAX                      (0x00000003U)

#define CSL_MDIO_USER_INT_MASK_SET_REG_RESETVAL                                (0x00000000U)

/* USER_INT_MASK_CLEAR_REG */

#define CSL_MDIO_USER_INT_MASK_CLEAR_REG_USERINTMASKCLR_MASK                   (0x00000003U)
#define CSL_MDIO_USER_INT_MASK_CLEAR_REG_USERINTMASKCLR_SHIFT                  (0x00000000U)
#define CSL_MDIO_USER_INT_MASK_CLEAR_REG_USERINTMASKCLR_RESETVAL               (0x00000000U)
#define CSL_MDIO_USER_INT_MASK_CLEAR_REG_USERINTMASKCLR_MAX                    (0x00000003U)

#define CSL_MDIO_USER_INT_MASK_CLEAR_REG_RESETVAL                              (0x00000000U)

/* MANUAL_IF_REG */

#define CSL_MDIO_MANUAL_IF_REG_MDIO_PIN_MASK                                   (0x00000001U)
#define CSL_MDIO_MANUAL_IF_REG_MDIO_PIN_SHIFT                                  (0x00000000U)
#define CSL_MDIO_MANUAL_IF_REG_MDIO_PIN_RESETVAL                               (0x00000000U)
#define CSL_MDIO_MANUAL_IF_REG_MDIO_PIN_MAX                                    (0x00000001U)

#define CSL_MDIO_MANUAL_IF_REG_MDIO_OE_MASK                                    (0x00000002U)
#define CSL_MDIO_MANUAL_IF_REG_MDIO_OE_SHIFT                                   (0x00000001U)
#define CSL_MDIO_MANUAL_IF_REG_MDIO_OE_RESETVAL                                (0x00000000U)
#define CSL_MDIO_MANUAL_IF_REG_MDIO_OE_MAX                                     (0x00000001U)

#define CSL_MDIO_MANUAL_IF_REG_MDIO_MDCLK_O_MASK                               (0x00000004U)
#define CSL_MDIO_MANUAL_IF_REG_MDIO_MDCLK_O_SHIFT                              (0x00000002U)
#define CSL_MDIO_MANUAL_IF_REG_MDIO_MDCLK_O_RESETVAL                           (0x00000000U)
#define CSL_MDIO_MANUAL_IF_REG_MDIO_MDCLK_O_MAX                                (0x00000001U)

#define CSL_MDIO_MANUAL_IF_REG_RESETVAL                                        (0x00000000U)

/* POLL_REG */

#define CSL_MDIO_POLL_REG_IPG_MASK                                             (0x000000FFU)
#define CSL_MDIO_POLL_REG_IPG_SHIFT                                            (0x00000000U)
#define CSL_MDIO_POLL_REG_IPG_RESETVAL                                         (0x00000000U)
#define CSL_MDIO_POLL_REG_IPG_MAX                                              (0x000000FFU)

#define CSL_MDIO_POLL_REG_STATECHANGEMODE_MASK                                 (0x40000000U)
#define CSL_MDIO_POLL_REG_STATECHANGEMODE_SHIFT                                (0x0000001EU)
#define CSL_MDIO_POLL_REG_STATECHANGEMODE_RESETVAL                             (0x00000000U)
#define CSL_MDIO_POLL_REG_STATECHANGEMODE_MAX                                  (0x00000001U)

#define CSL_MDIO_POLL_REG_MANUALMODE_MASK                                      (0x80000000U)
#define CSL_MDIO_POLL_REG_MANUALMODE_SHIFT                                     (0x0000001FU)
#define CSL_MDIO_POLL_REG_MANUALMODE_RESETVAL                                  (0x00000000U)
#define CSL_MDIO_POLL_REG_MANUALMODE_MAX                                       (0x00000001U)

#define CSL_MDIO_POLL_REG_RESETVAL                                             (0x00000000U)

/* POLL_EN_REG */

#define CSL_MDIO_POLL_EN_REG_POLL_EN_MASK                                      (0xFFFFFFFFU)
#define CSL_MDIO_POLL_EN_REG_POLL_EN_SHIFT                                     (0x00000000U)
#define CSL_MDIO_POLL_EN_REG_POLL_EN_RESETVAL                                  (0xFFFFFFFFU)
#define CSL_MDIO_POLL_EN_REG_POLL_EN_MAX                                       (0xFFFFFFFFU)

#define CSL_MDIO_POLL_EN_REG_RESETVAL                                          (0xFFFFFFFFU)

/* CLAUS45_REG */

#define CSL_MDIO_CLAUS45_REG_CLAUSE45_MASK                                     (0xFFFFFFFFU)
#define CSL_MDIO_CLAUS45_REG_CLAUSE45_SHIFT                                    (0x00000000U)
#define CSL_MDIO_CLAUS45_REG_CLAUSE45_RESETVAL                                 (0x00000000U)
#define CSL_MDIO_CLAUS45_REG_CLAUSE45_MAX                                      (0xFFFFFFFFU)

#define CSL_MDIO_CLAUS45_REG_RESETVAL                                          (0x00000000U)

/* USER_ADDR0_REG */

#define CSL_MDIO_USER_ADDR0_REG_USER_ADDR0_MASK                                (0x0000FFFFU)
#define CSL_MDIO_USER_ADDR0_REG_USER_ADDR0_SHIFT                               (0x00000000U)
#define CSL_MDIO_USER_ADDR0_REG_USER_ADDR0_RESETVAL                            (0x00000000U)
#define CSL_MDIO_USER_ADDR0_REG_USER_ADDR0_MAX                                 (0x0000FFFFU)

#define CSL_MDIO_USER_ADDR0_REG_RESETVAL                                       (0x00000000U)

/* USER_ADDR1_REG */

#define CSL_MDIO_USER_ADDR1_REG_USER_ADDR1_MASK                                (0x0000FFFFU)
#define CSL_MDIO_USER_ADDR1_REG_USER_ADDR1_SHIFT                               (0x00000000U)
#define CSL_MDIO_USER_ADDR1_REG_USER_ADDR1_RESETVAL                            (0x00000000U)
#define CSL_MDIO_USER_ADDR1_REG_USER_ADDR1_MAX                                 (0x0000FFFFU)

#define CSL_MDIO_USER_ADDR1_REG_RESETVAL                                       (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
