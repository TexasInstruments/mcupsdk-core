/********************************************************************
 * Copyright (C) 2003-2019 Texas Instruments Incorporated.
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
/*********************************************************************
* file: cslr_cpsgmii.h
*
* Brief: This file contains the Register Description for cpsgmii
*
*********************************************************************/
#ifndef CSLR_CPSGMII_V5_H
#define CSLR_CPSGMII_V5_H

/* CSL Modification:
 *  The file has been modified from the AUTOGEN file for the following
 *  reasons:-
 *      a) Modified the header file includes to be RTSC compliant
 */


#ifdef __cplusplus
extern "C" {
#endif

/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    volatile Uint32 IDVER_REG;
    volatile Uint32 SOFT_RESET_REG;
    volatile Uint8 RSVD0[8];
    volatile Uint32 CONTROL_REG;
    volatile Uint32 STATUS_REG;
    volatile Uint32 MR_ADV_ABILITY_REG;
    volatile Uint32 MR_NP_TX_REG;
    volatile Uint32 MR_LP_ADV_ABILITY_REG;
    volatile Uint32 MR_LP_NP_RX_REG;
    volatile Uint8 RSVD1[24];
    volatile Uint32 DIAG_CLEAR_REG;
    volatile Uint32 DIAG_CONTROL_REG;
    volatile Uint32 DIAG_STATUS_REG;

    /* Modified by the CSL team for:
     *
     *  (1) Adding extra padding to offset correctly
     *      to the next SGMII module start.
     */
#ifndef  CSLR_EMAC_NOMOD
    volatile Uint8  RSVD4[180];
#endif

} CSL_CpsgmiiRegs;

/* SGMII Port register structure for K3 devices.
 */
#ifndef  CSLR_EMAC_NOMOD
typedef struct  {
    CSL_CpsgmiiRegs SGMII_PORT [8];
} CSL_Cpsgmii_PortRegs;
#endif

/**************************************************************************\
* Field Definition Macros
\**************************************************************************/

/* idver_reg */

#define CSL_CPSGMII_IDVER_REG_MINOR_VER_MASK                                   (0x000000FFu)
#define CSL_CPSGMII_IDVER_REG_MINOR_VER_SHIFT                                  (0x00000000u)
#define CSL_CPSGMII_IDVER_REG_MINOR_VER_MAX                                    (0x000000FFu)
#define CSL_CPSGMII_IDVER_REG_MINOR_VER_RESETVAL                               (0x00000001u)

#define CSL_CPSGMII_IDVER_REG_MAJOR_VER_MASK                                   (0x00000700u)
#define CSL_CPSGMII_IDVER_REG_MAJOR_VER_SHIFT                                  (0x00000008u)
#define CSL_CPSGMII_IDVER_REG_MAJOR_VER_MAX                                    (0x00000007u)
#define CSL_CPSGMII_IDVER_REG_MAJOR_VER_RESETVAL                               (0x00000001u)

#define CSL_CPSGMII_IDVER_REG_RTL_VER_MASK                                     (0x0000F800u)
#define CSL_CPSGMII_IDVER_REG_RTL_VER_SHIFT                                    (0x0000000Bu)
#define CSL_CPSGMII_IDVER_REG_RTL_VER_MAX                                      (0x0000001Fu)
#define CSL_CPSGMII_IDVER_REG_RTL_VER_RESETVAL                                 (0x00000000u)

#define CSL_CPSGMII_IDVER_REG_TX_IDENT_MASK                                    (0xFFFF0000u)
#define CSL_CPSGMII_IDVER_REG_TX_IDENT_SHIFT                                   (0x00000010u)
#define CSL_CPSGMII_IDVER_REG_TX_IDENT_MAX                                     (0x0000FFFFu)
#define CSL_CPSGMII_IDVER_REG_TX_IDENT_RESETVAL                                (0x00004EC2u)

#define CSL_CPSGMII_IDVER_REG_RESETVAL                                         (0x4EC20101u)

/* soft_reset_reg */

#define CSL_CPSGMII_SOFT_RESET_REG_SOFT_RESET_MASK                             (0x00000001u)
#define CSL_CPSGMII_SOFT_RESET_REG_SOFT_RESET_SHIFT                            (0x00000000u)
#define CSL_CPSGMII_SOFT_RESET_REG_SOFT_RESET_MAX                              (0x00000001u)
#define CSL_CPSGMII_SOFT_RESET_REG_SOFT_RESET_RESETVAL                         (0x00000000u)

#define CSL_CPSGMII_SOFT_RESET_REG_RT_SOFT_RESET_MASK                          (0x00000002u)
#define CSL_CPSGMII_SOFT_RESET_REG_RT_SOFT_RESET_SHIFT                         (0x00000001u)
#define CSL_CPSGMII_SOFT_RESET_REG_RT_SOFT_RESET_MAX                           (0x00000001u)
#define CSL_CPSGMII_SOFT_RESET_REG_RT_SOFT_RESET_RESETVAL                      (0x00000000u)

#define CSL_CPSGMII_SOFT_RESET_REG_RESETVAL                                    (0x00000000u)

/* control_reg */

#define CSL_CPSGMII_CONTROL_REG_MR_AN_ENABLE_MASK                              (0x00000001u)
#define CSL_CPSGMII_CONTROL_REG_MR_AN_ENABLE_SHIFT                             (0x00000000u)
#define CSL_CPSGMII_CONTROL_REG_MR_AN_ENABLE_MAX                               (0x00000001u)
#define CSL_CPSGMII_CONTROL_REG_MR_AN_ENABLE_RESETVAL                          (0x00000000u)

#define CSL_CPSGMII_CONTROL_REG_MR_AN_RESTART_MASK                             (0x00000002u)
#define CSL_CPSGMII_CONTROL_REG_MR_AN_RESTART_SHIFT                            (0x00000001u)
#define CSL_CPSGMII_CONTROL_REG_MR_AN_RESTART_MAX                              (0x00000001u)
#define CSL_CPSGMII_CONTROL_REG_MR_AN_RESTART_RESETVAL                         (0x00000000u)

#define CSL_CPSGMII_CONTROL_REG_FAST_LINK_TIMER_MASK                           (0x00000004u)
#define CSL_CPSGMII_CONTROL_REG_FAST_LINK_TIMER_SHIFT                          (0x00000002u)
#define CSL_CPSGMII_CONTROL_REG_FAST_LINK_TIMER_MAX                            (0x00000001u)
#define CSL_CPSGMII_CONTROL_REG_FAST_LINK_TIMER_RESETVAL                       (0x00000000u)

#define CSL_CPSGMII_CONTROL_REG_MR_NP_LOADED_MASK                              (0x00000008u)
#define CSL_CPSGMII_CONTROL_REG_MR_NP_LOADED_SHIFT                             (0x00000003u)
#define CSL_CPSGMII_CONTROL_REG_MR_NP_LOADED_MAX                               (0x00000001u)
#define CSL_CPSGMII_CONTROL_REG_MR_NP_LOADED_RESETVAL                          (0x00000000u)

#define CSL_CPSGMII_CONTROL_REG_LOOPBACK_MASK                                  (0x00000010u)
#define CSL_CPSGMII_CONTROL_REG_LOOPBACK_SHIFT                                 (0x00000004u)
#define CSL_CPSGMII_CONTROL_REG_LOOPBACK_MAX                                   (0x00000001u)
#define CSL_CPSGMII_CONTROL_REG_LOOPBACK_RESETVAL                              (0x00000000u)

#define CSL_CPSGMII_CONTROL_REG_MASTER_MASK                                    (0x00000020u)
#define CSL_CPSGMII_CONTROL_REG_MASTER_SHIFT                                   (0x00000005u)
#define CSL_CPSGMII_CONTROL_REG_MASTER_MAX                                     (0x00000001u)
#define CSL_CPSGMII_CONTROL_REG_MASTER_RESETVAL                                (0x00000000u)

#define CSL_CPSGMII_CONTROL_REG_TEST_PATTERN_EN_MASK                           (0x00000040u)
#define CSL_CPSGMII_CONTROL_REG_TEST_PATTERN_EN_SHIFT                          (0x00000006u)
#define CSL_CPSGMII_CONTROL_REG_TEST_PATTERN_EN_MAX                            (0x00000001u)
#define CSL_CPSGMII_CONTROL_REG_TEST_PATTERN_EN_RESETVAL                       (0x00000000u)

#define CSL_CPSGMII_CONTROL_REG_RESETVAL                                       (0x00000000u)

/* status_reg */

#define CSL_CPSGMII_STATUS_REG_LINK_MASK                                       (0x00000001u)
#define CSL_CPSGMII_STATUS_REG_LINK_SHIFT                                      (0x00000000u)
#define CSL_CPSGMII_STATUS_REG_LINK_MAX                                        (0x00000001u)
#define CSL_CPSGMII_STATUS_REG_LINK_RESETVAL                                   (0x00000000u)

#define CSL_CPSGMII_STATUS_REG_AN_ERROR_MASK                                   (0x00000002u)
#define CSL_CPSGMII_STATUS_REG_AN_ERROR_SHIFT                                  (0x00000001u)
#define CSL_CPSGMII_STATUS_REG_AN_ERROR_MAX                                    (0x00000001u)
#define CSL_CPSGMII_STATUS_REG_AN_ERROR_RESETVAL                               (0x00000000u)

#define CSL_CPSGMII_STATUS_REG_MR_AN_COMPLETE_MASK                             (0x00000004u)
#define CSL_CPSGMII_STATUS_REG_MR_AN_COMPLETE_SHIFT                            (0x00000002u)
#define CSL_CPSGMII_STATUS_REG_MR_AN_COMPLETE_MAX                              (0x00000001u)
#define CSL_CPSGMII_STATUS_REG_MR_AN_COMPLETE_RESETVAL                         (0x00000000u)

#define CSL_CPSGMII_STATUS_REG_MR_PAGE_RX_MASK                                 (0x00000008u)
#define CSL_CPSGMII_STATUS_REG_MR_PAGE_RX_SHIFT                                (0x00000003u)
#define CSL_CPSGMII_STATUS_REG_MR_PAGE_RX_MAX                                  (0x00000001u)
#define CSL_CPSGMII_STATUS_REG_MR_PAGE_RX_RESETVAL                             (0x00000000u)

#define CSL_CPSGMII_STATUS_REG_LOCK_MASK                                       (0x00000010u)
#define CSL_CPSGMII_STATUS_REG_LOCK_SHIFT                                      (0x00000004u)
#define CSL_CPSGMII_STATUS_REG_LOCK_MAX                                        (0x00000001u)
#define CSL_CPSGMII_STATUS_REG_LOCK_RESETVAL                                   (0x00000000u)

#define CSL_CPSGMII_STATUS_REG_FIB_SIG_DETECT_MASK                             (0x00000020u)
#define CSL_CPSGMII_STATUS_REG_FIB_SIG_DETECT_SHIFT                            (0x00000005u)
#define CSL_CPSGMII_STATUS_REG_FIB_SIG_DETECT_MAX                              (0x00000001u)
#define CSL_CPSGMII_STATUS_REG_FIB_SIG_DETECT_RESETVAL                         (0x00000000u)

#define CSL_CPSGMII_STATUS_REG_RESETVAL                                        (0x00000000u)

/* mr_adv_ability_reg */

#define CSL_CPSGMII_MR_ADV_ABILITY_REG_MR_ADV_ABILITY_MASK                     (0x0000FFFFu)
#define CSL_CPSGMII_MR_ADV_ABILITY_REG_MR_ADV_ABILITY_SHIFT                    (0x00000000u)
#define CSL_CPSGMII_MR_ADV_ABILITY_REG_MR_ADV_ABILITY_MAX                      (0x0000FFFFu)
#define CSL_CPSGMII_MR_ADV_ABILITY_REG_MR_ADV_ABILITY_RESETVAL                 (0x00000000u)

#define CSL_CPSGMII_MR_ADV_ABILITY_REG_RESETVAL                                (0x00000000u)

/* mr_np_tx_reg */

#define CSL_CPSGMII_MR_NP_TX_REG_MR_NP_TX_MASK                                 (0x0000FFFFu)
#define CSL_CPSGMII_MR_NP_TX_REG_MR_NP_TX_SHIFT                                (0x00000000u)
#define CSL_CPSGMII_MR_NP_TX_REG_MR_NP_TX_MAX                                  (0x0000FFFFu)
#define CSL_CPSGMII_MR_NP_TX_REG_MR_NP_TX_RESETVAL                             (0x00000000u)

#define CSL_CPSGMII_MR_NP_TX_REG_RESETVAL                                      (0x00000000u)

/* mr_lp_adv_ability_reg */

#define CSL_CPSGMII_MR_LP_ADV_ABILITY_REG_MR_LP_ADV_ABILITY_MASK               (0x0000FFFFu)
#define CSL_CPSGMII_MR_LP_ADV_ABILITY_REG_MR_LP_ADV_ABILITY_SHIFT              (0x00000000u)
#define CSL_CPSGMII_MR_LP_ADV_ABILITY_REG_MR_LP_ADV_ABILITY_MAX                (0x0000FFFFu)
#define CSL_CPSGMII_MR_LP_ADV_ABILITY_REG_MR_LP_ADV_ABILITY_RESETVAL           (0x00000000u)

#define CSL_CPSGMII_MR_LP_ADV_ABILITY_REG_RESETVAL                             (0x00000000u)

/* mr_lp_np_rx_reg */

#define CSL_CPSGMII_MR_LP_NP_RX_REG_MR_LP_NP_RX_MASK                           (0x0000FFFFu)
#define CSL_CPSGMII_MR_LP_NP_RX_REG_MR_LP_NP_RX_SHIFT                          (0x00000000u)
#define CSL_CPSGMII_MR_LP_NP_RX_REG_MR_LP_NP_RX_MAX                            (0x0000FFFFu)
#define CSL_CPSGMII_MR_LP_NP_RX_REG_MR_LP_NP_RX_RESETVAL                       (0x00000000u)

#define CSL_CPSGMII_MR_LP_NP_RX_REG_RESETVAL                                   (0x00000000u)

/* diag_clear_reg */

#define CSL_CPSGMII_DIAG_CLEAR_REG_DIAG_CLEAR_MASK                             (0x00000001u)
#define CSL_CPSGMII_DIAG_CLEAR_REG_DIAG_CLEAR_SHIFT                            (0x00000000u)
#define CSL_CPSGMII_DIAG_CLEAR_REG_DIAG_CLEAR_MAX                              (0x00000001u)
#define CSL_CPSGMII_DIAG_CLEAR_REG_DIAG_CLEAR_RESETVAL                         (0x00000000u)

#define CSL_CPSGMII_DIAG_CLEAR_REG_RESETVAL                                    (0x00000000u)

/* diag_control_reg */

#define CSL_CPSGMII_DIAG_CONTROL_REG_DIAG_EDGE_SEL_MASK                        (0x00000003u)
#define CSL_CPSGMII_DIAG_CONTROL_REG_DIAG_EDGE_SEL_SHIFT                       (0x00000000u)
#define CSL_CPSGMII_DIAG_CONTROL_REG_DIAG_EDGE_SEL_MAX                         (0x00000003u)
#define CSL_CPSGMII_DIAG_CONTROL_REG_DIAG_EDGE_SEL_RESETVAL                    (0x00000000u)

#define CSL_CPSGMII_DIAG_CONTROL_REG_DIAG_SM_SEL_MASK                          (0x00000070u)
#define CSL_CPSGMII_DIAG_CONTROL_REG_DIAG_SM_SEL_SHIFT                         (0x00000004u)
#define CSL_CPSGMII_DIAG_CONTROL_REG_DIAG_SM_SEL_MAX                           (0x00000007u)
#define CSL_CPSGMII_DIAG_CONTROL_REG_DIAG_SM_SEL_RESETVAL                      (0x00000000u)

#define CSL_CPSGMII_DIAG_CONTROL_REG_RESETVAL                                  (0x00000000u)

/* diag_status_reg */

#define CSL_CPSGMII_DIAG_STATUS_REG_DIAG_STATUS_MASK                           (0x0000FFFFu)
#define CSL_CPSGMII_DIAG_STATUS_REG_DIAG_STATUS_SHIFT                          (0x00000000u)
#define CSL_CPSGMII_DIAG_STATUS_REG_DIAG_STATUS_MAX                            (0x0000FFFFu)
#define CSL_CPSGMII_DIAG_STATUS_REG_DIAG_STATUS_RESETVAL                       (0x00000000u)

#define CSL_CPSGMII_DIAG_STATUS_REG_RESETVAL                                   (0x00000000u)


#ifdef __cplusplus
}
#endif

#endif
