/*
 *  Copyright (c) Texas Instruments Incorporated 2020
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

/*!
 * \file  generic_phy.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        generic Ethernet PHY.
 */

#ifndef GENERIC_PHY_H_
#define GENERIC_PHY_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include "enetextphy_priv.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/* PHY Register Definitions */

/*! \brief Basic Mode Control Register (BMCR) */
#define PHY_BMCR                              (0x00U)

/*! \brief Basic Mode Status Register (BMSR) */
#define PHY_BMSR                              (0x01U)

/*! \brief PHY Identifier Register #1 (PHYIDR1) */
#define PHY_PHYIDR1                           (0x02U)

/*! \brief PHY Identifier Register #2 (PHYIDR2) */
#define PHY_PHYIDR2                           (0x03U)

/*! \brief Auto-Negotiation Advertisement Register (ANAR) */
#define PHY_ANAR                              (0x04U)

/*! \brief Auto-Negotiation Link Partner Abilitiy Register (ANLPAR) */
#define PHY_ANLPAR                            (0x05U)

/*! \brief Auto-Negotiation Expansion Register (ANER) */
#define PHY_ANER                              (0x06U)

/*! \brief Auto-Negotiation NP TX Register (ANNPTR) */
#define PHY_ANNPTR                            (0x07U)

/*! \brief Auto-Neg NP RX Register (ANNPRR) */
#define PHY_ANNPRR                            (0x08U)

/*! \brief 1000BASE-T Control Register (GIGCR) */
#define PHY_GIGCR                             (0x09U)

/*! \brief 1000BASE-T Status Register (GIGSR) */
#define PHY_GIGSR                             (0x0AU)

/*! \brief MMD Access Control Register */
#define PHY_MMD_CR                            (0x0DU)

/*! \brief MMD Access Data Register */
#define PHY_MMD_DR                            (0x0EU)

/*! \brief 1000BASE-T Extended Status Register (GIGESR) */
#define PHY_GIGESR                            (0x0FU)

/* BMCR register definitions */
#define BMCR_RESET                            ENETEXTPHY_BIT(15)
#define BMCR_LOOPBACK                         ENETEXTPHY_BIT(14)
#define BMCR_SPEED100                         ENETEXTPHY_BIT(13)
#define BMCR_ANEN                             ENETEXTPHY_BIT(12)
#define BMCR_PWRDOWN                          ENETEXTPHY_BIT(11)
#define BMCR_ISOLATE                          ENETEXTPHY_BIT(10)
#define BMCR_ANRESTART                        ENETEXTPHY_BIT(9)
#define BMCR_FD                               ENETEXTPHY_BIT(8)
#define BMCR_SPEED1000                        ENETEXTPHY_BIT(6)

/* BMSR register definitions */
#define BMSR_100FD                            ENETEXTPHY_BIT(14)
#define BMSR_100HD                            ENETEXTPHY_BIT(13)
#define BMSR_10FD                             ENETEXTPHY_BIT(12)
#define BMSR_10HD                             ENETEXTPHY_BIT(11)
#define BMSR_GIGEXTSTS                        ENETEXTPHY_BIT(8)
#define BMSR_ANCOMPLETE                       ENETEXTPHY_BIT(5)
#define BMSR_ANCAPABLE                        ENETEXTPHY_BIT(3)
#define BMSR_LINKSTS                          ENETEXTPHY_BIT(2)
#define BMSR_EXTCAP                           ENETEXTPHY_BIT(0)

/* PHYIDR1 register definitions */
#define PHYIDR1_OUI_OFFSET                    (6U)

/* PHYIDR2 register definitions */
#define PHYIDR2_OUI_MASK                      (0xFC00U)
#define PHYIDR2_VMDL_MASK                     (0x03F0U)
#define PHYIDR2_VREV_MASK                     (0x000FU)
#define PHYIDR2_OUI_OFFSET                    (10U)
#define PHYIDR2_VMDL_OFFSET                   (4U)
#define PHYIDR2_VREV_OFFSET                   (0U)

/* ANAR register definitions */
#define ANAR_100FD                            ENETEXTPHY_BIT(8)
#define ANAR_100HD                            ENETEXTPHY_BIT(7)
#define ANAR_10FD                             ENETEXTPHY_BIT(6)
#define ANAR_10HD                             ENETEXTPHY_BIT(5)
#define ANAR_802P3                            ENETEXTPHY_BIT(0)
#define ANAR_100                              (ANAR_100FD | ANAR_100HD)
#define ANAR_10                               (ANAR_10FD | ANAR_10HD)

/* ANLPAR register definitions */
#define ANLPAR_100FD                          ENETEXTPHY_BIT(8)
#define ANLPAR_100HD                          ENETEXTPHY_BIT(7)
#define ANLPAR_10FD                           ENETEXTPHY_BIT(6)
#define ANLPAR_10HD                           ENETEXTPHY_BIT(5)
#define ANLPAR_100                            (ANLPAR_100FD | ANLPAR_100HD)
#define ANLPAR_10                             (ANLPAR_10FD | ANLPAR_10HD)

/* GIGCR register definitions */
#define GIGCR_MASTERCFG                       ENETEXTPHY_BIT(12)
#define GIGCR_MASTEREN                        ENETEXTPHY_BIT(11)
#define GIGCR_1000FD                          ENETEXTPHY_BIT(9)
#define GIGCR_1000HD                          ENETEXTPHY_BIT(8)
#define GIGCR_1000                            (GIGCR_1000FD | GIGCR_1000HD)

/* GIGSR register definitions */
#define GIGSR_MASTERRES                       ENETEXTPHY_BIT(14)
#define GIGSR_LOCALSTS                        ENETEXTPHY_BIT(13)
#define GIGSR_LPARSTS                         ENETEXTPHY_BIT(12)
#define GIGSR_1000FD                          ENETEXTPHY_BIT(11)
#define GIGSR_1000HD                          ENETEXTPHY_BIT(10)
#define GIGSR_1000                            (GIGSR_1000FD | GIGSR_1000HD)

/* MMD_CR register definitions */
#define MMD_CR_ADDR                           (0x0000U)
#define MMD_CR_DATA_NOPOSTINC                 (0x4000U)
#define MMD_CR_DATA_POSTINC_RW                (0x8000U)
#define MMD_CR_DATA_POSTINC_W                 (0xC000U)
#define MMD_CR_DEVADDR                        (0x001FU)

/* GIGESR register definitions */
#define GIGESR_1000FD                         ENETEXTPHY_BIT(13)
#define GIGESR_1000HD                         ENETEXTPHY_BIT(12)
#define GIGESR_1000                           (GIGESR_1000FD | GIGESR_1000HD)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

int32_t GenericExtPhy_readExtReg(EnetExtPhy_Handle hPhy,
                              uint32_t reg,
                              uint16_t *val);

int32_t GenericExtPhy_writeExtReg(EnetExtPhy_Handle hPhy,
                               uint32_t reg,
                               uint16_t val);

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* GENERIC_PHY_H_ */
