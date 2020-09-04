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
 * \file  dp83867_priv.h
 *
 * \brief This file contains private type definitions and helper macros for the
 *        DP83867 Ethernet PHY.
 */

#ifndef DP83867_PRIV_H_
#define DP83867_PRIV_H_

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

/*! \brief PHY Control Register (PHYCR) */
#define DP83867_PHYCR                         (0x10U)

/*! \brief PHY Status Register (PHYSTS) */
#define DP83867_PHYSTS                        (0x11U)

/*! \brief MII Interrupt Control Register (MICR) */
#define DP83867_MICR                          (0x12U)

/*! \brief Interrupt Status Register (ISR) */
#define DP83867_ISR                           (0x13U)

/*! \brief Configuration Register 2 (CFG2) */
#define DP83867_CFG2                          (0x14U)

/*! \brief Receive Error Counter Register (RECR) */
#define DP83867_RECR                          (0x15U)

/*! \brief BIST Control Register (BISCR) */
#define DP83867_BISCR                         (0x16U)

/*! \brief Status Register 2 (STS2) */
#define DP83867_STS2                          (0x17U)

/*! \brief LED Configuration Register 1 (LEDCR1) */
#define DP83867_LEDCR1                        (0x18U)

/*! \brief LED Configuration Register 2 (LEDCR2) */
#define DP83867_LEDCR2                        (0x19U)

/*! \brief LED Configuration Register 3 (LEDCR3) */
#define DP83867_LEDCR3                        (0x1AU)

/*! \brief Configuration Register 3 (CFG3) */
#define DP83867_CFG3                          (0x1EU)

/*! \brief Control Register (CTRL) */
#define DP83867_CTRL                          (0x1FU)

/*! \brief Fast Link Drop Threshold Configuration Register (FLD_THR_CFG) */
#define DP83867_FLDTHRCFG                     (0x2EU)

/*! \brief RGMII Control Register (RGMIICTL) */
#define DP83867_RGMIICTL                      (0x32U)

/*! \brief Viterbi Module Configuration Register (VTM_CFG) */
#define DP83867_VTMCFG                        (0x53U)

/*! \brief Strap Configuration Status Register 2 (STRAP_STS2) */
#define DP83867_STRAPSTS2                     (0x6FU)

/*! \brief RGMII Delay Control Register (RGMIIDCTL) */
#define DP83867_RGMIIDCTL                     (0x86U)

/*! \brief Loopback Configuration Register (LOOPCR) */
#define DP83867_LOOPCR                        (0xFEU)

/*! \brief DSP Feedforward Equalizer Configuration Register (DSP_FFE_CFG) */
#define DP83867_DSPFFECFG                     (0x12CU)

/*! \brief I/O Configuration Register (IO_MUX_CFG) */
#define DP83867_IOMUXCFG                      (0x170U)

/*! \brief GPIO Mux Control Register (GPIO_MUX_CTRL). RGZ devices only */
#define DP83867_GPIOMUXCTRL                   (0x172U)

/* PHYCR register definitions */
#define PHYCR_TXFIFODEPTH_MASK                (0xC000U)
#define PHYCR_TXFIFODEPTH_8B                  (0xC000U)
#define PHYCR_TXFIFODEPTH_6B                  (0x8000U)
#define PHYCR_TXFIFODEPTH_4B                  (0x4000U)
#define PHYCR_TXFIFODEPTH_3B                  (0x0000U)
#define PHYCR_MDICROSSOVER_MASK               (0x0060U)
#define PHYCR_MDICROSSOVER_AUTO               (0x0040U)
#define PHYCR_MDICROSSOVER_MDIX               (0x0020U)
#define PHYCR_MDICROSSOVER_MDI                (0x0000U)

/* LEDCR1 register definitions */
#define LEDCR1_LEDGPIOSEL_MASK                (0xF000U)
#define LEDCR1_LEDGPIOSEL_OFFSET              (12U)
#define LEDCR1_LED2SEL_MASK                   (0x0F00U)
#define LEDCR1_LED2SEL_OFFSET                 (8U)
#define LEDCR1_LED1SEL_MASK                   (0x00F0U)
#define LEDCR1_LED1SEL_OFFSET                 (4U)
#define LEDCR1_LED0SEL_MASK                   (0x000FU)
#define LEDCR1_LED0SEL_OFFSET                 (0U)

#define LEDCR_RXERR                           (0xEU)
#define LEDCR_RXTXERR                         (0xDU)
#define LEDCR_BLINKACT                        (0xBU)
#define LEDCR_FD                              (0xAU)
#define LEDCR_100M1GLINK                      (0x9U)
#define LEDCR_10M100MLINK                     (0x8U)
#define LEDCR_10MLINK                         (0x7U)
#define LEDCR_100MLINK                        (0x6U)
#define LEDCR_1GLINK                          (0x5U)
#define LEDCR_COLLDET                         (0x4U)
#define LEDCR_RXACT                           (0x3U)
#define LEDCR_TXACT                           (0x2U)
#define LEDCR_RXTXACT                         (0x1U)
#define LEDCR_LINK                            (0x0U)

/* LEDCR1 register definitions */
#define LEDCR1_LED0SEL_MASK                   (0x000FU)
#define LEDCR1_LED0SEL_OFFSET                 (0U)
#define LEDCR1_LED1SEL_MASK                   (0x00F0U)
#define LEDCR1_LED1SEL_OFFSET                 (4U)
#define LEDCR1_LED2SEL_MASK                   (0x0F00U)
#define LEDCR1_LED2SEL_OFFSET                 (8U)
#define LEDCR1_LED3SEL_MASK                   (0xF000U)
#define LEDCR1_LED3SEL_OFFSET                 (12U)

/* CFG3 register definitions */
#define CFG3_ROBUSTAUTOMDIX                   ENETEXTPHY_BIT(9)

/* CTRL register definitions */
#define CTRL_SWRESET                          ENETEXTPHY_BIT(15)
#define CTRL_SWRESTART                        ENETEXTPHY_BIT(14)

/* RGMIICTL register definitions */
#define RGMIICTL_RGMIIEN                      ENETEXTPHY_BIT(7)
#define RGMIICTL_TXCLKDLY                     ENETEXTPHY_BIT(1)
#define RGMIICTL_RXCLKDLY                     ENETEXTPHY_BIT(0)

/* FLDTHRCFG register definitions */
#define FLDTHRCFG_FLDTHR_MASK                 (0x0007U)

/* VTMCFG register definitions */
#define VTMCFG_IDLETHR_MASK                   (0x000FU)

/*! STRAPSTS2 register definitions */
#define STRAPSTS2_FLD_MASK                    (0x0400U)

/* RGMIIDCTL register definitions */
#define RGMIIDCTL_TXDLYCTRL_MASK              (0x00F0U)
#define RGMIIDCTL_TXDLYCTRL_OFFSET            (4U)
#define RGMIIDCTL_RXDLYCTRL_MASK              (0x000FU)
#define RGMIIDCTL_RXDLYCTRL_OFFSET            (0U)
#define RGMIIDCTL_DELAY_MAX                   (4000U) /* 4.00 ns */
#define RGMIIDCTL_DELAY_STEP                  (250U)  /* 0.25 ns */

/* LOOPCR register definitions */
#define LOOPCR_CFG_LOOPBACK                   (0xE720)
#define LOOPCR_CFG_NORMAL                     (0xE721)

/* DSPFFECFG register definitions */
#define DSPFFECFG_FFEEQ_MASK                  (0x3FFU)
#define DSPFFECFG_FFEEQ_SHORTCABLE            (0x281U)

/* IOMUXCFG register definitions */
#define IOMUXCFG_IOIMPEDANCE_MASK             (0x001FU)
#define IOMUXCFG_IOIMPEDANCE_MIN              (35000U) /* 35 ohms */
#define IOMUXCFG_IOIMPEDANCE_MAX              (70000U) /* 70 ohms */
#define IOMUXCFG_IOIMPEDANCE_RANGE            (IOMUXCFG_IOIMPEDANCE_MAX - \
                                               IOMUXCFG_IOIMPEDANCE_MIN)

/* GPIOMUXCTRL register definitions */
#define GPIOMUXCTRL_GPIO1_MASK                (0x00F0U)
#define GPIOMUXCTRL_GPIO1_OFFSET              (4U)
#define GPIOMUXCTRL_GPIO0_MASK                (0x000FU)
#define GPIOMUXCTRL_GPIO0_OFFSET              (0U)

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

/* None */

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

#endif /* DP83867_PRIV_H_ */
