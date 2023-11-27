/*
 *  Copyright (c) Texas Instruments Incorporated 2023
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
 * \file  app_extphyconfig.c
 *
 * \brief This file contains the implementation of the DP83826e PHY.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include "app_extphyconfig.h"
#include "enet_apputils.h"
#include "ti_drivers_config.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define DP83826_OUI                           (0x080028U)
#define DP83826_MODEL                         (0x11U)
#define DP83826_REV                           (0x01U)

#define GPIOMUX_END                      (-1)
#define GPIO_DEFINE_INIT {                               \
                           {CSL_MCU_GPIO0_BASE, 5},      \
                                                         \
                           {CSL_GPIO0_BASE, 32},         \
                                                         \
                           {CSL_GPIO0_BASE, 31},         \
                                                         \
                           {GPIOMUX_END, GPIOMUX_END}    \
}

#define GPIO_LED_TEST2_PIN      (0U)
#define GPIO_RESET_ICSS1_PHY    (1U)
#define GPIO_RESET_ICSS1_PHY2    (2U)
#define GPIO_ARRAY_MAX          (GPIO_RESET_ICSS1_PHY2+2)

typedef struct GPIO_Function_g
{
    uint32_t    gpioBaseAddr;
    uint32_t    pinNum;
} GPIO_Function_g_t;

GPIO_Function_g_t GpioPins[GPIO_ARRAY_MAX] = GPIO_DEFINE_INIT;

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static bool Dp83826_isPhyDevSupported(EnetPhy_Handle hPhy,
                                      const EnetPhy_Version *version);

static bool Dp83826_isMacModeSupported(EnetPhy_Handle hPhy,
                                       EnetPhy_Mii mii);

static int32_t Dp83826_config(EnetPhy_Handle hPhy,
                              const EnetPhy_Cfg *cfg,
                              EnetPhy_Mii mii);

static void Dp83826_reset(EnetPhy_Handle hPhy);

static void Dp83826_enableAutoMdix(EnetPhy_Handle hPhy,
                                   bool enable);

static int32_t Dp83826_setOutputImpedance(EnetPhy_Handle hPhy,
                                          uint32_t impedance);

static void Dp83826_printRegs(EnetPhy_Handle hPhy);

static void PHY_Reg_Write(unsigned int baseAddr,unsigned int phyAddr,unsigned int DevAddr,unsigned short data);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

EnetPhy_Drv gEnetPhyDrvDp83826 =
{
    .name               = "dp83826",
    .isPhyDevSupported  = Dp83826_isPhyDevSupported,
    .isMacModeSupported = Dp83826_isMacModeSupported,
    .config             = Dp83826_config,
    .reset              = Dp83826_reset,
    .isResetComplete    = GenericPhy_isResetComplete,
    .readExtReg         = GenericPhy_readExtReg,
    .writeExtReg        = GenericPhy_writeExtReg,
    .printRegs          = Dp83826_printRegs,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void Dp83826_initCfg(Dp83826_Cfg *cfg)
{
    /* No extended config parameters at the moment */
}

static bool Dp83826_isPhyDevSupported(EnetPhy_Handle hPhy,
                                      const EnetPhy_Version *version)
{
    bool supported = false;

    if ((version->oui == DP83826_OUI) &&
        (version->model == DP83826_MODEL) &&
        (version->revision == DP83826_REV))
    {
        supported = true;
    }

    return supported;
}

static bool Dp83826_isMacModeSupported(EnetPhy_Handle hPhy,
                                       EnetPhy_Mii mii)
{
    bool supported;

    switch (mii)
    {
    /* This driver doesn't support MII and RGMII interfaces,
     * but the DP83826 PHY does support them */
    /* Adding the MII mode for temporary purposes
     * Has to be validated again. */
        case ENETPHY_MAC_MII_RMII:
        case ENETPHY_MAC_MII_MII:
            supported = true;
            break;

        /* This driver doesn't support MII and RGMII interfaces,
         * but the DP83826 PHY does support them */
        case ENETPHY_MAC_MII_RGMII:
        default:
            supported = false;
            break;
    }

    return supported;
}

static int32_t Dp83826_config(EnetPhy_Handle hPhy,
                              const EnetPhy_Cfg *cfg,
                              EnetPhy_Mii mii)
{
    const Dp83826_Cfg *extendedCfg = (const Dp83826_Cfg *)cfg->extendedCfg;
    uint32_t extendedCfgSize = cfg->extendedCfgSize;
    bool enableAutoMdix;
    int32_t status = ENETPHY_SOK;

    if ((extendedCfg == NULL) ||
        (extendedCfgSize != sizeof(*extendedCfg)))
    {
        ENETTRACE_ERR("PHY %u: invalid config params (cfg=%p, size=%u)\n",
                      hPhy->addr, extendedCfg, extendedCfgSize);
        status = ENETPHY_EINVALIDPARAMS;
    }

    /* Software restart is required after Viterbi idle detector and loopback
     * configuration change */
    if (status == ENETPHY_SOK)
    {
        Dp83826_reset(hPhy);
    }

    /* Auto-MDIX should be disabled in near-end loopback modes */
    enableAutoMdix = !cfg->loopbackEn;

    /* Enable Auto-MDIX and Robust Auto-MDIX */
    if (status == ENETPHY_SOK)
    {
        Dp83826_enableAutoMdix(hPhy, enableAutoMdix);
    }

    /* Set output impedance */
    if (status == ENETPHY_SOK)
    {
        status = Dp83826_setOutputImpedance(hPhy, extendedCfg->impedanceInMilliOhms);
    }

    return status;
}

static void Dp83826_enableAutoMdix(EnetPhy_Handle hPhy,
                                   bool enable)
{
    ENETTRACE_DBG("PHY %u: %s automatic cross-over %s\n",
                  hPhy->addr, enable ? "enable" : "disable");
    EnetPhy_rmwReg(hPhy, DP83826_PHYCR,
                   PHYCR_AUTOMDIX_ENABLE,
                   enable ? PHYCR_AUTOMDIX_ENABLE : 0);

    if (enable)
    {
        ENETTRACE_DBG("PHY %u: enable Robust Auto-MDIX\n", hPhy->addr);
        EnetPhy_rmwReg(hPhy, DP83826_CR1,
                       CR1_ROBUSTAUTOMDIX,
                       CR1_ROBUSTAUTOMDIX);
    }
    else
    {
        EnetPhy_rmwReg(hPhy, DP83826_PHYCR,
                       PHYCR_FORCEMDIX_MASK,
                       PHYCR_FORCEMDIX_MDI);
    }
}

void Dp83826_resetPHYs()
{
    EnetAppUtils_print("EXT PHY Reset\r\n");

    GPIO_setDirMode(CONFIG_GPIO_31_BASE_ADDR, CONFIG_GPIO_31_PIN, GPIO_DIRECTION_OUTPUT);
    GPIO_pinWriteHigh(CONFIG_GPIO_31_BASE_ADDR, CONFIG_GPIO_31_PIN);
    GPIO_setDirMode(CONFIG_GPIO_32_BASE_ADDR, CONFIG_GPIO_32_PIN, GPIO_DIRECTION_OUTPUT);
    GPIO_pinWriteHigh(CONFIG_GPIO_32_BASE_ADDR, CONFIG_GPIO_32_PIN);
    ClockP_usleep(1000);
    GPIO_pinWriteLow(CONFIG_GPIO_31_BASE_ADDR, CONFIG_GPIO_31_PIN);
    GPIO_pinWriteLow(CONFIG_GPIO_32_BASE_ADDR, CONFIG_GPIO_32_PIN);
    ClockP_usleep(1000);
    GPIO_pinWriteHigh(CONFIG_GPIO_31_BASE_ADDR, CONFIG_GPIO_31_PIN);
    GPIO_pinWriteHigh(CONFIG_GPIO_32_BASE_ADDR, CONFIG_GPIO_32_PIN);
    ClockP_usleep(1000);
}

static void Dp83826_reset(EnetPhy_Handle hPhy)
{
    /* Global software reset: all PHY internal circuits including IEEE-defined
     * registers and all extended registers are reset */
    ENETTRACE_DBG("PHY %u: global soft-reset\n", hPhy->addr);
    EnetPhy_rmwReg(hPhy, DP83826_PHYRCR, PHYRCR_SWRESET, PHYRCR_SWRESET);
}

static int32_t Dp83826_setOutputImpedance(EnetPhy_Handle hPhy,
                                          uint32_t impedance)
{
    int32_t status = ENETPHY_SOK;
    uint32_t val;

    if ((impedance == DP83826_IMPEDENCE_SLOW) ||
        (impedance == DP83826_IMPEDENCE_HIGH))
    {
        ENETTRACE_DBG("PHY %u: set output impedance to %u milli-ohms\n", hPhy->addr, impedance);

        val = DP83826_IMPEDENCE_VAL(impedance);

        EnetPhy_rmwReg(hPhy, DP83826_IO_CFG1,
                       IOCFG_IOIMPEDANCE_MASK,
                       val);
    }
    else
    {
        ENETTRACE_ERR("PHY %u: out-of-range impedance: %u\n", hPhy->addr, impedance);
        status = ENETPHY_EINVALIDPARAMS;
    }

    return status;
}

static void Dp83826_printRegs(EnetPhy_Handle hPhy)
{
    uint32_t phyAddr = hPhy->addr;
    uint16_t val;

    EnetPhy_readReg(hPhy, PHY_BMCR, &val);
    EnetUtils_printf("PHY %u: BMCR    = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_BMSR, &val);
    EnetUtils_printf("PHY %u: BMSR    = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_PHYIDR1, &val);
    EnetUtils_printf("PHY %u: PHYIDR1 = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_PHYIDR2, &val);
    EnetUtils_printf("PHY %u: PHYIDR2 = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_ANAR, &val);
    EnetUtils_printf("PHY %u: ANAR    = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_ANLPAR, &val);
    EnetUtils_printf("PHY %u: ANLPAR  = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_ANER, &val);
    EnetUtils_printf("PHY %u: ANER    = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_ANNPTR, &val);
    EnetUtils_printf("PHY %u: ANNPTR  = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_ANNPRR, &val);
    EnetUtils_printf("PHY %u: ANNPRR  = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, DP83826_CR1, &val);
    EnetUtils_printf("PHY %u: CR1     = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_GIGSR, &val);
    EnetUtils_printf("PHY %u: STS1    = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, PHY_GIGESR, &val);
    EnetUtils_printf("PHY %u: 1KSCR   = 0x%04x\n", phyAddr, val);
    EnetPhy_readReg(hPhy, DP83826_PHYCR, &val);
    EnetUtils_printf("PHY %u: PHYCR   = 0x%04x\n", phyAddr, val);
}

void EnetPhy_configPHYs (void)
{
    unsigned int PHY1 = 1;
    unsigned int PHY2 = 3;

    char option;


    EnetAppUtils_print("Press y to config PHYs\r\n");
    while(true)
    {
        DebugP_scanf("%c", &option);
        if (option == 'y')
        {
            //PHY1
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x001F, 0x8000);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x0573, 0x0801);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x0834, 0xC001);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x0405, 0x5800);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x08AD, 0x3C51);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x0894, 0x5DF7);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x08A0, 0x09E7);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x08C0, 0x4000);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x0814, 0x4800);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x080D, 0x2EBF);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x08C1, 0x0B00);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x087D, 0x0001);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x082E, 0x0000);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x0837, 0x00F4);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x08BE, 0x0200);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x08C5, 0x4000);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x08C7, 0x2000);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x08B3, 0x005A);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x08B4, 0x005A);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x08B0, 0x0202);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x08B5, 0x00EA);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x08BA, 0x2828);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x08BB, 0x6828);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x08BC, 0x0028);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x08BF, 0x0000);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x08B1, 0x0014);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x08B2, 0x0008);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x08EC, 0x0000);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x08C8, 0x0003);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x08BE, 0x0201);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x018C, 0x0001);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x001F, 0x4000);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x0573, 0x0001);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x056A, 0x5F41);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x0453, 0x0006);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x0452, 0x0600);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY1, 0x430, 0x0980);
            EnetAppUtils_print("PHY1 config done\r\n");

            //PHY2
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x001f, 0x8000);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x0573, 0x0801);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x0834, 0x8001);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x0894, 0x5DF7);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x056A, 0x5F40);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x0405, 0x5800);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x08AD, 0x3C51);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x08A0, 0x09E7);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x08C0, 0x4000);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x0814, 0x4800);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x080D, 0x2EBF);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x08C1, 0x0B00);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x087D, 0x0001);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x082E, 0x0000);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x0837, 0x00F4);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x08BE, 0x0200);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x08C5, 0x4000);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x08C7, 0x2000);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x08B3, 0x005A);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x08B4, 0x005A);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x08B0, 0x0202);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x08B5, 0x00EA);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x08BA, 0x2828);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x08BB, 0x6828);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x08BC, 0x0028);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x08BF, 0x0000);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x08B1, 0x0014);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x08B2, 0x0008);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x08EC, 0x0000);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x08C8, 0x0003);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x056A, 0x5F40);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x018C, 0x0001);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x001F, 0x4000);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x0573, 0x0001);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x056A, 0x5F41);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x0453, 0x0006);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x0452, 0x0600);
            PHY_Reg_Write(ICSS_MDIO_USED, PHY2, 0x430, 0x0980);
            EnetAppUtils_print("PHY2 config done\r\n");
            break;
        }
    }

    ENETTRACE_ERR_IF(true, "\n\rSetup PHYs for test purpose\n\r \n\r\r\n");

}

static void PHY_Reg_Write(unsigned int baseAddr,unsigned int phyAddr,unsigned int DevAddr,unsigned short data)
{

    int MMD_number = 0x1F;

    if(DevAddr<=0x1F)
    {
        CSL_MDIO_phyRegWrite(baseAddr, phyAddr, DevAddr, data);
    }
    else
    {
        /* Set the function as address with device address */
        CSL_MDIO_phyRegWrite(baseAddr, phyAddr, 0x000D,(0x0000|MMD_number));    //0x000D is DP83826e register
        CSL_MDIO_phyRegWrite(baseAddr, phyAddr, 0x000E, DevAddr);               //0x000E is DP83826e register
        /* Set the function as data with device address */
        CSL_MDIO_phyRegWrite(baseAddr, phyAddr, 0x000D,(0x4000|MMD_number));    //0x000D is DP83826e register 0x001F is REGCR_Devad register
        CSL_MDIO_phyRegWrite(baseAddr, phyAddr, 0x000E, data);                  //0x000E is DP83826e register
    }
}
