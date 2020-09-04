/**
 * @file  csl_serdes3.c
 *
 * @brief
 *  This is the C implementation for Serdes CSL-FL.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2016, Texas Instruments, Inc.
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
#include <ti/csl/csl_serdes.h>
#include <drivers/hw_include/cslr_soc.h>
#include <ti/csl/soc/j721e/src/cslr_main_ctrl_mmr.h>
#include "stdlib.h"
#include <serdes_cd/V0/cslr_wiz16b4m4cs.h>
#include <serdes_cd/V0/cslr_wiz16b8m4ct.h>

void CSL_serdesIPSelect(
        uint32_t                mainCtrlMMRbaseAddr,
        CSL_SerdesPhyType       phyType,
        uint32_t                phyInstanceNum,
        CSL_SerdesInstance      serdesInstance,
        uint32_t                serdeslaneNum
        )
{
    /* Serdes IP Select in Main Ctrl MMR */

    if(phyType == CSL_SERDES_PHY_TYPE_SGMII ||
       phyType == CSL_SERDES_PHY_TYPE_XAUI  ||
       phyType == CSL_SERDES_PHY_TYPE_QSGMII)
    {
        if(serdesInstance != CSL_TORRENT_SERDES0)
        {
            if(phyInstanceNum == 0)
            {
                if(serdeslaneNum == 0)
                {
                    /* Enet Switch Q/SGMII Lane 1 */
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES0_LN0_CTRL),1,0,0x0);
                }
                else if(serdeslaneNum == 1)
                {
                    /* Enet Switch Q/SGMII Lane 2 */
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES0_LN1_CTRL),1,0,0x0);
                }
                else if(serdeslaneNum == 2)
                {
                    /* Enet Switch Q/SGMII Lane 3 */
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES1_LN0_CTRL),1,0,0x0);
                }
                else if(serdeslaneNum == 3)
                {
                    /* Enet Switch Q/SGMII Lane 4 */
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES1_LN1_CTRL),1,0,0x0);
                }
            }
        }
        else //Torrent
        {
            if(phyInstanceNum == 0)
            {
                if(serdeslaneNum == 0)
                {
                    /* Enet Switch Q/SGMII Lane 5 */
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES4_LN0_CTRL),1,0,0x2);
                }
                else if(serdeslaneNum == 1)
                {
                    /* Enet Switch Q/SGMII Lane 6 */
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES4_LN1_CTRL),1,0,0x2);
                }
                else if(serdeslaneNum == 2)
                {
                    /* Enet Switch Q/SGMII Lane 7 */
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES4_LN2_CTRL),1,0,0x2);
                }
                else if(serdeslaneNum == 3)
                {
                    /* Enet Switch Q/SGMII Lane 8 */
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES4_LN3_CTRL),1,0,0x2);
                }
            }
        }
    }

    if(phyType == CSL_SERDES_PHY_TYPE_PCIe)
    {
        if(phyInstanceNum == 0)
        {
            if(serdeslaneNum == 0)
            {
                /* PCIe0 Lane0 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES0_LN0_CTRL),1,0,0x1);
            }
            else if(serdeslaneNum == 1)
            {
                /* PCIe0 Lane1 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES0_LN1_CTRL),1,0,0x1);
            }
        }
        if(phyInstanceNum == 1)
        {
            if(serdeslaneNum == 0)
            {
                /* PCIe1 Lane 0 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES1_LN0_CTRL),1,0,0x1);
            }
            else if(serdeslaneNum == 1)
            {
                /* PCIe1 Lane1 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES1_LN1_CTRL),1,0,0x1);
            }
        }
        if(phyInstanceNum == 2)
        {
            if(serdeslaneNum == 0)
            {
                /* PCIe2 Lane 0 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES2_LN0_CTRL),1,0,0x1);
            }
            else if(serdeslaneNum == 1)
            {
                /* PCIe2 Lane1 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES2_LN1_CTRL),1,0,0x1);
            }
        }
        if(phyInstanceNum == 3)
        {
            if(serdeslaneNum == 0)
            {
                /* PCIe3 Lane 0 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES3_LN0_CTRL),1,0,0x1);
            }
            else if(serdeslaneNum == 1)
            {
                /* PCIe3 Lane1 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES3_LN1_CTRL),1,0,0x1);
            }
        }
    }

    if(phyType == CSL_SERDES_PHY_TYPE_USB)
    {
            if(phyInstanceNum == 0)
            {
                /* USB3_0 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES0_LN1_CTRL),1,0,0x2);
            }
            if(phyInstanceNum == 1)
            {
                /* USB3_1 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES1_LN1_CTRL),1,0,0x2);
            }
    }

    if(phyType == CSL_SERDES_PHY_TYPE_SGMII_ICSSG)
    {
        if(phyInstanceNum == 0)
        {
            if(serdeslaneNum == 0)
            {
                /* ICSS_G0 SGMII Lane 0 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES1_LN0_CTRL),1,0,0x3);
            }
            if(serdeslaneNum == 1)
            {
                /* ICSS_G0 SGMII Lane 1 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES1_LN1_CTRL),1,0,0x3);
            }
        }
    }

    if(phyType == CSL_SERDES_PHY_TYPE_eDP)
    {
        if(phyInstanceNum == 0)
        {
            if(serdeslaneNum == 0)
            {
                /* eDP Lane 0 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES4_LN0_CTRL),1,0,0x0);
            }
            if(serdeslaneNum == 1)
            {
                /* eDP Lane 1 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES4_LN1_CTRL),1,0,0x0);
            }
            if(serdeslaneNum == 2)
            {
                /* eDP Lane 2 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES4_LN2_CTRL),1,0,0x0);
            }
            if(serdeslaneNum == 3)
            {
                /* eDP Lane 3 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES4_LN3_CTRL),1,0,0x0);
            }
        }
    }
}

CSL_SerdesResult CSL_serdesRefclkSel(
        uint32_t                mainCtrlMMRbaseAddr,
        uint32_t                baseAddr,
        CSL_SerdesRefClock      refClk,
        CSL_SerdesRefClockSrc   refClkSrc,
        CSL_SerdesInstance      serdesInstance,
        CSL_SerdesPhyType       phyType
        )
{
    CSL_SerdesResult result = CSL_SERDES_NO_ERR;

    if(serdesInstance != CSL_TORRENT_SERDES0)
    {
        CSL_wiz16b4m4csRegs *sierra_sds_reg = (CSL_wiz16b4m4csRegs *)(uintptr_t)(baseAddr);

        /* PMA_CMN_REFCLK_DIG_DIV */
        if(refClk == CSL_SERDES_REF_CLOCK_100M) /* 100MHz clock */
        {
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->WIZ_CONFIG.SERDES_TOP_CTRL),27,26,0x2);
        }
        else if(refClk == CSL_SERDES_REF_CLOCK_156p25M) /* 156.25MHz clock */
        {
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->WIZ_CONFIG.SERDES_TOP_CTRL),27,26,0x3);
        }
        else if(refClk <= CSL_SERDES_REF_CLOCK_27M) /* 19.2MHz-27MHz reference clock */
        {
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->WIZ_CONFIG.SERDES_TOP_CTRL),27,26,0x0);
        }

        if(refClkSrc == CSL_SERDES_REF_CLOCK_INT)
        {
            /* PMA_CMN_REFCLK_INT_MODE */
            if(refClk >= CSL_SERDES_REF_CLOCK_100M) /* 100MHz and greater reference clock */
            {
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->WIZ_CONFIG.SERDES_TOP_CTRL),29,28,0x1);
            }
            else /* Less than 100MHz reference clock */
            {
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->WIZ_CONFIG.SERDES_TOP_CTRL),29,28,0x3);
            }

            /* REFCLK_DIG_SEL = pma_cmn_refclk_int */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->WIZ_CONFIG.SERDES_RST),25,24,0x1);

            /* PLL0_REFCLK_SEL: Selects pma_cmn_refclk_int as reference clock source */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->WIZ_CONFIG.SERDES_RST),28,28,0x1);

            /* PLL1_REFCLK_SEL: Selects pma_cmn_refclk1_int as reference clock source. */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->WIZ_CONFIG.SERDES_RST),29,29,0x1);

            /* Wait 1us before writing to CMN registers */
            CSL_serdesCycleDelay(1000);

            /* Setup MAIN_CTRL_MMR refclk mux for internal refclk and skip setup if mainCtrlMMRbaseAddr is NULL */
            if(mainCtrlMMRbaseAddr != (uintptr_t)NULL)
            {
                if(refClk == CSL_SERDES_REF_CLOCK_156p25M)
                {
                    if(serdesInstance == CSL_SIERRA_SERDES0)
                        /*Select core_refclk for serdes0 sierra*/ //select MAIN_PLL3_HSDIV4_CLKOUT for 156.25 MHz
                        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CLKSEL),1,0,0x2);
                    if(serdesInstance == CSL_SIERRA_SERDES1)
                        /*Select core_refclk for serdes1 sierra*/ //select MAIN_PLL3_HSDIV4_CLKOUT for 156.25 MHz
                        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES1_CLKSEL),1,0,0x2);
                    if(serdesInstance == CSL_SIERRA_SERDES2)
                        /*Select core_refclk for serdes2 sierra*/ //select MAIN_PLL3_HSDIV4_CLKOUT for 156.25 MHz
                        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES2_CLKSEL),1,0,0x2);
                    if(serdesInstance == CSL_SIERRA_SERDES3)
                        /*Select core_refclk for serdes3 sierra*/ //select MAIN_PLL3_HSDIV4_CLKOUT for 156.25 MHz
                        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES3_CLKSEL),1,0,0x2);
                }
                else if (refClk == CSL_SERDES_REF_CLOCK_100M)
                {
                    if(serdesInstance == CSL_SIERRA_SERDES0)
                    /*Select core_refclk for serdes0 sierra*/ //select MAIN_PLL2_HSDIV4_CLKOUT for 100 MHz
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CLKSEL),1,0,0x3);
                    if(serdesInstance == CSL_SIERRA_SERDES1)
                    /*Select core_refclk for serdes1 sierra*/ //select MAIN_PLL2_HSDIV4_CLKOUT for 100 MHz
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES1_CLKSEL),1,0,0x3);
                    if(serdesInstance == CSL_SIERRA_SERDES2)
                    /*Select core_refclk for serdes2 sierra*/ //select MAIN_PLL2_HSDIV4_CLKOUT for 100 MHz
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES2_CLKSEL),1,0,0x3);
                    if(serdesInstance == CSL_SIERRA_SERDES3)
                    /*Select core_refclk for serdes3 sierra*/ //select MAIN_PLL2_HSDIV4_CLKOUT for 100 MHz
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES3_CLKSEL),1,0,0x3);
                }
                else if (refClk == CSL_SERDES_REF_CLOCK_19p2M)
                {
                    if(serdesInstance == CSL_SIERRA_SERDES0)
                    /*Select core_refclk for serdes0 sierra*/ //select HFOSC0_CLKOUT for 19.2 MHz
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CLKSEL),1,0,0x0);
                    if(serdesInstance == CSL_SIERRA_SERDES1)
                    /*Select core_refclk for serdes1 sierra*/ //select HFOSC0_CLKOUT for 19.2 MHz
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES1_CLKSEL),1,0,0x0);
                    if(serdesInstance == CSL_SIERRA_SERDES2)
                    /*Select core_refclk for serdes2 sierra*/ //select HFOSC0_CLKOUT for 19.2 MHz
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES2_CLKSEL),1,0,0x0);
                    if(serdesInstance == CSL_SIERRA_SERDES3)
                    /*Select core_refclk for serdes3 sierra*/ //select HFOSC0_CLKOUT for 19.2 MHz
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_SERDES3_CLKSEL),1,0,0x0);
                }
                else
                {
                    result = CSL_SERDES_INVALID_REF_CLOCK;
                }
            }
        }
        else if(refClkSrc == CSL_SERDES_REF_CLOCK_EXT_SSC || CSL_SERDES_REF_CLOCK_EXT_NO_SSC)
        {
            /* PMA_CMN_REFCLK_MODE */
            if(refClk >= CSL_SERDES_REF_CLOCK_100M) /* 100MHz and greater reference clock */
            {
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->WIZ_CONFIG.SERDES_TOP_CTRL),31,30,0x0);
            }
            else /* Less than 100MHz reference clock */
            {
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->WIZ_CONFIG.SERDES_TOP_CTRL),31,30,0x2);
            }

            /* REFCLK_DIG_SEL = cmn_refclk1_<p/m> */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->WIZ_CONFIG.SERDES_RST),25,24,0x2);

            /* Wait 1us before writing to CMN registers */
            CSL_serdesCycleDelay(1000);

            /* Enable REFRCV1_REFCLK_PLLC1EN to drive refclk to PLLCMNLC */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->CMN_CTRL_CDBREG.CMN_REFRCV1_PREG), 8, 8, (uint32_t)0x1);

            /* Enable on die termination for the refclk1 receiver */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->CMN_CTRL_CDBREG.CMN_REFRCV1_PREG), 0, 0, (uint32_t)0x1);

            /* Set CMN_PLLLC_PFDCLK1_SEL_PREG to 1 to select PLL0 to source from cmn_refclk1_<p/m> pads */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->CMN_CTRL_CDBREG.CMN_PLLLC_FBDIV_INT_PREG__CMN_PLLLC_GEN_PREG), 1, 1, (uint32_t)0x1);

            /* PLL0_REFCLK_SEL: Selects cmn_refclk_<m/p> as reference clock source */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->WIZ_CONFIG.SERDES_RST),28,28,0x0);

            /* PLL1_REFCLK_SEL: Selects cmn_refclk1_<m/p> as reference clock source */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->WIZ_CONFIG.SERDES_RST),29,29,0x0);
        }
    }
    else //Torrent
    {
        CSL_wiz16b8m4ctRegs *torrent_sds_reg = (CSL_wiz16b8m4ctRegs *)(uintptr_t)(baseAddr);

        /* PMA_CMN_REFCLK_DIG_DIV */
        if(refClk == CSL_SERDES_REF_CLOCK_100M) /* 100MHz reference clock */
        {
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_TOP_CTRL),27,26,0x2);
        }
        else if(refClk == CSL_SERDES_REF_CLOCK_156p25M) /* 156.25MHz reference clock */
        {
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_TOP_CTRL),27,26,0x3);
        }
        else if(refClk <= CSL_SERDES_REF_CLOCK_27M) /* 19.2MHz-27MHz reference clock */
        {
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_TOP_CTRL),27,26,0x0);
        }

        if(refClkSrc == CSL_SERDES_REF_CLOCK_INT)
        {
            /* PMA_CMN_REFCLK_INT_MODE */
            if(refClk >= CSL_SERDES_REF_CLOCK_100M) /* 100MHz and greater reference clock */
            {
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_TOP_CTRL),29,28,0x1);
            }
            else /* Less than 100MHz reference clock */
            {
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_TOP_CTRL),29,28,0x3);
            }

            /* REFCLK_DIG_SEL = pma_cmn_refclk_int */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_RST),24,24,0x1);
            /* PLL0_REFCLK_SEL: Selects pma_cmn_refclk_int as reference clock source */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_RST),28,28,0x1);
            /* PLL1_REFCLK_SEL: Selects pma_cmn_refclk1_int as reference clock source. */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_RST),29,29,0x1);

            /* Wait 1us before writing to CMN registers */
            CSL_serdesCycleDelay(1000);

            /* Setup MAIN_CTRL_MMR refclk mux for internal refclk and skip setup if mainCtrlMMRbaseAddr is NULL */
            if(mainCtrlMMRbaseAddr != (uintptr_t)NULL)
            {
                if (refClk == CSL_SERDES_REF_CLOCK_100M)
                {
                    if(serdesInstance == CSL_TORRENT_SERDES0)
                    {
                        /*Select core_refclk for serdes4 torrent */ //select MAIN_PLL2_HSDIV4_CLKOUT for 100 MHz
                        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_EDP_PHY0_CLKSEL),1,0,0x3);
                    }
                }
                else if (refClk == CSL_SERDES_REF_CLOCK_156p25M)
                {
                    if(serdesInstance == CSL_TORRENT_SERDES0)
                    {
                        /*Select core_refclk for serdes4 torrent */ //select MAIN_PLL3_HSDIV4_CLKOUT for 156.25 MHz
                        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_EDP_PHY0_CLKSEL),1,0,0x2);
                    }
                }
                else if (refClk == CSL_SERDES_REF_CLOCK_19p2M)
                {
                    if(serdesInstance == CSL_TORRENT_SERDES0)
                    {
                        /*Select core_refclk for serdes0 sierra*/ //select HFOSC0_CLKOUT for 19.2 MHz
                        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_EDP_PHY0_CLKSEL),1,0,0x0);
                    }
                }
                else
                {
                    result = CSL_SERDES_INVALID_REF_CLOCK;
                }
            }


        }
        else if(refClkSrc == CSL_SERDES_REF_CLOCK_EXT_SSC || CSL_SERDES_REF_CLOCK_EXT_NO_SSC)
        {
            /* PMA_CMN_REFCLK_MODE */
            if(refClk >= CSL_SERDES_REF_CLOCK_100M) /* 100MHz and greater reference clock */
            {
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_TOP_CTRL),31,30,0x0);
            }
            else /* Less than 100MHz reference clock */
            {
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_TOP_CTRL),31,30,0x2);
            }

            /* REFCLK_DIG_SEL = cmn_refclk_<p/m> */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_RST),24,24,0x0);
            /* PLL0_REFCLK_SEL: Selects cmn_refclk_<m/p> as reference clock source */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_RST),28,28,0x0);
            /* PLL1_REFCLK_SEL: Selects cmn_refclk_<m/p> as reference clock source */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_RST),29,29,0x0);

            /* Wait 1us before writing to CMN registers */
            CSL_serdesCycleDelay(1000);
        }
    }

    return result;
}
void CSL_serdesCycleDelay (uint64_t count)
{
  volatile uint64_t sat = 0;
  for(sat=0;sat<count;sat++);
}

void CSL_serdesPCIeModeSelect(uint32_t baseAddr, CSL_SerdesPCIeGenType  pcieGenType, uint32_t laneNum)
{
    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + 0x40*laneNum),25, 24, pcieGenType);
}

void CSL_serdesOutClkEn(uint32_t baseAddr, CSL_SerdesRefClock refClock, CSL_SerdesPhyType   phyType)
{

    CSL_wiz16b4m4csRegs *sierra_sds_reg = (CSL_wiz16b4m4csRegs *)(uintptr_t)(baseAddr);

    /* Output Clock enable for ref_clk_out and ref_der_clk_out */
    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->PHY_PMA_CMN_REGISTERS.PHY_PMA_CMN_CTRL), 7, 6, (uint32_t)0x2);
    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->CMN_CTRL_CDBREG.CMN_PLLLC_CLK1_PREG__CMN_PLLLC_LOCK_CNTTHRESH_PREG), 28, 28, (uint32_t)0x1);

    /* Value of 0x0 is divide by 4 */
    if(phyType == CSL_SERDES_PHY_TYPE_PCIe || phyType == CSL_SERDES_PHY_TYPE_USB || phyType == CSL_SERDES_PHY_TYPE_QSGMII)
    {
        /* Programming to get 100Mhz clock output in ref_der_clk_out 5GHz VCO/50 = 100MHz */
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->CMN_CTRL_CDBREG.CMN_PLLLC_CLK1_PREG__CMN_PLLLC_LOCK_CNTTHRESH_PREG), 22, 16, (uint32_t)0x2E);
    }
    else if(phyType == CSL_SERDES_PHY_TYPE_SGMII || phyType == CSL_SERDES_PHY_TYPE_XAUI)
    {
        /* Programming to get 156.25Mhz clock output in ref_der_clk_out 4.375GHz VCO/28 = 156.25Mhz */
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->CMN_CTRL_CDBREG.CMN_PLLLC_CLK1_PREG__CMN_PLLLC_LOCK_CNTTHRESH_PREG), 22, 16, (uint32_t)0x18);
    }
}


void CSL_serdesDisablePllAndLanes(uint32_t  baseAddr,
                                  uint32_t  numLanes,
                                  uint8_t   laneMask)
{
     uint32_t laneNum;
     CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x040C),31,31,0x0);

     for(laneNum=0;laneNum<numLanes;laneNum++)
     {
         if ((laneMask & (1<<laneNum))!=0)
         {
             /* disable p0, p1, p2, p3 */
             CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x0480+ (0x40*laneNum)),31, 30, 0x0); //p0
         }
     }
}

void CSL_serdesInvertLaneTXPolarity(uint32_t baseAddr,
                                  uint32_t laneNum)
{
    /* Invert TX Lane Polarity */
    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0xf000 + (0x200*laneNum)),8,8,1);
}

void CSL_serdesInvertLaneRXPolarity(uint32_t baseAddr,
                                  uint32_t laneNum)
{
    /* Invert RX Lane Polarity */
    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0xf000 + (0x200*laneNum)),0,0,1);
}

void CSL_serdesDisablePLL(uint32_t            baseAddr,
                          CSL_SerdesPhyType   phyType)
{
    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x040C),31,31,0x0);
}

void CSL_serdesDisableLanes(
        uint32_t    baseAddr,
        uint32_t    laneNum,
        uint8_t     laneMask)
{
    if ((laneMask & (1<<laneNum))!=0) //control which lanes are affected with bit masking
    //disable p0, p1, p2, p3
    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x0480+ (0x40*laneNum)),31, 30, 0x0); //p0
}

void CSL_serdesEnableLanes(uint32_t baseAddr,
                           uint32_t laneNum,
                           CSL_SerdesPhyType   phyType,
                           CSL_SerdesInstance  instance)
{

    CSL_wiz16b4m4csRegs *sierra_sds_reg = (CSL_wiz16b4m4csRegs *)(uintptr_t)(baseAddr);

    /* Enable P0/P1 depending on laneNum */

    /* PIPE should only enable P0/P1 enable */
    if(phyType == CSL_SERDES_PHY_TYPE_PCIe || phyType == CSL_SERDES_PHY_TYPE_USB)
    {
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + (0x40*laneNum)),31, 31, 0x1);
    }
    else
    {
        /* RAW mode should enable the force enable. SGMII needs comma align and raw auto start */
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + (0x40*laneNum)),31, 28, 0x7);
    }

    if(instance != CSL_TORRENT_SERDES0)
    {
        if(phyType == CSL_SERDES_PHY_TYPE_SGMII ||
           phyType == CSL_SERDES_PHY_TYPE_SGMII_ICSSG)
        {
            /* Set FULLRT div to /4 to get 125MHz */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + (0x40*laneNum)),23, 22, 0x2);
            *(volatile uint32_t *)(uintptr_t)(baseAddr + 0x484 + (0x40*laneNum)) = 0x10002;
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->PHY_PMA_LANE_REGISTERS[laneNum].PHY_PMA_XCVR_CTRL),15, 0, 0x9010);
        }
        if(phyType == CSL_SERDES_PHY_TYPE_XAUI)
        {
            /* Set FULLRT div to /2 to get 312.5MHz */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + (0x40*laneNum)),23, 22, 0x1);
            *(volatile uint32_t *)(uintptr_t)(baseAddr + 0x484 + (0x40*laneNum)) = 0x10002;
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->PHY_PMA_LANE_REGISTERS[laneNum].PHY_PMA_XCVR_CTRL),15, 0, 0x9010);
        }

        if(phyType == CSL_SERDES_PHY_TYPE_QSGMII)
        {
            *(volatile uint32_t *)(uintptr_t)(baseAddr + 0x484 + (0x40*laneNum)) = 0x10002;
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->PHY_PMA_LANE_REGISTERS[laneNum].PHY_PMA_XCVR_CTRL),15, 0, 0x9010);
        }
    }
    else
    {
        if(phyType == CSL_SERDES_PHY_TYPE_SGMII       ||
           phyType == CSL_SERDES_PHY_TYPE_SGMII_ICSSG)
        {
            /* Set FULLRT div to /4 to get 125MHz */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + (0x40*laneNum)),23, 22, 0x2);

            /* Enable RX 2x clock enable */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x4000 + (laneNum*0x400) + 0x1d0),31,16, (uint32_t)0x4000);

            /* Set MAC clock dividers */
            *(volatile uint32_t *)(uintptr_t)(baseAddr + 0x484 + (0x40*laneNum)) = 0x10002;
        }
        if(phyType == CSL_SERDES_PHY_TYPE_XAUI)
        {
            /* Set FULLRT div to /1 to get 312.5MHz */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + (0x40*laneNum)),23, 22, 0x0);

            /* Enable RX 2x clock enable */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x4000 + (laneNum*0x400) + 0x1d0),31,16, (uint32_t)0x4000);

            /* Set MAC clock dividers */
            *(volatile uint32_t *)(uintptr_t)(baseAddr + 0x484 + (0x40*laneNum)) = 0x10002;
        }
        if(phyType == CSL_SERDES_PHY_TYPE_QSGMII)
        {
            /* Set standard mode to 1 */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x480 + (0x40*laneNum)),25, 24, 0x1);

            /* Set MAC clock dividers */
            *(volatile uint32_t *)(uintptr_t)(baseAddr + 0x484 + (0x40*laneNum)) = 0x10002;

            /* Enable RX 2x clock enable */
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x4000 + (laneNum*0x400) + 0x1d0),31,16, (uint32_t)0x4000);
        }
    }
}


void CSL_serdesFastSimEnable(uint32_t baseAddr, uint32_t numLanes, CSL_SerdesPhyType phyType)
{
    uint32_t laneNum;
    CSL_wiz16b4m4csRegs *sierra_sds_reg = (CSL_wiz16b4m4csRegs *)(uintptr_t)(baseAddr);

    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->PHY_PCS_CMN_REGISTERS.PHY_PIPE_CMN_CTRL2__PHY_PIPE_CMN_CTRL1), 15, 0, (uint32_t)0x0430); //TODO
    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->PHY_PCS_CMN_REGISTERS.PHY_PIPE_COM_LOCK_CFG2__PHY_PIPE_COM_LOCK_CFG1), 15, 0, (uint32_t)0x4010);
    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->PHY_PCS_CMN_REGISTERS.PHY_PIPE_COM_LOCK_CFG2__PHY_PIPE_COM_LOCK_CFG1), 31, 16, (uint32_t)0x0810);
    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->PHY_PCS_CMN_REGISTERS.PHY_PIPE_LANE_DSBL__PHY_PIPE_EIE_LOCK_CFG), 15, 0, (uint32_t)0x0101);
    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->PHY_PCS_CMN_REGISTERS.PHY_PIPE_RX_ELEC_IDLE_DLY__PHY_PIPE_RCV_DET_INH), 15, 0, (uint32_t)0xA);

    for(laneNum=0; laneNum< numLanes; laneNum++)
    {
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CPICAL_CAP_OVR_PREG__CPICAL_CAP_STARTCODE_MODE01_PREG), 15, 0, (uint32_t)0x2E31); //TODO
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CREQ_EQ_OPEN_EYE_THRESH_PREG__CREQ_SPARE_PREG), 15, 0, (uint32_t)0x8000);
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_CONCUR_CTRL2_PREG__DEQ_CONCUR_CTRL1_PREG), 31, 16, (uint32_t)0xD664);
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].LANE_TX_RECEIVER_DETECT_PREG__DRVCTRL_BSCAN_PREG), 31, 16, (uint32_t)0x5);
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_CONCUR_CTRL2_PREG__DEQ_CONCUR_CTRL1_PREG), 15, 0, (uint32_t)0x0100);
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_FSM_OVR_PREG__DEQ_EPIPWR_CTRL_PREG), 15, 0, (uint32_t)0x8190);
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_EPIPWR_CTRL2_PREG__CONCUR_PREEVAL_MINITER_CTRL_PREG), 31, 16, (uint32_t)0x8A);
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CMP_AVR_TIMER_PREG__DEQ_ERRCMPB_OVR_PREG), 31, 16, (uint32_t)0x0A);
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_TAU_CTRL1_FAST_MAINT_PREG), 31, 16, (uint32_t)0x5008);
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_TAU_CTRL2_PREG__DEQ_TAU_CTRL1_SLOW_MAINT_PREG), 15, 0, (uint32_t)0x5008);
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].DEQ_OPENEYE_CTRL_PREG), 15, 0, (uint32_t)0x7425);
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].SMPCAL_NUM_WORDS_PREG__SMPCAL_ITER_PREG), 31, 16, (uint32_t)0x000F);
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].SMPCAL_TUNE_PREG__SMPCAL_START_PREG), 15, 0, (uint32_t)0x0003);
        CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CPICAL_RES_STARTCODE_MODE23_PREG__CPICAL_INCR_DECR_PREG), 31, 16, (uint32_t)0x7855);

        if(phyType == CSL_SERDES_PHY_TYPE_PCIe)
        {
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CPICAL_RES_INITTMR_PREG__CPICAL_RES_STARTCODE_MODE01_PREG), 15, 0, (uint32_t)0x462D);
        }
        else if(phyType == CSL_SERDES_PHY_TYPE_USB)
        {
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CPICAL_RES_INITTMR_PREG__CPICAL_RES_STARTCODE_MODE01_PREG), 15, 0, (uint32_t)0x5F46);
        }
        else if(phyType == CSL_SERDES_PHY_TYPE_QSGMII)
        {
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CPICAL_RES_INITTMR_PREG__CPICAL_RES_STARTCODE_MODE01_PREG), 15, 0, (uint32_t)0x3246);
        }
        else if(phyType == CSL_SERDES_PHY_TYPE_SGMII || phyType == CSL_SERDES_PHY_TYPE_SGMII_ICSSG)
        {
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].CPICAL_RES_INITTMR_PREG__CPICAL_RES_STARTCODE_MODE01_PREG), 15, 0, (uint32_t)0x1414);
        }

    }


}

void CSL_serdesRxEqDisable(uint32_t baseAddr)
{
    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr +0x1400),7,0, 0x3E);
}

void CSL_serdesPorReset(uint32_t baseAddr) //modified for sierra wiz
{
    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x404),31,31, 0x1);

    CSL_serdesCycleDelay(1000);

    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x404),31,31, 0x0);
}

void CSL_serdesSetLoopback
(
 uint32_t baseAddr,
 uint32_t laneNum,
 CSL_SerdesLoopback loopbackMode,
 CSL_SerdesInstance serdesInstance,
 CSL_SerdesPhyType phyType
)
{
    CSL_wiz16b4m4csRegs *sierra_sds_reg = (CSL_wiz16b4m4csRegs *)(uintptr_t)(baseAddr);
    CSL_wiz16b8m4ctRegs *torrent_sds_reg = (CSL_wiz16b8m4ctRegs *)(uintptr_t)(baseAddr);

    if (loopbackMode == CSL_SERDES_LOOPBACK_SER)
    {
        if(serdesInstance != CSL_TORRENT_SERDES0) /* Sierra serial loopback */
        {
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].BSCAN_LPBKLINE_PREG__LANE_LOOPBACK_CTRL_PREG), 2, 2, (uint32_t)0x1);
        }
        else /* Torrent serial loopback */
        {
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PHY_PMA_LANE_REGISTERS[laneNum].PHY_PMA_XCVR_LPBK__PHY_PMA_XCVR_CTRL),16, 16, (uint32_t)0x1);
        }
    }
    else if (loopbackMode == CSL_SERDES_LOOPBACK_LINE)
    {
        if(serdesInstance != CSL_TORRENT_SERDES0) /* Sierra LINE loopback */
        {
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&sierra_sds_reg->LN_CTRL_CDBREG[laneNum].BSCAN_LPBKLINE_PREG__LANE_LOOPBACK_CTRL_PREG), 3, 3, (uint32_t)0x1);
        }
        else /* Torrent LINE loopback */
        {
            CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PHY_PMA_LANE_REGISTERS[laneNum].PHY_PMA_XCVR_LPBK__PHY_PMA_XCVR_CTRL),18, 18, (uint32_t)0x1);
        }
    }
}

void CSL_serdesReleaseReset
(
 uint32_t               baseAddr
)
{
    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x40C),31,31, (uint32_t)0x1);
}

CSL_SerdesStatus CSL_serdesGetPLLStatus
(
 uint32_t baseAddr,
 uint32_t laneMask,
 CSL_SerdesInstance serdesInstance
)
{
    CSL_SerdesStatus retval = CSL_SERDES_STATUS_PLL_LOCKED;
    uint32_t laneNum;

    if(serdesInstance != CSL_TORRENT_SERDES0)
    {
        for(laneNum=0;laneNum<CSL_SERDES_MAX_LANES;laneNum++)
        {
            if ((laneMask & (1<<laneNum))!=0)
            {
                retval &= (CSL_SerdesStatus)CSL_FEXTR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x4000 + 0x400*laneNum +0x88), 0, 0);
            }
        }
    }
    else
    {
        retval &= (CSL_SerdesStatus)CSL_FEXTR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0xE000), 22, 22);
    }
    return retval;
}

CSL_SerdesStatus CSL_serdesGetLaneStatus
(
 uint32_t baseAddr,
 uint32_t numLanes,
 uint8_t laneMask,
 CSL_SerdesPhyType phyType
)
{
    uint32_t laneNum;
    CSL_SerdesStatus retval = CSL_SERDES_STATUS_PLL_LOCKED;
    for (laneNum=0; laneNum < numLanes; laneNum++)
    {
        if ((laneMask & (1<<laneNum))!=0)
        {
        /* Poll for Lane OK State */
        retval &= (CSL_SerdesStatus)CSL_FEXTR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x48C + laneNum * 0x40), 0, 0);
        }
    }
    return retval;
}

CSL_SerdesStatus CSL_serdesConfigStatus
(
 uint32_t baseAddr
)
{
    CSL_SerdesStatus retval;
    /* Return the PHY_RESET_N status /0 = not configured, /1 = serdes is configured and reset is deasserted */
    retval = (CSL_SerdesStatus)CSL_FEXTR(*(volatile uint32_t *)(uintptr_t)(baseAddr + 0x40c), 31, 31);
    return retval;
}


CSL_SerdesLaneEnableStatus CSL_serdesLaneEnable
(
         CSL_SerdesLaneEnableParams *serdesLaneEnableParams
)
{
    uint32_t laneNum;
    CSL_SerdesStatus   pllstat;
    CSL_SerdesLaneEnableStatus status = CSL_SERDES_LANE_ENABLE_NO_ERR;
    CSL_wiz16b8m4ctRegs *torrent_sds_reg = (CSL_wiz16b8m4ctRegs *)(uintptr_t)(serdesLaneEnableParams->baseAddr);

    /* Set LANE loopback bits based on the loopback type */
    if(serdesLaneEnableParams->operatingMode != CSL_SERDES_DIAGNOSTIC_MODE)
    {
        for(laneNum=0;laneNum<CSL_SERDES_MAX_LANES;laneNum++)
        {
            if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
            {
                CSL_serdesSetLoopback(serdesLaneEnableParams->baseAddr, laneNum, serdesLaneEnableParams->loopbackMode[laneNum], serdesLaneEnableParams->serdesInstance, serdesLaneEnableParams->phyType);
            }
        }
    }

    for(laneNum=0;laneNum<CSL_SERDES_MAX_LANES;laneNum++)
    {
        if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
        {
            if(serdesLaneEnableParams->invertTXPolarity[laneNum])
            {
                CSL_serdesInvertLaneTXPolarity(serdesLaneEnableParams->baseAddr, laneNum);
            }
            if(serdesLaneEnableParams->invertRXPolarity[laneNum])
            {
                CSL_serdesInvertLaneRXPolarity(serdesLaneEnableParams->baseAddr, laneNum);
            }
        }
    }

    /* Enable Refclk Out when the refclkout flag is set */
    if(serdesLaneEnableParams->refClkOut == CSL_SERDES_REFCLK_OUT_EN)
    {
        if(serdesLaneEnableParams->serdesInstance != CSL_TORRENT_SERDES0)
        {
            if(serdesLaneEnableParams->refClkSrc == CSL_SERDES_REF_CLOCK_INT)
            {
                CSL_serdesOutClkEn(serdesLaneEnableParams->baseAddr, serdesLaneEnableParams->refClock, serdesLaneEnableParams->phyType);
                if(serdesLaneEnableParams->baseAddr == CSL_SERDES_16G0_BASE)
                {
                    /* Select serdes0_ref_der_out_clk */
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(serdesLaneEnableParams->mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_PCIE_REFCLK0_CLKSEL),1,0,0x0);
                    /* Enable output of acspcie buffer: out_clk_en = 1'b1 */
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(serdesLaneEnableParams->mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_PCIE_REFCLK0_CLKSEL),8,8,0x1);
                    /* Enable PAD1/0 IO buffers of acspcie0: pwrdn1/0 = 1'b0 */
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(serdesLaneEnableParams->mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_ACSPCIE0_CTRL),1,0,0x0);
                }
                else if(serdesLaneEnableParams->baseAddr == CSL_SERDES_16G1_BASE)
                {
                    /* Select serdes1_ref_der_out_clk */
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(serdesLaneEnableParams->mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_PCIE_REFCLK1_CLKSEL),1,0,0x1);
                    /* Enable output of acspcie buffer: out_clk_en = 1'b1 */
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(serdesLaneEnableParams->mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_PCIE_REFCLK1_CLKSEL),8,8,0x1);
                    /* Enable PAD1/0 IO buffers of acspcie0: pwrdn1/0 = 1'b0 */
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(serdesLaneEnableParams->mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_ACSPCIE0_CTRL),1,0,0x0);
                }
                else if(serdesLaneEnableParams->baseAddr == CSL_SERDES_16G2_BASE)
                {
                    /* Select serdes2_ref_der_out_clk */
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(serdesLaneEnableParams->mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_PCIE_REFCLK2_CLKSEL),1,0,0x0);
                    /* Enable output of acspcie buffer: out_clk_en = 1'b1 */
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(serdesLaneEnableParams->mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_PCIE_REFCLK2_CLKSEL),8,8,0x1);
                    /* Enable PAD1/0 IO buffers of acspcie1: pwrdn1/0 = 1'b0 */
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(serdesLaneEnableParams->mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_ACSPCIE1_CTRL),1,0,0x0);
                }
                else if(serdesLaneEnableParams->baseAddr == CSL_SERDES_16G3_BASE)
                {
                    /* Select serdes3_ref_der_out_clk */
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(serdesLaneEnableParams->mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_PCIE_REFCLK3_CLKSEL),1,0,0x1);
                    /* Enable output of acspcie buffer: out_clk_en = 1'b1 */
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(serdesLaneEnableParams->mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_PCIE_REFCLK3_CLKSEL),8,8,0x1);
                    /* Enable PAD1/0 IO buffers of acspcie1: pwrdn1/0 = 1'b0 */
                    CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(serdesLaneEnableParams->mainCtrlMMRbaseAddr + CSL_MAIN_CTRL_MMR_CFG0_ACSPCIE1_CTRL),1,0,0x0);
                }
            }
        }
        else if(serdesLaneEnableParams->serdesInstance == CSL_TORRENT_SERDES0)
        {
            if(serdesLaneEnableParams->refClkSrc == CSL_SERDES_REF_CLOCK_INT)
            {
                /* 1. phy_en_refclk = 1'b0 (wiz) */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_RST),30,30,0x0);
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PHY_PCS_CMN_REGISTERS.PHY_ISO_CMN_CTRL),8,8,0x0);
                /* 2. Write 0x0210 to CMN_CDIAG_REFCLK_DRV0_CTRL to select derived reference cock */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_CDIAG_REFCLK_DRV0_CTRL),15,0,0x0210);
                /* 3. Write 0x0118 to CMN_CDIAG_REFCLK_OVRD to enable derived reference clock */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PMA_CMN_REGISTERS.CMN_CDIAG_REFCLK_TEST__CMN_CDIAG_REFCLK_OVRD),15,0,0x0118);
                /* 4. Write 0x0401 to PHY_PIPE_CMN_CTRL1 to enable phy_en_refclk pin input */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PHY_PCS_CMN_REGISTERS.PHY_PIPE_CMN_CTRL2__PHY_PIPE_CMN_CTRL1),15,0,0x0401);
                /* 5. Set phy_en_refclk = 1'b1 */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->WIZ_CONFIG.SERDES_RST),30,30,0x1);
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(&torrent_sds_reg->PHY_PCS_CMN_REGISTERS.PHY_ISO_CMN_CTRL),8,8,0x1);
            }
        }
    }

    /* Enable lanes in register */
    for(laneNum=0;laneNum<CSL_SERDES_MAX_LANES;laneNum++)
    {
        if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
        {
            CSL_serdesEnableLanes(serdesLaneEnableParams->baseAddr, laneNum, serdesLaneEnableParams->phyType,serdesLaneEnableParams->serdesInstance);
        }
    }

    /* Enable 125MHz pipe clock for USB */
    if(serdesLaneEnableParams->phyType == CSL_SERDES_PHY_TYPE_USB)
    {
        for(laneNum=0;laneNum<CSL_SERDES_MAX_LANES;laneNum++)
        {
            if ((serdesLaneEnableParams->laneMask & (1<<laneNum))!=0)
            {
                /* Enable P0_FULLRT_DIV/P1_FULLRT_DIV to /4 for 125MHz pipe clock */
                CSL_FINSR(*(volatile uint32_t *)(uintptr_t)(serdesLaneEnableParams->baseAddr + 0x480 + laneNum*0x40),23, 22, 0x2);
            }
        }
    }

    /* Release PHY Reset*/
    /* If diagnostic mode, reset and lock checks are handled by isolation and test functions */
    if(!(serdesLaneEnableParams->operatingMode == CSL_SERDES_DIAGNOSTIC_MODE))
    {
        CSL_serdesReleaseReset(serdesLaneEnableParams->baseAddr);

        CSL_serdesCycleDelay(100);

        /* Poll for PLL_OK */
        /* Do not poll for FAST SIM or PIPE mode */
        if (!(serdesLaneEnableParams->operatingMode == CSL_SERDES_FUNCTIONAL_MODE_FAST_SIM ||
                serdesLaneEnableParams->operatingMode == CSL_SERDES_DIAGNOSTIC_MODE_FAST_SIM ||
                serdesLaneEnableParams->phyType == CSL_SERDES_PHY_TYPE_USB ||
                serdesLaneEnableParams->phyType == CSL_SERDES_PHY_TYPE_PCIe))
        {
            do
            {
                pllstat = CSL_serdesGetPLLStatus(serdesLaneEnableParams->baseAddr,
                                                  serdesLaneEnableParams->laneMask,
                                                  serdesLaneEnableParams->serdesInstance);
            }while(pllstat == CSL_SERDES_STATUS_PLL_NOT_LOCKED);
        }

        CSL_serdesCycleDelay(100);
    }

    return status;
}
