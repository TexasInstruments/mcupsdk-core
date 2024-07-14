/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

#include <drivers/pcie.h>
#include <drivers/soc.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/hw_include/cslr_serdes.h>
#include <drivers/hw_include/cslr_serdes_pcie.h>

/* Number of PCIe instances */
#define PCIE_DEVICE_COUNT 2
#define SERDES_LANE_MASK  (0x1U)
#define LANE_FUNC_SEL_MSB 1
#define LANE_FUNC_SEL_LSB 0
#define CLK_SEL_MSB 7
#define CLK_SEL_LSB 4

#define PCIE0 0
#define PCIE1 1

#define SERDES0 0
#define SERDES1 1

/* Pcie0 Lanes, Pcie1 1 lane */
static int32_t Pcie_serdesConfigureLaneFunc(uint32_t deviceNum, uint32_t numLane)
{
    int32_t status = SystemP_FAILURE;
    if (deviceNum == PCIE0)
    {
        CSL_FINSR(*(volatile uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CTRL), LANE_FUNC_SEL_MSB, LANE_FUNC_SEL_LSB, 0x1);
        CSL_FINSR(*(volatile uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_SERDES0_CTRL), CLK_SEL_MSB, CLK_SEL_LSB, 0x6); // Left CML, pass to right CML, two lane case
        status = Sciclient_pmSetModuleClkParent(
            TISCI_DEV_SERDES0,
            TISCI_DEV_SERDES0_BUS_LI_REFCLK,
            TISCI_DEV_SERDES0_BUS_LI_REFCLK_PARENT_ADPLLLJM_HSDIV_WRAP_MAIN_0_BUS_HSDIV_CLKOUT4_CLK,
            0xFFFFFFFFU);

        if(numLane == 2)
        {
            CSL_FINSR(*(volatile uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_SERDES1_CTRL), 1, 0, 0x1);
            CSL_FINSR(*(volatile uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_SERDES1_CTRL), 7, 4, 0x4);
            status = Sciclient_pmSetModuleClkParent(
                TISCI_DEV_SERDES1,
                TISCI_DEV_SERDES1_BUS_RI_REFCLK,
                TISCI_DEV_SERDES1_BUS_RI_REFCLK_PARENT_ADPLLLJM_HSDIV_WRAP_MAIN_0_BUS_HSDIV_CLKOUT4_CLK,
                0xFFFFFFFFU);
        }
    }
    else
    {
        CSL_FINSR(*(volatile uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_SERDES1_CTRL), 1, 0, 0x0);
        CSL_FINSR(*(volatile uint32_t *)(CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_SERDES1_CTRL), 7, 4, 0x1);
        status = Sciclient_pmSetModuleClkParent(
            TISCI_DEV_SERDES1,
            TISCI_DEV_SERDES1_BUS_RI_REFCLK,
            TISCI_DEV_SERDES1_BUS_RI_REFCLK_PARENT_ADPLLLJM_HSDIV_WRAP_MAIN_0_BUS_HSDIV_CLKOUT4_CLK,
            0xFFFFFFFFU);
    }

    return status;
}

static int32_t Pcie_serdesConfig(int32_t serdesInst)
{
    CSL_SerdesLaneEnableParams serdesLaneEnableParams;
    uint32_t serdesBase;
    uint32_t status = SystemP_SUCCESS;

    memset(&serdesLaneEnableParams, 0, sizeof(serdesLaneEnableParams));

    if (serdesInst == 0)
    {
        serdesBase = CSL_SERDES0_BASE;
    }
    else
    {
        serdesBase = CSL_SERDES1_BASE;
    }

    CSL_serdesPorReset(serdesBase);

    serdesLaneEnableParams.baseAddr         = serdesBase;
    serdesLaneEnableParams.refClock         = CSL_SERDES_REF_CLOCK_100M;
    serdesLaneEnableParams.linkRate         = CSL_SERDES_LINK_RATE_8G;
    serdesLaneEnableParams.numLanes         = 1;
    serdesLaneEnableParams.laneMask         = SERDES_LANE_MASK;
    serdesLaneEnableParams.phyType          = CSL_SERDES_PHY_TYPE_PCIe;
    serdesLaneEnableParams.operatingMode    = CSL_SERDES_FUNCTIONAL_MODE_FAST_SIM;
    serdesLaneEnableParams.forceAttBoost    = CSL_SERDES_FORCE_ATT_BOOST_DISABLED;
    serdesLaneEnableParams.laneCtrlRate[0]  = CSL_SERDES_LANE_FULL_RATE;
    serdesLaneEnableParams.loopbackMode[0]  = CSL_SERDES_LOOPBACK_DISABLED;

    status = CSL_serdesPCIeInit(serdesLaneEnableParams.baseAddr,
                                serdesLaneEnableParams.numLanes,
                                serdesLaneEnableParams.refClock,
                                serdesLaneEnableParams.linkRate);

    CSL_serdesLaneEnable(&serdesLaneEnableParams);

    CSL_serdesWaitForCMUOK(serdesLaneEnableParams.baseAddr);

    return status;
}


/* Initialize Serdes corresponding to the PCIe device */
int32_t Pcie_serdesInit(Pcie_Handle handle, uint32_t deviceNum)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t numLanes;
    Pcie_Config *pcieCfg;

    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, 1);

    pcieCfg = (Pcie_Config *)handle;
    numLanes = pcieCfg->attrs->numLanes;

    status = Pcie_serdesConfigureLaneFunc(deviceNum, numLanes);

    if (deviceNum == 0)
    {
        status = Pcie_serdesConfig(SERDES0);
        if(numLanes == 2)
        {
            status = Pcie_serdesConfig(SERDES1);
        }
    }

    if (deviceNum == 1)
    {
        status = Pcie_serdesConfig(SERDES1);
    }

    return status;
}
