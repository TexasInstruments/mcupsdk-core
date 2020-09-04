/*!
 *  \file CUST_PHY_dp83869.c
 *
 *  \brief
 *  PHY implementation for TI DP83869.
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2022-02-24
 *
 *  \copyright
 *  Copyright (c) 2021, KUNBUS GmbH<br /><br />
 *  All rights reserved.<br />
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:<br />
 *  <ol>
 *  <li>Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.</li>
 *  <li>Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.</li>
 *  <li>Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.</li>
 *  </ol>
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <CUST_PHY_dp83869.h>
#include <custom_phy.h>

#define CUST_PHY_DP83869_IEEE_IDENT                             (0x2000A0F0u)
#define CUST_PHY_DP83869_IEEE_IDENT_MASK                        (0xFFFFFFF0u)
#define CUST_PHY_DP83869_AUTONEGTIMEOUT                         (1000u)

#define CUST_PHY_DP83869_BMCR_REG                               (0x00)
#define CUST_PHY_DP83869_BMSR_REG                               (0x01)
#define CUST_PHY_DP83869_ANAR_REG                               (0x04)
#define CUST_PHY_DP83869_CFG1_REG                               (0x09)
#define CUST_PHY_DP83869_PHYCR_REG                              (0x10)
#define CUST_PHY_DP83869_PHYSTS_REG                             (0x11)
#define CUST_PHY_DP83869_REGCR_REG                              (0x0d)
#define CUST_PHY_DP83869_ADDAR_REG                              (0x0e)
#define CUST_PHY_DP83869_LEDSCFG1_REG                           (0x18)
#define CUST_PHY_DP83869_LEDSCFG3_REG                           (0x1A)
#define CUST_PHY_DP83869_GEN_CFG4_REG                           (0x1e)
#define CUST_PHY_DP83869_GEN_CONTROL_REG                        (0x1f)

#define CUST_PHY_DP83869_EXT_RGMIICTL_REG                       (0x32)
#define CUST_PHY_DP83869_EXT_RGMIICTL2_REG                      (0x33)

#define CUST_PHY_DP83869_EXT_FLDCFG_REG                         (0x2D)
#define CUST_PHY_DP83869_EXT_100CFG_REG                         (0x43)
#define CUST_PHY_DP83869_OP_MODE_DECODE                         (0x1DF)

#define CUST_PHY_DP83869_BMCR_REG_RESET                         (0x8000)
#define CUST_PHY_DP83869_BMCR_REG_MII_LOOPBACK                  (0x4000)
#define CUST_PHY_DP83869_BMCR_REG_SPEED_SEL_LSB                 (0x2000)
#define CUST_PHY_DP83869_BMCR_REG_ANEG_ENABLE                   (0x1000)
#define CUST_PHY_DP83869_BMCR_POWER_DOWN                        (0x0800)
#define CUST_PHY_DP83869_BMCR_REG_ISOLATE                       (0x0400)
#define CUST_PHY_DP83869_BMCR_REG_ANEG_RESTART                  (0x0200)
#define CUST_PHY_DP83869_BMCR_REG_DUPLEX_EN                     (0x0100)
#define CUST_PHY_DP83869_BMCR_REG_COL_TST                       (0x0080)
#define CUST_PHY_DP83869_BMCR_REG_SPEED_SEL_MSB                 (0x0040)

#define CUST_PHY_DP83869_BMSR_REG_ANEG_COMPLETE                 (0x0020)
#define CUST_PHY_DP83869_BMSR_REG_LINK_STS                      (0x0004)

#define CUST_PHY_DP83869_CFG1_REG_1000BT_HDX                    (0x0100)
#define CUST_PHY_DP83869_CFG1_REG_1000BT_FDX                    (0x0200)
#define CUST_PHY_DP83869_PHYCR_REG_MDI_AUTO_CROSSOVER           (0x0040)
#define CUST_PHY_DP83869_PHYCR_REG_MDI_MANUAL_CROSSOVER         (0x0020)
#define CUST_PHY_DP83869_PHYCR_REG_1000BT_FDX                   (0x0200)
#define CUST_PHY_DP83869_PHYSTS_REG_SPEED                       (0xC000)
#define CUST_PHY_DP83869_PHYSTS_REG_DUPLEX                      (0x2000)
#define CUST_PHY_DP83869_REGCR_ADDRESS_ACCESS                   (0x001f)

#define CUST_PHY_DP83869_DATA_NORMAL_ACCESS                     (0x401f)
#define CUST_PHY_DP83869_DATA_OPMODE_DECODE_MII                 (1u<<5)
#define CUST_PHY_DP83869_DATA_OPMODE_RGMII2COPPER               (0u<<0)

#define CUST_PHY_DP83869_ANAR_100FD_SUPPORT                     (0x0100)
#define CUST_PHY_DP83869_ANAR_100HD_SUPPORT                     (0x0080)
#define CUST_PHY_DP83869_ANAR_10FD_SUPPORT                      (0x0040)
#define CUST_PHY_DP83869_ANAR_10HD_SUPPORT                      (0x0020)

#define CUST_PHY_DP83869_PHYSTS_SPEED_10M                       (0x0000)
#define CUST_PHY_DP83869_PHYSTS_SPEED_100M                      (0x4000)
#define CUST_PHY_DP83869_PHYSTS_SPEED_1000M                     (0xC000)

#define CUST_PHY_DP83869_LEDSCFG1_LED_GPIO_SEL                  (12)
#define CUST_PHY_DP83869_LEDSCFG1_LED2_SEL                      ( 8)
#define CUST_PHY_DP83869_LEDSCFG1_LED1_SEL                      ( 4)
#define CUST_PHY_DP83869_LEDSCFG1_LED0_SEL                      ( 0)
#define CUST_PHY_DP83869_LEDSCFG1_LINK_ESTABLISHED              (0x0)
#define CUST_PHY_DP83869_LEDSCFG1_RX_OR_TX_ACTIVITY             (0x1)
#define CUST_PHY_DP83869_LEDSCFG1_TX_ACTIVITY                   (0x2)
#define CUST_PHY_DP83869_LEDSCFG1_RX_ACTIVITY                   (0x3)
#define CUST_PHY_DP83869_LEDSCFG1_COLLISSION_DETECT             (0x4)
#define CUST_PHY_DP83869_LEDSCFG1_1000BT_LINK_ESTABLISHED       (0x5)
#define CUST_PHY_DP83869_LEDSCFG1_100BT_LINK_ESTABLISHED        (0x6)
#define CUST_PHY_DP83869_LEDSCFG1_10BT_LINK_ESTABLISHED         (0x7)
#define CUST_PHY_DP83869_LEDSCFG1_10_100BT_LINK_ESTABLISHED     (0x8)
#define CUST_PHY_DP83869_LEDSCFG1_100_1000BT_LINK_ESTABLISHED   (0x9)
#define CUST_PHY_DP83869_LEDSCFG1_FULL_DUPLEX                   (0xA)
#define CUST_PHY_DP83869_LEDSCFG1_LINK_BLINK_ACTIVITY           (0xB)
#define CUST_PHY_DP83869_LEDSCFG1_RX_OR_TX_ERROR                (0xD)
#define CUST_PHY_DP83869_LEDSCFG1_RX_ERROR                      (0xE)
#define CUST_PHY_DP83869_LEDSCFG1_MASK                          (0xF)

#define CUST_PHY_DP83869_LEDSCFG3_LEDBLINK_50MS                 (0x0)
#define CUST_PHY_DP83869_LEDSCFG3_LEDBLINK_100MS                (0x1)
#define CUST_PHY_DP83869_LEDSCFG3_LEDBLINK_200MS                (0x2)
#define CUST_PHY_DP83869_LEDSCFG3_LEDBLINK_500MS                (0x3)
#define CUST_PHY_DP83869_LEDSCFG3_LEDBLINK_MASK                 (0x3)
#define CUST_PHY_DP83869_LEDSCFG3_LEDBLINK_SHIFT                (0x0)

#define CUST_PHY_DP83869_GEN_CFG4_REG_ENA_EXTFD                 (0x0800)
#define CUST_PHY_DP83869_GEN_CONTROL_REG_SW_RESET               (0x8000)
#define CUST_PHY_DP83869_GEN_CONTROL_REG_SW_RESTART             (0x4000)

#define CUST_PHY_DP83869_EXT_100CFG_REG_IPGDET_ENA              (0x0010)
#define CUST_PHY_DP83869_EXT_100CFG_REG_ODDN_ENA                (0x0002)
#define CUST_PHY_DP83869_EXT_100CFG_REG_FRXDV_ENA               (0x0001)

#define CUST_PHY_DP83869_LOW_LATENCY_10_100_ENABLE              (1<<2)

#define CUST_PHY_DP83869_RGMII_TX_HALF_FULL_THR_SHIFT           (3)
#define CUST_PHY_DP83869_RGMII_RX_HALF_FULL_THR_SHIFT           (5)

#define CUST_PHY_DP83869_LINK_PRESENT_AUTONEG_POLL_STEP         (10u)
#define CUST_PHY_DP83869_LINK_ABSENT_AUTONEG_POLL_STEP          (1u)

/* DP8 global */
#define CLEARREGBIT(reg, bitNum) \
    (reg) = ((reg) & ~(1 << (bitNum)))

#define SETREGBIT(reg, bitNum) \
    (reg) = ((reg) | (1 << (bitNum)))

#define SETREGVAL(reg, shift, mask, value) \
    (reg) = (((reg) & (~((mask) << (shift)))) | ((value) << (shift)))

static uint16_t CUST_PHY_DP83869_readExtendedRegister       (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p
                                                            ,uint16_t   extAddress_p);
static void     CUST_PHY_DP83869_writeExtendedRegister      (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p
                                                            ,uint16_t   extAddress_p
                                                            ,uint16_t   value_p);
static void     CUST_PHY_DP83869_softwareReset              (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p);
static void     CUST_PHY_DP83869_softwareRestart            (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p);
static void     CUST_PHY_DP83869_enableAutoMDIX             (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p);
static void     CUST_PHY_DP83869_setMIIMode                 (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p);
static void     CUST_PHY_DP83869_configMLED                 (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p);
static void     CUST_PHY_DP83869_enableExtFD                (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p);
static void     CUST_PHY_DP83869_enableODDNibbleDet         (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p);
static void     CUST_PHY_DP83869_enableRxErrIdle            (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p);
static void     CUST_PHY_DP83869_configLed                  (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p);
static void     CUST_PHY_DP83869_configLedBlink             (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p);
static void     CUST_PHY_DP83869_enableFastLinkDownDet      (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p);
static void     CUST_PHY_DP83869_enableFastRXDVDet          (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p);
static void     CUST_PHY_DP83869_cofigSwStrapDone           (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p);
static void     CUST_PHY_DP83869_setPowerMode               (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p
                                                            ,bool       powerDown_p);
static bool     CUST_PHY_DP83869_getPowerMode               (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p);
static void     CUST_PHY_DP83869_setLinkConfig              (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p
                                                            ,bool       autoNeg_p
                                                            ,uint16_t   linkSpeed_p
                                                            ,bool       fullDuplex_p
                                                            ,uint32_t*  pResult_p);
static bool     CUST_PHY_DP83869_getAutoNegotiation         (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p);
static void     CUST_PHY_DP83869_setMdixMode                (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p
                                                            ,uint32_t   mdixMode_p);
static uint32_t CUST_PHY_DP83869_getMdixMode                (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p);
static void     CUST_PHY_DP83869_disable1GbAdver            (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p);
static void     CUST_PHY_DP83869_rgmiiLowLatencyEnable      (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p);
static void     CUST_PHY_DP83869_rgmiiTxHalfFullThreshold   (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p
                                                            ,uint32_t   threshold_p);
static void     CUST_PHY_DP83869_rgmiiRxHalfFullThreshold   (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p
                                                            ,uint32_t   threshold_p);
static void     CUST_PHY_DP83869_getSpeedDuplex             (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p
                                                            ,void*      pData_p
                                                            ,uint32_t   dataSize_p);
static uint32_t CUST_PHY_DP83869_openFxn                    (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p
                                                            ,void*      pParam_p);
static uint32_t CUST_PHY_DP83869_commandFxn                 (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p
                                                            ,uint32_t   command_p
                                                            ,void*      pData_p
                                                            ,uint32_t   dataSize_p);
static void     CUST_PHY_DP83869_closeFxn                   (void*      pAppCtxt_p
                                                            ,void*      pStackCtxt_p);
/*! <!-- Description: -->
 *
 *  \brief
 *  Detect Phy Type and setup access structures accordingly
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pPhyLibCtxt_p	Context of External PhyLib.
 *  \param[in]  phyId_p         Phy ID read from hardware
 *  \param[in]  pPhyLibDesc_p   External PhyLib Hooks
 *  \return     0 on success and Phy detected, error code otherwise
 *
 *  <!-- Group: -->
 *
 *  \ingroup CUST_PHY
 *
 * */
int16_t CUST_PHY_DP83869_detect(void* pPhyLibCtxt_p, uint32_t phyId_p, CUST_PHY_SPhyDescriptor_t *pPhyLibDesc_p)
{
    int16_t     retVal  = -CUST_PHY_STATUS_ERROR_UNKNOWN_PHY;

    OSALUNREF_PARM(pPhyLibCtxt_p);

    /* exact match */
    if ((phyId_p & CUST_PHY_DP83869_IEEE_IDENT_MASK) == CUST_PHY_DP83869_IEEE_IDENT)
    {
        OSAL_printf("DP83869 detected\r\n");
        pPhyLibDesc_p->softwareReset                 = CUST_PHY_DP83869_softwareReset;
        pPhyLibDesc_p->softwareRestart               = CUST_PHY_DP83869_softwareRestart;
        pPhyLibDesc_p->enableAutoMDIX                = CUST_PHY_DP83869_enableAutoMDIX;
        pPhyLibDesc_p->setMiiMode                    = CUST_PHY_DP83869_setMIIMode;
        pPhyLibDesc_p->setPowerMode                  = CUST_PHY_DP83869_setPowerMode;
        pPhyLibDesc_p->getPowerMode                  = CUST_PHY_DP83869_getPowerMode;
        pPhyLibDesc_p->configMLED                    = CUST_PHY_DP83869_configMLED;
        pPhyLibDesc_p->enableExtFD                   = CUST_PHY_DP83869_enableExtFD;
        pPhyLibDesc_p->enableODDNibbleDet            = CUST_PHY_DP83869_enableODDNibbleDet;
        pPhyLibDesc_p->enableRxErrIdle               = CUST_PHY_DP83869_enableRxErrIdle;
        pPhyLibDesc_p->configLed                     = CUST_PHY_DP83869_configLed;
        pPhyLibDesc_p->configLedBlink                = CUST_PHY_DP83869_configLedBlink;
        pPhyLibDesc_p->enableFastLinkDownDet         = CUST_PHY_DP83869_enableFastLinkDownDet;
        pPhyLibDesc_p->enableFastRXDVDet             = CUST_PHY_DP83869_enableFastRXDVDet;
        pPhyLibDesc_p->configSwStrapDone             = CUST_PHY_DP83869_cofigSwStrapDone;
        pPhyLibDesc_p->setLinkConfig                 = CUST_PHY_DP83869_setLinkConfig;
        pPhyLibDesc_p->getAutoNegotiation            = CUST_PHY_DP83869_getAutoNegotiation;
        pPhyLibDesc_p->setMdixMode                   = CUST_PHY_DP83869_setMdixMode;
        pPhyLibDesc_p->getMdixMode                   = CUST_PHY_DP83869_getMdixMode;
        pPhyLibDesc_p->disable1GbAdver               = CUST_PHY_DP83869_disable1GbAdver;
        pPhyLibDesc_p->rgmiiLowLatencyEnable         = CUST_PHY_DP83869_rgmiiLowLatencyEnable;
        pPhyLibDesc_p->rgmiiTxHalfFullThreshold      = CUST_PHY_DP83869_rgmiiTxHalfFullThreshold;
        pPhyLibDesc_p->rgmiiRxHalfFullThreshold      = CUST_PHY_DP83869_rgmiiRxHalfFullThreshold;
        pPhyLibDesc_p->getSpeedDuplex                = CUST_PHY_DP83869_getSpeedDuplex;
        pPhyLibDesc_p->openFxn                       = CUST_PHY_DP83869_openFxn;
        pPhyLibDesc_p->commandFxn                    = CUST_PHY_DP83869_commandFxn;
        pPhyLibDesc_p->closeFxn                      = CUST_PHY_DP83869_closeFxn;

        retVal = 0;
    }

    return retVal;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Issue PHY Reset by software (used if no Hard Reset is available)
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p      application context
 *  \param[in]  pStackCtxt_p    stack context
 *
 *  <!-- Group: -->
 *
 *  \ingroup CUST_PHY
 *
 * */
void CUST_PHY_DP83869_softwareReset(void* pAppCtxt_p, void* pStackCtxt_p)
{
    uint16_t    phyRegVal   = 0;

    OSALUNREF_PARM(pAppCtxt_p);

    CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_GEN_CONTROL_REG, &phyRegVal);
    phyRegVal |= CUST_PHY_DP83869_GEN_CONTROL_REG_SW_RESET;
    CUST_PHY_writeReg(pStackCtxt_p, CUST_PHY_DP83869_GEN_CONTROL_REG, phyRegVal);

    do
    {
        phyRegVal = 0;
        CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_GEN_CONTROL_REG, &phyRegVal);
    }
    while (phyRegVal & CUST_PHY_DP83869_GEN_CONTROL_REG_SW_RESET);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Issue PHY Restart by software (register left intact)
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p      application context
 *  \param[in]  pStackCtxt_p    stack context
 *
 *  <!-- Group: -->
 *
 *  \ingroup CUST_PHY
 *
 * */
void CUST_PHY_DP83869_softwareRestart(void* pAppCtxt_p, void* pStackCtxt_p)
{
    uint16_t    phyRegVal   = 0;
    bool        hadLink     = false;

    CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_BMSR_REG, &phyRegVal);
    if ( phyRegVal & CUST_PHY_DP83869_BMSR_REG_LINK_STS)
    {
        CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_BMSR_REG, &phyRegVal);
        if ( phyRegVal & CUST_PHY_DP83869_BMSR_REG_LINK_STS)
        {
            OSAL_printf("0x%x:Link up\r\n", CUST_PHY_getPhyAddr(pStackCtxt_p));
            hadLink = true;
        }
    }

    CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_BMCR_REG, &phyRegVal);
    phyRegVal |= CUST_PHY_DP83869_BMCR_REG_ANEG_ENABLE;
    CUST_PHY_writeReg(pStackCtxt_p, CUST_PHY_DP83869_BMCR_REG, phyRegVal);

    CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_GEN_CONTROL_REG, &phyRegVal);
    phyRegVal |= CUST_PHY_DP83869_GEN_CONTROL_REG_SW_RESTART;
    CUST_PHY_writeReg(pStackCtxt_p, CUST_PHY_DP83869_GEN_CONTROL_REG, phyRegVal);

    do
    {
        phyRegVal = 0;
        CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_GEN_CONTROL_REG, &phyRegVal);
    }
    while (phyRegVal & CUST_PHY_DP83869_GEN_CONTROL_REG_SW_RESTART);
    OSAL_SCHED_sleep(10); /* just for settle */

    if (hadLink)
    {
        /* we expect to gain link agn */
        do
        {
            CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_BMSR_REG, &phyRegVal);
            if ( !(phyRegVal & CUST_PHY_DP83869_BMSR_REG_LINK_STS))
            {
                OSAL_SCHED_sleep(100); /* if we got no re-link, waiting is not too bad at all */
            }
        } while (!(phyRegVal & CUST_PHY_DP83869_BMSR_REG_LINK_STS));
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Enable Auto MDIX
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p         application context
 *  \param[in]  pStackCtxt_p       stack context
 *
 *  <!-- Group: -->
 *
 *  \ingroup CUST_PHY
 *
 * */
void CUST_PHY_DP83869_enableAutoMDIX(void* pAppCtxt_p, void* pStackCtxt_p)
{
    uint16_t    phyRegVal   = 0;
    uint32_t    timeoutStep = 10;

    OSALUNREF_PARM(pAppCtxt_p);

    CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_BMSR_REG, &phyRegVal);
    if ( phyRegVal & CUST_PHY_DP83869_BMSR_REG_LINK_STS)
    {
        timeoutStep = CUST_PHY_DP83869_LINK_PRESENT_AUTONEG_POLL_STEP;
    }
    else
    {
        timeoutStep = CUST_PHY_DP83869_LINK_ABSENT_AUTONEG_POLL_STEP;
    }

    OSAL_printf("Phy %lu : Enable AutoMDIX\r\n", CUST_PHY_getPhyAddr(pStackCtxt_p));
    CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_BMCR_REG, &phyRegVal);
    phyRegVal |= CUST_PHY_DP83869_BMCR_REG_ANEG_ENABLE;
    CUST_PHY_writeReg(pStackCtxt_p, CUST_PHY_DP83869_BMCR_REG, phyRegVal);

    OSAL_printf("Phy %lu : Restart ANEG\r\n", CUST_PHY_getPhyAddr(pStackCtxt_p));
    phyRegVal |= CUST_PHY_DP83869_BMCR_REG_ANEG_RESTART;
    CUST_PHY_writeReg(pStackCtxt_p, CUST_PHY_DP83869_BMCR_REG, phyRegVal);

    do
    {
        phyRegVal = 0;
        CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_BMCR_REG, &phyRegVal);
    }
    while (phyRegVal & CUST_PHY_DP83869_BMCR_REG_ANEG_RESTART);

    CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_BMSR_REG, &phyRegVal);
    {
        uint16_t    timeout     = CUST_PHY_DP83869_AUTONEGTIMEOUT;

        do
        {
            phyRegVal = 0;
            --timeout;
            CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_BMSR_REG, &phyRegVal);
            if (((phyRegVal & CUST_PHY_DP83869_BMSR_REG_ANEG_COMPLETE) != CUST_PHY_DP83869_BMSR_REG_ANEG_COMPLETE))
            {
                OSAL_SCHED_sleep(timeoutStep); /* delay for 10 msecs steps */
            }
        }
        while (timeout && ((phyRegVal & CUST_PHY_DP83869_BMSR_REG_ANEG_COMPLETE) != CUST_PHY_DP83869_BMSR_REG_ANEG_COMPLETE));
    }
    OSAL_printf("Phy %lu: BMSR post ANEG: %x ANEG:%s Link:%s\r\n", CUST_PHY_getPhyAddr(pStackCtxt_p), phyRegVal,
                ((phyRegVal&CUST_PHY_DP83869_BMSR_REG_ANEG_COMPLETE)==CUST_PHY_DP83869_BMSR_REG_ANEG_COMPLETE)?"Complete":"NComplete",
                ((phyRegVal & CUST_PHY_DP83869_BMSR_REG_LINK_STS)==CUST_PHY_DP83869_BMSR_REG_LINK_STS)?"Yes":"No");
    return;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Set MII Mode
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p         application context
 *  \param[in]  pStackCtxt_p       stack context
 *
 *  <!-- Group: -->
 *
 *  \ingroup CUST_PHY
 *
 * */
void CUST_PHY_DP83869_setMIIMode(void *pAppCtxt_p, void *pStackCtxt_p)
{
    uint16_t phyRegVal = 0;

    OSALUNREF_PARM(pAppCtxt_p);

    OSAL_printf("Phy %lu : Disable RGMII mode\r\n", CUST_PHY_getPhyAddr(pStackCtxt_p));

    phyRegVal = CUST_PHY_DP83869_readExtendedRegister(pAppCtxt_p, pStackCtxt_p, CUST_PHY_DP83869_OP_MODE_DECODE);
    phyRegVal |= CUST_PHY_DP83869_DATA_OPMODE_DECODE_MII;
    CUST_PHY_DP83869_writeExtendedRegister(pAppCtxt_p, pStackCtxt_p, CUST_PHY_DP83869_OP_MODE_DECODE, phyRegVal);

    OSAL_printf("Phy %lu : Disable GBit ANEG\r\n", CUST_PHY_getPhyAddr(pStackCtxt_p));
    CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_CFG1_REG, &phyRegVal);

    phyRegVal &= ~CUST_PHY_DP83869_CFG1_REG_1000BT_FDX;
    phyRegVal &= ~CUST_PHY_DP83869_CFG1_REG_1000BT_HDX;

    CUST_PHY_writeReg(pStackCtxt_p, CUST_PHY_DP83869_CFG1_REG, phyRegVal);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Configure PhyMLED to detect RxLink by MLED (e.g. TLK)
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p         application context
 *  \param[in]  pStackCtxt_p       stack context
 *
 *  <!-- Group: -->
 *
 *  \ingroup CUST_PHY
 *
 * */
void CUST_PHY_DP83869_configMLED(void *pAppCtxt_p, void *pStackCtxt_p)
{
    /* currently no multicolor LED support for DP 83869 */
    OSALUNREF_PARM(pAppCtxt_p);
    OSALUNREF_PARM(pStackCtxt_p);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Set Ext Full Duplex enable
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p         application context
 *  \param[in]  pStackCtxt_p       stack context
 *
 *  <!-- Group: -->
 *
 *  \ingroup CUST_PHY
 *
 * */
void CUST_PHY_DP83869_enableExtFD(void *pAppCtxt_p, void *pStackCtxt_p)
{
    uint16_t    phyRegVal   = 0;

    OSALUNREF_PARM(pAppCtxt_p);

    CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_GEN_CFG4_REG, &phyRegVal);
    phyRegVal |= CUST_PHY_DP83869_GEN_CFG4_REG_ENA_EXTFD;
    CUST_PHY_writeReg(pStackCtxt_p, CUST_PHY_DP83869_GEN_CFG4_REG, phyRegVal);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Set ODD Nibble detection enable
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p         application context
 *  \param[in]  pStackCtxt_p       stack context
 *
 *  <!-- Group: -->
 *
 *  \ingroup CUST_PHY
 *
 * */
void CUST_PHY_DP83869_enableODDNibbleDet(void *pAppCtxt_p, void *pStackCtxt_p)
{
    uint16_t    phyRegVal   = 0;

    phyRegVal = CUST_PHY_DP83869_readExtendedRegister(pAppCtxt_p, pStackCtxt_p, CUST_PHY_DP83869_EXT_100CFG_REG);

    phyRegVal |= CUST_PHY_DP83869_EXT_100CFG_REG_ODDN_ENA;

    CUST_PHY_DP83869_writeExtendedRegister(pAppCtxt_p, pStackCtxt_p, CUST_PHY_DP83869_EXT_100CFG_REG, phyRegVal);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Set Rx Error Idle enable
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p         application context
 *  \param[in]  pStackCtxt_p       stack context
 *
 *  <!-- Group: -->
 *
 *  \ingroup CUST_PHY
 *
 * */
void CUST_PHY_DP83869_enableRxErrIdle(void *pAppCtxt_p, void *pStackCtxt_p)
{
    /* no support on RX Err Idle for DP 83869 */
    OSALUNREF_PARM(pAppCtxt_p);
    OSALUNREF_PARM(pStackCtxt_p);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Configure PHY LEDs
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p         application context
 *  \param[in]  pStackCtxt_p       stack context
 *
 *  <!-- Group: -->
 *
 *  \ingroup CUST_PHY
 *
 * */
void CUST_PHY_DP83869_configLed(void *pAppCtxt_p, void *pStackCtxt_p)
{
    uint16_t    phyRegVal   = 0;
    uint8_t     mode        = CUST_PHY_DP83869_LED_CFG_MODE3;

    OSALUNREF_PARM(pAppCtxt_p);

    CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_LEDSCFG1_REG, &phyRegVal);

    switch (mode)
    {
    case CUST_PHY_DP83869_LED_CFG_MODE1:
    {
        SETREGVAL(phyRegVal, CUST_PHY_DP83869_LEDSCFG1_LED0_SEL,        CUST_PHY_DP83869_LEDSCFG1_MASK, CUST_PHY_DP83869_LEDSCFG1_LINK_ESTABLISHED);
        SETREGVAL(phyRegVal, CUST_PHY_DP83869_LEDSCFG1_LED1_SEL,        CUST_PHY_DP83869_LEDSCFG1_MASK, CUST_PHY_DP83869_LEDSCFG1_1000BT_LINK_ESTABLISHED);
        SETREGVAL(phyRegVal, CUST_PHY_DP83869_LEDSCFG1_LED2_SEL,        CUST_PHY_DP83869_LEDSCFG1_MASK, CUST_PHY_DP83869_LEDSCFG1_LINK_BLINK_ACTIVITY);
        SETREGVAL(phyRegVal, CUST_PHY_DP83869_LEDSCFG1_LED_GPIO_SEL,    CUST_PHY_DP83869_LEDSCFG1_MASK, CUST_PHY_DP83869_LEDSCFG1_10_100BT_LINK_ESTABLISHED);
    } break;
    case CUST_PHY_DP83869_LED_CFG_MODE2:
    {
        SETREGVAL(phyRegVal, CUST_PHY_DP83869_LEDSCFG1_LED0_SEL,        CUST_PHY_DP83869_LEDSCFG1_MASK, CUST_PHY_DP83869_LEDSCFG1_LINK_ESTABLISHED);
        SETREGVAL(phyRegVal, CUST_PHY_DP83869_LEDSCFG1_LED1_SEL,        CUST_PHY_DP83869_LEDSCFG1_MASK, CUST_PHY_DP83869_LEDSCFG1_1000BT_LINK_ESTABLISHED);
        SETREGVAL(phyRegVal, CUST_PHY_DP83869_LEDSCFG1_LED2_SEL,        CUST_PHY_DP83869_LEDSCFG1_MASK, CUST_PHY_DP83869_LEDSCFG1_RX_OR_TX_ACTIVITY);
        SETREGVAL(phyRegVal, CUST_PHY_DP83869_LEDSCFG1_LED_GPIO_SEL,    CUST_PHY_DP83869_LEDSCFG1_MASK, CUST_PHY_DP83869_LEDSCFG1_10_100BT_LINK_ESTABLISHED);
    } break;
    case CUST_PHY_DP83869_LED_CFG_MODE3:
    {
        SETREGVAL(phyRegVal, CUST_PHY_DP83869_LEDSCFG1_LED0_SEL,        CUST_PHY_DP83869_LEDSCFG1_MASK, CUST_PHY_DP83869_LEDSCFG1_LINK_ESTABLISHED);
        SETREGVAL(phyRegVal, CUST_PHY_DP83869_LEDSCFG1_LED1_SEL,        CUST_PHY_DP83869_LEDSCFG1_MASK, CUST_PHY_DP83869_LEDSCFG1_RX_ERROR);
        SETREGVAL(phyRegVal, CUST_PHY_DP83869_LEDSCFG1_LED2_SEL,        CUST_PHY_DP83869_LEDSCFG1_MASK, CUST_PHY_DP83869_LEDSCFG1_LINK_BLINK_ACTIVITY);
        SETREGVAL(phyRegVal, CUST_PHY_DP83869_LEDSCFG1_LED_GPIO_SEL,    CUST_PHY_DP83869_LEDSCFG1_MASK, CUST_PHY_DP83869_LEDSCFG1_10_100BT_LINK_ESTABLISHED);
    } break;
    }

    CUST_PHY_writeReg(pStackCtxt_p, CUST_PHY_DP83869_LEDSCFG1_REG, phyRegVal);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Configure PHY Blink LED mode
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p         application context
 *  \param[in]  pStackCtxt_p       stack context
 *
 *  <!-- Group: -->
 *
 *  \ingroup CUST_PHY
 *
 * */
void CUST_PHY_DP83869_configLedBlink(void *pAppCtxt_p, void *pStackCtxt_p)
{
    uint16_t    phyRegVal   = 0;
    uint16_t    value       = CUST_PHY_DP83869_LED_BLINK_50;

    OSALUNREF_PARM(pAppCtxt_p);

    CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_LEDSCFG3_REG, &phyRegVal);

    switch (value)
    {
    case CUST_PHY_DP83869_LED_BLINK_500: SETREGVAL(phyRegVal, CUST_PHY_DP83869_LEDSCFG3_LEDBLINK_SHIFT,
                                                   CUST_PHY_DP83869_LEDSCFG3_LEDBLINK_MASK,
                                                   CUST_PHY_DP83869_LEDSCFG3_LEDBLINK_500MS); break;
    case CUST_PHY_DP83869_LED_BLINK_200: SETREGVAL(phyRegVal, CUST_PHY_DP83869_LEDSCFG3_LEDBLINK_SHIFT,
                                                   CUST_PHY_DP83869_LEDSCFG3_LEDBLINK_MASK,
                                                   CUST_PHY_DP83869_LEDSCFG3_LEDBLINK_200MS); break;
    case CUST_PHY_DP83869_LED_BLINK_100: SETREGVAL(phyRegVal, CUST_PHY_DP83869_LEDSCFG3_LEDBLINK_SHIFT,
                                                   CUST_PHY_DP83869_LEDSCFG3_LEDBLINK_MASK,
                                                   CUST_PHY_DP83869_LEDSCFG3_LEDBLINK_100MS); break;
    case CUST_PHY_DP83869_LED_BLINK_50:  SETREGVAL(phyRegVal, CUST_PHY_DP83869_LEDSCFG3_LEDBLINK_SHIFT,
                                                   CUST_PHY_DP83869_LEDSCFG3_LEDBLINK_MASK,
                                                   CUST_PHY_DP83869_LEDSCFG3_LEDBLINK_50MS); break;
    }

    phyRegVal &= ~0x4; /* Disable Bypass LED stretching */

    CUST_PHY_writeReg(pStackCtxt_p, CUST_PHY_DP83869_LEDSCFG3_REG, phyRegVal);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Set fast link down Detection enable
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p         application context
 *  \param[in]  pStackCtxt_p       stack context
 *
 *  <!-- Group: -->
 *
 *  \ingroup CUST_PHY
 *
 * */
void CUST_PHY_DP83869_enableFastLinkDownDet(void *pAppCtxt_p, void *pStackCtxt_p)
{
    uint16_t    phyRegVal   = 0;
    uint8_t     value       = CUST_PHY_DP83869_FAST_LINKDOWN_SIGENERGY
                            | CUST_PHY_DP83869_FAST_LINKDOWN_RXERR;

    OSALUNREF_PARM(pAppCtxt_p);

    phyRegVal = CUST_PHY_DP83869_readExtendedRegister(pAppCtxt_p, pStackCtxt_p,  CUST_PHY_DP83869_EXT_FLDCFG_REG);
    phyRegVal |= value;
    CUST_PHY_DP83869_writeExtendedRegister(pAppCtxt_p, pStackCtxt_p, CUST_PHY_DP83869_EXT_FLDCFG_REG, phyRegVal);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Set Fast RX DV detection enable
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p         application context
 *  \param[in]  pStackCtxt_p       stack context
 *
 *  <!-- Group: -->
 *
 *  \ingroup CUST_PHY
 *
 * */
void CUST_PHY_DP83869_enableFastRXDVDet(void *pAppCtxt_p, void *pStackCtxt_p)
{
    uint16_t    phyRegVal   = 0;

    OSALUNREF_PARM(pAppCtxt_p);

    phyRegVal = CUST_PHY_DP83869_readExtendedRegister(pAppCtxt_p, pStackCtxt_p, CUST_PHY_DP83869_EXT_100CFG_REG);

    phyRegVal |= CUST_PHY_DP83869_EXT_100CFG_REG_FRXDV_ENA;

    CUST_PHY_DP83869_writeExtendedRegister(pAppCtxt_p, pStackCtxt_p, CUST_PHY_DP83869_EXT_100CFG_REG, phyRegVal);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Set SW Strap config done
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p         application context
 *  \param[in]  pStackCtxt_p       stack context
 *
 *  <!-- Group: -->
 *
 *  \ingroup CUST_PHY
 *
 * */
void CUST_PHY_DP83869_cofigSwStrapDone(void *pAppCtxt_p, void *pStackCtxt_p)
{
    /* no SW strap on DP 83869 */
    OSALUNREF_PARM(pAppCtxt_p);
    OSALUNREF_PARM(pStackCtxt_p);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Set power mode
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p      application context
 *  \param[in]  pStackCtxt_p    stack context
 *  \param[in]  powerDown_p         False = Normal Operation - True = Power Down
 *
 *  <!-- Group: -->
 *
 *  \ingroup CUST_PHY
 *
 * */
void CUST_PHY_DP83869_setPowerMode(void* pAppCtxt_p, void *pStackCtxt_p, bool powerDown_p)
{
    uint16_t    phyRegVal   = 0;

    OSALUNREF_PARM(pAppCtxt_p);

    if(powerDown_p)
    {
        //Power Down mode
        CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_BMCR_REG, &phyRegVal);
        phyRegVal |= CUST_PHY_DP83869_BMCR_POWER_DOWN;
        CUST_PHY_writeReg(pStackCtxt_p, CUST_PHY_DP83869_BMCR_REG, phyRegVal);

        do
        {
            phyRegVal = 0;
            CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_BMCR_REG, &phyRegVal);
        }
        while (!(phyRegVal & CUST_PHY_DP83869_BMCR_POWER_DOWN));
    }
    else
    {
        //Normal Operation
        CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_BMCR_REG, &phyRegVal);
        phyRegVal &= ~CUST_PHY_DP83869_BMCR_POWER_DOWN;
        CUST_PHY_writeReg(pStackCtxt_p, CUST_PHY_DP83869_BMCR_REG, phyRegVal);

        do
        {
            phyRegVal = 0;
            CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_BMCR_REG, &phyRegVal);
        }
        while (phyRegVal & CUST_PHY_DP83869_BMCR_POWER_DOWN);
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Get power down
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p      application context
 *  \param[in]  pStackCtxt_p    stack context
 *
 *  \return False = Normal Operation - True = Power Down
 *
 *  <!-- Group: -->
 *
 *  \ingroup CUST_PHY
 *
 * */
bool CUST_PHY_DP83869_getPowerMode(void* pAppCtxt_p, void* pStackCtxt_p)
{
    uint16_t    phyRegVal   = 0;
    bool        powerDown   = false;

    OSALUNREF_PARM(pAppCtxt_p);

    CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_BMCR_REG, &phyRegVal);

    if ((phyRegVal & CUST_PHY_DP83869_BMCR_POWER_DOWN) == CUST_PHY_DP83869_BMCR_POWER_DOWN)
    {
        powerDown = true;
    }

    return powerDown;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Configures PHY for link connection.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p          application context
 *  \param[in]  pStackCtxt_p        stack context
 *  \param[in]  autoNeg_p           selected auto-negotiation mode (false - auto-negotiation inactive, true - auto-negotiation active)
 *  \param[in]  linkSpeed_p         required link speed (10, 100 or 1000 Mbps)
 *  \param[in]  fullDuplex_p        required duplex mode (false - half-duplex, true - full duplex)
 *  \param[out] pResult_p           pointer to variable where result of configuration needs to be stored (0 - Success, other - Error)
 *
 *  <!-- Group: -->
 *
 *  \ingroup CUST_PHY
 *
 * */
void CUST_PHY_DP83869_setLinkConfig(void* pAppCtxt_p, void* pStackCtxt_p, bool autoNeg_p, uint16_t linkSpeed_p, bool fullDuplex_p, uint32_t* pResult_p)
{
    uint16_t    phyRegVal   = 0;
    uint32_t    error       = 0;

    CUST_PHY_DP83869_setPowerMode(pAppCtxt_p, pStackCtxt_p, true);

    CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_CFG1_REG, &phyRegVal);

    phyRegVal &= ~CUST_PHY_DP83869_CFG1_REG_1000BT_HDX;
    phyRegVal &= ~CUST_PHY_DP83869_CFG1_REG_1000BT_FDX;

    CUST_PHY_writeReg(pStackCtxt_p, CUST_PHY_DP83869_CFG1_REG, phyRegVal);

    if (autoNeg_p == true)
    {
        CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_ANAR_REG, &phyRegVal);

        phyRegVal |= CUST_PHY_DP83869_ANAR_100FD_SUPPORT;
        phyRegVal |= CUST_PHY_DP83869_ANAR_100HD_SUPPORT;
        phyRegVal |= CUST_PHY_DP83869_ANAR_10FD_SUPPORT;
        phyRegVal |= CUST_PHY_DP83869_ANAR_10HD_SUPPORT;

        CUST_PHY_writeReg(pStackCtxt_p, CUST_PHY_DP83869_ANAR_REG, phyRegVal);

        CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_BMCR_REG, &phyRegVal);

        phyRegVal |=  CUST_PHY_DP83869_BMCR_REG_ANEG_ENABLE |
                      CUST_PHY_DP83869_BMCR_REG_ANEG_RESTART;

        CUST_PHY_writeReg(pStackCtxt_p, CUST_PHY_DP83869_BMCR_REG, phyRegVal);
    }
    else
    {
        CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_BMCR_REG, &phyRegVal);

        phyRegVal &= ~CUST_PHY_DP83869_BMCR_REG_ANEG_ENABLE;
        phyRegVal &= ~CUST_PHY_DP83869_BMCR_REG_SPEED_SEL_LSB;
        phyRegVal &= ~CUST_PHY_DP83869_BMCR_REG_DUPLEX_EN;
        phyRegVal &= ~CUST_PHY_DP83869_BMCR_REG_SPEED_SEL_MSB;

        switch(linkSpeed_p)
        {
            case 10:
                if (fullDuplex_p == true)
                {
                    phyRegVal |= CUST_PHY_DP83869_BMCR_REG_DUPLEX_EN;
                }
                break;
            case 100:
                phyRegVal |= CUST_PHY_DP83869_BMCR_REG_SPEED_SEL_LSB;

                // duplex mode
                if (fullDuplex_p == true)
                {
                    phyRegVal |= CUST_PHY_DP83869_BMCR_REG_DUPLEX_EN;
                }
                break;
            default:
                // 1000 Mbps not supported (PRU-CODE limitation)
                error = ~0;
                break;
        }

        if (error == 0)
        {
            CUST_PHY_writeReg(pStackCtxt_p, CUST_PHY_DP83869_BMCR_REG, phyRegVal);
        }
    }

    OSAL_waitTimerUs(10);

    CUST_PHY_DP83869_setPowerMode(pAppCtxt_p, pStackCtxt_p, false);

    if (pResult_p != NULL)
    {
        *pResult_p = error;
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Provides auto-negotiation enabled flag.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p          application context
 *  \param[in]  pStackCtxt_p        stack context
 *  \return     bool                auto-negotiation enabled flag (false - auto-negotiation disabled, true - auto-negotiation enabled)
 *
 *  <!-- Group: -->
 *
 *  \ingroup CUST_PHY
 *
 * */
bool CUST_PHY_DP83869_getAutoNegotiation(void* pAppCtxt_p, void* pStackCtxt_p)
{
    uint16_t    phyRegVal = 0;
    bool        autoNeg     = false;

    OSALUNREF_PARM(pAppCtxt_p);

    CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_BMCR_REG, &phyRegVal);

    phyRegVal = phyRegVal & CUST_PHY_DP83869_BMCR_REG_ANEG_ENABLE;

    if (phyRegVal != 0)
    {
        autoNeg = true;
    }

    return autoNeg;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Configures PHY MDI crossover mode.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p          application context
 *  \param[in]  pStackCtxt_p        stack context
 *  \param[in]  mdixMode_p          required MDI crossover mode (0 - Manual MDI configuration, 1 - Manual MDI-X configuration, 2 - Enable automatic crossover)
 *
 *  <!-- Group: -->
 *
 *  \ingroup CUST_PHY
 *
 * */
void CUST_PHY_DP83869_setMdixMode(void* pAppCtxt_p, void* pStackCtxt_p, uint32_t mdixMode_p)
{
    uint16_t    phyRegVal   = 0;

    OSALUNREF_PARM(pAppCtxt_p);

    CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_PHYCR_REG, &phyRegVal);

    phyRegVal &= ~CUST_PHY_DP83869_PHYCR_REG_MDI_MANUAL_CROSSOVER;
    phyRegVal &= ~CUST_PHY_DP83869_PHYCR_REG_MDI_AUTO_CROSSOVER;

    switch(mdixMode_p)
    {
        case CUST_PHY_MDI_MANUAL_CONFIG:
            break;
        case CUST_PHY_MDIX_MANUAL_CONFIG:
            phyRegVal |= CUST_PHY_DP83869_PHYCR_REG_MDI_MANUAL_CROSSOVER;
            break;
        default:
            phyRegVal |= CUST_PHY_DP83869_PHYCR_REG_MDI_AUTO_CROSSOVER;
            break;
    }

    CUST_PHY_writeReg(pStackCtxt_p, CUST_PHY_DP83869_PHYCR_REG, phyRegVal);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Provides actually used PHY MDI crossover mode.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p          application context
 *  \param[in]  pStackCtxt_p        stack context
 *  \return     uint32_t            value of used MDI crossover mode (0 - Manual MDI configuration, 1 - Manual MDI-X configuration, 2 - Enable automatic crossover)
 *
 *  <!-- Group: -->
 *
 *  \ingroup CUST_PHY
 *
 * */
uint32_t CUST_PHY_DP83869_getMdixMode(void* pAppCtxt_p, void* pStackCtxt_p)
{
    uint16_t   phyRegVal   = 0;
    uint32_t   mdixMode    = 0;

    OSALUNREF_PARM(pAppCtxt_p);

    CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_PHYCR_REG, &phyRegVal);

    phyRegVal = phyRegVal & (CUST_PHY_DP83869_PHYCR_REG_MDI_MANUAL_CROSSOVER | CUST_PHY_DP83869_PHYCR_REG_MDI_AUTO_CROSSOVER);
    phyRegVal = phyRegVal >> 5;

    switch(phyRegVal)
    {
        case 0:
            mdixMode = CUST_PHY_MDI_MANUAL_CONFIG;
            break;
        case 1:
            mdixMode = CUST_PHY_MDIX_MANUAL_CONFIG;
            break;
        default:
            mdixMode = CUST_PHY_MDIX_AUTO_CROSSOVER;
            break;
    }

    return mdixMode;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Disables 1Gbit Advertisment during autonegotiation.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p          application context
 *  \param[in]  pStackCtxt_p        stack context
 *
 *  <!-- Group: -->
 *
 *  \ingroup CUST_PHY
 *
 * */
void CUST_PHY_DP83869_disable1GbAdver(void* pAppCtxt_p, void* pStackCtxt_p)
{
    uint16_t phyRegVal = 0;

    OSALUNREF_PARM(pAppCtxt_p);

    OSAL_printf("Phy %lu : Disable GBit ANEG\r\n", CUST_PHY_getPhyAddr(pStackCtxt_p));
    CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_CFG1_REG, &phyRegVal);

    phyRegVal &= ~CUST_PHY_DP83869_CFG1_REG_1000BT_FDX;
    phyRegVal &= ~CUST_PHY_DP83869_CFG1_REG_1000BT_HDX;

    CUST_PHY_writeReg(pStackCtxt_p, CUST_PHY_DP83869_CFG1_REG, phyRegVal);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Enables low latency in RGMII mode
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p          application context
 *  \param[in]  pStackCtxt_p        stack context
 *
 *  <!-- Group: -->
 *
 *  \ingroup CUST_PHY
 *
 * */
void CUST_PHY_DP83869_rgmiiLowLatencyEnable(void* pAppCtxt_p, void* pStackCtxt_p)
{
    uint16_t phyRegVal = 0;

    OSALUNREF_PARM(pAppCtxt_p);

    OSAL_printf("Phy %lu : RGMII enable low latency\r\n", CUST_PHY_getPhyAddr(pStackCtxt_p));
    CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_EXT_RGMIICTL2_REG, &phyRegVal);

    phyRegVal |= CUST_PHY_DP83869_LOW_LATENCY_10_100_ENABLE;

    CUST_PHY_writeReg(pStackCtxt_p, CUST_PHY_DP83869_EXT_RGMIICTL2_REG, phyRegVal);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  RGMII_TX_HALF_FULL_THR[1:0] can be changed from the default of 0x2 to 0x1.
 *    This will reduce the latency by one clock period.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p          application context
 *  \param[in]  pStackCtxt_p        stack context
 *  \param[in]  threshold_p         Number of cycles for threshold (2, 1)
 *
 *  <!-- Group: -->
 *
 *  \ingroup CUST_PHY
 *
 * */
void CUST_PHY_DP83869_rgmiiTxHalfFullThreshold(void* pAppCtxt_p, void* pStackCtxt_p, uint32_t threshold_p)
{
    uint16_t phyRegVal = 0;
    uint16_t val = threshold_p;

    OSAL_printf("Phy %lu : RGMII set TX Half/Full Threshold: %d\r\n", CUST_PHY_getPhyAddr(pStackCtxt_p), threshold_p);
    CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_EXT_RGMIICTL_REG, &phyRegVal);

    /*Only Bit[1:0] of val will be used, as this is a 2 bit  field in register*/
    val &= 0x3;
    /* Set val in Bits[4:3] */
    phyRegVal |=  (val << CUST_PHY_DP83869_RGMII_TX_HALF_FULL_THR_SHIFT);

    CUST_PHY_writeReg(pStackCtxt_p, CUST_PHY_DP83869_EXT_RGMIICTL_REG, phyRegVal);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  RGMII_RX_HALF_FULL_THR[1:0] can be changed from the default of 0x2 to 0x1.
 *    This will reduce the latency by one clock period.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p          application context
 *  \param[in]  pStackCtxt_p        stack context
 *  \param[in]  threshold_p         Number of cycles for threshold (2, 1)
 *
 *  <!-- Group: -->
 *
 *  \ingroup CUST_PHY
 *
 * */
void CUST_PHY_DP83869_rgmiiRxHalfFullThreshold(void* pAppCtxt_p, void* pStackCtxt_p, uint32_t threshold_p)
{
    uint16_t phyRegVal = 0;
    uint16_t val = threshold_p;

    OSAL_printf("Phy %lu : RGMII set RX Half/Full Threshold: %d\r\n", CUST_PHY_getPhyAddr(pStackCtxt_p), threshold_p);
    CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_EXT_RGMIICTL_REG, &phyRegVal);

    /*Only Bit[1:0] of val will be used, as this is a 2 bit  field in register*/
    val &= 0x3;
    /* Set val in Bits[4:3] */
    phyRegVal |=  (val << CUST_PHY_DP83869_RGMII_RX_HALF_FULL_THR_SHIFT);

    CUST_PHY_writeReg(pStackCtxt_p, CUST_PHY_DP83869_EXT_RGMIICTL_REG, phyRegVal);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Provides actually used link speed and duplex mode.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p          application context
 *  \param[in]  pStackCtxt_p        stack context
 *  \param[in]  pData_p             pointer to data structure which contains link speed and duplex mode values
 *  \param[in]  dataSize_p          size of data structure
 *
 *  <!-- Group: -->
 *
 *  \ingroup CUST_PHY
 *
 * */
void CUST_PHY_DP83869_getSpeedDuplex (void* pAppCtxt_p, void* pStackCtxt_p, void* pData_p, uint32_t dataSize_p)
{
    CUST_PHY_SSpeedDuplexConfig_t *pSpeedDuplexCfg = NULL;

    uint16_t  phyRegVal    = 0;

    OSALUNREF_PARM(pAppCtxt_p);

    pSpeedDuplexCfg = (CUST_PHY_SSpeedDuplexConfig_t*) pData_p;

    if (pSpeedDuplexCfg == NULL)
    {
        return;
    }

    CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_PHYSTS_REG, &phyRegVal);

    switch(phyRegVal & CUST_PHY_DP83869_PHYSTS_REG_SPEED)
    {
        case CUST_PHY_DP83869_PHYSTS_SPEED_10M:
            if (phyRegVal & CUST_PHY_DP83869_PHYSTS_REG_DUPLEX)
            {
                pSpeedDuplexCfg->config = CUST_PHY_SPEED_DUPLEX_eCONFIG_10FD;
            }
            else
            {
                pSpeedDuplexCfg->config = CUST_PHY_SPEED_DUPLEX_eCONFIG_10HD;
            }
            break;
        case CUST_PHY_DP83869_PHYSTS_SPEED_100M:
            if (phyRegVal & CUST_PHY_DP83869_PHYSTS_REG_DUPLEX)
            {
                pSpeedDuplexCfg->config = CUST_PHY_SPEED_DUPLEX_eCONFIG_100FD;
            }
            else
            {
                pSpeedDuplexCfg->config = CUST_PHY_SPEED_DUPLEX_eCONFIG_100HD;
            }
            break;
        case CUST_PHY_DP83869_PHYSTS_SPEED_1000M:
            if (phyRegVal & CUST_PHY_DP83869_PHYSTS_REG_DUPLEX)
            {
                pSpeedDuplexCfg->config = CUST_PHY_SPEED_DUPLEX_eCONFIG_1000FD;
            }
            else
            {
                pSpeedDuplexCfg->config = CUST_PHY_SPEED_DUPLEX_eCONFIG_1000HD;
            }
            break;
        default:
            pSpeedDuplexCfg->config = CUST_PHY_SPEED_DUPLEX_eCONFIG_INVALID;
            break;
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Open PHY interface function to be able connect to TI ETHPHY interface.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p                   application context
 *  \param[in]  pStackCtxt_p                 stack context
 *  \param[in]  pParam_p                     parameters
 *  \return     uint32_t                     status
 *  \retval     CUST_PHY_eSTATUS_FAIL        function failed
 *  \retval     CUST_PHY_eSTATUS_SUCCESS     function success
 *
 *  <!-- Group: -->
 *
 *  \ingroup CUST_PHY
 *
 * */
uint32_t CUST_PHY_DP83869_openFxn (void *pAppCtxt_p, void *pStackCtxt_p, void* pParam_p)
{
    OSALUNREF_PARM(pAppCtxt_p);
    OSALUNREF_PARM(pParam_p);

    return CUST_PHY_open(pStackCtxt_p);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Command PHY interface function to be able connect to TI ETHPHY interface.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p                   application context
 *  \param[in]  pStackCtxt_p                 stack context
 *  \param[in]  command_p                    command id
 *  \param[in]  pData_p                      data
 *  \param[in]  dataSize_p                   data size
 *  \return     uint32_t                     status
 *  \retval     CUST_PHY_eSTATUS_FAIL        function failed
 *  \retval     CUST_PHY_eSTATUS_SUCCESS     function success
 *
 *  <!-- Group: -->
 *
 *  \ingroup PRU
 *
 * */
uint32_t CUST_PHY_DP83869_commandFxn (void *pAppCtxt_p, void *pStackCtxt_p, uint32_t command_p, void* pData_p, uint32_t dataSize_p)
{
    int32_t status = CUST_PHY_eSTATUS_SUCCESS;

    switch(command_p)
    {
        case CUST_PHY_eCOMMAND_GET_SPEED_AND_DUPLEX_CONFIG:
            CUST_PHY_DP83869_getSpeedDuplex(pAppCtxt_p, pStackCtxt_p, pData_p, dataSize_p);
            break;
        default:
            status = CUST_PHY_eSTATUS_FAIL;
            break;
    }

    return status;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Close PHY interface function to be able connect to TI ETHPHY interface.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pAppCtxt_p              application context
 *  \param[in]  pStackCtxt_p            stack context
 *
 *  <!-- Group: -->
 *
 *  \ingroup PRU
 *
 * */
void CUST_PHY_DP83869_closeFxn (void *pAppCtxt_p, void *pStackCtxt_p)
{
    OSALUNREF_PARM(pAppCtxt_p);
    OSALUNREF_PARM(pStackCtxt_p);

    return;
}

static uint16_t CUST_PHY_DP83869_readExtendedRegister(void *pAppCtxt_p, void *pStackCtxt_p, uint16_t extAddress_p)
{
    uint16_t    registerValue   = 0;

    /* 1. Write the value 0x001f (DEVAD=31) to REGCR */
    CUST_PHY_writeReg(pStackCtxt_p, CUST_PHY_DP83869_REGCR_REG, CUST_PHY_DP83869_REGCR_ADDRESS_ACCESS);

    /* 2. Write the desired register address to register ADDAR */
    CUST_PHY_writeReg(pStackCtxt_p, CUST_PHY_DP83869_ADDAR_REG, extAddress_p);

    /* 3. Write the value 0x401f (data no post increment function) to REGCR */
    CUST_PHY_writeReg(pStackCtxt_p, CUST_PHY_DP83869_REGCR_REG, CUST_PHY_DP83869_DATA_NORMAL_ACCESS);

    /* 4. Read the content of the desired extended register set from ADDAR */
    CUST_PHY_readReg(pStackCtxt_p, CUST_PHY_DP83869_ADDAR_REG, &registerValue);

    return registerValue;
}

static void CUST_PHY_DP83869_writeExtendedRegister(void *pAppCtxt_p, void *pStackCtxt_p, uint16_t extAddress_p, uint16_t value_p)
{
    /* 1. Write the value 0x001f (DEVAD=31) to REGCR */
    CUST_PHY_writeReg(pStackCtxt_p, CUST_PHY_DP83869_REGCR_REG, CUST_PHY_DP83869_REGCR_ADDRESS_ACCESS);

    /* 2. Write the desired register address to register ADDAR */
    CUST_PHY_writeReg(pStackCtxt_p, CUST_PHY_DP83869_ADDAR_REG, extAddress_p);

    /* 3. Write the value 0x401f (data no post increment function) to REGCR */
    CUST_PHY_writeReg(pStackCtxt_p, CUST_PHY_DP83869_REGCR_REG, CUST_PHY_DP83869_DATA_NORMAL_ACCESS);

    /* 4. Read the content of the desired extended register set from ADDAR */
    CUST_PHY_writeReg(pStackCtxt_p, CUST_PHY_DP83869_ADDAR_REG, value_p);
}

//*************************************************************************************************
