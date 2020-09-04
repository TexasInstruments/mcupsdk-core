/**
 * @file csl_serdes3.h
 *
 * @brief
 *  Header file for functional layer of CSL SERDES.
 *
 *  It contains the various enumerations, structure definitions and function
 *  declarations
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2022, Texas Instruments, Inc.
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
/** ============================================================================
 *
 * @defgroup CSL_SERDES
 *
 * @section Introduction
 *
 * @subsection xxx Overview
 * This is the top level SERDES API with enumerations for various supported
   reference clocks, link rates, lane control rates across different modules.
 *
 *
 * @subsection References
 *
 * ============================================================================
 */
#ifndef CSL_SERDES_V1_H_
#define CSL_SERDES_V1_H_
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <string.h>
#include <drivers/soc.h>
#include <drivers/hw_include/cslr.h>

#define CSL_SERDES_MAX_LANES                4
#define CSL_SERDES_MAX_LANES_TORRENT        4
#define CSL_SERDES_MAX_TAPS                 5
#define CSL_SERDES_MAX_COMPARATORS          5
#define CSL_SERDES_TBUS_SIZE                440

typedef uint32_t CSL_SerdesInstance;
#define CSL_TORRENT_SERDES0                 (0U)

typedef uint32_t CSL_SerdesRefClock;
#define CSL_SERDES_REF_CLOCK_19p2M          (0U)
#define CSL_SERDES_REF_CLOCK_20M            (1U)
#define CSL_SERDES_REF_CLOCK_24M            (2U)
#define CSL_SERDES_REF_CLOCK_25M            (3U)
#define CSL_SERDES_REF_CLOCK_26M            (4U)
#define CSL_SERDES_REF_CLOCK_27M            (5U)
#define CSL_SERDES_REF_CLOCK_60M            (6U)
#define CSL_SERDES_REF_CLOCK_100M           (7U)
#define CSL_SERDES_REF_CLOCK_122p88M        (8U)
#define CSL_SERDES_REF_CLOCK_125M           (9U)
#define CSL_SERDES_REF_CLOCK_153p6M         (10U)
#define CSL_SERDES_REF_CLOCK_156p25M        (11U)
#define CSL_SERDES_REF_CLOCK_312p5M         (12U)

typedef uint32_t CSL_SerdesRefClockSrc;
#define CSL_SERDES_REF_CLOCK_INT            (0U)
#define CSL_SERDES_REF_CLOCK_EXT_NO_SSC     (1U)
#define CSL_SERDES_REF_CLOCK_EXT_SSC        (2U)


typedef uint32_t CSL_SerdesSSCMode;
#define CSL_SERDES_NO_SSC                   (0U)
#define CSL_SERDES_INTERNAL_SSC             (1U)
#define CSL_SERDES_EXTERNAL_SSC             (2U)

typedef uint32_t CSL_SerdesMultilink;
#define CSL_SERDES_XAUI_SGMII_MULTILINK     (0U)
#define CSL_SERDES_XAUI_QSGMII_MULTILINK    (1U)
#define CSL_SERDES_QSGMII_SGMII_MULTILINK   (2U)
#define CSL_SERDES_QSGMII_USB_MULTILINK     (3U)
#define CSL_SERDES_PCIe_USB_MULTILINK       (4U)
#define CSL_SERDES_PCIe_XAUI_MULTILINK      (5U)
#define CSL_SERDES_PCIe_SGMII_MULTILINK     (6U)
#define CSL_SERDES_PCIe_QSGMII_MULTILINK    (7U)

typedef uint32_t CSL_SerdesLinkRate;
#define CSL_SERDES_LINK_RATE_1p25G          (0U)
#define CSL_SERDES_LINK_RATE_2p7G           (1U)
#define CSL_SERDES_LINK_RATE_3p125G         (2U)
#define CSL_SERDES_LINK_RATE_4p9152G        (3U)
#define CSL_SERDES_LINK_RATE_5G             (4U)
#define CSL_SERDES_LINK_RATE_5p15625G       (5U)
#define CSL_SERDES_LINK_RATE_5p4G           (6U)
#define CSL_SERDES_LINK_RATE_6p144G         (7U)
#define CSL_SERDES_LINK_RATE_6p25G          (8U)
#define CSL_SERDES_LINK_RATE_7p3728G        (9U)
#define CSL_SERDES_LINK_RATE_8G             (10U)
#define CSL_SERDES_LINK_RATE_8p1G           (11U)
#define CSL_SERDES_LINK_RATE_9p8304G        (12U)
#define CSL_SERDES_LINK_RATE_10G            (13U)
#define CSL_SERDES_LINK_RATE_10p3125G       (14U)
#define CSL_SERDES_LINK_RATE_12p5G          (15U)
#define CSL_SERDES_LINK_RATE_16G            (16U)

typedef uint32_t CSL_SerdesLoopback;
#define CSL_SERDES_LOOPBACK_DISABLED        (0U)
#define CSL_SERDES_LOOPBACK_LINE            (1U)
#define CSL_SERDES_LOOPBACK_SER             (2U)
#define CSL_SERDES_LOOPBACK_NEPAR           (3U)
#define CSL_SERDES_LOOPBACK_FEPAR           (4U)
#define CSL_SERDES_LOOPBACK_RECOVEREDCLOCK  (5U)
#define CSL_SERDES_LOOPBACK_TXONLY          (6U)
#define CSL_SERDES_LOOPBACK_PCS             (7U)
#define CSL_SERDES_LOOPBACK_ISI             (8U)

typedef uint32_t CSL_SerdesStatus;
#define CSL_SERDES_STATUS_PLL_NOT_LOCKED    (0U)
#define CSL_SERDES_STATUS_PLL_LOCKED        (1U)

typedef uint32_t CSL_SerdesPIPEStatus;
#define CSL_SERDES_STATUS_PIPE_CLK_VALID      (0U)
#define CSL_SERDES_STATUS_PIPE_CLK_NOT_VALID  (1U)

typedef uint32_t CSL_SerdesResult;
#define CSL_SERDES_NO_ERR                   (0U)
#define CSL_SERDES_INVALID_REF_CLOCK        (1U)
#define CSL_SERDES_INVALID_LANE_RATE        (2U)
#define CSL_SERDES_INVALID_NUM_LANES        (3U)
#define CSL_SERDES_INVALID_PHY_TYPE         (4U)
#define CSL_SERDES_INVALID_MULTILINK        (5U)

typedef uint32_t CSL_SerdesLaneCtrlRate;
#define CSL_SERDES_LANE_FULL_RATE           (0U)
#define CSL_SERDES_LANE_HALF_RATE           (1U)
#define CSL_SERDES_LANE_QUARTER_RATE        (2U)

typedef uint32_t CSL_SerdesLaneEnableStatus;
#define CSL_SERDES_LANE_ENABLE_NO_ERR                      (0U)
#define CSL_SERDES_LANE_ENABLE_INVALID_RATE                (1U)
#define CSL_SERDES_LANE_ENABLE_PERIPHERAL_BASE_NOT_SET     (2U)
#define CSL_SERDES_LANE_ENABLE_ITERATION_MODE_NOT_SET      (3U)
#define CSL_SERDES_LANE_ENABLE_SIG_UNDETECTED              (4U)

typedef uint32_t CSL_SerdesPhyType;
#define CSL_SERDES_PHY_TYPE_PCIe            (0U)
#define CSL_SERDES_PHY_TYPE_SGMII           (1U)
#define CSL_SERDES_PHY_TYPE_SGMII_ICSSG     (2U)
#define CSL_SERDES_PHY_TYPE_QSGMII          (3U)
#define CSL_SERDES_PHY_TYPE_USB             (4U)
#define CSL_SERDES_PHY_TYPE_eDP             (5U)
#define CSL_SERDES_PHY_TYPE_XFI             (6U)
#define CSL_SERDES_PHY_TYPE_XAUI            (7U)
#define CSL_SERDES_PHY_TYPE_USXGMII         (8U)

typedef uint32_t CSL_SerdesPCIeGenType;
#define CSL_SERDES_PCIE_GEN1                (0U)
#define CSL_SERDES_PCIE_GEN2                (1U)
#define CSL_SERDES_PCIE_GEN3                (2U)
#define CSL_SERDES_PCIE_GEN4                (3U)

typedef struct CSL_SerdesTxCoeff
{
    uint32_t txAtt[CSL_SERDES_MAX_LANES];
    uint32_t txVreg[CSL_SERDES_MAX_LANES];
    uint32_t cmCoeff[CSL_SERDES_MAX_LANES];
    uint32_t c1Coeff[CSL_SERDES_MAX_LANES];
    uint32_t c2Coeff[CSL_SERDES_MAX_LANES];
}CSL_SerdesTxCoeff;

typedef struct CSL_SerdesRxCoeff
{
    int32_t rxAtt[CSL_SERDES_MAX_LANES];
    int32_t rxBoost[CSL_SERDES_MAX_LANES];
    int32_t attStart[CSL_SERDES_MAX_LANES];
    int32_t boostStart[CSL_SERDES_MAX_LANES];
    int32_t forceAttVal[CSL_SERDES_MAX_LANES];
    int32_t forceBoostVal[CSL_SERDES_MAX_LANES];
}CSL_SerdesRxCoeff;

typedef struct CSL_SerdesTapOffsets
{
    uint32_t   tap1Offsets[CSL_SERDES_MAX_LANES][CSL_SERDES_MAX_TAPS];
    uint32_t   tap2Offsets[CSL_SERDES_MAX_LANES][CSL_SERDES_MAX_TAPS];
    uint32_t   tap3Offsets[CSL_SERDES_MAX_LANES][CSL_SERDES_MAX_TAPS];
    uint32_t   tap4Offsets[CSL_SERDES_MAX_LANES][CSL_SERDES_MAX_TAPS];
    uint32_t   tap5Offsets[CSL_SERDES_MAX_LANES][CSL_SERDES_MAX_TAPS];
    uint32_t   cmpOffsets[CSL_SERDES_MAX_LANES][CSL_SERDES_MAX_COMPARATORS];
    uint32_t   iZeroCmp[CSL_SERDES_MAX_LANES][CSL_SERDES_MAX_COMPARATORS];
    uint32_t   iZeroTap1[CSL_SERDES_MAX_LANES][CSL_SERDES_MAX_TAPS];
    uint32_t   iZeroTap2[CSL_SERDES_MAX_LANES][CSL_SERDES_MAX_TAPS];
    uint32_t   iZeroTap3[CSL_SERDES_MAX_LANES][CSL_SERDES_MAX_TAPS];
    uint32_t   iZeroTap4[CSL_SERDES_MAX_LANES][CSL_SERDES_MAX_TAPS];
    uint32_t   iZeroTap5[CSL_SERDES_MAX_LANES][CSL_SERDES_MAX_TAPS];
    uint32_t   tap1OffsetsTmp[CSL_SERDES_MAX_LANES][CSL_SERDES_MAX_TAPS];
    uint32_t   tap2OffsetsTmp[CSL_SERDES_MAX_LANES][CSL_SERDES_MAX_TAPS];
    uint32_t   tap3OffsetsTmp[CSL_SERDES_MAX_LANES][CSL_SERDES_MAX_TAPS];
    uint32_t   tap4OffsetsTmp[CSL_SERDES_MAX_LANES][CSL_SERDES_MAX_TAPS];
    uint32_t   tap5OffsetsTmp[CSL_SERDES_MAX_LANES][CSL_SERDES_MAX_TAPS];
} CSL_SerdesTapOffsets;

typedef struct CSL_SerdesTbusDump
{
    uint32_t    taddr[CSL_SERDES_TBUS_SIZE];
    uint32_t    tbusData[CSL_SERDES_TBUS_SIZE];
} CSL_SerdesTbusDump;

typedef uint32_t CSL_SerdesForceAttBoost;
#define CSL_SERDES_FORCE_ATT_BOOST_DISABLED (0U)
#define CSL_SERDES_FORCE_ATT_BOOST_ENABLED  (1U)

typedef uint32_t CSL_SerdesLaneEnableIterationMode;
#define CSL_SERDES_LANE_ENABLE_COMMON_INIT          (1U)
#define CSL_SERDES_LANE_ENABLE_LANE_INIT            (2U)
#define CSL_SERDES_LANE_ENABLE_LANE_INIT_NO_WAIT    (3U)

typedef uint32_t CSL_SerdesOperatingMode;
#define CSL_SERDES_FUNCTIONAL_MODE          (0U)
#define CSL_SERDES_FUNCTIONAL_MODE_QT       (1U)
#define CSL_SERDES_FUNCTIONAL_MODE_FAST_SIM (2U)
#define CSL_SERDES_DIAGNOSTIC_MODE          (3U)
#define CSL_SERDES_DIAGNOSTIC_MODE_QT       (4U)
#define CSL_SERDES_DIAGNOSTIC_MODE_FAST_SIM (5U)
#define CSL_SERDES_FUNCTIONAL_MODE_TB		(6U)

typedef uint32_t CSL_SerdesEnableRefClkOut;
#define CSL_SERDES_REFCLK_OUT_DIS           (0U)
#define CSL_SERDES_REFCLK_OUT_EN            (1U)

typedef uint32_t CSL_SerdesInvertTXPolarity;
#define CSL_SERDES_INV_TX_POLARITY_DIS         (0U)
#define CSL_SERDES_INV_TX_POLARITY_EN          (1U)

typedef uint32_t CSL_SerdesInvertRXPolarity;
#define CSL_SERDES_INV_RX_POLARITY_DIS         (0U)
#define CSL_SERDES_INV_RX_POLARITY_EN          (1U)

typedef struct CSL_SerdesTxTerm
{
    uint32_t txTermP;
    uint32_t txTermN;
}CSL_SerdesTxTerm;

typedef struct CSL_SerdesLaneEnableParams
{
    uint32_t                          baseAddr;
    uint32_t                          mainCtrlMMRbaseAddr;
    uint32_t                          numLanes;
    uint32_t                          numPLLs;
    uint8_t                           laneMask;
    uint8_t                           pllMask;
    uint32_t                          peripheralBaseAddr;
    uint32_t                          phyInstanceNum;
    CSL_SerdesInstance                serdesInstance;
    CSL_SerdesSSCMode                 SSC_mode;
    CSL_SerdesRefClock                refClock;
    CSL_SerdesLinkRate                linkRate;
    CSL_SerdesTxCoeff                 txCoeff;
    CSL_SerdesRxCoeff                 rxCoeff;
    CSL_SerdesForceAttBoost           forceAttBoost;
    CSL_SerdesPhyType                 phyType;
    CSL_SerdesPCIeGenType             pcieGenType;
    CSL_SerdesLaneCtrlRate            laneCtrlRate[CSL_SERDES_MAX_LANES];
    CSL_SerdesLoopback                loopbackMode[CSL_SERDES_MAX_LANES];
    CSL_SerdesOperatingMode           operatingMode;
    CSL_SerdesLaneEnableIterationMode iterationMode;
    CSL_SerdesRefClockSrc             refClkSrc;
    CSL_SerdesEnableRefClkOut         refClkOut;
    CSL_SerdesInvertTXPolarity        invertTXPolarity[CSL_SERDES_MAX_LANES];
    CSL_SerdesInvertRXPolarity        invertRXPolarity[CSL_SERDES_MAX_LANES];
}CSL_SerdesLaneEnableParams;

extern void CSL_serdesCycleDelay (uint64_t count);


extern void CSL_serdesDisablePllAndLanes(uint32_t   baseAddr,
                                         uint32_t   numLanes,
                                         uint8_t    laneMask);

extern void CSL_serdesInvertLaneTXPolarity(uint32_t baseAddr,
                                  uint32_t laneNum);

extern void CSL_serdesInvertLaneRXPolarity(uint32_t baseAddr,
                                  uint32_t laneNum);

extern void CSL_serdesPORResetDefault(uint32_t            baseAddr);

extern void CSL_serdesDisablePLL(uint32_t            baseAddr,
                                        CSL_SerdesPhyType   phyType);

extern void CSL_serdesDisableLanes(uint32_t     baseAddr,
                                   uint32_t     laneNum,
                                   uint8_t      laneMask);

extern void CSL_serdesEnableLanes(uint32_t baseAddr,
                                  uint32_t laneNum,
                                  CSL_SerdesPhyType   phyType,
                                  CSL_SerdesLinkRate  linkRate,
								  CSL_SerdesOperatingMode opMode);

extern void CSL_serdesFastSimEnable(uint32_t baseAddr);

extern void CSL_serdesSetCMUWaitVal(uint32_t baseAddr,
		                           uint32_t val);

extern void CSL_serdesSetPhanVal(uint32_t baseAddr);

extern void CSL_serdesWrite32Mask(uint32_t baseAddr,
                              uint32_t maskVal,
                              uint32_t setVal);

extern void CSL_serdesSetLoopback
(
 uint32_t baseAddr,
 uint32_t laneNum,
 CSL_SerdesLoopback loopbackMode,
 CSL_SerdesInstance serdesInstance,
 CSL_SerdesPhyType phyType
);

extern void CSL_serdesReleaseReset
(
 uint32_t               baseAddr
);

extern CSL_SerdesStatus CSL_serdesGetPLLStatus
(
 uint32_t baseAddr,
 uint32_t laneMask,
 CSL_SerdesInstance serdesInstance
);

extern CSL_SerdesStatus CSL_serdesGetSigDetStatus
(
 uint32_t baseAddr,
 uint32_t numLanes,
 uint8_t laneMask,
 CSL_SerdesPhyType phyType
);

extern CSL_SerdesStatus CSL_serdesGetLaneStatus
(
 uint32_t baseAddr,
 uint32_t numLanes,
 uint8_t laneMask,
 CSL_SerdesPhyType phyType
);

extern CSL_SerdesStatus CSL_serdesConfigStatus
(
 uint32_t baseAddr
);

extern CSL_SerdesLaneEnableStatus CSL_serdesLaneEnable
(
         CSL_SerdesLaneEnableParams *serdesLaneEnableParams
);

extern CSL_SerdesResult CSL_serdesRefclkSel(uint32_t mainCtrlMMRbaseAddr,
                                            uint32_t baseAddr,
                                            CSL_SerdesRefClock refClk,
                                            CSL_SerdesRefClockSrc refClkSrc,
                                            CSL_SerdesInstance serdesInstance,
                                            CSL_SerdesPhyType   phyType);
extern CSL_SerdesResult CSL_serdesIPSelect(uint32_t mainCtrlMMRbaseAddr,
                                           CSL_SerdesPhyType phyType,
                                           uint32_t phyInstanceNum,
                                           CSL_SerdesInstance serdesInstance,
                                           uint32_t serdeslaneNum);
extern void CSL_serdesPCIeModeSelect(uint32_t baseAddr,
                                     CSL_SerdesPCIeGenType  pcieGenType,
                                     uint32_t laneNum);

extern void CSL_serdesOutClkEn(uint32_t baseAddr,
                               CSL_SerdesRefClock refClock,
                               CSL_SerdesPhyType   phyType);
extern CSL_SerdesPIPEStatus CSL_serdesGetPIPEClkStatus(uint32_t baseAddr,
                                                       uint8_t laneMask,
                                                       CSL_SerdesPhyType phyType);
extern void CSL_serdesPorReset(uint32_t baseAddr);

#ifdef __cplusplus
}
#endif
#endif
/* @} */
