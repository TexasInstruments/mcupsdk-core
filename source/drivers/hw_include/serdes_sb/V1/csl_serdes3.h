/*
 * Copyright (C) 2024 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

#define CSL_SERDES_MAX_LANES                (1)
#define CSL_SERDES_MAX_TAPS                 (5)
#define CSL_SERDES_MAX_COMPARATORS          (5)
#define CSL_SERDES_TBUS_SIZE                (440)

typedef enum
{
   CSL_SERDES_REF_CLOCK_20M         =   0,
   CSL_SERDES_REF_CLOCK_60M         =   1,
   CSL_SERDES_REF_CLOCK_100M        =   2,
   CSL_SERDES_REF_CLOCK_122p88M     =   3,
   CSL_SERDES_REF_CLOCK_125M        =   4,
   CSL_SERDES_REF_CLOCK_153p6M     =    5,
   CSL_SERDES_REF_CLOCK_156p25M     =   6,
   CSL_SERDES_REF_CLOCK_312p5M      =   7
} CSL_SerdesRefClock;

typedef enum
{
   CSL_SERDES_LINK_RATE_1p25G        =   0,
   CSL_SERDES_LINK_RATE_3p125G       =   1,
   CSL_SERDES_LINK_RATE_4p9152G      =   2,
   CSL_SERDES_LINK_RATE_5G           =   3,
   CSL_SERDES_LINK_RATE_6p144G       =   4,
   CSL_SERDES_LINK_RATE_6p25G        =   5,
   CSL_SERDES_LINK_RATE_7p3728G      =   6,
   CSL_SERDES_LINK_RATE_8G           =   7,
   CSL_SERDES_LINK_RATE_9p8304G      =   8,
   CSL_SERDES_LINK_RATE_10G          =   9,
   CSL_SERDES_LINK_RATE_10p3125G     =   10,
   CSL_SERDES_LINK_RATE_12p5G        =   11
} CSL_SerdesLinkRate;

typedef enum
{
    CSL_SERDES_LOOPBACK_DISABLED      =  0,
    CSL_SERDES_LOOPBACK_TX_TO_BERT    =  1,
    CSL_SERDES_LOOPBACK_NES           =  2,
    CSL_SERDES_LOOPBACK_FES           =  3,
    CSL_SERDES_LOOPBACK_FEP           =  4,
    CSL_SERDES_LOOPBACK_FEP1p5        =  5,
    CSL_SERDES_LOOPBACK_PIPE          =  6,
    CSL_SERDES_LOOPBACK_ENCODER       =  7,
    CSL_SERDES_LOOPBACK_BOARD2BOARD   =  8
} CSL_SerdesLoopback;

typedef enum
{
    CSL_SERDES_STATUS_PLL_NOT_LOCKED = 0,
    CSL_SERDES_STATUS_PLL_LOCKED     = 1
} CSL_SerdesStatus;

typedef enum
{
    CSL_SERDES_NO_ERR               = 0,
    CSL_SERDES_INVALID_REF_CLOCK    = 1,
    CSL_SERDES_INVALID_LANE_RATE    = 2,
    CSL_SERDES_INVALID_NUM_LANES    = 3
} CSL_SerdesResult;

typedef enum
{
    CSL_SERDES_LANE_FULL_RATE      = 0,
    CSL_SERDES_LANE_HALF_RATE      = 1,
    CSL_SERDES_LANE_QUARTER_RATE   = 2
} CSL_SerdesLaneCtrlRate;

typedef enum
{
    CSL_SERDES_LANE_ENABLE_NO_ERR                     = 0,
    CSL_SERDES_LANE_ENABLE_INVALID_RATE               = 1,
    CSL_SERDES_LANE_ENABLE_PERIPHERAL_BASE_NOT_SET    = 2,
    CSL_SERDES_LANE_ENABLE_ITERATION_MODE_NOT_SET     = 3,
    CSL_SERDES_LANE_ENABLE_SIG_UNDETECTED             = 4
} CSL_SerdesLaneEnableStatus;

typedef enum
{
    CSL_SERDES_PHY_TYPE_10GE        = 0,
    CSL_SERDES_PHY_TYPE_AIF2_B8     = 1,
    CSL_SERDES_PHY_TYPE_AIF2_B4     = 2,
    CSL_SERDES_PHY_TYPE_SRIO        = 3,
    CSL_SERDES_PHY_TYPE_PCIe        = 4,
    CSL_SERDES_PHY_TYPE_HYPERLINK   = 5,
    CSL_SERDES_PHY_TYPE_SGMII       = 6,
    CSL_SERDES_PHY_TYPE_DFE         = 7,
    CSL_SERDES_PHY_TYPE_IQN         = 8,
    CSL_SERDES_PHY_TYPE_USB         = 9
} CSL_SerdesPhyType;

typedef struct CSL_SerdesTXCoeff
{
    uint32_t txAtt[CSL_SERDES_MAX_LANES];
    uint32_t txVreg[CSL_SERDES_MAX_LANES];
    uint32_t cmCoeff[CSL_SERDES_MAX_LANES];
    uint32_t c1Coeff[CSL_SERDES_MAX_LANES];
    uint32_t c2Coeff[CSL_SERDES_MAX_LANES];
}CSL_SerdesTXCoeff;

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


typedef enum
{
    CSL_SERDES_FORCE_ATT_BOOST_DISABLED  =  0,
    CSL_SERDES_FORCE_ATT_BOOST_ENABLED   =  1
} CSL_SerdesForceAttBoost;

typedef enum
{
    CSL_SERDES_LANE_ENABLE_COMMON_INIT  = 1,
    CSL_SERDES_LANE_ENABLE_LANE_INIT   = 2,
    CSL_SERDES_LANE_ENABLE_LANE_INIT_NO_WAIT   = 3
} CSL_SerdesLaneEnableIterationMode;

typedef enum
{
    CSL_SERDES_FUNCTIONAL_MODE           =  0,
    CSL_SERDES_FUNCTIONAL_MODE_QT        =  1,
    CSL_SERDES_FUNCTIONAL_MODE_FAST_SIM  =  2,
    CSL_SERDES_DIAGNOSTIC_MODE           =  3,
    CSL_SERDES_DIAGNOSTIC_MODE_QT        =  4,
    CSL_SERDES_DIAGNOSTIC_MODE_FAST_SIM  =  5
} CSL_SerdesOperatingMode;

typedef enum
{
    CSL_SERDES_SSC_DISABLED  =  0,
    CSL_SERDES_SSC_ENABLED   =  1
} CSL_SerdesSSCMode;

typedef struct CSL_SerdesTxTerm
{
    uint32_t txTermP;
    uint32_t txTermN;
}CSL_SerdesTxTerm;

typedef struct CSL_SerdesLaneEnableParams
{
    uint32_t                          baseAddr;
    uint32_t                          numLanes;
    uint8_t                           laneMask;
    uint32_t                          polarity[CSL_SERDES_MAX_LANES];
    uint32_t                          peripheralBaseAddr;
    CSL_SerdesRefClock                refClock;
    CSL_SerdesLinkRate                linkRate;
    CSL_SerdesTXCoeff                 txCoeff;
    CSL_SerdesRxCoeff                 rxCoeff;
    CSL_SerdesForceAttBoost           forceAttBoost;
    CSL_SerdesPhyType                 phyType;
    CSL_SerdesLaneCtrlRate            laneCtrlRate[CSL_SERDES_MAX_LANES];
    CSL_SerdesLoopback                loopbackMode[CSL_SERDES_MAX_LANES];
    CSL_SerdesOperatingMode           operatingMode;
    CSL_SerdesLaneEnableIterationMode iterationMode;
    CSL_SerdesSSCMode                 sscMode;
}CSL_SerdesLaneEnableParams;

#define CSL_serdesComRXEQMap(baseAddr, offset)        (baseAddr + 0x1400 + offset)
#define CSL_serdesLaneMap(baseAddr, laneNum, offset)  ((baseAddr) + 0x200*(laneNum + 1) + offset)

void CSL_serdesCycleDelay
(
uint64_t count
);

void CSL_serdesDisablePllAndLanes
(
uint32_t baseAddr,
uint32_t numLanes
);

void CSL_serdesPORResetDefault
(
uint32_t baseAddr
);

void CSL_serdesDisablePLL
(
uint32_t baseAddr,
CSL_SerdesPhyType   phyType
);

void CSL_serdesDisableLanes
(
uint32_t baseAddr,
uint32_t laneNum
);

void CSL_serdesEnableLanes
(
uint32_t baseAddr,
uint32_t laneNum
);

void CSL_serdesWaitForSigDet
(
uint32_t baseAddr,
uint32_t laneNum
);

void CSL_serdesFastSimEnable
(
uint32_t baseAddr
);

void CSL_serdesSetCMUWaitVal
(
uint32_t baseAddr,
uint32_t val
);

void CSL_serdesSetPhanVal
(
uint32_t baseAddr
);

void CSL_serdesWrite32Mask
(
uint32_t baseAddr,
uint32_t maskVal,
uint32_t setVal
);

void CSL_serdesRxEqDisable
(
uint32_t baseAddr
);

void CSL_serdesWaitForCMUOK
(
uint32_t baseAddr
);


CSL_SerdesLaneEnableStatus CSL_serdesSetLaneRate
(
uint32_t               baseAddr,
uint32_t               laneNum,
CSL_SerdesLaneCtrlRate laneCtrlRate,
CSL_SerdesLinkRate     linkrate,
CSL_SerdesPhyType      phyType
);

void CSL_serdesSetLoopback
(
uint32_t baseAddr,
uint32_t laneNum,
CSL_SerdesLoopback loopbackMode,
CSL_SerdesPhyType phyType
);

void CSL_serdesPllEnable
(
uint32_t               baseAddr,
CSL_SerdesPhyType      phyType,
CSL_SerdesLinkRate     link_rate
);

CSL_SerdesStatus CSL_serdesGetPLLStatus
(
uint32_t baseAddr,
CSL_SerdesPhyType phyType
);

CSL_SerdesStatus CSL_serdesGetSigDetStatus
(
uint32_t baseAddr,
uint32_t numLanes,
uint8_t laneMask,
CSL_SerdesPhyType phyType
);

CSL_SerdesStatus CSL_serdesGetLaneStatus
(
uint32_t baseAddr,
uint32_t numLanes,
uint8_t laneMask,
CSL_SerdesPhyType phyType
);

CSL_SerdesLaneEnableStatus CSL_serdesLaneEnable
(
CSL_SerdesLaneEnableParams *serdesLaneEnableParams
);

void CSL_serdesPorReset
(
uint32_t baseAddr
);

void CSL_serdesForceSigDetEn
(uint32_t baseAddr,
 uint32_t laneNum
 );

void CSL_serdesForceSigDetLow
(
uint32_t baseAddr,
uint32_t laneNum
);

void CSL_serdesForceSigDetHigh
(
uint32_t baseAddr,
uint32_t laneNum
);

void CSL_serdesWriteTbusAddr
(uint32_t baseAddr,
int32_t    iSelect,
int32_t    iOffset
);

uint32_t CSL_serdesReadTbusVal
(
uint32_t baseAddr
);

uint32_t CSL_serdesReadSelectedTbus
(
uint32_t baseAddr,
int32_t     iSelect,
int32_t     iOffset
);

void CSL_serdesDisablePllAndLanes
(
uint32_t baseAddr,
uint32_t numLanes
);

void CSL_SerdesInvertLanePolarity
(
uint32_t baseAddr,
uint32_t laneNum
);

void CSL_serdesAssertReset
(
uint32_t baseAddr,
CSL_SerdesPhyType phyType,
uint8_t           laneMask
);

void CSL_serdesTbusLaneDump
(
uint32_t baseAddr,
CSL_SerdesTbusDump *pTbusDump
);

void CSL_serdesTbusCMUDump
(
uint32_t baseAddr,
CSL_SerdesTbusDump *pTbusDump
);

void CSL_SerdesConfigCMC1C2
(
uint32_t base_addr,
uint32_t lane_num,
uint32_t CMcoeff,
uint32_t C1coeff,
uint32_t C2coeff,
CSL_SerdesPhyType phy_type
);

void CSL_serdesDeassertReset
(
uint32_t baseAddr,
CSL_SerdesPhyType phyType,
uint8_t laneMask
);

void CSL_serdesResetSigDet
(
uint32_t baseAddr,
uint32_t laneNum
 );

void CSL_serdesForceRxRecalEn
(
uint32_t baseAddr,
uint32_t laneNum
);

void CSL_serdesForceRxRecalLow
(
uint32_t baseAddr,
uint32_t laneNum
);

void CSL_serdesResetRxCal
(
uint32_t baseAddr,
uint32_t laneNum
);

void CSL_serdesResetReceiver
(
uint32_t baseAddr,
uint32_t laneNum
);

void CSL_serdesUSBLaneCtrlOvr
(
uint32_t baseAddr,
uint32_t laneMask
);

void CSL_serdesSetWaitAfter
(
uint32_t baseAddr
);

void CSL_serdesClearWaitAfter
(
uint32_t baseAddr
);

#ifdef __cplusplus
}
#endif
#endif
/* @} */

