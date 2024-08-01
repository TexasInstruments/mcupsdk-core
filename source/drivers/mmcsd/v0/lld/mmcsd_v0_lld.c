/*
 * Copyright (C) 2024 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
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
 */

/**
 *  \file   mmcsd_v0_lld.c
 *
 *  \brief  File containing MMCSD LLD Driver APIs implementation for V0.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <drivers/mmcsd/v0/lld/mmcsd_lld.h>
#include <kernel/dpl/CacheP.h>
#include <drivers/mmcsd/v0/lld/internal/mmcsd_parse.h>
#include <drivers/soc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief MACROS used to select one of the Transfer Types. */
#define MMCSD_CMD_XFER_TYPE_WRITE                       ((uint32_t) 0U)
#define MMCSD_CMD_XFER_TYPE_READ                        ((uint32_t) 1U)

/** \brief MACROS used to select one of the possible Bus Voltages. */
#define MMCSD_BUS_VOLT_1_8V                             ((uint32_t) 0x5U)
#define MMCSD_BUS_VOLT_3_0V                             ((uint32_t) 0x6U)
#define MMCSD_BUS_VOLT_3_3V                             ((uint32_t) 0x7U)

/** \brief MACROS used to select one of the possible UHS Modes internally. */
#define MMCSD_UHS_MODE_SDR12                            ((uint32_t) 0U)
#define MMCSD_UHS_MODE_SDR25                            ((uint32_t) 1U)
#define MMCSD_UHS_MODE_SDR50                            ((uint32_t) 2U)
#define MMCSD_UHS_MODE_SDR104                           ((uint32_t) 3U)
#define MMCSD_UHS_MODE_DDR50                            ((uint32_t) 4U)

/** \brief Macros used for selecting one of the possible tapdelay value. */
#define MMCSD_PHY_MODE_HS400                            ((uint32_t) 1U)
#define MMCSD_PHY_MODE_HS200                            ((uint32_t) 2U)
#define MMCSD_PHY_MODE_HSSDR50                          ((uint32_t) 3U)
#define MMCSD_PHY_MODE_HSDDR50                          ((uint32_t) 4U)
#define MMCSD_PHY_MODE_ENHANCED_STROBE                  ((uint32_t) 5U)
#define MMCSD_PHY_MODE_SDR104                           ((uint32_t) 6U)
#define MMCSD_PHY_MODE_SDR50                            ((uint32_t) 7U)
#define MMCSD_PHY_MODE_DDR50                            ((uint32_t) 8U)
#define MMCSD_PHY_MODE_SDR25                            ((uint32_t) 9U)
#define MMCSD_PHY_MODE_SDR12                            ((uint32_t) 10U)
#define MMCSD_PHY_MODE_HS                               ((uint32_t) 11U)
#define MMCSD_PHY_MODE_DS                               ((uint32_t) 12U)

/** \brief Interrupt Masks */
#define MMCSD_INTERRUPT_ALL_NORMAL                      ((uint16_t) 0xFFFFU)
#define MMCSD_INTERRUPT_ALL_ERROR                       ((uint16_t) 0x17FFU)

/** \brief Default Constants used by the Driver during initialization */
#define MMCSD_PHY_CTRL_3_REG_DEFAULT                    ((uint32_t) 0x10FF10FFU)
#define MMCSD_ID_MODE_FREQUENCY_HZ                      ((uint32_t) 400000U)
#define MMCSD_MMC_DEFAULT_RCA                           ((uint32_t) 0x02U)

/** \brief Macros used for CMD6 Argument formation */
#define MMCSD_CMD6_ACCESS_MASK_CMD_SET                  ((uint32_t) 0x00U)
#define MMCSD_CMD6_ACCESS_MASK_SET_BITS                 ((uint32_t) 0x01U)
#define MMCSD_CMD6_ACCESS_MASK_CLEAR_BITS               ((uint32_t) 0x02U)
#define MMCSD_CMD6_ACCESS_MASK_WRITE_BYTE               ((uint32_t) 0x03U)

/** \brief Macros related to setting operating voltage for SD devices */
#define MMCSD_SD_CMD8_CHECK_PATTERN                     ((uint32_t) 0xAAU)
#define MMCSD_SD_VOLT_2P7_3P6                           ((uint32_t) 0x01U)
#define MMCSD_SD_VOLT_LOW_RANGE                         ((uint32_t) 0x02U)

/** \brief Card status value (Bits 9-12) as defined in JESD84-B51 6.13 section
 * & physical layer specification section 4.10.1.
 */
#define MMCSD_MEDIA_STATE_IDLE                          ((uint32_t) 0U)
#define MMCSD_MEDIA_STATE_READY                         ((uint32_t) 1U)
#define MMCSD_MEDIA_STATE_IDENT                         ((uint32_t) 2U)
#define MMCSD_MEDIA_STATE_STBY                          ((uint32_t) 3U)
#define MMCSD_MEDIA_STATE_TRAN                          ((uint32_t) 4U)
#define MMCSD_MEDIA_STATE_DATA                          ((uint32_t) 5U)
#define MMCSD_MEDIA_STATE_RCV                           ((uint32_t) 6U)
#define MMCSD_MEDIA_STATE_PRG                           ((uint32_t) 7U)
#define MMCSD_MEDIA_STATE_DIS                           ((uint32_t) 8U)
#define MMCSD_MEDIA_STATE_BTST                          ((uint32_t) 9U)
#define MMCSD_MEDIA_STATE_SLP                           ((uint32_t) 10U)

/** \brief This is the timeout value for sending CMD13 to the card.
 * After every write, the CMD13 is sent this many times and wait for
 * the card to go to transfer state
 * */
#define MMCSD_MEDIA_STATE_THRESHOLD                     ((uint32_t) 30000U)

/** \brief Macros used for checking device Type of eMMC Device */
/*-------------------------------------------------------------------
Extended CSD register, DEVICE_TYPE [196]
---     -------------------------------------------------------------
Bit     Device Type
---     -------------------------------------------------------------
7       HS400 Dual Data Rate e•MMC at 200 MHz – 1.2 V I/O
6       HS400 Dual Data Rate e•MMC at 200 MHz – 1.8 V I/O
5       HS200 Single Data Rate e•MMC at 200 MHz - 1.2 V I/O
4       HS200 Single Data Rate e•MMC at 200 MHz - 1.8 V I/O
3       High-Speed Dual Data Rate e•MMC at 52 MHz - 1.2 V I/O
2       High-Speed Dual Data Rate e•MMC at 52 MHz - 1.8 V or 3 V I/O
1       High-Speed e•MMC at 52 MHz - at rated device voltage(s)
0       High-Speed e•MMC at 26 MHz - at rated device voltage(s)
-------------------------------------------------------------------*/
#define MMCSD_EMMC_ECSD_DEVICE_TYPE_HS400_200MHZ_1P2V_MASK  ((uint8_t) 0x80U)
#define MMCSD_EMMC_ECSD_DEVICE_TYPE_HS400_200MHZ_1P8V_MASK  ((uint8_t) 0x40U)
#define MMCSD_EMMC_ECSD_DEVICE_TYPE_HS200_200MHZ_1P2V_MASK  ((uint8_t) 0x20U)
#define MMCSD_EMMC_ECSD_DEVICE_TYPE_HS200_200MHZ_1P8V_MASK  ((uint8_t) 0x10U)
#define MMCSD_EMMC_ECSD_DEVICE_TYPE_HS_DDR_52MHZ_1P2V_MASK  ((uint8_t) 0x08U)
#define MMCSD_EMMC_ECSD_DEVICE_TYPE_HS_DDR_52MHZ_1P8V_MASK  ((uint8_t) 0x04U)
#define MMCSD_EMMC_ECSD_DEVICE_TYPE_HS_52MHZ_MASK           ((uint8_t) 0x02U)
#define MMCSD_EMMC_ECSD_DEVICE_TYPE_HS_26MHZ_MASK           ((uint8_t) 0x01U)
#define MMCSD_EMMC_ECSD_DEVICE_TYPE_DS_MASK                 ((uint8_t) 0x00U)
/*-----------------------------------------------------------------*/

/** \brief Macros used for selecting Drive Impedance */
#define MMCSD_DRIVE_IMPEDANCE_50OHMS                    (0x00U)
#define MMCSD_DRIVE_IMPEDANCE_33OHMS                    (0x01U)
#define MMCSD_DRIVE_IMPEDANCE_66OHMS                    (0x02U)
#define MMCSD_DRIVE_IMPEDANCE_100OHMS                   (0x03U)
#define MMCSD_DRIVE_IMPEDANCE_40OHMS                    (0x04U)

/** \brief Possible HS Timing Values (7.4.65 HS_TIMING [185])
 *  JEDEC Standard No. 84-B51 Page 202
 */
#define MMCSD_ECSD_HS_TIMING_BACKWARD_COMPATIBLE        ((uint32_t) 0U)
#define MMCSD_ECSD_HS_TIMING_HIGH_SPEED                 ((uint32_t) 1U)
#define MMCSD_ECSD_HS_TIMING_HS200                      ((uint32_t) 2U)
#define MMCSD_ECSD_HS_TIMING_HS400                      ((uint32_t) 3U)

/** \brief Possible Bus Mode Value (7.4.67 BUS_WIDTH [183])
 *  JEDEC Standard No. 84-B51 Page 223
 */
#define MMCSD_ECSD_BUS_WIDTH_1BIT                       ((uint32_t) 0U)
#define MMCSD_ECSD_BUS_WIDTH_4BIT                       ((uint32_t) 1U)
#define MMCSD_ECSD_BUS_WIDTH_8BIT                       ((uint32_t) 2U)
#define MMCSD_ECSD_BUS_WIDTH_4BIT_DDR                   ((uint32_t) 5U)
#define MMCSD_ECSD_BUS_WIDTH_8BIT_DDR                   ((uint32_t) 6U)

/** \brief Macros for ECSD Register Indexes */
#define MMCSD_ECSD_BUS_WIDTH_INDEX                      ((uint32_t) 183U)
#define MMCSD_ECSD_HS_TIMING_INDEX                      ((uint32_t) 185U)

/** \brief Macros for States Used during the Initialization of SD Device */
#define MMCSD_SD_INIT_STATE_CONTROLLER_INIT                 ((uint32_t) 0U)
#define MMCSD_SD_INIT_STATE_CHECK_VOLTAGE               ((uint32_t) 1U)
#define MMCSD_SD_INIT_STATE_CHECK_SDIO                  ((uint32_t) 2U)
#define MMCSD_SD_INIT_STATE_CARD_IDENTIFICATION         ((uint32_t) 3U)
#define MMCSD_SD_INIT_STATE_VOLTAGE_SWITCH              ((uint32_t) 4U)
#define MMCSD_SD_INIT_STATE_GET_CID                     ((uint32_t) 5U)
#define MMCSD_SD_INIT_STATE_GET_RCA                     ((uint32_t) 6U)
#define MMCSD_SD_INIT_STATE_GET_CSD                     ((uint32_t) 7U)
#define MMCSD_SD_INIT_STATE_CHECK_LOCKED                ((uint32_t) 8U)
#define MMCSD_SD_INIT_STATE_UNLOCK                      ((uint32_t) 9U)
#define MMCSD_SD_INIT_STATE_GET_SCR                     ((uint32_t) 10U)
#define MMCSD_SD_INIT_STATE_BUS_WIDTH_SWITCH            ((uint32_t) 11U)
#define MMCSD_SD_INIT_STATE_SELECT_BUS_SPEED_MODE       ((uint32_t) 12U)
#define MMCSD_SD_INIT_STATE_DRIVE_STRENGTH              ((uint32_t) 13U)
#define MMCSD_SD_INIT_STATE_CURRENT_LIMIT               ((uint32_t) 14U)
#define MMCSD_SD_INIT_STATE_SWITCH_BUS_SPEED_MODE       ((uint32_t) 15U)
#define MMCSD_SD_INIT_STATE_TUNING                      ((uint32_t) 16U)

/** \brief Macros for States Used during the initialization of MMC Device */
#define MMCSD_MMC_INIT_STATE_CONTROLLER_INIT            ((uint32_t) 0U)
#define MMCSD_MMC_INIT_STATE_CHECK_VOLTAGE              ((uint32_t) 1U)
#define MMCSD_MMC_INIT_STATE_GET_CID                    ((uint32_t) 2U)
#define MMCSD_MMC_INIT_STATE_GET_CSD                    ((uint32_t) 3U)
#define MMCSD_MMC_INIT_STATE_VERSION_CHECK              ((uint32_t) 4U)
#define MMCSD_MMC_INIT_STATE_GET_ECSD                   ((uint32_t) 5U)
#define MMCSD_MMC_INIT_STATE_SELECT_BUS_SPEED           ((uint32_t) 6U)
#define MMCSD_MMC_INIT_STATE_BUS_WIDTH_SWITCH           ((uint32_t) 7U)
#define MMCSD_MMC_INIT_STATE_SWITCH_BUS_SPEED_MODE      ((uint32_t) 8U)

/** \brief Command argument to configure for switch mode. */
#define MMCSD_SWITCH_MODE                               ((uint32_t) 0x80FFFFFFU)
#define MMCSD_CHECK_MODE                                ((uint32_t) 0x00FFFFFFU)

/** \brief Command argument width to configure for transfer speed. */
#define MMCSD_CMD6_GRP1_SEL                             ((uint32_t) 0xFFFFFFF0U)
#define MMCSD_CMD6_GRP2_SEL                             ((uint32_t) 0xFFFFFF0FU)
#define MMCSD_CMD6_GRP3_SEL                             ((uint32_t) 0xFFFFF0FFU)
#define MMCSD_CMD6_GRP4_SEL                             ((uint32_t) 0xFFFF0FFFU)

/** \brief Frequencies for SD Card's Different Speed Modes */
#define MMCSD_SD_DS_FREQUENCY_HZ                        ((uint32_t) 25000000U)
#define MMCSD_SD_HS_FREQUENCY_HZ                        ((uint32_t) 50000000U)
#define MMCSD_SD_SDR12_FREQUENCY_HZ                     ((uint32_t) 25000000U)
#define MMCSD_SD_SDR25_FREQUENCY_HZ                     ((uint32_t) 50000000U)
#define MMCSD_SD_SDR50_FREQUENCY_HZ                     ((uint32_t) 100000000U)
#define MMCSD_SD_SDR104_FREQUENCY_HZ                    ((uint32_t) 200000000U)
#define MMCSD_SD_DDR50_FREQUENCY_HZ                     ((uint32_t) 50000000U)

/** \brief Different Drive Strength for SD Device */
#define MMCSD_CMD6_DRIVE_STRENGTH_TYPE_B                ((uint32_t) 0x0U)
#define MMCSD_CMD6_DRIVE_STRENGTH_TYPE_A                ((uint32_t) 0x1U)
#define MMCSD_CMD6_DRIVE_STRENGTH_TYPE_C                ((uint32_t) 0x2U)
#define MMCSD_CMD6_DRIVE_STRENGTH_TYPE_D                ((uint32_t) 0x3U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* Default MMCSD transaction parameters structure */
const MMCSDLLD_Transaction MMCSD_lld_defaultTransaction = {

    0,                                          /* Command */
    MMCSD_CMD_XFER_TYPE_WRITE,                  /* Direction */
    0,                                          /* Argument */
    (uint8_t*)NULL,                             /* Data Buffer */
    512U,                                       /* Block Size */
    1U,                                         /* Block Count */
    0U,                                         /* Auto Command Enable */
    0U,                                         /* DMA Enable */
    0U,                                         /* Is Tuning */
    {0,0,0,0}                                   /* Response */
};

typedef struct
{
    uint32_t dmaParams;
    uint32_t addrLo;
    uint32_t addrHi;

} MMCSD_ADMA2Descriptor;

/* ========================================================================== */
/*                     Internal Function Declarations                         */
/* ========================================================================== */

/* CSL like functions for Host Controller Registers */
static int32_t MMCSD_softReset(uint32_t ctrlBaseAddr, uint32_t loopTimeout);
static int32_t MMCSD_linesResetCmd(uint32_t ctrlBaseAddr, uint32_t loopTimeout);
static int32_t MMCSD_linesResetDat(uint32_t ctrlBaseAddr, uint32_t loopTimeout);
static void MMCSD_setBusWidth(uint32_t ctrlBaseAddr, uint32_t busWidth);
static void MMCSD_selectDma(uint32_t ctrlBaseAddr, uint32_t dmaSel);
static void MMCSD_setBusVolt(uint32_t ctrlBaseAddr, uint32_t voltage);
static bool MMCSD_isCardInserted(uint32_t ctrlBaseAddr);
static int32_t MMCSD_busPowerOnCtrl(uint32_t ctrlBaseAddr, bool pwr,
                                 uint32_t loopTimeout);
static bool MMCSD_isIntClkStable(uint32_t ctrlBaseAddr, uint32_t loopTimeout);
static int32_t MMCSD_internalClockCtrl(uint32_t ctrlBaseAddr, bool clkState);
static int32_t MMCSD_pllClockCtrl(uint32_t ctrlBaseAddr, bool pllState);
static void MMCSD_sdClockCtrl(uint32_t ctrlBaseAddr, bool clkState);
static void MMCSD_setFreqSelSD(uint32_t ctrlBaseAddr, uint32_t clkDiv);
static int32_t MMCSD_setBusFreq(uint32_t ctrlBaseAddr, uint32_t inClk,
                                uint32_t outClk);
static int32_t MMCSD_setClockFreqSD(MMCSDLLD_Handle handle, uint32_t outClk);
static int32_t MMCSD_getCmdResponse(uint32_t ctrlBaseAddr, uint32_t *rsp);
static void MMCSD_setUHSMode(uint32_t ctrlBaseAddr, uint32_t uhsMode);
static void MMCSD_sendCommand(uint32_t ctrlBaseAddr,
                              MMCSDLLD_Transaction *trans);
static int32_t MMCSD_waitCommandInhibit(uint32_t ctrlBaseAddr,
                                        uint32_t loopTimeout);
static int32_t MMCSD_waitDataInhibit(uint32_t ctrlBaseAddr,
                                     uint32_t loopTimeout);
static void MMCSD_writeDataPort(uint32_t ctrlBaseAddr, uint32_t *dataP);
static void MMCSD_readDataPort(uint32_t ctrlBaseAddr, uint32_t *dataP);
static bool MMCSD_is3V3Supported(uint32_t ctrlBaseAddr);
static bool MMCSD_is3V0Supported(uint32_t ctrlBaseAddr);
static bool MMCSD_getDAT0(uint32_t ctrlBaseAddr);
static int32_t MMCSD_waitDAat0inBusy(uint32_t ctrlBaseAddr,
                                     uint32_t loopTimeout);
static void MMCSD_1P8VsignalCtrl(uint32_t ctrlBaseAddr, bool sigEnable);
static bool MMCSD_get1P8VsignalStat(uint32_t ctrlBaseAddr);
static void MMCSD_setExecuteTuning(uint32_t ctrlBaseAddr);
static void MMCSD_resetExecuteTuning(uint32_t ctrlBaseAddr);
static void MMCSD_setSamplingClkSelect(uint32_t ctrlBaseAddr);
static void MMCSD_resetSamplingClkSelect(uint32_t ctrlBaseAddr);
static uint16_t MMCSD_getExecuteTuning(uint32_t ctrlBaseAddr);
static uint16_t MMCSD_getSamplingClkSelect(uint32_t ctrlBaseAddr);
static void MMCSD_setVer40Enable(uint32_t ctrlBaseAddr);
static void MMCSD_setADMA2LenMode(uint32_t ctrlBaseAddr, bool bitLen26Mode);
static bool MMCSD_isUHSSpeedSupportedHOST(uint32_t ctrlBaseAddr,
                                          uint32_t speedMode);
static bool MMCSD_isUHSDriveStrengthSupportedHOST(uint32_t ctrlBaseAddr,
                                                  uint32_t driverStrength);
static void MMCSD_writeADMA2DescAddr(uint32_t ctrlBaseAddr, uint64_t desc);

/* CSL like functions for Host Controller Interrupt Registers */
static uint16_t MMCSD_getNormIntrStat(uint32_t ctrlBaseAddr, uint16_t intrFlag);
static void MMCSD_clearNormIntrStat(uint32_t ctrlBaseAddr, uint16_t intrFlag);
static uint16_t MMCSD_getErrIntrStat(uint32_t ctrlBaseAddr, uint16_t intrFlag);
static void MMCSD_clearErrIntrStat(uint32_t ctrlBaseAddr, uint16_t intrFlag);
static void MMCSD_enableNormIntrStat(uint32_t ctrlBaseAddr, uint16_t intrFlag);
static void MMCSD_disableNormIntrStat(uint32_t ctrlBaseAddr, uint16_t intrFlag);
static void MMCSD_enableErrIntrStat(uint32_t ctrlBaseAddr, uint16_t intrFlag);
static void MMCSD_disableErrIntrStat(uint32_t ctrlBaseAddr, uint16_t intrFlag);
static void MMCSD_enableNormSigIntr(uint32_t ctrlBaseAddr, uint16_t intrFlag);
static void MMCSD_disableNormSigIntr(uint32_t ctrlBaseAddr, uint16_t intrFlag);
static void MMCSD_enableErrSigIntr(uint32_t ctrlBaseAddr, uint16_t intrFlag);
static void MMCSD_disableErrSigIntr(uint32_t ctrlBaseAddr, uint16_t intrFlag);

/* CSL like NON PHY functions for Subsystem Registers */
static void MMCSD_setSlotType(uint32_t ssBaseAddr, uint8_t slotType);

/*  CSL like functions for Subsystem PHY Registers */
static void MMCSD_phySetDriveImpedance(uint32_t ssBaseAddr,
                                       uint32_t drvImpedance);
static void MMCSD_phyEnableDLL(uint32_t ssBaseAddr);
static void MMCSD_phyDisableDLL(uint32_t ssBaseAddr);
static void MMCSD_phySetDLLTrimICP(uint32_t ssBaseAddr, uint32_t dllTrimICP);
static int32_t MMCSD_phyCalibIO(uint32_t ssBaseAddr, uint32_t loopTimeout);
static void MMCSD_phySetSTRBSEL(uint32_t ssBaseAddr, uint32_t strobeSel);
static void MMCSD_phyDisableTapChgWIN(uint32_t ssBaseAddr);
static void MMCSD_phyEnableTapChgWIN(uint32_t ssBaseAddr);
static void MMCSD_phySetOutTapDelay(uint32_t ssBaseAddr, uint32_t dlySel,
                                    uint32_t dlyEn);
static void MMCSD_phySetInTapDelay(uint32_t ssBaseAddr, uint32_t dlySel,
                                   uint32_t dlyEn);
static void MMCSD_phySetFrqSelDLL(uint32_t ssBaseAddr, uint32_t clkFreq);
static int32_t MMCSD_phyWaitDLLReady(uint32_t ssBaseAddr, uint32_t loopTimeout);
static void MMCSD_phyWriteCTRL2Reg(uint32_t ssBaseAddr, uint32_t regVal);
static void MMCSD_phyWriteCTRL3Reg(uint32_t ssBaseAddr, uint32_t regVal);

/* NON CSL supporting Functions */
static void MMCSD_phyGetTapDelay(uint32_t *oTapDelayEn, uint32_t *oTapDelaySel,
                                 uint32_t *iTapDelayEn, uint32_t *iTapDelaySel,
                                 uint32_t phyMode);
static int32_t MMCSD_phyConfigure(uint32_t ssBaseAddr, uint32_t phyMode,
                                  uint32_t phyClkFreq, uint32_t driveImpedance);
static int32_t MMCSD_phyInit(uint32_t ssBaseAddr, uint32_t phyType);

/* NON CSL PHY Tuning Functions */
static int32_t MMCSD_sendTuningDataMMC(MMCSDLLD_Handle handle);
static int32_t MMCSD_sendTuningDataSD(MMCSDLLD_Handle handle);
static int32_t MMCSD_phyTuneAuto(MMCSDLLD_Handle handle);
static int32_t MMCSD_phyTuneManual(MMCSDLLD_Handle handle);

/* API Assist internal Functions */
static int32_t MMCSD_lld_initSD(MMCSDLLD_Handle handle);
static int32_t MMCSD_lld_initMMC(MMCSDLLD_Handle handle);
static int32_t MMCSD_waitCardDetect(MMCSDLLD_Handle handle,
                                    uint32_t loopTimeout);
static void MMCSD_intrConfigReset(MMCSDLLD_Handle handle);
static void MMCSD_lld_initTransaction(MMCSDLLD_Transaction *trans);
static uint32_t MMCSD_lld_makeOCRMMC(bool busyBit, bool V_1V7_1V95,
                                  bool V_2V7_3V6, bool sectorMode);
static int32_t MMCSD_isCardReadyForTransferMMC(MMCSDLLD_Handle handle);
static int32_t MMCSD_isCardReadyForTransferSD(MMCSDLLD_Handle handle);
static int32_t MMCSD_switchBusSpeedEMMC(MMCSDLLD_Handle handle,
                                        uint32_t speedMode);
static bool MMCSD_isUHSSpeedSupportedSD(MMCSDLLD_Handle handle,
                                        uint32_t speedMode);
static bool MMCSD_isUHSDrvStrengthSupportedSD(MMCSDLLD_Handle handle,
                                              uint32_t driverStrength);
static int32_t MMCSD_switchBusSpeedSD(MMCSDLLD_Handle handle,
                                      uint32_t setSpeed);
static int32_t MMCSD_lld_sendCmd23SD(MMCSDLLD_Handle handle, uint32_t numBlks);
static int32_t MMCSD_lld_sendCmd23MMC(MMCSDLLD_Handle handle, uint32_t numBlks);
static bool MMCSD_isSpeedSupportedMMC(MMCSDLLD_Handle handle,
                                      uint32_t speedMode);
static void MMCSD_setupADMA2(MMCSDLLD_Handle handle,
                             MMCSD_ADMA2Descriptor *desc,
                             uint64_t bufAddr, uint32_t dataSize);

/* Polling mode transfer internal Functions */
static int32_t MMCSD_lld_transferPoll(MMCSDLLD_Handle handle,
                                      MMCSDLLD_Transaction *trans);
static int32_t MMCSD_lld_transferDataPoll(MMCSDLLD_Handle handle,
                                          MMCSDLLD_Transaction *trans);
static int32_t MMCSD_lld_transferNoDataPoll(MMCSDLLD_Handle handle,
                                            MMCSDLLD_Transaction *trans);
static void MMCSD_lld_cmdCompleteStatusPoll(MMCSDLLD_Handle handle);
static void MMCSD_lld_xferCompleteStatusPoll(MMCSDLLD_Handle handle);
static void MMCSD_lld_xferCompleteStatusPollCMD19(MMCSDLLD_Handle handle);

/* Interrupt mode transfer internal functions */
static int32_t MMCSD_lld_transferIntr(MMCSDLLD_Handle handle,
                                      MMCSDLLD_Transaction *trans);
static int32_t MMCSD_lld_transferDataIntr(MMCSDLLD_Handle handle,
                                          MMCSDLLD_Transaction *trans);
static int32_t MMCSD_lld_transferNoDataIntr(MMCSDLLD_Handle handle,
                                            MMCSDLLD_Transaction *trans);
static void MMCSD_lld_completeCurrTransfer(MMCSDLLD_Handle handle,
                                           int32_t xferStatus);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** \brief JESD84-B51 Figure 32 — Tuning block pattern for 8 bit mode */
static const uint8_t gTuningPattern8Bit[] __attribute__((aligned(128U))) = {

    0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00,
    0xff, 0xff, 0xcc, 0xcc, 0xcc, 0x33, 0xcc, 0xcc,
    0xcc, 0x33, 0x33, 0xcc, 0xcc, 0xcc, 0xff, 0xff,
    0xff, 0xee, 0xff, 0xff, 0xff, 0xee, 0xee, 0xff,
    0xff, 0xff, 0xdd, 0xff, 0xff, 0xff, 0xdd, 0xdd,
    0xff, 0xff, 0xff, 0xbb, 0xff, 0xff, 0xff, 0xbb,
    0xbb, 0xff, 0xff, 0xff, 0x77, 0xff, 0xff, 0xff,
    0x77, 0x77, 0xff, 0x77, 0xbb, 0xdd, 0xee, 0xff,
    0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00,
    0x00, 0xff, 0xff, 0xcc, 0xcc, 0xcc, 0x33, 0xcc,
    0xcc, 0xcc, 0x33, 0x33, 0xcc, 0xcc, 0xcc, 0xff,
    0xff, 0xff, 0xee, 0xff, 0xff, 0xff, 0xee, 0xee,
    0xff, 0xff, 0xff, 0xdd, 0xff, 0xff, 0xff, 0xdd,
    0xdd, 0xff, 0xff, 0xff, 0xbb, 0xff, 0xff, 0xff,
    0xbb, 0xbb, 0xff, 0xff, 0xff, 0x77, 0xff, 0xff,
    0xff, 0x77, 0x77, 0xff, 0x77, 0xbb, 0xdd, 0xee
};

/** \brief Physical Layer Simplified Specification Version 3.01,
 *  Table 4-2 Tuning Block Pattern */
static const uint8_t gTuningPattern4Bit[] __attribute__((aligned(128U))) = {

    0xff, 0x0f, 0xff, 0x00, 0xff, 0xcc, 0xc3, 0xcc,
    0xc3, 0x3c, 0xcc, 0xff, 0xfe, 0xff, 0xfe, 0xef,
    0xff, 0xdf, 0xff, 0xdd, 0xff, 0xfb, 0xff, 0xfb,
    0xbf, 0xff, 0x7f, 0xff, 0x77, 0xf7, 0xbd, 0xef,
    0xff, 0xf0, 0xff, 0xf0, 0x0f, 0xfc, 0xcc, 0x3c,
    0xcc, 0x33, 0xcc, 0xcf, 0xff, 0xef, 0xff, 0xee,
    0xff, 0xfd, 0xff, 0xfd, 0xdf, 0xff, 0xbf, 0xff,
    0xbb, 0xff, 0xf7, 0xff, 0xf7, 0x7f, 0x7b, 0xde,
};

/** \brief Global ADMA2 Descriptor */
MMCSD_ADMA2Descriptor gADMA2Desc;

/* ========================================================================== */
/*                       API Function Definitions                             */
/* ========================================================================== */

int32_t MMCSD_lld_init(MMCSDLLD_Handle handle)
{
    int32_t             status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object     *object = NULL;

    if(handle != NULL)
    {
        /* Get pointer to driver Object */
        object = (MMCSDLLD_Object *)handle;
        /* State should be reset before initialization */
        if(object->state != MMCSD_STATE_RESET)
        {
            status = MMCSD_STS_ERR;
        }
    }
    else
    {
        status = MMCSD_STS_ERR_INVALID_PARAM;
    }

    if(status == MMCSD_STS_SUCCESS)
    {
        /* Check if the MMCSD base address is valid or not */
        status = MMCSD_lld_isBaseAddrValid(object->initHandle->ctrlBaseAddr,
                                           object->initHandle->ssBaseAddr);
    }

    if(status == MMCSD_STS_SUCCESS)
    {
        /* Transfer State Idle */
        object->xferState = MMCSD_XFER_IDLE_STATE;
        /* Driver State to IDLE */
        object->state = MMCSD_STATE_IDLE;

        if(object->initHandle->cardType == MMCSD_CARD_TYPE_SD)
        {
            status = MMCSD_lld_initSD(handle);
        }
        else if(object->initHandle->cardType == MMCSD_CARD_TYPE_EMMC)
        {
            status = MMCSD_lld_initMMC(handle);
        }
        else if(object->initHandle->cardType == MMCSD_CARD_TYPE_NO_DEVICE)
        {
            /* Nothing to be initialized */
            status = MMCSD_STS_SUCCESS;
        }
        else
        {
            status = MMCSD_STS_ERR;
        }
    }

    if  (   (status == MMCSD_STS_SUCCESS) &&
            (object->initHandle->cardType != MMCSD_CARD_TYPE_NO_DEVICE))
    {
        /* Card initialization successful, Change the object state to IDLE */
        object->state = MMCSD_STATE_IDLE;
    }
    else if((status == MMCSD_STS_SUCCESS) &&
            (object->initHandle->cardType == MMCSD_CARD_TYPE_NO_DEVICE))
    {
        /* No Card to initialize, Change the object state to RESET */
        object->state = MMCSD_STATE_RESET;
    }
    else
    {
        /* Fail Condition, return status */
    }

    return status;
}

int32_t MMCSD_lld_deInit(MMCSDLLD_Handle handle)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object         *object = NULL;
    MMCSDLLD_Transaction    trans;

    if(handle != NULL)
    {
        /* Get pointer to driver Object */
        object = (MMCSDLLD_Object *)handle;
        /* State should be reset before de-initialization */
        if( (object->state != MMCSD_STATE_IDLE) &&
            (object->state != MMCSD_STATE_RESET) &&
            (object->initHandle->cardType != MMCSD_CARD_TYPE_NO_DEVICE))
        {
            status = MMCSD_STS_ERR_BUSY;
        }
        else if((object->initHandle->cardType != MMCSD_CARD_TYPE_NO_DEVICE) &&
                (   (object->state == MMCSD_STATE_IDLE) ||
                    (object->state == MMCSD_STATE_RESET)))
        {
            status = MMCSD_STS_SUCCESS;
        }
        else
        {
            status = MMCSD_STS_ERR;
        }
    }
    else
    {
        status = MMCSD_STS_ERR_INVALID_PARAM;
    }

    if(status == MMCSD_STS_SUCCESS)
    {
        if(object->initHandle->cardType == MMCSD_CARD_TYPE_SD)
        {
            /* Send CMD 0 (Reset SD) */
            MMCSD_lld_initTransaction(&trans);
            trans.cmd = MMCSD_SD_CMD(0U);
            status = MMCSD_lld_transferPoll(handle, &trans);

        }
        else if(object->initHandle->cardType == MMCSD_CARD_TYPE_EMMC)
        {
            MMCSD_lld_initTransaction(&trans);
            trans.cmd = MMCSD_MMC_CMD(0U);
            trans.arg = 0U;
            status = MMCSD_lld_transferPoll(handle, &trans);

            object->initHandle->Clock_uSleep(5000);
        }
        else
        {
            /* Nothing to be reset */
        }
    }

    if(status == MMCSD_STS_SUCCESS)
    {
        /* Soft Reset IP */
        status = MMCSD_softReset(object->initHandle->ctrlBaseAddr,
                                 MMCSD_WAIT_FOREVER);
    }

    if(status == MMCSD_STS_SUCCESS)
    {
        (void)MMCSD_busPowerOnCtrl(object->initHandle->ctrlBaseAddr, false,
                                   MMCSD_WAIT_FOREVER);
    }

    /* Change the object state to RESET */
    object->state = MMCSD_STATE_RESET;

    return status;
}

uint32_t MMCSD_lld_getBlockSize(MMCSDLLD_Handle handle)
{
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;
    uint32_t                blockSize = 0U;

    if(object->initHandle->cardType == MMCSD_CARD_TYPE_EMMC)
    {
        MMCSD_EmmcDeviceData *deviceData = (MMCSD_EmmcDeviceData *)NULL;
        deviceData = (MMCSD_EmmcDeviceData *)(object->initHandle->deviceData);
        blockSize = deviceData->maxWriteBlockLen;
    }
    else if(object->initHandle->cardType == MMCSD_CARD_TYPE_SD)
    {
        MMCSD_SdDeviceData *deviceData = (MMCSD_SdDeviceData *)NULL;
        deviceData = (MMCSD_SdDeviceData *)(object->initHandle->deviceData);
        blockSize = deviceData->maxWriteBlockLen;
    }
    else
    {
        /* No Code */
    }

    return blockSize;
}

int32_t MMCSD_lld_write_SD_Poll(MMCSDLLD_Handle handle, uint8_t *buf,
                                uint32_t startBlk, uint32_t numBlks)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;
    MMCSD_SdDeviceData      *deviceData = (MMCSD_SdDeviceData *)NULL;
    MMCSDLLD_Transaction    trans;
    uint32_t                addr = 0U;
    uint32_t                stopCmd = MMCSD_SD_CMD(12U);
    uint32_t                blockSize = MMCSD_lld_getBlockSize(handle);

    deviceData = (MMCSD_SdDeviceData *)(object->initHandle->deviceData);

    if(object->initHandle->cardType == MMCSD_CARD_TYPE_SD)
    {
        status = MMCSD_isCardReadyForTransferSD(handle);

        if((deviceData->isCmd23 == TRUE) && (status == MMCSD_STS_SUCCESS))
        {
            status = MMCSD_lld_sendCmd23SD(handle, numBlks);
        }
        else
        {
            status = MMCSD_STS_ERR;
        }
    }
    else
    {
        status = MMCSD_STS_ERR;
    }

    if(MMCSD_STS_SUCCESS == status)
    {
        if(deviceData->isHC)
        {
            /* Addressing in Blocks */
            addr = startBlk;
        }
        else
        {
            /* Addresing is in Bytes */
            addr = startBlk * blockSize;
        }

        MMCSD_lld_initTransaction(&trans);
        trans.arg = addr;
        trans.blockCount = numBlks;
        trans.blockSize = blockSize;
        trans.dataBuf = buf;
        trans.dir = MMCSD_CMD_XFER_TYPE_WRITE;

        if(numBlks > 1U)
        {
            trans.cmd = MMCSD_SD_CMD(25U);
        }
        else
        {
            trans.cmd = MMCSD_SD_CMD(24U);
        }

        status = MMCSD_lld_transferPoll(handle, &trans);
    }

    /* Through the SD Back into trans State */
    if((MMCSD_STS_SUCCESS == status) && (deviceData->isCmd23 != TRUE))
    {
        if(trans.blockCount > 1U)
        {
            MMCSD_lld_initTransaction(&trans);
            trans.cmd = stopCmd;
            trans.arg = 0U;
            status = MMCSD_lld_transferPoll(handle, &trans);
        }
    }

    return status;
}

int32_t MMCSD_lld_read_SD_Poll(MMCSDLLD_Handle handle, uint8_t *buf,
                               uint32_t startBlk, uint32_t numBlks)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;
    MMCSD_SdDeviceData      *deviceData = (MMCSD_SdDeviceData *)NULL;
    MMCSDLLD_Transaction    trans;
    uint32_t                addr = 0U;
    uint32_t                blockSize = MMCSD_lld_getBlockSize(handle);

    deviceData = (MMCSD_SdDeviceData *)(object->initHandle->deviceData);

    if(object->initHandle->cardType == MMCSD_CARD_TYPE_SD)
    {
        status = MMCSD_isCardReadyForTransferSD(handle);
    }
    else
    {
        status = MMCSD_STS_ERR;
    }

    if(deviceData->isCmd23 == TRUE)
    {
        status = MMCSD_lld_sendCmd23SD(handle, numBlks);
    }

    if(status == MMCSD_STS_SUCCESS)
    {
        if(deviceData->isHC)
        {
            addr = startBlk;
        }
        else
        {
            addr = startBlk * blockSize;
        }

        MMCSD_lld_initTransaction(&trans);
        trans.dir = MMCSD_CMD_XFER_TYPE_READ;
        trans.arg = addr;
        trans.blockCount = numBlks;
        trans.blockSize = blockSize;
        trans.dataBuf = buf;
        if(numBlks > 1U)
        {
            trans.cmd = MMCSD_SD_CMD(18U);
        }
        else
        {
            trans.cmd = MMCSD_SD_CMD(17U);
        }

        status = MMCSD_lld_transferPoll(handle, &trans);
    }

    return status;
}

int32_t MMCSD_lld_write_SD_Intr(MMCSDLLD_Handle handle, uint8_t *buf,
                                uint32_t startBlk, uint32_t numBlks)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;
    MMCSD_SdDeviceData      *deviceData = (MMCSD_SdDeviceData *)NULL;
    uint32_t                addr = 0U;
    uint32_t                blockSize = MMCSD_lld_getBlockSize(handle);

    deviceData = (MMCSD_SdDeviceData *)(object->initHandle->deviceData);

    if(object->initHandle->cardType == MMCSD_CARD_TYPE_SD)
    {
        status = MMCSD_isCardReadyForTransferSD(handle);
    }
    else
    {
        status = MMCSD_STS_ERR;
    }

    if(MMCSD_STS_SUCCESS == status)
    {
        if(deviceData->isHC)
        {
            /* Addressing in Blocks */
            addr = startBlk;
        }
        else
        {
            /* Addresing is in Bytes */
            addr = startBlk * blockSize;
        }

        MMCSD_lld_initTransaction(&object->mmcsdTxn);
        object->mmcsdTxn.arg = addr;
        object->mmcsdTxn.blockCount = numBlks;
        object->mmcsdTxn.blockSize = blockSize;
        object->mmcsdTxn.dataBuf = buf;
        object->mmcsdTxn.dir = MMCSD_CMD_XFER_TYPE_WRITE;

        if(numBlks > 1U)
        {
            object->mmcsdTxn.cmd = MMCSD_SD_CMD(25U);
        }
        else
        {
            object->mmcsdTxn.cmd = MMCSD_SD_CMD(24U);
        }

        status = MMCSD_lld_transferIntr(handle, &object->mmcsdTxn);
    }

    return status;
}

int32_t MMCSD_lld_read_SD_Intr(MMCSDLLD_Handle handle, uint8_t *buf,
                               uint32_t startBlk, uint32_t numBlks)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;
    MMCSD_SdDeviceData      *deviceData = (MMCSD_SdDeviceData *)NULL;
    uint32_t                addr = 0U;
    uint32_t                blockSize = MMCSD_lld_getBlockSize(handle);

    deviceData = (MMCSD_SdDeviceData *)(object->initHandle->deviceData);

    if(object->initHandle->cardType == MMCSD_CARD_TYPE_SD)
    {
        status = MMCSD_isCardReadyForTransferSD(handle);
    }
    else
    {
        status = MMCSD_STS_ERR;
    }

    if(deviceData->isCmd23 == TRUE)
    {
        status = MMCSD_lld_sendCmd23SD(handle, numBlks);
    }

    if(status == MMCSD_STS_SUCCESS)
    {
        if(deviceData->isHC)
        {
            addr = startBlk;
        }
        else
        {
            addr = startBlk * blockSize;
        }

        MMCSD_lld_initTransaction(&object->mmcsdTxn);
        object->mmcsdTxn.arg = addr;
        object->mmcsdTxn.blockCount = numBlks;
        object->mmcsdTxn.blockSize = blockSize;
        object->mmcsdTxn.dataBuf = buf;
        object->mmcsdTxn.dir = MMCSD_CMD_XFER_TYPE_READ;

        if(numBlks > 1U)
        {
            object->mmcsdTxn.cmd = MMCSD_SD_CMD(18U);
        }
        else
        {
            object->mmcsdTxn.cmd = MMCSD_SD_CMD(17U);
        }

        status = MMCSD_lld_transferIntr(handle, &object->mmcsdTxn);
    }

    return status;
}

int32_t MMCSD_lld_write_MMC_Poll(MMCSDLLD_Handle handle, uint8_t *buf,
                                 uint32_t startBlk, uint32_t numBlks)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;
    MMCSD_EmmcDeviceData    *deviceData = (MMCSD_EmmcDeviceData *)NULL;
    MMCSDLLD_Transaction    trans;
    uint32_t                addr = 0U;
    uint32_t                blockSize = MMCSD_lld_getBlockSize(handle);

    deviceData = (MMCSD_EmmcDeviceData *)(object->initHandle->deviceData);

    if(object->initHandle->cardType == MMCSD_CARD_TYPE_EMMC)
    {
        status = MMCSD_isCardReadyForTransferMMC(handle);

        if(status == MMCSD_STS_SUCCESS)
        {
            status = MMCSD_lld_sendCmd23MMC(handle, numBlks);
        }
    }
    else
    {
        status = MMCSD_STS_ERR;
    }

    if(MMCSD_STS_SUCCESS == status)
    {
        if(deviceData->isHC)
        {
            /* Addressing in Blocks */
            addr = startBlk;
        }
        else
        {
            /* Addresing is in Bytes */
            addr = startBlk * blockSize;
        }

        MMCSD_lld_initTransaction(&trans);
        trans.arg = addr;
        trans.blockCount = numBlks;
        trans.blockSize = blockSize;
        trans.dataBuf = buf;
        if(object->initHandle->enableDma)
        {
            trans.enableDma = TRUE;
        }
        trans.dir = MMCSD_CMD_XFER_TYPE_WRITE;

        if(numBlks > 1U)
        {
            trans.cmd = MMCSD_MMC_CMD(25U);
        }
        else
        {
            trans.cmd = MMCSD_MMC_CMD(24U);
        }

        status = MMCSD_lld_transferPoll(handle, &trans);
    }

    return status;
}

int32_t MMCSD_lld_read_MMC_Poll(MMCSDLLD_Handle handle, uint8_t *buf,
                                uint32_t startBlk, uint32_t numBlks)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;
    MMCSD_EmmcDeviceData    *deviceData = (MMCSD_EmmcDeviceData *)NULL;
    MMCSDLLD_Transaction    trans;
    uint32_t                addr = 0U;
    uint32_t                blockSize = MMCSD_lld_getBlockSize(handle);

    deviceData = (MMCSD_EmmcDeviceData *)(object->initHandle->deviceData);

    if(object->initHandle->cardType == MMCSD_CARD_TYPE_EMMC)
    {
        status = MMCSD_isCardReadyForTransferMMC(handle);

        if(status == MMCSD_STS_SUCCESS)
        {
            status = MMCSD_lld_sendCmd23MMC(handle, numBlks);
        }
    }
    else
    {
        status = MMCSD_STS_ERR;
    }

    if(status == MMCSD_STS_SUCCESS)
    {
        if(deviceData->isHC)
        {
            addr = startBlk;
        }
        else
        {
            addr = startBlk * blockSize;
        }

        MMCSD_lld_initTransaction(&trans);
        trans.dir = MMCSD_CMD_XFER_TYPE_READ;
        trans.arg = addr;
        trans.blockCount = numBlks;
        trans.blockSize = blockSize;
        trans.dataBuf = buf;
        if(numBlks > 1U)
        {
            trans.cmd = MMCSD_MMC_CMD(18U);
        }
        else
        {
            trans.cmd = MMCSD_MMC_CMD(17U);
        }

        status = MMCSD_lld_transferPoll(handle, &trans);
    }

    return status;
}

int32_t MMCSD_lld_write_MMC_Intr(MMCSDLLD_Handle handle, uint8_t *buf,
                                 uint32_t startBlk, uint32_t numBlks)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;
    MMCSD_EmmcDeviceData    *deviceData = (MMCSD_EmmcDeviceData *)NULL;
    uint32_t                addr = 0U;
    uint32_t                blockSize = MMCSD_lld_getBlockSize(handle);

    deviceData = (MMCSD_EmmcDeviceData *)(object->initHandle->deviceData);

    if(object->initHandle->cardType == MMCSD_CARD_TYPE_EMMC)
    {
        status = MMCSD_isCardReadyForTransferMMC(handle);

        if(status == MMCSD_STS_SUCCESS)
        {
            status = MMCSD_lld_sendCmd23MMC(handle, numBlks);
        }
    }
    else
    {
        status = MMCSD_STS_ERR;
    }

    if(MMCSD_STS_SUCCESS == status)
    {
        if(deviceData->isHC)
        {
            /* Addressing in Blocks */
            addr = startBlk;
        }
        else
        {
            /* Addresing is in Bytes */
            addr = startBlk * blockSize;
        }

        MMCSD_lld_initTransaction(&object->mmcsdTxn);
        object->mmcsdTxn.arg = addr;
        object->mmcsdTxn.blockCount = numBlks;
        object->mmcsdTxn.blockSize = blockSize;
        object->mmcsdTxn.dataBuf = buf;
        object->mmcsdTxn.dir = MMCSD_CMD_XFER_TYPE_WRITE;

        if(numBlks > 1U)
        {
            object->mmcsdTxn.cmd = MMCSD_MMC_CMD(25U);
        }
        else
        {
            object->mmcsdTxn.cmd = MMCSD_MMC_CMD(24U);
        }

        status = MMCSD_lld_transferIntr(handle, &object->mmcsdTxn);
    }

    return status;
}

int32_t MMCSD_lld_read_MMC_Intr(MMCSDLLD_Handle handle, uint8_t *buf,
                                uint32_t startBlk, uint32_t numBlks)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;
    MMCSD_EmmcDeviceData    *deviceData = (MMCSD_EmmcDeviceData *)NULL;
    uint32_t                addr = 0U;
    uint32_t                blockSize = MMCSD_lld_getBlockSize(handle);

    deviceData = (MMCSD_EmmcDeviceData *)(object->initHandle->deviceData);

    if(object->initHandle->cardType == MMCSD_CARD_TYPE_EMMC)
    {
        status = MMCSD_isCardReadyForTransferMMC(handle);

        if(status == MMCSD_STS_SUCCESS)
        {
            status = MMCSD_lld_sendCmd23MMC(handle, numBlks);
        }
    }
    else
    {
        status = MMCSD_STS_ERR;
    }

    if(status == MMCSD_STS_SUCCESS)
    {
        if(deviceData->isHC)
        {
            addr = startBlk;
        }
        else
        {
            addr = startBlk * blockSize;
        }

        MMCSD_lld_initTransaction(&object->mmcsdTxn);
        object->mmcsdTxn.dir = MMCSD_CMD_XFER_TYPE_READ;
        object->mmcsdTxn.arg = addr;
        object->mmcsdTxn.blockCount = numBlks;
        object->mmcsdTxn.blockSize = blockSize;
        object->mmcsdTxn.dataBuf = buf;
        if(numBlks > 1U)
        {
            object->mmcsdTxn.cmd = MMCSD_MMC_CMD(18U);
        }
        else
        {
            object->mmcsdTxn.cmd = MMCSD_MMC_CMD(17U);
        }

        status = MMCSD_lld_transferIntr(handle, &object->mmcsdTxn);
    }

    return status;
}

int32_t MMCSD_lld_change_Bus_Config_MMC(MMCSDLLD_Handle handle,
                                        uint32_t busSpeed, uint32_t busWidth)
{
    MMCSDLLD_Object *object = (MMCSDLLD_Object *)handle;
    int32_t status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Transaction    trans;
    /* Manually assign bus speed */
    uint32_t setSpeed = 0U;
    uint32_t ecsdBusWidth = 0U;
    uint32_t setBusWidth = 0U;

    if(MMCSD_isSpeedSupportedMMC(handle, busSpeed))
    {
        setSpeed = busSpeed;
    }
    else
    {
        setSpeed = MMCSD_MMC_MODE_SDR25;
    }

    object->setBusSpeed = setSpeed;

    switch(setSpeed)
    {
        case MMCSD_MMC_MODE_SDR25:
        case MMCSD_MMC_MODE_SDR50:
        {
            switch(busWidth)
            {
                case MMCSD_BUS_WIDTH_8BIT:
                {
                    ecsdBusWidth = MMCSD_ECSD_BUS_WIDTH_8BIT;
                    setBusWidth = MMCSD_BUS_WIDTH_8BIT;
                }
                break;
                case MMCSD_BUS_WIDTH_4BIT:
                {
                    ecsdBusWidth = MMCSD_ECSD_BUS_WIDTH_4BIT;
                    setBusWidth = MMCSD_BUS_WIDTH_4BIT;
                }
                break;
                case MMCSD_BUS_WIDTH_1BIT:
                {
                    ecsdBusWidth = MMCSD_ECSD_BUS_WIDTH_1BIT;
                    setBusWidth = MMCSD_BUS_WIDTH_1BIT;
                }
                break;
                default:
                {
                    ecsdBusWidth = MMCSD_ECSD_BUS_WIDTH_1BIT;
                    setBusWidth = MMCSD_BUS_WIDTH_1BIT;
                }
                break;
            }
        }
        break;

        case MMCSD_MMC_MODE_HS200:
        {
            switch(busWidth)
            {
                case MMCSD_BUS_WIDTH_8BIT:
                {
                    ecsdBusWidth = MMCSD_ECSD_BUS_WIDTH_8BIT;
                    setBusWidth = MMCSD_BUS_WIDTH_8BIT;
                }
                break;

                case MMCSD_BUS_WIDTH_4BIT:
                {
                    ecsdBusWidth = MMCSD_ECSD_BUS_WIDTH_4BIT;
                    setBusWidth = MMCSD_BUS_WIDTH_4BIT;
                }
                break;

                default:
                {
                    ecsdBusWidth = MMCSD_ECSD_BUS_WIDTH_4BIT;
                    setBusWidth = MMCSD_BUS_WIDTH_4BIT;
                }
                break;
            }
        }
        break;

        default:
        break;
    }

    object->setBusWidth = setBusWidth;

    /* Send CMD6(Switch Command) with argument to
        change buswidth of card (Write ECSD REG) */
    MMCSD_lld_initTransaction(&trans);
    trans.cmd = MMCSD_MMC_CMD(6U);
    trans.arg = (MMCSD_CMD6_ACCESS_MASK_WRITE_BYTE << 24U)   |
                (MMCSD_ECSD_BUS_WIDTH_INDEX << 16U)          |
                (((uint32_t)(0x00U) | (uint32_t)ecsdBusWidth) << 8U);

    status = MMCSD_lld_transferPoll(handle, &trans);

    if(status == MMCSD_STS_SUCCESS)
    {
        /* Change bus width of host Controller */
        MMCSD_setBusWidth(object->initHandle->ctrlBaseAddr,
                          setBusWidth);

        /* Check if card is ready for transfer with the new bus width */
        status = MMCSD_isCardReadyForTransferMMC(handle);
    }

    if(status == MMCSD_STS_SUCCESS)
    {
        status = MMCSD_switchBusSpeedEMMC(handle, setSpeed);
    }

    return status;
}

int32_t MMCSD_lld_change_Bus_Config_SD(MMCSDLLD_Handle handle,
                                       uint32_t busSpeed)
{
    int32_t status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object *object = (MMCSDLLD_Object *)handle;
    uint32_t speedModeUHS = 0U;
    uint32_t setSpeed = 0U;
    uint32_t setSpeedUser = busSpeed;
    bool tuningRequired = false;

    switch(busSpeed)
    {
        case MMCSD_SD_MODE_HS:
        case MMCSD_SD_MODE_SDR12:
        {
            speedModeUHS = MMCSD_UHS_MODE_SDR12;

        }
        break;

        case MMCSD_SD_MODE_DS:
        case MMCSD_SD_MODE_SDR25:
        {
            speedModeUHS = MMCSD_UHS_MODE_SDR25;
        }
        break;

        case MMCSD_SD_MODE_SDR50:
        {
            speedModeUHS = MMCSD_UHS_MODE_SDR50;
        }
        break;

        case MMCSD_SD_MODE_DDR50:
        {
            speedModeUHS = MMCSD_UHS_MODE_DDR50;
        }
        break;

        case MMCSD_SD_MODE_SDR104:
        {
            speedModeUHS = MMCSD_UHS_MODE_SDR104;
        }
        break;

        default:
        break;
    }

    if(MMCSD_isUHSSpeedSupportedHOST(object->initHandle->ctrlBaseAddr,
                                     speedModeUHS))
    {
        if(MMCSD_isUHSSpeedSupportedSD(handle, speedModeUHS))
        {
            setSpeed = speedModeUHS;
        }
        else
        {
            setSpeed = MMCSD_UHS_MODE_SDR12;
            setSpeedUser = MMCSD_SD_MODE_SDR12;
        }
    }
    else
    {
        setSpeed = MMCSD_UHS_MODE_SDR12;
        setSpeedUser = MMCSD_SD_MODE_SDR12;
    }

    status = MMCSD_switchBusSpeedSD(handle, setSpeed);

    if(status == MMCSD_STS_SUCCESS)
    {
        /* Set UHS Mode */
        if((busSpeed != MMCSD_SD_MODE_DS) && (busSpeed != MMCSD_SD_MODE_HS))
        {
            MMCSD_setUHSMode(object->initHandle->ctrlBaseAddr,
                             setSpeed);
        }
    }

    switch(setSpeed)
    {
        case MMCSD_UHS_MODE_SDR12:
        {
            status = MMCSD_setClockFreqSD(handle,
                                MMCSD_SD_SDR12_FREQUENCY_HZ);
        }
        break;

        case MMCSD_UHS_MODE_SDR25:
        {
            status = MMCSD_setClockFreqSD(handle,
                                MMCSD_SD_SDR25_FREQUENCY_HZ);
        }
        break;

        case MMCSD_UHS_MODE_SDR50:
        {
            status = MMCSD_setClockFreqSD(handle,
                                MMCSD_SD_SDR50_FREQUENCY_HZ);
            tuningRequired = true;
        }
        break;

        case MMCSD_UHS_MODE_SDR104:
        {
            status = MMCSD_setClockFreqSD(handle,
                                MMCSD_SD_SDR104_FREQUENCY_HZ);
            tuningRequired = true;
        }
        break;

        case MMCSD_UHS_MODE_DDR50:
        {
            status = MMCSD_setClockFreqSD(handle,
                                MMCSD_SD_DDR50_FREQUENCY_HZ);
        }
        break;

        default:
        {
            status = MMCSD_setClockFreqSD(handle,
                                MMCSD_SD_SDR12_FREQUENCY_HZ);
        }
        break;
    }

    if(status == MMCSD_STS_SUCCESS)
    {
        if(tuningRequired == true)
        {
            if(object->initHandle->tuningType == MMCSD_PHY_TUNING_TYPE_AUTO)
            {
                status = MMCSD_phyTuneAuto(handle);
            }
            else
            {
                status = MMCSD_phyTuneManual(handle);
            }
        }
    }

    if(status == MMCSD_STS_SUCCESS)
    {
        object->setBusSpeed = setSpeedUser;
    }

    return status;
}


int32_t MMCSD_lld_change_Tuning_Type(MMCSDLLD_Handle handle,
                                     uint32_t tuningType)
{
    MMCSDLLD_Object *object = (MMCSDLLD_Object *)handle;
    int32_t status = MMCSD_STS_SUCCESS;

    if( (tuningType == MMCSD_PHY_TUNING_TYPE_AUTO) ||
        (tuningType == MMCSD_PHY_TUNING_TYPE_MANUAL))
    {
        object->initHandle->tuningType = tuningType;
    }
    else
    {
        status = MMCSD_STS_ERR_INVALID_PARAM;
    }

    return status;
}

int32_t MMCSD_lld_enableBootPartition(MMCSDLLD_Handle handle,
                                      uint32_t partitionNum)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;
    MMCSDLLD_Transaction    trans;

    if(object->initHandle->cardType == MMCSD_CARD_TYPE_EMMC)
    {
        /* Enable boot partition */
        if((partitionNum == 1U) || (partitionNum == 2U))
        {
            uint8_t bootAck = 1U; /* ROM Needs boot ack */
            uint8_t bootPartition = (   (bootAck << 6U)                 |
                                        ((uint8_t)partitionNum << 3U)   |
                                        ((uint8_t)partitionNum));
            uint8_t bootBusWidth = 0x02U;
            uint32_t arg = (uint32_t)(  ((uint32_t)bootPartition << 8U) |
                                        ((uint32_t)0xB3U << 16U)        |
                                        ((uint32_t)0x03U << 24U));

            /* Configure the ECSD register using CMD6 */
            MMCSD_lld_initTransaction(&trans);
            trans.cmd = MMCSD_MMC_CMD(6U);
            trans.arg = arg;
            status = MMCSD_lld_transferPoll(handle, &trans);

            /* Delay for 3 ms for the change to take effect in the device */
            object->initHandle->Clock_uSleep(3000);

            if(status == SystemP_SUCCESS)
            {
                /* Set bus width now */
                arg = (uint32_t)(   ((uint32_t)bootBusWidth << 8U)  |
                                    ((uint32_t)0xB1U << 16U)        |
                                    ((uint32_t)0x03U << 24U));

                MMCSD_lld_initTransaction(&trans);
                trans.cmd = MMCSD_MMC_CMD(6U);
                trans.arg = arg;
                status = MMCSD_lld_transferPoll(handle, &trans);
            }

            /* Delay for 3 ms for the change to take effect in the device */
            object->initHandle->Clock_uSleep(3000);
        }
        else
        {
            status = MMCSD_STS_ERR_INVALID_PARAM;
        }
    }
    else
    {
        status = MMCSD_STS_ERR;
    }

    return status;
}

int32_t MMCSD_lld_disableBootPartition(MMCSDLLD_Handle handle)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;
    MMCSDLLD_Transaction    trans;

    if(object->initHandle->cardType == MMCSD_CARD_TYPE_EMMC)
    {
        /* Disable boot partition */
        uint8_t bootPartition = 0U;
        uint32_t arg = (uint32_t)(  ((uint32_t)bootPartition << 8U) |
                                    ((uint32_t)0xB3U << 16U)                  |
                                    ((uint32_t)0x03U << 24U));

        /* Configure the ECSD register using CMD6 */
        MMCSD_lld_initTransaction(&trans);
        trans.cmd = MMCSD_MMC_CMD(6U);
        trans.arg = arg;
        status = MMCSD_lld_transferPoll(handle, &trans);

        /* Delay for 5 ms for the change to take effect in the device */
        object->initHandle->Clock_uSleep(3000);
    }
    else
    {
        status = MMCSD_STS_ERR;
    }

    return status;
}

/* ========================================================================== */
/*                     API ISR Function Definitions                           */
/* ========================================================================== */

/* Hwi interrupt handler to service the MMCSD peripheral */
void MMCSD_lld_Isr(void *args)
{
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)NULL_PTR;
    int32_t                 xferStatus = MMCSD_STS_SUCCESS;
    uint16_t                normalIntrStatus = 0U;
    uint16_t                errorIntrStatus = 0U;
    uint32_t                offset = 0U;
    uint32_t                dataLength = 0U;
    uint8_t                 *pTempWord = NULL;
    uint32_t                tempWord = 0xFFFFFFFFU;
    uint32_t                i = 0;

    object = (MMCSDLLD_Object *)args;

    if(args != NULL_PTR)
    {
        /* Get interrupt stats */
        normalIntrStatus = MMCSD_getNormIntrStat(
            object->initHandle->ctrlBaseAddr, MMCSD_INTERRUPT_ALL_NORMAL);
        errorIntrStatus = MMCSD_getErrIntrStat(
            object->initHandle->ctrlBaseAddr, MMCSD_INTERRUPT_ALL_ERROR);


        switch(object->xferState)
        {
            case MMCSD_XFER_IDLE_STATE:
            {
                /* Do Nothing */
            }
            break;

            case MMCSD_XFER_CMD_STATE:
            {
                /* Command Complete interrupt Happened */
                if((normalIntrStatus &
                    CSL_MMC_CTLCFG_NORMAL_INTR_STS_CMD_COMPLETE_MASK) !=
                    (uint16_t)0U)
                {
                    /* Clear the Interrupt */
                    MMCSD_clearNormIntrStat(object->initHandle->ctrlBaseAddr,
                            CSL_MMC_CTLCFG_NORMAL_INTR_STS_CMD_COMPLETE_MASK);

                    /* Check if transfer has data or not */
                    if((object->currentTxn->cmd &
                        CSL_MMC_CTLCFG_COMMAND_DATA_PRESENT_MASK) != 0U)
                    {
                        /* Transfer Has Data */
                        /* Get command response */
                        (void)MMCSD_getCmdResponse(object->initHandle->ctrlBaseAddr,
                                                   object->currentTxn->response);

                        if(object->currentTxn->dir == MMCSD_CMD_XFER_TYPE_WRITE)
                        {
                            object->xferState = MMCSD_XFER_WRITE_STATE;
                        }
                        else
                        {
                            object->xferState = MMCSD_XFER_READ_STATE;
                        }
                    }
                    else
                    {
                        /* Transfer Has no Data */
                        /* Get command response */
                        (void)MMCSD_getCmdResponse(object->initHandle->ctrlBaseAddr,
                                                   object->currentTxn->response);
                        /* Reset interrupt configuration */
                        MMCSD_intrConfigReset(object);
                        /* Success Case */
                        if(object->currentTxn->cmd == MMCSD_SD_CMD(12U))
                        {
                            /* Success Case */
                            xferStatus = MMCSD_STS_SUCCESS;
                            /* Callback to application and finishup */
                            MMCSD_lld_completeCurrTransfer(object, xferStatus);
                        }
                        else
                        {
                            /* Clear pointer to current Transaction */
                            object->currentTxn = NULL;
                            /* Change Transfer state back to IDLE */
                            object->xferState = MMCSD_XFER_IDLE_STATE;
                            /*Change Driver State back to IDLE */
                            object->state = MMCSD_STATE_IDLE;
                        }
                    }
                }
                else if(errorIntrStatus != 0U)
                {
                    /* Interrupt Happened due to some Error */
                    /* Store Command Error */
                    object->cmdErrorStat = errorIntrStatus;
                    /* Reset interrupt configuration */
                    MMCSD_intrConfigReset(object);
                    /* Failure Case */
                    xferStatus = MMCSD_STS_ERR;
                    /* Callback to application and finishup */
                    MMCSD_lld_completeCurrTransfer(object, xferStatus);
                }
                else
                {
                    /* No Code */
                }
            }
            break;

            case MMCSD_XFER_WRITE_STATE:
            {
                if((normalIntrStatus &
                    CSL_MMC_CTLCFG_NORMAL_INTR_STS_BUF_WR_READY_MASK) != 0U)
                {
                    MMCSD_clearNormIntrStat(object->initHandle->ctrlBaseAddr,
                            CSL_MMC_CTLCFG_NORMAL_INTR_STS_BUF_WR_READY_MASK);

                    if(object->remainingBlockCount != 0U)
                    {
                        dataLength = object->dataBlockSize;
                        offset = (( (object->dataBlockCount) -
                                    (object->remainingBlockCount)) *
                                    (object->dataBlockSize));

                        for(i = 0U; i < dataLength; i += 4U)
                        {
                            pTempWord = (uint8_t *)&tempWord;
                            *(pTempWord)      = object->dataBufIdx[offset + i];
                            *(pTempWord + 1U) = object->dataBufIdx[offset + i + 1U];
                            *(pTempWord + 2U) = object->dataBufIdx[offset + i + 2U];
                            *(pTempWord + 3U) = object->dataBufIdx[offset + i + 3U];

                            MMCSD_writeDataPort(object->initHandle->ctrlBaseAddr,
                                                &tempWord);
                        }
                        object->remainingBlockCount--;
                    }
                    else
                    {
                        /* Remaining Block Count is zero, do nothing */
                    }
                }
                /* Check if transfer Complete interrupt Happened */
                else if((normalIntrStatus &
                        CSL_MMC_CTLCFG_NORMAL_INTR_STS_XFER_COMPLETE_MASK) != 0U)
                {
                    /* Clear the transfer complete interrupt */
                    MMCSD_clearNormIntrStat(object->initHandle->ctrlBaseAddr,
                                CSL_MMC_CTLCFG_NORMAL_INTR_STS_XFER_COMPLETE_MASK);
                    /* Reset interrupt configuration */
                    MMCSD_intrConfigReset(object);
                    /* Through the SD Back into trans State */
                    if(object->currentTxn->blockCount > 1U)
                    {
                        if(object->initHandle->cardType == MMCSD_CARD_TYPE_SD)
                        {
                            MMCSD_lld_initTransaction(&object->mmcsdTxn);
                            object->mmcsdTxn.cmd = MMCSD_SD_CMD(12U);
                            object->mmcsdTxn.arg = 0U;
                            (void)MMCSD_lld_transferIntr(object,
                                                         &object->mmcsdTxn);
                        }
                        else
                        {
                            /* Success Case */
                            xferStatus = MMCSD_STS_SUCCESS;
                            /* Callback to application and finishup */
                            MMCSD_lld_completeCurrTransfer(object, xferStatus);
                        }
                    }
                    else
                    {
                        /* Success Case */
                        xferStatus = MMCSD_STS_SUCCESS;
                        /* Callback to application and finishup */
                        MMCSD_lld_completeCurrTransfer(object, xferStatus);
                    }
                }

                else if(errorIntrStatus != 0U)
                {
                    /* Interrupt Happened due to some Error */
                    /* Store Data Error */
                    object->xferErrorStat = errorIntrStatus;
                    /* Reset interrupt configuration */
                    MMCSD_intrConfigReset(object);
                    /* Failure Case */
                    xferStatus = MMCSD_STS_ERR;
                    /* Callback to application and finishup */
                    MMCSD_lld_completeCurrTransfer(object, xferStatus);
                }
                else
                {
                    /* No Code */
                }
            }
            break;

            case MMCSD_XFER_READ_STATE:
            {
                if((normalIntrStatus &
                    CSL_MMC_CTLCFG_NORMAL_INTR_STS_BUF_RD_READY_MASK) != 0U)
                {
                    MMCSD_clearNormIntrStat(object->initHandle->ctrlBaseAddr,
                            CSL_MMC_CTLCFG_NORMAL_INTR_STS_BUF_RD_READY_MASK);

                    if(object->remainingBlockCount != 0U)
                    {
                        dataLength = object->dataBlockSize;
                        offset = (( (object->dataBlockCount) -
                                    (object->remainingBlockCount)) *
                                    (object->dataBlockSize));

                        for(i = 0U; i < dataLength; i += 4U)
                        {
                            MMCSD_readDataPort(object->initHandle->ctrlBaseAddr,
                                               &tempWord);
                            pTempWord = (uint8_t *)&tempWord;
                            object->dataBufIdx[offset + i] = *(pTempWord);
                            object->dataBufIdx[offset + i + 1U] = *(pTempWord + 1U);
                            object->dataBufIdx[offset + i + 2U] = *(pTempWord + 2U);
                            object->dataBufIdx[offset + i + 3U] = *(pTempWord + 3U);
                        }
                        object->remainingBlockCount--;
                    }
                    else
                    {
                        /* Remaining Block Count is zero, do nothing */
                    }
                }
                else if((normalIntrStatus &
                        CSL_MMC_CTLCFG_NORMAL_INTR_STS_XFER_COMPLETE_MASK) != 0U)
                {
                    /* Clear the transfer complete interrupt */
                    MMCSD_clearNormIntrStat(object->initHandle->ctrlBaseAddr,
                            CSL_MMC_CTLCFG_NORMAL_INTR_STS_XFER_COMPLETE_MASK);
                    /* Reset interrupt configuration */
                    MMCSD_intrConfigReset(object);
                    /* Success Case */
                    xferStatus = MMCSD_STS_SUCCESS;
                    /* Callback to application and finishup */
                    MMCSD_lld_completeCurrTransfer(object, xferStatus);
                }
                else if(errorIntrStatus != 0U)
                {
                    /* Interrupt Happened due to some Error */
                    /* Store Data Error */
                    object->xferErrorStat = errorIntrStatus;
                    /* Reset interrupt configuration */
                    MMCSD_intrConfigReset(object);
                    /* Failure Case */
                    xferStatus = MMCSD_STS_ERR;
                    /* Callback to application and finishup */
                    MMCSD_lld_completeCurrTransfer(object, xferStatus);
                }
                else
                {
                    /* No Code */
                }
            }
            break;

            default:
            break;

        }
    }
}

/* ========================================================================== */
/*                      Internal Function Definitions                         */
/* ========================================================================== */

/* Software Reset for All */
static int32_t MMCSD_softReset(uint32_t ctrlBaseAddr, uint32_t loopTimeout)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    int32_t status = MMCSD_STS_SUCCESS;
    uint32_t timeout = loopTimeout;

    CSL_REG8_FINS(&mmcsdRegs->SOFTWARE_RESET,
        MMC_CTLCFG_SOFTWARE_RESET_SWRST_FOR_ALL,
        CSL_MMC_CTLCFG_SOFTWARE_RESET_SWRST_FOR_ALL_VAL_RESET);

    uint8_t resetStatus = CSL_REG8_FEXT(&mmcsdRegs->SOFTWARE_RESET,
                            MMC_CTLCFG_SOFTWARE_RESET_SWRST_FOR_ALL);

    while((resetStatus != CSL_MMC_CTLCFG_SOFTWARE_RESET_SWRST_FOR_CMD_VAL_WORK)
        && (timeout > 0U))
    {
        timeout--;
        resetStatus = CSL_REG8_FEXT(&mmcsdRegs->SOFTWARE_RESET,
                        MMC_CTLCFG_SOFTWARE_RESET_SWRST_FOR_ALL);
    }

    if(timeout == 0U)
    {
        status = MMCSD_STS_ERR_TIMEOUT;
    }

    return status;
}

/* Software Reset for CMD Line (SD Mode Only) */
static int32_t MMCSD_linesResetCmd(uint32_t ctrlBaseAddr, uint32_t loopTimeout)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    int32_t status = MMCSD_STS_SUCCESS;
    uint32_t timeout = loopTimeout;

    CSL_REG8_FINS(&mmcsdRegs->SOFTWARE_RESET,
        MMC_CTLCFG_SOFTWARE_RESET_SWRST_FOR_CMD,
        CSL_MMC_CTLCFG_SOFTWARE_RESET_SWRST_FOR_ALL_VAL_RESET);

    uint8_t resetStatus = CSL_REG8_FEXT(&mmcsdRegs->SOFTWARE_RESET,
                            MMC_CTLCFG_SOFTWARE_RESET_SWRST_FOR_CMD);

    while((resetStatus != CSL_MMC_CTLCFG_SOFTWARE_RESET_SWRST_FOR_ALL_VAL_WORK)
        && (timeout > 0U))
    {
        timeout--;
        resetStatus = CSL_REG8_FEXT(&mmcsdRegs->SOFTWARE_RESET,
                        MMC_CTLCFG_SOFTWARE_RESET_SWRST_FOR_CMD);
    }

    if(timeout == 0U)
    {
        status = MMCSD_STS_ERR_TIMEOUT;
    }

    return status;
}

/* Software Reset for DAT Line (SD Mode Only) */
static int32_t MMCSD_linesResetDat(uint32_t ctrlBaseAddr, uint32_t loopTimeout)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    int32_t status = MMCSD_STS_SUCCESS;
    uint32_t timeout = loopTimeout;

    CSL_REG8_FINS(&mmcsdRegs->SOFTWARE_RESET,
        MMC_CTLCFG_SOFTWARE_RESET_SWRST_FOR_DAT,
        CSL_MMC_CTLCFG_SOFTWARE_RESET_SWRST_FOR_ALL_VAL_RESET);

    uint8_t resetStatus = CSL_REG8_FEXT(&mmcsdRegs->SOFTWARE_RESET,
                            MMC_CTLCFG_SOFTWARE_RESET_SWRST_FOR_DAT);

    while((resetStatus != CSL_MMC_CTLCFG_SOFTWARE_RESET_SWRST_FOR_ALL_VAL_WORK)
        && (timeout > 0U))
    {
        timeout--;
        resetStatus = CSL_REG8_FEXT(&mmcsdRegs->SOFTWARE_RESET,
                        MMC_CTLCFG_SOFTWARE_RESET_SWRST_FOR_DAT);
    }

    if(timeout == 0U)
    {
        status = MMCSD_STS_ERR_TIMEOUT;
    }

    return status;
}

static void MMCSD_setBusWidth(uint32_t ctrlBaseAddr, uint32_t busWidth)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;

    switch(busWidth)
    {
        case MMCSD_BUS_WIDTH_8BIT:
            CSL_REG8_FINS(&mmcsdRegs->HOST_CONTROL1,
                MMC_CTLCFG_HOST_CONTROL1_EXT_DATA_WIDTH,
                CSL_MMC_CTLCFG_HOST_CONTROL1_EXT_DATA_WIDTH_VAL_ENABLE);
            break;

        case MMCSD_BUS_WIDTH_4BIT:
            CSL_REG8_FINS(&mmcsdRegs->HOST_CONTROL1,
                MMC_CTLCFG_HOST_CONTROL1_EXT_DATA_WIDTH,
                CSL_MMC_CTLCFG_HOST_CONTROL1_EXT_DATA_WIDTH_VAL_DISABLE);
            CSL_REG8_FINS(&mmcsdRegs->HOST_CONTROL1,
                MMC_CTLCFG_HOST_CONTROL1_DATA_WIDTH,
                CSL_MMC_CTLCFG_HOST_CONTROL1_DATA_WIDTH_VAL_BIT4);
            break;

        case MMCSD_BUS_WIDTH_1BIT:
            CSL_REG8_FINS(&mmcsdRegs->HOST_CONTROL1,
                MMC_CTLCFG_HOST_CONTROL1_EXT_DATA_WIDTH,
                CSL_MMC_CTLCFG_HOST_CONTROL1_EXT_DATA_WIDTH_VAL_DISABLE);
            CSL_REG8_FINS(&mmcsdRegs->HOST_CONTROL1,
                MMC_CTLCFG_HOST_CONTROL1_DATA_WIDTH,
                CSL_MMC_CTLCFG_HOST_CONTROL1_DATA_WIDTH_VAL_BIT1);
            break;

        default:
            CSL_REG8_FINS(&mmcsdRegs->HOST_CONTROL1,
                MMC_CTLCFG_HOST_CONTROL1_EXT_DATA_WIDTH,
                CSL_MMC_CTLCFG_HOST_CONTROL1_EXT_DATA_WIDTH_VAL_DISABLE);
            CSL_REG8_FINS(&mmcsdRegs->HOST_CONTROL1,
                MMC_CTLCFG_HOST_CONTROL1_DATA_WIDTH,
                CSL_MMC_CTLCFG_HOST_CONTROL1_DATA_WIDTH_VAL_BIT1);
            break;
    }
}

static void MMCSD_selectDma(uint32_t ctrlBaseAddr, uint32_t dmaSel)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    CSL_REG8_FINS(&mmcsdRegs->HOST_CONTROL1,
                  MMC_CTLCFG_HOST_CONTROL1_DMA_SELECT, dmaSel);
}

static void MMCSD_setBusVolt(uint32_t ctrlBaseAddr, uint32_t voltage)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;

    CSL_REG8_FINS(  &mmcsdRegs->POWER_CONTROL,
                    MMC_CTLCFG_POWER_CONTROL_SD_BUS_VOLTAGE,
                    voltage);
}

static bool MMCSD_isCardInserted(uint32_t ctrlBaseAddr)
{
    bool retVal = false;
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;

    if(CSL_REG32_FEXT(  &mmcsdRegs->PRESENTSTATE,
                        MMC_CTLCFG_PRESENTSTATE_CARD_INSERTED) != 0U)
    {
        retVal = true;
    }
    return retVal;
}

static int32_t MMCSD_busPowerOnCtrl(uint32_t ctrlBaseAddr, bool pwr,
                                    uint32_t loopTimeout)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    uint8_t regVal = 0U;
    int32_t status = MMCSD_STS_SUCCESS;
    uint32_t timeout = loopTimeout;

    if(pwr)
    {
        regVal = 1U;
    }

    CSL_REG8_FINS(  &mmcsdRegs->POWER_CONTROL,
                    MMC_CTLCFG_POWER_CONTROL_SD_BUS_POWER,
                    regVal);

    if(pwr)
    {
        while(( CSL_REG8_FEXT(&mmcsdRegs->POWER_CONTROL,
                MMC_CTLCFG_POWER_CONTROL_SD_BUS_POWER) != 1U)
                && (timeout > 0U))
        {
            timeout--;
        }

        if(timeout == 0U)
        {
            status = MMCSD_STS_ERR_TIMEOUT;
        }
    }

    return status;
}

static bool MMCSD_isIntClkStable(uint32_t ctrlBaseAddr, uint32_t loopTimeout)
{
    bool status = true;
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    uint32_t timeout = loopTimeout;

    while(( CSL_REG16_FEXT(&mmcsdRegs->CLOCK_CONTROL,
            MMC_CTLCFG_CLOCK_CONTROL_INT_CLK_STABLE) !=
            CSL_MMC_CTLCFG_CLOCK_CONTROL_INT_CLK_STABLE_VAL_READY) &&
            (timeout > 0U))
    {
        timeout--;
    }

    if(timeout == 0U)
    {
        status = false;
    }

    return status;
}

static int32_t MMCSD_internalClockCtrl(uint32_t ctrlBaseAddr, bool clkState)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    uint32_t clockState = 0U;
    int32_t status = MMCSD_STS_SUCCESS;

    if(clkState)
    {
        clockState = 1U;
    }

    CSL_REG16_FINS(&mmcsdRegs->CLOCK_CONTROL,
                   MMC_CTLCFG_CLOCK_CONTROL_INT_CLK_ENA,
                   clockState);

    if(clkState)
    {
        if(!(MMCSD_isIntClkStable(ctrlBaseAddr, 0xFFFFU)))
        {
            status = MMCSD_STS_ERR;
        }
    }

    return status;
}

static int32_t MMCSD_pllClockCtrl(uint32_t ctrlBaseAddr, bool pllState)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    uint32_t clockState = 0U;
    int32_t status = MMCSD_STS_SUCCESS;

    if(pllState)
    {
        clockState = 1U;
    }

    CSL_REG16_FINS(&mmcsdRegs->CLOCK_CONTROL,
                   MMC_CTLCFG_CLOCK_CONTROL_PLL_ENA,
                   clockState);

    if(pllState)
    {
        if(!(MMCSD_isIntClkStable(ctrlBaseAddr, 0xFFFFU)))
        {
            status = MMCSD_STS_ERR;
        }
    }

    return status;
}

static void MMCSD_sdClockCtrl(uint32_t ctrlBaseAddr, bool clkState)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    uint32_t clockState = 0U;
    if(clkState)
    {
        clockState = CSL_MMC_CTLCFG_CLOCK_CONTROL_SD_CLK_ENA_VAL_ENABLE;
    }
    else
    {
        clockState = CSL_MMC_CTLCFG_CLOCK_CONTROL_SD_CLK_ENA_VAL_DISABLE;
    }
    /* Disable SD Clock */
    CSL_REG16_FINS( &mmcsdRegs->CLOCK_CONTROL,
                    MMC_CTLCFG_CLOCK_CONTROL_SD_CLK_ENA,
                    clockState);
}

static void MMCSD_setFreqSelSD(uint32_t ctrlBaseAddr, uint32_t clkDiv)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;

    CSL_REG16_FINS( &mmcsdRegs->CLOCK_CONTROL,
                    MMC_CTLCFG_CLOCK_CONTROL_SDCLK_FRQSEL,
                    (clkDiv & (uint32_t)0xFFU));
    CSL_REG16_FINS( &mmcsdRegs->CLOCK_CONTROL,
                    MMC_CTLCFG_CLOCK_CONTROL_SDCLK_FRQSEL_UPBITS,
                    ((clkDiv >> 8U) & (uint32_t)0x03U));
}

static int32_t MMCSD_setBusFreq(uint32_t ctrlBaseAddr, uint32_t inClk,
                                uint32_t outClk)
{
    CSL_mmc_ctlcfgRegsOvly  mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    int32_t                 status = MMCSD_STS_SUCCESS;
    uint32_t                clkDiv = 0U;

    /* Enable internal clocks */
    status = MMCSD_internalClockCtrl(ctrlBaseAddr, true);

    /* Disable Clock */
    CSL_REG16_FINS( &mmcsdRegs->CLOCK_CONTROL,
                    MMC_CTLCFG_CLOCK_CONTROL_SD_CLK_ENA,
                    CSL_MMC_CTLCFG_CLOCK_CONTROL_SD_CLK_ENA_VAL_DISABLE);

    if(status == MMCSD_STS_SUCCESS)
    {
        clkDiv = inClk/(outClk * 2U);
        if(clkDiv > 1023U)
        {
            clkDiv = 1023U;
        }
        while((inClk/clkDiv) > (2U * outClk))
        {
            if(clkDiv == 1023U)
            {
                status = MMCSD_STS_ERR;
                break;
            }
            clkDiv++;
        }

        if(status == MMCSD_STS_SUCCESS)
        {
            CSL_REG16_FINS( &mmcsdRegs->CLOCK_CONTROL,
                            MMC_CTLCFG_CLOCK_CONTROL_SDCLK_FRQSEL,
                            (clkDiv & 0xFFU));
            CSL_REG16_FINS( &mmcsdRegs->CLOCK_CONTROL,
                            MMC_CTLCFG_CLOCK_CONTROL_SDCLK_FRQSEL_UPBITS,
                            ((clkDiv >> 8U) & 0x03U));

            if(!(MMCSD_isIntClkStable(ctrlBaseAddr, 0xFFFFU)))
            {
                status = MMCSD_STS_ERR;
            }
        }

        if(status == MMCSD_STS_SUCCESS)
        {
            /* Enable Clock */
            CSL_REG16_FINS( &mmcsdRegs->CLOCK_CONTROL,
                            MMC_CTLCFG_CLOCK_CONTROL_SD_CLK_ENA,
                            CSL_MMC_CTLCFG_CLOCK_CONTROL_SD_CLK_ENA_VAL_ENABLE);
        }
    }
    else
    {
        /* No Code; Status becomes Timeout */
    }

    return status;
}

static int32_t MMCSD_setClockFreqSD(MMCSDLLD_Handle handle, uint32_t outClk)
{
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;
    uint32_t                ctrlBaseAddr = object->initHandle->ctrlBaseAddr;
    uint32_t                inClk = object->initHandle->inputClkFreq;
    int32_t                 status = MMCSD_STS_SUCCESS;
    uint32_t                clkDiv = 0U;

    /* Disable internal clocks */
    status = MMCSD_internalClockCtrl(ctrlBaseAddr, false);

    /* Disable SD Clock */
    MMCSD_sdClockCtrl(ctrlBaseAddr, false);

    /* Calculate Devisor Val */
    clkDiv = inClk / (outClk * 2U);
    if(clkDiv > 1023U)
    {
        clkDiv = 1023U;
    }
    while((inClk/clkDiv) > (2U * outClk))
    {
        if(clkDiv == 1023U)
        {
            status = MMCSD_STS_ERR;
            break;
        }
        clkDiv++;
    }

    if(status == MMCSD_STS_SUCCESS)
    {
        /* Set FRQSEL Bits */
        MMCSD_setFreqSelSD(ctrlBaseAddr, clkDiv);
        /* Programmable Clock Mode is not used instead Divided Clock Mode
        is used as it is supported by all IPs */

        /* Enable internal clocks */
        status = MMCSD_internalClockCtrl(ctrlBaseAddr, true);
    }

    if(status == MMCSD_STS_SUCCESS)
    {
        if(handle->initHandle->pllEnableSD)
        {
            status = MMCSD_pllClockCtrl(ctrlBaseAddr, true);
        }
        else
        {
            status = MMCSD_STS_SUCCESS;
        }
    }

    if(status == MMCSD_STS_SUCCESS)
    {
        /* Enable SD Clock */
        MMCSD_sdClockCtrl(ctrlBaseAddr, true);
    }

    return status;
}


static int32_t MMCSD_getCmdResponse(uint32_t ctrlBaseAddr, uint32_t *rsp)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    CSL_mmc_ctlcfgRegsOvly  mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;

    if(rsp != NULL)
    {
        rsp[0] =    ((uint32_t)CSL_REG16_RD(&mmcsdRegs->RESPONSE[0]) |
                    ((uint32_t)CSL_REG16_RD(&mmcsdRegs->RESPONSE[1]) << 16));
        rsp[1] =    ((uint32_t)CSL_REG16_RD(&mmcsdRegs->RESPONSE[2]) |
                    ((uint32_t)CSL_REG16_RD(&mmcsdRegs->RESPONSE[3]) << 16));
        rsp[2] =    ((uint32_t)CSL_REG16_RD(&mmcsdRegs->RESPONSE[4]) |
                    ((uint32_t)CSL_REG16_RD(&mmcsdRegs->RESPONSE[5]) << 16));
        rsp[3] =    ((uint32_t)CSL_REG16_RD(&mmcsdRegs->RESPONSE[6]) |
                    ((uint32_t)CSL_REG16_RD(&mmcsdRegs->RESPONSE[7]) << 16));
    }
    else
    {
        status = MMCSD_STS_ERR_INVALID_PARAM;
    }

    return status;
}

static void MMCSD_setUHSMode(uint32_t ctrlBaseAddr, uint32_t uhsMode)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;

    /* Disable Clock */
    CSL_REG16_FINS( &mmcsdRegs->CLOCK_CONTROL,
                    MMC_CTLCFG_CLOCK_CONTROL_SD_CLK_ENA,
                    CSL_MMC_CTLCFG_CLOCK_CONTROL_SD_CLK_ENA_VAL_DISABLE);

    /* Set UHS Mode */
    CSL_REG16_FINS( &mmcsdRegs->HOST_CONTROL2,
                    MMC_CTLCFG_HOST_CONTROL2_UHS_MODE_SELECT, uhsMode);

    /* Enable Clock */
    CSL_REG16_FINS( &mmcsdRegs->CLOCK_CONTROL,
                    MMC_CTLCFG_CLOCK_CONTROL_SD_CLK_ENA,
                    CSL_MMC_CTLCFG_CLOCK_CONTROL_SD_CLK_ENA_VAL_ENABLE);

}

static void MMCSD_sendCommand(  uint32_t ctrlBaseAddr,
                                MMCSDLLD_Transaction *trans)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;

    /* Check if transaction is multi Block */
    bool multiBlock = (trans->blockCount > 1U) ? true : false;

    /* Clear transfer mode and command registers */
    CSL_REG16_WR(&mmcsdRegs->TRANSFER_MODE, 0U);

    /* Set command argument */
    CSL_REG16_WR(&mmcsdRegs->ARGUMENT1_LO, (trans->arg & 0xFFFFU));
    CSL_REG16_WR(&mmcsdRegs->ARGUMENT1_HI, ((trans->arg >> 16) & 0xFFFFU));

    /* Set Block Count */
    CSL_REG16_WR(&mmcsdRegs->BLOCK_COUNT, trans->blockCount);
    /* Set Block Size */
    CSL_REG16_WR(&mmcsdRegs->BLOCK_SIZE, trans->blockSize);

    if((trans->cmd & CSL_MMC_CTLCFG_COMMAND_DATA_PRESENT_MASK) != 0U)
    {
        /* Case where data is present */
        if(multiBlock)
        {
            /* Set Transfer Mode for Multi Block Transaction */
            CSL_REG16_WR(&mmcsdRegs->TRANSFER_MODE,
            ((trans->enableDma << CSL_MMC_CTLCFG_TRANSFER_MODE_DMA_ENA_SHIFT) |
            (1U << CSL_MMC_CTLCFG_TRANSFER_MODE_BLK_CNT_ENA_SHIFT) |
            (trans->autoCmdEn << CSL_MMC_CTLCFG_TRANSFER_MODE_AUTO_CMD_ENA_SHIFT) |
            (trans->dir << CSL_MMC_CTLCFG_TRANSFER_MODE_DATA_XFER_DIR_SHIFT) |
            (1U << CSL_MMC_CTLCFG_TRANSFER_MODE_MULTI_BLK_SEL_SHIFT)));

        }
        else
        {
            /* Set Transfer Mode for Single Block Transaction */
            CSL_REG16_WR(&mmcsdRegs->TRANSFER_MODE,
            ((trans->enableDma << CSL_MMC_CTLCFG_TRANSFER_MODE_DMA_ENA_SHIFT) |
            (0U << CSL_MMC_CTLCFG_TRANSFER_MODE_BLK_CNT_ENA_SHIFT) |
            (trans->autoCmdEn << CSL_MMC_CTLCFG_TRANSFER_MODE_AUTO_CMD_ENA_SHIFT) |
            (trans->dir << CSL_MMC_CTLCFG_TRANSFER_MODE_DATA_XFER_DIR_SHIFT) |
            (0U << CSL_MMC_CTLCFG_TRANSFER_MODE_MULTI_BLK_SEL_SHIFT)));
        }
    }
    else
    {
        /* Command where data is not Present, just write Command */
    }

    /* Write Command */
    CSL_REG16_WR(&mmcsdRegs->COMMAND, (uint16_t)trans->cmd);
}

static int32_t MMCSD_waitCommandInhibit(uint32_t ctrlBaseAddr,
                                        uint32_t loopTimeout)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    int32_t status = MMCSD_STS_SUCCESS;
    uint32_t timeout = loopTimeout;

    while(( CSL_REG32_FEXT(&mmcsdRegs->PRESENTSTATE,
            MMC_CTLCFG_PRESENTSTATE_INHIBIT_CMD) != 0U) && (timeout > 0U))
    {
        timeout--;
    }

    if(timeout == 0U)
    {
        status = MMCSD_STS_ERR_TIMEOUT;
    }

    return status;
}

static int32_t MMCSD_waitDataInhibit(   uint32_t ctrlBaseAddr,
                                        uint32_t loopTimeout)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    int32_t status = MMCSD_STS_SUCCESS;
    uint32_t timeout = loopTimeout;

    while(( CSL_REG32_FEXT(&mmcsdRegs->PRESENTSTATE,
            MMC_CTLCFG_PRESENTSTATE_INHIBIT_DAT) != 0U) && (timeout > 0U))
    {
        timeout--;
    }

    if(timeout == 0U)
    {
        status = MMCSD_STS_ERR_TIMEOUT;
    }

    return status;
}

static void MMCSD_writeDataPort(uint32_t ctrlBaseAddr, uint32_t *dataP)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    CSL_REG32_WR(&mmcsdRegs->DATA_PORT, *dataP);
}

static void MMCSD_readDataPort(uint32_t ctrlBaseAddr, uint32_t *dataP)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    *dataP = CSL_REG32_RD(&mmcsdRegs->DATA_PORT);
}

static bool MMCSD_is3V3Supported(uint32_t ctrlBaseAddr)
{
    bool retVal = false;
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;

    if(CSL_REG64_FEXT(  &mmcsdRegs->CAPABILITIES,
                        MMC_CTLCFG_CAPABILITIES_VOLT_3P3_SUPPORT) != 0U)
    {
        retVal = true;
    }
    return retVal;
}

static bool MMCSD_is3V0Supported(uint32_t ctrlBaseAddr)
{
    bool retVal = false;
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;

    if(CSL_REG64_FEXT(  &mmcsdRegs->CAPABILITIES,
                        MMC_CTLCFG_CAPABILITIES_VOLT_3P0_SUPPORT) != 0U)
    {
        retVal = true;
    }
    return retVal;
}

static bool MMCSD_getDAT0(uint32_t ctrlBaseAddr)
{
    bool retVal = false;
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    if(CSL_REG32_FEXT(&mmcsdRegs->PRESENTSTATE,
                         MMC_CTLCFG_PRESENTSTATE_SDIF_DAT0IN) == 1U)
    {
        retVal = true;
    }
    return retVal;
}

static int32_t MMCSD_waitDAat0inBusy(uint32_t ctrlBaseAddr,
                                     uint32_t loopTimeout)
{
    int32_t status = MMCSD_STS_SUCCESS;
    uint32_t timeout = loopTimeout;
    while(MMCSD_getDAT0(ctrlBaseAddr) != true)
    {
        timeout--;
        if(timeout == 0U){
            status = MMCSD_STS_ERR;
            break;
        }
    }
    return status;
}

static void MMCSD_1P8VsignalCtrl(uint32_t ctrlBaseAddr, bool sigEnable)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    uint32_t fieldVal = 0U;
    if(sigEnable)
    {
        fieldVal = 1U;
    }
    CSL_REG16_FINS(&mmcsdRegs->HOST_CONTROL2,
                   MMC_CTLCFG_HOST_CONTROL2_V1P8_SIGNAL_ENA, fieldVal);
}

static bool MMCSD_get1P8VsignalStat(uint32_t ctrlBaseAddr)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    bool retVal = false;

    if((CSL_REG16_FEXT(&mmcsdRegs->HOST_CONTROL2,
            MMC_CTLCFG_HOST_CONTROL2_V1P8_SIGNAL_ENA)) == 0x01U)
    {
        retVal = true;
    }

    return retVal;
}

static void MMCSD_setExecuteTuning(uint32_t ctrlBaseAddr)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    CSL_REG16_FINS(&mmcsdRegs->HOST_CONTROL2,
                   MMC_CTLCFG_HOST_CONTROL2_EXECUTE_TUNING, 1U);
}

static void MMCSD_resetExecuteTuning(uint32_t ctrlBaseAddr)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    CSL_REG16_FINS(&mmcsdRegs->HOST_CONTROL2,
                   MMC_CTLCFG_HOST_CONTROL2_EXECUTE_TUNING, 0U);
}

static void MMCSD_resetSamplingClkSelect(uint32_t ctrlBaseAddr)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    CSL_REG16_FINS(&mmcsdRegs->HOST_CONTROL2,
                   MMC_CTLCFG_HOST_CONTROL2_SAMPLING_CLK_SELECT, 0U);
}
static void MMCSD_setSamplingClkSelect(uint32_t ctrlBaseAddr)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    CSL_REG16_FINS(&mmcsdRegs->HOST_CONTROL2,
                   MMC_CTLCFG_HOST_CONTROL2_SAMPLING_CLK_SELECT, 1U);
}

static uint16_t MMCSD_getExecuteTuning(uint32_t ctrlBaseAddr)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    return (CSL_REG16_FEXT(&mmcsdRegs->HOST_CONTROL2,
                           MMC_CTLCFG_HOST_CONTROL2_EXECUTE_TUNING));
}

static uint16_t MMCSD_getSamplingClkSelect(uint32_t ctrlBaseAddr)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    return (CSL_REG16_FEXT(&mmcsdRegs->HOST_CONTROL2,
                           MMC_CTLCFG_HOST_CONTROL2_SAMPLING_CLK_SELECT));
}

static void MMCSD_setVer40Enable(uint32_t ctrlBaseAddr)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    CSL_REG16_FINS(&mmcsdRegs->HOST_CONTROL2,
                   MMC_CTLCFG_HOST_CONTROL2_HOST_VER40_ENA, 1U);
}

static void MMCSD_setADMA2LenMode(uint32_t ctrlBaseAddr, bool bitLen26Mode)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    if(bitLen26Mode)
    {
        CSL_REG16_FINS(&mmcsdRegs->HOST_CONTROL2,
                    MMC_CTLCFG_HOST_CONTROL2_ADMA2_LEN_MODE, 1U);
    }
    else
    {
        CSL_REG16_FINS(&mmcsdRegs->HOST_CONTROL2,
                    MMC_CTLCFG_HOST_CONTROL2_ADMA2_LEN_MODE, 0U);
    }
}

static bool MMCSD_isUHSSpeedSupportedHOST(uint32_t ctrlBaseAddr,
                                      uint32_t speedMode)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    bool retVal = false;
    switch(speedMode)
    {
        case MMCSD_UHS_MODE_SDR25:
        {
            retVal = true;
        }
        break;
        case MMCSD_UHS_MODE_SDR50:
        {
            if((mmcsdRegs->CAPABILITIES &
                    CSL_MMC_CTLCFG_CAPABILITIES_SDR50_SUPPORT_MASK) != 0x0U)
            {
                retVal = true;
            }
        }
        break;
        case MMCSD_UHS_MODE_SDR104:
        {
            if((mmcsdRegs->CAPABILITIES &
                    CSL_MMC_CTLCFG_CAPABILITIES_SDR104_SUPPORT_MASK) != 0x0U)
            {
                retVal = true;
            }
        }
        break;
        case MMCSD_UHS_MODE_DDR50:
        {
            if((mmcsdRegs->CAPABILITIES,
                    CSL_MMC_CTLCFG_CAPABILITIES_DDR50_SUPPORT_MASK) != 0x0U)
            {
                retVal = true;
            }
        }
        break;

        default:
        break;
    }

    return retVal;
}

static bool MMCSD_isUHSDriveStrengthSupportedHOST(uint32_t ctrlBaseAddr,
                                                  uint32_t driverStrength)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    bool retVal = false;

    switch(driverStrength)
    {
        case MMCSD_CMD6_DRIVE_STRENGTH_TYPE_B:
        {
            retVal = true;
        }
        break;
        case MMCSD_CMD6_DRIVE_STRENGTH_TYPE_A:
        {
            if((mmcsdRegs->CAPABILITIES &
                    CSL_MMC_CTLCFG_CAPABILITIES_DRIVERA_SUPPORT_MASK) != 0x0U)
            {
                retVal = true;
            }
        }
        break;
        case MMCSD_CMD6_DRIVE_STRENGTH_TYPE_C:
        {
            if((mmcsdRegs->CAPABILITIES &
                    CSL_MMC_CTLCFG_CAPABILITIES_DRIVERC_SUPPORT_MASK) != 0x0U)
            {
                retVal = true;
            }
        }
        break;
        case MMCSD_CMD6_DRIVE_STRENGTH_TYPE_D:
        {
            if((mmcsdRegs->CAPABILITIES &
                    CSL_MMC_CTLCFG_CAPABILITIES_DRIVERD_SUPPORT_MASK) != 0x0U)
            {
                retVal = true;
            }
        }
        break;

        default:
        break;
    }

    return retVal;
}

static void MMCSD_writeADMA2DescAddr(uint32_t ctrlBaseAddr, uint64_t desc)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;
    CSL_REG64_WR(&mmcsdRegs->ADMA_SYS_ADDRESS, desc);
}

static uint16_t MMCSD_getNormIntrStat(uint32_t ctrlBaseAddr, uint16_t intrFlag)
{
    uint16_t regVal = 0U;
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;

    regVal = (uint16_t)(CSL_REG16_RD(&mmcsdRegs->NORMAL_INTR_STS) & intrFlag);

    return regVal;
}

static void MMCSD_clearNormIntrStat(uint32_t ctrlBaseAddr, uint16_t intrFlag)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;

    CSL_REG16_WR(&mmcsdRegs->NORMAL_INTR_STS, intrFlag);
}

static uint16_t MMCSD_getErrIntrStat(uint32_t ctrlBaseAddr, uint16_t intrFlag)
{
    uint16_t regVal = 0U;
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;

    regVal = (uint16_t)(CSL_REG16_RD(&mmcsdRegs->ERROR_INTR_STS) & intrFlag);

    return regVal;
}

static void MMCSD_clearErrIntrStat(uint32_t ctrlBaseAddr, uint16_t intrFlag)
{
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;

    CSL_REG16_WR(&mmcsdRegs->ERROR_INTR_STS, intrFlag);
}

static void MMCSD_enableNormIntrStat(uint32_t ctrlBaseAddr, uint16_t intrFlag)
{
    uint16_t regVal = 0U;
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;

    regVal = CSL_REG16_RD(&mmcsdRegs->NORMAL_INTR_STS_ENA);
    regVal |= intrFlag;
    CSL_REG16_WR(&mmcsdRegs->NORMAL_INTR_STS_ENA, regVal);
}

static void MMCSD_disableNormIntrStat(uint32_t ctrlBaseAddr, uint16_t intrFlag)
{
    uint16_t regVal = 0U;
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;

    regVal = CSL_REG16_RD(&mmcsdRegs->NORMAL_INTR_STS_ENA);
    regVal &= ~intrFlag;
    CSL_REG16_WR(&mmcsdRegs->NORMAL_INTR_STS_ENA, regVal);
}

static void MMCSD_enableErrIntrStat(uint32_t ctrlBaseAddr, uint16_t intrFlag)
{
    uint16_t regVal = 0U;
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;

    regVal = CSL_REG16_RD(&mmcsdRegs->ERROR_INTR_STS_ENA);
    regVal |= intrFlag;
    CSL_REG16_WR(&mmcsdRegs->ERROR_INTR_STS_ENA, regVal);
}

static void MMCSD_disableErrIntrStat(uint32_t ctrlBaseAddr, uint16_t intrFlag)
{
    uint16_t regVal = 0U;
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;

    regVal = CSL_REG16_RD(&mmcsdRegs->ERROR_INTR_STS_ENA);
    regVal &= ~intrFlag;
    CSL_REG16_WR(&mmcsdRegs->ERROR_INTR_STS_ENA, regVal);
}

static void MMCSD_enableNormSigIntr(uint32_t ctrlBaseAddr, uint16_t intrFlag)
{
    uint16_t regVal = 0U;
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;

    regVal = CSL_REG16_RD(&mmcsdRegs->NORMAL_INTR_SIG_ENA);
    regVal |= intrFlag;
    CSL_REG16_WR(&mmcsdRegs->NORMAL_INTR_SIG_ENA, regVal);
}

static void MMCSD_disableNormSigIntr(uint32_t ctrlBaseAddr, uint16_t intrFlag)
{
    uint16_t regVal = 0U;
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;

    regVal = CSL_REG16_RD(&mmcsdRegs->NORMAL_INTR_SIG_ENA);
    regVal &= ~intrFlag;
    CSL_REG16_WR(&mmcsdRegs->NORMAL_INTR_SIG_ENA, regVal);
}

static void MMCSD_enableErrSigIntr(uint32_t ctrlBaseAddr, uint16_t intrFlag)
{
    uint16_t regVal = 0U;
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;

    regVal = CSL_REG16_RD(&mmcsdRegs->ERROR_INTR_SIG_ENA);
    regVal |= intrFlag;
    CSL_REG16_WR(&mmcsdRegs->ERROR_INTR_SIG_ENA, regVal);
}

static void MMCSD_disableErrSigIntr(uint32_t ctrlBaseAddr, uint16_t intrFlag)
{
    uint16_t regVal = 0U;
    CSL_mmc_ctlcfgRegsOvly mmcsdRegs = (CSL_mmc_ctlcfgRegsOvly)ctrlBaseAddr;

    regVal = CSL_REG16_RD(&mmcsdRegs->ERROR_INTR_SIG_ENA);
    regVal &= ~intrFlag;
    CSL_REG16_WR(&mmcsdRegs->ERROR_INTR_SIG_ENA, regVal);
}

static void MMCSD_setSlotType(uint32_t ssBaseAddr, uint8_t slotType)
{
    CSL_mmc_sscfgRegsOvly pSSReg = NULL;
    pSSReg = (CSL_mmc_sscfgRegsOvly)(ssBaseAddr);

    switch(slotType)
    {
        case MMCSD_SLOT_TYPE_VAL_REMOVABLE:
        {
            /* Set the slot type to removable */
            CSL_REG32_FINS(&pSSReg->CTL_CFG_2_REG,
            MMC_SSCFG_CTL_CFG_2_REG_SLOTTYPE,
            CSL_MMC_CTLCFG_CAPABILITIES_SLOT_TYPE_VAL_REMOVABLE);

            /* Enable IOMUX : 0 for MMCSD, 1(default) for GPIO */
            CSL_REG32_FINS(&pSSReg->PHY_CTRL_1_REG,
            MMC_SSCFG_PHY_CTRL_1_REG_IOMUX_ENABLE,
            0U);
        }
        break;

        case MMCSD_SLOT_TYPE_VAL_EMBEDDED:
        {
            CSL_REG32_FINS(&pSSReg->CTL_CFG_2_REG,
            MMC_SSCFG_CTL_CFG_2_REG_SLOTTYPE,
            CSL_MMC_CTLCFG_CAPABILITIES_SLOT_TYPE_VAL_EMBEDDED);
        }
        break;

        case MMCSD_SLOT_TYPE_VAL_SHARED:
        {
            /* TODO */
        }
        break;

        default:
        break;
    }
}

static void MMCSD_phySetDriveImpedance(uint32_t ssBaseAddr,
                                       uint32_t drvImpedance)
{
    CSL_mmc_sscfgRegsOvly pSSReg = (CSL_mmc_sscfgRegsOvly)(ssBaseAddr);
    CSL_REG32_FINS(&pSSReg->PHY_CTRL_1_REG,
                   MMC_SSCFG_PHY_CTRL_1_REG_DR_TY, drvImpedance);
}

static void MMCSD_phyEnableDLL(uint32_t ssBaseAddr)
{
    CSL_mmc_sscfgRegsOvly pSSReg = (CSL_mmc_sscfgRegsOvly)(ssBaseAddr);
    CSL_REG32_FINS(&pSSReg->PHY_CTRL_1_REG,
                   MMC_SSCFG_PHY_CTRL_1_REG_ENDLL, 1U);
}

static void MMCSD_phyDisableDLL(uint32_t ssBaseAddr)
{
    CSL_mmc_sscfgRegsOvly pSSReg = (CSL_mmc_sscfgRegsOvly)(ssBaseAddr);
    CSL_REG32_FINS(&pSSReg->PHY_CTRL_1_REG,
                   MMC_SSCFG_PHY_CTRL_1_REG_ENDLL, 0U);
}

static void MMCSD_phySetDLLTrimICP(uint32_t ssBaseAddr, uint32_t dllTrimICP)
{
    CSL_mmc_sscfgRegsOvly pSSReg = (CSL_mmc_sscfgRegsOvly)(ssBaseAddr);
    CSL_REG32_FINS(&pSSReg->PHY_CTRL_1_REG,
                   MMC_SSCFG_PHY_CTRL_1_REG_DLL_TRM_ICP, dllTrimICP);
}

static int32_t MMCSD_phyCalibIO(uint32_t ssBaseAddr, uint32_t loopTimeout)
{
    int32_t status = MMCSD_STS_SUCCESS;
    CSL_mmc_sscfgRegsOvly pSSReg = (CSL_mmc_sscfgRegsOvly)(ssBaseAddr);
    uint32_t timeout = loopTimeout;

    /* Set EN_RTRIM bit */
    CSL_REG32_FINS(&pSSReg->PHY_CTRL_1_REG,
                   MMC_SSCFG_PHY_CTRL_1_REG_EN_RTRIM, 1U);
    while(CSL_REG32_FEXT(&pSSReg->PHY_CTRL_1_REG,
                         MMC_SSCFG_PHY_CTRL_1_REG_EN_RTRIM) != 1U)
    {
        timeout--;
        if(timeout == 0U)
        {
            status = MMCSD_STS_ERR_TIMEOUT;
            break;
        }
    }

    if(status == MMCSD_STS_SUCCESS)
    {
        /* Set PDB to trigger calibration */
        CSL_REG32_FINS(&pSSReg->PHY_CTRL_1_REG,
                       MMC_SSCFG_PHY_CTRL_1_REG_PDB, 1U);
        /* Wait for calibration to finish */
        while(CSL_REG32_FEXT(&pSSReg->PHY_STAT_1_REG,
                             MMC_SSCFG_PHY_STAT_1_REG_CALDONE) != 1U)
        {
            timeout--;
            if(timeout == 0U)
            {
                status = MMCSD_STS_ERR_TIMEOUT;
                break;
            }
        }
    }

    return status;
}

static void MMCSD_phySetSTRBSEL(uint32_t ssBaseAddr, uint32_t strobeSel)
{
    CSL_mmc_sscfgRegsOvly pSSReg = (CSL_mmc_sscfgRegsOvly)(ssBaseAddr);
    CSL_REG32_FINS(&pSSReg->PHY_CTRL_4_REG,
                   MMC_SSCFG_PHY_CTRL_4_REG_STRBSEL, strobeSel);
}

static void MMCSD_phyDisableTapChgWIN(uint32_t ssBaseAddr)
{
    CSL_mmc_sscfgRegsOvly pSSReg = (CSL_mmc_sscfgRegsOvly)(ssBaseAddr);
    CSL_REG32_FINS(&pSSReg->PHY_CTRL_4_REG,
                   MMC_SSCFG_PHY_CTRL_4_REG_ITAPCHGWIN, 1U);
}

static void MMCSD_phyEnableTapChgWIN(uint32_t ssBaseAddr)
{
    CSL_mmc_sscfgRegsOvly pSSReg = (CSL_mmc_sscfgRegsOvly)(ssBaseAddr);
    CSL_REG32_FINS(&pSSReg->PHY_CTRL_4_REG,
                   MMC_SSCFG_PHY_CTRL_4_REG_ITAPCHGWIN, 0U);
}


static void MMCSD_phySetOutTapDelay(uint32_t ssBaseAddr, uint32_t dlySel,
                                    uint32_t dlyEn)
{
    CSL_mmc_sscfgRegsOvly pSSReg = (CSL_mmc_sscfgRegsOvly)(ssBaseAddr);

    /* Set Output Tap Delay and select the Output Tap Delay */
    CSL_REG32_FINS(&pSSReg->PHY_CTRL_4_REG,
                   MMC_SSCFG_PHY_CTRL_4_REG_OTAPDLYENA, dlyEn);
    CSL_REG32_FINS(&pSSReg->PHY_CTRL_4_REG,
                   MMC_SSCFG_PHY_CTRL_4_REG_OTAPDLYSEL, dlySel);
}
static void MMCSD_phySetInTapDelay(uint32_t ssBaseAddr, uint32_t dlySel,
                                   uint32_t dlyEn)
{
    CSL_mmc_sscfgRegsOvly pSSReg = (CSL_mmc_sscfgRegsOvly)(ssBaseAddr);

    /* Set Input Tap Delay and select the Input Tap Delay */
    CSL_REG32_FINS(&pSSReg->PHY_CTRL_4_REG,
                   MMC_SSCFG_PHY_CTRL_4_REG_ITAPDLYENA, dlyEn);
    CSL_REG32_FINS(&pSSReg->PHY_CTRL_4_REG,
                   MMC_SSCFG_PHY_CTRL_4_REG_ITAPDLYSEL, dlySel);
}

static void MMCSD_phySetFrqSelDLL(uint32_t ssBaseAddr, uint32_t clkFreq)
{
    CSL_mmc_sscfgRegsOvly pSSReg = (CSL_mmc_sscfgRegsOvly)(ssBaseAddr);
    uint32_t freqSel = 0U, regVal = 0U;

    /* Configure freqSel */
    if((clkFreq > (170U * 1000000U)) && (clkFreq <= (200U * 1000000U)))
    {
        freqSel = 0U;
    }
    else if((clkFreq > (140U * 1000000U)) && (clkFreq <= (170U * 1000000U)))
    {
        freqSel = 1U;
    }
    else if((clkFreq > (110U * 1000000U)) && (clkFreq <= (140U * 1000000U)))
    {
        freqSel = 2U;
    }
    else if((clkFreq > (80U * 1000000U)) && (clkFreq <= (110U * 1000000U)))
    {
        freqSel = 3U;
    }
    else if((clkFreq > (50U * 1000000U)) && (clkFreq <= (80U * 1000000U)))
    {
        freqSel = 4U;
    }
    else if((clkFreq > (250U * 1000000U)) && (clkFreq <= (275U * 1000000U)))
    {
        freqSel = 5U;
    }
    else if((clkFreq > (225U * 1000000U)) && (clkFreq <= (250U * 1000000U)))
    {
        freqSel = 6U;
    }
    else if((clkFreq > (200U * 1000000U)) && (clkFreq <= (225U * 1000000U)))
    {
        freqSel = 7U;
    }
    else
    {
        /* Default 50 MHz */
        freqSel = 4U;
    }

    /* FRQSEL bit field not available in CSLR? */
    regVal = CSL_REG32_RD(&pSSReg->PHY_CTRL_5_REG);
    regVal &= ~(0x00000700U);
    regVal |= (uint32_t)(freqSel << 8U);
    CSL_REG32_WR(&pSSReg->PHY_CTRL_5_REG, regVal);
}

static int32_t MMCSD_phyWaitDLLReady(uint32_t ssBaseAddr, uint32_t loopTimeout)
{
    int32_t status = MMCSD_STS_SUCCESS;
    CSL_mmc_sscfgRegsOvly pSSReg = (CSL_mmc_sscfgRegsOvly)(ssBaseAddr);
    uint32_t timeout = loopTimeout;

    while(CSL_REG32_FEXT(&pSSReg->PHY_STAT_1_REG,
                         MMC_SSCFG_PHY_STAT_1_REG_DLLRDY) != 1U)
    {
        timeout--;
        if(timeout == 0U)
        {
            status = MMCSD_STS_ERR_TIMEOUT;
            break;
        }
    }

    return status;
}

static void MMCSD_phyWriteCTRL2Reg(uint32_t ssBaseAddr, uint32_t regVal)
{
    CSL_mmc_sscfgRegsOvly pSSReg = (CSL_mmc_sscfgRegsOvly)(ssBaseAddr);
    CSL_REG32_WR(&pSSReg->PHY_CTRL_2_REG, regVal);
}

static void MMCSD_phyWriteCTRL3Reg(uint32_t ssBaseAddr, uint32_t regVal)
{
    CSL_mmc_sscfgRegsOvly pSSReg = (CSL_mmc_sscfgRegsOvly)(ssBaseAddr);
    CSL_REG32_WR(&pSSReg->PHY_CTRL_3_REG, regVal);
}

static void MMCSD_phyGetTapDelay(uint32_t *oTapDelayEn, uint32_t *oTapDelaySel,
                                 uint32_t *iTapDelayEn, uint32_t *iTapDelaySel,
                                 uint32_t phyMode)
{
    switch(phyMode) {
        case MMCSD_PHY_MODE_SDR50:
        case MMCSD_PHY_MODE_HSSDR50:
            *oTapDelayEn    = 1U;
            *oTapDelaySel   = 8U;
            *iTapDelayEn    = 0U;
            *iTapDelaySel   = 0U;
            break;
        case MMCSD_PHY_MODE_HS200:
        case MMCSD_PHY_MODE_SDR104:
            *oTapDelayEn    = 1U;
            *oTapDelaySel   = 7U;
            *iTapDelayEn    = 1U;
            *iTapDelaySel   = 0U;
            break;
        case MMCSD_PHY_MODE_DDR50:
            *oTapDelayEn    = 1U;
            *oTapDelaySel   = 6U;
            *iTapDelayEn    = 1U;
            *iTapDelaySel   = 3U;
            break;
        case MMCSD_PHY_MODE_HS400:
            *oTapDelayEn    = 1U;
            *oTapDelaySel   = 2U;
            *iTapDelayEn    = 0U;
            *iTapDelaySel   = 0U;
            break;
        case MMCSD_PHY_MODE_DS:
        case MMCSD_PHY_MODE_HS:
            *oTapDelayEn    = 0U;
            *oTapDelaySel   = 0U;
            *iTapDelayEn    = 0U;
            *iTapDelaySel   = 0U;
            break;
        default:
            break;
    }
}

static int32_t MMCSD_phyConfigure(uint32_t ssBaseAddr, uint32_t phyMode,
                                  uint32_t phyClkFreq, uint32_t driveImpedance)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    uint32_t                strobeSel = 0U;
    uint32_t                dllTrimICP = 8U;
    uint32_t                oTapDelayEn = 0U, oTapDelaySel = 0U;
    uint32_t                iTapDelayEn = 0U, iTapDelaySel = 0U;

    /* Sets STROBESEL Val */
    MMCSD_phySetSTRBSEL(ssBaseAddr, strobeSel);

    /* Disables DLL */
    MMCSD_phyDisableDLL(ssBaseAddr);

    /* Sets FRQSEL Value */
    MMCSD_phySetFrqSelDLL(ssBaseAddr, phyClkFreq);
    /* Sets DLL_TRIM_ICP to 8 */
    MMCSD_phySetDLLTrimICP(ssBaseAddr, dllTrimICP);
    /* Sets Drive Impedance */
    MMCSD_phySetDriveImpedance(ssBaseAddr, driveImpedance);
    /* Enables DLL */
    MMCSD_phyEnableDLL(ssBaseAddr);
    /* Disables ITAPCHGWIN */
    MMCSD_phyDisableTapChgWIN(ssBaseAddr);
    /* sets Tap Delays for input and output BASED on PHYMODE */
    MMCSD_phyGetTapDelay(&oTapDelayEn, &oTapDelaySel,
                         &iTapDelayEn, &iTapDelaySel, phyMode);
    MMCSD_phySetOutTapDelay(ssBaseAddr, oTapDelaySel, oTapDelayEn);
    MMCSD_phySetInTapDelay(ssBaseAddr, iTapDelaySel, iTapDelayEn);
    /* Enables ITAPCHGWIN */
    MMCSD_phyEnableTapChgWIN(ssBaseAddr);
    /* Waits for DLL to be Ready */
    status = MMCSD_phyWaitDLLReady(ssBaseAddr, MMCSD_WAIT_FOREVER);

    return status;
}

static int32_t MMCSD_phyInit(uint32_t ssBaseAddr, uint32_t phyType)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    uint32_t                dllTrimICP = 8U;
    uint32_t                timeout = 0xFFFF;

    switch(phyType)
    {
        case MMCSD_PHY_TYPE_HW_PHY:
            {
                /* DLL(Delay-locked loop)_TRIM_ICP value to 8 */
                MMCSD_phySetDLLTrimICP(ssBaseAddr, dllTrimICP);
                /* Reset MMCSD0_SS_PHY_CTRL_2_REG (Reset to default) */
                MMCSD_phyWriteCTRL2Reg(ssBaseAddr, 0U);
                /* Disable Pull Up/Down On The STRB Line in
                    MMCSD0_SS_PHY_CTRL_3_REG */
                MMCSD_phyWriteCTRL3Reg(ssBaseAddr,
                                       MMCSD_PHY_CTRL_3_REG_DEFAULT);
                /* Enable Calibration by setting EN_RTRIM bit PHY_CTRL_1_REG */
                status = MMCSD_phyCalibIO(ssBaseAddr, timeout);
            }
            break;

        case MMCSD_PHY_TYPE_SW_PHY:
            {
                /* Enable Output Tap Delay and select the Output Tap Delay */
                MMCSD_phySetOutTapDelay(ssBaseAddr, 2U, 1U);
            }
            break;

        case MMCSD_PHY_TYPE_NO_PHY:
            {
                /* Do Nothing */
            }
            break;

        default:
            break;
    }

    return status;
}

static int32_t MMCSD_sendTuningDataMMC(MMCSDLLD_Handle handle)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;
    MMCSDLLD_Transaction    trans;

    /* Send CMD 21 */
    MMCSD_lld_initTransaction(&trans);

    trans.cmd = MMCSD_MMC_CMD(21U);
    trans.arg = 0U;
    trans.dir = MMCSD_CMD_XFER_TYPE_READ;
    trans.blockCount = 1U;
    if(object->setBusWidth == MMCSD_BUS_WIDTH_8BIT)
    {
        trans.blockSize = sizeof(gTuningPattern8Bit);
    }
    else if(object->setBusWidth == MMCSD_BUS_WIDTH_4BIT)
    {
        trans.blockSize = sizeof(gTuningPattern4Bit);
    }
    else
    {
        /* No Code */
    }
    trans.dataBuf = object->initHandle->dataBuf;
    trans.isTuning = TRUE;
    trans.enableDma = FALSE;

    status = MMCSD_lld_transferPoll(handle, &trans);

    if( (status == MMCSD_STS_SUCCESS) &&
        (object->initHandle->tuningType == MMCSD_PHY_TUNING_TYPE_MANUAL))
    {
        if(object->setBusWidth == MMCSD_BUS_WIDTH_8BIT)
        {
            status = memcmp(gTuningPattern8Bit, object->initHandle->dataBuf,
                            sizeof(gTuningPattern8Bit));
        }
        else if(object->setBusWidth == MMCSD_BUS_WIDTH_4BIT)
        {
            status = memcmp(gTuningPattern4Bit, object->initHandle->dataBuf,
                            sizeof(gTuningPattern4Bit));
        }
        else
        {
            /* No  Code, Tuning wont happen in one bit case */
            status = MMCSD_STS_ERR;
        }

    }
    else
    {
        /* No Code */
    }

    return status;
}

static int32_t MMCSD_sendTuningDataSD(MMCSDLLD_Handle handle)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;
    MMCSDLLD_Transaction    trans;

    /* Send CMD 19 */
    MMCSD_lld_initTransaction(&trans);

    trans.cmd = MMCSD_SD_CMD(19U);
    trans.arg = 0U;
    trans.dir = MMCSD_CMD_XFER_TYPE_READ;
    trans.blockCount = 1U;
    trans.blockSize = sizeof(gTuningPattern4Bit);
    trans.dataBuf = object->initHandle->dataBuf;
    trans.isTuning = TRUE;
    trans.enableDma = FALSE;

    status = MMCSD_lld_transferPoll(handle, &trans);

    if( (status == MMCSD_STS_SUCCESS) &&
        (object->initHandle->tuningType == MMCSD_PHY_TUNING_TYPE_MANUAL))
    {
        status = memcmp(gTuningPattern4Bit, object->initHandle->dataBuf,
                        sizeof(gTuningPattern4Bit));
    }
    else
    {
        /* No Code */
    }

    return status;
}

static int32_t MMCSD_phyTuneAuto(MMCSDLLD_Handle handle)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;
    uint16_t                execTuning = 0xFFU, smpClock = 0U;
    uint32_t                i = 0;
    bool tuningSuccess = false;

    /* Reset SAMPLING_CLK_SELECT bit */
    MMCSD_resetSamplingClkSelect(object->initHandle->ctrlBaseAddr);
    /* Set EXECUTE_TUNING bit */
    MMCSD_setExecuteTuning(object->initHandle->ctrlBaseAddr);

    /* Send Tuning Data Multiple Times */
    for(i = 0U; i < 40U; i++)
    {
        /* CMD 19, get Tuning Data */
        if(object->initHandle->cardType == MMCSD_CARD_TYPE_EMMC)
        {
            (void)MMCSD_sendTuningDataMMC(handle);
        }
        else if(object->initHandle->cardType == MMCSD_CARD_TYPE_SD)
        {
            (void)MMCSD_sendTuningDataSD(handle);
        }
        else
        {
            /* No Code */
        }

        /* Get Execute Tuning & Sampling Clock Select bit */
        execTuning = MMCSD_getExecuteTuning(object->initHandle->ctrlBaseAddr);
        smpClock = MMCSD_getSamplingClkSelect(object->initHandle->ctrlBaseAddr);

        if((execTuning == 0U) && (smpClock == 1U))
        {
            /* Tuning Success Case */
            tuningSuccess = true;
            break;
        }
    }

    if(tuningSuccess != true)
    {
        /* Tuning Failed */
        MMCSD_resetExecuteTuning(object->initHandle->ctrlBaseAddr);
        MMCSD_resetSamplingClkSelect(object->initHandle->ctrlBaseAddr);
        status = MMCSD_STS_ERR;
    }

    /* Reset CMD Line */
    (void)MMCSD_linesResetCmd(object->initHandle->ctrlBaseAddr,
                              MMCSD_WAIT_FOREVER);
    /* Reset DAT Lines */
    (void)MMCSD_linesResetDat(object->initHandle->ctrlBaseAddr,
                              MMCSD_WAIT_FOREVER);

    return status;
}

static int32_t MMCSD_phyTuneManual(MMCSDLLD_Handle handle)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;
    uint32_t                minTapDelay = 1U;
    uint32_t                maxTapDelay = 32U;
    uint32_t                i = 0U;
    bool                    patternMatch = false;

    /* Reset SAMPLING_CLK_SELECT bit */
    MMCSD_resetSamplingClkSelect(object->initHandle->ctrlBaseAddr);
    /* Set EXECUTE_TUNING bit */
    MMCSD_setExecuteTuning(object->initHandle->ctrlBaseAddr);
    /* Disable DLL */
    MMCSD_phyDisableDLL(object->initHandle->ssBaseAddr);
    /* Enable input Tap Delay */
    MMCSD_phySetInTapDelay(object->initHandle->ssBaseAddr, 0U, 1U);
    /* Enable DLL */
    MMCSD_phyEnableDLL(object->initHandle->ssBaseAddr);
    /* Disable Tap Change Window before changing tap value */
    MMCSD_phyDisableTapChgWIN(object->initHandle->ssBaseAddr);
    /* Change input tap delay Select to 1 */
    MMCSD_phySetInTapDelay(object->initHandle->ssBaseAddr, 1U, 1U);

    /* Send Tuning Data Multiple Times */
    for(i = 1U; i < 32U; i++)
    {
        /* Enable tap delay change window window */
        MMCSD_phyEnableTapChgWIN(object->initHandle->ssBaseAddr);
        /* CMD 19/21, get Tuning Data */
        if(object->initHandle->cardType == MMCSD_CARD_TYPE_EMMC)
        {
            status = MMCSD_sendTuningDataMMC(handle);
        }
        else if(object->initHandle->cardType == MMCSD_CARD_TYPE_SD)
        {
            status = MMCSD_sendTuningDataSD(handle);
        }
        else
        {
            /* No Code */
        }

        if(status == MMCSD_STS_SUCCESS)
        {
            /* Check Pattern Match condition */
            if(patternMatch == false)
            {
                minTapDelay = i;
                maxTapDelay = i;
                patternMatch = true;
            }
            else
            {
                maxTapDelay = i;
            }
        }
        else
        {
            if(patternMatch == true)
            {
                break;
            }
        }

        /* Disable Tap Change Window before changing tap value */
        MMCSD_phyDisableTapChgWIN(object->initHandle->ssBaseAddr);
        /* Increment tap delay Select val by one */
        MMCSD_phySetInTapDelay(object->initHandle->ssBaseAddr, (1U + i), 1U);
    }

    if(patternMatch == true)
    {
        /* Tuning Success */
        /* Disable Tap Change Window before changing tap value */
        MMCSD_phyDisableTapChgWIN(object->initHandle->ssBaseAddr);
        /* Set Tap Delay Value */
        MMCSD_phySetInTapDelay(object->initHandle->ssBaseAddr,
                                ((minTapDelay + maxTapDelay) / 2U), 1U);
        /* Enable tap delay change window window */
        MMCSD_phyEnableTapChgWIN(object->initHandle->ssBaseAddr);
        /* Reset EXECUTE_TUNING bit, Tuning Complete */
        MMCSD_resetExecuteTuning(object->initHandle->ctrlBaseAddr);
        /* Tuned Clock is used to sample data */
        MMCSD_setSamplingClkSelect(object->initHandle->ctrlBaseAddr);
        status = MMCSD_STS_SUCCESS;
    }
    else
    {
        /* Tuning Filed */
        MMCSD_resetExecuteTuning(object->initHandle->ctrlBaseAddr);
        MMCSD_resetSamplingClkSelect(object->initHandle->ctrlBaseAddr);
        status = MMCSD_STS_ERR;
    }

    /* Reset CMD Line */
    (void)MMCSD_linesResetCmd(object->initHandle->ctrlBaseAddr,
                              MMCSD_WAIT_FOREVER);
    /* Reset DAT Lines */
    (void)MMCSD_linesResetDat(object->initHandle->ctrlBaseAddr,
                              MMCSD_WAIT_FOREVER);

    return status;
}

static int32_t MMCSD_lld_initSD(MMCSDLLD_Handle handle)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;
    MMCSD_SdDeviceData      *sdDeviceData = (MMCSD_SdDeviceData *)NULL;
    MMCSDLLD_Transaction    trans;
    uint32_t                flag_F8 = 0, flag_SDIO = 0;
    uint32_t                setSpeed = 0U, speedModeUHS = 0U;
    uint32_t                setDriverStrength = 0U, driverStrength = 0U;
    uint32_t                retry = 0xFFFFU;
    uint32_t                currState = MMCSD_SD_INIT_STATE_CONTROLLER_INIT;
    bool                    drvStrengthSet = false;
    bool                    speedSet = false;
    bool                    tuningRequired = false;
    bool                    initDone = false;

    sdDeviceData = (MMCSD_SdDeviceData *)object->initHandle->deviceData;

    while((initDone == false) && (status == MMCSD_STS_SUCCESS))
    {
        switch(currState)
        {
            case MMCSD_SD_INIT_STATE_CONTROLLER_INIT:
            {
                status = MMCSD_phyInit(object->initHandle->ssBaseAddr,
                                       object->initHandle->phyType);

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* Soft Reset The IP */
                    status = MMCSD_softReset(object->initHandle->ctrlBaseAddr,
                                             MMCSD_WAIT_FOREVER);
                }

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* Set Bus width to 1 as SD will start with 1 on Power up */
                    MMCSD_setBusWidth(object->initHandle->ctrlBaseAddr,
                                      MMCSD_BUS_WIDTH_1BIT);
                    /* Default bus voltage set to 3.3V */
                    MMCSD_setBusVolt(object->initHandle->ctrlBaseAddr,
                                     MMCSD_BUS_VOLT_3_3V);
                    /* Set Slot type to Removable */
                    MMCSD_setSlotType(object->initHandle->ssBaseAddr,
                                      MMCSD_SLOT_TYPE_VAL_REMOVABLE);
                    /* Wait for card detect */
                    status = MMCSD_waitCardDetect(handle, MMCSD_WAIT_FOREVER);
                }

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* Turn on Bus Power */
                    status = MMCSD_busPowerOnCtrl(
                                            object->initHandle->ctrlBaseAddr,
                                            true, MMCSD_WAIT_FOREVER);
                }

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* SD Clock Configuration, set it to DS(Default Speed) */
                    if(object->initHandle->uaBusSpeed == MMCSD_SD_MODE_HS)
                    {
                        status = MMCSD_setClockFreqSD(handle,
                                                      MMCSD_SD_HS_FREQUENCY_HZ);
                    }
                    else
                    {
                        status = MMCSD_setClockFreqSD(handle,
                                                      MMCSD_SD_DS_FREQUENCY_HZ);
                    }
                }

                if(status == MMCSD_STS_SUCCESS)
                {
                    currState = MMCSD_SD_INIT_STATE_CHECK_VOLTAGE;
                }
            }
            break;

            case MMCSD_SD_INIT_STATE_CHECK_VOLTAGE:
            {
                /* 3.9.4 Bus Speed Modes Selection Sequence
                   (Physical Layer Spec V3.01) */

                /* Send CMD 0 (Reset) */
                MMCSD_lld_initTransaction(&trans);
                trans.cmd = MMCSD_SD_CMD(0U);
                status = MMCSD_lld_transferPoll(handle, &trans);

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* CMD 8 to set Operating Voltage */
                    MMCSD_lld_initTransaction(&trans);
                    trans.cmd = MMCSD_SD_CMD(8U);
                    trans.arg = MMCSD_SD_CMD8_CHECK_PATTERN |
                                (MMCSD_SD_VOLT_2P7_3P6 << 8U);
                    status = MMCSD_lld_transferPoll(handle, &trans);
                }

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* Verify the Check Patter and Voltage */
                    if( ((trans.response[0] & 0xFFU) ==
                                                MMCSD_SD_CMD8_CHECK_PATTERN) &&
                        ((trans.response[0] & 0xF00U) ==
                                                (MMCSD_SD_VOLT_2P7_3P6 << 8U)) )
                    {
                        flag_F8 = 1U;
                    }

                    currState = MMCSD_SD_INIT_STATE_CHECK_SDIO;
                }
                else
                {
                    /* Unuseable Card */
                    status = MMCSD_STS_ERR_CARD_UNUSEABLE;
                }
            }
            break;

            case MMCSD_SD_INIT_STATE_CHECK_SDIO:
            {
                /* CMD 5 SDIO Check According to Figure 12-1753 AM64 TRM */
                MMCSD_lld_initTransaction(&trans);
                trans.cmd = MMCSD_SDIO_CMD(5U);
                trans.arg = 0U;
                status = MMCSD_lld_transferPoll(handle, &trans);

                /* Store the OCR */
                sdDeviceData->ocr = trans.response[0];

                if(status == MMCSD_STS_SUCCESS)
                {
                    if(trans.response[0] != 0x00U)
                    {
                        retry = 0xFFFFU;
                        MMCSD_lld_initTransaction(&trans);
                        while(  (((trans.response[0]) &
                                ((uint32_t)1U << 31U)) == 0U) && (retry != 0U))
                        {
                            MMCSD_lld_initTransaction(&trans);
                            trans.cmd = MMCSD_SDIO_CMD(5U);
                            trans.arg = ((uint32_t)1U << 24U) | (0xFFFFFFU);
                            status = MMCSD_lld_transferPoll(handle, &trans);

                            retry--;
                            if(retry == 0U)
                            {
                                status = MMCSD_STS_ERR;
                                break;
                            }
                        }

                        if(status == MMCSD_STS_SUCCESS)
                        {
                            /* Store the OCR */
                            sdDeviceData->ocr = trans.response[0];
                            flag_SDIO = 1U;
                        }

                        if(((sdDeviceData->ocr >> 30U) & 0x01U) != 0U)
                        {
                            /* Memory Present(MP) */
                        }
                        else
                        {
                            /* Memory Not Present */
                            if(flag_SDIO == 1U)
                            {
                                if(((sdDeviceData->ocr >> 27U) & 0x01U) != 0U)
                                {
                                    /* Change Bus Voltage */
                                    currState =
                                            MMCSD_SD_INIT_STATE_VOLTAGE_SWITCH;
                                }
                            }
                            else
                            {
                                status = MMCSD_STS_ERR_CARD_UNUSEABLE;
                            }
                        }
                    }
                }
                else
                {
                    /* CMD 5 will fail in case of a non SDIO card
                       change status back to
                    Success for next steps */
                    status = MMCSD_STS_SUCCESS;

                    currState = MMCSD_SD_INIT_STATE_CARD_IDENTIFICATION;
                }
            }
            break;

            case MMCSD_SD_INIT_STATE_CARD_IDENTIFICATION:
            {
                /* CMD55 Should always precede ACMD41 (APP_CMD) */
                MMCSD_lld_initTransaction(&trans);
                trans.cmd = MMCSD_SD_CMD(55U);
                (void)MMCSD_lld_transferPoll(handle, &trans);
                /* Enquiry ACMD 41 to get OCR */
                MMCSD_lld_initTransaction(&trans);
                trans.cmd = MMCSD_SD_ACMD(41U);
                trans.arg = 0U;
                status = MMCSD_lld_transferPoll(handle, &trans);

                if(status == MMCSD_STS_SUCCESS)
                {
                    retry = 0xFFFFU;
                    /* Start initialization */
                    MMCSD_lld_initTransaction(&trans);
                    while((((trans.response[0]) & ((uint32_t)1U << 31U)) == 0U)
                            && (retry != 0U))
                    {
                        MMCSD_lld_initTransaction(&trans);
                        trans.cmd = MMCSD_SD_CMD(55U);
                        status = MMCSD_lld_transferPoll(handle, &trans);

                        if(status == MMCSD_STS_SUCCESS)
                        {
                            /* 4.2.3.1 (ACMD41) V3.01 Physical Layer */
                            MMCSD_lld_initTransaction(&trans);
                            trans.cmd = MMCSD_SD_ACMD(41U);
                            trans.arg = 0U;

                            if(flag_F8 == 1U)
                            {
                                /* HCS=1 (SDHC & SDXC Supported by HOST) */
                                trans.arg |= ((uint32_t)1U << 30U);
                                /* S18R=1 (Switch to 1.8V Request) */
                                if(!(object->initHandle->autoAssignMaxSpeed) &&
                                    (   (object->initHandle->uaBusSpeed ==
                                                        MMCSD_SD_MODE_DS) ||
                                        (object->initHandle->uaBusSpeed ==
                                                        MMCSD_SD_MODE_HS)))
                                {
                                    trans.arg |= ((uint32_t)0U << 24U);
                                }
                                else
                                {
                                    trans.arg |= ((uint32_t)1U << 24U);
                                }

                                /* XPC=1 (SDXC Power Control-Max Performance) */
                                trans.arg |= ((uint32_t)1U << 28U);
                            }
                            /* Set voltage window to start Initialization */
                            trans.arg |= ((0xFF00U) << 8U);

                            status = MMCSD_lld_transferPoll(handle, &trans);
                        }

                        retry--;
                        if(retry == 0U)
                        {
                            status = MMCSD_STS_ERR;
                            break;
                        }
                    }

                    if(status == MMCSD_STS_SUCCESS)
                    {
                        /* Store the OCR (Page 104 V3.01 Physical Layer) */
                        sdDeviceData->ocr = trans.response[0];

                        /* Check CCS */
                        if(flag_F8 == 1U)
                        {
                            /* Check CCS */
                            if((((sdDeviceData->ocr) >> 30U) & 0x01U) == 0x01U)
                            {
                                /* Card type SDHC or SDXC */
                                sdDeviceData->isHC = true;
                                /* Check if switch voltage accepted or not */
                                if(((sdDeviceData->ocr >> 24U) & 0x01U) != 0U)
                                {
                                    /* Card ready to switch Voltage */
                                    if((!(object->initHandle->autoAssignMaxSpeed))
                                        &&( (object->initHandle->uaBusSpeed ==
                                                        MMCSD_SD_MODE_HS) ||
                                            (object->initHandle->uaBusSpeed ==
                                                        MMCSD_SD_MODE_DS)))
                                    {
                                        /* User has chosen HS or DS Manual Speed */
                                        /* Voltage switch should not happen */
                                        /* Go to Get CID State */
                                        currState = MMCSD_SD_INIT_STATE_GET_CID;
                                    }
                                    else
                                    {
                                        /* User Choice is not HS or DS or auto
                                        assign Enabled */
                                        currState =
                                            MMCSD_SD_INIT_STATE_VOLTAGE_SWITCH;
                                    }

                                }
                                else
                                {
                                    /* Card is Not SDSC therefore card
                                       already in low voltage mode */
                                    currState = MMCSD_SD_INIT_STATE_GET_CID;
                                }
                            }
                            else
                            {
                                /* Card type SDSC (Ver 2.0 or Ver 3.0) */
                                currState = MMCSD_SD_INIT_STATE_GET_CID;
                            }
                        }
                        else
                        {
                            /* Card type SDSC (Ver 1.01 or Ver 1.10) */
                            currState = MMCSD_SD_INIT_STATE_GET_CID;
                        }
                    }
                }
                else
                {
                    if(flag_SDIO == 1U)
                    {
                        currState = MMCSD_SD_INIT_STATE_VOLTAGE_SWITCH;
                    }
                    else
                    {
                        status = MMCSD_STS_ERR_CARD_UNUSEABLE;
                    }
                }
            }
            break;

            case MMCSD_SD_INIT_STATE_VOLTAGE_SWITCH:
            {
                /* Figure 12-1755. Signal Voltage Switch Procedure
                (AM64x_AM243x_TRM) */
                MMCSD_lld_initTransaction(&trans);
                trans.cmd = MMCSD_SD_CMD(11U);
                status = MMCSD_lld_transferPoll(handle, &trans);

                /* The card will Enter UHS Mode after this with speed SDR12 */
                /* In UHS Mode only 4 bit transfer data width is Supported */
                /* Check if response OK */
                if(status == MMCSD_STS_SUCCESS)
                {
                    /* Disable SD Clock */
                    MMCSD_sdClockCtrl(object->initHandle->ctrlBaseAddr, false);

                    /* Check DAT lines if LOW */
                    if(MMCSD_getDAT0(object->initHandle->ctrlBaseAddr))
                    {
                        /* Turn off Bus Power */
                        (void)MMCSD_busPowerOnCtrl(
                                        object->initHandle->ctrlBaseAddr,
                                        false, MMCSD_WAIT_FOREVER);
                        status = MMCSD_STS_ERR;
                    }
                }

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* Set 1.8V Signal Enable */
                    MMCSD_1P8VsignalCtrl(object->initHandle->ctrlBaseAddr,
                                         true);
                    /* Wait 5ms */
                    object->initHandle->Clock_uSleep(5000);
                    /* Check 1.8V signal Enable */
                    if(MMCSD_get1P8VsignalStat(
                                            object->initHandle->ctrlBaseAddr))
                    {
                        /* Enable SD Clock */
                        MMCSD_sdClockCtrl(object->initHandle->ctrlBaseAddr,
                                          true);
                        /* Wait 1ms */
                        object->initHandle->Clock_uSleep(1000);
                        /* Check DAT lines to go high */
                        if(!MMCSD_getDAT0(object->initHandle->ctrlBaseAddr))
                        {
                            /* Turn off Bus Power */
                            (void)MMCSD_busPowerOnCtrl(
                                            object->initHandle->ctrlBaseAddr,
                                            false, MMCSD_WAIT_FOREVER);
                            status = MMCSD_STS_ERR;
                        }
                    }
                    else
                    {
                        /* Turn off Bus Power */
                        (void)MMCSD_busPowerOnCtrl(
                                        object->initHandle->ctrlBaseAddr,
                                        false, MMCSD_WAIT_FOREVER);
                        status = MMCSD_STS_ERR;
                    }
                }

                if(status == MMCSD_STS_SUCCESS)
                {
                    if(flag_SDIO == 1U)
                    {
                        currState = MMCSD_SD_INIT_STATE_GET_RCA;
                    }
                    else
                    {
                        currState = MMCSD_SD_INIT_STATE_GET_CID;
                    }
                }
            }
            break;

            case MMCSD_SD_INIT_STATE_GET_CID:
            {
                /* Get CID */
                MMCSD_lld_initTransaction(&trans);
                trans.cmd = MMCSD_SD_CMD(2U);
                status = MMCSD_lld_transferPoll(handle, &trans);
                if(status == MMCSD_STS_SUCCESS)
                {
                    /* Purse the CID Register Content */
                    status = MMCSD_parseCIDSd(
                        (MMCSD_SdDeviceData *)object->initHandle->deviceData,
                        trans.response);
                }

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* Change State to get RCA */
                    currState = MMCSD_SD_INIT_STATE_GET_RCA;
                }
            }
            break;

            case MMCSD_SD_INIT_STATE_GET_RCA:
            {
                MMCSD_lld_initTransaction(&trans);
                trans.cmd = MMCSD_SD_CMD(3U);
                status = MMCSD_lld_transferPoll(handle, &trans);

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* RCA is the most significant 16 bits */
                    sdDeviceData->rca = ((trans.response[0] >> 16U) & 0xFFFFU);

                    currState = MMCSD_SD_INIT_STATE_GET_CSD;
                }
            }
            break;

            case MMCSD_SD_INIT_STATE_GET_CSD:
            {
                MMCSD_lld_initTransaction(&trans);
                trans.cmd = MMCSD_SD_CMD(9U);
                trans.arg = (sdDeviceData->rca << 16U);
                status = MMCSD_lld_transferPoll(handle, &trans);

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* Purse the CSD of the Device */
                    status = MMCSD_parseCSDSd(sdDeviceData, trans.response);
                }

                if(status == MMCSD_STS_SUCCESS)
                {
                    currState = MMCSD_SD_INIT_STATE_CHECK_LOCKED;
                }
            }
            break;

            case MMCSD_SD_INIT_STATE_CHECK_LOCKED:
            {
                /* Select Card (CMD7) */
                MMCSD_lld_initTransaction(&trans);
                trans.cmd = MMCSD_SD_CMD(7U);
                trans.arg = (sdDeviceData->rca << 16U);

                status = MMCSD_lld_transferPoll(handle, &trans);

                if(SystemP_SUCCESS == status)
                {
                    status = MMCSD_waitDAat0inBusy(
                                            object->initHandle->ctrlBaseAddr,
                                            MMCSD_WAIT_FOREVER);
                }

                if(status == MMCSD_STS_SUCCESS)
                {
                    if(((trans.response[0] >> 24) & 0x01U) != 0U)
                    {
                        /* Card is locked */
                        currState = MMCSD_SD_INIT_STATE_UNLOCK;
                    }
                    else
                    {
                        currState = MMCSD_SD_INIT_STATE_GET_SCR;
                    }

                    if((((sdDeviceData->ocr) >> 30U) & 0x01U) != 0x01U)
                    {
                        /* Card not High Capacity */
                        /* Block Length must be set manually by CMD 16 */
                        MMCSD_lld_initTransaction(&trans);
                        trans.cmd = MMCSD_SD_CMD(16U);
                        trans.arg = 512U;
                        status = MMCSD_lld_transferPoll(handle, &trans);
                    }
                }
            }
            break;

            case MMCSD_SD_INIT_STATE_UNLOCK:
            {
                /* Card unlock Procedure */
                /* TODO */
                status = MMCSD_STS_ERR_CARD_UNLOCK_FAIL;
            }
            break;

            case MMCSD_SD_INIT_STATE_GET_SCR:
            {
                MMCSD_lld_initTransaction(&trans);
                trans.cmd = MMCSD_SD_CMD(55U);
                trans.arg = (sdDeviceData->rca << 16U);
                status = MMCSD_lld_transferPoll(handle, &trans);

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* Get the SCR register */
                    /* At this stage the card is still in SDR12 speed (25MHz) */
                    /* At this point the Data Width is 1 bit */
                    MMCSD_lld_initTransaction(&trans);
                    trans.cmd = MMCSD_SD_ACMD(51U);
                    trans.dir = MMCSD_CMD_XFER_TYPE_READ;
                    trans.blockCount = 1U;
                    trans.blockSize = 8U;
                    trans.dataBuf = object->initHandle->dataBuf;
                    status = MMCSD_lld_transferPoll(handle, &trans);
                }
                if(status == MMCSD_STS_SUCCESS)
                {
                    /* Parse Data SCR Reg */
                    /* 5.6 SCR Register (V3.01 Physical Layer) */
                    status = MMCSD_parseSCRSd(sdDeviceData,
                                              object->initHandle->dataBuf);
                }

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* Check if bus Width switch is possible or not */
                    if(sdDeviceData->supportedDataWidths == 0x05U)
                    {
                        currState = MMCSD_SD_INIT_STATE_BUS_WIDTH_SWITCH;
                    }
                    else
                    {
                        /* Bus Width switch not Possible */
                        /* Bus Width stays in 1 Bit Mode */
                        initDone = true;
                    }
                }
            }
            break;

            case MMCSD_SD_INIT_STATE_BUS_WIDTH_SWITCH:
            {
                MMCSD_lld_initTransaction(&trans);
                trans.cmd = MMCSD_SD_CMD(55U);
                trans.arg = (sdDeviceData->rca << 16U);
                status = MMCSD_lld_transferPoll(handle, &trans);

                if(status == MMCSD_STS_SUCCESS)
                {
                    MMCSD_lld_initTransaction(&trans);
                    trans.cmd = MMCSD_SD_ACMD(6U);
                    uint32_t width = 0U;
                    if(object->initHandle->busWidth == MMCSD_BUS_WIDTH_4BIT)
                    {
                        width = MMCSD_BUS_WIDTH_4BIT;
                    }
                    else
                    {
                        width = MMCSD_BUS_WIDTH_1BIT;
                    }
                    trans.arg = width >> 1U;
                    status = MMCSD_lld_transferPoll(handle, &trans);

                    if(status == MMCSD_STS_SUCCESS)
                    {
                        MMCSD_setBusWidth(object->initHandle->ctrlBaseAddr,
                                          width);
                    }

                    if(sdDeviceData->specVersion != 0U)
                    {
                        /* Spec Version supports CMD6 */
                        currState = MMCSD_SD_INIT_STATE_SELECT_BUS_SPEED_MODE;
                    }
                    else
                    {
                        /* Initialization done */
                        initDone = true;
                    }
                }
            }
            break;

            case MMCSD_SD_INIT_STATE_SELECT_BUS_SPEED_MODE:
            {
                if(object->initHandle->autoAssignMaxSpeed)
                {
                    speedModeUHS = MMCSD_UHS_MODE_SDR104;
                    speedSet = false;
                    setSpeed = MMCSD_UHS_MODE_SDR104;
                    while(!speedSet)
                    {
                        /* Check if speed is supported by HOST */
                        if(MMCSD_isUHSSpeedSupportedHOST(
                                        object->initHandle->ctrlBaseAddr,
                                        speedModeUHS))
                        {
                            if(MMCSD_isUHSSpeedSupportedSD(handle,
                                                           speedModeUHS))
                            {
                                speedSet = true;
                                setSpeed = speedModeUHS;
                            }
                            else
                            {
                                if(speedModeUHS == MMCSD_UHS_MODE_SDR104)
                                {
                                    speedModeUHS = MMCSD_UHS_MODE_DDR50;
                                }
                                else if(speedModeUHS == MMCSD_UHS_MODE_DDR50)
                                {
                                    speedModeUHS = MMCSD_UHS_MODE_SDR25;
                                }
                                else if(speedModeUHS == MMCSD_UHS_MODE_SDR25)
                                {
                                    speedModeUHS = MMCSD_UHS_MODE_SDR12;
                                    speedSet = true;
                                }
                                else
                                {
                                    /* No Code */
                                }
                            }
                        }
                        else
                        {
                            /* Speed not supported by Host */
                            if(speedModeUHS == MMCSD_UHS_MODE_SDR104)
                            {
                                speedModeUHS = MMCSD_UHS_MODE_DDR50;
                            }
                            else if(speedModeUHS == MMCSD_UHS_MODE_DDR50)
                            {
                                speedModeUHS = MMCSD_UHS_MODE_SDR25;
                            }
                            else if(speedModeUHS == MMCSD_UHS_MODE_SDR25)
                            {
                                speedModeUHS = MMCSD_UHS_MODE_SDR12;
                            }
                            else
                            {
                                /* No Code */
                            }
                        }
                    }
                }
                else
                {
                    /* Manual speed setting */
                    switch(object->initHandle->uaBusSpeed)
                    {
                        case MMCSD_SD_MODE_HS:
                        case MMCSD_SD_MODE_SDR12:
                        {
                            speedModeUHS = MMCSD_UHS_MODE_SDR12;
                        }
                        break;

                        case MMCSD_SD_MODE_DS:
                        case MMCSD_SD_MODE_SDR25:
                        {
                            speedModeUHS = MMCSD_UHS_MODE_SDR25;
                        }
                        break;

                        case MMCSD_SD_MODE_SDR50:
                        {
                            speedModeUHS = MMCSD_UHS_MODE_SDR50;
                        }
                        break;

                        case MMCSD_SD_MODE_DDR50:
                        {
                            speedModeUHS = MMCSD_UHS_MODE_DDR50;
                        }
                        break;

                        case MMCSD_SD_MODE_SDR104:
                        {
                            speedModeUHS = MMCSD_UHS_MODE_SDR104;
                        }
                        break;

                        default:
                        break;
                    }

                    if(MMCSD_isUHSSpeedSupportedHOST(
                                        object->initHandle->ctrlBaseAddr,
                                        speedModeUHS))
                    {
                        if(MMCSD_isUHSSpeedSupportedSD(handle,
                                                       speedModeUHS))
                        {
                            setSpeed = speedModeUHS;
                        }
                        else
                        {
                            setSpeed = MMCSD_UHS_MODE_SDR12;
                        }
                    }
                    else
                    {
                        setSpeed = MMCSD_UHS_MODE_SDR12;
                    }
                }

                currState = MMCSD_SD_INIT_STATE_DRIVE_STRENGTH;
            }
            break;

            case MMCSD_SD_INIT_STATE_DRIVE_STRENGTH:
            {
                setDriverStrength = MMCSD_CMD6_DRIVE_STRENGTH_TYPE_D;
                drvStrengthSet = false;
                driverStrength = MMCSD_CMD6_DRIVE_STRENGTH_TYPE_D;

                if(object->initHandle->autoAssignMaxSpeed)
                {
                    while(!drvStrengthSet)
                    {
                        /* Check if speed is supported by HOST */
                        if(MMCSD_isUHSDriveStrengthSupportedHOST(
                                        object->initHandle->ctrlBaseAddr,
                                        driverStrength))
                        {
                            if(MMCSD_isUHSDrvStrengthSupportedSD(handle,
                                                           driverStrength))
                            {
                                drvStrengthSet = true;
                                setDriverStrength = driverStrength;
                            }
                            else
                            {
                                /* Driver Strength not supported by Device */
                                if(driverStrength ==
                                            MMCSD_CMD6_DRIVE_STRENGTH_TYPE_D)
                                {
                                    driverStrength =
                                            MMCSD_CMD6_DRIVE_STRENGTH_TYPE_C;
                                }
                                else if(driverStrength ==
                                            MMCSD_CMD6_DRIVE_STRENGTH_TYPE_C)
                                {
                                    driverStrength =
                                            MMCSD_CMD6_DRIVE_STRENGTH_TYPE_A;
                                }
                                else if(driverStrength ==
                                            MMCSD_CMD6_DRIVE_STRENGTH_TYPE_A)
                                {
                                    driverStrength =
                                            MMCSD_CMD6_DRIVE_STRENGTH_TYPE_B;
                                }
                                else
                                {
                                    /* No Code */
                                }

                            }
                        }
                        else
                        {
                            /* Driver strength not supported by host */
                            /* Chenge Driver Strength */
                            if(driverStrength ==
                                            MMCSD_CMD6_DRIVE_STRENGTH_TYPE_D)
                            {
                                driverStrength =
                                            MMCSD_CMD6_DRIVE_STRENGTH_TYPE_C;
                            }
                            if(driverStrength ==
                                            MMCSD_CMD6_DRIVE_STRENGTH_TYPE_C)
                            {
                                driverStrength =
                                            MMCSD_CMD6_DRIVE_STRENGTH_TYPE_A;
                            }
                            if(driverStrength ==
                                            MMCSD_CMD6_DRIVE_STRENGTH_TYPE_A)
                            {
                                driverStrength =
                                            MMCSD_CMD6_DRIVE_STRENGTH_TYPE_B;
                            }
                        }
                    }

                    /* Driver Strength is selected */
                    if(setDriverStrength != MMCSD_CMD6_DRIVE_STRENGTH_TYPE_B)
                    {
                        /* Driver Strength is not the default */
                        /* Do the switch using CMD 6 */
                        /* Do the switch on host side */
                    }
                }
                else
                {
                    /* Manual drive strength TODO */
                }

                currState = MMCSD_SD_INIT_STATE_CURRENT_LIMIT;
            }
            break;

            case MMCSD_SD_INIT_STATE_CURRENT_LIMIT:
            {
                currState = MMCSD_SD_INIT_STATE_SWITCH_BUS_SPEED_MODE;
            }
            break;

            case MMCSD_SD_INIT_STATE_SWITCH_BUS_SPEED_MODE:
            {
                /* Switch Bus Speed of SD Card with CMD 6 */
                status = MMCSD_switchBusSpeedSD(handle, setSpeed);

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* Set UHS Mode */
                    if(!(object->initHandle->autoAssignMaxSpeed) &&
                    ((object->initHandle->uaBusSpeed != MMCSD_SD_MODE_DS) &&
                    (object->initHandle->uaBusSpeed != MMCSD_SD_MODE_HS)))
                    {
                        MMCSD_setUHSMode(object->initHandle->ctrlBaseAddr,
                                         setSpeed);
                    }

                    /* Switch Bus Speed on Host Side */
                    switch(setSpeed)
                    {
                        case MMCSD_UHS_MODE_SDR12:
                        {
                            status = MMCSD_setClockFreqSD(handle,
                                                MMCSD_SD_SDR12_FREQUENCY_HZ);
                        }
                        break;

                        case MMCSD_UHS_MODE_SDR25:
                        {
                            status = MMCSD_setClockFreqSD(handle,
                                                MMCSD_SD_SDR25_FREQUENCY_HZ);
                        }
                        break;

                        case MMCSD_UHS_MODE_SDR50:
                        {
                            status = MMCSD_setClockFreqSD(handle,
                                                MMCSD_SD_SDR50_FREQUENCY_HZ);
                            tuningRequired = true;
                        }
                        break;

                        case MMCSD_UHS_MODE_SDR104:
                        {
                            status = MMCSD_setClockFreqSD(handle,
                                                MMCSD_SD_SDR104_FREQUENCY_HZ);
                            tuningRequired = true;
                        }
                        break;

                        case MMCSD_UHS_MODE_DDR50:
                        {
                            status = MMCSD_setClockFreqSD(handle,
                                                MMCSD_SD_DDR50_FREQUENCY_HZ);
                        }
                        break;

                        default:
                        {
                            status = MMCSD_setClockFreqSD(handle,
                                                MMCSD_SD_SDR12_FREQUENCY_HZ);
                        }
                        break;
                    }
                }

                if( (object->initHandle->uaBusSpeed == MMCSD_SD_MODE_DS) ||
                    (object->initHandle->uaBusSpeed == MMCSD_SD_MODE_HS))
                {
                    object->setBusSpeed = object->initHandle->uaBusSpeed;
                }
                else
                {
                    object->setBusSpeed = setSpeed;
                }


                /* Check if tuning required */
                if(tuningRequired == true)
                {
                    currState = MMCSD_SD_INIT_STATE_TUNING;
                }
                else
                {
                    initDone = true;
                }
            }
            break;

            case MMCSD_SD_INIT_STATE_TUNING:
            {
                if(object->initHandle->tuningType == MMCSD_PHY_TUNING_TYPE_AUTO)
                {
                    status = MMCSD_phyTuneAuto(handle);
                }
                else
                {
                    status = MMCSD_phyTuneManual(handle);
                }

                initDone = true;
            }
            break;

            default:
            break;
        }
    }

    return status;
}

static int32_t MMCSD_lld_initMMC(MMCSDLLD_Handle handle)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object         *object = (MMCSDLLD_Object*)handle;
    MMCSD_EmmcDeviceData    *mmcDeviceData = (MMCSD_EmmcDeviceData *)NULL;
    MMCSDLLD_Transaction    trans;
    uint32_t                hostOCR = 0U;
    uint32_t                retry = 0xFFFFU;
    uint32_t                speedMode = 0U;
    uint32_t                setSpeed = 0U;
    uint32_t                ecsdBusWidth = 0U;
    uint32_t                setBusWidth = 0U;
    uint32_t                currState = MMCSD_MMC_INIT_STATE_CONTROLLER_INIT;
    bool                    initDone = false;
    bool                    speedSet = false;
    bool                    v_2V7_3V6_Supp = false;

    mmcDeviceData = (MMCSD_EmmcDeviceData *)object->initHandle->deviceData;

    while((initDone == false) && (status == MMCSD_STS_SUCCESS))
    {
        switch(currState)
        {
            case MMCSD_MMC_INIT_STATE_CONTROLLER_INIT:
            {
                /* Do a soft reset of IP */
                status = MMCSD_softReset(object->initHandle->ctrlBaseAddr,
                                        MMCSD_WAIT_FOREVER);

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* Set bus width to 1 bit */
                    MMCSD_setBusWidth(object->initHandle->ctrlBaseAddr,
                                      MMCSD_BUS_WIDTH_1BIT);

                    /* Set Bus voltage to 1.8V for MMCSD0 */
                    MMCSD_setBusVolt(object->initHandle->ctrlBaseAddr,
                                     MMCSD_BUS_VOLT_1_8V);

                    /* Set the slot type to user given type in CTL_CFG_2_REG */
                    MMCSD_setSlotType(  object->initHandle->ssBaseAddr,
                                        (uint8_t)object->initHandle->slotType);

                    /* Wait for card detect */
                    status = MMCSD_waitCardDetect(handle, MMCSD_WAIT_FOREVER);
                }

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* Initialize the PHY Type */
                    status = MMCSD_phyInit( object->initHandle->ssBaseAddr,
                                            object->initHandle->phyType);
                }

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* Set bus frequency for initialization
                    * In identification mode the clock frequency can be maximum
                    * 400 KHz for EMMC cards with backword Comapatibility */
                    status = MMCSD_setBusFreq(object->initHandle->ctrlBaseAddr,
                                              object->initHandle->inputClkFreq,
                                              MMCSD_ID_MODE_FREQUENCY_HZ);
                }

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* Turn on on Bus Power */
                    status = MMCSD_busPowerOnCtrl(
                                        object->initHandle->ctrlBaseAddr,
                                        true, MMCSD_WAIT_FOREVER);
                }

                if(status == MMCSD_STS_SUCCESS)
                {
                    currState = MMCSD_MMC_INIT_STATE_CHECK_VOLTAGE;
                }
            }
            break;

            case MMCSD_MMC_INIT_STATE_CHECK_VOLTAGE:
            {
                /* CMD0 - reset card */
                MMCSD_lld_initTransaction(&trans);
                trans.cmd   = MMCSD_MMC_CMD(0U);
                trans.arg   = 0x0U;
                status = MMCSD_lld_transferPoll(handle, &trans);

                /* Sleep for 5ms for input clock frequency, as mentioned
                * in JEDEC standard JESD84-B51 section 10.1
                */
                object->initHandle->Clock_uSleep(5000);

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* Check whether any 2.7 V to 3.6 Volt mode is supported by IP or not */
                    if( (MMCSD_is3V3Supported(object->initHandle->ctrlBaseAddr)) ||
                        (MMCSD_is3V0Supported(object->initHandle->ctrlBaseAddr)))
                    {
                        v_2V7_3V6_Supp = true;
                    }

                    hostOCR = MMCSD_lld_makeOCRMMC(true, true, v_2V7_3V6_Supp,
                                                   true);

                    /* Get OCR Register */
                    MMCSD_lld_initTransaction(&trans);
                    trans.cmd = MMCSD_MMC_CMD(1U);
                    trans.arg = hostOCR;
                    status = MMCSD_lld_transferPoll(handle,&trans);

                    /* Store OCR of Card in EMMC Data Object */
                    mmcDeviceData->ocr = trans.response[0];
                    if(((mmcDeviceData->ocr >> 7) & 0x01U) == 0U)
                    {
                        /* 1.8V not supported */
                        status = MMCSD_STS_ERR;
                    }

                    if(status == MMCSD_STS_SUCCESS)
                    {
                        while(  (((trans.response[0] >> 31U) & (uint32_t)0x01) == 0U) &&
                                (retry != 0U))
                        {
                            status = MMCSD_lld_transferPoll(handle, &trans);
                            retry--;
                            if(retry == 0U)
                            {
                                status = MMCSD_STS_ERR;
                            }
                        }
                    }

                    if(status == MMCSD_STS_SUCCESS)
                    {
                        /* Store Valid OCR of Card in EMMC Data Object */
                        mmcDeviceData->ocr = trans.response[0];
                        /* Check if card is high Capacity or not */
                        if((mmcDeviceData->ocr & ((uint32_t)0x1U << 30U)) != 0U)
                        {
                            mmcDeviceData->isHC = true;
                        }
                        else
                        {
                            mmcDeviceData->isHC = false;
                        }
                    }
                }
                /* Go to next state, incase of error loop will break */
                currState = MMCSD_MMC_INIT_STATE_GET_CID;
            }
            break;

            case MMCSD_MMC_INIT_STATE_GET_CID:
            {
                MMCSD_lld_initTransaction(&trans);
                trans.cmd = MMCSD_MMC_CMD(2U);
                trans.arg = 0U;
                status = MMCSD_lld_transferPoll(handle, &trans);

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* Purse CID and store */
                    (void)MMCSD_parseCIDEmmc(mmcDeviceData, (trans.response));
                    /* Set RCA (Relative Card Address) */
                    MMCSD_lld_initTransaction(&trans);
                    mmcDeviceData->rca = MMCSD_MMC_DEFAULT_RCA;
                    trans.cmd = MMCSD_MMC_CMD(3U);
                    trans.arg = ((mmcDeviceData->rca) << 16U);
                    status = MMCSD_lld_transferPoll(handle, &trans);
                }
                /* After Command 3 the device goes to idle mode */
                currState = MMCSD_MMC_INIT_STATE_GET_CSD;
            }
            break;

            case MMCSD_MMC_INIT_STATE_GET_CSD:
            {
                /* Send CMD9 with RCA as argument to get the CSD
                   (Card Specific Data) */
                MMCSD_lld_initTransaction(&trans);
                trans.cmd = MMCSD_MMC_CMD(9U);
                trans.arg = ((mmcDeviceData->rca) << 16U);
                status = MMCSD_lld_transferPoll(handle, &trans);

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* Purse CSD and store */
                    (void)MMCSD_parseCSDEmmc(mmcDeviceData, (trans.response));
                }

                currState = MMCSD_MMC_INIT_STATE_VERSION_CHECK;
            }
            break;

            case MMCSD_MMC_INIT_STATE_VERSION_CHECK:
            {
                /* Check Spec Verison */
                /* Provide support for only Supported version */
                if(mmcDeviceData->specVersion != 0x04U)
                {
                    status = MMCSD_STS_ERR;
                }

                currState = MMCSD_MMC_INIT_STATE_GET_ECSD;
            }
            break;

            case MMCSD_MMC_INIT_STATE_GET_ECSD:
            {
                MMCSD_lld_initTransaction(&trans);
                trans.cmd = MMCSD_MMC_CMD(7U);
                trans.arg = ((mmcDeviceData->rca) << 16U);
                status = MMCSD_lld_transferPoll(handle, &trans);

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* Send CMD8 with RCA as argument to get the ECSD */
                    MMCSD_lld_initTransaction(&trans);
                    trans.cmd = MMCSD_MMC_CMD(8U);
                    trans.dir = MMCSD_CMD_XFER_TYPE_READ;
                    trans.arg = ((mmcDeviceData->rca) << 16U);
                    trans.blockCount = 1U;
                    trans.blockSize = 512U;
                    trans.dataBuf = object->initHandle->dataBuf;
                    status = MMCSD_lld_transferPoll(handle, &trans);
                }
                if(status == MMCSD_STS_SUCCESS)
                {
                    (void)MMCSD_parseECSDEmmc(mmcDeviceData,
                                              object->initHandle->dataBuf);
                }

                currState = MMCSD_MMC_INIT_STATE_SELECT_BUS_SPEED;
            }
            break;

            case MMCSD_MMC_INIT_STATE_SELECT_BUS_SPEED:
            {
                if(object->initHandle->autoAssignMaxSpeed)
                {
                    /* Auto assign max possible bus speed */
                    speedMode = MMCSD_MMC_MODE_HS200;
                    speedSet = false;
                    setSpeed = MMCSD_MMC_MODE_SDR25;
                    while(!speedSet)
                    {
                        /* Check if speed is supported by MMC */
                        if(MMCSD_isSpeedSupportedMMC(handle, speedMode))
                        {
                            speedSet = true;
                            setSpeed = speedMode;
                        }
                        else
                        {
                            if(speedMode == MMCSD_MMC_MODE_HS200)
                            {
                                speedMode = MMCSD_MMC_MODE_SDR50;
                            }
                            else if(speedMode == MMCSD_MMC_MODE_SDR50)
                            {
                                speedMode = MMCSD_MMC_MODE_SDR25;
                                speedSet = true;
                            }
                            else
                            {
                                /* No Code */
                            }
                        }
                    }
                }
                else
                {
                    /* Manually assign bus speed */
                    speedMode = object->initHandle->uaBusSpeed;
                    if(MMCSD_isSpeedSupportedMMC(handle, speedMode))
                    {
                        setSpeed = speedMode;
                    }
                    else
                    {
                        setSpeed = MMCSD_MMC_MODE_SDR25;
                    }

                    object->setBusSpeed = setSpeed;
                }

                currState = MMCSD_MMC_INIT_STATE_BUS_WIDTH_SWITCH;
            }
            break;

            case MMCSD_MMC_INIT_STATE_BUS_WIDTH_SWITCH:
            {
                switch(setSpeed)
                {
                    case MMCSD_MMC_MODE_SDR25:
                    case MMCSD_MMC_MODE_SDR50:
                    {
                        switch(object->initHandle->busWidth)
                        {
                            case MMCSD_BUS_WIDTH_8BIT:
                            {
                                ecsdBusWidth = MMCSD_ECSD_BUS_WIDTH_8BIT;
                                setBusWidth = MMCSD_BUS_WIDTH_8BIT;
                            }
                            break;
                            case MMCSD_BUS_WIDTH_4BIT:
                            {
                                ecsdBusWidth = MMCSD_ECSD_BUS_WIDTH_4BIT;
                                setBusWidth = MMCSD_BUS_WIDTH_4BIT;
                            }
                            break;
                            case MMCSD_BUS_WIDTH_1BIT:
                            {
                                ecsdBusWidth = MMCSD_ECSD_BUS_WIDTH_1BIT;
                                setBusWidth = MMCSD_BUS_WIDTH_1BIT;
                            }
                            break;
                            default:
                            {

                            }
                            break;
                        }
                    }
                    break;

                    case MMCSD_MMC_MODE_HS200:
                    {
                        switch(object->initHandle->busWidth)
                        {
                            case MMCSD_BUS_WIDTH_8BIT:
                            {
                                ecsdBusWidth = MMCSD_ECSD_BUS_WIDTH_8BIT;
                                setBusWidth = MMCSD_BUS_WIDTH_8BIT;
                            }
                            break;
                            case MMCSD_BUS_WIDTH_4BIT:
                            {
                                ecsdBusWidth = MMCSD_ECSD_BUS_WIDTH_4BIT;
                                setBusWidth = MMCSD_BUS_WIDTH_4BIT;
                            }
                            break;
                            default:
                            {
                                ecsdBusWidth = MMCSD_ECSD_BUS_WIDTH_4BIT;
                                setBusWidth = MMCSD_BUS_WIDTH_4BIT;
                            }
                            break;
                        }
                    }
                    break;

                    default:
                    break;
                }

                object->setBusWidth = setBusWidth;

                /* Send CMD6(Switch Command) with argument to
                    change buswidth of card (Write ECSD REG) */
                MMCSD_lld_initTransaction(&trans);
                trans.cmd = MMCSD_MMC_CMD(6U);
                trans.arg = (MMCSD_CMD6_ACCESS_MASK_WRITE_BYTE << 24U)   |
                            (MMCSD_ECSD_BUS_WIDTH_INDEX << 16U)          |
                            ((  (uint32_t)(0x00U) |
                                (uint32_t)ecsdBusWidth) << 8U);

                status = MMCSD_lld_transferPoll(handle, &trans);

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* Change bus width of host Controller */
                    MMCSD_setBusWidth(object->initHandle->ctrlBaseAddr,
                                      object->initHandle->busWidth);

                    /* Check if card is ready for transfer with the
                       new bus width */
                    status = MMCSD_isCardReadyForTransferMMC(handle);
                }

                currState = MMCSD_MMC_INIT_STATE_SWITCH_BUS_SPEED_MODE;

            }
            break;

            case MMCSD_MMC_INIT_STATE_SWITCH_BUS_SPEED_MODE:
            {
                status = MMCSD_switchBusSpeedEMMC(handle, setSpeed);
                initDone = true;
            }
            break;

            default:
            break;
        }
    }

    return status;
}

static int32_t MMCSD_waitCardDetect(MMCSDLLD_Handle handle,
                                    uint32_t loopTimeout)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object         *object = (MMCSDLLD_Object*)handle;
    uint32_t                timeout = loopTimeout;

    while ( (!MMCSD_isCardInserted(object->initHandle->ctrlBaseAddr)) &&
            (timeout != 0U))
    {
        timeout--;
    }
    if (timeout == 0U){
        status = MMCSD_STS_ERR_CARD_NOT_FOUND;
    }

    return status;
}

static void MMCSD_intrConfigReset(MMCSDLLD_Handle handle)
{
    MMCSDLLD_Object         *object = (MMCSDLLD_Object*)handle;

    /* Disable all Normal Interrupt Statuses */
    MMCSD_disableNormIntrStat(object->initHandle->ctrlBaseAddr,
                                MMCSD_INTERRUPT_ALL_NORMAL);
    /* Disable all Error Interrupt Statuses */
    MMCSD_disableErrIntrStat(object->initHandle->ctrlBaseAddr,
                            MMCSD_INTERRUPT_ALL_ERROR);
    /* Disable all Normal signals to interrupt */
    MMCSD_disableNormSigIntr(object->initHandle->ctrlBaseAddr,
                            MMCSD_INTERRUPT_ALL_NORMAL);
    /* Disable all Error signals to interrupt */
    MMCSD_disableErrSigIntr(object->initHandle->ctrlBaseAddr,
                            MMCSD_INTERRUPT_ALL_ERROR);
    /* Clear all Normal Interrupt Statuses */
    MMCSD_clearNormIntrStat(object->initHandle->ctrlBaseAddr,
                            MMCSD_INTERRUPT_ALL_NORMAL);
    /* Clear all Error Interrupt Statuses */
    MMCSD_clearErrIntrStat(object->initHandle->ctrlBaseAddr,
                            MMCSD_INTERRUPT_ALL_ERROR);
}

static void MMCSD_lld_initTransaction(MMCSDLLD_Transaction *trans)
{
    *trans = MMCSD_lld_defaultTransaction;
}

static uint32_t MMCSD_lld_makeOCRMMC(bool busyBit, bool V_1V7_1V95,
                                     bool V_2V7_3V6, bool sectorMode)
{
    uint32_t                retValOCR = 0U;
    uint32_t                accessMode = 0U;
    uint32_t                vSupport_1V7_1V95 = 0U;
    uint32_t                busyBitState = 0U;
    uint32_t                vSupport_2V7_3V6 = 0U;

    if(busyBit)
    {
        busyBitState = 1U;
    }
    if(V_1V7_1V95)
    {
        vSupport_1V7_1V95 = 1U;
    }
    if(V_2V7_3V6)
    {
        vSupport_2V7_3V6 = 0x1FFU;
    }
    if(sectorMode)
    {
        accessMode = 2U;
    }

    retValOCR |=    (accessMode << 29U)         |   \
                    (vSupport_1V7_1V95 << 7U)   |   \
                    (busyBitState << 31U)       |   \
                    (accessMode << 29U)         |   \
                    (vSupport_2V7_3V6 << 15U);

    return retValOCR;
}

static int32_t MMCSD_isCardReadyForTransferMMC(MMCSDLLD_Handle handle)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    uint32_t                readyCheckTryCount = 0U;
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;
    MMCSD_EmmcDeviceData    *mmcDeviceData = (MMCSD_EmmcDeviceData *)NULL;
    MMCSDLLD_Transaction    trans;
    uint32_t                mediaCurrentState = 0U;

    mmcDeviceData = (MMCSD_EmmcDeviceData *)(object->initHandle->deviceData);

    while(  (mediaCurrentState != MMCSD_MEDIA_STATE_TRAN) &&
            (readyCheckTryCount < MMCSD_MEDIA_STATE_THRESHOLD))
    {
        if(mediaCurrentState == MMCSD_MEDIA_STATE_RCV)
        {
            MMCSD_lld_initTransaction(&trans);
            trans.cmd = MMCSD_MMC_CMD(12U);
            trans.arg = (0U);
            status = MMCSD_lld_transferPoll(handle, &trans);
        }

        if(status == MMCSD_STS_SUCCESS)
        {
            MMCSD_lld_initTransaction(&trans);
            trans.cmd = MMCSD_MMC_CMD(13U);
            trans.arg = (mmcDeviceData->rca << 16U);
            status = MMCSD_lld_transferPoll(handle, &trans);
            readyCheckTryCount++;
            mediaCurrentState = ((trans.response[0] >> 9U) & 0x0FU);
        }
    }

    if(mediaCurrentState != MMCSD_MEDIA_STATE_TRAN)
    {
        status = MMCSD_STS_ERR;
    }

    return status;
}

static int32_t MMCSD_isCardReadyForTransferSD(MMCSDLLD_Handle handle)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    uint32_t                readyCheckTryCount = 0U;
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;
    MMCSD_SdDeviceData      *sdDeviceData = (MMCSD_SdDeviceData *)NULL;
    MMCSDLLD_Transaction    trans;
    uint32_t                mediaCurrentState = 0U;

    sdDeviceData = (MMCSD_SdDeviceData *)(object->initHandle->deviceData);

    while(  (mediaCurrentState != MMCSD_MEDIA_STATE_TRAN) &&
            (readyCheckTryCount < MMCSD_MEDIA_STATE_THRESHOLD))
    {
        if(mediaCurrentState == MMCSD_MEDIA_STATE_RCV)
        {
            MMCSD_lld_initTransaction(&trans);
            trans.cmd = MMCSD_SD_CMD(12U);
            trans.arg = (0U);
            status = MMCSD_lld_transferPoll(handle, &trans);
        }

        if(status == MMCSD_STS_SUCCESS)
        {
            MMCSD_lld_initTransaction(&trans);
            trans.cmd = MMCSD_SD_CMD(13U);
            trans.arg = (sdDeviceData->rca << 16U);
            status = MMCSD_lld_transferPoll(handle, &trans);
            readyCheckTryCount++;
            mediaCurrentState = ((trans.response[0] >> 9U) & 0x0FU);
        }
    }

    if(mediaCurrentState != MMCSD_MEDIA_STATE_TRAN)
    {
        status = MMCSD_STS_ERR;
    }

    return status;
}

static int32_t MMCSD_switchBusSpeedEMMC(MMCSDLLD_Handle handle,
                                        uint32_t speedMode)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    uint32_t                clkFreq = 0U;
    bool                    tuningRequried = false;
    uint32_t                phyMode = MMCSD_PHY_MODE_DS;
    uint32_t                hsTimingVal = 0U;
    uint32_t                phyDriverType = 0U;
    uint8_t                 driveStrength = 0U;
    MMCSDLLD_Transaction    trans;
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;
    MMCSD_EmmcDeviceData    *mmcDeviceData = (MMCSD_EmmcDeviceData *)NULL;

    mmcDeviceData = (MMCSD_EmmcDeviceData *)(object->initHandle->deviceData);
    driveStrength = mmcDeviceData->driveStrength;

    switch(speedMode)
    {
        case MMCSD_MMC_MODE_SDR25:
        {
            clkFreq = 25000000U;
            tuningRequried = false;
            phyMode = MMCSD_PHY_MODE_DS;
            hsTimingVal = MMCSD_ECSD_HS_TIMING_BACKWARD_COMPATIBLE;
        }
        break;

        case MMCSD_MMC_MODE_SDR50:
        {
            clkFreq = 50000000U;
            tuningRequried = false;
            phyMode = MMCSD_PHY_MODE_HSSDR50;
            hsTimingVal = MMCSD_ECSD_HS_TIMING_HIGH_SPEED;
        }
        break;

        case MMCSD_MMC_MODE_HS200:
        {
            clkFreq = 200000000U;
            tuningRequried = true;
            phyMode = MMCSD_PHY_MODE_HS200;
            hsTimingVal = MMCSD_ECSD_HS_TIMING_HS200;
        }
        break;

        default:
        break;
    }

    /* Change Bus Speed Mode with CMD 6 on Device Side */
    MMCSD_lld_initTransaction(&trans);
    trans.cmd = MMCSD_MMC_CMD(6U);
    trans.arg = (MMCSD_CMD6_ACCESS_MASK_WRITE_BYTE << 24U)   |
                (MMCSD_ECSD_HS_TIMING_INDEX << 16U)          |
                ((((uint32_t)driveStrength << 4U) | hsTimingVal) << 8U);

    status = MMCSD_lld_transferPoll(handle, &trans);

    /* Check DAT0 line status */
    if(status == MMCSD_STS_SUCCESS)
    {
        /* Wait for DAT0IN to go hight again */
        status = MMCSD_waitDAat0inBusy(object->initHandle->ctrlBaseAddr,
                                       MMCSD_WAIT_FOREVER);
    }

    if(status == MMCSD_STS_SUCCESS)
    {
        /* Disable PHY DLL */
        MMCSD_phyDisableDLL(object->initHandle->ssBaseAddr);
        /* Set Bus Frequency */
        status = MMCSD_setBusFreq(object->initHandle->ctrlBaseAddr,
                                object->initHandle->inputClkFreq, clkFreq);
    }

    if(status == MMCSD_STS_SUCCESS)
    {
        /* Enable PHY DLL */
        (void)MMCSD_phyConfigure(object->initHandle->ssBaseAddr,
                                 phyMode, clkFreq, phyDriverType);
        if(tuningRequried)
        {
            if(object->initHandle->tuningType == MMCSD_PHY_TUNING_TYPE_AUTO)
            {
                status = MMCSD_phyTuneAuto(handle);
            }
            else
            {
                status = MMCSD_phyTuneManual(handle);
            }
        }
        else
        {
            /* No code */
        }
    }

    return status;
}

static bool MMCSD_isUHSSpeedSupportedSD(MMCSDLLD_Handle handle,
                                        uint32_t speedMode)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;
    MMCSDLLD_Transaction    trans;
    bool                    retVal = false;

    /* Check if device supporst speed */
    MMCSD_lld_initTransaction(&trans);
    trans.cmd = MMCSD_SD_CMD(6U);
    trans.arg = ((MMCSD_CHECK_MODE & MMCSD_CMD6_GRP1_SEL) |
                (speedMode));
    trans.dir = MMCSD_CMD_XFER_TYPE_READ;
    trans.blockCount = 1U;
    trans.blockSize = 64U;
    trans.dataBuf = object->initHandle->dataBuf;
    status = MMCSD_lld_transferPoll(handle, &trans);

    if(status == MMCSD_STS_SUCCESS)
    {
        if(object->initHandle->dataBuf[16] ==
                                speedMode)
        {
            retVal = true;
        }
        else
        {
            /* No Code */
        }
    }
    else
    {
        /* No Code */
    }

    return retVal;
}

static bool MMCSD_isUHSDrvStrengthSupportedSD(MMCSDLLD_Handle handle,
                                              uint32_t driverStrength)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;
    MMCSDLLD_Transaction    trans;
    bool                    retVal = false;

    /* Check if device supporst speed */
    MMCSD_lld_initTransaction(&trans);
    trans.cmd = MMCSD_SD_CMD(6U);
    trans.arg = ((MMCSD_CHECK_MODE & MMCSD_CMD6_GRP3_SEL) |
                (driverStrength << 8U));
    trans.dir = MMCSD_CMD_XFER_TYPE_READ;
    trans.blockCount = 1U;
    trans.blockSize = 64U;
    trans.dataBuf = object->initHandle->dataBuf;
    status = MMCSD_lld_transferPoll(handle, &trans);

    if(status == MMCSD_STS_SUCCESS)
    {
        if(object->initHandle->dataBuf[15] ==
                                driverStrength)
        {
            retVal = true;
        }
        else
        {
            /* No Code */
        }
    }
    else
    {
        /* No Code */
    }

    return retVal;
}

static int32_t MMCSD_switchBusSpeedSD(MMCSDLLD_Handle handle,
                                      uint32_t setSpeed)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;
    MMCSDLLD_Transaction    trans;

    /* Check if device supporst speed */
    MMCSD_lld_initTransaction(&trans);
    trans.cmd = MMCSD_SD_CMD(6U);
    trans.arg = ((MMCSD_SWITCH_MODE & MMCSD_CMD6_GRP1_SEL) |
                (setSpeed));
    trans.dir = MMCSD_CMD_XFER_TYPE_READ;
    trans.blockCount = 1U;
    trans.blockSize = 64U;
    trans.dataBuf = object->initHandle->dataBuf;
    status = MMCSD_lld_transferPoll(handle, &trans);

    return status;
}

static int32_t MMCSD_lld_sendCmd23SD(MMCSDLLD_Handle handle, uint32_t numBlks)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Transaction    trans;

    MMCSD_lld_initTransaction(&trans);
    trans.cmd = MMCSD_SD_CMD(23U);
    trans.arg = numBlks;
    status = MMCSD_lld_transferPoll(handle, &trans);

    return status;
}

static int32_t MMCSD_lld_sendCmd23MMC(MMCSDLLD_Handle handle, uint32_t numBlks)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Transaction    trans;

    MMCSD_lld_initTransaction(&trans);
    trans.cmd = MMCSD_MMC_CMD(23U);
    trans.arg = numBlks;
    status = MMCSD_lld_transferPoll(handle, &trans);

    return status;
}

static bool MMCSD_isSpeedSupportedMMC(MMCSDLLD_Handle handle,
                                      uint32_t speedMode)
{
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;
    MMCSD_EmmcDeviceData    *mmcDeviceData = (MMCSD_EmmcDeviceData *)NULL;
    bool                    retVal = false;

    mmcDeviceData = (MMCSD_EmmcDeviceData *)object->initHandle->deviceData;

    switch(speedMode)
    {
        case MMCSD_MMC_MODE_SDR25:
        {
            if(((mmcDeviceData->supportedModes) &
                (MMCSD_EMMC_ECSD_DEVICE_TYPE_HS_26MHZ_MASK)) != 0U)
            {
                retVal = true;
            }
        }
        break;

        case MMCSD_MMC_MODE_SDR50:
        {
            if(((mmcDeviceData->supportedModes) &
                (MMCSD_EMMC_ECSD_DEVICE_TYPE_HS_52MHZ_MASK)) != 0U)
            {
                retVal = true;
            }
        }
        break;

        case MMCSD_MMC_MODE_HS200:
        {
            if(((mmcDeviceData->supportedModes) &
                (MMCSD_EMMC_ECSD_DEVICE_TYPE_HS200_200MHZ_1P8V_MASK)) != 0U)
            {
                retVal = true;
            }
        }
        break;

        default:
        break;
    }

    return retVal;
}


static void MMCSD_setupADMA2(MMCSDLLD_Handle handle,
                             MMCSD_ADMA2Descriptor *desc,
                             uint64_t bufAddr, uint32_t dataSize)
{
    MMCSDLLD_Object     *object = (MMCSDLLD_Object *)handle;
    uint32_t            dmaParams = 0U;

    /* Set Data Size */
    dmaParams = dataSize << 16U;
    dmaParams |= (((dataSize >> 16U) << 6U) | 0x0023U);
    /* Enable version 4 for 26 bit sizes */
    MMCSD_setVer40Enable(object->initHandle->ctrlBaseAddr);
    /* Set 26 bit size */
    MMCSD_setADMA2LenMode(object->initHandle->ctrlBaseAddr, true);
    /* Setup ADMA2 descriptor */
    desc->dmaParams = dmaParams;
    desc->addrLo    = (uint32_t)(bufAddr & 0xFFFFFFFFU);
    desc->addrHi    = (uint32_t)((bufAddr >> 32U) & 0xFFFFU);

    /* Set 32 bit ADMA2 */
    MMCSD_selectDma(object->initHandle->ctrlBaseAddr, 2U);
    /* Write the descriptor address to ADMA2 Address register */
    MMCSD_writeADMA2DescAddr(object->initHandle->ctrlBaseAddr, (uint64_t)desc);
    /* Invalidate Cache */
    CacheP_wbInv(desc, sizeof(MMCSD_ADMA2Descriptor), CacheP_TYPE_ALL);
}

static int32_t MMCSD_lld_transferPoll(MMCSDLLD_Handle handle,
                                      MMCSDLLD_Transaction *trans)
{
    int32_t             status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object     *object = (MMCSDLLD_Object*)handle;

    if(object->state == MMCSD_STATE_IDLE)
    {
        /* Set driver state to BUSY */
        object->state = MMCSD_STATE_BUSY;
        /* Reset interrupt configuration */
        MMCSD_intrConfigReset(handle);

        if((object->cmdErrorStat != 0U) || (object->xferErrorStat != 0U))
        {
            /* Some Error Happened in previous Transaction */
            (void)MMCSD_linesResetCmd(object->initHandle->ctrlBaseAddr,
                                        MMCSD_WAIT_FOREVER);
            (void)MMCSD_linesResetDat(object->initHandle->ctrlBaseAddr,
                                        MMCSD_WAIT_FOREVER);
        }
        /* Clear stored Error interrupt statuses */
        object->cmdErrorStat = (uint16_t)0U;
        object->xferErrorStat = (uint16_t)0U;

        if( (trans->cmd &
            (uint32_t)CSL_MMC_CTLCFG_COMMAND_DATA_PRESENT_MASK) != 0U)
        {
            status = MMCSD_lld_transferDataPoll(handle, trans);
        }
        else
        {
            status = MMCSD_lld_transferNoDataPoll(handle, trans);
        }

        /* Set driver state back to IDLE */
        object->state = MMCSD_STATE_IDLE;
    }
    else
    {
        status = MMCSD_STS_ERR_BUSY;
    }

    return status;
}

/* Command with Data Polling mode Transfer Function */
static int32_t MMCSD_lld_transferDataPoll(MMCSDLLD_Handle handle,
                                          MMCSDLLD_Transaction *trans)
{
    int32_t             status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object     *object = (MMCSDLLD_Object*)handle;
    uint32_t            dataSize = 0U;

    /* Store all the transaction data in driver object */
    object->dataBufIdx = trans->dataBuf;
    object->dataBlockCount = trans->blockCount;
    object->dataBlockSize = trans->blockSize;

    /* Check if the transfer is read type or write type default is write
     * If transfer is read type
        Enable buffer read Ready interrupt
     * If transfer is write type
        Enable buffer write Ready interrupt
    */
    if(trans->dir == MMCSD_CMD_XFER_TYPE_READ)
    {
        MMCSD_enableNormIntrStat(object->initHandle->ctrlBaseAddr,
                        CSL_MMC_CTLCFG_NORMAL_INTR_STS_ENA_BUF_RD_READY_MASK);
    }
    else
    {
        MMCSD_enableNormIntrStat(object->initHandle->ctrlBaseAddr,
                        CSL_MMC_CTLCFG_NORMAL_INTR_STS_ENA_BUF_WR_READY_MASK);
    }

    /* Enable Transfer Complete normal interrupt */
    /* Enable Command Complete normal interrupt */
    MMCSD_enableNormIntrStat(object->initHandle->ctrlBaseAddr,
                CSL_MMC_CTLCFG_NORMAL_INTR_STS_ENA_CMD_COMPLETE_MASK |
                CSL_MMC_CTLCFG_NORMAL_INTR_STS_ENA_XFER_COMPLETE_MASK);
    /* Enable all Error interrupts */
    MMCSD_enableErrIntrStat(object->initHandle->ctrlBaseAddr,
                            MMCSD_INTERRUPT_ALL_ERROR);

    /* Write back invalidate the write buffer idx in cache */
    CacheP_wbInv(object->dataBufIdx, (trans->blockSize * trans->blockCount),
                 CacheP_TYPE_ALL);

    /* If dma is enabled
     * Enable ADMA2
     */
    if((trans->enableDma == TRUE) && (trans->isTuning == FALSE))
    {
        /* Setup ADMA2 descriptor */
        dataSize = ((trans->blockCount) * (trans->blockSize));
        /* Set up EDMA */
        MMCSD_setupADMA2(handle, &gADMA2Desc, (uint64_t)trans->dataBuf,
                         dataSize);
        object->remainingBlockCount = 0U;
    }
    else
    {
        /* No Code */
        /* Set the remaining block count as it will be used for keeping count */
        object->remainingBlockCount = object->dataBlockCount;
    }

    /* Wait for the CMD inhibit line to go low (Command can be issued) */
    (void)MMCSD_waitCommandInhibit(object->initHandle->ctrlBaseAddr,
                                   MMCSD_WAIT_FOREVER);
    /* Wait for the DATA inhibit line to go low (Data can be transacted) */
    (void)MMCSD_waitDataInhibit(object->initHandle->ctrlBaseAddr,
                                MMCSD_WAIT_FOREVER);
    /* Send command */
    MMCSD_sendCommand(object->initHandle->ctrlBaseAddr, trans);
    /* Poll until Command complete or command Error or command timeout */
    if(trans->isTuning == TRUE)
    {
        /* No need to wait for command completion */
    }
    else
    {
        MMCSD_lld_cmdCompleteStatusPoll(handle);
    }

    if(object->cmdErrorStat == 0U)
    {
        /* Get and store response in transaction object */
        status = MMCSD_getCmdResponse(object->initHandle->ctrlBaseAddr,
                                      trans->response);
    }
    else
    {
        /* Some Error Happened */
        status = MMCSD_STS_ERR;
    }

    if(status == MMCSD_STS_SUCCESS)
    {
        if (trans->isTuning == TRUE)
        {
            /* Poll until check pattern received from device */
            MMCSD_lld_xferCompleteStatusPollCMD19(handle);
        }
        else
        {
            /* Poll until data transfer Completes */
            MMCSD_lld_xferCompleteStatusPoll(handle);
        }

        if(object->xferErrorStat != 0U)
        {
            /* Some Error happened */
            status = MMCSD_STS_ERR;
        }
        else
        {
            /* No Error happened */
        }
    }

    return status;
}

/* Command without Data Polling mode transfer function */
static int32_t MMCSD_lld_transferNoDataPoll(MMCSDLLD_Handle handle,
                                            MMCSDLLD_Transaction *trans)
{
    int32_t             status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object     *object = (MMCSDLLD_Object*)handle;

    /* Enable Command Complete Interrupt Status */
    MMCSD_enableNormIntrStat(object->initHandle->ctrlBaseAddr,
                             CSL_MMC_CTLCFG_NORMAL_INTR_STS_CMD_COMPLETE_MASK);
    /* Enable Command Timeout Interrupt Status */
    MMCSD_enableErrIntrStat(object->initHandle->ctrlBaseAddr,
                             CSL_MMC_CTLCFG_ERROR_INTR_STS_CMD_TIMEOUT_MASK);
    /* Wait for the command inhibit Status to go low */
    (void)MMCSD_waitCommandInhibit(   object->initHandle->ctrlBaseAddr,
                                MMCSD_WAIT_FOREVER);
    /* Send Command */
    MMCSD_sendCommand(object->initHandle->ctrlBaseAddr, trans);
    /* Poll until cmd complete interrupt status bit is set or any error
     * interrupt status bit is set */
    MMCSD_lld_cmdCompleteStatusPoll(handle);
    /* Get and store response in transaction object */
    (void)MMCSD_getCmdResponse(object->initHandle->ctrlBaseAddr,
                               trans->response);

    /* Check whether error happened or not */
    if(object->cmdErrorStat != (uint16_t)0U)
    {
        status = MMCSD_STS_ERR;
        /* Check if Command timeout happened */
        if((object->cmdErrorStat &
            CSL_MMC_CTLCFG_ERROR_INTR_STS_CMD_TIMEOUT_MASK) != 0U)
        {
            status = MMCSD_STS_ERR_TIMEOUT;
        }
    }

    return status;
}

static void MMCSD_lld_cmdCompleteStatusPoll(MMCSDLLD_Handle handle)
{
    MMCSDLLD_Object     *object = (MMCSDLLD_Object*)handle;
    uint16_t            normalIntrStatus = 0U;
    uint16_t            errorIntrStatus = 0U;
    bool                cmdComplete = false;
    bool                cmdError = false;

    while((cmdComplete == false) && (cmdError == false))
    {
        /* Get interrupt stats */
        normalIntrStatus = MMCSD_getNormIntrStat(
            object->initHandle->ctrlBaseAddr, MMCSD_INTERRUPT_ALL_NORMAL);
        errorIntrStatus = MMCSD_getErrIntrStat(
            object->initHandle->ctrlBaseAddr, MMCSD_INTERRUPT_ALL_ERROR);

        /* Check for command completion */
        if((normalIntrStatus &
            CSL_MMC_CTLCFG_NORMAL_INTR_STS_CMD_COMPLETE_MASK) !=
            (uint16_t)0U)
        {
            MMCSD_clearNormIntrStat(object->initHandle->ctrlBaseAddr,
                    CSL_MMC_CTLCFG_NORMAL_INTR_STS_CMD_COMPLETE_MASK);
            cmdComplete = true;
        }

        /* Check for errors */
        if(errorIntrStatus != (uint16_t)0U)
        {
            cmdError = true;
            object->cmdErrorStat = errorIntrStatus;
        }
    }
}

static void MMCSD_lld_xferCompleteStatusPoll(MMCSDLLD_Handle handle)
{
    MMCSDLLD_Object     *object = (MMCSDLLD_Object*)handle;
    uint16_t            normalIntrStatus = 0U;
    uint16_t            errorIntrStatus = 0U;
    bool                xferComplete = false;
    bool                xferError = false;
    uint32_t            offset = 0U;
    uint32_t            dataLength = 0U;
    uint8_t             *pTempWord = NULL;
    uint32_t            tempWord = 0xFFFFFFFFU;
    uint32_t            i = 0;

    while((xferComplete == false) && (xferError == false))
    {
        /* Get interrupt stats */
        normalIntrStatus = MMCSD_getNormIntrStat(
            object->initHandle->ctrlBaseAddr, MMCSD_INTERRUPT_ALL_NORMAL);
        errorIntrStatus = MMCSD_getErrIntrStat(
            object->initHandle->ctrlBaseAddr, MMCSD_INTERRUPT_ALL_ERROR);

        /* Read Data from Media condition */
        if((normalIntrStatus &
            CSL_MMC_CTLCFG_NORMAL_INTR_STS_BUF_RD_READY_MASK) != 0U)
        {
            MMCSD_clearNormIntrStat(object->initHandle->ctrlBaseAddr,
                            CSL_MMC_CTLCFG_NORMAL_INTR_STS_BUF_RD_READY_MASK);

            if( (object->dataBufIdx != NULL) &&
                (object->remainingBlockCount != 0U))
            {
                dataLength = object->dataBlockSize;
                offset = (( (object->dataBlockCount) -
                            (object->remainingBlockCount)) *
                            (object->dataBlockSize));

                for(i = 0; i < dataLength; i += 4U)
                {
                    MMCSD_readDataPort(object->initHandle->ctrlBaseAddr,
                                                                &tempWord);
                    pTempWord = (uint8_t *)&tempWord;
                    object->dataBufIdx[offset + i] = *(pTempWord);
                    object->dataBufIdx[offset + i + 1U] = *(pTempWord + 1U);
                    object->dataBufIdx[offset + i + 2U] = *(pTempWord + 2U);
                    object->dataBufIdx[offset + i + 3U] = *(pTempWord + 3U);
                }
                object->remainingBlockCount--;
            }
        }

        /* Write Data to Media condition */
        if((normalIntrStatus &
            CSL_MMC_CTLCFG_NORMAL_INTR_STS_BUF_WR_READY_MASK) != 0U)
        {
            MMCSD_clearNormIntrStat(object->initHandle->ctrlBaseAddr,
                            CSL_MMC_CTLCFG_NORMAL_INTR_STS_BUF_WR_READY_MASK);

            if( (object->dataBufIdx != NULL) &&
                (object->remainingBlockCount != 0U))
            {
                dataLength = object->dataBlockSize;
                offset = (( (object->dataBlockCount) -
                            (object->remainingBlockCount)) *
                            (object->dataBlockSize));

                for(i = 0; i < dataLength; i += 4U)
                {
                    pTempWord = (uint8_t *)&tempWord;
                    *(pTempWord)      = object->dataBufIdx[offset + i];
                    *(pTempWord + 1U) = object->dataBufIdx[offset + i + 1U];
                    *(pTempWord + 2U) = object->dataBufIdx[offset + i + 2U];
                    *(pTempWord + 3U) = object->dataBufIdx[offset + i + 3U];

                    MMCSD_writeDataPort(object->initHandle->ctrlBaseAddr,
                                                                    &tempWord);
                }
                object->remainingBlockCount--;
            }
        }

        /* Transfer Complete Condition */
        if((normalIntrStatus &
            CSL_MMC_CTLCFG_NORMAL_INTR_STS_XFER_COMPLETE_MASK) != 0U)
        {
            if(object->remainingBlockCount == 0U)
            {
                MMCSD_clearNormIntrStat(object->initHandle->ctrlBaseAddr,
                            CSL_MMC_CTLCFG_NORMAL_INTR_STS_XFER_COMPLETE_MASK);
                xferComplete = true;
            }
        }

        /* Error Check */
        if(errorIntrStatus != 0U)
        {
            /* Set xferError flag to break out of loop */
            xferError = true;
            /* Store Transfer Error in driver object */
            object->xferErrorStat = errorIntrStatus;
        }
    }
}

static void MMCSD_lld_xferCompleteStatusPollCMD19(MMCSDLLD_Handle handle)
{
    MMCSDLLD_Object     *object = (MMCSDLLD_Object*)handle;
    uint16_t            normalIntrStatus = 0U;
    uint16_t            errorIntrStatus = 0U;
    uint8_t             *pTempWord = NULL;
    uint32_t            i = 0U, tempWord = 0x00U;
    uint32_t            dataLength = 0U;
    bool                xferComplete = false;
    bool                xferError = false;

    while((xferComplete == false) && (xferError == false))
    {
        /* Get interrupt stats */
        normalIntrStatus = MMCSD_getNormIntrStat(
                                            object->initHandle->ctrlBaseAddr,
                                            MMCSD_INTERRUPT_ALL_NORMAL);
        errorIntrStatus = MMCSD_getErrIntrStat(
                                            object->initHandle->ctrlBaseAddr,
                                            MMCSD_INTERRUPT_ALL_ERROR);

        if((normalIntrStatus & CSL_MMC_CTLCFG_NORMAL_INTR_STS_BUF_RD_READY_MASK) != 0U)
        {
            MMCSD_clearNormIntrStat(object->initHandle->ctrlBaseAddr,
                            CSL_MMC_CTLCFG_NORMAL_INTR_STS_BUF_RD_READY_MASK);

            if(object->dataBufIdx != NULL)
            {
                dataLength = ((object->dataBlockSize)*(object->dataBlockCount));
                for(i = 0; i < dataLength; i += 4U)
                {
                    MMCSD_readDataPort(object->initHandle->ctrlBaseAddr,
                                                                &tempWord);
                    pTempWord = (uint8_t *)&tempWord;
                    object->dataBufIdx[i] = *(pTempWord);
                    object->dataBufIdx[i + 1U] = *(pTempWord + 1U);
                    object->dataBufIdx[i + 2U] = *(pTempWord + 2U);
                    object->dataBufIdx[i + 3U] = *(pTempWord + 3U);
                }
            }
            xferComplete = true;
        }

        MMCSD_clearNormIntrStat(object->initHandle->ctrlBaseAddr,
                            CSL_MMC_CTLCFG_NORMAL_INTR_STS_XFER_COMPLETE_MASK);

        if(     ((errorIntrStatus &
                CSL_MMC_CTLCFG_ERROR_INTR_STS_DATA_TIMEOUT_MASK) != 0U) &&
                !((normalIntrStatus &
                CSL_MMC_CTLCFG_NORMAL_INTR_STS_XFER_COMPLETE_MASK) != 0U))
        {
            MMCSD_clearNormIntrStat(object->initHandle->ctrlBaseAddr,
                            CSL_MMC_CTLCFG_ERROR_INTR_STS_DATA_TIMEOUT_MASK);
            /* Store Transfer Error in driver object */
            object->xferErrorStat = errorIntrStatus;
            xferError = true;
        }
    }
}

static int32_t MMCSD_lld_transferIntr(MMCSDLLD_Handle handle,
                                      MMCSDLLD_Transaction *trans)
{
    int32_t             status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object     *object = (MMCSDLLD_Object*)handle;

    if( (object->state == MMCSD_STATE_IDLE) ||
        (   (object->xferState == MMCSD_XFER_WRITE_STATE) &&
            (trans->cmd == MMCSD_SD_CMD(12U))))
    {
        /* Set current Transaction */
        object->currentTxn = trans;

        /* Set driver state to BUSY */
        object->state = MMCSD_STATE_BUSY;

        /* Clear all normal interrupt statuses */
        MMCSD_clearNormIntrStat(object->initHandle->ctrlBaseAddr,
                                MMCSD_INTERRUPT_ALL_NORMAL);
        /* Clear all Error Interrupt statuses */
        MMCSD_clearErrIntrStat(object->initHandle->ctrlBaseAddr,
                                MMCSD_INTERRUPT_ALL_ERROR);

        if((object->cmdErrorStat != 0U) || (object->xferErrorStat != 0U))
        {
            /* Some Error happened in last transaction */
            (void)MMCSD_linesResetCmd(object->initHandle->ctrlBaseAddr,
                                        MMCSD_WAIT_FOREVER);
            (void)MMCSD_linesResetDat(object->initHandle->ctrlBaseAddr,
                                        MMCSD_WAIT_FOREVER);
        }

        /* Clear stored Error interrupt statuses */
        object->cmdErrorStat = (uint16_t)0U;
        object->xferErrorStat = (uint16_t)0U;

        if((trans->cmd & CSL_MMC_CTLCFG_COMMAND_DATA_PRESENT_MASK) != 0U)
        {
            status = MMCSD_lld_transferDataIntr(handle, trans);
        }
        else
        {
            status = MMCSD_lld_transferNoDataIntr(handle, trans);
        }
    }
    else
    {
        status = MMCSD_STS_ERR_BUSY;
    }

    return status;
}

static int32_t MMCSD_lld_transferDataIntr(MMCSDLLD_Handle handle,
                                          MMCSDLLD_Transaction *trans)
{
    int32_t             status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object     *object = (MMCSDLLD_Object*)handle;
    uint32_t            dataSize = 0U;

    /* Store all the transaction data in driver object */
    object->dataBufIdx = trans->dataBuf;
    object->dataBlockCount = trans->blockCount;
    object->dataBlockSize = trans->blockSize;

    /* Check if the transfer is read type or write type default is write
     * If transfer is read type
        Enable buffer read Ready interrupt & Enable the signal for the same
     * If transfer is write type
        Enable buffer write Ready interrupt & Enable the signal for the same
     */
    if(trans->dir == MMCSD_CMD_XFER_TYPE_READ)
    {
        MMCSD_enableNormIntrStat(object->initHandle->ctrlBaseAddr,
                        CSL_MMC_CTLCFG_NORMAL_INTR_STS_ENA_BUF_RD_READY_MASK);
        MMCSD_enableNormSigIntr(object->initHandle->ctrlBaseAddr,
                        CSL_MMC_CTLCFG_NORMAL_INTR_SIG_ENA_BUF_RD_READY_MASK);
    }
    else
    {
        MMCSD_enableNormIntrStat(object->initHandle->ctrlBaseAddr,
                        CSL_MMC_CTLCFG_NORMAL_INTR_STS_ENA_BUF_WR_READY_MASK);
        MMCSD_enableNormSigIntr(object->initHandle->ctrlBaseAddr,
                        CSL_MMC_CTLCFG_NORMAL_INTR_STS_ENA_BUF_WR_READY_MASK);
    }

    /* Enable Transfer Complete normal interrupt & Signal for the same */
    /* Enable Command Complete normal interrupt & Signal for the same */
    MMCSD_enableNormIntrStat(object->initHandle->ctrlBaseAddr,
                CSL_MMC_CTLCFG_NORMAL_INTR_STS_ENA_CMD_COMPLETE_MASK |
                CSL_MMC_CTLCFG_NORMAL_INTR_STS_ENA_XFER_COMPLETE_MASK);
    MMCSD_enableNormSigIntr(object->initHandle->ctrlBaseAddr,
                CSL_MMC_CTLCFG_NORMAL_INTR_STS_ENA_CMD_COMPLETE_MASK |
                CSL_MMC_CTLCFG_NORMAL_INTR_STS_ENA_XFER_COMPLETE_MASK);
    /* Enable all Error interrupts & Signal for the Same */
    MMCSD_enableErrIntrStat(object->initHandle->ctrlBaseAddr,
                            MMCSD_INTERRUPT_ALL_ERROR);
    MMCSD_enableErrSigIntr(object->initHandle->ctrlBaseAddr,
                           MMCSD_INTERRUPT_ALL_ERROR);

    /* Write back invalidate the write buffer idx in cache */
    CacheP_wbInv(object->dataBufIdx, (trans->blockSize * trans->blockCount),
                    CacheP_TYPE_ALL);

    /* If dma is enabled
     * Enable ADMA2
     */
    if((trans->enableDma == TRUE) && (trans->isTuning == FALSE))
    {
        /* Setup ADMA2 descriptor */
        dataSize = ((trans->blockCount) * (trans->blockSize));
        /* Set up EDMA */
        MMCSD_setupADMA2(handle, &gADMA2Desc, (uint64_t)trans->dataBuf,
                         dataSize);
        object->remainingBlockCount = 0U;
    }
    else
    {
        /* Set the remaining block count as it will be used for keeping count */
        object->remainingBlockCount = object->dataBlockCount;
    }

    /* Wait for the CMD inhibit line to go low (Command can be issued) */
    status = MMCSD_waitCommandInhibit(object->initHandle->ctrlBaseAddr,
                                      MMCSD_WAIT_FOREVER);
    if(status == MMCSD_STS_SUCCESS)
    {
        /* Wait for the DATA inhibit line to go low (Data can be transacted) */
        status = MMCSD_waitDataInhibit(object->initHandle->ctrlBaseAddr,
                                       MMCSD_WAIT_FOREVER);
    }

    /* Set Transfer state to command */
    object->xferState = MMCSD_XFER_CMD_STATE;

    /* Send command */
    MMCSD_sendCommand(object->initHandle->ctrlBaseAddr, object->currentTxn);

    return status;
}

static int32_t MMCSD_lld_transferNoDataIntr(MMCSDLLD_Handle handle,
                                            MMCSDLLD_Transaction *trans)
{
    int32_t             status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object     *object = (MMCSDLLD_Object*)handle;

    /* Enable Command Complete Normal Interrupt Status */
    MMCSD_enableNormIntrStat(object->initHandle->ctrlBaseAddr,
                             CSL_MMC_CTLCFG_NORMAL_INTR_STS_CMD_COMPLETE_MASK);
    /* Enable Command Timeout Error Interrupt Status */
    MMCSD_enableErrIntrStat(object->initHandle->ctrlBaseAddr,
                            CSL_MMC_CTLCFG_ERROR_INTR_STS_CMD_TIMEOUT_MASK);
    /* Wait for the command inhibit Status to go low */
    status = MMCSD_waitCommandInhibit(object->initHandle->ctrlBaseAddr,
                                      MMCSD_WAIT_FOREVER);
    /* Enable Command Complete Normal Interrupt Signal */
    MMCSD_enableNormSigIntr(object->initHandle->ctrlBaseAddr,
                        CSL_MMC_CTLCFG_NORMAL_INTR_SIG_ENA_CMD_COMPLETE_MASK);
    /* Enable Command Timeout Error Interrupt Signal */
    MMCSD_enableErrSigIntr(object->initHandle->ctrlBaseAddr,
                        CSL_MMC_CTLCFG_ERROR_INTR_SIG_ENA_CMD_TIMEOUT_MASK);
    /* Set Transfer state to command */
    object->xferState = MMCSD_XFER_CMD_STATE;
    /* Send Command */
    MMCSD_sendCommand(object->initHandle->ctrlBaseAddr, object->currentTxn);

    return status;
}

static void MMCSD_lld_completeCurrTransfer(MMCSDLLD_Handle handle,
                                           int32_t xferStatus)
{
    MMCSDLLD_Object     *object = (MMCSDLLD_Object*)handle;

    /* Clear pointer to current Transaction */
    object->currentTxn = NULL;
    /* Callback to application */
    object->transferCompleteCallback(handle, xferStatus);
    /* Change Transfer state back to IDLE */
    object->xferState = MMCSD_XFER_IDLE_STATE;
    /*Change Driver State back to IDLE */
    object->state = MMCSD_STATE_IDLE;
}