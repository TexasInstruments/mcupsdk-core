/*
 *  Copyright (C) 2024-2025 Texas Instruments Incorporated
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

/**
 *  \file mmcsd_v1_lld.c
 *
 *  \brief File containing MMCSD Driver APIs implementation for version V1.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/mmcsd/v1/lld/mmcsd_lld.h>
#include <drivers/mmcsd/v1/lld/dma/edma/mmcsd_edma_lld.h>
#include <drivers/mmcsd/v1/lld/internal/mmcsd_parse.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/soc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * \anchor   MMCSDLLD_ClockActivity
 * \name MACROS for the possible values of CLOCKACTIVITY configuration.
 * @{
 */

/** \brief Both functional and interface clocks are off */
#define MMCSD_CLK_ACT_ICLK_FCLK_OFF     CSL_MMC_SYSCONFIG_CLOCKACTIVITY_NONE
/** \brief Interface clock is on and Functional clock is off */
#define MMCSD_CLK_ACT_FCLK_OFF          CSL_MMC_SYSCONFIG_CLOCKACTIVITY_FUNC
/** \brief Interface clock is off and Functional clock is on */
#define MMCSD_CLK_ACT_ICLK_OFF          CSL_MMC_SYSCONFIG_CLOCKACTIVITY_OCP
/** \brief Both functional and interface clocks are on */
#define MMCSD_CLK_ACT_ICLK_FLCK_ON      CSL_MMC_SYSCONFIG_CLOCKACTIVITY_BOTH

/** @} */

/**
 * \anchor   MMCSDLLD_StandByMode
 * \name MACROS for the possible values of STANDBYMODE configuration.
 * @{
 */

/** \brief Force standby mode */
#define MMCSD_STANDBY_MODE_FORCE        CSL_MMC_SYSCONFIG_STANDBYMODE_FORCE
/** \brief No standby mode */
#define MMCSD_STANDBY_MODE_NONE         CSL_MMC_SYSCONFIG_STANDBYMODE_NOIDLE
/** \brief Smart standby mode */
#define MMCSD_STANDBY_MODE_SMART        CSL_MMC_SYSCONFIG_STANDBYMODE_SMART
/** \brief Smart standby with wakeup mode */
#define MMCSD_STANDBY_MODE_SMART_WAKEUP CSL_MMC_SYSCONFIG_STANDBYMODE_SMARTWAKE

/** @} */

/**
 * \anchor   MMCSDLLD_SIdleMode
 * \name MACROS for the possible values of SIDLEMODE configuration.
 * @{
 */

/** \brief Force idle mode */
#define MMCSD_IDLE_MODE_FORCE           CSL_MMC_SYSCONFIG_SIDLEMODE_FORCE
/** \brief No idle mode */
#define MMCSD_IDLE_MODE_NONE            CSL_MMC_SYSCONFIG_SIDLEMODE_NOIDLE
/** \brief Smart idle mode */
#define MMCSD_IDLE_MODE_SMART           CSL_MMC_SYSCONFIG_SIDLEMODE_SMART

/** @} */

/**
 * \anchor   MMCSDLLD_BusVoltage
 * \name MACROS for the possible values of Bus Voltage.
 * @{
 */

/** \brief Bus voltage is 1.8 Volts */
#define MMCSD_BUS_VOLT_1P8              CSL_MMC_HCTL_SDVS__1V8
/** \brief Bus voltage is 3.0 Volts */
#define MMCSD_BUS_VOLT_3P0              CSL_MMC_HCTL_SDVS__3V0
/** \brief Bus voltage is 3.3 Volts */
#define MMCSD_BUS_VOLT_3P3              CSL_MMC_HCTL_SDVS__3V3

/** @} */


/**
 * \anchor   MMCSDLLD_SupVoltage
 * \name MACROS for the possible supported bus Voltages.
 * @{
 */

/** \brief Supported voltage is 1.8 Volts */
#define MMCSD_SUPP_VOLT_1P8             CSL_MMC_CAPA_VS18_MASK
/** \brief Supported voltage is 3.0 Volts */
#define MMCSD_SUPP_VOLT_3P0             CSL_MMC_CAPA_VS30_MASK
/** \brief Supported voltage is 3.3 Volts */
#define MMCSD_SUPP_VOLT_3P3             CSL_MMC_CAPA_VS33_MASK

/** @} */

/** \brief Enumerates the controller's interrupt masks. */
#define MMCSD_INTR_MASK_BADACCESS       CSL_MMC_IE_BADA_ENABLE_MASK
    /**< Bad access to data space interrupt. */
#define MMCSD_INTR_MASK_CARDERROR       CSL_MMC_IE_CERR_ENABLE_MASK
    /**< Card error interrupt. */
#define MMCSD_INTR_MASK_ADMAERROR       CSL_MMC_IE_ADMAE_ENABLE_MASK
    /**< ADMA error interrupt. */
#define MMCSD_INTR_MASK_ACMD12ERR       CSL_MMC_IE_ACE_ENABLE_MASK
    /**< Auto CMD12 error interrupt. */
#define MMCSD_INTR_MASK_DATABITERR      CSL_MMC_IE_DEB_ENABLE_MASK
    /**< Data end bit error interrupt. */
#define MMCSD_INTR_MASK_DATACRCERR      CSL_MMC_IE_DCRC_ENABLE_MASK
    /**< Data CRC error interrupt. */
#define MMCSD_INTR_MASK_DATATIMEOUT     CSL_MMC_IE_DTO_ENABLE_MASK
    /**< Data time out error interrupt. */
#define MMCSD_INTR_MASK_CMDINDXERR      CSL_MMC_IE_CIE_ENABLE_MASK
    /**< Command index error interrupt. */
#define MMCSD_INTR_MASK_CMDBITERR       CSL_MMC_IE_CEB_ENABLE_MASK
    /**< Command end bit error. */
#define MMCSD_INTR_MASK_CMDCRCERR       CSL_MMC_IE_CCRC_ENABLE_MASK
    /**< Command CRC error interrupt. */
#define MMCSD_INTR_MASK_CMDTIMEOUT      CSL_MMC_IE_CTO_ENABLE_MASK
    /**< Command timeout error interrupt. */
#define MMCSD_INTR_MASK_ERR             CSL_MMC_IE_NULL_MASK
    /**< Error interrupt. */
#define MMCSD_INTR_MASK_CARDREM         CSL_MMC_IE_CREM_ENABLE_MASK
    /**< Card removal signal interrupt. */
#define MMCSD_INTR_MASK_CARDINS         CSL_MMC_IE_CINS_ENABLE_MASK
    /**< Card insertion signal interrupt. */
#define MMCSD_INTR_MASK_BUFRDRDY        CSL_MMC_IE_BRR_ENABLE_MASK
    /**< Buffer read ready interrupt. */
#define MMCSD_INTR_MASK_BUFWRRDY        CSL_MMC_IE_BWR_ENABLE_MASK
    /**< Buffer write ready interrupt. */
#define MMCSD_INTR_MASK_TRNFCOMP        CSL_MMC_IE_TC_ENABLE_MASK
    /**< Transfer completed signal interrupt. */
#define MMCSD_INTR_MASK_CMDCOMP         CSL_MMC_IE_CC_ENABLE_MASK
    /**< Command completed signal interrupt. */

#define MMCSD_PSTATE_MASK_CMD_INHIBIT   (CSL_MMC_PSTATE_CMDI_MASK)
    /**< Command inhibit mask. */
#define MMCSD_PSTATE_MASK_DATA_INHIBIT  (CSL_MMC_PSTATE_DATI_MASK)
    /**< Data inhibit mask. */

#define MMCSD_ALL_INTS          (   MMCSD_INTR_MASK_BADACCESS       |   \
                                    MMCSD_INTR_MASK_CARDERROR       |   \
                                    MMCSD_INTR_MASK_ADMAERROR       |   \
                                    MMCSD_INTR_MASK_ACMD12ERR       |   \
                                    MMCSD_INTR_MASK_DATABITERR      |   \
                                    MMCSD_INTR_MASK_DATACRCERR      |   \
                                    MMCSD_INTR_MASK_DATATIMEOUT     |   \
                                    MMCSD_INTR_MASK_CMDINDXERR      |   \
                                    MMCSD_INTR_MASK_CMDBITERR       |   \
                                    MMCSD_INTR_MASK_CMDCRCERR       |   \
                                    MMCSD_INTR_MASK_CMDTIMEOUT      |   \
                                    MMCSD_INTR_MASK_ERR             |   \
                                    MMCSD_INTR_MASK_CARDREM         |   \
                                    MMCSD_INTR_MASK_CARDINS         |   \
                                    MMCSD_INTR_MASK_BUFRDRDY        |   \
                                    MMCSD_INTR_MASK_BUFWRRDY        |   \
                                    MMCSD_INTR_MASK_TRNFCOMP        |   \
                                    MMCSD_INTR_MASK_CMDCOMP             \
                                )

/**
 * \anchor   MMCSDLLD_BusPower
 * \name MACROS for the possible power configuration options, on/off.
 * @{
 */

/** \brief Power on the controller */
#define MMCSD_PWR_CTRL_ON               CSL_MMC_HCTL_SDBP_PWRON
/** \brief Power off the controller */
#define MMCSD_PWR_CTRL_OFF              CSL_MMC_HCTL_SDBP_PWROFF

/** @} */

/**
 * \anchor   MMCSDLLD_CommandType
 * \name MACROS for the possible MMCSD Command type.
 * @{
 */

/** Others commands. */
#define MMCSD_CMD_TYPE_OTHER            CSL_MMC_CMD_CMD_TYPE_NORMAL
/** Upon CMD52 "Bus Suspend" operation. */
#define MMCSD_CMD_TYPE_BUS_SUSPEND      CSL_MMC_CMD_CMD_TYPE_SUSPEND
/** Upon CMD52 "Function Select" operation. */
#define MMCSD_CMD_TYPE_FUNC_SEL         CSL_MMC_CMD_CMD_TYPE_RESUME
/** Upon CMD12 or CMD52 "I/O Abort" command. */
#define MMCSD_CMD_TYPE_IO_ABORT         CSL_MMC_CMD_CMD_TYPE_ABORT

/** @} */

/**
 * \anchor   MMCSDLLD_ResponseType
 * \name MACROS for the possible MMCSD Response type.
 * @{
 */

/** No response. */
#define MMCSD_RSP_TYPE_NONE             CSL_MMC_CMD_RSP_TYPE_NORSP
/** Response Length 136 bits. */
#define MMCSD_RSP_TYPE_LEN_136          CSL_MMC_CMD_RSP_TYPE_LGHT36
/** Response Length 48 bits. */
#define MMCSD_RSP_TYPE_LEN_48           CSL_MMC_CMD_RSP_TYPE_LGHT48
/** Response Length 48 bits with busy after response. */
#define MMCSD_RSP_TYPE_LEN_48_BUSY      CSL_MMC_CMD_RSP_TYPE_LGHT48B

/** @} */

/**
 * \anchor   MMCSDLLD_TransferType
 * \name MACROS for the possible transfer Data Direction.
 * @{
 */

/** Data Write (host to card). */
#define MMCSD_XFER_TYPE_TX              CSL_MMC_CMD_DDIR_WRITE
/** Data Read (card to host). */
#define MMCSD_XFER_TYPE_RX              CSL_MMC_CMD_DDIR_READ

/** @} */

/** \brief Frequencies for SD Card's Different Speed Modes */
#define MMCSD_SD_DS_FREQUENCY_HZ                            ((uint32_t) 25000000U)
#define MMCSD_SD_HS_FREQUENCY_HZ                            ((uint32_t) 50000000U)

/** \brief Frequencies for EMMC Card's Different Speed Modes */
#define MMCSD_EMMC_DS_FREQUENCY_HZ                           ((uint32_t) 25000000U)
#define MMCSD_EMMC_HS_FREQUENCY_HZ                           ((uint32_t) 50000000U)

/** \brief Card bus frequency configuration for 25 Mbps. */
#define MMCSD_TRANSPEED_25MBPS      (0x32U)

/** \brief Card bus frequency configuration for 50 Mbps. */
#define MMCSD_TRANSPEED_50MBPS      (0x5AU)

/** \brief MACROS used to select one of the possible Bus Voltages. */
#define MMCSD_BUS_VOLT_1_8V                                 (0x5U)
#define MMCSD_BUS_VOLT_3_0V                                 (0x6U)
#define MMCSD_BUS_VOLT_3_3V                                 (0x7U)

/** \brief Macros related to setting operating voltage for SD devices */
#define MMCSD_SD_CMD8_CHECK_PATTERN                         (0xAAU)
#define MMCSD_SD_VOLT_2P7_3P6                               ((uint32_t)0x1U)
#define MMCSD_SD_VOLT_LOW_RANGE                             ((uint32_t)0x2U)

/** \brief Command argument to configure for switch mode. */
#define MMCSD_SWITCH_MODE                                   (0x80FFFFFFU)
#define MMCSD_CHECK_MODE                                    (0x00FFFFFFU)
/** \brief Command argument width to configure for transfer speed. */
#define MMCSD_CMD6_GRP1_SEL                                 (0xFFFFFFF0U)
#define MMCSD_CMD6_GRP4_SEL                                 (0xFFFF0FFFU)

/** \brief Command argument to configure for default/SDR12 speed. */
#define MMCSD_CMD6_GRP1_DEFAULT                             (0x0U)
/** \brief Command argument to configure for high/SDR25 speed. */
#define MMCSD_CMD6_GRP1_HS                                  (0x1U)
/** \brief Command argument to configure for SDR50 speed. */
#define MMCSD_CMD6_GRP1_SDR50                               (0x2U)
/** \brief Command argument to configure for SDR104 speed. */
#define MMCSD_CMD6_GRP1_SDR104                              (0x3U)
/** \brief Command argument to configure for DDR50 speed. */
#define MMCSD_CMD6_GRP1_DDR50                               (0x4U)

/** \brief Macros for States Used during the Initialization of SD Device */
#define MMCSD_SD_INIT_STATE_CONTROLLER_INIT                 (0U)
#define MMCSD_SD_INIT_STATE_CHECK_VOLTAGE                   (1U)
// #define MMCSD_SD_INIT_STATE_CHECK_SDIO                      (2U)
#define MMCSD_SD_INIT_STATE_CARD_IDENTIFICATION             (3U)
#define MMCSD_SD_INIT_STATE_VOLTAGE_SWITCH                  (4U)
#define MMCSD_SD_INIT_STATE_GET_CID                         (5U)
#define MMCSD_SD_INIT_STATE_GET_RCA                         (6U)
#define MMCSD_SD_INIT_STATE_GET_CSD                         (7U)
#define MMCSD_SD_INIT_STATE_CHECK_LOCKED                    (8U)
#define MMCSD_SD_INIT_STATE_UNLOCK                          (9U)
#define MMCSD_SD_INIT_STATE_GET_SCR                         (10U)
#define MMCSD_SD_INIT_STATE_BUS_WIDTH_SWITCH                (11U)
#define MMCSD_SD_INIT_STATE_SELECT_BUS_SPEED_MODE           (12U)
// #define MMCSD_SD_INIT_STATE_DRIVE_STRENGTH                  (13U)
#define MMCSD_SD_INIT_STATE_CURRENT_LIMIT                   (14U)
// #define MMCSD_SD_INIT_STATE_SWITCH_BUS_SPEED_MODE           (15U)
#define MMCSD_SD_INIT_STATE_TUNING                          (16U)

#define MMCSD_ID_MODE_FREQUENCY_HZ                      ((uint32_t) 400000U)
/** \brief Macros for States Used during the Initialization of EMMC Device */
#define EMMC_INIT_STATE_CONTROLLER_INIT  0
#define EMMC_INIT_STATE_CHECK_VOLTAGE    1
#define EMMC_INIT_STATE_CARD_IDENTIFICATION 2
#define EMMC_INIT_STATE_GET_CID          3
#define EMMC_INIT_STATE_GET_RCA          4
#define EMMC_INIT_STATE_GET_CSD          5
#define EMMC_INIT_STATE_SELECT_CARD      6
#define EMMC_INIT_STATE_READ_EXT_CSD     7
#define EMMC_INIT_STATE_SET_BUS_WIDTH    8
#define EMMC_INIT_STATE_SET_SPEED        9
#define EMMC_INIT_STATE_DONE             10

#define MMCSD_CMD6_GRP4_200mA                               (0x0U)
#define MMCSD_CMD6_GRP4_400mA                               (0x1U)
#define MMCSD_CMD6_GRP4_600mA                               (0x2U)
#define MMCSD_CMD6_GRP4_800mA                               (0x3U)

/** \brief Card status value (Bits 9-12) as defined in JESD84-B51 6.13 section
 * & physical layer specification section 4.10.1.
 */
#define MMCSD_MEDIA_STATE_IDLE                              (0U)
#define MMCSD_MEDIA_STATE_READY                             (1U)
#define MMCSD_MEDIA_STATE_IDENT                             (2U)
#define MMCSD_MEDIA_STATE_STBY                              (3U)
#define MMCSD_MEDIA_STATE_TRAN                              (4U)
#define MMCSD_MEDIA_STATE_DATA                              (5U)
#define MMCSD_MEDIA_STATE_RCV                               (6U)
#define MMCSD_MEDIA_STATE_PRG                               (7U)
#define MMCSD_MEDIA_STATE_DIS                               (8U)
#define MMCSD_MEDIA_STATE_BTST                              (9U)
#define MMCSD_MEDIA_STATE_SLP                               (10U)

/** \brief This is the timeout value for sending CMD13 to the card.
 * After every write, the CMD13 is sent this many times and wait for
 * the card to go to transfer state
 * */
#define MMCSD_MEDIA_STATE_THRESHOLD                         (100000U)
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/** \brief Structure holding the hs mmcsd controller system configurations. */
typedef struct MMCSD_sysCfg_t
{
    /** \brief Configuration of clock activity during wake up period. This can
     * take values from the following \ref MMCSDLLD_ClockActivity */
    uint32_t clockActivity;
    /** \brief standby mode configuration. This can take values from the
     * following \ref MMCSDLLD_StandByMode */
    uint32_t standbyMode;
    /** Idle mode configuration. This can take values from the following
     * \ref MMCSDLLD_SIdleMode */
    uint32_t idleMode;
    /** Wake up enable/disable control. This can take following two values
     *  - TRUE  - Enable Wakeup
     *  - FALSE - Disable wakeup
     * */
    bool enableWakeup;
    /** Auto idle enable/disable control. This can take following two values
     *  - TRUE  - Enable auto idle
     *  - FALSE - Disable auto idle
    */
    bool enableAutoIdle;

}MMCSD_sysCfg;


/** \brief Structure for MMCSD command. */
typedef struct MMCSD_cmd_t{

    /** Command ID uses standard MMCSD card commands */
    uint32_t cmdId;
    /** Command type uses \ref MMCSDLLD_CommandType */
    uint32_t cmdType;
    /** Response type uses \ref MMCSDLLD_ResponseType */
    uint32_t rspType;
    /** Command direction uses \ref MMCSDLLD_TransferType */
    uint32_t xferType;

} MMCSD_cmd;

/** \brief Structure holding the hs mmcsd controller command object. */
typedef struct MMCSD_cmdObj_t
{
    MMCSD_cmd cmd;
    /**< Command to be passed to the controller/card. */
    uint32_t cmdArg;
    /**< Argument for the command. */
    bool enableData;
    /**< This flag enables/disables data transfer. This can take following
         two values:
         - TRUE  - Enable data transfer
         - FALSE - Disable data transfer. */
    uint32_t numBlks;
    /**< Transfer data length in number of blocks (multiple of BLEN). This can
         take values in the following range:
         1 <= numBlks <= 65535. Value of 0 means Stop count. */
    bool enableDma;
    /**< This flag enables/disables Dma. This can take following two values:
         - TRUE  - Enable DMA.
         - FALSE - Disable DMA. */
}MMCSD_cmdObj;

/* Default MMCSD transaction parameters structure */
const MMCSDLLD_Transaction MMCSD_lld_defaultTransaction = {

    0,                                          /* Command */
    0,                                          /* flags */
    0,                                          /* arg */
    (uint8_t*)NULL,                             /* Data Buffer */
    512U,                                       /* Block Size */
    1U,                                         /* Block Count */
    {0,0,0,0}                                   /* Response */
};

/* ========================================================================== */
/*                     Internal Function Declarations                         */
/* ========================================================================== */


static int32_t MMCSD_softReset(uint32_t baseAddr, uint32_t loopTimeout);
static int32_t MMCSD_linesResetCmd(uint32_t baseAddr, uint32_t loopTimeout);
static int32_t MMCSD_linesResetDat(uint32_t baseAddr, uint32_t loopTimeout);
static int32_t MMCSD_linesResetAll(uint32_t baseAddr, uint32_t loopTimeout);
static void MMCSD_systemConfig(uint32_t baseAddr, const MMCSD_sysCfg *pCfg);
static void MMCSD_setBusWidth(uint32_t baseAddr, uint32_t busWidth);
static void MMCSD_setBusVolt(uint32_t baseAddr, uint32_t busVoltage);
static uint32_t MMCSD_getBusVolt(uint32_t baseAddr);
static int32_t MMCSD_busPowerOnCtrl(uint32_t baseAddr, bool pwrCtrl,
                                    uint32_t loopTimeout);
static bool MMCSD_isIntClockStable(uint32_t baseAddr, uint32_t loopTimeout);
static int32_t MMCSD_intClockEnable(uint32_t baseAddr, bool enableIntClk);
static void MMCSD_setSupportedVoltage(uint32_t baseAddr, uint32_t voltMask);
static bool MMCSD_isHighSpeedSupported(uint32_t baseAddr);
static bool MMCSD_isUHSSDR50Supported(uint32_t baseAddr);
static bool MMCSD_isUHSSDR104Supported(uint32_t baseAddr);
static bool MMCSD_isUHSDDR50Supported(uint32_t baseAddr);
static void MMCSD_setDataTimeout(uint32_t baseAddr, uint32_t timeout);
static bool MMCSD_isCardInserted(uint32_t baseAddr);
static int32_t MMCSD_setBusFreq(uint32_t baseAddr, uint32_t inputFreq,
                                uint32_t outputFreq);
static void MMCSD_setBlkLength(uint32_t baseAddr, uint32_t blkLen);
static uint32_t MMCSD_getBlkLength(uint32_t baseAddr);
static uint32_t MMCSD_getBlkCount(uint32_t baseAddr);
static void MMCSD_getData(uint32_t baseAddr, uint8_t *pData, uint32_t len);
static void MMCSD_setData(uint32_t baseAddr, const uint8_t *pData, uint32_t len);
static void MMCSD_setDLLSWT(uint32_t baseAddr, uint32_t val);
static uint32_t MMCSD_getDLLSWT(uint32_t baseAddr);

static uint32_t MMCSD_getCAPA(uint32_t baseAddr);
static void MMCSD_enableIntrStatus(uint32_t baseAddr, uint32_t intrMask);
static void MMCSD_disableIntrStatus(uint32_t baseAddr, uint32_t intrMask);
static void MMCSD_enableSigIntr(uint32_t baseAddr, uint32_t intrMask);
static void MMCSD_disableSigIntr(uint32_t baseAddr, uint32_t intrMask);
static uint32_t MMCSD_getIntrSigEnable(uint32_t baseAddr);
static uint32_t MMCSD_getIntrStat(uint32_t baseAddr);
static uint32_t MMCSD_getPstateStat(uint32_t baseAddr);
static void MMCSD_clearIntrStat(uint32_t baseAddr, uint32_t intrMask);

static void MMCSD_setPADEN(uint32_t baseAddr, uint32_t val);
static uint32_t MMCSD_getCmdSignalLevel(uint32_t baseAddr);
static uint32_t MMCSD_getDataSignalLevel(uint32_t baseAddr);
static uint32_t MMCSD_isCardWriteProtected(uint32_t baseAddr);

static int32_t MMCSD_initStreamSend(uint32_t baseAddr);
static void MMCSD_commandSend(uint32_t baseAddr, const MMCSD_cmdObj *pObj);
static int32_t MMCSD_isCmdComplete(uint32_t baseAddr, uint32_t loopTimeout);
static int32_t MMCSD_isXferComplete(uint32_t baseAddr, uint32_t loopTimeout);
static void MMCSD_getResponse(uint32_t baseAddr, uint32_t *pRsp);

/* API Assist internal Functions */
static void MMCSD_lld_initTransaction(MMCSDLLD_Transaction *trans);
static void MMCSD_intrConfigReset(MMCSDLLD_Handle handle);
static int32_t MMCSD_lld_initSD(MMCSDLLD_Handle handle);
static int32_t MMCSD_lld_initMMC(MMCSDLLD_Handle handle);

static int32_t MMCSD_lld_transferPoll(MMCSDLLD_Handle handle,
                                      MMCSDLLD_Transaction *trans);
static int32_t MMCSD_lld_transferIntr(MMCSDLLD_Handle handle,
                                      MMCSDLLD_Transaction *trans);
static int32_t MMCSD_lld_transferDma(MMCSDLLD_Handle handle, 
                                     MMCSDLLD_Transaction *trans);
static void MMCSD_lld_cmdCompleteStatusPoll(MMCSDLLD_Handle handle);
static void MMCSD_lld_xferCompleteStatusPoll(MMCSDLLD_Handle handle);
static void MMCSD_lld_xferCompleteStatusPollCMD19(MMCSDLLD_Handle handle);
static int32_t MMCSD_switchCardCurrLimit(MMCSDLLD_Handle handle,
                                         uint32_t cmd16GrpFunc);
static int32_t MMCSD_isCardReadyForTransferSD(MMCSDLLD_Handle handle);
static int32_t MMCSD_isCardReadyForTransferMMC(MMCSDLLD_Handle handle);
void MMCSD_lld_completeCurrTransfer(MMCSDLLD_Handle handle,
                                           int32_t xferStatus);
static int32_t MMCSD_lld_sendCmd23MMC(MMCSDLLD_Handle handle, uint32_t numBlks);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */





/* ========================================================================== */
/*                       API Function Definitions                             */
/* ========================================================================== */

static void MMCSD_lld_initTransaction(MMCSDLLD_Transaction *trans)
{
    *trans = MMCSD_lld_defaultTransaction;
}

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
        status = MMCSD_lld_isBaseAddrValid(object->initHandle->baseAddr);
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

int32_t MMCSD_lld_InitDma(MMCSDLLD_Handle handle)
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
        /* Enable DMA channel for data transfer */
        status = MMCSD_edmaChInit(handle);
    }

    if(status == MMCSD_STS_SUCCESS)
    {
        /* Check if the MMCSD base address is valid or not */
        status = MMCSD_lld_isBaseAddrValid(object->initHandle->baseAddr);
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
    return MMCSD_STS_SUCCESS;
}

int32_t MMCSD_lld_deInitDma(MMCSDLLD_Handle handle)
{
    int32_t status = MMCSD_STS_SUCCESS;

    if (handle != NULL)
    {
        MMCSDLLD_Object *object = (MMCSDLLD_Object *)handle;

        /* Deinitialize EDMA channels */
        status = MMCSD_edmaChDeinit(handle);

        if (status == MMCSD_STS_SUCCESS)
        {
            object->state = MMCSD_STATE_RESET;
        }
    }
    else
    {
        status = MMCSD_STS_ERR_INVALID_PARAM;
    }

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
    // uint32_t                stopCmd = MMCSD_SD_CMD(12U);
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

        MMCSD_lld_initTransaction(&trans);
        trans.arg = addr;
        trans.flags =   MMCSD_CMDRSP_WRITE | MMCSD_CMDRSP_DATA |
                        MMCSD_CMDREQ_WR_RD;
        trans.blockCount = numBlks;
        trans.blockSize = blockSize;
        trans.dataBuf = buf;

        if(numBlks > 1U)
        {
            trans.cmd = MMCSD_CMD(25U);
            trans.flags |= MMCSD_CMDRSP_ABORT;
        }
        else
        {
            trans.cmd = MMCSD_CMD(24);
        }

        status = MMCSD_lld_transferPoll(handle, &trans);
    }

    /* Through the SD Back into trans State */
    if(MMCSD_STS_SUCCESS == status)
    {
        if(trans.blockCount > 1U)
        {
            MMCSD_lld_initTransaction(&trans);
            trans.cmd = MMCSD_CMD(12U);
            trans.flags = MMCSD_CMDRSP_BUSY;
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
    // uint32_t                stopCmd = MMCSD_SD_CMD(12U);
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

        MMCSD_lld_initTransaction(&trans);
        trans.arg = addr;
        trans.flags =   MMCSD_CMDRSP_READ | MMCSD_CMDRSP_DATA |
                        MMCSD_CMDREQ_WR_RD;
        trans.blockCount = numBlks;
        trans.blockSize = blockSize;
        trans.dataBuf = buf;

        if(numBlks > 1U)
        {
            trans.cmd = MMCSD_CMD(18U);
            trans.flags |= MMCSD_CMDRSP_ABORT;
        }
        else
        {
            trans.cmd = MMCSD_CMD(17U);
        }

        status = MMCSD_lld_transferPoll(handle, &trans);
    }

    /* Through the SD Back into trans State */
    if(MMCSD_STS_SUCCESS == status)
    {
        if(trans.blockCount > 1U)
        {
            MMCSD_lld_initTransaction(&trans);
            trans.cmd = MMCSD_CMD(12U);
            trans.flags = MMCSD_CMDRSP_BUSY;
            trans.arg = 0U;
            status = MMCSD_lld_transferPoll(handle, &trans);
        }
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
        object->mmcsdTxn.flags =    MMCSD_CMDRSP_WRITE | MMCSD_CMDRSP_DATA |
                                    MMCSD_CMDREQ_WR_RD;
        object->mmcsdTxn.blockCount = numBlks;
        object->mmcsdTxn.blockSize = blockSize;
        object->mmcsdTxn.dataBuf = buf;

        if(numBlks > 1U)
        {
            object->mmcsdTxn.cmd = MMCSD_CMD(25U);
            object->mmcsdTxn.flags |= MMCSD_CMDRSP_ABORT;
        }
        else
        {
            object->mmcsdTxn.cmd = MMCSD_CMD(24);
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
        object->mmcsdTxn.flags =    MMCSD_CMDRSP_READ | MMCSD_CMDRSP_DATA |
                                    MMCSD_CMDREQ_WR_RD;
        object->mmcsdTxn.blockCount = numBlks;
        object->mmcsdTxn.blockSize = blockSize;
        object->mmcsdTxn.dataBuf = buf;

        if(numBlks > 1U)
        {
            object->mmcsdTxn.cmd = MMCSD_CMD(18U);
            object->mmcsdTxn.flags |= MMCSD_CMDRSP_ABORT;
        }
        else
        {
            object->mmcsdTxn.cmd = MMCSD_CMD(17U);
        }

        status = MMCSD_lld_transferIntr(handle, &object->mmcsdTxn);
    }

    return status;
}

int32_t MMCSD_lld_write_SD_Dma(MMCSDLLD_Handle handle, uint8_t *buf, uint32_t startBlk, uint32_t numBlks)
{
    int32_t status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object *object = (MMCSDLLD_Object *)handle;
    MMCSD_SdDeviceData *deviceData = (MMCSD_SdDeviceData *)NULL;
    uint32_t addr = 0U;
    uint32_t blockSize = MMCSD_lld_getBlockSize(handle);

    deviceData = (MMCSD_SdDeviceData *)(object->initHandle->deviceData);

    if (object->initHandle->cardType == MMCSD_CARD_TYPE_SD)
    {
        status = MMCSD_isCardReadyForTransferSD(handle);
    }
    else
    {
        status = MMCSD_STS_ERR;
    }

    if (status == MMCSD_STS_SUCCESS)
    {
        addr = deviceData->isHC ? startBlk : startBlk * blockSize;

        MMCSD_lld_initTransaction(&object->mmcsdTxn);
        object->mmcsdTxn.arg = addr;
        object->mmcsdTxn.flags = MMCSD_CMDRSP_WRITE | MMCSD_CMDRSP_DATA | MMCSD_CMDREQ_WR_RD ;
        if(numBlks>1)
        {
            object->mmcsdTxn.flags |= MMCSD_CMDRSP_ABORT;
        }
        object->mmcsdTxn.blockCount = numBlks;
        object->mmcsdTxn.blockSize = blockSize;
        object->mmcsdTxn.dataBuf = buf;

        object->mmcsdTxn.cmd = (numBlks > 1U) ? MMCSD_CMD(25U) : MMCSD_CMD(24U);

        status = MMCSD_lld_transferDma(handle, &object->mmcsdTxn);
    }

    return status;
}

int32_t MMCSD_lld_read_SD_Dma(MMCSDLLD_Handle handle, uint8_t *buf, uint32_t startBlk, uint32_t numBlks)
{
    int32_t status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object *object = (MMCSDLLD_Object *)handle;
    MMCSD_SdDeviceData *deviceData = (MMCSD_SdDeviceData *)NULL;
    uint32_t addr = 0U;
    uint32_t blockSize = MMCSD_lld_getBlockSize(handle);

    deviceData = (MMCSD_SdDeviceData *)(object->initHandle->deviceData);

    if (object->initHandle->cardType == MMCSD_CARD_TYPE_SD)
    {
        status = MMCSD_isCardReadyForTransferSD(handle);
    }
    else
    {
        status = MMCSD_STS_ERR;
    }

    if (status == MMCSD_STS_SUCCESS)
    {
        addr = deviceData->isHC ? startBlk : startBlk * blockSize;

        MMCSD_lld_initTransaction(&object->mmcsdTxn);
        object->mmcsdTxn.arg = addr;
        object->mmcsdTxn.flags = MMCSD_CMDRSP_READ | MMCSD_CMDRSP_DATA | MMCSD_CMDREQ_WR_RD;
        if(numBlks>1)
        {
            object->mmcsdTxn.flags |= MMCSD_CMDRSP_ABORT;
        }
        object->mmcsdTxn.blockCount = numBlks;
        object->mmcsdTxn.blockSize = blockSize;
        object->mmcsdTxn.dataBuf = buf;

        object->mmcsdTxn.cmd = (numBlks > 1U) ? MMCSD_CMD(18U) : MMCSD_CMD(17U);

        status = MMCSD_lld_transferDma(handle, &object->mmcsdTxn);
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

    if (object->initHandle->cardType == MMCSD_CARD_TYPE_EMMC)
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

    if (MMCSD_STS_SUCCESS == status)
    {
        if (deviceData->isHC)
        {
            /* Addressing in Blocks */
            addr = startBlk;
        }
        else
        {
            /* Addressing is in Bytes */
            addr = startBlk * blockSize;
        }

        MMCSD_lld_initTransaction(&trans);
        trans.arg = addr;
        trans.flags =   MMCSD_CMDRSP_WRITE | MMCSD_CMDRSP_DATA |
                        MMCSD_CMDREQ_WR_RD;
        trans.blockCount = numBlks;
        trans.blockSize = blockSize;
        trans.dataBuf = buf;

        if (numBlks > 1U)
        {
            trans.cmd = MMCSD_CMD(25U);
            trans.flags |= MMCSD_CMDRSP_ABORT;
        }
        else
        {
            trans.cmd = MMCSD_CMD(24U);
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

    if (object->initHandle->cardType == MMCSD_CARD_TYPE_EMMC)
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

    if (status == MMCSD_STS_SUCCESS)
    {
        if (deviceData->isHC)
        {
            addr = startBlk;
        }
        else
        {
            addr = startBlk * blockSize;
        }

        MMCSD_lld_initTransaction(&trans);
        trans.arg = addr;
        trans.flags =   MMCSD_CMDRSP_READ | MMCSD_CMDRSP_DATA |
                        MMCSD_CMDREQ_WR_RD;
        trans.blockCount = numBlks;
        trans.blockSize = blockSize;
        trans.dataBuf = buf;
        if (numBlks > 1U)
        {
            trans.cmd = MMCSD_CMD(18U);
            trans.flags |= MMCSD_CMDRSP_ABORT;
        }
        else
        {
            trans.cmd = MMCSD_CMD(17U);
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

    if (object->initHandle->cardType == MMCSD_CARD_TYPE_EMMC)
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

    if (MMCSD_STS_SUCCESS == status)
    {
        if (deviceData->isHC)
        {
            /* Addressing in Blocks */
            addr = startBlk;
        }
        else
        {
            /* Addressing is in Bytes */
            addr = startBlk * blockSize;
        }

        MMCSD_lld_initTransaction(&object->mmcsdTxn);
        object->mmcsdTxn.arg = addr;
        object->mmcsdTxn.flags =   MMCSD_CMDRSP_WRITE | MMCSD_CMDRSP_DATA |
                                   MMCSD_CMDREQ_WR_RD;
        object->mmcsdTxn.blockCount = numBlks;
        object->mmcsdTxn.blockSize = blockSize;
        object->mmcsdTxn.dataBuf = buf;

        if (numBlks > 1U)
        {
            object->mmcsdTxn.cmd = MMCSD_CMD(25U);
            object->mmcsdTxn.flags |= MMCSD_CMDRSP_ABORT;
        }
        else
        {
            object->mmcsdTxn.cmd = MMCSD_CMD(24U);
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

    if (object->initHandle->cardType == MMCSD_CARD_TYPE_EMMC)
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

    if (status == MMCSD_STS_SUCCESS)
    {
        if (deviceData->isHC)
        {
            addr = startBlk;
        }
        else
        {
            addr = startBlk * blockSize;
        }

        MMCSD_lld_initTransaction(&object->mmcsdTxn);
        object->mmcsdTxn.arg = addr;
        object->mmcsdTxn.flags =   MMCSD_CMDRSP_READ | MMCSD_CMDRSP_DATA |
                                   MMCSD_CMDREQ_WR_RD;
        object->mmcsdTxn.blockCount = numBlks;
        object->mmcsdTxn.blockSize = blockSize;
        object->mmcsdTxn.dataBuf = buf;
        if (numBlks > 1U)
        {
            object->mmcsdTxn.cmd = MMCSD_CMD(18U);
            object->mmcsdTxn.flags |= MMCSD_CMDRSP_ABORT;
        }
        else
        {
            object->mmcsdTxn.cmd = MMCSD_CMD(17U);
        }

        status = MMCSD_lld_transferIntr(handle, &object->mmcsdTxn);
    }

    return status;
}

int32_t MMCSD_lld_write_MMC_Dma(MMCSDLLD_Handle handle, uint8_t *buf, uint32_t startBlk, uint32_t numBlks)
{
    int32_t status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object *object = (MMCSDLLD_Object *)handle;
    MMCSD_EmmcDeviceData *deviceData = (MMCSD_EmmcDeviceData *)NULL;
    uint32_t addr = 0U;
    uint32_t blockSize = MMCSD_lld_getBlockSize(handle);

    deviceData = (MMCSD_EmmcDeviceData *)(object->initHandle->deviceData);

    while (MMCSD_getPstateStat(object->initHandle->baseAddr) & MMCSD_PSTATE_MASK_DATA_INHIBIT);

    if (object->initHandle->cardType == MMCSD_CARD_TYPE_EMMC)
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

    if (status == MMCSD_STS_SUCCESS)
    {
        addr = deviceData->isHC ? startBlk : startBlk * blockSize;

        MMCSD_lld_initTransaction(&object->mmcsdTxn);
        object->mmcsdTxn.arg = addr;
        object->mmcsdTxn.flags = MMCSD_CMDRSP_WRITE | MMCSD_CMDRSP_DATA | MMCSD_CMDREQ_WR_RD;
        if(numBlks>1)
        {
            object->mmcsdTxn.flags |= MMCSD_CMDRSP_ABORT;
        }
        object->mmcsdTxn.blockCount = numBlks;
        object->mmcsdTxn.blockSize = blockSize;
        object->mmcsdTxn.dataBuf = buf;
        object->mmcsdTxn.cmd = (numBlks > 1U) ? MMCSD_CMD(25U) : MMCSD_CMD(24U);

        status = MMCSD_lld_transferDma(handle, &object->mmcsdTxn);
    }

    return status;
}

int32_t MMCSD_lld_read_MMC_Dma(MMCSDLLD_Handle handle, uint8_t *buf, uint32_t startBlk, uint32_t numBlks)
{
    int32_t status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object *object = (MMCSDLLD_Object *)handle;
    MMCSD_EmmcDeviceData *deviceData = (MMCSD_EmmcDeviceData *)NULL;
    uint32_t addr = 0U;
    uint32_t blockSize = MMCSD_lld_getBlockSize(handle);

    deviceData = (MMCSD_EmmcDeviceData *)(object->initHandle->deviceData);

    while (MMCSD_getPstateStat(object->initHandle->baseAddr) & MMCSD_PSTATE_MASK_DATA_INHIBIT);

    if (object->initHandle->cardType == MMCSD_CARD_TYPE_EMMC)
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

    if (status == MMCSD_STS_SUCCESS)
    {
        addr = deviceData->isHC ? startBlk : startBlk * blockSize;

        MMCSD_lld_initTransaction(&object->mmcsdTxn);
        object->mmcsdTxn.arg = addr;
        object->mmcsdTxn.flags = MMCSD_CMDRSP_READ | MMCSD_CMDRSP_DATA | MMCSD_CMDREQ_WR_RD;
        if(numBlks>1)
        {
            object->mmcsdTxn.flags |= MMCSD_CMDRSP_ABORT;
        }
        object->mmcsdTxn.blockCount = numBlks;
        object->mmcsdTxn.blockSize = blockSize;
        object->mmcsdTxn.dataBuf = buf;
        object->mmcsdTxn.cmd = (numBlks > 1U) ? MMCSD_CMD(18U) : MMCSD_CMD(17U);

        status = MMCSD_lld_transferDma(handle, &object->mmcsdTxn);
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
    uint16_t                intrStatus = 0U;
    uint32_t                offset = 0U;
    uint32_t                dataLength = 0U;
    // uint8_t                 *pTempWord = NULL;
    uint32_t                tempWord = 0xFFFFFFFFU;
    uint32_t                i = 0;

    object = (MMCSDLLD_Object *)args;

    if(args != NULL_PTR)
    {
        /* Get interrupt stats */
        intrStatus = MMCSD_getIntrStat(object->initHandle->baseAddr);

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
                if((intrStatus & MMCSD_INTR_MASK_CMDCOMP) != (uint16_t)0U)
                {
                    /* Clear the Interrupt */
                    MMCSD_clearIntrStat(object->initHandle->baseAddr,
                                        MMCSD_INTR_MASK_CMDCOMP);

                    /* Check if transfer has data or not */
                    if((object->currentTxn->flags & MMCSD_CMDRSP_DATA) != 0U)
                    {
                        /* Transfer Has Data */
                        /* Get and store response in transaction object */
                        MMCSD_getResponse(  object->initHandle->baseAddr,
                                            object->currentTxn->response);

                        if((object->currentTxn->flags & MMCSD_CMDRSP_WRITE) != 0U)
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
                        /* Get and store response in transaction object */
                        MMCSD_getResponse(  object->initHandle->baseAddr,
                                            object->currentTxn->response);
                        /* Reset interrupt configuration */
                        MMCSD_intrConfigReset(object);
                        /* Success Case */
                        if(object->currentTxn->cmd == MMCSD_CMD(12U))
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
                else if((intrStatus & 0xFFFF0000U) != 0U)
                {
                    /* Interrupt Happened due to some Error */
                    /* Store Command Error */
                    object->cmdErrorStat = (intrStatus & 0xFFFF0000U);
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
                if((intrStatus & MMCSD_INTR_MASK_BUFWRRDY) != 0U)
                {
                    /* Clear the interrupt */
                    MMCSD_clearIntrStat(object->initHandle->baseAddr,
                                        MMCSD_INTR_MASK_BUFWRRDY);

                    if(object->remainingBlockCount != 0U)
                    {
                        dataLength = object->dataBlockSize;
                        offset = (( (object->dataBlockCount) -
                                    (object->remainingBlockCount)) *
                                    (object->dataBlockSize));

                        for(i = 0U; i < dataLength; i += 4U)
                        {
                            *((uint8_t *)&tempWord) = object->dataBufIdx[offset + i];
                            *((uint8_t *)&tempWord + 1U) = object->dataBufIdx[offset + i + 1U];
                            *((uint8_t *)&tempWord + 2U) = object->dataBufIdx[offset + i + 2U];
                            *((uint8_t *)&tempWord + 3U) = object->dataBufIdx[offset + i + 3U];

                            MMCSD_setData(  object->initHandle->baseAddr,
                                            (uint8_t*)&tempWord, 4U);
                        }

                        object->remainingBlockCount--;
                    }
                    else
                    {
                        /* Remaining Block Count is zero, do nothing */
                    }
                }
                /* Check if transfer Complete interrupt Happened */
                else if((intrStatus & MMCSD_INTR_MASK_TRNFCOMP) != 0U)
                {
                    /* Clear the interrupt */
                    MMCSD_clearIntrStat(object->initHandle->baseAddr,
                                        MMCSD_INTR_MASK_TRNFCOMP);
                    /* Reset interrupt configuration */
                    MMCSD_intrConfigReset(object);
                    /* Through the SD Back into trans State */
                    if(object->currentTxn->blockCount > 1U)
                    {
                        if(object->initHandle->cardType == MMCSD_CARD_TYPE_SD)
                        {
                            MMCSD_lld_initTransaction(&object->mmcsdTxn);
                            object->mmcsdTxn.cmd = MMCSD_CMD(12U);
                            object->mmcsdTxn.flags = MMCSD_CMDRSP_BUSY;
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

                else if((intrStatus & 0xFFFF0000U) != 0U)
                {
                    /* Interrupt Happened due to some Error */
                    /* Store Data Error */
                    object->xferErrorStat = (intrStatus & 0xFFFF0000U);
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
                if((intrStatus & MMCSD_INTR_MASK_BUFRDRDY) != 0U)
                {
                    /* Clear the interrupt */
                    MMCSD_clearIntrStat(object->initHandle->baseAddr,
                                        MMCSD_INTR_MASK_BUFRDRDY);

                    if(object->remainingBlockCount != 0U)
                    {
                        dataLength = object->dataBlockSize;
                        offset = (( (object->dataBlockCount) -
                                    (object->remainingBlockCount)) *
                                    (object->dataBlockSize));

                        for(i = 0U; i < dataLength; i += 4U)
                        {
                            MMCSD_getData(  object->initHandle->baseAddr,
                                            (uint8_t*)&tempWord, 4U);

                            object->dataBufIdx[offset + i + 0U] = *((uint8_t *)&tempWord);
                            object->dataBufIdx[offset + i + 1U] = *((uint8_t *)&tempWord + 1U);
                            object->dataBufIdx[offset + i + 2U] = *((uint8_t *)&tempWord + 2U);
                            object->dataBufIdx[offset + i + 3U] = *((uint8_t *)&tempWord + 3U);
                        }

                        object->remainingBlockCount--;
                    }
                    else
                    {
                        /* Remaining Block Count is zero, do nothing */
                    }
                }
                else if((intrStatus & MMCSD_INTR_MASK_TRNFCOMP) != 0U)
                {
                    /* Clear the transfer complete interrupt */
                    MMCSD_clearIntrStat(object->initHandle->baseAddr,
                                        MMCSD_INTR_MASK_TRNFCOMP);
                    /* Reset interrupt configuration */
                    MMCSD_intrConfigReset(object);
                    /* Success Case */
                    xferStatus = MMCSD_STS_SUCCESS;
                    /* Callback to application and finishup */
                    MMCSD_lld_completeCurrTransfer(object, xferStatus);
                }
                else if((intrStatus & 0xFFFF0000U) != 0U)
                {
                    /* Interrupt Happened due to some Error */
                    /* Store Data Error */
                    object->xferErrorStat = (intrStatus & 0xFFFF0000U);
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

static int32_t MMCSD_softReset(uint32_t baseAddr, uint32_t loopTimeout)
{
    int32_t         retVal = MMCSD_STS_SUCCESS;
    uint32_t        timeout = loopTimeout;

    HW_WR_FIELD32((baseAddr + CSL_MMC_SYSCONFIG),
        CSL_MMC_SYSCONFIG_SOFTRESET, CSL_MMC_SYSCONFIG_SOFTRESET_ST_RST_W);

    while(CSL_MMC_SYSCONFIG_SOFTRESET_ONRESET_R !=
            HW_RD_FIELD32((baseAddr + CSL_MMC_SYSSTATUS),
            CSL_MMC_SYSSTATUS_RESETDONE))
    {
        timeout--;
        if(timeout == 0U)
        {
            retVal = MMCSD_STS_ERR;
            break;
        }
    }

    return retVal;
}

static int32_t MMCSD_linesResetCmd(uint32_t baseAddr, uint32_t loopTimeout)
{
    int32_t         status = MMCSD_STS_SUCCESS;
    uint32_t        timeout = loopTimeout;
    uint32_t        regVal = 0U;

    regVal = HW_RD_REG32(baseAddr + CSL_MMC_SYSCTL);
    regVal |= CSL_MMC_SYSCTL_SRC_MASK;
    HW_WR_REG32((baseAddr + CSL_MMC_SYSCTL), regVal);

    while(  (0U != (HW_RD_REG32(baseAddr + CSL_MMC_SYSCTL) & CSL_MMC_SYSCTL_SRC_MASK)) &&
            (timeout > 0U))
    {
        timeout--;
    }

    if(timeout == 0U)
    {
        status = MMCSD_STS_ERR_TIMEOUT;
    }

    return status;
}

static int32_t MMCSD_linesResetDat(uint32_t baseAddr, uint32_t loopTimeout)
{
    int32_t         status = MMCSD_STS_SUCCESS;
    uint32_t        timeout = loopTimeout;
    uint32_t        regVal = 0U;

    regVal = HW_RD_REG32(baseAddr + CSL_MMC_SYSCTL);
    regVal |= CSL_MMC_SYSCTL_SRD_MASK;
    HW_WR_REG32((baseAddr + CSL_MMC_SYSCTL), regVal);

    while(  (0U != ((HW_RD_REG32(baseAddr + CSL_MMC_SYSCTL) & CSL_MMC_SYSCTL_SRD_MASK) & CSL_MMC_SYSCTL_SRD_MASK)) &&
            (timeout > 0U))
    {
        timeout--;
    }

    if(timeout == 0U)
    {
        status = MMCSD_STS_ERR_TIMEOUT;
    }

    return status;
}

static int32_t MMCSD_linesResetAll(uint32_t baseAddr, uint32_t loopTimeout)
{
    int32_t         status = MMCSD_STS_SUCCESS;
    uint32_t        timeout = loopTimeout;
    uint32_t        regVal = 0U;

    regVal = HW_RD_REG32(baseAddr + CSL_MMC_SYSCTL);
    regVal |= CSL_MMC_SYSCTL_SRA_MASK;
    HW_WR_REG32((baseAddr + CSL_MMC_SYSCTL), regVal);

    while(  (0U != (HW_RD_REG32(baseAddr + CSL_MMC_SYSCTL) & CSL_MMC_SYSCTL_SRA_MASK)) &&
            (timeout > 0U))
    {
        timeout--;
    }

    if(timeout == 0U)
    {
        status = MMCSD_STS_ERR_TIMEOUT;
    }

    return status;
}

static void MMCSD_systemConfig(uint32_t baseAddr, const MMCSD_sysCfg *pCfg)
{
    uint32_t        regVal = 0U;

    if(pCfg != NULL)
    {
        regVal |=   (   (pCfg->clockActivity) <<
                        (uint32_t)(CSL_MMC_SYSCONFIG_CLOCKACTIVITY_SHIFT));
        regVal |=   (   (pCfg->standbyMode) <<
                        (uint32_t)(CSL_MMC_SYSCONFIG_STANDBYMODE_SHIFT));
        regVal |=   (   (pCfg->idleMode) <<
                        (uint32_t)(CSL_MMC_SYSCONFIG_SIDLEMODE_SHIFT));

        if((pCfg->enableWakeup) == true)
        {
            regVal |=   (   (uint32_t)CSL_MMC_SYSCONFIG_ENAWAKEUP_ENABLE <<
                            (uint32_t)CSL_MMC_SYSCONFIG_ENAWAKEUP_SHIFT);
        }
        if((pCfg->enableAutoIdle) == true)
        {
            regVal |=   (   (uint32_t)CSL_MMC_SYSCONFIG_AUTOIDLE_ON <<
                            (uint32_t)CSL_MMC_SYSCONFIG_AUTOIDLE_SHIFT);
        }
    }

    HW_WR_REG32((baseAddr + CSL_MMC_SYSCONFIG), regVal);
}

static void MMCSD_setBusWidth(uint32_t baseAddr, uint32_t busWidth)
{
    switch (busWidth)
    {
        case MMCSD_BUS_WIDTH_8BIT:
        {
            HW_WR_FIELD32((baseAddr + CSL_MMC_CON),
                CSL_MMC_CON_DW8, CSL_MMC_CON_DW8__8BITMODE);
        }
        break;

        case MMCSD_BUS_WIDTH_4BIT:
        {
            HW_WR_FIELD32((baseAddr + CSL_MMC_CON),
                CSL_MMC_CON_DW8, CSL_MMC_CON_DW8__1_4BITMODE);
            HW_WR_FIELD32((baseAddr + CSL_MMC_HCTL),
                CSL_MMC_HCTL_DTW, CSL_MMC_HCTL_DTW__4_BITMODE);
        }
        break;

        case MMCSD_BUS_WIDTH_1BIT:
        {
            HW_WR_FIELD32((baseAddr + CSL_MMC_CON),
                CSL_MMC_CON_DW8, CSL_MMC_CON_DW8__1_4BITMODE);
            HW_WR_FIELD32((baseAddr + CSL_MMC_HCTL),
                CSL_MMC_HCTL_DTW, CSL_MMC_HCTL_DTW__1_BITMODE);
        }
        break;

        default:
        break;
    }
}

static void MMCSD_setBusVolt(uint32_t baseAddr, uint32_t busVoltage)
{
    HW_WR_FIELD32((baseAddr + CSL_MMC_HCTL), CSL_MMC_HCTL_SDVS, busVoltage);
}

static uint32_t MMCSD_getBusVolt(uint32_t baseAddr)
{
    CSL_MmcsdRegs   mmcsdRegs = (CSL_MmcsdRegs)baseAddr;
    uint32_t        regVal = 0U;

    regVal = mmcsdRegs->HCTL;
    return((regVal & CSL_MMC_HCTL_SDVS_MASK) >> CSL_MMC_HCTL_SDVS_SHIFT);
}


static int32_t MMCSD_busPowerOnCtrl(uint32_t baseAddr, bool pwrCtrl,
                                    uint32_t loopTimeout)
{
    int32_t retVal = MMCSD_STS_SUCCESS;
    uint32_t regVal = 0U;
    uint32_t        timeout = loopTimeout;

    if(pwrCtrl == true)
    {
        regVal =  MMCSD_PWR_CTRL_ON;
    }

    HW_WR_FIELD32((baseAddr + CSL_MMC_HCTL), CSL_MMC_HCTL_SDBP, regVal);

    if (MMCSD_PWR_CTRL_ON == regVal)
    {
        while (regVal != HW_RD_FIELD32((baseAddr + CSL_MMC_HCTL),
            CSL_MMC_HCTL_SDBP))
        {
            timeout--;
            if(timeout == 0U)
            {
                retVal = MMCSD_STS_ERR;
                break;
            }
        }
    }

    return retVal;
}

static bool MMCSD_isIntClockStable(uint32_t baseAddr, uint32_t loopTimeout)
{
    uint32_t        status = false;
    uint32_t        timeout = loopTimeout;

    do
    {
        if( (CSL_MMC_SYSCTL_ICS_READY) ==
            ((HW_RD_FIELD32((baseAddr + CSL_MMC_SYSCTL), CSL_MMC_SYSCTL_ICS)) & (CSL_MMC_SYSCTL_ICS_READY)))
        {
            status = true;
            break;
        }
        else
        {
            timeout--;
        }
    }
    while (timeout != 0U);

    return status;
}

static int32_t MMCSD_intClockEnable(uint32_t baseAddr, bool enableIntClk)
{
    uint32_t clkEnable = 0U;
    int32_t retVal = MMCSD_STS_SUCCESS;

    if(enableIntClk)
    {
        clkEnable = CSL_MMC_SYSCTL_ICE_OSCILLATE;
    }
    else
    {
        clkEnable = CSL_MMC_SYSCTL_ICE_STOP;
    }

    HW_WR_FIELD32(  (baseAddr + CSL_MMC_SYSCTL), CSL_MMC_SYSCTL_ICE,
                    clkEnable);

    if (enableIntClk)
    {
        if (!MMCSD_isIntClockStable(baseAddr, 0xFFFFU))
        {
            retVal = MMCSD_STS_ERR;
        }
    }

    return retVal;
}

static void MMCSD_setSupportedVoltage(uint32_t baseAddr, uint32_t voltMask)
{
    CSL_MmcsdRegs   mmcsdRegs = (CSL_MmcsdRegs)baseAddr;
    uint32_t        regVal = 0U;

    regVal = mmcsdRegs->CAPA;
    regVal &= ~(    (uint32_t)CSL_MMC_CAPA_VS33_MASK |
                    (uint32_t)CSL_MMC_CAPA_VS30_MASK |
                    (uint32_t)CSL_MMC_CAPA_VS18_MASK);

    regVal |= voltMask;
    mmcsdRegs->CAPA = regVal;
}

static bool MMCSD_isHighSpeedSupported(uint32_t baseAddr)
{
    CSL_MmcsdRegs   mmcsdRegs = (CSL_MmcsdRegs)baseAddr;
    bool            retVal = false;

    if (((mmcsdRegs->CAPA) & (CSL_MMC_CAPA_HSS_MASK)) != 0U)
    {
        retVal = true;
    }

    return retVal;
}

static bool MMCSD_isUHSSDR50Supported(uint32_t baseAddr)
{
    CSL_MmcsdRegs   mmcsdRegs = (CSL_MmcsdRegs)baseAddr;
    bool            retVal = false;

    if (((mmcsdRegs->CAPA2) & (CSL_MMC_CAPA2_SDR50_MASK)) != 0U)
    {
        retVal = true;
    }

    return retVal;
}

static bool MMCSD_isUHSSDR104Supported(uint32_t baseAddr)
{
    CSL_MmcsdRegs   mmcsdRegs = (CSL_MmcsdRegs)baseAddr;
    bool            retVal = false;

    if (((mmcsdRegs->CAPA2) & (CSL_MMC_CAPA2_SDR104_MASK)) != 0U)
    {
        retVal = true;
    }

    return retVal;
}

static bool MMCSD_isUHSDDR50Supported(uint32_t baseAddr)
{
    CSL_MmcsdRegs   mmcsdRegs = (CSL_MmcsdRegs)baseAddr;
    bool            retVal = false;

    if (((mmcsdRegs->CAPA2) & (CSL_MMC_CAPA2_DDR50_MASK)) != 0U)
    {
        retVal = true;
    }

    return retVal;
}

static void MMCSD_setDataTimeout(uint32_t baseAddr, uint32_t timeout)
{
    CSL_MmcsdRegs   mmcsdRegs = (CSL_MmcsdRegs)baseAddr;
    uint32_t        regVal = 0U;

    timeout = (timeout - 13U) & 0xFU;

    regVal = mmcsdRegs->SYSCTL;
    regVal &= ~(CSL_MMC_SYSCTL_DTO_MASK);
    regVal |= (timeout << CSL_MMC_SYSCTL_DTO_SHIFT);
    mmcsdRegs->SYSCTL = regVal;
}

static bool MMCSD_isCardInserted(uint32_t baseAddr)
{
    bool retVal = false;
    uint32_t cins = 0U;

    cins = HW_RD_FIELD32(baseAddr + CSL_MMC_PSTATE, CSL_MMC_PSTATE_CINS);

    if(cins != 0U)
    {
        retVal = true;
    }

    return retVal;
}

static int32_t MMCSD_setBusFreq(uint32_t baseAddr, uint32_t inputFreq,
                                uint32_t outputFreq)
{
    uint32_t        clkDiv = 0U;
    int32_t         retVal = MMCSD_STS_SUCCESS;

    /* First enable the internal clocks */
    retVal = MMCSD_intClockEnable(baseAddr, true);

    if (MMCSD_STS_SUCCESS == retVal)
    {
        /* Calculate and program the divisor */
        clkDiv = inputFreq / outputFreq;
        if(clkDiv > 1023U)
        {
            clkDiv = 1023U;
        }
        else if (clkDiv == 0)
        {
            clkDiv = 1;
        }

        /* Do not cross the required freq */
        while(  ((inputFreq / clkDiv) > outputFreq) &&
                (MMCSD_STS_SUCCESS == retVal))
        {
            if (1023U == clkDiv)
            {
                /* Clock frequency Can not be set */
                retVal = MMCSD_STS_ERR;
            }
            clkDiv++;
        }

        if(MMCSD_STS_SUCCESS == retVal)
        {
            HW_WR_FIELD32((baseAddr + CSL_MMC_SYSCTL),
                          CSL_MMC_SYSCTL_CLKD, clkDiv);

            /* Wait for the interface clock stabilization */
            if(true == MMCSD_isIntClockStable(baseAddr, 0xFFFFU))
            {
                /* Enable clock to the card */
                HW_WR_FIELD32(  (baseAddr + CSL_MMC_SYSCTL),
                                CSL_MMC_SYSCTL_CEN,
                                CSL_MMC_SYSCTL_CEN_ENABLE);
            }
            else
            {
                retVal = MMCSD_STS_ERR;
            }
        }
    }
    else
    {
        retVal = MMCSD_STS_ERR;
    }

    return retVal;
}

static void MMCSD_setBlkLength(uint32_t baseAddr, uint32_t blkLen)
{
    HW_WR_FIELD32((baseAddr + CSL_MMC_BLK), CSL_MMC_BLK_BLEN, blkLen);
}

static uint32_t MMCSD_getBlkLength(uint32_t baseAddr)
{
    CSL_MmcsdRegs   mmcsdRegs = (CSL_MmcsdRegs)baseAddr;
    uint32_t        regVal = 0U;

    regVal = (  (mmcsdRegs->BLK & CSL_MMC_BLK_BLEN_MASK) >>
                (CSL_MMC_BLK_BLEN_SHIFT));

    return regVal;
}

static uint32_t MMCSD_getBlkCount(uint32_t baseAddr)
{
    CSL_MmcsdRegs   mmcsdRegs = (CSL_MmcsdRegs)baseAddr;
    uint32_t        regVal = 0U;

    regVal = (  (mmcsdRegs->BLK & CSL_MMC_BLK_NBLK_MASK) >>
                (CSL_MMC_BLK_NBLK_SHIFT));

    return regVal;
}

static void MMCSD_getData(uint32_t baseAddr, uint8_t *pData, uint32_t len)
{
    uint32_t idx;

    for (idx = 0U; idx < (len / 4U); idx++)
    {
        ((uint32_t*)pData)[idx] = HW_RD_REG32(baseAddr + CSL_MMC_DATA);
    }
}

static void MMCSD_setData(uint32_t baseAddr, const uint8_t *pData, uint32_t len)
{
    uint32_t idx;

    for (idx = 0U; idx < (len / 4U); idx++)
    {
        HW_WR_REG32((baseAddr + CSL_MMC_DATA), ((uint32_t*)pData)[idx]);
    }
}

static void MMCSD_setDLLSWT(uint32_t baseAddr, uint32_t val)
{
    CSL_MmcsdRegs   mmcsdRegs = (CSL_MmcsdRegs)baseAddr;
    uint32_t        regVal = mmcsdRegs->DLL;

    regVal &= ~(CSL_MMC_DLL_FORCE_SWT_MASK);
    regVal |= (val << CSL_MMC_DLL_FORCE_SWT_SHIFT);
    mmcsdRegs->DLL = regVal;
}

static uint32_t MMCSD_getDLLSWT(uint32_t baseAddr)
{
    CSL_MmcsdRegs   mmcsdRegs = (CSL_MmcsdRegs)baseAddr;
    uint32_t        regVal = 0U;

    regVal = (  (mmcsdRegs->DLL & CSL_MMC_DLL_FORCE_SWT_MASK) >>
                (CSL_MMC_DLL_FORCE_SWT_SHIFT));

    return regVal;
}

static uint32_t MMCSD_getCAPA(uint32_t baseAddr)
{
    CSL_MmcsdRegs   mmcsdRegs = (CSL_MmcsdRegs)baseAddr;
    uint32_t        regVal = 0U;

    regVal = mmcsdRegs->CAPA;

    return regVal;
}

static void MMCSD_enableIntrStatus(uint32_t baseAddr, uint32_t intrMask)
{
    uint32_t regVal = HW_RD_REG32(baseAddr + CSL_MMC_IE);
    regVal |= intrMask;
    HW_WR_REG32((baseAddr + CSL_MMC_IE), regVal);
}

static void MMCSD_disableIntrStatus(uint32_t baseAddr, uint32_t intrMask)
{
    uint32_t regVal = HW_RD_REG32(baseAddr + CSL_MMC_IE);
    regVal &= ~intrMask;
    HW_WR_REG32((baseAddr + CSL_MMC_IE), regVal);
}

static void MMCSD_enableSigIntr(uint32_t baseAddr, uint32_t intrMask)
{
    uint32_t regVal = 0U;

    regVal = HW_RD_REG32(baseAddr + CSL_MMC_ISE);
    regVal |= intrMask;
    HW_WR_REG32((baseAddr + CSL_MMC_ISE), regVal);

    MMCSD_enableIntrStatus(baseAddr, intrMask);
}

static void MMCSD_disableSigIntr(uint32_t baseAddr, uint32_t intrMask)
{
    uint32_t regVal = 0U;

    regVal = HW_RD_REG32(baseAddr + CSL_MMC_ISE);
    regVal &= ~intrMask;
    HW_WR_REG32((baseAddr + CSL_MMC_ISE), regVal);
}

static uint32_t MMCSD_getIntrSigEnable(uint32_t baseAddr)
{
    CSL_MmcsdRegs   mmcsdRegs = (CSL_MmcsdRegs)baseAddr;
    return (uint32_t)(mmcsdRegs->ISE);
}

static uint32_t MMCSD_getIntrStat(uint32_t baseAddr)
{
    return HW_RD_REG32(baseAddr + CSL_MMC_STAT);
}

static uint32_t MMCSD_getPstateStat(uint32_t baseAddr)
{
    return HW_RD_REG32(baseAddr + CSL_MMC_PSTATE);
}

static void MMCSD_clearIntrStat(uint32_t baseAddr, uint32_t intrMask)
{
    HW_WR_REG32((baseAddr + CSL_MMC_STAT), intrMask);
}

static void MMCSD_setPADEN(uint32_t baseAddr, uint32_t val)
{
    CSL_MmcsdRegs   mmcsdRegs = (CSL_MmcsdRegs)baseAddr;
    mmcsdRegs->CON |= ((val << CSL_MMC_CON_PADEN_SHIFT) &
                        CSL_MMC_CON_PADEN_MASK);
}

static uint32_t MMCSD_getCmdSignalLevel(uint32_t baseAddr)
{
    CSL_MmcsdRegs   mmcsdRegs = (CSL_MmcsdRegs)baseAddr;
    uint32_t        retVal = 0U;

    retVal = (  ((mmcsdRegs->PSTATE) & CSL_MMC_PSTATE_CLEV_MASK) >>
                CSL_MMC_PSTATE_CLEV_SHIFT);
    return retVal;
}

static uint32_t MMCSD_getDataSignalLevel(uint32_t baseAddr)
{
    CSL_MmcsdRegs   mmcsdRegs = (CSL_MmcsdRegs)baseAddr;
    uint32_t        retVal = 0U;

    retVal = (  ((mmcsdRegs->PSTATE) & CSL_MMC_PSTATE_DLEV_MASK) >>
                CSL_MMC_PSTATE_DLEV_SHIFT);
    return retVal;
}

static uint32_t MMCSD_isCardWriteProtected(uint32_t baseAddr)
{
    CSL_MmcsdRegs   mmcsdRegs = (CSL_MmcsdRegs)baseAddr;
    uint32_t        retVal = 0U;

    retVal = (  ((mmcsdRegs->PSTATE) & CSL_MMC_PSTATE_WP_MASK) >>
                CSL_MMC_PSTATE_WP_SHIFT);
    return retVal;
}

static int32_t MMCSD_initStreamSend(uint32_t baseAddr)
{
    uint32_t status = 0U;

    /* Enable the command completion status to be set */
    MMCSD_enableIntrStatus(baseAddr, MMCSD_INTR_MASK_CMDCOMP);

    /* Initiate the INIT command */
    HW_WR_FIELD32((baseAddr + CSL_MMC_CON), CSL_MMC_CON_INIT,
        CSL_MMC_CON_INIT_INITSTREAM);
    HW_WR_REG32((baseAddr + CSL_MMC_CMD), 0x00U);

    status = MMCSD_isCmdComplete(baseAddr, 0xFFFFU);

    HW_WR_FIELD32((baseAddr + CSL_MMC_CON), CSL_MMC_CON_INIT,
        CSL_MMC_CON_INIT_NOINIT);
    /* Clear all status */
    MMCSD_clearIntrStat(baseAddr, 0xFFFFFFFFU);

    return((int32_t)status);
}

static void MMCSD_commandSend(uint32_t baseAddr, const MMCSD_cmdObj *pObj)
{
    uint32_t cmdRegVal = 0U;

    HW_SET_FIELD(cmdRegVal, CSL_MMC_CMD_INDX, pObj->cmd.cmdId);
    HW_SET_FIELD(cmdRegVal, CSL_MMC_CMD_CMD_TYPE, pObj->cmd.cmdType);
    HW_SET_FIELD(cmdRegVal, CSL_MMC_CMD_RSP_TYPE, pObj->cmd.rspType);
    HW_SET_FIELD(cmdRegVal, CSL_MMC_CMD_DDIR, pObj->cmd.xferType);

    if (TRUE == pObj->enableData)
    {
        HW_SET_FIELD(cmdRegVal, CSL_MMC_CMD_DP, CSL_MMC_CMD_DP_DATA);
        HW_SET_FIELD(cmdRegVal, CSL_MMC_CMD_MSBS, CSL_MMC_CMD_MSBS_MULTIBLK);
        HW_SET_FIELD(cmdRegVal, CSL_MMC_CMD_BCE, CSL_MMC_CMD_BCE_ENABLE);
    }

    if (TRUE == pObj->enableDma)
    {
        HW_SET_FIELD(cmdRegVal, CSL_MMC_CMD_DE, CSL_MMC_CMD_DE_ENABLE);
    }

    /* Set the block information; block length is specified separately */
    HW_WR_FIELD32((baseAddr + CSL_MMC_BLK), CSL_MMC_BLK_NBLK,
        pObj->numBlks);

	/* Set the command/command argument */
    HW_WR_REG32((baseAddr + CSL_MMC_ARG), pObj->cmdArg);
    HW_WR_REG32((baseAddr + CSL_MMC_CMD), cmdRegVal);
}

static int32_t MMCSD_isCmdComplete(uint32_t baseAddr, uint32_t loopTimeout)
{
    int32_t         status = MMCSD_STS_ERR;
    uint32_t        timeout = loopTimeout;

    do
    {
        if( (CSL_MMC_STAT_CC_MASK) ==
            ((HW_RD_FIELD32((baseAddr + CSL_MMC_STAT), CSL_MMC_STAT_CC)) & (CSL_MMC_STAT_CC_MASK)))
        {
            status = MMCSD_STS_SUCCESS;
            break;
        }
        else
        {
            timeout--;
        }
    }
    while (timeout != 0U);

    return status;
}

static int32_t MMCSD_isXferComplete(uint32_t baseAddr, uint32_t loopTimeout)
{
    int32_t         status = MMCSD_STS_ERR;
    uint32_t        timeout = loopTimeout;

    do
    {
        if( (CSL_MMC_STAT_TC_MASK) ==
            (HW_RD_FIELD32((baseAddr + CSL_MMC_STAT), CSL_MMC_STAT_TC) & (CSL_MMC_STAT_TC_MASK)))
        {
            status = MMCSD_STS_SUCCESS;
            break;
        }
        else
        {
            timeout--;
        }
    }
    while (timeout != 0U);

    return status;
}

static void MMCSD_getResponse(uint32_t baseAddr, uint32_t *pRsp)
{
    pRsp[0U] = HW_RD_REG32(baseAddr + CSL_MMC_RSP10);
    pRsp[1U] = HW_RD_REG32(baseAddr + CSL_MMC_RSP32);
    pRsp[2U] = HW_RD_REG32(baseAddr + CSL_MMC_RSP54);
    pRsp[3U] = HW_RD_REG32(baseAddr + CSL_MMC_RSP76);
}



/* API Assist internal Functions */
static int32_t MMCSD_lld_initSD(MMCSDLLD_Handle handle)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;
    MMCSD_SdDeviceData      *sdDeviceData = (MMCSD_SdDeviceData *)NULL;
    MMCSDLLD_Transaction    trans;
    uint32_t                retry = 0xFFFFU;
    uint32_t                currState = MMCSD_SD_INIT_STATE_CONTROLLER_INIT;
    bool                    tuningRequired = false;
    bool                    initDone = false;

    uint32_t currBusVoltage;

    MMCSD_sysCfg sysCfg = { MMCSD_CLK_ACT_ICLK_FCLK_OFF,
                            MMCSD_STANDBY_MODE_FORCE,
                            MMCSD_IDLE_MODE_FORCE,
                            FALSE,
                            TRUE};

    sdDeviceData = (MMCSD_SdDeviceData *)object->initHandle->deviceData;

    while((initDone == false) && (status == MMCSD_STS_SUCCESS))
    {
        switch(currState)
        {
            case MMCSD_SD_INIT_STATE_CONTROLLER_INIT:
            {
                if(status == MMCSD_STS_SUCCESS)
                {
                    /* Soft Reset The IP */
                    status = MMCSD_softReset(object->initHandle->baseAddr,
                                             MMCSD_WAIT_FOREVER);
                }

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* Reset Lines */
                    MMCSD_linesResetAll(object->initHandle->baseAddr,
                                        MMCSD_WAIT_FOREVER);

                    MMCSD_setSupportedVoltage(  object->initHandle->baseAddr,
                                                (   MMCSD_SUPP_VOLT_1P8     |
                                                    MMCSD_SUPP_VOLT_3P3));
                    /* Set System Configuration */
                    MMCSD_systemConfig(object->initHandle->baseAddr, &sysCfg);

                    /* Set Bus width to 1 as SD will start with 1 on Power up */
                    MMCSD_setBusWidth(object->initHandle->baseAddr,
                                      MMCSD_BUS_WIDTH_1BIT);

                    /* Default bus voltage set to 3.3V */
                    MMCSD_setBusVolt(object->initHandle->baseAddr,
                                     MMCSD_BUS_VOLT_3_3V);


                    /* Wait for card detect */
                    retry = 0xFFFFU;
                    while(!MMCSD_isCardInserted(object->initHandle->baseAddr))
                    {
                        retry--;
                        if(retry == 0U)
                        {
                            break;
                        }
                    }

                    if(retry == 0U)
                    {
                        status = MMCSD_STS_ERR_CARD_NOT_FOUND;
                    }
                }

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* Turn on Bus Power */
                    status = MMCSD_busPowerOnCtrl(
                                            object->initHandle->baseAddr,
                                            MMCSD_PWR_CTRL_ON,
                                            MMCSD_WAIT_FOREVER);
                }

                if(status == MMCSD_STS_SUCCESS)
                {
                    status = MMCSD_setBusFreq(  object->initHandle->baseAddr,
                                                object->initHandle->inputClkFreq,
                                                400000U);
                }

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* Sent Init Stream */

                    /* Enable the command completion status to be set */
                    MMCSD_enableIntrStatus(object->initHandle->baseAddr, MMCSD_INTR_MASK_CMDCOMP);

                    /* Initiate the INIT command */
                    HW_WR_FIELD32(( object->initHandle->baseAddr + CSL_MMC_CON), CSL_MMC_CON_INIT,
                                    CSL_MMC_CON_INIT_INITSTREAM);
                    HW_WR_REG32((object->initHandle->baseAddr + CSL_MMC_CMD), 0x00U);

                    status = MMCSD_isCmdComplete(object->initHandle->baseAddr, 0xFFFFU);

                    HW_WR_FIELD32(  (object->initHandle->baseAddr + CSL_MMC_CON), CSL_MMC_CON_INIT,
                                    CSL_MMC_CON_INIT_NOINIT);

                    /* Clear all status */
                    MMCSD_clearIntrStat(object->initHandle->baseAddr, 0xFFFFFFFFU);
                }

                if(status == MMCSD_STS_SUCCESS)
                {
                    currState = MMCSD_SD_INIT_STATE_CHECK_VOLTAGE;
                }
            }
            break;

            case MMCSD_SD_INIT_STATE_CHECK_VOLTAGE:
            {
                /* Send CMD 0 (Reset) */
                MMCSD_lld_initTransaction(&trans);
                trans.cmd = MMCSD_CMD(0U);
                trans.flags = MMCSD_CMDRSP_NONE;
                trans.arg = 0U;
                status = MMCSD_lld_transferPoll(handle, &trans);

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* CMD 8 to set Operating Voltage */
                    MMCSD_lld_initTransaction(&trans);
                    trans.cmd = MMCSD_CMD(8);
                    trans.flags = 0U;
                    trans.arg = MMCSD_SD_CMD8_CHECK_PATTERN;
                    currBusVoltage =
                                MMCSD_getBusVolt(object->initHandle->baseAddr);

                    if(currBusVoltage == MMCSD_BUS_VOLT_1P8)
                    {
                        trans.arg |= (MMCSD_SD_VOLT_LOW_RANGE << 8U);
                    }
                    else
                    {
                        trans.arg |= (MMCSD_SD_VOLT_2P7_3P6 << 8U);
                    }

                    status = MMCSD_lld_transferPoll(handle, &trans);
                               }

                if(status == MMCSD_STS_SUCCESS)
                {
                    currState = MMCSD_SD_INIT_STATE_CARD_IDENTIFICATION;
                }
            }
            break;

            case MMCSD_SD_INIT_STATE_CARD_IDENTIFICATION:
            {
                retry = 0xFFFFU;
                do
                {
                    MMCSD_lld_initTransaction(&trans);
                    trans.cmd = MMCSD_CMD(55U);
                    trans.flags = 0U;
                    trans.arg = 0U;
                    status = MMCSD_lld_transferPoll(handle, &trans);

                    if(status == MMCSD_STS_SUCCESS)
                    {
                        /* Sending ACMD41 with host capabilities (host capacity support information) */
                        MMCSD_lld_initTransaction(&trans);
                        trans.cmd = MMCSD_CMD(41U);
                        trans.flags = 0U;
                        trans.arg = MMCSD_OCR_HIGH_CAPACITY |
                                    MMCSD_OCR_VDD_WILDCARD;

                        if(!(object->initHandle->autoAssignMaxSpeed) &&
                                    (   (object->initHandle->uaBusSpeed ==
                                                        MMCSD_SD_MODE_HS) ||
                                        (object->initHandle->uaBusSpeed ==
                                                        MMCSD_SD_MODE_DS)))
                        {
                            trans.arg |= ((uint32_t)0U << 24U);
                        }
                        else
                        {
                            trans.arg |= ((uint32_t)1U << 24U);
                        }

                        status = MMCSD_lld_transferPoll(handle, &trans);
                    }

                    retry--;

                } while (((trans.response[0U] & ((uint32_t)BIT(31U))) == 0U) && (retry != 0));

                if(status == MMCSD_STS_SUCCESS)
                {
                    sdDeviceData->ocr = trans.response[0];

                    if((sdDeviceData->ocr & MMCSD_OCR_HIGH_CAPACITY) != 0U)
                    {
                        sdDeviceData->isHC = true;
                    }
                    /* Check if switch voltage accepted or not */
                    if(((sdDeviceData->ocr >> 24U) & 0x01U) != 0U)
                    {
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
                            currState = MMCSD_SD_INIT_STATE_GET_CID;
                        }
                    }
                    else
                    {
                        currState = MMCSD_SD_INIT_STATE_GET_CID;
                    }
                }
            }
            break;

            case MMCSD_SD_INIT_STATE_VOLTAGE_SWITCH:
            {
                // TODO voltage switch not supported yet.
                uint32_t capaReg = MMCSD_getCAPA(object->initHandle->baseAddr);

                if((capaReg & MMCSD_SUPP_VOLT_1P8) != 0U)
                {
                    MMCSD_lld_initTransaction(&trans);
                    trans.cmd = MMCSD_CMD(11U);
                    trans.flags = MMCSD_CMDRSP_48BITS;
                    status = MMCSD_lld_transferPoll(handle, &trans);

                    if(status == MMCSD_STS_SUCCESS)
                    {
                        MMCSD_setBusVolt(object->initHandle->baseAddr,
                                         MMCSD_BUS_VOLT_1_8V);
                        /* Wait 5ms */
                        ClockP_usleep(5000);
                    }
                }
                currState = MMCSD_SD_INIT_STATE_GET_CID;
            }
            break;

            case MMCSD_SD_INIT_STATE_GET_CID:
            {
                MMCSD_lld_initTransaction(&trans);
                trans.cmd = MMCSD_CMD(2U);
                trans.flags = MMCSD_CMDRSP_136BITS;
                trans.arg = 0U;

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
                while(trans.response[0] == 0U){

                    trans.cmd = MMCSD_CMD(3U);
                    trans.flags = 0U;
                    trans.arg = 0U;
                    status = MMCSD_lld_transferPoll(handle, &trans);
                }

                if(status == MMCSD_STS_SUCCESS)
                {
                    /* RCA is the most significant 16 bits */
                    sdDeviceData->rca = ((trans.response[0] >> 16U) & 0xFFFFU);

                    /* Set the Block size */
                    MMCSD_lld_initTransaction(&trans);
                    trans.cmd = MMCSD_CMD(16U);
                    trans.flags = MMCSD_CMDRSP_NONE;
                    trans.arg = ((uint32_t)1U) << 9U;

                    status = MMCSD_lld_transferPoll(handle, &trans);

                    if(MMCSD_STS_SUCCESS == status)
                    {
                        currState = MMCSD_SD_INIT_STATE_GET_CSD;
                    }
                }
            }
            break;

            case MMCSD_SD_INIT_STATE_GET_CSD:
            {
                /* Send CMD9, to get the (CSD)card specific data */
                MMCSD_lld_initTransaction(&trans);
                trans.cmd = MMCSD_CMD(9U);
                trans.flags = MMCSD_CMDRSP_136BITS;
                trans.arg = sdDeviceData->rca << 16U;

                status = MMCSD_lld_transferPoll(handle,&trans);

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
                /* Select the card */
                MMCSD_lld_initTransaction(&trans);
                trans.cmd = MMCSD_CMD(7U);
                trans.flags = MMCSD_CMDRSP_BUSY;
                trans.arg = sdDeviceData->rca << 16U;

                status = MMCSD_lld_transferPoll(handle, &trans);

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
                trans.cmd = MMCSD_CMD(55U);
                trans.flags = 0U;
                trans.arg = sdDeviceData->rca << 16U;
                status = MMCSD_lld_transferPoll(handle, &trans);

                if(MMCSD_STS_SUCCESS == status)
                {
                    trans.cmd = MMCSD_CMD(51U);
                    trans.flags =   MMCSD_CMDRSP_48BITS |
                                    MMCSD_CMDRSP_READ | MMCSD_CMDRSP_DATA;
                    trans.arg = sdDeviceData->rca << 16U;
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
                trans.cmd = MMCSD_CMD(55U);
                trans.arg = (sdDeviceData->rca << 16U);
                status = MMCSD_lld_transferPoll(handle, &trans);

                if(status == MMCSD_STS_SUCCESS)
                {
                    MMCSD_lld_initTransaction(&trans);
                    trans.cmd = MMCSD_CMD(6U);
                    uint32_t width = 0U;

                    if(object->initHandle->autoAssignMaxSpeed == true)
                    {
                        width = MMCSD_BUS_WIDTH_4BIT;
                    }
                    else
                    {
                        if(object->initHandle->busWidth == MMCSD_BUS_WIDTH_4BIT)
                        {
                            width = MMCSD_BUS_WIDTH_4BIT;
                        }
                        else
                        {
                            width = MMCSD_BUS_WIDTH_1BIT;
                        }
                    }

                    trans.arg = width >> 1U;
                    status = MMCSD_lld_transferPoll(handle, &trans);

                    if(status == MMCSD_STS_SUCCESS)
                    {
                        MMCSD_setBusWidth(object->initHandle->baseAddr,
                                          width);
                    }
                }

                /* Spec Version supports CMD6 */
                currState = MMCSD_SD_INIT_STATE_SELECT_BUS_SPEED_MODE;
            }
            break;

            case MMCSD_SD_INIT_STATE_SELECT_BUS_SPEED_MODE:
            {
                if(status == MMCSD_STS_SUCCESS)
                {
                    if(object->initHandle->autoAssignMaxSpeed == true)
                    {

                        status = MMCSD_setBusFreq(  object->initHandle->baseAddr,
                                                    object->initHandle->inputClkFreq,
                                                    MMCSD_SD_HS_FREQUENCY_HZ);
                    }
                    else
                    {
                        /* SD Clock Configuration, set it to DS (Default Speed) */
                        if(object->initHandle->uaBusSpeed == MMCSD_SD_MODE_HS)
                        {
                            status = MMCSD_setBusFreq(  object->initHandle->baseAddr,
                                                        object->initHandle->inputClkFreq,
                                                        MMCSD_SD_HS_FREQUENCY_HZ);
                        }
                        else
                        {
                            status = MMCSD_setBusFreq(  object->initHandle->baseAddr,
                                                        object->initHandle->inputClkFreq,
                                                        MMCSD_SD_DS_FREQUENCY_HZ);
                        }
                    }

                    currState = MMCSD_SD_INIT_STATE_CURRENT_LIMIT;
                }
            }
            break;

            case MMCSD_SD_INIT_STATE_CURRENT_LIMIT:
            {
                // status = MMCSD_switchCardCurrLimit( handle,
                //                                     MMCSD_CMD6_GRP4_800mA);
                currState = MMCSD_SD_INIT_STATE_TUNING;
            }
            break;

            case MMCSD_SD_INIT_STATE_TUNING:
            {
                if(tuningRequired)
                {
                    /* TODO Perform Tuning */
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
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;
    MMCSD_EmmcDeviceData    *emmcDeviceData = (MMCSD_EmmcDeviceData *)NULL;
    MMCSDLLD_Transaction    trans;
    uint32_t                retry = 0xFFFFU;
    uint32_t                currState = 0;
    bool                    initDone = false;

    MMCSD_sysCfg sysCfg = { MMCSD_CLK_ACT_ICLK_FCLK_OFF,
                            MMCSD_STANDBY_MODE_FORCE,
                            MMCSD_IDLE_MODE_FORCE,
                            FALSE,
                            TRUE};


    emmcDeviceData = (MMCSD_EmmcDeviceData *)object->initHandle->deviceData;

    while((initDone == false) && (status == MMCSD_STS_SUCCESS))
    {
        switch(currState)
        {
            case EMMC_INIT_STATE_CONTROLLER_INIT:
            {
                // Soft Reset
                status = MMCSD_softReset(object->initHandle->baseAddr, MMCSD_WAIT_FOREVER);

                if(status == MMCSD_STS_SUCCESS)
                {
                    // Reset Lines
                    MMCSD_linesResetAll(object->initHandle->baseAddr, MMCSD_WAIT_FOREVER);

                    // Set supported voltage list
                    MMCSD_setSupportedVoltage(object->initHandle->baseAddr, (MMCSD_SUPP_VOLT_1P8 | MMCSD_SUPP_VOLT_3P0));
                    MMCSD_systemConfig(object->initHandle->baseAddr, &sysCfg);

                    // Set bus width to 1-bit
                    MMCSD_setBusWidth(object->initHandle->baseAddr, MMCSD_BUS_WIDTH_1BIT);

                    MMCSD_setBusVolt(object->initHandle->baseAddr,
                        MMCSD_BUS_VOLT_3_0V);

                    // Power on
                    status = MMCSD_busPowerOnCtrl(object->initHandle->baseAddr, MMCSD_PWR_CTRL_ON, MMCSD_WAIT_FOREVER);
                }

                if(status == MMCSD_STS_SUCCESS)
                {
                    // Set initialization frequency (400kHz)
                    status = MMCSD_setBusFreq(object->initHandle->baseAddr, object->initHandle->inputClkFreq, MMCSD_ID_MODE_FREQUENCY_HZ);
                }

                if(status == MMCSD_STS_SUCCESS)
                {
                    // Send init stream
                    status = MMCSD_initStreamSend(object->initHandle->baseAddr);
                }

                if(status == MMCSD_STS_SUCCESS)
                {
                    currState = EMMC_INIT_STATE_CHECK_VOLTAGE;
                }
            }
            break;
            
            case EMMC_INIT_STATE_CHECK_VOLTAGE:
            {

                if(status == MMCSD_STS_SUCCESS)
                {
                    currState = EMMC_INIT_STATE_CARD_IDENTIFICATION;
                }
            }
            break;

            case EMMC_INIT_STATE_CARD_IDENTIFICATION:
            {
                // Send CMD0 (reset)
                MMCSD_lld_initTransaction(&trans);
                trans.cmd = MMCSD_CMD(0U);
                trans.flags = MMCSD_CMDRSP_NONE;
                trans.arg = 0U;
                status = MMCSD_lld_transferPoll(handle, &trans);

                ClockP_usleep(5000);

                // Poll for card ready (CMD1)
                retry = 0xFFFFU;
                do
                {
                    MMCSD_lld_initTransaction(&trans);
                    trans.cmd = MMCSD_CMD(1U);
                    trans.flags = 0U;
                    trans.arg = 0xC0FF8080U;
                    status = MMCSD_lld_transferPoll(handle, &trans);
                    retry--;
                } while (((trans.response[0U] & ((uint32_t)BIT(31U))) == 0U) && (retry != 0) && (status == MMCSD_STS_SUCCESS));

                if((retry == 0U) || (status != MMCSD_STS_SUCCESS))
                {
                    status = MMCSD_STS_ERR;
                }
                else
                {
                    emmcDeviceData->ocr = trans.response[0U];
                    if((emmcDeviceData->ocr & MMCSD_OCR_HIGH_CAPACITY) != 0U)
                    {
                        emmcDeviceData->isHC = true;
                    }
                    currState = EMMC_INIT_STATE_GET_CID;
                }
            }
            break;

            case EMMC_INIT_STATE_GET_CID:
            {
                // Send CMD2 (get CID)
                MMCSD_lld_initTransaction(&trans);
                trans.cmd = MMCSD_CMD(2U);
                trans.flags = MMCSD_CMDRSP_136BITS;
                trans.arg = 0U;
                status = MMCSD_lld_transferPoll(handle, &trans);

                if(status == MMCSD_STS_SUCCESS)
                {
                    status = MMCSD_parseCIDEmmc(emmcDeviceData, trans.response);
                    currState = EMMC_INIT_STATE_GET_RCA;
                }
            }
            break;

            case EMMC_INIT_STATE_GET_RCA:
            {

                MMCSD_lld_initTransaction(&trans);
                emmcDeviceData->rca = 2;
                trans.cmd = MMCSD_CMD(3U);
                trans.flags = 0U;
                trans.arg = emmcDeviceData->rca << 16U;
                status = MMCSD_lld_transferPoll(handle, &trans);
                
                if(status == MMCSD_STS_SUCCESS)
                {
                    currState = EMMC_INIT_STATE_GET_CSD;
                }
            }
            break;

            case EMMC_INIT_STATE_GET_CSD:
            {
                // Send CMD9 (get CSD)
                MMCSD_lld_initTransaction(&trans);
                trans.cmd = MMCSD_CMD(9U);
                trans.flags = MMCSD_CMDRSP_136BITS;
                trans.arg = emmcDeviceData->rca << 16U;

                status = MMCSD_lld_transferPoll(handle,&trans);

                if(status == MMCSD_STS_SUCCESS)
                {
                    status = MMCSD_parseCSDEmmc(emmcDeviceData, trans.response);
                    currState = EMMC_INIT_STATE_SELECT_CARD;
                }
            }
            break;

            case EMMC_INIT_STATE_SELECT_CARD:
            {
                // Send CMD7 (select card)
                MMCSD_lld_initTransaction(&trans);
                trans.cmd = MMCSD_CMD(7U);
                trans.flags = MMCSD_CMDRSP_BUSY;
                trans.arg = emmcDeviceData->rca << 16U;
                status = MMCSD_lld_transferPoll(handle, &trans);

                if(status == MMCSD_STS_SUCCESS)
                {
                    currState = EMMC_INIT_STATE_READ_EXT_CSD;
                }
            }
            break;

            case EMMC_INIT_STATE_READ_EXT_CSD:
            {
                // Send CMD8 (read EXT_CSD)
                MMCSD_lld_initTransaction(&trans);
                trans.cmd = MMCSD_CMD(8U);
                trans.flags = MMCSD_CMDRSP_READ | MMCSD_CMDRSP_DATA;
                trans.arg = emmcDeviceData->rca << 16U;
                trans.blockCount = 1U;
                trans.blockSize = 512U;
                trans.dataBuf = emmcDeviceData->extCsd;
                status = MMCSD_lld_transferPoll(handle, &trans);

                ClockP_usleep(5000*20);

                if(status == MMCSD_STS_SUCCESS)
                {
                    status = MMCSD_parseECSDEmmc(emmcDeviceData, emmcDeviceData->extCsd);
                    currState = EMMC_INIT_STATE_SET_SPEED;
                }
            }
            break;

            case EMMC_INIT_STATE_SET_SPEED:
            {
                // Set high speed mode if supported
                MMCSD_lld_initTransaction(&trans);
                trans.cmd = MMCSD_CMD(6U);
                trans.arg = 0x03B90100;
                trans.flags = MMCSD_CMDRSP_BUSY;
                status = MMCSD_lld_transferPoll(handle, &trans);

                if(status == MMCSD_STS_SUCCESS)
                {
                    if(object->initHandle->autoAssignMaxSpeed == true)
                    {

                        status = MMCSD_setBusFreq(  object->initHandle->baseAddr,
                                                    object->initHandle->inputClkFreq,
                                                    MMCSD_EMMC_HS_FREQUENCY_HZ);
                    }
                    else
                    {
                        if(emmcDeviceData->transferSpeed == MMCSD_TRANSPEED_50MBPS)
                        {
                            status = MMCSD_setBusFreq(  object->initHandle->baseAddr,
                                                        object->initHandle->inputClkFreq,
                                                        MMCSD_EMMC_HS_FREQUENCY_HZ);
                        }
                        else
                        {
                            status = MMCSD_setBusFreq(  object->initHandle->baseAddr,
                                                        object->initHandle->inputClkFreq,
                                                        MMCSD_EMMC_DS_FREQUENCY_HZ);
                        }
                    }
                }
                ClockP_usleep(5000*20);
                
                currState = EMMC_INIT_STATE_SET_BUS_WIDTH;
            }
            break;

            case EMMC_INIT_STATE_SET_BUS_WIDTH:
            {
                // Set bus width (default to 4-bit if supported)
                uint32_t controllerBuswidth = MMCSD_BUS_WIDTH_1BIT;
                uint8_t ecsdBusWidth = MMCSD_ECSD_BUS_WIDTH_1BIT;

                if(object->initHandle->busWidth & MMCSD_BUS_WIDTH_4BIT)
                {
                    controllerBuswidth = MMCSD_BUS_WIDTH_4BIT;
                    ecsdBusWidth = MMCSD_ECSD_BUS_WIDTH_4BIT;
                }

                // Send CMD6 to set bus width in EXT_CSD
                MMCSD_lld_initTransaction(&trans);
                trans.cmd = MMCSD_CMD(6U);
                trans.arg = 0x03000000 | (MMCSD_ECSD_BUS_WIDTH_INDEX << 16) |
                            (((0 << MMCSD_ECSD_BUS_WIDTH_ES_SHIFT) | ecsdBusWidth) << 8);
                trans.flags = MMCSD_CMDRSP_BUSY;
                status = MMCSD_lld_transferPoll(handle, &trans);

                ClockP_usleep(5000*20);

                if(status == MMCSD_STS_SUCCESS)
                {
                    MMCSD_setBusWidth(object->initHandle->baseAddr, controllerBuswidth);
                }

                ClockP_usleep(5000*20);

                if(status == MMCSD_STS_SUCCESS)
                {
                    MMCSD_lld_initTransaction(&trans);
                    trans.cmd = MMCSD_CMD(6U);
                    trans.arg = 0x03A20100;
                    trans.flags = MMCSD_CMDRSP_BUSY;
                    status = MMCSD_lld_transferPoll(handle, &trans);
                    currState = EMMC_INIT_STATE_DONE;
                }

                ClockP_usleep(5000*20);
            }
            break;

            case EMMC_INIT_STATE_DONE:
            default:
                initDone = true;
                break;
        }
    }

    if(status == MMCSD_STS_SUCCESS)
    {
        object->state = MMCSD_STATE_IDLE;
    }
    else
    {
        object->state = MMCSD_STATE_RESET;
    }

    return status;
}

static int32_t MMCSD_lld_transferPoll(MMCSDLLD_Handle handle,
                                      MMCSDLLD_Transaction *trans)
{
    int32_t             status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object     *object = (MMCSDLLD_Object *)handle;
    MMCSD_cmdObj        cmdObj = {{0U, 0U, 0U, 0U}, 0U, 0U, 0U, 0U};

    if(object->state == MMCSD_STATE_IDLE)
    {
        /* Set driver state to BUSY */
        object->state = MMCSD_STATE_BUSY;
        /* Reset interrupt configuration */
        MMCSD_intrConfigReset(handle);

        if((object->cmdErrorStat != 0U) || (object->xferErrorStat != 0U))
        {
            /* Some Error Happened in previous Transaction */
            (void)MMCSD_linesResetAll(object->initHandle->baseAddr,
                                        MMCSD_WAIT_FOREVER);
        }

        /* Clear stored Error interrupt statuses */
        object->cmdErrorStat = (uint32_t)0U;
        object->xferErrorStat = (uint32_t)0U;

        /* Configure the cmd object */
        /* Configure the command type to be executed from the command flags */
        if(trans->flags & MMCSD_CMDRSP_STOP)
        {
            cmdObj.cmd.cmdType = MMCSD_CMD_TYPE_BUS_SUSPEND;
        }
        else if(trans->flags & MMCSD_CMDRSP_FS)
        {
            cmdObj.cmd.cmdType = MMCSD_CMD_TYPE_FUNC_SEL;
        }
        else if(trans->flags & MMCSD_CMDRSP_ABORT)
        {
            cmdObj.cmd.cmdType = MMCSD_CMD_TYPE_IO_ABORT;
        }
        else
        {
            cmdObj.cmd.cmdType = 0U; /*dummy statement for misra warning*/
        }

        /* Configure the response type from the command flags */
        if(trans->flags & MMCSD_CMDRSP_NONE)
        {
            cmdObj.cmd.rspType = MMCSD_RSP_TYPE_NONE;
        }
        else if(trans->flags & MMCSD_CMDRSP_136BITS)
        {
            cmdObj.cmd.rspType = MMCSD_RSP_TYPE_LEN_136;
        }
        else if(trans->flags & MMCSD_CMDRSP_BUSY)
        {
            cmdObj.cmd.rspType = MMCSD_RSP_TYPE_LEN_48_BUSY;
        }
        else
        {
            cmdObj.cmd.rspType = MMCSD_RSP_TYPE_LEN_48;
        }

        /* Configure the transfer type */
        if(trans->flags & MMCSD_CMDRSP_DATA)
        {
            cmdObj.enableData = (uint32_t)TRUE;
        }
        else
        {
            cmdObj.enableData = (uint32_t)FALSE;
        }

        if(0U != cmdObj.enableData)
        {
            /* Command with data treansfer */
            /* Store all the transaction data in driver object */
            object->dataBufIdx = (uint8_t*)trans->dataBuf;
            object->dataBlockCount = trans->blockCount;
            object->dataBlockSize = trans->blockSize;

            if(trans->flags & MMCSD_CMDRSP_READ)
            {
                cmdObj.cmd.xferType = MMCSD_XFER_TYPE_RX;
            }
            else
            {
                cmdObj.cmd.xferType = MMCSD_XFER_TYPE_TX;
            }

            // cmdObj.numBlks = object->dataBlockCount;
            cmdObj.numBlks = (cmdObj.enableData == TRUE) ? object->dataBlockCount : 0U;

            /* Set the remaining block count as it will be used for
                keeping count */
            object->remainingBlockCount = object->dataBlockCount;

            /* Set Block Length */
            MMCSD_setBlkLength(object->initHandle->baseAddr, trans->blockSize);
            /* Set Data Timeout */
            MMCSD_setDataTimeout(object->initHandle->baseAddr, 27U);

            /* Set command ID */
            cmdObj.cmd.cmdId = trans->cmd;
            /* Command Argument */
            cmdObj.cmdArg = trans->arg;
            /* DMA Disable */
            cmdObj.enableDma = 0U;

            MMCSD_clearIntrStat(object->initHandle->baseAddr, MMCSD_INTR_MASK_TRNFCOMP);

            /* Configure the interrupts accordingly */
            if(MMCSD_XFER_TYPE_RX == cmdObj.cmd.xferType)
            {
                /* Configure the transfer for read operation */
                MMCSD_clearIntrStat(object->initHandle->baseAddr, MMCSD_INTR_MASK_BUFRDRDY);
                MMCSD_enableIntrStatus(object->initHandle->baseAddr, (uint32_t)MMCSD_INTR_MASK_BUFRDRDY);
                MMCSD_disableIntrStatus(object->initHandle->baseAddr,
                                        MMCSD_INTR_MASK_BUFWRRDY);
            }
            else
            {
                MMCSD_clearIntrStat(object->initHandle->baseAddr,
                                    MMCSD_INTR_MASK_BUFWRRDY);
                /* Configure the transfer for write operation */
                MMCSD_enableIntrStatus(object->initHandle->baseAddr,
                                       MMCSD_INTR_MASK_BUFWRRDY);
                MMCSD_disableIntrStatus(object->initHandle->baseAddr,
                                        MMCSD_INTR_MASK_BUFRDRDY);
            }

            MMCSD_clearIntrStat(object->initHandle->baseAddr, MMCSD_INTR_MASK_TRNFCOMP);
            MMCSD_setDataTimeout(object->initHandle->baseAddr, 27U);
            /* Enable necessary interrupts */
            MMCSD_enableIntrStatus(object->initHandle->baseAddr,
                            (
                                MMCSD_INTR_MASK_CMDCOMP     |
                                MMCSD_INTR_MASK_CMDTIMEOUT  |
                                MMCSD_INTR_MASK_DATATIMEOUT |
                                MMCSD_INTR_MASK_TRNFCOMP
                            ) );

            /* Send out the command object */
            MMCSD_commandSend(object->initHandle->baseAddr, &cmdObj);

            /* Wait for Command Complete */
            MMCSD_lld_cmdCompleteStatusPoll(handle);

            if(object->cmdErrorStat == 0U)
            {
            }
            else
            {
                /* Some Error Happened */
                status = MMCSD_STS_ERR;
            }

            /* Get and store response in transaction object */
            MMCSD_getResponse(  object->initHandle->baseAddr,
                                trans->response);

            if(status == MMCSD_STS_SUCCESS)
            {
                if (cmdObj.cmd.cmdId == MMCSD_CMD(19U))
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
        }

        else
        {
            /* Command without data transfer */
            cmdObj.cmd.cmdId = trans->cmd;
            cmdObj.cmdArg = trans->arg;
            cmdObj.enableDma = 0;

            /* Send out the command object */
            MMCSD_commandSend(object->initHandle->baseAddr, &cmdObj);

            /* Enable the necessary interrupts */
            MMCSD_enableIntrStatus(object->initHandle->baseAddr,
                            (
                                MMCSD_INTR_MASK_CMDCOMP     |
                                MMCSD_INTR_MASK_CMDTIMEOUT
                            ) );

            /* Wait for Command Complete */
            MMCSD_lld_cmdCompleteStatusPoll(handle);

            if(object->cmdErrorStat == 0U)
            {

            }
            else
            {
                /* Some Error Happened */
                status = MMCSD_STS_ERR;
            }
            /* Get and store response in transaction object */
            MMCSD_getResponse(  object->initHandle->baseAddr,
                                trans->response);
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

static int32_t MMCSD_lld_transferIntr(MMCSDLLD_Handle handle,
                                      MMCSDLLD_Transaction *trans)
{
    int32_t             status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object     *object = (MMCSDLLD_Object *)handle;
    MMCSD_cmdObj        cmdObj = {{0U, 0U, 0U, 0U}, 0U, 0U, 0U, 0U};

    if( (object->state == MMCSD_STATE_IDLE) ||
        (   (object->xferState == MMCSD_XFER_WRITE_STATE) &&
            (trans->cmd == MMCSD_CMD(12U))))
    {
        /* Set driver state to BUSY */
        object->state = MMCSD_STATE_BUSY;
        /* Set current Transaction */
        object->currentTxn = trans;
        /* Reset interrupt configuration */
        MMCSD_intrConfigReset(handle);

        if((object->cmdErrorStat != 0U) || (object->xferErrorStat != 0U))
        {
            /* Some Error Happened in previous Transaction */
            (void)MMCSD_linesResetCmd(object->initHandle->baseAddr,
                                        MMCSD_WAIT_FOREVER);
            (void)MMCSD_linesResetDat(object->initHandle->baseAddr,
                                        MMCSD_WAIT_FOREVER);
        }

        /* Clear stored Error interrupt statuses */
        object->cmdErrorStat = (uint32_t)0U;
        object->xferErrorStat = (uint32_t)0U;

        /* Configure the cmd object */
        /* Configure the command type to be executed from the command flags */
        if(trans->flags & MMCSD_CMDRSP_STOP)
        {
            cmdObj.cmd.cmdType = MMCSD_CMD_TYPE_BUS_SUSPEND;
        }
        else if(trans->flags & MMCSD_CMDRSP_FS)
        {
            cmdObj.cmd.cmdType = MMCSD_CMD_TYPE_FUNC_SEL;
        }
        else if(trans->flags & MMCSD_CMDRSP_ABORT)
        {
            cmdObj.cmd.cmdType = MMCSD_CMD_TYPE_IO_ABORT;
        }
        else
        {
            cmdObj.cmd.cmdType = 0U; /*dummy statement for misra warning*/
        }

        /* Configure the response type from the command flags */
        if(trans->flags & MMCSD_CMDRSP_NONE)
        {
            cmdObj.cmd.rspType = MMCSD_RSP_TYPE_NONE;
        }
        else if(trans->flags & MMCSD_CMDRSP_136BITS)
        {
            cmdObj.cmd.rspType = MMCSD_RSP_TYPE_LEN_136;
        }
        else if(trans->flags & MMCSD_CMDRSP_BUSY)
        {
            cmdObj.cmd.rspType = MMCSD_RSP_TYPE_LEN_48_BUSY;
        }
        else
        {
            cmdObj.cmd.rspType = MMCSD_RSP_TYPE_LEN_48;
        }

        /* Configure the transfer type */
        if(trans->flags & MMCSD_CMDRSP_DATA)
        {
            cmdObj.enableData = (uint32_t)TRUE;
        }
        else
        {
            cmdObj.enableData = (uint32_t)FALSE;
        }

        if(0U != cmdObj.enableData)
        {
            /* Command with data treansfer */
            /* Store all the transaction data in driver object */
            object->dataBufIdx = (uint8_t*)trans->dataBuf;
            object->dataBlockCount = trans->blockCount;
            object->dataBlockSize = trans->blockSize;

            if(trans->flags & MMCSD_CMDRSP_READ)
            {
                cmdObj.cmd.xferType = MMCSD_XFER_TYPE_RX;
            }
            else
            {
                cmdObj.cmd.xferType = MMCSD_XFER_TYPE_TX;
            }

            cmdObj.numBlks = object->dataBlockCount;

            /* Set the remaining block count as it will be used for
                keeping count */
            object->remainingBlockCount = object->dataBlockCount;

            /* Set Block Length */
            MMCSD_setBlkLength(object->initHandle->baseAddr, trans->blockSize);
            /* Set Data Timeout */
            MMCSD_setDataTimeout(object->initHandle->baseAddr, 27U);

            /* Set command ID */
            cmdObj.cmd.cmdId = trans->cmd;
            /* Command Argument */
            cmdObj.cmdArg = trans->arg;
            /* DMA Disable */
            cmdObj.enableDma = 0U;

            /* Configure the interrupts accordingly */
            if(MMCSD_XFER_TYPE_RX == cmdObj.cmd.xferType)
            {
                /* Configure the transfer for read operation */
                MMCSD_enableIntrStatus( object->initHandle->baseAddr,
                                        MMCSD_INTR_MASK_BUFRDRDY);
                MMCSD_disableIntrStatus(object->initHandle->baseAddr,
                                        MMCSD_INTR_MASK_BUFWRRDY);
                /* Enable Signals */
                MMCSD_enableSigIntr(object->initHandle->baseAddr,
                                    MMCSD_INTR_MASK_BUFRDRDY);
            }
            else
            {
                /* Configure the transfer for write operation */
                MMCSD_enableIntrStatus(object->initHandle->baseAddr,
                                       MMCSD_INTR_MASK_BUFWRRDY);
                MMCSD_disableIntrStatus(object->initHandle->baseAddr,
                                        MMCSD_INTR_MASK_BUFRDRDY);
                /* Enable Signals */
                MMCSD_enableSigIntr(object->initHandle->baseAddr,
                                    MMCSD_INTR_MASK_BUFWRRDY);

            }

            /* Enable necessary interrupts */
            MMCSD_enableIntrStatus(object->initHandle->baseAddr,
                            (
                                MMCSD_INTR_MASK_CMDCOMP     |
                                MMCSD_INTR_MASK_CMDTIMEOUT  |
                                MMCSD_INTR_MASK_DATATIMEOUT |
                                MMCSD_INTR_MASK_TRNFCOMP
                            ) );
            /* Enable Signals */
            MMCSD_enableSigIntr(object->initHandle->baseAddr,
                            (
                                MMCSD_INTR_MASK_CMDCOMP     |
                                MMCSD_INTR_MASK_CMDTIMEOUT  |
                                MMCSD_INTR_MASK_DATATIMEOUT |
                                MMCSD_INTR_MASK_TRNFCOMP
                            ) );

            /* Set Transfer state to command */
            object->xferState = MMCSD_XFER_CMD_STATE;
            /* Send out the command object */
            MMCSD_commandSend(object->initHandle->baseAddr, &cmdObj);
        }

        else
        {
            /* Command without data transfer */
            cmdObj.cmd.cmdId = trans->cmd;
            cmdObj.cmdArg = trans->arg;
            cmdObj.enableDma = 0;

            /* Enable the necessary interrupts */

            MMCSD_enableIntrStatus(object->initHandle->baseAddr,
                            (
                                MMCSD_INTR_MASK_CMDCOMP     |
                                MMCSD_INTR_MASK_CMDTIMEOUT
                            ) );

            /* Enable Signals */
            MMCSD_enableSigIntr(object->initHandle->baseAddr,
                            (
                                MMCSD_INTR_MASK_CMDCOMP     |
                                MMCSD_INTR_MASK_CMDTIMEOUT
                            ) );

            /* Set Transfer state to command */
            object->xferState = MMCSD_XFER_CMD_STATE;
            /* Send out the command object */
            MMCSD_commandSend(object->initHandle->baseAddr, &cmdObj);
        }
    }
    else
    {
        status = MMCSD_STS_ERR_BUSY;
    }

    return status;
}

static int32_t MMCSD_lld_transferDma(MMCSDLLD_Handle handle,
                                      MMCSDLLD_Transaction *trans)
{
    int32_t             status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object     *object = (MMCSDLLD_Object *)handle;
    MMCSD_cmdObj        cmdObj = {{0U, 0U, 0U, 0U}, 0U, 0U, 0U, 0U};

    if(object->state == MMCSD_STATE_IDLE)
    {
        /* Set driver state to BUSY */
        object->state = MMCSD_STATE_BUSY;
        /* Set current Transaction */
        object->currentTxn = trans;
        /* Reset interrupt configuration */
        MMCSD_intrConfigReset(handle);

        if((object->cmdErrorStat != 0U) || (object->xferErrorStat != 0U))
        {
            /* Some Error Happened in previous Transaction */
            (void)MMCSD_linesResetCmd(object->initHandle->baseAddr,
                                        MMCSD_WAIT_FOREVER);
            (void)MMCSD_linesResetDat(object->initHandle->baseAddr,
                                        MMCSD_WAIT_FOREVER);
        }

        /* Clear stored Error interrupt statuses */
        object->cmdErrorStat = (uint32_t)0U;
        object->xferErrorStat = (uint32_t)0U;

        /* Configure the cmd object */
        /* Configure the command type to be executed from the command flags */
        if(trans->flags & MMCSD_CMDRSP_STOP)
        {
            cmdObj.cmd.cmdType = MMCSD_CMD_TYPE_BUS_SUSPEND;
        }
        else if(trans->flags & MMCSD_CMDRSP_FS)
        {
            cmdObj.cmd.cmdType = MMCSD_CMD_TYPE_FUNC_SEL;
        }
        else if(trans->flags & MMCSD_CMDRSP_ABORT)
        {
            cmdObj.cmd.cmdType = MMCSD_CMD_TYPE_IO_ABORT;
        }
        else
        {
            cmdObj.cmd.cmdType = 0U; /*dummy statement for misra warning*/
        }

        /* Configure the response type from the command flags */
        if(trans->flags & MMCSD_CMDRSP_NONE)
        {
            cmdObj.cmd.rspType = MMCSD_RSP_TYPE_NONE;
        }
        else if(trans->flags & MMCSD_CMDRSP_136BITS)
        {
            cmdObj.cmd.rspType = MMCSD_RSP_TYPE_LEN_136;
        }
        else if(trans->flags & MMCSD_CMDRSP_BUSY)
        {
            cmdObj.cmd.rspType = MMCSD_RSP_TYPE_LEN_48_BUSY;
        }
        else
        {
            cmdObj.cmd.rspType = MMCSD_RSP_TYPE_LEN_48;
        }

        /* Configure the transfer type */
        if(trans->flags & MMCSD_CMDRSP_DATA)
        {
            cmdObj.enableData = (uint32_t)TRUE;
        }
        else
        {
            cmdObj.enableData = (uint32_t)FALSE;
        }

        if(0U != cmdObj.enableData)
        {
            /* Command with data treansfer */
            /* Store all the transaction data in driver object */
            object->dataBufIdx = (uint8_t*)trans->dataBuf;
            object->dataBlockCount = trans->blockCount;
            object->dataBlockSize = trans->blockSize;

            if(trans->flags & MMCSD_CMDRSP_READ)
            {
                cmdObj.cmd.xferType = MMCSD_XFER_TYPE_RX;
            }
            else
            {
                cmdObj.cmd.xferType = MMCSD_XFER_TYPE_TX;
            }

            cmdObj.numBlks = object->dataBlockCount;

            /* Set the remaining block count as it will be used for
                keeping count */
            object->remainingBlockCount = object->dataBlockCount;

            /* Set Block Length */
            MMCSD_setBlkLength(object->initHandle->baseAddr, trans->blockSize);
            /* Set Data Timeout */
            MMCSD_setDataTimeout(object->initHandle->baseAddr, 27U);

            /* Set command ID */
            cmdObj.cmd.cmdId = trans->cmd;
            /* Command Argument */
            cmdObj.cmdArg = trans->arg;
            /* DMA Enable */
            cmdObj.enableDma = 1U;

            /* Enable necessary interrupts */
            MMCSD_enableIntrStatus(object->initHandle->baseAddr,
                            (
                                MMCSD_INTR_MASK_CMDCOMP     |
                                MMCSD_INTR_MASK_CMDTIMEOUT  
                            ) );

            MMCSD_edmaTransfer(handle,&object->mmcsdTxn);

            /* Send out the command object */
            MMCSD_commandSend(object->initHandle->baseAddr, &cmdObj);
            /* Wait for command completion */
        }

        else
        {
            /* Command without data transfer */
            cmdObj.cmd.cmdId = trans->cmd;
            cmdObj.cmdArg = trans->arg;
            cmdObj.enableDma = 0;

            /* Enable the necessary interrupts */

            MMCSD_enableIntrStatus(object->initHandle->baseAddr,
                            (
                                MMCSD_INTR_MASK_CMDCOMP     |
                                MMCSD_INTR_MASK_CMDTIMEOUT
                            ) );

            /* Send out the command object */
            MMCSD_commandSend(object->initHandle->baseAddr, &cmdObj);
        }
    }
    else
    {
        status = MMCSD_STS_ERR_BUSY;
    }

    return status;
}

static void MMCSD_intrConfigReset(MMCSDLLD_Handle handle)
{
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;

    /* Disable Interrupt Statuses */
    MMCSD_disableIntrStatus(object->initHandle->baseAddr,
                            MMCSD_ALL_INTS);
    MMCSD_disableSigIntr(   object->initHandle->baseAddr,
                            MMCSD_ALL_INTS);
}

static void MMCSD_lld_cmdCompleteStatusPoll(MMCSDLLD_Handle handle)
{
    MMCSDLLD_Object     *object = (MMCSDLLD_Object*)handle;
    uint32_t            intrStatus = 0U;
    bool                cmdComplete = false;
    bool                cmdError = false;

    while((cmdComplete == false) && (cmdError == false))
    {
        intrStatus = MMCSD_getIntrStat(object->initHandle->baseAddr);

        if((intrStatus & MMCSD_INTR_MASK_CMDCOMP) != 0U)
        {
            MMCSD_clearIntrStat(object->initHandle->baseAddr,
                                MMCSD_INTR_MASK_CMDCOMP);
            /* Set Flag */
            cmdComplete = true;
        }

        if((intrStatus & MMCSD_INTR_MASK_ERR) != 0U)
        {
            if((intrStatus & MMCSD_INTR_MASK_CMDTIMEOUT) != 0U)
            {
                MMCSD_clearIntrStat(object->initHandle->baseAddr,
                                    MMCSD_INTR_MASK_CMDTIMEOUT);
            }
            /* Store Stat reg status in object */
            object->cmdErrorStat = (intrStatus & 0xFFFFFFFFU);
            /* Set Flag */
            cmdError = true;
        }
    }
}

static void MMCSD_lld_xferCompleteStatusPoll(MMCSDLLD_Handle handle)
{
    MMCSDLLD_Object     *object = (MMCSDLLD_Object*)handle;
    uint32_t            intrStatus = 0U;
    bool                xferComplete = false;
    bool                xferError = false;
    uint32_t            offset = 0U;
    uint32_t            dataLength = 0U;
    uint32_t            i = 0U;
    uint32_t            tempWord = 0xFFFFFFFFU;

    while((xferComplete == false) && (xferError == false))
    {
        /* Get the interrupt status */
        intrStatus = MMCSD_getIntrStat(object->initHandle->baseAddr);

        /* Read data from Media condition */
        if((intrStatus & MMCSD_INTR_MASK_BUFRDRDY) != 0U)
        {
            /* Clear the interrupt */
            MMCSD_clearIntrStat(object->initHandle->baseAddr,
                                MMCSD_INTR_MASK_BUFRDRDY);

            if( (object->dataBufIdx != NULL) &&
                (object->remainingBlockCount != 0U))
            {
                dataLength = object->dataBlockSize;
                offset = ((object->dataBlockCount - object->remainingBlockCount)
                         * (object->dataBlockSize));

                for(i = 0; i < dataLength; i += 4U)
                {
                    MMCSD_getData(  object->initHandle->baseAddr,
                                    (uint8_t*)&tempWord, 4U);

                    object->dataBufIdx[offset + i + 0U] = *((uint8_t *)&tempWord);
                    object->dataBufIdx[offset + i + 1U] = *((uint8_t *)&tempWord + 1U);
                    object->dataBufIdx[offset + i + 2U] = *((uint8_t *)&tempWord + 2U);
                    object->dataBufIdx[offset + i + 3U] = *((uint8_t *)&tempWord + 3U);
                }

                object->remainingBlockCount--;
            }
        }

        /* Write data to Media Condition */
        if((intrStatus & MMCSD_INTR_MASK_BUFWRRDY) != 0U)
        {
            /* Clear the interrupt */
            MMCSD_clearIntrStat(object->initHandle->baseAddr,
                                MMCSD_INTR_MASK_BUFWRRDY);

            if( (object->dataBufIdx != NULL) &&
                (object->remainingBlockCount != 0U))
            {
                dataLength = object->dataBlockSize;
                offset = (( (object->dataBlockCount) -
                            (object->remainingBlockCount)) *
                            (object->dataBlockSize));

                for(i = 0; i < dataLength; i += 4U)
                {
                    *((uint8_t *)&tempWord) = object->dataBufIdx[offset + i];
                    *((uint8_t *)&tempWord + 1U) = object->dataBufIdx[offset + i + 1U];
                    *((uint8_t *)&tempWord + 2U) = object->dataBufIdx[offset + i + 2U];
                    *((uint8_t *)&tempWord + 3U) = object->dataBufIdx[offset + i + 3U];

                    MMCSD_setData(  object->initHandle->baseAddr,
                                    (uint8_t*)&tempWord, 4U);
                }

                object->remainingBlockCount--;
            }
        }

        /* Transfer Complete Condition */
        if((intrStatus & MMCSD_INTR_MASK_TRNFCOMP) != 0U)
        {
            /* Clear the interrupt */
            MMCSD_clearIntrStat(object->initHandle->baseAddr,
                                MMCSD_INTR_MASK_TRNFCOMP);
            /* Set Flag */
            xferComplete = true;
        }

        /* Error Check and Storage */
        if((intrStatus & MMCSD_INTR_MASK_ERR) != 0U)
        {
            /* Store interrupt status */
            object->xferErrorStat = (intrStatus & 0xFFFF0000U);
            /* Set Flag */
            xferError = true;

            if((intrStatus & MMCSD_INTR_MASK_DATATIMEOUT) != 0U)
            {
                MMCSD_clearIntrStat(object->initHandle->baseAddr,
                                    MMCSD_INTR_MASK_DATATIMEOUT);
            }
        }
    }
}

static void MMCSD_lld_xferCompleteStatusPollCMD19(MMCSDLLD_Handle handle)
{
    MMCSDLLD_Object     *object = (MMCSDLLD_Object *)handle;
    uint32_t            intrStatus = 0U;
    uint32_t            i = 0U, tempWord = 0x00U;
    uint32_t            dataLength = 0U;
    bool                xferComplete = false;
    bool                xferError = false;

    while((xferComplete == false) && (xferError == false))
    {
        /* Get the interrupt Status */
        intrStatus = MMCSD_getIntrStat(object->initHandle->baseAddr);
        /* Read data received from card */
        if(intrStatus & MMCSD_INTR_MASK_BUFRDRDY)
        {
            MMCSD_clearIntrStat(object->initHandle->baseAddr,
                                MMCSD_INTR_MASK_BUFRDRDY);

            if(object->dataBufIdx != NULL)
            {
                dataLength = ( (object->dataBlockCount)
                                * (object->dataBlockSize));

                for(i = 0; i < dataLength; i += 4U)
                {
                    MMCSD_getData(  object->initHandle->baseAddr,
                                    (uint8_t *)&tempWord, 4U);
                    object->dataBufIdx[i] = *((uint8_t *)&tempWord);
                    object->dataBufIdx[i + 1U] = *((uint8_t *)&tempWord + 1U);
                    object->dataBufIdx[i + 2U] = *((uint8_t *)&tempWord + 2U);
                    object->dataBufIdx[i + 3U] = *((uint8_t *)&tempWord + 3U);
                }

                xferComplete = true;
            }

            /* Transfer is complete in case of CMD19 */
            MMCSD_clearIntrStat(object->initHandle->baseAddr,
                                MMCSD_INTR_MASK_TRNFCOMP);
        }

        /* Error occurred in data transfer */
        if((intrStatus & MMCSD_INTR_MASK_ERR) != 0U)
        {
            /* Store interrupt status */
            object->xferErrorStat = (intrStatus & 0xFFFF0000U);
            /* Set Flag */
            xferError = true;
        }
    }
}

static int32_t MMCSD_switchCardCurrLimit(MMCSDLLD_Handle handle,
                                         uint32_t cmd16GrpFunc)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Object         *object = (MMCSDLLD_Object *)handle;
    MMCSDLLD_Transaction    trans;

    /* Send CMD6 to switch to the requested group */
    trans.cmd = MMCSD_CMD(6U);
    trans.arg = (   (MMCSD_SWITCH_MODE & MMCSD_CMD6_GRP4_SEL) |
                    (cmd16GrpFunc << 12U ));
    trans.flags = (MMCSD_CMDRSP_READ | MMCSD_CMDRSP_DATA);
    trans.blockCount = 1U;
    trans.blockSize = 64U;
    trans.dataBuf = object->initHandle->dataBuf;
    status = MMCSD_lld_transferPoll(object, &trans);

    if(MMCSD_STS_SUCCESS == status)
    {
        /* Checking bits 379:376 of the CMD6 response  to see if the switch happened successfully */
        if( ((object->initHandle->dataBuf[15U] >> 4) == MMCSD_CMD6_GRP4_200mA) &&
            (cmd16GrpFunc==MMCSD_CMD6_GRP4_200mA))
        {
            status = MMCSD_STS_SUCCESS;
        }
        else if(    ((object->initHandle->dataBuf[15U] >> 4) == MMCSD_CMD6_GRP4_400mA) &&
                    (cmd16GrpFunc==MMCSD_CMD6_GRP4_400mA))
        {
            status = MMCSD_STS_SUCCESS;
        }
        else if(    ((object->initHandle->dataBuf[15U] >> 4) == MMCSD_CMD6_GRP4_600mA) &&
                    (cmd16GrpFunc==MMCSD_CMD6_GRP4_600mA))
        {
            status = MMCSD_STS_SUCCESS;
        }
        else if(    ((object->initHandle->dataBuf[15U] >> 4) == MMCSD_CMD6_GRP4_800mA) &&
                    (cmd16GrpFunc==MMCSD_CMD6_GRP4_800mA))
        {
            status = MMCSD_STS_SUCCESS;
        }
        else
        {
            /* Current not approved */
            status = MMCSD_STS_ERR;
        }
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
        MMCSD_lld_initTransaction(&trans);
        trans.cmd = MMCSD_CMD(13U);
        trans.flags = MMCSD_CMDRSP_48BITS;
        trans.arg = (sdDeviceData->rca << 16U);
        status = MMCSD_lld_transferPoll(handle, &trans);
        readyCheckTryCount++;
        mediaCurrentState = (trans.response[0] >> (9U) & (0xFU));
    }

    if(mediaCurrentState != MMCSD_MEDIA_STATE_TRAN)
    {
        status = MMCSD_STS_ERR;
    }

    return status;
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
            (readyCheckTryCount < MMCSD_MEDIA_STATE_THRESHOLD) &&
            (status == MMCSD_STS_SUCCESS) )
    {

        if(mediaCurrentState == MMCSD_MEDIA_STATE_RCV)
        {
            MMCSD_lld_initTransaction(&trans);
            trans.cmd = MMCSD_CMD(12U);
            trans.arg = (0U);
            status = MMCSD_lld_transferPoll(handle, &trans);
            if(status != MMCSD_STS_SUCCESS)
            {
                break;
            }
        }

        MMCSD_lld_initTransaction(&trans);
        trans.cmd = MMCSD_CMD(13U);
        trans.flags = MMCSD_CMDRSP_48BITS;
        trans.arg = (mmcDeviceData->rca << 16U);
        status = MMCSD_lld_transferPoll(handle, &trans);
        if(status == MMCSD_STS_SUCCESS)
        {
            readyCheckTryCount++;
            mediaCurrentState = ((trans.response[0] >> 9U) & 0x0FU);
        }
        
    }

    if(mediaCurrentState != MMCSD_MEDIA_STATE_TRAN)
    {
        status = MMCSD_STS_ERR;
    }

    if(readyCheckTryCount >= MMCSD_MEDIA_STATE_THRESHOLD)
    {
        status = MMCSD_STS_ERR;
    }

    return status;
}

void MMCSD_lld_completeCurrTransfer(MMCSDLLD_Handle handle,
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

static int32_t MMCSD_lld_sendCmd23MMC(MMCSDLLD_Handle handle, uint32_t numBlks)
{
    int32_t                 status = MMCSD_STS_SUCCESS;
    MMCSDLLD_Transaction    trans;

    MMCSD_lld_initTransaction(&trans);
    trans.cmd = MMCSD_CMD(23U);
    trans.arg = numBlks;
    status = MMCSD_lld_transferPoll(handle, &trans);

    return status;
}