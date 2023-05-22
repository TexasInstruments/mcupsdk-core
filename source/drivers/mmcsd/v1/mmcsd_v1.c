/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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
 *  \file mmcsd_v1.c
 *
 *  \brief File containing MMCSD Driver APIs implementation for version V0.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h> /* For memcpy */
#include <stdbool.h>
#include <drivers/mmcsd.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/hw_include/cslr.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/** \brief This is the timeout value for sending CMD13 to the card.
 * After every write, the CMD13 is sent this many times and wait for
 * the card to go to transfer state
 * */

/** \brief Bit mask. */
#define BIT(x) (((uint32_t)1U) << (x))

/**
 * SD Card information structure
 */

/** \brief SD Commands enumeration. */
#define MMCSD_CMD(x)   (x)

/**
 * Command/Response flags for notifying some information to controller
 */

/** \brief To indicate no response. */
#define MMCSD_CMDRSP_NONE           (BIT(0U))

/** \brief Response to indicate stop condition. */
#define MMCSD_CMDRSP_STOP           (BIT(1U))

/** \brief Response to indicate stop condition. */
#define MMCSD_CMDRSP_FS             (BIT(2U))

/** \brief Response to indicate abort condition. */
#define MMCSD_CMDRSP_ABORT          (BIT(3U))

/** \brief Response to indicate busy state. */
#define MMCSD_CMDRSP_BUSY           (BIT(4U))

/** \brief Command to configure for 48bit R1 response */
#define MMCSD_CMDRSP_48BITS         (BIT(9U))

/** \brief Command to configure for 136 bits data width. */
#define MMCSD_CMDRSP_136BITS        (BIT(5U))

/** \brief Command to configure for data or response. */
#define MMCSD_CMDRSP_DATA           (BIT(6U))

/** \brief Command to configure for data read. */
#define MMCSD_CMDRSP_READ           (BIT(7U))

/** \brief Command to configure for data write. */
#define MMCSD_CMDRSP_WRITE          (BIT(8U))

/** \brief Command to configure for Read or Write interrupt */
#define MMCSD_CMDREQ_WR_RD          (BIT(31U))

/** \brief SD voltage enumeration as per VHS field, after the CHECK PATTERN FIELD */
#define MMCSD_VOLT_2P7_3P6          (0x000100U)
#define MMCSD_VOLT_LOW_RANGE        (0x000200U)

/**
 * SD OCR register definitions.
 */

/** \brief High capacity card type. */
#define MMCSD_OCR_HIGH_CAPACITY     (BIT(30U))

#define MMCSD_OCR_S18R  (BIT(24U))
/**
 * Voltage configurations.
 */

/** \brief Configure for 2.7V to 2.8V VDD level. */
#define MMCSD_OCR_VDD_2P7_2P8       (BIT(15U))

/** \brief Configure for 2.8V to 2.9V VDD level. */
#define MMCSD_OCR_VDD_2P8_2P9       (BIT(16U))

/** \brief Configure for 2.9V to 3.0V VDD level. */
#define MMCSD_OCR_VDD_2P9_3P0       (BIT(17U))

/** \brief Configure for 3.0V to 3.1V VDD level. */
#define MMCSD_OCR_VDD_3P0_3P1       (BIT(18U))

/** \brief Configure for 3.1V to 3.2V VDD level. */
#define MMCSD_OCR_VDD_3P1_3P2       (BIT(19U))

/** \brief Configure for 3.2V to 3.3V VDD level. */
#define MMCSD_OCR_VDD_3P2_3P3       (BIT(20U))

/** \brief Configure for 3.3V to 3.4V VDD level. */
#define MMCSD_OCR_VDD_3P3_3P4       (BIT(21U))

/** \brief Configure for 3.4V to 3.5V VDD level. */
#define MMCSD_OCR_VDD_3P4_3P5       (BIT(22U))

/** \brief Configure for 3.5V to 3.6V VDD level. */
#define MMCSD_OCR_VDD_3P5_3P6       (BIT(23U))

/** \brief Wild card to configure for VDD level. */
#define MMCSD_OCR_VDD_WILDCARD      (((uint32_t)0x1FFU) << 15U)

/**
 * SD CSD register definitions.
 */

/** \brief Card bus frequency configuration for 25 Mbps. */
#define MMCSD_TRANSPEED_25MBPS      (0x32U)

/** \brief Card bus frequency configuration for 50 Mbps. */
#define MMCSD_TRANSPEED_50MBPS      (0x5AU)

/** \brief Gives the card version. */
#define MMCSD_CARD_CMMCSD_VERSION(crd) \
    (((crd)->csd[3U] & 0xC0000000U) >> 30U)

/** \brief Extract the size of device for SD version 0. */
#define MMCSD_CSD0_DEV_SIZE(csd3, csd2, csd1, csd0) \
    ((uint64_t)(((csd2) & 0x000003FFU) << 2U) | (((csd1) & 0xC0000000U) >> 30U))

/** \brief TBD for SD version 0. */
#define MMCSD_CSD0_MULT(csd3, csd2, csd1, csd0) \
    (((csd1) & 0x00038000U) >> 15U)

/** \brief Extract the read block length for SD version 0. */
#define MMCSD_CSD0_RDBLKLEN(csd3, csd2, csd1, csd0) \
    (((csd2) & 0x000F0000U) >> 16U)

/** \brief Extract the card transfer speed for SD version 0. */
#define MMCSD_CSD0_TRANSPEED(csd3, csd2, csd1, csd0) \
    (((csd3) & 0x000000FFU) >> 0U)

/** \brief Extracts the size of card for SD version 0. */
#define MMCSD_CARD0_DEV_SIZE(crd) \
    (MMCSD_CSD0_DEV_SIZE((crd)->csd[3U], (crd)->csd[2U], \
    (crd)->csd[1U], (crd)->csd[0U]))

/** \brief TBD for SD version 0. */
#define MMCSD_CARD0_MULT(crd) \
    (MMCSD_CSD0_MULT((crd)->csd[3U], (crd)->csd[2U], \
    (crd)->csd[1U], (crd)->csd[0U]))

/** \brief Gives the card block length for SD version 0. */
#define MMCSD_CARD0_RDBLKLEN(crd) \
    (MMCSD_CSD0_RDBLKLEN((crd)->csd[3U], (crd)->csd[2U], \
    (crd)->csd[1U], (crd)->csd[0U]))

/** \brief Gives the card transfer speed for SD version 0. */
#define MMCSD_CARD0_TRANSPEED(crd) \
    (MMCSD_CSD0_TRANSPEED((crd)->csd[3U], (crd)->csd[2U], \
    (crd)->csd[1U], (crd)->csd[0U]))

/** \brief Gives number of blocks on card for SD version 0. */
#define MMCSD_CARD0_NUMBLK(crd) \
    ((MMCSD_CARD0_DEV_SIZE((crd)) + 1U) * \
    (((uint32_t)1U) << (MMCSD_CARD0_MULT((crd)) + 2U)))

/** \brief Gives the size of card for SD version 0. */
#define MMCSD_CARD0_SIZE(crd) ((MMCSD_CARD0_NUMBLK((crd))) * \
    (((uint32_t)1U) << (MMCSD_CARD0_RDBLKLEN(crd))))

/** \brief Extracts the size of card for SD version 1. */
#define MMCSD_CSD1_DEV_SIZE(csd3, csd2, csd1, csd0) \
    ((uint64_t)(((csd2) & 0x0000003FU) << 16U) | (((csd1) & 0xFFFF0000U) >> 16U))

/** \brief Extracts the card block length for SD version 1. */
#define MMCSD_CSD1_RDBLKLEN(csd3, csd2, csd1, csd0) \
    (((csd2) & 0x000F0000U) >> 16U)

/** \brief Extracts the card transfer speed for SD version 1. */
#define MMCSD_CSD1_TRANSPEED(csd3, csd2, csd1, csd0) \
    (((csd3) & 0x000000FFU) >> 0U)

/** \brief Gives the size of card for SD version 1. */
#define MMCSD_CARD1_DEV_SIZE(crd) \
    (MMCSD_CSD1_DEV_SIZE((crd)->csd[3U], (crd)->csd[2U], \
    (crd)->csd[1U], (crd)->csd[0U]))

/** \brief Reads the card block length for SD version 1. */
#define MMCSD_CARD1_RDBLKLEN(crd) \
    (MMCSD_CSD1_RDBLKLEN((crd)->csd[3U], (crd)->csd[2U], \
    (crd)->csd[1U], (crd)->csd[0U]))

/** \brief Reads the card transfer speed for SD version 1. */
#define MMCSD_CARD1_TRANSPEED(crd) \
    (MMCSD_CSD1_TRANSPEED((crd)->csd[3U], (crd)->csd[2U], \
    (crd)->csd[1U], (crd)->csd[0U]))

/** \brief Gives the size of card for SD version 1. */
#define MMCSD_CARD1_SIZE(crd) (((MMCSD_CARD1_DEV_SIZE((crd)) + 1U) * \
    (512U * 1024U)))

/** \brief This is the timeout value for sending CMD13 to the card.
 * After every write, the CMD13 is sent this many times and wait for
 * the card to go to transfer state
 * */
#define MMCSD_CARD_TRANS_STATE_THRESHOLD  (10000U)

/* Card status value (Bits 9-12) as defined in physical layer
 * specification section 4.10.1.
 */
#define MMCSD_CARD_STATE_TRANSFER  (4U)
/**
 * Check RCA/status.
 */

 /** \brief Command relative address. */
#define MMCSD_RCA_ADDR(rca)             (((rca) & 0xFFFF0000U) >> 16U)

/** \brief TBD. */
#define MMCSD_RCA_STAT(rca)             ((rca) & 0xFFFFU)

/** \brief Check pattern that can be used for card response validation. */
#define MMCSD_CHECK_PATTERN             (0xAAU)

/**
 * SD SCR related macros.
 */

/** \brief Card version 0. */
#define MMCSD_VERSION_1P0               (0U)

/** \brief Card version 1. */
#define MMCSD_VERSION_1P1               (1U)

/** \brief Card version 2. */
#define MMCSD_VERSION_2P0               (2U)

/**
 * Helper macros.
 * Note card registers are big endian.
 */

/** \brief Reads card version. */
#define MMCSD_CARD_VERSION(sdcard)      ((sdcard)->scr[0U] & 0xFU)

/** \brief Reads card bus width. */
#define MMCSD_CARD_BUSWIDTH(sdcard) (((sdcard)->scr[0U] & 0xF00U) >> 8U)

/** \brief Check for bus width. Give below values
 *         - MMCSD_BUS_WIDTH_1BIT for 1-bit.
 *         - MMCSD_BUS_WIDTH_4BIT for 4-bit.
 *         - 0xFFU                    for invalid bus width.
 */
#define MMCSD_GET_CARD_BUSWIDTH(sdcard) \
    (((((sdcard).busWidth) & 0x0FU) == 0x01) ? \
    0x1 : (((((sdcard).busWidth) & 0x04U) == 0x04U) ? 0x04U : 0xFFU))

/** \brief Check for bus width. Give below values
 *         - 50U for 50 MHz.
 *         - 25U for 25 Mhz.
 *         - 0U  for invalid bus width.
 */
#define MMCSD_GET_CARD_FRE(sdcard) ((((sdcard).tranSpeed) == 0x5AU) ? 50U : \
    ((((sdcard).tranSpeed) == 0x32U) ? 25U : \
    ((((sdcard).tranSpeed) == 0xBU) ? 100U : \
    ((((sdcard).tranSpeed) == 0x2BU) ? 200U : 0U))

/** \brief Define cache line size for buffer alignment. */
#ifndef SOC_CACHELINE_SIZE
#define SOC_CACHELINE_SIZE                  (128U)
#endif

/** \brief Command argument to configure for switch mode. */
#define MMCSD_SWITCH_MODE               (0x80FFFFFFU)
#define MMCSD_CHECK_MODE                (0x00FFFFFFU)
/** \brief Command argument width to configure for transfer speed. */
#define MMCSD_CMD6_GRP1_SEL             (0xFFFFFFF0U)
#define MMCSD_CMD6_GRP4_SEL             (0xFFFF0FFFU)

/** \brief Command argument to configure for default/SDR12 speed. */
#define MMCSD_CMD6_GRP1_DEFAULT         (0x0U)
/** \brief Command argument to configure for high/SDR25 speed. */
#define MMCSD_CMD6_GRP1_HS              (0x1U)
/** \brief Command argument to configure for SDR50 speed. */
#define MMCSD_CMD6_GRP1_SDR50           (0x2U)
/** \brief Command argument to configure for SDR104 speed. */
#define MMCSD_CMD6_GRP1_SDR104          (0x3U)
/** \brief Command argument to configure for DDR50 speed. */
#define MMCSD_CMD6_GRP1_DDR50           (0x4U)

#define MMCSD_CMD6_GRP4_200mA           (0x0U)
#define MMCSD_CMD6_GRP4_400mA           (0x1U)
#define MMCSD_CMD6_GRP4_600mA           (0x2U)
#define MMCSD_CMD6_GRP4_800mA           (0x3U)

#define MMCSD_ECSD_BUS_WIDTH_INDEX (183U)
#define MMCSD_ECSD_BUS_WIDTH_1BIT       (0U)
#define MMCSD_ECSD_BUS_WIDTH_4BIT       (1U)
#define MMCSD_ECSD_BUS_WIDTH_8BIT       (2U)
#define MMCSD_ECSD_BUS_WIDTH_4BIT_DDR   (5U)
#define MMCSD_ECSD_BUS_WIDTH_8BIT_DDR   (6U)

#define MMCSD_ECSD_BUS_WIDTH_BUSWIDTH_MASK    (0x0FU)
#define MMCSD_ECSD_BUS_WIDTH_BUSWIDTH_SHIFT   (0U)

#define MMCSD_ECSD_BUS_WIDTH_ES_ENABLE    (0x80U)

#define MMCSD_ECSD_BUS_WIDTH_ES_MASK    (0x80U)
#define MMCSD_ECSD_BUS_WIDTH_ES_SHIFT   (7U)

#define TUNING_MAX_PHASE_DELAY (0x7CU)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
typedef struct
{
    void *openLock;
    /**<  Lock to protect MMCSD open*/
    SemaphoreP_Object lockObj;
    /**< Lock object */
} MMCSD_DrvObj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static int32_t MMCSD_initSD(MMCSD_Handle handle);
static int32_t MMCSD_initEMMC(MMCSD_Handle handle);
static int32_t MMCSD_transfer(MMCSD_Handle, MMCSD_Transaction *trans);
static int32_t MMCSD_isReadyForTransfer(MMCSD_Handle handle);
static void MMCSD_cmdStatusFxn(MMCSD_Handle handle);
static void MMCSD_xferStatusFxn(MMCSD_Handle handle);
static void MMCSD_xferStatusFxn_CMD19(MMCSD_Handle handle);
static uint32_t MMCSD_isCmdComplete(uint32_t baseAddr, uint32_t retry);
static uint32_t MMCSD_isXferComplete(uint32_t baseAddr, uint32_t retry);
static int32_t MMCSD_switchCardCurrLimit(MMCSD_Handle handle, uint32_t cmd16GrpFunc);
static int32_t MMCSD_switchCardSpeed(MMCSD_Handle handle, uint32_t cmd16GrpFunc);
static void MMCSD_delay(uint32_t delayValue);
static int32_t MMCSD_tuningProcedure(MMCSD_Handle handle);
static int32_t MMCSD_sendTuning(MMCSD_Handle handle);
static void MMCSD_isr(void *arg);
static int32_t MMCSD_initStreamSend(uint32_t baseAddr);
static void MMCSD_commandSend(uint32_t baseAddr, const MMCSD_cmdObj *pObj);
static void MMCSD_setBlkLength(uint32_t baseAddr, uint32_t blkLen);
static uint32_t MMCSD_getBlkLength(uint32_t baseAddr);
static void MMCSD_getResponse(uint32_t baseAddr, uint32_t *pRsp);
static void MMCSD_getData(uint32_t baseAddr, uint8_t *pData, uint32_t len);
static uint32_t MMCSD_isCardWriteProtected(uint32_t baseAddr);
static uint32_t MMCSD_isUHSDDR50Supported(uint32_t baseAddr);

static void MMCSD_setDLL(uint32_t baseAddr, uint32_t count);
static void MMCSD_disableTuning(uint32_t baseAddr);
static void MMCSD_setDLLSWT(uint32_t baseAddr,uint32_t val);
static uint32_t MMCSD_getDLLSWT(uint32_t baseAddr);
static void MMCSD_setAC12SCLKSEL(uint32_t baseAddr,uint32_t val);
static uint32_t MMCSD_getAC12SCLKSEL(uint32_t baseAddr);
static void MMCSD_setCLKEXTFree(uint32_t baseAddr, uint32_t val);
static void MMCSD_setPAD(uint32_t baseAddr, uint32_t val);
static void MMCSD_v18SigSet(uint32_t baseAddr, uint32_t en);
static uint32_t MMCSD_setCmdSignalLevel(uint32_t baseAddr);
static void MMCSD_setAC12ExecuteTuning(uint32_t baseAddr,uint32_t val);
static uint32_t MMCSD_getAC12ExecuteTuning(uint32_t baseAddr);
static void MMCSD_setAC12UHSMode(uint32_t baseAddr,uint32_t val);
static uint32_t MMCSD_getBusVolt(uint32_t baseAddr);
static uint32_t MMCSD_getBlkCount(uint32_t baseAddr);

/* CSL like functions */
static int32_t MMCSD_softReset(uint32_t baseAddr);
static void MMCSD_linesReset(uint32_t baseAddr, uint32_t resetMask);
static void MMCSD_systemConfig(uint32_t baseAddr, const MMCSD_sysCfg *pCfg);
static void MMCSD_setBusWidth(uint32_t baseAddr, uint32_t width);
static void MMCSD_setBusVolt(uint32_t baseAddr, uint32_t voltage);
static int32_t MMCSD_busPowerOnCtrl(uint32_t baseAddr, uint32_t pwrCtrl);
static int32_t MMCSD_intClockEnable(uint32_t baseAddr, uint32_t enableIntClk);
static uint32_t MMCSD_isIntClockStable(uint32_t baseAddr, uint32_t retry);
static void MMCSD_setSupportedVoltage(uint32_t baseAddr, uint32_t voltMask);
static uint32_t MMCSD_isHighSpeedSupported(uint32_t baseAddr);
static uint32_t MMCSD_isUHSSDR50Supported(uint32_t baseAddr);
static uint32_t MMCSD_isUHSSDR104Supported(uint32_t baseAddr);
static void MMCSD_setDataTimeout(uint32_t baseAddr, uint32_t timeout);
static int32_t MMCSD_setBusFreq(uint32_t baseAddr, uint32_t inputFreq, uint32_t outputFreq, uint32_t bypass);
static int32_t MMCSD_isCardInserted(uint32_t baseAddr);

/* Interrupt related functions */
static void MMCSD_intrStatusEnable(uint32_t baseAddr, uint32_t intrMask);
static void MMCSD_intrStatusDisable(uint32_t baseAddr, uint32_t intrMask);
static void MMCSD_intrEnable(uint32_t baseAddr, uint32_t intrMask);
static void MMCSD_intrDisable(uint32_t baseAddr, uint32_t intrMask);
static uint32_t MMCSD_intrGet(uint32_t baseAddr);
static uint32_t MMCSD_intrStatus(uint32_t baseAddr);
static void MMCSD_intrClear(uint32_t baseAddr, uint32_t intrMask);


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
/** \brief Driver object */
static MMCSD_DrvObj gMmcsdDrvObj =
{
    .openLock      = NULL,
};

/* Tuning pattern for SDR104 mode */
static const uint8_t gTuningPattern4Bit[] = {
	0xff, 0x0f, 0xff, 0x00, 0xff, 0xcc, 0xc3, 0xcc,
	0xc3, 0x3c, 0xcc, 0xff, 0xfe, 0xff, 0xfe, 0xef,
	0xff, 0xdf, 0xff, 0xdd, 0xff, 0xfb, 0xff, 0xfb,
	0xbf, 0xff, 0x7f, 0xff, 0x77, 0xf7, 0xbd, 0xef,
	0xff, 0xf0, 0xff, 0xf0, 0x0f, 0xfc, 0xcc, 0x3c,
	0xcc, 0x33, 0xcc, 0xcf, 0xff, 0xef, 0xff, 0xee,
	0xff, 0xfd, 0xff, 0xfd, 0xdf, 0xff, 0xbf, 0xff,
	0xbb, 0xff, 0xf7, 0xff, 0xf7, 0x7f, 0x7b, 0xde,
};

static uint8_t gDataBuffer[64U];
static uint8_t gCmd6ResponseBuf[64U];

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
void MMCSD_init(void)
{
    int32_t status;
    uint32_t count;
    MMCSD_Object *obj;

    /* Init each driver instance object */
    for(count = 0U; count < gMmcsdConfigNum; count++)
    {
        /* Init object variables */
        obj = gMmcsdConfig[count].object;
        DebugP_assert(NULL != obj);
        memset(obj, 0, sizeof(MMCSD_Object));
    }

    /* Create the driver lock */
    status = SemaphoreP_constructMutex(&gMmcsdDrvObj.lockObj);
    if(SystemP_SUCCESS == status)
    {
        gMmcsdDrvObj.openLock = &gMmcsdDrvObj.lockObj;
    }

    return;
}

void MMCSD_deinit(void)
{
    /* Delete driver lock */
    if(NULL != gMmcsdDrvObj.openLock)
    {
        SemaphoreP_destruct(&gMmcsdDrvObj.lockObj);
        gMmcsdDrvObj.openLock = NULL;
    }
    return;
}

void MMCSD_Params_init(MMCSD_Params *mmcsdParams)
{
    if(mmcsdParams != NULL)
    {
        /* NULL init deviceData */
        mmcsdParams->deviceData = NULL;
    }
}

MMCSD_Handle MMCSD_open(uint32_t index, const MMCSD_Params *openParams)
{
    int32_t status = SystemP_SUCCESS;
    MMCSD_Handle handle = NULL;
    MMCSD_Config *config = NULL;
    MMCSD_Object *obj = NULL;
    HwiP_Params hwiPrms;
    const MMCSD_Attrs *attrs;

    /* Check for valid index */
    if(index >= gMmcsdConfigNum)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        config = &gMmcsdConfig[index];
    }

    /* Protect this region from a concurrent OSPI_Open */
    DebugP_assert(NULL != gMmcsdDrvObj.openLock);
    SemaphoreP_pend(&gMmcsdDrvObj.lockObj, SystemP_WAIT_FOREVER);

    if(SystemP_SUCCESS == status)
    {
        obj = config->object;
        DebugP_assert(NULL != obj);
        DebugP_assert(NULL != config->attrs);
        attrs = config->attrs;
        if(TRUE == obj->isOpen)
        {
            /* Handle already opened */
            status = SystemP_FAILURE;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        obj->handle = (MMCSD_Handle)config;

        /* Register interrupt */
        if(TRUE == attrs->intrEnable)
        {
            HwiP_Params_init(&hwiPrms);
            hwiPrms.intNum      = attrs->intrNum;
            hwiPrms.callback    = &MMCSD_isr;
            hwiPrms.args        = (void *)config;
            status += HwiP_construct(&obj->hwiObj, &hwiPrms);
        }

        /* Create semaphores for transfer completion */
        status += SemaphoreP_constructMutex(&obj->cmdMutex);
        status += SemaphoreP_constructMutex(&obj->xferMutex);
        status += SemaphoreP_constructBinary(&obj->cmdCompleteSemObj, 0);
        status += SemaphoreP_constructBinary(&obj->dataCopyCompleteSemObj, 0);
        status += SemaphoreP_constructBinary(&obj->xferCompleteSemObj, 0);

        /* Program MMCSD instance according the user config */
        obj->cardType = attrs->cardType;
        obj->dmaEnable = attrs->dmaEnable;
        obj->intrEnable = attrs->intrEnable;

        if(MMCSD_CARD_SD  == obj->cardType)
        {
            status = MMCSD_initSD(config);
        }
        else if(MMCSD_CARD_EMMC == obj->cardType)
        {
            status = MMCSD_initEMMC(config);
        }
        else if(MMCSD_CARD_TYPE_NO_DEVICE == obj->cardType)
        {
            /* Nothing to be initialized */
            status = SystemP_SUCCESS;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        obj->isOpen = 1;
        handle = (MMCSD_Handle) config;
    }

    SemaphoreP_post(&gMmcsdDrvObj.lockObj);

    /* Free up resources in case of error */
    if(SystemP_SUCCESS != status)
    {
        if(NULL != config)
        {
            MMCSD_close((MMCSD_Handle) config);
        }
    }
    return handle;
}

MMCSD_Handle MMCSD_getHandle(uint32_t driverInstanceIndex)
{
    MMCSD_Handle         handle = NULL;
    /* Check index */
    if(driverInstanceIndex < gMmcsdConfigNum)
    {
        MMCSD_Object *obj;
        obj = gMmcsdConfig[driverInstanceIndex].object;

        if(obj && (TRUE == obj->isOpen))
        {
            /* valid handle */
            handle = obj->handle;
        }
    }
    return handle;
}

void MMCSD_close(MMCSD_Handle handle)
{
    if(handle != NULL)
    {
        MMCSD_Object *obj = ((MMCSD_Config *)handle)->object;

        if(obj->intrEnable == CSL_TRUE)
        {
            /* Unregister the interrupt */
            HwiP_destruct(&obj->hwiObj);
        }

        SemaphoreP_destruct(&obj->cmdMutex);
        SemaphoreP_destruct(&obj->xferMutex);
        SemaphoreP_destruct(&obj->cmdCompleteSemObj);
        SemaphoreP_destruct(&obj->dataCopyCompleteSemObj);
        SemaphoreP_destruct(&obj->xferCompleteSemObj);

        memset(obj, 0, sizeof(MMCSD_Object));
    }
}

int32_t MMCSD_read(MMCSD_Handle handle, uint8_t *buf, uint32_t startBlk, uint32_t numBlks)
{
    int32_t status = SystemP_SUCCESS;
    MMCSD_Object *obj = NULL;
    MMCSD_Transaction trans;
    uint32_t addr = 0U;

    if(handle != NULL)
    {
        obj = ((MMCSD_Config *)handle)->object;

        if(obj != NULL && buf != NULL)
        {
            obj->readBufIdx = buf;
            obj->readBlockCount = numBlks;
            if(SystemP_SUCCESS == status)
            {
                status = MMCSD_isReadyForTransfer(handle);
            }

            if(SystemP_SUCCESS == status)
            {
                if(obj->highCap != 0)
                {
                    addr = startBlk;
                }
                else
                {
                    addr = startBlk * (obj->blockSize);
                }

                trans.flags = MMCSD_CMDRSP_READ | MMCSD_CMDRSP_DATA | MMCSD_CMDREQ_WR_RD;
                trans.arg = addr;
                trans.blockCount = numBlks;
                trans.blockSize = obj->blockSize;
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

                status = MMCSD_transfer(handle, &trans);
            }

            if((SystemP_SUCCESS == status))
            {
                if(trans.blockCount > 1U)
                {
                    memset(&trans,0,sizeof(trans));
                    trans.cmd = MMCSD_CMD(12U);
                    trans.flags = MMCSD_CMDRSP_BUSY;
                    trans.arg = 0U;
                    status = MMCSD_transfer(handle, &trans);
                }
            }
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t MMCSD_write(MMCSD_Handle handle, uint8_t *buf, uint32_t startBlk, uint32_t numBlks)
{
    int32_t status = SystemP_SUCCESS;
    MMCSD_Object *obj = NULL;
    MMCSD_Transaction trans;
    uint32_t addr = 0U;

    if(handle != NULL)
    {
        obj = ((MMCSD_Config *)handle)->object;

        if((obj != NULL) && (buf != NULL))
        {
            obj->writeBlockCount = numBlks;

            if(obj->isOpen == 0)
            {
                status = SystemP_FAILURE;
            }

            if(SystemP_SUCCESS == status)
            {
                status = MMCSD_isReadyForTransfer(handle);
            }

            if(SystemP_SUCCESS == status)
            {
                /*
                * Address is in blks for high cap cards and in actual bytes
                * for standard capacity cards
                */
                if(obj->highCap != 0)
                {
                    addr = startBlk;
                }
                else
                {
                    addr = startBlk * obj->blockSize;
                }

                trans.flags = MMCSD_CMDRSP_WRITE | MMCSD_CMDRSP_DATA | MMCSD_CMDREQ_WR_RD;
                trans.arg = addr;
                trans.blockCount = numBlks;
                trans.blockSize = obj->blockSize;
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

                status = MMCSD_transfer(handle, &trans);
            }

            if(SystemP_SUCCESS == status)
            {
                /* Send a STOP */
                if(trans.blockCount > 1U)
                {
                    memset(&trans,0,sizeof(trans));
                    trans.cmd = MMCSD_CMD(12U);
                    trans.flags = MMCSD_CMDRSP_BUSY;
                    trans.arg = 0U;

                    status = MMCSD_transfer(handle, &trans);
                }
            }
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }
    return status;
}

/* ========================================================================== */
/*                     Internal function definitions                          */
/* ========================================================================== */

static int32_t MMCSD_initSD(MMCSD_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    MMCSD_Object *obj = NULL;
    MMCSD_Attrs const *attrs;
    bool support18V_host = FALSE;
	bool attempt_18V_switch = TRUE; /* Always be optimistic to start with, unless some cards do not work well with it */

    MMCSD_ioDelayParams iodelayParams = {MMCSD_CARD_SD, MMCSD_TRANSPEED_25MBPS, MMCSD_VOLTAGE_ANY, MMCSD_LOOPBACK_ANY};
    MMCSD_Transaction trans;

    MMCSD_sysCfg sysCfg = {MMCSD_CLK_ACT_ICLK_FCLK_OFF,
                            MMCSD_STANDBY_MODE_FORCE,
                            MMCSD_IDLE_MODE_FORCE,
                            FALSE,
                            TRUE};

    if(handle != NULL)
    {
        /* Get the pointer to the object and attrs */
        obj = ((MMCSD_Config *)handle)->object;
        attrs = ((MMCSD_Config *)handle)->attrs;

        if((obj != NULL) && (attrs != NULL))
        {
            if(attrs->supportedBusVoltages & MMCSD_BUS_VOLTAGE_1_8V)
            {
                support18V_host = TRUE;
            }
            memset(gDataBuffer,0,sizeof(gDataBuffer));

            if(SystemP_SUCCESS == status)
            {
                status = MMCSD_softReset(attrs->baseAddr);
            }

            if(SystemP_SUCCESS == status)
            {
                /* Lines Reset */
                MMCSD_linesReset(attrs->baseAddr, MMCSD_RESET_LINE_MASK_ALL);

                /* Set supported voltage list */
                MMCSD_setSupportedVoltage(attrs->baseAddr,(MMCSD_SUPP_VOLT_1P8 | MMCSD_SUPP_VOLT_3P3));

                MMCSD_systemConfig(attrs->baseAddr, &sysCfg);

                /* Set the bus width */
                MMCSD_setBusWidth(attrs->baseAddr, MMCSD_BUS_WIDTH_1BIT);

                MMCSD_setBusVolt(attrs->baseAddr, MMCSD_BUS_VOLT_3P3);

                /* Set the bus voltage */
		        if(attrs->supportedBusVoltages & MMCSD_BUS_VOLTAGE_3_3V)
                {
                    MMCSD_setBusVolt(attrs->baseAddr, MMCSD_BUS_VOLT_3P3); /* Default */
		        }
                else
                {
                    if(attrs->supportedBusVoltages & MMCSD_BUS_VOLTAGE_1_8V)
                    {
                        MMCSD_setBusVolt(attrs->baseAddr, MMCSD_BUS_VOLT_1P8);
                    }
                }

                /* Wait for card detect */
                while(!MMCSD_isCardInserted(attrs->baseAddr));

                /* Bus power on */
                status = ((int32_t)(MMCSD_busPowerOnCtrl(attrs->baseAddr, MMCSD_PWR_CTRL_ON)));
		        obj->switched_to_v18 = FALSE;
            }

            if(SystemP_SUCCESS == status)
            {
                /* Set the initialization frequency */
                status = MMCSD_setBusFreq(attrs->baseAddr, attrs->inputClk,
                                        attrs->outputClk, FALSE);

                if(attrs->iodelayFxn != NULL)
                {
                    iodelayParams.transferSpeed = MMCSD_TRANSPEED_25MBPS;
                    attrs->iodelayFxn(attrs->instNum, &iodelayParams);
                }
                if(SystemP_SUCCESS == status)
                {
                    status = MMCSD_initStreamSend(attrs->baseAddr);
                }
            }

            /* Card initialization */
            if(SystemP_SUCCESS == status)
            {
                /* Send CMD 0 to reset */
                trans.cmd = MMCSD_CMD(0);
                trans.flags = MMCSD_CMDRSP_NONE;
                trans.arg = 0U;
                status = MMCSD_transfer(handle, &trans);
            }

            if(SystemP_SUCCESS == status)
            {
                /* APP cmd should be preceeded by a CMD55 */
                trans.cmd = MMCSD_CMD(55U);
                trans.flags = 0U;
                trans.arg = obj->rca << 16U;
                status = MMCSD_transfer(handle, &trans);
            }

            if(SystemP_SUCCESS == status)
            {
                /*
                 * Card type can be found by sending CMD55. If the card responds,
                 * it is a SD card. Else, we assume it is a MMC Card.
                 */
                trans.cmd = MMCSD_CMD(55U);
                trans.flags = CSL_FALSE;
                trans.arg = CSL_FALSE;
                status = MMCSD_transfer(handle, &trans);
            }

            /* SD Card */
            if(SystemP_SUCCESS == status)
            {
                obj->cardType = MMCSD_CARD_SD;

                /* CMD0 - reset card */
                trans.cmd = MMCSD_CMD(0U);
                trans.flags = MMCSD_CMDRSP_NONE;
                trans.arg = 0U;
                status = MMCSD_transfer(handle, &trans);

                if(SystemP_SUCCESS == status)
                {
                    uint32_t currBusVoltage;
                    /* CMD8 - Send operating voltage */
                    trans.cmd = MMCSD_CMD(8U);
                    trans.flags = 0U;
                    trans.arg = MMCSD_CHECK_PATTERN;
                    currBusVoltage = MMCSD_getBusVolt(attrs->baseAddr);

                    if(currBusVoltage == MMCSD_BUS_VOLT_1P8)
                    {
                        trans.arg |= MMCSD_VOLT_LOW_RANGE;
                    }
                    else
                    {
                        trans.arg |= MMCSD_VOLT_2P7_3P6;
                    }

                    status = MMCSD_transfer(handle, &trans);
                }

                if(SystemP_FAILURE == status)
                {
                    /*
                    * If the cmd fails, it can be due to version < 2.0, since
                    * we are currently supporting high voltage cards only.
                    */
                }

                if(SystemP_SUCCESS == status)
                {
                    /* APP cmd should be preceded by a CMD55 */
                    trans.cmd = MMCSD_CMD(55U);
                    trans.flags = 0U;
                    trans.arg = 0; /* obj->rca is zero as the card is in idle state */
                    status = MMCSD_transfer(handle, &trans);
                }

                if(status == SystemP_SUCCESS)
                {
                    /* Sending ACMD41 with host capabilities (host capacity support information) */
                    trans.cmd = MMCSD_CMD(41U);
                    trans.flags = 0U;
                    trans.arg = MMCSD_OCR_HIGH_CAPACITY | MMCSD_OCR_VDD_WILDCARD;
                    if(support18V_host)
                    {
                        /* Host supports 1.8V and seek if the card supports it */
                        trans.arg |= MMCSD_OCR_S18R;
                    }
                    status = MMCSD_transfer(handle, &trans);
                }

                if(SystemP_SUCCESS == status)
                {
                    uint32_t retry = 0xFFFFU;
                    /* Poll until we get the card status (BIT31 of OCR) is powered up */
                    do
                    {
                        /* APP cmd should be preceded by a CMD55*/
                        trans.cmd = MMCSD_CMD(55U);
                        trans.flags = 0U;
                        trans.arg = obj->rca << 16U;
                        status  = MMCSD_transfer(handle, &trans);

                        if(SystemP_SUCCESS == status)
                        {
                            trans.cmd = MMCSD_CMD(41U);
                            trans.flags = 0U;
                            trans.arg = MMCSD_OCR_HIGH_CAPACITY | MMCSD_OCR_VDD_WILDCARD;
                            if(support18V_host)
                            {
                                /* Host supports 1.8V and seek if the card supports it */
                                trans.arg |= MMCSD_OCR_S18R;
                            }
                            MMCSD_transfer(handle, &trans);
                        }
                        else
                        {
                            status = SystemP_FAILURE;
                            break;
                        }
                        retry--;
                    } while (((trans.response[0U] & ((uint32_t)BIT(31U))) == 0U) && (retry != 0));

                    if(retry == 0U)
                    {
                        /* No point in continuing */
                        status = SystemP_FAILURE;
                    }
                }

                if(SystemP_SUCCESS == status)
                {
                     obj->ocr = trans.response[0U];

                    /* Card capacity status bit 30 */
                    obj->highCap = (obj->ocr & MMCSD_OCR_HIGH_CAPACITY) ? 1U : 0U;

                    /* Bit32 of the response R3 (S18A)*/
                    obj->support1_8V = (trans.response[0U] & MMCSD_OCR_S18R) ? 1U : 0U;

                    if(MMCSD_getBusVolt(attrs->baseAddr) == MMCSD_BUS_VOLT_1P8)
                    {
                        attempt_18V_switch = FALSE;
                        obj->switched_to_v18 = TRUE; /* Already in 1.8V mode */
                    }

                    /* If 1.8V is supported, configure the card accordingly to switch to SDR modes */
                    if(obj->support1_8V && support18V_host && attempt_18V_switch && (attrs->switchVoltageFxn != NULL))
                    {
                        /* TODO */
                    }
                }
            }

            if(SystemP_SUCCESS == status)
            {
                /* Send CMD2, to get the card identification register */
                trans.cmd = MMCSD_CMD(2U);
                trans.flags = MMCSD_CMDRSP_136BITS;
                trans.arg = 0U;

                status = MMCSD_transfer(handle, &trans);

                memcpy(obj->cid, trans.response, 16U);
            }

            if(SystemP_SUCCESS == status)
            {
                /* Send CMD3, to get the card relative address*/
                trans.cmd = MMCSD_CMD(3U);
                trans.flags = 0U;
                trans.arg = 0U;

                status = MMCSD_transfer(handle, &trans);

                obj->rca = MMCSD_RCA_ADDR(trans.response[0U]);
            }
            uint8_t break_val = 0U;

            if(SystemP_SUCCESS == status)
            {
                do
                {
                    trans.cmd = MMCSD_CMD(16U);
                    trans.flags = MMCSD_CMDRSP_NONE;
                    trans.arg = ((uint32_t)1U) << 10U;

                    status = MMCSD_transfer(handle, &trans);

                    if(SystemP_SUCCESS == status)
                    {
                        obj->blockSize = 1024U;
                        obj->blockCount = obj->size / obj->blockSize;
                        break_val = 1U;
                    }

                    if(break_val == 0U)
                    {
                        /* Send CMD16, to set the block length */
                        trans.cmd = MMCSD_CMD(16U);
                        trans.flags = MMCSD_CMDRSP_NONE;
                        trans.arg = ((uint32_t)1U) << 9U;

                        status = MMCSD_transfer(handle, &trans);

                        if(SystemP_SUCCESS == status)
                        {
                            obj->size = 1024U;
                            obj->blockSize = 512U;
                            obj->blockCount = (obj->size) / (obj->blockSize);
                            break_val = 1U;
                        }
                    }

                    if(break_val == 1U)
                    {
                        break;
                    }

                } while (0);
            }

            if(SystemP_SUCCESS == status)
            {
                /* Send CMD9, to get the card specific data */
                trans.cmd = MMCSD_CMD(9U);
                trans.flags = MMCSD_CMDRSP_136BITS;
                trans.arg = obj->rca << 16U;

                status = MMCSD_transfer(handle,&trans);
                memcpy(obj->csd, trans.response, 16U);
            }

            if(SystemP_SUCCESS == status)
            {
                if(MMCSD_CARD_CMMCSD_VERSION(obj))
                {
                    obj->tranSpeed = MMCSD_CARD1_TRANSPEED(obj);
                    obj->blockSize = ((uint32_t)1U) << (MMCSD_CARD1_RDBLKLEN(obj));
                    obj->size = MMCSD_CARD1_SIZE(obj);
                    obj->blockCount = obj->size / obj->blockSize;
                }
                else
                {
                    obj->tranSpeed = MMCSD_CARD0_TRANSPEED(obj);
                    obj->blockSize = ((uint32_t)1U) << (MMCSD_CARD0_RDBLKLEN(obj));
                    obj->size = MMCSD_CARD0_SIZE(obj);
                    obj->blockCount = MMCSD_CARD0_NUMBLK(obj);
                }

                /* Set data block length to 512 (for byte addressing cards) */
                if((obj->highCap) == 0U)
                {
                    trans.cmd = MMCSD_CMD(16U);
                    trans.flags = MMCSD_CMDRSP_NONE;
                    trans.arg = 512U;
                    status = MMCSD_transfer(handle, &trans);

                    if(SystemP_SUCCESS == status)
                    {
                        obj->blockSize = 512U;
                    }
                }
            }

            if(SystemP_SUCCESS == status)
            {
                /* Select the card*/
                trans.cmd = MMCSD_CMD(7U);
                trans.flags = MMCSD_CMDRSP_BUSY;
                trans.arg = obj->rca << 16U;

                status = MMCSD_transfer(handle, &trans);
            }

            if(SystemP_SUCCESS == status)
            {
                /*
                * Send ACMD51, to get the SD Configuration register details.
                * Note, this needs data transfer (on data lines).
                */
                trans.cmd = MMCSD_CMD(55U);
                trans.flags = 0U;
                trans.arg = obj->rca << 16U;

                status = MMCSD_transfer(handle, &trans);
            }

            if(SystemP_SUCCESS == status)
            {
                trans.cmd = MMCSD_CMD(51U);
                trans.flags = MMCSD_CMDRSP_48BITS | MMCSD_CMDRSP_READ | MMCSD_CMDRSP_DATA;
                trans.arg = obj->rca << 16U;
                trans.blockCount = 1U;
                trans.blockSize = 8U;
                trans.dataBuf = gDataBuffer;

                status = MMCSD_transfer(handle, &trans);
            }

            if(SystemP_SUCCESS == status)
            {
                obj->scr[0U] = (((uint32_t)(gDataBuffer[3U])) << 24U) | (((uint32_t)(gDataBuffer[2U])) << 16U) | \
                               (((uint32_t)(gDataBuffer[1U])) << 8U) | (((uint32_t)(gDataBuffer[0U])));

                obj->scr[1U] = (((uint32_t)(gDataBuffer[7U])) << 24U) | (((uint32_t)(gDataBuffer[6U])) << 16U) | \
                               (((uint32_t)(gDataBuffer[5U])) << 8U) | (((uint32_t)(gDataBuffer[4U])));

                obj->sdVer = MMCSD_CARD_VERSION(obj);
                obj->busWidth = MMCSD_CARD_BUSWIDTH(obj);
            }

            if(SystemP_SUCCESS == status)
            {
                /* APP cmd should be preceeded by a CMD55 */
                trans.cmd = MMCSD_CMD(55U);
                trans.flags = 0U;
                trans.arg = obj->rca << 16U;
                status = MMCSD_transfer(handle, &trans);
            }

            if(SystemP_SUCCESS == status)
            {
                trans.cmd = MMCSD_CMD(6U);
                trans.arg = MMCSD_BUS_WIDTH_1BIT;
                trans.flags = 0U;

                if (((MMCSD_Attrs *)(((MMCSD_Config *) handle)->attrs))->supportedBusWidth & MMCSD_BUS_WIDTH_4BIT)
                {
                    if (obj->busWidth & MMCSD_BUS_WIDTH_4BIT)
                    {
                        trans.arg = MMCSD_BUS_WIDTH_4BIT;
                    }
                }

                trans.arg = trans.arg >> 1U;
                status = MMCSD_transfer(handle, &trans);

                if (SystemP_SUCCESS == status)
                {
                    if (0U == trans.arg)
                    {
                        MMCSD_setBusWidth(attrs->baseAddr, MMCSD_BUS_WIDTH_1BIT);
                    }
                    else
                    {
                        MMCSD_setBusWidth(attrs->baseAddr, MMCSD_BUS_WIDTH_4BIT);
                    }
                }
            }

            /* Check the CMD6 to see what function could be switched to */
            if(SystemP_SUCCESS == status)
            {
                trans.cmd = MMCSD_CMD(6U);
                trans.arg = ((MMCSD_CHECK_MODE & MMCSD_CMD6_GRP1_SEL) |
                            (MMCSD_CMD6_GRP1_DEFAULT));
                trans.flags = MMCSD_CMDRSP_READ | MMCSD_CMDRSP_DATA;
                trans.blockCount = 1U;
                trans.blockSize = 64U;
                trans.dataBuf = gDataBuffer;
                status = MMCSD_transfer(handle, &trans);
            }

            /* Perform the switch and see what function it switches to */
            if(SystemP_SUCCESS == status)
            {
                uint32_t cmd16GrpFunc;
                int32_t i;
                uint32_t cmd6Groups[5] = {MMCSD_CMD6_GRP1_SDR104,MMCSD_CMD6_GRP1_SDR50,MMCSD_CMD6_GRP1_DDR50,MMCSD_CMD6_GRP1_HS,MMCSD_CMD6_GRP1_DEFAULT};

                for(i=0; i<5; i++)
                {
                    /* Go through the capabilites register and match with the card's speed
                    to arrive at an agreeable speed */
                    /* See if 104 is available in response bits 415-400,  i.e bytes[12] and bytes[13] */
                    /* Please refer to the status response to CMD6 in the physical specification */

                    if((cmd6Groups[i] == MMCSD_CMD6_GRP1_SDR104) && obj->switched_to_v18 && MMCSD_isUHSSDR104Supported(attrs->baseAddr) && (gDataBuffer[13] & (1 << MMCSD_CMD6_GRP1_SDR104)))
                    {
                        cmd16GrpFunc = MMCSD_CMD6_GRP1_SDR104;
                    }
                    else if((cmd6Groups[i] == MMCSD_CMD6_GRP1_SDR50) && obj->switched_to_v18 && MMCSD_isUHSDDR50Supported(attrs->baseAddr) && (gDataBuffer[13] & (1<<MMCSD_CMD6_GRP1_SDR50)))
                    {
                        cmd16GrpFunc = MMCSD_CMD6_GRP1_SDR50;
                    }
                    else if((cmd6Groups[i] == MMCSD_CMD6_GRP1_DDR50) && obj->switched_to_v18 && MMCSD_isUHSDDR50Supported(attrs->baseAddr) && (gDataBuffer[13] & (1<<MMCSD_CMD6_GRP1_DDR50)))
                    {
                        cmd16GrpFunc = MMCSD_CMD6_GRP1_DDR50;
                    }
                    else if((cmd6Groups[i] == MMCSD_CMD6_GRP1_HS) && MMCSD_isHighSpeedSupported(attrs->baseAddr) && (gDataBuffer[13] & (1<< MMCSD_CMD6_GRP1_HS)))
                    {
                        cmd16GrpFunc = MMCSD_CMD6_GRP1_HS;
                    }
                    else if((cmd6Groups[i] == MMCSD_CMD6_GRP1_DEFAULT) && (gDataBuffer[13] & (1<< MMCSD_CMD6_GRP1_DEFAULT)))
                    {
                        cmd16GrpFunc = MMCSD_CMD6_GRP1_DEFAULT;
                    }
                    else
                    {
                        continue;
                    }

                    status = MMCSD_switchCardCurrLimit(handle, MMCSD_CMD6_GRP4_800mA);
                    if(SystemP_FAILURE == status)
                    {
                        //To Do
                    }
                    status = MMCSD_switchCardSpeed(handle, cmd16GrpFunc);
                    if(SystemP_SUCCESS == status)
                    {
                        break; /* Successful switching*/
                    }
                }
            }

            if(SystemP_FAILURE == status)
            {
                MMCSD_close(handle);
            }

        }
        else
        {
            status = SystemP_FAILURE;
        }

    }
    return status;
}

static int32_t MMCSD_initEMMC(MMCSD_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    MMCSD_Object *obj = NULL;
    MMCSD_Attrs const *attrs;

    MMCSD_ioDelayParams iodelayParams = {MMCSD_CARD_EMMC, MMCSD_TRANSPEED_25MBPS, MMCSD_VOLTAGE_ANY, MMCSD_LOOPBACK_ANY};
    MMCSD_Transaction trans;
    MMCSD_sysCfg sysCfg = {MMCSD_CLK_ACT_ICLK_FCLK_OFF,
                            MMCSD_STANDBY_MODE_FORCE,
                            MMCSD_IDLE_MODE_FORCE,
                            FALSE,
                            TRUE};

    /* Default */
    uint32_t controllerBuswidth = MMCSD_BUS_WIDTH_1BIT;
    /* Default */
    uint8_t ecsdBusWidth = MMCSD_ECSD_BUS_WIDTH_1BIT;

    /* Input parameter validation */
    if(handle != NULL)
    {
        /* Get the pointer to the object and hardware attrs */
        obj = ((MMCSD_Config *)handle)->object;
        attrs = ((MMCSD_Config *)handle)->attrs;

        if((obj != NULL) && (attrs != NULL))
        {
            if(SystemP_SUCCESS == status)
            {
                /* Refer to the MMC Host and Bus Configuration steps in TRM controller reset */
                status = MMCSD_softReset(attrs->baseAddr);
            }

            if(SystemP_SUCCESS == status)
            {
                /* Lines Reset */
                MMCSD_linesReset(attrs->baseAddr, MMCSD_RESET_LINE_MASK_ALL);

                /* Set supported voltage list */
                MMCSD_setSupportedVoltage(attrs->baseAddr,(MMCSD_SUPP_VOLT_1P8 | MMCSD_SUPP_VOLT_3P0));

                MMCSD_systemConfig(attrs->baseAddr, &sysCfg);

                /* Set the bus width */
                MMCSD_setBusWidth(attrs->baseAddr, MMCSD_BUS_WIDTH_1BIT);

                /* Set the bus voltage */
		        if(attrs->supportedBusVoltages & MMCSD_BUS_VOLTAGE_3_0V)
                {
                    MMCSD_setBusVolt(attrs->baseAddr, MMCSD_BUS_VOLT_3P0); /* Default */
		        }
                else
                {
                    if(attrs->supportedBusVoltages & MMCSD_BUS_VOLTAGE_1_8V)
                    {
                        MMCSD_setBusVolt(attrs->baseAddr, MMCSD_BUS_VOLT_1P8);
                    }
                }

                /* Bus power on */
                status = ((int32_t)(MMCSD_busPowerOnCtrl(attrs->baseAddr, MMCSD_PWR_CTRL_ON)));

                if(SystemP_SUCCESS == status)
                {
                    /* Set the initialization frequency */
                    status = MMCSD_setBusFreq(attrs->baseAddr, attrs->inputClk,
                                        attrs->outputClk, FALSE);

                    if(attrs->iodelayFxn != NULL)
                    {
                        iodelayParams.transferSpeed = MMCSD_TRANSPEED_25MBPS;
                        attrs->iodelayFxn(attrs->instNum, &iodelayParams);
                    }
                    if(SystemP_SUCCESS == status)
                    {
                        MMCSD_initStreamSend(attrs->baseAddr);
                    }
                }

                /* Card initialization */
                if(SystemP_SUCCESS == status)
                {
                    /* Send CMD 0 to reset */
                    trans.cmd = MMCSD_CMD(0);
                    trans.flags = MMCSD_CMDRSP_NONE;
                    trans.arg = 0U;
                    status = MMCSD_transfer(handle, &trans);
                }

                /*Add delay*/
                MMCSD_delay(5U);

                if(SystemP_SUCCESS == status)
                {
                    uint32_t retry = 0xFFFFU;
                    /* Poll unitl we get the card status (BIT31 of OCR) is powered up */
                    do
                    {
                        /* APP cmd should be preceded by a CMD55*/
                        trans.cmd = MMCSD_CMD(1);
                        trans.flags = 0U;
                        trans.arg = 0xC0FF8080U;
                        status  = MMCSD_transfer(handle, &trans);

                        if(SystemP_FAILURE == status)
                        {
                            break;
                        }
                        retry--;
                    } while (((trans.response[0U] & ((uint32_t)BIT(31U))) == 0U) && (retry != 0));

                    if(retry == 0U)
                    {
                        /* No point in continuing */
                        status = SystemP_FAILURE;
                    }
                }

                if(SystemP_SUCCESS == status)
                {
                    obj->ocr = trans.response[0U];
                    /* Card capacity status bit 30 */
                    obj->highCap = (obj->ocr & MMCSD_OCR_HIGH_CAPACITY) ? 1U : 0U;

                    /* Send CMD2, to get the card identification register */
                    trans.cmd = MMCSD_CMD(2U);
                    trans.flags = MMCSD_CMDRSP_136BITS;
                    trans.arg = 0U;

                    status = MMCSD_transfer(handle, &trans);
                    memcpy(obj->cid, trans.response, 16U);
                }

                if(SystemP_SUCCESS == status)
                {
                    obj->rca = 2U;

                    /* Send CMD3, to get the card relative address*/
                    trans.cmd = MMCSD_CMD(3U);
                    trans.flags = 0U;
                    trans.arg = obj->rca << 16U;

                    status = MMCSD_transfer(handle, &trans);
                }
                if(SystemP_SUCCESS == status)
                {
                    /* Send CMD9, to get the card specific data */
                    trans.cmd = MMCSD_CMD(9U);
                    trans.flags = MMCSD_CMDRSP_136BITS;
                    trans.arg = obj->rca << 16U;

                    status = MMCSD_transfer(handle, &trans);

                    memcpy(obj->csd, trans.response, 16U);
                }

                if(SystemP_SUCCESS == status)
                {
                    obj->tranSpeed = ((obj->csd[3] & 0x000000FFU));
                    obj->blockSize = (((uint32_t)2U)<<(((obj->csd[0] & 0x03C00000U) >> 22)-1U));

                    if(((obj->csd[3] & 0x3C000000U) >> 26) != 0x04U)
                    {
                        status = SystemP_FAILURE;
                    }
                }

                if(SystemP_SUCCESS == status)
                {
                    /*Select the card */
                    trans.cmd = MMCSD_CMD(7U);
                    trans.flags = MMCSD_CMDRSP_BUSY;
                    trans.arg = obj->rca << 16U;

                    status = MMCSD_transfer(handle, &trans);
                }

                if(SystemP_SUCCESS == status)
                {
                    trans.cmd = MMCSD_CMD(8U);
                    trans.flags = MMCSD_CMDRSP_READ | MMCSD_CMDRSP_DATA;
                    trans.arg = obj->rca << 16U;
                    trans.blockCount = 1U;
                    trans.blockSize = 512U;
                    trans.dataBuf = obj->ecsd;

                    status = MMCSD_transfer(handle, &trans);
                }

                /* Add delay */
                MMCSD_delay(100U);

                if(SystemP_SUCCESS == status)
                {
                    obj->blockCount = (((uint32_t)(obj->ecsd[215])) << 24) +
                                         (((uint32_t)(obj->ecsd[214])) << 16) +
                                         (((uint32_t)(obj->ecsd[213])) << 8) +
                                         (((uint32_t)(obj->ecsd[212])));
                    obj->size = (obj->blockCount * obj->blockSize);
                    obj->busWidth = MMCSD_BUS_WIDTH_4BIT;
                    obj->sdVer = obj->ecsd[192];
                }

                if(SystemP_SUCCESS == status)
                {
                    trans.cmd = MMCSD_CMD(6U);
                    trans.arg = 0x03B90100;
                    trans.flags = MMCSD_CMDRSP_BUSY;
                    status = MMCSD_transfer(handle, &trans);
                }

                if(SystemP_SUCCESS == status)
                {
                    if(MMCSD_TRANSPEED_50MBPS == obj->tranSpeed)
                    {
                        if(SystemP_SUCCESS == MMCSD_setBusFreq(attrs->baseAddr, attrs->inputClk, 52000000U, 0U))
                        {
                            if(attrs->iodelayFxn != NULL)
                            {
                                iodelayParams.transferSpeed = MMCSD_TRANSPEED_50MBPS;
                                attrs->iodelayFxn(attrs->instNum, &iodelayParams);
                            }
                            status = SystemP_SUCCESS;
                        }
                    }
                    else
                    {
                        if(SystemP_SUCCESS == MMCSD_setBusFreq(attrs->baseAddr, attrs->inputClk, 26000000U, 0U))
                        {
                            if(attrs->iodelayFxn != NULL)
                            {
                                iodelayParams.transferSpeed = MMCSD_TRANSPEED_25MBPS;
                                attrs->iodelayFxn(attrs->instNum, &iodelayParams);
                            }
                            status = SystemP_SUCCESS;
                        }
                    }
                }

                /* Add delay*/
                MMCSD_delay(100U);

                /* Setting the bus width as per the allowed configuration */
                if(attrs->supportedBusWidth & MMCSD_BUS_WIDTH_4BIT)
                {
                    controllerBuswidth = MMCSD_BUS_WIDTH_4BIT;
                    ecsdBusWidth = MMCSD_ECSD_BUS_WIDTH_4BIT;
                }
                else if(attrs->supportedBusWidth & MMCSD_BUS_WIDTH_1BIT)
                {
                    controllerBuswidth = MMCSD_BUS_WIDTH_1BIT;
                    ecsdBusWidth = MMCSD_ECSD_BUS_WIDTH_1BIT;
                }
                if(SystemP_SUCCESS == status)
                {
                    trans.cmd = MMCSD_CMD(6U);
                    trans.arg = 0x03000000 | (MMCSD_ECSD_BUS_WIDTH_INDEX << 16) | (( (0 << MMCSD_ECSD_BUS_WIDTH_ES_SHIFT) | ecsdBusWidth) << 8);
                    trans.flags = MMCSD_CMDRSP_BUSY;

                    status = MMCSD_transfer(handle, &trans);
                }
                obj->busWidth = controllerBuswidth;

                /* Add delay */
                MMCSD_delay(100U);

                if(SystemP_SUCCESS == status)
                {
                    MMCSD_setBusWidth(attrs->baseAddr, controllerBuswidth);
                }

                MMCSD_delay(100U);

                if(SystemP_SUCCESS == status)
                {
                    trans.cmd = MMCSD_CMD(6U);
                    trans.arg = 0x03A20100;
                    trans.flags = MMCSD_CMDRSP_BUSY;
                    status = MMCSD_transfer(handle, &trans);
                }

                MMCSD_delay(100U);
            }
            if(SystemP_FAILURE == status)
            {
                MMCSD_close(handle);
            }
        }
    }
    return status;
}

/* Function to check if the media is ready to accept read/write transfers */
static int32_t MMCSD_isReadyForTransfer(MMCSD_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t readyCheckTryCount = 0U;
    MMCSD_Object *obj = ((MMCSD_Config *)handle)->object;
    MMCSD_Transaction trans;

    uint32_t mediaCurrentState = 0U;

    if(obj != NULL)
    {
        /*
        * Send CMD13 to check if the card is still in the programming state.
        * The card needs to go to transfer state before it can send/receive data
        */
       /*
        while((mediaCurrentState != MMCSD_CARD_STATE_TRANSFER) && (readyCheckTryCount < MMCSD_CARD_TRANS_STATE_THRESHOLD))
        {
            memset(&trans,0,sizeof(trans));
            trans.cmd = MMCSD_CMD(13U);
            trans.flags = MMCSD_CMDRSP_48BITS;
            trans.arg = obj->rca << 16U;
            status = MMCSD_transfer(handle, &trans);
            readyCheckTryCount++;
            mediaCurrentState = ((trans.response[0] >> 9U) & 0xFU);
        }
        */
        readyCheckTryCount = 0;
        do
        {
            memset(&trans,0,sizeof(trans));
            trans.cmd = MMCSD_CMD(13U);
            trans.flags = MMCSD_CMDRSP_48BITS;
            trans.arg = obj->rca << 16U;
            status = MMCSD_transfer(handle, &trans);
            readyCheckTryCount++;
            mediaCurrentState = (trans.response[0] >> (9U) & (0xFU));

            if(mediaCurrentState == MMCSD_CARD_STATE_TRANSFER){
                status = SystemP_SUCCESS;
                break;
            }

        } while(readyCheckTryCount < MMCSD_CARD_TRANS_STATE_THRESHOLD);
    }
    return status;
}

static int32_t MMCSD_transfer(MMCSD_Handle handle, MMCSD_Transaction *trans)
{
    int32_t status = SystemP_SUCCESS;
    MMCSD_cmdObj cmdObj = {{0U, 0U, 0U, 0U}, 0U, 0U, 0U, 0U};
    MMCSD_Object *obj = NULL;
    const MMCSD_Attrs *attrs = NULL;

    if((handle != NULL) && (trans !=NULL))
    {
        /* Get the pointer to the object and attrs */
        obj = ((MMCSD_Config *)handle)->object;
        attrs = ((MMCSD_Config *)handle)->attrs;

        if((obj != NULL) && (attrs != NULL))
        {
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

            if(attrs->intrEnable == TRUE)
            {
                MMCSD_intrDisable(attrs->baseAddr, MMCSD_INTR_MASK_BUFWRRDY);
                MMCSD_intrDisable(attrs->baseAddr, MMCSD_INTR_MASK_BUFRDRDY);
                MMCSD_intrDisable(attrs->baseAddr, MMCSD_INTR_MASK_TRNFCOMP);
            }

            if(0 != cmdObj.enableData)
            {
                /* Acquire the lock for this particular MMCSD handle */
                SemaphoreP_pend(&(obj->xferMutex), SystemP_WAIT_FOREVER);
                SemaphoreP_pend(&(obj->cmdMutex), SystemP_WAIT_FOREVER);

                obj->dataBufIdx = (uint8_t*)trans->dataBuf;

                obj->dataBlockCount = trans->blockCount;
                obj->dataBlockSize = trans->blockSize;
                obj->cmdComp = 0;
                obj->cmdTimeout = 0;
                obj->xferInProgress = 0;
                obj->xferComp = 0;
                obj->xferTimeout = 0;

                if(trans->flags & MMCSD_CMDRSP_READ)
                {
                    cmdObj.cmd.xferType = MMCSD_XFER_TYPE_RX;
                }
                else
                {
                    cmdObj.cmd.xferType = MMCSD_XFER_TYPE_TX;
                }
                cmdObj.numBlks = (cmdObj.enableData == TRUE) ? obj->dataBlockCount : 0U;

                MMCSD_intrClear(attrs->baseAddr, MMCSD_INTR_MASK_TRNFCOMP);

                if(MMCSD_XFER_TYPE_RX == cmdObj.cmd.xferType)
                {
                    /* Configure the transfer for read operation */
                    MMCSD_intrClear(attrs->baseAddr, MMCSD_INTR_MASK_BUFRDRDY);
                    MMCSD_intrStatusEnable(attrs->baseAddr, MMCSD_INTR_MASK_BUFRDRDY);
                    MMCSD_intrStatusDisable(attrs->baseAddr, MMCSD_INTR_MASK_BUFWRRDY);

                    obj->readBlockCount = obj->dataBlockCount;
                }
                else
                {
                    /* Configure the transfer for write operation */
                    MMCSD_intrClear(attrs->baseAddr, MMCSD_INTR_MASK_BUFWRRDY);
                    MMCSD_intrStatusEnable(attrs->baseAddr, MMCSD_INTR_MASK_BUFWRRDY);
                    MMCSD_intrStatusDisable(attrs->baseAddr, MMCSD_INTR_MASK_BUFRDRDY);

                    if(attrs->intrEnable == TRUE)
                    {
                        MMCSD_intrDisable(attrs->baseAddr, MMCSD_INTR_MASK_BUFWRRDY);
                        MMCSD_intrDisable(attrs->baseAddr, MMCSD_INTR_MASK_BUFRDRDY);
                        MMCSD_intrDisable(attrs->baseAddr, MMCSD_INTR_MASK_TRNFCOMP);
                    }
                    obj->writeBlockCount = obj->dataBlockCount;
                }
                MMCSD_setBlkLength(attrs->baseAddr, trans->blockSize);

                MMCSD_intrClear(attrs->baseAddr, MMCSD_INTR_MASK_TRNFCOMP);
                MMCSD_setDataTimeout(attrs->baseAddr, 27U);

                MMCSD_intrStatusEnable(attrs->baseAddr,
                        (
                            MMCSD_INTR_MASK_CMDCOMP     |
                            MMCSD_INTR_MASK_CMDTIMEOUT  |
                            MMCSD_INTR_MASK_DATATIMEOUT |
                            MMCSD_INTR_MASK_TRNFCOMP
                        )
                    );

                if(attrs->intrEnable == TRUE)
                {
                    MMCSD_intrEnable(attrs->baseAddr,
                        (MMCSD_INTR_MASK_CMDCOMP | MMCSD_INTR_MASK_CMDTIMEOUT |
                        MMCSD_INTR_MASK_DATATIMEOUT));
                }

                cmdObj.cmd.cmdId = trans->cmd;
                cmdObj.cmdArg = trans->arg;
                cmdObj.enableDma = 0U;

                MMCSD_commandSend(attrs->baseAddr, &cmdObj);

                /*
                * Wait for the transfer to complete here.
                * It's OK to block from here because the MMCSD's Hwi will unblock
                * upon errors
                */

                if(attrs->intrEnable == TRUE)
                {
                    SemaphoreP_pend(&obj->cmdCompleteSemObj, SystemP_WAIT_FOREVER);
                }
                else
                {
                    while((obj->cmdComp == FALSE) && (obj->cmdTimeout == FALSE))
                    {
                        MMCSD_cmdStatusFxn(handle);
                    }
                }

                /* Command execution fail */
                if(obj->cmdTimeout == 1)
                {
                    status = SystemP_FAILURE;
                    obj->cmdTimeout = FALSE;
                }

                /* Command execution successful */
                if(obj->cmdComp == TRUE)
                {
                    status = SystemP_SUCCESS;
                    obj->cmdComp = FALSE;

                    if(attrs->intrEnable == FALSE)
                    {
                        obj->xferInProgress = TRUE;
                    }

                    if(attrs->intrEnable == TRUE)
                    {
                        if(MMCSD_XFER_TYPE_RX == cmdObj.cmd.xferType)
                        {
                            MMCSD_intrEnable(attrs->baseAddr, MMCSD_INTR_MASK_BUFRDRDY);
                        }
                        else
                        {
                            MMCSD_intrEnable(attrs->baseAddr, MMCSD_INTR_MASK_BUFWRRDY);
                        }
                        SemaphoreP_pend(&obj->dataCopyCompleteSemObj, SystemP_WAIT_FOREVER);
                        MMCSD_intrEnable(attrs->baseAddr, MMCSD_INTR_MASK_TRNFCOMP);
                    }

                    /* Get command response and update book keeping */
                    MMCSD_getResponse(attrs->baseAddr, trans->response);
                }

                /* Release the lock for this particular MMCSD handle */
                SemaphoreP_post(&obj->cmdMutex);

                if(SystemP_SUCCESS == status)
                {
                    /*
                    * Wait for the transfer to complete here.
                    * It's OK to block from here because the MMCSD's Hwi will unblock
                    * upon errors
                    */
                    if(attrs->intrEnable == TRUE)
                    {
                        SemaphoreP_pend(&obj->xferCompleteSemObj, SystemP_WAIT_FOREVER);
                    }
                    else
                    {
                        if(cmdObj.cmd.cmdId != MMCSD_CMD(19U))
                        {
                            while((obj->xferComp == FALSE) && (obj->xferTimeout == FALSE))
                            {
                                MMCSD_xferStatusFxn(handle);
                            }
                        }
                        else
                        {
                            while((obj->xferComp == FALSE) && (obj->xferTimeout == FALSE))
                            {
                                MMCSD_xferStatusFxn_CMD19(handle);
                            }
                        }
                    }
                    /* Data transfer fail */
                    if (obj->xferTimeout == TRUE)
                    {
                        status = SystemP_FAILURE;
                        obj->xferTimeout = FALSE;
                    }

                    /*Data transfer successful */
                    if(obj->xferComp == TRUE)
                    {
                        status = SystemP_SUCCESS;
                        obj->xferComp = FALSE;
                    }
                }

                /* Release the lock for this particular MMCSD handle */
                SemaphoreP_post(&obj->xferMutex);
            }
            else
            {
                /* Acquire the lock for this particular MMCSD handle */
                SemaphoreP_pend(&obj->cmdMutex, SystemP_WAIT_FOREVER);

                obj->cmdComp = FALSE;
                obj->cmdTimeout = FALSE;

                cmdObj.cmd.cmdId = trans->cmd;
                cmdObj.cmdArg = trans->arg;
                cmdObj.enableDma = 0;
                //obj->cmdError = CSL_FALSE;
                //obj->cmdCRCError = CSL_FALSE;
                MMCSD_commandSend(attrs->baseAddr, &cmdObj);

                MMCSD_intrStatusEnable(attrs->baseAddr,
                (MMCSD_INTR_MASK_CMDCOMP | MMCSD_INTR_MASK_CMDTIMEOUT));

                if(attrs->intrEnable == TRUE)
                {
                    MMCSD_intrEnable(attrs->baseAddr,
                    (MMCSD_INTR_MASK_CMDCOMP | MMCSD_INTR_MASK_CMDTIMEOUT));
                }

                /* Wait for transfer to complete */
                if(attrs->intrEnable == TRUE)
                {
                    SemaphoreP_pend(&obj->cmdCompleteSemObj, SystemP_WAIT_FOREVER);
                }
                else
                {
                    while((obj->cmdComp == FALSE) && (obj->cmdTimeout == FALSE))
                    {
                        MMCSD_cmdStatusFxn(handle);
                    }
                }

                /* Command execution successful */
                if (obj->cmdComp == TRUE)
                {
                    status = SystemP_SUCCESS;
                    obj->cmdComp = FALSE;
                }

                /* Command execution fail */
                if(obj->cmdTimeout == TRUE)
                {
                    status = SystemP_FAILURE;
                    obj->cmdTimeout = FALSE;
                }

                /* Get response for command */
                MMCSD_getResponse(attrs->baseAddr, trans->response);

                /* Release the lock for this particular MMCSD handle */
                SemaphoreP_post(&obj->cmdMutex);
            }
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    /* Return the transaction status */
    return status;
}

static void MMCSD_xferStatusFxn(MMCSD_Handle handle)
{
    MMCSD_Object *obj = NULL;
    const MMCSD_Attrs *attrs = NULL;
    volatile uint32_t dataLength = 0U;
    volatile uint32_t intrStatus = 0U;
    volatile uint32_t tempWord = 0U;
    uint32_t remainingBlocks = 0U, offset = 0U;
    //uint16_t errorIntrStatus = 0U;

    if(handle != NULL)
    {
        obj = ((MMCSD_Config *)handle)->object;
        attrs = ((MMCSD_Config *)handle)->attrs;

        if(obj != NULL && attrs != NULL)
        {
            intrStatus = MMCSD_intrStatus(attrs->baseAddr);

            /* Read data received from card */
            if(intrStatus & MMCSD_INTR_MASK_BUFRDRDY)
            {
                if(obj->xferInProgress == 1)
                {
                    MMCSD_intrClear(attrs->baseAddr,MMCSD_INTR_MASK_BUFRDRDY);

                    if((obj->dataBufIdx != NULL) & (obj->readBlockCount != 0))
                    {
                        dataLength = obj->dataBlockSize;
                        remainingBlocks = obj->readBlockCount;
                        offset = (obj->dataBlockCount - remainingBlocks) * (obj->dataBlockSize);

                        volatile uint32_t i;

                        for(i = 0; i < dataLength; i += 4U)
                        {
                            MMCSD_getData(attrs->baseAddr, (uint8_t *)&tempWord, 4U);
                            //uint8_t *pTempWord = (uint8_t *)&tempWord;
                            obj->dataBufIdx[offset + i] = *((uint8_t *)&tempWord);
                            obj->dataBufIdx[offset + i + 1U] = *((uint8_t *)&tempWord + 1U);
                            obj->dataBufIdx[offset + i + 2U] = *((uint8_t *)&tempWord + 2U);
                            obj->dataBufIdx[offset + i + 3U] = *((uint8_t *)&tempWord + 3U);
                        }

                        obj->readBlockCount--;
                    }
                }
            }

            /* Write data received from card */
            if(intrStatus & MMCSD_INTR_MASK_BUFWRRDY)
            {
                if(obj->xferInProgress == 1)
                {
                    MMCSD_intrClear(attrs->baseAddr, MMCSD_INTR_MASK_BUFWRRDY);

                    if(obj->dataBufIdx != NULL)
                    {
                        remainingBlocks = obj->writeBlockCount;
                        offset = (obj->dataBlockCount - remainingBlocks) * (obj->dataBlockSize);
                        dataLength = obj->dataBlockSize;

                        volatile uint32_t i;

                        for(i=0; i<dataLength; i += 4U)
                        {
                            *((uint8_t *)&tempWord) = obj->dataBufIdx[offset + i];
                            *((uint8_t *)&tempWord + 1U) = obj->dataBufIdx[offset + i + 1U];
                            *((uint8_t *)&tempWord + 2U) = obj->dataBufIdx[offset + i + 2U];
                            *((uint8_t *)&tempWord + 3U) = obj->dataBufIdx[offset + i + 3U];
                            HW_WR_REG32((attrs->baseAddr + CSL_MMC_DATA), tempWord);
                        }

                        obj->writeBlockCount--;
                    }
                }
            }

            /* Error occurred in data transfer */
            if(intrStatus & MMCSD_INTR_MASK_DATATIMEOUT)
            {
                if(obj->xferInProgress == 1)
                {
                    MMCSD_intrClear(attrs->baseAddr, MMCSD_INTR_MASK_DATATIMEOUT);
                    obj->xferTimeout = 1;
                    obj->xferInProgress = 0;
                }
            }

            /* Data transfer is complete */
            if(intrStatus & MMCSD_INTR_MASK_TRNFCOMP)
            {
                if(obj->xferInProgress == 1)
                {
                    MMCSD_intrClear(attrs->baseAddr, MMCSD_INTR_MASK_TRNFCOMP);
                    obj->xferComp = 1;
                    obj->xferInProgress = 0;
                }
            }
        }
    }
    return;
}

/* CMD19 is a bus test pattern command, used for manual tuning*/
static void MMCSD_xferStatusFxn_CMD19(MMCSD_Handle handle)
{
    MMCSD_Object *obj = NULL;
    const MMCSD_Attrs *attrs = NULL;

    if(handle != NULL)
    {
        obj = ((MMCSD_Config *)handle)->object;
        attrs = ((MMCSD_Config *)handle)->attrs;

        if(obj != NULL && attrs != NULL)
        {
            uint32_t intrStatus = MMCSD_intrStatus(attrs->baseAddr);
            /* Read data received from card */
            if(intrStatus & MMCSD_INTR_MASK_BUFRDRDY)
            {
                if(obj->xferInProgress == 1)
                {
                    MMCSD_intrClear(attrs->baseAddr, MMCSD_INTR_MASK_BUFRDRDY);

                    if(obj->dataBufIdx != NULL)
                    {
                        uint32_t dataLength = obj->dataBlockCount * obj->dataBlockSize;
                        uint32_t i, tempWord = 0U;

                        for(i=0; i < dataLength; i += 4U)
                        {
                            MMCSD_getData(attrs->baseAddr, (uint8_t *)&tempWord, 4U);
                            //uint8_t *pTempWord = (uint8_t *)&tempWord;
                            obj->dataBufIdx[i] = *((uint8_t *)&tempWord);
                            obj->dataBufIdx[i + 1U] = *((uint8_t *)&tempWord + 1U);
                            obj->dataBufIdx[i + 2U] = *((uint8_t *)&tempWord + 2U);
                            obj->dataBufIdx[i + 3U] = *((uint8_t *)&tempWord + 3U);
                        }
                    }

                    /* Transfer is complete in case of CMD19 */
                    MMCSD_intrClear(attrs->baseAddr, MMCSD_INTR_MASK_TRNFCOMP);
                    obj->xferComp = 1;
                    obj->xferInProgress = 0;
                }
            }

            /* Error occurred in data transfer */
            if(intrStatus & MMCSD_INTR_MASK_DATATIMEOUT)
            {
                if(obj->xferInProgress == 1)
                {
                    MMCSD_intrClear(attrs->baseAddr, MMCSD_INTR_MASK_DATATIMEOUT);
                    obj->xferTimeout = 1;
                    obj->xferInProgress = 0;
                }
            }
        }
    }
    return;
}

static void MMCSD_cmdStatusFxn(MMCSD_Handle handle)
{
    volatile uint32_t errorIntrStatus;
    volatile uint32_t intrStatus = 0U;
    MMCSD_Object *obj = NULL;
    const MMCSD_Attrs *attrs = NULL;

    if(handle != NULL)
    {
        obj = ((MMCSD_Config *)handle)->object;
        attrs = ((MMCSD_Config *)handle)->attrs;

        if(obj != NULL && attrs != NULL)
        {
            intrStatus = MMCSD_intrStatus(attrs->baseAddr);

            /* Check for command completion */
            if(intrStatus & MMCSD_INTR_MASK_CMDCOMP)
            {
                MMCSD_intrClear(attrs->baseAddr, MMCSD_INTR_MASK_CMDCOMP);
                obj->cmdComp = 1;
            }

            /* Check for errors */
            if(intrStatus & MMCSD_INTR_MASK_ERR)
            {
                errorIntrStatus = intrStatus & 0xFFFF0000U;

                if(errorIntrStatus & MMCSD_INTR_MASK_CMDTIMEOUT)
                {
                    MMCSD_intrClear(attrs->baseAddr,
                                    MMCSD_INTR_MASK_CMDTIMEOUT);
                    obj->cmdTimeout = 1;
                }
            }
        }
    }
    return;
}

static int32_t MMCSD_sendTuning(MMCSD_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t dataBuffer[64U] = { 0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,
                                0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,
                                0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,
                                0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U,0U};
    MMCSD_Transaction trans;
    bool tuningFail = FALSE;
    uint32_t enableInterrupts;
    MMCSD_Attrs *attrs = ((MMCSD_Config *)handle)->attrs;

    enableInterrupts = attrs->intrEnable;

    /* Send CMD19 */
    trans.cmd = MMCSD_CMD(19U);
    trans.arg = 0;
    trans.flags = MMCSD_CMDRSP_48BITS | MMCSD_CMDRSP_READ | MMCSD_CMDRSP_DATA;
    trans.blockCount = 1U;
    trans.blockSize = sizeof(gTuningPattern4Bit); /* 64 Bytes*/
    trans.dataBuf = dataBuffer; /* Tuning data from the SD card comes here */
    status = MMCSD_transfer(handle, &trans);

    if(SystemP_SUCCESS == status)
    {
        /* Compare the data recieved from the card to the expected values */
        uint32_t i;
        for(i=0; i<sizeof(gTuningPattern4Bit); i++)
        {
            if(dataBuffer[i] != gTuningPattern4Bit[i])
            {
                tuningFail = TRUE;
                break;
            }
        }
    }
    else
    {
        tuningFail = TRUE;
    }
    attrs->intrEnable = enableInterrupts;
    return (tuningFail);
}

static int32_t MMCSD_tuningProcedure(MMCSD_Handle handle)
{
    uint32_t phaseDelay = 0,length=0;
    uint32_t maxLen = 0,startWindow = 0,maxWindow = 0,curMatch = 0,prevMatch = 0;

    if(handle != NULL)
    {
        const MMCSD_Attrs *attrs = ((MMCSD_Config *)handle)->attrs;

        if(attrs != NULL)
        {
            MMCSD_setDLLSWT(attrs->baseAddr, 1);
            while(phaseDelay <= TUNING_MAX_PHASE_DELAY)
            {

                /*
                Set MMCHS_DLL[12] FORCE_VAL to 1
                Set MMCHS_DLL[19:13] FORCE_SR_C to 0x0 ( in increments of 4)
                Set MMCHS_DLL[1] DLL_CALIB to 0x1. This transfers the FORCE_SR_C value to the peripheral delay line.
                Set MMCHS_DLL[12] DLL_CALIB to 0
                */
                MMCSD_setDLL(attrs->baseAddr, phaseDelay);

                /* Send CMD19 for SD card */
                curMatch = !MMCSD_sendTuning(handle);
                if(curMatch)
                {
                    if(prevMatch)
                    {
                        length++;
                    }
                    else
                    {
                        startWindow = phaseDelay;
                        length = 1;
                    }
                }

                if(length > maxLen)
                {
                    maxWindow = startWindow;
                    maxLen = length;
                }

                prevMatch = curMatch;
                phaseDelay += 4;
            }

            /*  Check if MMCHS_AC12[23] SCLK_SEL= 0x1 */
            if(MMCSD_getAC12SCLKSEL(attrs->baseAddr) != 1)
            {

                return SystemP_FAILURE; /* Error */

            }

            if(maxLen == 0)
            {
                return SystemP_FAILURE;
            }

            /* Select the centred delay of the largest set of successful try
            and program it in to the MMCHS_DLL[19:13} FORCE_SR_C bit field */
            /* Set the MMCSHS DLL[1] DLL_CALIB to 0x1 so that the DLL takes in to account
            the new FORCE_SR_C value */
            phaseDelay = maxWindow + 4 * (maxLen >> 1);
            MMCSD_setDLL(attrs->baseAddr, phaseDelay);

            /* Reset the DAT and CMD lines by setting to 0x1 both the
            MMCHS_SYSCTRL[26] SRD and MMCHS_SYSCTRL[25] SRC bits in order to
            clear any pending interrupt sources that were masked during the tuning sequence */

            MMCSD_linesReset(attrs->baseAddr,CSL_MMC_SYSCTL_SRC_MASK);
            MMCSD_linesReset(attrs->baseAddr,CSL_MMC_SYSCTL_SRD_MASK);

            /* Ensure that MMCHS_DLL[20] SWT remains 0x1 at exit */
            if(!MMCSD_getDLLSWT(attrs->baseAddr))
            {
            return SystemP_FAILURE; /* Error */
            }
        }
    }
    else
    {
        return SystemP_FAILURE;
    }

    return SystemP_SUCCESS;
}

static int32_t MMCSD_switchCardCurrLimit(MMCSD_Handle handle, uint32_t cmd16GrpFunc)
{
    int32_t status = SystemP_SUCCESS;
    MMCSD_Transaction trans;

    memset(gCmd6ResponseBuf, 0, sizeof(gCmd6ResponseBuf));

    /* Send CMD16 to switch to the requested group */
    trans.cmd = MMCSD_CMD(6U);
    trans.arg = ((MMCSD_SWITCH_MODE & MMCSD_CMD6_GRP4_SEL) | (cmd16GrpFunc << 12 ));
    trans.flags = MMCSD_CMDRSP_READ | MMCSD_CMDRSP_DATA;
    trans.blockCount = 1U;
    trans.blockSize = 64U;
    trans.dataBuf = gCmd6ResponseBuf;
    status = MMCSD_transfer(handle, &trans);

    if(SystemP_SUCCESS == status)
    {

        /* Checking bits 379:376 of the CMD6 response  to see if the switch happened successfully */

        if((gCmd6ResponseBuf[15U] >> 4) == MMCSD_CMD6_GRP4_200mA && (cmd16GrpFunc==MMCSD_CMD6_GRP4_200mA))
        {
            status = SystemP_SUCCESS;
        }
        else if(((gCmd6ResponseBuf[15U] >> 4) == MMCSD_CMD6_GRP4_400mA ) && (cmd16GrpFunc==MMCSD_CMD6_GRP4_400mA))
        {
            status = SystemP_SUCCESS;
        }
        else if(((gCmd6ResponseBuf[15U] >> 4) == MMCSD_CMD6_GRP4_600mA) && (cmd16GrpFunc==MMCSD_CMD6_GRP4_600mA)){
            status = SystemP_SUCCESS;
        }
        else if(((gCmd6ResponseBuf[15U] >> 4) == MMCSD_CMD6_GRP4_800mA) && (cmd16GrpFunc==MMCSD_CMD6_GRP4_800mA)){
            status = SystemP_SUCCESS;
        }
        else{
            /* Current not approved */
            status = SystemP_FAILURE;
        }
    }
    return status;
}

static int32_t MMCSD_switchCardSpeed(MMCSD_Handle handle, uint32_t cmd16GrpFunc)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t uhsMode;
    unsigned short sdr104TuningRequired = FALSE, switchSpeedApproved = TRUE;
    MMCSD_Object *obj = NULL;
    MMCSD_Attrs *attrs = NULL;
    MMCSD_Transaction trans;

    MMCSD_ioDelayParams iodelayParams = {MMCSD_CARD_SD, MMCSD_TRANSPEED_25MBPS, MMCSD_VOLTAGE_ANY, MMCSD_LOOPBACK_ANY};

    if(handle != NULL)
    {
        obj = ((MMCSD_Config *)handle)->object;
        attrs = ((MMCSD_Config *)handle)->attrs;

        if((obj != NULL) && (attrs != NULL))
        {
            memset(gCmd6ResponseBuf, 0, sizeof(gCmd6ResponseBuf));
            /* Send CMD6 to switch to the requested group */
            trans.cmd = MMCSD_CMD(6U);
            trans.arg = ((MMCSD_SWITCH_MODE & MMCSD_CMD6_GRP1_SEL) | cmd16GrpFunc);
            trans.flags = MMCSD_CMDRSP_READ | MMCSD_CMDRSP_DATA;
            trans.blockCount = 1U;
            trans.blockSize = 64U;
            trans.dataBuf = gCmd6ResponseBuf;

            status = MMCSD_transfer(handle, &trans);

            if(SystemP_SUCCESS == status)
            {
                /* Checking bits 379:376 of the CMD6 response  to see if the switch happened successfully */
                uint32_t clkFreq,tranSpeed;

                if((gCmd6ResponseBuf[16U] & 0xFU) == MMCSD_CMD6_GRP1_SDR104 && (cmd16GrpFunc == MMCSD_CMD6_GRP1_SDR104))
                {
                    tranSpeed = MMCSD_TRANSPEED_SDR104;
                    uhsMode = CSL_MMC_AC12_UHSMS_SDR104;
                    sdr104TuningRequired = TRUE;

                    /* Max freq supported is 208MHz */
                    clkFreq = 208000000U;

                    #ifdef DEBUG_MODE
                    DebugP_log("MMCSD_switch_card_speed: Request to switch to SDR104\n");
                    #endif
                }
                else if(((gCmd6ResponseBuf[16U] & 0xFU) == MMCSD_CMD6_GRP1_SDR50 ) && (cmd16GrpFunc == MMCSD_CMD6_GRP1_SDR50))
                {
                    tranSpeed = MMCSD_TRANSPEED_SDR50;
                    uhsMode = CSL_MMC_AC12_UHSMS_SDR50;

                    /* 100MHz for SDR50  */
                    clkFreq = 100000000U;

                    #ifdef DEBUG_MODE
                    DebugP_log("MMCSD_switch_card_speed: Request to switch to SDR50\n");
                    #endif
                }
                else if(((gCmd6ResponseBuf[16U] & 0xFU) == MMCSD_CMD6_GRP1_DDR50) && (cmd16GrpFunc == MMCSD_CMD6_GRP1_DDR50))
                {
                    tranSpeed = MMCSD_TRANSPEED_DDR50;
                    uhsMode=CSL_MMC_AC12_UHSMS_DDR50;

                    /* 50MHz for DDR50 mode */
                    clkFreq = 50000000U;

                    #ifdef DEBUG_MODE
                    DebugP_log("MMCSD_switch_card_speed: Request to switch to DDR50\n");
                    #endif
                }
                else if(((gCmd6ResponseBuf[16U] & 0xFU) == MMCSD_CMD6_GRP1_HS) && (cmd16GrpFunc == MMCSD_CMD6_GRP1_HS))
                {
                    tranSpeed = MMCSD_TRANSPEED_50MBPS;
                    uhsMode = CSL_MMC_AC12_UHSMS_SDR25;

                    /* 50MHz for HS mode */
                    clkFreq = 50000000U;

                    #ifdef DEBUG_MODE
                    DebugP_log("MMCSD_switch_card_speed: Request to switch to HS\n");
                    #endif
                }
                else if(((gCmd6ResponseBuf[16U] & 0xFU) == MMCSD_CMD6_GRP1_DEFAULT) && (cmd16GrpFunc == MMCSD_CMD6_GRP1_DEFAULT))
                {
                    tranSpeed = MMCSD_TRANSPEED_25MBPS;
                    uhsMode = CSL_MMC_AC12_UHSMS_SDR12;

                    /* 25MHz for SDR12 */
                    clkFreq = 25000000U;

                    #ifdef DEBUG_MODE
                    DebugP_log("MMCSD_switch_card_speed: Request to switch to SDR12\n");
                    #endif
                }
                else
                {
                    /* Speed switch not approved */
                    switchSpeedApproved = FALSE;
                }

                if(switchSpeedApproved)
                {
                    MMCSD_setDLLSWT(attrs->baseAddr, 0);
                    MMCSD_disableTuning(attrs->baseAddr);
                    MMCSD_linesReset(attrs->baseAddr, CSL_MMC_SYSCTL_SRC_MASK);
                    MMCSD_linesReset(attrs->baseAddr, CSL_MMC_SYSCTL_SRD_MASK);

                    /* Set input clock to make sure that the input clock is equal or higher to the clock value requested */
                    if(attrs->inputClockControl != NULL)
                    {
                        uint32_t inputClockRet;
                        if((inputClockRet = attrs->inputClockControl(attrs->instNum,&clkFreq,MMCSD_INPUT_CLOCK_CTRL_SET)) != 0)
                        {
                            attrs->inputClk = inputClockRet;
                        }
                        else
                        {
                            #ifdef DEBUG_MODE
                            DebugP_log("Unable to change input clock to %d\n",clkFreq);
                            #endif
                        }
                    }
                    if(SystemP_SUCCESS == MMCSD_setBusFreq(attrs->baseAddr, attrs->inputClk, clkFreq, 0U))
                    {
                        #ifdef DEBUG_MODE
                        DebugP_log("MMCSD_switch_card_speed: Setting Bus Frequency Succeeded \n");
                        #endif

                        if(attrs->iodelayFxn != NULL)
                        {
                            iodelayParams.transferSpeed = tranSpeed;
                            status = attrs->iodelayFxn(attrs->instNum, &iodelayParams);

                            if(SystemP_SUCCESS == status)
                            {
                                #ifdef DEBUG_MODE
                                DebugP_log("MMCSD_switch_card_speed: Setting I/O delay succeeded\n");
                                #endif
                            }
                            else
                            {
                                #ifdef DEBUG_MODE
                                DebugP_log("MMCSD_switch_card_speed: Setting I/O delay failed\n");
                                #endif
                            }
                        }
                        else
                        {
                            status = SystemP_SUCCESS;
                        }
                    }
                    HW_WR_FIELD32((attrs->baseAddr + CSL_MMC_SYSCTL), CSL_MMC_SYSCTL_CEN, 0);
                    MMCSD_setAC12UHSMode(attrs->baseAddr,uhsMode);
                    HW_WR_FIELD32((attrs->baseAddr + CSL_MMC_SYSCTL), CSL_MMC_SYSCTL_CEN, 1);

                    /* Tuning mandatory for SDR104 */
                    if(sdr104TuningRequired)
                    {
                        status = MMCSD_tuningProcedure(handle);
                        if(SystemP_FAILURE == status)
                        {
                            #ifdef DEBUG_MODE
                            DebugP_log("MMCSD_switch_card_speed: Tuning failed!\n");
                            #endif
                        }
                        else
                        {
                            #ifdef DEBUG_MODE
                            DebugP_log("MMCSD_switch_card_speed: Tuning Successfully completed\n");
                            #endif
                        }
                    }

                    if(status == SystemP_SUCCESS)
                    {
                        obj->tranSpeed = tranSpeed;
                    }
                }
                else
                {
                    status = SystemP_FAILURE;
                }
            }
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }
    return status;
}

static void MMCSD_delay(uint32_t delayValue)
{
    volatile uint32_t delayDec = delayValue*10000U;
    while (delayDec--) {}
}

static void MMCSD_isr(void *arg)
{

}

/* ========================================================================== */
/*                          PHY function definitions                          */
/* ========================================================================== */

/* ========================================================================== */
/*                     HW Abstraction function definitions                    */
/* ========================================================================== */
static int32_t MMCSD_softReset(uint32_t baseAddr)
{
    int32_t retVal = SystemP_SUCCESS;

    HW_WR_FIELD32((baseAddr + CSL_MMC_SYSCONFIG),
        CSL_MMC_SYSCONFIG_SOFTRESET, CSL_MMC_SYSCONFIG_SOFTRESET_ST_RST_W);

    while(CSL_MMC_SYSCONFIG_SOFTRESET_ONRESET_R !=
            HW_RD_FIELD32((baseAddr + CSL_MMC_SYSSTATUS),
            CSL_MMC_SYSSTATUS_RESETDONE))
    {
    }

    return retVal;
}

static void MMCSD_linesReset(uint32_t baseAddr, uint32_t resetMask)
{
    uint32_t regVal = 0U;

    regVal = HW_RD_REG32(baseAddr + CSL_MMC_SYSCTL);
    regVal |= resetMask;
    HW_WR_REG32((baseAddr + CSL_MMC_SYSCTL), regVal);

    while(resetMask == (HW_RD_REG32(baseAddr + CSL_MMC_SYSCTL) & resetMask))
    {}
}

static void MMCSD_systemConfig(uint32_t baseAddr, const MMCSD_sysCfg *pCfg)
{
    uint32_t regVal = 0U;
    uint32_t enableAutoIdle = 0U;
    uint32_t enableWakeup = 0U;

    regVal = HW_RD_REG32(baseAddr + CSL_MMC_SYSCONFIG);

    if(pCfg != NULL)
    {
        /* Set clock activity, standby mode and idle mode */
        HW_SET_FIELD(regVal, CSL_MMC_SYSCONFIG_CLOCKACTIVITY,
            pCfg->clockActivity);
        HW_SET_FIELD(regVal, CSL_MMC_SYSCONFIG_STANDBYMODE, pCfg->standbyMode);
        HW_SET_FIELD(regVal, CSL_MMC_SYSCONFIG_SIDLEMODE, pCfg->idleMode);

        /* Set wake up enable control */
        enableWakeup = (TRUE == pCfg->enableWakeup) ?
            CSL_MMC_SYSCONFIG_ENAWAKEUP_ENABLE :
            CSL_MMC_SYSCONFIG_ENAWAKEUP_DISABLED;
        HW_SET_FIELD(regVal, CSL_MMC_SYSCONFIG_ENAWAKEUP, enableWakeup);

        /* Set auto idle enable control */
        enableAutoIdle = (TRUE == pCfg->enableAutoIdle) ?
            CSL_MMC_SYSCONFIG_AUTOIDLE_ON :
            CSL_MMC_SYSCONFIG_AUTOIDLE_OFF;
        HW_SET_FIELD(regVal, CSL_MMC_SYSCONFIG_AUTOIDLE, enableAutoIdle);

        HW_WR_REG32((baseAddr + CSL_MMC_SYSCONFIG), regVal);
    }
}

static void MMCSD_setBusWidth(uint32_t baseAddr, uint32_t width)
{
    switch (width)
    {
        case MMCSD_BUS_WIDTH_8BIT:
            HW_WR_FIELD32((baseAddr + CSL_MMC_CON),
                CSL_MMC_CON_DW8, CSL_MMC_CON_DW8__8BITMODE);
            break;

        case MMCSD_BUS_WIDTH_4BIT:
            HW_WR_FIELD32((baseAddr + CSL_MMC_CON),
                CSL_MMC_CON_DW8, CSL_MMC_CON_DW8__1_4BITMODE);
            HW_WR_FIELD32((baseAddr + CSL_MMC_HCTL),
                CSL_MMC_HCTL_DTW, CSL_MMC_HCTL_DTW__4_BITMODE);
            break;

        case MMCSD_BUS_WIDTH_1BIT:
            HW_WR_FIELD32((baseAddr + CSL_MMC_CON),
                CSL_MMC_CON_DW8, CSL_MMC_CON_DW8__1_4BITMODE);
            HW_WR_FIELD32((baseAddr + CSL_MMC_HCTL),
                CSL_MMC_HCTL_DTW, CSL_MMC_HCTL_DTW__1_BITMODE);
            break;

        default:
            break;
    }
}

static void MMCSD_setPAD(uint32_t baseAddr, uint32_t val)
{
    HW_WR_FIELD32((baseAddr + CSL_MMC_CON), CSL_MMC_CON_PADEN, val);
}

static void MMCSD_setDLL(uint32_t baseAddr, uint32_t count)
{
    uint32_t regVal;
	int i;
	/*
 	       Set MMCHS_DLL[12] FORCE_VAL to 1
		   Set MMCHS_DLL[1] DLL_CALIB to 0x1. This transfers the FORCE_SR_C value to the peripheral delay line.
		   Set MMCHS_DLL[12] DLL_CALIB to 0
    */
	regVal=HW_RD_REG32(baseAddr + CSL_MMC_DLL);
	HW_SET_FIELD(regVal, CSL_MMC_DLL_FORCE_VALUE, CSL_MMC_DLL_FORCE_VALUE_FORCE);
	HW_SET_FIELD(regVal, CSL_MMC_DLL_FORCE_SR_C, count);
    HW_WR_REG32((baseAddr + CSL_MMC_DLL), regVal);

    HW_WR_FIELD32((baseAddr + CSL_MMC_DLL), CSL_MMC_DLL_DLL_CALIB, CSL_MMC_DLL_DLL_CALIB_ENABLED);

    for(i=0;i<1000;i++) {
	 if(HW_RD_FIELD32((baseAddr + CSL_MMC_DLL), CSL_MMC_DLL_DLL_CALIB))
	    break;
	}

    HW_WR_FIELD32((baseAddr + CSL_MMC_DLL), CSL_MMC_DLL_DLL_CALIB, CSL_MMC_DLL_DLL_CALIB_DISABLED);
}

static void MMCSD_disableTuning(uint32_t baseAddr)
{
	HW_WR_FIELD32((baseAddr + CSL_MMC_AC12), CSL_MMC_AC12_SCLK_SEL,CSL_MMC_AC12_SCLK_SEL_FIXED);
	HW_WR_FIELD32((baseAddr + CSL_MMC_DLL), CSL_MMC_DLL_FORCE_VALUE, CSL_MMC_DLL_FORCE_VALUE_NO_FORCE);
	HW_WR_FIELD32((baseAddr + CSL_MMC_DLL), CSL_MMC_DLL_FORCE_SWT, CSL_MMC_DLL_FORCE_SWT_DISABLE);
}

static void MMCSD_setDLLSWT(uint32_t baseAddr,uint32_t val)
{
	HW_WR_FIELD32((baseAddr + CSL_MMC_DLL), CSL_MMC_DLL_FORCE_SWT,val);
}

static uint32_t MMCSD_getDLLSWT(uint32_t baseAddr)
{
	return(HW_RD_FIELD32((baseAddr + CSL_MMC_DLL), CSL_MMC_DLL_FORCE_SWT));
}

static void MMCSD_setAC12SCLKSEL(uint32_t baseAddr,uint32_t val)
{
	HW_WR_FIELD32((baseAddr + CSL_MMC_AC12), CSL_MMC_AC12_SCLK_SEL,val);
}

static void MMCSD_setAC12ExecuteTuning(uint32_t baseAddr,uint32_t val)
{
	HW_WR_FIELD32((baseAddr + CSL_MMC_AC12), CSL_MMC_AC12_ET,val);
}

static uint32_t MMCSD_getAC12ExecuteTuning(uint32_t baseAddr)
{
	return(HW_RD_FIELD32((baseAddr + CSL_MMC_AC12), CSL_MMC_AC12_ET));
}

static uint32_t MMCSD_getAC12SCLKSEL(uint32_t baseAddr)
{
	return(HW_RD_FIELD32((baseAddr + CSL_MMC_AC12), CSL_MMC_AC12_SCLK_SEL));
}

static void MMCSD_setCLKEXTFree(uint32_t baseAddr, uint32_t val)
{
    HW_WR_FIELD32((baseAddr + CSL_MMC_CON), CSL_MMC_CON_CLKEXTFREE, val);
}

static void MMCSD_setAC12UHSMode(uint32_t baseAddr,uint32_t val)
{
	HW_WR_FIELD32((baseAddr + CSL_MMC_SYSCTL),
                        CSL_MMC_SYSCTL_CEN,
                        CSL_MMC_SYSCTL_CEN_DISABLE);

	HW_WR_FIELD32((baseAddr + CSL_MMC_AC12), CSL_MMC_AC12_UHSMS,val);

	HW_WR_FIELD32((baseAddr + CSL_MMC_SYSCTL),
                        CSL_MMC_SYSCTL_CEN,
                        CSL_MMC_SYSCTL_CEN_ENABLE);

}

static void MMCSD_setBusVolt(uint32_t baseAddr, uint32_t voltage)
{
    HW_WR_FIELD32((baseAddr + CSL_MMC_HCTL), CSL_MMC_HCTL_SDVS, voltage);
}

static uint32_t MMCSD_getBusVolt(uint32_t baseAddr)
{
    return(HW_RD_FIELD32((baseAddr + CSL_MMC_HCTL), CSL_MMC_HCTL_SDVS));
}

static int32_t MMCSD_busPowerOnCtrl(uint32_t baseAddr, uint32_t pwrCtrl)
{
    int32_t retVal = SystemP_SUCCESS;

    HW_WR_FIELD32((baseAddr + CSL_MMC_HCTL), CSL_MMC_HCTL_SDBP, pwrCtrl);

    if (MMCSD_PWR_CTRL_ON == pwrCtrl)
    {
        while (pwrCtrl != HW_RD_FIELD32((baseAddr + CSL_MMC_HCTL),
            CSL_MMC_HCTL_SDBP))
        {
        }
    }

    return retVal;
}

static int32_t MMCSD_intClockEnable(uint32_t baseAddr, uint32_t enableIntClk)
{
    uint32_t clkEnable = 0U;
    int32_t retVal = SystemP_SUCCESS;

    clkEnable = (CSL_TRUE == enableIntClk) ? CSL_MMC_SYSCTL_ICE_OSCILLATE :
        CSL_MMC_SYSCTL_ICE_STOP;
    HW_WR_FIELD32((baseAddr + CSL_MMC_SYSCTL), CSL_MMC_SYSCTL_ICE,
        clkEnable);

    if (TRUE == enableIntClk)
    {
        if (CSL_FALSE == MMCSD_isIntClockStable(baseAddr, 0xFFFFU))
        {
            retVal = SystemP_FAILURE;
        }
    }

    return retVal;
}

static uint32_t MMCSD_isIntClockStable(uint32_t baseAddr, uint32_t retry)
{
    uint32_t status = CSL_FALSE;

    do
    {
        if ((CSL_MMC_SYSCTL_ICS_READY == HW_RD_FIELD32((baseAddr + CSL_MMC_SYSCTL),
            CSL_MMC_SYSCTL_ICS)) || (0U == retry))
        {
            status = CSL_TRUE;
            break;
        }
    }
    while (retry--);

    return status;
}

static void MMCSD_setSupportedVoltage(uint32_t baseAddr, uint32_t voltMask)
{
    uint32_t regVal = HW_RD_REG32(baseAddr + CSL_MMC_CAPA);

    regVal &= ~(CSL_MMC_CAPA_VS33_MASK | CSL_MMC_CAPA_VS30_MASK |
        CSL_MMC_CAPA_VS18_MASK);
    regVal |= voltMask;
    HW_WR_REG32((baseAddr + CSL_MMC_CAPA), regVal);
}

static uint32_t MMCSD_isHighSpeedSupported(uint32_t baseAddr)
{
    return (HW_RD_FIELD32((baseAddr + CSL_MMC_CAPA), CSL_MMC_CAPA_HSS));
}

static uint32_t MMCSD_isUHSSDR50Supported(uint32_t baseAddr)
{
    return (HW_RD_FIELD32((baseAddr + CSL_MMC_CAPA2), CSL_MMC_CAPA2_SDR50));
}

static uint32_t MMCSD_isUHSSDR104Supported(uint32_t baseAddr)
{
    return (HW_RD_FIELD32((baseAddr + CSL_MMC_CAPA2), CSL_MMC_CAPA2_SDR104));
}

static uint32_t MMCSD_isUHSDDR50Supported(uint32_t baseAddr)
{
    return (HW_RD_FIELD32((baseAddr + CSL_MMC_CAPA2), CSL_MMC_CAPA2_DDR50));
}

static void MMCSD_setDataTimeout(uint32_t baseAddr, uint32_t timeout)
{
    timeout = (timeout - 13U) & 0xFU;
    HW_WR_FIELD32((baseAddr + CSL_MMC_SYSCTL), CSL_MMC_SYSCTL_DTO, timeout);
}

static int32_t MMCSD_setBusFreq(uint32_t baseAddr, uint32_t inputFreq, uint32_t outputFreq, uint32_t bypass)
{
    volatile uint32_t clkDiv = 0;
    int32_t retVal = SystemP_SUCCESS;

    /* First enable the internal clocks */
    if (SystemP_SUCCESS == MMCSD_intClockEnable(baseAddr, CSL_TRUE))
    {
        if (FALSE == bypass)
        {
            /* Calculate and program the divisor */
            clkDiv = inputFreq / outputFreq;
            if(clkDiv > 1023U)
            {
                clkDiv = 1023U;
            } else if (clkDiv==0) {
			    clkDiv = 1;
			}

            /* Do not cross the required freq */
            while(((inputFreq / clkDiv) > outputFreq) && (SystemP_SUCCESS == retVal))
            {
                if (1023U == clkDiv)
                {
                    /* Return we we cannot set the clock freq */
                   retVal = SystemP_FAILURE;
                }

                clkDiv++;
            }

            if(SystemP_SUCCESS == retVal)
            {
                HW_WR_FIELD32((baseAddr + CSL_MMC_SYSCTL),
                    CSL_MMC_SYSCTL_CLKD, clkDiv);

                /* Wait for the interface clock stabilization */
                if(TRUE == MMCSD_isIntClockStable(baseAddr, 0xFFFFU))
                {
                    /* Enable clock to the card */
                    HW_WR_FIELD32((baseAddr + CSL_MMC_SYSCTL),
                        CSL_MMC_SYSCTL_CEN,
                        CSL_MMC_SYSCTL_CEN_ENABLE);
                }
                else
                {
                    retVal = SystemP_FAILURE;
                }
            }
        }
    }
    else
    {
        retVal = SystemP_FAILURE;
    }

    return retVal;
}

static int32_t MMCSD_initStreamSend(uint32_t baseAddr)
{
    uint32_t status = 0U;

    /* Enable the command completion status to be set */
    MMCSD_intrStatusEnable(baseAddr, MMCSD_INTR_MASK_CMDCOMP);

    /* Initiate the INIT command */
    HW_WR_FIELD32((baseAddr + CSL_MMC_CON), CSL_MMC_CON_INIT,
        CSL_MMC_CON_INIT_INITSTREAM);
    HW_WR_REG32((baseAddr + CSL_MMC_CMD), 0x00U);

    status = MMCSD_isCmdComplete(baseAddr, 0xFFFFU);

    HW_WR_FIELD32((baseAddr + CSL_MMC_CON), CSL_MMC_CON_INIT,
        CSL_MMC_CON_INIT_NOINIT);

    /* Clear all status */
    MMCSD_intrClear(baseAddr, 0xFFFFFFFFU);

    return((int32_t)status);
}

static void MMCSD_intrStatusEnable(uint32_t baseAddr, uint32_t intrMask)
{
    uint32_t regVal = HW_RD_REG32(baseAddr + CSL_MMC_IE);
    regVal |= intrMask;
    HW_WR_REG32((baseAddr + CSL_MMC_IE), regVal);
}

static void MMCSD_intrStatusDisable(uint32_t baseAddr, uint32_t intrMask)
{
    uint32_t regVal = HW_RD_REG32(baseAddr + CSL_MMC_IE);
    regVal &= ~intrMask;
    HW_WR_REG32((baseAddr + CSL_MMC_IE), regVal);
}

static void MMCSD_intrEnable(uint32_t baseAddr, uint32_t intrMask)
{
    uint32_t regVal = 0U;

    regVal = HW_RD_REG32(baseAddr + CSL_MMC_ISE);
    regVal |= intrMask;
    HW_WR_REG32((baseAddr + CSL_MMC_ISE), regVal);

    MMCSD_intrStatusEnable(baseAddr, intrMask);

}

static void MMCSD_intrDisable(uint32_t baseAddr, uint32_t intrMask)
{
    uint32_t regVal = 0U;

    regVal = HW_RD_REG32(baseAddr + CSL_MMC_ISE);
    regVal &= ~intrMask;
    HW_WR_REG32((baseAddr + CSL_MMC_ISE), regVal);
}

static uint32_t MMCSD_intrGet(uint32_t baseAddr)
{
    return HW_RD_REG32(baseAddr + CSL_MMC_ISE);
}

static uint32_t MMCSD_intrStatus(uint32_t baseAddr)
{
    return HW_RD_REG32(baseAddr + CSL_MMC_STAT);
}

static void MMCSD_intrClear(uint32_t baseAddr, uint32_t intrMask)
{
    HW_WR_REG32((baseAddr + CSL_MMC_STAT), intrMask);
}

static uint32_t MMCSD_isCmdComplete(uint32_t baseAddr, uint32_t retry)
{
    volatile uint32_t status = SystemP_FAILURE;

    do
    {
        if ((CSL_MMC_STAT_CC_IRQ_TRU_R == HW_RD_FIELD32((baseAddr + CSL_MMC_STAT), CSL_MMC_STAT_CC)) || (0U  == retry))
        {
            status = SystemP_SUCCESS;
            break;
        }
    }
    while(retry--);

    return status;
}

static uint32_t MMCSD_isXferComplete(uint32_t baseAddr, uint32_t retry)
{
    volatile uint32_t status = CSL_FALSE;

    do
    {
        if ((CSL_MMC_STAT_TC_IRQ_TRU_R == HW_RD_FIELD32((baseAddr + CSL_MMC_STAT), CSL_MMC_STAT_TC)) || (0U == retry))
        {
            status = (uint32_t)CSL_TRUE;
            break;
        }
    }
    while(retry--);

    return status;
}

static void MMCSD_setBlkLength(uint32_t baseAddr, uint32_t blkLen)
{
    HW_WR_FIELD32((baseAddr + CSL_MMC_BLK), CSL_MMC_BLK_BLEN, blkLen);
}

static uint32_t MMCSD_getBlkLength(uint32_t baseAddr)
{
    return (HW_RD_FIELD32((baseAddr + CSL_MMC_BLK), CSL_MMC_BLK_BLEN));
}

static uint32_t MMCSD_getBlkCount(uint32_t baseAddr)
{
    return (HW_RD_FIELD32((baseAddr + CSL_MMC_BLK), CSL_MMC_BLK_NBLK));
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

static void MMCSD_SDGetResponse(uint32_t baseAddr, uint32_t *pRsp)
{
    pRsp[0U] = HW_RD_REG32(baseAddr + CSL_MMC_RSP10);
    pRsp[1U] = HW_RD_REG32(baseAddr + CSL_MMC_RSP32);
    pRsp[2U] = HW_RD_REG32(baseAddr + CSL_MMC_RSP54);
    pRsp[3U] = HW_RD_REG32(baseAddr + CSL_MMC_RSP76);
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

static uint32_t MMCSD_getCmdSignalLevel(uint32_t baseAddr)
{
    return (HW_RD_FIELD32((baseAddr + CSL_MMC_PSTATE), CSL_MMC_PSTATE_CLEV));
}

static uint32_t MMCSD_getDataSignalLevel(uint32_t baseAddr)
{
    return (HW_RD_FIELD32((baseAddr + CSL_MMC_PSTATE), CSL_MMC_PSTATE_DLEV));
}

static uint32_t MMCSD_isCardWriteProtected(uint32_t baseAddr)
{
    return HW_RD_FIELD32((baseAddr + CSL_MMC_PSTATE), CSL_MMC_PSTATE_WP);
}

static void MMCSD_getResponse(uint32_t baseAddr, uint32_t *pRsp)
{
    pRsp[0U] = HW_RD_REG32(baseAddr + CSL_MMC_RSP10);
    pRsp[1U] = HW_RD_REG32(baseAddr + CSL_MMC_RSP32);
    pRsp[2U] = HW_RD_REG32(baseAddr + CSL_MMC_RSP54);
    pRsp[3U] = HW_RD_REG32(baseAddr + CSL_MMC_RSP76);
}

uint32_t MMCSD_getBlockSize(MMCSD_Handle handle)
{
    MMCSD_Object *obj = ((MMCSD_Config *)handle)->object;

    uint32_t blockSize = 0U;
    blockSize = obj->blockSize;

    return blockSize;
}

uint32_t MMCSD_getBlockCount(MMCSD_Handle handle)
{
    MMCSD_Object *obj = ((MMCSD_Config *)handle)->object;

    uint32_t blockCount = 0U;
    if(obj != NULL)
    {
        blockCount = obj->blockCount;;
    }

    return blockCount;
}

static int32_t MMCSD_isCardInserted(uint32_t baseAddr)
{
    volatile int32_t retVal = 0;

    retVal = HW_RD_FIELD32(baseAddr + CSL_MMC_PSTATE, CSL_MMC_PSTATE_CINS);
    return retVal;
}