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

/**
 *  \defgroup DRV_MMCSD_MODULE APIs for MMCSD
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the MMCSD module. The APIs
 *  can be used by other drivers to get access to MMCSD and also by
 *  application to initiate data transfer operation.
 *
 *  @{
 */

/**
 *  \file v2/mmcsd.h
 *
 *  \brief MMCSD Driver API/interface file.
 */

#ifndef MMCSD_H_
#define MMCSD_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/hw_include/cslr_mmcsd.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define MMCSD_CARD_TYPE_SD                             (0U)
#define MMCSD_CARD_TYPE_EMMC                           (1U)
#define MMCSD_CARD_TYPE_MMC                            (2U)

/* useful in cases where SoC has MMCSD IP but no device attached to peripheral */
#define MMCSD_CARD_TYPE_NO_DEVICE                      (3U)

/*
* \brief Macros that can be used for selecting the bus/data width
*/
#define MMCSD_BUS_WIDTH_4BIT    (0x4U)
#define MMCSD_BUS_WIDTH_8BIT    (0x8U)
#define MMCSD_BUS_WIDTH_1BIT    (0x1U)


/*
* \brief Macros that can be used for selecting the media transfer speed
*/
#define MMCSD_TRANSPEED_25MBPS          (0x32U)
#define MMCSD_TRANSPEED_50MBPS          (0x5AU)
#define MMCSD_TRANSPEED_DEFAULT         (0x32U)
#define MMCSD_TRANSPEED_HS              (0x5AU)
#define MMCSD_TRANSPEED_SDR12           (0x32U)
#define MMCSD_TRANSPEED_SDR25           (0x5AU)
#define MMCSD_TRANSPEED_SDR50           (0xBU)
#define MMCSD_TRANSPEED_SDR104          (0x2BU)
#define MMCSD_TRANSPEED_DDR50           (0x3BU)
#define MMCSD_TRANSPEED_HS200           (0x2BU)
#define MMCSD_TRANSPEED_HS400           (0x4BU)
/*
* \brief Macros that can be used for selecting the bus voltage
*/
#define MMCSD_BUS_VOLT_1_8V    (0x5U) /* Embedded */
#define MMCSD_BUS_VOLT_3_0V    (0x6U) /* Typical */
#define MMCSD_BUS_VOLT_3_3V    (0x7U) /* Flattop */

/*
* \brief Macros that can be used for selecting command types
*/
#define MMCSD_CMD_TYPE_NORMAL          (0U)
#define MMCSD_CMD_TYPE_BUS_SUSPEND     (1U)
#define MMCSD_CMD_TYPE_FUNC_SEL        (2U)
#define MMCSD_CMD_TYPE_IO_ABORT        (3U)

/*
* \brief Macros that can be used for selecting command response types
*/
#define MMCSD_CMD_RSP_TYPE_NORSP            (0U)
#define MMCSD_CMD_RSP_TYPE_L136             (1U)
#define MMCSD_CMD_RSP_TYPE_L48              (2U)
#define MMCSD_CMD_RSP_TYPE_L48_BUSY         (3U)

/*
* \brief Macros that can be used for selecting transfer types
*/
#define MMCSD_CMD_XFER_TYPE_WRITE             (0U)
#define MMCSD_CMD_XFER_TYPE_READ              (1U)

/*
* \brief Macros that can be used for selecting supported MMC modes
*/
#define MMCSD_SUPPORT_MMC_DS       (0x01U) /* DS (Up to 26Mhz) */
#define MMCSD_SUPPORT_MMC_HS_SDR   (0x02U) /* HS (Up to 52Mhz) */
#define MMCSD_SUPPORT_MMC_HS_DDR   (0x04U) /* HS (Up to 52Mhz) */
#define MMCSD_SUPPORT_MMC_HS200    (0x08U) /* HS200 (Up to 200Mhz) */
#define MMCSD_SUPPORT_MMC_HS400    (0x10U) /* HS400 (Up to 400Mhz) */
#define MMCSD_SUPPORT_MMC_HS400_ES (0x20U) /* HS400 Enhanced Strobe (Up to 400Mhz) */
#define MMCSD_SUPPORT_MMC_ALL      (0xFFU) /* All modes */

/*
* \brief Macros that can be used for selecting supported SD modes
*/
#define MMCSD_SUPPORT_SD_DS       (0x01U)
#define MMCSD_SUPPORT_SD_HS       (0x02U)
#define MMCSD_SUPPORT_SD_SDR50    (0x04U)
#define MMCSD_SUPPORT_SD_SDR104   (0x08U)
#define MMCSD_SUPPORT_SD_DDR50    (0x10U)
#define MMCSD_SUPPORT_SD_ALL      (0xFFU) /* All modes */

/*
* \brief Macros that can be used for selecting PHY types
*/
#define MMCSD_PHY_TYPE_HW_PHY                 (0U)
#define MMCSD_PHY_TYPE_SW_PHY                 (1U)
#define MMCSD_PHY_TYPE_NO_PHY                 (2U)

/*
* \brief Macros that can be used for selecting PHY tuning type
*/
#define MMCSD_PHY_TUNING_TYPE_AUTO            (0U)
#define MMCSD_PHY_TUNING_TYPE_MANUAL          (1U)

/*
* \brief Macros that can be used for selecting PHY modes
*/
#define MMCSD_PHY_MODE_HS400                  (1U)
#define MMCSD_PHY_MODE_HS200                  (2U)
#define MMCSD_PHY_MODE_HSSDR50                (3U)
#define MMCSD_PHY_MODE_HSDDR50                (4U)
#define MMCSD_PHY_MODE_ENHANCED_STROBE        (5U)
#define MMCSD_PHY_MODE_SDR104                 (6U)
#define MMCSD_PHY_MODE_SDR50                  (7U)
#define MMCSD_PHY_MODE_DDR50                  (8U)
#define MMCSD_PHY_MODE_SDR25                  (9U)
#define MMCSD_PHY_MODE_SDR12                  (10U)
#define MMCSD_PHY_MODE_HS                     (11U)
#define MMCSD_PHY_MODE_DS                     (12U)

/*
* \brief Macros that can be used for selecting UHS 1 modes
*/
#define MMCSD_UHS_MODE_SDR12                 (0U)
#define MMCSD_UHS_MODE_SDR25                 (1U)
#define MMCSD_UHS_MODE_SDR50                 (2U)
#define MMCSD_UHS_MODE_SDR104                (3U)
#define MMCSD_UHS_MODE_DDR50                 (4U)
#define MMCSD_UHS_MODE_HS400                 (5U)
#define MMCSD_UHS_MODE_UHS2                  (7U)

typedef void* MMCSD_Handle;

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
/**
 *  \brief MMCSD instance attributes - used during init time
 */


typedef struct
{
	void* deviceData;
    /* Pointer to eMMC/SD device data structure. Memory for this structure has to be allocated in application */

    uint8_t *dataBuf;
    /* Pointer to a 512 byte dataBuffer used for temporary data transactions internal to driver like ECSD read, tuning etc. To be allocated by application */

} MMCSD_Params;

/**
 *  \brief eMMC device properties
 */
typedef struct
{
    uint32_t ocr;
    /* Operating conditions register */

    uint32_t rca;
    /* Relative card address register */

    uint32_t maxReadBlockLen;
    /* Maximum supported block length for read */

    uint32_t maxWriteBlockLen;
    /* Maximum supported block length for read */

    char manuDate[9];
    /* ASCII string with the date of manufacture */

    uint8_t manuID;
    /* Card manufacturer ID */

    char productName[7];
    /* Product name */

    /* From CSD */
    uint8_t specVersion;
    /* eMMC specification version */

    uint32_t blockCount;
    /* Number of blocks in the eMMC */

    uint8_t transferSpeed;
    /* Transfer speed in code - Freq Unit x Mult Factor */

    uint8_t supportedModes;
    /* Supported speed modes by the device - HS200, HS400 etc */

    uint8_t eStrobeSupport;
    /* Support of enhanced strobe */

    uint8_t driveStrength;
    /* Drive strength of the device */

} MMCSD_EmmcDeviceData;

/**
 *  \brief SD device properties
 */
typedef struct
{
    uint32_t ocr;
    /* Operating conditions register */

    uint32_t rca;
    /* Relative card address register */

    uint32_t maxReadBlockLen;
    /* Maximum supported block length for read */

    uint32_t maxWriteBlockLen;
    /* Maximum supported block length for read */

    char manuDate[9];
    /* ASCII string with the date of manufacture */

    uint8_t manuID;
    /* Card manufacturer ID */

    char productName[6];
    /* Product name */

    /* From CSD */
    uint8_t specVersion;
    /* SD card specification version */

     uint32_t blockCount;
    /* Number of blocks in the SD */

    uint8_t transferSpeed;
    /* Transfer speed in code - Freq Unit x Mult Factor */

    uint32_t isCmd23;
    /* CMD23 support */

    uint32_t supportedDataWidths;
    /* Supported data widths by the device */

} MMCSD_SdDeviceData;

/**
 *  \brief  MMCSD transaction
 *
 *  This structure defines the nature of the MMCSD transaction. This structure
 *  specifies the buffer and buffer's size that is to be written to or read from
 *  the MMC slave peripheral.
 */
typedef struct
{
    uint32_t cmd;
    /**< Command register content composed of CMD ID, DP, TYPE, RESP TYPE etc */

    uint32_t dir;
    /**< Direction of transfer: Read/Write */

    uint32_t arg;
    /**< Command argument as per MMC device specification */

    void    *dataBuf;
    /**< buffer containing data to be read into or written */

    uint32_t blockSize;
    /**< Number of bytes to be transferred per block */

    uint32_t blockCount;
    /**< Number of block to be transferred */

    uint32_t autoCmdEn;
    /* AutoCMD12 or AutoCMD23 or no AutoCMD */

    uint32_t enableDma;
    /* Is DMA enabled for the command*/

    uint32_t isTuning;
    /* Is transaction used for tuning */

    uint32_t response[4];
    /**< Command response per MMC device specification */

} MMCSD_Transaction;

/**
 *  \brief MMCSD instance attributes - used during init time
 */

typedef struct
{
    uint32_t ctrlBaseAddr;
    /**< MMCSD Host control registers base address */

    uint32_t ssBaseAddr;
	/**< MMCSD subsystem registers base address */

    uint32_t inputClkFreq;
    /**< Module input clock frequency */

    uint32_t outputClkFreq;
    /**< Module output clock frequency */

    uint32_t enableDma;
    /**< DMA enable */

    uint32_t intrEnable;
    /**< Module interrupt enable */

    uint32_t intrNum;
    /**< Module interrupt vector */

    uint32_t eventId;
    /**< Module interrupt event ID */

    uint32_t cardType;
    /**< Type of card */

    uint32_t busWidth;
    /**< Supported bus width */

    uint32_t supportedModes;
    /**< Supported bus width */

    uint32_t busVoltage;
    /**< Supported bus voltages */

    uint32_t isHS;
    /**< Is high speed supported */

    uint32_t phyType;
    /**< HW or SW PHY */

    uint32_t tuningType;
    /**< Manual SW tuning or auto HW tuning for SDR104/HS200/HS400 modes */

} MMCSD_Attrs;

/**
 *  \brief MMCSD driver object
 */

typedef struct
{
    MMCSD_Handle handle;
    /**< Instance handle */

    uint32_t cardType;
    /**< Type of card */

    uint8_t *tempDataBuf;
    /* Temporary data buf for receiving tuning data, ecsd, scr etc. To be allocated in app and passed as MMCSD params to MMCSD_open */

    MMCSD_EmmcDeviceData *emmcData;
    /* EMMC device data structure. This has to be allocated in the app. Probably using sysconfig */

    MMCSD_SdDeviceData *sdData;
    /* SD device data structure. This has to be allocated in the app. Probably using sysconfig */

    uint8_t sdVer;
    /**< Version of SD card */

    uint32_t busWidth;
    /**< Current bus width */

    uint32_t transferSpeed;
    /**< Current transfer speed */

    uint32_t isHC;
    /**< Is card of high capacity */

    uint32_t isUHS;
    /**< Is card UHS */

    uint32_t isCmd23;
    /**< Is command 23 supported */

    uint32_t is1_8V;
    /**< Is 1.8V supported by card */

    uint32_t isSwitch1_8V;
    /**< Is the card switched to 1.8V */

    uint32_t blockSize;
    /**< Size of a block in bytes */

    uint64_t blockCount;
    /**< Number of blocks */

    uint64_t mediaSize;
    /**< Size of the card in bytes */

    uint32_t enableDma;
    /**< DMA enable */

    uint32_t intrEnable;
    /**< Module interrupt enable */

    volatile uint32_t cmdComp;
    /**< Command completion flag */

    volatile uint32_t cmdTimeout;
    /*< Command timeout flag */

    volatile uint32_t cmdCRCError;
    /*< Command CRC error flag */

    volatile uint32_t cmdEBError;
    /*< Command CRC error flag */

    volatile uint32_t cmdIndexError;
    /*< Command Index error flag */

    volatile uint32_t dataCRCError;
    /*< Data CRC error flag */

    volatile uint32_t dataEBError;
    /*< Data end bit error */

	volatile uint32_t cmdError;
	/*< Any error in processing of the command */

    volatile uint32_t xferInProgress;
    /*< Command completion flag */

    volatile uint32_t xferComp;
    /*< Transfer completion flag */

    volatile uint32_t xferTimeout;
    /*< Transfer timeout flag */

    uint8_t *dataBufIdx;
    uint32_t dataBlockCount;
    uint32_t dataBlockSize;

    uint8_t *readBufIdx;
    uint32_t readBlockCount;

    uint8_t *writeBufIdx;
    uint32_t writeBlockCount;

    uint32_t isManualTuning;
    /**< Flag to enable manual tuning */

    uint32_t isOpen;
    /**< Flag to indicate if the instance is already open */

    uint32_t xferHighSpeedEn;
    /**< Flag to enable HS200 transfer mode */

    SemaphoreP_Object       cmdMutex;
    /**< Command Mutex */

    SemaphoreP_Object       xferMutex;
    /**< Transfer Mutex */

    SemaphoreP_Object       cmdCompleteSemObj;
    /**< Command complete semaphore */

    SemaphoreP_Object       dataCopyCompleteSemObj;
    /**< Data buffer copy complete semaphore */

    SemaphoreP_Object       xferCompleteSemObj;
    /**< Transfer complete semaphore */

    HwiP_Object             hwiObj;
    /**< Interrupt object */

} MMCSD_Object;

typedef struct
{
    const MMCSD_Attrs *attrs;
    /**< Pointer to driver specific hardware attributes */
    MMCSD_Object *object;
    /**< Pointer to driver specific data object */
} MMCSD_Config;

/* ========================================================================== */
/*                                Externs                                     */
/* ========================================================================== */

/** \brief Externally defined driver configuration array */
extern MMCSD_Config gMmcsdConfig[];
/** \brief Externally defined driver configuration array size */
extern uint32_t    gMmcsdConfigNum;

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
 *  \brief  This function initializes the MMCSD module
 */
void MMCSD_init(void);

/**
 *  \brief  This function de-initializes the MMCSD module
 */
void MMCSD_deinit(void);

/**
 *  \brief  Initialize data structure with defaults
 *
 *  \param  mmcsdParams [out] Initialized parameters
 */
void MMCSD_Params_init(MMCSD_Params *mmcsdParams);

/**
 *  \brief  This function opens a given MMCSD peripheral
 *
 *  \pre    MMCSD controller has been initialized using #MMCSD_init()
 *
 *  \param  index       Index of config to use in the *MMCSD_Config* array
 *  \param  openParams  Pointer to parameters to open the driver with
 *
 *  \return A #MMCSD_Handle on success or a NULL on an error or if it has been
 *          opened already
 *
 *  \sa     #MMCSD_init()
 *  \sa     #MMCSD_close()
 */
MMCSD_Handle MMCSD_open(uint32_t index, const MMCSD_Params *openParams);

/**
 *  \brief  Function to close a MMCSD peripheral specified by the MMCSD handle
 *
 *  \pre    #MMCSD_open() has to be called first
 *
 *  \param  handle      #MMCSD_Handle returned from #MMCSD_open()
 *
 *  \sa     #MMCSD_open()
 */
void MMCSD_close(MMCSD_Handle handle);

/**
 *  \brief  This function returns the handle of an open MMCSD Instance from the instance index
 *
 *  \pre    MMCSD controller has been opened using #MMCSD_open()
 *
 *  \param  index Index of config to use in the *MMCSD_Config* array
 *
 *  \return An #MMCSD_Handle if it has been opened already or NULL otherwise
 *
 *  \sa     #MMCSD_init()
 *  \sa     #MMCSD_open()
 */
MMCSD_Handle MMCSD_getHandle(uint32_t index);

/**
 *  \brief  Function to perform block reads from the MMC/SD media
 *
 *
 *  \param  handle     #MMCSD_Handle returned from #MMCSD_open()
 *  \param  buf        Pointer to a buffer to which the data is to be read into
 *  \param  startBlk   Block to start reading data from
 *  \param  numBlks    Number of blocks to read
 *
 *  \return #SystemP_SUCCESS on successful read; else error on failure
 *
 *  \sa     #MMCSD_open
 */
int32_t MMCSD_read(MMCSD_Handle handle, uint8_t *buf, uint32_t startBlk, uint32_t numBlks);

/**
 *  \brief  Function to perform block writes to the MMC/SD media
 *
 *
 *  \param  handle     #MMCSD_Handle returned from #MMCSD_open()
 *  \param  buf        Pointer to a buffer to which the data is to be read into
 *  \param  startBlk   Block to start reading data from
 *  \param  numBlks    Number of blocks to read
 *
 *  \return #SystemP_SUCCESS on successful read; else error on failure
 *
 *  \sa     #MMCSD_open
 */
int32_t MMCSD_write(MMCSD_Handle handle, uint8_t *buf, uint32_t startBlk, uint32_t numBlks);

/**
 *  \brief  This function returns the block size of the MMC/SD media connected to the MMCSD controller
 *
 *  \pre    MMCSD controller has been opened using #MMCSD_open()
 *
 *  \param  handle     #MMCSD_Handle returned from #MMCSD_open()
 *
 *  \return Block size of the media
 *
 *  \sa     #MMCSD_init()
 *  \sa     #MMCSD_open()
 */
uint32_t MMCSD_getBlockSize(MMCSD_Handle handle);

/**
 *  \brief  This function returns the block count of User Data Area of the MMC/SD media connected to the MMCSD controller
 *
 *  \pre    MMCSD controller has been opened using #MMCSD_open()
 *
 *  \param  handle     #MMCSD_Handle returned from #MMCSD_open()
 *
 *  \return Block size of the media
 *
 *  \sa     #MMCSD_init()
 *  \sa     #MMCSD_open()
 */
uint32_t MMCSD_getBlockCount(MMCSD_Handle handle);

/**
 *  \brief  This function returns if the media connected is High Capacity (> 2GB in size) or not
 *
 *  \pre    MMCSD controller has been opened using #MMCSD_open()
 *
 *  \param  handle     #MMCSD_Handle returned from #MMCSD_open()
 *
 *  \return Block size of the media
 *
 *  \sa     #MMCSD_init()
 *  \sa     #MMCSD_open()
 */
uint32_t MMCSD_isHC(MMCSD_Handle handle);

/**
 *  \brief  This function enables the boot partition if the connected media is eMMC
 *
 *  \pre    MMCSD controller has been opened using #MMCSD_open()
 *
 *  \param  handle         #MMCSD_Handle returned from #MMCSD_open()
 *  \param  partitionNum   Boot partition to be enabled.
 *
 *  \return #SystemP_SUCCESS on successful read; else error on failure
 *
 *  \sa     #MMCSD_init()
 *  \sa     #MMCSD_open()
 */
int32_t MMCSD_enableBootPartition(MMCSD_Handle handle, uint32_t partitionNum);

/**
 *  \brief  This function disables the boot partition if the connected media is eMMC
 *
 *  \pre    MMCSD controller has been opened using #MMCSD_open()
 *
 *  \param  handle         #MMCSD_Handle returned from #MMCSD_open()
 *
 *  \return #SystemP_SUCCESS on successful read; else error on failure
 *
 *  \sa     #MMCSD_init()
 *  \sa     #MMCSD_open()
 */
int32_t MMCSD_disableBootPartition(MMCSD_Handle handle);

/**
 *  \brief  This function returns the input clk frequency MMCSD was programmed at
 *
 *  \pre    MMCSD controller has been opened using #MMCSD_open()
 *
 *  \param  handle  An #MMCSD_Handle returned from an #MMCSD_open()
 *
 *  \return MMCSD CLK in Hertz
 */
uint32_t MMCSD_getInputClk(MMCSD_Handle handle);



/** @} */

#ifdef __cplusplus
}
#endif

#endif /* MMCSD_H_ */
