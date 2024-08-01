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
 *  \defgroup DRV_MMCSD_LLD_MODULE APIs for MMCSD LLD
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the MMCSD LLD modules.
 *  The APIs can be used by other drivers to get access to MMCSD and also by
 *  application to initiate transaction operations.
 *
 *  The MMCSD LLD header file should be included in an application as follows:
 *  \code
 *  #include <drivers/MMCSD/v0/lld/mmcsd_lld.h>
 *  \endcode
 *
 *  @{
 */

/**
 *  \file v0/lld/mmcsd_lld.h
 *  \brief MMCSD LLD Driver API/interface file.
 */

#ifndef MMCSD_LLD_H_
#define MMCSD_LLD_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/cslr64.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/cslr_mmcsd.h>
#include <drivers/mmcsd/v0/cslr_mmcsd.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  \anchor MMCSD_StatusCode
 *  @name Return status
 *  @{
 */

/** \brief Return status when the API execution was successful */
#define MMCSD_STS_SUCCESS                       ((int32_t) 0)
/** \brief Return status when the API execution was not successful
 *  due to a generic Error */
#define MMCSD_STS_ERR                           ((int32_t)-1)
/** \brief Return status when the API execution was not successful
 *  due to a time out */
#define MMCSD_STS_ERR_TIMEOUT                   ((int32_t)-2)
/** \brief Return status when the API execution failed due invalid parameters */
#define MMCSD_STS_ERR_INVALID_PARAM             ((int32_t)-3)
/** \brief Return status when the API execution failed due to driver busy */
#define MMCSD_STS_ERR_BUSY                      ((int32_t)-4)
/** \brief Return status when the API execution failed due to missing card */
#define MMCSD_STS_ERR_CARD_NOT_FOUND            ((int32_t)-5)
/** \brief Return status when the API execution failed due to unuseable card */
#define MMCSD_STS_ERR_CARD_UNUSEABLE            ((int32_t)-6)
/** \brief Return status when the API execution failed due to card Unlock Failure */
#define MMCSD_STS_ERR_CARD_UNLOCK_FAIL          ((int32_t)-7)

/** @} */

/**
 *  \anchor MMCSDTimeoutValues
 *  @name Timeout values
 *  @{
 */

/** \brief Value to use when needing a timeout of zero or NO timeout,
 *  return immediately on resource not available. */
#define MMCSD_NO_WAIT                           ((uint32_t)0)
/** \brief Value to use when needing a timeout of infinity or
 *  wait forver until resource is available */
#define MMCSD_WAIT_FOREVER                      ((uint32_t)-1)

/** @} */

/**
 *  \anchor MMCSD_DriverStates
 *  @name MMCSD Driver states
 *  @{
 */

/** \brief MMCSD driver is in Reset State prior to driver init
 *  and post driver deinit */
#define MMCSD_STATE_RESET                       ((uint8_t) 0U)
/** \brief MMCSD driver accepts runtime APIs only Ready State,
 *  otherwise return error */
#define MMCSD_STATE_IDLE                        ((uint8_t) 1U)
/** \brief MMCSD driver is busy performing operation with peripherals,
 *  return error when APIs are invoked */
#define MMCSD_STATE_BUSY                        ((uint8_t) 2U)
/** \brief MMCSD driver ran into error, returns error for all APIs
 *  other than deinit in this state */
#define MMCSD_STATE_ERROR                       ((uint8_t) 3U)

/** @} */

/**
 *  \anchor MMCSD_XferStates
 *  @name MMCSD Transfer states
 *  @{
 */

/** \brief  XFER is in IDLE state */
#define MMCSD_XFER_IDLE_STATE                   ((uint8_t) 0U)
/** \brief  XFER is in CMD state */
#define MMCSD_XFER_CMD_STATE                    ((uint8_t) 1U)
/** \brief  XFER is in Write state */
#define MMCSD_XFER_WRITE_STATE                  ((uint8_t) 2U)
/** \brief  XFER is in Read state */
#define MMCSD_XFER_READ_STATE                   ((uint8_t) 3U)

/** @} */

/**
 *  \anchor MMCSDSlotType
 *  \name MACROS used to select one of the Card Slot Types.
 *  @{
 */

#define MMCSD_SLOT_TYPE_VAL_REMOVABLE           ((uint8_t) 0x0U)
#define MMCSD_SLOT_TYPE_VAL_EMBEDDED            ((uint8_t) 0x1U)
#define MMCSD_SLOT_TYPE_VAL_SHARED              ((uint8_t) 0x2U)

/** @} */

/**
 *  \anchor MMCSDCardType
 *  \name MACROS used to select one of the possible Card Types.
 *  @{
 */

#define MMCSD_CARD_TYPE_SD                      ((uint32_t) 0U)
#define MMCSD_CARD_TYPE_EMMC                    ((uint32_t) 2U)
#define MMCSD_CARD_TYPE_NO_DEVICE               ((uint32_t) 3U)

/** @} */

/**
 *  \anchor MMCSDPhyType
 *  \name MACROS used to select one of the possible PHY Types.
 *  @{
 */

#define MMCSD_PHY_TYPE_HW_PHY                   ((uint32_t) 0U)
#define MMCSD_PHY_TYPE_SW_PHY                   ((uint32_t) 1U)
#define MMCSD_PHY_TYPE_NO_PHY                   ((uint32_t) 2U)

/** @} */

/**
 *  \anchor MMCSDBusWidth
 *  \name MACROS used to select one of the possible Bus Widths.
 *  @{
 */

#define MMCSD_BUS_WIDTH_4BIT                    ((uint32_t) 0x4U)
#define MMCSD_BUS_WIDTH_8BIT                    ((uint32_t) 0x8U)
#define MMCSD_BUS_WIDTH_1BIT                    ((uint32_t) 0x1U)

/** @} */

/**
 *  \anchor MMCSDPhyTuningType
 *  \name MACROS used to select one of the possible PHY tuning types.
 *  @{
 */

#define MMCSD_PHY_TUNING_TYPE_AUTO              ((uint32_t) 0U)
#define MMCSD_PHY_TUNING_TYPE_MANUAL            ((uint32_t) 1U)

/** @} */

/**
 *  \anchor MMCSDSpeedModesSD
 *  \name MACROS used to select one of the possible Speed Modes for SD Device.
 *  @{
 */

#define MMCSD_SD_MODE_HS                        ((uint32_t) 10U)
#define MMCSD_SD_MODE_DS                        ((uint32_t) 11U)
#define MMCSD_SD_MODE_SDR12                     ((uint32_t) 12U)
#define MMCSD_SD_MODE_SDR25                     ((uint32_t) 13U)
#define MMCSD_SD_MODE_SDR50                     ((uint32_t) 14U)
#define MMCSD_SD_MODE_DDR50                     ((uint32_t) 15U)
#define MMCSD_SD_MODE_SDR104                    ((uint32_t) 16U)

/** @} */

/**
 *  \anchor MMCSDSpeedModesEMMC
 *  \name MACROS used to select one of the possible Speed Modes for EMMC Device.
 *  @{
 */

#define MMCSD_MMC_MODE_SDR25                    ((uint32_t) 20U)
#define MMCSD_MMC_MODE_SDR50                    ((uint32_t) 21U)
#define MMCSD_MMC_MODE_HS200                    ((uint32_t) 23U)

/** @} */

/* ========================================================================== */
/*                      Function Pointers Declarations                        */
/* ========================================================================== */

/**
 *  \brief  The definition of a get System Tick function used by
 *  the MMCSD driver to keep track of time
 *
 *  \return Returns system ticks in 32-bit unsigned int format
 *
 */
typedef uint32_t (*MMCSD_Clock_getTicks) (void);

/**
 *  \brief  The definition of a micro seconds to ticks function used by
 *  the MMCSD driver to get ticks from microseconds
 *
 *  \param usecs                        Micro Seconds
 *
 *  \return Returns system ticks in 32-bit unsigned int format
 *
 */
typedef uint32_t (*MMCSD_Clock_usecToTicks) (uint64_t usecs);

/**
 *  \brief  The definition of a sleep function used by
 *  the MMCSD driver for delay
 *
 *  \param usec                         Micro Seconds
 *
 */
typedef void (*MMCSD_Clock_uSleep) (uint32_t usec);

/**
 *  \brief  The definition of a transfer completion callback function used by
 *  the MMCSD driver when used in Callback Mode
 *
 *  \param args                         Void Pointer
 *  \param xferStatus                   Transfer Status
 */
typedef void (*MMCSD_lld_transferCompleteCallback) (void *args,
                                                    int32_t xferStatus);

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief eMMC device properties
 */
typedef struct {

/** Operating conditions register Value */
    uint32_t                ocr;
/** Relative card address */
    uint32_t                rca;
/** Maximum supported block length for read */
    uint16_t                maxReadBlockLen;
/** Maximum supported block length for write */
    uint16_t                maxWriteBlockLen;
/** ASCII string with the date of manufacture */
    char                    manuDate[9];
/** Card manufacturer ID */
    uint8_t                 manuID;
/** Product name */
    char                    productName[7];
/** eMMC specification version */
    uint8_t                 specVersion;
/** Number of blocks in the eMMC */
    uint32_t                blockCount;
/** Transfer speed in code - Freq Unit x Mult Factor */
    uint8_t                 transferSpeed;
/** Supported speed modes by the device - HS200, HS400 etc */
    uint8_t                 supportedModes;
/** Support of enhanced strobe */
    uint8_t                 eStrobeSupport;
/** Drive strength of the device */
    uint8_t                 driveStrength;
/** Card whether high Capacity or not */
    bool                    isHC;

} MMCSD_EmmcDeviceData;

/**
 *  \brief SD device properties
 */
typedef struct {

/** Operating conditions register */
    uint32_t                ocr;
/** Relative card address register */
    uint32_t                rca;
/** Maximum supported block length for read */
    uint16_t                maxReadBlockLen;
/** Maximum supported block length for write */
    uint16_t                maxWriteBlockLen;
/** ASCII string with the date of manufacture */
    char                    manuDate[9];
/** Card manufacturer ID */
    uint8_t                 manuID;
/** Product name */
    char                    productName[6];
/** SD card specification version */
    uint8_t                 specVersion;
/** Number of blocks in the SD */
    uint32_t                blockCount;
/** Transfer speed in code - Freq Unit x Mult Factor */
    uint8_t                 transferSpeed;
/** CMD23 support */
    uint32_t                isCmd23;
/** Supported data widths by the device */
    uint32_t                supportedDataWidths;
/** Card whether high Capacity or not */
    bool                    isHC;

} MMCSD_SdDeviceData;

/**
 *  \brief  MMCSDLLD Transaction
 *
 *  This structure defines the nature of the MMCSD transaction. This structure
 *  specifies the buffer and buffer's size that is to be written to or read from
 *  the MMC peripheral.
 */
typedef struct {

/** Command register content composed of CMD ID, DP, TYPE, RESP TYPE etc */
    uint32_t                cmd;
/** Direction of transfer: Read/Write */
    uint32_t                dir;
/** Command argument as per MMC device specification */
    uint32_t                arg;
/** Buffer containing data to be read into or written from */
    uint8_t                 *dataBuf;
/** Number of bytes to be transferred per block */
    uint32_t                blockSize;
/** Number of block to be transferred */
    uint32_t                blockCount;
/** AutoCMD12 or AutoCMD23 or no AutoCMD */
    uint32_t                autoCmdEn;
/** Is DMA enabled for the command */
    uint32_t                enableDma;
/** Is transaction used for tuning */
    uint32_t                isTuning;
/** Command response per MMC device specification */
    uint32_t                response[4];

} MMCSDLLD_Transaction;

/**
 *  \brief  MMCSD Driver Initialization Object
 */
typedef struct {

/** MMCSD Host control registers base address */
    uint32_t                ctrlBaseAddr;
/** MMCSD subsystem registers base address */
    uint32_t                ssBaseAddr;
/** Module input clock frequency */
    uint32_t                inputClkFreq;
/** Type of card \ref MMCSDCardType */
    uint32_t                cardType;
/** Auto Assign Maximum Speed Flag */
    bool                    autoAssignMaxSpeed;
/** User Assigned Bus Speed */
    uint32_t                uaBusSpeed;
/** Manual SW tuning or auto HW tuning for SDR104/HS200/HS400 modes
 * \ref MMCSDPhyTuningType*/
    uint32_t                tuningType;
/** Module interrupt vector */
    uint32_t                intrNum;
/** Slot Type \ref MMCSDSlotType */
    uint32_t                slotType;
/** Supported bus width \ref MMCSDBusWidth */
    uint32_t                busWidth;
/** DMA enable */
    bool                    enableDma;
/** HW or SW PHY \ref MMCSDPhyType */
    uint32_t                phyType;
/** PLL enable */
    bool                    pllEnableSD;
/** Pointer to device Data Structure, allocated and assigned by syscfg */
    void                    *deviceData;
/** Pointer to a 512 byte dataBuffer used for temporary data
 * transactions internal to driver like ECSD read, tuning etc.
 * This data is allocated by syscfg */
    uint8_t                 *dataBuf;
/** Clock_getTicks Function Pointer used by driver */
    MMCSD_Clock_getTicks    Clock_getTicks;
/** Clock_uSleep Function Pointer used by driver */
    MMCSD_Clock_uSleep      Clock_uSleep;

} MMCSDLLD_InitObject, *MMCSDLLD_InitHandle;


/**
 *  \brief  MMCSD Driver Object
 */
typedef struct {

/** Command Error Status */
    uint16_t                cmdErrorStat;
/** Transfer Error Status */
    uint16_t                xferErrorStat;
/** Data buffer index */
    uint8_t                 *dataBufIdx;
/** Data Block count used for Transfer */
    uint32_t                dataBlockCount;
/** Data Block size used for Transfer */
    uint32_t                dataBlockSize;
/** Remaining Block Count */
    uint32_t                remainingBlockCount;
/** Pointer to the current Transaction */
    MMCSDLLD_Transaction    *currentTxn;
/** MMCSDLLD_Transaction object used by driver */
    MMCSDLLD_Transaction    mmcsdTxn;
/** Set transfer Speed for the Device
 * \ref MMCSDSpeedModesSD \ref MMCSDSpeedModesEMMC*/
    uint32_t                setBusSpeed;
/** Set bus width \ref MMCSDBusWidth */
    uint32_t                setBusWidth;
/** Stores the MMCSD Transaction state \ref MMCSD_XferStates*/
    uint8_t                 xferState;
/** Stores the MMCSD state \ref MMCSD_DriverStates */
    uint8_t                 state;
/** Initialization parameters of MMCSD instance */
    MMCSDLLD_InitHandle     initHandle;
/** Pointer to be used by application to store miscellaneous data */
    void                    *args;
/** Callback Function Pointer */
    MMCSD_lld_transferCompleteCallback          transferCompleteCallback;

} MMCSDLLD_Object, *MMCSDLLD_Handle;

/* ========================================================================== */
/*                        API Function Declarations                           */
/* ========================================================================== */

/**
 *  \brief  This API Initializes the MMCSD instance.
 *
 *  \param  handle      [IN] Handle to the MMCSD instance used.
 *
 *  \return \ref MMCSD_StatusCode
 */
int32_t MMCSD_lld_init(MMCSDLLD_Handle handle);

/**
 *  \brief This API De-Initializes the MMCSD instance.
 *
 *  \param  handle      [IN] Handle to the MMCSD instance used.
 *
 *  \return \ref MMCSD_StatusCode
 */
int32_t MMCSD_lld_deInit(MMCSDLLD_Handle handle);

/**
 *  \brief  This function returns the block size of the MMC/SD media
 *          connected to the MMCSD controller.
 *
 *  \pre    MMCSD controller has been initialized using #MMCSD_lld_init().
 *
 *  \param  handle      [IN] Handle to the MMCSD instance used.
 *
 *  \return Block size of the media.
 *
 *  \sa     #MMCSD_lld_init()
 */
uint32_t MMCSD_lld_getBlockSize(MMCSDLLD_Handle handle);

/**
 *  \brief  Function to perform block writes to the SD media in Polling Mode.
 *
 *  \param  handle      [IN] Handle to the MMCSD instance used.
 *  \param  buf         [IN] Pointer to buffer from which data is to be written from.
 *  \param  startBlk    [IN] Block to start Writing data from.
 *  \param  numBlks     [IN] Number of blocks to write.
 *
 *  \return #MMCSD_STS_SUCCESS on successful execution of API; else error on failure.
 */
int32_t MMCSD_lld_write_SD_Poll(MMCSDLLD_Handle handle, uint8_t *buf,
                                uint32_t startBlk, uint32_t numBlks);

/**
 *  \brief  Function to perform block reads from the SD media in Polling Mode.
 *
 *  \param  handle      [IN] Handle to the MMCSD instance used.
 *  \param  buf         [IN] Pointer to buffer to which data is to be read into.
 *  \param  startBlk    [IN] Block to start reading data from.
 *  \param  numBlks     [IN] Number of blocks to read.
 *
 *  \return #MMCSD_STS_SUCCESS on successful execution of API; else error on failure.
 */
int32_t MMCSD_lld_read_SD_Poll(MMCSDLLD_Handle handle, uint8_t *buf,
                               uint32_t startBlk, uint32_t numBlks);

/**
 *  \brief  Function to perform block writes to the MMC media in Polling Mode.
 *
 *  \param  handle      [IN] Handle to the MMCSD instance used.
 *  \param  buf         [IN] Pointer to buffer from which data is to be written from.
 *  \param  startBlk    [IN] Block to start Writing data from.
 *  \param  numBlks     [IN] Number of blocks to write.
 *
 *  \return #MMCSD_STS_SUCCESS on successful execution of API; else error on failure.
 */
int32_t MMCSD_lld_write_MMC_Poll(MMCSDLLD_Handle handle, uint8_t *buf,
                                 uint32_t startBlk, uint32_t numBlks);

/**
 *  \brief  Function to perform block reads from the MMC media in Polling Mode.
 *
 *  \param  handle      [IN] Handle to the MMCSD instance used.
 *  \param  buf         [IN] Pointer to buffer to which data is to be read into.
 *  \param  startBlk    [IN] Block to start reading data from.
 *  \param  numBlks     [IN] Number of blocks to read.
 *
 *  \return #MMCSD_STS_SUCCESS on successful execution of API; else error on failure.
 */
int32_t MMCSD_lld_read_MMC_Poll(MMCSDLLD_Handle handle, uint8_t *buf,
                                uint32_t startBlk, uint32_t numBlks);

/**
 *  \brief  Function to perform block writes to the SD media in Interrupt Mode.
 *
 *  \param  handle      [IN] Handle to the MMCSD instance used.
 *  \param  buf         [IN] Pointer to buffer from which data is to be written from.
 *  \param  startBlk    [IN] Block to start Writing data from.
 *  \param  numBlks     [IN] Number of blocks to write.
 *
 *  \return #MMCSD_STS_SUCCESS on successful execution of API; else error on failure.
 */
int32_t MMCSD_lld_write_SD_Intr(MMCSDLLD_Handle handle, uint8_t *buf,
                                uint32_t startBlk, uint32_t numBlks);

/**
 *  \brief  Function to perform block reads from the SD media in Interrupt Mode.
 *
 *  \param  handle      [IN] Handle to the MMCSD instance used.
 *  \param  buf         [IN] Pointer to buffer to which data is to be read into.
 *  \param  startBlk    [IN] Block to start reading data from.
 *  \param  numBlks     [IN] Number of blocks to read.
 *
 *  \return #MMCSD_STS_SUCCESS on successful execution of API; else error on failure.
 */
int32_t MMCSD_lld_read_SD_Intr(MMCSDLLD_Handle handle, uint8_t *buf,
                               uint32_t startBlk, uint32_t numBlks);

/**
 *  \brief  Function to perform block writes to the MMC media in Interrupt Mode.
 *
 *  \param  handle      [IN] Handle to the MMCSD instance used.
 *  \param  buf         [IN] Pointer to buffer from which data is to be written from.
 *  \param  startBlk    [IN] Block to start Writing data from.
 *  \param  numBlks     [IN] Number of blocks to write.
 *
 *  \return #MMCSD_STS_SUCCESS on successful execution of API; else error on failure.
 */
int32_t MMCSD_lld_write_MMC_Intr(MMCSDLLD_Handle handle, uint8_t *buf,
                                 uint32_t startBlk, uint32_t numBlks);

/**
 *  \brief  Function to perform block reads from the MMC media in Interrupt Mode.
 *
 *  \param  handle      [IN] Handle to the MMCSD instance used.
 *  \param  buf         [IN] Pointer to buffer to which data is to be read into.
 *  \param  startBlk    [IN] Block to start reading data from.
 *  \param  numBlks     [IN] Number of blocks to read.
 *
 *  \return #MMCSD_STS_SUCCESS on successful execution of API; else error on failure.
 */
int32_t MMCSD_lld_read_MMC_Intr(MMCSDLLD_Handle handle, uint8_t *buf,
                                uint32_t startBlk, uint32_t numBlks);

/**
 *  \brief  Function to reconfigure the bus Speed and Bus width of the bus for MMC.
 *
 *  \param  handle      [IN] Handle to the MMCSD instance used.
 *  \param  busSpeed    [IN] Bus Speed to Configure. \ref MMCSDSpeedModesEMMC
 *  \param  busWidth    [IN] Bus Width to Configure. \ref MMCSDBusWidth
 *
 *  \return #MMCSD_STS_SUCCESS on successful execution of API; else error on failure.
 */
int32_t MMCSD_lld_change_Bus_Config_MMC(MMCSDLLD_Handle handle,
                                        uint32_t busSpeed, uint32_t busWidth);

/**
 *  \brief  Function to reconfigure the bus Speed and Bus width of the bus for MMC.
 *
 *  \param  handle      [IN] Handle to the MMCSD instance used.
 *  \param  busSpeed    [IN] Bus Speed to Configure. \ref MMCSDSpeedModesSD
 *
 *  \return #MMCSD_STS_SUCCESS on successful execution of API; else error on failure.
 */
int32_t MMCSD_lld_change_Bus_Config_SD(MMCSDLLD_Handle handle,
                                       uint32_t busSpeed);

/**
 *  \brief  Function to reconfigure the bus Speed and Bus width of the bus for MMC.
 *
 *  \param  handle      [IN] Handle to the MMCSD instance used.
 *  \param  tuningType  [IN] Tuning Type to Configure. \ref MMCSDPhyTuningType
 *
 *  \return #MMCSD_STS_SUCCESS on successful execution of API; else error on failure.
 */
int32_t MMCSD_lld_change_Tuning_Type(MMCSDLLD_Handle handle,
                                     uint32_t tuningType);

/**
 *  \brief  Function to enable the boot partition given the media is eMMC.
 *
 *  \pre    MMCSD controller has been initialized using #MMCSD_lld_init().
 *
 *  \param  handle          [IN] Handle to the MMCSD instance used.
 *  \param  partitionNum    [IN] Boot partition to be enabled.
 *
 *  \return #MMCSD_STS_SUCCESS on success; else error on failure.
 *
 *  \sa     #MMCSD_lld_init()
 */
int32_t MMCSD_lld_enableBootPartition(MMCSDLLD_Handle handle,
                                      uint32_t partitionNum);

/**
 *  \brief  This function disables the boot partition if the media is eMMC.
 *
 *  \pre    MMCSD controller has been initialized using #MMCSD_lld_init().
 *
 *  \param  handle         [IN] Handle to the MMCSD instance used.
 *
 *  \return #MMCSD_STS_SUCCESS on success; else error on failure.
 *
 *  \sa     #MMCSD_lld_init()
 */
int32_t MMCSD_lld_disableBootPartition(MMCSDLLD_Handle handle);

/* ========================================================================== */
/*                        ISR Function Declarations                           */
/* ========================================================================== */

/**
 *  \brief  This is the MMCSD Controller ISR and can be used as IRQ handler.
 *
 *  \param  args            [IN] Argument to the ISR.
 *
 */
void MMCSD_lld_Isr(void *args);

#ifdef __cplusplus
}
#endif

#endif /* MMCSD_LLD_H_ */

/** @} */