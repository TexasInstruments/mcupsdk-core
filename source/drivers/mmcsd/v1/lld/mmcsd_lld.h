/*
 * Copyright (C) 2024-2025 Texas Instruments Incorporated
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
 *  #include <drivers/MMCSD/v1/lld/mmcsd_lld.h>
 *  \endcode
 *
 *  @{
 */

/**
 *  \file v1/lld/mmcsd_lld.h
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
#include <drivers/edma/v0/edma.h>
#include <drivers/mmcsd/v1/cslr_mmcsd.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * \anchor   MMCSD_StatusCode
 * @name Return status
 */
/**@{*/

typedef EDMA_Handle MMCSD_DmaHandle;

typedef struct MMCSD_EdmaChConfig_s *MMCSD_DmaChConfig;

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
 *  wait forever until resource is available */
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
 *  \anchor MMCSDCardType
 *  \name MACROS used to select one of the possible Card Types.
 *  @{
 */

/** \brief  Card type SD */
#define MMCSD_CARD_TYPE_SD                      ((uint32_t) 0U)
/** \brief  Card type MMC */
#define MMCSD_CARD_TYPE_EMMC                    ((uint32_t) 2U)
/** \brief  No Card Installed */
#define MMCSD_CARD_TYPE_NO_DEVICE               ((uint32_t) 3U)

/** @} */

/**
 * \anchor   MMCSDLLD_BusWidth
 * \name MACROS for the possible values of Bus Width configuration.
 * @{
 */

/** \brief Card bus width configuration for 1-bit mode */
#define MMCSD_BUS_WIDTH_1BIT                    ((uint32_t) 1U)
/** \brief Card bus width configuration for 4-bit mode */
#define MMCSD_BUS_WIDTH_4BIT                    ((uint32_t) 4U)
/** \brief Card bus width configuration for 8-bit mode */
#define MMCSD_BUS_WIDTH_8BIT                    ((uint32_t) 8U)

/** @} */

/**
 * \anchor MMCSD_ECSD_BusWidth
 * @name ECSD Bus Width Macros
 * @brief Macros for Extended CSD (ECSD) bus width configuration in eMMC devices.
 * These macros are used to set or interpret the bus width field in the ECSD register.
 * @{*/

/** \brief ECSD register index for bus width configuration */
#define MMCSD_ECSD_BUS_WIDTH_INDEX           (183U)
/** \brief ECSD value for 1-bit bus width */
#define MMCSD_ECSD_BUS_WIDTH_1BIT            (0U)
/** \brief ECSD value for 4-bit bus width */
#define MMCSD_ECSD_BUS_WIDTH_4BIT            (1U)
/** \brief ECSD value for 8-bit bus width */
#define MMCSD_ECSD_BUS_WIDTH_8BIT            (2U)
/** \brief ECSD value for 4-bit DDR bus width */
#define MMCSD_ECSD_BUS_WIDTH_4BIT_DDR        (5U)
/** \brief ECSD value for 8-bit DDR bus width */
#define MMCSD_ECSD_BUS_WIDTH_8BIT_DDR        (6U)

/** \brief Mask for extracting bus width bits from ECSD value */
#define MMCSD_ECSD_BUS_WIDTH_BUSWIDTH_MASK   (0x0FU)
/** \brief Bit shift for bus width bits in ECSD value */
#define MMCSD_ECSD_BUS_WIDTH_BUSWIDTH_SHIFT  (0U)

/** \brief Enable Enhanced Strobe in ECSD bus width field */
#define MMCSD_ECSD_BUS_WIDTH_ES_ENABLE       (0x80U)
/** \brief Mask for Enhanced Strobe bit in ECSD bus width field */
#define MMCSD_ECSD_BUS_WIDTH_ES_MASK         (0x80U)
/** \brief Bit shift for Enhanced Strobe bit in ECSD bus width field */
#define MMCSD_ECSD_BUS_WIDTH_ES_SHIFT        (7U)

/** @} */

/**
 *  \anchor MMCSDSpeedModesSD
 *  \name MACROS used to select one of the possible Speed Modes for SD Device.
 *  @{
 */

/** \brief Default Speed */
#define MMCSD_SD_MODE_DS                        ((uint32_t) 10U)
/** \brief High Speed */
#define MMCSD_SD_MODE_HS                        ((uint32_t) 11U)

/** @} */

/* ========================================================================== */
/*                      Function Pointers Declarations                        */
/* ========================================================================== */

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
/** eMMC specific data */
    uint8_t                 extCsd[512];
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
 *  \brief  MMCSD LLD Transaction
 *
 *  This structure defines the nature of a MMCSD transaction. This structure
 *  specifies the buffer and buffer's size that is to be written to or read from
 *  the MMC peripheral.
 */
typedef struct {

/** Command register content composed of CMD ID, DP, TYPE, RESP TYPE etc */
    uint32_t                cmd;
/** Command flag as per MMC device specification */
    uint32_t                flags;
/** Command argument as per MMC device specification */
    uint32_t                arg;
/** Buffer containing data to be read into or written from */
    uint8_t                 *dataBuf;
/** Number of bytes to be transferred per block */
    uint32_t                blockSize;
/** Number of block to be transferred */
    uint32_t                blockCount;
/** Command response per MMC device specification */
    uint32_t                response[4];

} MMCSDLLD_Transaction;

/**
 *  \brief  MMCSD Driver Initialization Object
 */
typedef struct {

/** MMCSD Host control registers base address */
    uint32_t                baseAddr;
/** Module input clock frequency */
    uint32_t                inputClkFreq;
/** Module interrupt vector */
    uint32_t                intrNum;
/** Type of card \ref MMCSDCardType */
    uint32_t                cardType;
/** Auto Assign Maximum Speed Flag */
    bool                    autoAssignMaxSpeed;
/** User Assigned Bus Speed */
    uint32_t                uaBusSpeed;
/** Supported bus width \ref MMCSDLLD_BusWidth */
    uint32_t                busWidth;
/** DMA enable */
    bool                    enableDma;
/** Pointer to device Data Structure, allocated and assigned by Syscfg */    
    void                    *deviceData; 
/** DMA Handle */
    MMCSD_DmaHandle         mmcsdDmaHandle;
/** DMA Configuration for this instance. */
    MMCSD_DmaChConfig       mmcsdDmaChConfig;
/** Pointer to a 512 byte dataBuffer used for temporary data
 * transactions internal to driver like ECSD read, tuning etc.
 * This data is allocated by syscfg */
    uint8_t                 *dataBuf;

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
 * \ref MMCSDSpeedModesSD */
    uint32_t                setBusSpeed;
/** Set bus width \ref MMCSDLLD_BusWidth */
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
 *  \brief This API Initializes the MMCSD instance for DMA transfers.
 *
 *  \param  handle      [IN] Handle to the MMCSD instance used.
 *
 *  \return \ref MMCSD_StatusCode
 */
int32_t MMCSD_lld_InitDma(MMCSDLLD_Handle handle);

/**
 *  \brief This API De-Initializes the MMCSD instance.
 *
 *  \param  handle      [IN] Handle to the MMCSD instance used.
 *
 *  \return \ref MMCSD_StatusCode
 */
int32_t MMCSD_lld_deInit(MMCSDLLD_Handle handle);

/**
 *  \brief This API De-Initializes the MMCSD instance with DMA transfers.
 *
 *  \param  handle      [IN] Handle to the MMCSD instance used.
 *
 *  \return \ref MMCSD_StatusCode
 */
int32_t MMCSD_lld_deInitDma(MMCSDLLD_Handle handle);

/**
 *  \brief  Function to perform block writes to the SD media in Polling Mode.
 *
 *  \param  handle      [IN] Handle to the MMCSD instance used.
 *  \param  buf         [IN] Pointer to buffer from which data is to be written from.
 *  \param  startBlk    [IN] Block to start Writing data from.
 *  \param  numBlks     [IN] Number of blocks to write.
 *
 *  \return \ref MMCSD_StatusCode
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
 *  \return \ref MMCSD_StatusCode
 */
int32_t MMCSD_lld_read_SD_Poll(MMCSDLLD_Handle handle, uint8_t *buf,
                               uint32_t startBlk, uint32_t numBlks);

/**
 *  \brief  Function to perform block writes to the SD media in Interrupt Mode.
 *
 *  \param  handle      [IN] Handle to the MMCSD instance used.
 *  \param  buf         [IN] Pointer to buffer from which data is to be written from.
 *  \param  startBlk    [IN] Block to start Writing data from.
 *  \param  numBlks     [IN] Number of blocks to write.
 *
 *  \return \ref MMCSD_StatusCode
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
 *  \return \ref MMCSD_StatusCode
 */
int32_t MMCSD_lld_read_SD_Intr(MMCSDLLD_Handle handle, uint8_t *buf,
                               uint32_t startBlk, uint32_t numBlks);

/**
 *  \brief  Function to perform block writes to the SD media in DMA Mode.
 *
 *  \param  handle      [IN] Handle to the MMCSD instance used.
 *  \param  buf         [IN] Pointer to buffer from which data is to be written from.
 *  \param  startBlk    [IN] Block to start Writing data from.
 *  \param  numBlks     [IN] Number of blocks to write.
 *
 *  \return \ref MMCSD_StatusCode
 */                              
int32_t MMCSD_lld_write_SD_Dma(MMCSDLLD_Handle handle, uint8_t *buf,
                                uint32_t startBlk, uint32_t numBlks);

/**
 *  \brief  Function to perform block reads from the SD media in DMA Mode.
 *
 *  \param  handle      [IN] Handle to the MMCSD instance used.
 *  \param  buf         [IN] Pointer to buffer to which data is to be read into.
 *  \param  startBlk    [IN] Block to start reading data from.
 *  \param  numBlks     [IN] Number of blocks to read.
 *
 *  \return \ref MMCSD_StatusCode
 */
int32_t MMCSD_lld_read_SD_Dma(MMCSDLLD_Handle handle, uint8_t *buf,
                                uint32_t startBlk, uint32_t numBlks);


/**
 * @brief Writes data to the MMC card using polling method.
 *
 * This function writes data to the specified blocks on the MMC card using
 * a polling-based approach.
 *
 * @param handle    Handle to the MMCSD driver instance.
 * @param buf       Pointer to the buffer containing the data to be written.
 * @param startBlk  Starting block number where the data will be written.
 * @param numBlks   Number of blocks to write.
 *
 * @return int32_t  Returns 0 on success, or an error code on failure.
 */
int32_t MMCSD_lld_write_MMC_Poll(MMCSDLLD_Handle handle, uint8_t *buf,
                                uint32_t startBlk, uint32_t numBlks);

/**
 * @brief Reads data from the MMC card using polling method.
 *
 * This function reads data from the specified blocks on the MMC card using
 * a polling-based approach.
 *
 * @param handle    Handle to the MMCSD driver instance.
 * @param buf       Pointer to the buffer where the read data will be stored.
 * @param startBlk  Starting block number from where the data will be read.
 * @param numBlks   Number of blocks to read.
 *
 * @return int32_t  Returns 0 on success, or an error code on failure.
 */
int32_t MMCSD_lld_read_MMC_Poll(MMCSDLLD_Handle handle, uint8_t *buf,
                               uint32_t startBlk, uint32_t numBlks);

/**
 * @brief Writes data to the MMC card using interrupt method.
 *
 * This function writes data to the specified blocks on the MMC card using
 * an interrupt-based approach.
 *
 * @param handle    Handle to the MMCSD driver instance.
 * @param buf       Pointer to the buffer containing the data to be written.
 * @param startBlk  Starting block number where the data will be written.
 * @param numBlks   Number of blocks to write.
 *
 * @return int32_t  Returns 0 on success, or an error code on failure.
 */
int32_t MMCSD_lld_write_MMC_Intr(MMCSDLLD_Handle handle, uint8_t *buf,
                                uint32_t startBlk, uint32_t numBlks);

/**
 * @brief Reads data from the MMC card using interrupt method.
 *
 * This function reads data from the specified blocks on the MMC card using
 * an interrupt-based approach.
 *
 * @param handle    Handle to the MMCSD driver instance.
 * @param buf       Pointer to the buffer where the read data will be stored.
 * @param startBlk  Starting block number from where the data will be read.
 * @param numBlks   Number of blocks to read.
 *
 * @return int32_t  Returns 0 on success, or an error code on failure.
 */
int32_t MMCSD_lld_read_MMC_Intr(MMCSDLLD_Handle handle, uint8_t *buf,
                               uint32_t startBlk, uint32_t numBlks);

/**
 *  \brief  Function to perform block writes to the MMC media in DMA Mode.
 *
 *  \param  handle      [IN] Handle to the MMCSD instance used.
 *  \param  buf         [IN] Pointer to buffer from which data is to be written from.
 *  \param  startBlk    [IN] Block to start Writing data from.
 *  \param  numBlks     [IN] Number of blocks to write.
 *
 *  \return \ref MMCSD_StatusCode
 */                              
int32_t MMCSD_lld_write_MMC_Dma(MMCSDLLD_Handle handle, uint8_t *buf,
    uint32_t startBlk, uint32_t numBlks);

/**
*  \brief  Function to perform block reads from the MMC media in DMA Mode.
*
*  \param  handle      [IN] Handle to the MMCSD instance used.
*  \param  buf         [IN] Pointer to buffer to which data is to be read into.
*  \param  startBlk    [IN] Block to start reading data from.
*  \param  numBlks     [IN] Number of blocks to read.
*
*  \return \ref MMCSD_StatusCode
*/
int32_t MMCSD_lld_read_MMC_Dma(MMCSDLLD_Handle handle, uint8_t *buf,
    uint32_t startBlk, uint32_t numBlks);

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
 * @brief Completes the current MMC/SD data transfer operation.
 *
 * This function should be called when a data transfer (read or write)
 * operation is finished, either successfully or with an error. It handles
 * any necessary cleanup and notifies the higher layers of the transfer
 * completion status.
 *
 * @param handle      Handle to the MMCSD LLD (Low Level Driver) instance.
 * @param xferStatus  Status of the completed transfer. Typically, a value of
 *                    0 indicates success, while a negative value indicates
 *                    an error.
 */
void MMCSD_lld_completeCurrTransfer(MMCSDLLD_Handle handle,int32_t xferStatus);

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