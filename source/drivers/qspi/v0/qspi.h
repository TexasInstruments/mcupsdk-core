/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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
 *  \defgroup DRV_QSPI_MODULE APIs for QSPI
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the QSPI module. The APIs
 *  can be used by other drivers to get access to QSPI and also by
 *  application to initiate data transfer operation.
 *
 *  @{
 */

/**
 *  \file v0/qspi.h
 *
 *  \brief QSPI Driver API/interface file.
 */

#ifndef QSPI_H_
#define QSPI_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/hw_include/cslr_qspi.h>
#include <drivers/edma.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief A handle that is returned from a #QSPI_open() call */
typedef void *QSPI_Handle;

/**
*  \anchor QSPI_ChipSelect
*  \name Chip Selects
*
*  Chip selects
*
*  @{
*/
#define QSPI_CS0  (0U)
#define QSPI_CS1  (1U)
#define QSPI_CS2  (2U)
#define QSPI_CS3  (3U)
/** @} */

/**
*  \anchor QSPI_CmdMacros
*  \name Macros for invalid commands
*
*  Macros for invalid commands
*
*  @{
*/
#define QSPI_CMD_INVALID_OPCODE  (0xFFU)
#define QSPI_CMD_INVALID_ADDR    (0xFFFFFFFFU)
/** @} */

/**
 *  \anchor QSPI_TransferStatus
 *  \name Transfer Status Code
 *
 *  Status codes that are set by the QSPI driver
 *
 *  @{
 */
#define QSPI_TRANSFER_COMPLETED        (0U)
#define QSPI_TRANSFER_STARTED          (1U)
#define QSPI_TRANSFER_CANCELLED        (2U)
#define QSPI_TRANSFER_FAILED           (3U)
#define QSPI_TRANSFER_CSN_DEASSERT     (4U)
#define QSPI_TRANSFER_TIMEOUT          (5U)
/** @} */

/**
*  \anchor QSPI_TransferLines
*  \name Transfer Lines Number
*
*  Number of lines used for QSPI read transaction
*
*  @{
*/
#define QSPI_RX_LINES_SINGLE    (0U)
#define QSPI_RX_LINES_DUAL      (1U)
#define QSPI_RX_LINES_QUAD      (2U)
/** @} */

/**
 *  \anchor QSPI_FrameFormat
 *  \name Frame Format
 *
 *  Definitions for various SPI data frame formats
 *
 *  POL0 = QSPICLK is held low during the INACTIVE state
 *  POL1 = QSPICLK is held high during the INACTIVE state
 *
 *  PHA0 = Data launch is on the falling edge of QSPICLK
 *  PHA1 = Data launch is on the rising edge of QSPICLK
 *
 *  @{
 */
#define QSPI_FF_POL0_PHA0   ((CSL_QSPI_SPI_DC_REG_CKPH0_CKP_0_SHIFT_OUT_FALLING_EDGE \
                            << CSL_QSPI_SPI_DC_REG_CKPH0_SHIFT) |                    \
                            (CSL_QSPI_SPI_DC_REG_CKP0_DATA_INACTIVE <<               \
                            CSL_QSPI_SPI_DC_REG_CKP0_SHIFT))
#define QSPI_FF_POL0_PHA1   ((CSL_QSPI_SPI_DC_REG_CKPH0_CKP_0_SHIFT_OUT_RISING_EDGE  \
                            << CSL_QSPI_SPI_DC_REG_CKPH0_SHIFT) |                    \
                            (CSL_QSPI_SPI_DC_REG_CKP0_DATA_INACTIVE <<               \
                            CSL_QSPI_SPI_DC_REG_CKP0_SHIFT))
#define QSPI_FF_POL1_PHA0   ((CSL_QSPI_SPI_DC_REG_CKPH0_CKP_1_SHIFT_OUT_RISING_EDGE  \
                            << CSL_QSPI_SPI_DC_REG_CKPH0_SHIFT) |                    \
                            (CSL_QSPI_SPI_DC_REG_CKP0_DATA_ACTIVE <<                 \
                            CSL_QSPI_SPI_DC_REG_CKP0_SHIFT))
#define QSPI_FF_POL1_PHA1   ((CSL_QSPI_SPI_DC_REG_CKPH0_CKP_1_SHIFT_OUT_FALLING_EDGE \
                            << CSL_QSPI_SPI_DC_REG_CKPH0_SHIFT) |                    \
                            (CSL_QSPI_SPI_DC_REG_CKP0_DATA_ACTIVE <<                 \
                            CSL_QSPI_SPI_DC_REG_CKP0_SHIFT))
/** @} */

/**
*  \anchor QSPI_ChipSelectPolarity
*  \name Chip select polarity
*
*  Polarity of Chip Select
*
*  @{
*/
#define QSPI_CS_POL_ACTIVE_LOW    (CSL_QSPI_SPI_DC_REG_CSP0_ACTIVE_LOW)
#define QSPI_CS_POL_ACTIVE_HIGH   (CSL_QSPI_SPI_DC_REG_CSP0_ACTIVE_HIGH)
/** @} */

/**
*  \anchor QSPI_DataDelay
*  \name Data Delay
*
*  Value of delay in data output after CS goes active.
*
*  @{
*/
#define QSPI_DATA_DELAY_0   (CSL_QSPI_SPI_DC_REG_DD0_CS_TO_DATA_DELAY_0)
#define QSPI_DATA_DELAY_1   (CSL_QSPI_SPI_DC_REG_DD0_CS_TO_DATA_DELAY_1)
#define QSPI_DATA_DELAY_2   (CSL_QSPI_SPI_DC_REG_DD0_CS_TO_DATA_DELAY_2)
#define QSPI_DATA_DELAY_3   (CSL_QSPI_SPI_DC_REG_DD0_CS_TO_DATA_DELAY_3)
/** @} */

/* ========================================================================== */
/*                             Structure Definitions                          */
/* ========================================================================== */

/**
 *  \brief Data structure used with \ref QSPI_Transfers - #QSPI_writeConfigMode,
 *  #QSPI_readMemMapMode
 *
 *  It indicates how many \ref QSPI_FrameFormat frames are sent and received
 *  from and to the buffer buf.
 */
typedef struct
{
    uint32_t                count;
    /**< [IN] Number of frames for this transaction */
    void                   *buf;
    /**< [IN] void * to a buffer to receive/send data */
    uint32_t                addrOffset;
    /**< [IN] Address offset to write to an QSPI flash device. */
    uint32_t                status;
    /**< [OUT] \ref QSPI_TransferStatus code set by \ref QSPI_Transfers */
    uint32_t                transferTimeout;
    /**< [IN] Timeout of the specific transfer */
} QSPI_Transaction;

typedef struct
{
    uint8_t cmd;
    /**< [IN] Command Opcode */
    uint32_t cmdAddr;
    /**< Address required by the command. Usually needed in reading flash registers.
    Should be initialized to #QSPI_CMD_INVALID_ADDR if not used. */
    uint8_t numAddrBytes;
    /**< [IN] Number of address bytes used to send cmd address */
    void *rxDataBuf;
    /**< [OUT] Buffer to store response from flash */
    uint32_t rxDataLen;
    /**< [IN] Length of response buffer */
} QSPI_ReadCmdParams;

typedef struct
{
    uint8_t cmd;
    /**< [IN] Command Opcode */
    uint32_t cmdAddr;
    /**< [IN] Address required by the command. Usually needed in writing to flash registers.
    Should be initialized to #QSPI_CMD_INVALID_ADDR if not used. */
    uint8_t numAddrBytes;
    /**< [IN] Number of address bytes used to send cmd address */
    void *txDataBuf;
    /**< [IN] Buffer containing command data */
    uint32_t txDataLen;
    /**< [IN] Length of response buffer */
} QSPI_WriteCmdParams;

/**
 *  \brief QSPI Parameters
 *
 *  QSPI Parameters are used to with the #QSPI_open() call. Default values for
 *  these parameters are set using #QSPI_Params_init().
 *
 *  If NULL is passed for the parameters, #QSPI_open() uses default parameters.
 *
 *  \sa #QSPI_Params_init()
 */
typedef struct
{
    uint32_t edmaInst;
    /**< EDMA instance used for QSPI transfer */
} QSPI_Params;

/**
 *  \brief QSPI EDMA Parameters
 *
 *  Used to store the EDMA parameters allocated for QSPI transfer.
 *
 */
typedef struct
{
    uint32_t edmaTcc;
    /**< EDMA TCC used for QSPI transfer */
    uint32_t edmaChId;
    /**< EDMA Channel used for QSPI transfer */
    uint32_t edmaParam;
    /**< EDMA Param ID used for QSPI transfer */
    uint32_t edmaRegionId;
    /**< EDMA Region used for QSPI transfer */
    uint32_t edmaBaseAddr;
    /**< EDMA Base address used for QSPI transfer */
    uint32_t isIntEnabled;
    /**< EDMA Interrupt enabled status */
    Edma_IntrObject edmaIntrObj;
    /**< EDMA Interrupt object */
    SemaphoreP_Object gEdmaTransferDoneSem;
    /**< EDMA transfer done Semaphore */
} QSPI_EdmaParams;

/**
 *  \brief QSPI driver object
 */
typedef struct
{
    /*
     * User params
     */
    QSPI_Handle     handle;
    /**< Instance handle */
    uint32_t        transferMode;
    /**< Polling, Blocking or Callback mode. */
    uint32_t        rxLines;
    /**< Number of lines used for QSPI reading */
    uint8_t         readCmd;
    /**< Transfer command to be used for reading from QSPI flash */
    uint8_t         writeCmd;
    /**< Transfer command to be used for writing to QSPI flash */
    uint32_t        frmLength;
    /**< Frame length of total transfer */
    uint32_t        numAddrBytes;
    /**< Number of bytes used to represent address to be sent to flash. */
    uint32_t        numDummyBits;
    /**< Number of dummy bits required while reading from flash */
    QSPI_EdmaParams qspiEdmaParams;
    /**< EDMA parameters allocated for QSPI */
    void*           qspiEdmaHandle;
    /**< EDMA handle allocated for QSPI */

    /*
     * State variables
     */
    uint32_t            isOpen;
    /**< Flag to indicate if the instance is already open */
    SemaphoreP_Object   lockObj;
    /**< Driver lock object */
    SemaphoreP_Object   transferSemObj;
    /**< Transfer Sync Semaphore object */
    HwiP_Object         hwiObj;
    /**< Interrupt object */

    QSPI_Transaction    *transaction;
    /**< Pointer to current transaction struct */
} QSPI_Object;


/** \brief QSPI instance attributes - used during init time */
typedef struct
{
    /*
     * SOC configuration
     */
    uintptr_t   baseAddr;
    /**< Peripheral base address */
    uintptr_t   memMapBaseAddr;
    /**< Memory mapped mode base address of QSPI flash */
    uint32_t    inputClkFreq;
    /**< Module input clock frequency */
    uint32_t    baudRateDiv;
    /**< Module clock divider */
    uint32_t    chipSelect;
    /**< Qspi Chip select number */
    uint32_t    csPol;
    /**< Qspi Chip select polarity */
    uint32_t    frmFmt;
    /**< Qspi Frame format */
    uint32_t    dataDelay;
    /**< QSPI data delay */
    uint32_t    rxLines;
    /**< Number of rx Lines used for QSPI reading */
    uint32_t    wrdLen;
    /**< Number of bits in a word */
    uint32_t    intrNum;
    /**< Peripheral interrupt number */
    uint32_t    intrEnable;
    /**< Enable interrupt mode */
    uint8_t     intrPriority;
    /**< Interrupt priority */
    uint32_t    dmaEnable;
    /**< Enable DMA mode */

} QSPI_Attrs;

typedef struct
{
    const QSPI_Attrs *attrs;
    /**< Pointer to driver specific hardware attributes */
    QSPI_Object *object;
    /**< Pointer to driver specific data object */
} QSPI_Config;

/** \brief Externally defined driver configuration array */
extern QSPI_Config gQspiConfig[];
/** \brief Externally defined driver configuration array size */
extern uint32_t    gQspiConfigNum;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief  This function initializes the QSPI module
 */
void QSPI_init(void);

/**
 *  \brief  This function de-initializes the QSPI module
 */
void QSPI_deinit(void);

/**
 *  \brief  Initialize data structure with defaults
 *
 *  \param  qspiParams [out] Initialized parameters
 */
void QSPI_Params_init( QSPI_Params *qspiParams);

/**
 *  \brief  This function opens a given QSPI peripheral
 *
 *  \pre    QSPI controller has been initialized using #QSPI_init()
 *
 *  \param  index       Index of config to use in the *QSPI_Config* array
 *  \param  openParams  Pointer to parameters to open the driver with
 *
 *  \return A #QSPI_Handle on success or a NULL on an error or if it has been
 *          opened already
 *
 *  \sa     #QSPI_init()
 *  \sa     #QSPI_close()
 */
QSPI_Handle QSPI_open(uint32_t index, const QSPI_Params *openParams);

/**
 *  \brief  Function to close a QSPI peripheral specified by the QSPI handle
 *
 *  \pre    #QSPI_open() has to be called first
 *
 *  \param  handle      #QSPI_Handle returned from #QSPI_open()
 *
 *  \sa     #QSPI_open()
 */
void QSPI_close(QSPI_Handle handle);

/**
 *  \brief  This function returns the handle of an open QSPI Instance from the instance index
 *
 *  \pre    QSPI controller has been opened using #QSPI_open()
 *
 *  \param  index Index of config to use in the *QSPI_Config* array
 *
 *  \return A #QSPI_Handle if it has been opened already or NULL otherwise
 *
 *  \sa     #QSPI_init()
 *  \sa     #QSPI_open()
 */
QSPI_Handle QSPI_getHandle(uint32_t index);

/**
 *  \brief  This function returns the input clock at which QSPI was programmed
 *
 *  \pre    QSPI controller has been opened using #QSPI_open()
 *
 *  \param  handle #QSPI_Handle returned from #QSPI_open()
 *
 *  \return Clock frequency of the QSPI peripheral in Hertz
 *
 *  \sa     #QSPI_init()
 *  \sa     #QSPI_open()
 */
uint32_t QSPI_getInputClk(QSPI_Handle handle);

/**
 *  \anchor QSPI_Transfers
 *  \name Different QSPI Transfer functions
 *
 *  @{
 */
/**
 *  \brief  Function to perform reads from the flash in memory mapped
 *          mode.
 *
 *  \param  handle      #QSPI_Handle returned from #QSPI_open()
 *  \param  trans       Pointer to a #QSPI_Transaction
 *
 *  \return #SystemP_SUCCESS on successful read; else error on failure
 *
 *  \sa     #QSPI_open
 */
int32_t QSPI_readMemMapMode(QSPI_Handle handle, QSPI_Transaction *trans);

/**
 *  \brief  Function to perform writes to the flash in configuration mode.
 *
 *  \param  handle      #QSPI_Handle returned from #QSPI_open()
 *  \param  trans       Pointer to a #QSPI_Transaction
 *
 *  \return #SystemP_SUCCESS on successful write; else error on failure
 *
 *  \sa     #QSPI_open
 */
int32_t QSPI_writeConfigMode(QSPI_Handle handle, QSPI_Transaction *trans);
/** @} */

/**
 *  \brief  Function to initialize the #QSPI_Transaction structure
 *
 *
 *  \param  trans Pointer to a #QSPI_Transaction
 *
 */
void QSPI_transaction_init(QSPI_Transaction *trans);

/**
 *  \brief  Function to initialize the #QSPI_Transaction structure
 *
 *
 *  \param  rdParams Pointer to a #QSPI_ReadCmdParams
 *
 */
void QSPI_readCmdParams_init(QSPI_ReadCmdParams *rdParams);

/**
 *  \brief  Function to initialize the #QSPI_Transaction structure
 *
 *
 *  \param  wrParams Pointer to a #QSPI_WriteCmdParams
 *
 */
void QSPI_writeCmdParams_init(QSPI_WriteCmdParams *wrParams);

/**
 *  \brief  Function to send specific commands and receive related data from flash
 *
 *  \param  handle      #QSPI_Handle returned from #QSPI_open()
 *  \param  rdParams    Pointer to a #QSPI_ReadCmdParams
 *
 *  \return #SystemP_SUCCESS if command read was successful; else error on failure
 *
 *  \sa     #QSPI_open
 */
int32_t QSPI_readCmd(QSPI_Handle handle, QSPI_ReadCmdParams *rdParams);

/**
 *  \brief  Function to send specific commands and related data to flash
 *
 *  \param  handle      #QSPI_Handle returned from #QSPI_open()
 *  \param  wrParams    Pointer to a #QSPI_WriteCmdParams
 *
 *  \return #SystemP_SUCCESS if command write was successful; else error on failure
 *
 *  \sa     #QSPI_open
 */
int32_t QSPI_writeCmd(QSPI_Handle handle, QSPI_WriteCmdParams *wrParams);

/**
 *  \brief  Function to set write command to be used.
 *
 *  \param  handle      #QSPI_Handle returned from #QSPI_open()
 *  \param  command     Write command for a particular flash
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t QSPI_setWriteCmd(QSPI_Handle handle, uint8_t command);

/**
 *  \brief  Function to set read command to be used.
 *
 *  \param  handle      #QSPI_Handle returned from #QSPI_open()
 *  \param  command     Read command for a particular flash
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t QSPI_setReadCmd(QSPI_Handle handle, uint8_t command);

/**
 *  \brief  Function to set number of address bytes to be used.
 *
 *  \param  handle      #QSPI_Handle returned from #QSPI_open()
 *  \param  count       Number of Address Bytes. This function
 *                      should always be called before accessing
 *                      a memory location that doesn't fit in the
 *                      current address byte range.
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t QSPI_setAddressByteCount(QSPI_Handle handle, uint32_t count);

/**
 *  \brief  Function to set number of dummy bits to be used.
 *
 *  \param  handle      #QSPI_Handle returned from #QSPI_open()
 *  \param  count       Number of Dummy Bits. This should be either
 *                      less than 8 or a multiple of 8, because Bit
 *                      count field in setup register works only if
 *                      Byte count is 0.
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t QSPI_setDummyBitCount(QSPI_Handle handle, uint32_t count);

/**
 *  \brief  This function is used to switch between memory mapped and
 *          configuration mode.
 *
 *  \param   handle     A #QSPI_Handle returned from a #QSPI_open()
 *  \param   memMappedPortSwitch    Flag for switching mode.
 *
 *  \return  #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t QSPI_setMemAddrSpace(QSPI_Handle handle, uint32_t memMappedPortSwitch);

/**
 *  \brief  This function is used to enable word or frame complete interrupt.
 *
 *  \param   handle     A #QSPI_Handle returned from a #QSPI_open()
 *  \param   intFlag    Flag for enabling interrupt.
 *
 *  \return  #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t QSPI_intEnable(QSPI_Handle handle, uint32_t intFlag);

/**
 *  \brief  This function is used to disable word or frame complete interrupt.
 *
 *  \param   handle     A #QSPI_Handle returned from a #QSPI_open()
 *  \param   intFlag    Flag for disabling interrupt.
 *
 *  \return  #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t QSPI_intDisable(QSPI_Handle handle, uint32_t intFlag);

/**
 *  \brief  This function is used to clear word or frame complete interrupt.
 *
 *  \param   handle     A #QSPI_Handle returned from a #QSPI_open()
 *  \param   intFlag    Flag for clearing interrupt.
 *
 *  \return  #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t QSPI_intClear(QSPI_Handle handle, uint32_t intFlag);

/**
 *  \brief   Set the QSPI clock register divider value.
 *
 *  \details This function sets the QSPI clock control register
 *           with serial data clock divider ratio (DCLK_DIV)
 *           according to the input clock provided and the output clock
 *           required.
 *           DCLK_DIV = ((input clock) / (output clock)) - 1.
 *           This function also enables the clock for QSPI module.
 *           This can only be done if QSPI module is not busy.
 *
 *  \param   handle           A #QSPI_Handle returned from a #QSPI_open()
 *
 *  \param   clkDividerVal    Clock divider value to be set.
 *
 *  \return  #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t QSPI_setPreScaler(QSPI_Handle handle, uint32_t clkDividerVal);

/**
 *  \brief   Set QSPI Rx lines in the QSPI object
 *
 *  \details This function sets the QSPI RX lines in QSPI object so that it
 *           can be used in a subsequent read
 *
 *  \param   handle           A #QSPI_Handle returned from a #QSPI_open()
 *
 *  \param   rxLines    \ref QSPI_TransferLines value
 *
 *  \return  #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t QSPI_setRxLines(QSPI_Handle handle, uint32_t rxLines);

/**
 *  \brief   Get QSPI Rx lines in the QSPI object
 *
 *  \details This function returns the QSPI RX lines in QSPI object
 *
 *  \param   handle           A #QSPI_Handle returned from a #QSPI_open()
 *
 *  \return  #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
uint32_t QSPI_getRxLines(QSPI_Handle handle);

void OSPI_phyGetTuningData(uint32_t *phyTuningData, uint32_t *phyTuningDataSize);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef QSPI_H_ */

/** @} */

