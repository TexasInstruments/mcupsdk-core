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
 *  \defgroup DRV_QSPI_LLD_MODULE APIs for QSPI LLD
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the QSPI LLD module. The APIs
 *  can be used by other drivers to get access to QSPI and also by
 *  application to initiate data transfer operation.
 *
 *  @{
 */

/**
 *  \file v0/lld/qspi_lld.h
 *
 *  \brief QSPI LLD Driver API/interface file.
 */

#ifndef QSPI_LLD_H_
#define QSPI_LLD_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/hw_include/cslr_qspi.h>
#include <drivers/edma.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief The handle for DMA instance used with QSPI */
typedef EDMA_Handle QSPI_DmaHandle;

/** \brief A handle that holds DMA configuration parameters for QSPI */
typedef struct QSPI_EdmaParams_s *QSPI_DmaChConfig;

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
 * \brief NOT_IN_USE macro to highlight unused parameters
 */
#define QSPI_NOT_IN_USE(x) (void) 0

/**
*  \brief Macros for invalid commands
*
*  @{
*/
#define QSPI_LLD_CMD_INVALID_ADDR    (0xFFFFFFFFU)
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

/**
*  \anchor QSPI_Mode
*  \name Operation Mode
*
*  Operation Mode for read and write.
*
*  @{
*/
/** \brief    QSPI Operation mode- Configuration or memory mapped mode */
#define QSPI_MEM_MAP_PORT_SEL_CFG_PORT          \
                                (CSL_QSPI_SPI_SWITCH_REG_MMPT_S_SEL_CFG_PORT)
#define QSPI_MEM_MAP_PORT_SEL_MEM_MAP_PORT      \
                                (CSL_QSPI_SPI_SWITCH_REG_MMPT_S_SEL_MM_PORT)
/** @} */

/** \brief Maximum QSPI Word length */
#define QSPI_MAX_WORD_LENGTH     (128U)

/** @name Return status
 */
/**@{*/
/**
 * \brief Return status when the API execution was successful
 */
#define QSPI_SYSTEM_SUCCESS     ((int32_t )0)

/**
 * \brief Return status when the API execution was not successful due to a failure
 */
#define QSPI_SYSTEM_FAILURE     ((int32_t)-1)

/**
 * \brief Return status when the API execution was not successful due to a time out
 */
#define QSPI_TIMEOUT            ((int32_t)-2)

/**
 * \brief Return status when the API execution failed due invalid parameters
 */
#define QSPI_LLD_INVALID_PARAM  ((int32_t)-3)

/**
 * \brief Return status when the API execution failed due to driver busy
 */
#define QSPI_BUSY               ((int32_t)-4)

/**
 * \brief Return status when the API execution failed due to invalid LLD state
 */
#define QSPI_INVALID_STATE      ((int32_t)-5)

/**@}*/

/** @name LLD (Low Level Driver) states
 */
/**@{*/
/**
 * \brief LLD is in Reset State prior to driver init and post driver deinit
 */
#define QSPI_STATE_RESET        (0U)

/**
 * \brief LLD accepts runtime APIs only Ready State, otherwise return error
 */
#define QSPI_STATE_READY        (1U)

/**
 * \brief LLD is busy performing operation with peripherals, return error when APIs are invoked
 */
#define QSPI_STATE_BUSY         (2U)

/**
 * \brief LLD ran into error, returns error for all APIs other than deinit in this state
 */
#define QSPI_STATE_ERROR        (3U)

/**@}*/

/** @name LLD (Low Level Driver) Transaction states
 */
/**@{*/
/**
 * \brief Transaction is in idle state
 */
#define QSPI_STATE_IDLE                 (0U)
/**
 * \brief This transaction state indicates address to be transferred
 */
#define QSPI_STATE_ADDRESS_WRITE        (1U)
/**
 * \brief This transaction state indicates write functionality
 */
#define QSPI_STATE_DATA_WRITE           (2U)
/**
 * \brief This transaction state indicates read functionality
 */
#define QSPI_STATE_DATA_READ            (3U)
/**
 * \brief This transaction state indicates Block for read completion
 */
#define QSPI_STATE_BLOCK                (4U)
/**
 * \brief This transaction state  indicates Don't Block for read completion
 */
#define QSPI_STATE_NON_BLOCK            (5U)

/**@}*/

/* ========================================================================== */
/*                      Function pointers Declarations                        */
/* ========================================================================== */

/**
 *  \brief  The definition of a get System Tick function used by
 *  the QSPI driver to keep track of time
 *
 *  \return Returns system ticks in 32-bit unsigned int format
 *
 */
typedef uint32_t (*QSPI_Clock_getTicks)(void);

/**
 *  \brief  The definition of a micro seconds to ticks function used by
 *  the QSPI driver to get ticks from microseconds
 *
 *  \param usecs                        Micro Seconds
 *
 *  \return Returns system ticks in 32-bit unsigned int format
 *
 */
typedef uint32_t (*QSPI_Clock_usecToTicks)(uint64_t usecs);

/**
 *  \brief  The definition of a interrupt completion callback function used by
 *  the QSPI driver
 *
 *  \param args                         Void Pointer
 */
/* Type definition for the callback function*/
typedef void (*QSPI_lld_InterruptCallback)(void* args);

/**
 *  \brief  The definition of a dma read completion callback function used by
 *  the QSPI driver
 *
 *  \param args                         Void Pointer
 */
typedef void (*QSPI_lld_dma_readCompleteCallback)(void* args);

/**
 *  \anchor QSPI_Structure
 *  \name QSPI Driver Initialization Structure
 *
 *  Status codes that are set by the QSPI driver
 *
 *  @{
 */
/* ========================================================================== */
/*                             Structure Definitions                          */
/* ========================================================================== */
/**
 *  \brief QSPI Driver Transaction
 */
typedef struct
{
    uint32_t            count;
    /**< [IN] Number of word for this transaction */
    uint32_t            dataLen;
    /**< [IN] Number of frames for this transaction */
    void                *buf;
    /**< [IN] void * to a buffer to receive/send data */
    uint32_t            addrOffset;
    /**< [IN] Address offset to write to an QSPI flash device. */
    bool                status;
    /**< [OUT] \ref QSPI_TransferStatus code set by \ref QSPI_Transfers */
    bool                readWriteFlag;
    /**< [IN] True for write and vice versa for read. */
    uint32_t            cmdRegVal;
    /**< [IN] SPI_CMD register value to be written. */
    uint32_t            currentIndex;
    /**< [IN] Index for the transfer. */
    uint8_t             cmd;
    /**< [IN] Command Opcode */
    uint8_t             numAddrBytes;
    /**< [IN] Number of address bytes used to send cmd address */
    uint32_t            wlen;
    /**< [IN] word length to be used for this transaction. */
    uint32_t            state;
    /**< [OUT] Transaction state for read and write */
} QSPILLD_Transaction,*QSPILLD_TransactionHandle;

/**
 *  \brief QSPI Transaction Info
 */
typedef struct
{
    uint8_t             cmd;
    /**< [IN] Command Opcode */
    uint32_t            cmdAddr;
    /**< [IN] Address required by the command. Usually needed in writing to flash registers.
    Should be initialized to #QSPI_CMD_INVALID_ADDR if not used. */
    uint8_t             numAddrBytes;
    /**< [IN] Number of address bytes used to send cmd address */
    void                *dataBuf;
    /**< [IN] Buffer containing command data */
    uint32_t            dataLen;
    /**< [IN] Length of response buffer */
} QSPILLD_WriteCmdParams;

/**
 *  \brief QSPI driver initialization object
 */
typedef struct
{
    uint32_t            memMapBaseAddr;
    /**< Memory mapped mode base address of QSPI flash */
    uint32_t            inputClkFreq;
    /**< Module input clock frequency */
    uint32_t            qspiClockDiv;
    /**< Module clock divider */
    uint32_t            chipSelect;
    /**< Qspi Chip select number */
    uint32_t            csPol;
    /**< Qspi Chip select polarity */
    uint32_t            frmFmt;
    /**< Qspi Frame format */
    uint32_t            dataDelay;
    /**< QSPI data delay */
    uint32_t            wrdLen;
    /**< Number of bits in a word */
    uint32_t            rxLines;
    /**< Number of rx Lines used for QSPI reading */
    QSPI_DmaHandle      qspiDmaHandle;
    /**< DMA Handle */
    QSPI_DmaChConfig    qspiDmaChConfig;
    /**< DMA Configuration for this instance. */
    uint32_t            intrNum;
    /**< Peripheral interrupt number */
    bool                intrEnable;
    /**< Enable interrupt mode */
    bool                wordIntr;
    /**< Word interrupt mode */
    bool                frameIntr;
    /**< Word interrupt mode */
    uint8_t             intrPriority;
    /**< Interrupt priority */
    bool                dmaEnable;
    /**< Enable DMA mode */
    QSPI_Clock_getTicks Clock_getTicks;
    /**< Function Pointer for the QSPI clock get ticks >*/
    QSPI_Clock_usecToTicks Clock_usecToTicks;
    /**< Function Pointer for the QSPI micro second to tick conversion >*/
} QSPILLD_InitObject, *QSPILLD_InitHandle;

/**
 *  \brief QSPI driver object
 */
typedef struct
{
    /*
     * SOC configuration
     */
    uint32_t            baseAddr;
    /**< Peripheral base address */
    uint8_t             readCmd;
    /**< Transfer command to be used for reading from QSPI flash */
    uint8_t             writeCmd;
    /**< Transfer command to be used for writing to QSPI flash */
    uint32_t            frmLength;
    /**< Frame length of total transfer */
    uint32_t            numAddrBytes;
    /**< Number of bytes used to represent address to be sent to flash. */
    uint32_t            numDummyBits;
    /**< Number of dummy bits required while reading from flash */
    QSPILLD_InitHandle  hQspiInit;
    /**< Initialization parameters of QSPI instance */
    uint32_t            state;
    /** State variables. */
    void*               args;
    /**< Pointer to be used by application to store miscellaneous data.*/
    QSPILLD_Transaction        *transaction;
    /**< Pointer to current transaction struct */
    QSPILLD_Transaction        trans;
    /**< Current transaction struct */
    QSPI_lld_InterruptCallback interruptCallback;
    /* Interrupt Callback to be registered by the application*/
    QSPI_lld_dma_readCompleteCallback readCompleteCallback;
    /* DMA Readcallback function*/
} QSPILLD_Object, *QSPILLD_Handle;

/**@}*/

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief  This function initializes the QSPI module
 *
 *  \param  hQspi      #QSPILLD_Handle of QSPI instance.
 *
 *  \return #QSPI_SYSTEM_SUCCESS on success else error on failure
 */
int32_t QSPI_lld_init(QSPILLD_Handle hQspi);

/**
 *  \brief  This function initializes the QSPI module in DMA Mode
 *
 *  \param  hQspi      #QSPILLD_Handle of QSPI instance.
 *
 *  \return #QSPI_SYSTEM_SUCCESS on success else error on failure
 */
int32_t QSPI_lld_initDma(QSPILLD_Handle hQspi);

/**
 *  \brief  This function de-initializes the QSPI module
 *
 *  \param  hQspi      #QSPILLD_Handle of QSPI instance.
 *
 *  \return #QSPI_SYSTEM_SUCCESS on success else error on failure
 */
int32_t QSPI_lld_deInit(QSPILLD_Handle hQspi);

/**
 *  \brief  This function de-initializes the QSPI module in DMA Mode
 *
 *  \param  hQspi      #QSPILLD_Handle of QSPI instance.
 *
 *  \return #QSPI_SYSTEM_SUCCESS on success else error on failure
 */
int32_t QSPI_lld_deInitDma(QSPILLD_Handle hQspi);

/**
 *  \brief  Function to send specific commands and receive related data from flash in configuration mode
 *
 *  \param  hQspi      #QSPILLD_Handle of QSPI instance.
 *  \param  writeMsg   Pointer to a structure #QSPILLD_WriteCmdParams contains read command OPCODE,
 *                     flash memory address, buffer to store the response from flash and buffer length
 *
 *  \return #QSPI_SYSTEM_SUCCESS if command read was successful; else error on failure
 */
int32_t QSPI_lld_readCmd(QSPILLD_Handle hQspi, QSPILLD_WriteCmdParams *writeMsg);

/**
 *  \brief  Function to send specific commands and receive related data from flash in interrupt mode
 *
 *  \param  hQspi      #QSPILLD_Handle of QSPI instance.
 *  \param  msg        Pointer to a structure #QSPILLD_WriteCmdParams contains read command OPCODE,
 *                     flash memory address, buffer to store the response from flash and buffer length
 *
 *  \return #QSPI_SYSTEM_SUCCESS if command read was successful; else error on failure
 */
int32_t QSPI_lld_readCmdIntr(QSPILLD_Handle hQspi,const QSPILLD_WriteCmdParams *msg);

/**
 *  \brief  Function to send specific commands and write data into flash in configuration mode
 *
 *  \param  hQspi      #QSPILLD_Handle of QSPI instance.
 *  \param  writeMsg   Pointer to a structure #QSPILLD_WriteCmdParams contains write command OPCODE,
 *                     flash memory address, buffer to write into flash and buffer length
 *
 *  \return #QSPI_SYSTEM_SUCCESS if command read was successful; else error on failure
 */
int32_t QSPI_lld_writeCmd(QSPILLD_Handle hQspi, QSPILLD_WriteCmdParams *writeMsg);

/**
 *  \brief  Function to send specific commands and write data into flash in interrupt mode
 *
 *  \param  hQspi      #QSPILLD_Handle of QSPI instance.
 *  \param  msg        Pointer to a structure #QSPILLD_WriteCmdParams contains write command OPCODE,
 *                     flash memory address, buffer to write into flash and buffer length
 *
 *  \return #QSPI_SYSTEM_SUCCESS if command write was successful; else error on failure
 */
int32_t QSPI_lld_writeCmdIntr(QSPILLD_Handle hQspi, const QSPILLD_WriteCmdParams *msg);

/**
 *  \brief  Function to perform reads from the flash in memory map mode.
 *
 *  \param  hQspi       #QSPILLD_Handle of QSPI instance.
 *  \param  count       Number of bytes to be read from flash.
 *  \param  rxBuf       Pointer to Receive buffer
 *  \param  addrOffset  Address offset
 *  \param  timeout     Timeout for
 *
 *  \return #QSPI_SYSTEM_SUCCESS on successful read; else error on failure
 */
int32_t QSPI_lld_read(QSPILLD_Handle hQspi, uint32_t count, void* rxBuf, uint32_t addrOffset, uint32_t timeout);

/**
 *  \brief  Function to perform read from the flash in DMA
 *          mode.
 *
 *  \param  hQspi       #QSPILLD_Handle of QSPI instance.
 *  \param  count       Number of bytes to be read from flash.
 *  \param  rxBuf       Pointer to receive buffer
 *  \param  addrOffset  Address offset
 *  \param  timeout     timeout parameter
 *
 *  \return #QSPI_SYSTEM_SUCCESS on successful read; else error on failure
 */
int32_t QSPI_lld_readDma(QSPILLD_Handle hQspi, uint32_t count, void* rxBuf, uint32_t addrOffset, uint32_t timeout);

/* API's used by flash driver */

/**
 *  \brief  Function to set write command to be used.
 *
 *  \param  hQspi       #QSPILLD_Handle of QSPI instance.
 *  \param  command     Write command for a particular flash
 *
 *  \return #QSPI_SYSTEM_SUCCESS on success, #QSPI_SYSTEM_FAILURE otherwise
 */
int32_t QSPI_lld_setWriteCmd(QSPILLD_Handle hQspi, uint8_t command);

/**
 *  \brief  Function to set read command to be used.
 *
 *  \param  hQspi       #QSPILLD_Handle of QSPI instance.
 *  \param  command     Read command for a particular flash
 *
 *  \return #QSPI_SYSTEM_SUCCESS on success, #QSPI_SYSTEM_FAILURE otherwise
 */
int32_t QSPI_lld_setReadCmd(QSPILLD_Handle hQspi, uint8_t command);

/**
 *  \brief  Function to set number of address bytes to be used.
 *
 *  \param  hQspi       #QSPILLD_Handle of QSPI instance.
 *  \param  count       Number of Address Bytes. This function
 *                      should always be called before accessing
 *                      a memory location that doesn't fit in the
 *                      current address byte range.
 *
 *  \return #QSPI_SYSTEM_SUCCESS on success, #QSPI_SYSTEM_FAILURE otherwise
 */
int32_t QSPI_lld_setAddressByteCount(QSPILLD_Handle hQspi, uint32_t count);

/**
 *  \brief  Function to set number of dummy bits to be used.
 *
 *  \param  hQspi       #QSPILLD_Handle of QSPI instance.
 *  \param  count       Number of Dummy Bits. This should be either
 *                      less than 8 or a multiple of 8, because Bit
 *                      count field in setup register works only if
 *                      Byte count is 0.
 *
 *  \return #QSPI_SYSTEM_SUCCESS on success, #QSPI_SYSTEM_FAILURE otherwise
 */
int32_t QSPI_lld_setDummyBitCount(QSPILLD_Handle hQspi, uint32_t count);

/**
 *  \brief  This function is used to switch between memory mapped and
 *          configuration mode.
 *
 *  \param  hQspi       #QSPILLD_Handle of QSPI instance.
 *  \param   memMappedPortSwitch    Flag for switching mode.
 *
 *  \return  #QSPI_SYSTEM_SUCCESS on success, #QSPI_SYSTEM_FAILURE otherwise
 */
int32_t QSPI_lld_setMemAddrSpace(QSPILLD_Handle hQspi, uint32_t memMappedPortSwitch);

/**
 *  \brief   Set QSPI Rx lines in the QSPI object
 *
 *  \details This function sets the QSPI RX lines in QSPI object so that it
 *           can be used in a subsequent read
 *
 *  \param   hQspi       #QSPILLD_Handle of QSPI instance.
 *
 *  \param   rxLines    \ref QSPI_TransferLines value
 *
 *  \return  #QSPI_SYSTEM_SUCCESS on success, #QSPI_SYSTEM_FAILURE otherwise
 */
int32_t QSPI_lld_setRxLines(QSPILLD_Handle hQspi, uint32_t rxLines);

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
 *  \param   hQspi       #QSPILLD_Handle of QSPI instance.
 *
 *  \param   clkDividerVal    Clock divider value to be set.
 *
 *  \return  #QSPI_SYSTEM_SUCCESS on success, #QSPI_SYSTEM_SUCCESS otherwise
 */
int32_t QSPI_lld_setPreScaler(QSPILLD_Handle hQspi, uint32_t clkDividerVal);

/**
 *  \brief  This function returns the input clock at which QSPI was programmed
 *
 *  \pre    QSPI controller has been opened using #QSPI_open()
 *
 *  \param  hQspi       #QSPILLD_Handle of QSPI instance.
 *  \param  inputClk    Pointer where input clock frequency will be stored.
 *
 *  \return  #QSPI_SYSTEM_SUCCESS on success, #QSPI_SYSTEM_FAILURE otherwise
 *
 *  \sa     #QSPI_init()
 *  \sa     #QSPI_open()
 */
int32_t QSPI_lld_getInputClk(QSPILLD_Handle hQspi, uint32_t* inputClk);

/**
 *  \brief   Get QSPI Rx lines in the QSPI object
 *
 *  \details This function returns the QSPI RX lines in QSPI object
 *
 *  \param  hQspi       #QSPILLD_Handle of QSPI instance.
 *
 *  \return  #QSPI_SYSTEM_SUCCESS on success, #QSPI_SYSTEM_FAILURE otherwise
 */
uint32_t QSPI_lld_getRxLines(QSPILLD_Handle hQspi);

/**
 *  \brief   QSPI ISR for Read and Write Functionality.
 *
 *  \param  args       #QSPILLD_Handle of QSPI instance.
 */

void QSPI_lld_isr(void* args);

/**
 *  \brief   QSPI Read complete callback for DMA mode
 *
 *  \param  args       #QSPILLD_Handle of QSPI instance.
 */
void QSPI_lld_readCompleteCallback(void* args);


#ifdef __cplusplus
}
#endif

#endif /* #ifndef QSPI_LLD_H_ */

/** @} */
