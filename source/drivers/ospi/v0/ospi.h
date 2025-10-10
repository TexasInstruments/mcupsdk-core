/*
 *  Copyright (C) 2021-24 Texas Instruments Incorporated
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
 *  \defgroup DRV_OSPI_MODULE APIs for OSPI
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the OSPI module. The APIs
 *  can be used by other drivers to get access to OSPI and also by
 *  application to initiate data transfer operation.
 *
 *  @{
 */

/**
 *  \file v0/ospi.h
 *
 *  \brief OSPI Driver API/interface file.
 */

#ifndef OSPI_H_
#define OSPI_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <drivers/ospi/v0/lld/ospi_lld.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/SemaphoreP.h>


#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

struct OSPI_Config_s;

/** \brief A handle that is returned from a #OSPI_open() call */
typedef struct OSPI_Config_s *OSPI_Handle;

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/** \brief OSPI instance attributes - used during init time */
typedef struct
{
    /*
     * SOC configuration
     */
    uint32_t                baseAddr;
    /**< Peripheral base address */
    uint32_t                dataBaseAddr;
    /**< Base address of the OSPI flash */
    uint32_t                moduleId;
    /**< OSPI Module Id */
    uint32_t                clkId;
    /**< OSPI Clock Id */
    uint32_t                inputClkFreq;
    /**< Module input clock frequency */

    /*
     * Driver configuration
     */
    uint32_t protocol;
    /**< Protocol for OSPI reading/writing. 32 bit integer with
     * byte0 -> data lines
     * byte1 -> addr lines
     * byte2 -> cmd lines
     * byte3 -> STR/DTR (0 = STR, 1 = DTR)
     * */
    uint32_t                intrNum;
    /**< Peripheral interrupt number */
    uint32_t                intrEnable;
    /**< Enable interrupt mode */
    uint8_t                 intrPriority;
    /**< Interrupt priority */
    uint32_t                dmaEnable;
    /**< Enable DMA mode */
    uint32_t                phyEnable;
    /**< Enable PHY mode */
    uint32_t                dacEnable;
    /**< Enable DAC mode */
    uint32_t                frmFmt;
    /**< Ospi Frame Format */
    uint32_t                devDelays[4];
    /**< OSPI device delays (CSSOT, CSEOT, CSDADS and CSDA delays) */
    uint32_t                chipSelect;
    /**< Ospi Chip select number */
    uint32_t                decChipSelect;
    /**< Decoder Chip select number */
    uint32_t                baudRateDiv;
    /**< Baud-rate divisor to derive DQS and other output clks */
    uint32_t                validateOtp;
    /**< Enable Phy Tuning Point validatation */
    const OSPI_AddrRegion *dmaRestrictedRegions;
    /**< Pointer to array of OSPI_AddrRegion data structures filled by SysConfig. The
    array should be terminated by a { 0xFFFFFFFFU, 0U } entry. It is used while
    using DMA copy to check if the destination address is a region not accessible to DMA
    and switch to CPU copy */
    OSPI_PhyConfiguration  phyConfiguration;
    /**< OSPI PHY configuration params */

} OSPI_Attrs;

/**
 *  \brief OSPI driver object
 */

typedef struct
{
    /*
     * User params
     */
    OSPI_Handle handle;
    /**< Instance handle */
    /*
     * State variables
     */
    uint32_t isOpen;
    /**< Flag to indicate if the instance is already open */
    uint32_t isDacEnable;
    /**< Flag to indicate if DAC mode is enabled or not */
    SemaphoreP_Object       lockObj;
    /**< Driver lock object */
    SemaphoreP_Object       transferSemObj;
    /**< Transfer Sync Semaphore object */
    HwiP_Object             hwiObj;
    /**< Interrupt object */

    OSPI_Transaction *currTrans;
    /**< Pointer to current transaction struct */
    void* ospiDmaHandle;

    OSPILLD_InitObject      ospilldInitObject;
    /* OSPI LLD Init Object and Handle */

    OSPILLD_Object          ospilldObject;
    OSPILLD_Handle          ospilldHandle;
    /* QSPI LLD Object and Handle */
} OSPI_Object;

typedef struct OSPI_Config_s
{
    const OSPI_Attrs *attrs;
    /**< Pointer to driver specific hardware attributes */
    OSPI_Object *object;
    /**< Pointer to driver specific data object */
} OSPI_Config;

/** \brief Externally defined driver configuration array */
extern OSPI_Config gOspiConfig[];
/** \brief Externally defined driver configuration array size */
extern uint32_t    gOspiConfigNum;

extern OSPI_DmaConfig gOspiDmaConfig[];

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
 *  \brief  This function initializes the OSPI module
 */
void OSPI_init(void);

/**
 *  \brief  This function de-initializes the OSPI module
 */
void OSPI_deinit(void);


/**
 *  \brief  Initialize data structure with defaults
 *
 *  \param  ospiParams [out] Initialized parameters
 */
void OSPI_Params_init( OSPI_Params *ospiParams);

/**
 *  \brief  This function opens a given OSPI peripheral
 *
 *  \pre    OSPI controller has been initialized using #OSPI_init()
 *
 *  \param  index       Index of config to use in the *OSPI_Config* array
 *  \param  openParams  Pointer to parameters to open the driver with
 *
 *  \return A #OSPI_Handle on success or a NULL on an error or if it has been
 *          opened already
 *
 *  \sa     #OSPI_init()
 *  \sa     #OSPI_close()
 */
OSPI_Handle OSPI_open(uint32_t index, const OSPI_Params *openParams);

/**
 *  \brief  Function to close a OSPI peripheral specified by the OSPI handle
 *
 *  \pre    #OSPI_open() has to be called first
 *
 *  \param  handle      #OSPI_Handle returned from #OSPI_open()
 *
 *  \sa     #OSPI_open()
 */
void OSPI_close(OSPI_Handle handle);

/**
 *  \brief  This function returns the handle of an open OSPI Instance from the instance index
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  index Index of config to use in the *OSPI_Config* array
 *
 *  \return An #OSPI_Handle if it has been opened already or NULL otherwise
 *
 *  \sa     #OSPI_init()
 *  \sa     #OSPI_open()
 */
OSPI_Handle OSPI_getHandle(uint32_t index);

/**
 *  \anchor OSPI_Transfers_HLD
 *  \name Different OSPI Transfer functions
 *
 *  @{
 */
/**
 *  \brief  Function to perform direct reads from the flash using DAC controller
 *
 *
 *  \param  handle      #OSPI_Handle returned from #OSPI_open()
 *  \param  trans       Pointer to a OSPI_Trans
 *
 *  \return #SystemP_SUCCESS on successful read; else error on failure
 *
 *  \sa     #OSPI_open
 */
int32_t OSPI_readDirect(OSPI_Handle handle, OSPI_Transaction *trans);

/**
 *  \brief  Function to perform indirect reads from the flash using INDAC controller
 *
 *
 *  \param  handle      #OSPI_Handle returned from #OSPI_open()
 *  \param  trans       Pointer to a OSPI_Trans
 *
 *  \return #SystemP_SUCCESS on successful read; else error on failure
 *
 *  \sa     #OSPI_open
 */
int32_t OSPI_readIndirect(OSPI_Handle handle, OSPI_Transaction *trans);

/**
 *  \brief  Function to perform direct writes to the flash using DAC controller
 *
 *
 *  \param  handle      #OSPI_Handle returned from #OSPI_open()
 *  \param  trans       Pointer to a OSPI_Trans
 *
 *  \return #SystemP_SUCCESS on successful write; else error on failure
 *
 *  \sa     #OSPI_open
 */
int32_t OSPI_writeDirect(OSPI_Handle handle, OSPI_Transaction *trans);

/**
 *  \brief  Function to perform indirect writes to the flash using INDAC controller
 *
 *
 *  \param  handle      #OSPI_Handle returned from #OSPI_open()
 *  \param  trans       Pointer to a OSPI_Trans
 *
 *  \return #SystemP_SUCCESS on successful write; else error on failure
 *
 *  \sa     #OSPI_open
 */
int32_t OSPI_writeIndirect(OSPI_Handle handle, OSPI_Transaction *trans);
/** @} */

/**
 *  \brief  Function to send specific commands and receive related data from flash
 *
 *
 *  \param  handle      #OSPI_Handle returned from #OSPI_open()
 *  \param  rdParams    Pointer to a OSPI_ReadCmdParams
 *
 *  \return #SystemP_SUCCESS if command read was successful; else error on failure
 *
 *  \sa     #OSPI_open
 */
int32_t OSPI_readCmd(OSPI_Handle handle, OSPI_ReadCmdParams *rdParams);

/**
 *  \brief  Function to send specific commands and related data to flash
 *
 *
 *  \param  handle      #OSPI_Handle returned from #OSPI_open()
 *  \param  wrParams    Pointer to a OSPI_WriteCmdParams
 *
 *  \return #SystemP_SUCCESS if command write was successful; else error on failure
 *
 *  \sa     #OSPI_open
 */
int32_t OSPI_writeCmd(OSPI_Handle handle, OSPI_WriteCmdParams *wrParams);

/**
 *  \brief  Function to initialize the OSPI_Transaction
 *
 *
 *  \param  trans       Pointer to a OSPI_Transaction
 *
 */
void OSPI_Transaction_init(OSPI_Transaction *trans);

/**
 *  \brief  Function to initialize the OSPI_ReadCmdParams structure
 *
 *
 *  \param  rdParams Pointer to a OSPI_ReadCmdParams
 *
 */
void OSPI_ReadCmdParams_init(OSPI_ReadCmdParams *rdParams);

/**
 *  \brief  Function to initialize the OSPI_WriteCmdParams structure
 *
 *
 *  \param  wrParams Pointer to a OSPI_WriteCmdParams
 *
 */
void OSPI_WriteCmdParams_init(OSPI_WriteCmdParams *wrParams);

/**
 *  \brief  This function returns the input clk frequency OSPI was programmed at
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *
 *  \return OSPI RCLK in Hertz
 */
uint32_t OSPI_getInputClk(OSPI_Handle handle);

/**
 *  \brief  This function checks if the Direct Access Controller mode is enabled
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *
 *  \return 1 if DAC is enabled, 0 otherwise
 */
uint32_t OSPI_isDacEnable(OSPI_Handle handle);

/**
 *  \brief  This function checks if DMA is enabled for reads
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *
 *  \return 1 if DMA is enabled, 0 otherwise
 */
uint32_t OSPI_isDmaEnable(OSPI_Handle handle);

/**
 *  \brief  This function checks if interrupts are enabled
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *
 *  \return 1 if interrupts are enabled, 0 otherwise
 */
uint32_t OSPI_isIntrEnable(OSPI_Handle handle);

/**
 *  \brief  This function checks if the OSPI PHY controller is enabled
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *
 *  \return 1 if PHY is enabled, 0 otherwise
 */
uint32_t OSPI_isPhyEnable(OSPI_Handle handle);

/**
 *  \brief  This function checks if the Dual Transfer Rate
 *          (Sampling on both rising and falling edge of the clock) is enabled
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *
 *  \return 1 if Dual Transfer Rate is enabled, 0 otherwise
 */
uint32_t OSPI_isDtrEnable(OSPI_Handle handle);

/**
 *  \brief  This function enables the Dual Data Rate (DDR)
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *
 *  \return #SystemP_SUCCESS if DDR successfully enabled, else error on failure
 */
int32_t OSPI_enableDDR(OSPI_Handle handle);

/**
 *  \brief  This function enables the Single Data Rate (SDR)
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *
 *  \return #SystemP_SUCCESS if SDR successfully enabled, else error on failure
 */
int32_t OSPI_enableSDR(OSPI_Handle handle);

/**
 *  \brief  This function sets DDR bit in INSTR_RD register for RD commands
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *
 *  \return #SystemP_SUCCESS if SDR successfully enabled, else error on failure
 */
int32_t OSPI_enableDdrRdCmds(OSPI_Handle handle);

/**
 *  \brief  This function sets read data capture cycles in the OSPI controller
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle             An #OSPI_Handle returned from an #OSPI_open()
 *  \param  rdDataCapDelay     Number of read data capture cycles to be programmed
 *
 *  \return #SystemP_SUCCESS if read data capture cycles successfully set, else error on failure
 */
int32_t OSPI_setRdDataCaptureDelay(OSPI_Handle handle, uint32_t rdDataCapDelay);

/**
 *  \brief  This function set the number of bytes used to send address while reading or writing to flash memory
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle           An #OSPI_Handle returned from an #OSPI_open()
 *  \param  numAddrBytes     Number of address bytes to be used while sending addresses.
 *
 */
void OSPI_setNumAddrBytes(OSPI_Handle handle, uint32_t numAddrBytes);

/**
 *  \brief  This function sets the block size and page size of the flash to the device size register in OSPI
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle           An #OSPI_Handle returned from an #OSPI_open()
 *  \param  pageSize         Page size of the flash in bytes
 *  \param  blkSize          Block size of the flash in bytes
 *
 */
void OSPI_setDeviceSize(OSPI_Handle handle, uint32_t pageSize, uint32_t blkSize);

/**
 *  \brief  This function sets appropriate dummy cycles to be used while sending STIG commands to flash
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle          An #OSPI_Handle returned from an #OSPI_open()
 *  \param  cmdDummyCycles  Dummy cycles to be used for STIG cmd
 *
 */
void OSPI_setCmdDummyCycles(OSPI_Handle handle, uint32_t cmdDummyCycles);

/**
 *  \brief  This function sets appropriate dummy cycles for flash read
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *  \param  dummyCycles     Number of dummy cycles
 *
 */
void OSPI_setReadDummyCycles(OSPI_Handle handle, uint32_t dummyCycles);

/**
 *  \brief  This function sets appropriate dummy cycles for flash write
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *  \param  dummyCycles     Number of dummy cycles
 *
 */
void OSPI_setWriteDummyCycles(OSPI_Handle handle, uint32_t dummyCycles);

/**
 *  \brief  This function sets the phyEnableSuccess field in \ref OSPI_Object. Has to be called from flash driver
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *  \param  success Set this to 1 if phy enable was successful. 0 otherwise
 *
 */
void OSPI_setPhyEnableSuccess(OSPI_Handle handle, uint32_t success);

/**
 *  \brief  This function sets mode bits in the mode bit field of OSPI config register.
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle   An #OSPI_Handle returned from an #OSPI_open()
 *  \param  modeBits Number of mode bits to be set
 *
 */
void OSPI_setModeBits(OSPI_Handle handle, uint32_t modeBits);

/**
 *  \brief  This function enables mode bits transmission while sending CMDs
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *
 */
void OSPI_enableModeBitsCmd(OSPI_Handle handle);

/**
 *  \brief  This function enables mode bits transmission while reading
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *
 */
void OSPI_enableModeBitsRead(OSPI_Handle handle);

/**
 *  \brief  This function fetches the phyEnableSuccess field in \ref OSPI_Object.
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *
 *  \return 1 if phyEnable was successful, 0 otherwise.
 */
uint32_t OSPI_getPhyEnableSuccess(OSPI_Handle handle);

/**
 *  \brief  This function sets command mode bit
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *  \param  enable     command mode bit enable/disable
 *
 */
void OSPI_cmdModeBitSet(OSPI_Handle handle, uint32_t enable);

/**
 *  \brief  This function sets Read mode bit
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *  \param  enable     Read mode bit enable/disable
 *
 */
void OSPI_rdModeBitSet(OSPI_Handle handle, uint32_t enable);

/**
 *  \brief  This function returns the current protocol for which the transfer lines in
 *          OSPI driver is configured for.
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *
 *  \return protocol Protocol being used
 *
 */
uint32_t OSPI_getProtocol(OSPI_Handle handle);

/**
 *  \brief  This function sets the number of transfer lines in the OSPI driver to
 *          set the requested protocol
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *  \param  protocol Protocol to be used
 *
 */
void OSPI_setProtocol(OSPI_Handle handle, uint32_t protocol);

/**
 *  \brief  This function sets OSPI controller to use dual byte opcodes
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *
 */
void OSPI_setDualOpCodeMode(OSPI_Handle handle);

/**
 *  \brief  This function sets OSPI controller to not use dual byte opcodes
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *
 */
void OSPI_clearDualOpCodeMode(OSPI_Handle handle);

/**
 *  \brief  This function sets the opcodes for reading and page programming the flash
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle      An #OSPI_Handle returned from an #OSPI_open()
 *  \param  readCmd     Command opcode to be used for reading from the flash
 *  \param  pageProgCmd Command opcode to be used for writing to / programming the flash
 *
 */
void OSPI_setXferOpCodes(OSPI_Handle handle, uint8_t readCmd, uint8_t pageProgCmd);

/**
 *  \brief  This function sets the type of command extension used in dual byte opcode mode
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle      An #OSPI_Handle returned from an #OSPI_open()
 *  \param  cmdExtType  Type of command extension used. As of now only two types are supported - REPEAT and INVERSE
 *
 */
void OSPI_setCmdExtType(OSPI_Handle handle, uint32_t cmdExtType);

/**
 *  \brief  This function enables the Direct Access Mode
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t OSPI_enableDacMode(OSPI_Handle handle);

/**
 *  \brief  This function disables the Direct Access Mode
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t OSPI_disableDacMode(OSPI_Handle handle);

/**
 *  \brief  This function gets the SOC mapped data base address of the flash
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *
 *  \return Data BaseAddress of the flash
 */
uint32_t OSPI_getFlashDataBaseAddr(OSPI_Handle handle);

/**
 *  \brief  This function tunes the OSPI PHY for DDR mode to set optimal PHY parameters
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *
 *  \param  flashOffset Offset of the flash at which the PHY tuning data is present
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t OSPI_phyTuneDDR(OSPI_Handle handle, uint32_t flashOffset);

int32_t OSPI_phyTuneSDR(OSPI_Handle handle, uint32_t flashOffset);

/**
 *  \brief  This function takes a 4x128x128 array and fills it with TX RX DLL data for graphing purpose
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *
 *  \param  flashOffset Offset of the flash at which the PHY tuning data is present
 *
 *  \param  arrays A 4x128x128 array. In this, the tx rx dll data for 4 different read delay values will be stored. This can be later used for graphing
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t OSPI_phyTuneGrapher(OSPI_Handle handle, uint32_t flashOffset, uint8_t arrays[5][128][128]);

/**
 *  \brief  This function returns the address to the attack vector buf required for tuning the PHY
 *
 *  \param  tuningData Address of the tuningData array
 *
 *  \param  tuningDataSize Size of the tuningData array
 */
void OSPI_phyGetTuningData(uint32_t *tuningData, uint32_t *tuningDataSize);

/**
 *  \brief  This function checks if the attack vector, or the data used for tuning the PHY is present at an offset in the flash
 *
 *  \param  handle An #OSPI_Handle returned from an #OSPI_open()
 *
 *  \param  offset Offset in flash to check for tuningData
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t OSPI_phyReadAttackVector(OSPI_Handle handle, uint32_t offset);

/**
 *  \brief  This function enables the PHY
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t OSPI_enablePhy(OSPI_Handle handle);

/**
 *  \brief  This function disables the PHY
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t OSPI_disablePhy(OSPI_Handle handle);

/**
 *  \brief  This function enables the PHY Pipeline
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t OSPI_enablePhyPipeline(OSPI_Handle handle);

/**
 *  \brief  This function disables the PHY Pipeline
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t OSPI_disablePhyPipeline(OSPI_Handle handle);


/**
 *  \ingroup DRV_OSPI_MODULE
 *  \defgroup DRV_OSPI_NOR_FLASH_API_MODULE Generic NOR Flash API for single pin mode
 *
 *   These APIs try to communicate with whichever flash is connected
 *   the OSPI peripheral in 1-1-1 mode. This can be used in bringing up new flashes
 *   and mostly used for debug and diagnostic purposes
 *
 *  @{
 */
/**
 *  \brief  This function initializes the NOR flash to work in 1-1-1 mode
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPI_Handle returned from an #OSPI_open()
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t OSPI_norFlashInit1s1s1s(OSPI_Handle handle);

/**
 *  \brief  This function sets up internal bookkeeping variables for read, write
 *          and erase commands. This API has to be called immediately before
 *          \ref OSPI_norFlashInit1s1s1s
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  rdCmd     Command to be used in single mode read
 *  \param  wrCmd     Command to be used in single mode write/page program
 *  \param  eraseCmd  Command to be used to erase (block or sector)
 *
 */
void OSPI_norFlashSetCmds(uint8_t rdCmd, uint8_t wrCmd, uint8_t eraseCmd);

/**
 *  \brief  This function tries to read the JEDEC ID from the NOR flash connected to the OSPI peripheral
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle         An #OSPI_Handle returned from an #OSPI_open()
 *  \param  manufacturerId Pointer to a uint32_t variable. This will be filled with the manufacturer ID on success
 *  \param  deviceId       Pointer to a uint32_t variable. This will be filled with the device ID on success
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t OSPI_norFlashReadId(OSPI_Handle handle, uint32_t *manufacturerId, uint32_t *deviceId);

/**
 *  \brief  This function writes data to the flash at a specified offset
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle    An #OSPI_Handle returned from an #OSPI_open()
 *  \param  offset    Offset at which the data is to be written
 *  \param  buf       Buffer which has the data to be written to the flash
 *  \param  len       Number of bytes to be written to the flash
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t OSPI_norFlashWrite(OSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len);

/**
 *  \brief  This function reads data from the flash from a specified offset
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle    An #OSPI_Handle returned from an #OSPI_open()
 *  \param  offset    Offset at which the data is to be read from
 *  \param  buf       Buffer to which data will be written into
 *  \param  len       Number of bytes to be read from the flash
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t OSPI_norFlashRead(OSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len);

/**
 *  \brief  This function reads SFDP table from the flash from a specified offset
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle    An #OSPI_Handle returned from an #OSPI_open()
 *  \param  offset    Offset at which the data is to be read from
 *  \param  buf       Buffer to which data will be written into
 *  \param  len       Number of bytes to be read from the flash
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t OSPI_norFlashReadSfdp(OSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len);

/**
 *  \brief  This function erases 1 block of data starting from a provided address
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle    An #OSPI_Handle returned from an #OSPI_open()
 *  \param  address   Address of the data block to be erased. This address should be block aligned.
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t OSPI_norFlashErase(OSPI_Handle handle, uint32_t address);


/**
 *  \brief  This function configures reset functionality
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle    An #OSPI_Handle returned from an #OSPI_open()
 *  \param  config    reset config
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t OSPI_configResetPin(OSPI_Handle handle, uint32_t config);


/**
 *  \brief  Configures baud divider
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle      An #OSPI_Handle returned from an #OSPI_open()
 *  \param  baud        baudrate from 2 to 32 and divisible by 2
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t OSPI_configBaudrate(OSPI_Handle handle, uint32_t baud);

/**
 *  \brief  Return value of baudrate that is programmed in IP register
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle      An #OSPI_Handle returned from an #OSPI_open()
 *  \param  baudDiv     pointer to memory into which baudrate will be written
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t OSPI_readBaudRateDivFromReg(OSPI_Handle handle, uint32_t *baudDiv);

/**
 *  \brief  Return value of baudrate that is saved in OSPI Object
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle      An #OSPI_Handle returned from an #OSPI_open()
 *  \param  baudDiv     pointer to memory into which baudrate will be written
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t OSPI_getBaudRateDivFromObj(OSPI_Handle handle, uint32_t *baudDiv);

/**
 *  \brief  This function activates the RESET pin feature
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle       An #OSPI_Handle returned from an #OSPI_open()
 *  \param  pinStatus    pinStatus - HIGH/LOW
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t OSPI_setResetPinStatus(OSPI_Handle handle, uint32_t pinStatus);

/**
 * \brief Validates a specific tuning point for OSPI PHY
 *
 * This function validates whether a given tuning point can be used for
 * reliable communication with the OSPI flash device. It performs data
 * read operations at the specified flash offset to determine if
 * the current PHY settings provide error-free data transfer.
 *
 * \param handle       OSPI driver handle
 * \param flashOffset  Flash memory offset to use for validation testing
 *
 * \return SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t OSPI_phyValidateTuningPoint(OSPI_Handle handle, uint32_t flashOffset);

/**
 * \brief Checks if Phy tuning point validation is enabled
 *
 * This function verifies whether the validation for One-Time Programmable memory
 * is currently enabled on the OSPI interface.
 *
 * \param handle    Handle to the OSPI instance
 *
 * \return return 1 if OTP validation is enabled, 0 otherwise
 */
uint32_t OSPI_isValidateOtpEnable(OSPI_Handle handle);

/**
 *  \brief Sets the operating frequency for the OSPI peripheral
 *
 *  This function configures the OSPI controller to operate at the specified
 *  frequency based on the input clock frequency provided.
 *
 *  \param  handle       OSPI driver handle
 *  \param  inputClkFreq Input clock frequency in Hz
 *
 *  \return SystemP_SUCCESS on success, error code on failure
 */
int32_t OSPI_setFrequency(OSPI_Handle handle, uint64_t inputClkFreq);

/**
 *  \brief Sets timing delays for the OSPI interface based on input clock frequency
 *
 *  This function configures the appropriate timing delays for the OSPI interface
 *  to ensure reliable communication with external memory devices. The delays are
 *  calculated based on the provided input clock frequency.
 *
 *  \param handle        OSPI handle to the peripheral instance
 *  \param inputClkFreq  Input clock frequency in Hz
 *
 *  \return SystemP_SUCCESS on success, error code on failure
 */
int32_t OSPI_setDelays(OSPI_Handle handle, uint32_t inputClkFreq);

/**
 *  \brief Sets the baud rate divider for OSPI communication
 *
 *  This function configures the baud rate divider to control the OSPI clock frequency.
 *  The actual OSPI clock frequency is determined by dividing the input clock frequency
 *  by the specified baud rate divider.
 *
 *  \param handle       OSPI handle
 *  \param baudRateDiv  Baud rate divider value
 *
 *  \return SystemP_SUCCESS on success, error code on failure
 */
int32_t OSPI_setBaudRateDiv(OSPI_Handle handle, uint32_t baudRateDiv);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef OSPI_H_ */

