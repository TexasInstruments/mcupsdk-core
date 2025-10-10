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
 *  \defgroup DRV_OSPI_LLD_MODULE APIs for OSPI LLD
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the OSPI LLD module. The APIs
 *  can be used by other drivers to get access to OSPI and also by
 *  application to initiate data transfer operation.
 *
 *  @{
 */

/**
 *  \file v0/lld/ospi_lld.h
 *
 *  \brief OSPI LLD Driver API/interface file.
 */

#ifndef OSPI_LLD_H_
#define OSPI_LLD_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/hw_include/cslr_ospi.h>
#include <drivers/ospi/v0/lld/dma/ospi_lld_dma.h>
#include <drivers/ospi/v0/cslr_ospi.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/**
 *  \anchor OSPI_TransferStatus
 *  \name Transfer Status Code
 *
 *  Status codes that are set by the OSPI driver
 *
 *  @{
 */
#define OSPI_TRANSFER_COMPLETED        (0U)
#define OSPI_TRANSFER_STARTED          (1U)
#define OSPI_TRANSFER_CANCELLED        (2U)
#define OSPI_TRANSFER_FAILED           (3U)
#define OSPI_TRANSFER_CSN_DEASSERT     (4U)
#define OSPI_TRANSFER_TIMEOUT          (5U)
/** @} */

/**
 *  \anchor OSPI_TransferMode
 *  \name Transfer Mode
 *
 *  This determines whether the driver operates synchronously or asynchronously
 *
 *  In #OSPI_TRANSFER_MODE_BLOCKING OSPI_Transfers block code
 *  execution until the transaction has completed
 *
 *  In #OSPI_TRANSFER_MODE_CALLBACK OSPI_Transfers does not block code
 *  execution and instead calls a callback function when the
 *  transaction has completed
 *
 *  @{
 */
/**
 *  \brief \ref OSPI_Transfers blocks execution. This mode can only be used
 *  when called within a Task context and is interrupt based
 */
#define OSPI_TRANSFER_MODE_BLOCKING    (0U)
/**
 *  \brief \ref OSPI_Transfers does not block code execution and will call a
 *  callback function. This mode can be used in a Task, Swi, or Hwi context
 */
#define OSPI_TRANSFER_MODE_CALLBACK    (1U)
/**
 *  \brief \ref OSPI_Transfers blocks execution. This mode can only be used
 *  when called within a Task context and is polling based.
 */
#define OSPI_TRANSFER_MODE_POLLING     (2U)
/** @} */

/**
 *  \anchor OSPI_FrameFormat
 *  \name Frame Format
 *
 *  Definitions for various SPI data frame formats
 *
 *  POL0 = OSPICLK is held low during the INACTIVE state
 *  POL1 = OSPICLK is held high during the INACTIVE state
 *
 *  PHA0 = Data launch is on the falling edge of OSPICLK
 *  PHA1 = Data launch is on the rising edge of OSPICLK
 *
 *  @{
 */
#define OSPI_FF_POL0_PHA0  (0U)
#define OSPI_FF_POL0_PHA1  (CSL_OSPI_FLASH_CFG_CONFIG_REG_SEL_CLK_PHASE_FLD_MASK | 0U)
#define OSPI_FF_POL1_PHA0  (0U | CSL_OSPI_FLASH_CFG_CONFIG_REG_SEL_CLK_POL_FLD_MASK)
#define OSPI_FF_POL1_PHA1  (CSL_OSPI_FLASH_CFG_CONFIG_REG_SEL_CLK_PHASE_FLD_MASK | \
                               CSL_OSPI_FLASH_CFG_CONFIG_REG_SEL_CLK_POL_FLD_MASK)
/** @} */

/**
*  \anchor OSPI_CmdMacros
*  \name Macros for invalid commands
*
*  Macros for invalid commands
*
*  @{
*/
#define OSPI_CMD_INVALID_OPCODE  (0xFFU)
#define OSPI_CMD_INVALID_DUMMY   (0xFFU)
#define OSPI_CMD_INVALID_ADDR    (0xFFFFFFFFU)
/** @} */

/**
*  \anchor OSPI_CmdExtTypes
*  \name Macros for command extension types
*
*  Macros for command extension types
*
*  @{
*/
#define OSPI_CMD_EXT_TYPE_REPEAT  (0x00U)
#define OSPI_CMD_EXT_TYPE_INVERSE (0x01U)
#define OSPI_CMD_EXT_TYPE_NONE    (0x02U)
/** @} */

/**
*  \anchor OSPI_NorProtocolTypes
*  \name Macros for OSPI protocol types
*
*  Macros for protocol types
*
*  @{
*/
#define OSPI_NOR_PROTOCOL(cmd, addr, data, dtr) (uint32_t)(((uint32_t)(dtr) << 24) | \
                                                           ((uint32_t)(cmd) << 16) | \
                                                           ((uint32_t)(addr) << 8) | \
                                                           ((uint32_t)(data) << 0))
#define OSPI_NOR_PROTOCOL_INVALID (uint32_t)(0xFFFFFFFF)

#define OSPI_PROTO_1S_1S_1S (0x0001)
#define OSPI_PROTO_1S_1S_2S (0x0002)
#define OSPI_PROTO_1S_1S_4S (0x0003)
#define OSPI_PROTO_1S_1S_8S (0x0004)
#define OSPI_PROTO_1S_8S_8S (0x000A)
#define OSPI_PROTO_4S_4S_4S (0x0005)
#define OSPI_PROTO_4S_4D_4D (0x0006)
#define OSPI_PROTO_8S_8S_8S (0x0007)
#define OSPI_PROTO_8D_8D_8D (0x0008)
#define OSPI_PROTO_CUSTOM   (0x0009)

#define OSPI_NAND_PROTOCOL(cmd, addr, data, dtr) (uint32_t)(((uint32_t)(dtr) << 24) | \
                                                           ((uint32_t)(cmd) << 16) | \
                                                           ((uint32_t)(addr) << 8) | \
                                                           ((uint32_t)(data) << 0))
#define OSPI_NAND_PROTOCOL_INVALID (uint32_t)(0xFFFFFFFF)
/** @} */

/**
*  \anchor OSPI_ChipSelect
*  \name Chip Selects
*
*  Chip selects
*
*  @{
*/
#define OSPI_CS0  (0U)
#define OSPI_CS1  (1U)
#define OSPI_CS2  (2U)
#define OSPI_CS3  (3U)

/** \brief OSPI Chip select macro */
#define OSPI_CHIP_SELECT(x)   ((~((1U) << (x))) & 0xFU)
/** @} */

#define OSPI_RESETPIN_DQ3       (0U)
#define OSPI_RESETPIN_DEDICATED (1U)

/**
 * \brief   OSPI controller controller mode baud rate divisor.
 *          OSPI baud rate = controller_ref_clk/BD, where BD is:
 *          0000 = /2
 *          0001 = /4
 *          0010 = /6
 *          ...
 *          1111 = /32
 */
#define CSL_OSPI_BAUD_RATE_DIVISOR(x)        (((x) - 2U) >> 1U)
#define MAX_BAUDRATE_DIVIDER                 (32U)
#define CSL_OSPI_BAUD_RATE_DIVISOR_DEFAULT   (CSL_OSPI_BAUD_RATE_DIVISOR(MAX_BAUDRATE_DIVIDER))

/**
*  \anchor OSPI_DecChipSelect
*  \name Decoder Chip Selects
*
*  Decoder Chip selects
*
*  @{
*/
#define OSPI_DECODER_SELECT4  ((uint32_t) 0U)
#define OSPI_DECODER_SELECT16 ((uint32_t) 1U)
/** @} */

/** @} */

/** @name OSPI Driver states
 */
/** @{ */

/** \brief OSPI driver is in Reset State prior to driver init
 *  and post driver deinit */
#define OSPI_STATE_RESET                     ((uint32_t) 0U)
/** \brief OSPI driver accepts runtime APIs only Ready State,
 *  otherwise return error */
#define OSPI_STATE_IDLE                      ((uint32_t) 1U)
/** \brief OSPI driver is busy performing operation with peripherals,
 *  return error when APIs are invoked */
#define OSPI_STATE_BUSY                      ((uint32_t) 2U)
/** \brief OSPI driver ran into error, returns error for all APIs
 *  other than deinit in this state */
#define OSPI_STATE_ERROR                     ((uint32_t) 3U)

/** @} */

/** @name OSPI Driver Return status
 */
/**@{ */
/**
 * \brief Return status when the API execution was successful
 */
#define OSPI_SYSTEM_SUCCESS     ((int32_t )0)
/**
 * \brief Return status when the API execution was not successful due to a failure
 */
#define OSPI_SYSTEM_FAILURE     ((int32_t)-1)
/**
 * \brief Return status when the API execution was not successful due to a time out
 */
#define OSPI_TIMEOUT            ((int32_t)-2)
/**
 * \brief Return status when the API execution failed due invalid parameters
 */
#define OSPI_LLD_INVALID_PARAM  ((int32_t)-3)
/**
 * \brief Return status when the API execution failed due to driver busy
 */
#define OSPI_BUSY               ((int32_t)-4)
/**
 * \brief Return status when the API execution failed due to invalid LLD state
 */
#define OSPI_INVALID_STATE      ((int32_t)-5)

/**@} */

/** @name OSPI Driver Timeout values
  */
/**@{ */

/**
 * \brief Value to use when needing a timeout of zero or NO timeout, return immediately on resource not available.
 */
#define OSPI_NO_WAIT        ((uint32_t)0)
/**
 * \brief Value to use when needing a timeout of infinity or wait forver until resource is available
 */
#define OSPI_WAIT_FOREVER   ((uint32_t)-1)

/**@} */

/** @name OSPI Driver Transaction State
 */
/**@{ */
/**
 * \brief Indicates that the transaction is IDLE
 */
#define OSPI_TRANS_IDLE     ((uint32_t)0)
/**
 * \brief Indicates the Read transaction
 */
#define OSPI_TRANS_READ     ((uint32_t)1)
/**
 * \brief Indicates the Write transaction
 */
#define OSPI_TRANS_WRITE     ((uint32_t)2)

/**@} */

/**
 *  \anchor OSPI_PHY_Control_Mode
 *  \name OSPI PHY Control Mode
 * 
 *
 *  Controls the bypass mode of the Initiator and Target DLLs.
 *  If this bit is set, the bypass mode is intended to be used only for debug.
 *  0h = Initiator operational mode
 *  DLL works in normal mode of operation where the Target delay line
 *  settings are used as fractional delay of the Initiator delay line encoder
 *  reading of the number of delays in one cycle.
 *  1h = Bypass mode
 *  Initiator DLL is disabled with only 1 delay element in its delay line.
 *  The Target delay lines decode delays in absolute delay elements
 *  rather than as fractional delays.
 *
 *  @{
 */
#define OSPI_FLASH_CFG_PHY_MASTER_CONTROL_REG_PHY_MASTER_MODE           (0U)
#define OSPI_FLASH_CFG_PHY_MASTER_CONTROL_REG_PHY_BYPASS_MODE           (1U)
/** @} */


/**
 *  \anchor OSPI_PHY_DLL_Lock
 *  \name OSPI PHY DLL Lock
 *
 * Determines if the Initiator delay line locks on a full cycle or half cycle
 * of delay. This bit need not be written by software. Force DLL lock mode with this setting.
 *
 *  @{
 */
#define OSPI_PHY_DLL_FULL_CYCLE_LOCK                   ((uint16_t) 0U)
#define OSPI_PHY_DLL_HALF_CYCLE_LOCK                   ((uint16_t) 1U)
/** @} */

/**
 *  \anchor OSPI_Flash_Attack_Vector
 *  \name OSPI Flash Attack Vector
 *
 * Indicates the size of the Attack Vector Array
 *
 *  @{
 */
#define OSPI_FLASH_ATTACK_VECTOR_SIZE       (128U)
/** @} */

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
typedef uint32_t (*OSPI_Clock_getTicks)(void);

/**
 *  \brief  The definition of a micro seconds to ticks function used by
 *  the QSPI driver to get ticks from microseconds
 *
 *  \param usecs                        Micro Seconds
 *
 *  \return Returns system ticks in 32-bit unsigned int format
 *
 */
typedef uint32_t (*OSPI_Clock_usecToTicks)(uint64_t usecs);

/**
 *  \brief Sleep for user specified usecs
 *
 *  \param usecs                        Micro Seconds
 *
 */
typedef void (*OSPI_Clock_usec)(uint32_t usecs);

/**
 *  \brief  The definition of a interrupt completion callback function used by
 *  the QSPI driver
 *
 *  \param args                         Void Pointer
 */
/* Type definition for the callback function*/
typedef void (*OSPI_lld_InterruptCallback)(void* args);

/**
 *  \brief  The definition of a dma read completion callback function used by
 *  the QSPI driver
 *
 *  \param args                         Void Pointer
 */
typedef void (*OSPI_lld_dma_readCompleteCallback)(void* args);

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
/**
 *  \brief Data structure used with \ref OSPI_Transfers - #OSPI_lld_readDirect, #OSPI_lld_writeDirect,
 *         #OSPI_lld_readIndirect, #OSPI_lld_writeIndirect
 *
 *  It indicates how many \ref OSPI_FrameFormat frames are sent and received
 *  from and to the buffer buf.
 */
typedef struct
{
    uint32_t                count;
    /**< [IN] Number of frames for this transaction */
    void                   *buf;
    /**< [IN] void * to a buffer to receive/send data */
    uint32_t                addrOffset;
    /**< [IN] Address offset to write to an OSPI flash device. */
    uint32_t                status;
    /**< [OUT] \ref OSPI_TransferStatus code set by \ref OSPI_Transfers */
    uint32_t                dataLen;
    /**< [IN] Length of response buffer */
    uint32_t                transferOffset;
    /**< [IN] Holds the offset for the current transfer */
    uint32_t                transferTimeout;
    /**< [IN] Timeout of the specific transfer */
    uint32_t                state;
    /**< [IN] Transaction State */
} OSPI_Transaction;

/** \brief OSPI Instance Write Command Structure */

typedef struct
{
    uint8_t cmd;
    /**< [IN] Command Opcode */
    uint32_t cmdAddr;
    /**< [IN] Address required by the command. Usually needed in writing to flash registers.
    Should be initialized to #OSPI_CMD_INVALID_ADDR if not used. */
    uint8_t numAddrBytes;
    /**< [IN] Number of address bytes used to send cmd address */
    void *txDataBuf;
    /**< [IN] Buffer containing command data */
    uint32_t txDataLen;
    /**< [IN] Length of response buffer */
    uint32_t writeTimeout;
    /**< [IN] Timeout of the specific write */
} OSPI_WriteCmdParams;

/** \brief OSPI Instance Read Command Structure */

typedef struct
{
    uint8_t cmd;
    /**< [IN] Command Opcode */
    uint32_t cmdAddr;
    /**< Address required by the command. Usually needed in reading flash registers.
    Should be initialized to #OSPI_CMD_INVALID_ADDR if not used. */
    uint8_t numAddrBytes;
    /**< [IN] Number of address bytes used to send cmd address */
    uint8_t dummyBits;
    /**< [IN] Number dummyClks needed for the command */
    void *rxDataBuf;
    /**< [OUT] Buffer to store response from flash */
    uint16_t rxDataLen;
    /**< [IN] Length of response buffer */
    uint32_t readTimeout;
    /**< [IN] Timeout of the specific read */
} OSPI_ReadCmdParams;

/**
 *  \brief OSPI Address Region
 *
 *  OSPI Address Region will be a part of the attributes. It is used while using DMA Copy
 *  to check if the destination address is a region not accessible to DMA. This data is usually SOC
 *  specific and is filled by SysConfig.
 *
 */
typedef struct
{
    uint32_t regionStartAddr;
    /**< Start address of the region */
    uint32_t regionSize;
    /**< Size of the region */

} OSPI_AddrRegion;

/**
 *  \brief OSPI Parameters
 *
 *  OSPI Parameters are used to with the #OSPI_open() call. Default values for
 *  these parameters are set using #OSPI_Params_init().
 *
 *  If NULL is passed for the parameters, #OSPI_open() uses default parameters.
 *
 *  \sa #OSPI_Params_init()
 */
typedef struct
{
    int32_t ospiDmaChIndex;
    /* Index of Channel used by OSPI DMA Driver. This index will be set by SysCfg according to the DMA driver chosen.
     * The OSPI driver uses this index to do an \ref OSPI_dmaOpen inside the \ref OSPI_open if the DMA mode is enabled
     */
} OSPI_Params;


/**
 *  \brief OSPI PHY Tuning Window Parameters
 *
 *  These are input window parameters for OSPI PHY tuning algorithm. This data is usually SOC
 *  specific and is filled by SysConfig.
 *
 */
typedef struct
{
    int32_t txDllLowWindowStart;
    int32_t txDllLowWindowEnd;
    int32_t txDllHighWindowStart;
    int32_t txDllHighWindowEnd;
    int32_t rxLowSearchStart;
    int32_t rxLowSearchEnd;
    int32_t rxHighSearchStart;
    int32_t rxHighSearchEnd;
    int32_t txLowSearchStart;
    int32_t txLowSearchEnd;
    int32_t txHighSearchStart;
    int32_t txHighSearchEnd;
    int32_t txDLLSearchOffset;
    uint32_t rxTxDLLSearchStep;
    uint32_t rdDelayMin;
    uint32_t rdDelayMax;
} OSPI_PhyWindowParams;

/**
 *  \brief OSPI PHY Configuration
 *
 *  Global structure to provide input to OSPI PHY tuining algorithm with either default or
 *  fast boot tuning paramters. It maintains the essential PHY configurations parameters.
 *  This data is usually SOC specific and is filled by SysConfig.
 *
 */
typedef struct
{
    uint32_t phaseDelayElement;
    uint32_t phyControlMode;
    uint32_t dllLockMode;
    OSPI_PhyWindowParams tuningWindowParams;
} OSPI_PhyConfiguration;

/**
 *  \brief OSPI driver initialization object
 */
typedef struct
{
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
    /**< DMA Channel Index for OSPI DMA Driver*/
    OSPI_DmaHandle  ospiDmaHandle;
    /**< DMA Handle */
    OSPI_DmaChConfig ospiDmaChConfig;
    /**< DMA Configuration for this instance. */
    OSPI_PhyConfiguration  phyConfiguration;
    /**< OSPI PHY configuration params */
} OSPILLD_InitObject, *OSPILLD_InitHandle;

/**
 *  \brief OSPI driver object
 */
typedef struct
{
    /*
     * SOC configuration
     */
    uint32_t                baseAddr;
    /**< Peripheral base address */

    /*
     * User params
     */
    const OSPI_Params *openParams;
    uint32_t transferMode;
    /**< Polling, Blocking or Callback mode. Refer \ref OSPI_TransferMode */
    uint32_t protocol;
    /**< Protocol for OSPI reading/writing. 32 bit integer with
     * byte0 -> data lines
     * byte1 -> addr lines
     * byte2 -> cmd lines
     * byte3 -> STR/DTR (0 = STR, 1 = DTR)
     * */
    uint32_t rdDummyCycles;
    /**< Number of dummy cycles needed for read */
    uint32_t cmdDummyCycles;
    /**< Number of dummy cycles needed for cmd */
    uint32_t rdDataCapDelay;
    /**< Read data capture delays needed */
    uint32_t phyRdDataCapDelay;
    /**< Read data capture delays needed when phy is enabled*/
    uint32_t numAddrBytes;
    /**< Number of bytes used to represent address to be sent to flash.
    This is the actual number of bytes used. The code to be programmed to
    registers is this value-1. That is, for 4-byte addressing mode, register
    should be programmed as 3 */
    uint32_t cmdExtType;
    /**< In dual byte opcode mode, the extended opcode can vary depending on flash
    This variable should be populated from the flash driver using the \ref OSPI_lld_setCmdExtType API */
    uint32_t                phyEnableSuccess;
    /**< This has to be set from the flash driver if the PHY tuning completed successfully */

    OSPI_Transaction *currTrans;
    /**< Pointer to current transaction struct */

    OSPI_Transaction trans;
    /**< Pointer to current transaction struct */

    OSPILLD_InitHandle hOspiInit;
    /**< [IN] Initialization parameters of OSPI instance */

    /*
     * State variables
     */
    uint32_t            state;

    void*               args;
    /**< Pointer to be used by application to store miscellaneous data.*/

    OSPI_Clock_getTicks Clock_getTicks;
    /**< Function Pointer for the OSPI clock get ticks >*/
    OSPI_Clock_usecToTicks Clock_usecToTicks;
    /**< Function Pointer for the OSPI micro second to tick conversion >*/
    OSPI_Clock_usec Clock_usleep;

    OSPI_lld_InterruptCallback interruptCallback;
    /* Interrupt Callback to be registered by the application*/
    OSPI_lld_dma_readCompleteCallback readCompleteCallback;
    /* DMA Readcallback function*/

} OSPILLD_Object, *OSPILLD_Handle;

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
 *  \brief  This function opens a given OSPI peripheral
 *
 *  \pre    OSPI controller has been initialized using #OSPI_init()
 *
 *  \param  hOspi   OSPI Handle
 *
 *  \return A #OSPILLD_Handle on success or a NULL on an error or if it has been
 *          opened already
 *
 *  \sa     #OSPI_init()
 *  \sa     #OSPI_close()
 */
int32_t OSPI_lld_init(OSPILLD_Handle hOspi);

/**
 *  \brief  This function opens a given OSPI peripheral in DMA mode.
 *
 *  \pre    OSPI controller has been initialized using #OSPI_init()
 *
 *  \param  hOspi   OSPI Handle
 *
 *  \return A #OSPILLD_Handle on success or a NULL on an error or if it has been
 *          opened already
 *
 *  \sa     #OSPI_init()
 *  \sa     #OSPI_close()
 */
int32_t OSPI_lld_initDma(OSPILLD_Handle hOspi);

/**
 *  \brief  Function to close a OSPI peripheral specified by the OSPI handle
 *
 *  \pre    #OSPI_open() has to be called first
 *
 *  \param  hOspi      #OSPILLD_Handle returned from #OSPI_open()
 *
 *  \sa     #OSPI_open()
 */
int32_t OSPI_lld_deInit(OSPILLD_Handle hOspi);

/**
 *  \brief  Function to close a OSPI peripheral specified by the OSPI handle in DMA mode
 *
 *  \pre    #OSPI_open() has to be called first
 *
 *  \param  hOspi      #OSPILLD_Handle returned from #OSPI_open()
 *
 *  \sa     #OSPI_open()
 */
int32_t OSPI_lld_deInitDma(OSPILLD_Handle hOspi);

/**
 *  \ingroup DRV_OSPI_LLD_MODULE
 *  \defgroup DRV_OSPI_LLD_READ_WRITE_MODULE OSPI Read and Write API's
 *
 *   These APIs try are used for the phy tuning 
 * 
 *  @{
 */

/**
 *  \anchor OSPI_Transfers
 *  \name Different OSPI Transfer functions
 *
 *  @{
 */
/**
 *  \brief  Function to perform direct reads from the flash using DAC controller
 *
 *
 *  \param  handle      #OSPILLD_Handle returned from #OSPI_open()
 *  \param  trans       Pointer to a #OSPI_Transaction
 *
 *  \return #OSPI_SYSTEM_SUCCESS on successful read; else error on failure
 *
 *  \sa     #OSPI_open
 */
int32_t OSPI_lld_readDirect(OSPILLD_Handle handle, OSPI_Transaction *trans);

/**
 *  \brief  Function to perform direct reads from the flash using DAC controller in DMA mode
 *
 *
 *  \param  handle      #OSPILLD_Handle returned from #OSPI_open()
 *  \param  trans       Pointer to a #OSPI_Transaction
 *
 *  \return #OSPI_SYSTEM_SUCCESS on successful read; else error on failure
 *
 *  \sa     #OSPI_open
 */
int32_t OSPI_lld_readDirectDma(OSPILLD_Handle handle, OSPI_Transaction *trans);

/**
 *  \brief  Function to perform indirect reads from the flash using INDAC controller
 *
 *
 *  \param  handle      #OSPILLD_Handle returned from #OSPI_open()
 *  \param  trans       Pointer to a #OSPI_Transaction
 *
 *  \return #OSPI_SYSTEM_SUCCESS on successful read; else error on failure
 *
 *  \sa     #OSPI_open
 */
int32_t OSPI_lld_readIndirect(OSPILLD_Handle handle, OSPI_Transaction *trans);

/**
 *  \brief  Function to perform direct writes to the flash using DAC controller
 *
 *
 *  \param  handle      #OSPILLD_Handle returned from #OSPI_open()
 *  \param  trans       Pointer to a #OSPI_Transaction
 *
 *  \return #OSPI_SYSTEM_SUCCESS on successful write; else error on failure
 *
 *  \sa     #OSPI_open
 */
int32_t OSPI_lld_writeDirect(OSPILLD_Handle handle, OSPI_Transaction *trans);

/**
 *  \brief  Function to perform indirect writes to the flash using INDAC controller
 *
 *
 *  \param  handle      #OSPILLD_Handle returned from #OSPI_open()
 *  \param  trans       Pointer to a #OSPI_Transaction
 *
 *  \return #OSPI_SYSTEM_SUCCESS on successful write; else error on failure
 *
 *  \sa     #OSPI_open
 */
int32_t OSPI_lld_writeIndirect(OSPILLD_Handle handle, OSPI_Transaction *trans);
/** @} */

/**
 *  \brief  Function to send specific commands and receive related data from flash
 *
 *
 *  \param  handle      #OSPILLD_Handle returned from #OSPI_open()
 *  \param  rdParams    Pointer to a #OSPI_ReadCmdParams
 *
 *  \return #OSPI_SYSTEM_SUCCESS if command read was successful; else error on failure
 *
 *  \sa     #OSPI_open
 */
int32_t OSPI_lld_readCmd(OSPILLD_Handle handle, OSPI_ReadCmdParams *rdParams);

/**
 *  \brief  Function to send specific commands and related data to flash
 *
 *
 *  \param  handle      #OSPILLD_Handle returned from #OSPI_open()
 *  \param  wrParams    Pointer to a #OSPI_WriteCmdParams
 *
 *  \return #OSPI_SYSTEM_SUCCESS if command write was successful; else error on failure
 *
 *  \sa     #OSPI_open
 */
int32_t OSPI_lld_writeCmd(OSPILLD_Handle handle, OSPI_WriteCmdParams *wrParams);

/** @} */

/**
 *  \ingroup DRV_OSPI_LLD_MODULE
 *  \defgroup DRV_OSPI_LLD_CALLBACK_API_MODULE OSPI Callback Initialization
 *
 *   These APIs try are used for the phy tuning 
 * 
 *  @{
 */

/**
 *  \brief   QSPI ISR for Read and Write Functionality.
 *
 *  \param  args       #OSPILLD_Handle of OSPI instance.
 */

void OSPI_lld_isr(void* args);

/**
 *  \brief   QSPI Read complete callback for DMA mode
 *
 *  \param  args       #OSPILLD_Handle of OSPI instance.
 */
void OSPI_lld_readCompleteCallback(void* args);

/** @} */

/**
 *  \ingroup DRV_OSPI_LLD_MODULE
 *  \defgroup DRV_OSPI_LLD_TRANS_INIT_API_MODULE Transaction Initialization
 *
 *   These APIs try are used for the phy tuning 
 * 
 *  @{
 */

/**
 *  \brief  Function to initialize the #OSPI_Transaction structure
 *
 *
 *  \param  trans Pointer to a #OSPI_Transaction
 *
 */
void OSPI_lld_Transaction_init(OSPI_Transaction *trans);

/**
 *  \brief  Function to initialize the #OSPI_ReadCmdParams structure
 *
 *
 *  \param  rdParams Pointer to a #OSPI_ReadCmdParams
 *
 */
void OSPI_lld_readCmdParams_init(OSPI_ReadCmdParams *rdParams);

/** @} */

/**
 *  \brief  Function to initialize the #OSPI_WriteCmdParams structure
 *
 *
 *  \param  wrParams Pointer to a #OSPI_WriteCmdParams
 *
 */
void OSPI_lld_writeCmdParams_init(OSPI_WriteCmdParams *wrParams);

/**
 *  \ingroup DRV_OSPI_LLD_MODULE
 *  \defgroup DRV_OSPI_LLD_USER_INTERFACE_API_MODULE User Interface API's
 *
 *   These APIs try are used for the phy tuning 
 * 
 *  @{
 */

/**
 *  \brief  This function returns the input clk frequency OSPI was programmed at
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *
 *  \return OSPI RCLK in Hertz
 */
uint32_t OSPI_lld_getInputClk(OSPILLD_Handle handle);

/**
 *  \brief  This function checks if the Direct Access Controller is enabled
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *
 *  \return 1 if DAC is enabled, 0 otherwise
 */
uint32_t OSPI_lld_isDacEnable(OSPILLD_Handle handle);

/**
 *  \brief  This function checks if DMA is enabled for reads
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *
 *  \return 1 if DMA is enabled, 0 otherwise
 */
uint32_t OSPI_lld_isDmaEnable(OSPILLD_Handle handle);

/**
 *  \brief  This function checks if interrupts are enabled
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *
 *  \return 1 if interrupts are enabled, 0 otherwise
 */
uint32_t OSPI_lld_isIntrEnable(OSPILLD_Handle handle);

/**
 *  \brief  This function checks if the OSPI PHY controller is enabled
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *
 *  \return 1 if PHY is enabled, 0 otherwise
 */
uint32_t OSPI_lld_isPhyEnable(OSPILLD_Handle handle);

/**
 *  \brief  This function enables the Dual Data Rate (DDR)
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *
 *  \return #OSPI_SYSTEM_SUCCESS if DDR successfully enabled, else error on failure
 */
int32_t OSPI_lld_enableDDR(OSPILLD_Handle handle);

/**
 *  \brief  This function enables the Single Data Rate (SDR)
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *
 *  \return #OSPI_SYSTEM_SUCCESS if SDR successfully enabled, else error on failure
 */
int32_t OSPI_lld_enableSDR(OSPILLD_Handle handle);

/**
 *  \brief  This function sets DDR bit in INSTR_RD register for RD commands
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *
 *  \return #OSPI_SYSTEM_SUCCESS if SDR successfully enabled, else error on failure
 */
int32_t OSPI_lld_enableDdrRdCmds(OSPILLD_Handle handle);

/**
 *  \brief  This function sets read data capture cycles in the OSPI controller
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle             An #OSPILLD_Handle returned from an #OSPI_open()
 *  \param  rdDataCapDelay     Number of read data capture cycles to be programmed
 *
 *  \return #OSPI_SYSTEM_SUCCESS if read data capture cycles successfully set, else error on failure
 */
int32_t OSPI_lld_setRdDataCaptureDelay(OSPILLD_Handle handle, uint32_t rdDataCapDelay);

/**
 *  \brief  This function set the number of bytes used to send address while reading or writing to flash memory
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle           An #OSPILLD_Handle returned from an #OSPI_open()
 *  \param  numAddrBytes     Number of address bytes to be used while sending addresses.
 *
 */
void OSPI_lld_setNumAddrBytes(OSPILLD_Handle handle, uint32_t numAddrBytes);

/**
 *  \brief  This function sets the block size and page size of the flash to the device size register in OSPI
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle           An #OSPILLD_Handle returned from an #OSPI_open()
 *  \param  pageSize         Page size of the flash in bytes
 *  \param  blkSize          Block size of the flash in bytes
 *
 */
void OSPI_lld_setDeviceSize(OSPILLD_Handle handle, uint32_t pageSize, uint32_t blkSize);

/**
 *  \brief  This function sets appropriate dummy cycles to be used while sending STIG commands to flash
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle          An #OSPILLD_Handle returned from an #OSPI_open()
 *  \param  cmdDummyCycles  Dummy cycles to be used for STIG cmd
 *
 */
void OSPI_lld_setCmdDummyCycles(OSPILLD_Handle handle, uint32_t cmdDummyCycles);

/**
 *  \brief  This function sets appropriate dummy cycles for flash read
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *  \param  dummyCycles     Number of dummy cycles
 *
 */
void OSPI_lld_setReadDummyCycles(OSPILLD_Handle handle, uint32_t dummyCycles);

/**
 *  \brief  This function sets appropriate dummy cycles for flash write
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *  \param  dummyCycles     Number of dummy cycles
 *
 */
void OSPI_lld_setWriteDummyCycles(OSPILLD_Handle handle, uint32_t dummyCycles);

/**
 *  \brief  This function sets the phyEnableSuccess field in \ref OSPILLD_Object. Has to be called from flash driver
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *  \param  success Set this to 1 if phy enable was successful. 0 otherwise
 *
 */
void OSPI_lld_setPhyEnableSuccess(OSPILLD_Handle handle, uint32_t success);

/**
 *  \brief  This function sets mode bits in the mode bit field of OSPI config register.
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle   An #OSPILLD_Handle returned from an #OSPI_open()
 *  \param  modeBits Number of mode bits to be set
 *
 */
void OSPI_lld_setModeBits(OSPILLD_Handle handle, uint32_t modeBits);

/**
 *  \brief  This function enables mode bits transmission while sending CMDs
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *
 */
void OSPI_lld_enableModeBitsCmd(OSPILLD_Handle handle);

/**
 *  \brief  This function enables mode bits transmission while reading
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *
 */
void OSPI_lld_enableModeBitsRead(OSPILLD_Handle handle);

/**
 *  \brief  This function fetches the phyEnableSuccess field in \ref OSPILLD_Object.
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *
 *  \return 1 if phyEnable was successful, 0 otherwise.
 */
uint32_t OSPI_lld_getPhyEnableSuccess(OSPILLD_Handle handle);

/**
 *  \brief  This function returns the current protocol for which the transfer lines in
 *          OSPI driver is configured for.
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *
 *  \return protocol Protocol being used
 *
 */
uint32_t OSPI_lld_getProtocol(OSPILLD_Handle handle);

/**
 *  \brief  This function sets the number of transfer lines in the OSPI driver to
 *          set the requested protocol
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *  \param  protocol Protocol to be used
 *
 */
void OSPI_lld_setProtocol(OSPILLD_Handle handle, uint32_t protocol);

/**
 *  \brief  This function sets OSPI controller to use dual byte opcodes
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *
 */
void OSPI_lld_setDualOpCodeMode(OSPILLD_Handle handle);

/**
 *  \brief  This function sets OSPI controller to not use dual byte opcodes
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *
 */
void OSPI_lld_clearDualOpCodeMode(OSPILLD_Handle handle);

/**
 *  \brief  This function sets the opcodes for reading and page programming the flash
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle      An #OSPILLD_Handle returned from an #OSPI_open()
 *  \param  readCmd     Command opcode to be used for reading from the flash
 *  \param  pageProgCmd Command opcode to be used for writing to / programming the flash
 *
 */
void OSPI_lld_setXferOpCodes(OSPILLD_Handle handle, uint8_t readCmd, uint8_t pageProgCmd);

/**
 *  \brief  This function sets the type of command extension used in dual byte opcode mode
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle      An #OSPILLD_Handle returned from an #OSPI_open()
 *  \param  cmdExtType  Type of command extension used. As of now only two types are supported - REPEAT and INVERSE
 *
 */
void OSPI_lld_setCmdExtType(OSPILLD_Handle handle, uint32_t cmdExtType);

/**
 *  \brief  This function enables the Direct Access Mode
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *
 *  \return #OSPI_SYSTEM_SUCCESS on success, #OSPI_SYSTEM_FAILURE otherwise
 */
int32_t OSPI_lld_enableDacMode(OSPILLD_Handle handle);

/**
 *  \brief  This function disables the Direct Access Mode
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *
 *  \return #OSPI_SYSTEM_SUCCESS on success, #OSPI_SYSTEM_FAILURE otherwise
 */
int32_t OSPI_lld_disableDacMode(OSPILLD_Handle handle);

/**
 *  \brief  This function gets the SOC mapped data base address of the flash
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *
 *  \return Data BaseAddress of the flash
 */
uint32_t OSPI_lld_getFlashDataBaseAddr(OSPILLD_Handle handle);

/** @} */

/**
 *  \ingroup DRV_OSPI_LLD_MODULE
 *  \defgroup DRV_OSPI_LLD_PHY_API_MODULE Generic Phy layer API.
 *
 *   These APIs try are used for the phy tuning 
 * 
 *  @{
 */

/**
 *  \brief  This function tunes the OSPI PHY for DDR mode to set optimal PHY parameters
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *
 *  \param  flashOffset Offset of the flash at which the PHY tuning data is present
 *
 *  \return #OSPI_SYSTEM_SUCCESS on success, #OSPI_SYSTEM_FAILURE otherwise
 */
int32_t OSPI_lld_phyTuneDDR(OSPILLD_Handle handle, uint32_t flashOffset);

/**
 *  \brief  This function tunes the OSPI PHY for SDR mode to set optimal PHY parameters
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *
 *  \param  flashOffset Offset of the flash at which the PHY tuning data is present
 *
 *  \return #OSPI_SYSTEM_SUCCESS on success, #OSPI_SYSTEM_FAILURE otherwise
 */
int32_t OSPI_lld_phyTuneSDR(OSPILLD_Handle handle, uint32_t flashOffset);

/**
 *  \brief  This function takes a 4x128x128 array and fills it with TX RX DLL data for graphing purpose
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *
 *  \param  flashOffset Offset of the flash at which the PHY tuning data is present
 *
 *  \param  arrays A 4x128x128 array. In this, the tx rx dll data for 4 different read delay values will be stored. This can be later used for graphing
 *
 *  \return #OSPI_SYSTEM_SUCCESS on success, #OSPI_SYSTEM_FAILURE otherwise
 */
int32_t OSPI_lld_phyTuneGrapher(OSPILLD_Handle handle, uint32_t flashOffset, uint8_t arrays[5][128][128]);

/**
 *  \brief  This function returns the address to the attack vector buf required for tuning the PHY
 *
 *  \param  tuningData Address of the tuningData array
 *
 *  \param  tuningDataSize Size of the tuningData array
 */
void OSPI_lld_phyGetTuningData(uint32_t *tuningData, uint32_t *tuningDataSize);

/**
 *  \brief  This function checks if the attack vector, or the data used for tuning the PHY is present at an offset in the flash
 *
 *  \param  handle An #OSPILLD_Handle returned from an #OSPI_open()
 *
 *  \param  offset Offset in flash to check for tuningData
 *
 *  \return #OSPI_SYSTEM_SUCCESS on success, #OSPI_SYSTEM_FAILURE otherwise
 */
int32_t OSPI_lld_phyReadAttackVector(OSPILLD_Handle handle, uint32_t offset);

/**
 *  \brief  This function enables the PHY
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *
 *  \return #OSPI_SYSTEM_SUCCESS on success, #OSPI_SYSTEM_FAILURE otherwise
 */
int32_t OSPI_lld_enablePhy(OSPILLD_Handle handle);

/**
 *  \brief  This function disables the PHY
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *
 *  \return #OSPI_SYSTEM_SUCCESS on success, #OSPI_SYSTEM_FAILURE otherwise
 */
int32_t OSPI_lld_disablePhy(OSPILLD_Handle handle);

/**
 *  \brief  This function enables the PHY Pipeline
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *
 *  \return #OSPI_SYSTEM_SUCCESS on success, #OSPI_SYSTEM_FAILURE otherwise
 */
int32_t OSPI_lld_enablePhyPipeline(OSPILLD_Handle handle);

/**
 *  \brief  This function disables the PHY Pipeline
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *
 *  \return #OSPI_SYSTEM_SUCCESS on success, #OSPI_SYSTEM_FAILURE otherwise
 */
int32_t OSPI_lld_disablePhyPipeline(OSPILLD_Handle handle);

/** @} */

/**
 *  \ingroup DRV_OSPI_LLD_MODULE
 *  \defgroup DRV_OSPI_NOR_FLASH_LLD_API_MODULE Generic NOR Flash API for single pin mode
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
 *  \param  handle  An #OSPILLD_Handle returned from an #OSPI_open()
 *
 *  \return #OSPI_SYSTEM_SUCCESS on success, #OSPI_SYSTEM_FAILURE otherwise
 */
int32_t OSPI_lld_norFlashInit1s1s1s(OSPILLD_Handle handle);

/**
 *  \brief  This function sets up internal bookkeeping variables for read, write
 *          and erase commands. This API has to be called immediately before
 *          \ref OSPI_lld_norFlashInit1s1s1s
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  rdCmd     Command to be used in single mode read
 *  \param  wrCmd     Command to be used in single mode write/page program
 *  \param  eraseCmd  Command to be used to erase (block or sector)
 *
 */
void OSPI_lld_norFlashSetCmds(uint8_t rdCmd, uint8_t wrCmd, uint8_t eraseCmd);


/**
 *  \brief  This function tries to read the JEDEC ID from the NOR flash connected to the OSPI peripheral
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle         An #OSPILLD_Handle returned from an #OSPI_open()
 *  \param  manufacturerId Pointer to a uint32_t variable. This will be filled with the manufacturer ID on success
 *  \param  deviceId       Pointer to a uint32_t variable. This will be filled with the device ID on success
 *
 *  \return #OSPI_SYSTEM_SUCCESS on success, #OSPI_SYSTEM_FAILURE otherwise
 */
int32_t OSPI_lld_norFlashReadId(OSPILLD_Handle handle, uint32_t *manufacturerId, uint32_t *deviceId);

/**
 *  \brief  This function writes data to the flash at a specified offset
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle    An #OSPILLD_Handle returned from an #OSPI_open()
 *  \param  offset    Offset at which the data is to be written
 *  \param  buf       Buffer which has the data to be written to the flash
 *  \param  len       Number of bytes to be written to the flash
 *
 *  \return #OSPI_SYSTEM_SUCCESS on success, #OSPI_SYSTEM_FAILURE otherwise
 */
int32_t OSPI_lld_norFlashWrite(OSPILLD_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len);

/**
 *  \brief  This function reads data from the flash from a specified offset in DAC mode
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle    An #OSPILLD_Handle returned from an #OSPI_open()
 *  \param  offset    Offset at which the data is to be read from
 *  \param  buf       Buffer to which data will be written into
 *  \param  len       Number of bytes to be read from the flash
 *
 *  \return #OSPI_SYSTEM_SUCCESS on success, #OSPI_SYSTEM_FAILURE otherwise
 */
int32_t OSPI_lld_norFlashRead(OSPILLD_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len);

/**
 *  \brief  This function reads data from the flash from a specified offset in INDAC mode
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle    An #OSPILLD_Handle returned from an #OSPI_open()
 *  \param  offset    Offset at which the data is to be read from
 *  \param  buf       Buffer to which data will be written into
 *  \param  len       Number of bytes to be read from the flash
 *
 *  \return #OSPI_SYSTEM_SUCCESS on success, #OSPI_SYSTEM_FAILURE otherwise
 */
int32_t OSPI_lld_norFlashReadIndirect(OSPILLD_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len);

/**
 *  \brief  This function reads SFDP table from the flash from a specified offset
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle    An #OSPILLD_Handle returned from an #OSPI_open()
 *  \param  offset    Offset at which the data is to be read from
 *  \param  buf       Buffer to which data will be written into
 *  \param  len       Number of bytes to be read from the flash
 *
 *  \return #OSPI_SYSTEM_SUCCESS on success, #OSPI_SYSTEM_FAILURE otherwise
 */
int32_t OSPI_lld_norFlashReadSfdp(OSPILLD_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len);

/**
 *  \brief  This function erases 1 block of data starting from a provided address
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle    An #OSPILLD_Handle returned from an #OSPI_open()
 *  \param  address   Address of the data block to be erased. This address should be block aligned.
 *
 *  \return #OSPI_SYSTEM_SUCCESS on success, #OSPI_SYSTEM_FAILURE otherwise
 */
int32_t OSPI_lld_norFlashErase(OSPILLD_Handle handle, uint32_t address);

/**
 *  \brief  This function configures reset functionality
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle    An #OSPI_Handle returned from an #OSPI_open()
 *  \param  config    reset config
 *
 *  \return #OSPI_SYSTEM_SUCCESS on success, #OSPI_SYSTEM_FAILURE otherwise
 */
int32_t OSPI_lld_configResetPin(OSPILLD_Handle handle, uint32_t config);

/**
 *  \brief  Configures baud divider
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle      An #OSPI_Handle returned from an #OSPI_open()
 *  \param  baud        baudrate from 2 to 32 and divisible by 2
 *
 *  \return #OSPI_SYSTEM_SUCCESS on success, #OSPI_SYSTEM_FAILURE otherwise
 */
int32_t OSPI_lld_configBaudrate(OSPILLD_Handle handle, uint32_t baud);

/**
 *  \brief  Return value of baudrate that is programmed in IP register
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle      An #OSPI_Handle returned from an #OSPI_open()
 *  \param  baudDiv     pointer to memory into which baudrate will be written
 *
 *  \return #OSPI_SYSTEM_SUCCESS on success, #OSPI_SYSTEM_FAILURE otherwise
 */
int32_t OSPI_lld_readBaudRateDivFromReg(OSPILLD_Handle handle, uint32_t *baudDiv);

/**
 *  \brief  Return value of baudrate that is saved in OSPI Object
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  handle      An #OSPILLD_Handle returned from an #OSPI_init()
 *  \param  baudDiv     pointer to memory into which baudrate will be written
 *
 *  \return #OSPI_SYSTEM_SUCCESS on success, #OSPI_SYSTEM_FAILURE otherwise
 */
int32_t OSPI_lld_getBaudRateDivFromObj(OSPILLD_Handle handle, uint32_t *baudDiv);

/**
 *  \brief  This function activates the RESET pin feature
 *
 *  \pre    OSPI controller has been opened using #OSPI_open()
 *
 *  \param  hOspi        An #OSPILLD_Handle returned from an #OSPI_open()
 *  \param  pinStatus    pinStatus - HIGH/LOW
 *
 *  \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t OSPI_lld_setResetPinStatus(OSPILLD_Handle hOspi, uint32_t pinStatus);

/**
 * \brief API to open an OSPI DMA channel
 *
 * This API will open a DMA Channel using the appropriate DMA driver callbacks and the registered via Sysconfig
 *
 * \param  hOspi        An #OSPILLD_Handle returned from an #OSPI_open()
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t OSPI_dmaOpen(OSPILLD_Handle hOspi);

/**
 * \brief API to close an OSPI DMA channel
 *
 * This API will open a DMA Channel using the appropriate DMA driver callbacks registered via Sysconfig
 *
 * \param handle  [in] An #OSPILLD_Handle returned from an #OSPI_open()
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t OSPI_dmaClose(OSPILLD_Handle handle);

/**
 * \brief API to do a DMA Copy using appropriate DMA Channel opened
 *
 * This API will open a DMA Channel using the appropriate DMA driver callbacks registered via Sysconfig
 *
 * \param handle        [in] An #OSPILLD_Handle returned from an #OSPI_open()
 * \param dst           [in] Destination address to which the data is to be copied
 * \param src           [in] Source address from which the data is to be copied
 * \param length        [in] Data length
 * \param timeout 		[in] Timeout for the transaction
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t OSPI_dmaCopy(OSPILLD_Handle handle, void* dst, void* src, uint32_t length,uint32_t timeout);

/**
 * \brief API to get the DMA Interrupt status
 *
 * This API will retrieve the interrrupt status of the DMA Channel

 * \param handle        [in] An #OSPILLD_Handle returned from an #OSPI_open()
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t OSPI_isDmaInterruptEnabled(OSPILLD_Handle handle);

/**
 * \brief Validates a tuning point for the OSPI physical interface
 *
 * This function verifies if the current OSPI physical interface configuration
 * (tuning point) is valid by performing a read operation at the specified flash offset.
 *
 * \param hOspi OSPI LLD handle
 * \param flashOffset Flash memory offset to use for the validation read operation
 *
 * \return #SystemP_SUCCESS on success, #SystemP_FAILURE otherwise
 */
int32_t OSPI_lld_phyValidateTuningPoint(OSPILLD_Handle hOspi, uint32_t flashOffset);

/**
 * \brief Checks if OTP validation is enabled for the OSPI controller
 *
 * This function determines whether the OTP (One-Time Programmable) validation
 * feature is currently enabled in the OSPI controller.
 *
 * \param hOspi    Handle to the OSPI controller instance
 *
 * \return uint32_t    Returns 1 if OTP validation is enabled, 0 otherwise
 */
uint32_t OSPI_lld_isValidateOtpEnable(OSPILLD_Handle hOspi);

/**
 *  \brief Sets the operating frequency for the OSPI peripheral
 *
 *  This function configures the OSPI controller to operate at the specified
 *  frequency based on the input clock frequency provided.
 *
 *  \param hOspi OSPI LLD handle
 *  \param inputClkFreq Input clock frequency in Hz
 *
 *  \return SystemP_SUCCESS on success, error code on failure
 */
int32_t OSPI_lld_setFrequency(OSPILLD_Handle hOspi, uint64_t inputClkFreq);

/**
 *  \brief Sets timing delays for the OSPI interface based on input clock frequency
 *
 *  This function configures the appropriate timing delays for the OSPI interface
 *  to ensure reliable communication with external memory devices. The delays are
 *  calculated based on the provided input clock frequency.
 *
 *  \param hOspi OSPI LLD handle
 *  \param inputClkFreq Input clock frequency in Hz
 *
 *  \return SystemP_SUCCESS on success, error code on failure
 */
int32_t OSPI_lld_setDelays(OSPILLD_Handle hOspi, uint32_t inputClkFreq);

/**
 *  \brief Sets the baud rate divider for OSPI communication
 *
 *  This function configures the baud rate divider to control the OSPI clock frequency.
 *  The actual OSPI clock frequency is determined by dividing the input clock frequency
 *  by the specified baud rate divider.
 *
 *  \param hOspi OSPI LLD handle
 *  \param baudRateDiv Baud rate divider value
 *
 *  \return SystemP_SUCCESS on success, error code on failure
 */
int32_t OSPI_lld_setBaudRateDiv(OSPILLD_Handle hOspi, uint32_t baudRateDiv);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef OSPI_LLD_H_ */

