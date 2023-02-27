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
 *  \file v1/mmcsd.h
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
#include <drivers/hw_include/hw_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Determines the type of card is SD. */
#define MMCSD_CARD_SD                   (0U)

/** \brief Determines the type of card is MMC. */
#define MMCSD_CARD_MMC                  (1U)

/** \brief Determines the type of card is MMC. */
#define MMCSD_CARD_EMMC                 (2U)

/* useful in cases where SoC has MMCSD IP but no device attached to peripheral */
#define MMCSD_CARD_TYPE_NO_DEVICE       (3U)

/** \brief  Set bus width of MMCSD. */
#define MMCSD_CMD_SETBUSWIDTH        (MMCSD_CMD_RESERVED + 0U)

/** \brief  Set bus frequency of MMCSD. */
#define MMCSD_CMD_SETFREQUENCY       (MMCSD_CMD_RESERVED + 1U)

/** \brief  Get bus width of MMCSD. */
#define MMCSD_CMD_GETBUSWIDTH        (MMCSD_CMD_RESERVED + 2U)

/** \brief  Get bus frequency of MMCSD. */
#define MMCSD_CMD_GETFREQUENCY       (MMCSD_CMD_RESERVED + 3U)

/** \brief  Get media (SD/eMMC/MMC) parameters */
#define MMCSD_CMD_GETMEDIAPARAMS     (MMCSD_CMD_RESERVED + 4U)

/** \brief  Invalid Mux num */
#define MMCSD_INVALID_MUXNUM            (-1)

/** \brief  Invalid Mux In/Out event num */
#define MMCSD_INVALID_MUX_EVENTNUM      (-1)

/** \brief Card bus width configuration for 1-bit mode. */
#define MMCSD_BUS_WIDTH_1BIT            (1U)

/** \brief Card bus width configuration for 4-bit mode. */
#define MMCSD_BUS_WIDTH_4BIT            (4U)

/** \brief Card bus width configuration for 8-bit mode. */
#define MMCSD_BUS_WIDTH_8BIT            (8U)

/** \brief Card bus frequency configuration for 25 Mbps. */
#define MMCSD_TRANSPEED_25MBPS          (0x32U)

/** \brief Card bus frequency configuration for 50 Mbps. */
#define MMCSD_TRANSPEED_50MBPS          (0x5AU)

/** \brief Card bus frequency configuration for 25 Mbps. */
#define MMCSD_TRANSPEED_DEFAULT         (0x32U)

/** \brief Card bus frequency configuration for 50 Mbps. */
#define MMCSD_TRANSPEED_HS              (0x5AU)

/** \brief Card bus frequency configuration for 25 Mbps. */
#define MMCSD_TRANSPEED_SDR12           (0x32U)

/** \brief Card bus frequency configuration for 50 Mbps. */
#define MMCSD_TRANSPEED_SDR25           (0x5AU)

/** \brief Card bus frequency configuration for 100 Mbps. */
#define MMCSD_TRANSPEED_SDR50           (0xBU)

/** \brief Card bus frequency configuration for 200 Mbps. */
#define MMCSD_TRANSPEED_SDR104          (0x2BU)

/** \brief Card bus frequency configuration for 100 Mbps. */
#define MMCSD_TRANSPEED_DDR50           (0x3BU)

/** \brief Card bus frequency configuration for 200 Mbps. */
#define MMCSD_TRANSPEED_HS200           (0x2BU)

/** \brief MMC any loop back. */
#define MMCSD_LOOPBACK_ANY              (0x0U)

/** \brief MMC internal loop back. */
#define MMCSD_LOOPBACK_INTERNAL         (0x1U)

/** \brief MMC pad loop back. */
#define MMCSD_LOOPBACK_PAD              (0x2U)

/** \brief MMC any operating voltage. */
#define MMCSD_VOLTAGE_ANY               (0x0U)


typedef void* MMCSD_Handle;

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */
/**
 *  \brief MMCSD instance attributes - used during init time
 */
/*!
 *  @brief      A function pointer to configure iodelay.
 */

/** \brief Enumerates the types of lines fore hs mmcsd reset. */
typedef enum hsMmcsdResetLineMask
{
    MMCSD_RESET_LINE_MASK_DATA  = CSL_MMC_SYSCTL_SRD_MASK,
    /**< Data line reset. */
    MMCSD_RESET_LINE_MASK_CMD = CSL_MMC_SYSCTL_SRC_MASK,
    /**< Command line reset. */
    MMCSD_RESET_LINE_MASK_ALL  = CSL_MMC_SYSCTL_SRA_MASK
    /**< Reset of all the lines. */
}MMCSD_resetLineMask;

/** \brief Enumerates the clock activity configurations during wake up . */
typedef enum MMCSD_clkAct_t
{
    MMCSD_CLK_ACT_ICLK_FCLK_OFF = CSL_MMC_SYSCONFIG_CLOCKACTIVITY_NONE,
    /**< Both functional and interface clocks are off. */
    MMCSD_CLK_ACT_FCLK_OFF = CSL_MMC_SYSCONFIG_CLOCKACTIVITY_FUNC,
    /**< Interface clock is on and Functional clock is off. */
    MMCSD_CLK_ACT_ICLK_OFF = CSL_MMC_SYSCONFIG_CLOCKACTIVITY_OCP,
    /**< Interface clock is off and Functional clock is on. */
    MMCSD_CLK_ACT_ICLK_FLCK_ON  = CSL_MMC_SYSCONFIG_CLOCKACTIVITY_BOTH
    /**< Both functional and interface clocks are on. */
}MMCSD_clkAct;

/** \brief Enumerates the standby modes. */
typedef enum MMCSD_standbyMode_t
{
    MMCSD_STANDBY_MODE_FORCE = CSL_MMC_SYSCONFIG_STANDBYMODE_FORCE,
    /**< Force standby mode. */
    MMCSD_STANDBY_MODE_NONE = CSL_MMC_SYSCONFIG_STANDBYMODE_NOIDLE,
    /**< No standby mode. */
    MMCSD_STANDBY_MODE_SMART = CSL_MMC_SYSCONFIG_STANDBYMODE_SMART,
    /**< Smart standby mode. */
    MMCSD_STANDBY_MODE_SMART_WAKEUP =
        CSL_MMC_SYSCONFIG_STANDBYMODE_SMARTWAKE
    /**< Smart standby with wakeup mode. */
}MMCSD_standbyMode;

/** \brief Enumerates macros to configure power on/off. */
typedef enum MMCSD_pwrCtrl_t
{
    MMCSD_PWR_CTRL_ON  = CSL_MMC_HCTL_SDBP_PWRON,
    /**< Power on the controller. */
    MMCSD_PWR_CTRL_OFF = CSL_MMC_HCTL_SDBP_PWROFF
    /**< Power off the controller. */
}MMCSD_pwrCtrl;

/** \brief Enumerates the list of supported bus voltages. */
typedef enum MMCSD_suppVolt_t
{
    MMCSD_SUPP_VOLT_1P8 = CSL_MMC_CAPA_VS18_MASK,
    /**< Supported voltage is 1.8 Volts. */
    MMCSD_SUPP_VOLT_3P0 = CSL_MMC_CAPA_VS30_MASK,
    /**< Supported voltage is 3.0 Volts. */
    MMCSD_SUPP_VOLT_3P3 = CSL_MMC_CAPA_VS33_MASK
    /**< Supported voltage is 3.3 Volts. */
}MMCSD_suppVolt;

/** \brief Enumerates the list of command types. */
typedef enum MMCSD_cmdType_t
{
    MMCSD_CMD_TYPE_OTHER = CSL_MMC_CMD_CMD_TYPE_NORMAL,
    /**< Others commands. */
    MMCSD_CMD_TYPE_BUS_SUSPEND = CSL_MMC_CMD_CMD_TYPE_SUSPEND,
    /**< Upon CMD52 "Bus Suspend" operation. */
    MMCSD_CMD_TYPE_FUNC_SEL = CSL_MMC_CMD_CMD_TYPE_RESUME,
    /**< Upon CMD52 "Function Select" operation. */
    MMCSD_CMD_TYPE_IO_ABORT = CSL_MMC_CMD_CMD_TYPE_ABORT
    /**< Upon CMD12 or CMD52 "I/O Abort" command. */
}MMCSD_cmdType;

/** \brief Enumerates the list of response types. */
typedef enum MMCSD_rspType_t
{
    MMCSD_RSP_TYPE_NONE = CSL_MMC_CMD_RSP_TYPE_NORSP,
    /**< No response. */
    MMCSD_RSP_TYPE_LEN_136 = CSL_MMC_CMD_RSP_TYPE_LGHT36,
    /**< Response Length 136 bits. */
    MMCSD_RSP_TYPE_LEN_48 = CSL_MMC_CMD_RSP_TYPE_LGHT48,
    /**< Response Length 48 bits. */
    MMCSD_RSP_TYPE_LEN_48_BUSY = CSL_MMC_CMD_RSP_TYPE_LGHT48B
    /**< Response Length 48 bits with busy after response. */
}MMCSD_rspType;

/** \brief Enumerates the list of response types. */
typedef enum MMCSD_xFerType_t
{
    MMCSD_XFER_TYPE_TX = CSL_MMC_CMD_DDIR_WRITE,
    /**< Data Write (host to card). */
    MMCSD_XFER_TYPE_RX = CSL_MMC_CMD_DDIR_READ
    /**< Data Read (card to host). */
}MMCSD_xFerType;

/** \brief Enumerates the controller's interrupt masks. */
typedef enum MMCSD_intrMask_t
{
    MMCSD_INTR_MASK_BADACCESS = CSL_MMC_IE_BADA_ENABLE_MASK,
    /**< Bad access to data space interrupt. */
    MMCSD_INTR_MASK_CARDERROR = CSL_MMC_IE_CERR_ENABLE_MASK,
    /**< Card error interrupt. */
    MMCSD_INTR_MASK_ADMAERROR = CSL_MMC_IE_ADMAE_ENABLE_MASK,
    /**< ADMA error interrupt. */
    MMCSD_INTR_MASK_ACMD12ERR = CSL_MMC_IE_ACE_ENABLE_MASK,
    /**< Auto CMD12 error interrupt. */
    MMCSD_INTR_MASK_DATABITERR = CSL_MMC_IE_DEB_ENABLE_MASK,
    /**< Data end bit error interrupt. */
    MMCSD_INTR_MASK_DATACRCERR = CSL_MMC_IE_DCRC_ENABLE_MASK,
    /**< Data CRC error interrupt. */
    MMCSD_INTR_MASK_DATATIMEOUT = CSL_MMC_IE_DTO_ENABLE_MASK,
    /**< Data time out error interrupt. */
    MMCSD_INTR_MASK_CMDINDXERR = CSL_MMC_IE_CIE_ENABLE_MASK,
    /**< Command index error interrupt. */
    MMCSD_INTR_MASK_CMDBITERR = CSL_MMC_IE_CEB_ENABLE_MASK,
    /**< Command end bit error. */
    MMCSD_INTR_MASK_CMDCRCERR = CSL_MMC_IE_CCRC_ENABLE_MASK,
    /**< Command CRC error interrupt. */
    MMCSD_INTR_MASK_CMDTIMEOUT = CSL_MMC_IE_CTO_ENABLE_MASK,
    /**< Command timeout error interrupt. */
    MMCSD_INTR_MASK_ERR = CSL_MMC_IE_NULL_MASK,
    /**< Error interrupt. */
    MMCSD_INTR_MASK_CARDREM = CSL_MMC_IE_CREM_ENABLE_MASK,
    /**< Card removal signal interrupt. */
    MMCSD_INTR_MASK_CARDINS = CSL_MMC_IE_CINS_ENABLE_MASK,
    /**< Card insertion signal interrupt. */
    MMCSD_INTR_MASK_BUFRDRDY = CSL_MMC_IE_BRR_ENABLE_MASK,
    /**< Buffer read ready interrupt. */
    MMCSD_INTR_MASK_BUFWRRDY = CSL_MMC_IE_BWR_ENABLE_MASK,
    /**< Buffer write ready interrupt. */
    MMCSD_INTR_MASK_TRNFCOMP = CSL_MMC_IE_TC_ENABLE_MASK,
    /**< Transfer completed signal interrupt. */
    MMCSD_INTR_MASK_CMDCOMP = CSL_MMC_IE_CC_ENABLE_MASK
    /**< Command completed signal interrupt. */
}MMCSD_intrMask;

/** \brief Structure holding the hs mmcsd controller system configurations. */
typedef struct MMCSD_sysCfg_t
{
    uint32_t clockActivity;
    /**< Configuration of clock activity during wake up period. This can take
         values from the following enum #MMCSD_clkAct. */
    uint32_t standbyMode;
    /**< standby mode configuration. This can take values from the following
         enum #MMCSD_standbyMode. */
    uint32_t idleMode;
    /**< Idle mode configuration. This can take values from the following
         enum #MMCSD_idleMode. */
    uint32_t enableWakeup;
    /**< Wake up enable/disable control. This can take following two values
         - TRUE  - Enable Wakeup
         - FALSE - Disable wakeup */
    uint32_t enableAutoIdle;
    /**< Auto idle enable/disable control. This can take following two values
         - TRUE  - Enable auto idle.
         - FALSE - Disable auto idle. */
}MMCSD_sysCfg;

/**
 * \brief Structure for MMCSD command.
 */
typedef struct MMCSD_cmd_t{
    uint32_t cmdId;
    /**< Command ID uses standard MMCSD card commands. */
    uint32_t cmdType;
    /**< Command type uses #MMCSD_cmdType. */
    uint32_t rspType;
    /**< Response type uses #MMCSD_rspType. */
    uint32_t xferType;
    /**< Command direction uses #MMCSD_xFerType. */
} MMCSD_cmd;

/** \brief Structure holding the hs mmcsd controller command object. */
typedef struct MMCSD_cmdObj_t
{
    MMCSD_cmd cmd;
    /**< Command to be passed to the controller/card. */
    uint32_t cmdArg;
    /**< Argument for the command. */
    uint32_t enableData;
    /**< This flag enables/disables data transfer. This can take following
         two values:
         - TRUE  - Enable data transfer
         - FALSE - Disable data transfer. */
    uint32_t numBlks;
    /**< Transfer data length in number of blocks (multiple of BLEN). This can
         take values in the following range:
         1 <= numBlks <= 65535. Value of 0 means Stop count. */
    uint32_t enableDma;
    /**< This flag enables/disables Dma. This can take following two values:
         - TRUE  - Enable DMA.
         - FALSE - Disable DMA. */
}MMCSD_cmdObj;

/** \brief Enumerates the idle modes. */
typedef enum MMCSD_idleMode_t
{
    MMCSD_IDLE_MODE_FORCE = CSL_MMC_SYSCONFIG_SIDLEMODE_FORCE,
    /**< Force idle mode. */
    MMCSD_IDLE_MODE_NONE = CSL_MMC_SYSCONFIG_SIDLEMODE_NOIDLE,
    /**< No idle mode. */
    MMCSD_IDLE_MODE_SMART = CSL_MMC_SYSCONFIG_SIDLEMODE_SMART
    /**< Smart idle mode. */
}MMCSD_idleMode;

/** \brief Enumerates mmc sd controller's bus voltages. */
typedef enum MMCSD_busVolt_t
{
    MMCSD_BUS_VOLT_1P8 = CSL_MMC_HCTL_SDVS__1V8,
    /**< Bus voltage is 1.8 Volts. */
    MMCSD_BUS_VOLT_3P0 = CSL_MMC_HCTL_SDVS__3V0,
    /**< Bus voltage is 3.0 volts. */
    MMCSD_BUS_VOLT_3P3 = CSL_MMC_HCTL_SDVS__3V3
    /**< Bus voltage is 3.3 volts. */
}MMCSD_busVolt;

typedef struct MMCSD_iodelayParams_s {
    uint32_t deviceType;       /*!< MMC device type */
    uint32_t transferSpeed;    /*!< MMC transfer speed */
    uint32_t operatingVoltage; /*!< MMC peripheral operating voltage */
    uint32_t loopBackType;     /*!< MMC peripheral loop back type */
} MMCSD_ioDelayParams;

typedef enum {
    MMCSD_BUS_VOLTAGE_1_8V = 0x1U,
	MMCSD_BUS_VOLTAGE_3_0V = 0x2U,
	MMCSD_BUS_VOLTAGE_3_3V = 0x4U
} MMCSD_BusVoltage;

/*!
 *  @brief      ENUMs to read/configure the input clock to the MMC controller.
 */

typedef enum {
   MMCSD_INPUT_CLOCK_CTRL_GET, /* Returns the input clock to MMC'n' */
   MMCSD_INPUT_CLOCK_CTRL_SET  /* Sets the input clock to MMC'n' as per a value */
} MMCSD_inputClkCtrlMode_e;

/*!
 *  @brief      Return status of MMCSD.
 */

typedef int32_t (*MMCSD_iodelayFxn) (uint32_t instanceNum, MMCSD_ioDelayParams *iodelayParams);

/*!
 *  @brief      A function pointer to set the voltage of the MMC I/O Cells and PBAIS to the desired voltage.
 */
typedef int32_t (*MMCSD_switchVoltage) (uint32_t controllerNum, MMCSD_BusVoltage voltage);

/*!
 *  @brief      A function pointer to read/configure the input clock to the MMC controller.
 */
typedef uint32_t (*MMCSD_inputClockControl) (uint32_t instNum, uint32_t *clkFreq, MMCSD_inputClkCtrlMode_e);

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

    uint16_t maxReadBlockLen;
    /* Maximum supported block length for read */

    uint16_t maxWriteBlockLen;
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

    uint16_t maxReadBlockLen;
    /* Maximum supported block length for read */

    uint16_t maxWriteBlockLen;
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
 *  the MMC peripheral.
 */
typedef struct
{
    uint32_t cmd;
    /**< Command register content composed of CMD ID, DP, TYPE, RESP TYPE etc */

    uint32_t flags;
    /**< Command flag as per MMC device specification */

    uint32_t arg;
    /**< Command argument as per MMC device specification */

    void    *dataBuf;
    /**< buffer containing data to be read into or written */

    uint32_t blockSize;
    /**< Number of bytes to be transferred per block */

    uint32_t blockCount;
    /**< Number of block to be transferred */

    uint32_t response[4];
    /**< Command response per MMC device specification */

} MMCSD_Transaction;

/**
 *  \brief MMCSD instance attributes - used during init time
 */
typedef struct
{
    uint32_t instNum;
    /*< MMC Peripheral instance number */

    uint32_t baseAddr;
	/**< MMCSD subsystem registers base address */

    uint32_t intrNum;
    /**< Module interrupt vector */

    uint32_t inputClk;
    /*< MMC input functional clock */

    uint32_t outputClk;
    /*< MMC output clock */

    uint32_t cardType;
    /**< Type of card */

    uint32_t supportedBusWidth;
    /*! Supported bus width */

    uint32_t supportedBusVoltages;
    /**< Supported bus voltages */

    uint32_t intrEnable;
    /*< MMC enable interrupt */

    MMCSD_iodelayFxn iodelayFxn;
    /*< MMC Voltage Switch function */

    MMCSD_switchVoltage switchVoltageFxn;
    /*! MMC input clock control function */

    MMCSD_inputClockControl inputClockControl;

    uint32_t dmaEnable;
   /*EDMA related Hardware configuration details*/

} MMCSD_Attrs;

/**
 *  \brief MMCSD driver object
 */
typedef struct
{
    uint32_t cardType;
    /*< Type of card. */

    MMCSD_Handle handle;
    /**< Instance handle */

    uint32_t isOpen;
    /*< flag to indicate module is open */

    uint32_t ocr;
    /*< Operation condition register. */

    uint8_t switched_to_v18;
    /*< Has the card switched to 1.8V */

    uint32_t rca;
    /*< Relative card address. */

    uint32_t blockCount;
    /*< Number of blocks. */

    uint32_t dataBlockCount;
    /*< Data dec. block counter */

    uint32_t dataBlockSize;
    /*! Internal dec. blockCounter */

    uint8_t	support1_8V;
    /*< Does card support 1.8V. */

    uint64_t size;
    /*< Size of the card in bytes. */

    uint8_t highCap;
    /*< Is card of high capacity. */

    uint8_t *dataBufIdx;
    /*< Data buffer index */

    uint32_t cid[4];
    /*! Card identification register. */

    volatile uint32_t cmdComp;
    /*< Command completion flag */

    volatile uint32_t cmdTimeout;
    /*< Command timeout flag */

    volatile uint32_t xferInProgress;
    /*< Command completion flag */

    volatile uint32_t xferComp;
    /*< Transfer completion flag */

    uint8_t tranSpeed;
    /*< Transfer speed. */

    uint32_t csd[4];
    /*< Card specific data. */

    uint32_t scr[2];
    /*< First value of enumeration. Can be used for validation. */

    volatile uint32_t xferTimeout;
    /*< Transfer timeout flag */

    uint8_t sdVer;
    /*< Version of card. */

    uint32_t intrEnable;
    /*< MMC enable interrupt */

    uint32_t dmaEnable;
    /**< DMA enable */

    HwiP_Object hwiObj;
    /**< Interrupt object */

    uint8_t ecsd[512];
    /*< eMMC specific data. */

    uint32_t blockSize;
    /**< Number of bytes to be transferred per block */

    uint8_t busWidth;
    /*< Bus width. */

    uint8_t *writeBufIdx;        /*< Internal inc. dataBuf index */

    uint32_t writeBlockCount;    /*< Internal dec. blockCounter */

    uint8_t *readBufIdx;    /*< Internal inc. dataBuf index */

    uint32_t readBlockCount;    /*< Internal dec. blockCounter */

    uint32_t isHC;
    /**< Is card of high capacity */

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

} MMCSD_Object;

typedef struct
{
    MMCSD_Attrs *attrs;
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

#ifdef __cplusplus
}
#endif
/** @} */
#endif /* MMCSD_H_ */