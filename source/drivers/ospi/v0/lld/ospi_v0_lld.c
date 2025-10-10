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
 *  \file ospi_v0_lld.c
 *
 *  \brief File containing OSPI Driver APIs implementation for version V0.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* This is needed for memset/memcpy */
#include <string.h>
#include <drivers/ospi/v0/lld/ospi_lld.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/ospi/v0/lld/dma/ospi_lld_dma.h>
#include <drivers/soc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define OSPI_DIV_ROUND_UP(n, d)     (((n) + (d) - 1U) / (d))

/** \brief    OSPI DMA related macros */
#define OSPI_DMA_COPY_LOWER_LIMIT     (1024U)
#define OSPI_DMA_COPY_SRC_ALIGNMENT   (32U)
#define OSPI_DMA_COPY_SIZE_ALIGNMENT  (32U)

/** \brief    OSPI Command default Length */
#define CSL_OSPI_CMD_LEN_DEFAULT            (1U)            /*In bytes */
#define CSL_OSPI_CMD_LEN_EXTENDED           (5U)            /*In bytes */

/** \brief    OSPI operation timeout value */
#define CSL_OSPI_POLL_IDLE_TIMEOUT          (5000U)         /* in millisecond */
#define CSL_OSPI_POLL_IDLE_DELAY            (1U)            /* in millisecond */
#define CSL_OSPI_POLL_IDLE_RETRY            (3U)
#define CSL_OSPI_REG_RETRY                  (10U)
#define CSL_OSPI_FIFO_WIDTH                 (4U)

/**
 *  \brief   OSPI device delay parameter array size.
 */
#define CSL_OSPI_DEV_DELAY_ARRAY_SIZE  (4U)

/**
 *  \brief   OSPI ref clock generates max frequency of 200MHz, OSPI
 *           max operating frequency 200/8 equals 25MHz
 */
#define OSPI_MAX_OPERATING_FREQUENCY      (25000000U)

#if defined(SOC_AM64X) || defined (SOC_AM243X)
/** \brief   OSPI device delays in cycles of OSPI master ref clock */
#define CSL_OSPI_DEV_DELAY_CSSOT     (60U)  /* Chip Select Start of Transfer Delay */
#define CSL_OSPI_DEV_DELAY_CSEOT     (60U)  /* Chip Select End of Transfer Delay */
#define CSL_OSPI_DEV_DELAY_CSDADS    (60U) /* Chip Select De-Assert Different Slaves Delay */
#define CSL_OSPI_DEV_DELAY_CSDA      (60U) /* Chip Select De-Assert Delay */
#else
/** \brief   OSPI device delays in cycles of OSPI controller ref clock */
#define CSL_OSPI_DEV_DELAY_CSSOT     (46U)  /* Chip Select Start of Transfer Delay */
#define CSL_OSPI_DEV_DELAY_CSEOT     (46U)  /* Chip Select End of Transfer Delay */
#define CSL_OSPI_DEV_DELAY_CSDADS    (192U) /* Chip Select De-Assert Different Peripheral Delay */
#define CSL_OSPI_DEV_DELAY_CSDA      (192U) /* Chip Select De-Assert Delay */
#endif

/** \brief  SRAM partition configuration definitions */
/** size of the indirect read/write partition in the SRAM,
    in units of SRAM locations */
#define CSL_OSPI_SRAM_SIZE_WORDS        (128U)
#define CSL_OSPI_SRAM_PARTITION_RD      (64U)
#define CSL_OSPI_SRAM_PARTITION_WR      (CSL_OSPI_SRAM_SIZE_WORDS - \
                                         CSL_OSPI_SRAM_PARTITION_RD)
/* Default value for SRAM PARTITION register */
#define CSL_OSPI_SRAM_PARTITION_DEFAULT (CSL_OSPI_SRAM_PARTITION_RD -  1U)

#define OSPI_READ_WRITE_TIMEOUT (500000U)
#define OSPI_CHECK_IDLE_DELAY       (10U)
#define OSPI_CALIBRATE_DELAY        (20U)
#define OSPI_XIP_SETUP_DELAY        (250U)

/**  \brief  SRAM fill level watermark */
/* Read watermark fill level in words, will generate DMA request or
   interrupt when the SRAM fill level is above the watermark */
#define CSL_OSPI_SRAM_WARERMARK_RD_LVL  (CSL_OSPI_SRAM_PARTITION_RD / 4U)
/* Write watermark fill level in words, will generate DMA request or
   interrupt when the SRAM fill level is below the watermark */
#define CSL_OSPI_SRAM_WATERMARK_WR_LVL  (CSL_OSPI_SRAM_PARTITION_WR / 4U)


#define CSL_OSPI_INTR_MASK_IND_XFER (CSL_OSPI_FLASH_CFG_IRQ_STATUS_REG_INDIRECT_OP_DONE_FLD_MASK         | \
                                     CSL_OSPI_FLASH_CFG_IRQ_STATUS_REG_INDIRECT_XFER_LEVEL_BREACH_FLD_MASK | \
                                     CSL_OSPI_FLASH_CFG_IRQ_STATUS_REG_INDRD_SRAM_FULL_FLD_MASK)

#define CSL_OSPI_INTR_MASK_ALL  (CSL_OSPI_FLASH_CFG_IRQ_STATUS_REG_MODE_M_FAIL_FLD_MASK         | \
                                 CSL_OSPI_FLASH_CFG_IRQ_STATUS_REG_UNDERFLOW_DET_FLD_MASK       | \
                                 CSL_OSPI_FLASH_CFG_IRQ_STATUS_REG_INDIRECT_OP_DONE_FLD_MASK         | \
                                 CSL_OSPI_FLASH_CFG_IRQ_STATUS_REG_INDIRECT_READ_REJECT_FLD_MASK       | \
                                 CSL_OSPI_FLASH_CFG_IRQ_STATUS_REG_PROT_WR_ATTEMPT_FLD_MASK     | \
                                 CSL_OSPI_FLASH_CFG_IRQ_STATUS_REG_ILLEGAL_ACCESS_DET_FLD_MASK  | \
                                 CSL_OSPI_FLASH_CFG_IRQ_STATUS_REG_INDIRECT_XFER_LEVEL_BREACH_FLD_MASK | \
                                 CSL_OSPI_FLASH_CFG_IRQ_STATUS_REG_INDRD_SRAM_FULL_FLD_MASK    | \
                                 CSL_OSPI_FLASH_CFG_IRQ_STATUS_REG_POLL_EXP_INT_FLD_MASK)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Internal functions */

static int32_t OSPI_waitReadSRAMLevel(const CSL_ospi_flash_cfgRegs *pReg, uint32_t *rdLevel);
static int32_t OSPI_waitIndReadComplete(const CSL_ospi_flash_cfgRegs *pReg);
static int32_t OSPI_waitWriteSRAMLevel(const CSL_ospi_flash_cfgRegs *pReg,
                                     uint32_t *sramLvl);
static int32_t OSPI_waitIndWriteComplete(const CSL_ospi_flash_cfgRegs *pReg);
static int32_t OSPI_waitIdle(OSPILLD_Handle hOspi, uint32_t timeOut);

static int32_t OSPI_flashExecCmd(OSPILLD_Handle hOspi, const CSL_ospi_flash_cfgRegs *pReg, 
                                uint32_t timeout);
static void OSPI_readFifoData(uintptr_t indAddr, uint8_t *dest, uint32_t rdLen);
static void OSPI_writeFifoData(uintptr_t indAddr, const uint8_t *src, uint32_t wrLen);

static int32_t OSPI_programInstance(OSPILLD_Handle hOspi);
static int32_t OSPI_isDmaRestrictedRegion(OSPILLD_Handle hOspi, uint32_t addr);
static uint32_t OSPI_utilLog2(uint32_t num);
static uint8_t OSPI_getCmdExt(OSPILLD_Handle hOspi, uint8_t cmd);
static uint32_t OSPI_lld_calculateTicksForns(const uint32_t refClkhz, const uint32_t nsVal);

/* LLD Parameter Validation */ 
static inline int32_t OSPI_lld_isFrameFormatValid(uint32_t frmFmt);
static inline int32_t OSPI_lld_isChipSelectValid(uint32_t chipSelect);
static inline int32_t OSPI_lld_isDecoderChipSelectValid(uint32_t decChipSelect);
static int32_t OSPI_lld_param_check(bool);


/* Interrupt Functions */
static void OSPI_disableInterrupt(OSPILLD_Handle hOspi);
static void OSPI_enableInterrupt(OSPILLD_Handle hOspi);
static void OSPI_clearInterrupt(OSPILLD_Handle hOspi);
static uint32_t OSPI_getSramLevel(OSPILLD_Handle hOspi);
static uint32_t OSPI_getWriteSramLevel(OSPILLD_Handle hOspi);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** \brief LUT table for log2 calculation using DeBruijn sequence */
static const uint8_t gTable[32] =
{
    0, 9, 1, 10, 13, 21, 2, 29, 11, 14, 16, 18, 22, 25, 3, 30,
    8, 12, 20, 28, 15, 17, 24, 7, 19, 27, 23, 6, 26, 5, 4, 31
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t OSPI_lld_init(OSPILLD_Handle hOspi)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;
    OSPILLD_InitHandle    hOspiInit;

    /* Check if hOspi is NULL */
    if((hOspi != NULL) && (hOspi->hOspiInit != NULL))
    {
        if(hOspi->state != OSPI_STATE_RESET)
        {
            status = OSPI_INVALID_STATE;
        }
    }
    else
    {
        status = OSPI_LLD_INVALID_PARAM;
    }

    if(OSPI_SYSTEM_SUCCESS == status)
    {
        hOspi->state = OSPI_STATE_BUSY;
        hOspiInit = hOspi->hOspiInit;

        /* Verify the OSPI parameters */
        status += OSPI_lld_param_check(IS_OSPI_BASE_ADDR_VALID(hOspi->baseAddr));
        status += OSPI_lld_param_check(IS_OSPI_DATA_BASE_ADDR_VALID(hOspiInit->dataBaseAddr));
        status += (hOspiInit->inputClkFreq != 0) ? OSPI_SYSTEM_SUCCESS : OSPI_SYSTEM_FAILURE;
        status += OSPI_lld_isFrameFormatValid(hOspiInit->frmFmt);
        status += OSPI_lld_isChipSelectValid(hOspiInit->chipSelect);
        status += OSPI_lld_isDecoderChipSelectValid(hOspiInit->decChipSelect);

        /* Program OSPI instance according the user config */
        status += OSPI_programInstance(hOspi);

    }

    /* Free up resources in case of error */
    if(OSPI_SYSTEM_SUCCESS == status)
    {
        hOspi->state = OSPI_STATE_IDLE;
    }
    else
    {
        /* Free-up resources in case of error */
        OSPI_lld_deInit(hOspi);
    }

    return status;
}

int32_t OSPI_lld_initDma(OSPILLD_Handle hOspi)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;
    OSPILLD_InitHandle    hOspiInit;

    /* Check if hOspi is NULL */
    if((hOspi != NULL) && (hOspi->hOspiInit != NULL))
    {
        if(hOspi->state != OSPI_STATE_RESET)
        {
            status = OSPI_INVALID_STATE;
        }
    }
    else
    {
        status = OSPI_LLD_INVALID_PARAM;
    }

    if(OSPI_SYSTEM_SUCCESS == status)
    {
        hOspi->state = OSPI_STATE_BUSY;
        hOspiInit = hOspi->hOspiInit;

        /* Verify the OSPI parameters */
        status += OSPI_lld_param_check(IS_OSPI_BASE_ADDR_VALID(hOspi->baseAddr));
        status += OSPI_lld_param_check(IS_OSPI_DATA_BASE_ADDR_VALID(hOspiInit->dataBaseAddr));
        status += (hOspiInit->inputClkFreq != 0) ? OSPI_SYSTEM_SUCCESS : OSPI_SYSTEM_FAILURE;
        status += (hOspiInit->ospiDmaHandle != NULL) ? OSPI_SYSTEM_SUCCESS : OSPI_SYSTEM_FAILURE;
        status += (hOspiInit->ospiDmaChConfig != NULL) ? OSPI_SYSTEM_SUCCESS : OSPI_SYSTEM_FAILURE;
        status += OSPI_lld_isFrameFormatValid(hOspiInit->frmFmt);
        status += OSPI_lld_isChipSelectValid(hOspiInit->chipSelect);
        status += OSPI_lld_isDecoderChipSelectValid(hOspiInit->decChipSelect);

        status += OSPI_dmaOpen(hOspi);

        /* Program OSPI instance according the user config */
        status += OSPI_programInstance(hOspi);

    }

    /* Free up resources in case of error */
    if(OSPI_SYSTEM_SUCCESS == status)
    {
        hOspi->state = OSPI_STATE_IDLE;
    }
    else
    {
        /* Free-up resources in case of error */
        OSPI_lld_deInit(hOspi);
    }

    return status;
}

int32_t OSPI_lld_deInit(OSPILLD_Handle hOspi)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;
    const CSL_ospi_flash_cfgRegs *pReg;
    uint32_t regVal;
    OSPILLD_InitHandle hOspiInit;

    /* Check if hOspi is NULL */
    if(hOspi != NULL)
    {
        hOspi->state = OSPI_STATE_BUSY;

        pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        /* Disable interrupts by read-modify-write to IRQ mask register */
        regVal = CSL_REG32_RD(&pReg->IRQ_MASK_REG);
        regVal &= ~(CSL_OSPI_INTR_MASK_ALL);
        CSL_REG32_WR(&pReg->IRQ_MASK_REG, regVal);

        hOspiInit = hOspi->hOspiInit;
        if (hOspiInit != NULL)
        {
            if(hOspiInit->phyEnable == TRUE)
            {
                /*
                * Some fields in RD_DATA_CAPTURE_REG are modified by the PHY Tuning API.
                * These fields need to be reset here to avoid errors in subsequent tests.
                */
                CSL_REG32_FINS(&pReg->RD_DATA_CAPTURE_REG,
                                OSPI_FLASH_CFG_RD_DATA_CAPTURE_REG_SAMPLE_EDGE_SEL_FLD,
                                0);

                CSL_REG32_FINS(&pReg->RD_DATA_CAPTURE_REG,
                                OSPI_FLASH_CFG_RD_DATA_CAPTURE_REG_DQS_ENABLE_FLD,
                                0);
            }
        }
        else
        {
            status = OSPI_LLD_INVALID_PARAM;
        }

        hOspi->state = OSPI_STATE_RESET;
    }
    else
    {
        status = OSPI_LLD_INVALID_PARAM;
    }

    return status;
}

int32_t OSPI_lld_deInitDma(OSPILLD_Handle hOspi)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;
    const CSL_ospi_flash_cfgRegs *pReg;
    uint32_t regVal;
    OSPILLD_InitHandle hOspiInit;

    /* Check if hOspi is NULL */
    if(hOspi != NULL)
    {
        hOspi->state = OSPI_STATE_BUSY;

        pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        /* Disable interrupts by read-modify-write to IRQ mask register */
        regVal = CSL_REG32_RD(&pReg->IRQ_MASK_REG);
        regVal &= ~(CSL_OSPI_INTR_MASK_ALL);
        CSL_REG32_WR(&pReg->IRQ_MASK_REG, regVal);

        hOspiInit = hOspi->hOspiInit;
        if (hOspiInit != NULL)
        {
            status = OSPI_dmaClose(hOspi);

            if(hOspiInit->phyEnable == TRUE)
            {
                /*
                * Some fields in RD_DATA_CAPTURE_REG are modified by the PHY Tuning API.
                * These fields need to be reset here to avoid errors in subsequent tests.
                */
                CSL_REG32_FINS(&pReg->RD_DATA_CAPTURE_REG,
                                OSPI_FLASH_CFG_RD_DATA_CAPTURE_REG_SAMPLE_EDGE_SEL_FLD,
                                0);

                CSL_REG32_FINS(&pReg->RD_DATA_CAPTURE_REG,
                                OSPI_FLASH_CFG_RD_DATA_CAPTURE_REG_DQS_ENABLE_FLD,
                                0);
            }
        }
        else
        {
            status = OSPI_LLD_INVALID_PARAM;
        }

        hOspi->state = OSPI_STATE_RESET;
    }
    else
    {
        status = OSPI_LLD_INVALID_PARAM;
    }

    return status;
}

void OSPI_lld_Transaction_init(OSPI_Transaction *trans)
{
    /* OSPI transaction initialisation */
    trans->count = 0U;
    trans->buf = NULL;
    trans->addrOffset = 0U;
    trans->transferOffset = 0U;
    trans->transferTimeout = OSPI_WAIT_FOREVER;
    trans->status = OSPI_TRANSFER_STARTED;
}

void OSPI_lld_readCmdParams_init(OSPI_ReadCmdParams *rdParams)
{
    /* OSPI read parameters initialisation */
    rdParams->cmd = OSPI_CMD_INVALID_OPCODE;
    rdParams->cmdAddr = OSPI_CMD_INVALID_ADDR;
    rdParams->numAddrBytes = 3;
    rdParams->rxDataBuf = NULL;
    rdParams->rxDataLen = 0;
    rdParams->dummyBits = 0;
    rdParams->readTimeout = OSPI_READ_WRITE_TIMEOUT;
}

void OSPI_lld_writeCmdParams_init(OSPI_WriteCmdParams *wrParams)
{
    /* OSPI Write parameters initialisation */
    wrParams->cmd = OSPI_CMD_INVALID_OPCODE;
    wrParams->cmdAddr = OSPI_CMD_INVALID_ADDR;
    wrParams->numAddrBytes = 3;
    wrParams->txDataBuf = NULL;
    wrParams->txDataLen = 0;
    wrParams->writeTimeout = OSPI_READ_WRITE_TIMEOUT;
}

uint32_t OSPI_lld_getInputClk(OSPILLD_Handle hOspi)
{
    uint32_t retVal = 0U;
    /* Get the OSPI input clock frequency */
    if(hOspi != NULL)
    {
        retVal = hOspi->hOspiInit->inputClkFreq;
    }
    return retVal;
}

uint32_t OSPI_lld_isDacEnable(OSPILLD_Handle hOspi)
{
    uint32_t retVal = 0U;
    /* Check if DAC is enabled or not */
    retVal = hOspi->hOspiInit->dacEnable;
    return retVal;
}

uint32_t OSPI_lld_isDmaEnable(OSPILLD_Handle hOspi)
{
    uint32_t retVal = 0U;
    /* Check if DMA is enabled or not */
    retVal = hOspi->hOspiInit->dmaEnable;
    return retVal;
}

uint32_t OSPI_lld_isIntrEnable(OSPILLD_Handle hOspi)
{
    uint32_t retVal = 0U;
    /* Check if Interrupt is enabled or not */
    retVal = hOspi->hOspiInit->intrEnable;
    return retVal;
}

uint32_t OSPI_lld_isPhyEnable(OSPILLD_Handle hOspi)
{
    uint32_t retVal = 0U;
    /* Check is phy is enabled or not */
    retVal = hOspi->hOspiInit->phyEnable;
    return retVal;
}

void OSPI_lld_setPhyEnableSuccess(OSPILLD_Handle hOspi, uint32_t success)
{
    /* Check if hOspi is NULL */
    if(hOspi != NULL)
    {
        /* Set the Phy enable status */
        hOspi->phyEnableSuccess = success;
    }
}

uint32_t OSPI_lld_getPhyEnableSuccess(OSPILLD_Handle hOspi)
{
    uint32_t success = 0;

    if(hOspi != NULL)
    {
        /* Get the phy enable status */
        success = hOspi->phyEnableSuccess;
    }

    return success;
}

int32_t OSPI_lld_enableDDR(OSPILLD_Handle hOspi)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;

    if(hOspi != NULL)
    {
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        /* Enable DTR protocol */
        CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_ENABLE_DTR_PROTOCOL_FLD,
                   TRUE);
                   
    }
    else
    {
        status = OSPI_SYSTEM_FAILURE;
    }

    return status;
}

int32_t OSPI_lld_enableSDR(OSPILLD_Handle hOspi)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;

    /* Check if hOspi is NULL */
    if(hOspi != NULL)
    {
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        /* Disable DTR protocol */
        CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_ENABLE_DTR_PROTOCOL_FLD,
                   FALSE);
    }
    else
    {
        status = OSPI_SYSTEM_FAILURE;
    }

    return status;
}

int32_t OSPI_lld_enableDdrRdCmds(OSPILLD_Handle hOspi)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;

    /* Check if hOspi is NULL */
    if(hOspi != NULL)
    {
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        /* Enable DDR EN commands */
        CSL_REG32_FINS(&pReg->DEV_INSTR_RD_CONFIG_REG,
                   OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_DDR_EN_FLD,
                   1);
    }
    else
    {
        status = OSPI_SYSTEM_FAILURE;
    }

    return status;
}

int32_t OSPI_lld_setRdDataCaptureDelay(OSPILLD_Handle hOspi, uint32_t rdDataCapDelay)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;

    /* Check if hOspi is NULL */
    if(hOspi != NULL)
    {
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        /* Set read capture delay */
        CSL_REG32_FINS(&pReg->RD_DATA_CAPTURE_REG,
                   OSPI_FLASH_CFG_RD_DATA_CAPTURE_REG_DELAY_FLD,
                   rdDataCapDelay);

        /* Update book keeping */
        hOspi->rdDataCapDelay = rdDataCapDelay;
    }
    else
    {
        status = OSPI_SYSTEM_FAILURE;
    }

    return status;
}

void OSPI_lld_setNumAddrBytes(OSPILLD_Handle hOspi, uint32_t numAddrBytes)
{
    /* Check if hOspi is NULL */
    if(hOspi != NULL)
    {
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);
        uint32_t addrByteCode = 0;

        if(numAddrBytes > 0)
        {
            addrByteCode = numAddrBytes - 1;
        }
        else
        {
            addrByteCode = 0;
        }

        CSL_REG32_FINS(&pReg->DEV_SIZE_CONFIG_REG, OSPI_FLASH_CFG_DEV_SIZE_CONFIG_REG_NUM_ADDR_BYTES_FLD, addrByteCode);

        /* Update book-keeping variable in OSPI object */
        hOspi->numAddrBytes = numAddrBytes;
    }
    else
    {
        /* do nothing */
    }
}

void OSPI_lld_setDeviceSize(OSPILLD_Handle hOspi, uint32_t pageSize, uint32_t blkSize)
{
    /* Check if hOspi is NULL */
    if(hOspi != NULL)
    {
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        CSL_REG32_FINS(&pReg->DEV_SIZE_CONFIG_REG, OSPI_FLASH_CFG_DEV_SIZE_CONFIG_REG_BYTES_PER_DEVICE_PAGE_FLD, pageSize);
        CSL_REG32_FINS(&pReg->DEV_SIZE_CONFIG_REG, OSPI_FLASH_CFG_DEV_SIZE_CONFIG_REG_BYTES_PER_SUBSECTOR_FLD, OSPI_utilLog2(blkSize));
    }
    else
    {
        /* do nothing */
    }
}

void OSPI_lld_setModeBits(OSPILLD_Handle hOspi, uint32_t modeBits)
{
    /* Check if hOspi is NULL */
    if(NULL != hOspi)
    {
        const CSL_ospi_flash_cfgRegs *pReg;

        pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        CSL_REG32_FINS(&pReg->MODE_BIT_CONFIG_REG,
                    OSPI_FLASH_CFG_MODE_BIT_CONFIG_REG_MODE_FLD,
                    (uint8_t)modeBits);
    }
}

void OSPI_lld_enableModeBitsCmd(OSPILLD_Handle hOspi)
{
    /* Check if hOspi is NULL */
    if(NULL != hOspi)
    {
        const CSL_ospi_flash_cfgRegs *pReg;

        pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        CSL_REG32_FINS(&pReg->FLASH_CMD_CTRL_REG,
                    OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_ENB_MODE_BIT_FLD,
                    1);
    }
}

void OSPI_lld_enableModeBitsRead(OSPILLD_Handle hOspi)
{
    /* Check if hOspi is NULL */
    if(NULL != hOspi)
    {
        const CSL_ospi_flash_cfgRegs *pReg;

        pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        CSL_REG32_FINS(&pReg->DEV_INSTR_RD_CONFIG_REG,
                    OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_MODE_BIT_ENABLE_FLD,
                    1);
    }
}

static uint32_t OSPI_isProtocolValid(uint32_t cmd, uint32_t addr, uint32_t data, uint32_t strDtr)
{
    uint32_t isValid = TRUE;

    uint32_t lineArr[] = {1,2,4,8};
    uint32_t varArr[] = {cmd, addr, data};

    uint32_t i, j;

    for(i = 0; i < 3; i++)
    {
        uint32_t flag = 0;
        for(j = 0; j < 4; j++)
        {
            if(varArr[i] == lineArr[j])
            {
                flag = 1;
                break;
            }
        }
        if(flag == 0)
        {
            /* Error in one of the lines. Abort */
            isValid = FALSE;
            break;
        }
    }

    if((isValid == TRUE) && (strDtr > 1))
    {
        isValid = FALSE;
    }

    return isValid;
}

uint32_t OSPI_lld_getProtocol(OSPILLD_Handle hOspi)
{
    uint32_t retVal = OSPI_NOR_PROTOCOL_INVALID;

    /* Check if hOspi is NULL */
    if(hOspi != NULL)
    {
        retVal = hOspi->protocol;
    }
    return retVal;
}

void OSPI_lld_setProtocol(OSPILLD_Handle hOspi, uint32_t protocol)
{
    if(hOspi != NULL)
    {
        /* First set the transfer lines */
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        uint32_t dtr  = ((protocol >> 24) & 0xFF);
        uint32_t cmd  = ((protocol >> 16) & 0xFF);
        uint32_t addr = ((protocol >> 8) & 0xFF);
        uint32_t data = (protocol & 0xFF);

        /* Validate requested protocol */
        if(OSPI_isProtocolValid(cmd, addr, data, dtr) == TRUE)
        {
            /* Take log2 of each line value to set register */
            cmd = OSPI_utilLog2(cmd);
            addr = OSPI_utilLog2(addr);
            data = OSPI_utilLog2(data);

            /* Transfer lines for Read */
            /* Set transfer lines for sending command */
            CSL_REG32_FINS(&pReg->DEV_INSTR_RD_CONFIG_REG, OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_INSTR_TYPE_FLD, cmd);
            /* Set transfer lines for sending address */
            CSL_REG32_FINS(&pReg->DEV_INSTR_RD_CONFIG_REG, OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_ADDR_XFER_TYPE_STD_MODE_FLD, addr);
            /* Set transfer lines for sending data */
            CSL_REG32_FINS(&pReg->DEV_INSTR_RD_CONFIG_REG, OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_DATA_XFER_TYPE_EXT_MODE_FLD, data);

            /* Transfer lines for Write */
            /* Set transfer lines for sending address */
            CSL_REG32_FINS(&pReg->DEV_INSTR_WR_CONFIG_REG, OSPI_FLASH_CFG_DEV_INSTR_WR_CONFIG_REG_ADDR_XFER_TYPE_STD_MODE_FLD, addr);
            /* Set transfer lines for sending data */
            CSL_REG32_FINS(&pReg->DEV_INSTR_WR_CONFIG_REG, OSPI_FLASH_CFG_DEV_INSTR_WR_CONFIG_REG_DATA_XFER_TYPE_EXT_MODE_FLD, data);

            if(dtr)
            {
                if(protocol == OSPI_NOR_PROTOCOL(8,8,8,1))
                {
                    OSPI_lld_enableDDR(hOspi);
                    OSPI_lld_enableDdrRdCmds(hOspi);
                    OSPI_lld_setDualOpCodeMode(hOspi);
                }
                if(protocol == OSPI_NOR_PROTOCOL(4,4,4,1))
                {
                    OSPI_lld_enableDdrRdCmds(hOspi);
                }
            }

            /* Update book-keeping variable in OSPI object */
            hOspi->protocol = protocol;
        }
    }
    else
    {
        /* do nothing */
    }
}

void OSPI_lld_setReadDummyCycles(OSPILLD_Handle hOspi, uint32_t dummyCycles)
{
    /* Check if hOspi is NULL */
    if(hOspi != NULL)
    {
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        /* Set read dummy cycles */
        CSL_REG32_FINS(&pReg->DEV_INSTR_RD_CONFIG_REG, OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_DUMMY_RD_CLK_CYCLES_FLD, dummyCycles);

        /* Update book-keeping variable in OSPI object */
        hOspi->rdDummyCycles = dummyCycles;
    }
    else
    {
        /* do nothing */
    }
}

void OSPI_lld_setWriteDummyCycles(OSPILLD_Handle hOspi, uint32_t dummyCycles)
{
    /* Check if hOspi is NULL */
    if(hOspi != NULL)
    {
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        /* Set read dummy cycles */
        CSL_REG32_FINS(&pReg->DEV_INSTR_WR_CONFIG_REG, OSPI_FLASH_CFG_DEV_INSTR_WR_CONFIG_REG_DUMMY_WR_CLK_CYCLES_FLD, dummyCycles);

        /* Update book-keeping variable in OSPI object */
        hOspi->rdDummyCycles = dummyCycles;
    }
    else
    {
        /* do nothing */
    }
}

void OSPI_lld_setCmdDummyCycles(OSPILLD_Handle hOspi, uint32_t dummyCycles)
{
    /* Check if hOspi is NULL */
    if(hOspi != NULL)
    {
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        /* Set command dummy cycles */
        CSL_REG32_FINS(&pReg->FLASH_CMD_CTRL_REG, OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_NUM_DUMMY_CYCLES_FLD, dummyCycles);

        /* Update book-keeping variable in OSPI object */
        hOspi->cmdDummyCycles = dummyCycles;
    }
    else
    {
        /* do nothing */
    }
}

void OSPI_lld_setDualOpCodeMode(OSPILLD_Handle hOspi)
{
    /* Check if hOspi is NULL */
    if(hOspi != NULL)
    {
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        /* Enable dual byte opcode */
        CSL_REG32_FINS(&pReg->CONFIG_REG, OSPI_FLASH_CFG_CONFIG_REG_DUAL_BYTE_OPCODE_EN_FLD, TRUE);
    }
    else
    {
        /* do nothing */
    }
}

void OSPI_lld_clearDualOpCodeMode(OSPILLD_Handle hOspi)
{
    /* Check if hOspi is NULL */
    if(hOspi != NULL)
    {
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        /* Enable dual byte opcode */
        CSL_REG32_FINS(&pReg->CONFIG_REG, OSPI_FLASH_CFG_CONFIG_REG_DUAL_BYTE_OPCODE_EN_FLD, FALSE);
    }
    else
    {
        /* do nothing */
    }
}

void OSPI_lld_setXferOpCodes(OSPILLD_Handle hOspi, uint8_t readCmd, uint8_t pageProgCmd)
{
    /* Check if hOspi is NULL */
    if(hOspi != NULL)
    {
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);
        uint8_t cmdExt = OSPI_CMD_INVALID_OPCODE;

        /* Set opcode for read */
        CSL_REG32_FINS(&pReg->DEV_INSTR_RD_CONFIG_REG, OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_RD_OPCODE_NON_XIP_FLD, readCmd);

        /* Set the extended opcode for read */
        cmdExt = OSPI_getCmdExt(hOspi, readCmd);

        if(cmdExt != OSPI_CMD_INVALID_OPCODE)
        {
            CSL_REG32_FINS(&pReg->OPCODE_EXT_LOWER_REG, OSPI_FLASH_CFG_OPCODE_EXT_LOWER_REG_EXT_READ_OPCODE_FLD, cmdExt);
        }

        /* Set opcode for write */
        CSL_REG32_FINS(&pReg->DEV_INSTR_WR_CONFIG_REG, OSPI_FLASH_CFG_DEV_INSTR_WR_CONFIG_REG_WR_OPCODE_FLD, pageProgCmd);

        /* Set the extended opcode for read */
        cmdExt = OSPI_getCmdExt(hOspi, pageProgCmd);

        if(cmdExt != OSPI_CMD_INVALID_OPCODE)
        {
            CSL_REG32_FINS(&pReg->OPCODE_EXT_LOWER_REG, OSPI_FLASH_CFG_OPCODE_EXT_LOWER_REG_EXT_WRITE_OPCODE_FLD, cmdExt);
        }
    }
    else
    {
        /* do nothing */
    }
}

void OSPI_lld_setCmdExtType(OSPILLD_Handle hOspi, uint32_t cmdExtType)
{
    /* Check if hOspi is NULL */
    if(NULL != hOspi)
    {
        hOspi->cmdExtType = cmdExtType;
    }
    else
    {
        /* do nothing */
    }
}

int32_t OSPI_lld_enableDacMode(OSPILLD_Handle hOspi)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;

    /* Check if hOspi is NULL */
    if(NULL != hOspi)
    {
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        CSL_REG32_FINS(&pReg->CONFIG_REG,
                       OSPI_FLASH_CFG_CONFIG_REG_ENB_DIR_ACC_CTLR_FLD,
                       1);
        CSL_REG32_WR(&pReg->IND_AHB_ADDR_TRIGGER_REG, 0x04000000);
        hOspi->hOspiInit->dacEnable = TRUE;
    }
    else
    {
        status = OSPI_SYSTEM_FAILURE;
    }

    return status;
}

int32_t OSPI_lld_disableDacMode(OSPILLD_Handle hOspi)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;

    /* Check if hOspi is NULL */
    if(NULL != hOspi)
    {
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        CSL_REG32_FINS(&pReg->CONFIG_REG,
                       OSPI_FLASH_CFG_CONFIG_REG_ENB_DIR_ACC_CTLR_FLD,
                       0U);
        CSL_REG32_WR(&pReg->IND_AHB_ADDR_TRIGGER_REG, 0);
        hOspi->hOspiInit->dacEnable = FALSE;
    }
    else
    {
        status = OSPI_SYSTEM_FAILURE;
    }

    return status;
}

int32_t OSPI_lld_enablePhyPipeline(OSPILLD_Handle hOspi)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;

    /* Check if hOspi is NULL */
    if(NULL != hOspi)
    {
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        /* Enable PHY pipeline */
            CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_PIPELINE_PHY_FLD,
                   TRUE);
    }
    else
    {
        status = OSPI_SYSTEM_FAILURE;
    }

    return status;
}

int32_t OSPI_lld_disablePhyPipeline(OSPILLD_Handle hOspi)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;

    /* Check if hOspi is NULL */
    if(NULL != hOspi)
    {
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        /* Disable PHY pipeline */
            CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_PIPELINE_PHY_FLD,
                   FALSE);
    }
    else
    {
        status = OSPI_SYSTEM_FAILURE;
    }

    return status;
}

int32_t OSPI_lld_enablePhy(OSPILLD_Handle hOspi)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;

    /* Check if hOspi is NULL */
    if(NULL != hOspi)
    {
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        uint32_t phyEnable = CSL_REG32_FEXT(&pReg->CONFIG_REG,
                                            OSPI_FLASH_CFG_CONFIG_REG_PHY_MODE_ENABLE_FLD);
        if(phyEnable == FALSE)
        {
            /* Set dummyClks 1 less */
            uint32_t dummyClks = hOspi->rdDummyCycles - 1;

            /* Set new dummyClk */
            CSL_REG32_FINS(&pReg->DEV_INSTR_RD_CONFIG_REG,
                            OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_DUMMY_RD_CLK_CYCLES_FLD,
                            dummyClks);

            if(hOspi->phyRdDataCapDelay != 0xFF)
            {
                /* Tuning is already done, set read capture delay */
                CSL_REG32_FINS(&pReg->RD_DATA_CAPTURE_REG,
                           OSPI_FLASH_CFG_RD_DATA_CAPTURE_REG_DELAY_FLD,
                           hOspi->phyRdDataCapDelay);

            }

            /* Enable PHY mode */
            CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_PHY_MODE_ENABLE_FLD,
                   TRUE);
        }
    }
    else
    {
        status = OSPI_SYSTEM_FAILURE;
    }

    return status;
}

int32_t OSPI_lld_disablePhy(OSPILLD_Handle hOspi)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;

    /* Check if hOspi is NULL */
    if(NULL != hOspi)
    {
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        uint32_t phyEnable = CSL_REG32_FEXT(&pReg->CONFIG_REG,
                                            OSPI_FLASH_CFG_CONFIG_REG_PHY_MODE_ENABLE_FLD);

        if(phyEnable == TRUE)
        {
            uint32_t dummyClks = hOspi->rdDummyCycles;

            /* Set new dummyClk */
            CSL_REG32_FINS(&pReg->DEV_INSTR_RD_CONFIG_REG,
                            OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_DUMMY_RD_CLK_CYCLES_FLD,
                            dummyClks);

            /* Set the non-PHY read delay */
            CSL_REG32_FINS(&pReg->RD_DATA_CAPTURE_REG,
                       OSPI_FLASH_CFG_RD_DATA_CAPTURE_REG_DELAY_FLD,
                       hOspi->rdDataCapDelay);

            /* Disable PHY mode */
            CSL_REG32_FINS(&pReg->CONFIG_REG,
               OSPI_FLASH_CFG_CONFIG_REG_PHY_MODE_ENABLE_FLD,
               FALSE);

        }
    }
    else
    {
        status = OSPI_SYSTEM_FAILURE;
    }

    return status;
}

uint32_t OSPI_lld_getFlashDataBaseAddr(OSPILLD_Handle hOspi)
{
    uint32_t dataBaseAddr = 0U;

    /* Check if hOspi is NULL */
    if(NULL != hOspi)
    {
        /* Get the data base Addr */
        dataBaseAddr = hOspi->hOspiInit->dataBaseAddr;
    }

    return dataBaseAddr;
}

/* Different OSPI Read functions */
int32_t OSPI_lld_readCmd(OSPILLD_Handle hOspi, OSPI_ReadCmdParams *rdParams)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;
    const CSL_ospi_flash_cfgRegs *pReg;
    uint8_t *pBuf = (uint8_t *) rdParams->rxDataBuf;
    uint32_t rxLen = rdParams->rxDataLen;

    /* Check if hOspi is NULL */
    if(hOspi != NULL)
    {
        pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        /* Clear flash command control register */
        CSL_REG32_WR(&pReg->FLASH_CMD_CTRL_REG, 0U);

        /* Set command opcode */
        CSL_REG32_FINS(&pReg->FLASH_CMD_CTRL_REG, OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_CMD_OPCODE_FLD, rdParams->cmd);

        /* Enable read data in command control register */
        CSL_REG32_FINS(&pReg->FLASH_CMD_CTRL_REG, OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_ENB_READ_DATA_FLD, TRUE);

        /* Set number of read data bytes */
        CSL_REG32_FINS(&pReg->FLASH_CMD_CTRL_REG, OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_NUM_RD_DATA_BYTES_FLD, rxLen-1);

        /* Set dummyCycles for the command */
        if(rdParams->dummyBits != OSPI_CMD_INVALID_DUMMY)
        {
            CSL_REG32_FINS(&pReg->FLASH_CMD_CTRL_REG, OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_NUM_DUMMY_CYCLES_FLD, rdParams->dummyBits);
        }
        else
        {
            CSL_REG32_FINS(&pReg->FLASH_CMD_CTRL_REG, OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_NUM_DUMMY_CYCLES_FLD, hOspi->cmdDummyCycles);
        }

        uint32_t dualOpCode = CSL_REG32_FEXT(&pReg->CONFIG_REG,
                                    OSPI_FLASH_CFG_CONFIG_REG_DUAL_BYTE_OPCODE_EN_FLD);

        if(dualOpCode == 1)
        {
            uint8_t cmdExt = OSPI_getCmdExt(hOspi, rdParams->cmd);
            /* Set extended STIG opcode */
            CSL_REG32_FINS(&pReg->OPCODE_EXT_LOWER_REG, OSPI_FLASH_CFG_OPCODE_EXT_LOWER_REG_EXT_STIG_OPCODE_FLD, cmdExt);
        }
        else
        {
            /* do nothing */
        }

        if(rdParams->cmdAddr != OSPI_CMD_INVALID_ADDR && (rdParams->numAddrBytes > 0))
        {
            /* Enable Command address in command control register */
            CSL_REG32_FINS(&pReg->FLASH_CMD_CTRL_REG, OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_ENB_COMD_ADDR_FLD, TRUE);

            /* Set number of address bytes */
            CSL_REG32_FINS(&pReg->FLASH_CMD_CTRL_REG, OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_NUM_ADDR_BYTES_FLD, rdParams->numAddrBytes - 1);

            /* Update the flash cmd address register */
            CSL_REG32_WR(&pReg->FLASH_CMD_ADDR_REG, rdParams->cmdAddr);
        }
        else
        {
            /* do nothing */
        }

        status = OSPI_flashExecCmd(hOspi, pReg, rdParams->readTimeout);

        if(status == 0)
        {
            uint32_t regVal = CSL_REG32_RD(&pReg->FLASH_RD_DATA_LOWER_REG);
            uint32_t rdLen = (rxLen > 4U) ? 4U : rxLen;
            (void)memcpy((void *)pBuf, (void *)(&regVal), rdLen);
            pBuf += rdLen;

            if(rxLen > 4U)
            {
                regVal = CSL_REG32_RD(&pReg->FLASH_RD_DATA_UPPER_REG);
                rdLen = rxLen - rdLen;
                (void)memcpy((void *)pBuf, (void *)(&regVal), rdLen);
            }
        }
    }
    else
    {
        status = OSPI_SYSTEM_FAILURE;
    }
    return status;
}

int32_t OSPI_lld_readDirect(OSPILLD_Handle hOspi, OSPI_Transaction *trans)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;
    uint8_t *pSrc;
    uint8_t *pDst;
    uint32_t addrOffset;
    const CSL_ospi_flash_cfgRegs *pReg;
    OSPILLD_InitHandle hOspiInit;

    /* Check if hOspi & OSPI_transactions are NULL */
    if((NULL != hOspi) && (NULL != trans))
    {
        pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);
        hOspiInit = hOspi->hOspiInit;
        addrOffset = trans->addrOffset;
        pDst = (uint8_t *) trans->buf;

        /* Enable Direct Access Mode */
        CSL_REG32_FINS(&pReg->CONFIG_REG,
                    OSPI_FLASH_CFG_CONFIG_REG_ENB_DIR_ACC_CTLR_FLD,
                    1);
        CSL_REG32_WR(&pReg->IND_AHB_ADDR_TRIGGER_REG, 0x04000000);

        pSrc = (uint8_t *)(hOspiInit->dataBaseAddr + addrOffset);

        memcpy(pDst, pSrc, trans->count);
    }
    else
    {
        status = OSPI_LLD_INVALID_PARAM;
    }

    return status;
}

int32_t OSPI_lld_readDirectDma(OSPILLD_Handle hOspi, OSPI_Transaction *trans)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;
    uint8_t *pSrc;
    uint8_t *pDst;
    uint32_t addrOffset;
    const CSL_ospi_flash_cfgRegs *pReg;
    OSPILLD_InitHandle hOspiInit;

    /* Check if hOspi & OSPI_transactions are NULL */
    if((NULL != hOspi) && (NULL != trans))
    {
        pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);
        hOspiInit = hOspi->hOspiInit;
        addrOffset = trans->addrOffset;
        pDst = (uint8_t *) trans->buf;

        /* Enable Direct Access Mode */
        CSL_REG32_FINS(&pReg->CONFIG_REG,
                    OSPI_FLASH_CFG_CONFIG_REG_ENB_DIR_ACC_CTLR_FLD,
                    1);
        CSL_REG32_WR(&pReg->IND_AHB_ADDR_TRIGGER_REG, 0x04000000);

        pSrc = (uint8_t *)(hOspiInit->dataBaseAddr + addrOffset);

        /* DMA Copy fails when copying to to certain memory regions. So in this case we switch to normal memcpy
        for copying even if dmaEnable is true. Also do DMA copy only if size > 1KB*/
        uint32_t isDmaCopy = (OSPI_isDmaRestrictedRegion(hOspi, (uint32_t)pDst) == FALSE) &&
                             (trans->count > OSPI_DMA_COPY_LOWER_LIMIT);     

        if(isDmaCopy == TRUE)
        {
            uint8_t *tempSrc = pSrc;
            uint8_t *tempDst = pDst;
            uint32_t remainingBytes = trans->count;

            /* Check for 32B alignment of source address */
            if(((uint32_t)pSrc % OSPI_DMA_COPY_SRC_ALIGNMENT) != 0)
            {
                uint32_t initResidualBytes = OSPI_DMA_COPY_SRC_ALIGNMENT - (((uint32_t)pSrc) % OSPI_DMA_COPY_SRC_ALIGNMENT);

                /* Do CPU copy for the initial residual bytes */
                memcpy(pDst, pSrc, initResidualBytes);

                tempDst = (uint8_t *)((uint32_t)pDst + initResidualBytes);
                tempSrc = (uint8_t *)((uint32_t)pSrc + initResidualBytes);
                remainingBytes -= initResidualBytes;
            }

            /* Do DMA copy for 32B-aligned bytes */
            uint32_t unalignedBytes = (remainingBytes % OSPI_DMA_COPY_SIZE_ALIGNMENT);

            /* Do a CPU copy of unaligned bytes if any at the end */
            if(unalignedBytes > 0)
            {
                tempDst += (remainingBytes - unalignedBytes);
                tempSrc += (remainingBytes - unalignedBytes);
                memcpy(tempDst, tempSrc, unalignedBytes);
            }
#if defined(SOC_AM64X) || defined(SOC_AM243X)
            /* Enable PHY Pipeline only if phy is enabled */
            uint32_t phyEnable = CSL_REG32_FEXT(&pReg->CONFIG_REG,
                                                OSPI_FLASH_CFG_CONFIG_REG_PHY_MODE_ENABLE_FLD);
            if(phyEnable == TRUE)
            {
                /* Enable PHY pipeline */
                CSL_REG32_FINS(&pReg->CONFIG_REG,
                    OSPI_FLASH_CFG_CONFIG_REG_PIPELINE_PHY_FLD,
                    TRUE);
            }
#else
            if(hOspiInit->phyEnable == TRUE)
            {
                /* Enable PHY pipeline */
                CSL_REG32_FINS(&pReg->CONFIG_REG,
                    OSPI_FLASH_CFG_CONFIG_REG_PIPELINE_PHY_FLD,
                    TRUE);
            }
#endif
            if(remainingBytes > 0)
            {
                /* Move dest and src pointers back in case of dangling bytes */
                if(unalignedBytes > 0)
                {
                    tempDst -= (remainingBytes - unalignedBytes);
                    tempSrc -= (remainingBytes - unalignedBytes);
                }
                else
                {
                    /* There wasn't any non-aligned bytes in the transfer */
                }
                if (trans->transferTimeout == 0)
                {
                    trans->transferTimeout = 5000;
                }
                hOspi->currTrans->state = OSPI_TRANSFER_MODE_BLOCKING;
                OSPI_dmaCopy(hOspi, tempDst, tempSrc, remainingBytes - unalignedBytes, trans->transferTimeout);
            }
        }
        else
        {
            hOspi->currTrans->state = OSPI_TRANSFER_MODE_POLLING;
            memcpy(pDst, pSrc, trans->count);
        }
    }
    else
    {
        status = OSPI_LLD_INVALID_PARAM;
    }

    return status;
}

int32_t OSPI_lld_readIndirect(OSPILLD_Handle hOspi, OSPI_Transaction *trans)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;
    const CSL_ospi_flash_cfgRegs *pReg;
    uint8_t *pDst;
    uint32_t addrOffset;
    uint32_t remainingSize;
    uint32_t readFlag = 0U;
    uint32_t sramLevel = 0, readBytes = 0;
    OSPILLD_InitHandle hOspiInit;

    /* Check if hOspi & OSPI_transactions are NULL */
    if((NULL != hOspi) && (NULL != trans))
    {
        pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);
        hOspiInit = hOspi->hOspiInit;
        addrOffset = trans->addrOffset;
        pDst = (uint8_t *) trans->buf;

        OSPI_disableInterrupt(hOspi);
        OSPI_clearInterrupt(hOspi);
        if(TRUE == hOspi->hOspiInit->intrEnable)
        {
            OSPI_enableInterrupt(hOspi);
            hOspi->currTrans->addrOffset = trans->addrOffset;
            hOspi->currTrans->buf = trans->buf;
            hOspi->currTrans->count = trans->count;
            hOspi->currTrans->dataLen = trans->dataLen;
            hOspi->currTrans->transferOffset = 0U;
            hOspi->currTrans->state = OSPI_TRANS_READ;
            hOspi->currTrans->status = trans->status;
        }

        /* Disable DAC Mode */
        CSL_REG32_FINS(&pReg->CONFIG_REG,
                    OSPI_FLASH_CFG_CONFIG_REG_ENB_DIR_ACC_CTLR_FLD,
                    0U);
        CSL_REG32_WR(&pReg->IND_AHB_ADDR_TRIGGER_REG, 0);

        /* Config the Indirect Read Transfer Start Address Register */
        CSL_REG32_WR(&pReg->INDIRECT_READ_XFER_START_REG, addrOffset);

        /* Set the Indirect Write Transfer Start Address Register */
        CSL_REG32_WR(&pReg->INDIRECT_READ_XFER_NUM_BYTES_REG, trans->count);

        /* Set the Indirect Write Transfer Watermark Register */
        CSL_REG32_WR(&pReg->INDIRECT_READ_XFER_WATERMARK_REG,
                    CSL_OSPI_SRAM_WARERMARK_RD_LVL);

        /* Start the indirect read transfer */
        CSL_REG32_FINS(&pReg->INDIRECT_READ_XFER_CTRL_REG,
                    OSPI_FLASH_CFG_INDIRECT_READ_XFER_CTRL_REG_START_FLD,
                    1);

        if(OSPI_TRANSFER_MODE_POLLING == hOspi->transferMode)
        {
            remainingSize = trans->count;

            while(remainingSize > 0U)
            {
                if(OSPI_waitReadSRAMLevel(pReg, &sramLevel) != 0)
                {
                    /* SRAM FIFO has no data, failure */
                    readFlag = 1U;
                    status = OSPI_SYSTEM_FAILURE;
                    trans->status = OSPI_TRANSFER_FAILED;
                    break;
                }

                readBytes = sramLevel * CSL_OSPI_FIFO_WIDTH;
                readBytes = (readBytes > remainingSize) ? remainingSize : readBytes;

                /* Read data from FIFO */
                OSPI_readFifoData(hOspiInit->dataBaseAddr, pDst, readBytes);

                pDst += readBytes;
                remainingSize -= readBytes;
            }
            /* Wait for completion of INDAC Read */
            if(readFlag == 0U && OSPI_waitIndReadComplete(pReg) != 0)
            {
                readFlag = 1U;
                status = OSPI_SYSTEM_FAILURE;
                trans->status = OSPI_TRANSFER_FAILED;
            }

        }
    }
    else
    {
        status = OSPI_LLD_INVALID_PARAM;
    }

    return status;
}

/* Different OSPI write functions */
int32_t OSPI_lld_writeCmd(OSPILLD_Handle hOspi, OSPI_WriteCmdParams *wrParams)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;
    const CSL_ospi_flash_cfgRegs *pReg;
    uint8_t *txBuf = (uint8_t *) wrParams->txDataBuf;
    uint32_t txLen = wrParams->txDataLen;

    /* Check if hOspi is NULL */
    if(hOspi != NULL)
    {
        pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        /* Clear the flash command control register */
        CSL_REG32_WR(&pReg->FLASH_CMD_CTRL_REG, 0U);

        /* Set command opcode */
        CSL_REG32_FINS(&pReg->FLASH_CMD_CTRL_REG, OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_CMD_OPCODE_FLD, wrParams->cmd);

        /* Set command address if needed */
        if(wrParams->cmdAddr != OSPI_CMD_INVALID_ADDR)
        {
            /* Enable Command address in command control register */
            CSL_REG32_FINS(&pReg->FLASH_CMD_CTRL_REG, OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_ENB_COMD_ADDR_FLD, TRUE);

            /* Set number of address bytes */
            CSL_REG32_FINS(&pReg->FLASH_CMD_CTRL_REG, OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_NUM_ADDR_BYTES_FLD, wrParams->numAddrBytes - 1);

            /* Update the flash cmd address register */
            CSL_REG32_WR(&pReg->FLASH_CMD_ADDR_REG, wrParams->cmdAddr);
        }
        else
        {
            /* do nothing */
        }

        uint32_t dualOpCode = CSL_REG32_FEXT(&pReg->CONFIG_REG,
                                    OSPI_FLASH_CFG_CONFIG_REG_DUAL_BYTE_OPCODE_EN_FLD);

        if(dualOpCode == 1)
        {
            uint8_t cmdExt = OSPI_getCmdExt(hOspi, wrParams->cmd);
            /* Set extended STIG opcode */
            CSL_REG32_FINS(&pReg->OPCODE_EXT_LOWER_REG, OSPI_FLASH_CFG_OPCODE_EXT_LOWER_REG_EXT_STIG_OPCODE_FLD, cmdExt);
        }
        else
        {
            /* do nothing */
        }

        if (txLen != 0U)
        {
            uint32_t wrLen = 0;
            uint32_t wrData = 0;

            /* Enable write data in command control register */
            CSL_REG32_FINS(&pReg->FLASH_CMD_CTRL_REG, OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_ENB_WRITE_DATA_FLD, TRUE);

            /* Set number of data bytes to write */
            CSL_REG32_FINS(&pReg->FLASH_CMD_CTRL_REG, OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_NUM_WR_DATA_BYTES_FLD, txLen-1);

            wrLen = txLen > 4U ? 4U : txLen;
            memcpy(&wrData, txBuf, wrLen);
            CSL_REG32_WR(&pReg->FLASH_WR_DATA_LOWER_REG, wrData);

            if (txLen > 4U)
            {
                txBuf += wrLen;
                wrLen = txLen - wrLen;
                memcpy(&wrData, txBuf, wrLen);
                CSL_REG32_WR(&pReg->FLASH_WR_DATA_UPPER_REG, wrData);
            }
        }
        else
        {
            /* do nothing */
        }

        status = OSPI_flashExecCmd(hOspi, pReg, wrParams->writeTimeout);
    }
    else
    {
        status = OSPI_SYSTEM_FAILURE;
    }
    return status;
}

int32_t OSPI_lld_writeDirect(OSPILLD_Handle hOspi, OSPI_Transaction *trans)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;   
    const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

    uint8_t *pSrc;
    uint8_t *pDst;
    uint32_t addrOffset;

    addrOffset = trans->addrOffset;
    pSrc = (uint8_t *) trans->buf;

    /* Enable Direct Access Mode */
    CSL_REG32_FINS(&pReg->CONFIG_REG, OSPI_FLASH_CFG_CONFIG_REG_ENB_DIR_ACC_CTLR_FLD, 1);
    CSL_REG32_WR(&pReg->IND_AHB_ADDR_TRIGGER_REG, 0x04000000);

    pDst = (uint8_t *)(hOspi->hOspiInit->dataBaseAddr + addrOffset);

    uint8_t wrByte;
    uint32_t size, remainingSize;

    uint32_t *tempSrc = (uint32_t *)pSrc;
    uint32_t *tempDst = (uint32_t *)pDst;

    int i = 0;
    {
        remainingSize = trans->count & 3U;
        size = trans->count - remainingSize;

        for(i = 0; i < size; i+=4)
        {
            *tempDst = *tempSrc;
            tempDst++;
            tempSrc++;
        }

        //Dangling Bytes
        for(i = 0; i < remainingSize; i++)
        {
            wrByte = CSL_REG8_RD(pSrc + size + i);
            CSL_REG8_WR(pDst + size + i, wrByte);
        }
    }

    return status;
}

int32_t OSPI_lld_writeIndirect(OSPILLD_Handle hOspi, OSPI_Transaction *trans)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;
    const CSL_ospi_flash_cfgRegs *pReg;
    OSPILLD_InitHandle hOspiInit;
    uint8_t *pSrc;
    uint32_t addrOffset, remainingSize, sramLevel, wrBytes, wrFlag = 0;

    /* Check if hOspi & OSPI_transactions are NULL */
    if((NULL != hOspi) && (NULL != trans))
    {
        pReg = (const CSL_ospi_flash_cfgRegs *)hOspi->baseAddr;
        hOspiInit = hOspi->hOspiInit;
        addrOffset = trans->addrOffset;
        pSrc = (uint8_t *) trans->buf;

        OSPI_disableInterrupt(hOspi);
        OSPI_clearInterrupt(hOspi);
        if(TRUE == hOspi->hOspiInit->intrEnable)
        {
            OSPI_enableInterrupt(hOspi);
            hOspi->currTrans->addrOffset = trans->addrOffset;
            hOspi->currTrans->buf = trans->buf;
            hOspi->currTrans->count = trans->count;
            hOspi->currTrans->dataLen = trans->dataLen;
            hOspi->currTrans->transferOffset = 0U;
            hOspi->currTrans->state = OSPI_TRANS_WRITE;
            hOspi->currTrans->status = trans->status;
        }

        /* Disable DAC Mode */
        CSL_REG32_FINS(&pReg->CONFIG_REG,
                    OSPI_FLASH_CFG_CONFIG_REG_ENB_DIR_ACC_CTLR_FLD,
                    0U);

        CSL_REG32_WR(&pReg->IND_AHB_ADDR_TRIGGER_REG, 0);

        /* Set write address in indirect mode */
        CSL_REG32_WR(&pReg->INDIRECT_WRITE_XFER_START_REG, addrOffset);

        /* Set the Indirect Write Transfer Start Address Register */
        CSL_REG32_WR(&pReg->INDIRECT_WRITE_XFER_NUM_BYTES_REG, trans->count);

        /* Reset watermark register */
        CSL_REG32_WR(&pReg->INDIRECT_WRITE_XFER_WATERMARK_REG, 0);

        /* Set the Indirect Write Transfer Watermark Register */
        CSL_REG32_WR(&pReg->INDIRECT_WRITE_XFER_WATERMARK_REG,
                    CSL_OSPI_SRAM_WATERMARK_WR_LVL);

        /* Start the indirect write transfer */
        CSL_REG32_FINS(&pReg->INDIRECT_WRITE_XFER_CTRL_REG,
                    OSPI_FLASH_CFG_INDIRECT_WRITE_XFER_CTRL_REG_START_FLD,
                    1);

        if(OSPI_TRANSFER_MODE_POLLING == hOspi->transferMode)
        {
            if(OSPI_waitWriteSRAMLevel(pReg, &sramLevel) != 0)
            {
                wrFlag = 1U;
                status = OSPI_SYSTEM_FAILURE;
                trans->status = OSPI_TRANSFER_FAILED;
            }
            else
            {
                remainingSize = trans->count;
                while(remainingSize > 0U)
                {
                    if(OSPI_waitWriteSRAMLevel(pReg, &sramLevel) != 0)
                    {
                        wrFlag = 1U;
                        status = OSPI_SYSTEM_FAILURE;
                        break;
                    }

                    wrBytes = (CSL_OSPI_SRAM_PARTITION_WR - sramLevel) * CSL_OSPI_FIFO_WIDTH;
                    wrBytes = (wrBytes > remainingSize) ? remainingSize : wrBytes;

                    OSPI_writeFifoData(hOspiInit->dataBaseAddr, pSrc, wrBytes);

                    pSrc += wrBytes;
                    remainingSize -= wrBytes;
                }

                if(wrFlag == 0U && OSPI_waitIndWriteComplete(pReg) != 0)
                {
                    wrFlag = 1U;
                    status = -1;
                }
            }
        }

        if(wrFlag == 1U)
        {
            trans->status = OSPI_TRANSFER_FAILED;
            /* Cancel the indirect write */
            CSL_REG32_FINS(&pReg->INDIRECT_WRITE_XFER_CTRL_REG,
                    OSPI_FLASH_CFG_INDIRECT_WRITE_XFER_CTRL_REG_CANCEL_FLD,
                    1);
        }
    }
    else
    {
        status = OSPI_LLD_INVALID_PARAM;
    }

    return status;
}

int32_t OSPI_lld_configResetPin(OSPILLD_Handle hOspi, uint32_t config)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;

    if(NULL != hOspi)
    {
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        switch(config)
        {
            case OSPI_RESETPIN_DQ3:
                CSL_REG32_FINS(&pReg->CONFIG_REG,
                                OSPI_FLASH_CFG_CONFIG_REG_RESET_CFG_FLD,
                                FALSE);
                break;
            case OSPI_RESETPIN_DEDICATED:
            default:
                CSL_REG32_FINS(&pReg->CONFIG_REG,
                                OSPI_FLASH_CFG_CONFIG_REG_RESET_CFG_FLD,
                                TRUE);
        }
    }
    else
    {
        status = OSPI_SYSTEM_FAILURE;
    }

    return status;
}

int32_t OSPI_lld_setResetPinStatus(OSPILLD_Handle hOspi, uint32_t pinStatus)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;

    if(NULL != hOspi)
    {
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        CSL_REG32_FINS(&pReg->CONFIG_REG,
                                OSPI_FLASH_CFG_CONFIG_REG_RESET_PIN_FLD,
                                pinStatus);
    }
    else
    {
        status = OSPI_SYSTEM_FAILURE;
    }

    return status;
}

int32_t OSPI_lld_configBaudrate(OSPILLD_Handle hOspi, uint32_t baud)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;

    if(NULL != hOspi && (baud <= 32) && (baud >= 2) && (baud % 2) == 0)
    {
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);
        CSL_REG32_FINS(&pReg->CONFIG_REG, OSPI_FLASH_CFG_CONFIG_REG_MSTR_BAUD_DIV_FLD, CSL_OSPI_BAUD_RATE_DIVISOR(baud));
    }
    else
    {
        status = OSPI_SYSTEM_FAILURE;
    }

    return status;
}

int32_t OSPI_lld_readBaudRateDivFromReg(OSPILLD_Handle hOspi, uint32_t *baudDiv)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;
    if(NULL != baudDiv && NULL != hOspi)
    {
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);
        uint32_t progBaudDiv = CSL_REG32_FEXT(&pReg->CONFIG_REG, OSPI_FLASH_CFG_CONFIG_REG_MSTR_BAUD_DIV_FLD);
        // see CSL_OSPI_BAUD_RATE_DIVISOR
        progBaudDiv = (progBaudDiv << 1) + 2;
        *baudDiv = progBaudDiv;
    }
    else
    {
        status = OSPI_SYSTEM_FAILURE;
    }
    return status;
}

int32_t OSPI_lld_getBaudRateDivFromObj(OSPILLD_Handle hOspi, uint32_t *baudDiv)
{

    int32_t status = OSPI_SYSTEM_SUCCESS;
    if(NULL != baudDiv && NULL != hOspi)
    {
        *baudDiv = hOspi->hOspiInit->baudRateDiv;
    }
    else
    {
        status = OSPI_SYSTEM_FAILURE;
    }
    return status;
}

/* Internal function definitions */
static int32_t OSPI_programInstance(OSPILLD_Handle hOspi)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;
    uint32_t regVal;
    const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)hOspi->baseAddr;
    OSPILLD_InitHandle hOspiInit = hOspi->hOspiInit;
    // uint32_t protocol;

    /* Do the register programming to set the modes from the config */
    /* Optimal programming setup */
    /* Disable DAC */
    CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_ENB_DIR_ACC_CTLR_FLD,
                   0);
    /* Disable DTR */
    CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_ENABLE_DTR_PROTOCOL_FLD,
                   0);
    CSL_REG32_FINS(&pReg->DEV_INSTR_RD_CONFIG_REG,
                   OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_DDR_EN_FLD,
                   0);
    /* Disable XIP */
    CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_ENTER_XIP_MODE_FLD,
                   0);
    /* Disable OSPI Controller */
    CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_ENB_SPI_FLD,
                   0);
    /* Wait until Serial Interface and OSPI pipeline is IDLE. */
    if (OSPI_waitIdle(hOspi, OSPI_READ_WRITE_TIMEOUT) != (int32_t)0U)
    {
        OSPI_lld_deInit(hOspi);
        status = OSPI_SYSTEM_FAILURE;
    }

    if(OSPI_SYSTEM_SUCCESS == status)
    {
        /* User config */
        if(TRUE == hOspiInit->intrEnable)
        {
            hOspi->transferMode = OSPI_TRANSFER_MODE_BLOCKING;
        }
        else
        {
            hOspi->transferMode = OSPI_TRANSFER_MODE_POLLING;
        }
        /* Chip Select */
        regVal = CSL_REG32_RD(&pReg->CONFIG_REG);
        uint32_t chipSelect = OSPI_CHIP_SELECT(hOspiInit->chipSelect);
        uint32_t decSelect = hOspiInit->decChipSelect;

        regVal &= ~(CSL_OSPI_FLASH_CFG_CONFIG_REG_PERIPH_SEL_DEC_FLD_MASK | \
                    CSL_OSPI_FLASH_CFG_CONFIG_REG_PERIPH_CS_LINES_FLD_MASK);
        regVal |= (decSelect << CSL_OSPI_FLASH_CFG_CONFIG_REG_PERIPH_SEL_DEC_FLD_SHIFT) | \
                  (chipSelect << CSL_OSPI_FLASH_CFG_CONFIG_REG_PERIPH_CS_LINES_FLD_SHIFT);
        CSL_REG32_WR(&pReg->CONFIG_REG, regVal);
        /* Frame format */
        regVal = CSL_REG32_RD(&pReg->CONFIG_REG);
        regVal &= ~(CSL_OSPI_FLASH_CFG_CONFIG_REG_SEL_CLK_PHASE_FLD_MASK | \
                    CSL_OSPI_FLASH_CFG_CONFIG_REG_SEL_CLK_POL_FLD_MASK);
        regVal |= (hOspiInit->frmFmt);
        CSL_REG32_WR(&pReg->CONFIG_REG, regVal);
        /* Disable the adapted loop-back clock circuit */
        CSL_REG32_FINS(&pReg->RD_DATA_CAPTURE_REG,
                   OSPI_FLASH_CFG_RD_DATA_CAPTURE_REG_BYPASS_FLD,
                   1);
        /* Delay Setup */
#if defined(SOC_AM64X) || defined(SOC_AM243X)
        OSPI_lld_setDelays(hOspi, hOspiInit->inputClkFreq);
        OSPI_lld_setBaudRateDiv(hOspi, hOspiInit->baudRateDiv);
#else
        uint32_t delays[4] = { 10, 10, 10, 10 };
        uint32_t devDelay = ((delays[0] << CSL_OSPI_FLASH_CFG_DEV_DELAY_REG_D_INIT_FLD_SHIFT)  | \
                      (delays[1] << CSL_OSPI_FLASH_CFG_DEV_DELAY_REG_D_AFTER_FLD_SHIFT) | \
                      (delays[2] << CSL_OSPI_FLASH_CFG_DEV_DELAY_REG_D_BTWN_FLD_SHIFT)  | \
                      (delays[3] << CSL_OSPI_FLASH_CFG_DEV_DELAY_REG_D_NSS_FLD_SHIFT));
        CSL_REG32_WR(&pReg->DEV_DELAY_REG, devDelay);

        if(hOspiInit->baudRateDiv)
        {
            CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_MSTR_BAUD_DIV_FLD,
                   CSL_OSPI_BAUD_RATE_DIVISOR(hOspiInit->baudRateDiv));
        }
        else
        {

            CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_MSTR_BAUD_DIV_FLD,
                   CSL_OSPI_BAUD_RATE_DIVISOR_DEFAULT);
        }
#endif

        /* Disable PHY pipeline mode */
        CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_PIPELINE_PHY_FLD,
                   FALSE);

        /* Disable PHY mode by default. This will be later enabled from flash driver */
        CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_PHY_MODE_ENABLE_FLD,
                   FALSE);

        /* Set indirect trigger address register */
        if(hOspiInit->dacEnable)
        {
            CSL_REG32_FINS(&pReg->CONFIG_REG,
                       OSPI_FLASH_CFG_CONFIG_REG_ENB_DIR_ACC_CTLR_FLD,
                       1);
            CSL_REG32_WR(&pReg->IND_AHB_ADDR_TRIGGER_REG, 0x4000000);
        }
        else
        {
            CSL_REG32_FINS(&pReg->CONFIG_REG,
                       OSPI_FLASH_CFG_CONFIG_REG_ENB_DIR_ACC_CTLR_FLD,
                       0U);
            CSL_REG32_WR(&pReg->IND_AHB_ADDR_TRIGGER_REG, 0);
        }

        /* Disable write completion auto polling */
        CSL_REG32_FINS(&pReg->WRITE_COMPLETION_CTRL_REG,
                       OSPI_FLASH_CFG_WRITE_COMPLETION_CTRL_REG_DISABLE_POLLING_FLD,
                       1);
        /* Set and invalid opcode in the WRITE_COMPLETION_CTRL_REG due to IP limitation */
        CSL_REG32_FINS(&pReg->WRITE_COMPLETION_CTRL_REG,
                       OSPI_FLASH_CFG_WRITE_COMPLETION_CTRL_REG_OPCODE_FLD,
                       OSPI_CMD_INVALID_OPCODE);
        CSL_REG32_FINS(&pReg->WRITE_COMPLETION_CTRL_REG,
                       OSPI_FLASH_CFG_WRITE_COMPLETION_CTRL_REG_POLL_COUNT_FLD,
                       3U);

        /* Disable dual byte opcode. If OSPI boot mode was used, ROM would have set this. This can cause 1s mode applications to fail */
        CSL_REG32_FINS(&pReg->CONFIG_REG, OSPI_FLASH_CFG_CONFIG_REG_DUAL_BYTE_OPCODE_EN_FLD, FALSE);

        /* Set SRAM partition configuration */
        CSL_REG32_WR(&pReg->SRAM_PARTITION_CFG_REG, CSL_OSPI_SRAM_PARTITION_DEFAULT);

        /* Disable and clear the interrupts */
        regVal = CSL_REG32_RD(&pReg->IRQ_MASK_REG);
        regVal &= ~(CSL_OSPI_INTR_MASK_ALL);
        CSL_REG32_WR(&pReg->IRQ_MASK_REG, regVal);

        CSL_REG32_WR(&pReg->IRQ_STATUS_REG, CSL_OSPI_INTR_MASK_ALL);

        /* Enable/Disable DAC */
        CSL_REG32_FINS(&pReg->CONFIG_REG,
               OSPI_FLASH_CFG_CONFIG_REG_ENB_DIR_ACC_CTLR_FLD,
               hOspiInit->dacEnable);

        /* Initialize read delay and related book-keeping variables */
        hOspi->phyRdDataCapDelay = 0xFF;
        OSPI_lld_setRdDataCaptureDelay(hOspi, 0);

        /* Initialise controller to 1s1s1s mode to override any ROM settings */

        /* Set initial protocol to be 1s1s1s */
        OSPI_lld_setProtocol(hOspi, OSPI_NOR_PROTOCOL(1,1,1,0));
    
        OSPI_lld_setXferOpCodes(hOspi, 0x03, 0x02);

        /* Set address bytes to 3 */
        OSPI_lld_setNumAddrBytes(hOspi, 3);

        /* Initialize phy enable status */
        hOspi->phyEnableSuccess = FALSE;

        OSPI_lld_configResetPin(hOspi,OSPI_RESETPIN_DEDICATED);
        
        /* Enable OSPI Controller */
        CSL_REG32_FINS(&pReg->CONFIG_REG,
                       OSPI_FLASH_CFG_CONFIG_REG_ENB_SPI_FLD,
                       1);
    }

    return status;
}

static uint8_t OSPI_getCmdExt(OSPILLD_Handle hOspi, uint8_t cmd)
{
    uint8_t cmdExt = OSPI_CMD_INVALID_OPCODE;

    /* Get the command Ext Type */
    switch(hOspi->cmdExtType)
    {
        case OSPI_CMD_EXT_TYPE_REPEAT:
            cmdExt = cmd;
            break;
        case OSPI_CMD_EXT_TYPE_INVERSE:
            cmdExt = ~cmd;
            break;
        case OSPI_CMD_EXT_TYPE_NONE:
            cmdExt = OSPI_CMD_INVALID_OPCODE;
            break;
        default:
            cmdExt = OSPI_CMD_INVALID_OPCODE;
            break;
    }

    return cmdExt;
}

static int32_t OSPI_isDmaRestrictedRegion(OSPILLD_Handle hOspi, uint32_t addr)
{
    int32_t isRestricted = FALSE;
    OSPILLD_InitHandle hOspiInit = hOspi->hOspiInit;

    /* Check if OSPI is in DMA restricted Region */
    if(NULL != hOspiInit->dmaRestrictedRegions)
    {
        const OSPI_AddrRegion *addrRegions = hOspiInit->dmaRestrictedRegions;
        uint32_t i = 0;
        uint32_t start;
        uint32_t size;

        while(addrRegions[i].regionStartAddr != 0xFFFFFFFF)
        {
            start = addrRegions[i].regionStartAddr;
            size = addrRegions[i].regionSize;

            if((addr >= start) && (addr < (start + size)))
            {
                isRestricted = TRUE;
                break;
            }
            i++;
        }
    }

    return isRestricted;
}

static int32_t OSPI_waitReadSRAMLevel(const CSL_ospi_flash_cfgRegs *pReg, uint32_t *rdLevel)
{
    uint32_t retry = OSPI_READ_WRITE_TIMEOUT;
    uint32_t sramLevel;
    int32_t retVal = 0;

    /* Wait till SRAM Read level completed  */
    while(retry != 0U)
    {
        sramLevel = CSL_REG32_RD(&pReg->SRAM_FILL_REG) &
            CSL_OSPI_FLASH_CFG_SRAM_FILL_REG_SRAM_FILL_INDAC_READ_FLD_MASK;
        if(sramLevel != 0U)
        {
            *rdLevel = sramLevel;
            break;
        }
        uint32_t delay  = OSPI_CHECK_IDLE_DELAY;
        while(delay--);
        retry--;
    }

    if(retry != 0U)
    {
        retVal = 0;
    }
    else
    {
        retVal = -1;
    }

    return retVal;
}

static int32_t OSPI_waitIndReadComplete(const CSL_ospi_flash_cfgRegs *pReg)
{
    uint32_t retry = OSPI_READ_WRITE_TIMEOUT;
    int32_t retVal = 0;
    uint32_t regVal = 0U;

    /* Wait till OSPI Indirect Read is completed */
    while(retry != 0U)
    {
        regVal = CSL_REG32_FEXT(&pReg->INDIRECT_READ_XFER_CTRL_REG,
                         OSPI_FLASH_CFG_INDIRECT_READ_XFER_CTRL_REG_IND_OPS_DONE_STATUS_FLD);
        if(regVal == 1U)
        {
            break;
        }
        uint32_t delay = OSPI_CHECK_IDLE_DELAY;
        while(delay--);
        retry--;
    }

    if(retry != 0U)
    {
        /* Clear indirect completion status */
        CSL_REG32_FINS(&pReg->INDIRECT_READ_XFER_CTRL_REG,
                   OSPI_FLASH_CFG_INDIRECT_READ_XFER_CTRL_REG_IND_OPS_DONE_STATUS_FLD,
                   1);
    }
    else
    {
        retVal = -1;
    }

    return retVal;
}

static int32_t OSPI_waitWriteSRAMLevel(const CSL_ospi_flash_cfgRegs *pReg,
                                     uint32_t *sramLvl)
{
    uint32_t retry = CSL_OSPI_REG_RETRY;
    uint32_t sramLevel;
    int32_t  retVal = 0;

    /* Wait till SRAM Level write is completed */
    while(retry != 0U)
    {
        sramLevel = CSL_REG32_RD(&pReg->SRAM_FILL_REG) >> \
            CSL_OSPI_FLASH_CFG_SRAM_FILL_REG_SRAM_FILL_INDAC_WRITE_FLD_SHIFT;
        if (sramLevel <= CSL_OSPI_SRAM_WATERMARK_WR_LVL)
        {
            *sramLvl = sramLevel;
            break;
        }
        uint32_t delay = CSL_OSPI_POLL_IDLE_DELAY;
        while(delay--);
        retry--;
    }

    if (retry != 0U)
    {
        retVal = 0;
    }
    else
    {
        retVal = (int32_t)(-1);
    }
    return(retVal);
}

static int32_t OSPI_waitIndWriteComplete(const CSL_ospi_flash_cfgRegs *pReg)
{
    uint32_t retry = OSPI_READ_WRITE_TIMEOUT;
    int32_t  retVal = 0;
    uint32_t regVal = 0U;

    /* Check flash indirect write controller status */
    while (retry != 0U)
    {
        regVal = CSL_REG32_FEXT(&pReg->INDIRECT_WRITE_XFER_CTRL_REG,
                         OSPI_FLASH_CFG_INDIRECT_WRITE_XFER_CTRL_REG_IND_OPS_DONE_STATUS_FLD);
        if (regVal == 1)
        {
            break;
        }
        uint32_t delay = OSPI_CHECK_IDLE_DELAY;
        while(delay--);
        retry--;
    }

    if (retry != 0U)
    {
        /* Clear indirect completion status */
        CSL_REG32_FINS(&pReg->INDIRECT_WRITE_XFER_CTRL_REG,
                   OSPI_FLASH_CFG_INDIRECT_WRITE_XFER_CTRL_REG_IND_OPS_DONE_STATUS_FLD,
                   1);
        retVal = 0;
    }
    else
    {
        retVal = -1;
    }
    return(retVal);
}

static int32_t OSPI_flashExecCmd(OSPILLD_Handle hOspi, const CSL_ospi_flash_cfgRegs *pReg, uint32_t timeout)
{
    uint32_t retry = OSPI_READ_WRITE_TIMEOUT;
    int32_t  retVal = 0;
    uint32_t idleFlag = 0;
    uint32_t startTicks, elapsedTicks = 0;
    uint32_t itrTimeout = hOspi->Clock_usecToTicks(timeout);

    startTicks = hOspi->Clock_getTicks();
    while ((idleFlag == 0)  && (elapsedTicks < itrTimeout))
    {
        idleFlag = CSL_REG32_FEXT(&pReg->CONFIG_REG,
                                  OSPI_FLASH_CFG_CONFIG_REG_IDLE_FLD);
        elapsedTicks = hOspi->Clock_getTicks() - startTicks;
    }

    if(elapsedTicks >= itrTimeout)
    {
        retVal = OSPI_TIMEOUT;
    }
    else
    {
        /* Start to execute flash read/write command */
        CSL_REG32_FINS(&pReg->FLASH_CMD_CTRL_REG,
                    OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_CMD_EXEC_FLD,
                    1);


        while (retry != 0U)
        {
            /* Check the command execution status
            * If the execution is complete, this bit field will be zero
            */
            uint32_t execCompleteFlag = CSL_REG32_FEXT(&pReg->FLASH_CMD_CTRL_REG,
                        OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_CMD_EXEC_STATUS_FLD);

            if (execCompleteFlag == 0)
            {
                break;
            }
            uint32_t delay = OSPI_CHECK_IDLE_DELAY;
            while(delay--);
            retry--;
        }

        if (retry == 0U)
        {
            retVal = OSPI_SYSTEM_FAILURE;
        }

        idleFlag = 0;
        /* initialize elapsedTicks to 0 */
        startTicks = hOspi->Clock_getTicks();
        elapsedTicks = 0;
        while ((idleFlag == 0) && (elapsedTicks < itrTimeout))
        {
            idleFlag = CSL_REG32_FEXT(&pReg->CONFIG_REG,
                                    OSPI_FLASH_CFG_CONFIG_REG_IDLE_FLD);
            elapsedTicks = hOspi->Clock_getTicks() - startTicks;
        }

        if(elapsedTicks >= itrTimeout)
        {
            retVal = OSPI_TIMEOUT;
        }
    }

    return (retVal);
}

static void OSPI_writeFifoData(uintptr_t indAddr, const uint8_t *src, uint32_t wrLen)
{
    uint32_t temp = 0;
    uint32_t remaining = wrLen;
    uint32_t *srcPtr = (uint32_t *)src;

    while (remaining > 0)
    {
        if (remaining >= CSL_OSPI_FIFO_WIDTH)
        {
            CSL_REG32_WR(indAddr, *srcPtr);
            remaining -= CSL_OSPI_FIFO_WIDTH;
        }
        else
        {
            /* dangling bytes */
            memcpy(&temp, srcPtr, remaining);
            CSL_REG32_WR(indAddr, temp);
            break;
        }
        srcPtr++;
    }
}

static void OSPI_readFifoData(uintptr_t indAddr, uint8_t *dest, uint32_t rdLen)
{
    uint32_t temp;
    uint32_t remaining = rdLen;
    uint32_t *destPtr = (uint32_t *)dest;

    while(remaining > 0)
    {
        if (remaining >= CSL_OSPI_FIFO_WIDTH)
        {
            *destPtr = CSL_REG32_RD(indAddr);
            remaining -= CSL_OSPI_FIFO_WIDTH;
        }
        else
        {
            temp = CSL_REG32_RD(indAddr);
            memcpy(destPtr, &temp, remaining);
            break;
        }
        destPtr++;
    }
}

static int32_t OSPI_waitIdle(OSPILLD_Handle hOspi, uint32_t timeOut)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;
    const CSL_ospi_flash_cfgRegs *pReg;
    uint32_t baseAddr = hOspi->baseAddr;

    pReg = (const CSL_ospi_flash_cfgRegs *)baseAddr;

    uint32_t               retry = 0U;
    uint32_t               timeOutVal = timeOut;

    while (timeOutVal != 0U)
    {
        if (CSL_REG32_FEXT(&pReg->CONFIG_REG,OSPI_FLASH_CFG_CONFIG_REG_IDLE_FLD) != 0U)
        {
            retry++;
            if (retry == 3U)
            {
                status = OSPI_SYSTEM_SUCCESS;
                break;
            }
        }
        else
        {
            retry = 0U;
        }
        uint32_t delay = OSPI_CHECK_IDLE_DELAY;
        while(delay--);
        timeOutVal--;
    }

    return status;
}

static uint32_t OSPI_utilLog2(uint32_t num)
{
    /* LUT based bit scan method using deBruijn(2, 5) sequence to avoid the loop */
    uint32_t ret = 0U;
    uint32_t temp = num;

    if(num != 0U)
    {
        /* Assume num is not power of 2, fill 1's after the most significant 1 */
        temp |= (temp >> 1U);
        temp |= (temp >> 2U);
        temp |= (temp >> 4U);
        temp |= (temp >> 8U);
        temp |= (temp >> 16U);

        /* 0x07C4ACDD is a modified deBruijn sequence */
        ret = (uint32_t)gTable[(uint32_t)((temp * 0x07C4ACDD) >> 27U)];
    }

    return ret;
}


/** \brief Static function to check if the Frame Format is valid */
static inline int32_t OSPI_lld_isFrameFormatValid(uint32_t frmFmt)
{
    int32_t status = OSPI_SYSTEM_FAILURE;
    
    if((frmFmt == OSPI_FF_POL0_PHA0) || (frmFmt == OSPI_FF_POL0_PHA1) || \
       (frmFmt == OSPI_FF_POL1_PHA0) || (frmFmt == OSPI_FF_POL1_PHA1))
       {
            status = OSPI_SYSTEM_SUCCESS;
       }
    return status;
}

/** \brief Static function to check if the Chip Select is valid */
static inline int32_t OSPI_lld_isChipSelectValid(uint32_t chipSelect)
{
    int32_t status = OSPI_SYSTEM_FAILURE;


    if((chipSelect == OSPI_CS0) || (chipSelect == OSPI_CS1) || \
       (chipSelect == OSPI_CS2) || (chipSelect == OSPI_CS3))
       {
            status = OSPI_SYSTEM_SUCCESS;
       }
    return status;
}

/** \brief Static function to check if the Decoder Chip Select is valid */
static inline int32_t OSPI_lld_isDecoderChipSelectValid(uint32_t decChipSelect)
{
    int32_t status = OSPI_SYSTEM_FAILURE;

    if((decChipSelect == OSPI_DECODER_SELECT4) || \
       (decChipSelect == OSPI_DECODER_SELECT16))
      {
            status = OSPI_SYSTEM_SUCCESS;
      }
    return status;
}

/** \brief Static function to check the lld parameter */
static int32_t OSPI_lld_param_check(bool result)
{
    int32_t status = OSPI_SYSTEM_FAILURE;
    if(result)
    {
        status = OSPI_SYSTEM_SUCCESS;
    }
    return status;
}

void OSPI_lld_readCompleteCallback(void* args)
{
    OSPILLD_Handle hOspi = (OSPILLD_Handle) args;
    if (NULL == hOspi->readCompleteCallback)
    {
        /*  No Call Back Function Registered */
    }
    else
    {
        /* Read complete callback for DMA mode */
        hOspi->readCompleteCallback(hOspi);
    }
}

void OSPI_lld_isr(void* args)
{
    OSPILLD_Handle hOspi = (OSPILLD_Handle) args;
    const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);
    uint32_t regVal;
    uint8_t *pDst;
    uint32_t sramLevel = 0U, readBytes = 0U, writeBytes = 0U;
    
    switch (hOspi->currTrans->state)
    {
    case OSPI_TRANS_IDLE:                   
        break;
    case OSPI_TRANS_WRITE:

        regVal = CSL_REG32_RD(&pReg->IRQ_STATUS_REG);

        if (hOspi->currTrans->count != 0U && hOspi->currTrans->buf != NULL)
        {
            pDst = (uint8_t *) (hOspi->currTrans->buf);
            pDst += hOspi->currTrans->transferOffset;
            if (0U != (regVal & CSL_OSPI_INTR_MASK_IND_XFER))
            {
                sramLevel = OSPI_getWriteSramLevel(hOspi);

                writeBytes = (CSL_OSPI_SRAM_PARTITION_WR - sramLevel) * CSL_OSPI_FIFO_WIDTH;
                writeBytes = (writeBytes > hOspi->currTrans->count) ? hOspi->currTrans->count : writeBytes;

                OSPI_writeFifoData(hOspi->hOspiInit->dataBaseAddr, pDst, writeBytes);

                hOspi->currTrans->transferOffset += writeBytes;
                hOspi->currTrans->count -= writeBytes;
            }
            if((0U == hOspi->currTrans->count) || (0U != (regVal & CSL_OSPI_FLASH_CFG_IRQ_STATUS_REG_INDIRECT_OP_DONE_FLD_MASK)))
            {
                /* Clear indirect read operation complete status */
                CSL_REG32_FINS(&pReg->INDIRECT_READ_XFER_CTRL_REG,
                   OSPI_FLASH_CFG_INDIRECT_READ_XFER_CTRL_REG_IND_OPS_DONE_STATUS_FLD,
                   1);
                OSPI_disableInterrupt(hOspi);
                OSPI_clearInterrupt(hOspi);
                hOspi->currTrans->status = OSPI_TRANSFER_COMPLETED;
                hOspi->currTrans->state = OSPI_TRANS_IDLE;
                hOspi->interruptCallback(args);
            }
        }
        break;

    case OSPI_TRANS_READ:

        regVal = CSL_REG32_RD(&pReg->IRQ_STATUS_REG);

        if(hOspi->currTrans->count != 0U &&  hOspi->currTrans->buf != NULL)
        {            
            pDst = (uint8_t *) (hOspi->currTrans->buf);
            pDst += hOspi->currTrans->transferOffset;
            if (0U != (regVal & CSL_OSPI_INTR_MASK_IND_XFER))
            {
                sramLevel = OSPI_getSramLevel(hOspi);
                if (0U == sramLevel)
                {
                    break;
                }

                readBytes = sramLevel * CSL_OSPI_FIFO_WIDTH;
                readBytes = (readBytes > hOspi->currTrans->count) ? hOspi->currTrans->count : readBytes;

                /* Read data from FIFO */
                OSPI_readFifoData(hOspi->hOspiInit->dataBaseAddr, pDst, readBytes);

                hOspi->currTrans->transferOffset += readBytes;
                hOspi->currTrans->count -= readBytes;
            
            }
            if((0U == hOspi->currTrans->count) || (0U != (regVal & CSL_OSPI_FLASH_CFG_IRQ_STATUS_REG_INDIRECT_OP_DONE_FLD_MASK)))
            {
                /* Clear indirect read operation complete status */
                CSL_REG32_FINS(&pReg->INDIRECT_READ_XFER_CTRL_REG,
                   OSPI_FLASH_CFG_INDIRECT_READ_XFER_CTRL_REG_IND_OPS_DONE_STATUS_FLD,
                   1);
                OSPI_disableInterrupt(hOspi);
                OSPI_clearInterrupt(hOspi);
                hOspi->currTrans->state = OSPI_TRANS_IDLE;
                hOspi->currTrans->status = OSPI_TRANSFER_COMPLETED;
                hOspi->interruptCallback(args);
            }            
        }        
        break;

    default:
        break;
    }    
}       

static void OSPI_disableInterrupt(OSPILLD_Handle hOspi)
{
    const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);
    uint32_t regVal = CSL_REG32_RD(&pReg->IRQ_MASK_REG);
    regVal &= ~(CSL_OSPI_INTR_MASK_ALL);
    CSL_REG32_WR(&pReg->IRQ_MASK_REG, regVal);
}

static void OSPI_enableInterrupt(OSPILLD_Handle hOspi)
{
    const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);
    uint32_t regVal = CSL_REG32_RD(&pReg->IRQ_MASK_REG);
    regVal |= (CSL_OSPI_INTR_MASK_ALL);
    CSL_REG32_WR(&pReg->IRQ_MASK_REG, regVal);

}

static void OSPI_clearInterrupt(OSPILLD_Handle hOspi)
{
    const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);
    CSL_REG32_WR(&pReg->IRQ_STATUS_REG, CSL_OSPI_INTR_MASK_ALL);
}

static uint32_t OSPI_getSramLevel(OSPILLD_Handle hOspi)
{
    uint32_t sramLevel;
    const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

    sramLevel = CSL_REG32_RD(&pReg->SRAM_FILL_REG) &
        CSL_OSPI_FLASH_CFG_SRAM_FILL_REG_SRAM_FILL_INDAC_READ_FLD_MASK;

    return sramLevel;
}

static uint32_t OSPI_getWriteSramLevel(OSPILLD_Handle hOspi)
{
    uint32_t sramLevel;
    const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

    sramLevel = (CSL_REG32_RD(&pReg->SRAM_FILL_REG) &
        CSL_OSPI_FLASH_CFG_SRAM_FILL_REG_SRAM_FILL_INDAC_WRITE_FLD_MASK) >> CSL_OSPI_FLASH_CFG_SRAM_FILL_REG_SRAM_FILL_INDAC_WRITE_FLD_SHIFT;

    return sramLevel;
}

uint32_t OSPI_lld_isValidateOtpEnable(OSPILLD_Handle hOspi)
{
    uint32_t retVal = 0U;
    /* Check if DAC is enabled or not */
    retVal = hOspi->hOspiInit->validateOtp;
    return retVal;
}

static uint32_t OSPI_lld_calculateTicksForns(const uint32_t refClkhz, const uint32_t nsVal)
{
	uint32_t ticks;

	ticks = refClkhz / 1000;	/* kHz */
	ticks = OSPI_DIV_ROUND_UP(ticks * nsVal, 1000000);

	return ticks;
}

int32_t OSPI_lld_setFrequency(OSPILLD_Handle hOspi, uint64_t inputClkFreq)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;
    OSPILLD_InitHandle hOspiInit;

    if(hOspi != NULL)
    {
        hOspiInit = hOspi->hOspiInit;
        status = SOC_moduleSetClockFrequency(hOspiInit->moduleId, hOspiInit->clkId, inputClkFreq);
    }
    else
    {
        status = OSPI_SYSTEM_FAILURE;
    }

    return status;
}

int32_t OSPI_lld_setDelays(OSPILLD_Handle hOspi, uint32_t inputClkFreq)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;
    const CSL_ospi_flash_cfgRegs *pReg;
    uint32_t tsclk, cssot, csset, csdads, csda;

    if(hOspi != NULL)
    {
        pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        /* Delay Setup */
        tsclk = OSPI_DIV_ROUND_UP(inputClkFreq, OSPI_MAX_OPERATING_FREQUENCY);
        cssot = OSPI_lld_calculateTicksForns(inputClkFreq, CSL_OSPI_DEV_DELAY_CSSOT);

        if(cssot < tsclk)
        {
            cssot = tsclk;
        }

        csset = OSPI_lld_calculateTicksForns(inputClkFreq, CSL_OSPI_DEV_DELAY_CSEOT);
        csdads = OSPI_lld_calculateTicksForns(inputClkFreq, CSL_OSPI_DEV_DELAY_CSDADS);
        csda = OSPI_lld_calculateTicksForns(inputClkFreq, CSL_OSPI_DEV_DELAY_CSDA);

        uint32_t devDelay = ((cssot << CSL_OSPI_FLASH_CFG_DEV_DELAY_REG_D_INIT_FLD_SHIFT)  | \
                      (csset << CSL_OSPI_FLASH_CFG_DEV_DELAY_REG_D_AFTER_FLD_SHIFT) | \
                      (csdads << CSL_OSPI_FLASH_CFG_DEV_DELAY_REG_D_BTWN_FLD_SHIFT)  | \
                      (csda << CSL_OSPI_FLASH_CFG_DEV_DELAY_REG_D_NSS_FLD_SHIFT));
        CSL_REG32_WR(&pReg->DEV_DELAY_REG, devDelay);
    }
    else
    {
        status = OSPI_SYSTEM_FAILURE;
    }

    return status;
}

int32_t OSPI_lld_setBaudRateDiv(OSPILLD_Handle hOspi, uint32_t baudRateDiv)
{
    int32_t status = OSPI_SYSTEM_SUCCESS;
    const CSL_ospi_flash_cfgRegs *pReg;

    if(hOspi != NULL)
    {
        pReg = (const CSL_ospi_flash_cfgRegs *)(hOspi->baseAddr);

        if(baudRateDiv != 0U)
        {
            CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_MSTR_BAUD_DIV_FLD,
                   CSL_OSPI_BAUD_RATE_DIVISOR(baudRateDiv));
        }
        else
        {
            CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_MSTR_BAUD_DIV_FLD,
                   CSL_OSPI_BAUD_RATE_DIVISOR_DEFAULT);
        }
    }
    else
    {
        status = OSPI_SYSTEM_FAILURE;
    }

    return status;
}
