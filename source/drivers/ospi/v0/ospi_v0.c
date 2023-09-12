/*
 *  Copyright (C) 2021-23 Texas Instruments Incorporated
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
 *  \file ospi_v0.c
 *
 *  \brief File containing OSPI Driver APIs implementation for version V0.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* This is needed for memset/memcpy */
#include <string.h>
#include <drivers/ospi.h>
#include <drivers/ospi/v0/cslr_ospi.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/CacheP.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/ospi/v0/dma/ospi_dma.h>

/* TODO:HS hack, remove it when DMA bug is fixed */
#include <drivers/bootloader/soc/bootloader_soc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

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

/** \brief OSPI Chip select macro */
#define CSL_OSPI_CHIP_SELECT(x)   ((~((1U) << (x))) & 0xFU)

/**
 *  \brief   OSPI device delay parameter array size.
 */
#define CSL_OSPI_DEV_DELAY_ARRAY_SIZE  (4U)

/** \brief   OSPI device delays in cycles of OSPI controller ref clock */
#define CSL_OSPI_DEV_DELAY_CSSOT     (46U)  /* Chip Select Start of Transfer Delay */
#define CSL_OSPI_DEV_DELAY_CSEOT     (46U)  /* Chip Select End of Transfer Delay */
#define CSL_OSPI_DEV_DELAY_CSDADS    (192U) /* Chip Select De-Assert Different Peripheral Delay */
#define CSL_OSPI_DEV_DELAY_CSDA      (192U) /* Chip Select De-Assert Delay */

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

typedef struct
{
    void *openLock;
    /**<  Lock to protect OSPI open*/
    SemaphoreP_Object lockObj;
    /**< Lock object */
} OSPI_DrvObj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Internal functions */
static void OSPI_isr(void *args);

static int32_t OSPI_waitReadSRAMLevel(const CSL_ospi_flash_cfgRegs *pReg, uint32_t *rdLevel);
static int32_t OSPI_waitIndReadComplete(const CSL_ospi_flash_cfgRegs *pReg);
static int32_t OSPI_waitWriteSRAMLevel(const CSL_ospi_flash_cfgRegs *pReg,
                                     uint32_t *sramLvl);
static int32_t OSPI_waitIndWriteComplete(const CSL_ospi_flash_cfgRegs *pReg);
static int32_t OSPI_waitIdle(OSPI_Handle handle, uint32_t timeOut);

static int32_t OSPI_flashExecCmd(const CSL_ospi_flash_cfgRegs *pReg);
static void OSPI_readFifoData(uintptr_t indAddr, uint8_t *dest, uint32_t rdLen);
static void OSPI_writeFifoData(uintptr_t indAddr, const uint8_t *src, uint32_t wrLen);

static int32_t OSPI_programInstance(OSPI_Config *config);
static int32_t OSPI_isDmaRestrictedRegion(OSPI_Handle handle, uint32_t addr);
static uint32_t OSPI_utilLog2(uint32_t num);
static uint8_t OSPI_getCmdExt(OSPI_Handle handle, uint8_t cmd);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** \brief Driver object */
static OSPI_DrvObj gOspiDrvObj =
{
    .openLock      = NULL,
};

/** \brief LUT table for log2 calculation using DeBruijn sequence */
static const uint8_t gTable[32] =
{
    0, 9, 1, 10, 13, 21, 2, 29, 11, 14, 16, 18, 22, 25, 3, 30,
    8, 12, 20, 28, 15, 17, 24, 7, 19, 27, 23, 6, 26, 5, 4, 31
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void OSPI_init(void)
{
    int32_t status;
    uint32_t count;
    OSPI_Object *obj;

    /* Init each driver instance object */
    for(count = 0U; count < gOspiConfigNum; count++)
    {
        /* Init object variables */
        obj = gOspiConfig[count].object;
        DebugP_assert(NULL != obj);
        memset(obj, 0, sizeof(OSPI_Object));
    }

    /* Create the driver lock */
    status = SemaphoreP_constructMutex(&gOspiDrvObj.lockObj);
    if(SystemP_SUCCESS == status)
    {
        gOspiDrvObj.openLock = &gOspiDrvObj.lockObj;
    }

    return;
}

void OSPI_deinit(void)
{
    /* Delete driver lock */
    if(NULL != gOspiDrvObj.openLock)
    {
        SemaphoreP_destruct(&gOspiDrvObj.lockObj);
        gOspiDrvObj.openLock = NULL;
    }

    return;
}

OSPI_Handle OSPI_open(uint32_t index, const OSPI_Params *openParams)
{
    int32_t status = SystemP_SUCCESS;
    OSPI_Handle handle = NULL;
    OSPI_Config *config = NULL;
    OSPI_Object *obj = NULL;
    HwiP_Params hwiPrms;
    const OSPI_Attrs *attrs;

    /* Check for valid index */
    if(index >= gOspiConfigNum)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        config = &gOspiConfig[index];
    }

    /* Protect this region from a concurrent OSPI_Open */
    DebugP_assert(NULL != gOspiDrvObj.openLock);
    SemaphoreP_pend(&gOspiDrvObj.lockObj, SystemP_WAIT_FOREVER);

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
        obj->handle = (OSPI_Handle)config;

        /* If DMA is enabled, program UDMA block copy channel */
        if(TRUE == attrs->dmaEnable)
        {
            obj->ospiDmaHandle = OSPI_dmaOpen(openParams->ospiDmaChIndex);
        }
        else
        {
            obj->ospiDmaHandle = NULL;
        }

        /* Program OSPI instance according the user config */
        status = OSPI_programInstance(config);

        /* Create instance lock */
        status += SemaphoreP_constructMutex(&obj->lockObj);

        /* Create transfer sync semaphore */
        status += SemaphoreP_constructBinary(&obj->transferSemObj, 0U);

        /* Register interrupt */
        if(TRUE == attrs->intrEnable)
        {
            HwiP_Params_init(&hwiPrms);
            hwiPrms.intNum      = attrs->intrNum;
            hwiPrms.callback    = &OSPI_isr;
            hwiPrms.priority    = attrs->intrPriority;
            hwiPrms.args        = (void *) config;
            status += HwiP_construct(&obj->hwiObj, &hwiPrms);
        }
    }

    if(SystemP_SUCCESS == status)
    {
        obj->isOpen = 1;
        handle = (OSPI_Handle) config;
    }

    SemaphoreP_post(&gOspiDrvObj.lockObj);

    /* Free up resources in case of error */
    if(SystemP_SUCCESS != status)
    {
        if(NULL != config)
        {
            OSPI_close((OSPI_Handle) config);
        }
    }
    return handle;
}

void OSPI_close(OSPI_Handle handle)
{
    if(handle != NULL)
    {
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);

        /* Disable interrupts by read-modify-write to IRQ mask register */
        uint32_t regVal = CSL_REG32_RD(&pReg->IRQ_MASK_REG);
        regVal &= ~(CSL_OSPI_INTR_MASK_ALL);
        CSL_REG32_WR(&pReg->IRQ_MASK_REG, regVal);

        /* Destruct all locks and Hwi objects */
        SemaphoreP_destruct(&obj->lockObj);
        SemaphoreP_destruct(&obj->transferSemObj);
        HwiP_destruct(&obj->hwiObj);
        obj->isOpen = 0;
        SemaphoreP_post(&gOspiDrvObj.lockObj);

        /* If DMA block copy channel was opened, close it */
        if(attrs->dmaEnable == TRUE)
        {
            OSPI_dmaClose(obj->ospiDmaHandle);
        }

        if(attrs->phyEnable == TRUE)
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

    return;
}

OSPI_Handle OSPI_getHandle(uint32_t driverInstanceIndex)
{
    OSPI_Handle         handle = NULL;
    /* Check index */
    if(driverInstanceIndex < gOspiConfigNum)
    {
        OSPI_Object *obj;
        obj = gOspiConfig[driverInstanceIndex].object;

        if(obj && (TRUE == obj->isOpen))
        {
            /* valid handle */
            handle = obj->handle;
        }
    }
    return handle;
}

void OSPI_Transaction_init(OSPI_Transaction *trans)
{
    trans->count = 0U;
    trans->buf = NULL;
    trans->addrOffset = 0U;
    trans->transferTimeout = SystemP_WAIT_FOREVER;
    trans->status = OSPI_TRANSFER_STARTED;
}

void OSPI_ReadCmdParams_init(OSPI_ReadCmdParams *rdParams)
{
    rdParams->cmd = OSPI_CMD_INVALID_OPCODE;
    rdParams->cmdAddr = OSPI_CMD_INVALID_ADDR;
    rdParams->numAddrBytes = 3;
    rdParams->rxDataBuf = NULL;
    rdParams->rxDataLen = 0;
    rdParams->dummyBits = 0;
}

void OSPI_WriteCmdParams_init(OSPI_WriteCmdParams *wrParams)
{
    wrParams->cmd = OSPI_CMD_INVALID_OPCODE;
    wrParams->cmdAddr = OSPI_CMD_INVALID_ADDR;
    wrParams->numAddrBytes = 3;
    wrParams->txDataBuf = NULL;
    wrParams->txDataLen = 0;
}

uint32_t OSPI_getInputClk(OSPI_Handle handle)
{
    uint32_t retVal = 0U;
    if(handle != NULL)
    {
        const OSPI_Attrs* attrs = ((OSPI_Config *)handle)->attrs;
        retVal = attrs->inputClkFreq;
    }
    return retVal;
}

uint32_t OSPI_isDacEnable(OSPI_Handle handle)
{
    uint32_t retVal = 0U;
    const OSPI_Attrs* attrs = ((OSPI_Config *)handle)->attrs;
    retVal = attrs->dacEnable;
    return retVal;
}

uint32_t OSPI_isDmaEnable(OSPI_Handle handle)
{
    uint32_t retVal = 0U;
    const OSPI_Attrs* attrs = ((OSPI_Config *)handle)->attrs;
    retVal = attrs->dmaEnable;
    return retVal;
}

uint32_t OSPI_isIntrEnable(OSPI_Handle handle)
{
    uint32_t retVal = 0U;
    const OSPI_Attrs* attrs = ((OSPI_Config *)handle)->attrs;
    retVal = attrs->intrEnable;
    return retVal;
}

uint32_t OSPI_isPhyEnable(OSPI_Handle handle)
{
    uint32_t retVal = 0U;
    const OSPI_Attrs* attrs = ((OSPI_Config *)handle)->attrs;
    retVal = attrs->phyEnable;
    return retVal;
}

void OSPI_setPhyEnableSuccess(OSPI_Handle handle, uint32_t success)
{
    if(handle != NULL)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        obj->phyEnableSuccess = success;
    }
}

uint32_t OSPI_getPhyEnableSuccess(OSPI_Handle handle)
{
    uint32_t success = 0;

    if(handle != NULL)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        success = obj->phyEnableSuccess;
    }

    return success;
}

int32_t OSPI_enableDDR(OSPI_Handle handle)
{
    int32_t status = SystemP_SUCCESS;

    if(handle != NULL)
    {
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);

        /* Enable DTR protocol */
        CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_ENABLE_DTR_PROTOCOL_FLD,
                   TRUE);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t OSPI_enableSDR(OSPI_Handle handle)
{
    int32_t status = SystemP_SUCCESS;

    if(handle != NULL)
    {
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);

        /* Disable DTR protocol */
        CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_ENABLE_DTR_PROTOCOL_FLD,
                   FALSE);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t OSPI_enableDdrRdCmds(OSPI_Handle handle)
{
    int32_t status = SystemP_SUCCESS;

    if(handle != NULL)
    {
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);

        /* Enable DDR EN commands */
        CSL_REG32_FINS(&pReg->DEV_INSTR_RD_CONFIG_REG,
                   OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_DDR_EN_FLD,
                   1);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t OSPI_setRdDataCaptureDelay(OSPI_Handle handle, uint32_t rdDataCapDelay)
{
    int32_t status = SystemP_SUCCESS;

    if(handle != NULL)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);

        /* Set read capture delay */
        CSL_REG32_FINS(&pReg->RD_DATA_CAPTURE_REG,
                   OSPI_FLASH_CFG_RD_DATA_CAPTURE_REG_DELAY_FLD,
                   rdDataCapDelay);

        /* Update book keeping */
        obj->rdDataCapDelay = rdDataCapDelay;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

void OSPI_setNumAddrBytes(OSPI_Handle handle, uint32_t numAddrBytes)
{
    if(handle != NULL)
    {
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;

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
        obj->numAddrBytes = numAddrBytes;
    }
    else
    {
        /* do nothing */
    }
}

void OSPI_setDeviceSize(OSPI_Handle handle, uint32_t pageSize, uint32_t blkSize)
{
    if(handle != NULL)
    {
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);

        CSL_REG32_FINS(&pReg->DEV_SIZE_CONFIG_REG, OSPI_FLASH_CFG_DEV_SIZE_CONFIG_REG_BYTES_PER_DEVICE_PAGE_FLD, pageSize);
        CSL_REG32_FINS(&pReg->DEV_SIZE_CONFIG_REG, OSPI_FLASH_CFG_DEV_SIZE_CONFIG_REG_BYTES_PER_SUBSECTOR_FLD, OSPI_utilLog2(blkSize));
    }
    else
    {
        /* do nothing */
    }
}

void OSPI_setModeBits(OSPI_Handle handle, uint32_t modeBits)
{
    if(NULL != handle)
    {
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        const CSL_ospi_flash_cfgRegs *pReg;

        pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);

        CSL_REG32_FINS(&pReg->MODE_BIT_CONFIG_REG,
                    OSPI_FLASH_CFG_MODE_BIT_CONFIG_REG_MODE_FLD,
                    (uint8_t)modeBits);
    }
}

void OSPI_enableModeBitsCmd(OSPI_Handle handle)
{
    if(NULL != handle)
    {
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        const CSL_ospi_flash_cfgRegs *pReg;

        pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);

        CSL_REG32_FINS(&pReg->FLASH_CMD_CTRL_REG,
                    OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_ENB_MODE_BIT_FLD,
                    1);
    }
}

void OSPI_enableModeBitsRead(OSPI_Handle handle)
{
    if(NULL != handle)
    {
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        const CSL_ospi_flash_cfgRegs *pReg;

        pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);

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

uint32_t OSPI_getProtocol(OSPI_Handle handle)
{
    uint32_t retVal = OSPI_NOR_PROTOCOL_INVALID;

    if(handle != NULL)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        retVal = obj->protocol;
    }
    return retVal;
}

void OSPI_setProtocol(OSPI_Handle handle, uint32_t protocol)
{
    if(handle != NULL)
    {
        /* First set the transfer lines */
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;

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
                    OSPI_enableDDR(handle);
                    OSPI_setDualOpCodeMode(handle);
                }
                if(protocol == OSPI_NOR_PROTOCOL(4,4,4,1))
                {
                    OSPI_enableDdrRdCmds(handle);
                }
            }

            /* Update book-keeping variable in OSPI object */
            obj->protocol = protocol;
        }
    }
    else
    {
        /* do nothing */
    }
}

void OSPI_setReadDummyCycles(OSPI_Handle handle, uint32_t dummyCycles)
{
    if(handle != NULL)
    {
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;

        /* Set read dummy cycles */
        CSL_REG32_FINS(&pReg->DEV_INSTR_RD_CONFIG_REG, OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_DUMMY_RD_CLK_CYCLES_FLD, dummyCycles);

        /* Update book-keeping variable in OSPI object */
        obj->rdDummyCycles = dummyCycles;
    }
    else
    {
        /* do nothing */
    }
}

void OSPI_setCmdDummyCycles(OSPI_Handle handle, uint32_t dummyCycles)
{
    if(handle != NULL)
    {
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;

        /* Set command dummy cycles */
        CSL_REG32_FINS(&pReg->FLASH_CMD_CTRL_REG, OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_NUM_DUMMY_CYCLES_FLD, dummyCycles);

        /* Update book-keeping variable in OSPI object */
        obj->cmdDummyCycles = dummyCycles;
    }
    else
    {
        /* do nothing */
    }
}

void OSPI_setDualOpCodeMode(OSPI_Handle handle)
{
    if(handle != NULL)
    {
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);

        /* Enable dual byte opcode */
        CSL_REG32_FINS(&pReg->CONFIG_REG, OSPI_FLASH_CFG_CONFIG_REG_DUAL_BYTE_OPCODE_EN_FLD, TRUE);
    }
    else
    {
        /* do nothing */
    }
}

void OSPI_clearDualOpCodeMode(OSPI_Handle handle)
{
    if(handle != NULL)
    {
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);

        /* Enable dual byte opcode */
        CSL_REG32_FINS(&pReg->CONFIG_REG, OSPI_FLASH_CFG_CONFIG_REG_DUAL_BYTE_OPCODE_EN_FLD, FALSE);
    }
    else
    {
        /* do nothing */
    }
}

void OSPI_setXferOpCodes(OSPI_Handle handle, uint8_t readCmd, uint8_t pageProgCmd)
{
    if(handle != NULL)
    {
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);
        uint8_t cmdExt = OSPI_CMD_INVALID_OPCODE;

        /* Set opcode for read */
        CSL_REG32_FINS(&pReg->DEV_INSTR_RD_CONFIG_REG, OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_RD_OPCODE_NON_XIP_FLD, readCmd);

        /* Set the extended opcode for read */
        cmdExt = OSPI_getCmdExt(handle, readCmd);

        if(cmdExt != OSPI_CMD_INVALID_OPCODE)
        {
            CSL_REG32_FINS(&pReg->OPCODE_EXT_LOWER_REG, OSPI_FLASH_CFG_OPCODE_EXT_LOWER_REG_EXT_READ_OPCODE_FLD, cmdExt);
        }

        /* Set opcode for write */
        CSL_REG32_FINS(&pReg->DEV_INSTR_WR_CONFIG_REG, OSPI_FLASH_CFG_DEV_INSTR_WR_CONFIG_REG_WR_OPCODE_FLD, pageProgCmd);

        /* Set the extended opcode for read */
        cmdExt = OSPI_getCmdExt(handle, pageProgCmd);

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

void OSPI_setCmdExtType(OSPI_Handle handle, uint32_t cmdExtType)
{
    if(NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        obj->cmdExtType = cmdExtType;
    }
    else
    {
        /* do nothing */
    }
}

int32_t OSPI_enableDacMode(OSPI_Handle handle)
{
    int32_t status = SystemP_SUCCESS;

    if(NULL != handle)
    {
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);

        CSL_REG32_FINS(&pReg->CONFIG_REG,
                       OSPI_FLASH_CFG_CONFIG_REG_ENB_DIR_ACC_CTLR_FLD,
                       1);
        CSL_REG32_WR(&pReg->IND_AHB_ADDR_TRIGGER_REG, 0x04000000);

        obj->isDacEnable = TRUE;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t OSPI_disableDacMode(OSPI_Handle handle)
{
    int32_t status = SystemP_SUCCESS;

    if(NULL != handle)
    {
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);

        CSL_REG32_FINS(&pReg->CONFIG_REG,
                       OSPI_FLASH_CFG_CONFIG_REG_ENB_DIR_ACC_CTLR_FLD,
                       0U);
        CSL_REG32_WR(&pReg->IND_AHB_ADDR_TRIGGER_REG, 0);

        obj->isDacEnable = FALSE;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t OSPI_enablePhyPipeline(OSPI_Handle handle)
{
    int32_t status = SystemP_SUCCESS;

    if(NULL != handle)
    {
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);

        /* Enable PHY pipeline */
            CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_PIPELINE_PHY_FLD,
                   TRUE);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t OSPI_disablePhyPipeline(OSPI_Handle handle)
{
    int32_t status = SystemP_SUCCESS;

    if(NULL != handle)
    {
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);

        /* Disable PHY pipeline */
            CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_PIPELINE_PHY_FLD,
                   FALSE);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t OSPI_enablePhy(OSPI_Handle handle)
{
    int32_t status = SystemP_SUCCESS;

    if(NULL != handle)
    {
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);

        uint32_t phyEnable = CSL_REG32_FEXT(&pReg->CONFIG_REG,
                                            OSPI_FLASH_CFG_CONFIG_REG_PHY_MODE_ENABLE_FLD);
        if(phyEnable == FALSE)
        {
            /* Set dummyClks 1 less */
            uint32_t dummyClks = obj->rdDummyCycles - 1;

            /* Set new dummyClk */
            CSL_REG32_FINS(&pReg->DEV_INSTR_RD_CONFIG_REG,
                            OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_DUMMY_RD_CLK_CYCLES_FLD,
                            dummyClks);

            if(obj->phyRdDataCapDelay != 0xFF)
            {
                /* Tuning is already done, set read capture delay */
                CSL_REG32_FINS(&pReg->RD_DATA_CAPTURE_REG,
                           OSPI_FLASH_CFG_RD_DATA_CAPTURE_REG_DELAY_FLD,
                           obj->phyRdDataCapDelay);

            }

            /* Enable PHY mode */
            CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_PHY_MODE_ENABLE_FLD,
                   TRUE);
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t OSPI_disablePhy(OSPI_Handle handle)
{
    int32_t status = SystemP_SUCCESS;

    if(NULL != handle)
    {
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);

        uint32_t phyEnable = CSL_REG32_FEXT(&pReg->CONFIG_REG,
                                            OSPI_FLASH_CFG_CONFIG_REG_PHY_MODE_ENABLE_FLD);

        if(phyEnable == TRUE)
        {
            uint32_t dummyClks = obj->rdDummyCycles;

            /* Set new dummyClk */
            CSL_REG32_FINS(&pReg->DEV_INSTR_RD_CONFIG_REG,
                            OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_DUMMY_RD_CLK_CYCLES_FLD,
                            dummyClks);

            /* Set the non-PHY read delay */
            CSL_REG32_FINS(&pReg->RD_DATA_CAPTURE_REG,
                       OSPI_FLASH_CFG_RD_DATA_CAPTURE_REG_DELAY_FLD,
                       obj->rdDataCapDelay);

            /* Disable PHY mode */
            CSL_REG32_FINS(&pReg->CONFIG_REG,
               OSPI_FLASH_CFG_CONFIG_REG_PHY_MODE_ENABLE_FLD,
               FALSE);

        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

uint32_t OSPI_getFlashDataBaseAddr(OSPI_Handle handle)
{
    uint32_t dataBaseAddr = 0U;

    if(NULL != handle)
    {
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        dataBaseAddr = attrs->dataBaseAddr;
    }

    return dataBaseAddr;
}

/* Different OSPI Read functions */
int32_t OSPI_readCmd(OSPI_Handle handle, OSPI_ReadCmdParams *rdParams)
{
    int32_t status = SystemP_SUCCESS;
    const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
    const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);
    OSPI_Object *obj = ((OSPI_Config *)handle)->object;
    uint8_t *pBuf = (uint8_t *) rdParams->rxDataBuf;
    uint32_t rxLen = rdParams->rxDataLen;

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
        CSL_REG32_FINS(&pReg->FLASH_CMD_CTRL_REG, OSPI_FLASH_CFG_FLASH_CMD_CTRL_REG_NUM_DUMMY_CYCLES_FLD, obj->cmdDummyCycles);
    }

    uint32_t dualOpCode = CSL_REG32_FEXT(&pReg->CONFIG_REG,
                                OSPI_FLASH_CFG_CONFIG_REG_DUAL_BYTE_OPCODE_EN_FLD);

    if(dualOpCode == 1)
    {
        uint8_t cmdExt = OSPI_getCmdExt(handle, rdParams->cmd);
        /* Set extended STIG opcode */
        CSL_REG32_FINS(&pReg->OPCODE_EXT_LOWER_REG, OSPI_FLASH_CFG_OPCODE_EXT_LOWER_REG_EXT_STIG_OPCODE_FLD, cmdExt);
    }
    else
    {
        /* do nothing */
    }

    if((rdParams->cmdAddr != OSPI_CMD_INVALID_ADDR) && (rdParams->numAddrBytes > 0))
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

    status = OSPI_flashExecCmd(pReg);

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

    return status;
}

int32_t OSPI_readDirect(OSPI_Handle handle, OSPI_Transaction *trans)
{
    int32_t status = SystemP_SUCCESS;
    const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
    OSPI_Object *obj = ((OSPI_Config *)handle)->object;
    const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);

    uint8_t *pSrc;
    uint8_t *pDst;
    uint32_t addrOffset;
    uint32_t dacState;

    addrOffset = trans->addrOffset;
    pDst = (uint8_t *) trans->buf;

    /* Enable Direct Access Mode */
    dacState = obj->isDacEnable;
    if(dacState == FALSE)
    {
        OSPI_enableDacMode(handle);
    }

    pSrc = (uint8_t *)(attrs->dataBaseAddr + addrOffset);

    /* DMA Copy fails when copying to to certain memory regions. So in this case we switch to normal memcpy
       for copying even if dmaEnable is true. Also do DMA copy only if size > 1KB*/
    uint32_t isDmaCopy = (attrs->dmaEnable == TRUE) &&
                         (OSPI_isDmaRestrictedRegion(handle, (uint32_t)pDst) == FALSE) &&
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

        if(attrs->phyEnable == TRUE)
        {
            /* Enable PHY pipeline */
            CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_PIPELINE_PHY_FLD,
                   TRUE);
        }

        OSPI_dmaCopy(obj->ospiDmaHandle, tempDst, tempSrc, remainingBytes - unalignedBytes);

        if(attrs->phyEnable == TRUE)
        {
            /* Disable PHY pipeline */
            CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_PIPELINE_PHY_FLD,
                   FALSE);
        }

        /* Do a CPU copy of unaligned bytes if any */
        if(unalignedBytes > 0)
        {
            tempDst += (remainingBytes - unalignedBytes);
            tempSrc += (remainingBytes - unalignedBytes);
            memcpy(tempDst, tempSrc, unalignedBytes);
        }
    }
    else
    {
        memcpy(pDst, pSrc, trans->count);
    }

    /* Switch to INDAC mode if DAC was initially in disabled state */
    if(dacState == FALSE)
    {
        OSPI_disableDacMode(handle);
    }

    return status;
}

int32_t OSPI_readIndirect(OSPI_Handle handle, OSPI_Transaction *trans)
{
    int32_t status = SystemP_SUCCESS;
    const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
    OSPI_Object *obj = ((OSPI_Config *)handle)->object;
    const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);
    uint8_t *pDst;
    uint32_t addrOffset;
    uint32_t remainingSize;
    uint32_t readFlag = 0U;
    uint32_t sramLevel = 0, readBytes = 0;
    uint32_t dacState;

    addrOffset = trans->addrOffset;
    pDst = (uint8_t *) trans->buf;

    /* Disable DAC Mode */
    dacState = obj->isDacEnable;
    if(dacState == TRUE)
    {
        OSPI_disableDacMode(handle);
    }

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

    if(OSPI_TRANSFER_MODE_POLLING == obj->transferMode)
    {
        remainingSize = trans->count;

        while(remainingSize > 0U)
        {
            if(OSPI_waitReadSRAMLevel(pReg, &sramLevel) != 0)
            {
                /* SRAM FIFO has no data, failure */
                readFlag = 1U;
                status = SystemP_FAILURE;
                trans->status = OSPI_TRANSFER_FAILED;
                break;
            }

            readBytes = sramLevel * CSL_OSPI_FIFO_WIDTH;
            readBytes = (readBytes > remainingSize) ? remainingSize : readBytes;

            /* Read data from FIFO */
            OSPI_readFifoData(attrs->dataBaseAddr, pDst, readBytes);

            pDst += readBytes;
            remainingSize -= readBytes;
        }
        /* Wait for completion of INDAC Read */
        if(readFlag == 0U && OSPI_waitIndReadComplete(pReg) != 0)
        {
            readFlag = 1U;
            status = SystemP_FAILURE;
            trans->status = OSPI_TRANSFER_FAILED;
        }

    }

    /* Return to DAC mode if it was initially in enabled state */
    if(dacState == TRUE)
    {
        OSPI_enableDacMode(handle);
    }

    return status;
}

/* Different OSPI write functions */
int32_t OSPI_writeCmd(OSPI_Handle handle, OSPI_WriteCmdParams *wrParams)
{
    int32_t status = SystemP_SUCCESS;
    const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
    const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);

    uint8_t *txBuf = (uint8_t *) wrParams->txDataBuf;
    uint32_t txLen = wrParams->txDataLen;

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
        uint8_t cmdExt = OSPI_getCmdExt(handle, wrParams->cmd);
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

    status = OSPI_flashExecCmd(pReg);

    return status;
}

int32_t OSPI_writeDirect(OSPI_Handle handle, OSPI_Transaction *trans)
{
    int32_t status = SystemP_SUCCESS;
    const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
    // OSPI_Object *obj = ((OSPI_Config *)handle)->object;
    const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);

    uint8_t *pSrc;
    uint8_t *pDst;
    uint32_t addrOffset;

    addrOffset = trans->addrOffset;
    pSrc = (uint8_t *) trans->buf;

    /* Enable Direct Access Mode */
    CSL_REG32_FINS(&pReg->CONFIG_REG, OSPI_FLASH_CFG_CONFIG_REG_ENB_DIR_ACC_CTLR_FLD, 1);
    CSL_REG32_WR(&pReg->IND_AHB_ADDR_TRIGGER_REG, 0x04000000);

    pDst = (uint8_t *)(attrs->dataBaseAddr + addrOffset);


    memcpy(pDst, pSrc, trans->count);

    return status;
    return SystemP_SUCCESS;
}

int32_t OSPI_writeIndirect(OSPI_Handle handle, OSPI_Transaction *trans)
{
    int32_t status = SystemP_SUCCESS;

    const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
    OSPI_Object *obj = ((OSPI_Config *)handle)->object;
    const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)attrs->baseAddr;

    uint8_t *pSrc;
    uint32_t dacState, addrOffset, remainingSize, sramLevel, wrBytes, wrFlag = 0;

    addrOffset = trans->addrOffset;
    pSrc = (uint8_t *) trans->buf;

    /* Disable DAC Mode */
    dacState = obj->isDacEnable;
    if(dacState == TRUE)
    {
        OSPI_disableDacMode(handle);
    }

    /* Set write address in indirect mode */
    CSL_REG32_WR(&pReg->INDIRECT_WRITE_XFER_START_REG, addrOffset);

    /* Set the Indirect Write Transfer count */
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

    if(OSPI_TRANSFER_MODE_POLLING == obj->transferMode)
    {
        if(OSPI_waitWriteSRAMLevel(pReg, &sramLevel) != 0)
        {
            wrFlag = 1U;
            status = SystemP_FAILURE;
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
                    status = SystemP_FAILURE;
                    break;
                }

                wrBytes = (CSL_OSPI_SRAM_PARTITION_WR - sramLevel) * CSL_OSPI_FIFO_WIDTH;
                wrBytes = (wrBytes > remainingSize) ? remainingSize : wrBytes;

                OSPI_writeFifoData(attrs->dataBaseAddr, pSrc, wrBytes);

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

    /* Return to DAC mode if it was initially in enabled state */
    if(dacState == TRUE)
    {
        OSPI_enableDacMode(handle);
    }

    return status;
}

/* Internal function definitions */
static int32_t OSPI_programInstance(OSPI_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t regVal;
    OSPI_Handle handle = (OSPI_Handle)config;
    const OSPI_Attrs *attrs = config->attrs;
    OSPI_Object *obj = config->object;
    const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)attrs->baseAddr;

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
    if (OSPI_waitIdle(handle, OSPI_READ_WRITE_TIMEOUT) != (int32_t)0U)
    {
        OSPI_close(handle);
        status = SystemP_FAILURE;
    }

    if(SystemP_SUCCESS == status)
    {
        /* User config */
        if(TRUE == attrs->intrEnable)
        {
            obj->transferMode = OSPI_TRANSFER_MODE_BLOCKING;
            /* When callback is supported, add logic */
        }
        else
        {
            obj->transferMode = OSPI_TRANSFER_MODE_POLLING;
        }
        /* Chip Select */
        regVal = CSL_REG32_RD(&pReg->CONFIG_REG);
        uint32_t chipSelect = OSPI_CHIP_SELECT(attrs->chipSelect);
        uint32_t decSelect = attrs->decChipSelect;

        regVal &= ~(CSL_OSPI_FLASH_CFG_CONFIG_REG_PERIPH_SEL_DEC_FLD_MASK | \
                    CSL_OSPI_FLASH_CFG_CONFIG_REG_PERIPH_CS_LINES_FLD_MASK);
        regVal |= (decSelect << CSL_OSPI_FLASH_CFG_CONFIG_REG_PERIPH_SEL_DEC_FLD_SHIFT) | \
                  (chipSelect << CSL_OSPI_FLASH_CFG_CONFIG_REG_PERIPH_CS_LINES_FLD_SHIFT);
        CSL_REG32_WR(&pReg->CONFIG_REG, regVal);
        /* Frame format */
        regVal = CSL_REG32_RD(&pReg->CONFIG_REG);
        regVal &= ~(CSL_OSPI_FLASH_CFG_CONFIG_REG_SEL_CLK_PHASE_FLD_MASK | \
                    CSL_OSPI_FLASH_CFG_CONFIG_REG_SEL_CLK_POL_FLD_MASK);
        regVal |= (attrs->frmFmt);
        CSL_REG32_WR(&pReg->CONFIG_REG, regVal);
        /* Disable the adapted loop-back clock circuit */
        CSL_REG32_FINS(&pReg->RD_DATA_CAPTURE_REG,
                   OSPI_FLASH_CFG_RD_DATA_CAPTURE_REG_BYPASS_FLD,
                   1);
        /* Delay Setup */
        uint32_t delays[4] = { 10, 10, 10, 10 };
        uint32_t devDelay = ((delays[0] << CSL_OSPI_FLASH_CFG_DEV_DELAY_REG_D_INIT_FLD_SHIFT)  | \
                      (delays[1] << CSL_OSPI_FLASH_CFG_DEV_DELAY_REG_D_AFTER_FLD_SHIFT) | \
                      (delays[2] << CSL_OSPI_FLASH_CFG_DEV_DELAY_REG_D_BTWN_FLD_SHIFT)  | \
                      (delays[3] << CSL_OSPI_FLASH_CFG_DEV_DELAY_REG_D_NSS_FLD_SHIFT));
        CSL_REG32_WR(&pReg->DEV_DELAY_REG, devDelay);

        if(attrs->baudRateDiv)
        {
            CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_MSTR_BAUD_DIV_FLD,
                   CSL_OSPI_BAUD_RATE_DIVISOR(attrs->baudRateDiv));
        }
        else
        {

            CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_MSTR_BAUD_DIV_FLD,
                   CSL_OSPI_BAUD_RATE_DIVISOR_DEFAULT);
        }

        /* Disable PHY pipeline mode */
        CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_PIPELINE_PHY_FLD,
                   FALSE);

        /* Disable PHY mode by default. This will be later enabled from flash driver */
        CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_PHY_MODE_ENABLE_FLD,
                   FALSE);

        /* Set indirect trigger address register */
        if(attrs->dacEnable)
        {
            OSPI_enableDacMode(handle);
        }
        else
        {
            OSPI_disableDacMode(handle);
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

        /* Initialize read delay and related book-keeping variables */
        obj->phyRdDataCapDelay = 0xFF;
        OSPI_setRdDataCaptureDelay(config, 0);

        /* Initialise controller to 1s1s1s mode to override any ROM settings */

        /* Set initial protocol to be 1s1s1s */
        OSPI_setProtocol(config, OSPI_NOR_PROTOCOL(1,1,1,0));
        OSPI_setXferOpCodes(config, 0x03, 0x02);

        /* Set address bytes to 3 */
        OSPI_setNumAddrBytes(config, 3);

        /* Initialize phy enable status */
        obj->phyEnableSuccess = FALSE;

        /* Enable OSPI Controller */
        CSL_REG32_FINS(&pReg->CONFIG_REG,
                       OSPI_FLASH_CFG_CONFIG_REG_ENB_SPI_FLD,
                       1);
    }

    return status;
}

static uint8_t OSPI_getCmdExt(OSPI_Handle handle, uint8_t cmd)
{
    uint8_t cmdExt = OSPI_CMD_INVALID_OPCODE;

    if(NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;

        switch(obj->cmdExtType)
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
    }

    return cmdExt;
}

static int32_t OSPI_isDmaRestrictedRegion(OSPI_Handle handle, uint32_t addr)
{
    int32_t isRestricted = FALSE;
    const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;

    if(NULL != attrs->dmaRestrictedRegions)
    {
        const OSPI_AddrRegion *addrRegions = attrs->dmaRestrictedRegions;
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

static int32_t OSPI_flashExecCmd(const CSL_ospi_flash_cfgRegs *pReg)
{
    uint32_t retry = OSPI_READ_WRITE_TIMEOUT;
    int32_t  retVal = 0;
    uint32_t idleFlag = 0;

    while (idleFlag == 0)
    {
        idleFlag = CSL_REG32_FEXT(&pReg->CONFIG_REG,
                                  OSPI_FLASH_CFG_CONFIG_REG_IDLE_FLD);
    }

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
        retVal = -1;
    }

    idleFlag = 0;
    while (idleFlag == 0)
    {
        idleFlag = CSL_REG32_FEXT(&pReg->CONFIG_REG,
                                  OSPI_FLASH_CFG_CONFIG_REG_IDLE_FLD);
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

static int32_t OSPI_waitIdle(OSPI_Handle handle, uint32_t timeOut)
{
    int32_t status = SystemP_SUCCESS;

    const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
    const CSL_ospi_flash_cfgRegs *pReg;
    uint32_t baseAddr = attrs->baseAddr;

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
                status = SystemP_SUCCESS;
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

static void OSPI_isr(void *args)
{
    /* TODO */
    return ;
}

int32_t OSPI_configResetPin(OSPI_Handle handle, uint32_t config)
{
    int32_t status = SystemP_SUCCESS;

    if(NULL != handle)
    {
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);

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
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t OSPI_configBaudrate(OSPI_Handle handle, uint32_t baud)
{
    int32_t status = SystemP_SUCCESS;

    if(NULL != handle && (baud <= 32) && (baud >= 2) && (baud % 2) == 0)
    {
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);
        CSL_REG32_FINS(&pReg->CONFIG_REG, OSPI_FLASH_CFG_CONFIG_REG_MSTR_BAUD_DIV_FLD, CSL_OSPI_BAUD_RATE_DIVISOR(baud));
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t OSPI_readBaudRateDivFromReg(OSPI_Handle handle, uint32_t *baudDiv)
{
    int32_t status = SystemP_SUCCESS;
    if(NULL != baudDiv && NULL != handle)
    {
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);
        uint32_t progBaudDiv = CSL_REG32_FEXT(&pReg->CONFIG_REG, OSPI_FLASH_CFG_CONFIG_REG_MSTR_BAUD_DIV_FLD);
        // see CSL_OSPI_BAUD_RATE_DIVISOR
        progBaudDiv = (progBaudDiv << 1) + 2;
        *baudDiv = progBaudDiv;
    }
    return status;
}

int32_t OSPI_getBaudRateDivFromObj(OSPI_Handle handle, uint32_t *baudDiv)
{

    int32_t status = SystemP_SUCCESS;
    if(NULL != baudDiv && NULL != handle)
    {
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        *baudDiv = attrs->baudRateDiv;
    }
    return status;
}