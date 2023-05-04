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
 *  \file qspi.c
 *
 *  \brief File containing QSPI Driver APIs implementation.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* This is needed for memset/memcpy */
#include <string.h>
#include <drivers/qspi.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/CacheP.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/qspi/v0/edma/qspi_edma.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief    QSPI Command default Length in SPI words */
#define QSPI_CMD_LEN            (1U)
/** \brief    QSPI Address default Length in SPI words */
#define QSPI_ADDR_LEN            (1U)

/** \brief    QSPI Operation mode- Configuration or memory mapped mode */
#define QSPI_MEM_MAP_PORT_SEL_CFG_PORT          \
                                (CSL_QSPI_SPI_SWITCH_REG_MMPT_S_SEL_CFG_PORT)
#define QSPI_MEM_MAP_PORT_SEL_MEM_MAP_PORT      \
                                (CSL_QSPI_SPI_SWITCH_REG_MMPT_S_SEL_MM_PORT)


/** \brief    QSPI Read Type - Single, Dual or Quad */
#define QSPI_MEM_MAP_READ_TYPE_NORMAL               \
                                (CSL_QSPI_SPI_SETUP0_REG_READ_TYPE_NORMAL_READ)
#define QSPI_MEM_MAP_READ_TYPE_DUAL                 \
                                (CSL_QSPI_SPI_SETUP0_REG_READ_TYPE_DUAL_READ)
#define QSPI_MEM_MAP_READ_TYPE_NORMAL_READTYPE      \
                                (CSL_QSPI_SPI_SETUP0_REG_READ_TYPE_NORMAL_READ_TYPE)
#define QSPI_MEM_MAP_READ_TYPE_QUAD                 \
                                (CSL_QSPI_SPI_SETUP0_REG_READ_TYPE_QUAD_READ)

/** \brief    Number of Bits in a byte */
#define SIZE_OF_BYTE            (8U)

/** \brief    Number of Bytes in a register */
#define SIZE_OF_REG             (4U)

/** \brief    Used to fill data bytes into the dummy variable for data register.
 *            The data needs to be filled in from the left(MSB) position, so
 *            that it gets correctly written into flash and proper data can be
 *            read back even in memory mapped mode.
 */
#define QSPI_DATA_WRITE(dst, src, idx, dataSize, wordlen)       \
                                (dst[(wordlen - idx - 1)/SIZE_OF_REG] |= (((uint32_t)(*src) <<    \
                                (SIZE_OF_BYTE * (dataSize - (idx % SIZE_OF_REG) - 1)))))

/** \brief    Used to read bytes from the dummy variable for data register.
 *            The data needs to be read, starting from the left(MSB) position.
 */
#define QSPI_DATA_READ(dst, src, idx, dataSize, wordlen)         \
                                (*dst = (uint8_t)(((src[(wordlen - idx - 1)/SIZE_OF_REG]) >>       \
                                (SIZE_OF_BYTE * (dataSize - (idx % SIZE_OF_REG) - 1))) \
                                 & (0x000000FFU)))

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct
{
    void *openLock;
    /**<  Lock to protect QSPI open*/
    SemaphoreP_Object lockObj;
    /**< Lock object */
} QSPI_DrvObj;

typedef struct
{
    int32_t                count;
    /**< [IN] Number of frames for this transaction */
    void                   *buf;
    /**< [IN] void * to a buffer to receive/send data */
    uint32_t                wlen;
    /**< [IN] word length to be used for this transaction. */
    uint32_t                cmdRegVal;
    /**< [IN] cmd register value to be written. */
} QSPI_ConfigAccess;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Internal functions */
static void QSPI_isr(void *args);
static int32_t QSPI_waitIdle(QSPI_Handle handle);
static int32_t QSPI_spiMemMapRead(QSPI_Handle handle);
static int32_t QSPI_spiMemMapWrite(QSPI_Handle handle);
static int32_t QSPI_spiConfigRead(QSPI_Handle handle, QSPI_ConfigAccess *cfgAccess);
static int32_t QSPI_spiConfigWrite(QSPI_Handle handle, QSPI_ConfigAccess *cfgAccess);
static int32_t QSPI_programInstance(QSPI_Config *config);
static int32_t QSPI_readData(QSPI_Handle handle, uint32_t *data, uint32_t length);
static int32_t QSPI_writeData(QSPI_Handle handle, const uint32_t *data, uint32_t length);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** \brief Driver object */
static QSPI_DrvObj gQspiDrvObj =
{
    .openLock      = NULL,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void QSPI_init(void)
{
    int32_t status;
    uint32_t count;
    QSPI_Object *obj;

    /* Init each driver instance object */
    for(count = 0U; count < gQspiConfigNum; count++)
    {
        /* Init object variables */
        obj = gQspiConfig[count].object;
        DebugP_assert(NULL != obj);
        memset(obj, 0, sizeof(QSPI_Object));
    }

    /* Create the driver lock */
    status = SemaphoreP_constructMutex(&gQspiDrvObj.lockObj);
    if(SystemP_SUCCESS == status)
    {
        gQspiDrvObj.openLock = &gQspiDrvObj.lockObj;
    }

    return;
}

void QSPI_deinit(void)
{
    /* Delete driver lock */
    if(NULL != gQspiDrvObj.openLock)
    {
        SemaphoreP_destruct(&gQspiDrvObj.lockObj);
        gQspiDrvObj.openLock = NULL;
    }

    return;
}

QSPI_Handle QSPI_open(uint32_t index, const QSPI_Params *openParams)
{
    int32_t status = SystemP_SUCCESS;
    QSPI_Handle handle = NULL;
    QSPI_Config *config = NULL;
    QSPI_Object *obj = NULL;
    HwiP_Params hwiPrms;
    const QSPI_Attrs *attrs;

    /* Check for valid index */
    if(index >= gQspiConfigNum)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        config = &gQspiConfig[index];
    }

    /* Protect this region from a concurrent QSPI_Open */
    DebugP_assert(NULL != gQspiDrvObj.openLock);
    SemaphoreP_pend(&gQspiDrvObj.lockObj, SystemP_WAIT_FOREVER);

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
        obj->handle = (QSPI_Handle)config;

        /* If DMA is enabled, program EDMA channel */
        if(TRUE == attrs->dmaEnable)
        {
            status = QSPI_edmaChannelConfig((QSPI_Handle)config, openParams->edmaInst);
        }

        /* Program QSPI instance according the user config */
        status += QSPI_programInstance(config);

        /* Create instance lock */
        status += SemaphoreP_constructMutex(&obj->lockObj);

        /* Create transfer sync semaphore */
        status += SemaphoreP_constructBinary(&obj->transferSemObj, 0U);

        /* Register interrupt */
        if(TRUE == attrs->intrEnable)
        {
            HwiP_Params_init(&hwiPrms);
            hwiPrms.intNum      = attrs->intrNum;
            hwiPrms.callback    = &QSPI_isr;
            hwiPrms.priority    = attrs->intrPriority;
            hwiPrms.args        = (void *) config;
            status += HwiP_construct(&obj->hwiObj, &hwiPrms);
        }
    }

    if(SystemP_SUCCESS == status)
    {
        obj->isOpen = 1;
        handle = (QSPI_Handle) config;
    }

    SemaphoreP_post(&gQspiDrvObj.lockObj);

    /* Free up resources in case of error */
    if(SystemP_SUCCESS != status)
    {
        if(NULL != config)
        {
            QSPI_close((QSPI_Handle) config);
        }
    }
    return handle;
}

void QSPI_close(QSPI_Handle handle)
{
    if(handle != NULL)
    {
        QSPI_Object *obj = ((QSPI_Config *)handle)->object;
        const QSPI_Attrs *attrs = ((QSPI_Config *)handle)->attrs;

        if(TRUE == attrs->dmaEnable)
        {
            QSPI_edmaChannelFree(handle);
        }

        QSPI_intDisable(handle,(CSL_QSPI_INTR_ENABLE_SET_REG_FIRQ_ENA_SET_MASK |
                        CSL_QSPI_INTR_ENABLE_SET_REG_WIRQ_ENA_SET_MASK));

        /* Destruct all locks and Hwi objects */
        SemaphoreP_destruct(&obj->lockObj);
        SemaphoreP_destruct(&obj->transferSemObj);
        HwiP_destruct(&obj->hwiObj);
        obj->isOpen = 0;
        SemaphoreP_post(&gQspiDrvObj.lockObj);
    }

    return;
}

QSPI_Handle QSPI_getHandle(uint32_t driverInstanceIndex)
{
    QSPI_Handle         handle = NULL;
    /* Check index */
    if(driverInstanceIndex < gQspiConfigNum)
    {
        QSPI_Object *obj;
        obj = gQspiConfig[driverInstanceIndex].object;

        if(obj && (TRUE == obj->isOpen))
        {
            /* valid handle */
            handle = obj->handle;
        }
    }
    return handle;
}

uint32_t QSPI_getInputClk(QSPI_Handle handle)
{
    uint32_t retVal = 0U;

    if(handle != NULL)
    {
        const QSPI_Attrs *attrs = ((QSPI_Config *)handle)->attrs;
        retVal = attrs->inputClkFreq;
    }

    return retVal;
}

static int32_t QSPI_programInstance(QSPI_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    const QSPI_Attrs *attrs = config->attrs;
    QSPI_Object *obj = config->object;
    QSPI_Handle handle = (QSPI_Handle)config;
    uint32_t regVal;
    const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)attrs->baseAddr;

    obj->rxLines  = attrs->rxLines;

    /* Set the idle mode value */
    CSL_REG32_FINS(&pReg->SYSCONFIG, QSPI_SYSCONFIG_IDLE_MODE,
                   CSL_QSPI_SYSCONFIG_IDLE_MODE_NO_IDLE);

    regVal = CSL_REG32_RD(&pReg->SPI_DC_REG);

    /* Set the values of clock phase, clock polarity, chip select polarity
     * and data delay value for the required chip select in the device.
     * The clock mode for different chip select are at a separation of
     * 8 bits in the device control register.
     * So 8U has been multiplied to the chip select value.
     */
    regVal &= (uint32_t)(~((QSPI_FF_POL1_PHA1) << (8U * attrs->chipSelect)));
    regVal |= (attrs->frmFmt << (8U * attrs->chipSelect));

    regVal &= (uint32_t)(~((QSPI_CS_POL_ACTIVE_HIGH) << (CSL_QSPI_SPI_DC_REG_CSP0_SHIFT +
                                     (8U * attrs->chipSelect))));
    regVal |= (attrs->csPol << (CSL_QSPI_SPI_DC_REG_CSP0_SHIFT +
                                     (8U * attrs->chipSelect)));

    regVal &= (uint32_t)(~((QSPI_DATA_DELAY_3) << (CSL_QSPI_SPI_DC_REG_DD0_SHIFT +
                            (8U * attrs->chipSelect))));
    regVal |= (attrs->dataDelay << (CSL_QSPI_SPI_DC_REG_DD0_SHIFT +
                            (8U * attrs->chipSelect)));

    status = QSPI_waitIdle(handle);
    CSL_REG32_WR(&pReg->SPI_DC_REG, regVal);
    /* Enable clock and set divider value */
    status += QSPI_setPreScaler(handle, attrs->baudRateDiv);
    /* Clear the interrupts and interrupt status */
    status += QSPI_intDisable(handle,(CSL_QSPI_INTR_ENABLE_SET_REG_FIRQ_ENA_SET_MASK |
                           CSL_QSPI_INTR_ENABLE_SET_REG_WIRQ_ENA_SET_MASK));
    status += QSPI_intClear(handle,(CSL_QSPI_INTR_ENABLE_SET_REG_FIRQ_ENA_SET_MASK |
                           CSL_QSPI_INTR_ENABLE_SET_REG_WIRQ_ENA_SET_MASK));
    /* Enable memory mapped port by default */
    status += QSPI_setMemAddrSpace(handle, QSPI_MEM_MAP_PORT_SEL_MEM_MAP_PORT);

    return status;
}

int32_t QSPI_setPreScaler(QSPI_Handle handle, uint32_t clkDividerVal)
{
    int32_t status = SystemP_SUCCESS;

    if(handle != NULL)
    {
        uint32_t regVal;
        const QSPI_Attrs *attrs = ((QSPI_Config *)handle)->attrs;
        const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)attrs->baseAddr;

        /* Read the value of Clock control register */
        regVal = CSL_REG32_RD(&pReg->SPI_CLOCK_CNTRL_REG);

        /* wait for QSPI to be idle */
        QSPI_waitIdle(handle);

        /* turn off QSPI data clock */
        CSL_FINS(regVal, QSPI_SPI_CLOCK_CNTRL_REG_CLKEN,
            CSL_QSPI_SPI_CLOCK_CNTRL_REG_CLKEN_DCLOCK_OFF);
        /* Set the value of QSPI clock control register */
        CSL_REG32_WR(&pReg->SPI_CLOCK_CNTRL_REG, regVal);

        /* Set the QSPI clock divider bit field value*/
        CSL_FINS(regVal, QSPI_SPI_CLOCK_CNTRL_REG_DCLK_DIV,
            clkDividerVal);
        /* Set the value of QSPI clock control register */
        CSL_REG32_WR(&pReg->SPI_CLOCK_CNTRL_REG, regVal);

        /* Enable the QSPI data clock */
        CSL_FINS(regVal, QSPI_SPI_CLOCK_CNTRL_REG_CLKEN,
                    CSL_QSPI_SPI_CLOCK_CNTRL_REG_CLKEN_DCLOCK_ON);
        /* Set the value of QSPI clock control register */
        CSL_REG32_WR(&pReg->SPI_CLOCK_CNTRL_REG, regVal);
    }
    else
    {
        status = SystemP_FAILURE;
    }
    return status;
}

static int32_t QSPI_waitIdle(QSPI_Handle handle)
{
    int32_t status = SystemP_SUCCESS;

    if(handle != NULL)
    {
        const QSPI_Attrs *attrs = ((QSPI_Config *)handle)->attrs;
        const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)attrs->baseAddr;

        /* wait while QSPI is busy */
        while ((CSL_REG32_RD(&pReg->SPI_STATUS_REG) & CSL_QSPI_SPI_STATUS_REG_BUSY_MASK) ==
            CSL_QSPI_SPI_STATUS_REG_BUSY_BUSY)
        {
            /*Do nothing */
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t QSPI_setMemAddrSpace(QSPI_Handle handle, uint32_t memMappedPortSwitch)
{
    int32_t status = SystemP_SUCCESS;
    const QSPI_Attrs *attrs = ((QSPI_Config *)handle)->attrs;
    const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)attrs->baseAddr;

    CSL_REG32_FINS(&pReg->SPI_SWITCH_REG, QSPI_SPI_SWITCH_REG_MMPT_S,
                    memMappedPortSwitch);

    return status;
}

int32_t QSPI_intEnable(QSPI_Handle handle, uint32_t intFlag)
{
    int32_t status = SystemP_SUCCESS;

    if(handle != NULL)
    {
        uint32_t regVal;
        const QSPI_Attrs *attrs = ((QSPI_Config *)handle)->attrs;
        const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)attrs->baseAddr;

        regVal = CSL_REG32_RD(&pReg->INTR_ENABLE_SET_REG);
        regVal |= intFlag;
        CSL_REG32_WR(&pReg->INTR_ENABLE_SET_REG, regVal);
    }
    else
    {
        status = SystemP_FAILURE;
    }
    return status;
}

int32_t QSPI_intDisable(QSPI_Handle handle, uint32_t intFlag)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t regVal;

    if(handle != NULL)
    {
        const QSPI_Attrs *attrs = ((QSPI_Config *)handle)->attrs;
        const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)attrs->baseAddr;
        regVal = CSL_REG32_RD(&pReg->INTR_ENABLE_CLEAR_REG);
        regVal |= intFlag;
        CSL_REG32_WR(&pReg->INTR_ENABLE_CLEAR_REG, regVal);
    }
    else
    {
        status = SystemP_FAILURE;
    }
    return status;
}

int32_t QSPI_intClear(QSPI_Handle handle, uint32_t intFlag)
{
    int32_t status = SystemP_SUCCESS;

    if(handle != NULL)
    {
        uint32_t regVal;
        const QSPI_Attrs *attrs = ((QSPI_Config *)handle)->attrs;
        const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)attrs->baseAddr;

        regVal = CSL_REG32_RD(&pReg->INTR_STATUS_ENABLED_CLEAR);
        regVal |= intFlag;
        CSL_REG32_WR(&pReg->INTR_STATUS_ENABLED_CLEAR, regVal);
    }
    else
    {
        status = SystemP_FAILURE;
    }
    return status;
}

static int32_t QSPI_writeData(QSPI_Handle handle, const uint32_t *data, uint32_t length)
{
    int32_t status = SystemP_SUCCESS;
    const uint32_t *pData;
    pData = data;

    if(handle != NULL)
    {
        const QSPI_Attrs *attrs = ((QSPI_Config *)handle)->attrs;
        const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)attrs->baseAddr;

        if(pData != ((void *) NULL))
        {
            CSL_REG32_WR(&pReg->SPI_DATA_REG, *pData);
            if (length > 1)
            {
                pData++;
                CSL_REG32_WR(&pReg->SPI_DATA_REG_1, *pData);
            }
            if (length > 2)
            {
                pData++;
                CSL_REG32_WR(&pReg->SPI_DATA_REG_2, *pData);
            }
            if (length > 3)
            {
                pData++;
                CSL_REG32_WR(&pReg->SPI_DATA_REG_3, *pData);
            }
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }
    return status;
}

static int32_t QSPI_readData(QSPI_Handle handle, uint32_t *data, uint32_t length)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t *pData;
    pData = data;

    if(handle != NULL)
    {
        const QSPI_Attrs *attrs = ((QSPI_Config *)handle)->attrs;
        const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)attrs->baseAddr;

        if(pData != ((void *) NULL))
        {
            *pData = CSL_REG32_RD(&pReg->SPI_DATA_REG);
            if (length > 1)
            {
                pData++;
                *pData = CSL_REG32_RD(&pReg->SPI_DATA_REG_1);
            }
            if (length > 2)
            {
                pData++;
                *pData = CSL_REG32_RD(&pReg->SPI_DATA_REG_2);
            }
            if (length > 3)
            {
                pData++;
                *pData = CSL_REG32_RD(&pReg->SPI_DATA_REG_3);
            }
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }
    return status;
}

void QSPI_transaction_init(QSPI_Transaction *trans)
{
    if( trans != NULL)
    {
        trans->count = 0U;
        trans->buf = NULL;
        trans->addrOffset = 0U;
        trans->transferTimeout = SystemP_WAIT_FOREVER;
        trans->status = QSPI_TRANSFER_STARTED;
    }
}

void QSPI_readCmdParams_init(QSPI_ReadCmdParams *rdParams)
{
    if( rdParams != NULL)
    {
        rdParams->cmd = QSPI_CMD_INVALID_OPCODE;
        rdParams->cmdAddr = QSPI_CMD_INVALID_ADDR;
        rdParams->numAddrBytes = 3;
        rdParams->rxDataBuf = NULL;
        rdParams->rxDataLen = 0;
    }
}

void QSPI_writeCmdParams_init(QSPI_WriteCmdParams *wrParams)
{
    if( wrParams != NULL)
    {
        wrParams->cmd = QSPI_CMD_INVALID_OPCODE;
        wrParams->cmdAddr = QSPI_CMD_INVALID_ADDR;
        wrParams->numAddrBytes = 3;
        wrParams->txDataBuf = NULL;
        wrParams->txDataLen = 0;
    }
}

int32_t QSPI_setWriteCmd(QSPI_Handle handle, uint8_t command)
{
    int32_t status = SystemP_SUCCESS;

    if(handle != NULL)
    {
        QSPI_Object  *object = ((QSPI_Config *)handle)->object;
        object->writeCmd = command;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t QSPI_setReadCmd(QSPI_Handle handle, uint8_t command)
{
    int32_t status = SystemP_SUCCESS;

    if(handle != NULL)
    {
        QSPI_Object  *object = ((QSPI_Config *)handle)->object;
        object->readCmd = command;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t QSPI_setAddressByteCount(QSPI_Handle handle, uint32_t count)
{
    int32_t status = SystemP_SUCCESS;

    if(handle != NULL)
    {
        QSPI_Object  *object = ((QSPI_Config *)handle)->object;
        object->numAddrBytes = count;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t QSPI_setDummyBitCount(QSPI_Handle handle, uint32_t count)
{
    int32_t status = SystemP_SUCCESS;

    if(handle != NULL)
    {
        QSPI_Object  *object = ((QSPI_Config *)handle)->object;
        object->numDummyBits = count;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t QSPI_setRxLines(QSPI_Handle handle, uint32_t rxLines)
{
    int32_t status = SystemP_SUCCESS;

    if(handle != NULL)
    {
        QSPI_Object  *object = ((QSPI_Config *)handle)->object;
        object->rxLines = rxLines;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

uint32_t QSPI_getRxLines(QSPI_Handle handle)
{
    uint32_t retVal = 0xFFFFFFFFU;
    if(handle != NULL)
    {
        QSPI_Object  *object = ((QSPI_Config *)handle)->object;
        retVal = object->rxLines;
    }
    return retVal;
}

static int32_t QSPI_spiConfigWrite(QSPI_Handle handle, QSPI_ConfigAccess *cfgAccess)
{
    /* Source address */
    uint8_t *srcAddr8 = NULL;
    uint16_t *srcAddr16 = NULL;
    uint32_t *srcAddr32 = NULL;
    uint32_t wordLenBytes;
    /* Data to be written */
    uint32_t dataVal[4] = {0};
    int32_t status = SystemP_SUCCESS;

    QSPI_Attrs const  *attrs = ((QSPI_Config *)handle)->attrs;
    const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)attrs->baseAddr;

    if (cfgAccess->wlen <= 8)
    {
        srcAddr8 = (uint8_t *)(cfgAccess->buf);
        wordLenBytes = 1;
    }
    else if (cfgAccess->wlen <= 16)
    {
        srcAddr16 = (uint16_t *)(cfgAccess->buf);
        wordLenBytes = 2;
    }
    else
    {
        srcAddr32 = (uint32_t *)(cfgAccess->buf);
        wordLenBytes = 4;
    }

    /* Write the data into shift registers */
    while(cfgAccess->count > 0)
    {
        dataVal[0] = 0;
        if (wordLenBytes == 1)
        {
            dataVal[0] = *srcAddr8;
            srcAddr8++;
        }
        else if (wordLenBytes == 2)
        {
            dataVal[0] = *srcAddr16;
            srcAddr16++;
        }
        else
        {
            dataVal[0] = *srcAddr32;
            srcAddr32++;
        }
        /* Write data to data registers */
        status += QSPI_writeData(handle, &dataVal[0], 1U);

        /* Wait for the QSPI busy status */
        status += QSPI_waitIdle(handle);

        /* Write tx command to command register */
        CSL_REG32_WR(&pReg->SPI_CMD_REG, cfgAccess->cmdRegVal);

        /* Wait for the QSPI busy status */
        status += QSPI_waitIdle(handle);

        /* update the cmd Val reg by reading it again for next word. */
        cfgAccess->cmdRegVal = CSL_REG32_RD(&pReg->SPI_CMD_REG);

        /* Update the number of bytes to be transmitted */
        cfgAccess->count -= wordLenBytes;
    }
    return status;
}

static int32_t QSPI_spiConfigRead(QSPI_Handle handle, QSPI_ConfigAccess *cfgAccess)
{
    /* Source address */
    uint8_t *dstAddr8 = NULL;
    uint16_t *dstAddr16 = NULL;
    uint32_t *dstAddr32 = NULL;
    uint32_t wordLenBytes;
    /* Data to be written */
    uint32_t dataVal[4] = {0};
    int32_t status = SystemP_SUCCESS;

    QSPI_Attrs const  *attrs = ((QSPI_Config *)handle)->attrs;
    const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)attrs->baseAddr;

    if (cfgAccess->wlen <= 8)
    {
        dstAddr8 = (uint8_t *)(cfgAccess->buf);
        wordLenBytes = 1;
    }
    else if (cfgAccess->wlen <= 16)
    {
        dstAddr16 = (uint16_t *)(cfgAccess->buf);
        wordLenBytes = 2;
    }
    else
    {
        dstAddr32 = (uint32_t *)(cfgAccess->buf);
        wordLenBytes = 4;
    }

    /* Write the data into shift registers */
    while(cfgAccess->count)
    {
        /* Write tx command to command register */
        CSL_REG32_WR(&pReg->SPI_CMD_REG, cfgAccess->cmdRegVal);

        /* Wait for the QSPI busy status */
        status += QSPI_waitIdle(handle);

        /* Store the number of data registers needed to read data */
        status += QSPI_readData(handle, &dataVal[0], 1U);
        if (wordLenBytes == 1)
        {
            *dstAddr8 = (uint8_t) dataVal[0];
            dstAddr8++;
        }
        else if (wordLenBytes == 2)
        {
            *dstAddr16 = (uint16_t) dataVal[0];
            dstAddr16++;
        }
        else
        {
            *dstAddr32 = (uint32_t) dataVal[0];
            dstAddr32++;
        }

        /* update the cmd Val reg by reading it again for next word. */
        cfgAccess->cmdRegVal = CSL_REG32_RD(&pReg->SPI_CMD_REG);

        /* Update the number of bytes to be transmitted */
        cfgAccess->count -= wordLenBytes;
    }
    return status;
}

static int32_t QSPI_spiMemMapRead(QSPI_Handle handle)
{
    /* Destination address */
    uint8_t *pDst = NULL;
    /* Source address */
    uint8_t *pSrc = NULL;
    /* Transaction length */
    uint32_t count;
    /* Memory mapped command */
    uint32_t mmapReadCmd;
    uintptr_t temp_addr;
    int32_t status = SystemP_SUCCESS;
    uint32_t dummyBytes, dummyBits;
    uint32_t dmaOffset;
    uint32_t nonAlignedBytes;
    uint8_t *pDmaDst  = NULL;
    uint32_t dmaLen;

    if(handle != NULL)
    {
        QSPI_Attrs const  *attrs = ((QSPI_Config *)handle)->attrs;
        QSPI_Object  *object = ((QSPI_Config *)handle)->object;
        QSPI_Transaction *transaction = object->transaction;
        const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)attrs->baseAddr;

        /* Extract memory map mode read command */
        mmapReadCmd = (uint32_t)object->readCmd;

        /* Set the number of address bytes  */
        CSL_REG32_FINS((&pReg->SPI_SETUP0_REG)+(attrs->chipSelect * 0x4U),
                    QSPI_SPI_SETUP0_REG_NUM_A_BYTES, (object->numAddrBytes - 1));

        dummyBytes = object->numDummyBits / 8U;
        dummyBits = object->numDummyBits % 8U;

        CSL_REG32_FINS((&pReg->SPI_SETUP0_REG)+(attrs->chipSelect * 0x4U),
                        QSPI_SPI_SETUP0_REG_NUM_D_BITS, dummyBits);
        CSL_REG32_FINS((&pReg->SPI_SETUP0_REG)+(attrs->chipSelect * 0x4U),
                        QSPI_SPI_SETUP0_REG_NUM_D_BYTES, dummyBytes);
        CSL_REG32_FINS((&pReg->SPI_SETUP0_REG)+(attrs->chipSelect * 0x4U),
                        QSPI_SPI_SETUP0_REG_RCMD, mmapReadCmd);

        switch(object->rxLines)
        {
            case QSPI_RX_LINES_SINGLE:
            {
                CSL_REG32_FINS((&pReg->SPI_SETUP0_REG)+(attrs->chipSelect * 0x4U),
                    QSPI_SPI_SETUP0_REG_READ_TYPE, QSPI_MEM_MAP_READ_TYPE_NORMAL);
                break;
            }

            case QSPI_RX_LINES_DUAL:
            {
                CSL_REG32_FINS((&pReg->SPI_SETUP0_REG)+(attrs->chipSelect * 0x4U),
                    QSPI_SPI_SETUP0_REG_READ_TYPE, QSPI_MEM_MAP_READ_TYPE_DUAL);
                break;
            }

            case QSPI_RX_LINES_QUAD:
            {
                CSL_REG32_FINS((&pReg->SPI_SETUP0_REG)+(attrs->chipSelect * 0x4U),
                    QSPI_SPI_SETUP0_REG_READ_TYPE, QSPI_MEM_MAP_READ_TYPE_QUAD);
                break;
            }

            default:
            break;
        }
        temp_addr = ((uintptr_t)attrs->memMapBaseAddr + (uintptr_t)transaction->addrOffset);
        pSrc = ((uint8_t *)(temp_addr));
        pDst = (uint8_t *)transaction->buf;
        count = transaction->count;

        if (attrs->dmaEnable == true)
        {
            /* Check if the qspi memory address is 4 byte aligned. */
            dmaOffset  = (transaction->addrOffset + 0x3) & (~0x3);
            nonAlignedBytes = dmaOffset - transaction->addrOffset;
            pDmaDst = (uint8_t *)(pDst + nonAlignedBytes);
            dmaLen = count - nonAlignedBytes;
            while(nonAlignedBytes != 0)
            {
                *pDst = *pSrc;
                pDst++;
                pSrc++;
                nonAlignedBytes--;
            }
            if (dmaLen != 0)
            {
                /* calculate the nonAligned bytes at the end */
                nonAlignedBytes = dmaLen - ((dmaLen ) & (~0x3));

                /* Get the previous multiple of 4 of dmaLen as edma transfer can only be done with length in multiple of 4*/
                dmaLen = (dmaLen ) & (~0x3);
                QSPI_edmaTransfer(pDmaDst, pSrc, dmaLen, handle);

                pDst += dmaLen;
                pSrc += dmaLen;

                /* Do the normal memory to memory transfer of nonAligned bytes at the end. */
                while(nonAlignedBytes != 0)
                {
                    *pDst = *pSrc;
                    pDst++;
                    pSrc++;
                    nonAlignedBytes--;
                }
            }
        }
        else
        {
            while(count)
            {
                /* Do the normal memory to memory transfer. Copy will be in bytes */
                *pDst = *pSrc;
                pDst++;
                pSrc++;
                count--;
            }
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }
    return status;
}

int32_t QSPI_readMemMapMode(QSPI_Handle handle, QSPI_Transaction *trans)
{
    int32_t status = SystemP_SUCCESS;

    if((handle != NULL) && (trans != NULL))
    {
        QSPI_Object *object = ((QSPI_Config *)handle)->object;
        object->transaction = trans;
        status = QSPI_spiMemMapRead(handle);
        if(status != SystemP_SUCCESS)
        {
            trans->status = QSPI_TRANSFER_FAILED;
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }
    return status;
}

int32_t QSPI_readCmd(QSPI_Handle handle, QSPI_ReadCmdParams *rdParams)
{
    int32_t status = SystemP_SUCCESS;

    if((handle != NULL) && (rdParams != NULL))
    {
        QSPI_Attrs const  *attrs = ((QSPI_Config *)handle)->attrs;
        QSPI_ConfigAccess cfgAccess = {0};
        uint32_t frmLength = 0;

        QSPI_setMemAddrSpace(handle, QSPI_MEM_MAP_PORT_SEL_CFG_PORT);

        if(rdParams->cmdAddr != QSPI_CMD_INVALID_ADDR)
        {
            /* Total transaction frame length in words (bytes) */
            frmLength = QSPI_CMD_LEN + QSPI_ADDR_LEN +
                                (rdParams->rxDataLen / (attrs->wrdLen >> 3U));
        }
        else
        {
            /* Total transaction frame length in words (bytes) */
            frmLength = QSPI_CMD_LEN + (rdParams->rxDataLen / (attrs->wrdLen >> 3U));
        }

        /* Send the command */
        cfgAccess.buf = (unsigned char *)&rdParams->cmd;
        cfgAccess.count = QSPI_CMD_LEN;
        cfgAccess.wlen = 8;
        /* formulate the command */
        CSL_FINS(cfgAccess.cmdRegVal, QSPI_SPI_CMD_REG_FLEN, (frmLength - 1));
        CSL_FINS(cfgAccess.cmdRegVal, QSPI_SPI_CMD_REG_CSNUM, attrs->chipSelect);
        CSL_FINS(cfgAccess.cmdRegVal, QSPI_SPI_CMD_REG_CMD,
                            CSL_QSPI_SPI_CMD_REG_CMD_FOUR_PIN_WRITE_SINGLE);
        CSL_FINS(cfgAccess.cmdRegVal, QSPI_SPI_CMD_REG_WLEN, (cfgAccess.wlen - 1));
        status = QSPI_spiConfigWrite(handle, &cfgAccess);

        /* Send address associated with command, if any */
        if(rdParams->cmdAddr != QSPI_CMD_INVALID_ADDR)
        {
            cfgAccess.buf = (unsigned char *)&rdParams->cmdAddr;
            cfgAccess.count = QSPI_ADDR_LEN;
            /* Number of address Bytes to bits. */
            cfgAccess.wlen = (rdParams->numAddrBytes << 3U);
            /* Update the command register value. */
            CSL_FINS(cfgAccess.cmdRegVal, QSPI_SPI_CMD_REG_WLEN, (cfgAccess.wlen - 1));
            status += QSPI_spiConfigWrite(handle, &cfgAccess);
        }

        /* Send data associated with command, if any */
        if( rdParams->rxDataLen != 0)
        {
            cfgAccess.buf = (unsigned char *)rdParams->rxDataBuf;
            cfgAccess.count = rdParams->rxDataLen / (attrs->wrdLen >> 3U);
            cfgAccess.wlen = attrs->wrdLen;
            /* Update the command register value. */
            CSL_FINS(cfgAccess.cmdRegVal, QSPI_SPI_CMD_REG_WLEN, (cfgAccess.wlen - 1));
            CSL_FINS(cfgAccess.cmdRegVal, QSPI_SPI_CMD_REG_CMD,
                                CSL_QSPI_SPI_CMD_REG_CMD_FOUR_PIN_READ_SINGLE);
            status += QSPI_spiConfigRead(handle, &cfgAccess);
        }
        QSPI_setMemAddrSpace(handle, QSPI_MEM_MAP_PORT_SEL_MEM_MAP_PORT);
    }
    else
    {
        status = SystemP_FAILURE;
    }
    return status;
}

int32_t QSPI_writeCmd(QSPI_Handle handle, QSPI_WriteCmdParams *wrParams)
{
    int32_t status = SystemP_SUCCESS;

    if((handle != NULL) && (wrParams != NULL))
    {
        QSPI_Attrs const  *attrs = ((QSPI_Config *)handle)->attrs;
        QSPI_ConfigAccess cfgAccess = {0};
        uint32_t frmLength = 0;

        QSPI_setMemAddrSpace(handle, QSPI_MEM_MAP_PORT_SEL_CFG_PORT);

        if(wrParams->cmdAddr != QSPI_CMD_INVALID_ADDR)
        {
            /* Total transaction frame length in words (bytes) */
            frmLength = QSPI_CMD_LEN + QSPI_ADDR_LEN +
                                (wrParams->txDataLen / (attrs->wrdLen >> 3U));
        }
        else
        {
            /* Total transaction frame length in words (bytes) */
            frmLength = QSPI_CMD_LEN + (wrParams->txDataLen / (attrs->wrdLen >> 3U));
        }

        /* Send the command */
        cfgAccess.buf = (unsigned char *)&wrParams->cmd;
        cfgAccess.count = QSPI_CMD_LEN;
        cfgAccess.wlen = 8;
        /* formulate the command */
        CSL_FINS(cfgAccess.cmdRegVal, QSPI_SPI_CMD_REG_FLEN, (frmLength - 1));
        CSL_FINS(cfgAccess.cmdRegVal, QSPI_SPI_CMD_REG_CSNUM, attrs->chipSelect);
        CSL_FINS(cfgAccess.cmdRegVal, QSPI_SPI_CMD_REG_CMD,
                            CSL_QSPI_SPI_CMD_REG_CMD_FOUR_PIN_WRITE_SINGLE);
        CSL_FINS(cfgAccess.cmdRegVal, QSPI_SPI_CMD_REG_WLEN, (cfgAccess.wlen - 1));
        status = QSPI_spiConfigWrite(handle, &cfgAccess);

        /* Send address associated with command, if any */
        if(wrParams->cmdAddr != QSPI_CMD_INVALID_ADDR)
        {
            cfgAccess.buf = (unsigned char *)&wrParams->cmdAddr;
            cfgAccess.count = QSPI_ADDR_LEN;
            /* Number of address Bytes to bits. */
            cfgAccess.wlen = (wrParams->numAddrBytes << 3U);
            /* Update the command register value. */
            CSL_FINS(cfgAccess.cmdRegVal, QSPI_SPI_CMD_REG_WLEN, (cfgAccess.wlen - 1));
            status += QSPI_spiConfigWrite(handle, &cfgAccess);
        }

        /* Send data associated with command, if any */
        if( wrParams->txDataLen != 0)
        {
            cfgAccess.buf = (unsigned char *)wrParams->txDataBuf;
            cfgAccess.count = wrParams->txDataLen / (attrs->wrdLen >> 3U);
            cfgAccess.wlen = attrs->wrdLen;
            /* Update the command register value. */
            CSL_FINS(cfgAccess.cmdRegVal, QSPI_SPI_CMD_REG_WLEN, (cfgAccess.wlen - 1));
            status += QSPI_spiConfigWrite(handle, &cfgAccess);
        }

        QSPI_setMemAddrSpace(handle, QSPI_MEM_MAP_PORT_SEL_MEM_MAP_PORT);
    }
    else
    {
        status = SystemP_FAILURE;
    }
    return status;
}

int32_t QSPI_writeConfigMode(QSPI_Handle handle, QSPI_Transaction *trans)
{
    int32_t status = SystemP_SUCCESS;

    if((handle != NULL) && (trans != NULL))
    {
        QSPI_WriteCmdParams wrParams;
        QSPI_Object  *object = ((QSPI_Config *)handle)->object;

        wrParams.cmd = object->writeCmd;
        wrParams.cmdAddr = trans->addrOffset;
        wrParams.numAddrBytes = object->numAddrBytes;
        wrParams.txDataBuf = trans->buf;
        wrParams.txDataLen = trans->count;

        status = QSPI_writeCmd(handle, &wrParams);
    }
    else
    {
        status = SystemP_FAILURE;
    }
    return status;
}

static void QSPI_isr(void *args)
{
    DebugP_log("Interrupt triggered\r\n");
    /* TODO */
    return ;
}

void OSPI_phyGetTuningData(uint32_t *tuningData, uint32_t *tuningDataSize)
{
    /* Dummy function. Turing data not supported for QSPI. */
    *tuningData = NULL;
    *tuningDataSize = 0;
}
