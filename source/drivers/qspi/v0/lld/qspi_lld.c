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
 *  \file qspi_lld.c
 *
 *  \brief File containing QSPI LLD Driver APIs implementation.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/qspi/v0/lld/qspi_lld.h>
#include <drivers/qspi/v0/lld/edma/qspi_edma_lld.h>
#include <drivers/soc.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/hw_types.h>
#include <string.h>
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief    QSPI Command default Length in SPI words */
#define QSPI_CMD_LEN             (1U)
/** \brief    QSPI Address default Length in SPI words */
#define QSPI_ADDR_LEN            (1U)
/** \brief    QSPI idle timeout in micro seconds. */
#define QSPI_IDLE_TIMEOUT_IN_US   ((uint32_t)1000000)
/** \brief    Word Length in bits */
#define QSPI_8BIT_WRDLEN         (1U)
#define QSPI_16BIT_WRDLEN        (2U)
#define QSPI_32BIT_WRDLEN        (4U)
#define QSPI_8BIT_WLEN           ((uint32_t)8)
#define QSPI_16BIT_WLEN          ((uint32_t)16)
/** \brief    Interrupt trigger */
#define QSPI_INTR_TRIG           (0x0)

/** \brief    QSPI Read Type - Single, Dual or Quad */
#define QSPI_MEM_MAP_READ_TYPE_NORMAL               \
                                (CSL_QSPI_SPI_SETUP0_REG_READ_TYPE_NORMAL_READ)
#define QSPI_MEM_MAP_READ_TYPE_DUAL                 \
                                (CSL_QSPI_SPI_SETUP0_REG_READ_TYPE_DUAL_READ)
#define QSPI_MEM_MAP_READ_TYPE_NORMAL_READTYPE      \
                                (CSL_QSPI_SPI_SETUP0_REG_READ_TYPE_NORMAL_READ_TYPE)
#define QSPI_MEM_MAP_READ_TYPE_QUAD                 \
                                (CSL_QSPI_SPI_SETUP0_REG_READ_TYPE_QUAD_READ)
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct
{
    uint32_t                count;
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

/* Driver internal functions */
static int32_t QSPI_waitIdle(QSPILLD_Handle hQspi);
static int32_t QSPI_spiMemMapRead(QSPILLD_Handle hQspi, void *buf, uint32_t addrOffset, uint32_t count, uint32_t timeout);
static int32_t QSPI_spiMemMapReadDma(QSPILLD_Handle hQspi, void *buf, uint32_t addrOffset, uint32_t count, uint32_t timeout);
static int32_t QSPI_spiConfigRead(QSPILLD_Handle hQspi, QSPI_ConfigAccess *cfgAccess);
static int32_t QSPI_spiConfigWrite(QSPILLD_Handle hQspi, QSPI_ConfigAccess *cfgAccess);
static int32_t QSPI_programInstance(QSPILLD_Handle hQspi);
static void    QSPI_readData(QSPILLD_Handle hQspi, uint32_t *data, uint32_t length);
static void    QSPI_writeData(QSPILLD_Handle hQspi, const uint32_t *data, uint32_t length);

/* LLD Parameter Validation */
static inline int32_t QSPI_lld_isChipSelectValid(uint32_t chipSelect);
static inline int32_t QSPI_lld_isChipSelectPolarityValid(uint32_t csPol);
static inline int32_t QSPI_lld_isFrameFormatValid(uint32_t frmFmt);
static inline int32_t QSPI_lld_isDataDelayValid(uint32_t dataDelay);
static inline int32_t QSPI_lld_isWordLengthValid(uint32_t wrdLen);
static inline int32_t QSPI_lld_isRxLineCountValid(uint32_t rxLines);
static inline int32_t QSPI_lld_isPortValid(uint32_t port);
static inline int32_t QSPI_lld_isInputClkFreq(uint32_t clkFreq);
static inline int32_t QSPI_lld_isHandleValid(QSPI_DmaHandle handle);
static inline int32_t QSPI_lld_isChannelValid(QSPI_DmaChConfig channel);

/* Interrupt internal function for enable and disable */
static void QSPI_memoryMapIntrEnable(QSPILLD_Handle hQspi);
static void QSPI_memoryMapIntrDisable(QSPILLD_Handle hQspi);
static void QSPI_wordIntrEnableClear(QSPILLD_Handle hQspi);
static void QSPI_frameIntrEnableClearReset(QSPILLD_Handle hQspi);
static void QSPI_wordIntrEnableSet(QSPILLD_Handle hQspi);
static void QSPI_frameIntrEnableSet(QSPILLD_Handle hQspi);
static void QSPI_wordIntrStatusEnableClear(QSPILLD_Handle hQspi);
static void QSPI_wordIntrStatusEnableClearReset(QSPILLD_Handle hQspi);
static void QSPI_frameIntrStatusEnableClearReset(QSPILLD_Handle hQspi);
static void QSPI_interruptSet(QSPILLD_Handle hQspi);

static int32_t QSPI_writeInterrupt(QSPILLD_Handle hQspi);
static void    QSPI_writeDataIntrInit(QSPILLD_Handle hQspi);
static void    QSPI_writeCommandIntrInit(QSPILLD_Handle hQspi);
static void    QSPI_writeCommandIntr(QSPILLD_Handle hQspi);
static int32_t QSPI_readInterrupt(QSPILLD_Handle hQspi);

/* Config Mode Read and Write Initialization*/
static void QSPI_writeCommandInit(QSPILLD_Handle hQspi, QSPI_ConfigAccess *cfgAccess, QSPILLD_WriteCmdParams *msg);
static void QSPI_writeAddressInit(QSPILLD_Handle hQspi, QSPI_ConfigAccess *cfgAccess, QSPILLD_WriteCmdParams *msg);
static void QSPI_writeDataInit(QSPILLD_Handle hQspi, QSPI_ConfigAccess *cfgAccess, const QSPILLD_WriteCmdParams *msg);
static void QSPI_readDataInit(QSPILLD_Handle hQspi, QSPI_ConfigAccess *cfgAccess, const QSPILLD_WriteCmdParams *msg);

/* QSPI Bool Expression Checker*/
static int32_t QSPI_lld_param_check(bool result);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t QSPI_lld_init(QSPILLD_Handle hQspi)
{
    int32_t               status = QSPI_SYSTEM_SUCCESS;
    QSPILLD_InitHandle    hQspiInit;

    /* Check if handle is Null */
    if((hQspi != NULL) && (hQspi->hQspiInit != NULL))
    {
        if(hQspi->state != QSPI_STATE_RESET)
        {
            status = QSPI_INVALID_STATE;
        }
    }
    else
    {
        status = QSPI_LLD_INVALID_PARAM;
    }

    if(QSPI_SYSTEM_SUCCESS == status)
    {
        hQspiInit = hQspi->hQspiInit;
        hQspi->state = QSPI_STATE_BUSY;

        /* Validate LLD parameters */
        status += QSPI_lld_param_check(IS_QSPI_BASE_ADDR_VALID(hQspi->baseAddr));
        status += QSPI_lld_param_check(IS_QSPI_MEMORY_MAP_ADDR_VALID(hQspiInit->memMapBaseAddr));
        status += QSPI_lld_isInputClkFreq(hQspiInit->inputClkFreq);
        status += QSPI_lld_isChipSelectValid(hQspiInit->chipSelect);
        status += QSPI_lld_isChipSelectPolarityValid(hQspiInit->csPol);
        status += QSPI_lld_isFrameFormatValid(hQspiInit->frmFmt);
        status += QSPI_lld_isDataDelayValid(hQspiInit->dataDelay);
        status += QSPI_lld_isWordLengthValid(hQspiInit->wrdLen);
        status += QSPI_lld_isRxLineCountValid(hQspiInit->rxLines);

        /* Program QSPI instance according the user config */
        status += QSPI_programInstance(hQspi);
    }

    if(QSPI_SYSTEM_SUCCESS == status)
    {
        hQspi->state = QSPI_STATE_READY;
    }
    else
    {
        /* Free-up resources in case of error */
        status += QSPI_lld_deInit(hQspi);
    }

    return status;
}

int32_t QSPI_lld_initDma(QSPILLD_Handle hQspi)
{
    int32_t               status = QSPI_SYSTEM_SUCCESS;
    QSPILLD_InitHandle    hQspiInit;

    /* Check if handle is Null */
    if((hQspi != NULL) && (hQspi->hQspiInit != NULL))
    {
        if(hQspi->state != QSPI_STATE_RESET)
        {
            status = QSPI_INVALID_STATE;
        }
    }
    else
    {
        status = QSPI_LLD_INVALID_PARAM;
    }

    if(QSPI_SYSTEM_SUCCESS == status)
    {
        hQspiInit = hQspi->hQspiInit;
        hQspi->state = QSPI_STATE_BUSY;

        /* Validate LLD parameters */
        status += QSPI_lld_param_check(IS_QSPI_BASE_ADDR_VALID(hQspi->baseAddr));
        status += QSPI_lld_param_check(IS_QSPI_MEMORY_MAP_ADDR_VALID(hQspiInit->memMapBaseAddr));
        status += QSPI_lld_isInputClkFreq(hQspiInit->inputClkFreq);
        status += QSPI_lld_isHandleValid(hQspiInit->qspiDmaHandle);
        status += QSPI_lld_isChannelValid(hQspiInit->qspiDmaChConfig);
        status += QSPI_lld_isChipSelectValid(hQspiInit->chipSelect);
        status += QSPI_lld_isChipSelectPolarityValid(hQspiInit->csPol);
        status += QSPI_lld_isFrameFormatValid(hQspiInit->frmFmt);
        status += QSPI_lld_isDataDelayValid(hQspiInit->dataDelay);
        status += QSPI_lld_isWordLengthValid(hQspiInit->wrdLen);
        status += QSPI_lld_isRxLineCountValid(hQspiInit->rxLines);

        if(QSPI_SYSTEM_SUCCESS == status)
        {
            /* EDMA Initialization */
            status += QSPI_edmaChannelConfig(hQspi);
        }
        if(QSPI_SYSTEM_SUCCESS == status)
        {
            /* Program QSPI instance according the user config */
            status += QSPI_programInstance(hQspi);
        }
    }

    if(QSPI_SYSTEM_SUCCESS == status)
    {
        hQspi->state = QSPI_STATE_READY;
    }
    else
    {
        /* Free-up resources in case of error */
        status += QSPI_lld_deInitDma(hQspi);
    }

    return status;
}

int32_t QSPI_lld_deInit(QSPILLD_Handle hQspi)
{
    int32_t             status = QSPI_SYSTEM_SUCCESS;

    /* Check if handle is Null */
    if(NULL != hQspi)
    {
        hQspi->state = QSPI_STATE_BUSY;

        if(hQspi->hQspiInit->intrEnable == true)
        {
            /* Disable the interrupt */
            QSPI_wordIntrEnableClear(hQspi);
        }

        hQspi->state = QSPI_STATE_RESET;
    }
    else
    {
        status = QSPI_LLD_INVALID_PARAM;
    }

    return status;
}

int32_t QSPI_lld_deInitDma(QSPILLD_Handle hQspi)
{
    int32_t             status = QSPI_SYSTEM_SUCCESS;

    /* Check if handle is Null */
    if(NULL != hQspi)
    {
        hQspi->state = QSPI_STATE_BUSY;

        /* Deinitialize the DMA */
        status += QSPI_edmaChannelFree(hQspi);

        if(hQspi->hQspiInit->intrEnable == true)
        {
            /* Disable the interrupt */
            QSPI_wordIntrEnableClear(hQspi);
        }

        hQspi->state = QSPI_STATE_RESET;
    }
    else
    {
        status = QSPI_LLD_INVALID_PARAM;
    }

    return status;
}

int32_t QSPI_lld_readCmd(QSPILLD_Handle hQspi,  QSPILLD_WriteCmdParams *writeMsg)
{
    int32_t status = QSPI_LLD_INVALID_PARAM;

    /* Check if handle is Null */
    if(hQspi != NULL)
    {
        QSPI_ConfigAccess cfgAccess = {0};
        QSPILLD_WriteCmdParams *msg  = writeMsg;

        status = QSPI_lld_setMemAddrSpace(hQspi, QSPI_MEM_MAP_PORT_SEL_CFG_PORT);

        if(QSPI_SYSTEM_SUCCESS == status)
        {
            QSPI_writeCommandInit(hQspi, &cfgAccess,msg);
            status = QSPI_spiConfigWrite(hQspi, &cfgAccess);

            /* Send address associated with command, if any */
            if(msg->cmdAddr != QSPI_LLD_CMD_INVALID_ADDR)
            {
                QSPI_writeAddressInit(hQspi, &cfgAccess,msg);
                status += QSPI_spiConfigWrite(hQspi, &cfgAccess);
            }

            /* Send data associated with command, if any */
            if( msg->dataLen != 0U)
            {
                QSPI_readDataInit(hQspi, &cfgAccess,msg);
                status += QSPI_spiConfigRead(hQspi, &cfgAccess);
            }
            if (QSPI_SYSTEM_SUCCESS == status)
            {
                status = QSPI_lld_setMemAddrSpace(hQspi, QSPI_MEM_MAP_PORT_SEL_MEM_MAP_PORT);
            }
        }
    }
    else
    {
        status = QSPI_LLD_INVALID_PARAM;
    }
    return status;
}

int32_t QSPI_lld_writeCmd(QSPILLD_Handle hQspi, QSPILLD_WriteCmdParams *writeMsg)
{
    int32_t status = QSPI_SYSTEM_SUCCESS;

    /* Check if handle is Null */
    if(hQspi != NULL)
    {
        QSPI_ConfigAccess cfgAccess = {0};
        QSPILLD_WriteCmdParams *msg = writeMsg;

        status = QSPI_lld_setMemAddrSpace(hQspi, QSPI_MEM_MAP_PORT_SEL_CFG_PORT);

        if(QSPI_LLD_INVALID_PARAM != status)
        {
            QSPI_writeCommandInit(hQspi, &cfgAccess,msg);
            status = QSPI_spiConfigWrite(hQspi, &cfgAccess);

            /* Send address associated with command, if any */
            if(msg->cmdAddr != QSPI_LLD_CMD_INVALID_ADDR)
            {
                QSPI_writeAddressInit(hQspi, &cfgAccess,msg);
                status += QSPI_spiConfigWrite(hQspi, &cfgAccess);
            }

            /* Send data associated with command, if any */
            if( msg->dataLen != 0U)
            {
                QSPI_writeDataInit(hQspi, &cfgAccess,msg);
                status += QSPI_spiConfigWrite(hQspi, &cfgAccess);
            }
            if (QSPI_SYSTEM_SUCCESS == status)
            {
                status = QSPI_lld_setMemAddrSpace(hQspi, QSPI_MEM_MAP_PORT_SEL_MEM_MAP_PORT);
            }
        }
    }
    else
    {
        status = QSPI_SYSTEM_FAILURE;
    }
    return status;
}

int32_t QSPI_lld_writeCmdIntr(QSPILLD_Handle hQspi, const QSPILLD_WriteCmdParams *msg)
{
    int32_t status = QSPI_SYSTEM_SUCCESS;

    if(hQspi != NULL)
    {
        hQspi->transaction = &(hQspi->trans);

        hQspi->transaction->dataLen = msg->dataLen;
        hQspi->transaction->buf = msg->dataBuf;

        /* Address of the memory to be written to or read from */
        hQspi->transaction->addrOffset = msg->cmdAddr;

        /* The command to send (8 bits) -> READ/WRITE/OTHER */
        hQspi->transaction->cmd = msg->cmd;

        /* Number of address bytes */
        hQspi->transaction->numAddrBytes = msg->numAddrBytes;

        /* Write/Read flag .. true: Write ; false: Read */
        hQspi->transaction->readWriteFlag = true;

        /* Transactio status */
        hQspi->transaction->status = false;

        status = QSPI_writeInterrupt(hQspi);
    }
    else
    {
        status = QSPI_SYSTEM_FAILURE;
    }
    return status;
}

int32_t QSPI_lld_readCmdIntr(QSPILLD_Handle hQspi, const QSPILLD_WriteCmdParams *msg)
{
    int32_t status = QSPI_SYSTEM_SUCCESS;
    if(hQspi != NULL)
    {
        hQspi->transaction = &(hQspi->trans);

        hQspi->transaction->dataLen = msg->dataLen;
        hQspi->transaction->buf = msg->dataBuf;

        /* Address of the memory to be written to or read from */
        hQspi->transaction->addrOffset = msg->cmdAddr;

        /* The command to send (8 bits) -> READ/WRITE/OTHER */
        hQspi->transaction->cmd = msg->cmd;

        /* Number of address bytes */
        hQspi->transaction->numAddrBytes = msg->numAddrBytes;

        /* Write/Read flag .. true: Write ; false: Read */
        hQspi->transaction->readWriteFlag = false;

        /* Transactio status */
        hQspi->transaction->status = false;

        status = QSPI_writeInterrupt(hQspi);
    }
    else
    {
        status = QSPI_SYSTEM_FAILURE;
    }
    return status;
}

int32_t QSPI_lld_read(QSPILLD_Handle hQspi, uint32_t count, void* rxBuf, uint32_t addrOffset, uint32_t timeout)
{
    int32_t status = QSPI_SYSTEM_SUCCESS;

    /* Check if handle is Null */
    (void)QSPI_lld_setMemAddrSpace(hQspi, QSPI_MEM_MAP_PORT_SEL_MEM_MAP_PORT);
    if((hQspi != NULL))
    {
        /* Read the QSPI memory mapped region */
        status = QSPI_spiMemMapRead(hQspi, rxBuf, addrOffset, count, timeout);
    }
    else
    {
        status = QSPI_SYSTEM_FAILURE;
    }
    return status;
}

int32_t QSPI_lld_readDma(QSPILLD_Handle hQspi, uint32_t count, void* rxBuf, uint32_t addrOffset, uint32_t timeout)
{
    int32_t status = QSPI_SYSTEM_SUCCESS;

    /* Check if handle is Null */
    if((hQspi != NULL))
    {
        /* Read the DMA */
        status = QSPI_spiMemMapReadDma(hQspi, rxBuf, addrOffset, count, timeout);
    }
    else
    {
        status = QSPI_SYSTEM_FAILURE;
    }
    return status;
}

static void QSPI_writeCommandIntr(QSPILLD_Handle hQspi)
{
    uint32_t dataVal = 0U;
    const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)hQspi->baseAddr;

    QSPILLD_Handle handle = (QSPILLD_Object *)hQspi;

    dataVal = (uint32_t)handle->transaction->cmd;

    /* Write the Command */
    QSPI_writeData(handle, &dataVal,1U);

    /* Wait for the QSPI busy status */
    QSPI_waitIdle(handle);

    /* Update transaction state from idle to write/read */
    if (handle->transaction->addrOffset != QSPI_LLD_CMD_INVALID_ADDR)
    {
        handle->transaction->state = QSPI_STATE_ADDRESS_WRITE;
    }
    else
    {
        handle->transaction->state = QSPI_STATE_DATA_WRITE;
    }
    /* Assigning the datelen to count for tracking transaction*/
    handle->transaction->count = handle->transaction->dataLen;

    /* Clear Interrrupt Status */
    QSPI_wordIntrStatusEnableClear(hQspi);
    /* Enable Interrupt */
    QSPI_wordIntrEnableSet(hQspi);
    QSPI_interruptSet(hQspi);

    /* Write tx command to command register */
    CSL_REG32_WR(&pReg->SPI_CMD_REG, hQspi->transaction->cmdRegVal);
}

static int32_t QSPI_spiConfigWrite(QSPILLD_Handle hQspi, QSPI_ConfigAccess *cfgAccess)
{
    /* Source address */
    uint8_t *srcAddr8 = NULL;
    uint16_t *srcAddr16 = NULL;
    uint32_t *srcAddr32 = NULL;
    uint32_t wordLenBytes;
    /* Data to be written */
    uint32_t dataVal[4] = {0};
    int32_t status = QSPI_SYSTEM_SUCCESS;
    const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)hQspi->baseAddr;

    if (cfgAccess->wlen <= 8U)
    {
        srcAddr8 = (uint8_t *)(cfgAccess->buf);
        wordLenBytes = 1U;
    }
    else if (cfgAccess->wlen <= 16U)
    {
        srcAddr16 = (uint16_t *)(cfgAccess->buf);
        wordLenBytes = 2U;
    }
    else
    {
        srcAddr32 = (uint32_t *)(cfgAccess->buf);
        wordLenBytes = 4U;
    }

    /* Write the data into shift registers */
    while(cfgAccess->count > 0U)
    {
        dataVal[0] = 0;
        if (wordLenBytes == 1U)
        {
            dataVal[0] = *srcAddr8;
            srcAddr8++;
        }
        else if (wordLenBytes == 2U)
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
        QSPI_writeData(hQspi, &dataVal[0], 1U);

        /* Wait for the QSPI busy status */
        status += QSPI_waitIdle(hQspi);

        /* Write tx command to command register */
        CSL_REG32_WR(&pReg->SPI_CMD_REG, cfgAccess->cmdRegVal);

        /* Wait for the QSPI busy status */
        status += QSPI_waitIdle(hQspi);

        /* update the cmd Val reg by reading it again for next word. */
        cfgAccess->cmdRegVal = CSL_REG32_RD(&pReg->SPI_CMD_REG);

        /* Update the number of bytes to be transmitted */
        cfgAccess->count -=  wordLenBytes;
    }
    return status;
}

static int32_t QSPI_spiConfigRead(QSPILLD_Handle hQspi, QSPI_ConfigAccess *cfgAccess)
{
    /* Source address */
    uint8_t *dstAddr8 = NULL;
    uint16_t *dstAddr16 = NULL;
    uint32_t *dstAddr32 = NULL;
    uint32_t wordLenBytes;
    bool readCount = true;
    /* Data to be written */
    uint32_t dataVal[4] = {0};
    int32_t status = QSPI_SYSTEM_SUCCESS;
    const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)hQspi->baseAddr;

    if (cfgAccess->wlen <= QSPI_8BIT_WLEN)
    {
        dstAddr8 = (uint8_t *)(cfgAccess->buf);
        wordLenBytes = (uint32_t) QSPI_8BIT_WRDLEN;
    }
    else if (cfgAccess->wlen <= QSPI_16BIT_WLEN)
    {
        dstAddr16 = (uint16_t *)(cfgAccess->buf);
        wordLenBytes = (uint32_t) QSPI_16BIT_WRDLEN;
    }
    else
    {
        dstAddr32 = (uint32_t *)(cfgAccess->buf);
        wordLenBytes = (uint32_t) QSPI_32BIT_WRDLEN;
    }

    /* Write the data into shift registers */
    while(readCount)
    {
        /* Write tx command to command register */
        CSL_REG32_WR(&pReg->SPI_CMD_REG, cfgAccess->cmdRegVal);

        /* Wait for the QSPI busy status */
        status += QSPI_waitIdle(hQspi);

        /* Store the number of data registers needed to read data */
        QSPI_readData(hQspi, &dataVal[0], 1U);
        if (wordLenBytes == QSPI_8BIT_WRDLEN)
        {
            *dstAddr8 = (uint8_t) dataVal[0];
            dstAddr8++;
        }
        else if (wordLenBytes == QSPI_16BIT_WRDLEN)
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
        if(cfgAccess->count <= 0U)
        {
            readCount = false;
        }
    }
    return status;
}

static int32_t QSPI_spiMemMapRead(QSPILLD_Handle hQspi, void *buf, uint32_t addrOffset, uint32_t count, uint32_t timeout)
{
    /* Destination address */
    uint8_t *pDst = NULL;
    /* Source address */
    uint8_t *pSrc = NULL;
    /* Memory mapped command */
    uint32_t mmapReadCmd;
    uint32_t temp_addr;
    int32_t status = QSPI_SYSTEM_SUCCESS;
    uint32_t dummyBytes, dummyBits;
    uint32_t byteCount = count;
    uint32_t startTicks, elapsedTicks, timeoutTicks = 0;
    QSPILLD_InitHandle hQspiInit;
    bool readFlag = true;

    hQspiInit = hQspi->hQspiInit;
    const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)hQspi->baseAddr;

    /* Extract memory map mode read command */
    mmapReadCmd = (uint32_t)hQspi->readCmd;

    /* Set the number of address bytes  */
    CSL_REG32_FINS((&pReg->SPI_SETUP0_REG)+(hQspiInit->chipSelect * 0x4U),
                QSPI_SPI_SETUP0_REG_NUM_A_BYTES, (hQspi->numAddrBytes - (uint32_t)1U));

    dummyBytes = hQspi->numDummyBits / (uint32_t)8U;
    dummyBits = hQspi->numDummyBits % (uint32_t)8U;

    CSL_REG32_FINS((((&pReg->SPI_SETUP0_REG)+(hQspiInit->chipSelect * (uint32_t)0x4U))),
                    QSPI_SPI_SETUP0_REG_NUM_D_BITS, dummyBits);
    CSL_REG32_FINS((&pReg->SPI_SETUP0_REG)+(hQspiInit->chipSelect * 0x4U),
                    QSPI_SPI_SETUP0_REG_NUM_D_BYTES, dummyBytes);
    CSL_REG32_FINS((&pReg->SPI_SETUP0_REG)+(hQspiInit->chipSelect * 0x4U),
                    QSPI_SPI_SETUP0_REG_RCMD, mmapReadCmd);

    switch(hQspiInit->rxLines)
    {
        case QSPI_RX_LINES_SINGLE:
        {
            CSL_REG32_FINS((&pReg->SPI_SETUP0_REG)+(hQspiInit->chipSelect * 0x4U),
                QSPI_SPI_SETUP0_REG_READ_TYPE, QSPI_MEM_MAP_READ_TYPE_NORMAL);
            break;
        }

        case QSPI_RX_LINES_DUAL:
        {
            CSL_REG32_FINS((&pReg->SPI_SETUP0_REG)+(hQspiInit->chipSelect * 0x4U),
                QSPI_SPI_SETUP0_REG_READ_TYPE, QSPI_MEM_MAP_READ_TYPE_DUAL);
            break;
        }

        case QSPI_RX_LINES_QUAD:
        {
            CSL_REG32_FINS((&pReg->SPI_SETUP0_REG)+(hQspiInit->chipSelect * 0x4U),
                QSPI_SPI_SETUP0_REG_READ_TYPE, QSPI_MEM_MAP_READ_TYPE_QUAD);
            break;
        }

        default:
        break;
    }
    temp_addr = (uint32_t) (hQspiInit->memMapBaseAddr + addrOffset);
    pSrc = (uint8_t *)temp_addr;
    pDst = (uint8_t *)buf;
    startTicks = hQspiInit->Clock_getTicks();
    timeoutTicks = (uint32_t)hQspiInit->Clock_usecToTicks(timeout);
    while(readFlag)
    {
        /* Do the normal memory to memory transfer. Copy will be in bytes */
        *pDst = *pSrc;
        pDst++;
        pSrc++;
        byteCount--;
        elapsedTicks = hQspiInit->Clock_getTicks() - startTicks;
        if((byteCount == 0U) || (elapsedTicks > timeoutTicks))
        {
            readFlag = false;
        }
    }
    if(elapsedTicks > timeoutTicks)
    {
        status = QSPI_TIMEOUT;
    }

    return status;
}

static int32_t QSPI_spiMemMapReadDma(QSPILLD_Handle hQspi, void *buf, uint32_t addrOffset, uint32_t count, uint32_t timeout)
{
    /* Destination address */
    uint8_t *pDst = NULL;
    /* Source address */
    uint8_t *pSrc = NULL;
    /* Memory mapped command */
    uint32_t mmapReadCmd;
    uint32_t temp_addr;
    int32_t status = QSPI_SYSTEM_SUCCESS;
    uint32_t dummyBytes, dummyBits;
    QSPILLD_InitHandle hQspiInit;
    uint32_t dmaOffset;
    uint32_t nonAlignedBytes;
    uint32_t edmaNonAlignedBytes;
    uint8_t *pDmaDst  = NULL;
    uint32_t dmaLen;

    hQspiInit = hQspi->hQspiInit;
    const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)hQspi->baseAddr;

    /* Extract memory map mode read command */
    mmapReadCmd = (uint32_t)hQspi->readCmd;

    /* Set the number of address bytes  */
    CSL_REG32_FINS((&pReg->SPI_SETUP0_REG)+(hQspiInit->chipSelect * 0x4U),
                QSPI_SPI_SETUP0_REG_NUM_A_BYTES, (hQspi->numAddrBytes - 1U));

    dummyBytes = hQspi->numDummyBits / 8U;
    dummyBits = hQspi->numDummyBits % 8U;

    CSL_REG32_FINS((&pReg->SPI_SETUP0_REG)+(hQspiInit->chipSelect * 0x4U),
                    QSPI_SPI_SETUP0_REG_NUM_D_BITS, dummyBits);
    CSL_REG32_FINS((&pReg->SPI_SETUP0_REG)+(hQspiInit->chipSelect * 0x4U),
                    QSPI_SPI_SETUP0_REG_NUM_D_BYTES, dummyBytes);
    CSL_REG32_FINS((&pReg->SPI_SETUP0_REG)+(hQspiInit->chipSelect * 0x4U),
                    QSPI_SPI_SETUP0_REG_RCMD, mmapReadCmd);

    switch(hQspiInit->rxLines)
    {
        case QSPI_RX_LINES_SINGLE:
        {
            CSL_REG32_FINS((&pReg->SPI_SETUP0_REG)+(hQspiInit->chipSelect * 0x4U),
                QSPI_SPI_SETUP0_REG_READ_TYPE, QSPI_MEM_MAP_READ_TYPE_NORMAL);
            break;
        }

        case QSPI_RX_LINES_DUAL:
        {
            CSL_REG32_FINS((&pReg->SPI_SETUP0_REG)+(hQspiInit->chipSelect * 0x4U),
                QSPI_SPI_SETUP0_REG_READ_TYPE, QSPI_MEM_MAP_READ_TYPE_DUAL);
            break;
        }

        case QSPI_RX_LINES_QUAD:
        {
            CSL_REG32_FINS((&pReg->SPI_SETUP0_REG)+(hQspiInit->chipSelect * 0x4U),
                QSPI_SPI_SETUP0_REG_READ_TYPE, QSPI_MEM_MAP_READ_TYPE_QUAD);
            break;
        }

        default:
        break;
    }
    temp_addr = (uint32_t)(hQspiInit->memMapBaseAddr + addrOffset);
    pSrc = ((uint8_t *)(temp_addr));
    pDst = (uint8_t *)buf;
    /* Check if the qspi memory address is 4 byte aligned. */
    dmaOffset  = (addrOffset + 0x3) & (~0x3);
    nonAlignedBytes = dmaOffset - addrOffset;
    pDmaDst = (uint8_t *)(pDst + nonAlignedBytes);
    dmaLen = count - nonAlignedBytes;

    while(nonAlignedBytes != 0U)
    {
        *pDst = *pSrc;
        pDst++;
        pSrc++;
        nonAlignedBytes--;
    }
    /* Update QSPI state to Non-Block Mode */
    hQspi->state = QSPI_STATE_NON_BLOCK;
    if(dmaLen != 0U)
    {
        /* calculate the nonAligned bytes at the end */
        nonAlignedBytes = dmaLen - ((dmaLen ) & (~0x3));
        edmaNonAlignedBytes = nonAlignedBytes;
        /* Get the previous multiple of 4 of dmaLen as edma transfer can only be done with length in multiple of 4*/
        dmaLen = (dmaLen ) & (~0x3);

        if(nonAlignedBytes != 0)
        {
            pDst += dmaLen;
            pSrc += dmaLen;

            /* Do the normal memory to memory transfer of nonAligned bytes at the end. */
            while (nonAlignedBytes != 0)
            {
                *pDst = *pSrc;
                pDst++;
                pSrc++;
                nonAlignedBytes--;
            }

            pDst -= (dmaLen + edmaNonAlignedBytes);
            pSrc -= (dmaLen + edmaNonAlignedBytes);

        }
        if (dmaLen != 0)
        {
            /* Update QSPI state to Block Mode */
            hQspi->state = QSPI_STATE_BLOCK;
            QSPI_edmaTransfer(pDmaDst, pSrc, dmaLen, hQspi, timeout);
        }

    }

    return status;
}

int32_t QSPI_lld_getInputClk(QSPILLD_Handle hQspi, uint32_t* inputClk)
{
    int32_t status = QSPI_SYSTEM_SUCCESS;

    if((hQspi != NULL) && (inputClk != NULL))
    {
        /* Get the input clock frequency */
        *inputClk = hQspi->hQspiInit->inputClkFreq;
    }
    else
    {
        status = QSPI_SYSTEM_FAILURE;
    }

    return status;
}

static int32_t QSPI_programInstance(QSPILLD_Handle hQspi)
{
    int32_t status = QSPI_SYSTEM_SUCCESS;
    uint32_t regVal;
    const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)hQspi->baseAddr;
    QSPILLD_InitHandle hQspiInit = hQspi->hQspiInit;

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
    regVal &= (uint32_t)(~((QSPI_FF_POL1_PHA1) << (8U * hQspiInit->chipSelect)));
    regVal |= (hQspiInit->frmFmt << (8U * hQspiInit->chipSelect));

    regVal &= (uint32_t)(~((QSPI_CS_POL_ACTIVE_HIGH) << (CSL_QSPI_SPI_DC_REG_CSP0_SHIFT +
                                     (8U * hQspiInit->chipSelect))));
    regVal |= (hQspiInit->csPol << (CSL_QSPI_SPI_DC_REG_CSP0_SHIFT +
                                     (8U * hQspiInit->chipSelect)));

    regVal &= (uint32_t)(~((QSPI_DATA_DELAY_3) << (CSL_QSPI_SPI_DC_REG_DD0_SHIFT +
                            (8U * hQspiInit->chipSelect))));
    regVal |= (hQspiInit->dataDelay << (CSL_QSPI_SPI_DC_REG_DD0_SHIFT +
                            (8U * hQspiInit->chipSelect)));

    status = QSPI_waitIdle(hQspi);
    CSL_REG32_WR(&pReg->SPI_DC_REG, regVal);
    /* Enable clock and set divider value */
    status += QSPI_lld_setPreScaler(hQspi, hQspiInit->qspiClockDiv);

    /* Clear the interrupts and interrupt status */
    if(hQspiInit->intrEnable != true)
    {
        QSPI_wordIntrEnableClear(hQspi);
        QSPI_wordIntrStatusEnableClearReset(hQspi);
        QSPI_frameIntrStatusEnableClearReset(hQspi);
        QSPI_memoryMapIntrDisable(hQspi);
    }
    else
    {
        QSPI_frameIntrEnableClearReset(hQspi);
        if(hQspiInit->wordIntr == true)
        {
            QSPI_wordIntrStatusEnableClear(hQspi);
        }
    }
    /* Enable memory mapped port by default */
    status += QSPI_lld_setMemAddrSpace(hQspi, QSPI_MEM_MAP_PORT_SEL_MEM_MAP_PORT);

    return status;
}

int32_t QSPI_lld_setPreScaler(QSPILLD_Handle hQspi, uint32_t clkDividerVal)
{
    int32_t status = QSPI_SYSTEM_SUCCESS;

    uint32_t regVal;
    const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)hQspi->baseAddr;

    /* Read the value of Clock control register */
    regVal = CSL_REG32_RD(&pReg->SPI_CLOCK_CNTRL_REG);

    /* wait for QSPI to be idle */
    status += QSPI_waitIdle(hQspi);

    /* turn off QSPI data clock */
    CSL_FINS(regVal, QSPI_SPI_CLOCK_CNTRL_REG_CLKEN,
        (uint32_t)CSL_QSPI_SPI_CLOCK_CNTRL_REG_CLKEN_DCLOCK_OFF);
    /* Set the value of QSPI clock control register */
    CSL_REG32_WR(&pReg->SPI_CLOCK_CNTRL_REG, regVal);

    /* Set the QSPI clock divider bit field value*/
    CSL_FINS(regVal, QSPI_SPI_CLOCK_CNTRL_REG_DCLK_DIV,
        clkDividerVal);
    /* Set the value of QSPI clock control register */
    CSL_REG32_WR(&pReg->SPI_CLOCK_CNTRL_REG, regVal);

    /* Enable the QSPI data clock */
    CSL_FINS(regVal, QSPI_SPI_CLOCK_CNTRL_REG_CLKEN,
                (uint32_t)CSL_QSPI_SPI_CLOCK_CNTRL_REG_CLKEN_DCLOCK_ON);
    /* Set the value of QSPI clock control register */
    CSL_REG32_WR(&pReg->SPI_CLOCK_CNTRL_REG, regVal);

    return status;
}

static int32_t QSPI_waitIdle(QSPILLD_Handle hQspi)
{
    int32_t status = QSPI_SYSTEM_SUCCESS;
    uint32_t startTicks, elapsedTicks = 0U ,timeoutTicks = 0U;
    QSPILLD_InitHandle hQspiInit = hQspi->hQspiInit;

    const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)hQspi->baseAddr;

    startTicks = hQspiInit->Clock_getTicks();
    timeoutTicks = (uint32_t)hQspiInit->Clock_usecToTicks(QSPI_IDLE_TIMEOUT_IN_US);
    /* wait while QSPI is busy */
    while (((((uint32_t)(CSL_REG32_RD(&pReg->SPI_STATUS_REG))) & CSL_QSPI_SPI_STATUS_REG_BUSY_MASK) ==
        CSL_QSPI_SPI_STATUS_REG_BUSY_BUSY) && (elapsedTicks < timeoutTicks))
    {
        elapsedTicks = (uint32_t) (hQspiInit->Clock_getTicks() - startTicks);
    }

    if(elapsedTicks >= timeoutTicks)
    {
        status = QSPI_TIMEOUT;
    }

    return status;
}

int32_t QSPI_lld_setMemAddrSpace(QSPILLD_Handle hQspi, uint32_t memMappedPortSwitch)
{
    int32_t status = QSPI_LLD_INVALID_PARAM;
    const CSL_QspiRegs *pReg;

    /* Check is handle is NULL */
    if(hQspi != NULL)
    {
        status = QSPI_lld_isPortValid(memMappedPortSwitch);
        if(status == QSPI_SYSTEM_SUCCESS)
        {

            pReg = (const CSL_QspiRegs *)hQspi->baseAddr;
            CSL_REG32_FINS(&pReg->SPI_SWITCH_REG, QSPI_SPI_SWITCH_REG_MMPT_S,
                        memMappedPortSwitch);
        }
    }
    return status;
}

static void QSPI_writeData(QSPILLD_Handle hQspi, const uint32_t *data, uint32_t length)
{
    const uint32_t *pData;
    pData = data;

    const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)hQspi->baseAddr;

    /* Write Data to QSPI memory */
    if(pData != ((void *) NULL))
    {
        CSL_REG32_WR(&pReg->SPI_DATA_REG, *pData);
        if (length > 1U)
        {
            pData++;
            CSL_REG32_WR(&pReg->SPI_DATA_REG_1, *pData);
        }
        if (length > 2U)
        {
            pData++;
            CSL_REG32_WR(&pReg->SPI_DATA_REG_2, *pData);
        }
        if (length > 3U)
        {
            pData++;
            CSL_REG32_WR(&pReg->SPI_DATA_REG_3, *pData);
        }
    }

}

static void QSPI_readData(QSPILLD_Handle hQspi, uint32_t *data, uint32_t length)
{
    uint32_t *pData;
    pData = data;

    const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)hQspi->baseAddr;

    /* Read the data from QSPI memory */
    if(pData != ((void *) NULL))
    {
        *pData = CSL_REG32_RD(&pReg->SPI_DATA_REG);
        if (length > 1U)
        {
            pData++;
            *pData = CSL_REG32_RD(&pReg->SPI_DATA_REG_1);
        }
        if (length > 2U)
        {
            pData++;
            *pData = CSL_REG32_RD(&pReg->SPI_DATA_REG_2);
        }
        if (length > 3U)
        {
            pData++;
            *pData = CSL_REG32_RD(&pReg->SPI_DATA_REG_3);
        }
    }

}

int32_t QSPI_lld_setWriteCmd(QSPILLD_Handle hQspi, uint8_t command)
{
    int32_t status = QSPI_SYSTEM_SUCCESS;

    /* Check is handle is NULL */
    if(hQspi != NULL)
    {
        /* Set QSPI write command */
        hQspi->writeCmd = command;
    }
    else
    {
        status = QSPI_SYSTEM_FAILURE;
    }

    return status;
}

int32_t QSPI_lld_setReadCmd(QSPILLD_Handle hQspi, uint8_t command)
{
    int32_t status = QSPI_SYSTEM_SUCCESS;

    /* Check is handle is NULL */
    if(hQspi != NULL)
    {
        /* Read the QSPI command */
        hQspi->readCmd = command;
    }
    else
    {
        status = QSPI_SYSTEM_FAILURE;
    }

    return status;
}

int32_t QSPI_lld_setAddressByteCount(QSPILLD_Handle hQspi, uint32_t count)
{
    int32_t status = QSPI_SYSTEM_SUCCESS;

    /* Check is handle is NULL */
    if(hQspi != NULL)
    {
        /* Set the QSPI address byte count */
        hQspi->numAddrBytes = count;
    }
    else
    {
        status = QSPI_SYSTEM_FAILURE;
    }

    return status;
}

int32_t QSPI_lld_setDummyBitCount(QSPILLD_Handle hQspi, uint32_t count)
{
    int32_t status = QSPI_SYSTEM_SUCCESS;

    /* Check is handle is NULL */
    if(hQspi != NULL)
    {
        /* Set QSPI dummy bit count */
        hQspi->numDummyBits = count;
    }
    else
    {
        status = QSPI_SYSTEM_FAILURE;
    }

    return status;
}

int32_t QSPI_lld_setRxLines(QSPILLD_Handle hQspi, uint32_t rxLines)
{
    int32_t status = QSPI_SYSTEM_SUCCESS;

    /* Check is handle is NULL */
    if(hQspi != NULL)
    {
        /* Set QSPI Rx lines */
        hQspi->hQspiInit->rxLines = rxLines;
    }
    else
    {
        status = QSPI_SYSTEM_FAILURE;
    }

    return status;
}

uint32_t QSPI_lld_getRxLines(QSPILLD_Handle hQspi)
{
    uint32_t retVal = 0xFFFFFFFFU;
    if(hQspi != NULL)
    {
        /* Get the qspi Rx lines */
        retVal = hQspi->hQspiInit->rxLines;
    }
    return retVal;
}

void QSPI_lld_isr(void* args)
{
    QSPILLD_Handle hQspi = (QSPILLD_Handle) args;
    uint8_t *dataBuf = NULL;
    uint32_t dataVal = 0U;
    const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)hQspi->baseAddr;
    /* Clear Interrupt Status */
    QSPI_wordIntrStatusEnableClear(hQspi);
    /* Enable the interrupt */
    QSPI_interruptSet(hQspi);

    switch(hQspi->transaction->state)
    {
        case QSPI_STATE_ADDRESS_WRITE:

            /* Update the command register value. */
            CSL_FINS(hQspi->transaction->cmdRegVal, QSPI_SPI_CMD_REG_WLEN, (uint32_t)((hQspi->transaction->numAddrBytes << (uint8_t)3) - (uint8_t)1));
            dataVal = hQspi->transaction->addrOffset;
            QSPI_writeData(hQspi, &dataVal,1U);
            hQspi->transaction->currentIndex = 0U;
            if (hQspi->transaction->readWriteFlag == true)
            {
                hQspi->transaction->state = QSPI_STATE_DATA_WRITE;
            }
            else
            {
                hQspi->transaction->state = QSPI_STATE_DATA_READ;
            }
            CSL_REG32_WR(&pReg->SPI_CMD_REG, hQspi->transaction->cmdRegVal);

            break;

        case QSPI_STATE_DATA_WRITE:

            if(hQspi->transaction->count != 0U)
            {
                /* Write Trigger & Data Transfer */
                dataBuf = (uint8_t *)(hQspi->transaction->buf);
                dataVal = *(dataBuf+hQspi->transaction->currentIndex);
                if(hQspi->transaction->currentIndex == 0U)
                {
                    /* Update the command register value. */
                    CSL_FINS(hQspi->transaction->cmdRegVal, QSPI_SPI_CMD_REG_WLEN, (7U));
                    CSL_FINS(hQspi->transaction->cmdRegVal, QSPI_SPI_CMD_REG_CSNUM, hQspi->hQspiInit->chipSelect);
                }

                QSPI_writeData(hQspi, &dataVal,1U);
                hQspi->transaction->count--;
                hQspi->transaction->currentIndex++;
                CSL_REG32_WR(&pReg->SPI_CMD_REG, hQspi->transaction->cmdRegVal);

            }
            else
            {
                hQspi->transaction->status = true;
                hQspi->transaction->state = QSPI_STATE_IDLE;
                QSPI_wordIntrEnableClear(hQspi);
                hQspi->interruptCallback(args);
            }

            break;

        case QSPI_STATE_DATA_READ:

            if (hQspi->transaction->count == hQspi->transaction->dataLen)
            {
                /* Update the command register value. */
                CSL_FINS(hQspi->transaction->cmdRegVal, QSPI_SPI_CMD_REG_WLEN, (uint32_t)(7U));
                CSL_FINS(hQspi->transaction->cmdRegVal, QSPI_SPI_CMD_REG_CMD,
                                (uint32_t)CSL_QSPI_SPI_CMD_REG_CMD_FOUR_PIN_READ_SINGLE);
                CSL_FINS(hQspi->transaction->cmdRegVal, QSPI_SPI_CMD_REG_CSNUM, hQspi->hQspiInit->chipSelect);
                hQspi->transaction->count --;
                CSL_REG32_WR(&pReg->SPI_CMD_REG, hQspi->transaction->cmdRegVal);


            }
            else
            {
                if(hQspi->transaction->count != 0U)
                {
                    /* Date Read */
                    dataBuf = (uint8_t *)(hQspi->transaction->buf);
                    /* Read data from the RX register */
                    QSPI_readData(hQspi, &dataVal, 1U);
                    /* Update the value into the data buffer */
                    *(dataBuf+hQspi->transaction->currentIndex) = (uint8_t) dataVal;
                    /* Update the transaction index and count*/
                    hQspi->transaction->currentIndex++;
                    hQspi->transaction->count --;
                    CSL_REG32_WR(&pReg->SPI_CMD_REG, hQspi->transaction->cmdRegVal);

                }
                else
                {
                    /* Date Read */
                    dataBuf = (uint8_t *)(hQspi->transaction->buf);
                    /* Read data from the RX register */
                    QSPI_readData(hQspi, &dataVal, 1U);
                    /* Update the value into the data buffer */
                    *(dataBuf+hQspi->transaction->currentIndex) = (uint8_t) dataVal;
                    hQspi->transaction->status = true;
                    hQspi->transaction->state = QSPI_STATE_IDLE;
                    QSPI_wordIntrEnableClear(hQspi);
                    hQspi->interruptCallback(args);

                }
            }
            break;

        default:
            /* Not in Use */
            break;
    }
}

void QSPI_lld_readCompleteCallback(void* args)
{
    QSPILLD_Handle hQspi = (QSPILLD_Handle) args;
    if (NULL == hQspi->readCompleteCallback)
    {
        /*  No Call Back Function Registered */
        QSPI_NOT_IN_USE(hQspi);
    }
    else
    {
        /* Read complete callback for DMA mode */
        hQspi->readCompleteCallback(hQspi);
    }
}

static inline int32_t QSPI_lld_isChipSelectValid(uint32_t chipSelect)
{
    int32_t status = QSPI_LLD_INVALID_PARAM;
    if((chipSelect == QSPI_CS0) || (chipSelect == QSPI_CS1) ||
       (chipSelect == QSPI_CS2) || (chipSelect == QSPI_CS3))
       {
            status = QSPI_SYSTEM_SUCCESS;
       }
    return status;
}

static inline int32_t QSPI_lld_isChipSelectPolarityValid(uint32_t csPol)
{
    int32_t status = QSPI_LLD_INVALID_PARAM;
    if((csPol == QSPI_CS_POL_ACTIVE_LOW) || (csPol == QSPI_CS_POL_ACTIVE_HIGH))
    {
        status = QSPI_SYSTEM_SUCCESS;
    }
    return status;
}

static inline int32_t QSPI_lld_isFrameFormatValid(uint32_t frmFmt)
{
    int32_t status = QSPI_LLD_INVALID_PARAM;
    if ((frmFmt == QSPI_FF_POL0_PHA0) || (frmFmt == QSPI_FF_POL0_PHA1) ||
        (frmFmt == QSPI_FF_POL1_PHA0) || (frmFmt == QSPI_FF_POL1_PHA1))
    {
        status = QSPI_SYSTEM_SUCCESS;
    }
    return status;
}

static inline int32_t QSPI_lld_isDataDelayValid(uint32_t dataDelay)
{
    int32_t status = QSPI_LLD_INVALID_PARAM;
    if((dataDelay == QSPI_DATA_DELAY_0) || (dataDelay == QSPI_DATA_DELAY_1) ||
       (dataDelay == QSPI_DATA_DELAY_2) || (dataDelay == QSPI_DATA_DELAY_3))
    {
        status = QSPI_SYSTEM_SUCCESS;
    }
    return status;
}

static inline int32_t QSPI_lld_isWordLengthValid(uint32_t wrdLen)
{
    int32_t status = QSPI_LLD_INVALID_PARAM;
    if ((wrdLen >= 1U) && (wrdLen <= QSPI_MAX_WORD_LENGTH))
    {
        status = QSPI_SYSTEM_SUCCESS;
    }
    return status;
}

static inline int32_t QSPI_lld_isRxLineCountValid(uint32_t rxLines)
{
    int32_t status = QSPI_LLD_INVALID_PARAM;
    if ((rxLines == QSPI_RX_LINES_SINGLE) || (rxLines == QSPI_RX_LINES_DUAL) ||
        (rxLines == QSPI_RX_LINES_QUAD))
    {
        status = QSPI_SYSTEM_SUCCESS;
    }
    return status;
}

static inline int32_t QSPI_lld_isPortValid(uint32_t port)
{
    int32_t status = QSPI_LLD_INVALID_PARAM;
    if ((port == QSPI_MEM_MAP_PORT_SEL_CFG_PORT) ||
        (port == QSPI_MEM_MAP_PORT_SEL_MEM_MAP_PORT))
    {
        status = QSPI_SYSTEM_SUCCESS;
    }
    return status;
}

static inline int32_t QSPI_lld_isInputClkFreq(uint32_t clkFreq)
{
    int32_t status = QSPI_LLD_INVALID_PARAM;
    if (clkFreq != 0U)
    {
        status = QSPI_SYSTEM_SUCCESS;
    }
    return status;
}

static inline int32_t QSPI_lld_isHandleValid(QSPI_DmaHandle handle)
{
    int32_t status = QSPI_LLD_INVALID_PARAM;
    if (handle != NULL)
    {
        status = QSPI_SYSTEM_SUCCESS;
    }
    return status;
}

static inline int32_t QSPI_lld_isChannelValid(QSPI_DmaChConfig channel)
{
    int32_t status = QSPI_LLD_INVALID_PARAM;
    if (channel != NULL)
    {
        status = QSPI_SYSTEM_SUCCESS;
    }
    return status;
}

static void QSPI_interruptSet(QSPILLD_Handle hQspi)
{
    const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)hQspi->baseAddr;

    CSL_REG32_FINS((&pReg->INTC_EOI_REG),
        QSPI_INTC_EOI_REG_EOI_VECTOR, QSPI_INTR_TRIG);
}

static void QSPI_memoryMapIntrEnable(QSPILLD_Handle hQspi)
{
    const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)hQspi->baseAddr;
    CSL_REG32_FINS((&pReg->SPI_SWITCH_REG),
                    QSPI_SPI_SWITCH_REG_MM_INT_EN, CSL_QSPI_SPI_SWITCH_REG_MM_INT_EN_MM_MODE_INTR_ENABLED );
}

static void QSPI_memoryMapIntrDisable(QSPILLD_Handle hQspi)
{
    const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)hQspi->baseAddr;
    CSL_REG32_FINS((&pReg->SPI_SWITCH_REG),
                    QSPI_SPI_SWITCH_REG_MM_INT_EN, CSL_QSPI_SPI_SWITCH_REG_MM_INT_EN_MM_MODE_INTR_DISABLED );
}

static void QSPI_wordIntrEnableClear(QSPILLD_Handle hQspi)
{
    uint32_t regVal;

    /* Disable the word interrupt */
    const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)hQspi->baseAddr;
    regVal = CSL_REG32_RD(&pReg->INTR_ENABLE_CLEAR_REG);
    regVal |= (CSL_QSPI_INTR_ENABLE_SET_REG_WIRQ_ENA_SET_MASK |
        (CSL_QSPI_INTR_ENABLE_SET_REG_WIRQ_ENA_SET_ACTIVE << CSL_QSPI_INTR_ENABLE_SET_REG_WIRQ_ENA_SET_SHIFT));
    CSL_REG32_WR(&pReg->INTR_ENABLE_CLEAR_REG, regVal);
}

static void QSPI_frameIntrEnableClearReset(QSPILLD_Handle hQspi)
{
    uint32_t regVal;

    /* Disable the word interrupt */
    const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)hQspi->baseAddr;
    regVal = CSL_REG32_RD(&pReg->INTR_ENABLE_CLEAR_REG);
    regVal |= ((CSL_QSPI_INTR_ENABLE_CLEAR_REG_FIRQ_ENA_CLR_MASK |
                    CSL_QSPI_INTR_ENABLE_CLEAR_REG_FIRQ_ENA_CLR_RESETVAL));
    CSL_REG32_WR(&pReg->INTR_ENABLE_CLEAR_REG, regVal);
}

static void QSPI_wordIntrEnableSet(QSPILLD_Handle hQspi)
{
    uint32_t regVal;
    const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)hQspi->baseAddr;

    /* Clear the interrupt */
    regVal = CSL_REG32_RD(&pReg->INTR_ENABLE_SET_REG);
    regVal |= (CSL_QSPI_INTR_ENABLE_SET_REG_WIRQ_ENA_SET_MASK |
                (CSL_QSPI_INTR_ENABLE_SET_REG_WIRQ_ENA_SET_ACTIVE << CSL_QSPI_INTR_ENABLE_SET_REG_WIRQ_ENA_SET_SHIFT));
    CSL_REG32_WR(&pReg->INTR_ENABLE_SET_REG, regVal);
}


static void QSPI_frameIntrEnableSet(QSPILLD_Handle hQspi)
{
    uint32_t regVal;
    const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)hQspi->baseAddr;

    /* Clear the interrupt */
    regVal = CSL_REG32_RD(&pReg->INTR_ENABLE_SET_REG);
    regVal |= (CSL_QSPI_INTR_ENABLE_SET_REG_FIRQ_ENA_SET_MASK |
                (CSL_QSPI_INTR_ENABLE_SET_REG_FIRQ_ENA_SET_ACTIVE << CSL_QSPI_INTR_ENABLE_SET_REG_FIRQ_ENA_SET_SHIFT));
    CSL_REG32_WR(&pReg->INTR_ENABLE_SET_REG, regVal);
}


static void QSPI_wordIntrStatusEnableClear(QSPILLD_Handle hQspi)
{
    uint32_t regVal;
    const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)hQspi->baseAddr;

    /* Clear the interrupt */
    regVal = CSL_REG32_RD(&pReg->INTR_STATUS_ENABLED_CLEAR);
    regVal |= (CSL_QSPI_INTR_STATUS_ENABLED_CLEAR_WIRQ_ENA_MASK |
                (CSL_QSPI_INTR_STATUS_ENABLED_CLEAR_WIRQ_ENA_ACTIVE << CSL_QSPI_INTR_STATUS_ENABLED_CLEAR_WIRQ_ENA_SHIFT));
    CSL_REG32_WR(&pReg->INTR_STATUS_ENABLED_CLEAR, regVal);
}

static void QSPI_wordIntrStatusEnableClearReset(QSPILLD_Handle hQspi)
{
    uint32_t regVal;
    const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)hQspi->baseAddr;

    /* Clear the interrupt */
    regVal = CSL_REG32_RD(&pReg->INTR_STATUS_ENABLED_CLEAR);
    regVal |= (CSL_QSPI_INTR_STATUS_ENABLED_CLEAR_WIRQ_ENA_MASK |
                (CSL_QSPI_INTR_STATUS_ENABLED_CLEAR_WIRQ_ENA_RESETVAL << CSL_QSPI_INTR_STATUS_ENABLED_CLEAR_WIRQ_ENA_SHIFT));
    CSL_REG32_WR(&pReg->INTR_STATUS_ENABLED_CLEAR, regVal);
}

static void QSPI_frameIntrStatusEnableClearReset(QSPILLD_Handle hQspi)
{
    uint32_t regVal;
    const CSL_QspiRegs *pReg = (const CSL_QspiRegs *)hQspi->baseAddr;

    /* Clear the interrupt */
    regVal = CSL_REG32_RD(&pReg->INTR_STATUS_ENABLED_CLEAR);
    regVal |= (CSL_QSPI_INTR_STATUS_ENABLED_CLEAR_FIRQ_ENA_MASK |
                (CSL_QSPI_INTR_STATUS_ENABLED_CLEAR_FIRQ_ENA_RESETVAL << CSL_QSPI_INTR_STATUS_ENABLED_CLEAR_FIRQ_ENA_SHIFT));
    CSL_REG32_WR(&pReg->INTR_STATUS_ENABLED_CLEAR, regVal);
}

static void QSPI_writeCommandInit(QSPILLD_Handle hQspi, QSPI_ConfigAccess *cfgAccess, QSPILLD_WriteCmdParams *msg)
{
    QSPILLD_InitHandle hQspiInit;
    hQspiInit = hQspi->hQspiInit;
    uint32_t frmLength = 0;

    if(msg->cmdAddr != QSPI_LLD_CMD_INVALID_ADDR)
    {
        /* Total transaction frame length in words (bytes) */
        frmLength = QSPI_CMD_LEN + QSPI_ADDR_LEN +
                            (msg->dataLen / (hQspiInit->wrdLen >> 3U));
    }
    else
    {
        /* Total transaction frame length in words (bytes) */
        frmLength = QSPI_CMD_LEN + (msg->dataLen / (hQspiInit->wrdLen >> 3U));
    }

    /* Send the command */
    cfgAccess->buf = (uint8_t *)&msg->cmd;
    cfgAccess->count = (int32_t)QSPI_CMD_LEN;
    cfgAccess->wlen = 8;

    /* formulate the command */
    CSL_FINS(cfgAccess->cmdRegVal, QSPI_SPI_CMD_REG_FLEN, (frmLength - 1U));
    CSL_FINS(cfgAccess->cmdRegVal, QSPI_SPI_CMD_REG_CSNUM, hQspiInit->chipSelect);
    CSL_FINS(cfgAccess->cmdRegVal, QSPI_SPI_CMD_REG_CMD,
                        (uint32_t)CSL_QSPI_SPI_CMD_REG_CMD_FOUR_PIN_WRITE_SINGLE);
    CSL_FINS(cfgAccess->cmdRegVal, QSPI_SPI_CMD_REG_WLEN, (cfgAccess->wlen - 1U));
}

static void QSPI_writeAddressInit(QSPILLD_Handle hQspi, QSPI_ConfigAccess *cfgAccess, QSPILLD_WriteCmdParams *msg)
{
    cfgAccess->buf = (uint8_t *)&msg->cmdAddr;
    /* Number of address Bytes to bits. */
    cfgAccess->wlen = (uint32_t)(msg->numAddrBytes << (uint8_t)3);
    /* Address will in 3 bytes*/
    cfgAccess->count = (uint32_t) (QSPI_ADDR_LEN << (uint8_t)2);
    /* Update the command register value. */
    CSL_FINS(cfgAccess->cmdRegVal, QSPI_SPI_CMD_REG_WLEN, (cfgAccess->wlen - 1U));
}

static void QSPI_writeDataInit(QSPILLD_Handle hQspi, QSPI_ConfigAccess *cfgAccess, const QSPILLD_WriteCmdParams *msg)
{
    QSPILLD_InitHandle hQspiInit;
    hQspiInit = hQspi->hQspiInit;
    cfgAccess->buf = (uint8_t *)msg->dataBuf;
    // cfgAccess->count = (msg->dataLen / (hQspiInit->wrdLen >> 3U));
    cfgAccess->count = msg->dataLen;
    cfgAccess->wlen = hQspiInit->wrdLen;
    /* Update the command register value. */
    CSL_FINS(cfgAccess->cmdRegVal, QSPI_SPI_CMD_REG_WLEN, (cfgAccess->wlen - 1U));
    CSL_FINS(cfgAccess->cmdRegVal, QSPI_SPI_CMD_REG_CMD,
                                    (uint32_t)CSL_QSPI_SPI_CMD_REG_CMD_FOUR_PIN_WRITE_SINGLE);
}

static void QSPI_readDataInit(QSPILLD_Handle hQspi, QSPI_ConfigAccess *cfgAccess, const QSPILLD_WriteCmdParams *msg)
{
    QSPILLD_InitHandle hQspiInit;
    hQspiInit = hQspi->hQspiInit;
    cfgAccess->buf = (uint8_t *)msg->dataBuf;
    // cfgAccess->count = (msg->dataLen / (hQspiInit->wrdLen >> 3U));
    cfgAccess->count = msg->dataLen;
    cfgAccess->wlen = hQspiInit->wrdLen;
    /* Update the command register value. */
    CSL_FINS(cfgAccess->cmdRegVal, QSPI_SPI_CMD_REG_WLEN, (cfgAccess->wlen - 1U));
    CSL_FINS(cfgAccess->cmdRegVal, QSPI_SPI_CMD_REG_CMD,
                                    (uint32_t)CSL_QSPI_SPI_CMD_REG_CMD_FOUR_PIN_READ_SINGLE);
}

static void QSPI_writeCommandIntrInit(QSPILLD_Handle hQspi)
{
    QSPILLD_InitHandle hQspiInit;

    hQspiInit = hQspi->hQspiInit;

    uint32_t frmLength = 0;

    /* Address is valid (Command + Address + Data) */
    if(hQspi->transaction->addrOffset != QSPI_LLD_CMD_INVALID_ADDR)
    {
        /* Total transaction frame length in words (4 bytes max) */
        frmLength = QSPI_CMD_LEN + QSPI_ADDR_LEN +
                            (hQspi->transaction->dataLen / (hQspiInit->wrdLen >> 3U));
    }
    /* Address is invalid (Command + Data) */
    else
    {
        /* Total transaction frame length in words (4 bytes max) */
        frmLength = QSPI_CMD_LEN + (hQspi->transaction->dataLen / (hQspiInit->wrdLen >> 3U));
    }

    /* Send the command */
    hQspi->transaction->wlen = 8;

    /* formulate the command */
    CSL_FINS(hQspi->transaction->cmdRegVal, QSPI_SPI_CMD_REG_FLEN, (frmLength - 1U));
    CSL_FINS(hQspi->transaction->cmdRegVal, QSPI_SPI_CMD_REG_CSNUM, hQspiInit->chipSelect);
    CSL_FINS(hQspi->transaction->cmdRegVal, QSPI_SPI_CMD_REG_CMD,
                        (uint32_t)CSL_QSPI_SPI_CMD_REG_CMD_FOUR_PIN_WRITE_SINGLE);
    CSL_FINS(hQspi->transaction->cmdRegVal, QSPI_SPI_CMD_REG_WLEN, (hQspi->transaction->wlen - 1U));
    CSL_FINS(hQspi->transaction->cmdRegVal, QSPI_SPI_CMD_REG_WIRQ, (uint32_t)(CSL_QSPI_SPI_CMD_REG_WIRQ_WORD_COUNT_IRQ_ENABLE));
}

static void QSPI_writeDataIntrInit(QSPILLD_Handle hQspi)
{
    QSPILLD_InitHandle hQspiInit;
    hQspiInit = hQspi->hQspiInit;
    hQspi->transaction->count = (uint32_t) (hQspi->transaction->dataLen / (hQspiInit->wrdLen >> 3U));
    hQspi->transaction->wlen = hQspiInit->wrdLen;
    hQspi->transaction->currentIndex = 0;
    /* Update the command register value. */
    CSL_FINS(hQspi->transaction->cmdRegVal, QSPI_SPI_CMD_REG_WLEN, (hQspi->transaction->wlen - 1U));
    CSL_FINS(hQspi->transaction->cmdRegVal, QSPI_SPI_CMD_REG_WIRQ, (uint32_t)(CSL_QSPI_SPI_CMD_REG_WIRQ_WORD_COUNT_IRQ_ENABLE));
}


static int32_t QSPI_writeInterrupt(QSPILLD_Handle hQspi)
{
    int32_t status = QSPI_SYSTEM_SUCCESS;

    /* Put the QSPI in configuration mode */
    status = QSPI_lld_setMemAddrSpace(hQspi, QSPI_MEM_MAP_PORT_SEL_CFG_PORT);

    /* Set up transaction details -> Frame lenght / word lenght / IRQ ---> just for the Command */
    QSPI_writeCommandIntrInit(hQspi);

    QSPI_writeCommandIntr(hQspi);
    return status;
}

static int32_t QSPI_readInterrupt(QSPILLD_Handle hQspi)
{
    int32_t status = QSPI_SYSTEM_SUCCESS;

    /* Put the QSPI in configuration mode */
    status = QSPI_lld_setMemAddrSpace(hQspi, QSPI_MEM_MAP_PORT_SEL_CFG_PORT);

    /* Set up transaction details -> Frame lenght / word lenght / IRQ ---> just for the Command */
    QSPI_writeCommandIntrInit(hQspi);

    QSPI_writeCommandIntr(hQspi);
    return status;
}

static int32_t QSPI_lld_param_check(bool result)
{
    int32_t status = QSPI_SYSTEM_FAILURE;
    if(result)
    {
        status = QSPI_SYSTEM_SUCCESS;
    }
    return status;
}
