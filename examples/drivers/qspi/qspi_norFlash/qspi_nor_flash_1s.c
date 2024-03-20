/*
 *  Copyright (C) 2021-2024 Texas Instruments Incorporated
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
 *  \file qspi_nor_flash.c
 *
 *  \brief File containing generic NOR QSPI flash driver APIs.
 *
 */

#include "qspi_nor_flash_1s.h"

/* Some common NOR XSPI flash commands */
#define QSPI_NOR_CMD_RDID           (0x9FU)
#define QSPI_NOR_CMD_SINGLE_READ    (0x03U)
#define QSPI_NOR_CMD_QUAD_READ      (0x6BU)
#define QSPI_NOR_PAGE_PROG          (0x02U)
#define QSPI_NOR_CMD_RSTEN          (0x66U)
#define QSPI_NOR_CMD_RST            (0x99U)
#define QSPI_NOR_CMD_WREN           (0x06U)
#define QSPI_NOR_CMD_WRSR           (0x01U)
#define QSPI_NOR_CMD_RDSR1          (0x05U)
#define QSPI_NOR_CMD_RDSR2          (0x35U)
#define QSPI_NOR_CMD_SECTOR_ERASE   (0x20U)
#define QSPI_NOR_CMD_BLOCK_ERASE    (0xD8U)
#define QSPI_NOR_CMD_RDSFDP         (0x5AU)

#define QSPI_NOR_SFDP_DC            (8U)

#define QSPI_NOR_SR_WIP             (1U << 0U)
#define QSPI_NOR_SR_WEL             (1U << 1U)

#define QSPI_NOR_WRR_WRITE_TIMEOUT  (1200U * 1000U)
#define QSPI_NOR_PAGE_PROG_TIMEOUT  (400U)
#define QSPI_Timeout_10ms           (10000)

int32_t QSPI_norFlashCmdRead(QSPI_Handle handle, uint8_t cmd, uint32_t cmdAddr, uint8_t *rxBuf, uint32_t rxLen)
{
    int32_t status = SystemP_SUCCESS;

    QSPI_ReadCmdParams rdParams;
    QSPI_readCmdParams_init(&rdParams);
    rdParams.cmd       = cmd;
    rdParams.cmdAddr   = cmdAddr;
    rdParams.rxDataBuf = rxBuf;
    rdParams.rxDataLen = rxLen;

    status += QSPI_readCmd(handle, &rdParams);

    return status;
}

int32_t QSPI_norFlashCmdWrite(QSPI_Handle handle, uint8_t cmd, uint32_t cmdAddr, uint8_t *txBuf, uint32_t txLen)
{
    int32_t status = SystemP_SUCCESS;

    QSPI_WriteCmdParams wrParams;
    QSPI_writeCmdParams_init(&wrParams);
    wrParams.cmd        = cmd;
    wrParams.cmdAddr    = cmdAddr;
    wrParams.txDataBuf  = txBuf;
    wrParams.txDataLen  = txLen;
    status += QSPI_writeCmd(handle, &wrParams);

    return status;
}

int32_t QSPI_norFlashWriteEnableLatched(QSPI_Handle handle, uint32_t timeOut)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t readStatus = 0;
    uint8_t cmd;

    cmd = QSPI_NOR_CMD_RDSR1;

    status = QSPI_norFlashCmdRead(handle, cmd, QSPI_CMD_INVALID_ADDR, &readStatus, 1);

    while((status == SystemP_SUCCESS) && timeOut > 0)
    {
        status = QSPI_norFlashCmdRead(handle, cmd, QSPI_CMD_INVALID_ADDR, &readStatus, 1);

        if((status == SystemP_SUCCESS) && ((readStatus & QSPI_NOR_SR_WEL) != 0))
        {
            break;
        }

        timeOut--;
    }

    if((readStatus & QSPI_NOR_SR_WEL) != 0)
    {
        status = SystemP_SUCCESS;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t QSPI_norFlashWaitReady(QSPI_Handle handle, uint32_t timeOut)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t readStatus = 0;
    uint8_t cmd;

    cmd = QSPI_NOR_CMD_RDSR1;

    status = QSPI_norFlashCmdRead(handle, cmd, QSPI_CMD_INVALID_ADDR, &readStatus, 1);

    while((status == SystemP_SUCCESS) && timeOut > 0)
    {
        status = QSPI_norFlashCmdRead(handle, cmd, QSPI_CMD_INVALID_ADDR, &readStatus, 1);

        if((status == SystemP_SUCCESS) && ((readStatus & QSPI_NOR_SR_WIP) == 0))
        {
            break;
        }

        timeOut--;
    }

    if((readStatus & QSPI_NOR_SR_WIP) == 0)
    {
        status = SystemP_SUCCESS;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t QSPI_norFlashInit(QSPI_Handle handle)
{
    uint8_t cmd;

    /* Reset the Flash */
    cmd = QSPI_NOR_CMD_RSTEN;
    (void) QSPI_norFlashCmdWrite(handle, cmd, QSPI_CMD_INVALID_ADDR, NULL, 0U);

    cmd = QSPI_NOR_CMD_RST;
    (void) QSPI_norFlashCmdWrite(handle, cmd, QSPI_CMD_INVALID_ADDR, NULL, 0U);

    (void) QSPI_norFlashWaitReady(handle, QSPI_NOR_WRR_WRITE_TIMEOUT);

    (void) QSPI_setWriteCmd(handle, QSPI_NOR_PAGE_PROG);

    (void) QSPI_setReadCmd(handle, QSPI_NOR_CMD_SINGLE_READ);

    (void) QSPI_setAddressByteCount(handle, 3U);

    (void) QSPI_setDummyBitCount(handle, 0);

    return 0;
}

int32_t QSPI_norFlashWrite(QSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len)
{
    int32_t status = SystemP_SUCCESS;

    /* Check offset alignment */
    if(0 != (offset % 256))
    {
        status = SystemP_FAILURE;
    }
    if(status == SystemP_SUCCESS)
    {
        uint32_t pageSize, chunkLen, actual;
        uint8_t cmdWren = QSPI_NOR_CMD_WREN;
        QSPI_Transaction transaction;

        pageSize = 256;
        chunkLen = pageSize;

        for (actual = 0; actual < len; actual += chunkLen)
        {
            status = QSPI_norFlashCmdWrite(handle, cmdWren, QSPI_CMD_INVALID_ADDR, NULL, 0);

            if(status == SystemP_SUCCESS)
            {
                status = QSPI_norFlashWriteEnableLatched(handle, QSPI_NOR_WRR_WRITE_TIMEOUT);
            }
            if(status == SystemP_SUCCESS)
            {
                /* Send Page Program command */
                if((len - actual) < (pageSize))
                {
                    chunkLen = (len - actual);
                }
                else
                {
                    chunkLen = pageSize;
                }

                QSPI_transaction_init(&transaction);
                transaction.addrOffset = offset;
                transaction.buf = (void *)(buf + actual);
                transaction.count = chunkLen;
                status = QSPI_writeConfigMode(handle, &transaction);
            }

            if(status == SystemP_SUCCESS)
            {
                status = QSPI_norFlashWaitReady(handle, QSPI_NOR_PAGE_PROG_TIMEOUT);
            }

            if(status == SystemP_SUCCESS)
            {
                offset += chunkLen;
            }
            else
            {
                break;
            }
        }
    }

    return status;
}

int32_t QSPI_norFlashRead(QSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len)
{
    int32_t status = SystemP_FAILURE;

    QSPI_Transaction transaction;

    QSPI_transaction_init(&transaction);
    transaction.addrOffset = offset;
    transaction.buf = (void *)buf;
    transaction.count = len;
    status = QSPI_readMemMapMode(handle, &transaction);

    return status;
}

int32_t QSPI_norFlashErase(QSPI_Handle handle, uint32_t address)
{
    int32_t status = SystemP_SUCCESS;

    uint8_t cmdWren = QSPI_NOR_CMD_WREN;
    uint8_t cmd;

    cmd    = QSPI_NOR_CMD_BLOCK_ERASE;

    status = QSPI_norFlashWaitReady(handle, QSPI_NOR_WRR_WRITE_TIMEOUT);
    if(status == SystemP_SUCCESS)
    {
        status = QSPI_norFlashCmdWrite(handle, cmdWren, QSPI_CMD_INVALID_ADDR, NULL, 0);
    }
    if(status == SystemP_SUCCESS)
    {
        status = QSPI_norFlashWriteEnableLatched(handle, QSPI_NOR_WRR_WRITE_TIMEOUT);
    }
    if(status == SystemP_SUCCESS)
    {
        status = QSPI_norFlashCmdWrite(handle, cmd, address, NULL, 0);
    }
    if(status == SystemP_SUCCESS)
    {
        status = QSPI_norFlashWaitReady(handle, QSPI_NOR_WRR_WRITE_TIMEOUT);
    }

    return status;
}

int32_t QSPI_norFlashReadId(QSPI_Handle handle, uint32_t *manufacturerId, uint32_t *deviceId)
{
    int32_t status = SystemP_SUCCESS;

    uint8_t cmd = QSPI_NOR_CMD_RDID;
    uint8_t idCode[3] = { 0 };

    status += QSPI_norFlashCmdRead(handle, cmd, QSPI_CMD_INVALID_ADDR, idCode, 3);

    if(status == SystemP_SUCCESS)
    {
        *manufacturerId = (uint32_t)idCode[0];
        *deviceId = ((uint32_t)idCode[1] << 8) | ((uint32_t)idCode[2]);
    }

    return status;
}

int32_t QSPI_norFlashReadSfdp(QSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len)
{
    int32_t status = SystemP_SUCCESS;

    if(handle != NULL)
    {
        /* Set read command and dummyClks for reading sfdp table */
        (void)QSPI_setReadCmd(handle,QSPI_NOR_CMD_RDSFDP);
        (void)QSPI_setDummyBitCount(handle,QSPI_NOR_SFDP_DC);

        /* Perform SFDP read */
        status = QSPI_norFlashRead(handle, offset, buf, len);

        (void) QSPI_setWriteCmd(handle, QSPI_NOR_PAGE_PROG);
        (void) QSPI_setReadCmd(handle, QSPI_NOR_CMD_SINGLE_READ);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t QSPI_norFlashWriteIntr(QSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len)
{
    int32_t status = SystemP_SUCCESS;

    /* Check offset alignment */
    if(0U != (offset % 256U))
    {
        status = SystemP_FAILURE;
    }
    if(status == SystemP_SUCCESS)
    {

        uint8_t cmdWren = QSPI_NOR_CMD_WREN;
        uint8_t cmrProg = QSPI_NOR_PAGE_PROG;
        QSPI_WriteCmdParams wrParams = {0};

        status = QSPI_norFlashCmdWrite(handle, cmdWren, QSPI_CMD_INVALID_ADDR, NULL, 0U);

        if(status == SystemP_SUCCESS)
        {
            status = QSPI_norFlashWriteEnableLatched(handle, QSPI_NOR_WRR_WRITE_TIMEOUT);
        }
        if(status == SystemP_SUCCESS)
        {
            /* Send Page Program command */
            wrParams.cmd = cmrProg;
            wrParams.cmdAddr = offset;
            wrParams.numAddrBytes = 3U;
            wrParams.txDataBuf = (void *)(buf);
            wrParams.txDataLen = len;
            status = QSPI_writeConfigModeIntr(handle, &wrParams);
        }
    }

    return status;
}

int32_t QSPI_norFlashReadIntr(QSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len)
{
    int32_t status = SystemP_SUCCESS;
    QSPI_ReadCmdParams rdParams = {0};

    /* Send Read Program command */
    rdParams.cmd = QSPI_NOR_CMD_SINGLE_READ;
    rdParams.cmdAddr = offset;
    rdParams.numAddrBytes = 3U;
    rdParams.rxDataBuf = (void *)(buf);
    rdParams.rxDataLen = len;
    status = QSPI_readConfigModeIntr(handle,&rdParams);
    return status;
}