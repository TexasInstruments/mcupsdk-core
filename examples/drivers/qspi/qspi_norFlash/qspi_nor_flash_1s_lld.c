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
 *  \file qspi_nor_flash.c
 *
 *  \brief File containing generic NOR QSPI flash driver APIs.
 *
 */

#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/DebugP.h>
#include "qspi_nor_flash_1s_lld.h"

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
#define QSPI_NOR_CMD_BLOCK_ERASE    (0xD8U)  // It is sector erase
#define QSPI_NOR_CMD_RDSFDP         (0x5AU)

#define QSPI_NOR_SFDP_DC            (8U)

#define QSPI_NOR_SR_WIP             (1U << 0U)
#define QSPI_NOR_SR_WEL             (1U << 1U)

#define QSPI_NOR_WRR_WRITE_TIMEOUT  (1200U * 1000U)
#define QSPI_NOR_PAGE_PROG_TIMEOUT  (400U)

int32_t QSPI_norFlashWriteEnableLatched(QSPILLD_Handle handle, uint32_t timeOut)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t readStatus = 0;
    QSPILLD_WriteCmdParams msg = {0};
    uint32_t timeout = timeOut;

    msg.cmd = QSPI_NOR_CMD_RDSR1;
    msg.cmdAddr = QSPI_LLD_CMD_INVALID_ADDR;
    msg.dataLen = 1U;
    msg.numAddrBytes = 3U;
    msg.dataBuf = &readStatus;

    status = QSPI_lld_readCmd(handle,&msg);
    while((status == SystemP_SUCCESS) && timeout > 0U)
    {
        status = QSPI_lld_readCmd(handle,&msg);

        if((status == SystemP_SUCCESS) && ((readStatus & QSPI_NOR_SR_WEL) != 0))
        {
            break;
        }

        timeout--;
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

int32_t QSPI_norFlashWaitReady(QSPILLD_Handle handle, uint32_t timeOut)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t readStatus = 0;
    QSPILLD_WriteCmdParams msg = {0};

    msg.cmd = QSPI_NOR_CMD_RDSR1;
    msg.cmdAddr = QSPI_LLD_CMD_INVALID_ADDR;
    msg.dataLen = 1U;
    msg.numAddrBytes = 3U;
    msg.dataBuf = &readStatus;

    status = QSPI_lld_readCmd(handle, &msg);

    while((status == SystemP_SUCCESS) && timeOut > 0)
    {
        status = QSPI_lld_readCmd(handle, &msg);

        if((status == SystemP_SUCCESS) && ((readStatus & QSPI_NOR_SR_WIP) == 0))
        {
            break;
        }

        timeOut--;
        if(timeOut == 1)
        {
            DebugP_log("TimeoutReached\r\n");
        }
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

int32_t QSPI_norFlashInit(QSPILLD_Handle handle)
{
    QSPILLD_WriteCmdParams msg = {0};

    /* Reset the Flash */
    msg.cmd = QSPI_NOR_CMD_RSTEN;
    msg.cmdAddr = QSPI_LLD_CMD_INVALID_ADDR;
    msg.dataLen = 0U;
    msg.numAddrBytes = 3U;
    msg.dataBuf = NULL;
    (void) QSPI_lld_writeCmd(handle, &msg);

    msg.cmd = QSPI_NOR_CMD_RST;
    (void) QSPI_lld_writeCmd(handle, &msg);
    (void) QSPI_norFlashWaitReady(handle, QSPI_NOR_WRR_WRITE_TIMEOUT);
    (void) QSPI_lld_setWriteCmd(handle, QSPI_NOR_PAGE_PROG);
    (void) QSPI_lld_setReadCmd(handle, QSPI_NOR_CMD_SINGLE_READ);
    (void) QSPI_lld_setAddressByteCount(handle, 3);
    (void) QSPI_lld_setDummyBitCount(handle, 0);

    return 0;
}

int32_t QSPI_norFlashWrite(QSPILLD_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len)
{
    int32_t status = SystemP_SUCCESS;
    QSPILLD_WriteCmdParams msg = {0};
    uint32_t pageSize = 256U;
    uint32_t addrOffset = offset;

    /* Check offset alignment */
    if(0U != (addrOffset % pageSize))
    {
        status = SystemP_FAILURE;
    }
    if(status == SystemP_SUCCESS)
    {
        uint32_t chunkLen, actual;
        uint8_t cmdWren = QSPI_NOR_CMD_WREN;
        uint8_t cmrProg = QSPI_NOR_PAGE_PROG;

        chunkLen = pageSize;

        for (actual = 0; actual < len; actual += chunkLen)
        {
            msg.cmd = cmdWren;
            msg.cmdAddr = QSPI_LLD_CMD_INVALID_ADDR;
            msg.dataLen = 0U;
            msg.numAddrBytes = 3U;
            msg.dataBuf = NULL;
            status = QSPI_lld_writeCmd(handle,&msg);

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
                msg.cmd = cmrProg;
                msg.cmdAddr = addrOffset;
                msg.dataLen = chunkLen;
                msg.numAddrBytes = 3U;
                msg.dataBuf = (void *)(buf + actual);
                status = QSPI_lld_writeCmd(handle, &msg);
            }
            else
            {
                status = SystemP_FAILURE;
            }

            if(status == SystemP_SUCCESS)
            {
                status = QSPI_norFlashWaitReady(handle, QSPI_NOR_PAGE_PROG_TIMEOUT);
            }

            if(status == SystemP_SUCCESS)
            {
                addrOffset += chunkLen;
            }
            else
            {
                break;
            }
        }
    }

    return status;
}

int32_t QSPI_norFlashWriteIntr(QSPILLD_Handle handle,QSPILLD_WriteCmdParams *wrMsg)
{
    int32_t status = SystemP_SUCCESS;
    QSPILLD_WriteCmdParams writeEnable = {0};
    QSPILLD_WriteCmdParams *msg = wrMsg;
    /* Check offset alignment */
    if(0 != (msg->cmdAddr % 256U))
    {
        status = SystemP_FAILURE;
    }
    if(status == SystemP_SUCCESS)
    {
        writeEnable.cmd = QSPI_NOR_CMD_WREN;
        writeEnable.cmdAddr = QSPI_LLD_CMD_INVALID_ADDR;
        writeEnable.dataLen = 0U;
        writeEnable.numAddrBytes = 3U;
        writeEnable.dataBuf = NULL;

        status = QSPI_lld_writeCmd(handle,&writeEnable);

        if(status == SystemP_SUCCESS)
        {
            status = QSPI_norFlashWriteEnableLatched(handle, QSPI_NOR_WRR_WRITE_TIMEOUT);
        }
        if(status == SystemP_SUCCESS)
        {
            msg->cmd = QSPI_NOR_PAGE_PROG;
            msg->numAddrBytes = 3U;
            status = QSPI_lld_writeCmdIntr(handle,msg);
        }
    }

    return status;
}

int32_t QSPI_norFlashRead(QSPILLD_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len)
{
    int32_t status = SystemP_FAILURE;
    uint32_t dataLen = len;
    uint32_t itr = 0U;
    uint32_t pageSize = 256U;

    for(itr = 0U; itr < dataLen-1; itr = itr+256)
    {
        status = QSPI_lld_read(handle, pageSize, (void *)(buf+itr), (offset+itr), SystemP_WAIT_FOREVER);
        
        if(status == SystemP_FAILURE)
        {
            break;
        }
    }

    return status;
}

int32_t QSPI_norFlashReadDma(QSPILLD_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len)
{
    int32_t status = SystemP_FAILURE;

    status = QSPI_lld_readDma(handle, len, (void *)buf, offset, SystemP_WAIT_FOREVER);

    return status;
}

int32_t QSPI_norFlashErase(QSPILLD_Handle handle, uint32_t address)
{
    int32_t status = SystemP_SUCCESS;
    QSPILLD_WriteCmdParams msg = {0};

    /* Reset the Flash */
    msg.cmd = QSPI_NOR_CMD_WREN;
    msg.cmdAddr = QSPI_LLD_CMD_INVALID_ADDR;
    msg.dataLen = 0U;
    msg.numAddrBytes = 3U;
    msg.dataBuf = NULL;

    status = QSPI_norFlashWaitReady(handle, QSPI_NOR_WRR_WRITE_TIMEOUT);
    if(status == SystemP_SUCCESS)
    {
        status = QSPI_lld_writeCmd(handle, &msg);
    }
    if(status == SystemP_SUCCESS)
    {
        status = QSPI_norFlashWriteEnableLatched(handle, QSPI_NOR_WRR_WRITE_TIMEOUT);
    }
    if(status == SystemP_SUCCESS)
    {
        msg.cmd = QSPI_NOR_CMD_BLOCK_ERASE;
        msg.cmdAddr = address;
        status = QSPI_lld_writeCmd(handle, &msg);
    }
    if(status == SystemP_SUCCESS)
    {
        status = QSPI_norFlashWaitReady(handle, QSPI_NOR_WRR_WRITE_TIMEOUT);
    }

    return status;
}

int32_t QSPI_norFlashReadId(QSPILLD_Handle handle, uint32_t *manufacturerId, uint32_t *deviceId)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t idCode[3] = { 0 };
    QSPILLD_WriteCmdParams msg = {0};

    msg.cmd = QSPI_NOR_CMD_RDID;
    msg.cmdAddr = QSPI_LLD_CMD_INVALID_ADDR;
    msg.dataLen = 3U;
    msg.numAddrBytes = 3U;
    msg.dataBuf = &idCode;

    status += QSPI_lld_readCmd(handle, &msg);

    if(status == SystemP_SUCCESS)
    {
        *manufacturerId = (uint32_t)idCode[0];
        *deviceId = ((uint32_t)idCode[1] << 8) | ((uint32_t)idCode[2]);
    }

    return status;
}

int32_t QSPI_norFlashReadSfdp(QSPILLD_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t cmd;
    uint32_t dummyClks;

    if(handle != NULL)
    {
        /* Save the current command and dummy cycles */
        cmd = handle->readCmd;
        dummyClks = handle->numDummyBits;

        /* Set read command and dummyClks for reading sfdp table */
        handle->readCmd = QSPI_NOR_CMD_RDSFDP;
        handle->numDummyBits = QSPI_NOR_SFDP_DC;

        /* Perform SFDP read */
        status = QSPI_norFlashRead(handle, offset, buf, len);

        /* Set back to old read command and dummy clocks */
        handle->readCmd = cmd;
        handle->numDummyBits = dummyClks;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}
