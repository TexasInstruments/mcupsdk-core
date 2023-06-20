/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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
 *  \file ospi_nor_flash.c
 *
 *  \brief File containing generic NOR xSPI flash driver APIs.
 *
 */

#include <drivers/ospi.h>
#include <drivers/ospi/v0/cslr_ospi.h>
#include <drivers/hw_include/cslr.h>
#include <kernel/dpl/ClockP.h>

/* Some common NOR XSPI flash commands */
#define OSPI_NOR_CMD_RDID           (0x9FU)
#define OSPI_NOR_CMD_RSTEN          (0x66U)
#define OSPI_NOR_CMD_RST            (0x99U)
#define OSPI_NOR_CMD_WREN           (0x06U)
#define OSPI_NOR_CMD_RDSR           (0x05U)
#define OSPI_NOR_CMD_RDSFDP         (0x5AU)

#define OSPI_NOR_SR_WIP             (1U << 0U)

#define OSPI_NOR_SFDP_DC            (8U)

#define OSPI_NOR_WRR_WRITE_TIMEOUT  (600U*1000U)

static uint8_t gNorRdCmd = 0x03;
static uint8_t gNorWrCmd = 0x02;
static uint8_t gNorErCmd = 0xD8;

int32_t OSPI_norFlashCmdRead(OSPI_Handle handle, uint8_t cmd, uint8_t *rxBuf, uint32_t rxLen)
{
    int32_t status = SystemP_SUCCESS;

    OSPI_ReadCmdParams rdParams;
    OSPI_ReadCmdParams_init(&rdParams);
    rdParams.cmd       = cmd;
    rdParams.rxDataBuf = rxBuf;
    rdParams.rxDataLen = rxLen;

    status += OSPI_readCmd(handle, &rdParams);

    return status;
}

int32_t OSPI_norFlashCmdWrite(OSPI_Handle handle, uint8_t cmd, uint32_t cmdAddr, uint8_t *txBuf, uint32_t txLen)
{
    int32_t status = SystemP_SUCCESS;

    OSPI_WriteCmdParams wrParams;
    OSPI_WriteCmdParams_init(&wrParams);
    wrParams.cmd        = cmd;
    wrParams.cmdAddr    = cmdAddr;
    wrParams.txDataBuf  = txBuf;
    wrParams.txDataLen  = txLen;
    status += OSPI_writeCmd(handle, &wrParams);

    return status;
}

void OSPI_norFlashSetCmds(uint8_t rdCmd, uint8_t wrCmd, uint8_t eraseCmd)
{
    if(rdCmd !=0 && rdCmd != 0xFF)
    {
        gNorRdCmd = rdCmd;
    }
    if(wrCmd !=0 && wrCmd != 0xFF)
    {
        gNorWrCmd = wrCmd;
    }
    if(eraseCmd !=0 && eraseCmd != 0xFF)
    {
        gNorErCmd = eraseCmd;
    }
}

int32_t OSPI_norFlashInit1s1s1s(OSPI_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
    const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);
    uint32_t regVal = 0U;
    uint8_t cmd;

    /* Reset the Flash */
    cmd = OSPI_NOR_CMD_RSTEN;
    OSPI_norFlashCmdWrite(handle, cmd, 0xFFFFFFFF, NULL, 0);

    cmd = OSPI_NOR_CMD_RST;
    OSPI_norFlashCmdWrite(handle, cmd, 0xFFFFFFFF, NULL, 0);

    /* Wait for a while */
    uint32_t waitMicro = 500U * 1000U;
    ClockP_usleep(waitMicro);

    /* Set lowest bus clock */
    CSL_REG32_FINS(&pReg->CONFIG_REG,
                   OSPI_FLASH_CFG_CONFIG_REG_MSTR_BAUD_DIV_FLD,
                   0xF);

    /* SDR will be enabled in flash by default, set OSPI controller to 1S-1S-1S mode */
    uint32_t xferLines = 0;

    /* Set number of address bytes as 3 to support legacy flash devices also
        00 = 1 addr byte
        01 = 2 addr byte
        10 = 3 addr byte
        11 = 4 addr byte
    */
    CSL_REG32_FINS(&pReg->DEV_SIZE_CONFIG_REG, OSPI_FLASH_CFG_DEV_SIZE_CONFIG_REG_NUM_ADDR_BYTES_FLD, 0x02);

    /* Set RD and WR Config register */
    regVal = CSL_REG32_RD(&pReg->DEV_INSTR_RD_CONFIG_REG);
    /* Configure the Device Read Instruction Configuration Register */
    regVal &= ~(CSL_OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_INSTR_TYPE_FLD_MASK              | \
                CSL_OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_RD_OPCODE_NON_XIP_FLD_MASK       | \
                CSL_OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_ADDR_XFER_TYPE_STD_MODE_FLD_MASK | \
                CSL_OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_DATA_XFER_TYPE_EXT_MODE_FLD_MASK | \
                CSL_OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_DDR_EN_FLD_MASK                  | \
                CSL_OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_DUMMY_RD_CLK_CYCLES_FLD_MASK);
    regVal |= ((uint32_t)gNorRdCmd << CSL_OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_RD_OPCODE_NON_XIP_FLD_SHIFT)        | \
              (xferLines << CSL_OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_DATA_XFER_TYPE_EXT_MODE_FLD_SHIFT) | \
              (xferLines << CSL_OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_ADDR_XFER_TYPE_STD_MODE_FLD_SHIFT) | \
              (xferLines << CSL_OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_INSTR_TYPE_FLD_SHIFT)          | \
              (0U << CSL_OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_DUMMY_RD_CLK_CYCLES_FLD_SHIFT);
    CSL_REG32_WR(&pReg->DEV_INSTR_RD_CONFIG_REG, regVal);

    regVal = CSL_REG32_RD(&pReg->DEV_INSTR_WR_CONFIG_REG);

    /* Configure the Device Write Instruction Configuration Register */
    regVal &= ~(CSL_OSPI_FLASH_CFG_DEV_INSTR_WR_CONFIG_REG_WR_OPCODE_FLD_MASK               | \
                CSL_OSPI_FLASH_CFG_DEV_INSTR_WR_CONFIG_REG_DATA_XFER_TYPE_EXT_MODE_FLD_MASK | \
                CSL_OSPI_FLASH_CFG_DEV_INSTR_WR_CONFIG_REG_ADDR_XFER_TYPE_STD_MODE_FLD_MASK | \
                CSL_OSPI_FLASH_CFG_DEV_INSTR_WR_CONFIG_REG_DUMMY_WR_CLK_CYCLES_FLD_MASK);
    regVal |= ((uint32_t)gNorWrCmd << CSL_OSPI_FLASH_CFG_DEV_INSTR_WR_CONFIG_REG_WR_OPCODE_FLD_SHIFT) | \
              (xferLines << CSL_OSPI_FLASH_CFG_DEV_INSTR_WR_CONFIG_REG_ADDR_XFER_TYPE_STD_MODE_FLD_SHIFT) | \
              (xferLines << CSL_OSPI_FLASH_CFG_DEV_INSTR_WR_CONFIG_REG_DATA_XFER_TYPE_EXT_MODE_FLD_SHIFT);
    CSL_REG32_WR(&pReg->DEV_INSTR_WR_CONFIG_REG, regVal);

    /* Set read capture delay */
    status += OSPI_setRdDataCaptureDelay(handle, 0);

    return status;
}

int32_t OSPI_norFlashWaitReady(OSPI_Handle handle, uint32_t timeOut)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t readStatus = 0U;
    uint8_t cmd;

    cmd = OSPI_NOR_CMD_RDSR;

    status = OSPI_norFlashCmdRead(handle, cmd, &readStatus, 1);

    while((status != SystemP_SUCCESS) || timeOut > 0)
    {
        status = OSPI_norFlashCmdRead(handle, cmd, &readStatus, 1);

        if((status == SystemP_SUCCESS) && ((readStatus & OSPI_NOR_SR_WIP) == 0))
        {
            break;
        }

        timeOut--;
    }

    if((readStatus & OSPI_NOR_SR_WIP)==0)
    {
        status = SystemP_SUCCESS;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t OSPI_norFlashReadId(OSPI_Handle handle, uint32_t *manufacturerId, uint32_t *deviceId)
{
    int32_t status = SystemP_SUCCESS;

    uint8_t cmd = OSPI_NOR_CMD_RDID;
    uint8_t idCode[3] = { 0 };

    status += OSPI_norFlashCmdRead(handle, cmd, idCode, 3);

    if(status == SystemP_SUCCESS)
    {
        *manufacturerId = (uint32_t)idCode[0];
        *deviceId = ((uint32_t)idCode[1] << 8) | ((uint32_t)idCode[2]);
    }

    return status;
}

int32_t OSPI_norFlashWrite(OSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len)
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
        OSPI_Transaction transaction;

        pageSize = 256;
        chunkLen = pageSize;

        for (actual = 0; actual < len; actual += chunkLen)
        {
            status = OSPI_norFlashCmdWrite(handle, OSPI_NOR_CMD_WREN, 0xFFFFFFFF, NULL, 0);

            if(status == SystemP_SUCCESS)
            {
                status = OSPI_norFlashWaitReady(handle, OSPI_NOR_WRR_WRITE_TIMEOUT);
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

                OSPI_Transaction_init(&transaction);
                transaction.addrOffset = offset;
                transaction.buf = (void *)(buf + actual);
                transaction.count = chunkLen;
                status = OSPI_writeIndirect(handle, &transaction);
            }

            if(status == SystemP_SUCCESS)
            {
                status = OSPI_norFlashWaitReady(handle, OSPI_NOR_WRR_WRITE_TIMEOUT);
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

int32_t OSPI_norFlashRead(OSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len)
{
    int32_t status = SystemP_SUCCESS;

    OSPI_Transaction transaction;

    OSPI_Transaction_init(&transaction);
    transaction.addrOffset = offset;
    transaction.buf = (void *)buf;
    transaction.count = len;
    status = OSPI_readDirect(handle, &transaction);

    return status;
}

int32_t OSPI_norFlashReadSfdp(OSPI_Handle handle, uint32_t offset, uint8_t *buf, uint32_t len)
{
    int32_t status = SystemP_SUCCESS;
    const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
    const CSL_ospi_flash_cfgRegs *pReg = (const CSL_ospi_flash_cfgRegs *)(attrs->baseAddr);

    /* Save the current command and dummy cycles */
    uint8_t cmd = CSL_REG32_FEXT(&pReg->DEV_INSTR_RD_CONFIG_REG, OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_RD_OPCODE_NON_XIP_FLD);
    uint8_t dummyClks = CSL_REG32_FEXT(&pReg->DEV_INSTR_RD_CONFIG_REG, OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_DUMMY_RD_CLK_CYCLES_FLD);

    /* Set read command and dummyClks for reading sfdp table */
    CSL_REG32_FINS(&pReg->DEV_INSTR_RD_CONFIG_REG, OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_RD_OPCODE_NON_XIP_FLD, OSPI_NOR_CMD_RDSFDP);
    CSL_REG32_FINS(&pReg->DEV_INSTR_RD_CONFIG_REG, OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_DUMMY_RD_CLK_CYCLES_FLD, OSPI_NOR_SFDP_DC);

    /* Perform SFDP read */
    status = OSPI_norFlashRead(handle, offset, buf, len);

    /* Set back to old read command and dummy clocks */
    CSL_REG32_FINS(&pReg->DEV_INSTR_RD_CONFIG_REG, OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_RD_OPCODE_NON_XIP_FLD, cmd);
    CSL_REG32_FINS(&pReg->DEV_INSTR_RD_CONFIG_REG, OSPI_FLASH_CFG_DEV_INSTR_RD_CONFIG_REG_DUMMY_RD_CLK_CYCLES_FLD, dummyClks);

    return status;
}

int32_t OSPI_norFlashErase(OSPI_Handle handle, uint32_t address)
{
    int32_t status = SystemP_SUCCESS;

    status = OSPI_norFlashWaitReady(handle, OSPI_NOR_WRR_WRITE_TIMEOUT);

    if(status == SystemP_SUCCESS)
    {
        status = OSPI_norFlashCmdWrite(handle, OSPI_NOR_CMD_WREN, 0xFFFFFFFF, NULL, 0);
    }
    if(status == SystemP_SUCCESS)
    {
        status = OSPI_norFlashWaitReady(handle, OSPI_NOR_WRR_WRITE_TIMEOUT);
    }
    if(status == SystemP_SUCCESS)
    {
        status = OSPI_norFlashCmdWrite(handle, gNorErCmd, address, NULL, 0);
    }
    if(status == SystemP_SUCCESS)
    {
        status = OSPI_norFlashWaitReady(handle, OSPI_NOR_WRR_WRITE_TIMEOUT);
    }

    return status;
}

