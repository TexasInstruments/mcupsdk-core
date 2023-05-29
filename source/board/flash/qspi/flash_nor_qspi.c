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

#include <board/flash.h>
#include <board/flash/qspi/flash_nor_qspi.h>

#define FLASH_QSPI_JEDEC_ID_SIZE_MAX (8U)

static int32_t Flash_norQspiErase(Flash_Config *config, uint32_t blkNum);
static int32_t Flash_norQspiEraseSector(Flash_Config *config, uint32_t sectNum);
static int32_t Flash_norQspiRead(Flash_Config *config, uint32_t offset, uint8_t *buf, uint32_t len);
static int32_t Flash_norQspiWrite(Flash_Config *config, uint32_t offset, uint8_t *buf, uint32_t len);
static int32_t Flash_norQspiOpen(Flash_Config *config, Flash_Params *params);
static void Flash_norQspiClose(Flash_Config *config);
static int32_t Flash_norQspiReset(Flash_Config *config);

uint32_t gFlashToSpiProtocolMap[] =
{
    [FLASH_CFG_PROTO_1S_1S_1S] = QSPI_RX_LINES_SINGLE,
    [FLASH_CFG_PROTO_1S_1S_2S] = QSPI_RX_LINES_DUAL,
    [FLASH_CFG_PROTO_1S_1S_4S] = QSPI_RX_LINES_QUAD,
};

Flash_Fxns gFlashNorQspiFxns = {

    .openFxn = Flash_norQspiOpen,
    .closeFxn = Flash_norQspiClose,
    .readFxn = Flash_norQspiRead,
    .writeFxn = Flash_norQspiWrite,
    .eraseFxn = Flash_norQspiErase,
    .eraseSectorFxn = Flash_norQspiEraseSector,
    .resetFxn = Flash_norQspiReset,
};

static int32_t Flash_norQspiCmdWrite(Flash_Config *config, uint8_t cmd, uint32_t cmdAddr,
                                     uint8_t numAddrBytes, uint8_t *txBuf, uint32_t txLen)
{
    int32_t status = SystemP_SUCCESS;
    Flash_NorQspiObject *obj = (Flash_NorQspiObject *)(config->object);
    QSPI_WriteCmdParams wrParams;

    QSPI_writeCmdParams_init(&wrParams);
    wrParams.cmd          = cmd;
    wrParams.cmdAddr      = cmdAddr;
    wrParams.numAddrBytes = numAddrBytes;
    wrParams.txDataBuf    = txBuf;
    wrParams.txDataLen    = txLen;

    status = QSPI_writeCmd(obj->qspiHandle, &wrParams);

    return status;
}

static int32_t Flash_norQspiCmdRead(Flash_Config *config, uint8_t cmd, uint32_t cmdAddr,
                            uint8_t numAddrBytes, uint8_t *rxBuf, uint32_t rxLen)
{
    int32_t status = SystemP_SUCCESS;
    Flash_NorQspiObject *obj = (Flash_NorQspiObject *)(config->object);
    QSPI_ReadCmdParams  rdParams;

    QSPI_readCmdParams_init(&rdParams);
    rdParams.cmd           = cmd;
    rdParams.cmdAddr       = cmdAddr;
    rdParams.numAddrBytes  = numAddrBytes;
    rdParams.rxDataBuf     = rxBuf;
    rdParams.rxDataLen     = rxLen;

    status = QSPI_readCmd(obj->qspiHandle, &rdParams);

    return status;
}

static int32_t Flash_norQspiWEL(Flash_Config *config, uint32_t timeOut)
{
    int32_t status = SystemP_SUCCESS;
    Flash_DevConfig *devCfg = config->devConfig;
    uint8_t readStatus = 0U;

    while((timeOut != 0) && (status == SystemP_SUCCESS))
    {
        status = Flash_norQspiCmdRead(config, devCfg->cmdRdsr, QSPI_CMD_INVALID_ADDR, 0, &readStatus, 1);

        if((readStatus & devCfg->srWel) != 0)
        {
            break;
        }

        timeOut--;
    }

    if(timeOut == 0)
    {
        status = SystemP_TIMEOUT;
    }
    else
    {
        if((status == SystemP_SUCCESS) && (readStatus & devCfg->srWel) != 0)
        {
            status = SystemP_SUCCESS;
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }

    return status;
}

static int32_t Flash_norQspiWaitReady(Flash_Config *config, uint32_t timeOut)
{
    int32_t status = SystemP_SUCCESS;
    Flash_DevConfig *devCfg = config->devConfig;
    Flash_NorQspiObject *obj = (Flash_NorQspiObject *)(config->object);

    uint8_t  readStatus = 0;

    while((timeOut != 0) && (status == SystemP_SUCCESS))
    {
        status = Flash_norQspiCmdRead(config, devCfg->cmdRdsr, QSPI_CMD_INVALID_ADDR, obj->numAddrBytes, &readStatus, 1);

        if((readStatus & devCfg->srWip) == 0)
        {
            break;
        }

        timeOut--;
    }

    if(timeOut == 0)
    {
        status = SystemP_TIMEOUT;
    }
    else
    {
        if((status == SystemP_SUCCESS) && (readStatus & devCfg->srWip)==0)
        {
            status = SystemP_SUCCESS;
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }

    return status;
}

static int32_t Flash_norQspiSet4ByteAddrMode(Flash_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    Flash_DevConfig *devCfg = config->devConfig;
    Flash_NorQspiObject *obj = (Flash_NorQspiObject *)(config->object);

    if((devCfg->fourByteAddrEnSeq & (uint8_t)(1 << 0)) != 0)
    {
        /* Issue instruction 0xB7 without WREN */
        status = Flash_norQspiCmdWrite(config, 0xB7, QSPI_CMD_INVALID_ADDR, 0, NULL, 0);
    }
    if((devCfg->fourByteAddrEnSeq & (uint8_t)(1 << 1)) != 0)
    {
        /* Issue instruction 0xB7 with WREN */
        status = Flash_norQspiCmdWrite(config, devCfg->cmdWren, QSPI_CMD_INVALID_ADDR, 0, NULL, 0);
        if(status == SystemP_SUCCESS)
        {
            status = Flash_norQspiWaitReady(config, devCfg->flashBusyTimeout);
        }
        status = Flash_norQspiCmdWrite(config, 0xB7, QSPI_CMD_INVALID_ADDR, 0, NULL, 0);
        if(status == SystemP_SUCCESS)
        {
            status = Flash_norQspiWaitReady(config, devCfg->flashBusyTimeout);
        }
    }
    if((devCfg->fourByteAddrEnSeq & (uint8_t)(1 << 2)) != 0)
    {
        /* Extended Register read with instr 0xC8, write with instr 0xC5 to set
         * the MSByte of addr. To be taken care during read and write.
         */
        obj->extAddrRegSupport = TRUE;
    }
    if((devCfg->fourByteAddrEnSeq & (uint8_t)(1 << 3)) != 0)
    {
        /* Volatile bank register used to define 4 byte mode. To be taken care
         * during read and write
         */
        obj->vBankAddrRegSupport = TRUE;
    }
    if((devCfg->fourByteAddrEnSeq & (uint8_t)(1 << 4)) != 0)
    {
        /* Dedicated 4 byte address instruction set, consider 4 bytes always ON */
        obj->ded4bInstrSupport = TRUE;
    }

    return status;
}

static int32_t Flash_norQspiSetAddressBytes(Flash_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    Flash_DevConfig *devCfg = config->devConfig;
    Flash_NorQspiObject *obj = (Flash_NorQspiObject *)(config->object);

    obj->numAddrBytes = 3;

    switch (devCfg->addrByteSupport)
    {
    case 0:
        /* Only 3 byte addressing supported, nothing to do with flash. Set QSPI driver */
        QSPI_setAddressByteCount(obj->qspiHandle, 3);
        break;

    case 1:
        /* Both 3 and 4 byte addressing supported. Configure flash to switch to
         * 4 byte addressing if that's selected
         * */
        if(devCfg->enable4BAddr == TRUE)
        {
            Flash_norQspiSet4ByteAddrMode(config);
            obj->numAddrBytes = 4;
            QSPI_setAddressByteCount(obj->qspiHandle, 4);
        }
        else
        {
            QSPI_setAddressByteCount(obj->qspiHandle, 3);

        }
        break;

    case 2:
        /* Only 4 byte addressing supported. Configure flash to switch to 4 byte
         * addressing
         * */
        Flash_norQspiSet4ByteAddrMode(config);
        QSPI_setAddressByteCount(obj->qspiHandle, 4);
        obj->numAddrBytes = 4;
        break;

    default:
        QSPI_setAddressByteCount(obj->qspiHandle, 3);
        break;
    }

    return status;
}

static int32_t Flash_norQspiSetQeBit(Flash_Config *config, uint8_t qeType)
{
    int32_t status = SystemP_SUCCESS;
    Flash_DevConfig *devCfg = config->devConfig;
    uint8_t sr1 = 0, sr2 = 0, bitPos = 0;

    status = Flash_norQspiCmdWrite(config, devCfg->cmdWren, QSPI_CMD_INVALID_ADDR, 0, NULL, 0);
    if(status == SystemP_SUCCESS)
    {
        status = Flash_norQspiWaitReady(config, devCfg->flashBusyTimeout);
    }

    if(SystemP_SUCCESS == status)
    {
        switch (qeType)
        {
            case 0:
                /* No QE bit, detects 1-1-4 based on instruction */
                break;

            case 1:
            case 4:
            case 5:
                /* QE is bit 1 of SR2 */
                bitPos = (uint8_t)(1 << 1);
                status = Flash_norQspiCmdRead(config, devCfg->cmdRdsr, QSPI_CMD_INVALID_ADDR, 0, &sr1, 1);
                status += Flash_norQspiCmdRead(config, 0x35, QSPI_CMD_INVALID_ADDR, 0, &sr2, 1);

                if((sr2 & bitPos) != 0)
                {
                    /* QE bit already set */
                }
                else
                {
                    uint16_t sr = 0;
                    sr2 |= bitPos;
                    sr = ((sr2 << 8) | sr1);

                    status += Flash_norQspiCmdWrite(config, 0x01, QSPI_CMD_INVALID_ADDR, 0, (uint8_t *)&sr, 2);
                }
                break;
            case 2:
                /* QE is bit 6 of SR1 */
                sr1 = 0;
                bitPos = (uint8_t)(1 << 6);
                status = Flash_norQspiCmdRead(config, devCfg->cmdRdsr, QSPI_CMD_INVALID_ADDR, 0, &sr1, 1);

                if((sr1 & bitPos) != 0)
                {
                    /* QE is already set */
                }
                else
                {
                    sr1 |= bitPos;
                    status += Flash_norQspiCmdWrite(config, 0x01, QSPI_CMD_INVALID_ADDR, 0, &sr1, 1);
                }
                break;
            case 3:
                /* QE is bit 7 of SR2 */
                sr2 = 0;
                bitPos = (uint8_t)(1 << 7);
                status = Flash_norQspiCmdRead(config, 0x3F, QSPI_CMD_INVALID_ADDR, 0, (uint8_t *)&sr2, 1);

                if((sr2 & bitPos) != 0)
                {
                    /* QE is already set */
                }
                else
                {
                    sr2 |= bitPos;
                    status += Flash_norQspiCmdWrite(config, 0x3E, QSPI_CMD_INVALID_ADDR, 0, &sr2, 1);
                }
                break;
            case 6:
                /* QE is bit 1 of SR2, using different command */
                bitPos = (uint8_t)(1 << 1);
                status = Flash_norQspiCmdRead(config, 0x35, QSPI_CMD_INVALID_ADDR, 0, &sr2, 1);

                if((sr2 & bitPos) != 0)
                {
                    /* QE bit already set */
                }
                else
                {
                    sr2 |= bitPos;
                    status += Flash_norQspiCmdWrite(config, 0x31, QSPI_CMD_INVALID_ADDR, 0, &sr2, 1);
                }
                break;
            default:
                break;
        }
        if(status == SystemP_SUCCESS)
        {
            status = Flash_norQspiWaitReady(config, devCfg->flashBusyTimeout);
        }
    }
    return status;
}

static int32_t Flash_norQspiSetOpCodeModeDummy(Flash_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    Flash_DevConfig *devCfg = (Flash_DevConfig *)config->devConfig;
    Flash_NorQspiObject *obj = (Flash_NorQspiObject *)config->object;
    FlashCfg_ProtoEnConfig *pCfg = &(devCfg->protocolCfg);

    QSPI_setReadCmd(obj->qspiHandle, pCfg->cmdRd);
    QSPI_setWriteCmd(obj->qspiHandle, pCfg->cmdWr);

    status += QSPI_setDummyBitCount(obj->qspiHandle, pCfg->dummyClksRd);

    return status;
}

static int32_t Flash_norQspiSetProtocol(Flash_Config *config, Flash_Params *params)
{
    int32_t status = SystemP_SUCCESS;

    if(config == NULL)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        Flash_DevConfig *devCfg = (Flash_DevConfig *)config->devConfig;
        Flash_NorQspiObject *obj = (Flash_NorQspiObject *)config->object;

        /* Configure the flash according to protocol */
        uint32_t protocol = devCfg->protocolCfg.protocol;
        FlashCfg_ProtoEnConfig *pCfg = &(devCfg->protocolCfg);

        if(FLASH_CFG_PROTO_CUSTOM == protocol)
        {
            if(params->custProtoFxn != NULL)
            {
                status += params->custProtoFxn(config);
            }
            else
            {
                status = SystemP_FAILURE;
            }
        }
        else
        {
            /* OOB support available for:
            * 1S_1S_1S
            * 1S_1S_2S
            * 1S_1S_4S
            */
            switch (protocol)
            {
                case FLASH_CFG_PROTO_1S_1S_1S:
                    /* No config needed for flash driver, set commands */
                    QSPI_setReadCmd(obj->qspiHandle, pCfg->cmdRd);
                    QSPI_setWriteCmd(obj->qspiHandle, pCfg->cmdWr);
                    break;

                case FLASH_CFG_PROTO_1S_1S_2S:
                    /* No config needed for flash driver. Set commands, mode and dummy cycle if needed */
                    /* Check for mode Bits for 1-1-2 mode */
                    status += Flash_norQspiSetOpCodeModeDummy(config);
                    break;

                case FLASH_CFG_PROTO_1S_1S_4S:
                    /* Set Quad Enable Bit. Set commands, mode and dummy cycle if needed */
                    /* Set QE bit */
                    status += Flash_norQspiSetQeBit(config, pCfg->enableType);
                    status += Flash_norQspiSetOpCodeModeDummy(config);
                    break;

                default:
                    status = SystemP_FAILURE;
                    break;
            }
        }

        if((SystemP_SUCCESS == status) && (protocol <= FLASH_CFG_PROTO_1S_1S_4S))
        {
            QSPI_setRxLines(obj->qspiHandle, gFlashToSpiProtocolMap[protocol]);
        }
    }

    return status;
}

static int32_t Flash_norQspiReadId(Flash_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t manfID, devID;
    Flash_NorQspiObject *obj = (Flash_NorQspiObject *)(config->object);
    Flash_DevConfig *devCfg = (Flash_DevConfig *)config->devConfig;
    uint8_t idCode[FLASH_QSPI_JEDEC_ID_SIZE_MAX];
    uint8_t cmd = QSPI_CMD_INVALID_OPCODE;
    uint32_t cmdAddr = QSPI_CMD_INVALID_ADDR;
    uint32_t numRdIdBytes = 0U;

    DebugP_assert(FLASH_QSPI_JEDEC_ID_SIZE_MAX >= devCfg->idCfg.numBytes);

    cmd = devCfg->idCfg.cmd;
    numRdIdBytes = devCfg->idCfg.numBytes;

    status = Flash_norQspiCmdRead(config, cmd, cmdAddr, obj->numAddrBytes, idCode, numRdIdBytes);

    if(status == SystemP_SUCCESS)
    {
        manfID = (uint32_t)idCode[0];
        devID = ((uint32_t)idCode[1] << 8) | ((uint32_t)idCode[2]);
        if ((manfID == config->attrs->manufacturerId) && (devID == config->attrs->deviceId))
        {
            status = SystemP_SUCCESS;
        }
        else
        {
            status = SystemP_FAILURE;
        }
    }

    return status;
}

static int32_t Flash_norQspiOpen(Flash_Config *config, Flash_Params *params)
{
    int32_t status = SystemP_SUCCESS;
    Flash_NorQspiObject *obj = (Flash_NorQspiObject *)(config->object);
    Flash_Attrs *attrs = config->attrs;

    /* Start processing defines and fill up the attributes */
    obj->qspiHandle = QSPI_getHandle(attrs->driverInstance);

    if(obj->qspiHandle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if(SystemP_SUCCESS == status)
    {
        /* Set number of address bytes. If 4 byte addressing is supported, switch to that */
        status += Flash_norQspiSetAddressBytes(config);

        /* Set the protocol */
        status += Flash_norQspiSetProtocol(config, params);

        /* Read JEDEC ID */
        status += Flash_norQspiReadId(config);
    }

    /* Any flash specific quirks, like hybrid sector config etc. */
    if(params->quirksFxn != NULL)
    {
        params->quirksFxn(config);
    }
    else
    {
        /* Do nothing */
    }

    return status;
}

static void Flash_norQspiClose(Flash_Config *config)
{
    Flash_NorQspiObject *obj = (Flash_NorQspiObject *)(config->object);

    obj->qspiHandle = NULL;

    /* QSPI Driver will be closed outside flash */

    return;
}

static int32_t Flash_norQspiRead(Flash_Config *config, uint32_t offset, uint8_t *buf, uint32_t len)
{
    int32_t status = SystemP_SUCCESS;
    Flash_NorQspiObject *obj = (Flash_NorQspiObject *)(config->object);
    Flash_Attrs *attrs = config->attrs;

    /* Validate address input */
    if ((offset + len) > (attrs->flashSize))
    {
        status = SystemP_FAILURE;
    }
    if (status == SystemP_SUCCESS)
    {
        QSPI_Transaction transaction;

        QSPI_transaction_init(&transaction);
        transaction.addrOffset = offset;
        transaction.buf = (void *)buf;
        transaction.count = len;
        status = QSPI_readMemMapMode(obj->qspiHandle, &transaction);
    }
    return status;
}

static int32_t Flash_norQspiWrite(Flash_Config *config, uint32_t offset, uint8_t *buf, uint32_t len)
{
    int32_t status = SystemP_SUCCESS;
    Flash_NorQspiObject *obj = (Flash_NorQspiObject *)(config->object);
    Flash_DevConfig *devCfg = config->devConfig;
    Flash_Attrs *attrs = config->attrs;

    /* Validate address input */
    if ((offset + len) > (attrs->flashSize))
    {
        status = SystemP_FAILURE;
    }
    /* Check offset alignment */
    if(0 != (offset % attrs->pageSize))
    {
        status = SystemP_FAILURE;
    }
    if(status == SystemP_SUCCESS)
    {
        uint32_t pageSize, chunkLen, actual;
        QSPI_Transaction transaction;

        pageSize = attrs->pageSize;
        chunkLen = pageSize;

        for (actual = 0; actual < len; actual += chunkLen)
        {
            status = Flash_norQspiCmdWrite(config, devCfg->cmdWren, QSPI_CMD_INVALID_ADDR, 0, NULL, 0);

            if(status == SystemP_SUCCESS)
            {
                status = Flash_norQspiWEL(config, devCfg->flashWriteTimeout);
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
                status = QSPI_writeConfigMode(obj->qspiHandle, &transaction);
            }

            if(status == SystemP_SUCCESS)
            {
                status = Flash_norQspiWaitReady(config, devCfg->flashBusyTimeout);
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

static int32_t Flash_norQspiErase(Flash_Config *config, uint32_t blkNum)
{
    int32_t status = SystemP_SUCCESS;
    Flash_NorQspiObject *obj = (Flash_NorQspiObject *)(config->object);
    Flash_DevConfig *devCfg = (Flash_DevConfig *)config->devConfig;

    uint8_t  cmd = QSPI_CMD_INVALID_OPCODE;
    uint32_t cmdAddr = QSPI_CMD_INVALID_ADDR;
    uint32_t eraseTimeout = devCfg->flashBusyTimeout;

    if (blkNum == (uint32_t)(-1))
    {
        cmd    = devCfg->eraseCfg.cmdChipErase;
        /* Chip erase times can be several minutes and can vary from flash to
         * flash. Give UINT_MAX so that we wait for as much time it takes to
         * erase the flash completely.
         */
        eraseTimeout = (uint32_t)(-1);
    }
    else
    {
        cmdAddr = blkNum * config->attrs->blockSize;
        if(obj->numAddrBytes == 3)
        {
            cmd = devCfg->eraseCfg.cmdBlockErase3B;
        }
        else
        {
            cmd = devCfg->eraseCfg.cmdBlockErase4B;
        }

        if(blkNum >= config->attrs->blockCount)
        {
            status = SystemP_FAILURE;
        }
    }
    if(status == SystemP_SUCCESS)
    {
        status = Flash_norQspiCmdWrite(config, devCfg->cmdWren, QSPI_CMD_INVALID_ADDR, 0, NULL, 0);
    }
    if(status == SystemP_SUCCESS)
    {
        status = Flash_norQspiWaitReady(config, devCfg->flashBusyTimeout);
    }
    if(status == SystemP_SUCCESS)
    {
        status = Flash_norQspiCmdWrite(config, cmd, cmdAddr, obj->numAddrBytes, NULL, 0);
    }
    if(status == SystemP_SUCCESS)
    {
        status = Flash_norQspiWaitReady(config, eraseTimeout);
    }

    return status;
}

static int32_t Flash_norQspiEraseSector(Flash_Config *config, uint32_t sectorNum)
{
    int32_t status = SystemP_SUCCESS;
    Flash_NorQspiObject *obj = (Flash_NorQspiObject *)(config->object);
    Flash_Attrs *attrs = config->attrs;
    Flash_DevConfig *devCfg = config->devConfig;

    uint8_t cmd = QSPI_CMD_INVALID_OPCODE;
    uint32_t cmdAddr = QSPI_CMD_INVALID_ADDR;
    uint32_t eraseTimeout = devCfg->flashBusyTimeout;

    if(sectorNum == (uint32_t)(-1))
    {
        cmd = devCfg->eraseCfg.cmdChipErase;
        /* Chip erase times can be several minutes and can vary from flash to
         * flash. Give UINT_MAX so that we wait for as much time it takes to
         * erase the flash completely.
         */
        eraseTimeout = (uint32_t)(-1);
    }
    else
    {
        cmdAddr = sectorNum * attrs->sectorSize;
        if(obj->numAddrBytes == 3)
        {
            cmd = devCfg->eraseCfg.cmdSectorErase3B;
        }
        else
        {
            cmd = devCfg->eraseCfg.cmdSectorErase4B;
        }

        if(sectorNum >= attrs->sectorCount)
        {
            status = SystemP_FAILURE;
        }
    }
    if(SystemP_SUCCESS == status)
    {
        status = Flash_norQspiCmdWrite(config, devCfg->cmdWren, QSPI_CMD_INVALID_ADDR, 0, NULL, 0);
    }
    if(SystemP_SUCCESS == status)
    {
        status = Flash_norQspiWaitReady(config, devCfg->flashBusyTimeout);
    }
    if(SystemP_SUCCESS == status)
    {
        status = Flash_norQspiCmdWrite(config, cmd, cmdAddr, obj->numAddrBytes, NULL, 0);
    }
    if(SystemP_SUCCESS == status)
    {
        status = Flash_norQspiWaitReady(config, eraseTimeout);
    }

    return status;
}

static int32_t Flash_norQspiReset(Flash_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    Flash_NorQspiObject *obj = (Flash_NorQspiObject *)(config->object);
    Flash_DevConfig *devCfg = config->devConfig;

    if((devCfg->resetType & (uint8_t)(0x07)) != 0)
    {
        uint32_t clocks = 0;

        if((devCfg->resetType & (uint8_t)(0x01)) != 0)
        {
            clocks = 8;
        }
        if((devCfg->resetType & (uint8_t)(0x02)) != 0)
        {
            clocks = 10;
        }
        if((devCfg->resetType & (uint8_t)(0x04)) != 0)
        {
            clocks = 16;
        }
        if(clocks == 0)
        {
            status = SystemP_FAILURE;
        }
        else
        {
            if((clocks == 10) && (obj->numAddrBytes != 4))
            {
                clocks = 8;
            }

            while(clocks--)
            {
                Flash_norQspiCmdWrite(config, 0x0F, QSPI_CMD_INVALID_ADDR, 0, NULL, 0);
            }
        }
    }
    if((devCfg->resetType & (uint8_t)(0x08)) != 0)
    {
        /* Issue instruction 0xF0 */
        status = Flash_norQspiCmdWrite(config, 0xF0, QSPI_CMD_INVALID_ADDR, 0, NULL, 0);
    }
    if((devCfg->resetType & (uint8_t)(0x10)) != 0)
    {
        /* Issue reset enable, and then reset command */
        status = Flash_norQspiCmdWrite(config, 0x66, QSPI_CMD_INVALID_ADDR, 0, NULL, 0);
        if(SystemP_SUCCESS == status)
        {
            status = Flash_norQspiCmdWrite(config, 0x99, QSPI_CMD_INVALID_ADDR, 0, NULL, 0);
        }
    }

    return status;
}