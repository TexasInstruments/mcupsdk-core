/*
 *  Copyright(C) 2023 Texas Instruments Incorporated
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
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <board/flash.h>
#include <board/flash/ospi/flash_nand_ospi.h>
#include <drivers/hw_include/cslr.h>
#include <kernel/dpl/CacheP.h>

#define FLASH_OSPI_JEDEC_ID_SIZE_MAX            (8U)
#define FLASH_PAGE_SPARE_ARRAY_SIZE_BYTES       (64U)

static int32_t Flash_nandOspiOpen(Flash_Config *config, Flash_Params *params);
static int32_t Flash_nandOspiRead(Flash_Config *config, uint32_t offset, uint8_t *buf, uint32_t len);
static int32_t Flash_nandOspiErase(Flash_Config *config, uint32_t blkNum);
static int32_t Flash_nandOspiWrite(Flash_Config *config, uint32_t offset, uint8_t *buf, uint32_t len);
static int32_t Flash_nandOspiReset(Flash_Config *config);
static void Flash_nandOspiClose(Flash_Config *config);
static int32_t Flash_nandOspiCmdWrite(Flash_Config *config, uint8_t cmd, uint32_t cmdAddr,
                                     uint8_t numAddrBytes, uint8_t *txBuf, uint32_t txLen);
static int32_t Flash_nandOspiSetProtocol(Flash_Config *config, void *ospiHandle, Flash_Params *params);
static int32_t Flash_nandOspiDisableWriteProtection(Flash_Config *config);
static int32_t Flash_nandOspiCmdRead(Flash_Config *config, uint8_t cmd, uint32_t cmdAddr,
                            uint8_t numAddrBytes, uint8_t dummyBits, uint8_t *rxBuf, uint32_t rxLen);
static int32_t Flash_nandOspiWaitReady(Flash_Config *config, uint32_t timeOut);
static int32_t Flash_nandOspiSetDummyCycles(Flash_Config *config);
static int32_t Flash_nandOspiEnableDDR(Flash_Config *config);
static int32_t Flash_nandOspiEnableSDR(Flash_Config *config);
static int32_t Flash_nandOspiReadId(Flash_Config *config);
static int32_t Flash_NandOspiWriteDirect(Flash_Config *config, OSPI_Transaction *trans);
static int32_t Flash_nandOspiCheckEraseStatus(Flash_Config *config);
static int32_t Flash_nandOspiCheckProgStatus(Flash_Config *config);
static int32_t Flash_nandOspiPageLoad(Flash_Config *config, uint32_t offset);

/* Data to write to spare data section */
static uint32_t flashSpareAreaData[32] = { 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
                                           0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
                                           0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
                                           0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
                                           0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
                                           0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
                                           0xFFFFFFFF, 0xFFFFFFFF };

uint32_t gNandFlashToSpiProtocolMap[] =
{
    [FLASH_CFG_PROTO_1S_1S_1S] = OSPI_NAND_PROTOCOL(1,1,1,0),
    [FLASH_CFG_PROTO_1S_1S_4S] = OSPI_NAND_PROTOCOL(1,1,4,0),
    [FLASH_CFG_PROTO_8D_8D_8D] = OSPI_NAND_PROTOCOL(8,8,8,1),
};

Flash_Fxns gFlashNandOspiFxns = {

    .openFxn = Flash_nandOspiOpen,
    .closeFxn = Flash_nandOspiClose,
    .readFxn = Flash_nandOspiRead,
    .writeFxn = Flash_nandOspiWrite,
    .eraseFxn = Flash_nandOspiErase,
    .eraseSectorFxn = NULL,
    .resetFxn = Flash_nandOspiReset,
};

static int32_t Flash_nandOspiOpen(Flash_Config *config, Flash_Params *params)
{
    int32_t status = SystemP_SUCCESS;
    Flash_NandOspiObject *obj = NULL;
    Flash_Attrs *attrs = NULL;
    int32_t attackVectorStatus = SystemP_SUCCESS;

    if(obj->ospiHandle == NULL)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        attrs = config->attrs;
        obj = (Flash_NandOspiObject *)(config->object);
        obj->ospiHandle = OSPI_getHandle(attrs->driverInstance);
    }

    if(SystemP_SUCCESS == status)
    {
        OSPI_configResetPin(obj->ospiHandle, OSPI_RESETPIN_DEDICATED);
        /* Set device size and addressing bytes */
        OSPI_setDeviceSize(obj->ospiHandle, attrs->pageSize, attrs->blockSize);

        /* Set command opcode extension type */
        OSPI_setCmdExtType(obj->ospiHandle, config->devConfig->cmdExtType);

        /* Set current protocol as 1s1s1s */
        obj->currentProtocol = FLASH_CFG_PROTO_1S_1S_1S;

        Flash_nandOspiDisableWriteProtection(config);

        status += Flash_nandOspiSetProtocol(config, obj->ospiHandle, params);

        OSPI_setXferOpCodes(obj->ospiHandle, config->devConfig->protocolCfg.cmdRd, config->devConfig->protocolCfg.cmdWr);

        obj->currentProtocol = config->devConfig->protocolCfg.protocol;

        /* Set RD Capture Delay by reading ID */
        uint32_t readDataCapDelay = 4U;
        OSPI_setRdDataCaptureDelay(obj->ospiHandle, readDataCapDelay);

        status = Flash_nandOspiReadId(config);

        while((status != SystemP_SUCCESS) && (readDataCapDelay > 0U))
        {
            readDataCapDelay--;
            OSPI_setRdDataCaptureDelay(obj->ospiHandle, readDataCapDelay);
            status = Flash_nandOspiReadId(config);
        }

        /* Enable PHY if attack vector present and PHY mode is enabled */
        uint32_t phyTuningOffset = Flash_getPhyTuningOffset(config);
        if(OSPI_isPhyEnable(obj->ospiHandle))
        {
            /* For nand flash, data is read page by page. For phy pattern,
             * once the page load command is sent, the page data becomes available
             * at the offest zero of OSPI data region. Through out the tuning process,
             * phy pattern can be read from offset zero of OSPI data region.
             */
            attackVectorStatus = Flash_nandOspiPageLoad(config, phyTuningOffset);
            attackVectorStatus += OSPI_phyReadAttackVector(obj->ospiHandle, 0);

            if(attackVectorStatus != SystemP_SUCCESS)
            {
                /* Flash the attack vector to the last block */
                uint32_t blk = 0, page = 0;
                uint32_t phyTuningData = 0,phyTuningDataSize = 0;

                OSPI_phyGetTuningData(&phyTuningData, &phyTuningDataSize);
                Flash_offsetToBlkPage(config, phyTuningOffset, &blk, &page);
                Flash_nandOspiErase(config, blk);
                Flash_nandOspiWrite(config, phyTuningOffset, (uint8_t *)phyTuningData, phyTuningDataSize);
                attackVectorStatus = Flash_nandOspiPageLoad(config, phyTuningOffset);
                attackVectorStatus += OSPI_phyReadAttackVector(obj->ospiHandle, 0);
            }

            if(attackVectorStatus == SystemP_SUCCESS)
            {
                status += OSPI_phyTuneSDR(obj->ospiHandle, 0);
                if(status == SystemP_SUCCESS)
                {
                    obj->phyEnable = TRUE;
                    OSPI_setPhyEnableSuccess(obj->ospiHandle, TRUE);
                }
            }
            else
            {
                DebugP_logError("%s : PHY enabling failed!!! Continuing without PHY...\r\n", __func__);
                obj->phyEnable = FALSE;
                OSPI_setPhyEnableSuccess(obj->ospiHandle, FALSE);
            }
        }
        else
        {
            obj->phyEnable = FALSE;
        }
    }

    return status;
}

static void Flash_nandOspiClose(Flash_Config *config)
{
    Flash_NandOspiObject *obj = (Flash_NandOspiObject *)(config->object);

    /* Reset the flash such that other modules can initialise the
     *  Flash config registers again.
     */
    (void)Flash_nandOspiReset(config);

    /* Disable the PHY */
    OSPI_disablePhy(obj->ospiHandle);

    obj->ospiHandle = NULL;

    /* OSPI Driver will be closed outside flash */

    return;
}


static int32_t Flash_nandOspiPageLoad(Flash_Config *config, uint32_t offset)
{
    int32_t status = SystemP_SUCCESS;
    Flash_NandOspiObject *obj = NULL;
    Flash_DevConfig *devCfg = NULL;
    Flash_NandConfig *nandCfg = NULL;
    Flash_Attrs *attrs = NULL;

    uint32_t pageNum;
    uint8_t cmd;
    uint8_t addrLen;

    if(config != NULL)
    {
        obj = (Flash_NandOspiObject *)(config->object);
        devCfg = config->devConfig;
        attrs = config->attrs;
        nandCfg = devCfg->nandCfg;
        pageNum = offset / attrs->pageSize;
        cmd = nandCfg->cmdPageLoad;

        if(obj->currentProtocol == FLASH_CFG_PROTO_8D_8D_8D)
        {
            addrLen = 2;
        }
        else if(obj->currentProtocol == FLASH_CFG_PROTO_1S_1S_1S ||
                obj->currentProtocol == FLASH_CFG_PROTO_1S_1S_4S)
        {
            addrLen = 3;
        }

        if(status == SystemP_SUCCESS)
        {
            status += Flash_nandOspiCmdWrite(config, cmd, pageNum, addrLen, NULL, 0);

            if(status == SystemP_SUCCESS)
            {
                status += Flash_nandOspiWaitReady(config, 1000u);
            }
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

static int32_t Flash_nandOspiRead(Flash_Config *config, uint32_t offset, uint8_t *buf, uint32_t len)
{
    int32_t status = SystemP_SUCCESS;
    Flash_NandOspiObject *obj = NULL;
    OSPI_Transaction transaction;
    Flash_DevConfig *devCfg = NULL;
    Flash_NandConfig *nandCfg = NULL;
    Flash_Attrs *attrs = NULL;

    uint32_t pageSize;
    uint32_t numPages;
    uint32_t offsetFromPage;
    uint32_t pageNum;
    uint32_t readAddr;
    uint8_t cmd;
    uint8_t addrLen;

    if(config == NULL)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        obj = (Flash_NandOspiObject *)(config->object);
        devCfg = config->devConfig;
        nandCfg = devCfg->nandCfg;
        attrs = config->attrs;
    }

    if(status == SystemP_SUCCESS)
    {
        if(offset + len > attrs->flashSize)
        {
            status = SystemP_FAILURE;
        }
    }

    pageSize = attrs->pageSize;
    pageNum = offset / pageSize;
    offsetFromPage = offset % pageSize;
    numPages = (len + (pageSize - 1) + offsetFromPage)/pageSize;
    readAddr = offset;

    for(uint32_t i = 0; i < numPages; i++)
    {
        cmd = nandCfg->cmdPageLoad;

        if(obj->currentProtocol == FLASH_CFG_PROTO_8D_8D_8D)
        {
            addrLen = 2;
        }
        else if(obj->currentProtocol == FLASH_CFG_PROTO_1S_1S_1S ||
                obj->currentProtocol == FLASH_CFG_PROTO_1S_1S_4S )
        {
            addrLen = 3;
        }

        if(status == SystemP_SUCCESS)
        {
            status = Flash_nandOspiWaitReady(config, 1000u);
        }

        if(status == SystemP_SUCCESS)
        {
            status = Flash_nandOspiCmdWrite(config, cmd, pageNum, addrLen, NULL, 0);
        }

        if(status != SystemP_SUCCESS)
        {
            break;
        }

        if(status == SystemP_SUCCESS)
        {
            status = Flash_nandOspiWaitReady(config, 1000u);
        }

        /* Check for bad block */
        if(obj->badBlockCheck && status == SystemP_SUCCESS)
        {
            uint8_t readBBMarkerBuf[2];
            OSPI_Transaction_init(&transaction);

            transaction.addrOffset = attrs->pageSize;
            transaction.count = 2;
            transaction.buf = readBBMarkerBuf;

            status = OSPI_readDirect(obj->ospiHandle, &transaction);

            if(readBBMarkerBuf[0] != 0xFF || readBBMarkerBuf[1] != 0xFF)
            {
                status = SystemP_FAILURE;
                break;
            }
        }

        /* Read data if not Bad block */
        if(status == SystemP_SUCCESS)
        {
            OSPI_Transaction_init(&transaction);
            transaction.buf = (void *)((uint32_t)buf + readAddr - offset);

            if(numPages == 1)
            {
                transaction.addrOffset = offsetFromPage;
                transaction.count = len;
            }
            else
            {
                if(i == 0)
                {
                    transaction.addrOffset = offsetFromPage;
                    transaction.count = pageSize - offsetFromPage;
                }
                else if(i < (numPages-1))
                {
                    transaction.addrOffset = 0;
                    transaction.count = pageSize;
                }
                else if(i == (numPages-1))
                {
                    transaction.addrOffset = 0;
                    transaction.count = (offsetFromPage + len ) % pageSize;

                    if(transaction.count == 0)
                    {
                        transaction.count = pageSize;
                    }
                }
            }

            status = OSPI_readDirect(obj->ospiHandle, &transaction);
        }

        if(readAddr % pageSize == 0)
        {
            readAddr += pageSize;
        }
        else
        {
            readAddr += (pageSize - (readAddr % pageSize));
        }

        if(status != SystemP_SUCCESS)
        {
            break;
        }

        pageNum++;
    }

    return status;
}

static int32_t Flash_nandOspiWrite(Flash_Config *config, uint32_t offset, uint8_t *buf, uint32_t len)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t cmd;
    Flash_NandOspiObject *obj;
    Flash_Attrs *attrs = NULL;
    Flash_DevConfig *devCfg = NULL;
    Flash_NandConfig *nandCfg = NULL;

    if(config != NULL)
    {
        obj = (Flash_NandOspiObject *)(config->object);
        attrs = config->attrs;
        devCfg = config->devConfig;
        nandCfg = devCfg->nandCfg;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    /* Validate address input */
    if((offset + len) > (attrs->blockCount*attrs->pageCount*attrs->pageSize))
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
        uint32_t byteAddr;
        uint32_t pageAddr;
        uint32_t colmAddr;
        OSPI_Transaction transaction;

        pageSize = attrs->pageSize;

        byteAddr = offset & (pageSize - 1);

        for(actual = 0; actual < len; actual += chunkLen)
        {
            status = Flash_nandOspiCmdWrite(config, devCfg->cmdWren, OSPI_CMD_INVALID_ADDR, 0, NULL, 0);

            if(status == SystemP_SUCCESS)
            {
                status = Flash_nandOspiWaitReady(config, 1000U);
            }

            chunkLen = ((len - actual) < (pageSize - byteAddr) ?
                    (len - actual) : (pageSize - byteAddr));

            /* Split the page and column addresses */
            pageAddr = offset / pageSize;
            colmAddr = offset % pageSize;

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
                transaction.addrOffset = colmAddr;
                transaction.buf = (void *)(buf + actual);
                transaction.count = chunkLen;
                status = Flash_NandOspiWriteDirect(config, &transaction);
            }

            if(status == SystemP_SUCCESS)
            {
                status = Flash_nandOspiWaitReady(config, 10000U);
            }

            if(status == SystemP_SUCCESS)
            {
                cmd = nandCfg->cmdPageProg;
                if(obj->currentProtocol == FLASH_CFG_PROTO_8D_8D_8D)
                {
                    status = Flash_nandOspiCmdWrite(config, cmd, pageAddr, 2, NULL, 0);
                }
                else
                {
                    status = Flash_nandOspiCmdWrite(config, cmd, pageAddr, 3, NULL, 0);
                }
            }

            if(status == SystemP_SUCCESS)
            {
                status = Flash_nandOspiWaitReady(config, 10000U);
            }

            if(status == SystemP_SUCCESS)
            {
                for(int timeOut = 10000; timeOut > 0; timeOut--)
                {
                    status = Flash_nandOspiCheckProgStatus(config);
                    if(status == SystemP_SUCCESS)
                    {
                        break;
                    }
                }
            }

            if(status != SystemP_SUCCESS)
            {
                break;
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

static int32_t Flash_nandOspiErase(Flash_Config *config, uint32_t blkNum)
{
    int32_t status = SystemP_SUCCESS;
    Flash_NandOspiObject *obj;
    Flash_Attrs *attrs = NULL;
    Flash_DevConfig *devCfg = NULL;

    uint8_t cmd = OSPI_CMD_INVALID_OPCODE;
    uint32_t cmdAddr = OSPI_CMD_INVALID_ADDR;
    uint8_t cmdWren;

    if(config != NULL)
    {
        obj = (Flash_NandOspiObject *)(config->object);
        attrs = config->attrs;
        devCfg = config->devConfig;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        cmdWren = devCfg->cmdWren;
        cmdAddr = (blkNum * attrs->blockSize) / attrs->pageSize;
        cmd = devCfg->eraseCfg.cmdBlockErase;
    }

    if(blkNum >= config->attrs->blockCount)
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        status = Flash_nandOspiWaitReady(config, 1000U);
    }

    if(status == SystemP_SUCCESS)
    {
        status = Flash_nandOspiCmdWrite(config, cmdWren, OSPI_CMD_INVALID_ADDR, 0, NULL, 0);
    }

    if(status == SystemP_SUCCESS)
    {
        status = Flash_nandOspiWaitReady(config, 1000U);
    }

    if(status == SystemP_SUCCESS)
    {
        if(obj->currentProtocol == FLASH_CFG_PROTO_8D_8D_8D)
        {
            status = Flash_nandOspiCmdWrite(config, cmd, cmdAddr, 2, NULL, 0);
        }
        else
        {
            status = Flash_nandOspiCmdWrite(config, cmd, cmdAddr, 3, NULL, 0);
        }
    }

    if(status == SystemP_SUCCESS)
    {
        status = Flash_nandOspiWaitReady(config, 10000U);
    }

    if(status == SystemP_SUCCESS)
    {
        for(int timeOut = 10000; timeOut > 0; timeOut--)
        {
            status = Flash_nandOspiCheckEraseStatus(config);
            if(status == SystemP_SUCCESS)
            {
                break;
            }
        }
    }

    return status;
}

static int32_t Flash_nandOspiReset(Flash_Config *config)
{

    int32_t status = SystemP_FAILURE;
    Flash_DevConfig *devCfg = NULL;
    if(config != NULL)
    {
        devCfg = config->devConfig;
        status = Flash_nandOspiCmdWrite(config, devCfg->cmdWren, OSPI_CMD_INVALID_ADDR, 0, NULL, 0);
        if(status == SystemP_SUCCESS)
        {
            status = Flash_nandOspiWaitReady(config, devCfg->flashBusyTimeout);
        }

        if(status == SystemP_SUCCESS)
        {
            status = Flash_nandOspiCmdWrite(config, 0xFF, OSPI_CMD_INVALID_ADDR, 0, NULL, 0);
        }

        Flash_nandOspiWaitReady(config, devCfg->flashBusyTimeout);
    }

    return status;
}

static int32_t Flash_nandOspiSetProtocol(Flash_Config *config, void *ospiHandle, Flash_Params *params)
{
    int32_t status = SystemP_SUCCESS;
    Flash_NandOspiObject *obj;

    if((config == NULL) ||(NULL == ospiHandle))
    {
        status = SystemP_FAILURE;
    }
    else
    {
        Flash_DevConfig *devCfg = config->devConfig;
        obj =(Flash_NandOspiObject *)(config->object);

        /* Configure the flash according to protocol */
        uint32_t protocol = devCfg->protocolCfg.protocol;
        uint32_t dummyCycles = devCfg->protocolCfg.dummyClksRd;

        if(FLASH_CFG_PROTO_CUSTOM == protocol)
        {
            status += params->custProtoFxn(config);
        }
        else
        {
            /* OOB support available for:
                * 1S_1S_1S
                * 1S_8S_8S
                * 8D_8D_8D
                */
            switch(protocol)
            {
                case FLASH_CFG_PROTO_1S_1S_1S:
                    break;

                case FLASH_CFG_PROTO_1S_1S_4S:

                    OSPI_enableSDR(obj->ospiHandle);

                    status += Flash_nandOspiSetDummyCycles(config);
                    status += Flash_nandOspiEnableSDR(config);
                    break;

                case FLASH_CFG_PROTO_8D_8D_8D:

                    status = Flash_nandOspiSetDummyCycles(config);

                    status += Flash_nandOspiEnableDDR(config);

                    break;

                default:
                    status = SystemP_FAILURE;
                    break;
            }
        }

        OSPI_setReadDummyCycles(obj->ospiHandle, dummyCycles);
        OSPI_setNumAddrBytes(obj->ospiHandle, 2);
        OSPI_setProtocol((OSPI_Handle)(obj->ospiHandle), gNandFlashToSpiProtocolMap[protocol]);
    }

    return status;
}

static int32_t Flash_nandOspiDisableWriteProtection(Flash_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    Flash_DevConfig *devConfig = NULL;
    Flash_NandConfig *nandCfg = NULL;
    uint8_t txBuf = 0;
    uint8_t cmd = 0;
    uint32_t cmdAddr = 0;

    if(NULL != config)
    {
        devConfig =(Flash_DevConfig *)config->devConfig;
        nandCfg = devConfig->nandCfg;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        cmd = nandCfg->cmdWrsr;
        cmdAddr = nandCfg->srWriteProtectReg;
        txBuf = nandCfg->srWriteProtectMask & 0;

        status = Flash_nandOspiCmdWrite(config, cmd, cmdAddr, 1, &txBuf, 1);
    }

    return status;
}

static int32_t Flash_nandOspiCmdWrite(Flash_Config *config, uint8_t cmd, uint32_t cmdAddr,
                                     uint8_t numAddrBytes, uint8_t *txBuf, uint32_t txLen)
{
    int32_t status = SystemP_SUCCESS;
    Flash_NandOspiObject *obj = NULL;
    OSPI_WriteCmdParams wrParams;

    if(config !=  NULL)
    {
        obj =(Flash_NandOspiObject *)(config->object);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        OSPI_WriteCmdParams_init(&wrParams);
        wrParams.cmd          = cmd;
        wrParams.cmdAddr      = cmdAddr;
        wrParams.numAddrBytes = numAddrBytes;
        wrParams.txDataBuf    = txBuf;
        wrParams.txDataLen    = txLen;

        status = OSPI_writeCmd(obj->ospiHandle, &wrParams);
    }

    return status;
}

static int32_t Flash_nandOspiWaitReady(Flash_Config *config, uint32_t timeOut)
{
    int32_t status = SystemP_SUCCESS;
    Flash_DevConfig *devCfg = NULL;
    Flash_NandOspiObject *obj = NULL;
    Flash_NandConfig *nandCfg = NULL;

    uint8_t readStatus[2] = { 0 };
    uint8_t cmd = 0, numAddrBytes = 0, readBytes = 0;
    uint32_t cmdAddr = OSPI_CMD_INVALID_ADDR;
    uint8_t dummyBits = 0;
    uint8_t bitMask = 0;

    if(config != NULL)
    {
        obj = (Flash_NandOspiObject *)(config->object);
        devCfg = config->devConfig;
        nandCfg = devCfg->nandCfg;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        if((devCfg->xspiWipRdCmd != 0x00) && (obj->currentProtocol == FLASH_CFG_PROTO_8D_8D_8D))
        {
            cmd = devCfg->xspiWipRdCmd;
            cmdAddr = devCfg->xspiWipReg;
            dummyBits = nandCfg->xspiRdsrDummy;
            bitMask = devCfg->srWip;
            readBytes = 2;
            numAddrBytes = 2;
        }
        else if(obj->currentProtocol == FLASH_CFG_PROTO_1S_1S_1S ||
                obj->currentProtocol == FLASH_CFG_PROTO_1S_1S_4S )
        {
            cmd = devCfg->cmdRdsr;
            cmdAddr = nandCfg->srWipReg;
            dummyBits = 0;
            bitMask = devCfg->srWip;
            readBytes = 1;
            numAddrBytes = 1;
        }

        while((status != SystemP_SUCCESS) || (timeOut > 0))
        {
            status = Flash_nandOspiCmdRead(config, cmd, cmdAddr, numAddrBytes, dummyBits, readStatus, readBytes);

            if((status == SystemP_SUCCESS) && ((readStatus[0] & bitMask) == 0))
            {
                break;
            }
            timeOut --;
        }

        if((readStatus[0] & bitMask)==0)
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


static int32_t Flash_nandOspiCmdRead(Flash_Config *config, uint8_t cmd, uint32_t cmdAddr,
                            uint8_t numAddrBytes, uint8_t dummyBits, uint8_t *rxBuf, uint32_t rxLen)
{
    int32_t status = SystemP_SUCCESS;
    Flash_NandOspiObject *obj = NULL;
    OSPI_ReadCmdParams  rdParams;

    if(config != NULL)
    {
        obj = (Flash_NandOspiObject *)(config->object);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        OSPI_ReadCmdParams_init(&rdParams);
        rdParams.cmd           = cmd;
        rdParams.cmdAddr       = cmdAddr;
        rdParams.numAddrBytes  = numAddrBytes;
        rdParams.rxDataBuf     = rxBuf;
        rdParams.rxDataLen     = rxLen;
        rdParams.dummyBits     = dummyBits;

        status = OSPI_readCmd(obj->ospiHandle, &rdParams);
    }

    return status;

}

static int32_t Flash_nandOspiSetDummyCycles(Flash_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    Flash_NandOspiObject *obj = (Flash_NandOspiObject *)(config->object);
    Flash_DevConfig *devCfg = config->devConfig;
    FlashCfg_RegConfig *dummyCfg = &(devCfg->protocolCfg.dummyCfg);

    /* Send Write Enable Command */
    if(status == SystemP_SUCCESS)
    {
        status = Flash_nandOspiCmdWrite(config, devCfg->cmdWren,
                                OSPI_CMD_INVALID_ADDR, 0, NULL, 0);
    }

    if(status == SystemP_SUCCESS)
    {
        status = Flash_nandOspiWaitReady(config, 1000U);
    }

    if((dummyCfg->cmdRegRd != 0) && (dummyCfg->cmdRegWr != 0))
    {
        uint8_t cfgReg = 0;
        if(dummyCfg->isAddrReg == TRUE)
        {
            if(obj->currentProtocol == FLASH_CFG_PROTO_8D_8D_8D)
            {
                status += Flash_nandOspiCmdRead(config, dummyCfg->cmdRegRd, dummyCfg->cfgReg, 3, 8, &cfgReg, 2);
            }
            else
            {
                status += Flash_nandOspiCmdRead(config, dummyCfg->cmdRegRd, dummyCfg->cfgReg, 3, 8, &cfgReg, 1);
            }
        }
        else
        {
            if(obj->currentProtocol == FLASH_CFG_PROTO_8D_8D_8D)
            {
                status += Flash_nandOspiCmdRead(config, dummyCfg->cmdRegRd, OSPI_CMD_INVALID_ADDR, 0, 8, &cfgReg, 2);
            }
            else
            {
                status += Flash_nandOspiCmdRead(config, dummyCfg->cmdRegRd, OSPI_CMD_INVALID_ADDR, 0, 8, &cfgReg, 1);
            }
        }

        if(SystemP_SUCCESS == status)
        {
            /* Clear the config bits in the register  */
            cfgReg &= ~(uint8_t)(dummyCfg->mask);
            /* Bitwise OR the bit pattern for setting the dummyCycle selected */
            cfgReg |= (dummyCfg->cfgRegBitP << dummyCfg->shift);
            /* There is register config, address might not be needed */

            if(dummyCfg->isAddrReg == TRUE)
            {
                status += Flash_nandOspiCmdWrite(config, dummyCfg->cmdRegWr, dummyCfg->cfgReg, 3, &cfgReg, 1);
            }
            else
            {
                status += Flash_nandOspiCmdWrite(config, dummyCfg->cmdRegWr, OSPI_CMD_INVALID_ADDR, 0, &cfgReg, 1);
            }
        }
    }

    return status;
}

static int32_t Flash_nandOspiEnableDDR(Flash_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    Flash_DevConfig *devCfg = NULL;
    FlashCfg_RegConfig *ddrCfg = NULL;
    Flash_NandOspiObject *obj = NULL;

    if(config != NULL)
    {
        devCfg = config->devConfig;
        ddrCfg = &(devCfg->protocolCfg.protoCfg);
        obj = (Flash_NandOspiObject *)(config->object);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    /* Send Write Enable Command */
    if(status == SystemP_SUCCESS)
    {
        status = Flash_nandOspiCmdWrite(config, devCfg->cmdWren,
                                OSPI_CMD_INVALID_ADDR, 0, NULL, 0);
    }

    /* Check for busy bit of flash */
    if(status == SystemP_SUCCESS)
    {
        status = Flash_nandOspiWaitReady(config, 1000U);
    }

    if(status == SystemP_SUCCESS)
    {
        if((ddrCfg->cmdRegRd != 0) && (ddrCfg->cmdRegWr != 0))
        {
            uint8_t cfgReg = 0;
            if(ddrCfg->isAddrReg == TRUE)
            {
                if(obj->currentProtocol == FLASH_CFG_PROTO_8D_8D_8D)
                {
                    status += Flash_nandOspiCmdRead(config, ddrCfg->cmdRegRd, ddrCfg->cfgReg, 3, 8, &cfgReg, 2);
                }
                else
                {
                    status += Flash_nandOspiCmdRead(config, ddrCfg->cmdRegRd, ddrCfg->cfgReg, 3, 8, &cfgReg, 1);
                }
            }
            else
            {
                if(obj->currentProtocol == FLASH_CFG_PROTO_8D_8D_8D)
                {
                    status += Flash_nandOspiCmdRead(config, ddrCfg->cmdRegRd, OSPI_CMD_INVALID_ADDR, 0, 8, &cfgReg, 2);
                }
                else
                {
                    status += Flash_nandOspiCmdRead(config, ddrCfg->cmdRegRd, OSPI_CMD_INVALID_ADDR, 0, 8, &cfgReg, 1);
                }
            }

            if(SystemP_SUCCESS == status)
            {
                /* Clear the config bits in the register  */
                cfgReg &= ~(uint8_t)(ddrCfg->mask);

                /* Bitwise OR the bit pattern for setting the dummyCycle selected */
                cfgReg |= (ddrCfg->cfgRegBitP << ddrCfg->shift);

                /* There is register config, address might not be needed */
                if(ddrCfg->isAddrReg == TRUE)
                {
                    status += Flash_nandOspiCmdWrite(config, ddrCfg->cmdRegWr, ddrCfg->cfgReg, 3, &cfgReg, 1);
                }
                else
                {
                    status += Flash_nandOspiCmdWrite(config, ddrCfg->cmdRegWr, OSPI_CMD_INVALID_ADDR, 0, &cfgReg, 1);
                }
            }
        }
    }

    return status;
}

static int32_t Flash_nandOspiEnableSDR(Flash_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    Flash_DevConfig *devCfg = NULL;
    FlashCfg_RegConfig *sdrCfg = NULL;

    if(config != NULL)
    {
        devCfg = config->devConfig;
        sdrCfg = &(devCfg->protocolCfg.protoCfg);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    /* Send Write Enable Command */
    if(status == SystemP_SUCCESS)
    {
        status = Flash_nandOspiCmdWrite(config, devCfg->cmdWren,
                                OSPI_CMD_INVALID_ADDR, 0, NULL, 0);
    }

    /* Check for busy bit of flash */
    if(status == SystemP_SUCCESS)
    {
        status = Flash_nandOspiWaitReady(config, 1000U);
    }

    if(status == SystemP_SUCCESS)
    {
        if((sdrCfg->cmdRegRd != 0) && (sdrCfg->cmdRegWr != 0))
        {
            uint8_t cfgReg = 0;

            if(sdrCfg->isAddrReg == TRUE)
            {
                status += Flash_nandOspiCmdRead(config, sdrCfg->cmdRegRd, sdrCfg->cfgReg, 3, 8, &cfgReg, 1);
            }
            else
            {
                status += Flash_nandOspiCmdRead(config, sdrCfg->cmdRegRd, OSPI_CMD_INVALID_ADDR, 0, 8, &cfgReg, 1);
            }

            if(SystemP_SUCCESS == status)
            {
                /* Clear the config bits in the register  */
                cfgReg &= ~(uint8_t)(sdrCfg->mask & 0xFF);
                /* Bitwise OR the bit pattern for setting the dummyCycle selected */
                cfgReg |= (sdrCfg->cfgRegBitP << sdrCfg->shift);

                /* There is register config, address might not be needed */
                if(sdrCfg->isAddrReg == TRUE)
                {
                    status += Flash_nandOspiCmdWrite(config, sdrCfg->cmdRegWr, sdrCfg->cfgReg, 3, &cfgReg, 1);
                }
                else
                {
                    status += Flash_nandOspiCmdWrite(config, sdrCfg->cmdRegWr, OSPI_CMD_INVALID_ADDR, 0, &cfgReg, 1);
                }
            }
        }
    }

    return status;
}


static int32_t Flash_nandOspiReadId(Flash_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    Flash_NandOspiObject *obj = NULL;
    Flash_DevConfig *devCfg = NULL;
    FlashCfg_ReadIDConfig *idCfg = NULL;

    uint8_t  idCode[FLASH_OSPI_JEDEC_ID_SIZE_MAX];
    uint32_t idNumBytes = 3;
    uint32_t numAddrBytes = 0;

    if(config != NULL)
    {
        obj = (Flash_NandOspiObject *)(config->object);
        devCfg = config->devConfig;
        idCfg = &(devCfg->idCfg);
    }
    else
    {
        status  = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        if(obj->currentProtocol == FLASH_CFG_PROTO_8D_8D_8D)
        {
            idNumBytes = 4;
        }

        status = Flash_nandOspiCmdRead(config, idCfg->cmd,
                                                OSPI_CMD_INVALID_ADDR, numAddrBytes, 8, idCode, idNumBytes);
    }

    /* Verify ID with filled data */
    if (status == SystemP_SUCCESS)
    {
        uint32_t manfID, devID;

        manfID = (uint32_t)idCode[0];
        devID = ((uint32_t)idCode[1] << 8) | ((uint32_t)idCode[2]);
        if (!((manfID == config->attrs->manufacturerId) && (devID == config->attrs->deviceId)))
        {
            /* Try the other 3 bytes */
            manfID = (uint32_t)idCode[3];
            devID = ((uint32_t)idCode[4] << 8) | ((uint32_t)idCode[5]);
            if (!((manfID == config->attrs->manufacturerId) && (devID == config->attrs->deviceId)))
            {
                status = SystemP_FAILURE;
            }
        }
        else
        {
            /* Success, nothing to do */;
        }
    }

    return status;
}

static int32_t Flash_NandOspiWriteDirect(Flash_Config *config, OSPI_Transaction *trans)
{
    int32_t status = SystemP_SUCCESS;

    Flash_NandOspiObject *obj;
    Flash_Attrs *attrs = config->attrs;

    if(config != NULL)
    {
        obj = (Flash_NandOspiObject *)(config->object);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        OSPI_writeDirect(obj->ospiHandle, trans);

        OSPI_Transaction spareByteTrans;
        OSPI_Transaction_init(&spareByteTrans);

        spareByteTrans.count = FLASH_PAGE_SPARE_ARRAY_SIZE_BYTES;
        spareByteTrans.addrOffset = attrs->pageSize;
        spareByteTrans.buf = flashSpareAreaData;

        OSPI_writeDirect(obj->ospiHandle, &spareByteTrans);

    }

    return status;
}

static int32_t Flash_nandOspiCheckProgStatus(Flash_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t readStatus[2];
    Flash_NandOspiObject *obj = NULL;
    Flash_DevConfig *devCfg = NULL;
    Flash_NandConfig *nandCfg = NULL;
    uint8_t cmd = 0, numAddrBytes = 0, readBytes = 0;
    uint32_t cmdAddr = 0;
    uint8_t dummyBits = 0;

    if(config != NULL)
    {
        obj = (Flash_NandOspiObject *)(config->object);
        devCfg = config->devConfig;
        nandCfg = devCfg->nandCfg;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        if(obj->currentProtocol == FLASH_CFG_PROTO_8D_8D_8D)
        {
            cmd = devCfg->cmdRdsr;
            dummyBits = nandCfg->xspiRdsrDummy;
            cmdAddr = nandCfg->xspiProgStatusReg;
            numAddrBytes = 2;
            readBytes = 2;
        }
        else if(obj->currentProtocol == FLASH_CFG_PROTO_1S_1S_1S ||
                obj->currentProtocol == FLASH_CFG_PROTO_1S_1S_4S )
        {
            cmd = devCfg->cmdRdsr;
            dummyBits = 0;
            cmdAddr = nandCfg->progStatusReg;
            numAddrBytes = 1;
            readBytes = 1;
        }

        status = Flash_nandOspiCmdRead(config, cmd, cmdAddr, numAddrBytes, dummyBits, readStatus, readBytes);

        if(status == SystemP_SUCCESS)
        {
            if((readStatus[0] & nandCfg->srProgStatus) != 0)
            {
                status = SystemP_FAILURE;
            }
        }
    }

    return status;
}

static int32_t Flash_nandOspiCheckEraseStatus(Flash_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t readStatus[2];
    Flash_NandOspiObject *obj = NULL;
    Flash_DevConfig *devCfg = NULL;
    Flash_NandConfig *nandCfg = NULL;
    uint8_t cmd = 0, numAddrBytes = 0, readBytes = 0;
    uint32_t cmdAddr = 0;
    uint8_t dummyBits = 0;

    if(config != NULL)
    {
        obj = (Flash_NandOspiObject *)(config->object);
        devCfg = config->devConfig;
        nandCfg = devCfg->nandCfg;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        if(obj->currentProtocol == FLASH_CFG_PROTO_8D_8D_8D)
        {
            cmd = devCfg->cmdRdsr;
            dummyBits = nandCfg->xspiRdsrDummy;
            cmdAddr = nandCfg->xspiEraseStatusReg;
            numAddrBytes = 2;
            readBytes = 2;
        }
        else if(obj->currentProtocol == FLASH_CFG_PROTO_1S_1S_1S ||
                obj->currentProtocol == FLASH_CFG_PROTO_1S_1S_4S )
        {
            cmd = devCfg->cmdRdsr;
            dummyBits = 0;
            cmdAddr = nandCfg->eraseStatusReg;
            numAddrBytes = 1;
            readBytes = 1;
        }

        status = Flash_nandOspiCmdRead(config, cmd, cmdAddr, numAddrBytes, dummyBits, readStatus, readBytes);

        if(status == SystemP_SUCCESS)
        {
            if((readStatus[0] && nandCfg->srEraseStatus) != 0)
            {
                status = SystemP_FAILURE;
            }
        }
    }

    return status;
}
