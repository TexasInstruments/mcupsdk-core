/*
 *  Copyright (C) 2022-2023 Texas Instruments Incorporated
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

#include <string.h>
#include <board/flash.h>
#include <drivers/hw_include/cslr.h>
#include "flash_nand_gpmc.h"

static int32_t Flash_nandGpmcOpen (Flash_Config *config, Flash_Params *params);
static void Flash_nandGpmcClose (Flash_Config *config);
static int32_t Flash_nandGpmcRead (Flash_Config *config, uint32_t offset, uint8_t *buf, uint32_t len);
static int32_t Flash_nandGpmcWrite (Flash_Config *config, uint32_t offset, uint8_t *buf, uint32_t len);
static int32_t Flash_nandGpmcErase (Flash_Config *config, uint32_t blkNum);


static int32_t Flash_nandGpmcReadPage(Flash_Config *config, uint32_t blockNum,
                                uint32_t    pageNum,
                                uint8_t    *buf);
static int32_t Flash_nandGpmcWritePage(Flash_Config *config, uint32_t    blockNum,
                                uint32_t    pageNum,
                                uint8_t    *pTxData);
static int32_t Flash_nandGpmcResetMemory(Flash_Config *config);
static int32_t Flash_nandGpmcReadId (Flash_Config *config);
static int32_t Flash_nandGpmcConfigureEccBCH(Flash_Config *config);
static int32_t Flash_nandGpmcDeviceStatus(Flash_Config *config);
static int32_t Flash_nandGpmcBchEccCheckAndCorrect(Flash_Config *config,
                                                uint8_t    *pEccRead,
                                                uint8_t    *pData);
static int32_t Flash_nandGpmcEccCheckAndCorrect(Flash_Config *config,
                                               uint8_t    *pEccRead,
                                               uint8_t    *pData);
static int32_t Flash_nandGpmcEccCalculate(Flash_Config *config, uint8_t *pEccData);
static int32_t Flash_nandGpmcCheckBadBlock(Flash_Config *config, uint32_t blockNum);

Flash_Fxns gFlashNandGpmcFxns = {
    .openFxn = Flash_nandGpmcOpen,
    .closeFxn = Flash_nandGpmcClose,
    .readFxn = Flash_nandGpmcRead,
    .writeFxn = Flash_nandGpmcWrite,
    .eraseFxn = Flash_nandGpmcErase,
    .eraseSectorFxn = NULL,
    .resetFxn = Flash_nandGpmcResetMemory,
};

static int32_t Flash_nandGpmcResetMemory(Flash_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    Flash_NandGpmcObject *object = NULL;
    Flash_DevConfig *devConfig = NULL;
    GPMC_nandCmdParams params;

    /* Input parameter validation. */
    if (config != NULL)
    {
        object = (Flash_NandGpmcObject *)config->object;
        devConfig = (Flash_DevConfig *)config->devConfig;
    }
    else
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        /* Prepare command parameters to send FLASH reset command. */
        GPMC_writeNandCommandParamsInit(&params);
        params.cmdCycle1 = devConfig->cmdReset;
        params.waitTimeout = NAND_DEVICE_RESET_TIMEOUT;

        status += GPMC_writeNandCommand(object->gpmcHandle, &params);
    }

    if(status == SystemP_SUCCESS)
    {
        status += Flash_nandGpmcDeviceStatus(config);
    }

    return status;

}
static int32_t Flash_nandGpmcReadId (Flash_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    Flash_DevConfig *devConfig = NULL;
    Flash_NandGpmcObject *object = NULL;
    GPMC_nandCmdParams params;

    /* Input parameter validation. */
    if (config != NULL)
    {
        object = (Flash_NandGpmcObject *)(config->object);
        devConfig = (Flash_DevConfig *)config->devConfig;
        uint8_t readId[devConfig->idCfg.numBytes];

        /* Prepare command parameters to send FLASH READ ID command. */
        GPMC_writeNandCommandParamsInit(&params);
        params.cmdCycle1 = devConfig->idCfg.cmd;
        params.numColAddrCycles  = 1;
        params.colAddress = 0;
        params.waitTimeout = NAND_DEVICE_RESET_TIMEOUT;
        params.checkReadypin = FALSE;

        status += GPMC_writeNandCommand(object->gpmcHandle, &params);

        if (status == SystemP_SUCCESS)
        {
            GPMC_Transaction trans;
            GPMC_transactionInit(&trans);
            trans.Buf = readId;
            trans.count = devConfig->idCfg.numBytes;
            trans.transType = GPMC_TRANSACTION_TYPE_READ_CMDREG;

            /* Read data from GPMC command REG, since number of bytes to read are less than
             * GPMC FIFO THRESHOLD (64 bytes).
             */
            if(GPMC_nandReadData(object->gpmcHandle, &trans) == SystemP_SUCCESS)
            {
                uint32_t manfID = 0, devID = 0;
                manfID = (uint32_t)readId[0];
                devID = ((uint32_t)readId[1] << 8) | ((uint32_t)readId[2]);
                if(!(manfID == config->attrs->manufacturerId && devID == config->attrs->deviceId))
                {
                    status = SystemP_FAILURE;
                }
            }
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}


static int32_t Flash_nandGpmcCheckBadBlock(Flash_Config *config, uint32_t blockNum)
{
    int32_t status = SystemP_SUCCESS;

    GPMC_nandCmdParams params;
    Flash_DevConfig *devConfig = NULL;
    Flash_NandGpmcObject *object = NULL;
    uint8_t     badBlkMark[2] = {0};

    /* Input parameter validation. */
    if(config != NULL)
    {
        object = (Flash_NandGpmcObject *)(config->object);
        devConfig = (Flash_DevConfig *)config->devConfig;

        if (blockNum >= config->attrs->blockCount)
        {
            status = SystemP_FAILURE;
        }

        if(status == SystemP_SUCCESS)
        {
            /* Prepare command parameters to send FLASH PAGE READ command.
             * Set the flash in READ mode.
             */
            GPMC_writeNandCommandParamsInit(&params);
            params.cmdCycle1 = devConfig->cmdPageLoadCyc1;
            params.cmdCycle2 = devConfig->cmdPageLoadCyc2;
            params.numColAddrCycles  = devConfig->pageColAddrCyc;
            params.numRowAddrCycles = devConfig->pageRowAddrCyc;
            params.colAddress = 0;
            params.rowAddress = (blockNum * (config->attrs->blockSize / config->attrs->pageSize));
            params.waitTimeout = NAND_DEVICE_BUSY_TIMEOUT;

            status += GPMC_writeNandCommand(object->gpmcHandle, &params);

            if(status == SystemP_SUCCESS)
            {
                /* Prepare command parameters to send FLASH RANDOM READ command.
                 * Set the coloumn address to page size to read BB marker.
                 */
                GPMC_writeNandCommandParamsInit(&params);
                params.cmdCycle1 = devConfig->cmdRandomReadCyc1;
                params.cmdCycle2 = devConfig->cmdRandomReadCyc2;
                params.numColAddrCycles  = devConfig->pageColAddrCyc;
                params.colAddress =object->attrs.bbOffset;
                params.waitTimeout = NAND_DEVICE_BUSY_TIMEOUT;
                params.checkReadypin =FALSE;

                status += GPMC_writeNandCommand(object->gpmcHandle, &params);
            }
        }

        if(status == SystemP_SUCCESS)
        {
            GPMC_Transaction trans;
            GPMC_transactionInit(&trans);
            trans.Buf = (void*)badBlkMark;
            trans.count = NAND_BAD_BLOCK_MARKER_LENGTH;
            trans.transType = GPMC_TRANSACTION_TYPE_READ_CMDREG;

            /* Read data from GPMC command REG, since number of bytes to read are less than
             * GPMC FIFO THRESHOLD (64 bytes).
             */
            status += GPMC_nandReadData(object->gpmcHandle, &trans);

            if(badBlkMark[0] != NAND_BLK_GOOD_MARK && badBlkMark[1] != NAND_BLK_GOOD_MARK)
            {
                status = SystemP_FAILURE;
            }
        }

    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

static int32_t Flash_nandGpmcConfigureEccBCH(Flash_Config *config)
{
    int32_t status = SystemP_SUCCESS;

    /* For 8bit ECC correction level, 13 bytes or 26 nibbles are used in ECC value per sector.
     * For 512 bytes sector, 104 bit remainder (26 nibbles) is generated in BCH
     * syndrome polynomial.
     */
    uint32_t lengthProtected = NAND_ECC_BCH_8BIT_NIBBLECNT;
    /* 1byte or 2 nibbles are unused in ECC value per sector. This is done to match ROM validation. */
    uint32_t lengthUnused = NAND_ECC_BCH_8BIT_UNUSED_NIBBLECNT;

    /* Input parameter validation. */
    if(config != NULL)
    {
        Flash_NandGpmcObject *object = (Flash_NandGpmcObject *)config->object;

        /* Configure GPMC ECC engine for BCH8 algo.
         * ECC engine is configured to compute BCH syndrome polynomial for all the sectors in a page.
         */
        status +=GPMC_eccEngineBCHConfig(object->gpmcHandle, object->attrs.eccSteps - 1);

        if(status == SystemP_SUCCESS)
        {
            /* Set length of used bytes used in the ECC value.*/
            status += GPMC_eccValueSizeSet(object->gpmcHandle, GPMC_ECC_SIZE_0, lengthProtected);
            /* Set length of unused bytes used in the ECC value. */
            status += GPMC_eccValueSizeSet(object->gpmcHandle, GPMC_ECC_SIZE_1, lengthUnused);
        }

        if(status == SystemP_SUCCESS)
        {
            /* Configure ELM engine for BCH8 algo. ECC steps correspond to number of 512 bytes
             * sectors in a page. ECC is computed for each 512 bytes and ERROR correction
             * is also done for each 512 bytes.
             */
            status += GPMC_eccBchConfigureElm(object->gpmcHandle, object->attrs.eccSteps);
        }

        if(status == SystemP_SUCCESS)
        {
            /* Enable ECC engine. */
            status += GPMC_eccEngineEnable(object->gpmcHandle);
        }

    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

static int32_t Flash_nandGpmcOpen(Flash_Config *config, Flash_Params *params)
{

    int32_t status = SystemP_SUCCESS;
    Flash_NandGpmcObject *object = (Flash_NandGpmcObject *)config->object;
    Flash_Attrs *attrs = config->attrs;

    object->gpmcHandle = GPMC_getHandle(attrs->driverInstance);

    if(object->gpmcHandle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        /* Set device type for GPMC peripherial. NANDLIKE or NORLIKE*/
        status +=GPMC_setDeviceType(object->gpmcHandle);

        if(status == SystemP_SUCCESS)
        {
            /* Set device width.*/
            status += GPMC_setDeviceSize(object->gpmcHandle);
        }

        if(status == SystemP_SUCCESS)
        {
            /* Configure GPMC timing parameters for NAND. */
            status += GPMC_configureTimingParameters(object->gpmcHandle);
        }

        if(status ==  SystemP_SUCCESS )
        {
            object->attrs.bbOffset  = attrs->pageSize + NAND_BAD_BLK_OFFSET;
            object->attrs.eccOffset = attrs->pageSize + NAND_ECC_BCH_8BIT_OOB_OFFSET;
            object->attrs.eccSteps = attrs->pageSize/NAND_SECTOR_SIZE_BYTES;

            /* Check if spare area size is enough to hold ECC.
             * For BCH8, number of ECC bytes * sectors in a page should be less than spare area size.
             */
            if((object->attrs.eccSteps * object->attrs.eccByteCount) + NAND_BAD_BLOCK_MARKER_LENGTH > attrs->spareAreaSize)
            {
                status = SystemP_FAILURE;
            }
            else
            {
                /* Configure GPMC ECC engine only if ECC algo is selected.
                 * ECC algo parameter is selected if internal ECC config is disabled in sysconfig.
                 */
                if( object->attrs.eccAlgo != GPMC_NAND_ECC_ALGO_NONE)
                {
                    /* Only BCH 8 bit supported. */
                    object->attrs.eccByteCount = NAND_ECC_BCH_8BIT_BYTECNT;
                    /* Configure GPMC ECC engine for BCH. */
                    status += Flash_nandGpmcConfigureEccBCH(config);
                }

                if(status == SystemP_SUCCESS)
                {
                    /* Configure GPMC prefetch/post write engine for read and write access. */
                    status += GPMC_configurePrefetchPostWriteEngine(object->gpmcHandle);
                }
            }
        }
        else
        {
            /* Only BCH 8 bit supported. */
            status = SystemP_FAILURE;
        }

        if(status == SystemP_SUCCESS)
        {
            /* Reset flash.*/
            status += Flash_nandGpmcResetMemory(config);

            if(status == SystemP_SUCCESS)
            {
                /* Validate Manufacturer and Device ID. */
                status += Flash_nandGpmcReadId(config);
            }
        }

    }

    return status;
}

static void Flash_nandGpmcClose(Flash_Config *config)
{
    Flash_NandGpmcObject *object = ( Flash_NandGpmcObject *)config->object;

    /* Reset the flash such that other modules can initialise
     * the flash config registers later.
     */

    (void)Flash_nandGpmcResetMemory(config);

    object->gpmcHandle = NULL;
}


static int32_t Flash_nandGpmcBchEccCheckAndCorrect(Flash_Config *config,
                                                  uint8_t    *pEccRead,
                                                  uint8_t    *pData)
{
    int32_t status  = SystemP_SUCCESS;
    uint32_t bchData[NAND_MAX_ECC_WORDS_PER_TRANFS] ={0};
    uint32_t errCount = 0;
    uint32_t errLoc[NAND_ERROR_BIT_PER_SECTOR_MAX] = {0};
    Flash_NandGpmcObject *object = NULL;
    /* BCH8 value is 104 bits wide - from 0 to 103. */
    uint32_t lastEccBitNum = (NAND_ECC_BCH_8BIT_NIBBLECNT * 4U) - 1U;
    uint32_t errBytePos;
    uint8_t errBitMask;

    /* Input parameter validation. */
    if(config != NULL)
    {
        object = (Flash_NandGpmcObject *)config->object;
        uint8_t sectorEccCheck[object->attrs.eccSteps];
        uint8_t pageErrCorrect = 0;

        for(uint32_t sector = 0; sector < object->attrs.eccSteps && status == SystemP_SUCCESS; sector++)
        {
            sectorEccCheck[sector] = FALSE;
            /* Get BCH syndrome polynomial per sector. */
            status += GPMC_eccGetBchSyndromePolynomial(object->gpmcHandle,sector, bchData);

            if(status == SystemP_SUCCESS)
            {
                /* Check if syndrome polynomial computed is non zero. */
                for(int count = 0; count < (object->attrs.eccByteCount - 1 ); count++)
                {
                    if(((uint8_t*)bchData)[count] != 0x00)
                    {
                        sectorEccCheck[sector] = TRUE;
                        break;
                    }
                }
            }

            if(sectorEccCheck[sector])
            {
                sectorEccCheck[sector] = FALSE;
                /* Check whether the sector ECC calculated is zero or is an erased sector. */
                for(int count = 0; count < (object->attrs.eccByteCount - 1 ); count++)
                {
                    if(*(pEccRead + count) != 0xFF)
                    {
                        sectorEccCheck[sector] = TRUE;
                        break;
                    }
                }

                if(sectorEccCheck[sector])
                {
                    /* Fill syndrome polynomial to ELM per sector. */
                    status += GPMC_eccBchFillSyndromeValue(object->gpmcHandle, sector, bchData);

                    if(status == SystemP_SUCCESS)
                    {
                        /* Start processing of syndrome polynomial and move on to fill
                         * next syndrome polynomial for next sector. ELM is set in CONTINUOS mode for
                         * all the sectors.
                         */
                        status += GPMC_eccBchStartErrorProcessing(object->gpmcHandle, sector);
                        pageErrCorrect +=sectorEccCheck[sector];
                    }
                }
            }


            pEccRead += object->attrs.eccByteCount;
        }

        if(status == SystemP_SUCCESS && pageErrCorrect > 0)
        {
            for(uint32_t sector = 0; sector < object->attrs.eccSteps && status == SystemP_SUCCESS; sector++)
            {
                /* Check whether sector require ECC correction. */
                if(sectorEccCheck[sector])
                {
                    /* Check error processing status of ELM engine per sector. */
                    status += GPMC_eccBchCheckErrorProcessingStatus(object->gpmcHandle, sector);

                    if(status ==SystemP_SUCCESS)
                    {
                        /* Get number of errors per sector. */
                        status += GPMC_eccBchSectorGetError(object->gpmcHandle, sector, &errCount, errLoc);

                        if(status == SystemP_SUCCESS)
                        {
                            for(uint32_t count =0; count< errCount; count++)
                            {
                                /* Correct errors in data area per sector. */
                                if (errLoc[count] > lastEccBitNum)
                                {
                                    /* Calulate bit num that has error from start of BCH data block. */
                                    errLoc[count] -= (lastEccBitNum + 1U);

                                    /* Locate the faulty byte and the faulty bit in the byte and then flip it. */
                                    errBytePos = NAND_SECTOR_SIZE_BYTES - (errLoc[count] / 8) -1;
                                    errBitMask = 0x01U << (errLoc[count] % 8U);
                                    pData[errBytePos] ^= errBitMask;
                                }

                            }
                        }
                    }

                }

                pData += NAND_SECTOR_SIZE_BYTES;
            }
        }

    }
    else
    {
        status = SystemP_FAILURE;
    }


    return status;

}

static int32_t Flash_nandGpmcEccCheckAndCorrect(Flash_Config *config,
                                               uint8_t    *pEccRead,
                                               uint8_t    *pData)
{
    int32_t status = SystemP_SUCCESS;
    Flash_NandGpmcObject *object = NULL;

    /* Input parameter validation. */
    if(config != NULL)
    {
        object = (Flash_NandGpmcObject *)(config->object);

        if(object->attrs.eccAlgo == GPMC_NAND_ECC_ALGO_BCH_8BIT)
        {
            status += Flash_nandGpmcBchEccCheckAndCorrect(config, pEccRead, pData);
        }
        else
        {
            /* Other ECC algos not supported. */
            status = SystemP_FAILURE;
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

static int32_t Flash_nandGpmcReadPage(Flash_Config *config, uint32_t blockNum,
                                uint32_t    pageNum,
                                uint8_t    *buf)
{
    int32_t status = SystemP_SUCCESS;
    Flash_DevConfig *devConfig = NULL;
    Flash_NandGpmcObject *object = NULL;
    GPMC_Transaction  trans;
    GPMC_nandCmdParams params;

    /* Input parameter validation. */
    if(config != NULL)
    {
        object = (Flash_NandGpmcObject *)(config->object);
        devConfig = (Flash_DevConfig *)config->devConfig;
        uint8_t  *pEccData = object->eccMemScratch;

        if(object->attrs.eccAlgo != GPMC_NAND_ECC_ALGO_NONE)
        {
            /* Enable GPMC ECC engine and clear ECC result register. */
            status += GPMC_eccEngineEnable(object->gpmcHandle);
            GPMC_eccResultRegisterClear(object->gpmcHandle);
        }

        if(status == SystemP_SUCCESS)
        {
            /* Prepare command parameters to send FLASH PAGE READ command. */
            GPMC_writeNandCommandParamsInit(&params);
            params.cmdCycle1 = devConfig->cmdPageLoadCyc1;
            params.cmdCycle2 = devConfig->cmdPageLoadCyc2;
            params.numColAddrCycles  = devConfig->pageColAddrCyc;
            params.numRowAddrCycles = devConfig->pageRowAddrCyc;
            params.colAddress = 0;
            params.rowAddress = (blockNum * (config->attrs->blockSize / config->attrs->pageSize)) + pageNum;
            params.waitTimeout = NAND_DEVICE_BUSY_TIMEOUT;

            status += GPMC_writeNandCommand(object->gpmcHandle, &params);
        }

        if(status == SystemP_SUCCESS)
        {
            GPMC_transactionInit(&trans);
            trans.Buf = buf;
            trans.count = config->attrs->pageSize;
            trans.transType = GPMC_TRANSACTION_TYPE_READ;
            /* Read DATA either using the DMA or CPU prefetch read.
             * The GPMC ECC engine is confgiured to calculate BCH syndrome polynomial
             * for all sectors. A complete read of page data computes the BCH syndrome
             * polynomial for all sectors in a single read cycle.
             */
            status += GPMC_nandReadData(object->gpmcHandle, &trans);

            /* Avoid spare area reading and ECC correction if no ECC algo is selected.
             * ECC algo parameter is selected if internal ECC config is disabled in sysconfig.
             */
            if(status == SystemP_SUCCESS && object->attrs.eccAlgo != GPMC_NAND_ECC_ALGO_NONE)
            {
                /* Prepare command parameters to send FLASH RANDOM READ command. */
                GPMC_writeNandCommandParamsInit(&params);
                params.cmdCycle1 = devConfig->cmdRandomReadCyc1;
                params.cmdCycle2 = devConfig->cmdRandomReadCyc2;
                params.numColAddrCycles  = devConfig->pageColAddrCyc;
                params.colAddress = object->attrs.eccOffset;
                params.waitTimeout = NAND_DEVICE_BUSY_TIMEOUT;
                params.checkReadypin =FALSE;

                status += GPMC_writeNandCommand(object->gpmcHandle, &params);

                if(status == SystemP_SUCCESS)
                {
                    /* Read ECC data from the Spare area using CPU prefetch read since
                     * number ECC spare area bytes to read are less than DMA copy min bytes.
                     * Read ECC for the sectors in one cycle.
                     */
                    GPMC_transactionInit(&trans);
                    trans.Buf = pEccData;
                    trans.count = object->attrs.eccByteCount * object->attrs.eccSteps;
                    trans.transType = GPMC_TRANSACTION_TYPE_READ;
                    status += GPMC_nandReadData(object->gpmcHandle, &trans);

                    /* Correct page DATA using ELM module. */
                    status = Flash_nandGpmcEccCheckAndCorrect(config, pEccData, buf);
                }

            }
        }

    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

static int32_t Flash_nandGpmcRead(Flash_Config *config, uint32_t offset, uint8_t *buf ,uint32_t len)
{
    int32_t status = SystemP_SUCCESS;

    /* Input parameter validation. */
    if(config != NULL)
    {
        Flash_NandGpmcObject *object = (Flash_NandGpmcObject *)config->object;
        uint32_t bytesLeft = 0;
        uint32_t numBytesCopy = 0;
        uint32_t totalLength = len;
        uint8_t  *destAddr = buf;

        if(offset + len > config->attrs->flashSize)
        {
            status = SystemP_FAILURE;
        }

        if(status == SystemP_SUCCESS)
        {
            uint32_t blockNum = (offset / config->attrs->blockSize);
            uint32_t pageNum = (offset - (blockNum * config->attrs->blockSize)) / config->attrs->pageSize;
            uint32_t pageOffset = offset % config->attrs->pageSize;

            /* Check bad block. */
            status += Flash_nandGpmcCheckBadBlock(config, blockNum);

            if(status == SystemP_SUCCESS)
            {
                /* Perform buffered read if read offset is not at start of a page.
                 * Use buffered read for ease in ECC correction since ECC is computed per
                 * 512 bytes sector.
                 */
                if(pageOffset > 0)
                {
                    status += Flash_nandGpmcReadPage(config,blockNum,pageNum,object->dataMemScratch);

                    if(status == SystemP_SUCCESS)
                    {
                        bytesLeft = config->attrs->pageSize - pageOffset;
                        numBytesCopy = (bytesLeft >= totalLength) ? totalLength : bytesLeft;
                        memcpy((void*)destAddr, (void*)(object->dataMemScratch + pageOffset), numBytesCopy);
                        destAddr += numBytesCopy;
                        totalLength -=numBytesCopy;
                        pageNum++;
                    }
                }
            }

            if(status == SystemP_SUCCESS)
            {
                while(totalLength > 0)
                {
                    if(pageNum >= config->attrs->pageCount)
                    {
                        blockNum++;
                        pageNum = 0;
                        /* Check bad block for block number increment. */
                        status += Flash_nandGpmcCheckBadBlock(config, blockNum);
                        if(status != SystemP_SUCCESS)
                        {
                            break;
                        }
                    }

                    if(totalLength >= config->attrs->pageSize)
                    {
                        /* Perform full page read. */
                        numBytesCopy = config->attrs->pageSize;
                        status += Flash_nandGpmcReadPage(config,blockNum,pageNum,destAddr);

                        if(status != SystemP_SUCCESS)
                        {
                            break;
                        }
                    }
                    else
                    {
                        /* Perform buffered read if number of bytes left are less than page size. */
                        numBytesCopy = totalLength;
                        status += Flash_nandGpmcReadPage(config,blockNum,pageNum,object->dataMemScratch);

                        if(status == SystemP_SUCCESS)
                        {
                            memcpy((void*)destAddr, (void*)object->dataMemScratch, numBytesCopy);
                        }
                        else
                        {
                            break;
                        }
                    }

                    totalLength -= numBytesCopy;
                    destAddr += numBytesCopy;
                    pageNum++;
                }
            }

        }

    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

static int32_t Flash_nandGpmcDeviceStatus(Flash_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    Flash_DevConfig *devConfig = NULL;
    Flash_NandGpmcObject *object = NULL;
    GPMC_nandCmdParams params;
    GPMC_Transaction  trans;
    uint8_t nandDeviceStatus;

    /* Input parameter validation. */
    if(config != NULL)
    {
        object = (Flash_NandGpmcObject *)(config->object);
        devConfig = (Flash_DevConfig *)config->devConfig;

        /* Prepare command parameters to send FLASH READ STATUS command. */
        GPMC_writeNandCommandParamsInit(&params);
        params.cmdCycle1 = devConfig->cmdReadStatus;
        params.checkReadypin =FALSE;
        params.waitTimeout = NAND_DEVICE_BUSY_TIMEOUT;

        status += GPMC_writeNandCommand(object->gpmcHandle, &params);

        if(status == SystemP_SUCCESS)
        {
            GPMC_transactionInit(&trans);
            trans.Buf = (void*)&nandDeviceStatus;
            trans.count = sizeof(nandDeviceStatus);
            trans.transType = GPMC_TRANSACTION_TYPE_READ_CMDREG;
            /* Read data from GPMC command reg since numbers bytes to read are less than
             * GPMC FIFO THRESHOLD.
             */
            status += GPMC_nandReadData(object->gpmcHandle, &trans);

            if(status == SystemP_SUCCESS)
            {
                /* Check for FAIL bit for last flash command operation. */
                if(nandDeviceStatus & NAND_READ_STATUS_FAIL_MASK)
                {
                    status = SystemP_FAILURE;
                }
                /* Check for DEV READY bit in status. */
                if(!(nandDeviceStatus & NAND_READ_STATUS_DEVRDY_MASK))
                {
                    status = SystemP_FAILURE;
                }

            }
        }

    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

static int32_t Flash_nandGpmcErase (Flash_Config *config, uint32_t blkNum)
{
    int32_t status = SystemP_SUCCESS;
    GPMC_nandCmdParams params;
    Flash_DevConfig *devConfig = NULL;
    Flash_NandGpmcObject *object = NULL;

    /* Input parameter validation. */
    if(config != NULL)
    {
        object = (Flash_NandGpmcObject *)(config->object);
        devConfig = (Flash_DevConfig *)config->devConfig;

        if (blkNum >= config->attrs->blockCount)
        {
            status = SystemP_FAILURE;
        }

        /* Disable Write protect. */
        status += GPMC_disableFlashWriteProtect(object->gpmcHandle);

        if(status == SystemP_SUCCESS)
        {
            /* Check Bad block. */
            status += Flash_nandGpmcCheckBadBlock(config, blkNum);
        }

        if(status == SystemP_SUCCESS)
        {
            /* Prepare command parameters to send FLASH ERASE command. */
            GPMC_writeNandCommandParamsInit(&params);
            params.cmdCycle1 = devConfig->eraseCfg.cmdBlockErase;
            params.cmdCycle2 = devConfig->eraseCfg.cmdBlockEraseCyc2;
            params.numColAddrCycles  = devConfig->pageColAddrCyc;
            params.numRowAddrCycles = devConfig->pageRowAddrCyc;
            params.rowAddress = (blkNum * (config->attrs->blockSize / config->attrs->pageSize));
            params.waitTimeout = NAND_DEVICE_BUSY_TIMEOUT;

            status += GPMC_writeNandCommand(object->gpmcHandle, &params);
        }

        if(status == SystemP_SUCCESS)
        {
            /* Check device status for Erase failure. */
            status += Flash_nandGpmcDeviceStatus(config);
        }

        /* Enable write protect. */
        status += GPMC_enableFlashWriteProtect(object->gpmcHandle);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}


static int32_t Flash_nandGpmcEccCalculate(Flash_Config *config, uint8_t *pEccData)
{
    int32_t status = SystemP_SUCCESS;
    Flash_NandGpmcObject *object = NULL;

    /* Input parameter validation. */
    if(config != NULL)
    {
        object = (Flash_NandGpmcObject *)(config->object);

        if(object->attrs.eccAlgo == GPMC_NAND_ECC_ALGO_BCH_8BIT)
        {
            for(uint32_t count = 0; count < object->attrs.eccSteps; count++)
            {
                /* Compute BCH syndrome polynomial per sector.*/
                status += GPMC_eccCalculateBchSyndromePolynomial(object->gpmcHandle, pEccData, count);
                pEccData += object->attrs.eccByteCount;
            }
        }
        else
        {
            /* Other ECC algos not supported. */
            status = SystemP_FAILURE;
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

static int32_t Flash_nandGpmcWritePage(Flash_Config *config,
                                      uint32_t    blockNum,
                                      uint32_t    pageNum,
                                      uint8_t    *buf)
{
    int32_t status = SystemP_SUCCESS;
    Flash_DevConfig *devConfig = NULL;
    Flash_NandGpmcObject *object = NULL;
    GPMC_Transaction  trans;
    GPMC_nandCmdParams params;

    /* Input parameter validation. */
    if(config != NULL)
    {
        object = (Flash_NandGpmcObject *)(config->object);
        devConfig = (Flash_DevConfig *)config->devConfig;
        uint8_t *pEccData = object->eccMemScratch;

        if(object->attrs.eccAlgo != GPMC_NAND_ECC_ALGO_NONE)
        {
            /* Enable GPMC ECC engine and clear ECC result register. */
            status += GPMC_eccEngineEnable(object->gpmcHandle);
            GPMC_eccResultRegisterClear(object->gpmcHandle);
        }

        if( status == SystemP_SUCCESS)
        {
            /* Prepare command parameters to send FLASH PAGE PROG command. */
            GPMC_writeNandCommandParamsInit(&params);
            params.cmdCycle1 = devConfig->cmdPageProgCyc1;
            params.numColAddrCycles  = devConfig->pageColAddrCyc;
            params.numRowAddrCycles = devConfig->pageRowAddrCyc;
            params.colAddress = 0;
            params.rowAddress = (blockNum * (config->attrs->blockSize / config->attrs->pageSize)) + pageNum;
            params.waitTimeout = NAND_DEVICE_BUSY_TIMEOUT;
            params.checkReadypin = GPMC_CMD_INVALID;

            status += GPMC_writeNandCommand(object->gpmcHandle, &params);
        }

        if(status == SystemP_SUCCESS)
        {
            GPMC_transactionInit(&trans);
            trans.Buf = buf;
            trans.count = config->attrs->pageSize;;
            trans.transType = GPMC_TRANSACTION_TYPE_WRITE;

            /* Write data using CPU post write.*/
            status += GPMC_nandWriteData(object->gpmcHandle, &trans);

            /* Avoid spare area writing and ECC correction if no ECC algo is selected.
             * ECC algo parameter is selected if internal ECC config is disabled in sysconfig.
             */
            if(status == SystemP_SUCCESS && object->attrs.eccAlgo != GPMC_NAND_ECC_ALGO_NONE)
            {
                /* Compute ECC for all sectors in a page. GPMC ECC engine is configured
                 * to compute BCH syndrome polynomials for all sectors in single cycle.
                 */
                status += Flash_nandGpmcEccCalculate(config, pEccData);

                if(status == SystemP_SUCCESS)
                {
                    /* Prepare command parameters to send FLASH RANDOM INPUT command. */
                    GPMC_writeNandCommandParamsInit(&params);

                    params.cmdCycle1 = devConfig->cmdRandomInput;
                    params.numColAddrCycles  = devConfig->pageColAddrCyc;
                    params.colAddress = object->attrs.eccOffset;
                    params.waitTimeout = NAND_DEVICE_BUSY_TIMEOUT;
                    params.checkReadypin =FALSE;

                    status += GPMC_writeNandCommand(object->gpmcHandle, &params);

                    if(status == SystemP_SUCCESS)
                    {
                        GPMC_transactionInit(&trans);
                        trans.Buf = (void*)object->eccMemScratch;
                        trans.count = object->attrs.eccByteCount *
                                    (config->attrs->pageSize/NAND_SECTOR_SIZE_BYTES);
                        trans.transType = GPMC_TRANSACTION_TYPE_WRITE;
                        /* Write ECC data to spare area using CPU post write. */
                        status += GPMC_nandWriteData(object->gpmcHandle, &trans);
                    }

                    if(status == SystemP_SUCCESS)
                    {
                        /* Complete flash page program. */
                        GPMC_writeNandCommandParamsInit(&params);
                        params.cmdCycle2 = devConfig->cmdPageProgCyc2;
                        params.waitTimeout = NAND_DEVICE_BUSY_TIMEOUT;
                        status += GPMC_writeNandCommand(object->gpmcHandle, &params);
                    }
                }

            }
            /* Check device status for write failure. */
            status += Flash_nandGpmcDeviceStatus(config);

        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

static int32_t Flash_nandGpmcWrite (Flash_Config *config, uint32_t offset,
                                    uint8_t *buf, uint32_t len)
{
    int32_t status = SystemP_SUCCESS;

    /* Input parameter validation. */
    if(config != NULL)
    {
        Flash_NandGpmcObject *object = (Flash_NandGpmcObject *)config->object;
        uint32_t bytesLeft = 0;
        uint32_t numBytesCopy = 0;
        uint32_t totalLength = len;
        uint8_t  *srcAddr = buf;

        if(offset + len > config->attrs->flashSize)
        {
            status = SystemP_FAILURE;
        }
        /* Disable Write protect. */
        status += GPMC_disableFlashWriteProtect(object->gpmcHandle);

        if(status == SystemP_SUCCESS)
        {
            uint32_t blockNum = (offset / config->attrs->blockSize);
            uint32_t pageNum = (offset - (blockNum * config->attrs->blockSize)) / config->attrs->pageSize;
            uint32_t pageOffset = offset % config->attrs->pageSize;
            /* Check Bad block. */
            status += Flash_nandGpmcCheckBadBlock(config, blockNum);

            if(status == SystemP_SUCCESS)
            {
                /* Perform buffered write if write offset is not at start of a page.
                 * Use buffered write for ease in ECC computation since ECC is computed per
                 * 512 bytes sector.
                 */
                if(pageOffset > 0)
                {
                    bytesLeft = config->attrs->pageSize - pageOffset;
                    numBytesCopy = (bytesLeft >= totalLength) ? totalLength : bytesLeft;

                    memset((void *)object->dataMemScratch, 0xFF, config->attrs->pageSize);
                    memcpy((void *)(object->dataMemScratch + pageOffset), (void *)srcAddr, numBytesCopy);

                    status += Flash_nandGpmcWritePage(config,blockNum,pageNum,object->dataMemScratch);

                    if(status == SystemP_SUCCESS)
                    {
                        srcAddr += numBytesCopy;
                        totalLength -=numBytesCopy;
                        pageNum++;
                    }
                }
            }

            if(status == SystemP_SUCCESS)
            {
                while(totalLength > 0)
                {
                    if(pageNum >= config->attrs->pageCount)
                    {
                        blockNum++;
                        pageNum = 0;
                        /* Check bad block for block number increament. */
                        status += Flash_nandGpmcCheckBadBlock(config, blockNum);
                        if(status != SystemP_SUCCESS)
                        {
                            break;
                        }
                    }

                    if(totalLength >= config->attrs->pageSize)
                    {
                        /* Perform full page write. */
                        numBytesCopy = config->attrs->pageSize;
                        status += Flash_nandGpmcWritePage(config,blockNum,pageNum,srcAddr);

                        if(status != SystemP_SUCCESS)
                        {
                            break;
                        }
                    }
                    else
                    {
                        /* Perform buffered write if remaining bytes are less than page size. */
                        numBytesCopy = totalLength;

                        memset((void *)object->dataMemScratch, 0xFF, config->attrs->pageSize);
                        memcpy((void *)(object->dataMemScratch), (void *)srcAddr, numBytesCopy);

                        status += Flash_nandGpmcWritePage(config,blockNum,pageNum,object->dataMemScratch);

                        if(status != SystemP_SUCCESS)
                        {
                            break;
                        }
                    }

                    totalLength -= numBytesCopy;
                    srcAddr += numBytesCopy;
                    pageNum++;
                }
            }
        }
        /* Enable write protect. */
        status += GPMC_enableFlashWriteProtect(object->gpmcHandle);

    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

