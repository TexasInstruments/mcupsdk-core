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

#include <kernel/dpl/DebugP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <board/flash/sfdp/nor_spi_sfdp.h>
#include <string.h>

/* Cypress flashes have hybrid sector configuration, this puts the first 256KB of the flash in
    hybrid sector mode. This will make block erases to first 256 KB fail. Not to lose generality,
    choosing the offset to be at 512 KB */
#define APP_OSPI_FLASH_OFFSET  (512*1024U)

#define APP_OSPI_DATA_SIZE (256)
uint8_t gOspiTxBuf[APP_OSPI_DATA_SIZE];
/* read buffer MUST be cache line aligned when using DMA, we aligned to 128B though 32B is enough */
uint8_t gOspiRxBuf[APP_OSPI_DATA_SIZE] __attribute__((aligned(128U)));

NorSpi_SfdpHeader gSfdpHeader;
NorSpi_SfdpParamHeader gParamHeaders[NOR_SPI_SFDP_NPH_MAX];
NorSpi_SfdpParamHeader *gBfptHeader;

NorSpi_SfdpBasicFlashParamTable gBfpt;
NorSpi_SfdpSectorMapParamTable gSmpt;
NorSpi_SfdpSCCRParamTable gSccr;
NorSpi_SfdpProfile1ParamTable gXpt1;
NorSpi_Sfdp4ByteAddressingParamTable g4bait;

NorSpi_SfdpGenericDefines gNorSpiDevDefines;

void ospi_flash_diag_test_fill_buffers(void);
int32_t ospi_flash_diag_test_compare_buffers(void);
int32_t ospi_flash_diag_print_sfdp(OSPI_Handle handle);
void ospi_flash_diag_print_defines_json(NorSpi_SfdpGenericDefines *norSpiDefines);
void ospi_flash_diag_print_defines(NorSpi_SfdpGenericDefines *norSpiDefines);

void ospi_flash_diag_main(void *args)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t manfId, deviceId;

    /* Open OSPI Driver, among others */
    Drivers_open();

    DebugP_log("[OSPI Flash Diagnostic Test] Starting ...\r\n");

    OSPI_Handle ospiHandle = OSPI_getHandle(CONFIG_OSPI0);

    /* Zero init the dev defines struct */
    memset(&gNorSpiDevDefines, 0, sizeof(gNorSpiDevDefines));

    OSPI_norFlashSetCmds(0x03, 0x02, 0xD8);

    /* Initialize the flash device in 1s1s1s mode */
    OSPI_norFlashInit1s1s1s(ospiHandle);

    /* Read ID */
    status = OSPI_norFlashReadId(ospiHandle, &manfId, &deviceId);

    if(SystemP_SUCCESS == status)
    {
        DebugP_log("[OSPI Flash Diagnostic Test] Flash Manufacturer ID : 0x%X\r\n", manfId);
        DebugP_log("[OSPI Flash Diagnostic Test] Flash Device ID       : 0x%X\r\n", deviceId);

        gNorSpiDevDefines.manfId = manfId;
        gNorSpiDevDefines.deviceId = deviceId;
    }

    /* Fill buffers with known data,
     * find block number from offset,
     * erase block, write the data, read back from a specific offset
     * and finally compare the results.
     */

    if( SystemP_SUCCESS == status)
    {
        ospi_flash_diag_test_fill_buffers();

        uint32_t offset  = APP_OSPI_FLASH_OFFSET;

        DebugP_log("[OSPI Flash Diagnostic Test] Executing Flash Erase on first block...\r\n");
        status = OSPI_norFlashErase(ospiHandle, offset);
        if(SystemP_SUCCESS == status)
        {
            DebugP_log("[OSPI Flash Diagnostic Test] Done !!!\r\n");
        }
        else
        {
            DebugP_log("[OSPI Flash Diagnostic Test] Erase Failed !!!\r\n");
        }
        DebugP_log("[OSPI Flash Diagnostic Test] Performing Write-Read Test...\r\n");
        status = OSPI_norFlashWrite(ospiHandle, offset, gOspiTxBuf, APP_OSPI_DATA_SIZE);
        if(SystemP_SUCCESS != status)
        {
            DebugP_log("[OSPI Flash Diagnostic Test] Wtite Failed !!!\r\n");
        }
        else
        {
            /* Nothing */
        }
        OSPI_norFlashRead(ospiHandle, offset, gOspiRxBuf, APP_OSPI_DATA_SIZE);

        status |= ospi_flash_diag_test_compare_buffers();

        if(SystemP_SUCCESS == status)
        {
            DebugP_log("[OSPI Flash Diagnostic Test] Write-Read Test Passed!\r\n");
        }
    }

    if(SystemP_SUCCESS == status)
    {
        /* Read the SFDP table and print flash details */
        status = ospi_flash_diag_print_sfdp(ospiHandle);
    }

    if(SystemP_SUCCESS == status)
    {
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();
}

void ospi_flash_diag_test_fill_buffers(void)
{
    uint32_t i;

    for(i = 0U; i < APP_OSPI_DATA_SIZE; i++)
    {
        gOspiTxBuf[i] = i;
        gOspiRxBuf[i] = 0U;
    }
}

int32_t ospi_flash_diag_test_compare_buffers(void)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t i;

    for(i = 0U; i < APP_OSPI_DATA_SIZE; i++)
    {
        if(gOspiTxBuf[i] != gOspiRxBuf[i])
        {
            status = SystemP_FAILURE;
            DebugP_logError("OSPI read data mismatch !!!\r\n");
            break;
        }
    }
    return status;
}

int32_t ospi_flash_diag_print_sfdp(OSPI_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t ptp = 0xFFFFFFFFU;

    /* Read the SFDP header */
    status = OSPI_norFlashReadSfdp(handle, NOR_SPI_SFDP_HEADER_START_OFFSET, (void *)&gSfdpHeader, sizeof(NorSpi_SfdpHeader));

    gBfptHeader = &gSfdpHeader.firstParamHeader;

    /* Check if the signature is read correctly */
    if(gSfdpHeader.sfdpHeader.signature != NOR_SPI_SFDP_SIGNATURE)
    {
        DebugP_log("[QSPI Flash Diagnostic Test] Error in reading SFDP Table or SFDP not supported by Flash !!!\r\n");
    }
    else
    {
        /* Print SFDP basic information */
        DebugP_log("[QSPI Flash Diagnostic Test] SFDP Information : \r\n");
        DebugP_log("================================================\r\n");
        DebugP_log("                      SFDP                      \r\n");
        DebugP_log("================================================\r\n");

        DebugP_log("SFDP Major Revision                       : 0x%X\r\n", gSfdpHeader.sfdpHeader.majorRev);
        DebugP_log("SFDP Minor Revision                       : 0x%X\r\n", gSfdpHeader.sfdpHeader.minorRev);
        DebugP_log("Number of Parameter Headers in this Table : %u\r\n\r\n", gSfdpHeader.sfdpHeader.numParamHeaders + 1);

        /* First parameter header is already read, read the rest of parameter headers if they exist */
        uint32_t nph = gSfdpHeader.sfdpHeader.numParamHeaders;
        uint32_t i;

        if(nph > 0)
        {
            status = OSPI_norFlashReadSfdp(handle, NOR_SPI_SFDP_SECOND_PARAM_HEADER_OFFSET, (void *)&gParamHeaders, nph * sizeof(NorSpi_SfdpParamHeader));
        }

        if(status == SystemP_SUCCESS)
        {
            /* Print the types of parameter tables present */
            DebugP_log("Types of Additional Parameter Tables in this flash\r\n");
            DebugP_log("---------------------------------------------------\r\n");

            for(i = 0; i < nph; i++)
            {
                NorSpi_SfdpParamHeader *paramHeader = &gParamHeaders[i];

                uint32_t paramID = (uint32_t)((uint32_t)(paramHeader->paramIdMsb << 8U) | (uint32_t)(paramHeader->paramIdLsb));

                if(paramID != NOR_SPI_SFDP_BASIC_PARAM_TABLE_ID)
                {
                    char *paramName = NorSpi_Sfdp_getParameterTableName(paramID);

                    if(paramName == NULL)
                    {
                        DebugP_log("Unsupported Parameter Table type!!! - 0x%X\r\n", paramID);
                    }
                    else
                    {
                        DebugP_log("%s\r\n", paramName);
                    }
                }
                else
                {
                    /* Update the gBfpt pointer to the latest version */
                    if((paramHeader->paramTableMajorRev == NOR_SPI_SFDP_JESD216_MAJOR) &&
                        ((paramHeader->paramTableMinorRev > gBfptHeader->paramTableMinorRev) ||
                            ((paramHeader->paramTableMinorRev > gBfptHeader->paramTableMinorRev) &&
                                (paramHeader->paramTableLength > gBfptHeader->paramTableLength))))
                    {
                        gBfptHeader = paramHeader;
                    }
                }
            }
        }

        /* Read the Basic Flash Parameter Table (BFPT) */
        ptp = NorSpi_Sfdp_getPtp(gBfptHeader);

        status = OSPI_norFlashReadSfdp(handle, ptp, (void *)&gBfpt, gBfptHeader->paramTableLength * sizeof(uint32_t));

        /* Parse BFPT */
        if(status == SystemP_SUCCESS)
        {
            status = NorSpi_Sfdp_parseBfpt(&gBfpt, &gNorSpiDevDefines, gBfptHeader->paramTableLength);
        }

        /* Parse other parameter tables */
        for(i = 0; i < nph; i++)
        {
            NorSpi_SfdpParamHeader *paramHeader = &gParamHeaders[i];

            uint32_t paramID = (uint32_t)((uint32_t)(paramHeader->paramIdMsb << 8U) | (uint32_t)(paramHeader->paramIdLsb));

            ptp = NorSpi_Sfdp_getPtp(paramHeader);

            switch(paramID)
            {
                case NOR_SPI_SFDP_4BYTE_ADDR_INSTR_TABLE_ID:
                    status = OSPI_norFlashReadSfdp(handle, ptp, (void *)&g4bait, paramHeader->paramTableLength * sizeof(uint32_t));
                    status = NorSpi_Sfdp_parse4bait(&g4bait, &gNorSpiDevDefines, paramHeader->paramTableLength);
                    break;

                case NOR_SPI_SFDP_SECTOR_MAP_TABLE_ID:
                    status = OSPI_norFlashReadSfdp(handle, ptp, (void *)&gSmpt, paramHeader->paramTableLength * sizeof(uint32_t));
                    status = NorSpi_Sfdp_parseSmpt(&gSmpt, &gNorSpiDevDefines, paramHeader->paramTableLength);
                    break;

                case NOR_SPI_SFDP_SCCR_TABLE_ID:
                    status = OSPI_norFlashReadSfdp(handle, ptp, (void *)&gSccr, paramHeader->paramTableLength * sizeof(uint32_t));
                    status = NorSpi_Sfdp_parseSccr(&gSccr, &gNorSpiDevDefines, paramHeader->paramTableLength);
                    break;

                case NOR_SPI_SFDP_PROFILE_TABLE_ID:
                    status = OSPI_norFlashReadSfdp(handle, ptp, (void *)&gXpt1, paramHeader->paramTableLength * sizeof(uint32_t));
                    status = NorSpi_Sfdp_parseXpt1(&gXpt1, &gNorSpiDevDefines, paramHeader->paramTableLength);
                    break;

                default:
                    /* Parsing not yet supported */
                    DebugP_log("\r\n");
                    char *paramName = NorSpi_Sfdp_getParameterTableName(paramID);
                    if(paramName != NULL)
                    {
                        DebugP_log("Parsing of %s table not yet supported. \r\n", paramName);
                    }
                    break;
            }
        }

        /* Print the final config */
        if(status == SystemP_SUCCESS)
        {
            ospi_flash_diag_print_defines(&gNorSpiDevDefines);
            ospi_flash_diag_print_defines_json(&gNorSpiDevDefines);
        }
    }

    return status;
}

void ospi_flash_diag_print_defines(NorSpi_SfdpGenericDefines *norSpiDefines)
{
}

void ospi_flash_diag_print_defines_json(NorSpi_SfdpGenericDefines *norSpiDefines)
{
    if(norSpiDefines != NULL)
    {
        DebugP_log("JSON Data for the flash :\r\n");
        DebugP_log("\r\n");

        DebugP_log("{\r\n");
        DebugP_log("\r\n");
        DebugP_log("\t\"flashSize\": %d,\r\n", norSpiDefines->flashSize);
        DebugP_log("\t\"flashPageSize\": %d,\r\n", norSpiDefines->pageSize);
        DebugP_log("\t\"flashManfId\": \"0x%02X\",\r\n", norSpiDefines->manfId);
        DebugP_log("\t\"flashDeviceId\": \"0x%04X\",\r\n", norSpiDefines->deviceId);
        DebugP_log("\t\"flashBlockSize\": %d,\r\n", norSpiDefines->eraseCfg.blockSize);
        DebugP_log("\t\"flashSectorSize\": %d,\r\n", norSpiDefines->eraseCfg.sectorSize);
        DebugP_log("\t\"cmdBlockErase3B\": \"0x%02X\",\r\n", norSpiDefines->eraseCfg.cmdBlockErase3B);
        DebugP_log("\t\"cmdBlockErase4B\": \"0x%02X\",\r\n", norSpiDefines->eraseCfg.cmdBlockErase4B);
        DebugP_log("\t\"cmdSectorErase3B\": \"0x%02X\",\r\n", norSpiDefines->eraseCfg.cmdSectorErase3B);
        DebugP_log("\t\"cmdSectorErase4B\": \"0x%02X\",\r\n", norSpiDefines->eraseCfg.cmdSectorErase4B);
        DebugP_log("\t\"protos\": {\r\n");

        uint32_t protos[] = {
            FLASH_CFG_PROTO_1S_1S_1S,
            FLASH_CFG_PROTO_1S_1S_2S,
            FLASH_CFG_PROTO_1S_1S_4S,
            FLASH_CFG_PROTO_1S_1S_8S,
            FLASH_CFG_PROTO_4S_4S_4S,
            FLASH_CFG_PROTO_4S_4D_4D,
            FLASH_CFG_PROTO_8S_8S_8S,
            FLASH_CFG_PROTO_8D_8D_8D,
        };

        char* protos_json[] = {
            "p111", "p112", "p114", "p118", "p444s", "p444d","p888s", "p888d",
        };

        for(uint32_t i = 0; i < 8; i++)
        {
            FlashCfg_ProtoEnConfig *pCfg = &norSpiDefines->protos[protos[i]];
            if(pCfg->cmdRd != 0)
            {
                DebugP_log("\t\t\"%s\": {\r\n", protos_json[i]);
                DebugP_log("\t\t\t\"isDtr\": %s,\r\n", pCfg->isDtr ? "true" : "false");
                DebugP_log("\t\t\t\"cmdRd\": \"0x%02X\",\r\n", pCfg->cmdRd);
                DebugP_log("\t\t\t\"cmdWr\": \"0x%02X\",\r\n", pCfg->cmdWr);
                DebugP_log("\t\t\t\"modeClksCmd\": %d,\r\n", pCfg->modeClksCmd);
                DebugP_log("\t\t\t\"modeClksRd\": %d,\r\n", pCfg->modeClksRd);
                DebugP_log("\t\t\t\"dummyClksCmd\": %d,\r\n", pCfg->dummyClksCmd);
                DebugP_log("\t\t\t\"dummyClksRd\": %d,\r\n", pCfg->dummyClksRd);
                DebugP_log("\t\t\t\"enableType\": \"%d\",\r\n", pCfg->enableType);
                DebugP_log("\t\t\t\"enableSeq\": \"0x%02X\",\r\n", pCfg->enableSeq);

                if((protos[i] == FLASH_CFG_PROTO_4S_4S_4S) ||
                (protos[i] == FLASH_CFG_PROTO_4S_4D_4D) ||
                (protos[i] == FLASH_CFG_PROTO_8S_8S_8S) ||
                (protos[i] == FLASH_CFG_PROTO_8D_8D_8D))
                {
                    DebugP_log("\t\t\t\"dummyCfg\": {\r\n");
                    DebugP_log("\t\t\t\t\"isAddrReg\": %s,\r\n", pCfg->dummyCfg.isAddrReg ? "true" : "false");
                    DebugP_log("\t\t\t\t\"cmdRegRd\":\"0x%02X\",\r\n", pCfg->dummyCfg.cmdRegRd);
                    DebugP_log("\t\t\t\t\"cmdRegWr\":\"0x%02X\",\r\n", pCfg->dummyCfg.cmdRegWr);
                    DebugP_log("\t\t\t\t\"cfgReg\":\"0x%08X\",\r\n", pCfg->dummyCfg.cfgReg);
                    DebugP_log("\t\t\t\t\"shift\":%d,\r\n", pCfg->dummyCfg.shift);
                    DebugP_log("\t\t\t\t\"mask\":\"0x%02X\",\r\n", pCfg->dummyCfg.mask);
                    DebugP_log("\t\t\t\t\"bitP\":%d\r\n", pCfg->dummyCfg.cfgRegBitP);
                    DebugP_log("\t\t\t},\r\n");
                    DebugP_log("\t\t\t\"protoCfg\": {\r\n");
                    DebugP_log("\t\t\t\t\"isAddrReg\": %s,\r\n", pCfg->protoCfg.isAddrReg ? "true" : "false");
                    DebugP_log("\t\t\t\t\"cmdRegRd\": \"0x%02X\",\r\n", pCfg->protoCfg.cmdRegRd);
                    DebugP_log("\t\t\t\t\"cmdRegWr\": \"0x%02X\",\r\n", pCfg->protoCfg.cmdRegWr);
                    DebugP_log("\t\t\t\t\"cfgReg\": \"0x%08X\",\r\n", pCfg->protoCfg.cfgReg);
                    DebugP_log("\t\t\t\t\"shift\": %d,\r\n", pCfg->protoCfg.shift);
                    DebugP_log("\t\t\t\t\"mask\": \"0x%02X\",\r\n", pCfg->protoCfg.mask);
                    DebugP_log("\t\t\t\t\"bitP\": %d\r\n", pCfg->protoCfg.shift);
                    DebugP_log("\t\t\t},\r\n");
                    DebugP_log("\t\t\t\"strDtrCfg\": {\r\n");
                    DebugP_log("\t\t\t\t\"isAddrReg\": %s,\r\n", pCfg->strDtrCfg.isAddrReg ? "true" : "false");
                    DebugP_log("\t\t\t\t\"cmdRegRd\": \"0x%02X\",\r\n", pCfg->strDtrCfg.cmdRegRd);
                    DebugP_log("\t\t\t\t\"cmdRegWr\": \"0x%02X\",\r\n", pCfg->strDtrCfg.cmdRegWr);
                    DebugP_log("\t\t\t\t\"cfgReg\": \"0x%08X\",\r\n", pCfg->strDtrCfg.cfgReg);
                    DebugP_log("\t\t\t\t\"shift\": %d,\r\n", pCfg->strDtrCfg.shift);
                    DebugP_log("\t\t\t\t\"mask\": \"0x%02X\",\r\n", pCfg->strDtrCfg.mask);
                    DebugP_log("\t\t\t\t\"bitP\": %d\r\n", pCfg->strDtrCfg.shift);
                    DebugP_log("\t\t\t}\r\n");
                }
                else
                {
                    DebugP_log("\t\t\t\"dummyCfg\": null,\r\n");
                    DebugP_log("\t\t\t\"protoCfg\": null,\r\n");
                    DebugP_log("\t\t\t\"strDtrCfg\": null\r\n");
                }
                DebugP_log("\t\t},\r\n");
            }
            else
            {
                DebugP_log("\t\t\"%s\": null,\r\n", protos_json[i]);
            }
        }

        DebugP_log("\t\t\"pCustom\": { \r\n\t\t\t\"fxn\": null\r\n\t\t}\r\n");

        DebugP_log("\t},\r\n");
        DebugP_log("\t\"addrByteSupport\": \"%d\",\r\n", norSpiDefines->addrByteSupport);
        DebugP_log("\t\"fourByteAddrEnSeq\": \"0x%02X\",\r\n", norSpiDefines->fourByteAddrEnSeq);
        DebugP_log("\t\"cmdExtType\": \"%s\",\r\n", norSpiDefines->cmdExtType == 0 ? "REPEAT" : (norSpiDefines->cmdExtType == 1 ? "INVERSE" : "NONE"));
        DebugP_log("\t\"resetType\": \"0x%02X\",\r\n", norSpiDefines->rstType);
        DebugP_log("\t\"deviceBusyType\": \"%d\",\r\n", norSpiDefines->deviceBusyType);
        DebugP_log("\t\"cmdWren\": \"0x%02X\",\r\n", norSpiDefines->cmdWren);
        DebugP_log("\t\"cmdRdsr\": \"0x%02X\",\r\n", norSpiDefines->cmdRdsr);
        DebugP_log("\t\"srWip\":  %d,\r\n", norSpiDefines->srWip);
        DebugP_log("\t\"srWel\":  %d,\r\n", norSpiDefines->srWel);
        DebugP_log("\t\"cmdChipErase\": \"0x%02X\",\r\n", norSpiDefines->cmdChipErase);
        DebugP_log("\t\"rdIdSettings\": {\r\n");
        DebugP_log("\t\t\"cmd\": \"0x%02X\",\r\n", norSpiDefines->idCfg.cmd);
        DebugP_log("\t\t\"numBytes\": %d,\r\n", norSpiDefines->idCfg.numBytes);
        DebugP_log("\t\t\"dummy4\": %d,\r\n", norSpiDefines->idCfg.dummy4);
        DebugP_log("\t\t\"dummy8\": %d\r\n", norSpiDefines->idCfg.dummy8);
        DebugP_log("\t},\r\n");
        DebugP_log("\t\"xspiWipRdCmd\": \"0x%02X\",\r\n", norSpiDefines->xspiWipRdCmd);
        DebugP_log("\t\"xspiWipReg\": \"0x%08X\",\r\n", norSpiDefines->xspiWipReg);
        DebugP_log("\t\"xspiWipBit\": %d,\r\n", norSpiDefines->xspiWipBit);
        DebugP_log("\t\"flashDeviceBusyTimeout\": %d,\r\n", norSpiDefines->flashBusyTimeout);
        DebugP_log("\t\"flashPageProgTimeout\": %d\r\n", norSpiDefines->flashWriteTimeout);
        DebugP_log("}\r\n\r\n");
    }
}
