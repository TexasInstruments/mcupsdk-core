/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
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
#include "psram_ospi.h"

/* Mode Register 0 */
#define OSPI_PSRAM_MR0_ADDRESS            0x00000000U
#define OSPI_PSRAM_MR0_DRIVE_STRENGTH     0x01U       /* Drive Strength                      */
#define OSPI_PSRAM_MR0_READ_LATENCY_CODE  0x04U       /* Read Latency Code                   */
#define OSPI_PSRAM_MR0_RLC_3              0x00U       /* Read Latency Code 3                 */
#define OSPI_PSRAM_MR0_LATENCY_TYPE       0x20U       /* Latency Type                        */

/* Mode Register 1 */
#define OSPI_PSRAM_MR1_ADDRESS            0x00000001U
#define OSPI_PSRAM_MR1_VENDOR_ID_MASK     0x1FU       /* Vendor Identifier                   */

/* Mode Register 2 */
#define OSPI_PSRAM_MR2_ADDRESS            0x00000002U
#define OSPI_PSRAM_MR2_DEVICE_ID_MASK     0x18U       /* Device Identifier                   */

/* Mode Register 4 */
#define OSPI_PSRAM_MR4_ADDRESS            0x00000004U
#define OSPI_PSRAM_MR4_WLC_3              0x80U       /* Write Latency Code 3                */

/* Mode Register 6 */
#define OSPI_PSRAM_MR6_ADDRESS            0x00000006U

/* Mode Register 8 */
#define OSPI_PSRAM_MR8_ADDRESS            0x00000008U
#define OSPI_PSRAM_MR8_BT                 0x04U       /* Burst Type                          */

#define OSPI_PSRAM_RD_CAPTURE_DELAY       8U

static uint8_t gReadBuf[OSPI_FLASH_ATTACK_VECTOR_SIZE] = { 0U };

static int32_t Psram_ospiOpen(Ram_Config *config);
static int32_t Psram_ospiRead(Ram_Config *config, uint32_t ramOffset, uint8_t *pbuf, uint32_t pbufLen);
static int32_t Psram_ospiWrite(Ram_Config *config, uint32_t ramOffset,uint8_t *pbuf, uint32_t pbufLen);
static void Psram_ospiClose(Ram_Config *config);
static int32_t Psram_ospiReadCmd(Ram_Config *config, uint8_t cmd, uint32_t readOffset,uint8_t dummyBits,
                            uint8_t numAddrBytes, uint8_t *rxBuf, uint32_t rxBufLen);
static int32_t Psram_ospiReadId(Ram_Config *config, uint32_t *manufacturerId, uint32_t *deviceId);
static int32_t Psram_ospiReset(Ram_Config *config);
static int32_t Psram_ospiWriteCmd(Ram_Config *config, uint8_t cmd, uint32_t writeOffset, uint8_t *txBuf, uint32_t txBufLen);

Ram_Fxns gPsramOspiFxns = {
    .openFxn = Psram_ospiOpen,
    .closeFxn = Psram_ospiClose,
    .readFxn = Psram_ospiRead,
    .writeFxn = Psram_ospiWrite,
};

uint32_t gProtocolMap[] =
{
    [OSPI_PROTO_1S_1S_1S] = OSPI_NOR_PROTOCOL(1,1,1,0),
    [OSPI_PROTO_1S_1S_2S] = OSPI_NOR_PROTOCOL(1,1,2,0),
    [OSPI_PROTO_1S_1S_4S] = OSPI_NOR_PROTOCOL(1,1,4,0),
    [OSPI_PROTO_1S_1S_8S] = OSPI_NOR_PROTOCOL(1,1,8,0),
    [OSPI_PROTO_4S_4S_4S] = OSPI_NOR_PROTOCOL(4,4,4,0),
    [OSPI_PROTO_4S_4D_4D] = OSPI_NOR_PROTOCOL(4,4,4,1),
    [OSPI_PROTO_8S_8S_8S] = OSPI_NOR_PROTOCOL(8,8,8,0),
    [OSPI_PROTO_8D_8D_8D] = OSPI_NOR_PROTOCOL(8,8,8,1),
};


static int32_t Psram_ospiOpen(Ram_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t txDataMR0 = OSPI_PSRAM_MR0_DRIVE_STRENGTH + OSPI_PSRAM_MR0_READ_LATENCY_CODE + OSPI_PSRAM_MR0_LATENCY_TYPE;
    uint8_t txDataMR4 = OSPI_PSRAM_MR4_WLC_3;
    uint8_t txDataMR8 = OSPI_PSRAM_MR8_BT;

    Ram_Attrs *attrs = config->attrs;
    Ram_OspiPsramObject *obj = (Ram_OspiPsramObject*)(config->object);
    
    obj->ospiHandle = OSPI_getHandle(attrs->driverInstance);

    if(obj->ospiHandle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        const OSPI_Attrs *ospi_attrs = ((OSPI_Config *)obj->ospiHandle)->attrs;

        Psram_ospiReset(config);

        OSPI_configResetPin(obj->ospiHandle, OSPI_RESETPIN_DEDICATED);
        /* Set device size and addressing bytes */
        OSPI_setDeviceSize(obj->ospiHandle, config->attrs->ramSize, config->attrs->ramSize);
        
        OSPI_setProtocol(obj->ospiHandle, gProtocolMap[ospi_attrs->protocol]);

        OSPI_setReadDummyCycles(obj->ospiHandle,config->devConfig->dummyClksRd);

        OSPI_setWriteDummyCycles(obj->ospiHandle,config->devConfig->dummyClksWr);

        OSPI_setCmdDummyCycles(obj->ospiHandle,config->devConfig->dummyClksCmd);
    
        status += Psram_ospiWriteCmd(config,config->devConfig->cmdRegWr,OSPI_PSRAM_MR0_ADDRESS,&txDataMR0,(sizeof(txDataMR0)/sizeof(uint8_t)));
        
        if(status == SystemP_SUCCESS)
        {
            status += Psram_ospiWriteCmd(config,config->devConfig->cmdRegWr,OSPI_PSRAM_MR4_ADDRESS,&txDataMR4,(sizeof(txDataMR4)/sizeof(uint8_t)));
        }
        
        if(status == SystemP_SUCCESS)
        {
            status += Psram_ospiWriteCmd(config,config->devConfig->cmdRegWr,OSPI_PSRAM_MR8_ADDRESS,&txDataMR8,(sizeof(txDataMR8)/sizeof(uint8_t)));
        }

        OSPI_setNumAddrBytes(obj->ospiHandle,4);
        obj->numAddrBytes = 4;
    
    }

    if(status == SystemP_SUCCESS)
    {
        OSPI_setXferOpCodes(obj->ospiHandle, config->devConfig->cmdRd, config->devConfig->cmdWr);
    }

    if(status == SystemP_SUCCESS)
    {   
        uint8_t readCaptureDelay = 0U;

        status += Psram_ospiWrite(config, 0, gOspiFlashAttackVector, OSPI_FLASH_ATTACK_VECTOR_SIZE);
        
        if(status == SystemP_SUCCESS)
        {
            OSPI_setRdDataCaptureDelay(obj->ospiHandle, readCaptureDelay);
            status = Psram_ospiRead(config, 0, gReadBuf, OSPI_FLASH_ATTACK_VECTOR_SIZE);
            if(memcmp(gReadBuf, gOspiFlashAttackVector, OSPI_FLASH_ATTACK_VECTOR_SIZE)!=0)
            {
                status = SystemP_FAILURE;
            }
        }

        while((status != SystemP_SUCCESS) && readCaptureDelay <= OSPI_PSRAM_RD_CAPTURE_DELAY)
        {
            readCaptureDelay++;
            OSPI_setRdDataCaptureDelay(obj->ospiHandle, readCaptureDelay);
            status = Psram_ospiRead(config, 0, gReadBuf, OSPI_FLASH_ATTACK_VECTOR_SIZE);
            if(memcmp(gReadBuf, gOspiFlashAttackVector, OSPI_FLASH_ATTACK_VECTOR_SIZE)!=0)
            {
                status = SystemP_FAILURE;
            }
        }

    }

    obj->phyEnable = OSPI_isPhyEnable(obj->ospiHandle);

    return status;
}

static int32_t Psram_ospiReadCmd(Ram_Config *config, uint8_t cmd, uint32_t readOffset, uint8_t dummyBits,
                            uint8_t numAddrBytes, uint8_t *rxBuf, uint32_t rxBufLen)
{
    int32_t status = SystemP_SUCCESS;
    Ram_OspiPsramObject *obj = (Ram_OspiPsramObject*)(config->object);

    if(obj->ospiHandle == NULL)
    {
        status = SystemP_FAILURE;
    }

    OSPI_ReadCmdParams  rdParams;

    if(status == SystemP_SUCCESS)
    {
        OSPI_ReadCmdParams_init(&rdParams);
        rdParams.cmd           = cmd;
        rdParams.cmdAddr       = readOffset;
        rdParams.rxDataBuf     = rxBuf;
        rdParams.rxDataLen     = rxBufLen;
        rdParams.dummyBits     = dummyBits;
		rdParams.numAddrBytes  = numAddrBytes;
		
        status = OSPI_readCmd(obj->ospiHandle, &rdParams);
    }

    return status;
}

static int32_t Psram_ospiWriteCmd(Ram_Config *config, uint8_t cmd, uint32_t writeOffset, uint8_t *txBuf, uint32_t txBufLen)
{
    int32_t status = SystemP_SUCCESS;
    Ram_OspiPsramObject *obj = (Ram_OspiPsramObject *)(config->object);

    if(obj->ospiHandle == NULL)
    {
        status = SystemP_FAILURE;
    }

    if(SystemP_SUCCESS == status)
    {
        OSPI_WriteCmdParams wrParams;
        OSPI_WriteCmdParams_init(&wrParams);
        wrParams.cmd        = cmd;
        wrParams.cmdAddr    = writeOffset;
        wrParams.txDataBuf  = txBuf;
        wrParams.txDataLen  = txBufLen;
        wrParams.numAddrBytes = obj->numAddrBytes;
        status += OSPI_writeCmd(obj->ospiHandle , &wrParams);
    }

    return status;
}

static int32_t Psram_ospiReadId(Ram_Config *config, uint32_t *manufacturerId, uint32_t *deviceId)
{
    int32_t status = SystemP_SUCCESS;
    Ram_OspiPsramObject *obj = (Ram_OspiPsramObject *)(config->object);

    uint8_t cmd = config->devConfig->cmdRegRd;
    uint8_t numAddrBytes = obj->numAddrBytes;
    uint8_t dummyBits = config->devConfig->dummyClksCmd;
    uint32_t cmdAddr = OSPI_PSRAM_MR1_ADDRESS;
    uint8_t idCode[2] = { 0 };

    status += Psram_ospiReadCmd(config, cmd, cmdAddr, dummyBits, numAddrBytes, idCode, sizeof(idCode)/sizeof(uint8_t));

    if(status == SystemP_SUCCESS)
    {
        *manufacturerId = (uint32_t)idCode[0] & (OSPI_PSRAM_MR1_VENDOR_ID_MASK);
		*deviceId = ((uint32_t)idCode[1] & (OSPI_PSRAM_MR2_DEVICE_ID_MASK)) >> (3);
    }

    if((*manufacturerId != config->attrs->manufacturerId) || (*deviceId != config->attrs->deviceId))
    {
        status = SystemP_FAILURE;
    }

    return status;
}

static int32_t Psram_ospiWrite(Ram_Config *config, uint32_t ramOffset, uint8_t *buf, uint32_t bufLen)
{
    int32_t status = SystemP_SUCCESS;
    Ram_Attrs *attrs = config->attrs;
    Ram_OspiPsramObject *obj = (Ram_OspiPsramObject*)(config->object);

    if(obj->ospiHandle == NULL)
    {
        status = SystemP_FAILURE;
    }

    /* Validate address input */
    if((ramOffset + bufLen) > (attrs->ramSize) || (bufLen < 2))
    {
        status = SystemP_FAILURE;
    }
    
    /* Check for odd address */
    if(ramOffset & 1)
    {
        status = SystemP_FAILURE;
    }

    if(status == SystemP_SUCCESS)
    {
        const OSPI_Attrs *ospi_attrs = ((OSPI_Config *)obj->ospiHandle)->attrs;
        uint32_t baseAddress = ospi_attrs->dataBaseAddr;
        uint32_t size = bufLen;

        if (status == SystemP_SUCCESS)
        {
            status += OSPI_enableDacMode(obj->ospiHandle);

            if(status == SystemP_SUCCESS)
            {
                uint16_t *pSrc = (uint16_t *)buf;
                uint16_t *pDst = (uint16_t *)(ramOffset + baseAddress);
                while (size != 0U)
                {
                    *pDst++ = *pSrc++;
                    size-=2;
                }
            }

            status = SystemP_SUCCESS;
        }
    }
    
    return status;
}

static int32_t Psram_ospiRead(Ram_Config *config, uint32_t ramOffset, uint8_t *buf, uint32_t bufLen)
{
    int32_t status = SystemP_SUCCESS;
    Ram_Attrs *attrs = config->attrs;
    Ram_OspiPsramObject *obj = (Ram_OspiPsramObject*)(config->object);

    if(obj->ospiHandle == NULL)
    {
        status = SystemP_FAILURE;
    }

    /* Validate address input */
    if ((ramOffset + bufLen) > (attrs->ramSize))
    {
        status = SystemP_FAILURE;
    }

    if(obj->phyEnable)
    {
        OSPI_enablePhy(obj->ospiHandle);
    }

    if (status == SystemP_SUCCESS)
    {
        OSPI_Transaction transaction;
        OSPI_Transaction_init(&transaction);
        transaction.addrOffset = ramOffset;
        transaction.buf = (void *)buf;
        transaction.count = bufLen;
        status = OSPI_readDirect(obj->ospiHandle, &transaction);
    }

    if(obj->phyEnable)
    {
        OSPI_disablePhy(obj->ospiHandle);
    }
    return status;
}

static int32_t Psram_ospiReset(Ram_Config *config)
{
    int32_t status = SystemP_SUCCESS;

    status = Psram_ospiWriteCmd(config, config->devConfig->cmdReset, 0, NULL, 0);

    return status;
}

static void Psram_ospiClose(Ram_Config *config)
{
    return;
}