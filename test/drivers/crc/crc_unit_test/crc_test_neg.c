/* Copyright (C) 2023 Texas Instruments Incorporated
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
 *
 */

 /**
 *  \file     crc_test_neg.c
 *
 *  \brief    This file contains crc API unit test code..
 *
 *  \details  crc unit tests
 **/

#include "crc_main.h"
#include "ti_drivers_config.h"

int32_t crc_negTest(void)
{
    int32_t               testStatus = CSL_APP_TEST_PASS;
    CRC_Channel_t         channel = CRC_CHANNEL_1;
	uint32_t              baseAddr = 0;
    uint32_t              crcWatchdogPreload = CRC_WDTOPLD1;
    uint32_t              crcBlockPreload = CRC_BCTOPLD1;
	CRC_Config            config;
	uint32_t              patternCount = 0;
    uint32_t              sectorCount = 0;
    CRC_OperationMode_t   mode = 0;
	uint32_t              intrMask = 0x1U;
	CRC_Signature pCRCPSASeedSign;
    pCRCPSASeedSign.regL    = 255U;
    pCRCPSASeedSign.regH    = 255U;
	uint32_t   pIntrStatus = 0;
	uint32_t   pBusyFlag = 0;
	uint32_t   pCurSecNum = 0;
	uint32_t ctrlFlag = 0;
	CRC_StaticRegs pStaticRegs;
	CRC_DataBusMask_t dataBusMask = 0;
    CRC_DataBusMask_t busEnableMask = 0;

	/*  Error/Fault test of init API*/
    if (testStatus == CSL_APP_TEST_PASS)
    {
        if (CRC_initialize(NULL, channel, crcWatchdogPreload, crcBlockPreload)!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == CSL_APP_TEST_PASS)
    {
        if (CRC_initialize(baseAddr, 5U, crcWatchdogPreload, crcBlockPreload)!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	if (testStatus == CSL_APP_TEST_PASS)
    {
        if (CRC_initialize(baseAddr, channel, CRC_WDTOPLD_MAX+1U, crcBlockPreload)!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }


	if (testStatus == CSL_APP_TEST_PASS)
    {
        if (CRC_initialize(NULL, channel, CRC_WDTOPLD_MAX+1, CRC_BCTOPLD_MAX+1U)!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	if (testStatus == CSL_APP_TEST_PASS)
    {
        if (CRC_initialize(NULL, channel, CRC_WDTOPLD_MAX+1, CRC_BCTOPLD1)!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
        if (CRC_initialize(NULL, channel, CRC_WDTOPLD1, CRC_BCTOPLD_MAX+1U)!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
        if (CRC_initialize(baseAddr, channel, crcWatchdogPreload, CRC_BCTOPLD_MAX+1U)!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr  = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
        if (CRC_initialize(baseAddr, 5U, CRC_WDTOPLD1, CRC_BCTOPLD1)!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr  = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
        if (CRC_initialize(baseAddr, 5U, CRC_WDTOPLD1, CRC_BCTOPLD1)!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr  = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
        if (CRC_initialize(baseAddr, 5U, 0x00FFFFFF + 1U , 0x00FFFFFFU)!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr  = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
        if (CRC_initialize(baseAddr, 5U, 100U , 0x00FFFFFF+1U)!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr  = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
        if (CRC_initialize(baseAddr, 3U, 0x3cU , 0x7cU)!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr  = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
        if (CRC_initialize(baseAddr, 3U, 0x4cU , 0x7cU)!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    /*  Error/Fualt test of verify init API*/
	if (testStatus == CSL_APP_TEST_PASS)
    {
        if (CRC_verifyInitialize(NULL, channel, crcWatchdogPreload, crcBlockPreload)!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	if (testStatus == CSL_APP_TEST_PASS)
    {
        if (CRC_verifyInitialize(baseAddr, 7U, crcWatchdogPreload, crcBlockPreload)!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	if (testStatus == CSL_APP_TEST_PASS)
    {
        if (CRC_verifyInitialize(baseAddr, channel, CRC_WDTOPLD_MAX+1U, crcBlockPreload)!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	if (testStatus == CSL_APP_TEST_PASS)
    {
        if (CRC_verifyInitialize(baseAddr, channel, crcWatchdogPreload, CRC_BCTOPLD_MAX+1U)!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
        if (CRC_verifyInitialize(NULL, channel, CRC_WDTOPLD_MAX+1, CRC_BCTOPLD_MAX+1U)!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	if (testStatus == CSL_APP_TEST_PASS)
    {
        if (CRC_verifyInitialize(NULL, channel, CRC_WDTOPLD_MAX+1, CRC_BCTOPLD1)!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
        if (CRC_verifyInitialize(NULL, channel, CRC_WDTOPLD1, CRC_BCTOPLD_MAX+1U)!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr  = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
        if (CRC_verifyInitialize(baseAddr, 5U, CRC_WDTOPLD1, CRC_BCTOPLD1)!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr  = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
        if (CRC_verifyInitialize(baseAddr, 5U, 0x00FFFFFF + 1U , 0x00FFFFFFU)!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr  = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
        if (CRC_verifyInitialize(baseAddr, 5U, 100U , 0x00FFFFFF+1U)!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr  = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
        if (CRC_verifyInitialize(baseAddr, 2U, 0x3cU , 0x7cU)!= CSL_EFAIL)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr  = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
        if (CRC_verifyInitialize(baseAddr, 2U, 0x4cU , 0x7cU)!= CSL_EFAIL)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	/*---------------------------------------------------------------------------------------*/
    /*  Error/Fualt test of config API*/

	if (testStatus == CSL_APP_TEST_PASS)
    {
        if ((CRC_configure(NULL,channel,&config))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	if (testStatus == CSL_APP_TEST_PASS)
    {
        if ((CRC_configure(baseAddr,5U,&config))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	if (testStatus == CSL_APP_TEST_PASS)
    {
        if ((CRC_configure(baseAddr,channel,NULL))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	if (testStatus == CSL_APP_TEST_PASS)
    {
		config.mode         = CRC_OPERATION_MODE_FULLCPU + 1U;
		config.type         = CRC_TYPE_64BIT;
		config.dataLen      = CRC_DATALENGTH_64BIT;
		config.bitSwap      = CRC_BITSWAP_MSB + 1U;
		config.byteSwap     = CRC_BYTESWAP_DISABLE;
		config.patternCount = APP_CRC_PATTERN_CNT + 1U;
		config.sectorCount  = APP_CRC_SECT_CNT + 1U;
        if ((CRC_configure(baseAddr,channel,&config))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		config.mode         = 4U;
		config.type         = CRC_TYPE_16BIT;
		config.dataLen      = CRC_DATALENGTH_16BIT;
		config.bitSwap      = CRC_BITSWAP_MSB;
		config.byteSwap     = CRC_BYTESWAP_ENABLE;
		config.patternCount = APP_CRC_PATTERN_CNT;
		config.sectorCount  = APP_CRC_SECT_CNT;
		baseAddr            = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
        if ((CRC_configure(baseAddr,3U,&config))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
        if ((CRC_configure(baseAddr,3U,NULL))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		config.mode         = 1U;
		config.type         = 3U;
		config.dataLen      = CRC_DATALENGTH_16BIT;
		config.bitSwap      = CRC_BITSWAP_MSB;
		config.byteSwap     = CRC_BYTESWAP_ENABLE;
		config.patternCount = APP_CRC_PATTERN_CNT;
		config.sectorCount  = APP_CRC_SECT_CNT;
		baseAddr            = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
        if ((CRC_configure(baseAddr,3U,&config))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		config.mode         = 1U;
		config.type         = 1U;
		config.dataLen      = 3U;
		config.bitSwap      = CRC_BITSWAP_MSB;
		config.byteSwap     = CRC_BYTESWAP_ENABLE;
		config.patternCount = APP_CRC_PATTERN_CNT;
		config.sectorCount  = APP_CRC_SECT_CNT;
		baseAddr            = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
        if ((CRC_configure(baseAddr,3U,&config))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		config.mode         = 1U;
		config.type         = 1U;
		config.dataLen      = 0U;
		config.bitSwap      = 2U;
		config.byteSwap     = CRC_BYTESWAP_ENABLE;
		config.patternCount = APP_CRC_PATTERN_CNT;
		config.sectorCount  = APP_CRC_SECT_CNT;
		baseAddr            = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
        if ((CRC_configure(baseAddr,3U,&config))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		config.mode         = 1U;
		config.type         = 1U;
		config.dataLen      = 1U;
		config.bitSwap      = 0U;
		config.byteSwap     = CRC_BYTESWAP_ENABLE;
		config.patternCount = APP_CRC_PATTERN_CNT;
		config.sectorCount  = APP_CRC_SECT_CNT;
		baseAddr            = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
        if ((CRC_configure(baseAddr,3U,&config))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		config.mode         = 1U;
		config.type         = 1U;
		config.dataLen      = 1U;
		config.bitSwap      = 0U;
		config.byteSwap     = 2U;
		config.patternCount = APP_CRC_PATTERN_CNT;
		config.sectorCount  = APP_CRC_SECT_CNT;
		baseAddr            = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
        if ((CRC_configure(baseAddr,3U,&config))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		config.mode         = 1U;
		config.type         = 1U;
		config.dataLen      = 1U;
		config.bitSwap      = 0U;
		config.byteSwap     = 0U;
		config.patternCount = 0x000FFFFF+1U;
		config.sectorCount  = APP_CRC_SECT_CNT;
		baseAddr            = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
        if ((CRC_configure(baseAddr,3U,&config))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		config.mode         = 1U;
		config.type         = 1U;
		config.dataLen      = 1U;
		config.bitSwap      = 0U;
		config.byteSwap     = 0U;
		config.patternCount = 1U;
		config.sectorCount  = 0x0000FFFF+1U ;
		baseAddr            = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
        if ((CRC_configure(baseAddr,3U,&config))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    /*  Error/Fualt test of verify config API*/
	if (testStatus == CSL_APP_TEST_PASS)
    {
        if ((CRC_verifyConfigure(NULL, channel,patternCount,sectorCount, mode))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	if (testStatus == CSL_APP_TEST_PASS)
    {
        if ((CRC_verifyConfigure(baseAddr, 5U,patternCount,sectorCount, mode))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
        if ((CRC_verifyConfigure(baseAddr,channel,CRC_PATTERN_COUNT_MAX+1U,sectorCount, mode))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	if (testStatus == CSL_APP_TEST_PASS)
    {
        if ((CRC_verifyConfigure(baseAddr, channel,patternCount,CRC_SECTOR_COUNT_MAX+1U, mode))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
        if ((CRC_verifyConfigure(baseAddr, channel,patternCount,sectorCount, CRC_CTRL2_CH1_MODE_FULLCPU+1U))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
        if ((CRC_verifyConfigure(baseAddr,5U,APP_CRC_PATTERN_CNT,APP_CRC_SECT_CNT, CRC_OPERATION_MODE_FULLCPU))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
        if ((CRC_verifyConfigure(baseAddr,2U,APP_CRC_PATTERN_CNT+1,APP_CRC_SECT_CNT+1, CRC_OPERATION_MODE_SEMICPU))!= CSL_EFAIL)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
        if ((CRC_verifyConfigure(baseAddr,2U,APP_CRC_PATTERN_CNT,APP_CRC_SECT_CNT+1, CRC_OPERATION_MODE_SEMICPU))!= CSL_EFAIL)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
        if ((CRC_verifyConfigure(baseAddr,2U,APP_CRC_PATTERN_CNT,APP_CRC_SECT_CNT, CRC_OPERATION_MODE_SEMICPU))!= CSL_EFAIL)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    /*  Error/Fualt test of channel reset API*/
	if (testStatus == CSL_APP_TEST_PASS)
    {
        if ((CRC_channelReset(NULL,channel))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	if (testStatus == CSL_APP_TEST_PASS)
    {
        if ((CRC_channelReset(baseAddr,5U))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr  = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
        if ((CRC_channelReset(baseAddr,5U))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    /*  Error/Fualt test of read PSA register address API*/
	if (testStatus == CSL_APP_TEST_PASS)
    {
		CRC_SignatureRegAddr pCRCRegAddr;
        if ((CRC_getPSASigRegAddr(NULL,channel, &pCRCRegAddr))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		CRC_SignatureRegAddr pCRCRegAddr;
        if ((CRC_getPSASigRegAddr(baseAddr,5U, &pCRCRegAddr))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
        if ((CRC_getPSASigRegAddr(baseAddr,channel,NULL_PTR))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		CRC_SignatureRegAddr pCRCRegAddr;
		baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
        if ((CRC_getPSASigRegAddr(baseAddr,5U, &pCRCRegAddr))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    /*  Error/Fualt test of read PSA signature API*/
	if (testStatus == CSL_APP_TEST_PASS)
    {
		CRC_Signature pCRCPSASign;
        if ((CRC_getPSASig(NULL,channel, &pCRCPSASign))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	if (testStatus == CSL_APP_TEST_PASS)
    {
		CRC_Signature pCRCPSASign;
        if ((CRC_getPSASig(baseAddr,5U, &pCRCPSASign))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	if (testStatus == CSL_APP_TEST_PASS)
    {

        if ((CRC_getPSASig(baseAddr,channel, NULL_PTR))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }


    /*  Error/Fualt test of set PSA signature API*/
	if (testStatus == CSL_APP_TEST_PASS)
    {
        if ((CRC_setPSASeedSig(NULL,channel, &pCRCPSASeedSign))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
        if ((CRC_setPSASeedSig(baseAddr,7U, &pCRCPSASeedSign))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
        if ((CRC_setPSASeedSig(baseAddr,channel, NULL_PTR))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    /*  Error/Fault test of read priority API*/
	if (testStatus == CSL_APP_TEST_PASS)
    {
        uint32_t pIntVecAddr;
		if ((CRC_getHighestPriorityIntrStatus(NULL,&pIntVecAddr))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		if ((CRC_getHighestPriorityIntrStatus(baseAddr,NULL_PTR))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
		if ((CRC_getHighestPriorityIntrStatus(baseAddr,NULL_PTR))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    /*  Error/Fault test of interrupt status API*/
	if (testStatus == CSL_APP_TEST_PASS)
    {
		if ((CRC_getIntrStatus(NULL,channel,&pIntrStatus))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		if ((CRC_getIntrStatus(baseAddr,7U,&pIntrStatus))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		if ((CRC_getIntrStatus(baseAddr,channel,NULL_PTR))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
		if ((CRC_getIntrStatus(baseAddr,channel,NULL_PTR))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
		if ((CRC_getIntrStatus(baseAddr,7U,&pIntrStatus))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    /*  Error/Fault test of enable interrupt API*/
	if (testStatus == CSL_APP_TEST_PASS)
    {
		if ((CRC_enableIntr(NULL,channel,intrMask))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	if (testStatus == CSL_APP_TEST_PASS)
    {
		if ((CRC_enableIntr(baseAddr,7U,intrMask))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	if (testStatus == CSL_APP_TEST_PASS)
    {
		if ((CRC_enableIntr(baseAddr,channel,5U))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr  = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
		if ((CRC_enableIntr(baseAddr,channel,5U))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr  = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
		if ((CRC_enableIntr(baseAddr,3U,0U))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }


    /*  Error/Fault test of disable interrupt API*/
	if (testStatus == CSL_APP_TEST_PASS)
    {
		if ((CRC_disableIntr(NULL,channel,intrMask))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	if (testStatus == CSL_APP_TEST_PASS)
    {
		if ((CRC_disableIntr(baseAddr,7U,intrMask))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	if (testStatus == CSL_APP_TEST_PASS)
    {
		if ((CRC_disableIntr(baseAddr,channel,5U))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr  = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
		if ((CRC_disableIntr(baseAddr,channel,5U))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr  = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
		if ((CRC_disableIntr(baseAddr,3U,0U))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    /*  Error/Fault test of clear interrupt API*/
	if (testStatus == CSL_APP_TEST_PASS)
    {
		if ((CRC_clearIntr(NULL,channel,intrMask))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		if ((CRC_clearIntr(baseAddr,5U,intrMask))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		if ((CRC_clearIntr(baseAddr,channel,1U))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr  = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
		if ((CRC_clearIntr(baseAddr,channel,5U))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr  = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
		if ((CRC_clearIntr(baseAddr,3U,0U))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    /*  Error/Fault test of power down control API*/
	if (testStatus == CSL_APP_TEST_PASS)
    {
		if ((CRC_powerDownCtrl(NULL,ctrlFlag))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		if ((CRC_powerDownCtrl(baseAddr,2U))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr  = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
		if ((CRC_powerDownCtrl(baseAddr,2U))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    /*  Error/Fault test of CRC is busy API*/
	if (testStatus == CSL_APP_TEST_PASS)
    {
		if ((CRC_isBusy(NULL,channel,&pBusyFlag))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		if ((CRC_isBusy(baseAddr,4U,&pBusyFlag))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		if ((CRC_isBusy(baseAddr,channel,NULL_PTR))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr  = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
		if ((CRC_isBusy(baseAddr,channel,NULL_PTR))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    /*  Error/Fault test of read the current sector number API*/
	if (testStatus == CSL_APP_TEST_PASS)
    {
		if ((CRC_getCurSecNum(NULL,channel,&pCurSecNum))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	if (testStatus == CSL_APP_TEST_PASS)
    {
		if ((CRC_getCurSecNum(baseAddr,7U,&pCurSecNum))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		if ((CRC_getCurSecNum(baseAddr,channel,NULL_PTR))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr  = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
		if ((CRC_getCurSecNum(baseAddr,channel,NULL_PTR))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }


    /*  Error/Fault test of read the current PSA signature value API*/
	if (testStatus == CSL_APP_TEST_PASS)
    {
		CRC_Signature pCurPSASig;
		if ((CRC_getCurPSASig(NULL,channel,&pCurPSASig))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		CRC_Signature pCurPSASig;
		if ((CRC_getCurPSASig(baseAddr,7U,&pCurPSASig))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {

		if ((CRC_getCurPSASig(baseAddr,channel,NULL_PTR))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr  = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
		if ((CRC_getCurPSASig(baseAddr,channel,NULL_PTR))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    /*  Error/Fault test of read the raw data API*/
	if (testStatus == CSL_APP_TEST_PASS)
    {
		CRC_Signature pRawData;
		if ((CRC_getRawData(NULL,channel,&pRawData))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	if (testStatus == CSL_APP_TEST_PASS)
    {
		CRC_Signature pRawData;
		if ((CRC_getRawData(baseAddr,8U,&pRawData))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	if (testStatus == CSL_APP_TEST_PASS)
    {
		if ((CRC_getRawData(baseAddr,channel,NULL_PTR))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr  = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
		if ((CRC_getRawData(baseAddr,channel,NULL_PTR))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    /*  Error/Fault test of read static register API*/
	if (testStatus == CSL_APP_TEST_PASS)
    {

		if ((CRC_readStaticRegs(NULL,&pStaticRegs))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {

		if ((CRC_readStaticRegs(baseAddr,NULL_PTR))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr  = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
		if ((CRC_readStaticRegs(baseAddr,NULL_PTR))!= CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    /*  Error/Fault test of data bus tracing control API*/
	if (testStatus == CSL_APP_TEST_PASS)
    {
        if ((CRC_dataBusTracingCtrl(NULL,ctrlFlag,dataBusMask,busEnableMask)) != CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	if (testStatus == CSL_APP_TEST_PASS)
    {
        if ((CRC_dataBusTracingCtrl(baseAddr,2U,dataBusMask,busEnableMask)) != CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
        if ((CRC_dataBusTracingCtrl(baseAddr,ctrlFlag,20U,busEnableMask)) != CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
		baseAddr  = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
		dataBusMask=0U;
		busEnableMask=0U;
        if ((CRC_dataBusTracingCtrl(baseAddr,0U,dataBusMask,busEnableMask)) != CSL_PASS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
        if ((CRC_dataBusTracingCtrl(baseAddr,ctrlFlag,dataBusMask,30U)) != CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    /*  Error/Fault test of verify data bus tracing control API*/
	if (testStatus == CSL_APP_TEST_PASS)
    {
        if ((CRC_verifyBusTracingCtrl(NULL,ctrlFlag,dataBusMask,busEnableMask)) != CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
        if ((CRC_verifyBusTracingCtrl(baseAddr,2U,dataBusMask,busEnableMask)) != CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
        if ((CRC_verifyBusTracingCtrl(baseAddr,ctrlFlag,20U,busEnableMask)) != CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {
        if ((CRC_verifyBusTracingCtrl(baseAddr,ctrlFlag,dataBusMask,30U)) != CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	return (testStatus);

}
