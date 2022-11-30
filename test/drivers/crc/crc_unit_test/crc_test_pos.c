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
 *  \file     crc_test_pos.c
 *
 *  \brief    This file contains crc API unit test code.
 *
 *  \details  crc unit tests
 **/

#include "crc_main.h"
#include "ti_drivers_config.h"

int32_t crc_posTest(void)
{
    int32_t               testStatus = CSL_APP_TEST_PASS;
    CRC_Channel_t         channel = CRC_CHANNEL_1;
	uint32_t              baseAddr = 0;
    uint32_t              crcWatchdogPreload = CRC_WDTOPLD1;
    uint32_t              crcBlockPreload = CRC_BCTOPLD1;
	CRC_Config            config;
    uint32_t              intrMask = 0;
	uint32_t              ctrlFlag = 0;
	uint32_t              dataBusMask = 0;
	uint32_t              busEnableMask = 0;
	baseAddr           = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
    /* positive test of readStaticreg API */
    if (testStatus == CSL_APP_TEST_PASS)
    {
        CRC_StaticRegs pStaticRegs;
        if ((CRC_readStaticRegs(baseAddr,&pStaticRegs)) != CSL_PASS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

	if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	/* positive test of CRC_powerDownCtrl API */
    if (testStatus == CSL_APP_TEST_PASS)
    {
        uint32_t ctrlFlag = 0;
        if ((CRC_powerDownCtrl(baseAddr,ctrlFlag)) != CSL_PASS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	/* positive test of Bus tracing control API */
    if (testStatus == CSL_APP_TEST_PASS)
    {
        uint32_t         ctrlFlag = 0;
        CRC_DataBusMask_t dataBusMask = 0;
        CRC_DataBusMask_t busEnableMask = 0;
        if ((CRC_dataBusTracingCtrl(baseAddr,ctrlFlag,dataBusMask,busEnableMask)) != CSL_PASS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    /* positive test of Verify Bus tracing control API */
    if (testStatus == CSL_APP_TEST_PASS)
    {
        uint32_t         ctrlFlag = 0;
        CRC_DataBusMask_t dataBusMask = 0;
        CRC_DataBusMask_t busEnableMask = 0;
        if ((CRC_verifyBusTracingCtrl(baseAddr,ctrlFlag,dataBusMask,busEnableMask)) != CSL_PASS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	/*  positive test of read priority API*/
    if (testStatus == CSL_APP_TEST_PASS)
    {
        uint32_t pIntVecAddr;

        if ((CRC_getHighestPriorityIntrStatus(baseAddr,&pIntVecAddr)) != CSL_PASS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

	for (channel = CRC_CHANNEL_1; channel <= CRC_CHANNEL_2; channel++)
    {
        /*  positive test of init API*/
        if (testStatus == CSL_APP_TEST_PASS)
        {
            if (CRC_initialize(baseAddr, channel, crcWatchdogPreload, crcBlockPreload)!= CSL_PASS)
            {
                testStatus = CSL_APP_TEST_FAILED;
            }
        }

        if (testStatus != CSL_APP_TEST_PASS)
        {
            DebugP_log("crc_pos_Test: failure on line no. %d \r\n", __LINE__);
            return (testStatus);
        }

        /*  positive test of verify init API*/
        if (testStatus == CSL_APP_TEST_PASS)
        {
            if (CRC_verifyInitialize(baseAddr, channel, crcWatchdogPreload, crcBlockPreload) != CSL_PASS)
            {
                testStatus = CSL_APP_TEST_FAILED;
            }
        }

        if (testStatus != CSL_APP_TEST_PASS)
        {
            DebugP_log("crc_pos_Test: failure on line no. %d \r\n", __LINE__);
            return (testStatus);
        }
        /*  positive test of config API*/
        if (testStatus == CSL_APP_TEST_PASS)
        {
			/* Configure CRC channel */
			config.mode         = CRC_OPERATION_MODE_FULLCPU;
			config.type         = CRC_TYPE_16BIT;
			config.dataLen      = CRC_DATALENGTH_16BIT;
			config.bitSwap      = CRC_BITSWAP_MSB;
			config.byteSwap     = CRC_BYTESWAP_ENABLE;
			config.patternCount = APP_CRC_PATTERN_CNT;
			config.sectorCount  = APP_CRC_SECT_CNT;
            if ((CRC_configure(baseAddr,channel,&config)) != CSL_PASS)
            {
                testStatus = CSL_APP_TEST_FAILED;
            }
        }

        if (testStatus != CSL_APP_TEST_PASS)
        {
            DebugP_log("crc_pos_Test: failure on line no. %d \r\n", __LINE__);
            return (testStatus);
        }
		/*  positive test of verify config API*/
        if (testStatus == CSL_APP_TEST_PASS)
        {
            if ((CRC_verifyConfigure(baseAddr, channel,APP_CRC_PATTERN_CNT,APP_CRC_SECT_CNT, CRC_OPERATION_MODE_FULLCPU)) != CSL_PASS)
            {
                testStatus = CSL_APP_TEST_FAILED;
            }
        }
        if (testStatus != CSL_APP_TEST_PASS)
        {
            DebugP_log("crc_pos_Test: failure on line no. %d \r\n", __LINE__);
            return (testStatus);
        }
		if (testStatus == CSL_APP_TEST_PASS)
        {
			/* Configure CRC channel */
			config.mode         = CRC_OPERATION_MODE_SEMICPU;
			config.type         = CRC_TYPE_16BIT;
			config.dataLen      = CRC_DATALENGTH_16BIT;
			config.bitSwap      = CRC_BITSWAP_MSB;
			config.byteSwap     = CRC_BYTESWAP_ENABLE;
			config.patternCount = APP_CRC_PATTERN_CNT;
			config.sectorCount  = APP_CRC_SECT_CNT;
            if ((CRC_configure(baseAddr,channel,&config)) != CSL_PASS)
            {
                testStatus = CSL_APP_TEST_FAILED;
            }
        }

        if (testStatus != CSL_APP_TEST_PASS)
        {
            DebugP_log("crc_pos_Test: failure on line no. %d \r\n", __LINE__);
            return (testStatus);
        }

        /*  positive test of verify config API*/
        if (testStatus == CSL_APP_TEST_PASS)
        {
            if ((CRC_verifyConfigure(baseAddr, channel,APP_CRC_PATTERN_CNT,APP_CRC_SECT_CNT, CRC_OPERATION_MODE_SEMICPU)) != CSL_PASS)
            {
                testStatus = CSL_APP_TEST_FAILED;
            }
        }
        if (testStatus != CSL_APP_TEST_PASS)
        {
            DebugP_log("crc_pos_Test: failure on line no. %d \r\n", __LINE__);
            return (testStatus);
        }

        /*  positive test of channel reset API*/
        if (testStatus == CSL_APP_TEST_PASS)
        {
            if ((CRC_channelReset(baseAddr,channel)) != CSL_PASS)
            {
                testStatus = CSL_APP_TEST_FAILED;
            }
        }

        if (testStatus != CSL_APP_TEST_PASS)
        {
            DebugP_log("crc_pos_Test: failure on line no. %d \r\n", __LINE__);
            return (testStatus);
        }

		/*  positive test of read PSA register address API*/
        if (testStatus == CSL_APP_TEST_PASS)
        {
            CRC_SignatureRegAddr pCRCRegAddr;
            if ((CRC_getPSASigRegAddr(baseAddr,channel, &pCRCRegAddr)) != CSL_PASS)
            {
                testStatus = CSL_APP_TEST_FAILED;
            }
        }

        if (testStatus != CSL_APP_TEST_PASS)
        {
            DebugP_log("crc_pos_Test: failure on line no. %d \r\n", __LINE__);
            return (testStatus);
        }

		/*  positive test of get PSA signature API*/
        if (testStatus == CSL_APP_TEST_PASS)
        {
            CRC_Signature pCRCPSASign;
            if (CRC_getPSASig(baseAddr,channel, &pCRCPSASign)!= CSL_PASS)
            {
                testStatus = CSL_APP_TEST_FAILED;
            }
        }

        if (testStatus != CSL_APP_TEST_PASS)
        {
            DebugP_log("crc_pos_Test: failure on line no. %d \r\n", __LINE__);
            return (testStatus);
        }

		/*  positive test of set PSA signature API*/
        if (testStatus == CSL_APP_TEST_PASS)
        {
            CRC_Signature pCRCPSASeedSign;
            pCRCPSASeedSign.regL    = 255U;
            pCRCPSASeedSign.regH    = 255U;
            if (CRC_setPSASeedSig(baseAddr,channel, &pCRCPSASeedSign)!= CSL_PASS)
            {
                testStatus = CSL_APP_TEST_FAILED;
            }
        }

        if (testStatus != CSL_APP_TEST_PASS)
        {
            DebugP_log("crc_pos_Test: failure on line no. %d \r\n", __LINE__);
            return (testStatus);
        }


        /*  positive test of intrStatus API*/
        if (testStatus == CSL_APP_TEST_PASS)
        {
            uint32_t   pIntrStatus;
            if ((CRC_getIntrStatus(baseAddr,channel,&pIntrStatus))!= CSL_PASS)
            {
                testStatus = CSL_APP_TEST_FAILED;
            }
        }

        if (testStatus != CSL_APP_TEST_PASS)
        {
            DebugP_log("crc_pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        /* positive test of EnableIntr API */
        if (testStatus == CSL_APP_TEST_PASS)
        {

            if (CRC_enableIntr(baseAddr,channel,intrMask) != CSL_PASS)
            {
                testStatus = CSL_APP_TEST_FAILED;
            }
        }

        if (testStatus != CSL_APP_TEST_PASS)
        {
            DebugP_log("crc_pos_Test: failure on line no. %d \r\n", __LINE__);
            return (testStatus);
        }

        /* positive test of DisableIntr API */
        if (testStatus == CSL_APP_TEST_PASS)
        {
            if (CRC_disableIntr(baseAddr,channel,intrMask)!= CSL_PASS)
            {
                testStatus = CSL_APP_TEST_FAILED;
            }
        }

        if (testStatus != CSL_APP_TEST_PASS)
        {
            DebugP_log("crc_pos_Test: failure on line no. %d \r\n", __LINE__);
            return (testStatus);
        }

        /* positive test of ClearIntr API */
        if (testStatus == CSL_APP_TEST_PASS)
        {
            if (CRC_clearIntr(baseAddr,channel,intrMask) != CSL_PASS)
            {
                testStatus = CSL_APP_TEST_FAILED;
            }
        }

        if (testStatus != CSL_APP_TEST_PASS)
        {
            DebugP_log("crc_pos_Test: failure on line no. %d \r\n", __LINE__);
            return (testStatus);
        }
        /* positive test of CRC_isBusy API */
        if (testStatus == CSL_APP_TEST_PASS)
        {
            uint32_t   pBusyFlag;
            if ((CRC_isBusy(baseAddr,channel,&pBusyFlag)) != CSL_PASS)
            {
                testStatus = CSL_APP_TEST_FAILED;
            }
        }

		if (testStatus != CSL_APP_TEST_PASS)
        {
            DebugP_log("crc_pos_Test: failure on line no. %d \r\n", __LINE__);
            return (testStatus);
        }

        /* positive test of Get Currrent Sector Number API */
        if (testStatus == CSL_APP_TEST_PASS)
        {
            uint32_t pCurSecNum;
            if ((CRC_getCurSecNum(baseAddr,channel,&pCurSecNum)) != CSL_PASS)
            {
                testStatus = CSL_APP_TEST_FAILED;
            }
        }

        if (testStatus != CSL_APP_TEST_PASS)
        {
            DebugP_log("crc_pos_Test: failure on line no. %d \r\n", __LINE__);
            return (testStatus);
        }

		/* positive test of Get Currrent PSA Signature API */
        if (testStatus == CSL_APP_TEST_PASS)
        {
            CRC_Signature pCurPSASig;
            if ((CRC_getCurPSASig(baseAddr,channel,&pCurPSASig)) != CSL_PASS)
            {
                testStatus = CSL_APP_TEST_FAILED;
            }
        }

        if (testStatus != CSL_APP_TEST_PASS)
        {
            DebugP_log("crc_pos_Test: failure on line no. %d \r\n", __LINE__);
            return (testStatus);
        }

		/* positive test of Get Raw Data API */
        if (testStatus == CSL_APP_TEST_PASS)
        {
            CRC_Signature pRawData;
            if ((CRC_getRawData(baseAddr,channel,&pRawData)) != CSL_PASS)
            {
                testStatus = CSL_APP_TEST_FAILED;
            }
        }

        if (testStatus != CSL_APP_TEST_PASS)
        {
            DebugP_log("crc_pos_Test: failure on line no. %d \r\n", __LINE__);
            return (testStatus);
        }
    }

	if (testStatus == CSL_APP_TEST_PASS)
    {
        if (CRC_verifyBusTracingCtrl(0U, ctrlFlag, dataBusMask, busEnableMask)!= CSL_EBADARGS)
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
        if (CRC_verifyBusTracingCtrl(0U, CSL_TRUE+1U, dataBusMask, busEnableMask)!= CSL_EBADARGS)
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
        if (CRC_verifyBusTracingCtrl(baseAddr, CSL_TRUE+1U, 0U, 0U)!= CSL_EBADARGS)
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
        if (CRC_verifyBusTracingCtrl(1U, CSL_TRUE+1U, 0U, 0U)!= CSL_EBADARGS)
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
        if (CRC_verifyBusTracingCtrl(baseAddr, 2U, dataBusMask, busEnableMask)!= CSL_EBADARGS)
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
        if (CRC_verifyBusTracingCtrl(baseAddr, 1U, dataBusMask, 255U)!= CSL_EBADARGS)
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
        if (CRC_verifyBusTracingCtrl(baseAddr, 1U, 255U, busEnableMask)!= CSL_EBADARGS)
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
		uint32_t pIntVecAddr;
        if (CRC_getHighestPriorityIntrStatus(0U, &pIntVecAddr)!= CSL_EBADARGS)
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
        CRC_dataBusTracingCtrl(baseAddr, 1U, dataBusMask, busEnableMask);
        if (CRC_verifyBusTracingCtrl(baseAddr, 0U, dataBusMask, busEnableMask)!= CSL_EFAIL)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }
    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == CSL_APP_TEST_PASS)
    {
        CRC_dataBusTracingCtrl(baseAddr, 1U, dataBusMask, busEnableMask);
        if (CRC_verifyBusTracingCtrl(baseAddr, 1U, dataBusMask, CRC_MCRC_BUS_SEL_MEN_MASK)!= CSL_EFAIL)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == CSL_APP_TEST_PASS)
    {
        CRC_dataBusTracingCtrl(baseAddr, 1U, dataBusMask, busEnableMask);
        if (CRC_verifyBusTracingCtrl(baseAddr, 1U, dataBusMask, CRC_MCRC_BUS_SEL_MEN_MASK)!= CSL_EFAIL)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("Crc_Pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
	if (testStatus == CSL_APP_TEST_PASS)
    {


        if ((CRC_getHighestPriorityIntrStatus(baseAddr,NULL)) != CSL_EBADARGS)
        {
            testStatus = CSL_APP_TEST_FAILED;
        }
    }

    if (testStatus != CSL_APP_TEST_PASS)
    {
        DebugP_log("crc_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    return (testStatus);
}
