/*
 *   Copyright (c) Texas Instruments Incorporated 2022-2023
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
 *  \file     ecc_test_main.c
 *
 *  \brief    This file contains ECC test code.
 *
 *  \details  ECC unit tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <stdio.h>
#include <sdl/dpl/sdl_dpl.h>
#include <sdl/sdl_ecc.h>
#include <dpl_interface.h>
#include <kernel/dpl/TimerP.h>
#include <kernel/dpl/DebugP.h>
#include "ecc_test_main.h"
#include <unity.h>

#if defined(SOC_AM273X)
#if defined(R5F_INPUTS)
#include <sdl/include/am273x/sdlr_mss_ecc_agga.h>
#include <sdl/include/am273x/sdlr_mss_ecc_aggb.h>
#include <sdl/include/am273x/sdlr_mss_ecc_agg_mss.h>
#elif defined(C66_INPUTS)
#include <sdl/include/am273x/sdlr_dss_ecc_agg.h>
#include <sdl/include/am273x/sdlr_intr_esm_dss.h>
#endif
#endif

#if defined(SOC_AWR294X)
#if defined(R5F_INPUTS)
#include <sdl/include/awr294x/sdlr_mss_ecc_agga.h>
#include <sdl/include/awr294x/sdlr_mss_ecc_aggb.h>
#include <sdl/include/awr294x/sdlr_mss_ecc_agg_mss.h>
#elif defined(C66_INPUTS)
#include <sdl/include/awr294x/sdlr_dss_ecc_agg.h>
#endif
#endif
/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
/* delay*/
#define SDL_DELAY               			1000000

#define SDL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS				(0x50D18094u)
#define SDL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW			(0x50D18098u)
#define SDL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS 				(0x50D18084u)
#define SDL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW			(0x50D18088u)

#define SDL_CLEAR_STATUS									(0xffu)

#define R5FSS0_CORE0_ECC_AGGR_SEC_STATUS    				(0x02F7B840u)
#define R5FSS0_CORE0_ECC_AGGR_DED_STATUS    				(0x02F7B940u)
#define R5FSS0_CORE1_ECC_AGGR_SEC_STATUS    				(0x02F7BC40u)
#define R5FSS0_CORE1_ECC_AGGR_DED_STATUS    				(0x02F7BD40u)
#define MSS_ECC_AGG_SEC_STATUS								(0x02F7C040u)
#define MSS_ECC_AGG_DED_STATUS								(0x02F7C140u)
#define MSS_MCANA_ECC_AGG_SEC_STATUS	    				(0x02F7F840u)
#define MSS_MCANA_ECC_AGG_DED_STATUS						(0x02F7F940u)
#define MSS_MCANB_ECC_AGG_SEC_STATUS						(0x03F7F840u)
#define MSS_MCANB_ECC_AGG_DED_STATUS						(0x03F7F940u)

#define UNKNOW_MEMTYPE										(49783896u)

#define SDL_DSP_ICFG_DISABLE                                (0)
/*L2 Error Detection Address Register*/
#define SDL_DSP_ICFG_L2EDADDR                               (0x01846008U)

#if defined(SOC_AM273X) || defined(SOC_AWR294X)
#define SDL_DSS_ECC_AGG_ECC_VECTOR_ADDR                     (0x060A0008)
#define SDL_DSS_ECC_AGG_ERROR_STATUS1_ADDR                  (0x060A0020)

#define SDL_DSS_ECC_AGG_RAM_IDS_TOTAL_ENTRIES               22

#define SDL_MSS_ECC_AGGA_ECC_VECTOR_ADDR                    (0x02F7B808)
#define SDL_MSS_ECC_AGGA_ERROR_STATUS1_ADDR                 (0x02F7B820)

#define SDL_MSS_ECC_AGGA_RAM_IDS_TOTAL_ENTRIES              28U

#define SDL_MSS_ECC_AGGB_ECC_VECTOR_ADDR                    (0x02F7BC08)
#define SDL_MSS_ECC_AGGB_ERROR_STATUS1_ADDR                 (0x02F7BC20)

#define SDL_MSS_ECC_AGGB_RAM_IDS_TOTAL_ENTRIES              28U

#define SDL_MSS_ECC_AGG_MSS_ECC_VECTOR_ADDR                 (0x02F7C008)
#define SDL_MSS_ECC_AGG_MSS_ERROR_STATUS1_ADDR              (0x02F7C020)

#define SDL_MSS_ECC_AGG_RAM_IDS_TOTAL_ENTRIES               8U
#endif
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
volatile bool   gMsmcMemParityInterrupt = false;
volatile bool idmaTransferComplete = false;
/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
/* ========================================================================== */
/*                 Internal Function Definitions                              */
/* ========================================================================== */
#ifdef UNITY_INCLUDE_CONFIG_H
/*
 *  ======== Unity set up and tear down ========
 */
void setUp(void)
{
    /* Do nothing */
}

void tearDown(void)
{
    /* Do nothing */
}
#endif

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,
                                            uint32_t index,
                                            uint32_t intSrc,
                                            uintptr_t *arg)
{

    SDL_ECC_MemType eccmemtype;
    SDL_Ecc_AggrIntrSrc eccIntrSrc;
    SDL_ECC_ErrorInfo_t eccErrorInfo;
    int32_t retVal;
	uint32_t rd_data =0U;

    printf("\r\nESM Call back function called : instType 0x%x, intType 0x%x, " \
                "grpChannel 0x%x, index 0x%x, intSrc 0x%x \r\n",
                esmInst, esmIntrType, grpChannel, index, intSrc);
    printf("\r\nTake action \r\n");
    if(esmIntrType == 1u){
        printf("\r\nHigh Priority Interrupt Executed\r\n");
    }
    else{
        printf("\r\nLow Priority Interrupt Executed\r\n");
    }
    retVal = SDL_ECC_getESMErrorInfo(esmInst, intSrc, &eccmemtype, &eccIntrSrc);

	if(((eccmemtype == SDL_R5FSS0_CORE0_ECC_AGGR) || (eccmemtype == SDL_R5FSS0_CORE0_ECC_AGGR) ||
       (eccmemtype == SDL_R5FSS0_CORE0_ECC_AGGR) || (eccmemtype== SDL_R5FSS0_CORE0_ECC_AGGR)) && ((intSrc != 0x33) && (intSrc != 0x35)))
    {
        /* Clear DED MSS_CTRL register*/
        SDL_REG32_WR(SDL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS, SDL_CLEAR_STATUS);
        rd_data = SDL_REG32_RD(SDL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS);
        printf("\r\nRead data of DED MSS_CTRL register is 0x%u\r\n",rd_data);
        /* Clear DED RAW MSS_CTRL register*/
        SDL_REG32_WR(SDL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW, SDL_CLEAR_STATUS);
        rd_data = SDL_REG32_RD(SDL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW);
        printf("\r\nRead data of DED RAW MSS_CTRL register is 0x%u\r\n",rd_data);

        /* Clear SEC MSS_CTRL register*/
        SDL_REG32_WR(SDL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW, SDL_CLEAR_STATUS);
        rd_data = SDL_REG32_RD(SDL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW);
        printf("\r\nRead data of SEC MSS_CTRL register is  0x%u\r\n",rd_data);
        /* Clear SEC RAW MSS_CTRL register*/
        SDL_REG32_WR(SDL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS, SDL_CLEAR_STATUS);
        rd_data = SDL_REG32_RD(SDL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS);
        printf("\r\nRead data of SEC RAW MSS_CTRL register is 0x%u\r\n",rd_data);
    }
    else
    {

		if(intSrc == 0x33U)
		{
			eccmemtype = 1U;
			eccIntrSrc = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
		}
		if(intSrc == 0x35U)
		{
			eccmemtype = 1U;
			eccIntrSrc = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;
		}
        /* Any additional customer specific actions can be added here */
        retVal = SDL_ECC_getErrorInfo(eccmemtype, eccIntrSrc, &eccErrorInfo);

        printf("\r\nECC Error Call back function called : eccMemType %d, errorSrc 0x%x, " \
                   "ramId %d, bitErrorOffset 0x%04x%04x, bitErrorGroup %d\r\n",
                   eccmemtype, eccIntrSrc, eccErrorInfo.memSubType, (uint32_t)(eccErrorInfo.bitErrorOffset >> 32),
                   (uint32_t)(eccErrorInfo.bitErrorOffset & 0x00000000FFFFFFFF), eccErrorInfo.bitErrorGroup);

        if (eccErrorInfo.injectBitErrCnt != 0)
        {
            SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_INJECT, eccErrorInfo.injectBitErrCnt);
        }
        else
        {
            SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_NORMAL, eccErrorInfo.bitErrCnt);
        }

        retVal = SDL_ECC_ackIntr(eccmemtype, eccIntrSrc);
    }

    gMsmcMemParityInterrupt = true;

    return retVal;
}

#endif

#if defined(SOC_AM273X) || defined(SOC_AWR294X)
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                           int32_t grpChannel,
                                           int32_t intSrc,
                                           void *arg)
{

    SDL_ECC_MemType eccmemtype;
    SDL_Ecc_AggrIntrSrc eccIntrSrc;
    SDL_ECC_ErrorInfo_t eccErrorInfo;
    int32_t retVal;
	uint32_t rd_data = 0U;

    printf("\r\nESM Call back function called : instType 0x%x, " \
                "grpChannel 0x%x, intSrc 0x%x \r\n",
                esmInstType, grpChannel, intSrc);
    printf("\r\nTake action \r\n");

    retVal = SDL_ECC_getESMErrorInfo(esmInstType, intSrc, &eccmemtype, &eccIntrSrc);

	if((eccmemtype == SDL_R5FSS0_CORE0_ECC_AGGR) || (eccmemtype == UNKNOW_MEMTYPE))
    {
        rd_data = SDL_REG32_RD(R5FSS0_CORE0_ECC_AGGR_SEC_STATUS);
        printf("\r\nRead data of SEC RAW STATUS register is %d\r\n",rd_data);

        rd_data = SDL_REG32_RD(R5FSS0_CORE0_ECC_AGGR_DED_STATUS);
        printf("\r\nRead data of DED RAW STATUS register is %d\r\n",rd_data);
    }
    else if(eccmemtype == SDL_R5FSS0_CORE1_ECC_AGGR)
    {
        rd_data = SDL_REG32_RD(R5FSS0_CORE1_ECC_AGGR_SEC_STATUS);
        printf("\r\nRead data of SEC RAW STATUS register is %d\r\n",rd_data);

        rd_data = SDL_REG32_RD(R5FSS0_CORE1_ECC_AGGR_DED_STATUS);
        printf("\r\nRead data of DED RAW STATUS register is %d\r\n",rd_data);
    }
    else if(eccmemtype == SDL_MSS_ECC_AGG_MSS)
    {
        rd_data = SDL_REG32_RD(MSS_ECC_AGG_SEC_STATUS);
        printf("\r\nRead data of SEC RAW STATUS register is %d\r\n",rd_data);

        rd_data = SDL_REG32_RD(MSS_ECC_AGG_DED_STATUS);
        printf("\r\nRead data of DED RAW STATUS register is %d\r\n",rd_data);
    }
    else if(eccmemtype == SDL_MSS_MCANA_ECC)
    {
        rd_data = SDL_REG32_RD(MSS_MCANA_ECC_AGG_SEC_STATUS);
        printf("\r\nRead data of SEC RAW STATUS register is %d\r\n",rd_data);

        rd_data = SDL_REG32_RD(MSS_MCANA_ECC_AGG_DED_STATUS);
        printf("\r\nRead data of DED RAW STATUS register is %d\r\n",rd_data);
    }
    else if(eccmemtype == SDL_MSS_MCANB_ECC)
    {
        rd_data = SDL_REG32_RD(MSS_MCANB_ECC_AGG_SEC_STATUS);
        printf("\r\nRead data of SEC RAW STATUS register is %d\r\n",rd_data);

        rd_data = SDL_REG32_RD(MSS_MCANB_ECC_AGG_DED_STATUS);
        printf("\r\nRead data of DED RAW STATUS register is %d\r\n",rd_data);
    }

    else
    {
        /*Nothing*/
    }

    /* Any additional customer specific actions can be added here */
    retVal = SDL_ECC_getErrorInfo(eccmemtype, eccIntrSrc, &eccErrorInfo);

    printf("\r\nECC Error Call back function called : eccMemType %d, errorSrc 0x%x, " \
               "ramId %d, bitErrorOffset 0x%04x%04x, bitErrorGroup %d\r\n",
               eccmemtype, eccIntrSrc, eccErrorInfo.memSubType, (uint32_t)(eccErrorInfo.bitErrorOffset >> 32),
               (uint32_t)(eccErrorInfo.bitErrorOffset & 0x00000000FFFFFFFF), eccErrorInfo.bitErrorGroup);

    if (eccErrorInfo.injectBitErrCnt != 0)
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_INJECT, eccErrorInfo.injectBitErrCnt);
    }
    else
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_NORMAL, eccErrorInfo.bitErrCnt);
    }

    retVal = SDL_ECC_ackIntr(eccmemtype, eccIntrSrc);

    gMsmcMemParityInterrupt = true;

    return retVal;
}
#if defined(SOC_AM273X)
#if defined(C66_INPUTS)
int32_t SDL_ESM_DSP_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                           int32_t grpChannel,
                                           int32_t intSrc,
                                           void *arg)
{

    DebugP_log("\r\nESM Call back function called : instType 0x%x, " \
                "grpChannel 0x%x, intSrc 0x%x \r\n",
                esmInstType, grpChannel, intSrc);
    DebugP_log("\r\nTake action \r\n");

    /* Write to DSP_ICFG__EDCINTMASK REGISTER and DSP_ICFG__EDCINTFLG REGISTER
     * TO disable PROPOGATION OF EXCEPTION
     */
    if((intSrc == SDL_DSS_ESMG1_DSS_DSP_EDC_SEC_ERR) || (intSrc == SDL_DSS_ESMG1_DSS_DSP_EDC_DED_ERR))
    {
        SDL_REG32_WR(SDL_DSP_ICFG_EDCINTMASK, SDL_DSP_ICFG_DISABLE);
        SDL_REG32_WR(SDL_DSP_ICFG_EDCINTFLG, SDL_DSP_ICFG_DISABLE);
        gMsmcMemParityInterrupt = true;
    }
    else if((intSrc == SDL_DSS_ESMG1_DSS_DSP_L2_SEC_ERR) || (intSrc == SDL_DSS_ESMG1_DSS_DSP_L2_DED_ERR))
    {
        /*Clear Error Detection Address Register*/
        SDL_REG32_WR(SDL_DSP_ICFG_L2EDADDR, 0x00u);
        gMsmcMemParityInterrupt = true;
    }
    else if(intSrc == SDL_DSS_ESMG1_DSS_DSP_L1P_PARITY)
    {
        gMsmcMemParityInterrupt = true;
    }
    else if(intSrc == SDL_DSS_ESMG1_DSS_DSP_L2_PARITY_ERR_VB0_EVEN)
    {
        /*
         *Disable the parity by clearing DSS_CTRL.DSS_DSP_L2RAM_PARITY_CTRL.DSS_DSP_L2RAM_PARITY_CTRL_ENABLE
         *Disable register field
         */
        SDL_REG32_WR(SDL_DSS_DSP_L2RAM_PARITY_CTRL, SDL_DSS_L2RAM_PARITY_ERROR_CLEAR);
        SDL_REG32_WR(SDL_DSS_DSP_L2RAM_PARITY_CTRL, SDL_DSS_L2RAM_PARITY_ERROR_CLEAR);

        DebugP_log("\r\nclearing DSS_CTRL.DSS_DSP_L2RAM_PARITY_CTRL.DSS_DSP_L2RAM_PARITY_CTRL_ENABLE\r\n");
        gMsmcMemParityInterrupt = true;
    }
    else
    {
        /*Nothing*/
    }

    return 0;
}/* End of SDL_ESM_DSP_applicationCallbackFunction() */

int32_t SDL_ECC_DED_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                           int32_t grpChannel,
                                           int32_t intSrc,
                                           void *arg)
{

    DebugP_log("\r\nECC DED Call back function called : instType 0x%x, " \
                "grpChannel 0x%x, intSrc 0x%x \r\n",
                esmInstType, grpChannel, intSrc);
    DebugP_log("\r\nTake action \r\n");

    /* Write to DSP_ICFG__EDCINTMASK REGISTER and DSP_ICFG__EDCINTFLG REGISTER
     * TO disable PROPOGATION OF EXCEPTION
     */

    SDL_REG32_WR(SDL_DSP_ICFG_EDCINTMASK, SDL_DSP_ICFG_DISABLE);
    SDL_REG32_WR(SDL_DSP_ICFG_EDCINTFLG, SDL_DSP_ICFG_DISABLE);

    gMsmcMemParityInterrupt = true;

    return 0;
}/* End of SDL_ECC_DED_applicationCallbackFunction() */

int32_t SDL_ECC_SEC_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                           int32_t grpChannel,
                                           int32_t intSrc,
                                           void *arg)
{

    DebugP_log("\r\nECC SEC Call back function called : instType 0x%x, " \
                "grpChannel 0x%x, intSrc 0x%x \r\n",
                esmInstType, grpChannel, intSrc);
    DebugP_log("\r\nTake action \r\n");

    /* Write to DSP_ICFG__EDCINTMASK REGISTER and DSP_ICFG__EDCINTFLG REGISTER
     * TO disable PROPOGATION OF EXCEPTION
     */

    SDL_REG32_WR(SDL_DSP_ICFG_EDCINTMASK, SDL_DSP_ICFG_DISABLE);
    SDL_REG32_WR(SDL_DSP_ICFG_EDCINTFLG, SDL_DSP_ICFG_DISABLE);

    gMsmcMemParityInterrupt = true;

    return 0;
}/* End of SDL_ECC_SEC_applicationCallbackFunction() */

int32_t SDL_IDMA1_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                           int32_t grpChannel,
                                           int32_t intSrc,
                                           void *arg)
{
    printf("\r\nIDMA1 call back function called. \r\n");

    idmaTransferComplete = true;

    return 0;
}/* End of SDL_IDMA1_applicationCallbackFunction() */
#endif
#endif
#endif
void SDL_ECC_applicationCallbackFunction(SDL_ECC_MemType eccMemType,
                                         uint32_t errorSrc,
                                         uint32_t address,
                                         uint32_t ramId,
                                         uint64_t bitErrorOffset,
                                         uint32_t bitErrorGroup){

    DebugP_log("\r\nECC Error Call back function called : eccMemType %d, errorSrc 0x%x, " \
                "address 0x%x, ramId %d, bitErrorOffset 0x%04x%04x, bitErrorGroup %d\r\n",
                eccMemType, errorSrc, address, ramId, (uint32_t)(bitErrorOffset >> 32),
                (uint32_t)(bitErrorOffset & 0x00000000FFFFFFFF), bitErrorGroup);
    DebugP_log("\r\nTake action \r\n");

    /* Any additional customer specific actions can be added here */

}

static int32_t sdlApp_dplInit(void)
{
    SDL_ErrType_t ret = SDL_PASS;

    ret = SDL_TEST_dplInit();
    if (ret != SDL_PASS)
    {
        DebugP_log("\r\nError: Init Failed\r\n");
    }

    return ret;
}

static int32_t ECC_appTest(uint32_t testId)
{
    int32_t testResult=0;
#if defined(SOC_AM273X) || defined(SOC_AWR294X)
    uint8_t i;
#endif

    switch (testId)
    {
        case ECC_ERROR_TEST_ID:
#if defined(SOC_AM263X) || defined(SOC_AM273X) || defined(SOC_AWR294X) || defined(SOC_AM263PX) || defined (SOC_AM261X)
#if defined(R5F_INPUTS)
#if defined(SOC_AM273X) || defined(SOC_AWR294X)
            /* Clear all status registers of MSS AGGRA ECC AGGR.*/
            for(i=0;i<=SDL_MSS_ECC_AGGA_RAM_IDS_TOTAL_ENTRIES;i++)
            {
                /* Write the RAM ID in to Vector register*/
                SDL_REG32_FINS(SDL_MSS_ECC_AGGA_ECC_VECTOR_ADDR, MSS_ECC_AGGA_ECC_VECTOR_ECC_VECTOR, i);
                /* Clear pending interrupts.*/
                SDL_REG32_WR(SDL_MSS_ECC_AGGA_ERROR_STATUS1_ADDR, 0xF0F);
                ClockP_usleep(100);
            }

            /* Clear all status registers of MSS AGGRB ECC AGGR.*/
            for(i=0;i<=SDL_MSS_ECC_AGGB_RAM_IDS_TOTAL_ENTRIES;i++)
            {
                /* Write the RAM ID in to Vector register*/
                SDL_REG32_FINS(SDL_MSS_ECC_AGGB_ECC_VECTOR_ADDR, MSS_ECC_AGGB_ECC_VECTOR_ECC_VECTOR, i);
                /* Clear pending interrupts.*/
                SDL_REG32_WR(SDL_MSS_ECC_AGGB_ERROR_STATUS1_ADDR, 0xF0F);
                ClockP_usleep(100);
            }

            /* Clear all status registers of MSS ECC AGGR.*/
            for(i=0;i<=SDL_MSS_ECC_AGG_RAM_IDS_TOTAL_ENTRIES;i++)
            {
                /* Write the RAM ID in to Vector register*/
                SDL_REG32_FINS(SDL_MSS_ECC_AGG_MSS_ECC_VECTOR_ADDR, MSS_ECC_AGG_MSS_ECC_VECTOR_ECC_VECTOR, i);
                /* Clear pending interrupts.*/
                SDL_REG32_WR(SDL_MSS_ECC_AGG_MSS_ERROR_STATUS1_ADDR, 0xF0F);
                ClockP_usleep(100);
            }
#endif
            testResult = ECC_ip_errTest();
#elif defined(C66_INPUTS)
            /* Clear all status registers.*/
            for(i=0;i<=SDL_DSS_ECC_AGG_RAM_IDS_TOTAL_ENTRIES;i++)
            {
                /* Write the RAM ID in to Vector register*/
                SDL_REG32_FINS(SDL_DSS_ECC_AGG_ECC_VECTOR_ADDR, DSS_ECC_AGG_ECC_VECTOR_ECC_VECTOR, i);
                /* Clear pending interrupts.*/
                SDL_REG32_WR(SDL_DSS_ECC_AGG_ERROR_STATUS1_ADDR, 0xF0F);
                ClockP_usleep(100);
            }
            testResult = DSS_ECC_ip_errTest();
#endif
            DebugP_log("\r\nECC Error ip Module Unit Test\r\n");
            if (testResult == SDL_PASS)
            {
                DebugP_log("\r\nip Module unit test Passed.\r\n");
            }
            else
            {
                DebugP_log("\r\nip Module unit test Failed.\r\n");
            }
#if defined(R5F_INPUTS)
            testResult = ECC_r5_errTest();
            DebugP_log("\nECC Error r5 Module Unit Test");
            if (testResult == SDL_PASS)
            {
                DebugP_log("\r\nr5 Module unit test Passed.\r\n");
            }
            else
            {
                DebugP_log("\r\nr5 Module unit test Failed.\r\n");
            }
#endif
#if defined(R5F_INPUTS)
            testResult = ECC_errTest();
#elif defined(C66_INPUTS)
            testResult = DSS_ECC_errTest();
#endif
            DebugP_log("\r\nECC Error SDL Module Unit Test\r\n");
            if (testResult == SDL_PASS)
            {
                DebugP_log("\r\nSDL Module unit test Passed.\r\n");
            }
            else
            {
                DebugP_log("\r\nSDL Module unit test Failed.\r\n");
            }
#endif
            break;

        case ECC_FUNC_TEST_ID:
#if defined(SOC_AM263X) || defined(SOC_AM273X) || defined(SOC_AWR294X) || defined(SOC_AM263PX) || defined (SOC_AM261X)
#if defined(R5F_INPUTS)
            testResult = ECC_ip_funcTest();
#elif defined(C66_INPUTS)
            testResult = DSS_ECC_ip_funcTest();
#endif
            DebugP_log("\r\nECC ip func Test\r\n");
            if (testResult == SDL_PASS)
            {
                DebugP_log("\r\nAll ip tests passed. \r\n");
            }
            else
            {
                DebugP_log("\r\nSome ip tests failed. \r\n");
            }

#if defined(R5F_INPUTS)
            testResult = ECC_r5_funcTest();
            DebugP_log("\r\nECC r5 func Test\r\n");
            if (testResult == SDL_PASS)
            {
                DebugP_log("\r\nAll r5 tests passed. \r\n");
            }
            else
            {
                DebugP_log("\r\nSome r5 tests failed. \r\n");
            }
#endif
#if defined(R5F_INPUTS)
            testResult = ECC_sdl_funcTest();
#elif defined(C66_INPUTS)
            testResult = DSS_ECC_sdl_funcTest();
#endif
            DebugP_log("\r\nECC sdl func Test\r\n");
            if (testResult == SDL_PASS)
            {
                DebugP_log("\r\nAll sdl tests passed. \r\n");
            }
            else
            {
                DebugP_log("\r\nSome sdl tests failed. \r\n");
            }
#if defined(SOC_AM273X)
#if defined(C66_INPUTS)
            testResult = DSS_sdl_funcTest();
            if (testResult == SDL_PASS)
            {
                DebugP_log("\r\nAll DSS Parity and EDC tests passed. \r\n");
            }
            else
            {
                DebugP_log("\r\nSome DSS Parity and EDC tests failed. \r\n");
            }
#endif
#endif
            break;
#endif
        default:
            DebugP_log("\r\n[Error] Invalid ECC test ID.\r\n");
            testResult = SDL_EFAIL;
            break;
    }

    return (testResult);
}

void test_sdl_ecc_test_app(void *args)
{
    uint32_t testId;
    int32_t  testResult;

    for (testId = ((uint32_t)(0U)); testId < ECC_TOTAL_NUM_TESTS; testId++)
    {
        testResult = ECC_appTest(testId);
        if (testResult != SDL_PASS)
        {
            break;
        }
    }

    if (testResult == SDL_PASS)
    {
        DebugP_log("\r\nAll tests have passed. \r\n");
#ifdef UNITY_INCLUDE_CONFIG_H
        TEST_PASS();
#endif
    }
    else
    {
        DebugP_log("\r\nSome tests have failed. \r\n");
#ifdef UNITY_INCLUDE_CONFIG_H
        TEST_FAIL();
#endif
    }
}

void test_sdl_ecc_test_app_runner(void)
{
#ifdef UNITY_INCLUDE_CONFIG_H
    UNITY_BEGIN();
    RUN_TEST(test_sdl_ecc_test_app,0,NULL);
    UNITY_END();
#else
    test_sdl_ecc_test_app();
#endif
}

void ecc_syncup_delay(void)
{
    uint32_t maxTimeOutMilliSeconds = 0;
    for(maxTimeOutMilliSeconds = 0; maxTimeOutMilliSeconds < SDL_DELAY; maxTimeOutMilliSeconds++);
}

void test_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    sdlApp_dplInit();

    test_sdl_ecc_test_app_runner();

    /* Close drivers to close the UART driver for console */
    Board_driversClose();
    Drivers_close();
}

/* Nothing past this point */
