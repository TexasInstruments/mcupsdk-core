/* Copyright (c) 2023 Texas Instruments Incorporated
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
 *  \file     test_api.c
 *
 *  \brief    This file contains sdl_r5_utils.asm test code.
 *
 *  \details   API tests
 **/



#include "test_main.h"
#include "unity.h"
#include "unity_config.h"



#define SDL_MPU_REGION_MAX     4U

/*===========================================================================*/
/*                         Declarations                                      */
/*===========================================================================*/

/* Declaration of Global structure to contain all register values*/
SDL_R5FCPU_StaticRegs  pCPUStaticRegs;
SDL_R5MPU_staticRegs   pMPUStaticRegs;

int32_t  sdl_apiTest(void);
int32_t  sdl_apiNegTest(void);
void    print_CPU_RegisterValue(SDL_R5FCPU_StaticRegs  *pCPUStaticRegs);

/*===========================================================================*/
/*                         Local Function definitions                        */
/*===========================================================================*/
void  print_CPU_RegisterValue(SDL_R5FCPU_StaticRegs  *pCPUStaticRegs)
{
    DebugP_log("The SCTLR register value  is  0x%x \r\n",pCPUStaticRegs->SCTLR);
    DebugP_log("The ACTLR register value  is  0x%x \r\n",pCPUStaticRegs->ACTLR);
    DebugP_log("The SecondaryACTLR register value  is  0x%x \r\n",pCPUStaticRegs->SecondaryACTLR);
    DebugP_log("The CPACR register value  is  0x%x \r\n",pCPUStaticRegs->CPACR);
    DebugP_log("The BTCMRegionR register value  is  0x%x \r\n", pCPUStaticRegs->BTCMRegionR);
    DebugP_log("The ATCMRegionR register value  is  0x%x \r\n",pCPUStaticRegs->ATCMRegionR);
    DebugP_log("The SlavePortControlR register value  is  0x%x \r\n",pCPUStaticRegs->SlavePortControlR);
    DebugP_log("The CONTEXTIDR register value  is  0x%x \r\n",pCPUStaticRegs->CONTEXTIDR);
    DebugP_log("The nVALIRQSET register value  is  0x%x \r\n",pCPUStaticRegs->nVALIRQSET);
    DebugP_log("The nVALFIQSET register value  is  0x%x \r\n",pCPUStaticRegs->nVALFIQSET);
    DebugP_log("The nVALRESETSET register value  is  0x%x \r\n",pCPUStaticRegs->nVALRESETSET);
    DebugP_log("The nVALDEBUGSET register value  is  0x%x \r\n",pCPUStaticRegs->nVALDEBUGSET);
    DebugP_log("The nVALIRQCLEAR register value  is  0x%x \r\n",pCPUStaticRegs->nVALIRQCLEAR);
    DebugP_log("The nVALFIQCLEAR register value  is  0x%x \r\n",pCPUStaticRegs->nVALFIQCLEAR);
    DebugP_log("The nVALRESETCLEAR register value  is  0x%x \r\n",pCPUStaticRegs->nVALRESETCLEAR);
    DebugP_log("The nVALDEBUGCLEAR register value  is  0x%x \r\n",pCPUStaticRegs->nVALDEBUGCLEAR);
    DebugP_log("The BuildOption1R register value  is  0x%x \r\n",pCPUStaticRegs->BuildOption1R);
    DebugP_log("The BuildOption2R register value  is  0x%x \r\n",pCPUStaticRegs->BuildOption2R);
    DebugP_log("The PinOptionR register value  is  0x%x \r\n",pCPUStaticRegs->PinOptionR);
    DebugP_log("The LLPPnormalAXIRR register value  is  0x%x \r\n",pCPUStaticRegs->LLPPnormalAXIRR);
    DebugP_log("The LLPPvirtualAXIRR register value  is  0x%x \r\n",pCPUStaticRegs->LLPPvirtualAXIRR);
    DebugP_log("The AHBRR register value  is  0x%x \r\n",pCPUStaticRegs->AHBRR);
    DebugP_log("The PMCNTENSET register value  is  0x%x \r\n",pCPUStaticRegs->PMCNTENSET);
    DebugP_log("The PMCR register value  is  0x%x \r\n",pCPUStaticRegs->PMCR);
    DebugP_log("The PMUSERENR register value  is  0x%x \r\n",pCPUStaticRegs->PMUSERENR);
    DebugP_log("The PMINTENSET register value  is  0x%x \r\n",pCPUStaticRegs->PMINTENSET);
    DebugP_log("The PMINTENCLR register value  is  0x%x \r\n",pCPUStaticRegs->PMINTENCLR);
}
int32_t sdl_apiTest(void)
{
    int32_t testResult=0;
    uint32_t SDL_MPU_region;
    /*Read all R5F cpu static registers*/
    testResult = SDL_CPU_staticRegisterRead(&pCPUStaticRegs);

    for(SDL_MPU_region = 0; SDL_MPU_region < SDL_MPU_REGION_MAX; SDL_MPU_region++)
    {
        SDL_R5MPU_readStaticRegisters(&pMPUStaticRegs, SDL_MPU_region);
        DebugP_log("The MPU Register read started for MPU REGION -> 0x%d \r\n\n",SDL_MPU_region);

        DebugP_log("The SCTLR register value  is  0x%x \r\n",pMPUStaticRegs.sysControlReg);
        DebugP_log("The MPUIR register value  is  0x%x \r\n",pMPUStaticRegs.mpuTypeReg);
        DebugP_log("The RGNR register value  is  0x%x \r\n",pMPUStaticRegs.regionId);
        DebugP_log("The MPURbaseAddr register value  is  0x%x \r\n",pMPUStaticRegs.baseAddr);
        DebugP_log("The MPURsize register value  is  0x%x \r\n",pMPUStaticRegs.size);
        DebugP_log("The MPURaccessControl register value  is  0x%x \r\n\n",pMPUStaticRegs.accessPermission);
    }

    if (testResult==SDL_PASS)
    {
        DebugP_log("All the R5F MPU Static register read are complete. \r\n\n");
        testResult = SDL_PASS;
    }
    else
    {
        DebugP_log("Some MPU Register read are fail. \r\n\n");
        testResult = SDL_EBADARGS;
    }
    /*Print All register values*/
    print_CPU_RegisterValue(&pCPUStaticRegs);

	if(testResult == 0)
  	{
		testResult=SDL_UTILS_getCFLR();

		if (testResult != SDL_PASS)
		{
			DebugP_log("\n  SDL_UTILS_getCFLR API test failed on line no: %d\r\n", __LINE__);
			testResult = -1;
		}
	}
	if(testResult == 0)
  	{
		testResult=SDL_UTILS_getPMOVSR();

		if (testResult != SDL_PASS)
		{
			DebugP_log("\n  SDL_UTILS_getPMOVSR API test failed on line no: %d\r\n", __LINE__);
			testResult = -1;
		}
	}
	if(testResult == 0)
  	{
		testResult=SDL_UTILS_getDFSR();

		if (testResult != SDL_PASS)
		{
			DebugP_log("\n SDL_UTILS_getDFSR API test failed on line no: %d\r\n", __LINE__);
			testResult = -1;
		}
	}
	if(testResult == 0)
  	{
		testResult=SDL_UTILS_getDFAR();

		if (testResult != SDL_PASS)
		{
			DebugP_log("\n SDL_UTILS_getDFAR API test failed on line no: %d\r\n", __LINE__);
			testResult = -1;
		}
	}
	if(testResult == 0)
  	{
		testResult=SDL_UTILS_getIFSR();

		if (testResult != SDL_PASS)
		{
			DebugP_log("\n SDL_UTILS_getIFSR API test failed on line no: %d\r\n", __LINE__);
			testResult = -1;
		}
	}
	if(testResult == 0)
  	{
		testResult=SDL_UTILS_getIFAR();

		if (testResult != SDL_PASS)
		{
			DebugP_log("\n SDL_UTILS_getIFAR API test failed on line no: %d\r\n", __LINE__);
			testResult = -1;
		}
	}

	return (testResult);

}

int32_t sdl_apiNegTest(void)
{
    int32_t testResult=0;

    /*Read all R5F cpu static registers with negative arguments*/
    testResult = SDL_CPU_staticRegisterRead(NULL);

    if (testResult==SDL_PASS)
    {
        DebugP_log("\n SDL_CPU_staticRegisterRead API Neg test failed on line no: %d\r\n", __LINE__);
        testResult = -1;
    }
    else
    {
        testResult=SDL_PASS;
    }

    return (testResult);

}

/* Nothing past this point */
