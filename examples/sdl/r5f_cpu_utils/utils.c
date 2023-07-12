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
 *  \file     utils.c
 *
 *  \brief    This file contains sdl_r5_utils.asm example code.
 *            All the registers are being read, whcih all can be found
 *            ARM CORTEX R5F TRM in System Control section.
 *
 *  \details   API tests
 **/

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <sdl/r5/v0/sdl_r5_utils.h>
#include <sdl/dpl/sdl_dpl.h>
#include <dpl_interface.h>
#include <kernel/dpl/DebugP.h>

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/*===========================================================================*/
/*                         Declarations                                      */
/*===========================================================================*/

/* Declaration of Global structure to contain all register values*/
SDL_R5FCPU_StaticRegs  pCPUStaticRegs;

int32_t  sdl_apiTest(void);
void    print_CPU_RegisterValue(SDL_R5FCPU_StaticRegs  *pCPUStaticRegs);

/*===========================================================================*/
/*                         Local Function definitions                        */
/*===========================================================================*/

void  print_CPU_RegisterValue(SDL_R5FCPU_StaticRegs  *pCPUStaticRegs)
{
    DebugP_log("The MIDR register value  is  0x%x \r\n",pCPUStaticRegs->MIDR);
    DebugP_log("The CTR register value  is  0x%x \r\n",pCPUStaticRegs->CTR);
    DebugP_log("The TCMTR register value  is  0x%x \r\n",pCPUStaticRegs->TCMTR);
    DebugP_log("The MPUIR register value  is  0x%x \r\n",pCPUStaticRegs->MPUIR);
    DebugP_log("The MPIDR register value  is  0x%x \r\n",pCPUStaticRegs->MPIDR);
    DebugP_log("The PFR0 register value  is  0x%x \r\n",pCPUStaticRegs->PFR0);
    DebugP_log("The PFR1 register value  is  0x%x \r\n",pCPUStaticRegs->PFR1);
    DebugP_log("The ID_DFR0 register value  is  0x%x \r\n",pCPUStaticRegs->ID_DFR0);
    DebugP_log("The ID_AFR0 register value  is  0x%x \r\n",pCPUStaticRegs->ID_AFR0);
    DebugP_log("The ID_MMFR0 register value  is  0x%x \r\n",pCPUStaticRegs->ID_MMFR0);
    DebugP_log("The ID_MMFR1 register value  is  0x%x \r\n",pCPUStaticRegs->ID_MMFR1);
    DebugP_log("The ID_MMFR2 register value  is  0x%x \r\n",pCPUStaticRegs->ID_MMFR2);
    DebugP_log("The ID_MMFR3 register value  is  0x%x \r\n",pCPUStaticRegs->ID_MMFR3);
    DebugP_log("The ID_ISAR0 register value  is  0x%x \r\n",pCPUStaticRegs->ID_ISAR0);
    DebugP_log("The ID_ISAR1 register value  is  0x%x \r\n",pCPUStaticRegs->ID_ISAR1);
    DebugP_log("The ID_ISAR2 register value  is  0x%x \r\n",pCPUStaticRegs->ID_ISAR2);
    DebugP_log("The ID_ISAR3 register value  is  0x%x \r\n",pCPUStaticRegs->ID_ISAR3);
    DebugP_log("The ID_ISAR4 register value  is  0x%x \r\n",pCPUStaticRegs->ID_ISAR4);
    DebugP_log("The ID_ISAR5 register value  is  0x%x \r\n",pCPUStaticRegs->ID_ISAR5);
    DebugP_log("The CCSIDR register value  is  0x%x \r\n",pCPUStaticRegs->CCSIDR);
    DebugP_log("The CLIDR register value  is  0x%x \r\n",pCPUStaticRegs->CLIDR);
    DebugP_log("The AIDR register value  is  0x%x \r\n",pCPUStaticRegs->AIDR);
    DebugP_log("The CSSELR register value  is  0x%x \r\n",pCPUStaticRegs->CSSELR);
    DebugP_log("The SCTLR register value  is  0x%x \r\n",pCPUStaticRegs->SCTLR);
    DebugP_log("The ACTLR register value  is  0x%x \r\n",pCPUStaticRegs->ACTLR);
    DebugP_log("The SecondaryACTLR register value  is  0x%x \r\n",pCPUStaticRegs->SecondaryACTLR);
    DebugP_log("The CPACR register value  is  0x%x \r\n",pCPUStaticRegs->CPACR);
    DebugP_log("The MPURegionBaseADDR register value  is  0x%x \r\n",pCPUStaticRegs->MPURegionBaseADDR);
    DebugP_log("The MPURegionEnableR register value  is  0x%x \r\n",pCPUStaticRegs->MPURegionEnableR);
    DebugP_log("The MPURegionAccessControlR register value  is  0x%x \r\n",pCPUStaticRegs->MPURegionAccessControlR);
    DebugP_log("The RGNR register value  is  0x%x \r\n",pCPUStaticRegs->RGNR);
    DebugP_log("The BTCMRegionR register value  is  0x%x \r\n", pCPUStaticRegs->BTCMRegionR);
    DebugP_log("The ATCMRegionR register value  is  0x%x \r\n",pCPUStaticRegs->ATCMRegionR);
    DebugP_log("The SlavePortControlR register value  is  0x%x \r\n",pCPUStaticRegs->SlavePortControlR);
    DebugP_log("The CONTEXTIDR register value  is  0x%x \r\n",pCPUStaticRegs->CONTEXTIDR);
    DebugP_log("The ThreadProcessIDR1 register value  is  0x%x \r\n",pCPUStaticRegs->ThreadProcessIDR1);
    DebugP_log("The ThreadProcessIDR2 register value  is  0x%x \r\n",pCPUStaticRegs->ThreadProcessIDR2);
    DebugP_log("The ThreadProcessIDR3 register value  is  0x%x \r\n",pCPUStaticRegs->ThreadProcessIDR3);
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
    DebugP_log("The CFLR register value  is  0x%x \r\n",pCPUStaticRegs->CFLR);
    DebugP_log("The PMOVSR register value  is  0x%x \r\n",pCPUStaticRegs->PMOVSR);
    DebugP_log("The DFSR register value  is  0x%x \r\n",pCPUStaticRegs->DFSR);
    DebugP_log("The ADFSR register value  is  0x%x \r\n",pCPUStaticRegs->ADFSR);
    DebugP_log("The DFAR register value  is  0x%x \r\n",pCPUStaticRegs->DFAR);
    DebugP_log("The IFSR register value  is  0x%x \r\n",pCPUStaticRegs->IFSR);
    DebugP_log("The IFAR register value  is  0x%x \r\n", pCPUStaticRegs->IFAR);
    DebugP_log("The AIFSR register value  is  0x%x \r\n",pCPUStaticRegs->AIFSR);
}

int32_t sdl_apiTest(void)
{
    int32_t sdlResult = SDL_EBADARGS;
    /*Read all R5F cpu static registers*/
    sdlResult = SDL_CPU_staticRegisterRead(&pCPUStaticRegs);

    if (sdlResult==SDL_PASS)
    {
        DebugP_log("All the register read are complete. \r\n");
        sdlResult = SDL_PASS;
    }
    else
    {
        DebugP_log("Some Register read are fail. \r\n");
        sdlResult = SDL_EBADARGS;
    }
    /*Print All register values*/
    print_CPU_RegisterValue(&pCPUStaticRegs);

    return sdlResult;

}

void test_main(void *args)
{
    int32_t sdlResult = SDL_EBADARGS;
    Drivers_open();
    Board_driversOpen();

    DebugP_log("R5F CPU STATIC REGISTER READ Start... \r\n\n");

    sdlResult = sdl_apiTest();
    if (sdlResult==SDL_PASS)
    {
        DebugP_log("\nR5F CPU STATIC REGISTER READ Complete! \r\n");
        DebugP_log("\nAll test have passed. \r\n");
    }
    else
    {
        DebugP_log("\nR5F CPU STATIC REGISTER READ failed! \r\n");
        DebugP_log("\nSome/All test have failed! \r\n");
    }


    Board_driversClose();
    Drivers_close();
}

/* Nothing past this point */
