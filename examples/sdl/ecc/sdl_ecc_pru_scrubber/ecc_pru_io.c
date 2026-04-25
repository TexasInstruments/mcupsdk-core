/*
 *  Copyright (C) 2026 Texas Instruments Incorporated
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

 /*
 *  This is an example project to show R5F
 *  loading PRU firmware.
 */
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/pruicss.h>

#include "pru0_load_bin.h"

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
#define  DMEM_PARAMS_OFFSET      0
/*4 beats and each beat is of 4 bytes*/
#define  PRU_READ_BURST_SIZE     4*4
#define  SDL_ECC_RAM_TEST_ADDR   0x70000008U
#define  SDL_ECC_RAM_NUM_BYTES   0x40000U

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
typedef struct{
    uint32_t             startAddress;
    uint16_t             numOfIterations;
    volatile uint8_t     hostTriggerBit;
    /*This bit is set by PRU to indicate R5F not to reset */
    volatile uint8_t     pruTriggerBit;
}dmemParameters;

dmemParameters *gDmemParams;

/** \brief Global Structure pointer holding PRUSS1 memory Map. */
PRUICSS_Handle gPruIcss0Handle;

/* ========================================================================== */
/*                 External Function Declarations                             */
/* ========================================================================== */
extern void ecc_pru_io_main(void *args);
extern int32_t ECC_funcTest(void);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
uint8_t scrub_mem(uint32_t startAddress, uint32_t numOfBytes)
{
    /**
     *NOTE : Maximum bytes that can be read is 0x73FF8C with 16 byte burst read (0x73FF8C/16 = 0x73FF8)
     *as maximum number of iterations using PRU loop instruction is 0x73FF8
     */

    if(numOfBytes > 0x73FF8)
    {
        return SystemP_FAILURE;
    }

    /*TODO : configure vector ID*/

    gDmemParams->startAddress = startAddress;
    gDmemParams->numOfIterations = (numOfBytes + PRU_READ_BURST_SIZE - 1) / PRU_READ_BURST_SIZE;
    return SystemP_SUCCESS;
}

void ecc_pru_io_main(void *args)
{
     Drivers_open(); // check return status

     int status;
     status = Board_driversOpen();
     DebugP_assert(SystemP_SUCCESS == status);

     gPruIcss0Handle = PRUICSS_open(CONFIG_PRU_ICSS0);

     gDmemParams = (dmemParameters *)(gPruIcss0Handle->hwAttrs->pru0DramBase + DMEM_PARAMS_OFFSET);

     status = PRUICSS_initMemory(gPruIcss0Handle, PRUICSS_DATARAM(PRUICSS_PRU0));
     DebugP_assert(status != 0);

     status = scrub_mem(SDL_ECC_RAM_TEST_ADDR, SDL_ECC_RAM_NUM_BYTES);   /*Scrub initializations*/
     DebugP_assert(SystemP_SUCCESS == status);

     ECC_funcTest();

     status = PRUICSS_loadFirmware(gPruIcss0Handle, PRUICSS_PRU0, PRU0Firmware_0, sizeof(PRU0Firmware_0));
     DebugP_assert(SystemP_SUCCESS == status);

     gDmemParams->pruTriggerBit = 0;
     while(gDmemParams->pruTriggerBit == 0);

     DebugP_log("\r\nPRU updated pruTriggerBit value as 1\r\n");
     while (1)
     {
        ClockP_usleep(1);
     }

     Board_driversClose();
     Drivers_close();
}

/* Nothing past this point */
