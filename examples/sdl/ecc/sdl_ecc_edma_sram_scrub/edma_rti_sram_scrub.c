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
 * This example Scrubs(reads) the first 3 SRAM banks using EDMA transfers
 * triggered periodically by RTI. Each bank is 512 KB in size.
 * EDMA transfers chunks of 4 x 64-bit beats on each trigger.
 * The RTI is configured to trigger every 50 us (set in sysconfig).
 * 
 * Instruction(text) and constant data is placed in these first 3 SRAM banks
 * whereas the rest of the data is placed in the 4th SRAM bank.
 * EDMA read chunk data is copied to a separate buffer in TCM.
 * (Memory configuration in syscfg).
 * 
 * In case of SEC interrupt, the ECC fault address is compared with the
 * current EDMA read chunk address range. If it falls under this range,
 * the EDMA read buffer data is compared with the SRAM data. If there is
 * a data mismatch, it indicates that the EDMA read the wrong data
 * and hence the scrub is successful. The ECC error is cleared in this case.
 * If there is no data mismatch, it indicates that CPU may have read the wrong 
 * data and hence RESET the device would be required.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/edma.h>
#include "ti_drivers_config.h"
#include "ti_dpl_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "edma_rti_sram_scrub.h"
#include "string.h"
#include <sdl/include/sdl_types.h>

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
volatile uint8_t gBufferCpy[APP_CHUNK_SIZE_BYTES] __attribute__((aligned(64U), section(".data.tcma"))) = { 0U };
volatile bool edmaReadWrongData = false;
volatile uint8_t CB_Complete =0x00;

/* ========================================================================== */
/*                 External Function Declarations                             */
/* ========================================================================== */
extern int32_t ECC_Test_run_MSS_L2RAMB_1BitInjectTest(void);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
void App_serviceSecInterrupt(void)
{
    uint32_t eccSecStatus = App_getEccSecStatus();

    for(uint8_t bankIdx = 0U; bankIdx < APP_SRAM_SCRUB_NUM_BANKS; bankIdx++)
    {
        if (App_checkEccSecStatusPending(bankIdx, eccSecStatus))
        {
            edmaReadWrongData = false;

            uint32_t edmaChunkStartAddr, edmaChunkEndAddr;

            uint32_t eccFaultAddr = App_getEccFaultAddr(bankIdx);

            App_getEdmaChunkAddr(&edmaChunkStartAddr, &edmaChunkEndAddr);

            /* Check if ECC fault address falls under the EDMA read chunk address range */
            if((eccFaultAddr >= edmaChunkStartAddr) && (eccFaultAddr < edmaChunkEndAddr))
            {
                /* Read and compare the data with EDMA read buffer */
                uint8_t* edmaBuffAddr = App_getEdmaBuffAddr();

               memcpy((void*)gBufferCpy, edmaBuffAddr, APP_CHUNK_SIZE_BYTES);

                if(memcmp((void*)gBufferCpy, (void*)edmaChunkStartAddr, APP_CHUNK_SIZE_BYTES) != 0)
                {
                    /* Data mismatch - EDMA read the wrong data - Scrub successful. */
                    
                    edmaReadWrongData = true;

                    App_clearEccFault(bankIdx);
                }
            }
            
            if(edmaReadWrongData == false)
            {
                /* RESET the device. CPU may have read wrong data */
            }

            CB_Complete = 0xFF;
            /* Disable ESM event to print the edmaReadWrongData status */
            ESM_REGISTERS->EN = ESM_EN_DISABLE_VALUE;
        }
    }
}

void ecc_main(void *args)
{    
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("EDMA RTI triggered SRAM Scrub Test ...\r\n");

    App_configDma();
    App_configEccEsm();

    DebugP_log("Starting RTI for periodic trigger for EDMA ...\r\n");
    TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);

    ECC_funcTest();

    while(1)
    {
        /*Inject ECC Single bit error*/
        ECC_Test_run_MSS_L2RAMB_1BitInjectTest();
        while(CB_Complete == 0x00)
        {
            ;
        }
        
        if(edmaReadWrongData == true)
        {
            DebugP_log("Data mismatch - EDMA read the wrong data - Scrub successful. ...\r\n");
        }
        else
        {
            DebugP_log("RESET the device. CPU may have read wrong data ...\r\n");
        }
        CB_Complete = 0x00;

        /* Enable back ESM event once print the edmaReadWrongData status */
        ESM_REGISTERS->EN = ESM_EN_ENABLE_VALUE;

    }
}

/* Nothing past this point */
