/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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

#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include <drivers/pruicss.h>
#include <pru_ipc.h>

/*
 *  This is an example project to show ADC samples transfered to R5F TCM
 *  by PRU and read by R5F on interrupt.
 */

/** \brief Global Structure pointer holding PRUSS1 memory Map. */
PRUICSS_Handle gPruIcss0Handle;

PRU_IPC_Handle gPruIpc0Handle;

/* semaphore used to indicate that the ISR has finished reading samples */
SemaphoreP_Object gAdcDataRecSem;

int32_t samples[CONFIG_PRU_IPC0_BUFFERS][CONFIG_PRU_IPC0_BLOCKSIZE];

#ifdef _DEBUG_
void print_samples(int32_t array[CONFIG_PRU_IPC0_BUFFERS][CONFIG_PRU_IPC0_BLOCKSIZE])
{
    for (uint32_t channel = 0; channel < CONFIG_PRU_IPC0_BUFFERS; channel++) {
        DebugP_log("----- Channel: %d ----- \r\n", channel+1);
        for (uint32_t sample = 0; sample < CONFIG_PRU_IPC0_BLOCKSIZE; sample++) {
            DebugP_log("Sample %d:  %d  \r\n", sample+1, array[channel][sample]);
        }
    }
    DebugP_log("-----------------------\r\n");
}
#endif

/* Functions defined in adc_functions.c */
void ADC_init();
void ADC_powerUp();
void ADC_reset();
void ADC_configure(uint16_t noOfCommands, uint16_t adcConfigCommands[]);
void ADC_startConversion();
void ADC_stopConversion();

void PRU_IPC_Isr(void *args)
{
    PRU_IPC_getData(gPruIpc0Handle, samples);
    SemaphoreP_post(&gAdcDataRecSem);
    PRUICSS_clearEvent(gPruIcss0Handle, gPruIpc0Handle->attrs->sysEventNum);
}

void ads_example_main(void *args)
{
    Drivers_open(); // check return status
    int status;
    status = Board_driversOpen();
    DebugP_assert(SystemP_SUCCESS == status);

    gPruIcss0Handle = PRUICSS_open(CONFIG_PRU_ICSS0);
    status = PRUICSS_intcInit(gPruIcss0Handle, &icss0_intc_initdata);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Construct semaphore for ADC samples received */
    SemaphoreP_constructBinary(&gAdcDataRecSem, 0);

    /*
        Write Register Command - SDO:  <80h + Address[3:0]><Reg data>
        Register Map:
        ADDRESS | REGISTER | DEFAULT   | BIT 7 BIT 6 BIT 5 BIT 4 BIT 3 BIT 2 BIT 1 BIT 0
          0h    | DEV_ID   | 00h       | DEV_ID[7:0]
          1h    | REV_ID   | xxh       | REV_ID[7:0]
          2h    | STATUS   | x1100xxxb | CS_MODE ALV_FLAG POR_FLAG SPI_ERR REG_ERR ADC_ERR MOD_FLAG DRDY
          3h    | CONTROL  | 00h       | RESET[5:0]    START STOP
          4h    | MUX      | 00h       | RESERVED[5:0] MUX[1:0]
          5h    | CONFIG1  | 00h       | RESERVED REF_RNG INP_RNG VCM REFP_BUF RESERVED AINP_BUF AINN_BUF
          6h    | CONFIG2  | 00h       | EXT_RNG RESERVED SDO_MODE START_MODE[1:0] SPEED_MODE STBY_MODE PWDN
          7h    | CONFIG3  | 00h       | DELAY[2:0] FILTER[4:0]
          8h    | CONFIG4  | 00h       | CLK_SEL CLK_DIV OUT_DRV RESERVED DATA SPI_CRC REG_CRC STATUS
          9h    | OFFSET2  | 00h       | OFFSET[23:16]
          Ah    | OFFSET1  | 00h       | OFFSET[15:8]
          Bh    | OFFSET0  | 00h       | OFFSET[7:0]
          Ch    | GAIN2    | 40h       | GAIN[23:16]
          Dh    | GAIN1    | 00h       | GAIN[15:8]
          Eh    | GAIN0    | 00h       | GAIN[7:0]
          Fh    | CRC      | 00h       | CRC[7:0]

        For 1.067MSPS:
        CONFIG3 Reg: 0b01000 = sinc4, OSR = 12 => low-latency filter
        Command to Send: <87h><08h>
    */
    /*  Total number of Commands to send to ADC */
    uint16_t noOfCommands = 4;
    /*  Commands array
     *  Add your commands to this (maximum = noOfCommands)
     */
    uint16_t adcConfigCommands[4] = {0x8708, 0x8708, 0x8708, 0x8708};

    DebugP_log("--------------------------------------------------\r\n");
    DebugP_log("Initializing App \r\n");
    ADC_init();
    DebugP_log("Initialized \r\n");

    DebugP_log("--------------------------------------------------\r\n");
    DebugP_log("Powering up ADC... \r\n");
    ADC_powerUp();
    DebugP_log("ADC Powered Up \r\n");

    DebugP_log("--------------------------------------------------\r\n");
    DebugP_log("Configuring ADC... \r\n");
    ADC_configure(noOfCommands, adcConfigCommands);
    DebugP_log("ADC Configuration completed \r\n");

   DebugP_log("--------------------------------------------------\r\n");
   DebugP_log("Starting ADC Conversion \r\n");
   ADC_startConversion();
    DebugP_log("Started ADC Conversion \r\n");
   DebugP_log("--------------------------------------------------\r\n");

    while (1)
    {
        ClockP_usleep(1);
        //SemaphoreP_pend(&gAdcDataRecSem, SystemP_WAIT_FOREVER);
        /* print_samples(samples); */
    }

    Board_driversClose();
    Drivers_close();
}
