/**
 * tamagawa_app.c
 *
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * */



#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include <kernel/dpl/DebugP.h>
#include <drivers/soc.h>

#include <drivers/sciclient.h>
#include <kernel/dpl/TaskP.h>
#include <drivers/pinmux.h>
#include <drivers/hw_include/hw_types.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include <motor_control/position_sense/tamagawa/firmware/tamagawa_interface.h>
#include <motor_control/position_sense/tamagawa/firmware/tamagawa_receiver_bin.h>

#include <board/ioexp/ioexp_tca6424.h>

static TCA6424_Config  gTCA6424_Config;

#define PRUICSS_PRUx  PRUICSS_PRU1

static void tamagawa_i2c_io_expander(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    /* P20 = LED 3 bits, pin, 2 bits port.*/
    uint32_t            ioIndex = 0x10;
    TCA6424_Params      tca6424Params;

    TCA6424_Params_init(&tca6424Params);

    status = TCA6424_open(&gTCA6424_Config, &tca6424Params);

    if(status == SystemP_SUCCESS)
    {
        /* Set output to HIGH before config so that LED start with On state */
        status = TCA6424_setOutput(
                     &gTCA6424_Config,
                     ioIndex,
                     TCA6424_OUT_STATE_HIGH);

        /* Configure as output  */
        status += TCA6424_config(
                      &gTCA6424_Config,
                      ioIndex,
                      TCA6424_MODE_OUTPUT);
        /* set P12 high which controls CPSW_FET_SEL -> enable PRU1 and PRU0 GPIOs */
        ioIndex = 0x0a;
        status = TCA6424_setOutput(
                     &gTCA6424_Config,
                     ioIndex,
                     TCA6424_OUT_STATE_HIGH);

        /* Configure as output  */
        status += TCA6424_config(
                      &gTCA6424_Config,
                      ioIndex,
                      TCA6424_MODE_OUTPUT);


    }
    TCA6424_close(&gTCA6424_Config);
}

/** \brief Global Structure pointer holding PRUSS1 memory Map. */
PRUICSS_Handle gPruIcss0Handle;

void *gPru_dramx;

void tamagawa_pruss_init(void)
{
    gPruIcss0Handle = PRUICSS_open(CONFIG_PRU_ICSS0);

    /* Set in constant table C30 to shared RAM 0x40300000 */
    PRUICSS_setConstantTblEntry(gPruIcss0Handle, PRUICSS_PRUx, PRUICSS_CONST_TBL_ENTRY_C30, ((0x40300000 & 0x00FFFF00) >> 8));

    /* clear ICSS0 PRU1 data RAM */
    PRUICSS_initMemory(gPruIcss0Handle, PRUICSS_DATARAM(PRUICSS_PRUx));
    gPru_dramx = (void *)((((PRUICSS_HwAttrs *)(gPruIcss0Handle->hwAttrs))->baseAddr) + PRUICSS_DATARAM(PRUICSS_PRUx));

    PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_PRUx);
}

void tamagawa_pruss_load_run_fw(void)
{
    PRUICSS_disableCore(gPruIcss0Handle, PRUICSS_PRUx);

    /*Load firmware. Set buffer = write to Pru memory */

    PRUICSS_writeMemory(gPruIcss0Handle, PRUICSS_IRAM_PRU(PRUICSS_PRUx),
                        0, (uint32_t *) TamagawaFirmware,
                        sizeof(TamagawaFirmware));


    PRUICSS_resetCore(gPruIcss0Handle, PRUICSS_PRUx);
    /*Run firmware */
    PRUICSS_enableCore(gPruIcss0Handle, PRUICSS_PRUx);
}

static void tamagawa_display_result(volatile struct TamagawaInterface *p)
{
    switch(p->data_id)
    {
        case DATA_ID_0:
        case DATA_ID_7:
        case DATA_ID_8:
        case DATA_ID_C:
            DebugP_log("\r| ");
            DebugP_log("ABS: 0x%x\t", p->rx.abs);
            DebugP_log("SF: 0x%x\t", p->rx.sf);
            DebugP_logInfo("CF: 0x%x\t", p->rx.cf);
            DebugP_logInfo("CRC: 0x%x\t", p->rx.crc);
            DebugP_log("\n");
            break;

        case DATA_ID_1:
            DebugP_log("\r| ");
            DebugP_log("ABM: 0x%x\t", p->rx.abm);
            DebugP_log("SF: 0x%x\t", p->rx.sf);
            DebugP_logInfo("CF: 0x%x\t", p->rx.cf);
            DebugP_logInfo("CRC: 0x%x\t", p->rx.crc);
            DebugP_log("\n");
            break;

        case DATA_ID_2:
            DebugP_log("\r| ");
            DebugP_log("ENID: 0x%x\t", p->rx.enid);
            DebugP_log("SF: 0x%x\t", p->rx.sf);
            DebugP_logInfo("CF: 0x%x\t", p->rx.cf);
            DebugP_logInfo("CRC: 0x%x\t", p->rx.crc);
            DebugP_log("\n");
            break;

        case DATA_ID_3:
            DebugP_log("\r| ");
            DebugP_log("ABS: 0x%x\t", p->rx.abs);
            DebugP_log("ENID: 0x%x\t", p->rx.enid);
            DebugP_log("ABM: 0x%x\t", p->rx.abm);
            DebugP_log("ALMC: 0x%x\t", p->rx.almc);
            DebugP_log("SF: 0x%x\t", p->rx.sf);
            DebugP_logInfo("CF: 0x%x\t", p->rx.cf);
            DebugP_logInfo("CRC: 0x%x\t", p->rx.crc);
            DebugP_log("\n");
            break;

        case DATA_ID_6:
            DebugP_logInfo("\r| ");
            DebugP_logInfo("ADF: 0x%x\t", p->rx.adf);
            DebugP_logInfo("CF: 0x%x\t", p->rx.cf);
            DebugP_logInfo("CRC: 0x%x\t", p->rx.crc);
            DebugP_logInfo("\n");
            break;

        case DATA_ID_D:
            DebugP_log("\r| ");
            DebugP_log("EDF: 0x%x\t", p->rx.edf);
            DebugP_logInfo("ADF: 0x%x\t", p->rx.adf);
            DebugP_logInfo("CF: 0x%x\t", p->rx.cf);
            DebugP_logInfo("CRC: 0x%x\t", p->rx.crc);
            DebugP_log("\n");
            break;

        default:
            DebugP_log("\r| ERROR: unknown Data ID\n");
            break;
    }
}

static enum data_id tamagawa_get_command(uint8_t *adf, uint8_t *edf)
{
    int cmd;
#ifdef ENABLE_TAMAGAWA_EEPROM_OPS
    int val;
#endif

    if(DebugP_scanf("%d\n", &cmd) < 0)
    {
        cmd = DATA_ID_0;
        DebugP_log("\r| WARNING: invalid Data ID, Data readout Data ID 0 will be sent\n");
    }

    if(cmd >= DATA_ID_NUM)
    {
        cmd = DATA_ID_0;
        DebugP_log("\r| WARNING: invalid Data ID, Data readout Data ID 0 will be sent\n");
    }

#ifdef ENABLE_TAMAGAWA_EEPROM_OPS
    if((cmd == DATA_ID_D) || (cmd == DATA_ID_6))
    {
        DebugP_log("\r| enter EEPROM address (hex value): ");

        if(DebugP_scanf("%x\n", &val) < 0)
        {
            cmd = DATA_ID_NUM;
            DebugP_log("\r| ERROR: invalid EEPROM address\n|\n");
        }

        if(val > 126)
        {
            cmd = DATA_ID_NUM;
            DebugP_log("\r| ERROR: invalid EEPROM address\n|\n");
        }

        *adf = (uint8_t)val;
    }

    if(cmd == DATA_ID_6)
    {
        DebugP_log("\r| enter EEPROM data (hex value): ");

        if(DebugP_scanf("%x\n", &val) < 0)
        {
            cmd = DATA_ID_NUM;
            DebugP_log("\r| ERROR: invalid EEPROM address\n|\n");
        }

        if(val > 255)
        {
            cmd = DATA_ID_NUM;
            DebugP_log("\r| ERROR: invalid EEPROM address\n|\n");
        }

        *edf = (uint8_t)val;
    }
#else
    if((cmd == DATA_ID_D) || (cmd == DATA_ID_6))
    {
        cmd = DATA_ID_0;
        DebugP_log("\r| WARNING: unsupported Data ID, Data readout Data ID 0 will be sent\n");
    }
#endif

    return cmd;
}

static void tamagawa_display_menu(void)
{
    DebugP_log("\r|------------------------------------------------------------------------------|\n");
    DebugP_log("\r|                             Select DATA ID Code                              |\n");
    DebugP_log("\r|------------------------------------------------------------------------------|\n");
    DebugP_log("\r| 0 : Data readout, Absolute (Data ID 0)                                       |\n");
    DebugP_log("\r| 1 : Data readout, Multi-turn (Data ID 1)                                     |\n");
    DebugP_log("\r| 2 : Data readout, Encoder-ID (Data ID 2)                                     |\n");
    DebugP_log("\r| 3 : Data readout, Absolute & Multi-turn (Data ID 3)                          |\n");

#ifdef ENABLE_TAMAGAWA_EEPROM_OPS
    DebugP_log("\r| 4 : Writing to EEPROM (Data ID 6)                                            |\n");
#endif

    DebugP_log("\r| 5 : Reset (Data ID 7)                                                        |\n");
    DebugP_log("\r| 6 : Reset (Data ID 8)                                                        |\n");
    DebugP_log("\r| 7 : Reset (Data ID C)                                                        |\n");

#ifdef ENABLE_TAMAGAWA_EEPROM_OPS
    DebugP_log("\r| 8 : Readout from EEPROM (Data ID D)                                          |\n");
#endif

    DebugP_log("\r|------------------------------------------------------------------------------|\n|\n");
    DebugP_log("\r| enter value: ");
}


void tamagawa_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /* Configure g_mux_en to 0 in ICSSG_SA_MX_REG Register. */
     HW_WR_REG32((CSL_PRU_ICSSG0_PR1_CFG_SLV_BASE+0x40), (0x00));
     /* Configure IO Expander to get PRU GPIO's on HSEC */
     tamagawa_i2c_io_expander(NULL);

    tamagawa_pruss_init();

    DebugP_log("\n\nTamagawa pruss init done");

    tamagawa_pruss_load_run_fw();

    DebugP_log("\r\nTamagawa pruss firmware loaded and running");

    DebugP_log("\r\n\n");

    while(1)
    {
        volatile struct TamagawaInterface *p = gPru_dramx;
        /*
         * initialized to zero as compiler unable to detect that the
         * variable will not be used uninitialized, giving a warning
         */
        uint8_t adf = 0, edf = 0;
        enum data_id cmd;

        tamagawa_display_menu();
        cmd = tamagawa_get_command(&adf, &edf);

        if(cmd >= DATA_ID_NUM)
        {
            continue;
        }

        p->data_id = cmd;

        if((cmd == DATA_ID_6) || (cmd == DATA_ID_D))
        {
            p->tx.adf = adf;
        }

        if(cmd == DATA_ID_6)
        {
            p->tx.edf = edf;
        }

        p->config |= CONFIG_CMD_PROCESS;

        while(p->config & CONFIG_CMD_PROCESS)
            ;

        DebugP_log("\r|\n");

        if(p->config & CONFIG_CMD_STATUS)
        {
            tamagawa_display_result(p);
        }

        else
        {
            DebugP_log("ERROR: CRC failure\n");
        }

        DebugP_log("\r|\n\r|\n");
    }

    Board_driversClose();
    Drivers_close();
}
