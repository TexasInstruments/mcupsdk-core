/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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

/* This example demonstrates the UART RX and TX operation by echoing char
 * that it recieves in blocking, interrupt mode of operation.
 * When user types 'quit', the application ends.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <motor_control/position_sense/tamagawa_over_soc_uart/include/tamagawa_soc_uart_interface.h>

extern UART_Handle gUartHandle[CONFIG_UART_NUM_INSTANCES];
volatile struct tamagawa_uart_interface TamagawaInterfaceInstance;

volatile struct tamagawa_uart_interface *tamagawa_interface = &TamagawaInterfaceInstance;

void tamagawa_display_result(int32_t cmd)
{
    /* Prints the position value returned by the encoder for a particular command ID */
    switch(cmd)
    {
        case DATA_ID_7:
            /* Reset */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABS: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", tamagawa_interface->rx.abs, tamagawa_interface->rx.sf, tamagawa_interface->rx.cf, tamagawa_interface->rx.crc);
            break;

        case DATA_ID_8:
            /* Reset */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABS: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", tamagawa_interface->rx.abs, tamagawa_interface->rx.sf, tamagawa_interface->rx.cf, tamagawa_interface->rx.crc);
            break;

        case DATA_ID_C:
            /* Reset */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABS: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", tamagawa_interface->rx.abs, tamagawa_interface->rx.sf, tamagawa_interface->rx.cf, tamagawa_interface->rx.crc);
            break;

        case DATA_ID_0:
            /* Data readout: data in one revolution */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABS: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", tamagawa_interface->rx.abs, tamagawa_interface->rx.sf, tamagawa_interface->rx.cf, tamagawa_interface->rx.crc);
            break;

        case DATA_ID_1:
            /* Data readout: multi-turn data */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABM: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", tamagawa_interface->rx.abm, tamagawa_interface->rx.sf, tamagawa_interface->rx.cf, tamagawa_interface->rx.crc);
            break;

        case DATA_ID_2:
            /*  Data readout: encoder ID */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nENID: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", tamagawa_interface->rx.enid, tamagawa_interface->rx.sf, tamagawa_interface->rx.cf, tamagawa_interface->rx.crc);
            break;

        case DATA_ID_3:
            /* Data readout: data in one revolution, encoder ID, multi-turn, encoder error */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nABS: 0x%x\tENID: 0x%x\tABM: 0x%x\tALMC: 0x%x\tSF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", tamagawa_interface->rx.abs, tamagawa_interface->rx.enid, tamagawa_interface->rx.abm, tamagawa_interface->rx.almc, tamagawa_interface->rx.sf, tamagawa_interface->rx.cf, tamagawa_interface->rx.crc);
            break;

        case DATA_ID_6:
            /* EEPROM Write */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nEDF: 0x%x\tADF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", tamagawa_interface->rx.edf, tamagawa_interface->rx.adf, tamagawa_interface->rx.cf, tamagawa_interface->rx.crc);
            break;

        case DATA_ID_D:
            /* EEPROM Read */
            DebugP_log("\r\n| ");
            DebugP_log("\r\nEDF: 0x%x\tADF: 0x%x\tCF: 0x%x\tCRC: 0x%x\t\n", tamagawa_interface->rx.edf, tamagawa_interface->rx.adf, tamagawa_interface->rx.cf, tamagawa_interface->rx.crc);
            break;

        default:
            DebugP_log("\r\n| ERROR: unknown Data ID\n");
            break;
    }
}

static enum data_id tamagawa_get_command()
{
    int32_t cmd;
    uint32_t val;
    /* Check to make sure that the command issued is correct */
    if(DebugP_scanf("%d\n", &cmd) < 0)
    {
        cmd = DATA_ID_0;
        DebugP_log("\r\n| WARNING: invalid Data ID, Data readout Data ID 0 will be sent\n");
    }
    /* Check to make sure that the command issued is correct */
    if(cmd >= DATA_ID_NUM)
    {
        cmd = DATA_ID_0;
        DebugP_log("\r\n| WARNING: invalid Data ID, Data readout Data ID 0 will be sent\n");
    }
    /* In case of EEPROM commands */
    if((cmd == DATA_ID_D) || (cmd == DATA_ID_6))
    {
        DebugP_log("\r| enter EEPROM address (hex value): ");
        if(DebugP_scanf("%x\n", &val) < 0)
        {
            cmd = DATA_ID_NUM;
            DebugP_log("\r| ERROR: invalid EEPROM address\n|\n");
        }
        if(val > MAX_EEPROM_ADDRESS)
        {
            cmd = DATA_ID_NUM;
            DebugP_log("\r| ERROR: invalid EEPROM address\n|\n");
        }
        tamagawa_interface->tx.adf = (uint8_t)val;

    }
    /* In case of EEPROM Write */
    if(cmd == DATA_ID_6)
    {
        DebugP_log("\r| enter EEPROM data (hex value): ");
        if(DebugP_scanf("%x\n", &val) < 0)
        {
            cmd = DATA_ID_NUM;
            DebugP_log("\r| ERROR: invalid EEPROM address\n|\n");
        }
        if(val > MAX_EEPROM_WRITE_DATA)
        {
            cmd = DATA_ID_NUM;
            DebugP_log("\r| ERROR: invalid EEPROM address\n|\n");
        }
        tamagawa_interface->tx.edf = (uint8_t)val;

    }
    return cmd;
}
static void tamagawa_display_menu(void)
{
    DebugP_log("\r\n|------------------------------------------------------------------------------|");
    DebugP_log("\r\n|                             Select DATA ID Code                              |");
    DebugP_log("\r\n|------------------------------------------------------------------------------|");
    DebugP_log("\r\n| 0 : Data readout, Absolute (Data ID 0)                                       |");
    DebugP_log("\r\n| 1 : Data readout, Multi-turn (Data ID 1)                                     |");
    DebugP_log("\r\n| 2 : Data readout, Encoder-ID (Data ID 2)                                     |");
    DebugP_log("\r\n| 3 : Data readout, Absolute & Multi-turn (Data ID 3)                          |");
    DebugP_log("\r\n| 4 : Writing to EEPROM (Data ID 6)                                            |");
    DebugP_log("\r\n| 5 : Reset (Data ID 7)                                                        |");
    DebugP_log("\r\n| 6 : Reset (Data ID 8)                                                        |");
    DebugP_log("\r\n| 7 : Reset (Data ID C)                                                        |");
    DebugP_log("\r\n| 8 : Readout from EEPROM (Data ID D)                                          |");
    DebugP_log("\r\n|------------------------------------------------------------------------------|\n|\n");
    DebugP_log("\r\n| enter value: ");
}

void uart_tamagawa(void *args)
{
    Drivers_open();
    Board_driversOpen();


    uint32_t uart_communication_instance = CONFIG_UART0;
    uint32_t uart_gpio_base_address = CSL_GPIO0_U_BASE;
    uint32_t uart_gpio_pin_number = CONFIG_GPIO0_PIN;
    uint32_t uart_gpio_pin_direction = GPIO_DIRECTION_OUTPUT;

    tamagawa_init(tamagawa_interface, uart_communication_instance, uart_gpio_base_address, uart_gpio_pin_number,  uart_gpio_pin_direction);
    DebugP_log("[UART] Tamagawa example started ...\r\n");
    while(1)
    {
        enum data_id cmd;
        int32_t status;

        tamagawa_display_menu();
        cmd = tamagawa_get_command();

        if(cmd >= DATA_ID_NUM)
        {
            continue;
        }

        status = tamagawa_command_process(tamagawa_interface, gUartHandle, cmd);
        /* Case of command process failure */

        if (status < 0)
        {
            DebugP_log("\r\n ERROR: Command process failure \n");
            continue;
        }

        if (tamagawa_crc_verify(tamagawa_interface) == 1)
        {
            DebugP_log("\r\n CRC success \n");
            tamagawa_display_result(cmd);
        }
        else
        {
            DebugP_log("\r\n CRC Failure \n");
        }

    }

    Board_driversClose();
    Drivers_close();

    return;

}
