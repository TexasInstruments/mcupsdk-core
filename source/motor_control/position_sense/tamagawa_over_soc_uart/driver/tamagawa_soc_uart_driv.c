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
 *  "AS IS" AND ANY EXPgResS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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


#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/uart.h>
#include <drivers/gpio.h>
#include <motor_control/position_sense/tamagawa_over_soc_uart/include/tamagawa_soc_uart_interface.h>

#define MAX_TX_FRAME (4)
#define MAX_RX_FRAME (12)

#define APP_UART_ASSERT_ON_FAILURE(transferOK, transaction) \
    do { \
        if((SystemP_SUCCESS != (transferOK)) || (UART_TRANSFER_STATUS_SUCCESS != transaction.status)) \
        { \
            DebugP_assert(FALSE); /* UART TX/RX failed!! */ \
        } \
    } while(0) \



uint8_t tamagawa_calculate_crc(uint8_t *s, uint8_t len)
{
    uint8_t crc = 0;
    uint8_t val;
    uint32_t  i, j;
    uint8_t data[MAX_RX_FRAME];

    memcpy (data, s, len);

    for(i = 0; i < len; i++)
    {
        for(j = 0; j < 8; j++)
        {
            val = (data[i] >> 7) ^ (crc >> 7);

            crc <<= 1;
            data[i] <<= 1;
            crc |= val;
        }
    }
    return crc;
}

int32_t tamagawa_handle_rx(volatile struct tamagawa_uart_interface *tamagawa_interface, uint8_t *p)
{
    int ret = 0;
    switch(tamagawa_interface->data_id)
    {
        case DATA_ID_0:
        case DATA_ID_7:
        case DATA_ID_8:
        case DATA_ID_C:
            tamagawa_interface->rx.cf = p[0];
            tamagawa_interface->rx.sf = p[1];
            tamagawa_interface->rx.abs = p[2] | (p[3] << 8) | (p[4] << 16);
            tamagawa_interface->rx.crc = p[5];
            break;

        case DATA_ID_1:
            tamagawa_interface->rx.cf = p[0];
            tamagawa_interface->rx.sf = p[1];
            tamagawa_interface->rx.abm = p[2] | (p[3] << 8) | (p[4] << 16);
            tamagawa_interface->rx.crc = p[5];
            break;

        case DATA_ID_2:
            tamagawa_interface->rx.cf = p[0];
            tamagawa_interface->rx.sf = p[1];
            tamagawa_interface->rx.enid = p[2];
            tamagawa_interface->rx.crc = p[3];
            break;

        case DATA_ID_3:
            tamagawa_interface->rx.cf = p[0];
            tamagawa_interface->rx.sf = p[1];
            tamagawa_interface->rx.abs = p[2] | (p[3] << 8) | (p[4] << 16);
            tamagawa_interface->rx.enid = p[5];
            tamagawa_interface->rx.abm = p[6] | (p[7] << 8) | (p[8] << 16);
            tamagawa_interface->rx.almc = p[9];
            tamagawa_interface->rx.crc = p[10];
            break;

        case DATA_ID_6:
        case DATA_ID_D:
            tamagawa_interface->rx.cf = p[0];
            tamagawa_interface->rx.adf = p[1];
            tamagawa_interface->rx.edf = p[2];
            tamagawa_interface->rx.crc = p[3];
            break;

        default:
            ret = -1;
            break;
    }

    return ret;
}


int32_t tamagawa_command_build(volatile struct tamagawa_uart_interface *tamagawa_interface, uint8_t *tx, uint8_t *tx_size, uint8_t *rx_size)
{
    int ret = 0;

    switch(tamagawa_interface->data_id)
    {
        case DATA_ID_0:
            *tx = 0x02;
            *tx_size = 1;
            *rx_size = 6;
            break;

        case DATA_ID_1:
            *tx = 0x8A;
            *tx_size = 1;
            *rx_size = 6;
            break;

        case DATA_ID_2:
            *tx = 0x92;
            *tx_size = 1;
            *rx_size = 4;
            break;

        case DATA_ID_3:
            *tx = 0x1A;
            *tx_size = 1;
            *rx_size = 11;
            break;

        case DATA_ID_7:
            *tx = 0xBA;
            *tx_size = 1;
            *rx_size = 6;
            break;

        case DATA_ID_8:
            *tx = 0xC2;
            *tx_size = 1;
            *rx_size = 6;
            break;

        case DATA_ID_C:
            *tx = 0x62;
            *tx_size = 1;
            *rx_size = 6;
            break;

        case DATA_ID_6:
            *tx++ = 0x32;
            *tx++ = tamagawa_interface->tx.adf;
            *tx++ = tamagawa_interface->tx.edf;
            *tx_size = 4;
            *rx_size = 4;
            *tx = tamagawa_calculate_crc(tx - 3, *tx_size - 1);
            break;

        case DATA_ID_D:
            *tx++ = 0xEA;
            *tx++ = tamagawa_interface->tx.adf;
            *tx_size = 3;
            *rx_size = 4;
            *tx = tamagawa_calculate_crc(tx - 2, *tx_size - 1);
            break;

        default:
            ret = -1;
            break;
    }

    return ret;
}


int32_t tamagawa_command_process(volatile struct tamagawa_uart_interface *tamagawa_interface, UART_Handle *gUartHandle, int32_t cmd)
{
    int32_t          transferOK;
    UART_Transaction trans;

    UART_Transaction_init(&trans);

    uint8_t tx[MAX_TX_FRAME];
    uint8_t rx[MAX_RX_FRAME];
    uint8_t tx_size = 0, rx_size = 0;
    tamagawa_interface->data_id = cmd;

    /* Command build */
    cmd = tamagawa_command_build(tamagawa_interface, tx, &tx_size, &rx_size);
    if(cmd < 0)
    {
        return cmd;
    }

    /* Write data into Tx fifo*/
    trans.buf   = &tx[0U];
    trans.count = tx_size;
    /*configure GPIO pin high just before TX start (RTSn high)*/
    GPIO_pinWriteHigh(tamagawa_interface->gpio_base_address, tamagawa_interface->gpio_pin_number);
    transferOK = UART_write(gUartHandle[tamagawa_interface->uart_instance], &trans);
    /*configure GPIO pin low just after TX complete (RTSn low)*/
    GPIO_pinWriteLow(tamagawa_interface->gpio_base_address, tamagawa_interface->gpio_pin_number);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    /* Read data from Rx fifo */
    trans.buf   = &rx[0U];
    trans.count = rx_size + tx_size;
    transferOK = UART_read(gUartHandle[tamagawa_interface->uart_instance], &trans);
    APP_UART_ASSERT_ON_FAILURE(transferOK, trans);

    /* parsing of recevied data*/
     cmd = tamagawa_handle_rx(tamagawa_interface, rx + tx_size);


    /* CRC calculation*/
    tamagawa_interface->rx_crc = tamagawa_calculate_crc(rx + tx_size, rx_size - 1);

    return cmd;
}

int32_t tamagawa_crc_verify(volatile struct tamagawa_uart_interface *tamagawa_interface)
{
    return (tamagawa_interface->rx.crc == tamagawa_interface->rx_crc) ? 1 : 0;
}

void tamagawa_init(volatile struct tamagawa_uart_interface *tamagawa_interface, uint32_t instance , uint32_t base_address, uint32_t pin_number, uint32_t pin_direction)
{
    tamagawa_interface->uart_instance = instance;
    tamagawa_interface->gpio_base_address = base_address;
    tamagawa_interface->gpio_pin_number = pin_number;
    /*GPIO pin configurtaion for RTSn*/
    GPIO_setDirMode(base_address, pin_number, pin_direction);
}