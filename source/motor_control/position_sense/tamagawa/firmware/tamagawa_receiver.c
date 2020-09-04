/*
 * Copyright (C) 2021 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <string.h>
#include <pru_uart.h>
#include <pru_cfg.h>
#include "tamagawa_interface.h"

/* The FIFO size on the PRU UART is 16 bytes; however, we are (arbitrarily)
 * only going to send 8 at a time */
#define FIFO_SIZE   16
#define MAX_CHARS   12

volatile register uint32_t __R30;

volatile __near struct TamagawaInterface TamagawaInterface __attribute__((
            cregister("PRU_DMEM_0_1", near), peripheral));
		
uint8_t crc(uint8_t *s, uint8_t len)
{
    uint8_t crc = 0;
    uint8_t val;
    int  i, j;
    uint8_t data[12];

    strncpy((char *)data, (const char *)s, len);

    for(i = 0; i < len; i++)
        for(j = 0; j < 8; j++)
        {
            val = (data[i] >> 7) ^ (crc >> 7);

            crc <<= 1;
            data[i] <<= 1;
            crc |= val;
        }

    return crc;
}

void tx_rx(uint8_t tx_size, uint8_t rx_size, uint8_t *tx_buffer,
           uint8_t *rx_buffer)
{
    /* GPO to be toggled correspond to R30.t0 (pr0_pru1_gpo0) */
    volatile uint32_t gpo = 0x0001;

    /* set TxEN */
    __R30 ^= gpo;

    while(tx_size--)
    {
        CT_UART.THR = *tx_buffer++;

        /* wait for transmitted data to be received as Rx is always enabled */
        while((CT_UART.LSR & 0x1) == 0x0)
            ;

        *rx_buffer++ = CT_UART.RBR;
    }

    /* clear TxEN */
    __R30 ^= gpo;

    /* Receive CF, SF, DF0, DF1, DF2 & CRC */
    while(rx_size--)
    {
        while((CT_UART.LSR & 0x1) == 0x0)
            ;

        *rx_buffer++ = CT_UART.RBR;
    }
}

int rx_parse(uint8_t *p)
{
    int ret = 0;

    switch(TamagawaInterface.data_id)
    {
        case DATA_ID_0:
        case DATA_ID_7:
        case DATA_ID_8:
        case DATA_ID_C:
            TamagawaInterface.rx.cf = p[0];
            TamagawaInterface.rx.sf = p[1];
            TamagawaInterface.rx.abs = p[2] | (p[3] << 8) | (p[4] << 16);
            TamagawaInterface.rx.crc = p[5];
            break;

        case DATA_ID_1:
            TamagawaInterface.rx.cf = p[0];
            TamagawaInterface.rx.sf = p[1];
            TamagawaInterface.rx.abm = p[2] | (p[3] << 8) | (p[4] << 16);
            TamagawaInterface.rx.crc = p[5];
            break;

        case DATA_ID_2:
            TamagawaInterface.rx.cf = p[0];
            TamagawaInterface.rx.sf = p[1];
            TamagawaInterface.rx.enid = p[2];
            TamagawaInterface.rx.crc = p[3];
            break;

        case DATA_ID_3:
            TamagawaInterface.rx.cf = p[0];
            TamagawaInterface.rx.sf = p[1];
            TamagawaInterface.rx.abs = p[2] | (p[3] << 8) | (p[4] << 16);
            TamagawaInterface.rx.enid = p[5];
            TamagawaInterface.rx.abm = p[6] | (p[7] << 8) | (p[8] << 16);
            TamagawaInterface.rx.almc = p[9];
            TamagawaInterface.rx.crc = p[10];
            break;

        case DATA_ID_6:
        case DATA_ID_D:
            TamagawaInterface.rx.cf = p[0];
            TamagawaInterface.rx.adf = p[1];
            TamagawaInterface.rx.edf = p[2];
            TamagawaInterface.rx.crc = p[3];
            break;

        default:
            ret = -1;
            break;
    }

    return ret;
}

int tx_prepare(uint8_t *tx, uint8_t *tx_size, uint8_t *rx_size)
{
    int ret = 0;

    switch(TamagawaInterface.data_id)
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
            *tx++ = TamagawaInterface.tx.adf;
            *tx++ = TamagawaInterface.tx.edf;
            *tx_size = 4;
            *rx_size = 4;
            *tx = crc(tx - 3, *tx_size - 1);
            break;

        case DATA_ID_D:
            *tx++ = 0xEA;
            *tx++ = TamagawaInterface.tx.adf;
            *tx_size = 3;
            *rx_size = 4;
            *tx = crc(tx - 2, *tx_size - 1);
            break;

        default:
            ret = -1;
            break;
    }

    return ret;
}

void crc_verify(uint8_t *p, uint8_t len)
{
    if(TamagawaInterface.rx.crc == crc(p, len))
    {
        TamagawaInterface.config |= CONFIG_CMD_STATUS;
    }

    else
    {
        TamagawaInterface.config &= CONFIG_CMD_STATUS;
    }
}

void main()
{

    uint8_t tx[4], rx[12];

    TamagawaInterface.version = 0x00000001;
    TamagawaInterface.data_id = DATA_ID_0;

    /* bring GPO low initially */
    __R30 = 0x0;

    /* TODO: If modifying this to send data through the pins then PinMuxing
     * needs to be taken care of prior to running this code.
     * This is usually done via a GEL file in CCS or by the Linux driver */


    /*** INITIALIZATION ***/

    /* 192MHz / 13 / 6 = ~2.46Mbps */
    CT_UART.DLL = 6;
    CT_UART.DLH = 0;
    CT_UART.MDR = 0x1;

    /* Enable Interrupts in UART module. This allows the main thread to poll for
     * Receive Data Available and Transmit Holding Register Empty */
    CT_UART.IER = 0x7;

    /* If FIFOs are to be used, select desired trigger level and enable
     * FIFOs by writing to FCR. FIFOEN bit in FCR must be set first before
     * other bits are configured */
    /* Enable FIFOs for now at 1-byte, and flush them */
    CT_UART.FCR = (0x8) | (0x4) | (0x2) | (0x1);
    
    /* Choose desired protocol settings by writing to LCR */
    /* 8-bit word, 1 stop bit, no parity, no break control and no divisor latch */
    CT_UART.LCR = 3;

    /* clear MCR */
    CT_UART.MCR = 0x0;

    /* Choose desired response to emulation suspend events by configuring
     * FREE bit and enable UART by setting UTRST and URRST in PWREMU_MGMT */
    /* Allow UART to run free, enable UART TX/RX */
    CT_UART.PWREMU_MGMT = 0x6001;
	#if 0 /*Test loopback. Keeping this code here to debug UART related issue.*/
	uint8_t tx1;
	uint8_t cnt;
	
	/* Enable loopback for test */
	CT_UART.MCR = 0x10;
	
	/* Priming the 'hostbuffer' with a message */
	hostBuffer.data[0] = 'H';
	hostBuffer.data[1] = 'e';
	hostBuffer.data[2] = 'l';
	hostBuffer.data[3] = 'l';
	hostBuffer.data[4] = 'o';
	hostBuffer.data[5] = '!';
	hostBuffer.data[6] = '\0';	
	
	/* Let's send/receive some dummy data */
	for (cnt = 0; cnt < 18; cnt++){
		/* Load character, ensure it is not string termination */
		if ((tx1 = hostBuffer.data[cnt]) == '\0')
			break;
		CT_UART.THR = tx1;

		/* Because we are doing loopback, wait until LSR.DR == 1
		 * indicating there is data in the RX FIFO */
		while ((CT_UART.LSR & 0x1) == 0x0);

		/* Read the value from RBR */
		buffer[cnt] = CT_UART.RBR;

		/* Wait for TX FIFO to be empty */
		while (!((CT_UART.FCR & 0x2) == 0x2));
	}	
	#endif
    /*** END INITIALIZATION ***/

    /* poison the buffer */
    memset(rx, 0x6B, MAX_CHARS);

    /* send & receive the data */
    while(1)
    {
        uint8_t tx_size = 0, rx_size = 0;

        if(!(TamagawaInterface.config & CONFIG_CMD_PROCESS))
        {
            continue;
        }

        tx_prepare(tx, &tx_size, &rx_size);
        tx_rx(tx_size, rx_size, tx, rx);
        rx_parse(rx + tx_size);
        crc_verify(rx + tx_size, rx_size - 1);

        TamagawaInterface.config &= ~CONFIG_CMD_PROCESS;
    }

    /* Disable UART before halting */
    CT_UART.PWREMU_MGMT = 0x0;

    /* Halt PRU core */
    __halt();
}
