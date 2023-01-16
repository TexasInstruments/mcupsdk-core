/*
 *  Copyright (C) 2021-2023 Texas Instruments Incorporated
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

/* Adapted by TI for running on its platform and SDK */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include <usb/cdn/include/usb_init.h>
#include <usb/cdn/include/cdn_print.h>
#include "tusb.h"

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

static void cdc_task(void);


int cdc_echo_main(void)
{
    Drivers_open();
    Board_driversOpen();

	/* Enable debug for cdn module */ 
	DbgMsgEnableModule(USBSSP_DBG_CUSBD);
	DbgMsgEnableModule(USBSSP_DBG_CUSBD_ISR );
	DbgMsgSetLvl(100); 

    while (1)
    {
	    cusbd_dsr(); /* Cadence DSR task */
        tud_task(); /* tinyusb device task */
        cdc_task(); /* CDC handler */
    }

    return 0;
}

/* echo to either Serial0 or Serial1
   with Serial0 as all lower case, Serial1 as all upper case
 */
static void echo_serial_port(uint8_t itf, uint8_t buf[], uint32_t count)
{
    for(uint32_t i=0; i<count; i++)
    {
        if (itf == 0)
        {
            /* echo back 1st port as lower case */
            if (isupper(buf[i])) buf[i] += 'a' - 'A';
        }
        else
        {
            /* echo back additional ports as upper case */
            if (islower(buf[i])) buf[i] -= 'a' - 'A';
        }

        tud_cdc_n_write_char(itf, buf[i]);
    }
    tud_cdc_n_write_flush(itf);
}

static void cdc_task(void)
{
    uint8_t itf;

    for (itf = 0; itf < CFG_TUD_CDC; itf++)
    {
        /* connected() check for DTR bit
           Most but not all terminal client set this when making connection
         */
        /* if ( tud_cdc_n_connected(itf) ) */
        {
            if ( tud_cdc_n_available(itf) )
            {
                uint8_t buf[512];

                uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));

                /* echo back to both serial ports */
                echo_serial_port(0, buf, count);
                echo_serial_port(1, buf, count);
            }
        }
    }
}
