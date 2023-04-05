/*!
 *  \file CUST_led.c
 *
 *  \brief
 *  Provides initialization of custom LED's.
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2022-03-16
 *
 *  \copyright
 *  Copyright (c) 2022, KUNBUS GmbH<br /><br />
 *  All rights reserved.<br />
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:<br />
 *  <ol>
 *  <li>Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.</li>
 *  <li>Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.</li>
 *  <li>Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.</li>
 *  </ol>
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <stdio.h>
#include <string.h>

#include <board/led.h>
#include <drivers/i2c.h>

#include <drivers/led/CUST_led.h>
#include "ti_board_config.h"
#include "ti_board_open_close.h"


/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Initialization of the industrial LEDs on board.
 *
 *  \return     uint32_t                            Error code.
 *
 *  \retval     CUST_LED_eERR_NOERROR               Success.
 *  \retval     CUST_LED_eERR_HANDLE_INVALID        LED handle is invalid.
 *  \retval     CUST_LED_eERR_ATTRIBUTES_INVALID    LED attributes are invalid.
 *  \retval     CUST_LED_eERR_SET_MASK              LED set mask failed.
 *  \retval     CUST_LED_eERR_OFF                   LED off failed.
 *
 */
uint32_t CUST_LED_init(void)
{
    uint32_t   result = (uint32_t) CUST_LED_eERR_NOERROR;

    LED_Handle handle = NULL;
    LED_Attrs* pAttrs = NULL;
    int32_t    status = SystemP_FAILURE;
    int32_t    ledCnt = 0;

    // Get LED handle from sys config
    handle = gLedHandle[CONFIG_LED0];
    if(NULL == handle)
    {
        OSAL_printf("No led handle available.\n");
        result = (uint32_t) CUST_LED_eERR_HANDLE_INVALID;
    }
    else
    {
        pAttrs = (LED_Attrs*) LED_getAttrs(CONFIG_LED0);
        if(NULL == pAttrs)
        {
            OSAL_printf("Can not get LED attributes.\n");
            result = (uint32_t) CUST_LED_eERR_ATTRIBUTES_INVALID;
        }
        else
        {
            if(pAttrs->numLedPerGroup > 1U)
            {
                status = LED_setMask(handle, 0xFFU);
                if(SystemP_SUCCESS != status)
                {
                    OSAL_printf("Can not set LED Mask.\n");
                    result = (uint32_t) CUST_LED_eERR_SET_MASK;
                }
                else
                {
                    // Switch all LEDs off
                    for(ledCnt = 0U; ledCnt < pAttrs->numLedPerGroup; ledCnt++)
                    {
                        status = LED_off(handle, ledCnt);

                        if (SystemP_SUCCESS != status)
                        {
                            OSAL_printf("LED off failed.\n");
                            result = (uint32_t) CUST_LED_eERR_OFF;

                            break;
                        }
                    }
                }
            }
        }
    }

    return result;
}


/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Deinitialization of the industrial LEDs on board.
 *
 *  \return     uint32_t                        Error code.
 *
 *  \retval     CUST_LED_eERR_NOERROR           Success.
 *  \retval     CUST_LED_eERR_HANDLE_INVALID    LED handle is invalid.
 *  \retval     CUST_LED_eERR_GENERALERROR      Failed.
 *
 */
uint32_t CUST_LED_deInit(void)
{
    uint32_t   result = (uint32_t) CUST_LED_eERR_NOERROR;

    int32_t    status = SystemP_FAILURE;
    LED_Handle handle = NULL;

    // Get LED handle from sys config
    handle = gLedHandle[CONFIG_LED0];
    if(NULL == handle)
    {
        OSAL_printf("No led handle available.\n");
        result = (uint32_t) CUST_LED_eERR_HANDLE_INVALID;
    }

    status = LED_setMask(handle, 0x0U);

    if (SystemP_FAILURE == status)
    {
        OSAL_printf("Can not set LED Mask.\n");
        result = (uint32_t) CUST_LED_eERR_SET_MASK;
    }

    return result;
}
