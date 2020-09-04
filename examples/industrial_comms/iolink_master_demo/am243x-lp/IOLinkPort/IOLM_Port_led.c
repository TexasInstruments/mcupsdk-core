/*!
* \file IOLM_Port_led.c
*
* \brief
* Interface for LED IO-Expander Communication on IOLink Board
*
* \author
* KUNBUS GmbH
*
* \date
* 2021-05-19
*
* \copyright
* Copyright (c) 2021, KUNBUS GmbH<br /><br />
* All rights reserved.<br />
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:<br />
* <ol>
* <li>Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.</li>
* <li>Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.</li>
* <li>Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.</li>
* </ol>
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include <osal.h>
#include <board/led.h>
#include "IOLM_Port_led.h"
#include "ti_board_open_close.h"


extern LED_Handle gLedHandle[]; /* LED handle is created by sysCfg */

/**
* \brief  Initialize the IO-Expander instance
* Init is done on bootup via SysCfg and doesn't need to be executed again
*
*/
void IOLM_LED_IOEXP_init(void)
{
    
}

/**
* \brief  Close the IO-Expander instance
*
*/
void IOLM_LED_IOEXP_close(void)
{

}

/**
* \brief  Transfer LED Bitmask to IOLink IO-Expander Interface
* address_p is not used here. Other targets use the same interface for MCSPI,
* which needs an address of the recipient.
*
* \param  tx_p      bitmask data to send
*
* \return error if not equal 0
*
*/
int32_t IOLM_LED_IOEXP_transfer(uint16_t bitmask_p)
{
    int32_t     error = SystemP_SUCCESS;
    uint32_t    shiftedBitmask;
    uint32_t    shift;
    uint32_t    currentLedNumber;
    uint32_t    currentLedSate;

    for (currentLedNumber = 0; currentLedNumber < IOLM_LED_IOEXP_LED_NUM_IOL;
         currentLedNumber++)   /* walk though bitmask */
    {
        shift          = (IOLM_LED_IOEXP_LED_NUM_IOL - currentLedNumber);
        shiftedBitmask = bitmask_p >> shift;
        currentLedSate = shiftedBitmask & 0x01;

        if(currentLedSate == 0x1)
        {
            error += LED_off(gLedHandle[currentLedNumber], 0); /* off -> on on TCA6424 */
        }
        else
        {
            error += LED_on(gLedHandle[currentLedNumber], 0); /* on -> off on TCA6424 */
        }
    }

    return error;
}

/**
 * \brief  Set state of LED on base board
 *
 * \param[in]   ledNumber_p     number of LED
 * \param[in]   ledState_p      target state
 *
 * \return     error                           as uint32_t
 * \retval     #OSAL_eERR_NOERROR              Success.
 * \retval     #else                           Something went wrong.
 *
 */
int32_t IOLM_LED_IOEXP_baseBoardLED(uint32_t ledNumber_p, bool ledState_p)
{
    int32_t error     = SystemP_SUCCESS;
    uint32_t instance = IOLM_LED_IOEXP_LED_NUM_IOL + ledNumber_p;

    if (ledState_p == true)
    {
        error = LED_on(gLedHandle[instance], ledNumber_p);
    }
    else
    {
        error = LED_off(gLedHandle[instance], ledNumber_p);
    }

    return error;
}

