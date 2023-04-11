/*!
 *  \file ESL_BOARD_OS_config.c
 *
 *  \brief
 *  Board OS configuration for FreeRTOS AM64 GP EVM.
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2021-05-21
 *
 *  \copyright
 *  Copyright (c) 2021, KUNBUS GmbH<br /><br />
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

#include <ESL_gpioHelper.h>
#include <ESL_BOARD_config.h>
#include <ecSlvApi.h>

#include "ti_board_config.h"
#include "ti_board_open_close.h"

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Initialize board status LED
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pGpioHandle_p   GPIO handle
 *  \param[in]  selectedPru_p   Pru Instance index
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *
 *  // the Call
 *  ESL_BOARD_OS_initStatusLED(gpioHandle, EC_API_SLV_ePRUICSS_INSTANCE_ONE);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup ESL_OS
 *
 * */
void ESL_BOARD_OS_initStatusLED(void* pGpioHandle_p, uint32_t selectedPru_p)
{
    OSALUNREF_PARM(selectedPru_p);

#if (defined GPIO_BANK_LEDSTATUS) && (defined GPIO_PIN_LEDSTATUS)
    ESL_GPIO_setConfigMode(pGpioHandle_p, GPIO_BANK_LEDSTATUS, GPIO_PIN_LEDSTATUS, ESL_GPIO_enDIRECTION_MODE_OUTPUT, ESL_GPIO_enIRQ_MODE_NONE);
#endif
#if (defined GPIO_BANK_LEDERROR) && (defined GPIO_PIN_LEDERROR)
    ESL_GPIO_setConfigMode(pGpioHandle_p, GPIO_BANK_LEDERROR,  GPIO_PIN_LEDERROR,  ESL_GPIO_enDIRECTION_MODE_OUTPUT, ESL_GPIO_enIRQ_MODE_NONE);
#endif
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Configure/control board status LED
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pGpioHandle_p   GPIO handle
 *  \param[in]  selectedPru_p   Pru Instance index
 *  \param[in]  runLed_p        Status LED on/off
 *  \param[in]  errLed_p        Error LED on/off
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *
 *  // the Call
 *  ESL_BOARD_OS_statusLED(gpioHandle, EC_API_SLV_ePRUICSS_INSTANCE_ONE, true, false);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup ESL_OS
 *
 * */
void ESL_BOARD_OS_statusLED(void* pGpioHandle_p, uint32_t selectedPru_p, bool runLed_p, bool errLed_p)
{
    OSALUNREF_PARM(selectedPru_p);
    OSALUNREF_PARM(pGpioHandle_p);

#if !(defined FBTLPROVIDER) || (FBTLPROVIDER==0)
    if (runLed_p) /* ioexp port 0 */
    {
        LED_on(gLedHandle[CONFIG_LED_STATUS], 0);
    }
    else
    {
        LED_off(gLedHandle[CONFIG_LED_STATUS], 0);
    }

    if (errLed_p) /* gpio */
    {
        LED_on(gLedHandle[CONFIG_LED_ERROR], 0);
    }
    else
    {
        LED_off(gLedHandle[CONFIG_LED_ERROR], 0);
    }
#endif
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Configure PHY reset lines
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pGpioHandle_p   GPIO driver handle.
 *  \param[in]  selectedPru_p   PRU instance number.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <ESL_BOARD_config.h>
 *
 *  // the call
 *  ESL_BOARD_OS_configureResets(gpioHandle, EC_API_SLV_ePRUICSS_INSTANCE_ONE);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup ESL_OS
 *
 * */
void ESL_BOARD_OS_configureResets(void* pGpioHandle_p, uint32_t selectedPru_p)
{
    OSALUNREF_PARM(selectedPru_p);
#if (defined APP_PHYIN_RESET_MODULE) && (defined APP_PHYIN_RESET_PIN) && (defined APP_PHYOUT_RESET_MODULE) && (defined APP_PHYOUT_RESET_PIN)
    ESL_GPIO_setConfigMode(pGpioHandle_p, APP_PHYIN_RESET_MODULE,  APP_PHYIN_RESET_PIN,
                           ESL_GPIO_enDIRECTION_MODE_OUTPUT, ESL_GPIO_enIRQ_MODE_NONE);
    ESL_GPIO_setConfigMode(pGpioHandle_p, APP_PHYOUT_RESET_MODULE, APP_PHYOUT_RESET_PIN,
                           ESL_GPIO_enDIRECTION_MODE_OUTPUT, ESL_GPIO_enIRQ_MODE_NONE);
#else
#error "No PHY reset pins defined"
#endif
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Issue PHY reset
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pGpioHandle_p   GPIO driver handle.
 *  \param[in]  selectedPru_p   PRU instance number.
 *  \param[in]  phyIdx_p        PHY identifier.
 *  \param[in]  reset_p         true: reset, false: run
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <ESL_BOARD_config.h>
 *
 *  // the call
 *  ESL_BOARD_OS_phyReset(gpioHandle, EC_API_SLV_ePRUICSS_INSTANCE_ONE, EC_API_SLV_ePHY_IN, true);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup ESL_OS
 *
 * */
void ESL_BOARD_OS_phyReset(void* pGpioHandle_p, uint32_t selectedPru_p, uint8_t phyIdx_p, bool reset_p)
{
    uint8_t                     module                  = (uint8_t)~0;
    uint8_t                     pin                     = (uint8_t)~0;

    OSALUNREF_PARM(selectedPru_p);

    switch(phyIdx_p)
    {
    case EC_API_SLV_ePHY_IN:
        module  = APP_PHYIN_RESET_MODULE;
        pin     = APP_PHYIN_RESET_PIN;
        break;
    case EC_API_SLV_ePHY_OUT:
        module  = APP_PHYOUT_RESET_MODULE;
        pin     = APP_PHYOUT_RESET_PIN;
        break;
    default:
        return;
    }

    if ((uint8_t)-1 == pin || (uint8_t)-1 == module)
    {
        OSAL_printf("Invalid PHY config %u\r\n", phyIdx_p);
        OSAL_error(__func__, __LINE__, OSAL_STACK_PHYDETECT_ERROR, true, 0);
    }

    if (reset_p)
    {
        OSAL_printf("Phy Reset: %u.%u\r\n", module, pin);
        ESL_GPIO_write(pGpioHandle_p, (ESL_GPIO_EModule_t)module, (ESL_GPIO_EPin_t)pin, ESL_GPIO_enPINSTATE_LOW);
    }
    else
    {
        OSAL_printf("Phy UnReset: %u.%u\r\n", module, pin);
        ESL_GPIO_write(pGpioHandle_p, (ESL_GPIO_EModule_t)module, (ESL_GPIO_EPin_t)pin, ESL_GPIO_enPINSTATE_HIGH);
    }
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Register PHY descriptor to DTK
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  selectedPru_p   PRU instance number.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <ESL_BOARD_config.h>
 *
 *  // the call
 *  ESL_BOARD_OS_registerPhys(EC_API_SLV_ePRUICSS_INSTANCE_ONE);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup ESL_OS
 *
 * */
void ESL_BOARD_OS_registerPhys(uint32_t selectedPru_p)
{
#if !(defined ECAT_PHYADDR_IN) && !(defined ECAT_PHYADDR_OUT)
#error "EtherCAT without PHY is useless"
#endif

#if !(defined DPRAM_REMOTE) && !(defined FBTL_REMOTE)
    EC_API_SLV_registerPhy(EC_API_SLV_ePHY_IN,  ECAT_PHYADDR_IN,  ECAT_PHYPOLINVERT_IN,  ECAT_PHYUSERXLINK_IN);
    EC_API_SLV_registerPhy(EC_API_SLV_ePHY_OUT, ECAT_PHYADDR_OUT, ECAT_PHYPOLINVERT_OUT, ECAT_PHYUSERXLINK_OUT);
#endif
}

//*************************************************************************************************
