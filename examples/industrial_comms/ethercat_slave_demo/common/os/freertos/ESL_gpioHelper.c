/*!
 *  \file ESL_gpioHelper.c
 *
 *  \brief
 *  Application GPIO support FreeRTOS.
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2021-05-18
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

#include <osal.h>
#include <ESL_gpioHelper.h>
#include <ESL_os.h>
#include <ESL_BOARD_config.h>
#include <ESL_BOARD_OS_config.h>

#include <drivers/gpio.h>
#include <drivers/hw_include/am64x_am243x/cslr_soc_baseaddress.h>
#include <board/led.h>

typedef struct ESL_GPIO_SHandle
{
    uint32_t                dummy;
} ESL_GPIO_SHandle_t;


#if (defined GPIO_TEST_PINS) && (1==GPIO_TEST_PINS)
static void ESL_GPIO_testPins_init(void);
#endif

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Initializes GPIO interface structure.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \return     GPIO handle.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <ESL_gpioHelper.h>
 *
 *  void* gpioHandle;
 *
 *  // the Call
 *  gpioHandle = ESL_GPIO_init();
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup ESL_GPIO
 *
 * */
void* ESL_GPIO_init ( void )
{
    ESL_GPIO_SHandle_t* pGpioHandle = NULL;

    pGpioHandle = (ESL_GPIO_SHandle_t*)OSAL_MEMORY_calloc(1, sizeof(ESL_GPIO_SHandle_t));

    /* nothing to do on FRT */

    if (NULL == pGpioHandle)
    {
        goto Exit;
    }

Exit:
    return pGpioHandle;
}

/*!
 *  \brief
 *  Get IRQ flag.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pGpioHandle_p   GPIO handle
 *  \param[in]  irqMode_p       IRQ mode.
 *  \return     IRQ flag depending OS.
 *
 *  \ingroup ESL_GPIO
 *
 */
static uint32_t OSAL_FUNC_UNUSED ESL_GPIO_getIrqFlag(void* pGpioHandle_p, ESL_GPIO_EInterruptMode_t irqMode_p)
{
    uint32_t    retVal;

    OSALUNREF_PARM(pGpioHandle_p);
    OSALUNREF_PARM(irqMode_p);

    retVal = 0;

    return retVal;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Sets Direction mode of GPIO pin.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pGpioHandle_p   GPIO handle
 *  \param[in]  moduleId_p      GPIO Module.
 *  \param[in]  pinId_p         Pin Id within Module
 *  \param[in]  directionMode_p Pin Direction
 *  \param[in]  intMode_p       Interrupt mode
 *  \return     true on success, false otherwise
 *
 *  \remarks
 *  ESL_GPIO_init has to be called before this function is used.
 *
 *  <!-- References: -->
 *
 *  \sa ESL_GPIO_init
 *
 *  <!-- Group: -->
 *
 *  \ingroup ESL_GPIO
 *
 * */
bool ESL_GPIO_setConfigMode(void* pGpioHandle_p, ESL_GPIO_EModule_t moduleId_p, ESL_GPIO_EPin_t pinId_p
                           ,ESL_GPIO_EDirectionMode_t  directionMode_p, ESL_GPIO_EInterruptMode_t intMode_p)
{
    bool        bRet = false;
    uint32_t    baseAddr;
    uint8_t     direction;

    OSALUNREF_PARM(pGpioHandle_p);
    OSALUNREF_PARM(intMode_p);

    switch (moduleId_p)
    {
    case ESL_GPIO_enMODULE_0:   baseAddr = CSL_MCU_GPIO0_BASE; break;
    default: goto Exit;
    }

    switch(directionMode_p)
    {
    case ESL_GPIO_enDIRECTION_MODE_INPUT:   direction = GPIO_DIRECTION_INPUT; break;
    case ESL_GPIO_enDIRECTION_MODE_OUTPUT:  direction = GPIO_DIRECTION_OUTPUT; break;
    }

    GPIO_setDirMode(baseAddr, pinId_p, direction);

    bRet = true;
Exit:
    return bRet;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Apply GPIO settings
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pGpioHandle_p   GPIO handle
 *
 *  <!-- References: -->
 *
 *  \sa ESL_GPIO_init
 *
 *  <!-- Group: -->
 *
 *  \ingroup ESL_GPIO
 *
 * */
void ESL_GPIO_apply(void* pGpioHandle_p)
{
    OSALUNREF_PARM(pGpioHandle_p);

#if (defined GPIO_TEST_PINS) && (1==GPIO_TEST_PINS)
    ESL_GPIO_testPins_init();
#endif
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Reads pin state from GPIO input.
 *
 *  \details
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pGpioHandle_p   GPIO handle
 *  \param[in]  moduleId_p      GPIO Module.
 *  \param[in]  pinId_p         Pin Id within Module
 *  \return     true if PIN is high, false otherwise
 *
 *  \remarks
 *  ESL_GPIO_init has to be called before this function is used.
 *
 *  <!-- References: -->
 *
 *  \sa ESL_GPIO_init
 *
 *  <!-- Group: -->
 *
 *  \ingroup ESL_GPIO
 *
 * */
bool ESL_GPIO_read(void* pGpioHandle_p, ESL_GPIO_EModule_t  moduleId_p, ESL_GPIO_EPin_t pinId_p )
{
    bool		state       = false;
    uint32_t    value       = 0;

    uint32_t    baseAddr;

    OSALUNREF_PARM(pGpioHandle_p);

    switch (moduleId_p)
    {
    case ESL_GPIO_enMODULE_0:   baseAddr = CSL_MCU_GPIO0_BASE; break;
    default: goto Exit;
    }

    value = GPIO_pinRead(baseAddr, pinId_p);

    if (value)
    {
        state = true;
    }

Exit:
    return state;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Write GPIO Pin
 *
 *  \details
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pGpioHandle_p   GPIO handle
 *  \param[in]  moduleId_p      GPIO Module.
 *  \param[in]  pinId_p         Pin Id within Module
 *  \param[in]  pinState_p      Pin State
 *  \return     true on success, false otherwise.
 *
 *  \remarks
 *  ESL_GPIO_init has to be called before this function is used
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *
 *  <!-- References: -->
 *
 *  \sa ESL_GPIO_init
 *
 *  <!-- Group: -->
 *
 *  \ingroup ESL_GPIO
 *
 * */
bool ESL_GPIO_write(void* pGpioHandle_p, ESL_GPIO_EModule_t moduleId_p, ESL_GPIO_EPin_t pinId_p, ESL_GPIO_EPinState_t pinState_p )
{
    bool                        state       = false;
    uint32_t                    baseAddr;

    OSALUNREF_PARM(pGpioHandle_p);

    switch (moduleId_p)
    {
    case ESL_GPIO_enMODULE_0:       baseAddr = CSL_MCU_GPIO0_BASE; break;
    case ESL_GPIO_enMODULE_LED: break;
    default: goto Exit;
    }

    if (ESL_GPIO_enMODULE_LED == moduleId_p)
    {
#if (defined CONFIG_PHY0_RESET) || (defined CONFIG_PHY1_RESET)
        switch (pinId_p)
        {
#if (defined CONFIG_PHY0_RESET)
        case ESL_GPIO_enPIN_03: pinId_p = CONFIG_PHY0_RESET; break;
#endif
#if (defined CONFIG_PHY1_RESET)
        case ESL_GPIO_enPIN_04: pinId_p = CONFIG_PHY1_RESET; break;
#endif
        default: break;
        }

        if (pinState_p==ESL_GPIO_enPINSTATE_HIGH)
        {
            LED_on(gLedHandle[pinId_p], 0);
        }
        else if (pinState_p==ESL_GPIO_enPINSTATE_LOW)
        {
            LED_off(gLedHandle[pinId_p], 0);
        }
        else
        {
            goto Exit;
        }
#else
        goto Exit;
#endif
    }
    else
    {
        if (pinState_p==ESL_GPIO_enPINSTATE_HIGH)
        {
            GPIO_pinWriteHigh(baseAddr, pinId_p);
        }
        else if (pinState_p==ESL_GPIO_enPINSTATE_LOW)
        {
            GPIO_pinWriteLow(baseAddr, pinId_p);
        }
        else
        {
            goto Exit;
        }
    }

    state = true;
Exit:
    return state;
}

/*!
 *  \brief
 *  Get Pin index number.
 *
 *  \details
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pGpioHandle_p   GPIO handle
 *  \param[in]  moduleId_p      GPIO Module.
 *  \param[in]  pinId_p         Pin Id within Module
 *  \return     GPIO pin index number as uint32_t
 *
 *  \ingroup ESL_GPIO
 *
 */
uint32_t ESL_GPIO_getEntryIndex(void* pGpioHandle_p, ESL_GPIO_EModule_t moduleId_p, ESL_GPIO_EPin_t pinId_p)
{
    uint32_t    found = ~0;

    return found;
}

/*!
 *  \brief
 *  Register a callback to a GPIO pin.
 *
 *  \details
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pGpioHandle_p   GPIO handle
 *  \param[in]  moduleId_p      GPIO Module.
 *  \param[in]  pinId_p         Pin Id within Module.
 *  \param[in]  callback_p      Callback function.
 *  \return     Callback register ok or not.
 *
 *  \ingroup ESL_GPIO
 *
 */
bool ESL_GPIO_setCallback(void* pGpioHandle_p, ESL_GPIO_EModule_t moduleId_p, ESL_GPIO_EPin_t pinId_p, ESL_GPIO_CBIrq_t callback_p)
{
    bool    ret     = false;

    OSALUNREF_PARM(moduleId_p);
    OSALUNREF_PARM(pinId_p);
    OSALUNREF_PARM(callback_p);

    return ret;
}

/*!
 *  \brief
 *  Enable or disable IRQ
 *
 *  \details
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pGpioHandle_p   GPIO handle
 *  \param[in]  moduleId_p      GPIO Module.
 *  \param[in]  pinId_p         Pin Id within Module.
 *  \param[in]  enable_p        Enable or disable IRQ.
 *  \return     GPIO enable IRQ ok or not.
 *
 *  \ingroup ESL_GPIO
 *
 */
bool ESL_GPIO_enableIrq(void* pGpioHandle_p, ESL_GPIO_EModule_t moduleId_p, ESL_GPIO_EPin_t pinId_p, bool enable_p)
{
    bool    ret     = false;

    OSALUNREF_PARM(moduleId_p);
    OSALUNREF_PARM(pinId_p);
    OSALUNREF_PARM(enable_p);

    return ret;
}

#if (defined GPIO_TEST_PINS) && (1==GPIO_TEST_PINS)

void ESL_GPIO_testPins_init(void)
{
   for(ESL_GPIO_eTestPinNames_t pin = ESL_TESTPIN__FIRST; pin < ESL_TESTPIN__COUNT; pin++)
   {
       const ESL_GPIO_STestPinCfg_t * pPingCfg = &ESL_GPIO_testPinCfg_g[pin];
       GPIO_setDirMode(pPingCfg->baseAddress, pPingCfg->pinNumber, pPingCfg->pinDirection);
   }
}

void ESL_GPIO_testPins_setHigh(const ESL_GPIO_eTestPinNames_t testpin_p)
{
    GPIO_pinWriteHigh(ESL_GPIO_testPinCfg_g[testpin_p].baseAddress, ESL_GPIO_testPinCfg_g[testpin_p].pinNumber);
}

void ESL_GPIO_testPins_setLow(const ESL_GPIO_eTestPinNames_t testpin_p)
{
    GPIO_pinWriteLow(ESL_GPIO_testPinCfg_g[testpin_p].baseAddress, ESL_GPIO_testPinCfg_g[testpin_p].pinNumber);
}

/* Set pins given as a pin mask (bit set to : 1 means set, 0 means clear). Max 8 pins.
 * Duration: ~200ns */
void ESL_GPIO_testPins_write(volatile CSL_GpioBank_registersRegs * const pGpioRegBank_p, const uint32_t pinMask_p)
{
    CSL_REG32_WR(&pGpioRegBank_p->OUT_DATA, pinMask_p);
}

/* Activate pins given as a pin mask (bit set to 1 means set) and the rest keep as it is. Max 8 pins.
 * Duration: ~400ns */

void ESL_GPIO_testPins_set(volatile CSL_GpioBank_registersRegs * const pGpioRegBank_p, const uint32_t pinMask_p)
{
    const uint32_t newPinMask = CSL_REG32_RD(&pGpioRegBank_p->OUT_DATA) | pinMask_p;
    CSL_REG32_WR(&pGpioRegBank_p->OUT_DATA, newPinMask);
}

/* Clear pins given as a pin mask (bit set to 1 means clear) and the rest keep as it is. Max 8 pins.
 * Duration: ~400ns */
void ESL_GPIO_testPins_clear(volatile CSL_GpioBank_registersRegs * const pGpioRegBank_p, const uint32_t pinMask_p)
{
    const uint32_t newPinMask = CSL_REG32_RD(&pGpioRegBank_p->OUT_DATA) & ~pinMask_p;
    CSL_REG32_WR(&pGpioRegBank_p->OUT_DATA, newPinMask);
}
#endif

//*************************************************************************************************
