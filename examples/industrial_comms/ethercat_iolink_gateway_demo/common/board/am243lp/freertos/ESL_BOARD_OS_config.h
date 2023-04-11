/*!
 *  \file ESL_BOARD_OS_config.h
 *
 *  \brief
 *  Board OS dependent configuration FreeRTOS AM243X Launchpad.
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2022-02-11
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

#if !(defined __ESL_BOARD_OS_CONFIG_H__)
#define __ESL_BOARD_OS_CONFIG_H__		1
#include <ESL_os.h>

#include <ti_drivers_config.h>

#define ESL_DEFAULT_PRUICSS                 CONFIG_PRU_ICSS1

// PHY0 GPIO pins
#define APP_PHYIN_RESET_PIN                 ESL_GPIO_enPIN_28    /* E1/E2: ESL_GPIO_enPIN_73 */
#define APP_PHYIN_RESET_MODULE              ESL_GPIO_enMODULE_0
#define APP_PHYOUT_RESET_PIN                ESL_GPIO_enPIN_20
#define APP_PHYOUT_RESET_MODULE             ESL_GPIO_enMODULE_0

#if (defined CONFIG_GPIO_TEST0_PIN) || (defined CONFIG_GPIO_TEST1_PIN) || (defined CONFIG_GPIO_TEST2_PIN || CONFIG_GPIO_TEST3_PIN)
/* SYSCFG enhance for Testpins: be sure gpioX does not yet exist, otherwise fix indice
 * const gpio3      = gpio.addInstance();
 * const gpio4      = gpio.addInstance();
 * const gpio5      = gpio.addInstance();
 * const gpio6      = gpio.addInstance();
 * gpio3.$name                = "CONFIG_GPIO_TEST0";
 * gpio3.pinDir               = "OUTPUT";
 * gpio3.GPIO.gpioPin.rx      = false;
 * gpio3.GPIO.gpioPin.$assign = "ball.J3";
 * gpio4.$name                = "CONFIG_GPIO_TEST1";
 * gpio4.pinDir               = "OUTPUT";
 * gpio4.GPIO.gpioPin.rx      = false;
 * gpio4.GPIO.gpioPin.$assign = "ball.J4";
 * gpio5.$name                = "CONFIG_GPIO_TEST2";
 * gpio5.pinDir               = "OUTPUT";
 * gpio5.GPIO.gpioPin.rx      = false;
 * gpio5.GPIO.gpioPin.$assign = "ball.G1";
 * gpio6.$name                = "CONFIG_GPIO_TEST3";
 * gpio6.pinDir               = "OUTPUT";
 * gpio6.GPIO.gpioPin.rx      = false;
 * gpio6.GPIO.gpioPin.$assign = "ball.H1";
 * gpio3.GPIO.$suggestSolution                                = "GPIO1";
 * gpio4.GPIO.$suggestSolution                                = "GPIO1";
 * gpio5.GPIO.$suggestSolution                                = "GPIO1";
 * gpio6.GPIO.$suggestSolution                                = "GPIO1";
 */
#define GPIO_TEST_PINS  1

#define ESL_TESTPIN_0_MASK      GPIO_GET_BIT_MASK(CONFIG_GPIO_TEST0_PIN)
#define ESL_TESTPIN_1_MASK      GPIO_GET_BIT_MASK(CONFIG_GPIO_TEST1_PIN)
#define ESL_TESTPIN_2_MASK      GPIO_GET_BIT_MASK(CONFIG_GPIO_TEST2_PIN)
#define ESL_TESTPIN_3_MASK      GPIO_GET_BIT_MASK(CONFIG_GPIO_TEST3_PIN)

#define ESL_TESTPIN_0_REG_IDX   GPIO_GET_REG_INDEX(CONFIG_GPIO_TEST0_PIN)
#define ESL_TESTPIN_1_REG_IDX   GPIO_GET_REG_INDEX(CONFIG_GPIO_TEST1_PIN)
#define ESL_TESTPIN_2_REG_IDX   GPIO_GET_REG_INDEX(CONFIG_GPIO_TEST2_PIN)
#define ESL_TESTPIN_3_REG_IDX   GPIO_GET_REG_INDEX(CONFIG_GPIO_TEST3_PIN)

#define ESL_TESTPIN_0_GPIO      (volatile CSL_GpioRegs*)((uintptr_t) CONFIG_GPIO_TEST0_BASE_ADDR)
#define ESL_TESTPIN_1_GPIO      (volatile CSL_GpioRegs*)((uintptr_t) CONFIG_GPIO_TEST1_BASE_ADDR)
#define ESL_TESTPIN_2_GPIO      (volatile CSL_GpioRegs*)((uintptr_t) CONFIG_GPIO_TEST2_BASE_ADDR)
#define ESL_TESTPIN_3_GPIO      (volatile CSL_GpioRegs*)((uintptr_t) CONFIG_GPIO_TEST3_BASE_ADDR)

/* Pin register bank */
#define ESL_TESTPIN_0_REG_BANK  &((ESL_TESTPIN_0_GPIO)->BANK_REGISTERS[ESL_TESTPIN_0_REG_IDX])
#define ESL_TESTPIN_1_REG_BANK  &((ESL_TESTPIN_1_GPIO)->BANK_REGISTERS[ESL_TESTPIN_1_REG_IDX])
#define ESL_TESTPIN_2_REG_BANK  &((ESL_TESTPIN_2_GPIO)->BANK_REGISTERS[ESL_TESTPIN_2_REG_IDX])
#define ESL_TESTPIN_3_REG_BANK  &((ESL_TESTPIN_3_GPIO)->BANK_REGISTERS[ESL_TESTPIN_3_REG_IDX])

#define ESL_TESTPIN_STATE_REG_BANK ESL_TESTPIN_0_REG_BANK

extern const ESL_GPIO_STestPinCfg_t ESL_GPIO_testPinCfg_g[];

#else
#define GPIO_TEST_PINS  0
#endif

#if (defined GPIO_TEST_PINS) && (1==GPIO_TEST_PINS)
#define GPIO_TEST_PROFILE_1     0x0001  /* measure App only */
#define GPIO_TEST_PROFILE_2     0x0002  /* measure ISR threads */
#define GPIO_TEST_PROFILE_SEL   GPIO_TEST_PROFILE_2
#endif

#if (defined __cplusplus)
extern "C" {
#endif

/* extern void func(void); */

#if (defined __cplusplus)
}
#endif

#endif /* __ESL_BOARD_OS_CONFIG_H__ */
