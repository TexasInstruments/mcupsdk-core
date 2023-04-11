/*!
 *  \file ESL_BOARD_OS_config.h
 *
 *  \brief
 *  Board OS dependent configuration FreeRTOS AM243X GP EVM.
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

#if !(defined __ESL_BOARD_OS_CONFIG_H__)
#define __ESL_BOARD_OS_CONFIG_H__		1
#include <ESL_os.h>

#include <ti_drivers_config.h>

#define ESL_DEFAULT_PRUICSS                 CONFIG_PRU_ICSS1

/* Only 1 set of LEDs on AM65 EVM, define identically, PRU independent */

// PHY0 GPIO pins
#define APP_PHYIN_RESET_PIN                 ESL_GPIO_enPIN_03
#define APP_PHYIN_RESET_MODULE              ESL_GPIO_enMODULE_LED
#define APP_PHYOUT_RESET_PIN                ESL_GPIO_enPIN_04
#define APP_PHYOUT_RESET_MODULE             ESL_GPIO_enMODULE_LED

#define GPIO_TEST_PINS  0

#if (defined __cplusplus)
extern "C" {
#endif

/* extern void func(void); */

#if (defined __cplusplus)
}
#endif

#endif /* __ESL_BOARD_OS_CONFIG_H__ */
