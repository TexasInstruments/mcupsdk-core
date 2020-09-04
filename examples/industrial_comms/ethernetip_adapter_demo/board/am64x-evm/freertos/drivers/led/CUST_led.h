/*!
 *  \file CUST_led.h
 *
 *  \brief
 *  Custom LED's interface.
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

#if !(defined PROTECT_CUST_LED_H)
#define PROTECT_CUST_LED_H     1

#include <osal.h>

typedef enum CUST_LED_EError
{
    CUST_LED_eERR_NOERROR            =  0,   /*!< No error, everything is fine. */
    CUST_LED_eERR_OFF                = -5,   /*!< LED off failed. */
    CUST_LED_eERR_SET_MASK           = -4,   /*!< LED set mask failed. */
    CUST_LED_eERR_ATTRIBUTES_INVALID = -3,   /*!< LED attributes are invalid. */
    CUST_LED_eERR_HANDLE_INVALID     = -2,   /*!< LED handle is invalid. */
    CUST_LED_eERR_GENERALERROR       = -1    /*!< General error */
} CUST_LED_EError_t;

#if (defined __cplusplus)
extern "C" {
#endif

extern uint32_t CUST_LED_init               (void);
extern uint32_t CUST_LED_deInit             (void);

#if (defined __cplusplus)
}
#endif

#endif /* PROTECT_CUST_LED_H */
