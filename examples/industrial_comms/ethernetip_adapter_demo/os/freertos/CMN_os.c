/*!
 *  \file CMN_os.c
 *
 *  \brief
 *  Common OS support of FreeRTOS.
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

#include "ti_drivers_config.h"
#include <drivers/sciclient/include/sciclient_pm.h>

#include <CMN_os.h>

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Initialize OS module
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <CMN_os.h>
 *
 *  // the Call
 *  CMN_OS_init();
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup CMN_OS
 *
 * */
void CMN_OS_init(void)
{
    System_init();
    return;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Deinitialize OS module
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <CMN_os.h>
 *
 *  // the Call
 *  CMN_OS_deinit();
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup CMN_OS
 *
 * */
void CMN_OS_deinit(void)
{
    System_deinit();
    return;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Reset OS module
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <CMN_os.h>
 *
 *  // the Call
 *  CMN_OS_reset();
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup CMN_OS
 *
 * */
void CMN_OS_reset (void)
{
    Sciclient_pmDeviceReset(0xFFFFFFFFU);
}

//*************************************************************************************************
