/*!
 *  \file CMN_board.c
 *
 *  \brief
 *  Provides initialization of common board module.
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

#if (defined CMN_MEM_TRACE) && (1==CMN_MEM_TRACE)
#include <CMN_mem.h>
#endif

#include <CMN_board.h>

/*!
* <!-- Description: -->
*
* \brief
* Common board initialization for FreeRTOS.
*
*  \return     uint32_t                       Error code.
*
*  \retval     #CMN_BOARD_eERR_NOERROR        Success.
*  \retval     #CMN_BOARD_eERR_GENERALERROR   Initialization failed.
*
*/
uint32_t CMN_BOARD_init(void)
{
#if (defined CMN_MEM_TRACE) && (1==CMN_MEM_TRACE)
    CMN_MEM_traceInit();
#endif

    Board_init();

    return CMN_BOARD_eERR_NOERROR;
}

/*!
* <!-- Description: -->
*
* \brief
* Common board deinitialization for FreeRTOS.
*
* \return     uint32_t                      Error code.
*
* \retval     #CMN_BOARD_eERR_NOERROR       Success.
* \retval     #CMN_BOARD_eERR_GENERALERROR  Deinitialization failed.
*
*/
uint32_t CMN_BOARD_deinit(void)
{
    Board_deinit();

    return CMN_BOARD_eERR_NOERROR;
}
