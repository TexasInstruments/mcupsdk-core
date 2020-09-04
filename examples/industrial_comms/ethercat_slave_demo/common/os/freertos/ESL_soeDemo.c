/*!
 *  \file ESL_soeDemo.c
 *
 *  \brief
 *  Application SoE support FreeRTOS.
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

#include <ESL_soeDemo.h>

/*! <!-- Description: -->
 *
 *  \brief
 *  SoE Service received from the master. Application must prepare a response.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  soEService_p    SoE Service number.
 *  \param[in]  soEFlags_p		SoE Flag parameters.
 *  \param[out] pData_p         Response data.
 *  \param[out] pLen_p          Add to the parameter the length of the response data size.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <theHeader.h>
 *
 *  // required variables
 *  uint32_t retVal = 0;
 *  void* pvVariable = NULL;
 *
 *  // the Call
 *  EC_SLV_APP_SoE_recv(soEService_p, soEFlags_p, pData_p, pLen_p);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
void EC_SLV_APP_SoE_recv(uint16_t soEService_p, uint16_t soEFlags_p, void* pData_p, uint16_t *pLen_p)
{
    uint8_t     example[10] = {06, 00, 06, 00, 'H', 'E', 'L', 'L', 'O', '!'};
    uint16_t    errorCode = 0;

    OSALUNREF_PARM(soEFlags_p);

    switch (soEService_p)
    {
    case ECAT_SOE_OPCODE_RRQ:
    {
        *pLen_p = sizeof(example);
        OSAL_MEMORY_memcpy(pData_p, &example, sizeof(example));
    } break;
    case ECAT_SOE_OPCODE_WRQ:
    {
        if ( *pLen_p < sizeof(uint32_t))
        {
            errorCode = 0x7002;
        }
        else if ( *pLen_p > sizeof(uint32_t) )
        {
            errorCode = 0x7003;
        }
        *pLen_p += sizeof(errorCode);
        OSAL_MEMORY_memcpy(pData_p, &errorCode, sizeof(errorCode));
    } break;
    default:
        break;
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Called when the master sends a notification request.
 *
 *  \details
 *  Respond with servo parameters.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \return     ErrorCode Mailbox error code.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <theHeader.h>
 *
 *  // required variables
 *  uint32_t retVal = 0;
 *
 *  // the Call
 *  retVal = EC_SLV_APP_SoE_nofReq(void);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
uint8_t EC_SLV_APP_SoE_nofReq(void)
{
    uint16_t    idn         = 0;
    uint16_t    channel     = 0;
    uint16_t    dataState   = 0xFFFF;

    return EC_API_SLV_notificationResponse(idn, channel, dataState);
}

//*************************************************************************************************
