/*!
* \file FBTL_serviceTypes.h
*
* \brief
* FBTL global / generic services.
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

#if !(defined __FBTL_SERVICETYPES_H__)
#define __FBTL_SERVICETYPES_H__		1
/**

\brief Service Code Enum for global services

This enumeration is used to identify the service.
The Service IDs are splitted into groups. The 4 bit MSB are used to idetify the group
- 0 = General Service
- 1 = EtherCAT Service
- ...

\ingroup grp_gen_service

*/
// Service 16 bit -> highest 4 Bit = Unit (General or Bus)
typedef enum FBTL_EService
{
    /**
    \brief Unknown Service

    Reserved for transmission and used as Generic callback indicator
    */
    FBTL_eService_unknown = 0,

    /**
    \brief Setup the used bus

    use \ref FBTL_EServiceType
    */
    FBTL_eService_setBus,

    /**
    \brief Set CPU Speed

    */
    FBTL_eService_setCpuSpeed,

    /**
    \brief Get CPU Speed

    */
    FBTL_eService_getCpuSpeed,

    /**
    \brief Set Process Data
    */
    //FBTL_eService_setProcessData,

    /**
    \brief Get Process Data
    */
    //FBTL_eService_getProcessData,

    /**
    \brief Trigger Processdata exchange
    */
    FBTL_eService_pdExchange,

    /**
    \brief Initialize the API.
    */
    FBTL_eService_init,

    /**
    \brief Start the Protocol Stack
    */
    FBTL_eService_start,

    /**
    \brief Stop the Protocol Stack
    */
    FBTL_eService_stop,

    /**
    \brief Delete the API
    */
    FBTL_eService_delete,

    /**
    \brief Set Processdata configuration

    */
    FBTL_eService_setPdConfig,

    /**
    \brief Virtual MAC tunnel

    */
    FBTL_eService_ethernet,

    FBTL_eService_GeneralMax,
} FBTL_EService_t;

#if (defined __cplusplus)
extern "C" {
#endif

/* extern void func(void); */

#if (defined __cplusplus)
}
#endif

#endif /* __FBTL_SERVICETYPES_H__ */
