/* 
 *   Copyright (c) Texas Instruments Incorporated 2022
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef SDL_RTI_H_
#define SDL_RTI_H_
#include <sdl/rti/v0/sdl_ip_rti.h>
#ifdef __cplusplus
extern "C" {
#endif


/**
 *
 * @ingroup  SDL_MODULE
 * @defgroup SDL_RTI_API APIs for SDL RTI
 *
 *    The Digital Watchdog Timer(DWT) generates reset after a programmable
 *    period, if not serviced within that period. In DWT, time-out
 *    boundary is configurable.
 *    In DWWD, along with configurable time-out boundary, the start time
 *    boundary is also configurable. The DWWD can generate Reset or
 *    Interrupt, if not serviced within window(Open Window) defined by
 *    start time and time-out boundary. Also the DWWD can generate Reset or
 *    Interrupt if serviced outside Open Window (within Closed Window).
 *    Generation of Reset or Interrupt depends on the DWWD Reaction
 *    configuration.
 *
 */

/*  Declarations of API's  */
/**
@defgroup SDL_RTI_FUNCTION  RTI Functions
@ingroup SDL_RTI_API
*/

/**
 *  @addtogroup SDL_RTI_FUNCTION
    @{
 */

/**
 *
 * \brief   RTI configuration API.
 *          This API configures the specified RTI instance using the provided
 *          configuration.
 *
 * \param   InstanceType: RTI Instance
 * \param   pConfig: Pointer to the configuration structure
 *
 * \return  SDL_PASS for Success. SDL Error Code for failure.
 */
int32_t SDL_RTI_config(SDL_RTI_InstanceType InstanceType, const SDL_RTI_configParms *pConfig);

/**
 *
 * \brief   RTI API to verify the written configuration.
 *          This API can be used after calling #SDL_RTI_config in order to
 *          verify the configuration. It reads back the configuration from the
 *          registers and compares it with the provided configuration.
 *
 * \param   InstanceType: RTI Instance
 * \param   pConfig: Pointer to the configuration structure
 *
 * \return  SDL_PASS if verification passed. SDL Error Code for failure.
 */
int32_t SDL_RTI_verifyConfig(SDL_RTI_InstanceType InstanceType, SDL_RTI_configParms *pConfig);

/**
 *
 * \brief   Enable the RTI instance.
 *
 * \param   InstanceType: RTI Instance
 *
 * \return  SDL_PASS if success. SDL Error Code for failure.
 *
 * \note    Should be called after configuration of the RTI
 */
int32_t SDL_RTI_start(SDL_RTI_InstanceType InstanceType);

/**
 *
 * \brief   Service the RTI instance.
 *
 * \param   InstanceType: RTI Instance
 *
 * \return  SDL_PASS if success. SDL Error Code for failure.
 */
int32_t SDL_RTI_service(SDL_RTI_InstanceType InstanceType);

/**
 *
 * \brief   Clear the DWWD status.
 *
 * \param   InstanceType: RTI Instance
 * \param   status: Violation status to clear.
 *                  Values given by bitwise or of macro #RTI_Status_t
 *
 * \return  SDL_PASS if success. SDL Error Code for failure.
 */
int32_t SDL_RTI_clearStatus(SDL_RTI_InstanceType InstanceType, uint32_t status);

/**
 *
 * \brief   Clear the DWWD status.
 *
 * \param   InstanceType: RTI Instance
 * \param   pStatus     pointer to DWWD status. Refer macro #RTI_Status_t for
 *                      possible bitwise or of all return values.
 *
 * \return  SDL_PASS if success. SDL Error Code for failure.
 */
int32_t SDL_RTI_getStatus(SDL_RTI_InstanceType InstanceType, uint32_t *pStatus);

/**
 *
 * \brief   This API returns the static registers for RTI-DWWD.
 *
 * \param   InstanceType    Base Address of the RTI instance.
 * \param   pStaticRegs     Pointer to RTI DWWD Static Registers.
 *
 * \return  SDL_PASS if success. SDL Error Code for failure.
 */
int32_t SDL_RTI_readStaticRegs(SDL_RTI_InstanceType InstanceType, SDL_RTI_staticRegs *pStaticRegs);


#ifdef __cplusplus
}
#endif

#endif /* SDL_RTI_H_ */
/** @} */
