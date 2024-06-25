/*
 * SDL ECC
 *
 * Software Diagnostics Library module for ECC
 *
 *  Copyright (c) Texas Instruments Incorporated 2018-2020
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

#include <string.h>
#include <sdl/include/sdl_types.h>
#include <sdl/sdl_ecc.h>
#if defined(SOC_AM263X)
#include <sdl/include/am263x/sdlr_soc_ecc_aggr.h>
#endif
#if defined(SOC_AM263PX)
#include <sdl/include/am263px/sdlr_soc_ecc_aggr.h>
#endif
#if defined(SOC_AM261X)
#include <sdl/include/am261x/sdlr_soc_ecc_aggr.h>
#endif
#if defined(SOC_AM273X)
#include <sdl/include/am273x/sdlr_soc_ecc_aggr.h>
#endif
#if defined(SOC_AWR294X)
#include <sdl/include/awr294x/sdlr_soc_ecc_aggr.h>
#endif
#if defined(SOC_AM64X) || defined(SOC_AM243X)
#include <sdl/include/am64x_am243x/sdlr_soc_ecc_aggr.h>
#endif
#include "sdl_r5_utils.h"
#include <sdl/ecc/sdl_ecc_utils.h>
#include <sdl/ecc/sdl_ecc_priv.h>
#include <sdl/ecc/sdl_ecc_core.h>

/* Local defines */
#define SDL_ECC_R5_CFLR_ATCM_DATA_ERROR_MASK (0x1000001u)
#define SDL_ECC_R5_CFLR_BTCM_DATA_ERROR_MASK (0x2000001u)

/*********************************************************************
 *
 * /brief   Configure ECC for given ram Id for the CPU
 *
 * /param  ramId: Ram Id of memory section
 *
 * /return  SDL_PASS : Success; SDL_EFAIL for failures
 */
int32_t SDL_ECC_configECCRam(uint32_t ramId)
{
    int32_t retVal= SDL_PASS;

    switch(ramId) {

        case SDL_ECC_R5F_MEM_SUBTYPE_ATCM0_BANK0_VECTOR_ID:
        case SDL_ECC_R5F_MEM_SUBTYPE_ATCM0_BANK1_VECTOR_ID:
            /* Enable ECC for ATCM */
            SDL_ECC_UTILS_enableECCATCM();

            break;

        case SDL_ECC_R5F_MEM_SUBTYPE_B0TCM0_BANK0_VECTOR_ID:
        case SDL_ECC_R5F_MEM_SUBTYPE_B0TCM0_BANK1_VECTOR_ID:
            /* Enable ECC for B0TCM */
            SDL_ECC_UTILS_enableECCB0TCM();

            break;

        case SDL_ECC_R5F_MEM_SUBTYPE_B1TCM0_BANK0_VECTOR_ID:
        case SDL_ECC_R5F_MEM_SUBTYPE_B1TCM0_BANK1_VECTOR_ID:
            /* Enable ECC for B1TCM */
            SDL_ECC_UTILS_enableECCB1TCM();

            break;

        default:
            if ( ramId > SDL_ECC_R5F_MEM_SUBTYPE_KS_VIM_RAM_VECTOR_ID) {
                retVal = SDL_EFAIL;
            }
            break;
    }
    return retVal;

}

/*********************************************************************
 *
 * /brief   Poll ecc event
 *
 * \param  eccMemType: Memory type
 * \param  memSubType: Memory subtype
 * \param  errorType:  Error type to poll
 *
 * \return SDL_ECC_EVENT_FOUND or if no events found
 */
uint32_t SDL_ECC_pollErrorEvent(SDL_ECC_MemType eccMemType,
                               SDL_ECC_MemSubType memSubType,
                               SDL_ECC_InjectErrorType errorType)
{
    uint32_t retValue = 0u;
    uint32_t regValue;
    /* Polling only for R5F core self test */
#if defined(SOC_AM273X) || defined(SOC_AWR294X) || defined(SOC_AM263X) || defined(SOC_AM263PX) || defined(SOC_AM261X)
    if ((eccMemType == SDL_R5FSS0_CORE0_ECC_AGGR)
        || (eccMemType == SDL_R5FSS0_CORE1_ECC_AGGR)) {
        switch(errorType) {
            case SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE:
                /* Only for single bit error do polling */
                switch (memSubType) {
                    case SDL_ECC_R5F_MEM_SUBTYPE_ATCM0_BANK0_VECTOR_ID:
                    case SDL_ECC_R5F_MEM_SUBTYPE_ATCM0_BANK1_VECTOR_ID:
                         /* Check CFLR register */
                         regValue = SDL_UTILS_getCFLR();
                         if((regValue & SDL_ECC_R5_CFLR_ATCM_DATA_ERROR_MASK)
                             == SDL_ECC_R5_CFLR_ATCM_DATA_ERROR_MASK) {
                            retValue = (uint32_t)(SDL_ECC_EVENT_FOUND);
                         }
                        break;

                    case SDL_ECC_R5F_MEM_SUBTYPE_B0TCM0_BANK0_VECTOR_ID:
                    case SDL_ECC_R5F_MEM_SUBTYPE_B0TCM0_BANK1_VECTOR_ID:
                    case SDL_ECC_R5F_MEM_SUBTYPE_B1TCM0_BANK0_VECTOR_ID:
                    case SDL_ECC_R5F_MEM_SUBTYPE_B1TCM0_BANK1_VECTOR_ID:
                           /* Check CFLR register */
                            regValue = SDL_UTILS_getCFLR();
                            if( (regValue & SDL_ECC_R5_CFLR_BTCM_DATA_ERROR_MASK)
                                == SDL_ECC_R5_CFLR_BTCM_DATA_ERROR_MASK) {
                                retValue = (uint32_t)(SDL_ECC_EVENT_FOUND);
                            }

                        break;

                    default:

                        break;
                }
                break;

            default:
                break;
        }
    }
#endif

#if defined(SOC_AM64X) || defined(SOC_AM243X)
    if ((eccMemType == SDL_R5FSS1_PULSAR_LITE_CPU0_ECC_AGGR) || (eccMemType == SDL_R5FSS0_PULSAR_LITE_CPU0_ECC_AGGR)) {
        switch(errorType) {
            case SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE:
                /* Only for single bit error do polling */
                switch (memSubType) {
                    case SDL_ECC_R5F_MEM_SUBTYPE_ATCM0_BANK0_VECTOR_ID:
                    case SDL_ECC_R5F_MEM_SUBTYPE_ATCM0_BANK1_VECTOR_ID:
                         /* Check CFLR register */
                         regValue = SDL_UTILS_getCFLR();
                         if((regValue & SDL_ECC_R5_CFLR_ATCM_DATA_ERROR_MASK)
                             == SDL_ECC_R5_CFLR_ATCM_DATA_ERROR_MASK) {
                            retValue = (uint32_t)(SDL_ECC_EVENT_FOUND);
                         }
                        break;

                    case SDL_ECC_R5F_MEM_SUBTYPE_B0TCM0_BANK0_VECTOR_ID:
                    case SDL_ECC_R5F_MEM_SUBTYPE_B0TCM0_BANK1_VECTOR_ID:
                    case SDL_ECC_R5F_MEM_SUBTYPE_B1TCM0_BANK0_VECTOR_ID:
                    case SDL_ECC_R5F_MEM_SUBTYPE_B1TCM0_BANK1_VECTOR_ID:
                           /* Check CFLR register */
                            regValue = SDL_UTILS_getCFLR();
                            if( (regValue & SDL_ECC_R5_CFLR_BTCM_DATA_ERROR_MASK)
                                == SDL_ECC_R5_CFLR_BTCM_DATA_ERROR_MASK) {
                                retValue = (uint32_t)(SDL_ECC_EVENT_FOUND);
                            }

                        break;

                    default:

                        break;
                }
                break;

            default:
                break;
        }
    }
#endif
    return retValue;
}
