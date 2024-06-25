/**
 * @file  sdl_common.h
 *
 * @brief
 *  Header file contains enumerations, structure definitions and function
 *  declarations for SDL COMMON interface.
 *
 *  The SDL common enumerations include:
 *      1. SDL API function return result value
 *      2. SDL error code asserted
 *
 *  The SDL common APIs include:
 *      1. Application provided external assert function
 *      2. Application provided get timestamp function
 *
 *  Software Diagnostics Library common module which defines common data
 *
 *  Copyright (c) Texas Instruments Incorporated 2022
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

#ifndef INCLUDE_SDL_COMMON_H_
#define INCLUDE_SDL_COMMON_H_
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Software Diagnostics Library
 *
 *
 * Software Diagnostics Library(SDL) provides software interface for
 * Diagnostic functions to be used for application.
 *
 * <b>
 * Also refer to top level user guide for detailed features,
 * limitations and usage description.
 * </b>
 *
 */

/** ---------------------------------------------------------------------------
 * \enum SDL_assertErrorNumber
 * \brief Defines the different Assert error codes
 */

typedef enum {
    SDL_ESM_INT_SRC_OUT_OF_BOUNDS=1,
    /**< Assert when ESM Interrupt source out of bounds */
    SDL_ECC_INTERRUPT_WITH_NOEVENT=2,
    /**< Assert when ECC interrupt triggered with no event */
    SDL_ECC_RAM_ID_NOT_FOUND=3,
    /**< Assert when ECC Ram Id not found */
    SDL_CCM_READ_REG_FAILURE=4,
    /**< Assert CCM Read Register failure */
    SDL_CCM_INTERRUPT_WITHOUT_ANY_ERROR=5,
    /**< Assert CCM interupt received wihtout any error */
    SDL_CCM_CONFIG_REG_FAILURE=6,
    /**< Assert CCM Config Register failure */
    SDL_CCM_INVALID_INTERRUPT_SOURCE=7,
    /**< Assert CCM Invalid interrupt source */
} SDL_assertErrorNumber;

/** ============================================================================
 *
 * \brief   Application provided external assert function
 *          Called inside the reference functions when unexpected conditions
 *          are encountered.
 *          NOTE: This is application supplied and not part of the SDL
 *          If not supplied by application this will result in an linker error
 *          NOTE: This function is not expected to return as the unexpected
 *          condition encountered will cause indeterminate behaviour if it
 *          returns. Application will need to add code here which will bring the system
 *          to a safe state as required by the application. But the function
 *          should not return.
 *
 * \param  errorNumber: Error number for the assert
 */
void SDL_assertExternalFunction(SDL_assertErrorNumber errorNumber);

/** ============================================================================
 *
 * \brief   Application provided timestamp function
 *          This function will be called inside the reference function to get
 *          timestamp.
 *          NOTE: This is application supplied and not part of the SDL
 *          If not supplied by application this will result in an linker error
 *
 * \return  Timestamp value
 */
uint32_t SDL_getTime(void);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_SDL_COMMON_H_ */

