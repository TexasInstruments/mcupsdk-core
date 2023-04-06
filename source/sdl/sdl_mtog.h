/*
 *   Copyright (c) Texas Instruments Incorporated 2022-2023
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
/**
 *  \defgroup SDL_MTOG_MODULE APIs for SDL MTOG
 *  \ingroup SDL_MODULE
 *
 *  Header file containing various enumerations, structure definitions and function
 *  declarations for the VBUSM Slave Timeout Gasket IP.
 *
 *  This is the SDL-FL API documentation for the VBUSM Master Timeout Gasket module.
 *
 *  The following procedure describes how to properly use this SDL-FL API:
 *
 *  1. Call #SDL_MTOG_init to configure the desired timeout
 *     counter value
 *  2. Call #SDL_MTOG_start to start the timeout functionality
 *
 *  Whenever a read return or write status return is pending without being
 *  accepted, an internal timer increments by 1. Whenever the counter value
 *  meets or exceeds the programmed value, the gasket is in a timed out state
 *  and the following actions are taken:
 *  - timed_out_intr is set
 *  - All requests from the master are blocked
 *  - All readies from the master are set high
 *  - Once all transactions are returned, the gasket indicates idle
 *
 *  Following servicing of the timeout interrupt, software should...
 *
 *  3. Call #SDL_MTOG_reset to reset the timeout interrupt and timeout counter
 *  4. Call #SDL_MTOG_start to start the timeout functionality
 *
 * @{
 */
/** @} */
#ifndef SDL_MTOG_H_
#define SDL_MTOG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include <sdl/mtog/v0/sdl_ip_mtog.h>
#include <sdl/mtog/soc/sdl_soc_mtog.h>

/* Magic value used to force a timeout */
#define SDL_MTOG_FORCE_KEY       ((uint32_t) 0x95U)

/** \brief Master TOG Static Registers
 *
 *  This structure contains Master TOG static registers
 *  The register values are not expected to change until a new configuration
 *  is done.
 *
 */
typedef struct {
    /** Master timeout gasket control register */
    uint32_t  mtogCtrlRegister;
} SDL_MTOG_staticRegs;

/* This structure contains Master TOG configuration value */
typedef struct {
    /** Timeout values */
    SDL_MTOGVal timeOut;
} SDL_MTOG_config;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  @ingroup SDL_MTOG_MODULE
 *  @defgroup SDL_MTOG_API MTOG API
 *  @section MTOG Functions
 *  @{
 */
/**
 *  \brief Initialize Master TOG module with respect of instance
 *
 *  This function allows software to initialize MTOG module
 *
 *  The pConfig contains the timeout value
 *
 *  \param instance       [IN]    MTOG instance
 *  \param pConfig        [IN]    Pointer to store the configuration
 *
 *  \return SDL_PASS     The function completed successfully
 *          SDL_EBADARGS instance is invalid or pConfig is NULL
 */
int32_t SDL_MTOG_init(SDL_MTOG_Inst instance, const SDL_MTOG_config *pConfig);

/**
 *  \brief Verifying written configuration with Master TOG module
 *
 *  This function allows software to verifying MTOG module
 *
 *  The pConfig contains the timeout value
 *
 *  \param instance       [IN]    MTOG instance
 *  \param pConfig        [IN]    Pointer to store the configuration
 *
 *  \return SDL_PASS      The function completed successfully
 *          SDL_EBADARGS  instance is invalid or pConfig is NULL
 */
int32_t SDL_MTOG_verifyConfig(SDL_MTOG_Inst instance, const SDL_MTOG_config	*pConfig);

/**
 *  \brief Start the timeout function
 *
 *  This function starts the timeout counter functionality.
 *
 *  \param instance       [IN]    MTOG instance
 *
 *  \return SDL_PASS      Function completed successfully
 *          SDL_EBADARGS  pRegs is NULL
 */
int32_t SDL_MTOG_start( SDL_MTOG_Inst instance );

/**
 *  \brief Stop the timeout function
 *
 *  This function stops the timeout counter functionality.
 *
 *  \param instance       [IN]    MTOG instance
 *
 *  \return SDL_PASS      Function completed successfully
 *          SDL_EBADARGS  pRegs is NULL
 */
int32_t SDL_MTOG_stop( SDL_MTOG_Inst instance );

/**
 *  \brief Force a timeout
 *
 *  This function enables software to force a timeout and flush of
 *  the interface transactions.
 *
 *  \param instance       [IN]    MTOG instance
 *
 *  \return SDL_PASS      Function completed successfully
 *          SDL_EFAIL     Timeout function is disabled. Call
 *                        #SDL_MTOG_start first.
 *          SDL_EBADARGS  pRegs is NULL
 */
int32_t SDL_MTOG_forceTimeout( SDL_MTOG_Inst instance );

/**
 *  \brief Reset the timeout functionality
 *
 *  This function resets the timeout functionality by clearing the timeout
 *  interrupt and timeout counter. It should be called after servicing a
 *  timeout interrupt and before re-enabling the timeout counter via the
 *  #SDL_MTOG_start function.
 *
 *  Note that the timeout value set via the #SDL_MTOG_setTimeoutVal
 *  function is unaffected.
 *
 *  \param instance       [IN]    MTOG instance
 *
 *  \return SDL_PASS      Function completed successfully
 *          SDL_EBADARGS  pRegs is NULL
 */
int32_t SDL_MTOG_reset( SDL_MTOG_Inst instance );

/**
 *  \brief Readback Static configuration register
 *
 *  This function reads back configuration registers that are static.
 *  Note that this is just one register master TOG
 *
 *  \param instance       [IN]    MTOG instance
 *
 *  \param pStaticRegs    [OUT]    Pointer to store the read static registers
 *
 *  \return SDL_PASS      Function completed successfully
 *          SDL_EBADARGS  pRegs is NULL
 */
int32_t SDL_MTOG_getStaticRegisters( SDL_MTOG_Inst instance,
                                          SDL_MTOG_staticRegs *pStaticRegs);


#ifdef __cplusplus
}
#endif

#endif	/* SDL_MTOG_H_ */

/* nothing past this point */
/** @} */
