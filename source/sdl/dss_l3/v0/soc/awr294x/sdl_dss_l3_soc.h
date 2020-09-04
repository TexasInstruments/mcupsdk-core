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

/**
 *  @addtogroup SDL_DSS_L3_API DSS_L3 API
    @{
 */

#ifndef SDL_DSS_L3_SOC_H_
#define SDL_DSS_L3_SOC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <sdl/include/awr294x/sdlr_soc_baseaddress.h>


#ifdef _cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */
#define SDL_DSS_L3_CTRL                   SDL_DSS_CTRL_U_BASE
#define SDL_DSS_L3_BANKA_ADDRESS          SDL_DSS_L3_U_BASE

#if defined (SOC_AWR294X)
#define SDL_DSS_L3_BANKB_ADDRESS          SDL_DSS_L3_U_BASE+0xC0000U
#define SDL_DSS_L3_BANKC_ADDRESS          SDL_DSS_L3_U_BASE+0x180000U
#define SDL_DSS_L3_BANKD_ADDRESS          SDL_DSS_L3_U_BASE+0x200000U
#endif
/**
 * \brief This enumerator defines the HWA IDs
 *
 */
typedef enum {
    SDL_DSS_L3_BANKA_MEM_ID = 0,
    /**< DSS L3 Bank A */
    SDL_DSS_L3_BANKB_MEM_ID = 1,
    /**< DSS L3 Bank B */
    SDL_DSS_L3_BANKC_MEM_ID = 2,
    /**< DSS L3 Bank C */
    SDL_DSS_L3_BANKD_MEM_ID = 3,
    /**< DSS L3 Bank D */
    SDL_DSS_L3_INVALID_MEM_ID = 4,
    /**< Invalid   */
} SDL_DSS_L3_MemID;


/* ========================================================================== */
/*                         Function Declarations                              */
/* ========================================================================== */

#ifdef _cplusplus
}

#endif /*extern "C" */

#endif
 /** @} */
