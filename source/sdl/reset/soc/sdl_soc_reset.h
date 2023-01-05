 /*
 *  Copyright (c) Texas Instruments Incorporated 2023
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

#ifndef INCLUDE_SDL_RESET_SOC_H_
#define INCLUDE_SDL_RESET_SOC_H_

#ifdef __cplusplus
extern "C" {
#endif

#if defined (SOC_AWR294X)
#include <sdl/include/awr294x/sdlr_mss_toprcm.h>
#include <sdl/include/awr294x/sdlr_dss_rcm.h>
#include <sdl/include/awr294x/sdlr_mss_rcm.h>
#include <sdl/include/awr294x/sdlr_soc_baseaddress.h>


#endif /* SOC_AWR294X */

#if defined (SOC_AM273X)
#include <sdl/include/am273x/sdlr_mss_toprcm.h>
#include <sdl/include/am273x/sdlr_dss_rcm.h>
#include <sdl/include/am273x/sdlr_mss_rcm.h>
#include <sdl/include/am273x/sdlr_soc_baseaddress.h>

#endif /* SOC_AM273X */


#ifdef __cplusplus
}
#endif  /* extern "C" */

#endif /* INCLUDE_SDL_RESET_SOC_H_ */
