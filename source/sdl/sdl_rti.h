/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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
 */

/**
 *  \defgroup SDL_RTI_API APIs for SDL RTI
 *  \ingroup SDL_MODULE
 *
 * The Digital Watchdog Timer(DWT) generates reset after a programmable period,
 * if not serviced within that period. In DWT, time-out boundary is configurable.
 * In DWWD, along with configurable time-out boundary, the start time boundary is also configurable.
 * The DWWD can generate Reset or Interrupt, if not serviced within window(Open Window) defined by start time and time-out boundary.
 * Also the DWWD can generate Reset or Interrupt if serviced outside Open Window (within Closed Window). Generation * of Reset or Interrupt depends on the DWWD Reaction configuration.
 *
 *  @{
 */
/** @} */

#ifndef SDL_RTI_TOP_H_
#define SDL_RTI_TOP_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <sdl/include/soc_config.h>

#if defined (IP_VERSION_RTI_V0)
#include <sdl/rti/v0/sdl_rti.h>
#include <sdl/rti/v0/sdl_ip_rti.h>
#include <sdl/include/sdl_types.h>
#include <sdl/include/hw_types.h>
#include <sdl/rti/v0/soc/sdl_rti_soc.h>
#endif

#ifdef __cplusplus
}
#endif

#endif
