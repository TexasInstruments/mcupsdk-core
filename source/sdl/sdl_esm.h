/********************************************************************
 * Copyright (C) 2022-2024 Texas Instruments Incorporated.
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
 *  \defgroup SDL_ESM_MODULE APIs for SDL ESM
 *  \ingroup SDL_MODULE
 *
 *  This module contains APIs for using the ESM module. The APIs can be
 *  used to configure the ESM instances for notification when error events
 *  occur and also to set the error pin.
 *
 *  @{
 */
/**
 *   \file  sdl_esm.h
 *
 *   \brief This file contains the SDL ESM API's.
 */

#ifndef SDL_ESM_H_
#define SDL_ESM_H_

#ifdef __cplusplus
extern "C" {
#endif
#include <sdl/include/soc_config.h>

#if defined (IP_VERSION_ESM_V0)
#include <sdl/esm/v0/v0_0/sdl_ip_esm.h>
#include <sdl/esm/v0/sdl_esm.h>
#include <sdl/esm/v0/v0_0/sdl_esm_priv.h>
#endif

#if defined (IP_VERSION_ESM_V2_0)
#include <sdl/esm/v2/v2_0/sdl_ip_esm.h>
#include <sdl/esm/v2/sdl_esm.h>
#include <sdl/esm/v2/v2_0/sdl_esm_priv.h>
#endif

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* SDL_ESM_H_ */