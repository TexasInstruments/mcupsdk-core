/*  ============================================================================
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

/**
 *   \file  sdl_types.h
 *
 *   \brief  This file contains the Register Desciptions for SDL types
 */

#ifndef SDL_TYPES_H
#define SDL_TYPES_H

#include <stdbool.h>
#include <stdint.h>

/**
 *  \anchor SDL_Type_t
 *  \name SDL Types
 *
 *  SDL data types.
 *
 *  @{
 */
#ifndef TRUE
#define TRUE		((bool) 1)
#define FALSE		((bool) 0)
#endif

/* @} */

/** \brief SDL error type */
typedef int32_t SDL_ErrType_t;

/**
 *  \anchor SDL_ErrType_t
 *  \name SDL Error Types
 *
 *  SDL function return error codes.
 *
 *  @{
 */
#define SDL_PASS                        ( (int32_t) (0))
#define SDL_EFAIL                       (-(int32_t) (1))
#define SDL_EBADARGS                    (-(int32_t) (2))
#define SDL_EINVALID_PARAMS             (-(int32_t) (3))
#define SDL_ETIMEOUT                    (-(int32_t) (4))
#define SDL_EOUT_OF_RANGE               (-(int32_t) (5))
#define SDL_EUNSUPPORTED_CMD            (-(int32_t) (6))
#define SDL_EUNSUPPORTED_OPS            (-(int32_t) (7))
#define SDL_EALLOC                      (-(int32_t) (8))
/* @} */

/** \brief Define NULL if not defined */
#ifndef NULL
#define NULL            ((void*)0)
#endif

#ifndef NULL_PTR
#if defined (HOST_EMULATION)
/* Host emulation compilation throws below error when void * is used. So
 * retain as normal 0 comparision.
 *
 * error: invalid conversion from 'void*' to 'xxx *' [-fpermissive]
 */
#define NULL_PTR (NULL)
#else
#define NULL_PTR ((void *)0)
#endif
#endif

#endif /* SDL_TYPES_H */

