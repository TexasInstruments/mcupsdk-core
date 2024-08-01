/*
 *   Copyright (c) Texas Instruments Incorporated 2022-24
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
 *  \file    main.h
 *
 *  \brief    This file contains CPU main test defines.
 *
 *
 **/
#ifndef CCM_TEST_MAIN_H
#define CCM_TEST_MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <string.h>
#include <sdl/include/sdl_types.h>
#include <sdl/r5/v0/sdl_ip_ccm.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/include/hw_types.h>
#include <sdl/sdl_ecc.h>
#include <sdl/sdl_ccm.h>
#include <dpl_interface.h>
/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
#define  CCM_FUNC_TEST_ID         (0U)


#define SDL_INTR_PRIORITY_LVL      1U
#define SDL_ENABLE_ERR_PIN         1U

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
#define SDL_INTR_GROUP_NUM      1U
#define ESM_INSTANCE 			SDL_ESM_INST_MAIN_ESM0
#define  CCM_NUM_INSTANCE       (2U)
#elif defined (SOC_AM273X) || defined (SOC_AWR294X)
#define SDL_INTR_GROUP_NUM      1U
#define ESM_INSTANCE 			SDL_ESM_INST_MSS_ESM
#define SDL_ESM_MAX_EVENT_MAP   1U
#define  CCM_NUM_INSTANCE       (1U)
#endif
/* ========================================================================== */
/*                 External Function Declarations                             */
/* ========================================================================== */
extern int32_t CCM_funcTest(void);

#ifdef __cplusplus
}
#endif

#endif /* CCM_TEST_MAIN_H */

/* Nothing past this point */
