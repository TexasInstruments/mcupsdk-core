/* Copyright (c) 2021 Texas Instruments Incorporated
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
 *  \file     mcrc_main.h
 *
 *  \brief    This file contains mcrc test code defines.
 *
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
/*===========================================================================*/
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <sdl/include/sdl_types.h>
#include <sdl/include/hw_types.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/sdl_mcrc.h>


#if !defined(MCRC_MAIN_H)
#define MCRC_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif



/*===========================================================================*/
/*                         Declarations                                      */
/*===========================================================================*/

/* Define the test interface */
typedef struct sdlmcrcTest_s
{
    int32_t  (*testFunction)(void);   /* The code that runs the test */
    char      *name;                  /* The test name */
    int32_t    testStatus;            /* Test Status */
} sdlmcrcTest_t;

/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/
#if defined (R5F_INPUTS)
#define MCRC_INSTANCE  MSS_MCRC
#endif
#if defined (C66_INPUTS)
#define MCRC_INSTANCE  DSS_MCRC
#endif
#define SDL_APP_TEST_NOT_RUN        (-(int32_t) (2))
#define SDL_APP_TEST_FAILED         (-(int32_t) (1))
#define SDL_APP_TEST_PASS           ( (int32_t) (0))

#define MCRC_WATCHDOG_PRELOAD        ((uint32_t) 0U)
#define MCRC_BLOCK_PRELOAD           ((uint32_t) 0U)

/* SDTF CRC local defines */
#define SDL_MCRC_REF_SIGN_HIGH    (0xF2C2E9EEU)  /* Reference high 32-bit CRC signautre value */
#define SDL_MCRC_REF_SIGN_LOW     (0xEBEB19C1U)  /* Reference low 32-bit CRC signautre value */
#define SDL_MCRC_DATA_SIZE        (20000U)

/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/


/*===========================================================================*/
/*                         External function declarations                    */
/*===========================================================================*/
extern int32_t sdl_mcrc_posTest(void);
extern int32_t sdl_mcrc_negTest(void);
extern int32_t sdl_ip_mcrcNegTest(void);
extern int32_t sdl_ip_mcrcPosTest(void);

/*===========================================================================*/
/*                   Local Function definitions                              */
/*===========================================================================*/
/* MCRC Self Test data */
static uint8_t SDL_mcrcTestData[SDL_MCRC_DATA_SIZE] __attribute__ ((aligned(128))) __attribute__((section(".bss:extMemCache:ramdisk")));

#ifdef __cplusplus
}

#endif /*extern "C" */

#endif /* MCRC_MAIN_H */
/* Nothing past this point */
