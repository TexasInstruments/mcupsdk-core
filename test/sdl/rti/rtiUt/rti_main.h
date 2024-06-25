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
 *  \file     rti_main.h
 *
 *  \brief    This file contains rti test code defines.
 *
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
/*===========================================================================*/
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include <dpl_interface.h>
#include <sdl/sdl_rti.h>
#if defined(SOC_AM64X) || defined (SOC_AM243X)
#include <drivers/sciclient.h>
#endif
#include <drivers/soc.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#if defined(SOC_AM263X) || defined(SOC_AM263PX) || defined (SOC_AM261X)
#define SDL_INSTANCE_RTI SDL_INSTANCE_WDT0
#endif
#if defined(R5F_INPUTS)
#define SDL_INSTANCE_RTI SDL_INSTANCE_MSS_WDT
#endif
#if defined(C66_INPUTS)
#define SDL_INSTANCE_RTI SDL_INSTANCE_DSS_WDT
#endif
#if defined (SOC_AM64X) || defined (SOC_AM243X)
#define SDL_INSTANCE_RTI SDL_INSTANCE_MCU_RTI0_CFG
#endif

#if !defined(RTI_MAIN_H)
#define RTI_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif



/*===========================================================================*/
/*                         Declarations                                      */
/*===========================================================================*/

/* Define the test interface */
typedef struct sdlrtiTest_s
{
    int32_t  (*testFunction)(void);   /* The code that runs the test */
    char      *name;                  /* The test name */
    int32_t    testStatus;            /* Test Status */
} sdlrtiTest_t;

/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/
#define SDL_APP_TEST_NOT_RUN        (-(int32_t) (2))
#define SDL_APP_TEST_FAILED         (-(int32_t) (1))
#define SDL_APP_TEST_PASS           ( (int32_t) (0))

#define SOC_MODULES_END     (0xFFFFFFFFu)
/*===========================================================================*/
/*                    Dependant macros in sdl_rti_negTest.c                  */
/*===========================================================================*/

#define  SDL_RTI_DWWDRXN_INVLD 					(0x1U)
#define  SDL_RTI_DWWDPRLD_INVLD 				(0x1U)
#define  SDL_RTI_DWWDPRLD_VLD 		    		(0x0009FFF)
#define  INSTANCE_VLD                       	(SDL_INSTANCE_WDT0)
#define  SDL_RTI_WINSZ_INVLD 		      		(RTI_DWWD_WINDOWSIZE_3_125_PERCENT << 1)
#define  STATUS_INVLD							(0x46U)

/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/


/*===========================================================================*/
/*                         External function declarations                    */
/*===========================================================================*/
extern int32_t SDL_RTI_posTest(void);
extern int32_t SDL_RTI_negTest(void);

/*===========================================================================*/
/*                   Local Function definitions                              */
/*===========================================================================*/



#ifdef __cplusplus
}

#endif /*extern "C" */

#endif /* RTI_MAIN_H */
/* Nothing past this point */
