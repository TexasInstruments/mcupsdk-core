/* Copyright (c) 2023 Texas Instruments Incorporated
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
 *  \file     main.c
 *
 *  \brief    This file contains RESET example code.
 *
 *  \details  RESET app
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/

#include <sdl/include/sdl_types.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <sdl/sdl_reset.h>


#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */
/* Define the application interface */


/**
 *  \brief RESET configuration parameter structure.
 */


/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/

# define WARM_RESET      (1U)

/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/



/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/

static  int32_t test_Result;
static  int32_t r5f_Reset_Cause;

/*===========================================================================*/
/*                         Function definitions                              */
/*===========================================================================*/


void reset_func_test_main(void *args)
{
    Drivers_open();
    Board_driversOpen();

    bool SW_Warm_Reset_Flag =0;
    bool POR_Reset_Flag =0;

    /* get warm reset cause*/
    test_Result=  SDL_getWarmResetCause();

    switch(test_Result)
    {
        case SDL_WarmResetCause_POWER_ON_RESET:
        {
            DebugP_log("\r\nPower on Reset is happened.\r\n");
            POR_Reset_Flag=1;
            break;
        }
        case SDL_WarmResetCause_MSS_WDT:
        {
            DebugP_log("\r\nDue to MSS WDT Reset is happened.\r\n");
            break;
        }
        case SDL_WarmResetCause_TOP_RCM_WARM_RESET_CONFIG:
        {
            DebugP_log("\r\nDue to SW, Warm Reset is happened.\r\n");
            SW_Warm_Reset_Flag=1;
            break;
        }
         case SDL_WarmResetCause_EXT_PAD_RESET:
        {
            DebugP_log("\r\nDue to EXT PAD, Reset is happened.\r\n");
            break;
        }
        case SDL_WarmResetCause_HSM_WDT:
        {
            DebugP_log("\r\nDue to HSM WDT, Warm Reset is happened.\r\n");
            break;
        }
        default :
        {
            DebugP_log("\r\nNot able to check, Reset cause is clear Already.\r\n");
            break;
        }
    }

    /* get R5F core reset cause*/
    r5f_Reset_Cause = SDL_r5fGetResetCause();

    if(r5f_Reset_Cause == WARM_RESET)
    {

       if(POR_Reset_Flag==1)
       {
        DebugP_log("Foe R5F core, Reset cause is Warm Reset asserted by Power on Reset.\r\n");
       }
       else
       {
        DebugP_log("Foe R5F core, Reset cause is Warm Reset asserted by Software.\r\n");
       }
    }
    else
    {
        DebugP_log("Foe R5F core, Reset cause is not the Warm reset.\r\n");
    }


    if(SW_Warm_Reset_Flag==0)
    {
        DebugP_log("SW is asserting Warm reset.....\r\n");
        /* WARM Reset assert by SW, */
        SDL_generateSwWarmReset();
    }

    /* Assert Local Reset for DSP Core */
    SDL_rcmDspLocalReset();
    DebugP_log("DSP local reset is asserted.\r\n");

    if(SW_Warm_Reset_Flag==1)
    {
     DebugP_log("All test have Passed.\r\n");
    }
    else
    {
     DebugP_log("All/few test are failing.\r\n");
    }

    Board_driversClose();
    Drivers_close();

}

/* Nothing past this point */
