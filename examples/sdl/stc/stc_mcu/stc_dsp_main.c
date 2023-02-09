/* Copyright (c) 2022-2023 Texas Instruments Incorporated
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
 *  \brief    This file contains STC example code.
 *
 *  \details  STC app
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include <dpl_interface.h>
#include <sdl/include/sdl_types.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <sdl/sdl_stc.h>

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"


/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/
#define DSS_DSP_PDC_INT   118U

#define SHARED_ADDRESS           (0xC02E8000)
#define COMMON_VARIABLE_MASK      (0x0000FFFF)
#define COMMON_VARIABLE_SHIFT      (0U)

/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/
 void ISR (void *args );


/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
/**
 *  \brief global variable for holding data buffer.
 */


/*===========================================================================*/
/*                         Function definitions                              */
/*===========================================================================*/

volatile uint32_t Done_flag=0;

 void ISR (void *args )
 {
    Done_flag=1;
    /* Configure this resister to go DSP Core in low power mode during idle state */
    HW_WR_FIELD32(DSS_DSP_ICFG_PDCCMD, SDL_DSS_DSP_ICFG_PDCCMD_GEMPD,
        1U);
    asm(" idle");

 }

void STC_main(void *args)
{
    Drivers_open();
    Board_driversOpen();

    HwiP_Params stc_Params;
    HwiP_Object stcHwiObj;
    uint32_t intNum =DSS_DSP_PDC_INT;

    HwiP_Params_init(&stc_Params);
    stc_Params.intNum = intNum;
    stc_Params.callback =&ISR;
    stc_Params.isPulse = 0U;
    stc_Params.priority =1U;
    HwiP_construct(&stcHwiObj, &stc_Params);
    HwiP_enableInt(intNum);
    DebugP_log("DSP core is done with interrupt register.\r\n");
    /* This is shared memory register, which is being used for
    confirmatoion for intrrupt Registartion is complete in DSP core */
    HW_WR_FIELD32(SHARED_ADDRESS, COMMON_VARIABLE, 0x1111);

    while(1)
    {
        if(Done_flag==1)
        {
            DebugP_log("DSP core is in low power mode now.\r\n");
            break;
        }
    }

    Board_driversClose();
    Drivers_close();
}

/* Nothing past this point */
