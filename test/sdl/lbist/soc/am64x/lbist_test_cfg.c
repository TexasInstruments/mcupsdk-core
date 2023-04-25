/*
 * Copyright (C) 2023 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

 /**
 *  \file     lbist_test_cfg.c
 *
 *  \brief    This file contains LBIST test configuration
 *
 *  \details  LBIST Test Configuration
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <string.h>
#include "lbist_test_cfg.h"
#include <sdl/include/am64x_am243x/sdlr_soc_baseaddress.h>
#include <sdl/include/am64x_am243x/sdlr_mcu_ctrl_mmr.h>
#include <drivers/sciclient/include/am64x_am243x/sciclient_fmwMsgParams.h>


/* #define DEBUG */

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
/* Lbist Parameters */
#define LBIST_DC_DEF         (0x3u)
#define LBIST_DIVIDE_RATIO   (0x01u)
#define LBIST_RESET_PC_DEF   (0x0fu)
#define LBIST_SET_PC_DEF     (0x00u)
#define LBIST_SCAN_PC_DEF    (0x08u)
#define LBIST_PRPG_DEF       (0x1fffffffffffffu)

/* PC Definitions */
#define LBIST_M4F_STATIC_PC_DEF        (0x1000)
/*
* LBIST expected MISR's (using parameters above)
*/
#define M4F_MISR_EXP_VAL           (0xB89963D9)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

LBIST_TestHandle_t LBIST_TestHandleArray[LBIST_MAX_CORE_INDEX+1] =
{
 /* M4F */
 {
  .coreName               = "M4F",
  .instance               = LBIST_MCU_M4F,
  .secondaryCoreNeeded    = false,            /* Secondary core needed */
  .wfiCheckNeeded         = false,            /* wfi check needed */
  .cpuStatusFlagMask      = 0x00000002U, /* Expected boot status value for wfi */
  .tisciProcId            = SCICLIENT_PROCID_MCU_M4FSS0_C0, /* M4F Proc Id */
  .tisciDeviceId          = TISCI_DEV_MCU_M4FSS0_CORE0,     /* M4F Device Id */
  .numAuxDevices          = 0u,                       /* No Aux devices */
 },

};

LBIST_TestHandle_t* LBIST_getTestHandleArray(void)
{
    return LBIST_TestHandleArray;
}

/* Nothing past this point */
