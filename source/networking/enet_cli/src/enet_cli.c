/*
 * Copyright (C) 2024 Texas Instruments Incorporated
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

/*!
 * \file  enet_cli.c
 *
 * \brief This file contains the function definitions for enet_cli library.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "enet_cli.h"
#include "enet_cli_mod.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

EnetCli_Obj EnetCli_inst;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetCli_init(Enet_Type enetType, uint32_t instId)
{
    /* Retrieve board info and enet handle */
    memset(&EnetCli_inst, 0, sizeof(EnetCli_Obj));
    EnetCli_inst.enetType = enetType;
    EnetCli_inst.instId = instId;
    EnetCli_inst.coreId = Enet_getCoreId();
    EnetCli_inst.hEnet = Enet_getHandle(enetType, instId);
    EnetCli_inst.numMacPorts = Enet_getMacPortMax(enetType, instId);

    EnetAppUtils_assert(EnetCli_inst.hEnet != NULL);
    EnetAppUtils_assert(EnetCli_inst.enetType == ENET_CPSW_3G);
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

/* None */
