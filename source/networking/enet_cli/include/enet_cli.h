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
 * \file  enet_cli.h
 *
 * \brief Header file for enet_cli library.
 */

#ifndef _ENET_CLI_H_
#define _ENET_CLI_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* Standard Libraries */
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <kernel/dpl/TaskP.h>

/* Networking Libraries */
#include <enet.h>
#include <enet_apputils.h>

/* FreeRTOS Libraries */
#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define ENET_CLI_MIN_WRITE_BUF_LEN 150
#define ENET_CLI_MIN_READ_BUF_LEN 100

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

typedef struct EnetInfo_Obj_s
{
    Enet_Type enetType;
    uint32_t instId;
    Enet_Handle hEnet;
    uint8_t numMacPorts;
    uint32_t coreId;
} EnetInfo_Obj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void EnetCli_init(Enet_Type enetType, uint32_t instId);

BaseType_t EnetCli_processCommand(const char *commandInput, char *writeBuffer,
        size_t writeBufferLen);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

EnetInfo_Obj EnetInfo_inst;

#ifdef __cplusplus
}
#endif

#endif /* _ENET_CLI_H_ */
