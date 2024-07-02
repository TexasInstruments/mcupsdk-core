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
 * \brief This file contains the APIs of enet_cli lib.
 */

/*!
 * \ingroup  NETWORKING_MODULE
 * \defgroup ENET_CLI_API Enet CLI API
 *
 * @{
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

/* Networking Libraries */
#include <enet.h>
#include <enet_apputils.h>

#include "enet_cli_port.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Initialize variables necessary for CLI.
 *
 * Gets necessary data like enet handle, core ID and number of MAC
 * ports that are required by the built-in commands.
 *
 * \param enetType  Enet peripheral type
 * \param instId    Enet peripheral instance ID
 */
void EnetCli_init(Enet_Type enetType, uint32_t instId);

/*!
 * \brief Registers all built-in commands to command interpreter.
 *
 * Use this to enable built-in commands in the application. Should be
 * called only after calling EnetCli_init().
 */
void EnetCli_registerBuiltInCommands();

/*!
 * \brief Processes the command and runs the associated function.
 *
 * Processes the command and executes the function that is associated to the
 * command.
 *
 * \param pCommandInput     The command to be processed
 * \param pWriteBuffer      Buffer to store output after command is executed
 * \param writeBufferLen    Length of the output buffer
 *
 * \return true if more data needs to be returned. Otherwise false.
 */
bool EnetCli_processCommand(const char *pCommandInput, char *pWriteBuffer,
        size_t writeBufferLen);

/*!
 * \brief Extracts a specific parameter from the command.
 *
 * \param pCommandString    The command from which the parameter needs to be extracted
 * \param wantedParam       The parameter index that needs to be extracted
 * \param paramLen          The length of the extracted parameter
 *
 * \return A pointer to the first character of the extracted parameter string.
 */
const char* EnetCli_getParameter(const char *pCommandString,
        uint32_t wantedParam, uint32_t *paramLen);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* _ENET_CLI_H_ */

/*! @} */
