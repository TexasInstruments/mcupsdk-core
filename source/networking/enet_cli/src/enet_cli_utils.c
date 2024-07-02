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
 * \file  enet_cli_utils.c
 *
 * \brief this file contains function definitions for utility commands
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <kernel/dpl/TaskP.h>

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

static bool EnetCli_utilsHelp(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

static bool EnetCli_getCpuLoad(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

static bool EnetCli_readMem(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

static bool EnetCli_writeMem(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

bool EnetCli_utilsCommandHandler(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    const char *parameter;
    uint32_t paramLen;
    parameter = EnetCli_getParameter(commandString, 1, &paramLen);

    if (parameter == NULL || strncmp(parameter, "help", paramLen) == 0)
    {
        return EnetCli_utilsHelp(writeBuffer, writeBufferLen, commandString);
    }
    else if (strncmp(parameter, "cpuload", paramLen) == 0)
    {
        return EnetCli_getCpuLoad(writeBuffer, writeBufferLen, commandString);
    }
    else if (strncmp(parameter, "readmem", paramLen) == 0)
    {
        return EnetCli_readMem(writeBuffer, writeBufferLen, commandString);
    }
    else if (strncmp(parameter, "writemem", paramLen) == 0)
    {
        return EnetCli_writeMem(writeBuffer, writeBufferLen, commandString);
    }
    else
    {
        snprintf(writeBuffer, writeBufferLen,
                "Bad argument\r\nFor more info run \'utils help\'\r\n");
        return false;
    }
}

/* ========================================================================== */
/*                   Static Function Definitions                              */
/* ========================================================================== */

static bool EnetCli_utilsHelp(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    EnetAppUtils_print("Utility commands for SOC.\r\nUsage:\r\n");
    EnetAppUtils_print(
            "\tutils cpuload\t\t\t\t\tPrints current CPU usage.\r\n");
    EnetAppUtils_print(
            "\tutils readmem <start_addr> [<num_of_words>]\tPrints value stored in memory address.\r\n");
    EnetAppUtils_print(
            "\t\t\t\t\t\t\tIf number of words is not specified, prints one word.\r\n");
    EnetAppUtils_print(
            "\tutils writemem <start_addr> <word1> ...\tWrites data to memory address.\r\n");
    EnetAppUtils_print("\tutils help\t\t\t\t\tPrints this message.\r\n");
    return false;
}

static bool EnetCli_getCpuLoad(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    uint32_t cpuLoad = TaskP_loadGetTotalCpuLoad();
    snprintf(writeBuffer, writeBufferLen, "Current CPU load = %3d.%02d%%\r\n",
            cpuLoad / 100, cpuLoad % 100);
    return false;
}

static bool EnetCli_readMem(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    static bool processCommand = true;
    static uint32_t addrVal = 0;
    static uint32_t endAddr = 0;
    uint32_t wordCnt = 1;
    uint32_t *addr;
    char *parameter;
    uint32_t paramLen;

    if (processCommand)
    {
        parameter = (char*) EnetCli_getParameter(commandString, 2,
                &paramLen);
        if (parameter == NULL)
        {
            snprintf(writeBuffer, writeBufferLen,
                    "Missing argument(s)\r\nFor more info run \'utils help\'\r\n");
            return false;
        }
        addrVal = (uint32_t) strtol(parameter, NULL, 16);
        parameter = (char*) EnetCli_getParameter(commandString, 3,
                &paramLen);
        if (parameter != NULL)
        {
            wordCnt = atoi(parameter);
        }
        endAddr = addrVal + (wordCnt - 1) * 4;

        if (addrVal % 16)
        {
            addrVal = addrVal - (addrVal % 16);
        }
        if (endAddr % 4)
        {
            endAddr = endAddr - (endAddr % 4);
        }
        processCommand = false;
    }

    addr = (uint32_t*) addrVal;
    if (addrVal % 16 == 0 && addrVal == endAddr)
    {
        snprintf(writeBuffer, writeBufferLen, "\r\n0x%.8x | %.8x\r\n", addrVal,
                addr[0]);
    }
    else if (addrVal % 16 == 0)
    {
        snprintf(writeBuffer, writeBufferLen, "\r\n0x%.8x | %.8x", addrVal,
                addr[0]);
    }
    else if (addrVal == endAddr)
    {
        snprintf(writeBuffer, writeBufferLen, "   %.8x\r\n", addr[0]);
    }
    else
    {
        snprintf(writeBuffer, writeBufferLen, "   %.8x", addr[0]);
    }
    addrVal += 4;
    if (addrVal > endAddr)
    {
        processCommand = true;
        return false;
    }
    return true;
}

static bool EnetCli_writeMem(char *writeBuffer, size_t writeBufferLen,
        const char *commandString)
{
    uint32_t addrVal = 0;
    uint32_t data = 0;
    uint32_t *addr;
    char *parameter;
    uint32_t paramLen;
    uint32_t wordCnt = 0;

    parameter = (char*) EnetCli_getParameter(commandString, 2, &paramLen);
    if (parameter == NULL)
    {
        snprintf(writeBuffer, writeBufferLen,
                "Missing argument(s)\r\nFor more info run \'utils help\'\r\n");
        return false;
    }
    addrVal = (uint32_t) strtol(parameter, NULL, 16);
    if (addrVal % 4)
    {
        addrVal = addrVal - (addrVal % 4);
    }
    addr = (uint32_t*) addrVal;

    parameter = (char*) EnetCli_getParameter(commandString, wordCnt + 3,
            &paramLen);
    while (parameter != NULL)
    {
        data = (uint32_t) strtol(parameter, NULL, 16);
        memcpy(addr + (4 * wordCnt), &data, 4);
        wordCnt++;
        parameter = (char*) EnetCli_getParameter(commandString, wordCnt + 3,
                &paramLen);
    }
    snprintf(writeBuffer, writeBufferLen, "Wrote %d bytes to address %p\r\n",
            wordCnt * 4, addr);
    return false;
}
