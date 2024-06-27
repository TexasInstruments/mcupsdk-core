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
 * \file  l2_networking.h
 *
 * \brief Header file for l2_netwroking.c
 */

#ifndef _L2_NETWORKING_H_
#define _L2_NETWORKING_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Status flag for Enet drives */
#define ENET_UP 1
#define ENET_DOWN 0

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

BaseType_t EnetCLI_openTxChn(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

BaseType_t EnetCLI_openRxChn(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

BaseType_t EnetCLI_transmitPkt(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

BaseType_t EnetCLI_capturePkt(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

BaseType_t EnetCLI_dumpRxBuffer(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

BaseType_t EnetCLI_getHostMac(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

BaseType_t EnetCLI_quitTerminal(char *writeBuffer, size_t writeBufferLen,
        const char *commandString);

int32_t EnetApp_init(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* _L2_NETWORKING_H_ */
