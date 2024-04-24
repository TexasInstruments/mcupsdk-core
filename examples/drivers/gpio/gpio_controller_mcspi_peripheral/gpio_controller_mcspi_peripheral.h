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

#ifndef GPIO_CONTROLLER_MCSPI_PERIPHERAL_H_
#define GPIO_CONTROLLER_MCSPI_PERIPHERAL_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>

/* ========================================================================== */
/*                             Typedefs & Macros                              */
/* ========================================================================== */
// #define APP_GPIOSPI_8BIT           /* Select the datsize by uncommenting */
#define APP_GPIOSPI_32BIT

#define APP_GPIOSPI_MSGSIZE  (4U)     /* Select the message size (Buffer size) */

#ifdef APP_GPIOSPI_8BIT
    #define APP_GPIOSPI_DATASIZE (8U)
    #define MASK 0x80
#else
    #define APP_GPIOSPI_DATASIZE (32U)
    #define MASK 0x80000000
#endif

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

#ifdef APP_GPIOSPI_8BIT
    uint8_t gMcspiTxBuffer[APP_GPIOSPI_MSGSIZE];
    uint8_t gMcspiRxBuffer[APP_GPIOSPI_MSGSIZE];
#else
    uint32_t gMcspiTxBuffer[APP_GPIOSPI_MSGSIZE];
    uint32_t gMcspiRxBuffer[APP_GPIOSPI_MSGSIZE];
#endif

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */



/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

#endif