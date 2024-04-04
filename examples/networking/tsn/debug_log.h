/*
 *  Copyright (c) Texas Instruments Incorporated 2023
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
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#ifndef __DEBUG_LOG_H__
#define __DEBUG_LOG_H__
#include "tsnapp_porting.h"

/* Writing log directly to the console can impact the performance.
 * So by the default the log will be written to the buffer and then a log task
 * will print to the console later. */
#ifndef TSN_USE_LOG_BUFFER
#define TSN_USE_LOG_BUFFER 1
#endif

typedef void (*Logger_onConsoleOut)(const char *str, ...);

extern Logger_onConsoleOut sDrvConsoleOut;

#define DPRINT(str,...) sDrvConsoleOut(str ENDLINE, ##__VA_ARGS__)

int Logger_logToBuffer(bool flush, const char *str);
int Logger_directLog(bool flush, const char *str);
#if TSN_USE_LOG_BUFFER == 1
#define LOG_OUTPUT Logger_logToBuffer
#else
#define LOG_OUTPUT Logger_directLog
#endif

int Logger_init(Logger_onConsoleOut consoleOutCb);
void Logger_deInit(void);

#endif
