/*
 *  Copyright (c) 2021, KUNBUS GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
 
#ifndef _KBTRACE_H_INC
#define _KBTRACE_H_INC

#include <stdint.h>
#include <stdarg.h>

#define KBT_MAX_TEXT_LEN      50

typedef struct KBT_StrTraceEntry
{
    char acText[KBT_MAX_TEXT_LEN];
    const char *pcFile;
    int iLine;
} KBT_TTraceEntry;

#define KBT_MAX_ENTRIES         100

typedef struct KBT_StrTrace
{
    int iHead;
    int iTail;
    KBT_TTraceEntry atTrace[KBT_MAX_ENTRIES];
} KBT_TTrace;


#ifdef __cplusplus
extern "C" {
#endif

// For Debugging
#define KBT_addEntry(a,...) KBT_addEntryFL(__FILE__, __LINE__, a, ##__VA_ARGS__)

// For Release 
//#define KBT_addEntry(a,...)


    extern void KBT_traceInit (void);
    extern void KBT_addEntryFL (const char *pcFile_p, int iLine_p, char *pcFormat_p, ...);
    extern uint32_t KBT_getTimeStampUs (void);
    extern uint64_t KBT_getTimeStampNs (void);
    extern uint64_t KBT_getTimeStampRaw (void);

    extern KBT_TTrace KBT_tTrace_g;



#ifdef __cplusplus
}
#endif

#endif   // _KBTRACE_H_INC
