/*
 *  Copyright (C) 2018-2023 Texas Instruments Incorporated
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

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/nortos/dpl/common/printf.h>

extern uint32_t gDebugLogZone;
int32_t _DebugP_log(char *format, ...);
void DebugP_shmLogReaderTaskCreate(void);

void _DebugP_logZone(uint32_t logZone, char *format, ...)
{
    /* cannot be used in ISR */
    if((HwiP_inISR())== 0U )
    {
        if( ( gDebugLogZone & logZone ) == logZone )
        {
            va_list va;
            (void) va_start(va, format);
            (void) vprintf_(format, va);
            (void) va_end(va);
        }
    }
}

int32_t _DebugP_log(char *format, ...)
{
    /* cannot be used in ISR */
    if(( HwiP_inISR())==0U )
    {
        {
            va_list va;
            (void) va_start(va, format);
            (void) vprintf_(format, va);
            (void) va_end(va);
        }
    }
    return 0;
}

/* Empty function definition for noRTOS case */ 
void DebugP_shmLogReaderTaskCreate(void)
{
	return ; 
}
