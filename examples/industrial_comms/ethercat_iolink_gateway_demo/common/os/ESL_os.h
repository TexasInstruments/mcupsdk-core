/*!
 *  \file ESL_os.h
 *
 *  \brief
 *  Application OS support.
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2021-12-03
 *
 *  \copyright
 *  Copyright (c) 2021, KUNBUS GmbH<br /><br />
 *  All rights reserved.<br />
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:<br />
 *  <ol>
 *  <li>Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.</li>
 *  <li>Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.</li>
 *  <li>Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.</li>
 *  </ol>
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

#if !(defined __ESL_OS_H__)
#define __ESL_OS_H__		1

#include <osal.h>
#include <ESL_OS_os.h>

#if (defined __cplusplus)
extern "C" {
#endif

extern void     ESL_OS_init(void);
extern uint32_t ESL_OS_boardInit(uint32_t pruInstance_p);
extern void     ESL_OS_boardDeinit(void);

extern void     ESL_OS_manualMdioConfig(void* pEcSlvApi_p);

extern void     ESL_OS_taskLeave(void);

extern clock_t  ESL_OS_clockGet(void);
extern clock_t  ESL_OS_clockDiff(clock_t reference_p, clock_t* pNow_p);

extern void*    ESL_OS_ioexp_leds_init(void);
extern void     ESL_OS_ioexp_leds_write(void* pI2cHandle_p, uint8_t ledValue_p);
extern void     ESL_OS_printf(void* pContext_p, const char* __restrict pFormat_p, va_list arg_p);

#if (defined __cplusplus)
}
#endif

#endif /* __ESL_OS_H__ */
