/*!
 *  \file
 *
 *  \brief
 *  File is only a placeholder for later error handling !!
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2022-07-18
 *
 *  \copyright
 *  Copyright (c) 2022, KUNBUS GmbH<br /><br />
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

#if !(defined PROTECT_ERRORHANDLING_H)
#define PROTECT_ERRORHANDLING_H        1

#include <stdint.h>

#define GW_ECIOL_ERRORCODE uint32_t

// ERROR-CLASS Sitara EtherCAT gateway  0x4810 0000 till 0x481F FFFF
#define GW_ECIOL_ERROR_CLASS(x)               (0x48100000U | ((x) & (uint32_t)0x000fffffU))

#define GW_ECIOL_ERRORCODE_OK                 (0U)
#define GW_ECIOL_ERRORCODE_INVALIDPARAM       GW_ECIOL_ERROR_CLASS(0x1234U)
#define GW_ECIOL_ERRORCODE_NOMUTEX            GW_ECIOL_ERROR_CLASS(0x1235U)
#define GW_ECIOL_ERRORCODE_NOEVENT            GW_ECIOL_ERROR_CLASS(0x1236U)
#define GW_ECIOL_ERRORCODE_EVENTWAIT          GW_ECIOL_ERROR_CLASS(0x1237U)
#define GW_ECIOL_ERRORCODE_TASKCREATE         GW_ECIOL_ERROR_CLASS(0x1238U)
#define GW_ECIOL_ERRORCODE_NOTSUPPORTED       GW_ECIOL_ERROR_CLASS(0x1239U)
#define GW_ECIOL_ERRORCODE_TQINIT             GW_ECIOL_ERROR_CLASS(0x123AU)
#define GW_ECIOL_ERRORCODE_TQWAIT             GW_ECIOL_ERROR_CLASS(0x123BU)

#define GW_ECIOL_ERRORCODE_IIL_SMIOFFSET      GW_ECIOL_ERROR_CLASS(0x10000U)

#define GW_ECIOL_ERRORCODE_IIL_NOENTRY        GW_ECIOL_ERROR_CLASS(0x21234U)
#define GW_ECIOL_ERRORCODE_IIL_NOCLIENTID     GW_ECIOL_ERROR_CLASS(0x21235U)
#define GW_ECIOL_ERRORCODE_IIL_WRONGPORT      GW_ECIOL_ERROR_CLASS(0x21236U)
#define GW_ECIOL_ERRORCODE_IIL_CBBUSY         GW_ECIOL_ERROR_CLASS(0x21237U)
#define GW_ECIOL_ERRORCODE_IIL_LENGTH         GW_ECIOL_ERROR_CLASS(0x21238U)

#define GW_ECIOL_ERRORCODE_GWL_PORTCNT        GW_ECIOL_ERROR_CLASS(0x31234U)
#define GW_ECIOL_ERRORCODE_GWL_CONFINVALID    GW_ECIOL_ERROR_CLASS(0x31235U)
#define GW_ECIOL_ERRORCODE_GWL_CONFBUSY       GW_ECIOL_ERROR_CLASS(0x31236U)
#define GW_ECIOL_ERRORCODE_GWL_NVRAM          GW_ECIOL_ERROR_CLASS(0x31237U)
#define GW_ECIOL_ERRORCODE_INSUFFICIENT_STATE GW_ECIOL_ERROR_CLASS(0x31238U)

#define GW_ECIOL_ERRORCODE_SLEEPTIME_MS       5000U

#define GW_LOG_IILLEVEL 0
#define GW_LOG_IIL1(...) if(GW_LOG_IILLEVEL > 0) { OSAL_printf(__VA_ARGS__); }
#define GW_LOG_IIL2(...) if(GW_LOG_IILLEVEL > 1) { OSAL_printf(__VA_ARGS__); }
#define GW_LOG_IIL3(...) if(GW_LOG_IILLEVEL > 2) { OSAL_printf(__VA_ARGS__); }

#define GW_LOG_GWLLEVEL 0
#define GW_LOG_GWL1(...) if(GW_LOG_GWLLEVEL > 0) { OSAL_printf(__VA_ARGS__); }
#define GW_LOG_GWL2(...) if(GW_LOG_GWLLEVEL > 1) { OSAL_printf(__VA_ARGS__); }
#define GW_LOG_GWL3(...) if(GW_LOG_GWLLEVEL > 2) { OSAL_printf(__VA_ARGS__); }

#define GW_LOG_EILLEVEL 0
#define GW_LOG_EIL1(...) if(GW_LOG_EILLEVEL > 0) { OSAL_printf(__VA_ARGS__); }
#define GW_LOG_EIL2(...) if(GW_LOG_EILLEVEL > 1) { OSAL_printf(__VA_ARGS__); }
#define GW_LOG_EIL3(...) if(GW_LOG_EILLEVEL > 2) { OSAL_printf(__VA_ARGS__); }

#define GW_LOGLEVEL 0
#define GW_LOG1(...) if(GW_LOGLEVEL > 0) { OSAL_printf(__VA_ARGS__); }
#define GW_LOG2(...) if(GW_LOGLEVEL > 1) { OSAL_printf(__VA_ARGS__); }
#define GW_LOG3(...) if(GW_LOGLEVEL > 2) { OSAL_printf(__VA_ARGS__); }

#endif /* PROTECT_ERRORHANDLING_H */
