/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

#ifndef PN_DRV_CONFIG_H_
#define PN_DRV_CONFIG_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                      Driver feature definitions                            */
/* ========================================================================== */

/** @def PTCP_SUPPORT
 *       Enable PTPCP support - required for IRT device
 */
//#define PTCP_SUPPORT

/** @def IRT_LEGACY_STARTUP_SUPPORT
 *       Enable IRT legacy startup mode support in driver
 */
#define IRT_LEGACY_STARTUP_SUPPORT

/** @def MRP_SUPPORT
 *       needs to be defined in case of MRP support
 */
#define MRP_SUPPORT

/** @def APP_NAME
 *       Used in UART logs/LCD etc..
 */
#define APP_NAME "Profinet Slave RT/MRP"

/** application version */
#define APP_VERSION "3.0.0.0"
/* @def RTC_DEBUG
 *      Enable debugging in driver
 */
//#define RTC_DEBUG


#ifdef __cplusplus
}
#endif

#endif /* PN_DRV_CONFIG_H_ */
