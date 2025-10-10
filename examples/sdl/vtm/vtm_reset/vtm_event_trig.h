/*
 *  Copyright (c) 2025 Texas Instruments Incorporated
 *
 *  VTM Example
 *
 *  Voltage and Thermal Monitor (VTM) Example Application
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
/**
 *  \file vtm_event_trig.h
 *
 *  \brief This file contains declarations and macros for VTM Reset Example
 *
 */

 /* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifndef INCLUDE_EVENT_TRIG_H_
#define INCLUDE_EVENT_TRIG_H_
#include <stdint.h>

/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/

int32_t VTM_runTestCaseTrigger(uint8_t useCaseId);
void VTM_deactivateMaxTempOutrngAlert(void);

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define VTM_USE_CASE_STATUS_NOT_RUN           (0u)
#define VTM_USE_CASE_STATUS_COMPLETED_SUCCESS (1u)
#define VTM_USE_CASE_STATUS_COMPLETED_FAILURE (2u)


#endif /*  INCLUDE_EVENT_TRIG_H_ */
