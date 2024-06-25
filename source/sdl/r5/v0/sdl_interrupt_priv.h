/*
 *  Copyright (C) 2013-2017 Texas Instruments Incorporated - http://www.ti.com/
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
 *
 */

/**
 *  \file  sdl_interrupt_priv.h
 *
 *  \brief This file contains the internal defines for configuring interrupt
 *         support for the ARM Cortex-R5 and VIM
 *
 *  The following is the recommended calling sequence for this API:
 *
 *  1. Call Intc_Init() to initialize the interrupt support
 *  2. For each interrupt you wish to enable...
 *     a. Call Intc_IntSetSrcType() to set interrupt type (level or pulse)
 *     b. Call Intc_IntPrioritySet() to set the interrupt priority (0-15) and
 *        mapping (IRQ or FIQ)
 *     c. Call Intc_IntRegister() to register the interrupt
 *     d. Call Intc_IntEnable() to enable the interrupt
 *  3. Call Intc_SystemEnable() to enable the R5 IRQ and FIQ interrupts
 *
 *  When an interrupt occurs, control goes to the ISR given as a parameter
 *  in the Intc_IntRegister() call for that specific interrupt. The pending
 *  interrupt is automatically cleared and acknowledged in the VIM.
 *
 *  Other calls available:
 *  o Call Intc_IntUnregister() to unregister a previously registered interrupt
 *  o Call Intc_IntDisable() to disable a previously enabled interrupt
 *  o Call Intc_SystemDisable() to disable the R5 IRQ and FIQ interrupts
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#ifndef INTERRUPT_PRIV_H
#define INTERRUPT_PRIV_H

#include <stdint.h>
#include "sdl_interrupt.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Global Declarations                               */
/* ========================================================================== */
extern SDL_R5ExptnHandlers gExptnHandlers;


#ifdef __cplusplus
}
#endif
#endif /* INTERRUPT_PRIV_H */
/********************************* End of file ******************************/
