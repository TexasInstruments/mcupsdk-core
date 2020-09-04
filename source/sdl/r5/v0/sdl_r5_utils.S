/******************************************************************************
* SDL UTILS
*
* Software Diagnostics Library Utils module
*
*  (C) Copyright 2022 Texas Instruments, Inc.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   Redistributions of source code must retain the above copyright
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
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES LOSS OF USE,
*  DATA, OR PROFITS OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*  @file sdl_r5_utils.S
*
*  @brief
*   Implementation file for the ARM R5 utils
*
*   Contains the different control command and status query functions definitions
*
*   \par
******************************************************************************/
    .text

/*==============================================================================
*   Get Correctable Fault Location Register (CFLR) value
==============================================================================*/
    .global SDL_UTILS_getCFLR
SDL_UTILS_getCFLR:
    MRC p15, #0, r0, c15, c3, #0
    BX      lr

/*=============================================================================
*   Get Performance Monitors Overflow Flag Status Register (PMOVSR) value
==============================================================================*/
    .global SDL_UTILS_getPMOVSR
SDL_UTILS_getPMOVSR:
    MRC p15, #0, r0, c9, c12, #3
    BX      lr

/*==============================================================================
*   Get Data Fault Status Register (DFSR) value
==============================================================================*/
    .global SDL_UTILS_getDFSR
SDL_UTILS_getDFSR:
    MRC p15, #0, r0, c5, c0, #0
    BX      lr

/*==============================================================================
*   Get Data Fault Address Register (DFAR) value
==============================================================================*/
    .global SDL_UTILS_getDFAR
SDL_UTILS_getDFAR:
    MRC p15, #0, r0, c6, c0, #0
    BX      lr

/*==============================================================================
*   Get Instruction Fault Status Register Register (IFSR) value
==============================================================================*/
    .global SDL_UTILS_getIFSR
SDL_UTILS_getIFSR:
    MRC p15, #0, r0, c5, c0, #1
    BX      lr

/*==============================================================================
*   Get Instruction Fault Address Register Register (IFAR) value
==============================================================================*/
    .global SDL_UTILS_getIFAR
SDL_UTILS_getIFAR:
    MRC p15, #0, r0, c6, c0, #2
    BX      lr

/*****************************************************************************
* Enable Event Bus
******************************************************************************/
	.global SDL_UTILS_enable_event_bus 
SDL_UTILS_enable_event_bus: 
	ORR r1,r1,#0x7 << 25           	/* Don't need to force dpu_tcm_check_parity_en_o */
	MCR p15, #0, r1, c1, c0, #1   	/* ACTLR enable ECC for cache */
	MRC p15, #0, r0, c1, c0, #0   	/* Read System Control Register */
	NOP
	MRC P15,#0, R5, C9, C12,#0 		/* enable export of events to Eventbus for monitoring */
	ORR R5, R5, #0x11
	MCR P15,#0, R5, C9, C12,#0
	BX      lr
