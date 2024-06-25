/*
 *  Copyright (C) 2022-23 Texas Instruments Incorporated
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
 *  \file sdl_hwa.c
 *
 *  \brief Common across test-cases using HWA
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "hwa_main.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
/********************************************************************************************************
* Parity Test for HWA DMA 0 DMEM 0
*********************************************************************************************************/
int32_t hwaParityDMA0DMEM0_testExecute(void)
{
    return (SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM0));
}
/********************************************************************************************************
* Parity Test for HWA DMA 0 DMEM 1
*********************************************************************************************************/
int32_t hwaParityDMA0DMEM1_testExecute(void)
{
    return (SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM1));
}
/********************************************************************************************************
* Parity Test for HWA DMA 0 DMEM 2
*********************************************************************************************************/
int32_t hwaParityDMA0DMEM2_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM2));
}
/********************************************************************************************************
* Parity Test for HWA DMA 0 DMEM 3
*********************************************************************************************************/
int32_t hwaParityDMA0DMEM3_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM3));
}
/********************************************************************************************************
* Parity Test for HWA DMA 0 DMEM 4
*********************************************************************************************************/
int32_t hwaParityDMA0DMEM4_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM4));
}
/********************************************************************************************************
* Parity Test for HWA DMA 0 DMEM 5
*********************************************************************************************************/
int32_t hwaParityDMA0DMEM5_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM5));
}
/********************************************************************************************************
* Parity Test for HWA DMA 0 DMEM 6
*********************************************************************************************************/
int32_t hwaParityDMA0DMEM6_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM6));
}
/********************************************************************************************************
* Parity Test for HWA DMA 0 DMEM 7
*********************************************************************************************************/
int32_t hwaParityDMA0DMEM7_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM7));
}
/********************************************************************************************************
* Parity Test for HWA DMA 0 WINDOW RAM
*********************************************************************************************************/
int32_t hwaParityDMA0WindowRam_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_WINDOW_RAM_MEM_ID , SDL_HWA_WINDOW_RAM));
}
/********************************************************************************************************
* Parity Test for HWA DMA 1 DMEM 0
*********************************************************************************************************/
int32_t hwaParityDMA1DMEM0_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM0));
}
/********************************************************************************************************
* Parity Test for HWA DMA 1 DMEM 1
*********************************************************************************************************/
int32_t hwaParityDMA1DMEM1_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM1));
}
/********************************************************************************************************
* Parity Test for HWA DMA 1 DMEM 2
*********************************************************************************************************/
int32_t hwaParityDMA1DMEM2_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM2));
}
/********************************************************************************************************
* Parity Test for HWA DMA 1 DMEM 3
*********************************************************************************************************/
int32_t hwaParityDMA1DMEM3_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM3));
}
/********************************************************************************************************
* Parity Test for HWA DMA 1 DMEM 4
*********************************************************************************************************/
int32_t hwaParityDMA1DMEM4_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM4));
}
/********************************************************************************************************
* Parity Test for HWA DMA 1 DMEM 5
*********************************************************************************************************/
int32_t hwaParityDMA1DMEM5_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM5));
}
/********************************************************************************************************
* Parity Test for HWA DMA 1 DMEM 6
*********************************************************************************************************/
int32_t hwaParityDMA1DMEM6_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM6));
}
/********************************************************************************************************
* Parity Test for HWA DMA 1 DMEM 7
*********************************************************************************************************/
int32_t hwaParityDMA1DMEM7_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM7));
}
/********************************************************************************************************
* Parity Test for HWA Window RAM
*********************************************************************************************************/
int32_t hwaParityDMA1WindowRam_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_WINDOW_RAM_MEM_ID , SDL_HWA_WINDOW_RAM));
}

