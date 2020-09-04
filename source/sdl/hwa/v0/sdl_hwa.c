/*
 *   Copyright (c) Texas Instruments Incorporated 2022
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
 *  \file     sdl_hwa.c
 *
 *  \brief    This file contains the implementation of the APIs present in the
 *            device abstraction layer file of hwa.
 *            This also contains some related macros.
 */




/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdbool.h>
#include <sdl/include/sdl_types.h>
#include <sdl/include/hw_types.h>
#include "sdl_ip_hwa.h"
#include "sdl_hwa_hw.h"
#include "sdl_hwa.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define SDL_HWA_INIT_TIMEOUT   (0XFFU)
#define SDL_HWA_ECC_TIMEOUT    (0x10000U)
/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */


/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */
 /**
 *  \brief   This API is used to inject error in HWA DMA1
 *
 * \param   fiType     indicates the Fi type
 *
 * \param   redType   indicates interface type
 *
 */
static void SDL_HWA_DMA1_busSftyFitype(SDL_HWA_busSftyFiType fiType,\
                                       SDL_HWA_busSftyFiRedType redType );

 /**
 *  \brief   This API is used to inject error in HWA DMA0
 *
 * \param   fiType     indicates the Fi type
 *
 * \param   redType   indicates interface type
 *
 */

static void SDL_HWA_DMA0_busSftyFitype(SDL_HWA_busSftyFiType fiType, \
                                       SDL_HWA_busSftyFiRedType redType );

/* ========================================================================== */

/*                   Internal Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
/********************************************************************************************************
*   API to clear SEC error on HWA DMA0
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3764
 */
void SDL_HWA_DMA0_secErrorClear(void)
{
    /* clear SEC error on node */
    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_SEC, 0x0U);

    /* clear error */
    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL),\
    SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_ERR_CLEAR,0x1U);

    /* clear SEC error in DSS_BUS_SAFETY_SEC_ERR_STAT0 */
    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1),\
    SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_HWA_DMA0,0x1U);
}

/********************************************************************************************************
*   API to get SEC error status on HWA DMA0
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3764
 */
uint32_t SDL_HWA_DMA0_secErrorStatus(void)
{
    return((uint32_t) HW_RD_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR),\
                         SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_DSS_HWA_DMA0_BUS_SAFETY_ERR_SEC));
}


/********************************************************************************************************
*   API to clear SEC error on HWA DMA1
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3765
 */

void SDL_HWA_DMA1_secErrorClear(void)
{
    /* clear SEC error on node */
    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_SEC, 0x0U);

    /* clear error */
    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL),\
    SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_ERR_CLEAR,0x1U);

    /* clear SEC error in DSS_BUS_SAFETY_SEC_ERR_STAT0 */
    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1),\
    SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_HWA_DMA1,0x1U);

}

/********************************************************************************************************
*   API to get SEC error status on HWA DMA1
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3765
 */

uint32_t SDL_HWA_DMA1_secErrorStatus(void)
{
    return((uint32_t) HW_RD_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR),\
                         SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_DSS_HWA_DMA1_BUS_SAFETY_ERR_SEC));
}

/********************************************************************************************************
*   API to clear DED error on HWA DMA0
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3766
 */
void SDL_HWA_DMA0_dedErrorClear(void)
{
    /* clear SEC error on node */
    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_DED, 0x0U);
    /* clear error */
    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL),\
    SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_ERR_CLEAR,0x1U);

}

/********************************************************************************************************
*   API to get DED error status on HWA DMA0
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3766
 */
uint32_t SDL_HWA_DMA0_dedErrorStatus(void)
{
    return((uint32_t) HW_RD_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR),\
                         SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_DSS_HWA_DMA0_BUS_SAFETY_ERR_DED));
}

/********************************************************************************************************
*   API to clear DED error on HWA DMA1
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3767
 */

void SDL_HWA_DMA1_dedErrorClear(void)
{
    /* clear DED error on node */
    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_DED, 0x0U);
    /* clear error */
    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL),\
    SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_ERR_CLEAR,0x1U);

}

/********************************************************************************************************
*   API to get DED error status on HWA DMA1
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3767
 */

uint32_t SDL_HWA_DMA1_dedErrorStatus(void)
{
    return((uint32_t) HW_RD_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR),\
                         SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_DSS_HWA_DMA1_BUS_SAFETY_ERR_DED));
}

/********************************************************************************************************
*   API to clear RED error on HWA DMA0
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3768
 */

void SDL_HWA_DMA0_redErrorClear(void)
{
    /* clear RED error on node */
    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_GLOBAL_MAIN, 0x0U);
    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_GLOBAL_SAFE, 0x0U);
    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_MAIN, 0x0U);
    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_SAFE, 0x0U);
}

/********************************************************************************************************
*   API to clear RED error on HWA DMA0
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3768
 */

uint32_t SDL_HWA_DMA0_redErrorStatus(void)
{
    uint32_t cmdErr = 0U;
    uint32_t writeErr = 0U;
    uint32_t readErr = 0U;
    uint32_t writestepErr = 0U;
    uint32_t retval = 0U;
    cmdErr = ((uint32_t) HW_RD_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_CMD),\
                        SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_CMD_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_CMD_STAT));
    writeErr = ((uint32_t) HW_RD_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITE),\
                        SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITE_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITE_STAT));
    readErr = ((uint32_t) HW_RD_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_READ),\
                        SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_READ_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_READ_STAT));
    writestepErr = ((uint32_t) HW_RD_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITERESP),\
                        SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITERESP_STAT));
    if((cmdErr!=0U) || (writeErr!=0U) || (readErr!=0U)  || (writestepErr !=0U) )
    {
        retval = 1U;
    }
    else
    {
        retval = 0U;
    }
    return (retval );
}

/********************************************************************************************************
*   API to clear RED error on HWA DMA1
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3769
 */
void SDL_HWA_DMA1_redErrorClear(void)
{
    /* clear RED error on node */
    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_GLOBAL_MAIN, 0x0U);
    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_GLOBAL_SAFE, 0x0U);
    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_MAIN, 0x0U);
    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_SAFE, 0x0U);
}

/********************************************************************************************************
*   API to clear RED error on HWA DMA1
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3769
 */
uint32_t SDL_HWA_DMA1_redErrorStatus(void)
{
    uint32_t cmdErr = 0U;
    uint32_t writeErr = 0U;
    uint32_t readErr = 0U;
    uint32_t writestepErr = 0U;
    uint32_t retval = 0U;
    cmdErr = ((uint32_t) HW_RD_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_CMD),\
                        SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_CMD_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_CMD_STAT));
    writeErr = ((uint32_t) HW_RD_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITE),\
                        SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITE_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITE_STAT));
    readErr = ((uint32_t) HW_RD_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_READ),\
                        SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_READ_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_READ_STAT));
    writestepErr = ((uint32_t) HW_RD_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITERESP),\
                        SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITERESP_STAT));
    if((cmdErr!=0U) || (writeErr!=0U) || (readErr!=0U)  || (writestepErr !=0U) )
    {
        retval = 1U;
    }
    else
    {
        retval = 0U;
    }
    return (retval );
}

/* Helper function to induce red error on Bank B*/
static void SDL_HWA_DMA1_busSftyFitype(SDL_HWA_busSftyFiType fiType, SDL_HWA_busSftyFiRedType redType )
{
    switch(fiType)
    {
        case SDL_HWA_FI_MAIN:
 	    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_MAIN,\
		(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_HWA_FI_SAFE:

 	    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_SAFE,\
		(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_HWA_FI_GLOBAL_MAIN:
 	    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_GLOBAL_MAIN,\
		0x1U);
        break;
        case SDL_HWA_FI_GLOBAL_SAFE:
	    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_GLOBAL_SAFE,\
		0x1U);
        break;
        default:
        /* do nothing */
        break;
    }
}
/* Helper function to induce red error on Bank B*/
static void SDL_HWA_DMA0_busSftyFitype(SDL_HWA_busSftyFiType fiType, SDL_HWA_busSftyFiRedType redType )
{
    switch(fiType)
    {
        case SDL_HWA_FI_MAIN:
 	    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_MAIN,\
		(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_HWA_FI_SAFE:

 	    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_SAFE,\
		(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_HWA_FI_GLOBAL_MAIN:
 	    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_GLOBAL_MAIN,\
		0x1U);
        break;
        case SDL_HWA_FI_GLOBAL_SAFE:
	    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_GLOBAL_SAFE,\
		0x1U);
        break;
        default:
        /* do nothing */
        break;
    }
}

/********************************************************************************************************
* API to Execute SEC test on HWA DMA0
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3770
 */

void SDL_HWA_DMA0_secExecute(void)
{
    volatile uint32_t addr, wr_data;
    /* enable the bus safety for DSS */
	HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
    /* enable the bus safety for DMA0 */
	HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
    /* Start diagnostic test case*/
    addr = SDL_HWA_DMA0_ADDRESS;
    wr_data = (uint32_t)0x12345678U;
    /* enable fault injection for node with: single error correction, appropriate 32 bits of bus */
	HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_SEC,\
		0x1U);
    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_DATA,\
         0x1);

    /* issue a bus transaction to node and read back */
    *(volatile uint32_t *)addr = wr_data;
}

/********************************************************************************************************
* API to Execute SEC test on HWA DMA1
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3771
 */

void SDL_HWA_DMA1_secExecute(void)
{
    volatile uint32_t addr, wr_data;
    /* enable the bus safety for DSS */
	HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
    /* enable the bus safety for DMA1 */
	HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
    /* Start diagnostic test case*/
    addr = SDL_HWA_DMA1_ADDRESS;
    wr_data =(uint32_t) 0x12345678U;

    /* enable fault injection for node with: single error correction, appropriate 32 bits of bus */
	HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_SEC,\
		0x1U);
    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_DATA,\
         0x1);

    /* issue a bus transaction to node and read back */
    *(volatile uint32_t *)addr = wr_data;
}

/********************************************************************************************************
* API to Execute Ded test on HWA DMA0
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3772
 */
void SDL_HWA_DMA0_dedExecute(void)
{
    volatile uint32_t addr, wr_data;
    /* enable the bus safety for DSS */
	HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
    /* enable the bus safety for DMA0 */
	HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
    /* Start diagnostic test case*/
    addr = SDL_HWA_DMA0_ADDRESS;
    wr_data = (uint32_t)0x12345678U;
    /* enable fault injection for node with: double error correction, appropriate 32 bits of bus */
	HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_DED,\
		0x1U);
    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI_DSS_HWA_DMA0_BUS_SAFETY_FI_DATA,\
         0x1);

    /* issue a bus transaction to node and read back */
    *(volatile uint32_t *)addr = wr_data;
}

/********************************************************************************************************
* API to Execute DED test on HWA DMA1
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3773
 */
void SDL_HWA_DMA1_dedExecute(void)
{
    volatile uint32_t addr, wr_data;
    /* enable the bus safety for DSS */
	HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
    /* enable the bus safety for DMA1 */
	HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
    /* Start diagnostic test case*/
    addr = SDL_HWA_DMA1_ADDRESS;
    wr_data = (uint32_t)0x12345678U;
    /* enable fault injection for node with: double error correction, appropriate 32 bits of bus */
	HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_DED,\
		0x1U);
    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI_DSS_HWA_DMA1_BUS_SAFETY_FI_DATA,\
         0x1);

    /* issue a bus transaction to node and read back */
    *(volatile uint32_t *)addr = wr_data;
}

/********************************************************************************************************
* API to Execute RED test on HWA DMA0
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3774
 */
int32_t SDL_HWA_DMA0_redExecute(SDL_HWA_busSftyFiType fiType, SDL_HWA_busSftyFiRedType redType )
{
    int32_t ret_val = SDL_EFAIL;
    if ((fiType< SDL_HWA_FI_INVALID ) && (redType <SDL_HWA_FI_TYPE_INVALID))
    {
        /* enable the bus safety for DSS */
	    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
        /* enable the bus safety for DMA1 */
	    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
        /* Inject fault. */
        SDL_HWA_DMA0_busSftyFitype(fiType,redType);
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EBADARGS;
    }
    return ret_val;
}

/********************************************************************************************************
* API to Execute RED test on HWA DMA1
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3775
 */
int32_t SDL_HWA_DMA1_redExecute(SDL_HWA_busSftyFiType fiType, SDL_HWA_busSftyFiRedType redType)
{
    int32_t ret_val = SDL_EFAIL;
    if ((fiType< SDL_HWA_FI_INVALID ) && (redType <SDL_HWA_FI_TYPE_INVALID))
    {
        /* enable the bus safety for DSS */
	    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
        /* enable the bus safety for DMA1 */
	    HW_WR_FIELD32((SDL_HWA_BUS_CFG+SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
        /* Start diagnostic test case*/
        /* Inject fault. */
        SDL_HWA_DMA1_busSftyFitype(fiType,redType);
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EBADARGS;
    }
    return ret_val;
}

/********************************************************************************************************
* API to configuring and test parity error on HWA memory
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3883
 */
int32_t SDL_HWA_memParityExecute( SDL_HWA_MemID memID , SDL_HWA_MemBlock memBlock)
{
    uint32_t timeout = SDL_HWA_INIT_TIMEOUT;
    int32_t retSts = SDL_EFAIL;
    uint8_t retMemInitSts = 0U;
    uint8_t memBlockParity = 0U;
    volatile uint32_t writeData = 0x12345678U;
    uint32_t baseAddr;
    /* validate the memory block */
    if(SDL_HWA_getMemblockBaseaddr(memID,memBlock,
                                 &baseAddr) == SDL_PASS )
    {
        /* Select parity operation for Window RAM */
        if(memBlock == SDL_HWA_WINDOW_RAM )
        {
            memBlockParity = SDL_HWA_DMEM_WINDOW_RAM_PARITY;
        }
        /* Select parity operation for DMEM */
        else
        {
            memBlockParity = SDL_HWA_DMEM_PARITY;
        }
        /* Initialize the memory */
        SDL_HWA_memoryInitStart(memBlock);
        /* wait for memory initialization */
        do
        {
            timeout--;
        }
        while(((uint8_t)1U==SDL_HWA_memoryInitStatus(memBlock))&&(timeout>(uint32_t)0U))
        ;
        timeout = SDL_HWA_ECC_TIMEOUT;
        /* Memory init done Status */
        retMemInitSts = SDL_HWA_memoryInitDone(memBlock);
        if(1U == retMemInitSts)
        {
            /* Parity unmask */
            SDL_HWA_setParityErrMaskUnmask(memBlock, SDL_HWA_DISABLE_STS);
            if((uint8_t)0U == SDL_HWA_getParityErrMaskUnmask(memBlock))
            {
                /* Enable safety */
                SDL_HWA_setParityEnableDisable(memBlockParity,SDL_HWA_ENABLE_STS);
                if((uint8_t)1U == SDL_HWA_getParityStatus(memBlockParity))
                {
                    *(volatile uint32_t *)baseAddr = writeData;
                    SDL_HWA_setParityEnableDisable(memBlockParity,SDL_HWA_DISABLE_STS);
                    if((uint8_t)0U == SDL_HWA_getParityStatus(memBlockParity))
                    {
                        while(timeout!=0U)
                        {
                            timeout--;
                        }
                        timeout = SDL_HWA_ECC_TIMEOUT;
                        writeData = (writeData^((uint32_t)0x1));
                        *(volatile uint32_t *)baseAddr = writeData;
                        SDL_HWA_setParityEnableDisable(memBlockParity,SDL_HWA_ENABLE_STS);
                        if((uint8_t)1U == SDL_HWA_getParityStatus(memBlockParity))
                        {
                            writeData =*(volatile uint32_t *)baseAddr;
                            /* wait for error notification from ESM, or for timeout */
                            /* fault injection gets deasserted in ISR */
                            while((SDL_HWA_getErrStatus(memBlock)!=1U) && (timeout!=0U))
                            {
                                timeout--;
                            }
                            if(SDL_HWA_getErrStatus(memBlock)==1U)
                            {
                                SDL_HWA_clearErrStatus(memBlock);
                                retSts = SDL_PASS;
                            }
                            else
                            {
                                retSts = SDL_EFAIL;
                            }

                        }
                        else
                        {
                            retSts = SDL_EFAIL;
                        }

                    }
                    else
                    {
                        retSts = SDL_PASS;
                    }
                }
                else
                {
                    retSts = SDL_EFAIL;
                }
            }
            else
            {
                retSts = SDL_EFAIL;
            }
        }
        else
        {
            retSts = SDL_EFAIL;
        }
    }
    else
    {
         retSts = SDL_EBADARGS;
    }
    return (retSts);
}
/********************************************************************************************************
* API to induce the error in the fsm lockstep for HWA
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3884
 */
int32_t SDL_HWA_fsmLockStepExecute( void)
{
    int32_t retSts = SDL_EFAIL;
    SDL_HWA_Status_s enSts;
    uint32_t timeout = SDL_HWA_ECC_TIMEOUT;
    uint8_t memBlockParity=0U;
    SDL_HWA_MemBlock memBlock = SDL_HWA_FSM_LOCKSTEP;
    /* Enable HWA for FSM*/
    SDL_HWA_setHwaEnableDisable(SDL_HWA_ENABLE_STS);
    /* Get the FSM enable Status */
    SDL_HWA_getHwaEnableDisable(&enSts);
    if((enSts.SDL_HWA_enableHwaSTS == SDL_HWA_ENABLE_HWA_EN_ENABLE) &&
             (enSts.SDL_HWA_enableHwaClkSTS == SDL_HWA_ENABLE_HWA_CLK_EN_ENABLE))
    {
        /*Enable Parity*/
        SDL_HWA_setParityErrMaskUnmask(memBlock, SDL_HWA_DISABLE_STS);
        if((uint8_t)0U == SDL_HWA_getParityErrMaskUnmask(memBlock))
        {
            memBlockParity = SDL_HWA_DMEM_FSM_LOCKSTEP_EN;
            /* enable FSM safety */
            SDL_HWA_setParityEnableDisable(memBlockParity,SDL_HWA_ENABLE_STS);
            if((uint8_t)1U == SDL_HWA_getParityStatus(memBlockParity))
            {
                memBlockParity = SDL_HWA_DMEM_FSM_LOCKSTEP_INV_EN;
                /*Enable FSM safety*/
                SDL_HWA_setParityEnableDisable(memBlockParity,SDL_HWA_ENABLE_STS);
                if((uint8_t)0U == SDL_HWA_getParityStatus(memBlockParity))
                {
                    /* wait for error notification from ESM, or for timeout */
                    /* fault injection gets deasserted in ISR */
                    while((SDL_HWA_getErrStatus(memBlock)!=1U) && (timeout!=0U))
                    {
                        timeout--;
                    }
                    if(SDL_HWA_getErrStatus(memBlock)==1U)
                    {
                        SDL_HWA_clearErrStatus(memBlock);
                        retSts = SDL_PASS;
                    }
                    else
                    {
                        retSts = SDL_EFAIL;
                    }
                }
                else
                {
                    retSts = SDL_EFAIL;
                }
            }
            else
            {
                retSts = SDL_EFAIL;
            }
        }
        else
        {
            retSts = SDL_EFAIL;
        }
    }
    else
    {
        retSts = SDL_EFAIL;
    }
    return (retSts);
}




