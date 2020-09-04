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
 *  \file     sdl_mbox.c
 *
 *  \brief    This file contains the implementation of the APIs present in the
 *            device abstraction layer file of mbox.
 *            This also contains some related macros.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdbool.h>
#include <sdl/include/sdl_types.h>
#include <sdl/include/hw_types.h>
#include "sdl_mbox_hw.h"
#include "sdl_mbox.h"


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */


/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */
 /**
 *  \brief   This API is used to inject error in MSS MBOX
 *
 * \param   fiType     indicates the Fi type
 *
 * \param   redType   indicates interface type
 *
 */
static void SDL_MSS_MBOX_busSftyFitype(SDL_MBOX_busSftyFiType fiType,\
                                       SDL_MBOX_busSftyFiRedType redType );

 /**
 *  \brief   This API is used to inject error in DSS MBOX
 *
 * \param   fiType     indicates the Fi type
 *
 * \param   redType   indicates interface type
 *
 */
static void SDL_DSS_MBOX_busSftyFitype(SDL_MBOX_busSftyFiType fiType,\
                                       SDL_MBOX_busSftyFiRedType redType );

#if defined (SOC_AWR294X)
 /**
 *  \brief   This API is used to inject error in RSS MBOX
 *
 * \param   fiType     indicates the Fi type
 *
 * \param   redType   indicates interface type
 *
 */
static void SDL_RSS_MBOX_busSftyFitype(SDL_MBOX_busSftyFiType fiType,\
                                       SDL_MBOX_busSftyFiRedType redType );

#endif
/* ========================================================================== */

/*                   Internal Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
/********************************************************************************************************
*   API to clear SEC error on MSS MBOX
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3744
 */
void SDL_MSS_MBOX_secErrorClear(void)
{
    /* clear SEC error on node */
    HW_WR_FIELD32((SDL_MSS_MBOX_BUS_CFG+SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI),\
    SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_SEC, 0x0U);

    /* clear error */
    HW_WR_FIELD32((SDL_MSS_MBOX_BUS_CFG+SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_CTRL),\
    SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_ERR_CLEAR,0x1U);

    /* clear SEC error in MSS_BUS_SAFETY_SEC_ERR_STAT0 */
    HW_WR_FIELD32((SDL_MSS_MBOX_BUS_CFG+SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0),\
    SDL_MSS_CTRL_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_BUS_SAFETY_SEC_ERR_STAT0_MSS_MBOX,0x1U);
}

/********************************************************************************************************
*   API to clear SEC error on DSS MBOX
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3745
 */
void SDL_DSS_MBOX_secErrorClear(void)
{
    /* clear SEC error on node */
    HW_WR_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_SEC, 0x0U);

    /* clear error */
    HW_WR_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_CTRL),\
    SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_ERR_CLEAR,0x1U);

    /* clear SEC error in DSS_BUS_SAFETY_SEC_ERR_STAT0 */
    HW_WR_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1),\
    SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_BUS_SAFETY_SEC_ERR_STAT1_DSS_MBOX,0x1U);
}

/********************************************************************************************************
*   API to clear SEC error on RSS MBOX
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3746
 */
#if defined (SOC_AWR294X)
void SDL_RSS_MBOX_secErrorClear(void)
{
    /* clear SEC error on node */
    HW_WR_FIELD32((SDL_RSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI),\
    SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_SEC, 0x0U);

    /* clear error */
    HW_WR_FIELD32((SDL_RSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_CTRL),\
    SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_ERR_CLEAR,0x1U);

    /* clear SEC error in RSS_BUS_SAFETY_SEC_ERR_STAT0 */
    HW_WR_FIELD32((SDL_RSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0),\
    SDL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_MBOX,0x1U);
}
#endif

/********************************************************************************************************
*   API to clear DED error on MSS MBOX
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3747
 */
void SDL_MSS_MBOX_dedErrorClear(void)
{
    /* clear SEC error on node */
    HW_WR_FIELD32((SDL_MSS_MBOX_BUS_CFG+SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI),\
    SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_DED, 0x0U);
    /* clear error */
    HW_WR_FIELD32((SDL_MSS_MBOX_BUS_CFG+SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_CTRL),\
    SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_ERR_CLEAR,0x1U);
}

/********************************************************************************************************
*   API to clear DED error on DSS MBOX
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3748
 */
void SDL_DSS_MBOX_dedErrorClear(void)
{
    /* clear SEC error on node */
    HW_WR_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_DED, 0x0U);
    /* clear error */
    HW_WR_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_CTRL),\
    SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_ERR_CLEAR,0x1U);
}


/********************************************************************************************************
*   API to clear DED error on RSS MBOX
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3749
 */

#if defined (SOC_AWR294X)
void SDL_RSS_MBOX_dedErrorClear(void)
{
    /* clear SEC error on node */
    HW_WR_FIELD32((SDL_RSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI),\
    SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_DED, 0x0U);
    /* clear error */
    HW_WR_FIELD32((SDL_RSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_CTRL),\
    SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_ERR_CLEAR,0x1U);
}
#endif
/********************************************************************************************************
*   API to clear RED error on MSS MBOX
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3750
 */
void SDL_MSS_MBOX_redErrorClear(void)
{
    /* clear RED error on node */
    HW_WR_FIELD32((SDL_MSS_MBOX_BUS_CFG+SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI),\
    SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN, 0x0U);
    HW_WR_FIELD32((SDL_MSS_MBOX_BUS_CFG+SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI),\
    SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE, 0x0U);
    HW_WR_FIELD32((SDL_MSS_MBOX_BUS_CFG+SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI),\
    SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_MAIN, 0x0U);
    HW_WR_FIELD32((SDL_MSS_MBOX_BUS_CFG+SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI),\
    SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_SAFE, 0x0U);
}

/********************************************************************************************************
*   API to clear RED error on DSS MBOX
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3751
 */
void SDL_DSS_MBOX_redErrorClear(void)
{
    /* clear RED error on node */
    HW_WR_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN, 0x0U);
    HW_WR_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE, 0x0U);
    HW_WR_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_MAIN, 0x0U);
    HW_WR_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_SAFE, 0x0U);
}

/********************************************************************************************************
*   API to clear RED error on RSS MBOX
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3752
 */
#if defined (SOC_AWR294X)
void SDL_RSS_MBOX_redErrorClear(void)
{
    /* clear RED error on node */
    HW_WR_FIELD32((SDL_RSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI),\
    SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN, 0x0U);
    HW_WR_FIELD32((SDL_RSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI),\
    SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE, 0x0U);
    HW_WR_FIELD32((SDL_RSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI),\
    SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_MAIN, 0x0U);
    HW_WR_FIELD32((SDL_RSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI),\
    SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_SAFE, 0x0U);
}
#endif
/* Helper function to induce red error on MSS MBOX*/
static void SDL_MSS_MBOX_busSftyFitype(SDL_MBOX_busSftyFiType fiType, SDL_MBOX_busSftyFiRedType redType )
{
    switch(fiType)
    {
        case SDL_MBOX_FI_MAIN:
 	    HW_WR_FIELD32((SDL_MSS_MBOX_BUS_CFG+SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI),\
        SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_MAIN,\
		(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_MBOX_FI_SAFE:
 	    HW_WR_FIELD32((SDL_MSS_MBOX_BUS_CFG+SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI),\
        SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_SAFE,\
		(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_MBOX_FI_GLOBAL_MAIN:
 	    HW_WR_FIELD32((SDL_MSS_MBOX_BUS_CFG+SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI),\
        SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN,\
		0x1U);
        break;
        case SDL_MBOX_FI_GLOBAL_SAFE:
	    HW_WR_FIELD32((SDL_MSS_MBOX_BUS_CFG+SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI),\
        SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE,\
		0x1U);
        break;
        default:
        /* do nothing */
        break;
    }
}

/* Helper function to induce red error on DSS MBOX*/
static void SDL_DSS_MBOX_busSftyFitype(SDL_MBOX_busSftyFiType fiType, SDL_MBOX_busSftyFiRedType redType )
{
    switch(fiType)
    {
        case SDL_MBOX_FI_MAIN:
 	    HW_WR_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_MAIN,\
		(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_MBOX_FI_SAFE:
 	    HW_WR_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_SAFE,\
		(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_MBOX_FI_GLOBAL_MAIN:
 	    HW_WR_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN,\
		0x1U);
        break;
        case SDL_MBOX_FI_GLOBAL_SAFE:
	    HW_WR_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE,\
		0x1U);
        break;
        default:
        /* do nothing */
        break;
    }
}
#if defined (SOC_AWR294X)
/* Helper function to induce red error on RSS MBOX*/
static void SDL_RSS_MBOX_busSftyFitype(SDL_MBOX_busSftyFiType fiType, SDL_MBOX_busSftyFiRedType redType )
{
    switch(fiType)
    {
        case SDL_MBOX_FI_MAIN:
 	    HW_WR_FIELD32((SDL_RSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI),\
        SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_MAIN,\
		(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_MBOX_FI_SAFE:
 	    HW_WR_FIELD32((SDL_RSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI),\
        SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_SAFE,\
		(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_MBOX_FI_GLOBAL_MAIN:
 	    HW_WR_FIELD32((SDL_RSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI),\
        SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_GLOBAL_MAIN,\
		0x1U);
        break;
        case SDL_MBOX_FI_GLOBAL_SAFE:
	    HW_WR_FIELD32((SDL_RSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI),\
        SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_GLOBAL_SAFE,\
		0x1U);
        break;
        default:
        /* do nothing */
        break;
    }
}
#endif
/********************************************************************************************************
* API to Execute SEC test on MSS MBOX
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3753
 */
void SDL_MSS_MBOX_secExecute(void)
{
    volatile uint32_t addr, wr_data;
    /* enable the bus safety for MSS */
	HW_WR_FIELD32((SDL_MSS_MBOX_BUS_CFG+SDL_MSS_CTRL_MSS_BUS_SAFETY_CTRL),\
        SDL_MSS_CTRL_MSS_BUS_SAFETY_CTRL_MSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
    /* enable the bus safety for MSS MBOX */
	HW_WR_FIELD32((SDL_MSS_MBOX_BUS_CFG+SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_CTRL),\
        SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
    /* Start diagnostic test case*/
    addr = SDL_MSS_MBOX_U_BASE;
    wr_data = (uint32_t)0x12345678U;
    /* enable fault injection for node with: single error correction, appropriate 32 bits of bus */
	HW_WR_FIELD32((SDL_MSS_MBOX_BUS_CFG+SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI),\
        SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_SEC,\
		0x1U);
    HW_WR_FIELD32((SDL_MSS_MBOX_BUS_CFG+SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI),\
        SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_DATA,\
         0x1);

    /* issue a bus transaction to node and read back */
    *(volatile uint32_t *)addr = wr_data;
}

/********************************************************************************************************
* API to Execute SEC test on DSS MBOX
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3754
 */
void SDL_DSS_MBOX_secExecute(void)
{
    volatile uint32_t addr, wr_data;
    /* enable the bus safety for DSS */
	HW_WR_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
    /* enable the bus safety for DSS MBOX */
	HW_WR_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
    /* Start diagnostic test case*/
    addr = SDL_DSS_MAILBOX_U_BASE;
    wr_data = (uint32_t)0x12345678U;
    /* enable fault injection for node with: single error correction, appropriate 32 bits of bus */
	HW_WR_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_SEC,\
		0x1U);
    HW_WR_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_DATA,\
         0x1);

    /* issue a bus transaction to node and read back */
    *(volatile uint32_t *)addr = wr_data;
}

/********************************************************************************************************
* API to Execute SEC test on RSS MBOX
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3755
 */
#if defined (SOC_AWR294X)
void SDL_RSS_MBOX_secExecute(void)
{
    volatile uint32_t addr, wr_data;
    /* enable the bus safety for RSS */
	HW_WR_FIELD32((SDL_RSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_BUS_SAFETY_CTRL),\
        SDL_RSS_CTRL_RSS_BUS_SAFETY_CTRL_RSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
    /* enable the bus safety for RSS MBOX */
	HW_WR_FIELD32((SDL_RSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_CTRL),\
        SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
    /* Start diagnostic test case*/
    addr = SDL_RSS_CR4_MBOX_U_BASE;
    wr_data = (uint32_t)0x12345678U;
    /* enable fault injection for node with: single error correction, appropriate 32 bits of bus */
	HW_WR_FIELD32((SDL_RSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI),\
        SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_SEC,\
		0x1U);
    HW_WR_FIELD32((SDL_RSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI),\
        SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_DATA,\
         0x1);

    /* issue a bus transaction to node and read back */
    *(volatile uint32_t *)addr = wr_data;
}
#endif

/********************************************************************************************************
* API to Execute DED test on MSS MBOX
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3756
 */
void SDL_MSS_MBOX_dedExecute(void)
{
    volatile uint32_t addr, wr_data;
    /* enable the bus safety for MSS */
	HW_WR_FIELD32((SDL_MSS_MBOX_BUS_CFG+SDL_MSS_CTRL_MSS_BUS_SAFETY_CTRL),\
        SDL_MSS_CTRL_MSS_BUS_SAFETY_CTRL_MSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
    /* enable the bus safety for MSS MBOX */
	HW_WR_FIELD32((SDL_MSS_MBOX_BUS_CFG+SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_CTRL),\
        SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
    /* Start diagnostic test case*/
    addr = SDL_MSS_MBOX_U_BASE;
    wr_data = (uint32_t)0x12345678U;
    /* enable fault injection for node with: double error correction, appropriate 32 bits of bus */
	HW_WR_FIELD32((SDL_MSS_MBOX_BUS_CFG+SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI),\
        SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_DED,\
		0x1U);
    HW_WR_FIELD32((SDL_MSS_MBOX_BUS_CFG+SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI),\
        SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI_MSS_MBOX_BUS_SAFETY_FI_DATA,\
         0x1);

    /* issue a bus transaction to node and read back */
    *(volatile uint32_t *)addr = wr_data;
}

/********************************************************************************************************
* API to Execute DED test on DSS MBOX
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3757
 */
void SDL_DSS_MBOX_dedExecute(void)
{
    volatile uint32_t addr, wr_data;
    /* enable the bus safety for DSS */
	HW_WR_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
    /* enable the bus safety for DSS MBOX */
	HW_WR_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
    /* Start diagnostic test case*/
    addr = SDL_DSS_MAILBOX_U_BASE;
    wr_data = (uint32_t)0x12345678U;
    /* enable fault injection for node with: double error correction, appropriate 32 bits of bus */
	HW_WR_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_DED,\
		0x1U);
    HW_WR_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI_DSS_MBOX_BUS_SAFETY_FI_DATA,\
         0x1);

    /* issue a bus transaction to node and read back */
    *(volatile uint32_t *)addr = wr_data;
}

/********************************************************************************************************
* API to Execute DED test on RSS MBOX
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3758
 */
#if defined (SOC_AWR294X)
void SDL_RSS_MBOX_dedExecute(void)
{
    volatile uint32_t addr, wr_data;
    /* enable the bus safety for RSS */
	HW_WR_FIELD32((SDL_RSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_BUS_SAFETY_CTRL),\
        SDL_RSS_CTRL_RSS_BUS_SAFETY_CTRL_RSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
    /* enable the bus safety for RSS MBOX */
	HW_WR_FIELD32((SDL_RSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_CTRL),\
        SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
    /* Start diagnostic test case*/
    addr = SDL_RSS_CR4_MBOX_U_BASE;
    wr_data = (uint32_t)0x12345678U;
    /* enable fault injection for node with: double error correction, appropriate 32 bits of bus */
	HW_WR_FIELD32((SDL_RSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI),\
        SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_DED,\
		0x1U);
    HW_WR_FIELD32((SDL_RSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI),\
        SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI_RSS_MBOX_BUS_SAFETY_FI_DATA,\
         0x1);

    /* issue a bus transaction to node and read back */
    *(volatile uint32_t *)addr = wr_data;
}
#endif
/********************************************************************************************************
* API to Execute RED test on MSS MBOX
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3759
 */
int32_t SDL_MSS_MBOX_redExecute(SDL_MBOX_busSftyFiType fiType, SDL_MBOX_busSftyFiRedType redType )
{
    int32_t ret_val = SDL_EFAIL;
    if ((fiType< SDL_MBOX_FI_INVALID ) && (redType <SDL_MBOX_FI_TYPE_INVALID))
    {
        ret_val = SDL_PASS;
        /* enable the bus safety for DSS */
	    HW_WR_FIELD32((SDL_MSS_MBOX_BUS_CFG+SDL_MSS_CTRL_MSS_BUS_SAFETY_CTRL),\
        SDL_MSS_CTRL_MSS_BUS_SAFETY_CTRL_MSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
        /* enable the bus safety for MSS MBOX */
	    HW_WR_FIELD32((SDL_MSS_MBOX_BUS_CFG+SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_CTRL),\
        SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_MSS_MBOX_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
        /* Inject fault. */
        SDL_MSS_MBOX_busSftyFitype(fiType,redType);
    }
    else
    {
        ret_val = SDL_EBADARGS;
    }
    return ret_val;
}

/********************************************************************************************************
* API to Execute RED test on DSS MBOX
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3760
 */
int32_t SDL_DSS_MBOX_redExecute(SDL_MBOX_busSftyFiType fiType, SDL_MBOX_busSftyFiRedType redType )
{
    int32_t ret_val = SDL_EFAIL;
    if ((fiType< SDL_MBOX_FI_INVALID ) && (redType <SDL_MBOX_FI_TYPE_INVALID))
    {
        ret_val = SDL_PASS;
        /* enable the bus safety for DSS */
	    HW_WR_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
        /* enable the bus safety for DSS MBOX */
	    HW_WR_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_DSS_MBOX_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
        /* Inject fault. */
        SDL_DSS_MBOX_busSftyFitype(fiType,redType);
    }
    else
    {
        ret_val = SDL_EBADARGS;
    }
    return ret_val;
}

/********************************************************************************************************
* API to Execute RED test on RSS MBOX
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3761
 */
#if defined (SOC_AWR294X)
int32_t SDL_RSS_MBOX_redExecute(SDL_MBOX_busSftyFiType fiType, SDL_MBOX_busSftyFiRedType redType )
{
    int32_t ret_val = SDL_EFAIL;
    if ((fiType< SDL_MBOX_FI_INVALID ) && (redType <SDL_MBOX_FI_TYPE_INVALID))
    {
        ret_val = SDL_PASS;
        /* enable the bus safety for RSS */
	    HW_WR_FIELD32((SDL_RSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_BUS_SAFETY_CTRL),\
        SDL_RSS_CTRL_RSS_BUS_SAFETY_CTRL_RSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
        /* enable the bus safety for RSS MBOX */
	    HW_WR_FIELD32((SDL_RSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_CTRL),\
        SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_RSS_MBOX_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
        /* Inject fault. */
        SDL_RSS_MBOX_busSftyFitype(fiType,redType);
    }
    else
    {
        ret_val = SDL_EBADARGS;
    }
    return ret_val;
}
#endif
/********************************************************************************************************
* API to get SEC error status DSS MBOX
*********************************************************************************************************/
/* For Polling */
/**
 *  Design: PROC_SDL-3745
 */
uint32_t SDL_DSS_MBOX_secErrorStatus(void)
{
    return((uint32_t) HW_RD_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR),\
                        SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_DSS_MBOX_BUS_SAFETY_ERR_SEC));
}

/********************************************************************************************************
* API to get SEC error status RSS MBOX
*********************************************************************************************************/
/* For Polling */
/**
 *  Design: PROC_SDL-3746
 */
#if defined (SOC_AWR294X)
uint32_t SDL_RSS_MBOX_secErrorStatus(void)
{
    return((uint32_t) HW_RD_FIELD32((SDL_RSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR),\
                        SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_RSS_MBOX_BUS_SAFETY_ERR_SEC));

}
#endif
/********************************************************************************************************
* API to get DED error status DSS MBOX
*********************************************************************************************************/
/* For Polling */
/**
 *  Design: PROC_SDL-3748
 */
uint32_t SDL_DSS_MBOX_dedErrorStatus(void)
{
    return((uint32_t) HW_RD_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR),\
                        SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_DSS_MBOX_BUS_SAFETY_ERR_DED));
}
/********************************************************************************************************
* API to get DED error status RSS MBOX
*********************************************************************************************************/
/* For Polling */
/**
 *  Design: PROC_SDL-3749
 */
#if defined (SOC_AWR294X)
uint32_t SDL_RSS_MBOX_dedErrorStatus(void)
{
    return((uint32_t) HW_RD_FIELD32((SDL_RSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR),\
                        SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_RSS_MBOX_BUS_SAFETY_ERR_DED));
}
#endif
/********************************************************************************************************
* API to get RED error status DSS MBOX
*********************************************************************************************************/
/* For Polling */
/**
 *  Design: PROC_SDL-3751
 */
uint32_t SDL_DSS_MBOX_redErrorStatus(void)
{
    uint32_t cmdErr = 0U;
    uint32_t writeErr = 0U;
    uint32_t readErr = 0U;
    uint32_t writestepErr = 0U;
    uint32_t retval = 0U;
    cmdErr = ((uint32_t) HW_RD_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_CMD),\
                        SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_DSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_STAT));
    writeErr = ((uint32_t) HW_RD_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE),\
                        SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_STAT));
    readErr = ((uint32_t) HW_RD_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_READ),\
                        SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_READ_DSS_MBOX_BUS_SAFETY_ERR_STAT_READ_STAT));
    writestepErr = ((uint32_t) HW_RD_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_READ),\
                        SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_STAT));
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
* API to get RED error status RSS MBOX
*********************************************************************************************************/
/* For Polling */
/**
 *  Design: PROC_SDL-3752
 */
#if defined (SOC_AWR294X)
uint32_t SDL_RSS_MBOX_redErrorStatus(void)
{
    uint32_t cmdErr = 0U;
    uint32_t writeErr = 0U;
    uint32_t readErr = 0U;
    uint32_t writestepErr = 0U;
    uint32_t retval = 0U;
    cmdErr = ((uint32_t) HW_RD_FIELD32((SDL_RSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_CMD),\
                        SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_RSS_MBOX_BUS_SAFETY_ERR_STAT_CMD_STAT));
    writeErr = ((uint32_t) HW_RD_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE),\
                        SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE_STAT));
    readErr = ((uint32_t) HW_RD_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_READ),\
                        SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_READ_RSS_MBOX_BUS_SAFETY_ERR_STAT_READ_STAT));
    writestepErr = ((uint32_t) HW_RD_FIELD32((SDL_DSS_MBOX_BUS_CFG+SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP),\
                        SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP_STAT));
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
#endif


