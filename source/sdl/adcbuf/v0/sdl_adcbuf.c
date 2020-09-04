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
 *  \file     sdl_adcbuf.c
 *
 *  \brief    This file contains the implementation of the APIs present in the
 *            device abstraction layer file of adcbuf.
 *            This also contains some related macros.
 */
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdbool.h>
#include <sdl/include/sdl_types.h>
#include <sdl/include/hw_types.h>
#include "sdl_adcbuf_hw.h"
#include "sdl_adcbuf.h"
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */
/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */
 /* \brief   This API is used to inject error in ADCBUF_RD
 *
 * \param   fiType     indicates the Fi type
 *
 * \param   redType   indicates interface type
 *
 */
static void SDL_ADCBUF_RD_busSftyFitype(SDL_ADCBUF_busSftyFiType fiType, \
                                        SDL_ADCBUF_busSftyFiRedType redType );

 /* \brief   This API is used to inject  error in ADCBUF_WR
 *
 * \param   fiType     indicates the Fi type
 *
 * \param   redType   indicates interface type
 *
 */
static void SDL_ADCBUF_WR_busSftyFitype(SDL_ADCBUF_busSftyFiType fiType,\
                                        SDL_ADCBUF_busSftyFiRedType redType );


/* ========================================================================== */

/*                   Internal Global Variables                                */
/* ========================================================================== */
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
/********************************************************************************************************
*   API for RED Error Clear On ADCBUF RD
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3708
 */
void SDL_ADCBUF_rdRedErrorClear(void)
{
    /* Clear fault injection */
    HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI),\
    SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_GLOBAL_MAIN, 0x0U);
    HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI),\
    SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_GLOBAL_SAFE, 0x0U);
    HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI),\
    SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_MAIN, 0x0U);
    HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI),\
    SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_SAFE, 0x0U);

}

/********************************************************************************************************
* API to get RED error status On ADCBUF RD
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3708
 */
uint32_t SDL_ADCBUF_rdRedErrorStatus(void)
{
    uint32_t cmdErr = 0U;
    uint32_t readErr = 0U;
    uint32_t retval = 0U;
    cmdErr = ((uint32_t) HW_RD_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_CMD),\
                        SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_CMD_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_CMD_STAT));
    readErr = ((uint32_t) HW_RD_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_READ),\
                        SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_READ_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_READ_STAT));
    if((cmdErr!=0U) || (readErr!=0U) )
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
*   API for RED Error Clear On ADCBUF WR
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3709
 */
void SDL_ADCBUF_wrRedErrorClear(void)
{
    /* Clear fault injection */
    HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI),\
    SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_GLOBAL_MAIN, 0x0U);
    HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI),\
    SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_GLOBAL_SAFE, 0x0U);
    HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI),\
    SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_MAIN, 0x0U);
    HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI),\
    SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_SAFE, 0x0U);
}
/********************************************************************************************************
* API to get RED error status On ADCBUF WR
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3709
 */
uint32_t SDL_ADCBUF_wrRedErrorStatus(void)
{
    uint32_t cmdErr = 0U;
    uint32_t writeErr = 0U;
    uint32_t writestepErr = 0U;
    uint32_t retval = 0U;
    cmdErr = ((uint32_t) HW_RD_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_CMD),\
                        SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_READ_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_READ_STAT));
    writeErr = ((uint32_t) HW_RD_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITE),\
                        SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITE_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITE_STAT));
    writestepErr = ((uint32_t) HW_RD_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITERESP),\
                        SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITERESP_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITERESP_STAT));
    if((cmdErr!=0U) || (writeErr!=0U) || (writestepErr !=0U) )
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
*   API for DED Error Clear On ADCBUF WR
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3710
 */
void SDL_ADCBUF_wrDedErrorClear(void)
{
    /* clear SEC error on node */
    HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI),\
    SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_DED, 0x0U);
    /* clear error */
    HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL),\
    SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_ERR_CLEAR,0x1U);

}
/********************************************************************************************************
*   API to get  DED Error status On ADCBUF WR
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3710
 */
uint32_t SDL_ADCBUF_wrDedErrorStatus(void)
{
    return((uint32_t) HW_RD_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR),\
                        SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_RSS_ADCBUF_WR_BUS_SAFETY_ERR_DED));
}
/********************************************************************************************************
*   API for SEC Error Clear On ADCBUF WR
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3711
 */
void SDL_ADCBUF_wrSecErrorClear(void)
{
    /* clear SEC error on node */
    HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI),\
    SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_SEC, 0x0U);
    /* clear error */
    HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL),\
    SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_ERR_CLEAR,0x1U);
    /* clear SEC error in DSS_BUS_SAFETY_SEC_ERR_STAT0 */
    HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0),\
    SDL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0_RSS_BUS_SAFETY_SEC_ERR_STAT0_ADC_BUF_WR,0x1U);
}
/********************************************************************************************************
*   API to get  SEC Error status On ADCBUF WR
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3711
 */
uint32_t SDL_ADCBUF_wrSecErrorStatus(void)
{
    return((uint32_t) HW_RD_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR),\
                        SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_RSS_ADCBUF_WR_BUS_SAFETY_ERR_SEC));
}

/* Helper function to induce RED error on ADCBUF RD*/
static void SDL_ADCBUF_RD_busSftyFitype(SDL_ADCBUF_busSftyFiType fiType, SDL_ADCBUF_busSftyFiRedType redType )
{
    switch(fiType)
    {
        case SDL_ADCBUF_FI_MAIN:
 	    HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI),\
        SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_MAIN,\
		(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_ADCBUF_FI_SAFE:

 	    HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI),\
        SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_SAFE,\
		(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_ADCBUF_FI_GLOBAL_MAIN:
 	    HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI),\
        SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_GLOBAL_MAIN,\
		0x1U);
        break;
        case SDL_ADCBUF_FI_GLOBAL_SAFE:
	    HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI),\
        SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI_RSS_ADCBUF_RD_BUS_SAFETY_FI_GLOBAL_SAFE,\
		0x1U);
        break;
        default:
        /* do nothing */
        break;
    }
}
/* Helper function to induce RED error on ADCBUF WR*/
static void SDL_ADCBUF_WR_busSftyFitype(SDL_ADCBUF_busSftyFiType fiType, SDL_ADCBUF_busSftyFiRedType redType )
{
    switch(fiType)
    {
        case SDL_ADCBUF_FI_MAIN:
 	    HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI),\
        SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_MAIN,\
		(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_ADCBUF_FI_SAFE:

 	    HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI),\
        SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_SAFE,\
		(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_ADCBUF_FI_GLOBAL_MAIN:
 	    HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI),\
        SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_GLOBAL_MAIN,\
		0x1U);
        break;
        case SDL_ADCBUF_FI_GLOBAL_SAFE:
	    HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI),\
        SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_GLOBAL_SAFE,\
		0x1U);
        break;
        default:
        /* do nothing */
        break;
    }
}
/********************************************************************************************************
*   API for RED Test On ADCBUF RD
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3707
 */
int32_t SDL_ADCBUF_RD_redExecute(SDL_ADCBUF_busSftyFiType fiType, SDL_ADCBUF_busSftyFiRedType redType )
{
    int32_t ret_val = SDL_EFAIL;
    if ((fiType< SDL_ADCBUF_FI_INVALID ) && ((redType ==SDL_ADCBUF_MAIN_CMD_INTERFACE) || \
                                             (redType ==SDL_ADCBUF_MAIN_READ_INTERFACE)))
    {
        /* enable the bus safety for RSS */
	    HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_BUS_SAFETY_CTRL),\
        SDL_RSS_CTRL_RSS_BUS_SAFETY_CTRL_RSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);

        /* enable the bus safety for ADC BUF RD */
	    HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL),\
        SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
       /* Inject fault. */
        SDL_ADCBUF_RD_busSftyFitype(fiType,redType);
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EBADARGS;
    }
    return ret_val;
}

/********************************************************************************************************
*   API for RED Test On ADCBUF WR
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3706
 */

int32_t SDL_ADCBUF_WR_redExecute(SDL_ADCBUF_busSftyFiType fiType, SDL_ADCBUF_busSftyFiRedType redType )
{
    int32_t ret_val = SDL_EFAIL;
    if ((fiType< SDL_ADCBUF_FI_INVALID ) && (redType <SDL_ADCBUF_MAIN_READ_INTERFACE))
    {
        /* enable the bus safety for RSS */
	    HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_BUS_SAFETY_CTRL),\
        SDL_RSS_CTRL_RSS_BUS_SAFETY_CTRL_RSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
        /* enable the bus safety for ADC BUF WR */
	    HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL),\
        SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
       /* Inject fault. */
        SDL_ADCBUF_WR_busSftyFitype(fiType,redType);
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EBADARGS;
    }
    return ret_val;
}

/********************************************************************************************************
*   API for SEC Test On ADCBUF WR
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3704
 */

void SDL_ADCBUF_WR_secExecute(void)
{
    volatile uint32_t addr, wr_data;
    /* Ennable write */
	HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_DMMSWINT1),\
    SDL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMADCBUFWREN,0x1U);

    /* enable Bus Safety for the RSS subsystem globally */
	HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_BUS_SAFETY_CTRL),\
    SDL_RSS_CTRL_RSS_BUS_SAFETY_CTRL_RSS_BUS_SAFETY_CTRL_ENABLE, \
	0X7U);

    /* enable the bus safety for ADC BUF WR */
	HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL),\
    SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_ENABLE,\
	0x7U);

    /* Start diagnostic test case*/
    addr = SDL_RSS_ADCBUF_WRITE_U_BASE;
    wr_data = 0x12345678U;

    /* enable fault injection for node with: single error correction, appropriate 32 bits of bus */
	HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI),\
    SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_SEC,\
	0x1U);
	HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI),\
    SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_DATA,\
	0x1U);

    /* issue a bus transaction to node and read back */
    *(volatile uint32_t *)addr = wr_data;
	HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_DMMSWINT1),\
    SDL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMADCBUFWREN,0x0U);
}

/********************************************************************************************************
*   API for DED Test On ADCBUF WR
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3705
 */

void SDL_ADCBUF_WR_dedExecute(void)
{
    uint32_t addr, wr_data;
	HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_DMMSWINT1),\
    SDL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMADCBUFWREN,0x1U);


    /* enable Bus Safety for the RSS subsystem globally */
	HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_BUS_SAFETY_CTRL),\
    SDL_RSS_CTRL_RSS_BUS_SAFETY_CTRL_RSS_BUS_SAFETY_CTRL_ENABLE, \
	0X7U);

    /* enable the bus safety for ADC BUF WR */
	HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL),\
    SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL_ENABLE,\
	0X7U);
    /* Start diagnostic test case*/
    addr = SDL_RSS_ADCBUF_WRITE_U_BASE;
    wr_data = 0x12345678U;

    /* enable fault injection for node with: double error detection, appropriate 32 bits of the bus */
	HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI),\
    SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_DED,\
	0x1U);
	HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI),\
    SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI_RSS_ADCBUF_WR_BUS_SAFETY_FI_DATA,\
	0x1U);

    /*issue a bus transaction to node via appropriate 32 bits of bus, and read back*/
    *(volatile uint32_t *) addr = wr_data;

	HW_WR_FIELD32((SDL_ADCBUF_CFG+SDL_RSS_CTRL_DMMSWINT1),\
    SDL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMADCBUFWREN,0x0U);
}

