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
 *  \file     sdl_dss_l3.c
 *
 *  \brief    This file contains the implementation of the APIs present in the
 *            device abstraction layer file of dss l3.
 *            This also contains some related macros.
 */




/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdbool.h>
#include <sdl/include/sdl_types.h>
#include <sdl/include/hw_types.h>
#include "sdl_dss_l3_hw.h"
#include "sdl_dss_l3.h"


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */


/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */
 /* \brief   This API is used to inject error in DSS L3 Bank A
 *
 * \param   fiType     indicates the Fi type
 *
 * \param   redType   indicates interface type
 *
 * \return  None
 *
 */
static void SDL_DSS_L3_BankA_busSftyFitype(SDL_DSS_L3_busSftyFiType fiType,\
                                       SDL_DSS_L3_busSftyFiRedType redType );

 /* \brief   This API is used to inject error in DSS L3 Bank B
 *
 * \param   fiType     indicates the Fi type
 *
 * \param   redType   indicates interface type
 *
 * \return  None
 *
 */
static void SDL_DSS_L3_BankB_busSftyFitype(SDL_DSS_L3_busSftyFiType fiType,\
                                       SDL_DSS_L3_busSftyFiRedType redType );

 /* \brief   This API is used to inject error in DSS L3 Bank C
 *
 * \param   fiType     indicates the Fi type
 *
 * \param   redType   indicates interface type
 *
 * \return  None
 *
 */
static void SDL_DSS_L3_BankC_busSftyFitype(SDL_DSS_L3_busSftyFiType fiType,\
                                       SDL_DSS_L3_busSftyFiRedType redType );

 /* \brief   This API is used to inject error in DSS L3 Bank D
 *
 * \param   fiType     indicates the Fi type
 *
 * \param   redType   indicates interface type
 *
 * \return  None
 *
 */
static void SDL_DSS_L3_BankD_busSftyFitype(SDL_DSS_L3_busSftyFiType fiType,\
                                       SDL_DSS_L3_busSftyFiRedType redType );


/* ========================================================================== */

/*                   Internal Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
/********************************************************************************************************
*   API to clear SEC error on DSS L3 Bank A
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3712
 */
void SDL_DSS_L3_BankA_secErrorClear(void)
{
    /* clear SEC error on node */
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_SEC, 0x0U);

    /* clear error */
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL),\
    SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_ERR_CLEAR,0x1U);

    /* clear SEC error in DSS_BUS_SAFETY_SEC_ERR_STAT0 */
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0),\
    SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_L3_BANKA,0x1U);
}

/********************************************************************************************************
*   API to get SEC error status  on DSS L3 Bank A
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3712
 */
uint32_t SDL_DSS_L3_BankA_secErrorStatus(void)
{
    return((uint32_t) HW_RD_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR),\
                         SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_DSS_L3_BANKA_BUS_SAFETY_ERR_SEC));
}

/********************************************************************************************************
*   API to clear SEC error on DSS L3 Bank B
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3712
 */
void SDL_DSS_L3_BankB_secErrorClear(void)
{
    /* clear SEC error on node */
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_SEC, 0x0U);

    /* clear error */
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL),\
    SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_ERR_CLEAR,0x1U);

    /* clear SEC error in DSS_BUS_SAFETY_SEC_ERR_STAT0 */
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0),\
    SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_L3_BANKB,0x1U);
}

/********************************************************************************************************
*   API to get SEC error status  on DSS L3 Bank B
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3713
 */
uint32_t SDL_DSS_L3_BankB_secErrorStatus(void)
{
    return((uint32_t) HW_RD_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR),\
                     SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_DSS_L3_BANKB_BUS_SAFETY_ERR_SEC));
}
/********************************************************************************************************
*   API to clear SEC error on DSS L3 Bank C
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3714
 */

void SDL_DSS_L3_BankC_secErrorClear(void)
{
    /* clear SEC error on node */
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_SEC, 0x0U);

    /* clear error */
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL),\
    SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_ERR_CLEAR,0x1U);

    /* clear SEC error in DSS_BUS_SAFETY_SEC_ERR_STAT0 */
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0),\
    SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_L3_BANKC,0x1U);
}

/********************************************************************************************************
*   API to get SEC error status  on DSS L3 Bank C
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3714
 */
uint32_t SDL_DSS_L3_BankC_secErrorStatus(void)
{
    return((uint32_t) HW_RD_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR),\
                     SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_DSS_L3_BANKC_BUS_SAFETY_ERR_SEC));
}

/********************************************************************************************************
*   API to clear SEC error on DSS L3 Bank D
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3715
 */

void SDL_DSS_L3_BankD_secErrorClear(void)
{
    /* clear SEC error on node */
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_SEC, 0x0U);

    /* clear error */
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL),\
    SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_ERR_CLEAR,0x1U);

    /* clear SEC error in DSS_BUS_SAFETY_SEC_ERR_STAT0 */
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0),\
    SDL_DSS_CTRL_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_BUS_SAFETY_SEC_ERR_STAT0_DSS_L3_BANKD,0x1U);
}

/********************************************************************************************************
*   API to get SEC error status  on DSS L3 Bank D
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3715
 */
uint32_t SDL_DSS_L3_BankD_secErrorStatus(void)
{
    return((uint32_t) HW_RD_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR),\
                     SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_DSS_L3_BANKD_BUS_SAFETY_ERR_SEC));
}

/********************************************************************************************************
*   API to clear DED error on DSS L3 Bank A
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3716
 */

void SDL_DSS_L3_BankA_dedErrorClear(void)
{
    /* clear DED error on node */
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_DED, 0x0U);
    /* clear error */
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL),\
    SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_ERR_CLEAR,0x1U);
}

/********************************************************************************************************
*   API to get DED error status  on DSS L3 Bank A
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3716
 */
uint32_t SDL_DSS_L3_BankA_dedErrorStatus(void)
{
    return((uint32_t) HW_RD_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR),\
                     SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_DSS_L3_BANKA_BUS_SAFETY_ERR_DED));
}
/********************************************************************************************************
*   API to clear DED error on DSS L3 Bank B
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3717
 */

void SDL_DSS_L3_BankB_dedErrorClear(void)
{
    /* clear DED error on node */
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_DED, 0x0U);
    /* clear error */
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL),\
    SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_ERR_CLEAR,0x1U);
}
/********************************************************************************************************
*   API to get DED error status  on DSS L3 Bank B
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3717
 */
uint32_t SDL_DSS_L3_BankB_dedErrorStatus(void)
{
    return((uint32_t) HW_RD_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR),\
                     SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_DSS_L3_BANKB_BUS_SAFETY_ERR_DED));
}
/********************************************************************************************************
*   API to clear DED error on DSS L3 Bank C
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3718
 */

void SDL_DSS_L3_BankC_dedErrorClear(void)
{
    /* clear DED error on node */
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_DED, 0x0U);
    /* clear error */
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL),\
    SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_ERR_CLEAR,0x1U);
}

/********************************************************************************************************
*   API to get DED error status  on DSS L3 Bank C
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3718
 */
uint32_t SDL_DSS_L3_BankC_dedErrorStatus(void)
{
    return((uint32_t) HW_RD_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR),\
                     SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_DSS_L3_BANKC_BUS_SAFETY_ERR_DED));
}

/********************************************************************************************************
*   API to clear DED error on DSS L3 Bank D
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3719
 */

void SDL_DSS_L3_BankD_dedErrorClear(void)
{
    /* clear DED error on node */
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_DED, 0x0U);
    /* clear error */
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL),\
    SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_ERR_CLEAR,0x1U);
}

/********************************************************************************************************
*   API to get DED error status  on DSS L3 Bank D
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3718
 */
uint32_t SDL_DSS_L3_BankD_dedErrorStatus(void)
{
    return((uint32_t) HW_RD_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR),\
                     SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_DSS_L3_BANKD_BUS_SAFETY_ERR_DED));
}

/********************************************************************************************************
*   API to clear RED error on DSS L3 Bank A
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3720
 */

void SDL_DSS_L3_BankA_redErrorClear(void)
{
    /* clear RED error on node */
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_GLOBAL_MAIN, 0x0U);
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_GLOBAL_SAFE, 0x0U);
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_MAIN, 0x0U);
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_SAFE, 0x0U);
}
/********************************************************************************************************
*   API to get RED error status on DSS L3 Bank A
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3720
 */

uint32_t SDL_DSS_L3_BankA_redErrorStatus(void)
{
    uint32_t cmdErr = 0U;
    uint32_t writeErr = 0U;
    uint32_t readErr = 0U;
    uint32_t writestepErr = 0U;
    uint32_t retval = 0U;
    cmdErr = ((uint32_t) HW_RD_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_CMD),\
                        SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_CMD_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_CMD_STAT));
    writeErr = ((uint32_t) HW_RD_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITE),\
                        SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITE_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITE_STAT));
    readErr = ((uint32_t) HW_RD_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_READ),\
                        SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_READ_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_READ_STAT));
    writestepErr = ((uint32_t) HW_RD_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITERESP),\
                        SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITERESP_STAT));
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
*   API to clear RED error on DSS L3 Bank B
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3721
 */

void SDL_DSS_L3_BankB_redErrorClear(void)
{
    /* clear RED error on node */
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_GLOBAL_MAIN, 0x0U);
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_GLOBAL_SAFE, 0x0U);
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_MAIN, 0x0U);
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_SAFE, 0x0U);
}
/********************************************************************************************************
*   API to get RED error status on DSS L3 Bank B
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3721
 */

uint32_t SDL_DSS_L3_BankB_redErrorStatus(void)
{
    uint32_t cmdErr = 0U;
    uint32_t writeErr = 0U;
    uint32_t readErr = 0U;
    uint32_t writestepErr = 0U;
    uint32_t retval = 0U;
    cmdErr = ((uint32_t) HW_RD_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_CMD),\
                        SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_CMD_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_CMD_STAT));
    writeErr = ((uint32_t) HW_RD_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITE),\
                        SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITE_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITE_STAT));
    readErr = ((uint32_t) HW_RD_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_READ),\
                        SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_READ_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_READ_STAT));
    writestepErr = ((uint32_t) HW_RD_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITERESP),\
                        SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITERESP_STAT));
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
*   API to clear RED error on DSS L3 Bank C
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3722
 */

void SDL_DSS_L3_BankC_redErrorClear(void)
{
    /* clear RED error on node */
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_GLOBAL_MAIN, 0x0U);
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_GLOBAL_SAFE, 0x0U);
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_MAIN, 0x0U);
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_SAFE, 0x0U);
}

/********************************************************************************************************
*   API to get RED error status on DSS L3 Bank C
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3722
 */

uint32_t SDL_DSS_L3_BankC_redErrorStatus(void)
{
    uint32_t cmdErr = 0U;
    uint32_t writeErr = 0U;
    uint32_t readErr = 0U;
    uint32_t writestepErr = 0U;
    uint32_t retval = 0U;
    cmdErr = ((uint32_t) HW_RD_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_CMD),\
                        SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_CMD_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_CMD_STAT));
    writeErr = ((uint32_t) HW_RD_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITE),\
                        SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITE_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITE_STAT));
    readErr = ((uint32_t) HW_RD_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_READ),\
                        SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_READ_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_READ_STAT));
    writestepErr = ((uint32_t) HW_RD_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITERESP),\
                        SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITERESP_STAT));
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
*   API to clear RED error on DSS L3 Bank D
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3723
 */

void SDL_DSS_L3_BankD_redErrorClear(void)
{
    /* clear RED error on node */
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_GLOBAL_MAIN, 0x0U);
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_GLOBAL_SAFE, 0x0U);
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_MAIN, 0x0U);
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI),\
    SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_SAFE, 0x0U);
}
/********************************************************************************************************
*   API to get RED error status on DSS L3 Bank D
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3723
 */

uint32_t SDL_DSS_L3_BankD_redErrorStatus(void)
{
    uint32_t cmdErr = 0U;
    uint32_t writeErr = 0U;
    uint32_t readErr = 0U;
    uint32_t writestepErr = 0U;
    uint32_t retval = 0U;
    cmdErr = ((uint32_t) HW_RD_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_CMD),\
                        SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_CMD_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_CMD_STAT));
    writeErr = ((uint32_t) HW_RD_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITE),\
                        SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITE_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITE_STAT));
    readErr = ((uint32_t) HW_RD_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_READ),\
                        SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_READ_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_READ_STAT));
    writestepErr = ((uint32_t) HW_RD_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITERESP),\
                        SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITERESP_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITERESP_STAT));    
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

/* Helper function to induce red error on Bank A*/
static void SDL_DSS_L3_BankA_busSftyFitype(SDL_DSS_L3_busSftyFiType fiType, SDL_DSS_L3_busSftyFiRedType redType )
{
    switch(fiType)
    {
        case SDL_DSS_L3_FI_MAIN:
 	    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_MAIN,\
		(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_DSS_L3_FI_SAFE:
 	    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_SAFE,\
		(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_DSS_L3_FI_GLOBAL_MAIN:
 	    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_GLOBAL_MAIN,\
		0x1U);
        break;
        case SDL_DSS_L3_FI_GLOBAL_SAFE:
	    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_GLOBAL_SAFE,\
		0x1U);
        break;
        default:
        /* do nothing */
        break;
    }
}
/* Helper function to induce red error on Bank B*/
static void SDL_DSS_L3_BankB_busSftyFitype(SDL_DSS_L3_busSftyFiType fiType, SDL_DSS_L3_busSftyFiRedType redType )
{
    switch(fiType)
    {
        case SDL_DSS_L3_FI_MAIN:
 	    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_MAIN,\
		(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_DSS_L3_FI_SAFE:
 	    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_SAFE,\
		(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_DSS_L3_FI_GLOBAL_MAIN:
 	    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_GLOBAL_MAIN,\
		0x1U);
        break;
        case SDL_DSS_L3_FI_GLOBAL_SAFE:
	    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_GLOBAL_SAFE,\
		0x1U);
        break;
        default:
        /* do nothing */
        break;
    }
}
/* Helper function to induce red error on Bank C*/
static void SDL_DSS_L3_BankC_busSftyFitype(SDL_DSS_L3_busSftyFiType fiType, SDL_DSS_L3_busSftyFiRedType redType )
{
    switch(fiType)
    {
        case SDL_DSS_L3_FI_MAIN:
 	    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_MAIN,\
		(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_DSS_L3_FI_SAFE:
 	    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_SAFE,\
		(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_DSS_L3_FI_GLOBAL_MAIN:
 	    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_GLOBAL_MAIN,\
		0x1U);
        break;
        case SDL_DSS_L3_FI_GLOBAL_SAFE:
	    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_GLOBAL_SAFE,\
		0x1U);
        break;
        default:
        /* do nothing */
        break;
    }
}
/* Helper function to induce red error on Bank D*/
static void SDL_DSS_L3_BankD_busSftyFitype(SDL_DSS_L3_busSftyFiType fiType, SDL_DSS_L3_busSftyFiRedType redType )
{
    switch(fiType)
    {
        case SDL_DSS_L3_FI_MAIN:
 	    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_MAIN,\
		(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_DSS_L3_FI_SAFE:
 	    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_SAFE,\
		(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_DSS_L3_FI_GLOBAL_MAIN:
 	    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_GLOBAL_MAIN,\
		0x1U);
        break;
        case SDL_DSS_L3_FI_GLOBAL_SAFE:
	    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_GLOBAL_SAFE,\
		0x1U);
        break;
        default:
        /* do nothing */
        break;
    }
}

/********************************************************************************************************
* API to Execute SEC test on DSS L3 Bank A
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3724
 */

void SDL_DSS_L3_BANKA_secExecute(void)
{
    volatile uint32_t addr, wr_data;
    /* enable the bus safety for DSS */
	HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
    /* enable the bus safety for BANK A */
	HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
    /* Start diagnostic test case*/
    addr = SDL_DSS_L3_BANKA_ADDRESS;
    wr_data = 0x12345678;

    /* enable fault injection for node with: single error correction, appropriate 32 bits of bus */
	HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_SEC,\
		0x1U);
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_DATA,\
         0x1);

    /* issue a bus transaction to node and read back */
    *(volatile uint32_t *)addr = wr_data;

}

/********************************************************************************************************
* API to Execute SEC test on DSS L3 Bank B
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3725
 */
void SDL_DSS_L3_BANKB_secExecute(void)
{
    volatile uint32_t addr, wr_data;
    /* enable the bus safety for DSS */
	HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
    /* enable the bus safety for BANK B */
	HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
    /* Start diagnostic test case*/
    addr = SDL_DSS_L3_BANKB_ADDRESS;
    wr_data = 0x12345678;

    /* enable fault injection for node with: single error correction, appropriate 32 bits of bus */
	HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_SEC,\
		0x1U);
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_DATA,\
         0x1);

    /* issue a bus transaction to node and read back */
    *(volatile uint32_t *)addr = wr_data;
}

/********************************************************************************************************
* API to Execute SEC test on DSS L3 Bank C
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3726
 */
void SDL_DSS_L3_BANKC_secExecute(void)
{
    volatile uint32_t addr, wr_data;
    /* enable the bus safety for DSS */
	HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
    /* enable the bus safety for BANK C */
	HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
    /* Start diagnostic test case*/
    addr = SDL_DSS_L3_BANKC_ADDRESS;
    wr_data = 0x12345678;

    /* enable fault injection for node with: single error correction, appropriate 32 bits of bus */
	HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_SEC,\
		0x1U);
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_DATA,\
         0x1);

    /* issue a bus transaction to node and read back */
    *(volatile uint32_t *)addr = wr_data;
}

/********************************************************************************************************
* API to Execute SEC test on DSS L3 Bank D
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3727
 */
void SDL_DSS_L3_BANKD_secExecute(void)
{
    volatile uint32_t addr, wr_data;
    /* enable the bus safety for DSS */
	HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
    /* enable the bus safety for BANK D */
	HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
    /* Start diagnostic test case*/
    addr = SDL_DSS_L3_BANKD_ADDRESS;
    wr_data = 0x12345678;

    /* enable fault injection for node with: single error correction, appropriate 32 bits of bus */
	HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_SEC,\
		0x1U);
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_DATA,\
         0x1);

    /* issue a bus transaction to node and read back */
    *(volatile uint32_t *)addr = wr_data;
}

/********************************************************************************************************
* API to Execute DED test on DSS L3 Bank A
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3728
 */

void SDL_DSS_L3_BANKA_dedExecute(void)
{
    volatile uint32_t addr, wr_data;
    /* enable the bus safety for DSS */
	HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
    /* enable the bus safety for BANK A */
	HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
    /* Start diagnostic test case*/
    addr = SDL_DSS_L3_BANKA_ADDRESS;
    wr_data = 0x12345678;

    /* enable fault injection for node with: double error correction, appropriate 32 bits of bus */
	HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_DED,\
		0x1U);
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI_DSS_L3_BANKA_BUS_SAFETY_FI_DATA,\
         0x1);

    /* issue a bus transaction to node and read back */
    *(volatile uint32_t *)addr = wr_data;
}

/********************************************************************************************************
* API to Execute DED test on DSS L3 Bank B
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3729
 */

void SDL_DSS_L3_BANKB_dedExecute(void)
{
    volatile uint32_t addr, wr_data;
    /* enable the bus safety for DSS */
	HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
    /* enable the bus safety for BANK B */
	HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
    /* Start diagnostic test case*/
    addr = SDL_DSS_L3_BANKB_ADDRESS;
    wr_data = 0x12345678;
    /* enable fault injection for node with: double error correction, appropriate 32 bits of bus */
	HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_DED,\
		0x1U);
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI_DSS_L3_BANKB_BUS_SAFETY_FI_DATA,\
         0x1);

    /* issue a bus transaction to node and read back */
    *(volatile uint32_t *)addr = wr_data;
}

/********************************************************************************************************
* API to Execute DED test on DSS L3 Bank C
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3730
 */
void SDL_DSS_L3_BANKC_dedExecute(void)
{
    volatile uint32_t addr, wr_data;
    /* enable the bus safety for DSS */
	HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
    /* enable the bus safety for BANK C */
	HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
    /* Start diagnostic test case*/
    addr = SDL_DSS_L3_BANKC_ADDRESS;
    wr_data = 0x12345678;
    /* enable fault injection for node with: double error correction, appropriate 32 bits of bus */
	HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_DED,\
		0x1U);
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI_DSS_L3_BANKC_BUS_SAFETY_FI_DATA,\
         0x1);

    /* issue a bus transaction to node and read back */
    *(volatile uint32_t *)addr = wr_data;
}

/********************************************************************************************************
* API to Execute DED test on DSS L3 Bank D
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3731
 */
void SDL_DSS_L3_BANKD_dedExecute(void)
{
    volatile uint32_t addr, wr_data;
    /* enable the bus safety for DSS */
	HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
    /* enable the bus safety for BANK D */
	HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
    /* Start diagnostic test case*/
    addr = SDL_DSS_L3_BANKD_ADDRESS;
    wr_data = 0x12345678;
    /* enable fault injection for node with: double error correction, appropriate 32 bits of bus */
	HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_DED,\
		0x1U);
    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI),\
        SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI_DSS_L3_BANKD_BUS_SAFETY_FI_DATA,\
         0x1);

    /* issue a bus transaction to node and read back */
    *(volatile uint32_t *)addr = wr_data;
}

/********************************************************************************************************
* API to Execute RED test on DSS L3 Bank A
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3732
 */

int32_t SDL_DSS_L3_BANKA_redExecute(SDL_DSS_L3_busSftyFiType fiType, SDL_DSS_L3_busSftyFiRedType redType )
{
    int32_t ret_val = SDL_EFAIL;
    if ((fiType< SDL_DSS_L3_FI_INVALID ) && (redType <SDL_DSS_L3_FI_TYPE_INVALID))
    {
        /* enable the bus safety for DSS */
	    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
        /* enable the bus safety for BANK A */
	    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
        /* Inject fault. */
        SDL_DSS_L3_BankA_busSftyFitype(fiType,redType);
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EBADARGS;
    }
    return ret_val;
}

/********************************************************************************************************
* API to Execute RED test on DSS L3 Bank B
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3733
 */

int32_t SDL_DSS_L3_BANKB_redExecute(SDL_DSS_L3_busSftyFiType fiType, SDL_DSS_L3_busSftyFiRedType redType )
{
    int32_t ret_val = SDL_EFAIL;
    if ((fiType< SDL_DSS_L3_FI_INVALID ) && (redType <SDL_DSS_L3_FI_TYPE_INVALID))
    {
        /* enable the bus safety for DSS */
	    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
        /* enable the bus safety for BANK B */
	    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
        /* Inject fault. */
        SDL_DSS_L3_BankB_busSftyFitype(fiType,redType);
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EBADARGS;
    }
    return ret_val;
}

/********************************************************************************************************
* API to Execute RED test on DSS L3 Bank C
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3734
 */

int32_t SDL_DSS_L3_BANKC_redExecute(SDL_DSS_L3_busSftyFiType fiType, SDL_DSS_L3_busSftyFiRedType redType )
{
    int32_t ret_val = SDL_EFAIL;
    if ((fiType< SDL_DSS_L3_FI_INVALID ) && (redType <SDL_DSS_L3_FI_TYPE_INVALID))
    {
        /* enable the bus safety for DSS */
	    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
        /* enable the bus safety for BANCK C */
	    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
        /* Inject fault. */
        SDL_DSS_L3_BankC_busSftyFitype(fiType,redType);
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EBADARGS;
    }
    return ret_val;
}

/********************************************************************************************************
* API to Execute RED test on DSS L3 Bank D
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3735
 */
int32_t SDL_DSS_L3_BANKD_redExecute(SDL_DSS_L3_busSftyFiType fiType, SDL_DSS_L3_busSftyFiRedType redType )
{
    int32_t ret_val = SDL_EFAIL;
    if ((fiType< SDL_DSS_L3_FI_INVALID ) && (redType <SDL_DSS_L3_FI_TYPE_INVALID))
    {
        /* enable the bus safety for DSS */
	    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL_DSS_BUS_SAFETY_CTRL_ENABLE, \
		0X7U);
        /* enable the bus safety for BANK D */
	    HW_WR_FIELD32((SDL_DSS_L3_CTRL+SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL),\
        SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL_ENABLE,\
		0x7U);
        /* Inject fault. */
        SDL_DSS_L3_BankD_busSftyFitype(fiType,redType);
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EBADARGS;
    }
    return ret_val;
}






