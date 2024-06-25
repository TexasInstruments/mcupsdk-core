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

#include "sdl_rti.h"


/********************************************************************************************************
*   API for configuring the RTI module
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-1166
 */

int32_t SDL_RTI_config(SDL_RTI_InstanceType      InstanceType,
                       const SDL_RTI_configParms  *pConfig
                       )
{
    int32_t      sdlResult = SDL_PASS;
    uint32_t     baseAddr;

    /* This will assign base address of given Instance type    */
    sdlResult = SDL_RTI_getBaseaddr(InstanceType, &baseAddr);

    if((pConfig != NULL) && (sdlResult == SDL_PASS))
    {

        /* This will assign base address of given Instance type    */


        if( SDL_RTI_chkWindowSize(pConfig->SDL_RTI_dwwdWindowSize) == SDL_PASS)
        {
           SDL_RTI_writeWinSz(baseAddr, pConfig->SDL_RTI_dwwdWindowSize);
        }

        SDL_RTI_setPreload(baseAddr, pConfig->SDL_RTI_dwwdPreloadVal);

        sdlResult = SDL_RTI_chkReaction(pConfig->SDL_RTI_dwwdReaction);

        if (sdlResult == SDL_PASS)
        {
            SDL_RTI_writeReaction(baseAddr, pConfig->SDL_RTI_dwwdReaction);
        }
        else
        {
            sdlResult = SDL_EFAIL;
        }
    }
    else
    {
        sdlResult = SDL_EBADARGS;
    }

    return (sdlResult);
}



/**
 *   Design: PROC_SDL-1173
 */

int32_t SDL_RTI_verifyConfig(SDL_RTI_InstanceType     InstanceType,
                             SDL_RTI_configParms    *pConfig)
{
    int32_t    sdlResult;
    uint32_t   reaction_rd, preload_rd, winSz_rd, preload, baseAddr;

    /* This will assign base address of given Instance type    */
    sdlResult = SDL_RTI_getBaseaddr(InstanceType, &baseAddr);

    if ((pConfig != NULL) && (sdlResult == SDL_PASS))
    {
        SDL_RTI_getPreload(baseAddr, &preload_rd);

        preload = (pConfig->SDL_RTI_dwwdPreloadVal >>  ((uint32_t) RTI_DWWDPRLD_MULTIPLIER_SHIFT));

        SDL_RTI_getWindowSize(baseAddr, &winSz_rd);

        reaction_rd = SDL_RTI_readReaction(baseAddr);

        /* Checking with the structure parameters with the parameters read from registers */
        if ((pConfig->SDL_RTI_dwwdReaction   == reaction_rd) &&
            (preload                            == preload_rd)  &&
            (pConfig->SDL_RTI_dwwdWindowSize == winSz_rd) )
        {
            sdlResult = SDL_PASS;
        }
        else
        {
            sdlResult = SDL_EFAIL;
        }
    }
    else
    {
        sdlResult = SDL_EBADARGS;
    }

    return (sdlResult);
}



/********************************************************************************************************
*   API for Initializing the RTI module
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-1512
 */

int32_t SDL_RTI_start(SDL_RTI_InstanceType InstanceType)
{
    int32_t     sdlResult = SDL_PASS;
    uint32_t     baseAddr;


    /* This will assign base address of given Instance type    */
    sdlResult = SDL_RTI_getBaseaddr(InstanceType, &baseAddr);

    if(sdlResult == SDL_PASS)
    {
        /* Enable DWWD by writing pre-defined value '0xA98559DA' to RTIDWDCTRL */
        HW_WR_REG32(baseAddr + RTI_RTIDWDCTRL,
                    RTI_RTIDWDCTRL_DWDCTRL_ENABLE);
        sdlResult = SDL_PASS;
    }
    else
    {
        sdlResult = SDL_EBADARGS;
    }

    return (sdlResult);
}



/********************************************************************************************************
*   This API is for servicing the DWWD
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-1515
 */

int32_t  SDL_RTI_service(SDL_RTI_InstanceType     InstanceType)
{
    int32_t     sdlResult;
    uint32_t     baseAddr;

    /* This will assign bass address of given Instance type    */
    sdlResult = SDL_RTI_getBaseaddr(InstanceType, &baseAddr);

    if (sdlResult == SDL_PASS)
    {
        /* First write operation 0xE51A */
        HW_WR_FIELD32(baseAddr + RTI_RTIWDKEY,
                      RTI_RTIWDKEY_WDKEY,
                      RTI_RTIWDKEY_WDKEY_FIRST_WRITE);
        /* Second write operation 0xA35C */
        HW_WR_FIELD32(baseAddr + RTI_RTIWDKEY,
                      RTI_RTIWDKEY_WDKEY,
                      RTI_RTIWDKEY_WDKEY_SECOND_WRITE);

        sdlResult = SDL_PASS;
    }
    else
    {
        sdlResult = SDL_EBADARGS;
    }
    return (sdlResult);
}



/********************************************************************************************************
*   This API is for clearing the Interrupts
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-1514
 */

int32_t SDL_RTI_clearStatus(SDL_RTI_InstanceType     InstanceType, uint32_t status)
{
    int32_t sdlResult = SDL_EFAIL;
    uint32_t     baseAddr;

    /* This will assign bass address of given Instance type    */
    sdlResult = SDL_RTI_getBaseaddr(InstanceType, &baseAddr);

    if ((status == STATUS_VLD) && (sdlResult == SDL_PASS))
    {
        /* If status value == 1, then it clears all the flags to 0 and Clearing
            of the status flags will deassert the non-maskable interrupt generated
            due to violation of the DWWD.
            0h = Leaves the current value unchanged. */

        HW_WR_REG32(baseAddr + RTI_RTIWDSTATUS, status);
        sdlResult = SDL_PASS;
    }
    else
    {
        sdlResult = SDL_EBADARGS;
    }
    return (sdlResult);
}



/********************************************************************************************************
*   API to READ the status of the RTI module
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-1513
 */

int32_t SDL_RTI_getStatus(SDL_RTI_InstanceType InstanceType, uint32_t *pStatus)
{
    int32_t sdlResult = SDL_EFAIL;
    uint32_t    baseAddr;

    /* This will assign bass address of given Instance type    */
    sdlResult = SDL_RTI_getBaseaddr(InstanceType, &baseAddr);

    if(sdlResult == SDL_PASS)
    {
        if (pStatus != (NULL_PTR))
        {
            *pStatus = HW_RD_REG32(baseAddr + RTI_RTIWDSTATUS);
            sdlResult = SDL_PASS;
        }
        else
        {
            sdlResult = SDL_EBADARGS;
        }
    }
    else
    {
        sdlResult = SDL_EBADARGS;
    }
      return (sdlResult);
}



/********************************************************************************************************
*   This funtion will provide the application to read the static registers
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-1172
 */

int32_t SDL_RTI_readStaticRegs(SDL_RTI_InstanceType         InstanceType,
                                    SDL_RTI_staticRegs         *pStaticRegs
                                    )
{
    int32_t     sdlResult;
    uint32_t     baseAddr;

    /* This will assign bass address of given Instance type    */
    sdlResult = SDL_RTI_getBaseaddr(InstanceType, &baseAddr);

    if(sdlResult == SDL_PASS)
    {
        if (pStaticRegs != (NULL_PTR))
        {
            pStaticRegs->RTI_DWDCTRL = HW_RD_REG32(baseAddr + RTI_RTIDWDCTRL);
            pStaticRegs->RTI_DWDPRLD = HW_RD_REG32(baseAddr + RTI_RTIDWDPRLD);
            pStaticRegs->RTI_WWDRXNCTRL = HW_RD_REG32(baseAddr + RTI_RTIDWWDRXNCTRL);
            pStaticRegs->RTI_WWDSIZECTRL = HW_RD_REG32(baseAddr + RTI_RTIDWWDSIZECTRL);
            sdlResult = SDL_PASS;
        }
        else
        {
            sdlResult = SDL_EBADARGS;
        }
    }
    else
    {
        sdlResult = SDL_EBADARGS;
    }

    return (sdlResult);
}
