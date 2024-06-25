/* Copyright (c) 2022-23 Texas Instruments Incorporated
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
 *  \file     ecc_bus_safety_polling.c
 *
 *  \brief    This file contains Ecc Bus Safety for DSS MDO example code in polling method.
 *
 *  \details  ECC_BUS_SAFETY APP
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include "ecc_bus_safety.h"
/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/
#if defined (SOC_AM273X)
#if defined (SUBSYS_DSS)
/* Generalized function for sec test in polling methode */
static int32_t SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(uint32_t busSftyNode ,uint32_t addr );
/* Generalized function for ded test in polling methode */
static int32_t SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(uint32_t busSftyNode ,uint32_t addr);
/* Generalized function for red test in polling methode */
static int32_t SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(uint32_t busSftyNode, \
                        SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType);
#endif
#endif

/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/

/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/

/*===========================================================================*/
/*                   Local Function definitions                              */
/*===========================================================================*/
#if defined (SOC_AM273X)
#if defined (SUBSYS_DSS)

/********************************************************************************************************
*   For Node DSS_MDO_FIFO
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO ,SDL_DSS_MDO_FIFO_U_BASE ));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO ,SDL_DSS_MDO_FIFO_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO,\
            SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For DSS SEC
*********************************************************************************************************/
static int32_t SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(uint32_t busSftyNode ,uint32_t addr )
{
    int32_t retval = SDL_EFAIL;
    uint32_t timeout = SDL_ECC_BUS_SAFETY_TIMEOUT;
    uint32_t writeData = 0x1234567U;
    uint32_t status = 0U;
    retval = SDL_ECC_BUS_SAFETY_DSS_secExecute(busSftyNode,addr,writeData);
    if(retval !=SDL_PASS )
    {
        retval = SDL_EFAIL;
    }
    else
    {
        /* Wait for test to complete/timeout. */
        while((status == 0U) && (timeout!=0U))
        {
            retval =SDL_ECC_BUS_SAFETY_DSS_getSecErrorStatus(busSftyNode,&status);
            if(retval !=SDL_PASS )
            {
                timeout--;
            }
            else
            {
                break;
            }

        }
        if(status!=0U)
        {
            retval = SDL_PASS;
            SDL_ECC_BUS_SAFETY_DSS_secErrorClear(busSftyNode );
        }
        else
        {
            retval = SDL_EFAIL;
        }
    }
    return retval;
}

/********************************************************************************************************
*   For DSS DED
*********************************************************************************************************/

static int32_t SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(uint32_t busSftyNode ,uint32_t addr)
{
    int32_t retval = SDL_EFAIL;
    uint32_t timeout = SDL_ECC_BUS_SAFETY_TIMEOUT;
    uint32_t writeData = 0x1234567U;
    uint32_t status = 0U;
    retval = SDL_ECC_BUS_SAFETY_DSS_dedExecute(busSftyNode,addr,writeData);
    if(retval !=SDL_PASS )
    {
        retval = SDL_EFAIL;
    }
    else
    {
        /* Wait for test to complete/timeout. */
        while((status == 0U) && (timeout!=0U))
        {
            retval =SDL_ECC_BUS_SAFETY_DSS_getDedErrorStatus(busSftyNode,&status);
            if(retval !=SDL_PASS )
            {
                timeout--;
            }
            else
            {
                break;
            }

        }
        if(status!=0U)
        {
            retval = SDL_PASS;
            SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(busSftyNode);
        }
        else
        {
            retval = SDL_EFAIL;
        }
    }
    return retval;
}

/********************************************************************************************************
*   For DSS RED
*********************************************************************************************************/
static int32_t SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(uint32_t busSftyNode, SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType)
{
    int32_t retval = SDL_EFAIL;
    uint32_t timeout = SDL_ECC_BUS_SAFETY_TIMEOUT;
    uint32_t status = 0U;
    retval= SDL_ECC_BUS_SAFETY_DSS_redExecute(busSftyNode, fiType, redType);
    if(retval !=SDL_PASS )
    {
        retval = SDL_EFAIL;
    }
    else
    {
        /* Wait for test to complete/timeout. */
        while((status == 0U) && (timeout!=0U))
        {
            retval =SDL_ECC_BUS_SAFETY_DSS_getRedErrorStatus(busSftyNode,&status);
            if(retval !=SDL_PASS )
            {
                timeout--;
            }
            else
            {
                break;
            }

        }
        if(status!=0U)
        {
            retval = SDL_PASS;
            SDL_ECC_BUS_SAFETY_DSS_redErrorClear(busSftyNode);
        }
        else
        {
            retval = SDL_EFAIL;
        }
    }
    return retval;
}

#endif
#endif
/* Nothing past this point */
