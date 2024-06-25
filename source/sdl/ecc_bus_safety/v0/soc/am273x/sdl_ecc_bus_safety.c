/*
 *   Copyright (c) 2022-23 Texas Instruments Incorporated
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
 *  \file     sdl_ecc_bus_safety.c
 *
 *  \brief    This file contains the implementation of the APIs present in the
 *            device abstraction layer file of ECC BUS SAFETY.
 *            This also contains some related macros.
 */




/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdbool.h>
#include <sdl/include/sdl_types.h>
#include <sdl/include/hw_types.h>
#include <sdl/ecc_bus_safety/v0/sdl_ecc_bus_safety_hw.h>
#include <sdl/ecc_bus_safety/v0/sdl_ecc_bus_safety.h>
#include <sdl/ecc_bus_safety/v0/soc/am273x/sdl_ecc_bus_safety_soc.h>
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */


/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */
#if defined (SUBSYS_MSS)
/**
*  \brief   This API is used to inject error in Node
*
* \param   fiType       indicates the Fi type
*
* \param   redType      indicates interface type
*
* \param   baseAddrOffst pointer to hold the base and offset address
*/
static void SDL_ECC_BUS_SAFETY_MSS_busSftyFitypeSet(SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType,
                                               SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S *baseAddrOffst);

/**
*  \brief   This API is used enble the MSS ECC bus Safety
*
*/
static void SDL_ECC_BUS_SAFETY_MSS_SAFETY_CTRL_enable (void);

/**
*  \brief   This API is used to get the offset address of MSS node
*
* \param   busSftyNode Node identifier
*
* \param   baseAddrOffst pointer to hold the base and offset address
*/
static int32_t SDL_ECC_BUS_SAFETY_MSS_getRegOffset(uint32_t busSftyNode , SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S *baseAddrOffst);


#endif

#if defined (SUBSYS_DSS)

 /**
 *  \brief   This API is used to inject error in Node
 *
 * \param   fiType       indicates the Fi type
 *
 * \param   redType      indicates interface type
 *
* \param   baseAddrOffst pointer to hold the base and offset address
 */
static void SDL_ECC_BUS_SAFETY_DSS_busSftyFitypeSet(SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType,
                                                SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S *baseAddrOffst);

 /**
 *  \brief   This API is used enble the DSS ECC bus Safety
 *
 */
static void SDL_ECC_BUS_SAFETY_DSS_SAFETY_CTRL_enable (void);

 /**
 *  \brief   This API is used to get the offset address of DSS node
 *
 * \param   busSftyNode Node identifier
 *
* \param   baseAddrOffst pointer to hold the base and offset address
 */
static int32_t SDL_ECC_BUS_SAFETY_DSS_getRegOffset(uint32_t busSftyNode , SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S *baseAddrOffst);

#endif

/* ========================================================================== */

/*                   Internal Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

#if defined (SUBSYS_DSS)

/********************************************************************************************************
* API to enable the bus Safety on DSS
* Helper function
*********************************************************************************************************/
static void SDL_ECC_BUS_SAFETY_DSS_SAFETY_CTRL_enable (void)
{
    /* enable the bus safety for DSS */
	HW_WR_FIELD32((SDL_ECC_BUS_SAFETY_DSS_BUS_CFG+SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL),\
    SDL_CTRL_BUS_SAFETY_CTRL_ENABLE, 0X7U);
}

/********************************************************************************************************
*   API to clear SEC error on DSS Node
*********************************************************************************************************/
/**
 *  Design:  PROC_SDL-3713
 */
int32_t SDL_ECC_BUS_SAFETY_DSS_secErrorClear(uint32_t busSftyNode)
{
    int32_t retval = SDL_EFAIL;
    SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S baseAddrOffst;
    /* get base address and offset */
    retval = SDL_ECC_BUS_SAFETY_DSS_getRegOffset(busSftyNode,&baseAddrOffst);
    if ( SDL_PASS == retval)
    {
        /* clear SEC error on node */
        HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyFi),\
        SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_SEC, 0x0U);
        /* clear error */
        HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyCtrl),\
        SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ERR_CLEAR,0x1U);
        HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyFi),\
            SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DATA,\
            0X0U);

        retval = SDL_PASS;
    }
    else
    {
        /* Invalid input parameter */
        retval = SDL_EBADARGS;
    }
    return retval;
}

/********************************************************************************************************
*   API to get SEC error status on DSS Node
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3712
 */
int32_t SDL_ECC_BUS_SAFETY_DSS_getSecErrorStatus(uint32_t busSftyNode , uint32_t *status)
{
    int32_t retval = SDL_EFAIL;
    SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S baseAddrOffst;
    /* get base address and offset */
    retval = SDL_ECC_BUS_SAFETY_DSS_getRegOffset(busSftyNode,&baseAddrOffst);
    if (( SDL_PASS == retval)&&(status!=NULL_PTR))
    {
        *status = ((uint32_t) HW_RD_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyErr),\
                         SDL_CTRL_BUS_SAFETY_ERR_BUS_SAFETY_ERR_SEC));
        retval = SDL_PASS;
    }
    else
    {
        /* Invalid input parameter */
        retval = SDL_EBADARGS;
    }
    return retval;
}

/********************************************************************************************************
*   API to clear DED error on DSS Node
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3716
 */
int32_t SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(uint32_t busSftyNode)
{
    int32_t retval = SDL_EFAIL;
    SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S baseAddrOffst;
    /* get base address and offset */
    retval = SDL_ECC_BUS_SAFETY_DSS_getRegOffset(busSftyNode,&baseAddrOffst);
    if ( SDL_PASS == retval)
    {
        /* clear DED error on node */
        HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyFi),\
        SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DED, 0x0U);
        /* clear error */
        HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyCtrl),\
        SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ERR_CLEAR,0x1U);
        HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyFi),\
            SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DATA,\
            0X0U);
        retval = SDL_PASS;
    }
    else
    {
        /* Invalid input parameter */
        retval = SDL_EBADARGS;
    }
    return retval;
}
/********************************************************************************************************
*   API to get DED error status on DSS Node
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-6287
 */
int32_t SDL_ECC_BUS_SAFETY_DSS_getDedErrorStatus(uint32_t busSftyNode , uint32_t *status)
{
    int32_t retval = SDL_EFAIL;
    SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S baseAddrOffst;
    /* get base address and offset */
    retval = SDL_ECC_BUS_SAFETY_DSS_getRegOffset(busSftyNode,&baseAddrOffst);
    if (( SDL_PASS == retval)&&(status!=NULL_PTR))
    {
        *status = ((uint32_t) HW_RD_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyErr),\
                          SDL_CTRL_BUS_SAFETY_ERR_BUS_SAFETY_ERR_DED));
        retval = SDL_PASS;
    }
    else
    {
        /* Invalid input parameter */
        retval = SDL_EBADARGS;
    }
    return retval;
}

/********************************************************************************************************
*   API to clear RED error on DSS Node
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3720
 */
int32_t SDL_ECC_BUS_SAFETY_DSS_redErrorClear(uint32_t busSftyNode)
{
    int32_t retval = SDL_EFAIL;
    SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S baseAddrOffst;
    /* get base address and offset */
    retval = SDL_ECC_BUS_SAFETY_DSS_getRegOffset(busSftyNode,&baseAddrOffst);
    if ( SDL_PASS == retval)
    {
        /* clear error */
        HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyCtrl),\
        SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ERR_CLEAR,0x1U);
        HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyFi),\
        SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_GLOBAL_MAIN, 0x0U);
        HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyFi),\
        SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_GLOBAL_SAFE, 0x0U);
        HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyFi),\
        SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_MAIN, 0x0U);
        HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyFi),\
        SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_SAFE, 0x0U);
        retval = SDL_PASS;
    }
    else
    {
        /* Invalid input parameter */
        retval = SDL_EBADARGS;
    }
    return retval;
}

/********************************************************************************************************
*   API to get RED error status on DSS Node
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-6288
 */
int32_t SDL_ECC_BUS_SAFETY_DSS_getRedErrorStatus(uint32_t busSftyNode , uint32_t *status)
{
    uint32_t cmdErr = 0U;
    uint32_t writeErr = 0U;
    uint32_t readErr = 0U;
    uint32_t writestepErr = 0U;
    int32_t retval = SDL_EFAIL;
    SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S baseAddrOffst;
    /* get base address and offset */
    retval = SDL_ECC_BUS_SAFETY_DSS_getRegOffset(busSftyNode,&baseAddrOffst);
    if (( SDL_PASS == retval)&&(status!=NULL_PTR))
    {
        cmdErr = ((uint32_t) HW_RD_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyErrStatCmd),\
                        SDL_CTRL_BUS_SAFETY_ERR_STAT_CMD_BUS_SAFETY_ERR_STAT_CMD_STAT));

        if((bool)((SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR <= busSftyNode ) && ( SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR >=busSftyNode )) == (bool)0U )
        {
            readErr = ((uint32_t) HW_RD_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyErrStatRd),\
                        SDL_CTRL_BUS_SAFETY_ERR_STAT_READ_BUS_SAFETY_ERR_STAT_READ_STAT));
        }
        else
        {
            readErr =0U;
        }
        if((bool)((SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD <= busSftyNode ) && (SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD >=busSftyNode )) == (bool)0U )
        {
            writeErr = ((uint32_t) HW_RD_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyErrStatWr),\
                        SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITE_BUS_SAFETY_ERR_STAT_WRITE_STAT));
            writestepErr = ((uint32_t) HW_RD_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyErrStatWrResp),\
                        SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITERESP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT));
        }
        else
        {
            writeErr     =0U;
            writestepErr =0U;
        }
        if((cmdErr!=0U) || (writeErr!=0U) || (readErr!=0U)  || (writestepErr !=0U) )
        {
            *status = 1U;
        }
        else
        {
            *status = 0U;
        }
        retval = SDL_PASS;
    }
    else
    {
        /* Invalid input parameter */
        retval = SDL_EBADARGS;
    }
    return retval;
}
/********************************************************************************************************
* Helper function to induce red error on DSS Node
*********************************************************************************************************/

/* Helper function to induce red error */
static void SDL_ECC_BUS_SAFETY_DSS_busSftyFitypeSet(SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType,
                                                SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S *baseAddrOffst)
{
    switch(fiType)
    {
        case SDL_ECC_BUS_SAFETY_FI_MAIN:
 	    HW_WR_FIELD32(((baseAddrOffst->baseAddr)+(baseAddrOffst->busSftyFi)),\
        SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_MAIN,(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_ECC_BUS_SAFETY_FI_SAFE:

 	    HW_WR_FIELD32(((baseAddrOffst->baseAddr)+(baseAddrOffst->busSftyFi)),\
        SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_SAFE,(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_ECC_BUS_SAFETY_FI_GLOBAL_MAIN:
 	    HW_WR_FIELD32(((baseAddrOffst->baseAddr)+(baseAddrOffst->busSftyFi)),\
        SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_GLOBAL_MAIN,0x1U);
        break;
        case SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE:
	    HW_WR_FIELD32(((baseAddrOffst->baseAddr)+(baseAddrOffst->busSftyFi)),\
        SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_GLOBAL_SAFE,0x1U);
        break;
        default:
        /* do nothing */
        break;
    }
}

/********************************************************************************************************
* API to Execute SEC test on DSS Node
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3724
 */

int32_t SDL_ECC_BUS_SAFETY_DSS_secExecute(uint32_t busSftyNode,uint32_t addr, uint32_t wr_data)
{
    int32_t retval = SDL_EFAIL;
    SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S baseAddrOffst;
    /* get base address and offset */
    retval = SDL_ECC_BUS_SAFETY_DSS_getRegOffset(busSftyNode,&baseAddrOffst);
    if ( SDL_PASS == retval)
    {

        /* Check for SEC support on Node */
        if((bool)((SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR>=busSftyNode))==(bool)0U)

        {
            /* Check for EDMA dependency */
            if(((bool)((SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD>=busSftyNode)) == (bool)0U)
            && ((bool)(SDL_ECC_BUS_SAFETY_DSS_PCR==busSftyNode)== (bool)0U))
            {
                /* Validate Start and End address of node */
                if ((baseAddrOffst.nodeStartAddr <= addr)&&(baseAddrOffst.nodeEndAddr>=addr))
                {
                    retval = SDL_PASS;
                }
                else
                {
                    /* Invalid input parameter */
                    retval = SDL_EBADARGS;
                }
            }
            else
            {
                retval = SDL_PASS;
            }

            if(SDL_PASS == retval )
            {
                /* enable the bus safety for DSS */
                SDL_ECC_BUS_SAFETY_DSS_SAFETY_CTRL_enable();
                /* enable the bus safety for node */
	            HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyCtrl),\
                    SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ENABLE,\
		            0x7U);
                /* enable fault injection for node with: single error correction, appropriate 32 bits of bus */
	            HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyFi),\
                    SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_SEC,\
	    	        0x1U);
                /* Enable data */
                HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyFi),\
                    SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DATA,\
                    SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DATA_MAX);
                /* Check for EDMA dependency */
                if(((SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD>=busSftyNode))
                    ||(SDL_ECC_BUS_SAFETY_DSS_PCR==busSftyNode))
                {
                    retval = SDL_PASS;
                }
                else
                {
                    /* issue a bus transaction to node and read back */
                    HW_WR_REG32(addr, wr_data);
                    retval = SDL_PASS;
                }
            }
            else
            {
                /* Invalid input parameter */
                retval = SDL_EBADARGS;
            }
        }
        else
        {
            /* Invalid input parameter */
            retval = SDL_EBADARGS;
        }
    }
    else
    {
        /* Invalid input parameter */
        retval = SDL_EBADARGS;
    }
    return retval;
}

/********************************************************************************************************
* API to Execute Ded test on DSS Node
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3728
 */
int32_t SDL_ECC_BUS_SAFETY_DSS_dedExecute(uint32_t busSftyNode, uint32_t addr, uint32_t wr_data)
{
    int32_t retval = SDL_EFAIL;
    SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S baseAddrOffst;
    /* get base address and offset */
    retval = SDL_ECC_BUS_SAFETY_DSS_getRegOffset(busSftyNode,&baseAddrOffst);
    if ( SDL_PASS == retval)
    {
        /* Check for SEC support on Node */
        if((((bool)((SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR>=busSftyNode)) == (bool)0U)))
        {
            /* Check for EDMA dependency */
            if(((bool)((SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD>=busSftyNode)) ==(bool)0U)
            && ((bool)(SDL_ECC_BUS_SAFETY_DSS_PCR==busSftyNode)== (bool)0U))
            {
                /* Validate Start and End address of node */
                if ((baseAddrOffst.nodeStartAddr <= addr)&&(baseAddrOffst.nodeEndAddr>=addr))
                {
                    retval = SDL_PASS;
                }
                else
                {
                    /* Invalid input parameter */
                    retval = SDL_EBADARGS;
                }
            }
            else
            {
                retval = SDL_PASS;
            }
            if(SDL_PASS == retval )
            {
                /* enable the bus safety for DSS */
                SDL_ECC_BUS_SAFETY_DSS_SAFETY_CTRL_enable();
                /* enable the bus safety for  node */
	            HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyCtrl),\
                    SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ENABLE,\
		            0x7U);
                /* enable fault injection for node with: double error correction, appropriate 32 bits of bus */
	            HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyFi),\
                    SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DED,\
		            0x1U);
                /* Enable data */
                HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyFi),\
                    SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DATA,\
                    SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DATA_MAX);
                /* Check for EDMA dependency */
                if(((SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD>=busSftyNode))
                    ||(SDL_ECC_BUS_SAFETY_DSS_PCR==busSftyNode))
                {
                    retval = SDL_PASS;
                }
                else
                {
                    /* issue a bus transaction to node and read back */
                    HW_WR_REG32(addr, wr_data);

                    retval = SDL_PASS;
                }
            }
            else
            {
                /* Invalid input parameter */
                retval = SDL_EBADARGS;
            }
        }
        else
        {
            /* Invalid input parameter */
            retval = SDL_EBADARGS;
        }
    }
    else
    {
        /* Invalid input parameter */
        retval = SDL_EBADARGS;
    }
    return retval;
}

/********************************************************************************************************
* API to Execute RED test for DSS Node
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3732
 */
int32_t SDL_ECC_BUS_SAFETY_DSS_redExecute(uint32_t busSftyNode,\
         SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType )
{
    int32_t retval = SDL_EFAIL;
    SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S baseAddrOffst;
    /* get base address and offset */
    retval = SDL_ECC_BUS_SAFETY_DSS_getRegOffset(busSftyNode,&baseAddrOffst);
    if ( SDL_PASS == retval)
    {
        if ((fiType< SDL_ECC_BUS_SAFETY_FI_INVALID ) && (redType <SDL_ECC_BUS_SAFETY_FI_TYPE_INVALID))
        {
            /* enable the bus safety for DSS */
            SDL_ECC_BUS_SAFETY_DSS_SAFETY_CTRL_enable();
            /* enable the bus safety for node */
	        HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyCtrl),\
            SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ENABLE,\
		    0x7U);
            /* Inject fault. */
            SDL_ECC_BUS_SAFETY_DSS_busSftyFitypeSet(fiType,redType,&baseAddrOffst);
            retval = SDL_PASS;
        }
        else
        {
            /* Invalid input parameter */
            retval = SDL_EBADARGS;
        }
    }
    else
    {
        /* Invalid input parameter */
        retval = SDL_EBADARGS;
    }
    return retval;
}

/********************************************************************************************************
* Helper API to get the offset address of particular DSS node
*********************************************************************************************************/

static int32_t SDL_ECC_BUS_SAFETY_DSS_getRegOffset(uint32_t busSftyNode , SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S *baseAddrOffst)
{
    int32_t retval = SDL_PASS;
    baseAddrOffst->baseAddr = SDL_ECC_BUS_SAFETY_DSS_BUS_CFG;
    switch(busSftyNode)
    {
        /* DSS_MCRC */
        case SDL_ECC_BUS_SAFETY_DSS_MCRC :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = SDL_DSS_MCRC_U_END;
            baseAddrOffst->nodeStartAddr        = SDL_DSS_MCRC_U_BASE;
            break;
        }
        /* DSS_MDO_FIFO */
        case SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = SDL_DSS_MDO_FIFO_U_END;
            baseAddrOffst->nodeStartAddr        = SDL_DSS_MDO_FIFO_U_BASE;
            break;
        }
        /* DSS_CBUFF_FIFO */
        case SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = SDL_DSS_CBUFF_FIFO_U_END;
            baseAddrOffst->nodeStartAddr        = SDL_DSS_CBUFF_FIFO_U_BASE;
            break;
        }
        /* DSS_TPTC_A0_WR */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = 0U;
            baseAddrOffst->busSftyErrStatWr     = SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* DSS_TPTC_A1_WR */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_WR :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = 0U;
            baseAddrOffst->busSftyErrStatWr     = SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* DSS_TPTC_B0_WR */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_WR :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = 0U;
            baseAddrOffst->busSftyErrStatWr     = SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* DSS_TPTC_B1_WR */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_WR :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = 0U;
            baseAddrOffst->busSftyErrStatWr     = SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* DSS_TPTC_C0_WR */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_WR :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = 0U;
            baseAddrOffst->busSftyErrStatWr     = SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* DSS_TPTC_C1_WR */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_WR :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = 0U;
            baseAddrOffst->busSftyErrStatWr     = SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* DSS_TPTC_C2_WR */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_WR :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = 0U;
            baseAddrOffst->busSftyErrStatWr     = SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* DSS_TPTC_C3_WR */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_WR :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = 0U;
            baseAddrOffst->busSftyErrStatWr     = SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* DSS_TPTC_C4_WR */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_WR :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = 0U;
            baseAddrOffst->busSftyErrStatWr     = SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* DSS_TPTC_C5_WR */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = 0U;
            baseAddrOffst->busSftyErrStatWr     = SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }

        /* DSS_TPTC_A0_RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = 0U;
            baseAddrOffst->busSftyErrStatWrResp = 0U;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* DSS_TPTC_C3_RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = 0U;
            baseAddrOffst->busSftyErrStatWrResp = 0U;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* DSS_TPTC_B0_RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = 0U;
            baseAddrOffst->busSftyErrStatWrResp = 0U;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* DSS_TPTC_B1_RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = 0U;
            baseAddrOffst->busSftyErrStatWrResp = 0U;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* DSS_TPTC_C0_RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = 0U;
            baseAddrOffst->busSftyErrStatWrResp = 0U;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* DSS_TPTC_C1_RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = 0U;
            baseAddrOffst->busSftyErrStatWrResp = 0U;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* DSS_TPTC_C2_RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = 0U;
            baseAddrOffst->busSftyErrStatWrResp = 0U;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* DSS_TPTC_C3_RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = 0U;
            baseAddrOffst->busSftyErrStatWrResp = 0U;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* DSS_TPTC_C4_RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = 0U;
            baseAddrOffst->busSftyErrStatWrResp = 0U;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* DSS_TPTC_C5_RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = 0U;
            baseAddrOffst->busSftyErrStatWrResp = 0U;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* DSS_L3_BANKA */
        case SDL_ECC_BUS_SAFETY_DSS_L3_BANKA :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = SDL_DSS_L3_BANKA_ADDRESS_END;
            baseAddrOffst->nodeStartAddr        = SDL_DSS_L3_BANKA_ADDRESS;
            break;
        }
        /* DSS_L3_BANKB */
        case SDL_ECC_BUS_SAFETY_DSS_L3_BANKB :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = SDL_DSS_L3_BANKB_ADDRESS_END;
            baseAddrOffst->nodeStartAddr        = SDL_DSS_L3_BANKB_ADDRESS;
            break;
        }
        /* DSS_L3_BANKC */
        case SDL_ECC_BUS_SAFETY_DSS_L3_BANKC :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = SDL_DSS_L3_BANKC_ADDRESS_END;
            baseAddrOffst->nodeStartAddr        = SDL_DSS_L3_BANKC_ADDRESS;
            break;
        }
        /* DSS_L3_BANKD */
        case SDL_ECC_BUS_SAFETY_DSS_L3_BANKD :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = SDL_DSS_L3_BANKD_ADDRESS_END;
            baseAddrOffst->nodeStartAddr        = SDL_DSS_L3_BANKD_ADDRESS;
            break;
        }
        /* DSS_MBOX */
        case SDL_ECC_BUS_SAFETY_DSS_MBOX :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = SDL_DSS_MAILBOX_U_BASE_END;
            baseAddrOffst->nodeStartAddr        = SDL_DSS_MAILBOX_U_BASE;
            break;
        }

        /* DSS_HWA_DMA0 */
        case SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0 :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = SDL_DSS_HWA_DMA0_U_BASE_END;
            baseAddrOffst->nodeStartAddr        = SDL_DSS_HWA_DMA0_U_BASE;
            break;
        }

        /* DSS_HWA_DMA1 */
        case SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1 :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = SDL_DSS_HWA_DMA1_U_BASE_END;
            baseAddrOffst->nodeStartAddr        = SDL_DSS_HWA_DMA1_U_BASE;
            break;
        }

        /* DSS_DSP_MDMA */
        case SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = SDL_DSS_L3_BANKD_ADDRESS_END;
            baseAddrOffst->nodeStartAddr        = SDL_DSS_L3_BANKA_ADDRESS;
            break;
        }

        /* DSS_DSP_SDMA */
        case SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = SDL_DSS_L2_U_BASE_END;
            baseAddrOffst->nodeStartAddr        = SDL_DSS_L2_U_BASE;
            break;
        }

        /* DSS_PCR */
        case SDL_ECC_BUS_SAFETY_DSS_PCR :
        {
            baseAddrOffst->busSftyCtrl          = SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        default:
        {
            baseAddrOffst->busSftyCtrl          = 0U;
            baseAddrOffst->busSftyErr           = 0U;
            baseAddrOffst->busSftyFi            = 0U;
            baseAddrOffst->busSftyErrStatCmd    = 0U;
            baseAddrOffst->busSftyErrStatRd     = 0U;
            baseAddrOffst->busSftyErrStatWr     = 0U;
            baseAddrOffst->busSftyErrStatWrResp = 0U;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            retval = SDL_EBADARGS;
            break;
        }
    }
    return retval;
}

/********************************************************************************************************
*   This funtion will provide the application to read the static registers
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-5496
 */

int32_t SDL_ECC_BUS_SAFETY_DSS_readStaticRegs(uint32_t busSftyNode ,\
                                               SDL_ECC_BUS_SAFETY_staticRegs *pStaticRegs)
{
    int32_t retval = SDL_EFAIL;
    SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S baseAddrOffst;
    if (pStaticRegs != (NULL_PTR))
    {
        /* get base address and offset */
        retval = SDL_ECC_BUS_SAFETY_DSS_getRegOffset(busSftyNode,&baseAddrOffst);
        if ( SDL_PASS == retval)
        {

            pStaticRegs->coreSftyCtrl=HW_RD_REG32(SDL_ECC_BUS_SAFETY_DSS_BUS_CFG+SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL);
            pStaticRegs->busSftyCtrl = HW_RD_REG32(baseAddrOffst.baseAddr + baseAddrOffst.busSftyCtrl);
            pStaticRegs->busSftyFi = HW_RD_REG32(baseAddrOffst.baseAddr + baseAddrOffst.busSftyFi);
            retval = SDL_PASS;
        }
        else
        {
            retval = SDL_EBADARGS;
        }
    }
    else
    {
        retval = SDL_EBADARGS;
    }

    return (retval);
}

#endif

#if defined (SUBSYS_MSS)
/********************************************************************************************************
* API to enable the bus Safety on MSS
* Helper function
*********************************************************************************************************/
static void SDL_ECC_BUS_SAFETY_MSS_SAFETY_CTRL_enable (void)
{
    /* enable the bus safety for MSS */
	HW_WR_FIELD32((SDL_ECC_BUS_SAFETY_MSS_BUS_CFG+SDL_MSS_CTRL_MSS_BUS_SAFETY_CTRL),\
    SDL_CTRL_BUS_SAFETY_CTRL_ENABLE, 0X7U);
}

/********************************************************************************************************
*   API to clear SEC error on MSS Node
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3713
 */
int32_t SDL_ECC_BUS_SAFETY_MSS_secErrorClear(uint32_t busSftyNode)
{
    int32_t retval = SDL_EFAIL;
    SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S baseAddrOffst;
    /* get base address and offset */
    retval = SDL_ECC_BUS_SAFETY_MSS_getRegOffset(busSftyNode,&baseAddrOffst);
    if ( SDL_PASS == retval)
    {
        /* clear SEC error on node */
        HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyFi),\
        SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_SEC, 0x0U);
        /* clear error */
        HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyCtrl),\
        SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ERR_CLEAR,0x1U);
        HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyFi),\
            SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DATA,\
            0X0U);

        retval = SDL_PASS;
    }
    else
    {
        /* Invalid input parameter */
        retval = SDL_EBADARGS;
    }
    return retval;
}

/********************************************************************************************************
*   API to get SEC error status on MSS Node
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3712
 */
int32_t SDL_ECC_BUS_SAFETY_MSS_getSecErrorStatus(uint32_t busSftyNode , uint32_t *status)
{
    int32_t retval = SDL_EFAIL;
    SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S baseAddrOffst;
    /* get base address and offset */
    retval = SDL_ECC_BUS_SAFETY_MSS_getRegOffset(busSftyNode,&baseAddrOffst);
    if (( SDL_PASS == retval)&&(status!=NULL_PTR))
    {
        *status = ((uint32_t) HW_RD_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyErr),\
                         SDL_CTRL_BUS_SAFETY_ERR_BUS_SAFETY_ERR_SEC));
        retval = SDL_PASS;
    }
    else
    {
        /* Invalid input parameter */
        retval = SDL_EBADARGS;
    }
    return retval;
}

/********************************************************************************************************
*   API to clear DED error on MSS Node
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3716
 */
int32_t SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(uint32_t busSftyNode)
{
    int32_t retval = SDL_EFAIL;
    SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S baseAddrOffst;
    /* get base address and offset */
    retval = SDL_ECC_BUS_SAFETY_MSS_getRegOffset(busSftyNode,&baseAddrOffst);
    if ( SDL_PASS == retval)
    {
        /* clear DED error on node */
        HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyFi),\
        SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DED, 0x0U);
        /* clear error */
        HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyCtrl),\
        SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ERR_CLEAR,0x1U);
        HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyFi),\
            SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DATA,\
            0X0U);
        retval = SDL_PASS;
    }
    else
    {
        /* Invalid input parameter */
        retval = SDL_EBADARGS;
    }
    return retval;
}
/********************************************************************************************************
*   API to get DED error status on MSS Node
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-6287
 */
int32_t SDL_ECC_BUS_SAFETY_MSS_getDedErrorStatus(uint32_t busSftyNode , uint32_t *status)
{
    int32_t retval = SDL_EFAIL;
    SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S baseAddrOffst;
    /* get base address and offset */
    retval = SDL_ECC_BUS_SAFETY_MSS_getRegOffset(busSftyNode,&baseAddrOffst);
    if (( SDL_PASS == retval)&&(status!=NULL_PTR))
    {
        *status = ((uint32_t) HW_RD_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyErr),\
                          SDL_CTRL_BUS_SAFETY_ERR_BUS_SAFETY_ERR_DED));
        retval = SDL_PASS;
    }
    else
    {
        /* Invalid input parameter */
        retval = SDL_EBADARGS;
    }
    return retval;
}

/********************************************************************************************************
*   API to clear RED error on MSS Node
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3720
 */
int32_t SDL_ECC_BUS_SAFETY_MSS_redErrorClear(uint32_t busSftyNode)
{
    int32_t retval = SDL_EFAIL;
    SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S baseAddrOffst;
    /* get base address and offset */
    retval = SDL_ECC_BUS_SAFETY_MSS_getRegOffset(busSftyNode,&baseAddrOffst);
    if ( SDL_PASS == retval)
    {
        /* clear error */
        HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyCtrl),\
        SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ERR_CLEAR,0x1U);
        HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyFi),\
        SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_GLOBAL_MAIN, 0x0U);
        HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyFi),\
        SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_GLOBAL_SAFE, 0x0U);
        HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyFi),\
        SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_MAIN, 0x0U);
        HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyFi),\
        SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_SAFE, 0x0U);
        retval = SDL_PASS;
    }
    else
    {
        /* Invalid input parameter */
        retval = SDL_EBADARGS;
    }
    return retval;
}

/********************************************************************************************************
*   API to clear RED error on MSS Node
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-6288
 */
int32_t SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(uint32_t busSftyNode , uint32_t *status)
{
    uint32_t cmdErr = 0U;
    uint32_t writeErr = 0U;
    uint32_t readErr = 0U;
    uint32_t writestepErr = 0U;
    int32_t retval = SDL_EFAIL;
    SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S baseAddrOffst;
    /* get base address and offset */
    retval = SDL_ECC_BUS_SAFETY_MSS_getRegOffset(busSftyNode,&baseAddrOffst);
    /* Check for valid input parameter */
    if (( SDL_PASS == retval)&&(status!=NULL_PTR))
    {
        cmdErr = ((uint32_t) HW_RD_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyErrStatCmd),\
                        SDL_CTRL_BUS_SAFETY_ERR_STAT_CMD_BUS_SAFETY_ERR_STAT_CMD_STAT));

        if((bool)((SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR <= busSftyNode ) && ( SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR >=busSftyNode )) == (bool)0U )
        {
            readErr = ((uint32_t) HW_RD_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyErrStatRd),\
                        SDL_CTRL_BUS_SAFETY_ERR_STAT_READ_BUS_SAFETY_ERR_STAT_READ_STAT));
        }
        else
        {
            readErr =0U;
        }
        if((bool)((SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD <= busSftyNode ) && ( SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR >=busSftyNode )) == (bool)0U )
        {
            writeErr = ((uint32_t) HW_RD_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyErrStatWr),\
                        SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITE_BUS_SAFETY_ERR_STAT_WRITE_STAT));
            writestepErr = ((uint32_t) HW_RD_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyErrStatWrResp),\
                        SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITERESP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT));
        }
        else
        {
            writeErr     =0U;
            writestepErr =0U;
        }
        if((cmdErr!=0U) || (writeErr!=0U) || (readErr!=0U)  || (writestepErr !=0U) )
        {
            *status = 1U;
        }
        else
        {
            *status = 0U;
        }
        retval = SDL_PASS;
    }
    else
    {
        /* Invalid input parameter */
        retval = SDL_EBADARGS;
    }
    return retval;
}
/********************************************************************************************************
* Helper function to induce red error on MSS node
*********************************************************************************************************/
static void SDL_ECC_BUS_SAFETY_MSS_busSftyFitypeSet(SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType,
                                                SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S *baseAddrOffst)
{
    switch(fiType)
    {
        case SDL_ECC_BUS_SAFETY_FI_MAIN:
 	    HW_WR_FIELD32((baseAddrOffst->baseAddr+baseAddrOffst->busSftyFi),\
        SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_MAIN,(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_ECC_BUS_SAFETY_FI_SAFE:
 	    HW_WR_FIELD32((baseAddrOffst->baseAddr+baseAddrOffst->busSftyFi),\
        SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_SAFE,(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_ECC_BUS_SAFETY_FI_GLOBAL_MAIN:
 	    HW_WR_FIELD32((baseAddrOffst->baseAddr+baseAddrOffst->busSftyFi),\
        SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_GLOBAL_MAIN,0x1U);
        break;
        case SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE:
	    HW_WR_FIELD32((baseAddrOffst->baseAddr+baseAddrOffst->busSftyFi),\
        SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_GLOBAL_SAFE,0x1U);
        break;
        default:
        /* do nothing */
        break;
    }
}

/********************************************************************************************************
* API to Execute SEC test for MSS Nodes
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3724
 */

int32_t SDL_ECC_BUS_SAFETY_MSS_secExecute(uint32_t busSftyNode,uint32_t addr, uint32_t wr_data)
{
    int32_t retval = SDL_EFAIL;
    SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S baseAddrOffst;
    /* get base address and offset */
    retval = SDL_ECC_BUS_SAFETY_MSS_getRegOffset(busSftyNode,&baseAddrOffst);
    if ( SDL_PASS == retval)
    {
        /* Check for SEC support on Node */
      if((bool)((SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR>=busSftyNode))==(bool)0U)
        {
            /* Check for  dependency */
            if(((bool)((SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD>=busSftyNode)) == (bool)0U)&&
                ((bool)(((SDL_ECC_BUS_SAFETY_MSS_PCR == busSftyNode) ==(bool)0U)&& ((bool)(SDL_ECC_BUS_SAFETY_MSS_PCR2 == busSftyNode)))==(bool)0U)&&
                ((bool)(SDL_ECC_BUS_SAFETY_MSS_CPSW == busSftyNode )==(bool)0U))
            {
                /* Validate Start and End address of node */
                if ((baseAddrOffst.nodeStartAddr <= addr)&&(baseAddrOffst.nodeEndAddr>=addr))
                {
                    retval = SDL_PASS;
                }
                else
                {
                    /* Invalid input parameter */
                    retval = SDL_EBADARGS;
                }
            }
            else
            {
                retval = SDL_PASS;
            }

            if (SDL_ECC_BUS_SAFETY_MSS_TO_MDO == busSftyNode )
            {

                /* Enabling MDO registers */
                HW_WR_REG32(SDL_TOP_MDO_INFRA_U_BASE+0x1008U, 0x01234567U);
                HW_WR_REG32(SDL_TOP_MDO_INFRA_U_BASE+0x100CU, 0xFEDCBA8U);
                HW_WR_REG32(SDL_TOP_MDO_INFRA_U_BASE+0x00BCU, 0x10000U);
                HW_WR_REG32(SDL_TOP_MDO_INFRA_U_BASE+0x00E4U, 0x2U);
                HW_WR_REG32(SDL_TOP_MDO_INFRA_U_BASE+0x00C0U, 0x0U);
                HW_WR_REG32(SDL_TOP_MDO_INFRA_U_BASE+0x00C4U, 0x1FFFU);
                HW_WR_REG32(SDL_TOP_MDO_INFRA_U_BASE+0x00E8U, 0x10000000U);
                HW_WR_REG32(SDL_TOP_MDO_INFRA_U_BASE+0x00ECU, 0x300U);
                HW_WR_REG32(SDL_TOP_MDO_INFRA_U_BASE+0x00F0U, 0x9U);
                HW_WR_REG32(SDL_TOP_MDO_INFRA_U_BASE+0x00BCU, 0x10001U);

          }

          else
          {
            /*do nothing*/
          }

            if(SDL_PASS == retval )
            {
                /* enable the bus safety for MSS */
                SDL_ECC_BUS_SAFETY_MSS_SAFETY_CTRL_enable();
                /* enable the bus safety for node */
	            HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyCtrl),\
                    SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ENABLE,\
		            0x7U);
                /* enable fault injection for node with: single error correction, appropriate 32 bits of bus */
	            HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyFi),\
                    SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_SEC,\
	    	        0x1U);
                /* Enable data */
                HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyFi),\
                    SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DATA,\
                    SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DATA_MAX);
                /* Check for  dependency */
                if(((SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD>=busSftyNode)) ||
                    (SDL_ECC_BUS_SAFETY_MSS_PCR == busSftyNode) || (SDL_ECC_BUS_SAFETY_MSS_PCR2 == busSftyNode )||
                    (SDL_ECC_BUS_SAFETY_MSS_CPSW == busSftyNode ))
                {
                    retval = SDL_PASS;
                }
                else
                {
                    /* issue a bus transaction to node and read back */
                    HW_WR_REG32(addr, wr_data);
                    retval = SDL_PASS;
                }
                }
            else
            {
                /* Invalid input parameter */
                retval = SDL_EBADARGS;
            }
        }
        else
        {
            /* Invalid input parameter */
            retval = SDL_EBADARGS;
        }
    }
    else
    {
        /* Invalid input parameter */
        retval = SDL_EBADARGS;
    }
    return retval;
}

/********************************************************************************************************
* API to Execute Ded test for MSS Node
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3728
 */
int32_t SDL_ECC_BUS_SAFETY_MSS_dedExecute(uint32_t busSftyNode, uint32_t addr, uint32_t wr_data)
{
    int32_t retval = SDL_EFAIL;
    SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S baseAddrOffst;
    /* get base address and offset */
    retval = SDL_ECC_BUS_SAFETY_MSS_getRegOffset(busSftyNode,&baseAddrOffst);
    if ( SDL_PASS == retval)
    {
        /* Check for SEC support on Node */
      if((bool)((SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR>=busSftyNode))==(bool)0U)
        {
            /* Check for  dependency */
            if(((bool)((SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD>=busSftyNode)) == (bool)0U)&&
                ((bool)(((SDL_ECC_BUS_SAFETY_MSS_PCR == busSftyNode) ==(bool)0U)&& ((bool)(SDL_ECC_BUS_SAFETY_MSS_PCR2 == busSftyNode)))==(bool)0U)&&
                ((bool)(SDL_ECC_BUS_SAFETY_MSS_CPSW == busSftyNode )==(bool)0U))
            {
                /* Validate Start and End address of node */
                if ((baseAddrOffst.nodeStartAddr <= addr)&&(baseAddrOffst.nodeEndAddr>=addr))
                {
                    retval = SDL_PASS;
                }
                else
                {
                    /* Invalid input parameter */
                    retval = SDL_EBADARGS;
                }
            }
            else
            {
                retval = SDL_PASS;
            }

            if (SDL_ECC_BUS_SAFETY_MSS_TO_MDO == busSftyNode )
            {
                /* Enabling MDO registers */
                HW_WR_REG32(SDL_TOP_MDO_INFRA_U_BASE+0x1008U, 0x01234567U);
                HW_WR_REG32(SDL_TOP_MDO_INFRA_U_BASE+0x100CU, 0xFEDCBA8U);
            }

            if(SDL_PASS == retval )
            {
                /* enable the bus safety for MSS */
                SDL_ECC_BUS_SAFETY_MSS_SAFETY_CTRL_enable();
                /* enable the bus safety for  node */
	              HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyCtrl),\
                    SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ENABLE,\
		            0x7U);
                /* enable fault injection for node with: double error correction, appropriate 32 bits of bus */
	              HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyFi),\
                    SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DED,\
		            0x1U);
                /* Enable data */
                HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyFi),\
                    SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DATA,\
                    SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DATA_MAX);
                /* Check for  dependency */
                if(((SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD>=busSftyNode)) ||
                    (SDL_ECC_BUS_SAFETY_MSS_PCR == busSftyNode) || (SDL_ECC_BUS_SAFETY_MSS_PCR2 == busSftyNode )||
                    (SDL_ECC_BUS_SAFETY_MSS_CPSW == busSftyNode ))
                {
                    retval = SDL_PASS;
                }
                else
                {
                    /* issue a bus transaction to node and read back */
                    HW_WR_REG32(addr, wr_data);
                    retval = SDL_PASS;
                }
            }
            else
            {
                /* Invalid input parameter */
                retval = SDL_EBADARGS;
            }
        }
        else
        {
            /* Invalid input parameter */
            retval = SDL_EBADARGS;
        }
    }
    else
    {
        /* Invalid input parameter */
        retval = SDL_EBADARGS;
    }
    return retval;
}

/********************************************************************************************************
* API to Execute RED test on MSS Node
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3732
 */
int32_t SDL_ECC_BUS_SAFETY_MSS_redExecute(uint32_t busSftyNode,\
         SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType )
{
    int32_t retval = SDL_EFAIL;
    SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S baseAddrOffst;
    /* get base address and offset */
    retval = SDL_ECC_BUS_SAFETY_MSS_getRegOffset(busSftyNode,&baseAddrOffst);
    if ( SDL_PASS == retval)
    {
        /* Check for valid input FI and FY type */
        if ((fiType< SDL_ECC_BUS_SAFETY_FI_INVALID ) && (redType <SDL_ECC_BUS_SAFETY_FI_TYPE_INVALID))
        {
            /* enable the bus safety for MSS */
            SDL_ECC_BUS_SAFETY_MSS_SAFETY_CTRL_enable();
            /* enable the bus safety for node */
	        HW_WR_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyCtrl),\
            SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ENABLE,\
		    0x7U);
            /* Inject fault. */
            SDL_ECC_BUS_SAFETY_MSS_busSftyFitypeSet(fiType,redType,&baseAddrOffst);
            retval = SDL_PASS;
        }
        else
        {
            /* Invalid input parameter */
            retval = SDL_EBADARGS;
        }
    }
    else
    {
        /* Invalid input parameter */
        retval = SDL_EBADARGS;
    }
    return retval;
}

/********************************************************************************************************
* Helper API to get the offset address of particular MSS node
*********************************************************************************************************/
static int32_t SDL_ECC_BUS_SAFETY_MSS_getRegOffset(uint32_t busSftyNode , SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S *baseAddrOffst)
{
    int32_t retval = SDL_PASS;
    baseAddrOffst->baseAddr = SDL_ECC_BUS_SAFETY_MSS_BUS_CFG;
    switch(busSftyNode)
    {
        /* MSS_TPTC_A0_WR */
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR :
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = 0U;
            baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* MSS_TPTC_A1_WR */
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR :
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = 0U;
            baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* MSS_TPTC_B0_WR */
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR :
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = 0U;
            baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* MSS_TPTC_A0_RD */
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD :
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = 0U;
            baseAddrOffst->busSftyErrStatWrResp = 0U;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* MSS_TPTC_A1_RD */
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD :
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = 0U;
            baseAddrOffst->busSftyErrStatWrResp = 0U;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* MSS_TPTC_B0_RD */
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD :
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = 0U;
            baseAddrOffst->busSftyErrStatWrResp = 0U;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* MSS_MBOX*/
        case SDL_ECC_BUS_SAFETY_MSS_MBOX  :
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = SDL_MSS_MBOX_U_END;
            baseAddrOffst->nodeStartAddr        = SDL_MSS_MBOX_U_BASE;
            break;
        }
        /* MSS_CR5A_AHB */
        case SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB  :
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = SDL_MSS_CTRL_R5SS0_CORE0_AHB_END;
            baseAddrOffst->nodeStartAddr        = SDL_MSS_CTRL_R5SS0_CORE0_AHB_BASE;
            break;
        }
        /* MSS_CR5B_AHB */
        case SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB  :
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = SDL_MSS_CTRL_R5SS1_CORE0_AHB_END;
            baseAddrOffst->nodeStartAddr        = SDL_MSS_CTRL_R5SS1_CORE0_AHB_BASE;
            break;
        }
        /* MSS_CR5A_AXI_RD */
        case SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD  :
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = 0U;
            baseAddrOffst->busSftyErrStatWrResp = 0U;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* MSS_CR5B_AXI_RD */
        case SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD  :
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = 0U;
            baseAddrOffst->busSftyErrStatWrResp = 0U;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* MSS_CR5A_AXI_WR */
        case SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR  :
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = 0U;
            baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
       /* MSS_CR5B_AXI_WR */
           case SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR  :
            {
                baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL;
                baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR;
                baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI;
                baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_CMD;
                baseAddrOffst->busSftyErrStatRd     = 0U;
                baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE;
                baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
                baseAddrOffst->nodeEndAddr          = 0U;
                baseAddrOffst->nodeStartAddr        = 0U;
                break;
            }
          /* MSS_CR5A_AXI_S */
            case SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S  :
             {
                 baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL;
                 baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR;
                 baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI;
                 baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_CMD;
                 baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_READ;
                 baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITE;
                 baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP;
                 baseAddrOffst->nodeEndAddr          = 0U;
                 baseAddrOffst->nodeStartAddr        = 0U;
                 break;
             }
           /* MSS_CR5B_AXI_S */
             case SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S  :
              {
                  baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL;
                  baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR;
                  baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI;
                  baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_CMD;
                  baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_READ;
                  baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITE;
                  baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP;
                  baseAddrOffst->nodeEndAddr          = 0U;
                  baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }

        /* MSS_QSPI */
        case SDL_ECC_BUS_SAFETY_MSS_QSPI :
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_QSPI_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = SDL_MSS_QSPI_U_END;
            baseAddrOffst->nodeStartAddr        = SDL_MSS_QSPI_U_BASE;
            break;
        }
        /* MSS_MCRC */
        case SDL_ECC_BUS_SAFETY_MSS_MCRC :
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_MCRC_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = SDL_MSS_MCRC_U_END;
            baseAddrOffst->nodeStartAddr        = SDL_MSS_MCRC_U_BASE;
            break;
        }
        /* MSS_SWBUF */
        case SDL_ECC_BUS_SAFETY_MSS_SWBUF :
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_SWBUF_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = SDL_MSS_SWBUF_U_END;
            baseAddrOffst->nodeStartAddr        = SDL_MSS_SWBUF_U_BASE;
            break;
        }
        /* MSS_SCRP */
        case SDL_ECC_BUS_SAFETY_MSS_SCRP :
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_SCRP_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* MSS_TO_MDO */
        case SDL_ECC_BUS_SAFETY_MSS_TO_MDO :
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_TO_MDO_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = SDL_MSS_TO_MDO_U_END;
            baseAddrOffst->nodeStartAddr        = SDL_MSS_TO_MDO_U_BASE;
            break;
        }
        /* DAP_R232 */
        case SDL_ECC_BUS_SAFETY_DAP_R232 :
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_DAP_R232_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* MSS_CPSW */
        case SDL_ECC_BUS_SAFETY_MSS_CPSW  :
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_CPSW_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* MSS_PCR */
        case SDL_ECC_BUS_SAFETY_MSS_PCR  :
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* MSS_PCR2 */
        case SDL_ECC_BUS_SAFETY_MSS_PCR2  :
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_PCR2_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* MSS_L2_A */
        case SDL_ECC_BUS_SAFETY_MSS_L2_A  :
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_L2_A_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = SDL_MSS_L2_A_BASE_END;
            baseAddrOffst->nodeStartAddr        = SDL_MSS_L2_A_BASE_START;
            break;
        }
        /* MSS_L2_B */
        case SDL_ECC_BUS_SAFETY_MSS_L2_B  :
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_L2_B_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = SDL_MSS_L2_B_BASE_END;
            baseAddrOffst->nodeStartAddr        = SDL_MSS_L2_B_BASE_START;
            break;
        }
        /* MSS_GPADC */
        case SDL_ECC_BUS_SAFETY_MSS_GPADC:
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_GPADC_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = SDL_MSS_GPADC_DATA_RAM_U_BASE_END;
            baseAddrOffst->nodeStartAddr        = SDL_MSS_GPADC_DATA_RAM_U_BASE;
            break;
        }
        /* MSS_DMM */
        case SDL_ECC_BUS_SAFETY_MSS_DMM:
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_DMM_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            break;
        }
        /* MSS_DMM_SLV */
        case SDL_ECC_BUS_SAFETY_MSS_DMM_SLV:
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_DMM_SLV_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = SDL_MSS_DMM_A_DATA_U_BASE_END;
            baseAddrOffst->nodeStartAddr        = SDL_MSS_DMM_A_DATA_U_BASE;
            break;
        }
        default:
        {
            baseAddrOffst->busSftyCtrl          = 0U;
            baseAddrOffst->busSftyErr           = 0U;
            baseAddrOffst->busSftyFi            = 0U;
            baseAddrOffst->busSftyErrStatCmd    = 0U;
            baseAddrOffst->busSftyErrStatRd     = 0U;
            baseAddrOffst->busSftyErrStatWr     = 0U;
            baseAddrOffst->busSftyErrStatWrResp = 0U;
            baseAddrOffst->nodeEndAddr          = 0U;
            baseAddrOffst->nodeStartAddr        = 0U;
            retval = SDL_EBADARGS;
            break;
        }
    }
    return retval;
}

/********************************************************************************************************
*   This funtion will provide the application to read the static registers
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-5496
 */

int32_t SDL_ECC_BUS_SAFETY_MSS_readStaticRegs(uint32_t busSftyNode ,\
                                               SDL_ECC_BUS_SAFETY_staticRegs *pStaticRegs)
{
    int32_t retval = SDL_EFAIL;
    SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S baseAddrOffst;
    if (pStaticRegs != (NULL_PTR))
    {
        /* get base address and offset */
        retval = SDL_ECC_BUS_SAFETY_MSS_getRegOffset(busSftyNode,&baseAddrOffst);
        if ( SDL_PASS == retval)
        {

            pStaticRegs->coreSftyCtrl=HW_RD_REG32(SDL_ECC_BUS_SAFETY_MSS_BUS_CFG+SDL_MSS_CTRL_MSS_BUS_SAFETY_CTRL);
            pStaticRegs->busSftyCtrl = HW_RD_REG32(baseAddrOffst.baseAddr + baseAddrOffst.busSftyCtrl);
            pStaticRegs->busSftyFi = HW_RD_REG32(baseAddrOffst.baseAddr + baseAddrOffst.busSftyFi);
            retval = SDL_PASS;
        }
        else
        {
            retval = SDL_EBADARGS;
        }
    }
    else
    {
        retval = SDL_EBADARGS;
    }

    return (retval);
}

#endif
