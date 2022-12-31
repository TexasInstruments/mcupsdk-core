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

/* ========================================================================== */

/*                   Internal Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

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
*   API to clear SEC error
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3713
 */
int32_t SDL_ECC_BUS_SAFETY_MSS_secErrorClear(uint32_t busSftyNode)
{
    int32_t retval = SDL_EFAIL;
    SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S baseAddrOffst;
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
        retval = SDL_EBADARGS;
    }
    return retval;
}

/********************************************************************************************************
*   API to get SEC error status
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3712
 */
int32_t SDL_ECC_BUS_SAFETY_MSS_getSecErrorStatus(uint32_t busSftyNode , uint32_t *status)
{
    int32_t retval = SDL_EFAIL;
    SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S baseAddrOffst;
    retval = SDL_ECC_BUS_SAFETY_MSS_getRegOffset(busSftyNode,&baseAddrOffst);
    if (( SDL_PASS == retval)&&(status!=NULL_PTR))
    {
        *status = ((uint32_t) HW_RD_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyErr),\
                         SDL_CTRL_BUS_SAFETY_ERR_BUS_SAFETY_ERR_SEC));
        retval = SDL_PASS;
    }
    else
    {
       retval = SDL_EBADARGS;
    }
    return retval;
}

/********************************************************************************************************
*   API to clear DED error
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3716
 */
int32_t SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(uint32_t busSftyNode)
{
    int32_t retval = SDL_EFAIL;
    SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S baseAddrOffst;
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
       retval = SDL_EBADARGS;
    }
    return retval;
}
/********************************************************************************************************
*   API to get DED error status
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3716
 */
int32_t SDL_ECC_BUS_SAFETY_MSS_getDedErrorStatus(uint32_t busSftyNode , uint32_t *status)
{
    int32_t retval = SDL_EFAIL;
    SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S baseAddrOffst;
    retval = SDL_ECC_BUS_SAFETY_MSS_getRegOffset(busSftyNode,&baseAddrOffst);
    if (( SDL_PASS == retval)&&(status!=NULL_PTR))
    {
        *status = ((uint32_t) HW_RD_FIELD32((baseAddrOffst.baseAddr+baseAddrOffst.busSftyErr),\
                          SDL_CTRL_BUS_SAFETY_ERR_BUS_SAFETY_ERR_DED));
        retval = SDL_PASS;
    }
    else
    {
       retval = SDL_EBADARGS;
    }
    return retval;
}

/********************************************************************************************************
*   API to clear RED error
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3720
 */
int32_t SDL_ECC_BUS_SAFETY_MSS_redErrorClear(uint32_t busSftyNode)
{
    int32_t retval = SDL_EFAIL;
    SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S baseAddrOffst;
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
       retval = SDL_EBADARGS;
    }
    return retval;
}

/********************************************************************************************************
*   API to clear RED error
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3720
 */
int32_t SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(uint32_t busSftyNode , uint32_t *status)
{
    uint32_t cmdErr = 0U;
    uint32_t writeErr = 0U;
    uint32_t readErr = 0U;
    uint32_t writestepErr = 0U;
    int32_t retval = SDL_EFAIL;
    SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S baseAddrOffst;
    retval = SDL_ECC_BUS_SAFETY_MSS_getRegOffset(busSftyNode,&baseAddrOffst);
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
       retval = SDL_EBADARGS;
    }
    return retval;
}

/* Helper function to induce red error */
static void SDL_ECC_BUS_SAFETY_MSS_busSftyFitypeSet(SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType,
                                                SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S *baseAddrOffst)
{
    switch(fiType)
    {
        case SDL_ECC_BUS_SAFETY_FI_MAIN:
 	    HW_WR_FIELD32((baseAddrOffst->baseAddr+baseAddrOffst->busSftyFi),\
        SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_MAIN,\
		(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_ECC_BUS_SAFETY_FI_SAFE:

 	    HW_WR_FIELD32((baseAddrOffst->baseAddr+baseAddrOffst->busSftyFi),\
        SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_SAFE,\
		(uint8_t)((0x1U)<<(uint8_t)redType));
        break;
        case SDL_ECC_BUS_SAFETY_FI_GLOBAL_MAIN:
 	    HW_WR_FIELD32((baseAddrOffst->baseAddr+baseAddrOffst->busSftyFi),\
        SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_GLOBAL_MAIN,\
		0x1U);
        break;
        case SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE:
	    HW_WR_FIELD32((baseAddrOffst->baseAddr+baseAddrOffst->busSftyFi),\
        SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_GLOBAL_SAFE,\
		0x1U);
        break;
        default:
        /* do nothing */
        break;
    }
}

/********************************************************************************************************
* API to Execute SEC test
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3724
 */

int32_t SDL_ECC_BUS_SAFETY_MSS_secExecute(uint32_t busSftyNode,uint32_t addr, uint32_t wr_data)
{
    int32_t retval = SDL_EFAIL;
    SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S baseAddrOffst;
    retval = SDL_ECC_BUS_SAFETY_MSS_getRegOffset(busSftyNode,&baseAddrOffst);
    if ( SDL_PASS == retval)
    {
        {
            /* Check for EDMA dependency */
            if((bool)(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD>=busSftyNode) == (bool)0U)
            {
                /* Validate Start and End address of node */
                if ((baseAddrOffst.nodeStartAddr <= addr)&&(baseAddrOffst.nodeEndAddr>=addr))
                {
                    retval = SDL_PASS;
                }
                else
                {
                    retval = SDL_EBADARGS;
                }
            }
            else
            {
                retval = SDL_PASS;
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
                /* Check for EDMA dependency */
                if((SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD>=busSftyNode))
                {
                    retval = SDL_PASS;
                }
                else
                {
                    /* issue a bus transaction to node and read back */
                    *(volatile uint32_t *)addr = wr_data;
                    retval = SDL_PASS;
                }
                }
            else
            {
                retval = SDL_EBADARGS;
            }
        }
    }
    else
    {
        retval = SDL_EBADARGS;
    }
    return retval;
}

/********************************************************************************************************
* API to Execute Ded test
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
        {
            /* Check for EDMA dependency */
            if((bool)(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD>=busSftyNode) == (bool)0U)
            {
                /* Validate Start and End address of node */
                if ((baseAddrOffst.nodeStartAddr <= addr)&&(baseAddrOffst.nodeEndAddr>=addr))
                {
                    retval = SDL_PASS;
                }
                else
                {
                    retval = SDL_EBADARGS;
                }
                }
            else
            {
                retval = SDL_PASS;
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
                /* Check for EDMA dependency */
                if((SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD>=busSftyNode))
                {
                    retval = SDL_PASS;
                }
                else
                {
                    /* issue a bus transaction to node and read back */
                    *(volatile uint32_t *)addr = wr_data;
                    retval = SDL_PASS;
                }
            }
            else
            {
                retval = SDL_EBADARGS;
            }
        }
    }
    else
    {
        retval = SDL_EBADARGS;
    }
    return retval;
}

/********************************************************************************************************
* API to Execute RED test
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
            retval = SDL_EBADARGS;
        }
    }
    else
    {
        retval = SDL_EBADARGS;
    }
    return retval;
}

/********************************************************************************************************
* Helper API to get the offset address of MSS node
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
    /* MSS_CR5C_AHB*/
         case SDL_ECC_BUS_SAFETY_MSS_CR5C_AHB  :
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_CR5C_AHB_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_CR5C_AHB_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_CR5C_AHB_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_CR5C_AHB_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_CR5C_AHB_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_CR5C_AHB_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_CR5C_AHB_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = SDL_MSS_CTRL_R5SS0_CORE1_AHB_END;
            baseAddrOffst->nodeStartAddr        = SDL_MSS_CTRL_R5SS0_CORE1_AHB_BASE;
            break;
        }
        /* MSS_CR5D_AHB*/
         case SDL_ECC_BUS_SAFETY_MSS_CR5D_AHB  :
        {
            baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_CR5D_AHB_BUS_SAFETY_CTRL;
            baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_CR5D_AHB_BUS_SAFETY_ERR;
            baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_CR5D_AHB_BUS_SAFETY_FI;
            baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_CR5D_AHB_BUS_SAFETY_ERR_STAT_CMD;
            baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_CR5D_AHB_BUS_SAFETY_ERR_STAT_READ;
            baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_CR5D_AHB_BUS_SAFETY_ERR_STAT_WRITE;
            baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_CR5D_AHB_BUS_SAFETY_ERR_STAT_WRITERESP;
            baseAddrOffst->nodeEndAddr          = SDL_MSS_CTRL_R5SS1_CORE1_AHB_END;
            baseAddrOffst->nodeStartAddr        = SDL_MSS_CTRL_R5SS1_CORE1_AHB_BASE;
            break;
        }
          /* MSS_CR5A_AXI_RD*/
        case SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD  :
       {
           baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL;
           baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR;
           baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI;
           baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_CMD;
           baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_READ;
           baseAddrOffst->busSftyErrStatWr     = 0;
           baseAddrOffst->busSftyErrStatWrResp = 0;
           baseAddrOffst->nodeEndAddr          = 0;
           baseAddrOffst->nodeStartAddr        = 0;
           break;
       }
         /* MSS_CR5B_AXI_RD*/
       case SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD  :
      {
          baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL;
          baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR;
          baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI;
          baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_CMD;
          baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_READ;
          baseAddrOffst->busSftyErrStatWr     = 0;
          baseAddrOffst->busSftyErrStatWrResp = 0;
          baseAddrOffst->nodeEndAddr          = 0;
          baseAddrOffst->nodeStartAddr        = 0;
          break;
      }
        /* MSS_CR5C_AXI_RD*/
      case SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD  :
     {
         baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_CR5C_AXI_RD_BUS_SAFETY_CTRL;
         baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_CR5C_AXI_RD_BUS_SAFETY_ERR;
         baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_CR5C_AXI_RD_BUS_SAFETY_FI;
         baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_CR5C_AXI_RD_BUS_SAFETY_ERR_STAT_CMD;
         baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_CR5C_AXI_RD_BUS_SAFETY_ERR_STAT_READ;
         baseAddrOffst->busSftyErrStatWr     = 0;
         baseAddrOffst->busSftyErrStatWrResp = 0;
         baseAddrOffst->nodeEndAddr          = 0;
         baseAddrOffst->nodeStartAddr        = 0;
         break;
     }
        /* MSS_CR5D_AXI_RD*/
     case SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD  :
    {
        baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_CR5D_AXI_RD_BUS_SAFETY_CTRL;
        baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_CR5D_AXI_RD_BUS_SAFETY_ERR;
        baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_CR5D_AXI_RD_BUS_SAFETY_FI;
        baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_CR5D_AXI_RD_BUS_SAFETY_ERR_STAT_CMD;
        baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_CR5D_AXI_RD_BUS_SAFETY_ERR_STAT_READ;
        baseAddrOffst->busSftyErrStatWr     = 0;
        baseAddrOffst->busSftyErrStatWrResp = 0;
        baseAddrOffst->nodeEndAddr          = 0;
        baseAddrOffst->nodeStartAddr        = 0;
        break;
    }
        /* MSS_CR5A_AXI_WR*/
      case SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR  :
     {
         baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL;
         baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR;
         baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI;
         baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_CMD;
         baseAddrOffst->busSftyErrStatRd     = 0;
         baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE;
         baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
         baseAddrOffst->nodeEndAddr          = 0;
         baseAddrOffst->nodeStartAddr        = 0;
         break;
     }
       /* MSS_CR5B_AXI_WR*/
     case SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR  :
    {
        baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL;
        baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR;
        baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI;
        baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_CMD;
        baseAddrOffst->busSftyErrStatRd     = 0;
        baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE;
        baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
        baseAddrOffst->nodeEndAddr          = 0;
        baseAddrOffst->nodeStartAddr        = 0;
        break;
    }
      /* MSS_CR5C_AXI_WR*/
      case SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR  :
     {
         baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_CR5C_AXI_WR_BUS_SAFETY_CTRL;
         baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_CR5C_AXI_WR_BUS_SAFETY_ERR;
         baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_CR5C_AXI_WR_BUS_SAFETY_FI;
         baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_CR5C_AXI_WR_BUS_SAFETY_ERR_STAT_CMD;
         baseAddrOffst->busSftyErrStatRd     = 0;
         baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_CR5C_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE;
         baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_CR5C_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
         baseAddrOffst->nodeEndAddr          = 0;
         baseAddrOffst->nodeStartAddr        = 0;
         break;
     }
     /* MSS_CR5D_AXI_WR*/
     case SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR  :
    {
        baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_CR5D_AXI_WR_BUS_SAFETY_CTRL;
        baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_CR5D_AXI_WR_BUS_SAFETY_ERR;
        baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_CR5D_AXI_WR_BUS_SAFETY_FI;
        baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_CR5D_AXI_WR_BUS_SAFETY_ERR_STAT_CMD;
        baseAddrOffst->busSftyErrStatRd     = 0;
        baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_CR5D_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE;
        baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_CR5D_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP;
        baseAddrOffst->nodeEndAddr          = 0;
        baseAddrOffst->nodeStartAddr        = 0;
        break;
    }
      /* MSS_CR5A_AXI_S*/
    case SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S  :
   {
       baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL;
       baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR;
       baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI;
       baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_CMD;
       baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_READ;
       baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITE;
       baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP;
       baseAddrOffst->nodeEndAddr          = 0;
       baseAddrOffst->nodeStartAddr        = 0;
       break;
   }
     /* MSS_CR5B_AXI_S*/
   case SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S  :
  {
      baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL;
      baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR;
      baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI;
      baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_CMD;
      baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_READ;
      baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITE;
      baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP;
      baseAddrOffst->nodeEndAddr          = 0;
      baseAddrOffst->nodeStartAddr        = 0;
      break;
  }
     /* MSS_CR5C_AXI_S*/
    case SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S  :
   {
       baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_CR5C_AXI_S_BUS_SAFETY_CTRL;
       baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_CR5C_AXI_S_BUS_SAFETY_ERR;
       baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_CR5C_AXI_S_BUS_SAFETY_FI;
       baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_CR5C_AXI_S_BUS_SAFETY_ERR_STAT_CMD;
       baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_CR5C_AXI_S_BUS_SAFETY_ERR_STAT_READ;
       baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_CR5C_AXI_S_BUS_SAFETY_ERR_STAT_WRITE;
       baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_CR5C_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP;
       baseAddrOffst->nodeEndAddr          = 0;
       baseAddrOffst->nodeStartAddr        = 0;
       break;
   }
    /* MSS_CR5D_AXI_S*/
   case SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S  :
  {
      baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MSS_CR5D_AXI_S_BUS_SAFETY_CTRL;
      baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MSS_CR5D_AXI_S_BUS_SAFETY_ERR;
      baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MSS_CR5D_AXI_S_BUS_SAFETY_FI;
      baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MSS_CR5D_AXI_S_BUS_SAFETY_ERR_STAT_CMD;
      baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MSS_CR5D_AXI_S_BUS_SAFETY_ERR_STAT_READ;
      baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MSS_CR5D_AXI_S_BUS_SAFETY_ERR_STAT_WRITE;
      baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MSS_CR5D_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP;
      baseAddrOffst->nodeEndAddr          = 0;
      baseAddrOffst->nodeStartAddr        = 0;
      break;
  }
   /* MSS_MBOX*/
    case SDL_ECC_BUS_SAFETY_MSS_MBOX  :
    {
        baseAddrOffst->busSftyCtrl          = SDL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_CTRL;
        baseAddrOffst->busSftyErr           = SDL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR;
        baseAddrOffst->busSftyFi            = SDL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI;
        baseAddrOffst->busSftyErrStatCmd    = SDL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_CMD;
        baseAddrOffst->busSftyErrStatRd     = SDL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_READ;
        baseAddrOffst->busSftyErrStatWr     = SDL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_WRITE;
        baseAddrOffst->busSftyErrStatWrResp = SDL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_WRITERESP;
        baseAddrOffst->nodeEndAddr          = SDL_MBOX_SRAM_U_BASE_END;
        baseAddrOffst->nodeStartAddr        = SDL_MBOX_SRAM_U_BASE;
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
