# ECC BUS SAFETY {#SDL_ECC_BUS_SAFETY_PAGE}

[TOC]

# Features Supported

ECC BUS SAFETY module consists ECC Bus Safety Diagnostics.

* BUS Safety Errors : This includes SEC , DED and RED error injection on the specified node

  Note : SEC - Single Error Correction, DED - Double Error Correction, RED - Redundancy Error Correction

The module supports below API's for the application

* API to support and induce error on particular node bus.
* API to support interrupt configuration

## SysConfig Features

- None

## Features NOT Supported

- None

## Important Usage Guidelines

- None

## Example Usage

The following shows an example of SDL ECC BUS SAFETY API usage by the application for Error Injection Tests.

Include the below file to access the APIs

\code{.c}
#include <sdl/sdl_ecc_bus_safety.h>
\endcode

Induce the error SEC in Interrupt Method for MSS Nodes
\code{.c}
    ret_val = SDL_ECC_BUS_SAFETY_MSS_secExecute(busSftyNode,addr,writeData);
    if(ret_val !=SDL_PASS )
    {
        ret_val = SDL_EFAIL;
    }
    else
    {
        if(((SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD>=busSftyNode)) ||
            (SDL_ECC_BUS_SAFETY_MSS_PCR == busSftyNode) || (SDL_ECC_BUS_SAFETY_MSS_PCR2 == busSftyNode ))
        {
            /* get EDMA parameter */
            SDL_ECC_BUS_SAFETY_MSS_getEDMAParameters(busSftyNode,&dmaCh, &tcc, &param, &queNum, &edmaNum );
            /* initiate the EDMA transfer */
            test_edmaATransfer(busSftyNode,dmaCh,tcc,param,queNum,edmaNum );
        }
        else if (SDL_ECC_BUS_SAFETY_MSS_CPSW ==busSftyNode  )
        {
            /* CPSW SetUP */
            setup_CPSW();
            /* CPSW Transfer */
            cpsw_transfer();
        }
        else
        {
            /* do nothing */
        }
        /* wait for error notification from ESM, or for timeout */
        /* fault injection gets deasserted in ISR */
        while((mssSecFlag!=TRUE) && (timeout!=0U))
        {
            timeout--;
        }
        if(mssSecFlag==TRUE)
        {
            mssSecFlag=FALSE;
            ret_val = SDL_PASS;
        }
        else
        {
            ret_val = SDL_EFAIL;
        }
    }
\endcode

Induce the error DED in Interrupt Method for MSS Nodes
\code{.c}
    ret_val = SDL_ECC_BUS_SAFETY_MSS_dedExecute(busSftyNode,addr,writeData);
    if(ret_val !=SDL_PASS )
    {
        ret_val = SDL_EFAIL;
    }
    else
    {
        if(((SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD>=busSftyNode)) ||
            (SDL_ECC_BUS_SAFETY_MSS_PCR == busSftyNode) || (SDL_ECC_BUS_SAFETY_MSS_PCR2 == busSftyNode ))
        {
        SDL_ECC_BUS_SAFETY_MSS_getEDMAParameters(busSftyNode,&dmaCh, &tcc, &param, &queNum, &edmaNum );
        test_edmaATransfer(busSftyNode,dmaCh,tcc,param,queNum,edmaNum );
        }
        else if (SDL_ECC_BUS_SAFETY_MSS_CPSW ==busSftyNode  )
        {
            /* CPSW SetUP */
            setup_CPSW();
            /* CPSW Transfer */
            cpsw_transfer();
        }
        else
        {
            /* do nothing */
        }
        /* wait for error notification from ESM, or for timeout */
        /* fault injection gets deasserted in ISR */
        while((SDL_MSS_intrFlg[busSftyNode]!=TRUE) && (timeout!=0U))
        {
            timeout--;
        }
        if(SDL_MSS_intrFlg[busSftyNode]==TRUE)
        {
            SDL_MSS_intrFlg[busSftyNode]=FALSE;
            ret_val = SDL_PASS;
        }
        else
        {
             ret_val = SDL_EFAIL;
        }
    }

\endcode

Induce the error RED in Interrupt Method for MSS Nodes
\code{.c}
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_ECC_BUS_SAFETY_TIMEOUT;
    ret_val= SDL_ECC_BUS_SAFETY_MSS_redExecute(busSftyNode, fiType, redType);
    if(ret_val !=SDL_PASS )
    {
        ret_val = SDL_EFAIL;
    }
    else
    {
        /* wait for error notification from ESM, or for timeout */
        /* fault injection gets deasserted in ISR */
        while((SDL_MSS_intrFlg[busSftyNode]!=TRUE) && (timeout!=0U))
        {
            timeout--;
        }
        if(SDL_MSS_intrFlg[busSftyNode]==TRUE)
        {
            SDL_MSS_intrFlg[busSftyNode]=FALSE;
            ret_val = SDL_PASS;
        }
        else
        {
            ret_val = SDL_EFAIL;
        }
    }

\endcode

Induce the error SEC in Polling Method for DSS Nodes
\code{.c}
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

\endcode

Induce the error DED in Polling Method for DSS Nodes
\code{.c}
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
\endcode

Induce the error RED in Polling Method for DSS Nodes
\code{.c}
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
\endcode

Induce the error SEC in Interrupt Method for DSS Nodes
\code{.c}
    ret_val = SDL_ECC_BUS_SAFETY_DSS_secExecute(busSftyNode,addr,writeData);
    if(ret_val !=SDL_PASS )
    {
        ret_val = SDL_EFAIL;
    }
    else
    {
        if(((SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD>=busSftyNode))||(SDL_ECC_BUS_SAFETY_DSS_PCR==busSftyNode))
        {
            SDL_ECC_BUS_SAFETY_DSS_getEDMAParameters(busSftyNode,&dmaCh, &tcc, &param, &queNum, &edmaNum );
            test_edmaATransfer(busSftyNode, dmaCh,tcc,param,queNum,edmaNum );
        }
        else
        {
            /* Do Nothing */
        }
        /* wait for error notification from ESM, or for timeout */
        /* fault injection gets deasserted in ISR */
        while((dssSecFlag!=TRUE) && (timeout!=0U))
        {
            timeout--;
        }
        if(dssSecFlag==TRUE)
        {
            dssSecFlag=FALSE;
            ret_val = SDL_PASS;
        }
        else
        {
            ret_val = SDL_EFAIL;
        }
    }
\endcode

Induce the error DED in Interrupt Method for DSS Nodes
\code{.c}
    ret_val = SDL_ECC_BUS_SAFETY_DSS_dedExecute(busSftyNode,addr,writeData);
    if(ret_val !=SDL_PASS )
    {
        ret_val = SDL_EFAIL;
    }
    else
    {
        if(((SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD>=busSftyNode))||(SDL_ECC_BUS_SAFETY_DSS_PCR==busSftyNode))
        {
            SDL_ECC_BUS_SAFETY_DSS_getEDMAParameters(busSftyNode,&dmaCh, &tcc, &param, &queNum, &edmaNum );
            test_edmaATransfer(busSftyNode,dmaCh,tcc,param,queNum,edmaNum );
        }
        else
        {
            /* Do Nothing */
        }
        /* wait for error notification from ESM, or for timeout */
        /* fault injection gets deasserted in ISR */
        while((SDL_DSS_intrFlg[busSftyNode]!=TRUE) && (timeout!=0U))
        {
            timeout--;
        }
        if(SDL_DSS_intrFlg[busSftyNode]==TRUE)
        {
            SDL_DSS_intrFlg[busSftyNode]=FALSE;
            ret_val = SDL_PASS;
        }
        else
        {
            ret_val = SDL_EFAIL;
        }
    }
\endcode

Induce the error RED in Interrupt Method for DSS Nodes
\code{.c}
    ret_val= SDL_ECC_BUS_SAFETY_DSS_redExecute(busSftyNode, fiType, redType);
    if(ret_val !=SDL_PASS )
    {
        ret_val = SDL_EFAIL;
    }
    else
    {
        /* wait for error notification from ESM, or for timeout */
        /* fault injection gets deasserted in ISR */
        while((SDL_DSS_intrFlg[busSftyNode]!=TRUE) && (timeout!=0U))
        {
            timeout--;
        }
        if(SDL_DSS_intrFlg[busSftyNode]==TRUE)
        {
            SDL_DSS_intrFlg[busSftyNode]=FALSE;
            ret_val = SDL_PASS;
        }
        else
        {
            ret_val = SDL_EFAIL;
        }
    }
\endcode
## API

\ref SDL_ECC_BUS_SAFETY_MODULE
