/**
 * @file  csl_emif.c
 *
 * @brief
 *  C implementation file for the EMIF module CSL-FL for ECC.
 *
 *  Contains the different control command and status query functions definitions
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2021, Texas Instruments, Inc.
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
/** \brief This is to disable HW_SYNC_BARRIER for register access */
#define MEM_BARRIER_DISABLE

#include <drivers/ddr/v0/cslr_emif.h>
#include <drivers/ddr/v0/csl_emif.h>

/*=============================================================================
 *  Internal function Prototype
 *===========================================================================*/
static void CSL_emifConfigController(CSL_emif_sscfgRegs *pEmifSsRegs,
                                     CSL_EmifConfig *pCfg);
static bool CSL_emifIsEmifConfigValid(CSL_emif_sscfgRegs *pEmifSsRegs,
                                      CSL_EmifConfig *pCfg );

/*=============================================================================
 *  Internal functions
 *===========================================================================*/

/*=============================================================================
 *  This internal function configures the ECC related parameters
 *===========================================================================*/
static void CSL_emifConfigController(CSL_emif_sscfgRegs *pEmifSsRegs,
                                     CSL_EmifConfig *pCfg)
{
    uint32_t    regVal = 0U;

    /*-------------------------------------------------------------------------
     *  Configure ECC related registers
     *-----------------------------------------------------------------------*/
    pEmifSsRegs->ECC_R0_STR_ADDR_REG = ((pCfg->pMemEccCfg.startAddr[0]) >> 16)
                                        & CSL_EMIF_SSCFG_ECC_R0_STR_ADDR_REG_ECC_STR_ADR_0_MASK;
    pEmifSsRegs->ECC_R0_END_ADDR_REG = ((pCfg->pMemEccCfg.endAddr[0]) >> 16)
                                        & CSL_EMIF_SSCFG_ECC_R0_END_ADDR_REG_ECC_END_ADR_0_MASK;
    pEmifSsRegs->ECC_R1_STR_ADDR_REG = ((pCfg->pMemEccCfg.startAddr[1]) >> 16)
                                        & CSL_EMIF_SSCFG_ECC_R1_STR_ADDR_REG_ECC_STR_ADR_1_MASK;
    pEmifSsRegs->ECC_R1_END_ADDR_REG = ((pCfg->pMemEccCfg.endAddr[1]) >> 16)
                                        & CSL_EMIF_SSCFG_ECC_R1_END_ADDR_REG_ECC_END_ADR_1_MASK;
    pEmifSsRegs->ECC_R2_STR_ADDR_REG = ((pCfg->pMemEccCfg.startAddr[2]) >> 16)
                                        & CSL_EMIF_SSCFG_ECC_R2_STR_ADDR_REG_ECC_STR_ADR_2_MASK;
    pEmifSsRegs->ECC_R2_END_ADDR_REG = ((pCfg->pMemEccCfg.endAddr[2]) >> 16)
                                        & CSL_EMIF_SSCFG_ECC_R2_END_ADDR_REG_ECC_END_ADR_2_MASK;		

    if( pCfg->bEnableMemoryECC == (bool)true )
    {
        /* Enable ECC bit */
        regVal = CSL_FMK(EMIF_SSCFG_ECC_CTRL_REG_ECC_EN, 1U);
        /* Enable Read modify write */
        regVal |= CSL_FMK(EMIF_SSCFG_ECC_CTRL_REG_RMW_EN, (pCfg->bReadModifyWriteEnable ? 1U : 0U));
        /* Enable ECC check */
        regVal |= CSL_FMK(EMIF_SSCFG_ECC_CTRL_REG_ECC_CK,  (pCfg->bECCCheck ? 1U : 0U));
        /* Enable Write alloc */
        regVal |= CSL_FMK(EMIF_SSCFG_ECC_CTRL_REG_WR_ALLOC,  (pCfg->bWriteAlloc ? 1U : 0U));
        /* Set threshold */        
        regVal |= CSL_FMK(EMIF_SSCFG_ECC_CTRL_REG_COR_ECC_THRESH, pCfg->ECCThreshold);        
        CSL_REG32_WR( &pEmifSsRegs->ECC_CTRL_REG, regVal );
        CSL_REG32_WR( &pEmifSsRegs->ECC_1B_ERR_THRSH_REG, pCfg->ECCThreshold );
    }
    else
    {
        /* Disable ECC bit */
        regVal = CSL_FMK(EMIF_SSCFG_ECC_CTRL_REG_ECC_EN, 0U);
        CSL_REG32_WR( &pEmifSsRegs->ECC_CTRL_REG,  regVal );
    }
}

static bool CSL_emifIsEmifConfigValid(CSL_emif_sscfgRegs *pEmifSsRegs, CSL_EmifConfig *pCfg )
{
    bool bParmsOk = (bool)true;

    /*-------------------------------------------------------------------------
     *  Do NULL checks
     *-----------------------------------------------------------------------*/
    if((pEmifSsRegs == NULL) || (pCfg == NULL))
    {
        bParmsOk = (bool)false;
    }

    return bParmsOk;
}

/*=============================================================================
 *  CSL_emifGetRevision
 *
 *  This function returns the revision of the EMIF subsystem module.
 *===========================================================================*/
uint32_t CSL_emifGetRevision(CSL_emif_sscfgRegs *pEmifSsRegs)
{

    return CSL_REG32_RD(&pEmifSsRegs->SS_ID_REV_REG);
}

/*=============================================================================
 *  CSL_emifConfig
 *
 *  This function performs the proper configuration sequence for the SDRAM
 *  controller and PHY per the specified configuration parameters.
 *===========================================================================*/
int32_t CSL_emifConfig(CSL_emif_sscfgRegs *pEmifSsRegs, CSL_EmifConfig *pCfg)
{
    int32_t  retVal = CSL_EMIF_CONFIG_RETURN_OK;

    if( CSL_emifIsEmifConfigValid(pEmifSsRegs, pCfg ) == (bool)true )
    {
        /*---------------------------------------------------------------------
         *  Program the emif controller
         *-------------------------------------------------------------------*/
        CSL_emifConfigController(pEmifSsRegs,  pCfg );
    }
    else
    {
        retVal = CSL_EMIF_CONFIG_RETURN_INVALID_CFG;
    }

    return retVal;
}

/*=============================================================================
 *  CSL_emifEnableECC
 *
 *  This function enables ECC
 *===========================================================================*/
int32_t CSL_emifEnableECC(CSL_emif_sscfgRegs *pEmifSsRegs)
{
    int32_t  retVal = CSL_EMIF_CONFIG_RETURN_OK;
    uint32_t    regVal = 0U;

    if( pEmifSsRegs != NULL )
    {
         regVal = pEmifSsRegs->ECC_CTRL_REG;
         regVal |= CSL_FMK(EMIF_SSCFG_ECC_CTRL_REG_ECC_EN, 1U);
         CSL_REG32_WR( &pEmifSsRegs->ECC_CTRL_REG, regVal );
    }
    else
    {
        retVal = CSL_EMIF_CONFIG_RETURN_INVALID_CFG;
    }

    return retVal;
}

/*=============================================================================
 *  CSL_emifDisableECC
 *
 *  This function disables ECC
 *===========================================================================*/
int32_t CSL_emifDisableECC(CSL_emif_sscfgRegs *pEmifSsRegs)
{
    int32_t  retVal = CSL_EMIF_CONFIG_RETURN_OK;
    uint32_t    regVal = 0U;

    if( pEmifSsRegs != NULL )
    {
         regVal = pEmifSsRegs->ECC_CTRL_REG;
         regVal &= (~CSL_FMK(EMIF_SSCFG_ECC_CTRL_REG_ECC_EN, 1U));
         CSL_REG32_WR( &pEmifSsRegs->ECC_CTRL_REG, regVal );
    }
    else
    {
        retVal = CSL_EMIF_CONFIG_RETURN_INVALID_CFG;
    }

    return retVal;
}

/*=============================================================================
 *  CSL_emifGetECCErrorInfo
 *
 *  This function gets the ECC error information
 *===========================================================================*/
int32_t CSL_emifGetECCErrorInfo(CSL_emif_sscfgRegs *pEmifSsRegs,
                                CSL_ECCErrorInfo *pECCErrorInfo)
{
    int32_t  retVal = CSL_EMIF_CONFIG_RETURN_OK;
    uint32_t bitIndex;
    bool bitSet = false;
    uint32_t errorMask;

    if ((pEmifSsRegs == NULL) || (pECCErrorInfo == NULL))
    {
        retVal = CSL_EMIF_CONFIG_RETURN_INVALID_CFG;
    }
    else
    {
        /* Get 1 bit error count */
        pECCErrorInfo->singlebitErrorCount = pEmifSsRegs->ECC_1B_ERR_CNT_REG;

        pECCErrorInfo->singlebitErrorAddress = 0U;
        if ( pEmifSsRegs->ECC_1B_ERR_ADR_LOG_REG != 0u)
        {
            /* Get the error address */
            pECCErrorInfo->singlebitErrorAddress = (pEmifSsRegs->ECC_1B_ERR_ADR_LOG_REG << 4u);
            /* Get the error mask */
            errorMask = pEmifSsRegs->ECC_1B_ERR_MSK_LOG_REG;
            /* Check the error mask to determine more granular address */
            bitIndex = 0;
            do
            {
                if ((errorMask & (1u << bitIndex)) != 0u)
                {
                    bitSet = true;
                }
                bitIndex++;
            } while ((bitIndex < 4)  && (!bitSet));

            if (bitSet == true)
            {
                pECCErrorInfo->singlebitErrorAddress |= ((bitIndex-1) << 1u);
            }
        }

        pECCErrorInfo->doublebitErrorAddress  = 0U;
        
        if (pEmifSsRegs->ECC_2B_ERR_ADR_LOG_REG != 0u)
        {
            /* Get the error address */
            pECCErrorInfo->doublebitErrorAddress = (pEmifSsRegs->ECC_2B_ERR_ADR_LOG_REG << 4u);
            /* Get the error mask */
            errorMask = pEmifSsRegs->ECC_2B_ERR_MSK_LOG_REG;
            /* Check the error mask to determine more granular address */
            bitIndex = 0;
            do
            {
                if ((errorMask & (1u << bitIndex)) != 0u)
                {
                    bitSet = true;
                }
                bitIndex++;
            } while ((bitIndex < 4)  && (!bitSet));
            if (bitSet == true)
            {
               pECCErrorInfo->doublebitErrorAddress |= ((bitIndex-1) << 1u);
            }
        }
    }

    return retVal;
}

/*=============================================================================
 *  CSL_emifClearAllECCErrors
 *
 *  This function clears all ECC errors
 *===========================================================================*/
int32_t CSL_emifClearAllECCErrors(CSL_emif_sscfgRegs *pEmifSsRegs)
{
    int32_t  retVal = CSL_EMIF_CONFIG_RETURN_OK;

    if( pEmifSsRegs != NULL )
    {
        /* Clear error 1 bit, 2 bit error address and log */
        CSL_REG32_WR( &pEmifSsRegs->ECC_1B_ERR_ADR_LOG_REG, 1u );
        CSL_REG32_WR( &pEmifSsRegs->ECC_2B_ERR_ADR_LOG_REG, 1u );
        do
        {
            CSL_REG32_WR( &pEmifSsRegs->ECC_1B_ERR_CNT_REG, 1u );
        } while (pEmifSsRegs->ECC_1B_ERR_CNT_REG != 0);
    }
    else
    {
        retVal = CSL_EMIF_CONFIG_RETURN_INVALID_CFG;
    }

    return retVal;
}

/*=============================================================================
 *  CSL_emifClearECCError
 *
 *  This function clears specific ECC error
 *===========================================================================*/
int32_t CSL_emifClearECCError(CSL_emif_sscfgRegs *pEmifSsRegs,
                              CSL_EmifECCErrorType errorType)
{
    int32_t  retVal = CSL_EMIF_CONFIG_RETURN_OK;

    if( pEmifSsRegs != NULL )
    {
        switch (errorType)
        {
            case CSL_EMIF_ECC_ERROR_TYPE_SINGLE_BIT:
                /* Clear error 1 bit error address and count */
                CSL_REG32_WR( &pEmifSsRegs->ECC_1B_ERR_ADR_LOG_REG, 1u );
                CSL_REG32_WR( &pEmifSsRegs->ECC_1B_ERR_CNT_REG, 1u );
                break;

            case CSL_EMIF_ECC_ERROR_TYPE_DOUBLE_BIT:
                /* Clear error 2 bit error address */
                CSL_REG32_WR( &pEmifSsRegs->ECC_2B_ERR_ADR_LOG_REG, 1u );
                break;

            default:
                break;
        }
    }
    else
    {
        retVal = CSL_EMIF_CONFIG_RETURN_INVALID_CFG;
    }

    return retVal;
}

/*=============================================================================
 *  CSL_emifEnableECCInterrupts
 *
 *  This function enables ECC interrupts
 *===========================================================================*/
int32_t CSL_emifEnableECCInterrupts(CSL_emif_sscfgRegs *pEmifSsRegs,
                                    uint32_t eccInterruptBitmap)
{
    int32_t  retVal = CSL_EMIF_CONFIG_RETURN_OK;

    if( pEmifSsRegs != NULL )
    {
         CSL_REG32_WR( &pEmifSsRegs->V2A_INT_SET_REG, eccInterruptBitmap );
    }
    else
    {
        retVal = CSL_EMIF_CONFIG_RETURN_INVALID_CFG;
    }

    return retVal;
}

/*=============================================================================
 *  CSL_emifDisableECCInterrupts
 *
 *  This function disables ECC interrupts
 *===========================================================================*/
int32_t CSL_emifDisableECCInterrupts(CSL_emif_sscfgRegs *pEmifSsRegs,
                                     uint32_t eccInterruptBitmap)
{
    int32_t  retVal = CSL_EMIF_CONFIG_RETURN_OK;

    if( pEmifSsRegs != NULL )
    {
         CSL_REG32_WR( &pEmifSsRegs->V2A_INT_CLR_REG, eccInterruptBitmap );
    }
    else
    {
        retVal = CSL_EMIF_CONFIG_RETURN_INVALID_CFG;
    }

    return retVal;
}

/*=============================================================================
 *  CSL_emifClearECCInterruptStatus
 *
 *  This function clears ECC interrupt status
 *===========================================================================*/
int32_t CSL_emifClearECCInterruptStatus(CSL_emif_sscfgRegs *pEmifSsRegs,
                                        uint32_t eccInterruptBitmap)
{
    int32_t  retVal = CSL_EMIF_CONFIG_RETURN_OK;

    if( pEmifSsRegs != NULL )
    {
         CSL_REG32_WR( &pEmifSsRegs->V2A_INT_STAT_REG, eccInterruptBitmap );
    }
    else
    {
        retVal = CSL_EMIF_CONFIG_RETURN_INVALID_CFG;
    }

    return retVal;
}
