/**
 * @file  csl_mdio.c
 *
 * @brief
 *  Functional layer implementation of CSL MDIO IP
 *
 *  Contains the different control command and status query functions definitions
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2018, Texas Instruments, Inc.
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

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <drivers/hw_include/csl_types.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/hw_types.h>
#include <csl_mdio.h>

void CSL_MDIO_getVersionInfo(
    CSL_mdioHandle          hMdioRegs,
    CSL_MDIO_VERSION*       mdioVersionInfo
)
{
    mdioVersionInfo->revMin     =   CSL_FEXT (hMdioRegs->VERSION_REG, MDIO_VERSION_REG_REVMINOR);
    mdioVersionInfo->revMaj     =   CSL_FEXT (hMdioRegs->VERSION_REG, MDIO_VERSION_REG_REVMAJ);
    mdioVersionInfo->modId      =   CSL_FEXT (hMdioRegs->VERSION_REG, MDIO_VERSION_REG_MODID);
    mdioVersionInfo->scheme     =   CSL_FEXT (hMdioRegs->VERSION_REG, MDIO_VERSION_REG_SCHEME);
    mdioVersionInfo->revRtl     =   CSL_FEXT (hMdioRegs->VERSION_REG, MDIO_VERSION_REG_REVRTL);
    mdioVersionInfo->bu         =   CSL_FEXT (hMdioRegs->VERSION_REG, MDIO_VERSION_REG_BU);

    return;
}

uint16_t CSL_MDIO_getClkDivVal(
    CSL_mdioHandle          hMdioRegs
)
{
    return (uint16_t)CSL_FEXT (hMdioRegs->CONTROL_REG, MDIO_CONTROL_REG_CLKDIV);
}

void CSL_MDIO_setClkDivVal(
    CSL_mdioHandle          hMdioRegs,
    uint16_t                 clkDivVal
)
{
    CSL_FINS (hMdioRegs->CONTROL_REG, MDIO_CONTROL_REG_CLKDIV, (uint32_t)clkDivVal);

    return;
}

uint32_t CSL_MDIO_isStateMachineEnabled(
    CSL_mdioHandle          hMdioRegs
)
{
    return CSL_FEXT (hMdioRegs->CONTROL_REG, MDIO_CONTROL_REG_ENABLE);
}

void CSL_MDIO_enableStateMachine(
    CSL_mdioHandle          hMdioRegs
)
{
    CSL_FINS (hMdioRegs->CONTROL_REG, MDIO_CONTROL_REG_ENABLE, 1U);

    return;
}

void CSL_MDIO_disableStateMachine(
    CSL_mdioHandle          hMdioRegs
)
{
    CSL_FINS (hMdioRegs->CONTROL_REG, MDIO_CONTROL_REG_ENABLE, 0U);

    return;
}

uint32_t CSL_MDIO_isPhyAlive(
    CSL_mdioHandle          hMdioRegs,
    uint32_t                  phyAddr
)
{
    return CSL_FEXTR (hMdioRegs->ALIVE_REG, phyAddr, phyAddr);
}


uint32_t CSL_MDIO_isPhyLinked(
    CSL_mdioHandle          hMdioRegs,
    uint32_t                  phyAddr
)
{
    return CSL_FEXTR (hMdioRegs->LINK_REG, phyAddr, phyAddr);
}

uint32_t CSL_MDIO_isUnmaskedLinkStatusChangeIntSet(
    CSL_mdioHandle          hMdioRegs,
    uint32_t                  index
)
{
    return CSL_FEXTR (hMdioRegs->LINK_INT_RAW_REG, index, index);
}

void CSL_MDIO_clearUnmaskedLinkStatusChangeInt(
    CSL_mdioHandle          hMdioRegs,
    uint32_t                  index
)
{
    CSL_FINSR (hMdioRegs->LINK_INT_RAW_REG, index, index, 1U);
}

uint32_t CSL_MDIO_isStatusChangeModeInterruptEnabled(
    CSL_mdioHandle          hMdioRegs
)
{
    return CSL_FEXT(hMdioRegs->LINK_INT_MASK_SET_REG,
                    MDIO_LINK_INT_MASK_SET_REG_LINKINTMASKSET);
}

void CSL_MDIO_enableStatusChangeModeInterrupt(
    CSL_mdioHandle          hMdioRegs
)
{
    CSL_FINS(hMdioRegs->LINK_INT_MASK_SET_REG,
             MDIO_LINK_INT_MASK_SET_REG_LINKINTMASKSET, 1U);
}

void CSL_MDIO_disableStatusChangeModeInterrupt(
    CSL_mdioHandle          hMdioRegs
)
{
    CSL_FINS(hMdioRegs->LINK_INT_MASK_CLEAR_REG,
             MDIO_LINK_INT_MASK_CLEAR_REG_LINKINTMASKCLR, 1U);
}

uint32_t  CSL_MDIO_phyRegRead2(CSL_mdioHandle hMdioRegs,
                    uint32_t userGroup,
                    uint32_t phyAddr,
                    uint32_t regNum,
                    uint16_t *pData)
{
    uint32_t regVal = 0U;
    uint32_t ret_val = 0U;

    /* Wait till transaction completion if any */
    while(CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_EN_0x1 ==
		  CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG,
                 MDIO_USER_GROUP_USER_ACCESS_REG_GO))
    {}
    CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_GO, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_EN_0x1);
    CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_WRITE, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_READ);
    CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_PHYADR, phyAddr);
    CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_REGADR, regNum);
    CSL_REG_WR(&hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG, regVal);

    /* wait for command completion */
    while(CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_EN_0x1 ==
		  CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG,
                 MDIO_USER_GROUP_USER_ACCESS_REG_GO))
    {}

    /* Store the data if the read is acknowledged */
    if(CSL_MDIO_USER_GROUP_USER_ACCESS_REG_ACK_PASS ==
        CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG,
        MDIO_USER_GROUP_USER_ACCESS_REG_ACK))
    {
        *pData = (uint16_t)(CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG,
            MDIO_USER_GROUP_USER_ACCESS_REG_DATA));
        ret_val = (uint32_t)TRUE;
    }
    else
    {
        ret_val = (uint32_t)FALSE;
    }

    return(ret_val);
}

bool  CSL_MDIO_phyRegReadAsyncTrigger(CSL_mdioHandle hMdioRegs,
                    uint32_t userGroup,
                    uint32_t phyAddr,
                    uint32_t regNum)
{
    uint32_t regVal = 0U;
    bool accessRegBusy = true;

    /* Wait till transaction completion if any */
    if(CSL_MDIO_isPhyRegAccessComplete(hMdioRegs, userGroup))
    {
        accessRegBusy = false;
    }

    if(accessRegBusy == false)
    {
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_GO, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_EN_0x1);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_WRITE, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_READ);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_PHYADR, phyAddr);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_REGADR, regNum);
        CSL_REG_WR(&hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG, regVal);
    }
    return(accessRegBusy);
}

uint32_t  CSL_MDIO_phyRegReadAsyncComplete(CSL_mdioHandle hMdioRegs,
                    uint32_t userGroup,
                    uint32_t phyAddr,
                    uint32_t regNum,
                    uint16_t *pData)
{
    uint32_t ret_val = CSL_PASS;
    bool accessRegBusy = true;

    /* wait for command completion */
    if(CSL_MDIO_isPhyRegAccessComplete(hMdioRegs, userGroup))
    {
        accessRegBusy = false;
    }

    if(accessRegBusy == false)
    {
        /* Store the data if the read is acknowledged */
        if(CSL_MDIO_USER_GROUP_USER_ACCESS_REG_ACK_PASS ==
            CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG,
            MDIO_USER_GROUP_USER_ACCESS_REG_ACK))
        {
            uint32_t regPhyAddr, regPhyRegNum;

            regPhyAddr = CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG, MDIO_USER_GROUP_USER_ACCESS_REG_PHYADR);
            regPhyRegNum = CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG, MDIO_USER_GROUP_USER_ACCESS_REG_REGADR);
            if ((regPhyAddr == phyAddr)
                &&
                (regPhyRegNum == regNum))
            {
                *pData = (uint16_t)(CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG,
                    MDIO_USER_GROUP_USER_ACCESS_REG_DATA));
                ret_val = CSL_PASS;
            }
            else
            {
                ret_val = CSL_EFAIL;
            }
        }
        else
        {
            ret_val = CSL_EFAIL;
        }
    }
    else
    {
        ret_val = CSL_ETIMEOUT;
    }

    return ret_val;
}

void CSL_MDIO_phyRegWrite2(CSL_mdioHandle hMdioRegs, uint32_t userGroup, uint32_t phyAddr, uint32_t regNum, uint16_t wrVal)
{
    uint32_t regVal = 0U;

    /* Wait till transaction completion if any */
    while(CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_EN_0x1 ==
		  CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG,
                 MDIO_USER_GROUP_USER_ACCESS_REG_GO))
    {}

    CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_GO, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_EN_0x1);
    CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_WRITE, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_WRITE);
    CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_PHYADR, phyAddr);
    CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_REGADR, regNum);
    CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_DATA, wrVal);
    CSL_REG_WR(&hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG, regVal);

    /* wait for command completion */
    while(CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_EN_0x1 ==
		  CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG,
                 MDIO_USER_GROUP_USER_ACCESS_REG_GO))
    {}
}

bool CSL_MDIO_phyRegWriteAsyncTrigger(CSL_mdioHandle hMdioRegs,
                    uint32_t userGroup,
                    uint32_t phyAddr,
                    uint32_t regNum,
                    uint16_t wrVal)
{
    uint32_t regVal = 0U;
    bool accessRegBusy = false;

    /* Wait till transaction completion if any */
    if(CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_EN_0x1 ==
		  CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG,
                 MDIO_USER_GROUP_USER_ACCESS_REG_GO))
    {
        accessRegBusy = true;
    }

    if(accessRegBusy == false)
    {
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_GO, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_EN_0x1);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_WRITE, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_WRITE);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_PHYADR, phyAddr);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_REGADR, regNum);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_DATA, wrVal);
        CSL_REG_WR(&hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG, regVal);
    }
    return accessRegBusy;
}

uint32_t  CSL_MDIO_phyRegWriteAsyncComplete(CSL_mdioHandle hMdioRegs,
                    uint32_t userGroup,
                    uint32_t phyAddr,
                    uint32_t regNum,
                    uint32_t wrVal)
{
    uint32_t ret_val = CSL_PASS;
    bool accessRegBusy = true;

    /* wait for command completion */
    if(CSL_MDIO_isPhyRegAccessComplete(hMdioRegs, userGroup))
    {
        accessRegBusy = false;
    }

    if(accessRegBusy == false)
    {
        uint32_t regPhyAddr, regPhyRegNum, regWrVal;

        regPhyAddr = CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG, MDIO_USER_GROUP_USER_ACCESS_REG_PHYADR);
        regPhyRegNum = CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG, MDIO_USER_GROUP_USER_ACCESS_REG_REGADR);
        regWrVal = CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG, MDIO_USER_GROUP_USER_ACCESS_REG_DATA);
        if ((regPhyAddr == phyAddr)
            &&
            (regPhyRegNum == regNum)
            &&
            (regWrVal == wrVal))
        {
            ret_val = CSL_PASS;
        }
        else
        {
            ret_val = CSL_EFAIL;
        }
    }
    else
    {
        ret_val = CSL_ETIMEOUT;
    }
    return ret_val;
}

void CSL_MDIO_setClause45EnableMask(CSL_mdioHandle hMdioRegs,
                                    uint32_t clause45EnableMask)
{
    hMdioRegs->CLAUS45_REG = clause45EnableMask;
}

uint32_t CSL_MDIO_getClause45EnableMask(CSL_mdioHandle hMdioRegs)
{
    return hMdioRegs->CLAUS45_REG;
}

int32_t CSL_MDIO_phyInitiateRegWriteC45(CSL_mdioHandle hMdioRegs,
                                        uint32_t userGroup,
                                        uint32_t phyAddr,
                                        uint32_t mmdNum,
                                        uint32_t regAddr,
                                        uint16_t wrVal)
{
    uint32_t regVal = 0U;
    int32_t retVal = CSL_PASS;

    /* Initiate register write if no other transaction is still going on */
    if (CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_EN_0x0 ==
        CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG, MDIO_USER_GROUP_USER_ACCESS_REG_GO))
    {
        if (userGroup == 0U)
        {
            CSL_FINS(hMdioRegs->USER_ADDR0_REG, MDIO_USER_ADDR0_REG_USER_ADDR0, regAddr);
        }
        else
        {
            CSL_FINS(hMdioRegs->USER_ADDR1_REG, MDIO_USER_ADDR1_REG_USER_ADDR1, regAddr);
        }

        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_GO, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_EN_0x1);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_WRITE, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_WRITE);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_PHYADR, phyAddr);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_REGADR, mmdNum);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_DATA, wrVal);
        CSL_REG_WR(&hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG, regVal);
    }
    else
    {
        retVal = CSL_EFAIL;
    }

    return retVal;
}

int32_t CSL_MDIO_phyInitiateRegReadC45(CSL_mdioHandle hMdioRegs,
                                       uint32_t userGroup,
                                       uint32_t phyAddr,
                                       uint32_t mmdNum,
                                       uint32_t regAddr)
{
    uint32_t regVal = 0U;
    int32_t retVal = CSL_PASS;

    /* Initiate register read if no other transaction is still going on */
    if (CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_EN_0x0 ==
        CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG, MDIO_USER_GROUP_USER_ACCESS_REG_GO))
    {
        if (userGroup == 0U)
        {
            CSL_FINS(hMdioRegs->USER_ADDR0_REG, MDIO_USER_ADDR0_REG_USER_ADDR0, regAddr);
        }
        else
        {
            CSL_FINS(hMdioRegs->USER_ADDR1_REG, MDIO_USER_ADDR1_REG_USER_ADDR1, regAddr);
        }

        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_GO, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_EN_0x1);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_WRITE, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_READ);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_PHYADR, phyAddr);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_REGADR, mmdNum);
        CSL_REG_WR(&hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG, regVal);
    }
    else
    {
        retVal = CSL_EFAIL;
    }

    return retVal;
}

int32_t CSL_MDIO_phyRegInitiateWriteC22(CSL_mdioHandle hMdioRegs,
                                        uint32_t userGroup,
                                        uint32_t phyAddr,
                                        uint32_t regAddr,
                                        uint16_t wrVal)
{
    uint32_t regVal = 0U;
    int32_t retVal = CSL_PASS;

    /* Initiate register write if no other transaction is still going on */
    if (CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_EN_0x0 ==
        CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG, MDIO_USER_GROUP_USER_ACCESS_REG_GO))
    {
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_GO, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_EN_0x1);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_WRITE, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_WRITE);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_PHYADR, phyAddr);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_REGADR, regAddr);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_DATA, wrVal);
        CSL_REG_WR(&hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG, regVal);
    }
    else
    {
        retVal = CSL_EFAIL;
    }

    return retVal;
}

int32_t CSL_MDIO_phyInitiateRegReadC22(CSL_mdioHandle hMdioRegs,
                                       uint32_t userGroup,
                                       uint32_t phyAddr,
                                       uint32_t regAddr)
{
    uint32_t regVal = 0U;
    int32_t retVal = CSL_PASS;

    /* Initiate register read if no other transaction is still going on */
    if (CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_EN_0x0 ==
        CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG, MDIO_USER_GROUP_USER_ACCESS_REG_GO))
    {
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_GO, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_EN_0x1);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_WRITE, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_READ);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_PHYADR, phyAddr);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_REGADR, regAddr);
        CSL_REG_WR(&hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG, regVal);
    }
    else
    {
        retVal = CSL_EFAIL;
    }

    return retVal;
}

int32_t CSL_MDIO_phyGetRegReadVal(CSL_mdioHandle hMdioRegs,
                                  uint32_t userGroup,
                                  uint16_t *pData)
{
    int32_t retVal = CSL_PASS;

    /* Get read value if previous transaction is complete */
    if (CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_EN_0x0 ==
        CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG, MDIO_USER_GROUP_USER_ACCESS_REG_GO))
    {
        /* Store the data if the read is acknowledged */
        if (CSL_MDIO_USER_GROUP_USER_ACCESS_REG_ACK_PASS ==
            CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG, MDIO_USER_GROUP_USER_ACCESS_REG_ACK))
        {
            *pData = (uint16_t)(CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG,
                                         MDIO_USER_GROUP_USER_ACCESS_REG_DATA));
            retVal = CSL_PASS;
        }
        else
        {
            retVal = CSL_ETIMEOUT;
        }
    }
    else
    {
        retVal = CSL_EFAIL;
    }

    return retVal;
}

uint32_t CSL_MDIO_isPhyRegAccessComplete(CSL_mdioHandle hMdioRegs,
                                         uint32_t userGroup)
{
    uint32_t isComplete = (uint32_t)TRUE;

    if (CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_EN_0x1 ==
        CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG, MDIO_USER_GROUP_USER_ACCESS_REG_GO))
    {
        isComplete = (uint32_t)FALSE;
    }

    return isComplete;
}

uint32_t CSL_MDIO_phyLinkStatus2(CSL_mdioHandle hMdioRegs, uint32_t phyAddr)
{
    uint32_t retVal = FALSE;

    if(0U != ((CSL_FEXT(hMdioRegs->LINK_REG,MDIO_LINK_REG_LINK)) & ((1U) << phyAddr)))
    {
        retVal =  (uint32_t)TRUE;
    }

    return retVal;
}

void CSL_MDIO_enableLinkStatusChangeInterrupt (
    CSL_mdioHandle          hMdioRegs,
    Uint32                  index,
    Uint32                  phyAddr
)
{
    hMdioRegs->USER_GROUP [index].USER_PHY_SEL_REG  =   CSL_FMK (MDIO_USER_GROUP_USER_PHY_SEL_REG_PHYADR_MON, phyAddr) |
                                                        CSL_FMK (MDIO_USER_GROUP_USER_PHY_SEL_REG_LINKINT_ENABLE, 1U);
    return;
}

void CSL_MDIO_disableLinkStatusChangeInterrupt (
    CSL_mdioHandle          hMdioRegs,
    Uint32                  index,
    Uint32                  phyAddr
)
{
    hMdioRegs->USER_GROUP [index].USER_PHY_SEL_REG  =   CSL_FMK (MDIO_USER_GROUP_USER_PHY_SEL_REG_PHYADR_MON, phyAddr) |
                                                        CSL_FMK (MDIO_USER_GROUP_USER_PHY_SEL_REG_LINKINT_ENABLE, 0U);

    return;
}

Uint32 CSL_MDIO_getLinkStatusChangePhyAddr (
    CSL_mdioHandle          hMdioRegs,
    Uint32                  index
)
{
    return CSL_FEXT(hMdioRegs->USER_GROUP[index].USER_PHY_SEL_REG,
                    MDIO_USER_GROUP_USER_PHY_SEL_REG_PHYADR_MON);
}

void CSL_MDIO_enableFaultDetect (
    CSL_mdioHandle          hMdioRegs
)
{
    CSL_FINS (hMdioRegs->CONTROL_REG, MDIO_CONTROL_REG_FAULT_DETECT_ENABLE, 1U);

    return;
}

void CSL_MDIO_disableFaultDetect (
    CSL_mdioHandle          hMdioRegs
)
{
    CSL_FINS (hMdioRegs->CONTROL_REG, MDIO_CONTROL_REG_FAULT_DETECT_ENABLE, 0U);

    return;
}

void CSL_MDIO_enablePreamble (
    CSL_mdioHandle          hMdioRegs
)
{
    CSL_FINS (hMdioRegs->CONTROL_REG, MDIO_CONTROL_REG_PREAMBLE, 0U);

    return;
}

void CSL_MDIO_disablePreamble (CSL_mdioHandle hMdioRegs)
{
    CSL_FINS (hMdioRegs->CONTROL_REG, MDIO_CONTROL_REG_PREAMBLE, 1U);

    return;
}

void CSL_MDIO_enableStateChangeMode(CSL_mdioHandle hMdioRegs)
{
    CSL_FINS(hMdioRegs->POLL_REG, MDIO_POLL_REG_STATECHANGEMODE, 1U);

    return;
}


void CSL_MDIO_enableManualMode(CSL_mdioHandle hMdioRegs)
{
    CSL_FINS(hMdioRegs->POLL_REG, MDIO_POLL_REG_STATECHANGEMODE, 1U);
    CSL_FINS(hMdioRegs->POLL_REG, MDIO_POLL_REG_MANUALMODE, 1U);
    return;
}

void CSL_MDIO_disableManualMode(CSL_mdioHandle hMdioRegs)
{
    CSL_FINS(hMdioRegs->POLL_REG, MDIO_POLL_REG_MANUALMODE, 0U);
    return;
}

void CSL_MDIO_disableStateChangeMode(CSL_mdioHandle hMdioRegs)
{
    CSL_FINS(hMdioRegs->POLL_REG, MDIO_POLL_REG_STATECHANGEMODE, 0U);

    return;
}

uint32_t CSL_MDIO_isStateChangeModeEnabled(CSL_mdioHandle hMdioRegs)
{
    return CSL_FEXT(hMdioRegs->POLL_REG, MDIO_POLL_REG_STATECHANGEMODE);
}

void CSL_MDIO_setPollIPG(CSL_mdioHandle hMdioRegs,
                         uint8_t ipgVal)
{
    CSL_FINS(hMdioRegs->POLL_REG, MDIO_POLL_REG_IPG, ipgVal);
}

uint8_t CSL_MDIO_getPollIPG(CSL_mdioHandle hMdioRegs)
{
    return CSL_FEXT(hMdioRegs->POLL_REG, MDIO_POLL_REG_IPG);
}

void CSL_MDIO_setPollEnableMask(CSL_mdioHandle hMdioRegs,
                                uint32_t pollEnableMask)
{
    /* Due to a hardware limitation, bit 31 must always be set */
    hMdioRegs->POLL_EN_REG = pollEnableMask | (1U << 31U);
}

void CSL_MDIO_clearPollEnableMask(CSL_mdioHandle hMdioRegs)
{
    /* Due to a hardware limitation, bit 31 must always be set */
    hMdioRegs->POLL_EN_REG = (1U << 31U);
}

uint32_t CSL_MDIO_getPollEnableMask(CSL_mdioHandle hMdioRegs)
{
    return hMdioRegs->POLL_EN_REG;
}

void CSL_MDIO_setMdclkHigh(CSL_mdioHandle hMdioRegs)
{
    CSL_FINS(hMdioRegs->MANUAL_IF_REG, MDIO_MANUAL_IF_REG_MDIO_MDCLK_O, 1U);
}

void CSL_MDIO_setMdclkLow(CSL_mdioHandle hMdioRegs)
{
    CSL_FINS(hMdioRegs->MANUAL_IF_REG, MDIO_MANUAL_IF_REG_MDIO_MDCLK_O, 0U);
}

void CSL_MDIO_setMdoHigh(CSL_mdioHandle hMdioRegs)
{
    CSL_FINS(hMdioRegs->MANUAL_IF_REG, MDIO_MANUAL_IF_REG_MDIO_PIN, 1U);
}

void CSL_MDIO_setMdoLow(CSL_mdioHandle hMdioRegs)
{
    CSL_FINS(hMdioRegs->MANUAL_IF_REG, MDIO_MANUAL_IF_REG_MDIO_PIN, 0U);
}

uint32_t CSL_MDIO_readMdi(CSL_mdioHandle hMdioRegs)
{
    return CSL_FEXT(hMdioRegs->MANUAL_IF_REG, MDIO_MANUAL_IF_REG_MDIO_PIN);
}

void CSL_MDIO_setMdoOutputEnable(CSL_mdioHandle hMdioRegs)
{
    CSL_FINS(hMdioRegs->MANUAL_IF_REG, MDIO_MANUAL_IF_REG_MDIO_OE, 1U);
}

void CSL_MDIO_setMdoInputEnable(CSL_mdioHandle hMdioRegs)
{
    CSL_FINS(hMdioRegs->MANUAL_IF_REG, MDIO_MANUAL_IF_REG_MDIO_OE, 0U);
}

uint32_t CSL_MDIO_phyLinkStatus(uint32_t baseAddr, uint32_t phyAddr)
{
    return (CSL_MDIO_phyLinkStatus2((CSL_mdioHandle)((uintptr_t)baseAddr),phyAddr));
}

uint32_t  CSL_MDIO_phyRegRead(uint32_t baseAddr,
                    uint32_t phyAddr,
                    uint32_t regNum,
                    uint16_t *pData)
{
    return CSL_MDIO_phyRegRead2((CSL_mdioHandle)((uintptr_t)baseAddr),
                                 0U /* Hardcode user group to 0 */,
                                 phyAddr,
                                 regNum,
                                 pData);
}

void CSL_MDIO_phyRegWrite(uint32_t baseAddr, uint32_t phyAddr, uint32_t regNum, uint16_t wrVal)
{
    CSL_MDIO_phyRegWrite2((CSL_mdioHandle)((uintptr_t)baseAddr),
                            0U /* Hardcode user group to 0 */,
                            phyAddr,
                            regNum,
                            wrVal);

}

