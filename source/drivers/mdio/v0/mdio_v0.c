/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/mdio.h>
#include <drivers/hw_include/hw_types.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define MDIO_CLK_DIV_CFG            (0xFFU)

#define PHY_REGCR_REG               (0x0D)
#define PHY_ADDR_REG                (0x0E)

#define EXT_REG_ADDRESS_ACCESS      (0x001F)
#define EXT_REG_DATA_NORMAL_ACCESS  (0x401F)

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

typedef volatile CSL_MdioRegs *MDIO_Handle;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t MDIO_initClock(uint32_t baseAddr)
{
    int32_t retVal = SystemP_SUCCESS;

    HW_WR_REG32((baseAddr + CSL_MDIO_CONTROL_REG),
                (CSL_FMKT(MDIO_CONTROL_REG_ENABLE, YES) |
                CSL_FMK(MDIO_CONTROL_REG_CLKDIV, MDIO_CLK_DIV_CFG)));

    return retVal;
}

int32_t MDIO_phyRegRead(uint32_t baseAddr,
                        void     *pUserGroup,
                        uint32_t phyAddr,
                        uint32_t regNum,
                        uint16_t *pData)
{
    uint32_t regVal = 0U;
    int32_t retVal = SystemP_SUCCESS;
    uint32_t userGroup;
    MDIO_Handle hMdioRegs = (MDIO_Handle)((uintptr_t)baseAddr);

    if(pUserGroup == NULL)
    {
        userGroup = 0;
    }
    else
    {
        userGroup = *((uint32_t *)pUserGroup);
        if(userGroup > 1)
        {
            retVal = SystemP_FAILURE;
        }
    }

    if(retVal == SystemP_SUCCESS)
    {
        /* Wait till transaction completion if any */
        while(CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_EN_0x1 ==
            CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG, MDIO_USER_GROUP_USER_ACCESS_REG_GO))
        {}
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_GO, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_EN_0x1);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_WRITE, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_READ);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_PHYADR, phyAddr);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_REGADR, regNum);
        CSL_REG_WR(&hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG, regVal);

        /* wait for command completion */
        while(CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_EN_0x1 ==
            CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG, MDIO_USER_GROUP_USER_ACCESS_REG_GO))
        {}

        /* Store the data if the read is acknowledged */
        if(CSL_MDIO_USER_GROUP_USER_ACCESS_REG_ACK_PASS ==
        CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG, MDIO_USER_GROUP_USER_ACCESS_REG_ACK))
        {
            *pData = (uint16_t)(CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG,
                MDIO_USER_GROUP_USER_ACCESS_REG_DATA));
            retVal = SystemP_SUCCESS;
        }
        else
        {
            retVal = SystemP_FAILURE;
        }
    }

    return retVal;
}

int32_t MDIO_phyRegWrite(uint32_t baseAddr,
                         void     *pUserGroup,
                         uint32_t phyAddr,
                         uint32_t regNum,
                         uint16_t wrVal)
{
    uint32_t regVal = 0U;
    int32_t retVal = SystemP_SUCCESS;
    uint32_t userGroup;
    MDIO_Handle hMdioRegs = (MDIO_Handle)((uintptr_t)baseAddr);

    if(pUserGroup == NULL)
    {
        userGroup = 0;
    }
    else
    {
        userGroup = *((uint32_t *)pUserGroup);
        if(userGroup > 1)
        {
            retVal = SystemP_FAILURE;
        }
    }

    if(retVal == SystemP_SUCCESS)
    {
        /* Wait till transaction completion if any */
        while(CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_EN_0x1 ==
            CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG, MDIO_USER_GROUP_USER_ACCESS_REG_GO))
        {}

        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_GO, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_EN_0x1);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_WRITE, CSL_MDIO_USER_GROUP_USER_ACCESS_REG_WRITE);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_PHYADR, phyAddr);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_REGADR, regNum);
        CSL_FINS(regVal, MDIO_USER_GROUP_USER_ACCESS_REG_DATA, wrVal);
        CSL_REG_WR(&hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG, regVal);

        /* wait for command completion */
        while(CSL_MDIO_USER_GROUP_USER_ACCESS_REG_GO_EN_0x1 ==
            CSL_FEXT(hMdioRegs->USER_GROUP[userGroup].USER_ACCESS_REG, MDIO_USER_GROUP_USER_ACCESS_REG_GO))
        {}
    }

    return retVal;
}

int32_t MDIO_phyExtRegRead(uint32_t baseAddr,
                           void     *pUserGroup,
                           uint32_t phyAddr,
                           uint32_t regNum,
                           uint16_t *pData)
{
    int32_t retVal;

    retVal = MDIO_phyRegWrite(baseAddr, pUserGroup, phyAddr, PHY_REGCR_REG, EXT_REG_ADDRESS_ACCESS);

    if(retVal == SystemP_SUCCESS)
    {
        retVal = MDIO_phyRegWrite(baseAddr, pUserGroup, phyAddr, PHY_ADDR_REG, regNum);
    }

    if(retVal == SystemP_SUCCESS)
    {
        retVal = MDIO_phyRegWrite(baseAddr, pUserGroup, phyAddr, PHY_REGCR_REG, EXT_REG_DATA_NORMAL_ACCESS);
    }

    if(retVal == SystemP_SUCCESS)
    {
        retVal = MDIO_phyRegRead(baseAddr, pUserGroup, phyAddr, PHY_ADDR_REG, pData);
    }
    return retVal;
}

int32_t MDIO_phyExtRegWrite(uint32_t baseAddr,
                            void     *pUserGroup,
                            uint32_t phyAddr,
                            uint32_t regNum,
                            uint16_t wrVal)
{
    uint32_t retVal;

    retVal = MDIO_phyRegWrite(baseAddr, pUserGroup, phyAddr, PHY_REGCR_REG, EXT_REG_ADDRESS_ACCESS);

    if(retVal == SystemP_SUCCESS)
    {
        retVal = MDIO_phyRegWrite(baseAddr, pUserGroup, phyAddr, PHY_ADDR_REG, regNum);
    }

    if(retVal == SystemP_SUCCESS)
    {
        retVal = MDIO_phyRegWrite(baseAddr, pUserGroup, phyAddr, PHY_REGCR_REG, EXT_REG_DATA_NORMAL_ACCESS);
    }

    if(retVal == SystemP_SUCCESS)
    {
        retVal = MDIO_phyRegWrite(baseAddr, pUserGroup, phyAddr, PHY_ADDR_REG, wrVal);
    }      
    return retVal;
}

int32_t MDIO_phyLinkStatus(uint32_t baseAddr, uint32_t phyAddr)
{
    uint32_t retVal = SystemP_FAILURE;
    MDIO_Handle hMdioRegs = (MDIO_Handle)((uintptr_t)baseAddr);

    if(0U != ((CSL_FEXT(hMdioRegs->LINK_REG,MDIO_LINK_REG_LINK)) & ((1U) << phyAddr)))
    {
        retVal = SystemP_SUCCESS;
    }
    return retVal;
}

int32_t MDIO_enableLinkInterrupt(uint32_t mdioBaseAddress,
                                 uint32_t regInst,
                                 uint32_t phyAddr,
                                 uint8_t linkSel)
{
    uint32_t retVal = SystemP_FAILURE;
    uint32_t regVal;

    regVal = phyAddr;
    regVal |=  0x40;

    if(MDIO_LINKSEL_MLINK_MODE == linkSel)
    {
        regVal |= 0x80;
    }

    HW_WR_REG32((mdioBaseAddress + CSL_MDIO_USER_PHY_SEL_REG(regInst)), regVal);

    return retVal;
}
