/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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

/**
 *  \file rti.c
 *
 *  \brief File containing RTI Driver APIs implementation.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/rti.h>
#include <stdint.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/csl_types.h>

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t RTIG_setStallMode(uint32_t baseAddr, uint32_t stallMode)
{
    int32_t retVal = SystemP_FAILURE;
    if (baseAddr  != ((uint32_t) NULL))
    {
        retVal = SystemP_SUCCESS;
        if (stallMode == RTI_GC_STALL_MODE_OFF)
        {
            HW_WR_FIELD32(baseAddr + CSL_RTI_RTIGCTRL,
                          CSL_RTI_RTIGCTRL_COS,
                          RTI_GC_STALL_MODE_OFF);
        }
        else
        {
            HW_WR_FIELD32(baseAddr + CSL_RTI_RTIGCTRL,
                          CSL_RTI_RTIGCTRL_COS,
                          RTI_GC_STALL_MODE_ON);
        }
    }
    return (retVal);
}

int32_t RTI_counterEnable(uint32_t baseAddr, uint32_t cntIndex)
{
    int32_t retVal = SystemP_FAILURE;

    if ((baseAddr != ((uint32_t) NULL)) &&
        (cntIndex <= RTI_TMR_CNT_BLK_INDEX_MAX))
    {
        /* Start the timer */
        if (cntIndex == RTI_TMR_CNT_BLK_INDEX_0)
        {
            HW_WR_FIELD32(baseAddr + CSL_RTI_RTIGCTRL, CSL_RTI_RTIGCTRL_CNT0EN, 1);
        }
        else
        {
            HW_WR_FIELD32(baseAddr + CSL_RTI_RTIGCTRL, CSL_RTI_RTIGCTRL_CNT1EN, 1);
        }
        retVal = SystemP_SUCCESS;
    }

    return (retVal);
}

int32_t RTI_counterDisable(uint32_t baseAddr, uint32_t cntIndex)
{
    int32_t retVal = SystemP_FAILURE;

    if ((baseAddr != ((uint32_t) NULL)) &&
        (cntIndex <= RTI_TMR_CNT_BLK_INDEX_MAX))
    {
        /* Start the timer */
        if (cntIndex == RTI_TMR_CNT_BLK_INDEX_0)
        {
            HW_WR_FIELD32(baseAddr + CSL_RTI_RTIGCTRL, CSL_RTI_RTIGCTRL_CNT0EN, 0);
        }
        else
        {
            HW_WR_FIELD32(baseAddr + CSL_RTI_RTIGCTRL, CSL_RTI_RTIGCTRL_CNT1EN, 0);
        }
        retVal = SystemP_SUCCESS;
    }

    return (retVal);
}

int32_t RTI_counterConfigure(uint32_t baseAddr, uint32_t cntBlkIndex, uint32_t clkSrc, uint32_t ntu, uint32_t prescaler)
{
    int32_t  retVal = SystemP_FAILURE;

    if ((baseAddr != ((uint32_t) NULL)) &&
        (cntBlkIndex <= RTI_TMR_CNT_BLK_INDEX_MAX) &&
        (prescaler >= RTI_TMR_MIN_PRESCALER_VAL))
    {
        if (cntBlkIndex == RTI_TMR_CNT_BLK_INDEX_0)
        {
            /* Disable the counter block */
            HW_WR_FIELD32(baseAddr + CSL_RTI_RTIGCTRL, CSL_RTI_RTIGCTRL_CNT0EN, 0);

            /* Clear free-running counter */
            HW_WR_REG32(baseAddr + CSL_RTI_RTIFRC0, 0);

            /* Configure FRC clock source for counter block 0 only */
            if (clkSrc == RTI_TMR_CLK_SRC_NTU)
            {
                uint32_t ntuSel = ntu;
                HW_WR_FIELD32(baseAddr + CSL_RTI_RTITBCTRL, CSL_RTI_RTITBCTRL_TBEXT, 1);
                if ((ntuSel != RTI_TMR_NTU_1) &&
                    (ntuSel != RTI_TMR_NTU_2) &&
                    (ntuSel != RTI_TMR_NTU_3))
                {
                    ntuSel = RTI_TMR_NTU_0;
                }
                HW_WR_FIELD32(baseAddr + CSL_RTI_RTIGCTRL, CSL_RTI_RTIGCTRL_NTUSEL, ntuSel);
            }
            else
            {
                HW_WR_FIELD32(baseAddr + CSL_RTI_RTITBCTRL, CSL_RTI_RTITBCTRL_TBEXT, 0);
                HW_WR_REG32(baseAddr + CSL_RTI_RTIUC0, 0);

                /* Configure the upper counter comapre register */
                HW_WR_REG32(baseAddr + CSL_RTI_RTICPUC0, prescaler);
            }
        }
        else /* Couner Block 1 */
        {
            /* Disable the counter block */
            HW_WR_FIELD32(baseAddr + CSL_RTI_RTIGCTRL, CSL_RTI_RTIGCTRL_CNT1EN, 0);

            /* Clear both counters */
            HW_WR_REG32(baseAddr + CSL_RTI_RTIUC1, 0);
            HW_WR_REG32(baseAddr + CSL_RTI_RTIFRC1, 0);

            /* Configure the upper counter comapre register */
            HW_WR_REG32(baseAddr + CSL_RTI_RTICPUC1, prescaler);
        }
        retVal = SystemP_SUCCESS;
    }
    return (retVal);
}

int32_t RTI_counterClear(uint32_t baseAddr, uint32_t cntIndex)
{
    int32_t  retVal = SystemP_FAILURE;

    if ((baseAddr != ((uint32_t) NULL)) &&
        (cntIndex <= RTI_TMR_CNT_BLK_INDEX_MAX))
    {
        if (cntIndex == RTI_TMR_CNT_BLK_INDEX_0)
        {
            /* Disable the counter block */
            HW_WR_FIELD32(baseAddr + CSL_RTI_RTIGCTRL, CSL_RTI_RTIGCTRL_CNT0EN, 0);

            /* Clear all counter related registers */
            HW_WR_REG32(baseAddr + CSL_RTI_RTIUC0, 0);
            HW_WR_REG32(baseAddr + CSL_RTI_RTIFRC0, 0);
            HW_WR_REG32(baseAddr + CSL_RTI_RTICPUC0, 0);

            /* Select up counter as the source of FRC */
            HW_WR_FIELD32(baseAddr + CSL_RTI_RTITBCTRL, CSL_RTI_RTITBCTRL_TBEXT, 0);
        }
        else /* Couner Block 1 */
        {
            /* Disable the counter block */
            HW_WR_FIELD32(baseAddr + CSL_RTI_RTIGCTRL, CSL_RTI_RTIGCTRL_CNT1EN, 0);

            /* Clear all counter related registers */
            HW_WR_REG32(baseAddr + CSL_RTI_RTIUC1, 0);
            HW_WR_REG32(baseAddr + CSL_RTI_RTIFRC1, 0);
            HW_WR_REG32(baseAddr + CSL_RTI_RTICPUC1, 0);
        }
        retVal = SystemP_SUCCESS;
    }
    return (retVal);
}

int32_t RTI_compareClear(uint32_t baseAddr, uint32_t cmpIndex)
{
    int32_t  retVal = SystemP_FAILURE;

    if ((baseAddr != ((uint32_t) NULL)) &&
        (cmpIndex <= RTI_TMR_CMP_BLK_INDEX_MAX))
    {
        /* Clear the compare blocks */
        switch (cmpIndex)
        {
            case RTI_TMR_CMP_BLK_INDEX_0:
                HW_WR_REG32(baseAddr + CSL_RTI_RTICOMP0, 0);
                HW_WR_REG32(baseAddr + CSL_RTI_RTIUDCP0, 0);
                HW_WR_FIELD32(baseAddr + CSL_RTI_RTICOMPCTRL, CSL_RTI_RTICOMPCTRL_COMP0SEL, 0);
                break;

            case RTI_TMR_CMP_BLK_INDEX_1:
                HW_WR_REG32(baseAddr + CSL_RTI_RTICOMP1, 0);
                HW_WR_REG32(baseAddr + CSL_RTI_RTIUDCP1, 0);
                HW_WR_FIELD32(baseAddr + CSL_RTI_RTICOMPCTRL, CSL_RTI_RTICOMPCTRL_COMP1SEL, 0);
                break;

            case RTI_TMR_CMP_BLK_INDEX_2:
                HW_WR_REG32(baseAddr + CSL_RTI_RTICOMP2, 0);
                HW_WR_REG32(baseAddr + CSL_RTI_RTIUDCP2, 0);
                HW_WR_FIELD32(baseAddr + CSL_RTI_RTICOMPCTRL, CSL_RTI_RTICOMPCTRL_COMP2SEL, 0);
                break;

            case RTI_TMR_CMP_BLK_INDEX_3:
                HW_WR_REG32(baseAddr + CSL_RTI_RTICOMP3, 0);
                HW_WR_REG32(baseAddr + CSL_RTI_RTIUDCP3, 0);
                HW_WR_FIELD32(baseAddr + CSL_RTI_RTICOMPCTRL, CSL_RTI_RTICOMPCTRL_COMP3SEL, 0);
                break;

            default:
                break;
        }
        retVal = SystemP_SUCCESS;
    }
    return (retVal);
}

int32_t RTI_counterGet(uint32_t baseAddr, uint32_t cntIndex, uint32_t *counterLow, uint32_t *counterHigh)
{
    int32_t retVal = SystemP_FAILURE;

    if ((baseAddr != ((uint32_t) NULL)) &&
        (cntIndex <= RTI_TMR_CNT_BLK_INDEX_MAX))
    {
        /* Start the timer */
        if (cntIndex == RTI_TMR_CNT_BLK_INDEX_0)
        {
            *counterHigh = HW_RD_REG32(baseAddr + CSL_RTI_RTIFRC0);
            *counterLow =  HW_RD_REG32(baseAddr + CSL_RTI_RTIUC0);
        }
        else
        {
            *counterHigh = HW_RD_REG32(baseAddr + CSL_RTI_RTIFRC1);
            *counterLow =  HW_RD_REG32(baseAddr + CSL_RTI_RTIUC1);
        }
        retVal = SystemP_SUCCESS;
    }
    return (retVal);
}

int32_t RTI_captureConfig(uint32_t baseAddr, uint32_t cntIndex, uint32_t cntrCapSrc)
{
    int32_t retVal = SystemP_FAILURE;

    if ((baseAddr != ((uint32_t) NULL)) &&
        (cntIndex <= RTI_TMR_CNT_BLK_INDEX_MAX))
    {
        if (cntIndex == RTI_TMR_CNT_BLK_INDEX_0)
        {
            /* Configure Capture source events */
            HW_WR_FIELD32(baseAddr + CSL_RTI_RTICAPCTRL, CSL_RTI_RTICAPCTRL_CAPCNTR0, cntrCapSrc);
        }
        else
        {
            HW_WR_FIELD32(baseAddr + CSL_RTI_RTICAPCTRL, CSL_RTI_RTICAPCTRL_CAPCNTR1, cntrCapSrc);
        }
        retVal = SystemP_SUCCESS;
    }
    return (retVal);
}

int32_t RTI_captureCounterGet(uint32_t baseAddr, uint32_t cntIndex, uint32_t *counterLow, uint32_t *counterHigh)
{
    int32_t retVal = SystemP_FAILURE;

    if ((baseAddr != ((uint32_t) NULL)) &&
        (cntIndex <= RTI_TMR_CNT_BLK_INDEX_MAX))
    {
        /* Start the timer */
        if (cntIndex == RTI_TMR_CNT_BLK_INDEX_0)
        {
            *counterLow =  HW_RD_REG32(baseAddr + CSL_RTI_RTICAUC0);
            *counterHigh = HW_RD_REG32(baseAddr + CSL_RTI_RTICAFRC0);
        }
        else
        {
            *counterLow =  HW_RD_REG32(baseAddr + CSL_RTI_RTICAUC1);
            *counterHigh = HW_RD_REG32(baseAddr + CSL_RTI_RTICAFRC1);
        }
        retVal = SystemP_SUCCESS;
    }
    return (retVal);
}

int32_t RTI_compareEventConfig(uint32_t baseAddr, uint32_t cmpIndex, uint32_t cntBlkIndex, uint32_t cmpVal, uint32_t period)
{
    int32_t retVal = SystemP_FAILURE;

    if ((baseAddr != ((uint32_t) NULL)) &&
        (cmpIndex <= RTI_TMR_CMP_BLK_INDEX_MAX) &&
        (cntBlkIndex <= RTI_TMR_CNT_BLK_INDEX_MAX))
    {
        /* Configure the compare blocks */
        switch (cmpIndex)
        {
            case RTI_TMR_CMP_BLK_INDEX_0:
                HW_WR_REG32(baseAddr + CSL_RTI_RTICOMP0, cmpVal);
                HW_WR_REG32(baseAddr + CSL_RTI_RTIUDCP0, period);
                HW_WR_FIELD32(baseAddr + CSL_RTI_RTICOMPCTRL, CSL_RTI_RTICOMPCTRL_COMP0SEL, cntBlkIndex);
                break;

            case RTI_TMR_CMP_BLK_INDEX_1:
                HW_WR_REG32(baseAddr + CSL_RTI_RTICOMP1, cmpVal);
                HW_WR_REG32(baseAddr + CSL_RTI_RTIUDCP1, period);
                HW_WR_FIELD32(baseAddr + CSL_RTI_RTICOMPCTRL, CSL_RTI_RTICOMPCTRL_COMP1SEL, cntBlkIndex);
                break;

            case RTI_TMR_CMP_BLK_INDEX_2:
                HW_WR_REG32(baseAddr + CSL_RTI_RTICOMP2, cmpVal);
                HW_WR_REG32(baseAddr + CSL_RTI_RTIUDCP2, period);
                HW_WR_FIELD32(baseAddr + CSL_RTI_RTICOMPCTRL, CSL_RTI_RTICOMPCTRL_COMP2SEL, cntBlkIndex);
                break;

            case RTI_TMR_CMP_BLK_INDEX_3:
                HW_WR_REG32(baseAddr + CSL_RTI_RTICOMP3, cmpVal);
                HW_WR_REG32(baseAddr + CSL_RTI_RTIUDCP3, period);
                HW_WR_FIELD32(baseAddr + CSL_RTI_RTICOMPCTRL, CSL_RTI_RTICOMPCTRL_COMP3SEL, cntBlkIndex);
                break;

            default:
                break;
        }
        retVal = SystemP_SUCCESS;
    }
    return (retVal);
}

uint32_t RTI_compareGet(uint32_t baseAddr, uint32_t cmpIndex)
{
    uint32_t regVal = 0U;
    uint32_t regOffset = 0;

    if ((baseAddr != ((uint32_t) NULL)) &&
        (cmpIndex <= RTI_TMR_CMP_BLK_INDEX_MAX))
    {
        switch (cmpIndex)
        {
            case RTI_TMR_CMP_BLK_INDEX_0:
                regOffset = CSL_RTI_RTICOMP0;
                break;

            case RTI_TMR_CMP_BLK_INDEX_1:
                regOffset = CSL_RTI_RTICOMP1;
                break;

            case RTI_TMR_CMP_BLK_INDEX_2:
                regOffset = CSL_RTI_RTICOMP2;
                break;

            case RTI_TMR_CMP_BLK_INDEX_3:
                regOffset = CSL_RTI_RTICOMP3;
                break;

            default:
                break;
        }
        regVal = HW_RD_REG32(baseAddr + regOffset);
    }
    return (regVal);
}

int32_t RTI_compareClearConfig(uint32_t baseAddr, uint32_t cmpIndex, uint32_t cmpClearVal)
{
    int32_t retVal = SystemP_FAILURE;

    if ((baseAddr != ((uint32_t) NULL)) &&
        (cmpIndex <= RTI_TMR_CMP_BLK_INDEX_MAX))
    {
        /* Configure the compare blocks */
        switch (cmpIndex)
        {
            case RTI_TMR_CMP_BLK_INDEX_0:
                HW_WR_REG32(baseAddr + CSL_RTI_RTICOMP0CLR, cmpClearVal);
                break;

            case RTI_TMR_CMP_BLK_INDEX_1:
                HW_WR_REG32(baseAddr + CSL_RTI_RTICOMP1CLR, cmpClearVal);
                break;

            case RTI_TMR_CMP_BLK_INDEX_2:
                HW_WR_REG32(baseAddr + CSL_RTI_RTICOMP2CLR, cmpClearVal);
                break;

            case RTI_TMR_CMP_BLK_INDEX_3:
                HW_WR_REG32(baseAddr + CSL_RTI_RTICOMP3CLR, cmpClearVal);
                break;

            default:
                break;
        }
        retVal = SystemP_SUCCESS;
    }
    return (retVal);
}

uint32_t RTI_intStatusGet(uint32_t baseAddr)
{
    uint32_t intFlags = 0U;

    if (baseAddr != (uint32_t)(NULL))
    {
        intFlags = HW_RD_REG32(baseAddr + CSL_RTI_RTIINTFLAG);
    }

    return (intFlags);
}

int32_t RTI_intStatusClear(uint32_t baseAddr, uint32_t intFlags)
{
    int32_t retVal = SystemP_FAILURE;
    uint32_t flags = intFlags & RTI_TMR_INT_STATUS_ALL;

    if ((baseAddr != ((uint32_t) NULL)) &&
        (flags    != (uint32_t)0))
    {
        /* Clear the interrupt status from IRQSTATUS register */
        HW_WR_REG32(baseAddr + CSL_RTI_RTIINTFLAG, flags);
        retVal = SystemP_SUCCESS;
    }
    return (retVal);
}

int32_t RTI_intEnable(uint32_t baseAddr, uint32_t intFlags)
{
    int32_t retVal = SystemP_FAILURE;
    uint32_t flags = intFlags & RTI_TMR_INT_REQ_ALL;

    if ((baseAddr != ((uint32_t) NULL)) &&
        (flags    != (uint32_t)0))
    {
        /* Enable the Timer interrupts represented by intFlags */
        HW_WR_REG32(baseAddr + CSL_RTI_RTISETINT, flags);
        retVal = SystemP_SUCCESS;
    }
    return(retVal);
}

int32_t RTI_intDisable(uint32_t baseAddr, uint32_t intFlags)
{
    int32_t retVal = SystemP_FAILURE;
    uint32_t flags = intFlags & RTI_TMR_INT_REQ_ALL;

    if ((baseAddr != ((uint32_t) NULL)) &&
        (flags    != (uint32_t)0))
    {
        /* Enable the Timer interrupts represented by intFlags */
        HW_WR_REG32(baseAddr + CSL_RTI_RTICLEARINT, flags);
        retVal = SystemP_SUCCESS;
    }
    return(retVal);
}

int32_t RTI_intAutoClearEnable(uint32_t baseAddr, uint32_t cmpIndex)
{
    int32_t retVal = SystemP_FAILURE;

    if ((baseAddr != ((uint32_t) NULL)) &&
        (cmpIndex <= RTI_TMR_CMP_BLK_INDEX_MAX))
    {
        /* Configure the compare blocks */
        switch (cmpIndex)
        {
            case RTI_TMR_CMP_BLK_INDEX_0:
                HW_WR_FIELD32(baseAddr + CSL_RTI_RTIINTCLRENABLE, CSL_RTI_RTICOMP0CLR_COMP0CLR, RTI_TMR_INT_AUTO_CLR_ENABLE_FLAG);
                break;

            case RTI_TMR_CMP_BLK_INDEX_1:
                HW_WR_FIELD32(baseAddr + CSL_RTI_RTIINTCLRENABLE, CSL_RTI_RTICOMP1CLR_COMP1CLR, RTI_TMR_INT_AUTO_CLR_ENABLE_FLAG);
                break;

            case RTI_TMR_CMP_BLK_INDEX_2:
                HW_WR_FIELD32(baseAddr + CSL_RTI_RTIINTCLRENABLE, CSL_RTI_RTICOMP2CLR_COMP2CLR, RTI_TMR_INT_AUTO_CLR_ENABLE_FLAG);
                break;

            case RTI_TMR_CMP_BLK_INDEX_3:
                HW_WR_FIELD32(baseAddr + CSL_RTI_RTIINTCLRENABLE, CSL_RTI_RTICOMP3CLR_COMP3CLR, RTI_TMR_INT_AUTO_CLR_ENABLE_FLAG);
                break;

            default:
                break;
        }
        retVal = SystemP_SUCCESS;
    }
    return (retVal);
}

int32_t RTI_intAutoClearDisable(uint32_t baseAddr, uint32_t cmpIndex)
{
    int32_t retVal = SystemP_FAILURE;

    if ((baseAddr != ((uint32_t) NULL)) &&
        (cmpIndex <= RTI_TMR_CMP_BLK_INDEX_MAX))
    {
        /* Configure the compare blocks */
        switch (cmpIndex)
        {
            case RTI_TMR_CMP_BLK_INDEX_0:
                HW_WR_FIELD32(baseAddr + CSL_RTI_RTIINTCLRENABLE, CSL_RTI_RTICOMP0CLR_COMP0CLR, RTI_TMR_INT_AUTO_CLR_DISABLE_FLAG);
                break;

            case RTI_TMR_CMP_BLK_INDEX_1:
                HW_WR_FIELD32(baseAddr + CSL_RTI_RTIINTCLRENABLE, CSL_RTI_RTICOMP1CLR_COMP1CLR, RTI_TMR_INT_AUTO_CLR_DISABLE_FLAG);
                break;

            case RTI_TMR_CMP_BLK_INDEX_2:
                HW_WR_FIELD32(baseAddr + CSL_RTI_RTIINTCLRENABLE, CSL_RTI_RTICOMP2CLR_COMP2CLR, RTI_TMR_INT_AUTO_CLR_DISABLE_FLAG);
                break;

            case RTI_TMR_CMP_BLK_INDEX_3:
                HW_WR_FIELD32(baseAddr + CSL_RTI_RTIINTCLRENABLE, CSL_RTI_RTICOMP3CLR_COMP3CLR, RTI_TMR_INT_AUTO_CLR_DISABLE_FLAG);
                break;

            default:
                break;
        }
        retVal = SystemP_SUCCESS;
    }
    return (retVal);
}
