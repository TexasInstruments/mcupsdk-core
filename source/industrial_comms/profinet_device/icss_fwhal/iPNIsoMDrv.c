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
#include <string.h>
#include "iPNIsoMDrv.h"
#include "PN_HandleDef.h"
#include <drivers/hw_include/hw_types.h>

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


PNISOM_Handle PN_ISO_initGPIOEvent(PN_Handle pnHandle, uint8_t isoMode, uint32_t timeval, uint32_t duration)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);
    uint32_t iepCmpCfg = 0;
    PN_IsoMConfig *isomConfig = &(pnHandle->pnIsoMConfig);
    uint8_t enableMask = 0;
    uint32_t timevalComp=0;
    PNISOM_Handle tempHandle;
    uint8_t regVal;
    PN_IntAttrs *intConfig = &(isomConfig->isoMIntConfig);
    HwiP_Params hwiParams;
    uint32_t status = SystemP_FAILURE;

    /*TODO: Review the changes in ISOM Handle*/
    // if(!(handle= (uint32_t*)malloc(sizeof(uint32_t))))
    // {
    //     return NULL;
    // }
    if(isomConfig->isoMNumEvents >= PNISO_MAX_NUM_EVENTS)
    {
        return NULL;
    }
    else
    {
        if(PNISO_MODE_NONE == HW_RD_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_TYPE1))
        {
            HW_WR_REG32(pruicssHwAttrs->pru0DramBase + ISOM_TIO_TIMEVAL1, timeval);  /*ISOM_TIO_TIMEVAL*/
            HW_WR_REG32(pruicssHwAttrs->pru0DramBase + ISOM_TIO_DURATION1, timeval + duration); /*Not used*/
            HW_WR_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_TYPE1, isoMode); /*ISOM_TIO_TYPE*/

            enableMask = 0x1;
            regVal = HW_RD_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_ENABLE_OFFSET);
            regVal &= ~enableMask;
            HW_WR_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_ENABLE_OFFSET, regVal);

            /*TODO: Review this*/
            // isomConfig->event1Handle = (uint32_t)(handle);
            // *handle = ISOM_TIO_TIMEVAL1;
            isomConfig->event1Handle = (uint32_t)(&(pnHandle->pnIsoMObject));
            pnHandle->pnIsoMObject = ISOM_TIO_TIMEVAL1;
        }
        else if(PNISO_MODE_NONE == HW_RD_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_TYPE2))
        {

            timevalComp = HW_RD_REG32(pruicssHwAttrs->pru0DramBase + ISOM_TIO_TIMEVAL1);
            if(timeval >= timevalComp)
            {
                enableMask = HW_RD_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_ENABLE_OFFSET);
                HW_WR_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_ENABLE_OFFSET, 0); /*Disable all signals*/

                HW_WR_REG32(pruicssHwAttrs->pru0DramBase + ISOM_TIO_TIMEVAL2, timeval);  /*ISOM_TIO_TIMEVAL*/
                HW_WR_REG32(pruicssHwAttrs->pru0DramBase + ISOM_TIO_DURATION2, timeval + duration); /*Not used*/
                HW_WR_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_TYPE2, isoMode); /*ISOM_TIO_TYPE*/

                enableMask &= ~((uint8_t)0x2);
                HW_WR_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_ENABLE_OFFSET, enableMask); /*Disable mode*/

                /*TODO: Review this*/
                // isomConfig->event2Handle = (uint32_t)handle;
                // *handle = ISOM_TIO_TIMEVAL2;
                isomConfig->event2Handle = (uint32_t)(&(pnHandle->pnIsoMObject));
                pnHandle->pnIsoMObject = ISOM_TIO_TIMEVAL2;
            }
            else if((timeval < timevalComp))
            {

                enableMask = HW_RD_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_ENABLE_OFFSET);
                enableMask <<= 1; // Move the enable bit0 to bit1
                HW_WR_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_ENABLE_OFFSET, 0); /*Disable all signals*/

                memcpy((void*)(pruicssHwAttrs->pru0DramBase + ISOM_TIO_TIMEVAL2),(void*)(pruicssHwAttrs->pru0DramBase + ISOM_TIO_TIMEVAL1),9);

                HW_WR_REG32(pruicssHwAttrs->pru0DramBase + ISOM_TIO_TIMEVAL1, timeval);  /*ISOM_TIO_TIMEVAL*/
                HW_WR_REG32(pruicssHwAttrs->pru0DramBase + ISOM_TIO_DURATION1, timeval + duration); /*Not used*/
                HW_WR_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_TYPE1, isoMode); /*ISOM_TIO_TYPE*/

                enableMask &= ~((uint8_t)0x1);
                HW_WR_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_ENABLE_OFFSET, enableMask); /*Disable mode*/

                isomConfig->event2Handle = isomConfig->event1Handle;

                tempHandle = (PNISOM_Handle)isomConfig->event2Handle;
                *(tempHandle) = ISOM_TIO_TIMEVAL2;

                /*TODO: Review this*/
                // isomConfig->event1Handle = (uint32_t)handle;
                // *handle = ISOM_TIO_TIMEVAL1;
                isomConfig->event1Handle = (uint32_t)(&(pnHandle->pnIsoMObject));
                pnHandle->pnIsoMObject = ISOM_TIO_TIMEVAL1;
            }
        }
        else
            return NULL;

        if(PNISO_MODE_INTERRUPT == isoMode)
        {
            if(!(isomConfig->isoMIntCreateFlag))
            {
                HwiP_Params_init(&hwiParams);

                /* setup ISOM ISR*/
                hwiParams.intNum = intConfig->coreIntNum;
                hwiParams.callback = (HwiP_FxnCallback)(intConfig->isrFnPtr);
                hwiParams.args = (void *)intConfig->args;
                hwiParams.priority = intConfig->intPrio;

                status = HwiP_construct(&(isomConfig->isoMInterruptObject), &hwiParams);
                DebugP_assert(status == SystemP_SUCCESS);

                isomConfig->isoMIntCreateFlag = 1;
            }
        }

        iepCmpCfg = HW_RD_REG32(pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG);
        iepCmpCfg = iepCmpCfg | 0x48;  /*Enable CMP2 and CMP5*/
        HW_WR_REG32(pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG, iepCmpCfg);

        /*configure the pulse width for sync signal. need to configure nof cycles*/
        HW_WR_REG32(pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_SYNC_PWIDTH_REG, duration/5);

        isomConfig->isoMNumEvents++;
        /* TODO: Review this*/
        return (&(pnHandle->pnIsoMObject));
    }
}

void PN_ISO_enableGPIOEvent(PN_Handle pnHandle,PNISOM_Handle isoHandle)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);
    uint32_t offset = *(uint32_t*)isoHandle;
    uint8_t enableMask = 0;
    uint8_t regVal;

    if(offset == ISOM_TIO_TIMEVAL1)
        enableMask = 0x1;
    else if(offset == ISOM_TIO_TIMEVAL2)
        enableMask = 0x2;

    regVal = HW_RD_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_ENABLE_OFFSET);
    regVal |= enableMask;
    HW_WR_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_ENABLE_OFFSET, regVal); /*Enable ISOM*/
}

void PN_ISO_disableGPIOEvent(PN_Handle pnHandle,PNISOM_Handle isoHandle)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);

    uint32_t offset = *(uint32_t*)isoHandle;
    uint8_t enableMask = 0;
    uint8_t regVal;

    if(offset == ISOM_TIO_TIMEVAL1)
        enableMask = 0x1;
    else if(offset == ISOM_TIO_TIMEVAL2)
        enableMask = 0x2;

    regVal = HW_RD_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_ENABLE_OFFSET);
    regVal &= ~enableMask;
    HW_WR_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_ENABLE_OFFSET, regVal); /*Disable ISOM*/
}

void PN_ISO_deInitGPIOEvent(PN_Handle pnHandle,PNISOM_Handle isoHandle)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);
    PN_IsoMConfig* isomConfig = &(pnHandle->pnIsoMConfig);
    uint32_t offset = *(uint32_t*)isoHandle;
    uint8_t enableMask = 0;
    uint8_t othermode=0;
    uint32_t modeoffset=0;
    PNISOM_Handle tempHandle;
    uint8_t mode = HW_RD_REG8(pruicssHwAttrs->pru0DramBase + offset + 8);
    uint8_t regVal;

    if(offset == ISOM_TIO_TIMEVAL1)
    {
        modeoffset = ISOM_TIO_TYPE2;
        enableMask = 0x1;
        othermode =  HW_RD_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_TYPE2);

        if(PNISO_MODE_NONE == othermode)
        {
            enableMask = 0x1;

            HW_WR_REG32(pruicssHwAttrs->pru0DramBase + ISOM_TIO_TIMEVAL1, PNISO_MODE_DISABLE);
            HW_WR_REG32(pruicssHwAttrs->pru0DramBase + ISOM_TIO_DURATION1, PNISO_MODE_DISABLE);
            HW_WR_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_TYPE1, PNISO_MODE_DISABLE); /*Disable mode*/

            regVal = HW_RD_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_ENABLE_OFFSET);
            regVal &= ~enableMask;
            HW_WR_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_ENABLE_OFFSET, regVal); /*Disable ISOM*/

            isomConfig->event1Handle = 0;
            *isoHandle = 0;
        }
        else
        {
            enableMask = HW_RD_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_ENABLE_OFFSET);
            HW_WR_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_ENABLE_OFFSET, 0); /*Disable all signals*/

            memcpy((void*)(pruicssHwAttrs->pru0DramBase + ISOM_TIO_TIMEVAL1),(void*)(pruicssHwAttrs->pru0DramBase + ISOM_TIO_TIMEVAL2),9);

            HW_WR_REG32(pruicssHwAttrs->pru0DramBase + ISOM_TIO_TIMEVAL2, PNISO_MODE_DISABLE);
            HW_WR_REG32(pruicssHwAttrs->pru0DramBase + ISOM_TIO_DURATION2, PNISO_MODE_DISABLE);
            HW_WR_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_TYPE2, PNISO_MODE_DISABLE); /*Disable mode*/

            enableMask >>= 1; // Move the enable bit1 to bit0
            enableMask &= ~((uint8_t)0x2);
            HW_WR_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_ENABLE_OFFSET, enableMask); /*Disable ISOM*/

            isomConfig->event1Handle = isomConfig->event2Handle;
            tempHandle = (PNISOM_Handle)isomConfig->event1Handle;
            *(tempHandle) = ISOM_TIO_TIMEVAL1;

            isomConfig->event2Handle = 0;
            *isoHandle = 0;
        }

    }
    else if(offset == ISOM_TIO_TIMEVAL2)
    {
        modeoffset = ISOM_TIO_TYPE1;

        enableMask = HW_RD_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_ENABLE_OFFSET);
        HW_WR_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_ENABLE_OFFSET, 0); /*Disable all signals*/

        HW_WR_REG32(pruicssHwAttrs->pru0DramBase + ISOM_TIO_TIMEVAL2, PNISO_MODE_DISABLE);
        HW_WR_REG32(pruicssHwAttrs->pru0DramBase + ISOM_TIO_DURATION2, PNISO_MODE_DISABLE);
        HW_WR_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_TYPE2, PNISO_MODE_DISABLE); /*Disable mode*/

        enableMask &= ~((uint8_t)0x2);
        HW_WR_REG8(pruicssHwAttrs->pru0DramBase + ISOM_TIO_ENABLE_OFFSET, enableMask); /*Disable ISOM*/

        isomConfig->event2Handle = 0;
        *isoHandle = 0;
    }
    else
        return;

    if(PNISO_MODE_INTERRUPT == mode)
    {
        othermode = HW_RD_REG8(pruicssHwAttrs->pru0DramBase + modeoffset);

        if((othermode != PNISO_MODE_INTERRUPT) && (isomConfig->isoMIntCreateFlag))
        {
            HwiP_destruct(&(isomConfig->isoMInterruptObject));
            isomConfig->isoMIntCreateFlag = 0;
        }
    }

    isomConfig->isoMNumEvents--;

    /*TODO: Review this*/
    // free(isoHandle);
    pnHandle->pnIsoMObject = 0;
}
