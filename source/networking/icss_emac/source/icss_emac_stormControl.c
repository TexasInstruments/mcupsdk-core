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

#include "icss_emac_stormControl.h"
#include "icss_emac_local.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
* @brief   returns true if storm prevention is enabled
*
* @param stormPrevPtr Pointer to Storm Prevention member instance for that port
* @param spType storm prevention type, weather BC/MC/UC
*
* @retval Status. 1 if storm prevention is enabled on any port else 0
*/
inline uint8_t ICSS_EMAC_ifStormPreventionEnabled(const ICSS_EMAC_StormPrevention   *stormPrevPtr,
                                                  uint8_t                           spType);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void ICSS_EMAC_initStormPreventionTable(uint8_t             portnum,
                                        ICSS_EMAC_Handle    icssEmacHandle,
                                        uint8_t             spType)
{
    ICSS_EMAC_StormPrevention *stormPrevPtr;

    if(ICSS_EMAC_MODE_SWITCH != ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask)
    {
        portnum = 1U;
    }

    stormPrevPtr = (ICSS_EMAC_StormPrevention *)(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->stormPrev));
    stormPrevPtr += (portnum - 1U);

    ICSS_EMAC_setCreditValue(ICSS_EMAC_DEFAULT_CREDITS, stormPrevPtr, spType);
    ICSS_EMAC_enableStormPrevention(portnum, icssEmacHandle, spType);
    
    /* Reset credit value during init */
    ICSS_EMAC_resetStormPreventionCounter(icssEmacHandle, spType);
}                                        

void ICSS_EMAC_disableStormPrevention(uint8_t           portnum,
                                      ICSS_EMAC_Handle  icssEmacHandle,
                                      uint8_t           spType)
{
    uint16_t                    *controlPointer;
    ICSS_EMAC_StormPrevention   *stormPrevPtr;
    uint32_t                    temp_addr = 0U;
    uint32_t                    *stormPreventionOffsetPtr[2] = {NULL, NULL};
    uint16_t                    *suppressionEnabledPtr = NULL;
    uint16_t                    *creditsPtr = NULL;

    if (portnum <= (uint8_t)ICSS_EMAC_PORT_2) 
    {
        if(ICSS_EMAC_MODE_SWITCH != ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask)
        {
            portnum = 1U;
        }

        stormPrevPtr = (ICSS_EMAC_StormPrevention*)(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->stormPrev));
        stormPrevPtr += (portnum - 1U);
        ICSS_EMAC_checkStormPreventionType(stormPreventionOffsetPtr, &suppressionEnabledPtr, &creditsPtr, spType, icssEmacHandle, stormPrevPtr);

        *(suppressionEnabledPtr) = 0;

        if(portnum == (uint8_t)ICSS_EMAC_PORT_1)
        {
            temp_addr = (uint32_t)stormPreventionOffsetPtr[0];
            controlPointer = (uint16_t*)(temp_addr);
        } 
        else
        {
            temp_addr = (uint32_t)stormPreventionOffsetPtr[1];
            controlPointer = (uint16_t*)(temp_addr);
        }
        ICSS_EMAC_byteCopy((uint8_t*)controlPointer, (uint8_t*)suppressionEnabledPtr, 2);
    }
    return;
}                                      

void ICSS_EMAC_setCreditValue(uint16_t                  creditValue,
                              ICSS_EMAC_StormPrevention *stormPrevPtr,
                              uint8_t                   spType)
{
    uint16_t *creditsPtr = NULL;

    switch(spType)
    {
        case ICSS_EMAC_BC_STORM_PREVENTION:
            creditsPtr = &(stormPrevPtr->creditsBC);
            break;
        case ICSS_EMAC_MC_STORM_PREVENTION:
            creditsPtr = &(stormPrevPtr->creditsMC);
            break;
        case ICSS_EMAC_UC_STORM_PREVENTION:
            creditsPtr = &(stormPrevPtr->creditsUC);
            break;
        default:
            break;
    }
    if (creditsPtr != NULL)
    {
        *(creditsPtr) = creditValue;
    }
}                              

void ICSS_EMAC_enableStormPrevention(uint8_t            portnum,
                                     ICSS_EMAC_Handle   icssEmacHandle,
                                     uint8_t            spType)
{
    uint16_t                    *controlPointer;
    ICSS_EMAC_StormPrevention   *stormPrevPtr;
    uint32_t                    temp_addr = 0U;
    uint32_t                    *stormPreventionOffsetPtr[2] = {NULL, NULL};
    uint16_t                    *suppressionEnabledPtr = NULL;
    uint16_t                    *creditsPtr = NULL;

    if (portnum <= (uint8_t)ICSS_EMAC_PORT_2) 
    {
        if(ICSS_EMAC_MODE_SWITCH != ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask)
        {
            portnum = 1u;
        }

        /*get the pointer to the correct port*/
        stormPrevPtr = (ICSS_EMAC_StormPrevention*)(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->stormPrev));
        stormPrevPtr += (portnum - 1u);
        ICSS_EMAC_checkStormPreventionType(stormPreventionOffsetPtr, &suppressionEnabledPtr, &creditsPtr, spType, icssEmacHandle, stormPrevPtr);

        *(suppressionEnabledPtr) = 1;

        if(portnum == (uint8_t)ICSS_EMAC_PORT_1)
        {
            temp_addr = (uint32_t)stormPreventionOffsetPtr[0];
            controlPointer = (uint16_t*)(temp_addr);
        }
        else
        {
            temp_addr = (uint32_t)stormPreventionOffsetPtr[1];
            controlPointer = (uint16_t*)(temp_addr);
        }
        ICSS_EMAC_byteCopy((uint8_t*)controlPointer, (uint8_t*)suppressionEnabledPtr, 2);
    }
    return;
}                                     

void ICSS_EMAC_resetStormPreventionCounter(ICSS_EMAC_Handle icssEmacHandle,
                                           uint8_t          spType)
{
    uint32_t                    *controlPointer;
    uint32_t                    controlWord;
    uint32_t                    temp_addr = 0U;
    uint32_t                    temp_var1 = 0u;
    ICSS_EMAC_StormPrevention   *stormPrevPtr = (ICSS_EMAC_StormPrevention*)(&(((ICSS_EMAC_Object *)icssEmacHandle->object)->stormPrev));
    uint32_t                    *stormPreventionOffsetPtr[2] = {NULL, NULL};
    uint16_t                    *suppressionEnabledPtr = NULL;
    uint16_t                    *creditsPtr = NULL;

    ICSS_EMAC_checkStormPreventionType(stormPreventionOffsetPtr, &suppressionEnabledPtr, &creditsPtr, spType, icssEmacHandle, stormPrevPtr);
    if(*suppressionEnabledPtr)
    {
        temp_addr = (uint32_t)stormPreventionOffsetPtr[0];
        controlPointer = (uint32_t*)(temp_addr);
        temp_var1 = (((uint32_t)(*suppressionEnabledPtr)) | ((uint32_t)(((uint32_t)(*creditsPtr)) << 8U)));
        controlWord = ((uint32_t)(temp_var1));
        ICSS_EMAC_byteCopy((uint8_t*)controlPointer, (uint8_t*)(&controlWord), 4);
    }

    if(ICSS_EMAC_MODE_SWITCH == ((ICSS_EMAC_Attrs *)icssEmacHandle->attrs)->portMask) 
    {
        /*Access next port member*/
        stormPrevPtr += 1;
        ICSS_EMAC_checkStormPreventionType(stormPreventionOffsetPtr, &suppressionEnabledPtr, &creditsPtr, spType, icssEmacHandle, stormPrevPtr);

        if(*suppressionEnabledPtr)
        {
            temp_addr = (uint32_t)stormPreventionOffsetPtr[1];
            controlPointer = (uint32_t*)(temp_addr);
            temp_var1 = (((uint32_t)(*suppressionEnabledPtr)) | ((uint32_t)(((uint32_t)(*creditsPtr)) << 8U)));
            controlWord = ((uint32_t)(temp_var1));
            ICSS_EMAC_byteCopy((uint8_t*)controlPointer, (uint8_t*)(&controlWord), 4);
        }
    }
    return;
}                                           

void ICSS_EMAC_checkStormPreventionType(uint32_t                    **stormPreventionOffsetPtr,
                                        uint16_t                    **suppressionEnabledPtr,
                                        uint16_t                    **creditsPtr,
                                        uint8_t                     spType,
                                        ICSS_EMAC_Handle            icssEmacHandle,
                                        ICSS_EMAC_StormPrevention   *stormPrevPtr)
{
    ICSS_EMAC_FwStaticMmap  *pStaticMMap = (&((ICSS_EMAC_Object *)icssEmacHandle->object)->fwStaticMMap);
    PRUICSS_Handle          pruicssHandle = ((ICSS_EMAC_Object *)icssEmacHandle->object)->pruicssHandle;
    PRUICSS_HwAttrs const   *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pruicssHandle->hwAttrs); 

    switch(spType)
    {
        case ICSS_EMAC_BC_STORM_PREVENTION:
            stormPreventionOffsetPtr[0] = (uint32_t *)(pruicssHwAttrs->pru0DramBase + pStaticMMap->stormPreventionOffsetBC);
            stormPreventionOffsetPtr[1] = (uint32_t *)(pruicssHwAttrs->pru1DramBase + pStaticMMap->stormPreventionOffsetBC);
            *suppressionEnabledPtr = &(stormPrevPtr->suppressionEnabledBC);
            *creditsPtr = &(stormPrevPtr->creditsBC);
            break;
        case ICSS_EMAC_MC_STORM_PREVENTION:
            stormPreventionOffsetPtr[0] = (uint32_t *)(pruicssHwAttrs->pru0DramBase + pStaticMMap->stormPreventionOffsetMC);
            stormPreventionOffsetPtr[1] = (uint32_t *)(pruicssHwAttrs->pru1DramBase + pStaticMMap->stormPreventionOffsetMC);
            *suppressionEnabledPtr = &(stormPrevPtr->suppressionEnabledMC);
            *creditsPtr = &(stormPrevPtr->creditsMC);
            break;
        case ICSS_EMAC_UC_STORM_PREVENTION:
            stormPreventionOffsetPtr[0] = (uint32_t *)(pruicssHwAttrs->pru0DramBase + pStaticMMap->stormPreventionOffsetUC);
            stormPreventionOffsetPtr[1] = (uint32_t *)(pruicssHwAttrs->pru1DramBase + pStaticMMap->stormPreventionOffsetUC);
            *suppressionEnabledPtr = &(stormPrevPtr->suppressionEnabledUC);
            *creditsPtr = &(stormPrevPtr->creditsUC);
            break;
        default:
            break;
    }
}                                        

void ICSS_EMAC_byteCopy(uint8_t *dst_ptr, uint8_t *src_ptr, uint32_t size_bytes)
{
    uint32_t i;
    for (i = 0; i < size_bytes; i++)
    {
        *dst_ptr++ = *src_ptr++;
    }
}

inline uint8_t ICSS_EMAC_ifStormPreventionEnabled(const ICSS_EMAC_StormPrevention   *stormPrevPtr,
                                                  uint8_t                           spType)
{
    uint16_t suppressionEnabled;

    switch(spType)
    {
        case ICSS_EMAC_BC_STORM_PREVENTION:
            suppressionEnabled = (stormPrevPtr->suppressionEnabledBC);
            break;
        case ICSS_EMAC_MC_STORM_PREVENTION:
            suppressionEnabled = (stormPrevPtr->suppressionEnabledMC);
            break;
        case ICSS_EMAC_UC_STORM_PREVENTION:
            suppressionEnabled = (stormPrevPtr->suppressionEnabledUC);
            break;
        default:
            suppressionEnabled = 0U;
            break;
    }
    return suppressionEnabled;
}
