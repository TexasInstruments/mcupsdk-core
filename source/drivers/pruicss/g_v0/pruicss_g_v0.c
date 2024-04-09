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

#include <stddef.h>
#include <stdint.h>
#include <drivers/pruicss.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/hw_types.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

typedef struct PRUICSS_MemInfo_s {
    uintptr_t   addr;
    uint32_t    size;
} PRUICSS_MemInfo;

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

static uintptr_t PRUICSS_getCtrlAddr(PRUICSS_HwAttrs const *hwAttrs,
                                     uint8_t pruNum);

static void PRUICSS_getMemInfo(PRUICSS_HwAttrs const *hwAttrs,
                               uint32_t pruicssMem,
                               PRUICSS_MemInfo *pMemInfo);

/**
 * \brief   PRUICSS Interrupt Handler
 *
 * \param   ptrPpruEvtoutNum PRUICSS handler pointer
 *
 * \return  None
 *
 **/
static void PRUICSS_hwiIntHandler(uintptr_t ptrPpruEvtoutNum);

/**
 * @brief   This function sets System-Channel Map registers
 *
 * @param   sysevt         System event number
 * @param   channel        Host channel number
 * @param   polarity       Polarity of event
 * @param   type           Type of event
 * @param   baseaddr       Base address of PRUICSS
 *
 * @return   None
 */
static void PRUICSS_intcSetCmr(uint8_t sysevt,
                               uint8_t channel,
                               uint8_t polarity,
                               uint8_t type,
                               uintptr_t baseaddr);
/**
 * \brief    This function sets Channel-Host Map registers
 *
 * \param    channel         Channel number
 * \param    host            Host number
 * @param    baseaddr        Base address of PRUICSS
 *
 * \return   None
 */
static void PRUICSS_intcSetHmr(uint8_t channel,
                               uint8_t host,
                               uintptr_t baseaddr);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern int32_t gPruIcssConfigNum;
extern PRUICSS_Config gPruIcssConfig[];

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t PRUICSS_init(void)
{
    return SystemP_SUCCESS;
}

int32_t PRUICSS_deinit(void)
{
    return SystemP_SUCCESS;
}

PRUICSS_Handle PRUICSS_open(uint32_t index)
{
    PRUICSS_Handle          handle = NULL;
    PRUICSS_Object          *object = NULL;
    PRUICSS_HwAttrs const   *hwAttrs = NULL;

    /* Check index */
    if(index >= gPruIcssConfigNum)
    {
        return NULL;
    }
    else
    {
        handle = (PRUICSS_Handle)(&gPruIcssConfig[index]);
    }
    hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;
    object = (PRUICSS_Object *)handle->object;
    object->pruicssVersion = CSL_REG32_RD((hwAttrs->cfgRegBase + CSL_ICSSCFG_REVID)) & 0xFFFFU;

    return handle;
}

int32_t PRUICSS_close(PRUICSS_Handle handle)
{
    return SystemP_SUCCESS;
}

int32_t PRUICSS_intcInit(PRUICSS_Handle              handle,
                         const PRUICSS_IntcInitData  *intcInitData)
{
    uintptr_t               baseaddr;
    PRUICSS_HwAttrs const   *hwAttrs;
    uint32_t                i = 0;
    uint32_t                mask[5];
    int32_t                 retVal = SystemP_SUCCESS;

    if((handle == NULL) || (intcInitData == NULL))
    {
        retVal = SystemP_FAILURE;
    }

    if(retVal == SystemP_SUCCESS)
    {
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;
        baseaddr = hwAttrs->intcRegBase;

        HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_POLARITY_REG0), 0xFFFFFFFFU);
        HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_POLARITY_REG1), 0xFFFFFFFFU);
        HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_POLARITY_REG2), 0xFFFFFFFFU);
        HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_POLARITY_REG3), 0xFFFFFFFFU);
        HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_POLARITY_REG4), 0xFFFFFFFFU);

        HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_TYPE_REG0), 0xFFFFFFFFU);
        HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_TYPE_REG1), 0xFFFFFFFFU);
        HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_TYPE_REG2), 0xFFFFFFFFU);
        HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_TYPE_REG3), 0xFFFFFFFFU);
        HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_TYPE_REG4), 0xFFFFFFFFU);

        for (i = 0; i < (PRUICSS_INTC_NUM_SYS_EVTS + 3U) >> 2; i++)
        {
            HW_WR_REG32(((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_CH_MAP_REG0) + (i << 2) ), 0x0U);
        }
        for (i = 0;
             ((intcInitData->sysevtToChannelMap[i].sysevt != 0xFF)
             && (intcInitData->sysevtToChannelMap[i].channel != 0xFF));
             i++)
        {
            PRUICSS_intcSetCmr(intcInitData->sysevtToChannelMap[i].sysevt,
                               intcInitData->sysevtToChannelMap[i].channel,
                               ((uint8_t)(~(intcInitData->sysevtToChannelMap[i].polarity)))&0x01U,
                               ((uint8_t)(~(intcInitData->sysevtToChannelMap[i].type)))&0x01U,
                               baseaddr);
        }

        for (i = 0; i < (PRUICSS_INTC_NUM_HOST_INTERRUPTS + 3U) >> 2; i++)
        {
            HW_WR_REG32(((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_HINT_MAP_REG0) + (i << 2) ), 0x0U);
        }
        for (i = 0;
               ((i<PRUICSS_INTC_NUM_HOST_INTERRUPTS) &&
               (intcInitData->channelToHostMap[i].channel != 0xFF) &&
               (intcInitData->channelToHostMap[i].host != 0xFF));
               i++)
        {
            PRUICSS_intcSetHmr(intcInitData->channelToHostMap[i].channel,
                               intcInitData->channelToHostMap[i].host,
                               baseaddr);
        }

        mask[0] = 0;
        mask[1] = 0;
        mask[2] = 0;
        mask[3] = 0;
        mask[4] = 0;

        for (i = 0; (uint8_t)intcInitData->sysevtsEnabled[i] != 0xFFU; i++)
        {
            if (intcInitData->sysevtsEnabled[i] < 32)
            {
                mask[0] = mask[0] + (((uint32_t)1U) << (intcInitData->sysevtsEnabled[i]));
            }
            else if (intcInitData->sysevtsEnabled[i] < 64)
            {
                mask[1] = mask[1] + (((uint32_t)1U) << (intcInitData->sysevtsEnabled[i] - 32));
            }
            else if (intcInitData->sysevtsEnabled[i] < 96)
            {
                mask[2] = mask[2] + (((uint32_t)1U) << (intcInitData->sysevtsEnabled[i] - 64));
            }
            else if (intcInitData->sysevtsEnabled[i] < 128)
            {
                mask[3] = mask[3] + (((uint32_t)1U) << (intcInitData->sysevtsEnabled[i] - 96));
            }
            else if (intcInitData->sysevtsEnabled[i] < 160)
            {
                mask[4] = mask[4] + (((uint32_t)1U) << (intcInitData->sysevtsEnabled[i] - 128));
            }
            else
            {
                /* Error */
                retVal = SystemP_FAILURE;
                break;
            }
        }
    }

    if(retVal == SystemP_SUCCESS)
    {
        HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_ENABLE_REG0), mask[0]);
        HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_ENA_STATUS_REG0), mask[0]);

        HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_ENABLE_REG1), mask[1]);
        HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_ENA_STATUS_REG1), mask[1]);

        HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_ENABLE_REG2), mask[2]);
        HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_ENA_STATUS_REG2), mask[2]);

        HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_ENABLE_REG3), mask[3]);
        HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_ENA_STATUS_REG3), mask[3]);

        HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_ENABLE_REG4), mask[4]);
        HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_ENA_STATUS_REG4), mask[4]);

        HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_ENABLE_HINT_REG0), intcInitData->hostEnableBitmask);

        HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_GLOBAL_ENABLE_HINT_REG), 0x1);
    }
    return retVal;
}

int32_t PRUICSS_registerIrqHandler(PRUICSS_Handle           handle,
                                   uint32_t                 pruEvtoutNum,
                                   int32_t                  intrNum,
                                   int32_t                  eventNum,
                                   uint8_t                  waitEnable,
                                   PRUICSS_IrqHandler       irqHandler)
{
    PRUICSS_Object      *object;
    HwiP_Params         hwiParams;
    int32_t             retVal = SystemP_SUCCESS;

    object = (PRUICSS_Object *)handle->object;

    if ((pruEvtoutNum >= PRUICSS_INTC_NUM_HOST_INTERRUPTS) || (irqHandler == NULL) || (waitEnable > 1))
    {
        retVal = SystemP_FAILURE;
    }
    else
    {
        object->pruEvntOutFnMapArray[pruEvtoutNum].waitEnable = waitEnable;

        /* create a Semaphore Instance */
        if(waitEnable)
        {
            retVal = SemaphoreP_constructBinary(&(object->pruEvntOutFnMapArray[pruEvtoutNum].semObj), 0);
        }

        if(retVal == SystemP_SUCCESS)
        {
            object->pruEvntOutFnMapArray[pruEvtoutNum].irqHandler = irqHandler;
            object->pruEvntOutFnMapArray[pruEvtoutNum].pruicssHandle = handle;

            HwiP_Params_init(&hwiParams);
            hwiParams.args = (void *)(&object->pruEvntOutFnMapArray[pruEvtoutNum]);
            /*TODO: Take priority as an argument to this function*/
            hwiParams.priority = 0x1;
            hwiParams.intNum = intrNum;
            hwiParams.callback = (HwiP_FxnCallback)PRUICSS_hwiIntHandler;

            retVal = HwiP_construct(&(object->pruEvntOutFnMapArray[pruEvtoutNum].hwiObj), &hwiParams);
        }

        if(retVal == SystemP_FAILURE)
        {
            SemaphoreP_destruct(&(object->pruEvntOutFnMapArray[pruEvtoutNum].semObj));
        }
    }
    return retVal;
}

int32_t PRUICSS_resetCore(PRUICSS_Handle handle, uint8_t pruNum)
{
    uintptr_t               baseaddr;
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pruNum < PRUICSS_NUM_CORES))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

        if (hwAttrs != NULL)
        {
            switch (pruNum)
            {
                case PRUICSS_PRU0:
                    baseaddr = hwAttrs->pru0CtrlRegBase;
                    HW_WR_FIELD32((baseaddr + CSL_ICSS_G_PR1_PDSP0_IRAM_CONTROL),  CSL_ICSS_G_PR1_PDSP0_IRAM_CONTROL_SOFT_RST_N, 0x0U);
                    break;
                case PRUICSS_PRU1:
                    baseaddr = hwAttrs->pru1CtrlRegBase;
                    HW_WR_FIELD32((baseaddr + CSL_ICSS_G_PR1_PDSP1_IRAM_CONTROL),  CSL_ICSS_G_PR1_PDSP1_IRAM_CONTROL_SOFT_RST_N, 0x0U);
                    break;
                case PRUICSS_RTU_PRU0:
                    baseaddr = hwAttrs->rtu0CtrlRegBase;
                    HW_WR_FIELD32((baseaddr + CSL_ICSS_G_PR1_RTU0_PR1_RTU0_IRAM_CONTROL),  CSL_ICSS_G_PR1_RTU0_PR1_RTU0_IRAM_CONTROL_SOFT_RST_N, 0x0U);
                    break;
                case PRUICSS_RTU_PRU1:
                    baseaddr = hwAttrs->rtu1CtrlRegBase;
                    HW_WR_FIELD32((baseaddr + CSL_ICSS_G_PR1_RTU1_PR1_RTU1_IRAM_CONTROL),  CSL_ICSS_G_PR1_RTU1_PR1_RTU1_IRAM_CONTROL_SOFT_RST_N, 0x0U);
                    break;
                case PRUICSS_TX_PRU0:
                    baseaddr = hwAttrs->txPru0CtrlRegBase;
                    HW_WR_FIELD32((baseaddr + CSL_ICSS_G_PR1_PDSP_TX0_IRAM_CONTROL),  CSL_ICSS_G_PR1_PDSP_TX0_IRAM_CONTROL_SOFT_RST_N, 0x0U);
                    break;
                case PRUICSS_TX_PRU1:
                    baseaddr = hwAttrs->txPru1CtrlRegBase;
                    HW_WR_FIELD32((baseaddr + CSL_ICSS_G_PR1_PDSP_TX1_IRAM_CONTROL),  CSL_ICSS_G_PR1_PDSP_TX1_IRAM_CONTROL_SOFT_RST_N, 0x0U);
                    break;
            }
        }
    }
    return retVal;
}

int32_t PRUICSS_disableCore(PRUICSS_Handle handle, uint8_t pruNum)
{
    uintptr_t               baseaddr;
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pruNum < PRUICSS_NUM_CORES))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

        if (hwAttrs != NULL)
        {
            switch (pruNum)
            {
                case PRUICSS_PRU0:
                    baseaddr = hwAttrs->pru0CtrlRegBase;
                    HW_WR_FIELD32((baseaddr + CSL_ICSS_G_PR1_PDSP0_IRAM_CONTROL),  CSL_ICSS_G_PR1_PDSP0_IRAM_CONTROL_PDSP_ENABLE, 0x0U);
                    break;
                case PRUICSS_PRU1:
                    baseaddr = hwAttrs->pru1CtrlRegBase;
                    HW_WR_FIELD32((baseaddr + CSL_ICSS_G_PR1_PDSP1_IRAM_CONTROL),  CSL_ICSS_G_PR1_PDSP1_IRAM_CONTROL_PDSP_ENABLE, 0x0U);
                    break;
                case PRUICSS_RTU_PRU0:
                    baseaddr = hwAttrs->rtu0CtrlRegBase;
                    HW_WR_FIELD32((baseaddr + CSL_ICSS_G_PR1_RTU0_PR1_RTU0_IRAM_CONTROL),  CSL_ICSS_G_PR1_RTU0_PR1_RTU0_IRAM_CONTROL_PDSP_ENABLE, 0x0U);
                    break;
                case PRUICSS_RTU_PRU1:
                    baseaddr = hwAttrs->rtu1CtrlRegBase;
                    HW_WR_FIELD32((baseaddr + CSL_ICSS_G_PR1_RTU1_PR1_RTU1_IRAM_CONTROL),  CSL_ICSS_G_PR1_RTU1_PR1_RTU1_IRAM_CONTROL_PDSP_ENABLE, 0x0U);
                    break;
                case PRUICSS_TX_PRU0:
                    baseaddr = hwAttrs->txPru0CtrlRegBase;
                    HW_WR_FIELD32((baseaddr + CSL_ICSS_G_PR1_PDSP_TX0_IRAM_CONTROL),  CSL_ICSS_G_PR1_PDSP_TX0_IRAM_CONTROL_PDSP_ENABLE, 0x0U);
                    break;
                case PRUICSS_TX_PRU1:
                    baseaddr = hwAttrs->txPru1CtrlRegBase;
                    HW_WR_FIELD32((baseaddr + CSL_ICSS_G_PR1_PDSP_TX1_IRAM_CONTROL),  CSL_ICSS_G_PR1_PDSP_TX1_IRAM_CONTROL_PDSP_ENABLE, 0x0U);
                    break;
            }
        }
    }
    return retVal;
}

int32_t PRUICSS_enableCore(PRUICSS_Handle handle, uint8_t pruNum)
{
    uintptr_t               baseaddr;
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pruNum < PRUICSS_NUM_CORES))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

        if (hwAttrs != NULL)
        {
            switch (pruNum)
            {
                case PRUICSS_PRU0:
                    baseaddr = hwAttrs->pru0CtrlRegBase;
                    HW_WR_FIELD32((baseaddr + CSL_ICSS_G_PR1_PDSP0_IRAM_CONTROL),  CSL_ICSS_G_PR1_PDSP0_IRAM_CONTROL_PDSP_ENABLE, 0x1U);
                    break;
                case PRUICSS_PRU1:
                    baseaddr = hwAttrs->pru1CtrlRegBase;
                    HW_WR_FIELD32((baseaddr + CSL_ICSS_G_PR1_PDSP1_IRAM_CONTROL),  CSL_ICSS_G_PR1_PDSP1_IRAM_CONTROL_PDSP_ENABLE, 0x1U);
                    break;
                case PRUICSS_RTU_PRU0:
                    baseaddr = hwAttrs->rtu0CtrlRegBase;
                    HW_WR_FIELD32((baseaddr + CSL_ICSS_G_PR1_RTU0_PR1_RTU0_IRAM_CONTROL),  CSL_ICSS_G_PR1_RTU0_PR1_RTU0_IRAM_CONTROL_PDSP_ENABLE, 0x1U);
                    break;
                case PRUICSS_RTU_PRU1:
                    baseaddr = hwAttrs->rtu1CtrlRegBase;
                    HW_WR_FIELD32((baseaddr + CSL_ICSS_G_PR1_RTU1_PR1_RTU1_IRAM_CONTROL),  CSL_ICSS_G_PR1_RTU1_PR1_RTU1_IRAM_CONTROL_PDSP_ENABLE, 0x1U);
                    break;
                case PRUICSS_TX_PRU0:
                    baseaddr = hwAttrs->txPru0CtrlRegBase;
                    HW_WR_FIELD32((baseaddr + CSL_ICSS_G_PR1_PDSP_TX0_IRAM_CONTROL),  CSL_ICSS_G_PR1_PDSP_TX0_IRAM_CONTROL_PDSP_ENABLE, 0x1U);
                    break;
                case PRUICSS_TX_PRU1:
                    baseaddr = hwAttrs->txPru1CtrlRegBase;
                    HW_WR_FIELD32((baseaddr + CSL_ICSS_G_PR1_PDSP_TX1_IRAM_CONTROL),  CSL_ICSS_G_PR1_PDSP_TX1_IRAM_CONTROL_PDSP_ENABLE, 0x1U);
                    break;
            }
        }
    }
    return retVal;
}

uint32_t PRUICSS_initMemory(PRUICSS_Handle handle, uint32_t pruicssMem)
{
    uintptr_t               tempAddr = 0U;
    uint32_t                i = 0;
    PRUICSS_MemInfo         memInfo = {0U, 0U};
    PRUICSS_HwAttrs const   *hwAttrs;
    if (handle != NULL)
    {
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;
        PRUICSS_getMemInfo(hwAttrs, pruicssMem, &memInfo);

        if(memInfo.size > 0)
        {
            /*TODO: Check if memset can be used*/
            for (i = 0U; i < memInfo.size; i = i + 4U)
            {
                tempAddr = (memInfo.addr + (uint32_t)i);
                HW_WR_REG32(tempAddr, (uint32_t)0x0);
            }
        }
    }
    return memInfo.size;
}

uint32_t PRUICSS_writeMemory(PRUICSS_Handle   handle,
                            uint32_t         pruicssMem,
                            uint32_t         wordoffset,
                            const uint32_t   *source_mem,
                            uint32_t         bytelength)
{
    uintptr_t               tempAddr = 0U;
    uint32_t                i;
    uint32_t                wordlength = 0;
    PRUICSS_MemInfo         memInfo = {0U, 0U};
    PRUICSS_HwAttrs const   *hwAttrs;

    if (handle != NULL)
    {
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;
        PRUICSS_getMemInfo(hwAttrs, pruicssMem, &memInfo);

        if (memInfo.addr != 0U)
        {
            wordlength = (bytelength + 3U) >> 2U;
            for (i = 0; i < wordlength; i++)
            {
                tempAddr = (memInfo.addr + (i << 2) + wordoffset);
                CSL_REG32_WR(tempAddr, source_mem[i]);
            }
        }
    }
    return wordlength;
}

uint32_t PRUICSS_readMemory(PRUICSS_Handle    handle,
                           uint32_t          pruicssMem,
                           uint32_t          wordoffset,
                           uint32_t          *dest_mem,
                           uint32_t          bytelength)
{
    uintptr_t               tempAddr = 0U;
    uint32_t                i;
    uint32_t                wordlength = 0U;
    PRUICSS_MemInfo         memInfo = {0U, 0U};
    PRUICSS_HwAttrs const   *hwAttrs;

    if (handle != NULL)
    {
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;
        PRUICSS_getMemInfo(hwAttrs, pruicssMem, &memInfo);

        if (memInfo.addr != 0)
        {
            wordlength = (bytelength + 3U) >> 2U;
            for (i = 0; i < wordlength; i++)
            {
                tempAddr = (memInfo.addr+ (i << 2) + wordoffset);
                dest_mem[i] = CSL_REG32_RD(tempAddr);
            }
        }
    }
    return wordlength;
}

int32_t PRUICSS_sendEvent(PRUICSS_Handle handle, uint32_t eventnum)
{
    uintptr_t               baseaddr;
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_SUCCESS;

    if (handle != NULL)
    {
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;
        baseaddr = hwAttrs->intcRegBase;

        switch ((eventnum >> 5))
        {
        case 0:
            HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_RAW_STATUS_REG0), ((uint32_t)1U) << eventnum);
            break;
        case 1:
            HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_RAW_STATUS_REG1), ((uint32_t)1U) << (eventnum - 32U));
            break;
        case 2:
            HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_RAW_STATUS_REG2), ((uint32_t)1U) << (eventnum - 64U));
            break;
        case 3:
            HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_RAW_STATUS_REG3), ((uint32_t)1U) << (eventnum - 96U));
            break;
        case 4:
            HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_RAW_STATUS_REG4), ((uint32_t)1U) << (eventnum - 128U));
            break;
        default:
            retVal = SystemP_FAILURE;
            break;
        }
    }
    return retVal;
}

int32_t PRUICSS_waitEvent(PRUICSS_Handle handle, uint32_t pruEvtoutNum)
{
    PRUICSS_Object  *object;
    int32_t         retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pruEvtoutNum < PRUICSS_INTC_NUM_HOST_INTERRUPTS))
    {
        object = (PRUICSS_Object *)handle->object;
        if(object->pruEvntOutFnMapArray[pruEvtoutNum].waitEnable == 1)
            retVal = SemaphoreP_pend(&(object->pruEvntOutFnMapArray[pruEvtoutNum].semObj), SystemP_WAIT_FOREVER);
    }
    return retVal;
}

int32_t PRUICSS_clearEvent(PRUICSS_Handle handle, uint32_t eventnum)
{
    uintptr_t               baseaddr;
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if (handle != NULL)
    {
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;
        baseaddr = hwAttrs->intcRegBase;
        HW_WR_REG32((baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_STATUS_CLR_INDEX_REG), (eventnum & CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_STATUS_CLR_INDEX_REG_STATUS_CLR_INDEX_MASK));
        retVal = SystemP_SUCCESS;
    }
    return retVal;
}

int32_t PRUICSS_sendWaitClearEvent(PRUICSS_Handle    handle,
                                   uint32_t          sendEventNum,
                                   uint32_t          pruEvtoutNum,
                                   uint32_t          ackEventNum)
{
    int32_t retVal = SystemP_FAILURE;

    retVal = PRUICSS_sendEvent(handle, sendEventNum);

    if(retVal == SystemP_SUCCESS)
    {
        retVal = PRUICSS_waitEvent(handle, pruEvtoutNum);
    }

    if(retVal == SystemP_SUCCESS)
    {
        retVal = PRUICSS_clearEvent(handle, ackEventNum);
    }
    return retVal;
}

uint32_t PRUICSS_getVersion(PRUICSS_Handle handle)
{
    PRUICSS_Object *object;
    object = (PRUICSS_Object *)handle->object;
    return object->pruicssVersion;
}

int32_t PRUICSS_setConstantTblEntry(PRUICSS_Handle  handle,
                                    uint8_t         pruNum,
                                    int32_t         constantTblEntry,
                                    uint32_t        constantTblVal)
{
    PRUICSS_HwAttrs const   *hwAttrs = NULL;
    uint32_t                baseaddr = 0U;
    uintptr_t               tempaddr = 0U;
    uint32_t                tempval = 0U;
    uint32_t                currentval = 0U;
    int32_t                 retVal = SystemP_SUCCESS;

    static uint32_t pruicssConstantIndex[PRUICSS_NUM_CONST_TBL_ENTRY] =
    {
        CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_BLOCK_INDEX_0, /*For entry 24, PRU-ICSS PRU0 Data RAM */
        CSL_ICSS_G_PR1_PDSP1_IRAM_CONSTANT_TABLE_BLOCK_INDEX_0, /*For entry 25, PRU-ICSS PRU1 Data RAM */
        CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_BLOCK_INDEX_1, /*For entry 26, PRU-ICSS IEP */
        CSL_ICSS_G_PR1_PDSP1_IRAM_CONSTANT_TABLE_BLOCK_INDEX_1, /*For entry 27, PRU-ICSS MII_RT*/
        CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_0,    /*For entry 28, PRU-ICSS Shared RAM*/
        CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_0,    /*For entry 29, TPCC*/
        CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_1,    /*For entry 30, L3 OCMC0 */
        CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_1     /*For entry 31, EMIF0 DDR Base */
    };

    /* verify constant table index is within valid range */
    if (((constantTblEntry < 0) || (constantTblEntry >= PRUICSS_NUM_CONST_TBL_ENTRY)) || (pruNum > PRUICSS_NUM_CORES))
    {
        retVal = SystemP_FAILURE;
    }
    else
    {
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;
        if(PRUICSS_PRU0 == pruNum)
        {
            baseaddr = hwAttrs->pru0CtrlRegBase;
        }
        else if(PRUICSS_PRU1 == pruNum)
        {
            baseaddr = hwAttrs->pru1CtrlRegBase;
        }
        else if(PRUICSS_RTU_PRU0 == pruNum)
        {
            baseaddr = hwAttrs->rtu0CtrlRegBase;
        }
        else if (PRUICSS_TX_PRU0 == pruNum)
        {
             baseaddr = hwAttrs->txPru0CtrlRegBase;;
        }
        else if(PRUICSS_RTU_PRU1 == pruNum)
        {
            baseaddr = hwAttrs->rtu1CtrlRegBase;
        }
        else if (PRUICSS_TX_PRU1 == pruNum)
        {
             baseaddr = hwAttrs->txPru1CtrlRegBase;;
        }
        else
        {
            retVal = SystemP_FAILURE;
        }
        if (retVal == SystemP_SUCCESS)
        {
            tempaddr = baseaddr + pruicssConstantIndex[constantTblEntry];
            switch (constantTblEntry)
            {
                /*NOTE: CSL macros for PRU0 are used irrespective of pruNum, as
                        values are same across different PRU cores*/
                case PRUICSS_CONST_TBL_ENTRY_C24:
                {
                    currentval = HW_RD_REG32(tempaddr);
                    currentval &= (~((uint32_t)CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_BLOCK_INDEX_0_C24_BLK_INDEX_MASK));
                    tempval = CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_BLOCK_INDEX_0_C24_BLK_INDEX_MASK & constantTblVal;
                    HW_WR_REG32(tempaddr, (uint32_t)(currentval | tempval));
                    break;
                }
                case PRUICSS_CONST_TBL_ENTRY_C25:
                {
                    currentval = HW_RD_REG32(tempaddr);
                    currentval &= (~((uint32_t)CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_BLOCK_INDEX_0_C25_BLK_INDEX_MASK));
                    tempval = constantTblVal << CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_BLOCK_INDEX_0_C25_BLK_INDEX_SHIFT;
                    tempval = CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_BLOCK_INDEX_0_C25_BLK_INDEX_MASK & tempval;
                    HW_WR_REG32(tempaddr, (uint32_t)(currentval | tempval));
                    break;
                }
                case PRUICSS_CONST_TBL_ENTRY_C26:
                {
                    currentval = HW_RD_REG32(tempaddr);
                    currentval &= (~((uint32_t)CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_BLOCK_INDEX_1_C26_BLK_INDEX_MASK));
                    tempval = constantTblVal << CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_BLOCK_INDEX_1_C26_BLK_INDEX_SHIFT;
                    tempval = CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_BLOCK_INDEX_1_C26_BLK_INDEX_MASK & tempval;
                    HW_WR_REG32(tempaddr, (uint32_t)(currentval | tempval));
                    break;
                }
                case PRUICSS_CONST_TBL_ENTRY_C27:
                {
                    currentval = HW_RD_REG32(tempaddr);
                    currentval &= (~((uint32_t)CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_BLOCK_INDEX_1_C27_BLK_INDEX_MASK));
                    tempval = constantTblVal << CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_BLOCK_INDEX_1_C27_BLK_INDEX_SHIFT;
                    tempval = CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_BLOCK_INDEX_1_C27_BLK_INDEX_MASK & tempval;
                    HW_WR_REG32(tempaddr, (uint32_t)(currentval | tempval));
                    break;
                }
                case PRUICSS_CONST_TBL_ENTRY_C28:
                {
                    currentval = HW_RD_REG32(tempaddr);
                    currentval &= (~((uint32_t)CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_0_C28_POINTER_MASK));
                    tempval = constantTblVal << CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_0_C28_POINTER_SHIFT;
                    tempval &= CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_0_C28_POINTER_MASK;
                    HW_WR_REG32(tempaddr, (uint32_t)(currentval | tempval));
                    break;
                }
                case PRUICSS_CONST_TBL_ENTRY_C29:
                {
                    currentval = HW_RD_REG32(tempaddr);
                    currentval &= (~((uint32_t)CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_0_C29_POINTER_MASK));
                    tempval = constantTblVal << CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_0_C29_POINTER_SHIFT;
                    tempval &= CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_0_C29_POINTER_MASK;
                    HW_WR_REG32(tempaddr, (uint32_t)(currentval | tempval));
                    break;
                }
                case PRUICSS_CONST_TBL_ENTRY_C30:
                {
                    currentval = HW_RD_REG32(tempaddr);
                    currentval &= (~((uint32_t)CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_1_C30_POINTER_MASK));
                    tempval = constantTblVal << CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_1_C30_POINTER_SHIFT;
                    tempval &= CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_1_C30_POINTER_MASK;
                    HW_WR_REG32(tempaddr, (uint32_t)(currentval | tempval));
                    break;
                }
                case PRUICSS_CONST_TBL_ENTRY_C31:
                {
                    currentval = HW_RD_REG32(tempaddr);
                    currentval &= (~((uint32_t)CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_1_C31_POINTER_MASK));
                    tempval = constantTblVal << CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_1_C31_POINTER_SHIFT;
                    tempval &= CSL_ICSS_G_PR1_PDSP0_IRAM_CONSTANT_TABLE_PROG_PTR_1_C31_POINTER_MASK;
                    HW_WR_REG32(tempaddr, (uint32_t)(currentval | tempval));
                    break;
                }
                default:
                {
                    break;
                }
            }
        }
    }
    return retVal;
}

int32_t PRUICSS_setIepClkSrc(PRUICSS_Handle handle, uint32_t source)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (source < 2))
    {
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;
        HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_IEPCLK), CSL_ICSSCFG_IEPCLK_OCP_EN, source);
        retVal = SystemP_SUCCESS;
    }

    return retVal;
}

int32_t PRUICSS_setGpMuxSelect(PRUICSS_Handle handle, uint8_t pruNum, uint8_t mode)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (mode < PRUICSS_NUM_GP_MUX_SEL_MODES))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

        switch (pruNum)
        {
            case PRUICSS_PRU0:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_GPCFG0), CSL_ICSSCFG_GPCFG0_PR1_PRU0_GP_MUX_SEL, mode);
                break;
            case PRUICSS_PRU1:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_GPCFG1), CSL_ICSSCFG_GPCFG1_PR1_PRU1_GP_MUX_SEL, mode);
                break;
            default:
                retVal = SystemP_FAILURE;
                break;
        }
    }

    return retVal;
}

int32_t PRUICSS_setGpiMode(PRUICSS_Handle handle, uint8_t pruNum, uint8_t mode)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (mode < PRUICSS_NUM_GPI_MODES))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

        switch (pruNum)
        {
            case PRUICSS_PRU0:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_GPCFG0), CSL_ICSSCFG_GPCFG0_PRU0_GPI_MODE, mode);
                break;
            case PRUICSS_PRU1:
                HW_WR_FIELD32((hwAttrs->cfgRegBase + CSL_ICSSCFG_GPCFG1), CSL_ICSSCFG_GPCFG1_PRU1_GPI_MODE, mode);
                break;
            default:
                retVal = SystemP_FAILURE;
                break;
        }
    }

    return retVal;
}

int32_t PRUICSS_setSaMuxMode(PRUICSS_Handle handle, uint8_t mode)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;
    uint32_t                regVal;

    if ((handle != NULL) && (mode < PRUICSS_NUM_SA_MUX_MODES))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

        regVal = HW_RD_REG32(hwAttrs->cfgRegBase + CSL_ICSSCFG_PIN_MX);
        /*FIXME: SHIFT macro to be used from CSL instead of 7.
                CSL_ICSSCFG_PIN_MX_PIN_MUX_SEL_SHIFT value is incorrect */
        regVal &= ~(1   << 7);
        regVal |=  (mode << 7);
        HW_WR_REG32(hwAttrs->cfgRegBase + CSL_ICSSCFG_PIN_MX, regVal);
    }

    return retVal;
}

int32_t PRUICSS_configureCycleCounter(PRUICSS_Handle handle, uint8_t pruNum, uint8_t enable)
{
    uintptr_t               baseaddr;
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (pruNum < PRUICSS_NUM_CORES) && (enable < 2))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

        if (hwAttrs != NULL)
        {
            switch (pruNum)
            {
                case PRUICSS_PRU0:
                    baseaddr = hwAttrs->pru0CtrlRegBase;
                    HW_WR_FIELD32((baseaddr + CSL_ICSS_G_PR1_PDSP0_IRAM_CONTROL),  CSL_ICSS_G_PR1_PDSP0_IRAM_CONTROL_COUNTER_ENABLE, enable);
                    break;
                case PRUICSS_PRU1:
                    baseaddr = hwAttrs->pru1CtrlRegBase;
                    HW_WR_FIELD32((baseaddr + CSL_ICSS_G_PR1_PDSP1_IRAM_CONTROL),  CSL_ICSS_G_PR1_PDSP1_IRAM_CONTROL_COUNTER_ENABLE, enable);
                    break;
                case PRUICSS_RTU_PRU0:
                    baseaddr = hwAttrs->rtu0CtrlRegBase;
                    HW_WR_FIELD32((baseaddr + CSL_ICSS_G_PR1_RTU0_PR1_RTU0_IRAM_CONTROL),  CSL_ICSS_G_PR1_RTU0_PR1_RTU0_IRAM_CONTROL_COUNTER_ENABLE, enable);
                    break;
                case PRUICSS_RTU_PRU1:
                    baseaddr = hwAttrs->rtu1CtrlRegBase;
                    HW_WR_FIELD32((baseaddr + CSL_ICSS_G_PR1_RTU1_PR1_RTU1_IRAM_CONTROL),  CSL_ICSS_G_PR1_RTU1_PR1_RTU1_IRAM_CONTROL_COUNTER_ENABLE, enable);
                    break;
                case PRUICSS_TX_PRU0:
                    baseaddr = hwAttrs->txPru0CtrlRegBase;
                    HW_WR_FIELD32((baseaddr + CSL_ICSS_G_PR1_PDSP_TX0_IRAM_CONTROL),  CSL_ICSS_G_PR1_PDSP_TX0_IRAM_CONTROL_COUNTER_ENABLE, enable);
                    break;
                case PRUICSS_TX_PRU1:
                    baseaddr = hwAttrs->txPru1CtrlRegBase;
                    HW_WR_FIELD32((baseaddr + CSL_ICSS_G_PR1_PDSP_TX1_IRAM_CONTROL),  CSL_ICSS_G_PR1_PDSP_TX1_IRAM_CONTROL_COUNTER_ENABLE, enable);
                    break;
            }
        }
    }
    return retVal;
}

int32_t PRUICSS_controlIepCounter(PRUICSS_Handle handle, uint8_t iepInstance, uint8_t enable)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (iepInstance < 2) && (enable < 2))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

        switch (iepInstance)
        {
            case 0:
                HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG),
                              CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG_CNT_ENABLE, enable);
                break;
            case 1:
                HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG),
                              CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG_CNT_ENABLE, enable);
                break;
        }
    }

    return retVal;
}

int32_t PRUICSS_setIepCounterIncrementValue(PRUICSS_Handle handle, uint8_t iepInstance, uint8_t value)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (iepInstance < 2) && (value < 16))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

        switch (iepInstance)
        {
            case 0:
                HW_WR_FIELD32((hwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG),
                               CSL_ICSS_G_PR1_IEP0_SLV_GLOBAL_CFG_REG_DEFAULT_INC, value);
                break;
            case 1:
                HW_WR_FIELD32((hwAttrs->iep1RegBase + CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG),
                               CSL_ICSS_G_PR1_IEP1_SLV_GLOBAL_CFG_REG_DEFAULT_INC, value);
                break;
        }
    }

    return retVal;
}

int32_t PRUICSS_readEfuse(PRUICSS_Handle handle, uint8_t *data)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if (handle != NULL)
    {
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;
        *data = (HW_RD_REG32(hwAttrs->cfgRegBase + CSL_ICSSCFG_HWDIS)) & 0xF;
        retVal = SystemP_SUCCESS;
    }

    return retVal;
}

int32_t PRUICSS_setIcssCfgMiiMode(PRUICSS_Handle handle, uint8_t miiInstance, uint8_t mode)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && (miiInstance < 2) && (mode < PRUICSS_NUM_ICSS_CFG_MII_MODES))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

        switch (miiInstance)
        {
            case 0:
                HW_WR_FIELD32((hwAttrs->miiGRtCfgRegBase + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG),
                              CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG_MII0_MODE, mode);
                break;
            case 1:
                HW_WR_FIELD32((hwAttrs->miiGRtCfgRegBase + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG),
                              CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG_MII1_MODE, mode);
                break;
        }
    }

    return retVal;
}

int32_t PRUICSS_setIcssCfgTxFifo(PRUICSS_Handle handle, uint8_t fifoInstance, uint8_t enable)
{
    PRUICSS_HwAttrs const   *hwAttrs;
    int32_t                 retVal = SystemP_FAILURE;

    if ((handle != NULL) && ((fifoInstance == PRUICSS_TX_L1_FIFO) || (fifoInstance == PRUICSS_TX_L2_FIFO)) && (enable < 2))
    {
        retVal = SystemP_SUCCESS;
        hwAttrs = (PRUICSS_HwAttrs const *)handle->hwAttrs;

        switch (fifoInstance)
        {
            case PRUICSS_TX_L1_FIFO:
                HW_WR_FIELD32((hwAttrs->miiGRtCfgRegBase + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG),
                              CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG_TX_L1_EN, enable);
                break;
            case PRUICSS_TX_L2_FIFO:
                HW_WR_FIELD32((hwAttrs->miiGRtCfgRegBase + CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG),
                              CSL_ICSS_G_PR1_MII_RT_PR1_MII_RT_G_CFG_REGS_G_ICSS_G_CFG_TX_L2_EN, enable);
                break;
        }
    }

    return retVal;
}

const PRUICSS_HwAttrs *PRUICSS_getAttrs(uint32_t instance)
{
    PRUICSS_HwAttrs const   *attrs = NULL;
    PRUICSS_Config          *config = NULL;

    if(instance < gPruIcssConfigNum)
    {
        config = &gPruIcssConfig[instance];
        attrs = config->hwAttrs;
    }

    return (attrs);
}

static uintptr_t PRUICSS_getCtrlAddr(PRUICSS_HwAttrs const *hwAttrs,
                                     uint8_t pruNum)
{
    uintptr_t baseaddr = 0U;

    if (hwAttrs != NULL)
    {
        switch (pruNum)
        {
            case PRUICSS_PRU0:
                baseaddr = hwAttrs->pru0CtrlRegBase;
                break;
            case PRUICSS_PRU1:
                baseaddr = hwAttrs->pru1CtrlRegBase;
                break;
            case PRUICSS_RTU_PRU0:
                baseaddr = hwAttrs->rtu0CtrlRegBase;
                break;
            case PRUICSS_RTU_PRU1:
                baseaddr = hwAttrs->rtu1CtrlRegBase;
                break;
            case PRUICSS_TX_PRU0:
                baseaddr = hwAttrs->txPru0CtrlRegBase;
                break;
            case PRUICSS_TX_PRU1:
                baseaddr = hwAttrs->txPru1CtrlRegBase;
                break;
            default:
                break;
        }
    }
    return baseaddr;
}

static void PRUICSS_getMemInfo(PRUICSS_HwAttrs const *hwAttrs,
                               uint32_t pruicssMem,
                               PRUICSS_MemInfo *pMemInfo)
{
    switch(pruicssMem)
    {
        case PRUICSS_DATARAM(0U):
            pMemInfo->addr = hwAttrs->pru0DramBase;
            pMemInfo->size = hwAttrs->pru0DramSize;
            break;
        case PRUICSS_DATARAM(1U):
            pMemInfo->addr = hwAttrs->pru1DramBase;
            pMemInfo->size = hwAttrs->pru1DramSize;
            break;
        case PRUICSS_SHARED_RAM:
            pMemInfo->addr = hwAttrs->sharedDramBase;
            pMemInfo->size = hwAttrs->sharedDramSize;
            break;
        case PRUICSS_IRAM_PRU(0U):
            pMemInfo->addr = hwAttrs->pru0IramBase;
            pMemInfo->size = hwAttrs->pru0IramSize;
            break;
        case PRUICSS_IRAM_PRU(1U):
            pMemInfo->addr = hwAttrs->pru1IramBase;
            pMemInfo->size = hwAttrs->pru1IramSize;
            break;
        case PRUICSS_IRAM_RTU_PRU(0U):
            pMemInfo->addr = hwAttrs->rtu0IramBase;
            pMemInfo->size = hwAttrs->rtu0IramSize;
            break;
        case PRUICSS_IRAM_RTU_PRU(1U):
            pMemInfo->addr = hwAttrs->rtu1IramBase;
            pMemInfo->size = hwAttrs->rtu1IramSize;
            break;
        case PRUICSS_IRAM_TX_PRU(0U):
            pMemInfo->addr = hwAttrs->txPru0IramBase;
            pMemInfo->size = hwAttrs->txPru0IramSize;
            break;
         case PRUICSS_IRAM_TX_PRU(1U):
            pMemInfo->addr = hwAttrs->txPru1IramBase;
            pMemInfo->size = hwAttrs->txPru1IramSize;
            break;
        default:
            pMemInfo->addr = 0U;
            pMemInfo->size = 0U;
            break;
    }
}

static void PRUICSS_hwiIntHandler(uintptr_t ptrPpruEvtoutNum)
{
    PRUICSS_IrqFunMap *pruFunMap;

    pruFunMap = (PRUICSS_IrqFunMap*)ptrPpruEvtoutNum;

    if(pruFunMap != NULL)
    {
        pruFunMap->irqHandler(pruFunMap->pruicssHandle);

        if(pruFunMap->waitEnable == 1)
        {
            SemaphoreP_post(&(pruFunMap->semObj));
        }
    }
}

static void PRUICSS_intcSetCmr(uint8_t sysevt,
                               uint8_t channel,
                               uint8_t polarity,
                               uint8_t type,
                               uintptr_t baseaddr)
{
    uintptr_t tempAddr1 = 0U;
    uintptr_t tempAddr2 = 0U;

    tempAddr1 = ((baseaddr)+(CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_CH_MAP_REG0 + (((uint32_t)sysevt) & ~((uint32_t)0x3U))));
    CSL_REG32_WR(tempAddr1, CSL_REG32_RD(tempAddr1) | ((((uint32_t)channel) & ((uint32_t)0xFU)) << ((((uint32_t)sysevt) & ((uint32_t)0x3U)) << 3U)));

    switch ((sysevt >> 5))
    {
    case 0:
        tempAddr1 = (baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_POLARITY_REG0);
        tempAddr2 = (baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_TYPE_REG0);
        CSL_REG32_WR(tempAddr1, CSL_REG32_RD (tempAddr1) & ~(((uint32_t)polarity) << sysevt));
        CSL_REG32_WR(tempAddr2, CSL_REG32_RD (tempAddr2) & ~(((uint32_t)type) << sysevt));
        break;
    case 1:
        tempAddr1 = (baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_POLARITY_REG1);
        tempAddr2 = (baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_TYPE_REG1);
        CSL_REG32_WR(tempAddr1, CSL_REG32_RD (tempAddr1) & ~(((uint32_t)polarity) << (sysevt - 32U)));
        CSL_REG32_WR(tempAddr2, CSL_REG32_RD (tempAddr2) & ~(((uint32_t)type) << (sysevt - 32U)));
        break;
    case 2:
        tempAddr1 = (baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_POLARITY_REG2);
        tempAddr2 = (baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_TYPE_REG2);
        CSL_REG32_WR(tempAddr1, CSL_REG32_RD (tempAddr1) & ~(((uint32_t)polarity) << (sysevt - 64U)));
        CSL_REG32_WR(tempAddr2, CSL_REG32_RD (tempAddr2) & ~(((uint32_t)type) << (sysevt - 64U)));
        break;
    case 3:
        tempAddr1 = (baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_POLARITY_REG3);
        tempAddr2 = (baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_TYPE_REG3);
        CSL_REG32_WR(tempAddr1, CSL_REG32_RD (tempAddr1) & ~(((uint32_t)polarity) << (sysevt - 96U)));
        CSL_REG32_WR(tempAddr2, CSL_REG32_RD (tempAddr2) & ~(((uint32_t)type) << (sysevt - 96U)));
        break;
    case 4:
        tempAddr1 = (baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_POLARITY_REG4);
        tempAddr2 = (baseaddr + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_TYPE_REG4);
        CSL_REG32_WR(tempAddr1, CSL_REG32_RD (tempAddr1) & ~(((uint32_t)polarity) << (sysevt - 128U)));
        CSL_REG32_WR(tempAddr2, CSL_REG32_RD (tempAddr2) & ~(((uint32_t)type) << (sysevt - 128U)));
        break;
    default:
        break;
    }
}

static void PRUICSS_intcSetHmr(uint8_t channel,
                               uint8_t host,
                               uintptr_t baseaddr)
{
    uintptr_t tempAddr1 = 0U;

    tempAddr1 = ((baseaddr) + (CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_HINT_MAP_REG0 + (((uint32_t)channel) & ~((uint32_t)0x3U))));
    CSL_REG32_WR(tempAddr1, CSL_REG32_RD(tempAddr1) | ((((uint32_t)host) & ((uint32_t)0xFU)) << ((((uint32_t)channel) & ((uint32_t)0x3U)) << 3U)));
}

int32_t PRUICSS_loadFirmware(PRUICSS_Handle handle, uint8_t pruIcssCore, const uint32_t pruFirmware[], uint32_t byteLength)
{
    int32_t retVal = SystemP_FAILURE;
    int32_t status;
    uint32_t baseaddr;

    if(handle == NULL || (!(pruIcssCore < PRUICSS_NUM_CORES)))
    {
        return retVal;
    }

    retVal = PRUICSS_disableCore(handle, pruIcssCore);

    if(SystemP_SUCCESS == retVal)
    {
        switch(pruIcssCore)
        {
            case PRUICSS_PRU0:
                baseaddr = PRUICSS_IRAM_PRU(0);
                break;
            case PRUICSS_PRU1:
                baseaddr = PRUICSS_IRAM_PRU(1);
                break;
            case PRUICSS_RTU_PRU0:
                baseaddr = PRUICSS_IRAM_RTU_PRU(0);
                break;
            case PRUICSS_RTU_PRU1:
                baseaddr = PRUICSS_IRAM_RTU_PRU(1);
                break;
            case PRUICSS_TX_PRU0:
                baseaddr = PRUICSS_IRAM_TX_PRU(0);
                break;
            case PRUICSS_TX_PRU1:
                baseaddr = PRUICSS_IRAM_TX_PRU(1);
                break;

        }
        status = PRUICSS_writeMemory(handle, baseaddr, 0, (const uint32_t *) pruFirmware, byteLength);
        if((status * 4) != byteLength)
        {
            retVal = SystemP_FAILURE;
        }
    }

    if(retVal == SystemP_SUCCESS)
    {
        retVal = PRUICSS_resetCore(handle, pruIcssCore);
    }

    if(retVal == SystemP_SUCCESS)
    {
        /* Run firmware */
        retVal = PRUICSS_enableCore(handle, pruIcssCore);
    }

    return retVal;
}