/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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
 *  \file gpmc_v0.c
 *
 *  \brief File containing GPMC Driver APIs implementation for version V1.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <string.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/gpmc.h>
#include <drivers/elm.h>
#include <drivers/gpmc/v0/dma/gpmc_dma.h>


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/* GPMC Module wait time*/
#define GPMC_MODULE_RESET_WAIT_TIME_MAX              (10 * 1000)   /*10ms*/
#define GPMC_WAIT_PIN_STATUS_WAIT_TIME_MAX           (10 * 1000U)  /*1ms*/
#define GPMC_WAIT_PIN_STATUS_WAIT_TIME_MIN           (0U)          /*1ms*/
#define GPMC_ELM_ERR_STATUS_TIMEOUT_MAX              (10 * 1000U)  /*1ms*/
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct
{
    void *openLock;
    /**<  Lock to protect GPMC open*/
    SemaphoreP_Object lockObj;
    /**< Lock object */
} GPMC_DrvObj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Internal functions */
static void GPMC_isr(void *arg);
static uint32_t GPMC_eccBchResultGet(GPMC_Handle handle, uint32_t resIndex, uint32_t sector);
static void GPMC_eccResultSizeSelect(GPMC_Handle handle, uint32_t eccResReg,
                             uint32_t eccSize);

static void GPMC_nandAddressWrite(GPMC_Handle handle, uint32_t address);
static void GPMC_nandCommandWrite(GPMC_Handle handle, uint32_t cmd);
static void GPMC_transferCallback(GPMC_Handle handle, GPMC_Transaction *msg);
static int32_t GPMC_programInstance(GPMC_Config *config);
static int32_t GPMC_programInstancePsram(GPMC_Config *config);
static void GPMC_disableInterupt(uint32_t baseAddr, uint32_t interupt);
static void GPMC_enableInterupt(uint32_t baseAddr, uint32_t interupt);
static void GPMC_interuptStatusClear(uint32_t baseAddr, uint32_t interupt);
static uint32_t  GPMC_interuptStatusGet(uint32_t baseAddr, uint32_t interupt);
static void GPMC_waitPinPolaritySelect(GPMC_Handle handle, uint32_t pin,
                                uint32_t polarity);
static int32_t GPMC_waitPinInteruptStatusReadyWaitTimeout(GPMC_Handle handle,
                                uint32_t timeOut);
static int32_t GPMC_waitPinStatusReadyWaitTimeout(GPMC_Handle handle,
                                uint32_t timeOut);
static uint32_t GPMC_waitPinStatusGet(uint32_t baseAddr, uint32_t pin);
static int32_t GPMC_moduleResetStatusWaitTimeout(GPMC_Config *config,
                                uint32_t timeOut);
static int32_t GPMC_isDmaRestrictedRegion(GPMC_Handle handle, uint32_t addr);
static int32_t GPMC_prefetchPostWriteConfigEnable(GPMC_Handle handle, uint8_t mode,
                                    uint32_t transferCount, uint8_t modeDMA);
static int32_t GPMC_prefetchPostWriteConfigDisable(GPMC_Handle handle);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** \brief Driver object */
static GPMC_DrvObj gGpmcDrvObj =
{
    .openLock      = NULL,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void GPMC_init(void)
{
    int32_t status = SystemP_SUCCESS;
    GPMC_Object *obj;

    for(uint8_t count =0; count < gGpmcConfigNum; count++)
    {
        /* Init object variables */
        obj = gGpmcConfig[count].object;
        DebugP_assert(NULL != obj);
        memset(obj, 0, sizeof(GPMC_Object));
    }

    /* Create the driver lock */
    status = SemaphoreP_constructMutex(&gGpmcDrvObj.lockObj);
    if(SystemP_SUCCESS == status)
    {
        gGpmcDrvObj.openLock = &gGpmcDrvObj.lockObj;
    }

    return;
}

void GPMC_deinit(void)
{
    if(gGpmcDrvObj.openLock != NULL)
    {
        /* Delete Semaphore. */
        SemaphoreP_destruct(&gGpmcDrvObj.lockObj);
        gGpmcDrvObj.openLock = NULL;
    }

    return;
}

int32_t GPMC_configureTimingParametersPsram(GPMC_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t timeConfig = 0;

    if(handle != NULL)
    {
        const GPMC_HwAttrs *hwAttrs = ((GPMC_Config*)handle)->attrs;
        GPMC_Object *object  = ((GPMC_Config*)handle)->object;

        /* CONFIG2 register timing config, no extra delay */
        timeConfig = GPMC_CS_TIMING_CONFIG(GPMC_PSRAM_CS_WR_OFF_TIME,GPMC_PSRAM_CS_RD_OFF_TIME,0,GPMC_PSRAM_CS_ON_TIME);
        CSL_REG32_WR(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG2(object->params.chipSel), timeConfig);

        /* CONFIG3 register timing config, no extra delay */
        timeConfig = GPMC_ADV_TIMING_CONFIG(GPMC_PSRAM_ADV_AADMUX_WR_OFF_TIME,GPMC_PSRAM_ADV_AADMUX_RD_OFF_TIME,GPMC_PSRAM_ADV_WR_OFF_TIME,GPMC_PSRAM_ADV_RD_OFF_TIME,CSL_GPMC_CONFIG3_ADVEXTRADELAY_NOTDELAYED,GPMC_PSRAM_ADV_AADMUX_ON_TIME,GPMC_PSRAM_ADV_ON_TIME);
        CSL_REG32_WR(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG3(object->params.chipSel), timeConfig);

        /* CONFIG4 register timing config, no xtra delay */
        timeConfig = GPMC_WE_OE_TIMING_CONFIG(GPMC_PSRAM_WE_OFF_TIME,CSL_GPMC_CONFIG4_WEEXTRADELAY_NOTDELAYED,GPMC_PSRAM_WE_ON_TIME,GPMC_PSRAM_OE_AADMUX_OFF_TIME,GPMC_PSRAM_OE_OFF_TIME,CSL_GPMC_CONFIG4_OEEXTRADELAY_NOTDELAYED,GPMC_PSRAM_OE_AADMUX_ON_TIME,GPMC_PSRAM_OE_ON_TIME);
        CSL_REG32_WR(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG4(object->params.chipSel), timeConfig);

        /* CONFIG5 register timing config */
        timeConfig = GPMC_RDACCESS_CYCLETIME_TIMING_CONFIG(GPMC_PSRAM_RD_CYCLE_TIME,GPMC_PSRAM_WR_CYCLE_TIME,GPMC_PSRAM_RD_ACCESS_TIME,GPMC_PSRAM_PAGEBURST_ACCESS_TIME);
        CSL_REG32_WR(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG5(object->params.chipSel), timeConfig);

        /* CONFIG6 register timing config */
        CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG6(object->params.chipSel),
                    GPMC_CONFIG6_WRACCESSTIME, hwAttrs->timingParams.wrAcessTime);
        CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG6(object->params.chipSel),
                    GPMC_CONFIG6_WRDATAONADMUXBUS, hwAttrs->timingParams.wrDataOnMuxBusTime);


        timeConfig = GPMC_CYCLE2CYCLE_BUSTURNAROUND_TIMING_CONFIG(hwAttrs->timingParams.cycle2CycleDelay,
                                                                    hwAttrs->timingParams.cycleDelaySameChipSel,
                                                                    hwAttrs->timingParams.cycleDelayDiffChipSel,
                                                                    hwAttrs->timingParams.busTurnAroundTime);

        CSL_REG32_WR(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG6(object->params.chipSel), \
        (CSL_REG32_RD(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG6(object->params.chipSel)) | timeConfig));

        CSL_REG32_WR(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG7(object->params.chipSel), 0xf28);
        CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG7(object->params.chipSel), \
                         GPMC_CONFIG7_CSVALID, CSL_GPMC_CONFIG7_CSVALID_CSENABLED);
    }
    else
    {
        status = SystemP_FAILURE;
    }
    return status;
}

int32_t GPMC_configureTimingParameters(GPMC_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t timeConfig = 0;

    if(handle != NULL)
    {
        const GPMC_HwAttrs *hwAttrs = ((GPMC_Config*)handle)->attrs;
        GPMC_Object *object  = ((GPMC_Config*)handle)->object;


        /* CONFIG2 reister timing config, no extra delay */
        timeConfig = GPMC_CS_TIMING_CONFIG(hwAttrs->timingParams.csWrOffTime,
                                            hwAttrs->timingParams.csRdOffTime,
                                            hwAttrs->csExDelay,
                                            hwAttrs->timingParams.csOnTime);

        CSL_REG32_WR(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG2(object->params.chipSel), timeConfig);


        /* CONFIG3 reister timing config, no extra delay */
        timeConfig = GPMC_ADV_TIMING_CONFIG(hwAttrs->timingParams.advAadMuxWrOffTime,
                                            hwAttrs->timingParams.advAadMuxRdOffTime,
                                            hwAttrs->timingParams.advWrOffTime,
                                            hwAttrs->timingParams.advRdOffTime,
                                            CSL_GPMC_CONFIG3_ADVEXTRADELAY_NOTDELAYED,
                                            hwAttrs->timingParams.advAadMuxOnTime,
                                            hwAttrs->timingParams.advOnTime);
        CSL_REG32_WR(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG3(object->params.chipSel), timeConfig);

       /* CONFIG4 reister timing config, no xtra delay */
        timeConfig = GPMC_WE_OE_TIMING_CONFIG(hwAttrs->timingParams.weOffTime,
                                                CSL_GPMC_CONFIG4_WEEXTRADELAY_NOTDELAYED,
                                                hwAttrs->timingParams.weOnTtime,
                                                hwAttrs->timingParams.oeAadMuxOffTime,
                                                hwAttrs->timingParams.oeOffTime,
                                                CSL_GPMC_CONFIG4_OEEXTRADELAY_NOTDELAYED,
                                                hwAttrs->timingParams.oeAadMuxOnTime,
                                                hwAttrs->timingParams.oeOnTime);

        CSL_REG32_WR(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG4(object->params.chipSel), timeConfig);

        /* CONFIG5 reister timing config */
        timeConfig = GPMC_RDACCESS_CYCLETIME_TIMING_CONFIG(hwAttrs->timingParams.rdCycleTime,
                                                            hwAttrs->timingParams.wrCycleTime,
                                                            hwAttrs->timingParams.rdAccessTime,
                                                            hwAttrs->timingParams.pageBurstAccess);
        CSL_REG32_WR(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG5(object->params.chipSel), timeConfig);


        /* CONFIG6 register timing config */
        CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG6(object->params.chipSel),
                    GPMC_CONFIG6_WRACCESSTIME, hwAttrs->timingParams.wrAcessTime);
        CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG6(object->params.chipSel),
                    GPMC_CONFIG6_WRDATAONADMUXBUS, hwAttrs->timingParams.wrDataOnMuxBusTime);


        timeConfig = GPMC_CYCLE2CYCLE_BUSTURNAROUND_TIMING_CONFIG(hwAttrs->timingParams.cycle2CycleDelay,
                                                                    hwAttrs->timingParams.cycleDelaySameChipSel,
                                                                    hwAttrs->timingParams.cycleDelayDiffChipSel,
                                                                    hwAttrs->timingParams.busTurnAroundTime);

        CSL_REG32_WR(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG6(object->params.chipSel), \
        (CSL_REG32_RD(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG6(object->params.chipSel)) | timeConfig));

    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t GPMC_setDeviceType(GPMC_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    /* Input parameter validation. */
    if(handle != NULL)
    {
        GPMC_Config *config = (GPMC_Config*)handle;
        const GPMC_HwAttrs *attrs = config->attrs;
        GPMC_Object *object = config->object;
        /* Set Device type interfaced with GPMC. */
        CSL_REG32_FINS(attrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                  GPMC_CONFIG1_DEVICETYPE, object->params.devType);

    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t GPMC_setDeviceSize(GPMC_Handle handle)
{
    int32_t status = SystemP_SUCCESS;

    /* Input parameter validation. */
    if(handle != NULL)
    {
        GPMC_Config *config = (GPMC_Config*)handle;
        const GPMC_HwAttrs *attrs = config->attrs;
        GPMC_Object *object = config->object;
        /* Set device width interfaced with GPMC. */
        CSL_REG32_FINS(attrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                  GPMC_CONFIG1_DEVICESIZE, object->params.devSize);

    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

GPMC_Handle GPMC_open(uint32_t index, const GPMC_Params *prms)
{
    int32_t status = SystemP_SUCCESS;
    GPMC_Handle         handle = NULL;
    GPMC_Config         *config = NULL;
    GPMC_Object         *object = NULL;
    const GPMC_HwAttrs  *hwAttrs = NULL;
    HwiP_Params         hwiParams;


    /* Check for valid index */
    if(index >= gGpmcConfigNum)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        config = &gGpmcConfig[index];
    }

    /* Protect this region from a concurrent GPMC_Open */
    DebugP_assert(NULL != gGpmcDrvObj.openLock);
    SemaphoreP_pend(&gGpmcDrvObj.lockObj, SystemP_WAIT_FOREVER);

    if(status == SystemP_SUCCESS)
    {
        object = config->object;
        DebugP_assert(object != NULL);
        DebugP_assert(config->attrs != NULL);
        hwAttrs = config->attrs;
        if(object->isOpen == TRUE)
        {
            status = SystemP_FAILURE;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        object->handle = (GPMC_Handle)config;

        if(prms != NULL)
        {
            memcpy((void*)&object->params, (void*)prms, sizeof(GPMC_Params));
        }
        else
        {
            /* Init with default if NULL is passed */
            GPMC_Params_init(&object->params);
        }

        /* If DMA is enabled, program UDMA block copy channel */
        if(object->params.dmaEnable == TRUE)
        {
            object->gpmcDmaHandle = GPMC_dmaOpen(object->params.gpmcDmaChIndex);
        }
        else
        {
            object->gpmcDmaHandle = NULL;
        }

        if(object->params.transferMode == GPMC_TRANSFER_MODE_BLOCKING)
        {
            if(object->params.intrEnable == TRUE)
            {
                object->operMode = GPMC_OPERATING_MODE_BLOCKING;
            }
            else
            {
                object->operMode = GPMC_OPERATING_MODE_POLLING;
            }
        }
        else
        {
            object->operMode = GPMC_OPERATING_MODE_CALLBACK;
        }

        if(object->params.intrEnable == TRUE)
        {
            HwiP_Params_init(&hwiParams);
            hwiParams.intNum = hwAttrs->intrNum;
            hwiParams.callback = GPMC_isr;
            hwiParams.args     = (void*)config;
            hwiParams.priority = hwAttrs->intrPriority;
            status += HwiP_construct(&object->hwi, &hwiParams);

        }

    }

    if(status == SystemP_SUCCESS)
    {
        if(object->params.memDevice == GPMC_MEM_TYPE_NAND)
        {
            status += GPMC_programInstance(config);
        }
        else if(object->params.memDevice == GPMC_MEM_TYPE_PSRAM)
        {
            status += GPMC_programInstancePsram(config);
        }

        status += SemaphoreP_constructMutex(&object->mutex);

        if (object->operMode == GPMC_OPERATING_MODE_BLOCKING)
        {
            /*
            * Semaphore to cause the waiting task to block for the GPMC to finish.
            */
            status += SemaphoreP_constructBinary(&object->transferComplete, 0);

            /* Store internal callback function */
            object->params.transferCallBckFunc = &GPMC_transferCallback;
        }

        if(object->operMode == GPMC_OPERATING_MODE_CALLBACK)
        {
            /* Currently not supported. */
        }

    }

    if(status == SystemP_SUCCESS)
    {
        object->isOpen = 1;
        handle = (GPMC_Handle)config;
    }

    SemaphoreP_post(&gGpmcDrvObj.lockObj);

    /* Free up resources in case of error. */
    if(SystemP_SUCCESS != status)
    {
        if(NULL != config)
        {
            GPMC_close((GPMC_Handle) config);
        }
    }

    return(handle);
}

void GPMC_close(GPMC_Handle handle)
{
    /* Input parameter validation */
    if (handle != NULL)
    {
        GPMC_Object        *object = NULL;
        const GPMC_HwAttrs *hwAttrs = NULL;
        /* Get the pointer to the object and hwAttrs */
        object = ((GPMC_Config*)handle)->object;
        hwAttrs = ((GPMC_Config*)handle)->attrs;

        /* Disable all interupts associated to GPMC. */
        GPMC_disableInterupt(hwAttrs->gpmcBaseAddr,GPMC_FIFOEVENT_INT);
        GPMC_disableInterupt(hwAttrs->gpmcBaseAddr,GPMC_TERMINALCOUNT_INT);
        GPMC_disableInterupt(hwAttrs->gpmcBaseAddr,GPMC_WAIT0EDGEDETECTION_INT);
        GPMC_disableInterupt(hwAttrs->gpmcBaseAddr,GPMC_WAIT1EDGEDETECTION_INT);

        /* If DMA block copy channel was opened, close it. */
        if(object->params.dmaEnable == TRUE)
        {
            GPMC_dmaClose(object->gpmcDmaHandle);
        }

        /* Destruct the Hwi. */
        if(object->operMode != GPMC_OPERATING_MODE_POLLING)
        {
            HwiP_destruct(&object->hwi);
        }

        /* Destruct the instance lock. */
        SemaphoreP_destruct(&object->mutex);

        /* Destruct the transfer completion lock. */
        if(object->operMode == GPMC_OPERATING_MODE_BLOCKING)
        {
            SemaphoreP_destruct(&object->transferComplete);
        }

        /* Open flag is set false. */
        object->isOpen = 0;
    }

    return;
}

void GPMC_transactionInit(GPMC_Transaction *trans)
{
    trans->Buf = NULL;
    trans->count = 0;
    trans->status = GPMC_TRANSFER_STARTED;
    trans->transType = GPMC_TRANSACTION_TYPE_READ;
    trans->arg = NULL;
    trans->transferTimeout = SystemP_WAIT_FOREVER;
}

int32_t GPMC_configurePrefetchPostWriteEngine(GPMC_Handle handle)
{
    int32_t status = SystemP_SUCCESS;

    /* Input parameter validation */
    if(handle != NULL)
    {
        const GPMC_HwAttrs *attrs = ((GPMC_Config*)handle)->attrs;
        GPMC_Object *object = ((GPMC_Config*)handle)->object;

        /*Disable and stop the prefetch engine*/
        CSL_REG32_FINS(attrs->gpmcBaseAddr + CSL_GPMC_PREFETCH_CONTROL, GPMC_PREFETCH_CONTROL_STARTENGINE, \
        CSL_GPMC_PREFETCH_CONTROL_STARTENGINE_STOP);
        CSL_REG32_FINS(attrs->gpmcBaseAddr + CSL_GPMC_PREFETCH_CONFIG1, GPMC_PREFETCH_CONFIG1_ENABLEENGINE, \
        CSL_GPMC_PREFETCH_CONFIG1_ENABLEENGINE_PPDISABLED);

        /*Select the chip select associated with the external device*/
        CSL_REG32_FINS(attrs->gpmcBaseAddr + CSL_GPMC_PREFETCH_CONFIG1, GPMC_PREFETCH_CONFIG1_ENGINECSSELECTOR, \
        object->params.chipSel);

        /*Set FIFOTHRESHOLD value. */
        CSL_REG32_FINS(attrs->gpmcBaseAddr + CSL_GPMC_PREFETCH_CONFIG1, GPMC_PREFETCH_CONFIG1_FIFOTHRESHOLD, \
        CSL_GPMC_PREFETCH_CONFIG1_FIFOTHRESHOLD_RESETVAL);

    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t GPMC_nandReadData(GPMC_Handle handle, GPMC_Transaction *trans)
{
    int32_t status = SystemP_SUCCESS;

    /* Input parameter validation */
    if(handle != NULL && trans != NULL)
    {
        GPMC_Object *object = ((GPMC_Config*)handle)->object;
        const GPMC_HwAttrs *attrs = ((GPMC_Config*)handle)->attrs;
        uint32_t byteCount = trans->count;
        uint32_t threshold = 0;

        if(object->operMode == GPMC_OPERATING_MODE_POLLING)
        {
            if(trans->transType == GPMC_TRANSACTION_TYPE_READ)
            {
                /* Check for read data length more thand DMA copy lower limit (512 bytes -> 1 sector).
                 * Check if destination address is not a restricted dma region.
                 */
                if(object->params.dmaEnable && (trans->count >= GPMC_DMA_COPY_LOWER_LIMIT)
                    && (GPMC_isDmaRestrictedRegion(handle, (uint32_t)trans->Buf) == FALSE))
                {
                    /* Enable prefetch read engine.  */
                    status += GPMC_prefetchPostWriteConfigEnable(handle, GPMC_PREFETCH_ACCESSMODE_READ, byteCount, TRUE);

                    if(status == SystemP_SUCCESS)
                    {
                        /* Perform DMA copy. */
                        GPMC_dmaCopy(object->gpmcDmaHandle, trans->Buf, (void*)attrs->chipSelBaseAddr, trans->count, TRUE);
                    }
                    /* Disable prefetch read engine. */
                    status += GPMC_prefetchPostWriteConfigDisable(handle);
                }
                else
                {
                    /* Perform CPU read with prefetch read engine. */

                    /* Enable prefetch read engine. */
                    status += GPMC_prefetchPostWriteConfigEnable(handle, GPMC_PREFETCH_ACCESSMODE_READ, byteCount, FALSE);

                    if(status == SystemP_SUCCESS)
                    {
                        uint32_t *ptr = (uint32_t *)trans->Buf;

                        while(byteCount)
                        {
                            /* Get GPMC FIFO counter value. */
                            threshold = CSL_REG32_FEXT(attrs->gpmcBaseAddr + CSL_GPMC_PREFETCH_STATUS, GPMC_PREFETCH_STATUS_FIFOPOINTER);

                            for(uint32_t i =0; i< threshold/4;i++)
                            {
                                *ptr++ = *(volatile uint32_t*)attrs->chipSelBaseAddr;
                                byteCount -=4;
                            }
                        }


                    }

                    /* Disable prefetch read engine. */
                    status += GPMC_prefetchPostWriteConfigDisable(handle);
                }
            }
            else if(trans->transType == GPMC_TRANSACTION_TYPE_READ_CMDREG)
            {
                /* Read data from GPMC command register. */
                uint32_t *bufPtr = (uint32_t*)trans->Buf;

                *bufPtr = CSL_REG32_RD(attrs->gpmcBaseAddr + CSL_GPMC_NAND_DATA(object->params.chipSel));
            }
            else
            {
                status = SystemP_FAILURE;
            }
        }
        else
        {
            /* Interupt mode not supported. */
            status = SystemP_FAILURE;
        }

    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t GPMC_nandWriteData(GPMC_Handle handle, GPMC_Transaction *trans)
{
    int32_t status = SystemP_SUCCESS;

    /* Input parameter validation */
    if(handle != NULL && trans != NULL)
    {
        GPMC_Object *object = ((GPMC_Config*)handle)->object;
        const GPMC_HwAttrs *attrs = ((GPMC_Config*)handle)->attrs;
        uint32_t byteCount = trans->count;
        uint32_t threshold = 0;
        uint32_t remainBytes = 0;

        if(object->operMode == GPMC_OPERATING_MODE_POLLING)
        {
            /* Check for write buffer empty status.*/
            while(CSL_REG32_FEXT(attrs->gpmcBaseAddr + CSL_GPMC_STATUS,
                  GPMC_STATUS_EMPTYWRITEBUFFERSTATUS) == CSL_GPMC_STATUS_EMPTYWRITEBUFFERSTATUS_B0);

            if(trans->transType == GPMC_TRANSACTION_TYPE_WRITE)
            {
                /* Perform write using post write engine with CPU. */
                uint32_t *bufPtr = (uint32_t*)trans->Buf;
                /* Enable post write engine. */
                status += GPMC_prefetchPostWriteConfigEnable(handle, GPMC_PREFETCH_ACCESSMODE_WRITE, byteCount, FALSE);
                /* Enable FIFO event interupt. */
                GPMC_enableInterupt(attrs->gpmcBaseAddr, GPMC_FIFOEVENT_INT);

                if(status == SystemP_SUCCESS)
                {
                    while(byteCount)
                    {
                        /* Wait until FIFO is empty or full 64 bytes FIFO is available to fill. */
                        while (GPMC_interuptStatusGet(attrs->gpmcBaseAddr, GPMC_FIFOEVENT_STATUS) == 0);

                        /* Get FIFO pointer and remaining bytes to write value.*/
                        threshold = CSL_REG32_FEXT(attrs->gpmcBaseAddr + CSL_GPMC_PREFETCH_STATUS, GPMC_PREFETCH_STATUS_FIFOPOINTER);
                        remainBytes = CSL_REG32_FEXT(attrs->gpmcBaseAddr + CSL_GPMC_PREFETCH_STATUS, GPMC_PREFETCH_STATUS_COUNTVALUE);

                        if(remainBytes < threshold)
                        {
                            threshold = remainBytes;
                        }
                        for(uint32_t i =0; i< threshold/4;i++)
                        {
                            *(volatile uint32_t*)attrs->chipSelBaseAddr = *bufPtr++;
                            byteCount -=4;
                        }

                        /* Clear FIFO event interupt status . */
                        GPMC_interuptStatusClear(attrs->gpmcBaseAddr, GPMC_FIFOEVENT_STATUS);
                    }

                }

                /* Disable FIFO event interupt and post write engine. */
                GPMC_disableInterupt(attrs->gpmcBaseAddr, GPMC_FIFOEVENT_INT);
                status += GPMC_prefetchPostWriteConfigDisable(handle);

            }
            else if(trans->transType == GPMC_TRANSACTION_TYPE_WRITE_CMDREG)
            {
                /* Write data using GPMC command register. */
                uint32_t *bufPtr = (uint32_t*)trans->Buf;

                CSL_REG32_WR(attrs->gpmcBaseAddr + CSL_GPMC_NAND_DATA(object->params.chipSel), *bufPtr);
            }
            else
            {
                status = SystemP_FAILURE;
            }
        }
        else
        {
            /* Interupt mode not supported. */
            status = SystemP_FAILURE;
        }

    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

void GPMC_writeNandCommandParamsInit(GPMC_nandCmdParams *cmdParams)
{
    cmdParams->cmdCycle1 = GPMC_CMD_INVALID;
    cmdParams->cmdCycle2 = GPMC_CMD_INVALID;
    cmdParams->colAddress = GPMC_CMD_INVALID;
    cmdParams->rowAddress = GPMC_CMD_INVALID;
    cmdParams->numColAddrCycles = GPMC_CMD_INVALID;
    cmdParams->numRowAddrCycles = GPMC_CMD_INVALID;
    cmdParams->waitTimeout = 0;
    cmdParams->checkReadypin = TRUE;
}

int32_t GPMC_writeNandCommand(GPMC_Handle handle, GPMC_nandCmdParams *cmdParams)
{
    int32_t status = SystemP_SUCCESS;

    /* Input parameter validation */
    if(handle != NULL &&  cmdParams != NULL)
    {
        uint32_t waitPinInterupt = 0;
        uint32_t colAddress = 0;
        uint32_t rowAddress = 0;

        const GPMC_HwAttrs *hwAttrs = ((GPMC_Config*)handle)->attrs;

        if(hwAttrs->waitPinNum == CSL_GPMC_CONFIG1_WAITPINSELECT_W0)
        {
            waitPinInterupt = GPMC_WAIT0EDGEDETECTION_STATUS;
        }
        else if(hwAttrs->waitPinNum == CSL_GPMC_CONFIG1_WAITPINSELECT_W1)
        {
            waitPinInterupt = GPMC_WAIT1EDGEDETECTION_STATUS;
        }

        /* Clear WAIT PIN interupt status. */
        GPMC_interuptStatusClear(hwAttrs->gpmcBaseAddr, waitPinInterupt);

        /* Write valid nand command cycle 1. */
        if(cmdParams->cmdCycle1 != GPMC_CMD_INVALID)
        {
            GPMC_nandCommandWrite(handle,cmdParams->cmdCycle1);
        }

        /* Write valid nand column address. */
        if(cmdParams->colAddress != GPMC_CMD_INVALID)
        {
            colAddress = cmdParams->colAddress;
            for (uint8_t count = 0; count < cmdParams->numColAddrCycles; count++)
            {
                GPMC_nandAddressWrite(handle, (colAddress & 0xFF));
                colAddress = colAddress >> 0x8;
            }
        }

        /* Write valid nand row address. */
        if(cmdParams->rowAddress != GPMC_CMD_INVALID)
        {
            rowAddress = cmdParams->rowAddress;
            for (uint8_t count = 0; count < cmdParams->numRowAddrCycles; count++)
            {
                GPMC_nandAddressWrite(handle, (rowAddress & 0xFF));
                rowAddress = rowAddress >> 0x8;
            }
        }

        /* Write valid nand command cycle 2. */
        if(cmdParams->cmdCycle2 != GPMC_CMD_INVALID)
        {
            GPMC_nandCommandWrite(handle,cmdParams->cmdCycle2);
        }

        /* Check WAIT PIN (mapped to R/B signal of nand flash) interupt status with
         * or without timeout.
         */
        if(cmdParams->checkReadypin != GPMC_CMD_INVALID)
        {
            if(!cmdParams->checkReadypin)
            {
                status += GPMC_waitPinStatusReadyWaitTimeout(handle, cmdParams->waitTimeout);
            }
            else
            {
                status += GPMC_waitPinInteruptStatusReadyWaitTimeout(handle,cmdParams->waitTimeout);
                status += GPMC_waitPinStatusReadyWaitTimeout(handle, GPMC_WAIT_PIN_STATUS_WAIT_TIME_MIN);
            }
        }

    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;

}

int32_t GPMC_eccValueSizeSet(GPMC_Handle handle, uint32_t eccSize,
                       uint32_t eccSizeVal)
{
    int32_t status = SystemP_SUCCESS;

    /* Input parameter validation. */
    if(handle != NULL)
    {
        /* Set ECC used and unused bytes size in nibbles. */
        const GPMC_HwAttrs *hwAttrs = ((GPMC_Config*)handle)->attrs;

        if (eccSize == GPMC_ECC_SIZE_0)
        {
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_ECC_SIZE_CONFIG,
                          GPMC_ECC_SIZE_CONFIG_ECCSIZE0,
                          eccSizeVal);
        }
        else if (eccSize == GPMC_ECC_SIZE_1)
        {
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_ECC_SIZE_CONFIG,
                          GPMC_ECC_SIZE_CONFIG_ECCSIZE1,
                          eccSizeVal);
        }
        else
        {
            /*
            * Do nothing. Error will be generated by the hardware
            */
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;

}

int32_t GPMC_eccBchConfigureElm(GPMC_Handle handle, uint8_t numSectors)
{
    int32_t status = SystemP_SUCCESS;

    /* Input parameter validation. */
    if(handle != NULL)
    {
        const GPMC_HwAttrs *hwAttrs = ((GPMC_Config*)handle)->attrs;
        /* Configure ELM module.*/
        ELM_moduleReset(hwAttrs->elmBaseAddr);

        /* Set max bytes per sector for ECC corrrection. */
        ELM_setECCSize(hwAttrs->elmBaseAddr, CSL_ELM_LOCATION_CONFIG_ECC_SIZE_MAX);

        /* Set error correction level. */
        ELM_errorCorrectionLevelSet(hwAttrs->elmBaseAddr, ELM_ECC_BCH_LEVEL_8BITS);

        /* Set all sectors for ELM in CONTINUOUS mode*/
        for(uint8_t count = 0; count < numSectors; count++)
        {
            ELM_interuptConfig(hwAttrs->elmBaseAddr, count, ELM_INT_ENALBLE);
            ELM_setSectorMode(hwAttrs->elmBaseAddr,ELM_MODE_CONTINUOUS,count);
        }

    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t GPMC_eccEngineBCHConfig (GPMC_Handle handle , uint32_t eccSteps)
{
    int32_t status = SystemP_SUCCESS;

    /* Input parameter validation. */
    if(handle != NULL)
    {
        const GPMC_HwAttrs *hwAttrs = ((GPMC_Config*)handle)->attrs;
        GPMC_Object *object = ((GPMC_Config*)handle)->object;

        /* Select ECC result register */
        CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_ECC_CONTROL, GPMC_ECC_CONTROL_ECCPOINTER, GPMC_ECCPOINTER_RESULT_1);

        /* Configure chip select for ECC engine. */
        CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_ECC_CONFIG, GPMC_ECC_CONFIG_ECCCS, object->params.chipSel);

        /* Set number of sectors to process with the BCH algorithm. */
        CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_ECC_CONFIG, GPMC_ECC_CONFIG_ECCTOPSECTOR, eccSteps);

        /* Set error correction capability used for BCH. */
        CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_ECC_CONFIG, GPMC_ECC_CONFIG_ECC16B, CSL_GPMC_ECC_CONFIG_ECC16B_EIGHTCOL);

        /* Spare area organization definition for the BCH algorithm. */
        CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_ECC_CONFIG, GPMC_ECC_CONFIG_ECCWRAPMODE, GPMC_ECC_WRAP_MODE1);

        /* Set error correction level. 8 bit or 16 bit. */
        CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_ECC_CONFIG, GPMC_ECC_CONFIG_ECCBCHTSEL, GPMC_ECC_BCH_ERRCORRCAP_UPTO_8BITS);

        /* Set ECC algo for ECC engine. */
        CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_ECC_CONFIG, GPMC_ECC_CONFIG_ECCALGORITHM, GPMC_ECC_ALGORITHM_BCH);

        /* Set ECC size for ECC result register. */
        GPMC_eccResultSizeSelect(handle, GPMC_ECC_RESULT_1, CSL_GPMC_ECC_SIZE_CONFIG_ECC1RESULTSIZE_SIZE0SEL);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t GPMC_eccEngineEnable(GPMC_Handle handle)
{
    int32_t status = SystemP_SUCCESS;

    /* Input parameter validation. */
    if(handle != NULL)
    {
        const GPMC_HwAttrs *hwAttrs = ((GPMC_Config*)handle)->attrs;
        /* Enable ECC engine. */
        CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_ECC_CONTROL, GPMC_ECC_CONTROL_ECCPOINTER,
                        GPMC_ECCPOINTER_RESULT_1);
        CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_ECC_CONFIG, GPMC_ECC_CONFIG_ECCENABLE,
                  CSL_GPMC_ECC_CONFIG_ECCENABLE_ECCENABLED);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

void GPMC_eccResultRegisterClear(GPMC_Handle handle)
{
    /* Input parameter validation. */
    if(handle != NULL)
    {
        const GPMC_HwAttrs *attrs = ((GPMC_Config*)handle)->attrs;
        /* Clear all the ECC result registers. */
        CSL_REG32_FINS(attrs->gpmcBaseAddr + CSL_GPMC_ECC_CONTROL, GPMC_ECC_CONTROL_ECCCLEAR,
                    CSL_GPMC_ECC_CONTROL_ECCCLEAR_MAX);
    }
}

GPMC_Handle GPMC_getHandle(uint32_t driverInstanceIndex)
{
    GPMC_Handle         handle = NULL;
    /* Check index */
    if(driverInstanceIndex < gGpmcConfigNum)
    {
        GPMC_Object *obj;
        obj = gGpmcConfig[driverInstanceIndex].object;

        if(obj && (TRUE == obj->isOpen))
        {
            /* valid handle */
            handle = obj->handle;
        }
    }
    return handle;
}

int32_t GPMC_eccGetBchSyndromePolynomial(GPMC_Handle handle, uint32_t sector, uint32_t *bchData)
{
        int32_t status = SystemP_SUCCESS;

    /* Input parameter validation. */
    if(handle != NULL)
    {
        bchData[0] = GPMC_eccBchResultGet(handle, GPMC_BCH_RESULT0, sector);
        bchData[1] = GPMC_eccBchResultGet(handle, GPMC_BCH_RESULT1, sector);
        bchData[2] = GPMC_eccBchResultGet(handle, GPMC_BCH_RESULT2, sector);
        bchData[3] = GPMC_eccBchResultGet(handle, GPMC_BCH_RESULT3, sector);

    }
    else
    {
        status =  SystemP_FAILURE;
    }

    return status;
}

int32_t GPMC_eccBchFillSyndromeValue(GPMC_Handle handle, uint32_t sector, uint32_t *bchData)
{
    int32_t status = SystemP_SUCCESS;

    /* Input parameter validation. */
    if(handle != NULL)
    {
        const GPMC_HwAttrs *attrs = ((GPMC_Config*)handle)->attrs;

        /* Fill BCH syndrome polynomial to ELM module per sector. */
        ELM_setSyndromeFragment(attrs->elmBaseAddr, ELM_SYNDROME_FRGMT_0, bchData[0], sector);
        ELM_setSyndromeFragment(attrs->elmBaseAddr, ELM_SYNDROME_FRGMT_1, bchData[1], sector);
        ELM_setSyndromeFragment(attrs->elmBaseAddr, ELM_SYNDROME_FRGMT_2, bchData[2], sector);
        ELM_setSyndromeFragment(attrs->elmBaseAddr, ELM_SYNDROME_FRGMT_3, bchData[3], sector);

    }
    else
    {
        status =  SystemP_FAILURE;
    }

    return status;
}

int32_t GPMC_eccBchStartErrorProcessing(GPMC_Handle handle, uint8_t sector)
{
    int32_t status = SystemP_SUCCESS;

    /* Input parameter validation. */
    if(handle != NULL)
    {
        const GPMC_HwAttrs *attrs = ((GPMC_Config*)handle)->attrs;

        /* Start ELM error processing. */
        ELM_errorLocationProcessingStart(attrs->elmBaseAddr, sector);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t GPMC_eccBchCheckErrorProcessingStatus(GPMC_Handle handle, uint32_t sector)
{
    int32_t status = SystemP_SUCCESS;
    uint64_t curTime;

    /* Input parameter validation. */
    if(handle != NULL)
    {
        const GPMC_HwAttrs *attrs = ((GPMC_Config*)handle)->attrs;

        curTime = ClockP_getTimeUsec();

        /* Check ELM error processing status with timeout. */
        while((ELM_interuptStatusGet(attrs->elmBaseAddr, sector) == ELM_BIT_SET_LOW) &&
        ((ClockP_getTimeUsec() - curTime) < GPMC_ELM_ERR_STATUS_TIMEOUT_MAX))
        {
            /* Do nothing. */
        }

        if(ELM_interuptStatusGet(attrs->elmBaseAddr, sector) == ELM_BIT_SET_LOW)
        {
            status = SystemP_FAILURE;
        }
        else
        {
            ELM_interuptStatusClear(attrs->elmBaseAddr, sector);
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t GPMC_eccBchSectorGetError(GPMC_Handle handle, uint32_t sector, uint32_t *errCount, uint32_t *errLoc)
{
    int32_t status = SystemP_SUCCESS;

    /* Input parameter validation. */
    if(handle != NULL)
    {
        /* Get number of errors located by ELM per sector. */
        const GPMC_HwAttrs *attrs = ((GPMC_Config*)handle)->attrs;

        if(status == SystemP_SUCCESS)
        {
            if (ELM_errorLocationProcessingStatusGet(attrs->elmBaseAddr, sector) > 0U)
            {
                uint32_t count;

                *errCount = ELM_getNumError(attrs->elmBaseAddr, sector);
                for (count = 0; count < *errCount; count++)
                {
                    errLoc[count] = ELM_errorLocationBitAddrGet(attrs->elmBaseAddr, count, sector);
                }
            }
            else
            {
                status = SystemP_FAILURE;
            }
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t GPMC_eccCalculateBchSyndromePolynomial(GPMC_Handle handle, uint8_t *pEccdata, uint32_t sector)
{
    int32_t status = SystemP_SUCCESS;

    /* Input parameter validation. */
    if(handle != NULL)
    {
        uint32_t eccRes;
        /* Get BCH syndrome polynomial per sector. */
        eccRes = GPMC_eccBchResultGet(handle, GPMC_BCH_RESULT3, sector);
        pEccdata[0] = (eccRes & 0xFF);

        eccRes = GPMC_eccBchResultGet(handle, GPMC_BCH_RESULT2, sector);
        pEccdata[1] = ((eccRes >> 24) & 0xFF);
        pEccdata[2] = ((eccRes >> 16) & 0xFF);
        pEccdata[3] = ((eccRes >> 8) & 0xFF);
        pEccdata[4] = (eccRes & 0xFF);

        eccRes = GPMC_eccBchResultGet(handle, GPMC_BCH_RESULT1, sector);
        pEccdata[5] = ((eccRes >> 24) & 0xFF);
        pEccdata[6] = ((eccRes >> 16) & 0xFF);
        pEccdata[7] = ((eccRes >> 8) & 0xFF);
        pEccdata[8] = (eccRes & 0xFF);

        eccRes = GPMC_eccBchResultGet(handle, GPMC_BCH_RESULT0, sector);
        pEccdata[9] = ((eccRes >> 24) & 0xFF);
        pEccdata[10] = ((eccRes >> 16) & 0xFF);
        pEccdata[11] = ((eccRes >> 8) & 0xFF);
        pEccdata[12] = (eccRes & 0xFF);

    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t GPMC_disableFlashWriteProtect(GPMC_Handle handle)
{
    int32_t status = SystemP_SUCCESS;

    /* Input parameter validation. */
    if(handle != NULL)
    {
        const GPMC_HwAttrs *attrs = ((GPMC_Config*)handle)->attrs;
        /* Disable write protect. */
        CSL_REG32_FINS(attrs->gpmcBaseAddr + CSL_GPMC_CONFIG, GPMC_CONFIG_WRITEPROTECT, \
                        CSL_GPMC_CONFIG_WRITEPROTECT_WPHIGH);

    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t GPMC_enableFlashWriteProtect(GPMC_Handle handle)
{
    int32_t status = SystemP_SUCCESS;

    /* Input parameter validation. */
    if(handle != NULL)
    {
        const GPMC_HwAttrs *attrs = ((GPMC_Config*)handle)->attrs;

        /* Enable Write protect. */
        CSL_REG32_FINS(attrs->gpmcBaseAddr + CSL_GPMC_CONFIG, GPMC_CONFIG_WRITEPROTECT, \
                        CSL_GPMC_CONFIG_WRITEPROTECT_WPHIGH);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

uint32_t GPMC_getInputClk(GPMC_Handle handle)
{
    uint32_t retVal = 0U;

    /* Input parameter validation. */
    if(handle != NULL)
    {
        /* Get GPMC interface clock. */
        const GPMC_HwAttrs* attrs = ((GPMC_Config *)handle)->attrs;
        retVal = attrs->inputClkFreq;
    }
    return retVal;
}

/* ========================================================================== */
/*                     Internal function definitions                          */
/* ========================================================================== */

static void GPMC_isr(void *arg)
{
    /* Currently interupt mode not supported.*/
}

static int32_t GPMC_moduleResetStatusWaitTimeout(GPMC_Config *config, uint32_t timeOut)
{
    int32_t status  =   SystemP_SUCCESS;
    const GPMC_HwAttrs *hwAttrs = config->attrs;
    uint64_t curTime = 0;

    if(timeOut != 0)
    {
        curTime = ClockP_getTimeUsec();
        /* Check for GPMC module reset status with timeout. */
        while((CSL_REG32_FEXT(hwAttrs->gpmcBaseAddr + CSL_GPMC_SYSSTATUS, GPMC_SYSSTATUS_RESETDONE) != CSL_GPMC_SYSSTATUS_RESETDONE_RSTDONE) &&
            ((ClockP_getTimeUsec() - curTime) < timeOut))
        {

        }

        if(CSL_REG32_FEXT(hwAttrs->gpmcBaseAddr + CSL_GPMC_SYSSTATUS, GPMC_SYSSTATUS_RESETDONE) != CSL_GPMC_SYSSTATUS_RESETDONE_RSTDONE)
        {
            return SystemP_FAILURE;
        }

    }
    else
    {
        if(CSL_REG32_FEXT(hwAttrs->gpmcBaseAddr + CSL_GPMC_SYSSTATUS, GPMC_SYSSTATUS_RESETDONE) != CSL_GPMC_SYSSTATUS_RESETDONE_RSTDONE)
        {
            return SystemP_FAILURE;
        }
    }

    return status;

 }

static uint32_t GPMC_waitPinStatusGet(uint32_t baseAddr, uint32_t pin)
{
    uint32_t pinStatus;

    pinStatus = 0;
    /* Check WAIT PIN status. */
    if (pin == CSL_GPMC_CONFIG1_WAITPINSELECT_W0)
    {
        pinStatus = CSL_REG32_FEXT(baseAddr + CSL_GPMC_STATUS,
                                  GPMC_STATUS_WAIT0STATUS);
    }
    else if (pin == CSL_GPMC_CONFIG1_WAITPINSELECT_W1)
    {
        pinStatus = CSL_REG32_FEXT(baseAddr + CSL_GPMC_STATUS,
                                  GPMC_STATUS_WAIT1STATUS);
    }
    else
    {
        /*
         * Do nothing. Error will be generated by the hardware
         */
    }

    return (pinStatus);
}

static int32_t GPMC_waitPinStatusReadyWaitTimeout(GPMC_Handle handle, uint32_t timeOut)
{

    int32_t status  =   SystemP_SUCCESS;
    const GPMC_HwAttrs *hwAttrs = NULL;
    uint64_t curTime= 0;

    if(handle != NULL)
    {
        hwAttrs = ((GPMC_Config*)handle)->attrs;

        if(timeOut != 0)
        {
            curTime = ClockP_getTimeUsec();
            /* Check WAIT PIN status with timeout. */
            while((GPMC_waitPinStatusGet(hwAttrs->gpmcBaseAddr,hwAttrs->waitPinNum) == \
            CSL_GPMC_STATUS_WAIT0STATUS_W0ACTIVEL) && ((ClockP_getTimeUsec() - curTime) < timeOut))
            {
                /* Do nothing. */
            }

            if((GPMC_waitPinStatusGet(hwAttrs->gpmcBaseAddr,hwAttrs->waitPinNum) == \
            CSL_GPMC_STATUS_WAIT0STATUS_W0ACTIVEH))
            {
                status = SystemP_SUCCESS;
            }
            else
            {
                status = SystemP_FAILURE;
            }
        }
        else
        {
            if((GPMC_waitPinStatusGet(hwAttrs->gpmcBaseAddr,hwAttrs->waitPinNum) == \
            CSL_GPMC_STATUS_WAIT0STATUS_W0ACTIVEH))
            {
                status = SystemP_SUCCESS;
            }
            else
            {
                status = SystemP_FAILURE;
            }
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;

 }

static int32_t GPMC_waitPinInteruptStatusReadyWaitTimeout(GPMC_Handle handle, uint32_t timeOut)
{
    int32_t status  =   SystemP_SUCCESS;
    uint32_t waitPinInterupt = 0;
    uint64_t curTime = 0;

    if(handle != NULL)
    {
        const GPMC_HwAttrs *hwAttrs = ((GPMC_Config*)handle)->attrs;

        if(hwAttrs->waitPinNum == CSL_GPMC_CONFIG1_WAITPINSELECT_W0)
        {
            waitPinInterupt = GPMC_WAIT0EDGEDETECTION_STATUS;
        }
        else if(hwAttrs->waitPinNum == CSL_GPMC_CONFIG1_WAITPINSELECT_W1)
        {
            waitPinInterupt = GPMC_WAIT1EDGEDETECTION_STATUS;
        }

        if(timeOut != 0)
        {

            curTime = ClockP_getTimeUsec();
            /* Check WAIT PIN interupt status with timeout.*/
            while((GPMC_interuptStatusGet(hwAttrs->gpmcBaseAddr,waitPinInterupt) == 0) \
            && ((ClockP_getTimeUsec() - curTime) < timeOut))
            {
                /* Do nothing.*/
            }


            if(GPMC_interuptStatusGet(hwAttrs->gpmcBaseAddr,waitPinInterupt) == 1)
            {
                status = SystemP_SUCCESS;
            }
            else
            {
                status = SystemP_FAILURE;
            }

        }
        else
        {
            if(GPMC_interuptStatusGet(hwAttrs->gpmcBaseAddr,hwAttrs->waitPinNum) == 1)
            {
                status = SystemP_SUCCESS;
            }
            else
            {
                status = SystemP_FAILURE;
            }
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;

 }

static void GPMC_waitPinPolaritySelect(GPMC_Handle handle, uint32_t pin,
                                uint32_t polarity)
{
    if(handle != NULL)
    {
        const GPMC_HwAttrs *hwAttrs = ((GPMC_Config*)handle)->attrs;
        /* Select WAIT PIN polarity. */
        if (pin == CSL_GPMC_CONFIG1_WAITPINSELECT_W0)
        {
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG, GPMC_CONFIG_WAIT0PINPOLARITY,
                        polarity);
        }
        else if (pin == CSL_GPMC_CONFIG1_WAITPINSELECT_W1)
        {
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG, GPMC_CONFIG_WAIT1PINPOLARITY,
                        polarity);
        }
        else
        {
            /*
            * Do nothing. Error will be generated by the hardware
            */
        }
    }
}


static uint32_t  GPMC_interuptStatusGet(uint32_t baseAddr, uint32_t interupt)
{
    uint32_t retVal;

    retVal = 0;
    /* Get GPMC interupt status. */
    switch (interupt)
    {
        case GPMC_FIFOEVENT_STATUS:
            retVal = CSL_REG32_FEXT(baseAddr + CSL_GPMC_IRQSTATUS,
                                   GPMC_IRQSTATUS_FIFOEVENTSTATUS);
            break;
        case GPMC_TERMINALCOUNT_STATUS:
            retVal = CSL_REG32_FEXT(baseAddr + CSL_GPMC_IRQSTATUS,
                                   GPMC_IRQSTATUS_TERMINALCOUNTSTATUS);
            break;
        case GPMC_WAIT0EDGEDETECTION_STATUS:
            retVal = CSL_REG32_FEXT(baseAddr + CSL_GPMC_IRQSTATUS,
                                   GPMC_IRQSTATUS_WAIT0EDGEDETECTIONSTATUS);
            break;
        case GPMC_WAIT1EDGEDETECTION_STATUS:
            retVal = CSL_REG32_FEXT(baseAddr + CSL_GPMC_IRQSTATUS,
                                   GPMC_IRQSTATUS_WAIT1EDGEDETECTIONSTATUS);
            break;

        default:
            break;
    }

    return (retVal);
}

static void GPMC_interuptStatusClear(uint32_t baseAddr, uint32_t interupt)
{
    /* Clear GPMC interupt status. */
    switch (interupt)
    {
        case GPMC_FIFOEVENT_STATUS:
            CSL_REG32_FINS(baseAddr + CSL_GPMC_IRQSTATUS,
                          GPMC_IRQSTATUS_FIFOEVENTSTATUS,
                          CSL_GPMC_IRQSTATUS_FIFOEVENTSTATUS_FIFOSTAT1_W);
            break;
        case GPMC_TERMINALCOUNT_STATUS:
            CSL_REG32_FINS(
                baseAddr + CSL_GPMC_IRQSTATUS, GPMC_IRQSTATUS_TERMINALCOUNTSTATUS,
                CSL_GPMC_IRQSTATUS_TERMINALCOUNTSTATUS_TCSTAT1_W);
            break;
        case GPMC_WAIT0EDGEDETECTION_STATUS:
            CSL_REG32_FINS(
                baseAddr + CSL_GPMC_IRQSTATUS,
                GPMC_IRQSTATUS_WAIT0EDGEDETECTIONSTATUS,
                CSL_GPMC_IRQSTATUS_WAIT0EDGEDETECTIONSTATUS_W0DET1_W);
            break;
        case GPMC_WAIT1EDGEDETECTION_STATUS:
            CSL_REG32_FINS(
                baseAddr + CSL_GPMC_IRQSTATUS,
                GPMC_IRQSTATUS_WAIT1EDGEDETECTIONSTATUS,
                CSL_GPMC_IRQSTATUS_WAIT1EDGEDETECTIONSTATUS_W1DET1_W);
            break;

        default:
            break;
    }
}

static void GPMC_enableInterupt(uint32_t baseAddr, uint32_t interupt)
{
    /* Enable GPMC interupt. */
    switch (interupt)
    {
        case GPMC_FIFOEVENT_INT:
            CSL_REG32_FINS(baseAddr + CSL_GPMC_IRQENABLE,
                          GPMC_IRQENABLE_FIFOEVENTENABLE,
                          CSL_GPMC_IRQENABLE_FIFOEVENTENABLE_FIFOENABLED);
            break;
        case GPMC_TERMINALCOUNT_INT:
            CSL_REG32_FINS(
                baseAddr + CSL_GPMC_IRQENABLE,
                GPMC_IRQENABLE_TERMINALCOUNTEVENTENABLE,
                CSL_GPMC_IRQENABLE_TERMINALCOUNTEVENTENABLE_TCENABLED);
            break;
        case GPMC_WAIT0EDGEDETECTION_INT:
            CSL_REG32_FINS(
                baseAddr + CSL_GPMC_IRQENABLE,
                GPMC_IRQENABLE_WAIT0EDGEDETECTIONENABLE,
                CSL_GPMC_IRQENABLE_WAIT0EDGEDETECTIONENABLE_W0ENABLED);
            break;
        case GPMC_WAIT1EDGEDETECTION_INT:
            CSL_REG32_FINS(
                baseAddr + CSL_GPMC_IRQENABLE,
                GPMC_IRQENABLE_WAIT1EDGEDETECTIONENABLE,
                CSL_GPMC_IRQENABLE_WAIT1EDGEDETECTIONENABLE_W1ENABLED);
            break;

        default:
            break;
    }
}

static void GPMC_disableInterupt(uint32_t baseAddr, uint32_t interupt)
{
    /* Disable GPMC interupt. */
    switch (interupt)
    {
        case GPMC_FIFOEVENT_INT:
            CSL_REG32_FINS(baseAddr + CSL_GPMC_IRQENABLE,
                          GPMC_IRQENABLE_FIFOEVENTENABLE,
                          CSL_GPMC_IRQENABLE_FIFOEVENTENABLE_FIFOMASKED);
            break;
        case GPMC_TERMINALCOUNT_INT:
            CSL_REG32_FINS(
                baseAddr + CSL_GPMC_IRQENABLE,
                GPMC_IRQENABLE_TERMINALCOUNTEVENTENABLE,
                CSL_GPMC_IRQENABLE_TERMINALCOUNTEVENTENABLE_TCMASKED);
            break;
        case GPMC_WAIT0EDGEDETECTION_INT:
            CSL_REG32_FINS(
                baseAddr + CSL_GPMC_IRQENABLE,
                GPMC_IRQENABLE_WAIT0EDGEDETECTIONENABLE,
                CSL_GPMC_IRQENABLE_WAIT0EDGEDETECTIONENABLE_W0MASKED);
            break;
        case GPMC_WAIT1EDGEDETECTION_INT:
            CSL_REG32_FINS(
                baseAddr + CSL_GPMC_IRQENABLE,
                GPMC_IRQENABLE_WAIT1EDGEDETECTIONENABLE,
                CSL_GPMC_IRQENABLE_WAIT1EDGEDETECTIONENABLE_W1MASKED);
            break;

        default:
            break;
    }
}

static int32_t GPMC_programInstance(GPMC_Config *config)
{
    int32_t status = SystemP_SUCCESS;

    if(config != NULL)
    {
        const GPMC_HwAttrs *hwAttrs = config->attrs;
        GPMC_Object *object  = config->object;

        /* Reset GPMC */
        CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_SYSCONFIG, GPMC_SYSCONFIG_SOFTRESET,CSL_GPMC_SYSCONFIG_SOFTRESET_RESET);

        status += GPMC_moduleResetStatusWaitTimeout(config,GPMC_MODULE_RESET_WAIT_TIME_MAX);

        if(status == SystemP_SUCCESS)
        {
            /*set GPMC in NORMAL mode*/
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_SYSCONFIG, GPMC_SYSCONFIG_SOFTRESET,CSL_GPMC_SYSCONFIG_SOFTRESET_NORMAL);

            /* Disable all interrupts */
            GPMC_disableInterupt(hwAttrs->gpmcBaseAddr,GPMC_FIFOEVENT_INT);
            GPMC_disableInterupt(hwAttrs->gpmcBaseAddr,GPMC_TERMINALCOUNT_INT);
            GPMC_disableInterupt(hwAttrs->gpmcBaseAddr,GPMC_WAIT0EDGEDETECTION_INT);
            GPMC_disableInterupt(hwAttrs->gpmcBaseAddr,GPMC_WAIT1EDGEDETECTION_INT);

            /* Disable Chip select*/
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG7(object->params.chipSel), GPMC_CONFIG7_CSVALID, CSL_GPMC_CONFIG7_CSVALID_CSDISABLED);

            /* Timeout control disable */
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_TIMEOUT_CONTROL,
                        GPMC_TIMEOUT_CONTROL_TIMEOUTENABLE,
                        CSL_GPMC_TIMEOUT_CONTROL_TIMEOUTENABLE_TODISABLED);
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_TIMEOUT_CONTROL,
                        GPMC_TIMEOUT_CONTROL_TIMEOUTSTARTVALUE, 0);
            /* Set CONFIG 1 Parameters*/
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                        GPMC_CONFIG1_GPMCFCLKDIVIDER, hwAttrs->clkDivider );
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                        GPMC_CONFIG1_MUXADDDATA,  hwAttrs->addrDataMux);

            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                        GPMC_CONFIG1_WAITMONITORINGTIME, CSL_GPMC_CONFIG1_WAITMONITORINGTIME_ATVALID);

            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                        GPMC_CONFIG1_WAITREADMONITORING, CSL_GPMC_CONFIG1_WAITREADMONITORING_WNOTMONIT);

            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                        GPMC_CONFIG1_READTYPE, hwAttrs->readType);

            /* Set the wait pin polarity */
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                        GPMC_CONFIG1_WAITPINSELECT,  hwAttrs->waitPinNum);
            /* Set the Time granularity */
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                        GPMC_CONFIG1_TIMEPARAGRANULARITY, hwAttrs->timeLatency);

            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                        GPMC_CONFIG1_READMULTIPLE,  hwAttrs->accessType);

            /* Set chip select address*/
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG7(object->params.chipSel),
                        GPMC_CONFIG7_BASEADDRESS, (hwAttrs->chipSelBaseAddr >> GPMC_CS_BASE_ADDR_SHIFT) & 0x3fU);

            /* Set chip select address */
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG7(object->params.chipSel),
                        GPMC_CONFIG7_MASKADDRESS,  hwAttrs->chipSelAddrSize);

            /* Set Wait pin polarity*/
            GPMC_waitPinPolaritySelect((GPMC_Handle*)config, hwAttrs->waitPinNum, hwAttrs->waitPinPol);

            GPMC_interuptStatusClear(hwAttrs->gpmcBaseAddr,GPMC_WAIT0EDGEDETECTION_STATUS);
            /* Enable interupt for the WAIT PIN*/
            GPMC_enableInterupt(hwAttrs->gpmcBaseAddr,GPMC_WAIT0EDGEDETECTION_INT);
            /* Enable CHIP SELECT*/
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG7(object->params.chipSel), \
                            GPMC_CONFIG7_CSVALID, CSL_GPMC_CONFIG7_CSVALID_CSENABLED);

            status += GPMC_waitPinStatusReadyWaitTimeout((GPMC_Handle)config, GPMC_WAIT_PIN_STATUS_WAIT_TIME_MAX);
        }

    }

    return status;
}

static int32_t GPMC_programInstancePsram(GPMC_Config *config)
{
    int32_t status = SystemP_SUCCESS;

    if(config != NULL)
    {
        const GPMC_HwAttrs *hwAttrs = config->attrs;
        GPMC_Object *object  = config->object;

        /* Reset GPMC */
        CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_SYSCONFIG, GPMC_SYSCONFIG_SOFTRESET,CSL_GPMC_SYSCONFIG_SOFTRESET_RESET);

        status += GPMC_moduleResetStatusWaitTimeout(config,GPMC_MODULE_RESET_WAIT_TIME_MAX);

        if(status == SystemP_SUCCESS)
        {
            /*set GPMC in NORMAL mode*/
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_SYSCONFIG, GPMC_SYSCONFIG_SOFTRESET,CSL_GPMC_SYSCONFIG_SOFTRESET_NORMAL);

            /* Disable Chip select*/
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG7(object->params.chipSel), GPMC_CONFIG7_CSVALID, CSL_GPMC_CONFIG7_CSVALID_CSDISABLED);

            /* Set CONFIG 1 Parameters*/
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                        GPMC_CONFIG1_GPMCFCLKDIVIDER, hwAttrs->clkDivider );
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                        GPMC_CONFIG1_MUXADDDATA,  hwAttrs->addrDataMux);

            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                        GPMC_CONFIG1_WAITMONITORINGTIME, CSL_GPMC_CONFIG1_WAITMONITORINGTIME_ATVALID);

            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                        GPMC_CONFIG1_WAITREADMONITORING, CSL_GPMC_CONFIG1_WAITREADMONITORING_WNOTMONIT);

            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                        GPMC_CONFIG1_READTYPE, hwAttrs->readType);

            CSL_REG32_WR(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel), 0x1000);

            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                        GPMC_CONFIG1_TIMEPARAGRANULARITY, hwAttrs->timeLatency);

            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG1(object->params.chipSel),
                        GPMC_CONFIG1_READMULTIPLE,  hwAttrs->accessType);

            /* Set chip select address*/
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG7(object->params.chipSel),
                        GPMC_CONFIG7_BASEADDRESS, (hwAttrs->chipSelBaseAddr >> GPMC_CS_BASE_ADDR_SHIFT) & 0x3fU);

            CSL_REG32_WR(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG, 0x0);

            /* Set chip select address */
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_CONFIG7(object->params.chipSel),
                        GPMC_CONFIG7_MASKADDRESS,  hwAttrs->chipSelAddrSize);
        }

    }

    return status;
}

static void GPMC_transferCallback(GPMC_Handle handle, GPMC_Transaction *msg)
{
    GPMC_Object   *object; /* GPMC object */

    /* Input parameter validation */
    if (handle != NULL)
    {
        /* Get the pointer to the object. */
        object = ((GPMC_Config*)handle)->object;

        /* Indicate transfer complete. */
        SemaphoreP_post(&object->transferComplete);
    }
}

static int32_t GPMC_prefetchPostWriteConfigEnable(GPMC_Handle handle, uint8_t mode,
                                                uint32_t transferCount, uint8_t modeDMA)
{
    int32_t status = SystemP_SUCCESS;

    /* Input parameter validation */
    if(handle != NULL)
    {
        const GPMC_HwAttrs *attrs = ((GPMC_Config*)handle)->attrs;

        if(mode == GPMC_PREFETCH_ACCESSMODE_READ)
        {
            /* Set PREFETCH/POST write engine to read mode. */
            CSL_REG32_FINS(attrs->gpmcBaseAddr + CSL_GPMC_PREFETCH_CONFIG1, GPMC_PREFETCH_CONFIG1_ACCESSMODE, \
            CSL_GPMC_PREFETCH_CONFIG1_ACCESSMODE_PREFETCHREAD);

            if(modeDMA)
            {
                /* Enable DMA sync bit for READ operation. */
                CSL_REG32_FINS(attrs->gpmcBaseAddr + CSL_GPMC_PREFETCH_CONFIG1, GPMC_PREFETCH_CONFIG1_DMAMODE, \
                CSL_GPMC_PREFETCH_CONFIG1_DMAMODE_DMAREQSYNC);
            }

        }
        else
        {
            /* Set PREFETCH/POST write engine to write mode. */
            CSL_REG32_FINS(attrs->gpmcBaseAddr + CSL_GPMC_PREFETCH_CONFIG1, GPMC_PREFETCH_CONFIG1_ACCESSMODE, \
            CSL_GPMC_PREFETCH_CONFIG1_ACCESSMODE_WRITEPOSTING);
        }

        /*Set transfer count*/
        CSL_REG32_FINS(attrs->gpmcBaseAddr + CSL_GPMC_PREFETCH_CONFIG2, GPMC_PREFETCH_CONFIG2_TRANSFERCOUNT, \
        transferCount);

        /*Enable the engine */
        CSL_REG32_FINS(attrs->gpmcBaseAddr + CSL_GPMC_PREFETCH_CONFIG1, GPMC_PREFETCH_CONFIG1_ENABLEENGINE, \
        CSL_GPMC_PREFETCH_CONFIG1_ENABLEENGINE_PPENABLED);

        /*Start the prefetch engine*/
        CSL_REG32_FINS(attrs->gpmcBaseAddr + CSL_GPMC_PREFETCH_CONTROL, GPMC_PREFETCH_CONTROL_STARTENGINE, \
        CSL_GPMC_PREFETCH_CONTROL_STARTENGINE_START);

    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

static int32_t GPMC_prefetchPostWriteConfigDisable(GPMC_Handle handle)
{
    int32_t status = SystemP_SUCCESS;

    /* Input parameter validation */
    if(handle != NULL)
    {
        const GPMC_HwAttrs *attrs = ((GPMC_Config*)handle)->attrs;

        /*Disable and stop the prefetch engine*/
        CSL_REG32_FINS(attrs->gpmcBaseAddr + CSL_GPMC_PREFETCH_CONTROL, GPMC_PREFETCH_CONTROL_STARTENGINE, \
        CSL_GPMC_PREFETCH_CONTROL_STARTENGINE_STOP);
        CSL_REG32_FINS(attrs->gpmcBaseAddr + CSL_GPMC_PREFETCH_CONFIG1, GPMC_PREFETCH_CONFIG1_ENABLEENGINE, \
        CSL_GPMC_PREFETCH_CONFIG1_ENABLEENGINE_PPDISABLED);
        /* Disable DMA sync bit. */
        CSL_REG32_FINS(attrs->gpmcBaseAddr + CSL_GPMC_PREFETCH_CONFIG1, GPMC_PREFETCH_CONFIG1_DMAMODE, \
                CSL_GPMC_PREFETCH_CONFIG1_DMAMODE_RESETVAL);

    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

static int32_t GPMC_isDmaRestrictedRegion(GPMC_Handle handle, uint32_t addr)
{
    int32_t isRestricted = FALSE;
    const GPMC_HwAttrs *attrs = ((GPMC_Config *)handle)->attrs;

    if(NULL != attrs->dmaRestrictedRegions)
    {
        const GPMC_AddrRegion *addrRegions = attrs->dmaRestrictedRegions;
        uint32_t i = 0;
        uint32_t start;
        uint32_t size;

        /* Check for DMA restricted regions. */
        while(addrRegions[i].regionStartAddr != 0xFFFFFFFF)
        {
            start = addrRegions[i].regionStartAddr;
            size = addrRegions[i].regionSize;

            if((addr >= start) && (addr < (start + size)))
            {
                isRestricted = TRUE;
                break;
            }
            i++;
        }
    }

    return isRestricted;
}

static void GPMC_nandCommandWrite(GPMC_Handle handle, uint32_t cmd)
{
    /* Input parameter validation */
    if(handle != NULL)
    {
        const GPMC_HwAttrs *hwAttrs = ((GPMC_Config*)handle)->attrs;
        GPMC_Object *object = ((GPMC_Config*)handle)->object;
        /* Set NAND command. */
        CSL_REG8_WR(hwAttrs->gpmcBaseAddr + CSL_GPMC_NAND_COMMAND(object->params.chipSel),
                    cmd);
    }
}

static void GPMC_nandAddressWrite(GPMC_Handle handle, uint32_t address)
{
    /* Input parameter validation */
    if(handle != NULL)
    {
        const GPMC_HwAttrs *hwAttrs = ((GPMC_Config*)handle)->attrs;
        GPMC_Object *object = ((GPMC_Config*)handle)->object;
        /* Set NAND address. */
        CSL_REG8_WR(hwAttrs->gpmcBaseAddr + CSL_GPMC_NAND_ADDRESS(object->params.chipSel),
                    address);
    }
}

static void GPMC_eccResultSizeSelect(GPMC_Handle handle, uint32_t eccResReg,
                             uint32_t eccSize)
{
    const GPMC_HwAttrs *hwAttrs = ((GPMC_Config*)handle)->attrs;

    /* Set ECC size for ECC result register. */
    switch (eccResReg)
    {
        case GPMC_ECC_RESULT_1:
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_ECC_SIZE_CONFIG,
                          GPMC_ECC_SIZE_CONFIG_ECC1RESULTSIZE,
                          eccSize);
            break;
        case GPMC_ECC_RESULT_2:
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_ECC_SIZE_CONFIG,
                          GPMC_ECC_SIZE_CONFIG_ECC2RESULTSIZE,
                          eccSize);
            break;
        case GPMC_ECC_RESULT_3:
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_ECC_SIZE_CONFIG,
                          GPMC_ECC_SIZE_CONFIG_ECC3RESULTSIZE,
                          eccSize);
            break;
        case GPMC_ECC_RESULT_4:
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_ECC_SIZE_CONFIG,
                          GPMC_ECC_SIZE_CONFIG_ECC4RESULTSIZE,
                          eccSize);
            break;
        case GPMC_ECC_RESULT_5:
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_ECC_SIZE_CONFIG,
                          GPMC_ECC_SIZE_CONFIG_ECC5RESULTSIZE,
                          eccSize);
            break;
        case GPMC_ECC_RESULT_6:
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_ECC_SIZE_CONFIG,
                          GPMC_ECC_SIZE_CONFIG_ECC6RESULTSIZE,
                          eccSize);
            break;
        case GPMC_ECC_RESULT_7:
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_ECC_SIZE_CONFIG,
                          GPMC_ECC_SIZE_CONFIG_ECC7RESULTSIZE,
                          eccSize);
            break;
        case GPMC_ECC_RESULT_8:
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_ECC_SIZE_CONFIG,
                          GPMC_ECC_SIZE_CONFIG_ECC8RESULTSIZE,
                          eccSize);
            break;
        case GPMC_ECC_RESULT_9:
            CSL_REG32_FINS(hwAttrs->gpmcBaseAddr + CSL_GPMC_ECC_SIZE_CONFIG,
                          GPMC_ECC_SIZE_CONFIG_ECC9RESULTSIZE,
                          eccSize);
            break;

        default:
            break;
    }
}

static uint32_t GPMC_eccBchResultGet(GPMC_Handle handle, uint32_t resIndex , uint32_t sector)
{
    uint32_t result = 0;

    /* Input parameter validation. */
    if(handle != NULL)
    {
        const GPMC_HwAttrs *attrs = ((GPMC_Config*)handle)->attrs;

        /* Get BCH syndrome polynomial per sector. */
        switch (resIndex)
        {
            case GPMC_BCH_RESULT0:
                result = CSL_REG32_RD(attrs->gpmcBaseAddr + CSL_GPMC_BCH_RESULT_0(sector));
                break;
            case GPMC_BCH_RESULT1:
                result = CSL_REG32_RD(attrs->gpmcBaseAddr + CSL_GPMC_BCH_RESULT_1(sector));
                break;
            case GPMC_BCH_RESULT2:
                result = CSL_REG32_RD(attrs->gpmcBaseAddr + CSL_GPMC_BCH_RESULT_2(sector));
                break;
            case GPMC_BCH_RESULT3:
                result = CSL_REG32_RD(attrs->gpmcBaseAddr + CSL_GPMC_BCH_RESULT_3(sector));
                break;
            case GPMC_BCH_RESULT4:
                result = CSL_REG32_RD(attrs->gpmcBaseAddr + CSL_GPMC_BCH_RESULT_4(sector));
                break;
            case GPMC_BCH_RESULT5:
                result = CSL_REG32_RD(attrs->gpmcBaseAddr + CSL_GPMC_BCH_RESULT_5(sector));
                break;
            case GPMC_BCH_RESULT6:
                result = CSL_REG32_RD(attrs->gpmcBaseAddr + CSL_GPMC_BCH_RESULT_6(sector));
                break;

            default:
                break;
        }
    }

    return (result);
}