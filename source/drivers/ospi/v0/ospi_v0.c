/*
 *  Copyright (C) 2021-24 Texas Instruments Incorporated
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
 *  \file ospi_v0.c
 *
 *  \brief File containing OSPI Driver APIs implementation for version V0.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* This is needed for memset/memcpy */
#include <string.h>
#include <drivers/ospi.h>
#include <drivers/ospi/v0/cslr_ospi.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/CacheP.h>
#include <drivers/hw_include/cslr.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/ospi/v0/lld/dma/ospi_lld_dma.h>

/* TODO:HS hack, remove it when DMA bug is fixed */
// #include <drivers/bootloader/soc/bootloader_soc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define OSPI_READ_WRITE_TIMEOUT (500000U)
#define OSPI_TRUE               (1U)
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct
{
    void *openLock;
    /**<  Lock to protect OSPI open*/
    SemaphoreP_Object lockObj;
    /**< Lock object */
} OSPI_DrvObj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Internal functions */
static void OSPI_interruptCallback(void *args);
static void OSPI_dmaInterruptCallback(void* args);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** \brief Driver object */
static OSPI_DrvObj gOspiDrvObj =
{
    .openLock      = NULL,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void OSPI_init(void)
{
    int32_t status;
    uint32_t count;
    OSPI_Object *obj;

    /* Init each driver instance object */
    for(count = 0U; count < gOspiConfigNum; count++)
    {
        /* Init object variables */
        obj = gOspiConfig[count].object;
        DebugP_assert(NULL != obj);
        memset(obj, 0, sizeof(OSPI_Object));
    }

    /* Create the driver lock */
    status = SemaphoreP_constructMutex(&gOspiDrvObj.lockObj);
    if(SystemP_SUCCESS == status)
    {
        gOspiDrvObj.openLock = &gOspiDrvObj.lockObj;
    }

    return;
}

void OSPI_deinit(void)
{
    /* Delete driver lock */
    if(NULL != gOspiDrvObj.openLock)
    {
        SemaphoreP_destruct(&gOspiDrvObj.lockObj);
        gOspiDrvObj.openLock = NULL;
    }

    return;
}

OSPI_Handle OSPI_open(uint32_t index, const OSPI_Params *openParams)
{
    int32_t status = SystemP_SUCCESS;
    OSPI_Handle handle = NULL;
    OSPI_Config *config = NULL;
    OSPI_Object *obj = NULL;
    HwiP_Params hwiPrms;
    const OSPI_Attrs *attrs;

    OSPILLD_InitHandle ospilldInitHandle;
    /* OSPI LLD Init Handle */

    OSPILLD_Handle      ospilldHandle;
    /* OSPI LLD Handle */

    int32_t dmaInterrupt = SystemP_SUCCESS;

    OSPI_DmaConfig *dmaConfig = NULL;
    size_t resMemoryCount = 4;

    /* Check for valid index */
    if(index >= gOspiConfigNum)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        config = &gOspiConfig[index];
        dmaConfig = &gOspiDmaConfig[openParams->ospiDmaChIndex]; // Before am263px it was [index]
    }

    /* Protect this region from a concurrent OSPI_Open */
    DebugP_assert(NULL != gOspiDrvObj.openLock);
    SemaphoreP_pend(&gOspiDrvObj.lockObj, SystemP_WAIT_FOREVER);

    if(SystemP_SUCCESS == status)
    {
        obj = config->object;
        DebugP_assert(NULL != obj);
        DebugP_assert(NULL != config->attrs);
        attrs = config->attrs;
        if(OSPI_TRUE == obj->isOpen)
        {
            /* Handle already opened */
            status = SystemP_FAILURE;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        obj->handle = (OSPI_Handle)config;

        /* Mapping HLD parameter with LLD*/
        obj->ospilldHandle       = &obj->ospilldObject;
        ospilldHandle            = obj->ospilldHandle;
        ospilldHandle->hOspiInit = &obj->ospilldInitObject;
        ospilldInitHandle        = ospilldHandle->hOspiInit;

        /* Populating LLD parameters. */
        ospilldHandle->baseAddr                  = attrs->baseAddr;
        ospilldHandle->args                      = (void *)obj->handle;
        ospilldHandle->protocol                  = attrs->protocol;
        ospilldHandle->openParams                = openParams;
        ospilldHandle->Clock_getTicks            = ClockP_getTicks;
        ospilldHandle->Clock_usecToTicks         = ClockP_usecToTicks;
        ospilldHandle->Clock_usleep              = ClockP_usleep;
        ospilldHandle->currTrans                 = &(ospilldHandle->trans);

        ospilldInitHandle->dataBaseAddr          = attrs->dataBaseAddr;
        ospilldInitHandle->inputClkFreq          = attrs->inputClkFreq;
        ospilldInitHandle->moduleId              = attrs->moduleId;
        ospilldInitHandle->clkId                 = attrs->clkId;
        ospilldInitHandle->intrNum               = attrs->intrNum;
        ospilldInitHandle->intrEnable            = attrs->intrEnable;
        ospilldInitHandle->intrPriority          = attrs->intrPriority;
        ospilldInitHandle->dmaEnable             = attrs->dmaEnable;
        ospilldInitHandle->phyEnable             = attrs->phyEnable;
        ospilldInitHandle->dacEnable             = attrs->dacEnable;
        ospilldInitHandle->frmFmt                = attrs->frmFmt;
        ospilldInitHandle->chipSelect            = attrs->chipSelect;
        ospilldInitHandle->decChipSelect         = attrs->decChipSelect;
        ospilldInitHandle->baudRateDiv           = attrs->baudRateDiv;
        ospilldInitHandle->dmaRestrictedRegions  = attrs->dmaRestrictedRegions;
        ospilldInitHandle->phyConfiguration      = attrs->phyConfiguration;
        ospilldInitHandle->validateOtp           = attrs->validateOtp;

        memcpy(ospilldInitHandle->devDelays,attrs->devDelays, resMemoryCount * sizeof(uint32_t));       

        /* If DMA is enabled, program UDMA block copy channel */
        if(OSPI_TRUE == attrs->dmaEnable)
        {
            ospilldInitHandle->ospiDmaHandle    = (OSPI_DmaHandle) dmaConfig;
            ospilldInitHandle->ospiDmaChConfig  = (OSPI_DmaChConfig) dmaConfig->ospiDmaArgs;

            if (NULL != ospilldInitHandle->ospiDmaHandle)
            {
                /* Program OSPI instance according the user config */
                status += OSPI_lld_initDma(ospilldHandle);

                dmaInterrupt = OSPI_isDmaInterruptEnabled(ospilldHandle);
                if (dmaInterrupt == OSPI_TRUE)
                {
                    ospilldHandle->readCompleteCallback = &OSPI_dmaInterruptCallback;
                }
            }
            else
            {
                dmaInterrupt = FALSE;
            }

        }
        else
        {
            ospilldInitHandle->ospiDmaHandle = NULL;
            ospilldInitHandle->ospiDmaChConfig = NULL;
            /* Program OSPI instance according the user config */
            status += OSPI_lld_init(ospilldHandle);
        }

        /* Create instance lock */
        status += SemaphoreP_constructMutex(&obj->lockObj);

        /* Create transfer sync semaphore */
        status += SemaphoreP_constructBinary(&obj->transferSemObj, 0U);

        /* Register interrupt */
        if(OSPI_TRUE == attrs->intrEnable && OSPI_TRUE != attrs->dmaEnable)
        {
            HwiP_Params_init(&hwiPrms);
            hwiPrms.intNum      = attrs->intrNum;
            hwiPrms.callback    = &OSPI_lld_isr;
            hwiPrms.priority    = attrs->intrPriority;
            hwiPrms.args        = (void *) ospilldHandle;
            hwiPrms.isPulse     = 0U;
            status += HwiP_construct(&obj->hwiObj, &hwiPrms);
            ospilldHandle->interruptCallback = &OSPI_interruptCallback;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        obj->isOpen = 1U;
        handle = (OSPI_Handle) config;
    }

    SemaphoreP_post(&gOspiDrvObj.lockObj);

    /* Free up resources in case of error */
    if(SystemP_SUCCESS != status)
    {
        if(NULL != config)
        {
            OSPI_close((OSPI_Handle) config);
        }
    }
    return handle;
}

void OSPI_close(OSPI_Handle handle)
{
    OSPILLD_Handle ospilldHandle;

    if(handle != NULL)
    {
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        ospilldHandle = obj->ospilldHandle;

        /* If DMA block copy channel was opened, close it */
        if(attrs->dmaEnable == OSPI_TRUE)
        {
            (void) OSPI_lld_deInitDma(ospilldHandle);
        }
        else
        {
            (void) OSPI_lld_deInit(ospilldHandle);
        }

        /* Destruct all locks and Hwi objects */
        SemaphoreP_destruct(&obj->lockObj);
        SemaphoreP_destruct(&obj->transferSemObj);
        HwiP_destruct(&obj->hwiObj);
        obj->isOpen = 0;
        SemaphoreP_post(&gOspiDrvObj.lockObj);
    }

    return;
}

OSPI_Handle OSPI_getHandle(uint32_t driverInstanceIndex)
{
    OSPI_Handle         handle = NULL;
    /* Check index */
    if(driverInstanceIndex < gOspiConfigNum)
    {
        OSPI_Object *obj;
        obj = gOspiConfig[driverInstanceIndex].object;

        if(obj && (OSPI_TRUE == obj->isOpen))
        {
            /* valid handle */
            handle = obj->handle;
        }
    }
    return handle;
}

void OSPI_Transaction_init(OSPI_Transaction *trans)
{
    if(NULL != trans)
    {
        OSPI_lld_Transaction_init(trans);
    }
}

void OSPI_ReadCmdParams_init(OSPI_ReadCmdParams *rdParams)
{
    if(NULL != rdParams)
    {
        OSPI_lld_readCmdParams_init(rdParams);
    }
}

void OSPI_WriteCmdParams_init(OSPI_WriteCmdParams *wrParams)
{
    if(NULL != wrParams)
    {
        OSPI_lld_writeCmdParams_init(wrParams);
    }
}

uint32_t OSPI_getInputClk(OSPI_Handle handle)
{
    uint32_t retVal = 0U;
    OSPILLD_Handle hOspi;
    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        retVal = OSPI_lld_getInputClk(hOspi);
    }
    return retVal;
}

uint32_t OSPI_isDacEnable(OSPI_Handle handle)
{
    uint32_t retVal = 0U;
    OSPILLD_Handle hOspi;
    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        retVal = OSPI_lld_isDacEnable(hOspi);
    }
    return retVal;
}

uint32_t OSPI_isDmaEnable(OSPI_Handle handle)
{
    uint32_t retVal = 0U;
    OSPILLD_Handle hOspi;
    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        retVal = OSPI_lld_isDmaEnable(hOspi);
    }
    return retVal;
}

uint32_t OSPI_isIntrEnable(OSPI_Handle handle)
{
    uint32_t retVal = 0U;
    OSPILLD_Handle hOspi;
    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        retVal = OSPI_lld_isIntrEnable(hOspi);
    }
    return retVal;
}

uint32_t OSPI_isPhyEnable(OSPI_Handle handle)
{
    uint32_t retVal = 0U;
    OSPILLD_Handle hOspi;
    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        retVal = OSPI_lld_isPhyEnable(hOspi);
    }
    return retVal;
}

void OSPI_setPhyEnableSuccess(OSPI_Handle handle, uint32_t success)
{
    OSPILLD_Handle hOspi;
    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        OSPI_lld_setPhyEnableSuccess(hOspi,success);
    }
}

uint32_t OSPI_getPhyEnableSuccess(OSPI_Handle handle)
{
    uint32_t success = 0;
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        success = OSPI_lld_getPhyEnableSuccess(hOspi);
    }

    return success;
}

int32_t OSPI_enableDDR(OSPI_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status += OSPI_lld_enableDDR(hOspi);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t OSPI_enableSDR(OSPI_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status += OSPI_lld_enableSDR(hOspi);

    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t OSPI_enableDdrRdCmds(OSPI_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status = OSPI_lld_enableDdrRdCmds(hOspi);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t OSPI_setRdDataCaptureDelay(OSPI_Handle handle, uint32_t rdDataCapDelay)
{
    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status = OSPI_lld_setRdDataCaptureDelay(hOspi,rdDataCapDelay);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

void OSPI_setNumAddrBytes(OSPI_Handle handle, uint32_t numAddrBytes)
{
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        OSPI_lld_setNumAddrBytes(hOspi,numAddrBytes);
    }
}

void OSPI_setDeviceSize(OSPI_Handle handle, uint32_t pageSize, uint32_t blkSize)
{
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        OSPI_lld_setDeviceSize(hOspi,pageSize,blkSize);
    }
}

void OSPI_setModeBits(OSPI_Handle handle, uint32_t modeBits)
{
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        OSPI_lld_setModeBits(hOspi,modeBits);
    }
}

void OSPI_enableModeBitsCmd(OSPI_Handle handle)
{
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        OSPI_lld_enableModeBitsCmd(hOspi);
    }
}

void OSPI_enableModeBitsRead(OSPI_Handle handle)
{
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        OSPI_lld_enableModeBitsRead(hOspi);
    }
}

uint32_t OSPI_getProtocol(OSPI_Handle handle)
{
    uint32_t retVal = OSPI_NOR_PROTOCOL_INVALID;
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        retVal = OSPI_lld_getProtocol(hOspi);
    }
    return retVal;
}

void OSPI_setProtocol(OSPI_Handle handle, uint32_t protocol)
{
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        OSPI_lld_setProtocol(hOspi,protocol);
    }
}

void OSPI_setReadDummyCycles(OSPI_Handle handle, uint32_t dummyCycles)
{
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        OSPI_lld_setReadDummyCycles(hOspi,dummyCycles);
    }
}

void OSPI_setWriteDummyCycles(OSPI_Handle handle, uint32_t dummyCycles)
{
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        OSPI_lld_setWriteDummyCycles(hOspi,dummyCycles);
    }
}

void OSPI_setCmdDummyCycles(OSPI_Handle handle, uint32_t dummyCycles)
{
    OSPILLD_Handle hOspi;
    if((OSPI_Handle) NULL != handle)
    {

        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        OSPI_lld_setCmdDummyCycles(hOspi,dummyCycles);
    }
}

void OSPI_setDualOpCodeMode(OSPI_Handle handle)
{
    OSPILLD_Handle hOspi;
    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        OSPI_lld_setDualOpCodeMode(hOspi);
    }
    else
    {
        /* do nothing */
    }
}

void OSPI_clearDualOpCodeMode(OSPI_Handle handle)
{
    OSPILLD_Handle hOspi;
    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        OSPI_lld_clearDualOpCodeMode(hOspi);
    }

}

void OSPI_setXferOpCodes(OSPI_Handle handle, uint8_t readCmd, uint8_t pageProgCmd)
{
    OSPILLD_Handle hOspi;
    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        OSPI_lld_setXferOpCodes(hOspi,readCmd,pageProgCmd);
    }

}

void OSPI_setCmdExtType(OSPI_Handle handle, uint32_t cmdExtType)
{
    OSPILLD_Handle hOspi;
    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        OSPI_lld_setCmdExtType(hOspi,cmdExtType);
    }
}

int32_t OSPI_enableDacMode(OSPI_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status = OSPI_lld_enableDacMode(hOspi);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t OSPI_disableDacMode(OSPI_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status = OSPI_lld_disableDacMode(hOspi);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t OSPI_enablePhyPipeline(OSPI_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status = OSPI_lld_enablePhyPipeline(hOspi);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t OSPI_disablePhyPipeline(OSPI_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status = OSPI_lld_disablePhyPipeline(hOspi);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t OSPI_enablePhy(OSPI_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status = OSPI_lld_enablePhy(hOspi);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t OSPI_disablePhy(OSPI_Handle handle)
{
    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status = OSPI_lld_disablePhy(hOspi);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

uint32_t OSPI_getFlashDataBaseAddr(OSPI_Handle handle)
{
    uint32_t dataBaseAddr = 0U;

    if((OSPI_Handle) NULL != handle)
    {
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        dataBaseAddr = attrs->dataBaseAddr;
    }

    return dataBaseAddr;
}

/* Different OSPI Read functions */
int32_t OSPI_readCmd(OSPI_Handle handle, OSPI_ReadCmdParams *rdParams)
{
    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        if (0U == rdParams->readTimeout)
        {
            rdParams->readTimeout = OSPI_READ_WRITE_TIMEOUT;
        }
        status = OSPI_lld_readCmd(hOspi,rdParams);
    }
    else
    {
        status = SystemP_FAILURE;
    }


    return status;
}

int32_t OSPI_readDirect(OSPI_Handle handle, OSPI_Transaction *trans)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t dmaInterruptStatus = 0;
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        if (hOspi->hOspiInit->dmaEnable == OSPI_TRUE)
        {
            dmaInterruptStatus = OSPI_isDmaInterruptEnabled(hOspi);

            status = OSPI_lld_readDirectDma(hOspi, trans);

            if (dmaInterruptStatus == OSPI_TRUE && hOspi->currTrans->state == OSPI_TRANSFER_MODE_BLOCKING)
            {
                /* Pend the semaphore */
                (void) SemaphoreP_pend(&obj->transferSemObj, SystemP_WAIT_FOREVER);
            }

            if(hOspi->hOspiInit->phyEnable == OSPI_TRUE)
            {
                /* Disable PHY pipeline */
                status += OSPI_lld_disablePhyPipeline(hOspi);
            }
        }
        else
        {
            status += OSPI_lld_readDirect(hOspi, trans);
        }
        /* Switch to INDAC mode if DAC was initially in disabled state */
        if (FALSE == OSPI_isDacEnable(handle))
        {
            status += OSPI_disableDacMode(handle);
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t OSPI_readIndirect(OSPI_Handle handle, OSPI_Transaction *trans)
{
    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;

        status = OSPI_lld_readIndirect(hOspi, trans);

        if(OSPI_TRUE == hOspi->hOspiInit->intrEnable)
        {
            /* Pend the semaphore */
            (void) SemaphoreP_pend(&obj->transferSemObj, SystemP_WAIT_FOREVER);
        }

        /* Return to DAC mode if it was initially in enabled state */
        if (OSPI_TRUE == OSPI_isDacEnable(handle))
        {
            status = OSPI_enableDacMode(handle);
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }


    return status;
}

/* Different OSPI write functions */
int32_t OSPI_writeCmd(OSPI_Handle handle, OSPI_WriteCmdParams *wrParams)
{
    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        if (0U == wrParams->writeTimeout)
        {
            wrParams->writeTimeout = OSPI_READ_WRITE_TIMEOUT;
        }

        status = OSPI_lld_writeCmd(hOspi, wrParams);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t OSPI_writeDirect(OSPI_Handle handle, OSPI_Transaction *trans)
{
    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        const OSPI_Attrs *attrs = ((OSPI_Config *)handle)->attrs;
        hOspi = &obj->ospilldObject;

        status = OSPI_lld_writeDirect(hOspi, trans);

        CacheP_wbInv((void*)(attrs->dataBaseAddr + trans->addrOffset), trans->count, CacheP_TYPE_ALL);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t OSPI_writeIndirect(OSPI_Handle handle, OSPI_Transaction *trans)
{
    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;

        status += OSPI_lld_writeIndirect(hOspi, trans);

        if(OSPI_TRUE == hOspi->hOspiInit->intrEnable)
        {
            /* Pend the semaphore */
            (void) SemaphoreP_pend(&obj->transferSemObj, SystemP_WAIT_FOREVER);
        }

        /* Return to DAC mode if it was initially in enabled state */
        if (OSPI_TRUE == OSPI_isDacEnable(handle))
        {
            status += OSPI_enableDacMode(handle);
        }
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t OSPI_configResetPin(OSPI_Handle handle, uint32_t config)
{
    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status = OSPI_lld_configResetPin(hOspi,config);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t OSPI_setResetPinStatus(OSPI_Handle handle, uint32_t pinStatus)
{
    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;

    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status = OSPI_lld_setResetPinStatus(hOspi,pinStatus);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t OSPI_configBaudrate(OSPI_Handle handle, uint32_t baud)
{
    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;
    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status = OSPI_lld_configBaudrate(hOspi,baud);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t OSPI_readBaudRateDivFromReg(OSPI_Handle handle, uint32_t *baudDiv)
{
    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;
    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status = OSPI_lld_readBaudRateDivFromReg(hOspi,baudDiv);
    }
    else
    {
        status = SystemP_FAILURE;
    }
    return status;
}

int32_t OSPI_getBaudRateDivFromObj(OSPI_Handle handle, uint32_t *baudDiv)
{

    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;
    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status = OSPI_lld_readBaudRateDivFromReg(hOspi,baudDiv);
    }
    else
    {
        status = SystemP_FAILURE;
    }
    return status;
}

static void OSPI_dmaInterruptCallback(void* args)
{
    OSPILLD_Handle handle = (OSPILLD_Handle) args;
    OSPI_Object *obj    = ((OSPI_Config *)handle->args)->object;
    (void) SemaphoreP_post(&obj->transferSemObj);
}

static void OSPI_interruptCallback(void *args)
{
    OSPILLD_Handle handle = (OSPILLD_Handle)args;
    OSPI_Object *obj    = ((OSPI_Config *)handle->args)->object;
    (void) SemaphoreP_post(&obj->transferSemObj);
}

uint32_t OSPI_isValidateOtpEnable(OSPI_Handle handle)
{
    uint32_t retVal = 0U;
    OSPILLD_Handle hOspi;
    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        retVal = OSPI_lld_isValidateOtpEnable(hOspi);
    }
    return retVal;
}

int32_t OSPI_setFrequency(OSPI_Handle handle, uint64_t inputClkFreq)
{
    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;
    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status = OSPI_lld_setFrequency(hOspi, inputClkFreq);
    }
    else
    {
        status = SystemP_FAILURE;
    }
    return status;
}

int32_t OSPI_setDelays(OSPI_Handle handle, uint32_t inputClkFreq)
{
    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;
    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status = OSPI_lld_setDelays(hOspi, inputClkFreq);
    }
    else
    {
        status = SystemP_FAILURE;
    }
    return status;
}

int32_t OSPI_setBaudRateDiv(OSPI_Handle handle, uint32_t baudRateDiv)
{
    int32_t status = SystemP_SUCCESS;
    OSPILLD_Handle hOspi;
    if((OSPI_Handle) NULL != handle)
    {
        OSPI_Object *obj = ((OSPI_Config *)handle)->object;
        hOspi = &obj->ospilldObject;
        status = OSPI_lld_setBaudRateDiv(hOspi, baudRateDiv);
    }
    else
    {
        status = SystemP_FAILURE;
    }
    return status;
}
