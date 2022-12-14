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

/**
 *  \file mibspi_v0.c
 *
 *  \brief File containing MIBSPI Driver APIs implementation for version V0.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
/* This is needed for memset/memcpy */
#include <string.h>
#include <drivers/mibspi.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/mibspi/v0/edma/mibspi_edma.h>
#include <drivers/soc.h>
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/** \brief Macro to get the size of an array */
#define MIBSPI_UTILS_ARRAYSIZE(x)  (sizeof(x) / sizeof(x[0]))

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/** \brief Driver object for semaphore */
typedef struct
{
    void                   *lock;
    /**< Driver lock - to protect across open/close */
    SemaphoreP_Object       lockObj;
    /**< Driver lock object */
} MIBSPI_DrvObj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Driver internal functions */
static int32_t MIBSPI_validateParams(const MIBSPI_OpenParams *params);
static int32_t MIBSPI_validateTransferParams(const MIBSPI_Transaction *transaction,
                                             const MIBSPI_Object *ptrMibSpiDriver);
static void MIBSPI_enablePinSettings(CSL_mss_spiRegs  *ptrMibSpiReg,
                                     MIBSPI_PinMode pinMode,
                                     uint8_t chipSelectMask,
                                     uint32_t spiEnaPin);
static void MIBSPI_setControllerClockRate(CSL_mss_spiRegs *ptrMibSpiReg,
                                      uint32_t clockSrcFreq,
                                      uint32_t desiredSpiClock);
static void MIBSPI_setResetMode(CSL_mss_spiRegs *ptrMibSpiReg,
                                Bool reset);
static void MIBSPI_SPIEnable(CSL_mss_spiRegs *ptrMibSpiReg);
static void MIBSPI_SPIDisable(CSL_mss_spiRegs *ptrMibSpiReg);
static void MIBSPI_transferGroupEnable(CSL_mss_spiRegs *ptrMibSpiReg,
                                       uint8_t group);
static void MIBSPI_transferGroupDisable(CSL_mss_spiRegs *ptrMibSpiReg,
                                        uint8_t group);
static void MIBSPI_transferSetPStart(CSL_mss_spiRegs *ptrMibSpiReg,
                                     uint8_t group,
                                     uint8_t offset);
static uint32_t MIBSPI_checkTGComplete(CSL_mss_spiRegs *ptrMibSpiReg,
                                       uint8_t group);
static void MIBSPI_enableGroupInterrupt(CSL_mss_spiRegs *ptrMibSpiReg,
                                        uint8_t group,
                                        uint32_t intLine);
static void MIBSPI_disableGroupInterrupt(CSL_mss_spiRegs *ptrMibSpiReg,
                                         uint8_t group);
static void MIBSPI_dmaCtrlGroupConfig(CSL_mss_spiRegs *ptrMibSpiReg,
                                      uint16_t bufId,
                                      uint8_t iCount,
                                      uint8_t dmaCtrlGroup);
static void MIBSPI_dmaCtrlGroupStart(CSL_mss_spiRegs *ptrMibSpiReg,
                                     uint8_t dmaCtrlGroup,
                                     MIBSPI_DmaCtrlChType chType );
static void MIBSPI_dmaCtrlGroupDisable(CSL_mss_spiRegs *ptrMibSpiReg,
                                       uint8_t dmaCtrlGroup);
static void MIBSPI_initController(const MIBSPI_Attrs* ptrHwCfg,
                              const MIBSPI_OpenParams *params);
static void MIBSPI_initPeripheral(const MIBSPI_Attrs* ptrHwCfg,
                             const MIBSPI_OpenParams *params);
static void MIBSPI_writeDataRAM(MIBSPI_Object *ptrMibSpiDriver,
                                uint8_t group,
                                uint16_t *data,
                                uint16_t dataElem);
static uint32_t MIBSPI_readDataRAM(MIBSPI_Object *ptrMibSpiDriver,
                                   uint8_t group,
                                   uint16_t *data,
                                   uint16_t dataElem);
static void MIBSPI_ISR(void *args);
static void MIBSPI_dataTransfer(MIBSPI_Object *ptrMibSpiDriver,
                                uint8_t *srcData,
                                uint8_t *dstData,
                                uint16_t dataElemSize,
                                uint8_t group
                                );
static int32_t MIBSPI_openPeripheralMode(MIBSPI_Object *ptrMibSpiDriver,
                                    const MIBSPI_Attrs* ptrHwCfg,
                                    const MIBSPI_OpenParams *params);
static int32_t MIBSPI_openControllerMode(MIBSPI_Object *ptrMibSpiDriver,
                                    const MIBSPI_Attrs* ptrHwCfg,
                                    const MIBSPI_OpenParams *params);
static int32_t MIBSPI_closeCore(MIBSPI_Handle handle);
static void MIBSPI_initTransactionState(MIBSPI_TransactionState *transactionState,
                                        MIBSPI_Transaction     *transaction);
static void MIBSPI_resetTransactionState(MIBSPI_TransactionState *transactionState);
static int32_t MIBSPI_transferCore(MIBSPI_Handle handle,
                                   MIBSPI_Transaction *transaction);
static void MIBSPI_transferCancelCore(MIBSPI_Handle handle);
static uint32_t MIBSPI_getPhase(MIBSPI_FrameFormat frameFormat);
static uint32_t MIBSPI_getPolarity(MIBSPI_FrameFormat frameFormat);
static void MIBSPI_enableErrorInterrupt(CSL_mss_spiRegs  *ptrMibSpiReg,
                                        uint32_t enableFlag);
static void MIBSPI_setErrorInterruptLevel(CSL_mss_spiRegs  *ptrMibSpiReg,
                                          uint32_t level);

/* ========================================================================== */
/*               Global Variables and default intialization                    */
/* ========================================================================== */

/** \brief Driver object */
static MIBSPI_DrvObj     gMibspiDrvObj =
{
    .lock           = NULL,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
void MIBSPI_init(void)
{
    int32_t          status;
    uint32_t         cnt;
    MIBSPI_Object   *obj;

    /* Init each driver instance object */
    for (cnt = 0U; cnt < gMibspiConfigNum; cnt++)
    {
        /* initialize object varibles */
        obj = gMibspiConfig[cnt].object;
        DebugP_assert(NULL != obj);
        memset(obj, 0, sizeof(MIBSPI_Object));
    }

    /*
     * Create driver lock
     * Construct thread safe handles for SPI driver level
     * Semaphore to provide exclusive access to the SPI APIs
     */
    status = SemaphoreP_constructMutex(&gMibspiDrvObj.lockObj);
    if(SystemP_SUCCESS == status)
    {
        gMibspiDrvObj.lock = &gMibspiDrvObj.lockObj;
    }

    return;
}

void MIBSPI_deinit(void)
{
    /* Delete driver lock */
    if(NULL != gMibspiDrvObj.lock)
    {
        SemaphoreP_destruct(&gMibspiDrvObj.lockObj);
        gMibspiDrvObj.lock = NULL;
    }

    return;
}

MIBSPI_Handle MIBSPI_open(uint32_t index, MIBSPI_OpenParams *params)
{
    int32_t                     status = SystemP_SUCCESS;
    MIBSPI_Handle               handle = NULL;
    MIBSPI_Config              *ptrSPIConfig = NULL;
    const MIBSPI_Attrs         *ptrHwCfg     = NULL;
    MIBSPI_Object              *ptrMibSpiDriver = NULL;
    HwiP_Params                 hwiPrms;

    /* Check index */
    if(index >= gMibspiConfigNum)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        ptrSPIConfig = &gMibspiConfig[index];
    }

    DebugP_assert(NULL != gMibspiDrvObj.lock);
    SemaphoreP_pend(&gMibspiDrvObj.lockObj, SystemP_WAIT_FOREVER);

    if(SystemP_SUCCESS == status)
    {
        /* Get the hardware configuration: */
        ptrHwCfg = ptrSPIConfig->attrs;

        /* Validate hardware configuration */
        DebugP_assert(ptrHwCfg != NULL);
        DebugP_assert(ptrHwCfg->ptrSpiRegBase != NULL);
        DebugP_assert(ptrHwCfg->ptrMibSpiRam != NULL);

        /* Validate the driver object configuration */
        DebugP_assert (ptrSPIConfig->object != NULL);
        ptrMibSpiDriver = ptrSPIConfig->object;

        /* Check instance was not previously opened */
        if(TRUE == ptrMibSpiDriver->isOpen)
        {
            /* Handle already opened */
            status = SystemP_FAILURE;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        /* Initialize the memory */
        memset ((void *)ptrMibSpiDriver, 0, sizeof(MIBSPI_Object));
        if(NULL != params)
        {
            /* copy params into the driver object structure */
            memcpy(&ptrMibSpiDriver->params, params, sizeof(MIBSPI_OpenParams));
        }
        else
        {
            /* Init with default if NULL is passed */
            MIBSPI_Params_init(&ptrMibSpiDriver->params);
        }

        /* If DMA is enabled, open the EDMA handle */
        if(TRUE == ptrMibSpiDriver->params.dmaEnable)
        {
            /* open the DMA channel */
            status = MIBSPI_edmaOpen((MIBSPI_Handle)ptrSPIConfig, ptrMibSpiDriver->params.edmaInst);
        }
        else
        {
            ptrMibSpiDriver->params.dmaHandle = NULL;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        /* Validate params for SPI driver */
        status = MIBSPI_validateParams(&ptrMibSpiDriver->params);
    }

    if(SystemP_SUCCESS == status)
    {
        /* Save the Driver handle and Hw config */
        ptrMibSpiDriver->ptrHwCfg = ptrHwCfg;
        /* Init state */
        ptrMibSpiDriver->mibspiHandle = (MIBSPI_Handle)ptrSPIConfig;
        ptrMibSpiDriver->txScratchBuffer = ptrMibSpiDriver->params.txDummyValue;
        MIBSPI_resetTransactionState(&ptrMibSpiDriver->transactionState);
        /* Call open function for the MibSPI operating mode */
        if(ptrMibSpiDriver->params.mode == MIBSPI_PERIPHERAL)
        {
            status = MIBSPI_openPeripheralMode(ptrMibSpiDriver, ptrHwCfg, &ptrMibSpiDriver->params);
        }
        else
        {
            status = MIBSPI_openControllerMode(ptrMibSpiDriver, ptrHwCfg, &ptrMibSpiDriver->params);
        }
    }

    if(SystemP_SUCCESS == status)
    {

        /* Create a binary semaphore which is used to handle the Blocking operation. */
        /* Create transfer sync semaphore */
        status = SemaphoreP_constructBinary(&ptrMibSpiDriver->transferSemObj, 0U);

        if(SystemP_SUCCESS == status)
        {
            ptrMibSpiDriver->transferSem = &ptrMibSpiDriver->transferSemObj;
        }


        /* Register SPI Interrupt handling ISR */
        /* Initialize with defaults */
        HwiP_Params_init(&hwiPrms);

        /* Populate the interrupt parameters */
        hwiPrms.intNum         = ptrHwCfg->interrupt1Num;
        hwiPrms.priority       = 0x1U;
        hwiPrms.callback       = &MIBSPI_ISR;
        hwiPrms.args           = (void *)    ptrMibSpiDriver;

        /* Register interrupts */
        status += HwiP_construct(&ptrMibSpiDriver->hwiObj, &hwiPrms);;

        if(SystemP_SUCCESS == status)
        {
            ptrMibSpiDriver->hwiHandle = &ptrMibSpiDriver->hwiObj;
        }
    }

    if(SystemP_SUCCESS == status)
    {
        ptrMibSpiDriver->isOpen = TRUE;
        handle = (MIBSPI_Handle) ptrSPIConfig;
    }

    /* Release the the SPI driver semaphore */
    SemaphoreP_post(&gMibspiDrvObj.lockObj);

     /* Free-up resources in case of error */
    if(SystemP_SUCCESS != status)
    {
        if(NULL != ptrSPIConfig)
        {
            MIBSPI_close((MIBSPI_Handle) ptrSPIConfig);
        }
    }

    return handle;
}

void MIBSPI_close(MIBSPI_Handle handle)
{
    int32_t status = SystemP_FAILURE;

    if (handle != NULL)
    {
        DebugP_assert(NULL != gMibspiDrvObj.lock);

        /* Acquire the the SPI driver semaphore */
        SemaphoreP_pend(&gMibspiDrvObj.lockObj, SystemP_WAIT_FOREVER);

        status = MIBSPI_closeCore(handle);
        DebugP_assert(status == SystemP_SUCCESS);

        /* Release the the SPI driver semaphore */
        SemaphoreP_post(&gMibspiDrvObj.lockObj);

    }
}

int32_t MIBSPI_transfer(MIBSPI_Handle handle, MIBSPI_Transaction *transaction)
{
    int32_t status = SystemP_FAILURE;

    if ((handle != NULL) && (transaction != NULL))
    {
        status = MIBSPI_transferCore(handle, transaction);
    }
    return status;
}

void MIBSPI_transferCancel(MIBSPI_Handle handle)
{
    if (handle != NULL)
    {
        MIBSPI_transferCancelCore(handle);
    }
}

int32_t MIBSPI_enableLoopback(MIBSPI_Handle handle, MIBSPI_LoopBackType loopbacktype)
{
    MIBSPI_Config           *config;
    const MIBSPI_Attrs      *attr;
    CSL_mss_spiRegs         *ptrMibSpiReg;
    uint32_t                 regVal;
    int32_t                  status = SystemP_SUCCESS;

    if(NULL == handle)
    {
        status = SystemP_FAILURE;
    }
    else
    {
       config              = (MIBSPI_Config *) handle;
       attr                = config->attrs;
       ptrMibSpiReg        = attr->ptrSpiRegBase;
    }

    if(SystemP_SUCCESS == status)
    {
        /* Sanity check the input parameters */
        if ((loopbacktype == MIBSPI_LOOPBK_DIGITAL) &&
            (CSL_FEXT(ptrMibSpiReg->SPIGCR1,SPI_SPIGCR1_MASTER) == CSL_SPI_SPIGCR1_MASTER_SLAVE))
        {
            /* Digital mode Loopback is not supported in Peripheral mode */
            status = SystemP_FAILURE;
        }

        if(SystemP_SUCCESS == status)
        {
            if((loopbacktype == MIBSPI_LOOPBK_DIGITAL) || (loopbacktype == MIBSPI_LOOPBK_ANALOG))
            {
                /* Put MibSpi module in reset */
                CSL_FINS(ptrMibSpiReg->SPIGCR1,SPI_SPIGCR1_SPIEN, 0U);

                regVal = ptrMibSpiReg->IOLPBKTSTCR;
                /* Set Loopback either in Analog or Digital Mode */
                CSL_FINS(regVal,SPI_IOLPBKTSTCR_LPBKTYPE, loopbacktype);

                /* For IOLPBK in SLave mode, set loopback enable in MIBSPI_transaction as it immediately trigger TG0 */
                if ((loopbacktype == MIBSPI_LOOPBK_ANALOG)
                    &&
                    (CSL_FEXT(ptrMibSpiReg->SPIGCR1,SPI_SPIGCR1_MASTER) == CSL_SPI_SPIGCR1_MASTER_SLAVE))
                {
                    CSL_FINS(regVal,SPI_IOLPBKTSTCR_IOLPBKTSTENA, 0x5U);
                }
                else
                {
                    /* Enable Loopback  */
                    CSL_FINS(regVal,SPI_IOLPBKTSTCR_IOLPBKTSTENA, 0xAU);
                }
                if (loopbacktype == MIBSPI_LOOPBK_ANALOG)
                {
                    CSL_FINS(regVal,SPI_IOLPBKTSTCR_RXPENA, 0x1U);
                }

                ptrMibSpiReg->IOLPBKTSTCR = regVal;

                /* Restart MIBSPI1 */
                regVal = ptrMibSpiReg->SPIGCR1;
                CSL_FINS(regVal,SPI_SPIGCR1_SPIEN, 1U);

                if (CSL_FEXT(ptrMibSpiReg->SPIGCR1,SPI_SPIGCR1_MASTER) == CSL_SPI_SPIGCR1_MASTER_MASTER)
                {
                    CSL_FINS(regVal,SPI_SPIGCR1_LOOPBACK, 1U);
                }
                ptrMibSpiReg->SPIGCR1 = regVal;
            }
        }
    }

    return status;
}

int32_t MIBSPI_disableLoopback(MIBSPI_Handle handle)
{
    MIBSPI_Config           *config;
    const MIBSPI_Attrs      *attr;
    CSL_mss_spiRegs         *ptrMibSpiReg;
    int32_t                  status = SystemP_SUCCESS;

    if(NULL == handle)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        config              = (MIBSPI_Config *) handle;
        attr                = config->attrs;
        ptrMibSpiReg        = attr->ptrSpiRegBase;
    }

    if(SystemP_SUCCESS == status)
    {
        /* Put MibSpi module in reset */
        CSL_FINS(ptrMibSpiReg->SPIGCR1,SPI_SPIGCR1_SPIEN, 0U);

        /* Disable Loopback either in Analog or Digital Mode */
        CSL_FINS(ptrMibSpiReg->IOLPBKTSTCR,SPI_IOLPBKTSTCR_IOLPBKTSTENA, 0x5U);

        /* Restart MIBSPI1 */
        CSL_FINS(ptrMibSpiReg->SPIGCR1,SPI_SPIGCR1_SPIEN, 1U);
    }

    return status;
}

int32_t MIBSPI_setClockPhasePolarity(MIBSPI_Handle handle,
                                         uint8_t clockFmt)
{
    MIBSPI_Config           *config;
    const MIBSPI_Attrs      *attr;
    CSL_mss_spiRegs         *ptrMibSpiReg;
    int32_t                  status = SystemP_SUCCESS;
    uint32_t                 regVal;
    uint8_t                  frameFmt;

    if(NULL == handle)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        config              = (MIBSPI_Config *) handle;
        attr                = config->attrs;
        ptrMibSpiReg        = attr->ptrSpiRegBase;
    }

    if(SystemP_SUCCESS == status)
    {
        /* 2 bit setting */
        frameFmt = clockFmt & 0x3U;

        /* Put MibSpi module in reset */
        CSL_FINS(ptrMibSpiReg->SPIGCR1,SPI_SPIGCR1_SPIEN, 0U);

        /* Set MibSpi Peripheral functional Mode Clock/polarity */
        regVal = ptrMibSpiReg->SPIFMT[0];
        CSL_FINS(regVal, SPI_SPIFMT_PHASE, MIBSPI_getPhase((MIBSPI_FrameFormat)frameFmt));   /* PHASE */
        CSL_FINS(regVal, SPI_SPIFMT_POLARITY, MIBSPI_getPolarity((MIBSPI_FrameFormat)frameFmt));   /* POLARITY */
        ptrMibSpiReg->SPIFMT[0] = regVal;

        /* Finally start MIBSPI */
        CSL_FINS(ptrMibSpiReg->SPIGCR1,SPI_SPIGCR1_SPIEN, 1U);
    }

    return status;
}

int32_t MIBSPI_getDrvStats(MIBSPI_Handle handle, MIBSPI_Stats *ptrStats)
{
    MIBSPI_Config      *config;
    MIBSPI_Object      *ptrMibSpiDriver;
    int32_t             status = SystemP_SUCCESS;

    /* Validate input parameters */
    if((handle == NULL) || (ptrStats == NULL))
    {
        status = SystemP_FAILURE;
    }
    else
    {
        config = (MIBSPI_Config *) handle;
        /* Get the MibSpi driver handle */
        ptrMibSpiDriver = config->object;

        ptrStats->dlenErr = ptrMibSpiDriver->hwStats.dlenErr;
        ptrStats->timeout = ptrMibSpiDriver->hwStats.timeout;
        ptrStats->parErr = ptrMibSpiDriver->hwStats.parErr;
        ptrStats->desync = ptrMibSpiDriver->hwStats.desync;
        ptrStats->bitErr = ptrMibSpiDriver->hwStats.bitErr;
        ptrStats->rxOvrnErr =  ptrMibSpiDriver->hwStats.rxOvrnErr;
    }

    return status;
}

/* ========================================================================== */
/*                          Static Function Definitions                       */
/* ========================================================================== */
static int32_t MIBSPI_validateParams(const MIBSPI_OpenParams *params)
{
    int32_t             status = SystemP_SUCCESS;
    uint8_t             index;
    uint8_t             ramBufOffset = 0U;


    /* Validate dataSize, only 8bits and 16bits are supported */
    if( (params->dataSize != 8U) && (params->dataSize != 16U))
    {
        status = SystemP_FAILURE;

    }

    if(SystemP_SUCCESS == status)
    {
        /* Validate bitRate */
        if(params->mode == MIBSPI_CONTROLLER)
        {
            if(params->u.controllerParams.bitRate == 0U)
            {
                status = SystemP_FAILURE;

            }

            if( (params->u.controllerParams.numPeripherals == 0U) || (params->u.controllerParams.numPeripherals > MIBSPI_PERIPHERAL_MAX) )
            {
                status = SystemP_FAILURE;

            }


            if(SystemP_SUCCESS == status)
            {
                /* Validate peripheral profile configuraiton */
                for(index = 0; index < params->u.controllerParams.numPeripherals; index++)
                {
                    const MIBSPI_PeripheralProfile    *ptrPeripheralProf;

                    /* Get the pointer to the peripheral profile */
                    ptrPeripheralProf = &params->u.controllerParams.peripheralProf[index];

                    /* Validate CS signal number */
                    if(ptrPeripheralProf->chipSelect >= MIBSPI_MAX_CS)
                    {
                        status = SystemP_FAILURE;
                        break;
                    }

                    if(ptrPeripheralProf->ramBufLen > MIBSPI_RAM_MAX_ELEM)
                    {
                        status = SystemP_FAILURE;
                        break;
                    }

                    ramBufOffset += ptrPeripheralProf->ramBufLen;

                }

                /* Validate total RAM Elements exceed the size of MibSPI RAM */
                if(ramBufOffset > MIBSPI_RAM_MAX_ELEM)
                {
                    status = SystemP_FAILURE;
                }
            }
        }
        else
        {
            /* Validate CS signal number */
            if(params->u.peripheralParams.chipSelect >= MIBSPI_MAX_CS)
            {
                status = SystemP_FAILURE;
            }

            /* Validate transfer mode. */
            if(params->transferMode == MIBSPI_MODE_CALLBACK)
            {
                if(params->transferCallbackFxn == NULL)
                {
                    status = SystemP_FAILURE;
                }
            }
        }
    }

    return status;
}

static int32_t MIBSPI_validateTransferParams(const MIBSPI_Transaction *transaction,
                                             const MIBSPI_Object *ptrMibSpiDriver)
{
    const MIBSPI_Attrs      *hwAttrs;
    int32_t                  status = SystemP_SUCCESS;
    uintptr_t                key;

    hwAttrs = ptrMibSpiDriver->ptrHwCfg;

    /* Check the transaction arguments */
    if (transaction->count == 0U)
    {
        status = SystemP_FAILURE;
    }

    /* Sanity check of parameters */
    /* Both Tx and Rx cannot be dummy transaction */
    if((transaction->txBuf == NULL) && (transaction->rxBuf == NULL))
    {
        status = SystemP_FAILURE;
    }

    if((transaction->peripheralIndex >= MIBSPI_PERIPHERAL_MAX)
       ||
       (transaction->peripheralIndex >= hwAttrs->numDmaReqLines))
    {
        status = SystemP_FAILURE;
    }

    if(ptrMibSpiDriver->params.dataSize == 16U)
    {
        if(transaction->count % 2U != 0)
        {
            status = SystemP_FAILURE;
        }
    }

    /* if Multi iCount support is not defined */
    if(ptrMibSpiDriver->params.iCountSupport == FALSE)
    {
        {
            uint16_t   dataLength;

            if(ptrMibSpiDriver->params.dataSize == 16U)
            {
                /* MIBSPI reads in 2 bytes format */
                dataLength = (uint16_t)transaction->count >> 1U;
            }
            else
            {
                dataLength = transaction->count;
            }

            /* Check data elements */
            if(dataLength > hwAttrs->mibspiRamSize)
            {
                status = SystemP_FAILURE;
            }
        }
    }

    /* Check if a transfer is in progress */
    key = HwiP_disable();

    if ( ptrMibSpiDriver->transactionState.transaction != NULL)
    {
        status = SystemP_FAILURE;
    }

    /* Transfer is in progress */
    HwiP_restore(key);
    return status;
}

static void MIBSPI_enablePinSettings(CSL_mss_spiRegs  *ptrMibSpiReg,
                                     MIBSPI_PinMode pinMode,
                                     uint8_t chipSelectMask,
                                     uint32_t spiEnaPin)
{
    uint32_t regVal = 0;

    switch(pinMode)
    {
        /* 3pin setting, enable CLK, SIMO, SOMI */
        case MIBSPI_PINMODE_3PIN:
            /* SPIPC0 Register: Set Port pins to functional */
            CSL_FINS(regVal, SPI_SPIPC0_CLKFUN, 1U);
            CSL_FINS(regVal, SPI_SPIPC0_SIMOFUN0, 1U);
            CSL_FINS(regVal, SPI_SPIPC0_SOMIFUN0, 1U);
            ptrMibSpiReg->SPIPC0 = regVal;
            break;

        /* 4pin with CS setting, enable CLK, SIMO, SOMI and CSx */
        case MIBSPI_PINMODE_4PIN_CS:
            /* SPIPC0 Register: Set Port pins to functional */
            CSL_FINS(regVal, SPI_SPIPC0_SCSFUN, (uint32_t)chipSelectMask);
            CSL_FINS(regVal, SPI_SPIPC0_CLKFUN, 1U);
            CSL_FINS(regVal, SPI_SPIPC0_SIMOFUN0, 1U);
            CSL_FINS(regVal, SPI_SPIPC0_SOMIFUN0, 1U);
            /* SPIENA pin Enable */
            if(MIBSPI_FEATURE_SPIENA_PIN_ENA == spiEnaPin)
            {
                CSL_FINS(regVal, SPI_SPIPC0_ENAFUN, 1U);
            }

            ptrMibSpiReg->SPIPC0 = regVal;         /* enable SOMI */
            break;

        default:
            DebugP_assert(0);
            break;
    }

    /*  SPIPC7 Register: Set Port Pullup/Pulldown control: 0 to enable, 1 to disable  */
    regVal = ptrMibSpiReg->SPIPC7;
    CSL_FINS(regVal, SPI_SPIPC7_SOMIPDIS0, 0);
    CSL_FINS(regVal, SPI_SPIPC7_SIMOPDIS0, 0);
    CSL_FINS(regVal, SPI_SPIPC7_CLKPDIS, 0);
    CSL_FINS(regVal, SPI_SPIPC7_ENAPDIS, 0);
    CSL_FINS(regVal, SPI_SPIPC7_SCSPDIS, 0);
    ptrMibSpiReg->SPIPC7 = regVal;

    /*  SPIPC8 Register: Set Port Pullup/Pulldown value: 0 to pulldown, 1 to pullup  */
    regVal = ptrMibSpiReg->SPIPC8;
    CSL_FINS(regVal, SPI_SPIPC8_SCSPSEL, 0x1U);
    CSL_FINS(regVal, SPI_SPIPC8_ENAPSEL, 1U);
    CSL_FINS(regVal, SPI_SPIPC8_CLKPSEL, 1U);
    CSL_FINS(regVal, SPI_SPIPC8_SIMOPSEL0, 1U);
    CSL_FINS(regVal, SPI_SPIPC8_SOMIPSEL0, 1U);
    ptrMibSpiReg->SPIPC8 = regVal;

    return;
}

static void MIBSPI_setControllerClockRate(CSL_mss_spiRegs *ptrMibSpiReg,
                                      uint32_t clockSrcFreq,
                                      uint32_t desiredSpiClock)
{
    uint8_t     clockDivisor;

    clockDivisor = (uint8_t)(clockSrcFreq / desiredSpiClock);

    /* Put MibSpi module in reset */
    CSL_FINS(ptrMibSpiReg->SPIGCR1, SPI_SPIGCR1_SPIEN, 0U);

    /* Set MibSpi clockDivisor */
    CSL_FINS(ptrMibSpiReg->SPIFMT[0],SPI_SPIFMT_PRESCALE, (uint32_t)((uint32_t)clockDivisor - 1U));

    /* Finally start MIBSPI1 */
    CSL_FINS(ptrMibSpiReg->SPIGCR1, SPI_SPIGCR1_SPIEN, 1U);

}

static void MIBSPI_setResetMode(CSL_mss_spiRegs *ptrMibSpiReg, Bool reset)
{
    if(reset == TRUE)
    {
        /* Set MibSpi in Reset mode */
        CSL_FINS(ptrMibSpiReg->SPIGCR0, SPI_SPIGCR0_NRESET, 0U);
    }
    else
    {

        /* Bring MibSpi out of Reset mode */
        CSL_FINS(ptrMibSpiReg->SPIGCR0, SPI_SPIGCR0_NRESET, 1U);
    }
}

static void MIBSPI_SPIEnable(CSL_mss_spiRegs *ptrMibSpiReg)
{
    /* Acitvate SPI */
    CSL_FINS(ptrMibSpiReg->SPIGCR1, SPI_SPIGCR1_ENABLE, 1U);

    /* Enable MibSpi multibuffered mode and enable buffer RAM */
    CSL_FINS(ptrMibSpiReg->MIBSPIE, SPI_MIBSPIE_MSPIENA, 1U);
}


static void MIBSPI_SPIDisable(CSL_mss_spiRegs *ptrMibSpiReg)
{
    /* Disable MibSpi multibuffered mode and enable buffer RAM */
    CSL_FINS(ptrMibSpiReg->MIBSPIE, SPI_MIBSPIE_MSPIENA, 0U);

    /* De-acitvate SPI */
    CSL_FINS(ptrMibSpiReg->SPIGCR1, SPI_SPIGCR1_SPIEN, 0U);
}

static void MIBSPI_transferGroupEnable(CSL_mss_spiRegs *ptrMibSpiReg,
                                       uint8_t group)
{
    /* Enable Transfer group */
    CSL_FINS(ptrMibSpiReg->TGCTRL[group],SPI_TGCTRL_TGENA, 1U);
}

static void MIBSPI_transferGroupDisable(CSL_mss_spiRegs *ptrMibSpiReg,
                                        uint8_t group)
{
    /* Disable Transfer group */
    CSL_FINS(ptrMibSpiReg->TGCTRL[group],SPI_TGCTRL_TGENA, 0U);

    /* Transfer is completed , disable SPI */
    MIBSPI_SPIDisable(ptrMibSpiReg);
}

static void MIBSPI_transferSetPStart(CSL_mss_spiRegs *ptrMibSpiReg,
                                     uint8_t group,
                                     uint8_t offset)
{
    CSL_FINS(ptrMibSpiReg->TGCTRL[group],SPI_TGCTRL_PSTART, (uint32_t)offset);
}

static uint32_t MIBSPI_checkTGComplete(CSL_mss_spiRegs *ptrMibSpiReg,
                                       uint8_t group)
{
    uint32_t    status = 0U;
    uint32_t    groupMask;
    uint32_t    intFlagReady = 0U;

    /* Get the bit mask of the group */
    groupMask = (uint32_t)1U << group;

    /* Read TGINT flag */
    intFlagReady = CSL_FEXT(ptrMibSpiReg->TGINTFLAG,SPI_TGINTFLAG_INTFLGRDY);
    if ( intFlagReady & groupMask)
    {
        /* Transfer finished, clear the corresponding flag */
        CSL_FINS(ptrMibSpiReg->TGINTFLAG,SPI_TGINTFLAG_INTFLGRDY, groupMask);
        status = 1U;
    }

    return (status);
}

static void MIBSPI_enableGroupInterrupt(CSL_mss_spiRegs *ptrMibSpiReg,
                                        uint8_t group,
                                        uint32_t intLine)
{
    if(intLine == 0)
    {
        /* INT0 interrupt */
        CSL_FINS(ptrMibSpiReg->TGITLVCR,SPI_TGITLVCR_CLRINTLVLRDY, (uint32_t)1U << group);
    }
    else
    {
        /* INT1 interrupt */
        CSL_FINS(ptrMibSpiReg->TGITLVST,SPI_TGITLVST_SETINTLVLRDY, (uint32_t)1U << group);
    }

    /* Enable interrupt */
    CSL_FINS(ptrMibSpiReg->TGITENST,SPI_TGITENST_SETINTENRDY, (uint32_t)1U << group);
}

static void MIBSPI_disableGroupInterrupt(CSL_mss_spiRegs *ptrMibSpiReg, uint8_t group)
{
    /* Disable interrupt */
    CSL_FINS(ptrMibSpiReg->TGITENCR,SPI_TGITENCR_CLRINTENRDY, (uint32_t)1U << group);
}

static void MIBSPI_dmaCtrlGroupConfig(CSL_mss_spiRegs *ptrMibSpiReg,
                                      uint16_t bufId,
                                      uint8_t iCount,
                                      uint8_t dmaCtrlGroup)
{
    uint32_t regVal = 0;

    /* Setting Transmit channel DMA request */
    CSL_FINS(regVal,SPI_DMACTRL_TXDMA_MAP, (uint32_t)dmaCtrlGroup * 2U);
    CSL_FINS(regVal,SPI_DMACTRL_TXDMAENA, 0U);
    CSL_FINS(regVal,SPI_DMACTRL_BUFID, (uint32_t)bufId);
    CSL_FINS(regVal,SPI_DMACTRL_ICOUNT, (uint32_t)iCount);
    CSL_FINS(regVal,SPI_DMACTRL_ONESHOT, 1U);

    /* Setting Receive channel DMA request */
    CSL_FINS(regVal,SPI_DMACTRL_RXDMA_MAP, (uint32_t)dmaCtrlGroup * 2U + 1);
    CSL_FINS(regVal,SPI_DMACTRL_RXDMAENA, 0U);
    CSL_FINS(regVal,SPI_DMACTRL_BUFID, (uint32_t)bufId);
    CSL_FINS(regVal,SPI_DMACTRL_ICOUNT, (uint32_t)iCount);
    CSL_FINS(regVal,SPI_DMACTRL_ONESHOT, 1U);

    ptrMibSpiReg->DMACTRL[dmaCtrlGroup] = regVal;
    return;
}

static void MIBSPI_dmaCtrlGroupStart(CSL_mss_spiRegs *ptrMibSpiReg,
                                     uint8_t dmaCtrlGroup,
                                     MIBSPI_DmaCtrlChType chType )
{
    uint32_t regVal = ptrMibSpiReg->DMACTRL[dmaCtrlGroup];

    if ((chType == MIBSPI_DMACTRL_CH_TX)
        ||
        (chType == MIBSPI_DMACTRL_CH_BOTH))
    {
        /* Setting Transmit channel DMA request */
        CSL_FINS(regVal,SPI_DMACTRL_TXDMAENA, 1U);
    }
    if ((chType == MIBSPI_DMACTRL_CH_RX)
        ||
        (chType == MIBSPI_DMACTRL_CH_BOTH))
    {
        /* Setting Receive channel DMA request */
        CSL_FINS(regVal,SPI_DMACTRL_RXDMAENA, 1U);
    }
    ptrMibSpiReg->DMACTRL[dmaCtrlGroup] = regVal;
    return;
}

static void MIBSPI_dmaCtrlGroupDisable(CSL_mss_spiRegs *ptrMibSpiReg,
                                       uint8_t dmaCtrlGroup)
{
    /* Get MibSpi Register & Ram Base address */
    uint32_t regVal = ptrMibSpiReg->DMACTRL[dmaCtrlGroup];

    CSL_FINS(regVal, SPI_DMACTRL_ONESHOT, 0U);
    CSL_FINS(regVal, SPI_DMACTRL_BUFID, 0U);
    CSL_FINS(regVal, SPI_DMACTRL_RXDMA_MAP, 0U);
    CSL_FINS(regVal, SPI_DMACTRL_TXDMA_MAP, 0U);
    CSL_FINS(regVal, SPI_DMACTRL_RXDMAENA, 0U);
    CSL_FINS(regVal, SPI_DMACTRL_TXDMAENA, 0U);
    CSL_FINS(regVal, SPI_DMACTRL_NOBRK, 0U);
    CSL_FINS(regVal, SPI_DMACTRL_ICOUNT, 0U);
    CSL_FINS(regVal, SPI_DMACTRL_BUFID7, 0U);
    ptrMibSpiReg->DMACTRL[dmaCtrlGroup] = regVal;

}

static void MIBSPI_initController(const MIBSPI_Attrs* ptrHwCfg,
                              const MIBSPI_OpenParams * params)
{
    CSL_mibspiRam           *ptrMibSpiRam;
    CSL_mss_spiRegs         *ptrMibSpiReg;
    uint32_t                 flag;
    uint32_t                 index;
    uint8_t                  csnr = 0;
    uint8_t                  chipSelectMask;
    uint8_t                  ramBufOffset = 0;
    uint32_t                 regVal;

    /* Get Register and RAM base */
    ptrMibSpiRam  = ptrHwCfg->ptrMibSpiRam;
    ptrMibSpiReg  = ptrHwCfg->ptrSpiRegBase;

    /* Bring MIBSPI out of reset */
    ptrMibSpiReg->SPIGCR0 = 0U;
    ptrMibSpiReg->SPIGCR0 = 1U;

    /* Enable MibSpi multibuffered mode and enable buffer RAM */
    CSL_FINS(ptrMibSpiReg->MIBSPIE,SPI_MIBSPIE_MSPIENA, 1U);

    /* Wait for buffer initialization complete before accessing MibSPI Ram */
    while(1)
    {
        flag = CSL_FEXT(ptrMibSpiReg->SPIFLG,SPI_SPIFLG_BUFINITACTIVE);
        if(flag == 0)
        {
            break;
        }
    }

    MIBSPI_socMemInit(ptrHwCfg->mibspiInstId);
    /* Set MibSpi controller mode and clock configuration */
    regVal = ptrMibSpiReg->SPIGCR1;
    CSL_FINS(regVal,SPI_SPIGCR1_MASTER, CSL_SPI_SPIGCR1_MASTER_MASTER);
    CSL_FINS(regVal,SPI_SPIGCR1_CLKMOD, CSL_SPI_SPIGCR1_CLKMOD_INTERNAL);
    ptrMibSpiReg->SPIGCR1 = regVal;

    /* SPIENV pin pulled high when not active */
    CSL_FINS(ptrMibSpiReg->SPIINT0,SPI_SPIINT0_ENABLEHIGHZ, 0U);

    /* Delays Configuration: this controller only configuraion, hence set it to all zeros */
    if( (params->u.controllerParams.c2tDelay != 0U) ||
       (params->u.controllerParams.t2cDelay != 0U) )
    {
        regVal = ptrMibSpiReg->SPIDELAY;
        CSL_FINS(regVal, SPI_SPIDELAY_C2TDELAY, (uint32_t)params->u.controllerParams.c2tDelay);
        CSL_FINS(regVal, SPI_SPIDELAY_T2CDELAY, (uint32_t)params->u.controllerParams.t2cDelay);
        ptrMibSpiReg->SPIDELAY = regVal;
    }
    else
    {
        ptrMibSpiReg->SPIDELAY = 0x0U;
    }

    /* Set Data Format 0 */
    regVal = ptrMibSpiReg->SPIFMT[0];
    CSL_FINS(regVal, SPI_SPIFMT_PHASE, MIBSPI_getPhase(params->frameFormat));   /* PHASE */
    CSL_FINS(regVal, SPI_SPIFMT_POLARITY, MIBSPI_getPolarity(params->frameFormat));   /* POLARITY */
    CSL_FINS(regVal, SPI_SPIFMT_WDELAY, (uint32_t) params->u.controllerParams.wDelay);  /* WDELAY */
    CSL_FINS(regVal, SPI_SPIFMT_PRESCALE,  0U);                   /* PRESCALE */
    CSL_FINS(regVal, SPI_SPIFMT_SHIFTDIR, (uint32_t) params->shiftFormat);  /* SHIFTDIR */
    CSL_FINS(regVal, SPI_SPIFMT_CHARLEN, (uint32_t) params->dataSize);        /* CHARlEN */
    ptrMibSpiReg->SPIFMT[0] = regVal;

    /* Set Data Format 1 */
    regVal = ptrMibSpiReg->SPIFMT[1];
    CSL_FINS(regVal, SPI_SPIFMT_PRESCALE, 18U);                 /* PRESCALE */
    CSL_FINS(regVal, SPI_SPIFMT_CHARLEN, (uint32_t) params->dataSize);  /* CHARlEN */
    ptrMibSpiReg->SPIFMT[1] = regVal;

    /* Set Data Format 2 */
    regVal = ptrMibSpiReg->SPIFMT[2];
    CSL_FINS(regVal, SPI_SPIFMT_PRESCALE, 18U);                 /* PRESCALE */
    CSL_FINS(regVal, SPI_SPIFMT_CHARLEN, (uint32_t)params->dataSize);        /* CHARlEN */
    ptrMibSpiReg->SPIFMT[2] = regVal;

    /* Set Data Format 3 */
    regVal = ptrMibSpiReg->SPIFMT[3];
    CSL_FINS(regVal, SPI_SPIFMT_PRESCALE, 18U);                 /* PRESCALE */
    CSL_FINS(regVal, SPI_SPIFMT_CHARLEN, (uint32_t)params->dataSize);        /* CHARlEN */
    ptrMibSpiReg->SPIFMT[3] = regVal;

    /* Set Default Chip Select pattern: 1- chip select is set to "1" when SPI is IDLE
       Debug Note: Only CS0 can be set to 1 */
    CSL_FINS(ptrMibSpiReg->SPIDEF,SPI_SPIDEF_CSDEF0, 0xFFU);

    /* Enable ECC if enabled */
    if(params->eccEnable)
    {
        /* Enable ECC detection and signal bit Error correction */
        regVal = ptrMibSpiReg->PAR_ECC_CTRL;
        CSL_FINS(regVal, SPI_PAR_ECC_CTRL_EDEN, 0xAU);
        CSL_FINS(regVal, SPI_PAR_ECC_CTRL_EDAC_MODE, 0xAU);
        CSL_FINS(regVal, SPI_PAR_ECC_CTRL_SBE_EVT_EN, 0xAU);
        ptrMibSpiReg->PAR_ECC_CTRL = regVal;
    }

    for (index = 0; index < params->u.controllerParams.numPeripherals; index++)
    {
        /* Initialize transfer groups for number of peripherals connected to SPI controller */
        regVal = ptrMibSpiReg->TGCTRL[index];
        CSL_FINS(regVal, SPI_TGCTRL_ONESHOT, 1U);  /* Oneshot trigger */
        CSL_FINS(regVal, SPI_TGCTRL_TRIGEVT, 7U);  /* Trigger event : Always */
        CSL_FINS(regVal, SPI_TGCTRL_TRIGSRC, 0U);  /* Trigger source : disabled */
        CSL_FINS(regVal, SPI_TGCTRL_PSTART, 0U);   /* TG start address : 0 */
        ptrMibSpiReg->TGCTRL[index] = regVal;
    }

    /* Initialize transfer groups end pointer */
    CSL_FINS(ptrMibSpiReg->LTGPEND,SPI_LTGPEND_LPEND, 0xFFU);

    /* Initialize TX Buffer Ram */
    for (index = 0; index < params->u.controllerParams.numPeripherals; index++)
    {
        uint8_t    ramBufIndex = 0;
        uint8_t    wDelayEnable = 0;

        /* Multibuffer RAM control:
         * buffer mode : 0x6
         * CSHOLD: 0x0
         */

        wDelayEnable = params->u.controllerParams.wDelay? (uint8_t)1U : (uint8_t)0U;
        if(params->pinMode == MIBSPI_PINMODE_4PIN_CS)
        {
            chipSelectMask = (uint8_t)(0x1U << params->u.controllerParams.peripheralProf[index].chipSelect);
            csnr = ~chipSelectMask;
        }
        else
        {
            /* 3-pin mode, set CSNR to 0xFF */
            csnr = (uint8_t)MIBSPI_CS_NONE;
        }

        for (ramBufIndex = 0; ramBufIndex < params->u.controllerParams.peripheralProf[index].ramBufLen; ramBufIndex++)
        {
            uint32_t txControlWd = 0;
            uint16_t txCtrlWd16;
            volatile uint16_t *txCtrlWdPtr;

            CSL_FINS(txControlWd, MIBSPIRAM_TX_BUFMODE, MIBSPI_RAM_BUFFER_MODE);
            CSL_FINS(txControlWd, MIBSPIRAM_TX_CSHOLD, params->csHold);
            CSL_FINS(txControlWd, MIBSPIRAM_TX_WDEL, wDelayEnable);
            CSL_FINS(txControlWd, MIBSPIRAM_TX_CSNR, csnr);
            txCtrlWd16 = ((txControlWd & 0xFFFF0000) >> 16);
            txCtrlWdPtr = (uint16_t *)&ptrMibSpiRam->tx[ramBufOffset];
#if !defined(__ARM_BIG_ENDIAN)

            txCtrlWdPtr++;
#else
            //Do nothing. FIrst 16 bit word is the control word */
#endif
            *txCtrlWdPtr = txCtrlWd16;
            ramBufOffset++;
        }

        if(ramBufOffset > MIBSPI_RAM_MAX_ELEM)
        {
            DebugP_assert(0);
        }
    }

    /* Clear pending interrupts */
    ptrMibSpiReg->SPIFLG |= 0xFFFFU;

    /* Clear pending TG interrupts */
    ptrMibSpiReg->TGINTFLAG |= 0xFFFFFFFFU;

    /* Enable Error interrupts: Lower 8bits  */
    MIBSPI_enableErrorInterrupt(ptrMibSpiReg, 0x1U);

    /* Set Interrupt Levels - Interrupts are mapped to INT1 */
    MIBSPI_setErrorInterruptLevel(ptrMibSpiReg, 0x1U);

    /* Enable TG Interrupts to INT1 */
    ptrMibSpiReg->TGITENST|= 0xFFFFFFFFU;

    /* Pin settings for SPI signal */
    if(params->pinMode == MIBSPI_PINMODE_4PIN_CS)
    {
        chipSelectMask = (uint8_t)0U;
        for (index = 0; index < params->u.controllerParams.numPeripherals; index++)
        {
            chipSelectMask |= (uint8_t)(0x1U << params->u.controllerParams.peripheralProf[index].chipSelect);
        }
    }
    else
    {
        chipSelectMask = (uint8_t)0x0U;
    }
    MIBSPI_enablePinSettings(ptrMibSpiReg, params->pinMode, chipSelectMask, ptrHwCfg->featureBitMap);

    /* Finally start MIBSPI1 */
    CSL_FINS(ptrMibSpiReg->SPIGCR1,SPI_SPIGCR1_SPIEN, 1U);
}

static void MIBSPI_initPeripheral(const MIBSPI_Attrs* ptrHwCfg,
                             const MIBSPI_OpenParams *params)
{
    CSL_mibspiRam        *ptrMibSpiRam;
    CSL_mss_spiRegs      *ptrMibSpiReg;
    uint32_t              flag;
    uint32_t              index;
    uint32_t              regVal;

    /* Get Register and RAM base */
    ptrMibSpiRam = ptrHwCfg->ptrMibSpiRam;
    ptrMibSpiReg = ptrHwCfg->ptrSpiRegBase;

    /* Bring MIBSPI out of reset */
    ptrMibSpiReg->SPIGCR0 = 0U;
    ptrMibSpiReg->SPIGCR0 = 1U;

    /* Enable MIBSPI1 multibuffered mode and enable buffer RAM */
    CSL_FINS(ptrMibSpiReg->MIBSPIE,SPI_MIBSPIE_MSPIENA, 1U);

    /* Wait for buffer initialization complete before accessing MibSPI registers */
    while(1)
    {
        flag = CSL_FEXT(ptrMibSpiReg->SPIFLG,SPI_SPIFLG_BUFINITACTIVE);
        if(flag == 0)
        {
            break;
        }
    }

    MIBSPI_socMemInit(ptrHwCfg->mibspiInstId);

    regVal = ptrMibSpiReg->SPIGCR1;
    /* MIBSPI1 peripheral mode and clock configuration */
    CSL_FINS(regVal,SPI_SPIGCR1_MASTER, CSL_SPI_SPIGCR1_MASTER_SLAVE);
    CSL_FINS(regVal,SPI_SPIGCR1_CLKMOD, CSL_SPI_SPIGCR1_CLKMOD_EXTERNAL);
    ptrMibSpiReg->SPIGCR1 = regVal;

    /* SPIENA pin pulled high when not active */
    CSL_FINS(ptrMibSpiReg->SPIINT0,SPI_SPIINT0_ENABLEHIGHZ, 0U);

    /* Delays Configuration: this is only used by controller, hence set it to all zeros */
    ptrMibSpiReg->SPIDELAY = 0x0U;

    /* Set Data Format 0 */
    regVal = ptrMibSpiReg->SPIFMT[0];
    CSL_FINS(regVal, SPI_SPIFMT_PHASE, MIBSPI_getPhase(params->frameFormat));         /* PHASE */
    CSL_FINS(regVal, SPI_SPIFMT_POLARITY, MIBSPI_getPolarity(params->frameFormat));   /* POLARITY */
    CSL_FINS(regVal, SPI_SPIFMT_PRESCALE, 4U);   /* PRESCALE */
    CSL_FINS(regVal, SPI_SPIFMT_SHIFTDIR, (uint32_t)(params->shiftFormat)); /* SHIFTDIR */
    CSL_FINS(regVal, SPI_SPIFMT_CHARLEN, (uint32_t)params->dataSize);  /* CHARlEN */
    ptrMibSpiReg->SPIFMT[0] = regVal;

    /* Set Data Format 1,2,3. Used mulitple TG group transfer */
    regVal = ptrMibSpiReg->SPIFMT[1];
    CSL_FINS(regVal, SPI_SPIFMT_PRESCALE, 18U); /* PRESCALE */
    CSL_FINS(regVal, SPI_SPIFMT_CHARLEN, (uint32_t)params->dataSize); /* CHARlEN */
    ptrMibSpiReg->SPIFMT[1] = regVal;

    regVal = ptrMibSpiReg->SPIFMT[2];
    CSL_FINS(regVal, SPI_SPIFMT_PRESCALE, 18U); /* PRESCALE */
    CSL_FINS(regVal, SPI_SPIFMT_CHARLEN, (uint32_t)params->dataSize);    /* CHARlEN */
    ptrMibSpiReg->SPIFMT[2] = regVal;

    regVal = ptrMibSpiReg->SPIFMT[3];
    CSL_FINS(regVal, SPI_SPIFMT_PRESCALE, 18U); /* PRESCALE */
    CSL_FINS(regVal, SPI_SPIFMT_CHARLEN, (uint32_t)params->dataSize);    /* CHARlEN */
    ptrMibSpiReg->SPIFMT[3] = regVal;


    /* Enable ECC if enabled */
    if(params->eccEnable)
    {
        /* Enable ECC detection and signal bit Error correction */
        regVal = ptrMibSpiReg->PAR_ECC_CTRL;
        CSL_FINS(regVal,SPI_PAR_ECC_CTRL_EDEN, 0xAU);
        CSL_FINS(regVal,SPI_PAR_ECC_CTRL_EDAC_MODE, 0xAU);
        CSL_FINS(regVal,SPI_PAR_ECC_CTRL_SBE_EVT_EN, 0xAU);
        ptrMibSpiReg->PAR_ECC_CTRL = regVal;
    }

    for (index = 0; index < MIBSPI_NUM_TRANS_GROUP; index++)
    {
        /* Initialize transfer groups */
        regVal = ptrMibSpiReg->TGCTRL[index];
        CSL_FINS(regVal,SPI_TGCTRL_ONESHOT, 1U);  /* Oneshot trigger */
        CSL_FINS(regVal,SPI_TGCTRL_TRIGEVT, 7U);  /* Trigger event : Always */
        CSL_FINS(regVal,SPI_TGCTRL_TRIGSRC, 0U);  /* Trigger source : disabled */
        CSL_FINS(regVal,SPI_TGCTRL_PSTART,  0U);  /* TG start address : 0 */
        ptrMibSpiReg->TGCTRL[index] = regVal;
    }

    /* Initialize transfer groups end pointer */
    CSL_FINS(ptrMibSpiReg->LTGPEND,SPI_LTGPEND_LPEND, 0x0U);

    /* Initialize TX Buffer Ram, every element contains 16bits of data */
    for (index = 0; index < MIBSPI_RAM_MAX_ELEM; index++)
    {
            uint32_t txControlWd = 0;
            uint16_t txCtrlWd16;
            uint16_t *txCtrlWdPtr;

            CSL_FINS(txControlWd, MIBSPIRAM_TX_BUFMODE, MIBSPI_RAM_BUFFER_MODE);
            CSL_FINS(txControlWd, MIBSPIRAM_TX_CSHOLD, params->csHold);
            CSL_FINS(txControlWd, MIBSPIRAM_TX_CSNR, MIBSPI_CS_NONE);
            txCtrlWd16 = ((txControlWd & 0xFFFF0000) >> 16);
            txCtrlWdPtr = (uint16_t *)&ptrMibSpiRam->tx[index];
#if !defined(__ARM_BIG_ENDIAN)

           txCtrlWdPtr++;
#else
            //Do nothing. FIrst 16 bit word is the control word */
#endif
            *txCtrlWdPtr = txCtrlWd16;
    }

    /* Clear pending interrupts */
    ptrMibSpiReg->SPIFLG |= 0xFFFFU;

    /* Clear pending TG interrupts */
    ptrMibSpiReg->TGINTFLAG |= 0xFFFFFFFFU;

    /* Enable Error interrupts: Lower 8bits  */
    MIBSPI_enableErrorInterrupt(ptrMibSpiReg, 0x1U);

    /* Set Interrupt Levels - Interrupts are mapped to INT0 */
    MIBSPI_setErrorInterruptLevel(ptrMibSpiReg, 0x1U);

    /* Enable TG Interrupts to INT1 */
    ptrMibSpiReg->TGITENST|= 0xFFFFFFFFU;

    /* Pin settings for SPI signal */
    MIBSPI_enablePinSettings(ptrMibSpiReg, params->pinMode, (uint8_t)(0x1U <<params->u.peripheralParams.chipSelect),
                             ptrHwCfg->featureBitMap);

    /* Finally start MIBSPI1 */
    CSL_FINS(ptrMibSpiReg->SPIGCR1,SPI_SPIGCR1_SPIEN, 1U);
}

static void MIBSPI_writeDataRAM(MIBSPI_Object *ptrMibSpiDriver,
                                uint8_t group, uint16_t *data,
                                uint16_t dataElem)
{
    CSL_mss_spiRegs      *ptrMibSpiReg;
    CSL_mibspiRam        *ptrMibSpiRam;
    uint32_t              start;
    uint32_t              index;

    /* Get MibSpi Register & RAM Base address */
    ptrMibSpiReg  = ptrMibSpiDriver->ptrHwCfg->ptrSpiRegBase;
    ptrMibSpiRam  = ptrMibSpiDriver->ptrHwCfg->ptrMibSpiRam;

    /* Fetch the start address from Register */
    start = CSL_FEXT(ptrMibSpiReg->TGCTRL[group],SPI_TGCTRL_PSTART);

    /* Write data in TX RAM */
    if(data == NULL)
    {
        for(index=start; index < (start + dataElem); index++)
        {
            CSL_MIBSPIRAM_SET_TX_TXDATA(ptrMibSpiRam, index, ptrMibSpiDriver->txScratchBuffer);
        }
    }
    else
    {
        if(ptrMibSpiDriver->params.dataSize == 16U)
        {
            for(index=start; index < (start + dataElem); index++)
            {
                CSL_MIBSPIRAM_SET_TX_TXDATA(ptrMibSpiRam, index, data[index-start]);
            }
        }
        else
        {
            uint8_t *pBuffer = (uint8_t *)data;

            for(index=start; index < (start + dataElem); index++)
            {
                CSL_MIBSPIRAM_SET_TX_TXDATA(ptrMibSpiRam, index, pBuffer[index-start]);
            }
        }
    }
}

static uint32_t MIBSPI_readDataRAM(MIBSPI_Object *ptrMibSpiDriver,
                                   uint8_t group, uint16_t *data,
                                   uint16_t dataElem)
{
    CSL_mss_spiRegs              *ptrMibSpiReg;
    volatile CSL_mibspiRam       *ptrMibSpiRam;
    uint16_t                      mibspiFlags  = 0U;
    uint32_t                      ret;
    uint32_t                      start;
    uint32_t                      index;

    /* Get MibSpi Register & Ram Base address */
    ptrMibSpiReg  = ptrMibSpiDriver->ptrHwCfg->ptrSpiRegBase;
    ptrMibSpiRam  = ptrMibSpiDriver->ptrHwCfg->ptrMibSpiRam;

    /* Fetch the start address from Register */
    start = CSL_FEXT(ptrMibSpiReg->TGCTRL[group],SPI_TGCTRL_PSTART);

    if(data == NULL)
    {
         /* Save data from RAM into scrach buffer */
         for(index=start; index < (start + dataElem); index++)
         {
             /* Wait until data is available */
             while(CSL_MIBSPIRAM_GET_RX_RXEMPTY(ptrMibSpiRam, index)  != 0U)
             {
             }

             mibspiFlags |= CSL_MIBSPIRAM_GET_RX_RXFLAGS(ptrMibSpiRam, index);

             ptrMibSpiDriver->rxScratchBuffer = CSL_MIBSPIRAM_GET_RX_RXDATA(ptrMibSpiRam, index);
         }
    }
    else
    {

         /* Save data from RAM */
         for(index=start; index < (start + dataElem); index++)
         {

             /* Wait until data is available */
             while(CSL_MIBSPIRAM_GET_RX_RXEMPTY(ptrMibSpiRam,index) != 0U)
             {
             }

             mibspiFlags |= CSL_MIBSPIRAM_GET_RX_RXFLAGS(ptrMibSpiRam, index);

             if(ptrMibSpiDriver->params.dataSize == 16U)
             {
                 data[index-start] = CSL_MIBSPIRAM_GET_RX_RXDATA(ptrMibSpiRam, index);
             }
             else
             {
                 uint8_t *pBuffer = (uint8_t *)data;

                 pBuffer[index-start] = (uint8_t)(CSL_MIBSPIRAM_GET_RX_RXDATA(ptrMibSpiRam, index) & 0xFFU);
             }
         }
    }

    /* Rx Flags */
    ret = ((uint32_t)mibspiFlags >> 8U) & 0x5FU;

    return ret;
}

static void MIBSPI_ISR (void *args)
{
    MIBSPI_Object            *ptrMibSpiDriver;
    CSL_mss_spiRegs          *ptrMibSpiReg;
    volatile uint32_t         intVector;
    uint8_t                   group = 0U;

    /* Get the MibSpi driver handle */
    ptrMibSpiDriver = (MIBSPI_Object *)args;

    if(ptrMibSpiDriver != (MIBSPI_Object *)NULL)
    {
        ptrMibSpiReg    = ptrMibSpiDriver->ptrHwCfg->ptrSpiRegBase;

        /* Handles the TG Complete Interrupt */
        intVector = ptrMibSpiReg->TGINTFLAG;
        intVector = intVector >> 16U;
        for(group = 0U ; group < MIBSPI_PERIPHERAL_MAX; group++)
        {
            if(intVector & (0x1U << group))
            {
                ptrMibSpiDriver->hwStats.tgComplete[group]++;
                ptrMibSpiReg->TGINTFLAG = (uint32_t)0x10000U << group;

                /* if defined Multi iCount support is defined */
                if(ptrMibSpiDriver->params.iCountSupport == TRUE)
                {
                    /* All transfer done ? - check icount status */
                    if((ptrMibSpiReg->DMACTRL[group] & 0x3FU) != 0)
                    {
                        MIBSPI_transferGroupEnable(ptrMibSpiReg, group);

                    }
                }
            }
        }

        /* Found out the interrupt source and increment the stats count */
        intVector = ptrMibSpiReg->SPIFLG;
        if (intVector & 0x1U)
        {
            ptrMibSpiDriver->hwStats.dlenErr++;
        }
        if (intVector & 0x2U)
        {
            ptrMibSpiDriver->hwStats.timeout++;
        }
        if (intVector & 0x4U)
        {
            ptrMibSpiDriver->hwStats.parErr++;
        }
        if (intVector & 0x8U)
        {
            /* Controller only interrupt */
            ptrMibSpiDriver->hwStats.desync++;
        }
        if (intVector & 0x10U)
        {
            /* Controller only interrupt */
            ptrMibSpiDriver->hwStats.bitErr++;
        }
        if (intVector & 0x40U)
        {
            /* Controller only interrupt */
            ptrMibSpiDriver->hwStats.rxOvrnErr++;
        }
        if (intVector & 0x100U)
        {
            /* Controller only interrupt */
            ptrMibSpiDriver->hwStats.rxFull++;
        }
        if (intVector & 0x200U)
        {
            /* Controller only interrupt */
            ptrMibSpiDriver->hwStats.txEmpty++;
        }

        ptrMibSpiReg->SPIFLG = intVector;
    }

    return;
}

static void MIBSPI_dataTransfer(MIBSPI_Object       *ptrMibSpiDriver,
                                uint8_t             *srcData,
                                uint8_t             *dstData,
                                uint16_t             dataElemSize,
                                uint8_t              group)
{
    const MIBSPI_Attrs           *ptrHwCfg;
    CSL_mss_spiRegs              *ptrMibSpiReg;
    uint8_t                       ramOffset = 0U;
    uint16_t                      bufId = 0U;
    uint8_t                       iCount = 0U;

    /* Get MibSpi driver hardware config */
    ptrHwCfg = ptrMibSpiDriver->ptrHwCfg;

    /* Get MibSpi Register & Ram Base address */
    ptrMibSpiReg  = ptrHwCfg->ptrSpiRegBase;

    /* Put SPI in active mode */
    MIBSPI_SPIEnable(ptrMibSpiReg);

    /* Find out bufId and RAM offset */
    if(ptrMibSpiDriver->params.mode == MIBSPI_PERIPHERAL)
    {
        ramOffset = 0U;

        /* Find out iCount and bufid */
        if (dataElemSize > MIBSPI_RAM_MAX_ELEM)
        {
            /* (iCount + 1) transfer of size MIBSPI_RAM_MAX_ELEM */
            iCount = (uint8_t)(dataElemSize / MIBSPI_RAM_MAX_ELEM - 1U);
            bufId = (uint16_t)(MIBSPI_RAM_MAX_ELEM -1U);
        }
        else
        {
            /* One transfer of dataElemSize */
            iCount = (uint8_t)0U;
            bufId = (uint16_t)(dataElemSize -1U);
        }
    }
    else
    {
        uint8_t    ramLen = 0;

        DebugP_assert(group < MIBSPI_UTILS_ARRAYSIZE(ptrMibSpiDriver->rambufStart));
        ramOffset = ptrMibSpiDriver->rambufStart[group];
        ramLen =  ptrMibSpiDriver->params.u.controllerParams.peripheralProf[group].ramBufLen;

        /* Find out iCound and bufid */
        if (dataElemSize > ramLen)
        {
            /* (iCount + 1) transfer of size ramLen */
            iCount = (uint8_t)(dataElemSize / ramLen - 1U);
            bufId = (uint16_t)ramOffset + (uint16_t)ramLen -(uint16_t)1U;
        }
        else
        {
            /* One transfer of dataElemSize */
            iCount = 0;
            bufId = (uint16_t)ramOffset + dataElemSize - (uint16_t)1U;
        }
    }

    /* Initialize transfer group start offset */
    MIBSPI_transferSetPStart(ptrMibSpiReg, group, ramOffset);

    /* This is only needed for XWR16xx/XWR18xx/XWR68xx device */
    MIBSPI_transferSetPStart(ptrMibSpiReg, (uint8_t)(group + 1U), (uint8_t)(bufId + 1U));

    /* Initialize transfer groups end pointer */
    CSL_FINS(ptrMibSpiReg->LTGPEND,SPI_LTGPEND_LPEND, (uint32_t)bufId);

    /* Configure DMA in the following 3 cases
       Case 1: SrcData=NULL, DstData!=NULL   => Read data from SPI with dummy write
       Case 2: SrcData!=NULL, DstData=NULL   => Write data to SPI with dummy read
       Case 3: SrcData!=NULL, DstData!=NULL  => duplex Read/Write
    */
    if(ptrMibSpiDriver->params.dmaEnable == (uint8_t)1U)
    {
        MIBSPI_DMAXferInfo dmaXferInfo;
        uint32_t   txRAMAddr;
        uint32_t   rxRAMAddr;
        CSL_mibspiRam       *ptrMibSpiRam;

        /* Disable TG complete interrupt */
        MIBSPI_disableGroupInterrupt(ptrMibSpiReg, group);

        /* Disable SPI DMA */
        MIBSPI_dmaCtrlGroupDisable(ptrMibSpiReg, group);


        ptrMibSpiRam  = ptrHwCfg->ptrMibSpiRam;
        /* Get MibSPI RAM address */
        txRAMAddr = (uint32_t)(&(ptrMibSpiRam->tx[ramOffset]));
        rxRAMAddr = (uint32_t)(&(ptrMibSpiRam->rx[ramOffset]));


         /* Case 1: SrcData=NULL, DstData!=NULL  => Read data from SPI with dummy write */
        dmaXferInfo.dmaReqLine = group;
        if (srcData != NULL)
        {
            dmaXferInfo.tx.saddr = SOC_virtToPhy(srcData);
        }
        else
        {
            dmaXferInfo.tx.saddr = NULL;
        }
        dmaXferInfo.tx.daddr = SOC_virtToPhy((void*)txRAMAddr);
        dmaXferInfo.rx.saddr = SOC_virtToPhy((void*)rxRAMAddr);
        if (dstData != NULL)
        {
            dmaXferInfo.rx.daddr = SOC_virtToPhy(dstData);
        }
        else
        {
            dmaXferInfo.rx.daddr = NULL;
        }
        if(ptrMibSpiDriver->params.dataSize == 8U)
        {
            dmaXferInfo.size.elemSize = 1;
        }
        else
        {
            dmaXferInfo.size.elemSize =  2;
        }
        dmaXferInfo.size.elemCnt = ((uint32_t)bufId + 1U) - (uint32_t)ramOffset;
        dmaXferInfo.size.frameCnt = iCount + 1U;

        /* Configuring the mibspi dmaCtrl for the channel */
        MIBSPI_dmaCtrlGroupConfig(ptrMibSpiReg, bufId, iCount, group);

        /* Setup TG interrupt */
        MIBSPI_enableGroupInterrupt(ptrMibSpiReg, group, MIBSPI_INT_LEVEL);

        if ((CSL_FEXT(ptrMibSpiReg->IOLPBKTSTCR,SPI_IOLPBKTSTCR_LPBKTYPE) == MIBSPI_LOOPBK_ANALOG)
            &&
            (CSL_FEXT(ptrMibSpiReg->SPIGCR1,SPI_SPIGCR1_MASTER) == CSL_SPI_SPIGCR1_MASTER_SLAVE))
        {
            CSL_FINS(ptrMibSpiReg->IOLPBKTSTCR,SPI_IOLPBKTSTCR_IOLPBKTSTENA, 0xAU);
        }

        DebugP_assert(MIBSPI_edmaTransfer(ptrMibSpiDriver->mibspiHandle, &dmaXferInfo) == SystemP_SUCCESS);

        MIBSPI_dmaCtrlGroupStart(ptrMibSpiReg, group, MIBSPI_DMACTRL_CH_BOTH);

        /* Start TG group transfer */
        MIBSPI_transferGroupEnable(ptrMibSpiReg, group);
    }
    else
    {
        uint32_t  index;
        uint16_t  size;

        size = (uint32_t)bufId + 1U - (uint32_t)ramOffset;

        for(index = 0; index <= iCount; index++)
        {
            /* Read data with dummy write through CPU mode */
            if ((srcData == (uint8_t *)NULL) && (dstData != (uint8_t *)NULL))
            {
                MIBSPI_transferGroupEnable(ptrHwCfg->ptrSpiRegBase, group);

                while(MIBSPI_checkTGComplete(ptrMibSpiReg, group) == 0U)
                {
                }

                /* Read data from local buffer to MibSPI Rx RAM */
                MIBSPI_readDataRAM(ptrMibSpiDriver, group, (uint16_t *)dstData, size);

            }
            else if( (srcData != (uint8_t *)NULL) && (dstData == (uint8_t *)NULL))
            {
                /* Write data from MibSPI Rx RAM to local buffer */
                MIBSPI_writeDataRAM(ptrMibSpiDriver, group, (uint16_t *)srcData, size);

                MIBSPI_transferGroupEnable(ptrHwCfg->ptrSpiRegBase, group);

                while(MIBSPI_checkTGComplete(ptrMibSpiReg, group) == 0U)
                {
                }
                /* Read data from local buffer to MibSPI Rx RAM */
                MIBSPI_readDataRAM(ptrMibSpiDriver, group, (uint16_t *)NULL, size);

            }
            else
            {
                /* Write data from local buffer to MibSPI Tx RAM */
                MIBSPI_writeDataRAM(ptrMibSpiDriver, group, (uint16_t *)srcData, size);

                MIBSPI_transferGroupEnable(ptrHwCfg->ptrSpiRegBase, group);

                /* Wait for the transfer to complete */
                while(MIBSPI_checkTGComplete(ptrMibSpiReg, group) == 0U)
                {
                }

                /* Read the data from RX RAM */
                MIBSPI_readDataRAM(ptrMibSpiDriver, group, (uint16_t *)dstData, size);

            }

            srcData = (srcData == (uint8_t *)NULL)? (uint8_t *)NULL : (uint8_t *)(srcData + size * ptrMibSpiDriver->params.dataSize / 8U);
            dstData = (dstData == (uint8_t *)NULL)? (uint8_t *)NULL : (uint8_t *)(dstData + size * ptrMibSpiDriver->params.dataSize / 8U);

        }
        /* Transfer finished, unblock the thread */
        SemaphoreP_post(&ptrMibSpiDriver->transferSemObj);
    }

    return;
}

static int32_t MIBSPI_openPeripheralMode(MIBSPI_Object *ptrMibSpiDriver,
                                    const MIBSPI_Attrs* ptrHwCfg,
                                    const MIBSPI_OpenParams *params)
{
    int32_t     status = SystemP_SUCCESS;

    /* Initializing mibspi as a SPI peripheral */
    MIBSPI_initPeripheral(ptrHwCfg, params);

    if(params->dmaEnable)
    {
        status = MIBSPI_edmaAllocChResource(ptrMibSpiDriver->mibspiHandle, MIBSPI_PERIPHERALMODE_TRANS_GROUP);
    }
    if(SystemP_SUCCESS == status)
    {
        /* Save driver info for Peripheral mode */
        ptrMibSpiDriver->rambufStart [MIBSPI_PERIPHERALMODE_TRANS_GROUP]  = 0U;
        ptrMibSpiDriver->rambufEnd [MIBSPI_PERIPHERALMODE_TRANS_GROUP]    = MIBSPI_RAM_MAX_ELEM;
    }

    return (status);
}

static int32_t MIBSPI_openControllerMode(MIBSPI_Object *ptrMibSpiDriver,
                                     const MIBSPI_Attrs* ptrHwCfg,
                                     const MIBSPI_OpenParams *params)
{
    int32_t     status = SystemP_SUCCESS;
    uint8_t     index;
    uint8_t     ramBufOffset = 0;

    /* Initializing mibspi as a SPI controller */
    MIBSPI_initController(ptrHwCfg, params);

    /* Configure the Controller clock prescaler */
    MIBSPI_setControllerClockRate(ptrHwCfg->ptrSpiRegBase, ptrHwCfg->clockSrcFreq, params->u.controllerParams.bitRate);

    for(index = 0; index < params->u.controllerParams.numPeripherals; index++)
    {
        /***************************************
         ******** Save RAM offset information *******
         **************************************/
        ptrMibSpiDriver->rambufStart[index] = ramBufOffset;
        ptrMibSpiDriver->rambufEnd[index] = ramBufOffset + params->u.controllerParams.peripheralProf[index].ramBufLen;
        ramBufOffset =ptrMibSpiDriver->rambufEnd[index];

        /***************************************
         ****** DMA Configuration if enabled *******
         **************************************/
        if(params->dmaEnable)
        {
            status = MIBSPI_edmaAllocChResource(ptrMibSpiDriver->mibspiHandle, index);
            if (status != SystemP_SUCCESS)
            {
                break;
            }
        }
    }

    return (status);
}

static int32_t MIBSPI_closeCore(MIBSPI_Handle handle)
{
    int32_t                   status = SystemP_SUCCESS;
    MIBSPI_Config            *config;
    MIBSPI_Object            *ptrMibSpiDriver;
    CSL_mss_spiRegs          *ptrMibSpiReg;

    /* Sanity check handle */
    DebugP_assert(handle != NULL);

    config = (MIBSPI_Config *) handle;

    /* Get the Object from SPI Handle */
    ptrMibSpiDriver = config->object;

    /* Sanity check driver handle */
    if(ptrMibSpiDriver != NULL)
    {
        /* Get the Register base address from SPI Handle */
        ptrMibSpiReg = ptrMibSpiDriver->ptrHwCfg->ptrSpiRegBase;

        DebugP_assert(ptrMibSpiDriver->transactionState.transaction == NULL);

        if(ptrMibSpiDriver->params.dmaEnable)
        {

            if (ptrMibSpiDriver->params.mode == MIBSPI_PERIPHERAL)
            {
                status = MIBSPI_edmaFreeChResource(ptrMibSpiDriver->mibspiHandle, MIBSPI_PERIPHERALMODE_TRANS_GROUP);
            }
            else
            {
                uint32_t index;

                for (index = 0; index < ptrMibSpiDriver->params.u.controllerParams.numPeripherals; index++)
                {
                   status += MIBSPI_edmaFreeChResource(ptrMibSpiDriver->mibspiHandle, index);
                }
            }
        }

        if (NULL != ptrMibSpiDriver->transferSem)
        {
            /* Delete the semaphore */
            SemaphoreP_destruct (&ptrMibSpiDriver->transferSemObj);
            ptrMibSpiDriver->transferSem = NULL;
        }

        /* Disable MibSPI */
        MIBSPI_setResetMode(ptrMibSpiReg, TRUE);

        /* Delete Hwi */
        if(NULL != ptrMibSpiDriver->hwiHandle)
        {
            HwiP_destruct(&ptrMibSpiDriver->hwiObj);
            ptrMibSpiDriver->hwiHandle = NULL;

        }
        memset(ptrMibSpiDriver, 0, sizeof(*ptrMibSpiDriver));

        ptrMibSpiDriver->isOpen = FALSE;
    }
    return (status);
}

static void MIBSPI_initTransactionState(MIBSPI_TransactionState *transactionState,
                                        MIBSPI_Transaction     *transaction)
{
    uintptr_t key;

    key = HwiP_disable();

    transactionState->edmaCbCheck = MIBSPI_NONE_EDMA_CALLBACK_OCCURED;
    transactionState->transferErr = MIBSPI_XFER_ERR_NONE;
    transactionState->transaction = transaction;
    transactionState->transaction->status = MIBSPI_TRANSFER_STARTED;
    transactionState->remainSize = 0U;
    transactionState->dataLength = 0U;
    transactionState->dataSizeInBytes = 0U;

    HwiP_restore(key);
}

static void MIBSPI_resetTransactionState(MIBSPI_TransactionState *transactionState)
{
    uintptr_t key;

    key = HwiP_disable();

    transactionState->edmaCbCheck = MIBSPI_NONE_EDMA_CALLBACK_OCCURED;
    transactionState->transferErr = MIBSPI_XFER_ERR_NONE;
    transactionState->transaction = NULL;
    transactionState->remainSize = 0U;
    transactionState->dataLength = 0U;
    transactionState->dataSizeInBytes = 0U;

    HwiP_restore(key);
}

static int32_t MIBSPI_transferCore(MIBSPI_Handle handle,
                                   MIBSPI_Transaction *transaction)
{
    uintptr_t                 key;
    MIBSPI_Config            *config;
    MIBSPI_Object            *ptrMibSpiDriver;
    int32_t                   semaStatus;
    uint16_t                  dataLength;
    int32_t                   status = SystemP_SUCCESS;

    DebugP_assert(handle != NULL);

    config = (MIBSPI_Config *) handle;

    /* Get the MibSpi driver handle */
    ptrMibSpiDriver = config->object;

    status = MIBSPI_validateTransferParams(transaction, ptrMibSpiDriver);
    if (status != SystemP_SUCCESS)
    {
        /* Initialiaze transaction as failed transer */
        transaction->status = MIBSPI_TRANSFER_FAILED;
        status = SystemP_FAILURE;
    }

    if (status == SystemP_SUCCESS)
    {
        if(ptrMibSpiDriver->params.dataSize == 16U)
        {
            /* MIBSPI reads in 2 bytes format */
            dataLength = (uint16_t)transaction->count >> 1U;
        }
        else
        {
            dataLength = transaction->count;
        }

        MIBSPI_initTransactionState(&ptrMibSpiDriver->transactionState, transaction);

        /* if  Multi iCount support is not defined */
        if(ptrMibSpiDriver->params.iCountSupport == FALSE)
        {

            if(ptrMibSpiDriver->params.mode == MIBSPI_PERIPHERAL)
            {
                MIBSPI_dataTransfer(ptrMibSpiDriver,
                                    (uint8_t *)transaction->txBuf,
                                    (uint8_t *)transaction->rxBuf,
                                    dataLength,
                                    MIBSPI_PERIPHERALMODE_TRANS_GROUP);
            }
            else
            {
                MIBSPI_dataTransfer(ptrMibSpiDriver,
                                    (uint8_t *)transaction->txBuf,
                                    (uint8_t *)transaction->rxBuf,
                                    dataLength,
                                    transaction->peripheralIndex);
            }

            if (ptrMibSpiDriver->params.transferMode == MIBSPI_MODE_BLOCKING)
            {
                semaStatus = SemaphoreP_pend(&ptrMibSpiDriver->transferSemObj, ptrMibSpiDriver->params.transferTimeout);
                if(SystemP_SUCCESS != semaStatus)
                {
                    /* Populate status code */
                    transaction->status = MIBSPI_TRANSFER_TIMEOUT;
                    status = SystemP_FAILURE;
                }
                else
                {
                    /* Populate status code */
                    transaction->status = MIBSPI_TRANSFER_COMPLETED;
                    status = SystemP_SUCCESS;
                }
            }
            else
            {
                /* Execution should not reach here */
                transaction->status = MIBSPI_TRANSFER_FAILED;
                status = SystemP_FAILURE;
            }
        }
        else
        {
            uint32_t    ramSize;
            uint16_t    remainSize;
            uint16_t    dataSizeInBytes = 2U;

            if(ptrMibSpiDriver->params.dataSize == 16U)
            {
                dataSizeInBytes = 2U;
            }
            else
            {
                dataSizeInBytes = 1U;
            }

            /* Find out rambuf size */
            if(ptrMibSpiDriver->params.mode == MIBSPI_PERIPHERAL)
            {
                ramSize = MIBSPI_RAM_MAX_ELEM;
            }
            else
            {
                ramSize = ptrMibSpiDriver->params.u.controllerParams.peripheralProf[transaction->peripheralIndex].ramBufLen;
            }

            /*
             *  If dataLength is bigger than ramSize, there will be two data transfers:
             *      1. multiple of ramSize
             *      2. remaining data after multiple of ramsize
             *  If dataLength is smaller or equal to ramSize, there will be one data transfer
             */
            remainSize = (dataLength > ramSize) ? dataLength%ramSize : 0U;

            if(remainSize > 0U)
            {
                /* Size of the first transfer */
                dataLength -= remainSize;
            }

            if (ptrMibSpiDriver->params.transferMode == MIBSPI_MODE_CALLBACK)
            {
                /* Tell ISR function how much data there is remaining after this transfer
                 * (if remainSize > 0 then this is the first of two transfers) */
                ptrMibSpiDriver->transactionState.remainSize = remainSize;
                ptrMibSpiDriver->transactionState.dataLength = dataLength;
                ptrMibSpiDriver->transactionState.dataSizeInBytes = dataSizeInBytes;
            }

            if(ptrMibSpiDriver->params.mode == MIBSPI_PERIPHERAL)
            {
                MIBSPI_dataTransfer(ptrMibSpiDriver,
                                    (uint8_t *)transaction->txBuf,
                                    (uint8_t *)transaction->rxBuf,
                                    dataLength,
                                    MIBSPI_PERIPHERALMODE_TRANS_GROUP);
            }
            else
            {
                MIBSPI_dataTransfer(ptrMibSpiDriver,
                                    (uint8_t *)transaction->txBuf,
                                    (uint8_t *)transaction->rxBuf,
                                    dataLength,
                                    transaction->peripheralIndex);
            }

            if (ptrMibSpiDriver->params.transferMode == MIBSPI_MODE_BLOCKING)
            {
                semaStatus = SemaphoreP_pend(&ptrMibSpiDriver->transferSemObj, ptrMibSpiDriver->params.transferTimeout);
                if(SystemP_SUCCESS != semaStatus)
                {
                    /* Populate status code */
                    transaction->status = MIBSPI_TRANSFER_TIMEOUT;
                    status = SystemP_FAILURE;
                    remainSize = 0;
                }
                else
                {
                    /* Populate status code */
                    transaction->status = MIBSPI_TRANSFER_COMPLETED;
                    status = SystemP_SUCCESS;
                }
            }
            else
            {
                /* Return success status if non-blocking mode */
                status = SystemP_SUCCESS;
                return status;
            }

            /* Check if transfer finished */
            if(remainSize > 0)
            {
                /* Change buffer pointer and data size for the second transfer */
                transaction->txBuf = (void *)((transaction->txBuf == NULL)? NULL: (uint8_t *)transaction->txBuf + dataLength * dataSizeInBytes);
                transaction->rxBuf = (void *)((transaction->rxBuf == NULL)? NULL: (uint8_t *)transaction->rxBuf + dataLength * dataSizeInBytes);
                dataLength = remainSize;

                /* Transfer the remaining size in blocking mode */
                if(ptrMibSpiDriver->params.mode == MIBSPI_PERIPHERAL)
                {
                    MIBSPI_dataTransfer(ptrMibSpiDriver,
                                        (uint8_t *)transaction->txBuf,
                                        (uint8_t *)transaction->rxBuf,
                                        dataLength,
                                        MIBSPI_PERIPHERALMODE_TRANS_GROUP);
                }
                else
                {
                    MIBSPI_dataTransfer(ptrMibSpiDriver,
                                        (uint8_t *)transaction->txBuf,
                                        (uint8_t *)transaction->rxBuf,
                                        dataLength,
                                        transaction->peripheralIndex);
                }

                semaStatus = SemaphoreP_pend(&ptrMibSpiDriver->transferSemObj, ptrMibSpiDriver->params.transferTimeout);
                if(SystemP_SUCCESS != semaStatus)
                {
                    /* Populate status code */
                    transaction->status = MIBSPI_TRANSFER_TIMEOUT;
                    status = SystemP_FAILURE;
                }
                else
                {
                    /* Populate status code */
                    transaction->status = MIBSPI_TRANSFER_COMPLETED;
                    status = SystemP_SUCCESS;
                }
            }
        }

        /* For controller blocking mode, if CS Hold is set clear it in SPIDAT1 reg here. */
        if ((status == SystemP_SUCCESS) && (ptrMibSpiDriver->params.mode == MIBSPI_CONTROLLER) &&
            (ptrMibSpiDriver->params.transferMode == MIBSPI_MODE_BLOCKING) &&
            (ptrMibSpiDriver->params.csHold != 0))
            {
                CSL_mss_spiRegs *ptrMibSpiReg = ptrMibSpiDriver->ptrHwCfg->ptrSpiRegBase;
                CSL_FINS(ptrMibSpiReg->SPIDAT1,SPI_SPIDAT1_CSHOLD, 0U);
            }
    }

    /* Disable transfer group */
    MIBSPI_transferGroupDisable((ptrMibSpiDriver->ptrHwCfg->ptrSpiRegBase), transaction->peripheralIndex);

    /* Clear transaction handle */
    key = HwiP_disable();

    ptrMibSpiDriver->transactionState.transaction = NULL;

    HwiP_restore(key);
    return status;
}

static void MIBSPI_transferCancelCore(MIBSPI_Handle handle)
{
    /* This API is not supported, hence throw an assertion */
    DebugP_assert(0);

    return;
}

void MIBSPI_dmaDoneCb(MIBSPI_Handle mibspiHandle)
{
    MIBSPI_Object       *ptrMibSpiDriver = NULL;
    MIBSPI_Config       *ptrSPIConfig;
    CSL_mss_spiRegs     *ptrMibSpiReg;

    /* Get the SPI driver Configuration: */
    ptrSPIConfig = (MIBSPI_Config*)mibspiHandle;

    ptrMibSpiDriver = ptrSPIConfig->object;

    ptrMibSpiReg = ptrMibSpiDriver->ptrHwCfg->ptrSpiRegBase;
    if ((CSL_FEXT(ptrMibSpiReg->IOLPBKTSTCR,SPI_IOLPBKTSTCR_LPBKTYPE) == MIBSPI_LOOPBK_ANALOG)
        &&
        (CSL_FEXT(ptrMibSpiReg->SPIGCR1,SPI_SPIGCR1_MASTER) == CSL_SPI_SPIGCR1_MASTER_SLAVE)
        &&
        (CSL_FEXT(ptrMibSpiReg->IOLPBKTSTCR,SPI_IOLPBKTSTCR_IOLPBKTSTENA) ==  0xAU))
    {
        CSL_FINS(ptrMibSpiReg->IOLPBKTSTCR,SPI_IOLPBKTSTCR_IOLPBKTSTENA, 0x5U);
    }

    if ((ptrMibSpiDriver->params.transferMode == MIBSPI_MODE_BLOCKING) && (ptrMibSpiDriver->transferSem != NULL) )
    {
        /* MibSPI-RX DMA complete interrupt */
        SemaphoreP_post(&ptrMibSpiDriver->transferSemObj);
    }
    else
    {
        if (ptrMibSpiDriver->transactionState.transaction != NULL)
        {
            /* Call the transfer completion callback function */
            if ((ptrMibSpiDriver->transactionState.transaction->status == MIBSPI_TRANSFER_STARTED) &&
                (ptrMibSpiDriver->transactionState.transferErr == MIBSPI_XFER_ERR_NONE) &&
                (ptrMibSpiDriver->transactionState.remainSize > 0U))
            {
                /* Change buffer pointer and data size for the second transfer */
                ptrMibSpiDriver->transactionState.transaction->txBuf = (void *)((ptrMibSpiDriver->transactionState.transaction->txBuf == NULL) ?
                    NULL : (uint8_t *)ptrMibSpiDriver->transactionState.transaction->txBuf +
                    ptrMibSpiDriver->transactionState.dataLength * (ptrMibSpiDriver->transactionState.dataSizeInBytes));
                ptrMibSpiDriver->transactionState.transaction->rxBuf = (void *)((ptrMibSpiDriver->transactionState.transaction->rxBuf == NULL) ?
                    NULL : (uint8_t *)ptrMibSpiDriver->transactionState.transaction->rxBuf +
                    ptrMibSpiDriver->transactionState.dataLength * (ptrMibSpiDriver->transactionState.dataSizeInBytes));
                ptrMibSpiDriver->transactionState.dataLength = ptrMibSpiDriver->transactionState.remainSize;
                ptrMibSpiDriver->transactionState.remainSize = 0U;

                /* Transfer the remaining size */
                if(ptrMibSpiDriver->params.mode == MIBSPI_PERIPHERAL)
                {
                    MIBSPI_dataTransfer(ptrMibSpiDriver, (uint8_t *)ptrMibSpiDriver->transactionState.transaction->txBuf,
                        (uint8_t *)ptrMibSpiDriver->transactionState.transaction->rxBuf, ptrMibSpiDriver->transactionState.dataLength, MIBSPI_PERIPHERALMODE_TRANS_GROUP);
                }
                else
                {
                    MIBSPI_dataTransfer(ptrMibSpiDriver, (uint8_t *)ptrMibSpiDriver->transactionState.transaction->txBuf,
                        (uint8_t *)ptrMibSpiDriver->transactionState.transaction->rxBuf, ptrMibSpiDriver->transactionState.dataLength, ptrMibSpiDriver->transactionState.transaction->peripheralIndex);
                }
            }
            else if ((ptrMibSpiDriver->transactionState.transaction->status == MIBSPI_TRANSFER_STARTED) &&
                (ptrMibSpiDriver->transactionState.transferErr == MIBSPI_XFER_ERR_NONE))
            {
                /* Update status*/
                ptrMibSpiDriver->transactionState.transaction->status = MIBSPI_TRANSFER_COMPLETED;
            }
            else
            {
                /* Update status */
                ptrMibSpiDriver->transactionState.transaction->status = MIBSPI_TRANSFER_FAILED;
            }

            /* For controller blocking mode, if CS Hold is set clear it in SPIDAT1 reg here. */
            if ((ptrMibSpiDriver->transactionState.transaction->status == MIBSPI_TRANSFER_COMPLETED) &&
                (ptrMibSpiDriver->params.mode == MIBSPI_CONTROLLER) &&
                (ptrMibSpiDriver->params.csHold != 0))
            {
                CSL_FINS(ptrMibSpiReg->SPIDAT1,SPI_SPIDAT1_CSHOLD, 0U);
            }

            if ((ptrMibSpiDriver->transactionState.transaction->status == MIBSPI_TRANSFER_COMPLETED) ||
                (ptrMibSpiDriver->transactionState.transaction->status == MIBSPI_TRANSFER_FAILED))
            {
                /* Call transferCallbackFxn */
                ptrMibSpiDriver->params.transferCallbackFxn(mibspiHandle, ptrMibSpiDriver->transactionState.transaction);

                /* Disable transfer group */
                MIBSPI_transferGroupDisable((ptrMibSpiDriver->ptrHwCfg->ptrSpiRegBase), ptrMibSpiDriver->transactionState.transaction->peripheralIndex);

                /* Reset transaction state */
                MIBSPI_resetTransactionState(&ptrMibSpiDriver->transactionState);
            }
        }
    }
}

static uint32_t MIBSPI_getPhase(MIBSPI_FrameFormat frameFormat)
{
    uint32_t phase = 0;

    switch (frameFormat)
    {
        case MIBSPI_POL0_PHA0:    /*!< SPI mode Polarity 0 Phase 0 */
        case MIBSPI_POL1_PHA0:    /*!< SPI mode Polarity 1 Phase 0 */
            phase = 0;
            break;
        case MIBSPI_POL0_PHA1:    /*!< SPI mode Polarity 0 Phase 1 */
        case MIBSPI_POL1_PHA1:    /*!< SPI mode Polarity 1 Phase 1 */
            phase = 1;
            break;
        default:
            DebugP_assert(FALSE);
    }
    return phase;
}

static uint32_t MIBSPI_getPolarity(MIBSPI_FrameFormat frameFormat)
{
    uint32_t polarity = 0;

    switch (frameFormat)
    {
        case MIBSPI_POL0_PHA0:    /*!< SPI mode Polarity 0 Phase 0 */
        case MIBSPI_POL0_PHA1:    /*!< SPI mode Polarity 0 Phase 1 */
            polarity = 0;
            break;
        case MIBSPI_POL1_PHA0:    /*!< SPI mode Polarity 1 Phase 0 */
        case MIBSPI_POL1_PHA1:    /*!< SPI mode Polarity 1 Phase 1 */
            polarity = 1;
            break;
        default:
            DebugP_assert(FALSE);
    }
    return polarity;
}

static void MIBSPI_enableErrorInterrupt(CSL_mss_spiRegs  *ptrMibSpiReg, uint32_t enableFlag)
{
    uint32_t regVal = ptrMibSpiReg->SPIINT0;

    CSL_FINS(regVal, SPI_SPIINT0_DLENERRENA, enableFlag);
    CSL_FINS(regVal, SPI_SPIINT0_TIMEOUTENA, enableFlag);
    CSL_FINS(regVal, SPI_SPIINT0_PARERRENA, enableFlag);
    CSL_FINS(regVal, SPI_SPIINT0_DESYNCENA, enableFlag);
    CSL_FINS(regVal, SPI_SPIINT0_BITERRENA, enableFlag);
    CSL_FINS(regVal, SPI_SPIINT0_OVRNINTENA, enableFlag);
    ptrMibSpiReg->SPIINT0 = regVal;
}

static void MIBSPI_setErrorInterruptLevel(CSL_mss_spiRegs  *ptrMibSpiReg, uint32_t level)
{
    uint32_t regVal = ptrMibSpiReg->SPILVL;

    CSL_FINS(regVal, SPI_SPILVL_DLENERRLVL, level);
    CSL_FINS(regVal, SPI_SPILVL_TIMEOUTLVL, level);
    CSL_FINS(regVal, SPI_SPILVL_PARERRLVL, level);
    CSL_FINS(regVal, SPI_SPILVL_DESYNCLVL, level);
    CSL_FINS(regVal, SPI_SPILVL_BITERRLVL, level);
    CSL_FINS(regVal, SPI_SPILVL_OVRNINTLVL, level);
    ptrMibSpiReg->SPILVL = regVal;
}
