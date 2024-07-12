/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
 *  \file gpmc_nandlike_v0.c
 *
 *  \brief File containing GPMC nandlike Driver APIs implementation for version V0.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <string.h>
#include <kernel/dpl/ClockP.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/gpmc.h>
#include <drivers/elm.h>
#include <drivers/gpmc/v0/dma/gpmc_dma.h>
#include "gpmc_priv_v0.h"

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Internal functions */
static uint32_t GPMC_eccBchResultGet(GPMC_Handle handle, uint32_t resIndex, uint32_t sector);
static void GPMC_eccResultSizeSelect(GPMC_Handle handle, uint32_t eccResReg,
                             uint32_t eccSize);
static void GPMC_nandAddressWrite(GPMC_Handle handle, uint32_t address);
static void GPMC_nandCommandWrite(GPMC_Handle handle, uint32_t cmd);
static int32_t GPMC_prefetchPostWriteConfigEnable(GPMC_Handle handle, uint8_t mode,
                                    uint32_t transferCount, uint8_t modeDMA);
static int32_t GPMC_prefetchPostWriteConfigDisable(GPMC_Handle handle);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

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
                    while(CSL_REG32_FEXT(attrs->gpmcBaseAddr + CSL_GPMC_PREFETCH_STATUS, GPMC_PREFETCH_STATUS_COUNTVALUE))
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


/* ========================================================================== */
/*                     Internal function definitions                          */
/* ========================================================================== */

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

        if(attrs->optimisedAccess == CSL_GPMC_PREFETCH_CONFIG1_ENABLEOPTIMIZEDACCESS_OPTENABLED)
        {
            CSL_REG32_FINS(attrs->gpmcBaseAddr + CSL_GPMC_PREFETCH_CONFIG1, GPMC_PREFETCH_CONFIG1_CYCLEOPTIMIZATION, \
            attrs->cycleOptimisation);

            CSL_REG32_FINS(attrs->gpmcBaseAddr + CSL_GPMC_PREFETCH_CONFIG1, GPMC_PREFETCH_CONFIG1_ENABLEOPTIMIZEDACCESS, \
            attrs->optimisedAccess);
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

        CSL_REG32_FINS(attrs->gpmcBaseAddr + CSL_GPMC_PREFETCH_CONFIG1, GPMC_PREFETCH_CONFIG1_ENABLEOPTIMIZEDACCESS, \
        CSL_GPMC_PREFETCH_CONFIG1_ENABLEOPTIMIZEDACCESS_OPTDISABLED);

    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
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
