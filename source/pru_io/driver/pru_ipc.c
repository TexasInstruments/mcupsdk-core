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

#include <pru_io/driver/pru_ipc.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/hw_include/cslr_soc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern int32_t gPruIpcConfigNum;
extern PRU_IPC_Config gPruIpcConfig[];

/* ========================================================================== */
/*                       Function Definitions                                 */
/* ========================================================================== */

void PRU_IPC_init(void)
{
    /* Do Nothing */
}

void PRU_IPC_deinit(void)
{
    /* Do Nothing */
}

void PRU_IPC_Params_init(PRU_IPC_Params *params)
{
    params->pruicssHandle       = NULL;
    params->transferCallbackFxn = NULL;
}

/*
 *  This function gets the handle according to the passed idx,
 *  writes the config info to PRU configMemAddr and returns handle
 */
PRU_IPC_Handle PRU_IPC_open(uint32_t idx, PRU_IPC_Params *params)
{
    PRU_IPC_Handle handle = NULL;
    PRU_IPC_Object *obj   = NULL;

    /* idx will be generated from sysconfig:  module-instance name */
    if(idx < gPruIpcConfigNum)
    {
        handle = (PRU_IPC_Handle)&(gPruIpcConfig[idx]);
    }

    if(handle != NULL)
    {
        obj = (PRU_IPC_Object *)handle->object;
        /* check if these are NULL */
        obj->pruIpcParams.pruicssHandle = params->pruicssHandle;
        if(handle->attrs->enableRxInt)
        {
            obj->pruIpcParams.transferCallbackFxn = params->transferCallbackFxn;
            int32_t status;
            status = PRUICSS_registerIrqHandler(obj->pruIpcParams.pruicssHandle,
                                            handle->attrs->pruEvtoutNum,   /*  pruEvtoutNum */
                                            handle->attrs->r5fIntrNum,     /*  intrNum      */
                                            1,                      /*  eventNum     */
                                            0,                      /*  wait_enable  */
                                            obj->pruIpcParams.transferCallbackFxn
                                            );
            DebugP_assert(SystemP_SUCCESS == status);
        }
    }

    if(handle != NULL)
    {
        handle->attrs->config->dataSize    = handle->attrs->dataSize;
        handle->attrs->config->noOfBuffers = handle->attrs->noOfBuffers;
        handle->attrs->config->blockSize   = handle->attrs->blockSize;
        handle->attrs->config->noOfBlocks  = handle->attrs->noOfBlocks;

        for(uint32_t bufferIdx = 0; bufferIdx < handle->attrs->noOfBuffers; bufferIdx++)
            CSL_REG32_WR((uint32_t) (&(handle->attrs->config->bufferAddrs)) + (bufferIdx << 2),
                            (handle->attrs->bufferAddrs[bufferIdx]));
    }

    return handle;
}

void PRU_IPC_close(PRU_IPC_Handle handle)
{
    /* Do Nothing */
}

uint16_t PRU_IPC_getBlockId(PRU_IPC_Handle handle)
{
    return handle->attrs->config->blockId;
}

int32_t PRU_IPC_getData(PRU_IPC_Handle handle, void *container)
{
    int32_t *array = (int32_t *) container;
    uint16_t id_block    = PRU_IPC_getBlockId(handle);
    uint32_t blockOffset = id_block * handle->attrs->blockSizeBytes;
    if (handle->attrs->dataSize == 4)
    {
    for (uint32_t bufferIdx = 0; bufferIdx < handle->attrs->noOfBuffers; bufferIdx++)
        for (uint32_t dataPacketIdx = 0; dataPacketIdx < handle->attrs->blockSize; dataPacketIdx++)
            array[bufferIdx*handle->attrs->blockSize + dataPacketIdx] = CSL_REG32_RD(handle->attrs->bufferAddrs[bufferIdx]
                                            + blockOffset + (dataPacketIdx << 2));
    }
    else if (handle->attrs->dataSize == 2)
    {
    for (uint32_t bufferIdx = 0; bufferIdx < handle->attrs->noOfBuffers; bufferIdx++)
        for (uint32_t dataPacketIdx = 0; dataPacketIdx < handle->attrs->blockSize; dataPacketIdx++)
            array[bufferIdx*handle->attrs->blockSize + dataPacketIdx] = CSL_REG32_RD(handle->attrs->bufferAddrs[bufferIdx]
                                            + blockOffset + (dataPacketIdx << 1));
    }
    else if (handle->attrs->dataSize == 1)
    {
    for (uint32_t bufferIdx = 0; bufferIdx < handle->attrs->noOfBuffers; bufferIdx++)
        for (uint32_t dataPacketIdx = 0; dataPacketIdx < handle->attrs->blockSize; dataPacketIdx++)
            array[bufferIdx*handle->attrs->blockSize + dataPacketIdx] = CSL_REG32_RD(handle->attrs->bufferAddrs[bufferIdx]
                                            + blockOffset + (dataPacketIdx));
    }
    /*
      TODO: memcpy for variable size? test -
      memcpy(array[bufferIdx*handle->attrs->blockSize + dataPacketIdx], (handle->attrs->bufferAddrs[bufferIdx] +
             blockOffset + (dataPacketIdx * handle->attrs->dataSize)), handle->attrs->dataSize);
    */
    return SystemP_SUCCESS;
}

int32_t PRU_IPC_sendData(PRU_IPC_Handle handle, void *container)
{
    int32_t status = SystemP_SUCCESS;
    int32_t *array = (int32_t *) container;
    uint16_t id_block = PRU_IPC_getBlockId(handle);
    uint32_t blockOffset = id_block * handle->attrs->blockSizeBytes;
    if (handle->attrs->dataSize == 4)
    {
        for (uint32_t bufferIdx = 0; bufferIdx < handle->attrs->noOfBuffers; bufferIdx++)
            for (uint32_t dataPacketIdx = 0; dataPacketIdx < handle->attrs->blockSize; dataPacketIdx++)
                CSL_REG32_WR(handle->attrs->bufferAddrs[bufferIdx] + blockOffset + (dataPacketIdx << 2),
                            array[bufferIdx*handle->attrs->blockSize + dataPacketIdx]);
    }
    else if (handle->attrs->dataSize == 2)
    {
        for (uint32_t bufferIdx = 0; bufferIdx < handle->attrs->noOfBuffers; bufferIdx++)
            for (uint32_t dataPacketIdx = 0; dataPacketIdx < handle->attrs->blockSize; dataPacketIdx++)
                CSL_REG32_WR(handle->attrs->bufferAddrs[bufferIdx] + blockOffset + (dataPacketIdx << 1),
                            array[bufferIdx*handle->attrs->blockSize + dataPacketIdx]);
    }
    else if (handle->attrs->dataSize == 1)
    {
        for (uint32_t bufferIdx = 0; bufferIdx < handle->attrs->noOfBuffers; bufferIdx++)
            for (uint32_t dataPacketIdx = 0; dataPacketIdx < handle->attrs->blockSize; dataPacketIdx++)
                CSL_REG32_WR(handle->attrs->bufferAddrs[bufferIdx] + blockOffset + (dataPacketIdx),
                            array[bufferIdx*handle->attrs->blockSize + dataPacketIdx]);
    }

    if (handle->attrs->enableTxInt)
    {
        status = PRUICSS_sendEvent(handle->object->pruIpcParams.pruicssHandle, handle->attrs->txEventNum);
    }
    return status;
}