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
#include <board/eeprom.h>
#include <board/eeprom/eeprom_cat24m.h>
#include <drivers/hw_include/csl_types.h>
#include <kernel/dpl/ClockP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/** \brief For offset more than this, the I2C address needs to be incremented */
#define EEPROM_AT24C_WRAP_OFFSET        (64U * 1024U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

EEPROM_Attrs gEepromAttrs_CAT24M =
{
    .size       = 128U * 1024U,
    .pageCount  = 512U,
    .pageSize   = 256U,
};

EEPROM_Fxns gEepromFxns_CAT24M =
{
    .openFxn    = EEPROM_CAT24M_open,
    .closeFxn   = EEPROM_CAT24M_close,
    .readFxn    = EEPROM_CAT24M_read,
    .writeFxn   = EEPROM_CAT24M_write,
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t EEPROM_CAT24M_open(EEPROM_Config *config, const EEPROM_Params *params)
{
    int32_t         status = SystemP_SUCCESS;
    EEPROM_Object  *object;

    if((NULL == config) || (NULL == params))
    {
        status = SystemP_FAILURE;
    }
    else
    {
        object = (EEPROM_Object *) config->object;
        object->driverInstance = params->driverInstance;
        object->i2cAddress     = params->i2cAddress;
        object->i2cHandle      = I2C_getHandle(object->driverInstance);
        object->lock           = NULL;
        if(NULL == object->i2cHandle)
        {
            status = SystemP_FAILURE;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        status = SemaphoreP_constructMutex(&object->lockObj);
        object->lock = &object->lockObj;
    }

    return (status);
}

void EEPROM_CAT24M_close(EEPROM_Config *config)
{
    EEPROM_Object  *object;

    if(NULL != config)
    {
        object = (EEPROM_Object *) config->object;

        /* I2C Driver will be closed outside flash */
        object->i2cHandle = NULL;
        if(NULL != object->lock)
        {
            SemaphoreP_destruct(&object->lockObj);
            object->lock = NULL;
        }
    }
    return;
}

int32_t EEPROM_CAT24M_read(EEPROM_Config *config,
                          uint32_t offset,
                          uint8_t *buf,
                          uint32_t len)
{
    int32_t         status = SystemP_SUCCESS;
    EEPROM_Object  *object;
    EEPROM_Attrs   *attrs;
    I2C_Transaction i2cTransaction;
    uint32_t        i2cAddress;
    uint32_t        readLen;
    uint32_t        readOffset;
    uint8_t         offsetBuf[2U];

    if((NULL == config) || (NULL == buf))
    {
        status = SystemP_FAILURE;
    }
    else
    {
        object      = (EEPROM_Object *) config->object;
        attrs       = config->attrs;
        /* Validate address input */
        if((offset + len) > attrs->size)
        {
            status = SystemP_FAILURE;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        i2cAddress  = object->i2cAddress;
        readLen         = len;
        readOffset      = offset;

        SemaphoreP_pend(&object->lockObj, SystemP_WAIT_FOREVER);

        if(readLen != 0U)
        {
            /* Perform dummy write to set the right offset and read data */
            offsetBuf[0U] = (readOffset & 0xFF00U) >> 8U;
            offsetBuf[1U] = (readOffset & 0x00FFU);
            I2C_Transaction_init(&i2cTransaction);
            i2cTransaction.writeBuf     = &offsetBuf[0U];
            i2cTransaction.writeCount   = 2U;
            i2cTransaction.readBuf      = buf;
            i2cTransaction.readCount    = readLen;
            i2cTransaction.slaveAddress = i2cAddress;
            status += I2C_transfer(object->i2cHandle, &i2cTransaction);
        }

        SemaphoreP_post(&object->lockObj);
    }

    return (status);
}

int32_t EEPROM_CAT24M_write(EEPROM_Config *config,
                           uint32_t offset,
                           const uint8_t *buf,
                           uint32_t len)
{ 
    int32_t         status = SystemP_SUCCESS, writeStatus;
    EEPROM_Object  *object;
    EEPROM_Attrs   *attrs;
    I2C_Transaction i2cTransaction;
    uint32_t        i2cAddress;
    uint32_t        curWriteLen, curOffset, bytesWritten;
    uint8_t        *pageWrBuf, dummyRead;

    if((NULL == config) || (NULL == buf))
    {
        status = SystemP_FAILURE;
    }
    else
    {
        object      = (EEPROM_Object *) config->object;
        attrs       = config->attrs;
        /* Validate address input */
        if((offset + len) > attrs->size)
        {
            status = SystemP_FAILURE;
        }
    }

    if(status == SystemP_SUCCESS)
    {
        SemaphoreP_pend(&object->lockObj, SystemP_WAIT_FOREVER);

        /* Perform page by page write operation */
        curOffset = offset;
        bytesWritten = 0U;
        pageWrBuf = object->pageWrBuf;
        while(len > 0)
        {
            /* For offset more than 64KB we need to perform seperate write
             * operation with I2C device address as +1 -> Bit 1 (A16) set to 1 */
            i2cAddress  = object->i2cAddress;

            /* Calculate num bytes to write for this iteration */
            curWriteLen = attrs->pageSize;
            if(curOffset % attrs->pageSize)
            {
                /* For first non-page boundary writes */
                curWriteLen = attrs->pageSize - (curOffset % attrs->pageSize);
            }
            /* Adjust for last chunk */
            if(curWriteLen > len)
            {
                curWriteLen = len;
            }

            /* Perform dummy write operation to set the right offset */
            pageWrBuf[0U] = (curOffset & 0xFF00U) >> 8U;
            pageWrBuf[1U] = (curOffset & 0x00FFU);
            memcpy(&pageWrBuf[2U], buf + bytesWritten, curWriteLen);
            I2C_Transaction_init(&i2cTransaction);
            i2cTransaction.writeBuf     = pageWrBuf;
            i2cTransaction.writeCount   = 2U + curWriteLen;
            i2cTransaction.slaveAddress = i2cAddress;
            /* Perform write operation */
            status += I2C_transfer(object->i2cHandle, &i2cTransaction);
            if(SystemP_SUCCESS != status)
            {
                /* return if the write operation is not successful. */
                break;
            }

            /* After write operation flash will not respond for write cycle time.
             * This is approximately 4ms (min). */
            ClockP_usleep(4000);

            /* Wait for write to finish */
            I2C_Transaction_init(&i2cTransaction);
            i2cTransaction.writeBuf     = pageWrBuf;
            i2cTransaction.writeCount   = 2U;
            i2cTransaction.readBuf      = &dummyRead;
            i2cTransaction.readCount    = 1U;
            i2cTransaction.slaveAddress = i2cAddress;
            while(1U)
            {
                writeStatus = I2C_transfer(object->i2cHandle, &i2cTransaction);
                if(SystemP_SUCCESS == writeStatus)
                {
                    break;
                }
            }

            len          -= curWriteLen;
            curOffset    += curWriteLen;
            bytesWritten += curWriteLen;
        }

        SemaphoreP_post(&object->lockObj);
    }

    return (status);
}

