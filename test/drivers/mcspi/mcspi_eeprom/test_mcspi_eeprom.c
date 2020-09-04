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

#include <kernel/dpl/DebugP.h>
#include "ti_drivers_open_close.h"

/* Internal Function declarations. */
static int32_t eeprom_read(uint32_t address, uint32_t *data);
static int32_t eeprom_erase_write_enable(void);
static int32_t eeprom_write(uint32_t address, uint32_t data);
static int32_t eeprom_write_all(uint32_t data);
static int32_t eeprom_spi_transfer(uint32_t dataTx, uint32_t *dataRx);

/* EEPROM Commands. */
#define MCSPI_EEPROM_READ           (0x6U)
#define MCSPI_EEPROM_WRITE          (0x7U)
#define MCSPI_EEPROM_ERASE          (0x6U)
#define MCSPI_EEPROM_OTHER          (0x4U)

#define MCSPI_EEPROM_ERAL_ADDR      (0x20U)
#define MCSPI_EEPROM_EWDS_ADDR      (0x00U)
#define MCSPI_EEPROM_EWEN_ADDR      (0x30U)
#define MCSPI_EEPROM_WRAL_ADDR      (0x10U)

#define MCSPI_EEPROM_APP_DATA_PATTERN   (0x1234)
void *mcspi_eeprom_main(void *args)
{
    int32_t             status = SystemP_SUCCESS;
    uint32_t            i;
    uint32_t            dataRx[64];

    Drivers_open();

    DebugP_log("[MCSPI] Eeprom example started ...\r\n");

    /* Enable erase and write operations. */
    status = eeprom_erase_write_enable();
    DebugP_assert(SystemP_SUCCESS == status);

    if (status == SystemP_SUCCESS)
    {
        DebugP_log("Eeprom Write all with same data pattern: 0x%x...\r\n", MCSPI_EEPROM_APP_DATA_PATTERN);
        status = eeprom_write_all(MCSPI_EEPROM_APP_DATA_PATTERN);
        DebugP_assert(SystemP_SUCCESS == status);
    }

    if (status == SystemP_SUCCESS)
    {
        /* Read the data from the flash. */
        for (i= 0; i<64; i++)
        {
            status = eeprom_read(i, &dataRx[i]);
            DebugP_assert(SystemP_SUCCESS == status);
        }
    }

    if (status == SystemP_SUCCESS)
    {
        /* Verify the data. */
        for (i= 0; i<64; i++)
        {
            if (dataRx[i] != MCSPI_EEPROM_APP_DATA_PATTERN)
            {
                DebugP_log("Eeprom Data mismatch found at offset %d...\r\n", i);
                status = SystemP_FAILURE;
                DebugP_assert(SystemP_SUCCESS == status);
                break;
            }
        }
    }

    if(SystemP_SUCCESS == status)
    {
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    Drivers_close();

    return NULL;
}

static int32_t eeprom_read(uint32_t address, uint32_t *data)
{
    uint32_t            dataTx = 0U;
    int32_t             transferStatus;

    dataTx = (MCSPI_EEPROM_READ << 29) | (address & 0x3F) << 23;
    *data = 0;

    transferStatus = eeprom_spi_transfer(dataTx, data);

    *data = (*data >> 6) & 0xFFFFU;

    return transferStatus;
}

static int32_t eeprom_erase_write_enable(void)
{
    uint32_t            dataTx = 0U;
    int32_t             transferStatus;

    dataTx = (MCSPI_EEPROM_OTHER << 29) | (MCSPI_EEPROM_EWEN_ADDR & 0x3F) << 23;

    transferStatus = eeprom_spi_transfer(dataTx, NULL);
    {
        volatile int32_t delayCnt = 0xff;
        while (delayCnt-- > 0x00);
    }

    return transferStatus;
}

static int32_t eeprom_write_all(uint32_t data)
{
    uint32_t            dataTx = 0U, statusRx = 0U;
    int32_t             transferStatus;
    int32_t             retryCnt = 0xFF;

    dataTx = (MCSPI_EEPROM_OTHER << 29) | ((MCSPI_EEPROM_WRAL_ADDR & 0x3F) << 23) | (data & 0xFFFF) << 7;

    transferStatus = eeprom_spi_transfer(dataTx, NULL);

    {
        volatile int32_t delayCnt = 0xff;
        while (delayCnt-- > 0x00);
    }
    
    if (transferStatus == SystemP_SUCCESS)
    {
        /* Wait for the write to complete. */
        dataTx = 0;
        while(retryCnt-- > 0)
        {
            transferStatus = eeprom_spi_transfer(dataTx, &statusRx);
            if (transferStatus != SystemP_SUCCESS)
            {
                /* transfer failed. */
                break;
            }
            if (statusRx != 0)
            {
                /* Write successful. */
                break;
            }
        }
        if (retryCnt <= 0)
        {
            DebugP_log("Write un successful!!\r\n");
            transferStatus = SystemP_FAILURE;
        }
    }

    return transferStatus;
}

static int32_t eeprom_spi_transfer(uint32_t dataTx, uint32_t *dataRx)
{
    int32_t             transferStatus;
    MCSPI_Transaction   spiTransaction;

    /* Initiate transfer */
    spiTransaction.channel  = 0U;
    spiTransaction.count    = 1U;
    spiTransaction.dataSize  = 32;
    spiTransaction.csDisable = TRUE;
    spiTransaction.txBuf    = (void *)&dataTx;
    spiTransaction.rxBuf    = (void *)dataRx;
    spiTransaction.args     = NULL;
    transferStatus = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI0], &spiTransaction);
    if((SystemP_SUCCESS != transferStatus) ||
       (MCSPI_TRANSFER_COMPLETED != spiTransaction.status))
    {
        DebugP_log("Eeprom read failed!!\r\n");
    }
    return transferStatus;
}

