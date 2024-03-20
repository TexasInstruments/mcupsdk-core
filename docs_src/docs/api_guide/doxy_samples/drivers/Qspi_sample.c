
//! [include]
#include <stdio.h>
#include <string.h>
#include <drivers/qspi.h>
#include <kernel/dpl/MutexArmP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/nortos/dpl/r5/HwiP_armv7r_vim.h>
//! [include]

QSPI_Handle gQspiHandle;
#define CONFIG_QSPI0               (0U)
#define APP_QSPI_FLASH_OFFSET      (0x40000U)
#define APP_QSPI_DATA_SIZE         (256)
#define QSPI_ADDR_BYTES            (3U)
#define QSPI_Timeout               (10000)
#define QSPI_NOR_PAGE_PROG         (0x02)
#define QSPI_NOR_CMD_SINGLE_READ   (0x03U)
/* The source buffer used for transfer */
uint8_t gQspiTxBuf[APP_QSPI_DATA_SIZE];
uint8_t gQspiRxBuf[APP_QSPI_DATA_SIZE];

void open(void)
{
//! [open]
    QSPI_Params qspiParams;

    QSPI_Params_init(&qspiParams);
    gQspiHandle = QSPI_open(CONFIG_QSPI0, &qspiParams);
    DebugP_assert(gQspiHandle != NULL);
//! [open]
}

void close(void)
{
//! [close]
    QSPI_close(gQspiHandle);
//! [close]
}

//! [transfer_blocking]
void transfer_blocking(void)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t itr = 0U;
    QSPI_Transaction transaction;

    QSPI_transaction_init(&transaction);
    
    transaction.addrOffset = APP_QSPI_FLASH_OFFSET;
    transaction.buf = (void *)gQspiTxBuf;
    transaction.count = APP_QSPI_DATA_SIZE;
    transaction.transferTimeout = QSPI_Timeout;
    
    status = QSPI_writeConfigMode(gQspiHandle, &transaction);

    transaction.buf = (void *)gQspiRxBuf;

    status = QSPI_readMemMapMode(gQspiHandle,&transaction);

    for(itr = 0U; itr < APP_QSPI_DATA_SIZE; itr++)
    {
        if(gQspiTxBuf[itr] != gQspiRxBuf[itr])
        {
            status = SystemP_FAILURE;
            DebugP_logError("QSPI read data mismatch %d !!!\r\n",itr);
            break;
        }
    }

    if (status == SystemP_SUCCESS)
    {
        DebugP_log("QSPI Write and Read PASS");
    }
}
//! [transfer_blocking]

//! [transfer_nonblocking]
void transfer_nonblocking(void)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t itr = 0U;
    QSPI_WriteCmdParams wrParams;
    QSPI_ReadCmdParams rdParams;

    /* Read and Write Initialization */
    QSPI_readCmdParams_init(&rdParams);
    QSPI_writeCmdParams_init(&wrParams);

    /* Populate the write commands */
    wrParams.cmd = QSPI_NOR_PAGE_PROG;
    wrParams.cmdAddr = APP_QSPI_FLASH_OFFSET;
    wrParams.numAddrBytes = QSPI_ADDR_BYTES;
    wrParams.txDataBuf = (void *)(gQspiTxBuf);
    wrParams.txDataLen = APP_QSPI_DATA_SIZE;

    QSPI_writeConfigModeIntr(gQspiHandle,&wrParams);

    /* Populate the Read commands */
    rdParams.cmd = QSPI_NOR_CMD_SINGLE_READ;
    rdParams.cmdAddr = APP_QSPI_FLASH_OFFSET;
    rdParams.numAddrBytes = QSPI_ADDR_BYTES;
    rdParams.rxDataBuf = (void *)(gQspiRxBuf);
    rdParams.rxDataLen = APP_QSPI_DATA_SIZE;

    QSPI_readConfigModeIntr(gQspiHandle,&rdParams);

    for(itr = 0U; itr < APP_QSPI_DATA_SIZE; itr++)
    {
        if(gQspiTxBuf[itr] != gQspiRxBuf[itr])
        {
            status = SystemP_FAILURE;
            DebugP_logError("QSPI read data mismatch %d !!!\r\n",itr);
            break;
        }
    }
    if (status == SystemP_SUCCESS)
    {
        DebugP_log("QSPI Write and Read PASS");
    }
}
//! [transfer_nonblocking]