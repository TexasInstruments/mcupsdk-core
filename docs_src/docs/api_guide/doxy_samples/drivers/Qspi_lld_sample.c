
//! [include]
#include <stdio.h>
#include <string.h>
#include <drivers/qspi/v0/lld/qspi_lld.h>
#include <kernel/dpl/MutexArmP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/nortos/dpl/r5/HwiP_armv7r_vim.h>
//! [include]
#define CONFIG_QSPI0               (0U)
#define CONFIG_QSPI_INSTANCES      (1U)
#define QSPI_NOR_PAGE_PROG         (0x02)
#define QSPI_ADDR_BYTES            (3U)
#define QSPI_NOR_CMD_SINGLE_READ   (0x03U)
#define CONFIG_QSPI_NUM_INSTANCES  (1U)

QSPILLD_Handle     gQspiHandle;
QSPILLD_Object     gQspiObject[CONFIG_QSPI_NUM_INSTANCES];
uint32_t intrNum;
uint32_t gQSPIVimStsAddr, intrNum, gQSPIVimStsClrMask, intcBaseAddr;

#define APP_QSPI_FLASH_OFFSET  (0x40000U)
#define APP_QSPI_DATA_SIZE     (256)
/* The source buffer used for transfer */
uint8_t gQspiTxBuf[APP_QSPI_DATA_SIZE];
uint8_t gQspiRxBuf[APP_QSPI_DATA_SIZE];
uint32_t transferMutex = MUTEX_ARM_LOCKED;

void isrCallback(void *args);
static __attribute__((__section__(".text.hwi"), noinline, naked, target("arm"), aligned(4))) void App_QSPI_ISR(void);

void open(void)
{
//! [open]
    int32_t status = QSPI_SYSTEM_SUCCESS;
    gQspiHandle = &gQspiObject[CONFIG_QSPI0];

    status = QSPI_lld_init(gQspiHandle);
    DebugP_assert(status == QSPI_SYSTEM_SUCCESS);
//! [open]
}

void close(void)
{
//! [close]
    int32_t status = QSPI_SYSTEM_SUCCESS;

    status = QSPI_lld_deInit(gQspiHandle);
    DebugP_assert(status == QSPI_SYSTEM_SUCCESS);
//! [close]
}

//! [transfer_nonblocking]
void transfer_nonblocking(void)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t itr = 0U;
    QSPILLD_WriteCmdParams msg = {0};

    /*  Interrupt configuration and registration*/
    gQspiHandle->interruptCallback = isrCallback;
    intrNum = gQspiHandle->hQspiInit->intrNum;
    intcBaseAddr = gHwiConfig.intcBaseAddr;
    gQSPIVimStsAddr = intcBaseAddr + (0x404u + (((intrNum)>> 5) & 0xFu) * 0x20u);
    gQSPIVimStsClrMask = 0x1u << ((intrNum) & 0x1Fu);

    /* Register Interrupt */
    HwiP_setPri(intrNum, 4U);
    HwiP_setVecAddr(intrNum, (uintptr_t)&App_QSPI_ISR);
    HwiP_enableInt(intrNum);

    /* Populating the command and Tx buffer*/
    msg.dataLen = APP_QSPI_DATA_SIZE;
    msg.dataBuf = gQspiTxBuf;
    msg.cmdAddr = APP_QSPI_FLASH_OFFSET;
    msg.cmd = QSPI_NOR_PAGE_PROG;
    msg.numAddrBytes = QSPI_ADDR_BYTES;
    status = QSPI_lld_writeCmdIntr(gQspiHandle,&msg);
    if (status == SystemP_SUCCESS )
    {
        while(try_lock_mutex(&transferMutex) == MUTEX_ARM_LOCKED);
    }
    else
    {
        DebugP_log("QSPI Write Fail");
    }
    /* Mutex Lock */
    transferMutex = MUTEX_ARM_LOCKED;

    msg.dataBuf = gQspiRxBuf;
    msg.cmd = QSPI_NOR_CMD_SINGLE_READ;
    msg.numAddrBytes = QSPI_ADDR_BYTES;

    QSPI_lld_readCmdIntr(gQspiHandle,&msg);
    if (status == SystemP_SUCCESS )
    {
        while(try_lock_mutex(&transferMutex) == MUTEX_ARM_LOCKED);
    }
    else
    {
        DebugP_log("QSPI Write Fail");
    }

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

    /* De-Register Interrupt */
    HwiP_setVecAddr(intrNum, 0);
    HwiP_disableInt(intrNum);
}
//! [transfer_nonblocking]

//! [isr_call]
static __attribute__((__section__(".text.hwi"), noinline, naked, target("arm"), aligned(4))) void App_QSPI_ISR(void)
{
    ISR_CALL_LEVEL_NONFLOAT_REENTRANT(QSPI_lld_isr, \
                                      gQspiHandle, \
                                      intrNum, \
                                      gQSPIVimStsAddr, \
                                      gQSPIVimStsClrMask,
                                      intcBaseAddr);
}
//! [isr_call]

//! [transfer_nonblocking_callback]
void isrCallback(void *args)
{
    unlock_mutex(&transferMutex);
}
//! [transfer_nonblocking_callback]

//! [blocking_writeRead]
void writeRead_blocking(void)
{
    int32_t status = SystemP_SUCCESS;
    QSPILLD_WriteCmdParams msg = {0};

    msg.cmd = QSPI_NOR_PAGE_PROG;
    msg.cmdAddr = APP_QSPI_FLASH_OFFSET;
    msg.dataLen = APP_QSPI_DATA_SIZE;
    msg.numAddrBytes = QSPI_ADDR_BYTES;
    msg.dataBuf = (void *)gQspiTxBuf;
    status = QSPI_lld_writeCmd(gQspiHandle, &msg);

    status = QSPI_lld_read(gQspiHandle,APP_QSPI_DATA_SIZE,gQspiRxBuf,APP_QSPI_FLASH_OFFSET,SystemP_WAIT_FOREVER);
    if(status == SystemP_SUCCESS)
    {
        DebugP_log("QSPI Read Success");
    }
    else
    {
        DebugP_log("QSPI Read Fail");
    }
}
//! [blocking_writeRead]