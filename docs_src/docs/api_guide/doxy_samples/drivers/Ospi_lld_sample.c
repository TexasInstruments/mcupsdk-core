
//! [include]
#include <stdio.h>
#include <drivers/ospi.h>
#include <string.h>
#include <drivers/ospi/v0/lld/ospi_lld.h>
#include <kernel/dpl/MutexArmP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/nortos/dpl/r5/HwiP_armv7r_vim.h>
//! [include]

#define CONFIG_OSPI0               (0U)
#define CONFIG_OSPI_INSTANCES      (1U)
#define OSPI_NOR_PAGE_PROG         (0x02)
#define OSPI_ADDR_BYTES            (3U)
#define OSPI_NOR_CMD_SINGLE_READ   (0x03U)
#define CONFIG_OSPI_NUM_INSTANCES  (1U)

OSPILLD_Handle     gOspiHandle;
OSPILLD_Object     gOspiObject[CONFIG_OSPI_NUM_INSTANCES];
uint32_t intrNum;
uint32_t gOSPIVimStsAddr, intrNum, gOSPIVimStsClrMask, intcBaseAddr;

#define APP_OSPI_FLASH_OFFSET  (0x200000U)
#define APP_OSPI_DATA_SIZE     (256)
/* The source buffer used for transfer */
uint8_t gOspiTxBuf[APP_OSPI_DATA_SIZE];
uint8_t gOspiRxBuf[APP_OSPI_DATA_SIZE];
uint32_t transferMutex = MUTEX_ARM_LOCKED;

void isrCallback(void *args);
static __attribute__((__section__(".text.hwi"), noinline, naked, target("arm"), aligned(4))) void App_OSPI_ISR(void);

void open(void)
{
//! [open]
    int32_t status = OSPI_SYSTEM_SUCCESS;
    gOspiHandle = &gOspiObject[CONFIG_OSPI0];

    status = OSPI_lld_init(gOspiHandle);
    DebugP_assert(status == OSPI_SYSTEM_SUCCESS);
//! [open]
}
void close(void)
{
//! [close]
    int32_t status = OSPI_SYSTEM_SUCCESS;

    status = OSPI_lld_deInit(gOspiHandle);
    DebugP_assert(status == OSPI_SYSTEM_SUCCESS);
//! [close]
}

//! [transfer_nonblocking]
void transfer_nonblocking(void)
{
    OSPI_Transaction transaction;
    uint32_t status = SystemP_SUCCESS;

    gOspiHandle->interruptCallback = isrCallback;
    intrNum = gOspiHandle->hOspiInit->intrNum;
    intcBaseAddr = gHwiConfig.intcBaseAddr;
    gOSPIVimStsAddr = intcBaseAddr + (0x404u + (((intrNum)>> 5) & 0xFu) * 0x20u);
    gOSPIVimStsClrMask = 0x1u << ((intrNum) & 0x1Fu);
    

    /* Register Interrupt */
    HwiP_setPri(intrNum, 4U);
    HwiP_setVecAddr(intrNum, (uintptr_t)&App_OSPI_ISR);
    HwiP_enableInt(intrNum);
    HwiP_setAsPulse(intrNum, FALSE);


    OSPI_lld_Transaction_init(&transaction);
    transaction.addrOffset = APP_OSPI_FLASH_OFFSET;
    transaction.buf = (void *)(gOspiTxBuf);
    transaction.count = APP_OSPI_DATA_SIZE;
    OSPI_lld_writeIndirect(gOspiHandle, &transaction);
    if (status == SystemP_SUCCESS )
    {
        while(try_lock_mutex(&transferMutex) == MUTEX_ARM_LOCKED);
    }
    else
    {
        DebugP_log("OSPI Write Fail");
    }
    /* Mutex Lock */
    transferMutex = MUTEX_ARM_LOCKED;
    transaction.buf = (void *)(gOspiRxBuf);
    status = OSPI_lld_readIndirect(gOspiHandle, &transaction);
    if (status == SystemP_SUCCESS )
    {
        while(try_lock_mutex(&transferMutex) == MUTEX_ARM_LOCKED);
    }
    else
    {
        DebugP_log("OSPI Read Fail");
    }

    /* De-Register Interrupt */
    HwiP_setVecAddr(intrNum, 0);
    HwiP_disableInt(intrNum);
}
//! [transfer_nonblocking]


//! [isr_call]
static __attribute__((__section__(".text.hwi"), noinline, naked, target("arm"), aligned(4))) void App_OSPI_ISR(void)
{
    ISR_CALL_LEVEL_NONFLOAT_REENTRANT(OSPI_lld_isr, \
                                      gOspiHandle, \
                                      intrNum, \
                                      gOSPIVimStsAddr, \
                                      gOSPIVimStsClrMask,
                                      intcBaseAddr);
}
//! [isr_call]

//! [transfer_nonblocking_callback]
void isrCallback(void *args)
{
    unlock_mutex(&transferMutex);
}
//! [transfer_nonblocking_callback]


