//! [include]
#include <kernel/dpl/DebugP.h>
#include <string.h>
#include <kernel/dpl/MutexArmP.h>
#include <kernel/nortos/dpl/r5/HwiP_armv7r_vim.h>
#include <drivers/mmcsd/v0/mmcsd.h>
//! [include]

#define APP_MMCSD_START_BLK             (0x300000U) /* @1.5GB */
#define APP_MMCSD_DATA_SIZE             (4096)

uint8_t gMmcsdTxBuf[APP_MMCSD_DATA_SIZE] __attribute__((aligned(128U)));
uint8_t gMmcsdRxBuf[APP_MMCSD_DATA_SIZE] __attribute__((aligned(128U)));


MMCSDLLD_Handle           gMmcsdLldHandle;
MMCSDLLD_Object           gMmcsdLldObject;
MMCSDLLD_InitObject       gMmcsdLldInitObject;

uint32_t writeTicks = 0U, readTicks = 0U;
uint32_t startTicks = 0;

void mmcsd_io_fill_buffers(void);

/* MMCSD interrupt Priority */
#define MMCSD_INTERRUPT_PRIORITY        (4U)

MMCSDLLD_Handle gMmcsdLldHandle;
uint32_t gMmcsdVimStsAddr, intrNum, gMmcsdVimStsClrMask, intcBaseAddr;
uint32_t gMMCSDTransferMutex = MUTEX_ARM_UNLOCKED;

extern HwiP_Config gHwiConfig;

void Drivers_open(void);
void Drivers_close(void);

/* Transfer Complete Callback Function Declaration */
void MMCSD_lld_transferCompleteCallback_implementation (void * args, int32_t transferStatus);

static __attribute__((__section__(".text.hwi"), noinline, naked, target("arm"), aligned(4))) void App_MMCSD_ISR(void);

void mmcsd_raw_io_sd_intr_lld_main(void *args)
{
    Drivers_open();

    //! [transfer_nonblocking]
    gMmcsdLldHandle = (MMCSDLLD_Handle)(gMmcsdLldHandle);
    intrNum = gMmcsdLldHandle->initHandle->intrNum;
    intcBaseAddr = gHwiConfig.intcBaseAddr;
    gMmcsdVimStsAddr = intcBaseAddr + (0x404u + (((intrNum)>> 5) & 0xFu)*0x20u);
    gMmcsdVimStsClrMask = 0x1u << ((intrNum) & 0x1Fu);

    /* Assign Transfer Complete Callback Function */
    gMmcsdLldHandle->transferCompleteCallback = MMCSD_lld_transferCompleteCallback_implementation;

    /* Register Interrupt */
    HwiP_setVecAddr(intrNum, (uintptr_t)&App_MMCSD_ISR);
    HwiP_setPri(intrNum, MMCSD_INTERRUPT_PRIORITY);
    HwiP_enableInt(intrNum);

    int32_t     status = MMCSD_STS_SUCCESS;
    uint32_t    blockSize = MMCSD_lld_getBlockSize(gMmcsdLldHandle);
    uint32_t    numBlocks = APP_MMCSD_DATA_SIZE / blockSize;

    if((APP_MMCSD_DATA_SIZE % blockSize) != 0)
    {
        numBlocks += 1;
    }

    /* Fill Write and Read Buffer */
    mmcsd_io_fill_buffers();

    /* Lock Mutex */
    gMMCSDTransferMutex = MUTEX_ARM_LOCKED;
    /* Initiate Transfer */
    status = MMCSD_lld_write_SD_Intr(gMmcsdLldHandle, gMmcsdTxBuf,
                                     APP_MMCSD_START_BLK, numBlocks);
    /* Wait for Mutex to unlock */
    while(try_lock_mutex(&gMMCSDTransferMutex) == MUTEX_ARM_LOCKED);
    //! [transfer_nonblocking]

    if(status == MMCSD_STS_SUCCESS)
    {
        /* Lock Mutex */
        gMMCSDTransferMutex = MUTEX_ARM_LOCKED;
        /* Initiate Transfer */
        status = MMCSD_lld_read_SD_Intr(gMmcsdLldHandle, gMmcsdRxBuf,
                                        APP_MMCSD_START_BLK, numBlocks);
        /* Wait for Mutex to unlock */
        while(try_lock_mutex(&gMMCSDTransferMutex) == MUTEX_ARM_LOCKED);
    }

    if(status == MMCSD_STS_SUCCESS)
    {
        status = memcmp(gMmcsdRxBuf, gMmcsdTxBuf, APP_MMCSD_DATA_SIZE);
    }

    if(status == MMCSD_STS_SUCCESS)
    {
        DebugP_log("Data Matched !!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Data did not Match !!\r\n");
        DebugP_log("Some tests have failed!!\r\n");
    }

    Drivers_close();

    return;
}

//! [isr_call]
static __attribute__((__section__(".text.hwi"), noinline, naked, target("arm"), aligned(4))) void App_MMCSD_ISR(void)
{
    ISR_CALL_LEVEL_NONFLOAT_REENTRANT(MMCSD_lld_Isr, \
                                      gMmcsdLldHandle, \
                                      intrNum, \
                                      gMmcsdVimStsAddr, \
                                      gMmcsdVimStsClrMask,
                                      intcBaseAddr);
}
//! [isr_call]

//! [transfer_callback]
void MMCSD_lld_transferCompleteCallback_implementation (void * args,
                                                        int32_t transferStatus)
{
    unlock_mutex(&gMMCSDTransferMutex);
}
//! [transfer_callback]

void Drivers_open(void)
{
//! [open]
    int32_t status = MMCSD_STS_SUCCESS;
    gMmcsdLldHandle = (MMCSDLLD_Handle)(&gMmcsdLldObject);
    gMmcsdLldHandle->initHandle = (MMCSDLLD_InitHandle)(&gMmcsdLldInitObject);

    status = MMCSD_lld_init(gMmcsdLldHandle);
    DebugP_assert(status == MMCSD_STS_SUCCESS);

//! [open]
}

void Driver_close(void)
{
//! [close]
    int32_t status = MMCSD_STS_SUCCESS;

    status = MMCSD_lld_deInit(gMmcsdLldHandle);
    DebugP_assert(status == MMCSD_STS_SUCCESS);
//! [close]
}

void mmcsd_io_fill_buffers(void)
{
    uint32_t i;

    for(i = 0U; i < APP_MMCSD_DATA_SIZE; i++)
    {
        gMmcsdTxBuf[i] = i % 256;
        gMmcsdRxBuf[i] = 0U;
    }
}
