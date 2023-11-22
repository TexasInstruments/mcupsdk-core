//! [include]
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/MutexArmP.h>
#include <kernel/nortos/dpl/r5/HwiP_armv7r_vim.h>
#include <drivers/mcspi/v0/lld/mcspi_lld.h>
//! [include]
#define CONFIG_MCSPI0                   (0U)
#define CONFIG_MCSPI_NUM_INSTANCES      (1U)
#define CONFIG_MCSPI0_NUM_CH            (1U)
#define APP_MCSPI_MSGSIZE               (8U)
#define CSL_VIM_U_BASE			        (0x50F00000ul)

MCSPILLD_Handle     gMcspiHandle0;
MCSPILLD_Object     gMcspiObject[CONFIG_MCSPI_NUM_INSTANCES];
MCSPILLD_Handle     gMcspiHandle[CONFIG_MCSPI_NUM_INSTANCES];
MCSPILLD_InitObject gConfigMcspi0InitObject[CONFIG_MCSPI0_NUM_CH];
MCSPI_ChConfig      gConfigMcspi0ChCfg[CONFIG_MCSPI0_NUM_CH];

uint32_t gMutexLockUnlock = MUTEX_ARM_LOCKED;
uint8_t  gMcspiTxBuffer[APP_MCSPI_MSGSIZE];
uint8_t  gMcspiRxBuffer[APP_MCSPI_MSGSIZE];
uint32_t gMcspiVimStsAddr, intrNum;
uint32_t gMcspiVimStsClrMask;
uint32_t gVimBaseAddr = CSL_VIM_U_BASE;

//! [isr_call]
static __attribute__((__section__(".text.hwi"), noinline, naked, target("arm"), aligned(4))) void App_MCSPI_ISR(void)
{
    ISR_CALL_LEVEL_NONFLOAT_REENTRANT(MCSPI_lld_controllerIsr, \
                                      gMcspiHandle0, \
                                      intrNum, \
                                      gMcspiVimStsAddr, \
                                      gMcspiVimStsClrMask,
                                      gVimBaseAddr);
}
//! [isr_call]

//! [transfer_callback]
void MCSPI_lld_transferCallback(void *args, uint32_t transferStatus)
{
    unlock_mutex(gMcspiObject[CONFIG_MCSPI0].transferMutex);
}
//! [transfer_callback]

void MCSPI_lld_errorCallback(void *args)
{
    return;
}

void open(void)
{
//! [open]
    int32_t status = MCSPI_STATUS_SUCCESS;
    gMcspiHandle0 = &gMcspiObject[CONFIG_MCSPI0];

    status = MCSPI_lld_init(gMcspiHandle0);
    DebugP_assert(status == MCSPI_STATUS_SUCCESS);

//! [open]
}

void close(void)
{
//! [close]
    int32_t status = MCSPI_STATUS_SUCCESS;

    status = MCSPI_lld_deInit(gMcspiHandle0);
    DebugP_assert(status == MCSPI_STATUS_SUCCESS);
//! [close]
}

void transfer_nonblocking(void)
{
//! [transfer_nonblocking]
    uint32_t              bufIndex;
    uint32_t              count;
    uint32_t              timeout = MCSPI_WAIT_FOREVER;
    int32_t               status = MCSPI_STATUS_SUCCESS;
    MCSPI_ExtendedParams  extendedParams;

    /*  Interrupt configuration and registration*/
    intrNum = gConfigMcspi0InitObject[CONFIG_MCSPI0].intrNum;
    gMcspiVimStsAddr = gVimBaseAddr + (0x404u + (((intrNum)>> 5) & 0xFu) * 0x20u);
    gMcspiVimStsClrMask = 0x1u << ((intrNum) & 0x1Fu);

    HwiP_setVecAddr(intrNum, (uintptr_t)&App_MCSPI_ISR);
    HwiP_setPri(intrNum, gConfigMcspi0InitObject[CONFIG_MCSPI0].intrPriority);
    HwiP_enableInt(intrNum);

    if(status == MCSPI_STATUS_SUCCESS)
    {
        DebugP_log("\n[MCSPI] Loopback example started ...\r\n");

        /* Memfill buffers */
        for(bufIndex = 0U; bufIndex < APP_MCSPI_MSGSIZE; bufIndex++)
        {
            gMcspiTxBuffer[bufIndex] = bufIndex;
            gMcspiRxBuffer[bufIndex] = 0U;
        }

        gMcspiObject[CONFIG_MCSPI0].transferMutex = &gMutexLockUnlock;

        /* populate extended parameters */
        extendedParams.channel    = 0;
        extendedParams.csDisable  = TRUE;
        extendedParams.dataSize   = 32;
        count = APP_MCSPI_MSGSIZE / (extendedParams.dataSize/8);

        status = MCSPI_lld_readWriteIntr(gMcspiHandle0, \
                                         gMcspiTxBuffer, \
                                         &gMcspiRxBuffer, \
                                         count, \
                                         timeout, \
                                         &extendedParams);
        while(try_lock_mutex(gMcspiObject[CONFIG_MCSPI0].transferMutex) == MUTEX_ARM_LOCKED);

        if(MCSPI_TRANSFER_COMPLETED != status)
        {
            DebugP_assert(FALSE); /* MCSPI transfer failed!! */
        }
        else
        {
            /* Compare data */
            for(bufIndex = 0U; bufIndex < APP_MCSPI_MSGSIZE; bufIndex++)
            {
                if(gMcspiTxBuffer[bufIndex] != gMcspiRxBuffer[bufIndex])
                {
                    status = MCSPI_STATUS_FAILURE;   /* Data mismatch */
                    DebugP_log("Data Mismatch at offset %d\r\n", bufIndex);
                    break;
                }
            }
        }
    }

    /* Deregister IRQ */
    HwiP_setVecAddr(intrNum, 0);
    HwiP_setPri(intrNum, 15);

    if(MCSPI_STATUS_SUCCESS == status)
    {
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

//! [transfer_nonblocking]
    return;
}
