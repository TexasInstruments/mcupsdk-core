//! [include]
#include "string.h"
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/MutexArmP.h>
#include <kernel/nortos/dpl/r5/HwiP_armv7r_vim.h>
#include <drivers/uart/v0/lld/uart_lld.h>
//! [include]

#define CONFIG_UART0                 (0U)
#define APP_UART_BUFSIZE             (200U)
#define APP_UART_RECEIVE_BUFSIZE     (8U)
#define CSL_VIM_U_BASE			        (0x50F00000ul)

#define APP_UART_ASSERT_ON_FAILURE(transferOK, transaction) \
    do { \
        if((SystemP_SUCCESS != (transferOK)) || (UART_TRANSFER_STATUS_SUCCESS != transaction.status)) \
        { \
            DebugP_assert(FALSE); /* UART TX/RX failed!! */ \
        } \
    } while(0) \

UARTLLD_Handle     gUartHandle0;
UARTLLD_Object     gUartObject[CONFIG_UART0];
UARTLLD_InitObject gUartInitObject[CONFIG_UART0];

uint8_t gUartBuffer[APP_UART_BUFSIZE];
uint8_t gUartReceiveBuffer[APP_UART_RECEIVE_BUFSIZE];
volatile uint32_t gNumBytesRead = 0U, gNumBytesWritten = 0U;

uint32_t gUartVimStsAddr, intrNum;
uint32_t gUartVimStsClrMask;
uint32_t gVimBaseAddr = CSL_VIM_U_BASE;

//! [isr_call]
static __attribute__((__section__(".text.hwi"), noinline, naked, target("arm"), aligned(4))) void App_UART_ISR(void)
{
    ISR_CALL_LEVEL_NONFLOAT_REENTRANT(UART_lld_controllerIsr, \
                                      gUartHandle0, \
                                      intrNum, \
                                      gUartVimStsAddr, \
                                      gUartVimStsClrMask,
                                      gVimBaseAddr);
}
//! [isr_call]

//! [write_callback]
void UART_lld_writeCompleteCallback(void *args)
{
    unlock_mutex(gUartObject[CONFIG_UART0].writeTransferMutex);
}
//! [write_callback]

//! [read_callback]
void UART_lld_readCompleteCallback(void *args)
{
    unlock_mutex(gUartObject[CONFIG_UART0].readTransferMutex);
    return;
}
//! [read_callback]

void open(void)
{
//! [open]
    int32_t status = UART_STATUS_SUCCESS;
    gUartHandle0 = &gUartObject[CONFIG_UART0];

    status = UART_lld_init(gUartHandle0);
    DebugP_assert(status == UART_STATUS_SUCCESS);

//! [open]
}

void close(void)
{
//! [close]
    int32_t status = UART_STATUS_SUCCESS;

    status = UART_lld_deInit(gUartHandle0);
    DebugP_assert(status == UART_STATUS_SUCCESS);
//! [close]
}

void write_transfer_nonblocking(void)
{
//! [write_transfer_nonblocking]
    int32_t             transferOK;
    UART_Transaction    transaction;

    intrNum = gUartInitObject[CONFIG_UART0].intrNum;
    gUartVimStsAddr = gVimBaseAddr +  (0x404u + (((intrNum)>> 5) & 0xFu) * 0x20u);
    gUartVimStsClrMask = 0x1u << ((intrNum) & 0x1Fu);

    UART_lld_Transaction_init(&transaction);

    /* Send entry string */
    gNumBytesWritten = 0U;
    transaction.buf   = &gUartBuffer[0U];
    strncpy(transaction.buf,"This is uart echo test blocking mode\r\nReceives 8 characters then echo's back. Please input..\r\n", APP_UART_BUFSIZE);
    transaction.count = strlen(transaction.buf);
    transferOK = UART_lld_writeIntr(gUartHandle0, transaction.buf, transaction.count, NULL);

    while(try_lock_mutex(gUartObject[CONFIG_UART0].writeTransferMutex) == MUTEX_ARM_LOCKED);
    APP_UART_ASSERT_ON_FAILURE(transferOK, transaction);

    /* Deregister IRQ */
    HwiP_setVecAddr(intrNum, 0);
    HwiP_setPri(intrNum, 15);

//! [write_transfer_nonblocking]
}

void read_transfer_nonblocking(void)
{
//! [read_transfer_nonblocking]
    int32_t             transferOK;
    UART_Transaction    transaction;

    intrNum = gUartInitObject[CONFIG_UART0].intrNum;
    gUartVimStsAddr = gVimBaseAddr +  (0x404u + (((intrNum)>> 5) & 0xFu) * 0x20u);
    gUartVimStsClrMask = 0x1u << ((intrNum) & 0x1Fu);

    UART_lld_Transaction_init(&transaction);

    /* Read 8 chars */
    gNumBytesRead = 0U;
    transaction.buf   = &gUartReceiveBuffer[0U];
    transaction.count = APP_UART_RECEIVE_BUFSIZE;
    transferOK = UART_lld_readIntr(gUartHandle0, transaction.buf, transaction.count, NULL);

    while(try_lock_mutex(gUartObject[CONFIG_UART0].readTransferMutex) == MUTEX_ARM_LOCKED);
    APP_UART_ASSERT_ON_FAILURE(transferOK, transaction);

    /* Deregister IRQ */
    HwiP_setVecAddr(intrNum, 0);
    HwiP_setPri(intrNum, 15);

//! [read_transfer_nonblocking]
}