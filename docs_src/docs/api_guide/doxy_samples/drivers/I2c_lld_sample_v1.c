//! [include]
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/MutexArmP.h>
#include <kernel/nortos/dpl/r5/HwiP_armv7r_vim.h>
#include <drivers/i2c/v1/lld/i2c_lld.h>
//! [include]

/* Sub address command as per TPIC2810 datasheet */
#define TPIC2810_CMD_RD                 (0x11U)
#define TPIC2810_CMD_WR_SHIFT_REG       (0x11U)
#define TPIC2810_CMD_LOAD_OUTPUT        (0x22U)
#define TPIC2810_CMD_WR_IMMEDIATE       (0x44U)

/* I2C Interrupt Priority */
#define I2C_INTERRUPT_PRIORITY          (4U)

/* LED States */
#define LED_SET_ALL                     (0xFFU)
#define LED_RESET_ALL                   (0x00U)

#define CONFIG_I2CLLD0                  (0U)

I2CLLD_Handle           gI2cLldHandle0;
I2CLLD_Object           gI2cLldObjects[CONFIG_I2CLLD0];

uint32_t                gI2cVimStsAddr, gI2cVimStsClrMask, intcBaseAddr;
uint32_t                gI2CTransferMutex = MUTEX_ARM_UNLOCKED;
uint32_t                intrNum;
uint32_t                deviceAddress;

I2CLLD_Transaction      i2cTransaction;
I2CLLD_Message          i2cMsg;

//! [isr_call]
static __attribute__((__section__(".text.hwi"), noinline, naked, target("arm"), aligned(4))) void App_I2C_ISR(void)
{
    ISR_CALL_LEVEL_NONFLOAT_REENTRANT(I2C_lld_controllerIsr, \
                                      gI2cLldHandle0, \
                                      intrNum, \
                                      gI2cVimStsAddr, \
                                      gI2cVimStsClrMask,
                                      intcBaseAddr);
}
//! [isr_call]

//! [transfer_callback]
void I2C_lld_transferCompleteCallback_implementation(void * args, const I2CLLD_Message * msg, int32_t transferStatus)
{
    unlock_mutex(&gI2CTransferMutex);
}
//! [transfer_callback]


void open(void)
{
//! [open]
    int32_t status = I2C_STS_SUCCESS;
    gI2cLldHandle0 = (I2CLLD_Handle)(&gI2cLldObjects[CONFIG_I2CLLD0]);

    status = I2C_lld_init(gI2cLldHandle0);
    DebugP_assert(status == I2C_STS_SUCCESS);

//! [open]
}

void close(void)
{
//! [close]
    int32_t status = I2C_STS_SUCCESS;

    status = I2C_lld_deInit(gI2cLldHandle0);
    DebugP_assert(status == I2C_STS_SUCCESS);
//! [close]
}
void transfer_nonblocking(void)
{
//! [transfer_nonblocking]
    int32_t             status = I2C_STS_SUCCESS;
    uint8_t             wrData[2U];

    /*  Interrupt configuration and registration*/
    intrNum = gI2cLldHandle0->intrNum;

    gI2cLldHandle0 = (I2CLLD_Handle)(&gI2cLldObjects[CONFIG_I2CLLD0]);
    intrNum = gI2cLldHandle0->intrNum;
    intcBaseAddr = gHwiConfig.intcBaseAddr;
    gI2cVimStsAddr = intcBaseAddr + (0x404u + (((intrNum)>> 5) & 0xFu) * 0x20u);
    gI2cVimStsClrMask = 0x1u << ((intrNum) & 0x1Fu);

    DebugP_log("I2C LED Blink Test Started ...\r\n");

    /* Assign Transfer Complete Callback Function */
    gI2cLldHandle0->transferCompleteCallback = I2C_lld_transferCompleteCallback_implementation;

    /* Register Interrupt */
    HwiP_setVecAddr(intrNum, (uintptr_t)&App_I2C_ISR);
    HwiP_setPri(intrNum, I2C_INTERRUPT_PRIORITY);
    HwiP_enableInt(intrNum);

    /* Set mask */
    wrData[0U] = TPIC2810_CMD_WR_IMMEDIATE;
    wrData[1U] = (uint8_t) (LED_SET_ALL);
    /* Initialize transaction object */
    I2C_lld_Transaction_init(&i2cTransaction);
    i2cTransaction.writeBuf = &wrData[0U];
    i2cTransaction.writeCount = 2U;
    /* Initialize message object */
    I2C_lld_Message_init(&i2cMsg);
    i2cMsg.txn = &i2cTransaction;
    i2cMsg.txnCount = 1U;
    i2cMsg.targetAddress = deviceAddress;

    /* Lock Mutex */
    gI2CTransferMutex = MUTEX_ARM_LOCKED;
    /* Start Transfer in interrupt Mode */
    status = I2C_lld_transferIntr(gI2cLldHandle0, &i2cMsg);
    /* Wait for the Mutex to Unlock */
    while(try_lock_mutex(&gI2CTransferMutex) == MUTEX_ARM_LOCKED);

    if(I2C_STS_SUCCESS != status)
    {
        DebugP_assert(FALSE); /* I2C transfer failed!! */
    }

    /* Deregister IRQ */
    HwiP_setVecAddr(intrNum, 0);
    HwiP_disableInt(intrNum);

    if(I2C_STS_SUCCESS == status)
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
