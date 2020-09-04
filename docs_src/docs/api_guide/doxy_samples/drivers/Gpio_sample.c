
#include <stdio.h>
//! [include]
#include <drivers/gpio.h>
//! [include]
#include <drivers/hw_include/cslr_soc.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>

uint32_t        gGpioBaseAddr = CSL_GPIO0_BASE;
uint32_t        gGpioPinNum = 1;
uint32_t        gGpioBankIntrNum = CSLR_R5FSS0_CORE0_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_0;
uint32_t        gGpioPinIntrNum = CSLR_R5FSS0_CORE0_INTR_MAIN_GPIOMUX_INTROUTER0_OUTP_0;
HwiP_Object     gGpioHwiObject;

void gpio_output(void)
{
//! [output]
    uint32_t pinNum = gGpioPinNum, pinValue;

    GPIO_setDirMode(gGpioBaseAddr, pinNum, GPIO_DIRECTION_OUTPUT);

    GPIO_pinWriteHigh(gGpioBaseAddr, pinNum);
    ClockP_sleep(1);
    GPIO_pinWriteLow(gGpioBaseAddr, pinNum);

    /* Check output value */
    pinValue = GPIO_pinOutValueRead(gGpioBaseAddr, pinNum);
    if(pinValue != GPIO_PIN_LOW)
    {
        DebugP_assert(FALSE);
    }
//! [output]
}

void gpio_input(void)
{
//! [input]
    uint32_t pinNum = gGpioPinNum, pinValue;

    GPIO_setDirMode(gGpioBaseAddr, pinNum, GPIO_DIRECTION_INPUT);

    pinValue = GPIO_pinRead(gGpioBaseAddr, pinNum);
    if(pinValue == GPIO_PIN_HIGH)
    {
        DebugP_log("Pin value is HIGH\r\n");
    }
    else
    {
        DebugP_log("Pin value is LOW\r\n");
    }
//! [input]
}

//! [bank_interrupt]
static void GPIO_bankIsrFxn(void *args)
{
    uint32_t    pinNum = (uint32_t) args, bankNum;
    uint32_t    intrStatus, pinMask = GPIO_GET_BANK_BIT_MASK(pinNum);

    bankNum = GPIO_GET_BANK_INDEX(pinNum);
    intrStatus = GPIO_getBankIntrStatus(gGpioBaseAddr, bankNum);
    GPIO_clearBankIntrStatus(gGpioBaseAddr, bankNum, intrStatus);

    if(intrStatus & pinMask)
    {
        /*
        * Handle all the expected pin interrupts within a bank using intrStatus flag
        */
    }
}

void gpio_bank_interrupt_init(void)
{
    int32_t         retVal;
    uint32_t        pinNum = gGpioPinNum, bankNum;
    HwiP_Params     hwiPrms;

    bankNum = GPIO_GET_BANK_INDEX(pinNum);

    /* Interrupt setup */
    GPIO_setDirMode(gGpioBaseAddr, pinNum, GPIO_DIRECTION_INPUT);
    GPIO_setTrigType(gGpioBaseAddr, pinNum, GPIO_TRIG_TYPE_RISE_EDGE);
    GPIO_bankIntrEnable(gGpioBaseAddr, bankNum);

    /* Register bank interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum = gGpioBankIntrNum;
    hwiPrms.callback = &GPIO_bankIsrFxn;
    hwiPrms.args = (void *) pinNum;
    retVal = HwiP_construct(&gGpioHwiObject, &hwiPrms);
    if(SystemP_SUCCESS != retVal)
    {
        DebugP_assert(FALSE);
    }
}

void gpio_bank_interrupt_deinit(void)
{
    uint32_t        pinNum = gGpioPinNum, bankNum, intrStatus;

    bankNum = GPIO_GET_BANK_INDEX(pinNum);

    /* Interrupt disable and clear any pending interrupts */
    GPIO_bankIntrDisable(gGpioBaseAddr, bankNum);
    GPIO_setTrigType(gGpioBaseAddr, pinNum, GPIO_TRIG_TYPE_NONE);
    intrStatus = GPIO_getBankIntrStatus(gGpioBaseAddr, bankNum);
    GPIO_clearBankIntrStatus(gGpioBaseAddr, bankNum, intrStatus);

    /* Unregister interrupt */
    HwiP_destruct(&gGpioHwiObject);
}
//! [bank_interrupt]

//! [pin_interrupt]
static void GPIO_pinIsrFxn(void *args)
{
    /*
     * Handle pin interrupt - This is pulse interrupt. No need to clear status
     */
}

void gpio_pin_interrupt_init(void)
{
    int32_t         retVal;
    uint32_t        pinNum = gGpioPinNum, bankNum;
    HwiP_Params     hwiPrms;

    bankNum = GPIO_GET_BANK_INDEX(pinNum);

    /* Interrupt setup */
    GPIO_setDirMode(gGpioBaseAddr, pinNum, GPIO_DIRECTION_INPUT);
    GPIO_setTrigType(gGpioBaseAddr, pinNum, GPIO_TRIG_TYPE_RISE_EDGE);
    GPIO_bankIntrEnable(gGpioBaseAddr, bankNum);

    /* Register pin interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum = gGpioPinIntrNum;
    hwiPrms.callback = &GPIO_pinIsrFxn;
    hwiPrms.args = (void *) pinNum;
    retVal = HwiP_construct(&gGpioHwiObject, &hwiPrms);
    if(SystemP_SUCCESS != retVal)
    {
        DebugP_assert(FALSE);
    }
}

void gpio_pin_interrupt_deinit(void)
{
    uint32_t        pinNum = gGpioPinNum, bankNum;

    bankNum = GPIO_GET_BANK_INDEX(pinNum);

    /* Interrupt disable and clear any pending interrupts */
    GPIO_bankIntrDisable(gGpioBaseAddr, bankNum);
    GPIO_setTrigType(gGpioBaseAddr, pinNum, GPIO_TRIG_TYPE_NONE);
    GPIO_clearIntrStatus(gGpioBaseAddr, pinNum);

    /* Unregister interrupt */
    HwiP_destruct(&gGpioHwiObject);
}
//! [pin_interrupt]
