
#include <stdio.h>
//! [include]
#include <drivers/gpio.h>
//! [include]
#include <drivers/hw_include/cslr_soc.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>

uint32_t        gGpioBaseAddr = CSL_MSS_GIO_U_BASE;
uint32_t        gGpioPinNum = 26;
uint32_t        gGpioIntrNum = CSL_MSS_INTR_MSS_GIO_INT0;
uint32_t        gGpioIntrPinNum = 28;
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
    pinValue = GPIO_pinRead(gGpioBaseAddr, pinNum);
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

//! [interrupt]
static void GPIO_portIsrFxn(void *args)
{
    uint32_t    pinNum = (uint32_t) args;
    uint32_t    pendingInterrupt;

    pendingInterrupt = GPIO_getHighLowLevelPendingInterrupt(gGpioBaseAddr,GPIO_INTR_LEVEL_LOW);
    GPIO_clearInterrupt(gGpioBaseAddr, pinNum);

    if(pendingInterrupt )
    {
        /*
        * Handle all the expected  interrupts
        */
    }
}

void gpio_interrupt_init(void)
{
    int32_t         retVal;
    uint32_t        pinNum = gGpioIntrPinNum;
    HwiP_Params     hwiPrms;

    /* Interrupt setup */
    GPIO_setDirMode(gGpioBaseAddr, pinNum, GPIO_DIRECTION_INPUT);
    GPIO_ignoreOrHonorPolarity(gGpioBaseAddr, pinNum, GPIO_TRIG_TYPE_RISE_EDGE);
    GPIO_setTrigType(gGpioBaseAddr, pinNum, GPIO_TRIG_TYPE_RISE_EDGE);
    GPIO_markHighLowLevelInterrupt(gGpioBaseAddr, pinNum, GPIO_INTR_LEVEL_LOW);
    GPIO_enableInterrupt(gGpioBaseAddr, pinNum);

    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum = gGpioIntrNum;
    hwiPrms.callback = &GPIO_portIsrFxn;
    hwiPrms.args = (void *) pinNum;
    retVal = HwiP_construct(&gGpioHwiObject, &hwiPrms);
    if(SystemP_SUCCESS != retVal)
    {
        DebugP_assert(FALSE);
    }
}

void gpio_interrupt_deinit(void)
{
    uint32_t        pinNum = gGpioIntrPinNum, pendingInterrupt;

    /* Interrupt disable and clear any pending interrupts */
    GPIO_disableInterrupt(gGpioBaseAddr, pinNum);
    GPIO_setTrigType(gGpioBaseAddr, pinNum, GPIO_TRIG_TYPE_NONE);
    pendingInterrupt = GPIO_getHighLowLevelPendingInterrupt(gGpioBaseAddr,GPIO_INTR_LEVEL_LOW);
    (void)pendingInterrupt; /* kill warning about variable set but not used */
    GPIO_clearInterrupt(gGpioBaseAddr, pinNum);

    /* Unregister interrupt */
    HwiP_destruct(&gGpioHwiObject);
}
//! [interrupt]
