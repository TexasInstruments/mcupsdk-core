
#include <stdio.h>
//! [include]
#include <board/led.h>
//! [include]
#include <drivers/hw_include/cslr_soc.h>
#include <kernel/dpl/DebugP.h>

LED_Handle  ledHandle;

void led_on(void)
{
    int32_t     status;
//! [on]
    status = LED_on(ledHandle, 0U);
    DebugP_assert(SystemP_SUCCESS == status);
//! [on]
}

void led_off(void)
{
    int32_t     status;
//! [off]
    status = LED_off(ledHandle, 0U);
    DebugP_assert(SystemP_SUCCESS == status);
//! [off]
}
