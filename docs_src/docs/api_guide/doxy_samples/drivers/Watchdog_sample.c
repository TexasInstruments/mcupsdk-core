
#include <stdio.h>
#include <kernel/dpl/DebugP.h>
//! [include]
#include <drivers/watchdog.h>
//! [include]
#include <drivers/hw_include/cslr_soc.h>

#define CONFIG_WDT0            (0U)

uint32_t watchdogBaseAddr = CSL_WDT0_U_BASE;

Watchdog_Handle                gWatchdogHandle;

void open(void)
{
//! [open]
    Watchdog_Params      params;

    Watchdog_paramsInit(&params);
    params.resetMode  = Watchdog_RESET_ON;
    gWatchdogHandle = Watchdog_open(CONFIG_WDT0, &params);
    if (!gWatchdogHandle) {
        DebugP_assert(FALSE);
    }
//! [open]
}

void close(void)
{
//! [close]
    Watchdog_close(gWatchdogHandle);
//! [close]
}

void Watchdog_service(void)
{
//! [Watchdog_service]
    Watchdog_clear(gWatchdogHandle);
//! [Watchdog_service]
}
