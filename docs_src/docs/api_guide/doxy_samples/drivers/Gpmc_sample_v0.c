//! [include]
#include <stdio.h>
#include <drivers/gpmc.h>
//! [include]


GPMC_Handle gGpmcHandle;

void open(void)
{
//! [gpmc_open]
    GPMC_Params gpmcParams;

    GPMC_Params_init(&gpmcParams);
    gGpmcHandle = GPMC_open(0, &gpmcParams);
    DebugP_assert(gGpmcHandle != NULL);
//! [gpmc_open]
}

void close(void)
{
//! [gpmc_close]
    GPMC_close(gGpmcHandle);
//! [gpmc_close]
}

