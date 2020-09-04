//! [include]
#include <stdio.h>
#include <drivers/mmcsd.h>
//! [include]

MMCSD_Handle gMmcsdHandle;

void open(void)
{
//! [open]
    MMCSD_Params mmcsdParams;

    MMCSD_Params_init(&mmcsdParams);
    gMmcsdHandle = MMCSD_open(0, &mmcsdParams);
    DebugP_assert(gMmcsdHandle != NULL);
//! [open]
}

void close(void)
{
//! [close]
    MMCSD_close(gMmcsdHandle);
//! [close]
}