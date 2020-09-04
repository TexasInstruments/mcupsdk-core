
//! [include]
#include <stdio.h>
#include <drivers/ospi.h>
//! [include]

OSPI_Handle gOspiHandle;

void open(void)
{
//! [open]
    OSPI_Params ospiParams;

    OSPI_Params_init(&ospiParams);
    gOspiHandle = OSPI_open(0, &ospiParams);
    DebugP_assert(gOspiHandle != NULL);
//! [open]
}

void close(void)
{
//! [close]
    OSPI_close(gOspiHandle);
//! [close]
}