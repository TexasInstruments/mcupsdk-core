
//! [include]
#include <stdio.h>
#include <drivers/qspi.h>
//! [include]

QSPI_Handle gQspiHandle;

void open(void)
{
//! [open]
    QSPI_Params qspiParams;

    QSPI_Params_init(&qspiParams);
    gQspiHandle = QSPI_open(0, &qspiParams);
    DebugP_assert(gQspiHandle != NULL);
//! [open]
}

void close(void)
{
//! [close]
    QSPI_close(gQspiHandle);
//! [close]
}