
#include <stdio.h>
#include <kernel/dpl/DebugP.h>
//! [include]
#include <drivers/esm.h>
//! [include]
#include <drivers/hw_include/csl_types.h>
#include <drivers/hw_include/cslr_soc.h>

#define CONFIG_ESM0           (0U)

ESM_Handle        gEsmHandle;

void open(void)
{
//! [open]
    ESM_OpenParams     esmParams;

    ESM_Params_init(&esmParams);      /* Initialize ESM parameters */
    gEsmHandle = ESM_open(CONFIG_ESM0, &esmParams);
    DebugP_assert(gEsmHandle != NULL);
//! [open]
}

void close(void)
{
//! [close]
    ESM_close(gEsmHandle);
//! [close]
}
