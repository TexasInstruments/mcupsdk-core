#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include "pru_io/driver/pru_ipc.h"
#include <drivers/pruicss.h>

PRU_IPC_Handle gPruIpc0Handle;
PRUICSS_Handle gPruIcss0Handle;

# define CONFIG_PRU_IPC0     0

int32_t samples[1][32];

void PRU_IPC_Isr(void *args)
{
    // sample isr
}

void init_pruIpc(void)
{
//! [init_pruIpc]
    /* ----------------------------------------------------------------- */
    /* Initialize IPC between r5f and PRU cores                          */
    /* ----------------------------------------------------------------- */
    PRU_IPC_Params pruIpcparams = {
                                .pruicssHandle = gPruIcss0Handle,
                                .transferCallbackFxn = &PRU_IPC_Isr,
    };

    gPruIpc0Handle = PRU_IPC_open(CONFIG_PRU_IPC0, &pruIpcparams);
    DebugP_assert(gPruIpc0Handle != NULL);
//! [init_pruIpc]
}

void transferData(void)
{
//! [transferData]
    /* Receive Data from PRU core */
    PRU_IPC_getData(gPruIpc0Handle, samples);
    /* Send Data to PRU core */
    PRU_IPC_sendData(gPruIpc0Handle, samples);
//! [transferData]
}
