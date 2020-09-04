#include <stddef.h>
//! [pruicss_include]
#include <drivers/pruicss.h>
//! [pruicss_include]

#define CONFIG_PRU_ICSS1 (0U)
PRUICSS_Handle          gPruicssHandle;
uint32_t                pruEvtoutNum;
int32_t                 intrNum;
int32_t                 eventNum;
uint8_t                 waitEnable;
PRUICSS_IrqHandler      irqHandler;
PRUICSS_IntcInitData    pruicssIntcInitData;
uint32_t                pruFirmware;
uint32_t                pruFirmwareLength;

void pruicss_open(void)
{
//! [pruicss_open]
    /*Use CONFIG_PRU_ICSS1 macro as parameter to PRUICSS_open */
    gPruicssHandle = PRUICSS_open(CONFIG_PRU_ICSS1);

    DebugP_assert(gPruicssHandle != NULL);
//! [pruicss_open]
}

void pruicss_run_firmware(void)
{
//! [pruicss_run_firmware]
    /* Reset and disable PRU core */
    PRUICSS_resetCore(gPruicssHandle, PRUICSS_PRU0);
    PRUICSS_disableCore(gPruicssHandle, PRUICSS_PRU0);

    /* Register an Interrupt Handler for an event */
    PRUICSS_registerIrqHandler(gPruicssHandle,
                               pruEvtoutNum,
                               intrNum,
                               eventNum,
                               waitEnable,
                               irqHandler);


    /* API to do Interrupt-Channel-host mapping */
    PRUICSS_intcInit(gPruicssHandle, &pruicssIntcInitData);

    /* Load the IRAM in PRU */
    PRUICSS_writeMemory(gPruicssHandle,
                        PRUICSS_IRAM_PRU(0),
                        0,
                        (uint32_t *) pruFirmware,
                        pruFirmwareLength);
    /* Enable PRU */
    PRUICSS_enableCore(gPruicssHandle, PRUICSS_PRU0);
//! [pruicss_run_firmware]
}

