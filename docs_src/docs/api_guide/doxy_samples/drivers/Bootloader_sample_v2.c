
//! [include]
#include <stdio.h>
#include <drivers/bootloader.h>
//! [include]


Bootloader_Handle gBootloaderHandle;
Bootloader_BootImageInfo gBootImageInfo;


void open(void)
{
//! [open]

    gBootloaderHandle = Bootloader_open(0, NULL);
    DebugP_assert(gBootloaderHandle != NULL);
//! [open]
}

void bootcores_am263x(void)
{
//! [bootcores_am263x]

    int32_t status = SystemP_SUCCESS;
    /* Loading of segments done inside this API */
    status = Bootloader_parseAndLoadMultiCoreELF(gBootloaderHandle, &gBootImageInfo);

    Bootloader_profileAddProfilePoint("CPU load");
    /* Run CPUs */
    if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(gBootloaderHandle, CSL_CORE_ID_R5FSS1_1)))
    {
        status = Bootloader_runCpu(gBootloaderHandle, &gBootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_1]);
    }
    if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(gBootloaderHandle, CSL_CORE_ID_R5FSS1_0)))
    {
        status = Bootloader_runCpu(gBootloaderHandle, &gBootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_0]);
    }
    if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(gBootloaderHandle, CSL_CORE_ID_R5FSS0_1)))
    {
        status = Bootloader_runCpu(gBootloaderHandle, &gBootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1]);
    }
    if(status == SystemP_SUCCESS)
    {
        /* If any of the R5 core 0 have valid image reset the R5 core. */
        status = Bootloader_runSelfCpu(gBootloaderHandle, &gBootImageInfo);
    } 

//! [bootcores_am263x]
}

void bootcores_am263px(void)
{
//! [bootcores_am263px]

    int32_t status = SystemP_SUCCESS;
    /* Loading of segments done inside this API */
    status = Bootloader_parseAndLoadMultiCoreELF(gBootloaderHandle, &gBootImageInfo);

    Bootloader_profileAddProfilePoint("CPU load");
    /* Run CPUs */
    if (status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(gBootloaderHandle, CSL_CORE_ID_R5FSS1_1)))
    {
        status = Bootloader_runCpu(gBootloaderHandle, &gBootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_1]);
    }
    if (status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(gBootloaderHandle, CSL_CORE_ID_R5FSS1_0)))
    {
        status = Bootloader_runCpu(gBootloaderHandle, &gBootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_0]);
    }
    if (status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(gBootloaderHandle, CSL_CORE_ID_R5FSS0_1)))
    {
        status = Bootloader_runCpu(gBootloaderHandle, &gBootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1]);
    }
    if(status == SystemP_SUCCESS)
    {
        /* If any of the R5 core 0 have valid image reset the R5 core. */
        status = Bootloader_runSelfCpu(gBootloaderHandle, &gBootImageInfo);
    }

//! [bootcores_am263px]
}

void close(void)
{
//! [close]
    Bootloader_close(gBootloaderHandle);
//! [close]
}