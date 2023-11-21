
#include <stdio.h>
#include <kernel/dpl/DebugP.h>
//! [include]
#include <drivers/soc.h>
//! [include]

void get_corename(void)
{
//! [get_corename]
    const char *coreName;

    coreName = SOC_getCoreName(CSL_CORE_ID_R5FSS0_0);
    DebugP_log("Core name is: %s\r\n", coreName);
//! [get_corename]
}

void get_selfcpuclk(void)
{
//! [get_selfcpuclk]
    uint64_t cpuClockRate;

    cpuClockRate = SOC_getSelfCpuClk();
    DebugP_log("CPU Clock Frequency: %u\r\n", cpuClockRate);
//! [get_selfcpuclk]
}

void set_moduleclk(void)
{
//! [set_moduleclk]
    #include <drivers/sciclient.h> /* For the device and clock macros */

    uint32_t moduleId = TISCI_DEV_PRU_ICSSG0;
    uint32_t clkId = TISCI_DEV_PRU_ICSSG0_CORE_CLK;
    uint64_t clkRate = 300000000U;

    SOC_moduleSetClockFrequency(moduleId, clkId, clkRate);
//! [set_moduleclk]
}

void set_moduleclkwithparent(void)
{
//! [set_moduleclkwithparent]
    #include <drivers/sciclient.h> /* For the device and clock macros */

    uint32_t moduleId = TISCI_DEV_PRU_ICSSG0;
    uint32_t clkId = TISCI_DEV_PRU_ICSSG0_CORE_CLK;
    uint32_t clkParentId = TISCI_DEV_PRU_ICSSG0_CORE_CLK_PARENT_HSDIV4_16FFT_MAIN_2_HSDIVOUT0_CLK;
    uint64_t clkRate = 300000000U;

    SOC_moduleSetClockFrequencyWithParent(moduleId, clkId, clkParentId, clkRate);
//! [set_moduleclkwithparent]
}
