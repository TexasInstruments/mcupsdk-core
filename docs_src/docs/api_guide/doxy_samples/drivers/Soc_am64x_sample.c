
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
