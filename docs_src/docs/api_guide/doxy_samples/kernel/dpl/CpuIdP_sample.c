
//! [include]
#include <kernel/dpl/CpuIdP.h>
#include <drivers/hw_include/cslr_soc.h>
//! [include]

void samples()
{
//! [cpuIdP]

    CSL_ArmR5CPUInfo cpuInfo;

    /* Get Core ID Info */
    CSL_armR5GetCpuID(&cpuInfo);
    if (cpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_0) /* R5SS0-0 */
    {
        if(cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_0)
        {
            /* R5FSS0 Core 0 */
        }
    }

//! [cpuIdP]
}