/*
 *  Copyright (C) 2018-2022 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CPUIDP_H
#define CPUIDP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <drivers/hw_include/cslr_soc.h>

/**
 * \defgroup KERNEL_DPL_CPU APIs for CPU ID
 * \ingroup KERNEL_DPL
 *
 * @{
 */

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
/**
 * \brief  Structure containing the CPU Info such as CPU ID and Cluster Group ID.
 */
typedef struct
{
    uint32_t cpuID;
    /**< CPU/core ID within cluster
     *   Refer \ref CSL_ArmR5CPUID
     */
    uint32_t grpId;
    /**< Group Id of the cluster
     *   Refer \ref CSL_ArmR5ClusterGroupID
     */
}CSL_ArmR5CPUInfo;

/**
 *  \brief Get the cluster group and CPU ID for current R5 Core
 *
 *  \param cpuInfo          Pointer to CPU info structure
 *                          Refer struct \ref CSL_ArmR5CPUInfo
 *
 * Please NOTE that this function has to be called in privileged mode only
 */
void CSL_armR5GetCpuID(CSL_ArmR5CPUInfo *cpuInfo);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* CPUIDP_H */
