/*
 *  Copyright (C) 2021-2024 Texas Instruments Incorporated
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

#ifndef BOOTLOADER_SOC_AM64X_H_
#define BOOTLOADER_SOC_AM64X_H_

#ifdef __cplusplus
extern "C"
{
#endif


#include <drivers/hw_include/cslr_soc.h>

#define BOOTLOADER_DEVICE_VARIANT_SINGLE_CORE    (0x00000000U)
#define BOOTLOADER_DEVICE_VARIANT_DUAL_CORE      (0x00040000U)
#define BOOTLOADER_DEVICE_VARIANT_QUAD_CORE      (0x000C0000U)

#define BOOTLOADER_R5FSS0                        (0x00000000U)
#define BOOTLOADER_R5FSS1                        (0x00010000U)

#define BOOTLOADER_ICSS_CORE_DEFAULT_FREQUENCY   (200000000U)

/**
 * \brief Data structure containing information about a core specific to the AM64x SOC
 *
 * This structure is used to store the data about cores in the SoC in the form of a lookup table which will be
 * used by various APIs.
 *
 */
typedef struct
{
	uint32_t tisciProcId;
	uint32_t tisciDevId;
	uint32_t tisciClockId;
	uint32_t defaultClockHz;
	char coreName[8];

} Bootloader_CoreBootInfo;

/**
 * \brief Request for a particular CPU in the AM64x SOC
 *
 * This API internally makes Sciclient calls to request control of the CPU
 *
 * \param cpuId [in] The CSL ID of the core
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t  Bootloader_socCpuRequest(uint32_t cpuId);

/**
 * \brief Release a particular CPU in the AM64x SOC
 *
 * This API internally makes Sciclient calls to release control of the CPU
 *
 * \param cpuId [in] The CSL ID of the core
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t  Bootloader_socCpuRelease(uint32_t cpuId);

/**
 * \brief Set the clock of a particular CPU in the AM64x SOC
 *
 * This API internally makes Sciclient calls to set CPU clock
 *
 * \param cpuId [in] The CSL ID of the core
 * \param cpuHz [in] Desired clock frequency of the CPU in Hertz
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t  Bootloader_socCpuSetClock(uint32_t cpuId, uint32_t cpuHz);

/**
 * \brief Get the clock of a particular CPU in the AM64x SOC
 *
 * This API internally makes Sciclient calls to get the current clock frequency of CPU
 *
 * \param cpuId [in] The CSL ID of the core
 *
 * \return Current clock speed of the CPU
 */
uint64_t Bootloader_socCpuGetClock(uint32_t cpuId);

/**
 * \brief Get the default clock of a particular CPU in the AM64x SOC
 *
 * This API queries and internal lookup table to fetch the default clock speed at which a
 * particular CPU should run.
 *
 * \param cpuId [in] The CSL ID of the core
 *
 * \return Default clock speed of the CPU
 */
uint32_t Bootloader_socCpuGetClkDefault(uint32_t cpuId);

/**
 * \brief Do power-on-reset of a particular CPU in the AM64x SOC
 *
 * This API is called only when booting a non-self CPU.
 *
 * \param cpuId [in] The CSL ID of the core
 *
 * \param socCoreOpMode Lockstep/Dual core mode as per setting in syscfg.
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t  Bootloader_socCpuPowerOnReset(uint32_t cpuId,void *socCoreOpMode);

/**
 * \brief Release a particular CPU in the AM64x SOC from reset
 *
 * This API is called only when booting a non-self CPU. There is a different
 * API \ref Bootloader_socCpuResetReleaseSelf in the case of a self CPU
 *
 * \param cpuId       [in] The CSL ID of the core
 * \param entryPoint [in] The entryPoint of the CPU, from where it should start execution
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t  Bootloader_socCpuResetRelease(uint32_t cpuId, uintptr_t entryPoint);

/**
 * \brief Release self CPU in the AM64x SOC from reset
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t  Bootloader_socCpuResetReleaseSelf(void);

/**
 * \brief Set entry point for self CPU in the AM64x SOC from reset
 *
 * This API need not be called when booting a non-self CPU. The entry point can be specified
 * in the \ref Bootloader_socCpuResetRelease function itself
 *
 * \param cpuId      [in] The CSL ID of the core
 * \param entryPoint [in] The entryPoint of the CPU, from where it should start execution
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t  Bootloader_socCpuSetEntryPoint(uint32_t cpuId, uintptr_t entryPoint);

/**
 * \brief Translate a CPU address to the SOC address wherever applicable
 *
 * This API need not be called when booting a non-self CPU. The entry point can be specified
 * in the \ref Bootloader_socCpuResetRelease function itself
 *
 * \param cslCoreId [in] The CSL ID of the core
 * \param addr      [in] The CPU addr
 *
 * \return SystemP_SUCCESS on success, else failure
 */
uint32_t Bootloader_socTranslateSectionAddr(uint32_t cslCoreId, uint32_t addr);

/**
 * \brief Obtain the CSL core ID of a CPU from its RPRC core ID
 *
 * \param rprcCoreId [in] The RPRC ID of the core
 *
 * \return CSL core ID of a CPU
 */
uint32_t Bootloader_socRprcToCslCoreId(uint32_t rprcCoreId);

/**
 * \brief Get the list of self cpus in the SOC.
 *
 * \return List of self cpus ending with an invalid core id
 */
uint32_t* Bootloader_socGetSelfCpuList(void);

/**
 * \brief Get the name of a core
 *
 * \param cpuId [in] The CSL ID of the core
 *
 * \return Name of the CPU
 */
char* Bootloader_socGetCoreName(uint32_t cpuId);

/**
 * \brief Initialize the core memories of a specific core
 *
 * \param cpuId [in] The CSL ID of the core
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Bootloader_socMemInitCpu(uint32_t cpuId);

/**
 * \brief Obtain the Sciclient Proc Id corresponding to the CSL core ID
 *
 * \param cpuId [in] The CSL ID of the core
 *
 * \return CSL core ID of a CPU
 */
uint32_t Bootloader_socGetSciclientCpuProcId(uint32_t cpuId);

/**
 * \brief Obtain the Sciclient Device Id corresponding to the CSL core ID
 *
 * \param cpuId [in] The CSL ID of the core
 *
 * \return CSL core ID of a CPU
 */
uint32_t Bootloader_socGetSciclientCpuDevId(uint32_t cpuId);

/**
 * \brief API to trigger the security handover from SYSFW
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Bootloader_socSecHandover(void);

/**
 * \brief API to wait for boot notification from SYSFW/ROM
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Bootloader_socWaitForFWBoot(void);

/**
 * \brief API to open required firewalls using SYSFW
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Bootloader_socOpenFirewalls(void);

/**
 * \brief API to authenticate (and decrypt if needed) an appimage using SYSFW
 *
 * \param certLoadAddr [in] The SOC address pointing to the certificate+appimage
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t Bootloader_socAuthImage(uint32_t certLoadAddr);

/**
 * \brief API to check if authentication is required for the device. Checks the
 *  SYS_STATUS register to see if device is GP, HS-FS, HS-SE etc
 *
 * \return TRUE (1U) if authentication required, FALSE (0U) if not.
 */
uint32_t Bootloader_socIsAuthRequired(void);

/**
 * \brief API to check if an R5 cluster/subsystem has dual cores enabled or not
 *
 * \return TRUE (1U) if dual cores enabled, FALSE (0U) if not.
 */
uint32_t Bootloader_socIsR5FSSDual(uint32_t ssNum);

/**
 * \brief API to check the GPN variant of the SOC - whether it's a quad core,
 * dual core or a single core variant
 *
 * \return \ref BOOTLOADER_DEVICE_VARIANT_SINGLE_CORE,
 *         \ref BOOTLOADER_DEVICE_VARIANT_DUAL_CORE or
 *         \ref BOOTLOADER_DEVICE_VARIANT_QUAD_CORE or
 *          0xFFFFFFFF if invalid bit field read
 */
uint32_t Bootloader_socGetCoreVariant(void);

/**
 * \brief Workaround API to prevent CPSW register lockup. Checks the reset source
 * and does a warm reset in case of POR MCU , POR MAIN or COLD BOOT reset.
 *
 */
void Bootloader_socResetWorkaround(void);

/**
 * \brief Enable MCU PLL.
 *        The MCU PLL will be initialized by DMSC if devgrp is set to DEVGRP_ALL.
 *        If devgrp is set to DEVGRP_00 (0x01) (Main Domain), the MCU PLL will not
 *        be initialized.
 *        This API initializes MCU PLL when devgrp is set to DEVGRP_00
 */
void Bootloader_enableMCUPLL(void);

/**
 * \brief Check if MCU domain is reset isolated
 *
 * \return TRUE (1U) if MCU domain is reset isolated, else return 0.
 */
uint32_t Bootloader_socIsMCUResetIsoEnabled(void);

/**
 * \brief Notify other cores firewall open from SBL. Function writes a Software defined
 * magic word to PSRAM address (software defined) to signal Firewall open to MCU cores.
 * This function should only be called from SBL
 */
void Bootloader_socNotifyFirewallOpen(void);

/**
 * \brief API to enable ICSS cores when applicable
 *
 * \param clkFreq [in] Clock frequency of ICSS cores in Hz
 *
 * \return System_SUCCESS if enable passes, else return SystemP_FAILURE
 */
int32_t Bootloader_socEnableICSSCores(uint32_t clkFreq);

/**
 * \brief API to get boot sequence oid
 *
 * \param boot_seq_oid [in] pointer to integer array for populating boot sequence oid
 *
 */
void Bootloader_socGetBootSeqOid(uint8_t* boot_seq_oid);

/**
 * dummy api call
 */
int32_t Bootloader_socCpuSetAppEntryPoint(uint32_t cpuId, uintptr_t entryPoint);

#ifdef __cplusplus
}
#endif

#endif /* BOOTLOADER_SOC_AM64X_H_ */