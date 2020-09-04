/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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


#ifndef PCIE_V0_H_
#define PCIE_V0_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "pcie_v0_reg.h"

/* ========================================================================== */
/*                             Macros                                         */
/* ========================================================================== */
/* The LSB of MSI data received is used to switch between MSI irq handler */
/* This is the mask for getting the interrupt number from the MSI data */
#define PCIE_MSI_IRQNUM_MASK         (PCIE_MAX_MSI_IRQ-1)

/* The LSB of MSIx data received is used to switch between MSIx irq handler */
/* This is the mask for getting the interrupt number from the MSIx data */
#define PCIE_MSIX_IRQNUM_MASK        (PCIE_MAX_MSIX_IRQ-1)

typedef struct
{
    volatile uint32_t        *pcieCtrlAddr;   /** address of PCIECTRL register */
    volatile uint32_t        *pcieRefClkAddr; /** address of PCIEREFCLK register */
    void                     *userCfgBase;    /** base address of user config */
    void                     *intCfgBase;     /** base address of int config */
    uint8_t                   numLane;        /** Lane Number **/
    uint8_t                   linkSpeed;      /** Link Speed **/
    uint8_t                   pfNum;          /** Physical Function Number **/
} Pcie_DevParams;

typedef struct
{
    void                    *cfgBase;          /** base address of RC/EP config */
    uint32_t                remoteOffset;      /** offset relative to data area for remote config */
} Pcie_DeviceCfgBaseAddrs;

typedef struct
{
    uint32_t intNum;
    void (* funcPtr)(void *);
} Pcie_legacyIrqRegisterParam;

typedef struct
{
    uint32_t intNum;
    uint32_t assert;
} Pcie_legacyIrqSetParams;

typedef struct
{
    uint32_t enable;
    uint32_t loAddr;
    uint32_t upAddr;
    uint32_t data;
}Pcie_MsiParams;

typedef struct
{
    uint64_t addr;
    uint32_t intNum;
    uint32_t data;
} Pcie_sendMsiParams;

typedef struct
{
    uint32_t enable;
} Pcie_MsixParams;

typedef struct
{
    uint32_t tbl_offset;
    uint32_t barIndex;
} Pcie_MsixTblParams;

typedef struct
{
    uintptr_t addr;
    uint32_t intNum;
    uint32_t data;
} Pcie_sendMsixParams;

typedef struct
{
    uint8_t enable;
    uint32_t tblSize;
} Pcie_MsixCtrlReg;

typedef enum
{
    PCIE_EPBARA_128B = 0x0U,
    PCIE_EPBARA_256B,
    PCIE_EPBARA_512B,
    PCIE_EPBARA_1K,
    PCIE_EPBARA_2K,
    PCIE_EPBARA_4K,
    PCIE_EPBARA_8K,
    PCIE_EPBARA_16K,
    PCIE_EPBARA_32K,
    PCIE_EPBARA_64K,
    PCIE_EPBARA_128K,
    PCIE_EPBARA_256K,
    PCIE_EPBARA_512K,
    PCIE_EPBARA_1M,
    PCIE_EPBARA_2M,
    PCIE_EPBARA_4M,
    PCIE_EPBARA_8M,
    PCIE_EPBARA_16M,
    PCIE_EPBARA_32M,
    PCIE_EPBARA_64M,
    PCIE_EPBARA_128M,
    PCIE_EPBARA_256M,
    PCIE_EPBARA_512M,
    PCIE_EPBARA_1G,
    PCIE_EPBARA_2G
} Pcie_EPBarAperture;

typedef enum
{
    PCIE_RCBARA_4B = 0x0U,
    PCIE_RCBARA_8B,
    PCIE_RCBARA_16B,
    PCIE_RCBARA_32B,
    PCIE_RCBARA_64B,
    PCIE_RCBARA_128B,
    PCIE_RCBARA_256B,
    PCIE_RCBARA_512B,
    PCIE_RCBARA_1K,
    PCIE_RCBARA_2K,
    PCIE_RCBARA_4K,
    PCIE_RCBARA_8K,
    PCIE_RCBARA_16K,
    PCIE_RCBARA_32K,
    PCIE_RCBARA_64K,
    PCIE_RCBARA_128K,
    PCIE_RCBARA_256K,
    PCIE_RCBARA_512K,
    PCIE_RCBARA_1M,
    PCIE_RCBARA_2M,
    PCIE_RCBARA_4M,
    PCIE_RCBARA_8M,
    PCIE_RCBARA_16M,
    PCIE_RCBARA_32M,
    PCIE_RCBARA_64M,
    PCIE_RCBARA_128M,
    PCIE_RCBARA_256M,
    PCIE_RCBARA_512M,
    PCIE_RCBARA_1G,
    PCIE_RCBARA_2G,
    PCIE_RCBARA_4G,
    PCIE_RCBARA_8G,
    PCIE_RCBARA_16G,
    PCIE_RCBARA_32G,
    PCIE_RCBARA_64G,
    PCIE_RCBARA_128G,
    PCIE_RCBARA_256G,
} Pcie_RCBarAperture;

typedef enum
{
    PCIE_BARC_DISABLED = 0x0,
    PCIE_BARC_32B_IO_BAR = 0x1,
    PCIE_BARC_32B_MEM_BAR_NON_PREFETCH = 0x4,
    PCIE_BARC_32B_MEM_BAR_PREFETCH = 0x5,
    PCIE_BARC_64B_MEM_BAR_NON_PREFETCH = 0x6,
    PCIE_BARC_64B_MEM_BAR_PREFETCH = 0x7
} Pcie_BarConfig;

/**
 * \brief Function to read PCIe registers
 *
 * \param handle #Pcie_Handle returned from #Pcie_open()
 * \param location Local or remote configuration space
 * \param readRegs Register that needs to be read
 *
 * \return #SystemP_SUCCESS on successful; else error on failure
 */
int32_t Pcie_readRegs (Pcie_Handle handle, Pcie_Location location, Pcie_Registers *readRegs);

/**
 * \brief Function to read PCIe registers
 *
 * \param handle #Pcie_Handle returned from #Pcie_open()
 * \param location Local or remote configuration space
 * \param writeRegs Details of register that needs to be written to
 *
 * \return #SystemP_SUCCESS on successful; else error on failure
 */
int32_t Pcie_writeRegs (Pcie_Handle handle, Pcie_Location location, Pcie_Registers *writeRegs);

/**
 * \brief Initialize serializer deserializer for PCIe
 *
 * \param handle #Pcie_Handle returned from #Pcie_open()
 * \param deviceNum Pcie device number
 *
 * \return #SystemP_SUCCESS on successful; else error on failure
 */
int32_t Pcie_serdesInit (Pcie_Handle handle, uint32_t deviceNum);

/**
 * \brief Enable or disable SRIS control
 *
 * \param handle #Pcie_Handle returned from #Pcie_open()
 * \param enable Enable (1) / Disable (0) SRIS control
 */
void Pcie_srisControl (Pcie_Handle handle, uint32_t enable);

/**
 * \brief Enable legacy interrupts for the root complex mode of operation
 *
 * \param handle #Pcie_Handle returned from #Pcie_open()
 * \param enable Enable (1) / Disable (0) Legacy interrupts
 *
 * \return #SystemP_SUCCESS on successful; else error on failure
 */
int32_t Pcie_rcLegacyIrqEnable (Pcie_Handle handle, uint32_t enable);

/**
 * \brief Register ISR for legacy IRQ in Root complex (RC)
 *
 * \param handle #Pcie_Handle returned from #Pcie_open()
 * \param irqParams Params for registering ISR
 *
 * \return #SystemP_SUCCESS on successful; else error on failure
 */
int32_t Pcie_rclegacyIrqIsrRegister (Pcie_Handle handle, Pcie_legacyIrqRegisterParam irqParams);

/**
 * \brief Send legacy IRQ to Root Complex (RC) from End Point (EP)
 *
 * \param handle #Pcie_Handle returned from #Pcie_open()
 * \param irqSetParams Params for sending IRQ
 *
 * \return #SystemP_SUCCESS on successful; else error on failure
 */
int32_t Pcie_epLegacyIrqSet(Pcie_Handle handle, Pcie_legacyIrqSetParams irqSetParams);

/**
 * \brief Mark End of Interrupt for Legacy IRQ
 *
 * \param handle #Pcie_Handle returned from #Pcie_open()
 *
 *  \return #SystemP_SUCCESS on successful; else error on failure
 */
int32_t Pcie_rcLegacyIrqEoi (Pcie_Handle handle);

/**
 * \brief Enable MSI interrupts
 *
 * \param handle #Pcie_Handle returned by #Pcie_open()
 * \param drvIndex Index of Pcie driver instance
 * \param msiParams Parameters to enable MSI
 *
 * \return #SystemP_SUCCESS on successful; else error on failure
 */
int32_t Pcie_rcEnableMSI (Pcie_Handle handle, uint32_t drvIndex,
                            Pcie_MsiParams msiParams);

/**
 * \brief Enable MSIX interrupts
 *
 * \param handle #Pcie_Handle returned by #Pcie_open()
 * \param drvIndex Index of Pcie driver instance
 * \param msiParams Parameters to enable MSIX
 *
 * \return #SystemP_SUCCESS on successful; else error on failure
 */
int32_t Pcie_rcEnableMSIX (Pcie_Handle handle, uint32_t drvIndex,
                            Pcie_MsixParams msixParams);

/**
 * \brief Register MSI ISR for RC (Root Complex) device.
 *
 * \param handle #Pcie_Handle returned by #Pcie_open()
 * \param params Parameters for setting ISR for MSI
 *
 * \return #SystemP_SUCCESS on successful; else error on failure
 */
int32_t Pcie_rcRegisterMsiIsr (Pcie_Handle handle, Pcie_RegisterMsiIsrParams params);

/**
 * \brief Register MSIX ISR for RC (Root Complex) device.
 *
 * \param handle #Pcie_Handle returned by #Pcie_open()
 * \param params Parameters for setting ISR for MSIX
 *
 * \return #SystemP_SUCCESS on successful; else error on failure
 */
int32_t Pcie_rcRegisterMsixIsr (Pcie_Handle handle, Pcie_RegisterMsixIsrParams params);

/**
 * \brief Write MISX table offset
 *
 * \param handle #Pcie_Handle returned by #Pcie_open()
 * \param location Local or remote configuration space
 * \param params Parameters for writing MSIX table offset and bar index
 *
 * \return #SystemP_SUCCESS on successful; else error on failure
 */
int32_t Pcie_setMsixTblOffset (Pcie_Handle handle, Pcie_Location location, Pcie_MsixTblParams params);

/**
 * \brief Read MISX table offset
 *
 * \param handle #Pcie_Handle returned by #Pcie_open()
 * \param location Local or remote configuration space
 * \param params Parameters to read MSIX table offset and bar index
 *
 * \return #SystemP_SUCCESS on successful; else error on failure
 */
int32_t Pcie_getMsixTblOffset (Pcie_Handle handle, Pcie_Location location, Pcie_MsixTblParams *params);

/**
 * \brief Read MSI registers (Enable/Disable, Data, Lo address, Up address)
 *
 * \param handle #Pcie_Handle returned by #Pcie_open()
 * \param location Local or remote configuration space
 * \param params #Pcie_MsiParams to read the MSI parameters
 *
 * \return #SystemP_SUCCESS on successful; else error on failure
 */
int32_t Pcie_getMsiRegs (Pcie_Handle handle, Pcie_Location location, Pcie_MsiParams *params);

/**
 * \brief Write MSI registers (Enable/Disable, Lo address, Up address)
 *
 * \param handle #Pcie_Handle returned by #Pcie_open()
 * \param location Local or remote configuration space
 * \param params #Pcie_MsiParams to write the MSI parameters
 *
 * \return #SystemP_SUCCESS on successful; else error on failure
 */
int32_t Pcie_setMsiRegs (Pcie_Handle handle, Pcie_Location location, Pcie_MsiParams params);

/**
 * \brief Read MSIx ctrl register
 *
 * \param handle #Pcie_Handle returned by #Pcie_open()
 * \param location Local or remote configuration space
 * \param params #Pcie_MsixCtrl to read MSIx ctrl register
 *
 * @return #SystemP_SUCCESS on successful; else error on failure
 */
int32_t Pcie_getMsixCtrlRegister (Pcie_Handle handle, Pcie_Location location,
                                         Pcie_MsixCtrlReg *params);

/**
 * \brief Write MSIx ctrl register
 *
 * \param handle #Pcie_Handle returned by #Pcie_open()
 * \param location Local or remote configuration space
 * \param params #Pcie_MsixCtrl to write MSIx ctrl register
 *
 * \return #SystemP_SUCCESS on successful; else error on failure
 */
int32_t Pcie_setMsixCtrlRegister (Pcie_Handle handle, Pcie_Location location,
                                         Pcie_MsixCtrlReg params);

/**
 * \brief Add MSIx table entry
 *
 * \param handle #Pcie_Handle returned by #Pcie_open()
 * \param params #Pcie_sendMsixParams to add MSIx table entry
 *
 * \return #SystemP_SUCCESS on successful; else error on failure
 */
int32_t Pcie_epSetMsixTblIntr (Pcie_Handle handle, Pcie_sendMsixParams params);

/**
 * \brief Send MSIx interrupt to RC
 *
 * \param handle #Pcie_Handle returned by #Pcie_open()
 * \param intNum Interrupt number
 * \param addr Address to write MSIx to
 *
 * \return #SystemP_SUCCESS on successful; else error on failure
 */
int32_t Pcie_epSendMsixIrq(Pcie_Handle handle, uint32_t intNum, uint64_t addr);

/**
 * \brief Send MSI Irq from EP to RC
 *
 * \param handle #Pcie_Handle returned by #Pcie_open()
 * \param intNum MSI interrupt number
 * \param params #Pcie_MsiParams for sending MSI
 *
 * \return #SystemP_SUCCESS on successful; else error on failure
 */
int32_t Pcie_epSendMsiIrq(Pcie_Handle handle, uint32_t intNum, Pcie_sendMsiParams params);

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef PCIE_V0_H_ */

