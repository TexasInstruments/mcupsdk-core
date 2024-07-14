/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

#include "pcie_v1_reg.h"

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
    volatile uint32_t *Pcie_SSModeAddr;
} Pcie_DevParams;
#define PCIE_WINDOW_START    0x10000000U
#define PCIE_PCIE_MSI_BASE   (0x00010000U)
#define PCIE_WINDOW_CFG_MASK 0x0000FFFFU
#define PCIE_WINDOW_MSI_ADDR (PCIE_WINDOW_START + 0x00020000U)
#define PCIE_WINDOW_MSI_MASK 0x0000FFFFU
#define PCIE_WINDOW_CFG_BASE (PCIE_WINDOW_START + 0x00010000U)
#define PCIE_WINDOW_MEM_BASE (PCIE_WINDOW_START + 0x01000000U)
#define PCIE_WINDOW_MEM_MASK 0x00FFFFFFU
#define PCIE_PCIE_MSI_OFF    (0x00000040U)

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

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef PCIE_V0_H_ */

