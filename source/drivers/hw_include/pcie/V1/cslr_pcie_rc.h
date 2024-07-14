/*
 * Copyright (C) 2024 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef CSLR_PCIE_RC_H_
#define CSLR_PCIE_RC_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

#ifndef CSL_MODIFICATION
typedef struct {
    volatile uint32_t MMR_IRQ_STATUS_RAW;
    volatile uint32_t MMR_IRQ_STATUS;
    volatile uint32_t MMR_IRQ_ENABLE_SET;
    volatile uint32_t MMR_IRQ_ENABLE_CLR;
} CSL_pcie_rc_mmrIrqRegs;

typedef struct {
    volatile uint32_t LEGACY_IRQ_STATUS_RAW;
    volatile uint32_t LEGACY_IRQ_STATUS;
    volatile uint32_t LEGACY_IRQ_ENABLE_SET;
    volatile uint32_t LEGACY_IRQ_ENABLE_CLR;
} CSL_pcie_rc_legacyIrqRegs;

typedef struct {
    volatile uint32_t IATU_REGION_CTRL_1_OFF_OUTBOUND;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_OUTBOUND;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_OUTBOUND;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_OUTBOUND;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_OUTBOUND;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_OUTBOUND;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND;
    volatile uint8_t  Resv[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_INBOUND;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_INBOUND;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_INBOUND;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_INBOUND;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_INBOUND;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_INBOUND;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_INBOUND;
    volatile uint8_t  Resv_25088[228];
} CSL_pcie_rc_iatuRegs;

typedef struct {
    volatile uint32_t VC_P_RX_Q_CTRL_OFF;
    volatile uint32_t VC_NP_RX_Q_CTRL_OFF;
    volatile uint32_t VC_CPL_RX_Q_CTRL_OFF;
} CSL_pcie_rc_vcRxQCtrlRegs;

typedef struct {
    volatile uint32_t MSI_CTRL_INT_EN_OFF;
    volatile uint32_t MSI_CTRL_INT_MASK_OFF;
    volatile uint32_t MSI_CTRL_INT_STATUS_OFF;
} CSL_pcie_rc_msiCtrlRegs;

#endif /* CSL_MODIFICATION */

/**************************************************************************
* Hardware Region  : RC mode PCIE core and ELBI registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PID;
    volatile uint32_t CMD_STATUS;
    volatile uint8_t  Resv_20[12];
    volatile uint32_t RSTCMD;
    volatile uint32_t PTMCFG;
    volatile uint8_t  Resv_32[4];
    volatile uint32_t PMCMD;
    volatile uint8_t  Resv_80[44];
    volatile uint32_t IRQ_EOI;
    volatile uint32_t MMR_IRQ;
    volatile uint8_t  Resv_100[12];
    volatile uint32_t LEGACY_IRQ_SET;
    volatile uint32_t LEGACY_IRQ_CLR;
    volatile uint32_t LEGACY_IRQ_STATUS;
#ifdef CSL_MODIFICATION
    volatile uint32_t GPR0;
    volatile uint32_t GPR1;
    volatile uint32_t GPR2;
    volatile uint32_t GPR3;
#else /* CSL_MODIFICATION */
    volatile uint32_t GPR[4];
#endif /* CSL_MODIFICATION */
    volatile uint8_t  Resv_256[128];
#ifdef CSL_MODIFICATION
    volatile uint32_t MMR0_IRQ_STATUS_RAW;
    volatile uint32_t MMR0_IRQ_STATUS;
    volatile uint32_t MMR0_IRQ_ENABLE_SET;
    volatile uint32_t MMR0_IRQ_ENABLE_CLR;
    volatile uint32_t MMR1_IRQ_STATUS_RAW;
    volatile uint32_t MMR1_IRQ_STATUS;
    volatile uint32_t MMR1_IRQ_ENABLE_SET;
    volatile uint32_t MMR1_IRQ_ENABLE_CLR;
    volatile uint32_t MMR2_IRQ_STATUS_RAW;
    volatile uint32_t MMR2_IRQ_STATUS;
    volatile uint32_t MMR2_IRQ_ENABLE_SET;
    volatile uint32_t MMR2_IRQ_ENABLE_CLR;
    volatile uint32_t MMR3_IRQ_STATUS_RAW;
    volatile uint32_t MMR3_IRQ_STATUS;
    volatile uint32_t MMR3_IRQ_ENABLE_SET;
    volatile uint32_t MMR3_IRQ_ENABLE_CLR;
    volatile uint32_t MMR4_IRQ_STATUS_RAW;
    volatile uint32_t MMR4_IRQ_STATUS;
    volatile uint32_t MMR4_IRQ_ENABLE_SET;
    volatile uint32_t MMR4_IRQ_ENABLE_CLR;
    volatile uint32_t MMR5_IRQ_STATUS_RAW;
    volatile uint32_t MMR5_IRQ_STATUS;
    volatile uint32_t MMR5_IRQ_ENABLE_SET;
    volatile uint32_t MMR5_IRQ_ENABLE_CLR;
    volatile uint32_t MMR6_IRQ_STATUS_RAW;
    volatile uint32_t MMR6_IRQ_STATUS;
    volatile uint32_t MMR6_IRQ_ENABLE_SET;
    volatile uint32_t MMR6_IRQ_ENABLE_CLR;
    volatile uint32_t MMR7_IRQ_STATUS_RAW;
    volatile uint32_t MMR7_IRQ_STATUS;
    volatile uint32_t MMR7_IRQ_ENABLE_SET;
    volatile uint32_t MMR7_IRQ_ENABLE_CLR;
#else /* CSL_MODIFICATION */
    CSL_pcie_rc_mmrIrqRegs MMR_IRQ_FLAGS[8];
#endif /* CSL_MODIFICATION */
#ifdef CSL_MODIFICATION
    volatile uint32_t LEGACY_A_IRQ_STATUS_RAW;
    volatile uint32_t LEGACY_A_IRQ_STATUS;
    volatile uint32_t LEGACY_A_IRQ_ENABLE_SET;
    volatile uint32_t LEGACY_A_IRQ_ENABLE_CLR;
    volatile uint32_t LEGACY_B_IRQ_STATUS_RAW;
    volatile uint32_t LEGACY_B_IRQ_STATUS;
    volatile uint32_t LEGACY_B_IRQ_ENABLE_SET;
    volatile uint32_t LEGACY_B_IRQ_ENABLE_CLR;
    volatile uint32_t LEGACY_C_IRQ_STATUS_RAW;
    volatile uint32_t LEGACY_C_IRQ_STATUS;
    volatile uint32_t LEGACY_C_IRQ_ENABLE_SET;
    volatile uint32_t LEGACY_C_IRQ_ENABLE_CLR;
    volatile uint32_t LEGACY_D_IRQ_STATUS_RAW;
    volatile uint32_t LEGACY_D_IRQ_STATUS;
    volatile uint32_t LEGACY_D_IRQ_ENABLE_SET;
    volatile uint32_t LEGACY_D_IRQ_ENABLE_CLR;
#else /* CSL_MODIFICATION */
    CSL_pcie_rc_legacyIrqRegs LEGACY_IRQ_FLAGS[4];
#endif /* CSL_MODIFICATION */
    volatile uint32_t ERR_IRQ_STATUS_RAW;
    volatile uint32_t ERR_IRQ_STATUS;
    volatile uint32_t ERR_IRQ_ENABLE_SET;
    volatile uint32_t ERR_IRQ_ENABLE_CLR;
    volatile uint32_t PMRST_IRQ_STATUS_RAW;
    volatile uint32_t PMRST_IRQ_STATUS;
    volatile uint32_t PMRST_IRQ_ENABLE_SET;
    volatile uint32_t PMRST_IRQ_ENABLE_CLR;
    volatile uint32_t PTM_IRQ_STATUS_RAW;
    volatile uint32_t PTM_IRQ_STATUS;
    volatile uint32_t PTM_IRQ_ENABLE_SET;
    volatile uint32_t PTM_IRQ_ENABLE_CLR;
    volatile uint8_t  Resv_4096[3600];
    volatile uint32_t TYPE1_DEV_ID_VEND_ID_REG;
    volatile uint32_t TYPE1_STATUS_COMMAND_REG;
    volatile uint32_t TYPE1_CLASS_CODE_REV_ID_REG;
    volatile uint32_t TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG;
    volatile uint8_t  Resv_4120[8];
    volatile uint32_t SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG;
    volatile uint32_t SEC_STAT_IO_LIMIT_IO_BASE_REG;
    volatile uint32_t MEM_LIMIT_MEM_BASE_REG;
    volatile uint32_t PREF_MEM_LIMIT_PREF_MEM_BASE_REG;
    volatile uint32_t PREF_BASE_UPPER_REG;
    volatile uint32_t PREF_LIMIT_UPPER_REG;
    volatile uint32_t IO_LIMIT_UPPER_IO_BASE_UPPER_REG;
    volatile uint32_t TYPE1_CAP_PTR_REG;
    volatile uint32_t TYPE1_EXP_ROM_BASE_REG;
    volatile uint32_t BRIDGE_CTRL_INT_PIN_INT_LINE_REG;
    volatile uint32_t CAP_ID_NXT_PTR_REG;
    volatile uint32_t CON_STATUS_REG;
    volatile uint8_t  Resv_4176[8];
    volatile uint32_t PCI_MSI_CAP_ID_NEXT_CTRL_REG;
    volatile uint32_t MSI_CAP_OFF_04H_REG;
    volatile uint32_t MSI_CAP_OFF_08H_REG;
    volatile uint32_t MSI_CAP_OFF_0CH_REG;
    volatile uint32_t MSI_CAP_OFF_10H_REG;
    volatile uint32_t MSI_CAP_OFF_14H_REG;
    volatile uint8_t  Resv_4208[8];
    volatile uint32_t PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG;
    volatile uint32_t DEVICE_CAPABILITIES_REG;
    volatile uint32_t DEVICE_CONTROL_DEVICE_STATUS;
    volatile uint32_t LINK_CAPABILITIES_REG;
    volatile uint32_t LINK_CONTROL_LINK_STATUS_REG;
    volatile uint32_t SLOT_CAPABILITIES_REG;
    volatile uint32_t SLOT_CONTROL_SLOT_STATUS;
    volatile uint32_t ROOT_CONTROL_ROOT_CAPABILITIES_REG;
    volatile uint32_t ROOT_STATUS_REG;
    volatile uint32_t DEVICE_CAPABILITIES2_REG;
    volatile uint32_t DEVICE_CONTROL2_DEVICE_STATUS2_REG;
    volatile uint32_t LINK_CAPABILITIES2_REG;
    volatile uint32_t LINK_CONTROL2_LINK_STATUS2_REG;
    volatile uint8_t  Resv_4272[12];
    volatile uint32_t PCI_MSIX_CAP_ID_NEXT_CTRL_REG;
    volatile uint32_t MSIX_TABLE_OFFSET_REG;
    volatile uint32_t MSIX_PBA_OFFSET_REG;
    volatile uint8_t  Resv_4352[68];
    volatile uint32_t AER_EXT_CAP_HDR_OFF;
    volatile uint32_t UNCORR_ERR_STATUS_OFF;
    volatile uint32_t UNCORR_ERR_MASK_OFF;
    volatile uint32_t UNCORR_ERR_SEV_OFF;
    volatile uint32_t CORR_ERR_STATUS_OFF;
    volatile uint32_t CORR_ERR_MASK_OFF;
    volatile uint32_t ADV_ERR_CAP_CTRL_OFF;
#ifdef CSL_MODIFICATION
    volatile uint32_t HDR_LOG_0_OFF;
    volatile uint32_t HDR_LOG_1_OFF;
    volatile uint32_t HDR_LOG_2_OFF;
    volatile uint32_t HDR_LOG_3_OFF;
#else /* CSL_MODIFICATION */
    volatile uint32_t HDR_LOG_OFF[4];
#endif /* CSL_MODIFICATION */
    volatile uint32_t ROOT_ERR_CMD_OFF;
    volatile uint32_t ROOT_ERR_STATUS_OFF;
    volatile uint32_t ERR_SRC_ID_OFF;
    volatile uint32_t TLP_PREFIX_LOG_1_OFF;
    volatile uint32_t TLP_PREFIX_LOG_2_OFF;
    volatile uint32_t TLP_PREFIX_LOG_3_OFF;
    volatile uint32_t TLP_PREFIX_LOG_4_OFF;
    volatile uint32_t VC_BASE;
    volatile uint32_t VC_CAPABILITIES_REG_1;
    volatile uint32_t VC_CAPABILITIES_REG_2;
    volatile uint32_t VC_STATUS_CONTROL_REG;
    volatile uint32_t RESOURCE_CAP_REG_VC0;
    volatile uint32_t RESOURCE_CON_REG_VC0;
    volatile uint32_t RESOURCE_STATUS_REG_VC0;
    volatile uint32_t RESOURCE_CAP_REG_VC1;
    volatile uint32_t RESOURCE_CON_REG_VC1;
    volatile uint32_t RESOURCE_STATUS_REG_VC1;
    volatile uint32_t RESOURCE_CAP_REG_VC2;
    volatile uint32_t RESOURCE_CON_REG_VC2;
    volatile uint32_t RESOURCE_STATUS_REG_VC2;
    volatile uint32_t RESOURCE_CAP_REG_VC3;
    volatile uint32_t RESOURCE_CON_REG_VC3;
    volatile uint32_t RESOURCE_STATUS_REG_VC3;
    volatile uint8_t  Resv_4504[16];
    volatile uint32_t SPCIE_CAP_HEADER_REG;
    volatile uint32_t LINK_CONTROL3_REG;
    volatile uint32_t LANE_ERR_STATUS_REG;
    volatile uint32_t SPCIE_CAP_OFF_0CH_REG;
    volatile uint32_t L1SUB_CAP_HEADER_REG;
    volatile uint32_t L1SUB_CAPABILITY_REG;
    volatile uint32_t L1SUB_CONTROL1_REG;
    volatile uint32_t L1SUB_CONTROL2_REG;
    volatile uint32_t PTM_EXT_CAP_HDR_OFF;
    volatile uint32_t PTM_CAP_OFF;
    volatile uint32_t PTM_CONTROL_OFF;
    volatile uint32_t PTM_RES_CAP_HDR_OFF;
    volatile uint32_t PTM_RES_HDR_OFF;
    volatile uint32_t PTM_RES_CONTROL_OFF;
    volatile uint32_t PTM_RES_STATUS_OFF;
    volatile uint32_t PTM_RES_LOCAL_LSB_OFF;
    volatile uint32_t PTM_RES_LOCAL_MSB_OFF;
    volatile uint32_t PTM_RES_T2_LSB_OFF;
    volatile uint32_t PTM_RES_T2_MSB_OFF;
    volatile uint32_t PTM_RES_T2P_LSB_OFF;
    volatile uint32_t PTM_RES_T2P_MSB_OFF;
    volatile uint32_t PTM_RES_T3_LSB_OFF;
    volatile uint32_t PTM_RES_T3_MSB_OFF;
    volatile uint32_t PTM_RES_T3P_LSB_OFF;
    volatile uint32_t PTM_RES_T3P_MSB_OFF;
    volatile uint32_t PTM_RES_TX_LATENCY_OFF;
    volatile uint32_t PTM_RES_RX_LATENCY_OFF;
    volatile uint8_t  Resv_5888[1276];
    volatile uint32_t ACK_LATENCY_TIMER_OFF;
    volatile uint32_t VENDOR_SPEC_DLLP_OFF;
    volatile uint32_t PORT_FORCE_OFF;
    volatile uint32_t ACK_F_ASPM_CTRL_OFF;
    volatile uint32_t PORT_LINK_CTRL_OFF;
    volatile uint32_t LANE_SKEW_OFF;
    volatile uint32_t TIMER_CTRL_MAX_FUNC_NUM_OFF;
    volatile uint32_t SYMBOL_TIMER_FILTER_1_OFF;
    volatile uint32_t FILTER_MASK_2_OFF;
    volatile uint32_t AMBA_MUL_OB_DECOMP_NP_SUB_REQ_CTRL_OFF;
    volatile uint32_t PL_DEBUG0_OFF;
    volatile uint32_t PL_DEBUG1_OFF;
    volatile uint32_t TX_P_FC_CREDIT_STATUS_OFF;
    volatile uint32_t TX_NP_FC_CREDIT_STATUS_OFF;
    volatile uint32_t TX_CPL_FC_CREDIT_STATUS_OFF;
    volatile uint32_t QUEUE_STATUS_OFF;
    volatile uint32_t VC_TX_ARBI_1_OFF;
    volatile uint32_t VC_TX_ARBI_2_OFF;
#ifdef CSL_MODIFICATION
    volatile uint32_t VC0_P_RX_Q_CTRL_OFF;
    volatile uint32_t VC0_NP_RX_Q_CTRL_OFF;
    volatile uint32_t VC0_CPL_RX_Q_CTRL_OFF;
    volatile uint32_t VC1_P_RX_Q_CTRL_OFF;
    volatile uint32_t VC1_NP_RX_Q_CTRL_OFF;
    volatile uint32_t VC1_CPL_RX_Q_CTRL_OFF;
    volatile uint32_t VC2_P_RX_Q_CTRL_OFF;
    volatile uint32_t VC2_NP_RX_Q_CTRL_OFF;
    volatile uint32_t VC2_CPL_RX_Q_CTRL_OFF;
    volatile uint32_t VC3_P_RX_Q_CTRL_OFF;
    volatile uint32_t VC3_NP_RX_Q_CTRL_OFF;
    volatile uint32_t VC3_CPL_RX_Q_CTRL_OFF;
#else
    CSL_pcie_rc_vcRxQCtrlRegs VC_RX_Q_CTRL[4];
#endif
    volatile uint8_t  Resv_6156[148];
    volatile uint32_t GEN2_CTRL_OFF;
    volatile uint32_t PHY_STATUS_OFF;
    volatile uint32_t PHY_CONTROL_OFF;
    volatile uint8_t  Resv_6172[4];
    volatile uint32_t TRGT_MAP_CTRL_OFF;
    volatile uint32_t MSI_CTRL_ADDR_OFF;
    volatile uint32_t MSI_CTRL_UPPER_ADDR_OFF;
#ifdef CSL_MODIFICATION
    volatile uint32_t MSI_CTRL_INT_0_EN_OFF;
    volatile uint32_t MSI_CTRL_INT_0_MASK_OFF;
    volatile uint32_t MSI_CTRL_INT_0_STATUS_OFF;
    volatile uint32_t MSI_CTRL_INT_1_EN_OFF;
    volatile uint32_t MSI_CTRL_INT_1_MASK_OFF;
    volatile uint32_t MSI_CTRL_INT_1_STATUS_OFF;
    volatile uint32_t MSI_CTRL_INT_2_EN_OFF;
    volatile uint32_t MSI_CTRL_INT_2_MASK_OFF;
    volatile uint32_t MSI_CTRL_INT_2_STATUS_OFF;
    volatile uint32_t MSI_CTRL_INT_3_EN_OFF;
    volatile uint32_t MSI_CTRL_INT_3_MASK_OFF;
    volatile uint32_t MSI_CTRL_INT_3_STATUS_OFF;
    volatile uint32_t MSI_CTRL_INT_4_EN_OFF;
    volatile uint32_t MSI_CTRL_INT_4_MASK_OFF;
    volatile uint32_t MSI_CTRL_INT_4_STATUS_OFF;
    volatile uint32_t MSI_CTRL_INT_5_EN_OFF;
    volatile uint32_t MSI_CTRL_INT_5_MASK_OFF;
    volatile uint32_t MSI_CTRL_INT_5_STATUS_OFF;
    volatile uint32_t MSI_CTRL_INT_6_EN_OFF;
    volatile uint32_t MSI_CTRL_INT_6_MASK_OFF;
    volatile uint32_t MSI_CTRL_INT_6_STATUS_OFF;
    volatile uint32_t MSI_CTRL_INT_7_EN_OFF;
    volatile uint32_t MSI_CTRL_INT_7_MASK_OFF;
    volatile uint32_t MSI_CTRL_INT_7_STATUS_OFF;
#else /* CSL_MODIFICATION */
    CSL_pcie_rc_msiCtrlRegs MSI_CTRL[8];
#endif /* CSL_MODIFICATION */
    volatile uint32_t MSI_GPIO_IO_OFF;
    volatile uint32_t CLOCK_GATING_CTRL_OFF;
    volatile uint32_t GEN3_RELATED_OFF;
    volatile uint8_t  Resv_6312[20];
    volatile uint32_t GEN3_EQ_CONTROL_OFF;
    volatile uint8_t  Resv_6324[8];
    volatile uint32_t ORDER_RULE_CTRL_OFF;
    volatile uint32_t PIPE_LOOPBACK_CONTROL_OFF;
    volatile uint32_t MISC_CONTROL_1_OFF;
    volatile uint32_t MULTI_LANE_CONTROL_OFF;
    volatile uint32_t PHY_INTEROP_CTRL_OFF;
    volatile uint32_t TRGT_CPL_LUT_DELETE_ENTRY_OFF;
    volatile uint32_t LINK_FLUSH_CONTROL_OFF;
    volatile uint32_t AMBA_ERROR_RESPONSE_DEFAULT_OFF;
    volatile uint32_t AMBA_LINK_TIMEOUT_OFF;
    volatile uint32_t AMBA_ORDERING_CTRL_OFF;
    volatile uint8_t  Resv_6368[4];
    volatile uint32_t COHERENCY_CONTROL_1_OFF;
    volatile uint32_t COHERENCY_CONTROL_2_OFF;
    volatile uint32_t COHERENCY_CONTROL_3_OFF;
    volatile uint8_t  Resv_6384[4];
    volatile uint32_t AXI_MSTR_MSG_ADDR_LOW_OFF;
    volatile uint32_t AXI_MSTR_MSG_ADDR_HIGH_OFF;
    volatile uint32_t PCIE_VERSION_NUMBER_OFF;
    volatile uint32_t PCIE_VERSION_TYPE_OFF;
    volatile uint8_t  Resv_6464[64];
    volatile uint32_t MSIX_ADDRESS_MATCH_LOW_OFF;
    volatile uint32_t MSIX_ADDRESS_MATCH_HIGH_OFF;
    volatile uint32_t MSIX_DOORBELL_OFF;
    volatile uint32_t MSIX_RAM_CTRL_OFF;
    volatile uint8_t  Resv_6976[496];
    volatile uint32_t AUX_CLK_FREQ_OFF;
    volatile uint32_t L1_SUBSTATES_OFF;
    volatile uint8_t  Resv_24576[17592];
#ifdef CSL_MODIFICATION
    volatile uint32_t IATU_REGION_CTRL_1_OFF_OUTBOUND_0;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_OUTBOUND_0;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_OUTBOUND_0;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_0;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_OUTBOUND_0;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_0;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_0;
    volatile uint8_t  Resv_24832[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_INBOUND_0;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_INBOUND_0;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_INBOUND_0;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_INBOUND_0;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_INBOUND_0;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_INBOUND_0;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_INBOUND_0;
    volatile uint8_t  Resv_25088[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_OUTBOUND_1;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_OUTBOUND_1;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_OUTBOUND_1;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_1;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_OUTBOUND_1;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_1;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_1;
    volatile uint8_t  Resv_25344[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_INBOUND_1;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_INBOUND_1;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_INBOUND_1;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_INBOUND_1;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_INBOUND_1;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_INBOUND_1;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_INBOUND_1;
    volatile uint8_t  Resv_25600[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_OUTBOUND_2;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_OUTBOUND_2;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_OUTBOUND_2;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_2;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_OUTBOUND_2;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_2;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_2;
    volatile uint8_t  Resv_25856[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_INBOUND_2;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_INBOUND_2;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_INBOUND_2;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_INBOUND_2;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_INBOUND_2;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_INBOUND_2;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_INBOUND_2;
    volatile uint8_t  Resv_26112[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_OUTBOUND_3;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_OUTBOUND_3;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_OUTBOUND_3;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_3;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_OUTBOUND_3;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_3;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_3;
    volatile uint8_t  Resv_26368[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_INBOUND_3;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_INBOUND_3;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_INBOUND_3;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_INBOUND_3;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_INBOUND_3;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_INBOUND_3;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_INBOUND_3;
    volatile uint8_t  Resv_26624[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_OUTBOUND_4;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_OUTBOUND_4;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_OUTBOUND_4;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_4;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_OUTBOUND_4;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_4;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_4;
    volatile uint8_t  Resv_26880[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_INBOUND_4;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_INBOUND_4;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_INBOUND_4;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_INBOUND_4;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_INBOUND_4;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_INBOUND_4;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_INBOUND_4;
    volatile uint8_t  Resv_27136[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_OUTBOUND_5;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_OUTBOUND_5;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_OUTBOUND_5;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_5;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_OUTBOUND_5;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_5;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_5;
    volatile uint8_t  Resv_27392[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_INBOUND_5;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_INBOUND_5;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_INBOUND_5;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_INBOUND_5;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_INBOUND_5;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_INBOUND_5;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_INBOUND_5;
    volatile uint8_t  Resv_27648[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_OUTBOUND_6;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_OUTBOUND_6;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_OUTBOUND_6;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_6;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_OUTBOUND_6;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_6;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_6;
    volatile uint8_t  Resv_27904[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_INBOUND_6;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_INBOUND_6;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_INBOUND_6;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_INBOUND_6;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_INBOUND_6;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_INBOUND_6;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_INBOUND_6;
    volatile uint8_t  Resv_28160[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_OUTBOUND_7;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_OUTBOUND_7;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_OUTBOUND_7;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_7;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_OUTBOUND_7;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_7;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_7;
    volatile uint8_t  Resv_28416[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_INBOUND_7;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_INBOUND_7;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_INBOUND_7;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_INBOUND_7;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_INBOUND_7;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_INBOUND_7;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_INBOUND_7;
    volatile uint8_t  Resv_28672[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_OUTBOUND_8;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_OUTBOUND_8;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_OUTBOUND_8;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_8;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_OUTBOUND_8;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_8;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_8;
    volatile uint8_t  Resv_28928[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_INBOUND_8;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_INBOUND_8;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_INBOUND_8;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_INBOUND_8;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_INBOUND_8;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_INBOUND_8;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_INBOUND_8;
    volatile uint8_t  Resv_29184[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_OUTBOUND_9;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_OUTBOUND_9;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_OUTBOUND_9;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_9;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_OUTBOUND_9;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_9;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_9;
    volatile uint8_t  Resv_29440[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_INBOUND_9;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_INBOUND_9;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_INBOUND_9;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_INBOUND_9;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_INBOUND_9;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_INBOUND_9;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_INBOUND_9;
    volatile uint8_t  Resv_29696[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_OUTBOUND_10;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_OUTBOUND_10;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_OUTBOUND_10;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_10;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_OUTBOUND_10;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_10;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_10;
    volatile uint8_t  Resv_29952[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_INBOUND_10;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_INBOUND_10;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_INBOUND_10;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_INBOUND_10;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_INBOUND_10;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_INBOUND_10;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_INBOUND_10;
    volatile uint8_t  Resv_30208[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_OUTBOUND_11;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_OUTBOUND_11;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_OUTBOUND_11;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_11;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_OUTBOUND_11;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_11;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_11;
    volatile uint8_t  Resv_30464[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_INBOUND_11;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_INBOUND_11;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_INBOUND_11;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_INBOUND_11;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_INBOUND_11;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_INBOUND_11;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_INBOUND_11;
    volatile uint8_t  Resv_30720[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_OUTBOUND_12;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_OUTBOUND_12;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_OUTBOUND_12;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_12;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_OUTBOUND_12;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_12;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_12;
    volatile uint8_t  Resv_30976[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_INBOUND_12;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_INBOUND_12;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_INBOUND_12;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_INBOUND_12;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_INBOUND_12;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_INBOUND_12;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_INBOUND_12;
    volatile uint8_t  Resv_31232[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_OUTBOUND_13;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_OUTBOUND_13;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_OUTBOUND_13;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_13;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_OUTBOUND_13;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_13;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_13;
    volatile uint8_t  Resv_31488[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_INBOUND_13;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_INBOUND_13;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_INBOUND_13;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_INBOUND_13;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_INBOUND_13;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_INBOUND_13;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_INBOUND_13;
    volatile uint8_t  Resv_31744[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_OUTBOUND_14;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_OUTBOUND_14;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_OUTBOUND_14;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_14;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_OUTBOUND_14;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_14;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_14;
    volatile uint8_t  Resv_32000[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_INBOUND_14;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_INBOUND_14;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_INBOUND_14;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_INBOUND_14;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_INBOUND_14;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_INBOUND_14;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_INBOUND_14;
    volatile uint8_t  Resv_32256[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_OUTBOUND_15;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_OUTBOUND_15;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_OUTBOUND_15;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_15;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_OUTBOUND_15;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_15;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_15;
    volatile uint8_t  Resv_32512[228];
    volatile uint32_t IATU_REGION_CTRL_1_OFF_INBOUND_15;
    volatile uint32_t IATU_REGION_CTRL_2_OFF_INBOUND_15;
    volatile uint32_t IATU_LWR_BASE_ADDR_OFF_INBOUND_15;
    volatile uint32_t IATU_UPPER_BASE_ADDR_OFF_INBOUND_15;
    volatile uint32_t IATU_LIMIT_ADDR_OFF_INBOUND_15;
    volatile uint32_t IATU_LWR_TARGET_ADDR_OFF_INBOUND_15;
    volatile uint32_t IATU_UPPER_TARGET_ADDR_OFF_INBOUND_15;
#else /* CSL_MODIFICATION */
    CSL_pcie_rc_iatuRegs iatu[16];
#endif /* CSL_MODIFICATION */
} CSL_pcie_rc_coreRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_PCIE_RC_CORE_PID                                                      (0x00000000U)
#define CSL_PCIE_RC_CORE_CMD_STATUS                                               (0x00000004U)
#define CSL_PCIE_RC_CORE_RSTCMD                                                   (0x00000014U)
#define CSL_PCIE_RC_CORE_PTMCFG                                                   (0x00000018U)
#define CSL_PCIE_RC_CORE_PMCMD                                                    (0x00000020U)
#define CSL_PCIE_RC_CORE_IRQ_EOI                                                  (0x00000050U)
#define CSL_PCIE_RC_CORE_MMR_IRQ                                                  (0x00000054U)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_SET                                           (0x00000064U)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_CLR                                           (0x00000068U)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_STATUS                                        (0x0000006CU)
#ifdef CSL_MODIFICATION
#define CSL_PCIE_RC_CORE_GPR0                                                     (0x00000070U)
#define CSL_PCIE_RC_CORE_GPR1                                                     (0x00000074U)
#define CSL_PCIE_RC_CORE_GPR2                                                     (0x00000078U)
#define CSL_PCIE_RC_CORE_GPR3                                                     (0x0000007CU)
#else /* CSL_MODIFICATION */
#define CSL_PCIE_RC_CORE_GPR(x)                                                   ((0x00000070U) + (0x4u * (x)))
#endif /* CSL_MODIFICATION */
#ifdef CSL_MODIFICATION
#define CSL_PCIE_RC_CORE_MMR0_IRQ_STATUS_RAW                                      (0x00000100U)
#define CSL_PCIE_RC_CORE_MMR0_IRQ_STATUS                                          (0x00000104U)
#define CSL_PCIE_RC_CORE_MMR0_IRQ_ENABLE_SET                                      (0x00000108U)
#define CSL_PCIE_RC_CORE_MMR0_IRQ_ENABLE_CLR                                      (0x0000010CU)
#define CSL_PCIE_RC_CORE_MMR1_IRQ_STATUS_RAW                                      (0x00000110U)
#define CSL_PCIE_RC_CORE_MMR1_IRQ_STATUS                                          (0x00000114U)
#define CSL_PCIE_RC_CORE_MMR1_IRQ_ENABLE_SET                                      (0x00000118U)
#define CSL_PCIE_RC_CORE_MMR1_IRQ_ENABLE_CLR                                      (0x0000011CU)
#define CSL_PCIE_RC_CORE_MMR2_IRQ_STATUS_RAW                                      (0x00000120U)
#define CSL_PCIE_RC_CORE_MMR2_IRQ_STATUS                                          (0x00000124U)
#define CSL_PCIE_RC_CORE_MMR2_IRQ_ENABLE_SET                                      (0x00000128U)
#define CSL_PCIE_RC_CORE_MMR2_IRQ_ENABLE_CLR                                      (0x0000012CU)
#define CSL_PCIE_RC_CORE_MMR3_IRQ_STATUS_RAW                                      (0x00000130U)
#define CSL_PCIE_RC_CORE_MMR3_IRQ_STATUS                                          (0x00000134U)
#define CSL_PCIE_RC_CORE_MMR3_IRQ_ENABLE_SET                                      (0x00000138U)
#define CSL_PCIE_RC_CORE_MMR3_IRQ_ENABLE_CLR                                      (0x0000013CU)
#define CSL_PCIE_RC_CORE_MMR4_IRQ_STATUS_RAW                                      (0x00000140U)
#define CSL_PCIE_RC_CORE_MMR4_IRQ_STATUS                                          (0x00000144U)
#define CSL_PCIE_RC_CORE_MMR4_IRQ_ENABLE_SET                                      (0x00000148U)
#define CSL_PCIE_RC_CORE_MMR4_IRQ_ENABLE_CLR                                      (0x0000014CU)
#define CSL_PCIE_RC_CORE_MMR5_IRQ_STATUS_RAW                                      (0x00000150U)
#define CSL_PCIE_RC_CORE_MMR5_IRQ_STATUS                                          (0x00000154U)
#define CSL_PCIE_RC_CORE_MMR5_IRQ_ENABLE_SET                                      (0x00000158U)
#define CSL_PCIE_RC_CORE_MMR5_IRQ_ENABLE_CLR                                      (0x0000015CU)
#define CSL_PCIE_RC_CORE_MMR6_IRQ_STATUS_RAW                                      (0x00000160U)
#define CSL_PCIE_RC_CORE_MMR6_IRQ_STATUS                                          (0x00000164U)
#define CSL_PCIE_RC_CORE_MMR6_IRQ_ENABLE_SET                                      (0x00000168U)
#define CSL_PCIE_RC_CORE_MMR6_IRQ_ENABLE_CLR                                      (0x0000016CU)
#define CSL_PCIE_RC_CORE_MMR7_IRQ_STATUS_RAW                                      (0x00000170U)
#define CSL_PCIE_RC_CORE_MMR7_IRQ_STATUS                                          (0x00000174U)
#define CSL_PCIE_RC_CORE_MMR7_IRQ_ENABLE_SET                                      (0x00000178U)
#define CSL_PCIE_RC_CORE_MMR7_IRQ_ENABLE_CLR                                      (0x0000017CU)
#else /* CSL_MODIFICATION */
#define CSL_PCIE_RC_CORE_MMR_IRQ_STATUS_RAW(x)                                    ((0x00000100U) + (0x10U * (x)))
#define CSL_PCIE_RC_CORE_MMR_IRQ_STATUS(x)                                        ((0x00000104U) + (0x10U * (x)))
#define CSL_PCIE_RC_CORE_MMR_IRQ_ENABLE_SET(x)                                    ((0x00000108U) + (0x10U * (x)))
#define CSL_PCIE_RC_CORE_MMR_IRQ_ENABLE_CLR(x)                                    ((0x0000010CU) + (0x10U * (x)))
#endif /* CSL_MODIFICATION */
#ifdef CSL_MODIFICATION
#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_STATUS_RAW                                  (0x00000180U)
#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_STATUS                                      (0x00000184U)
#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_ENABLE_SET                                  (0x00000188U)
#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_ENABLE_CLR                                  (0x0000018CU)
#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_STATUS_RAW                                  (0x00000190U)
#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_STATUS                                      (0x00000194U)
#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_ENABLE_SET                                  (0x00000198U)
#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_ENABLE_CLR                                  (0x0000019CU)
#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_STATUS_RAW                                  (0x000001A0U)
#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_STATUS                                      (0x000001A4U)
#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_ENABLE_SET                                  (0x000001A8U)
#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_ENABLE_CLR                                  (0x000001ACU)
#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_STATUS_RAW                                  (0x000001B0U)
#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_STATUS                                      (0x000001B4U)
#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_ENABLE_SET                                  (0x000001B8U)
#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_ENABLE_CLR                                  (0x000001BCU)
#else /* CSL_MODIFICATION */
#define CSL_PCIE_RC_CORE_LEGACY_X_IRQ_STATUS_RAW(x)                               ((0x00000180U) + (0x10u * (x)))
#define CSL_PCIE_RC_CORE_LEGACY_X_IRQ_STATUS(x)                                   ((0x00000184U) + (0x10u * (x)))
#define CSL_PCIE_RC_CORE_LEGACY_X_IRQ_ENABLE_SET(x)                               ((0x00000188U) + (0x10u * (x)))
#define CSL_PCIE_RC_CORE_LEGACY_X_IRQ_ENABLE_CLR(x)                               ((0x0000018CU) + (0x10u * (x)))
#endif /* CSL_MODIFICATION */
#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_RAW                                       (0x000001C0U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS                                           (0x000001C4U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_SET                                       (0x000001C8U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_CLR                                       (0x000001CCU)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_RAW                                     (0x000001D0U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS                                         (0x000001D4U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_SET                                     (0x000001D8U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_CLR                                     (0x000001DCU)
#define CSL_PCIE_RC_CORE_PTM_IRQ_STATUS_RAW                                       (0x000001E0U)
#define CSL_PCIE_RC_CORE_PTM_IRQ_STATUS                                           (0x000001E4U)
#define CSL_PCIE_RC_CORE_PTM_IRQ_ENABLE_SET                                       (0x000001E8U)
#define CSL_PCIE_RC_CORE_PTM_IRQ_ENABLE_CLR                                       (0x000001ECU)
#define CSL_PCIE_RC_CORE_TYPE1_DEV_ID_VEND_ID_REG                                 (0x00001000U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG                                 (0x00001004U)
#define CSL_PCIE_RC_CORE_TYPE1_CLASS_CODE_REV_ID_REG                              (0x00001008U)
#define CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG              (0x0000100CU)
#define CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG                (0x00001018U)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG                            (0x0000101CU)
#define CSL_PCIE_RC_CORE_MEM_LIMIT_MEM_BASE_REG                                   (0x00001020U)
#define CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG                         (0x00001024U)
#define CSL_PCIE_RC_CORE_PREF_BASE_UPPER_REG                                      (0x00001028U)
#define CSL_PCIE_RC_CORE_PREF_LIMIT_UPPER_REG                                     (0x0000102CU)
#define CSL_PCIE_RC_CORE_IO_LIMIT_UPPER_IO_BASE_UPPER_REG                         (0x00001030U)
#define CSL_PCIE_RC_CORE_TYPE1_CAP_PTR_REG                                        (0x00001034U)
#define CSL_PCIE_RC_CORE_TYPE1_EXP_ROM_BASE_REG                                   (0x00001038U)
#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG                         (0x0000103CU)
#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG                                       (0x00001040U)
#define CSL_PCIE_RC_CORE_CON_STATUS_REG                                           (0x00001044U)
#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG                             (0x00001050U)
#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_04H_REG                                      (0x00001054U)
#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_08H_REG                                      (0x00001058U)
#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_0CH_REG                                      (0x0000105CU)
#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_10H_REG                                      (0x00001060U)
#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_14H_REG                                      (0x00001064U)
#define CSL_PCIE_RC_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG               (0x00001070U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES_REG                                  (0x00001074U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS                             (0x00001078U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG                                    (0x0000107CU)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG                             (0x00001080U)
#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG                                    (0x00001084U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS                                 (0x00001088U)
#define CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG                       (0x0000108CU)
#define CSL_PCIE_RC_CORE_ROOT_STATUS_REG                                          (0x00001090U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG                                 (0x00001094U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL2_DEVICE_STATUS2_REG                       (0x00001098U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES2_REG                                   (0x0000109CU)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG                           (0x000010A0U)
#define CSL_PCIE_RC_CORE_PCI_MSIX_CAP_ID_NEXT_CTRL_REG                            (0x000010B0U)
#define CSL_PCIE_RC_CORE_MSIX_TABLE_OFFSET_REG                                    (0x000010B4U)
#define CSL_PCIE_RC_CORE_MSIX_PBA_OFFSET_REG                                      (0x000010B8U)
#define CSL_PCIE_RC_CORE_AER_EXT_CAP_HDR_OFF                                      (0x00001100U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF                                    (0x00001104U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF                                      (0x00001108U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF                                       (0x0000110CU)
#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF                                      (0x00001110U)
#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF                                        (0x00001114U)
#define CSL_PCIE_RC_CORE_ADV_ERR_CAP_CTRL_OFF                                     (0x00001118U)
#ifdef CSL_MODIFICATION
#define CSL_PCIE_RC_CORE_HDR_LOG_0_OFF                                            (0x0000111CU)
#define CSL_PCIE_RC_CORE_HDR_LOG_1_OFF                                            (0x00001120U)
#define CSL_PCIE_RC_CORE_HDR_LOG_2_OFF                                            (0x00001124U)
#define CSL_PCIE_RC_CORE_HDR_LOG_3_OFF                                            (0x00001128U)
#else /* CSL_MODIFICATION */
#define CSL_PCIE_RC_CORE_HDR_LOG_n_OFF(x)                                         ((0x0000111CU) * (0x4U * (x)))
#endif /* CSL_MODIFICATION */
#define CSL_PCIE_RC_CORE_ROOT_ERR_CMD_OFF                                         (0x0000112CU)
#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF                                      (0x00001130U)
#define CSL_PCIE_RC_CORE_ERR_SRC_ID_OFF                                           (0x00001134U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_1_OFF                                     (0x00001138U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_2_OFF                                     (0x0000113CU)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_3_OFF                                     (0x00001140U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_4_OFF                                     (0x00001144U)
#define CSL_PCIE_RC_CORE_VC_BASE                                                  (0x00001148U)
#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_1                                    (0x0000114CU)
#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_2                                    (0x00001150U)
#define CSL_PCIE_RC_CORE_VC_STATUS_CONTROL_REG                                    (0x00001154U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC0                                     (0x00001158U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0                                     (0x0000115CU)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC0                                  (0x00001160U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC1                                     (0x00001164U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1                                     (0x00001168U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC1                                  (0x0000116CU)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC2                                     (0x00001170U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2                                     (0x00001174U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC2                                  (0x00001178U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC3                                     (0x0000117CU)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3                                     (0x00001180U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC3                                  (0x00001184U)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_HEADER_REG                                     (0x00001198U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL3_REG                                        (0x0000119CU)
#define CSL_PCIE_RC_CORE_LANE_ERR_STATUS_REG                                      (0x000011A0U)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG                                    (0x000011A4U)
#define CSL_PCIE_RC_CORE_L1SUB_CAP_HEADER_REG                                     (0x000011A8U)
#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG                                     (0x000011ACU)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG                                       (0x000011B0U)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL2_REG                                       (0x000011B4U)
#define CSL_PCIE_RC_CORE_PTM_EXT_CAP_HDR_OFF                                      (0x000011B8U)
#define CSL_PCIE_RC_CORE_PTM_CAP_OFF                                              (0x000011BCU)
#define CSL_PCIE_RC_CORE_PTM_CONTROL_OFF                                          (0x000011C0U)
#define CSL_PCIE_RC_CORE_PTM_RES_CAP_HDR_OFF                                      (0x000011C4U)
#define CSL_PCIE_RC_CORE_PTM_RES_HDR_OFF                                          (0x000011C8U)
#define CSL_PCIE_RC_CORE_PTM_RES_CONTROL_OFF                                      (0x000011CCU)
#define CSL_PCIE_RC_CORE_PTM_RES_STATUS_OFF                                       (0x000011D0U)
#define CSL_PCIE_RC_CORE_PTM_RES_LOCAL_LSB_OFF                                    (0x000011D4U)
#define CSL_PCIE_RC_CORE_PTM_RES_LOCAL_MSB_OFF                                    (0x000011D8U)
#define CSL_PCIE_RC_CORE_PTM_RES_T2_LSB_OFF                                       (0x000011DCU)
#define CSL_PCIE_RC_CORE_PTM_RES_T2_MSB_OFF                                       (0x000011E0U)
#define CSL_PCIE_RC_CORE_PTM_RES_T2P_LSB_OFF                                      (0x000011E4U)
#define CSL_PCIE_RC_CORE_PTM_RES_T2P_MSB_OFF                                      (0x000011E8U)
#define CSL_PCIE_RC_CORE_PTM_RES_T3_LSB_OFF                                       (0x000011ECU)
#define CSL_PCIE_RC_CORE_PTM_RES_T3_MSB_OFF                                       (0x000011F0U)
#define CSL_PCIE_RC_CORE_PTM_RES_T3P_LSB_OFF                                      (0x000011F4U)
#define CSL_PCIE_RC_CORE_PTM_RES_T3P_MSB_OFF                                      (0x000011F8U)
#define CSL_PCIE_RC_CORE_PTM_RES_TX_LATENCY_OFF                                   (0x000011FCU)
#define CSL_PCIE_RC_CORE_PTM_RES_RX_LATENCY_OFF                                   (0x00001200U)
#define CSL_PCIE_RC_CORE_ACK_LATENCY_TIMER_OFF                                    (0x00001700U)
#define CSL_PCIE_RC_CORE_VENDOR_SPEC_DLLP_OFF                                     (0x00001704U)
#define CSL_PCIE_RC_CORE_PORT_FORCE_OFF                                           (0x00001708U)
#define CSL_PCIE_RC_CORE_ACK_F_ASPM_CTRL_OFF                                      (0x0000170CU)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF                                       (0x00001710U)
#define CSL_PCIE_RC_CORE_LANE_SKEW_OFF                                            (0x00001714U)
#define CSL_PCIE_RC_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF                              (0x00001718U)
#define CSL_PCIE_RC_CORE_SYMBOL_TIMER_FILTER_1_OFF                                (0x0000171CU)
#define CSL_PCIE_RC_CORE_FILTER_MASK_2_OFF                                        (0x00001720U)
#define CSL_PCIE_RC_CORE_AMBA_MUL_OB_DECOMP_NP_SUB_REQ_CTRL_OFF                   (0x00001724U)
#define CSL_PCIE_RC_CORE_PL_DEBUG0_OFF                                            (0x00001728U)
#define CSL_PCIE_RC_CORE_PL_DEBUG1_OFF                                            (0x0000172CU)
#define CSL_PCIE_RC_CORE_TX_P_FC_CREDIT_STATUS_OFF                                (0x00001730U)
#define CSL_PCIE_RC_CORE_TX_NP_FC_CREDIT_STATUS_OFF                               (0x00001734U)
#define CSL_PCIE_RC_CORE_TX_CPL_FC_CREDIT_STATUS_OFF                              (0x00001738U)
#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF                                         (0x0000173CU)
#define CSL_PCIE_RC_CORE_VC_TX_ARBI_1_OFF                                         (0x00001740U)
#define CSL_PCIE_RC_CORE_VC_TX_ARBI_2_OFF                                         (0x00001744U)
#ifdef CSL_MODIFCATION
#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF                                      (0x00001748U)
#define CSL_PCIE_RC_CORE_VC0_NP_RX_Q_CTRL_OFF                                     (0x0000174CU)
#define CSL_PCIE_RC_CORE_VC0_CPL_RX_Q_CTRL_OFF                                    (0x00001750U)
#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF                                      (0x00001754U)
#define CSL_PCIE_RC_CORE_VC1_NP_RX_Q_CTRL_OFF                                     (0x00001758U)
#define CSL_PCIE_RC_CORE_VC1_CPL_RX_Q_CTRL_OFF                                    (0x0000175CU)
#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF                                      (0x00001760U)
#define CSL_PCIE_RC_CORE_VC2_NP_RX_Q_CTRL_OFF                                     (0x00001764U)
#define CSL_PCIE_RC_CORE_VC2_CPL_RX_Q_CTRL_OFF                                    (0x00001768U)
#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF                                      (0x0000176CU)
#define CSL_PCIE_RC_CORE_VC3_NP_RX_Q_CTRL_OFF                                     (0x00001770U)
#define CSL_PCIE_RC_CORE_VC3_CPL_RX_Q_CTRL_OFF                                    (0x00001774U)
#else
#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF(n)                                    ((0x00001748U) + (0xCU * (n)))
#define CSL_PCIE_RC_CORE_VC_NP_RX_Q_CTRL_OFF(n)                                   ((0x0000174CU) + (0xCU * (n)))
#define CSL_PCIE_RC_CORE_VC_CPL_RX_Q_CTRL_OFF(n)                                  ((0x00001750U) + (0xCU * (n)))
#endif
#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF                                            (0x0000180CU)
#define CSL_PCIE_RC_CORE_PHY_STATUS_OFF                                           (0x00001810U)
#define CSL_PCIE_RC_CORE_PHY_CONTROL_OFF                                          (0x00001814U)
#define CSL_PCIE_RC_CORE_TRGT_MAP_CTRL_OFF                                        (0x0000181CU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_ADDR_OFF                                        (0x00001820U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_UPPER_ADDR_OFF                                  (0x00001824U)
#ifdef CSL_MODIFICATION
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_0_EN_OFF                                    (0x00001828U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_0_MASK_OFF                                  (0x0000182CU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_0_STATUS_OFF                                (0x00001830U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_1_EN_OFF                                    (0x00001834U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_1_MASK_OFF                                  (0x00001838U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_1_STATUS_OFF                                (0x0000183CU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_2_EN_OFF                                    (0x00001840U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_2_MASK_OFF                                  (0x00001844U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_2_STATUS_OFF                                (0x00001848U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_3_EN_OFF                                    (0x0000184CU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_3_MASK_OFF                                  (0x00001850U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_3_STATUS_OFF                                (0x00001854U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_4_EN_OFF                                    (0x00001858U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_4_MASK_OFF                                  (0x0000185CU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_4_STATUS_OFF                                (0x00001860U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_5_EN_OFF                                    (0x00001864U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_5_MASK_OFF                                  (0x00001868U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_5_STATUS_OFF                                (0x0000186CU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_6_EN_OFF                                    (0x00001870U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_6_MASK_OFF                                  (0x00001874U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_6_STATUS_OFF                                (0x00001878U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_7_EN_OFF                                    (0x0000187CU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_7_MASK_OFF                                  (0x00001880U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_7_STATUS_OFF                                (0x00001884U)
#else /* CSL_MODIFICATION */
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_EN_OFF(n)                                   ((0x00001828U) + (0xCU * (n)))
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_MASK_OFF(n)                                 ((0x0000182CU) + (0xCU * (n)))
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_STATUS_OFF(n)                               ((0x00001830U) + (0xCU * (n)))
#endif /* CSL_MODIFICATION */
#define CSL_PCIE_RC_CORE_MSI_GPIO_IO_OFF                                          (0x00001888U)
#define CSL_PCIE_RC_CORE_CLOCK_GATING_CTRL_OFF                                    (0x0000188CU)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF                                         (0x00001890U)
#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF                                      (0x000018A8U)
#define CSL_PCIE_RC_CORE_ORDER_RULE_CTRL_OFF                                      (0x000018B4U)
#define CSL_PCIE_RC_CORE_PIPE_LOOPBACK_CONTROL_OFF                                (0x000018B8U)
#define CSL_PCIE_RC_CORE_MISC_CONTROL_1_OFF                                       (0x000018BCU)
#define CSL_PCIE_RC_CORE_MULTI_LANE_CONTROL_OFF                                   (0x000018C0U)
#define CSL_PCIE_RC_CORE_PHY_INTEROP_CTRL_OFF                                     (0x000018C4U)
#define CSL_PCIE_RC_CORE_TRGT_CPL_LUT_DELETE_ENTRY_OFF                            (0x000018C8U)
#define CSL_PCIE_RC_CORE_LINK_FLUSH_CONTROL_OFF                                   (0x000018CCU)
#define CSL_PCIE_RC_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF                          (0x000018D0U)
#define CSL_PCIE_RC_CORE_AMBA_LINK_TIMEOUT_OFF                                    (0x000018D4U)
#define CSL_PCIE_RC_CORE_AMBA_ORDERING_CTRL_OFF                                   (0x000018D8U)
#define CSL_PCIE_RC_CORE_COHERENCY_CONTROL_1_OFF                                  (0x000018E0U)
#define CSL_PCIE_RC_CORE_COHERENCY_CONTROL_2_OFF                                  (0x000018E4U)
#define CSL_PCIE_RC_CORE_COHERENCY_CONTROL_3_OFF                                  (0x000018E8U)
#define CSL_PCIE_RC_CORE_AXI_MSTR_MSG_ADDR_LOW_OFF                                (0x000018F0U)
#define CSL_PCIE_RC_CORE_AXI_MSTR_MSG_ADDR_HIGH_OFF                               (0x000018F4U)
#define CSL_PCIE_RC_CORE_PCIE_VERSION_NUMBER_OFF                                  (0x000018F8U)
#define CSL_PCIE_RC_CORE_PCIE_VERSION_TYPE_OFF                                    (0x000018FCU)
#define CSL_PCIE_RC_CORE_MSIX_ADDRESS_MATCH_LOW_OFF                               (0x00001940U)
#define CSL_PCIE_RC_CORE_MSIX_ADDRESS_MATCH_HIGH_OFF                              (0x00001944U)
#define CSL_PCIE_RC_CORE_MSIX_DOORBELL_OFF                                        (0x00001948U)
#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF                                        (0x0000194CU)
#define CSL_PCIE_RC_CORE_AUX_CLK_FREQ_OFF                                         (0x00001B40U)
#define CSL_PCIE_RC_CORE_L1_SUBSTATES_OFF                                         (0x00001B44U)
#ifdef CSL_MODIFICATION
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_0                        (0x00006000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0                        (0x00006004U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_0                        (0x00006008U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_0                      (0x0000600CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_0                           (0x00006010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_0                      (0x00006014U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_0                    (0x00006018U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_0                         (0x00006100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0                         (0x00006104U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_0                         (0x00006108U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_0                       (0x0000610CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_0                            (0x00006110U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_0                       (0x00006114U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_0                     (0x00006118U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_1                        (0x00006200U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1                        (0x00006204U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_1                        (0x00006208U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_1                      (0x0000620CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_1                           (0x00006210U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_1                      (0x00006214U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_1                    (0x00006218U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_1                         (0x00006300U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1                         (0x00006304U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_1                         (0x00006308U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_1                       (0x0000630CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_1                            (0x00006310U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_1                       (0x00006314U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_1                     (0x00006318U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_2                        (0x00006400U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2                        (0x00006404U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_2                        (0x00006408U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_2                      (0x0000640CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_2                           (0x00006410U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_2                      (0x00006414U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_2                    (0x00006418U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_2                         (0x00006500U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2                         (0x00006504U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_2                         (0x00006508U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_2                       (0x0000650CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_2                            (0x00006510U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_2                       (0x00006514U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_2                     (0x00006518U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_3                        (0x00006600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3                        (0x00006604U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_3                        (0x00006608U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_3                      (0x0000660CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_3                           (0x00006610U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_3                      (0x00006614U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_3                    (0x00006618U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_3                         (0x00006700U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3                         (0x00006704U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_3                         (0x00006708U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_3                       (0x0000670CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_3                            (0x00006710U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_3                       (0x00006714U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_3                     (0x00006718U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_4                        (0x00006800U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4                        (0x00006804U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_4                        (0x00006808U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_4                      (0x0000680CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_4                           (0x00006810U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_4                      (0x00006814U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_4                    (0x00006818U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_4                         (0x00006900U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4                         (0x00006904U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_4                         (0x00006908U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_4                       (0x0000690CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_4                            (0x00006910U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_4                       (0x00006914U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_4                     (0x00006918U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_5                        (0x00006A00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5                        (0x00006A04U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_5                        (0x00006A08U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_5                      (0x00006A0CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_5                           (0x00006A10U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_5                      (0x00006A14U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_5                    (0x00006A18U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_5                         (0x00006B00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5                         (0x00006B04U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_5                         (0x00006B08U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_5                       (0x00006B0CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_5                            (0x00006B10U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_5                       (0x00006B14U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_5                     (0x00006B18U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_6                        (0x00006C00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6                        (0x00006C04U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_6                        (0x00006C08U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_6                      (0x00006C0CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_6                           (0x00006C10U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_6                      (0x00006C14U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_6                    (0x00006C18U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_6                         (0x00006D00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6                         (0x00006D04U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_6                         (0x00006D08U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_6                       (0x00006D0CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_6                            (0x00006D10U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_6                       (0x00006D14U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_6                     (0x00006D18U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_7                        (0x00006E00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7                        (0x00006E04U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_7                        (0x00006E08U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_7                      (0x00006E0CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_7                           (0x00006E10U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_7                      (0x00006E14U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_7                    (0x00006E18U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_7                         (0x00006F00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7                         (0x00006F04U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_7                         (0x00006F08U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_7                       (0x00006F0CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_7                            (0x00006F10U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_7                       (0x00006F14U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_7                     (0x00006F18U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_8                        (0x00007000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8                        (0x00007004U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_8                        (0x00007008U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_8                      (0x0000700CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_8                           (0x00007010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_8                      (0x00007014U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_8                    (0x00007018U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_8                         (0x00007100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8                         (0x00007104U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_8                         (0x00007108U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_8                       (0x0000710CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_8                            (0x00007110U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_8                       (0x00007114U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_8                     (0x00007118U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_9                        (0x00007200U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9                        (0x00007204U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_9                        (0x00007208U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_9                      (0x0000720CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_9                           (0x00007210U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_9                      (0x00007214U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_9                    (0x00007218U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_9                         (0x00007300U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9                         (0x00007304U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_9                         (0x00007308U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_9                       (0x0000730CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_9                            (0x00007310U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_9                       (0x00007314U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_9                     (0x00007318U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_10                       (0x00007400U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10                       (0x00007404U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_10                       (0x00007408U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_10                     (0x0000740CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_10                          (0x00007410U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_10                     (0x00007414U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_10                   (0x00007418U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_10                        (0x00007500U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10                        (0x00007504U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_10                        (0x00007508U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_10                      (0x0000750CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_10                           (0x00007510U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_10                      (0x00007514U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_10                    (0x00007518U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_11                       (0x00007600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11                       (0x00007604U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_11                       (0x00007608U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_11                     (0x0000760CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_11                          (0x00007610U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_11                     (0x00007614U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_11                   (0x00007618U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_11                        (0x00007700U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11                        (0x00007704U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_11                        (0x00007708U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_11                      (0x0000770CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_11                           (0x00007710U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_11                      (0x00007714U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_11                    (0x00007718U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_12                       (0x00007800U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12                       (0x00007804U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_12                       (0x00007808U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_12                     (0x0000780CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_12                          (0x00007810U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_12                     (0x00007814U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_12                   (0x00007818U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_12                        (0x00007900U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12                        (0x00007904U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_12                        (0x00007908U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_12                      (0x0000790CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_12                           (0x00007910U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_12                      (0x00007914U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_12                    (0x00007918U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_13                       (0x00007A00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13                       (0x00007A04U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_13                       (0x00007A08U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_13                     (0x00007A0CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_13                          (0x00007A10U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_13                     (0x00007A14U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_13                   (0x00007A18U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_13                        (0x00007B00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13                        (0x00007B04U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_13                        (0x00007B08U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_13                      (0x00007B0CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_13                           (0x00007B10U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_13                      (0x00007B14U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_13                    (0x00007B18U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_14                       (0x00007C00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14                       (0x00007C04U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_14                       (0x00007C08U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_14                     (0x00007C0CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_14                          (0x00007C10U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_14                     (0x00007C14U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_14                   (0x00007C18U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_14                        (0x00007D00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14                        (0x00007D04U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_14                        (0x00007D08U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_14                      (0x00007D0CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_14                           (0x00007D10U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_14                      (0x00007D14U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_14                    (0x00007D18U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_15                       (0x00007E00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15                       (0x00007E04U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_15                       (0x00007E08U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_15                     (0x00007E0CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_15                          (0x00007E10U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_15                     (0x00007E14U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_15                   (0x00007E18U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_15                        (0x00007F00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15                        (0x00007F04U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_15                        (0x00007F08U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_15                      (0x00007F0CU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_15                           (0x00007F10U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_15                      (0x00007F14U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_15                    (0x00007F18U)
#else /* CSL_MODIFICATION */
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND(x)                       (0x00006000U + (0x200U * (x)))
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND(x)                       (0x00006004U + (0x200U * (x)))
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND(x)                       (0x00006008U + (0x200U * (x)))
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND(x)                     (0x0000600CU + (0x200U * (x)))
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND(x)                          (0x00006010U + (0x200U * (x)))
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND(x)                     (0x00006014U + (0x200U * (x)))
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND(x)                   (0x00006018U + (0x200U * (x)))
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND(x)                        (0x00006100U + (0x200U * (x)))
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND(x)                        (0x00006104U + (0x200U * (x)))
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND(x)                        (0x00006108U + (0x200U * (x)))
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND(x)                      (0x0000610CU + (0x200U * (x)))
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND(x)                           (0x00006110U + (0x200U * (x)))
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND(x)                      (0x00006114U + (0x200U * (x)))
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND(x)                    (0x00006118U + (0x200U * (x)))
#endif /* CSL_MODIFICATION */

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_PCIE_RC_CORE_PID_MODID_MASK                                           (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_PID_MODID_SHIFT                                          (0x00000010U)
#define CSL_PCIE_RC_CORE_PID_MODID_MAX                                            (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_PID_RTL_MASK                                             (0x0000F800U)
#define CSL_PCIE_RC_CORE_PID_RTL_SHIFT                                            (0x0000000BU)
#define CSL_PCIE_RC_CORE_PID_RTL_MAX                                              (0x0000001FU)

#define CSL_PCIE_RC_CORE_PID_MAJOR_MASK                                           (0x00000700U)
#define CSL_PCIE_RC_CORE_PID_MAJOR_SHIFT                                          (0x00000008U)
#define CSL_PCIE_RC_CORE_PID_MAJOR_MAX                                            (0x00000007U)

#define CSL_PCIE_RC_CORE_PID_CUSTOM_MASK                                          (0x000000C0U)
#define CSL_PCIE_RC_CORE_PID_CUSTOM_SHIFT                                         (0x00000006U)
#define CSL_PCIE_RC_CORE_PID_CUSTOM_MAX                                           (0x00000003U)

#define CSL_PCIE_RC_CORE_PID_MINOR_MASK                                           (0x0000003FU)
#define CSL_PCIE_RC_CORE_PID_MINOR_SHIFT                                          (0x00000000U)
#define CSL_PCIE_RC_CORE_PID_MINOR_MAX                                            (0x0000003FU)

/* CMD_STATUS */

#define CSL_PCIE_RC_CORE_CMD_STATUS_RSVD11_MASK                                   (0xFFFFFF00U)
#define CSL_PCIE_RC_CORE_CMD_STATUS_RSVD11_SHIFT                                  (0x00000008U)
#define CSL_PCIE_RC_CORE_CMD_STATUS_RSVD11_MAX                                    (0x00FFFFFFU)

#define CSL_PCIE_RC_CORE_CMD_STATUS_RX_LANE_FLIP_EN_MASK                          (0x00000080U)
#define CSL_PCIE_RC_CORE_CMD_STATUS_RX_LANE_FLIP_EN_SHIFT                         (0x00000007U)
#define CSL_PCIE_RC_CORE_CMD_STATUS_RX_LANE_FLIP_EN_MAX                           (0x00000001U)

#define CSL_PCIE_RC_CORE_CMD_STATUS_TX_LANE_FLIP_EN_MASK                          (0x00000040U)
#define CSL_PCIE_RC_CORE_CMD_STATUS_TX_LANE_FLIP_EN_SHIFT                         (0x00000006U)
#define CSL_PCIE_RC_CORE_CMD_STATUS_TX_LANE_FLIP_EN_MAX                           (0x00000001U)

#define CSL_PCIE_RC_CORE_CMD_STATUS_DBI_CS2_MASK                                  (0x00000020U)
#define CSL_PCIE_RC_CORE_CMD_STATUS_DBI_CS2_SHIFT                                 (0x00000005U)
#define CSL_PCIE_RC_CORE_CMD_STATUS_DBI_CS2_MAX                                   (0x00000001U)

#define CSL_PCIE_RC_CORE_CMD_STATUS_APP_RETRY_EN_MASK                             (0x00000010U)
#define CSL_PCIE_RC_CORE_CMD_STATUS_APP_RETRY_EN_SHIFT                            (0x00000004U)
#define CSL_PCIE_RC_CORE_CMD_STATUS_APP_RETRY_EN_MAX                              (0x00000001U)

#define CSL_PCIE_RC_CORE_CMD_STATUS_RSVD10_MASK                                   (0x0000000EU)
#define CSL_PCIE_RC_CORE_CMD_STATUS_RSVD10_SHIFT                                  (0x00000001U)
#define CSL_PCIE_RC_CORE_CMD_STATUS_RSVD10_MAX                                    (0x00000007U)

#define CSL_PCIE_RC_CORE_CMD_STATUS_LTSSM_EN_MASK                                 (0x00000001U)
#define CSL_PCIE_RC_CORE_CMD_STATUS_LTSSM_EN_SHIFT                                (0x00000000U)
#define CSL_PCIE_RC_CORE_CMD_STATUS_LTSSM_EN_MAX                                  (0x00000001U)

/* RSTCMD */

#define CSL_PCIE_RC_CORE_RSTCMD_RSVD21_MASK                                       (0xE0000000U)
#define CSL_PCIE_RC_CORE_RSTCMD_RSVD21_SHIFT                                      (0x0000001DU)
#define CSL_PCIE_RC_CORE_RSTCMD_RSVD21_MAX                                        (0x00000007U)

#define CSL_PCIE_RC_CORE_RSTCMD_FLR_PF_ACTIVE_MASK                                (0x10000000U)
#define CSL_PCIE_RC_CORE_RSTCMD_FLR_PF_ACTIVE_SHIFT                               (0x0000001CU)
#define CSL_PCIE_RC_CORE_RSTCMD_FLR_PF_ACTIVE_MAX                                 (0x00000001U)

#define CSL_PCIE_RC_CORE_RSTCMD_RSVD20_MASK                                       (0x0FFFFFFEU)
#define CSL_PCIE_RC_CORE_RSTCMD_RSVD20_SHIFT                                      (0x00000001U)
#define CSL_PCIE_RC_CORE_RSTCMD_RSVD20_MAX                                        (0x07FFFFFFU)

#define CSL_PCIE_RC_CORE_RSTCMD_INIT_RST_MASK                                     (0x00000001U)
#define CSL_PCIE_RC_CORE_RSTCMD_INIT_RST_SHIFT                                    (0x00000000U)
#define CSL_PCIE_RC_CORE_RSTCMD_INIT_RST_MAX                                      (0x00000001U)

/* PTMCFG */

#define CSL_PCIE_RC_CORE_PTMCFG_RSVD32_MASK                                       (0xFFFFC000U)
#define CSL_PCIE_RC_CORE_PTMCFG_RSVD32_SHIFT                                      (0x0000000EU)
#define CSL_PCIE_RC_CORE_PTMCFG_RSVD32_MAX                                        (0x0003FFFFU)

#define CSL_PCIE_RC_CORE_PTMCFG_PTM_CLK_SEL_MASK                                  (0x00003F00U)
#define CSL_PCIE_RC_CORE_PTMCFG_PTM_CLK_SEL_SHIFT                                 (0x00000008U)
#define CSL_PCIE_RC_CORE_PTMCFG_PTM_CLK_SEL_MAX                                   (0x0000003FU)

#define CSL_PCIE_RC_CORE_PTMCFG_RSVD30_MASK                                       (0x000000F8U)
#define CSL_PCIE_RC_CORE_PTMCFG_RSVD30_SHIFT                                      (0x00000003U)
#define CSL_PCIE_RC_CORE_PTMCFG_RSVD30_MAX                                        (0x0000001FU)

#define CSL_PCIE_RC_CORE_PTMCFG_PTM_CONTEXT_VALID_MASK                            (0x00000004U)
#define CSL_PCIE_RC_CORE_PTMCFG_PTM_CONTEXT_VALID_SHIFT                           (0x00000002U)
#define CSL_PCIE_RC_CORE_PTMCFG_PTM_CONTEXT_VALID_MAX                             (0x00000001U)

#define CSL_PCIE_RC_CORE_PTMCFG_PTM_MANUAL_UPDATE_MASK                            (0x00000002U)
#define CSL_PCIE_RC_CORE_PTMCFG_PTM_MANUAL_UPDATE_SHIFT                           (0x00000001U)
#define CSL_PCIE_RC_CORE_PTMCFG_PTM_MANUAL_UPDATE_MAX                             (0x00000001U)

#define CSL_PCIE_RC_CORE_PTMCFG_PTM_AUTO_UPDATE_MASK                              (0x00000001U)
#define CSL_PCIE_RC_CORE_PTMCFG_PTM_AUTO_UPDATE_SHIFT                             (0x00000000U)
#define CSL_PCIE_RC_CORE_PTMCFG_PTM_AUTO_UPDATE_MAX                               (0x00000001U)

/* PMCMD */

#define CSL_PCIE_RC_CORE_PMCMD_RSVD40_MASK                                        (0xFFFFFFFCU)
#define CSL_PCIE_RC_CORE_PMCMD_RSVD40_SHIFT                                       (0x00000002U)
#define CSL_PCIE_RC_CORE_PMCMD_RSVD40_MAX                                         (0x3FFFFFFFU)

#define CSL_PCIE_RC_CORE_PMCMD_PM_XMT_TURNOFF_MASK                                (0x00000002U)
#define CSL_PCIE_RC_CORE_PMCMD_PM_XMT_TURNOFF_SHIFT                               (0x00000001U)
#define CSL_PCIE_RC_CORE_PMCMD_PM_XMT_TURNOFF_MAX                                 (0x00000001U)

#define CSL_PCIE_RC_CORE_PMCMD_PM_XMT_PE_MASK                                     (0x00000001U)
#define CSL_PCIE_RC_CORE_PMCMD_PM_XMT_PE_SHIFT                                    (0x00000000U)
#define CSL_PCIE_RC_CORE_PMCMD_PM_XMT_PE_MAX                                      (0x00000001U)

/* IRQ_EOI */

#define CSL_PCIE_RC_CORE_IRQ_EOI_RSVD60_MASK                                      (0xFFFFFFE0U)
#define CSL_PCIE_RC_CORE_IRQ_EOI_RSVD60_SHIFT                                     (0x00000005U)
#define CSL_PCIE_RC_CORE_IRQ_EOI_RSVD60_MAX                                       (0x07FFFFFFU)

#define CSL_PCIE_RC_CORE_IRQ_EOI_EOI_MASK                                         (0x0000000FU)
#define CSL_PCIE_RC_CORE_IRQ_EOI_EOI_SHIFT                                        (0x00000000U)
#define CSL_PCIE_RC_CORE_IRQ_EOI_EOI_MAX                                          (0x0000000FU)

/* MMR_IRQ */

#define CSL_PCIE_RC_CORE_MMR_IRQ_MMR_IRQ_MASK                                     (0x7FFFFFFFU)
#define CSL_PCIE_RC_CORE_MMR_IRQ_MMR_IRQ_SHIFT                                    (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR_IRQ_MMR_IRQ_MAX                                      (0x7FFFFFFFU)

/* LEGACY_IRQ_SET */

#define CSL_PCIE_RC_CORE_LEGACY_IRQ_SET_RSVD70_MASK                               (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_SET_RSVD70_SHIFT                              (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_SET_RSVD70_MAX                                (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_LEGACY_IRQ_SET_LEGACY_IRQ_SET_0_MASK                     (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_SET_LEGACY_IRQ_SET_0_SHIFT                    (0x00000000U)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_SET_LEGACY_IRQ_SET_0_MAX                      (0x00000001U)

/* LEGACY_IRQ_CLR */

#define CSL_PCIE_RC_CORE_LEGACY_IRQ_CLR_RSVD80_MASK                               (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_CLR_RSVD80_SHIFT                              (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_CLR_RSVD80_MAX                                (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_LEGACY_IRQ_CLR_LEGACY_IRQ_CLR_0_MASK                     (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_CLR_LEGACY_IRQ_CLR_0_SHIFT                    (0x00000000U)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_CLR_LEGACY_IRQ_CLR_0_MAX                      (0x00000001U)

/* LEGACY_IRQ_STATUS */

#define CSL_PCIE_RC_CORE_LEGACY_IRQ_STATUS_RSVD90_MASK                            (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_STATUS_RSVD90_SHIFT                           (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_STATUS_RSVD90_MAX                             (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_LEGACY_IRQ_STATUS_LEGACY_IRQ_STATUS_0_MASK               (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_STATUS_LEGACY_IRQ_STATUS_0_SHIFT              (0x00000000U)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_STATUS_LEGACY_IRQ_STATUS_0_MAX                (0x00000001U)

#ifdef CSL_MODIFICATION
/* GPR0 */

#define CSL_PCIE_RC_CORE_GPR0_GENERIC0_MASK                                       (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_GPR0_GENERIC0_SHIFT                                      (0x00000000U)
#define CSL_PCIE_RC_CORE_GPR0_GENERIC0_MAX                                        (0xFFFFFFFFU)

/* GPR1 */

#define CSL_PCIE_RC_CORE_GPR1_GENERIC1_MASK                                       (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_GPR1_GENERIC1_SHIFT                                      (0x00000000U)
#define CSL_PCIE_RC_CORE_GPR1_GENERIC1_MAX                                        (0xFFFFFFFFU)

/* GPR2 */

#define CSL_PCIE_RC_CORE_GPR2_GENERIC2_MASK                                       (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_GPR2_GENERIC2_SHIFT                                      (0x00000000U)
#define CSL_PCIE_RC_CORE_GPR2_GENERIC2_MAX                                        (0xFFFFFFFFU)

/* GPR3 */

#define CSL_PCIE_RC_CORE_GPR3_GENERIC3_MASK                                       (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_GPR3_GENERIC3_SHIFT                                      (0x00000000U)
#define CSL_PCIE_RC_CORE_GPR3_GENERIC3_MAX                                        (0xFFFFFFFFU)

#else /* CSL_MODIFICATION */

/* GPRn */

#define CSL_PCIE_RC_CORE_GPR_GENERIC_MASK                                         (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_GPR_GENERIC_SHIFT                                        (0x00000000U)
#define CSL_PCIE_RC_CORE_GPR_GENERIC_MAX                                          (0xFFFFFFFFU)

#endif /* CSL_MODIFICATION */

#ifdef CSL_MODIFICATION
/* MMR0_IRQ_STATUS_RAW */

#define CSL_PCIE_RC_CORE_MMR0_IRQ_STATUS_RAW_RSVD100_MASK                         (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR0_IRQ_STATUS_RAW_RSVD100_SHIFT                        (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR0_IRQ_STATUS_RAW_RSVD100_MAX                          (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR0_IRQ_STATUS_RAW_MMR0_IRQ_STATUS_RAW_MASK             (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR0_IRQ_STATUS_RAW_MMR0_IRQ_STATUS_RAW_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR0_IRQ_STATUS_RAW_MMR0_IRQ_STATUS_RAW_MAX              (0x0000000FU)

/* MMR0_IRQ_STATUS */

#define CSL_PCIE_RC_CORE_MMR0_IRQ_STATUS_RSVD110_MASK                             (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR0_IRQ_STATUS_RSVD110_SHIFT                            (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR0_IRQ_STATUS_RSVD110_MAX                              (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR0_IRQ_STATUS_MMR0_IRQ_STATUS_MASK                     (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR0_IRQ_STATUS_MMR0_IRQ_STATUS_SHIFT                    (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR0_IRQ_STATUS_MMR0_IRQ_STATUS_MAX                      (0x0000000FU)

/* MMR0_IRQ_ENABLE_SET */

#define CSL_PCIE_RC_CORE_MMR0_IRQ_ENABLE_SET_RSVD111_MASK                         (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR0_IRQ_ENABLE_SET_RSVD111_SHIFT                        (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR0_IRQ_ENABLE_SET_RSVD111_MAX                          (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR0_IRQ_ENABLE_SET_MMR0_IRQ_EN_SET_MASK                 (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR0_IRQ_ENABLE_SET_MMR0_IRQ_EN_SET_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR0_IRQ_ENABLE_SET_MMR0_IRQ_EN_SET_MAX                  (0x0000000FU)

/* MMR0_IRQ_ENABLE_CLR */

#define CSL_PCIE_RC_CORE_MMR0_IRQ_ENABLE_CLR_RSVD112_MASK                         (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR0_IRQ_ENABLE_CLR_RSVD112_SHIFT                        (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR0_IRQ_ENABLE_CLR_RSVD112_MAX                          (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR0_IRQ_ENABLE_CLR_MMR0_IRQ_EN_CLR_MASK                 (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR0_IRQ_ENABLE_CLR_MMR0_IRQ_EN_CLR_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR0_IRQ_ENABLE_CLR_MMR0_IRQ_EN_CLR_MAX                  (0x0000000FU)

/* MMR1_IRQ_STATUS_RAW */

#define CSL_PCIE_RC_CORE_MMR1_IRQ_STATUS_RAW_RSVD120_MASK                         (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR1_IRQ_STATUS_RAW_RSVD120_SHIFT                        (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR1_IRQ_STATUS_RAW_RSVD120_MAX                          (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR1_IRQ_STATUS_RAW_MMR1_IRQ_STATUS_RAW_MASK             (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR1_IRQ_STATUS_RAW_MMR1_IRQ_STATUS_RAW_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR1_IRQ_STATUS_RAW_MMR1_IRQ_STATUS_RAW_MAX              (0x0000000FU)

/* MMR1_IRQ_STATUS */

#define CSL_PCIE_RC_CORE_MMR1_IRQ_STATUS_RSVD121_MASK                             (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR1_IRQ_STATUS_RSVD121_SHIFT                            (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR1_IRQ_STATUS_RSVD121_MAX                              (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR1_IRQ_STATUS_MMR1_IRQ_STATUS_MASK                     (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR1_IRQ_STATUS_MMR1_IRQ_STATUS_SHIFT                    (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR1_IRQ_STATUS_MMR1_IRQ_STATUS_MAX                      (0x0000000FU)

/* MMR1_IRQ_ENABLE_SET */

#define CSL_PCIE_RC_CORE_MMR1_IRQ_ENABLE_SET_RSVD122_MASK                         (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR1_IRQ_ENABLE_SET_RSVD122_SHIFT                        (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR1_IRQ_ENABLE_SET_RSVD122_MAX                          (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR1_IRQ_ENABLE_SET_MMR1_IRQ_EN_SET_MASK                 (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR1_IRQ_ENABLE_SET_MMR1_IRQ_EN_SET_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR1_IRQ_ENABLE_SET_MMR1_IRQ_EN_SET_MAX                  (0x0000000FU)

/* MMR1_IRQ_ENABLE_CLR */

#define CSL_PCIE_RC_CORE_MMR1_IRQ_ENABLE_CLR_RSVD123_MASK                         (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR1_IRQ_ENABLE_CLR_RSVD123_SHIFT                        (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR1_IRQ_ENABLE_CLR_RSVD123_MAX                          (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR1_IRQ_ENABLE_CLR_MMR1_IRQ_EN_CLR_MASK                 (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR1_IRQ_ENABLE_CLR_MMR1_IRQ_EN_CLR_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR1_IRQ_ENABLE_CLR_MMR1_IRQ_EN_CLR_MAX                  (0x0000000FU)

/* MMR2_IRQ_STATUS_RAW */

#define CSL_PCIE_RC_CORE_MMR2_IRQ_STATUS_RAW_RSVD130_MASK                         (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR2_IRQ_STATUS_RAW_RSVD130_SHIFT                        (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR2_IRQ_STATUS_RAW_RSVD130_MAX                          (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR2_IRQ_STATUS_RAW_MMR2_IRQ_STATUS_RAW_MASK             (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR2_IRQ_STATUS_RAW_MMR2_IRQ_STATUS_RAW_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR2_IRQ_STATUS_RAW_MMR2_IRQ_STATUS_RAW_MAX              (0x0000000FU)

/* MMR2_IRQ_STATUS */

#define CSL_PCIE_RC_CORE_MMR2_IRQ_STATUS_RSVD131_MASK                             (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR2_IRQ_STATUS_RSVD131_SHIFT                            (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR2_IRQ_STATUS_RSVD131_MAX                              (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR2_IRQ_STATUS_MMR2_IRQ_STATUS_MASK                     (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR2_IRQ_STATUS_MMR2_IRQ_STATUS_SHIFT                    (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR2_IRQ_STATUS_MMR2_IRQ_STATUS_MAX                      (0x0000000FU)

/* MMR2_IRQ_ENABLE_SET */

#define CSL_PCIE_RC_CORE_MMR2_IRQ_ENABLE_SET_RSVD132_MASK                         (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR2_IRQ_ENABLE_SET_RSVD132_SHIFT                        (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR2_IRQ_ENABLE_SET_RSVD132_MAX                          (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR2_IRQ_ENABLE_SET_MMR2_IRQ_EN_SET_MASK                 (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR2_IRQ_ENABLE_SET_MMR2_IRQ_EN_SET_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR2_IRQ_ENABLE_SET_MMR2_IRQ_EN_SET_MAX                  (0x0000000FU)

/* MMR2_IRQ_ENABLE_CLR */

#define CSL_PCIE_RC_CORE_MMR2_IRQ_ENABLE_CLR_RSVD133_MASK                         (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR2_IRQ_ENABLE_CLR_RSVD133_SHIFT                        (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR2_IRQ_ENABLE_CLR_RSVD133_MAX                          (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR2_IRQ_ENABLE_CLR_MMR2_IRQ_EN_CLR_MASK                 (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR2_IRQ_ENABLE_CLR_MMR2_IRQ_EN_CLR_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR2_IRQ_ENABLE_CLR_MMR2_IRQ_EN_CLR_MAX                  (0x0000000FU)

/* MMR3_IRQ_STATUS_RAW */

#define CSL_PCIE_RC_CORE_MMR3_IRQ_STATUS_RAW_RSVD140_MASK                         (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR3_IRQ_STATUS_RAW_RSVD140_SHIFT                        (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR3_IRQ_STATUS_RAW_RSVD140_MAX                          (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR3_IRQ_STATUS_RAW_MMR3_IRQ_STATUS_RAW_MASK             (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR3_IRQ_STATUS_RAW_MMR3_IRQ_STATUS_RAW_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR3_IRQ_STATUS_RAW_MMR3_IRQ_STATUS_RAW_MAX              (0x0000000FU)

/* MMR3_IRQ_STATUS */

#define CSL_PCIE_RC_CORE_MMR3_IRQ_STATUS_RSVD141_MASK                             (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR3_IRQ_STATUS_RSVD141_SHIFT                            (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR3_IRQ_STATUS_RSVD141_MAX                              (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR3_IRQ_STATUS_MMR3_IRQ_STATUS_MASK                     (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR3_IRQ_STATUS_MMR3_IRQ_STATUS_SHIFT                    (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR3_IRQ_STATUS_MMR3_IRQ_STATUS_MAX                      (0x0000000FU)

/* MMR3_IRQ_ENABLE_SET */

#define CSL_PCIE_RC_CORE_MMR3_IRQ_ENABLE_SET_RSVD142_MASK                         (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR3_IRQ_ENABLE_SET_RSVD142_SHIFT                        (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR3_IRQ_ENABLE_SET_RSVD142_MAX                          (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR3_IRQ_ENABLE_SET_MMR3_IRQ_EN_SET_MASK                 (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR3_IRQ_ENABLE_SET_MMR3_IRQ_EN_SET_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR3_IRQ_ENABLE_SET_MMR3_IRQ_EN_SET_MAX                  (0x0000000FU)

/* MMR3_IRQ_ENABLE_CLR */

#define CSL_PCIE_RC_CORE_MMR3_IRQ_ENABLE_CLR_RSVD143_MASK                         (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR3_IRQ_ENABLE_CLR_RSVD143_SHIFT                        (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR3_IRQ_ENABLE_CLR_RSVD143_MAX                          (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR3_IRQ_ENABLE_CLR_MMR3_IRQ_EN_CLR_MASK                 (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR3_IRQ_ENABLE_CLR_MMR3_IRQ_EN_CLR_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR3_IRQ_ENABLE_CLR_MMR3_IRQ_EN_CLR_MAX                  (0x0000000FU)

/* MMR4_IRQ_STATUS_RAW */

#define CSL_PCIE_RC_CORE_MMR4_IRQ_STATUS_RAW_RSVD150_MASK                         (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR4_IRQ_STATUS_RAW_RSVD150_SHIFT                        (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR4_IRQ_STATUS_RAW_RSVD150_MAX                          (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR4_IRQ_STATUS_RAW_MMR4_IRQ_STATUS_RAW_MASK             (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR4_IRQ_STATUS_RAW_MMR4_IRQ_STATUS_RAW_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR4_IRQ_STATUS_RAW_MMR4_IRQ_STATUS_RAW_MAX              (0x0000000FU)

/* MMR4_IRQ_STATUS */

#define CSL_PCIE_RC_CORE_MMR4_IRQ_STATUS_RSVD151_MASK                             (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR4_IRQ_STATUS_RSVD151_SHIFT                            (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR4_IRQ_STATUS_RSVD151_MAX                              (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR4_IRQ_STATUS_MMR4_IRQ_STATUS_MASK                     (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR4_IRQ_STATUS_MMR4_IRQ_STATUS_SHIFT                    (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR4_IRQ_STATUS_MMR4_IRQ_STATUS_MAX                      (0x0000000FU)

/* MMR4_IRQ_ENABLE_SET */

#define CSL_PCIE_RC_CORE_MMR4_IRQ_ENABLE_SET_RSVD152_MASK                         (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR4_IRQ_ENABLE_SET_RSVD152_SHIFT                        (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR4_IRQ_ENABLE_SET_RSVD152_MAX                          (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR4_IRQ_ENABLE_SET_MMR4_IRQ_EN_SET_MASK                 (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR4_IRQ_ENABLE_SET_MMR4_IRQ_EN_SET_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR4_IRQ_ENABLE_SET_MMR4_IRQ_EN_SET_MAX                  (0x0000000FU)

/* MMR4_IRQ_ENABLE_CLR */

#define CSL_PCIE_RC_CORE_MMR4_IRQ_ENABLE_CLR_RSVD153_MASK                         (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR4_IRQ_ENABLE_CLR_RSVD153_SHIFT                        (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR4_IRQ_ENABLE_CLR_RSVD153_MAX                          (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR4_IRQ_ENABLE_CLR_MMR4_IRQ_EN_CLR_MASK                 (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR4_IRQ_ENABLE_CLR_MMR4_IRQ_EN_CLR_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR4_IRQ_ENABLE_CLR_MMR4_IRQ_EN_CLR_MAX                  (0x0000000FU)

/* MMR5_IRQ_STATUS_RAW */

#define CSL_PCIE_RC_CORE_MMR5_IRQ_STATUS_RAW_RSVD160_MASK                         (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR5_IRQ_STATUS_RAW_RSVD160_SHIFT                        (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR5_IRQ_STATUS_RAW_RSVD160_MAX                          (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR5_IRQ_STATUS_RAW_MMR5_IRQ_STATUS_RAW_MASK             (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR5_IRQ_STATUS_RAW_MMR5_IRQ_STATUS_RAW_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR5_IRQ_STATUS_RAW_MMR5_IRQ_STATUS_RAW_MAX              (0x0000000FU)

/* MMR5_IRQ_STATUS */

#define CSL_PCIE_RC_CORE_MMR5_IRQ_STATUS_RSVD161_MASK                             (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR5_IRQ_STATUS_RSVD161_SHIFT                            (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR5_IRQ_STATUS_RSVD161_MAX                              (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR5_IRQ_STATUS_MMR5_IRQ_STATUS_MASK                     (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR5_IRQ_STATUS_MMR5_IRQ_STATUS_SHIFT                    (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR5_IRQ_STATUS_MMR5_IRQ_STATUS_MAX                      (0x0000000FU)

/* MMR5_IRQ_ENABLE_SET */

#define CSL_PCIE_RC_CORE_MMR5_IRQ_ENABLE_SET_RSVD162_MASK                         (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR5_IRQ_ENABLE_SET_RSVD162_SHIFT                        (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR5_IRQ_ENABLE_SET_RSVD162_MAX                          (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR5_IRQ_ENABLE_SET_MMR5_IRQ_EN_SET_MASK                 (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR5_IRQ_ENABLE_SET_MMR5_IRQ_EN_SET_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR5_IRQ_ENABLE_SET_MMR5_IRQ_EN_SET_MAX                  (0x0000000FU)

/* MMR5_IRQ_ENABLE_CLR */

#define CSL_PCIE_RC_CORE_MMR5_IRQ_ENABLE_CLR_RSVD163_MASK                         (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR5_IRQ_ENABLE_CLR_RSVD163_SHIFT                        (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR5_IRQ_ENABLE_CLR_RSVD163_MAX                          (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR5_IRQ_ENABLE_CLR_MMR5_IRQ_EN_CLR_MASK                 (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR5_IRQ_ENABLE_CLR_MMR5_IRQ_EN_CLR_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR5_IRQ_ENABLE_CLR_MMR5_IRQ_EN_CLR_MAX                  (0x0000000FU)

/* MMR6_IRQ_STATUS_RAW */

#define CSL_PCIE_RC_CORE_MMR6_IRQ_STATUS_RAW_RSVD170_MASK                         (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR6_IRQ_STATUS_RAW_RSVD170_SHIFT                        (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR6_IRQ_STATUS_RAW_RSVD170_MAX                          (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR6_IRQ_STATUS_RAW_MMR6_IRQ_STATUS_RAW_MASK             (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR6_IRQ_STATUS_RAW_MMR6_IRQ_STATUS_RAW_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR6_IRQ_STATUS_RAW_MMR6_IRQ_STATUS_RAW_MAX              (0x0000000FU)

/* MMR6_IRQ_STATUS */

#define CSL_PCIE_RC_CORE_MMR6_IRQ_STATUS_RSVD171_MASK                             (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR6_IRQ_STATUS_RSVD171_SHIFT                            (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR6_IRQ_STATUS_RSVD171_MAX                              (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR6_IRQ_STATUS_MMR6_IRQ_STATUS_MASK                     (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR6_IRQ_STATUS_MMR6_IRQ_STATUS_SHIFT                    (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR6_IRQ_STATUS_MMR6_IRQ_STATUS_MAX                      (0x0000000FU)

/* MMR6_IRQ_ENABLE_SET */

#define CSL_PCIE_RC_CORE_MMR6_IRQ_ENABLE_SET_RSVD172_MASK                         (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR6_IRQ_ENABLE_SET_RSVD172_SHIFT                        (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR6_IRQ_ENABLE_SET_RSVD172_MAX                          (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR6_IRQ_ENABLE_SET_MMR6_IRQ_EN_SET_MASK                 (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR6_IRQ_ENABLE_SET_MMR6_IRQ_EN_SET_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR6_IRQ_ENABLE_SET_MMR6_IRQ_EN_SET_MAX                  (0x0000000FU)

/* MMR6_IRQ_ENABLE_CLR */

#define CSL_PCIE_RC_CORE_MMR6_IRQ_ENABLE_CLR_RSVD173_MASK                         (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR6_IRQ_ENABLE_CLR_RSVD173_SHIFT                        (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR6_IRQ_ENABLE_CLR_RSVD173_MAX                          (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR6_IRQ_ENABLE_CLR_MMR6_IRQ_EN_CLR_MASK                 (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR6_IRQ_ENABLE_CLR_MMR6_IRQ_EN_CLR_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR6_IRQ_ENABLE_CLR_MMR6_IRQ_EN_CLR_MAX                  (0x0000000FU)

/* MMR7_IRQ_STATUS_RAW */

#define CSL_PCIE_RC_CORE_MMR7_IRQ_STATUS_RAW_RSVD180_MASK                         (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR7_IRQ_STATUS_RAW_RSVD180_SHIFT                        (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR7_IRQ_STATUS_RAW_RSVD180_MAX                          (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR7_IRQ_STATUS_RAW_MMR7_IRQ_STATUS_RAW_MASK             (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR7_IRQ_STATUS_RAW_MMR7_IRQ_STATUS_RAW_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR7_IRQ_STATUS_RAW_MMR7_IRQ_STATUS_RAW_MAX              (0x0000000FU)

/* MMR7_IRQ_STATUS */

#define CSL_PCIE_RC_CORE_MMR7_IRQ_STATUS_RSVD181_MASK                             (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR7_IRQ_STATUS_RSVD181_SHIFT                            (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR7_IRQ_STATUS_RSVD181_MAX                              (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR7_IRQ_STATUS_MMR7_IRQ_STATUS_MASK                     (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR7_IRQ_STATUS_MMR7_IRQ_STATUS_SHIFT                    (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR7_IRQ_STATUS_MMR7_IRQ_STATUS_MAX                      (0x0000000FU)

/* MMR7_IRQ_ENABLE_SET */

#define CSL_PCIE_RC_CORE_MMR7_IRQ_ENABLE_SET_RSVD182_MASK                         (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR7_IRQ_ENABLE_SET_RSVD182_SHIFT                        (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR7_IRQ_ENABLE_SET_RSVD182_MAX                          (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR7_IRQ_ENABLE_SET_MMR7_IRQ_EN_SET_MASK                 (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR7_IRQ_ENABLE_SET_MMR7_IRQ_EN_SET_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR7_IRQ_ENABLE_SET_MMR7_IRQ_EN_SET_MAX                  (0x0000000FU)

/* MMR7_IRQ_ENABLE_CLR */

#define CSL_PCIE_RC_CORE_MMR7_IRQ_ENABLE_CLR_RSVD183_MASK                         (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR7_IRQ_ENABLE_CLR_RSVD183_SHIFT                        (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR7_IRQ_ENABLE_CLR_RSVD183_MAX                          (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR7_IRQ_ENABLE_CLR_MMR7_IRQ_EN_CLR_MASK                 (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR7_IRQ_ENABLE_CLR_MMR7_IRQ_EN_CLR_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR7_IRQ_ENABLE_CLR_MMR7_IRQ_EN_CLR_MAX                  (0x0000000FU)

#else /* CSL_MODIFICATION */
/* MMRn_IRQ_STATUS_RAW */

#define CSL_PCIE_RC_CORE_MMR_IRQ_STATUS_RAW_RSVD100_MASK                          (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR_IRQ_STATUS_RAW_RSVD100_SHIFT                         (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR_IRQ_STATUS_RAW_RSVD100_MAX                           (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR_IRQ_STATUS_RAW_MMR_IRQ_STATUS_RAW_MASK               (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR_IRQ_STATUS_RAW_MMR_IRQ_STATUS_RAW_SHIFT              (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR_IRQ_STATUS_RAW_MMR_IRQ_STATUS_RAW_MAX                (0x0000000FU)

/* MMRn_IRQ_STATUS */

#define CSL_PCIE_RC_CORE_MMR_IRQ_STATUS_RSVD110_MASK                              (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR_IRQ_STATUS_RSVD110_SHIFT                             (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR_IRQ_STATUS_RSVD110_MAX                               (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR_IRQ_STATUS_MMR_IRQ_STATUS_MASK                       (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR_IRQ_STATUS_MMR_IRQ_STATUS_SHIFT                      (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR_IRQ_STATUS_MMR_IRQ_STATUS_MAX                        (0x0000000FU)

/* MMRn_IRQ_ENABLE_SET */

#define CSL_PCIE_RC_CORE_MMR_IRQ_ENABLE_SET_RSVD111_MASK                          (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR_IRQ_ENABLE_SET_RSVD111_SHIFT                         (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR_IRQ_ENABLE_SET_RSVD111_MAX                           (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR_IRQ_ENABLE_SET_MMR_IRQ_EN_SET_MASK                   (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR_IRQ_ENABLE_SET_MMR_IRQ_EN_SET_SHIFT                  (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR_IRQ_ENABLE_SET_MMR_IRQ_EN_SET_MAX                    (0x0000000FU)

/* MMRn_IRQ_ENABLE_CLR */

#define CSL_PCIE_RC_CORE_MMR_IRQ_ENABLE_CLR_RSVD112_MASK                          (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_MMR_IRQ_ENABLE_CLR_RSVD112_SHIFT                         (0x00000004U)
#define CSL_PCIE_RC_CORE_MMR_IRQ_ENABLE_CLR_RSVD112_MAX                           (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_MMR_IRQ_ENABLE_CLR_MMR_IRQ_EN_CLR_MASK                   (0x0000000FU)
#define CSL_PCIE_RC_CORE_MMR_IRQ_ENABLE_CLR_MMR_IRQ_EN_CLR_SHIFT                  (0x00000000U)
#define CSL_PCIE_RC_CORE_MMR_IRQ_ENABLE_CLR_MMR_IRQ_EN_CLR_MAX                    (0x0000000FU)

#endif /* CSL_MODIFICATION */

#ifdef CSL_MODIFICATION
/* LEGACY_A_IRQ_STATUS_RAW */

#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_STATUS_RAW_RSVD200_MASK                     (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_STATUS_RAW_RSVD200_SHIFT                    (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_STATUS_RAW_RSVD200_MAX                      (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_STATUS_RAW_INTA_RAW_MASK                    (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_STATUS_RAW_INTA_RAW_SHIFT                   (0x00000000U)
#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_STATUS_RAW_INTA_RAW_MAX                     (0x00000001U)

/* LEGACY_A_IRQ_STATUS */

#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_STATUS_RSVD210_MASK                         (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_STATUS_RSVD210_SHIFT                        (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_STATUS_RSVD210_MAX                          (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_STATUS_INTA_MASK                            (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_STATUS_INTA_SHIFT                           (0x00000000U)
#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_STATUS_INTA_MAX                             (0x00000001U)

/* LEGACY_A_IRQ_ENABLE_SET */

#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_ENABLE_SET_RSVD220_MASK                     (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_ENABLE_SET_RSVD220_SHIFT                    (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_ENABLE_SET_RSVD220_MAX                      (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_ENABLE_SET_INTA_EN_SET_MASK                 (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_ENABLE_SET_INTA_EN_SET_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_ENABLE_SET_INTA_EN_SET_MAX                  (0x00000001U)

/* LEGACY_A_IRQ_ENABLE_CLR */

#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_ENABLE_CLR_RSVD230_MASK                     (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_ENABLE_CLR_RSVD230_SHIFT                    (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_ENABLE_CLR_RSVD230_MAX                      (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_ENABLE_CLR_INTA_EN_CLR_MASK                 (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_ENABLE_CLR_INTA_EN_CLR_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_LEGACY_A_IRQ_ENABLE_CLR_INTA_EN_CLR_MAX                  (0x00000001U)

/* LEGACY_B_IRQ_STATUS_RAW */

#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_STATUS_RAW_RSVD300_MASK                     (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_STATUS_RAW_RSVD300_SHIFT                    (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_STATUS_RAW_RSVD300_MAX                      (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_STATUS_RAW_INTB_RAW_MASK                    (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_STATUS_RAW_INTB_RAW_SHIFT                   (0x00000000U)
#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_STATUS_RAW_INTB_RAW_MAX                     (0x00000001U)

/* LEGACY_B_IRQ_STATUS */

#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_STATUS_RSVD310_MASK                         (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_STATUS_RSVD310_SHIFT                        (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_STATUS_RSVD310_MAX                          (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_STATUS_INTA_MASK                            (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_STATUS_INTA_SHIFT                           (0x00000000U)
#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_STATUS_INTA_MAX                             (0x00000001U)

/* LEGACY_B_IRQ_ENABLE_SET */

#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_ENABLE_SET_RSVD320_MASK                     (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_ENABLE_SET_RSVD320_SHIFT                    (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_ENABLE_SET_RSVD320_MAX                      (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_ENABLE_SET_INTB_EN_SET_MASK                 (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_ENABLE_SET_INTB_EN_SET_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_ENABLE_SET_INTB_EN_SET_MAX                  (0x00000001U)

/* LEGACY_B_IRQ_ENABLE_CLR */

#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_ENABLE_CLR_RSVD330_MASK                     (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_ENABLE_CLR_RSVD330_SHIFT                    (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_ENABLE_CLR_RSVD330_MAX                      (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_ENABLE_CLR_INTB_EN_CLR_MASK                 (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_ENABLE_CLR_INTB_EN_CLR_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_LEGACY_B_IRQ_ENABLE_CLR_INTB_EN_CLR_MAX                  (0x00000001U)

/* LEGACY_C_IRQ_STATUS_RAW */

#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_STATUS_RAW_RSVD400_MASK                     (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_STATUS_RAW_RSVD400_SHIFT                    (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_STATUS_RAW_RSVD400_MAX                      (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_STATUS_RAW_INTC_RAW_MASK                    (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_STATUS_RAW_INTC_RAW_SHIFT                   (0x00000000U)
#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_STATUS_RAW_INTC_RAW_MAX                     (0x00000001U)

/* LEGACY_C_IRQ_STATUS */

#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_STATUS_RSVD410_MASK                         (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_STATUS_RSVD410_SHIFT                        (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_STATUS_RSVD410_MAX                          (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_STATUS_INTA_MASK                            (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_STATUS_INTA_SHIFT                           (0x00000000U)
#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_STATUS_INTA_MAX                             (0x00000001U)

/* LEGACY_C_IRQ_ENABLE_SET */

#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_ENABLE_SET_RSVD420_MASK                     (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_ENABLE_SET_RSVD420_SHIFT                    (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_ENABLE_SET_RSVD420_MAX                      (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_ENABLE_SET_INTC_EN_SET_MASK                 (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_ENABLE_SET_INTC_EN_SET_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_ENABLE_SET_INTC_EN_SET_MAX                  (0x00000001U)

/* LEGACY_C_IRQ_ENABLE_CLR */

#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_ENABLE_CLR_RSVD430_MASK                     (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_ENABLE_CLR_RSVD430_SHIFT                    (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_ENABLE_CLR_RSVD430_MAX                      (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_ENABLE_CLR_INTC_EN_CLR_MASK                 (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_ENABLE_CLR_INTC_EN_CLR_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_LEGACY_C_IRQ_ENABLE_CLR_INTC_EN_CLR_MAX                  (0x00000001U)

/* LEGACY_D_IRQ_STATUS_RAW */

#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_STATUS_RAW_RSVD500_MASK                     (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_STATUS_RAW_RSVD500_SHIFT                    (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_STATUS_RAW_RSVD500_MAX                      (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_STATUS_RAW_INTD_RAW_MASK                    (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_STATUS_RAW_INTD_RAW_SHIFT                   (0x00000000U)
#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_STATUS_RAW_INTD_RAW_MAX                     (0x00000001U)

/* LEGACY_D_IRQ_STATUS */

#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_STATUS_RSVD510_MASK                         (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_STATUS_RSVD510_SHIFT                        (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_STATUS_RSVD510_MAX                          (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_STATUS_INTA_MASK                            (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_STATUS_INTA_SHIFT                           (0x00000000U)
#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_STATUS_INTA_MAX                             (0x00000001U)

/* LEGACY_D_IRQ_ENABLE_SET */

#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_ENABLE_SET_RSVD520_MASK                     (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_ENABLE_SET_RSVD520_SHIFT                    (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_ENABLE_SET_RSVD520_MAX                      (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_ENABLE_SET_INTD_EN_SET_MASK                 (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_ENABLE_SET_INTD_EN_SET_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_ENABLE_SET_INTD_EN_SET_MAX                  (0x00000001U)

/* LEGACY_D_IRQ_ENABLE_CLR */

#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_ENABLE_CLR_RSVD530_MASK                     (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_ENABLE_CLR_RSVD530_SHIFT                    (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_ENABLE_CLR_RSVD530_MAX                      (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_ENABLE_CLR_INTD_EN_CLR_MASK                 (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_ENABLE_CLR_INTD_EN_CLR_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_LEGACY_D_IRQ_ENABLE_CLR_INTD_EN_CLR_MAX                  (0x00000001U)

#else /* CSL_MODIFICATION */

/* LEGACY_n_IRQ_STATUS_RAW */

#define CSL_PCIE_RC_CORE_LEGACY_IRQ_STATUS_RAW_RSVD200_MASK                       (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_STATUS_RAW_RSVD200_SHIFT                      (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_STATUS_RAW_RSVD200_MAX                        (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_LEGACY_IRQ_STATUS_RAW_INT_RAW_MASK                       (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_STATUS_RAW_INT_RAW_SHIFT                      (0x00000000U)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_STATUS_RAW_INT_RAW_MAX                        (0x00000001U)

/* LEGACY_n_IRQ_STATUS */

#define CSL_PCIE_RC_CORE_LEGACY_IRQ_STATUS_RSVD210_MASK                           (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_STATUS_RSVD210_SHIFT                          (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_STATUS_RSVD210_MAX                            (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_LEGACY_IRQ_STATUS_INT_MASK                               (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_STATUS_INT_SHIFT                              (0x00000000U)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_STATUS_INT_MAX                                (0x00000001U)

/* LEGACY_n_IRQ_ENABLE_SET */

#define CSL_PCIE_RC_CORE_LEGACY_IRQ_ENABLE_SET_RSVD220_MASK                       (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_ENABLE_SET_RSVD220_SHIFT                      (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_ENABLE_SET_RSVD220_MAX                        (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_LEGACY_IRQ_ENABLE_SET_INT_EN_SET_MASK                    (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_ENABLE_SET_INT_EN_SET_SHIFT                   (0x00000000U)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_ENABLE_SET_INT_EN_SET_MAX                     (0x00000001U)

/* LEGACY_n_IRQ_ENABLE_CLR */

#define CSL_PCIE_RC_CORE_LEGACY_IRQ_ENABLE_CLR_RSVD230_MASK                       (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_ENABLE_CLR_RSVD230_SHIFT                      (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_ENABLE_CLR_RSVD230_MAX                        (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_LEGACY_IRQ_ENABLE_CLR_INT_EN_CLR_MASK                    (0x00000001U)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_ENABLE_CLR_INT_EN_CLR_SHIFT                   (0x00000000U)
#define CSL_PCIE_RC_CORE_LEGACY_IRQ_ENABLE_CLR_INT_EN_CLR_MAX                     (0x00000001U)

#endif /* CSL_MODIFICATION */

/* ERR_IRQ_STATUS_RAW */

#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_RAW_RSVD600_MASK                          (0xFFFFFFE0U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_RAW_RSVD600_SHIFT                         (0x00000005U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_RAW_RSVD600_MAX                           (0x07FFFFFFU)

#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_RAW_ERR_AER_RAW_MASK                      (0x00000010U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_RAW_ERR_AER_RAW_SHIFT                     (0x00000004U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_RAW_ERR_AER_RAW_MAX                       (0x00000001U)

#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_RAW_ERR_CORR_RAW_MASK                     (0x00000008U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_RAW_ERR_CORR_RAW_SHIFT                    (0x00000003U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_RAW_ERR_CORR_RAW_MAX                      (0x00000001U)

#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_RAW_ERR_NONFATAL_RAW_MASK                 (0x00000004U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_RAW_ERR_NONFATAL_RAW_SHIFT                (0x00000002U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_RAW_ERR_NONFATAL_RAW_MAX                  (0x00000001U)

#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_RAW_ERR_FATAL_RAW_MASK                    (0x00000002U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_RAW_ERR_FATAL_RAW_SHIFT                   (0x00000001U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_RAW_ERR_FATAL_RAW_MAX                     (0x00000001U)

#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_RAW_ERR_SYS_RAW_MASK                      (0x00000001U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_RAW_ERR_SYS_RAW_SHIFT                     (0x00000000U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_RAW_ERR_SYS_RAW_MAX                       (0x00000001U)

/* ERR_IRQ_STATUS */

#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_RSVD610_MASK                              (0xFFFFFFE0U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_RSVD610_SHIFT                             (0x00000005U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_RSVD610_MAX                               (0x07FFFFFFU)

#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_ERR_AER_MASK                              (0x00000010U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_ERR_AER_SHIFT                             (0x00000004U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_ERR_AER_MAX                               (0x00000001U)

#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_ERR_CORR_MASK                             (0x00000008U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_ERR_CORR_SHIFT                            (0x00000003U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_ERR_CORR_MAX                              (0x00000001U)

#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_ERR_NONFATAL_MASK                         (0x00000004U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_ERR_NONFATAL_SHIFT                        (0x00000002U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_ERR_NONFATAL_MAX                          (0x00000001U)

#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_ERR_FATAL_MASK                            (0x00000002U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_ERR_FATAL_SHIFT                           (0x00000001U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_ERR_FATAL_MAX                             (0x00000001U)

#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_ERR_SYS_MASK                              (0x00000001U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_ERR_SYS_SHIFT                             (0x00000000U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_STATUS_ERR_SYS_MAX                               (0x00000001U)

/* ERR_IRQ_ENABLE_SET */

#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_SET_RSVD620_MASK                          (0xFFFFFFE0U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_SET_RSVD620_SHIFT                         (0x00000005U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_SET_RSVD620_MAX                           (0x07FFFFFFU)

#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_SET_ERR_AER_EN_SET_MASK                   (0x00000010U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_SET_ERR_AER_EN_SET_SHIFT                  (0x00000004U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_SET_ERR_AER_EN_SET_MAX                    (0x00000001U)

#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_SET_ERR_CORR_EN_SET_MASK                  (0x00000008U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_SET_ERR_CORR_EN_SET_SHIFT                 (0x00000003U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_SET_ERR_CORR_EN_SET_MAX                   (0x00000001U)

#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_SET_ERR_NONFATAL_EN_SET_MASK              (0x00000004U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_SET_ERR_NONFATAL_EN_SET_SHIFT             (0x00000002U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_SET_ERR_NONFATAL_EN_SET_MAX               (0x00000001U)

#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_SET_ERR_FATAL_EN_SET_MASK                 (0x00000002U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_SET_ERR_FATAL_EN_SET_SHIFT                (0x00000001U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_SET_ERR_FATAL_EN_SET_MAX                  (0x00000001U)

#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_SET_ERR_SYS_EN_SET_MASK                   (0x00000001U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_SET_ERR_SYS_EN_SET_SHIFT                  (0x00000000U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_SET_ERR_SYS_EN_SET_MAX                    (0x00000001U)

/* ERR_IRQ_ENABLE_CLR */

#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_CLR_RSVD630_MASK                          (0xFFFFFFE0U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_CLR_RSVD630_SHIFT                         (0x00000005U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_CLR_RSVD630_MAX                           (0x07FFFFFFU)

#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_CLR_ERR_AER_EN_CLR_MASK                   (0x00000010U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_CLR_ERR_AER_EN_CLR_SHIFT                  (0x00000004U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_CLR_ERR_AER_EN_CLR_MAX                    (0x00000001U)

#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_CLR_ERR_CORR_EN_CLR_MASK                  (0x00000008U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_CLR_ERR_CORR_EN_CLR_SHIFT                 (0x00000003U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_CLR_ERR_CORR_EN_CLR_MAX                   (0x00000001U)

#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_CLR_ERR_NONFATAL_EN_CLR_MASK              (0x00000004U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_CLR_ERR_NONFATAL_EN_CLR_SHIFT             (0x00000002U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_CLR_ERR_NONFATAL_EN_CLR_MAX               (0x00000001U)

#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_CLR_ERR_FATAL_EN_CLR_MASK                 (0x00000002U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_CLR_ERR_FATAL_EN_CLR_SHIFT                (0x00000001U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_CLR_ERR_FATAL_EN_CLR_MAX                  (0x00000001U)

#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_CLR_ERR_SYS_EN_CLR_MASK                   (0x00000001U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_CLR_ERR_SYS_EN_CLR_SHIFT                  (0x00000000U)
#define CSL_PCIE_RC_CORE_ERR_IRQ_ENABLE_CLR_ERR_SYS_EN_CLR_MAX                    (0x00000001U)

/* PMRST_IRQ_STATUS_RAW */

#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_RAW_RSVD700_MASK                        (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_RAW_RSVD700_SHIFT                       (0x00000004U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_RAW_RSVD700_MAX                         (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_RAW_LNK_RST_REQ_RAW_MASK                (0x00000008U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_RAW_LNK_RST_REQ_RAW_SHIFT               (0x00000003U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_RAW_LNK_RST_REQ_RAW_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_RAW_PM_PME_RAW_MASK                     (0x00000004U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_RAW_PM_PME_RAW_SHIFT                    (0x00000002U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_RAW_PM_PME_RAW_MAX                      (0x00000001U)

#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_RAW_PM_TO_ACK_RAW_MASK                  (0x00000002U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_RAW_PM_TO_ACK_RAW_SHIFT                 (0x00000001U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_RAW_PM_TO_ACK_RAW_MAX                   (0x00000001U)

#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_RAW_PM_TURNOFF_RAW_MASK                 (0x00000001U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_RAW_PM_TURNOFF_RAW_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_RAW_PM_TURNOFF_RAW_MAX                  (0x00000001U)

/* PMRST_IRQ_STATUS */

#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_RSVD710_MASK                            (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_RSVD710_SHIFT                           (0x00000004U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_RSVD710_MAX                             (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_LNK_RST_REQ_MASK                        (0x00000008U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_LNK_RST_REQ_SHIFT                       (0x00000003U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_LNK_RST_REQ_MAX                         (0x00000001U)

#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_PM_PME_MASK                             (0x00000004U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_PM_PME_SHIFT                            (0x00000002U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_PM_PME_MAX                              (0x00000001U)

#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_PM_TO_ACK_MASK                          (0x00000002U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_PM_TO_ACK_SHIFT                         (0x00000001U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_PM_TO_ACK_MAX                           (0x00000001U)

#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_PM_TURNOFF_MASK                         (0x00000001U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_PM_TURNOFF_SHIFT                        (0x00000000U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_STATUS_PM_TURNOFF_MAX                          (0x00000001U)

/* PMRST_IRQ_ENABLE_SET */

#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_SET_RSVD720_MASK                        (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_SET_RSVD720_SHIFT                       (0x00000004U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_SET_RSVD720_MAX                         (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_SET_LNK_RST_REQ_EN_SET_MASK             (0x00000008U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_SET_LNK_RST_REQ_EN_SET_SHIFT            (0x00000003U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_SET_LNK_RST_REQ_EN_SET_MAX              (0x00000001U)

#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_SET_PM_PME_EN_SET_MASK                  (0x00000004U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_SET_PM_PME_EN_SET_SHIFT                 (0x00000002U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_SET_PM_PME_EN_SET_MAX                   (0x00000001U)

#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_SET_PM_TO_ACK_EN_SET_MASK               (0x00000002U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_SET_PM_TO_ACK_EN_SET_SHIFT              (0x00000001U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_SET_PM_TO_ACK_EN_SET_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_SET_PM_TURNOFF_EN_SET_MASK              (0x00000001U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_SET_PM_TURNOFF_EN_SET_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_SET_PM_TURNOFF_EN_SET_MAX               (0x00000001U)

/* PMRST_IRQ_ENABLE_CLR */

#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_CLR_RSVD730_MASK                        (0xFFFFFFF0U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_CLR_RSVD730_SHIFT                       (0x00000004U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_CLR_RSVD730_MAX                         (0x0FFFFFFFU)

#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_CLR_LNK_RST_REQ_EN_CLR_MASK             (0x00000008U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_CLR_LNK_RST_REQ_EN_CLR_SHIFT            (0x00000003U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_CLR_LNK_RST_REQ_EN_CLR_MAX              (0x00000001U)

#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_CLR_PM_PME_EN_CLR_MASK                  (0x00000004U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_CLR_PM_PME_EN_CLR_SHIFT                 (0x00000002U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_CLR_PM_PME_EN_CLR_MAX                   (0x00000001U)

#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_CLR_PM_TO_ACK_EN_CLR_MASK               (0x00000002U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_CLR_PM_TO_ACK_EN_CLR_SHIFT              (0x00000001U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_CLR_PM_TO_ACK_EN_CLR_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_CLR_PM_TURNOFF_EN_CLR_MASK              (0x00000001U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_CLR_PM_TURNOFF_EN_CLR_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_PMRST_IRQ_ENABLE_CLR_PM_TURNOFF_EN_CLR_MAX               (0x00000001U)

/* PTM_IRQ_STATUS_RAW */

#define CSL_PCIE_RC_CORE_PTM_IRQ_STATUS_RAW_RSVD800_MASK                          (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_PTM_IRQ_STATUS_RAW_RSVD800_SHIFT                         (0x00000001U)
#define CSL_PCIE_RC_CORE_PTM_IRQ_STATUS_RAW_RSVD800_MAX                           (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_PTM_IRQ_STATUS_RAW_PTM_CLK_UPDATED_RAW_MASK              (0x00000001U)
#define CSL_PCIE_RC_CORE_PTM_IRQ_STATUS_RAW_PTM_CLK_UPDATED_RAW_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_PTM_IRQ_STATUS_RAW_PTM_CLK_UPDATED_RAW_MAX               (0x00000001U)

/* PTM_IRQ_STATUS */

#define CSL_PCIE_RC_CORE_PTM_IRQ_STATUS_RSVD810_MASK                              (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_PTM_IRQ_STATUS_RSVD810_SHIFT                             (0x00000001U)
#define CSL_PCIE_RC_CORE_PTM_IRQ_STATUS_RSVD810_MAX                               (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_PTM_IRQ_STATUS_PTM_CLK_UPDATED_MASK                      (0x00000001U)
#define CSL_PCIE_RC_CORE_PTM_IRQ_STATUS_PTM_CLK_UPDATED_SHIFT                     (0x00000000U)
#define CSL_PCIE_RC_CORE_PTM_IRQ_STATUS_PTM_CLK_UPDATED_MAX                       (0x00000001U)

/* PTM_IRQ_ENABLE_SET */

#define CSL_PCIE_RC_CORE_PTM_IRQ_ENABLE_SET_RSVD820_MASK                          (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_PTM_IRQ_ENABLE_SET_RSVD820_SHIFT                         (0x00000001U)
#define CSL_PCIE_RC_CORE_PTM_IRQ_ENABLE_SET_RSVD820_MAX                           (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_PTM_IRQ_ENABLE_SET_PTM_CLK_UPDATED_EN_SET_MASK           (0x00000001U)
#define CSL_PCIE_RC_CORE_PTM_IRQ_ENABLE_SET_PTM_CLK_UPDATED_EN_SET_SHIFT          (0x00000000U)
#define CSL_PCIE_RC_CORE_PTM_IRQ_ENABLE_SET_PTM_CLK_UPDATED_EN_SET_MAX            (0x00000001U)

/* PTM_IRQ_ENABLE_CLR */

#define CSL_PCIE_RC_CORE_PTM_IRQ_ENABLE_CLR_RSVD830_MASK                          (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_PTM_IRQ_ENABLE_CLR_RSVD830_SHIFT                         (0x00000001U)
#define CSL_PCIE_RC_CORE_PTM_IRQ_ENABLE_CLR_RSVD830_MAX                           (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_PTM_IRQ_ENABLE_CLR_PTM_CLK_UPDATED_EN_CLR_MASK           (0x00000001U)
#define CSL_PCIE_RC_CORE_PTM_IRQ_ENABLE_CLR_PTM_CLK_UPDATED_EN_CLR_SHIFT          (0x00000000U)
#define CSL_PCIE_RC_CORE_PTM_IRQ_ENABLE_CLR_PTM_CLK_UPDATED_EN_CLR_MAX            (0x00000001U)

/* TYPE1_DEV_ID_VEND_ID_REG */

#define CSL_PCIE_RC_CORE_TYPE1_DEV_ID_VEND_ID_REG_VENDOR_ID_MASK                  (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_TYPE1_DEV_ID_VEND_ID_REG_VENDOR_ID_SHIFT                 (0x00000000U)
#define CSL_PCIE_RC_CORE_TYPE1_DEV_ID_VEND_ID_REG_VENDOR_ID_MAX                   (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_TYPE1_DEV_ID_VEND_ID_REG_DEVICE_ID_MASK                  (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_TYPE1_DEV_ID_VEND_ID_REG_DEVICE_ID_SHIFT                 (0x00000010U)
#define CSL_PCIE_RC_CORE_TYPE1_DEV_ID_VEND_ID_REG_DEVICE_ID_MAX                   (0x0000FFFFU)

/* TYPE1_STATUS_COMMAND_REG */

#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_IO_EN_MASK                      (0x00000001U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_IO_EN_SHIFT                     (0x00000000U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_IO_EN_MAX                       (0x00000001U)

#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_MSE_MASK                        (0x00000002U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_MSE_SHIFT                       (0x00000001U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_MSE_MAX                         (0x00000001U)

#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_BME_MASK                        (0x00000004U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_BME_SHIFT                       (0x00000002U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_BME_MAX                         (0x00000001U)

#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_SCO_MASK                        (0x00000008U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_SCO_SHIFT                       (0x00000003U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_SCO_MAX                         (0x00000001U)

#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_MWI_EN_MASK                     (0x00000010U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_MWI_EN_SHIFT                    (0x00000004U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_MWI_EN_MAX                      (0x00000001U)

#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_VGAPS_MASK                      (0x00000020U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_VGAPS_SHIFT                     (0x00000005U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_VGAPS_MAX                       (0x00000001U)

#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_PERREN_MASK                     (0x00000040U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_PERREN_SHIFT                    (0x00000006U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_PERREN_MAX                      (0x00000001U)

#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_IDSEL_MASK                      (0x00000080U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_IDSEL_SHIFT                     (0x00000007U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_IDSEL_MAX                       (0x00000001U)

#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_SERREN_MASK                     (0x00000100U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_SERREN_SHIFT                    (0x00000008U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_SERREN_MAX                      (0x00000001U)

#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_RSVDP_9_MASK                    (0x00000200U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_RSVDP_9_SHIFT                   (0x00000009U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_RSVDP_9_MAX                     (0x00000001U)

#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_INT_EN_MASK                     (0x00000400U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_INT_EN_SHIFT                    (0x0000000AU)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_INT_EN_MAX                      (0x00000001U)

#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_RESERV_MASK                     (0x0000F800U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_RESERV_SHIFT                    (0x0000000BU)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_RESERV_MAX                      (0x0000001FU)

#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_RSVDP_17_MASK                   (0x00060000U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_RSVDP_17_SHIFT                  (0x00000011U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_RSVDP_17_MAX                    (0x00000003U)

#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_INT_STATUS_MASK                 (0x00080000U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_INT_STATUS_SHIFT                (0x00000013U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_INT_STATUS_MAX                  (0x00000001U)

#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_CAP_LIST_MASK                   (0x00100000U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_CAP_LIST_SHIFT                  (0x00000014U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_CAP_LIST_MAX                    (0x00000001U)

#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_FAST_66MHZ_CAP_MASK             (0x00200000U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_FAST_66MHZ_CAP_SHIFT            (0x00000015U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_FAST_66MHZ_CAP_MAX              (0x00000001U)

#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_RSVDP_22_MASK                   (0x00400000U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_RSVDP_22_SHIFT                  (0x00000016U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_RSVDP_22_MAX                    (0x00000001U)

#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_FAST_B2B_CAP_MASK               (0x00800000U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_FAST_B2B_CAP_SHIFT              (0x00000017U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_FAST_B2B_CAP_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_MASTER_DPE_MASK                 (0x01000000U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_MASTER_DPE_SHIFT                (0x00000018U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_MASTER_DPE_MAX                  (0x00000001U)

#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_DEV_SEL_TIMING_MASK             (0x06000000U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_DEV_SEL_TIMING_SHIFT            (0x00000019U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_DEV_SEL_TIMING_MAX              (0x00000003U)

#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_SIGNALED_TARGET_ABORT_MASK      (0x08000000U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_SIGNALED_TARGET_ABORT_SHIFT     (0x0000001BU)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_SIGNALED_TARGET_ABORT_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_RCVD_TARGET_ABORT_MASK          (0x10000000U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_RCVD_TARGET_ABORT_SHIFT         (0x0000001CU)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_RCVD_TARGET_ABORT_MAX           (0x00000001U)

#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_RCVD_MASTER_ABORT_MASK          (0x20000000U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_RCVD_MASTER_ABORT_SHIFT         (0x0000001DU)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_RCVD_MASTER_ABORT_MAX           (0x00000001U)

#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_SIGNALED_SYS_ERROR_MASK         (0x40000000U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_SIGNALED_SYS_ERROR_SHIFT        (0x0000001EU)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_SIGNALED_SYS_ERROR_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_DETECTED_PARITY_ERROR_MASK      (0x80000000U)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_DETECTED_PARITY_ERROR_SHIFT     (0x0000001FU)
#define CSL_PCIE_RC_CORE_TYPE1_STATUS_COMMAND_REG_DETECTED_PARITY_ERROR_MAX       (0x00000001U)

/* TYPE1_CLASS_CODE_REV_ID_REG */

#define CSL_PCIE_RC_CORE_TYPE1_CLASS_CODE_REV_ID_REG_REVISION_ID_MASK             (0x000000FFU)
#define CSL_PCIE_RC_CORE_TYPE1_CLASS_CODE_REV_ID_REG_REVISION_ID_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_TYPE1_CLASS_CODE_REV_ID_REG_REVISION_ID_MAX              (0x000000FFU)

#define CSL_PCIE_RC_CORE_TYPE1_CLASS_CODE_REV_ID_REG_PROGRAM_INTERFACE_MASK       (0x0000FF00U)
#define CSL_PCIE_RC_CORE_TYPE1_CLASS_CODE_REV_ID_REG_PROGRAM_INTERFACE_SHIFT      (0x00000008U)
#define CSL_PCIE_RC_CORE_TYPE1_CLASS_CODE_REV_ID_REG_PROGRAM_INTERFACE_MAX        (0x000000FFU)

#define CSL_PCIE_RC_CORE_TYPE1_CLASS_CODE_REV_ID_REG_SUBCLASS_CODE_MASK           (0x00FF0000U)
#define CSL_PCIE_RC_CORE_TYPE1_CLASS_CODE_REV_ID_REG_SUBCLASS_CODE_SHIFT          (0x00000010U)
#define CSL_PCIE_RC_CORE_TYPE1_CLASS_CODE_REV_ID_REG_SUBCLASS_CODE_MAX            (0x000000FFU)

#define CSL_PCIE_RC_CORE_TYPE1_CLASS_CODE_REV_ID_REG_BASE_CLASS_CODE_MASK         (0xFF000000U)
#define CSL_PCIE_RC_CORE_TYPE1_CLASS_CODE_REV_ID_REG_BASE_CLASS_CODE_SHIFT        (0x00000018U)
#define CSL_PCIE_RC_CORE_TYPE1_CLASS_CODE_REV_ID_REG_BASE_CLASS_CODE_MAX          (0x000000FFU)

/* TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG */

#define CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_CACHE_LINE_SIZE_MASK (0x000000FFU)
#define CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_CACHE_LINE_SIZE_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_CACHE_LINE_SIZE_MAX (0x000000FFU)

#define CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_LATENCY_MASTER_TIMER_MASK (0x0000FF00U)
#define CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_LATENCY_MASTER_TIMER_SHIFT (0x00000008U)
#define CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_LATENCY_MASTER_TIMER_MAX (0x000000FFU)

#define CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_HEADER_TYPE_MASK (0x007F0000U)
#define CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_HEADER_TYPE_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_HEADER_TYPE_MAX (0x0000007FU)

#define CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_MULTI_FUNC_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_MULTI_FUNC_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_MULTI_FUNC_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_BIST_MASK    (0xFF000000U)
#define CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_BIST_SHIFT   (0x00000018U)
#define CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_BIST_MAX     (0x000000FFU)

/* SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG */

#define CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_PRIM_BUS_MASK  (0x000000FFU)
#define CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_PRIM_BUS_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_PRIM_BUS_MAX   (0x000000FFU)

#define CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_SEC_BUS_MASK   (0x0000FF00U)
#define CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_SEC_BUS_SHIFT  (0x00000008U)
#define CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_SEC_BUS_MAX    (0x000000FFU)

#define CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_SUB_BUS_MASK   (0x00FF0000U)
#define CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_SUB_BUS_SHIFT  (0x00000010U)
#define CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_SUB_BUS_MAX    (0x000000FFU)

#define CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_SEC_LAT_TIMER_MASK (0xFF000000U)
#define CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_SEC_LAT_TIMER_SHIFT (0x00000018U)
#define CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_SEC_LAT_TIMER_MAX (0x000000FFU)

/* SEC_STAT_IO_LIMIT_IO_BASE_REG */

#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_DECODE_MASK             (0x00000001U)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_DECODE_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_DECODE_MAX              (0x00000001U)

#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_RESERV_MASK             (0x0000000EU)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_RESERV_SHIFT            (0x00000001U)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_RESERV_MAX              (0x00000007U)

#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_BASE_MASK               (0x000000F0U)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_BASE_SHIFT              (0x00000004U)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_BASE_MAX                (0x0000000FU)

#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_DECODE_BIT8_MASK        (0x00000100U)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_DECODE_BIT8_SHIFT       (0x00000008U)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_DECODE_BIT8_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_RESERV1_MASK            (0x00000E00U)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_RESERV1_SHIFT           (0x00000009U)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_RESERV1_MAX             (0x00000007U)

#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_LIMIT_MASK              (0x0000F000U)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_LIMIT_SHIFT             (0x0000000CU)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_LIMIT_MAX               (0x0000000FU)

#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_RESERV_MASK       (0x007F0000U)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_RESERV_SHIFT      (0x00000010U)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_RESERV_MAX        (0x0000007FU)

#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_RSVDP_23_MASK              (0x00800000U)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_RSVDP_23_SHIFT             (0x00000017U)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_RSVDP_23_MAX               (0x00000001U)

#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_MDPE_MASK         (0x01000000U)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_MDPE_SHIFT        (0x00000018U)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_MDPE_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_RSVDP_25_MASK              (0x06000000U)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_RSVDP_25_SHIFT             (0x00000019U)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_RSVDP_25_MAX               (0x00000003U)

#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_SIG_TRGT_ABRT_MASK (0x08000000U)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_SIG_TRGT_ABRT_SHIFT (0x0000001BU)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_SIG_TRGT_ABRT_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_RCVD_TRGT_ABRT_MASK (0x10000000U)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_RCVD_TRGT_ABRT_SHIFT (0x0000001CU)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_RCVD_TRGT_ABRT_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_RCVD_MSTR_ABRT_MASK (0x20000000U)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_RCVD_MSTR_ABRT_SHIFT (0x0000001DU)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_RCVD_MSTR_ABRT_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_RCVD_SYS_ERR_MASK (0x40000000U)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_RCVD_SYS_ERR_SHIFT (0x0000001EU)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_RCVD_SYS_ERR_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_DPE_MASK          (0x80000000U)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_DPE_SHIFT         (0x0000001FU)
#define CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_DPE_MAX           (0x00000001U)

/* MEM_LIMIT_MEM_BASE_REG */

#define CSL_PCIE_RC_CORE_MEM_LIMIT_MEM_BASE_REG_MEM_BASE_RESERV_MASK              (0x0000000FU)
#define CSL_PCIE_RC_CORE_MEM_LIMIT_MEM_BASE_REG_MEM_BASE_RESERV_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_MEM_LIMIT_MEM_BASE_REG_MEM_BASE_RESERV_MAX               (0x0000000FU)

#define CSL_PCIE_RC_CORE_MEM_LIMIT_MEM_BASE_REG_MEM_BASE_MASK                     (0x0000FFF0U)
#define CSL_PCIE_RC_CORE_MEM_LIMIT_MEM_BASE_REG_MEM_BASE_SHIFT                    (0x00000004U)
#define CSL_PCIE_RC_CORE_MEM_LIMIT_MEM_BASE_REG_MEM_BASE_MAX                      (0x00000FFFU)

#define CSL_PCIE_RC_CORE_MEM_LIMIT_MEM_BASE_REG_MEM_LIMIT_RESERV_MASK             (0x000F0000U)
#define CSL_PCIE_RC_CORE_MEM_LIMIT_MEM_BASE_REG_MEM_LIMIT_RESERV_SHIFT            (0x00000010U)
#define CSL_PCIE_RC_CORE_MEM_LIMIT_MEM_BASE_REG_MEM_LIMIT_RESERV_MAX              (0x0000000FU)

#define CSL_PCIE_RC_CORE_MEM_LIMIT_MEM_BASE_REG_MEM_LIMIT_MASK                    (0xFFF00000U)
#define CSL_PCIE_RC_CORE_MEM_LIMIT_MEM_BASE_REG_MEM_LIMIT_SHIFT                   (0x00000014U)
#define CSL_PCIE_RC_CORE_MEM_LIMIT_MEM_BASE_REG_MEM_LIMIT_MAX                     (0x00000FFFU)

/* PREF_MEM_LIMIT_PREF_MEM_BASE_REG */

#define CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_DECODE_MASK    (0x00000001U)
#define CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_DECODE_SHIFT   (0x00000000U)
#define CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_DECODE_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_RESERV_MASK        (0x0000000EU)
#define CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_RESERV_SHIFT       (0x00000001U)
#define CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_RESERV_MAX         (0x00000007U)

#define CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_BASE_MASK      (0x0000FFF0U)
#define CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_BASE_SHIFT     (0x00000004U)
#define CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_BASE_MAX       (0x00000FFFU)

#define CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_LIMIT_DECODE_MASK (0x00010000U)
#define CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_LIMIT_DECODE_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_LIMIT_DECODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_RESERV1_MASK       (0x000E0000U)
#define CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_RESERV1_SHIFT      (0x00000011U)
#define CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_RESERV1_MAX        (0x00000007U)

#define CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_LIMIT_MASK     (0xFFF00000U)
#define CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_LIMIT_SHIFT    (0x00000014U)
#define CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_LIMIT_MAX      (0x00000FFFU)

/* PREF_BASE_UPPER_REG */

#define CSL_PCIE_RC_CORE_PREF_BASE_UPPER_REG_PREF_MEM_BASE_UPPER_MASK             (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_PREF_BASE_UPPER_REG_PREF_MEM_BASE_UPPER_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_PREF_BASE_UPPER_REG_PREF_MEM_BASE_UPPER_MAX              (0xFFFFFFFFU)

/* PREF_LIMIT_UPPER_REG */

#define CSL_PCIE_RC_CORE_PREF_LIMIT_UPPER_REG_PREF_MEM_LIMIT_UPPER_MASK           (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_PREF_LIMIT_UPPER_REG_PREF_MEM_LIMIT_UPPER_SHIFT          (0x00000000U)
#define CSL_PCIE_RC_CORE_PREF_LIMIT_UPPER_REG_PREF_MEM_LIMIT_UPPER_MAX            (0xFFFFFFFFU)

/* IO_LIMIT_UPPER_IO_BASE_UPPER_REG */

#define CSL_PCIE_RC_CORE_IO_LIMIT_UPPER_IO_BASE_UPPER_REG_IO_BASE_UPPER_MASK      (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IO_LIMIT_UPPER_IO_BASE_UPPER_REG_IO_BASE_UPPER_SHIFT     (0x00000000U)
#define CSL_PCIE_RC_CORE_IO_LIMIT_UPPER_IO_BASE_UPPER_REG_IO_BASE_UPPER_MAX       (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IO_LIMIT_UPPER_IO_BASE_UPPER_REG_IO_LIMIT_UPPER_MASK     (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IO_LIMIT_UPPER_IO_BASE_UPPER_REG_IO_LIMIT_UPPER_SHIFT    (0x00000010U)
#define CSL_PCIE_RC_CORE_IO_LIMIT_UPPER_IO_BASE_UPPER_REG_IO_LIMIT_UPPER_MAX      (0x0000FFFFU)

/* TYPE1_CAP_PTR_REG */

#define CSL_PCIE_RC_CORE_TYPE1_CAP_PTR_REG_CAP_POINTER_MASK                       (0x000000FFU)
#define CSL_PCIE_RC_CORE_TYPE1_CAP_PTR_REG_CAP_POINTER_SHIFT                      (0x00000000U)
#define CSL_PCIE_RC_CORE_TYPE1_CAP_PTR_REG_CAP_POINTER_MAX                        (0x000000FFU)

#define CSL_PCIE_RC_CORE_TYPE1_CAP_PTR_REG_RSVDP_8_MASK                           (0xFFFFFF00U)
#define CSL_PCIE_RC_CORE_TYPE1_CAP_PTR_REG_RSVDP_8_SHIFT                          (0x00000008U)
#define CSL_PCIE_RC_CORE_TYPE1_CAP_PTR_REG_RSVDP_8_MAX                            (0x00FFFFFFU)

/* TYPE1_EXP_ROM_BASE_REG */

#define CSL_PCIE_RC_CORE_TYPE1_EXP_ROM_BASE_REG_ROM_BAR_ENABLE_MASK               (0x00000001U)
#define CSL_PCIE_RC_CORE_TYPE1_EXP_ROM_BASE_REG_ROM_BAR_ENABLE_SHIFT              (0x00000000U)
#define CSL_PCIE_RC_CORE_TYPE1_EXP_ROM_BASE_REG_ROM_BAR_ENABLE_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_TYPE1_EXP_ROM_BASE_REG_RSVDP_1_MASK                      (0x000007FEU)
#define CSL_PCIE_RC_CORE_TYPE1_EXP_ROM_BASE_REG_RSVDP_1_SHIFT                     (0x00000001U)
#define CSL_PCIE_RC_CORE_TYPE1_EXP_ROM_BASE_REG_RSVDP_1_MAX                       (0x000003FFU)

#define CSL_PCIE_RC_CORE_TYPE1_EXP_ROM_BASE_REG_EXP_ROM_BASE_ADDRESS_MASK         (0xFFFFF800U)
#define CSL_PCIE_RC_CORE_TYPE1_EXP_ROM_BASE_REG_EXP_ROM_BASE_ADDRESS_SHIFT        (0x0000000BU)
#define CSL_PCIE_RC_CORE_TYPE1_EXP_ROM_BASE_REG_EXP_ROM_BASE_ADDRESS_MAX          (0x001FFFFFU)

/* BRIDGE_CTRL_INT_PIN_INT_LINE_REG */

#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_INT_LINE_MASK           (0x000000FFU)
#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_INT_LINE_SHIFT          (0x00000000U)
#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_INT_LINE_MAX            (0x000000FFU)

#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_INT_PIN_MASK            (0x0000FF00U)
#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_INT_PIN_SHIFT           (0x00000008U)
#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_INT_PIN_MAX             (0x000000FFU)

#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_PERE_MASK               (0x00010000U)
#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_PERE_SHIFT              (0x00000010U)
#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_PERE_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_SERR_EN_MASK            (0x00020000U)
#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_SERR_EN_SHIFT           (0x00000011U)
#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_SERR_EN_MAX             (0x00000001U)

#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_ISA_EN_MASK             (0x00040000U)
#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_ISA_EN_SHIFT            (0x00000012U)
#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_ISA_EN_MAX              (0x00000001U)

#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_VGA_EN_MASK             (0x00080000U)
#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_VGA_EN_SHIFT            (0x00000013U)
#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_VGA_EN_MAX              (0x00000001U)

#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_VGA_16B_DEC_MASK        (0x00100000U)
#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_VGA_16B_DEC_SHIFT       (0x00000014U)
#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_VGA_16B_DEC_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_MSTR_ABORT_MODE_MASK    (0x00200000U)
#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_MSTR_ABORT_MODE_SHIFT   (0x00000015U)
#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_MSTR_ABORT_MODE_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_SBR_MASK                (0x00400000U)
#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_SBR_SHIFT               (0x00000016U)
#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_SBR_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_BRIDGE_CTRL_RESERV_MASK (0xFF800000U)
#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_BRIDGE_CTRL_RESERV_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_BRIDGE_CTRL_RESERV_MAX  (0x000001FFU)

/* CAP_ID_NXT_PTR_REG */

#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG_PM_CAP_ID_MASK                        (0x000000FFU)
#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG_PM_CAP_ID_SHIFT                       (0x00000000U)
#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG_PM_CAP_ID_MAX                         (0x000000FFU)

#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG_PM_NEXT_POINTER_MASK                  (0x0000FF00U)
#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG_PM_NEXT_POINTER_SHIFT                 (0x00000008U)
#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG_PM_NEXT_POINTER_MAX                   (0x000000FFU)

#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG_PM_SPEC_VER_MASK                      (0x00070000U)
#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG_PM_SPEC_VER_SHIFT                     (0x00000010U)
#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG_PM_SPEC_VER_MAX                       (0x00000007U)

#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG_PME_CLK_MASK                          (0x00080000U)
#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG_PME_CLK_SHIFT                         (0x00000013U)
#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG_PME_CLK_MAX                           (0x00000001U)

#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG_DSI_MASK                              (0x00200000U)
#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG_DSI_SHIFT                             (0x00000015U)
#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG_DSI_MAX                               (0x00000001U)

#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG_AUX_CURR_MASK                         (0x01C00000U)
#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG_AUX_CURR_SHIFT                        (0x00000016U)
#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG_AUX_CURR_MAX                          (0x00000007U)

#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG_D1_SUPPORT_MASK                       (0x02000000U)
#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG_D1_SUPPORT_SHIFT                      (0x00000019U)
#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG_D1_SUPPORT_MAX                        (0x00000001U)

#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG_D2_SUPPORT_MASK                       (0x04000000U)
#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG_D2_SUPPORT_SHIFT                      (0x0000001AU)
#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG_D2_SUPPORT_MAX                        (0x00000001U)

#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG_PME_SUPPORT_MASK                      (0xF8000000U)
#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG_PME_SUPPORT_SHIFT                     (0x0000001BU)
#define CSL_PCIE_RC_CORE_CAP_ID_NXT_PTR_REG_PME_SUPPORT_MAX                       (0x0000001FU)

/* CON_STATUS_REG */

#define CSL_PCIE_RC_CORE_CON_STATUS_REG_POWER_STATE_MASK                          (0x00000003U)
#define CSL_PCIE_RC_CORE_CON_STATUS_REG_POWER_STATE_SHIFT                         (0x00000000U)
#define CSL_PCIE_RC_CORE_CON_STATUS_REG_POWER_STATE_MAX                           (0x00000003U)

#define CSL_PCIE_RC_CORE_CON_STATUS_REG_RSVDP_2_MASK                              (0x00000004U)
#define CSL_PCIE_RC_CORE_CON_STATUS_REG_RSVDP_2_SHIFT                             (0x00000002U)
#define CSL_PCIE_RC_CORE_CON_STATUS_REG_RSVDP_2_MAX                               (0x00000001U)

#define CSL_PCIE_RC_CORE_CON_STATUS_REG_NO_SOFT_RST_MASK                          (0x00000008U)
#define CSL_PCIE_RC_CORE_CON_STATUS_REG_NO_SOFT_RST_SHIFT                         (0x00000003U)
#define CSL_PCIE_RC_CORE_CON_STATUS_REG_NO_SOFT_RST_MAX                           (0x00000001U)

#define CSL_PCIE_RC_CORE_CON_STATUS_REG_RSVDP_4_MASK                              (0x000000F0U)
#define CSL_PCIE_RC_CORE_CON_STATUS_REG_RSVDP_4_SHIFT                             (0x00000004U)
#define CSL_PCIE_RC_CORE_CON_STATUS_REG_RSVDP_4_MAX                               (0x0000000FU)

#define CSL_PCIE_RC_CORE_CON_STATUS_REG_PME_ENABLE_MASK                           (0x00000100U)
#define CSL_PCIE_RC_CORE_CON_STATUS_REG_PME_ENABLE_SHIFT                          (0x00000008U)
#define CSL_PCIE_RC_CORE_CON_STATUS_REG_PME_ENABLE_MAX                            (0x00000001U)

#define CSL_PCIE_RC_CORE_CON_STATUS_REG_DATA_SELECT_MASK                          (0x00001E00U)
#define CSL_PCIE_RC_CORE_CON_STATUS_REG_DATA_SELECT_SHIFT                         (0x00000009U)
#define CSL_PCIE_RC_CORE_CON_STATUS_REG_DATA_SELECT_MAX                           (0x0000000FU)

#define CSL_PCIE_RC_CORE_CON_STATUS_REG_DATA_SCALE_MASK                           (0x00006000U)
#define CSL_PCIE_RC_CORE_CON_STATUS_REG_DATA_SCALE_SHIFT                          (0x0000000DU)
#define CSL_PCIE_RC_CORE_CON_STATUS_REG_DATA_SCALE_MAX                            (0x00000003U)

#define CSL_PCIE_RC_CORE_CON_STATUS_REG_PME_STATUS_MASK                           (0x00008000U)
#define CSL_PCIE_RC_CORE_CON_STATUS_REG_PME_STATUS_SHIFT                          (0x0000000FU)
#define CSL_PCIE_RC_CORE_CON_STATUS_REG_PME_STATUS_MAX                            (0x00000001U)

#define CSL_PCIE_RC_CORE_CON_STATUS_REG_RSVDP_16_MASK                             (0x003F0000U)
#define CSL_PCIE_RC_CORE_CON_STATUS_REG_RSVDP_16_SHIFT                            (0x00000010U)
#define CSL_PCIE_RC_CORE_CON_STATUS_REG_RSVDP_16_MAX                              (0x0000003FU)

#define CSL_PCIE_RC_CORE_CON_STATUS_REG_B2_B3_SUPPORT_MASK                        (0x00400000U)
#define CSL_PCIE_RC_CORE_CON_STATUS_REG_B2_B3_SUPPORT_SHIFT                       (0x00000016U)
#define CSL_PCIE_RC_CORE_CON_STATUS_REG_B2_B3_SUPPORT_MAX                         (0x00000001U)

#define CSL_PCIE_RC_CORE_CON_STATUS_REG_BUS_PWR_CLK_CON_EN_MASK                   (0x00800000U)
#define CSL_PCIE_RC_CORE_CON_STATUS_REG_BUS_PWR_CLK_CON_EN_SHIFT                  (0x00000017U)
#define CSL_PCIE_RC_CORE_CON_STATUS_REG_BUS_PWR_CLK_CON_EN_MAX                    (0x00000001U)

#define CSL_PCIE_RC_CORE_CON_STATUS_REG_DATA_REG_ADD_INFO_MASK                    (0xFF000000U)
#define CSL_PCIE_RC_CORE_CON_STATUS_REG_DATA_REG_ADD_INFO_SHIFT                   (0x00000018U)
#define CSL_PCIE_RC_CORE_CON_STATUS_REG_DATA_REG_ADD_INFO_MAX                     (0x000000FFU)

/* PCI_MSI_CAP_ID_NEXT_CTRL_REG */

#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_CAP_ID_MASK         (0x000000FFU)
#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_CAP_ID_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_CAP_ID_MAX          (0x000000FFU)

#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_CAP_NEXT_OFFSET_MASK (0x0000FF00U)
#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_CAP_NEXT_OFFSET_SHIFT (0x00000008U)
#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_CAP_NEXT_OFFSET_MAX (0x000000FFU)

#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_ENABLE_MASK         (0x00010000U)
#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_ENABLE_SHIFT        (0x00000010U)
#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_ENABLE_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_MULTIPLE_MSG_CAP_MASK (0x000E0000U)
#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_MULTIPLE_MSG_CAP_SHIFT (0x00000011U)
#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_MULTIPLE_MSG_CAP_MAX (0x00000007U)

#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_MULTIPLE_MSG_EN_MASK (0x00700000U)
#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_MULTIPLE_MSG_EN_SHIFT (0x00000014U)
#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_MULTIPLE_MSG_EN_MAX (0x00000007U)

#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_64_BIT_ADDR_CAP_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_64_BIT_ADDR_CAP_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_64_BIT_ADDR_CAP_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_PVM_SUPPORT_MASK        (0x01000000U)
#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_PVM_SUPPORT_SHIFT       (0x00000018U)
#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_PVM_SUPPORT_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_EXT_DATA_CAP_MASK   (0x02000000U)
#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_EXT_DATA_CAP_SHIFT  (0x00000019U)
#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_EXT_DATA_CAP_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_EXT_DATA_EN_MASK    (0x04000000U)
#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_EXT_DATA_EN_SHIFT   (0x0000001AU)
#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_EXT_DATA_EN_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_RSVDP_27_MASK               (0xF8000000U)
#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_RSVDP_27_SHIFT              (0x0000001BU)
#define CSL_PCIE_RC_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_RSVDP_27_MAX                (0x0000001FU)

/* MSI_CAP_OFF_04H_REG */

#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_04H_REG_RSVDP_0_MASK                         (0x00000003U)
#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_04H_REG_RSVDP_0_SHIFT                        (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_04H_REG_RSVDP_0_MAX                          (0x00000003U)

#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_04H_REG_PCI_MSI_CAP_OFF_04H_MASK             (0xFFFFFFFCU)
#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_04H_REG_PCI_MSI_CAP_OFF_04H_SHIFT            (0x00000002U)
#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_04H_REG_PCI_MSI_CAP_OFF_04H_MAX              (0x3FFFFFFFU)

/* MSI_CAP_OFF_08H_REG */

#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_08H_REG_PCI_MSI_CAP_OFF_08H_MASK             (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_08H_REG_PCI_MSI_CAP_OFF_08H_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_08H_REG_PCI_MSI_CAP_OFF_08H_MAX              (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_08H_REG_PCI_MSI_CAP_OFF_0AH_MASK             (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_08H_REG_PCI_MSI_CAP_OFF_0AH_SHIFT            (0x00000010U)
#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_08H_REG_PCI_MSI_CAP_OFF_0AH_MAX              (0x0000FFFFU)

/* MSI_CAP_OFF_0CH_REG */

#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_0CH_REG_PCI_MSI_CAP_OFF_0CH_MASK             (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_0CH_REG_PCI_MSI_CAP_OFF_0CH_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_0CH_REG_PCI_MSI_CAP_OFF_0CH_MAX              (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_0CH_REG_PCI_MSI_CAP_OFF_0EH_MASK             (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_0CH_REG_PCI_MSI_CAP_OFF_0EH_SHIFT            (0x00000010U)
#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_0CH_REG_PCI_MSI_CAP_OFF_0EH_MAX              (0x0000FFFFU)

/* MSI_CAP_OFF_10H_REG */

#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_10H_REG_PCI_MSI_CAP_OFF_10H_MASK             (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_10H_REG_PCI_MSI_CAP_OFF_10H_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_10H_REG_PCI_MSI_CAP_OFF_10H_MAX              (0xFFFFFFFFU)

/* MSI_CAP_OFF_14H_REG */

#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_14H_REG_PCI_MSI_CAP_OFF_14H_MASK             (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_14H_REG_PCI_MSI_CAP_OFF_14H_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CAP_OFF_14H_REG_PCI_MSI_CAP_OFF_14H_MAX              (0xFFFFFFFFU)

/* PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG */

#define CSL_PCIE_RC_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_CAP_ID_MASK (0x000000FFU)
#define CSL_PCIE_RC_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_CAP_ID_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_CAP_ID_MAX (0x000000FFU)

#define CSL_PCIE_RC_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_CAP_NEXT_PTR_MASK (0x0000FF00U)
#define CSL_PCIE_RC_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_CAP_NEXT_PTR_SHIFT (0x00000008U)
#define CSL_PCIE_RC_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_CAP_NEXT_PTR_MAX (0x000000FFU)

#define CSL_PCIE_RC_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_CAP_REG_MASK (0x000F0000U)
#define CSL_PCIE_RC_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_CAP_REG_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_CAP_REG_MAX (0x0000000FU)

#define CSL_PCIE_RC_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_DEV_PORT_TYPE_MASK (0x00F00000U)
#define CSL_PCIE_RC_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_DEV_PORT_TYPE_SHIFT (0x00000014U)
#define CSL_PCIE_RC_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_DEV_PORT_TYPE_MAX (0x0000000FU)

#define CSL_PCIE_RC_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_SLOT_IMP_MASK (0x01000000U)
#define CSL_PCIE_RC_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_SLOT_IMP_SHIFT (0x00000018U)
#define CSL_PCIE_RC_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_SLOT_IMP_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_INT_MSG_NUM_MASK (0x3E000000U)
#define CSL_PCIE_RC_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_INT_MSG_NUM_SHIFT (0x00000019U)
#define CSL_PCIE_RC_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_INT_MSG_NUM_MAX (0x0000001FU)

#define CSL_PCIE_RC_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_RSVD_MASK     (0x40000000U)
#define CSL_PCIE_RC_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_RSVD_SHIFT    (0x0000001EU)
#define CSL_PCIE_RC_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_RSVD_MAX      (0x00000001U)

#define CSL_PCIE_RC_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_RSVDP_31_MASK (0x80000000U)
#define CSL_PCIE_RC_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_RSVDP_31_SHIFT (0x0000001FU)
#define CSL_PCIE_RC_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_RSVDP_31_MAX  (0x00000001U)

/* DEVICE_CAPABILITIES_REG */

#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_MAX_PAYLOAD_SIZE_MASK   (0x00000007U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_MAX_PAYLOAD_SIZE_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_MAX_PAYLOAD_SIZE_MAX    (0x00000007U)

#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_PHANTOM_FUNC_SUPPORT_MASK (0x00000018U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_PHANTOM_FUNC_SUPPORT_SHIFT (0x00000003U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_PHANTOM_FUNC_SUPPORT_MAX (0x00000003U)

#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_EXT_TAG_SUPP_MASK       (0x00000020U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_EXT_TAG_SUPP_SHIFT      (0x00000005U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_EXT_TAG_SUPP_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES_REG_RSVDP_6_MASK                     (0x00007FC0U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES_REG_RSVDP_6_SHIFT                    (0x00000006U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES_REG_RSVDP_6_MAX                      (0x000001FFU)

#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_ROLE_BASED_ERR_REPORT_MASK (0x00008000U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_ROLE_BASED_ERR_REPORT_SHIFT (0x0000000FU)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_ROLE_BASED_ERR_REPORT_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES_REG_RSVDP_16_MASK                    (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES_REG_RSVDP_16_SHIFT                   (0x00000010U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES_REG_RSVDP_16_MAX                     (0x0000FFFFU)

/* DEVICE_CONTROL_DEVICE_STATUS */

#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_CORR_ERR_REPORT_EN_MASK (0x00000001U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_CORR_ERR_REPORT_EN_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_CORR_ERR_REPORT_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_NON_FATAL_ERR_REPORT_EN_MASK (0x00000002U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_NON_FATAL_ERR_REPORT_EN_SHIFT (0x00000001U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_NON_FATAL_ERR_REPORT_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_FATAL_ERR_REPORT_EN_MASK (0x00000004U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_FATAL_ERR_REPORT_EN_SHIFT (0x00000002U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_FATAL_ERR_REPORT_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_UNSUPPORT_REQ_REP_EN_MASK (0x00000008U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_UNSUPPORT_REQ_REP_EN_SHIFT (0x00000003U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_UNSUPPORT_REQ_REP_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_EN_REL_ORDER_MASK  (0x00000010U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_EN_REL_ORDER_SHIFT (0x00000004U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_EN_REL_ORDER_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_MAX_PAYLOAD_SIZE_CS_MASK (0x000000E0U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_MAX_PAYLOAD_SIZE_CS_SHIFT (0x00000005U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_MAX_PAYLOAD_SIZE_CS_MAX (0x00000007U)

#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_EXT_TAG_EN_MASK    (0x00000100U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_EXT_TAG_EN_SHIFT   (0x00000008U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_EXT_TAG_EN_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_PHANTOM_FUNC_EN_MASK (0x00000200U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_PHANTOM_FUNC_EN_SHIFT (0x00000009U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_PHANTOM_FUNC_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_AUX_POWER_PM_EN_MASK (0x00000400U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_AUX_POWER_PM_EN_SHIFT (0x0000000AU)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_AUX_POWER_PM_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_EN_NO_SNOOP_MASK   (0x00000800U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_EN_NO_SNOOP_SHIFT  (0x0000000BU)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_EN_NO_SNOOP_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_MAX_READ_REQ_SIZE_MASK (0x00007000U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_MAX_READ_REQ_SIZE_SHIFT (0x0000000CU)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_MAX_READ_REQ_SIZE_MAX (0x00000007U)

#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_INITIATE_FLR_MASK  (0x00008000U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_INITIATE_FLR_SHIFT (0x0000000FU)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_INITIATE_FLR_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_CORR_ERR_DETECTED_MASK (0x00010000U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_CORR_ERR_DETECTED_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_CORR_ERR_DETECTED_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_NON_FATAL_ERR_DETECTED_MASK (0x00020000U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_NON_FATAL_ERR_DETECTED_SHIFT (0x00000011U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_NON_FATAL_ERR_DETECTED_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_FATAL_ERR_DETECTED_MASK (0x00040000U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_FATAL_ERR_DETECTED_SHIFT (0x00000012U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_FATAL_ERR_DETECTED_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_UNSUPPORTED_REQ_DETECTED_MASK (0x00080000U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_UNSUPPORTED_REQ_DETECTED_SHIFT (0x00000013U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_UNSUPPORTED_REQ_DETECTED_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_AUX_POWER_DETECTED_MASK (0x00100000U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_AUX_POWER_DETECTED_SHIFT (0x00000014U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_AUX_POWER_DETECTED_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_TRANS_PENDING_MASK (0x00200000U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_TRANS_PENDING_SHIFT (0x00000015U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_TRANS_PENDING_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_RSVDP_22_MASK               (0xFFC00000U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_RSVDP_22_SHIFT              (0x00000016U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL_DEVICE_STATUS_RSVDP_22_MAX                (0x000003FFU)

/* LINK_CAPABILITIES_REG */

#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_MAX_LINK_SPEED_MASK       (0x0000000FU)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_MAX_LINK_SPEED_SHIFT      (0x00000000U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_MAX_LINK_SPEED_MAX        (0x0000000FU)

#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_MAX_LINK_WIDTH_MASK       (0x000003F0U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_MAX_LINK_WIDTH_SHIFT      (0x00000004U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_MAX_LINK_WIDTH_MAX        (0x0000003FU)

#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_ACTIVE_STATE_LINK_PM_SUPPORT_MASK (0x00000C00U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_ACTIVE_STATE_LINK_PM_SUPPORT_SHIFT (0x0000000AU)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_ACTIVE_STATE_LINK_PM_SUPPORT_MAX (0x00000003U)

#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_L0S_EXIT_LATENCY_MASK     (0x00007000U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_L0S_EXIT_LATENCY_SHIFT    (0x0000000CU)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_L0S_EXIT_LATENCY_MAX      (0x00000007U)

#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_L1_EXIT_LATENCY_MASK      (0x00038000U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_L1_EXIT_LATENCY_SHIFT     (0x0000000FU)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_L1_EXIT_LATENCY_MAX       (0x00000007U)

#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_CLOCK_POWER_MAN_MASK      (0x00040000U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_CLOCK_POWER_MAN_SHIFT     (0x00000012U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_CLOCK_POWER_MAN_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_SURPRISE_DOWN_ERR_REP_CAP_MASK (0x00080000U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_SURPRISE_DOWN_ERR_REP_CAP_SHIFT (0x00000013U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_SURPRISE_DOWN_ERR_REP_CAP_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_DLL_ACTIVE_REP_CAP_MASK   (0x00100000U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_DLL_ACTIVE_REP_CAP_SHIFT  (0x00000014U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_DLL_ACTIVE_REP_CAP_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_LINK_BW_NOT_CAP_MASK      (0x00200000U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_LINK_BW_NOT_CAP_SHIFT     (0x00000015U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_LINK_BW_NOT_CAP_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_ASPM_OPT_COMPLIANCE_MASK  (0x00400000U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_ASPM_OPT_COMPLIANCE_SHIFT (0x00000016U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_ASPM_OPT_COMPLIANCE_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_RSVDP_23_MASK                      (0x00800000U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_RSVDP_23_SHIFT                     (0x00000017U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_RSVDP_23_MAX                       (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_PORT_NUM_MASK             (0xFF000000U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_PORT_NUM_SHIFT            (0x00000018U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_PORT_NUM_MAX              (0x000000FFU)

/* LINK_CONTROL_LINK_STATUS_REG */

#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_ACTIVE_STATE_LINK_PM_CONTROL_MASK (0x00000003U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_ACTIVE_STATE_LINK_PM_CONTROL_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_ACTIVE_STATE_LINK_PM_CONTROL_MAX (0x00000003U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_RSVDP_2_MASK                (0x00000004U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_RSVDP_2_SHIFT               (0x00000002U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_RSVDP_2_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_RCB_MASK           (0x00000008U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_RCB_SHIFT          (0x00000003U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_RCB_MAX            (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_DISABLE_MASK  (0x00000010U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_DISABLE_SHIFT (0x00000004U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_DISABLE_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_RETRAIN_LINK_MASK  (0x00000020U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_RETRAIN_LINK_SHIFT (0x00000005U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_RETRAIN_LINK_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_COMMON_CLK_CONFIG_MASK (0x00000040U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_COMMON_CLK_CONFIG_SHIFT (0x00000006U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_COMMON_CLK_CONFIG_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_EXTENDED_SYNCH_MASK (0x00000080U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_EXTENDED_SYNCH_SHIFT (0x00000007U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_EXTENDED_SYNCH_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_EN_CLK_POWER_MAN_MASK (0x00000100U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_EN_CLK_POWER_MAN_SHIFT (0x00000008U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_EN_CLK_POWER_MAN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_HW_AUTO_WIDTH_DISABLE_MASK (0x00000200U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_HW_AUTO_WIDTH_DISABLE_SHIFT (0x00000009U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_HW_AUTO_WIDTH_DISABLE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_BW_MAN_INT_EN_MASK (0x00000400U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_BW_MAN_INT_EN_SHIFT (0x0000000AU)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_BW_MAN_INT_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_AUTO_BW_INT_EN_MASK (0x00000800U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_AUTO_BW_INT_EN_SHIFT (0x0000000BU)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_AUTO_BW_INT_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_RSVDP_12_MASK               (0x00003000U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_RSVDP_12_SHIFT              (0x0000000CU)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_RSVDP_12_MAX                (0x00000003U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_DRS_SIGNALING_CONTROL_MASK (0x0000C000U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_DRS_SIGNALING_CONTROL_SHIFT (0x0000000EU)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_DRS_SIGNALING_CONTROL_MAX (0x00000003U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_SPEED_MASK    (0x000F0000U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_SPEED_SHIFT   (0x00000010U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_SPEED_MAX     (0x0000000FU)

#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_NEGO_LINK_WIDTH_MASK (0x03F00000U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_NEGO_LINK_WIDTH_SHIFT (0x00000014U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_NEGO_LINK_WIDTH_MAX (0x0000003FU)

#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_RSVDP_26_MASK               (0x04000000U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_RSVDP_26_SHIFT              (0x0000001AU)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_RSVDP_26_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_TRAINING_MASK (0x08000000U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_TRAINING_SHIFT (0x0000001BU)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_TRAINING_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_SLOT_CLK_CONFIG_MASK (0x10000000U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_SLOT_CLK_CONFIG_SHIFT (0x0000001CU)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_SLOT_CLK_CONFIG_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_DLL_ACTIVE_MASK    (0x20000000U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_DLL_ACTIVE_SHIFT   (0x0000001DU)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_DLL_ACTIVE_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_BW_MAN_STATUS_MASK (0x40000000U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_BW_MAN_STATUS_SHIFT (0x0000001EU)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_BW_MAN_STATUS_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_AUTO_BW_STATUS_MASK (0x80000000U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_AUTO_BW_STATUS_SHIFT (0x0000001FU)
#define CSL_PCIE_RC_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_AUTO_BW_STATUS_MAX (0x00000001U)

/* SLOT_CAPABILITIES_REG */

#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_ATTENTION_INDICATOR_BUTTON_MASK (0x00000001U)
#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_ATTENTION_INDICATOR_BUTTON_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_ATTENTION_INDICATOR_BUTTON_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_POWER_CONTROLLER_MASK     (0x00000002U)
#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_POWER_CONTROLLER_SHIFT    (0x00000001U)
#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_POWER_CONTROLLER_MAX      (0x00000001U)

#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_MRL_SENSOR_MASK           (0x00000004U)
#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_MRL_SENSOR_SHIFT          (0x00000002U)
#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_MRL_SENSOR_MAX            (0x00000001U)

#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_ATTENTION_INDICATOR_MASK  (0x00000008U)
#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_ATTENTION_INDICATOR_SHIFT (0x00000003U)
#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_ATTENTION_INDICATOR_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_POWER_INDICATOR_MASK      (0x00000010U)
#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_POWER_INDICATOR_SHIFT     (0x00000004U)
#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_POWER_INDICATOR_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_HOT_PLUG_SURPRISE_MASK    (0x00000020U)
#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_HOT_PLUG_SURPRISE_SHIFT   (0x00000005U)
#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_HOT_PLUG_SURPRISE_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_HOT_PLUG_CAPABLE_MASK     (0x00000040U)
#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_HOT_PLUG_CAPABLE_SHIFT    (0x00000006U)
#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_HOT_PLUG_CAPABLE_MAX      (0x00000001U)

#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_SLOT_POWER_LIMIT_VALUE_MASK (0x00007F80U)
#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_SLOT_POWER_LIMIT_VALUE_SHIFT (0x00000007U)
#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_SLOT_POWER_LIMIT_VALUE_MAX (0x000000FFU)

#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_SLOT_POWER_LIMIT_SCALE_MASK (0x00018000U)
#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_SLOT_POWER_LIMIT_SCALE_SHIFT (0x0000000FU)
#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_SLOT_POWER_LIMIT_SCALE_MAX (0x00000003U)

#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_ELECTROMECH_INTERLOCK_MASK (0x00020000U)
#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_ELECTROMECH_INTERLOCK_SHIFT (0x00000011U)
#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_ELECTROMECH_INTERLOCK_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_NO_CMD_CPL_SUPPORT_MASK   (0x00040000U)
#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_NO_CMD_CPL_SUPPORT_SHIFT  (0x00000012U)
#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_NO_CMD_CPL_SUPPORT_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_PHY_SLOT_NUM_MASK         (0xFFF80000U)
#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_PHY_SLOT_NUM_SHIFT        (0x00000013U)
#define CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_PHY_SLOT_NUM_MAX          (0x00001FFFU)

/* SLOT_CONTROL_SLOT_STATUS */

#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ATTENTION_BUTTON_PRESSED_EN_MASK (0x00000001U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ATTENTION_BUTTON_PRESSED_EN_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ATTENTION_BUTTON_PRESSED_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_FAULT_DETECTED_EN_MASK (0x00000002U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_FAULT_DETECTED_EN_SHIFT (0x00000001U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_FAULT_DETECTED_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_MRL_SENSOR_CHANGED_EN_MASK (0x00000004U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_MRL_SENSOR_CHANGED_EN_SHIFT (0x00000002U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_MRL_SENSOR_CHANGED_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_PRESENCE_DETECT_CHANGE_EN_MASK (0x00000008U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_PRESENCE_DETECT_CHANGE_EN_SHIFT (0x00000003U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_PRESENCE_DETECT_CHANGE_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_CMD_CPL_INT_EN_MASK    (0x00000010U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_CMD_CPL_INT_EN_SHIFT   (0x00000004U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_CMD_CPL_INT_EN_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_HOT_PLUG_INT_EN_MASK   (0x00000020U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_HOT_PLUG_INT_EN_SHIFT  (0x00000005U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_HOT_PLUG_INT_EN_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ATTENTION_INDICATOR_CTRL_MASK (0x000000C0U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ATTENTION_INDICATOR_CTRL_SHIFT (0x00000006U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ATTENTION_INDICATOR_CTRL_MAX (0x00000003U)

#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_INDICATOR_CTRL_MASK (0x00000300U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_INDICATOR_CTRL_SHIFT (0x00000008U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_INDICATOR_CTRL_MAX (0x00000003U)

#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_CONTROLLER_CTRL_MASK (0x00000400U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_CONTROLLER_CTRL_SHIFT (0x0000000AU)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_CONTROLLER_CTRL_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ELECTROMECH_INTERLOCK_CTRL_MASK (0x00000800U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ELECTROMECH_INTERLOCK_CTRL_SHIFT (0x0000000BU)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ELECTROMECH_INTERLOCK_CTRL_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_DLL_STATE_CHANGED_EN_MASK (0x00001000U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_DLL_STATE_CHANGED_EN_SHIFT (0x0000000CU)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_DLL_STATE_CHANGED_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_RSVDP_13_MASK                   (0x0000E000U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_RSVDP_13_SHIFT                  (0x0000000DU)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_RSVDP_13_MAX                    (0x00000007U)

#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ATTENTION_BUTTON_PRESSED_MASK (0x00010000U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ATTENTION_BUTTON_PRESSED_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ATTENTION_BUTTON_PRESSED_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_FAULT_DETECTED_MASK (0x00020000U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_FAULT_DETECTED_SHIFT (0x00000011U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_FAULT_DETECTED_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_MRL_SENSOR_CHANGED_MASK (0x00040000U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_MRL_SENSOR_CHANGED_SHIFT (0x00000012U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_MRL_SENSOR_CHANGED_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_PRESENCE_DETECTED_CHANGED_MASK (0x00080000U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_PRESENCE_DETECTED_CHANGED_SHIFT (0x00000013U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_PRESENCE_DETECTED_CHANGED_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_CMD_CPLD_MASK          (0x00100000U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_CMD_CPLD_SHIFT         (0x00000014U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_CMD_CPLD_MAX           (0x00000001U)

#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_MRL_SENSOR_STATE_MASK  (0x00200000U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_MRL_SENSOR_STATE_SHIFT (0x00000015U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_MRL_SENSOR_STATE_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_PRESENCE_DETECT_STATE_MASK (0x00400000U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_PRESENCE_DETECT_STATE_SHIFT (0x00000016U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_PRESENCE_DETECT_STATE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ELECTROMECH_INTERLOCK_STATUS_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ELECTROMECH_INTERLOCK_STATUS_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ELECTROMECH_INTERLOCK_STATUS_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_DLL_STATE_CHANGED_MASK (0x01000000U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_DLL_STATE_CHANGED_SHIFT (0x00000018U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_DLL_STATE_CHANGED_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_RSVDP_25_MASK                   (0xFE000000U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_RSVDP_25_SHIFT                  (0x00000019U)
#define CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_RSVDP_25_MAX                    (0x0000007FU)

/* ROOT_CONTROL_ROOT_CAPABILITIES_REG */

#define CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_SYS_ERR_ON_CORR_ERR_EN_MASK (0x00000001U)
#define CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_SYS_ERR_ON_CORR_ERR_EN_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_SYS_ERR_ON_CORR_ERR_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_SYS_ERR_ON_NON_FATAL_ERR_EN_MASK (0x00000002U)
#define CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_SYS_ERR_ON_NON_FATAL_ERR_EN_SHIFT (0x00000001U)
#define CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_SYS_ERR_ON_NON_FATAL_ERR_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_SYS_ERR_ON_FATAL_ERR_EN_MASK (0x00000004U)
#define CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_SYS_ERR_ON_FATAL_ERR_EN_SHIFT (0x00000002U)
#define CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_SYS_ERR_ON_FATAL_ERR_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_PME_INT_EN_MASK (0x00000008U)
#define CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_PME_INT_EN_SHIFT (0x00000003U)
#define CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_PME_INT_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_CRS_SW_VISIBILITY_EN_MASK (0x00000010U)
#define CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_CRS_SW_VISIBILITY_EN_SHIFT (0x00000004U)
#define CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_CRS_SW_VISIBILITY_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_RSVDP_5_MASK          (0x0000FFE0U)
#define CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_RSVDP_5_SHIFT         (0x00000005U)
#define CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_RSVDP_5_MAX           (0x000007FFU)

#define CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_CRS_SW_VISIBILITY_MASK (0x00010000U)
#define CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_CRS_SW_VISIBILITY_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_CRS_SW_VISIBILITY_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_RSVDP_17_MASK         (0xFFFE0000U)
#define CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_RSVDP_17_SHIFT        (0x00000011U)
#define CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_RSVDP_17_MAX          (0x00007FFFU)

/* ROOT_STATUS_REG */

#define CSL_PCIE_RC_CORE_ROOT_STATUS_REG_PCIE_CAP_PME_REQ_ID_MASK                 (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_ROOT_STATUS_REG_PCIE_CAP_PME_REQ_ID_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_ROOT_STATUS_REG_PCIE_CAP_PME_REQ_ID_MAX                  (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_ROOT_STATUS_REG_PCIE_CAP_PME_STATUS_MASK                 (0x00010000U)
#define CSL_PCIE_RC_CORE_ROOT_STATUS_REG_PCIE_CAP_PME_STATUS_SHIFT                (0x00000010U)
#define CSL_PCIE_RC_CORE_ROOT_STATUS_REG_PCIE_CAP_PME_STATUS_MAX                  (0x00000001U)

#define CSL_PCIE_RC_CORE_ROOT_STATUS_REG_PCIE_CAP_PME_PENDING_MASK                (0x00020000U)
#define CSL_PCIE_RC_CORE_ROOT_STATUS_REG_PCIE_CAP_PME_PENDING_SHIFT               (0x00000011U)
#define CSL_PCIE_RC_CORE_ROOT_STATUS_REG_PCIE_CAP_PME_PENDING_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_ROOT_STATUS_REG_RSVDP_18_MASK                            (0xFFFC0000U)
#define CSL_PCIE_RC_CORE_ROOT_STATUS_REG_RSVDP_18_SHIFT                           (0x00000012U)
#define CSL_PCIE_RC_CORE_ROOT_STATUS_REG_RSVDP_18_MAX                             (0x00003FFFU)

/* DEVICE_CAPABILITIES2_REG */

#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_CPL_TIMEOUT_RANGE_MASK (0x0000000FU)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_CPL_TIMEOUT_RANGE_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_CPL_TIMEOUT_RANGE_MAX  (0x0000000FU)

#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_CPL_TIMEOUT_DISABLE_SUPPORT_MASK (0x00000010U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_CPL_TIMEOUT_DISABLE_SUPPORT_SHIFT (0x00000004U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_CPL_TIMEOUT_DISABLE_SUPPORT_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_ARI_FORWARD_SUPPORT_MASK (0x00000020U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_ARI_FORWARD_SUPPORT_SHIFT (0x00000005U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_ARI_FORWARD_SUPPORT_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_ATOMIC_ROUTING_SUPP_MASK (0x00000040U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_ATOMIC_ROUTING_SUPP_SHIFT (0x00000006U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_ATOMIC_ROUTING_SUPP_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_32_ATOMIC_CPL_SUPP_MASK (0x00000080U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_32_ATOMIC_CPL_SUPP_SHIFT (0x00000007U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_32_ATOMIC_CPL_SUPP_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_64_ATOMIC_CPL_SUPP_MASK (0x00000100U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_64_ATOMIC_CPL_SUPP_SHIFT (0x00000008U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_64_ATOMIC_CPL_SUPP_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_128_CAS_CPL_SUPP_MASK  (0x00000200U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_128_CAS_CPL_SUPP_SHIFT (0x00000009U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_128_CAS_CPL_SUPP_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_NO_RO_EN_PR2PR_PAR_MASK (0x00000400U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_NO_RO_EN_PR2PR_PAR_SHIFT (0x0000000AU)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_NO_RO_EN_PR2PR_PAR_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_LTR_SUPP_MASK          (0x00000800U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_LTR_SUPP_SHIFT         (0x0000000BU)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_LTR_SUPP_MAX           (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_TPH_CMPLT_SUPPORT_0_MASK (0x00001000U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_TPH_CMPLT_SUPPORT_0_SHIFT (0x0000000CU)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_TPH_CMPLT_SUPPORT_0_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_TPH_CMPLT_SUPPORT_1_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_TPH_CMPLT_SUPPORT_1_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_TPH_CMPLT_SUPPORT_1_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP2_10_BIT_TAG_COMP_SUPPORT_MASK (0x00010000U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP2_10_BIT_TAG_COMP_SUPPORT_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP2_10_BIT_TAG_COMP_SUPPORT_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP2_10_BIT_TAG_REQ_SUPPORT_MASK (0x00020000U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP2_10_BIT_TAG_REQ_SUPPORT_SHIFT (0x00000011U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP2_10_BIT_TAG_REQ_SUPPORT_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_OBFF_SUPPORT_MASK      (0x000C0000U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_OBFF_SUPPORT_SHIFT     (0x00000012U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_OBFF_SUPPORT_MAX       (0x00000003U)

#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_RSVDP_24_MASK                   (0x7F000000U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_RSVDP_24_SHIFT                  (0x00000018U)
#define CSL_PCIE_RC_CORE_DEVICE_CAPABILITIES2_REG_RSVDP_24_MAX                    (0x0000007FU)

/* DEVICE_CONTROL2_DEVICE_STATUS2_REG */

#define CSL_PCIE_RC_CORE_DEVICE_CONTROL2_DEVICE_STATUS2_REG_PCIE_CAP_CPL_TIMEOUT_VALUE_MASK (0x0000000FU)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL2_DEVICE_STATUS2_REG_PCIE_CAP_CPL_TIMEOUT_VALUE_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL2_DEVICE_STATUS2_REG_PCIE_CAP_CPL_TIMEOUT_VALUE_MAX (0x0000000FU)

#define CSL_PCIE_RC_CORE_DEVICE_CONTROL2_DEVICE_STATUS2_REG_PCIE_CAP_CPL_TIMEOUT_DISABLE_MASK (0x00000010U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL2_DEVICE_STATUS2_REG_PCIE_CAP_CPL_TIMEOUT_DISABLE_SHIFT (0x00000004U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL2_DEVICE_STATUS2_REG_PCIE_CAP_CPL_TIMEOUT_DISABLE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_DEVICE_CONTROL2_DEVICE_STATUS2_REG_PCIE_CAP_ARI_FORWARD_SUPPORT_CS_MASK (0x00000020U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL2_DEVICE_STATUS2_REG_PCIE_CAP_ARI_FORWARD_SUPPORT_CS_SHIFT (0x00000005U)
#define CSL_PCIE_RC_CORE_DEVICE_CONTROL2_DEVICE_STATUS2_REG_PCIE_CAP_ARI_FORWARD_SUPPORT_CS_MAX (0x00000001U)

/* LINK_CAPABILITIES2_REG */

#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES2_REG_RSVDP_0_MASK                      (0x00000001U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES2_REG_RSVDP_0_SHIFT                     (0x00000000U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES2_REG_RSVDP_0_MAX                       (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES2_REG_PCIE_CAP_SUPPORT_LINK_SPEED_VECTOR_MASK (0x000000FEU)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES2_REG_PCIE_CAP_SUPPORT_LINK_SPEED_VECTOR_SHIFT (0x00000001U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES2_REG_PCIE_CAP_SUPPORT_LINK_SPEED_VECTOR_MAX (0x0000007FU)

#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES2_REG_PCIE_CAP_CROSS_LINK_SUPPORT_MASK  (0x00000100U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES2_REG_PCIE_CAP_CROSS_LINK_SUPPORT_SHIFT (0x00000008U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES2_REG_PCIE_CAP_CROSS_LINK_SUPPORT_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES2_REG_RSVDP_9_MASK                      (0x007FFE00U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES2_REG_RSVDP_9_SHIFT                     (0x00000009U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES2_REG_RSVDP_9_MAX                       (0x00003FFFU)

#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES2_REG_RSVDP_25_MASK                     (0x7E000000U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES2_REG_RSVDP_25_SHIFT                    (0x00000019U)
#define CSL_PCIE_RC_CORE_LINK_CAPABILITIES2_REG_RSVDP_25_MAX                      (0x0000003FU)

/* LINK_CONTROL2_LINK_STATUS2_REG */

#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_TARGET_LINK_SPEED_MASK (0x0000000FU)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_TARGET_LINK_SPEED_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_TARGET_LINK_SPEED_MAX (0x0000000FU)

#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_ENTER_COMPLIANCE_MASK (0x00000010U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_ENTER_COMPLIANCE_SHIFT (0x00000004U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_ENTER_COMPLIANCE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_HW_AUTO_SPEED_DISABLE_MASK (0x00000020U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_HW_AUTO_SPEED_DISABLE_SHIFT (0x00000005U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_HW_AUTO_SPEED_DISABLE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_SEL_DEEMPHASIS_MASK (0x00000040U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_SEL_DEEMPHASIS_SHIFT (0x00000006U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_SEL_DEEMPHASIS_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_TX_MARGIN_MASK   (0x00000380U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_TX_MARGIN_SHIFT  (0x00000007U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_TX_MARGIN_MAX    (0x00000007U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_ENTER_MODIFIED_COMPLIANCE_MASK (0x00000400U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_ENTER_MODIFIED_COMPLIANCE_SHIFT (0x0000000AU)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_ENTER_MODIFIED_COMPLIANCE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_COMPLIANCE_SOS_MASK (0x00000800U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_COMPLIANCE_SOS_SHIFT (0x0000000BU)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_COMPLIANCE_SOS_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_COMPLIANCE_PRESET_MASK (0x0000F000U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_COMPLIANCE_PRESET_SHIFT (0x0000000CU)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_COMPLIANCE_PRESET_MAX (0x0000000FU)

#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_CURR_DEEMPHASIS_MASK (0x00010000U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_CURR_DEEMPHASIS_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_CURR_DEEMPHASIS_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL_MASK      (0x00020000U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL_SHIFT     (0x00000011U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL_P1_MASK   (0x00040000U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL_P1_SHIFT  (0x00000012U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL_P1_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL_P2_MASK   (0x00080000U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL_P2_SHIFT  (0x00000013U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL_P2_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL_P3_MASK   (0x00100000U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL_P3_SHIFT  (0x00000014U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL_P3_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_LINK_EQ_REQ_MASK (0x00200000U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_LINK_EQ_REQ_SHIFT (0x00000015U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_LINK_EQ_REQ_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_RSVDP_26_MASK             (0x0C000000U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_RSVDP_26_SHIFT            (0x0000001AU)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_RSVDP_26_MAX              (0x00000003U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_DOWNSTREAM_COMPO_PRESENCE_MASK (0x70000000U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_DOWNSTREAM_COMPO_PRESENCE_SHIFT (0x0000001CU)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_DOWNSTREAM_COMPO_PRESENCE_MAX (0x00000007U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_DRS_MESSAGE_RECEIVED_MASK (0x80000000U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_DRS_MESSAGE_RECEIVED_SHIFT (0x0000001FU)
#define CSL_PCIE_RC_CORE_LINK_CONTROL2_LINK_STATUS2_REG_DRS_MESSAGE_RECEIVED_MAX  (0x00000001U)

/* PCI_MSIX_CAP_ID_NEXT_CTRL_REG */

#define CSL_PCIE_RC_CORE_PCI_MSIX_CAP_ID_NEXT_CTRL_REG_PCI_MSIX_CAP_ID_MASK       (0x000000FFU)
#define CSL_PCIE_RC_CORE_PCI_MSIX_CAP_ID_NEXT_CTRL_REG_PCI_MSIX_CAP_ID_SHIFT      (0x00000000U)
#define CSL_PCIE_RC_CORE_PCI_MSIX_CAP_ID_NEXT_CTRL_REG_PCI_MSIX_CAP_ID_MAX        (0x000000FFU)

#define CSL_PCIE_RC_CORE_PCI_MSIX_CAP_ID_NEXT_CTRL_REG_PCI_MSIX_CAP_NEXT_OFFSET_MASK (0x0000FF00U)
#define CSL_PCIE_RC_CORE_PCI_MSIX_CAP_ID_NEXT_CTRL_REG_PCI_MSIX_CAP_NEXT_OFFSET_SHIFT (0x00000008U)
#define CSL_PCIE_RC_CORE_PCI_MSIX_CAP_ID_NEXT_CTRL_REG_PCI_MSIX_CAP_NEXT_OFFSET_MAX (0x000000FFU)

#define CSL_PCIE_RC_CORE_PCI_MSIX_CAP_ID_NEXT_CTRL_REG_PCI_MSIX_TABLE_SIZE_MASK   (0x07FF0000U)
#define CSL_PCIE_RC_CORE_PCI_MSIX_CAP_ID_NEXT_CTRL_REG_PCI_MSIX_TABLE_SIZE_SHIFT  (0x00000010U)
#define CSL_PCIE_RC_CORE_PCI_MSIX_CAP_ID_NEXT_CTRL_REG_PCI_MSIX_TABLE_SIZE_MAX    (0x000007FFU)

#define CSL_PCIE_RC_CORE_PCI_MSIX_CAP_ID_NEXT_CTRL_REG_RSVDP_27_MASK              (0x38000000U)
#define CSL_PCIE_RC_CORE_PCI_MSIX_CAP_ID_NEXT_CTRL_REG_RSVDP_27_SHIFT             (0x0000001BU)
#define CSL_PCIE_RC_CORE_PCI_MSIX_CAP_ID_NEXT_CTRL_REG_RSVDP_27_MAX               (0x00000007U)

#define CSL_PCIE_RC_CORE_PCI_MSIX_CAP_ID_NEXT_CTRL_REG_PCI_MSIX_FUNCTION_MASK_MASK (0x40000000U)
#define CSL_PCIE_RC_CORE_PCI_MSIX_CAP_ID_NEXT_CTRL_REG_PCI_MSIX_FUNCTION_MASK_SHIFT (0x0000001EU)
#define CSL_PCIE_RC_CORE_PCI_MSIX_CAP_ID_NEXT_CTRL_REG_PCI_MSIX_FUNCTION_MASK_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_PCI_MSIX_CAP_ID_NEXT_CTRL_REG_PCI_MSIX_ENABLE_MASK       (0x80000000U)
#define CSL_PCIE_RC_CORE_PCI_MSIX_CAP_ID_NEXT_CTRL_REG_PCI_MSIX_ENABLE_SHIFT      (0x0000001FU)
#define CSL_PCIE_RC_CORE_PCI_MSIX_CAP_ID_NEXT_CTRL_REG_PCI_MSIX_ENABLE_MAX        (0x00000001U)

/* MSIX_TABLE_OFFSET_REG */

#define CSL_PCIE_RC_CORE_MSIX_TABLE_OFFSET_REG_PCI_MSIX_BIR_MASK                  (0x00000007U)
#define CSL_PCIE_RC_CORE_MSIX_TABLE_OFFSET_REG_PCI_MSIX_BIR_SHIFT                 (0x00000000U)
#define CSL_PCIE_RC_CORE_MSIX_TABLE_OFFSET_REG_PCI_MSIX_BIR_MAX                   (0x00000007U)

#define CSL_PCIE_RC_CORE_MSIX_TABLE_OFFSET_REG_PCI_MSIX_TABLE_OFFSET_MASK         (0xFFFFFFF8U)
#define CSL_PCIE_RC_CORE_MSIX_TABLE_OFFSET_REG_PCI_MSIX_TABLE_OFFSET_SHIFT        (0x00000003U)
#define CSL_PCIE_RC_CORE_MSIX_TABLE_OFFSET_REG_PCI_MSIX_TABLE_OFFSET_MAX          (0x1FFFFFFFU)

/* MSIX_PBA_OFFSET_REG */

#define CSL_PCIE_RC_CORE_MSIX_PBA_OFFSET_REG_PCI_MSIX_PBA_MASK                    (0x00000007U)
#define CSL_PCIE_RC_CORE_MSIX_PBA_OFFSET_REG_PCI_MSIX_PBA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RC_CORE_MSIX_PBA_OFFSET_REG_PCI_MSIX_PBA_MAX                     (0x00000007U)

#define CSL_PCIE_RC_CORE_MSIX_PBA_OFFSET_REG_PCI_MSIX_PBA_OFFSET_MASK             (0xFFFFFFF8U)
#define CSL_PCIE_RC_CORE_MSIX_PBA_OFFSET_REG_PCI_MSIX_PBA_OFFSET_SHIFT            (0x00000003U)
#define CSL_PCIE_RC_CORE_MSIX_PBA_OFFSET_REG_PCI_MSIX_PBA_OFFSET_MAX              (0x1FFFFFFFU)

/* AER_EXT_CAP_HDR_OFF */

#define CSL_PCIE_RC_CORE_AER_EXT_CAP_HDR_OFF_CAP_ID_MASK                          (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_AER_EXT_CAP_HDR_OFF_CAP_ID_SHIFT                         (0x00000000U)
#define CSL_PCIE_RC_CORE_AER_EXT_CAP_HDR_OFF_CAP_ID_MAX                           (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_AER_EXT_CAP_HDR_OFF_CAP_VERSION_MASK                     (0x000F0000U)
#define CSL_PCIE_RC_CORE_AER_EXT_CAP_HDR_OFF_CAP_VERSION_SHIFT                    (0x00000010U)
#define CSL_PCIE_RC_CORE_AER_EXT_CAP_HDR_OFF_CAP_VERSION_MAX                      (0x0000000FU)

#define CSL_PCIE_RC_CORE_AER_EXT_CAP_HDR_OFF_NEXT_OFFSET_MASK                     (0xFFF00000U)
#define CSL_PCIE_RC_CORE_AER_EXT_CAP_HDR_OFF_NEXT_OFFSET_SHIFT                    (0x00000014U)
#define CSL_PCIE_RC_CORE_AER_EXT_CAP_HDR_OFF_NEXT_OFFSET_MAX                      (0x00000FFFU)

/* UNCORR_ERR_STATUS_OFF */

#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_RSVDP_0_MASK                       (0x0000000FU)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_RSVDP_0_SHIFT                      (0x00000000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_RSVDP_0_MAX                        (0x0000000FU)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_DL_PROTOCOL_ERR_STATUS_MASK        (0x00000010U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_DL_PROTOCOL_ERR_STATUS_SHIFT       (0x00000004U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_DL_PROTOCOL_ERR_STATUS_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_SURPRISE_DOWN_ERR_STATUS_MASK      (0x00000020U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_SURPRISE_DOWN_ERR_STATUS_SHIFT     (0x00000005U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_SURPRISE_DOWN_ERR_STATUS_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_RSVDP_6_MASK                       (0x00000FC0U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_RSVDP_6_SHIFT                      (0x00000006U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_RSVDP_6_MAX                        (0x0000003FU)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_POIS_TLP_ERR_STATUS_MASK           (0x00001000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_POIS_TLP_ERR_STATUS_SHIFT          (0x0000000CU)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_POIS_TLP_ERR_STATUS_MAX            (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_FC_PROTOCOL_ERR_STATUS_MASK        (0x00002000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_FC_PROTOCOL_ERR_STATUS_SHIFT       (0x0000000DU)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_FC_PROTOCOL_ERR_STATUS_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_CMPLT_TIMEOUT_ERR_STATUS_MASK      (0x00004000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_CMPLT_TIMEOUT_ERR_STATUS_SHIFT     (0x0000000EU)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_CMPLT_TIMEOUT_ERR_STATUS_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_CMPLT_ABORT_ERR_STATUS_MASK        (0x00008000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_CMPLT_ABORT_ERR_STATUS_SHIFT       (0x0000000FU)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_CMPLT_ABORT_ERR_STATUS_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_UNEXP_CMPLT_ERR_STATUS_MASK        (0x00010000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_UNEXP_CMPLT_ERR_STATUS_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_UNEXP_CMPLT_ERR_STATUS_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_REC_OVERFLOW_ERR_STATUS_MASK       (0x00020000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_REC_OVERFLOW_ERR_STATUS_SHIFT      (0x00000011U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_REC_OVERFLOW_ERR_STATUS_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_MALF_TLP_ERR_STATUS_MASK           (0x00040000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_MALF_TLP_ERR_STATUS_SHIFT          (0x00000012U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_MALF_TLP_ERR_STATUS_MAX            (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_ECRC_ERR_STATUS_MASK               (0x00080000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_ECRC_ERR_STATUS_SHIFT              (0x00000013U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_ECRC_ERR_STATUS_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_UNSUPPORTED_REQ_ERR_STATUS_MASK    (0x00100000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_UNSUPPORTED_REQ_ERR_STATUS_SHIFT   (0x00000014U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_UNSUPPORTED_REQ_ERR_STATUS_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_INTERNAL_ERR_STATUS_MASK           (0x00400000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_INTERNAL_ERR_STATUS_SHIFT          (0x00000016U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_INTERNAL_ERR_STATUS_MAX            (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_RSVDP_23_MASK                      (0x01800000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_RSVDP_23_SHIFT                     (0x00000017U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_RSVDP_23_MAX                       (0x00000003U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_TLP_PRFX_BLOCKED_ERR_STATUS_MASK   (0x02000000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_TLP_PRFX_BLOCKED_ERR_STATUS_SHIFT  (0x00000019U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_TLP_PRFX_BLOCKED_ERR_STATUS_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_RSVDP_26_MASK                      (0xFC000000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_RSVDP_26_SHIFT                     (0x0000001AU)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_STATUS_OFF_RSVDP_26_MAX                       (0x0000003FU)

/* UNCORR_ERR_MASK_OFF */

#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_RSVDP_0_MASK                         (0x0000000FU)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_RSVDP_0_SHIFT                        (0x00000000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_RSVDP_0_MAX                          (0x0000000FU)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_DL_PROTOCOL_ERR_MASK_MASK            (0x00000010U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_DL_PROTOCOL_ERR_MASK_SHIFT           (0x00000004U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_DL_PROTOCOL_ERR_MASK_MAX             (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_SURPRISE_DOWN_ERR_MASK_MASK          (0x00000020U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_SURPRISE_DOWN_ERR_MASK_SHIFT         (0x00000005U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_SURPRISE_DOWN_ERR_MASK_MAX           (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_RSVDP_6_MASK                         (0x00000FC0U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_RSVDP_6_SHIFT                        (0x00000006U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_RSVDP_6_MAX                          (0x0000003FU)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_POIS_TLP_ERR_MASK_MASK               (0x00001000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_POIS_TLP_ERR_MASK_SHIFT              (0x0000000CU)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_POIS_TLP_ERR_MASK_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_FC_PROTOCOL_ERR_MASK_MASK            (0x00002000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_FC_PROTOCOL_ERR_MASK_SHIFT           (0x0000000DU)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_FC_PROTOCOL_ERR_MASK_MAX             (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_CMPLT_TIMEOUT_ERR_MASK_MASK          (0x00004000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_CMPLT_TIMEOUT_ERR_MASK_SHIFT         (0x0000000EU)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_CMPLT_TIMEOUT_ERR_MASK_MAX           (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_CMPLT_ABORT_ERR_MASK_MASK            (0x00008000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_CMPLT_ABORT_ERR_MASK_SHIFT           (0x0000000FU)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_CMPLT_ABORT_ERR_MASK_MAX             (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_UNEXP_CMPLT_ERR_MASK_MASK            (0x00010000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_UNEXP_CMPLT_ERR_MASK_SHIFT           (0x00000010U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_UNEXP_CMPLT_ERR_MASK_MAX             (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_REC_OVERFLOW_ERR_MASK_MASK           (0x00020000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_REC_OVERFLOW_ERR_MASK_SHIFT          (0x00000011U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_REC_OVERFLOW_ERR_MASK_MAX            (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_MALF_TLP_ERR_MASK_MASK               (0x00040000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_MALF_TLP_ERR_MASK_SHIFT              (0x00000012U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_MALF_TLP_ERR_MASK_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_ECRC_ERR_MASK_MASK                   (0x00080000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_ECRC_ERR_MASK_SHIFT                  (0x00000013U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_ECRC_ERR_MASK_MAX                    (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_UNSUPPORTED_REQ_ERR_MASK_MASK        (0x00100000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_UNSUPPORTED_REQ_ERR_MASK_SHIFT       (0x00000014U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_UNSUPPORTED_REQ_ERR_MASK_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_INTERNAL_ERR_MASK_MASK               (0x00400000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_INTERNAL_ERR_MASK_SHIFT              (0x00000016U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_INTERNAL_ERR_MASK_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_RSVDP_23_MASK                        (0x00800000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_RSVDP_23_SHIFT                       (0x00000017U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_RSVDP_23_MAX                         (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_ATOMIC_EGRESS_BLOCKED_ERR_MASK_MASK  (0x01000000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_ATOMIC_EGRESS_BLOCKED_ERR_MASK_SHIFT (0x00000018U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_ATOMIC_EGRESS_BLOCKED_ERR_MASK_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_TLP_PRFX_BLOCKED_ERR_MASK_MASK       (0x02000000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_TLP_PRFX_BLOCKED_ERR_MASK_SHIFT      (0x00000019U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_TLP_PRFX_BLOCKED_ERR_MASK_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_RSVDP_26_MASK                        (0xFC000000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_RSVDP_26_SHIFT                       (0x0000001AU)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_MASK_OFF_RSVDP_26_MAX                         (0x0000003FU)

/* UNCORR_ERR_SEV_OFF */

#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_RSVDP_0_MASK                          (0x0000000FU)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_RSVDP_0_SHIFT                         (0x00000000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_RSVDP_0_MAX                           (0x0000000FU)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_DL_PROTOCOL_ERR_SEVERITY_MASK         (0x00000010U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_DL_PROTOCOL_ERR_SEVERITY_SHIFT        (0x00000004U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_DL_PROTOCOL_ERR_SEVERITY_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_SURPRISE_DOWN_ERR_SVRITY_MASK         (0x00000020U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_SURPRISE_DOWN_ERR_SVRITY_SHIFT        (0x00000005U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_SURPRISE_DOWN_ERR_SVRITY_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_RSVDP_6_MASK                          (0x00000FC0U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_RSVDP_6_SHIFT                         (0x00000006U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_RSVDP_6_MAX                           (0x0000003FU)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_POIS_TLP_ERR_SEVERITY_MASK            (0x00001000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_POIS_TLP_ERR_SEVERITY_SHIFT           (0x0000000CU)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_POIS_TLP_ERR_SEVERITY_MAX             (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_FC_PROTOCOL_ERR_SEVERITY_MASK         (0x00002000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_FC_PROTOCOL_ERR_SEVERITY_SHIFT        (0x0000000DU)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_FC_PROTOCOL_ERR_SEVERITY_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_CMPLT_TIMEOUT_ERR_SEVERITY_MASK       (0x00004000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_CMPLT_TIMEOUT_ERR_SEVERITY_SHIFT      (0x0000000EU)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_CMPLT_TIMEOUT_ERR_SEVERITY_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_CMPLT_ABORT_ERR_SEVERITY_MASK         (0x00008000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_CMPLT_ABORT_ERR_SEVERITY_SHIFT        (0x0000000FU)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_CMPLT_ABORT_ERR_SEVERITY_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_UNEXP_CMPLT_ERR_SEVERITY_MASK         (0x00010000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_UNEXP_CMPLT_ERR_SEVERITY_SHIFT        (0x00000010U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_UNEXP_CMPLT_ERR_SEVERITY_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_REC_OVERFLOW_ERR_SEVERITY_MASK        (0x00020000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_REC_OVERFLOW_ERR_SEVERITY_SHIFT       (0x00000011U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_REC_OVERFLOW_ERR_SEVERITY_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_MALF_TLP_ERR_SEVERITY_MASK            (0x00040000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_MALF_TLP_ERR_SEVERITY_SHIFT           (0x00000012U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_MALF_TLP_ERR_SEVERITY_MAX             (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_ECRC_ERR_SEVERITY_MASK                (0x00080000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_ECRC_ERR_SEVERITY_SHIFT               (0x00000013U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_ECRC_ERR_SEVERITY_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_UNSUPPORTED_REQ_ERR_SEVERITY_MASK     (0x00100000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_UNSUPPORTED_REQ_ERR_SEVERITY_SHIFT    (0x00000014U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_UNSUPPORTED_REQ_ERR_SEVERITY_MAX      (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_INTERNAL_ERR_SEVERITY_MASK            (0x00400000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_INTERNAL_ERR_SEVERITY_SHIFT           (0x00000016U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_INTERNAL_ERR_SEVERITY_MAX             (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_RSVDP_23_MASK                         (0x00800000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_RSVDP_23_SHIFT                        (0x00000017U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_RSVDP_23_MAX                          (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_ATOMIC_EGRESS_BLOCKED_ERR_SEVERITY_MASK (0x01000000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_ATOMIC_EGRESS_BLOCKED_ERR_SEVERITY_SHIFT (0x00000018U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_ATOMIC_EGRESS_BLOCKED_ERR_SEVERITY_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_TLP_PRFX_BLOCKED_ERR_SEVERITY_MASK    (0x02000000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_TLP_PRFX_BLOCKED_ERR_SEVERITY_SHIFT   (0x00000019U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_TLP_PRFX_BLOCKED_ERR_SEVERITY_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_RSVDP_26_MASK                         (0xFC000000U)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_RSVDP_26_SHIFT                        (0x0000001AU)
#define CSL_PCIE_RC_CORE_UNCORR_ERR_SEV_OFF_RSVDP_26_MAX                          (0x0000003FU)

/* CORR_ERR_STATUS_OFF */

#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_RX_ERR_STATUS_MASK                   (0x00000001U)
#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_RX_ERR_STATUS_SHIFT                  (0x00000000U)
#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_RX_ERR_STATUS_MAX                    (0x00000001U)

#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_RSVDP_1_MASK                         (0x0000003EU)
#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_RSVDP_1_SHIFT                        (0x00000001U)
#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_RSVDP_1_MAX                          (0x0000001FU)

#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_BAD_TLP_STATUS_MASK                  (0x00000040U)
#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_BAD_TLP_STATUS_SHIFT                 (0x00000006U)
#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_BAD_TLP_STATUS_MAX                   (0x00000001U)

#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_BAD_DLLP_STATUS_MASK                 (0x00000080U)
#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_BAD_DLLP_STATUS_SHIFT                (0x00000007U)
#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_BAD_DLLP_STATUS_MAX                  (0x00000001U)

#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_REPLAY_NO_ROLEOVER_STATUS_MASK       (0x00000100U)
#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_REPLAY_NO_ROLEOVER_STATUS_SHIFT      (0x00000008U)
#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_REPLAY_NO_ROLEOVER_STATUS_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_RSVDP_9_MASK                         (0x00000E00U)
#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_RSVDP_9_SHIFT                        (0x00000009U)
#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_RSVDP_9_MAX                          (0x00000007U)

#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_RPL_TIMER_TIMEOUT_STATUS_MASK        (0x00001000U)
#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_RPL_TIMER_TIMEOUT_STATUS_SHIFT       (0x0000000CU)
#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_RPL_TIMER_TIMEOUT_STATUS_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_ADVISORY_NON_FATAL_ERR_STATUS_MASK   (0x00002000U)
#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_ADVISORY_NON_FATAL_ERR_STATUS_SHIFT  (0x0000000DU)
#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_ADVISORY_NON_FATAL_ERR_STATUS_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_CORRECTED_INT_ERR_STATUS_MASK        (0x00004000U)
#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_CORRECTED_INT_ERR_STATUS_SHIFT       (0x0000000EU)
#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_CORRECTED_INT_ERR_STATUS_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_HEADER_LOG_OVERFLOW_STATUS_MASK      (0x00008000U)
#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_HEADER_LOG_OVERFLOW_STATUS_SHIFT     (0x0000000FU)
#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_HEADER_LOG_OVERFLOW_STATUS_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_RSVDP_16_MASK                        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_RSVDP_16_SHIFT                       (0x00000010U)
#define CSL_PCIE_RC_CORE_CORR_ERR_STATUS_OFF_RSVDP_16_MAX                         (0x0000FFFFU)

/* CORR_ERR_MASK_OFF */

#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_RX_ERR_MASK_MASK                       (0x00000001U)
#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_RX_ERR_MASK_SHIFT                      (0x00000000U)
#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_RX_ERR_MASK_MAX                        (0x00000001U)

#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_RSVDP_1_MASK                           (0x0000003EU)
#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_RSVDP_1_SHIFT                          (0x00000001U)
#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_RSVDP_1_MAX                            (0x0000001FU)

#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_BAD_TLP_MASK_MASK                      (0x00000040U)
#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_BAD_TLP_MASK_SHIFT                     (0x00000006U)
#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_BAD_TLP_MASK_MAX                       (0x00000001U)

#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_BAD_DLLP_MASK_MASK                     (0x00000080U)
#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_BAD_DLLP_MASK_SHIFT                    (0x00000007U)
#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_BAD_DLLP_MASK_MAX                      (0x00000001U)

#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_REPLAY_NO_ROLEOVER_MASK_MASK           (0x00000100U)
#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_REPLAY_NO_ROLEOVER_MASK_SHIFT          (0x00000008U)
#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_REPLAY_NO_ROLEOVER_MASK_MAX            (0x00000001U)

#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_RSVDP_9_MASK                           (0x00000E00U)
#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_RSVDP_9_SHIFT                          (0x00000009U)
#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_RSVDP_9_MAX                            (0x00000007U)

#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_RPL_TIMER_TIMEOUT_MASK_MASK            (0x00001000U)
#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_RPL_TIMER_TIMEOUT_MASK_SHIFT           (0x0000000CU)
#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_RPL_TIMER_TIMEOUT_MASK_MAX             (0x00000001U)

#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_ADVISORY_NON_FATAL_ERR_MASK_MASK       (0x00002000U)
#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_ADVISORY_NON_FATAL_ERR_MASK_SHIFT      (0x0000000DU)
#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_ADVISORY_NON_FATAL_ERR_MASK_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_CORRECTED_INT_ERR_MASK_MASK            (0x00004000U)
#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_CORRECTED_INT_ERR_MASK_SHIFT           (0x0000000EU)
#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_CORRECTED_INT_ERR_MASK_MAX             (0x00000001U)

#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_HEADER_LOG_OVERFLOW_MASK_MASK          (0x00008000U)
#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_HEADER_LOG_OVERFLOW_MASK_SHIFT         (0x0000000FU)
#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_HEADER_LOG_OVERFLOW_MASK_MAX           (0x00000001U)

#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_RSVDP_16_MASK                          (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_RSVDP_16_SHIFT                         (0x00000010U)
#define CSL_PCIE_RC_CORE_CORR_ERR_MASK_OFF_RSVDP_16_MAX                           (0x0000FFFFU)

/* ADV_ERR_CAP_CTRL_OFF */

#define CSL_PCIE_RC_CORE_ADV_ERR_CAP_CTRL_OFF_FIRST_ERR_POINTER_MASK              (0x0000001FU)
#define CSL_PCIE_RC_CORE_ADV_ERR_CAP_CTRL_OFF_FIRST_ERR_POINTER_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_ADV_ERR_CAP_CTRL_OFF_FIRST_ERR_POINTER_MAX               (0x0000001FU)

#define CSL_PCIE_RC_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_GEN_CAP_MASK                   (0x00000020U)
#define CSL_PCIE_RC_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_GEN_CAP_SHIFT                  (0x00000005U)
#define CSL_PCIE_RC_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_GEN_CAP_MAX                    (0x00000001U)

#define CSL_PCIE_RC_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_GEN_EN_MASK                    (0x00000040U)
#define CSL_PCIE_RC_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_GEN_EN_SHIFT                   (0x00000006U)
#define CSL_PCIE_RC_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_GEN_EN_MAX                     (0x00000001U)

#define CSL_PCIE_RC_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_CHECK_CAP_MASK                 (0x00000080U)
#define CSL_PCIE_RC_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_CHECK_CAP_SHIFT                (0x00000007U)
#define CSL_PCIE_RC_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_CHECK_CAP_MAX                  (0x00000001U)

#define CSL_PCIE_RC_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_CHECK_EN_MASK                  (0x00000100U)
#define CSL_PCIE_RC_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_CHECK_EN_SHIFT                 (0x00000008U)
#define CSL_PCIE_RC_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_CHECK_EN_MAX                   (0x00000001U)

#define CSL_PCIE_RC_CORE_ADV_ERR_CAP_CTRL_OFF_MULTIPLE_HEADER_CAP_MASK            (0x00000200U)
#define CSL_PCIE_RC_CORE_ADV_ERR_CAP_CTRL_OFF_MULTIPLE_HEADER_CAP_SHIFT           (0x00000009U)
#define CSL_PCIE_RC_CORE_ADV_ERR_CAP_CTRL_OFF_MULTIPLE_HEADER_CAP_MAX             (0x00000001U)

#define CSL_PCIE_RC_CORE_ADV_ERR_CAP_CTRL_OFF_MULTIPLE_HEADER_EN_MASK             (0x00000400U)
#define CSL_PCIE_RC_CORE_ADV_ERR_CAP_CTRL_OFF_MULTIPLE_HEADER_EN_SHIFT            (0x0000000AU)
#define CSL_PCIE_RC_CORE_ADV_ERR_CAP_CTRL_OFF_MULTIPLE_HEADER_EN_MAX              (0x00000001U)

#define CSL_PCIE_RC_CORE_ADV_ERR_CAP_CTRL_OFF_RSVDP_12_MASK                       (0xFFFFF000U)
#define CSL_PCIE_RC_CORE_ADV_ERR_CAP_CTRL_OFF_RSVDP_12_SHIFT                      (0x0000000CU)
#define CSL_PCIE_RC_CORE_ADV_ERR_CAP_CTRL_OFF_RSVDP_12_MAX                        (0x000FFFFFU)

#ifdef CSL_MODIFICATION
/* HDR_LOG_0_OFF */

#define CSL_PCIE_RC_CORE_HDR_LOG_0_OFF_FIRST_DWORD_FIRST_BYTE_MASK                (0x000000FFU)
#define CSL_PCIE_RC_CORE_HDR_LOG_0_OFF_FIRST_DWORD_FIRST_BYTE_SHIFT               (0x00000000U)
#define CSL_PCIE_RC_CORE_HDR_LOG_0_OFF_FIRST_DWORD_FIRST_BYTE_MAX                 (0x000000FFU)

#define CSL_PCIE_RC_CORE_HDR_LOG_0_OFF_FIRST_DWORD_SECOND_BYTE_MASK               (0x0000FF00U)
#define CSL_PCIE_RC_CORE_HDR_LOG_0_OFF_FIRST_DWORD_SECOND_BYTE_SHIFT              (0x00000008U)
#define CSL_PCIE_RC_CORE_HDR_LOG_0_OFF_FIRST_DWORD_SECOND_BYTE_MAX                (0x000000FFU)

#define CSL_PCIE_RC_CORE_HDR_LOG_0_OFF_FIRST_DWORD_THIRD_BYTE_MASK                (0x00FF0000U)
#define CSL_PCIE_RC_CORE_HDR_LOG_0_OFF_FIRST_DWORD_THIRD_BYTE_SHIFT               (0x00000010U)
#define CSL_PCIE_RC_CORE_HDR_LOG_0_OFF_FIRST_DWORD_THIRD_BYTE_MAX                 (0x000000FFU)

#define CSL_PCIE_RC_CORE_HDR_LOG_0_OFF_FIRST_DWORD_FOURTH_BYTE_MASK               (0xFF000000U)
#define CSL_PCIE_RC_CORE_HDR_LOG_0_OFF_FIRST_DWORD_FOURTH_BYTE_SHIFT              (0x00000018U)
#define CSL_PCIE_RC_CORE_HDR_LOG_0_OFF_FIRST_DWORD_FOURTH_BYTE_MAX                (0x000000FFU)

/* HDR_LOG_1_OFF */

#define CSL_PCIE_RC_CORE_HDR_LOG_1_OFF_SECOND_DWORD_FIRST_BYTE_MASK               (0x000000FFU)
#define CSL_PCIE_RC_CORE_HDR_LOG_1_OFF_SECOND_DWORD_FIRST_BYTE_SHIFT              (0x00000000U)
#define CSL_PCIE_RC_CORE_HDR_LOG_1_OFF_SECOND_DWORD_FIRST_BYTE_MAX                (0x000000FFU)

#define CSL_PCIE_RC_CORE_HDR_LOG_1_OFF_SECOND_DWORD_SECOND_BYTE_MASK              (0x0000FF00U)
#define CSL_PCIE_RC_CORE_HDR_LOG_1_OFF_SECOND_DWORD_SECOND_BYTE_SHIFT             (0x00000008U)
#define CSL_PCIE_RC_CORE_HDR_LOG_1_OFF_SECOND_DWORD_SECOND_BYTE_MAX               (0x000000FFU)

#define CSL_PCIE_RC_CORE_HDR_LOG_1_OFF_SECOND_DWORD_THIRD_BYTE_MASK               (0x00FF0000U)
#define CSL_PCIE_RC_CORE_HDR_LOG_1_OFF_SECOND_DWORD_THIRD_BYTE_SHIFT              (0x00000010U)
#define CSL_PCIE_RC_CORE_HDR_LOG_1_OFF_SECOND_DWORD_THIRD_BYTE_MAX                (0x000000FFU)

#define CSL_PCIE_RC_CORE_HDR_LOG_1_OFF_SECOND_DWORD_FOURTH_BYTE_MASK              (0xFF000000U)
#define CSL_PCIE_RC_CORE_HDR_LOG_1_OFF_SECOND_DWORD_FOURTH_BYTE_SHIFT             (0x00000018U)
#define CSL_PCIE_RC_CORE_HDR_LOG_1_OFF_SECOND_DWORD_FOURTH_BYTE_MAX               (0x000000FFU)

/* HDR_LOG_2_OFF */

#define CSL_PCIE_RC_CORE_HDR_LOG_2_OFF_THIRD_DWORD_FIRST_BYTE_MASK                (0x000000FFU)
#define CSL_PCIE_RC_CORE_HDR_LOG_2_OFF_THIRD_DWORD_FIRST_BYTE_SHIFT               (0x00000000U)
#define CSL_PCIE_RC_CORE_HDR_LOG_2_OFF_THIRD_DWORD_FIRST_BYTE_MAX                 (0x000000FFU)

#define CSL_PCIE_RC_CORE_HDR_LOG_2_OFF_THIRD_DWORD_SECOND_BYTE_MASK               (0x0000FF00U)
#define CSL_PCIE_RC_CORE_HDR_LOG_2_OFF_THIRD_DWORD_SECOND_BYTE_SHIFT              (0x00000008U)
#define CSL_PCIE_RC_CORE_HDR_LOG_2_OFF_THIRD_DWORD_SECOND_BYTE_MAX                (0x000000FFU)

#define CSL_PCIE_RC_CORE_HDR_LOG_2_OFF_THIRD_DWORD_THIRD_BYTE_MASK                (0x00FF0000U)
#define CSL_PCIE_RC_CORE_HDR_LOG_2_OFF_THIRD_DWORD_THIRD_BYTE_SHIFT               (0x00000010U)
#define CSL_PCIE_RC_CORE_HDR_LOG_2_OFF_THIRD_DWORD_THIRD_BYTE_MAX                 (0x000000FFU)

#define CSL_PCIE_RC_CORE_HDR_LOG_2_OFF_THIRD_DWORD_FOURTH_BYTE_MASK               (0xFF000000U)
#define CSL_PCIE_RC_CORE_HDR_LOG_2_OFF_THIRD_DWORD_FOURTH_BYTE_SHIFT              (0x00000018U)
#define CSL_PCIE_RC_CORE_HDR_LOG_2_OFF_THIRD_DWORD_FOURTH_BYTE_MAX                (0x000000FFU)

/* HDR_LOG_3_OFF */

#define CSL_PCIE_RC_CORE_HDR_LOG_3_OFF_FOURTH_DWORD_FIRST_BYTE_MASK               (0x000000FFU)
#define CSL_PCIE_RC_CORE_HDR_LOG_3_OFF_FOURTH_DWORD_FIRST_BYTE_SHIFT              (0x00000000U)
#define CSL_PCIE_RC_CORE_HDR_LOG_3_OFF_FOURTH_DWORD_FIRST_BYTE_MAX                (0x000000FFU)

#define CSL_PCIE_RC_CORE_HDR_LOG_3_OFF_FOURTH_DWORD_SECOND_BYTE_MASK              (0x0000FF00U)
#define CSL_PCIE_RC_CORE_HDR_LOG_3_OFF_FOURTH_DWORD_SECOND_BYTE_SHIFT             (0x00000008U)
#define CSL_PCIE_RC_CORE_HDR_LOG_3_OFF_FOURTH_DWORD_SECOND_BYTE_MAX               (0x000000FFU)

#define CSL_PCIE_RC_CORE_HDR_LOG_3_OFF_FOURTH_DWORD_THIRD_BYTE_MASK               (0x00FF0000U)
#define CSL_PCIE_RC_CORE_HDR_LOG_3_OFF_FOURTH_DWORD_THIRD_BYTE_SHIFT              (0x00000010U)
#define CSL_PCIE_RC_CORE_HDR_LOG_3_OFF_FOURTH_DWORD_THIRD_BYTE_MAX                (0x000000FFU)

#define CSL_PCIE_RC_CORE_HDR_LOG_3_OFF_FOURTH_DWORD_FOURTH_BYTE_MASK              (0xFF000000U)
#define CSL_PCIE_RC_CORE_HDR_LOG_3_OFF_FOURTH_DWORD_FOURTH_BYTE_SHIFT             (0x00000018U)
#define CSL_PCIE_RC_CORE_HDR_LOG_3_OFF_FOURTH_DWORD_FOURTH_BYTE_MAX               (0x000000FFU)

#else /* CSL_MODIFICATION */

#define CSL_PCIE_RC_CORE_HDR_LOG_OFF_FIRST_DWORD_FIRST_BYTE_MASK                  (0x000000FFU)
#define CSL_PCIE_RC_CORE_HDR_LOG_OFF_FIRST_DWORD_FIRST_BYTE_SHIFT                 (0x00000000U)
#define CSL_PCIE_RC_CORE_HDR_LOG_OFF_FIRST_DWORD_FIRST_BYTE_MAX                   (0x000000FFU)

#define CSL_PCIE_RC_CORE_HDR_LOG_OFF_FIRST_DWORD_SECOND_BYTE_MASK                 (0x0000FF00U)
#define CSL_PCIE_RC_CORE_HDR_LOG_OFF_FIRST_DWORD_SECOND_BYTE_SHIFT                (0x00000008U)
#define CSL_PCIE_RC_CORE_HDR_LOG_OFF_FIRST_DWORD_SECOND_BYTE_MAX                  (0x000000FFU)

#define CSL_PCIE_RC_CORE_HDR_LOG_OFF_FIRST_DWORD_THIRD_BYTE_MASK                  (0x00FF0000U)
#define CSL_PCIE_RC_CORE_HDR_LOG_OFF_FIRST_DWORD_THIRD_BYTE_SHIFT                 (0x00000010U)
#define CSL_PCIE_RC_CORE_HDR_LOG_OFF_FIRST_DWORD_THIRD_BYTE_MAX                   (0x000000FFU)

#define CSL_PCIE_RC_CORE_HDR_LOG_OFF_FIRST_DWORD_FOURTH_BYTE_MASK                 (0xFF000000U)
#define CSL_PCIE_RC_CORE_HDR_LOG_OFF_FIRST_DWORD_FOURTH_BYTE_SHIFT                (0x00000018U)
#define CSL_PCIE_RC_CORE_HDR_LOG_OFF_FIRST_DWORD_FOURTH_BYTE_MAX                  (0x000000FFU)

#endif /* CSL_MODIFICATION */

/* ROOT_ERR_CMD_OFF */

#define CSL_PCIE_RC_CORE_ROOT_ERR_CMD_OFF_CORR_ERR_REPORTING_EN_MASK              (0x00000001U)
#define CSL_PCIE_RC_CORE_ROOT_ERR_CMD_OFF_CORR_ERR_REPORTING_EN_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_ROOT_ERR_CMD_OFF_CORR_ERR_REPORTING_EN_MAX               (0x00000001U)

#define CSL_PCIE_RC_CORE_ROOT_ERR_CMD_OFF_NON_FATAL_ERR_REPORTING_EN_MASK         (0x00000002U)
#define CSL_PCIE_RC_CORE_ROOT_ERR_CMD_OFF_NON_FATAL_ERR_REPORTING_EN_SHIFT        (0x00000001U)
#define CSL_PCIE_RC_CORE_ROOT_ERR_CMD_OFF_NON_FATAL_ERR_REPORTING_EN_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_ROOT_ERR_CMD_OFF_FATAL_ERR_REPORTING_EN_MASK             (0x00000004U)
#define CSL_PCIE_RC_CORE_ROOT_ERR_CMD_OFF_FATAL_ERR_REPORTING_EN_SHIFT            (0x00000002U)
#define CSL_PCIE_RC_CORE_ROOT_ERR_CMD_OFF_FATAL_ERR_REPORTING_EN_MAX              (0x00000001U)

#define CSL_PCIE_RC_CORE_ROOT_ERR_CMD_OFF_RSVDP_3_MASK                            (0xFFFFFFF8U)
#define CSL_PCIE_RC_CORE_ROOT_ERR_CMD_OFF_RSVDP_3_SHIFT                           (0x00000003U)
#define CSL_PCIE_RC_CORE_ROOT_ERR_CMD_OFF_RSVDP_3_MAX                             (0x1FFFFFFFU)

/* ROOT_ERR_STATUS_OFF */

#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_ERR_COR_RX_MASK                      (0x00000001U)
#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_ERR_COR_RX_SHIFT                     (0x00000000U)
#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_ERR_COR_RX_MAX                       (0x00000001U)

#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_MUL_ERR_COR_RX_MASK                  (0x00000002U)
#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_MUL_ERR_COR_RX_SHIFT                 (0x00000001U)
#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_MUL_ERR_COR_RX_MAX                   (0x00000001U)

#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_ERR_FATAL_NON_FATAL_RX_MASK          (0x00000004U)
#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_ERR_FATAL_NON_FATAL_RX_SHIFT         (0x00000002U)
#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_ERR_FATAL_NON_FATAL_RX_MAX           (0x00000001U)

#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_MUL_ERR_FATAL_NON_FATAL_RX_MASK      (0x00000008U)
#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_MUL_ERR_FATAL_NON_FATAL_RX_SHIFT     (0x00000003U)
#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_MUL_ERR_FATAL_NON_FATAL_RX_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_FIRST_UNCORR_FATAL_MASK              (0x00000010U)
#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_FIRST_UNCORR_FATAL_SHIFT             (0x00000004U)
#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_FIRST_UNCORR_FATAL_MAX               (0x00000001U)

#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_NON_FATAL_ERR_MSG_RX_MASK            (0x00000020U)
#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_NON_FATAL_ERR_MSG_RX_SHIFT           (0x00000005U)
#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_NON_FATAL_ERR_MSG_RX_MAX             (0x00000001U)

#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_FATAL_ERR_MSG_RX_MASK                (0x00000040U)
#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_FATAL_ERR_MSG_RX_SHIFT               (0x00000006U)
#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_FATAL_ERR_MSG_RX_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_RSVDP_7_MASK                         (0x07FFFF80U)
#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_RSVDP_7_SHIFT                        (0x00000007U)
#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_RSVDP_7_MAX                          (0x000FFFFFU)

#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_ADV_ERR_INT_MSG_NUM_MASK             (0xF8000000U)
#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_ADV_ERR_INT_MSG_NUM_SHIFT            (0x0000001BU)
#define CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_ADV_ERR_INT_MSG_NUM_MAX              (0x0000001FU)

/* ERR_SRC_ID_OFF */

#define CSL_PCIE_RC_CORE_ERR_SRC_ID_OFF_ERR_COR_SOURCE_ID_MASK                    (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_ERR_SRC_ID_OFF_ERR_COR_SOURCE_ID_SHIFT                   (0x00000000U)
#define CSL_PCIE_RC_CORE_ERR_SRC_ID_OFF_ERR_COR_SOURCE_ID_MAX                     (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_ERR_SRC_ID_OFF_ERR_FATAL_NON_FATAL_SOURCE_ID_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_ERR_SRC_ID_OFF_ERR_FATAL_NON_FATAL_SOURCE_ID_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_ERR_SRC_ID_OFF_ERR_FATAL_NON_FATAL_SOURCE_ID_MAX         (0x0000FFFFU)

/* TLP_PREFIX_LOG_1_OFF */

#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_1_OFF_CFG_TLP_PFX_LOG_1_FIRST_BYTE_MASK   (0x000000FFU)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_1_OFF_CFG_TLP_PFX_LOG_1_FIRST_BYTE_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_1_OFF_CFG_TLP_PFX_LOG_1_FIRST_BYTE_MAX    (0x000000FFU)

#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_1_OFF_CFG_TLP_PFX_LOG_1_SECOND_BYTE_MASK  (0x0000FF00U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_1_OFF_CFG_TLP_PFX_LOG_1_SECOND_BYTE_SHIFT (0x00000008U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_1_OFF_CFG_TLP_PFX_LOG_1_SECOND_BYTE_MAX   (0x000000FFU)

#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_1_OFF_CFG_TLP_PFX_LOG_1_THIRD_BYTE_MASK   (0x00FF0000U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_1_OFF_CFG_TLP_PFX_LOG_1_THIRD_BYTE_SHIFT  (0x00000010U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_1_OFF_CFG_TLP_PFX_LOG_1_THIRD_BYTE_MAX    (0x000000FFU)

#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_1_OFF_CFG_TLP_PFX_LOG_1_FOURTH_BYTE_MASK  (0xFF000000U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_1_OFF_CFG_TLP_PFX_LOG_1_FOURTH_BYTE_SHIFT (0x00000018U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_1_OFF_CFG_TLP_PFX_LOG_1_FOURTH_BYTE_MAX   (0x000000FFU)

/* TLP_PREFIX_LOG_2_OFF */

#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_2_OFF_CFG_TLP_PFX_LOG_2_FIRST_BYTE_MASK   (0x000000FFU)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_2_OFF_CFG_TLP_PFX_LOG_2_FIRST_BYTE_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_2_OFF_CFG_TLP_PFX_LOG_2_FIRST_BYTE_MAX    (0x000000FFU)

#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_2_OFF_CFG_TLP_PFX_LOG_2_SECOND_BYTE_MASK  (0x0000FF00U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_2_OFF_CFG_TLP_PFX_LOG_2_SECOND_BYTE_SHIFT (0x00000008U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_2_OFF_CFG_TLP_PFX_LOG_2_SECOND_BYTE_MAX   (0x000000FFU)

#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_2_OFF_CFG_TLP_PFX_LOG_2_THIRD_BYTE_MASK   (0x00FF0000U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_2_OFF_CFG_TLP_PFX_LOG_2_THIRD_BYTE_SHIFT  (0x00000010U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_2_OFF_CFG_TLP_PFX_LOG_2_THIRD_BYTE_MAX    (0x000000FFU)

#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_2_OFF_CFG_TLP_PFX_LOG_2_FOURTH_BYTE_MASK  (0xFF000000U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_2_OFF_CFG_TLP_PFX_LOG_2_FOURTH_BYTE_SHIFT (0x00000018U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_2_OFF_CFG_TLP_PFX_LOG_2_FOURTH_BYTE_MAX   (0x000000FFU)

/* TLP_PREFIX_LOG_3_OFF */

#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_3_OFF_CFG_TLP_PFX_LOG_3_FIRST_BYTE_MASK   (0x000000FFU)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_3_OFF_CFG_TLP_PFX_LOG_3_FIRST_BYTE_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_3_OFF_CFG_TLP_PFX_LOG_3_FIRST_BYTE_MAX    (0x000000FFU)

#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_3_OFF_CFG_TLP_PFX_LOG_3_SECOND_BYTE_MASK  (0x0000FF00U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_3_OFF_CFG_TLP_PFX_LOG_3_SECOND_BYTE_SHIFT (0x00000008U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_3_OFF_CFG_TLP_PFX_LOG_3_SECOND_BYTE_MAX   (0x000000FFU)

#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_3_OFF_CFG_TLP_PFX_LOG_3_THIRD_BYTE_MASK   (0x00FF0000U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_3_OFF_CFG_TLP_PFX_LOG_3_THIRD_BYTE_SHIFT  (0x00000010U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_3_OFF_CFG_TLP_PFX_LOG_3_THIRD_BYTE_MAX    (0x000000FFU)

#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_3_OFF_CFG_TLP_PFX_LOG_3_FOURTH_BYTE_MASK  (0xFF000000U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_3_OFF_CFG_TLP_PFX_LOG_3_FOURTH_BYTE_SHIFT (0x00000018U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_3_OFF_CFG_TLP_PFX_LOG_3_FOURTH_BYTE_MAX   (0x000000FFU)

/* TLP_PREFIX_LOG_4_OFF */

#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_4_OFF_CFG_TLP_PFX_LOG_4_FIRST_BYTE_MASK   (0x000000FFU)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_4_OFF_CFG_TLP_PFX_LOG_4_FIRST_BYTE_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_4_OFF_CFG_TLP_PFX_LOG_4_FIRST_BYTE_MAX    (0x000000FFU)

#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_4_OFF_CFG_TLP_PFX_LOG_4_SECOND_BYTE_MASK  (0x0000FF00U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_4_OFF_CFG_TLP_PFX_LOG_4_SECOND_BYTE_SHIFT (0x00000008U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_4_OFF_CFG_TLP_PFX_LOG_4_SECOND_BYTE_MAX   (0x000000FFU)

#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_4_OFF_CFG_TLP_PFX_LOG_4_THIRD_BYTE_MASK   (0x00FF0000U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_4_OFF_CFG_TLP_PFX_LOG_4_THIRD_BYTE_SHIFT  (0x00000010U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_4_OFF_CFG_TLP_PFX_LOG_4_THIRD_BYTE_MAX    (0x000000FFU)

#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_4_OFF_CFG_TLP_PFX_LOG_4_FOURTH_BYTE_MASK  (0xFF000000U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_4_OFF_CFG_TLP_PFX_LOG_4_FOURTH_BYTE_SHIFT (0x00000018U)
#define CSL_PCIE_RC_CORE_TLP_PREFIX_LOG_4_OFF_CFG_TLP_PFX_LOG_4_FOURTH_BYTE_MAX   (0x000000FFU)

/* VC_BASE */

#define CSL_PCIE_RC_CORE_VC_BASE_VC_PCIE_EXTENDED_CAP_ID_MASK                     (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_VC_BASE_VC_PCIE_EXTENDED_CAP_ID_SHIFT                    (0x00000000U)
#define CSL_PCIE_RC_CORE_VC_BASE_VC_PCIE_EXTENDED_CAP_ID_MAX                      (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_VC_BASE_VC_CAP_VERSION_MASK                              (0x000F0000U)
#define CSL_PCIE_RC_CORE_VC_BASE_VC_CAP_VERSION_SHIFT                             (0x00000010U)
#define CSL_PCIE_RC_CORE_VC_BASE_VC_CAP_VERSION_MAX                               (0x0000000FU)

#define CSL_PCIE_RC_CORE_VC_BASE_VC_NEXT_OFFSET_MASK                              (0xFFF00000U)
#define CSL_PCIE_RC_CORE_VC_BASE_VC_NEXT_OFFSET_SHIFT                             (0x00000014U)
#define CSL_PCIE_RC_CORE_VC_BASE_VC_NEXT_OFFSET_MAX                               (0x00000FFFU)

/* VC_CAPABILITIES_REG_1 */

#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_1_VC_EXT_VC_CNT_MASK                 (0x00000007U)
#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_1_VC_EXT_VC_CNT_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_1_VC_EXT_VC_CNT_MAX                  (0x00000007U)

#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_1_RSVDP_3_MASK                       (0x00000008U)
#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_1_RSVDP_3_SHIFT                      (0x00000003U)
#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_1_RSVDP_3_MAX                        (0x00000001U)

#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_1_VC_LOW_PRI_EXT_VC_CNT_MASK         (0x00000070U)
#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_1_VC_LOW_PRI_EXT_VC_CNT_SHIFT        (0x00000004U)
#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_1_VC_LOW_PRI_EXT_VC_CNT_MAX          (0x00000007U)

#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_1_RSVDP_7_MASK                       (0x00000080U)
#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_1_RSVDP_7_SHIFT                      (0x00000007U)
#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_1_RSVDP_7_MAX                        (0x00000001U)

#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_1_VC_REFERENCE_CLOCK_MASK            (0x00000300U)
#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_1_VC_REFERENCE_CLOCK_SHIFT           (0x00000008U)
#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_1_VC_REFERENCE_CLOCK_MAX             (0x00000003U)

#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_1_VC_PORT_ARBI_TBL_ENTRY_SIZE_MASK   (0x00000C00U)
#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_1_VC_PORT_ARBI_TBL_ENTRY_SIZE_SHIFT  (0x0000000AU)
#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_1_VC_PORT_ARBI_TBL_ENTRY_SIZE_MAX    (0x00000003U)

#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_1_RSVDP_12_MASK                      (0xFFFFF000U)
#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_1_RSVDP_12_SHIFT                     (0x0000000CU)
#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_1_RSVDP_12_MAX                       (0x000FFFFFU)

/* VC_CAPABILITIES_REG_2 */

#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_2_VC_ARBI_CAP_MASK                   (0x0000000FU)
#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_2_VC_ARBI_CAP_SHIFT                  (0x00000000U)
#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_2_VC_ARBI_CAP_MAX                    (0x0000000FU)

#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_2_RSVDP_4_MASK                       (0x00FFFFF0U)
#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_2_RSVDP_4_SHIFT                      (0x00000004U)
#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_2_RSVDP_4_MAX                        (0x000FFFFFU)

#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_2_VC_ARBI_TABLE_OFFSET_MASK          (0xFF000000U)
#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_2_VC_ARBI_TABLE_OFFSET_SHIFT         (0x00000018U)
#define CSL_PCIE_RC_CORE_VC_CAPABILITIES_REG_2_VC_ARBI_TABLE_OFFSET_MAX           (0x000000FFU)

/* VC_STATUS_CONTROL_REG */

#define CSL_PCIE_RC_CORE_VC_STATUS_CONTROL_REG_VC_LOAD_VC_ARBI_TABLE_MASK         (0x00000001U)
#define CSL_PCIE_RC_CORE_VC_STATUS_CONTROL_REG_VC_LOAD_VC_ARBI_TABLE_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_VC_STATUS_CONTROL_REG_VC_LOAD_VC_ARBI_TABLE_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_VC_STATUS_CONTROL_REG_VC_ARBI_SELECT_MASK                (0x0000000EU)
#define CSL_PCIE_RC_CORE_VC_STATUS_CONTROL_REG_VC_ARBI_SELECT_SHIFT               (0x00000001U)
#define CSL_PCIE_RC_CORE_VC_STATUS_CONTROL_REG_VC_ARBI_SELECT_MAX                 (0x00000007U)

#define CSL_PCIE_RC_CORE_VC_STATUS_CONTROL_REG_RSVDP_4_MASK                       (0x0000FFF0U)
#define CSL_PCIE_RC_CORE_VC_STATUS_CONTROL_REG_RSVDP_4_SHIFT                      (0x00000004U)
#define CSL_PCIE_RC_CORE_VC_STATUS_CONTROL_REG_RSVDP_4_MAX                        (0x00000FFFU)

#define CSL_PCIE_RC_CORE_VC_STATUS_CONTROL_REG_VC_ARBI_TABLE_STATUS_MASK          (0x00010000U)
#define CSL_PCIE_RC_CORE_VC_STATUS_CONTROL_REG_VC_ARBI_TABLE_STATUS_SHIFT         (0x00000010U)
#define CSL_PCIE_RC_CORE_VC_STATUS_CONTROL_REG_VC_ARBI_TABLE_STATUS_MAX           (0x00000001U)

#define CSL_PCIE_RC_CORE_VC_STATUS_CONTROL_REG_RSVDP_17_MASK                      (0xFFFE0000U)
#define CSL_PCIE_RC_CORE_VC_STATUS_CONTROL_REG_RSVDP_17_SHIFT                     (0x00000011U)
#define CSL_PCIE_RC_CORE_VC_STATUS_CONTROL_REG_RSVDP_17_MAX                       (0x00007FFFU)

/* RESOURCE_CAP_REG_VC0 */

#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC0_VC_PORT_ARBI_CAP_VC0_MASK           (0x000000FFU)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC0_VC_PORT_ARBI_CAP_VC0_SHIFT          (0x00000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC0_VC_PORT_ARBI_CAP_VC0_MAX            (0x000000FFU)

#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC0_RSVDP_8_MASK                        (0x00007F00U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC0_RSVDP_8_SHIFT                       (0x00000008U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC0_RSVDP_8_MAX                         (0x0000007FU)

#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC0_VC_REJECT_SNOOP_TRANS_VC0_MASK      (0x00008000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC0_VC_REJECT_SNOOP_TRANS_VC0_SHIFT     (0x0000000FU)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC0_VC_REJECT_SNOOP_TRANS_VC0_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC0_VC_MAX_TIME_SLOT_VC0_MASK           (0x003F0000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC0_VC_MAX_TIME_SLOT_VC0_SHIFT          (0x00000010U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC0_VC_MAX_TIME_SLOT_VC0_MAX            (0x0000003FU)

#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC0_RSVDP_22_MASK                       (0x00C00000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC0_RSVDP_22_SHIFT                      (0x00000016U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC0_RSVDP_22_MAX                        (0x00000003U)

#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC0_VC_PORT_ARBI_TABLE_VC0_MASK         (0xFF000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC0_VC_PORT_ARBI_TABLE_VC0_SHIFT        (0x00000018U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC0_VC_PORT_ARBI_TABLE_VC0_MAX          (0x000000FFU)

/* RESOURCE_CON_REG_VC0 */

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0_VC_TC_MAP_VC0_MASK                  (0x00000001U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0_VC_TC_MAP_VC0_SHIFT                 (0x00000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0_VC_TC_MAP_VC0_MAX                   (0x00000001U)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0_VC_TC_MAP_VC0_BIT1_MASK             (0x000000FEU)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0_VC_TC_MAP_VC0_BIT1_SHIFT            (0x00000001U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0_VC_TC_MAP_VC0_BIT1_MAX              (0x0000007FU)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0_RSVDP_8_MASK                        (0x0000FF00U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0_RSVDP_8_SHIFT                       (0x00000008U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0_RSVDP_8_MAX                         (0x000000FFU)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0_VC_LOAD_PORT_ARBI_TABLE_VC0_MASK    (0x00010000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0_VC_LOAD_PORT_ARBI_TABLE_VC0_SHIFT   (0x00000010U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0_VC_LOAD_PORT_ARBI_TABLE_VC0_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0_VC_PORT_ARBI_SELECT_VC0_MASK        (0x00020000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0_VC_PORT_ARBI_SELECT_VC0_SHIFT       (0x00000011U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0_VC_PORT_ARBI_SELECT_VC0_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0_RSVDP_18_MASK                       (0x00FC0000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0_RSVDP_18_SHIFT                      (0x00000012U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0_RSVDP_18_MAX                        (0x0000003FU)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0_VC_ID_VC_MASK                       (0x07000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0_VC_ID_VC_SHIFT                      (0x00000018U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0_VC_ID_VC_MAX                        (0x00000007U)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0_RSVDP_27_MASK                       (0x78000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0_RSVDP_27_SHIFT                      (0x0000001BU)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0_RSVDP_27_MAX                        (0x0000000FU)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0_VC_ENABLE_VC0_MASK                  (0x80000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0_VC_ENABLE_VC0_SHIFT                 (0x0000001FU)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC0_VC_ENABLE_VC0_MAX                   (0x00000001U)

/* RESOURCE_STATUS_REG_VC0 */

#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC0_RSVDP_0_MASK                     (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC0_RSVDP_0_SHIFT                    (0x00000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC0_RSVDP_0_MAX                      (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC0_VC_PORT_ARBI_TABLE_STATUS_VC0_MASK (0x00010000U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC0_VC_PORT_ARBI_TABLE_STATUS_VC0_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC0_VC_PORT_ARBI_TABLE_STATUS_VC0_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC0_VC_NEGO_PENDING_VC0_MASK         (0x00020000U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC0_VC_NEGO_PENDING_VC0_SHIFT        (0x00000011U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC0_VC_NEGO_PENDING_VC0_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC0_RSVDP_18_MASK                    (0xFFFC0000U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC0_RSVDP_18_SHIFT                   (0x00000012U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC0_RSVDP_18_MAX                     (0x00003FFFU)

/* RESOURCE_CAP_REG_VC1 */

#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC1_VC_PORT_ARBI_CAP_VC1_MASK           (0x000000FFU)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC1_VC_PORT_ARBI_CAP_VC1_SHIFT          (0x00000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC1_VC_PORT_ARBI_CAP_VC1_MAX            (0x000000FFU)

#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC1_RSVDP_8_MASK                        (0x00007F00U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC1_RSVDP_8_SHIFT                       (0x00000008U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC1_RSVDP_8_MAX                         (0x0000007FU)

#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC1_VC_REJECT_SNOOP_TRANS_VC1_MASK      (0x00008000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC1_VC_REJECT_SNOOP_TRANS_VC1_SHIFT     (0x0000000FU)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC1_VC_REJECT_SNOOP_TRANS_VC1_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC1_VC_MAX_TIME_SLOT_VC1_MASK           (0x003F0000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC1_VC_MAX_TIME_SLOT_VC1_SHIFT          (0x00000010U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC1_VC_MAX_TIME_SLOT_VC1_MAX            (0x0000003FU)

#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC1_RSVDP_22_MASK                       (0x00C00000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC1_RSVDP_22_SHIFT                      (0x00000016U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC1_RSVDP_22_MAX                        (0x00000003U)

#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC1_VC_PORT_ARBI_TABLE_VC1_MASK         (0xFF000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC1_VC_PORT_ARBI_TABLE_VC1_SHIFT        (0x00000018U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC1_VC_PORT_ARBI_TABLE_VC1_MAX          (0x000000FFU)

/* RESOURCE_CON_REG_VC1 */

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1_VC_TC_MAP_VC1_MASK                  (0x00000001U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1_VC_TC_MAP_VC1_SHIFT                 (0x00000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1_VC_TC_MAP_VC1_MAX                   (0x00000001U)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1_VC_TC_MAP_VC1_BIT1_MASK             (0x000000FEU)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1_VC_TC_MAP_VC1_BIT1_SHIFT            (0x00000001U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1_VC_TC_MAP_VC1_BIT1_MAX              (0x0000007FU)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1_RSVDP_8_MASK                        (0x0000FF00U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1_RSVDP_8_SHIFT                       (0x00000008U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1_RSVDP_8_MAX                         (0x000000FFU)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1_VC_LOAD_PORT_ARBI_TABLE_VC1_MASK    (0x00010000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1_VC_LOAD_PORT_ARBI_TABLE_VC1_SHIFT   (0x00000010U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1_VC_LOAD_PORT_ARBI_TABLE_VC1_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1_VC_PORT_ARBI_SELECT_VC1_MASK        (0x000E0000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1_VC_PORT_ARBI_SELECT_VC1_SHIFT       (0x00000011U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1_VC_PORT_ARBI_SELECT_VC1_MAX         (0x00000007U)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1_RSVDP_20_MASK                       (0x00F00000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1_RSVDP_20_SHIFT                      (0x00000014U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1_RSVDP_20_MAX                        (0x0000000FU)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1_VC_ID_VC1_MASK                      (0x07000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1_VC_ID_VC1_SHIFT                     (0x00000018U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1_VC_ID_VC1_MAX                       (0x00000007U)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1_RSVDP_27_MASK                       (0x78000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1_RSVDP_27_SHIFT                      (0x0000001BU)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1_RSVDP_27_MAX                        (0x0000000FU)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1_VC_ENABLE_VC1_MASK                  (0x80000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1_VC_ENABLE_VC1_SHIFT                 (0x0000001FU)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC1_VC_ENABLE_VC1_MAX                   (0x00000001U)

/* RESOURCE_STATUS_REG_VC1 */

#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC1_RSVDP_0_MASK                     (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC1_RSVDP_0_SHIFT                    (0x00000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC1_RSVDP_0_MAX                      (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC1_VC_PORT_ARBI_TABLE_STATUS_VC1_MASK (0x00010000U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC1_VC_PORT_ARBI_TABLE_STATUS_VC1_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC1_VC_PORT_ARBI_TABLE_STATUS_VC1_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC1_VC_NEGO_PENDING_VC1_MASK         (0x00020000U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC1_VC_NEGO_PENDING_VC1_SHIFT        (0x00000011U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC1_VC_NEGO_PENDING_VC1_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC1_RSVDP_18_MASK                    (0xFFFC0000U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC1_RSVDP_18_SHIFT                   (0x00000012U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC1_RSVDP_18_MAX                     (0x00003FFFU)

/* RESOURCE_CAP_REG_VC2 */

#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC2_VC_PORT_ARBI_CAP_VC2_MASK           (0x000000FFU)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC2_VC_PORT_ARBI_CAP_VC2_SHIFT          (0x00000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC2_VC_PORT_ARBI_CAP_VC2_MAX            (0x000000FFU)

#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC2_RSVDP_8_MASK                        (0x00007F00U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC2_RSVDP_8_SHIFT                       (0x00000008U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC2_RSVDP_8_MAX                         (0x0000007FU)

#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC2_VC_REJECT_SNOOP_TRANS_VC2_MASK      (0x00008000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC2_VC_REJECT_SNOOP_TRANS_VC2_SHIFT     (0x0000000FU)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC2_VC_REJECT_SNOOP_TRANS_VC2_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC2_VC_MAX_TIME_SLOT_VC2_MASK           (0x003F0000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC2_VC_MAX_TIME_SLOT_VC2_SHIFT          (0x00000010U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC2_VC_MAX_TIME_SLOT_VC2_MAX            (0x0000003FU)

#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC2_RSVDP_22_MASK                       (0x00C00000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC2_RSVDP_22_SHIFT                      (0x00000016U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC2_RSVDP_22_MAX                        (0x00000003U)

#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC2_VC_PORT_ARBI_TABLE_VC2_MASK         (0xFF000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC2_VC_PORT_ARBI_TABLE_VC2_SHIFT        (0x00000018U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC2_VC_PORT_ARBI_TABLE_VC2_MAX          (0x000000FFU)

/* RESOURCE_CON_REG_VC2 */

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2_VC_TC_MAP_VC2_MASK                  (0x00000001U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2_VC_TC_MAP_VC2_SHIFT                 (0x00000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2_VC_TC_MAP_VC2_MAX                   (0x00000001U)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2_VC_TC_MAP_VC2_BIT1_MASK             (0x000000FEU)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2_VC_TC_MAP_VC2_BIT1_SHIFT            (0x00000001U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2_VC_TC_MAP_VC2_BIT1_MAX              (0x0000007FU)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2_RSVDP_8_MASK                        (0x0000FF00U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2_RSVDP_8_SHIFT                       (0x00000008U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2_RSVDP_8_MAX                         (0x000000FFU)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2_VC_LOAD_PORT_ARBI_TABLE_VC2_MASK    (0x00010000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2_VC_LOAD_PORT_ARBI_TABLE_VC2_SHIFT   (0x00000010U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2_VC_LOAD_PORT_ARBI_TABLE_VC2_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2_VC_PORT_ARBI_SELECT_VC2_MASK        (0x000E0000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2_VC_PORT_ARBI_SELECT_VC2_SHIFT       (0x00000011U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2_VC_PORT_ARBI_SELECT_VC2_MAX         (0x00000007U)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2_RSVDP_20_MASK                       (0x00F00000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2_RSVDP_20_SHIFT                      (0x00000014U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2_RSVDP_20_MAX                        (0x0000000FU)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2_VC_ID_VC2_MASK                      (0x07000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2_VC_ID_VC2_SHIFT                     (0x00000018U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2_VC_ID_VC2_MAX                       (0x00000007U)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2_RSVDP_27_MASK                       (0x78000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2_RSVDP_27_SHIFT                      (0x0000001BU)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2_RSVDP_27_MAX                        (0x0000000FU)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2_VC_ENABLE_VC2_MASK                  (0x80000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2_VC_ENABLE_VC2_SHIFT                 (0x0000001FU)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC2_VC_ENABLE_VC2_MAX                   (0x00000001U)

/* RESOURCE_STATUS_REG_VC2 */

#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC2_RSVDP_0_MASK                     (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC2_RSVDP_0_SHIFT                    (0x00000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC2_RSVDP_0_MAX                      (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC2_VC_PORT_ARBI_TABLE_STATUS_VC2_MASK (0x00010000U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC2_VC_PORT_ARBI_TABLE_STATUS_VC2_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC2_VC_PORT_ARBI_TABLE_STATUS_VC2_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC2_VC_NEGO_PENDING_VC2_MASK         (0x00020000U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC2_VC_NEGO_PENDING_VC2_SHIFT        (0x00000011U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC2_VC_NEGO_PENDING_VC2_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC2_RSVDP_18_MASK                    (0xFFFC0000U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC2_RSVDP_18_SHIFT                   (0x00000012U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC2_RSVDP_18_MAX                     (0x00003FFFU)

/* RESOURCE_CAP_REG_VC3 */

#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC3_VC_PORT_ARBI_CAP_VC3_MASK           (0x000000FFU)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC3_VC_PORT_ARBI_CAP_VC3_SHIFT          (0x00000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC3_VC_PORT_ARBI_CAP_VC3_MAX            (0x000000FFU)

#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC3_RSVDP_8_MASK                        (0x00007F00U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC3_RSVDP_8_SHIFT                       (0x00000008U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC3_RSVDP_8_MAX                         (0x0000007FU)

#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC3_VC_REJECT_SNOOP_TRANS_VC3_MASK      (0x00008000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC3_VC_REJECT_SNOOP_TRANS_VC3_SHIFT     (0x0000000FU)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC3_VC_REJECT_SNOOP_TRANS_VC3_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC3_VC_MAX_TIME_SLOT_VC3_MASK           (0x003F0000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC3_VC_MAX_TIME_SLOT_VC3_SHIFT          (0x00000010U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC3_VC_MAX_TIME_SLOT_VC3_MAX            (0x0000003FU)

#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC3_RSVDP_22_MASK                       (0x00C00000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC3_RSVDP_22_SHIFT                      (0x00000016U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC3_RSVDP_22_MAX                        (0x00000003U)

#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC3_VC_PORT_ARBI_TABLE_VC3_MASK         (0xFF000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC3_VC_PORT_ARBI_TABLE_VC3_SHIFT        (0x00000018U)
#define CSL_PCIE_RC_CORE_RESOURCE_CAP_REG_VC3_VC_PORT_ARBI_TABLE_VC3_MAX          (0x000000FFU)

/* RESOURCE_CON_REG_VC3 */

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3_VC_TC_MAP_VC3_MASK                  (0x00000001U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3_VC_TC_MAP_VC3_SHIFT                 (0x00000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3_VC_TC_MAP_VC3_MAX                   (0x00000001U)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3_VC_TC_MAP_VC3_BIT1_MASK             (0x000000FEU)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3_VC_TC_MAP_VC3_BIT1_SHIFT            (0x00000001U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3_VC_TC_MAP_VC3_BIT1_MAX              (0x0000007FU)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3_RSVDP_8_MASK                        (0x0000FF00U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3_RSVDP_8_SHIFT                       (0x00000008U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3_RSVDP_8_MAX                         (0x000000FFU)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3_VC_LOAD_PORT_ARBI_TABLE_VC3_MASK    (0x00010000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3_VC_LOAD_PORT_ARBI_TABLE_VC3_SHIFT   (0x00000010U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3_VC_LOAD_PORT_ARBI_TABLE_VC3_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3_VC_PORT_ARBI_SELECT_VC3_MASK        (0x000E0000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3_VC_PORT_ARBI_SELECT_VC3_SHIFT       (0x00000011U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3_VC_PORT_ARBI_SELECT_VC3_MAX         (0x00000007U)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3_RSVDP_20_MASK                       (0x00F00000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3_RSVDP_20_SHIFT                      (0x00000014U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3_RSVDP_20_MAX                        (0x0000000FU)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3_VC_ID_VC3_MASK                      (0x07000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3_VC_ID_VC3_SHIFT                     (0x00000018U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3_VC_ID_VC3_MAX                       (0x00000007U)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3_RSVDP_27_MASK                       (0x78000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3_RSVDP_27_SHIFT                      (0x0000001BU)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3_RSVDP_27_MAX                        (0x0000000FU)

#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3_VC_ENABLE_VC3_MASK                  (0x80000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3_VC_ENABLE_VC3_SHIFT                 (0x0000001FU)
#define CSL_PCIE_RC_CORE_RESOURCE_CON_REG_VC3_VC_ENABLE_VC3_MAX                   (0x00000001U)

/* RESOURCE_STATUS_REG_VC3 */

#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC3_RSVDP_0_MASK                     (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC3_RSVDP_0_SHIFT                    (0x00000000U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC3_RSVDP_0_MAX                      (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC3_VC_PORT_ARBI_TABLE_STATUS_VC3_MASK (0x00010000U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC3_VC_PORT_ARBI_TABLE_STATUS_VC3_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC3_VC_PORT_ARBI_TABLE_STATUS_VC3_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC3_VC_NEGO_PENDING_VC3_MASK         (0x00020000U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC3_VC_NEGO_PENDING_VC3_SHIFT        (0x00000011U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC3_VC_NEGO_PENDING_VC3_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC3_RSVDP_18_MASK                    (0xFFFC0000U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC3_RSVDP_18_SHIFT                   (0x00000012U)
#define CSL_PCIE_RC_CORE_RESOURCE_STATUS_REG_VC3_RSVDP_18_MAX                     (0x00003FFFU)

/* SPCIE_CAP_HEADER_REG */

#define CSL_PCIE_RC_CORE_SPCIE_CAP_HEADER_REG_EXTENDED_CAP_ID_MASK                (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_HEADER_REG_EXTENDED_CAP_ID_SHIFT               (0x00000000U)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_HEADER_REG_EXTENDED_CAP_ID_MAX                 (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_SPCIE_CAP_HEADER_REG_CAP_VERSION_MASK                    (0x000F0000U)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_HEADER_REG_CAP_VERSION_SHIFT                   (0x00000010U)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_HEADER_REG_CAP_VERSION_MAX                     (0x0000000FU)

#define CSL_PCIE_RC_CORE_SPCIE_CAP_HEADER_REG_NEXT_OFFSET_MASK                    (0xFFF00000U)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_HEADER_REG_NEXT_OFFSET_SHIFT                   (0x00000014U)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_HEADER_REG_NEXT_OFFSET_MAX                     (0x00000FFFU)

/* LINK_CONTROL3_REG */

#define CSL_PCIE_RC_CORE_LINK_CONTROL3_REG_PERFORM_EQ_MASK                        (0x00000001U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL3_REG_PERFORM_EQ_SHIFT                       (0x00000000U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL3_REG_PERFORM_EQ_MAX                         (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL3_REG_EQ_REQ_INT_EN_MASK                     (0x00000002U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL3_REG_EQ_REQ_INT_EN_SHIFT                    (0x00000001U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL3_REG_EQ_REQ_INT_EN_MAX                      (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_CONTROL3_REG_RSVDP_2_MASK                           (0xFFFFFFFCU)
#define CSL_PCIE_RC_CORE_LINK_CONTROL3_REG_RSVDP_2_SHIFT                          (0x00000002U)
#define CSL_PCIE_RC_CORE_LINK_CONTROL3_REG_RSVDP_2_MAX                            (0x3FFFFFFFU)

/* LANE_ERR_STATUS_REG */

#define CSL_PCIE_RC_CORE_LANE_ERR_STATUS_REG_LANE_ERR_STATUS_MASK                 (0x00000003U)
#define CSL_PCIE_RC_CORE_LANE_ERR_STATUS_REG_LANE_ERR_STATUS_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_LANE_ERR_STATUS_REG_LANE_ERR_STATUS_MAX                  (0x00000003U)

#define CSL_PCIE_RC_CORE_LANE_ERR_STATUS_REG_RSVDP_LANE_ERR_STATUS_MASK           (0xFFFFFFFCU)
#define CSL_PCIE_RC_CORE_LANE_ERR_STATUS_REG_RSVDP_LANE_ERR_STATUS_SHIFT          (0x00000002U)
#define CSL_PCIE_RC_CORE_LANE_ERR_STATUS_REG_RSVDP_LANE_ERR_STATUS_MAX            (0x3FFFFFFFU)

/* SPCIE_CAP_OFF_0CH_REG */

#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_DSP_TX_PRESET0_MASK                (0x0000000FU)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_DSP_TX_PRESET0_SHIFT               (0x00000000U)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_DSP_TX_PRESET0_MAX                 (0x0000000FU)

#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_DSP_RX_PRESET_HINT0_MASK           (0x00000070U)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_DSP_RX_PRESET_HINT0_SHIFT          (0x00000004U)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_DSP_RX_PRESET_HINT0_MAX            (0x00000007U)

#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_RSVDP_7_MASK                       (0x00000080U)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_RSVDP_7_SHIFT                      (0x00000007U)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_RSVDP_7_MAX                        (0x00000001U)

#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_USP_TX_PRESET0_MASK                (0x00000F00U)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_USP_TX_PRESET0_SHIFT               (0x00000008U)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_USP_TX_PRESET0_MAX                 (0x0000000FU)

#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_USP_RX_PRESET_HINT0_MASK           (0x00007000U)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_USP_RX_PRESET_HINT0_SHIFT          (0x0000000CU)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_USP_RX_PRESET_HINT0_MAX            (0x00000007U)

#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_RSVDP_15_MASK                      (0x00008000U)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_RSVDP_15_SHIFT                     (0x0000000FU)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_RSVDP_15_MAX                       (0x00000001U)

#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_DSP_TX_PRESET1_MASK                (0x000F0000U)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_DSP_TX_PRESET1_SHIFT               (0x00000010U)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_DSP_TX_PRESET1_MAX                 (0x0000000FU)

#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_DSP_RX_PRESET_HINT1_MASK           (0x00700000U)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_DSP_RX_PRESET_HINT1_SHIFT          (0x00000014U)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_DSP_RX_PRESET_HINT1_MAX            (0x00000007U)

#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_RSVDP_23_MASK                      (0x00800000U)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_RSVDP_23_SHIFT                     (0x00000017U)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_RSVDP_23_MAX                       (0x00000001U)

#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_USP_TX_PRESET1_MASK                (0x0F000000U)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_USP_TX_PRESET1_SHIFT               (0x00000018U)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_USP_TX_PRESET1_MAX                 (0x0000000FU)

#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_USP_RX_PRESET_HINT1_MASK           (0x70000000U)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_USP_RX_PRESET_HINT1_SHIFT          (0x0000001CU)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_USP_RX_PRESET_HINT1_MAX            (0x00000007U)

#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_RSVDP_31_MASK                      (0x80000000U)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_RSVDP_31_SHIFT                     (0x0000001FU)
#define CSL_PCIE_RC_CORE_SPCIE_CAP_OFF_0CH_REG_RSVDP_31_MAX                       (0x00000001U)

/* L1SUB_CAP_HEADER_REG */

#define CSL_PCIE_RC_CORE_L1SUB_CAP_HEADER_REG_EXTENDED_CAP_ID_MASK                (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_L1SUB_CAP_HEADER_REG_EXTENDED_CAP_ID_SHIFT               (0x00000000U)
#define CSL_PCIE_RC_CORE_L1SUB_CAP_HEADER_REG_EXTENDED_CAP_ID_MAX                 (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_L1SUB_CAP_HEADER_REG_CAP_VERSION_MASK                    (0x000F0000U)
#define CSL_PCIE_RC_CORE_L1SUB_CAP_HEADER_REG_CAP_VERSION_SHIFT                   (0x00000010U)
#define CSL_PCIE_RC_CORE_L1SUB_CAP_HEADER_REG_CAP_VERSION_MAX                     (0x0000000FU)

#define CSL_PCIE_RC_CORE_L1SUB_CAP_HEADER_REG_NEXT_OFFSET_MASK                    (0xFFF00000U)
#define CSL_PCIE_RC_CORE_L1SUB_CAP_HEADER_REG_NEXT_OFFSET_SHIFT                   (0x00000014U)
#define CSL_PCIE_RC_CORE_L1SUB_CAP_HEADER_REG_NEXT_OFFSET_MAX                     (0x00000FFFU)

/* L1SUB_CAPABILITY_REG */

#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_L1_2_PCIPM_SUPPORT_MASK             (0x00000001U)
#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_L1_2_PCIPM_SUPPORT_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_L1_2_PCIPM_SUPPORT_MAX              (0x00000001U)

#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_L1_1_PCIPM_SUPPORT_MASK             (0x00000002U)
#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_L1_1_PCIPM_SUPPORT_SHIFT            (0x00000001U)
#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_L1_1_PCIPM_SUPPORT_MAX              (0x00000001U)

#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_L1_2_ASPM_SUPPORT_MASK              (0x00000004U)
#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_L1_2_ASPM_SUPPORT_SHIFT             (0x00000002U)
#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_L1_2_ASPM_SUPPORT_MAX               (0x00000001U)

#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_L1_1_ASPM_SUPPORT_MASK              (0x00000008U)
#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_L1_1_ASPM_SUPPORT_SHIFT             (0x00000003U)
#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_L1_1_ASPM_SUPPORT_MAX               (0x00000001U)

#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_L1_PMSUB_SUPPORT_MASK               (0x00000010U)
#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_L1_PMSUB_SUPPORT_SHIFT              (0x00000004U)
#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_L1_PMSUB_SUPPORT_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_RSVDP_5_MASK                        (0x000000E0U)
#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_RSVDP_5_SHIFT                       (0x00000005U)
#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_RSVDP_5_MAX                         (0x00000007U)

#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_COMM_MODE_SUPPORT_MASK              (0x0000FF00U)
#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_COMM_MODE_SUPPORT_SHIFT             (0x00000008U)
#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_COMM_MODE_SUPPORT_MAX               (0x000000FFU)

#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_PWR_ON_SCALE_SUPPORT_MASK           (0x00030000U)
#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_PWR_ON_SCALE_SUPPORT_SHIFT          (0x00000010U)
#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_PWR_ON_SCALE_SUPPORT_MAX            (0x00000003U)

#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_RSVDP_18_MASK                       (0x00040000U)
#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_RSVDP_18_SHIFT                      (0x00000012U)
#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_RSVDP_18_MAX                        (0x00000001U)

#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_PWR_ON_VALUE_SUPPORT_MASK           (0x00F80000U)
#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_PWR_ON_VALUE_SUPPORT_SHIFT          (0x00000013U)
#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_PWR_ON_VALUE_SUPPORT_MAX            (0x0000001FU)

#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_RSVDP_24_MASK                       (0xFF000000U)
#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_RSVDP_24_SHIFT                      (0x00000018U)
#define CSL_PCIE_RC_CORE_L1SUB_CAPABILITY_REG_RSVDP_24_MAX                        (0x000000FFU)

/* L1SUB_CONTROL1_REG */

#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG_L1_2_PCIPM_EN_MASK                    (0x00000001U)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG_L1_2_PCIPM_EN_SHIFT                   (0x00000000U)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG_L1_2_PCIPM_EN_MAX                     (0x00000001U)

#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG_L1_1_PCIPM_EN_MASK                    (0x00000002U)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG_L1_1_PCIPM_EN_SHIFT                   (0x00000001U)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG_L1_1_PCIPM_EN_MAX                     (0x00000001U)

#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG_L1_2_ASPM_EN_MASK                     (0x00000004U)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG_L1_2_ASPM_EN_SHIFT                    (0x00000002U)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG_L1_2_ASPM_EN_MAX                      (0x00000001U)

#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG_L1_1_ASPM_EN_MASK                     (0x00000008U)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG_L1_1_ASPM_EN_SHIFT                    (0x00000003U)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG_L1_1_ASPM_EN_MAX                      (0x00000001U)

#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG_RSVDP_4_MASK                          (0x000000F0U)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG_RSVDP_4_SHIFT                         (0x00000004U)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG_RSVDP_4_MAX                           (0x0000000FU)

#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG_T_COMMON_MODE_MASK                    (0x0000FF00U)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG_T_COMMON_MODE_SHIFT                   (0x00000008U)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG_T_COMMON_MODE_MAX                     (0x000000FFU)

#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG_L1_2_TH_VAL_MASK                      (0x03FF0000U)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG_L1_2_TH_VAL_SHIFT                     (0x00000010U)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG_L1_2_TH_VAL_MAX                       (0x000003FFU)

#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG_RSVDP_26_MASK                         (0x1C000000U)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG_RSVDP_26_SHIFT                        (0x0000001AU)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG_RSVDP_26_MAX                          (0x00000007U)

#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG_L1_2_TH_SCA_MASK                      (0xE0000000U)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG_L1_2_TH_SCA_SHIFT                     (0x0000001DU)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL1_REG_L1_2_TH_SCA_MAX                       (0x00000007U)

/* L1SUB_CONTROL2_REG */

#define CSL_PCIE_RC_CORE_L1SUB_CONTROL2_REG_T_POWER_ON_SCALE_MASK                 (0x00000003U)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL2_REG_T_POWER_ON_SCALE_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL2_REG_T_POWER_ON_SCALE_MAX                  (0x00000003U)

#define CSL_PCIE_RC_CORE_L1SUB_CONTROL2_REG_RSVDP_2_MASK                          (0x00000004U)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL2_REG_RSVDP_2_SHIFT                         (0x00000002U)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL2_REG_RSVDP_2_MAX                           (0x00000001U)

#define CSL_PCIE_RC_CORE_L1SUB_CONTROL2_REG_T_POWER_ON_VALUE_MASK                 (0x000000F8U)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL2_REG_T_POWER_ON_VALUE_SHIFT                (0x00000003U)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL2_REG_T_POWER_ON_VALUE_MAX                  (0x0000001FU)

#define CSL_PCIE_RC_CORE_L1SUB_CONTROL2_REG_RSVDP_8_MASK                          (0xFFFFFF00U)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL2_REG_RSVDP_8_SHIFT                         (0x00000008U)
#define CSL_PCIE_RC_CORE_L1SUB_CONTROL2_REG_RSVDP_8_MAX                           (0x00FFFFFFU)

/* PTM_EXT_CAP_HDR_OFF */

#define CSL_PCIE_RC_CORE_PTM_EXT_CAP_HDR_OFF_PTM_CAP_ID_MASK                      (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_PTM_EXT_CAP_HDR_OFF_PTM_CAP_ID_SHIFT                     (0x00000000U)
#define CSL_PCIE_RC_CORE_PTM_EXT_CAP_HDR_OFF_PTM_CAP_ID_MAX                       (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_PTM_EXT_CAP_HDR_OFF_PTM_CAP_VERSION_MASK                 (0x000F0000U)
#define CSL_PCIE_RC_CORE_PTM_EXT_CAP_HDR_OFF_PTM_CAP_VERSION_SHIFT                (0x00000010U)
#define CSL_PCIE_RC_CORE_PTM_EXT_CAP_HDR_OFF_PTM_CAP_VERSION_MAX                  (0x0000000FU)

#define CSL_PCIE_RC_CORE_PTM_EXT_CAP_HDR_OFF_PTM_NEXT_OFFSET_MASK                 (0xFFF00000U)
#define CSL_PCIE_RC_CORE_PTM_EXT_CAP_HDR_OFF_PTM_NEXT_OFFSET_SHIFT                (0x00000014U)
#define CSL_PCIE_RC_CORE_PTM_EXT_CAP_HDR_OFF_PTM_NEXT_OFFSET_MAX                  (0x00000FFFU)

/* PTM_CAP_OFF */

#define CSL_PCIE_RC_CORE_PTM_CAP_OFF_PTM_REQ_CAPABLE_MASK                         (0x00000001U)
#define CSL_PCIE_RC_CORE_PTM_CAP_OFF_PTM_REQ_CAPABLE_SHIFT                        (0x00000000U)
#define CSL_PCIE_RC_CORE_PTM_CAP_OFF_PTM_REQ_CAPABLE_MAX                          (0x00000001U)

#define CSL_PCIE_RC_CORE_PTM_CAP_OFF_PTM_RES_CAPABLE_MASK                         (0x00000002U)
#define CSL_PCIE_RC_CORE_PTM_CAP_OFF_PTM_RES_CAPABLE_SHIFT                        (0x00000001U)
#define CSL_PCIE_RC_CORE_PTM_CAP_OFF_PTM_RES_CAPABLE_MAX                          (0x00000001U)

#define CSL_PCIE_RC_CORE_PTM_CAP_OFF_PTM_ROOT_CAPABLE_MASK                        (0x00000004U)
#define CSL_PCIE_RC_CORE_PTM_CAP_OFF_PTM_ROOT_CAPABLE_SHIFT                       (0x00000002U)
#define CSL_PCIE_RC_CORE_PTM_CAP_OFF_PTM_ROOT_CAPABLE_MAX                         (0x00000001U)

#define CSL_PCIE_RC_CORE_PTM_CAP_OFF_RSVDP_3_MASK                                 (0x000000F8U)
#define CSL_PCIE_RC_CORE_PTM_CAP_OFF_RSVDP_3_SHIFT                                (0x00000003U)
#define CSL_PCIE_RC_CORE_PTM_CAP_OFF_RSVDP_3_MAX                                  (0x0000001FU)

#define CSL_PCIE_RC_CORE_PTM_CAP_OFF_PTM_CLK_GRAN_MASK                            (0x0000FF00U)
#define CSL_PCIE_RC_CORE_PTM_CAP_OFF_PTM_CLK_GRAN_SHIFT                           (0x00000008U)
#define CSL_PCIE_RC_CORE_PTM_CAP_OFF_PTM_CLK_GRAN_MAX                             (0x000000FFU)

#define CSL_PCIE_RC_CORE_PTM_CAP_OFF_RSVDP_16_MASK                                (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_PTM_CAP_OFF_RSVDP_16_SHIFT                               (0x00000010U)
#define CSL_PCIE_RC_CORE_PTM_CAP_OFF_RSVDP_16_MAX                                 (0x0000FFFFU)

/* PTM_CONTROL_OFF */

#define CSL_PCIE_RC_CORE_PTM_CONTROL_OFF_PTM_ENABLE_MASK                          (0x00000001U)
#define CSL_PCIE_RC_CORE_PTM_CONTROL_OFF_PTM_ENABLE_SHIFT                         (0x00000000U)
#define CSL_PCIE_RC_CORE_PTM_CONTROL_OFF_PTM_ENABLE_MAX                           (0x00000001U)

#define CSL_PCIE_RC_CORE_PTM_CONTROL_OFF_ROOT_SELECT_MASK                         (0x00000002U)
#define CSL_PCIE_RC_CORE_PTM_CONTROL_OFF_ROOT_SELECT_SHIFT                        (0x00000001U)
#define CSL_PCIE_RC_CORE_PTM_CONTROL_OFF_ROOT_SELECT_MAX                          (0x00000001U)

#define CSL_PCIE_RC_CORE_PTM_CONTROL_OFF_RSVDP_2_MASK                             (0x000000FCU)
#define CSL_PCIE_RC_CORE_PTM_CONTROL_OFF_RSVDP_2_SHIFT                            (0x00000002U)
#define CSL_PCIE_RC_CORE_PTM_CONTROL_OFF_RSVDP_2_MAX                              (0x0000003FU)

#define CSL_PCIE_RC_CORE_PTM_CONTROL_OFF_EFF_GRAN_MASK                            (0x0000FF00U)
#define CSL_PCIE_RC_CORE_PTM_CONTROL_OFF_EFF_GRAN_SHIFT                           (0x00000008U)
#define CSL_PCIE_RC_CORE_PTM_CONTROL_OFF_EFF_GRAN_MAX                             (0x000000FFU)

#define CSL_PCIE_RC_CORE_PTM_CONTROL_OFF_RSVDP_16_MASK                            (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_PTM_CONTROL_OFF_RSVDP_16_SHIFT                           (0x00000010U)
#define CSL_PCIE_RC_CORE_PTM_CONTROL_OFF_RSVDP_16_MAX                             (0x0000FFFFU)

/* PTM_RES_CAP_HDR_OFF */

#define CSL_PCIE_RC_CORE_PTM_RES_CAP_HDR_OFF_PTM_RES_EXT_CAP_ID_MASK              (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_PTM_RES_CAP_HDR_OFF_PTM_RES_EXT_CAP_ID_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_PTM_RES_CAP_HDR_OFF_PTM_RES_EXT_CAP_ID_MAX               (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_PTM_RES_CAP_HDR_OFF_PTM_RES_EXT_CAP_VER_MASK             (0x000F0000U)
#define CSL_PCIE_RC_CORE_PTM_RES_CAP_HDR_OFF_PTM_RES_EXT_CAP_VER_SHIFT            (0x00000010U)
#define CSL_PCIE_RC_CORE_PTM_RES_CAP_HDR_OFF_PTM_RES_EXT_CAP_VER_MAX              (0x0000000FU)

#define CSL_PCIE_RC_CORE_PTM_RES_CAP_HDR_OFF_PTM_RES_EXT_CAP_NEXT_OFFS_MASK       (0xFFF00000U)
#define CSL_PCIE_RC_CORE_PTM_RES_CAP_HDR_OFF_PTM_RES_EXT_CAP_NEXT_OFFS_SHIFT      (0x00000014U)
#define CSL_PCIE_RC_CORE_PTM_RES_CAP_HDR_OFF_PTM_RES_EXT_CAP_NEXT_OFFS_MAX        (0x00000FFFU)

/* PTM_RES_HDR_OFF */

#define CSL_PCIE_RC_CORE_PTM_RES_HDR_OFF_PTM_RES_VSEC_ID_MASK                     (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_PTM_RES_HDR_OFF_PTM_RES_VSEC_ID_SHIFT                    (0x00000000U)
#define CSL_PCIE_RC_CORE_PTM_RES_HDR_OFF_PTM_RES_VSEC_ID_MAX                      (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_PTM_RES_HDR_OFF_PTM_RES_VSEC_REV_MASK                    (0x000F0000U)
#define CSL_PCIE_RC_CORE_PTM_RES_HDR_OFF_PTM_RES_VSEC_REV_SHIFT                   (0x00000010U)
#define CSL_PCIE_RC_CORE_PTM_RES_HDR_OFF_PTM_RES_VSEC_REV_MAX                     (0x0000000FU)

#define CSL_PCIE_RC_CORE_PTM_RES_HDR_OFF_PTM_RES_VSEC_LENGTH_MASK                 (0xFFF00000U)
#define CSL_PCIE_RC_CORE_PTM_RES_HDR_OFF_PTM_RES_VSEC_LENGTH_SHIFT                (0x00000014U)
#define CSL_PCIE_RC_CORE_PTM_RES_HDR_OFF_PTM_RES_VSEC_LENGTH_MAX                  (0x00000FFFU)

/* PTM_RES_CONTROL_OFF */

#define CSL_PCIE_RC_CORE_PTM_RES_CONTROL_OFF_PTM_RES_CCONTEXT_VALID_MASK          (0x00000001U)
#define CSL_PCIE_RC_CORE_PTM_RES_CONTROL_OFF_PTM_RES_CCONTEXT_VALID_SHIFT         (0x00000000U)
#define CSL_PCIE_RC_CORE_PTM_RES_CONTROL_OFF_PTM_RES_CCONTEXT_VALID_MAX           (0x00000001U)

#define CSL_PCIE_RC_CORE_PTM_RES_CONTROL_OFF_RSVDP_1_MASK                         (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_PTM_RES_CONTROL_OFF_RSVDP_1_SHIFT                        (0x00000001U)
#define CSL_PCIE_RC_CORE_PTM_RES_CONTROL_OFF_RSVDP_1_MAX                          (0x7FFFFFFFU)

/* PTM_RES_STATUS_OFF */

#define CSL_PCIE_RC_CORE_PTM_RES_STATUS_OFF_PTM_RES_CONTEXT_VALID_MASK            (0x00000001U)
#define CSL_PCIE_RC_CORE_PTM_RES_STATUS_OFF_PTM_RES_CONTEXT_VALID_SHIFT           (0x00000000U)
#define CSL_PCIE_RC_CORE_PTM_RES_STATUS_OFF_PTM_RES_CONTEXT_VALID_MAX             (0x00000001U)

#define CSL_PCIE_RC_CORE_PTM_RES_STATUS_OFF_PTM_RES_REQUEST_RECEIVED_MASK         (0x00000002U)
#define CSL_PCIE_RC_CORE_PTM_RES_STATUS_OFF_PTM_RES_REQUEST_RECEIVED_SHIFT        (0x00000001U)
#define CSL_PCIE_RC_CORE_PTM_RES_STATUS_OFF_PTM_RES_REQUEST_RECEIVED_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_PTM_RES_STATUS_OFF_RSVDP_2_MASK                          (0xFFFFFFFCU)
#define CSL_PCIE_RC_CORE_PTM_RES_STATUS_OFF_RSVDP_2_SHIFT                         (0x00000002U)
#define CSL_PCIE_RC_CORE_PTM_RES_STATUS_OFF_RSVDP_2_MAX                           (0x3FFFFFFFU)

/* PTM_RES_LOCAL_LSB_OFF */

#define CSL_PCIE_RC_CORE_PTM_RES_LOCAL_LSB_OFF_PTM_RES_LOCAL_LSB_MASK             (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_PTM_RES_LOCAL_LSB_OFF_PTM_RES_LOCAL_LSB_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_PTM_RES_LOCAL_LSB_OFF_PTM_RES_LOCAL_LSB_MAX              (0xFFFFFFFFU)

/* PTM_RES_LOCAL_MSB_OFF */

#define CSL_PCIE_RC_CORE_PTM_RES_LOCAL_MSB_OFF_PTM_RES_LOCAL_MSB_MASK             (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_PTM_RES_LOCAL_MSB_OFF_PTM_RES_LOCAL_MSB_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_PTM_RES_LOCAL_MSB_OFF_PTM_RES_LOCAL_MSB_MAX              (0xFFFFFFFFU)

/* PTM_RES_T2_LSB_OFF */

#define CSL_PCIE_RC_CORE_PTM_RES_T2_LSB_OFF_PTM_RES_T2_LSB_MASK                   (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_PTM_RES_T2_LSB_OFF_PTM_RES_T2_LSB_SHIFT                  (0x00000000U)
#define CSL_PCIE_RC_CORE_PTM_RES_T2_LSB_OFF_PTM_RES_T2_LSB_MAX                    (0xFFFFFFFFU)

/* PTM_RES_T2_MSB_OFF */

#define CSL_PCIE_RC_CORE_PTM_RES_T2_MSB_OFF_PTM_RES_T2_MSB_MASK                   (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_PTM_RES_T2_MSB_OFF_PTM_RES_T2_MSB_SHIFT                  (0x00000000U)
#define CSL_PCIE_RC_CORE_PTM_RES_T2_MSB_OFF_PTM_RES_T2_MSB_MAX                    (0xFFFFFFFFU)

/* PTM_RES_T2P_LSB_OFF */

#define CSL_PCIE_RC_CORE_PTM_RES_T2P_LSB_OFF_PTM_RES_T2P_LSB_MASK                 (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_PTM_RES_T2P_LSB_OFF_PTM_RES_T2P_LSB_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_PTM_RES_T2P_LSB_OFF_PTM_RES_T2P_LSB_MAX                  (0xFFFFFFFFU)

/* PTM_RES_T2P_MSB_OFF */

#define CSL_PCIE_RC_CORE_PTM_RES_T2P_MSB_OFF_PTM_RES_T2P_MSB_MASK                 (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_PTM_RES_T2P_MSB_OFF_PTM_RES_T2P_MSB_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_PTM_RES_T2P_MSB_OFF_PTM_RES_T2P_MSB_MAX                  (0xFFFFFFFFU)

/* PTM_RES_T3_LSB_OFF */

#define CSL_PCIE_RC_CORE_PTM_RES_T3_LSB_OFF_PTM_RES_T3_LSB_MASK                   (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_PTM_RES_T3_LSB_OFF_PTM_RES_T3_LSB_SHIFT                  (0x00000000U)
#define CSL_PCIE_RC_CORE_PTM_RES_T3_LSB_OFF_PTM_RES_T3_LSB_MAX                    (0xFFFFFFFFU)

/* PTM_RES_T3_MSB_OFF */

#define CSL_PCIE_RC_CORE_PTM_RES_T3_MSB_OFF_PTM_RES_T3_MSB_MASK                   (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_PTM_RES_T3_MSB_OFF_PTM_RES_T3_MSB_SHIFT                  (0x00000000U)
#define CSL_PCIE_RC_CORE_PTM_RES_T3_MSB_OFF_PTM_RES_T3_MSB_MAX                    (0xFFFFFFFFU)

/* PTM_RES_T3P_LSB_OFF */

#define CSL_PCIE_RC_CORE_PTM_RES_T3P_LSB_OFF_PTM_RES_T3P_LSB_MASK                 (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_PTM_RES_T3P_LSB_OFF_PTM_RES_T3P_LSB_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_PTM_RES_T3P_LSB_OFF_PTM_RES_T3P_LSB_MAX                  (0xFFFFFFFFU)

/* PTM_RES_T3P_MSB_OFF */

#define CSL_PCIE_RC_CORE_PTM_RES_T3P_MSB_OFF_PTM_RES_T3P_MSB_MASK                 (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_PTM_RES_T3P_MSB_OFF_PTM_RES_T3P_MSB_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_PTM_RES_T3P_MSB_OFF_PTM_RES_T3P_MSB_MAX                  (0xFFFFFFFFU)

/* PTM_RES_TX_LATENCY_OFF */

#define CSL_PCIE_RC_CORE_PTM_RES_TX_LATENCY_OFF_PTM_RES_TX_LATENCY_MASK           (0x00000FFFU)
#define CSL_PCIE_RC_CORE_PTM_RES_TX_LATENCY_OFF_PTM_RES_TX_LATENCY_SHIFT          (0x00000000U)
#define CSL_PCIE_RC_CORE_PTM_RES_TX_LATENCY_OFF_PTM_RES_TX_LATENCY_MAX            (0x00000FFFU)

#define CSL_PCIE_RC_CORE_PTM_RES_TX_LATENCY_OFF_RSVDP_12_MASK                     (0xFFFFF000U)
#define CSL_PCIE_RC_CORE_PTM_RES_TX_LATENCY_OFF_RSVDP_12_SHIFT                    (0x0000000CU)
#define CSL_PCIE_RC_CORE_PTM_RES_TX_LATENCY_OFF_RSVDP_12_MAX                      (0x000FFFFFU)

/* PTM_RES_RX_LATENCY_OFF */

#define CSL_PCIE_RC_CORE_PTM_RES_RX_LATENCY_OFF_PTM_RES_RX_LATENCY_MASK           (0x00000FFFU)
#define CSL_PCIE_RC_CORE_PTM_RES_RX_LATENCY_OFF_PTM_RES_RX_LATENCY_SHIFT          (0x00000000U)
#define CSL_PCIE_RC_CORE_PTM_RES_RX_LATENCY_OFF_PTM_RES_RX_LATENCY_MAX            (0x00000FFFU)

#define CSL_PCIE_RC_CORE_PTM_RES_RX_LATENCY_OFF_RSVDP_12_MASK                     (0xFFFFF000U)
#define CSL_PCIE_RC_CORE_PTM_RES_RX_LATENCY_OFF_RSVDP_12_SHIFT                    (0x0000000CU)
#define CSL_PCIE_RC_CORE_PTM_RES_RX_LATENCY_OFF_RSVDP_12_MAX                      (0x000FFFFFU)

/* ACK_LATENCY_TIMER_OFF */

#define CSL_PCIE_RC_CORE_ACK_LATENCY_TIMER_OFF_ROUND_TRIP_LATENCY_TIME_LIMIT_MASK (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_ACK_LATENCY_TIMER_OFF_ROUND_TRIP_LATENCY_TIME_LIMIT_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_ACK_LATENCY_TIMER_OFF_ROUND_TRIP_LATENCY_TIME_LIMIT_MAX  (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_ACK_LATENCY_TIMER_OFF_REPLAY_TIME_LIMIT_MASK             (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_ACK_LATENCY_TIMER_OFF_REPLAY_TIME_LIMIT_SHIFT            (0x00000010U)
#define CSL_PCIE_RC_CORE_ACK_LATENCY_TIMER_OFF_REPLAY_TIME_LIMIT_MAX              (0x0000FFFFU)

/* VENDOR_SPEC_DLLP_OFF */

#define CSL_PCIE_RC_CORE_VENDOR_SPEC_DLLP_OFF_VENDOR_SPEC_DLLP_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_VENDOR_SPEC_DLLP_OFF_VENDOR_SPEC_DLLP_SHIFT              (0x00000000U)
#define CSL_PCIE_RC_CORE_VENDOR_SPEC_DLLP_OFF_VENDOR_SPEC_DLLP_MAX                (0xFFFFFFFFU)

/* PORT_FORCE_OFF */

#define CSL_PCIE_RC_CORE_PORT_FORCE_OFF_LINK_NUM_MASK                             (0x000000FFU)
#define CSL_PCIE_RC_CORE_PORT_FORCE_OFF_LINK_NUM_SHIFT                            (0x00000000U)
#define CSL_PCIE_RC_CORE_PORT_FORCE_OFF_LINK_NUM_MAX                              (0x000000FFU)

#define CSL_PCIE_RC_CORE_PORT_FORCE_OFF_FORCED_LTSSM_MASK                         (0x00000F00U)
#define CSL_PCIE_RC_CORE_PORT_FORCE_OFF_FORCED_LTSSM_SHIFT                        (0x00000008U)
#define CSL_PCIE_RC_CORE_PORT_FORCE_OFF_FORCED_LTSSM_MAX                          (0x0000000FU)

#define CSL_PCIE_RC_CORE_PORT_FORCE_OFF_RSVDP_12_MASK                             (0x00007000U)
#define CSL_PCIE_RC_CORE_PORT_FORCE_OFF_RSVDP_12_SHIFT                            (0x0000000CU)
#define CSL_PCIE_RC_CORE_PORT_FORCE_OFF_RSVDP_12_MAX                              (0x00000007U)

#define CSL_PCIE_RC_CORE_PORT_FORCE_OFF_FORCE_EN_MASK                             (0x00008000U)
#define CSL_PCIE_RC_CORE_PORT_FORCE_OFF_FORCE_EN_SHIFT                            (0x0000000FU)
#define CSL_PCIE_RC_CORE_PORT_FORCE_OFF_FORCE_EN_MAX                              (0x00000001U)

#define CSL_PCIE_RC_CORE_PORT_FORCE_OFF_LINK_STATE_MASK                           (0x003F0000U)
#define CSL_PCIE_RC_CORE_PORT_FORCE_OFF_LINK_STATE_SHIFT                          (0x00000010U)
#define CSL_PCIE_RC_CORE_PORT_FORCE_OFF_LINK_STATE_MAX                            (0x0000003FU)

#define CSL_PCIE_RC_CORE_PORT_FORCE_OFF_RSVDP_22_MASK                             (0x00400000U)
#define CSL_PCIE_RC_CORE_PORT_FORCE_OFF_RSVDP_22_SHIFT                            (0x00000016U)
#define CSL_PCIE_RC_CORE_PORT_FORCE_OFF_RSVDP_22_MAX                              (0x00000001U)

#define CSL_PCIE_RC_CORE_PORT_FORCE_OFF_DO_DESKEW_FOR_SRIS_MASK                   (0x00800000U)
#define CSL_PCIE_RC_CORE_PORT_FORCE_OFF_DO_DESKEW_FOR_SRIS_SHIFT                  (0x00000017U)
#define CSL_PCIE_RC_CORE_PORT_FORCE_OFF_DO_DESKEW_FOR_SRIS_MAX                    (0x00000001U)

#define CSL_PCIE_RC_CORE_PORT_FORCE_OFF_RSVDP_24_MASK                             (0xFF000000U)
#define CSL_PCIE_RC_CORE_PORT_FORCE_OFF_RSVDP_24_SHIFT                            (0x00000018U)
#define CSL_PCIE_RC_CORE_PORT_FORCE_OFF_RSVDP_24_MAX                              (0x000000FFU)

/* ACK_F_ASPM_CTRL_OFF */

#define CSL_PCIE_RC_CORE_ACK_F_ASPM_CTRL_OFF_ACK_FREQ_MASK                        (0x000000FFU)
#define CSL_PCIE_RC_CORE_ACK_F_ASPM_CTRL_OFF_ACK_FREQ_SHIFT                       (0x00000000U)
#define CSL_PCIE_RC_CORE_ACK_F_ASPM_CTRL_OFF_ACK_FREQ_MAX                         (0x000000FFU)

#define CSL_PCIE_RC_CORE_ACK_F_ASPM_CTRL_OFF_ACK_N_FTS_MASK                       (0x0000FF00U)
#define CSL_PCIE_RC_CORE_ACK_F_ASPM_CTRL_OFF_ACK_N_FTS_SHIFT                      (0x00000008U)
#define CSL_PCIE_RC_CORE_ACK_F_ASPM_CTRL_OFF_ACK_N_FTS_MAX                        (0x000000FFU)

#define CSL_PCIE_RC_CORE_ACK_F_ASPM_CTRL_OFF_COMMON_CLK_N_FTS_MASK                (0x00FF0000U)
#define CSL_PCIE_RC_CORE_ACK_F_ASPM_CTRL_OFF_COMMON_CLK_N_FTS_SHIFT               (0x00000010U)
#define CSL_PCIE_RC_CORE_ACK_F_ASPM_CTRL_OFF_COMMON_CLK_N_FTS_MAX                 (0x000000FFU)

#define CSL_PCIE_RC_CORE_ACK_F_ASPM_CTRL_OFF_L0S_ENTRANCE_LATENCY_MASK            (0x07000000U)
#define CSL_PCIE_RC_CORE_ACK_F_ASPM_CTRL_OFF_L0S_ENTRANCE_LATENCY_SHIFT           (0x00000018U)
#define CSL_PCIE_RC_CORE_ACK_F_ASPM_CTRL_OFF_L0S_ENTRANCE_LATENCY_MAX             (0x00000007U)

#define CSL_PCIE_RC_CORE_ACK_F_ASPM_CTRL_OFF_L1_ENTRANCE_LATENCY_MASK             (0x38000000U)
#define CSL_PCIE_RC_CORE_ACK_F_ASPM_CTRL_OFF_L1_ENTRANCE_LATENCY_SHIFT            (0x0000001BU)
#define CSL_PCIE_RC_CORE_ACK_F_ASPM_CTRL_OFF_L1_ENTRANCE_LATENCY_MAX              (0x00000007U)

#define CSL_PCIE_RC_CORE_ACK_F_ASPM_CTRL_OFF_ENTER_ASPM_MASK                      (0x40000000U)
#define CSL_PCIE_RC_CORE_ACK_F_ASPM_CTRL_OFF_ENTER_ASPM_SHIFT                     (0x0000001EU)
#define CSL_PCIE_RC_CORE_ACK_F_ASPM_CTRL_OFF_ENTER_ASPM_MAX                       (0x00000001U)

#define CSL_PCIE_RC_CORE_ACK_F_ASPM_CTRL_OFF_RSVDP_31_MASK                        (0x80000000U)
#define CSL_PCIE_RC_CORE_ACK_F_ASPM_CTRL_OFF_RSVDP_31_SHIFT                       (0x0000001FU)
#define CSL_PCIE_RC_CORE_ACK_F_ASPM_CTRL_OFF_RSVDP_31_MAX                         (0x00000001U)

/* PORT_LINK_CTRL_OFF */

#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_VENDOR_SPECIFIC_DLLP_REQ_MASK         (0x00000001U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_VENDOR_SPECIFIC_DLLP_REQ_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_VENDOR_SPECIFIC_DLLP_REQ_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_SCRAMBLE_DISABLE_MASK                 (0x00000002U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_SCRAMBLE_DISABLE_SHIFT                (0x00000001U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_SCRAMBLE_DISABLE_MAX                  (0x00000001U)

#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_LOOPBACK_ENABLE_MASK                  (0x00000004U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_LOOPBACK_ENABLE_SHIFT                 (0x00000002U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_LOOPBACK_ENABLE_MAX                   (0x00000001U)

#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_RESET_ASSERT_MASK                     (0x00000008U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_RESET_ASSERT_SHIFT                    (0x00000003U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_RESET_ASSERT_MAX                      (0x00000001U)

#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_RSVDP_4_MASK                          (0x00000010U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_RSVDP_4_SHIFT                         (0x00000004U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_RSVDP_4_MAX                           (0x00000001U)

#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_DLL_LINK_EN_MASK                      (0x00000020U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_DLL_LINK_EN_SHIFT                     (0x00000005U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_DLL_LINK_EN_MAX                       (0x00000001U)

#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_LINK_DISABLE_MASK                     (0x00000040U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_LINK_DISABLE_SHIFT                    (0x00000006U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_LINK_DISABLE_MAX                      (0x00000001U)

#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_FAST_LINK_MODE_MASK                   (0x00000080U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_FAST_LINK_MODE_SHIFT                  (0x00000007U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_FAST_LINK_MODE_MAX                    (0x00000001U)

#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_LINK_RATE_MASK                        (0x00000F00U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_LINK_RATE_SHIFT                       (0x00000008U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_LINK_RATE_MAX                         (0x0000000FU)

#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_RSVDP_12_MASK                         (0x0000F000U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_RSVDP_12_SHIFT                        (0x0000000CU)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_RSVDP_12_MAX                          (0x0000000FU)

#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_LINK_CAPABLE_MASK                     (0x003F0000U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_LINK_CAPABLE_SHIFT                    (0x00000010U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_LINK_CAPABLE_MAX                      (0x0000003FU)

#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_BEACON_ENABLE_MASK                    (0x01000000U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_BEACON_ENABLE_SHIFT                   (0x00000018U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_BEACON_ENABLE_MAX                     (0x00000001U)

#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_CORRUPT_LCRC_ENABLE_MASK              (0x02000000U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_CORRUPT_LCRC_ENABLE_SHIFT             (0x00000019U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_CORRUPT_LCRC_ENABLE_MAX               (0x00000001U)

#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_EXTENDED_SYNCH_MASK                   (0x04000000U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_EXTENDED_SYNCH_SHIFT                  (0x0000001AU)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_EXTENDED_SYNCH_MAX                    (0x00000001U)

#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_TRANSMIT_LANE_REVERSALE_ENABLE_MASK   (0x08000000U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_TRANSMIT_LANE_REVERSALE_ENABLE_SHIFT  (0x0000001BU)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_TRANSMIT_LANE_REVERSALE_ENABLE_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_RSVDP_28_MASK                         (0xF0000000U)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_RSVDP_28_SHIFT                        (0x0000001CU)
#define CSL_PCIE_RC_CORE_PORT_LINK_CTRL_OFF_RSVDP_28_MAX                          (0x0000000FU)

/* LANE_SKEW_OFF */

#define CSL_PCIE_RC_CORE_LANE_SKEW_OFF_INSERT_LANE_SKEW_MASK                      (0x00FFFFFFU)
#define CSL_PCIE_RC_CORE_LANE_SKEW_OFF_INSERT_LANE_SKEW_SHIFT                     (0x00000000U)
#define CSL_PCIE_RC_CORE_LANE_SKEW_OFF_INSERT_LANE_SKEW_MAX                       (0x00FFFFFFU)

#define CSL_PCIE_RC_CORE_LANE_SKEW_OFF_FLOW_CTRL_DISABLE_MASK                     (0x01000000U)
#define CSL_PCIE_RC_CORE_LANE_SKEW_OFF_FLOW_CTRL_DISABLE_SHIFT                    (0x00000018U)
#define CSL_PCIE_RC_CORE_LANE_SKEW_OFF_FLOW_CTRL_DISABLE_MAX                      (0x00000001U)

#define CSL_PCIE_RC_CORE_LANE_SKEW_OFF_ACK_NAK_DISABLE_MASK                       (0x02000000U)
#define CSL_PCIE_RC_CORE_LANE_SKEW_OFF_ACK_NAK_DISABLE_SHIFT                      (0x00000019U)
#define CSL_PCIE_RC_CORE_LANE_SKEW_OFF_ACK_NAK_DISABLE_MAX                        (0x00000001U)

#define CSL_PCIE_RC_CORE_LANE_SKEW_OFF_LANE_SKEW_OFF_26_MASK                      (0x04000000U)
#define CSL_PCIE_RC_CORE_LANE_SKEW_OFF_LANE_SKEW_OFF_26_SHIFT                     (0x0000001AU)
#define CSL_PCIE_RC_CORE_LANE_SKEW_OFF_LANE_SKEW_OFF_26_MAX                       (0x00000001U)

#define CSL_PCIE_RC_CORE_LANE_SKEW_OFF_IMPLEMENT_NUM_LANES_MASK                   (0x78000000U)
#define CSL_PCIE_RC_CORE_LANE_SKEW_OFF_IMPLEMENT_NUM_LANES_SHIFT                  (0x0000001BU)
#define CSL_PCIE_RC_CORE_LANE_SKEW_OFF_IMPLEMENT_NUM_LANES_MAX                    (0x0000000FU)

#define CSL_PCIE_RC_CORE_LANE_SKEW_OFF_DISABLE_LANE_TO_LANE_DESKEW_MASK           (0x80000000U)
#define CSL_PCIE_RC_CORE_LANE_SKEW_OFF_DISABLE_LANE_TO_LANE_DESKEW_SHIFT          (0x0000001FU)
#define CSL_PCIE_RC_CORE_LANE_SKEW_OFF_DISABLE_LANE_TO_LANE_DESKEW_MAX            (0x00000001U)

/* TIMER_CTRL_MAX_FUNC_NUM_OFF */

#define CSL_PCIE_RC_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_MAX_FUNC_NUM_MASK            (0x000000FFU)
#define CSL_PCIE_RC_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_MAX_FUNC_NUM_SHIFT           (0x00000000U)
#define CSL_PCIE_RC_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_MAX_FUNC_NUM_MAX             (0x000000FFU)

#define CSL_PCIE_RC_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_RSVDP_8_MASK                 (0x00003F00U)
#define CSL_PCIE_RC_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_RSVDP_8_SHIFT                (0x00000008U)
#define CSL_PCIE_RC_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_RSVDP_8_MAX                  (0x0000003FU)

#define CSL_PCIE_RC_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_TIMER_MOD_REPLAY_TIMER_MASK  (0x0007C000U)
#define CSL_PCIE_RC_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_TIMER_MOD_REPLAY_TIMER_SHIFT (0x0000000EU)
#define CSL_PCIE_RC_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_TIMER_MOD_REPLAY_TIMER_MAX   (0x0000001FU)

#define CSL_PCIE_RC_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_TIMER_MOD_ACK_NAK_MASK       (0x00F80000U)
#define CSL_PCIE_RC_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_TIMER_MOD_ACK_NAK_SHIFT      (0x00000013U)
#define CSL_PCIE_RC_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_TIMER_MOD_ACK_NAK_MAX        (0x0000001FU)

#define CSL_PCIE_RC_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_UPDATE_FREQ_TIMER_MASK       (0x1F000000U)
#define CSL_PCIE_RC_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_UPDATE_FREQ_TIMER_SHIFT      (0x00000018U)
#define CSL_PCIE_RC_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_UPDATE_FREQ_TIMER_MAX        (0x0000001FU)

#define CSL_PCIE_RC_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_FAST_LINK_SCALING_FACTOR_MASK (0x60000000U)
#define CSL_PCIE_RC_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_FAST_LINK_SCALING_FACTOR_SHIFT (0x0000001DU)
#define CSL_PCIE_RC_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_FAST_LINK_SCALING_FACTOR_MAX (0x00000003U)

#define CSL_PCIE_RC_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_RSVDP_31_MASK                (0x80000000U)
#define CSL_PCIE_RC_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_RSVDP_31_SHIFT               (0x0000001FU)
#define CSL_PCIE_RC_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_RSVDP_31_MAX                 (0x00000001U)

/* SYMBOL_TIMER_FILTER_1_OFF */

#define CSL_PCIE_RC_CORE_SYMBOL_TIMER_FILTER_1_OFF_SKP_INT_VAL_MASK               (0x000007FFU)
#define CSL_PCIE_RC_CORE_SYMBOL_TIMER_FILTER_1_OFF_SKP_INT_VAL_SHIFT              (0x00000000U)
#define CSL_PCIE_RC_CORE_SYMBOL_TIMER_FILTER_1_OFF_SKP_INT_VAL_MAX                (0x000007FFU)

#define CSL_PCIE_RC_CORE_SYMBOL_TIMER_FILTER_1_OFF_EIDLE_TIMER_MASK               (0x00007800U)
#define CSL_PCIE_RC_CORE_SYMBOL_TIMER_FILTER_1_OFF_EIDLE_TIMER_SHIFT              (0x0000000BU)
#define CSL_PCIE_RC_CORE_SYMBOL_TIMER_FILTER_1_OFF_EIDLE_TIMER_MAX                (0x0000000FU)

#define CSL_PCIE_RC_CORE_SYMBOL_TIMER_FILTER_1_OFF_DISABLE_FC_WD_TIMER_MASK       (0x00008000U)
#define CSL_PCIE_RC_CORE_SYMBOL_TIMER_FILTER_1_OFF_DISABLE_FC_WD_TIMER_SHIFT      (0x0000000FU)
#define CSL_PCIE_RC_CORE_SYMBOL_TIMER_FILTER_1_OFF_DISABLE_FC_WD_TIMER_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_SYMBOL_TIMER_FILTER_1_OFF_MASK_RADM_1_MASK               (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_SYMBOL_TIMER_FILTER_1_OFF_MASK_RADM_1_SHIFT              (0x00000010U)
#define CSL_PCIE_RC_CORE_SYMBOL_TIMER_FILTER_1_OFF_MASK_RADM_1_MAX                (0x0000FFFFU)

/* FILTER_MASK_2_OFF */

#define CSL_PCIE_RC_CORE_FILTER_MASK_2_OFF_MASK_RADM_2_MASK                       (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_FILTER_MASK_2_OFF_MASK_RADM_2_SHIFT                      (0x00000000U)
#define CSL_PCIE_RC_CORE_FILTER_MASK_2_OFF_MASK_RADM_2_MAX                        (0xFFFFFFFFU)

/* AMBA_MUL_OB_DECOMP_NP_SUB_REQ_CTRL_OFF */

#define CSL_PCIE_RC_CORE_AMBA_MUL_OB_DECOMP_NP_SUB_REQ_CTRL_OFF_OB_RD_SPLIT_BURST_EN_MASK (0x00000001U)
#define CSL_PCIE_RC_CORE_AMBA_MUL_OB_DECOMP_NP_SUB_REQ_CTRL_OFF_OB_RD_SPLIT_BURST_EN_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_AMBA_MUL_OB_DECOMP_NP_SUB_REQ_CTRL_OFF_OB_RD_SPLIT_BURST_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_AMBA_MUL_OB_DECOMP_NP_SUB_REQ_CTRL_OFF_RSVDP_1_MASK      (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_AMBA_MUL_OB_DECOMP_NP_SUB_REQ_CTRL_OFF_RSVDP_1_SHIFT     (0x00000001U)
#define CSL_PCIE_RC_CORE_AMBA_MUL_OB_DECOMP_NP_SUB_REQ_CTRL_OFF_RSVDP_1_MAX       (0x7FFFFFFFU)

/* PL_DEBUG0_OFF */

#define CSL_PCIE_RC_CORE_PL_DEBUG0_OFF_DEB_REG_0_MASK                             (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_PL_DEBUG0_OFF_DEB_REG_0_SHIFT                            (0x00000000U)
#define CSL_PCIE_RC_CORE_PL_DEBUG0_OFF_DEB_REG_0_MAX                              (0xFFFFFFFFU)

/* PL_DEBUG1_OFF */

#define CSL_PCIE_RC_CORE_PL_DEBUG1_OFF_DEB_REG_1_MASK                             (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_PL_DEBUG1_OFF_DEB_REG_1_SHIFT                            (0x00000000U)
#define CSL_PCIE_RC_CORE_PL_DEBUG1_OFF_DEB_REG_1_MAX                              (0xFFFFFFFFU)

/* TX_P_FC_CREDIT_STATUS_OFF */

#define CSL_PCIE_RC_CORE_TX_P_FC_CREDIT_STATUS_OFF_TX_P_DATA_FC_CREDIT_MASK       (0x00000FFFU)
#define CSL_PCIE_RC_CORE_TX_P_FC_CREDIT_STATUS_OFF_TX_P_DATA_FC_CREDIT_SHIFT      (0x00000000U)
#define CSL_PCIE_RC_CORE_TX_P_FC_CREDIT_STATUS_OFF_TX_P_DATA_FC_CREDIT_MAX        (0x00000FFFU)

#define CSL_PCIE_RC_CORE_TX_P_FC_CREDIT_STATUS_OFF_TX_P_HEADER_FC_CREDIT_MASK     (0x000FF000U)
#define CSL_PCIE_RC_CORE_TX_P_FC_CREDIT_STATUS_OFF_TX_P_HEADER_FC_CREDIT_SHIFT    (0x0000000CU)
#define CSL_PCIE_RC_CORE_TX_P_FC_CREDIT_STATUS_OFF_TX_P_HEADER_FC_CREDIT_MAX      (0x000000FFU)

#define CSL_PCIE_RC_CORE_TX_P_FC_CREDIT_STATUS_OFF_RSVDP_20_MASK                  (0xFFF00000U)
#define CSL_PCIE_RC_CORE_TX_P_FC_CREDIT_STATUS_OFF_RSVDP_20_SHIFT                 (0x00000014U)
#define CSL_PCIE_RC_CORE_TX_P_FC_CREDIT_STATUS_OFF_RSVDP_20_MAX                   (0x00000FFFU)

/* TX_NP_FC_CREDIT_STATUS_OFF */

#define CSL_PCIE_RC_CORE_TX_NP_FC_CREDIT_STATUS_OFF_TX_NP_DATA_FC_CREDIT_MASK     (0x00000FFFU)
#define CSL_PCIE_RC_CORE_TX_NP_FC_CREDIT_STATUS_OFF_TX_NP_DATA_FC_CREDIT_SHIFT    (0x00000000U)
#define CSL_PCIE_RC_CORE_TX_NP_FC_CREDIT_STATUS_OFF_TX_NP_DATA_FC_CREDIT_MAX      (0x00000FFFU)

#define CSL_PCIE_RC_CORE_TX_NP_FC_CREDIT_STATUS_OFF_TX_NP_HEADER_FC_CREDIT_MASK   (0x000FF000U)
#define CSL_PCIE_RC_CORE_TX_NP_FC_CREDIT_STATUS_OFF_TX_NP_HEADER_FC_CREDIT_SHIFT  (0x0000000CU)
#define CSL_PCIE_RC_CORE_TX_NP_FC_CREDIT_STATUS_OFF_TX_NP_HEADER_FC_CREDIT_MAX    (0x000000FFU)

#define CSL_PCIE_RC_CORE_TX_NP_FC_CREDIT_STATUS_OFF_RSVDP_20_MASK                 (0xFFF00000U)
#define CSL_PCIE_RC_CORE_TX_NP_FC_CREDIT_STATUS_OFF_RSVDP_20_SHIFT                (0x00000014U)
#define CSL_PCIE_RC_CORE_TX_NP_FC_CREDIT_STATUS_OFF_RSVDP_20_MAX                  (0x00000FFFU)

/* TX_CPL_FC_CREDIT_STATUS_OFF */

#define CSL_PCIE_RC_CORE_TX_CPL_FC_CREDIT_STATUS_OFF_TX_CPL_DATA_FC_CREDIT_MASK   (0x00000FFFU)
#define CSL_PCIE_RC_CORE_TX_CPL_FC_CREDIT_STATUS_OFF_TX_CPL_DATA_FC_CREDIT_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_TX_CPL_FC_CREDIT_STATUS_OFF_TX_CPL_DATA_FC_CREDIT_MAX    (0x00000FFFU)

#define CSL_PCIE_RC_CORE_TX_CPL_FC_CREDIT_STATUS_OFF_TX_CPL_HEADER_FC_CREDIT_MASK (0x000FF000U)
#define CSL_PCIE_RC_CORE_TX_CPL_FC_CREDIT_STATUS_OFF_TX_CPL_HEADER_FC_CREDIT_SHIFT (0x0000000CU)
#define CSL_PCIE_RC_CORE_TX_CPL_FC_CREDIT_STATUS_OFF_TX_CPL_HEADER_FC_CREDIT_MAX  (0x000000FFU)

#define CSL_PCIE_RC_CORE_TX_CPL_FC_CREDIT_STATUS_OFF_RSVDP_20_MASK                (0xFFF00000U)
#define CSL_PCIE_RC_CORE_TX_CPL_FC_CREDIT_STATUS_OFF_RSVDP_20_SHIFT               (0x00000014U)
#define CSL_PCIE_RC_CORE_TX_CPL_FC_CREDIT_STATUS_OFF_RSVDP_20_MAX                 (0x00000FFFU)

/* QUEUE_STATUS_OFF */

#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_RX_TLP_FC_CREDIT_NON_RETURN_MASK        (0x00000001U)
#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_RX_TLP_FC_CREDIT_NON_RETURN_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_RX_TLP_FC_CREDIT_NON_RETURN_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_TX_RETRY_BUFFER_NE_MASK                 (0x00000002U)
#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_TX_RETRY_BUFFER_NE_SHIFT                (0x00000001U)
#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_TX_RETRY_BUFFER_NE_MAX                  (0x00000001U)

#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_RX_QUEUE_NON_EMPTY_MASK                 (0x00000004U)
#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_RX_QUEUE_NON_EMPTY_SHIFT                (0x00000002U)
#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_RX_QUEUE_NON_EMPTY_MAX                  (0x00000001U)

#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_RX_QUEUE_OVERFLOW_MASK                  (0x00000008U)
#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_RX_QUEUE_OVERFLOW_SHIFT                 (0x00000003U)
#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_RX_QUEUE_OVERFLOW_MAX                   (0x00000001U)

#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_RSVDP_4_MASK                            (0x00001FF0U)
#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_RSVDP_4_SHIFT                           (0x00000004U)
#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_RSVDP_4_MAX                             (0x000001FFU)

#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_RX_SERIALIZATION_Q_NON_EMPTY_MASK       (0x00002000U)
#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_RX_SERIALIZATION_Q_NON_EMPTY_SHIFT      (0x0000000DU)
#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_RX_SERIALIZATION_Q_NON_EMPTY_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_RX_SERIALIZATION_Q_WRITE_ERR_MASK       (0x00004000U)
#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_RX_SERIALIZATION_Q_WRITE_ERR_SHIFT      (0x0000000EU)
#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_RX_SERIALIZATION_Q_WRITE_ERR_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_RX_SERIALIZATION_Q_READ_ERR_MASK        (0x00008000U)
#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_RX_SERIALIZATION_Q_READ_ERR_SHIFT       (0x0000000FU)
#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_RX_SERIALIZATION_Q_READ_ERR_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_TIMER_MOD_FLOW_CONTROL_MASK             (0x1FFF0000U)
#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_TIMER_MOD_FLOW_CONTROL_SHIFT            (0x00000010U)
#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_TIMER_MOD_FLOW_CONTROL_MAX              (0x00001FFFU)

#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_RSVDP_29_MASK                           (0x60000000U)
#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_RSVDP_29_SHIFT                          (0x0000001DU)
#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_RSVDP_29_MAX                            (0x00000003U)

#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_TIMER_MOD_FLOW_CONTROL_EN_MASK          (0x80000000U)
#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_TIMER_MOD_FLOW_CONTROL_EN_SHIFT         (0x0000001FU)
#define CSL_PCIE_RC_CORE_QUEUE_STATUS_OFF_TIMER_MOD_FLOW_CONTROL_EN_MAX           (0x00000001U)

/* VC_TX_ARBI_1_OFF */

#define CSL_PCIE_RC_CORE_VC_TX_ARBI_1_OFF_WRR_WEIGHT_VC_0_MASK                    (0x000000FFU)
#define CSL_PCIE_RC_CORE_VC_TX_ARBI_1_OFF_WRR_WEIGHT_VC_0_SHIFT                   (0x00000000U)
#define CSL_PCIE_RC_CORE_VC_TX_ARBI_1_OFF_WRR_WEIGHT_VC_0_MAX                     (0x000000FFU)

#define CSL_PCIE_RC_CORE_VC_TX_ARBI_1_OFF_WRR_WEIGHT_VC_1_MASK                    (0x0000FF00U)
#define CSL_PCIE_RC_CORE_VC_TX_ARBI_1_OFF_WRR_WEIGHT_VC_1_SHIFT                   (0x00000008U)
#define CSL_PCIE_RC_CORE_VC_TX_ARBI_1_OFF_WRR_WEIGHT_VC_1_MAX                     (0x000000FFU)

#define CSL_PCIE_RC_CORE_VC_TX_ARBI_1_OFF_WRR_WEIGHT_VC_2_MASK                    (0x00FF0000U)
#define CSL_PCIE_RC_CORE_VC_TX_ARBI_1_OFF_WRR_WEIGHT_VC_2_SHIFT                   (0x00000010U)
#define CSL_PCIE_RC_CORE_VC_TX_ARBI_1_OFF_WRR_WEIGHT_VC_2_MAX                     (0x000000FFU)

#define CSL_PCIE_RC_CORE_VC_TX_ARBI_1_OFF_WRR_WEIGHT_VC_3_MASK                    (0xFF000000U)
#define CSL_PCIE_RC_CORE_VC_TX_ARBI_1_OFF_WRR_WEIGHT_VC_3_SHIFT                   (0x00000018U)
#define CSL_PCIE_RC_CORE_VC_TX_ARBI_1_OFF_WRR_WEIGHT_VC_3_MAX                     (0x000000FFU)

/* VC_TX_ARBI_2_OFF */

#define CSL_PCIE_RC_CORE_VC_TX_ARBI_2_OFF_WRR_WEIGHT_VC_4_MASK                    (0x000000FFU)
#define CSL_PCIE_RC_CORE_VC_TX_ARBI_2_OFF_WRR_WEIGHT_VC_4_SHIFT                   (0x00000000U)
#define CSL_PCIE_RC_CORE_VC_TX_ARBI_2_OFF_WRR_WEIGHT_VC_4_MAX                     (0x000000FFU)

#define CSL_PCIE_RC_CORE_VC_TX_ARBI_2_OFF_WRR_WEIGHT_VC_5_MASK                    (0x0000FF00U)
#define CSL_PCIE_RC_CORE_VC_TX_ARBI_2_OFF_WRR_WEIGHT_VC_5_SHIFT                   (0x00000008U)
#define CSL_PCIE_RC_CORE_VC_TX_ARBI_2_OFF_WRR_WEIGHT_VC_5_MAX                     (0x000000FFU)

#define CSL_PCIE_RC_CORE_VC_TX_ARBI_2_OFF_WRR_WEIGHT_VC_6_MASK                    (0x00FF0000U)
#define CSL_PCIE_RC_CORE_VC_TX_ARBI_2_OFF_WRR_WEIGHT_VC_6_SHIFT                   (0x00000010U)
#define CSL_PCIE_RC_CORE_VC_TX_ARBI_2_OFF_WRR_WEIGHT_VC_6_MAX                     (0x000000FFU)

#define CSL_PCIE_RC_CORE_VC_TX_ARBI_2_OFF_WRR_WEIGHT_VC_7_MASK                    (0xFF000000U)
#define CSL_PCIE_RC_CORE_VC_TX_ARBI_2_OFF_WRR_WEIGHT_VC_7_SHIFT                   (0x00000018U)
#define CSL_PCIE_RC_CORE_VC_TX_ARBI_2_OFF_WRR_WEIGHT_VC_7_MAX                     (0x000000FFU)

#ifdef CSL_MODIFICATION
/* VC0_P_RX_Q_CTRL_OFF */

#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF_VC0_P_DATA_CREDIT_MASK               (0x00000FFFU)
#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF_VC0_P_DATA_CREDIT_SHIFT              (0x00000000U)
#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF_VC0_P_DATA_CREDIT_MAX                (0x00000FFFU)

#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF_VC0_P_HEADER_CREDIT_MASK             (0x000FF000U)
#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF_VC0_P_HEADER_CREDIT_SHIFT            (0x0000000CU)
#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF_VC0_P_HEADER_CREDIT_MAX              (0x000000FFU)

#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF_RESERVED4_MASK                       (0x00100000U)
#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF_RESERVED4_SHIFT                      (0x00000014U)
#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF_RESERVED4_MAX                        (0x00000001U)

#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF_VC0_P_TLP_Q_MODE_MASK                (0x00E00000U)
#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF_VC0_P_TLP_Q_MODE_SHIFT               (0x00000015U)
#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF_VC0_P_TLP_Q_MODE_MAX                 (0x00000007U)

#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF_VC0_P_HDR_SCALE_MASK                 (0x03000000U)
#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF_VC0_P_HDR_SCALE_SHIFT                (0x00000018U)
#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF_VC0_P_HDR_SCALE_MAX                  (0x00000003U)

#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF_VC0_P_DATA_SCALE_MASK                (0x0C000000U)
#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF_VC0_P_DATA_SCALE_SHIFT               (0x0000001AU)
#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF_VC0_P_DATA_SCALE_MAX                 (0x00000003U)

#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF_RESERVED5_MASK                       (0x30000000U)
#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF_RESERVED5_SHIFT                      (0x0000001CU)
#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF_RESERVED5_MAX                        (0x00000003U)

#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF_TLP_TYPE_ORDERING_VC0_MASK           (0x40000000U)
#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF_TLP_TYPE_ORDERING_VC0_SHIFT          (0x0000001EU)
#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF_TLP_TYPE_ORDERING_VC0_MAX            (0x00000001U)

#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF_VC_ORDERING_RX_Q_MASK                (0x80000000U)
#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF_VC_ORDERING_RX_Q_SHIFT               (0x0000001FU)
#define CSL_PCIE_RC_CORE_VC0_P_RX_Q_CTRL_OFF_VC_ORDERING_RX_Q_MAX                 (0x00000001U)

/* VC0_NP_RX_Q_CTRL_OFF */

#define CSL_PCIE_RC_CORE_VC0_NP_RX_Q_CTRL_OFF_VC0_NP_DATA_CREDIT_MASK             (0x00000FFFU)
#define CSL_PCIE_RC_CORE_VC0_NP_RX_Q_CTRL_OFF_VC0_NP_DATA_CREDIT_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_VC0_NP_RX_Q_CTRL_OFF_VC0_NP_DATA_CREDIT_MAX              (0x00000FFFU)

#define CSL_PCIE_RC_CORE_VC0_NP_RX_Q_CTRL_OFF_VC0_NP_HEADER_CREDIT_MASK           (0x000FF000U)
#define CSL_PCIE_RC_CORE_VC0_NP_RX_Q_CTRL_OFF_VC0_NP_HEADER_CREDIT_SHIFT          (0x0000000CU)
#define CSL_PCIE_RC_CORE_VC0_NP_RX_Q_CTRL_OFF_VC0_NP_HEADER_CREDIT_MAX            (0x000000FFU)

#define CSL_PCIE_RC_CORE_VC0_NP_RX_Q_CTRL_OFF_RESERVED6_MASK                      (0x00100000U)
#define CSL_PCIE_RC_CORE_VC0_NP_RX_Q_CTRL_OFF_RESERVED6_SHIFT                     (0x00000014U)
#define CSL_PCIE_RC_CORE_VC0_NP_RX_Q_CTRL_OFF_RESERVED6_MAX                       (0x00000001U)

#define CSL_PCIE_RC_CORE_VC0_NP_RX_Q_CTRL_OFF_VC0_NP_TLP_Q_MODE_MASK              (0x00E00000U)
#define CSL_PCIE_RC_CORE_VC0_NP_RX_Q_CTRL_OFF_VC0_NP_TLP_Q_MODE_SHIFT             (0x00000015U)
#define CSL_PCIE_RC_CORE_VC0_NP_RX_Q_CTRL_OFF_VC0_NP_TLP_Q_MODE_MAX               (0x00000007U)

#define CSL_PCIE_RC_CORE_VC0_NP_RX_Q_CTRL_OFF_VC0_NP_HDR_SCALE_MASK               (0x03000000U)
#define CSL_PCIE_RC_CORE_VC0_NP_RX_Q_CTRL_OFF_VC0_NP_HDR_SCALE_SHIFT              (0x00000018U)
#define CSL_PCIE_RC_CORE_VC0_NP_RX_Q_CTRL_OFF_VC0_NP_HDR_SCALE_MAX                (0x00000003U)

#define CSL_PCIE_RC_CORE_VC0_NP_RX_Q_CTRL_OFF_VC0_NP_DATA_SCALE_MASK              (0x0C000000U)
#define CSL_PCIE_RC_CORE_VC0_NP_RX_Q_CTRL_OFF_VC0_NP_DATA_SCALE_SHIFT             (0x0000001AU)
#define CSL_PCIE_RC_CORE_VC0_NP_RX_Q_CTRL_OFF_VC0_NP_DATA_SCALE_MAX               (0x00000003U)

#define CSL_PCIE_RC_CORE_VC0_NP_RX_Q_CTRL_OFF_RESERVED7_MASK                      (0xF0000000U)
#define CSL_PCIE_RC_CORE_VC0_NP_RX_Q_CTRL_OFF_RESERVED7_SHIFT                     (0x0000001CU)
#define CSL_PCIE_RC_CORE_VC0_NP_RX_Q_CTRL_OFF_RESERVED7_MAX                       (0x0000000FU)

/* VC0_CPL_RX_Q_CTRL_OFF */

#define CSL_PCIE_RC_CORE_VC0_CPL_RX_Q_CTRL_OFF_VC0_CPL_DATA_CREDIT_MASK           (0x00000FFFU)
#define CSL_PCIE_RC_CORE_VC0_CPL_RX_Q_CTRL_OFF_VC0_CPL_DATA_CREDIT_SHIFT          (0x00000000U)
#define CSL_PCIE_RC_CORE_VC0_CPL_RX_Q_CTRL_OFF_VC0_CPL_DATA_CREDIT_MAX            (0x00000FFFU)

#define CSL_PCIE_RC_CORE_VC0_CPL_RX_Q_CTRL_OFF_VC0_CPL_HEADER_CREDIT_MASK         (0x000FF000U)
#define CSL_PCIE_RC_CORE_VC0_CPL_RX_Q_CTRL_OFF_VC0_CPL_HEADER_CREDIT_SHIFT        (0x0000000CU)
#define CSL_PCIE_RC_CORE_VC0_CPL_RX_Q_CTRL_OFF_VC0_CPL_HEADER_CREDIT_MAX          (0x000000FFU)

#define CSL_PCIE_RC_CORE_VC0_CPL_RX_Q_CTRL_OFF_RESERVED8_MASK                     (0x00100000U)
#define CSL_PCIE_RC_CORE_VC0_CPL_RX_Q_CTRL_OFF_RESERVED8_SHIFT                    (0x00000014U)
#define CSL_PCIE_RC_CORE_VC0_CPL_RX_Q_CTRL_OFF_RESERVED8_MAX                      (0x00000001U)

#define CSL_PCIE_RC_CORE_VC0_CPL_RX_Q_CTRL_OFF_VC0_CPL_TLP_Q_MODE_MASK            (0x00E00000U)
#define CSL_PCIE_RC_CORE_VC0_CPL_RX_Q_CTRL_OFF_VC0_CPL_TLP_Q_MODE_SHIFT           (0x00000015U)
#define CSL_PCIE_RC_CORE_VC0_CPL_RX_Q_CTRL_OFF_VC0_CPL_TLP_Q_MODE_MAX             (0x00000007U)

#define CSL_PCIE_RC_CORE_VC0_CPL_RX_Q_CTRL_OFF_VC0_CPL_HDR_SCALE_MASK             (0x03000000U)
#define CSL_PCIE_RC_CORE_VC0_CPL_RX_Q_CTRL_OFF_VC0_CPL_HDR_SCALE_SHIFT            (0x00000018U)
#define CSL_PCIE_RC_CORE_VC0_CPL_RX_Q_CTRL_OFF_VC0_CPL_HDR_SCALE_MAX              (0x00000003U)

#define CSL_PCIE_RC_CORE_VC0_CPL_RX_Q_CTRL_OFF_VC0_CPL_DATA_SCALE_MASK            (0x0C000000U)
#define CSL_PCIE_RC_CORE_VC0_CPL_RX_Q_CTRL_OFF_VC0_CPL_DATA_SCALE_SHIFT           (0x0000001AU)
#define CSL_PCIE_RC_CORE_VC0_CPL_RX_Q_CTRL_OFF_VC0_CPL_DATA_SCALE_MAX             (0x00000003U)

#define CSL_PCIE_RC_CORE_VC0_CPL_RX_Q_CTRL_OFF_RESERVED9_MASK                     (0xF0000000U)
#define CSL_PCIE_RC_CORE_VC0_CPL_RX_Q_CTRL_OFF_RESERVED9_SHIFT                    (0x0000001CU)
#define CSL_PCIE_RC_CORE_VC0_CPL_RX_Q_CTRL_OFF_RESERVED9_MAX                      (0x0000000FU)

/* VC1_P_RX_Q_CTRL_OFF */

#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF_VC1_P_DATA_CREDIT_MASK               (0x00000FFFU)
#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF_VC1_P_DATA_CREDIT_SHIFT              (0x00000000U)
#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF_VC1_P_DATA_CREDIT_MAX                (0x00000FFFU)

#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF_VC1_P_HEADER_CREDIT_MASK             (0x000FF000U)
#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF_VC1_P_HEADER_CREDIT_SHIFT            (0x0000000CU)
#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF_VC1_P_HEADER_CREDIT_MAX              (0x000000FFU)

#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF_RESERVED0_VC1_MASK                   (0x00100000U)
#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF_RESERVED0_VC1_SHIFT                  (0x00000014U)
#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF_RESERVED0_VC1_MAX                    (0x00000001U)

#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF_VC1_P_TLP_Q_MODE_MASK                (0x00E00000U)
#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF_VC1_P_TLP_Q_MODE_SHIFT               (0x00000015U)
#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF_VC1_P_TLP_Q_MODE_MAX                 (0x00000007U)

#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF_VC1_P_HDR_SCALE_MASK                 (0x03000000U)
#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF_VC1_P_HDR_SCALE_SHIFT                (0x00000018U)
#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF_VC1_P_HDR_SCALE_MAX                  (0x00000003U)

#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF_VC1_P_DATA_SCALE_MASK                (0x0C000000U)
#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF_VC1_P_DATA_SCALE_SHIFT               (0x0000001AU)
#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF_VC1_P_DATA_SCALE_MAX                 (0x00000003U)

#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF_RESERVED1_VC1_MASK                   (0x30000000U)
#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF_RESERVED1_VC1_SHIFT                  (0x0000001CU)
#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF_RESERVED1_VC1_MAX                    (0x00000003U)

#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF_TLP_TYPE_ORDERING_VC1_MASK           (0x40000000U)
#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF_TLP_TYPE_ORDERING_VC1_SHIFT          (0x0000001EU)
#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF_TLP_TYPE_ORDERING_VC1_MAX            (0x00000001U)

#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF_RESERVED2_VC1_MASK                   (0x80000000U)
#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF_RESERVED2_VC1_SHIFT                  (0x0000001FU)
#define CSL_PCIE_RC_CORE_VC1_P_RX_Q_CTRL_OFF_RESERVED2_VC1_MAX                    (0x00000001U)

/* VC1_NP_RX_Q_CTRL_OFF */

#define CSL_PCIE_RC_CORE_VC1_NP_RX_Q_CTRL_OFF_VC1_NP_DATA_CREDIT_MASK             (0x00000FFFU)
#define CSL_PCIE_RC_CORE_VC1_NP_RX_Q_CTRL_OFF_VC1_NP_DATA_CREDIT_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_VC1_NP_RX_Q_CTRL_OFF_VC1_NP_DATA_CREDIT_MAX              (0x00000FFFU)

#define CSL_PCIE_RC_CORE_VC1_NP_RX_Q_CTRL_OFF_VC1_NP_HEADER_CREDIT_MASK           (0x000FF000U)
#define CSL_PCIE_RC_CORE_VC1_NP_RX_Q_CTRL_OFF_VC1_NP_HEADER_CREDIT_SHIFT          (0x0000000CU)
#define CSL_PCIE_RC_CORE_VC1_NP_RX_Q_CTRL_OFF_VC1_NP_HEADER_CREDIT_MAX            (0x000000FFU)

#define CSL_PCIE_RC_CORE_VC1_NP_RX_Q_CTRL_OFF_RESERVED3_VC1_MASK                  (0x00100000U)
#define CSL_PCIE_RC_CORE_VC1_NP_RX_Q_CTRL_OFF_RESERVED3_VC1_SHIFT                 (0x00000014U)
#define CSL_PCIE_RC_CORE_VC1_NP_RX_Q_CTRL_OFF_RESERVED3_VC1_MAX                   (0x00000001U)

#define CSL_PCIE_RC_CORE_VC1_NP_RX_Q_CTRL_OFF_VC1_NP_TLP_Q_MODE_MASK              (0x00E00000U)
#define CSL_PCIE_RC_CORE_VC1_NP_RX_Q_CTRL_OFF_VC1_NP_TLP_Q_MODE_SHIFT             (0x00000015U)
#define CSL_PCIE_RC_CORE_VC1_NP_RX_Q_CTRL_OFF_VC1_NP_TLP_Q_MODE_MAX               (0x00000007U)

#define CSL_PCIE_RC_CORE_VC1_NP_RX_Q_CTRL_OFF_VC1_NP_HDR_SCALE_MASK               (0x03000000U)
#define CSL_PCIE_RC_CORE_VC1_NP_RX_Q_CTRL_OFF_VC1_NP_HDR_SCALE_SHIFT              (0x00000018U)
#define CSL_PCIE_RC_CORE_VC1_NP_RX_Q_CTRL_OFF_VC1_NP_HDR_SCALE_MAX                (0x00000003U)

#define CSL_PCIE_RC_CORE_VC1_NP_RX_Q_CTRL_OFF_VC1_NP_DATA_SCALE_MASK              (0x0C000000U)
#define CSL_PCIE_RC_CORE_VC1_NP_RX_Q_CTRL_OFF_VC1_NP_DATA_SCALE_SHIFT             (0x0000001AU)
#define CSL_PCIE_RC_CORE_VC1_NP_RX_Q_CTRL_OFF_VC1_NP_DATA_SCALE_MAX               (0x00000003U)

#define CSL_PCIE_RC_CORE_VC1_NP_RX_Q_CTRL_OFF_RESERVED4_VC1_MASK                  (0xF0000000U)
#define CSL_PCIE_RC_CORE_VC1_NP_RX_Q_CTRL_OFF_RESERVED4_VC1_SHIFT                 (0x0000001CU)
#define CSL_PCIE_RC_CORE_VC1_NP_RX_Q_CTRL_OFF_RESERVED4_VC1_MAX                   (0x0000000FU)

/* VC1_CPL_RX_Q_CTRL_OFF */

#define CSL_PCIE_RC_CORE_VC1_CPL_RX_Q_CTRL_OFF_VC1_CPL_DATA_CREDIT_MASK           (0x00000FFFU)
#define CSL_PCIE_RC_CORE_VC1_CPL_RX_Q_CTRL_OFF_VC1_CPL_DATA_CREDIT_SHIFT          (0x00000000U)
#define CSL_PCIE_RC_CORE_VC1_CPL_RX_Q_CTRL_OFF_VC1_CPL_DATA_CREDIT_MAX            (0x00000FFFU)

#define CSL_PCIE_RC_CORE_VC1_CPL_RX_Q_CTRL_OFF_VC1_CPL_HEADER_CREDIT_MASK         (0x000FF000U)
#define CSL_PCIE_RC_CORE_VC1_CPL_RX_Q_CTRL_OFF_VC1_CPL_HEADER_CREDIT_SHIFT        (0x0000000CU)
#define CSL_PCIE_RC_CORE_VC1_CPL_RX_Q_CTRL_OFF_VC1_CPL_HEADER_CREDIT_MAX          (0x000000FFU)

#define CSL_PCIE_RC_CORE_VC1_CPL_RX_Q_CTRL_OFF_RESERVED5_VC1_MASK                 (0x00100000U)
#define CSL_PCIE_RC_CORE_VC1_CPL_RX_Q_CTRL_OFF_RESERVED5_VC1_SHIFT                (0x00000014U)
#define CSL_PCIE_RC_CORE_VC1_CPL_RX_Q_CTRL_OFF_RESERVED5_VC1_MAX                  (0x00000001U)

#define CSL_PCIE_RC_CORE_VC1_CPL_RX_Q_CTRL_OFF_VC1_CPL_TLP_Q_MODE_MASK            (0x00E00000U)
#define CSL_PCIE_RC_CORE_VC1_CPL_RX_Q_CTRL_OFF_VC1_CPL_TLP_Q_MODE_SHIFT           (0x00000015U)
#define CSL_PCIE_RC_CORE_VC1_CPL_RX_Q_CTRL_OFF_VC1_CPL_TLP_Q_MODE_MAX             (0x00000007U)

#define CSL_PCIE_RC_CORE_VC1_CPL_RX_Q_CTRL_OFF_VC1_CPL_HDR_SCALE_MASK             (0x03000000U)
#define CSL_PCIE_RC_CORE_VC1_CPL_RX_Q_CTRL_OFF_VC1_CPL_HDR_SCALE_SHIFT            (0x00000018U)
#define CSL_PCIE_RC_CORE_VC1_CPL_RX_Q_CTRL_OFF_VC1_CPL_HDR_SCALE_MAX              (0x00000003U)

#define CSL_PCIE_RC_CORE_VC1_CPL_RX_Q_CTRL_OFF_VC1_CPL_DATA_SCALE_MASK            (0x0C000000U)
#define CSL_PCIE_RC_CORE_VC1_CPL_RX_Q_CTRL_OFF_VC1_CPL_DATA_SCALE_SHIFT           (0x0000001AU)
#define CSL_PCIE_RC_CORE_VC1_CPL_RX_Q_CTRL_OFF_VC1_CPL_DATA_SCALE_MAX             (0x00000003U)

#define CSL_PCIE_RC_CORE_VC1_CPL_RX_Q_CTRL_OFF_RESERVED6_VC1_MASK                 (0xF0000000U)
#define CSL_PCIE_RC_CORE_VC1_CPL_RX_Q_CTRL_OFF_RESERVED6_VC1_SHIFT                (0x0000001CU)
#define CSL_PCIE_RC_CORE_VC1_CPL_RX_Q_CTRL_OFF_RESERVED6_VC1_MAX                  (0x0000000FU)

/* VC2_P_RX_Q_CTRL_OFF */

#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF_VC2_P_DATA_CREDIT_MASK               (0x00000FFFU)
#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF_VC2_P_DATA_CREDIT_SHIFT              (0x00000000U)
#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF_VC2_P_DATA_CREDIT_MAX                (0x00000FFFU)

#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF_VC2_P_HEADER_CREDIT_MASK             (0x000FF000U)
#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF_VC2_P_HEADER_CREDIT_SHIFT            (0x0000000CU)
#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF_VC2_P_HEADER_CREDIT_MAX              (0x000000FFU)

#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF_RESERVED0_VC2_MASK                   (0x00100000U)
#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF_RESERVED0_VC2_SHIFT                  (0x00000014U)
#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF_RESERVED0_VC2_MAX                    (0x00000001U)

#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF_VC2_P_TLP_Q_MODE_MASK                (0x00E00000U)
#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF_VC2_P_TLP_Q_MODE_SHIFT               (0x00000015U)
#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF_VC2_P_TLP_Q_MODE_MAX                 (0x00000007U)

#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF_VC2_P_HDR_SCALE_MASK                 (0x03000000U)
#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF_VC2_P_HDR_SCALE_SHIFT                (0x00000018U)
#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF_VC2_P_HDR_SCALE_MAX                  (0x00000003U)

#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF_VC2_P_DATA_SCALE_MASK                (0x0C000000U)
#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF_VC2_P_DATA_SCALE_SHIFT               (0x0000001AU)
#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF_VC2_P_DATA_SCALE_MAX                 (0x00000003U)

#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF_RESERVED1_VC2_MASK                   (0x30000000U)
#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF_RESERVED1_VC2_SHIFT                  (0x0000001CU)
#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF_RESERVED1_VC2_MAX                    (0x00000003U)

#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF_TLP_TYPE_ORDERING_VC2_MASK           (0x40000000U)
#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF_TLP_TYPE_ORDERING_VC2_SHIFT          (0x0000001EU)
#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF_TLP_TYPE_ORDERING_VC2_MAX            (0x00000001U)

#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF_RESERVED2_VC2_MASK                   (0x80000000U)
#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF_RESERVED2_VC2_SHIFT                  (0x0000001FU)
#define CSL_PCIE_RC_CORE_VC2_P_RX_Q_CTRL_OFF_RESERVED2_VC2_MAX                    (0x00000001U)

/* VC2_NP_RX_Q_CTRL_OFF */

#define CSL_PCIE_RC_CORE_VC2_NP_RX_Q_CTRL_OFF_VC2_NP_DATA_CREDIT_MASK             (0x00000FFFU)
#define CSL_PCIE_RC_CORE_VC2_NP_RX_Q_CTRL_OFF_VC2_NP_DATA_CREDIT_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_VC2_NP_RX_Q_CTRL_OFF_VC2_NP_DATA_CREDIT_MAX              (0x00000FFFU)

#define CSL_PCIE_RC_CORE_VC2_NP_RX_Q_CTRL_OFF_VC2_NP_HEADER_CREDIT_MASK           (0x000FF000U)
#define CSL_PCIE_RC_CORE_VC2_NP_RX_Q_CTRL_OFF_VC2_NP_HEADER_CREDIT_SHIFT          (0x0000000CU)
#define CSL_PCIE_RC_CORE_VC2_NP_RX_Q_CTRL_OFF_VC2_NP_HEADER_CREDIT_MAX            (0x000000FFU)

#define CSL_PCIE_RC_CORE_VC2_NP_RX_Q_CTRL_OFF_RESERVED3_VC2_MASK                  (0x00100000U)
#define CSL_PCIE_RC_CORE_VC2_NP_RX_Q_CTRL_OFF_RESERVED3_VC2_SHIFT                 (0x00000014U)
#define CSL_PCIE_RC_CORE_VC2_NP_RX_Q_CTRL_OFF_RESERVED3_VC2_MAX                   (0x00000001U)

#define CSL_PCIE_RC_CORE_VC2_NP_RX_Q_CTRL_OFF_VC2_NP_TLP_Q_MODE_MASK              (0x00E00000U)
#define CSL_PCIE_RC_CORE_VC2_NP_RX_Q_CTRL_OFF_VC2_NP_TLP_Q_MODE_SHIFT             (0x00000015U)
#define CSL_PCIE_RC_CORE_VC2_NP_RX_Q_CTRL_OFF_VC2_NP_TLP_Q_MODE_MAX               (0x00000007U)

#define CSL_PCIE_RC_CORE_VC2_NP_RX_Q_CTRL_OFF_VC2_NP_HDR_SCALE_MASK               (0x03000000U)
#define CSL_PCIE_RC_CORE_VC2_NP_RX_Q_CTRL_OFF_VC2_NP_HDR_SCALE_SHIFT              (0x00000018U)
#define CSL_PCIE_RC_CORE_VC2_NP_RX_Q_CTRL_OFF_VC2_NP_HDR_SCALE_MAX                (0x00000003U)

#define CSL_PCIE_RC_CORE_VC2_NP_RX_Q_CTRL_OFF_VC2_NP_DATA_SCALE_MASK              (0x0C000000U)
#define CSL_PCIE_RC_CORE_VC2_NP_RX_Q_CTRL_OFF_VC2_NP_DATA_SCALE_SHIFT             (0x0000001AU)
#define CSL_PCIE_RC_CORE_VC2_NP_RX_Q_CTRL_OFF_VC2_NP_DATA_SCALE_MAX               (0x00000003U)

#define CSL_PCIE_RC_CORE_VC2_NP_RX_Q_CTRL_OFF_RESERVED4_VC2_MASK                  (0xF0000000U)
#define CSL_PCIE_RC_CORE_VC2_NP_RX_Q_CTRL_OFF_RESERVED4_VC2_SHIFT                 (0x0000001CU)
#define CSL_PCIE_RC_CORE_VC2_NP_RX_Q_CTRL_OFF_RESERVED4_VC2_MAX                   (0x0000000FU)

/* VC2_CPL_RX_Q_CTRL_OFF */

#define CSL_PCIE_RC_CORE_VC2_CPL_RX_Q_CTRL_OFF_VC2_CPL_DATA_CREDIT_MASK           (0x00000FFFU)
#define CSL_PCIE_RC_CORE_VC2_CPL_RX_Q_CTRL_OFF_VC2_CPL_DATA_CREDIT_SHIFT          (0x00000000U)
#define CSL_PCIE_RC_CORE_VC2_CPL_RX_Q_CTRL_OFF_VC2_CPL_DATA_CREDIT_MAX            (0x00000FFFU)

#define CSL_PCIE_RC_CORE_VC2_CPL_RX_Q_CTRL_OFF_VC2_CPL_HEADER_CREDIT_MASK         (0x000FF000U)
#define CSL_PCIE_RC_CORE_VC2_CPL_RX_Q_CTRL_OFF_VC2_CPL_HEADER_CREDIT_SHIFT        (0x0000000CU)
#define CSL_PCIE_RC_CORE_VC2_CPL_RX_Q_CTRL_OFF_VC2_CPL_HEADER_CREDIT_MAX          (0x000000FFU)

#define CSL_PCIE_RC_CORE_VC2_CPL_RX_Q_CTRL_OFF_RESERVED5_VC2_MASK                 (0x00100000U)
#define CSL_PCIE_RC_CORE_VC2_CPL_RX_Q_CTRL_OFF_RESERVED5_VC2_SHIFT                (0x00000014U)
#define CSL_PCIE_RC_CORE_VC2_CPL_RX_Q_CTRL_OFF_RESERVED5_VC2_MAX                  (0x00000001U)

#define CSL_PCIE_RC_CORE_VC2_CPL_RX_Q_CTRL_OFF_VC2_CPL_TLP_Q_MODE_MASK            (0x00E00000U)
#define CSL_PCIE_RC_CORE_VC2_CPL_RX_Q_CTRL_OFF_VC2_CPL_TLP_Q_MODE_SHIFT           (0x00000015U)
#define CSL_PCIE_RC_CORE_VC2_CPL_RX_Q_CTRL_OFF_VC2_CPL_TLP_Q_MODE_MAX             (0x00000007U)

#define CSL_PCIE_RC_CORE_VC2_CPL_RX_Q_CTRL_OFF_VC2_CPL_HDR_SCALE_MASK             (0x03000000U)
#define CSL_PCIE_RC_CORE_VC2_CPL_RX_Q_CTRL_OFF_VC2_CPL_HDR_SCALE_SHIFT            (0x00000018U)
#define CSL_PCIE_RC_CORE_VC2_CPL_RX_Q_CTRL_OFF_VC2_CPL_HDR_SCALE_MAX              (0x00000003U)

#define CSL_PCIE_RC_CORE_VC2_CPL_RX_Q_CTRL_OFF_VC2_CPL_DATA_SCALE_MASK            (0x0C000000U)
#define CSL_PCIE_RC_CORE_VC2_CPL_RX_Q_CTRL_OFF_VC2_CPL_DATA_SCALE_SHIFT           (0x0000001AU)
#define CSL_PCIE_RC_CORE_VC2_CPL_RX_Q_CTRL_OFF_VC2_CPL_DATA_SCALE_MAX             (0x00000003U)

#define CSL_PCIE_RC_CORE_VC2_CPL_RX_Q_CTRL_OFF_RESERVED6_VC2_MASK                 (0xF0000000U)
#define CSL_PCIE_RC_CORE_VC2_CPL_RX_Q_CTRL_OFF_RESERVED6_VC2_SHIFT                (0x0000001CU)
#define CSL_PCIE_RC_CORE_VC2_CPL_RX_Q_CTRL_OFF_RESERVED6_VC2_MAX                  (0x0000000FU)

/* VC3_P_RX_Q_CTRL_OFF */

#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF_VC3_P_DATA_CREDIT_MASK               (0x00000FFFU)
#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF_VC3_P_DATA_CREDIT_SHIFT              (0x00000000U)
#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF_VC3_P_DATA_CREDIT_MAX                (0x00000FFFU)

#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF_VC3_P_HEADER_CREDIT_MASK             (0x000FF000U)
#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF_VC3_P_HEADER_CREDIT_SHIFT            (0x0000000CU)
#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF_VC3_P_HEADER_CREDIT_MAX              (0x000000FFU)

#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF_RESERVED0_VC3_MASK                   (0x00100000U)
#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF_RESERVED0_VC3_SHIFT                  (0x00000014U)
#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF_RESERVED0_VC3_MAX                    (0x00000001U)

#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF_VC3_P_TLP_Q_MODE_MASK                (0x00E00000U)
#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF_VC3_P_TLP_Q_MODE_SHIFT               (0x00000015U)
#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF_VC3_P_TLP_Q_MODE_MAX                 (0x00000007U)

#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF_VC3_P_HDR_SCALE_MASK                 (0x03000000U)
#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF_VC3_P_HDR_SCALE_SHIFT                (0x00000018U)
#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF_VC3_P_HDR_SCALE_MAX                  (0x00000003U)

#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF_VC3_P_DATA_SCALE_MASK                (0x0C000000U)
#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF_VC3_P_DATA_SCALE_SHIFT               (0x0000001AU)
#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF_VC3_P_DATA_SCALE_MAX                 (0x00000003U)

#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF_RESERVED1_VC3_MASK                   (0x30000000U)
#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF_RESERVED1_VC3_SHIFT                  (0x0000001CU)
#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF_RESERVED1_VC3_MAX                    (0x00000003U)

#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF_TLP_TYPE_ORDERING_VC3_MASK           (0x40000000U)
#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF_TLP_TYPE_ORDERING_VC3_SHIFT          (0x0000001EU)
#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF_TLP_TYPE_ORDERING_VC3_MAX            (0x00000001U)

#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF_RESERVED2_VC3_MASK                   (0x80000000U)
#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF_RESERVED2_VC3_SHIFT                  (0x0000001FU)
#define CSL_PCIE_RC_CORE_VC3_P_RX_Q_CTRL_OFF_RESERVED2_VC3_MAX                    (0x00000001U)

/* VC3_NP_RX_Q_CTRL_OFF */

#define CSL_PCIE_RC_CORE_VC3_NP_RX_Q_CTRL_OFF_VC3_NP_DATA_CREDIT_MASK             (0x00000FFFU)
#define CSL_PCIE_RC_CORE_VC3_NP_RX_Q_CTRL_OFF_VC3_NP_DATA_CREDIT_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_VC3_NP_RX_Q_CTRL_OFF_VC3_NP_DATA_CREDIT_MAX              (0x00000FFFU)

#define CSL_PCIE_RC_CORE_VC3_NP_RX_Q_CTRL_OFF_VC3_NP_HEADER_CREDIT_MASK           (0x000FF000U)
#define CSL_PCIE_RC_CORE_VC3_NP_RX_Q_CTRL_OFF_VC3_NP_HEADER_CREDIT_SHIFT          (0x0000000CU)
#define CSL_PCIE_RC_CORE_VC3_NP_RX_Q_CTRL_OFF_VC3_NP_HEADER_CREDIT_MAX            (0x000000FFU)

#define CSL_PCIE_RC_CORE_VC3_NP_RX_Q_CTRL_OFF_RESERVED3_VC3_MASK                  (0x00100000U)
#define CSL_PCIE_RC_CORE_VC3_NP_RX_Q_CTRL_OFF_RESERVED3_VC3_SHIFT                 (0x00000014U)
#define CSL_PCIE_RC_CORE_VC3_NP_RX_Q_CTRL_OFF_RESERVED3_VC3_MAX                   (0x00000001U)

#define CSL_PCIE_RC_CORE_VC3_NP_RX_Q_CTRL_OFF_VC3_NP_TLP_Q_MODE_MASK              (0x00E00000U)
#define CSL_PCIE_RC_CORE_VC3_NP_RX_Q_CTRL_OFF_VC3_NP_TLP_Q_MODE_SHIFT             (0x00000015U)
#define CSL_PCIE_RC_CORE_VC3_NP_RX_Q_CTRL_OFF_VC3_NP_TLP_Q_MODE_MAX               (0x00000007U)

#define CSL_PCIE_RC_CORE_VC3_NP_RX_Q_CTRL_OFF_VC3_NP_HDR_SCALE_MASK               (0x03000000U)
#define CSL_PCIE_RC_CORE_VC3_NP_RX_Q_CTRL_OFF_VC3_NP_HDR_SCALE_SHIFT              (0x00000018U)
#define CSL_PCIE_RC_CORE_VC3_NP_RX_Q_CTRL_OFF_VC3_NP_HDR_SCALE_MAX                (0x00000003U)

#define CSL_PCIE_RC_CORE_VC3_NP_RX_Q_CTRL_OFF_VC3_NP_DATA_SCALE_MASK              (0x0C000000U)
#define CSL_PCIE_RC_CORE_VC3_NP_RX_Q_CTRL_OFF_VC3_NP_DATA_SCALE_SHIFT             (0x0000001AU)
#define CSL_PCIE_RC_CORE_VC3_NP_RX_Q_CTRL_OFF_VC3_NP_DATA_SCALE_MAX               (0x00000003U)

#define CSL_PCIE_RC_CORE_VC3_NP_RX_Q_CTRL_OFF_RESERVED4_VC3_MASK                  (0xF0000000U)
#define CSL_PCIE_RC_CORE_VC3_NP_RX_Q_CTRL_OFF_RESERVED4_VC3_SHIFT                 (0x0000001CU)
#define CSL_PCIE_RC_CORE_VC3_NP_RX_Q_CTRL_OFF_RESERVED4_VC3_MAX                   (0x0000000FU)

/* VC3_CPL_RX_Q_CTRL_OFF */

#define CSL_PCIE_RC_CORE_VC3_CPL_RX_Q_CTRL_OFF_VC3_CPL_DATA_CREDIT_MASK           (0x00000FFFU)
#define CSL_PCIE_RC_CORE_VC3_CPL_RX_Q_CTRL_OFF_VC3_CPL_DATA_CREDIT_SHIFT          (0x00000000U)
#define CSL_PCIE_RC_CORE_VC3_CPL_RX_Q_CTRL_OFF_VC3_CPL_DATA_CREDIT_MAX            (0x00000FFFU)

#define CSL_PCIE_RC_CORE_VC3_CPL_RX_Q_CTRL_OFF_VC3_CPL_HEADER_CREDIT_MASK         (0x000FF000U)
#define CSL_PCIE_RC_CORE_VC3_CPL_RX_Q_CTRL_OFF_VC3_CPL_HEADER_CREDIT_SHIFT        (0x0000000CU)
#define CSL_PCIE_RC_CORE_VC3_CPL_RX_Q_CTRL_OFF_VC3_CPL_HEADER_CREDIT_MAX          (0x000000FFU)

#define CSL_PCIE_RC_CORE_VC3_CPL_RX_Q_CTRL_OFF_RESERVED5_VC3_MASK                 (0x00100000U)
#define CSL_PCIE_RC_CORE_VC3_CPL_RX_Q_CTRL_OFF_RESERVED5_VC3_SHIFT                (0x00000014U)
#define CSL_PCIE_RC_CORE_VC3_CPL_RX_Q_CTRL_OFF_RESERVED5_VC3_MAX                  (0x00000001U)

#define CSL_PCIE_RC_CORE_VC3_CPL_RX_Q_CTRL_OFF_VC3_CPL_TLP_Q_MODE_MASK            (0x00E00000U)
#define CSL_PCIE_RC_CORE_VC3_CPL_RX_Q_CTRL_OFF_VC3_CPL_TLP_Q_MODE_SHIFT           (0x00000015U)
#define CSL_PCIE_RC_CORE_VC3_CPL_RX_Q_CTRL_OFF_VC3_CPL_TLP_Q_MODE_MAX             (0x00000007U)

#define CSL_PCIE_RC_CORE_VC3_CPL_RX_Q_CTRL_OFF_VC3_CPL_HDR_SCALE_MASK             (0x03000000U)
#define CSL_PCIE_RC_CORE_VC3_CPL_RX_Q_CTRL_OFF_VC3_CPL_HDR_SCALE_SHIFT            (0x00000018U)
#define CSL_PCIE_RC_CORE_VC3_CPL_RX_Q_CTRL_OFF_VC3_CPL_HDR_SCALE_MAX              (0x00000003U)

#define CSL_PCIE_RC_CORE_VC3_CPL_RX_Q_CTRL_OFF_VC3_CPL_DATA_SCALE_MASK            (0x0C000000U)
#define CSL_PCIE_RC_CORE_VC3_CPL_RX_Q_CTRL_OFF_VC3_CPL_DATA_SCALE_SHIFT           (0x0000001AU)
#define CSL_PCIE_RC_CORE_VC3_CPL_RX_Q_CTRL_OFF_VC3_CPL_DATA_SCALE_MAX             (0x00000003U)

#define CSL_PCIE_RC_CORE_VC3_CPL_RX_Q_CTRL_OFF_RESERVED6_VC3_MASK                 (0xF0000000U)
#define CSL_PCIE_RC_CORE_VC3_CPL_RX_Q_CTRL_OFF_RESERVED6_VC3_SHIFT                (0x0000001CU)
#define CSL_PCIE_RC_CORE_VC3_CPL_RX_Q_CTRL_OFF_RESERVED6_VC3_MAX                  (0x0000000FU)

#else /* CSL_MODIFICATION */

#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_DATA_CREDIT_MASK                 (0x00000FFFU)
#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_DATA_CREDIT_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_DATA_CREDIT_MAX                  (0x00000FFFU)

#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_HEADER_CREDIT_MASK               (0x000FF000U)
#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_HEADER_CREDIT_SHIFT              (0x0000000CU)
#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_HEADER_CREDIT_MAX                (0x000000FFU)

#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF_RESERVED4_MASK                        (0x00100000U)
#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF_RESERVED4_SHIFT                       (0x00000014U)
#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF_RESERVED4_MAX                         (0x00000001U)

#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_TLP_Q_MODE_MASK                  (0x00E00000U)
#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_TLP_Q_MODE_SHIFT                 (0x00000015U)
#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_TLP_Q_MODE_MAX                   (0x00000007U)

#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_HDR_SCALE_MASK                   (0x03000000U)
#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_HDR_SCALE_SHIFT                  (0x00000018U)
#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_HDR_SCALE_MAX                    (0x00000003U)

#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_DATA_SCALE_MASK                  (0x0C000000U)
#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_DATA_SCALE_SHIFT                 (0x0000001AU)
#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_DATA_SCALE_MAX                   (0x00000003U)

#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF_RESERVED5_MASK                        (0x30000000U)
#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF_RESERVED5_SHIFT                       (0x0000001CU)
#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF_RESERVED5_MAX                         (0x00000003U)

#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF_TLP_TYPE_ORDERING_VC_MASK             (0x40000000U)
#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF_TLP_TYPE_ORDERING_VC_SHIFT            (0x0000001EU)
#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF_TLP_TYPE_ORDERING_VC_MAX              (0x00000001U)

#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF_VC_ORDERING_RX_Q_MASK                 (0x80000000U)
#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF_VC_ORDERING_RX_Q_SHIFT                (0x0000001FU)
#define CSL_PCIE_RC_CORE_VC_P_RX_Q_CTRL_OFF_VC_ORDERING_RX_Q_MAX                  (0x00000001U)

/* VC_NP_RX_Q_CTRL_OFF */

#define CSL_PCIE_RC_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_DATA_CREDIT_MASK               (0x00000FFFU)
#define CSL_PCIE_RC_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_DATA_CREDIT_SHIFT              (0x00000000U)
#define CSL_PCIE_RC_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_DATA_CREDIT_MAX                (0x00000FFFU)

#define CSL_PCIE_RC_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_HEADER_CREDIT_MASK             (0x000FF000U)
#define CSL_PCIE_RC_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_HEADER_CREDIT_SHIFT            (0x0000000CU)
#define CSL_PCIE_RC_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_HEADER_CREDIT_MAX              (0x000000FFU)

#define CSL_PCIE_RC_CORE_VC_NP_RX_Q_CTRL_OFF_RESERVED6_MASK                       (0x00100000U)
#define CSL_PCIE_RC_CORE_VC_NP_RX_Q_CTRL_OFF_RESERVED6_SHIFT                      (0x00000014U)
#define CSL_PCIE_RC_CORE_VC_NP_RX_Q_CTRL_OFF_RESERVED6_MAX                        (0x00000001U)

#define CSL_PCIE_RC_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_TLP_Q_MODE_MASK                (0x00E00000U)
#define CSL_PCIE_RC_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_TLP_Q_MODE_SHIFT               (0x00000015U)
#define CSL_PCIE_RC_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_TLP_Q_MODE_MAX                 (0x00000007U)

#define CSL_PCIE_RC_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_HDR_SCALE_MASK                 (0x03000000U)
#define CSL_PCIE_RC_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_HDR_SCALE_SHIFT                (0x00000018U)
#define CSL_PCIE_RC_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_HDR_SCALE_MAX                  (0x00000003U)

#define CSL_PCIE_RC_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_DATA_SCALE_MASK                (0x0C000000U)
#define CSL_PCIE_RC_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_DATA_SCALE_SHIFT               (0x0000001AU)
#define CSL_PCIE_RC_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_DATA_SCALE_MAX                 (0x00000003U)

#define CSL_PCIE_RC_CORE_VC_NP_RX_Q_CTRL_OFF_RESERVED7_MASK                       (0xF0000000U)
#define CSL_PCIE_RC_CORE_VC_NP_RX_Q_CTRL_OFF_RESERVED7_SHIFT                      (0x0000001CU)
#define CSL_PCIE_RC_CORE_VC_NP_RX_Q_CTRL_OFF_RESERVED7_MAX                        (0x0000000FU)

/* VC_CPL_RX_Q_CTRL_OFF */

#define CSL_PCIE_RC_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_DATA_CREDIT_MASK             (0x00000FFFU)
#define CSL_PCIE_RC_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_DATA_CREDIT_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_DATA_CREDIT_MAX              (0x00000FFFU)

#define CSL_PCIE_RC_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_HEADER_CREDIT_MASK           (0x000FF000U)
#define CSL_PCIE_RC_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_HEADER_CREDIT_SHIFT          (0x0000000CU)
#define CSL_PCIE_RC_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_HEADER_CREDIT_MAX            (0x000000FFU)

#define CSL_PCIE_RC_CORE_VC_CPL_RX_Q_CTRL_OFF_RESERVED8_MASK                      (0x00100000U)
#define CSL_PCIE_RC_CORE_VC_CPL_RX_Q_CTRL_OFF_RESERVED8_SHIFT                     (0x00000014U)
#define CSL_PCIE_RC_CORE_VC_CPL_RX_Q_CTRL_OFF_RESERVED8_MAX                       (0x00000001U)

#define CSL_PCIE_RC_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_TLP_Q_MODE_MASK              (0x00E00000U)
#define CSL_PCIE_RC_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_TLP_Q_MODE_SHIFT             (0x00000015U)
#define CSL_PCIE_RC_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_TLP_Q_MODE_MAX               (0x00000007U)

#define CSL_PCIE_RC_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_HDR_SCALE_MASK               (0x03000000U)
#define CSL_PCIE_RC_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_HDR_SCALE_SHIFT              (0x00000018U)
#define CSL_PCIE_RC_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_HDR_SCALE_MAX                (0x00000003U)

#define CSL_PCIE_RC_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_DATA_SCALE_MASK              (0x0C000000U)
#define CSL_PCIE_RC_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_DATA_SCALE_SHIFT             (0x0000001AU)
#define CSL_PCIE_RC_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_DATA_SCALE_MAX               (0x00000003U)

#define CSL_PCIE_RC_CORE_VC_CPL_RX_Q_CTRL_OFF_RESERVED9_MASK                      (0xF0000000U)
#define CSL_PCIE_RC_CORE_VC_CPL_RX_Q_CTRL_OFF_RESERVED9_SHIFT                     (0x0000001CU)
#define CSL_PCIE_RC_CORE_VC_CPL_RX_Q_CTRL_OFF_RESERVED9_MAX                       (0x0000000FU)

#endif /* CSL_MODIFICATION */

/* GEN2_CTRL_OFF */

#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_FAST_TRAINING_SEQ_MASK                     (0x000000FFU)
#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_FAST_TRAINING_SEQ_SHIFT                    (0x00000000U)
#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_FAST_TRAINING_SEQ_MAX                      (0x000000FFU)

#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_NUM_OF_LANES_MASK                          (0x00001F00U)
#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_NUM_OF_LANES_SHIFT                         (0x00000008U)
#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_NUM_OF_LANES_MAX                           (0x0000001FU)

#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_PRE_DET_LANE_MASK                          (0x0000E000U)
#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_PRE_DET_LANE_SHIFT                         (0x0000000DU)
#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_PRE_DET_LANE_MAX                           (0x00000007U)

#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_AUTO_LANE_FLIP_CTRL_EN_MASK                (0x00010000U)
#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_AUTO_LANE_FLIP_CTRL_EN_SHIFT               (0x00000010U)
#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_AUTO_LANE_FLIP_CTRL_EN_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_DIRECT_SPEED_CHANGE_MASK                   (0x00020000U)
#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_DIRECT_SPEED_CHANGE_SHIFT                  (0x00000011U)
#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_DIRECT_SPEED_CHANGE_MAX                    (0x00000001U)

#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_CONFIG_PHY_TX_CHANGE_MASK                  (0x00040000U)
#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_CONFIG_PHY_TX_CHANGE_SHIFT                 (0x00000012U)
#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_CONFIG_PHY_TX_CHANGE_MAX                   (0x00000001U)

#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_CONFIG_TX_COMP_RX_MASK                     (0x00080000U)
#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_CONFIG_TX_COMP_RX_SHIFT                    (0x00000013U)
#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_CONFIG_TX_COMP_RX_MAX                      (0x00000001U)

#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_SEL_DEEMPHASIS_MASK                        (0x00100000U)
#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_SEL_DEEMPHASIS_SHIFT                       (0x00000014U)
#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_SEL_DEEMPHASIS_MAX                         (0x00000001U)

#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_GEN1_EI_INFERENCE_MASK                     (0x00200000U)
#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_GEN1_EI_INFERENCE_SHIFT                    (0x00000015U)
#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_GEN1_EI_INFERENCE_MAX                      (0x00000001U)

#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_RSVDP_22_MASK                              (0xFFC00000U)
#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_RSVDP_22_SHIFT                             (0x00000016U)
#define CSL_PCIE_RC_CORE_GEN2_CTRL_OFF_RSVDP_22_MAX                               (0x000003FFU)

/* PHY_STATUS_OFF */

#define CSL_PCIE_RC_CORE_PHY_STATUS_OFF_PHY_STATUS_MASK                           (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_PHY_STATUS_OFF_PHY_STATUS_SHIFT                          (0x00000000U)
#define CSL_PCIE_RC_CORE_PHY_STATUS_OFF_PHY_STATUS_MAX                            (0xFFFFFFFFU)

/* PHY_CONTROL_OFF */

#define CSL_PCIE_RC_CORE_PHY_CONTROL_OFF_PHY_CONTROL_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_PHY_CONTROL_OFF_PHY_CONTROL_SHIFT                        (0x00000000U)
#define CSL_PCIE_RC_CORE_PHY_CONTROL_OFF_PHY_CONTROL_MAX                          (0xFFFFFFFFU)

/* TRGT_MAP_CTRL_OFF */

#define CSL_PCIE_RC_CORE_TRGT_MAP_CTRL_OFF_TARGET_MAP_PF_MASK                     (0x0000003FU)
#define CSL_PCIE_RC_CORE_TRGT_MAP_CTRL_OFF_TARGET_MAP_PF_SHIFT                    (0x00000000U)
#define CSL_PCIE_RC_CORE_TRGT_MAP_CTRL_OFF_TARGET_MAP_PF_MAX                      (0x0000003FU)

#define CSL_PCIE_RC_CORE_TRGT_MAP_CTRL_OFF_TARGET_MAP_ROM_MASK                    (0x00000040U)
#define CSL_PCIE_RC_CORE_TRGT_MAP_CTRL_OFF_TARGET_MAP_ROM_SHIFT                   (0x00000006U)
#define CSL_PCIE_RC_CORE_TRGT_MAP_CTRL_OFF_TARGET_MAP_ROM_MAX                     (0x00000001U)

#define CSL_PCIE_RC_CORE_TRGT_MAP_CTRL_OFF_TARGET_MAP_RESERVED_13_15_MASK         (0x0000E000U)
#define CSL_PCIE_RC_CORE_TRGT_MAP_CTRL_OFF_TARGET_MAP_RESERVED_13_15_SHIFT        (0x0000000DU)
#define CSL_PCIE_RC_CORE_TRGT_MAP_CTRL_OFF_TARGET_MAP_RESERVED_13_15_MAX          (0x00000007U)

#define CSL_PCIE_RC_CORE_TRGT_MAP_CTRL_OFF_TARGET_MAP_INDEX_MASK                  (0x001F0000U)
#define CSL_PCIE_RC_CORE_TRGT_MAP_CTRL_OFF_TARGET_MAP_INDEX_SHIFT                 (0x00000010U)
#define CSL_PCIE_RC_CORE_TRGT_MAP_CTRL_OFF_TARGET_MAP_INDEX_MAX                   (0x0000001FU)

#define CSL_PCIE_RC_CORE_TRGT_MAP_CTRL_OFF_TARGET_MAP_RESERVED_21_31_MASK         (0xFFE00000U)
#define CSL_PCIE_RC_CORE_TRGT_MAP_CTRL_OFF_TARGET_MAP_RESERVED_21_31_SHIFT        (0x00000015U)
#define CSL_PCIE_RC_CORE_TRGT_MAP_CTRL_OFF_TARGET_MAP_RESERVED_21_31_MAX          (0x000007FFU)

/* MSI_CTRL_ADDR_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_ADDR_OFF_MSI_CTRL_ADDR_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_ADDR_OFF_MSI_CTRL_ADDR_SHIFT                    (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_ADDR_OFF_MSI_CTRL_ADDR_MAX                      (0xFFFFFFFFU)

/* MSI_CTRL_UPPER_ADDR_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_UPPER_ADDR_OFF_MSI_CTRL_UPPER_ADDR_MASK         (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_UPPER_ADDR_OFF_MSI_CTRL_UPPER_ADDR_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_UPPER_ADDR_OFF_MSI_CTRL_UPPER_ADDR_MAX          (0xFFFFFFFFU)

#ifdef CSL_MODIFICATION
/* MSI_CTRL_INT_0_EN_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_0_EN_OFF_MSI_CTRL_INT_0_EN_MASK             (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_0_EN_OFF_MSI_CTRL_INT_0_EN_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_0_EN_OFF_MSI_CTRL_INT_0_EN_MAX              (0xFFFFFFFFU)

/* MSI_CTRL_INT_0_MASK_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_0_MASK_OFF_MSI_CTRL_INT_0_MASK_MASK         (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_0_MASK_OFF_MSI_CTRL_INT_0_MASK_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_0_MASK_OFF_MSI_CTRL_INT_0_MASK_MAX          (0xFFFFFFFFU)

/* MSI_CTRL_INT_0_STATUS_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_0_STATUS_OFF_MSI_CTRL_INT_0_STATUS_MASK     (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_0_STATUS_OFF_MSI_CTRL_INT_0_STATUS_SHIFT    (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_0_STATUS_OFF_MSI_CTRL_INT_0_STATUS_MAX      (0xFFFFFFFFU)

/* MSI_CTRL_INT_1_EN_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_1_EN_OFF_MSI_CTRL_INT_1_EN_MASK             (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_1_EN_OFF_MSI_CTRL_INT_1_EN_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_1_EN_OFF_MSI_CTRL_INT_1_EN_MAX              (0xFFFFFFFFU)

/* MSI_CTRL_INT_1_MASK_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_1_MASK_OFF_MSI_CTRL_INT_1_MASK_MASK         (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_1_MASK_OFF_MSI_CTRL_INT_1_MASK_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_1_MASK_OFF_MSI_CTRL_INT_1_MASK_MAX          (0xFFFFFFFFU)

/* MSI_CTRL_INT_1_STATUS_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_1_STATUS_OFF_MSI_CTRL_INT_1_STATUS_MASK     (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_1_STATUS_OFF_MSI_CTRL_INT_1_STATUS_SHIFT    (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_1_STATUS_OFF_MSI_CTRL_INT_1_STATUS_MAX      (0xFFFFFFFFU)

/* MSI_CTRL_INT_2_EN_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_2_EN_OFF_MSI_CTRL_INT_2_EN_MASK             (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_2_EN_OFF_MSI_CTRL_INT_2_EN_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_2_EN_OFF_MSI_CTRL_INT_2_EN_MAX              (0xFFFFFFFFU)

/* MSI_CTRL_INT_2_MASK_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_2_MASK_OFF_MSI_CTRL_INT_2_MASK_MASK         (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_2_MASK_OFF_MSI_CTRL_INT_2_MASK_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_2_MASK_OFF_MSI_CTRL_INT_2_MASK_MAX          (0xFFFFFFFFU)

/* MSI_CTRL_INT_2_STATUS_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_2_STATUS_OFF_MSI_CTRL_INT_2_STATUS_MASK     (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_2_STATUS_OFF_MSI_CTRL_INT_2_STATUS_SHIFT    (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_2_STATUS_OFF_MSI_CTRL_INT_2_STATUS_MAX      (0xFFFFFFFFU)

/* MSI_CTRL_INT_3_EN_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_3_EN_OFF_MSI_CTRL_INT_3_EN_MASK             (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_3_EN_OFF_MSI_CTRL_INT_3_EN_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_3_EN_OFF_MSI_CTRL_INT_3_EN_MAX              (0xFFFFFFFFU)

/* MSI_CTRL_INT_3_MASK_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_3_MASK_OFF_MSI_CTRL_INT_3_MASK_MASK         (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_3_MASK_OFF_MSI_CTRL_INT_3_MASK_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_3_MASK_OFF_MSI_CTRL_INT_3_MASK_MAX          (0xFFFFFFFFU)

/* MSI_CTRL_INT_3_STATUS_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_3_STATUS_OFF_MSI_CTRL_INT_3_STATUS_MASK     (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_3_STATUS_OFF_MSI_CTRL_INT_3_STATUS_SHIFT    (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_3_STATUS_OFF_MSI_CTRL_INT_3_STATUS_MAX      (0xFFFFFFFFU)

/* MSI_CTRL_INT_4_EN_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_4_EN_OFF_MSI_CTRL_INT_4_EN_MASK             (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_4_EN_OFF_MSI_CTRL_INT_4_EN_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_4_EN_OFF_MSI_CTRL_INT_4_EN_MAX              (0xFFFFFFFFU)

/* MSI_CTRL_INT_4_MASK_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_4_MASK_OFF_MSI_CTRL_INT_4_MASK_MASK         (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_4_MASK_OFF_MSI_CTRL_INT_4_MASK_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_4_MASK_OFF_MSI_CTRL_INT_4_MASK_MAX          (0xFFFFFFFFU)

/* MSI_CTRL_INT_4_STATUS_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_4_STATUS_OFF_MSI_CTRL_INT_4_STATUS_MASK     (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_4_STATUS_OFF_MSI_CTRL_INT_4_STATUS_SHIFT    (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_4_STATUS_OFF_MSI_CTRL_INT_4_STATUS_MAX      (0xFFFFFFFFU)

/* MSI_CTRL_INT_5_EN_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_5_EN_OFF_MSI_CTRL_INT_5_EN_MASK             (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_5_EN_OFF_MSI_CTRL_INT_5_EN_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_5_EN_OFF_MSI_CTRL_INT_5_EN_MAX              (0xFFFFFFFFU)

/* MSI_CTRL_INT_5_MASK_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_5_MASK_OFF_MSI_CTRL_INT_5_MASK_MASK         (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_5_MASK_OFF_MSI_CTRL_INT_5_MASK_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_5_MASK_OFF_MSI_CTRL_INT_5_MASK_MAX          (0xFFFFFFFFU)

/* MSI_CTRL_INT_5_STATUS_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_5_STATUS_OFF_MSI_CTRL_INT_5_STATUS_MASK     (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_5_STATUS_OFF_MSI_CTRL_INT_5_STATUS_SHIFT    (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_5_STATUS_OFF_MSI_CTRL_INT_5_STATUS_MAX      (0xFFFFFFFFU)

/* MSI_CTRL_INT_6_EN_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_6_EN_OFF_MSI_CTRL_INT_6_EN_MASK             (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_6_EN_OFF_MSI_CTRL_INT_6_EN_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_6_EN_OFF_MSI_CTRL_INT_6_EN_MAX              (0xFFFFFFFFU)

/* MSI_CTRL_INT_6_MASK_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_6_MASK_OFF_MSI_CTRL_INT_6_MASK_MASK         (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_6_MASK_OFF_MSI_CTRL_INT_6_MASK_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_6_MASK_OFF_MSI_CTRL_INT_6_MASK_MAX          (0xFFFFFFFFU)

/* MSI_CTRL_INT_6_STATUS_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_6_STATUS_OFF_MSI_CTRL_INT_6_STATUS_MASK     (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_6_STATUS_OFF_MSI_CTRL_INT_6_STATUS_SHIFT    (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_6_STATUS_OFF_MSI_CTRL_INT_6_STATUS_MAX      (0xFFFFFFFFU)

/* MSI_CTRL_INT_7_EN_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_7_EN_OFF_MSI_CTRL_INT_7_EN_MASK             (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_7_EN_OFF_MSI_CTRL_INT_7_EN_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_7_EN_OFF_MSI_CTRL_INT_7_EN_MAX              (0xFFFFFFFFU)

/* MSI_CTRL_INT_7_MASK_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_7_MASK_OFF_MSI_CTRL_INT_7_MASK_MASK         (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_7_MASK_OFF_MSI_CTRL_INT_7_MASK_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_7_MASK_OFF_MSI_CTRL_INT_7_MASK_MAX          (0xFFFFFFFFU)

/* MSI_CTRL_INT_7_STATUS_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_7_STATUS_OFF_MSI_CTRL_INT_7_STATUS_MASK     (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_7_STATUS_OFF_MSI_CTRL_INT_7_STATUS_SHIFT    (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_7_STATUS_OFF_MSI_CTRL_INT_7_STATUS_MAX      (0xFFFFFFFFU)

#else /* CSL_MODIFICATION */

/* MSI_CTRL_INT_EN_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_EN_OFF_MSI_CTRL_INT_EN_MASK                 (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_EN_OFF_MSI_CTRL_INT_EN_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_EN_OFF_MSI_CTRL_INT_EN_MAX                  (0xFFFFFFFFU)

/* MSI_CTRL_INT_MASK_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_MASK_OFF_MSI_CTRL_INT_MASK_MASK             (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_MASK_OFF_MSI_CTRL_INT_MASK_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_MASK_OFF_MSI_CTRL_INT_MASK_MAX              (0xFFFFFFFFU)

/* MSI_CTRL_INT_0_STATUS_OFF */

#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_STATUS_OFF_MSI_CTRL_INT_STATUS_MASK         (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_STATUS_OFF_MSI_CTRL_INT_STATUS_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_CTRL_INT_STATUS_OFF_MSI_CTRL_INT_STATUS_MAX          (0xFFFFFFFFU)

#endif /* CSL_MODIFICATION */

/* MSI_GPIO_IO_OFF */

#define CSL_PCIE_RC_CORE_MSI_GPIO_IO_OFF_MSI_GPIO_REG_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSI_GPIO_IO_OFF_MSI_GPIO_REG_SHIFT                       (0x00000000U)
#define CSL_PCIE_RC_CORE_MSI_GPIO_IO_OFF_MSI_GPIO_REG_MAX                         (0xFFFFFFFFU)

/* CLOCK_GATING_CTRL_OFF */

#define CSL_PCIE_RC_CORE_CLOCK_GATING_CTRL_OFF_RADM_CLK_GATING_EN_MASK            (0x00000001U)
#define CSL_PCIE_RC_CORE_CLOCK_GATING_CTRL_OFF_RADM_CLK_GATING_EN_SHIFT           (0x00000000U)
#define CSL_PCIE_RC_CORE_CLOCK_GATING_CTRL_OFF_RADM_CLK_GATING_EN_MAX             (0x00000001U)

#define CSL_PCIE_RC_CORE_CLOCK_GATING_CTRL_OFF_RSVDP_1_MASK                       (0xFFFFFFFEU)
#define CSL_PCIE_RC_CORE_CLOCK_GATING_CTRL_OFF_RSVDP_1_SHIFT                      (0x00000001U)
#define CSL_PCIE_RC_CORE_CLOCK_GATING_CTRL_OFF_RSVDP_1_MAX                        (0x7FFFFFFFU)

/* GEN3_RELATED_OFF */

#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_GEN3_ZRXDC_NONCOMPL_MASK                (0x00000001U)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_GEN3_ZRXDC_NONCOMPL_SHIFT               (0x00000000U)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_GEN3_ZRXDC_NONCOMPL_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_RSVDP_1_MASK                            (0x000000FEU)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_RSVDP_1_SHIFT                           (0x00000001U)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_RSVDP_1_MAX                             (0x0000007FU)

#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_DISABLE_SCRAMBLER_GEN_3_MASK            (0x00000100U)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_DISABLE_SCRAMBLER_GEN_3_SHIFT           (0x00000008U)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_DISABLE_SCRAMBLER_GEN_3_MAX             (0x00000001U)

#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_EQ_PHASE_2_3_MASK                       (0x00000200U)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_EQ_PHASE_2_3_SHIFT                      (0x00000009U)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_EQ_PHASE_2_3_MAX                        (0x00000001U)

#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_EQ_EIEOS_CNT_MASK                       (0x00000400U)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_EQ_EIEOS_CNT_SHIFT                      (0x0000000AU)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_EQ_EIEOS_CNT_MAX                        (0x00000001U)

#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_EQ_REDO_MASK                            (0x00000800U)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_EQ_REDO_SHIFT                           (0x0000000BU)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_EQ_REDO_MAX                             (0x00000001U)

#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_RXEQ_PH01_EN_MASK                       (0x00001000U)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_RXEQ_PH01_EN_SHIFT                      (0x0000000CU)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_RXEQ_PH01_EN_MAX                        (0x00000001U)

#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_RXEQ_RGRDLESS_RXTS_MASK                 (0x00002000U)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_RXEQ_RGRDLESS_RXTS_SHIFT                (0x0000000DU)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_RXEQ_RGRDLESS_RXTS_MAX                  (0x00000001U)

#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_RSVDP_14_MASK                           (0x0000C000U)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_RSVDP_14_SHIFT                          (0x0000000EU)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_RSVDP_14_MAX                            (0x00000003U)

#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_GEN3_EQUALIZATION_DISABLE_MASK          (0x00010000U)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_GEN3_EQUALIZATION_DISABLE_SHIFT         (0x00000010U)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_GEN3_EQUALIZATION_DISABLE_MAX           (0x00000001U)

#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_GEN3_DLLP_XMT_DELAY_DISABLE_MASK        (0x00020000U)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_GEN3_DLLP_XMT_DELAY_DISABLE_SHIFT       (0x00000011U)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_GEN3_DLLP_XMT_DELAY_DISABLE_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_GEN3_DC_BALANCE_DISABLE_MASK            (0x00040000U)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_GEN3_DC_BALANCE_DISABLE_SHIFT           (0x00000012U)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_GEN3_DC_BALANCE_DISABLE_MAX             (0x00000001U)

#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_RSVDP_19_MASK                           (0x00180000U)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_RSVDP_19_SHIFT                          (0x00000013U)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_RSVDP_19_MAX                            (0x00000003U)

#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_GEN3_EQ_INVREQ_EVAL_DIFF_DISABLE_MASK   (0x00800000U)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_GEN3_EQ_INVREQ_EVAL_DIFF_DISABLE_SHIFT  (0x00000017U)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_GEN3_EQ_INVREQ_EVAL_DIFF_DISABLE_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_RSVDP_26_MASK                           (0xFC000000U)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_RSVDP_26_SHIFT                          (0x0000001AU)
#define CSL_PCIE_RC_CORE_GEN3_RELATED_OFF_RSVDP_26_MAX                            (0x0000003FU)

/* GEN3_EQ_CONTROL_OFF */

#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_GEN3_EQ_FB_MODE_MASK                 (0x0000000FU)
#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_GEN3_EQ_FB_MODE_SHIFT                (0x00000000U)
#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_GEN3_EQ_FB_MODE_MAX                  (0x0000000FU)

#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_GEN3_EQ_PHASE23_EXIT_MODE_MASK       (0x00000010U)
#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_GEN3_EQ_PHASE23_EXIT_MODE_SHIFT      (0x00000004U)
#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_GEN3_EQ_PHASE23_EXIT_MODE_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_GEN3_EQ_EVAL_2MS_DISABLE_MASK        (0x00000020U)
#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_GEN3_EQ_EVAL_2MS_DISABLE_SHIFT       (0x00000005U)
#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_GEN3_EQ_EVAL_2MS_DISABLE_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_GEN3_LOWER_RATE_EQ_REDO_ENABLE_MASK  (0x00000040U)
#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_GEN3_LOWER_RATE_EQ_REDO_ENABLE_SHIFT (0x00000006U)
#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_GEN3_LOWER_RATE_EQ_REDO_ENABLE_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_RSVDP_7_MASK                         (0x00000080U)
#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_RSVDP_7_SHIFT                        (0x00000007U)
#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_RSVDP_7_MAX                          (0x00000001U)

#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_GEN3_EQ_PSET_REQ_VEC_MASK            (0x00FFFF00U)
#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_GEN3_EQ_PSET_REQ_VEC_SHIFT           (0x00000008U)
#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_GEN3_EQ_PSET_REQ_VEC_MAX             (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_GEN3_EQ_FOM_INC_INITIAL_EVAL_MASK    (0x01000000U)
#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_GEN3_EQ_FOM_INC_INITIAL_EVAL_SHIFT   (0x00000018U)
#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_GEN3_EQ_FOM_INC_INITIAL_EVAL_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_GEN3_EQ_PSET_REQ_AS_COEF_MASK        (0x02000000U)
#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_GEN3_EQ_PSET_REQ_AS_COEF_SHIFT       (0x00000019U)
#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_GEN3_EQ_PSET_REQ_AS_COEF_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_GEN3_REQ_SEND_CONSEC_EIEOS_FOR_PSET_MAP_MASK (0x04000000U)
#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_GEN3_REQ_SEND_CONSEC_EIEOS_FOR_PSET_MAP_SHIFT (0x0000001AU)
#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_GEN3_REQ_SEND_CONSEC_EIEOS_FOR_PSET_MAP_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_RSVDP_27_MASK                        (0xF8000000U)
#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_RSVDP_27_SHIFT                       (0x0000001BU)
#define CSL_PCIE_RC_CORE_GEN3_EQ_CONTROL_OFF_RSVDP_27_MAX                         (0x0000001FU)

/* ORDER_RULE_CTRL_OFF */

#define CSL_PCIE_RC_CORE_ORDER_RULE_CTRL_OFF_NP_PASS_P_MASK                       (0x000000FFU)
#define CSL_PCIE_RC_CORE_ORDER_RULE_CTRL_OFF_NP_PASS_P_SHIFT                      (0x00000000U)
#define CSL_PCIE_RC_CORE_ORDER_RULE_CTRL_OFF_NP_PASS_P_MAX                        (0x000000FFU)

#define CSL_PCIE_RC_CORE_ORDER_RULE_CTRL_OFF_CPL_PASS_P_MASK                      (0x0000FF00U)
#define CSL_PCIE_RC_CORE_ORDER_RULE_CTRL_OFF_CPL_PASS_P_SHIFT                     (0x00000008U)
#define CSL_PCIE_RC_CORE_ORDER_RULE_CTRL_OFF_CPL_PASS_P_MAX                       (0x000000FFU)

#define CSL_PCIE_RC_CORE_ORDER_RULE_CTRL_OFF_RSVDP_16_MASK                        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_ORDER_RULE_CTRL_OFF_RSVDP_16_SHIFT                       (0x00000010U)
#define CSL_PCIE_RC_CORE_ORDER_RULE_CTRL_OFF_RSVDP_16_MAX                         (0x0000FFFFU)

/* PIPE_LOOPBACK_CONTROL_OFF */

#define CSL_PCIE_RC_CORE_PIPE_LOOPBACK_CONTROL_OFF_LPBK_RXVALID_MASK              (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_PIPE_LOOPBACK_CONTROL_OFF_LPBK_RXVALID_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_PIPE_LOOPBACK_CONTROL_OFF_LPBK_RXVALID_MAX               (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_PIPE_LOOPBACK_CONTROL_OFF_RXSTATUS_LANE_MASK             (0x003F0000U)
#define CSL_PCIE_RC_CORE_PIPE_LOOPBACK_CONTROL_OFF_RXSTATUS_LANE_SHIFT            (0x00000010U)
#define CSL_PCIE_RC_CORE_PIPE_LOOPBACK_CONTROL_OFF_RXSTATUS_LANE_MAX              (0x0000003FU)

#define CSL_PCIE_RC_CORE_PIPE_LOOPBACK_CONTROL_OFF_RSVDP_22_MASK                  (0x00C00000U)
#define CSL_PCIE_RC_CORE_PIPE_LOOPBACK_CONTROL_OFF_RSVDP_22_SHIFT                 (0x00000016U)
#define CSL_PCIE_RC_CORE_PIPE_LOOPBACK_CONTROL_OFF_RSVDP_22_MAX                   (0x00000003U)

#define CSL_PCIE_RC_CORE_PIPE_LOOPBACK_CONTROL_OFF_RXSTATUS_VALUE_MASK            (0x07000000U)
#define CSL_PCIE_RC_CORE_PIPE_LOOPBACK_CONTROL_OFF_RXSTATUS_VALUE_SHIFT           (0x00000018U)
#define CSL_PCIE_RC_CORE_PIPE_LOOPBACK_CONTROL_OFF_RXSTATUS_VALUE_MAX             (0x00000007U)

#define CSL_PCIE_RC_CORE_PIPE_LOOPBACK_CONTROL_OFF_RSVDP_27_MASK                  (0x78000000U)
#define CSL_PCIE_RC_CORE_PIPE_LOOPBACK_CONTROL_OFF_RSVDP_27_SHIFT                 (0x0000001BU)
#define CSL_PCIE_RC_CORE_PIPE_LOOPBACK_CONTROL_OFF_RSVDP_27_MAX                   (0x0000000FU)

#define CSL_PCIE_RC_CORE_PIPE_LOOPBACK_CONTROL_OFF_PIPE_LOOPBACK_MASK             (0x80000000U)
#define CSL_PCIE_RC_CORE_PIPE_LOOPBACK_CONTROL_OFF_PIPE_LOOPBACK_SHIFT            (0x0000001FU)
#define CSL_PCIE_RC_CORE_PIPE_LOOPBACK_CONTROL_OFF_PIPE_LOOPBACK_MAX              (0x00000001U)

/* MISC_CONTROL_1_OFF */

#define CSL_PCIE_RC_CORE_MISC_CONTROL_1_OFF_DBI_RO_WR_EN_MASK                     (0x00000001U)
#define CSL_PCIE_RC_CORE_MISC_CONTROL_1_OFF_DBI_RO_WR_EN_SHIFT                    (0x00000000U)
#define CSL_PCIE_RC_CORE_MISC_CONTROL_1_OFF_DBI_RO_WR_EN_MAX                      (0x00000001U)

#define CSL_PCIE_RC_CORE_MISC_CONTROL_1_OFF_DEFAULT_TARGET_MASK                   (0x00000002U)
#define CSL_PCIE_RC_CORE_MISC_CONTROL_1_OFF_DEFAULT_TARGET_SHIFT                  (0x00000001U)
#define CSL_PCIE_RC_CORE_MISC_CONTROL_1_OFF_DEFAULT_TARGET_MAX                    (0x00000001U)

#define CSL_PCIE_RC_CORE_MISC_CONTROL_1_OFF_UR_CA_MASK_4_TRGT1_MASK               (0x00000004U)
#define CSL_PCIE_RC_CORE_MISC_CONTROL_1_OFF_UR_CA_MASK_4_TRGT1_SHIFT              (0x00000002U)
#define CSL_PCIE_RC_CORE_MISC_CONTROL_1_OFF_UR_CA_MASK_4_TRGT1_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_MISC_CONTROL_1_OFF_SIMPLIFIED_REPLAY_TIMER_MASK          (0x00000008U)
#define CSL_PCIE_RC_CORE_MISC_CONTROL_1_OFF_SIMPLIFIED_REPLAY_TIMER_SHIFT         (0x00000003U)
#define CSL_PCIE_RC_CORE_MISC_CONTROL_1_OFF_SIMPLIFIED_REPLAY_TIMER_MAX           (0x00000001U)

#define CSL_PCIE_RC_CORE_MISC_CONTROL_1_OFF_RSVDP_4_MASK                          (0x00000010U)
#define CSL_PCIE_RC_CORE_MISC_CONTROL_1_OFF_RSVDP_4_SHIFT                         (0x00000004U)
#define CSL_PCIE_RC_CORE_MISC_CONTROL_1_OFF_RSVDP_4_MAX                           (0x00000001U)

#define CSL_PCIE_RC_CORE_MISC_CONTROL_1_OFF_ARI_DEVICE_NUMBER_MASK                (0x00000020U)
#define CSL_PCIE_RC_CORE_MISC_CONTROL_1_OFF_ARI_DEVICE_NUMBER_SHIFT               (0x00000005U)
#define CSL_PCIE_RC_CORE_MISC_CONTROL_1_OFF_ARI_DEVICE_NUMBER_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_MISC_CONTROL_1_OFF_RSVDP_6_MASK                          (0xFFFFFFC0U)
#define CSL_PCIE_RC_CORE_MISC_CONTROL_1_OFF_RSVDP_6_SHIFT                         (0x00000006U)
#define CSL_PCIE_RC_CORE_MISC_CONTROL_1_OFF_RSVDP_6_MAX                           (0x03FFFFFFU)

/* MULTI_LANE_CONTROL_OFF */

#define CSL_PCIE_RC_CORE_MULTI_LANE_CONTROL_OFF_TARGET_LINK_WIDTH_MASK            (0x0000003FU)
#define CSL_PCIE_RC_CORE_MULTI_LANE_CONTROL_OFF_TARGET_LINK_WIDTH_SHIFT           (0x00000000U)
#define CSL_PCIE_RC_CORE_MULTI_LANE_CONTROL_OFF_TARGET_LINK_WIDTH_MAX             (0x0000003FU)

#define CSL_PCIE_RC_CORE_MULTI_LANE_CONTROL_OFF_DIRECT_LINK_WIDTH_CHANGE_MASK     (0x00000040U)
#define CSL_PCIE_RC_CORE_MULTI_LANE_CONTROL_OFF_DIRECT_LINK_WIDTH_CHANGE_SHIFT    (0x00000006U)
#define CSL_PCIE_RC_CORE_MULTI_LANE_CONTROL_OFF_DIRECT_LINK_WIDTH_CHANGE_MAX      (0x00000001U)

#define CSL_PCIE_RC_CORE_MULTI_LANE_CONTROL_OFF_UPCONFIGURE_SUPPORT_MASK          (0x00000080U)
#define CSL_PCIE_RC_CORE_MULTI_LANE_CONTROL_OFF_UPCONFIGURE_SUPPORT_SHIFT         (0x00000007U)
#define CSL_PCIE_RC_CORE_MULTI_LANE_CONTROL_OFF_UPCONFIGURE_SUPPORT_MAX           (0x00000001U)

#define CSL_PCIE_RC_CORE_MULTI_LANE_CONTROL_OFF_RSVDP_8_MASK                      (0xFFFFFF00U)
#define CSL_PCIE_RC_CORE_MULTI_LANE_CONTROL_OFF_RSVDP_8_SHIFT                     (0x00000008U)
#define CSL_PCIE_RC_CORE_MULTI_LANE_CONTROL_OFF_RSVDP_8_MAX                       (0x00FFFFFFU)

/* PHY_INTEROP_CTRL_OFF */

#define CSL_PCIE_RC_CORE_PHY_INTEROP_CTRL_OFF_RXSTANDBY_CONTROL_MASK              (0x0000007FU)
#define CSL_PCIE_RC_CORE_PHY_INTEROP_CTRL_OFF_RXSTANDBY_CONTROL_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_PHY_INTEROP_CTRL_OFF_RXSTANDBY_CONTROL_MAX               (0x0000007FU)

#define CSL_PCIE_RC_CORE_PHY_INTEROP_CTRL_OFF_RSVDP_7_MASK                        (0x00000080U)
#define CSL_PCIE_RC_CORE_PHY_INTEROP_CTRL_OFF_RSVDP_7_SHIFT                       (0x00000007U)
#define CSL_PCIE_RC_CORE_PHY_INTEROP_CTRL_OFF_RSVDP_7_MAX                         (0x00000001U)

#define CSL_PCIE_RC_CORE_PHY_INTEROP_CTRL_OFF_L1SUB_EXIT_MODE_MASK                (0x00000100U)
#define CSL_PCIE_RC_CORE_PHY_INTEROP_CTRL_OFF_L1SUB_EXIT_MODE_SHIFT               (0x00000008U)
#define CSL_PCIE_RC_CORE_PHY_INTEROP_CTRL_OFF_L1SUB_EXIT_MODE_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_PHY_INTEROP_CTRL_OFF_L1_NOWAIT_P1_MASK                   (0x00000200U)
#define CSL_PCIE_RC_CORE_PHY_INTEROP_CTRL_OFF_L1_NOWAIT_P1_SHIFT                  (0x00000009U)
#define CSL_PCIE_RC_CORE_PHY_INTEROP_CTRL_OFF_L1_NOWAIT_P1_MAX                    (0x00000001U)

#define CSL_PCIE_RC_CORE_PHY_INTEROP_CTRL_OFF_L1_CLK_SEL_MASK                     (0x00000400U)
#define CSL_PCIE_RC_CORE_PHY_INTEROP_CTRL_OFF_L1_CLK_SEL_SHIFT                    (0x0000000AU)
#define CSL_PCIE_RC_CORE_PHY_INTEROP_CTRL_OFF_L1_CLK_SEL_MAX                      (0x00000001U)

#define CSL_PCIE_RC_CORE_PHY_INTEROP_CTRL_OFF_RSVDP_11_MASK                       (0xFFFFF800U)
#define CSL_PCIE_RC_CORE_PHY_INTEROP_CTRL_OFF_RSVDP_11_SHIFT                      (0x0000000BU)
#define CSL_PCIE_RC_CORE_PHY_INTEROP_CTRL_OFF_RSVDP_11_MAX                        (0x001FFFFFU)

/* TRGT_CPL_LUT_DELETE_ENTRY_OFF */

#define CSL_PCIE_RC_CORE_TRGT_CPL_LUT_DELETE_ENTRY_OFF_LOOK_UP_ID_MASK            (0x7FFFFFFFU)
#define CSL_PCIE_RC_CORE_TRGT_CPL_LUT_DELETE_ENTRY_OFF_LOOK_UP_ID_SHIFT           (0x00000000U)
#define CSL_PCIE_RC_CORE_TRGT_CPL_LUT_DELETE_ENTRY_OFF_LOOK_UP_ID_MAX             (0x7FFFFFFFU)

#define CSL_PCIE_RC_CORE_TRGT_CPL_LUT_DELETE_ENTRY_OFF_DELETE_EN_MASK             (0x80000000U)
#define CSL_PCIE_RC_CORE_TRGT_CPL_LUT_DELETE_ENTRY_OFF_DELETE_EN_SHIFT            (0x0000001FU)
#define CSL_PCIE_RC_CORE_TRGT_CPL_LUT_DELETE_ENTRY_OFF_DELETE_EN_MAX              (0x00000001U)

/* LINK_FLUSH_CONTROL_OFF */

#define CSL_PCIE_RC_CORE_LINK_FLUSH_CONTROL_OFF_AUTO_FLUSH_EN_MASK                (0x00000001U)
#define CSL_PCIE_RC_CORE_LINK_FLUSH_CONTROL_OFF_AUTO_FLUSH_EN_SHIFT               (0x00000000U)
#define CSL_PCIE_RC_CORE_LINK_FLUSH_CONTROL_OFF_AUTO_FLUSH_EN_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_LINK_FLUSH_CONTROL_OFF_RSVDP_1_MASK                      (0x00FFFFFEU)
#define CSL_PCIE_RC_CORE_LINK_FLUSH_CONTROL_OFF_RSVDP_1_SHIFT                     (0x00000001U)
#define CSL_PCIE_RC_CORE_LINK_FLUSH_CONTROL_OFF_RSVDP_1_MAX                       (0x007FFFFFU)

#define CSL_PCIE_RC_CORE_LINK_FLUSH_CONTROL_OFF_RSVD_I_8_MASK                     (0xFF000000U)
#define CSL_PCIE_RC_CORE_LINK_FLUSH_CONTROL_OFF_RSVD_I_8_SHIFT                    (0x00000018U)
#define CSL_PCIE_RC_CORE_LINK_FLUSH_CONTROL_OFF_RSVD_I_8_MAX                      (0x000000FFU)

/* AMBA_ERROR_RESPONSE_DEFAULT_OFF */

#define CSL_PCIE_RC_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_GLOBAL_MASK (0x00000001U)
#define CSL_PCIE_RC_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_GLOBAL_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_GLOBAL_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_RSVDP_1_MASK             (0x00000002U)
#define CSL_PCIE_RC_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_RSVDP_1_SHIFT            (0x00000001U)
#define CSL_PCIE_RC_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_RSVDP_1_MAX              (0x00000001U)

#define CSL_PCIE_RC_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_VENDORID_MASK (0x00000004U)
#define CSL_PCIE_RC_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_VENDORID_SHIFT (0x00000002U)
#define CSL_PCIE_RC_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_VENDORID_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_CRS_MASK (0x00000018U)
#define CSL_PCIE_RC_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_CRS_SHIFT (0x00000003U)
#define CSL_PCIE_RC_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_CRS_MAX (0x00000003U)

#define CSL_PCIE_RC_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_RSVDP_5_MASK             (0x000003E0U)
#define CSL_PCIE_RC_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_RSVDP_5_SHIFT            (0x00000005U)
#define CSL_PCIE_RC_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_RSVDP_5_MAX              (0x0000001FU)

#define CSL_PCIE_RC_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_MAP_MASK (0x0000FC00U)
#define CSL_PCIE_RC_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_MAP_SHIFT (0x0000000AU)
#define CSL_PCIE_RC_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_MAP_MAX (0x0000003FU)

#define CSL_PCIE_RC_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_RSVDP_16_MASK            (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_RSVDP_16_SHIFT           (0x00000010U)
#define CSL_PCIE_RC_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_RSVDP_16_MAX             (0x0000FFFFU)

/* AMBA_LINK_TIMEOUT_OFF */

#define CSL_PCIE_RC_CORE_AMBA_LINK_TIMEOUT_OFF_LINK_TIMEOUT_PERIOD_DEFAULT_MASK   (0x000000FFU)
#define CSL_PCIE_RC_CORE_AMBA_LINK_TIMEOUT_OFF_LINK_TIMEOUT_PERIOD_DEFAULT_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_AMBA_LINK_TIMEOUT_OFF_LINK_TIMEOUT_PERIOD_DEFAULT_MAX    (0x000000FFU)

#define CSL_PCIE_RC_CORE_AMBA_LINK_TIMEOUT_OFF_LINK_TIMEOUT_ENABLE_DEFAULT_MASK   (0x00000100U)
#define CSL_PCIE_RC_CORE_AMBA_LINK_TIMEOUT_OFF_LINK_TIMEOUT_ENABLE_DEFAULT_SHIFT  (0x00000008U)
#define CSL_PCIE_RC_CORE_AMBA_LINK_TIMEOUT_OFF_LINK_TIMEOUT_ENABLE_DEFAULT_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_AMBA_LINK_TIMEOUT_OFF_RSVDP_9_MASK                       (0xFFFFFE00U)
#define CSL_PCIE_RC_CORE_AMBA_LINK_TIMEOUT_OFF_RSVDP_9_SHIFT                      (0x00000009U)
#define CSL_PCIE_RC_CORE_AMBA_LINK_TIMEOUT_OFF_RSVDP_9_MAX                        (0x007FFFFFU)

/* AMBA_ORDERING_CTRL_OFF */

#define CSL_PCIE_RC_CORE_AMBA_ORDERING_CTRL_OFF_RSVDP_0_MASK                      (0x00000001U)
#define CSL_PCIE_RC_CORE_AMBA_ORDERING_CTRL_OFF_RSVDP_0_SHIFT                     (0x00000000U)
#define CSL_PCIE_RC_CORE_AMBA_ORDERING_CTRL_OFF_RSVDP_0_MAX                       (0x00000001U)

#define CSL_PCIE_RC_CORE_AMBA_ORDERING_CTRL_OFF_AX_SNP_EN_MASK                    (0x00000002U)
#define CSL_PCIE_RC_CORE_AMBA_ORDERING_CTRL_OFF_AX_SNP_EN_SHIFT                   (0x00000001U)
#define CSL_PCIE_RC_CORE_AMBA_ORDERING_CTRL_OFF_AX_SNP_EN_MAX                     (0x00000001U)

#define CSL_PCIE_RC_CORE_AMBA_ORDERING_CTRL_OFF_RSVDP_2_MASK                      (0x00000004U)
#define CSL_PCIE_RC_CORE_AMBA_ORDERING_CTRL_OFF_RSVDP_2_SHIFT                     (0x00000002U)
#define CSL_PCIE_RC_CORE_AMBA_ORDERING_CTRL_OFF_RSVDP_2_MAX                       (0x00000001U)

#define CSL_PCIE_RC_CORE_AMBA_ORDERING_CTRL_OFF_AX_MSTR_ORDR_P_EVENT_SEL_MASK     (0x00000018U)
#define CSL_PCIE_RC_CORE_AMBA_ORDERING_CTRL_OFF_AX_MSTR_ORDR_P_EVENT_SEL_SHIFT    (0x00000003U)
#define CSL_PCIE_RC_CORE_AMBA_ORDERING_CTRL_OFF_AX_MSTR_ORDR_P_EVENT_SEL_MAX      (0x00000003U)

#define CSL_PCIE_RC_CORE_AMBA_ORDERING_CTRL_OFF_RSVDP_5_MASK                      (0xFFFFFFE0U)
#define CSL_PCIE_RC_CORE_AMBA_ORDERING_CTRL_OFF_RSVDP_5_SHIFT                     (0x00000005U)
#define CSL_PCIE_RC_CORE_AMBA_ORDERING_CTRL_OFF_RSVDP_5_MAX                       (0x07FFFFFFU)

/* COHERENCY_CONTROL_1_OFF */

#define CSL_PCIE_RC_CORE_COHERENCY_CONTROL_1_OFF_CFG_MEMTYPE_VALUE_MASK           (0x00000001U)
#define CSL_PCIE_RC_CORE_COHERENCY_CONTROL_1_OFF_CFG_MEMTYPE_VALUE_SHIFT          (0x00000000U)
#define CSL_PCIE_RC_CORE_COHERENCY_CONTROL_1_OFF_CFG_MEMTYPE_VALUE_MAX            (0x00000001U)

#define CSL_PCIE_RC_CORE_COHERENCY_CONTROL_1_OFF_RSVDP_1_MASK                     (0x00000002U)
#define CSL_PCIE_RC_CORE_COHERENCY_CONTROL_1_OFF_RSVDP_1_SHIFT                    (0x00000001U)
#define CSL_PCIE_RC_CORE_COHERENCY_CONTROL_1_OFF_RSVDP_1_MAX                      (0x00000001U)

#define CSL_PCIE_RC_CORE_COHERENCY_CONTROL_1_OFF_CFG_MEMTYPE_BOUNDARY_LOW_ADDR_MASK (0xFFFFFFFCU)
#define CSL_PCIE_RC_CORE_COHERENCY_CONTROL_1_OFF_CFG_MEMTYPE_BOUNDARY_LOW_ADDR_SHIFT (0x00000002U)
#define CSL_PCIE_RC_CORE_COHERENCY_CONTROL_1_OFF_CFG_MEMTYPE_BOUNDARY_LOW_ADDR_MAX (0x3FFFFFFFU)

/* COHERENCY_CONTROL_2_OFF */

#define CSL_PCIE_RC_CORE_COHERENCY_CONTROL_2_OFF_CFG_MEMTYPE_BOUNDARY_HIGH_ADDR_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_COHERENCY_CONTROL_2_OFF_CFG_MEMTYPE_BOUNDARY_HIGH_ADDR_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_COHERENCY_CONTROL_2_OFF_CFG_MEMTYPE_BOUNDARY_HIGH_ADDR_MAX (0xFFFFFFFFU)

/* COHERENCY_CONTROL_3_OFF */

#define CSL_PCIE_RC_CORE_COHERENCY_CONTROL_3_OFF_CFG_MSTR_ARCACHE_MODE_MASK       (0x00000078U)
#define CSL_PCIE_RC_CORE_COHERENCY_CONTROL_3_OFF_CFG_MSTR_ARCACHE_MODE_SHIFT      (0x00000003U)
#define CSL_PCIE_RC_CORE_COHERENCY_CONTROL_3_OFF_CFG_MSTR_ARCACHE_MODE_MAX        (0x0000000FU)

#define CSL_PCIE_RC_CORE_COHERENCY_CONTROL_3_OFF_CFG_MSTR_AWCACHE_MODE_MASK       (0x00007800U)
#define CSL_PCIE_RC_CORE_COHERENCY_CONTROL_3_OFF_CFG_MSTR_AWCACHE_MODE_SHIFT      (0x0000000BU)
#define CSL_PCIE_RC_CORE_COHERENCY_CONTROL_3_OFF_CFG_MSTR_AWCACHE_MODE_MAX        (0x0000000FU)

#define CSL_PCIE_RC_CORE_COHERENCY_CONTROL_3_OFF_CFG_MSTR_ARCACHE_VALUE_MASK      (0x00780000U)
#define CSL_PCIE_RC_CORE_COHERENCY_CONTROL_3_OFF_CFG_MSTR_ARCACHE_VALUE_SHIFT     (0x00000013U)
#define CSL_PCIE_RC_CORE_COHERENCY_CONTROL_3_OFF_CFG_MSTR_ARCACHE_VALUE_MAX       (0x0000000FU)

#define CSL_PCIE_RC_CORE_COHERENCY_CONTROL_3_OFF_CFG_MSTR_AWCACHE_VALUE_MASK      (0x78000000U)
#define CSL_PCIE_RC_CORE_COHERENCY_CONTROL_3_OFF_CFG_MSTR_AWCACHE_VALUE_SHIFT     (0x0000001BU)
#define CSL_PCIE_RC_CORE_COHERENCY_CONTROL_3_OFF_CFG_MSTR_AWCACHE_VALUE_MAX       (0x0000000FU)

/* AXI_MSTR_MSG_ADDR_LOW_OFF */

#define CSL_PCIE_RC_CORE_AXI_MSTR_MSG_ADDR_LOW_OFF_CFG_AXIMSTR_MSG_ADDR_LOW_RESERVED_MASK (0x00000FFFU)
#define CSL_PCIE_RC_CORE_AXI_MSTR_MSG_ADDR_LOW_OFF_CFG_AXIMSTR_MSG_ADDR_LOW_RESERVED_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_AXI_MSTR_MSG_ADDR_LOW_OFF_CFG_AXIMSTR_MSG_ADDR_LOW_RESERVED_MAX (0x00000FFFU)

#define CSL_PCIE_RC_CORE_AXI_MSTR_MSG_ADDR_LOW_OFF_CFG_AXIMSTR_MSG_ADDR_LOW_MASK  (0xFFFFF000U)
#define CSL_PCIE_RC_CORE_AXI_MSTR_MSG_ADDR_LOW_OFF_CFG_AXIMSTR_MSG_ADDR_LOW_SHIFT (0x0000000CU)
#define CSL_PCIE_RC_CORE_AXI_MSTR_MSG_ADDR_LOW_OFF_CFG_AXIMSTR_MSG_ADDR_LOW_MAX   (0x000FFFFFU)

/* AXI_MSTR_MSG_ADDR_HIGH_OFF */

#define CSL_PCIE_RC_CORE_AXI_MSTR_MSG_ADDR_HIGH_OFF_CFG_AXIMSTR_MSG_ADDR_HIGH_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_AXI_MSTR_MSG_ADDR_HIGH_OFF_CFG_AXIMSTR_MSG_ADDR_HIGH_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_AXI_MSTR_MSG_ADDR_HIGH_OFF_CFG_AXIMSTR_MSG_ADDR_HIGH_MAX (0xFFFFFFFFU)

/* PCIE_VERSION_NUMBER_OFF */

#define CSL_PCIE_RC_CORE_PCIE_VERSION_NUMBER_OFF_VERSION_NUMBER_MASK              (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_PCIE_VERSION_NUMBER_OFF_VERSION_NUMBER_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_PCIE_VERSION_NUMBER_OFF_VERSION_NUMBER_MAX               (0xFFFFFFFFU)

/* PCIE_VERSION_TYPE_OFF */

#define CSL_PCIE_RC_CORE_PCIE_VERSION_TYPE_OFF_VERSION_TYPE_MASK                  (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_PCIE_VERSION_TYPE_OFF_VERSION_TYPE_SHIFT                 (0x00000000U)
#define CSL_PCIE_RC_CORE_PCIE_VERSION_TYPE_OFF_VERSION_TYPE_MAX                   (0xFFFFFFFFU)

/* MSIX_ADDRESS_MATCH_LOW_OFF */

#define CSL_PCIE_RC_CORE_MSIX_ADDRESS_MATCH_LOW_OFF_MSIX_ADDRESS_MATCH_EN_MASK    (0x00000001U)
#define CSL_PCIE_RC_CORE_MSIX_ADDRESS_MATCH_LOW_OFF_MSIX_ADDRESS_MATCH_EN_SHIFT   (0x00000000U)
#define CSL_PCIE_RC_CORE_MSIX_ADDRESS_MATCH_LOW_OFF_MSIX_ADDRESS_MATCH_EN_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_MSIX_ADDRESS_MATCH_LOW_OFF_MSIX_ADDRESS_MATCH_RESERVED_1_MASK (0x00000002U)
#define CSL_PCIE_RC_CORE_MSIX_ADDRESS_MATCH_LOW_OFF_MSIX_ADDRESS_MATCH_RESERVED_1_SHIFT (0x00000001U)
#define CSL_PCIE_RC_CORE_MSIX_ADDRESS_MATCH_LOW_OFF_MSIX_ADDRESS_MATCH_RESERVED_1_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_MSIX_ADDRESS_MATCH_LOW_OFF_MSIX_ADDRESS_MATCH_LOW_MASK   (0xFFFFFFFCU)
#define CSL_PCIE_RC_CORE_MSIX_ADDRESS_MATCH_LOW_OFF_MSIX_ADDRESS_MATCH_LOW_SHIFT  (0x00000002U)
#define CSL_PCIE_RC_CORE_MSIX_ADDRESS_MATCH_LOW_OFF_MSIX_ADDRESS_MATCH_LOW_MAX    (0x3FFFFFFFU)

/* MSIX_ADDRESS_MATCH_HIGH_OFF */

#define CSL_PCIE_RC_CORE_MSIX_ADDRESS_MATCH_HIGH_OFF_MSIX_ADDRESS_MATCH_HIGH_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_MSIX_ADDRESS_MATCH_HIGH_OFF_MSIX_ADDRESS_MATCH_HIGH_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_MSIX_ADDRESS_MATCH_HIGH_OFF_MSIX_ADDRESS_MATCH_HIGH_MAX  (0xFFFFFFFFU)

/* MSIX_DOORBELL_OFF */

#define CSL_PCIE_RC_CORE_MSIX_DOORBELL_OFF_MSIX_DOORBELL_VECTOR_MASK              (0x000007FFU)
#define CSL_PCIE_RC_CORE_MSIX_DOORBELL_OFF_MSIX_DOORBELL_VECTOR_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_MSIX_DOORBELL_OFF_MSIX_DOORBELL_VECTOR_MAX               (0x000007FFU)

#define CSL_PCIE_RC_CORE_MSIX_DOORBELL_OFF_MSIX_DOORBELL_RESERVED_11_MASK         (0x00000800U)
#define CSL_PCIE_RC_CORE_MSIX_DOORBELL_OFF_MSIX_DOORBELL_RESERVED_11_SHIFT        (0x0000000BU)
#define CSL_PCIE_RC_CORE_MSIX_DOORBELL_OFF_MSIX_DOORBELL_RESERVED_11_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_MSIX_DOORBELL_OFF_MSIX_DOORBELL_TC_MASK                  (0x00007000U)
#define CSL_PCIE_RC_CORE_MSIX_DOORBELL_OFF_MSIX_DOORBELL_TC_SHIFT                 (0x0000000CU)
#define CSL_PCIE_RC_CORE_MSIX_DOORBELL_OFF_MSIX_DOORBELL_TC_MAX                   (0x00000007U)

#define CSL_PCIE_RC_CORE_MSIX_DOORBELL_OFF_MSIX_DOORBELL_VF_ACTIVE_MASK           (0x00008000U)
#define CSL_PCIE_RC_CORE_MSIX_DOORBELL_OFF_MSIX_DOORBELL_VF_ACTIVE_SHIFT          (0x0000000FU)
#define CSL_PCIE_RC_CORE_MSIX_DOORBELL_OFF_MSIX_DOORBELL_VF_ACTIVE_MAX            (0x00000001U)

#define CSL_PCIE_RC_CORE_MSIX_DOORBELL_OFF_MSIX_DOORBELL_VF_MASK                  (0x00FF0000U)
#define CSL_PCIE_RC_CORE_MSIX_DOORBELL_OFF_MSIX_DOORBELL_VF_SHIFT                 (0x00000010U)
#define CSL_PCIE_RC_CORE_MSIX_DOORBELL_OFF_MSIX_DOORBELL_VF_MAX                   (0x000000FFU)

#define CSL_PCIE_RC_CORE_MSIX_DOORBELL_OFF_MSIX_DOORBELL_PF_MASK                  (0x1F000000U)
#define CSL_PCIE_RC_CORE_MSIX_DOORBELL_OFF_MSIX_DOORBELL_PF_SHIFT                 (0x00000018U)
#define CSL_PCIE_RC_CORE_MSIX_DOORBELL_OFF_MSIX_DOORBELL_PF_MAX                   (0x0000001FU)

#define CSL_PCIE_RC_CORE_MSIX_DOORBELL_OFF_MSIX_DOORBELL_RESERVED_29_31_MASK      (0xE0000000U)
#define CSL_PCIE_RC_CORE_MSIX_DOORBELL_OFF_MSIX_DOORBELL_RESERVED_29_31_SHIFT     (0x0000001DU)
#define CSL_PCIE_RC_CORE_MSIX_DOORBELL_OFF_MSIX_DOORBELL_RESERVED_29_31_MAX       (0x00000007U)

/* MSIX_RAM_CTRL_OFF */

#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_TABLE_DS_MASK            (0x00000001U)
#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_TABLE_DS_SHIFT           (0x00000000U)
#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_TABLE_DS_MAX             (0x00000001U)

#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_TABLE_SD_MASK            (0x00000002U)
#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_TABLE_SD_SHIFT           (0x00000001U)
#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_TABLE_SD_MAX             (0x00000001U)

#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_RESERVED_2_7_MASK        (0x000000FCU)
#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_RESERVED_2_7_SHIFT       (0x00000002U)
#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_RESERVED_2_7_MAX         (0x0000003FU)

#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_PBA_DS_MASK              (0x00000100U)
#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_PBA_DS_SHIFT             (0x00000008U)
#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_PBA_DS_MAX               (0x00000001U)

#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_PBA_SD_MASK              (0x00000200U)
#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_PBA_SD_SHIFT             (0x00000009U)
#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_PBA_SD_MAX               (0x00000001U)

#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_RESERVED_10_15_MASK      (0x0000FC00U)
#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_RESERVED_10_15_SHIFT     (0x0000000AU)
#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_RESERVED_10_15_MAX       (0x0000003FU)

#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_BYPASS_MASK              (0x00010000U)
#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_BYPASS_SHIFT             (0x00000010U)
#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_BYPASS_MAX               (0x00000001U)

#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_RESERVED_17_23_MASK      (0x00FE0000U)
#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_RESERVED_17_23_SHIFT     (0x00000011U)
#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_RESERVED_17_23_MAX       (0x0000007FU)

#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_DBG_TABLE_MASK           (0x01000000U)
#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_DBG_TABLE_SHIFT          (0x00000018U)
#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_DBG_TABLE_MAX            (0x00000001U)

#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_DBG_PBA_MASK             (0x02000000U)
#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_DBG_PBA_SHIFT            (0x00000019U)
#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_DBG_PBA_MAX              (0x00000001U)

#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_RESERVED_26_31_MASK      (0xFC000000U)
#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_RESERVED_26_31_SHIFT     (0x0000001AU)
#define CSL_PCIE_RC_CORE_MSIX_RAM_CTRL_OFF_MSIX_RAM_CTRL_RESERVED_26_31_MAX       (0x0000003FU)

/* AUX_CLK_FREQ_OFF */

#define CSL_PCIE_RC_CORE_AUX_CLK_FREQ_OFF_AUX_CLK_FREQ_MASK                       (0x000003FFU)
#define CSL_PCIE_RC_CORE_AUX_CLK_FREQ_OFF_AUX_CLK_FREQ_SHIFT                      (0x00000000U)
#define CSL_PCIE_RC_CORE_AUX_CLK_FREQ_OFF_AUX_CLK_FREQ_MAX                        (0x000003FFU)

#define CSL_PCIE_RC_CORE_AUX_CLK_FREQ_OFF_RSVDP_10_MASK                           (0xFFFFFC00U)
#define CSL_PCIE_RC_CORE_AUX_CLK_FREQ_OFF_RSVDP_10_SHIFT                          (0x0000000AU)
#define CSL_PCIE_RC_CORE_AUX_CLK_FREQ_OFF_RSVDP_10_MAX                            (0x003FFFFFU)

/* L1_SUBSTATES_OFF */

#define CSL_PCIE_RC_CORE_L1_SUBSTATES_OFF_L1SUB_T_POWER_OFF_MASK                  (0x00000003U)
#define CSL_PCIE_RC_CORE_L1_SUBSTATES_OFF_L1SUB_T_POWER_OFF_SHIFT                 (0x00000000U)
#define CSL_PCIE_RC_CORE_L1_SUBSTATES_OFF_L1SUB_T_POWER_OFF_MAX                   (0x00000003U)

#define CSL_PCIE_RC_CORE_L1_SUBSTATES_OFF_L1SUB_T_L1_2_MASK                       (0x0000003CU)
#define CSL_PCIE_RC_CORE_L1_SUBSTATES_OFF_L1SUB_T_L1_2_SHIFT                      (0x00000002U)
#define CSL_PCIE_RC_CORE_L1_SUBSTATES_OFF_L1SUB_T_L1_2_MAX                        (0x0000000FU)

#define CSL_PCIE_RC_CORE_L1_SUBSTATES_OFF_L1SUB_T_PCLKACK_MASK                    (0x000000C0U)
#define CSL_PCIE_RC_CORE_L1_SUBSTATES_OFF_L1SUB_T_PCLKACK_SHIFT                   (0x00000006U)
#define CSL_PCIE_RC_CORE_L1_SUBSTATES_OFF_L1SUB_T_PCLKACK_MAX                     (0x00000003U)

#define CSL_PCIE_RC_CORE_L1_SUBSTATES_OFF_RSVDP_8_MASK                            (0xFFFFFF00U)
#define CSL_PCIE_RC_CORE_L1_SUBSTATES_OFF_RSVDP_8_SHIFT                           (0x00000008U)
#define CSL_PCIE_RC_CORE_L1_SUBSTATES_OFF_RSVDP_8_MAX                             (0x00FFFFFFU)

#ifdef CSL_MODIFICATION
/* IATU_REGION_CTRL_1_OFF_OUTBOUND_0 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TYPE_MASK              (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TYPE_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TYPE_MAX               (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TC_MASK                (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TC_SHIFT               (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TC_MAX                 (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TD_MASK                (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TD_SHIFT               (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TD_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_0_ATTR_MASK              (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_0_ATTR_SHIFT             (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_0_ATTR_MAX               (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_0_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_0_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_0_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_0_CTRL_1_FUNC_NUM_MASK   (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_0_CTRL_1_FUNC_NUM_SHIFT  (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_0_CTRL_1_FUNC_NUM_MAX    (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_OUTBOUND_0 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_MSG_CODE_MASK          (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_MSG_CODE_SHIFT         (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_MSG_CODE_MAX           (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_TAG_MASK               (0x0000FF00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_TAG_SHIFT              (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_TAG_MAX                (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_TAG_SUBSTITUTE_EN_MASK (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_TAG_SUBSTITUTE_EN_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_TAG_SUBSTITUTE_EN_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_FUNC_BYPASS_MASK       (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_FUNC_BYPASS_SHIFT      (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_FUNC_BYPASS_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_SNP_MASK               (0x00100000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_SNP_SHIFT              (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_SNP_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_INHIBIT_PAYLOAD_MASK   (0x00400000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_INHIBIT_PAYLOAD_SHIFT  (0x00000016U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_INHIBIT_PAYLOAD_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_HEADER_SUBSTITUTE_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_HEADER_SUBSTITUTE_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_HEADER_SUBSTITUTE_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_CFG_SHIFT_MODE_MASK    (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_CFG_SHIFT_MODE_SHIFT   (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_CFG_SHIFT_MODE_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_INVERT_MODE_MASK       (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_INVERT_MODE_SHIFT      (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_INVERT_MODE_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_REGION_EN_MASK         (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_REGION_EN_SHIFT        (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0_REGION_EN_MAX          (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_OUTBOUND_0 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_0_LWR_BASE_HW_MASK       (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_0_LWR_BASE_HW_SHIFT      (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_0_LWR_BASE_HW_MAX        (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_0_LWR_BASE_RW_MASK       (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_0_LWR_BASE_RW_SHIFT      (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_0_LWR_BASE_RW_MAX        (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_0 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_0_UPPER_BASE_RW_MASK   (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_0_UPPER_BASE_RW_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_0_UPPER_BASE_RW_MAX    (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_OUTBOUND_0 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_0_LIMIT_ADDR_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_0_LIMIT_ADDR_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_0_LIMIT_ADDR_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_0_LIMIT_ADDR_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_0_LIMIT_ADDR_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_0_LIMIT_ADDR_RW_MAX         (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_0 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_0_LWR_TARGET_RW_OUTBOUND_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_0_LWR_TARGET_RW_OUTBOUND_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_0_LWR_TARGET_RW_OUTBOUND_MAX (0xFFFFFFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_0 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_0_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_0_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_0_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_INBOUND_0 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_0_TYPE_MASK               (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_0_TYPE_SHIFT              (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_0_TYPE_MAX                (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_0_TC_MASK                 (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_0_TC_SHIFT                (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_0_TC_MAX                  (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_0_TD_MASK                 (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_0_TD_SHIFT                (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_0_TD_MAX                  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_0_ATTR_MASK               (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_0_ATTR_SHIFT              (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_0_ATTR_MAX                (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_0_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_0_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_0_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_0_CTRL_1_FUNC_NUM_MASK    (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_0_CTRL_1_FUNC_NUM_SHIFT   (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_0_CTRL_1_FUNC_NUM_MAX     (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_INBOUND_0 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_MSG_CODE_MASK           (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_MSG_CODE_SHIFT          (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_MSG_CODE_MAX            (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_BAR_NUM_MASK            (0x00000700U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_BAR_NUM_SHIFT           (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_BAR_NUM_MAX             (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_MSG_TYPE_MATCH_MODE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_MSG_TYPE_MATCH_MODE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_MSG_TYPE_MATCH_MODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_TC_MATCH_EN_MASK        (0x00004000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_TC_MATCH_EN_SHIFT       (0x0000000EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_TC_MATCH_EN_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_TD_MATCH_EN_MASK        (0x00008000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_TD_MATCH_EN_SHIFT       (0x0000000FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_TD_MATCH_EN_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_ATTR_MATCH_EN_MASK      (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_ATTR_MATCH_EN_SHIFT     (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_ATTR_MATCH_EN_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_FUNC_NUM_MATCH_EN_MASK  (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_FUNC_NUM_MATCH_EN_SHIFT (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_FUNC_NUM_MATCH_EN_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_MSG_CODE_MATCH_EN_MASK  (0x00200000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_MSG_CODE_MATCH_EN_SHIFT (0x00000015U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_MSG_CODE_MATCH_EN_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_SINGLE_ADDR_LOC_TRANS_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_SINGLE_ADDR_LOC_TRANS_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_SINGLE_ADDR_LOC_TRANS_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_RESPONSE_CODE_MASK      (0x03000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_RESPONSE_CODE_SHIFT     (0x00000018U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_RESPONSE_CODE_MAX       (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_FUZZY_TYPE_MATCH_CODE_MASK (0x08000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_FUZZY_TYPE_MATCH_CODE_SHIFT (0x0000001BU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_FUZZY_TYPE_MATCH_CODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_CFG_SHIFT_MODE_MASK     (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_CFG_SHIFT_MODE_SHIFT    (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_CFG_SHIFT_MODE_MAX      (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_INVERT_MODE_MASK        (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_INVERT_MODE_SHIFT       (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_INVERT_MODE_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_MATCH_MODE_MASK         (0x40000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_MATCH_MODE_SHIFT        (0x0000001EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_MATCH_MODE_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_REGION_EN_MASK          (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_REGION_EN_SHIFT         (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_0_REGION_EN_MAX           (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_INBOUND_0 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_0_LWR_BASE_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_0_LWR_BASE_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_0_LWR_BASE_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_0_LWR_BASE_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_0_LWR_BASE_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_0_LWR_BASE_RW_MAX         (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_INBOUND_0 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_0_UPPER_BASE_RW_MASK    (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_0_UPPER_BASE_RW_SHIFT   (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_0_UPPER_BASE_RW_MAX     (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_INBOUND_0 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_0_LIMIT_ADDR_HW_MASK         (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_0_LIMIT_ADDR_HW_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_0_LIMIT_ADDR_HW_MAX          (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_0_LIMIT_ADDR_RW_MASK         (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_0_LIMIT_ADDR_RW_SHIFT        (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_0_LIMIT_ADDR_RW_MAX          (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_INBOUND_0 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_0_LWR_TARGET_HW_MASK    (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_0_LWR_TARGET_HW_SHIFT   (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_0_LWR_TARGET_HW_MAX     (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_0_LWR_TARGET_RW_MASK    (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_0_LWR_TARGET_RW_SHIFT   (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_0_LWR_TARGET_RW_MAX     (0x0000FFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_INBOUND_0 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_0_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_0_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_0_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_OUTBOUND_1 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_1_TYPE_MASK              (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_1_TYPE_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_1_TYPE_MAX               (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_1_TC_MASK                (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_1_TC_SHIFT               (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_1_TC_MAX                 (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_1_TD_MASK                (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_1_TD_SHIFT               (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_1_TD_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_1_ATTR_MASK              (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_1_ATTR_SHIFT             (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_1_ATTR_MAX               (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_1_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_1_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_1_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_1_CTRL_1_FUNC_NUM_MASK   (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_1_CTRL_1_FUNC_NUM_SHIFT  (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_1_CTRL_1_FUNC_NUM_MAX    (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_OUTBOUND_1 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_MSG_CODE_MASK          (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_MSG_CODE_SHIFT         (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_MSG_CODE_MAX           (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_TAG_MASK               (0x0000FF00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_TAG_SHIFT              (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_TAG_MAX                (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_TAG_SUBSTITUTE_EN_MASK (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_TAG_SUBSTITUTE_EN_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_TAG_SUBSTITUTE_EN_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_FUNC_BYPASS_MASK       (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_FUNC_BYPASS_SHIFT      (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_FUNC_BYPASS_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_SNP_MASK               (0x00100000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_SNP_SHIFT              (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_SNP_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_INHIBIT_PAYLOAD_MASK   (0x00400000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_INHIBIT_PAYLOAD_SHIFT  (0x00000016U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_INHIBIT_PAYLOAD_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_HEADER_SUBSTITUTE_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_HEADER_SUBSTITUTE_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_HEADER_SUBSTITUTE_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_CFG_SHIFT_MODE_MASK    (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_CFG_SHIFT_MODE_SHIFT   (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_CFG_SHIFT_MODE_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_INVERT_MODE_MASK       (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_INVERT_MODE_SHIFT      (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_INVERT_MODE_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_REGION_EN_MASK         (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_REGION_EN_SHIFT        (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_1_REGION_EN_MAX          (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_OUTBOUND_1 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_1_LWR_BASE_HW_MASK       (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_1_LWR_BASE_HW_SHIFT      (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_1_LWR_BASE_HW_MAX        (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_1_LWR_BASE_RW_MASK       (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_1_LWR_BASE_RW_SHIFT      (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_1_LWR_BASE_RW_MAX        (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_1 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_1_UPPER_BASE_RW_MASK   (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_1_UPPER_BASE_RW_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_1_UPPER_BASE_RW_MAX    (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_OUTBOUND_1 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_1_LIMIT_ADDR_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_1_LIMIT_ADDR_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_1_LIMIT_ADDR_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_1_LIMIT_ADDR_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_1_LIMIT_ADDR_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_1_LIMIT_ADDR_RW_MAX         (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_1 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_1_LWR_TARGET_RW_OUTBOUND_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_1_LWR_TARGET_RW_OUTBOUND_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_1_LWR_TARGET_RW_OUTBOUND_MAX (0xFFFFFFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_1 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_1_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_1_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_1_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_INBOUND_1 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_1_TYPE_MASK               (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_1_TYPE_SHIFT              (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_1_TYPE_MAX                (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_1_TC_MASK                 (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_1_TC_SHIFT                (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_1_TC_MAX                  (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_1_TD_MASK                 (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_1_TD_SHIFT                (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_1_TD_MAX                  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_1_ATTR_MASK               (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_1_ATTR_SHIFT              (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_1_ATTR_MAX                (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_1_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_1_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_1_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_1_CTRL_1_FUNC_NUM_MASK    (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_1_CTRL_1_FUNC_NUM_SHIFT   (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_1_CTRL_1_FUNC_NUM_MAX     (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_INBOUND_1 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_MSG_CODE_MASK           (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_MSG_CODE_SHIFT          (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_MSG_CODE_MAX            (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_BAR_NUM_MASK            (0x00000700U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_BAR_NUM_SHIFT           (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_BAR_NUM_MAX             (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_MSG_TYPE_MATCH_MODE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_MSG_TYPE_MATCH_MODE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_MSG_TYPE_MATCH_MODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_TC_MATCH_EN_MASK        (0x00004000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_TC_MATCH_EN_SHIFT       (0x0000000EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_TC_MATCH_EN_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_TD_MATCH_EN_MASK        (0x00008000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_TD_MATCH_EN_SHIFT       (0x0000000FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_TD_MATCH_EN_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_ATTR_MATCH_EN_MASK      (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_ATTR_MATCH_EN_SHIFT     (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_ATTR_MATCH_EN_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_FUNC_NUM_MATCH_EN_MASK  (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_FUNC_NUM_MATCH_EN_SHIFT (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_FUNC_NUM_MATCH_EN_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_MSG_CODE_MATCH_EN_MASK  (0x00200000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_MSG_CODE_MATCH_EN_SHIFT (0x00000015U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_MSG_CODE_MATCH_EN_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_SINGLE_ADDR_LOC_TRANS_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_SINGLE_ADDR_LOC_TRANS_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_SINGLE_ADDR_LOC_TRANS_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_RESPONSE_CODE_MASK      (0x03000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_RESPONSE_CODE_SHIFT     (0x00000018U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_RESPONSE_CODE_MAX       (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_FUZZY_TYPE_MATCH_CODE_MASK (0x08000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_FUZZY_TYPE_MATCH_CODE_SHIFT (0x0000001BU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_FUZZY_TYPE_MATCH_CODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_CFG_SHIFT_MODE_MASK     (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_CFG_SHIFT_MODE_SHIFT    (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_CFG_SHIFT_MODE_MAX      (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_INVERT_MODE_MASK        (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_INVERT_MODE_SHIFT       (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_INVERT_MODE_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_MATCH_MODE_MASK         (0x40000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_MATCH_MODE_SHIFT        (0x0000001EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_MATCH_MODE_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_REGION_EN_MASK          (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_REGION_EN_SHIFT         (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_1_REGION_EN_MAX           (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_INBOUND_1 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_1_LWR_BASE_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_1_LWR_BASE_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_1_LWR_BASE_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_1_LWR_BASE_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_1_LWR_BASE_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_1_LWR_BASE_RW_MAX         (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_INBOUND_1 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_1_UPPER_BASE_RW_MASK    (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_1_UPPER_BASE_RW_SHIFT   (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_1_UPPER_BASE_RW_MAX     (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_INBOUND_1 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_1_LIMIT_ADDR_HW_MASK         (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_1_LIMIT_ADDR_HW_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_1_LIMIT_ADDR_HW_MAX          (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_1_LIMIT_ADDR_RW_MASK         (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_1_LIMIT_ADDR_RW_SHIFT        (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_1_LIMIT_ADDR_RW_MAX          (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_INBOUND_1 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_1_LWR_TARGET_HW_MASK    (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_1_LWR_TARGET_HW_SHIFT   (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_1_LWR_TARGET_HW_MAX     (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_1_LWR_TARGET_RW_MASK    (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_1_LWR_TARGET_RW_SHIFT   (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_1_LWR_TARGET_RW_MAX     (0x0000FFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_INBOUND_1 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_1_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_1_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_1_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_OUTBOUND_2 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_2_TYPE_MASK              (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_2_TYPE_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_2_TYPE_MAX               (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_2_TC_MASK                (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_2_TC_SHIFT               (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_2_TC_MAX                 (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_2_TD_MASK                (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_2_TD_SHIFT               (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_2_TD_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_2_ATTR_MASK              (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_2_ATTR_SHIFT             (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_2_ATTR_MAX               (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_2_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_2_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_2_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_2_CTRL_1_FUNC_NUM_MASK   (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_2_CTRL_1_FUNC_NUM_SHIFT  (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_2_CTRL_1_FUNC_NUM_MAX    (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_OUTBOUND_2 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_MSG_CODE_MASK          (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_MSG_CODE_SHIFT         (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_MSG_CODE_MAX           (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_TAG_MASK               (0x0000FF00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_TAG_SHIFT              (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_TAG_MAX                (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_TAG_SUBSTITUTE_EN_MASK (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_TAG_SUBSTITUTE_EN_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_TAG_SUBSTITUTE_EN_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_FUNC_BYPASS_MASK       (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_FUNC_BYPASS_SHIFT      (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_FUNC_BYPASS_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_SNP_MASK               (0x00100000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_SNP_SHIFT              (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_SNP_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_INHIBIT_PAYLOAD_MASK   (0x00400000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_INHIBIT_PAYLOAD_SHIFT  (0x00000016U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_INHIBIT_PAYLOAD_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_HEADER_SUBSTITUTE_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_HEADER_SUBSTITUTE_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_HEADER_SUBSTITUTE_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_CFG_SHIFT_MODE_MASK    (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_CFG_SHIFT_MODE_SHIFT   (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_CFG_SHIFT_MODE_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_INVERT_MODE_MASK       (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_INVERT_MODE_SHIFT      (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_INVERT_MODE_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_REGION_EN_MASK         (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_REGION_EN_SHIFT        (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_2_REGION_EN_MAX          (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_OUTBOUND_2 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_2_LWR_BASE_HW_MASK       (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_2_LWR_BASE_HW_SHIFT      (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_2_LWR_BASE_HW_MAX        (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_2_LWR_BASE_RW_MASK       (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_2_LWR_BASE_RW_SHIFT      (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_2_LWR_BASE_RW_MAX        (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_2 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_2_UPPER_BASE_RW_MASK   (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_2_UPPER_BASE_RW_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_2_UPPER_BASE_RW_MAX    (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_OUTBOUND_2 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_2_LIMIT_ADDR_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_2_LIMIT_ADDR_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_2_LIMIT_ADDR_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_2_LIMIT_ADDR_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_2_LIMIT_ADDR_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_2_LIMIT_ADDR_RW_MAX         (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_2 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_2_LWR_TARGET_RW_OUTBOUND_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_2_LWR_TARGET_RW_OUTBOUND_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_2_LWR_TARGET_RW_OUTBOUND_MAX (0xFFFFFFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_2 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_2_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_2_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_2_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_INBOUND_2 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_2_TYPE_MASK               (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_2_TYPE_SHIFT              (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_2_TYPE_MAX                (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_2_TC_MASK                 (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_2_TC_SHIFT                (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_2_TC_MAX                  (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_2_TD_MASK                 (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_2_TD_SHIFT                (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_2_TD_MAX                  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_2_ATTR_MASK               (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_2_ATTR_SHIFT              (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_2_ATTR_MAX                (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_2_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_2_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_2_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_2_CTRL_1_FUNC_NUM_MASK    (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_2_CTRL_1_FUNC_NUM_SHIFT   (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_2_CTRL_1_FUNC_NUM_MAX     (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_INBOUND_2 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_MSG_CODE_MASK           (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_MSG_CODE_SHIFT          (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_MSG_CODE_MAX            (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_BAR_NUM_MASK            (0x00000700U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_BAR_NUM_SHIFT           (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_BAR_NUM_MAX             (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_MSG_TYPE_MATCH_MODE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_MSG_TYPE_MATCH_MODE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_MSG_TYPE_MATCH_MODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_TC_MATCH_EN_MASK        (0x00004000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_TC_MATCH_EN_SHIFT       (0x0000000EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_TC_MATCH_EN_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_TD_MATCH_EN_MASK        (0x00008000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_TD_MATCH_EN_SHIFT       (0x0000000FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_TD_MATCH_EN_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_ATTR_MATCH_EN_MASK      (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_ATTR_MATCH_EN_SHIFT     (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_ATTR_MATCH_EN_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_FUNC_NUM_MATCH_EN_MASK  (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_FUNC_NUM_MATCH_EN_SHIFT (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_FUNC_NUM_MATCH_EN_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_MSG_CODE_MATCH_EN_MASK  (0x00200000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_MSG_CODE_MATCH_EN_SHIFT (0x00000015U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_MSG_CODE_MATCH_EN_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_SINGLE_ADDR_LOC_TRANS_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_SINGLE_ADDR_LOC_TRANS_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_SINGLE_ADDR_LOC_TRANS_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_RESPONSE_CODE_MASK      (0x03000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_RESPONSE_CODE_SHIFT     (0x00000018U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_RESPONSE_CODE_MAX       (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_FUZZY_TYPE_MATCH_CODE_MASK (0x08000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_FUZZY_TYPE_MATCH_CODE_SHIFT (0x0000001BU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_FUZZY_TYPE_MATCH_CODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_CFG_SHIFT_MODE_MASK     (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_CFG_SHIFT_MODE_SHIFT    (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_CFG_SHIFT_MODE_MAX      (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_INVERT_MODE_MASK        (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_INVERT_MODE_SHIFT       (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_INVERT_MODE_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_MATCH_MODE_MASK         (0x40000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_MATCH_MODE_SHIFT        (0x0000001EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_MATCH_MODE_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_REGION_EN_MASK          (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_REGION_EN_SHIFT         (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_2_REGION_EN_MAX           (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_INBOUND_2 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_2_LWR_BASE_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_2_LWR_BASE_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_2_LWR_BASE_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_2_LWR_BASE_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_2_LWR_BASE_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_2_LWR_BASE_RW_MAX         (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_INBOUND_2 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_2_UPPER_BASE_RW_MASK    (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_2_UPPER_BASE_RW_SHIFT   (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_2_UPPER_BASE_RW_MAX     (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_INBOUND_2 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_2_LIMIT_ADDR_HW_MASK         (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_2_LIMIT_ADDR_HW_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_2_LIMIT_ADDR_HW_MAX          (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_2_LIMIT_ADDR_RW_MASK         (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_2_LIMIT_ADDR_RW_SHIFT        (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_2_LIMIT_ADDR_RW_MAX          (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_INBOUND_2 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_2_LWR_TARGET_HW_MASK    (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_2_LWR_TARGET_HW_SHIFT   (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_2_LWR_TARGET_HW_MAX     (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_2_LWR_TARGET_RW_MASK    (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_2_LWR_TARGET_RW_SHIFT   (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_2_LWR_TARGET_RW_MAX     (0x0000FFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_INBOUND_2 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_2_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_2_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_2_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_OUTBOUND_3 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_3_TYPE_MASK              (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_3_TYPE_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_3_TYPE_MAX               (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_3_TC_MASK                (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_3_TC_SHIFT               (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_3_TC_MAX                 (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_3_TD_MASK                (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_3_TD_SHIFT               (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_3_TD_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_3_ATTR_MASK              (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_3_ATTR_SHIFT             (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_3_ATTR_MAX               (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_3_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_3_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_3_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_3_CTRL_1_FUNC_NUM_MASK   (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_3_CTRL_1_FUNC_NUM_SHIFT  (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_3_CTRL_1_FUNC_NUM_MAX    (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_OUTBOUND_3 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_MSG_CODE_MASK          (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_MSG_CODE_SHIFT         (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_MSG_CODE_MAX           (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_TAG_MASK               (0x0000FF00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_TAG_SHIFT              (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_TAG_MAX                (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_TAG_SUBSTITUTE_EN_MASK (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_TAG_SUBSTITUTE_EN_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_TAG_SUBSTITUTE_EN_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_FUNC_BYPASS_MASK       (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_FUNC_BYPASS_SHIFT      (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_FUNC_BYPASS_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_SNP_MASK               (0x00100000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_SNP_SHIFT              (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_SNP_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_INHIBIT_PAYLOAD_MASK   (0x00400000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_INHIBIT_PAYLOAD_SHIFT  (0x00000016U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_INHIBIT_PAYLOAD_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_HEADER_SUBSTITUTE_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_HEADER_SUBSTITUTE_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_HEADER_SUBSTITUTE_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_CFG_SHIFT_MODE_MASK    (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_CFG_SHIFT_MODE_SHIFT   (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_CFG_SHIFT_MODE_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_INVERT_MODE_MASK       (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_INVERT_MODE_SHIFT      (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_INVERT_MODE_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_REGION_EN_MASK         (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_REGION_EN_SHIFT        (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_3_REGION_EN_MAX          (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_OUTBOUND_3 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_3_LWR_BASE_HW_MASK       (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_3_LWR_BASE_HW_SHIFT      (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_3_LWR_BASE_HW_MAX        (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_3_LWR_BASE_RW_MASK       (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_3_LWR_BASE_RW_SHIFT      (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_3_LWR_BASE_RW_MAX        (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_3 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_3_UPPER_BASE_RW_MASK   (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_3_UPPER_BASE_RW_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_3_UPPER_BASE_RW_MAX    (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_OUTBOUND_3 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_3_LIMIT_ADDR_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_3_LIMIT_ADDR_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_3_LIMIT_ADDR_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_3_LIMIT_ADDR_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_3_LIMIT_ADDR_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_3_LIMIT_ADDR_RW_MAX         (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_3 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_3_LWR_TARGET_RW_OUTBOUND_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_3_LWR_TARGET_RW_OUTBOUND_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_3_LWR_TARGET_RW_OUTBOUND_MAX (0xFFFFFFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_3 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_3_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_3_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_3_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_INBOUND_3 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_3_TYPE_MASK               (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_3_TYPE_SHIFT              (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_3_TYPE_MAX                (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_3_TC_MASK                 (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_3_TC_SHIFT                (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_3_TC_MAX                  (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_3_TD_MASK                 (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_3_TD_SHIFT                (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_3_TD_MAX                  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_3_ATTR_MASK               (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_3_ATTR_SHIFT              (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_3_ATTR_MAX                (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_3_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_3_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_3_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_3_CTRL_1_FUNC_NUM_MASK    (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_3_CTRL_1_FUNC_NUM_SHIFT   (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_3_CTRL_1_FUNC_NUM_MAX     (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_INBOUND_3 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_MSG_CODE_MASK           (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_MSG_CODE_SHIFT          (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_MSG_CODE_MAX            (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_BAR_NUM_MASK            (0x00000700U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_BAR_NUM_SHIFT           (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_BAR_NUM_MAX             (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_MSG_TYPE_MATCH_MODE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_MSG_TYPE_MATCH_MODE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_MSG_TYPE_MATCH_MODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_TC_MATCH_EN_MASK        (0x00004000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_TC_MATCH_EN_SHIFT       (0x0000000EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_TC_MATCH_EN_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_TD_MATCH_EN_MASK        (0x00008000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_TD_MATCH_EN_SHIFT       (0x0000000FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_TD_MATCH_EN_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_ATTR_MATCH_EN_MASK      (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_ATTR_MATCH_EN_SHIFT     (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_ATTR_MATCH_EN_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_FUNC_NUM_MATCH_EN_MASK  (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_FUNC_NUM_MATCH_EN_SHIFT (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_FUNC_NUM_MATCH_EN_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_MSG_CODE_MATCH_EN_MASK  (0x00200000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_MSG_CODE_MATCH_EN_SHIFT (0x00000015U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_MSG_CODE_MATCH_EN_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_SINGLE_ADDR_LOC_TRANS_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_SINGLE_ADDR_LOC_TRANS_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_SINGLE_ADDR_LOC_TRANS_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_RESPONSE_CODE_MASK      (0x03000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_RESPONSE_CODE_SHIFT     (0x00000018U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_RESPONSE_CODE_MAX       (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_FUZZY_TYPE_MATCH_CODE_MASK (0x08000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_FUZZY_TYPE_MATCH_CODE_SHIFT (0x0000001BU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_FUZZY_TYPE_MATCH_CODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_CFG_SHIFT_MODE_MASK     (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_CFG_SHIFT_MODE_SHIFT    (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_CFG_SHIFT_MODE_MAX      (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_INVERT_MODE_MASK        (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_INVERT_MODE_SHIFT       (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_INVERT_MODE_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_MATCH_MODE_MASK         (0x40000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_MATCH_MODE_SHIFT        (0x0000001EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_MATCH_MODE_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_REGION_EN_MASK          (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_REGION_EN_SHIFT         (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_3_REGION_EN_MAX           (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_INBOUND_3 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_3_LWR_BASE_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_3_LWR_BASE_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_3_LWR_BASE_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_3_LWR_BASE_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_3_LWR_BASE_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_3_LWR_BASE_RW_MAX         (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_INBOUND_3 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_3_UPPER_BASE_RW_MASK    (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_3_UPPER_BASE_RW_SHIFT   (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_3_UPPER_BASE_RW_MAX     (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_INBOUND_3 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_3_LIMIT_ADDR_HW_MASK         (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_3_LIMIT_ADDR_HW_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_3_LIMIT_ADDR_HW_MAX          (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_3_LIMIT_ADDR_RW_MASK         (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_3_LIMIT_ADDR_RW_SHIFT        (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_3_LIMIT_ADDR_RW_MAX          (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_INBOUND_3 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_3_LWR_TARGET_HW_MASK    (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_3_LWR_TARGET_HW_SHIFT   (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_3_LWR_TARGET_HW_MAX     (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_3_LWR_TARGET_RW_MASK    (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_3_LWR_TARGET_RW_SHIFT   (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_3_LWR_TARGET_RW_MAX     (0x0000FFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_INBOUND_3 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_3_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_3_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_3_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_OUTBOUND_4 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_4_TYPE_MASK              (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_4_TYPE_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_4_TYPE_MAX               (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_4_TC_MASK                (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_4_TC_SHIFT               (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_4_TC_MAX                 (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_4_TD_MASK                (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_4_TD_SHIFT               (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_4_TD_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_4_ATTR_MASK              (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_4_ATTR_SHIFT             (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_4_ATTR_MAX               (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_4_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_4_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_4_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_4_CTRL_1_FUNC_NUM_MASK   (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_4_CTRL_1_FUNC_NUM_SHIFT  (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_4_CTRL_1_FUNC_NUM_MAX    (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_OUTBOUND_4 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_MSG_CODE_MASK          (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_MSG_CODE_SHIFT         (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_MSG_CODE_MAX           (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_TAG_MASK               (0x0000FF00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_TAG_SHIFT              (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_TAG_MAX                (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_TAG_SUBSTITUTE_EN_MASK (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_TAG_SUBSTITUTE_EN_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_TAG_SUBSTITUTE_EN_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_FUNC_BYPASS_MASK       (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_FUNC_BYPASS_SHIFT      (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_FUNC_BYPASS_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_SNP_MASK               (0x00100000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_SNP_SHIFT              (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_SNP_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_INHIBIT_PAYLOAD_MASK   (0x00400000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_INHIBIT_PAYLOAD_SHIFT  (0x00000016U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_INHIBIT_PAYLOAD_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_HEADER_SUBSTITUTE_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_HEADER_SUBSTITUTE_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_HEADER_SUBSTITUTE_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_CFG_SHIFT_MODE_MASK    (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_CFG_SHIFT_MODE_SHIFT   (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_CFG_SHIFT_MODE_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_INVERT_MODE_MASK       (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_INVERT_MODE_SHIFT      (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_INVERT_MODE_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_REGION_EN_MASK         (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_REGION_EN_SHIFT        (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_4_REGION_EN_MAX          (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_OUTBOUND_4 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_4_LWR_BASE_HW_MASK       (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_4_LWR_BASE_HW_SHIFT      (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_4_LWR_BASE_HW_MAX        (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_4_LWR_BASE_RW_MASK       (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_4_LWR_BASE_RW_SHIFT      (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_4_LWR_BASE_RW_MAX        (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_4 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_4_UPPER_BASE_RW_MASK   (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_4_UPPER_BASE_RW_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_4_UPPER_BASE_RW_MAX    (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_OUTBOUND_4 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_4_LIMIT_ADDR_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_4_LIMIT_ADDR_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_4_LIMIT_ADDR_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_4_LIMIT_ADDR_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_4_LIMIT_ADDR_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_4_LIMIT_ADDR_RW_MAX         (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_4 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_4_LWR_TARGET_RW_OUTBOUND_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_4_LWR_TARGET_RW_OUTBOUND_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_4_LWR_TARGET_RW_OUTBOUND_MAX (0xFFFFFFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_4 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_4_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_4_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_4_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_INBOUND_4 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_4_TYPE_MASK               (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_4_TYPE_SHIFT              (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_4_TYPE_MAX                (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_4_TC_MASK                 (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_4_TC_SHIFT                (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_4_TC_MAX                  (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_4_TD_MASK                 (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_4_TD_SHIFT                (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_4_TD_MAX                  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_4_ATTR_MASK               (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_4_ATTR_SHIFT              (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_4_ATTR_MAX                (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_4_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_4_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_4_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_4_CTRL_1_FUNC_NUM_MASK    (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_4_CTRL_1_FUNC_NUM_SHIFT   (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_4_CTRL_1_FUNC_NUM_MAX     (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_INBOUND_4 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_MSG_CODE_MASK           (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_MSG_CODE_SHIFT          (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_MSG_CODE_MAX            (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_BAR_NUM_MASK            (0x00000700U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_BAR_NUM_SHIFT           (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_BAR_NUM_MAX             (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_MSG_TYPE_MATCH_MODE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_MSG_TYPE_MATCH_MODE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_MSG_TYPE_MATCH_MODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_TC_MATCH_EN_MASK        (0x00004000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_TC_MATCH_EN_SHIFT       (0x0000000EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_TC_MATCH_EN_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_TD_MATCH_EN_MASK        (0x00008000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_TD_MATCH_EN_SHIFT       (0x0000000FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_TD_MATCH_EN_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_ATTR_MATCH_EN_MASK      (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_ATTR_MATCH_EN_SHIFT     (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_ATTR_MATCH_EN_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_FUNC_NUM_MATCH_EN_MASK  (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_FUNC_NUM_MATCH_EN_SHIFT (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_FUNC_NUM_MATCH_EN_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_MSG_CODE_MATCH_EN_MASK  (0x00200000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_MSG_CODE_MATCH_EN_SHIFT (0x00000015U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_MSG_CODE_MATCH_EN_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_SINGLE_ADDR_LOC_TRANS_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_SINGLE_ADDR_LOC_TRANS_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_SINGLE_ADDR_LOC_TRANS_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_RESPONSE_CODE_MASK      (0x03000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_RESPONSE_CODE_SHIFT     (0x00000018U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_RESPONSE_CODE_MAX       (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_FUZZY_TYPE_MATCH_CODE_MASK (0x08000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_FUZZY_TYPE_MATCH_CODE_SHIFT (0x0000001BU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_FUZZY_TYPE_MATCH_CODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_CFG_SHIFT_MODE_MASK     (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_CFG_SHIFT_MODE_SHIFT    (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_CFG_SHIFT_MODE_MAX      (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_INVERT_MODE_MASK        (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_INVERT_MODE_SHIFT       (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_INVERT_MODE_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_MATCH_MODE_MASK         (0x40000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_MATCH_MODE_SHIFT        (0x0000001EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_MATCH_MODE_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_REGION_EN_MASK          (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_REGION_EN_SHIFT         (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_4_REGION_EN_MAX           (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_INBOUND_4 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_4_LWR_BASE_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_4_LWR_BASE_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_4_LWR_BASE_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_4_LWR_BASE_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_4_LWR_BASE_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_4_LWR_BASE_RW_MAX         (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_INBOUND_4 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_4_UPPER_BASE_RW_MASK    (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_4_UPPER_BASE_RW_SHIFT   (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_4_UPPER_BASE_RW_MAX     (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_INBOUND_4 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_4_LIMIT_ADDR_HW_MASK         (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_4_LIMIT_ADDR_HW_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_4_LIMIT_ADDR_HW_MAX          (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_4_LIMIT_ADDR_RW_MASK         (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_4_LIMIT_ADDR_RW_SHIFT        (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_4_LIMIT_ADDR_RW_MAX          (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_INBOUND_4 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_4_LWR_TARGET_HW_MASK    (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_4_LWR_TARGET_HW_SHIFT   (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_4_LWR_TARGET_HW_MAX     (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_4_LWR_TARGET_RW_MASK    (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_4_LWR_TARGET_RW_SHIFT   (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_4_LWR_TARGET_RW_MAX     (0x0000FFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_INBOUND_4 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_4_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_4_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_4_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_OUTBOUND_5 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_5_TYPE_MASK              (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_5_TYPE_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_5_TYPE_MAX               (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_5_TC_MASK                (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_5_TC_SHIFT               (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_5_TC_MAX                 (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_5_TD_MASK                (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_5_TD_SHIFT               (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_5_TD_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_5_ATTR_MASK              (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_5_ATTR_SHIFT             (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_5_ATTR_MAX               (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_5_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_5_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_5_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_5_CTRL_1_FUNC_NUM_MASK   (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_5_CTRL_1_FUNC_NUM_SHIFT  (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_5_CTRL_1_FUNC_NUM_MAX    (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_OUTBOUND_5 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_MSG_CODE_MASK          (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_MSG_CODE_SHIFT         (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_MSG_CODE_MAX           (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_TAG_MASK               (0x0000FF00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_TAG_SHIFT              (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_TAG_MAX                (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_TAG_SUBSTITUTE_EN_MASK (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_TAG_SUBSTITUTE_EN_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_TAG_SUBSTITUTE_EN_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_FUNC_BYPASS_MASK       (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_FUNC_BYPASS_SHIFT      (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_FUNC_BYPASS_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_SNP_MASK               (0x00100000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_SNP_SHIFT              (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_SNP_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_INHIBIT_PAYLOAD_MASK   (0x00400000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_INHIBIT_PAYLOAD_SHIFT  (0x00000016U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_INHIBIT_PAYLOAD_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_HEADER_SUBSTITUTE_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_HEADER_SUBSTITUTE_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_HEADER_SUBSTITUTE_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_CFG_SHIFT_MODE_MASK    (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_CFG_SHIFT_MODE_SHIFT   (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_CFG_SHIFT_MODE_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_INVERT_MODE_MASK       (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_INVERT_MODE_SHIFT      (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_INVERT_MODE_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_REGION_EN_MASK         (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_REGION_EN_SHIFT        (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_5_REGION_EN_MAX          (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_OUTBOUND_5 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_5_LWR_BASE_HW_MASK       (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_5_LWR_BASE_HW_SHIFT      (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_5_LWR_BASE_HW_MAX        (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_5_LWR_BASE_RW_MASK       (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_5_LWR_BASE_RW_SHIFT      (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_5_LWR_BASE_RW_MAX        (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_5 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_5_UPPER_BASE_RW_MASK   (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_5_UPPER_BASE_RW_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_5_UPPER_BASE_RW_MAX    (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_OUTBOUND_5 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_5_LIMIT_ADDR_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_5_LIMIT_ADDR_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_5_LIMIT_ADDR_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_5_LIMIT_ADDR_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_5_LIMIT_ADDR_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_5_LIMIT_ADDR_RW_MAX         (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_5 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_5_LWR_TARGET_RW_OUTBOUND_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_5_LWR_TARGET_RW_OUTBOUND_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_5_LWR_TARGET_RW_OUTBOUND_MAX (0xFFFFFFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_5 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_5_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_5_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_5_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_INBOUND_5 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_5_TYPE_MASK               (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_5_TYPE_SHIFT              (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_5_TYPE_MAX                (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_5_TC_MASK                 (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_5_TC_SHIFT                (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_5_TC_MAX                  (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_5_TD_MASK                 (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_5_TD_SHIFT                (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_5_TD_MAX                  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_5_ATTR_MASK               (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_5_ATTR_SHIFT              (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_5_ATTR_MAX                (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_5_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_5_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_5_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_5_CTRL_1_FUNC_NUM_MASK    (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_5_CTRL_1_FUNC_NUM_SHIFT   (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_5_CTRL_1_FUNC_NUM_MAX     (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_INBOUND_5 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_MSG_CODE_MASK           (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_MSG_CODE_SHIFT          (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_MSG_CODE_MAX            (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_BAR_NUM_MASK            (0x00000700U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_BAR_NUM_SHIFT           (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_BAR_NUM_MAX             (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_MSG_TYPE_MATCH_MODE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_MSG_TYPE_MATCH_MODE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_MSG_TYPE_MATCH_MODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_TC_MATCH_EN_MASK        (0x00004000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_TC_MATCH_EN_SHIFT       (0x0000000EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_TC_MATCH_EN_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_TD_MATCH_EN_MASK        (0x00008000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_TD_MATCH_EN_SHIFT       (0x0000000FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_TD_MATCH_EN_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_ATTR_MATCH_EN_MASK      (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_ATTR_MATCH_EN_SHIFT     (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_ATTR_MATCH_EN_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_FUNC_NUM_MATCH_EN_MASK  (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_FUNC_NUM_MATCH_EN_SHIFT (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_FUNC_NUM_MATCH_EN_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_MSG_CODE_MATCH_EN_MASK  (0x00200000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_MSG_CODE_MATCH_EN_SHIFT (0x00000015U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_MSG_CODE_MATCH_EN_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_SINGLE_ADDR_LOC_TRANS_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_SINGLE_ADDR_LOC_TRANS_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_SINGLE_ADDR_LOC_TRANS_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_RESPONSE_CODE_MASK      (0x03000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_RESPONSE_CODE_SHIFT     (0x00000018U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_RESPONSE_CODE_MAX       (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_FUZZY_TYPE_MATCH_CODE_MASK (0x08000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_FUZZY_TYPE_MATCH_CODE_SHIFT (0x0000001BU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_FUZZY_TYPE_MATCH_CODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_CFG_SHIFT_MODE_MASK     (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_CFG_SHIFT_MODE_SHIFT    (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_CFG_SHIFT_MODE_MAX      (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_INVERT_MODE_MASK        (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_INVERT_MODE_SHIFT       (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_INVERT_MODE_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_MATCH_MODE_MASK         (0x40000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_MATCH_MODE_SHIFT        (0x0000001EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_MATCH_MODE_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_REGION_EN_MASK          (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_REGION_EN_SHIFT         (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_5_REGION_EN_MAX           (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_INBOUND_5 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_5_LWR_BASE_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_5_LWR_BASE_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_5_LWR_BASE_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_5_LWR_BASE_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_5_LWR_BASE_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_5_LWR_BASE_RW_MAX         (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_INBOUND_5 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_5_UPPER_BASE_RW_MASK    (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_5_UPPER_BASE_RW_SHIFT   (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_5_UPPER_BASE_RW_MAX     (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_INBOUND_5 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_5_LIMIT_ADDR_HW_MASK         (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_5_LIMIT_ADDR_HW_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_5_LIMIT_ADDR_HW_MAX          (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_5_LIMIT_ADDR_RW_MASK         (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_5_LIMIT_ADDR_RW_SHIFT        (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_5_LIMIT_ADDR_RW_MAX          (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_INBOUND_5 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_5_LWR_TARGET_HW_MASK    (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_5_LWR_TARGET_HW_SHIFT   (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_5_LWR_TARGET_HW_MAX     (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_5_LWR_TARGET_RW_MASK    (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_5_LWR_TARGET_RW_SHIFT   (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_5_LWR_TARGET_RW_MAX     (0x0000FFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_INBOUND_5 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_5_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_5_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_5_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_OUTBOUND_6 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_6_TYPE_MASK              (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_6_TYPE_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_6_TYPE_MAX               (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_6_TC_MASK                (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_6_TC_SHIFT               (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_6_TC_MAX                 (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_6_TD_MASK                (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_6_TD_SHIFT               (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_6_TD_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_6_ATTR_MASK              (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_6_ATTR_SHIFT             (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_6_ATTR_MAX               (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_6_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_6_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_6_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_6_CTRL_1_FUNC_NUM_MASK   (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_6_CTRL_1_FUNC_NUM_SHIFT  (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_6_CTRL_1_FUNC_NUM_MAX    (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_OUTBOUND_6 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_MSG_CODE_MASK          (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_MSG_CODE_SHIFT         (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_MSG_CODE_MAX           (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_TAG_MASK               (0x0000FF00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_TAG_SHIFT              (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_TAG_MAX                (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_TAG_SUBSTITUTE_EN_MASK (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_TAG_SUBSTITUTE_EN_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_TAG_SUBSTITUTE_EN_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_FUNC_BYPASS_MASK       (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_FUNC_BYPASS_SHIFT      (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_FUNC_BYPASS_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_SNP_MASK               (0x00100000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_SNP_SHIFT              (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_SNP_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_INHIBIT_PAYLOAD_MASK   (0x00400000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_INHIBIT_PAYLOAD_SHIFT  (0x00000016U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_INHIBIT_PAYLOAD_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_HEADER_SUBSTITUTE_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_HEADER_SUBSTITUTE_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_HEADER_SUBSTITUTE_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_CFG_SHIFT_MODE_MASK    (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_CFG_SHIFT_MODE_SHIFT   (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_CFG_SHIFT_MODE_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_INVERT_MODE_MASK       (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_INVERT_MODE_SHIFT      (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_INVERT_MODE_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_REGION_EN_MASK         (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_REGION_EN_SHIFT        (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_6_REGION_EN_MAX          (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_OUTBOUND_6 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_6_LWR_BASE_HW_MASK       (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_6_LWR_BASE_HW_SHIFT      (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_6_LWR_BASE_HW_MAX        (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_6_LWR_BASE_RW_MASK       (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_6_LWR_BASE_RW_SHIFT      (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_6_LWR_BASE_RW_MAX        (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_6 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_6_UPPER_BASE_RW_MASK   (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_6_UPPER_BASE_RW_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_6_UPPER_BASE_RW_MAX    (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_OUTBOUND_6 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_6_LIMIT_ADDR_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_6_LIMIT_ADDR_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_6_LIMIT_ADDR_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_6_LIMIT_ADDR_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_6_LIMIT_ADDR_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_6_LIMIT_ADDR_RW_MAX         (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_6 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_6_LWR_TARGET_RW_OUTBOUND_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_6_LWR_TARGET_RW_OUTBOUND_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_6_LWR_TARGET_RW_OUTBOUND_MAX (0xFFFFFFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_6 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_6_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_6_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_6_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_INBOUND_6 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_6_TYPE_MASK               (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_6_TYPE_SHIFT              (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_6_TYPE_MAX                (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_6_TC_MASK                 (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_6_TC_SHIFT                (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_6_TC_MAX                  (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_6_TD_MASK                 (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_6_TD_SHIFT                (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_6_TD_MAX                  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_6_ATTR_MASK               (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_6_ATTR_SHIFT              (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_6_ATTR_MAX                (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_6_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_6_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_6_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_6_CTRL_1_FUNC_NUM_MASK    (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_6_CTRL_1_FUNC_NUM_SHIFT   (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_6_CTRL_1_FUNC_NUM_MAX     (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_INBOUND_6 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_MSG_CODE_MASK           (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_MSG_CODE_SHIFT          (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_MSG_CODE_MAX            (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_BAR_NUM_MASK            (0x00000700U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_BAR_NUM_SHIFT           (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_BAR_NUM_MAX             (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_MSG_TYPE_MATCH_MODE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_MSG_TYPE_MATCH_MODE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_MSG_TYPE_MATCH_MODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_TC_MATCH_EN_MASK        (0x00004000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_TC_MATCH_EN_SHIFT       (0x0000000EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_TC_MATCH_EN_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_TD_MATCH_EN_MASK        (0x00008000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_TD_MATCH_EN_SHIFT       (0x0000000FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_TD_MATCH_EN_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_ATTR_MATCH_EN_MASK      (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_ATTR_MATCH_EN_SHIFT     (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_ATTR_MATCH_EN_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_FUNC_NUM_MATCH_EN_MASK  (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_FUNC_NUM_MATCH_EN_SHIFT (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_FUNC_NUM_MATCH_EN_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_MSG_CODE_MATCH_EN_MASK  (0x00200000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_MSG_CODE_MATCH_EN_SHIFT (0x00000015U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_MSG_CODE_MATCH_EN_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_SINGLE_ADDR_LOC_TRANS_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_SINGLE_ADDR_LOC_TRANS_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_SINGLE_ADDR_LOC_TRANS_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_RESPONSE_CODE_MASK      (0x03000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_RESPONSE_CODE_SHIFT     (0x00000018U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_RESPONSE_CODE_MAX       (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_FUZZY_TYPE_MATCH_CODE_MASK (0x08000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_FUZZY_TYPE_MATCH_CODE_SHIFT (0x0000001BU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_FUZZY_TYPE_MATCH_CODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_CFG_SHIFT_MODE_MASK     (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_CFG_SHIFT_MODE_SHIFT    (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_CFG_SHIFT_MODE_MAX      (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_INVERT_MODE_MASK        (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_INVERT_MODE_SHIFT       (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_INVERT_MODE_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_MATCH_MODE_MASK         (0x40000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_MATCH_MODE_SHIFT        (0x0000001EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_MATCH_MODE_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_REGION_EN_MASK          (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_REGION_EN_SHIFT         (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_6_REGION_EN_MAX           (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_INBOUND_6 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_6_LWR_BASE_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_6_LWR_BASE_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_6_LWR_BASE_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_6_LWR_BASE_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_6_LWR_BASE_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_6_LWR_BASE_RW_MAX         (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_INBOUND_6 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_6_UPPER_BASE_RW_MASK    (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_6_UPPER_BASE_RW_SHIFT   (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_6_UPPER_BASE_RW_MAX     (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_INBOUND_6 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_6_LIMIT_ADDR_HW_MASK         (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_6_LIMIT_ADDR_HW_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_6_LIMIT_ADDR_HW_MAX          (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_6_LIMIT_ADDR_RW_MASK         (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_6_LIMIT_ADDR_RW_SHIFT        (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_6_LIMIT_ADDR_RW_MAX          (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_INBOUND_6 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_6_LWR_TARGET_HW_MASK    (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_6_LWR_TARGET_HW_SHIFT   (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_6_LWR_TARGET_HW_MAX     (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_6_LWR_TARGET_RW_MASK    (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_6_LWR_TARGET_RW_SHIFT   (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_6_LWR_TARGET_RW_MAX     (0x0000FFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_INBOUND_6 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_6_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_6_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_6_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_OUTBOUND_7 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_7_TYPE_MASK              (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_7_TYPE_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_7_TYPE_MAX               (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_7_TC_MASK                (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_7_TC_SHIFT               (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_7_TC_MAX                 (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_7_TD_MASK                (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_7_TD_SHIFT               (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_7_TD_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_7_ATTR_MASK              (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_7_ATTR_SHIFT             (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_7_ATTR_MAX               (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_7_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_7_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_7_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_7_CTRL_1_FUNC_NUM_MASK   (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_7_CTRL_1_FUNC_NUM_SHIFT  (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_7_CTRL_1_FUNC_NUM_MAX    (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_OUTBOUND_7 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_MSG_CODE_MASK          (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_MSG_CODE_SHIFT         (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_MSG_CODE_MAX           (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_TAG_MASK               (0x0000FF00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_TAG_SHIFT              (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_TAG_MAX                (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_TAG_SUBSTITUTE_EN_MASK (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_TAG_SUBSTITUTE_EN_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_TAG_SUBSTITUTE_EN_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_FUNC_BYPASS_MASK       (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_FUNC_BYPASS_SHIFT      (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_FUNC_BYPASS_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_SNP_MASK               (0x00100000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_SNP_SHIFT              (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_SNP_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_INHIBIT_PAYLOAD_MASK   (0x00400000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_INHIBIT_PAYLOAD_SHIFT  (0x00000016U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_INHIBIT_PAYLOAD_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_HEADER_SUBSTITUTE_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_HEADER_SUBSTITUTE_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_HEADER_SUBSTITUTE_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_CFG_SHIFT_MODE_MASK    (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_CFG_SHIFT_MODE_SHIFT   (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_CFG_SHIFT_MODE_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_INVERT_MODE_MASK       (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_INVERT_MODE_SHIFT      (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_INVERT_MODE_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_REGION_EN_MASK         (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_REGION_EN_SHIFT        (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_7_REGION_EN_MAX          (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_OUTBOUND_7 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_7_LWR_BASE_HW_MASK       (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_7_LWR_BASE_HW_SHIFT      (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_7_LWR_BASE_HW_MAX        (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_7_LWR_BASE_RW_MASK       (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_7_LWR_BASE_RW_SHIFT      (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_7_LWR_BASE_RW_MAX        (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_7 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_7_UPPER_BASE_RW_MASK   (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_7_UPPER_BASE_RW_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_7_UPPER_BASE_RW_MAX    (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_OUTBOUND_7 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_7_LIMIT_ADDR_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_7_LIMIT_ADDR_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_7_LIMIT_ADDR_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_7_LIMIT_ADDR_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_7_LIMIT_ADDR_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_7_LIMIT_ADDR_RW_MAX         (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_7 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_7_LWR_TARGET_RW_OUTBOUND_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_7_LWR_TARGET_RW_OUTBOUND_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_7_LWR_TARGET_RW_OUTBOUND_MAX (0xFFFFFFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_7 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_7_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_7_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_7_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_INBOUND_7 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_7_TYPE_MASK               (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_7_TYPE_SHIFT              (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_7_TYPE_MAX                (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_7_TC_MASK                 (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_7_TC_SHIFT                (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_7_TC_MAX                  (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_7_TD_MASK                 (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_7_TD_SHIFT                (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_7_TD_MAX                  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_7_ATTR_MASK               (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_7_ATTR_SHIFT              (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_7_ATTR_MAX                (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_7_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_7_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_7_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_7_CTRL_1_FUNC_NUM_MASK    (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_7_CTRL_1_FUNC_NUM_SHIFT   (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_7_CTRL_1_FUNC_NUM_MAX     (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_INBOUND_7 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_MSG_CODE_MASK           (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_MSG_CODE_SHIFT          (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_MSG_CODE_MAX            (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_BAR_NUM_MASK            (0x00000700U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_BAR_NUM_SHIFT           (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_BAR_NUM_MAX             (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_MSG_TYPE_MATCH_MODE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_MSG_TYPE_MATCH_MODE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_MSG_TYPE_MATCH_MODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_TC_MATCH_EN_MASK        (0x00004000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_TC_MATCH_EN_SHIFT       (0x0000000EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_TC_MATCH_EN_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_TD_MATCH_EN_MASK        (0x00008000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_TD_MATCH_EN_SHIFT       (0x0000000FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_TD_MATCH_EN_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_ATTR_MATCH_EN_MASK      (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_ATTR_MATCH_EN_SHIFT     (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_ATTR_MATCH_EN_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_FUNC_NUM_MATCH_EN_MASK  (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_FUNC_NUM_MATCH_EN_SHIFT (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_FUNC_NUM_MATCH_EN_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_MSG_CODE_MATCH_EN_MASK  (0x00200000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_MSG_CODE_MATCH_EN_SHIFT (0x00000015U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_MSG_CODE_MATCH_EN_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_SINGLE_ADDR_LOC_TRANS_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_SINGLE_ADDR_LOC_TRANS_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_SINGLE_ADDR_LOC_TRANS_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_RESPONSE_CODE_MASK      (0x03000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_RESPONSE_CODE_SHIFT     (0x00000018U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_RESPONSE_CODE_MAX       (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_FUZZY_TYPE_MATCH_CODE_MASK (0x08000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_FUZZY_TYPE_MATCH_CODE_SHIFT (0x0000001BU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_FUZZY_TYPE_MATCH_CODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_CFG_SHIFT_MODE_MASK     (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_CFG_SHIFT_MODE_SHIFT    (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_CFG_SHIFT_MODE_MAX      (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_INVERT_MODE_MASK        (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_INVERT_MODE_SHIFT       (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_INVERT_MODE_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_MATCH_MODE_MASK         (0x40000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_MATCH_MODE_SHIFT        (0x0000001EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_MATCH_MODE_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_REGION_EN_MASK          (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_REGION_EN_SHIFT         (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_7_REGION_EN_MAX           (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_INBOUND_7 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_7_LWR_BASE_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_7_LWR_BASE_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_7_LWR_BASE_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_7_LWR_BASE_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_7_LWR_BASE_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_7_LWR_BASE_RW_MAX         (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_INBOUND_7 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_7_UPPER_BASE_RW_MASK    (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_7_UPPER_BASE_RW_SHIFT   (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_7_UPPER_BASE_RW_MAX     (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_INBOUND_7 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_7_LIMIT_ADDR_HW_MASK         (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_7_LIMIT_ADDR_HW_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_7_LIMIT_ADDR_HW_MAX          (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_7_LIMIT_ADDR_RW_MASK         (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_7_LIMIT_ADDR_RW_SHIFT        (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_7_LIMIT_ADDR_RW_MAX          (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_INBOUND_7 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_7_LWR_TARGET_HW_MASK    (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_7_LWR_TARGET_HW_SHIFT   (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_7_LWR_TARGET_HW_MAX     (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_7_LWR_TARGET_RW_MASK    (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_7_LWR_TARGET_RW_SHIFT   (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_7_LWR_TARGET_RW_MAX     (0x0000FFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_INBOUND_7 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_7_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_7_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_7_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_OUTBOUND_8 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_8_TYPE_MASK              (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_8_TYPE_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_8_TYPE_MAX               (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_8_TC_MASK                (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_8_TC_SHIFT               (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_8_TC_MAX                 (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_8_TD_MASK                (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_8_TD_SHIFT               (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_8_TD_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_8_ATTR_MASK              (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_8_ATTR_SHIFT             (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_8_ATTR_MAX               (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_8_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_8_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_8_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_8_CTRL_1_FUNC_NUM_MASK   (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_8_CTRL_1_FUNC_NUM_SHIFT  (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_8_CTRL_1_FUNC_NUM_MAX    (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_OUTBOUND_8 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_MSG_CODE_MASK          (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_MSG_CODE_SHIFT         (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_MSG_CODE_MAX           (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_TAG_MASK               (0x0000FF00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_TAG_SHIFT              (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_TAG_MAX                (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_TAG_SUBSTITUTE_EN_MASK (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_TAG_SUBSTITUTE_EN_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_TAG_SUBSTITUTE_EN_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_FUNC_BYPASS_MASK       (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_FUNC_BYPASS_SHIFT      (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_FUNC_BYPASS_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_SNP_MASK               (0x00100000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_SNP_SHIFT              (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_SNP_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_INHIBIT_PAYLOAD_MASK   (0x00400000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_INHIBIT_PAYLOAD_SHIFT  (0x00000016U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_INHIBIT_PAYLOAD_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_HEADER_SUBSTITUTE_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_HEADER_SUBSTITUTE_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_HEADER_SUBSTITUTE_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_CFG_SHIFT_MODE_MASK    (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_CFG_SHIFT_MODE_SHIFT   (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_CFG_SHIFT_MODE_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_INVERT_MODE_MASK       (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_INVERT_MODE_SHIFT      (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_INVERT_MODE_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_REGION_EN_MASK         (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_REGION_EN_SHIFT        (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_8_REGION_EN_MAX          (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_OUTBOUND_8 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_8_LWR_BASE_HW_MASK       (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_8_LWR_BASE_HW_SHIFT      (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_8_LWR_BASE_HW_MAX        (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_8_LWR_BASE_RW_MASK       (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_8_LWR_BASE_RW_SHIFT      (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_8_LWR_BASE_RW_MAX        (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_8 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_8_UPPER_BASE_RW_MASK   (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_8_UPPER_BASE_RW_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_8_UPPER_BASE_RW_MAX    (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_OUTBOUND_8 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_8_LIMIT_ADDR_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_8_LIMIT_ADDR_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_8_LIMIT_ADDR_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_8_LIMIT_ADDR_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_8_LIMIT_ADDR_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_8_LIMIT_ADDR_RW_MAX         (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_8 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_8_LWR_TARGET_RW_OUTBOUND_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_8_LWR_TARGET_RW_OUTBOUND_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_8_LWR_TARGET_RW_OUTBOUND_MAX (0xFFFFFFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_8 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_8_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_8_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_8_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_INBOUND_8 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_8_TYPE_MASK               (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_8_TYPE_SHIFT              (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_8_TYPE_MAX                (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_8_TC_MASK                 (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_8_TC_SHIFT                (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_8_TC_MAX                  (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_8_TD_MASK                 (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_8_TD_SHIFT                (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_8_TD_MAX                  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_8_ATTR_MASK               (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_8_ATTR_SHIFT              (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_8_ATTR_MAX                (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_8_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_8_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_8_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_8_CTRL_1_FUNC_NUM_MASK    (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_8_CTRL_1_FUNC_NUM_SHIFT   (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_8_CTRL_1_FUNC_NUM_MAX     (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_INBOUND_8 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_MSG_CODE_MASK           (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_MSG_CODE_SHIFT          (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_MSG_CODE_MAX            (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_BAR_NUM_MASK            (0x00000700U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_BAR_NUM_SHIFT           (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_BAR_NUM_MAX             (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_MSG_TYPE_MATCH_MODE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_MSG_TYPE_MATCH_MODE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_MSG_TYPE_MATCH_MODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_TC_MATCH_EN_MASK        (0x00004000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_TC_MATCH_EN_SHIFT       (0x0000000EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_TC_MATCH_EN_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_TD_MATCH_EN_MASK        (0x00008000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_TD_MATCH_EN_SHIFT       (0x0000000FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_TD_MATCH_EN_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_ATTR_MATCH_EN_MASK      (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_ATTR_MATCH_EN_SHIFT     (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_ATTR_MATCH_EN_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_FUNC_NUM_MATCH_EN_MASK  (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_FUNC_NUM_MATCH_EN_SHIFT (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_FUNC_NUM_MATCH_EN_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_MSG_CODE_MATCH_EN_MASK  (0x00200000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_MSG_CODE_MATCH_EN_SHIFT (0x00000015U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_MSG_CODE_MATCH_EN_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_SINGLE_ADDR_LOC_TRANS_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_SINGLE_ADDR_LOC_TRANS_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_SINGLE_ADDR_LOC_TRANS_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_RESPONSE_CODE_MASK      (0x03000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_RESPONSE_CODE_SHIFT     (0x00000018U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_RESPONSE_CODE_MAX       (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_FUZZY_TYPE_MATCH_CODE_MASK (0x08000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_FUZZY_TYPE_MATCH_CODE_SHIFT (0x0000001BU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_FUZZY_TYPE_MATCH_CODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_CFG_SHIFT_MODE_MASK     (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_CFG_SHIFT_MODE_SHIFT    (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_CFG_SHIFT_MODE_MAX      (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_INVERT_MODE_MASK        (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_INVERT_MODE_SHIFT       (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_INVERT_MODE_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_MATCH_MODE_MASK         (0x40000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_MATCH_MODE_SHIFT        (0x0000001EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_MATCH_MODE_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_REGION_EN_MASK          (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_REGION_EN_SHIFT         (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_8_REGION_EN_MAX           (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_INBOUND_8 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_8_LWR_BASE_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_8_LWR_BASE_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_8_LWR_BASE_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_8_LWR_BASE_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_8_LWR_BASE_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_8_LWR_BASE_RW_MAX         (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_INBOUND_8 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_8_UPPER_BASE_RW_MASK    (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_8_UPPER_BASE_RW_SHIFT   (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_8_UPPER_BASE_RW_MAX     (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_INBOUND_8 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_8_LIMIT_ADDR_HW_MASK         (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_8_LIMIT_ADDR_HW_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_8_LIMIT_ADDR_HW_MAX          (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_8_LIMIT_ADDR_RW_MASK         (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_8_LIMIT_ADDR_RW_SHIFT        (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_8_LIMIT_ADDR_RW_MAX          (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_INBOUND_8 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_8_LWR_TARGET_HW_MASK    (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_8_LWR_TARGET_HW_SHIFT   (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_8_LWR_TARGET_HW_MAX     (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_8_LWR_TARGET_RW_MASK    (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_8_LWR_TARGET_RW_SHIFT   (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_8_LWR_TARGET_RW_MAX     (0x0000FFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_INBOUND_8 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_8_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_8_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_8_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_OUTBOUND_9 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_9_TYPE_MASK              (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_9_TYPE_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_9_TYPE_MAX               (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_9_TC_MASK                (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_9_TC_SHIFT               (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_9_TC_MAX                 (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_9_TD_MASK                (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_9_TD_SHIFT               (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_9_TD_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_9_ATTR_MASK              (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_9_ATTR_SHIFT             (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_9_ATTR_MAX               (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_9_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_9_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_9_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_9_CTRL_1_FUNC_NUM_MASK   (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_9_CTRL_1_FUNC_NUM_SHIFT  (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_9_CTRL_1_FUNC_NUM_MAX    (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_OUTBOUND_9 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_MSG_CODE_MASK          (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_MSG_CODE_SHIFT         (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_MSG_CODE_MAX           (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_TAG_MASK               (0x0000FF00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_TAG_SHIFT              (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_TAG_MAX                (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_TAG_SUBSTITUTE_EN_MASK (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_TAG_SUBSTITUTE_EN_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_TAG_SUBSTITUTE_EN_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_FUNC_BYPASS_MASK       (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_FUNC_BYPASS_SHIFT      (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_FUNC_BYPASS_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_SNP_MASK               (0x00100000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_SNP_SHIFT              (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_SNP_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_INHIBIT_PAYLOAD_MASK   (0x00400000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_INHIBIT_PAYLOAD_SHIFT  (0x00000016U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_INHIBIT_PAYLOAD_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_HEADER_SUBSTITUTE_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_HEADER_SUBSTITUTE_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_HEADER_SUBSTITUTE_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_CFG_SHIFT_MODE_MASK    (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_CFG_SHIFT_MODE_SHIFT   (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_CFG_SHIFT_MODE_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_INVERT_MODE_MASK       (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_INVERT_MODE_SHIFT      (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_INVERT_MODE_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_REGION_EN_MASK         (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_REGION_EN_SHIFT        (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_9_REGION_EN_MAX          (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_OUTBOUND_9 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_9_LWR_BASE_HW_MASK       (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_9_LWR_BASE_HW_SHIFT      (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_9_LWR_BASE_HW_MAX        (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_9_LWR_BASE_RW_MASK       (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_9_LWR_BASE_RW_SHIFT      (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_9_LWR_BASE_RW_MAX        (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_9 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_9_UPPER_BASE_RW_MASK   (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_9_UPPER_BASE_RW_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_9_UPPER_BASE_RW_MAX    (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_OUTBOUND_9 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_9_LIMIT_ADDR_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_9_LIMIT_ADDR_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_9_LIMIT_ADDR_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_9_LIMIT_ADDR_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_9_LIMIT_ADDR_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_9_LIMIT_ADDR_RW_MAX         (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_9 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_9_LWR_TARGET_RW_OUTBOUND_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_9_LWR_TARGET_RW_OUTBOUND_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_9_LWR_TARGET_RW_OUTBOUND_MAX (0xFFFFFFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_9 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_9_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_9_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_9_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_INBOUND_9 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_9_TYPE_MASK               (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_9_TYPE_SHIFT              (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_9_TYPE_MAX                (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_9_TC_MASK                 (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_9_TC_SHIFT                (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_9_TC_MAX                  (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_9_TD_MASK                 (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_9_TD_SHIFT                (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_9_TD_MAX                  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_9_ATTR_MASK               (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_9_ATTR_SHIFT              (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_9_ATTR_MAX                (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_9_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_9_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_9_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_9_CTRL_1_FUNC_NUM_MASK    (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_9_CTRL_1_FUNC_NUM_SHIFT   (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_9_CTRL_1_FUNC_NUM_MAX     (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_INBOUND_9 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_MSG_CODE_MASK           (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_MSG_CODE_SHIFT          (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_MSG_CODE_MAX            (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_BAR_NUM_MASK            (0x00000700U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_BAR_NUM_SHIFT           (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_BAR_NUM_MAX             (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_MSG_TYPE_MATCH_MODE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_MSG_TYPE_MATCH_MODE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_MSG_TYPE_MATCH_MODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_TC_MATCH_EN_MASK        (0x00004000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_TC_MATCH_EN_SHIFT       (0x0000000EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_TC_MATCH_EN_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_TD_MATCH_EN_MASK        (0x00008000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_TD_MATCH_EN_SHIFT       (0x0000000FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_TD_MATCH_EN_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_ATTR_MATCH_EN_MASK      (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_ATTR_MATCH_EN_SHIFT     (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_ATTR_MATCH_EN_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_FUNC_NUM_MATCH_EN_MASK  (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_FUNC_NUM_MATCH_EN_SHIFT (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_FUNC_NUM_MATCH_EN_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_MSG_CODE_MATCH_EN_MASK  (0x00200000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_MSG_CODE_MATCH_EN_SHIFT (0x00000015U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_MSG_CODE_MATCH_EN_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_SINGLE_ADDR_LOC_TRANS_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_SINGLE_ADDR_LOC_TRANS_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_SINGLE_ADDR_LOC_TRANS_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_RESPONSE_CODE_MASK      (0x03000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_RESPONSE_CODE_SHIFT     (0x00000018U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_RESPONSE_CODE_MAX       (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_FUZZY_TYPE_MATCH_CODE_MASK (0x08000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_FUZZY_TYPE_MATCH_CODE_SHIFT (0x0000001BU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_FUZZY_TYPE_MATCH_CODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_CFG_SHIFT_MODE_MASK     (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_CFG_SHIFT_MODE_SHIFT    (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_CFG_SHIFT_MODE_MAX      (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_INVERT_MODE_MASK        (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_INVERT_MODE_SHIFT       (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_INVERT_MODE_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_MATCH_MODE_MASK         (0x40000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_MATCH_MODE_SHIFT        (0x0000001EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_MATCH_MODE_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_REGION_EN_MASK          (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_REGION_EN_SHIFT         (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_9_REGION_EN_MAX           (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_INBOUND_9 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_9_LWR_BASE_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_9_LWR_BASE_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_9_LWR_BASE_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_9_LWR_BASE_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_9_LWR_BASE_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_9_LWR_BASE_RW_MAX         (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_INBOUND_9 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_9_UPPER_BASE_RW_MASK    (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_9_UPPER_BASE_RW_SHIFT   (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_9_UPPER_BASE_RW_MAX     (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_INBOUND_9 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_9_LIMIT_ADDR_HW_MASK         (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_9_LIMIT_ADDR_HW_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_9_LIMIT_ADDR_HW_MAX          (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_9_LIMIT_ADDR_RW_MASK         (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_9_LIMIT_ADDR_RW_SHIFT        (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_9_LIMIT_ADDR_RW_MAX          (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_INBOUND_9 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_9_LWR_TARGET_HW_MASK    (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_9_LWR_TARGET_HW_SHIFT   (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_9_LWR_TARGET_HW_MAX     (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_9_LWR_TARGET_RW_MASK    (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_9_LWR_TARGET_RW_SHIFT   (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_9_LWR_TARGET_RW_MAX     (0x0000FFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_INBOUND_9 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_9_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_9_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_9_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_OUTBOUND_10 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_10_TYPE_MASK             (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_10_TYPE_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_10_TYPE_MAX              (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_10_TC_MASK               (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_10_TC_SHIFT              (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_10_TC_MAX                (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_10_TD_MASK               (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_10_TD_SHIFT              (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_10_TD_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_10_ATTR_MASK             (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_10_ATTR_SHIFT            (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_10_ATTR_MAX              (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_10_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_10_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_10_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_10_CTRL_1_FUNC_NUM_MASK  (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_10_CTRL_1_FUNC_NUM_SHIFT (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_10_CTRL_1_FUNC_NUM_MAX   (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_OUTBOUND_10 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_MSG_CODE_MASK         (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_MSG_CODE_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_MSG_CODE_MAX          (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_TAG_MASK              (0x0000FF00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_TAG_SHIFT             (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_TAG_MAX               (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_TAG_SUBSTITUTE_EN_MASK (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_TAG_SUBSTITUTE_EN_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_TAG_SUBSTITUTE_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_FUNC_BYPASS_MASK      (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_FUNC_BYPASS_SHIFT     (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_FUNC_BYPASS_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_SNP_MASK              (0x00100000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_SNP_SHIFT             (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_SNP_MAX               (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_INHIBIT_PAYLOAD_MASK  (0x00400000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_INHIBIT_PAYLOAD_SHIFT (0x00000016U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_INHIBIT_PAYLOAD_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_HEADER_SUBSTITUTE_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_HEADER_SUBSTITUTE_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_HEADER_SUBSTITUTE_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_CFG_SHIFT_MODE_MASK   (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_CFG_SHIFT_MODE_SHIFT  (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_CFG_SHIFT_MODE_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_INVERT_MODE_MASK      (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_INVERT_MODE_SHIFT     (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_INVERT_MODE_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_REGION_EN_MASK        (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_REGION_EN_SHIFT       (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_10_REGION_EN_MAX         (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_OUTBOUND_10 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_10_LWR_BASE_HW_MASK      (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_10_LWR_BASE_HW_SHIFT     (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_10_LWR_BASE_HW_MAX       (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_10_LWR_BASE_RW_MASK      (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_10_LWR_BASE_RW_SHIFT     (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_10_LWR_BASE_RW_MAX       (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_10 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_10_UPPER_BASE_RW_MASK  (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_10_UPPER_BASE_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_10_UPPER_BASE_RW_MAX   (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_OUTBOUND_10 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_10_LIMIT_ADDR_HW_MASK       (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_10_LIMIT_ADDR_HW_SHIFT      (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_10_LIMIT_ADDR_HW_MAX        (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_10_LIMIT_ADDR_RW_MASK       (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_10_LIMIT_ADDR_RW_SHIFT      (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_10_LIMIT_ADDR_RW_MAX        (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_10 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_10_LWR_TARGET_RW_OUTBOUND_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_10_LWR_TARGET_RW_OUTBOUND_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_10_LWR_TARGET_RW_OUTBOUND_MAX (0xFFFFFFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_10 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_10_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_10_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_10_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_INBOUND_10 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_10_TYPE_MASK              (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_10_TYPE_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_10_TYPE_MAX               (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_10_TC_MASK                (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_10_TC_SHIFT               (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_10_TC_MAX                 (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_10_TD_MASK                (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_10_TD_SHIFT               (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_10_TD_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_10_ATTR_MASK              (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_10_ATTR_SHIFT             (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_10_ATTR_MAX               (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_10_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_10_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_10_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_10_CTRL_1_FUNC_NUM_MASK   (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_10_CTRL_1_FUNC_NUM_SHIFT  (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_10_CTRL_1_FUNC_NUM_MAX    (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_INBOUND_10 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_MSG_CODE_MASK          (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_MSG_CODE_SHIFT         (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_MSG_CODE_MAX           (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_BAR_NUM_MASK           (0x00000700U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_BAR_NUM_SHIFT          (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_BAR_NUM_MAX            (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_MSG_TYPE_MATCH_MODE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_MSG_TYPE_MATCH_MODE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_MSG_TYPE_MATCH_MODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_TC_MATCH_EN_MASK       (0x00004000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_TC_MATCH_EN_SHIFT      (0x0000000EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_TC_MATCH_EN_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_TD_MATCH_EN_MASK       (0x00008000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_TD_MATCH_EN_SHIFT      (0x0000000FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_TD_MATCH_EN_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_ATTR_MATCH_EN_MASK     (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_ATTR_MATCH_EN_SHIFT    (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_ATTR_MATCH_EN_MAX      (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_FUNC_NUM_MATCH_EN_MASK (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_FUNC_NUM_MATCH_EN_SHIFT (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_FUNC_NUM_MATCH_EN_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_MSG_CODE_MATCH_EN_MASK (0x00200000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_MSG_CODE_MATCH_EN_SHIFT (0x00000015U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_MSG_CODE_MATCH_EN_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_SINGLE_ADDR_LOC_TRANS_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_SINGLE_ADDR_LOC_TRANS_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_SINGLE_ADDR_LOC_TRANS_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_RESPONSE_CODE_MASK     (0x03000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_RESPONSE_CODE_SHIFT    (0x00000018U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_RESPONSE_CODE_MAX      (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_FUZZY_TYPE_MATCH_CODE_MASK (0x08000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_FUZZY_TYPE_MATCH_CODE_SHIFT (0x0000001BU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_FUZZY_TYPE_MATCH_CODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_CFG_SHIFT_MODE_MASK    (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_CFG_SHIFT_MODE_SHIFT   (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_CFG_SHIFT_MODE_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_INVERT_MODE_MASK       (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_INVERT_MODE_SHIFT      (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_INVERT_MODE_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_MATCH_MODE_MASK        (0x40000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_MATCH_MODE_SHIFT       (0x0000001EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_MATCH_MODE_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_REGION_EN_MASK         (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_REGION_EN_SHIFT        (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_10_REGION_EN_MAX          (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_INBOUND_10 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_10_LWR_BASE_HW_MASK       (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_10_LWR_BASE_HW_SHIFT      (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_10_LWR_BASE_HW_MAX        (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_10_LWR_BASE_RW_MASK       (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_10_LWR_BASE_RW_SHIFT      (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_10_LWR_BASE_RW_MAX        (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_INBOUND_10 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_10_UPPER_BASE_RW_MASK   (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_10_UPPER_BASE_RW_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_10_UPPER_BASE_RW_MAX    (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_INBOUND_10 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_10_LIMIT_ADDR_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_10_LIMIT_ADDR_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_10_LIMIT_ADDR_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_10_LIMIT_ADDR_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_10_LIMIT_ADDR_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_10_LIMIT_ADDR_RW_MAX         (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_INBOUND_10 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_10_LWR_TARGET_HW_MASK   (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_10_LWR_TARGET_HW_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_10_LWR_TARGET_HW_MAX    (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_10_LWR_TARGET_RW_MASK   (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_10_LWR_TARGET_RW_SHIFT  (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_10_LWR_TARGET_RW_MAX    (0x0000FFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_INBOUND_10 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_10_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_10_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_10_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_OUTBOUND_11 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_11_TYPE_MASK             (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_11_TYPE_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_11_TYPE_MAX              (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_11_TC_MASK               (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_11_TC_SHIFT              (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_11_TC_MAX                (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_11_TD_MASK               (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_11_TD_SHIFT              (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_11_TD_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_11_ATTR_MASK             (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_11_ATTR_SHIFT            (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_11_ATTR_MAX              (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_11_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_11_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_11_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_11_CTRL_1_FUNC_NUM_MASK  (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_11_CTRL_1_FUNC_NUM_SHIFT (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_11_CTRL_1_FUNC_NUM_MAX   (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_OUTBOUND_11 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_MSG_CODE_MASK         (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_MSG_CODE_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_MSG_CODE_MAX          (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_TAG_MASK              (0x0000FF00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_TAG_SHIFT             (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_TAG_MAX               (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_TAG_SUBSTITUTE_EN_MASK (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_TAG_SUBSTITUTE_EN_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_TAG_SUBSTITUTE_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_FUNC_BYPASS_MASK      (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_FUNC_BYPASS_SHIFT     (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_FUNC_BYPASS_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_SNP_MASK              (0x00100000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_SNP_SHIFT             (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_SNP_MAX               (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_INHIBIT_PAYLOAD_MASK  (0x00400000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_INHIBIT_PAYLOAD_SHIFT (0x00000016U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_INHIBIT_PAYLOAD_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_HEADER_SUBSTITUTE_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_HEADER_SUBSTITUTE_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_HEADER_SUBSTITUTE_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_CFG_SHIFT_MODE_MASK   (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_CFG_SHIFT_MODE_SHIFT  (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_CFG_SHIFT_MODE_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_INVERT_MODE_MASK      (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_INVERT_MODE_SHIFT     (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_INVERT_MODE_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_REGION_EN_MASK        (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_REGION_EN_SHIFT       (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_11_REGION_EN_MAX         (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_OUTBOUND_11 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_11_LWR_BASE_HW_MASK      (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_11_LWR_BASE_HW_SHIFT     (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_11_LWR_BASE_HW_MAX       (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_11_LWR_BASE_RW_MASK      (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_11_LWR_BASE_RW_SHIFT     (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_11_LWR_BASE_RW_MAX       (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_11 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_11_UPPER_BASE_RW_MASK  (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_11_UPPER_BASE_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_11_UPPER_BASE_RW_MAX   (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_OUTBOUND_11 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_11_LIMIT_ADDR_HW_MASK       (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_11_LIMIT_ADDR_HW_SHIFT      (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_11_LIMIT_ADDR_HW_MAX        (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_11_LIMIT_ADDR_RW_MASK       (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_11_LIMIT_ADDR_RW_SHIFT      (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_11_LIMIT_ADDR_RW_MAX        (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_11 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_11_LWR_TARGET_RW_OUTBOUND_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_11_LWR_TARGET_RW_OUTBOUND_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_11_LWR_TARGET_RW_OUTBOUND_MAX (0xFFFFFFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_11 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_11_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_11_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_11_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_INBOUND_11 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_11_TYPE_MASK              (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_11_TYPE_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_11_TYPE_MAX               (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_11_TC_MASK                (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_11_TC_SHIFT               (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_11_TC_MAX                 (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_11_TD_MASK                (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_11_TD_SHIFT               (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_11_TD_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_11_ATTR_MASK              (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_11_ATTR_SHIFT             (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_11_ATTR_MAX               (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_11_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_11_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_11_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_11_CTRL_1_FUNC_NUM_MASK   (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_11_CTRL_1_FUNC_NUM_SHIFT  (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_11_CTRL_1_FUNC_NUM_MAX    (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_INBOUND_11 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_MSG_CODE_MASK          (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_MSG_CODE_SHIFT         (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_MSG_CODE_MAX           (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_BAR_NUM_MASK           (0x00000700U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_BAR_NUM_SHIFT          (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_BAR_NUM_MAX            (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_MSG_TYPE_MATCH_MODE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_MSG_TYPE_MATCH_MODE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_MSG_TYPE_MATCH_MODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_TC_MATCH_EN_MASK       (0x00004000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_TC_MATCH_EN_SHIFT      (0x0000000EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_TC_MATCH_EN_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_TD_MATCH_EN_MASK       (0x00008000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_TD_MATCH_EN_SHIFT      (0x0000000FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_TD_MATCH_EN_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_ATTR_MATCH_EN_MASK     (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_ATTR_MATCH_EN_SHIFT    (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_ATTR_MATCH_EN_MAX      (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_FUNC_NUM_MATCH_EN_MASK (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_FUNC_NUM_MATCH_EN_SHIFT (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_FUNC_NUM_MATCH_EN_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_MSG_CODE_MATCH_EN_MASK (0x00200000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_MSG_CODE_MATCH_EN_SHIFT (0x00000015U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_MSG_CODE_MATCH_EN_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_SINGLE_ADDR_LOC_TRANS_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_SINGLE_ADDR_LOC_TRANS_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_SINGLE_ADDR_LOC_TRANS_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_RESPONSE_CODE_MASK     (0x03000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_RESPONSE_CODE_SHIFT    (0x00000018U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_RESPONSE_CODE_MAX      (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_FUZZY_TYPE_MATCH_CODE_MASK (0x08000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_FUZZY_TYPE_MATCH_CODE_SHIFT (0x0000001BU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_FUZZY_TYPE_MATCH_CODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_CFG_SHIFT_MODE_MASK    (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_CFG_SHIFT_MODE_SHIFT   (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_CFG_SHIFT_MODE_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_INVERT_MODE_MASK       (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_INVERT_MODE_SHIFT      (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_INVERT_MODE_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_MATCH_MODE_MASK        (0x40000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_MATCH_MODE_SHIFT       (0x0000001EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_MATCH_MODE_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_REGION_EN_MASK         (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_REGION_EN_SHIFT        (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_11_REGION_EN_MAX          (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_INBOUND_11 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_11_LWR_BASE_HW_MASK       (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_11_LWR_BASE_HW_SHIFT      (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_11_LWR_BASE_HW_MAX        (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_11_LWR_BASE_RW_MASK       (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_11_LWR_BASE_RW_SHIFT      (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_11_LWR_BASE_RW_MAX        (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_INBOUND_11 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_11_UPPER_BASE_RW_MASK   (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_11_UPPER_BASE_RW_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_11_UPPER_BASE_RW_MAX    (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_INBOUND_11 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_11_LIMIT_ADDR_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_11_LIMIT_ADDR_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_11_LIMIT_ADDR_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_11_LIMIT_ADDR_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_11_LIMIT_ADDR_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_11_LIMIT_ADDR_RW_MAX         (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_INBOUND_11 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_11_LWR_TARGET_HW_MASK   (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_11_LWR_TARGET_HW_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_11_LWR_TARGET_HW_MAX    (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_11_LWR_TARGET_RW_MASK   (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_11_LWR_TARGET_RW_SHIFT  (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_11_LWR_TARGET_RW_MAX    (0x0000FFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_INBOUND_11 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_11_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_11_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_11_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_OUTBOUND_12 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_12_TYPE_MASK             (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_12_TYPE_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_12_TYPE_MAX              (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_12_TC_MASK               (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_12_TC_SHIFT              (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_12_TC_MAX                (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_12_TD_MASK               (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_12_TD_SHIFT              (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_12_TD_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_12_ATTR_MASK             (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_12_ATTR_SHIFT            (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_12_ATTR_MAX              (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_12_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_12_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_12_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_12_CTRL_1_FUNC_NUM_MASK  (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_12_CTRL_1_FUNC_NUM_SHIFT (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_12_CTRL_1_FUNC_NUM_MAX   (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_OUTBOUND_12 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_MSG_CODE_MASK         (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_MSG_CODE_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_MSG_CODE_MAX          (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_TAG_MASK              (0x0000FF00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_TAG_SHIFT             (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_TAG_MAX               (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_TAG_SUBSTITUTE_EN_MASK (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_TAG_SUBSTITUTE_EN_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_TAG_SUBSTITUTE_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_FUNC_BYPASS_MASK      (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_FUNC_BYPASS_SHIFT     (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_FUNC_BYPASS_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_SNP_MASK              (0x00100000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_SNP_SHIFT             (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_SNP_MAX               (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_INHIBIT_PAYLOAD_MASK  (0x00400000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_INHIBIT_PAYLOAD_SHIFT (0x00000016U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_INHIBIT_PAYLOAD_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_HEADER_SUBSTITUTE_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_HEADER_SUBSTITUTE_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_HEADER_SUBSTITUTE_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_CFG_SHIFT_MODE_MASK   (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_CFG_SHIFT_MODE_SHIFT  (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_CFG_SHIFT_MODE_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_INVERT_MODE_MASK      (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_INVERT_MODE_SHIFT     (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_INVERT_MODE_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_REGION_EN_MASK        (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_REGION_EN_SHIFT       (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_12_REGION_EN_MAX         (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_OUTBOUND_12 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_12_LWR_BASE_HW_MASK      (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_12_LWR_BASE_HW_SHIFT     (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_12_LWR_BASE_HW_MAX       (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_12_LWR_BASE_RW_MASK      (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_12_LWR_BASE_RW_SHIFT     (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_12_LWR_BASE_RW_MAX       (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_12 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_12_UPPER_BASE_RW_MASK  (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_12_UPPER_BASE_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_12_UPPER_BASE_RW_MAX   (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_OUTBOUND_12 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_12_LIMIT_ADDR_HW_MASK       (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_12_LIMIT_ADDR_HW_SHIFT      (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_12_LIMIT_ADDR_HW_MAX        (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_12_LIMIT_ADDR_RW_MASK       (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_12_LIMIT_ADDR_RW_SHIFT      (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_12_LIMIT_ADDR_RW_MAX        (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_12 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_12_LWR_TARGET_RW_OUTBOUND_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_12_LWR_TARGET_RW_OUTBOUND_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_12_LWR_TARGET_RW_OUTBOUND_MAX (0xFFFFFFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_12 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_12_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_12_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_12_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_INBOUND_12 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_12_TYPE_MASK              (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_12_TYPE_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_12_TYPE_MAX               (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_12_TC_MASK                (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_12_TC_SHIFT               (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_12_TC_MAX                 (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_12_TD_MASK                (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_12_TD_SHIFT               (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_12_TD_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_12_ATTR_MASK              (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_12_ATTR_SHIFT             (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_12_ATTR_MAX               (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_12_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_12_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_12_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_12_CTRL_1_FUNC_NUM_MASK   (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_12_CTRL_1_FUNC_NUM_SHIFT  (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_12_CTRL_1_FUNC_NUM_MAX    (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_INBOUND_12 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_MSG_CODE_MASK          (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_MSG_CODE_SHIFT         (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_MSG_CODE_MAX           (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_BAR_NUM_MASK           (0x00000700U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_BAR_NUM_SHIFT          (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_BAR_NUM_MAX            (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_MSG_TYPE_MATCH_MODE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_MSG_TYPE_MATCH_MODE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_MSG_TYPE_MATCH_MODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_TC_MATCH_EN_MASK       (0x00004000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_TC_MATCH_EN_SHIFT      (0x0000000EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_TC_MATCH_EN_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_TD_MATCH_EN_MASK       (0x00008000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_TD_MATCH_EN_SHIFT      (0x0000000FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_TD_MATCH_EN_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_ATTR_MATCH_EN_MASK     (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_ATTR_MATCH_EN_SHIFT    (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_ATTR_MATCH_EN_MAX      (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_FUNC_NUM_MATCH_EN_MASK (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_FUNC_NUM_MATCH_EN_SHIFT (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_FUNC_NUM_MATCH_EN_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_MSG_CODE_MATCH_EN_MASK (0x00200000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_MSG_CODE_MATCH_EN_SHIFT (0x00000015U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_MSG_CODE_MATCH_EN_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_SINGLE_ADDR_LOC_TRANS_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_SINGLE_ADDR_LOC_TRANS_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_SINGLE_ADDR_LOC_TRANS_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_RESPONSE_CODE_MASK     (0x03000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_RESPONSE_CODE_SHIFT    (0x00000018U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_RESPONSE_CODE_MAX      (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_FUZZY_TYPE_MATCH_CODE_MASK (0x08000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_FUZZY_TYPE_MATCH_CODE_SHIFT (0x0000001BU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_FUZZY_TYPE_MATCH_CODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_CFG_SHIFT_MODE_MASK    (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_CFG_SHIFT_MODE_SHIFT   (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_CFG_SHIFT_MODE_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_INVERT_MODE_MASK       (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_INVERT_MODE_SHIFT      (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_INVERT_MODE_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_MATCH_MODE_MASK        (0x40000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_MATCH_MODE_SHIFT       (0x0000001EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_MATCH_MODE_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_REGION_EN_MASK         (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_REGION_EN_SHIFT        (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_12_REGION_EN_MAX          (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_INBOUND_12 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_12_LWR_BASE_HW_MASK       (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_12_LWR_BASE_HW_SHIFT      (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_12_LWR_BASE_HW_MAX        (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_12_LWR_BASE_RW_MASK       (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_12_LWR_BASE_RW_SHIFT      (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_12_LWR_BASE_RW_MAX        (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_INBOUND_12 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_12_UPPER_BASE_RW_MASK   (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_12_UPPER_BASE_RW_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_12_UPPER_BASE_RW_MAX    (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_INBOUND_12 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_12_LIMIT_ADDR_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_12_LIMIT_ADDR_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_12_LIMIT_ADDR_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_12_LIMIT_ADDR_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_12_LIMIT_ADDR_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_12_LIMIT_ADDR_RW_MAX         (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_INBOUND_12 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_12_LWR_TARGET_HW_MASK   (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_12_LWR_TARGET_HW_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_12_LWR_TARGET_HW_MAX    (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_12_LWR_TARGET_RW_MASK   (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_12_LWR_TARGET_RW_SHIFT  (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_12_LWR_TARGET_RW_MAX    (0x0000FFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_INBOUND_12 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_12_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_12_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_12_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_OUTBOUND_13 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_13_TYPE_MASK             (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_13_TYPE_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_13_TYPE_MAX              (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_13_TC_MASK               (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_13_TC_SHIFT              (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_13_TC_MAX                (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_13_TD_MASK               (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_13_TD_SHIFT              (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_13_TD_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_13_ATTR_MASK             (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_13_ATTR_SHIFT            (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_13_ATTR_MAX              (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_13_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_13_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_13_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_13_CTRL_1_FUNC_NUM_MASK  (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_13_CTRL_1_FUNC_NUM_SHIFT (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_13_CTRL_1_FUNC_NUM_MAX   (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_OUTBOUND_13 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_MSG_CODE_MASK         (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_MSG_CODE_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_MSG_CODE_MAX          (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_TAG_MASK              (0x0000FF00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_TAG_SHIFT             (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_TAG_MAX               (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_TAG_SUBSTITUTE_EN_MASK (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_TAG_SUBSTITUTE_EN_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_TAG_SUBSTITUTE_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_FUNC_BYPASS_MASK      (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_FUNC_BYPASS_SHIFT     (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_FUNC_BYPASS_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_SNP_MASK              (0x00100000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_SNP_SHIFT             (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_SNP_MAX               (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_INHIBIT_PAYLOAD_MASK  (0x00400000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_INHIBIT_PAYLOAD_SHIFT (0x00000016U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_INHIBIT_PAYLOAD_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_HEADER_SUBSTITUTE_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_HEADER_SUBSTITUTE_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_HEADER_SUBSTITUTE_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_CFG_SHIFT_MODE_MASK   (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_CFG_SHIFT_MODE_SHIFT  (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_CFG_SHIFT_MODE_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_INVERT_MODE_MASK      (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_INVERT_MODE_SHIFT     (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_INVERT_MODE_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_REGION_EN_MASK        (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_REGION_EN_SHIFT       (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_13_REGION_EN_MAX         (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_OUTBOUND_13 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_13_LWR_BASE_HW_MASK      (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_13_LWR_BASE_HW_SHIFT     (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_13_LWR_BASE_HW_MAX       (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_13_LWR_BASE_RW_MASK      (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_13_LWR_BASE_RW_SHIFT     (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_13_LWR_BASE_RW_MAX       (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_13 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_13_UPPER_BASE_RW_MASK  (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_13_UPPER_BASE_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_13_UPPER_BASE_RW_MAX   (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_OUTBOUND_13 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_13_LIMIT_ADDR_HW_MASK       (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_13_LIMIT_ADDR_HW_SHIFT      (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_13_LIMIT_ADDR_HW_MAX        (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_13_LIMIT_ADDR_RW_MASK       (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_13_LIMIT_ADDR_RW_SHIFT      (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_13_LIMIT_ADDR_RW_MAX        (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_13 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_13_LWR_TARGET_RW_OUTBOUND_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_13_LWR_TARGET_RW_OUTBOUND_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_13_LWR_TARGET_RW_OUTBOUND_MAX (0xFFFFFFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_13 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_13_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_13_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_13_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_INBOUND_13 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_13_TYPE_MASK              (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_13_TYPE_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_13_TYPE_MAX               (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_13_TC_MASK                (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_13_TC_SHIFT               (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_13_TC_MAX                 (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_13_TD_MASK                (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_13_TD_SHIFT               (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_13_TD_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_13_ATTR_MASK              (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_13_ATTR_SHIFT             (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_13_ATTR_MAX               (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_13_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_13_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_13_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_13_CTRL_1_FUNC_NUM_MASK   (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_13_CTRL_1_FUNC_NUM_SHIFT  (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_13_CTRL_1_FUNC_NUM_MAX    (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_INBOUND_13 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_MSG_CODE_MASK          (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_MSG_CODE_SHIFT         (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_MSG_CODE_MAX           (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_BAR_NUM_MASK           (0x00000700U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_BAR_NUM_SHIFT          (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_BAR_NUM_MAX            (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_MSG_TYPE_MATCH_MODE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_MSG_TYPE_MATCH_MODE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_MSG_TYPE_MATCH_MODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_TC_MATCH_EN_MASK       (0x00004000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_TC_MATCH_EN_SHIFT      (0x0000000EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_TC_MATCH_EN_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_TD_MATCH_EN_MASK       (0x00008000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_TD_MATCH_EN_SHIFT      (0x0000000FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_TD_MATCH_EN_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_ATTR_MATCH_EN_MASK     (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_ATTR_MATCH_EN_SHIFT    (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_ATTR_MATCH_EN_MAX      (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_FUNC_NUM_MATCH_EN_MASK (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_FUNC_NUM_MATCH_EN_SHIFT (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_FUNC_NUM_MATCH_EN_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_MSG_CODE_MATCH_EN_MASK (0x00200000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_MSG_CODE_MATCH_EN_SHIFT (0x00000015U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_MSG_CODE_MATCH_EN_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_SINGLE_ADDR_LOC_TRANS_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_SINGLE_ADDR_LOC_TRANS_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_SINGLE_ADDR_LOC_TRANS_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_RESPONSE_CODE_MASK     (0x03000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_RESPONSE_CODE_SHIFT    (0x00000018U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_RESPONSE_CODE_MAX      (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_FUZZY_TYPE_MATCH_CODE_MASK (0x08000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_FUZZY_TYPE_MATCH_CODE_SHIFT (0x0000001BU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_FUZZY_TYPE_MATCH_CODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_CFG_SHIFT_MODE_MASK    (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_CFG_SHIFT_MODE_SHIFT   (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_CFG_SHIFT_MODE_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_INVERT_MODE_MASK       (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_INVERT_MODE_SHIFT      (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_INVERT_MODE_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_MATCH_MODE_MASK        (0x40000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_MATCH_MODE_SHIFT       (0x0000001EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_MATCH_MODE_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_REGION_EN_MASK         (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_REGION_EN_SHIFT        (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_13_REGION_EN_MAX          (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_INBOUND_13 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_13_LWR_BASE_HW_MASK       (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_13_LWR_BASE_HW_SHIFT      (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_13_LWR_BASE_HW_MAX        (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_13_LWR_BASE_RW_MASK       (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_13_LWR_BASE_RW_SHIFT      (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_13_LWR_BASE_RW_MAX        (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_INBOUND_13 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_13_UPPER_BASE_RW_MASK   (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_13_UPPER_BASE_RW_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_13_UPPER_BASE_RW_MAX    (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_INBOUND_13 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_13_LIMIT_ADDR_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_13_LIMIT_ADDR_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_13_LIMIT_ADDR_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_13_LIMIT_ADDR_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_13_LIMIT_ADDR_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_13_LIMIT_ADDR_RW_MAX         (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_INBOUND_13 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_13_LWR_TARGET_HW_MASK   (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_13_LWR_TARGET_HW_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_13_LWR_TARGET_HW_MAX    (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_13_LWR_TARGET_RW_MASK   (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_13_LWR_TARGET_RW_SHIFT  (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_13_LWR_TARGET_RW_MAX    (0x0000FFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_INBOUND_13 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_13_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_13_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_13_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_OUTBOUND_14 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_14_TYPE_MASK             (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_14_TYPE_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_14_TYPE_MAX              (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_14_TC_MASK               (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_14_TC_SHIFT              (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_14_TC_MAX                (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_14_TD_MASK               (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_14_TD_SHIFT              (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_14_TD_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_14_ATTR_MASK             (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_14_ATTR_SHIFT            (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_14_ATTR_MAX              (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_14_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_14_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_14_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_14_CTRL_1_FUNC_NUM_MASK  (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_14_CTRL_1_FUNC_NUM_SHIFT (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_14_CTRL_1_FUNC_NUM_MAX   (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_OUTBOUND_14 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_MSG_CODE_MASK         (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_MSG_CODE_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_MSG_CODE_MAX          (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_TAG_MASK              (0x0000FF00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_TAG_SHIFT             (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_TAG_MAX               (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_TAG_SUBSTITUTE_EN_MASK (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_TAG_SUBSTITUTE_EN_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_TAG_SUBSTITUTE_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_FUNC_BYPASS_MASK      (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_FUNC_BYPASS_SHIFT     (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_FUNC_BYPASS_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_SNP_MASK              (0x00100000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_SNP_SHIFT             (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_SNP_MAX               (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_INHIBIT_PAYLOAD_MASK  (0x00400000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_INHIBIT_PAYLOAD_SHIFT (0x00000016U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_INHIBIT_PAYLOAD_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_HEADER_SUBSTITUTE_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_HEADER_SUBSTITUTE_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_HEADER_SUBSTITUTE_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_CFG_SHIFT_MODE_MASK   (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_CFG_SHIFT_MODE_SHIFT  (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_CFG_SHIFT_MODE_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_INVERT_MODE_MASK      (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_INVERT_MODE_SHIFT     (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_INVERT_MODE_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_REGION_EN_MASK        (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_REGION_EN_SHIFT       (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_14_REGION_EN_MAX         (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_OUTBOUND_14 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_14_LWR_BASE_HW_MASK      (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_14_LWR_BASE_HW_SHIFT     (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_14_LWR_BASE_HW_MAX       (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_14_LWR_BASE_RW_MASK      (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_14_LWR_BASE_RW_SHIFT     (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_14_LWR_BASE_RW_MAX       (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_14 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_14_UPPER_BASE_RW_MASK  (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_14_UPPER_BASE_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_14_UPPER_BASE_RW_MAX   (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_OUTBOUND_14 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_14_LIMIT_ADDR_HW_MASK       (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_14_LIMIT_ADDR_HW_SHIFT      (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_14_LIMIT_ADDR_HW_MAX        (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_14_LIMIT_ADDR_RW_MASK       (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_14_LIMIT_ADDR_RW_SHIFT      (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_14_LIMIT_ADDR_RW_MAX        (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_14 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_14_LWR_TARGET_RW_OUTBOUND_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_14_LWR_TARGET_RW_OUTBOUND_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_14_LWR_TARGET_RW_OUTBOUND_MAX (0xFFFFFFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_14 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_14_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_14_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_14_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_INBOUND_14 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_14_TYPE_MASK              (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_14_TYPE_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_14_TYPE_MAX               (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_14_TC_MASK                (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_14_TC_SHIFT               (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_14_TC_MAX                 (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_14_TD_MASK                (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_14_TD_SHIFT               (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_14_TD_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_14_ATTR_MASK              (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_14_ATTR_SHIFT             (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_14_ATTR_MAX               (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_14_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_14_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_14_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_14_CTRL_1_FUNC_NUM_MASK   (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_14_CTRL_1_FUNC_NUM_SHIFT  (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_14_CTRL_1_FUNC_NUM_MAX    (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_INBOUND_14 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_MSG_CODE_MASK          (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_MSG_CODE_SHIFT         (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_MSG_CODE_MAX           (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_BAR_NUM_MASK           (0x00000700U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_BAR_NUM_SHIFT          (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_BAR_NUM_MAX            (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_MSG_TYPE_MATCH_MODE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_MSG_TYPE_MATCH_MODE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_MSG_TYPE_MATCH_MODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_TC_MATCH_EN_MASK       (0x00004000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_TC_MATCH_EN_SHIFT      (0x0000000EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_TC_MATCH_EN_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_TD_MATCH_EN_MASK       (0x00008000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_TD_MATCH_EN_SHIFT      (0x0000000FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_TD_MATCH_EN_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_ATTR_MATCH_EN_MASK     (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_ATTR_MATCH_EN_SHIFT    (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_ATTR_MATCH_EN_MAX      (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_FUNC_NUM_MATCH_EN_MASK (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_FUNC_NUM_MATCH_EN_SHIFT (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_FUNC_NUM_MATCH_EN_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_MSG_CODE_MATCH_EN_MASK (0x00200000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_MSG_CODE_MATCH_EN_SHIFT (0x00000015U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_MSG_CODE_MATCH_EN_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_SINGLE_ADDR_LOC_TRANS_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_SINGLE_ADDR_LOC_TRANS_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_SINGLE_ADDR_LOC_TRANS_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_RESPONSE_CODE_MASK     (0x03000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_RESPONSE_CODE_SHIFT    (0x00000018U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_RESPONSE_CODE_MAX      (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_FUZZY_TYPE_MATCH_CODE_MASK (0x08000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_FUZZY_TYPE_MATCH_CODE_SHIFT (0x0000001BU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_FUZZY_TYPE_MATCH_CODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_CFG_SHIFT_MODE_MASK    (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_CFG_SHIFT_MODE_SHIFT   (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_CFG_SHIFT_MODE_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_INVERT_MODE_MASK       (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_INVERT_MODE_SHIFT      (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_INVERT_MODE_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_MATCH_MODE_MASK        (0x40000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_MATCH_MODE_SHIFT       (0x0000001EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_MATCH_MODE_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_REGION_EN_MASK         (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_REGION_EN_SHIFT        (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_14_REGION_EN_MAX          (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_INBOUND_14 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_14_LWR_BASE_HW_MASK       (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_14_LWR_BASE_HW_SHIFT      (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_14_LWR_BASE_HW_MAX        (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_14_LWR_BASE_RW_MASK       (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_14_LWR_BASE_RW_SHIFT      (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_14_LWR_BASE_RW_MAX        (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_INBOUND_14 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_14_UPPER_BASE_RW_MASK   (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_14_UPPER_BASE_RW_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_14_UPPER_BASE_RW_MAX    (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_INBOUND_14 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_14_LIMIT_ADDR_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_14_LIMIT_ADDR_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_14_LIMIT_ADDR_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_14_LIMIT_ADDR_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_14_LIMIT_ADDR_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_14_LIMIT_ADDR_RW_MAX         (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_INBOUND_14 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_14_LWR_TARGET_HW_MASK   (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_14_LWR_TARGET_HW_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_14_LWR_TARGET_HW_MAX    (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_14_LWR_TARGET_RW_MASK   (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_14_LWR_TARGET_RW_SHIFT  (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_14_LWR_TARGET_RW_MAX    (0x0000FFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_INBOUND_14 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_14_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_14_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_14_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_OUTBOUND_15 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_15_TYPE_MASK             (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_15_TYPE_SHIFT            (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_15_TYPE_MAX              (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_15_TC_MASK               (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_15_TC_SHIFT              (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_15_TC_MAX                (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_15_TD_MASK               (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_15_TD_SHIFT              (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_15_TD_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_15_ATTR_MASK             (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_15_ATTR_SHIFT            (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_15_ATTR_MAX              (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_15_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_15_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_15_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_15_CTRL_1_FUNC_NUM_MASK  (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_15_CTRL_1_FUNC_NUM_SHIFT (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_15_CTRL_1_FUNC_NUM_MAX   (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_OUTBOUND_15 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_MSG_CODE_MASK         (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_MSG_CODE_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_MSG_CODE_MAX          (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_TAG_MASK              (0x0000FF00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_TAG_SHIFT             (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_TAG_MAX               (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_TAG_SUBSTITUTE_EN_MASK (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_TAG_SUBSTITUTE_EN_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_TAG_SUBSTITUTE_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_FUNC_BYPASS_MASK      (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_FUNC_BYPASS_SHIFT     (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_FUNC_BYPASS_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_SNP_MASK              (0x00100000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_SNP_SHIFT             (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_SNP_MAX               (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_INHIBIT_PAYLOAD_MASK  (0x00400000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_INHIBIT_PAYLOAD_SHIFT (0x00000016U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_INHIBIT_PAYLOAD_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_HEADER_SUBSTITUTE_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_HEADER_SUBSTITUTE_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_HEADER_SUBSTITUTE_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_CFG_SHIFT_MODE_MASK   (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_CFG_SHIFT_MODE_SHIFT  (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_CFG_SHIFT_MODE_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_INVERT_MODE_MASK      (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_INVERT_MODE_SHIFT     (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_INVERT_MODE_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_REGION_EN_MASK        (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_REGION_EN_SHIFT       (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_15_REGION_EN_MAX         (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_OUTBOUND_15 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_15_LWR_BASE_HW_MASK      (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_15_LWR_BASE_HW_SHIFT     (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_15_LWR_BASE_HW_MAX       (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_15_LWR_BASE_RW_MASK      (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_15_LWR_BASE_RW_SHIFT     (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_15_LWR_BASE_RW_MAX       (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_15 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_15_UPPER_BASE_RW_MASK  (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_15_UPPER_BASE_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_15_UPPER_BASE_RW_MAX   (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_OUTBOUND_15 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_15_LIMIT_ADDR_HW_MASK       (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_15_LIMIT_ADDR_HW_SHIFT      (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_15_LIMIT_ADDR_HW_MAX        (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_15_LIMIT_ADDR_RW_MASK       (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_15_LIMIT_ADDR_RW_SHIFT      (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_15_LIMIT_ADDR_RW_MAX        (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_15 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_15_LWR_TARGET_RW_OUTBOUND_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_15_LWR_TARGET_RW_OUTBOUND_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_15_LWR_TARGET_RW_OUTBOUND_MAX (0xFFFFFFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_15 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_15_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_15_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_15_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_INBOUND_15 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_15_TYPE_MASK              (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_15_TYPE_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_15_TYPE_MAX               (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_15_TC_MASK                (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_15_TC_SHIFT               (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_15_TC_MAX                 (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_15_TD_MASK                (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_15_TD_SHIFT               (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_15_TD_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_15_ATTR_MASK              (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_15_ATTR_SHIFT             (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_15_ATTR_MAX               (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_15_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_15_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_15_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_15_CTRL_1_FUNC_NUM_MASK   (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_15_CTRL_1_FUNC_NUM_SHIFT  (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_15_CTRL_1_FUNC_NUM_MAX    (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_INBOUND_15 */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_MSG_CODE_MASK          (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_MSG_CODE_SHIFT         (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_MSG_CODE_MAX           (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_BAR_NUM_MASK           (0x00000700U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_BAR_NUM_SHIFT          (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_BAR_NUM_MAX            (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_MSG_TYPE_MATCH_MODE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_MSG_TYPE_MATCH_MODE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_MSG_TYPE_MATCH_MODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_TC_MATCH_EN_MASK       (0x00004000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_TC_MATCH_EN_SHIFT      (0x0000000EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_TC_MATCH_EN_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_TD_MATCH_EN_MASK       (0x00008000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_TD_MATCH_EN_SHIFT      (0x0000000FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_TD_MATCH_EN_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_ATTR_MATCH_EN_MASK     (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_ATTR_MATCH_EN_SHIFT    (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_ATTR_MATCH_EN_MAX      (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_FUNC_NUM_MATCH_EN_MASK (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_FUNC_NUM_MATCH_EN_SHIFT (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_FUNC_NUM_MATCH_EN_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_MSG_CODE_MATCH_EN_MASK (0x00200000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_MSG_CODE_MATCH_EN_SHIFT (0x00000015U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_MSG_CODE_MATCH_EN_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_SINGLE_ADDR_LOC_TRANS_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_SINGLE_ADDR_LOC_TRANS_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_SINGLE_ADDR_LOC_TRANS_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_RESPONSE_CODE_MASK     (0x03000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_RESPONSE_CODE_SHIFT    (0x00000018U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_RESPONSE_CODE_MAX      (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_FUZZY_TYPE_MATCH_CODE_MASK (0x08000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_FUZZY_TYPE_MATCH_CODE_SHIFT (0x0000001BU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_FUZZY_TYPE_MATCH_CODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_CFG_SHIFT_MODE_MASK    (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_CFG_SHIFT_MODE_SHIFT   (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_CFG_SHIFT_MODE_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_INVERT_MODE_MASK       (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_INVERT_MODE_SHIFT      (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_INVERT_MODE_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_MATCH_MODE_MASK        (0x40000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_MATCH_MODE_SHIFT       (0x0000001EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_MATCH_MODE_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_REGION_EN_MASK         (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_REGION_EN_SHIFT        (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_15_REGION_EN_MAX          (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_INBOUND_15 */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_15_LWR_BASE_HW_MASK       (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_15_LWR_BASE_HW_SHIFT      (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_15_LWR_BASE_HW_MAX        (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_15_LWR_BASE_RW_MASK       (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_15_LWR_BASE_RW_SHIFT      (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_15_LWR_BASE_RW_MAX        (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_INBOUND_15 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_15_UPPER_BASE_RW_MASK   (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_15_UPPER_BASE_RW_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_15_UPPER_BASE_RW_MAX    (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_INBOUND_15 */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_15_LIMIT_ADDR_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_15_LIMIT_ADDR_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_15_LIMIT_ADDR_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_15_LIMIT_ADDR_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_15_LIMIT_ADDR_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_15_LIMIT_ADDR_RW_MAX         (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_INBOUND_15 */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_15_LWR_TARGET_HW_MASK   (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_15_LWR_TARGET_HW_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_15_LWR_TARGET_HW_MAX    (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_15_LWR_TARGET_RW_MASK   (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_15_LWR_TARGET_RW_SHIFT  (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_15_LWR_TARGET_RW_MAX    (0x0000FFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_INBOUND_15 */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_15_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_15_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_15_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

#else /* CSL_MODIFICATION */

/* IATU_REGION_CTRL_1_OFF_OUTBOUND */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_TYPE_MASK              (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_TYPE_SHIFT             (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_TYPE_MAX               (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_TC_MASK                (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_TC_SHIFT               (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_TC_MAX                 (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_TD_MASK                (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_TD_SHIFT               (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_TD_MAX                 (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_ATTR_MASK              (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_ATTR_SHIFT             (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_ATTR_MAX               (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_CTRL_1_FUNC_NUM_MASK   (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_CTRL_1_FUNC_NUM_SHIFT  (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_CTRL_1_FUNC_NUM_MAX    (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_OUTBOUND */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_MSG_CODE_MASK          (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_MSG_CODE_SHIFT         (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_MSG_CODE_MAX           (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_TAG_MASK               (0x0000FF00U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_TAG_SHIFT              (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_TAG_MAX                (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_TAG_SUBSTITUTE_EN_MASK (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_TAG_SUBSTITUTE_EN_SHIFT (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_TAG_SUBSTITUTE_EN_MAX  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_FUNC_BYPASS_MASK       (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_FUNC_BYPASS_SHIFT      (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_FUNC_BYPASS_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_SNP_MASK               (0x00100000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_SNP_SHIFT              (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_SNP_MAX                (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_INHIBIT_PAYLOAD_MASK   (0x00400000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_INHIBIT_PAYLOAD_SHIFT  (0x00000016U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_INHIBIT_PAYLOAD_MAX    (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_HEADER_SUBSTITUTE_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_HEADER_SUBSTITUTE_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_HEADER_SUBSTITUTE_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_CFG_SHIFT_MODE_MASK    (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_CFG_SHIFT_MODE_SHIFT   (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_CFG_SHIFT_MODE_MAX     (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_INVERT_MODE_MASK       (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_INVERT_MODE_SHIFT      (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_INVERT_MODE_MAX        (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_REGION_EN_MASK         (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_REGION_EN_SHIFT        (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_REGION_EN_MAX          (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_OUTBOUND */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_LWR_BASE_HW_MASK       (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_LWR_BASE_HW_SHIFT      (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_LWR_BASE_HW_MAX        (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_LWR_BASE_RW_MASK       (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_LWR_BASE_RW_SHIFT      (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_LWR_BASE_RW_MAX        (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_OUTBOUND */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_UPPER_BASE_RW_MASK   (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_UPPER_BASE_RW_SHIFT  (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_UPPER_BASE_RW_MAX    (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_OUTBOUND */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_LIMIT_ADDR_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_LIMIT_ADDR_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_LIMIT_ADDR_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_LIMIT_ADDR_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_LIMIT_ADDR_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_LIMIT_ADDR_RW_MAX         (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_OUTBOUND */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_LWR_TARGET_RW_OUTBOUND_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_LWR_TARGET_RW_OUTBOUND_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_LWR_TARGET_RW_OUTBOUND_MAX (0xFFFFFFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)

/* IATU_REGION_CTRL_1_OFF_INBOUND */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_TYPE_MASK               (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_TYPE_SHIFT              (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_TYPE_MAX                (0x0000001FU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_TC_MASK                 (0x000000E0U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_TC_SHIFT                (0x00000005U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_TC_MAX                  (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_TD_MASK                 (0x00000100U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_TD_SHIFT                (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_TD_MAX                  (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_ATTR_MASK               (0x00000600U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_ATTR_SHIFT              (0x00000009U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_ATTR_MAX                (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_INCREASE_REGION_SIZE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_INCREASE_REGION_SIZE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_INCREASE_REGION_SIZE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_CTRL_1_FUNC_NUM_MASK    (0x00700000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_CTRL_1_FUNC_NUM_SHIFT   (0x00000014U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_CTRL_1_FUNC_NUM_MAX     (0x00000007U)

/* IATU_REGION_CTRL_2_OFF_INBOUND */

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MSG_CODE_MASK           (0x000000FFU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MSG_CODE_SHIFT          (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MSG_CODE_MAX            (0x000000FFU)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_BAR_NUM_MASK            (0x00000700U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_BAR_NUM_SHIFT           (0x00000008U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_BAR_NUM_MAX             (0x00000007U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MSG_TYPE_MATCH_MODE_MASK (0x00002000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MSG_TYPE_MATCH_MODE_SHIFT (0x0000000DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MSG_TYPE_MATCH_MODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_TC_MATCH_EN_MASK        (0x00004000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_TC_MATCH_EN_SHIFT       (0x0000000EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_TC_MATCH_EN_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_TD_MATCH_EN_MASK        (0x00008000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_TD_MATCH_EN_SHIFT       (0x0000000FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_TD_MATCH_EN_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_ATTR_MATCH_EN_MASK      (0x00010000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_ATTR_MATCH_EN_SHIFT     (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_ATTR_MATCH_EN_MAX       (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_FUNC_NUM_MATCH_EN_MASK  (0x00080000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_FUNC_NUM_MATCH_EN_SHIFT (0x00000013U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_FUNC_NUM_MATCH_EN_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MSG_CODE_MATCH_EN_MASK  (0x00200000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MSG_CODE_MATCH_EN_SHIFT (0x00000015U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MSG_CODE_MATCH_EN_MAX   (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_SINGLE_ADDR_LOC_TRANS_EN_MASK (0x00800000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_SINGLE_ADDR_LOC_TRANS_EN_SHIFT (0x00000017U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_SINGLE_ADDR_LOC_TRANS_EN_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_RESPONSE_CODE_MASK      (0x03000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_RESPONSE_CODE_SHIFT     (0x00000018U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_RESPONSE_CODE_MAX       (0x00000003U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_FUZZY_TYPE_MATCH_CODE_MASK (0x08000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_FUZZY_TYPE_MATCH_CODE_SHIFT (0x0000001BU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_FUZZY_TYPE_MATCH_CODE_MAX (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_CFG_SHIFT_MODE_MASK     (0x10000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_CFG_SHIFT_MODE_SHIFT    (0x0000001CU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_CFG_SHIFT_MODE_MAX      (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_INVERT_MODE_MASK        (0x20000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_INVERT_MODE_SHIFT       (0x0000001DU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_INVERT_MODE_MAX         (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MATCH_MODE_MASK         (0x40000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MATCH_MODE_SHIFT        (0x0000001EU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MATCH_MODE_MAX          (0x00000001U)

#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_REGION_EN_MASK          (0x80000000U)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_REGION_EN_SHIFT         (0x0000001FU)
#define CSL_PCIE_RC_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_REGION_EN_MAX           (0x00000001U)

/* IATU_LWR_BASE_ADDR_OFF_INBOUND */

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_LWR_BASE_HW_MASK        (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_LWR_BASE_HW_SHIFT       (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_LWR_BASE_HW_MAX         (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_LWR_BASE_RW_MASK        (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_LWR_BASE_RW_SHIFT       (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_LWR_BASE_RW_MAX         (0x0000FFFFU)

/* IATU_UPPER_BASE_ADDR_OFF_INBOUND */

#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_UPPER_BASE_RW_MASK    (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_UPPER_BASE_RW_SHIFT   (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_UPPER_BASE_RW_MAX     (0xFFFFFFFFU)

/* IATU_LIMIT_ADDR_OFF_INBOUND */

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_LIMIT_ADDR_HW_MASK         (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_LIMIT_ADDR_HW_SHIFT        (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_LIMIT_ADDR_HW_MAX          (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_LIMIT_ADDR_RW_MASK         (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_LIMIT_ADDR_RW_SHIFT        (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_LIMIT_ADDR_RW_MAX          (0x0000FFFFU)

/* IATU_LWR_TARGET_ADDR_OFF_INBOUND */

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_LWR_TARGET_HW_MASK    (0x0000FFFFU)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_LWR_TARGET_HW_SHIFT   (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_LWR_TARGET_HW_MAX     (0x0000FFFFU)

#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_LWR_TARGET_RW_MASK    (0xFFFF0000U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_LWR_TARGET_RW_SHIFT   (0x00000010U)
#define CSL_PCIE_RC_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_LWR_TARGET_RW_MAX     (0x0000FFFFU)

/* IATU_UPPER_TARGET_ADDR_OFF_INBOUND */

#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_UPPER_TARGET_RW_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_UPPER_TARGET_RW_SHIFT (0x00000000U)
#define CSL_PCIE_RC_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_UPPER_TARGET_RW_MAX (0xFFFFFFFFU)
#endif /* CSL_MODIFICATION */


/**************************************************************************
* Hardware Region  : PCIE data region0
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PCIE_DATA_MEM[67108864];   /* PCIe data region0 */
} CSL_pcie_rc_dat0Regs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_PCIE_RC_DAT0_PCIE_DATA_MEM(PCIE_DATA_MEM)                             (0x00000000U+((PCIE_DATA_MEM)*0x4U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PCIE_DATA_MEM */

#define CSL_PCIE_RC_DAT0_PCIE_DATA_MEM_PCIE_DATA_MASK                             (0xFFFFFFFFU)
#define CSL_PCIE_RC_DAT0_PCIE_DATA_MEM_PCIE_DATA_SHIFT                            (0x00000000U)
#define CSL_PCIE_RC_DAT0_PCIE_DATA_MEM_PCIE_DATA_MAX                              (0xFFFFFFFFU)

/**************************************************************************
* Hardware Region  : PCIE data region1
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PCIE_DATA_MEM[67108864];   /* PCIe data region1 */
} CSL_pcie_rc_dat1Regs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_PCIE_RC_DAT1_PCIE_DATA_MEM(PCIE_DATA_MEM)                             (0x00000000U+((PCIE_DATA_MEM)*0x4U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PCIE_DATA_MEM */

#define CSL_PCIE_RC_DAT1_PCIE_DATA_MEM_PCIE_DATA_MASK                             (0xFFFFFFFFU)
#define CSL_PCIE_RC_DAT1_PCIE_DATA_MEM_PCIE_DATA_SHIFT                            (0x00000000U)
#define CSL_PCIE_RC_DAT1_PCIE_DATA_MEM_PCIE_DATA_MAX                              (0xFFFFFFFFU)

/**************************************************************************
* Hardware Region  : PCIE data region2-untranslated address
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PCIE_DATA_MEM[67108864];   /* PCIe data region2 */
} CSL_pcie_rc_dat2Regs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_PCIE_RC_DAT2_PCIE_DATA_MEM(PCIE_DATA_MEM)                             (0x00000000U+((PCIE_DATA_MEM)*0x4U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PCIE_DATA_MEM */

#define CSL_PCIE_RC_DAT2_PCIE_DATA_MEM_PCIE_DATA_MASK                             (0xFFFFFFFFU)
#define CSL_PCIE_RC_DAT2_PCIE_DATA_MEM_PCIE_DATA_SHIFT                            (0x00000000U)
#define CSL_PCIE_RC_DAT2_PCIE_DATA_MEM_PCIE_DATA_MAX                              (0xFFFFFFFFU)

#ifdef __cplusplus
}
#endif
#endif
