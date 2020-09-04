/********************************************************************
 * Copyright (C) 2022 Texas Instruments Incorporated.
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
 *
 *  Name        : cslr_pcie_rp.h
*/
#ifndef CSLR_PCIE_RP_H_
#define CSLR_PCIE_RP_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  : PCIE data region0
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PCIE_HP_DATA_MEM[1];   /* PCIe data region0 */
} CSL_pcie_rp_hp_dat0Regs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_PCIE_RP_HP_DAT0_PCIE_HP_DATA_MEM(PCIE_HP_DATA_MEM)                    (0x00000000U+((PCIE_HP_DATA_MEM)*0x4U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PCIE_HP_DATA_MEM */

#define CSL_PCIE_RP_HP_DAT0_PCIE_HP_DATA_MEM_PCIE_HP_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_HP_DAT0_PCIE_HP_DATA_MEM_PCIE_HP_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_HP_DAT0_PCIE_HP_DATA_MEM_PCIE_HP_DATA_MAX                     (0xFFFFFFFFU)

/**************************************************************************
* Hardware Region  : PCIE data region1
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PCIE_HP_DATA_MEM[1];   /* PCIe data region1 */
} CSL_pcie_rp_hp_dat1Regs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_PCIE_RP_HP_DAT1_PCIE_HP_DATA_MEM(PCIE_HP_DATA_MEM)                    (0x00000000U+((PCIE_HP_DATA_MEM)*0x4U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PCIE_HP_DATA_MEM */

#define CSL_PCIE_RP_HP_DAT1_PCIE_HP_DATA_MEM_PCIE_HP_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_HP_DAT1_PCIE_HP_DATA_MEM_PCIE_HP_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_HP_DAT1_PCIE_HP_DATA_MEM_PCIE_HP_DATA_MAX                     (0xFFFFFFFFU)

/**************************************************************************
* Hardware Region  : PCIE data region2-untranslated address
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PCIE_HP_DATA_MEM[1];   /* PCIe data region2 */
} CSL_pcie_rp_hp_dat2Regs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_PCIE_RP_HP_DAT2_PCIE_HP_DATA_MEM(PCIE_HP_DATA_MEM)                    (0x00000000U+((PCIE_HP_DATA_MEM)*0x4U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PCIE_HP_DATA_MEM */

#define CSL_PCIE_RP_HP_DAT2_PCIE_HP_DATA_MEM_PCIE_HP_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_HP_DAT2_PCIE_HP_DATA_MEM_PCIE_HP_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_HP_DAT2_PCIE_HP_DATA_MEM_PCIE_HP_DATA_MAX                     (0xFFFFFFFFU)

/**************************************************************************
* Hardware Region  : PCIE data region0
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PCIE_LP_DATA_MEM[1];   /* PCIe data region0 */
} CSL_pcie_rp_lp_dat0Regs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_PCIE_RP_LP_DAT0_PCIE_LP_DATA_MEM(PCIE_LP_DATA_MEM)                    (0x00000000U+((PCIE_LP_DATA_MEM)*0x4U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PCIE_LP_DATA_MEM */

#define CSL_PCIE_RP_LP_DAT0_PCIE_LP_DATA_MEM_PCIE_LP_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_LP_DAT0_PCIE_LP_DATA_MEM_PCIE_LP_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_LP_DAT0_PCIE_LP_DATA_MEM_PCIE_LP_DATA_MAX                     (0xFFFFFFFFU)

/**************************************************************************
* Hardware Region  : PCIE data region1
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PCIE_LP_DATA_MEM[1];   /* PCIe data region1 */
} CSL_pcie_rp_lp_dat1Regs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_PCIE_RP_LP_DAT1_PCIE_LP_DATA_MEM(PCIE_LP_DATA_MEM)                    (0x00000000U+((PCIE_LP_DATA_MEM)*0x4U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PCIE_LP_DATA_MEM */

#define CSL_PCIE_RP_LP_DAT1_PCIE_LP_DATA_MEM_PCIE_LP_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_LP_DAT1_PCIE_LP_DATA_MEM_PCIE_LP_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_LP_DAT1_PCIE_LP_DATA_MEM_PCIE_LP_DATA_MAX                     (0xFFFFFFFFU)

/**************************************************************************
* Hardware Region  : PCIE data region2-untranslated address
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PCIE_LP_DATA_MEM[1];   /* PCIe data region2 */
} CSL_pcie_rp_lp_dat2Regs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_PCIE_RP_LP_DAT2_PCIE_LP_DATA_MEM(PCIE_LP_DATA_MEM)                    (0x00000000U+((PCIE_LP_DATA_MEM)*0x4U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PCIE_LP_DATA_MEM */

#define CSL_PCIE_RP_LP_DAT2_PCIE_LP_DATA_MEM_PCIE_LP_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_LP_DAT2_PCIE_LP_DATA_MEM_PCIE_LP_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_LP_DAT2_PCIE_LP_DATA_MEM_PCIE_LP_DATA_MAX                     (0xFFFFFFFFU)

/**************************************************************************
* Hardware Region  : RC mode PCIE core registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t I_VENDOR_ID_DEVICE_ID;
    volatile uint32_t I_COMMAND_STATUS;
    volatile uint32_t I_REVISION_ID_CLASS_CODE;
    volatile uint32_t I_BIST_HEADER_LATENCY_CACHE_LINE;
    volatile uint32_t I_RC_BAR_0;
    volatile uint32_t I_RC_BAR_1;
    volatile uint32_t I_PCIE_BUS_NUMBERS;
    volatile uint32_t I_PCIE_IO_BASE_LIMIT;
    volatile uint32_t I_PCIE_MEM_BASE_LIMIT;
    volatile uint32_t I_PCIE_PREFETCH_BASE_LIMIT;
    volatile uint32_t I_PCIE_PREFETCH_BASE_UPPER;
    volatile uint32_t I_PCIE_PREFETCH_LIMIT_UPPER;
    volatile uint32_t I_PCIE_IO_BASE_LIMIT_UPPER;
    volatile uint32_t I_CAPABILITIES_POINTER;
    volatile uint32_t RSVD_0E;
    volatile uint32_t I_INTRPT_LINE_INTRPT_PIN;
    volatile uint8_t  Resv_128[64];
    volatile uint32_t I_PWR_MGMT_CAP;
    volatile uint32_t I_PWR_MGMT_CTRL_STAT_REP;
    volatile uint8_t  Resv_144[8];
    volatile uint32_t I_MSI_CTRL_REG;
    volatile uint32_t I_MSI_MSG_LOW_ADDR;
    volatile uint32_t I_MSI_MSG_HI_ADDR;
    volatile uint32_t I_MSI_MSG_DATA;
    volatile uint32_t I_MSI_MASK;
    volatile uint32_t I_MSI_PENDING_BITS;
    volatile uint8_t  Resv_176[8];
    volatile uint32_t I_MSIX_CTRL;
    volatile uint32_t I_MSIX_TBL_OFFSET;
    volatile uint32_t I_MSIX_PENDING_INTRPT;
    volatile uint8_t  Resv_192[4];
    volatile uint32_t I_PCIE_CAP_LIST;
    volatile uint32_t I_PCIE_CAP;
    volatile uint32_t I_PCIE_DEV_CTRL_STATUS;
    volatile uint32_t I_LINK_CAP;
    volatile uint32_t I_LINK_CTRL_STATUS;
    volatile uint32_t I_SLOT_CAPABILITY;
    volatile uint32_t I_SLOT_CTRL_STATUS;
    volatile uint32_t I_ROOT_CTRL_CAP;
    volatile uint32_t I_ROOT_STATUS;
    volatile uint32_t I_PCIE_CAP_2;
    volatile uint32_t I_PCIE_DEV_CTRL_STATUS_2;
    volatile uint32_t I_LINK_CAP_2;
    volatile uint32_t I_LINK_CTRL_STATUS_2;
    volatile uint8_t  Resv_256[12];
    volatile uint32_t I_AER_ENHNCD_CAP;
    volatile uint32_t I_UNCORR_ERR_STATUS;
    volatile uint32_t I_UNCORR_ERR_MASK;
    volatile uint32_t I_UNCORR_ERR_SEVERITY;
    volatile uint32_t I_CORR_ERR_STATUS;
    volatile uint32_t I_CORR_ERR_MASK;
    volatile uint32_t I_ADV_ERR_CAP_CTL;
    volatile uint32_t I_HDR_LOG_0;
    volatile uint32_t I_HDR_LOG_1;
    volatile uint32_t I_HDR_LOG_2;
    volatile uint32_t I_HDR_LOG_3;
    volatile uint32_t I_ROOT_ERR_CMD;
    volatile uint32_t I_ROOT_ERR_STAT;
    volatile uint32_t I_ERR_SRC_ID;
    volatile uint32_t I_TLP_PRE_LOG_0;
    volatile uint8_t  Resv_336[20];
    volatile uint32_t I_DEV_SER_NUM_CAP_HDR;
    volatile uint32_t I_DEV_SER_NUM_0;
    volatile uint32_t I_DEV_SER_NUM_1;
    volatile uint8_t  Resv_768[420];
    volatile uint32_t I_SEC_PCIE_CAP_HDR_REG;
    volatile uint32_t I_LINK_CONTROL3;
    volatile uint32_t I_LANE_ERROR_STATUS;
    volatile uint32_t I_LANE_EQUALIZATION_CONTROL_0;
} CSL_pcie_rp_coreRegs_RC_i_rc_pcie_base;


typedef struct {
    volatile uint32_t I_VC_ENH_CAP_HEADER_REG;
    volatile uint32_t I_PORT_VC_CAP_REG_1;
    volatile uint32_t I_PORT_VC_CAP_REG_2;
    volatile uint32_t I_PORT_VC_CTRL_STS_REG;
    volatile uint32_t I_VC_RES_CAP_REG_0;
    volatile uint32_t I_VC_RES_CTRL_REG_0;
    volatile uint32_t I_VC_RES_STS_REG_0;
    volatile uint32_t I_VC_RES_CAP_REG_1;
    volatile uint32_t I_VC_RES_CTRL_REG_1;
    volatile uint32_t I_VC_RES_STS_REG_1;
    volatile uint32_t I_VC_RES_CAP_REG_2;
    volatile uint32_t I_VC_RES_CTRL_REG_2;
    volatile uint32_t I_VC_RES_STS_REG_2;
    volatile uint32_t I_VC_RES_CAP_REG_3;
    volatile uint32_t I_VC_RES_CTRL_REG_3;
    volatile uint32_t I_VC_RES_STS_REG_3;
} CSL_pcie_rp_coreRegs_RC_i_VC_cap_struct;


typedef struct {
    volatile uint32_t I_L1_PM_EXT_CAP_HDR;
    volatile uint32_t I_L1_PM_CAP;
    volatile uint32_t I_L1_PM_CTRL_1;
    volatile uint32_t I_L1_PM_CTRL_2;
} CSL_pcie_rp_coreRegs_RC_i_regf_L1_PM_cap_struct;


typedef struct {
    volatile uint32_t I_DL_FEATURE_EXTENDED_CAPABILITY_HEADER_REG;
    volatile uint32_t I_DL_FEATURE_CAPABILITIES_REG;
    volatile uint32_t I_DL_FEATURE_STATUS_REG;
} CSL_pcie_rp_coreRegs_RC_i_regf_dl_feature_cap;


typedef struct {
    volatile uint32_t I_MARGINING_EXTENDED_CAPABILITY_HEADER_REG;
    volatile uint32_t I_MARGINING_PORT_CAPABILITIES_STATUS_REG;
    volatile uint32_t I_MARGINING_LANE_CONTROL_STATUS_REG0;
    volatile uint32_t I_MARGINING_LANE_CONTROL_STATUS_REG1;
} CSL_pcie_rp_coreRegs_RC_i_regf_margining_cap;


typedef struct {
    volatile uint32_t I_PL_16GTS_EXTENDED_CAPABILITY_HEADER_REG;
    volatile uint32_t I_PL_16GTS_CAPABILITIES_REG;
    volatile uint32_t I_PL_16GTS_CONTROL_REG;
    volatile uint32_t I_PL_16GTS_STATUS_REG;
    volatile uint32_t I_PL_16GTS_LOCAL_DATA_PARITY_MISMATCH_STATUS_REG;
    volatile uint32_t I_PL_16GTS_FIRST_RETIMER_DATA_PARITY_MISMATCH_STATUS_REG;
    volatile uint32_t I_PL_16GTS_SECOND_RETIMER_DATA_PARITY_MISMATCH_STATUS_REG;
    volatile uint32_t I_PL_16GTS_RESERVED_REG;
    volatile uint32_t I_PL_16GTS_LANE_EQUALIZATION_CONTROL_REG0;
} CSL_pcie_rp_coreRegs_RC_i_regf_pl_16gts_cap;


typedef struct {
    volatile uint32_t I_PTM_EXTENDED_CAPABILITY_HEADER_REG;
    volatile uint32_t I_PTM_CAPABILITIES_REG;
    volatile uint32_t I_PTM_CONTROL_REG;
} CSL_pcie_rp_coreRegs_RC_i_regf_ptm_cap;


#ifdef CSL_MODIFICATION
typedef struct {
    volatile uint32_t I_PL_CONFIG_0_REG;
    volatile uint32_t I_PL_CONFIG_1_REG;
    volatile uint32_t I_DLL_TMR_CONFIG_REG;
    volatile uint32_t I_RCV_CRED_LIM_0_REG;
    volatile uint32_t I_RCV_CRED_LIM_1_REG;
    volatile uint32_t I_TRANSM_CRED_LIM_0_REG;
    volatile uint32_t I_TRANSM_CRED_LIM_1_REG;
    volatile uint32_t I_TRANSM_CRED_UPDATE_INT_CONFIG_0_REG;
    volatile uint32_t I_TRANSM_CRED_UPDATE_INT_CONFIG_1_REG;
    volatile uint32_t I_L0S_TIMEOUT_LIMIT_REG;
    volatile uint32_t I_TRANSMIT_TLP_COUNT_REG;
    volatile uint32_t I_TRANSMIT_TLP_PAYLOAD_DWORD_COUNT_REG;
    volatile uint32_t I_RECEIVE_TLP_COUNT_REG;
    volatile uint32_t I_RECEIVE_TLP_PAYLOAD_DWORD_COUNT_REG;
    volatile uint32_t I_COMPLN_TMOUT_LIM_0_REG;
    volatile uint32_t I_COMPLN_TMOUT_LIM_1_REG;
    volatile uint32_t I_L1_ST_REENTRY_DELAY_REG;
    volatile uint32_t I_VENDOR_ID_REG;
    volatile uint32_t I_ASPM_L1_ENTRY_TMOUT_DELAY_REG;
    volatile uint32_t I_PME_TURNOFF_ACK_DELAY_REG;
    volatile uint32_t I_LINKWIDTH_CONTROL_REG;
    volatile uint8_t  Resv_112[28];
    volatile uint32_t I_MULTI_VC_CONROL_REG;
    volatile uint32_t I_SRIS_CONTROL_REG;
    volatile uint8_t  Resv_128[8];
    volatile uint32_t I_RCV_CRED_LIM_0_REG_VC1;
    volatile uint32_t I_RCV_CRED_LIM_1_REG_VC1;
    volatile uint32_t I_TRANSM_CRED_LIM_0_REG_VC1;
    volatile uint32_t I_TRANSM_CRED_LIM_1_REG_VC1;
    volatile uint32_t I_RCV_CRED_LIM_0_REG_VC2;
    volatile uint32_t I_RCV_CRED_LIM_1_REG_VC2;
    volatile uint32_t I_TRANSM_CRED_LIM_0_REG_VC2;
    volatile uint32_t I_TRANSM_CRED_LIM_1_REG_VC2;
    volatile uint32_t I_RCV_CRED_LIM_0_REG_VC3;
    volatile uint32_t I_RCV_CRED_LIM_1_REG_VC3;
    volatile uint32_t I_TRANSM_CRED_LIM_0_REG_VC3;
    volatile uint32_t I_TRANSM_CRED_LIM_1_REG_VC3;
    volatile uint8_t  Resv_240[64];
    volatile uint32_t I_FC_INIT_DELAY_REG;
    volatile uint8_t  Resv_256[12];
    volatile uint32_t I_SHDW_HDR_LOG_0_REG;
    volatile uint32_t I_SHDW_HDR_LOG_1_REG;
    volatile uint32_t I_SHDW_HDR_LOG_2_REG;
    volatile uint32_t I_SHDW_HDR_LOG_3_REG;
    volatile uint32_t I_SHDW_FUNC_NUM_REG;
    volatile uint32_t I_SHDW_UR_ERR_REG;
    volatile uint8_t  Resv_320[40];
    volatile uint32_t I_PM_CLK_FREQUENCY_REG;
    volatile uint32_t I_DEBUG_DLLP_COUNT_GEN1_REG;
    volatile uint32_t I_DEBUG_DLLP_COUNT_GEN2_REG;
    volatile uint32_t I_DEBUG_DLLP_COUNT_GEN3_REG;
    volatile uint32_t I_DEBUG_DLLP_COUNT_GEN4_REG;
    volatile uint8_t  Resv_344[4];
    volatile uint32_t I_VENDOR_DEFINED_MESSAGE_TAG_REG;
    volatile uint8_t  Resv_512[164];
    volatile uint32_t I_NEGOTIATED_LANE_MAP_REG;
    volatile uint32_t I_RECEIVE_FTS_COUNT_REG;
    volatile uint32_t I_DEBUG_MUX_CONTROL_REG;
    volatile uint32_t I_LOCAL_ERROR_STATUS_REGISTER;
    volatile uint32_t I_LOCAL_INTRPT_MASK_REG;
    volatile uint32_t I_LCRC_ERR_COUNT_REG;
    volatile uint32_t I_ECC_CORR_ERR_COUNT_REG;
    volatile uint32_t I_LTR_SNOOP_LAT_REG;
    volatile uint32_t I_LTR_MSG_GEN_CTL_REG;
    volatile uint32_t I_PME_SERVICE_TIMEOUT_DELAY_REG;
    volatile uint32_t I_ROOT_PORT_REQUESTOR_ID_REG;
    volatile uint32_t I_EP_BUS_DEVICE_NUMBER_REG;
    volatile uint8_t  Resv_564[4];
    volatile uint32_t I_DEBUG_MUX_CONTROL_2_REG;
    volatile uint32_t I_PHY_STATUS_1_REG;
    volatile uint8_t  Resv_576[4];
    volatile uint32_t I_PF_0_BAR_CONFIG_0_REG;
    volatile uint32_t I_PF_0_BAR_CONFIG_1_REG;
    volatile uint32_t I_PF_1_BAR_CONFIG_0_REG;
    volatile uint32_t I_PF_1_BAR_CONFIG_1_REG;
    volatile uint32_t I_PF_2_BAR_CONFIG_0_REG;
    volatile uint32_t I_PF_2_BAR_CONFIG_1_REG;
    volatile uint32_t I_PF_3_BAR_CONFIG_0_REG;
    volatile uint32_t I_PF_3_BAR_CONFIG_1_REG;
    volatile uint32_t I_PF_4_BAR_CONFIG_0_REG;
    volatile uint32_t I_PF_4_BAR_CONFIG_1_REG;
    volatile uint32_t I_PF_5_BAR_CONFIG_0_REG;
    volatile uint32_t I_PF_5_BAR_CONFIG_1_REG;
    volatile uint8_t  Resv_640[16];
    volatile uint32_t I_PF_0_VF_BAR_CONFIG_0_REG;
    volatile uint32_t I_PF_0_VF_BAR_CONFIG_1_REG;
    volatile uint32_t I_PF_1_VF_BAR_CONFIG_0_REG;
    volatile uint32_t I_PF_1_VF_BAR_CONFIG_1_REG;
    volatile uint32_t I_PF_2_VF_BAR_CONFIG_0_REG;
    volatile uint32_t I_PF_2_VF_BAR_CONFIG_1_REG;
    volatile uint32_t I_PF_3_VF_BAR_CONFIG_0_REG;
    volatile uint32_t I_PF_3_VF_BAR_CONFIG_1_REG;
    volatile uint32_t I_PF_4_VF_BAR_CONFIG_0_REG;
    volatile uint32_t I_PF_4_VF_BAR_CONFIG_1_REG;
    volatile uint32_t I_PF_5_VF_BAR_CONFIG_0_REG;
    volatile uint32_t I_PF_5_VF_BAR_CONFIG_1_REG;
    volatile uint8_t  Resv_704[16];
    volatile uint32_t I_PF_CONFIG_REG;
    volatile uint8_t  Resv_768[60];
    volatile uint32_t I_RC_BAR_CONFIG_REG;
    volatile uint8_t  Resv_864[92];
    volatile uint32_t I_GEN3_DEFAULT_PRESET_REG;
    volatile uint32_t I_GEN3_GEN4_LINK_EQ_TIMEOUT_2MS_REG;
    volatile uint32_t I_PIPE_FIFO_LATENCY_CTRL_REG;
    volatile uint8_t  Resv_884[8];
    volatile uint32_t I_GEN4_DEFAULT_PRESET_REG;
    volatile uint32_t I_PHY_CONFIG_REG3;
    volatile uint32_t I_GEN3_GEN4_LINK_EQ_CTRL_REG;
    volatile uint32_t I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE0;
    volatile uint32_t I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE1;
    volatile uint8_t  Resv_960[56];
    volatile uint32_t I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE0;
    volatile uint32_t I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE1;
    volatile uint8_t  Resv_3200[2232];
    volatile uint32_t I_ECC_CORR_ERR_COUNT_REG_AXI;
    volatile uint8_t  Resv_3208[4];
    volatile uint32_t LOW_POWER_DEBUG_AND_CONTROL0;
    volatile uint32_t LOW_POWER_DEBUG_AND_CONTROL1;
    volatile uint32_t LOW_POWER_DEBUG_AND_CONTROL2;
    volatile uint32_t TL_INTERNAL_CONTROL;
    volatile uint32_t I_DTI_ATS_STATUS;
    volatile uint32_t I_DTI_ATS_CTRL;
    volatile uint8_t  Resv_3264[32];
    volatile uint32_t I_SCALED_FLOW_CONTROL_MGMT_VC_SELECT_REG;
    volatile uint32_t I_SCALED_FLOW_CONTROL_MGMT_REG;
    volatile uint8_t  Resv_3280[8];
    volatile uint32_t I_MARGINING_PARAMETERS_1_REG;
    volatile uint32_t I_MARGINING_PARAMETERS_2_REG;
    volatile uint32_t I_MARGINING_LOCAL_CONTROL_REG;
    volatile uint32_t I_MARGINING_ERROR_STATUS1_REG;
    volatile uint32_t I_MARGINING_ERROR_STATUS2_REG;
    volatile uint8_t  Resv_3328[28];
    volatile uint32_t I_LOCAL_ERROR_STATUS_2_REGISTER;
    volatile uint32_t I_LOCAL_INTRPT_MASK_2_REG;
    volatile uint8_t  Resv_3344[8];
    volatile uint32_t MSI_MASK_CLEARED_STATUS_1;
    volatile uint32_t MSI_MASK_SET_STATUS_1;
    volatile uint32_t MSIX_FUNCTION_MASK_CLEARED_STATUS_1;
    volatile uint32_t MSIX_FUNCTION_MASK_SET_STATUS_1;
    volatile uint8_t  Resv_3488[128];
    volatile uint32_t I_LD_CTRL;
    volatile uint32_t RX_ELEC_IDLE_FILTER_CONTROL;
    volatile uint32_t I_PTM_LOCAL_CONTROL_REG;
    volatile uint32_t I_PTM_LOCAL_STATUS_REG;
    volatile uint32_t I_PTM_LATENCY_PARAMETERS_INDEX_REG;
    volatile uint32_t I_PTM_LATENCY_PARAMETERS_REG;
    volatile uint32_t I_PTM_CONTEXT_1_REG;
    volatile uint32_t I_PTM_CONTEXT_2_REG;
    volatile uint32_t I_PTM_CONTEXT_3_REG;
    volatile uint32_t I_PTM_CONTEXT_4_REG;
    volatile uint32_t I_PTM_CONTEXT_5_REG;
    volatile uint32_t I_PTM_CONTEXT_6_REG;
    volatile uint32_t I_PTM_CONTEXT_7_REG;
    volatile uint32_t I_PTM_CONTEXT_8_REG;
    volatile uint32_t I_PTM_CONTEXT_9_REG;
    volatile uint32_t I_PTM_CONTEXT_10_REG;
    volatile uint32_t I_PTM_CONTEXT_11_REG;
    volatile uint8_t  Resv_3564[8];
    volatile uint32_t I_ASF_INTRPT_STATUS;
    volatile uint32_t I_ASF_INTRPT_RAW_STATUS;
    volatile uint32_t I_ASF_INTRPT_MASK_REG;
    volatile uint32_t I_ASF_INTRPT_TEST;
    volatile uint32_t I_ASF_INTRPT_FATAL_NONFATAL_SEL;
    volatile uint32_t I_ASF_SRAM_CORR_FAULT_STATUS;
    volatile uint32_t I_ASF_SRAM_UNCORR_FAULT_STATUS;
    volatile uint32_t I_ASF_SRAM_FAULT_STATSTICS;
    volatile uint32_t I_ASF_TRANS_TO_CTRL;
    volatile uint32_t I_ASF_TRANS_TO_FAULT_MASK;
    volatile uint32_t I_ASF_TRANS_TO_FAULT_STATUS;
    volatile uint32_t I_ASF_PROTOCOL_FAULT_MASK;
    volatile uint32_t I_ASF_PROTOCOL_FAULT_STATUS_REG;
    volatile uint32_t DUAL_TL_CTRL;
    volatile uint8_t  Resv_3648[28];
    volatile uint32_t I_ASF_MAGIC_NUM_CTRLLER_VER_REG;
} CSL_pcie_rp_coreRegs_LM_i_regf_lm_pcie_base;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_0;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_1;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_2;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_3;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_4;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_5;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_6;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_7;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_8;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_9;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_10;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_11;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_12;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_13;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_14;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_15;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_16;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_17;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_18;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_19;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_20;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_21;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_22;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_23;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_24;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_25;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_26;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_27;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_28;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_29;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_30;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob_31;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_0;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_1;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_2;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_3;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_4;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_5;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_6;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_7;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_8;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_9;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_10;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_11;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_12;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_13;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_14;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_15;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_16;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_17;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_18;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_19;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_20;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_21;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_22;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_23;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_24;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_25;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_26;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_27;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_28;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_29;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_30;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_31;

typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ib_0;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ib_1;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ib_2;
#else /* CSL_MODIFICATION */
typedef struct {
    volatile uint32_t I_PF_BAR_CONFIG_0_REG;
    volatile uint32_t I_PF_BAR_CONFIG_1_REG;
} CSL_i_pf_bar_config_reg;


typedef struct {
    volatile uint32_t I_PL_CONFIG_0_REG;
    volatile uint32_t I_PL_CONFIG_1_REG;
    volatile uint32_t I_DLL_TMR_CONFIG_REG;
    volatile uint32_t I_RCV_CRED_LIM_0_REG;
    volatile uint32_t I_RCV_CRED_LIM_1_REG;
    volatile uint32_t I_TRANSM_CRED_LIM_0_REG;
    volatile uint32_t I_TRANSM_CRED_LIM_1_REG;
    volatile uint32_t I_TRANSM_CRED_UPDATE_INT_CONFIG_0_REG;
    volatile uint32_t I_TRANSM_CRED_UPDATE_INT_CONFIG_1_REG;
    volatile uint32_t I_L0S_TIMEOUT_LIMIT_REG;
    volatile uint32_t I_TRANSMIT_TLP_COUNT_REG;
    volatile uint32_t I_TRANSMIT_TLP_PAYLOAD_DWORD_COUNT_REG;
    volatile uint32_t I_RECEIVE_TLP_COUNT_REG;
    volatile uint32_t I_RECEIVE_TLP_PAYLOAD_DWORD_COUNT_REG;
    volatile uint32_t I_COMPLN_TMOUT_LIM_0_REG;
    volatile uint32_t I_COMPLN_TMOUT_LIM_1_REG;
    volatile uint32_t I_L1_ST_REENTRY_DELAY_REG;
    volatile uint32_t I_VENDOR_ID_REG;
    volatile uint32_t I_ASPM_L1_ENTRY_TMOUT_DELAY_REG;
    volatile uint32_t I_PME_TURNOFF_ACK_DELAY_REG;
    volatile uint32_t I_LINKWIDTH_CONTROL_REG;
    volatile uint8_t  Resv_112[28];
    volatile uint32_t I_MULTI_VC_CONROL_REG;
    volatile uint32_t I_SRIS_CONTROL_REG;
    volatile uint8_t  Resv_128[8];
    volatile uint32_t I_RCV_CRED_LIM_0_REG_VC1;
    volatile uint32_t I_RCV_CRED_LIM_1_REG_VC1;
    volatile uint32_t I_TRANSM_CRED_LIM_0_REG_VC1;
    volatile uint32_t I_TRANSM_CRED_LIM_1_REG_VC1;
    volatile uint32_t I_RCV_CRED_LIM_0_REG_VC2;
    volatile uint32_t I_RCV_CRED_LIM_1_REG_VC2;
    volatile uint32_t I_TRANSM_CRED_LIM_0_REG_VC2;
    volatile uint32_t I_TRANSM_CRED_LIM_1_REG_VC2;
    volatile uint32_t I_RCV_CRED_LIM_0_REG_VC3;
    volatile uint32_t I_RCV_CRED_LIM_1_REG_VC3;
    volatile uint32_t I_TRANSM_CRED_LIM_0_REG_VC3;
    volatile uint32_t I_TRANSM_CRED_LIM_1_REG_VC3;
    volatile uint8_t  Resv_240[64];
    volatile uint32_t I_FC_INIT_DELAY_REG;
    volatile uint8_t  Resv_256[12];
    volatile uint32_t I_SHDW_HDR_LOG_0_REG;
    volatile uint32_t I_SHDW_HDR_LOG_1_REG;
    volatile uint32_t I_SHDW_HDR_LOG_2_REG;
    volatile uint32_t I_SHDW_HDR_LOG_3_REG;
    volatile uint32_t I_SHDW_FUNC_NUM_REG;
    volatile uint32_t I_SHDW_UR_ERR_REG;
    volatile uint8_t  Resv_320[40];
    volatile uint32_t I_PM_CLK_FREQUENCY_REG;
    volatile uint32_t I_DEBUG_DLLP_COUNT_GEN1_REG;
    volatile uint32_t I_DEBUG_DLLP_COUNT_GEN2_REG;
    volatile uint32_t I_DEBUG_DLLP_COUNT_GEN3_REG;
    volatile uint32_t I_DEBUG_DLLP_COUNT_GEN4_REG;
    volatile uint8_t  Resv_344[4];
    volatile uint32_t I_VENDOR_DEFINED_MESSAGE_TAG_REG;
    volatile uint8_t  Resv_512[164];
    volatile uint32_t I_NEGOTIATED_LANE_MAP_REG;
    volatile uint32_t I_RECEIVE_FTS_COUNT_REG;
    volatile uint32_t I_DEBUG_MUX_CONTROL_REG;
    volatile uint32_t I_LOCAL_ERROR_STATUS_REGISTER;
    volatile uint32_t I_LOCAL_INTRPT_MASK_REG;
    volatile uint32_t I_LCRC_ERR_COUNT_REG;
    volatile uint32_t I_ECC_CORR_ERR_COUNT_REG;
    volatile uint32_t I_LTR_SNOOP_LAT_REG;
    volatile uint32_t I_LTR_MSG_GEN_CTL_REG;
    volatile uint32_t I_PME_SERVICE_TIMEOUT_DELAY_REG;
    volatile uint32_t I_ROOT_PORT_REQUESTOR_ID_REG;
    volatile uint32_t I_EP_BUS_DEVICE_NUMBER_REG;
    volatile uint8_t  Resv_564[4];
    volatile uint32_t I_DEBUG_MUX_CONTROL_2_REG;
    volatile uint32_t I_PHY_STATUS_1_REG;
    volatile uint8_t  Resv_576[4];
	CSL_i_pf_bar_config_reg I_PF_BAR_CONFIG_REG[6];
    volatile uint8_t  Resv_640[16];
	CSL_i_pf_bar_config_reg I_PF_VF_BAR_CONFIG_REG[6];
    volatile uint8_t  Resv_704[16];
    volatile uint32_t I_PF_CONFIG_REG;
    volatile uint8_t  Resv_768[60];
    volatile uint32_t I_RC_BAR_CONFIG_REG;
    volatile uint8_t  Resv_864[92];
    volatile uint32_t I_GEN3_DEFAULT_PRESET_REG;
    volatile uint32_t I_GEN3_GEN4_LINK_EQ_TIMEOUT_2MS_REG;
    volatile uint32_t I_PIPE_FIFO_LATENCY_CTRL_REG;
    volatile uint8_t  Resv_884[8];
    volatile uint32_t I_GEN4_DEFAULT_PRESET_REG;
    volatile uint32_t I_PHY_CONFIG_REG3;
    volatile uint32_t I_GEN3_GEN4_LINK_EQ_CTRL_REG;
    volatile uint32_t I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE0;
    volatile uint32_t I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE1;
    volatile uint8_t  Resv_960[56];
    volatile uint32_t I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE0;
    volatile uint32_t I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE1;
    volatile uint8_t  Resv_3200[2232];
    volatile uint32_t I_ECC_CORR_ERR_COUNT_REG_AXI;
    volatile uint8_t  Resv_3208[4];
    volatile uint32_t LOW_POWER_DEBUG_AND_CONTROL0;
    volatile uint32_t LOW_POWER_DEBUG_AND_CONTROL1;
    volatile uint32_t LOW_POWER_DEBUG_AND_CONTROL2;
    volatile uint32_t TL_INTERNAL_CONTROL;
    volatile uint32_t I_DTI_ATS_STATUS;
    volatile uint32_t I_DTI_ATS_CTRL;
    volatile uint8_t  Resv_3264[32];
    volatile uint32_t I_SCALED_FLOW_CONTROL_MGMT_VC_SELECT_REG;
    volatile uint32_t I_SCALED_FLOW_CONTROL_MGMT_REG;
    volatile uint8_t  Resv_3280[8];
    volatile uint32_t I_MARGINING_PARAMETERS_1_REG;
    volatile uint32_t I_MARGINING_PARAMETERS_2_REG;
    volatile uint32_t I_MARGINING_LOCAL_CONTROL_REG;
    volatile uint32_t I_MARGINING_ERROR_STATUS1_REG;
    volatile uint32_t I_MARGINING_ERROR_STATUS2_REG;
    volatile uint8_t  Resv_3328[28];
    volatile uint32_t I_LOCAL_ERROR_STATUS_2_REGISTER;
    volatile uint32_t I_LOCAL_INTRPT_MASK_2_REG;
    volatile uint8_t  Resv_3344[8];
    volatile uint32_t MSI_MASK_CLEARED_STATUS_1;
    volatile uint32_t MSI_MASK_SET_STATUS_1;
    volatile uint32_t MSIX_FUNCTION_MASK_CLEARED_STATUS_1;
    volatile uint32_t MSIX_FUNCTION_MASK_SET_STATUS_1;
    volatile uint8_t  Resv_3488[128];
    volatile uint32_t I_LD_CTRL;
    volatile uint32_t RX_ELEC_IDLE_FILTER_CONTROL;
    volatile uint32_t I_PTM_LOCAL_CONTROL_REG;
    volatile uint32_t I_PTM_LOCAL_STATUS_REG;
    volatile uint32_t I_PTM_LATENCY_PARAMETERS_INDEX_REG;
    volatile uint32_t I_PTM_LATENCY_PARAMETERS_REG;
    volatile uint32_t I_PTM_CONTEXT_REG[11];
    volatile uint8_t  Resv_3564[8];
    volatile uint32_t I_ASF_INTRPT_STATUS;
    volatile uint32_t I_ASF_INTRPT_RAW_STATUS;
    volatile uint32_t I_ASF_INTRPT_MASK_REG;
    volatile uint32_t I_ASF_INTRPT_TEST;
    volatile uint32_t I_ASF_INTRPT_FATAL_NONFATAL_SEL;
    volatile uint32_t I_ASF_SRAM_CORR_FAULT_STATUS;
    volatile uint32_t I_ASF_SRAM_UNCORR_FAULT_STATUS;
    volatile uint32_t I_ASF_SRAM_FAULT_STATSTICS;
    volatile uint32_t I_ASF_TRANS_TO_CTRL;
    volatile uint32_t I_ASF_TRANS_TO_FAULT_MASK;
    volatile uint32_t I_ASF_TRANS_TO_FAULT_STATUS;
    volatile uint32_t I_ASF_PROTOCOL_FAULT_MASK;
    volatile uint32_t I_ASF_PROTOCOL_FAULT_STATUS_REG;
    volatile uint32_t DUAL_TL_CTRL;
    volatile uint8_t  Resv_3648[28];
    volatile uint32_t I_ASF_MAGIC_NUM_CTRLLER_VER_REG;
} CSL_pcie_rp_coreRegs_LM_i_regf_lm_pcie_base;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
    volatile uint32_t DESC0;
    volatile uint32_t DESC1;
    volatile uint8_t  Resv_20[4];
    volatile uint32_t DESC3;
    volatile uint32_t AXI_ADDR0;
    volatile uint32_t AXI_ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ob;

typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_wrapper_ib;
#endif /* CSL_MODIFICATION */

typedef struct {
    volatile uint32_t C0;
} CSL_pcie_rp_coreRegs_atu_credit_threshold;


typedef struct {
    volatile uint32_t L0;
} CSL_pcie_rp_coreRegs_atu_link_down_indicator_bit;

#ifdef CSL_MODIFICATION
typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func0_wrapper_ib_ep_0;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func0_wrapper_ib_ep_1;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func0_wrapper_ib_ep_2;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func0_wrapper_ib_ep_3;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func0_wrapper_ib_ep_4;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func0_wrapper_ib_ep_5;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func0_wrapper_ib_ep_6;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func0_wrapper_ib_ep_7;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func1_wrapper_ib_ep_0;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func1_wrapper_ib_ep_1;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func1_wrapper_ib_ep_2;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func1_wrapper_ib_ep_3;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func1_wrapper_ib_ep_4;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func1_wrapper_ib_ep_5;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func1_wrapper_ib_ep_6;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func1_wrapper_ib_ep_7;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func2_wrapper_ib_ep_0;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func2_wrapper_ib_ep_1;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func2_wrapper_ib_ep_2;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func2_wrapper_ib_ep_3;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func2_wrapper_ib_ep_4;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func2_wrapper_ib_ep_5;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func2_wrapper_ib_ep_6;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func2_wrapper_ib_ep_7;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func3_wrapper_ib_ep_0;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func3_wrapper_ib_ep_1;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func3_wrapper_ib_ep_2;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func3_wrapper_ib_ep_3;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func3_wrapper_ib_ep_4;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func3_wrapper_ib_ep_5;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func3_wrapper_ib_ep_6;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func3_wrapper_ib_ep_7;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func4_wrapper_ib_ep_0;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func4_wrapper_ib_ep_1;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func4_wrapper_ib_ep_2;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func4_wrapper_ib_ep_3;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func4_wrapper_ib_ep_4;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func4_wrapper_ib_ep_5;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func4_wrapper_ib_ep_6;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func4_wrapper_ib_ep_7;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func5_wrapper_ib_ep_0;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func5_wrapper_ib_ep_1;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func5_wrapper_ib_ep_2;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func5_wrapper_ib_ep_3;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func5_wrapper_ib_ep_4;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func5_wrapper_ib_ep_5;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func5_wrapper_ib_ep_6;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func5_wrapper_ib_ep_7;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func6_wrapper_ib_ep_0;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func6_wrapper_ib_ep_1;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func6_wrapper_ib_ep_2;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func6_wrapper_ib_ep_3;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func6_wrapper_ib_ep_4;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func6_wrapper_ib_ep_5;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func6_wrapper_ib_ep_6;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func6_wrapper_ib_ep_7;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func7_wrapper_ib_ep_0;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func7_wrapper_ib_ep_1;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func7_wrapper_ib_ep_2;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func7_wrapper_ib_ep_3;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func7_wrapper_ib_ep_4;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func7_wrapper_ib_ep_5;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func7_wrapper_ib_ep_6;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func7_wrapper_ib_ep_7;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func8_wrapper_ib_ep_0;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func8_wrapper_ib_ep_1;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func8_wrapper_ib_ep_2;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func8_wrapper_ib_ep_3;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func8_wrapper_ib_ep_4;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func8_wrapper_ib_ep_5;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func8_wrapper_ib_ep_6;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func8_wrapper_ib_ep_7;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func9_wrapper_ib_ep_0;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func9_wrapper_ib_ep_1;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func9_wrapper_ib_ep_2;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func9_wrapper_ib_ep_3;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func9_wrapper_ib_ep_4;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func9_wrapper_ib_ep_5;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func9_wrapper_ib_ep_6;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func9_wrapper_ib_ep_7;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func10_wrapper_ib_ep_0;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func10_wrapper_ib_ep_1;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func10_wrapper_ib_ep_2;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func10_wrapper_ib_ep_3;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func10_wrapper_ib_ep_4;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func10_wrapper_ib_ep_5;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func10_wrapper_ib_ep_6;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func10_wrapper_ib_ep_7;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func11_wrapper_ib_ep_0;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func11_wrapper_ib_ep_1;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func11_wrapper_ib_ep_2;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func11_wrapper_ib_ep_3;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func11_wrapper_ib_ep_4;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func11_wrapper_ib_ep_5;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func11_wrapper_ib_ep_6;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func11_wrapper_ib_ep_7;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func12_wrapper_ib_ep_0;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func12_wrapper_ib_ep_1;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func12_wrapper_ib_ep_2;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func12_wrapper_ib_ep_3;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func12_wrapper_ib_ep_4;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func12_wrapper_ib_ep_5;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func12_wrapper_ib_ep_6;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func12_wrapper_ib_ep_7;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func13_wrapper_ib_ep_0;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func13_wrapper_ib_ep_1;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func13_wrapper_ib_ep_2;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func13_wrapper_ib_ep_3;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func13_wrapper_ib_ep_4;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func13_wrapper_ib_ep_5;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func13_wrapper_ib_ep_6;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func13_wrapper_ib_ep_7;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func14_wrapper_ib_ep_0;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func14_wrapper_ib_ep_1;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func14_wrapper_ib_ep_2;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func14_wrapper_ib_ep_3;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func14_wrapper_ib_ep_4;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func14_wrapper_ib_ep_5;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func14_wrapper_ib_ep_6;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func14_wrapper_ib_ep_7;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func15_wrapper_ib_ep_0;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func15_wrapper_ib_ep_1;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func15_wrapper_ib_ep_2;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func15_wrapper_ib_ep_3;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func15_wrapper_ib_ep_4;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func15_wrapper_ib_ep_5;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func15_wrapper_ib_ep_6;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func15_wrapper_ib_ep_7;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func16_wrapper_ib_ep_0;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func16_wrapper_ib_ep_1;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func16_wrapper_ib_ep_2;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func16_wrapper_ib_ep_3;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func16_wrapper_ib_ep_4;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func16_wrapper_ib_ep_5;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func16_wrapper_ib_ep_6;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func16_wrapper_ib_ep_7;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func17_wrapper_ib_ep_0;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func17_wrapper_ib_ep_1;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func17_wrapper_ib_ep_2;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func17_wrapper_ib_ep_3;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func17_wrapper_ib_ep_4;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func17_wrapper_ib_ep_5;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func17_wrapper_ib_ep_6;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func17_wrapper_ib_ep_7;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func18_wrapper_ib_ep_0;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func18_wrapper_ib_ep_1;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func18_wrapper_ib_ep_2;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func18_wrapper_ib_ep_3;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func18_wrapper_ib_ep_4;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func18_wrapper_ib_ep_5;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func18_wrapper_ib_ep_6;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func18_wrapper_ib_ep_7;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func19_wrapper_ib_ep_0;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func19_wrapper_ib_ep_1;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func19_wrapper_ib_ep_2;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func19_wrapper_ib_ep_3;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func19_wrapper_ib_ep_4;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func19_wrapper_ib_ep_5;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func19_wrapper_ib_ep_6;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func19_wrapper_ib_ep_7;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func20_wrapper_ib_ep_0;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func20_wrapper_ib_ep_1;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func20_wrapper_ib_ep_2;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func20_wrapper_ib_ep_3;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func20_wrapper_ib_ep_4;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func20_wrapper_ib_ep_5;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func20_wrapper_ib_ep_6;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func20_wrapper_ib_ep_7;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func21_wrapper_ib_ep_0;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func21_wrapper_ib_ep_1;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func21_wrapper_ib_ep_2;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func21_wrapper_ib_ep_3;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func21_wrapper_ib_ep_4;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func21_wrapper_ib_ep_5;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func21_wrapper_ib_ep_6;


typedef struct {
    volatile uint32_t ADDR0;
    volatile uint32_t ADDR1;
} CSL_pcie_rp_coreRegs_atu_func21_wrapper_ib_ep_7;


typedef struct {
    CSL_pcie_rp_coreRegs_RC_i_rc_pcie_base RC_I_RC_PCIE_BASE;
    volatile uint8_t  Resv_1216[432];
    CSL_pcie_rp_coreRegs_RC_i_VC_cap_struct RC_I_VC_CAP_STRUCT;
    volatile uint8_t  Resv_2304[1024];
    CSL_pcie_rp_coreRegs_RC_i_regf_L1_PM_cap_struct RC_I_REGF_L1_PM_CAP_STRUCT;
    CSL_pcie_rp_coreRegs_RC_i_regf_dl_feature_cap RC_I_REGF_DL_FEATURE_CAP;
    volatile uint8_t  Resv_2336[4];
    CSL_pcie_rp_coreRegs_RC_i_regf_margining_cap RC_I_REGF_MARGINING_CAP;
    volatile uint8_t  Resv_2496[144];
    CSL_pcie_rp_coreRegs_RC_i_regf_pl_16gts_cap RC_I_REGF_PL_16GTS_CAP;
    volatile uint8_t  Resv_2592[60];
    CSL_pcie_rp_coreRegs_RC_i_regf_ptm_cap RC_I_REGF_PTM_CAP;
    volatile uint8_t  Resv_1048576[1045972];
    CSL_pcie_rp_coreRegs_LM_i_regf_lm_pcie_base LM_I_REGF_LM_PCIE_BASE;
    volatile uint8_t  Resv_4194304[3142076];
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_0 ATU_WRAPPER_OB_0;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_1 ATU_WRAPPER_OB_1;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_2 ATU_WRAPPER_OB_2;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_3 ATU_WRAPPER_OB_3;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_4 ATU_WRAPPER_OB_4;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_5 ATU_WRAPPER_OB_5;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_6 ATU_WRAPPER_OB_6;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_7 ATU_WRAPPER_OB_7;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_8 ATU_WRAPPER_OB_8;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_9 ATU_WRAPPER_OB_9;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_10 ATU_WRAPPER_OB_10;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_11 ATU_WRAPPER_OB_11;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_12 ATU_WRAPPER_OB_12;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_13 ATU_WRAPPER_OB_13;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_14 ATU_WRAPPER_OB_14;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_15 ATU_WRAPPER_OB_15;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_16 ATU_WRAPPER_OB_16;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_17 ATU_WRAPPER_OB_17;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_18 ATU_WRAPPER_OB_18;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_19 ATU_WRAPPER_OB_19;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_20 ATU_WRAPPER_OB_20;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_21 ATU_WRAPPER_OB_21;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_22 ATU_WRAPPER_OB_22;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_23 ATU_WRAPPER_OB_23;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_24 ATU_WRAPPER_OB_24;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_25 ATU_WRAPPER_OB_25;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_26 ATU_WRAPPER_OB_26;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_27 ATU_WRAPPER_OB_27;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_28 ATU_WRAPPER_OB_28;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_29 ATU_WRAPPER_OB_29;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_30 ATU_WRAPPER_OB_30;
    CSL_pcie_rp_coreRegs_atu_wrapper_ob_31 ATU_WRAPPER_OB_31;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_0 ATU_HP_WRAPPER_OB_0;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_1 ATU_HP_WRAPPER_OB_1;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_2 ATU_HP_WRAPPER_OB_2;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_3 ATU_HP_WRAPPER_OB_3;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_4 ATU_HP_WRAPPER_OB_4;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_5 ATU_HP_WRAPPER_OB_5;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_6 ATU_HP_WRAPPER_OB_6;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_7 ATU_HP_WRAPPER_OB_7;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_8 ATU_HP_WRAPPER_OB_8;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_9 ATU_HP_WRAPPER_OB_9;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_10 ATU_HP_WRAPPER_OB_10;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_11 ATU_HP_WRAPPER_OB_11;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_12 ATU_HP_WRAPPER_OB_12;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_13 ATU_HP_WRAPPER_OB_13;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_14 ATU_HP_WRAPPER_OB_14;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_15 ATU_HP_WRAPPER_OB_15;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_16 ATU_HP_WRAPPER_OB_16;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_17 ATU_HP_WRAPPER_OB_17;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_18 ATU_HP_WRAPPER_OB_18;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_19 ATU_HP_WRAPPER_OB_19;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_20 ATU_HP_WRAPPER_OB_20;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_21 ATU_HP_WRAPPER_OB_21;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_22 ATU_HP_WRAPPER_OB_22;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_23 ATU_HP_WRAPPER_OB_23;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_24 ATU_HP_WRAPPER_OB_24;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_25 ATU_HP_WRAPPER_OB_25;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_26 ATU_HP_WRAPPER_OB_26;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_27 ATU_HP_WRAPPER_OB_27;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_28 ATU_HP_WRAPPER_OB_28;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_29 ATU_HP_WRAPPER_OB_29;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_30 ATU_HP_WRAPPER_OB_30;
    CSL_pcie_rp_coreRegs_atu_hp_wrapper_ob_31 ATU_HP_WRAPPER_OB_31;
    CSL_pcie_rp_coreRegs_atu_wrapper_ib_0 ATU_WRAPPER_IB_0;
    CSL_pcie_rp_coreRegs_atu_wrapper_ib_1 ATU_WRAPPER_IB_1;
    CSL_pcie_rp_coreRegs_atu_wrapper_ib_2 ATU_WRAPPER_IB_2;
    volatile uint8_t  Resv_4196384[8];
    CSL_pcie_rp_coreRegs_atu_credit_threshold ATU_CREDIT_THRESHOLD;
    CSL_pcie_rp_coreRegs_atu_link_down_indicator_bit ATU_LINK_DOWN_INDICATOR_BIT;
    volatile uint8_t  Resv_4196416[24];
    CSL_pcie_rp_coreRegs_atu_func0_wrapper_ib_ep_0 ATU_FUNC0_WRAPPER_IB_EP_0;
    CSL_pcie_rp_coreRegs_atu_func0_wrapper_ib_ep_1 ATU_FUNC0_WRAPPER_IB_EP_1;
    CSL_pcie_rp_coreRegs_atu_func0_wrapper_ib_ep_2 ATU_FUNC0_WRAPPER_IB_EP_2;
    CSL_pcie_rp_coreRegs_atu_func0_wrapper_ib_ep_3 ATU_FUNC0_WRAPPER_IB_EP_3;
    CSL_pcie_rp_coreRegs_atu_func0_wrapper_ib_ep_4 ATU_FUNC0_WRAPPER_IB_EP_4;
    CSL_pcie_rp_coreRegs_atu_func0_wrapper_ib_ep_5 ATU_FUNC0_WRAPPER_IB_EP_5;
    CSL_pcie_rp_coreRegs_atu_func0_wrapper_ib_ep_6 ATU_FUNC0_WRAPPER_IB_EP_6;
    CSL_pcie_rp_coreRegs_atu_func0_wrapper_ib_ep_7 ATU_FUNC0_WRAPPER_IB_EP_7;
    CSL_pcie_rp_coreRegs_atu_func1_wrapper_ib_ep_0 ATU_FUNC1_WRAPPER_IB_EP_0;
    CSL_pcie_rp_coreRegs_atu_func1_wrapper_ib_ep_1 ATU_FUNC1_WRAPPER_IB_EP_1;
    CSL_pcie_rp_coreRegs_atu_func1_wrapper_ib_ep_2 ATU_FUNC1_WRAPPER_IB_EP_2;
    CSL_pcie_rp_coreRegs_atu_func1_wrapper_ib_ep_3 ATU_FUNC1_WRAPPER_IB_EP_3;
    CSL_pcie_rp_coreRegs_atu_func1_wrapper_ib_ep_4 ATU_FUNC1_WRAPPER_IB_EP_4;
    CSL_pcie_rp_coreRegs_atu_func1_wrapper_ib_ep_5 ATU_FUNC1_WRAPPER_IB_EP_5;
    CSL_pcie_rp_coreRegs_atu_func1_wrapper_ib_ep_6 ATU_FUNC1_WRAPPER_IB_EP_6;
    CSL_pcie_rp_coreRegs_atu_func1_wrapper_ib_ep_7 ATU_FUNC1_WRAPPER_IB_EP_7;
    CSL_pcie_rp_coreRegs_atu_func2_wrapper_ib_ep_0 ATU_FUNC2_WRAPPER_IB_EP_0;
    CSL_pcie_rp_coreRegs_atu_func2_wrapper_ib_ep_1 ATU_FUNC2_WRAPPER_IB_EP_1;
    CSL_pcie_rp_coreRegs_atu_func2_wrapper_ib_ep_2 ATU_FUNC2_WRAPPER_IB_EP_2;
    CSL_pcie_rp_coreRegs_atu_func2_wrapper_ib_ep_3 ATU_FUNC2_WRAPPER_IB_EP_3;
    CSL_pcie_rp_coreRegs_atu_func2_wrapper_ib_ep_4 ATU_FUNC2_WRAPPER_IB_EP_4;
    CSL_pcie_rp_coreRegs_atu_func2_wrapper_ib_ep_5 ATU_FUNC2_WRAPPER_IB_EP_5;
    CSL_pcie_rp_coreRegs_atu_func2_wrapper_ib_ep_6 ATU_FUNC2_WRAPPER_IB_EP_6;
    CSL_pcie_rp_coreRegs_atu_func2_wrapper_ib_ep_7 ATU_FUNC2_WRAPPER_IB_EP_7;
    CSL_pcie_rp_coreRegs_atu_func3_wrapper_ib_ep_0 ATU_FUNC3_WRAPPER_IB_EP_0;
    CSL_pcie_rp_coreRegs_atu_func3_wrapper_ib_ep_1 ATU_FUNC3_WRAPPER_IB_EP_1;
    CSL_pcie_rp_coreRegs_atu_func3_wrapper_ib_ep_2 ATU_FUNC3_WRAPPER_IB_EP_2;
    CSL_pcie_rp_coreRegs_atu_func3_wrapper_ib_ep_3 ATU_FUNC3_WRAPPER_IB_EP_3;
    CSL_pcie_rp_coreRegs_atu_func3_wrapper_ib_ep_4 ATU_FUNC3_WRAPPER_IB_EP_4;
    CSL_pcie_rp_coreRegs_atu_func3_wrapper_ib_ep_5 ATU_FUNC3_WRAPPER_IB_EP_5;
    CSL_pcie_rp_coreRegs_atu_func3_wrapper_ib_ep_6 ATU_FUNC3_WRAPPER_IB_EP_6;
    CSL_pcie_rp_coreRegs_atu_func3_wrapper_ib_ep_7 ATU_FUNC3_WRAPPER_IB_EP_7;
    CSL_pcie_rp_coreRegs_atu_func4_wrapper_ib_ep_0 ATU_FUNC4_WRAPPER_IB_EP_0;
    CSL_pcie_rp_coreRegs_atu_func4_wrapper_ib_ep_1 ATU_FUNC4_WRAPPER_IB_EP_1;
    CSL_pcie_rp_coreRegs_atu_func4_wrapper_ib_ep_2 ATU_FUNC4_WRAPPER_IB_EP_2;
    CSL_pcie_rp_coreRegs_atu_func4_wrapper_ib_ep_3 ATU_FUNC4_WRAPPER_IB_EP_3;
    CSL_pcie_rp_coreRegs_atu_func4_wrapper_ib_ep_4 ATU_FUNC4_WRAPPER_IB_EP_4;
    CSL_pcie_rp_coreRegs_atu_func4_wrapper_ib_ep_5 ATU_FUNC4_WRAPPER_IB_EP_5;
    CSL_pcie_rp_coreRegs_atu_func4_wrapper_ib_ep_6 ATU_FUNC4_WRAPPER_IB_EP_6;
    CSL_pcie_rp_coreRegs_atu_func4_wrapper_ib_ep_7 ATU_FUNC4_WRAPPER_IB_EP_7;
    CSL_pcie_rp_coreRegs_atu_func5_wrapper_ib_ep_0 ATU_FUNC5_WRAPPER_IB_EP_0;
    CSL_pcie_rp_coreRegs_atu_func5_wrapper_ib_ep_1 ATU_FUNC5_WRAPPER_IB_EP_1;
    CSL_pcie_rp_coreRegs_atu_func5_wrapper_ib_ep_2 ATU_FUNC5_WRAPPER_IB_EP_2;
    CSL_pcie_rp_coreRegs_atu_func5_wrapper_ib_ep_3 ATU_FUNC5_WRAPPER_IB_EP_3;
    CSL_pcie_rp_coreRegs_atu_func5_wrapper_ib_ep_4 ATU_FUNC5_WRAPPER_IB_EP_4;
    CSL_pcie_rp_coreRegs_atu_func5_wrapper_ib_ep_5 ATU_FUNC5_WRAPPER_IB_EP_5;
    CSL_pcie_rp_coreRegs_atu_func5_wrapper_ib_ep_6 ATU_FUNC5_WRAPPER_IB_EP_6;
    CSL_pcie_rp_coreRegs_atu_func5_wrapper_ib_ep_7 ATU_FUNC5_WRAPPER_IB_EP_7;
    CSL_pcie_rp_coreRegs_atu_func6_wrapper_ib_ep_0 ATU_FUNC6_WRAPPER_IB_EP_0;
    CSL_pcie_rp_coreRegs_atu_func6_wrapper_ib_ep_1 ATU_FUNC6_WRAPPER_IB_EP_1;
    CSL_pcie_rp_coreRegs_atu_func6_wrapper_ib_ep_2 ATU_FUNC6_WRAPPER_IB_EP_2;
    CSL_pcie_rp_coreRegs_atu_func6_wrapper_ib_ep_3 ATU_FUNC6_WRAPPER_IB_EP_3;
    CSL_pcie_rp_coreRegs_atu_func6_wrapper_ib_ep_4 ATU_FUNC6_WRAPPER_IB_EP_4;
    CSL_pcie_rp_coreRegs_atu_func6_wrapper_ib_ep_5 ATU_FUNC6_WRAPPER_IB_EP_5;
    CSL_pcie_rp_coreRegs_atu_func6_wrapper_ib_ep_6 ATU_FUNC6_WRAPPER_IB_EP_6;
    CSL_pcie_rp_coreRegs_atu_func6_wrapper_ib_ep_7 ATU_FUNC6_WRAPPER_IB_EP_7;
    CSL_pcie_rp_coreRegs_atu_func7_wrapper_ib_ep_0 ATU_FUNC7_WRAPPER_IB_EP_0;
    CSL_pcie_rp_coreRegs_atu_func7_wrapper_ib_ep_1 ATU_FUNC7_WRAPPER_IB_EP_1;
    CSL_pcie_rp_coreRegs_atu_func7_wrapper_ib_ep_2 ATU_FUNC7_WRAPPER_IB_EP_2;
    CSL_pcie_rp_coreRegs_atu_func7_wrapper_ib_ep_3 ATU_FUNC7_WRAPPER_IB_EP_3;
    CSL_pcie_rp_coreRegs_atu_func7_wrapper_ib_ep_4 ATU_FUNC7_WRAPPER_IB_EP_4;
    CSL_pcie_rp_coreRegs_atu_func7_wrapper_ib_ep_5 ATU_FUNC7_WRAPPER_IB_EP_5;
    CSL_pcie_rp_coreRegs_atu_func7_wrapper_ib_ep_6 ATU_FUNC7_WRAPPER_IB_EP_6;
    CSL_pcie_rp_coreRegs_atu_func7_wrapper_ib_ep_7 ATU_FUNC7_WRAPPER_IB_EP_7;
    CSL_pcie_rp_coreRegs_atu_func8_wrapper_ib_ep_0 ATU_FUNC8_WRAPPER_IB_EP_0;
    CSL_pcie_rp_coreRegs_atu_func8_wrapper_ib_ep_1 ATU_FUNC8_WRAPPER_IB_EP_1;
    CSL_pcie_rp_coreRegs_atu_func8_wrapper_ib_ep_2 ATU_FUNC8_WRAPPER_IB_EP_2;
    CSL_pcie_rp_coreRegs_atu_func8_wrapper_ib_ep_3 ATU_FUNC8_WRAPPER_IB_EP_3;
    CSL_pcie_rp_coreRegs_atu_func8_wrapper_ib_ep_4 ATU_FUNC8_WRAPPER_IB_EP_4;
    CSL_pcie_rp_coreRegs_atu_func8_wrapper_ib_ep_5 ATU_FUNC8_WRAPPER_IB_EP_5;
    CSL_pcie_rp_coreRegs_atu_func8_wrapper_ib_ep_6 ATU_FUNC8_WRAPPER_IB_EP_6;
    CSL_pcie_rp_coreRegs_atu_func8_wrapper_ib_ep_7 ATU_FUNC8_WRAPPER_IB_EP_7;
    CSL_pcie_rp_coreRegs_atu_func9_wrapper_ib_ep_0 ATU_FUNC9_WRAPPER_IB_EP_0;
    CSL_pcie_rp_coreRegs_atu_func9_wrapper_ib_ep_1 ATU_FUNC9_WRAPPER_IB_EP_1;
    CSL_pcie_rp_coreRegs_atu_func9_wrapper_ib_ep_2 ATU_FUNC9_WRAPPER_IB_EP_2;
    CSL_pcie_rp_coreRegs_atu_func9_wrapper_ib_ep_3 ATU_FUNC9_WRAPPER_IB_EP_3;
    CSL_pcie_rp_coreRegs_atu_func9_wrapper_ib_ep_4 ATU_FUNC9_WRAPPER_IB_EP_4;
    CSL_pcie_rp_coreRegs_atu_func9_wrapper_ib_ep_5 ATU_FUNC9_WRAPPER_IB_EP_5;
    CSL_pcie_rp_coreRegs_atu_func9_wrapper_ib_ep_6 ATU_FUNC9_WRAPPER_IB_EP_6;
    CSL_pcie_rp_coreRegs_atu_func9_wrapper_ib_ep_7 ATU_FUNC9_WRAPPER_IB_EP_7;
    CSL_pcie_rp_coreRegs_atu_func10_wrapper_ib_ep_0 ATU_FUNC10_WRAPPER_IB_EP_0;
    CSL_pcie_rp_coreRegs_atu_func10_wrapper_ib_ep_1 ATU_FUNC10_WRAPPER_IB_EP_1;
    CSL_pcie_rp_coreRegs_atu_func10_wrapper_ib_ep_2 ATU_FUNC10_WRAPPER_IB_EP_2;
    CSL_pcie_rp_coreRegs_atu_func10_wrapper_ib_ep_3 ATU_FUNC10_WRAPPER_IB_EP_3;
    CSL_pcie_rp_coreRegs_atu_func10_wrapper_ib_ep_4 ATU_FUNC10_WRAPPER_IB_EP_4;
    CSL_pcie_rp_coreRegs_atu_func10_wrapper_ib_ep_5 ATU_FUNC10_WRAPPER_IB_EP_5;
    CSL_pcie_rp_coreRegs_atu_func10_wrapper_ib_ep_6 ATU_FUNC10_WRAPPER_IB_EP_6;
    CSL_pcie_rp_coreRegs_atu_func10_wrapper_ib_ep_7 ATU_FUNC10_WRAPPER_IB_EP_7;
    CSL_pcie_rp_coreRegs_atu_func11_wrapper_ib_ep_0 ATU_FUNC11_WRAPPER_IB_EP_0;
    CSL_pcie_rp_coreRegs_atu_func11_wrapper_ib_ep_1 ATU_FUNC11_WRAPPER_IB_EP_1;
    CSL_pcie_rp_coreRegs_atu_func11_wrapper_ib_ep_2 ATU_FUNC11_WRAPPER_IB_EP_2;
    CSL_pcie_rp_coreRegs_atu_func11_wrapper_ib_ep_3 ATU_FUNC11_WRAPPER_IB_EP_3;
    CSL_pcie_rp_coreRegs_atu_func11_wrapper_ib_ep_4 ATU_FUNC11_WRAPPER_IB_EP_4;
    CSL_pcie_rp_coreRegs_atu_func11_wrapper_ib_ep_5 ATU_FUNC11_WRAPPER_IB_EP_5;
    CSL_pcie_rp_coreRegs_atu_func11_wrapper_ib_ep_6 ATU_FUNC11_WRAPPER_IB_EP_6;
    CSL_pcie_rp_coreRegs_atu_func11_wrapper_ib_ep_7 ATU_FUNC11_WRAPPER_IB_EP_7;
    CSL_pcie_rp_coreRegs_atu_func12_wrapper_ib_ep_0 ATU_FUNC12_WRAPPER_IB_EP_0;
    CSL_pcie_rp_coreRegs_atu_func12_wrapper_ib_ep_1 ATU_FUNC12_WRAPPER_IB_EP_1;
    CSL_pcie_rp_coreRegs_atu_func12_wrapper_ib_ep_2 ATU_FUNC12_WRAPPER_IB_EP_2;
    CSL_pcie_rp_coreRegs_atu_func12_wrapper_ib_ep_3 ATU_FUNC12_WRAPPER_IB_EP_3;
    CSL_pcie_rp_coreRegs_atu_func12_wrapper_ib_ep_4 ATU_FUNC12_WRAPPER_IB_EP_4;
    CSL_pcie_rp_coreRegs_atu_func12_wrapper_ib_ep_5 ATU_FUNC12_WRAPPER_IB_EP_5;
    CSL_pcie_rp_coreRegs_atu_func12_wrapper_ib_ep_6 ATU_FUNC12_WRAPPER_IB_EP_6;
    CSL_pcie_rp_coreRegs_atu_func12_wrapper_ib_ep_7 ATU_FUNC12_WRAPPER_IB_EP_7;
    CSL_pcie_rp_coreRegs_atu_func13_wrapper_ib_ep_0 ATU_FUNC13_WRAPPER_IB_EP_0;
    CSL_pcie_rp_coreRegs_atu_func13_wrapper_ib_ep_1 ATU_FUNC13_WRAPPER_IB_EP_1;
    CSL_pcie_rp_coreRegs_atu_func13_wrapper_ib_ep_2 ATU_FUNC13_WRAPPER_IB_EP_2;
    CSL_pcie_rp_coreRegs_atu_func13_wrapper_ib_ep_3 ATU_FUNC13_WRAPPER_IB_EP_3;
    CSL_pcie_rp_coreRegs_atu_func13_wrapper_ib_ep_4 ATU_FUNC13_WRAPPER_IB_EP_4;
    CSL_pcie_rp_coreRegs_atu_func13_wrapper_ib_ep_5 ATU_FUNC13_WRAPPER_IB_EP_5;
    CSL_pcie_rp_coreRegs_atu_func13_wrapper_ib_ep_6 ATU_FUNC13_WRAPPER_IB_EP_6;
    CSL_pcie_rp_coreRegs_atu_func13_wrapper_ib_ep_7 ATU_FUNC13_WRAPPER_IB_EP_7;
    CSL_pcie_rp_coreRegs_atu_func14_wrapper_ib_ep_0 ATU_FUNC14_WRAPPER_IB_EP_0;
    CSL_pcie_rp_coreRegs_atu_func14_wrapper_ib_ep_1 ATU_FUNC14_WRAPPER_IB_EP_1;
    CSL_pcie_rp_coreRegs_atu_func14_wrapper_ib_ep_2 ATU_FUNC14_WRAPPER_IB_EP_2;
    CSL_pcie_rp_coreRegs_atu_func14_wrapper_ib_ep_3 ATU_FUNC14_WRAPPER_IB_EP_3;
    CSL_pcie_rp_coreRegs_atu_func14_wrapper_ib_ep_4 ATU_FUNC14_WRAPPER_IB_EP_4;
    CSL_pcie_rp_coreRegs_atu_func14_wrapper_ib_ep_5 ATU_FUNC14_WRAPPER_IB_EP_5;
    CSL_pcie_rp_coreRegs_atu_func14_wrapper_ib_ep_6 ATU_FUNC14_WRAPPER_IB_EP_6;
    CSL_pcie_rp_coreRegs_atu_func14_wrapper_ib_ep_7 ATU_FUNC14_WRAPPER_IB_EP_7;
    CSL_pcie_rp_coreRegs_atu_func15_wrapper_ib_ep_0 ATU_FUNC15_WRAPPER_IB_EP_0;
    CSL_pcie_rp_coreRegs_atu_func15_wrapper_ib_ep_1 ATU_FUNC15_WRAPPER_IB_EP_1;
    CSL_pcie_rp_coreRegs_atu_func15_wrapper_ib_ep_2 ATU_FUNC15_WRAPPER_IB_EP_2;
    CSL_pcie_rp_coreRegs_atu_func15_wrapper_ib_ep_3 ATU_FUNC15_WRAPPER_IB_EP_3;
    CSL_pcie_rp_coreRegs_atu_func15_wrapper_ib_ep_4 ATU_FUNC15_WRAPPER_IB_EP_4;
    CSL_pcie_rp_coreRegs_atu_func15_wrapper_ib_ep_5 ATU_FUNC15_WRAPPER_IB_EP_5;
    CSL_pcie_rp_coreRegs_atu_func15_wrapper_ib_ep_6 ATU_FUNC15_WRAPPER_IB_EP_6;
    CSL_pcie_rp_coreRegs_atu_func15_wrapper_ib_ep_7 ATU_FUNC15_WRAPPER_IB_EP_7;
    CSL_pcie_rp_coreRegs_atu_func16_wrapper_ib_ep_0 ATU_FUNC16_WRAPPER_IB_EP_0;
    CSL_pcie_rp_coreRegs_atu_func16_wrapper_ib_ep_1 ATU_FUNC16_WRAPPER_IB_EP_1;
    CSL_pcie_rp_coreRegs_atu_func16_wrapper_ib_ep_2 ATU_FUNC16_WRAPPER_IB_EP_2;
    CSL_pcie_rp_coreRegs_atu_func16_wrapper_ib_ep_3 ATU_FUNC16_WRAPPER_IB_EP_3;
    CSL_pcie_rp_coreRegs_atu_func16_wrapper_ib_ep_4 ATU_FUNC16_WRAPPER_IB_EP_4;
    CSL_pcie_rp_coreRegs_atu_func16_wrapper_ib_ep_5 ATU_FUNC16_WRAPPER_IB_EP_5;
    CSL_pcie_rp_coreRegs_atu_func16_wrapper_ib_ep_6 ATU_FUNC16_WRAPPER_IB_EP_6;
    CSL_pcie_rp_coreRegs_atu_func16_wrapper_ib_ep_7 ATU_FUNC16_WRAPPER_IB_EP_7;
    CSL_pcie_rp_coreRegs_atu_func17_wrapper_ib_ep_0 ATU_FUNC17_WRAPPER_IB_EP_0;
    CSL_pcie_rp_coreRegs_atu_func17_wrapper_ib_ep_1 ATU_FUNC17_WRAPPER_IB_EP_1;
    CSL_pcie_rp_coreRegs_atu_func17_wrapper_ib_ep_2 ATU_FUNC17_WRAPPER_IB_EP_2;
    CSL_pcie_rp_coreRegs_atu_func17_wrapper_ib_ep_3 ATU_FUNC17_WRAPPER_IB_EP_3;
    CSL_pcie_rp_coreRegs_atu_func17_wrapper_ib_ep_4 ATU_FUNC17_WRAPPER_IB_EP_4;
    CSL_pcie_rp_coreRegs_atu_func17_wrapper_ib_ep_5 ATU_FUNC17_WRAPPER_IB_EP_5;
    CSL_pcie_rp_coreRegs_atu_func17_wrapper_ib_ep_6 ATU_FUNC17_WRAPPER_IB_EP_6;
    CSL_pcie_rp_coreRegs_atu_func17_wrapper_ib_ep_7 ATU_FUNC17_WRAPPER_IB_EP_7;
    CSL_pcie_rp_coreRegs_atu_func18_wrapper_ib_ep_0 ATU_FUNC18_WRAPPER_IB_EP_0;
    CSL_pcie_rp_coreRegs_atu_func18_wrapper_ib_ep_1 ATU_FUNC18_WRAPPER_IB_EP_1;
    CSL_pcie_rp_coreRegs_atu_func18_wrapper_ib_ep_2 ATU_FUNC18_WRAPPER_IB_EP_2;
    CSL_pcie_rp_coreRegs_atu_func18_wrapper_ib_ep_3 ATU_FUNC18_WRAPPER_IB_EP_3;
    CSL_pcie_rp_coreRegs_atu_func18_wrapper_ib_ep_4 ATU_FUNC18_WRAPPER_IB_EP_4;
    CSL_pcie_rp_coreRegs_atu_func18_wrapper_ib_ep_5 ATU_FUNC18_WRAPPER_IB_EP_5;
    CSL_pcie_rp_coreRegs_atu_func18_wrapper_ib_ep_6 ATU_FUNC18_WRAPPER_IB_EP_6;
    CSL_pcie_rp_coreRegs_atu_func18_wrapper_ib_ep_7 ATU_FUNC18_WRAPPER_IB_EP_7;
    CSL_pcie_rp_coreRegs_atu_func19_wrapper_ib_ep_0 ATU_FUNC19_WRAPPER_IB_EP_0;
    CSL_pcie_rp_coreRegs_atu_func19_wrapper_ib_ep_1 ATU_FUNC19_WRAPPER_IB_EP_1;
    CSL_pcie_rp_coreRegs_atu_func19_wrapper_ib_ep_2 ATU_FUNC19_WRAPPER_IB_EP_2;
    CSL_pcie_rp_coreRegs_atu_func19_wrapper_ib_ep_3 ATU_FUNC19_WRAPPER_IB_EP_3;
    CSL_pcie_rp_coreRegs_atu_func19_wrapper_ib_ep_4 ATU_FUNC19_WRAPPER_IB_EP_4;
    CSL_pcie_rp_coreRegs_atu_func19_wrapper_ib_ep_5 ATU_FUNC19_WRAPPER_IB_EP_5;
    CSL_pcie_rp_coreRegs_atu_func19_wrapper_ib_ep_6 ATU_FUNC19_WRAPPER_IB_EP_6;
    CSL_pcie_rp_coreRegs_atu_func19_wrapper_ib_ep_7 ATU_FUNC19_WRAPPER_IB_EP_7;
    CSL_pcie_rp_coreRegs_atu_func20_wrapper_ib_ep_0 ATU_FUNC20_WRAPPER_IB_EP_0;
    CSL_pcie_rp_coreRegs_atu_func20_wrapper_ib_ep_1 ATU_FUNC20_WRAPPER_IB_EP_1;
    CSL_pcie_rp_coreRegs_atu_func20_wrapper_ib_ep_2 ATU_FUNC20_WRAPPER_IB_EP_2;
    CSL_pcie_rp_coreRegs_atu_func20_wrapper_ib_ep_3 ATU_FUNC20_WRAPPER_IB_EP_3;
    CSL_pcie_rp_coreRegs_atu_func20_wrapper_ib_ep_4 ATU_FUNC20_WRAPPER_IB_EP_4;
    CSL_pcie_rp_coreRegs_atu_func20_wrapper_ib_ep_5 ATU_FUNC20_WRAPPER_IB_EP_5;
    CSL_pcie_rp_coreRegs_atu_func20_wrapper_ib_ep_6 ATU_FUNC20_WRAPPER_IB_EP_6;
    CSL_pcie_rp_coreRegs_atu_func20_wrapper_ib_ep_7 ATU_FUNC20_WRAPPER_IB_EP_7;
    CSL_pcie_rp_coreRegs_atu_func21_wrapper_ib_ep_0 ATU_FUNC21_WRAPPER_IB_EP_0;
    CSL_pcie_rp_coreRegs_atu_func21_wrapper_ib_ep_1 ATU_FUNC21_WRAPPER_IB_EP_1;
    CSL_pcie_rp_coreRegs_atu_func21_wrapper_ib_ep_2 ATU_FUNC21_WRAPPER_IB_EP_2;
    CSL_pcie_rp_coreRegs_atu_func21_wrapper_ib_ep_3 ATU_FUNC21_WRAPPER_IB_EP_3;
    CSL_pcie_rp_coreRegs_atu_func21_wrapper_ib_ep_4 ATU_FUNC21_WRAPPER_IB_EP_4;
    CSL_pcie_rp_coreRegs_atu_func21_wrapper_ib_ep_5 ATU_FUNC21_WRAPPER_IB_EP_5;
    CSL_pcie_rp_coreRegs_atu_func21_wrapper_ib_ep_6 ATU_FUNC21_WRAPPER_IB_EP_6;
    CSL_pcie_rp_coreRegs_atu_func21_wrapper_ib_ep_7 ATU_FUNC21_WRAPPER_IB_EP_7;
} CSL_pcie_rp_coreRegs;
#else /* CSL_MODIFICATION */
typedef struct {
    CSL_pcie_rp_coreRegs_RC_i_rc_pcie_base RC_I_RC_PCIE_BASE;
    volatile uint8_t  Resv_1216[432];
    CSL_pcie_rp_coreRegs_RC_i_VC_cap_struct RC_I_VC_CAP_STRUCT;
    volatile uint8_t  Resv_2304[1024];
    CSL_pcie_rp_coreRegs_RC_i_regf_L1_PM_cap_struct RC_I_REGF_L1_PM_CAP_STRUCT;
    CSL_pcie_rp_coreRegs_RC_i_regf_dl_feature_cap RC_I_REGF_DL_FEATURE_CAP;
    volatile uint8_t  Resv_2336[4];
    CSL_pcie_rp_coreRegs_RC_i_regf_margining_cap RC_I_REGF_MARGINING_CAP;
    volatile uint8_t  Resv_2496[144];
    CSL_pcie_rp_coreRegs_RC_i_regf_pl_16gts_cap RC_I_REGF_PL_16GTS_CAP;
    volatile uint8_t  Resv_2592[60];
    CSL_pcie_rp_coreRegs_RC_i_regf_ptm_cap RC_I_REGF_PTM_CAP;
    volatile uint8_t  Resv_1048576[1045972];
    CSL_pcie_rp_coreRegs_LM_i_regf_lm_pcie_base LM_I_REGF_LM_PCIE_BASE;
    volatile uint8_t  Resv_4194304[3142076];
    CSL_pcie_rp_coreRegs_atu_wrapper_ob ATU_WRAPPER_OB[32];
    CSL_pcie_rp_coreRegs_atu_wrapper_ob ATU_HP_WRAPPER_OB[32];
    CSL_pcie_rp_coreRegs_atu_wrapper_ib ATU_WRAPPER_IB[3];
    volatile uint8_t  Resv_4196384[8];
    CSL_pcie_rp_coreRegs_atu_credit_threshold ATU_CREDIT_THRESHOLD;
    CSL_pcie_rp_coreRegs_atu_link_down_indicator_bit ATU_LINK_DOWN_INDICATOR_BIT;
    volatile uint8_t  Resv_4196416[24];
    CSL_pcie_rp_coreRegs_atu_wrapper_ib ATU_FUNC0_WRAPPER_IB_EP[22][8];
} CSL_pcie_rp_coreRegs;
#endif /* CSL_MODIFICATION */


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_VENDOR_ID_DEVICE_ID                  (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS                       (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_REVISION_ID_CLASS_CODE               (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_BIST_HEADER_LATENCY_CACHE_LINE       (0x0000000CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_RC_BAR_0                             (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_RC_BAR_1                             (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_BUS_NUMBERS                     (0x00000018U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT                   (0x0000001CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_MEM_BASE_LIMIT                  (0x00000020U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_PREFETCH_BASE_LIMIT             (0x00000024U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_PREFETCH_BASE_UPPER             (0x00000028U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_PREFETCH_LIMIT_UPPER            (0x0000002CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_UPPER             (0x00000030U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CAPABILITIES_POINTER                 (0x00000034U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_RSVD_0E                                (0x00000038U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN               (0x0000003CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP                         (0x00000080U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP               (0x00000084U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_CTRL_REG                         (0x00000090U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MSG_LOW_ADDR                     (0x00000094U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MSG_HI_ADDR                      (0x00000098U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MSG_DATA                         (0x0000009CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MASK                             (0x000000A0U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_PENDING_BITS                     (0x000000A4U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL                            (0x000000B0U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_TBL_OFFSET                      (0x000000B4U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_PENDING_INTRPT                  (0x000000B8U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_LIST                        (0x000000C0U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP                             (0x000000C4U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS                 (0x000000C8U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP                             (0x000000CCU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS                     (0x000000D0U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY                      (0x000000D4U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS                     (0x000000D8U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_CTRL_CAP                        (0x000000DCU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_STATUS                          (0x000000E0U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2                           (0x000000E4U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2               (0x000000E8U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2                           (0x000000ECU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2                   (0x000000F0U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_AER_ENHNCD_CAP                       (0x00000100U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS                    (0x00000104U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK                      (0x00000108U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY                  (0x0000010CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS                      (0x00000110U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK                        (0x00000114U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL                      (0x00000118U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_HDR_LOG_0                            (0x0000011CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_HDR_LOG_1                            (0x00000120U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_HDR_LOG_2                            (0x00000124U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_HDR_LOG_3                            (0x00000128U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_CMD                         (0x0000012CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_STAT                        (0x00000130U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ERR_SRC_ID                           (0x00000134U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_TLP_PRE_LOG_0                        (0x00000138U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_DEV_SER_NUM_CAP_HDR                  (0x00000150U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_DEV_SER_NUM_0                        (0x00000154U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_DEV_SER_NUM_1                        (0x00000158U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SEC_PCIE_CAP_HDR_REG                 (0x00000300U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CONTROL3                        (0x00000304U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_ERROR_STATUS                    (0x00000308U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0          (0x0000030CU)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_ENH_CAP_HEADER_REG               (0x000004C0U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_PORT_VC_CAP_REG_1                   (0x000004C4U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_PORT_VC_CAP_REG_2                   (0x000004C8U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_PORT_VC_CTRL_STS_REG                (0x000004CCU)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_0                    (0x000004D0U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_0                   (0x000004D4U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_0                    (0x000004D8U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_1                    (0x000004DCU)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_1                   (0x000004E0U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_1                    (0x000004E4U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_2                    (0x000004E8U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_2                   (0x000004ECU)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_2                    (0x000004F0U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_3                    (0x000004F4U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_3                   (0x000004F8U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_3                    (0x000004FCU)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_EXT_CAP_HDR           (0x00000900U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CAP                   (0x00000904U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_1                (0x00000908U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_2                (0x0000090CU)
#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_EXTENDED_CAPABILITY_HEADER_REG (0x00000910U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_CAPABILITIES_REG   (0x00000914U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_STATUS_REG         (0x00000918U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_EXTENDED_CAPABILITY_HEADER_REG (0x00000920U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_PORT_CAPABILITIES_STATUS_REG (0x00000924U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0 (0x00000928U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1 (0x0000092CU)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_EXTENDED_CAPABILITY_HEADER_REG (0x000009C0U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_CAPABILITIES_REG       (0x000009C4U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_CONTROL_REG            (0x000009C8U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_STATUS_REG             (0x000009CCU)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_LOCAL_DATA_PARITY_MISMATCH_STATUS_REG (0x000009D0U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_FIRST_RETIMER_DATA_PARITY_MISMATCH_STATUS_REG (0x000009D4U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_SECOND_RETIMER_DATA_PARITY_MISMATCH_STATUS_REG (0x000009D8U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_RESERVED_REG           (0x000009DCU)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_LANE_EQUALIZATION_CONTROL_REG0 (0x000009E0U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_EXTENDED_CAPABILITY_HEADER_REG   (0x00000A20U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CAPABILITIES_REG                 (0x00000A24U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CONTROL_REG                      (0x00000A28U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG                 (0x00100000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_1_REG                 (0x00100004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DLL_TMR_CONFIG_REG              (0x00100008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG              (0x0010000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG              (0x00100010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG           (0x00100014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG           (0x00100018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_UPDATE_INT_CONFIG_0_REG (0x0010001CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_UPDATE_INT_CONFIG_1_REG (0x00100020U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_L0S_TIMEOUT_LIMIT_REG           (0x00100024U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSMIT_TLP_COUNT_REG          (0x00100028U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSMIT_TLP_PAYLOAD_DWORD_COUNT_REG (0x0010002CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RECEIVE_TLP_COUNT_REG           (0x00100030U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RECEIVE_TLP_PAYLOAD_DWORD_COUNT_REG (0x00100034U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_COMPLN_TMOUT_LIM_0_REG          (0x00100038U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_COMPLN_TMOUT_LIM_1_REG          (0x0010003CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_L1_ST_REENTRY_DELAY_REG         (0x00100040U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_VENDOR_ID_REG                   (0x00100044U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASPM_L1_ENTRY_TMOUT_DELAY_REG   (0x00100048U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PME_TURNOFF_ACK_DELAY_REG       (0x0010004CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LINKWIDTH_CONTROL_REG           (0x00100050U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MULTI_VC_CONROL_REG             (0x00100070U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SRIS_CONTROL_REG                (0x00100074U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC1          (0x00100080U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC1          (0x00100084U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC1       (0x00100088U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC1       (0x0010008CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC2          (0x00100090U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC2          (0x00100094U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC2       (0x00100098U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC2       (0x0010009CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC3          (0x001000A0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC3          (0x001000A4U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC3       (0x001000A8U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC3       (0x001000ACU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_FC_INIT_DELAY_REG               (0x001000F0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_HDR_LOG_0_REG              (0x00100100U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_HDR_LOG_1_REG              (0x00100104U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_HDR_LOG_2_REG              (0x00100108U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_HDR_LOG_3_REG              (0x0010010CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_FUNC_NUM_REG               (0x00100110U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_UR_ERR_REG                 (0x00100114U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PM_CLK_FREQUENCY_REG            (0x00100140U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_DLLP_COUNT_GEN1_REG       (0x00100144U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_DLLP_COUNT_GEN2_REG       (0x00100148U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_DLLP_COUNT_GEN3_REG       (0x0010014CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_DLLP_COUNT_GEN4_REG       (0x00100150U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_VENDOR_DEFINED_MESSAGE_TAG_REG  (0x00100158U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_NEGOTIATED_LANE_MAP_REG         (0x00100200U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RECEIVE_FTS_COUNT_REG           (0x00100204U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG           (0x00100208U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER     (0x0010020CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG           (0x00100210U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LCRC_ERR_COUNT_REG              (0x00100214U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ECC_CORR_ERR_COUNT_REG          (0x00100218U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_SNOOP_LAT_REG               (0x0010021CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_MSG_GEN_CTL_REG             (0x00100220U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PME_SERVICE_TIMEOUT_DELAY_REG   (0x00100224U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ROOT_PORT_REQUESTOR_ID_REG      (0x00100228U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_EP_BUS_DEVICE_NUMBER_REG        (0x0010022CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG         (0x00100234U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG                (0x00100238U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG           (0x00100240U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_1_REG           (0x00100244U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_0_REG           (0x00100248U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_1_REG           (0x0010024CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_0_REG           (0x00100250U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_1_REG           (0x00100254U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_0_REG           (0x00100258U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_1_REG           (0x0010025CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_0_REG           (0x00100260U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_1_REG           (0x00100264U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_0_REG           (0x00100268U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_1_REG           (0x0010026CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_0_REG        (0x00100280U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_1_REG        (0x00100284U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_0_REG        (0x00100288U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_1_REG        (0x0010028CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_0_REG        (0x00100290U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_1_REG        (0x00100294U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_0_REG        (0x00100298U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_1_REG        (0x0010029CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_0_REG        (0x001002A0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_1_REG        (0x001002A4U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_0_REG        (0x001002A8U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_1_REG        (0x001002ACU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_CONFIG_REG                   (0x001002C0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG               (0x00100300U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_DEFAULT_PRESET_REG         (0x00100360U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_TIMEOUT_2MS_REG (0x00100364U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PIPE_FIFO_LATENCY_CTRL_REG      (0x00100368U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_DEFAULT_PRESET_REG         (0x00100374U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_CONFIG_REG3                 (0x00100378U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG      (0x0010037CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE0 (0x00100380U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE1 (0x00100384U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE0 (0x001003C0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE1 (0x001003C4U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ECC_CORR_ERR_COUNT_REG_AXI      (0x00100C80U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL0      (0x00100C88U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL1      (0x00100C8CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL2      (0x00100C90U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_TL_INTERNAL_CONTROL               (0x00100C94U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_STATUS                  (0x00100C98U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_CTRL                    (0x00100C9CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_VC_SELECT_REG (0x00100CC0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG    (0x00100CC4U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG      (0x00100CD0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_2_REG      (0x00100CD4U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_LOCAL_CONTROL_REG     (0x00100CD8U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS1_REG     (0x00100CDCU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS2_REG     (0x00100CE0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER   (0x00100D00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG         (0x00100D04U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1         (0x00100D10U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1             (0x00100D14U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1 (0x00100D18U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1   (0x00100D1CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LD_CTRL                         (0x00100DA0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_RX_ELEC_IDLE_FILTER_CONTROL       (0x00100DA4U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG           (0x00100DA8U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_STATUS_REG            (0x00100DACU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LATENCY_PARAMETERS_INDEX_REG (0x00100DB0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LATENCY_PARAMETERS_REG      (0x00100DB4U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_1_REG               (0x00100DB8U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_2_REG               (0x00100DBCU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_3_REG               (0x00100DC0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_4_REG               (0x00100DC4U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_5_REG               (0x00100DC8U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_6_REG               (0x00100DCCU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_7_REG               (0x00100DD0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_8_REG               (0x00100DD4U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_9_REG               (0x00100DD8U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_10_REG              (0x00100DDCU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_11_REG              (0x00100DE0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_STATUS               (0x00100DECU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_RAW_STATUS           (0x00100DF0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_MASK_REG             (0x00100DF4U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_TEST                 (0x00100DF8U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_FATAL_NONFATAL_SEL   (0x00100DFCU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_SRAM_CORR_FAULT_STATUS      (0x00100E00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_SRAM_UNCORR_FAULT_STATUS    (0x00100E04U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_SRAM_FAULT_STATSTICS        (0x00100E08U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_CTRL               (0x00100E0CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK         (0x00100E10U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS       (0x00100E14U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK         (0x00100E18U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG   (0x00100E1CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_DUAL_TL_CTRL                      (0x00100E20U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_MAGIC_NUM_CTRLLER_VER_REG   (0x00100E40U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_ADDR0                                   (0x00400000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_ADDR1                                   (0x00400004U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC0                                   (0x00400008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC1                                   (0x0040000CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC3                                   (0x00400014U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_AXI_ADDR0                               (0x00400018U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_AXI_ADDR1                               (0x0040001CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_ADDR0                                   (0x00400020U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_ADDR1                                   (0x00400024U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_DESC0                                   (0x00400028U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_DESC1                                   (0x0040002CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_DESC3                                   (0x00400034U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_AXI_ADDR0                               (0x00400038U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_AXI_ADDR1                               (0x0040003CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_ADDR0                                   (0x00400040U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_ADDR1                                   (0x00400044U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_DESC0                                   (0x00400048U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_DESC1                                   (0x0040004CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_DESC3                                   (0x00400054U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_AXI_ADDR0                               (0x00400058U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_AXI_ADDR1                               (0x0040005CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_ADDR0                                   (0x00400060U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_ADDR1                                   (0x00400064U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_DESC0                                   (0x00400068U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_DESC1                                   (0x0040006CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_DESC3                                   (0x00400074U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_AXI_ADDR0                               (0x00400078U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_AXI_ADDR1                               (0x0040007CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_ADDR0                                   (0x00400080U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_ADDR1                                   (0x00400084U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_DESC0                                   (0x00400088U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_DESC1                                   (0x0040008CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_DESC3                                   (0x00400094U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_AXI_ADDR0                               (0x00400098U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_AXI_ADDR1                               (0x0040009CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_ADDR0                                   (0x004000A0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_ADDR1                                   (0x004000A4U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_DESC0                                   (0x004000A8U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_DESC1                                   (0x004000ACU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_DESC3                                   (0x004000B4U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_AXI_ADDR0                               (0x004000B8U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_AXI_ADDR1                               (0x004000BCU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_ADDR0                                   (0x004000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_ADDR1                                   (0x004000C4U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_DESC0                                   (0x004000C8U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_DESC1                                   (0x004000CCU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_DESC3                                   (0x004000D4U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_AXI_ADDR0                               (0x004000D8U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_AXI_ADDR1                               (0x004000DCU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_ADDR0                                   (0x004000E0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_ADDR1                                   (0x004000E4U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_DESC0                                   (0x004000E8U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_DESC1                                   (0x004000ECU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_DESC3                                   (0x004000F4U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_AXI_ADDR0                               (0x004000F8U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_AXI_ADDR1                               (0x004000FCU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_ADDR0                                   (0x00400100U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_ADDR1                                   (0x00400104U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_DESC0                                   (0x00400108U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_DESC1                                   (0x0040010CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_DESC3                                   (0x00400114U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_AXI_ADDR0                               (0x00400118U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_AXI_ADDR1                               (0x0040011CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_ADDR0                                   (0x00400120U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_ADDR1                                   (0x00400124U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_DESC0                                   (0x00400128U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_DESC1                                   (0x0040012CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_DESC3                                   (0x00400134U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_AXI_ADDR0                               (0x00400138U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_AXI_ADDR1                               (0x0040013CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_ADDR0                                  (0x00400140U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_ADDR1                                  (0x00400144U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_DESC0                                  (0x00400148U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_DESC1                                  (0x0040014CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_DESC3                                  (0x00400154U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_AXI_ADDR0                              (0x00400158U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_AXI_ADDR1                              (0x0040015CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_ADDR0                                  (0x00400160U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_ADDR1                                  (0x00400164U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_DESC0                                  (0x00400168U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_DESC1                                  (0x0040016CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_DESC3                                  (0x00400174U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_AXI_ADDR0                              (0x00400178U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_AXI_ADDR1                              (0x0040017CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_ADDR0                                  (0x00400180U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_ADDR1                                  (0x00400184U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_DESC0                                  (0x00400188U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_DESC1                                  (0x0040018CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_DESC3                                  (0x00400194U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_AXI_ADDR0                              (0x00400198U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_AXI_ADDR1                              (0x0040019CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_ADDR0                                  (0x004001A0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_ADDR1                                  (0x004001A4U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_DESC0                                  (0x004001A8U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_DESC1                                  (0x004001ACU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_DESC3                                  (0x004001B4U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_AXI_ADDR0                              (0x004001B8U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_AXI_ADDR1                              (0x004001BCU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_ADDR0                                  (0x004001C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_ADDR1                                  (0x004001C4U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_DESC0                                  (0x004001C8U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_DESC1                                  (0x004001CCU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_DESC3                                  (0x004001D4U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_AXI_ADDR0                              (0x004001D8U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_AXI_ADDR1                              (0x004001DCU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_ADDR0                                  (0x004001E0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_ADDR1                                  (0x004001E4U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_DESC0                                  (0x004001E8U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_DESC1                                  (0x004001ECU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_DESC3                                  (0x004001F4U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_AXI_ADDR0                              (0x004001F8U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_AXI_ADDR1                              (0x004001FCU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_ADDR0                                  (0x00400200U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_ADDR1                                  (0x00400204U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_DESC0                                  (0x00400208U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_DESC1                                  (0x0040020CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_DESC3                                  (0x00400214U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_AXI_ADDR0                              (0x00400218U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_AXI_ADDR1                              (0x0040021CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_ADDR0                                  (0x00400220U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_ADDR1                                  (0x00400224U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_DESC0                                  (0x00400228U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_DESC1                                  (0x0040022CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_DESC3                                  (0x00400234U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_AXI_ADDR0                              (0x00400238U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_AXI_ADDR1                              (0x0040023CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_ADDR0                                  (0x00400240U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_ADDR1                                  (0x00400244U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_DESC0                                  (0x00400248U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_DESC1                                  (0x0040024CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_DESC3                                  (0x00400254U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_AXI_ADDR0                              (0x00400258U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_AXI_ADDR1                              (0x0040025CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_ADDR0                                  (0x00400260U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_ADDR1                                  (0x00400264U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_DESC0                                  (0x00400268U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_DESC1                                  (0x0040026CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_DESC3                                  (0x00400274U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_AXI_ADDR0                              (0x00400278U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_AXI_ADDR1                              (0x0040027CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_ADDR0                                  (0x00400280U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_ADDR1                                  (0x00400284U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_DESC0                                  (0x00400288U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_DESC1                                  (0x0040028CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_DESC3                                  (0x00400294U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_AXI_ADDR0                              (0x00400298U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_AXI_ADDR1                              (0x0040029CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_ADDR0                                  (0x004002A0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_ADDR1                                  (0x004002A4U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_DESC0                                  (0x004002A8U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_DESC1                                  (0x004002ACU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_DESC3                                  (0x004002B4U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_AXI_ADDR0                              (0x004002B8U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_AXI_ADDR1                              (0x004002BCU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_ADDR0                                  (0x004002C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_ADDR1                                  (0x004002C4U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_DESC0                                  (0x004002C8U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_DESC1                                  (0x004002CCU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_DESC3                                  (0x004002D4U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_AXI_ADDR0                              (0x004002D8U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_AXI_ADDR1                              (0x004002DCU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_ADDR0                                  (0x004002E0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_ADDR1                                  (0x004002E4U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_DESC0                                  (0x004002E8U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_DESC1                                  (0x004002ECU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_DESC3                                  (0x004002F4U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_AXI_ADDR0                              (0x004002F8U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_AXI_ADDR1                              (0x004002FCU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_ADDR0                                  (0x00400300U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_ADDR1                                  (0x00400304U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_DESC0                                  (0x00400308U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_DESC1                                  (0x0040030CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_DESC3                                  (0x00400314U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_AXI_ADDR0                              (0x00400318U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_AXI_ADDR1                              (0x0040031CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_ADDR0                                  (0x00400320U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_ADDR1                                  (0x00400324U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_DESC0                                  (0x00400328U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_DESC1                                  (0x0040032CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_DESC3                                  (0x00400334U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_AXI_ADDR0                              (0x00400338U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_AXI_ADDR1                              (0x0040033CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_ADDR0                                  (0x00400340U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_ADDR1                                  (0x00400344U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_DESC0                                  (0x00400348U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_DESC1                                  (0x0040034CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_DESC3                                  (0x00400354U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_AXI_ADDR0                              (0x00400358U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_AXI_ADDR1                              (0x0040035CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_ADDR0                                  (0x00400360U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_ADDR1                                  (0x00400364U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_DESC0                                  (0x00400368U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_DESC1                                  (0x0040036CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_DESC3                                  (0x00400374U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_AXI_ADDR0                              (0x00400378U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_AXI_ADDR1                              (0x0040037CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_ADDR0                                  (0x00400380U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_ADDR1                                  (0x00400384U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_DESC0                                  (0x00400388U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_DESC1                                  (0x0040038CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_DESC3                                  (0x00400394U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_AXI_ADDR0                              (0x00400398U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_AXI_ADDR1                              (0x0040039CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_ADDR0                                  (0x004003A0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_ADDR1                                  (0x004003A4U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_DESC0                                  (0x004003A8U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_DESC1                                  (0x004003ACU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_DESC3                                  (0x004003B4U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_AXI_ADDR0                              (0x004003B8U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_AXI_ADDR1                              (0x004003BCU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_ADDR0                                  (0x004003C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_ADDR1                                  (0x004003C4U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_DESC0                                  (0x004003C8U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_DESC1                                  (0x004003CCU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_DESC3                                  (0x004003D4U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_AXI_ADDR0                              (0x004003D8U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_AXI_ADDR1                              (0x004003DCU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_ADDR0                                  (0x004003E0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_ADDR1                                  (0x004003E4U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_DESC0                                  (0x004003E8U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_DESC1                                  (0x004003ECU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_DESC3                                  (0x004003F4U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_AXI_ADDR0                              (0x004003F8U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_AXI_ADDR1                              (0x004003FCU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_ADDR0                                (0x00400400U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_ADDR1                                (0x00400404U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_DESC0                                (0x00400408U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_DESC1                                (0x0040040CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_DESC3                                (0x00400414U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_AXI_ADDR0                            (0x00400418U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_AXI_ADDR1                            (0x0040041CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_ADDR0                                (0x00400420U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_ADDR1                                (0x00400424U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_DESC0                                (0x00400428U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_DESC1                                (0x0040042CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_DESC3                                (0x00400434U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_AXI_ADDR0                            (0x00400438U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_AXI_ADDR1                            (0x0040043CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_ADDR0                                (0x00400440U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_ADDR1                                (0x00400444U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_DESC0                                (0x00400448U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_DESC1                                (0x0040044CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_DESC3                                (0x00400454U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_AXI_ADDR0                            (0x00400458U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_AXI_ADDR1                            (0x0040045CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_ADDR0                                (0x00400460U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_ADDR1                                (0x00400464U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_DESC0                                (0x00400468U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_DESC1                                (0x0040046CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_DESC3                                (0x00400474U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_AXI_ADDR0                            (0x00400478U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_AXI_ADDR1                            (0x0040047CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_ADDR0                                (0x00400480U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_ADDR1                                (0x00400484U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_DESC0                                (0x00400488U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_DESC1                                (0x0040048CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_DESC3                                (0x00400494U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_AXI_ADDR0                            (0x00400498U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_AXI_ADDR1                            (0x0040049CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_ADDR0                                (0x004004A0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_ADDR1                                (0x004004A4U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_DESC0                                (0x004004A8U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_DESC1                                (0x004004ACU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_DESC3                                (0x004004B4U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_AXI_ADDR0                            (0x004004B8U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_AXI_ADDR1                            (0x004004BCU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_ADDR0                                (0x004004C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_ADDR1                                (0x004004C4U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_DESC0                                (0x004004C8U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_DESC1                                (0x004004CCU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_DESC3                                (0x004004D4U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_AXI_ADDR0                            (0x004004D8U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_AXI_ADDR1                            (0x004004DCU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_ADDR0                                (0x004004E0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_ADDR1                                (0x004004E4U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_DESC0                                (0x004004E8U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_DESC1                                (0x004004ECU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_DESC3                                (0x004004F4U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_AXI_ADDR0                            (0x004004F8U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_AXI_ADDR1                            (0x004004FCU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_ADDR0                                (0x00400500U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_ADDR1                                (0x00400504U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_DESC0                                (0x00400508U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_DESC1                                (0x0040050CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_DESC3                                (0x00400514U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_AXI_ADDR0                            (0x00400518U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_AXI_ADDR1                            (0x0040051CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_ADDR0                                (0x00400520U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_ADDR1                                (0x00400524U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_DESC0                                (0x00400528U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_DESC1                                (0x0040052CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_DESC3                                (0x00400534U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_AXI_ADDR0                            (0x00400538U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_AXI_ADDR1                            (0x0040053CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_ADDR0                               (0x00400540U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_ADDR1                               (0x00400544U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_DESC0                               (0x00400548U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_DESC1                               (0x0040054CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_DESC3                               (0x00400554U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_AXI_ADDR0                           (0x00400558U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_AXI_ADDR1                           (0x0040055CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_ADDR0                               (0x00400560U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_ADDR1                               (0x00400564U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_DESC0                               (0x00400568U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_DESC1                               (0x0040056CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_DESC3                               (0x00400574U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_AXI_ADDR0                           (0x00400578U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_AXI_ADDR1                           (0x0040057CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_ADDR0                               (0x00400580U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_ADDR1                               (0x00400584U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_DESC0                               (0x00400588U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_DESC1                               (0x0040058CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_DESC3                               (0x00400594U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_AXI_ADDR0                           (0x00400598U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_AXI_ADDR1                           (0x0040059CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_ADDR0                               (0x004005A0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_ADDR1                               (0x004005A4U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_DESC0                               (0x004005A8U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_DESC1                               (0x004005ACU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_DESC3                               (0x004005B4U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_AXI_ADDR0                           (0x004005B8U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_AXI_ADDR1                           (0x004005BCU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_ADDR0                               (0x004005C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_ADDR1                               (0x004005C4U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_DESC0                               (0x004005C8U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_DESC1                               (0x004005CCU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_DESC3                               (0x004005D4U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_AXI_ADDR0                           (0x004005D8U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_AXI_ADDR1                           (0x004005DCU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_ADDR0                               (0x004005E0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_ADDR1                               (0x004005E4U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_DESC0                               (0x004005E8U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_DESC1                               (0x004005ECU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_DESC3                               (0x004005F4U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_AXI_ADDR0                           (0x004005F8U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_AXI_ADDR1                           (0x004005FCU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_ADDR0                               (0x00400600U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_ADDR1                               (0x00400604U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_DESC0                               (0x00400608U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_DESC1                               (0x0040060CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_DESC3                               (0x00400614U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_AXI_ADDR0                           (0x00400618U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_AXI_ADDR1                           (0x0040061CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_ADDR0                               (0x00400620U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_ADDR1                               (0x00400624U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_DESC0                               (0x00400628U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_DESC1                               (0x0040062CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_DESC3                               (0x00400634U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_AXI_ADDR0                           (0x00400638U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_AXI_ADDR1                           (0x0040063CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_ADDR0                               (0x00400640U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_ADDR1                               (0x00400644U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_DESC0                               (0x00400648U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_DESC1                               (0x0040064CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_DESC3                               (0x00400654U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_AXI_ADDR0                           (0x00400658U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_AXI_ADDR1                           (0x0040065CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_ADDR0                               (0x00400660U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_ADDR1                               (0x00400664U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_DESC0                               (0x00400668U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_DESC1                               (0x0040066CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_DESC3                               (0x00400674U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_AXI_ADDR0                           (0x00400678U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_AXI_ADDR1                           (0x0040067CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_ADDR0                               (0x00400680U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_ADDR1                               (0x00400684U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_DESC0                               (0x00400688U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_DESC1                               (0x0040068CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_DESC3                               (0x00400694U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_AXI_ADDR0                           (0x00400698U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_AXI_ADDR1                           (0x0040069CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_ADDR0                               (0x004006A0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_ADDR1                               (0x004006A4U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_DESC0                               (0x004006A8U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_DESC1                               (0x004006ACU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_DESC3                               (0x004006B4U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_AXI_ADDR0                           (0x004006B8U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_AXI_ADDR1                           (0x004006BCU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_ADDR0                               (0x004006C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_ADDR1                               (0x004006C4U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_DESC0                               (0x004006C8U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_DESC1                               (0x004006CCU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_DESC3                               (0x004006D4U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_AXI_ADDR0                           (0x004006D8U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_AXI_ADDR1                           (0x004006DCU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_ADDR0                               (0x004006E0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_ADDR1                               (0x004006E4U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_DESC0                               (0x004006E8U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_DESC1                               (0x004006ECU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_DESC3                               (0x004006F4U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_AXI_ADDR0                           (0x004006F8U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_AXI_ADDR1                           (0x004006FCU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_ADDR0                               (0x00400700U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_ADDR1                               (0x00400704U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_DESC0                               (0x00400708U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_DESC1                               (0x0040070CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_DESC3                               (0x00400714U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_AXI_ADDR0                           (0x00400718U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_AXI_ADDR1                           (0x0040071CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_ADDR0                               (0x00400720U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_ADDR1                               (0x00400724U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_DESC0                               (0x00400728U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_DESC1                               (0x0040072CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_DESC3                               (0x00400734U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_AXI_ADDR0                           (0x00400738U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_AXI_ADDR1                           (0x0040073CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_ADDR0                               (0x00400740U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_ADDR1                               (0x00400744U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_DESC0                               (0x00400748U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_DESC1                               (0x0040074CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_DESC3                               (0x00400754U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_AXI_ADDR0                           (0x00400758U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_AXI_ADDR1                           (0x0040075CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_ADDR0                               (0x00400760U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_ADDR1                               (0x00400764U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_DESC0                               (0x00400768U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_DESC1                               (0x0040076CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_DESC3                               (0x00400774U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_AXI_ADDR0                           (0x00400778U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_AXI_ADDR1                           (0x0040077CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_ADDR0                               (0x00400780U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_ADDR1                               (0x00400784U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_DESC0                               (0x00400788U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_DESC1                               (0x0040078CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_DESC3                               (0x00400794U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_AXI_ADDR0                           (0x00400798U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_AXI_ADDR1                           (0x0040079CU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_ADDR0                               (0x004007A0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_ADDR1                               (0x004007A4U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_DESC0                               (0x004007A8U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_DESC1                               (0x004007ACU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_DESC3                               (0x004007B4U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_AXI_ADDR0                           (0x004007B8U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_AXI_ADDR1                           (0x004007BCU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_ADDR0                               (0x004007C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_ADDR1                               (0x004007C4U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_DESC0                               (0x004007C8U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_DESC1                               (0x004007CCU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_DESC3                               (0x004007D4U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_AXI_ADDR0                           (0x004007D8U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_AXI_ADDR1                           (0x004007DCU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_ADDR0                               (0x004007E0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_ADDR1                               (0x004007E4U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_DESC0                               (0x004007E8U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_DESC1                               (0x004007ECU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_DESC3                               (0x004007F4U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_AXI_ADDR0                           (0x004007F8U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_AXI_ADDR1                           (0x004007FCU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_0_ADDR0                                   (0x00400800U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_0_ADDR1                                   (0x00400804U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_1_ADDR0                                   (0x00400808U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_1_ADDR1                                   (0x0040080CU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_2_ADDR0                                   (0x00400810U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_2_ADDR1                                   (0x00400814U)
#define CSL_PCIE_RP_CORE_ATU_CREDIT_THRESHOLD_C0                                  (0x00400820U)
#define CSL_PCIE_RP_CORE_ATU_LINK_DOWN_INDICATOR_BIT_L0                           (0x00400824U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_0_ADDR0                          (0x00400840U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_0_ADDR1                          (0x00400844U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_1_ADDR0                          (0x00400848U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_1_ADDR1                          (0x0040084CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_2_ADDR0                          (0x00400850U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_2_ADDR1                          (0x00400854U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_3_ADDR0                          (0x00400858U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_3_ADDR1                          (0x0040085CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_4_ADDR0                          (0x00400860U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_4_ADDR1                          (0x00400864U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_5_ADDR0                          (0x00400868U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_5_ADDR1                          (0x0040086CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_6_ADDR0                          (0x00400870U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_6_ADDR1                          (0x00400874U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_7_ADDR0                          (0x00400878U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_7_ADDR1                          (0x0040087CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_0_ADDR0                          (0x00400880U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_0_ADDR1                          (0x00400884U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_1_ADDR0                          (0x00400888U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_1_ADDR1                          (0x0040088CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_2_ADDR0                          (0x00400890U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_2_ADDR1                          (0x00400894U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_3_ADDR0                          (0x00400898U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_3_ADDR1                          (0x0040089CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_4_ADDR0                          (0x004008A0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_4_ADDR1                          (0x004008A4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_5_ADDR0                          (0x004008A8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_5_ADDR1                          (0x004008ACU)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_6_ADDR0                          (0x004008B0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_6_ADDR1                          (0x004008B4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_7_ADDR0                          (0x004008B8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_7_ADDR1                          (0x004008BCU)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_0_ADDR0                          (0x004008C0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_0_ADDR1                          (0x004008C4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_1_ADDR0                          (0x004008C8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_1_ADDR1                          (0x004008CCU)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_2_ADDR0                          (0x004008D0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_2_ADDR1                          (0x004008D4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_3_ADDR0                          (0x004008D8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_3_ADDR1                          (0x004008DCU)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_4_ADDR0                          (0x004008E0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_4_ADDR1                          (0x004008E4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_5_ADDR0                          (0x004008E8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_5_ADDR1                          (0x004008ECU)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_6_ADDR0                          (0x004008F0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_6_ADDR1                          (0x004008F4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_7_ADDR0                          (0x004008F8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_7_ADDR1                          (0x004008FCU)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_0_ADDR0                          (0x00400900U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_0_ADDR1                          (0x00400904U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_1_ADDR0                          (0x00400908U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_1_ADDR1                          (0x0040090CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_2_ADDR0                          (0x00400910U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_2_ADDR1                          (0x00400914U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_3_ADDR0                          (0x00400918U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_3_ADDR1                          (0x0040091CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_4_ADDR0                          (0x00400920U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_4_ADDR1                          (0x00400924U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_5_ADDR0                          (0x00400928U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_5_ADDR1                          (0x0040092CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_6_ADDR0                          (0x00400930U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_6_ADDR1                          (0x00400934U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_7_ADDR0                          (0x00400938U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_7_ADDR1                          (0x0040093CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_0_ADDR0                          (0x00400940U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_0_ADDR1                          (0x00400944U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_1_ADDR0                          (0x00400948U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_1_ADDR1                          (0x0040094CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_2_ADDR0                          (0x00400950U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_2_ADDR1                          (0x00400954U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_3_ADDR0                          (0x00400958U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_3_ADDR1                          (0x0040095CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_4_ADDR0                          (0x00400960U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_4_ADDR1                          (0x00400964U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_5_ADDR0                          (0x00400968U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_5_ADDR1                          (0x0040096CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_6_ADDR0                          (0x00400970U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_6_ADDR1                          (0x00400974U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_7_ADDR0                          (0x00400978U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_7_ADDR1                          (0x0040097CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_0_ADDR0                          (0x00400980U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_0_ADDR1                          (0x00400984U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_1_ADDR0                          (0x00400988U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_1_ADDR1                          (0x0040098CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_2_ADDR0                          (0x00400990U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_2_ADDR1                          (0x00400994U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_3_ADDR0                          (0x00400998U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_3_ADDR1                          (0x0040099CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_4_ADDR0                          (0x004009A0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_4_ADDR1                          (0x004009A4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_5_ADDR0                          (0x004009A8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_5_ADDR1                          (0x004009ACU)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_6_ADDR0                          (0x004009B0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_6_ADDR1                          (0x004009B4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_7_ADDR0                          (0x004009B8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_7_ADDR1                          (0x004009BCU)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_0_ADDR0                          (0x004009C0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_0_ADDR1                          (0x004009C4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_1_ADDR0                          (0x004009C8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_1_ADDR1                          (0x004009CCU)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_2_ADDR0                          (0x004009D0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_2_ADDR1                          (0x004009D4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_3_ADDR0                          (0x004009D8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_3_ADDR1                          (0x004009DCU)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_4_ADDR0                          (0x004009E0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_4_ADDR1                          (0x004009E4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_5_ADDR0                          (0x004009E8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_5_ADDR1                          (0x004009ECU)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_6_ADDR0                          (0x004009F0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_6_ADDR1                          (0x004009F4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_7_ADDR0                          (0x004009F8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_7_ADDR1                          (0x004009FCU)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_0_ADDR0                          (0x00400A00U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_0_ADDR1                          (0x00400A04U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_1_ADDR0                          (0x00400A08U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_1_ADDR1                          (0x00400A0CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_2_ADDR0                          (0x00400A10U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_2_ADDR1                          (0x00400A14U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_3_ADDR0                          (0x00400A18U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_3_ADDR1                          (0x00400A1CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_4_ADDR0                          (0x00400A20U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_4_ADDR1                          (0x00400A24U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_5_ADDR0                          (0x00400A28U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_5_ADDR1                          (0x00400A2CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_6_ADDR0                          (0x00400A30U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_6_ADDR1                          (0x00400A34U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_7_ADDR0                          (0x00400A38U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_7_ADDR1                          (0x00400A3CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_0_ADDR0                          (0x00400A40U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_0_ADDR1                          (0x00400A44U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_1_ADDR0                          (0x00400A48U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_1_ADDR1                          (0x00400A4CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_2_ADDR0                          (0x00400A50U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_2_ADDR1                          (0x00400A54U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_3_ADDR0                          (0x00400A58U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_3_ADDR1                          (0x00400A5CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_4_ADDR0                          (0x00400A60U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_4_ADDR1                          (0x00400A64U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_5_ADDR0                          (0x00400A68U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_5_ADDR1                          (0x00400A6CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_6_ADDR0                          (0x00400A70U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_6_ADDR1                          (0x00400A74U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_7_ADDR0                          (0x00400A78U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_7_ADDR1                          (0x00400A7CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_0_ADDR0                          (0x00400A80U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_0_ADDR1                          (0x00400A84U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_1_ADDR0                          (0x00400A88U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_1_ADDR1                          (0x00400A8CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_2_ADDR0                          (0x00400A90U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_2_ADDR1                          (0x00400A94U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_3_ADDR0                          (0x00400A98U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_3_ADDR1                          (0x00400A9CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_4_ADDR0                          (0x00400AA0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_4_ADDR1                          (0x00400AA4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_5_ADDR0                          (0x00400AA8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_5_ADDR1                          (0x00400AACU)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_6_ADDR0                          (0x00400AB0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_6_ADDR1                          (0x00400AB4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_7_ADDR0                          (0x00400AB8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_7_ADDR1                          (0x00400ABCU)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_0_ADDR0                         (0x00400AC0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_0_ADDR1                         (0x00400AC4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_1_ADDR0                         (0x00400AC8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_1_ADDR1                         (0x00400ACCU)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_2_ADDR0                         (0x00400AD0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_2_ADDR1                         (0x00400AD4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_3_ADDR0                         (0x00400AD8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_3_ADDR1                         (0x00400ADCU)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_4_ADDR0                         (0x00400AE0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_4_ADDR1                         (0x00400AE4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_5_ADDR0                         (0x00400AE8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_5_ADDR1                         (0x00400AECU)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_6_ADDR0                         (0x00400AF0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_6_ADDR1                         (0x00400AF4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_7_ADDR0                         (0x00400AF8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_7_ADDR1                         (0x00400AFCU)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_0_ADDR0                         (0x00400B00U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_0_ADDR1                         (0x00400B04U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_1_ADDR0                         (0x00400B08U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_1_ADDR1                         (0x00400B0CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_2_ADDR0                         (0x00400B10U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_2_ADDR1                         (0x00400B14U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_3_ADDR0                         (0x00400B18U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_3_ADDR1                         (0x00400B1CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_4_ADDR0                         (0x00400B20U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_4_ADDR1                         (0x00400B24U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_5_ADDR0                         (0x00400B28U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_5_ADDR1                         (0x00400B2CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_6_ADDR0                         (0x00400B30U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_6_ADDR1                         (0x00400B34U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_7_ADDR0                         (0x00400B38U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_7_ADDR1                         (0x00400B3CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_0_ADDR0                         (0x00400B40U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_0_ADDR1                         (0x00400B44U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_1_ADDR0                         (0x00400B48U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_1_ADDR1                         (0x00400B4CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_2_ADDR0                         (0x00400B50U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_2_ADDR1                         (0x00400B54U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_3_ADDR0                         (0x00400B58U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_3_ADDR1                         (0x00400B5CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_4_ADDR0                         (0x00400B60U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_4_ADDR1                         (0x00400B64U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_5_ADDR0                         (0x00400B68U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_5_ADDR1                         (0x00400B6CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_6_ADDR0                         (0x00400B70U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_6_ADDR1                         (0x00400B74U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_7_ADDR0                         (0x00400B78U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_7_ADDR1                         (0x00400B7CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_0_ADDR0                         (0x00400B80U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_0_ADDR1                         (0x00400B84U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_1_ADDR0                         (0x00400B88U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_1_ADDR1                         (0x00400B8CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_2_ADDR0                         (0x00400B90U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_2_ADDR1                         (0x00400B94U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_3_ADDR0                         (0x00400B98U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_3_ADDR1                         (0x00400B9CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_4_ADDR0                         (0x00400BA0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_4_ADDR1                         (0x00400BA4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_5_ADDR0                         (0x00400BA8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_5_ADDR1                         (0x00400BACU)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_6_ADDR0                         (0x00400BB0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_6_ADDR1                         (0x00400BB4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_7_ADDR0                         (0x00400BB8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_7_ADDR1                         (0x00400BBCU)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_0_ADDR0                         (0x00400BC0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_0_ADDR1                         (0x00400BC4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_1_ADDR0                         (0x00400BC8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_1_ADDR1                         (0x00400BCCU)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_2_ADDR0                         (0x00400BD0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_2_ADDR1                         (0x00400BD4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_3_ADDR0                         (0x00400BD8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_3_ADDR1                         (0x00400BDCU)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_4_ADDR0                         (0x00400BE0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_4_ADDR1                         (0x00400BE4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_5_ADDR0                         (0x00400BE8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_5_ADDR1                         (0x00400BECU)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_6_ADDR0                         (0x00400BF0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_6_ADDR1                         (0x00400BF4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_7_ADDR0                         (0x00400BF8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_7_ADDR1                         (0x00400BFCU)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_0_ADDR0                         (0x00400C00U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_0_ADDR1                         (0x00400C04U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_1_ADDR0                         (0x00400C08U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_1_ADDR1                         (0x00400C0CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_2_ADDR0                         (0x00400C10U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_2_ADDR1                         (0x00400C14U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_3_ADDR0                         (0x00400C18U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_3_ADDR1                         (0x00400C1CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_4_ADDR0                         (0x00400C20U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_4_ADDR1                         (0x00400C24U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_5_ADDR0                         (0x00400C28U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_5_ADDR1                         (0x00400C2CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_6_ADDR0                         (0x00400C30U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_6_ADDR1                         (0x00400C34U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_7_ADDR0                         (0x00400C38U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_7_ADDR1                         (0x00400C3CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_0_ADDR0                         (0x00400C40U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_0_ADDR1                         (0x00400C44U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_1_ADDR0                         (0x00400C48U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_1_ADDR1                         (0x00400C4CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_2_ADDR0                         (0x00400C50U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_2_ADDR1                         (0x00400C54U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_3_ADDR0                         (0x00400C58U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_3_ADDR1                         (0x00400C5CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_4_ADDR0                         (0x00400C60U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_4_ADDR1                         (0x00400C64U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_5_ADDR0                         (0x00400C68U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_5_ADDR1                         (0x00400C6CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_6_ADDR0                         (0x00400C70U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_6_ADDR1                         (0x00400C74U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_7_ADDR0                         (0x00400C78U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_7_ADDR1                         (0x00400C7CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_0_ADDR0                         (0x00400C80U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_0_ADDR1                         (0x00400C84U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_1_ADDR0                         (0x00400C88U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_1_ADDR1                         (0x00400C8CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_2_ADDR0                         (0x00400C90U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_2_ADDR1                         (0x00400C94U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_3_ADDR0                         (0x00400C98U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_3_ADDR1                         (0x00400C9CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_4_ADDR0                         (0x00400CA0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_4_ADDR1                         (0x00400CA4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_5_ADDR0                         (0x00400CA8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_5_ADDR1                         (0x00400CACU)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_6_ADDR0                         (0x00400CB0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_6_ADDR1                         (0x00400CB4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_7_ADDR0                         (0x00400CB8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_7_ADDR1                         (0x00400CBCU)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_0_ADDR0                         (0x00400CC0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_0_ADDR1                         (0x00400CC4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_1_ADDR0                         (0x00400CC8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_1_ADDR1                         (0x00400CCCU)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_2_ADDR0                         (0x00400CD0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_2_ADDR1                         (0x00400CD4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_3_ADDR0                         (0x00400CD8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_3_ADDR1                         (0x00400CDCU)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_4_ADDR0                         (0x00400CE0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_4_ADDR1                         (0x00400CE4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_5_ADDR0                         (0x00400CE8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_5_ADDR1                         (0x00400CECU)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_6_ADDR0                         (0x00400CF0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_6_ADDR1                         (0x00400CF4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_7_ADDR0                         (0x00400CF8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_7_ADDR1                         (0x00400CFCU)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_0_ADDR0                         (0x00400D00U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_0_ADDR1                         (0x00400D04U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_1_ADDR0                         (0x00400D08U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_1_ADDR1                         (0x00400D0CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_2_ADDR0                         (0x00400D10U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_2_ADDR1                         (0x00400D14U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_3_ADDR0                         (0x00400D18U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_3_ADDR1                         (0x00400D1CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_4_ADDR0                         (0x00400D20U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_4_ADDR1                         (0x00400D24U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_5_ADDR0                         (0x00400D28U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_5_ADDR1                         (0x00400D2CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_6_ADDR0                         (0x00400D30U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_6_ADDR1                         (0x00400D34U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_7_ADDR0                         (0x00400D38U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_7_ADDR1                         (0x00400D3CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_0_ADDR0                         (0x00400D40U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_0_ADDR1                         (0x00400D44U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_1_ADDR0                         (0x00400D48U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_1_ADDR1                         (0x00400D4CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_2_ADDR0                         (0x00400D50U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_2_ADDR1                         (0x00400D54U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_3_ADDR0                         (0x00400D58U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_3_ADDR1                         (0x00400D5CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_4_ADDR0                         (0x00400D60U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_4_ADDR1                         (0x00400D64U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_5_ADDR0                         (0x00400D68U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_5_ADDR1                         (0x00400D6CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_6_ADDR0                         (0x00400D70U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_6_ADDR1                         (0x00400D74U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_7_ADDR0                         (0x00400D78U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_7_ADDR1                         (0x00400D7CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_0_ADDR0                         (0x00400D80U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_0_ADDR1                         (0x00400D84U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_1_ADDR0                         (0x00400D88U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_1_ADDR1                         (0x00400D8CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_2_ADDR0                         (0x00400D90U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_2_ADDR1                         (0x00400D94U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_3_ADDR0                         (0x00400D98U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_3_ADDR1                         (0x00400D9CU)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_4_ADDR0                         (0x00400DA0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_4_ADDR1                         (0x00400DA4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_5_ADDR0                         (0x00400DA8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_5_ADDR1                         (0x00400DACU)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_6_ADDR0                         (0x00400DB0U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_6_ADDR1                         (0x00400DB4U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_7_ADDR0                         (0x00400DB8U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_7_ADDR1                         (0x00400DBCU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* I_VENDOR_ID_DEVICE_ID */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_VENDOR_ID_DEVICE_ID_VID_MASK         (0x0000FFFFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_VENDOR_ID_DEVICE_ID_VID_SHIFT        (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_VENDOR_ID_DEVICE_ID_VID_MAX          (0x0000FFFFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_VENDOR_ID_DEVICE_ID_DID_MASK         (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_VENDOR_ID_DEVICE_ID_DID_SHIFT        (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_VENDOR_ID_DEVICE_ID_DID_MAX          (0x0000FFFFU)

/* I_COMMAND_STATUS */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_ISE_MASK              (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_ISE_SHIFT             (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_ISE_MAX               (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_MSE_MASK              (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_MSE_SHIFT             (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_MSE_MAX               (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_BE_MASK               (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_BE_SHIFT              (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_BE_MAX                (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_R0_MASK               (0x00000038U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_R0_SHIFT              (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_R0_MAX                (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_PERE_MASK             (0x00000040U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_PERE_SHIFT            (0x00000006U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_PERE_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_R1_MASK               (0x00000080U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_R1_SHIFT              (0x00000007U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_R1_MAX                (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_SE_MASK               (0x00000100U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_SE_SHIFT              (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_SE_MAX                (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_R2_MASK               (0x00000200U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_R2_SHIFT              (0x00000009U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_R2_MAX                (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_IMD_MASK              (0x00000400U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_IMD_SHIFT             (0x0000000AU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_IMD_MAX               (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_R3_MASK               (0x0000F800U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_R3_SHIFT              (0x0000000BU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_R3_MAX                (0x0000001FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_R4_MASK               (0x00070000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_R4_SHIFT              (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_R4_MAX                (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_IS_MASK               (0x00080000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_IS_SHIFT              (0x00000013U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_IS_MAX                (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_CL_MASK               (0x00100000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_CL_SHIFT              (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_CL_MAX                (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_R5_MASK               (0x00E00000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_R5_SHIFT              (0x00000015U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_R5_MAX                (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_MDPE_MASK             (0x01000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_MDPE_SHIFT            (0x00000018U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_MDPE_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_R6_MASK               (0x06000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_R6_SHIFT              (0x00000019U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_R6_MAX                (0x00000003U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_STA_MASK              (0x08000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_STA_SHIFT             (0x0000001BU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_STA_MAX               (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_RTA_MASK              (0x10000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_RTA_SHIFT             (0x0000001CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_RTA_MAX               (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_RMA_MASK              (0x20000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_RMA_SHIFT             (0x0000001DU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_RMA_MAX               (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_SSE_MASK              (0x40000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_SSE_SHIFT             (0x0000001EU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_SSE_MAX               (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_DPE_MASK              (0x80000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_DPE_SHIFT             (0x0000001FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_COMMAND_STATUS_DPE_MAX               (0x00000001U)

/* I_REVISION_ID_CLASS_CODE */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_REVISION_ID_CLASS_CODE_RID_MASK      (0x000000FFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_REVISION_ID_CLASS_CODE_RID_SHIFT     (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_REVISION_ID_CLASS_CODE_RID_MAX       (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_REVISION_ID_CLASS_CODE_PIB_MASK      (0x0000FF00U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_REVISION_ID_CLASS_CODE_PIB_SHIFT     (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_REVISION_ID_CLASS_CODE_PIB_MAX       (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_REVISION_ID_CLASS_CODE_SCC_MASK      (0x00FF0000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_REVISION_ID_CLASS_CODE_SCC_SHIFT     (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_REVISION_ID_CLASS_CODE_SCC_MAX       (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_REVISION_ID_CLASS_CODE_CC_MASK       (0xFF000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_REVISION_ID_CLASS_CODE_CC_SHIFT      (0x00000018U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_REVISION_ID_CLASS_CODE_CC_MAX        (0x000000FFU)

/* I_BIST_HEADER_LATENCY_CACHE_LINE */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_BIST_HEADER_LATENCY_CACHE_LINE_CLS_MASK (0x000000FFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_BIST_HEADER_LATENCY_CACHE_LINE_CLS_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_BIST_HEADER_LATENCY_CACHE_LINE_CLS_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_BIST_HEADER_LATENCY_CACHE_LINE_LT_MASK (0x0000FF00U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_BIST_HEADER_LATENCY_CACHE_LINE_LT_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_BIST_HEADER_LATENCY_CACHE_LINE_LT_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_BIST_HEADER_LATENCY_CACHE_LINE_HT_MASK (0x007F0000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_BIST_HEADER_LATENCY_CACHE_LINE_HT_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_BIST_HEADER_LATENCY_CACHE_LINE_HT_MAX (0x0000007FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_BIST_HEADER_LATENCY_CACHE_LINE_DT_MASK (0x00800000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_BIST_HEADER_LATENCY_CACHE_LINE_DT_SHIFT (0x00000017U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_BIST_HEADER_LATENCY_CACHE_LINE_DT_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_BIST_HEADER_LATENCY_CACHE_LINE_BR_MASK (0xFF000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_BIST_HEADER_LATENCY_CACHE_LINE_BR_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_BIST_HEADER_LATENCY_CACHE_LINE_BR_MAX (0x000000FFU)

/* I_RC_BAR_0 */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_RC_BAR_0_MSI0_MASK                   (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_RC_BAR_0_MSI0_SHIFT                  (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_RC_BAR_0_MSI0_MAX                    (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_RC_BAR_0_R7_MASK                     (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_RC_BAR_0_R7_SHIFT                    (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_RC_BAR_0_R7_MAX                      (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_RC_BAR_0_S0_MASK                     (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_RC_BAR_0_S0_SHIFT                    (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_RC_BAR_0_S0_MAX                      (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_RC_BAR_0_P0_MASK                     (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_RC_BAR_0_P0_SHIFT                    (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_RC_BAR_0_P0_MAX                      (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_RC_BAR_0_BAMR0_MASK                  (0x003FFFF0U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_RC_BAR_0_BAMR0_SHIFT                 (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_RC_BAR_0_BAMR0_MAX                   (0x0003FFFFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_RC_BAR_0_BAMRW_MASK                  (0xFFC00000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_RC_BAR_0_BAMRW_SHIFT                 (0x00000016U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_RC_BAR_0_BAMRW_MAX                   (0x000003FFU)

/* I_RC_BAR_1 */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_RC_BAR_1_R7_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_RC_BAR_1_R7_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_RC_BAR_1_R7_MAX                      (0xFFFFFFFFU)

/* I_PCIE_BUS_NUMBERS */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_BUS_NUMBERS_PBN_MASK            (0x000000FFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_BUS_NUMBERS_PBN_SHIFT           (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_BUS_NUMBERS_PBN_MAX             (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_BUS_NUMBERS_SBN_MASK            (0x0000FF00U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_BUS_NUMBERS_SBN_SHIFT           (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_BUS_NUMBERS_SBN_MAX             (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_BUS_NUMBERS_SUBN_MASK           (0x00FF0000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_BUS_NUMBERS_SUBN_SHIFT          (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_BUS_NUMBERS_SUBN_MAX            (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_BUS_NUMBERS_SLTN_MASK           (0xFF000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_BUS_NUMBERS_SLTN_SHIFT          (0x00000018U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_BUS_NUMBERS_SLTN_MAX            (0x000000FFU)

/* I_PCIE_IO_BASE_LIMIT */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_IOBS1_MASK        (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_IOBS1_SHIFT       (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_IOBS1_MAX         (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_R1_MASK           (0x0000000EU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_R1_SHIFT          (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_R1_MAX            (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_IBR_MASK          (0x000000F0U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_IBR_SHIFT         (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_IBR_MAX           (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_IOBS2_MASK        (0x00000100U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_IOBS2_SHIFT       (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_IOBS2_MAX         (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_R2_MASK           (0x00000E00U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_R2_SHIFT          (0x00000009U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_R2_MAX            (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_ILR_MASK          (0x0000F000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_ILR_SHIFT         (0x0000000CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_ILR_MAX           (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_R3_MASK           (0x00FF0000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_R3_SHIFT          (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_R3_MAX            (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_MPE_MASK          (0x01000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_MPE_SHIFT         (0x00000018U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_MPE_MAX           (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_R4_MASK           (0x06000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_R4_SHIFT          (0x00000019U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_R4_MAX            (0x00000003U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_STA_MASK          (0x08000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_STA_SHIFT         (0x0000001BU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_STA_MAX           (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_RTA_MASK          (0x10000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_RTA_SHIFT         (0x0000001CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_RTA_MAX           (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_RMA_MASK          (0x20000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_RMA_SHIFT         (0x0000001DU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_RMA_MAX           (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_RSE_MASK          (0x40000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_RSE_SHIFT         (0x0000001EU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_RSE_MAX           (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_DPE_MASK          (0x80000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_DPE_SHIFT         (0x0000001FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_DPE_MAX           (0x00000001U)

/* I_PCIE_MEM_BASE_LIMIT */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_MEM_BASE_LIMIT_R1_MASK          (0x0000000FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_MEM_BASE_LIMIT_R1_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_MEM_BASE_LIMIT_R1_MAX           (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_MEM_BASE_LIMIT_MBR_MASK         (0x0000FFF0U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_MEM_BASE_LIMIT_MBR_SHIFT        (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_MEM_BASE_LIMIT_MBR_MAX          (0x00000FFFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_MEM_BASE_LIMIT_R2_MASK          (0x000F0000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_MEM_BASE_LIMIT_R2_SHIFT         (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_MEM_BASE_LIMIT_R2_MAX           (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_MEM_BASE_LIMIT_MLR_MASK         (0xFFF00000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_MEM_BASE_LIMIT_MLR_SHIFT        (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_MEM_BASE_LIMIT_MLR_MAX          (0x00000FFFU)

/* I_PCIE_PREFETCH_BASE_LIMIT */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_PREFETCH_BASE_LIMIT_PMBR_MASK   (0x0000FFFFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_PREFETCH_BASE_LIMIT_PMBR_SHIFT  (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_PREFETCH_BASE_LIMIT_PMBR_MAX    (0x0000FFFFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_PREFETCH_BASE_LIMIT_PMLR_MASK   (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_PREFETCH_BASE_LIMIT_PMLR_SHIFT  (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_PREFETCH_BASE_LIMIT_PMLR_MAX    (0x0000FFFFU)

/* I_PCIE_PREFETCH_BASE_UPPER */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_PREFETCH_BASE_UPPER_PBRU_MASK   (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_PREFETCH_BASE_UPPER_PBRU_SHIFT  (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_PREFETCH_BASE_UPPER_PBRU_MAX    (0xFFFFFFFFU)

/* I_PCIE_PREFETCH_LIMIT_UPPER */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_PREFETCH_LIMIT_UPPER_PLRU_MASK  (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_PREFETCH_LIMIT_UPPER_PLRU_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_PREFETCH_LIMIT_UPPER_PLRU_MAX   (0xFFFFFFFFU)

/* I_PCIE_IO_BASE_LIMIT_UPPER */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_UPPER_IBRU_MASK   (0x0000FFFFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_UPPER_IBRU_SHIFT  (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_UPPER_IBRU_MAX    (0x0000FFFFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_UPPER_ILR_MASK    (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_UPPER_ILR_SHIFT   (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_IO_BASE_LIMIT_UPPER_ILR_MAX     (0x0000FFFFU)

/* I_CAPABILITIES_POINTER */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CAPABILITIES_POINTER_CP_MASK         (0x000000FFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CAPABILITIES_POINTER_CP_SHIFT        (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CAPABILITIES_POINTER_CP_MAX          (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CAPABILITIES_POINTER_R15_MASK        (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CAPABILITIES_POINTER_R15_SHIFT       (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CAPABILITIES_POINTER_R15_MAX         (0x00FFFFFFU)

/* RSVD_0E */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_RSVD_0E_RSVD_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_RSVD_0E_RSVD_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_RSVD_0E_RSVD_MAX                       (0xFFFFFFFFU)

/* I_INTRPT_LINE_INTRPT_PIN */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_ILR_MASK      (0x000000FFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_ILR_SHIFT     (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_ILR_MAX       (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_IPR_MASK      (0x00000700U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_IPR_SHIFT     (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_IPR_MAX       (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_R5_MASK       (0x0000F800U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_R5_SHIFT      (0x0000000BU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_R5_MAX        (0x0000001FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_PERE_MASK     (0x00010000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_PERE_SHIFT    (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_PERE_MAX      (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_BCSE_MASK     (0x00020000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_BCSE_SHIFT    (0x00000011U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_BCSE_MAX      (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_ISAE_MASK     (0x00040000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_ISAE_SHIFT    (0x00000012U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_ISAE_MAX      (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_VGAE_MASK     (0x00080000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_VGAE_SHIFT    (0x00000013U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_VGAE_MAX      (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_VGA16D_MASK   (0x00100000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_VGA16D_SHIFT  (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_VGA16D_MAX    (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_R21_MASK      (0x00200000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_R21_SHIFT     (0x00000015U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_R21_MAX       (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_BCRSBR_MASK   (0x00400000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_BCRSBR_SHIFT  (0x00000016U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_BCRSBR_MAX    (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_R23_MASK      (0xFF800000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_R23_SHIFT     (0x00000017U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_INTRPT_LINE_INTRPT_PIN_R23_MAX       (0x000001FFU)

/* I_PWR_MGMT_CAP */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_CID_MASK                (0x000000FFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_CID_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_CID_MAX                 (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_CP_MASK                 (0x0000FF00U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_CP_SHIFT                (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_CP_MAX                  (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_VID_MASK                (0x00070000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_VID_SHIFT               (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_VID_MAX                 (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_PC_MASK                 (0x00080000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_PC_SHIFT                (0x00000013U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_PC_MAX                  (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_R0_MASK                 (0x00100000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_R0_SHIFT                (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_R0_MAX                  (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_DSI_MASK                (0x00200000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_DSI_SHIFT               (0x00000015U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_DSI_MAX                 (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_MCRAPS_MASK             (0x01C00000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_MCRAPS_SHIFT            (0x00000016U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_MCRAPS_MAX              (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_D1S_MASK                (0x02000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_D1S_SHIFT               (0x00000019U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_D1S_MAX                 (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_D2S_MASK                (0x04000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_D2S_SHIFT               (0x0000001AU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_D2S_MAX                 (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_PSD0S_MASK              (0x08000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_PSD0S_SHIFT             (0x0000001BU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_PSD0S_MAX               (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_PSD1S_MASK              (0x10000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_PSD1S_SHIFT             (0x0000001CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_PSD1S_MAX               (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_PSD2S_MASK              (0x20000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_PSD2S_SHIFT             (0x0000001DU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_PSD2S_MAX               (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_PSDHS_MASK              (0x40000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_PSDHS_SHIFT             (0x0000001EU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_PSDHS_MAX               (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_PSDCS_MASK              (0x80000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_PSDCS_SHIFT             (0x0000001FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CAP_PSDCS_MAX               (0x00000001U)

/* I_PWR_MGMT_CTRL_STAT_REP */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP_PS_MASK       (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP_PS_SHIFT      (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP_PS_MAX        (0x00000003U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP_R4_MASK       (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP_R4_SHIFT      (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP_R4_MAX        (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP_NSR_MASK      (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP_NSR_SHIFT     (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP_NSR_MAX       (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP_R3_MASK       (0x000000F0U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP_R3_SHIFT      (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP_R3_MAX        (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP_PE_MASK       (0x00000100U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP_PE_SHIFT      (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP_PE_MAX        (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP_R2_MASK       (0x00007E00U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP_R2_SHIFT      (0x00000009U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP_R2_MAX        (0x0000003FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP_PMES_MASK     (0x00008000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP_PMES_SHIFT    (0x0000000FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP_PMES_MAX      (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP_R1_MASK       (0x00FF0000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP_R1_SHIFT      (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP_R1_MAX        (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP_DR_MASK       (0xFF000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP_DR_SHIFT      (0x00000018U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PWR_MGMT_CTRL_STAT_REP_DR_MAX        (0x000000FFU)

/* I_MSI_CTRL_REG */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_CTRL_REG_CID1_MASK               (0x000000FFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_CTRL_REG_CID1_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_CTRL_REG_CID1_MAX                (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_CTRL_REG_CP1_MASK                (0x0000FF00U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_CTRL_REG_CP1_SHIFT               (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_CTRL_REG_CP1_MAX                 (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_CTRL_REG_ME_MASK                 (0x00010000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_CTRL_REG_ME_SHIFT                (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_CTRL_REG_ME_MAX                  (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_CTRL_REG_MMC_MASK                (0x000E0000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_CTRL_REG_MMC_SHIFT               (0x00000011U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_CTRL_REG_MMC_MAX                 (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_CTRL_REG_MME_MASK                (0x00700000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_CTRL_REG_MME_SHIFT               (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_CTRL_REG_MME_MAX                 (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_CTRL_REG_BAC64_MASK              (0x00800000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_CTRL_REG_BAC64_SHIFT             (0x00000017U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_CTRL_REG_BAC64_MAX               (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_CTRL_REG_MC_MASK                 (0x01000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_CTRL_REG_MC_SHIFT                (0x00000018U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_CTRL_REG_MC_MAX                  (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_CTRL_REG_R0_MASK                 (0xFE000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_CTRL_REG_R0_SHIFT                (0x00000019U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_CTRL_REG_R0_MAX                  (0x0000007FU)

/* I_MSI_MSG_LOW_ADDR */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MSG_LOW_ADDR_R1_MASK             (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MSG_LOW_ADDR_R1_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MSG_LOW_ADDR_R1_MAX              (0x00000003U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MSG_LOW_ADDR_MAL_MASK            (0xFFFFFFFCU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MSG_LOW_ADDR_MAL_SHIFT           (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MSG_LOW_ADDR_MAL_MAX             (0x3FFFFFFFU)

/* I_MSI_MSG_HI_ADDR */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MSG_HI_ADDR_MAH_MASK             (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MSG_HI_ADDR_MAH_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MSG_HI_ADDR_MAH_MAX              (0xFFFFFFFFU)

/* I_MSI_MSG_DATA */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MSG_DATA_MD_MASK                 (0x0000FFFFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MSG_DATA_MD_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MSG_DATA_MD_MAX                  (0x0000FFFFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MSG_DATA_R2_MASK                 (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MSG_DATA_R2_SHIFT                (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MSG_DATA_R2_MAX                  (0x0000FFFFU)

/* I_MSI_MASK */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MASK_MM_MASK                     (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MASK_MM_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_MASK_MM_MAX                      (0x00000001U)

/* I_MSI_PENDING_BITS */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_PENDING_BITS_MP_MASK             (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_PENDING_BITS_MP_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSI_PENDING_BITS_MP_MAX              (0x00000001U)

/* I_MSIX_CTRL */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL_CID_MASK                   (0x000000FFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL_CID_SHIFT                  (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL_CID_MAX                    (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL_CP_MASK                    (0x0000FF00U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL_CP_SHIFT                   (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL_CP_MAX                     (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL_MSIXTS_MASK                (0x07FF0000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL_MSIXTS_SHIFT               (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL_MSIXTS_MAX                 (0x000007FFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL_R0_MASK                    (0x38000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL_R0_SHIFT                   (0x0000001BU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL_R0_MAX                     (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL_FM_MASK                    (0x40000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL_FM_SHIFT                   (0x0000001EU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL_FM_MAX                     (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL_MSIXE_MASK                 (0x80000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL_MSIXE_SHIFT                (0x0000001FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_CTRL_MSIXE_MAX                  (0x00000001U)

/* I_MSIX_TBL_OFFSET */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_TBL_OFFSET_BARI_MASK            (0x00000007U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_TBL_OFFSET_BARI_SHIFT           (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_TBL_OFFSET_BARI_MAX             (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_TBL_OFFSET_TO_MASK              (0xFFFFFFF8U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_TBL_OFFSET_TO_SHIFT             (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_TBL_OFFSET_TO_MAX               (0x1FFFFFFFU)

/* I_MSIX_PENDING_INTRPT */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_PENDING_INTRPT_BARI1_MASK       (0x00000007U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_PENDING_INTRPT_BARI1_SHIFT      (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_PENDING_INTRPT_BARI1_MAX        (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_PENDING_INTRPT_PBAO_MASK        (0xFFFFFFF8U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_PENDING_INTRPT_PBAO_SHIFT       (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_MSIX_PENDING_INTRPT_PBAO_MAX         (0x1FFFFFFFU)

/* I_PCIE_CAP_LIST */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_LIST_CID_MASK               (0x000000FFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_LIST_CID_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_LIST_CID_MAX                (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_LIST_NCP_MASK               (0x0000FF00U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_LIST_NCP_SHIFT              (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_LIST_NCP_MAX                (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_LIST_PCV_MASK               (0x000F0000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_LIST_PCV_SHIFT              (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_LIST_PCV_MAX                (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_LIST_DT_MASK                (0x00F00000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_LIST_DT_SHIFT               (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_LIST_DT_MAX                 (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_LIST_SI_MASK                (0x01000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_LIST_SI_SHIFT               (0x00000018U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_LIST_SI_MAX                 (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_LIST_IMN_MASK               (0x3E000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_LIST_IMN_SHIFT              (0x00000019U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_LIST_IMN_MAX                (0x0000001FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_LIST_TRS_MASK               (0x40000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_LIST_TRS_SHIFT              (0x0000001EU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_LIST_TRS_MAX                (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_LIST_R0_MASK                (0x80000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_LIST_R0_SHIFT               (0x0000001FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_LIST_R0_MAX                 (0x00000001U)

/* I_PCIE_CAP */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_MP_MASK                     (0x00000007U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_MP_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_MP_MAX                      (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_PFS_MASK                    (0x00000018U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_PFS_SHIFT                   (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_PFS_MAX                     (0x00000003U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_ETFS_MASK                   (0x00000020U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_ETFS_SHIFT                  (0x00000005U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_ETFS_MAX                    (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_AL0L_MASK                   (0x000001C0U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_AL0L_SHIFT                  (0x00000006U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_AL0L_MAX                    (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_AL1L_MASK                   (0x00000E00U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_AL1L_SHIFT                  (0x00000009U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_AL1L_MAX                    (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_R3_MASK                     (0x00007000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_R3_SHIFT                    (0x0000000CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_R3_MAX                      (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_RER_MASK                    (0x00008000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_RER_SHIFT                   (0x0000000FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_RER_MAX                     (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_R4_MASK                     (0x00030000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_R4_SHIFT                    (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_R4_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_CSP_MASK                    (0x03FC0000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_CSP_SHIFT                   (0x00000012U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_CSP_MAX                     (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_CPLS_MASK                   (0x0C000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_CPLS_SHIFT                  (0x0000001AU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_CPLS_MAX                    (0x00000003U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_FLRC_MASK                   (0x10000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_FLRC_SHIFT                  (0x0000001CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_FLRC_MAX                    (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_R5_MASK                     (0xE0000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_R5_SHIFT                    (0x0000001DU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_R5_MAX                      (0x00000007U)

/* I_PCIE_DEV_CTRL_STATUS */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ECER_MASK       (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ECER_SHIFT      (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ECER_MAX        (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ENFER_MASK      (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ENFER_SHIFT     (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ENFER_MAX       (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_EFER_MASK       (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_EFER_SHIFT      (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_EFER_MAX        (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_EURR_MASK       (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_EURR_SHIFT      (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_EURR_MAX        (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ERO_MASK        (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ERO_SHIFT       (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ERO_MAX         (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_MP_MASK         (0x000000E0U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_MP_SHIFT        (0x00000005U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_MP_MAX          (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ETE_MASK        (0x00000100U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ETE_SHIFT       (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ETE_MAX         (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_PFE_MASK        (0x00000200U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_PFE_SHIFT       (0x00000009U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_PFE_MAX         (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_APPME_MASK      (0x00000400U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_APPME_SHIFT     (0x0000000AU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_APPME_MAX       (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ENS_MASK        (0x00000800U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ENS_SHIFT       (0x0000000BU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_ENS_MAX         (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_MRR_MASK        (0x00007000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_MRR_SHIFT       (0x0000000CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_MRR_MAX         (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_R7_MASK         (0x00008000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_R7_SHIFT        (0x0000000FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_R7_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_CED_MASK        (0x00010000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_CED_SHIFT       (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_CED_MAX         (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_NFED_MASK       (0x00020000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_NFED_SHIFT      (0x00000011U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_NFED_MAX        (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_FED_MASK        (0x00040000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_FED_SHIFT       (0x00000012U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_FED_MAX         (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_URD_MASK        (0x00080000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_URD_SHIFT       (0x00000013U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_URD_MAX         (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_APD_MASK        (0x00100000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_APD_SHIFT       (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_APD_MAX         (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_TP_MASK         (0x00200000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_TP_SHIFT        (0x00000015U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_TP_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_R8_MASK         (0xFFC00000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_R8_SHIFT        (0x00000016U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_R8_MAX          (0x000003FFU)

/* I_LINK_CAP */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_MLS_MASK                    (0x0000000FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_MLS_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_MLS_MAX                     (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_MLW_MASK                    (0x000003F0U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_MLW_SHIFT                   (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_MLW_MAX                     (0x0000003FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_ASPM_MASK                   (0x00000C00U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_ASPM_SHIFT                  (0x0000000AU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_ASPM_MAX                    (0x00000003U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_L0EL_MASK                   (0x00007000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_L0EL_SHIFT                  (0x0000000CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_L0EL_MAX                    (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_L1EL_MASK                   (0x00038000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_L1EL_SHIFT                  (0x0000000FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_L1EL_MAX                    (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_CPM_MASK                    (0x00040000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_CPM_SHIFT                   (0x00000012U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_CPM_MAX                     (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_SERC_MASK                   (0x00080000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_SERC_SHIFT                  (0x00000013U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_SERC_MAX                    (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_DARC_MASK                   (0x00100000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_DARC_SHIFT                  (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_DARC_MAX                    (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_LBNC_MASK                   (0x00200000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_LBNC_SHIFT                  (0x00000015U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_LBNC_MAX                    (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_ASPMOC_MASK                 (0x00400000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_ASPMOC_SHIFT                (0x00000016U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_ASPMOC_MAX                  (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_R9_MASK                     (0x00800000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_R9_SHIFT                    (0x00000017U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_R9_MAX                      (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_PN_MASK                     (0xFF000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_PN_SHIFT                    (0x00000018U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_PN_MAX                      (0x000000FFU)

/* I_LINK_CTRL_STATUS */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_ASPMC_MASK          (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_ASPMC_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_ASPMC_MAX           (0x00000003U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_R10_MASK            (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_R10_SHIFT           (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_R10_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_RCB_MASK            (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_RCB_SHIFT           (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_RCB_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_LD_MASK             (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_LD_SHIFT            (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_LD_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_RL_MASK             (0x00000020U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_RL_SHIFT            (0x00000005U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_RL_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_CCC_MASK            (0x00000040U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_CCC_SHIFT           (0x00000006U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_CCC_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_ES_MASK             (0x00000080U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_ES_SHIFT            (0x00000007U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_ES_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_ECPM_MASK           (0x00000100U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_ECPM_SHIFT          (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_ECPM_MAX            (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_HAWD_MASK           (0x00000200U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_HAWD_SHIFT          (0x00000009U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_HAWD_MAX            (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_LBMIE_MASK          (0x00000400U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_LBMIE_SHIFT         (0x0000000AU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_LBMIE_MAX           (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_LABIE_MASK          (0x00000800U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_LABIE_SHIFT         (0x0000000BU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_LABIE_MAX           (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_R11_MASK            (0x0000F000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_R11_SHIFT           (0x0000000CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_R11_MAX             (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_NLS_MASK            (0x000F0000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_NLS_SHIFT           (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_NLS_MAX             (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_NLW_MASK            (0x03F00000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_NLW_SHIFT           (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_NLW_MAX             (0x0000003FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_R12_MASK            (0x04000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_R12_SHIFT           (0x0000001AU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_R12_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_LTS_MASK            (0x08000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_LTS_SHIFT           (0x0000001BU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_LTS_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_SCC_MASK            (0x10000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_SCC_SHIFT           (0x0000001CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_SCC_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_DA_MASK             (0x20000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_DA_SHIFT            (0x0000001DU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_DA_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_LBMS_MASK           (0x40000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_LBMS_SHIFT          (0x0000001EU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_LBMS_MAX            (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_LABS_MASK           (0x80000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_LABS_SHIFT          (0x0000001FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_LABS_MAX            (0x00000001U)

/* I_SLOT_CAPABILITY */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_ABPRSNT_MASK         (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_ABPRSNT_SHIFT        (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_ABPRSNT_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_PCP_MASK             (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_PCP_SHIFT            (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_PCP_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_MRLSP_MASK           (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_MRLSP_SHIFT          (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_MRLSP_MAX            (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_AIP_MASK             (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_AIP_SHIFT            (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_AIP_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_PIP_MASK             (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_PIP_SHIFT            (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_PIP_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_HPS_MASK             (0x00000020U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_HPS_SHIFT            (0x00000005U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_HPS_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_HPC_MASK             (0x00000040U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_HPC_SHIFT            (0x00000006U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_HPC_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_SPLV_MASK            (0x00007F80U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_SPLV_SHIFT           (0x00000007U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_SPLV_MAX             (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_SPLS_MASK            (0x00018000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_SPLS_SHIFT           (0x0000000FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_SPLS_MAX             (0x00000003U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_EIP_MASK             (0x00020000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_EIP_SHIFT            (0x00000011U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_EIP_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_NCCS_MASK            (0x00040000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_NCCS_SHIFT           (0x00000012U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_NCCS_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_PSN_MASK             (0xFFF80000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_PSN_SHIFT            (0x00000013U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CAPABILITY_PSN_MAX              (0x00001FFFU)

/* I_SLOT_CTRL_STATUS */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_ABPE_MASK           (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_ABPE_SHIFT          (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_ABPE_MAX            (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_PFDE_MASK           (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_PFDE_SHIFT          (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_PFDE_MAX            (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_MSCE_MASK           (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_MSCE_SHIFT          (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_MSCE_MAX            (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_PDCE_MASK           (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_PDCE_SHIFT          (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_PDCE_MAX            (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_CCIE_MASK           (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_CCIE_SHIFT          (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_CCIE_MAX            (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_HPIE_MASK           (0x00000020U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_HPIE_SHIFT          (0x00000005U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_HPIE_MAX            (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_AIC_MASK            (0x000000C0U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_AIC_SHIFT           (0x00000006U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_AIC_MAX             (0x00000003U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_PIC_MASK            (0x00000300U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_PIC_SHIFT           (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_PIC_MAX             (0x00000003U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_PCC_MASK            (0x00000400U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_PCC_SHIFT           (0x0000000AU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_PCC_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_EMIC_MASK           (0x00000800U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_EMIC_SHIFT          (0x0000000BU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_EMIC_MAX            (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_DLLSCE_MASK         (0x00001000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_DLLSCE_SHIFT        (0x0000000CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_DLLSCE_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_RSCS1_MASK          (0x0000E000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_RSCS1_SHIFT         (0x0000000DU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_RSCS1_MAX           (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_ABPRSD_MASK         (0x00010000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_ABPRSD_SHIFT        (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_ABPRSD_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_PFD_MASK            (0x00020000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_PFD_SHIFT           (0x00000011U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_PFD_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_MRLSC_MASK          (0x00040000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_MRLSC_SHIFT         (0x00000012U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_MRLSC_MAX           (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_PDC_MASK            (0x00080000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_PDC_SHIFT           (0x00000013U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_PDC_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_CMDCMPL_MASK        (0x00100000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_CMDCMPL_SHIFT       (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_CMDCMPL_MAX         (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_MRLSS_MASK          (0x00200000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_MRLSS_SHIFT         (0x00000015U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_MRLSS_MAX           (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_PDS_MASK            (0x00400000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_PDS_SHIFT           (0x00000016U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_PDS_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_EMIS_MASK           (0x00800000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_EMIS_SHIFT          (0x00000017U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_EMIS_MAX            (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_DLLSC_MASK          (0x01000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_DLLSC_SHIFT         (0x00000018U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_DLLSC_MAX           (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_RSCS2_MASK          (0xFE000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_RSCS2_SHIFT         (0x00000019U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SLOT_CTRL_STATUS_RSCS2_MAX           (0x0000007FU)

/* I_ROOT_CTRL_CAP */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_CTRL_CAP_SECEE_MASK             (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_CTRL_CAP_SECEE_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_CTRL_CAP_SECEE_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_CTRL_CAP_SENFEE_MASK            (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_CTRL_CAP_SENFEE_SHIFT           (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_CTRL_CAP_SENFEE_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_CTRL_CAP_SEFEE_MASK             (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_CTRL_CAP_SEFEE_SHIFT            (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_CTRL_CAP_SEFEE_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_CTRL_CAP_PMEIE_MASK             (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_CTRL_CAP_PMEIE_SHIFT            (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_CTRL_CAP_PMEIE_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_CTRL_CAP_CRSSVE_MASK            (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_CTRL_CAP_CRSSVE_SHIFT           (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_CTRL_CAP_CRSSVE_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_CTRL_CAP_R27_MASK               (0xFFFFFFE0U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_CTRL_CAP_R27_SHIFT              (0x00000005U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_CTRL_CAP_R27_MAX                (0x07FFFFFFU)

/* I_ROOT_STATUS */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_STATUS_PMERID_MASK              (0x0000FFFFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_STATUS_PMERID_SHIFT             (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_STATUS_PMERID_MAX               (0x0000FFFFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_STATUS_PMES_MASK                (0x00010000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_STATUS_PMES_SHIFT               (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_STATUS_PMES_MAX                 (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_STATUS_PMEP_MASK                (0x00020000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_STATUS_PMEP_SHIFT               (0x00000011U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_STATUS_PMEP_MAX                 (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_STATUS_R18_MASK                 (0xFFFC0000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_STATUS_R18_SHIFT                (0x00000012U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_STATUS_R18_MAX                  (0x00003FFFU)

/* I_PCIE_CAP_2 */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_CTR_MASK                  (0x0000000FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_CTR_SHIFT                 (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_CTR_MAX                   (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_CTDS_MASK                 (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_CTDS_SHIFT                (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_CTDS_MAX                  (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_AFS_MASK                  (0x00000020U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_AFS_SHIFT                 (0x00000005U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_AFS_MAX                   (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_AOPRS_MASK                (0x00000040U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_AOPRS_SHIFT               (0x00000006U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_AOPRS_MAX                 (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_ACS32_MASK                (0x00000080U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_ACS32_SHIFT               (0x00000007U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_ACS32_MAX                 (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_ACS64_MASK                (0x00000100U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_ACS64_SHIFT               (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_ACS64_MAX                 (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_ACS128_MASK               (0x00000200U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_ACS128_SHIFT              (0x00000009U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_ACS128_MAX                (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_R14_MASK                  (0x00000400U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_R14_SHIFT                 (0x0000000AU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_R14_MAX                   (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_LMS_MASK                  (0x00000800U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_LMS_SHIFT                 (0x0000000BU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_LMS_MAX                   (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_TPHC_MASK                 (0x00001000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_TPHC_SHIFT                (0x0000000CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_TPHC_MAX                  (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_R15_MASK                  (0x0000C000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_R15_SHIFT                 (0x0000000EU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_R15_MAX                   (0x00000003U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_T10CS_MASK                (0x00010000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_T10CS_SHIFT               (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_T10CS_MAX                 (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_T10RS_MASK                (0x00020000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_T10RS_SHIFT               (0x00000011U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_T10RS_MAX                 (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_OBFF_MASK                 (0x000C0000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_OBFF_SHIFT                (0x00000012U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_OBFF_MAX                  (0x00000003U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_EXFS_MASK                 (0x00100000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_EXFS_SHIFT                (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_EXFS_MAX                  (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_EEPS_MASK                 (0x00200000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_EEPS_SHIFT                (0x00000015U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_EEPS_MAX                  (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_MEEP_MASK                 (0x00C00000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_MEEP_SHIFT                (0x00000016U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_MEEP_MAX                  (0x00000003U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_R16_MASK                  (0xFF000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_R16_SHIFT                 (0x00000018U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_CAP_2_R16_MAX                   (0x000000FFU)

/* I_PCIE_DEV_CTRL_STATUS_2 */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_CTV_MASK      (0x0000000FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_CTV_SHIFT     (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_CTV_MAX       (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_CTD_MASK      (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_CTD_SHIFT     (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_CTD_MAX       (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_AFE_MASK      (0x00000020U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_AFE_SHIFT     (0x00000005U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_AFE_MAX       (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_AORE_MASK     (0x00000040U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_AORE_SHIFT    (0x00000006U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_AORE_MAX      (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_R18_MASK      (0x00000080U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_R18_SHIFT     (0x00000007U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_R18_MAX       (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_IRE_MASK      (0x00000100U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_IRE_SHIFT     (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_IRE_MAX       (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_ICE_MASK      (0x00000200U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_ICE_SHIFT     (0x00000009U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_ICE_MAX       (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_LTRME_MASK    (0x00000400U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_LTRME_SHIFT   (0x0000000AU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_LTRME_MAX     (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_R19_MASK      (0x00000800U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_R19_SHIFT     (0x0000000BU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_R19_MAX       (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_T10RE_MASK    (0x00001000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_T10RE_SHIFT   (0x0000000CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_T10RE_MAX     (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_OBFFE_MASK    (0x00006000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_OBFFE_SHIFT   (0x0000000DU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_OBFFE_MAX     (0x00000003U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_R20_MASK      (0xFFFF8000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_R20_SHIFT     (0x0000000FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_PCIE_DEV_CTRL_STATUS_2_R20_MAX       (0x0001FFFFU)

/* I_LINK_CAP_2 */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_SLSV_MASK                 (0x0000001EU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_SLSV_SHIFT                (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_SLSV_MAX                  (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_R1_MASK                   (0x000001E0U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_R1_SHIFT                  (0x00000005U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_R1_MAX                    (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_LSOGSSV_MASK              (0x00001E00U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_LSOGSSV_SHIFT             (0x00000009U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_LSOGSSV_MAX               (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_R2_MASK                   (0x0000E000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_R2_SHIFT                  (0x0000000DU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_R2_MAX                    (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_LSORSSV_MASK              (0x000F0000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_LSORSSV_SHIFT             (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_LSORSSV_MAX               (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_R3_MASK                   (0x00700000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_R3_SHIFT                  (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_R3_MAX                    (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_RTPDS_MASK                (0x00800000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_RTPDS_SHIFT               (0x00000017U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_RTPDS_MAX                 (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_TWRTPDS_MASK              (0x01000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_TWRTPDS_SHIFT             (0x00000018U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_TWRTPDS_MAX               (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_R25_MASK                  (0x7E000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_R25_SHIFT                 (0x00000019U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_R25_MAX                   (0x0000003FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_R31_MASK                  (0x80000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_R31_SHIFT                 (0x0000001FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CAP_2_R31_MAX                   (0x00000001U)

/* I_LINK_CTRL_STATUS_2 */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_TLS_MASK          (0x0000000FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_TLS_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_TLS_MAX           (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_EC_MASK           (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_EC_SHIFT          (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_EC_MAX            (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_HASD_MASK         (0x00000020U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_HASD_SHIFT        (0x00000005U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_HASD_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_SD_MASK           (0x00000040U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_SD_SHIFT          (0x00000006U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_SD_MAX            (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_TM_MASK           (0x00000380U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_TM_SHIFT          (0x00000007U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_TM_MAX            (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_EMC_MASK          (0x00000400U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_EMC_SHIFT         (0x0000000AU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_EMC_MAX           (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_CS_MASK           (0x00000800U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_CS_SHIFT          (0x0000000BU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_CS_MAX            (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_CD_MASK           (0x0000F000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_CD_SHIFT          (0x0000000CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_CD_MAX            (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_CDEL_MASK         (0x00010000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_CDEL_SHIFT        (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_CDEL_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_EQC_MASK          (0x00020000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_EQC_SHIFT         (0x00000011U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_EQC_MAX           (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_EP1S_MASK         (0x00040000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_EP1S_SHIFT        (0x00000012U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_EP1S_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_EP2S_MASK         (0x00080000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_EP2S_SHIFT        (0x00000013U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_EP2S_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_EP3S_MASK         (0x00100000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_EP3S_SHIFT        (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_EP3S_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_LE_MASK           (0x00200000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_LE_SHIFT          (0x00000015U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_LE_MAX            (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_RTP_MASK          (0x00400000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_RTP_SHIFT         (0x00000016U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_RTP_MAX           (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_TWRTP_MASK        (0x00800000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_TWRTP_SHIFT       (0x00000017U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_TWRTP_MAX         (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_R21_MASK          (0x0F000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_R21_SHIFT         (0x00000018U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_R21_MAX           (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_DCP_MASK          (0x70000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_DCP_SHIFT         (0x0000001CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_DCP_MAX           (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_DMR_MASK          (0x80000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_DMR_SHIFT         (0x0000001FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CTRL_STATUS_2_DMR_MAX           (0x00000001U)

/* I_AER_ENHNCD_CAP */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_AER_ENHNCD_CAP_PECID_MASK            (0x0000FFFFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_AER_ENHNCD_CAP_PECID_SHIFT           (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_AER_ENHNCD_CAP_PECID_MAX             (0x0000FFFFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_AER_ENHNCD_CAP_CV_MASK               (0x000F0000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_AER_ENHNCD_CAP_CV_SHIFT              (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_AER_ENHNCD_CAP_CV_MAX                (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_AER_ENHNCD_CAP_NCO_MASK              (0xFFF00000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_AER_ENHNCD_CAP_NCO_SHIFT             (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_AER_ENHNCD_CAP_NCO_MAX               (0x00000FFFU)

/* I_UNCORR_ERR_STATUS */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_R25_MASK           (0x0000000FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_R25_SHIFT          (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_R25_MAX            (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_DLPE_MASK          (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_DLPE_SHIFT         (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_DLPE_MAX           (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_R26_MASK           (0x00000FE0U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_R26_SHIFT          (0x00000005U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_R26_MAX            (0x0000007FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_PT_MASK            (0x00001000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_PT_SHIFT           (0x0000000CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_PT_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_FCPE_MASK          (0x00002000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_FCPE_SHIFT         (0x0000000DU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_FCPE_MAX           (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_CT_MASK            (0x00004000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_CT_SHIFT           (0x0000000EU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_CT_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_CA_MASK            (0x00008000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_CA_SHIFT           (0x0000000FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_CA_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_UC_MASK            (0x00010000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_UC_SHIFT           (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_UC_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_RO_MASK            (0x00020000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_RO_SHIFT           (0x00000011U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_RO_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_MT_MASK            (0x00040000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_MT_SHIFT           (0x00000012U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_MT_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_EE_MASK            (0x00080000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_EE_SHIFT           (0x00000013U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_EE_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_URE_MASK           (0x00100000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_URE_SHIFT          (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_URE_MAX            (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_R27_MASK           (0x00200000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_R27_SHIFT          (0x00000015U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_R27_MAX            (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_UIE_MASK           (0x00400000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_UIE_SHIFT          (0x00000016U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_UIE_MAX            (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_R28_MASK           (0xFF800000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_R28_SHIFT          (0x00000017U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_STATUS_R28_MAX            (0x000001FFU)

/* I_UNCORR_ERR_MASK */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_R29_MASK             (0x0000000FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_R29_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_R29_MAX              (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_DLPER_MASK           (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_DLPER_SHIFT          (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_DLPER_MAX            (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_R30_MASK             (0x00000FE0U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_R30_SHIFT            (0x00000005U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_R30_MAX              (0x0000007FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_PTM_MASK             (0x00001000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_PTM_SHIFT            (0x0000000CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_PTM_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_FCPER_MASK           (0x00002000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_FCPER_SHIFT          (0x0000000DU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_FCPER_MAX            (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_CTM_MASK             (0x00004000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_CTM_SHIFT            (0x0000000EU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_CTM_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_CAM_MASK             (0x00008000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_CAM_SHIFT            (0x0000000FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_CAM_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_UCM_MASK             (0x00010000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_UCM_SHIFT            (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_UCM_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_ROM_MASK             (0x00020000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_ROM_SHIFT            (0x00000011U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_ROM_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_MTM_MASK             (0x00040000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_MTM_SHIFT            (0x00000012U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_MTM_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_EEM_MASK             (0x00080000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_EEM_SHIFT            (0x00000013U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_EEM_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_UREM_MASK            (0x00100000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_UREM_SHIFT           (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_UREM_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_R31_MASK             (0x00200000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_R31_SHIFT            (0x00000015U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_R31_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_UIEM_MASK            (0x00400000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_UIEM_SHIFT           (0x00000016U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_UIEM_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_R32_MASK             (0xFF800000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_R32_SHIFT            (0x00000017U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_MASK_R32_MAX              (0x000001FFU)

/* I_UNCORR_ERR_SEVERITY */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_R33_MASK         (0x0000000FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_R33_SHIFT        (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_R33_MAX          (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_DLPES_MASK       (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_DLPES_SHIFT      (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_DLPES_MAX        (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_SDES_MASK        (0x00000020U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_SDES_SHIFT       (0x00000005U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_SDES_MAX         (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_R35_MASK         (0x00000FC0U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_R35_SHIFT        (0x00000006U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_R35_MAX          (0x0000003FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_PTS_MASK         (0x00001000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_PTS_SHIFT        (0x0000000CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_PTS_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_FCPES_MASK       (0x00002000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_FCPES_SHIFT      (0x0000000DU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_FCPES_MAX        (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_CTS_MASK         (0x00004000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_CTS_SHIFT        (0x0000000EU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_CTS_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_CAS_MASK         (0x00008000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_CAS_SHIFT        (0x0000000FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_CAS_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_UCS_MASK         (0x00010000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_UCS_SHIFT        (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_UCS_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_ROS_MASK         (0x00020000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_ROS_SHIFT        (0x00000011U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_ROS_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_MTS_MASK         (0x00040000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_MTS_SHIFT        (0x00000012U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_MTS_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_EES_MASK         (0x00080000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_EES_SHIFT        (0x00000013U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_EES_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_URES_MASK        (0x00100000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_URES_SHIFT       (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_URES_MAX         (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_R36_MASK         (0x00200000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_R36_SHIFT        (0x00000015U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_R36_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_UNCORR_INTRNL_ERR_SVRTY_MASK (0x00400000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_UNCORR_INTRNL_ERR_SVRTY_SHIFT (0x00000016U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_UNCORR_INTRNL_ERR_SVRTY_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_R37_MASK         (0xFF800000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_R37_SHIFT        (0x00000017U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_UNCORR_ERR_SEVERITY_R37_MAX          (0x000001FFU)

/* I_CORR_ERR_STATUS */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_RES_MASK             (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_RES_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_RES_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_R37_MASK             (0x0000003EU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_R37_SHIFT            (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_R37_MAX              (0x0000001FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_BTS_MASK             (0x00000040U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_BTS_SHIFT            (0x00000006U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_BTS_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_BDS_MASK             (0x00000080U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_BDS_SHIFT            (0x00000007U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_BDS_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_RNRS_MASK            (0x00000100U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_RNRS_SHIFT           (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_RNRS_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_R38_MASK             (0x00000E00U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_R38_SHIFT            (0x00000009U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_R38_MAX              (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_RTTS_MASK            (0x00001000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_RTTS_SHIFT           (0x0000000CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_RTTS_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_ANES_MASK            (0x00002000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_ANES_SHIFT           (0x0000000DU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_ANES_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_CIES_MASK            (0x00004000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_CIES_SHIFT           (0x0000000EU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_CIES_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_HLOS_MASK            (0x00008000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_HLOS_SHIFT           (0x0000000FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_HLOS_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_R39_MASK             (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_R39_SHIFT            (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_STATUS_R39_MAX              (0x0000FFFFU)

/* I_CORR_ERR_MASK */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_REM_MASK               (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_REM_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_REM_MAX                (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_R40_MASK               (0x0000003EU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_R40_SHIFT              (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_R40_MAX                (0x0000001FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_BTM_MASK               (0x00000040U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_BTM_SHIFT              (0x00000006U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_BTM_MAX                (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_BDM_MASK               (0x00000080U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_BDM_SHIFT              (0x00000007U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_BDM_MAX                (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_RNRM_MASK              (0x00000100U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_RNRM_SHIFT             (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_RNRM_MAX               (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_R41_MASK               (0x00000E00U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_R41_SHIFT              (0x00000009U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_R41_MAX                (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_RTTM_MASK              (0x00001000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_RTTM_SHIFT             (0x0000000CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_RTTM_MAX               (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_ANEM_MASK              (0x00002000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_ANEM_SHIFT             (0x0000000DU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_ANEM_MAX               (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_CIEM_MASK              (0x00004000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_CIEM_SHIFT             (0x0000000EU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_CIEM_MAX               (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_HLOM_MASK              (0x00008000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_HLOM_SHIFT             (0x0000000FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_HLOM_MAX               (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_R42_MASK               (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_R42_SHIFT              (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_CORR_ERR_MASK_R42_MAX                (0x0000FFFFU)

/* I_ADV_ERR_CAP_CTL */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_FEP_MASK             (0x0000001FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_FEP_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_FEP_MAX              (0x0000001FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_EGC_MASK             (0x00000020U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_EGC_SHIFT            (0x00000005U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_EGC_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_EEG_MASK             (0x00000040U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_EEG_SHIFT            (0x00000006U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_EEG_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_ECC_MASK             (0x00000080U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_ECC_SHIFT            (0x00000007U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_ECC_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_EEC_MASK             (0x00000100U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_EEC_SHIFT            (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_EEC_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_MHRC_MASK            (0x00000200U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_MHRC_SHIFT           (0x00000009U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_MHRC_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_MHRE_MASK            (0x00000400U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_MHRE_SHIFT           (0x0000000AU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_MHRE_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_TPLP_MASK            (0x00000800U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_TPLP_SHIFT           (0x0000000BU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_TPLP_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_R43_MASK             (0xFFFFF000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_R43_SHIFT            (0x0000000CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ADV_ERR_CAP_CTL_R43_MAX              (0x000FFFFFU)

/* I_HDR_LOG_0 */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_HDR_LOG_0_HD0_MASK                   (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_HDR_LOG_0_HD0_SHIFT                  (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_HDR_LOG_0_HD0_MAX                    (0xFFFFFFFFU)

/* I_HDR_LOG_1 */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_HDR_LOG_1_HD1_MASK                   (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_HDR_LOG_1_HD1_SHIFT                  (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_HDR_LOG_1_HD1_MAX                    (0xFFFFFFFFU)

/* I_HDR_LOG_2 */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_HDR_LOG_2_HD2_MASK                   (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_HDR_LOG_2_HD2_SHIFT                  (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_HDR_LOG_2_HD2_MAX                    (0xFFFFFFFFU)

/* I_HDR_LOG_3 */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_HDR_LOG_3_HD3_MASK                   (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_HDR_LOG_3_HD3_SHIFT                  (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_HDR_LOG_3_HD3_MAX                    (0xFFFFFFFFU)

/* I_ROOT_ERR_CMD */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_CMD_CERE_MASK               (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_CMD_CERE_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_CMD_CERE_MAX                (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_CMD_NFERE_MASK              (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_CMD_NFERE_SHIFT             (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_CMD_NFERE_MAX               (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_CMD_FERE_MASK               (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_CMD_FERE_SHIFT              (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_CMD_FERE_MAX                (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_CMD_R44_MASK                (0xFFFFFFF8U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_CMD_R44_SHIFT               (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_CMD_R44_MAX                 (0x1FFFFFFFU)

/* I_ROOT_ERR_STAT */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_STAT_ECR_MASK               (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_STAT_ECR_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_STAT_ECR_MAX                (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_STAT_MECR_MASK              (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_STAT_MECR_SHIFT             (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_STAT_MECR_MAX               (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_STAT_EFNR_MASK              (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_STAT_EFNR_SHIFT             (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_STAT_EFNR_MAX               (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_STAT_MEFNR_MASK             (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_STAT_MEFNR_SHIFT            (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_STAT_MEFNR_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_STAT_FUF_MASK               (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_STAT_FUF_SHIFT              (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_STAT_FUF_MAX                (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_STAT_NEMR_MASK              (0x00000020U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_STAT_NEMR_SHIFT             (0x00000005U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_STAT_NEMR_MAX               (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_STAT_FEMR_MASK              (0x00000040U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_STAT_FEMR_SHIFT             (0x00000006U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_STAT_FEMR_MAX               (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_STAT_R45_MASK               (0xFFFFFF80U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_STAT_R45_SHIFT              (0x00000007U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ROOT_ERR_STAT_R45_MAX                (0x01FFFFFFU)

/* I_ERR_SRC_ID */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ERR_SRC_ID_ECSI_MASK                 (0x0000FFFFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ERR_SRC_ID_ECSI_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ERR_SRC_ID_ECSI_MAX                  (0x0000FFFFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ERR_SRC_ID_EFNSI_MASK                (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ERR_SRC_ID_EFNSI_SHIFT               (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_ERR_SRC_ID_EFNSI_MAX                 (0x0000FFFFU)

/* I_TLP_PRE_LOG_0 */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_TLP_PRE_LOG_0_HD1_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_TLP_PRE_LOG_0_HD1_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_TLP_PRE_LOG_0_HD1_MAX                (0xFFFFFFFFU)

/* I_DEV_SER_NUM_CAP_HDR */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_DEV_SER_NUM_CAP_HDR_PECID_MASK       (0x0000FFFFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_DEV_SER_NUM_CAP_HDR_PECID_SHIFT      (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_DEV_SER_NUM_CAP_HDR_PECID_MAX        (0x0000FFFFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_DEV_SER_NUM_CAP_HDR_DSNCV_MASK       (0x000F0000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_DEV_SER_NUM_CAP_HDR_DSNCV_SHIFT      (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_DEV_SER_NUM_CAP_HDR_DSNCV_MAX        (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_DEV_SER_NUM_CAP_HDR_SNNCO_MASK       (0xFFF00000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_DEV_SER_NUM_CAP_HDR_SNNCO_SHIFT      (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_DEV_SER_NUM_CAP_HDR_SNNCO_MAX        (0x00000FFFU)

/* I_DEV_SER_NUM_0 */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_DEV_SER_NUM_0_DSND0_MASK             (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_DEV_SER_NUM_0_DSND0_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_DEV_SER_NUM_0_DSND0_MAX              (0xFFFFFFFFU)

/* I_DEV_SER_NUM_1 */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_DEV_SER_NUM_1_DSND1_MASK             (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_DEV_SER_NUM_1_DSND1_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_DEV_SER_NUM_1_DSND1_MAX              (0xFFFFFFFFU)

/* I_SEC_PCIE_CAP_HDR_REG */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SEC_PCIE_CAP_HDR_REG_PECI_MASK       (0x0000FFFFU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SEC_PCIE_CAP_HDR_REG_PECI_SHIFT      (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SEC_PCIE_CAP_HDR_REG_PECI_MAX        (0x0000FFFFU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SEC_PCIE_CAP_HDR_REG_CV_MASK         (0x000F0000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SEC_PCIE_CAP_HDR_REG_CV_SHIFT        (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SEC_PCIE_CAP_HDR_REG_CV_MAX          (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SEC_PCIE_CAP_HDR_REG_NCO_MASK        (0xFFF00000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SEC_PCIE_CAP_HDR_REG_NCO_SHIFT       (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_SEC_PCIE_CAP_HDR_REG_NCO_MAX         (0x00000FFFU)

/* I_LINK_CONTROL3 */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CONTROL3_PE_MASK                (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CONTROL3_PE_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CONTROL3_PE_MAX                 (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CONTROL3_LERIE_MASK             (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CONTROL3_LERIE_SHIFT            (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CONTROL3_LERIE_MAX              (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CONTROL3_R1_MASK                (0x000001FCU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CONTROL3_R1_SHIFT               (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CONTROL3_R1_MAX                 (0x0000007FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CONTROL3_ELSOSGV_MASK           (0x00001E00U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CONTROL3_ELSOSGV_SHIFT          (0x00000009U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CONTROL3_ELSOSGV_MAX            (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CONTROL3_R2_MASK                (0xFFFFE000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CONTROL3_R2_SHIFT               (0x0000000DU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LINK_CONTROL3_R2_MAX                 (0x0007FFFFU)

/* I_LANE_ERROR_STATUS */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_ERROR_STATUS_LES_MASK           (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_ERROR_STATUS_LES_SHIFT          (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_ERROR_STATUS_LES_MAX            (0x00000003U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_ERROR_STATUS_R0_MASK            (0xFFFFFFFCU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_ERROR_STATUS_R0_SHIFT           (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_ERROR_STATUS_R0_MAX             (0x3FFFFFFFU)

/* I_LANE_EQUALIZATION_CONTROL_0 */

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_DNTP0_MASK (0x0000000FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_DNTP0_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_DNTP0_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_DNRPH0_MASK (0x00000070U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_DNRPH0_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_DNRPH0_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_R0_1_MASK (0x00000080U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_R0_1_SHIFT (0x00000007U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_R0_1_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_UPTP0_MASK (0x00000F00U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_UPTP0_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_UPTP0_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_UPRPH0_MASK (0x00007000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_UPRPH0_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_UPRPH0_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_R1_MASK  (0x00008000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_R1_SHIFT (0x0000000FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_R1_MAX   (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_DNTP1_MASK (0x000F0000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_DNTP1_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_DNTP1_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_DNRPH1_MASK (0x00700000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_DNRPH1_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_DNRPH1_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_R2_1_MASK (0x00800000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_R2_1_SHIFT (0x00000017U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_R2_1_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_UPTP1_MASK (0x0F000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_UPTP1_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_UPTP1_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_UPRPH1_MASK (0x70000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_UPRPH1_SHIFT (0x0000001CU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_UPRPH1_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_R3_MASK  (0x80000000U)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_R3_SHIFT (0x0000001FU)
#define CSL_PCIE_RP_CORE_RC_I_RC_PCIE_BASE_I_LANE_EQUALIZATION_CONTROL_0_R3_MAX   (0x00000001U)

/* I_VC_ENH_CAP_HEADER_REG */

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_ENH_CAP_HEADER_REG_PECID_MASK    (0x0000FFFFU)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_ENH_CAP_HEADER_REG_PECID_SHIFT   (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_ENH_CAP_HEADER_REG_PECID_MAX     (0x0000FFFFU)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_ENH_CAP_HEADER_REG_CV_MASK       (0x000F0000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_ENH_CAP_HEADER_REG_CV_SHIFT      (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_ENH_CAP_HEADER_REG_CV_MAX        (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_ENH_CAP_HEADER_REG_NCO_MASK      (0xFFF00000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_ENH_CAP_HEADER_REG_NCO_SHIFT     (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_ENH_CAP_HEADER_REG_NCO_MAX       (0x00000FFFU)

/* I_PORT_VC_CAP_REG_1 */

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_PORT_VC_CAP_REG_1_EVC_MASK          (0x00000007U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_PORT_VC_CAP_REG_1_EVC_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_PORT_VC_CAP_REG_1_EVC_MAX           (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_PORT_VC_CAP_REG_1_R0_MASK           (0xFFFFFFF0U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_PORT_VC_CAP_REG_1_R0_SHIFT          (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_PORT_VC_CAP_REG_1_R0_MAX            (0x0FFFFFFFU)

/* I_PORT_VC_CAP_REG_2 */

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_PORT_VC_CAP_REG_2_R1_MASK           (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_PORT_VC_CAP_REG_2_R1_SHIFT          (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_PORT_VC_CAP_REG_2_R1_MAX            (0xFFFFFFFFU)

/* I_PORT_VC_CTRL_STS_REG */

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_PORT_VC_CTRL_STS_REG_R2_MASK        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_PORT_VC_CTRL_STS_REG_R2_SHIFT       (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_PORT_VC_CTRL_STS_REG_R2_MAX         (0xFFFFFFFFU)

/* I_VC_RES_CAP_REG_0 */

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_0_R1_MASK            (0x00007FFFU)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_0_R1_SHIFT           (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_0_R1_MAX             (0x00007FFFU)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_0_RST_MASK           (0x00008000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_0_RST_SHIFT          (0x0000000FU)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_0_RST_MAX            (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_0_R3_MASK            (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_0_R3_SHIFT           (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_0_R3_MAX             (0x0000FFFFU)

/* I_VC_RES_CTRL_REG_0 */

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_0_TVM0_MASK         (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_0_TVM0_SHIFT        (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_0_TVM0_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_0_TVM_MASK          (0x000000FEU)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_0_TVM_SHIFT         (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_0_TVM_MAX           (0x0000007FU)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_0_LPAT_MASK         (0x00010000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_0_LPAT_SHIFT        (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_0_LPAT_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_0_PARS_MASK         (0x000E0000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_0_PARS_SHIFT        (0x00000011U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_0_PARS_MAX          (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_0_R5_MASK           (0x00F00000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_0_R5_SHIFT          (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_0_R5_MAX            (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_0_VCI_MASK          (0x07000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_0_VCI_SHIFT         (0x00000018U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_0_VCI_MAX           (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_0_R6_MASK           (0x78000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_0_R6_SHIFT          (0x0000001BU)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_0_R6_MAX            (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_0_VCEN_MASK         (0x80000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_0_VCEN_SHIFT        (0x0000001FU)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_0_VCEN_MAX          (0x00000001U)

/* I_VC_RES_STS_REG_0 */

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_0_PATS_MASK          (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_0_PATS_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_0_PATS_MAX           (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_0_VCNP_MASK          (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_0_VCNP_SHIFT         (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_0_VCNP_MAX           (0x00000001U)

/* I_VC_RES_CAP_REG_1 */

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_1_R1_MASK            (0x00007FFFU)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_1_R1_SHIFT           (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_1_R1_MAX             (0x00007FFFU)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_1_RST_MASK           (0x00008000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_1_RST_SHIFT          (0x0000000FU)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_1_RST_MAX            (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_1_R3_MASK            (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_1_R3_SHIFT           (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_1_R3_MAX             (0x0000FFFFU)

/* I_VC_RES_CTRL_REG_1 */

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_1_TVM0_MASK         (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_1_TVM0_SHIFT        (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_1_TVM0_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_1_TVM_MASK          (0x000000FEU)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_1_TVM_SHIFT         (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_1_TVM_MAX           (0x0000007FU)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_1_LPAT_MASK         (0x00010000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_1_LPAT_SHIFT        (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_1_LPAT_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_1_PARS_MASK         (0x000E0000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_1_PARS_SHIFT        (0x00000011U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_1_PARS_MAX          (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_1_R5_MASK           (0x00F00000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_1_R5_SHIFT          (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_1_R5_MAX            (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_1_VCI_MASK          (0x07000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_1_VCI_SHIFT         (0x00000018U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_1_VCI_MAX           (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_1_R6_MASK           (0x78000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_1_R6_SHIFT          (0x0000001BU)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_1_R6_MAX            (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_1_VCEN_MASK         (0x80000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_1_VCEN_SHIFT        (0x0000001FU)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_1_VCEN_MAX          (0x00000001U)

/* I_VC_RES_STS_REG_1 */

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_1_PATS_MASK          (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_1_PATS_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_1_PATS_MAX           (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_1_VCNP_MASK          (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_1_VCNP_SHIFT         (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_1_VCNP_MAX           (0x00000001U)

/* I_VC_RES_CAP_REG_2 */

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_2_R1_MASK            (0x00007FFFU)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_2_R1_SHIFT           (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_2_R1_MAX             (0x00007FFFU)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_2_RST_MASK           (0x00008000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_2_RST_SHIFT          (0x0000000FU)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_2_RST_MAX            (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_2_R3_MASK            (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_2_R3_SHIFT           (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_2_R3_MAX             (0x0000FFFFU)

/* I_VC_RES_CTRL_REG_2 */

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_2_TVM0_MASK         (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_2_TVM0_SHIFT        (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_2_TVM0_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_2_TVM_MASK          (0x000000FEU)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_2_TVM_SHIFT         (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_2_TVM_MAX           (0x0000007FU)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_2_LPAT_MASK         (0x00010000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_2_LPAT_SHIFT        (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_2_LPAT_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_2_PARS_MASK         (0x000E0000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_2_PARS_SHIFT        (0x00000011U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_2_PARS_MAX          (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_2_R5_MASK           (0x00F00000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_2_R5_SHIFT          (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_2_R5_MAX            (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_2_VCI_MASK          (0x07000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_2_VCI_SHIFT         (0x00000018U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_2_VCI_MAX           (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_2_R6_MASK           (0x78000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_2_R6_SHIFT          (0x0000001BU)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_2_R6_MAX            (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_2_VCEN_MASK         (0x80000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_2_VCEN_SHIFT        (0x0000001FU)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_2_VCEN_MAX          (0x00000001U)

/* I_VC_RES_STS_REG_2 */

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_2_PATS_MASK          (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_2_PATS_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_2_PATS_MAX           (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_2_VCNP_MASK          (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_2_VCNP_SHIFT         (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_2_VCNP_MAX           (0x00000001U)

/* I_VC_RES_CAP_REG_3 */

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_3_R1_MASK            (0x00007FFFU)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_3_R1_SHIFT           (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_3_R1_MAX             (0x00007FFFU)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_3_RST_MASK           (0x00008000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_3_RST_SHIFT          (0x0000000FU)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_3_RST_MAX            (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_3_R3_MASK            (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_3_R3_SHIFT           (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CAP_REG_3_R3_MAX             (0x0000FFFFU)

/* I_VC_RES_CTRL_REG_3 */

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_3_TVM0_MASK         (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_3_TVM0_SHIFT        (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_3_TVM0_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_3_TVM_MASK          (0x000000FEU)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_3_TVM_SHIFT         (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_3_TVM_MAX           (0x0000007FU)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_3_LPAT_MASK         (0x00010000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_3_LPAT_SHIFT        (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_3_LPAT_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_3_PARS_MASK         (0x000E0000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_3_PARS_SHIFT        (0x00000011U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_3_PARS_MAX          (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_3_R5_MASK           (0x00F00000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_3_R5_SHIFT          (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_3_R5_MAX            (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_3_VCI_MASK          (0x07000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_3_VCI_SHIFT         (0x00000018U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_3_VCI_MAX           (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_3_R6_MASK           (0x78000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_3_R6_SHIFT          (0x0000001BU)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_3_R6_MAX            (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_3_VCEN_MASK         (0x80000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_3_VCEN_SHIFT        (0x0000001FU)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_CTRL_REG_3_VCEN_MAX          (0x00000001U)

/* I_VC_RES_STS_REG_3 */

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_3_PATS_MASK          (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_3_PATS_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_3_PATS_MAX           (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_3_VCNP_MASK          (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_3_VCNP_SHIFT         (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_VC_CAP_STRUCT_I_VC_RES_STS_REG_3_VCNP_MAX           (0x00000001U)

/* I_L1_PM_EXT_CAP_HDR */

#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_EXT_CAP_HDR_PECID_MASK (0x0000FFFFU)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_EXT_CAP_HDR_PECID_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_EXT_CAP_HDR_PECID_MAX (0x0000FFFFU)

#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_EXT_CAP_HDR_CV_MASK   (0x000F0000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_EXT_CAP_HDR_CV_SHIFT  (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_EXT_CAP_HDR_CV_MAX    (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_EXT_CAP_HDR_NCO_MASK  (0xFFF00000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_EXT_CAP_HDR_NCO_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_EXT_CAP_HDR_NCO_MAX   (0x00000FFFU)

/* I_L1_PM_CAP */

#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CAP_L1PML12SUPP_MASK  (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CAP_L1PML12SUPP_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CAP_L1PML12SUPP_MAX   (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CAP_L1PML11SUPP_MASK  (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CAP_L1PML11SUPP_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CAP_L1PML11SUPP_MAX   (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CAP_L1ASPML12SUPP_MASK (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CAP_L1ASPML12SUPP_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CAP_L1ASPML12SUPP_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CAP_L1ASPML11SUPP_MASK (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CAP_L1ASPML11SUPP_SHIFT (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CAP_L1ASPML11SUPP_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CAP_L1PMSUPP_MASK     (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CAP_L1PMSUPP_SHIFT    (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CAP_L1PMSUPP_MAX      (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CAP_L1PRTCMMDRESTRTIME_MASK (0x0000FF00U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CAP_L1PRTCMMDRESTRTIME_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CAP_L1PRTCMMDRESTRTIME_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CAP_L1PRTPVRONSCALE_MASK (0x00030000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CAP_L1PRTPVRONSCALE_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CAP_L1PRTPVRONSCALE_MAX (0x00000003U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CAP_R0_MASK           (0x00F80000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CAP_R0_SHIFT          (0x00000013U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CAP_R0_MAX            (0x0000001FU)

/* I_L1_PM_CTRL_1 */

#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_1_L1PML12EN_MASK (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_1_L1PML12EN_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_1_L1PML12EN_MAX  (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_1_L1PML11EN_MASK (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_1_L1PML11EN_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_1_L1PML11EN_MAX  (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_1_L1ASPML12EN_MASK (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_1_L1ASPML12EN_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_1_L1ASPML12EN_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_1_L1ASPML11EN_MASK (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_1_L1ASPML11EN_SHIFT (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_1_L1ASPML11EN_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_1_L1CMMDRESTRTIME_MASK (0x0000FF00U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_1_L1CMMDRESTRTIME_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_1_L1CMMDRESTRTIME_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_1_L1THRSHLDVAL_MASK (0x03FF0000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_1_L1THRSHLDVAL_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_1_L1THRSHLDVAL_MAX (0x000003FFU)

#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_1_L1THRSHLDSC_MASK (0xE0000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_1_L1THRSHLDSC_SHIFT (0x0000001DU)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_1_L1THRSHLDSC_MAX (0x00000007U)

/* I_L1_PM_CTRL_2 */

#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_2_L1PWRONSC_MASK (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_2_L1PWRONSC_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_2_L1PWRONSC_MAX  (0x00000003U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_2_L1PWRONVAL_MASK (0x000000F8U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_2_L1PWRONVAL_SHIFT (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_L1_PM_CAP_STRUCT_I_L1_PM_CTRL_2_L1PWRONVAL_MAX (0x0000001FU)

/* I_DL_FEATURE_EXTENDED_CAPABILITY_HEADER_REG */

#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_EXTENDED_CAPABILITY_HEADER_REG_DLFCAPID_MASK (0x0000FFFFU)
#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_EXTENDED_CAPABILITY_HEADER_REG_DLFCAPID_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_EXTENDED_CAPABILITY_HEADER_REG_DLFCAPID_MAX (0x0000FFFFU)

#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_EXTENDED_CAPABILITY_HEADER_REG_DLFCAPVER_MASK (0x000F0000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_EXTENDED_CAPABILITY_HEADER_REG_DLFCAPVER_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_EXTENDED_CAPABILITY_HEADER_REG_DLFCAPVER_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_EXTENDED_CAPABILITY_HEADER_REG_DLFNXCAP_MASK (0xFFF00000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_EXTENDED_CAPABILITY_HEADER_REG_DLFNXCAP_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_EXTENDED_CAPABILITY_HEADER_REG_DLFNXCAP_MAX (0x00000FFFU)

/* I_DL_FEATURE_CAPABILITIES_REG */

#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_CAPABILITIES_REG_DLFCAPVER_MASK (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_CAPABILITIES_REG_DLFCAPVER_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_CAPABILITIES_REG_DLFCAPVER_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_CAPABILITIES_REG_R0_MASK (0x7FFFFFFEU)
#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_CAPABILITIES_REG_R0_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_CAPABILITIES_REG_R0_MAX (0x3FFFFFFFU)

#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_CAPABILITIES_REG_DLFEXEN_MASK (0x80000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_CAPABILITIES_REG_DLFEXEN_SHIFT (0x0000001FU)
#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_CAPABILITIES_REG_DLFEXEN_MAX (0x00000001U)

/* I_DL_FEATURE_STATUS_REG */

#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_STATUS_REG_RSFSUP_MASK (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_STATUS_REG_RSFSUP_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_STATUS_REG_RSFSUP_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_STATUS_REG_R0_MASK (0x007FFFFEU)
#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_STATUS_REG_R0_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_STATUS_REG_R0_MAX  (0x003FFFFFU)

#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_STATUS_REG_R23_MASK (0x00800000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_STATUS_REG_R23_SHIFT (0x00000017U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_STATUS_REG_R23_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_STATUS_REG_R1_MASK (0x7F000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_STATUS_REG_R1_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_STATUS_REG_R1_MAX  (0x0000007FU)

#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_STATUS_REG_RDLFSVAL_MASK (0x80000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_STATUS_REG_RDLFSVAL_SHIFT (0x0000001FU)
#define CSL_PCIE_RP_CORE_RC_I_REGF_DL_FEATURE_CAP_I_DL_FEATURE_STATUS_REG_RDLFSVAL_MAX (0x00000001U)

/* I_MARGINING_EXTENDED_CAPABILITY_HEADER_REG */

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_EXTENDED_CAPABILITY_HEADER_REG_MARCAPID_MASK (0x0000FFFFU)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_EXTENDED_CAPABILITY_HEADER_REG_MARCAPID_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_EXTENDED_CAPABILITY_HEADER_REG_MARCAPID_MAX (0x0000FFFFU)

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_EXTENDED_CAPABILITY_HEADER_REG_MARCAPVER_MASK (0x000F0000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_EXTENDED_CAPABILITY_HEADER_REG_MARCAPVER_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_EXTENDED_CAPABILITY_HEADER_REG_MARCAPVER_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_EXTENDED_CAPABILITY_HEADER_REG_MARNXCAP_MASK (0xFFF00000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_EXTENDED_CAPABILITY_HEADER_REG_MARNXCAP_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_EXTENDED_CAPABILITY_HEADER_REG_MARNXCAP_MAX (0x00000FFFU)

/* I_MARGINING_PORT_CAPABILITIES_STATUS_REG */

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_PORT_CAPABILITIES_STATUS_REG_MARUDS_MASK (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_PORT_CAPABILITIES_STATUS_REG_MARUDS_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_PORT_CAPABILITIES_STATUS_REG_MARUDS_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_PORT_CAPABILITIES_STATUS_REG_R0_MASK (0x0000FFFEU)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_PORT_CAPABILITIES_STATUS_REG_R0_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_PORT_CAPABILITIES_STATUS_REG_R0_MAX (0x00007FFFU)

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_PORT_CAPABILITIES_STATUS_REG_MRDY_MASK (0x00010000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_PORT_CAPABILITIES_STATUS_REG_MRDY_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_PORT_CAPABILITIES_STATUS_REG_MRDY_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_PORT_CAPABILITIES_STATUS_REG_MSRDY_MASK (0x00020000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_PORT_CAPABILITIES_STATUS_REG_MSRDY_SHIFT (0x00000011U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_PORT_CAPABILITIES_STATUS_REG_MSRDY_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_PORT_CAPABILITIES_STATUS_REG_R1_MASK (0xFFFC0000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_PORT_CAPABILITIES_STATUS_REG_R1_SHIFT (0x00000012U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_PORT_CAPABILITIES_STATUS_REG_R1_MAX (0x00003FFFU)

/* I_MARGINING_LANE_CONTROL_STATUS_REG0 */

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_RCVNUM_MASK (0x00000007U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_RCVNUM_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_RCVNUM_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_MRGTYP_MASK (0x00000038U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_MRGTYP_SHIFT (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_MRGTYP_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_USGMOD_MASK (0x00000040U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_USGMOD_SHIFT (0x00000006U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_USGMOD_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_R0_MASK (0x00000080U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_R0_SHIFT (0x00000007U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_R0_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_MRGPAY_MASK (0x0000FF00U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_MRGPAY_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_MRGPAY_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_RNSTS_MASK (0x00070000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_RNSTS_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_RNSTS_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_MTSTS_MASK (0x00380000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_MTSTS_SHIFT (0x00000013U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_MTSTS_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_UMSTS_MASK (0x00400000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_UMSTS_SHIFT (0x00000016U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_UMSTS_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_R1_MASK (0x00800000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_R1_SHIFT (0x00000017U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_R1_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_MPSTS_MASK (0xFF000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_MPSTS_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG0_MPSTS_MAX (0x000000FFU)

/* I_MARGINING_LANE_CONTROL_STATUS_REG1 */

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_RCVNUM_MASK (0x00000007U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_RCVNUM_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_RCVNUM_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_MRGTYP_MASK (0x00000038U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_MRGTYP_SHIFT (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_MRGTYP_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_USGMOD_MASK (0x00000040U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_USGMOD_SHIFT (0x00000006U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_USGMOD_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_R0_MASK (0x00000080U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_R0_SHIFT (0x00000007U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_R0_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_MRGPAY_MASK (0x0000FF00U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_MRGPAY_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_MRGPAY_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_RNSTS_MASK (0x00070000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_RNSTS_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_RNSTS_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_MTSTS_MASK (0x00380000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_MTSTS_SHIFT (0x00000013U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_MTSTS_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_UMSTS_MASK (0x00400000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_UMSTS_SHIFT (0x00000016U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_UMSTS_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_R1_MASK (0x00800000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_R1_SHIFT (0x00000017U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_R1_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_MPSTS_MASK (0xFF000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_MPSTS_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_MARGINING_CAP_I_MARGINING_LANE_CONTROL_STATUS_REG1_MPSTS_MAX (0x000000FFU)

/* I_PL_16GTS_EXTENDED_CAPABILITY_HEADER_REG */

#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_EXTENDED_CAPABILITY_HEADER_REG_PL16CAPID_MASK (0x0000FFFFU)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_EXTENDED_CAPABILITY_HEADER_REG_PL16CAPID_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_EXTENDED_CAPABILITY_HEADER_REG_PL16CAPID_MAX (0x0000FFFFU)

#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_EXTENDED_CAPABILITY_HEADER_REG_PL16CAPVER_MASK (0x000F0000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_EXTENDED_CAPABILITY_HEADER_REG_PL16CAPVER_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_EXTENDED_CAPABILITY_HEADER_REG_PL16CAPVER_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_EXTENDED_CAPABILITY_HEADER_REG_PL16NXCAP_MASK (0xFFF00000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_EXTENDED_CAPABILITY_HEADER_REG_PL16NXCAP_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_EXTENDED_CAPABILITY_HEADER_REG_PL16NXCAP_MAX (0x00000FFFU)

/* I_PL_16GTS_CAPABILITIES_REG */

#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_CAPABILITIES_REG_R0_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_CAPABILITIES_REG_R0_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_CAPABILITIES_REG_R0_MAX (0xFFFFFFFFU)

/* I_PL_16GTS_CONTROL_REG */

#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_CONTROL_REG_R0_MASK    (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_CONTROL_REG_R0_SHIFT   (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_CONTROL_REG_R0_MAX     (0xFFFFFFFFU)

/* I_PL_16GTS_STATUS_REG */

#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_STATUS_REG_EQC16_MASK  (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_STATUS_REG_EQC16_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_STATUS_REG_EQC16_MAX   (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_STATUS_REG_EP1S16_MASK (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_STATUS_REG_EP1S16_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_STATUS_REG_EP1S16_MAX  (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_STATUS_REG_EP2S16_MASK (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_STATUS_REG_EP2S16_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_STATUS_REG_EP2S16_MAX  (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_STATUS_REG_EP3S16_MASK (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_STATUS_REG_EP3S16_SHIFT (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_STATUS_REG_EP3S16_MAX  (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_STATUS_REG_LE16_MASK   (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_STATUS_REG_LE16_SHIFT  (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_STATUS_REG_LE16_MAX    (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_STATUS_REG_R0_MASK     (0xFFFFFFE0U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_STATUS_REG_R0_SHIFT    (0x00000005U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_STATUS_REG_R0_MAX      (0x07FFFFFFU)

/* I_PL_16GTS_LOCAL_DATA_PARITY_MISMATCH_STATUS_REG */

#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_LOCAL_DATA_PARITY_MISMATCH_STATUS_REG_LDPMS16_MASK (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_LOCAL_DATA_PARITY_MISMATCH_STATUS_REG_LDPMS16_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_LOCAL_DATA_PARITY_MISMATCH_STATUS_REG_LDPMS16_MAX (0x00000003U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_LOCAL_DATA_PARITY_MISMATCH_STATUS_REG_R0_MASK (0xFFFFFFFCU)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_LOCAL_DATA_PARITY_MISMATCH_STATUS_REG_R0_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_LOCAL_DATA_PARITY_MISMATCH_STATUS_REG_R0_MAX (0x3FFFFFFFU)

/* I_PL_16GTS_FIRST_RETIMER_DATA_PARITY_MISMATCH_STATUS_REG */

#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_FIRST_RETIMER_DATA_PARITY_MISMATCH_STATUS_REG_FRDPMS16_MASK (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_FIRST_RETIMER_DATA_PARITY_MISMATCH_STATUS_REG_FRDPMS16_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_FIRST_RETIMER_DATA_PARITY_MISMATCH_STATUS_REG_FRDPMS16_MAX (0x00000003U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_FIRST_RETIMER_DATA_PARITY_MISMATCH_STATUS_REG_R0_MASK (0xFFFFFFFCU)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_FIRST_RETIMER_DATA_PARITY_MISMATCH_STATUS_REG_R0_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_FIRST_RETIMER_DATA_PARITY_MISMATCH_STATUS_REG_R0_MAX (0x3FFFFFFFU)

/* I_PL_16GTS_SECOND_RETIMER_DATA_PARITY_MISMATCH_STATUS_REG */

#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_SECOND_RETIMER_DATA_PARITY_MISMATCH_STATUS_REG_SRDPMS16_MASK (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_SECOND_RETIMER_DATA_PARITY_MISMATCH_STATUS_REG_SRDPMS16_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_SECOND_RETIMER_DATA_PARITY_MISMATCH_STATUS_REG_SRDPMS16_MAX (0x00000003U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_SECOND_RETIMER_DATA_PARITY_MISMATCH_STATUS_REG_R0_MASK (0xFFFFFFFCU)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_SECOND_RETIMER_DATA_PARITY_MISMATCH_STATUS_REG_R0_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_SECOND_RETIMER_DATA_PARITY_MISMATCH_STATUS_REG_R0_MAX (0x3FFFFFFFU)

/* I_PL_16GTS_RESERVED_REG */

#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_RESERVED_REG_R0_MASK   (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_RESERVED_REG_R0_SHIFT  (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_RESERVED_REG_R0_MAX    (0xFFFFFFFFU)

/* I_PL_16GTS_LANE_EQUALIZATION_CONTROL_REG0 */

#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_LANE_EQUALIZATION_CONTROL_REG0_DPTP016_MASK (0x0000000FU)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_LANE_EQUALIZATION_CONTROL_REG0_DPTP016_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_LANE_EQUALIZATION_CONTROL_REG0_DPTP016_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_LANE_EQUALIZATION_CONTROL_REG0_UPTP016_MASK (0x000000F0U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_LANE_EQUALIZATION_CONTROL_REG0_UPTP016_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_LANE_EQUALIZATION_CONTROL_REG0_UPTP016_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_LANE_EQUALIZATION_CONTROL_REG0_DPTP116_MASK (0x00000F00U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_LANE_EQUALIZATION_CONTROL_REG0_DPTP116_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_LANE_EQUALIZATION_CONTROL_REG0_DPTP116_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_LANE_EQUALIZATION_CONTROL_REG0_UPTP116_MASK (0x0000F000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_LANE_EQUALIZATION_CONTROL_REG0_UPTP116_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_LANE_EQUALIZATION_CONTROL_REG0_UPTP116_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_LANE_EQUALIZATION_CONTROL_REG0_R16_MASK (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_LANE_EQUALIZATION_CONTROL_REG0_R16_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PL_16GTS_CAP_I_PL_16GTS_LANE_EQUALIZATION_CONTROL_REG0_R16_MAX (0x0000FFFFU)

/* I_PTM_EXTENDED_CAPABILITY_HEADER_REG */

#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_EXTENDED_CAPABILITY_HEADER_REG_PTMCAPID_MASK (0x0000FFFFU)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_EXTENDED_CAPABILITY_HEADER_REG_PTMCAPID_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_EXTENDED_CAPABILITY_HEADER_REG_PTMCAPID_MAX (0x0000FFFFU)

#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_EXTENDED_CAPABILITY_HEADER_REG_PTMCAPVER_MASK (0x000F0000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_EXTENDED_CAPABILITY_HEADER_REG_PTMCAPVER_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_EXTENDED_CAPABILITY_HEADER_REG_PTMCAPVER_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_EXTENDED_CAPABILITY_HEADER_REG_PTMNXCAP_MASK (0xFFF00000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_EXTENDED_CAPABILITY_HEADER_REG_PTMNXCAP_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_EXTENDED_CAPABILITY_HEADER_REG_PTMNXCAP_MAX (0x00000FFFU)

/* I_PTM_CAPABILITIES_REG */

#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CAPABILITIES_REG_PTMRQCAP_MASK   (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CAPABILITIES_REG_PTMRQCAP_SHIFT  (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CAPABILITIES_REG_PTMRQCAP_MAX    (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CAPABILITIES_REG_PTMRSCAP_MASK   (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CAPABILITIES_REG_PTMRSCAP_SHIFT  (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CAPABILITIES_REG_PTMRSCAP_MAX    (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CAPABILITIES_REG_PTMRTCAP_MASK   (0x00000004U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CAPABILITIES_REG_PTMRTCAP_SHIFT  (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CAPABILITIES_REG_PTMRTCAP_MAX    (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CAPABILITIES_REG_R3_MASK         (0x000000F8U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CAPABILITIES_REG_R3_SHIFT        (0x00000003U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CAPABILITIES_REG_R3_MAX          (0x0000001FU)

#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CAPABILITIES_REG_LOCCLKGR_MASK   (0x0000FF00U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CAPABILITIES_REG_LOCCLKGR_SHIFT  (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CAPABILITIES_REG_LOCCLKGR_MAX    (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CAPABILITIES_REG_R16_MASK        (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CAPABILITIES_REG_R16_SHIFT       (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CAPABILITIES_REG_R16_MAX         (0x0000FFFFU)

/* I_PTM_CONTROL_REG */

#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CONTROL_REG_PTMEN_MASK           (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CONTROL_REG_PTMEN_SHIFT          (0x00000000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CONTROL_REG_PTMEN_MAX            (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CONTROL_REG_RTSEL_MASK           (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CONTROL_REG_RTSEL_SHIFT          (0x00000001U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CONTROL_REG_RTSEL_MAX            (0x00000001U)

#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CONTROL_REG_R2_MASK              (0x000000FCU)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CONTROL_REG_R2_SHIFT             (0x00000002U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CONTROL_REG_R2_MAX               (0x0000003FU)

#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CONTROL_REG_EFFGRN_MASK          (0x0000FF00U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CONTROL_REG_EFFGRN_SHIFT         (0x00000008U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CONTROL_REG_EFFGRN_MAX           (0x000000FFU)

#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CONTROL_REG_R16_MASK             (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CONTROL_REG_R16_SHIFT            (0x00000010U)
#define CSL_PCIE_RP_CORE_RC_I_REGF_PTM_CAP_I_PTM_CONTROL_REG_R16_MAX              (0x0000FFFFU)

/* I_PL_CONFIG_0_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_LS_MASK         (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_LS_SHIFT        (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_LS_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_NLC_MASK        (0x00000006U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_NLC_SHIFT       (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_NLC_MAX         (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_NS_MASK         (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_NS_SHIFT        (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_NS_MAX          (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_LTD_MASK        (0x00000020U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_LTD_SHIFT       (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_LTD_MAX         (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_APER_MASK       (0x00000040U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_APER_SHIFT      (0x00000006U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_APER_MAX        (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_TSS_MASK        (0x00000080U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_TSS_SHIFT       (0x00000007U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_TSS_MAX         (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_RFC_MASK        (0x0000FF00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_RFC_SHIFT       (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_RFC_MAX         (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_RLID_MASK       (0x00FF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_RLID_SHIFT      (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_RLID_MAX        (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_LTSSM_MASK      (0x3F000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_LTSSM_SHIFT     (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_LTSSM_MAX       (0x0000003FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_R0_MASK         (0x40000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_R0_SHIFT        (0x0000001EU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_R0_MAX          (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_MLE_MASK        (0x80000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_MLE_SHIFT       (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_0_REG_MLE_MAX         (0x00000001U)

/* I_PL_CONFIG_1_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_1_REG_TLI_MASK        (0x000000FFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_1_REG_TLI_SHIFT       (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_1_REG_TLI_MAX         (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_1_REG_TFC1_MASK       (0x0000FF00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_1_REG_TFC1_SHIFT      (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_1_REG_TFC1_MAX        (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_1_REG_TFC2_MASK       (0x00FF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_1_REG_TFC2_SHIFT      (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_1_REG_TFC2_MAX        (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_1_REG_TFC3_MASK       (0xFF000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_1_REG_TFC3_SHIFT      (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PL_CONFIG_1_REG_TFC3_MAX        (0x000000FFU)

/* I_DLL_TMR_CONFIG_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DLL_TMR_CONFIG_REG_TSRT_MASK    (0x000001FFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DLL_TMR_CONFIG_REG_TSRT_SHIFT   (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DLL_TMR_CONFIG_REG_TSRT_MAX     (0x000001FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DLL_TMR_CONFIG_REG_R9_MASK      (0x0000FE00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DLL_TMR_CONFIG_REG_R9_SHIFT     (0x00000009U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DLL_TMR_CONFIG_REG_R9_MAX       (0x0000007FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DLL_TMR_CONFIG_REG_RSART_MASK   (0x01FF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DLL_TMR_CONFIG_REG_RSART_SHIFT  (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DLL_TMR_CONFIG_REG_RSART_MAX    (0x000001FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DLL_TMR_CONFIG_REG_R25_MASK     (0xFE000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DLL_TMR_CONFIG_REG_R25_SHIFT    (0x00000019U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DLL_TMR_CONFIG_REG_R25_MAX      (0x0000007FU)

/* I_RCV_CRED_LIM_0_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_PPC_MASK     (0x00000FFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_PPC_SHIFT    (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_PPC_MAX      (0x00000FFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_PHC_MASK     (0x000FF000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_PHC_SHIFT    (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_PHC_MAX      (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_NPPC_MASK    (0xFFF00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_NPPC_SHIFT   (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_NPPC_MAX     (0x00000FFFU)

/* I_RCV_CRED_LIM_1_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_NPHCL_MASK   (0x000000FFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_NPHCL_SHIFT  (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_NPHCL_MAX    (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_CPC_MASK     (0x000FFF00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_CPC_SHIFT    (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_CPC_MAX      (0x00000FFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_R2_MASK      (0x00F00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_R2_SHIFT     (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_R2_MAX       (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_CHC_MASK     (0xFF000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_CHC_SHIFT    (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_CHC_MAX      (0x000000FFU)

/* I_TRANSM_CRED_LIM_0_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_PPC_MASK  (0x00000FFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_PPC_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_PPC_MAX   (0x00000FFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_PHC_MASK  (0x000FF000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_PHC_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_PHC_MAX   (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_NPPC_MASK (0xFFF00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_NPPC_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_NPPC_MAX  (0x00000FFFU)

/* I_TRANSM_CRED_LIM_1_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_NPHC_MASK (0x000000FFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_NPHC_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_NPHC_MAX  (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_CPC_MASK  (0x000FFF00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_CPC_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_CPC_MAX   (0x00000FFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_R3_MASK   (0x00F00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_R3_SHIFT  (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_R3_MAX    (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_CHC_MASK  (0xFF000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_CHC_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_CHC_MAX   (0x000000FFU)

/* I_TRANSM_CRED_UPDATE_INT_CONFIG_0_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_UPDATE_INT_CONFIG_0_REG_MPUI_MASK (0x0000FFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_UPDATE_INT_CONFIG_0_REG_MPUI_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_UPDATE_INT_CONFIG_0_REG_MPUI_MAX (0x0000FFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_UPDATE_INT_CONFIG_0_REG_MNUI_MASK (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_UPDATE_INT_CONFIG_0_REG_MNUI_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_UPDATE_INT_CONFIG_0_REG_MNUI_MAX (0x0000FFFFU)

/* I_TRANSM_CRED_UPDATE_INT_CONFIG_1_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_UPDATE_INT_CONFIG_1_REG_CUI_MASK (0x0000FFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_UPDATE_INT_CONFIG_1_REG_CUI_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_UPDATE_INT_CONFIG_1_REG_CUI_MAX (0x0000FFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_UPDATE_INT_CONFIG_1_REG_MUI_MASK (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_UPDATE_INT_CONFIG_1_REG_MUI_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_UPDATE_INT_CONFIG_1_REG_MUI_MAX (0x0000FFFFU)

/* I_L0S_TIMEOUT_LIMIT_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_L0S_TIMEOUT_LIMIT_REG_LT_MASK   (0x0000FFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_L0S_TIMEOUT_LIMIT_REG_LT_SHIFT  (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_L0S_TIMEOUT_LIMIT_REG_LT_MAX    (0x0000FFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_L0S_TIMEOUT_LIMIT_REG_R4_MASK   (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_L0S_TIMEOUT_LIMIT_REG_R4_SHIFT  (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_L0S_TIMEOUT_LIMIT_REG_R4_MAX    (0x0000FFFFU)

/* I_TRANSMIT_TLP_COUNT_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSMIT_TLP_COUNT_REG_TTC_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSMIT_TLP_COUNT_REG_TTC_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSMIT_TLP_COUNT_REG_TTC_MAX  (0xFFFFFFFFU)

/* I_TRANSMIT_TLP_PAYLOAD_DWORD_COUNT_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSMIT_TLP_PAYLOAD_DWORD_COUNT_REG_TTPBC_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSMIT_TLP_PAYLOAD_DWORD_COUNT_REG_TTPBC_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSMIT_TLP_PAYLOAD_DWORD_COUNT_REG_TTPBC_MAX (0xFFFFFFFFU)

/* I_RECEIVE_TLP_COUNT_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RECEIVE_TLP_COUNT_REG_RTC_MASK  (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RECEIVE_TLP_COUNT_REG_RTC_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RECEIVE_TLP_COUNT_REG_RTC_MAX   (0xFFFFFFFFU)

/* I_RECEIVE_TLP_PAYLOAD_DWORD_COUNT_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RECEIVE_TLP_PAYLOAD_DWORD_COUNT_REG_RTPDC_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RECEIVE_TLP_PAYLOAD_DWORD_COUNT_REG_RTPDC_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RECEIVE_TLP_PAYLOAD_DWORD_COUNT_REG_RTPDC_MAX (0xFFFFFFFFU)

/* I_COMPLN_TMOUT_LIM_0_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_COMPLN_TMOUT_LIM_0_REG_CTL_MASK (0x00FFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_COMPLN_TMOUT_LIM_0_REG_CTL_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_COMPLN_TMOUT_LIM_0_REG_CTL_MAX  (0x00FFFFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_COMPLN_TMOUT_LIM_0_REG_R5_MASK  (0xFF000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_COMPLN_TMOUT_LIM_0_REG_R5_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_COMPLN_TMOUT_LIM_0_REG_R5_MAX   (0x000000FFU)

/* I_COMPLN_TMOUT_LIM_1_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_COMPLN_TMOUT_LIM_1_REG_CTL_MASK (0x0FFFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_COMPLN_TMOUT_LIM_1_REG_CTL_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_COMPLN_TMOUT_LIM_1_REG_CTL_MAX  (0x0FFFFFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_COMPLN_TMOUT_LIM_1_REG_R6_MASK  (0xF0000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_COMPLN_TMOUT_LIM_1_REG_R6_SHIFT (0x0000001CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_COMPLN_TMOUT_LIM_1_REG_R6_MAX   (0x0000000FU)

/* I_L1_ST_REENTRY_DELAY_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_L1_ST_REENTRY_DELAY_REG_L1RD_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_L1_ST_REENTRY_DELAY_REG_L1RD_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_L1_ST_REENTRY_DELAY_REG_L1RD_MAX (0xFFFFFFFFU)

/* I_VENDOR_ID_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_VENDOR_ID_REG_VID_MASK          (0x0000FFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_VENDOR_ID_REG_VID_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_VENDOR_ID_REG_VID_MAX           (0x0000FFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_VENDOR_ID_REG_SVID_MASK         (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_VENDOR_ID_REG_SVID_SHIFT        (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_VENDOR_ID_REG_SVID_MAX          (0x0000FFFFU)

/* I_ASPM_L1_ENTRY_TMOUT_DELAY_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASPM_L1_ENTRY_TMOUT_DELAY_REG_L1T_MASK (0x000FFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASPM_L1_ENTRY_TMOUT_DELAY_REG_L1T_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASPM_L1_ENTRY_TMOUT_DELAY_REG_L1T_MAX (0x000FFFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASPM_L1_ENTRY_TMOUT_DELAY_REG_R7_MASK (0x7FF00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASPM_L1_ENTRY_TMOUT_DELAY_REG_R7_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASPM_L1_ENTRY_TMOUT_DELAY_REG_R7_MAX (0x000007FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASPM_L1_ENTRY_TMOUT_DELAY_REG_DISLNRXCHK_MASK (0x80000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASPM_L1_ENTRY_TMOUT_DELAY_REG_DISLNRXCHK_SHIFT (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASPM_L1_ENTRY_TMOUT_DELAY_REG_DISLNRXCHK_MAX (0x00000001U)

/* I_PME_TURNOFF_ACK_DELAY_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PME_TURNOFF_ACK_DELAY_REG_PTOAD_MASK (0x0000FFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PME_TURNOFF_ACK_DELAY_REG_PTOAD_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PME_TURNOFF_ACK_DELAY_REG_PTOAD_MAX (0x0000FFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PME_TURNOFF_ACK_DELAY_REG_R7_MASK (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PME_TURNOFF_ACK_DELAY_REG_R7_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PME_TURNOFF_ACK_DELAY_REG_R7_MAX (0x0000FFFFU)

/* I_LINKWIDTH_CONTROL_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LINKWIDTH_CONTROL_REG_TLM_MASK  (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LINKWIDTH_CONTROL_REG_TLM_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LINKWIDTH_CONTROL_REG_TLM_MAX   (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LINKWIDTH_CONTROL_REG_R0_MASK   (0x0000FFFCU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LINKWIDTH_CONTROL_REG_R0_SHIFT  (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LINKWIDTH_CONTROL_REG_R0_MAX    (0x00003FFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LINKWIDTH_CONTROL_REG_RL_MASK   (0x00010000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LINKWIDTH_CONTROL_REG_RL_SHIFT  (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LINKWIDTH_CONTROL_REG_RL_MAX    (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LINKWIDTH_CONTROL_REG_R1_MASK   (0x00FE0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LINKWIDTH_CONTROL_REG_R1_SHIFT  (0x00000011U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LINKWIDTH_CONTROL_REG_R1_MAX    (0x0000007FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LINKWIDTH_CONTROL_REG_EPTLS_MASK (0x03000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LINKWIDTH_CONTROL_REG_EPTLS_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LINKWIDTH_CONTROL_REG_EPTLS_MAX (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LINKWIDTH_CONTROL_REG_R2_MASK   (0x7C000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LINKWIDTH_CONTROL_REG_R2_SHIFT  (0x0000001AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LINKWIDTH_CONTROL_REG_R2_MAX    (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LINKWIDTH_CONTROL_REG_EPLSCRL_MASK (0x80000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LINKWIDTH_CONTROL_REG_EPLSCRL_SHIFT (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LINKWIDTH_CONTROL_REG_EPLSCRL_MAX (0x00000001U)

/* I_MULTI_VC_CONROL_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MULTI_VC_CONROL_REG_DMAAM_MASK  (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MULTI_VC_CONROL_REG_DMAAM_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MULTI_VC_CONROL_REG_DMAAM_MAX   (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MULTI_VC_CONROL_REG_WAIT_4_ALL_VC_CC_RDY_MASK (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MULTI_VC_CONROL_REG_WAIT_4_ALL_VC_CC_RDY_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MULTI_VC_CONROL_REG_WAIT_4_ALL_VC_CC_RDY_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MULTI_VC_CONROL_REG_RES2_MASK   (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MULTI_VC_CONROL_REG_RES2_SHIFT  (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MULTI_VC_CONROL_REG_RES2_MAX    (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MULTI_VC_CONROL_REG_RES4_MASK   (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MULTI_VC_CONROL_REG_RES4_SHIFT  (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MULTI_VC_CONROL_REG_RES4_MAX    (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MULTI_VC_CONROL_REG_R31_MASK    (0xFFFFFFE0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MULTI_VC_CONROL_REG_R31_SHIFT   (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MULTI_VC_CONROL_REG_R31_MAX     (0x07FFFFFFU)

/* I_SRIS_CONTROL_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SRIS_CONTROL_REG_SRISE_MASK     (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SRIS_CONTROL_REG_SRISE_SHIFT    (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SRIS_CONTROL_REG_SRISE_MAX      (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SRIS_CONTROL_REG_R31_MASK       (0xFFFFFFFEU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SRIS_CONTROL_REG_R31_SHIFT      (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SRIS_CONTROL_REG_R31_MAX        (0x7FFFFFFFU)

/* I_RCV_CRED_LIM_0_REG_VC1 */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC1_PPC_MASK (0x00000FFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC1_PPC_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC1_PPC_MAX  (0x00000FFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC1_PHC_MASK (0x000FF000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC1_PHC_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC1_PHC_MAX  (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC1_NPPC_MASK (0xFFF00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC1_NPPC_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC1_NPPC_MAX (0x00000FFFU)

/* I_RCV_CRED_LIM_1_REG_VC1 */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC1_NPHCL_MASK (0x000000FFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC1_NPHCL_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC1_NPHCL_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC1_CPC_MASK (0x000FFF00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC1_CPC_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC1_CPC_MAX  (0x00000FFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC1_R2_MASK  (0x00F00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC1_R2_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC1_R2_MAX   (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC1_CHC_MASK (0xFF000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC1_CHC_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC1_CHC_MAX  (0x000000FFU)

/* I_TRANSM_CRED_LIM_0_REG_VC1 */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC1_PPC_MASK (0x00000FFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC1_PPC_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC1_PPC_MAX (0x00000FFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC1_PHC_MASK (0x000FF000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC1_PHC_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC1_PHC_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC1_NPPC_MASK (0xFFF00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC1_NPPC_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC1_NPPC_MAX (0x00000FFFU)

/* I_TRANSM_CRED_LIM_1_REG_VC1 */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC1_NPHC_MASK (0x000000FFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC1_NPHC_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC1_NPHC_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC1_CPC_MASK (0x000FFF00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC1_CPC_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC1_CPC_MAX (0x00000FFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC1_R3_MASK (0x00F00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC1_R3_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC1_R3_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC1_CHC_MASK (0xFF000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC1_CHC_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC1_CHC_MAX (0x000000FFU)

/* I_RCV_CRED_LIM_0_REG_VC2 */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC2_PPC_MASK (0x00000FFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC2_PPC_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC2_PPC_MAX  (0x00000FFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC2_PHC_MASK (0x000FF000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC2_PHC_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC2_PHC_MAX  (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC2_NPPC_MASK (0xFFF00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC2_NPPC_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC2_NPPC_MAX (0x00000FFFU)

/* I_RCV_CRED_LIM_1_REG_VC2 */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC2_NPHCL_MASK (0x000000FFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC2_NPHCL_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC2_NPHCL_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC2_CPC_MASK (0x000FFF00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC2_CPC_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC2_CPC_MAX  (0x00000FFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC2_R2_MASK  (0x00F00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC2_R2_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC2_R2_MAX   (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC2_CHC_MASK (0xFF000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC2_CHC_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC2_CHC_MAX  (0x000000FFU)

/* I_TRANSM_CRED_LIM_0_REG_VC2 */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC2_PPC_MASK (0x00000FFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC2_PPC_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC2_PPC_MAX (0x00000FFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC2_PHC_MASK (0x000FF000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC2_PHC_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC2_PHC_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC2_NPPC_MASK (0xFFF00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC2_NPPC_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC2_NPPC_MAX (0x00000FFFU)

/* I_TRANSM_CRED_LIM_1_REG_VC2 */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC2_NPHC_MASK (0x000000FFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC2_NPHC_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC2_NPHC_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC2_CPC_MASK (0x000FFF00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC2_CPC_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC2_CPC_MAX (0x00000FFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC2_R3_MASK (0x00F00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC2_R3_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC2_R3_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC2_CHC_MASK (0xFF000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC2_CHC_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC2_CHC_MAX (0x000000FFU)

/* I_RCV_CRED_LIM_0_REG_VC3 */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC3_PPC_MASK (0x00000FFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC3_PPC_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC3_PPC_MAX  (0x00000FFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC3_PHC_MASK (0x000FF000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC3_PHC_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC3_PHC_MAX  (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC3_NPPC_MASK (0xFFF00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC3_NPPC_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_0_REG_VC3_NPPC_MAX (0x00000FFFU)

/* I_RCV_CRED_LIM_1_REG_VC3 */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC3_NPHCL_MASK (0x000000FFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC3_NPHCL_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC3_NPHCL_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC3_CPC_MASK (0x000FFF00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC3_CPC_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC3_CPC_MAX  (0x00000FFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC3_R2_MASK  (0x00F00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC3_R2_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC3_R2_MAX   (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC3_CHC_MASK (0xFF000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC3_CHC_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RCV_CRED_LIM_1_REG_VC3_CHC_MAX  (0x000000FFU)

/* I_TRANSM_CRED_LIM_0_REG_VC3 */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC3_PPC_MASK (0x00000FFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC3_PPC_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC3_PPC_MAX (0x00000FFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC3_PHC_MASK (0x000FF000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC3_PHC_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC3_PHC_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC3_NPPC_MASK (0xFFF00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC3_NPPC_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_0_REG_VC3_NPPC_MAX (0x00000FFFU)

/* I_TRANSM_CRED_LIM_1_REG_VC3 */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC3_NPHC_MASK (0x000000FFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC3_NPHC_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC3_NPHC_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC3_CPC_MASK (0x000FFF00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC3_CPC_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC3_CPC_MAX (0x00000FFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC3_R3_MASK (0x00F00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC3_R3_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC3_R3_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC3_CHC_MASK (0xFF000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC3_CHC_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_TRANSM_CRED_LIM_1_REG_VC3_CHC_MAX (0x000000FFU)

/* I_FC_INIT_DELAY_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_FC_INIT_DELAY_REG_FCINITDLY_MASK (0x0000FFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_FC_INIT_DELAY_REG_FCINITDLY_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_FC_INIT_DELAY_REG_FCINITDLY_MAX (0x0000FFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_FC_INIT_DELAY_REG_R4_MASK       (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_FC_INIT_DELAY_REG_R4_SHIFT      (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_FC_INIT_DELAY_REG_R4_MAX        (0x0000FFFFU)

/* I_SHDW_HDR_LOG_0_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_HDR_LOG_0_REG_SHDW_HDR_LOG_0_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_HDR_LOG_0_REG_SHDW_HDR_LOG_0_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_HDR_LOG_0_REG_SHDW_HDR_LOG_0_MAX (0xFFFFFFFFU)

/* I_SHDW_HDR_LOG_1_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_HDR_LOG_1_REG_SHDW_HDR_LOG_1_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_HDR_LOG_1_REG_SHDW_HDR_LOG_1_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_HDR_LOG_1_REG_SHDW_HDR_LOG_1_MAX (0xFFFFFFFFU)

/* I_SHDW_HDR_LOG_2_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_HDR_LOG_2_REG_SHDW_HDR_LOG_2_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_HDR_LOG_2_REG_SHDW_HDR_LOG_2_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_HDR_LOG_2_REG_SHDW_HDR_LOG_2_MAX (0xFFFFFFFFU)

/* I_SHDW_HDR_LOG_3_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_HDR_LOG_3_REG_SHDW_HDR_LOG_3_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_HDR_LOG_3_REG_SHDW_HDR_LOG_3_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_HDR_LOG_3_REG_SHDW_HDR_LOG_3_MAX (0xFFFFFFFFU)

/* I_SHDW_FUNC_NUM_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_FUNC_NUM_REG_SHDW_FUNC_NUM_MASK (0x000000FFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_FUNC_NUM_REG_SHDW_FUNC_NUM_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_FUNC_NUM_REG_SHDW_FUNC_NUM_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_FUNC_NUM_REG_R0_MASK       (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_FUNC_NUM_REG_R0_SHIFT      (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_FUNC_NUM_REG_R0_MAX        (0x00FFFFFFU)

/* I_SHDW_UR_ERR_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_UR_ERR_REG_P_UR_ERR_MASK   (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_UR_ERR_REG_P_UR_ERR_SHIFT  (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_UR_ERR_REG_P_UR_ERR_MAX    (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_UR_ERR_REG_NP_UR_ERR_MASK  (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_UR_ERR_REG_NP_UR_ERR_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_UR_ERR_REG_NP_UR_ERR_MAX   (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_UR_ERR_REG_R0_MASK         (0xFFFFFFFCU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_UR_ERR_REG_R0_SHIFT        (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SHDW_UR_ERR_REG_R0_MAX          (0x3FFFFFFFU)

/* I_PM_CLK_FREQUENCY_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PM_CLK_FREQUENCY_REG_PMCLKFRQ_MASK (0x000000FFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PM_CLK_FREQUENCY_REG_PMCLKFRQ_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PM_CLK_FREQUENCY_REG_PMCLKFRQ_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PM_CLK_FREQUENCY_REG_R0_MASK    (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PM_CLK_FREQUENCY_REG_R0_SHIFT   (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PM_CLK_FREQUENCY_REG_R0_MAX     (0x00FFFFFFU)

/* I_DEBUG_DLLP_COUNT_GEN1_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_DLLP_COUNT_GEN1_REG_DLLPCNT1_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_DLLP_COUNT_GEN1_REG_DLLPCNT1_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_DLLP_COUNT_GEN1_REG_DLLPCNT1_MAX (0xFFFFFFFFU)

/* I_DEBUG_DLLP_COUNT_GEN2_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_DLLP_COUNT_GEN2_REG_DLLPCNT2_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_DLLP_COUNT_GEN2_REG_DLLPCNT2_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_DLLP_COUNT_GEN2_REG_DLLPCNT2_MAX (0xFFFFFFFFU)

/* I_DEBUG_DLLP_COUNT_GEN3_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_DLLP_COUNT_GEN3_REG_DLLPCNT3_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_DLLP_COUNT_GEN3_REG_DLLPCNT3_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_DLLP_COUNT_GEN3_REG_DLLPCNT3_MAX (0xFFFFFFFFU)

/* I_DEBUG_DLLP_COUNT_GEN4_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_DLLP_COUNT_GEN4_REG_DLLPCNT4_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_DLLP_COUNT_GEN4_REG_DLLPCNT4_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_DLLP_COUNT_GEN4_REG_DLLPCNT4_MAX (0xFFFFFFFFU)

/* I_VENDOR_DEFINED_MESSAGE_TAG_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_VENDOR_DEFINED_MESSAGE_TAG_REG_VDMTAG_MASK (0x000000FFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_VENDOR_DEFINED_MESSAGE_TAG_REG_VDMTAG_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_VENDOR_DEFINED_MESSAGE_TAG_REG_VDMTAG_MAX (0x000000FFU)

/* I_NEGOTIATED_LANE_MAP_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_NEGOTIATED_LANE_MAP_REG_NLM_MASK (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_NEGOTIATED_LANE_MAP_REG_NLM_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_NEGOTIATED_LANE_MAP_REG_NLM_MAX (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_NEGOTIATED_LANE_MAP_REG_R70_MASK (0x0000FFFCU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_NEGOTIATED_LANE_MAP_REG_R70_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_NEGOTIATED_LANE_MAP_REG_R70_MAX (0x00003FFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_NEGOTIATED_LANE_MAP_REG_LRS_MASK (0x00010000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_NEGOTIATED_LANE_MAP_REG_LRS_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_NEGOTIATED_LANE_MAP_REG_LRS_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_NEGOTIATED_LANE_MAP_REG_R71_MASK (0xFFFE0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_NEGOTIATED_LANE_MAP_REG_R71_SHIFT (0x00000011U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_NEGOTIATED_LANE_MAP_REG_R71_MAX (0x00007FFFU)

/* I_RECEIVE_FTS_COUNT_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RECEIVE_FTS_COUNT_REG_RFC5S_MASK (0x000000FFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RECEIVE_FTS_COUNT_REG_RFC5S_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RECEIVE_FTS_COUNT_REG_RFC5S_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RECEIVE_FTS_COUNT_REG_RFC8S_MASK (0x0000FF00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RECEIVE_FTS_COUNT_REG_RFC8S_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RECEIVE_FTS_COUNT_REG_RFC8S_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RECEIVE_FTS_COUNT_REG_RFC16S_MASK (0x00FF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RECEIVE_FTS_COUNT_REG_RFC16S_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RECEIVE_FTS_COUNT_REG_RFC16S_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RECEIVE_FTS_COUNT_REG_R24_MASK  (0xFF000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RECEIVE_FTS_COUNT_REG_R24_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RECEIVE_FTS_COUNT_REG_R24_MAX   (0x000000FFU)

/* I_DEBUG_MUX_CONTROL_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_MS_MASK   (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_MS_SHIFT  (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_MS_MAX    (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_R6_MASK   (0x00000060U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_R6_SHIFT  (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_R6_MAX    (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_R77_MASK  (0x00000080U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_R77_SHIFT (0x00000007U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_R77_MAX   (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DIDBOC_MASK (0x00000100U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DIDBOC_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DIDBOC_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_MSIVCMS_MASK (0x00000200U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_MSIVCMS_SHIFT (0x00000009U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_MSIVCMS_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_R1010_MASK (0x00000400U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_R1010_SHIFT (0x0000000AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_R1010_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_R1111_MASK (0x00000800U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_R1111_SHIFT (0x0000000BU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_R1111_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_R1212_MASK (0x00001000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_R1212_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_R1212_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_R1313_MASK (0x00002000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_R1313_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_R1313_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DSSPLM_MASK (0x00004000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DSSPLM_SHIFT (0x0000000EU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DSSPLM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_FDS_MASK  (0x00008000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_FDS_SHIFT (0x0000000FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_FDS_MAX   (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_AWRPRI_MASK (0x00010000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_AWRPRI_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_AWRPRI_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_HPRSUPP_MASK (0x00020000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_HPRSUPP_SHIFT (0x00000011U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_HPRSUPP_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DOASFC_MASK (0x00040000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DOASFC_SHIFT (0x00000012U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DOASFC_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DIOAEFC_MASK (0x00080000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DIOAEFC_SHIFT (0x00000013U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DIOAEFC_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DCIVMC_MASK (0x00100000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DCIVMC_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DCIVMC_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DSHEC_MASK (0x00200000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DSHEC_SHIFT (0x00000015U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DSHEC_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DLRFE_MASK (0x00400000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DLRFE_SHIFT (0x00000016U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DLRFE_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DLUC_MASK (0x00800000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DLUC_SHIFT (0x00000017U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DLUC_MAX  (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_EFLT_MASK (0x01000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_EFLT_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_EFLT_MAX  (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_ESPC_MASK (0x02000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_ESPC_SHIFT (0x00000019U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_ESPC_MAX  (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_IEDPPE_MASK (0x04000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_IEDPPE_SHIFT (0x0000001AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_IEDPPE_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DGLUS_MASK (0x08000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DGLUS_SHIFT (0x0000001BU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DGLUS_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DEI_MASK  (0x10000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DEI_SHIFT (0x0000001CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DEI_MAX   (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DFCUT_MASK (0x20000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DFCUT_SHIFT (0x0000001DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DFCUT_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DOC_MASK  (0x40000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DOC_SHIFT (0x0000001EU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_DOC_MAX   (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_EFSRTCA_MASK (0x80000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_EFSRTCA_SHIFT (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_REG_EFSRTCA_MAX (0x00000001U)

/* I_LOCAL_ERROR_STATUS_REGISTER */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_PRFPE_MASK (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_PRFPE_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_PRFPE_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_CRFPE_MASK (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_CRFPE_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_CRFPE_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_RRPE_MASK (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_RRPE_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_RRPE_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_PRFO_MASK (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_PRFO_SHIFT (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_PRFO_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_CRFO_MASK (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_CRFO_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_CRFO_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_RT_MASK (0x00000020U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_RT_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_RT_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_RTR_MASK (0x00000040U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_RTR_SHIFT (0x00000006U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_RTR_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_PE_MASK (0x00000080U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_PE_SHIFT (0x00000007U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_PE_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_MTR_MASK (0x00000100U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_MTR_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_MTR_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_UCR_MASK (0x00000200U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_UCR_SHIFT (0x00000009U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_UCR_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_FCE_MASK (0x00000400U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_FCE_SHIFT (0x0000000AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_FCE_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_CT_MASK (0x00000800U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_CT_SHIFT (0x0000000BU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_CT_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_R12_MASK (0x00001000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_R12_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_R12_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_R13_MASK (0x0001E000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_R13_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_R13_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_EEPE_MASK (0x00020000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_EEPE_SHIFT (0x00000011U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_EEPE_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_UTC_MASK (0x00040000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_UTC_SHIFT (0x00000012U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_UTC_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_MMVC_MASK (0x00080000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_MMVC_SHIFT (0x00000013U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_MMVC_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_R22_MASK (0x00100000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_R22_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_R22_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_HAWCD_MASK (0x00200000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_HAWCD_SHIFT (0x00000015U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_HAWCD_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_R24_MASK (0x01C00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_R24_SHIFT (0x00000016U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_R24_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_MSIXMSKST_MASK (0x02000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_MSIXMSKST_SHIFT (0x00000019U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_MSIXMSKST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_R27_MASK (0x0C000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_R27_SHIFT (0x0000001AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_R27_MAX (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_AXIMASTER_DIB_ER_UN_MASK (0x10000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_AXIMASTER_DIB_ER_UN_SHIFT (0x0000001CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_AXIMASTER_DIB_ER_UN_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_AXIMASTER_RFIFO_ER_UN_MASK (0x20000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_AXIMASTER_RFIFO_ER_UN_SHIFT (0x0000001DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_AXIMASTER_RFIFO_ER_UN_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_AXISLAVE_WFIFO_ER_UN_MASK (0x40000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_AXISLAVE_WFIFO_ER_UN_SHIFT (0x0000001EU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_AXISLAVE_WFIFO_ER_UN_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_REORDER_ER_UN_MASK (0x80000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_REORDER_ER_UN_SHIFT (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_REGISTER_REORDER_ER_UN_MAX (0x00000001U)

/* I_LOCAL_INTRPT_MASK_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_PRFPE_MASK (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_PRFPE_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_PRFPE_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_CRFPE_MASK (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_CRFPE_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_CRFPE_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_RRPE_MASK (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_RRPE_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_RRPE_MAX  (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_PRFO_MASK (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_PRFO_SHIFT (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_PRFO_MAX  (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_CRFO_MASK (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_CRFO_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_CRFO_MAX  (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_RT_MASK   (0x00000020U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_RT_SHIFT  (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_RT_MAX    (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_RTR_MASK  (0x00000040U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_RTR_SHIFT (0x00000006U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_RTR_MAX   (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_PE_MASK   (0x00000080U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_PE_SHIFT  (0x00000007U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_PE_MAX    (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_MTR_MASK  (0x00000100U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_MTR_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_MTR_MAX   (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_UCR_MASK  (0x00000200U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_UCR_SHIFT (0x00000009U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_UCR_MAX   (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_FCE_MASK  (0x00000400U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_FCE_SHIFT (0x0000000AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_FCE_MAX   (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_CT_MASK   (0x00000800U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_CT_SHIFT  (0x0000000BU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_CT_MAX    (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_R12_MASK  (0x00001000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_R12_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_R12_MAX   (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_R13_MASK  (0x0001E000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_R13_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_R13_MAX   (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_EEPE_MASK (0x00020000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_EEPE_SHIFT (0x00000011U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_EEPE_MAX  (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_UTC_MASK  (0x00040000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_UTC_SHIFT (0x00000012U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_UTC_MAX   (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_MMVC_MASK (0x00080000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_MMVC_SHIFT (0x00000013U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_MMVC_MAX  (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_R45_MASK  (0x00100000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_R45_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_R45_MAX   (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_HAWCD_MASK (0x00200000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_HAWCD_SHIFT (0x00000015U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_HAWCD_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_R24_MASK  (0x01C00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_R24_SHIFT (0x00000016U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_R24_MAX   (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_MSIXMSK_MASK (0x02000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_MSIXMSK_SHIFT (0x00000019U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_MSIXMSK_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_R27_MASK  (0x0C000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_R27_SHIFT (0x0000001AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_R27_MAX   (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_AXIMASTER_DIB_ER_UN_MASK (0x10000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_AXIMASTER_DIB_ER_UN_SHIFT (0x0000001CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_AXIMASTER_DIB_ER_UN_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_AXIMASTER_RFIFO_ER_UN_MASK (0x20000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_AXIMASTER_RFIFO_ER_UN_SHIFT (0x0000001DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_AXIMASTER_RFIFO_ER_UN_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_AXISLAVE_WFIFO_ER_UN_MASK (0x40000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_AXISLAVE_WFIFO_ER_UN_SHIFT (0x0000001EU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_AXISLAVE_WFIFO_ER_UN_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_REORDER_ER_UN_MASK (0x80000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_REORDER_ER_UN_SHIFT (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_REG_REORDER_ER_UN_MAX (0x00000001U)

/* I_LCRC_ERR_COUNT_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LCRC_ERR_COUNT_REG_LEC_MASK     (0x0000FFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LCRC_ERR_COUNT_REG_LEC_SHIFT    (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LCRC_ERR_COUNT_REG_LEC_MAX      (0x0000FFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LCRC_ERR_COUNT_REG_R11_MASK     (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LCRC_ERR_COUNT_REG_R11_SHIFT    (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LCRC_ERR_COUNT_REG_R11_MAX      (0x0000FFFFU)

/* I_ECC_CORR_ERR_COUNT_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ECC_CORR_ERR_COUNT_REG_PFRCER_MASK (0x000000FFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ECC_CORR_ERR_COUNT_REG_PFRCER_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ECC_CORR_ERR_COUNT_REG_PFRCER_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ECC_CORR_ERR_COUNT_REG_SFRCER_MASK (0x0000FF00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ECC_CORR_ERR_COUNT_REG_SFRCER_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ECC_CORR_ERR_COUNT_REG_SFRCER_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ECC_CORR_ERR_COUNT_REG_RRCER_MASK (0x00FF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ECC_CORR_ERR_COUNT_REG_RRCER_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ECC_CORR_ERR_COUNT_REG_RRCER_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ECC_CORR_ERR_COUNT_REG_R12_MASK (0xFF000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ECC_CORR_ERR_COUNT_REG_R12_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ECC_CORR_ERR_COUNT_REG_R12_MAX  (0x000000FFU)

/* I_LTR_SNOOP_LAT_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_SNOOP_LAT_REG_NSLV_MASK     (0x000003FFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_SNOOP_LAT_REG_NSLV_SHIFT    (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_SNOOP_LAT_REG_NSLV_MAX      (0x000003FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_SNOOP_LAT_REG_NSLS_MASK     (0x00001C00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_SNOOP_LAT_REG_NSLS_SHIFT    (0x0000000AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_SNOOP_LAT_REG_NSLS_MAX      (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_SNOOP_LAT_REG_R12_MASK      (0x00006000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_SNOOP_LAT_REG_R12_SHIFT     (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_SNOOP_LAT_REG_R12_MAX       (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_SNOOP_LAT_REG_NSLR_MASK     (0x00008000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_SNOOP_LAT_REG_NSLR_SHIFT    (0x0000000FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_SNOOP_LAT_REG_NSLR_MAX      (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_SNOOP_LAT_REG_SLV_MASK      (0x03FF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_SNOOP_LAT_REG_SLV_SHIFT     (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_SNOOP_LAT_REG_SLV_MAX       (0x000003FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_SNOOP_LAT_REG_SLS_MASK      (0x1C000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_SNOOP_LAT_REG_SLS_SHIFT     (0x0000001AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_SNOOP_LAT_REG_SLS_MAX       (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_SNOOP_LAT_REG_R13_MASK      (0x60000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_SNOOP_LAT_REG_R13_SHIFT     (0x0000001DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_SNOOP_LAT_REG_R13_MAX       (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_SNOOP_LAT_REG_SL_MASK       (0x80000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_SNOOP_LAT_REG_SL_SHIFT      (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_SNOOP_LAT_REG_SL_MAX        (0x00000001U)

/* I_LTR_MSG_GEN_CTL_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_MSG_GEN_CTL_REG_MLI_MASK    (0x000003FFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_MSG_GEN_CTL_REG_MLI_SHIFT   (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_MSG_GEN_CTL_REG_MLI_MAX     (0x000003FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_MSG_GEN_CTL_REG_SLM_MASK    (0x00000400U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_MSG_GEN_CTL_REG_SLM_SHIFT   (0x0000000AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_MSG_GEN_CTL_REG_SLM_MAX     (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_MSG_GEN_CTL_REG_TMLMET_MASK (0x00000800U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_MSG_GEN_CTL_REG_TMLMET_SHIFT (0x0000000BU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_MSG_GEN_CTL_REG_TMLMET_MAX  (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_MSG_GEN_CTL_REG_TMFPSC_MASK (0x00001000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_MSG_GEN_CTL_REG_TMFPSC_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LTR_MSG_GEN_CTL_REG_TMFPSC_MAX  (0x00000001U)

/* I_PME_SERVICE_TIMEOUT_DELAY_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PME_SERVICE_TIMEOUT_DELAY_REG_PSTD_MASK (0x000FFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PME_SERVICE_TIMEOUT_DELAY_REG_PSTD_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PME_SERVICE_TIMEOUT_DELAY_REG_PSTD_MAX (0x000FFFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PME_SERVICE_TIMEOUT_DELAY_REG_DPMOPS_MASK (0x00100000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PME_SERVICE_TIMEOUT_DELAY_REG_DPMOPS_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PME_SERVICE_TIMEOUT_DELAY_REG_DPMOPS_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PME_SERVICE_TIMEOUT_DELAY_REG_R21_MASK (0xFFE00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PME_SERVICE_TIMEOUT_DELAY_REG_R21_SHIFT (0x00000015U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PME_SERVICE_TIMEOUT_DELAY_REG_R21_MAX (0x000007FFU)

/* I_ROOT_PORT_REQUESTOR_ID_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ROOT_PORT_REQUESTOR_ID_REG_RPRI_MASK (0x0000FFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ROOT_PORT_REQUESTOR_ID_REG_RPRI_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ROOT_PORT_REQUESTOR_ID_REG_RPRI_MAX (0x0000FFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ROOT_PORT_REQUESTOR_ID_REG_R0_MASK (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ROOT_PORT_REQUESTOR_ID_REG_R0_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ROOT_PORT_REQUESTOR_ID_REG_R0_MAX (0x0000FFFFU)

/* I_EP_BUS_DEVICE_NUMBER_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_EP_BUS_DEVICE_NUMBER_REG_EPDN_MASK (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_EP_BUS_DEVICE_NUMBER_REG_EPDN_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_EP_BUS_DEVICE_NUMBER_REG_EPDN_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_EP_BUS_DEVICE_NUMBER_REG_R5_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_EP_BUS_DEVICE_NUMBER_REG_R5_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_EP_BUS_DEVICE_NUMBER_REG_R5_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_EP_BUS_DEVICE_NUMBER_REG_EPBN_MASK (0x0000FF00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_EP_BUS_DEVICE_NUMBER_REG_EPBN_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_EP_BUS_DEVICE_NUMBER_REG_EPBN_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_EP_BUS_DEVICE_NUMBER_REG_R16_MASK (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_EP_BUS_DEVICE_NUMBER_REG_R16_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_EP_BUS_DEVICE_NUMBER_REG_R16_MAX (0x0000FFFFU)

/* I_DEBUG_MUX_CONTROL_2_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_DLFFS_MASK (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_DLFFS_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_DLFFS_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_EXTSNP_MASK (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_EXTSNP_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_EXTSNP_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_DISSDSCHK_MASK (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_DISSDSCHK_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_DISSDSCHK_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_ENLNCHK_MASK (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_ENLNCHK_SHIFT (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_ENLNCHK_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_ARICAPMOD_MASK (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_ARICAPMOD_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_ARICAPMOD_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_BLKALNCHK_MASK (0x00000020U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_BLKALNCHK_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_BLKALNCHK_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_BLKALNWIN_MASK (0x000000C0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_BLKALNWIN_SHIFT (0x00000006U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_BLKALNWIN_MAX (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_ENG4REV05_MASK (0x00000100U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_ENG4REV05_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_ENG4REV05_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_MSIPIMS_MASK (0x00000200U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_MSIPIMS_SHIFT (0x00000009U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_MSIPIMS_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_PSNADV_MASK (0x00000400U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_PSNADV_SHIFT (0x0000000AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_PSNADV_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_CMPTOADV_MASK (0x00000800U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_CMPTOADV_SHIFT (0x0000000BU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_CMPTOADV_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_AXINPSPEN_RSVD_MASK (0x00001000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_AXINPSPEN_RSVD_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_AXINPSPEN_RSVD_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_MAXNPREQ_MASK (0x007FE000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_MAXNPREQ_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_MAXNPREQ_MAX (0x000003FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_VARCCLKEN_MASK (0x00800000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_VARCCLKEN_SHIFT (0x00000017U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_VARCCLKEN_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_MSIMSKEN_MASK (0x01000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_MSIMSKEN_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_MSIMSKEN_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_MSIXMSKEN_MASK (0x02000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_MSIXMSKEN_SHIFT (0x00000019U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_MSIXMSKEN_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_R26_MASK (0x04000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_R26_SHIFT (0x0000001AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_R26_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_DTAE2EP_MASK (0x08000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_DTAE2EP_SHIFT (0x0000001BU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_DTAE2EP_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_DFLRTRB_MASK (0x10000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_DFLRTRB_SHIFT (0x0000001CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_DFLRTRB_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_R31_MASK (0xE0000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_R31_SHIFT (0x0000001DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DEBUG_MUX_CONTROL_2_REG_R31_MAX (0x00000007U)

/* I_PHY_STATUS_1_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_TLPPHYER_MASK  (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_TLPPHYER_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_TLPPHYER_MAX   (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_OSASKP_MASK    (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_OSASKP_SHIFT   (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_OSASKP_MAX     (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_ILOSEDS_MASK   (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_ILOSEDS_SHIFT  (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_ILOSEDS_MAX    (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_DATEDS_MASK    (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_DATEDS_SHIFT   (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_DATEDS_MAX     (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_OSWOEDS_MASK   (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_OSWOEDS_SHIFT  (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_OSWOEDS_MAX    (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_G3FRERR_MASK   (0x00000020U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_G3FRERR_SHIFT  (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_G3FRERR_MAX    (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_OSAFSDS_MASK   (0x00000040U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_OSAFSDS_SHIFT  (0x00000006U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_OSAFSDS_MAX    (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_INVSYNHR_MASK  (0x00000080U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_INVSYNHR_SHIFT (0x00000007U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_INVSYNHR_MAX   (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_LOSBLKALN_MASK (0x00000100U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_LOSBLKALN_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_LOSBLKALN_MAX  (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_R31_MASK       (0xFFFFFE00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_R31_SHIFT      (0x00000009U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_STATUS_1_REG_R31_MAX        (0x007FFFFFU)

/* I_PF_0_BAR_CONFIG_0_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR0A_MASK (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR0A_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR0A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR0C_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR0C_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR0C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR1A_MASK (0x00001F00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR1A_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR1A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR1C_MASK (0x0000E000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR1C_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR1C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR2A_MASK (0x001F0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR2A_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR2A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR2C_MASK (0x00E00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR2C_SHIFT (0x00000015U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR2C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR3A_MASK (0x1F000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR3A_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR3A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR3C_MASK (0xE0000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR3C_SHIFT (0x0000001DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_0_REG_BAR3C_MAX (0x00000007U)

/* I_PF_0_BAR_CONFIG_1_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_1_REG_BAR4A_MASK (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_1_REG_BAR4A_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_1_REG_BAR4A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_1_REG_BAR4C_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_1_REG_BAR4C_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_1_REG_BAR4C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_1_REG_BAR5A_MASK (0x00001F00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_1_REG_BAR5A_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_1_REG_BAR5A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_1_REG_BAR5C_MASK (0x0000E000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_1_REG_BAR5C_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_1_REG_BAR5C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_1_REG_R16_MASK  (0x00FF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_1_REG_R16_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_1_REG_R16_MAX   (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_1_REG_R24_MASK  (0x7F000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_1_REG_R24_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_1_REG_R24_MAX   (0x0000007FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_1_REG_ERBC_MASK (0x80000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_1_REG_ERBC_SHIFT (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_BAR_CONFIG_1_REG_ERBC_MAX  (0x00000001U)

/* I_PF_1_BAR_CONFIG_0_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_0_REG_BAR0A_MASK (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_0_REG_BAR0A_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_0_REG_BAR0A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_0_REG_BAR0C_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_0_REG_BAR0C_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_0_REG_BAR0C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_0_REG_BAR1A_MASK (0x00001F00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_0_REG_BAR1A_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_0_REG_BAR1A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_0_REG_BAR1C_MASK (0x0000E000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_0_REG_BAR1C_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_0_REG_BAR1C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_0_REG_BAR2A_MASK (0x001F0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_0_REG_BAR2A_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_0_REG_BAR2A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_0_REG_BAR2C_MASK (0x00E00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_0_REG_BAR2C_SHIFT (0x00000015U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_0_REG_BAR2C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_0_REG_BAR3A_MASK (0x1F000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_0_REG_BAR3A_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_0_REG_BAR3A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_0_REG_BAR3C_MASK (0xE0000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_0_REG_BAR3C_SHIFT (0x0000001DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_0_REG_BAR3C_MAX (0x00000007U)

/* I_PF_1_BAR_CONFIG_1_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_1_REG_BAR4A_MASK (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_1_REG_BAR4A_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_1_REG_BAR4A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_1_REG_BAR4C_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_1_REG_BAR4C_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_1_REG_BAR4C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_1_REG_BAR5A_MASK (0x00001F00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_1_REG_BAR5A_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_1_REG_BAR5A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_1_REG_BAR5C_MASK (0x0000E000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_1_REG_BAR5C_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_1_REG_BAR5C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_1_REG_R16_MASK  (0x00FF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_1_REG_R16_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_1_REG_R16_MAX   (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_1_REG_R24_MASK  (0x7F000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_1_REG_R24_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_1_REG_R24_MAX   (0x0000007FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_1_REG_ERBC_MASK (0x80000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_1_REG_ERBC_SHIFT (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_BAR_CONFIG_1_REG_ERBC_MAX  (0x00000001U)

/* I_PF_2_BAR_CONFIG_0_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_0_REG_BAR0A_MASK (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_0_REG_BAR0A_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_0_REG_BAR0A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_0_REG_BAR0C_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_0_REG_BAR0C_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_0_REG_BAR0C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_0_REG_BAR1A_MASK (0x00001F00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_0_REG_BAR1A_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_0_REG_BAR1A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_0_REG_BAR1C_MASK (0x0000E000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_0_REG_BAR1C_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_0_REG_BAR1C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_0_REG_BAR2A_MASK (0x001F0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_0_REG_BAR2A_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_0_REG_BAR2A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_0_REG_BAR2C_MASK (0x00E00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_0_REG_BAR2C_SHIFT (0x00000015U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_0_REG_BAR2C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_0_REG_BAR3A_MASK (0x1F000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_0_REG_BAR3A_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_0_REG_BAR3A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_0_REG_BAR3C_MASK (0xE0000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_0_REG_BAR3C_SHIFT (0x0000001DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_0_REG_BAR3C_MAX (0x00000007U)

/* I_PF_2_BAR_CONFIG_1_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_1_REG_BAR4A_MASK (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_1_REG_BAR4A_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_1_REG_BAR4A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_1_REG_BAR4C_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_1_REG_BAR4C_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_1_REG_BAR4C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_1_REG_BAR5A_MASK (0x00001F00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_1_REG_BAR5A_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_1_REG_BAR5A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_1_REG_BAR5C_MASK (0x0000E000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_1_REG_BAR5C_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_1_REG_BAR5C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_1_REG_R16_MASK  (0x00FF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_1_REG_R16_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_1_REG_R16_MAX   (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_1_REG_R24_MASK  (0x7F000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_1_REG_R24_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_1_REG_R24_MAX   (0x0000007FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_1_REG_ERBC_MASK (0x80000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_1_REG_ERBC_SHIFT (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_BAR_CONFIG_1_REG_ERBC_MAX  (0x00000001U)

/* I_PF_3_BAR_CONFIG_0_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_0_REG_BAR0A_MASK (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_0_REG_BAR0A_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_0_REG_BAR0A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_0_REG_BAR0C_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_0_REG_BAR0C_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_0_REG_BAR0C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_0_REG_BAR1A_MASK (0x00001F00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_0_REG_BAR1A_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_0_REG_BAR1A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_0_REG_BAR1C_MASK (0x0000E000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_0_REG_BAR1C_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_0_REG_BAR1C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_0_REG_BAR2A_MASK (0x001F0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_0_REG_BAR2A_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_0_REG_BAR2A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_0_REG_BAR2C_MASK (0x00E00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_0_REG_BAR2C_SHIFT (0x00000015U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_0_REG_BAR2C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_0_REG_BAR3A_MASK (0x1F000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_0_REG_BAR3A_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_0_REG_BAR3A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_0_REG_BAR3C_MASK (0xE0000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_0_REG_BAR3C_SHIFT (0x0000001DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_0_REG_BAR3C_MAX (0x00000007U)

/* I_PF_3_BAR_CONFIG_1_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_1_REG_BAR4A_MASK (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_1_REG_BAR4A_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_1_REG_BAR4A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_1_REG_BAR4C_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_1_REG_BAR4C_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_1_REG_BAR4C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_1_REG_BAR5A_MASK (0x00001F00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_1_REG_BAR5A_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_1_REG_BAR5A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_1_REG_BAR5C_MASK (0x0000E000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_1_REG_BAR5C_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_1_REG_BAR5C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_1_REG_R16_MASK  (0x00FF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_1_REG_R16_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_1_REG_R16_MAX   (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_1_REG_R24_MASK  (0x7F000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_1_REG_R24_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_1_REG_R24_MAX   (0x0000007FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_1_REG_ERBC_MASK (0x80000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_1_REG_ERBC_SHIFT (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_BAR_CONFIG_1_REG_ERBC_MAX  (0x00000001U)

/* I_PF_4_BAR_CONFIG_0_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_0_REG_BAR0A_MASK (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_0_REG_BAR0A_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_0_REG_BAR0A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_0_REG_BAR0C_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_0_REG_BAR0C_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_0_REG_BAR0C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_0_REG_BAR1A_MASK (0x00001F00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_0_REG_BAR1A_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_0_REG_BAR1A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_0_REG_BAR1C_MASK (0x0000E000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_0_REG_BAR1C_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_0_REG_BAR1C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_0_REG_BAR2A_MASK (0x001F0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_0_REG_BAR2A_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_0_REG_BAR2A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_0_REG_BAR2C_MASK (0x00E00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_0_REG_BAR2C_SHIFT (0x00000015U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_0_REG_BAR2C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_0_REG_BAR3A_MASK (0x1F000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_0_REG_BAR3A_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_0_REG_BAR3A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_0_REG_BAR3C_MASK (0xE0000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_0_REG_BAR3C_SHIFT (0x0000001DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_0_REG_BAR3C_MAX (0x00000007U)

/* I_PF_4_BAR_CONFIG_1_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_1_REG_BAR4A_MASK (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_1_REG_BAR4A_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_1_REG_BAR4A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_1_REG_BAR4C_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_1_REG_BAR4C_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_1_REG_BAR4C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_1_REG_BAR5A_MASK (0x00001F00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_1_REG_BAR5A_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_1_REG_BAR5A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_1_REG_BAR5C_MASK (0x0000E000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_1_REG_BAR5C_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_1_REG_BAR5C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_1_REG_R16_MASK  (0x00FF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_1_REG_R16_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_1_REG_R16_MAX   (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_1_REG_R24_MASK  (0x7F000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_1_REG_R24_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_1_REG_R24_MAX   (0x0000007FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_1_REG_ERBC_MASK (0x80000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_1_REG_ERBC_SHIFT (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_BAR_CONFIG_1_REG_ERBC_MAX  (0x00000001U)

/* I_PF_5_BAR_CONFIG_0_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_0_REG_BAR0A_MASK (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_0_REG_BAR0A_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_0_REG_BAR0A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_0_REG_BAR0C_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_0_REG_BAR0C_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_0_REG_BAR0C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_0_REG_BAR1A_MASK (0x00001F00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_0_REG_BAR1A_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_0_REG_BAR1A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_0_REG_BAR1C_MASK (0x0000E000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_0_REG_BAR1C_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_0_REG_BAR1C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_0_REG_BAR2A_MASK (0x001F0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_0_REG_BAR2A_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_0_REG_BAR2A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_0_REG_BAR2C_MASK (0x00E00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_0_REG_BAR2C_SHIFT (0x00000015U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_0_REG_BAR2C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_0_REG_BAR3A_MASK (0x1F000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_0_REG_BAR3A_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_0_REG_BAR3A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_0_REG_BAR3C_MASK (0xE0000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_0_REG_BAR3C_SHIFT (0x0000001DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_0_REG_BAR3C_MAX (0x00000007U)

/* I_PF_5_BAR_CONFIG_1_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_1_REG_BAR4A_MASK (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_1_REG_BAR4A_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_1_REG_BAR4A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_1_REG_BAR4C_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_1_REG_BAR4C_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_1_REG_BAR4C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_1_REG_BAR5A_MASK (0x00001F00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_1_REG_BAR5A_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_1_REG_BAR5A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_1_REG_BAR5C_MASK (0x0000E000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_1_REG_BAR5C_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_1_REG_BAR5C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_1_REG_R16_MASK  (0x00FF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_1_REG_R16_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_1_REG_R16_MAX   (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_1_REG_R24_MASK  (0x7F000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_1_REG_R24_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_1_REG_R24_MAX   (0x0000007FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_1_REG_ERBC_MASK (0x80000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_1_REG_ERBC_SHIFT (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_BAR_CONFIG_1_REG_ERBC_MAX  (0x00000001U)

/* I_PF_0_VF_BAR_CONFIG_0_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_0_REG_VFBAR0A_MASK (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_0_REG_VFBAR0A_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_0_REG_VFBAR0A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_0_REG_VFBAR0C_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_0_REG_VFBAR0C_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_0_REG_VFBAR0C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_0_REG_VFBAR1A_MASK (0x00001F00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_0_REG_VFBAR1A_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_0_REG_VFBAR1A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_0_REG_VFBAR1C_MASK (0x0000E000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_0_REG_VFBAR1C_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_0_REG_VFBAR1C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_0_REG_VFBAR2A_MASK (0x001F0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_0_REG_VFBAR2A_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_0_REG_VFBAR2A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_0_REG_VFBAR2C_MASK (0x00E00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_0_REG_VFBAR2C_SHIFT (0x00000015U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_0_REG_VFBAR2C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_0_REG_VFBAR3A_MASK (0x1F000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_0_REG_VFBAR3A_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_0_REG_VFBAR3A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_0_REG_VFBAR3C_MASK (0xE0000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_0_REG_VFBAR3C_SHIFT (0x0000001DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_0_REG_VFBAR3C_MAX (0x00000007U)

/* I_PF_0_VF_BAR_CONFIG_1_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_1_REG_VFBAR4A_MASK (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_1_REG_VFBAR4A_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_1_REG_VFBAR4A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_1_REG_VFBAR4C_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_1_REG_VFBAR4C_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_1_REG_VFBAR4C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_1_REG_VFBAR5A_MASK (0x00001F00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_1_REG_VFBAR5A_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_1_REG_VFBAR5A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_1_REG_VFBAR5C_MASK (0x0000E000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_1_REG_VFBAR5C_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_1_REG_VFBAR5C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_1_REG_R16_MASK (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_1_REG_R16_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_0_VF_BAR_CONFIG_1_REG_R16_MAX (0x0000FFFFU)

/* I_PF_1_VF_BAR_CONFIG_0_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_0_REG_VFBAR0A_MASK (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_0_REG_VFBAR0A_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_0_REG_VFBAR0A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_0_REG_VFBAR0C_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_0_REG_VFBAR0C_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_0_REG_VFBAR0C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_0_REG_VFBAR1A_MASK (0x00001F00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_0_REG_VFBAR1A_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_0_REG_VFBAR1A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_0_REG_VFBAR1C_MASK (0x0000E000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_0_REG_VFBAR1C_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_0_REG_VFBAR1C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_0_REG_VFBAR2A_MASK (0x001F0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_0_REG_VFBAR2A_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_0_REG_VFBAR2A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_0_REG_VFBAR2C_MASK (0x00E00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_0_REG_VFBAR2C_SHIFT (0x00000015U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_0_REG_VFBAR2C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_0_REG_VFBAR3A_MASK (0x1F000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_0_REG_VFBAR3A_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_0_REG_VFBAR3A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_0_REG_VFBAR3C_MASK (0xE0000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_0_REG_VFBAR3C_SHIFT (0x0000001DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_0_REG_VFBAR3C_MAX (0x00000007U)

/* I_PF_1_VF_BAR_CONFIG_1_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_1_REG_VFBAR4A_MASK (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_1_REG_VFBAR4A_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_1_REG_VFBAR4A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_1_REG_VFBAR4C_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_1_REG_VFBAR4C_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_1_REG_VFBAR4C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_1_REG_VFBAR5A_MASK (0x00001F00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_1_REG_VFBAR5A_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_1_REG_VFBAR5A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_1_REG_VFBAR5C_MASK (0x0000E000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_1_REG_VFBAR5C_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_1_REG_VFBAR5C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_1_REG_R16_MASK (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_1_REG_R16_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_1_VF_BAR_CONFIG_1_REG_R16_MAX (0x0000FFFFU)

/* I_PF_2_VF_BAR_CONFIG_0_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_0_REG_VFBAR0A_MASK (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_0_REG_VFBAR0A_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_0_REG_VFBAR0A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_0_REG_VFBAR0C_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_0_REG_VFBAR0C_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_0_REG_VFBAR0C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_0_REG_VFBAR1A_MASK (0x00001F00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_0_REG_VFBAR1A_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_0_REG_VFBAR1A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_0_REG_VFBAR1C_MASK (0x0000E000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_0_REG_VFBAR1C_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_0_REG_VFBAR1C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_0_REG_VFBAR2A_MASK (0x001F0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_0_REG_VFBAR2A_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_0_REG_VFBAR2A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_0_REG_VFBAR2C_MASK (0x00E00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_0_REG_VFBAR2C_SHIFT (0x00000015U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_0_REG_VFBAR2C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_0_REG_VFBAR3A_MASK (0x1F000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_0_REG_VFBAR3A_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_0_REG_VFBAR3A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_0_REG_VFBAR3C_MASK (0xE0000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_0_REG_VFBAR3C_SHIFT (0x0000001DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_0_REG_VFBAR3C_MAX (0x00000007U)

/* I_PF_2_VF_BAR_CONFIG_1_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_1_REG_VFBAR4A_MASK (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_1_REG_VFBAR4A_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_1_REG_VFBAR4A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_1_REG_VFBAR4C_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_1_REG_VFBAR4C_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_1_REG_VFBAR4C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_1_REG_VFBAR5A_MASK (0x00001F00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_1_REG_VFBAR5A_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_1_REG_VFBAR5A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_1_REG_VFBAR5C_MASK (0x0000E000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_1_REG_VFBAR5C_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_1_REG_VFBAR5C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_1_REG_R16_MASK (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_1_REG_R16_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_2_VF_BAR_CONFIG_1_REG_R16_MAX (0x0000FFFFU)

/* I_PF_3_VF_BAR_CONFIG_0_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_0_REG_VFBAR0A_MASK (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_0_REG_VFBAR0A_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_0_REG_VFBAR0A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_0_REG_VFBAR0C_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_0_REG_VFBAR0C_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_0_REG_VFBAR0C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_0_REG_VFBAR1A_MASK (0x00001F00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_0_REG_VFBAR1A_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_0_REG_VFBAR1A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_0_REG_VFBAR1C_MASK (0x0000E000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_0_REG_VFBAR1C_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_0_REG_VFBAR1C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_0_REG_VFBAR2A_MASK (0x001F0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_0_REG_VFBAR2A_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_0_REG_VFBAR2A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_0_REG_VFBAR2C_MASK (0x00E00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_0_REG_VFBAR2C_SHIFT (0x00000015U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_0_REG_VFBAR2C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_0_REG_VFBAR3A_MASK (0x1F000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_0_REG_VFBAR3A_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_0_REG_VFBAR3A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_0_REG_VFBAR3C_MASK (0xE0000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_0_REG_VFBAR3C_SHIFT (0x0000001DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_0_REG_VFBAR3C_MAX (0x00000007U)

/* I_PF_3_VF_BAR_CONFIG_1_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_1_REG_VFBAR4A_MASK (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_1_REG_VFBAR4A_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_1_REG_VFBAR4A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_1_REG_VFBAR4C_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_1_REG_VFBAR4C_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_1_REG_VFBAR4C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_1_REG_VFBAR5A_MASK (0x00001F00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_1_REG_VFBAR5A_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_1_REG_VFBAR5A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_1_REG_VFBAR5C_MASK (0x0000E000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_1_REG_VFBAR5C_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_1_REG_VFBAR5C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_1_REG_R16_MASK (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_1_REG_R16_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_3_VF_BAR_CONFIG_1_REG_R16_MAX (0x0000FFFFU)

/* I_PF_4_VF_BAR_CONFIG_0_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_0_REG_VFBAR0A_MASK (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_0_REG_VFBAR0A_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_0_REG_VFBAR0A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_0_REG_VFBAR0C_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_0_REG_VFBAR0C_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_0_REG_VFBAR0C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_0_REG_VFBAR1A_MASK (0x00001F00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_0_REG_VFBAR1A_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_0_REG_VFBAR1A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_0_REG_VFBAR1C_MASK (0x0000E000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_0_REG_VFBAR1C_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_0_REG_VFBAR1C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_0_REG_VFBAR2A_MASK (0x001F0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_0_REG_VFBAR2A_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_0_REG_VFBAR2A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_0_REG_VFBAR2C_MASK (0x00E00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_0_REG_VFBAR2C_SHIFT (0x00000015U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_0_REG_VFBAR2C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_0_REG_VFBAR3A_MASK (0x1F000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_0_REG_VFBAR3A_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_0_REG_VFBAR3A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_0_REG_VFBAR3C_MASK (0xE0000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_0_REG_VFBAR3C_SHIFT (0x0000001DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_0_REG_VFBAR3C_MAX (0x00000007U)

/* I_PF_4_VF_BAR_CONFIG_1_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_1_REG_VFBAR4A_MASK (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_1_REG_VFBAR4A_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_1_REG_VFBAR4A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_1_REG_VFBAR4C_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_1_REG_VFBAR4C_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_1_REG_VFBAR4C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_1_REG_VFBAR5A_MASK (0x00001F00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_1_REG_VFBAR5A_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_1_REG_VFBAR5A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_1_REG_VFBAR5C_MASK (0x0000E000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_1_REG_VFBAR5C_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_1_REG_VFBAR5C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_1_REG_R16_MASK (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_1_REG_R16_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_4_VF_BAR_CONFIG_1_REG_R16_MAX (0x0000FFFFU)

/* I_PF_5_VF_BAR_CONFIG_0_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_0_REG_VFBAR0A_MASK (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_0_REG_VFBAR0A_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_0_REG_VFBAR0A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_0_REG_VFBAR0C_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_0_REG_VFBAR0C_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_0_REG_VFBAR0C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_0_REG_VFBAR1A_MASK (0x00001F00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_0_REG_VFBAR1A_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_0_REG_VFBAR1A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_0_REG_VFBAR1C_MASK (0x0000E000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_0_REG_VFBAR1C_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_0_REG_VFBAR1C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_0_REG_VFBAR2A_MASK (0x001F0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_0_REG_VFBAR2A_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_0_REG_VFBAR2A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_0_REG_VFBAR2C_MASK (0x00E00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_0_REG_VFBAR2C_SHIFT (0x00000015U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_0_REG_VFBAR2C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_0_REG_VFBAR3A_MASK (0x1F000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_0_REG_VFBAR3A_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_0_REG_VFBAR3A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_0_REG_VFBAR3C_MASK (0xE0000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_0_REG_VFBAR3C_SHIFT (0x0000001DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_0_REG_VFBAR3C_MAX (0x00000007U)

/* I_PF_5_VF_BAR_CONFIG_1_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_1_REG_VFBAR4A_MASK (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_1_REG_VFBAR4A_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_1_REG_VFBAR4A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_1_REG_VFBAR4C_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_1_REG_VFBAR4C_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_1_REG_VFBAR4C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_1_REG_VFBAR5A_MASK (0x00001F00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_1_REG_VFBAR5A_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_1_REG_VFBAR5A_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_1_REG_VFBAR5C_MASK (0x0000E000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_1_REG_VFBAR5C_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_1_REG_VFBAR5C_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_1_REG_R16_MASK (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_1_REG_R16_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_5_VF_BAR_CONFIG_1_REG_R16_MAX (0x0000FFFFU)

/* I_PF_CONFIG_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_CONFIG_REG_F0E_MASK          (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_CONFIG_REG_F0E_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_CONFIG_REG_F0E_MAX           (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_CONFIG_REG_F1E_MASK          (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_CONFIG_REG_F1E_SHIFT         (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_CONFIG_REG_F1E_MAX           (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_CONFIG_REG_F2E_MASK          (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_CONFIG_REG_F2E_SHIFT         (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_CONFIG_REG_F2E_MAX           (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_CONFIG_REG_F3E_MASK          (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_CONFIG_REG_F3E_SHIFT         (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_CONFIG_REG_F3E_MAX           (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_CONFIG_REG_F4E_MASK          (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_CONFIG_REG_F4E_SHIFT         (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_CONFIG_REG_F4E_MAX           (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_CONFIG_REG_F5E_MASK          (0x00000020U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_CONFIG_REG_F5E_SHIFT         (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_CONFIG_REG_F5E_MAX           (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_CONFIG_REG_R_MASK            (0xFFFFFFC0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_CONFIG_REG_R_SHIFT           (0x00000006U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PF_CONFIG_REG_R_MAX             (0x03FFFFFFU)

/* I_RC_BAR_CONFIG_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBAR0A_MASK  (0x0000003FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBAR0A_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBAR0A_MAX   (0x0000003FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBAR0C_MASK  (0x000001C0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBAR0C_SHIFT (0x00000006U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBAR0C_MAX   (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBAR1A_MASK  (0x00003E00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBAR1A_SHIFT (0x00000009U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBAR1A_MAX   (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBAR1C_MASK  (0x0001C000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBAR1C_SHIFT (0x0000000EU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBAR1C_MAX   (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBARPME_MASK (0x00020000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBARPME_SHIFT (0x00000011U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBARPME_MAX  (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBARPMS_MASK (0x00040000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBARPMS_SHIFT (0x00000012U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBARPMS_MAX  (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBARPIE_MASK (0x00080000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBARPIE_SHIFT (0x00000013U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBARPIE_MAX  (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBARPIS_MASK (0x00100000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBARPIS_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBARPIS_MAX  (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_R10_MASK      (0x7FE00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_R10_SHIFT     (0x00000015U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_R10_MAX       (0x000003FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBCE_MASK    (0x80000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBCE_SHIFT   (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_RC_BAR_CONFIG_REG_RCBCE_MAX     (0x00000001U)

/* I_GEN3_DEFAULT_PRESET_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_DEFAULT_PRESET_REG_GDTXP_MASK (0x0000000FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_DEFAULT_PRESET_REG_GDTXP_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_DEFAULT_PRESET_REG_GDTXP_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_DEFAULT_PRESET_REG_GDRXPH_MASK (0x00000070U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_DEFAULT_PRESET_REG_GDRXPH_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_DEFAULT_PRESET_REG_GDRXPH_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_DEFAULT_PRESET_REG_R7_MASK (0x00000080U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_DEFAULT_PRESET_REG_R7_SHIFT (0x00000007U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_DEFAULT_PRESET_REG_R7_MAX  (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_DEFAULT_PRESET_REG_S8GPR_MASK (0x0007FF00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_DEFAULT_PRESET_REG_S8GPR_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_DEFAULT_PRESET_REG_S8GPR_MAX (0x000007FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_DEFAULT_PRESET_REG_R31_MASK (0xFFF80000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_DEFAULT_PRESET_REG_R31_SHIFT (0x00000013U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_DEFAULT_PRESET_REG_R31_MAX (0x00001FFFU)

/* I_GEN3_GEN4_LINK_EQ_TIMEOUT_2MS_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_TIMEOUT_2MS_REG_LEQT2MS_MASK (0x0FFFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_TIMEOUT_2MS_REG_LEQT2MS_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_TIMEOUT_2MS_REG_LEQT2MS_MAX (0x0FFFFFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_TIMEOUT_2MS_REG_R28_MASK (0x10000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_TIMEOUT_2MS_REG_R28_SHIFT (0x0000001CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_TIMEOUT_2MS_REG_R28_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_TIMEOUT_2MS_REG_RXEQABD_MASK (0x20000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_TIMEOUT_2MS_REG_RXEQABD_SHIFT (0x0000001DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_TIMEOUT_2MS_REG_RXEQABD_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_TIMEOUT_2MS_REG_RXEQABM_MASK (0xC0000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_TIMEOUT_2MS_REG_RXEQABM_SHIFT (0x0000001EU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_TIMEOUT_2MS_REG_RXEQABM_MAX (0x00000003U)

/* I_PIPE_FIFO_LATENCY_CTRL_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PIPE_FIFO_LATENCY_CTRL_REG_DPTFCE_MASK (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PIPE_FIFO_LATENCY_CTRL_REG_DPTFCE_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PIPE_FIFO_LATENCY_CTRL_REG_DPTFCE_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PIPE_FIFO_LATENCY_CTRL_REG_R31_MASK (0xFFFFFFFEU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PIPE_FIFO_LATENCY_CTRL_REG_R31_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PIPE_FIFO_LATENCY_CTRL_REG_R31_MAX (0x7FFFFFFFU)

/* I_GEN4_DEFAULT_PRESET_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_DEFAULT_PRESET_REG_GDTXP_MASK (0x0000000FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_DEFAULT_PRESET_REG_GDTXP_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_DEFAULT_PRESET_REG_GDTXP_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_DEFAULT_PRESET_REG_GDRXPH_MASK (0x00000070U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_DEFAULT_PRESET_REG_GDRXPH_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_DEFAULT_PRESET_REG_GDRXPH_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_DEFAULT_PRESET_REG_R7_MASK (0x00000080U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_DEFAULT_PRESET_REG_R7_SHIFT (0x00000007U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_DEFAULT_PRESET_REG_R7_MAX  (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_DEFAULT_PRESET_REG_S16GPR_MASK (0x0007FF00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_DEFAULT_PRESET_REG_S16GPR_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_DEFAULT_PRESET_REG_S16GPR_MAX (0x000007FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_DEFAULT_PRESET_REG_R31_MASK (0xFFF80000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_DEFAULT_PRESET_REG_R31_SHIFT (0x00000013U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_DEFAULT_PRESET_REG_R31_MAX (0x00001FFFU)

/* I_PHY_CONFIG_REG3 */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_CONFIG_REG3_TFC4_MASK       (0x000000FFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_CONFIG_REG3_TFC4_SHIFT      (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_CONFIG_REG3_TFC4_MAX        (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_CONFIG_REG3_R24_MASK        (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_CONFIG_REG3_R24_SHIFT       (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PHY_CONFIG_REG3_R24_MAX         (0x00FFFFFFU)

/* I_GEN3_GEN4_LINK_EQ_CTRL_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_MXECC_MASK (0x00000007U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_MXECC_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_MXECC_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_RES3_MASK (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_RES3_SHIFT (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_RES3_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_EP8GRE_MASK (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_EP8GRE_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_EP8GRE_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_EP16GRE_MASK (0x00000020U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_EP16GRE_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_EP16GRE_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_RES6_MASK (0x000000C0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_RES6_SHIFT (0x00000006U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_RES6_MAX (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_QG8GT_MASK (0x00000100U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_QG8GT_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_QG8GT_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_QG16GT_MASK (0x00000200U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_QG16GT_SHIFT (0x00000009U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_QG16GT_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_RES10_MASK (0x00000C00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_RES10_SHIFT (0x0000000AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_RES10_MAX (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_MX8GERL_MASK (0x0000F000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_MX8GERL_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_MX8GERL_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_MX16GERL_MASK (0x000F0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_MX16GERL_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_MX16GERL_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_RES20_MASK (0xFFF00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_RES20_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_GEN4_LINK_EQ_CTRL_REG_RES20_MAX (0x00000FFFU)

/* I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE0 */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE0_LEQTXPR_MASK (0x0000000FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE0_LEQTXPR_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE0_LEQTXPR_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE0_LEQTXPRV_MASK (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE0_LEQTXPRV_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE0_LEQTXPRV_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE0_RES75_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE0_RES75_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE0_RES75_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE0_LEQTXCO_MASK (0x03FFFF00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE0_LEQTXCO_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE0_LEQTXCO_MAX (0x0003FFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE0_RES3126_MASK (0xFC000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE0_RES3126_SHIFT (0x0000001AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE0_RES3126_MAX (0x0000003FU)

/* I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE1 */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE1_LEQTXPR_MASK (0x0000000FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE1_LEQTXPR_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE1_LEQTXPR_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE1_LEQTXPRV_MASK (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE1_LEQTXPRV_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE1_LEQTXPRV_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE1_RES75_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE1_RES75_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE1_RES75_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE1_LEQTXCO_MASK (0x03FFFF00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE1_LEQTXCO_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE1_LEQTXCO_MAX (0x0003FFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE1_RES3126_MASK (0xFC000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE1_RES3126_SHIFT (0x0000001AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN3_LINK_EQ_DEBUG_STATUS_REG_LANE1_RES3126_MAX (0x0000003FU)

/* I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE0 */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE0_LEQTXPR_MASK (0x0000000FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE0_LEQTXPR_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE0_LEQTXPR_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE0_LEQTXPRV_MASK (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE0_LEQTXPRV_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE0_LEQTXPRV_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE0_RES75_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE0_RES75_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE0_RES75_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE0_LEQTXCO_MASK (0x03FFFF00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE0_LEQTXCO_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE0_LEQTXCO_MAX (0x0003FFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE0_RES3126_MASK (0xFC000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE0_RES3126_SHIFT (0x0000001AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE0_RES3126_MAX (0x0000003FU)

/* I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE1 */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE1_LEQTXPR_MASK (0x0000000FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE1_LEQTXPR_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE1_LEQTXPR_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE1_LEQTXPRV_MASK (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE1_LEQTXPRV_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE1_LEQTXPRV_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE1_RES75_MASK (0x000000E0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE1_RES75_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE1_RES75_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE1_LEQTXCO_MASK (0x03FFFF00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE1_LEQTXCO_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE1_LEQTXCO_MAX (0x0003FFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE1_RES3126_MASK (0xFC000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE1_RES3126_SHIFT (0x0000001AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_GEN4_LINK_EQ_DEBUG_STATUS_REG_LANE1_RES3126_MAX (0x0000003FU)

/* I_ECC_CORR_ERR_COUNT_REG_AXI */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ECC_CORR_ERR_COUNT_REG_AXI_REORDER_CER_MASK (0x000000FFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ECC_CORR_ERR_COUNT_REG_AXI_REORDER_CER_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ECC_CORR_ERR_COUNT_REG_AXI_REORDER_CER_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ECC_CORR_ERR_COUNT_REG_AXI_AXI_SLAVE_WFIFO_CER_MASK (0x0000FF00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ECC_CORR_ERR_COUNT_REG_AXI_AXI_SLAVE_WFIFO_CER_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ECC_CORR_ERR_COUNT_REG_AXI_AXI_SLAVE_WFIFO_CER_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ECC_CORR_ERR_COUNT_REG_AXI_AXI_MASTER_RFIFO_CER_MASK (0x00FF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ECC_CORR_ERR_COUNT_REG_AXI_AXI_MASTER_RFIFO_CER_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ECC_CORR_ERR_COUNT_REG_AXI_AXI_MASTER_RFIFO_CER_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ECC_CORR_ERR_COUNT_REG_AXI_AXI_MASTER_DIB_CER_MASK (0xFF000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ECC_CORR_ERR_COUNT_REG_AXI_AXI_MASTER_DIB_CER_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ECC_CORR_ERR_COUNT_REG_AXI_AXI_MASTER_DIB_CER_MAX (0x000000FFU)

/* LOW_POWER_DEBUG_AND_CONTROL0 */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL0_L1XDELAY_MASK (0x00FFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL0_L1XDELAY_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL0_L1XDELAY_MAX (0x00FFFFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL0_L1DBRI_MASK (0x01000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL0_L1DBRI_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL0_L1DBRI_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL0_L1EM_MASK (0x06000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL0_L1EM_SHIFT (0x00000019U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL0_L1EM_MAX (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL0_L1DLEUP_MASK (0x08000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL0_L1DLEUP_SHIFT (0x0000001BU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL0_L1DLEUP_MAX (0x00000001U)

/* LOW_POWER_DEBUG_AND_CONTROL1 */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL1_L1ER_MASK (0x000000FFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL1_L1ER_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL1_L1ER_MAX (0x000000FFU)

/* LOW_POWER_DEBUG_AND_CONTROL2 */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL2_L1TWROI_MASK (0x00FFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL2_L1TWROI_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL2_L1TWROI_MAX (0x00FFFFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL2_L1EOC_MASK (0x02000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL2_L1EOC_SHIFT (0x00000019U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL2_L1EOC_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL2_L1ERC_MASK (0x04000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL2_L1ERC_SHIFT (0x0000001AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL2_L1ERC_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL2_L1PS_MASK (0x08000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL2_L1PS_SHIFT (0x0000001BU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL2_L1PS_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL2_L1TROW_MASK (0x10000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL2_L1TROW_SHIFT (0x0000001CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL2_L1TROW_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL2_L1DAET_MASK (0x20000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL2_L1DAET_SHIFT (0x0000001DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL2_L1DAET_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL2_L1CSC_MASK (0x40000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL2_L1CSC_SHIFT (0x0000001EU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL2_L1CSC_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL2_L1UPACR_MASK (0x80000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL2_L1UPACR_SHIFT (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_LOW_POWER_DEBUG_AND_CONTROL2_L1UPACR_MAX (0x00000001U)

/* TL_INTERNAL_CONTROL */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_TL_INTERNAL_CONTROL_ECFLR_MASK    (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_TL_INTERNAL_CONTROL_ECFLR_SHIFT   (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_TL_INTERNAL_CONTROL_ECFLR_MAX     (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_TL_INTERNAL_CONTROL_RES1_MASK     (0xFFFFFFFEU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_TL_INTERNAL_CONTROL_RES1_SHIFT    (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_TL_INTERNAL_CONTROL_RES1_MAX      (0x7FFFFFFFU)

/* I_DTI_ATS_STATUS */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_STATUS_WRONGITAG_MASK   (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_STATUS_WRONGITAG_SHIFT  (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_STATUS_WRONGITAG_MAX    (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_STATUS_NOTAG_MASK       (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_STATUS_NOTAG_SHIFT      (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_STATUS_NOTAG_MAX        (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_STATUS_INVREQIGNORED_MASK (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_STATUS_INVREQIGNORED_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_STATUS_INVREQIGNORED_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_STATUS_ITAGTIMEOUT_MASK (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_STATUS_ITAGTIMEOUT_SHIFT (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_STATUS_ITAGTIMEOUT_MAX  (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_STATUS_R12_MASK         (0x0000FFF0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_STATUS_R12_SHIFT        (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_STATUS_R12_MAX          (0x00000FFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_STATUS_CONSTATE_MASK    (0x00010000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_STATUS_CONSTATE_SHIFT   (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_STATUS_CONSTATE_MAX     (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_STATUS_ITAG_MASK        (0x003E0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_STATUS_ITAG_SHIFT       (0x00000011U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_STATUS_ITAG_MAX         (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_STATUS_R10_MASK         (0xFFC00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_STATUS_R10_SHIFT        (0x00000016U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_STATUS_R10_MAX          (0x000003FFU)

/* I_DTI_ATS_CTRL */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_CTRL_INVTIMERCF_MASK    (0x000FFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_CTRL_INVTIMERCF_SHIFT   (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_CTRL_INVTIMERCF_MAX     (0x000FFFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_CTRL_INVTIMERCC_MASK    (0x07F00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_CTRL_INVTIMERCC_SHIFT   (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_CTRL_INVTIMERCC_MAX     (0x0000007FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_CTRL_CONREQ_MASK        (0x08000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_CTRL_CONREQ_SHIFT       (0x0000001BU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_CTRL_CONREQ_MAX         (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_CTRL_DISCONREQ_MASK     (0x10000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_CTRL_DISCONREQ_SHIFT    (0x0000001CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_CTRL_DISCONREQ_MAX      (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_CTRL_LDCTRL_MASK        (0x20000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_CTRL_LDCTRL_SHIFT       (0x0000001DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_CTRL_LDCTRL_MAX         (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_CTRL_R3_MASK            (0xC0000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_CTRL_R3_SHIFT           (0x0000001EU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_DTI_ATS_CTRL_R3_MAX             (0x00000003U)

/* I_SCALED_FLOW_CONTROL_MGMT_VC_SELECT_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_VC_SELECT_REG_SFCVCS_MASK (0x0000000FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_VC_SELECT_REG_SFCVCS_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_VC_SELECT_REG_SFCVCS_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_VC_SELECT_REG_RES3116_MASK (0xFFFFFFF0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_VC_SELECT_REG_RES3116_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_VC_SELECT_REG_RES3116_MAX (0x0FFFFFFFU)

/* I_SCALED_FLOW_CONTROL_MGMT_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_LPHCS_MASK (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_LPHCS_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_LPHCS_MAX (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_LPPCS_MASK (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_LPPCS_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_LPPCS_MAX (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_LNPHCS_MASK (0x00000030U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_LNPHCS_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_LNPHCS_MAX (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_LNPPCS_MASK (0x000000C0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_LNPPCS_SHIFT (0x00000006U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_LNPPCS_MAX (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_LCHCS_MASK (0x00000300U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_LCHCS_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_LCHCS_MAX (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_LCPCS_MASK (0x00000C00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_LCPCS_SHIFT (0x0000000AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_LCPCS_MAX (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_RES1_MASK (0x0000F000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_RES1_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_RES1_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_RPHCS_MASK (0x00030000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_RPHCS_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_RPHCS_MAX (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_RPPCS_MASK (0x000C0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_RPPCS_SHIFT (0x00000012U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_RPPCS_MAX (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_RNPHCS_MASK (0x00300000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_RNPHCS_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_RNPHCS_MAX (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_RNPPCS_MASK (0x00C00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_RNPPCS_SHIFT (0x00000016U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_RNPPCS_MAX (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_RCHCS_MASK (0x03000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_RCHCS_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_RCHCS_MAX (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_RCPCS_MASK (0x0C000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_RCPCS_SHIFT (0x0000001AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_RCPCS_MAX (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_RES2_MASK (0xF0000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_RES2_SHIFT (0x0000001CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_SCALED_FLOW_CONTROL_MGMT_REG_RES2_MAX (0x0000000FU)

/* I_MARGINING_PARAMETERS_1_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_MVS_MASK (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_MVS_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_MVS_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_MINDUDVS_MASK (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_MINDUDVS_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_MINDUDVS_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_MINDLRTS_MASK (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_MINDLRTS_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_MINDLRTS_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_MSRM_MASK (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_MSRM_SHIFT (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_MSRM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_MIES_MASK (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_MIES_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_MIES_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_MNVS_MASK (0x00000FE0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_MNVS_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_MNVS_MAX (0x0000007FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_MNTS_MASK (0x0003F000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_MNTS_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_MNTS_MAX (0x0000003FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_MMTO_MASK (0x00FC0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_MMTO_SHIFT (0x00000012U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_MMTO_MAX (0x0000003FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_MMVO_MASK (0x3F000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_MMVO_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_MMVO_MAX (0x0000003FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_RES_MASK (0xC0000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_RES_SHIFT (0x0000001EU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_1_REG_RES_MAX (0x00000003U)

/* I_MARGINING_PARAMETERS_2_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_2_REG_MSRV_MASK (0x0000003FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_2_REG_MSRV_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_2_REG_MSRV_MAX (0x0000003FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_2_REG_MSRT_MASK (0x00000FC0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_2_REG_MSRT_SHIFT (0x00000006U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_2_REG_MSRT_MAX (0x0000003FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_2_REG_MML_MASK (0x0001F000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_2_REG_MML_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_2_REG_MML_MAX (0x0000001FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_2_REG_RES1_MASK (0xFFFE0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_2_REG_RES1_SHIFT (0x00000011U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_PARAMETERS_2_REG_RES1_MAX (0x00007FFFU)

/* I_MARGINING_LOCAL_CONTROL_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_LOCAL_CONTROL_REG_MSR_MASK (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_LOCAL_CONTROL_REG_MSR_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_LOCAL_CONTROL_REG_MSR_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_LOCAL_CONTROL_REG_AMCNG4_MASK (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_LOCAL_CONTROL_REG_AMCNG4_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_LOCAL_CONTROL_REG_AMCNG4_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_LOCAL_CONTROL_REG_DMSUSC_MASK (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_LOCAL_CONTROL_REG_DMSUSC_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_LOCAL_CONTROL_REG_DMSUSC_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_LOCAL_CONTROL_REG_RES_MASK (0x1FFFFFF8U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_LOCAL_CONTROL_REG_RES_SHIFT (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_LOCAL_CONTROL_REG_RES_MAX (0x03FFFFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_LOCAL_CONTROL_REG_WAWTC_MASK (0xE0000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_LOCAL_CONTROL_REG_WAWTC_SHIFT (0x0000001DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_LOCAL_CONTROL_REG_WAWTC_MAX (0x00000007U)

/* I_MARGINING_ERROR_STATUS1_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS1_REG_ISWMC_MASK (0x0000FFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS1_REG_ISWMC_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS1_REG_ISWMC_MAX (0x0000FFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS1_REG_ISWMCLN_MASK (0x000F0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS1_REG_ISWMCLN_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS1_REG_ISWMCLN_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS1_REG_RES_MASK (0xFFF00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS1_REG_RES_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS1_REG_RES_MAX (0x00000FFFU)

/* I_MARGINING_ERROR_STATUS2_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS2_REG_IPHYMC_MASK (0x000000FFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS2_REG_IPHYMC_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS2_REG_IPHYMC_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS2_REG_IPHYMCLN_MASK (0x00000F00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS2_REG_IPHYMCLN_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS2_REG_IPHYMCLN_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS2_REG_RES12_MASK (0x00003000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS2_REG_RES12_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS2_REG_RES12_MAX (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS2_REG_WAWTLN_MASK (0x0003C000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS2_REG_WAWTLN_SHIFT (0x0000000EU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS2_REG_WAWTLN_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS2_REG_UPRLN_MASK (0x003C0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS2_REG_UPRLN_SHIFT (0x00000012U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS2_REG_UPRLN_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS2_REG_RES22_MASK (0xFFC00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS2_REG_RES22_SHIFT (0x00000016U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_MARGINING_ERROR_STATUS2_REG_RES22_MAX (0x000003FFU)

/* I_LOCAL_ERROR_STATUS_2_REGISTER */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_MSIMSKCLST_MASK (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_MSIMSKCLST_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_MSIMSKSETST_MASK (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_MSIMSKSETST_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_MSIMSKSETST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_MSIXMSKCLST_MASK (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_MSIXMSKCLST_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_MSIXMSKSETST_MASK (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_MSIXMSKSETST_SHIFT (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_MSIXMSKSETST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_ISWMCR_MASK (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_ISWMCR_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_ISWMCR_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_IPHYMCR_MASK (0x00000020U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_IPHYMCR_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_IPHYMCR_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_WAWTE_MASK (0x00000040U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_WAWTE_SHIFT (0x00000006U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_WAWTE_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_UPRR_MASK (0x00000080U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_UPRR_SHIFT (0x00000007U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_UPRR_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_NFTSTOS_MASK (0x00000100U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_NFTSTOS_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_NFTSTOS_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_PTMCNTAINV_MASK (0x00000200U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_PTMCNTAINV_SHIFT (0x00000009U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_PTMCNTAINV_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_R10_MASK (0x00000400U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_R10_SHIFT (0x0000000AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_R10_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_R13_11_MASK (0x00003800U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_R13_11_SHIFT (0x0000000BU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_R13_11_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_LEQRQIN_MASK (0x00004000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_LEQRQIN_SHIFT (0x0000000EU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_LEQRQIN_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_R31_MASK (0xFFFF8000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_R31_SHIFT (0x0000000FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_ERROR_STATUS_2_REGISTER_R31_MAX (0x0001FFFFU)

/* I_LOCAL_INTRPT_MASK_2_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_MSIMSKCL_MASK (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_MSIMSKCL_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_MSIMSKCL_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_MSIMSKSET_MASK (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_MSIMSKSET_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_MSIMSKSET_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_MSIXMSKCL_MASK (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_MSIXMSKCL_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_MSIXMSKCL_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_MSIXMSKSET_MASK (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_MSIXMSKSET_SHIFT (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_MSIXMSKSET_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_ISWMEM_MASK (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_ISWMEM_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_ISWMEM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_IPHYMEM_MASK (0x00000020U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_IPHYMEM_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_IPHYMEM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_WAWTEM_MASK (0x00000040U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_WAWTEM_SHIFT (0x00000006U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_WAWTEM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_UPREM_MASK (0x00000080U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_UPREM_SHIFT (0x00000007U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_UPREM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_NFTSTOM_MASK (0x00000100U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_NFTSTOM_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_NFTSTOM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_PCAIM_MASK (0x00000200U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_PCAIM_SHIFT (0x00000009U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_PCAIM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_R10_MASK (0x00000400U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_R10_SHIFT (0x0000000AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_R10_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_R13_11_MASK (0x00003800U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_R13_11_SHIFT (0x0000000BU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_R13_11_MAX (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_LEQRQINM_MASK (0x00004000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_LEQRQINM_SHIFT (0x0000000EU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_LEQRQINM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_R31_MASK (0xFFFF8000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_R31_SHIFT (0x0000000FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LOCAL_INTRPT_MASK_2_REG_R31_MAX (0x0001FFFFU)

/* MSI_MASK_CLEARED_STATUS_1 */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_PF0MSIMSKCLST_MASK (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_PF0MSIMSKCLST_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_PF0MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_PF1MSIMSKCLST_MASK (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_PF1MSIMSKCLST_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_PF1MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_PF2MSIMSKCLST_MASK (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_PF2MSIMSKCLST_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_PF2MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_PF3MSIMSKCLST_MASK (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_PF3MSIMSKCLST_SHIFT (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_PF3MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_PF4MSIMSKCLST_MASK (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_PF4MSIMSKCLST_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_PF4MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_PF5MSIMSKCLST_MASK (0x00000020U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_PF5MSIMSKCLST_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_PF5MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF0MSIMSKCLST_MASK (0x00000040U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF0MSIMSKCLST_SHIFT (0x00000006U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF0MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF1MSIMSKCLST_MASK (0x00000080U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF1MSIMSKCLST_SHIFT (0x00000007U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF1MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF2MSIMSKCLST_MASK (0x00000100U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF2MSIMSKCLST_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF2MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF3MSIMSKCLST_MASK (0x00000200U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF3MSIMSKCLST_SHIFT (0x00000009U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF3MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF4MSIMSKCLST_MASK (0x00000400U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF4MSIMSKCLST_SHIFT (0x0000000AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF4MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF5MSIMSKCLST_MASK (0x00000800U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF5MSIMSKCLST_SHIFT (0x0000000BU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF5MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF6MSIMSKCLST_MASK (0x00001000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF6MSIMSKCLST_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF6MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF7MSIMSKCLST_MASK (0x00002000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF7MSIMSKCLST_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF7MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF8MSIMSKCLST_MASK (0x00004000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF8MSIMSKCLST_SHIFT (0x0000000EU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF8MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF9MSIMSKCLST_MASK (0x00008000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF9MSIMSKCLST_SHIFT (0x0000000FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF9MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF10MSIMSKCLST_MASK (0x00010000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF10MSIMSKCLST_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF10MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF11MSIMSKCLST_MASK (0x00020000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF11MSIMSKCLST_SHIFT (0x00000011U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF11MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF12MSIMSKCLST_MASK (0x00040000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF12MSIMSKCLST_SHIFT (0x00000012U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF12MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF13MSIMSKCLST_MASK (0x00080000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF13MSIMSKCLST_SHIFT (0x00000013U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF13MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF14MSIMSKCLST_MASK (0x00100000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF14MSIMSKCLST_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF14MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF15MSIMSKCLST_MASK (0x00200000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF15MSIMSKCLST_SHIFT (0x00000015U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_VF15MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_R31_MASK (0xFFC00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_R31_SHIFT (0x00000016U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_CLEARED_STATUS_1_R31_MAX (0x000003FFU)

/* MSI_MASK_SET_STATUS_1 */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_PF0MSIMSKCLST_MASK (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_PF0MSIMSKCLST_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_PF0MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_PF1MSIMSKCLST_MASK (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_PF1MSIMSKCLST_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_PF1MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_PF2MSIMSKCLST_MASK (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_PF2MSIMSKCLST_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_PF2MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_PF3MSIMSKCLST_MASK (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_PF3MSIMSKCLST_SHIFT (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_PF3MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_PF4MSIMSKCLST_MASK (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_PF4MSIMSKCLST_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_PF4MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_PF5MSIMSKCLST_MASK (0x00000020U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_PF5MSIMSKCLST_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_PF5MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF0MSIMSKCLST_MASK (0x00000040U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF0MSIMSKCLST_SHIFT (0x00000006U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF0MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF1MSIMSKCLST_MASK (0x00000080U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF1MSIMSKCLST_SHIFT (0x00000007U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF1MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF2MSIMSKCLST_MASK (0x00000100U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF2MSIMSKCLST_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF2MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF3MSIMSKCLST_MASK (0x00000200U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF3MSIMSKCLST_SHIFT (0x00000009U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF3MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF4MSIMSKCLST_MASK (0x00000400U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF4MSIMSKCLST_SHIFT (0x0000000AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF4MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF5MSIMSKCLST_MASK (0x00000800U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF5MSIMSKCLST_SHIFT (0x0000000BU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF5MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF6MSIMSKCLST_MASK (0x00001000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF6MSIMSKCLST_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF6MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF7MSIMSKCLST_MASK (0x00002000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF7MSIMSKCLST_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF7MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF8MSIMSKCLST_MASK (0x00004000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF8MSIMSKCLST_SHIFT (0x0000000EU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF8MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF9MSIMSKCLST_MASK (0x00008000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF9MSIMSKCLST_SHIFT (0x0000000FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF9MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF10MSIMSKCLST_MASK (0x00010000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF10MSIMSKCLST_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF10MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF11MSIMSKCLST_MASK (0x00020000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF11MSIMSKCLST_SHIFT (0x00000011U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF11MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF12MSIMSKCLST_MASK (0x00040000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF12MSIMSKCLST_SHIFT (0x00000012U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF12MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF13MSIMSKCLST_MASK (0x00080000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF13MSIMSKCLST_SHIFT (0x00000013U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF13MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF14MSIMSKCLST_MASK (0x00100000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF14MSIMSKCLST_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF14MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF15MSIMSKCLST_MASK (0x00200000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF15MSIMSKCLST_SHIFT (0x00000015U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_VF15MSIMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_R31_MASK    (0xFFC00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_R31_SHIFT   (0x00000016U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSI_MASK_SET_STATUS_1_R31_MAX     (0x000003FFU)

/* MSIX_FUNCTION_MASK_CLEARED_STATUS_1 */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_PF0MSIXMSKCLST_MASK (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_PF0MSIXMSKCLST_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_PF0MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_PF1MSIXMSKCLST_MASK (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_PF1MSIXMSKCLST_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_PF1MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_PF2MSIXMSKCLST_MASK (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_PF2MSIXMSKCLST_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_PF2MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_PF3MSIXMSKCLST_MASK (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_PF3MSIXMSKCLST_SHIFT (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_PF3MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_PF4MSIXMSKCLST_MASK (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_PF4MSIXMSKCLST_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_PF4MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_PF5MSIXMSKCLST_MASK (0x00000020U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_PF5MSIXMSKCLST_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_PF5MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF0MSIXMSKCLST_MASK (0x00000040U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF0MSIXMSKCLST_SHIFT (0x00000006U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF0MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF1MSIXMSKCLST_MASK (0x00000080U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF1MSIXMSKCLST_SHIFT (0x00000007U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF1MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF2MSIXMSKCLST_MASK (0x00000100U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF2MSIXMSKCLST_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF2MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF3MSIXMSKCLST_MASK (0x00000200U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF3MSIXMSKCLST_SHIFT (0x00000009U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF3MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF4MSIXMSKCLST_MASK (0x00000400U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF4MSIXMSKCLST_SHIFT (0x0000000AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF4MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF5MSIXMSKCLST_MASK (0x00000800U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF5MSIXMSKCLST_SHIFT (0x0000000BU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF5MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF6MSIXMSKCLST_MASK (0x00001000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF6MSIXMSKCLST_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF6MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF7MSIXMSKCLST_MASK (0x00002000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF7MSIXMSKCLST_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF7MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF8MSIXMSKCLST_MASK (0x00004000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF8MSIXMSKCLST_SHIFT (0x0000000EU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF8MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF9MSIXMSKCLST_MASK (0x00008000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF9MSIXMSKCLST_SHIFT (0x0000000FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF9MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF10MSIXMSKCLST_MASK (0x00010000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF10MSIXMSKCLST_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF10MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF11MSIXMSKCLST_MASK (0x00020000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF11MSIXMSKCLST_SHIFT (0x00000011U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF11MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF12MSIXMSKCLST_MASK (0x00040000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF12MSIXMSKCLST_SHIFT (0x00000012U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF12MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF13MSIXMSKCLST_MASK (0x00080000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF13MSIXMSKCLST_SHIFT (0x00000013U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF13MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF14MSIXMSKCLST_MASK (0x00100000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF14MSIXMSKCLST_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF14MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF15MSIXMSKCLST_MASK (0x00200000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF15MSIXMSKCLST_SHIFT (0x00000015U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_VF15MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_R31_MASK (0xFFC00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_R31_SHIFT (0x00000016U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_CLEARED_STATUS_1_R31_MAX (0x000003FFU)

/* MSIX_FUNCTION_MASK_SET_STATUS_1 */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_PF0MSIXMSKCLST_MASK (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_PF0MSIXMSKCLST_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_PF0MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_PF1MSIXMSKCLST_MASK (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_PF1MSIXMSKCLST_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_PF1MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_PF2MSIXMSKCLST_MASK (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_PF2MSIXMSKCLST_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_PF2MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_PF3MSIXMSKCLST_MASK (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_PF3MSIXMSKCLST_SHIFT (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_PF3MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_PF4MSIXMSKCLST_MASK (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_PF4MSIXMSKCLST_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_PF4MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_PF5MSIXMSKCLST_MASK (0x00000020U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_PF5MSIXMSKCLST_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_PF5MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF0MSIXMSKCLST_MASK (0x00000040U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF0MSIXMSKCLST_SHIFT (0x00000006U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF0MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF1MSIXMSKCLST_MASK (0x00000080U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF1MSIXMSKCLST_SHIFT (0x00000007U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF1MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF2MSIXMSKCLST_MASK (0x00000100U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF2MSIXMSKCLST_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF2MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF3MSIXMSKCLST_MASK (0x00000200U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF3MSIXMSKCLST_SHIFT (0x00000009U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF3MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF4MSIXMSKCLST_MASK (0x00000400U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF4MSIXMSKCLST_SHIFT (0x0000000AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF4MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF5MSIXMSKCLST_MASK (0x00000800U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF5MSIXMSKCLST_SHIFT (0x0000000BU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF5MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF6MSIXMSKCLST_MASK (0x00001000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF6MSIXMSKCLST_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF6MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF7MSIXMSKCLST_MASK (0x00002000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF7MSIXMSKCLST_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF7MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF8MSIXMSKCLST_MASK (0x00004000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF8MSIXMSKCLST_SHIFT (0x0000000EU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF8MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF9MSIXMSKCLST_MASK (0x00008000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF9MSIXMSKCLST_SHIFT (0x0000000FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF9MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF10MSIXMSKCLST_MASK (0x00010000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF10MSIXMSKCLST_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF10MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF11MSIXMSKCLST_MASK (0x00020000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF11MSIXMSKCLST_SHIFT (0x00000011U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF11MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF12MSIXMSKCLST_MASK (0x00040000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF12MSIXMSKCLST_SHIFT (0x00000012U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF12MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF13MSIXMSKCLST_MASK (0x00080000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF13MSIXMSKCLST_SHIFT (0x00000013U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF13MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF14MSIXMSKCLST_MASK (0x00100000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF14MSIXMSKCLST_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF14MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF15MSIXMSKCLST_MASK (0x00200000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF15MSIXMSKCLST_SHIFT (0x00000015U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_VF15MSIXMSKCLST_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_R31_MASK (0xFFC00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_R31_SHIFT (0x00000016U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_MSIX_FUNCTION_MASK_SET_STATUS_1_R31_MAX (0x000003FFU)

/* I_LD_CTRL */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LD_CTRL_LDTIMER_MASK            (0x00FFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LD_CTRL_LDTIMER_SHIFT           (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LD_CTRL_LDTIMER_MAX             (0x00FFFFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LD_CTRL_AUTO_EN_MASK            (0x01000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LD_CTRL_AUTO_EN_SHIFT           (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LD_CTRL_AUTO_EN_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LD_CTRL_R7_MASK                 (0xFE000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LD_CTRL_R7_SHIFT                (0x00000019U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_LD_CTRL_R7_MAX                  (0x0000007FU)

/* RX_ELEC_IDLE_FILTER_CONTROL */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_RX_ELEC_IDLE_FILTER_CONTROL_GFLD_MASK (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_RX_ELEC_IDLE_FILTER_CONTROL_GFLD_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_RX_ELEC_IDLE_FILTER_CONTROL_GFLD_MAX (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_RX_ELEC_IDLE_FILTER_CONTROL_RSVGFLD_MASK (0x0000FFFCU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_RX_ELEC_IDLE_FILTER_CONTROL_RSVGFLD_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_RX_ELEC_IDLE_FILTER_CONTROL_RSVGFLD_MAX (0x00003FFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_RX_ELEC_IDLE_FILTER_CONTROL_GFLCC_MASK (0x00FF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_RX_ELEC_IDLE_FILTER_CONTROL_GFLCC_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_RX_ELEC_IDLE_FILTER_CONTROL_GFLCC_MAX (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_RX_ELEC_IDLE_FILTER_CONTROL_GFLCP_MASK (0xFF000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_RX_ELEC_IDLE_FILTER_CONTROL_GFLCP_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_RX_ELEC_IDLE_FILTER_CONTROL_GFLCP_MAX (0x000000FFU)

/* I_PTM_LOCAL_CONTROL_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_PTMRQM_MASK (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_PTMRQM_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_PTMRQM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_PTMRQEN_MASK (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_PTMRQEN_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_PTMRQEN_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_RES2_MASK (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_RES2_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_RES2_MAX  (0x00000003U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_PTMRFRSC_MASK (0x000000F0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_PTMRFRSC_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_PTMRFRSC_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_PTMRFRVL_MASK (0x00000F00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_PTMRFRVL_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_PTMRFRVL_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_PTMRINT_MASK (0x0000F000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_PTMRINT_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_PTMRINT_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_PTMRSM_MASK (0x00010000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_PTMRSM_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_PTMRSM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_PTMRSEN_MASK (0x00020000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_PTMRSEN_SHIFT (0x00000011U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_PTMRSEN_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_RES18_MASK (0x07FC0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_RES18_SHIFT (0x00000012U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_RES18_MAX (0x000001FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_INVPTMCNT_MASK (0x08000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_INVPTMCNT_SHIFT (0x0000001BU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_INVPTMCNT_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_DAINVCNT_MASK (0x10000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_DAINVCNT_SHIFT (0x0000001CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_DAINVCNT_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_RES29_MASK (0xE0000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_RES29_SHIFT (0x0000001DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_CONTROL_REG_RES29_MAX (0x00000007U)

/* I_PTM_LOCAL_STATUS_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_STATUS_REG_PTMCNST_MASK (0x0000000FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_STATUS_REG_PTMCNST_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_STATUS_REG_PTMCNST_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_STATUS_REG_RES3_MASK  (0xFFFFFFF0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_STATUS_REG_RES3_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LOCAL_STATUS_REG_RES3_MAX   (0x0FFFFFFFU)

/* I_PTM_LATENCY_PARAMETERS_INDEX_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LATENCY_PARAMETERS_INDEX_REG_PTMLATIN_MASK (0x0000000FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LATENCY_PARAMETERS_INDEX_REG_PTMLATIN_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LATENCY_PARAMETERS_INDEX_REG_PTMLATIN_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LATENCY_PARAMETERS_INDEX_REG_RES4_MASK (0xFFFFFFF0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LATENCY_PARAMETERS_INDEX_REG_RES4_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LATENCY_PARAMETERS_INDEX_REG_RES4_MAX (0x0FFFFFFFU)

/* I_PTM_LATENCY_PARAMETERS_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LATENCY_PARAMETERS_REG_PTMTXLAT_MASK (0x000003FFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LATENCY_PARAMETERS_REG_PTMTXLAT_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LATENCY_PARAMETERS_REG_PTMTXLAT_MAX (0x000003FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LATENCY_PARAMETERS_REG_PTMRXLAT_MASK (0x000FFC00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LATENCY_PARAMETERS_REG_PTMRXLAT_SHIFT (0x0000000AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LATENCY_PARAMETERS_REG_PTMRXLAT_MAX (0x000003FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LATENCY_PARAMETERS_REG_RES20_MASK (0x00F00000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LATENCY_PARAMETERS_REG_RES20_SHIFT (0x00000014U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LATENCY_PARAMETERS_REG_RES20_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LATENCY_PARAMETERS_REG_TXDLTUN_MASK (0x0F000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LATENCY_PARAMETERS_REG_TXDLTUN_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LATENCY_PARAMETERS_REG_TXDLTUN_MAX (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LATENCY_PARAMETERS_REG_RXDLTUN_MASK (0xF0000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LATENCY_PARAMETERS_REG_RXDLTUN_SHIFT (0x0000001CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_LATENCY_PARAMETERS_REG_RXDLTUN_MAX (0x0000000FU)

/* I_PTM_CONTEXT_1_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_1_REG_PTMT1T2_MASK  (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_1_REG_PTMT1T2_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_1_REG_PTMT1T2_MAX   (0xFFFFFFFFU)

/* I_PTM_CONTEXT_2_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_2_REG_PTMT1T2U_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_2_REG_PTMT1T2U_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_2_REG_PTMT1T2U_MAX  (0xFFFFFFFFU)

/* I_PTM_CONTEXT_3_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_3_REG_PTMT4T3_MASK  (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_3_REG_PTMT4T3_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_3_REG_PTMT4T3_MAX   (0xFFFFFFFFU)

/* I_PTM_CONTEXT_4_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_4_REG_PTMT4T3U_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_4_REG_PTMT4T3U_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_4_REG_PTMT4T3U_MAX  (0xFFFFFFFFU)

/* I_PTM_CONTEXT_5_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_5_REG_PTMT1KT2K_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_5_REG_PTMT1KT2K_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_5_REG_PTMT1KT2K_MAX (0xFFFFFFFFU)

/* I_PTM_CONTEXT_6_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_6_REG_PTMT1KT2KU_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_6_REG_PTMT1KT2KU_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_6_REG_PTMT1KT2KU_MAX (0xFFFFFFFFU)

/* I_PTM_CONTEXT_7_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_7_REG_PTMT4KT3K_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_7_REG_PTMT4KT3K_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_7_REG_PTMT4KT3K_MAX (0xFFFFFFFFU)

/* I_PTM_CONTEXT_8_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_8_REG_PTMT4KT3KU_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_8_REG_PTMT4KT3KU_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_8_REG_PTMT4KT3KU_MAX (0xFFFFFFFFU)

/* I_PTM_CONTEXT_9_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_9_REG_PTMT3MT2_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_9_REG_PTMT3MT2_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_9_REG_PTMT3MT2_MAX  (0xFFFFFFFFU)

/* I_PTM_CONTEXT_10_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_10_REG_PTMMSTT1T_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_10_REG_PTMMSTT1T_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_10_REG_PTMMSTT1T_MAX (0xFFFFFFFFU)

/* I_PTM_CONTEXT_11_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_11_REG_PTMMSTT1TU_MASK (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_11_REG_PTMMSTT1TU_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_PTM_CONTEXT_11_REG_PTMMSTT1TU_MAX (0xFFFFFFFFU)

/* I_ASF_INTRPT_STATUS */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_STATUS_SRCORER_MASK  (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_STATUS_SRCORER_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_STATUS_SRCORER_MAX   (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_STATUS_SRUCORER_MASK (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_STATUS_SRUCORER_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_STATUS_SRUCORER_MAX  (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_STATUS_DAPER_MASK    (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_STATUS_DAPER_SHIFT   (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_STATUS_DAPER_MAX     (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_STATUS_CSRER_MASK    (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_STATUS_CSRER_SHIFT   (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_STATUS_CSRER_MAX     (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_STATUS_TRANSTOER_MASK (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_STATUS_TRANSTOER_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_STATUS_TRANSTOER_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_STATUS_PROTER_MASK   (0x00000020U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_STATUS_PROTER_SHIFT  (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_STATUS_PROTER_MAX    (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_STATUS_INTEGRER_MASK (0x00000040U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_STATUS_INTEGRER_SHIFT (0x00000006U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_STATUS_INTEGRER_MAX  (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_STATUS_R31_MASK      (0xFFFFFF80U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_STATUS_R31_SHIFT     (0x00000007U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_STATUS_R31_MAX       (0x01FFFFFFU)

/* I_ASF_INTRPT_RAW_STATUS */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_RAW_STATUS_SRCORER_MASK (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_RAW_STATUS_SRCORER_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_RAW_STATUS_SRCORER_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_RAW_STATUS_SRUCORER_MASK (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_RAW_STATUS_SRUCORER_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_RAW_STATUS_SRUCORER_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_RAW_STATUS_DAPER_MASK (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_RAW_STATUS_DAPER_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_RAW_STATUS_DAPER_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_RAW_STATUS_CSRER_MASK (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_RAW_STATUS_CSRER_SHIFT (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_RAW_STATUS_CSRER_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_RAW_STATUS_TRANSTOER_MASK (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_RAW_STATUS_TRANSTOER_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_RAW_STATUS_TRANSTOER_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_RAW_STATUS_PROTER_MASK (0x00000020U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_RAW_STATUS_PROTER_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_RAW_STATUS_PROTER_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_RAW_STATUS_INTEGRER_MASK (0x00000040U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_RAW_STATUS_INTEGRER_SHIFT (0x00000006U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_RAW_STATUS_INTEGRER_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_RAW_STATUS_R31_MASK  (0xFFFFFF80U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_RAW_STATUS_R31_SHIFT (0x00000007U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_RAW_STATUS_R31_MAX   (0x01FFFFFFU)

/* I_ASF_INTRPT_MASK_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_MASK_REG_SRCORERM_MASK (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_MASK_REG_SRCORERM_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_MASK_REG_SRCORERM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_MASK_REG_SRUCORERM_MASK (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_MASK_REG_SRUCORERM_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_MASK_REG_SRUCORERM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_MASK_REG_DAPERM_MASK (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_MASK_REG_DAPERM_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_MASK_REG_DAPERM_MAX  (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_MASK_REG_CSRERM_MASK (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_MASK_REG_CSRERM_SHIFT (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_MASK_REG_CSRERM_MAX  (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_MASK_REG_TRANTOEM_MASK (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_MASK_REG_TRANTOEM_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_MASK_REG_TRANTOEM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_MASK_REG_PROTERM_MASK (0x00000020U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_MASK_REG_PROTERM_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_MASK_REG_PROTERM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_MASK_REG_INTEGRERM_MASK (0x00000040U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_MASK_REG_INTEGRERM_SHIFT (0x00000006U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_MASK_REG_INTEGRERM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_MASK_REG_R31_MASK    (0xFFFFFF80U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_MASK_REG_R31_SHIFT   (0x00000007U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_MASK_REG_R31_MAX     (0x01FFFFFFU)

/* I_ASF_INTRPT_TEST */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_TEST_SRCORERT_MASK   (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_TEST_SRCORERT_SHIFT  (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_TEST_SRCORERT_MAX    (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_TEST_SRUCORERT_MASK  (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_TEST_SRUCORERT_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_TEST_SRUCORERT_MAX   (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_TEST_DAPERT_MASK     (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_TEST_DAPERT_SHIFT    (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_TEST_DAPERT_MAX      (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_TEST_CSRERT_MASK     (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_TEST_CSRERT_SHIFT    (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_TEST_CSRERT_MAX      (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_TEST_TRANTOET_MASK   (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_TEST_TRANTOET_SHIFT  (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_TEST_TRANTOET_MAX    (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_TEST_PROTERT_MASK    (0x00000020U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_TEST_PROTERT_SHIFT   (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_TEST_PROTERT_MAX     (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_TEST_INTEGRERT_MASK  (0x00000040U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_TEST_INTEGRERT_SHIFT (0x00000006U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_TEST_INTEGRERT_MAX   (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_TEST_R31_MASK        (0xFFFFFF80U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_TEST_R31_SHIFT       (0x00000007U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_TEST_R31_MAX         (0x01FFFFFFU)

/* I_ASF_INTRPT_FATAL_NONFATAL_SEL */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_FATAL_NONFATAL_SEL_SRCORERS_MASK (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_FATAL_NONFATAL_SEL_SRCORERS_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_FATAL_NONFATAL_SEL_SRCORERS_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_FATAL_NONFATAL_SEL_SRUCORERS_MASK (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_FATAL_NONFATAL_SEL_SRUCORERS_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_FATAL_NONFATAL_SEL_SRUCORERS_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_FATAL_NONFATAL_SEL_DAPERS_MASK (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_FATAL_NONFATAL_SEL_DAPERS_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_FATAL_NONFATAL_SEL_DAPERS_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_FATAL_NONFATAL_SEL_CSRERS_MASK (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_FATAL_NONFATAL_SEL_CSRERS_SHIFT (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_FATAL_NONFATAL_SEL_CSRERS_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_FATAL_NONFATAL_SEL_TRANTOES_MASK (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_FATAL_NONFATAL_SEL_TRANTOES_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_FATAL_NONFATAL_SEL_TRANTOES_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_FATAL_NONFATAL_SEL_PROTERS_MASK (0x00000020U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_FATAL_NONFATAL_SEL_PROTERS_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_FATAL_NONFATAL_SEL_PROTERS_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_FATAL_NONFATAL_SEL_INTEGRERS_MASK (0x00000040U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_FATAL_NONFATAL_SEL_INTEGRERS_SHIFT (0x00000006U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_FATAL_NONFATAL_SEL_INTEGRERS_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_FATAL_NONFATAL_SEL_R31_MASK (0xFFFFFF80U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_FATAL_NONFATAL_SEL_R31_SHIFT (0x00000007U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_INTRPT_FATAL_NONFATAL_SEL_R31_MAX (0x01FFFFFFU)

/* I_ASF_SRAM_CORR_FAULT_STATUS */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_SRAM_CORR_FAULT_STATUS_SRCORFADR_MASK (0x00FFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_SRAM_CORR_FAULT_STATUS_SRCORFADR_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_SRAM_CORR_FAULT_STATUS_SRCORFADR_MAX (0x00FFFFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_SRAM_CORR_FAULT_STATUS_SRCORFI_MASK (0xFF000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_SRAM_CORR_FAULT_STATUS_SRCORFI_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_SRAM_CORR_FAULT_STATUS_SRCORFI_MAX (0x000000FFU)

/* I_ASF_SRAM_UNCORR_FAULT_STATUS */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_SRAM_UNCORR_FAULT_STATUS_SRUCRFADR_MASK (0x00FFFFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_SRAM_UNCORR_FAULT_STATUS_SRUCRFADR_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_SRAM_UNCORR_FAULT_STATUS_SRUCRFADR_MAX (0x00FFFFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_SRAM_UNCORR_FAULT_STATUS_SRUCORFI_MASK (0xFF000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_SRAM_UNCORR_FAULT_STATUS_SRUCORFI_SHIFT (0x00000018U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_SRAM_UNCORR_FAULT_STATUS_SRUCORFI_MAX (0x000000FFU)

/* I_ASF_SRAM_FAULT_STATSTICS */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_SRAM_FAULT_STATSTICS_SRCORFS_MASK (0x0000FFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_SRAM_FAULT_STATSTICS_SRCORFS_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_SRAM_FAULT_STATSTICS_SRCORFS_MAX (0x0000FFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_SRAM_FAULT_STATSTICS_SRUCORFS_MASK (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_SRAM_FAULT_STATSTICS_SRUCORFS_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_SRAM_FAULT_STATSTICS_SRUCORFS_MAX (0x0000FFFFU)

/* I_ASF_TRANS_TO_CTRL */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_CTRL_TRTOCTRL_MASK (0x0000FFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_CTRL_TRTOCTRL_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_CTRL_TRTOCTRL_MAX  (0x0000FFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_CTRL_R1_MASK       (0x7FFF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_CTRL_R1_SHIFT      (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_CTRL_R1_MAX        (0x00007FFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_CTRL_TRTOEN_MASK   (0x80000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_CTRL_TRTOEN_SHIFT  (0x0000001FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_CTRL_TRTOEN_MAX    (0x00000001U)

/* I_ASF_TRANS_TO_FAULT_MASK */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_PCOMTOM_MASK (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_PCOMTOM_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_PCOMTOM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_LTPLCFTOM_MASK (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_LTPLCFTOM_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_LTPLCFTOM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_LCFLWSTOM_MASK (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_LCFLWSTOM_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_LCFLWSTOM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_LRESPDTOM_MASK (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_LRESPDTOM_SHIFT (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_LRESPDTOM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_HLMSTOM_MASK (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_HLMSTOM_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_HLMSTOM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_HLTGTOM_MASK (0x00000020U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_HLTGTOM_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_HLTGTOM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_AXMSTOM_MASK (0x00000040U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_AXMSTOM_SHIFT (0x00000006U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_AXMSTOM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_AXSLTOM_MASK (0x00000080U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_AXSLTOM_SHIFT (0x00000007U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_AXSLTOM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_LMITOM_MASK (0x00000100U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_LMITOM_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_LMITOM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_APBTOM_MASK (0x00000200U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_APBTOM_SHIFT (0x00000009U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_APBTOM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_DTIUTOM_MASK (0x00000400U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_DTIUTOM_SHIFT (0x0000000AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_DTIUTOM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_DTIDTOM_MASK (0x00000800U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_DTIDTOM_SHIFT (0x0000000BU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_DTIDTOM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_HPHLMSTOM_MASK (0x00001000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_HPHLMSTOM_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_HPHLMSTOM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_HPHLTGTOM_MASK (0x00002000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_HPHLTGTOM_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_HPHLTGTOM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_HPAXMSTM_MASK (0x00004000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_HPAXMSTM_SHIFT (0x0000000EU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_HPAXMSTM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_HPAXSLTO_MASK (0x00008000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_HPAXSLTO_SHIFT (0x0000000FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_HPAXSLTO_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_R31_MASK (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_R31_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_MASK_R31_MAX (0x0000FFFFU)

/* I_ASF_TRANS_TO_FAULT_STATUS */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_PCOMTO_MASK (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_PCOMTO_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_PCOMTO_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_LTPLCFTO_MASK (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_LTPLCFTO_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_LTPLCFTO_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_LCFLWSTO_MASK (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_LCFLWSTO_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_LCFLWSTO_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_LRESPDTO_MASK (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_LRESPDTO_SHIFT (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_LRESPDTO_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_HLMSTO_MASK (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_HLMSTO_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_HLMSTO_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_HLTGTO_MASK (0x00000020U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_HLTGTO_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_HLTGTO_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_AXMSTO_MASK (0x00000040U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_AXMSTO_SHIFT (0x00000006U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_AXMSTO_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_AXSLTO_MASK (0x00000080U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_AXSLTO_SHIFT (0x00000007U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_AXSLTO_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_LMITO_MASK (0x00000100U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_LMITO_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_LMITO_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_APBTOM_MASK (0x00000200U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_APBTOM_SHIFT (0x00000009U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_APBTOM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_DTIUTO_MASK (0x00000400U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_DTIUTO_SHIFT (0x0000000AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_DTIUTO_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_DTIDTO_MASK (0x00000800U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_DTIDTO_SHIFT (0x0000000BU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_DTIDTO_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_HPHLMSTO_MASK (0x00001000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_HPHLMSTO_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_HPHLMSTO_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_HPHLTGTO_MASK (0x00002000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_HPHLTGTO_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_HPHLTGTO_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_HPAXMSTO_MASK (0x00004000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_HPAXMSTO_SHIFT (0x0000000EU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_HPAXMSTO_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_HPAXSLTO_MASK (0x00008000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_HPAXSLTO_SHIFT (0x0000000FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_HPAXSLTO_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_R31_MASK (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_R31_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_TRANS_TO_FAULT_STATUS_R31_MAX (0x0000FFFFU)

/* I_ASF_PROTOCOL_FAULT_MASK */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_DLPROTM_MASK (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_DLPROTM_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_DLPROTM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_POTLRCVM_MASK (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_POTLRCVM_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_POTLRCVM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_FCPROERM_MASK (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_FCPROERM_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_FCPROERM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_CPLTOM_MASK (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_CPLTOM_SHIFT (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_CPLTOM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_CMPLABTM_MASK (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_CMPLABTM_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_CMPLABTM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_UNCPLRCM_MASK (0x00000020U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_UNCPLRCM_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_UNCPLRCM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_RCVROVFLM_MASK (0x00000040U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_RCVROVFLM_SHIFT (0x00000006U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_RCVROVFLM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_MALTLPEM_MASK (0x00000080U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_MALTLPEM_SHIFT (0x00000007U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_MALTLPEM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_ECRCERRM_MASK (0x00000100U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_ECRCERRM_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_ECRCERRM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_USPREQM_MASK (0x00000200U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_USPREQM_SHIFT (0x00000009U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_USPREQM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_PHRCVERM_MASK (0x00000400U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_PHRCVERM_SHIFT (0x0000000AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_PHRCVERM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_BADTLPM_MASK (0x00000800U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_BADTLPM_SHIFT (0x0000000BU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_BADTLPM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_BADDLPM_MASK (0x00001000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_BADDLPM_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_BADDLPM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_RPLROLM_MASK (0x00002000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_RPLROLM_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_RPLROLM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_RPLTOM_MASK (0x00004000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_RPLTOM_SHIFT (0x0000000EU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_RPLTOM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_AXISLDECM_MASK (0x00008000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_AXISLDECM_SHIFT (0x0000000FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_AXISLDECM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_R2_MASK (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_R2_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_MASK_R2_MAX  (0x0000FFFFU)

/* I_ASF_PROTOCOL_FAULT_STATUS_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_DLPROT_MASK (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_DLPROT_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_DLPROT_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_POTLRCV_MASK (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_POTLRCV_SHIFT (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_POTLRCV_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_FCPROER_MASK (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_FCPROER_SHIFT (0x00000002U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_FCPROER_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_CPLTO_MASK (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_CPLTO_SHIFT (0x00000003U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_CPLTO_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_CMPLABT_MASK (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_CMPLABT_SHIFT (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_CMPLABT_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_UNCMLRCV_MASK (0x00000020U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_UNCMLRCV_SHIFT (0x00000005U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_UNCMLRCV_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_RCVROVFL_MASK (0x00000040U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_RCVROVFL_SHIFT (0x00000006U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_RCVROVFL_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_MALTLPER_MASK (0x00000080U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_MALTLPER_SHIFT (0x00000007U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_MALTLPER_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_ECRCERR_MASK (0x00000100U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_ECRCERR_SHIFT (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_ECRCERR_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_USPREQ_MASK (0x00000200U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_USPREQ_SHIFT (0x00000009U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_USPREQ_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_PHRCVER_MASK (0x00000400U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_PHRCVER_SHIFT (0x0000000AU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_PHRCVER_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_BADTLPM_MASK (0x00000800U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_BADTLPM_SHIFT (0x0000000BU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_BADTLPM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_BADDLP_MASK (0x00001000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_BADDLP_SHIFT (0x0000000CU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_BADDLP_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_RPLROL_MASK (0x00002000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_RPLROL_SHIFT (0x0000000DU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_RPLROL_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_RPLTOM_MASK (0x00004000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_RPLTOM_SHIFT (0x0000000EU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_RPLTOM_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_AXISLVDEC_MASK (0x00008000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_AXISLVDEC_SHIFT (0x0000000FU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_AXISLVDEC_MAX (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_R2_MASK (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_R2_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_PROTOCOL_FAULT_STATUS_REG_R2_MAX (0x0000FFFFU)

/* DUAL_TL_CTRL */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_DUAL_TL_CTRL_GPLP_MASK            (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_DUAL_TL_CTRL_GPLP_SHIFT           (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_DUAL_TL_CTRL_GPLP_MAX             (0x00000001U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_DUAL_TL_CTRL_DTLTS_MASK           (0x0000000EU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_DUAL_TL_CTRL_DTLTS_SHIFT          (0x00000001U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_DUAL_TL_CTRL_DTLTS_MAX            (0x00000007U)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_DUAL_TL_CTRL_DTL_RSVD_MASK        (0x000000F0U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_DUAL_TL_CTRL_DTL_RSVD_SHIFT       (0x00000004U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_DUAL_TL_CTRL_DTL_RSVD_MAX         (0x0000000FU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_DUAL_TL_CTRL_DTLAW_MASK           (0x0000FF00U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_DUAL_TL_CTRL_DTLAW_SHIFT          (0x00000008U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_DUAL_TL_CTRL_DTLAW_MAX            (0x000000FFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_DUAL_TL_CTRL_DTLHDRT_MASK         (0x00FF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_DUAL_TL_CTRL_DTLHDRT_SHIFT        (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_DUAL_TL_CTRL_DTLHDRT_MAX          (0x000000FFU)

/* I_ASF_MAGIC_NUM_CTRLLER_VER_REG */

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_MAGIC_NUM_CTRLLER_VER_REG_MGCNM_MASK (0x0000FFFFU)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_MAGIC_NUM_CTRLLER_VER_REG_MGCNM_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_MAGIC_NUM_CTRLLER_VER_REG_MGCNM_MAX (0x0000FFFFU)

#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_MAGIC_NUM_CTRLLER_VER_REG_CNTVER_MASK (0xFFFF0000U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_MAGIC_NUM_CTRLLER_VER_REG_CNTVER_SHIFT (0x00000010U)
#define CSL_PCIE_RP_CORE_LM_I_REGF_LM_PCIE_BASE_I_ASF_MAGIC_NUM_CTRLLER_VER_REG_CNTVER_MAX (0x0000FFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_ADDR0_NUM_BITS_MASK                     (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_ADDR0_NUM_BITS_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_ADDR0_NUM_BITS_MAX                      (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_ADDR0_RSVD_MASK                         (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_ADDR0_RSVD_SHIFT                        (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_ADDR0_RSVD_MAX                          (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_ADDR0_DATA_MASK                         (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_ADDR0_DATA_SHIFT                        (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_ADDR0_DATA_MAX                          (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_ADDR1_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_ADDR1_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_ADDR1_DATA_MAX                          (0xFFFFFFFFU)

/* DESC0 */

#ifdef CSL_MODIFICATION
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC0_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC0_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC0_DATA_MAX                          (0xFFFFFFFFU)
#else /* CSL_MODIFICATION */
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC0_TT_MASK                           (0x0000000FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC0_TT_SHIFT                          (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC0_TT_MAX                            (0x0000000FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC0_ATTR_MASK                         (0x00000070U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC0_ATTR_SHIFT                        (0x00000004U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC0_ATTR_MAX                          (0x00000007U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC0_PTC_MASK                          (0x000E0000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC0_PTC_SHIFT                         (0x00000011U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC0_PTC_MAX                           (0x00000007U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC0_BNEN_MASK                         (0x00800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC0_BNEN_SHIFT                        (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC0_BNEN_MAX                          (0x00000001U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC0_FNUM_MASK                         (0xFF000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC0_FNUM_SHIFT                        (0x00000018U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC0_FNUM_MAX                          (0x000000FFU)
#endif /* CSL_MODIFICATION */


/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC1_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC1_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC1_DATA_MAX                          (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC3_DATA_MASK                         (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC3_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC3_DATA_MAX                          (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC3_RSVD_MASK                         (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC3_RSVD_SHIFT                        (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_DESC3_RSVD_MAX                          (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_AXI_ADDR0_REGION_SIZE_MASK              (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_AXI_ADDR0_REGION_SIZE_SHIFT             (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_AXI_ADDR0_REGION_SIZE_MAX               (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_AXI_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_AXI_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_AXI_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_AXI_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_AXI_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_AXI_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_AXI_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_AXI_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_0_AXI_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_ADDR0_NUM_BITS_MASK                     (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_ADDR0_NUM_BITS_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_ADDR0_NUM_BITS_MAX                      (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_ADDR0_RSVD_MASK                         (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_ADDR0_RSVD_SHIFT                        (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_ADDR0_RSVD_MAX                          (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_ADDR0_DATA_MASK                         (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_ADDR0_DATA_SHIFT                        (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_ADDR0_DATA_MAX                          (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_ADDR1_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_ADDR1_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_ADDR1_DATA_MAX                          (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_DESC0_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_DESC0_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_DESC0_DATA_MAX                          (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_DESC1_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_DESC1_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_DESC1_DATA_MAX                          (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_DESC3_DATA_MASK                         (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_DESC3_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_DESC3_DATA_MAX                          (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_DESC3_RSVD_MASK                         (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_DESC3_RSVD_SHIFT                        (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_DESC3_RSVD_MAX                          (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_AXI_ADDR0_REGION_SIZE_MASK              (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_AXI_ADDR0_REGION_SIZE_SHIFT             (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_AXI_ADDR0_REGION_SIZE_MAX               (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_AXI_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_AXI_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_AXI_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_AXI_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_AXI_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_AXI_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_AXI_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_AXI_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_1_AXI_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_ADDR0_NUM_BITS_MASK                     (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_ADDR0_NUM_BITS_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_ADDR0_NUM_BITS_MAX                      (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_ADDR0_RSVD_MASK                         (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_ADDR0_RSVD_SHIFT                        (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_ADDR0_RSVD_MAX                          (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_ADDR0_DATA_MASK                         (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_ADDR0_DATA_SHIFT                        (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_ADDR0_DATA_MAX                          (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_ADDR1_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_ADDR1_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_ADDR1_DATA_MAX                          (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_DESC0_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_DESC0_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_DESC0_DATA_MAX                          (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_DESC1_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_DESC1_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_DESC1_DATA_MAX                          (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_DESC3_DATA_MASK                         (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_DESC3_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_DESC3_DATA_MAX                          (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_DESC3_RSVD_MASK                         (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_DESC3_RSVD_SHIFT                        (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_DESC3_RSVD_MAX                          (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_AXI_ADDR0_REGION_SIZE_MASK              (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_AXI_ADDR0_REGION_SIZE_SHIFT             (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_AXI_ADDR0_REGION_SIZE_MAX               (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_AXI_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_AXI_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_AXI_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_AXI_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_AXI_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_AXI_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_AXI_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_AXI_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_2_AXI_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_ADDR0_NUM_BITS_MASK                     (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_ADDR0_NUM_BITS_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_ADDR0_NUM_BITS_MAX                      (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_ADDR0_RSVD_MASK                         (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_ADDR0_RSVD_SHIFT                        (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_ADDR0_RSVD_MAX                          (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_ADDR0_DATA_MASK                         (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_ADDR0_DATA_SHIFT                        (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_ADDR0_DATA_MAX                          (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_ADDR1_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_ADDR1_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_ADDR1_DATA_MAX                          (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_DESC0_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_DESC0_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_DESC0_DATA_MAX                          (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_DESC1_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_DESC1_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_DESC1_DATA_MAX                          (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_DESC3_DATA_MASK                         (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_DESC3_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_DESC3_DATA_MAX                          (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_DESC3_RSVD_MASK                         (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_DESC3_RSVD_SHIFT                        (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_DESC3_RSVD_MAX                          (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_AXI_ADDR0_REGION_SIZE_MASK              (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_AXI_ADDR0_REGION_SIZE_SHIFT             (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_AXI_ADDR0_REGION_SIZE_MAX               (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_AXI_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_AXI_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_AXI_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_AXI_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_AXI_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_AXI_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_AXI_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_AXI_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_3_AXI_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_ADDR0_NUM_BITS_MASK                     (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_ADDR0_NUM_BITS_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_ADDR0_NUM_BITS_MAX                      (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_ADDR0_RSVD_MASK                         (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_ADDR0_RSVD_SHIFT                        (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_ADDR0_RSVD_MAX                          (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_ADDR0_DATA_MASK                         (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_ADDR0_DATA_SHIFT                        (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_ADDR0_DATA_MAX                          (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_ADDR1_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_ADDR1_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_ADDR1_DATA_MAX                          (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_DESC0_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_DESC0_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_DESC0_DATA_MAX                          (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_DESC1_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_DESC1_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_DESC1_DATA_MAX                          (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_DESC3_DATA_MASK                         (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_DESC3_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_DESC3_DATA_MAX                          (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_DESC3_RSVD_MASK                         (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_DESC3_RSVD_SHIFT                        (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_DESC3_RSVD_MAX                          (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_AXI_ADDR0_REGION_SIZE_MASK              (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_AXI_ADDR0_REGION_SIZE_SHIFT             (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_AXI_ADDR0_REGION_SIZE_MAX               (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_AXI_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_AXI_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_AXI_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_AXI_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_AXI_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_AXI_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_AXI_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_AXI_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_4_AXI_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_ADDR0_NUM_BITS_MASK                     (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_ADDR0_NUM_BITS_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_ADDR0_NUM_BITS_MAX                      (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_ADDR0_RSVD_MASK                         (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_ADDR0_RSVD_SHIFT                        (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_ADDR0_RSVD_MAX                          (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_ADDR0_DATA_MASK                         (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_ADDR0_DATA_SHIFT                        (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_ADDR0_DATA_MAX                          (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_ADDR1_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_ADDR1_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_ADDR1_DATA_MAX                          (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_DESC0_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_DESC0_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_DESC0_DATA_MAX                          (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_DESC1_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_DESC1_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_DESC1_DATA_MAX                          (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_DESC3_DATA_MASK                         (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_DESC3_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_DESC3_DATA_MAX                          (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_DESC3_RSVD_MASK                         (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_DESC3_RSVD_SHIFT                        (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_DESC3_RSVD_MAX                          (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_AXI_ADDR0_REGION_SIZE_MASK              (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_AXI_ADDR0_REGION_SIZE_SHIFT             (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_AXI_ADDR0_REGION_SIZE_MAX               (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_AXI_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_AXI_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_AXI_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_AXI_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_AXI_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_AXI_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_AXI_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_AXI_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_5_AXI_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_ADDR0_NUM_BITS_MASK                     (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_ADDR0_NUM_BITS_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_ADDR0_NUM_BITS_MAX                      (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_ADDR0_RSVD_MASK                         (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_ADDR0_RSVD_SHIFT                        (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_ADDR0_RSVD_MAX                          (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_ADDR0_DATA_MASK                         (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_ADDR0_DATA_SHIFT                        (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_ADDR0_DATA_MAX                          (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_ADDR1_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_ADDR1_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_ADDR1_DATA_MAX                          (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_DESC0_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_DESC0_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_DESC0_DATA_MAX                          (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_DESC1_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_DESC1_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_DESC1_DATA_MAX                          (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_DESC3_DATA_MASK                         (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_DESC3_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_DESC3_DATA_MAX                          (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_DESC3_RSVD_MASK                         (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_DESC3_RSVD_SHIFT                        (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_DESC3_RSVD_MAX                          (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_AXI_ADDR0_REGION_SIZE_MASK              (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_AXI_ADDR0_REGION_SIZE_SHIFT             (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_AXI_ADDR0_REGION_SIZE_MAX               (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_AXI_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_AXI_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_AXI_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_AXI_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_AXI_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_AXI_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_AXI_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_AXI_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_6_AXI_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_ADDR0_NUM_BITS_MASK                     (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_ADDR0_NUM_BITS_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_ADDR0_NUM_BITS_MAX                      (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_ADDR0_RSVD_MASK                         (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_ADDR0_RSVD_SHIFT                        (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_ADDR0_RSVD_MAX                          (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_ADDR0_DATA_MASK                         (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_ADDR0_DATA_SHIFT                        (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_ADDR0_DATA_MAX                          (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_ADDR1_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_ADDR1_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_ADDR1_DATA_MAX                          (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_DESC0_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_DESC0_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_DESC0_DATA_MAX                          (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_DESC1_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_DESC1_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_DESC1_DATA_MAX                          (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_DESC3_DATA_MASK                         (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_DESC3_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_DESC3_DATA_MAX                          (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_DESC3_RSVD_MASK                         (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_DESC3_RSVD_SHIFT                        (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_DESC3_RSVD_MAX                          (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_AXI_ADDR0_REGION_SIZE_MASK              (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_AXI_ADDR0_REGION_SIZE_SHIFT             (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_AXI_ADDR0_REGION_SIZE_MAX               (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_AXI_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_AXI_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_AXI_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_AXI_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_AXI_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_AXI_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_AXI_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_AXI_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_7_AXI_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_ADDR0_NUM_BITS_MASK                     (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_ADDR0_NUM_BITS_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_ADDR0_NUM_BITS_MAX                      (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_ADDR0_RSVD_MASK                         (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_ADDR0_RSVD_SHIFT                        (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_ADDR0_RSVD_MAX                          (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_ADDR0_DATA_MASK                         (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_ADDR0_DATA_SHIFT                        (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_ADDR0_DATA_MAX                          (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_ADDR1_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_ADDR1_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_ADDR1_DATA_MAX                          (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_DESC0_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_DESC0_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_DESC0_DATA_MAX                          (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_DESC1_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_DESC1_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_DESC1_DATA_MAX                          (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_DESC3_DATA_MASK                         (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_DESC3_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_DESC3_DATA_MAX                          (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_DESC3_RSVD_MASK                         (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_DESC3_RSVD_SHIFT                        (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_DESC3_RSVD_MAX                          (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_AXI_ADDR0_REGION_SIZE_MASK              (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_AXI_ADDR0_REGION_SIZE_SHIFT             (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_AXI_ADDR0_REGION_SIZE_MAX               (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_AXI_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_AXI_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_AXI_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_AXI_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_AXI_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_AXI_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_AXI_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_AXI_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_8_AXI_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_ADDR0_NUM_BITS_MASK                     (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_ADDR0_NUM_BITS_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_ADDR0_NUM_BITS_MAX                      (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_ADDR0_RSVD_MASK                         (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_ADDR0_RSVD_SHIFT                        (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_ADDR0_RSVD_MAX                          (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_ADDR0_DATA_MASK                         (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_ADDR0_DATA_SHIFT                        (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_ADDR0_DATA_MAX                          (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_ADDR1_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_ADDR1_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_ADDR1_DATA_MAX                          (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_DESC0_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_DESC0_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_DESC0_DATA_MAX                          (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_DESC1_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_DESC1_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_DESC1_DATA_MAX                          (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_DESC3_DATA_MASK                         (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_DESC3_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_DESC3_DATA_MAX                          (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_DESC3_RSVD_MASK                         (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_DESC3_RSVD_SHIFT                        (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_DESC3_RSVD_MAX                          (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_AXI_ADDR0_REGION_SIZE_MASK              (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_AXI_ADDR0_REGION_SIZE_SHIFT             (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_AXI_ADDR0_REGION_SIZE_MAX               (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_AXI_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_AXI_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_AXI_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_AXI_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_AXI_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_AXI_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_AXI_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_AXI_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_9_AXI_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_ADDR0_NUM_BITS_MASK                    (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_ADDR0_NUM_BITS_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_ADDR0_NUM_BITS_MAX                     (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_ADDR0_RSVD_MASK                        (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_ADDR0_RSVD_SHIFT                       (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_ADDR0_RSVD_MAX                         (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_ADDR0_DATA_MASK                        (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_ADDR0_DATA_SHIFT                       (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_ADDR0_DATA_MAX                         (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_ADDR1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_ADDR1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_ADDR1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_DESC0_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_DESC0_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_DESC0_DATA_MAX                         (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_DESC1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_DESC1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_DESC1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_DESC3_DATA_MASK                        (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_DESC3_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_DESC3_DATA_MAX                         (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_DESC3_RSVD_MASK                        (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_DESC3_RSVD_SHIFT                       (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_DESC3_RSVD_MAX                         (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_AXI_ADDR0_REGION_SIZE_MASK             (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_AXI_ADDR0_REGION_SIZE_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_AXI_ADDR0_REGION_SIZE_MAX              (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_AXI_ADDR0_RSVD_MASK                    (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_AXI_ADDR0_RSVD_SHIFT                   (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_AXI_ADDR0_RSVD_MAX                     (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_AXI_ADDR0_DATA_MASK                    (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_AXI_ADDR0_DATA_SHIFT                   (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_AXI_ADDR0_DATA_MAX                     (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_AXI_ADDR1_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_AXI_ADDR1_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_10_AXI_ADDR1_DATA_MAX                     (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_ADDR0_NUM_BITS_MASK                    (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_ADDR0_NUM_BITS_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_ADDR0_NUM_BITS_MAX                     (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_ADDR0_RSVD_MASK                        (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_ADDR0_RSVD_SHIFT                       (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_ADDR0_RSVD_MAX                         (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_ADDR0_DATA_MASK                        (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_ADDR0_DATA_SHIFT                       (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_ADDR0_DATA_MAX                         (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_ADDR1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_ADDR1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_ADDR1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_DESC0_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_DESC0_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_DESC0_DATA_MAX                         (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_DESC1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_DESC1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_DESC1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_DESC3_DATA_MASK                        (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_DESC3_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_DESC3_DATA_MAX                         (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_DESC3_RSVD_MASK                        (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_DESC3_RSVD_SHIFT                       (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_DESC3_RSVD_MAX                         (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_AXI_ADDR0_REGION_SIZE_MASK             (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_AXI_ADDR0_REGION_SIZE_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_AXI_ADDR0_REGION_SIZE_MAX              (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_AXI_ADDR0_RSVD_MASK                    (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_AXI_ADDR0_RSVD_SHIFT                   (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_AXI_ADDR0_RSVD_MAX                     (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_AXI_ADDR0_DATA_MASK                    (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_AXI_ADDR0_DATA_SHIFT                   (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_AXI_ADDR0_DATA_MAX                     (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_AXI_ADDR1_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_AXI_ADDR1_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_11_AXI_ADDR1_DATA_MAX                     (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_ADDR0_NUM_BITS_MASK                    (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_ADDR0_NUM_BITS_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_ADDR0_NUM_BITS_MAX                     (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_ADDR0_RSVD_MASK                        (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_ADDR0_RSVD_SHIFT                       (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_ADDR0_RSVD_MAX                         (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_ADDR0_DATA_MASK                        (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_ADDR0_DATA_SHIFT                       (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_ADDR0_DATA_MAX                         (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_ADDR1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_ADDR1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_ADDR1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_DESC0_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_DESC0_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_DESC0_DATA_MAX                         (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_DESC1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_DESC1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_DESC1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_DESC3_DATA_MASK                        (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_DESC3_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_DESC3_DATA_MAX                         (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_DESC3_RSVD_MASK                        (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_DESC3_RSVD_SHIFT                       (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_DESC3_RSVD_MAX                         (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_AXI_ADDR0_REGION_SIZE_MASK             (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_AXI_ADDR0_REGION_SIZE_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_AXI_ADDR0_REGION_SIZE_MAX              (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_AXI_ADDR0_RSVD_MASK                    (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_AXI_ADDR0_RSVD_SHIFT                   (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_AXI_ADDR0_RSVD_MAX                     (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_AXI_ADDR0_DATA_MASK                    (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_AXI_ADDR0_DATA_SHIFT                   (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_AXI_ADDR0_DATA_MAX                     (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_AXI_ADDR1_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_AXI_ADDR1_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_12_AXI_ADDR1_DATA_MAX                     (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_ADDR0_NUM_BITS_MASK                    (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_ADDR0_NUM_BITS_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_ADDR0_NUM_BITS_MAX                     (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_ADDR0_RSVD_MASK                        (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_ADDR0_RSVD_SHIFT                       (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_ADDR0_RSVD_MAX                         (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_ADDR0_DATA_MASK                        (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_ADDR0_DATA_SHIFT                       (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_ADDR0_DATA_MAX                         (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_ADDR1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_ADDR1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_ADDR1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_DESC0_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_DESC0_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_DESC0_DATA_MAX                         (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_DESC1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_DESC1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_DESC1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_DESC3_DATA_MASK                        (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_DESC3_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_DESC3_DATA_MAX                         (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_DESC3_RSVD_MASK                        (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_DESC3_RSVD_SHIFT                       (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_DESC3_RSVD_MAX                         (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_AXI_ADDR0_REGION_SIZE_MASK             (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_AXI_ADDR0_REGION_SIZE_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_AXI_ADDR0_REGION_SIZE_MAX              (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_AXI_ADDR0_RSVD_MASK                    (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_AXI_ADDR0_RSVD_SHIFT                   (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_AXI_ADDR0_RSVD_MAX                     (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_AXI_ADDR0_DATA_MASK                    (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_AXI_ADDR0_DATA_SHIFT                   (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_AXI_ADDR0_DATA_MAX                     (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_AXI_ADDR1_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_AXI_ADDR1_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_13_AXI_ADDR1_DATA_MAX                     (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_ADDR0_NUM_BITS_MASK                    (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_ADDR0_NUM_BITS_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_ADDR0_NUM_BITS_MAX                     (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_ADDR0_RSVD_MASK                        (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_ADDR0_RSVD_SHIFT                       (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_ADDR0_RSVD_MAX                         (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_ADDR0_DATA_MASK                        (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_ADDR0_DATA_SHIFT                       (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_ADDR0_DATA_MAX                         (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_ADDR1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_ADDR1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_ADDR1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_DESC0_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_DESC0_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_DESC0_DATA_MAX                         (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_DESC1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_DESC1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_DESC1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_DESC3_DATA_MASK                        (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_DESC3_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_DESC3_DATA_MAX                         (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_DESC3_RSVD_MASK                        (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_DESC3_RSVD_SHIFT                       (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_DESC3_RSVD_MAX                         (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_AXI_ADDR0_REGION_SIZE_MASK             (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_AXI_ADDR0_REGION_SIZE_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_AXI_ADDR0_REGION_SIZE_MAX              (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_AXI_ADDR0_RSVD_MASK                    (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_AXI_ADDR0_RSVD_SHIFT                   (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_AXI_ADDR0_RSVD_MAX                     (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_AXI_ADDR0_DATA_MASK                    (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_AXI_ADDR0_DATA_SHIFT                   (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_AXI_ADDR0_DATA_MAX                     (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_AXI_ADDR1_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_AXI_ADDR1_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_14_AXI_ADDR1_DATA_MAX                     (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_ADDR0_NUM_BITS_MASK                    (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_ADDR0_NUM_BITS_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_ADDR0_NUM_BITS_MAX                     (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_ADDR0_RSVD_MASK                        (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_ADDR0_RSVD_SHIFT                       (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_ADDR0_RSVD_MAX                         (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_ADDR0_DATA_MASK                        (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_ADDR0_DATA_SHIFT                       (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_ADDR0_DATA_MAX                         (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_ADDR1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_ADDR1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_ADDR1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_DESC0_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_DESC0_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_DESC0_DATA_MAX                         (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_DESC1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_DESC1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_DESC1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_DESC3_DATA_MASK                        (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_DESC3_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_DESC3_DATA_MAX                         (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_DESC3_RSVD_MASK                        (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_DESC3_RSVD_SHIFT                       (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_DESC3_RSVD_MAX                         (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_AXI_ADDR0_REGION_SIZE_MASK             (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_AXI_ADDR0_REGION_SIZE_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_AXI_ADDR0_REGION_SIZE_MAX              (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_AXI_ADDR0_RSVD_MASK                    (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_AXI_ADDR0_RSVD_SHIFT                   (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_AXI_ADDR0_RSVD_MAX                     (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_AXI_ADDR0_DATA_MASK                    (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_AXI_ADDR0_DATA_SHIFT                   (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_AXI_ADDR0_DATA_MAX                     (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_AXI_ADDR1_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_AXI_ADDR1_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_15_AXI_ADDR1_DATA_MAX                     (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_ADDR0_NUM_BITS_MASK                    (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_ADDR0_NUM_BITS_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_ADDR0_NUM_BITS_MAX                     (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_ADDR0_RSVD_MASK                        (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_ADDR0_RSVD_SHIFT                       (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_ADDR0_RSVD_MAX                         (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_ADDR0_DATA_MASK                        (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_ADDR0_DATA_SHIFT                       (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_ADDR0_DATA_MAX                         (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_ADDR1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_ADDR1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_ADDR1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_DESC0_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_DESC0_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_DESC0_DATA_MAX                         (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_DESC1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_DESC1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_DESC1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_DESC3_DATA_MASK                        (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_DESC3_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_DESC3_DATA_MAX                         (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_DESC3_RSVD_MASK                        (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_DESC3_RSVD_SHIFT                       (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_DESC3_RSVD_MAX                         (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_AXI_ADDR0_REGION_SIZE_MASK             (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_AXI_ADDR0_REGION_SIZE_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_AXI_ADDR0_REGION_SIZE_MAX              (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_AXI_ADDR0_RSVD_MASK                    (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_AXI_ADDR0_RSVD_SHIFT                   (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_AXI_ADDR0_RSVD_MAX                     (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_AXI_ADDR0_DATA_MASK                    (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_AXI_ADDR0_DATA_SHIFT                   (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_AXI_ADDR0_DATA_MAX                     (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_AXI_ADDR1_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_AXI_ADDR1_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_16_AXI_ADDR1_DATA_MAX                     (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_ADDR0_NUM_BITS_MASK                    (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_ADDR0_NUM_BITS_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_ADDR0_NUM_BITS_MAX                     (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_ADDR0_RSVD_MASK                        (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_ADDR0_RSVD_SHIFT                       (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_ADDR0_RSVD_MAX                         (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_ADDR0_DATA_MASK                        (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_ADDR0_DATA_SHIFT                       (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_ADDR0_DATA_MAX                         (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_ADDR1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_ADDR1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_ADDR1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_DESC0_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_DESC0_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_DESC0_DATA_MAX                         (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_DESC1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_DESC1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_DESC1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_DESC3_DATA_MASK                        (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_DESC3_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_DESC3_DATA_MAX                         (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_DESC3_RSVD_MASK                        (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_DESC3_RSVD_SHIFT                       (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_DESC3_RSVD_MAX                         (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_AXI_ADDR0_REGION_SIZE_MASK             (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_AXI_ADDR0_REGION_SIZE_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_AXI_ADDR0_REGION_SIZE_MAX              (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_AXI_ADDR0_RSVD_MASK                    (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_AXI_ADDR0_RSVD_SHIFT                   (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_AXI_ADDR0_RSVD_MAX                     (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_AXI_ADDR0_DATA_MASK                    (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_AXI_ADDR0_DATA_SHIFT                   (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_AXI_ADDR0_DATA_MAX                     (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_AXI_ADDR1_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_AXI_ADDR1_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_17_AXI_ADDR1_DATA_MAX                     (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_ADDR0_NUM_BITS_MASK                    (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_ADDR0_NUM_BITS_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_ADDR0_NUM_BITS_MAX                     (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_ADDR0_RSVD_MASK                        (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_ADDR0_RSVD_SHIFT                       (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_ADDR0_RSVD_MAX                         (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_ADDR0_DATA_MASK                        (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_ADDR0_DATA_SHIFT                       (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_ADDR0_DATA_MAX                         (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_ADDR1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_ADDR1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_ADDR1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_DESC0_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_DESC0_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_DESC0_DATA_MAX                         (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_DESC1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_DESC1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_DESC1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_DESC3_DATA_MASK                        (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_DESC3_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_DESC3_DATA_MAX                         (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_DESC3_RSVD_MASK                        (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_DESC3_RSVD_SHIFT                       (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_DESC3_RSVD_MAX                         (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_AXI_ADDR0_REGION_SIZE_MASK             (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_AXI_ADDR0_REGION_SIZE_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_AXI_ADDR0_REGION_SIZE_MAX              (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_AXI_ADDR0_RSVD_MASK                    (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_AXI_ADDR0_RSVD_SHIFT                   (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_AXI_ADDR0_RSVD_MAX                     (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_AXI_ADDR0_DATA_MASK                    (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_AXI_ADDR0_DATA_SHIFT                   (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_AXI_ADDR0_DATA_MAX                     (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_AXI_ADDR1_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_AXI_ADDR1_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_18_AXI_ADDR1_DATA_MAX                     (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_ADDR0_NUM_BITS_MASK                    (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_ADDR0_NUM_BITS_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_ADDR0_NUM_BITS_MAX                     (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_ADDR0_RSVD_MASK                        (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_ADDR0_RSVD_SHIFT                       (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_ADDR0_RSVD_MAX                         (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_ADDR0_DATA_MASK                        (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_ADDR0_DATA_SHIFT                       (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_ADDR0_DATA_MAX                         (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_ADDR1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_ADDR1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_ADDR1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_DESC0_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_DESC0_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_DESC0_DATA_MAX                         (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_DESC1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_DESC1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_DESC1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_DESC3_DATA_MASK                        (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_DESC3_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_DESC3_DATA_MAX                         (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_DESC3_RSVD_MASK                        (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_DESC3_RSVD_SHIFT                       (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_DESC3_RSVD_MAX                         (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_AXI_ADDR0_REGION_SIZE_MASK             (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_AXI_ADDR0_REGION_SIZE_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_AXI_ADDR0_REGION_SIZE_MAX              (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_AXI_ADDR0_RSVD_MASK                    (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_AXI_ADDR0_RSVD_SHIFT                   (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_AXI_ADDR0_RSVD_MAX                     (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_AXI_ADDR0_DATA_MASK                    (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_AXI_ADDR0_DATA_SHIFT                   (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_AXI_ADDR0_DATA_MAX                     (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_AXI_ADDR1_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_AXI_ADDR1_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_19_AXI_ADDR1_DATA_MAX                     (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_ADDR0_NUM_BITS_MASK                    (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_ADDR0_NUM_BITS_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_ADDR0_NUM_BITS_MAX                     (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_ADDR0_RSVD_MASK                        (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_ADDR0_RSVD_SHIFT                       (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_ADDR0_RSVD_MAX                         (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_ADDR0_DATA_MASK                        (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_ADDR0_DATA_SHIFT                       (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_ADDR0_DATA_MAX                         (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_ADDR1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_ADDR1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_ADDR1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_DESC0_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_DESC0_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_DESC0_DATA_MAX                         (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_DESC1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_DESC1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_DESC1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_DESC3_DATA_MASK                        (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_DESC3_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_DESC3_DATA_MAX                         (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_DESC3_RSVD_MASK                        (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_DESC3_RSVD_SHIFT                       (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_DESC3_RSVD_MAX                         (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_AXI_ADDR0_REGION_SIZE_MASK             (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_AXI_ADDR0_REGION_SIZE_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_AXI_ADDR0_REGION_SIZE_MAX              (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_AXI_ADDR0_RSVD_MASK                    (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_AXI_ADDR0_RSVD_SHIFT                   (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_AXI_ADDR0_RSVD_MAX                     (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_AXI_ADDR0_DATA_MASK                    (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_AXI_ADDR0_DATA_SHIFT                   (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_AXI_ADDR0_DATA_MAX                     (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_AXI_ADDR1_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_AXI_ADDR1_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_20_AXI_ADDR1_DATA_MAX                     (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_ADDR0_NUM_BITS_MASK                    (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_ADDR0_NUM_BITS_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_ADDR0_NUM_BITS_MAX                     (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_ADDR0_RSVD_MASK                        (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_ADDR0_RSVD_SHIFT                       (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_ADDR0_RSVD_MAX                         (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_ADDR0_DATA_MASK                        (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_ADDR0_DATA_SHIFT                       (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_ADDR0_DATA_MAX                         (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_ADDR1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_ADDR1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_ADDR1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_DESC0_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_DESC0_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_DESC0_DATA_MAX                         (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_DESC1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_DESC1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_DESC1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_DESC3_DATA_MASK                        (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_DESC3_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_DESC3_DATA_MAX                         (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_DESC3_RSVD_MASK                        (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_DESC3_RSVD_SHIFT                       (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_DESC3_RSVD_MAX                         (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_AXI_ADDR0_REGION_SIZE_MASK             (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_AXI_ADDR0_REGION_SIZE_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_AXI_ADDR0_REGION_SIZE_MAX              (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_AXI_ADDR0_RSVD_MASK                    (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_AXI_ADDR0_RSVD_SHIFT                   (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_AXI_ADDR0_RSVD_MAX                     (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_AXI_ADDR0_DATA_MASK                    (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_AXI_ADDR0_DATA_SHIFT                   (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_AXI_ADDR0_DATA_MAX                     (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_AXI_ADDR1_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_AXI_ADDR1_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_21_AXI_ADDR1_DATA_MAX                     (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_ADDR0_NUM_BITS_MASK                    (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_ADDR0_NUM_BITS_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_ADDR0_NUM_BITS_MAX                     (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_ADDR0_RSVD_MASK                        (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_ADDR0_RSVD_SHIFT                       (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_ADDR0_RSVD_MAX                         (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_ADDR0_DATA_MASK                        (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_ADDR0_DATA_SHIFT                       (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_ADDR0_DATA_MAX                         (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_ADDR1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_ADDR1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_ADDR1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_DESC0_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_DESC0_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_DESC0_DATA_MAX                         (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_DESC1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_DESC1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_DESC1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_DESC3_DATA_MASK                        (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_DESC3_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_DESC3_DATA_MAX                         (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_DESC3_RSVD_MASK                        (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_DESC3_RSVD_SHIFT                       (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_DESC3_RSVD_MAX                         (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_AXI_ADDR0_REGION_SIZE_MASK             (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_AXI_ADDR0_REGION_SIZE_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_AXI_ADDR0_REGION_SIZE_MAX              (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_AXI_ADDR0_RSVD_MASK                    (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_AXI_ADDR0_RSVD_SHIFT                   (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_AXI_ADDR0_RSVD_MAX                     (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_AXI_ADDR0_DATA_MASK                    (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_AXI_ADDR0_DATA_SHIFT                   (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_AXI_ADDR0_DATA_MAX                     (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_AXI_ADDR1_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_AXI_ADDR1_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_22_AXI_ADDR1_DATA_MAX                     (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_ADDR0_NUM_BITS_MASK                    (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_ADDR0_NUM_BITS_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_ADDR0_NUM_BITS_MAX                     (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_ADDR0_RSVD_MASK                        (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_ADDR0_RSVD_SHIFT                       (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_ADDR0_RSVD_MAX                         (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_ADDR0_DATA_MASK                        (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_ADDR0_DATA_SHIFT                       (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_ADDR0_DATA_MAX                         (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_ADDR1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_ADDR1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_ADDR1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_DESC0_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_DESC0_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_DESC0_DATA_MAX                         (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_DESC1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_DESC1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_DESC1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_DESC3_DATA_MASK                        (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_DESC3_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_DESC3_DATA_MAX                         (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_DESC3_RSVD_MASK                        (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_DESC3_RSVD_SHIFT                       (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_DESC3_RSVD_MAX                         (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_AXI_ADDR0_REGION_SIZE_MASK             (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_AXI_ADDR0_REGION_SIZE_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_AXI_ADDR0_REGION_SIZE_MAX              (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_AXI_ADDR0_RSVD_MASK                    (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_AXI_ADDR0_RSVD_SHIFT                   (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_AXI_ADDR0_RSVD_MAX                     (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_AXI_ADDR0_DATA_MASK                    (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_AXI_ADDR0_DATA_SHIFT                   (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_AXI_ADDR0_DATA_MAX                     (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_AXI_ADDR1_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_AXI_ADDR1_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_23_AXI_ADDR1_DATA_MAX                     (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_ADDR0_NUM_BITS_MASK                    (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_ADDR0_NUM_BITS_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_ADDR0_NUM_BITS_MAX                     (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_ADDR0_RSVD_MASK                        (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_ADDR0_RSVD_SHIFT                       (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_ADDR0_RSVD_MAX                         (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_ADDR0_DATA_MASK                        (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_ADDR0_DATA_SHIFT                       (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_ADDR0_DATA_MAX                         (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_ADDR1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_ADDR1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_ADDR1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_DESC0_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_DESC0_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_DESC0_DATA_MAX                         (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_DESC1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_DESC1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_DESC1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_DESC3_DATA_MASK                        (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_DESC3_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_DESC3_DATA_MAX                         (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_DESC3_RSVD_MASK                        (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_DESC3_RSVD_SHIFT                       (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_DESC3_RSVD_MAX                         (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_AXI_ADDR0_REGION_SIZE_MASK             (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_AXI_ADDR0_REGION_SIZE_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_AXI_ADDR0_REGION_SIZE_MAX              (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_AXI_ADDR0_RSVD_MASK                    (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_AXI_ADDR0_RSVD_SHIFT                   (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_AXI_ADDR0_RSVD_MAX                     (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_AXI_ADDR0_DATA_MASK                    (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_AXI_ADDR0_DATA_SHIFT                   (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_AXI_ADDR0_DATA_MAX                     (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_AXI_ADDR1_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_AXI_ADDR1_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_24_AXI_ADDR1_DATA_MAX                     (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_ADDR0_NUM_BITS_MASK                    (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_ADDR0_NUM_BITS_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_ADDR0_NUM_BITS_MAX                     (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_ADDR0_RSVD_MASK                        (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_ADDR0_RSVD_SHIFT                       (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_ADDR0_RSVD_MAX                         (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_ADDR0_DATA_MASK                        (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_ADDR0_DATA_SHIFT                       (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_ADDR0_DATA_MAX                         (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_ADDR1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_ADDR1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_ADDR1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_DESC0_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_DESC0_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_DESC0_DATA_MAX                         (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_DESC1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_DESC1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_DESC1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_DESC3_DATA_MASK                        (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_DESC3_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_DESC3_DATA_MAX                         (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_DESC3_RSVD_MASK                        (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_DESC3_RSVD_SHIFT                       (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_DESC3_RSVD_MAX                         (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_AXI_ADDR0_REGION_SIZE_MASK             (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_AXI_ADDR0_REGION_SIZE_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_AXI_ADDR0_REGION_SIZE_MAX              (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_AXI_ADDR0_RSVD_MASK                    (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_AXI_ADDR0_RSVD_SHIFT                   (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_AXI_ADDR0_RSVD_MAX                     (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_AXI_ADDR0_DATA_MASK                    (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_AXI_ADDR0_DATA_SHIFT                   (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_AXI_ADDR0_DATA_MAX                     (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_AXI_ADDR1_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_AXI_ADDR1_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_25_AXI_ADDR1_DATA_MAX                     (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_ADDR0_NUM_BITS_MASK                    (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_ADDR0_NUM_BITS_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_ADDR0_NUM_BITS_MAX                     (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_ADDR0_RSVD_MASK                        (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_ADDR0_RSVD_SHIFT                       (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_ADDR0_RSVD_MAX                         (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_ADDR0_DATA_MASK                        (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_ADDR0_DATA_SHIFT                       (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_ADDR0_DATA_MAX                         (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_ADDR1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_ADDR1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_ADDR1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_DESC0_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_DESC0_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_DESC0_DATA_MAX                         (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_DESC1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_DESC1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_DESC1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_DESC3_DATA_MASK                        (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_DESC3_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_DESC3_DATA_MAX                         (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_DESC3_RSVD_MASK                        (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_DESC3_RSVD_SHIFT                       (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_DESC3_RSVD_MAX                         (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_AXI_ADDR0_REGION_SIZE_MASK             (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_AXI_ADDR0_REGION_SIZE_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_AXI_ADDR0_REGION_SIZE_MAX              (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_AXI_ADDR0_RSVD_MASK                    (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_AXI_ADDR0_RSVD_SHIFT                   (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_AXI_ADDR0_RSVD_MAX                     (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_AXI_ADDR0_DATA_MASK                    (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_AXI_ADDR0_DATA_SHIFT                   (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_AXI_ADDR0_DATA_MAX                     (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_AXI_ADDR1_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_AXI_ADDR1_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_26_AXI_ADDR1_DATA_MAX                     (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_ADDR0_NUM_BITS_MASK                    (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_ADDR0_NUM_BITS_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_ADDR0_NUM_BITS_MAX                     (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_ADDR0_RSVD_MASK                        (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_ADDR0_RSVD_SHIFT                       (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_ADDR0_RSVD_MAX                         (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_ADDR0_DATA_MASK                        (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_ADDR0_DATA_SHIFT                       (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_ADDR0_DATA_MAX                         (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_ADDR1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_ADDR1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_ADDR1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_DESC0_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_DESC0_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_DESC0_DATA_MAX                         (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_DESC1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_DESC1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_DESC1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_DESC3_DATA_MASK                        (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_DESC3_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_DESC3_DATA_MAX                         (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_DESC3_RSVD_MASK                        (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_DESC3_RSVD_SHIFT                       (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_DESC3_RSVD_MAX                         (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_AXI_ADDR0_REGION_SIZE_MASK             (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_AXI_ADDR0_REGION_SIZE_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_AXI_ADDR0_REGION_SIZE_MAX              (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_AXI_ADDR0_RSVD_MASK                    (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_AXI_ADDR0_RSVD_SHIFT                   (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_AXI_ADDR0_RSVD_MAX                     (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_AXI_ADDR0_DATA_MASK                    (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_AXI_ADDR0_DATA_SHIFT                   (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_AXI_ADDR0_DATA_MAX                     (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_AXI_ADDR1_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_AXI_ADDR1_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_27_AXI_ADDR1_DATA_MAX                     (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_ADDR0_NUM_BITS_MASK                    (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_ADDR0_NUM_BITS_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_ADDR0_NUM_BITS_MAX                     (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_ADDR0_RSVD_MASK                        (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_ADDR0_RSVD_SHIFT                       (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_ADDR0_RSVD_MAX                         (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_ADDR0_DATA_MASK                        (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_ADDR0_DATA_SHIFT                       (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_ADDR0_DATA_MAX                         (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_ADDR1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_ADDR1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_ADDR1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_DESC0_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_DESC0_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_DESC0_DATA_MAX                         (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_DESC1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_DESC1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_DESC1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_DESC3_DATA_MASK                        (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_DESC3_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_DESC3_DATA_MAX                         (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_DESC3_RSVD_MASK                        (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_DESC3_RSVD_SHIFT                       (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_DESC3_RSVD_MAX                         (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_AXI_ADDR0_REGION_SIZE_MASK             (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_AXI_ADDR0_REGION_SIZE_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_AXI_ADDR0_REGION_SIZE_MAX              (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_AXI_ADDR0_RSVD_MASK                    (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_AXI_ADDR0_RSVD_SHIFT                   (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_AXI_ADDR0_RSVD_MAX                     (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_AXI_ADDR0_DATA_MASK                    (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_AXI_ADDR0_DATA_SHIFT                   (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_AXI_ADDR0_DATA_MAX                     (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_AXI_ADDR1_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_AXI_ADDR1_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_28_AXI_ADDR1_DATA_MAX                     (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_ADDR0_NUM_BITS_MASK                    (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_ADDR0_NUM_BITS_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_ADDR0_NUM_BITS_MAX                     (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_ADDR0_RSVD_MASK                        (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_ADDR0_RSVD_SHIFT                       (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_ADDR0_RSVD_MAX                         (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_ADDR0_DATA_MASK                        (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_ADDR0_DATA_SHIFT                       (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_ADDR0_DATA_MAX                         (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_ADDR1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_ADDR1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_ADDR1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_DESC0_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_DESC0_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_DESC0_DATA_MAX                         (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_DESC1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_DESC1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_DESC1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_DESC3_DATA_MASK                        (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_DESC3_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_DESC3_DATA_MAX                         (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_DESC3_RSVD_MASK                        (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_DESC3_RSVD_SHIFT                       (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_DESC3_RSVD_MAX                         (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_AXI_ADDR0_REGION_SIZE_MASK             (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_AXI_ADDR0_REGION_SIZE_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_AXI_ADDR0_REGION_SIZE_MAX              (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_AXI_ADDR0_RSVD_MASK                    (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_AXI_ADDR0_RSVD_SHIFT                   (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_AXI_ADDR0_RSVD_MAX                     (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_AXI_ADDR0_DATA_MASK                    (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_AXI_ADDR0_DATA_SHIFT                   (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_AXI_ADDR0_DATA_MAX                     (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_AXI_ADDR1_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_AXI_ADDR1_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_29_AXI_ADDR1_DATA_MAX                     (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_ADDR0_NUM_BITS_MASK                    (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_ADDR0_NUM_BITS_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_ADDR0_NUM_BITS_MAX                     (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_ADDR0_RSVD_MASK                        (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_ADDR0_RSVD_SHIFT                       (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_ADDR0_RSVD_MAX                         (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_ADDR0_DATA_MASK                        (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_ADDR0_DATA_SHIFT                       (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_ADDR0_DATA_MAX                         (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_ADDR1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_ADDR1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_ADDR1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_DESC0_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_DESC0_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_DESC0_DATA_MAX                         (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_DESC1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_DESC1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_DESC1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_DESC3_DATA_MASK                        (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_DESC3_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_DESC3_DATA_MAX                         (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_DESC3_RSVD_MASK                        (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_DESC3_RSVD_SHIFT                       (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_DESC3_RSVD_MAX                         (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_AXI_ADDR0_REGION_SIZE_MASK             (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_AXI_ADDR0_REGION_SIZE_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_AXI_ADDR0_REGION_SIZE_MAX              (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_AXI_ADDR0_RSVD_MASK                    (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_AXI_ADDR0_RSVD_SHIFT                   (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_AXI_ADDR0_RSVD_MAX                     (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_AXI_ADDR0_DATA_MASK                    (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_AXI_ADDR0_DATA_SHIFT                   (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_AXI_ADDR0_DATA_MAX                     (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_AXI_ADDR1_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_AXI_ADDR1_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_30_AXI_ADDR1_DATA_MAX                     (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_ADDR0_NUM_BITS_MASK                    (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_ADDR0_NUM_BITS_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_ADDR0_NUM_BITS_MAX                     (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_ADDR0_RSVD_MASK                        (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_ADDR0_RSVD_SHIFT                       (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_ADDR0_RSVD_MAX                         (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_ADDR0_DATA_MASK                        (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_ADDR0_DATA_SHIFT                       (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_ADDR0_DATA_MAX                         (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_ADDR1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_ADDR1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_ADDR1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_DESC0_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_DESC0_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_DESC0_DATA_MAX                         (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_DESC1_DATA_MASK                        (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_DESC1_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_DESC1_DATA_MAX                         (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_DESC3_DATA_MASK                        (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_DESC3_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_DESC3_DATA_MAX                         (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_DESC3_RSVD_MASK                        (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_DESC3_RSVD_SHIFT                       (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_DESC3_RSVD_MAX                         (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_AXI_ADDR0_REGION_SIZE_MASK             (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_AXI_ADDR0_REGION_SIZE_SHIFT            (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_AXI_ADDR0_REGION_SIZE_MAX              (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_AXI_ADDR0_RSVD_MASK                    (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_AXI_ADDR0_RSVD_SHIFT                   (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_AXI_ADDR0_RSVD_MAX                     (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_AXI_ADDR0_DATA_MASK                    (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_AXI_ADDR0_DATA_SHIFT                   (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_AXI_ADDR0_DATA_MAX                     (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_AXI_ADDR1_DATA_MASK                    (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_AXI_ADDR1_DATA_SHIFT                   (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_OB_31_AXI_ADDR1_DATA_MAX                     (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_ADDR0_NUM_BITS_MASK                  (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_ADDR0_NUM_BITS_SHIFT                 (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_ADDR0_NUM_BITS_MAX                   (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_ADDR0_RSVD_MASK                      (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_ADDR0_RSVD_SHIFT                     (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_ADDR0_RSVD_MAX                       (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_ADDR0_DATA_MASK                      (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_ADDR0_DATA_SHIFT                     (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_ADDR0_DATA_MAX                       (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_ADDR1_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_ADDR1_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_ADDR1_DATA_MAX                       (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_DESC0_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_DESC0_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_DESC0_DATA_MAX                       (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_DESC1_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_DESC1_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_DESC1_DATA_MAX                       (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_DESC3_DATA_MASK                      (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_DESC3_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_DESC3_DATA_MAX                       (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_DESC3_RSVD_MASK                      (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_DESC3_RSVD_SHIFT                     (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_DESC3_RSVD_MAX                       (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_AXI_ADDR0_REGION_SIZE_MASK           (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_AXI_ADDR0_REGION_SIZE_SHIFT          (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_AXI_ADDR0_REGION_SIZE_MAX            (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_AXI_ADDR0_RSVD_MASK                  (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_AXI_ADDR0_RSVD_SHIFT                 (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_AXI_ADDR0_RSVD_MAX                   (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_AXI_ADDR0_DATA_MASK                  (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_AXI_ADDR0_DATA_SHIFT                 (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_AXI_ADDR0_DATA_MAX                   (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_AXI_ADDR1_DATA_MASK                  (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_AXI_ADDR1_DATA_SHIFT                 (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_0_AXI_ADDR1_DATA_MAX                   (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_ADDR0_NUM_BITS_MASK                  (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_ADDR0_NUM_BITS_SHIFT                 (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_ADDR0_NUM_BITS_MAX                   (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_ADDR0_RSVD_MASK                      (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_ADDR0_RSVD_SHIFT                     (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_ADDR0_RSVD_MAX                       (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_ADDR0_DATA_MASK                      (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_ADDR0_DATA_SHIFT                     (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_ADDR0_DATA_MAX                       (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_ADDR1_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_ADDR1_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_ADDR1_DATA_MAX                       (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_DESC0_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_DESC0_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_DESC0_DATA_MAX                       (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_DESC1_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_DESC1_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_DESC1_DATA_MAX                       (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_DESC3_DATA_MASK                      (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_DESC3_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_DESC3_DATA_MAX                       (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_DESC3_RSVD_MASK                      (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_DESC3_RSVD_SHIFT                     (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_DESC3_RSVD_MAX                       (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_AXI_ADDR0_REGION_SIZE_MASK           (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_AXI_ADDR0_REGION_SIZE_SHIFT          (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_AXI_ADDR0_REGION_SIZE_MAX            (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_AXI_ADDR0_RSVD_MASK                  (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_AXI_ADDR0_RSVD_SHIFT                 (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_AXI_ADDR0_RSVD_MAX                   (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_AXI_ADDR0_DATA_MASK                  (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_AXI_ADDR0_DATA_SHIFT                 (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_AXI_ADDR0_DATA_MAX                   (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_AXI_ADDR1_DATA_MASK                  (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_AXI_ADDR1_DATA_SHIFT                 (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_1_AXI_ADDR1_DATA_MAX                   (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_ADDR0_NUM_BITS_MASK                  (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_ADDR0_NUM_BITS_SHIFT                 (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_ADDR0_NUM_BITS_MAX                   (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_ADDR0_RSVD_MASK                      (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_ADDR0_RSVD_SHIFT                     (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_ADDR0_RSVD_MAX                       (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_ADDR0_DATA_MASK                      (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_ADDR0_DATA_SHIFT                     (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_ADDR0_DATA_MAX                       (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_ADDR1_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_ADDR1_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_ADDR1_DATA_MAX                       (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_DESC0_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_DESC0_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_DESC0_DATA_MAX                       (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_DESC1_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_DESC1_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_DESC1_DATA_MAX                       (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_DESC3_DATA_MASK                      (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_DESC3_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_DESC3_DATA_MAX                       (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_DESC3_RSVD_MASK                      (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_DESC3_RSVD_SHIFT                     (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_DESC3_RSVD_MAX                       (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_AXI_ADDR0_REGION_SIZE_MASK           (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_AXI_ADDR0_REGION_SIZE_SHIFT          (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_AXI_ADDR0_REGION_SIZE_MAX            (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_AXI_ADDR0_RSVD_MASK                  (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_AXI_ADDR0_RSVD_SHIFT                 (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_AXI_ADDR0_RSVD_MAX                   (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_AXI_ADDR0_DATA_MASK                  (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_AXI_ADDR0_DATA_SHIFT                 (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_AXI_ADDR0_DATA_MAX                   (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_AXI_ADDR1_DATA_MASK                  (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_AXI_ADDR1_DATA_SHIFT                 (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_2_AXI_ADDR1_DATA_MAX                   (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_ADDR0_NUM_BITS_MASK                  (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_ADDR0_NUM_BITS_SHIFT                 (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_ADDR0_NUM_BITS_MAX                   (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_ADDR0_RSVD_MASK                      (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_ADDR0_RSVD_SHIFT                     (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_ADDR0_RSVD_MAX                       (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_ADDR0_DATA_MASK                      (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_ADDR0_DATA_SHIFT                     (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_ADDR0_DATA_MAX                       (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_ADDR1_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_ADDR1_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_ADDR1_DATA_MAX                       (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_DESC0_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_DESC0_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_DESC0_DATA_MAX                       (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_DESC1_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_DESC1_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_DESC1_DATA_MAX                       (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_DESC3_DATA_MASK                      (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_DESC3_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_DESC3_DATA_MAX                       (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_DESC3_RSVD_MASK                      (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_DESC3_RSVD_SHIFT                     (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_DESC3_RSVD_MAX                       (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_AXI_ADDR0_REGION_SIZE_MASK           (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_AXI_ADDR0_REGION_SIZE_SHIFT          (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_AXI_ADDR0_REGION_SIZE_MAX            (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_AXI_ADDR0_RSVD_MASK                  (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_AXI_ADDR0_RSVD_SHIFT                 (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_AXI_ADDR0_RSVD_MAX                   (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_AXI_ADDR0_DATA_MASK                  (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_AXI_ADDR0_DATA_SHIFT                 (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_AXI_ADDR0_DATA_MAX                   (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_AXI_ADDR1_DATA_MASK                  (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_AXI_ADDR1_DATA_SHIFT                 (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_3_AXI_ADDR1_DATA_MAX                   (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_ADDR0_NUM_BITS_MASK                  (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_ADDR0_NUM_BITS_SHIFT                 (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_ADDR0_NUM_BITS_MAX                   (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_ADDR0_RSVD_MASK                      (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_ADDR0_RSVD_SHIFT                     (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_ADDR0_RSVD_MAX                       (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_ADDR0_DATA_MASK                      (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_ADDR0_DATA_SHIFT                     (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_ADDR0_DATA_MAX                       (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_ADDR1_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_ADDR1_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_ADDR1_DATA_MAX                       (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_DESC0_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_DESC0_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_DESC0_DATA_MAX                       (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_DESC1_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_DESC1_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_DESC1_DATA_MAX                       (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_DESC3_DATA_MASK                      (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_DESC3_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_DESC3_DATA_MAX                       (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_DESC3_RSVD_MASK                      (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_DESC3_RSVD_SHIFT                     (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_DESC3_RSVD_MAX                       (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_AXI_ADDR0_REGION_SIZE_MASK           (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_AXI_ADDR0_REGION_SIZE_SHIFT          (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_AXI_ADDR0_REGION_SIZE_MAX            (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_AXI_ADDR0_RSVD_MASK                  (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_AXI_ADDR0_RSVD_SHIFT                 (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_AXI_ADDR0_RSVD_MAX                   (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_AXI_ADDR0_DATA_MASK                  (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_AXI_ADDR0_DATA_SHIFT                 (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_AXI_ADDR0_DATA_MAX                   (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_AXI_ADDR1_DATA_MASK                  (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_AXI_ADDR1_DATA_SHIFT                 (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_4_AXI_ADDR1_DATA_MAX                   (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_ADDR0_NUM_BITS_MASK                  (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_ADDR0_NUM_BITS_SHIFT                 (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_ADDR0_NUM_BITS_MAX                   (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_ADDR0_RSVD_MASK                      (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_ADDR0_RSVD_SHIFT                     (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_ADDR0_RSVD_MAX                       (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_ADDR0_DATA_MASK                      (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_ADDR0_DATA_SHIFT                     (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_ADDR0_DATA_MAX                       (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_ADDR1_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_ADDR1_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_ADDR1_DATA_MAX                       (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_DESC0_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_DESC0_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_DESC0_DATA_MAX                       (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_DESC1_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_DESC1_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_DESC1_DATA_MAX                       (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_DESC3_DATA_MASK                      (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_DESC3_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_DESC3_DATA_MAX                       (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_DESC3_RSVD_MASK                      (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_DESC3_RSVD_SHIFT                     (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_DESC3_RSVD_MAX                       (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_AXI_ADDR0_REGION_SIZE_MASK           (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_AXI_ADDR0_REGION_SIZE_SHIFT          (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_AXI_ADDR0_REGION_SIZE_MAX            (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_AXI_ADDR0_RSVD_MASK                  (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_AXI_ADDR0_RSVD_SHIFT                 (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_AXI_ADDR0_RSVD_MAX                   (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_AXI_ADDR0_DATA_MASK                  (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_AXI_ADDR0_DATA_SHIFT                 (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_AXI_ADDR0_DATA_MAX                   (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_AXI_ADDR1_DATA_MASK                  (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_AXI_ADDR1_DATA_SHIFT                 (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_5_AXI_ADDR1_DATA_MAX                   (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_ADDR0_NUM_BITS_MASK                  (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_ADDR0_NUM_BITS_SHIFT                 (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_ADDR0_NUM_BITS_MAX                   (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_ADDR0_RSVD_MASK                      (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_ADDR0_RSVD_SHIFT                     (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_ADDR0_RSVD_MAX                       (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_ADDR0_DATA_MASK                      (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_ADDR0_DATA_SHIFT                     (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_ADDR0_DATA_MAX                       (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_ADDR1_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_ADDR1_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_ADDR1_DATA_MAX                       (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_DESC0_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_DESC0_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_DESC0_DATA_MAX                       (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_DESC1_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_DESC1_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_DESC1_DATA_MAX                       (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_DESC3_DATA_MASK                      (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_DESC3_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_DESC3_DATA_MAX                       (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_DESC3_RSVD_MASK                      (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_DESC3_RSVD_SHIFT                     (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_DESC3_RSVD_MAX                       (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_AXI_ADDR0_REGION_SIZE_MASK           (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_AXI_ADDR0_REGION_SIZE_SHIFT          (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_AXI_ADDR0_REGION_SIZE_MAX            (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_AXI_ADDR0_RSVD_MASK                  (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_AXI_ADDR0_RSVD_SHIFT                 (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_AXI_ADDR0_RSVD_MAX                   (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_AXI_ADDR0_DATA_MASK                  (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_AXI_ADDR0_DATA_SHIFT                 (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_AXI_ADDR0_DATA_MAX                   (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_AXI_ADDR1_DATA_MASK                  (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_AXI_ADDR1_DATA_SHIFT                 (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_6_AXI_ADDR1_DATA_MAX                   (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_ADDR0_NUM_BITS_MASK                  (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_ADDR0_NUM_BITS_SHIFT                 (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_ADDR0_NUM_BITS_MAX                   (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_ADDR0_RSVD_MASK                      (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_ADDR0_RSVD_SHIFT                     (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_ADDR0_RSVD_MAX                       (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_ADDR0_DATA_MASK                      (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_ADDR0_DATA_SHIFT                     (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_ADDR0_DATA_MAX                       (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_ADDR1_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_ADDR1_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_ADDR1_DATA_MAX                       (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_DESC0_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_DESC0_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_DESC0_DATA_MAX                       (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_DESC1_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_DESC1_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_DESC1_DATA_MAX                       (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_DESC3_DATA_MASK                      (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_DESC3_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_DESC3_DATA_MAX                       (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_DESC3_RSVD_MASK                      (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_DESC3_RSVD_SHIFT                     (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_DESC3_RSVD_MAX                       (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_AXI_ADDR0_REGION_SIZE_MASK           (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_AXI_ADDR0_REGION_SIZE_SHIFT          (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_AXI_ADDR0_REGION_SIZE_MAX            (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_AXI_ADDR0_RSVD_MASK                  (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_AXI_ADDR0_RSVD_SHIFT                 (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_AXI_ADDR0_RSVD_MAX                   (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_AXI_ADDR0_DATA_MASK                  (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_AXI_ADDR0_DATA_SHIFT                 (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_AXI_ADDR0_DATA_MAX                   (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_AXI_ADDR1_DATA_MASK                  (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_AXI_ADDR1_DATA_SHIFT                 (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_7_AXI_ADDR1_DATA_MAX                   (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_ADDR0_NUM_BITS_MASK                  (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_ADDR0_NUM_BITS_SHIFT                 (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_ADDR0_NUM_BITS_MAX                   (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_ADDR0_RSVD_MASK                      (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_ADDR0_RSVD_SHIFT                     (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_ADDR0_RSVD_MAX                       (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_ADDR0_DATA_MASK                      (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_ADDR0_DATA_SHIFT                     (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_ADDR0_DATA_MAX                       (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_ADDR1_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_ADDR1_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_ADDR1_DATA_MAX                       (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_DESC0_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_DESC0_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_DESC0_DATA_MAX                       (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_DESC1_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_DESC1_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_DESC1_DATA_MAX                       (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_DESC3_DATA_MASK                      (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_DESC3_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_DESC3_DATA_MAX                       (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_DESC3_RSVD_MASK                      (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_DESC3_RSVD_SHIFT                     (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_DESC3_RSVD_MAX                       (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_AXI_ADDR0_REGION_SIZE_MASK           (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_AXI_ADDR0_REGION_SIZE_SHIFT          (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_AXI_ADDR0_REGION_SIZE_MAX            (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_AXI_ADDR0_RSVD_MASK                  (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_AXI_ADDR0_RSVD_SHIFT                 (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_AXI_ADDR0_RSVD_MAX                   (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_AXI_ADDR0_DATA_MASK                  (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_AXI_ADDR0_DATA_SHIFT                 (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_AXI_ADDR0_DATA_MAX                   (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_AXI_ADDR1_DATA_MASK                  (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_AXI_ADDR1_DATA_SHIFT                 (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_8_AXI_ADDR1_DATA_MAX                   (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_ADDR0_NUM_BITS_MASK                  (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_ADDR0_NUM_BITS_SHIFT                 (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_ADDR0_NUM_BITS_MAX                   (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_ADDR0_RSVD_MASK                      (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_ADDR0_RSVD_SHIFT                     (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_ADDR0_RSVD_MAX                       (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_ADDR0_DATA_MASK                      (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_ADDR0_DATA_SHIFT                     (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_ADDR0_DATA_MAX                       (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_ADDR1_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_ADDR1_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_ADDR1_DATA_MAX                       (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_DESC0_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_DESC0_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_DESC0_DATA_MAX                       (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_DESC1_DATA_MASK                      (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_DESC1_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_DESC1_DATA_MAX                       (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_DESC3_DATA_MASK                      (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_DESC3_DATA_SHIFT                     (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_DESC3_DATA_MAX                       (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_DESC3_RSVD_MASK                      (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_DESC3_RSVD_SHIFT                     (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_DESC3_RSVD_MAX                       (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_AXI_ADDR0_REGION_SIZE_MASK           (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_AXI_ADDR0_REGION_SIZE_SHIFT          (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_AXI_ADDR0_REGION_SIZE_MAX            (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_AXI_ADDR0_RSVD_MASK                  (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_AXI_ADDR0_RSVD_SHIFT                 (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_AXI_ADDR0_RSVD_MAX                   (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_AXI_ADDR0_DATA_MASK                  (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_AXI_ADDR0_DATA_SHIFT                 (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_AXI_ADDR0_DATA_MAX                   (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_AXI_ADDR1_DATA_MASK                  (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_AXI_ADDR1_DATA_SHIFT                 (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_9_AXI_ADDR1_DATA_MAX                   (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_ADDR0_NUM_BITS_MASK                 (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_ADDR0_NUM_BITS_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_ADDR0_NUM_BITS_MAX                  (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_DESC0_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_DESC0_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_DESC0_DATA_MAX                      (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_DESC1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_DESC1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_DESC1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_DESC3_DATA_MASK                     (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_DESC3_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_DESC3_DATA_MAX                      (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_DESC3_RSVD_MASK                     (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_DESC3_RSVD_SHIFT                    (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_DESC3_RSVD_MAX                      (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_AXI_ADDR0_REGION_SIZE_MASK          (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_AXI_ADDR0_REGION_SIZE_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_AXI_ADDR0_REGION_SIZE_MAX           (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_AXI_ADDR0_RSVD_MASK                 (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_AXI_ADDR0_RSVD_SHIFT                (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_AXI_ADDR0_RSVD_MAX                  (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_AXI_ADDR0_DATA_MASK                 (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_AXI_ADDR0_DATA_SHIFT                (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_AXI_ADDR0_DATA_MAX                  (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_AXI_ADDR1_DATA_MASK                 (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_AXI_ADDR1_DATA_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_10_AXI_ADDR1_DATA_MAX                  (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_ADDR0_NUM_BITS_MASK                 (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_ADDR0_NUM_BITS_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_ADDR0_NUM_BITS_MAX                  (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_DESC0_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_DESC0_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_DESC0_DATA_MAX                      (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_DESC1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_DESC1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_DESC1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_DESC3_DATA_MASK                     (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_DESC3_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_DESC3_DATA_MAX                      (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_DESC3_RSVD_MASK                     (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_DESC3_RSVD_SHIFT                    (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_DESC3_RSVD_MAX                      (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_AXI_ADDR0_REGION_SIZE_MASK          (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_AXI_ADDR0_REGION_SIZE_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_AXI_ADDR0_REGION_SIZE_MAX           (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_AXI_ADDR0_RSVD_MASK                 (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_AXI_ADDR0_RSVD_SHIFT                (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_AXI_ADDR0_RSVD_MAX                  (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_AXI_ADDR0_DATA_MASK                 (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_AXI_ADDR0_DATA_SHIFT                (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_AXI_ADDR0_DATA_MAX                  (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_AXI_ADDR1_DATA_MASK                 (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_AXI_ADDR1_DATA_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_11_AXI_ADDR1_DATA_MAX                  (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_ADDR0_NUM_BITS_MASK                 (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_ADDR0_NUM_BITS_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_ADDR0_NUM_BITS_MAX                  (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_DESC0_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_DESC0_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_DESC0_DATA_MAX                      (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_DESC1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_DESC1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_DESC1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_DESC3_DATA_MASK                     (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_DESC3_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_DESC3_DATA_MAX                      (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_DESC3_RSVD_MASK                     (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_DESC3_RSVD_SHIFT                    (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_DESC3_RSVD_MAX                      (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_AXI_ADDR0_REGION_SIZE_MASK          (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_AXI_ADDR0_REGION_SIZE_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_AXI_ADDR0_REGION_SIZE_MAX           (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_AXI_ADDR0_RSVD_MASK                 (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_AXI_ADDR0_RSVD_SHIFT                (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_AXI_ADDR0_RSVD_MAX                  (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_AXI_ADDR0_DATA_MASK                 (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_AXI_ADDR0_DATA_SHIFT                (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_AXI_ADDR0_DATA_MAX                  (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_AXI_ADDR1_DATA_MASK                 (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_AXI_ADDR1_DATA_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_12_AXI_ADDR1_DATA_MAX                  (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_ADDR0_NUM_BITS_MASK                 (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_ADDR0_NUM_BITS_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_ADDR0_NUM_BITS_MAX                  (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_DESC0_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_DESC0_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_DESC0_DATA_MAX                      (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_DESC1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_DESC1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_DESC1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_DESC3_DATA_MASK                     (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_DESC3_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_DESC3_DATA_MAX                      (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_DESC3_RSVD_MASK                     (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_DESC3_RSVD_SHIFT                    (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_DESC3_RSVD_MAX                      (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_AXI_ADDR0_REGION_SIZE_MASK          (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_AXI_ADDR0_REGION_SIZE_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_AXI_ADDR0_REGION_SIZE_MAX           (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_AXI_ADDR0_RSVD_MASK                 (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_AXI_ADDR0_RSVD_SHIFT                (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_AXI_ADDR0_RSVD_MAX                  (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_AXI_ADDR0_DATA_MASK                 (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_AXI_ADDR0_DATA_SHIFT                (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_AXI_ADDR0_DATA_MAX                  (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_AXI_ADDR1_DATA_MASK                 (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_AXI_ADDR1_DATA_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_13_AXI_ADDR1_DATA_MAX                  (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_ADDR0_NUM_BITS_MASK                 (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_ADDR0_NUM_BITS_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_ADDR0_NUM_BITS_MAX                  (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_DESC0_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_DESC0_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_DESC0_DATA_MAX                      (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_DESC1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_DESC1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_DESC1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_DESC3_DATA_MASK                     (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_DESC3_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_DESC3_DATA_MAX                      (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_DESC3_RSVD_MASK                     (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_DESC3_RSVD_SHIFT                    (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_DESC3_RSVD_MAX                      (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_AXI_ADDR0_REGION_SIZE_MASK          (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_AXI_ADDR0_REGION_SIZE_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_AXI_ADDR0_REGION_SIZE_MAX           (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_AXI_ADDR0_RSVD_MASK                 (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_AXI_ADDR0_RSVD_SHIFT                (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_AXI_ADDR0_RSVD_MAX                  (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_AXI_ADDR0_DATA_MASK                 (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_AXI_ADDR0_DATA_SHIFT                (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_AXI_ADDR0_DATA_MAX                  (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_AXI_ADDR1_DATA_MASK                 (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_AXI_ADDR1_DATA_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_14_AXI_ADDR1_DATA_MAX                  (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_ADDR0_NUM_BITS_MASK                 (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_ADDR0_NUM_BITS_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_ADDR0_NUM_BITS_MAX                  (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_DESC0_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_DESC0_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_DESC0_DATA_MAX                      (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_DESC1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_DESC1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_DESC1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_DESC3_DATA_MASK                     (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_DESC3_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_DESC3_DATA_MAX                      (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_DESC3_RSVD_MASK                     (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_DESC3_RSVD_SHIFT                    (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_DESC3_RSVD_MAX                      (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_AXI_ADDR0_REGION_SIZE_MASK          (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_AXI_ADDR0_REGION_SIZE_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_AXI_ADDR0_REGION_SIZE_MAX           (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_AXI_ADDR0_RSVD_MASK                 (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_AXI_ADDR0_RSVD_SHIFT                (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_AXI_ADDR0_RSVD_MAX                  (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_AXI_ADDR0_DATA_MASK                 (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_AXI_ADDR0_DATA_SHIFT                (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_AXI_ADDR0_DATA_MAX                  (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_AXI_ADDR1_DATA_MASK                 (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_AXI_ADDR1_DATA_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_15_AXI_ADDR1_DATA_MAX                  (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_ADDR0_NUM_BITS_MASK                 (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_ADDR0_NUM_BITS_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_ADDR0_NUM_BITS_MAX                  (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_DESC0_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_DESC0_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_DESC0_DATA_MAX                      (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_DESC1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_DESC1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_DESC1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_DESC3_DATA_MASK                     (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_DESC3_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_DESC3_DATA_MAX                      (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_DESC3_RSVD_MASK                     (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_DESC3_RSVD_SHIFT                    (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_DESC3_RSVD_MAX                      (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_AXI_ADDR0_REGION_SIZE_MASK          (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_AXI_ADDR0_REGION_SIZE_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_AXI_ADDR0_REGION_SIZE_MAX           (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_AXI_ADDR0_RSVD_MASK                 (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_AXI_ADDR0_RSVD_SHIFT                (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_AXI_ADDR0_RSVD_MAX                  (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_AXI_ADDR0_DATA_MASK                 (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_AXI_ADDR0_DATA_SHIFT                (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_AXI_ADDR0_DATA_MAX                  (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_AXI_ADDR1_DATA_MASK                 (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_AXI_ADDR1_DATA_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_16_AXI_ADDR1_DATA_MAX                  (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_ADDR0_NUM_BITS_MASK                 (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_ADDR0_NUM_BITS_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_ADDR0_NUM_BITS_MAX                  (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_DESC0_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_DESC0_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_DESC0_DATA_MAX                      (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_DESC1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_DESC1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_DESC1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_DESC3_DATA_MASK                     (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_DESC3_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_DESC3_DATA_MAX                      (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_DESC3_RSVD_MASK                     (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_DESC3_RSVD_SHIFT                    (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_DESC3_RSVD_MAX                      (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_AXI_ADDR0_REGION_SIZE_MASK          (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_AXI_ADDR0_REGION_SIZE_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_AXI_ADDR0_REGION_SIZE_MAX           (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_AXI_ADDR0_RSVD_MASK                 (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_AXI_ADDR0_RSVD_SHIFT                (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_AXI_ADDR0_RSVD_MAX                  (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_AXI_ADDR0_DATA_MASK                 (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_AXI_ADDR0_DATA_SHIFT                (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_AXI_ADDR0_DATA_MAX                  (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_AXI_ADDR1_DATA_MASK                 (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_AXI_ADDR1_DATA_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_17_AXI_ADDR1_DATA_MAX                  (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_ADDR0_NUM_BITS_MASK                 (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_ADDR0_NUM_BITS_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_ADDR0_NUM_BITS_MAX                  (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_DESC0_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_DESC0_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_DESC0_DATA_MAX                      (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_DESC1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_DESC1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_DESC1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_DESC3_DATA_MASK                     (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_DESC3_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_DESC3_DATA_MAX                      (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_DESC3_RSVD_MASK                     (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_DESC3_RSVD_SHIFT                    (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_DESC3_RSVD_MAX                      (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_AXI_ADDR0_REGION_SIZE_MASK          (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_AXI_ADDR0_REGION_SIZE_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_AXI_ADDR0_REGION_SIZE_MAX           (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_AXI_ADDR0_RSVD_MASK                 (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_AXI_ADDR0_RSVD_SHIFT                (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_AXI_ADDR0_RSVD_MAX                  (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_AXI_ADDR0_DATA_MASK                 (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_AXI_ADDR0_DATA_SHIFT                (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_AXI_ADDR0_DATA_MAX                  (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_AXI_ADDR1_DATA_MASK                 (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_AXI_ADDR1_DATA_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_18_AXI_ADDR1_DATA_MAX                  (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_ADDR0_NUM_BITS_MASK                 (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_ADDR0_NUM_BITS_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_ADDR0_NUM_BITS_MAX                  (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_DESC0_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_DESC0_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_DESC0_DATA_MAX                      (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_DESC1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_DESC1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_DESC1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_DESC3_DATA_MASK                     (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_DESC3_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_DESC3_DATA_MAX                      (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_DESC3_RSVD_MASK                     (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_DESC3_RSVD_SHIFT                    (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_DESC3_RSVD_MAX                      (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_AXI_ADDR0_REGION_SIZE_MASK          (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_AXI_ADDR0_REGION_SIZE_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_AXI_ADDR0_REGION_SIZE_MAX           (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_AXI_ADDR0_RSVD_MASK                 (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_AXI_ADDR0_RSVD_SHIFT                (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_AXI_ADDR0_RSVD_MAX                  (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_AXI_ADDR0_DATA_MASK                 (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_AXI_ADDR0_DATA_SHIFT                (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_AXI_ADDR0_DATA_MAX                  (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_AXI_ADDR1_DATA_MASK                 (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_AXI_ADDR1_DATA_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_19_AXI_ADDR1_DATA_MAX                  (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_ADDR0_NUM_BITS_MASK                 (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_ADDR0_NUM_BITS_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_ADDR0_NUM_BITS_MAX                  (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_DESC0_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_DESC0_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_DESC0_DATA_MAX                      (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_DESC1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_DESC1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_DESC1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_DESC3_DATA_MASK                     (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_DESC3_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_DESC3_DATA_MAX                      (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_DESC3_RSVD_MASK                     (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_DESC3_RSVD_SHIFT                    (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_DESC3_RSVD_MAX                      (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_AXI_ADDR0_REGION_SIZE_MASK          (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_AXI_ADDR0_REGION_SIZE_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_AXI_ADDR0_REGION_SIZE_MAX           (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_AXI_ADDR0_RSVD_MASK                 (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_AXI_ADDR0_RSVD_SHIFT                (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_AXI_ADDR0_RSVD_MAX                  (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_AXI_ADDR0_DATA_MASK                 (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_AXI_ADDR0_DATA_SHIFT                (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_AXI_ADDR0_DATA_MAX                  (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_AXI_ADDR1_DATA_MASK                 (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_AXI_ADDR1_DATA_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_20_AXI_ADDR1_DATA_MAX                  (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_ADDR0_NUM_BITS_MASK                 (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_ADDR0_NUM_BITS_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_ADDR0_NUM_BITS_MAX                  (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_DESC0_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_DESC0_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_DESC0_DATA_MAX                      (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_DESC1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_DESC1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_DESC1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_DESC3_DATA_MASK                     (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_DESC3_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_DESC3_DATA_MAX                      (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_DESC3_RSVD_MASK                     (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_DESC3_RSVD_SHIFT                    (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_DESC3_RSVD_MAX                      (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_AXI_ADDR0_REGION_SIZE_MASK          (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_AXI_ADDR0_REGION_SIZE_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_AXI_ADDR0_REGION_SIZE_MAX           (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_AXI_ADDR0_RSVD_MASK                 (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_AXI_ADDR0_RSVD_SHIFT                (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_AXI_ADDR0_RSVD_MAX                  (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_AXI_ADDR0_DATA_MASK                 (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_AXI_ADDR0_DATA_SHIFT                (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_AXI_ADDR0_DATA_MAX                  (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_AXI_ADDR1_DATA_MASK                 (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_AXI_ADDR1_DATA_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_21_AXI_ADDR1_DATA_MAX                  (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_ADDR0_NUM_BITS_MASK                 (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_ADDR0_NUM_BITS_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_ADDR0_NUM_BITS_MAX                  (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_DESC0_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_DESC0_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_DESC0_DATA_MAX                      (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_DESC1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_DESC1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_DESC1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_DESC3_DATA_MASK                     (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_DESC3_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_DESC3_DATA_MAX                      (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_DESC3_RSVD_MASK                     (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_DESC3_RSVD_SHIFT                    (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_DESC3_RSVD_MAX                      (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_AXI_ADDR0_REGION_SIZE_MASK          (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_AXI_ADDR0_REGION_SIZE_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_AXI_ADDR0_REGION_SIZE_MAX           (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_AXI_ADDR0_RSVD_MASK                 (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_AXI_ADDR0_RSVD_SHIFT                (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_AXI_ADDR0_RSVD_MAX                  (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_AXI_ADDR0_DATA_MASK                 (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_AXI_ADDR0_DATA_SHIFT                (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_AXI_ADDR0_DATA_MAX                  (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_AXI_ADDR1_DATA_MASK                 (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_AXI_ADDR1_DATA_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_22_AXI_ADDR1_DATA_MAX                  (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_ADDR0_NUM_BITS_MASK                 (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_ADDR0_NUM_BITS_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_ADDR0_NUM_BITS_MAX                  (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_DESC0_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_DESC0_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_DESC0_DATA_MAX                      (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_DESC1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_DESC1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_DESC1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_DESC3_DATA_MASK                     (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_DESC3_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_DESC3_DATA_MAX                      (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_DESC3_RSVD_MASK                     (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_DESC3_RSVD_SHIFT                    (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_DESC3_RSVD_MAX                      (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_AXI_ADDR0_REGION_SIZE_MASK          (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_AXI_ADDR0_REGION_SIZE_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_AXI_ADDR0_REGION_SIZE_MAX           (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_AXI_ADDR0_RSVD_MASK                 (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_AXI_ADDR0_RSVD_SHIFT                (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_AXI_ADDR0_RSVD_MAX                  (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_AXI_ADDR0_DATA_MASK                 (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_AXI_ADDR0_DATA_SHIFT                (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_AXI_ADDR0_DATA_MAX                  (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_AXI_ADDR1_DATA_MASK                 (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_AXI_ADDR1_DATA_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_23_AXI_ADDR1_DATA_MAX                  (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_ADDR0_NUM_BITS_MASK                 (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_ADDR0_NUM_BITS_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_ADDR0_NUM_BITS_MAX                  (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_DESC0_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_DESC0_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_DESC0_DATA_MAX                      (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_DESC1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_DESC1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_DESC1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_DESC3_DATA_MASK                     (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_DESC3_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_DESC3_DATA_MAX                      (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_DESC3_RSVD_MASK                     (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_DESC3_RSVD_SHIFT                    (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_DESC3_RSVD_MAX                      (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_AXI_ADDR0_REGION_SIZE_MASK          (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_AXI_ADDR0_REGION_SIZE_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_AXI_ADDR0_REGION_SIZE_MAX           (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_AXI_ADDR0_RSVD_MASK                 (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_AXI_ADDR0_RSVD_SHIFT                (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_AXI_ADDR0_RSVD_MAX                  (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_AXI_ADDR0_DATA_MASK                 (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_AXI_ADDR0_DATA_SHIFT                (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_AXI_ADDR0_DATA_MAX                  (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_AXI_ADDR1_DATA_MASK                 (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_AXI_ADDR1_DATA_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_24_AXI_ADDR1_DATA_MAX                  (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_ADDR0_NUM_BITS_MASK                 (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_ADDR0_NUM_BITS_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_ADDR0_NUM_BITS_MAX                  (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_DESC0_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_DESC0_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_DESC0_DATA_MAX                      (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_DESC1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_DESC1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_DESC1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_DESC3_DATA_MASK                     (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_DESC3_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_DESC3_DATA_MAX                      (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_DESC3_RSVD_MASK                     (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_DESC3_RSVD_SHIFT                    (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_DESC3_RSVD_MAX                      (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_AXI_ADDR0_REGION_SIZE_MASK          (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_AXI_ADDR0_REGION_SIZE_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_AXI_ADDR0_REGION_SIZE_MAX           (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_AXI_ADDR0_RSVD_MASK                 (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_AXI_ADDR0_RSVD_SHIFT                (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_AXI_ADDR0_RSVD_MAX                  (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_AXI_ADDR0_DATA_MASK                 (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_AXI_ADDR0_DATA_SHIFT                (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_AXI_ADDR0_DATA_MAX                  (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_AXI_ADDR1_DATA_MASK                 (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_AXI_ADDR1_DATA_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_25_AXI_ADDR1_DATA_MAX                  (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_ADDR0_NUM_BITS_MASK                 (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_ADDR0_NUM_BITS_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_ADDR0_NUM_BITS_MAX                  (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_DESC0_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_DESC0_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_DESC0_DATA_MAX                      (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_DESC1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_DESC1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_DESC1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_DESC3_DATA_MASK                     (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_DESC3_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_DESC3_DATA_MAX                      (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_DESC3_RSVD_MASK                     (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_DESC3_RSVD_SHIFT                    (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_DESC3_RSVD_MAX                      (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_AXI_ADDR0_REGION_SIZE_MASK          (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_AXI_ADDR0_REGION_SIZE_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_AXI_ADDR0_REGION_SIZE_MAX           (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_AXI_ADDR0_RSVD_MASK                 (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_AXI_ADDR0_RSVD_SHIFT                (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_AXI_ADDR0_RSVD_MAX                  (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_AXI_ADDR0_DATA_MASK                 (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_AXI_ADDR0_DATA_SHIFT                (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_AXI_ADDR0_DATA_MAX                  (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_AXI_ADDR1_DATA_MASK                 (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_AXI_ADDR1_DATA_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_26_AXI_ADDR1_DATA_MAX                  (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_ADDR0_NUM_BITS_MASK                 (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_ADDR0_NUM_BITS_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_ADDR0_NUM_BITS_MAX                  (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_DESC0_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_DESC0_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_DESC0_DATA_MAX                      (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_DESC1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_DESC1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_DESC1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_DESC3_DATA_MASK                     (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_DESC3_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_DESC3_DATA_MAX                      (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_DESC3_RSVD_MASK                     (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_DESC3_RSVD_SHIFT                    (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_DESC3_RSVD_MAX                      (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_AXI_ADDR0_REGION_SIZE_MASK          (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_AXI_ADDR0_REGION_SIZE_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_AXI_ADDR0_REGION_SIZE_MAX           (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_AXI_ADDR0_RSVD_MASK                 (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_AXI_ADDR0_RSVD_SHIFT                (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_AXI_ADDR0_RSVD_MAX                  (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_AXI_ADDR0_DATA_MASK                 (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_AXI_ADDR0_DATA_SHIFT                (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_AXI_ADDR0_DATA_MAX                  (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_AXI_ADDR1_DATA_MASK                 (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_AXI_ADDR1_DATA_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_27_AXI_ADDR1_DATA_MAX                  (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_ADDR0_NUM_BITS_MASK                 (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_ADDR0_NUM_BITS_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_ADDR0_NUM_BITS_MAX                  (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_DESC0_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_DESC0_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_DESC0_DATA_MAX                      (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_DESC1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_DESC1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_DESC1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_DESC3_DATA_MASK                     (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_DESC3_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_DESC3_DATA_MAX                      (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_DESC3_RSVD_MASK                     (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_DESC3_RSVD_SHIFT                    (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_DESC3_RSVD_MAX                      (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_AXI_ADDR0_REGION_SIZE_MASK          (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_AXI_ADDR0_REGION_SIZE_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_AXI_ADDR0_REGION_SIZE_MAX           (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_AXI_ADDR0_RSVD_MASK                 (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_AXI_ADDR0_RSVD_SHIFT                (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_AXI_ADDR0_RSVD_MAX                  (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_AXI_ADDR0_DATA_MASK                 (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_AXI_ADDR0_DATA_SHIFT                (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_AXI_ADDR0_DATA_MAX                  (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_AXI_ADDR1_DATA_MASK                 (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_AXI_ADDR1_DATA_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_28_AXI_ADDR1_DATA_MAX                  (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_ADDR0_NUM_BITS_MASK                 (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_ADDR0_NUM_BITS_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_ADDR0_NUM_BITS_MAX                  (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_DESC0_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_DESC0_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_DESC0_DATA_MAX                      (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_DESC1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_DESC1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_DESC1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_DESC3_DATA_MASK                     (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_DESC3_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_DESC3_DATA_MAX                      (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_DESC3_RSVD_MASK                     (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_DESC3_RSVD_SHIFT                    (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_DESC3_RSVD_MAX                      (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_AXI_ADDR0_REGION_SIZE_MASK          (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_AXI_ADDR0_REGION_SIZE_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_AXI_ADDR0_REGION_SIZE_MAX           (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_AXI_ADDR0_RSVD_MASK                 (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_AXI_ADDR0_RSVD_SHIFT                (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_AXI_ADDR0_RSVD_MAX                  (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_AXI_ADDR0_DATA_MASK                 (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_AXI_ADDR0_DATA_SHIFT                (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_AXI_ADDR0_DATA_MAX                  (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_AXI_ADDR1_DATA_MASK                 (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_AXI_ADDR1_DATA_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_29_AXI_ADDR1_DATA_MAX                  (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_ADDR0_NUM_BITS_MASK                 (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_ADDR0_NUM_BITS_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_ADDR0_NUM_BITS_MAX                  (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_DESC0_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_DESC0_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_DESC0_DATA_MAX                      (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_DESC1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_DESC1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_DESC1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_DESC3_DATA_MASK                     (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_DESC3_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_DESC3_DATA_MAX                      (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_DESC3_RSVD_MASK                     (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_DESC3_RSVD_SHIFT                    (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_DESC3_RSVD_MAX                      (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_AXI_ADDR0_REGION_SIZE_MASK          (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_AXI_ADDR0_REGION_SIZE_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_AXI_ADDR0_REGION_SIZE_MAX           (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_AXI_ADDR0_RSVD_MASK                 (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_AXI_ADDR0_RSVD_SHIFT                (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_AXI_ADDR0_RSVD_MAX                  (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_AXI_ADDR0_DATA_MASK                 (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_AXI_ADDR0_DATA_SHIFT                (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_AXI_ADDR0_DATA_MAX                  (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_AXI_ADDR1_DATA_MASK                 (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_AXI_ADDR1_DATA_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_30_AXI_ADDR1_DATA_MAX                  (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_ADDR0_NUM_BITS_MASK                 (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_ADDR0_NUM_BITS_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_ADDR0_NUM_BITS_MAX                  (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_ADDR0_RSVD_MASK                     (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_ADDR0_RSVD_SHIFT                    (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_ADDR0_RSVD_MAX                      (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_ADDR0_DATA_MASK                     (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_ADDR0_DATA_SHIFT                    (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_ADDR0_DATA_MAX                      (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_ADDR1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_ADDR1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_ADDR1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_DESC0_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_DESC0_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_DESC0_DATA_MAX                      (0xFFFFFFFFU)

/* DESC1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_DESC1_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_DESC1_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_DESC1_DATA_MAX                      (0xFFFFFFFFU)

/* DESC3 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_DESC3_DATA_MASK                     (0x007FFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_DESC3_DATA_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_DESC3_DATA_MAX                      (0x007FFFFFU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_DESC3_RSVD_MASK                     (0xFF800000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_DESC3_RSVD_SHIFT                    (0x00000017U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_DESC3_RSVD_MAX                      (0x000001FFU)

/* AXI_ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_AXI_ADDR0_REGION_SIZE_MASK          (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_AXI_ADDR0_REGION_SIZE_SHIFT         (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_AXI_ADDR0_REGION_SIZE_MAX           (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_AXI_ADDR0_RSVD_MASK                 (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_AXI_ADDR0_RSVD_SHIFT                (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_AXI_ADDR0_RSVD_MAX                  (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_AXI_ADDR0_DATA_MASK                 (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_AXI_ADDR0_DATA_SHIFT                (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_AXI_ADDR0_DATA_MAX                  (0x00FFFFFFU)

/* AXI_ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_AXI_ADDR1_DATA_MASK                 (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_AXI_ADDR1_DATA_SHIFT                (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_HP_WRAPPER_OB_31_AXI_ADDR1_DATA_MAX                  (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_0_ADDR0_NUM_BITS_MASK                     (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_0_ADDR0_NUM_BITS_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_0_ADDR0_NUM_BITS_MAX                      (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_0_ADDR0_RSVD0_MASK                        (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_0_ADDR0_RSVD0_SHIFT                       (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_0_ADDR0_RSVD0_MAX                         (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_0_ADDR0_DATA_MASK                         (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_0_ADDR0_DATA_SHIFT                        (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_0_ADDR0_DATA_MAX                          (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_0_ADDR1_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_0_ADDR1_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_0_ADDR1_DATA_MAX                          (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_1_ADDR0_NUM_BITS_MASK                     (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_1_ADDR0_NUM_BITS_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_1_ADDR0_NUM_BITS_MAX                      (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_1_ADDR0_RSVD0_MASK                        (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_1_ADDR0_RSVD0_SHIFT                       (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_1_ADDR0_RSVD0_MAX                         (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_1_ADDR0_DATA_MASK                         (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_1_ADDR0_DATA_SHIFT                        (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_1_ADDR0_DATA_MAX                          (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_1_ADDR1_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_1_ADDR1_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_1_ADDR1_DATA_MAX                          (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_2_ADDR0_NUM_BITS_MASK                     (0x0000003FU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_2_ADDR0_NUM_BITS_SHIFT                    (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_2_ADDR0_NUM_BITS_MAX                      (0x0000003FU)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_2_ADDR0_RSVD0_MASK                        (0x000000C0U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_2_ADDR0_RSVD0_SHIFT                       (0x00000006U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_2_ADDR0_RSVD0_MAX                         (0x00000003U)

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_2_ADDR0_DATA_MASK                         (0xFFFFFF00U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_2_ADDR0_DATA_SHIFT                        (0x00000008U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_2_ADDR0_DATA_MAX                          (0x00FFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_2_ADDR1_DATA_MASK                         (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_2_ADDR1_DATA_SHIFT                        (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_WRAPPER_IB_2_ADDR1_DATA_MAX                          (0xFFFFFFFFU)

/* C0 */

#define CSL_PCIE_RP_CORE_ATU_CREDIT_THRESHOLD_C0_DATA_MASK                        (0x00000FFFU)
#define CSL_PCIE_RP_CORE_ATU_CREDIT_THRESHOLD_C0_DATA_SHIFT                       (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_CREDIT_THRESHOLD_C0_DATA_MAX                         (0x00000FFFU)

#define CSL_PCIE_RP_CORE_ATU_CREDIT_THRESHOLD_C0_HEADER_MASK                      (0x000FF000U)
#define CSL_PCIE_RP_CORE_ATU_CREDIT_THRESHOLD_C0_HEADER_SHIFT                     (0x0000000CU)
#define CSL_PCIE_RP_CORE_ATU_CREDIT_THRESHOLD_C0_HEADER_MAX                       (0x000000FFU)

/* L0 */

#define CSL_PCIE_RP_CORE_ATU_LINK_DOWN_INDICATOR_BIT_L0_CLEAR_LINK_DOWN_BIT_TO_PROCEED_MASK (0x00000001U)
#define CSL_PCIE_RP_CORE_ATU_LINK_DOWN_INDICATOR_BIT_L0_CLEAR_LINK_DOWN_BIT_TO_PROCEED_SHIFT (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_LINK_DOWN_INDICATOR_BIT_L0_CLEAR_LINK_DOWN_BIT_TO_PROCEED_MAX (0x00000001U)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_0_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_0_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_0_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_0_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_0_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_0_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_1_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_1_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_1_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_1_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_1_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_1_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_2_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_2_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_2_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_2_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_2_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_2_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_3_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_3_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_3_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_3_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_3_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_3_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_4_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_4_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_4_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_4_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_4_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_4_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_5_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_5_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_5_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_5_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_5_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_5_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_6_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_6_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_6_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_6_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_6_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_6_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_7_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_7_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_7_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_7_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_7_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC0_WRAPPER_IB_EP_7_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_0_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_0_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_0_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_0_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_0_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_0_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_1_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_1_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_1_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_1_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_1_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_1_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_2_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_2_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_2_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_2_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_2_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_2_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_3_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_3_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_3_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_3_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_3_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_3_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_4_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_4_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_4_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_4_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_4_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_4_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_5_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_5_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_5_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_5_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_5_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_5_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_6_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_6_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_6_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_6_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_6_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_6_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_7_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_7_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_7_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_7_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_7_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC1_WRAPPER_IB_EP_7_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_0_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_0_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_0_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_0_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_0_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_0_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_1_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_1_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_1_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_1_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_1_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_1_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_2_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_2_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_2_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_2_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_2_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_2_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_3_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_3_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_3_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_3_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_3_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_3_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_4_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_4_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_4_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_4_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_4_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_4_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_5_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_5_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_5_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_5_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_5_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_5_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_6_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_6_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_6_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_6_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_6_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_6_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_7_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_7_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_7_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_7_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_7_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC2_WRAPPER_IB_EP_7_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_0_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_0_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_0_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_0_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_0_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_0_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_1_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_1_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_1_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_1_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_1_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_1_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_2_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_2_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_2_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_2_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_2_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_2_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_3_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_3_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_3_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_3_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_3_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_3_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_4_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_4_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_4_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_4_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_4_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_4_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_5_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_5_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_5_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_5_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_5_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_5_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_6_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_6_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_6_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_6_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_6_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_6_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_7_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_7_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_7_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_7_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_7_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC3_WRAPPER_IB_EP_7_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_0_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_0_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_0_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_0_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_0_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_0_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_1_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_1_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_1_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_1_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_1_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_1_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_2_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_2_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_2_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_2_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_2_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_2_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_3_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_3_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_3_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_3_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_3_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_3_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_4_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_4_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_4_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_4_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_4_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_4_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_5_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_5_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_5_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_5_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_5_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_5_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_6_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_6_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_6_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_6_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_6_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_6_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_7_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_7_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_7_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_7_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_7_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC4_WRAPPER_IB_EP_7_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_0_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_0_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_0_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_0_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_0_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_0_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_1_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_1_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_1_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_1_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_1_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_1_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_2_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_2_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_2_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_2_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_2_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_2_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_3_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_3_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_3_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_3_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_3_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_3_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_4_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_4_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_4_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_4_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_4_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_4_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_5_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_5_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_5_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_5_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_5_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_5_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_6_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_6_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_6_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_6_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_6_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_6_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_7_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_7_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_7_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_7_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_7_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC5_WRAPPER_IB_EP_7_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_0_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_0_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_0_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_0_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_0_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_0_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_1_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_1_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_1_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_1_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_1_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_1_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_2_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_2_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_2_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_2_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_2_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_2_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_3_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_3_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_3_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_3_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_3_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_3_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_4_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_4_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_4_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_4_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_4_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_4_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_5_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_5_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_5_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_5_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_5_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_5_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_6_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_6_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_6_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_6_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_6_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_6_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_7_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_7_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_7_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_7_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_7_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC6_WRAPPER_IB_EP_7_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_0_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_0_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_0_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_0_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_0_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_0_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_1_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_1_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_1_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_1_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_1_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_1_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_2_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_2_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_2_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_2_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_2_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_2_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_3_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_3_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_3_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_3_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_3_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_3_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_4_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_4_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_4_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_4_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_4_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_4_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_5_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_5_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_5_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_5_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_5_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_5_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_6_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_6_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_6_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_6_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_6_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_6_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_7_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_7_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_7_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_7_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_7_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC7_WRAPPER_IB_EP_7_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_0_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_0_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_0_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_0_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_0_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_0_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_1_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_1_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_1_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_1_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_1_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_1_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_2_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_2_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_2_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_2_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_2_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_2_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_3_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_3_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_3_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_3_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_3_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_3_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_4_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_4_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_4_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_4_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_4_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_4_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_5_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_5_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_5_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_5_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_5_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_5_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_6_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_6_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_6_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_6_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_6_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_6_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_7_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_7_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_7_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_7_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_7_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC8_WRAPPER_IB_EP_7_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_0_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_0_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_0_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_0_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_0_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_0_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_1_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_1_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_1_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_1_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_1_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_1_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_2_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_2_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_2_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_2_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_2_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_2_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_3_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_3_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_3_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_3_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_3_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_3_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_4_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_4_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_4_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_4_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_4_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_4_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_5_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_5_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_5_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_5_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_5_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_5_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_6_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_6_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_6_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_6_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_6_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_6_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_7_ADDR0_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_7_ADDR0_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_7_ADDR0_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_7_ADDR1_DATA_MASK                (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_7_ADDR1_DATA_SHIFT               (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC9_WRAPPER_IB_EP_7_ADDR1_DATA_MAX                 (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_0_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_0_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_0_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_0_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_0_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_0_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_1_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_1_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_1_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_1_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_1_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_1_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_2_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_2_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_2_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_2_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_2_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_2_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_3_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_3_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_3_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_3_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_3_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_3_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_4_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_4_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_4_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_4_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_4_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_4_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_5_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_5_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_5_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_5_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_5_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_5_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_6_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_6_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_6_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_6_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_6_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_6_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_7_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_7_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_7_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_7_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_7_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC10_WRAPPER_IB_EP_7_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_0_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_0_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_0_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_0_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_0_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_0_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_1_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_1_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_1_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_1_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_1_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_1_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_2_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_2_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_2_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_2_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_2_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_2_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_3_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_3_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_3_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_3_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_3_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_3_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_4_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_4_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_4_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_4_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_4_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_4_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_5_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_5_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_5_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_5_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_5_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_5_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_6_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_6_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_6_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_6_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_6_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_6_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_7_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_7_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_7_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_7_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_7_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC11_WRAPPER_IB_EP_7_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_0_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_0_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_0_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_0_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_0_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_0_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_1_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_1_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_1_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_1_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_1_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_1_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_2_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_2_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_2_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_2_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_2_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_2_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_3_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_3_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_3_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_3_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_3_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_3_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_4_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_4_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_4_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_4_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_4_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_4_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_5_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_5_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_5_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_5_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_5_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_5_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_6_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_6_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_6_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_6_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_6_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_6_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_7_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_7_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_7_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_7_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_7_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC12_WRAPPER_IB_EP_7_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_0_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_0_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_0_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_0_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_0_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_0_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_1_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_1_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_1_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_1_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_1_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_1_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_2_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_2_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_2_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_2_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_2_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_2_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_3_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_3_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_3_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_3_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_3_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_3_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_4_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_4_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_4_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_4_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_4_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_4_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_5_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_5_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_5_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_5_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_5_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_5_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_6_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_6_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_6_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_6_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_6_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_6_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_7_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_7_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_7_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_7_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_7_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC13_WRAPPER_IB_EP_7_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_0_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_0_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_0_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_0_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_0_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_0_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_1_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_1_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_1_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_1_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_1_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_1_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_2_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_2_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_2_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_2_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_2_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_2_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_3_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_3_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_3_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_3_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_3_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_3_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_4_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_4_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_4_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_4_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_4_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_4_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_5_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_5_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_5_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_5_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_5_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_5_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_6_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_6_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_6_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_6_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_6_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_6_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_7_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_7_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_7_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_7_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_7_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC14_WRAPPER_IB_EP_7_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_0_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_0_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_0_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_0_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_0_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_0_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_1_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_1_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_1_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_1_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_1_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_1_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_2_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_2_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_2_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_2_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_2_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_2_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_3_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_3_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_3_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_3_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_3_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_3_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_4_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_4_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_4_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_4_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_4_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_4_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_5_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_5_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_5_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_5_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_5_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_5_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_6_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_6_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_6_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_6_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_6_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_6_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_7_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_7_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_7_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_7_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_7_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC15_WRAPPER_IB_EP_7_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_0_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_0_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_0_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_0_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_0_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_0_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_1_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_1_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_1_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_1_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_1_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_1_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_2_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_2_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_2_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_2_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_2_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_2_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_3_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_3_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_3_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_3_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_3_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_3_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_4_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_4_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_4_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_4_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_4_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_4_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_5_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_5_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_5_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_5_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_5_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_5_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_6_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_6_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_6_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_6_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_6_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_6_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_7_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_7_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_7_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_7_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_7_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC16_WRAPPER_IB_EP_7_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_0_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_0_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_0_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_0_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_0_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_0_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_1_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_1_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_1_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_1_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_1_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_1_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_2_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_2_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_2_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_2_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_2_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_2_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_3_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_3_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_3_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_3_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_3_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_3_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_4_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_4_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_4_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_4_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_4_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_4_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_5_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_5_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_5_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_5_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_5_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_5_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_6_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_6_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_6_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_6_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_6_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_6_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_7_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_7_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_7_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_7_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_7_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC17_WRAPPER_IB_EP_7_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_0_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_0_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_0_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_0_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_0_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_0_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_1_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_1_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_1_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_1_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_1_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_1_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_2_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_2_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_2_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_2_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_2_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_2_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_3_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_3_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_3_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_3_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_3_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_3_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_4_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_4_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_4_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_4_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_4_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_4_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_5_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_5_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_5_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_5_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_5_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_5_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_6_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_6_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_6_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_6_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_6_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_6_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_7_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_7_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_7_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_7_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_7_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC18_WRAPPER_IB_EP_7_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_0_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_0_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_0_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_0_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_0_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_0_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_1_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_1_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_1_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_1_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_1_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_1_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_2_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_2_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_2_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_2_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_2_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_2_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_3_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_3_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_3_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_3_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_3_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_3_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_4_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_4_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_4_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_4_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_4_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_4_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_5_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_5_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_5_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_5_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_5_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_5_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_6_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_6_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_6_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_6_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_6_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_6_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_7_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_7_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_7_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_7_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_7_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC19_WRAPPER_IB_EP_7_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_0_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_0_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_0_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_0_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_0_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_0_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_1_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_1_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_1_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_1_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_1_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_1_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_2_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_2_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_2_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_2_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_2_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_2_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_3_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_3_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_3_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_3_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_3_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_3_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_4_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_4_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_4_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_4_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_4_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_4_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_5_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_5_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_5_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_5_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_5_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_5_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_6_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_6_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_6_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_6_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_6_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_6_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_7_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_7_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_7_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_7_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_7_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC20_WRAPPER_IB_EP_7_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_0_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_0_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_0_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_0_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_0_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_0_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_1_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_1_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_1_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_1_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_1_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_1_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_2_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_2_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_2_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_2_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_2_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_2_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_3_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_3_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_3_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_3_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_3_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_3_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_4_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_4_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_4_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_4_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_4_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_4_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_5_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_5_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_5_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_5_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_5_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_5_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_6_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_6_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_6_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_6_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_6_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_6_ADDR1_DATA_MAX                (0xFFFFFFFFU)

/* ADDR0 */

#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_7_ADDR0_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_7_ADDR0_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_7_ADDR0_DATA_MAX                (0xFFFFFFFFU)

/* ADDR1 */

#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_7_ADDR1_DATA_MASK               (0xFFFFFFFFU)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_7_ADDR1_DATA_SHIFT              (0x00000000U)
#define CSL_PCIE_RP_CORE_ATU_FUNC21_WRAPPER_IB_EP_7_ADDR1_DATA_MAX                (0xFFFFFFFFU)

#ifdef __cplusplus
}
#endif
#endif
