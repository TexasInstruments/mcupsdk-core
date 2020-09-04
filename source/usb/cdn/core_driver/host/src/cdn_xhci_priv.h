/**********************************************************************
* Copyright (C) 2012-2021 Cadence Design Systems, Inc.
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* 3. Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
**********************************************************************
* WARNING: This file is auto-generated using api-generator utility.
*          api-generator: 13.05.b3ee589
*          Do not edit it manually.
**********************************************************************
* XHCI driver for both host and device mode header file
**********************************************************************/

#ifndef CDN_XHCI_PRIV_H
#define CDN_XHCI_PRIV_H

#ifdef __cplusplus
extern "C"
{
#endif


/* parasoft-begin-suppress MISRA2012-RULE-1_1_a_c90-2 "C90 - limits, DRV-3906" */
/* parasoft-begin-suppress MISRA2012-RULE-1_1_b_c90-2 "C90 - limits, DRV-3906" */
/* parasoft suppress item  MISRA2012-DIR-4_8 "Consider hiding implementation of structure, DRV-4932" */

/** @defgroup ConfigInfo  Configuration and Hardware Operation Information
 *  The following definitions specify the driver operation environment that
 *  is defined by hardware configuration or client code. These defines are
 *  located in the header file of the core driver.
 *  @{
 */

/**********************************************************************
* Defines
**********************************************************************/
/** Maximum number of 32-bit double words for single ext. capability */
#define USBSSP_MAX_EXT_CAP_ELEM_DWORDS 256U

/**
 * Maximum number of extended capabilities
 * @remarks USB legacy support are not taken under consideration
 */
#define USBSSP_MAX_EXT_CAPS_COUNT 3U

/** Normal TRB */
#define USBSSP_TRB_NORMAL (uint32_t)1U

/** Setup stage TRB */
#define USBSSP_TRB_SETUP_STAGE (uint32_t)2U

/** Data stage TRB */
#define USBSSP_TRB_DATA_STAGE (uint32_t)3U

/** Status stage TRB */
#define USBSSP_TRB_STATUS_STAGE (uint32_t)4U

/** Isoch TRB */
#define USBSSP_TRB_ISOCH (uint32_t)5U

/** Link TRB */
#define USBSSP_TRB_LINK (uint32_t)6U

/** Event data TRB */
#define USBSSP_TRB_EVENT_DATA (uint32_t)7U

/** No Op TRB */
#define USBSSP_TRB_NO_OP (uint32_t)8U

/** Enable slot command TRB */
#define USBSSP_TRB_ENABLE_SLOT_COMMAND (uint32_t)9U

/** Disable slot command TRB */
#define USBSSP_TRB_DISABLE_SLOT_COMMAND (uint32_t)10U

/** Address device command TRB */
#define USBSSP_TRB_ADDR_DEV_CMD (uint32_t)11U

/** Configure endpoint command TRB */
#define USBSSP_TRB_CONF_EP_CMD (uint32_t)12U

/** Evaluate context command TRB */
#define USBSSP_TRB_EVALUATE_CXT_CMD (uint32_t)13U

/** Reset endpoint command TRB */
#define USBSSP_TRB_RESET_EP_CMD (uint32_t)14U

/** Stop endpoint command TRB */
#define USBSSP_TRB_STOP_EP_CMD (uint32_t)15U

/** Set TR Dequeue pointer command TRB */
#define USBSSP_TRB_SET_TR_DQ_PTR_CMD (uint32_t)16U

/** Reset device command TRB */
#define USBSSP_TRB_RESET_DEVICE_COMMAND (uint32_t)17U

/** Force event command TRB */
#define USBSSP_TRB_FORCE_EVENT_COMMAND (uint32_t)18U

/** Negotiate bandwidth TRB */
#define USBSSP_TRB_NEGOTIATE_BANDWIDTH (uint32_t)19U

/** Set Latency Tolerance Value Command TRB */
#define USBSSP_TRB_SET_LAT_TOL_VAL_CMD (uint32_t)20U

/** Get port bandwidth command TRB */
#define USBSSP_TRB_GET_PORT_BNDWTH_CMD (uint32_t)21U

/** Force header command TRB */
#define USBSSP_TRB_FORCE_HEADER_COMMAND (uint32_t)22U

/** No Op command TRB */
#define USBSSP_TRB_NO_OP_COMMAND (uint32_t)23U

/** Transfer event TRB */
#define USBSSP_TRB_TRANSFER_EVENT (uint32_t)32U

/** Command completion event TRB */
#define USBSSP_TRB_CMD_CMPL_EVT (uint32_t)33U

/** Port status change event TRB */
#define USBSSP_TRB_PORT_ST_CHG_EVT (uint32_t)34U

/** Bandwidth request event TRB */
#define USBSSP_TRB_BNDWTH_RQ_EVT (uint32_t)35U

/** Doorbell event TRB */
#define USBSSP_TRB_DOORBELL_EVENT (uint32_t)36U

/** Host controller event TRB */
#define USBSSP_TRB_HOST_CTRL_EVT (uint32_t)37U

/** Device notification event TRB */
#define USBSSP_TRB_DEV_NOTIFCN_EVT (uint32_t)38U

/** MFINDEX Wrap event TRB */
#define USBSSP_TRB_MFINDEX_WRAP_EVENT (uint32_t)39U

/** Vendor-defined TRBs (for USBSSP) */
#define USBSSP_TRB_NRDY_EVT (uint32_t)48U

/** Setup proto endpoint command TRB */
#define USBSSP_TRB_SETUP_PROTO_ENDP_CMD (uint32_t)49U

/** Get proto endpoint command TRB */
#define USBSSP_TRB_GET_PROTO_ENDP_CMD (uint32_t)50U

/** Set endpoint enable command TRB */
#define USBSSP_TRB_SET_ENDPS_ENA_CMD (uint32_t)51U

/** Get endpoint enable command TRB */
#define USBSSP_TRB_GET_ENDPS_ENA_CMD (uint32_t)52U

/** Add TDL command TRB */
#define USBSSP_TRB_ADD_TDL_CMD (uint32_t)53U

/** Halt endpoint command TRB */
#define USBSSP_TRB_HALT_ENDP_CMD (uint32_t)54U

/** Setup stage TRB */
#define USBSSP_TRB_SETUP_STAGE1 (uint32_t)55U

/** Halt endpoint command TRB */
#define USBSSP_TRB_HALT_ENDP_CMD1 (uint32_t)56U

/** Doorbell overflow event TRB */
#define USBSSP_TRB_DRBL_OVERFLOW_EVENT (uint32_t)57U

/** Flush endpoint command TRB */
#define USBSSP_TRB_FLUSH_EP_CMD (uint32_t)58U

/** VF security violation event TRB */
#define USBSSP_TRB_VF_SEC_VIOLN_EVT (uint32_t)59U

/** Position of Transfer Burst Count of a TRB depending on ETE value */
#define USBSSP_TRB_TBC_TBSTS_POS (uint32_t)7U

/** Mask of Transfer Burst Count of a TRB depending on ETE value */
#define USBSSP_TRB_TBC_TBSTS_MASK (uint32_t)0x180U

/** Position of TD size of TBC of a TRB depending on ETE value */
#define USBSSP_TRB_TDSIZE_TBC_POS (uint32_t)17U

/** Mask of TD size of TBC of a TRB depending on ETE value */
#define USBSSP_TRB_TDSIZE_TBC_MASK (uint32_t)0x3E0000U

/** Interrupter target position in TRB dword2 */
#define USBSSP_TRB_INTR_TRGT_POS (uint32_t)22U

/** Position offset for TRB Type */
#define USBSSP_TRB_TYPE_POS (uint32_t)10U

/** Position offset for BSR */
#define USBSSP_BSR_POS 9U

/** TRB max transfer length */
#define USBSSP_TRB_MAX_TRANSFER_LENGTH (uint32_t)0x10000U

/** System memory page size */
#define USBSSP_SYSTEM_MEMORY_PAGE_SIZE (uint32_t)0x10000U

/** Mask for TRB normal ISP field */
#define USBSSP_TRB_NORMAL_ISP_MASK (uint32_t)0x4U

/** Mask for TRB normal CH field */
#define USBSSP_TRB_NORMAL_CH_MASK (uint32_t)0x10U

/** Position offset for bmrequesttype field */
#define USBSSP_TRB_BMREQUESTTYPE_POS 0U

/** Position offset for brequest field */
#define USBSSP_TRB_BREQUEST_POS 8U

/** Position offset for wvalue field */
#define USBSSP_TRB_WVALUE_POS 16U

/** Position offset for windex field */
#define USBSSP_TRB_WINDEX_POS 0U

/** Position offset for VF ID field in force event command TRB */
#define USBSSP_TRB_FORCEEV_VF_ID_POS 16U

/** Position offset for VF Interrupter target in force event command TRB */
#define USBSSP_TRB_FRCEVT_VFINTTGT_POS 22U

/** Position offset for wlength field */
#define USBSSP_TRB_WLENGTH_POS 16U

/** Position offset for setup ID field */
#define USBSSP_TRB_SETUPID_POS 8U

/** Mask for setup ID field */
#define USBSSP_TRB_SETUPID_MASK (uint32_t)0x300U

/** Position offset for setup ID field */
#define USBSSP_TRB_STS_STG_STAT_POS 6U

/** TRB speed ID 2 */
#define USBSSP_TRB_SPEED_ID_2 0x00U

/** TRB speed ID 3 */
#define USBSSP_TRB_SPEED_ID_3 0x80U

/** Setup Data acknowledgment */
#define USBSSP_TRB_STS_STG_STAT_ACK 1U

/** Setup data stall */
#define USBSSP_TRB_STS_STG_STAT_STALL 0U

/** IOC mask for normal TRB */
#define USBSSP_TRB_NORMAL_IOC_MASK (uint32_t)0x20U

/** IDT mask for normal TRB */
#define USBSSP_TRB_NORMAL_IDT_MASK (uint32_t)0x40U

/** ENT mask for normal TRB */
#define USBSSP_TRB_NORMAL_ENT_MASK (uint32_t)0x02U

/** Transfer length mask for normal TRB */
#define USBSSP_TRB_NORM_TRFR_LEN_MSK (uint32_t)0x1FFFFU

/** Toggle cycle mask for Link TRB */
#define USBSSP_TRB_LNK_TGLE_CYC_MSK (uint32_t)0x02U

/** Residual length mask for event transfer TRB */
#define USBSSP_TRB_EVT_RESIDL_LEN_MSK (uint32_t)0xFFFFFFU

/** TRB completion code invalid */
#define USBSSP_TRB_COMPLETE_INVALID 0U

/** TRB completion code success */
#define USBSSP_TRB_COMPLETE_SUCCESS 1U

/** TRB completion code data buffer error */
#define USBSSP_TRB_CMPL_DATA_BUFF_ER 2U

/** TRB completion code babble detected error */
#define USBSSP_TRB_CMPL_BBL_DETECT_ER 3U

/** TRB completion code USB transaction error */
#define USBSSP_TRB_CMPL_USB_TRANSCN_ER 4U

/** TRB completion code TRB error */
#define USBSSP_TRB_COMPLETE_TRB_ERROR 5U

/** TRB completion code stall error */
#define USBSSP_TRB_COMPLETE_STALL_ERROR 6U

/** TRB completion code resource error */
#define USBSSP_TRB_CMPL_RSRC_ER 7U

/** TRB completion code bandwidth error */
#define USBSSP_TRB_CMPL_BDWTH_ER 8U

/** TRB completion code no slots available error */
#define USBSSP_TRB_CMPL_NO_SLTS_AVL_ER 9U

/** TRB completion code invalid stream type error */
#define USBSSP_TRB_CMPL_INVSTRM_TYP_ER 10U

/** TRB completion code slot not enabled error */
#define USBSSP_TRB_CMPL_SLT_NOT_EN_ER 11U

/** TRB completion code endpoint not enabled error */
#define USBSSP_TRB_CMPL_EP_NOT_EN_ER 12U

/** TRB completion code short packet error */
#define USBSSP_TRB_CMPL_SHORT_PKT 13U

/** TRB completion code ring underrun error */
#define USBSSP_TRB_CMPL_RING_UNDERRUN 14U

/** TRB completion code ring overrun error */
#define USBSSP_TRB_CMPL_RING_OVERRUN 15U

/** TRB completion code VF event ring full error */
#define USBSSP_TRB_CMPL_VF_EVTRNGFL_ER 16U

/** TRB completion code parameter error */
#define USBSSP_TRB_CMPL_PARAMETER_ER 17U

/** TRB completion code bandwidth overrun error */
#define USBSSP_TRB_CMPL_BDWTH_OVRRN_ER 18U

/** TRB completion code context state error */
#define USBSSP_TRB_CMPL_CXT_ST_ER 19U

/** TRB completion code no ping response error */
#define USBSSP_TRB_CMPL_NO_PNG_RSP_ER 20U

/** TRB completion code event ring full error */
#define USBSSP_TRB_CMPL_EVT_RNG_FL_ER 21U

/** TRB completion code incompatible device error */
#define USBSSP_TRB_CMPL_INCMPT_DEV_ER 22U

/** TRB completion code missed service error */
#define USBSSP_TRB_CMPL_MISSED_SRV_ER 23U

/** TRB completion code command ring stopped error */
#define USBSSP_TRB_CMPL_CMD_RNG_STOPPED 24U

/** TRB completion code command aborted error */
#define USBSSP_TRB_CMPL_CMD_ABORTED 25U

/** TRB completion code stopped error */
#define USBSSP_TRB_COMPLETE_STOPPED 26U

/** TRB completion code stopped - length invalid error */
#define USBSSP_TRB_CMPL_STOP_LEN_INV 27U

/** TRB completion code stopped - short packet error */
#define USBSSP_TRB_CMPL_STOP_SHORT_PKT 28U

/** TRB completion code max exit latency too large error */
#define USBSSP_TRB_CMPL_MAXEXTLT_LG_ER 29U

/** TRB completion code isoch buffer overrun error */
#define USBSSP_TRB_CMPL_ISO_BUFF_OVRUN 31U

/** TRB completion code event lost error */
#define USBSSP_TRB_CMPL_EVT_LOST_ER 32U

/** TRB completion code undefined error */
#define USBSSP_TRB_CMPL_UNDEFINED_ER 33U

/** TRB completion code invalid stream ID error */
#define USBSSP_TRB_CMPL_INV_STRM_ID_ER 34U

/** TRB completion code secondary bandwidth error */
#define USBSSP_TRB_CMPL_SEC_BDWTH_ER 35U

/** TRB completion code split transaction error */
#define USBSSP_TRB_CMPL_SPLT_TRNSCN_ER 36U

/** TRB completion code vendor defined error */
#define USBSSP_TRB_CMPL_CDNSDEF_ERCODES 192U

/** Position offset for TRT field in setup stage TRB */
#define USBSSP_TRB_SETUP_TRT_POS 16U

/** NO_DATA in TRT field of setup stage TRB */
#define USBSSP_TRB_SETUP_TRT_NO_DATA (uint32_t)0x0U

/** OUT_DATA in TRT field of setup stage TRB */
#define USBSSP_TRB_SETUP_TRT_OUT_DATA (uint32_t)0x2U

/** IN_DATA in TRT field of setup stage TRB */
#define USBSSP_TRB_SETUP_TRT_IN_DATA (uint32_t)0x3U

/** Position offset for Frame ID bitfield in Isoch TRB */
#define USBSSP_TRB_ISOCH_FRAME_ID_POS 20U

/** Mask of Frame ID bitfield in Isoch TRB */
#define USBSSP_TRB_ISOCH_FRAME_ID_MASK (uint32_t)0x7FF00000U

/** Position offset for SIA bitfield in Isoch TRB */
#define USBSSP_TRB_ISOCH_SIA_POS 31U

/** Mask for SIA bitfield in Isoch TRB */
#define USBSSP_TRB_ISOCH_SIA_MASK (uint32_t)0x80000000U

/** TRB transfer length mask */
#define USBSSP_TRB_TRANSFER_LENGTH_MASK (uint32_t)0x1FFFFU

/** Position offset for completion code */
#define USBSSP_COMPLETION_CODE_POS 24U

/** Position offset for Slot ID */
#define USBSSP_SLOT_ID_POS 24U

/** Position offset for endpoint */
#define USBSSP_ENDPOINT_POS 16U

/** Position offset for interrupter target */
#define USBSSP_INTERRUPTER_TARGET_POS 22U

/** position offset for transfer direction */
#define USBSSP_TRANSFER_DIR_POS 16U

/** Port Link state is RxDetect */
#define USBSSP_PORTSCUSB_PLS__RXDETECT 5U

/** Position offset for endpoint context interval */
#define USBSSP_EP_CONTEXT_INTERVAL_POS 16U

/** Upper 8 bits. The MAX ESIT Payload represent the total number of bytes this endpoint will transfer during an ESIT.  */
#define USBSSP_EP_CXT_MAXESITPLD_HI_POS 24U

/** Position offset for max packet size in endpoint context data structure */
#define USBSSP_EP_CXT_MAX_PKT_SZ_POS 16U

/** Position offset for max burst size in endpoint context data structure */
#define USBSSP_EP_CXT_MAX_BURST_SZ_POS 8U

/** Mask for max burst size in endpoint context data structure */
#define USBSSP_EP_CXT_MAX_BURST_SZ_MASK (uint32_t)0xFF00U

/** Position offset for MULT bitfield in endpoint context data structure. The value of MULT is dependent on LEC bit */
#define USBSSP_EP_CONTEXT_MULT_POS 8U

/** Mask for MULT bitfield in endpoint context data structure */
#define USBSSP_EP_CONTEXT_MULT_MASK (uint32_t)0x300U

/** Position offset for CErr bitfield in endpoint context data structure. This bitfield identifies the number of consecutive USB Bus Errors allowed while executing a TD */
#define USBSSP_EP_CONTEXT_CERR_POS 1U

/** Mask for CErr bitfield in endpoint context data structure */
#define USBSSP_EP_CONTEXT_CERR_MASK (uint32_t)0x6U

/** Position offset for max primary stream IDs in endpoint context data structure */
#define USBSSP_EP_CXT_PMAXSTREAMS_POS 10U

/** CErr bitfield in endpoint context data structure is set to '3' in normal operations */
#define USBSSP_EP_CONTEXT_3ERR 3U

/** Value to be set depending on endpoint direction */
#define USBSSP_EP_CXT_EP_DIR_IN 4U

/** Value to be set depending on endpoint direction */
#define USBSSP_EP_CXT_EP_DIR_OUT 0U

/**
 * The xHCI Spec says that for a control ep the average trb
 * length must be set by SW to 8.
 */
#define USBSSP_EP_CXT_EP_CTL_AVGTRB_LEN 8U

/**
 * The xHCI Spec says that for a int ep the average trb
 * length must be set by SW to 1024.
 */
#define USBSSP_EP_CXT_EP_INT_AVGTRB_LEN 1024U

/**
 * The xHCI Spec says that for a ISOC ep the average trb
 * length must be set by SW to 3072.
 */
#define USBSSP_EP_CXT_EP_ISO_AVGTRB_LEN 3072U

/**
 * The xHCI Spec says that for a BULK ep the average trb
 * length must be set by SW to 3072.
 */
#define USBSSP_EP_CXT_EP_BLK_AVGTRB_LEN 3072U

/** Position offset for average TRB length */
#define USBSSP_EP_CXT_EP_AVGTRBLEN_POS 0U

/** Lower 16 bits. The MAX ESIT Payload represent the total number of bytes this endpoint will transfer during an ESIT.  */
#define USBSSP_EP_CXT_MAXESITPLD_LO_POS 16U

/** Position offset for endpoint type */
#define USBSSP_EP_CONTEXT_EP_TYPE_POS 3U

/** Mask for endpoint type */
#define USBSSP_EP_CONTEXT_EP_TYPE_MASK (uint32_t)0x38U

/** Mask for endpoint context state */
#define USBSSP_EP_CONTEXT_STATE_MASK 7U

/** Endpoint 0 context offset */
#define USBSSP_EP0_CONTEXT_OFFSET 1U

/** Position offset for the index of the last valid endpoint context */
#define USBSSP_SLOT_CXT_CXT_ENT_POS 27U

/** Position offset for speed of the device */
#define USBSSP_SLOT_CONTEXT_SPEED_POS 20U

/** Position offset for the numver of ports */
#define USBSSP_SLOT_CXT_NUM_PORTS_POS 24U

/** Position offset for Root Hub Port Number */
#define USBSSP_SLOT_CXT_PORT_NUM_POS 16U

/** Position offset for slot state. This field is updated when a device slot transitions from one state to another */
#define USBSSP_SLOT_CONTEXT_STATE_POS 27U

/**
 *  @}
 */

/** @defgroup DataStructure Dynamic Data Structures
 *  This section defines the data structures used by the driver to provide
 *  hardware information, modification and dynamic operation of the driver.
 *  These data structures are defined in the header file of the core driver
 *  and utilized by the API.
 *  @{
 */

/**********************************************************************
* Forward declarations
**********************************************************************/
typedef struct USBSSP_CapabilityT_s USBSSP_CapabilityT;
typedef struct USBSSP_PortControlT_s USBSSP_PortControlT;
typedef struct USBSSP_OperationalT_s USBSSP_OperationalT;
typedef struct USBSSP_InterrupterT_s USBSSP_InterrupterT;
typedef struct USBSSP_RuntimeT_s USBSSP_RuntimeT;
typedef struct USBSSP_ExtCapElemT_s USBSSP_ExtCapElemT;
typedef struct USBSSP_ExtCapSetT_s USBSSP_ExtCapSetT;

/**********************************************************************
* Enumerations
**********************************************************************/
/** Endpoint types */
typedef enum
{
    /** Endpoint ISO Out */
    USBSSP_EP_CXT_EPTYP_ISO_OUT = 1U,
    /** Endpoint Bulk Out */
    USBSSP_EP_CXT_EPTYP_BLK_OUT = 2U,
    /** Endpoint INT Out */
    USBSSP_EP_CXT_EPTYP_INT_OUT = 3U,
    /** Endpoint control bidirectional */
    USBSSP_EP_CXT_EPTYP_CTL_BI = 4U,
    /** Endpoint ISO In */
    USBSSP_EP_CXT_EPTYP_ISO_IN = 5U,
    /** Endpoint Bulk In */
    USBSSP_EP_CXT_EPTYP_BLK_IN = 6U,
    /** Endpoint Int In */
    USBSSP_EP_CXT_EPTYP_INT_IN = 7U
} USBSSP_EpContextEpTypeT;

/** Endpoint states */
typedef enum
{
    /** Endpoint state disabled */
    USBSSP_EP_CONTEXT_EP_STATE_DISABLED = 0U,
    /** Endpoint state running */
    USBSSP_EP_CONTEXT_EP_STATE_RUNNING = 1U,
    /** Endpoint state halted */
    USBSSP_EP_CONTEXT_EP_STATE_HALTED = 2U,
    /** Endpoint state stopped */
    USBSSP_EP_CONTEXT_EP_STATE_STOPPED = 3U,
    /** Endpoint state error */
    USBSSP_EP_CONTEXT_EP_STATE_ERROR = 4U
} USBSSP_EpContexEpState;

/** Slot context states */
typedef enum
{
    /** Slot state disabled/enabled */
    USBSSP_SLOT_CONTEXT_STATE_DISABLED_ENABLED = 0U,
    /** Slot state default */
    USBSSP_SLOT_CONTEXT_STATE_DEFAULT = 1U,
    /** Slot state addressed */
    USBSSP_SLOT_CONTEXT_STATE_ADDRESSED = 2U,
    /** Slot state configured */
    USBSSP_SLOT_CONTEXT_STATE_CONFIGURED = 3U,
    /** Slot state reserved */
    USBSSP_SLOT_CONTEXT_STATE_RESERVED = 4U
} USBSSP_SlotContexState;

/** Port Control Register ID */
typedef enum
{
    /** Port status change register */
    USBSSP_PORTSC_REG_IDX = 0U,
    /** Port Power Management Status and Control register */
    USBSSP_PORTPMSC_REG_IDX = 1U,
    /** Port Link Info register */
    USBSSP_PORTLI_REG_IDX = 2U,
    /** Port hardware LPM control register */
    USBSSP_PORTHLPMC_REG_IDX = 3U
} USBSSP_PortControlRegIdx;

/**********************************************************************
* Structures and unions
**********************************************************************/
/** capability register structure */
struct USBSSP_CapabilityT_s
{
    /** Capability Register Length and Interface Version Number */
    uint32_t caplength_hciver;
    /** Structural Parameters 1 */
    uint32_t hcsparams1;
    /** Structural Parameters 2 */
    uint32_t hcsparams2;
    /** Structural Parameters 3 */
    uint32_t hcsparams3;
    /** Capability Parameters 1 */
    uint32_t hccparams1;
    /** Doorbell Offset 1 */
    uint32_t dboff;
    /** Runtime Registers Space Offset */
    uint32_t rtsoff;
    /** Capability Parameters 2 */
    uint32_t hccparams2;
} __attribute__((packed));

/** Quickaccess register structure */
struct USBSSP_QuickAccessRegs_s
{
    /** Copy of xhci capability register for quick access. */
    USBSSP_CapabilityT xHCCaps;
} __attribute__((packed));

/** Port control register structure */
struct USBSSP_PortControlT_s
{
    /** Port Status and Control */
    uint32_t portsc;
    /** Port Power Management Status and Control */
    uint32_t portpmsc;
    /** Port Link Info */
    uint32_t portli;
    /** Port Hardware LPM Control Register */
    uint32_t porthlpmc;
} __attribute__((packed));

/** operational register structure */
struct USBSSP_OperationalT_s
{
    /** USB Command */
    uint32_t usbcmd;
    /** USB Status */
    uint32_t usbsts;
    /** Page Size */
    uint32_t pagesize;
    /** reserved */
    uint32_t reserved_0[2];
    /** Device Notification Control */
    uint32_t dnctrl;
    /** Command Ring Control */
    uint64_t crcr;
    /** reserved */
    uint32_t reserved_1[4];
    /** Device Context Base Address Array Pointer */
    uint64_t dcbaap;
    /** Configure */
    uint32_t config;
    /** reserved */
    uint32_t reserved_2[241];
    /** Port Control Registers */
    USBSSP_PortControlT portControl;
} __attribute__((packed));

/** interrupter register structure */
struct USBSSP_InterrupterT_s
{
    /** Interrupter Management */
    uint32_t iman;
    /** Interrupter Moderation */
    uint32_t imod;
    /** Event Ring Segment Table Size */
    uint32_t erstsz;
    /** Reserved */
    uint32_t reserved;
    /** Event Ring Segment Table Base Address */
    uint64_t erstba;
    /** Event Ring Dequeue Pointer */
    uint64_t erdp;
} __attribute__((packed));

/** runtime register structure */
struct USBSSP_RuntimeT_s
{
    /** Microframe Index */
    uint32_t mfindex;
    /** Reserved */
    uint8_t reserved[28];
    /** Interrupter Register Sets */
    USBSSP_InterrupterT interrupters;
} __attribute__((packed));

/** Structure that describes single Extended Capability */
struct USBSSP_ExtCapElemT_s
{
    /** Value of first 32bit word for this Ext. Cap. (DWORD[0]) */
    uint32_t firstDwordVal;
    /** Capability ID (DWORD[0].CapabilityID) */
    uint8_t capId;
    /** Pointer to first SFR belonging to this capability */
    uint32_t* firstCapSfrPtr;
};

/** extCapSet structure */
struct USBSSP_ExtCapSetT_s
{
    /** Address of first Extended Capabilities' SFR (XEC_USBLEGSUP) */
    uint32_t* extCapsBaseAddr;
    /** Contents of USBLEGSUP SFR */
    uint32_t usbLegSup;
    /** Contents of USBLEGCTLSTS SFR */
    uint32_t usbLegCtlSts;
    /** Array with Extended Capabilities */
    USBSSP_ExtCapElemT extCaps[USBSSP_MAX_EXT_CAPS_COUNT];
    /** Number of Extended Capabilities recognized */
    uint8_t extCapsCount;
};

/** sfr structure */
struct USBSSP_SfrT_s
{
    /** XHCI capability register */
    USBSSP_CapabilityT* xhciCapability;
    /** XHCI operational register */
    USBSSP_OperationalT* xhciOperational;
    /** XHCI port control register */
    USBSSP_PortControlT* xhciPortControl;
    /** XHCI runtime register */
    USBSSP_RuntimeT* xhciRuntime;
    /** XHCI interrupter register */
    USBSSP_InterrupterT* xhciInterrupter;
    /** XHCI doorbell register */
    uint32_t* xhciDoorbell;
    /** xHCI capabilities are not handled as ordinary SFRs */
    USBSSP_ExtCapSetT xhciExtCaps;
};

/**
 *  @}
 */

/* parasoft-end-suppress MISRA2012-RULE-1_1_b_c90-2 */
/* parasoft-end-suppress MISRA2012-RULE-1_1_a_c90-2 */


#ifdef __cplusplus
}
#endif

#endif  /* CDN_XHCI_PRIV_H */
