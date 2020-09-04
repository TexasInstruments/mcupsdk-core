/********************************************************************
 * Copyright (C) 2019-2022 Texas Instruments Incorporated.
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
 *  Name        : cslr_xge_cpsw.h
*/
#ifndef CSLR_XGE_CPSW_H_
#define CSLR_XGE_CPSW_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PN_RESERVED_REG;           /* pn_reserved_reg */
    volatile uint32_t PN_CONTROL_REG;            /* pn_control_reg */
    volatile uint32_t PN_MAX_BLKS_REG;           /* pn_max_blks_reg */
    volatile uint8_t  Resv_16[4];
    volatile uint32_t PN_BLK_CNT_REG;            /* pn_blk_cnt_reg */
    volatile uint32_t PN_PORT_VLAN_REG;          /* pn_port_vlan_reg */
    volatile uint32_t PN_TX_PRI_MAP_REG;         /* pn_tx_pri_map_reg */
    volatile uint32_t PN_PRI_CTL_REG;            /* pn_pri_ctl_reg */
    volatile uint32_t PN_RX_PRI_MAP_REG;         /* pn_rx_pri_map_reg */
    volatile uint32_t PN_RX_MAXLEN_REG;          /* pn_rx_maxlen_reg */
    volatile uint32_t PN_TX_BLKS_PRI_REG;        /* pn_tx_blks_pri_reg */
    volatile uint32_t PN_RX_FLOW_THRESH_REG;     /* pn_rx_flow_thresh_reg */
    volatile uint32_t PN_IDLE2LPI_REG;           /* pn_idle2lpi_reg */
    volatile uint32_t PN_LPI2WAKE_REG;           /* pn_lpi2wake_reg */
    volatile uint32_t PN_EEE_STATUS_REG;         /* pn_eee_status_reg */
    volatile uint8_t  Resv_64[4];
    volatile uint32_t PN_IET_CONTROL_REG;        /* pn_iet_control_reg */
    volatile uint32_t PN_IET_STATUS_REG;         /* pn_iet_status_reg */
    volatile uint32_t PN_IET_VERIFY_REG;         /* pn_iet_verify_reg */
    volatile uint8_t  Resv_80[4];
    volatile uint32_t PN_FIFO_STATUS_REG;        /* pn_fifo_status_reg */
    volatile uint8_t  Resv_96[12];
    volatile uint32_t PN_EST_CONTROL_REG;        /* pn_est_control_reg */
    volatile uint8_t  Resv_288[188];
    volatile uint32_t PN_RX_DSCP_MAP_REG[8];     /* pn_rx_dscp_map_reg */
    volatile uint32_t PN_PRI_CIR_REG[8];         /* pn_pri_send_reg */
    volatile uint32_t PN_PRI_EIR_REG[8];         /* pn_pri_idle_reg */
    volatile uint32_t PN_TX_D_THRESH_SET_L_REG;   /* pn_tx_d_thresh_set_l_reg */
    volatile uint32_t PN_TX_D_THRESH_SET_H_REG;   /* pn_tx_d_thresh_set_h_reg */
    volatile uint32_t PN_TX_D_THRESH_CLR_L_REG;   /* pn_tx_d_thresh_clr_l_reg */
    volatile uint32_t PN_TX_D_THRESH_CLR_H_REG;   /* pn_tx_d_thresh_clr_h_reg */
    volatile uint32_t PN_TX_G_BUF_THRESH_SET_L_REG;   /* pn_tx_g_buf_thresh_set_l_reg */
    volatile uint32_t PN_TX_G_BUF_THRESH_SET_H_REG;   /* pn_tx_g_buf_thresh_set_h_reg */
    volatile uint32_t PN_TX_G_BUF_THRESH_CLR_L_REG;   /* pn_tx_g_buf_thresh_clr_l_reg */
    volatile uint32_t PN_TX_G_BUF_THRESH_CLR_H_REG;   /* pn_tx_g_buf_thresh_clr_h_reg */
    volatile uint8_t  Resv_768[352];
    volatile uint32_t PN_TX_D_OFLOW_ADDVAL_L_REG;   /* pn_tx_d_oflow_addval_l_reg */
    volatile uint32_t PN_TX_D_OFLOW_ADDVAL_H_REG;   /* pn_tx_d_oflow_addval_h_reg */
    volatile uint32_t PN_SA_L_REG;               /* pn_sa_l_reg */
    volatile uint32_t PN_SA_H_REG;               /* pn_sa_h_reg */
    volatile uint32_t PN_TS_CTL_REG;             /* pn_ts_ctl_reg */
    volatile uint32_t PN_TS_SEQ_LTYPE_REG;       /* pn_ts_seq_ltype_reg */
    volatile uint32_t PN_TS_VLAN_LTYPE_REG;      /* pn_ts_vlan_ltype_reg */
    volatile uint32_t PN_TS_CTL_LTYPE2_REG;      /* pn_ts_ctl_ltype2_reg */
    volatile uint32_t PN_TS_CTL2_REG;            /* pn_ts_ctl2_reg */
    volatile uint8_t  Resv_816[12];
    volatile uint32_t PN_MAC_CONTROL_REG;        /* pn_mac_control_reg */
    volatile uint32_t PN_MAC_STATUS_REG;         /* pn_mac_status_reg */
    volatile uint32_t PN_MAC_SOFT_RESET_REG;     /* pn_mac_soft_reset_reg */
    volatile uint32_t PN_MAC_BOFFTEST_REG;       /* pn_mac_bofftest_reg */
    volatile uint32_t PN_MAC_RX_PAUSETIMER_REG;   /* pn_mac_rx_pausetimer_reg */
    volatile uint8_t  Resv_848[12];
    volatile uint32_t PN_MAC_RXN_PAUSETIMER_REG[8];   /* pn_mac_rxn_pausetimer_reg */
    volatile uint32_t PN_MAC_TX_PAUSETIMER_REG;   /* pn_mac_tx_pausetimer_reg */
    volatile uint8_t  Resv_896[12];
    volatile uint32_t PN_MAC_TXN_PAUSETIMER_REG[8];   /* pn_mac_txn_pausetimer_reg */
    volatile uint32_t PN_MAC_EMCONTROL_REG;      /* pn_mac_emcontrol_reg */
    volatile uint32_t PN_MAC_TX_GAP_REG;         /* pn_mac_tx_gap_reg */
    volatile uint8_t  Resv_940[4];
    volatile uint32_t PN_INTERVLAN_OPX_POINTER_REG;   /* pn_opx_pointer_reg */
    volatile uint32_t PN_INTERVLAN_OPX_A_REG;    /* pn_opx_a_reg */
    volatile uint32_t PN_INTERVLAN_OPX_B_REG;    /* pn_opx_b_reg */
    volatile uint32_t PN_INTERVLAN_OPX_C_REG;    /* pn_opx_c_reg */
    volatile uint32_t PN_INTERVLAN_OPX_D_REG;    /* pn_opx_d_reg */
    volatile uint8_t  Resv_4096[3136];
} CSL_Xge_cpswEnetportRegs;


typedef struct {
    volatile uint32_t FETCH_LOC[128];            /* EST Fetch RAM */
} CSL_Xge_cpswRegs_CPSW_NU_EST;


typedef struct {
    volatile uint32_t RXGOODFRAMES;              /* RxGoodFrames */
    volatile uint32_t RXBROADCASTFRAMES;         /* RxBroadcastFrames */
    volatile uint32_t RXMULTICASTFRAMES;         /* RxMulticastFrames */
    volatile uint32_t RXPAUSEFRAMES;             /* RxPauseFrames */
    volatile uint32_t RXCRCERRORS;               /* RxCRCErrors */
    volatile uint32_t RXALIGNCODEERRORS;         /* RxAlignCodeErrors */
    volatile uint32_t RXOVERSIZEDFRAMES;         /* RxOversizedFrames */
    volatile uint32_t RXJABBERFRAMES;            /* RxJabberFrames */
    volatile uint32_t RXUNDERSIZEDFRAMES;        /* RxUndersizedFrames */
    volatile uint32_t RXFRAGMENTS;               /* RxFragments */
    volatile uint32_t ALE_DROP;                  /* ALE_Drop */
    volatile uint32_t ALE_OVERRUN_DROP;          /* ALE_Overrun_Drop */
    volatile uint32_t RXOCTETS;                  /* RxOctets */
    volatile uint32_t TXGOODFRAMES;              /* TxGoodFrames */
    volatile uint32_t TXBROADCASTFRAMES;         /* TxBroadcastFrames */
    volatile uint32_t TXMULTICASTFRAMES;         /* TxMulticastFrames */
    volatile uint32_t TXPAUSEFRAMES;             /* TxPauseFrames */
    volatile uint32_t TXDEFERREDFRAMES;          /* TxDeferredFrames */
    volatile uint32_t TXCOLLISIONFRAMES;         /* TxCollisionFrames */
    volatile uint32_t TXSINGLECOLLFRAMES;        /* TxSingleCollFrames */
    volatile uint32_t TXMULTCOLLFRAMES;          /* TxMultCollFrames */
    volatile uint32_t TXEXCESSIVECOLLISIONS;     /* TxExcessiveCollisions */
    volatile uint32_t TXLATECOLLISIONS;          /* TxLateCollisions */
    volatile uint32_t RXIPGERROR;                /* RxIPGError */
    volatile uint32_t TXCARRIERSENSEERRORS;      /* TxCarrierSenseErrors */
    volatile uint32_t TXOCTETS;                  /* TxOctets */
    volatile uint32_t OCTETFRAMES64;             /* OctetFrames64 */
    volatile uint32_t OCTETFRAMES65T127;         /* OctetFrames65t127 */
    volatile uint32_t OCTETFRAMES128T255;        /* OctetFrames128t255 */
    volatile uint32_t OCTETFRAMES256T511;        /* OctetFrames256t511 */
    volatile uint32_t OCTETFRAMES512T1023;       /* OctetFrames512t1023 */
    volatile uint32_t OCTETFRAMES1024TUP;        /* OctetFrames1024tUP */
    volatile uint32_t NETOCTETS;                 /* NetOctets */
    volatile uint32_t RX_BOTTOM_OF_FIFO_DROP;    /* Rx_Bottom_of_FIFO_Drop */
    volatile uint32_t PORTMASK_DROP;             /* Portmask_Drop */
    volatile uint32_t RX_TOP_OF_FIFO_DROP;       /* Rx_Top_of_FIFO_Drop */
    volatile uint32_t ALE_RATE_LIMIT_DROP;       /* ALE_Rate_Limit_Drop */
    volatile uint32_t ALE_VID_INGRESS_DROP;      /* ALE_VID_Ingress_Drop */
    volatile uint32_t ALE_DA_EQ_SA_DROP;         /* ALE_DA_EQ_SA_Drop */
    volatile uint32_t ALE_BLOCK_DROP;            /* ALE_Block_Drop */
    volatile uint32_t ALE_SECURE_DROP;           /* ALE_Secure_Drop */
    volatile uint32_t ALE_AUTH_DROP;             /* ALE_Auth_Drop */
    volatile uint32_t ALE_UNKN_UNI;              /* ALE_Unkn_Uni */
    volatile uint32_t ALE_UNKN_UNI_BCNT;         /* ALE_Unkn_Uni_Bcnt */
    volatile uint32_t ALE_UNKN_MLT;              /* ALE_Unkn_Mlt */
    volatile uint32_t ALE_UNKN_MLT_BCNT;         /* ALE_Unkn_Mlt_Bcnt */
    volatile uint32_t ALE_UNKN_BRD;              /* ALE_Unkn_Brd */
    volatile uint32_t ALE_UNKN_BRD_BCNT;         /* ALE_Unkn_Brd_Bcnt */
    volatile uint32_t ALE_POL_MATCH;             /* ALE_Pol_Match */
    volatile uint32_t ALE_POL_MATCH_RED;         /* ALE_Pol_Match_Red */
    volatile uint32_t ALE_POL_MATCH_YELLOW;      /* ALE_Pol_Match_Yellow */
    volatile uint32_t ALE_MULT_SA_DROP;          /* ALE_MULT_SA_DROP */
    volatile uint32_t ALE_DUAL_VLAN_DROP;        /* ALE_DUAL_VLAN_DROP */
    volatile uint32_t ALE_LEN_ERROR_DROP;        /* ALE_LEN_ERROR_DROP */
    volatile uint32_t ALE_IP_NEXT_HDR_DROP;      /* ALE_IP_NEXT_HDR_DROP */
    volatile uint32_t ALE_IPV4_FRAG_DROP;        /* ALE_IPV4_FRAG_DROP */
    volatile uint8_t  Resv_320[96];
    volatile uint32_t IET_RX_ASSEMBLY_ERROR_REG;   /* iet_rx_assembly_error */
    volatile uint32_t IET_RX_ASSEMBLY_OK_REG;    /* iet_rx_assembly_ok */
    volatile uint32_t IET_RX_SMD_ERROR_REG;      /* iet_rx_smd_error */
    volatile uint32_t IET_RX_FRAG_REG;           /* iet_rx_frag */
    volatile uint32_t IET_TX_HOLD_REG;           /* iet_tx_hold */
    volatile uint32_t IET_TX_FRAG_REG;           /* iet_tx_frag */
    volatile uint8_t  Resv_380[36];
    volatile uint32_t TX_MEMORY_PROTECT_ERROR;   /* Tx_Memory_Protect_Error */
    volatile uint32_t ENET_PN_TX_PRI_REG[8];     /* enet_pn_tx_pri */
    volatile uint32_t ENET_PN_TX_PRI_BCNT_REG[8];   /* enet_pn_tx_pri_bcnt */
    volatile uint32_t ENET_PN_TX_PRI_DROP_REG[8];   /* enet_pn_tx_pri_drop */
    volatile uint32_t ENET_PN_TX_PRI_DROP_BCNT_REG[8];   /* enet_pn_tx_pri_drop_bcnt */
} CSL_Xge_cpswStatsRegs;


typedef struct {
    volatile uint32_t ID_VER_REG;           /* idver_reg */
    volatile uint32_t CONTROL_REG;               /* control_reg */
    volatile uint8_t  Resv_16[8];
    volatile uint32_t EM_CONTROL_REG;            /* em_control_reg */
    volatile uint32_t STAT_PORT_EN_REG;          /* stat_port_en_reg */
    volatile uint32_t PTYPE_REG;                 /* ptype_reg */
    volatile uint32_t SOFT_IDLE_REG;             /* soft_idle_reg */
    volatile uint32_t THRU_RATE_REG;             /* thru_rate_reg */
    volatile uint32_t GAP_THRESH_REG;            /* gap_thresh_reg */
    volatile uint32_t TX_START_WDS_REG;          /* tx_start_wds_reg */
    volatile uint32_t EEE_PRESCALE_REG;          /* eee_prescale_reg */
    volatile uint32_t TX_G_OFLOW_THRESH_SET_REG;   /* tx_g_oflow_thresh_set_reg */
    volatile uint32_t TX_G_OFLOW_THRESH_CLR_REG;   /* tx_g_oflow_thresh_clr_reg */
    volatile uint32_t TX_G_BUF_THRESH_SET_L_REG;   /* tx_g_buf_thresh_set_l_reg */
    volatile uint32_t TX_G_BUF_THRESH_SET_H_REG;   /* tx_g_buf_thresh_set_h_reg */
    volatile uint32_t TX_G_BUF_THRESH_CLR_L_REG;   /* tx_g_buf_thresh_clr_l_reg */
    volatile uint32_t TX_G_BUF_THRESH_CLR_H_REG;   /* tx_g_buf_thresh_clr_h_reg */
    volatile uint8_t  Resv_80[8];
    volatile uint32_t VLAN_LTYPE_REG;            /* vlan_ltype_reg */
    volatile uint32_t EST_TS_DOMAIN_REG;         /* est_ts_domain_reg */
    volatile uint8_t  Resv_256[168];
    volatile uint32_t TX_PRI0_MAXLEN_REG;        /* tx_pri0_maxlen_reg */
    volatile uint32_t TX_PRI1_MAXLEN_REG;        /* tx_pri1_maxlen_reg */
    volatile uint32_t TX_PRI2_MAXLEN_REG;        /* tx_pri2_maxlen_reg */
    volatile uint32_t TX_PRI3_MAXLEN_REG;        /* tx_pri3_maxlen_reg */
    volatile uint32_t TX_PRI4_MAXLEN_REG;        /* tx_pri4_maxlen_reg */
    volatile uint32_t TX_PRI5_MAXLEN_REG;        /* tx_pri5_maxlen_reg */
    volatile uint32_t TX_PRI6_MAXLEN_REG;        /* tx_pri6_maxlen_reg */
    volatile uint32_t TX_PRI7_MAXLEN_REG;        /* tx_pri7_maxlen_reg */
    volatile uint8_t  Resv_4100[3812];
    volatile uint32_t P0_CONTROL_REG;            /* p0_control_reg */
    volatile uint32_t P0_FLOW_ID_OFFSET_REG;     /* p0_flow_id_offset_reg */
    volatile uint8_t  Resv_4112[4];
    volatile uint32_t P0_BLK_CNT_REG;            /* p0_blk_cnt_reg */
    volatile uint32_t P0_PORT_VLAN_REG;          /* p0_port_vlan_reg */
    volatile uint32_t P0_TX_PRI_MAP_REG;         /* p0_tx_pri_map_reg */
    volatile uint32_t P0_PRI_CTL_REG;            /* p0_pri_ctl_reg */
    volatile uint32_t P0_RX_PRI_MAP_REG;         /* p0_rx_pri_map_reg */
    volatile uint32_t P0_RX_MAXLEN_REG;          /* p0_rx_maxlen_reg */
    volatile uint32_t P0_TX_BLKS_PRI_REG;        /* p0_tx_blks_pri_reg */
    volatile uint8_t  Resv_4144[4];
    volatile uint32_t P0_IDLE2LPI_REG;           /* p0_idle2lpi_reg */
    volatile uint32_t P0_LPI2WAKE_REG;           /* p0_lpi2wake_reg */
    volatile uint32_t P0_EEE_STATUS_REG;         /* p0_eee_status_reg */
    volatile uint32_t P0_RX_PKTS_PRI_REG;        /* p0_rx_pkts_pri_reg */
    volatile uint8_t  Resv_76[12];
    volatile uint32_t P0_RX_GAP_REG;             /* p0_rx_gap_reg */
    volatile uint32_t P0_FIFO_STATUS_REG;        /* p0_fifo_status_reg */
    volatile uint8_t  Resv_4384[204];
    volatile uint32_t P0_RX_DSCP_MAP_REG[8];     /* p0_rx_dscp_map_reg */
    volatile uint32_t P0_PRI_CIR_REG[8];         /* p0_pri_cir_reg */
    volatile uint32_t P0_PRI_EIR_REG[8];         /* p0_pri_eir_reg */
    volatile uint32_t P0_TX_D_THRESH_SET_L_REG;   /* p0_tx_d_thresh_set_l_reg */
    volatile uint32_t P0_TX_D_THRESH_SET_H_REG;   /* p0_tx_d_thresh_set_h_reg */
    volatile uint32_t P0_TX_D_THRESH_CLR_L_REG;   /* p0_tx_d_thresh_clr_l_reg */
    volatile uint32_t P0_TX_D_THRESH_CLR_H_REG;   /* p0_tx_d_thresh_clr_h_reg */
    volatile uint32_t P0_TX_G_BUF_THRESH_SET_L_REG;   /* p0_tx_g_buf_thresh_set_l_reg */
    volatile uint32_t P0_TX_G_BUF_THRESH_SET_H_REG;   /* p0_tx_g_buf_thresh_set_h_reg */
    volatile uint32_t P0_TX_G_BUF_THRESH_CLR_L_REG;   /* p0_tx_g_buf_thresh_clr_l_reg */
    volatile uint32_t P0_TX_G_BUF_THRESH_CLR_H_REG;   /* p0_tx_g_buf_thresh_clr_h_reg */
    volatile uint8_t  Resv_4864[352];
    volatile uint32_t P0_SRC_ID_A_REG;           /* p0_src_id_a_reg */
    volatile uint32_t P0_SRC_ID_B_REG;           /* p0_src_id_b_reg */
    volatile uint8_t  Resv_4896[24];
    volatile uint32_t P0_HOST_BLKS_PRI_REG;      /* p0_host_blks_pri_reg */
    volatile uint8_t  Resv_8192[3292];
    CSL_Xge_cpswEnetportRegs ENETPORT[8];
    volatile uint8_t  Resv_73728[32768];
    CSL_Xge_cpswRegs_CPSW_NU_EST CPSW_NU_EST[8];
    volatile uint8_t  Resv_106496[28672];
    CSL_Xge_cpswStatsRegs STATS[9];
} CSL_Xge_cpswRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_XGE_CPSW_ID_VER_REG                                           (0x00000000U)
#define CSL_XGE_CPSW_CONTROL_REG                                               (0x00000004U)
#define CSL_XGE_CPSW_EM_CONTROL_REG                                            (0x00000010U)
#define CSL_XGE_CPSW_STAT_PORT_EN_REG                                          (0x00000014U)
#define CSL_XGE_CPSW_PTYPE_REG                                                 (0x00000018U)
#define CSL_XGE_CPSW_SOFT_IDLE_REG                                             (0x0000001CU)
#define CSL_XGE_CPSW_THRU_RATE_REG                                             (0x00000020U)
#define CSL_XGE_CPSW_GAP_THRESH_REG                                            (0x00000024U)
#define CSL_XGE_CPSW_TX_START_WDS_REG                                          (0x00000028U)
#define CSL_XGE_CPSW_EEE_PRESCALE_REG                                          (0x0000002CU)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG                                 (0x00000030U)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG                                 (0x00000034U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_SET_L_REG                                 (0x00000038U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_SET_H_REG                                 (0x0000003CU)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_CLR_L_REG                                 (0x00000040U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_CLR_H_REG                                 (0x00000044U)
#define CSL_XGE_CPSW_VLAN_LTYPE_REG                                            (0x00000050U)
#define CSL_XGE_CPSW_EST_TS_DOMAIN_REG                                         (0x00000054U)
#define CSL_XGE_CPSW_TX_PRI0_MAXLEN_REG                                        (0x00000100U)
#define CSL_XGE_CPSW_TX_PRI1_MAXLEN_REG                                        (0x00000104U)
#define CSL_XGE_CPSW_TX_PRI2_MAXLEN_REG                                        (0x00000108U)
#define CSL_XGE_CPSW_TX_PRI3_MAXLEN_REG                                        (0x0000010CU)
#define CSL_XGE_CPSW_TX_PRI4_MAXLEN_REG                                        (0x00000110U)
#define CSL_XGE_CPSW_TX_PRI5_MAXLEN_REG                                        (0x00000114U)
#define CSL_XGE_CPSW_TX_PRI6_MAXLEN_REG                                        (0x00000118U)
#define CSL_XGE_CPSW_TX_PRI7_MAXLEN_REG                                        (0x0000011CU)
#define CSL_XGE_CPSW_P0_CONTROL_REG                                            (0x00001004U)
#define CSL_XGE_CPSW_P0_FLOW_ID_OFFSET_REG                                     (0x00001008U)
#define CSL_XGE_CPSW_P0_BLK_CNT_REG                                            (0x00001010U)
#define CSL_XGE_CPSW_P0_PORT_VLAN_REG                                          (0x00001014U)
#define CSL_XGE_CPSW_P0_TX_PRI_MAP_REG                                         (0x00001018U)
#define CSL_XGE_CPSW_P0_PRI_CTL_REG                                            (0x0000101CU)
#define CSL_XGE_CPSW_P0_RX_PRI_MAP_REG                                         (0x00001020U)
#define CSL_XGE_CPSW_P0_RX_MAXLEN_REG                                          (0x00001024U)
#define CSL_XGE_CPSW_P0_TX_BLKS_PRI_REG                                        (0x00001028U)
#define CSL_XGE_CPSW_P0_IDLE2LPI_REG                                           (0x00001030U)
#define CSL_XGE_CPSW_P0_LPI2WAKE_REG                                           (0x00001034U)
#define CSL_XGE_CPSW_P0_EEE_STATUS_REG                                         (0x00001038U)
#define CSL_XGE_CPSW_P0_RX_PKTS_PRI_REG                                        (0x0000103CU)
#define CSL_XGE_CPSW_P0_RX_GAP_REG                                             (0x0000104CU)
#define CSL_XGE_CPSW_P0_FIFO_STATUS_REG                                        (0x00001050U)
#define CSL_XGE_CPSW_P0_RX_DSCP_MAP_REG(P0_RX_DSCP_MAP_REG)                    (0x00001120U+((P0_RX_DSCP_MAP_REG)*0x4U))
#define CSL_XGE_CPSW_P0_PRI_CIR_REG(P0_PRI_CIR_REG)                            (0x00001140U+((P0_PRI_CIR_REG)*0x4U))
#define CSL_XGE_CPSW_P0_PRI_EIR_REG(P0_PRI_EIR_REG)                            (0x00001160U+((P0_PRI_EIR_REG)*0x4U))
#define CSL_XGE_CPSW_P0_TX_D_THRESH_SET_L_REG                                  (0x00001180U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_SET_H_REG                                  (0x00001184U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_CLR_L_REG                                  (0x00001188U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_CLR_H_REG                                  (0x0000118CU)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_SET_L_REG                              (0x00001190U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_SET_H_REG                              (0x00001194U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_L_REG                              (0x00001198U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_H_REG                              (0x0000119CU)
#define CSL_XGE_CPSW_P0_SRC_ID_A_REG                                           (0x00001300U)
#define CSL_XGE_CPSW_P0_SRC_ID_B_REG                                           (0x00001304U)
#define CSL_XGE_CPSW_P0_HOST_BLKS_PRI_REG                                      (0x00001320U)
#define CSL_XGE_CPSW_PN_RESERVED_REG(ENETPORT)                        (0x00002000U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_CONTROL_REG(ENETPORT)                         (0x00002004U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_MAX_BLKS_REG(ENETPORT)                        (0x00002008U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_BLK_CNT_REG(ENETPORT)                         (0x00002010U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_PORT_VLAN_REG(ENETPORT)                       (0x00002014U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_TX_PRI_MAP_REG(ENETPORT)                      (0x00002018U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_PRI_CTL_REG(ENETPORT)                         (0x0000201CU+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_RX_PRI_MAP_REG(ENETPORT)                      (0x00002020U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_RX_MAXLEN_REG(ENETPORT)                       (0x00002024U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_TX_BLKS_PRI_REG(ENETPORT)                     (0x00002028U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_RX_FLOW_THRESH_REG(ENETPORT)                  (0x0000202CU+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_IDLE2LPI_REG(ENETPORT)                        (0x00002030U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_LPI2WAKE_REG(ENETPORT)                        (0x00002034U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_EEE_STATUS_REG(ENETPORT)                      (0x00002038U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_IET_CONTROL_REG(ENETPORT)                     (0x00002040U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_IET_STATUS_REG(ENETPORT)                      (0x00002044U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_IET_VERIFY_REG(ENETPORT)                      (0x00002048U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_FIFO_STATUS_REG(ENETPORT)                     (0x00002050U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_EST_CONTROL_REG(ENETPORT)                     (0x00002060U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_RX_DSCP_MAP_REG(ENETPORT,PN_RX_DSCP_MAP_REG)  (0x00002120U+((ENETPORT)*0x1000U)+((PN_RX_DSCP_MAP_REG)*0x4U))
#define CSL_XGE_CPSW_PN_PRI_CIR_REG(ENETPORT,PN_PRI_CIR_REG)          (0x00002140U+((ENETPORT)*0x1000U)+((PN_PRI_CIR_REG)*0x4U))
#define CSL_XGE_CPSW_PN_PRI_EIR_REG(ENETPORT,PN_PRI_EIR_REG)          (0x00002160U+((ENETPORT)*0x1000U)+((PN_PRI_EIR_REG)*0x4U))
#define CSL_XGE_CPSW_PN_TX_D_THRESH_SET_L_REG(ENETPORT)               (0x00002180U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_TX_D_THRESH_SET_H_REG(ENETPORT)               (0x00002184U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_TX_D_THRESH_CLR_L_REG(ENETPORT)               (0x00002188U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_TX_D_THRESH_CLR_H_REG(ENETPORT)               (0x0000218CU+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_SET_L_REG(ENETPORT)           (0x00002190U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_SET_H_REG(ENETPORT)           (0x00002194U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_L_REG(ENETPORT)           (0x00002198U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_H_REG(ENETPORT)           (0x0000219CU+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_L_REG(ENETPORT)             (0x00002300U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_H_REG(ENETPORT)             (0x00002304U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_SA_L_REG(ENETPORT)                            (0x00002308U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_SA_H_REG(ENETPORT)                            (0x0000230CU+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_TS_CTL_REG(ENETPORT)                          (0x00002310U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_TS_SEQ_LTYPE_REG(ENETPORT)                    (0x00002314U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_TS_VLAN_LTYPE_REG(ENETPORT)                   (0x00002318U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG(ENETPORT)                   (0x0000231CU+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_TS_CTL2_REG(ENETPORT)                         (0x00002320U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG(ENETPORT)                     (0x00002330U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG(ENETPORT)                      (0x00002334U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_MAC_SOFT_RESET_REG(ENETPORT)                  (0x00002338U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_MAC_BOFFTEST_REG(ENETPORT)                    (0x0000233CU+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_MAC_RX_PAUSETIMER_REG(ENETPORT)               (0x00002340U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_MAC_RXN_PAUSETIMER_REG(ENETPORT,PN_MAC_RXN_PAUSETIMER_REG) (0x00002350U+((ENETPORT)*0x1000U)+((PN_MAC_RXN_PAUSETIMER_REG)*0x4U))
#define CSL_XGE_CPSW_PN_MAC_TX_PAUSETIMER_REG(ENETPORT)               (0x00002370U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_MAC_TXN_PAUSETIMER_REG(ENETPORT,PN_MAC_TXN_PAUSETIMER_REG) (0x00002380U+((ENETPORT)*0x1000U)+((PN_MAC_TXN_PAUSETIMER_REG)*0x4U))
#define CSL_XGE_CPSW_PN_MAC_EMCONTROL_REG(ENETPORT)                   (0x000023A0U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_MAC_TX_GAP_REG(ENETPORT)                      (0x000023A4U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_POINTER_REG(ENETPORT)           (0x000023ACU+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_A_REG(ENETPORT)                 (0x000023B0U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_B_REG(ENETPORT)                 (0x000023B4U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_C_REG(ENETPORT)                 (0x000023B8U+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_D_REG(ENETPORT)                 (0x000023BCU+((ENETPORT)*0x1000U))
#define CSL_XGE_CPSW_CPSW_NU_EST_FETCH_LOC(CPSW_NU_EST,FETCH_LOC)              (0x00012000U+((CPSW_NU_EST)*0x200U)+((FETCH_LOC)*0x4U))
#define CSL_XGE_CPSW_STATS_RXGOODFRAMES(STATS)                                 (0x0001A000U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_RXBROADCASTFRAMES(STATS)                            (0x0001A004U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_RXMULTICASTFRAMES(STATS)                            (0x0001A008U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_RXPAUSEFRAMES(STATS)                                (0x0001A00CU+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_RXCRCERRORS(STATS)                                  (0x0001A010U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_RXALIGNCODEERRORS(STATS)                            (0x0001A014U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_RXOVERSIZEDFRAMES(STATS)                            (0x0001A018U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_RXJABBERFRAMES(STATS)                               (0x0001A01CU+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_RXUNDERSIZEDFRAMES(STATS)                           (0x0001A020U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_RXFRAGMENTS(STATS)                                  (0x0001A024U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_ALE_DROP(STATS)                                     (0x0001A028U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_ALE_OVERRUN_DROP(STATS)                             (0x0001A02CU+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_RXOCTETS(STATS)                                     (0x0001A030U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_TXGOODFRAMES(STATS)                                 (0x0001A034U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_TXBROADCASTFRAMES(STATS)                            (0x0001A038U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_TXMULTICASTFRAMES(STATS)                            (0x0001A03CU+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_TXPAUSEFRAMES(STATS)                                (0x0001A040U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_TXDEFERREDFRAMES(STATS)                             (0x0001A044U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_TXCOLLISIONFRAMES(STATS)                            (0x0001A048U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_TXSINGLECOLLFRAMES(STATS)                           (0x0001A04CU+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_TXMULTCOLLFRAMES(STATS)                             (0x0001A050U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_TXEXCESSIVECOLLISIONS(STATS)                        (0x0001A054U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_TXLATECOLLISIONS(STATS)                             (0x0001A058U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_RXIPGERROR(STATS)                                   (0x0001A05CU+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_TXCARRIERSENSEERRORS(STATS)                         (0x0001A060U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_TXOCTETS(STATS)                                     (0x0001A064U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_OCTETFRAMES64(STATS)                                (0x0001A068U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_OCTETFRAMES65T127(STATS)                            (0x0001A06CU+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_OCTETFRAMES128T255(STATS)                           (0x0001A070U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_OCTETFRAMES256T511(STATS)                           (0x0001A074U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_OCTETFRAMES512T1023(STATS)                          (0x0001A078U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_OCTETFRAMES1024TUP(STATS)                           (0x0001A07CU+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_NETOCTETS(STATS)                                    (0x0001A080U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_RX_BOTTOM_OF_FIFO_DROP(STATS)                       (0x0001A084U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_PORTMASK_DROP(STATS)                                (0x0001A088U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_RX_TOP_OF_FIFO_DROP(STATS)                          (0x0001A08CU+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_ALE_RATE_LIMIT_DROP(STATS)                          (0x0001A090U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_ALE_VID_INGRESS_DROP(STATS)                         (0x0001A094U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_ALE_DA_EQ_SA_DROP(STATS)                            (0x0001A098U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_ALE_BLOCK_DROP(STATS)                               (0x0001A09CU+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_ALE_SECURE_DROP(STATS)                              (0x0001A0A0U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_ALE_AUTH_DROP(STATS)                                (0x0001A0A4U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_ALE_UNKN_UNI(STATS)                                 (0x0001A0A8U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_ALE_UNKN_UNI_BCNT(STATS)                            (0x0001A0ACU+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_ALE_UNKN_MLT(STATS)                                 (0x0001A0B0U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_ALE_UNKN_MLT_BCNT(STATS)                            (0x0001A0B4U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_ALE_UNKN_BRD(STATS)                                 (0x0001A0B8U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_ALE_UNKN_BRD_BCNT(STATS)                            (0x0001A0BCU+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_ALE_POL_MATCH(STATS)                                (0x0001A0C0U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_ALE_POL_MATCH_RED(STATS)                            (0x0001A0C4U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_ALE_POL_MATCH_YELLOW(STATS)                         (0x0001A0C8U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_ALE_MULT_SA_DROP(STATS)                             (0x0001A0CCU+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_ALE_DUAL_VLAN_DROP(STATS)                           (0x0001A0D0U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_ALE_LEN_ERROR_DROP(STATS)                           (0x0001A0D4U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_ALE_IP_NEXT_HDR_DROP(STATS)                         (0x0001A0D8U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_ALE_IPV4_FRAG_DROP(STATS)                           (0x0001A0DCU+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_IET_RX_ASSEMBLY_ERROR_REG(STATS)                    (0x0001A140U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_IET_RX_ASSEMBLY_OK_REG(STATS)                       (0x0001A144U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_IET_RX_SMD_ERROR_REG(STATS)                         (0x0001A148U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_IET_RX_FRAG_REG(STATS)                              (0x0001A14CU+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_IET_TX_HOLD_REG(STATS)                              (0x0001A150U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_IET_TX_FRAG_REG(STATS)                              (0x0001A154U+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_TX_MEMORY_PROTECT_ERROR(STATS)                      (0x0001A17CU+((STATS)*0x200U))
#define CSL_XGE_CPSW_STATS_ENET_PN_TX_PRI_REG(STATS,ENET_PN_TX_PRI_REG)        (0x0001A180U+((STATS)*0x200U)+((ENET_PN_TX_PRI_REG)*0x4U))
#define CSL_XGE_CPSW_STATS_ENET_PN_TX_PRI_BCNT_REG(STATS,ENET_PN_TX_PRI_BCNT_REG) (0x0001A1A0U+((STATS)*0x200U)+((ENET_PN_TX_PRI_BCNT_REG)*0x4U))
#define CSL_XGE_CPSW_STATS_ENET_PN_TX_PRI_DROP_REG(STATS,ENET_PN_TX_PRI_DROP_REG) (0x0001A1C0U+((STATS)*0x200U)+((ENET_PN_TX_PRI_DROP_REG)*0x4U))
#define CSL_XGE_CPSW_STATS_ENET_PN_TX_PRI_DROP_BCNT_REG(STATS,ENET_PN_TX_PRI_DROP_BCNT_REG) (0x0001A1E0U+((STATS)*0x200U)+((ENET_PN_TX_PRI_DROP_BCNT_REG)*0x4U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PN_RESERVED_REG */

#define CSL_XGE_CPSW_PN_RESERVED_REG_RESERVED_MASK                    (0xFFFFFFFFU)
#define CSL_XGE_CPSW_PN_RESERVED_REG_RESERVED_SHIFT                   (0x00000000U)
#define CSL_XGE_CPSW_PN_RESERVED_REG_RESERVED_MAX                     (0xFFFFFFFFU)

/* PN_CONTROL_REG */

#define CSL_XGE_CPSW_PN_CONTROL_REG_DSCP_IPV4_EN_MASK                 (0x00000002U)
#define CSL_XGE_CPSW_PN_CONTROL_REG_DSCP_IPV4_EN_SHIFT                (0x00000001U)
#define CSL_XGE_CPSW_PN_CONTROL_REG_DSCP_IPV4_EN_MAX                  (0x00000001U)

#define CSL_XGE_CPSW_PN_CONTROL_REG_DSCP_IPV6_EN_MASK                 (0x00000004U)
#define CSL_XGE_CPSW_PN_CONTROL_REG_DSCP_IPV6_EN_SHIFT                (0x00000002U)
#define CSL_XGE_CPSW_PN_CONTROL_REG_DSCP_IPV6_EN_MAX                  (0x00000001U)

#define CSL_XGE_CPSW_PN_CONTROL_REG_TX_LPI_CLKSTOP_EN_MASK            (0x00001000U)
#define CSL_XGE_CPSW_PN_CONTROL_REG_TX_LPI_CLKSTOP_EN_SHIFT           (0x0000000CU)
#define CSL_XGE_CPSW_PN_CONTROL_REG_TX_LPI_CLKSTOP_EN_MAX             (0x00000001U)

#define CSL_XGE_CPSW_PN_CONTROL_REG_TX_ECC_ERR_EN_MASK                (0x00004000U)
#define CSL_XGE_CPSW_PN_CONTROL_REG_TX_ECC_ERR_EN_SHIFT               (0x0000000EU)
#define CSL_XGE_CPSW_PN_CONTROL_REG_TX_ECC_ERR_EN_MAX                 (0x00000001U)

#define CSL_XGE_CPSW_PN_CONTROL_REG_RX_ECC_ERR_EN_MASK                (0x00008000U)
#define CSL_XGE_CPSW_PN_CONTROL_REG_RX_ECC_ERR_EN_SHIFT               (0x0000000FU)
#define CSL_XGE_CPSW_PN_CONTROL_REG_RX_ECC_ERR_EN_MAX                 (0x00000001U)

#define CSL_XGE_CPSW_PN_CONTROL_REG_IET_PORT_EN_MASK                  (0x00010000U)
#define CSL_XGE_CPSW_PN_CONTROL_REG_IET_PORT_EN_SHIFT                 (0x00000010U)
#define CSL_XGE_CPSW_PN_CONTROL_REG_IET_PORT_EN_MAX                   (0x00000001U)

#define CSL_XGE_CPSW_PN_CONTROL_REG_EST_PORT_EN_MASK                  (0x00020000U)
#define CSL_XGE_CPSW_PN_CONTROL_REG_EST_PORT_EN_SHIFT                 (0x00000011U)
#define CSL_XGE_CPSW_PN_CONTROL_REG_EST_PORT_EN_MAX                   (0x00000001U)

/* PN_MAX_BLKS_REG */

#define CSL_XGE_CPSW_PN_MAX_BLKS_REG_RX_MAX_BLKS_MASK                 (0x000000FFU)
#define CSL_XGE_CPSW_PN_MAX_BLKS_REG_RX_MAX_BLKS_SHIFT                (0x00000000U)
#define CSL_XGE_CPSW_PN_MAX_BLKS_REG_RX_MAX_BLKS_MAX                  (0x000000FFU)

#define CSL_XGE_CPSW_PN_MAX_BLKS_REG_TX_MAX_BLKS_MASK                 (0x0000FF00U)
#define CSL_XGE_CPSW_PN_MAX_BLKS_REG_TX_MAX_BLKS_SHIFT                (0x00000008U)
#define CSL_XGE_CPSW_PN_MAX_BLKS_REG_TX_MAX_BLKS_MAX                  (0x000000FFU)

/* PN_BLK_CNT_REG */

#define CSL_XGE_CPSW_PN_BLK_CNT_REG_RX_BLK_CNT_E_MASK                 (0x0000003FU)
#define CSL_XGE_CPSW_PN_BLK_CNT_REG_RX_BLK_CNT_E_SHIFT                (0x00000000U)
#define CSL_XGE_CPSW_PN_BLK_CNT_REG_RX_BLK_CNT_E_MAX                  (0x0000003FU)

#define CSL_XGE_CPSW_PN_BLK_CNT_REG_TX_BLK_CNT_MASK                   (0x00001F00U)
#define CSL_XGE_CPSW_PN_BLK_CNT_REG_TX_BLK_CNT_SHIFT                  (0x00000008U)
#define CSL_XGE_CPSW_PN_BLK_CNT_REG_TX_BLK_CNT_MAX                    (0x0000001FU)

#define CSL_XGE_CPSW_PN_BLK_CNT_REG_RX_BLK_CNT_P_MASK                 (0x003F0000U)
#define CSL_XGE_CPSW_PN_BLK_CNT_REG_RX_BLK_CNT_P_SHIFT                (0x00000010U)
#define CSL_XGE_CPSW_PN_BLK_CNT_REG_RX_BLK_CNT_P_MAX                  (0x0000003FU)

/* PN_PORT_VLAN_REG */

#define CSL_XGE_CPSW_PN_PORT_VLAN_REG_PORT_VID_MASK                   (0x00000FFFU)
#define CSL_XGE_CPSW_PN_PORT_VLAN_REG_PORT_VID_SHIFT                  (0x00000000U)
#define CSL_XGE_CPSW_PN_PORT_VLAN_REG_PORT_VID_MAX                    (0x00000FFFU)

#define CSL_XGE_CPSW_PN_PORT_VLAN_REG_PORT_CFI_MASK                   (0x00001000U)
#define CSL_XGE_CPSW_PN_PORT_VLAN_REG_PORT_CFI_SHIFT                  (0x0000000CU)
#define CSL_XGE_CPSW_PN_PORT_VLAN_REG_PORT_CFI_MAX                    (0x00000001U)

#define CSL_XGE_CPSW_PN_PORT_VLAN_REG_PORT_PRI_MASK                   (0x0000E000U)
#define CSL_XGE_CPSW_PN_PORT_VLAN_REG_PORT_PRI_SHIFT                  (0x0000000DU)
#define CSL_XGE_CPSW_PN_PORT_VLAN_REG_PORT_PRI_MAX                    (0x00000007U)

/* PN_TX_PRI_MAP_REG */

#define CSL_XGE_CPSW_PN_TX_PRI_MAP_REG_PRI0_MASK                      (0x00000007U)
#define CSL_XGE_CPSW_PN_TX_PRI_MAP_REG_PRI0_SHIFT                     (0x00000000U)
#define CSL_XGE_CPSW_PN_TX_PRI_MAP_REG_PRI0_MAX                       (0x00000007U)

#define CSL_XGE_CPSW_PN_TX_PRI_MAP_REG_PRI1_MASK                      (0x00000070U)
#define CSL_XGE_CPSW_PN_TX_PRI_MAP_REG_PRI1_SHIFT                     (0x00000004U)
#define CSL_XGE_CPSW_PN_TX_PRI_MAP_REG_PRI1_MAX                       (0x00000007U)

#define CSL_XGE_CPSW_PN_TX_PRI_MAP_REG_PRI2_MASK                      (0x00000700U)
#define CSL_XGE_CPSW_PN_TX_PRI_MAP_REG_PRI2_SHIFT                     (0x00000008U)
#define CSL_XGE_CPSW_PN_TX_PRI_MAP_REG_PRI2_MAX                       (0x00000007U)

#define CSL_XGE_CPSW_PN_TX_PRI_MAP_REG_PRI3_MASK                      (0x00007000U)
#define CSL_XGE_CPSW_PN_TX_PRI_MAP_REG_PRI3_SHIFT                     (0x0000000CU)
#define CSL_XGE_CPSW_PN_TX_PRI_MAP_REG_PRI3_MAX                       (0x00000007U)

#define CSL_XGE_CPSW_PN_TX_PRI_MAP_REG_PRI4_MASK                      (0x00070000U)
#define CSL_XGE_CPSW_PN_TX_PRI_MAP_REG_PRI4_SHIFT                     (0x00000010U)
#define CSL_XGE_CPSW_PN_TX_PRI_MAP_REG_PRI4_MAX                       (0x00000007U)

#define CSL_XGE_CPSW_PN_TX_PRI_MAP_REG_PRI5_MASK                      (0x00700000U)
#define CSL_XGE_CPSW_PN_TX_PRI_MAP_REG_PRI5_SHIFT                     (0x00000014U)
#define CSL_XGE_CPSW_PN_TX_PRI_MAP_REG_PRI5_MAX                       (0x00000007U)

#define CSL_XGE_CPSW_PN_TX_PRI_MAP_REG_PRI6_MASK                      (0x07000000U)
#define CSL_XGE_CPSW_PN_TX_PRI_MAP_REG_PRI6_SHIFT                     (0x00000018U)
#define CSL_XGE_CPSW_PN_TX_PRI_MAP_REG_PRI6_MAX                       (0x00000007U)

#define CSL_XGE_CPSW_PN_TX_PRI_MAP_REG_PRI7_MASK                      (0x70000000U)
#define CSL_XGE_CPSW_PN_TX_PRI_MAP_REG_PRI7_SHIFT                     (0x0000001CU)
#define CSL_XGE_CPSW_PN_TX_PRI_MAP_REG_PRI7_MAX                       (0x00000007U)

/* PN_PRI_CTL_REG */

#define CSL_XGE_CPSW_PN_PRI_CTL_REG_TX_HOST_BLKS_REM_MASK             (0x0000F000U)
#define CSL_XGE_CPSW_PN_PRI_CTL_REG_TX_HOST_BLKS_REM_SHIFT            (0x0000000CU)
#define CSL_XGE_CPSW_PN_PRI_CTL_REG_TX_HOST_BLKS_REM_MAX              (0x0000000FU)

#define CSL_XGE_CPSW_PN_PRI_CTL_REG_RX_FLOW_PRI_MASK                  (0x00FF0000U)
#define CSL_XGE_CPSW_PN_PRI_CTL_REG_RX_FLOW_PRI_SHIFT                 (0x00000010U)
#define CSL_XGE_CPSW_PN_PRI_CTL_REG_RX_FLOW_PRI_MAX                   (0x000000FFU)

#define CSL_XGE_CPSW_PN_PRI_CTL_REG_TX_FLOW_PRI_MASK                  (0xFF000000U)
#define CSL_XGE_CPSW_PN_PRI_CTL_REG_TX_FLOW_PRI_SHIFT                 (0x00000018U)
#define CSL_XGE_CPSW_PN_PRI_CTL_REG_TX_FLOW_PRI_MAX                   (0x000000FFU)

/* PN_RX_PRI_MAP_REG */

#define CSL_XGE_CPSW_PN_RX_PRI_MAP_REG_PRI0_MASK                      (0x00000007U)
#define CSL_XGE_CPSW_PN_RX_PRI_MAP_REG_PRI0_SHIFT                     (0x00000000U)
#define CSL_XGE_CPSW_PN_RX_PRI_MAP_REG_PRI0_MAX                       (0x00000007U)

#define CSL_XGE_CPSW_PN_RX_PRI_MAP_REG_PRI1_MASK                      (0x00000070U)
#define CSL_XGE_CPSW_PN_RX_PRI_MAP_REG_PRI1_SHIFT                     (0x00000004U)
#define CSL_XGE_CPSW_PN_RX_PRI_MAP_REG_PRI1_MAX                       (0x00000007U)

#define CSL_XGE_CPSW_PN_RX_PRI_MAP_REG_PRI2_MASK                      (0x00000700U)
#define CSL_XGE_CPSW_PN_RX_PRI_MAP_REG_PRI2_SHIFT                     (0x00000008U)
#define CSL_XGE_CPSW_PN_RX_PRI_MAP_REG_PRI2_MAX                       (0x00000007U)

#define CSL_XGE_CPSW_PN_RX_PRI_MAP_REG_PRI3_MASK                      (0x00007000U)
#define CSL_XGE_CPSW_PN_RX_PRI_MAP_REG_PRI3_SHIFT                     (0x0000000CU)
#define CSL_XGE_CPSW_PN_RX_PRI_MAP_REG_PRI3_MAX                       (0x00000007U)

#define CSL_XGE_CPSW_PN_RX_PRI_MAP_REG_PRI4_MASK                      (0x00070000U)
#define CSL_XGE_CPSW_PN_RX_PRI_MAP_REG_PRI4_SHIFT                     (0x00000010U)
#define CSL_XGE_CPSW_PN_RX_PRI_MAP_REG_PRI4_MAX                       (0x00000007U)

#define CSL_XGE_CPSW_PN_RX_PRI_MAP_REG_PRI5_MASK                      (0x00700000U)
#define CSL_XGE_CPSW_PN_RX_PRI_MAP_REG_PRI5_SHIFT                     (0x00000014U)
#define CSL_XGE_CPSW_PN_RX_PRI_MAP_REG_PRI5_MAX                       (0x00000007U)

#define CSL_XGE_CPSW_PN_RX_PRI_MAP_REG_PRI6_MASK                      (0x07000000U)
#define CSL_XGE_CPSW_PN_RX_PRI_MAP_REG_PRI6_SHIFT                     (0x00000018U)
#define CSL_XGE_CPSW_PN_RX_PRI_MAP_REG_PRI6_MAX                       (0x00000007U)

#define CSL_XGE_CPSW_PN_RX_PRI_MAP_REG_PRI7_MASK                      (0x70000000U)
#define CSL_XGE_CPSW_PN_RX_PRI_MAP_REG_PRI7_SHIFT                     (0x0000001CU)
#define CSL_XGE_CPSW_PN_RX_PRI_MAP_REG_PRI7_MAX                       (0x00000007U)

/* PN_RX_MAXLEN_REG */

#define CSL_XGE_CPSW_PN_RX_MAXLEN_REG_RX_MAXLEN_MASK                  (0x00003FFFU)
#define CSL_XGE_CPSW_PN_RX_MAXLEN_REG_RX_MAXLEN_SHIFT                 (0x00000000U)
#define CSL_XGE_CPSW_PN_RX_MAXLEN_REG_RX_MAXLEN_MAX                   (0x00003FFFU)

/* PN_TX_BLKS_PRI_REG */

#define CSL_XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI0_MASK                     (0x0000000FU)
#define CSL_XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI0_SHIFT                    (0x00000000U)
#define CSL_XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI0_MAX                      (0x0000000FU)

#define CSL_XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI1_MASK                     (0x000000F0U)
#define CSL_XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI1_SHIFT                    (0x00000004U)
#define CSL_XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI1_MAX                      (0x0000000FU)

#define CSL_XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI2_MASK                     (0x00000F00U)
#define CSL_XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI2_SHIFT                    (0x00000008U)
#define CSL_XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI2_MAX                      (0x0000000FU)

#define CSL_XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI3_MASK                     (0x0000F000U)
#define CSL_XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI3_SHIFT                    (0x0000000CU)
#define CSL_XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI3_MAX                      (0x0000000FU)

#define CSL_XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI4_MASK                     (0x000F0000U)
#define CSL_XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI4_SHIFT                    (0x00000010U)
#define CSL_XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI4_MAX                      (0x0000000FU)

#define CSL_XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI5_MASK                     (0x00F00000U)
#define CSL_XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI5_SHIFT                    (0x00000014U)
#define CSL_XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI5_MAX                      (0x0000000FU)

#define CSL_XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI6_MASK                     (0x0F000000U)
#define CSL_XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI6_SHIFT                    (0x00000018U)
#define CSL_XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI6_MAX                      (0x0000000FU)

#define CSL_XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI7_MASK                     (0xF0000000U)
#define CSL_XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI7_SHIFT                    (0x0000001CU)
#define CSL_XGE_CPSW_PN_TX_BLKS_PRI_REG_PRI7_MAX                      (0x0000000FU)

/* PN_RX_FLOW_THRESH_REG */

#define CSL_XGE_CPSW_PN_RX_FLOW_THRESH_REG_COUNT_MASK                 (0x000001FFU)
#define CSL_XGE_CPSW_PN_RX_FLOW_THRESH_REG_COUNT_SHIFT                (0x00000000U)
#define CSL_XGE_CPSW_PN_RX_FLOW_THRESH_REG_COUNT_MAX                  (0x000001FFU)

/* PN_IDLE2LPI_REG */

#define CSL_XGE_CPSW_PN_IDLE2LPI_REG_COUNT_MASK                       (0x00FFFFFFU)
#define CSL_XGE_CPSW_PN_IDLE2LPI_REG_COUNT_SHIFT                      (0x00000000U)
#define CSL_XGE_CPSW_PN_IDLE2LPI_REG_COUNT_MAX                        (0x00FFFFFFU)

/* PN_LPI2WAKE_REG */

#define CSL_XGE_CPSW_PN_LPI2WAKE_REG_COUNT_MASK                       (0x00FFFFFFU)
#define CSL_XGE_CPSW_PN_LPI2WAKE_REG_COUNT_SHIFT                      (0x00000000U)
#define CSL_XGE_CPSW_PN_LPI2WAKE_REG_COUNT_MAX                        (0x00FFFFFFU)

/* PN_EEE_STATUS_REG */

#define CSL_XGE_CPSW_PN_EEE_STATUS_REG_WAIT_IDLE2LPI_MASK             (0x00000001U)
#define CSL_XGE_CPSW_PN_EEE_STATUS_REG_WAIT_IDLE2LPI_SHIFT            (0x00000000U)
#define CSL_XGE_CPSW_PN_EEE_STATUS_REG_WAIT_IDLE2LPI_MAX              (0x00000001U)

#define CSL_XGE_CPSW_PN_EEE_STATUS_REG_RX_LPI_MASK                    (0x00000002U)
#define CSL_XGE_CPSW_PN_EEE_STATUS_REG_RX_LPI_SHIFT                   (0x00000001U)
#define CSL_XGE_CPSW_PN_EEE_STATUS_REG_RX_LPI_MAX                     (0x00000001U)

#define CSL_XGE_CPSW_PN_EEE_STATUS_REG_TX_LPI_MASK                    (0x00000004U)
#define CSL_XGE_CPSW_PN_EEE_STATUS_REG_TX_LPI_SHIFT                   (0x00000002U)
#define CSL_XGE_CPSW_PN_EEE_STATUS_REG_TX_LPI_MAX                     (0x00000001U)

#define CSL_XGE_CPSW_PN_EEE_STATUS_REG_TX_WAKE_MASK                   (0x00000008U)
#define CSL_XGE_CPSW_PN_EEE_STATUS_REG_TX_WAKE_SHIFT                  (0x00000003U)
#define CSL_XGE_CPSW_PN_EEE_STATUS_REG_TX_WAKE_MAX                    (0x00000001U)

#define CSL_XGE_CPSW_PN_EEE_STATUS_REG_TX_FIFO_HOLD_MASK              (0x00000010U)
#define CSL_XGE_CPSW_PN_EEE_STATUS_REG_TX_FIFO_HOLD_SHIFT             (0x00000004U)
#define CSL_XGE_CPSW_PN_EEE_STATUS_REG_TX_FIFO_HOLD_MAX               (0x00000001U)

#define CSL_XGE_CPSW_PN_EEE_STATUS_REG_RX_FIFO_EMPTY_MASK             (0x00000020U)
#define CSL_XGE_CPSW_PN_EEE_STATUS_REG_RX_FIFO_EMPTY_SHIFT            (0x00000005U)
#define CSL_XGE_CPSW_PN_EEE_STATUS_REG_RX_FIFO_EMPTY_MAX              (0x00000001U)

#define CSL_XGE_CPSW_PN_EEE_STATUS_REG_TX_FIFO_EMPTY_MASK             (0x00000040U)
#define CSL_XGE_CPSW_PN_EEE_STATUS_REG_TX_FIFO_EMPTY_SHIFT            (0x00000006U)
#define CSL_XGE_CPSW_PN_EEE_STATUS_REG_TX_FIFO_EMPTY_MAX              (0x00000001U)

/* PN_IET_CONTROL_REG */

#define CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_PENABLE_MASK              (0x00000001U)
#define CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_PENABLE_SHIFT             (0x00000000U)
#define CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_PENABLE_MAX               (0x00000001U)

#define CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_HOLD_MASK                 (0x00000002U)
#define CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_HOLD_SHIFT                (0x00000001U)
#define CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_HOLD_MAX                  (0x00000001U)

#define CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_DISABLEVERIFY_MASK        (0x00000004U)
#define CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_DISABLEVERIFY_SHIFT       (0x00000002U)
#define CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_DISABLEVERIFY_MAX         (0x00000001U)

#define CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_LINKFAIL_MASK             (0x00000008U)
#define CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_LINKFAIL_SHIFT            (0x00000003U)
#define CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_LINKFAIL_MAX              (0x00000001U)

#define CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_ADDFRAGSIZE_MASK          (0x00000700U)
#define CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_ADDFRAGSIZE_SHIFT         (0x00000008U)
#define CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_ADDFRAGSIZE_MAX           (0x00000007U)

#define CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_PREMPT_MASK               (0x00FF0000U)
#define CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_PREMPT_SHIFT              (0x00000010U)
#define CSL_XGE_CPSW_PN_IET_CONTROL_REG_MAC_PREMPT_MAX                (0x000000FFU)

/* PN_IET_STATUS_REG */

#define CSL_XGE_CPSW_PN_IET_STATUS_REG_MAC_VERIFIED_MASK              (0x00000001U)
#define CSL_XGE_CPSW_PN_IET_STATUS_REG_MAC_VERIFIED_SHIFT             (0x00000000U)
#define CSL_XGE_CPSW_PN_IET_STATUS_REG_MAC_VERIFIED_MAX               (0x00000001U)

#define CSL_XGE_CPSW_PN_IET_STATUS_REG_MAC_VERIFY_FAIL_MASK           (0x00000002U)
#define CSL_XGE_CPSW_PN_IET_STATUS_REG_MAC_VERIFY_FAIL_SHIFT          (0x00000001U)
#define CSL_XGE_CPSW_PN_IET_STATUS_REG_MAC_VERIFY_FAIL_MAX            (0x00000001U)

#define CSL_XGE_CPSW_PN_IET_STATUS_REG_MAC_RESPOND_ERR_MASK           (0x00000004U)
#define CSL_XGE_CPSW_PN_IET_STATUS_REG_MAC_RESPOND_ERR_SHIFT          (0x00000002U)
#define CSL_XGE_CPSW_PN_IET_STATUS_REG_MAC_RESPOND_ERR_MAX            (0x00000001U)

#define CSL_XGE_CPSW_PN_IET_STATUS_REG_MAC_VERIFY_ERR_MASK            (0x00000008U)
#define CSL_XGE_CPSW_PN_IET_STATUS_REG_MAC_VERIFY_ERR_SHIFT           (0x00000003U)
#define CSL_XGE_CPSW_PN_IET_STATUS_REG_MAC_VERIFY_ERR_MAX             (0x00000001U)

/* PN_IET_VERIFY_REG */

#define CSL_XGE_CPSW_PN_IET_VERIFY_REG_MAC_VERIFY_CNT_MASK            (0x00FFFFFFU)
#define CSL_XGE_CPSW_PN_IET_VERIFY_REG_MAC_VERIFY_CNT_SHIFT           (0x00000000U)
#define CSL_XGE_CPSW_PN_IET_VERIFY_REG_MAC_VERIFY_CNT_MAX             (0x00FFFFFFU)

/* PN_FIFO_STATUS_REG */

#define CSL_XGE_CPSW_PN_FIFO_STATUS_REG_TX_PRI_ACTIVE_MASK            (0x000000FFU)
#define CSL_XGE_CPSW_PN_FIFO_STATUS_REG_TX_PRI_ACTIVE_SHIFT           (0x00000000U)
#define CSL_XGE_CPSW_PN_FIFO_STATUS_REG_TX_PRI_ACTIVE_MAX             (0x000000FFU)

#define CSL_XGE_CPSW_PN_FIFO_STATUS_REG_TX_E_MAC_ALLOW_MASK           (0x0000FF00U)
#define CSL_XGE_CPSW_PN_FIFO_STATUS_REG_TX_E_MAC_ALLOW_SHIFT          (0x00000008U)
#define CSL_XGE_CPSW_PN_FIFO_STATUS_REG_TX_E_MAC_ALLOW_MAX            (0x000000FFU)

#define CSL_XGE_CPSW_PN_FIFO_STATUS_REG_EST_CNT_ERR_MASK              (0x00010000U)
#define CSL_XGE_CPSW_PN_FIFO_STATUS_REG_EST_CNT_ERR_SHIFT             (0x00000010U)
#define CSL_XGE_CPSW_PN_FIFO_STATUS_REG_EST_CNT_ERR_MAX               (0x00000001U)

#define CSL_XGE_CPSW_PN_FIFO_STATUS_REG_EST_ADD_ERR_MASK              (0x00020000U)
#define CSL_XGE_CPSW_PN_FIFO_STATUS_REG_EST_ADD_ERR_SHIFT             (0x00000011U)
#define CSL_XGE_CPSW_PN_FIFO_STATUS_REG_EST_ADD_ERR_MAX               (0x00000001U)

#define CSL_XGE_CPSW_PN_FIFO_STATUS_REG_EST_BUFACT_MASK               (0x00040000U)
#define CSL_XGE_CPSW_PN_FIFO_STATUS_REG_EST_BUFACT_SHIFT              (0x00000012U)
#define CSL_XGE_CPSW_PN_FIFO_STATUS_REG_EST_BUFACT_MAX                (0x00000001U)

/* PN_EST_CONTROL_REG */

#define CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_ONEBUF_MASK               (0x00000001U)
#define CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_ONEBUF_SHIFT              (0x00000000U)
#define CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_ONEBUF_MAX                (0x00000001U)

#define CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_BUFSEL_MASK               (0x00000002U)
#define CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_BUFSEL_SHIFT              (0x00000001U)
#define CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_BUFSEL_MAX                (0x00000001U)

#define CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_EN_MASK                (0x00000004U)
#define CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_EN_SHIFT               (0x00000002U)
#define CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_EN_MAX                 (0x00000001U)

#define CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_FIRST_MASK             (0x00000008U)
#define CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_FIRST_SHIFT            (0x00000003U)
#define CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_FIRST_MAX              (0x00000001U)

#define CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_ONEPRI_MASK            (0x00000010U)
#define CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_ONEPRI_SHIFT           (0x00000004U)
#define CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_ONEPRI_MAX             (0x00000001U)

#define CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_PRI_MASK               (0x000000E0U)
#define CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_PRI_SHIFT              (0x00000005U)
#define CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_TS_PRI_MAX                (0x00000007U)

#define CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_FILL_EN_MASK              (0x00000100U)
#define CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_FILL_EN_SHIFT             (0x00000008U)
#define CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_FILL_EN_MAX               (0x00000001U)

#define CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_FILL_MARGIN_MASK          (0x03FF0000U)
#define CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_FILL_MARGIN_SHIFT         (0x00000010U)
#define CSL_XGE_CPSW_PN_EST_CONTROL_REG_EST_FILL_MARGIN_MAX           (0x000003FFU)

/* PN_RX_DSCP_MAP_REG */

#define CSL_XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI0_MASK                     (0x00000007U)
#define CSL_XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI0_SHIFT                    (0x00000000U)
#define CSL_XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI0_MAX                      (0x00000007U)

#define CSL_XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI1_MASK                     (0x00000070U)
#define CSL_XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI1_SHIFT                    (0x00000004U)
#define CSL_XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI1_MAX                      (0x00000007U)

#define CSL_XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI2_MASK                     (0x00000700U)
#define CSL_XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI2_SHIFT                    (0x00000008U)
#define CSL_XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI2_MAX                      (0x00000007U)

#define CSL_XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI3_MASK                     (0x00007000U)
#define CSL_XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI3_SHIFT                    (0x0000000CU)
#define CSL_XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI3_MAX                      (0x00000007U)

#define CSL_XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI4_MASK                     (0x00070000U)
#define CSL_XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI4_SHIFT                    (0x00000010U)
#define CSL_XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI4_MAX                      (0x00000007U)

#define CSL_XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI5_MASK                     (0x00700000U)
#define CSL_XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI5_SHIFT                    (0x00000014U)
#define CSL_XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI5_MAX                      (0x00000007U)

#define CSL_XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI6_MASK                     (0x07000000U)
#define CSL_XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI6_SHIFT                    (0x00000018U)
#define CSL_XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI6_MAX                      (0x00000007U)

#define CSL_XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI7_MASK                     (0x70000000U)
#define CSL_XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI7_SHIFT                    (0x0000001CU)
#define CSL_XGE_CPSW_PN_RX_DSCP_MAP_REG_PRI7_MAX                      (0x00000007U)

/* PN_PRI_CIR_REG */

#define CSL_XGE_CPSW_PN_PRI_CIR_REG_PRI_CIR_MASK                      (0x0FFFFFFFU)
#define CSL_XGE_CPSW_PN_PRI_CIR_REG_PRI_CIR_SHIFT                     (0x00000000U)
#define CSL_XGE_CPSW_PN_PRI_CIR_REG_PRI_CIR_MAX                       (0x0FFFFFFFU)

/* PN_PRI_EIR_REG */

#define CSL_XGE_CPSW_PN_PRI_EIR_REG_PRI_EIR_MASK                      (0x0FFFFFFFU)
#define CSL_XGE_CPSW_PN_PRI_EIR_REG_PRI_EIR_SHIFT                     (0x00000000U)
#define CSL_XGE_CPSW_PN_PRI_EIR_REG_PRI_EIR_MAX                       (0x0FFFFFFFU)

/* PN_TX_D_THRESH_SET_L_REG */

#define CSL_XGE_CPSW_PN_TX_D_THRESH_SET_L_REG_PRI0_MASK               (0x0000001FU)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_SET_L_REG_PRI0_SHIFT              (0x00000000U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_SET_L_REG_PRI0_MAX                (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_D_THRESH_SET_L_REG_PRI1_MASK               (0x00001F00U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_SET_L_REG_PRI1_SHIFT              (0x00000008U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_SET_L_REG_PRI1_MAX                (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_D_THRESH_SET_L_REG_PRI2_MASK               (0x001F0000U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_SET_L_REG_PRI2_SHIFT              (0x00000010U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_SET_L_REG_PRI2_MAX                (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_D_THRESH_SET_L_REG_PRI3_MASK               (0x1F000000U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_SET_L_REG_PRI3_SHIFT              (0x00000018U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_SET_L_REG_PRI3_MAX                (0x0000001FU)

/* PN_TX_D_THRESH_SET_H_REG */

#define CSL_XGE_CPSW_PN_TX_D_THRESH_SET_H_REG_PRI4_MASK               (0x0000001FU)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_SET_H_REG_PRI4_SHIFT              (0x00000000U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_SET_H_REG_PRI4_MAX                (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_D_THRESH_SET_H_REG_PRI5_MASK               (0x00001F00U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_SET_H_REG_PRI5_SHIFT              (0x00000008U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_SET_H_REG_PRI5_MAX                (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_D_THRESH_SET_H_REG_PRI6_MASK               (0x001F0000U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_SET_H_REG_PRI6_SHIFT              (0x00000010U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_SET_H_REG_PRI6_MAX                (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_D_THRESH_SET_H_REG_PRI7_MASK               (0x1F000000U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_SET_H_REG_PRI7_SHIFT              (0x00000018U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_SET_H_REG_PRI7_MAX                (0x0000001FU)

/* PN_TX_D_THRESH_CLR_L_REG */

#define CSL_XGE_CPSW_PN_TX_D_THRESH_CLR_L_REG_PRI0_MASK               (0x0000001FU)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_CLR_L_REG_PRI0_SHIFT              (0x00000000U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_CLR_L_REG_PRI0_MAX                (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_D_THRESH_CLR_L_REG_PRI1_MASK               (0x00001F00U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_CLR_L_REG_PRI1_SHIFT              (0x00000008U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_CLR_L_REG_PRI1_MAX                (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_D_THRESH_CLR_L_REG_PRI2_MASK               (0x001F0000U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_CLR_L_REG_PRI2_SHIFT              (0x00000010U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_CLR_L_REG_PRI2_MAX                (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_D_THRESH_CLR_L_REG_PRI3_MASK               (0x1F000000U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_CLR_L_REG_PRI3_SHIFT              (0x00000018U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_CLR_L_REG_PRI3_MAX                (0x0000001FU)

/* PN_TX_D_THRESH_CLR_H_REG */

#define CSL_XGE_CPSW_PN_TX_D_THRESH_CLR_H_REG_PRI4_MASK               (0x0000001FU)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_CLR_H_REG_PRI4_SHIFT              (0x00000000U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_CLR_H_REG_PRI4_MAX                (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_D_THRESH_CLR_H_REG_PRI5_MASK               (0x00001F00U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_CLR_H_REG_PRI5_SHIFT              (0x00000008U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_CLR_H_REG_PRI5_MAX                (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_D_THRESH_CLR_H_REG_PRI6_MASK               (0x001F0000U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_CLR_H_REG_PRI6_SHIFT              (0x00000010U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_CLR_H_REG_PRI6_MAX                (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_D_THRESH_CLR_H_REG_PRI7_MASK               (0x1F000000U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_CLR_H_REG_PRI7_SHIFT              (0x00000018U)
#define CSL_XGE_CPSW_PN_TX_D_THRESH_CLR_H_REG_PRI7_MAX                (0x0000001FU)

/* PN_TX_G_BUF_THRESH_SET_L_REG */

#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_SET_L_REG_PRI0_MASK           (0x0000001FU)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_SET_L_REG_PRI0_SHIFT          (0x00000000U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_SET_L_REG_PRI0_MAX            (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_SET_L_REG_PRI1_MASK           (0x00001F00U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_SET_L_REG_PRI1_SHIFT          (0x00000008U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_SET_L_REG_PRI1_MAX            (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_SET_L_REG_PRI2_MASK           (0x001F0000U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_SET_L_REG_PRI2_SHIFT          (0x00000010U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_SET_L_REG_PRI2_MAX            (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_SET_L_REG_PRI3_MASK           (0x1F000000U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_SET_L_REG_PRI3_SHIFT          (0x00000018U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_SET_L_REG_PRI3_MAX            (0x0000001FU)

/* PN_TX_G_BUF_THRESH_SET_H_REG */

#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_SET_H_REG_PRI4_MASK           (0x0000001FU)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_SET_H_REG_PRI4_SHIFT          (0x00000000U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_SET_H_REG_PRI4_MAX            (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_SET_H_REG_PRI5_MASK           (0x00001F00U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_SET_H_REG_PRI5_SHIFT          (0x00000008U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_SET_H_REG_PRI5_MAX            (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_SET_H_REG_PRI6_MASK           (0x001F0000U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_SET_H_REG_PRI6_SHIFT          (0x00000010U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_SET_H_REG_PRI6_MAX            (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_SET_H_REG_PRI7_MASK           (0x1F000000U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_SET_H_REG_PRI7_SHIFT          (0x00000018U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_SET_H_REG_PRI7_MAX            (0x0000001FU)

/* PN_TX_G_BUF_THRESH_CLR_L_REG */

#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI0_MASK           (0x0000001FU)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI0_SHIFT          (0x00000000U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI0_MAX            (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI1_MASK           (0x00001F00U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI1_SHIFT          (0x00000008U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI1_MAX            (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI2_MASK           (0x001F0000U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI2_SHIFT          (0x00000010U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI2_MAX            (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI3_MASK           (0x1F000000U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI3_SHIFT          (0x00000018U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI3_MAX            (0x0000001FU)

/* PN_TX_G_BUF_THRESH_CLR_H_REG */

#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI4_MASK           (0x0000001FU)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI4_SHIFT          (0x00000000U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI4_MAX            (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI5_MASK           (0x00001F00U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI5_SHIFT          (0x00000008U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI5_MAX            (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI6_MASK           (0x001F0000U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI6_SHIFT          (0x00000010U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI6_MAX            (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI7_MASK           (0x1F000000U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI7_SHIFT          (0x00000018U)
#define CSL_XGE_CPSW_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI7_MAX            (0x0000001FU)

/* PN_TX_D_OFLOW_ADDVAL_L_REG */

#define CSL_XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI0_MASK             (0x0000001FU)
#define CSL_XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI0_SHIFT            (0x00000000U)
#define CSL_XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI0_MAX              (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI1_MASK             (0x00001F00U)
#define CSL_XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI1_SHIFT            (0x00000008U)
#define CSL_XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI1_MAX              (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI2_MASK             (0x001F0000U)
#define CSL_XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI2_SHIFT            (0x00000010U)
#define CSL_XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI2_MAX              (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI3_MASK             (0x1F000000U)
#define CSL_XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI3_SHIFT            (0x00000018U)
#define CSL_XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI3_MAX              (0x0000001FU)

/* PN_TX_D_OFLOW_ADDVAL_H_REG */

#define CSL_XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI4_MASK             (0x0000001FU)
#define CSL_XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI4_SHIFT            (0x00000000U)
#define CSL_XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI4_MAX              (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI5_MASK             (0x00001F00U)
#define CSL_XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI5_SHIFT            (0x00000008U)
#define CSL_XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI5_MAX              (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI6_MASK             (0x001F0000U)
#define CSL_XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI6_SHIFT            (0x00000010U)
#define CSL_XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI6_MAX              (0x0000001FU)

#define CSL_XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI7_MASK             (0x1F000000U)
#define CSL_XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI7_SHIFT            (0x00000018U)
#define CSL_XGE_CPSW_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI7_MAX              (0x0000001FU)

/* PN_SA_L_REG */

#define CSL_XGE_CPSW_PN_SA_L_REG_MACSRCADDR_15_8_MASK                 (0x000000FFU)
#define CSL_XGE_CPSW_PN_SA_L_REG_MACSRCADDR_15_8_SHIFT                (0x00000000U)
#define CSL_XGE_CPSW_PN_SA_L_REG_MACSRCADDR_15_8_MAX                  (0x000000FFU)

#define CSL_XGE_CPSW_PN_SA_L_REG_MACSRCADDR_7_0_MASK                  (0x0000FF00U)
#define CSL_XGE_CPSW_PN_SA_L_REG_MACSRCADDR_7_0_SHIFT                 (0x00000008U)
#define CSL_XGE_CPSW_PN_SA_L_REG_MACSRCADDR_7_0_MAX                   (0x000000FFU)

/* PN_SA_H_REG */

#define CSL_XGE_CPSW_PN_SA_H_REG_MACSRCADDR_47_40_MASK                (0x000000FFU)
#define CSL_XGE_CPSW_PN_SA_H_REG_MACSRCADDR_47_40_SHIFT               (0x00000000U)
#define CSL_XGE_CPSW_PN_SA_H_REG_MACSRCADDR_47_40_MAX                 (0x000000FFU)

#define CSL_XGE_CPSW_PN_SA_H_REG_MACSRCADDR_39_32_MASK                (0x0000FF00U)
#define CSL_XGE_CPSW_PN_SA_H_REG_MACSRCADDR_39_32_SHIFT               (0x00000008U)
#define CSL_XGE_CPSW_PN_SA_H_REG_MACSRCADDR_39_32_MAX                 (0x000000FFU)

#define CSL_XGE_CPSW_PN_SA_H_REG_MACSRCADDR_31_24_MASK                (0x00FF0000U)
#define CSL_XGE_CPSW_PN_SA_H_REG_MACSRCADDR_31_24_SHIFT               (0x00000010U)
#define CSL_XGE_CPSW_PN_SA_H_REG_MACSRCADDR_31_24_MAX                 (0x000000FFU)

#define CSL_XGE_CPSW_PN_SA_H_REG_MACSRCADDR_23_16_MASK                (0xFF000000U)
#define CSL_XGE_CPSW_PN_SA_H_REG_MACSRCADDR_23_16_SHIFT               (0x00000018U)
#define CSL_XGE_CPSW_PN_SA_H_REG_MACSRCADDR_23_16_MAX                 (0x000000FFU)

/* PN_TS_CTL_REG */

#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_F_EN_MASK              (0x00000001U)
#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_F_EN_SHIFT             (0x00000000U)
#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_F_EN_MAX               (0x00000001U)

#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE1_EN_MASK          (0x00000002U)
#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE1_EN_SHIFT         (0x00000001U)
#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE1_EN_MAX           (0x00000001U)

#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE2_EN_MASK          (0x00000004U)
#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE2_EN_SHIFT         (0x00000002U)
#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE2_EN_MAX           (0x00000001U)

#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_D_EN_MASK              (0x00000008U)
#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_D_EN_SHIFT             (0x00000003U)
#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_D_EN_MAX               (0x00000001U)

#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_F_EN_MASK              (0x00000010U)
#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_F_EN_SHIFT             (0x00000004U)
#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_F_EN_MAX               (0x00000001U)

#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE1_EN_MASK          (0x00000020U)
#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE1_EN_SHIFT         (0x00000005U)
#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE1_EN_MAX           (0x00000001U)

#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE2_EN_MASK          (0x00000040U)
#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE2_EN_SHIFT         (0x00000006U)
#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE2_EN_MAX           (0x00000001U)

#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_D_EN_MASK              (0x00000080U)
#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_D_EN_SHIFT             (0x00000007U)
#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_D_EN_MAX               (0x00000001U)

#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_LTYPE2_EN_MASK                  (0x00000100U)
#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_LTYPE2_EN_SHIFT                 (0x00000008U)
#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_LTYPE2_EN_MAX                   (0x00000001U)

#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_E_EN_MASK              (0x00000200U)
#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_E_EN_SHIFT             (0x00000009U)
#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_RX_ANNEX_E_EN_MAX               (0x00000001U)

#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_E_EN_MASK              (0x00000400U)
#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_E_EN_SHIFT             (0x0000000AU)
#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_TX_ANNEX_E_EN_MAX               (0x00000001U)

#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_TX_HOST_TS_EN_MASK              (0x00000800U)
#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_TX_HOST_TS_EN_SHIFT             (0x0000000BU)
#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_TX_HOST_TS_EN_MAX               (0x00000001U)

#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_MSG_TYPE_EN_MASK                (0xFFFF0000U)
#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_MSG_TYPE_EN_SHIFT               (0x00000010U)
#define CSL_XGE_CPSW_PN_TS_CTL_REG_TS_MSG_TYPE_EN_MAX                 (0x0000FFFFU)

/* PN_TS_SEQ_LTYPE_REG */

#define CSL_XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_LTYPE1_MASK               (0x0000FFFFU)
#define CSL_XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_LTYPE1_SHIFT              (0x00000000U)
#define CSL_XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_LTYPE1_MAX                (0x0000FFFFU)

#define CSL_XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_SEQ_ID_OFFSET_MASK        (0x003F0000U)
#define CSL_XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_SEQ_ID_OFFSET_SHIFT       (0x00000010U)
#define CSL_XGE_CPSW_PN_TS_SEQ_LTYPE_REG_TS_SEQ_ID_OFFSET_MAX         (0x0000003FU)

/* PN_TS_VLAN_LTYPE_REG */

#define CSL_XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE1_MASK         (0x0000FFFFU)
#define CSL_XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE1_SHIFT        (0x00000000U)
#define CSL_XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE1_MAX          (0x0000FFFFU)

#define CSL_XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE2_MASK         (0xFFFF0000U)
#define CSL_XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE2_SHIFT        (0x00000010U)
#define CSL_XGE_CPSW_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE2_MAX          (0x0000FFFFU)

/* PN_TS_CTL_LTYPE2_REG */

#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_LTYPE2_MASK              (0x0000FFFFU)
#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_LTYPE2_SHIFT             (0x00000000U)
#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_LTYPE2_MAX               (0x0000FFFFU)

#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_107_MASK                 (0x00010000U)
#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_107_SHIFT                (0x00000010U)
#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_107_MAX                  (0x00000001U)

#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_129_MASK                 (0x00020000U)
#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_129_SHIFT                (0x00000011U)
#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_129_MAX                  (0x00000001U)

#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_130_MASK                 (0x00040000U)
#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_130_SHIFT                (0x00000012U)
#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_130_MAX                  (0x00000001U)

#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_131_MASK                 (0x00080000U)
#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_131_SHIFT                (0x00000013U)
#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_131_MAX                  (0x00000001U)

#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_132_MASK                 (0x00100000U)
#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_132_SHIFT                (0x00000014U)
#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_132_MAX                  (0x00000001U)

#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_319_MASK                 (0x00200000U)
#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_319_SHIFT                (0x00000015U)
#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_319_MAX                  (0x00000001U)

#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_320_MASK                 (0x00400000U)
#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_320_SHIFT                (0x00000016U)
#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_320_MAX                  (0x00000001U)

#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_TTL_NONZERO_MASK         (0x00800000U)
#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_TTL_NONZERO_SHIFT        (0x00000017U)
#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_TTL_NONZERO_MAX          (0x00000001U)

#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_UNI_EN_MASK              (0x01000000U)
#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_UNI_EN_SHIFT             (0x00000018U)
#define CSL_XGE_CPSW_PN_TS_CTL_LTYPE2_REG_TS_UNI_EN_MAX               (0x00000001U)

/* PN_TS_CTL2_REG */

#define CSL_XGE_CPSW_PN_TS_CTL2_REG_TS_MCAST_TYPE_EN_MASK             (0x0000FFFFU)
#define CSL_XGE_CPSW_PN_TS_CTL2_REG_TS_MCAST_TYPE_EN_SHIFT            (0x00000000U)
#define CSL_XGE_CPSW_PN_TS_CTL2_REG_TS_MCAST_TYPE_EN_MAX              (0x0000FFFFU)

#define CSL_XGE_CPSW_PN_TS_CTL2_REG_TS_DOMAIN_OFFSET_MASK             (0x003F0000U)
#define CSL_XGE_CPSW_PN_TS_CTL2_REG_TS_DOMAIN_OFFSET_SHIFT            (0x00000010U)
#define CSL_XGE_CPSW_PN_TS_CTL2_REG_TS_DOMAIN_OFFSET_MAX              (0x0000003FU)

/* PN_MAC_CONTROL_REG */

#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_FULLDUPLEX_MASK               (0x00000001U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_FULLDUPLEX_SHIFT              (0x00000000U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_FULLDUPLEX_MAX                (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_LOOPBACK_MASK                 (0x00000002U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_LOOPBACK_SHIFT                (0x00000001U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_LOOPBACK_MAX                  (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_MTEST_MASK                    (0x00000004U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_MTEST_SHIFT                   (0x00000002U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_MTEST_MAX                     (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_RX_FLOW_EN_MASK               (0x00000008U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_RX_FLOW_EN_SHIFT              (0x00000003U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_RX_FLOW_EN_MAX                (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_TX_FLOW_EN_MASK               (0x00000010U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_TX_FLOW_EN_SHIFT              (0x00000004U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_TX_FLOW_EN_MAX                (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_GMII_EN_MASK                  (0x00000020U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_GMII_EN_SHIFT                 (0x00000005U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_GMII_EN_MAX                   (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_TX_PACE_MASK                  (0x00000040U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_TX_PACE_SHIFT                 (0x00000006U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_TX_PACE_MAX                   (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_GIG_MASK                      (0x00000080U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_GIG_SHIFT                     (0x00000007U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_GIG_MAX                       (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_XGIG_MASK                     (0x00000100U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_XGIG_SHIFT                    (0x00000008U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_XGIG_MAX                      (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_TX_SHORT_GAP_ENABLE_MASK      (0x00000400U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_TX_SHORT_GAP_ENABLE_SHIFT     (0x0000000AU)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_TX_SHORT_GAP_ENABLE_MAX       (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_CMD_IDLE_MASK                 (0x00000800U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_CMD_IDLE_SHIFT                (0x0000000BU)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_CMD_IDLE_MAX                  (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_CRC_TYPE_MASK                 (0x00001000U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_CRC_TYPE_SHIFT                (0x0000000CU)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_CRC_TYPE_MAX                  (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_XGMII_EN_MASK                 (0x00002000U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_XGMII_EN_SHIFT                (0x0000000DU)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_XGMII_EN_MAX                  (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_IFCTL_A_MASK                  (0x00008000U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_IFCTL_A_SHIFT                 (0x0000000FU)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_IFCTL_A_MAX                   (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_IFCTL_B_MASK                  (0x00010000U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_IFCTL_B_SHIFT                 (0x00000010U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_IFCTL_B_MAX                   (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_GIG_FORCE_MASK                (0x00020000U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_GIG_FORCE_SHIFT               (0x00000011U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_GIG_FORCE_MAX                 (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_EXT_EN_MASK                   (0x00040000U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_EXT_EN_SHIFT                  (0x00000012U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_EXT_EN_MAX                    (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_EXT_RX_FLOW_EN_MASK           (0x00080000U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_EXT_RX_FLOW_EN_SHIFT          (0x00000013U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_EXT_RX_FLOW_EN_MAX            (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_EXT_TX_FLOW_EN_MASK           (0x00100000U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_EXT_TX_FLOW_EN_SHIFT          (0x00000014U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_EXT_TX_FLOW_EN_MAX            (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_TX_SHORT_GAP_LIM_EN_MASK      (0x00200000U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_TX_SHORT_GAP_LIM_EN_SHIFT     (0x00000015U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_TX_SHORT_GAP_LIM_EN_MAX       (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_RX_CEF_EN_MASK                (0x00400000U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_RX_CEF_EN_SHIFT               (0x00000016U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_RX_CEF_EN_MAX                 (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_RX_CSF_EN_MASK                (0x00800000U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_RX_CSF_EN_SHIFT               (0x00000017U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_RX_CSF_EN_MAX                 (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_RX_CMF_EN_MASK                (0x01000000U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_RX_CMF_EN_SHIFT               (0x00000018U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_RX_CMF_EN_MAX                 (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_EXT_EN_XGIG_MASK              (0x02000000U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_EXT_EN_XGIG_SHIFT             (0x00000019U)
#define CSL_XGE_CPSW_PN_MAC_CONTROL_REG_EXT_EN_XGIG_MAX               (0x00000001U)

/* PN_MAC_STATUS_REG */

#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_TX_FLOW_ACT_MASK               (0x00000001U)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_TX_FLOW_ACT_SHIFT              (0x00000000U)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_TX_FLOW_ACT_MAX                (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_RX_FLOW_ACT_MASK               (0x00000002U)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_RX_FLOW_ACT_SHIFT              (0x00000001U)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_RX_FLOW_ACT_MAX                (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_EXT_FULLDUPLEX_MASK            (0x00000008U)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_EXT_FULLDUPLEX_SHIFT           (0x00000003U)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_EXT_FULLDUPLEX_MAX             (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_EXT_GIG_MASK                   (0x00000010U)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_EXT_GIG_SHIFT                  (0x00000004U)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_EXT_GIG_MAX                    (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_EXT_TX_FLOW_EN_MASK            (0x00000020U)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_EXT_TX_FLOW_EN_SHIFT           (0x00000005U)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_EXT_TX_FLOW_EN_MAX             (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_EXT_RX_FLOW_EN_MASK            (0x00000040U)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_EXT_RX_FLOW_EN_SHIFT           (0x00000006U)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_EXT_RX_FLOW_EN_MAX             (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_RX_PFC_FLOW_ACT_MASK           (0x0000FF00U)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_RX_PFC_FLOW_ACT_SHIFT          (0x00000008U)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_RX_PFC_FLOW_ACT_MAX            (0x000000FFU)

#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_TX_PFC_FLOW_ACT_MASK           (0x00FF0000U)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_TX_PFC_FLOW_ACT_SHIFT          (0x00000010U)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_TX_PFC_FLOW_ACT_MAX            (0x000000FFU)

#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_TORF_PRI_MASK                  (0x07000000U)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_TORF_PRI_SHIFT                 (0x00000018U)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_TORF_PRI_MAX                   (0x00000007U)

#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_TORF_MASK                      (0x08000000U)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_TORF_SHIFT                     (0x0000001BU)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_TORF_MAX                       (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_MAC_TX_IDLE_MASK               (0x10000000U)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_MAC_TX_IDLE_SHIFT              (0x0000001CU)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_MAC_TX_IDLE_MAX                (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_P_IDLE_MASK                    (0x20000000U)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_P_IDLE_SHIFT                   (0x0000001DU)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_P_IDLE_MAX                     (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_E_IDLE_MASK                    (0x40000000U)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_E_IDLE_SHIFT                   (0x0000001EU)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_E_IDLE_MAX                     (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_IDLE_MASK                      (0x80000000U)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_IDLE_SHIFT                     (0x0000001FU)
#define CSL_XGE_CPSW_PN_MAC_STATUS_REG_IDLE_MAX                       (0x00000001U)

/* PN_MAC_SOFT_RESET_REG */

#define CSL_XGE_CPSW_PN_MAC_SOFT_RESET_REG_SOFT_RESET_MASK            (0x00000001U)
#define CSL_XGE_CPSW_PN_MAC_SOFT_RESET_REG_SOFT_RESET_SHIFT           (0x00000000U)
#define CSL_XGE_CPSW_PN_MAC_SOFT_RESET_REG_SOFT_RESET_MAX             (0x00000001U)

/* PN_MAC_BOFFTEST_REG */

#define CSL_XGE_CPSW_PN_MAC_BOFFTEST_REG_TX_BACKOFF_MASK              (0x000003FFU)
#define CSL_XGE_CPSW_PN_MAC_BOFFTEST_REG_TX_BACKOFF_SHIFT             (0x00000000U)
#define CSL_XGE_CPSW_PN_MAC_BOFFTEST_REG_TX_BACKOFF_MAX               (0x000003FFU)

#define CSL_XGE_CPSW_PN_MAC_BOFFTEST_REG_COLL_COUNT_MASK              (0x0000F000U)
#define CSL_XGE_CPSW_PN_MAC_BOFFTEST_REG_COLL_COUNT_SHIFT             (0x0000000CU)
#define CSL_XGE_CPSW_PN_MAC_BOFFTEST_REG_COLL_COUNT_MAX               (0x0000000FU)

#define CSL_XGE_CPSW_PN_MAC_BOFFTEST_REG_RNDNUM_MASK                  (0x03FF0000U)
#define CSL_XGE_CPSW_PN_MAC_BOFFTEST_REG_RNDNUM_SHIFT                 (0x00000010U)
#define CSL_XGE_CPSW_PN_MAC_BOFFTEST_REG_RNDNUM_MAX                   (0x000003FFU)

#define CSL_XGE_CPSW_PN_MAC_BOFFTEST_REG_PACEVAL_MASK                 (0x7C000000U)
#define CSL_XGE_CPSW_PN_MAC_BOFFTEST_REG_PACEVAL_SHIFT                (0x0000001AU)
#define CSL_XGE_CPSW_PN_MAC_BOFFTEST_REG_PACEVAL_MAX                  (0x0000001FU)

/* PN_MAC_RX_PAUSETIMER_REG */

#define CSL_XGE_CPSW_PN_MAC_RX_PAUSETIMER_REG_RX_PAUSETIMER_MASK      (0x0000FFFFU)
#define CSL_XGE_CPSW_PN_MAC_RX_PAUSETIMER_REG_RX_PAUSETIMER_SHIFT     (0x00000000U)
#define CSL_XGE_CPSW_PN_MAC_RX_PAUSETIMER_REG_RX_PAUSETIMER_MAX       (0x0000FFFFU)

/* PN_MAC_RXN_PAUSETIMER_REG */

#define CSL_XGE_CPSW_PN_MAC_RXN_PAUSETIMER_REG_RX_PAUSETIMER_MASK     (0x0000FFFFU)
#define CSL_XGE_CPSW_PN_MAC_RXN_PAUSETIMER_REG_RX_PAUSETIMER_SHIFT    (0x00000000U)
#define CSL_XGE_CPSW_PN_MAC_RXN_PAUSETIMER_REG_RX_PAUSETIMER_MAX      (0x0000FFFFU)

/* PN_MAC_TX_PAUSETIMER_REG */

#define CSL_XGE_CPSW_PN_MAC_TX_PAUSETIMER_REG_TX_PAUSETIMER_MASK      (0x0000FFFFU)
#define CSL_XGE_CPSW_PN_MAC_TX_PAUSETIMER_REG_TX_PAUSETIMER_SHIFT     (0x00000000U)
#define CSL_XGE_CPSW_PN_MAC_TX_PAUSETIMER_REG_TX_PAUSETIMER_MAX       (0x0000FFFFU)

/* PN_MAC_TXN_PAUSETIMER_REG */

#define CSL_XGE_CPSW_PN_MAC_TXN_PAUSETIMER_REG_TX_PAUSETIMER_MASK     (0x0000FFFFU)
#define CSL_XGE_CPSW_PN_MAC_TXN_PAUSETIMER_REG_TX_PAUSETIMER_SHIFT    (0x00000000U)
#define CSL_XGE_CPSW_PN_MAC_TXN_PAUSETIMER_REG_TX_PAUSETIMER_MAX      (0x0000FFFFU)

/* PN_MAC_EMCONTROL_REG */

#define CSL_XGE_CPSW_PN_MAC_EMCONTROL_REG_FREE_MASK                   (0x00000001U)
#define CSL_XGE_CPSW_PN_MAC_EMCONTROL_REG_FREE_SHIFT                  (0x00000000U)
#define CSL_XGE_CPSW_PN_MAC_EMCONTROL_REG_FREE_MAX                    (0x00000001U)

#define CSL_XGE_CPSW_PN_MAC_EMCONTROL_REG_SOFT_MASK                   (0x00000002U)
#define CSL_XGE_CPSW_PN_MAC_EMCONTROL_REG_SOFT_SHIFT                  (0x00000001U)
#define CSL_XGE_CPSW_PN_MAC_EMCONTROL_REG_SOFT_MAX                    (0x00000001U)

/* PN_MAC_TX_GAP_REG */

#define CSL_XGE_CPSW_PN_MAC_TX_GAP_REG_TX_GAP_MASK                    (0x0000FFFFU)
#define CSL_XGE_CPSW_PN_MAC_TX_GAP_REG_TX_GAP_SHIFT                   (0x00000000U)
#define CSL_XGE_CPSW_PN_MAC_TX_GAP_REG_TX_GAP_MAX                     (0x0000FFFFU)

/* PN_INTERVLAN_OPX_POINTER_REG */

#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_POINTER_REG_POINTER_MASK        (0x00000007U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_POINTER_REG_POINTER_SHIFT       (0x00000000U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_POINTER_REG_POINTER_MAX         (0x00000007U)

/* PN_INTERVLAN_OPX_A_REG */

#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_A_REG_DA_47_40_MASK             (0x000000FFU)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_A_REG_DA_47_40_SHIFT            (0x00000000U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_A_REG_DA_47_40_MAX              (0x000000FFU)

#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_A_REG_DA_39_32_MASK             (0x0000FF00U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_A_REG_DA_39_32_SHIFT            (0x00000008U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_A_REG_DA_39_32_MAX              (0x000000FFU)

#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_A_REG_DA_31_24_MASK             (0x00FF0000U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_A_REG_DA_31_24_SHIFT            (0x00000010U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_A_REG_DA_31_24_MAX              (0x000000FFU)

#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_A_REG_DA_23_16_MASK             (0xFF000000U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_A_REG_DA_23_16_SHIFT            (0x00000018U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_A_REG_DA_23_16_MAX              (0x000000FFU)

/* PN_INTERVLAN_OPX_B_REG */

#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_B_REG_DA_15_8_MASK              (0x000000FFU)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_B_REG_DA_15_8_SHIFT             (0x00000000U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_B_REG_DA_15_8_MAX               (0x000000FFU)

#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_B_REG_DA_7_0_MASK               (0x0000FF00U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_B_REG_DA_7_0_SHIFT              (0x00000008U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_B_REG_DA_7_0_MAX                (0x000000FFU)

#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_B_REG_SA_47_40_MASK             (0x00FF0000U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_B_REG_SA_47_40_SHIFT            (0x00000010U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_B_REG_SA_47_40_MAX              (0x000000FFU)

#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_B_REG_SA_39_32_MASK             (0xFF000000U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_B_REG_SA_39_32_SHIFT            (0x00000018U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_B_REG_SA_39_32_MAX              (0x000000FFU)

/* PN_INTERVLAN_OPX_C_REG */

#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_C_REG_SA_31_24_MASK             (0x000000FFU)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_C_REG_SA_31_24_SHIFT            (0x00000000U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_C_REG_SA_31_24_MAX              (0x000000FFU)

#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_C_REG_SA_23_16_MASK             (0x0000FF00U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_C_REG_SA_23_16_SHIFT            (0x00000008U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_C_REG_SA_23_16_MAX              (0x000000FFU)

#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_C_REG_SA_15_8_MASK              (0x00FF0000U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_C_REG_SA_15_8_SHIFT             (0x00000010U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_C_REG_SA_15_8_MAX               (0x000000FFU)

#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_C_REG_SA_7_0_MASK               (0xFF000000U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_C_REG_SA_7_0_SHIFT              (0x00000018U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_C_REG_SA_7_0_MAX                (0x000000FFU)

/* PN_INTERVLAN_OPX_D_REG */

#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_D_REG_VID_MASK                  (0x00000FFFU)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_D_REG_VID_SHIFT                 (0x00000000U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_D_REG_VID_MAX                   (0x00000FFFU)

#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_D_REG_REPLACE_VID_MASK          (0x00001000U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_D_REG_REPLACE_VID_SHIFT         (0x0000000CU)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_D_REG_REPLACE_VID_MAX           (0x00000001U)

#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_D_REG_REPLACE_DA_SA_MASK        (0x00002000U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_D_REG_REPLACE_DA_SA_SHIFT       (0x0000000DU)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_D_REG_REPLACE_DA_SA_MAX         (0x00000001U)

#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_D_REG_DEST_FORCE_UNTAGGED_EGRESS_MASK (0x00004000U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_D_REG_DEST_FORCE_UNTAGGED_EGRESS_SHIFT (0x0000000EU)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_D_REG_DEST_FORCE_UNTAGGED_EGRESS_MAX (0x00000001U)

#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_D_REG_DECREMENT_TTL_MASK        (0x00008000U)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_D_REG_DECREMENT_TTL_SHIFT       (0x0000000FU)
#define CSL_XGE_CPSW_PN_INTERVLAN_OPX_D_REG_DECREMENT_TTL_MAX         (0x00000001U)

/* FETCH_LOC */

#define CSL_XGE_CPSW_CPSW_NU_EST_FETCH_LOC_LOC_MASK                            (0x003FFFFFU)
#define CSL_XGE_CPSW_CPSW_NU_EST_FETCH_LOC_LOC_SHIFT                           (0x00000000U)
#define CSL_XGE_CPSW_CPSW_NU_EST_FETCH_LOC_LOC_MAX                             (0x003FFFFFU)

/* RXGOODFRAMES */

#define CSL_XGE_CPSW_STATS_RXGOODFRAMES_COUNT_MASK                             (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_RXGOODFRAMES_COUNT_SHIFT                            (0x00000000U)
#define CSL_XGE_CPSW_STATS_RXGOODFRAMES_COUNT_MAX                              (0xFFFFFFFFU)

/* RXBROADCASTFRAMES */

#define CSL_XGE_CPSW_STATS_RXBROADCASTFRAMES_COUNT_MASK                        (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_RXBROADCASTFRAMES_COUNT_SHIFT                       (0x00000000U)
#define CSL_XGE_CPSW_STATS_RXBROADCASTFRAMES_COUNT_MAX                         (0xFFFFFFFFU)

/* RXMULTICASTFRAMES */

#define CSL_XGE_CPSW_STATS_RXMULTICASTFRAMES_COUNT_MASK                        (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_RXMULTICASTFRAMES_COUNT_SHIFT                       (0x00000000U)
#define CSL_XGE_CPSW_STATS_RXMULTICASTFRAMES_COUNT_MAX                         (0xFFFFFFFFU)

/* RXPAUSEFRAMES */

#define CSL_XGE_CPSW_STATS_RXPAUSEFRAMES_COUNT_MASK                            (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_RXPAUSEFRAMES_COUNT_SHIFT                           (0x00000000U)
#define CSL_XGE_CPSW_STATS_RXPAUSEFRAMES_COUNT_MAX                             (0xFFFFFFFFU)

/* RXCRCERRORS */

#define CSL_XGE_CPSW_STATS_RXCRCERRORS_COUNT_MASK                              (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_RXCRCERRORS_COUNT_SHIFT                             (0x00000000U)
#define CSL_XGE_CPSW_STATS_RXCRCERRORS_COUNT_MAX                               (0xFFFFFFFFU)

/* RXALIGNCODEERRORS */

#define CSL_XGE_CPSW_STATS_RXALIGNCODEERRORS_COUNT_MASK                        (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_RXALIGNCODEERRORS_COUNT_SHIFT                       (0x00000000U)
#define CSL_XGE_CPSW_STATS_RXALIGNCODEERRORS_COUNT_MAX                         (0xFFFFFFFFU)

/* RXOVERSIZEDFRAMES */

#define CSL_XGE_CPSW_STATS_RXOVERSIZEDFRAMES_COUNT_MASK                        (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_RXOVERSIZEDFRAMES_COUNT_SHIFT                       (0x00000000U)
#define CSL_XGE_CPSW_STATS_RXOVERSIZEDFRAMES_COUNT_MAX                         (0xFFFFFFFFU)

/* RXJABBERFRAMES */

#define CSL_XGE_CPSW_STATS_RXJABBERFRAMES_COUNT_MASK                           (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_RXJABBERFRAMES_COUNT_SHIFT                          (0x00000000U)
#define CSL_XGE_CPSW_STATS_RXJABBERFRAMES_COUNT_MAX                            (0xFFFFFFFFU)

/* RXUNDERSIZEDFRAMES */

#define CSL_XGE_CPSW_STATS_RXUNDERSIZEDFRAMES_COUNT_MASK                       (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_RXUNDERSIZEDFRAMES_COUNT_SHIFT                      (0x00000000U)
#define CSL_XGE_CPSW_STATS_RXUNDERSIZEDFRAMES_COUNT_MAX                        (0xFFFFFFFFU)

/* RXFRAGMENTS */

#define CSL_XGE_CPSW_STATS_RXFRAGMENTS_COUNT_MASK                              (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_RXFRAGMENTS_COUNT_SHIFT                             (0x00000000U)
#define CSL_XGE_CPSW_STATS_RXFRAGMENTS_COUNT_MAX                               (0xFFFFFFFFU)

/* ALE_DROP */

#define CSL_XGE_CPSW_STATS_ALE_DROP_COUNT_MASK                                 (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_ALE_DROP_COUNT_SHIFT                                (0x00000000U)
#define CSL_XGE_CPSW_STATS_ALE_DROP_COUNT_MAX                                  (0xFFFFFFFFU)

/* ALE_OVERRUN_DROP */

#define CSL_XGE_CPSW_STATS_ALE_OVERRUN_DROP_COUNT_MASK                         (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_ALE_OVERRUN_DROP_COUNT_SHIFT                        (0x00000000U)
#define CSL_XGE_CPSW_STATS_ALE_OVERRUN_DROP_COUNT_MAX                          (0xFFFFFFFFU)

/* RXOCTETS */

#define CSL_XGE_CPSW_STATS_RXOCTETS_COUNT_MASK                                 (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_RXOCTETS_COUNT_SHIFT                                (0x00000000U)
#define CSL_XGE_CPSW_STATS_RXOCTETS_COUNT_MAX                                  (0xFFFFFFFFU)

/* TXGOODFRAMES */

#define CSL_XGE_CPSW_STATS_TXGOODFRAMES_COUNT_MASK                             (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_TXGOODFRAMES_COUNT_SHIFT                            (0x00000000U)
#define CSL_XGE_CPSW_STATS_TXGOODFRAMES_COUNT_MAX                              (0xFFFFFFFFU)

/* TXBROADCASTFRAMES */

#define CSL_XGE_CPSW_STATS_TXBROADCASTFRAMES_COUNT_MASK                        (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_TXBROADCASTFRAMES_COUNT_SHIFT                       (0x00000000U)
#define CSL_XGE_CPSW_STATS_TXBROADCASTFRAMES_COUNT_MAX                         (0xFFFFFFFFU)

/* TXMULTICASTFRAMES */

#define CSL_XGE_CPSW_STATS_TXMULTICASTFRAMES_COUNT_MASK                        (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_TXMULTICASTFRAMES_COUNT_SHIFT                       (0x00000000U)
#define CSL_XGE_CPSW_STATS_TXMULTICASTFRAMES_COUNT_MAX                         (0xFFFFFFFFU)

/* TXPAUSEFRAMES */

#define CSL_XGE_CPSW_STATS_TXPAUSEFRAMES_COUNT_MASK                            (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_TXPAUSEFRAMES_COUNT_SHIFT                           (0x00000000U)
#define CSL_XGE_CPSW_STATS_TXPAUSEFRAMES_COUNT_MAX                             (0xFFFFFFFFU)

/* TXDEFERREDFRAMES */

#define CSL_XGE_CPSW_STATS_TXDEFERREDFRAMES_COUNT_MASK                         (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_TXDEFERREDFRAMES_COUNT_SHIFT                        (0x00000000U)
#define CSL_XGE_CPSW_STATS_TXDEFERREDFRAMES_COUNT_MAX                          (0xFFFFFFFFU)

/* TXCOLLISIONFRAMES */

#define CSL_XGE_CPSW_STATS_TXCOLLISIONFRAMES_COUNT_MASK                        (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_TXCOLLISIONFRAMES_COUNT_SHIFT                       (0x00000000U)
#define CSL_XGE_CPSW_STATS_TXCOLLISIONFRAMES_COUNT_MAX                         (0xFFFFFFFFU)

/* TXSINGLECOLLFRAMES */

#define CSL_XGE_CPSW_STATS_TXSINGLECOLLFRAMES_COUNT_MASK                       (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_TXSINGLECOLLFRAMES_COUNT_SHIFT                      (0x00000000U)
#define CSL_XGE_CPSW_STATS_TXSINGLECOLLFRAMES_COUNT_MAX                        (0xFFFFFFFFU)

/* TXMULTCOLLFRAMES */

#define CSL_XGE_CPSW_STATS_TXMULTCOLLFRAMES_COUNT_MASK                         (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_TXMULTCOLLFRAMES_COUNT_SHIFT                        (0x00000000U)
#define CSL_XGE_CPSW_STATS_TXMULTCOLLFRAMES_COUNT_MAX                          (0xFFFFFFFFU)

/* TXEXCESSIVECOLLISIONS */

#define CSL_XGE_CPSW_STATS_TXEXCESSIVECOLLISIONS_COUNT_MASK                    (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_TXEXCESSIVECOLLISIONS_COUNT_SHIFT                   (0x00000000U)
#define CSL_XGE_CPSW_STATS_TXEXCESSIVECOLLISIONS_COUNT_MAX                     (0xFFFFFFFFU)

/* TXLATECOLLISIONS */

#define CSL_XGE_CPSW_STATS_TXLATECOLLISIONS_COUNT_MASK                         (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_TXLATECOLLISIONS_COUNT_SHIFT                        (0x00000000U)
#define CSL_XGE_CPSW_STATS_TXLATECOLLISIONS_COUNT_MAX                          (0xFFFFFFFFU)

/* RXIPGERROR */

#define CSL_XGE_CPSW_STATS_RXIPGERROR_COUNT_MASK                               (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_RXIPGERROR_COUNT_SHIFT                              (0x00000000U)
#define CSL_XGE_CPSW_STATS_RXIPGERROR_COUNT_MAX                                (0xFFFFFFFFU)

/* TXCARRIERSENSEERRORS */

#define CSL_XGE_CPSW_STATS_TXCARRIERSENSEERRORS_COUNT_MASK                     (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_TXCARRIERSENSEERRORS_COUNT_SHIFT                    (0x00000000U)
#define CSL_XGE_CPSW_STATS_TXCARRIERSENSEERRORS_COUNT_MAX                      (0xFFFFFFFFU)

/* TXOCTETS */

#define CSL_XGE_CPSW_STATS_TXOCTETS_COUNT_MASK                                 (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_TXOCTETS_COUNT_SHIFT                                (0x00000000U)
#define CSL_XGE_CPSW_STATS_TXOCTETS_COUNT_MAX                                  (0xFFFFFFFFU)

/* OCTETFRAMES64 */

#define CSL_XGE_CPSW_STATS_OCTETFRAMES64_COUNT_MASK                            (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_OCTETFRAMES64_COUNT_SHIFT                           (0x00000000U)
#define CSL_XGE_CPSW_STATS_OCTETFRAMES64_COUNT_MAX                             (0xFFFFFFFFU)

/* OCTETFRAMES65T127 */

#define CSL_XGE_CPSW_STATS_OCTETFRAMES65T127_COUNT_MASK                        (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_OCTETFRAMES65T127_COUNT_SHIFT                       (0x00000000U)
#define CSL_XGE_CPSW_STATS_OCTETFRAMES65T127_COUNT_MAX                         (0xFFFFFFFFU)

/* OCTETFRAMES128T255 */

#define CSL_XGE_CPSW_STATS_OCTETFRAMES128T255_COUNT_MASK                       (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_OCTETFRAMES128T255_COUNT_SHIFT                      (0x00000000U)
#define CSL_XGE_CPSW_STATS_OCTETFRAMES128T255_COUNT_MAX                        (0xFFFFFFFFU)

/* OCTETFRAMES256T511 */

#define CSL_XGE_CPSW_STATS_OCTETFRAMES256T511_COUNT_MASK                       (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_OCTETFRAMES256T511_COUNT_SHIFT                      (0x00000000U)
#define CSL_XGE_CPSW_STATS_OCTETFRAMES256T511_COUNT_MAX                        (0xFFFFFFFFU)

/* OCTETFRAMES512T1023 */

#define CSL_XGE_CPSW_STATS_OCTETFRAMES512T1023_COUNT_MASK                      (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_OCTETFRAMES512T1023_COUNT_SHIFT                     (0x00000000U)
#define CSL_XGE_CPSW_STATS_OCTETFRAMES512T1023_COUNT_MAX                       (0xFFFFFFFFU)

/* OCTETFRAMES1024TUP */

#define CSL_XGE_CPSW_STATS_OCTETFRAMES1024TUP_COUNT_MASK                       (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_OCTETFRAMES1024TUP_COUNT_SHIFT                      (0x00000000U)
#define CSL_XGE_CPSW_STATS_OCTETFRAMES1024TUP_COUNT_MAX                        (0xFFFFFFFFU)

/* NETOCTETS */

#define CSL_XGE_CPSW_STATS_NETOCTETS_COUNT_MASK                                (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_NETOCTETS_COUNT_SHIFT                               (0x00000000U)
#define CSL_XGE_CPSW_STATS_NETOCTETS_COUNT_MAX                                 (0xFFFFFFFFU)

/* RX_BOTTOM_OF_FIFO_DROP */

#define CSL_XGE_CPSW_STATS_RX_BOTTOM_OF_FIFO_DROP_COUNT_MASK                   (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_RX_BOTTOM_OF_FIFO_DROP_COUNT_SHIFT                  (0x00000000U)
#define CSL_XGE_CPSW_STATS_RX_BOTTOM_OF_FIFO_DROP_COUNT_MAX                    (0xFFFFFFFFU)

/* PORTMASK_DROP */

#define CSL_XGE_CPSW_STATS_PORTMASK_DROP_COUNT_MASK                            (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_PORTMASK_DROP_COUNT_SHIFT                           (0x00000000U)
#define CSL_XGE_CPSW_STATS_PORTMASK_DROP_COUNT_MAX                             (0xFFFFFFFFU)

/* RX_TOP_OF_FIFO_DROP */

#define CSL_XGE_CPSW_STATS_RX_TOP_OF_FIFO_DROP_COUNT_MASK                      (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_RX_TOP_OF_FIFO_DROP_COUNT_SHIFT                     (0x00000000U)
#define CSL_XGE_CPSW_STATS_RX_TOP_OF_FIFO_DROP_COUNT_MAX                       (0xFFFFFFFFU)

/* ALE_RATE_LIMIT_DROP */

#define CSL_XGE_CPSW_STATS_ALE_RATE_LIMIT_DROP_COUNT_MASK                      (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_ALE_RATE_LIMIT_DROP_COUNT_SHIFT                     (0x00000000U)
#define CSL_XGE_CPSW_STATS_ALE_RATE_LIMIT_DROP_COUNT_MAX                       (0xFFFFFFFFU)

/* ALE_VID_INGRESS_DROP */

#define CSL_XGE_CPSW_STATS_ALE_VID_INGRESS_DROP_COUNT_MASK                     (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_ALE_VID_INGRESS_DROP_COUNT_SHIFT                    (0x00000000U)
#define CSL_XGE_CPSW_STATS_ALE_VID_INGRESS_DROP_COUNT_MAX                      (0xFFFFFFFFU)

/* ALE_DA_EQ_SA_DROP */

#define CSL_XGE_CPSW_STATS_ALE_DA_EQ_SA_DROP_COUNT_MASK                        (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_ALE_DA_EQ_SA_DROP_COUNT_SHIFT                       (0x00000000U)
#define CSL_XGE_CPSW_STATS_ALE_DA_EQ_SA_DROP_COUNT_MAX                         (0xFFFFFFFFU)

/* ALE_BLOCK_DROP */

#define CSL_XGE_CPSW_STATS_ALE_BLOCK_DROP_COUNT_MASK                           (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_ALE_BLOCK_DROP_COUNT_SHIFT                          (0x00000000U)
#define CSL_XGE_CPSW_STATS_ALE_BLOCK_DROP_COUNT_MAX                            (0xFFFFFFFFU)

/* ALE_SECURE_DROP */

#define CSL_XGE_CPSW_STATS_ALE_SECURE_DROP_COUNT_MASK                          (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_ALE_SECURE_DROP_COUNT_SHIFT                         (0x00000000U)
#define CSL_XGE_CPSW_STATS_ALE_SECURE_DROP_COUNT_MAX                           (0xFFFFFFFFU)

/* ALE_AUTH_DROP */

#define CSL_XGE_CPSW_STATS_ALE_AUTH_DROP_COUNT_MASK                            (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_ALE_AUTH_DROP_COUNT_SHIFT                           (0x00000000U)
#define CSL_XGE_CPSW_STATS_ALE_AUTH_DROP_COUNT_MAX                             (0xFFFFFFFFU)

/* ALE_UNKN_UNI */

#define CSL_XGE_CPSW_STATS_ALE_UNKN_UNI_COUNT_MASK                             (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_ALE_UNKN_UNI_COUNT_SHIFT                            (0x00000000U)
#define CSL_XGE_CPSW_STATS_ALE_UNKN_UNI_COUNT_MAX                              (0xFFFFFFFFU)

/* ALE_UNKN_UNI_BCNT */

#define CSL_XGE_CPSW_STATS_ALE_UNKN_UNI_BCNT_COUNT_MASK                        (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_ALE_UNKN_UNI_BCNT_COUNT_SHIFT                       (0x00000000U)
#define CSL_XGE_CPSW_STATS_ALE_UNKN_UNI_BCNT_COUNT_MAX                         (0xFFFFFFFFU)

/* ALE_UNKN_MLT */

#define CSL_XGE_CPSW_STATS_ALE_UNKN_MLT_COUNT_MASK                             (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_ALE_UNKN_MLT_COUNT_SHIFT                            (0x00000000U)
#define CSL_XGE_CPSW_STATS_ALE_UNKN_MLT_COUNT_MAX                              (0xFFFFFFFFU)

/* ALE_UNKN_MLT_BCNT */

#define CSL_XGE_CPSW_STATS_ALE_UNKN_MLT_BCNT_COUNT_MASK                        (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_ALE_UNKN_MLT_BCNT_COUNT_SHIFT                       (0x00000000U)
#define CSL_XGE_CPSW_STATS_ALE_UNKN_MLT_BCNT_COUNT_MAX                         (0xFFFFFFFFU)

/* ALE_UNKN_BRD */

#define CSL_XGE_CPSW_STATS_ALE_UNKN_BRD_COUNT_MASK                             (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_ALE_UNKN_BRD_COUNT_SHIFT                            (0x00000000U)
#define CSL_XGE_CPSW_STATS_ALE_UNKN_BRD_COUNT_MAX                              (0xFFFFFFFFU)

/* ALE_UNKN_BRD_BCNT */

#define CSL_XGE_CPSW_STATS_ALE_UNKN_BRD_BCNT_COUNT_MASK                        (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_ALE_UNKN_BRD_BCNT_COUNT_SHIFT                       (0x00000000U)
#define CSL_XGE_CPSW_STATS_ALE_UNKN_BRD_BCNT_COUNT_MAX                         (0xFFFFFFFFU)

/* ALE_POL_MATCH */

#define CSL_XGE_CPSW_STATS_ALE_POL_MATCH_COUNT_MASK                            (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_ALE_POL_MATCH_COUNT_SHIFT                           (0x00000000U)
#define CSL_XGE_CPSW_STATS_ALE_POL_MATCH_COUNT_MAX                             (0xFFFFFFFFU)

/* ALE_POL_MATCH_RED */

#define CSL_XGE_CPSW_STATS_ALE_POL_MATCH_RED_COUNT_MASK                        (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_ALE_POL_MATCH_RED_COUNT_SHIFT                       (0x00000000U)
#define CSL_XGE_CPSW_STATS_ALE_POL_MATCH_RED_COUNT_MAX                         (0xFFFFFFFFU)

/* ALE_POL_MATCH_YELLOW */

#define CSL_XGE_CPSW_STATS_ALE_POL_MATCH_YELLOW_COUNT_MASK                     (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_ALE_POL_MATCH_YELLOW_COUNT_SHIFT                    (0x00000000U)
#define CSL_XGE_CPSW_STATS_ALE_POL_MATCH_YELLOW_COUNT_MAX                      (0xFFFFFFFFU)

/* ALE_MULT_SA_DROP */

#define CSL_XGE_CPSW_STATS_ALE_MULT_SA_DROP_COUNT_MASK                         (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_ALE_MULT_SA_DROP_COUNT_SHIFT                        (0x00000000U)
#define CSL_XGE_CPSW_STATS_ALE_MULT_SA_DROP_COUNT_MAX                          (0xFFFFFFFFU)

/* ALE_DUAL_VLAN_DROP */

#define CSL_XGE_CPSW_STATS_ALE_DUAL_VLAN_DROP_COUNT_MASK                       (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_ALE_DUAL_VLAN_DROP_COUNT_SHIFT                      (0x00000000U)
#define CSL_XGE_CPSW_STATS_ALE_DUAL_VLAN_DROP_COUNT_MAX                        (0xFFFFFFFFU)

/* ALE_LEN_ERROR_DROP */

#define CSL_XGE_CPSW_STATS_ALE_LEN_ERROR_DROP_COUNT_MASK                       (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_ALE_LEN_ERROR_DROP_COUNT_SHIFT                      (0x00000000U)
#define CSL_XGE_CPSW_STATS_ALE_LEN_ERROR_DROP_COUNT_MAX                        (0xFFFFFFFFU)

/* ALE_IP_NEXT_HDR_DROP */

#define CSL_XGE_CPSW_STATS_ALE_IP_NEXT_HDR_DROP_COUNT_MASK                     (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_ALE_IP_NEXT_HDR_DROP_COUNT_SHIFT                    (0x00000000U)
#define CSL_XGE_CPSW_STATS_ALE_IP_NEXT_HDR_DROP_COUNT_MAX                      (0xFFFFFFFFU)

/* ALE_IPV4_FRAG_DROP */

#define CSL_XGE_CPSW_STATS_ALE_IPV4_FRAG_DROP_COUNT_MASK                       (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_ALE_IPV4_FRAG_DROP_COUNT_SHIFT                      (0x00000000U)
#define CSL_XGE_CPSW_STATS_ALE_IPV4_FRAG_DROP_COUNT_MAX                        (0xFFFFFFFFU)

/* IET_RX_ASSEMBLY_ERROR_REG */

#define CSL_XGE_CPSW_STATS_IET_RX_ASSEMBLY_ERROR_REG_IET_RX_ASSEMBLY_ERROR_MASK (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_IET_RX_ASSEMBLY_ERROR_REG_IET_RX_ASSEMBLY_ERROR_SHIFT (0x00000000U)
#define CSL_XGE_CPSW_STATS_IET_RX_ASSEMBLY_ERROR_REG_IET_RX_ASSEMBLY_ERROR_MAX (0xFFFFFFFFU)

/* IET_RX_ASSEMBLY_OK_REG */

#define CSL_XGE_CPSW_STATS_IET_RX_ASSEMBLY_OK_REG_IET_RX_ASSEMBLY_OK_MASK      (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_IET_RX_ASSEMBLY_OK_REG_IET_RX_ASSEMBLY_OK_SHIFT     (0x00000000U)
#define CSL_XGE_CPSW_STATS_IET_RX_ASSEMBLY_OK_REG_IET_RX_ASSEMBLY_OK_MAX       (0xFFFFFFFFU)

/* IET_RX_SMD_ERROR_REG */

#define CSL_XGE_CPSW_STATS_IET_RX_SMD_ERROR_REG_IET_RX_SMD_ERROR_MASK          (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_IET_RX_SMD_ERROR_REG_IET_RX_SMD_ERROR_SHIFT         (0x00000000U)
#define CSL_XGE_CPSW_STATS_IET_RX_SMD_ERROR_REG_IET_RX_SMD_ERROR_MAX           (0xFFFFFFFFU)

/* IET_RX_FRAG_REG */

#define CSL_XGE_CPSW_STATS_IET_RX_FRAG_REG_IET_RX_FRAG_MASK                    (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_IET_RX_FRAG_REG_IET_RX_FRAG_SHIFT                   (0x00000000U)
#define CSL_XGE_CPSW_STATS_IET_RX_FRAG_REG_IET_RX_FRAG_MAX                     (0xFFFFFFFFU)

/* IET_TX_HOLD_REG */

#define CSL_XGE_CPSW_STATS_IET_TX_HOLD_REG_IET_TX_HOLD_MASK                    (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_IET_TX_HOLD_REG_IET_TX_HOLD_SHIFT                   (0x00000000U)
#define CSL_XGE_CPSW_STATS_IET_TX_HOLD_REG_IET_TX_HOLD_MAX                     (0xFFFFFFFFU)

/* IET_TX_FRAG_REG */

#define CSL_XGE_CPSW_STATS_IET_TX_FRAG_REG_IET_TX_FRAG_MASK                    (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_IET_TX_FRAG_REG_IET_TX_FRAG_SHIFT                   (0x00000000U)
#define CSL_XGE_CPSW_STATS_IET_TX_FRAG_REG_IET_TX_FRAG_MAX                     (0xFFFFFFFFU)

/* TX_MEMORY_PROTECT_ERROR */

#define CSL_XGE_CPSW_STATS_TX_MEMORY_PROTECT_ERROR_COUNT_MASK                  (0x000000FFU)
#define CSL_XGE_CPSW_STATS_TX_MEMORY_PROTECT_ERROR_COUNT_SHIFT                 (0x00000000U)
#define CSL_XGE_CPSW_STATS_TX_MEMORY_PROTECT_ERROR_COUNT_MAX                   (0x000000FFU)

/* ENET_PN_TX_PRI_REG */

#define CSL_XGE_CPSW_STATS_ENET_PN_TX_PRI_REG_PN_TX_PRIN_MASK                  (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_ENET_PN_TX_PRI_REG_PN_TX_PRIN_SHIFT                 (0x00000000U)
#define CSL_XGE_CPSW_STATS_ENET_PN_TX_PRI_REG_PN_TX_PRIN_MAX                   (0xFFFFFFFFU)

/* ENET_PN_TX_PRI_BCNT_REG */

#define CSL_XGE_CPSW_STATS_ENET_PN_TX_PRI_BCNT_REG_PN_TX_PRIN_BCNT_MASK        (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_ENET_PN_TX_PRI_BCNT_REG_PN_TX_PRIN_BCNT_SHIFT       (0x00000000U)
#define CSL_XGE_CPSW_STATS_ENET_PN_TX_PRI_BCNT_REG_PN_TX_PRIN_BCNT_MAX         (0xFFFFFFFFU)

/* ENET_PN_TX_PRI_DROP_REG */

#define CSL_XGE_CPSW_STATS_ENET_PN_TX_PRI_DROP_REG_PN_TX_PRIN_DROP_MASK        (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_ENET_PN_TX_PRI_DROP_REG_PN_TX_PRIN_DROP_SHIFT       (0x00000000U)
#define CSL_XGE_CPSW_STATS_ENET_PN_TX_PRI_DROP_REG_PN_TX_PRIN_DROP_MAX         (0xFFFFFFFFU)

/* ENET_PN_TX_PRI_DROP_BCNT_REG */

#define CSL_XGE_CPSW_STATS_ENET_PN_TX_PRI_DROP_BCNT_REG_PN_TX_PRIN_DROP_BCNT_MASK (0xFFFFFFFFU)
#define CSL_XGE_CPSW_STATS_ENET_PN_TX_PRI_DROP_BCNT_REG_PN_TX_PRIN_DROP_BCNT_SHIFT (0x00000000U)
#define CSL_XGE_CPSW_STATS_ENET_PN_TX_PRI_DROP_BCNT_REG_PN_TX_PRIN_DROP_BCNT_MAX (0xFFFFFFFFU)

/* ID_VER_REG */

#define CSL_XGE_CPSW_ID_VER_REG_MINOR_VER_MASK                            (0x0000003FU)
#define CSL_XGE_CPSW_ID_VER_REG_MINOR_VER_SHIFT                           (0x00000000U)
#define CSL_XGE_CPSW_ID_VER_REG_MINOR_VER_MAX                             (0x0000003FU)

#define CSL_XGE_CPSW_ID_VER_REG_CUSTOM_VER_MASK                           (0x000000C0U)
#define CSL_XGE_CPSW_ID_VER_REG_CUSTOM_VER_SHIFT                          (0x00000006U)
#define CSL_XGE_CPSW_ID_VER_REG_CUSTOM_VER_MAX                            (0x00000003U)

#define CSL_XGE_CPSW_ID_VER_REG_MAJOR_VER_MASK                            (0x00000700U)
#define CSL_XGE_CPSW_ID_VER_REG_MAJOR_VER_SHIFT                           (0x00000008U)
#define CSL_XGE_CPSW_ID_VER_REG_MAJOR_VER_MAX                             (0x00000007U)

#define CSL_XGE_CPSW_ID_VER_REG_RTL_VER_MASK                              (0x0000F800U)
#define CSL_XGE_CPSW_ID_VER_REG_RTL_VER_SHIFT                             (0x0000000BU)
#define CSL_XGE_CPSW_ID_VER_REG_RTL_VER_MAX                               (0x0000001FU)

#define CSL_XGE_CPSW_ID_VER_REG_IDENT_MASK                                (0xFFFF0000U)
#define CSL_XGE_CPSW_ID_VER_REG_IDENT_SHIFT                               (0x00000010U)
#define CSL_XGE_CPSW_ID_VER_REG_IDENT_MAX                                 (0x0000FFFFU)

/* CONTROL_REG */

#define CSL_XGE_CPSW_CONTROL_REG_S_CN_SWITCH_MASK                              (0x00000001U)
#define CSL_XGE_CPSW_CONTROL_REG_S_CN_SWITCH_SHIFT                             (0x00000000U)
#define CSL_XGE_CPSW_CONTROL_REG_S_CN_SWITCH_MAX                               (0x00000001U)

#define CSL_XGE_CPSW_CONTROL_REG_VLAN_AWARE_MASK                               (0x00000002U)
#define CSL_XGE_CPSW_CONTROL_REG_VLAN_AWARE_SHIFT                              (0x00000001U)
#define CSL_XGE_CPSW_CONTROL_REG_VLAN_AWARE_MAX                                (0x00000001U)

#define CSL_XGE_CPSW_CONTROL_REG_P0_ENABLE_MASK                                (0x00000004U)
#define CSL_XGE_CPSW_CONTROL_REG_P0_ENABLE_SHIFT                               (0x00000002U)
#define CSL_XGE_CPSW_CONTROL_REG_P0_ENABLE_MAX                                 (0x00000001U)

#define CSL_XGE_CPSW_CONTROL_REG_P0_PASS_PRI_TAGGED_MASK                       (0x00000008U)
#define CSL_XGE_CPSW_CONTROL_REG_P0_PASS_PRI_TAGGED_SHIFT                      (0x00000003U)
#define CSL_XGE_CPSW_CONTROL_REG_P0_PASS_PRI_TAGGED_MAX                        (0x00000001U)

#define CSL_XGE_CPSW_CONTROL_REG_P1_PASS_PRI_TAGGED_MASK                       (0x00000010U)
#define CSL_XGE_CPSW_CONTROL_REG_P1_PASS_PRI_TAGGED_SHIFT                      (0x00000004U)
#define CSL_XGE_CPSW_CONTROL_REG_P1_PASS_PRI_TAGGED_MAX                        (0x00000001U)

#define CSL_XGE_CPSW_CONTROL_REG_P2_PASS_PRI_TAGGED_MASK                       (0x00000020U)
#define CSL_XGE_CPSW_CONTROL_REG_P2_PASS_PRI_TAGGED_SHIFT                      (0x00000005U)
#define CSL_XGE_CPSW_CONTROL_REG_P2_PASS_PRI_TAGGED_MAX                        (0x00000001U)

#define CSL_XGE_CPSW_CONTROL_REG_P3_PASS_PRI_TAGGED_MASK                       (0x00000040U)
#define CSL_XGE_CPSW_CONTROL_REG_P3_PASS_PRI_TAGGED_SHIFT                      (0x00000006U)
#define CSL_XGE_CPSW_CONTROL_REG_P3_PASS_PRI_TAGGED_MAX                        (0x00000001U)

#define CSL_XGE_CPSW_CONTROL_REG_P4_PASS_PRI_TAGGED_MASK                       (0x00000080U)
#define CSL_XGE_CPSW_CONTROL_REG_P4_PASS_PRI_TAGGED_SHIFT                      (0x00000007U)
#define CSL_XGE_CPSW_CONTROL_REG_P4_PASS_PRI_TAGGED_MAX                        (0x00000001U)

#define CSL_XGE_CPSW_CONTROL_REG_P5_PASS_PRI_TAGGED_MASK                       (0x00000100U)
#define CSL_XGE_CPSW_CONTROL_REG_P5_PASS_PRI_TAGGED_SHIFT                      (0x00000008U)
#define CSL_XGE_CPSW_CONTROL_REG_P5_PASS_PRI_TAGGED_MAX                        (0x00000001U)

#define CSL_XGE_CPSW_CONTROL_REG_P6_PASS_PRI_TAGGED_MASK                       (0x00000200U)
#define CSL_XGE_CPSW_CONTROL_REG_P6_PASS_PRI_TAGGED_SHIFT                      (0x00000009U)
#define CSL_XGE_CPSW_CONTROL_REG_P6_PASS_PRI_TAGGED_MAX                        (0x00000001U)

#define CSL_XGE_CPSW_CONTROL_REG_P7_PASS_PRI_TAGGED_MASK                       (0x00000400U)
#define CSL_XGE_CPSW_CONTROL_REG_P7_PASS_PRI_TAGGED_SHIFT                      (0x0000000AU)
#define CSL_XGE_CPSW_CONTROL_REG_P7_PASS_PRI_TAGGED_MAX                        (0x00000001U)

#define CSL_XGE_CPSW_CONTROL_REG_P8_PASS_PRI_TAGGED_MASK                       (0x00000800U)
#define CSL_XGE_CPSW_CONTROL_REG_P8_PASS_PRI_TAGGED_SHIFT                      (0x0000000BU)
#define CSL_XGE_CPSW_CONTROL_REG_P8_PASS_PRI_TAGGED_MAX                        (0x00000001U)

#define CSL_XGE_CPSW_CONTROL_REG_P0_TX_CRC_TYPE_MASK                           (0x00001000U)
#define CSL_XGE_CPSW_CONTROL_REG_P0_TX_CRC_TYPE_SHIFT                          (0x0000000CU)
#define CSL_XGE_CPSW_CONTROL_REG_P0_TX_CRC_TYPE_MAX                            (0x00000001U)

#define CSL_XGE_CPSW_CONTROL_REG_P0_TX_CRC_REMOVE_MASK                         (0x00002000U)
#define CSL_XGE_CPSW_CONTROL_REG_P0_TX_CRC_REMOVE_SHIFT                        (0x0000000DU)
#define CSL_XGE_CPSW_CONTROL_REG_P0_TX_CRC_REMOVE_MAX                          (0x00000001U)

#define CSL_XGE_CPSW_CONTROL_REG_P0_RX_PAD_MASK                                (0x00004000U)
#define CSL_XGE_CPSW_CONTROL_REG_P0_RX_PAD_SHIFT                               (0x0000000EU)
#define CSL_XGE_CPSW_CONTROL_REG_P0_RX_PAD_MAX                                 (0x00000001U)

#define CSL_XGE_CPSW_CONTROL_REG_P0_RX_PASS_CRC_ERR_MASK                       (0x00008000U)
#define CSL_XGE_CPSW_CONTROL_REG_P0_RX_PASS_CRC_ERR_SHIFT                      (0x0000000FU)
#define CSL_XGE_CPSW_CONTROL_REG_P0_RX_PASS_CRC_ERR_MAX                        (0x00000001U)

#define CSL_XGE_CPSW_CONTROL_REG_EEE_ENABLE_MASK                               (0x00010000U)
#define CSL_XGE_CPSW_CONTROL_REG_EEE_ENABLE_SHIFT                              (0x00000010U)
#define CSL_XGE_CPSW_CONTROL_REG_EEE_ENABLE_MAX                                (0x00000001U)

#define CSL_XGE_CPSW_CONTROL_REG_IET_ENABLE_MASK                               (0x00020000U)
#define CSL_XGE_CPSW_CONTROL_REG_IET_ENABLE_SHIFT                              (0x00000011U)
#define CSL_XGE_CPSW_CONTROL_REG_IET_ENABLE_MAX                                (0x00000001U)

#define CSL_XGE_CPSW_CONTROL_REG_EST_ENABLE_MASK                               (0x00040000U)
#define CSL_XGE_CPSW_CONTROL_REG_EST_ENABLE_SHIFT                              (0x00000012U)
#define CSL_XGE_CPSW_CONTROL_REG_EST_ENABLE_MAX                                (0x00000001U)

#define CSL_XGE_CPSW_CONTROL_REG_ECC_CRC_MODE_MASK                             (0x80000000U)
#define CSL_XGE_CPSW_CONTROL_REG_ECC_CRC_MODE_SHIFT                            (0x0000001FU)
#define CSL_XGE_CPSW_CONTROL_REG_ECC_CRC_MODE_MAX                              (0x00000001U)

/* EM_CONTROL_REG */

#define CSL_XGE_CPSW_EM_CONTROL_REG_FREE_MASK                                  (0x00000001U)
#define CSL_XGE_CPSW_EM_CONTROL_REG_FREE_SHIFT                                 (0x00000000U)
#define CSL_XGE_CPSW_EM_CONTROL_REG_FREE_MAX                                   (0x00000001U)

#define CSL_XGE_CPSW_EM_CONTROL_REG_SOFT_MASK                                  (0x00000002U)
#define CSL_XGE_CPSW_EM_CONTROL_REG_SOFT_SHIFT                                 (0x00000001U)
#define CSL_XGE_CPSW_EM_CONTROL_REG_SOFT_MAX                                   (0x00000001U)

/* STAT_PORT_EN_REG */

#define CSL_XGE_CPSW_STAT_PORT_EN_REG_P0_STAT_EN_MASK                          (0x00000001U)
#define CSL_XGE_CPSW_STAT_PORT_EN_REG_P0_STAT_EN_SHIFT                         (0x00000000U)
#define CSL_XGE_CPSW_STAT_PORT_EN_REG_P0_STAT_EN_MAX                           (0x00000001U)

#define CSL_XGE_CPSW_STAT_PORT_EN_REG_P1_STAT_EN_MASK                          (0x00000002U)
#define CSL_XGE_CPSW_STAT_PORT_EN_REG_P1_STAT_EN_SHIFT                         (0x00000001U)
#define CSL_XGE_CPSW_STAT_PORT_EN_REG_P1_STAT_EN_MAX                           (0x00000001U)

#define CSL_XGE_CPSW_STAT_PORT_EN_REG_P2_STAT_EN_MASK                          (0x00000004U)
#define CSL_XGE_CPSW_STAT_PORT_EN_REG_P2_STAT_EN_SHIFT                         (0x00000002U)
#define CSL_XGE_CPSW_STAT_PORT_EN_REG_P2_STAT_EN_MAX                           (0x00000001U)

#define CSL_XGE_CPSW_STAT_PORT_EN_REG_P3_STAT_EN_MASK                          (0x00000008U)
#define CSL_XGE_CPSW_STAT_PORT_EN_REG_P3_STAT_EN_SHIFT                         (0x00000003U)
#define CSL_XGE_CPSW_STAT_PORT_EN_REG_P3_STAT_EN_MAX                           (0x00000001U)

#define CSL_XGE_CPSW_STAT_PORT_EN_REG_P4_STAT_EN_MASK                          (0x00000010U)
#define CSL_XGE_CPSW_STAT_PORT_EN_REG_P4_STAT_EN_SHIFT                         (0x00000004U)
#define CSL_XGE_CPSW_STAT_PORT_EN_REG_P4_STAT_EN_MAX                           (0x00000001U)

#define CSL_XGE_CPSW_STAT_PORT_EN_REG_P5_STAT_EN_MASK                          (0x00000020U)
#define CSL_XGE_CPSW_STAT_PORT_EN_REG_P5_STAT_EN_SHIFT                         (0x00000005U)
#define CSL_XGE_CPSW_STAT_PORT_EN_REG_P5_STAT_EN_MAX                           (0x00000001U)

#define CSL_XGE_CPSW_STAT_PORT_EN_REG_P6_STAT_EN_MASK                          (0x00000040U)
#define CSL_XGE_CPSW_STAT_PORT_EN_REG_P6_STAT_EN_SHIFT                         (0x00000006U)
#define CSL_XGE_CPSW_STAT_PORT_EN_REG_P6_STAT_EN_MAX                           (0x00000001U)

#define CSL_XGE_CPSW_STAT_PORT_EN_REG_P7_STAT_EN_MASK                          (0x00000080U)
#define CSL_XGE_CPSW_STAT_PORT_EN_REG_P7_STAT_EN_SHIFT                         (0x00000007U)
#define CSL_XGE_CPSW_STAT_PORT_EN_REG_P7_STAT_EN_MAX                           (0x00000001U)

#define CSL_XGE_CPSW_STAT_PORT_EN_REG_P8_STAT_EN_MASK                          (0x00000100U)
#define CSL_XGE_CPSW_STAT_PORT_EN_REG_P8_STAT_EN_SHIFT                         (0x00000008U)
#define CSL_XGE_CPSW_STAT_PORT_EN_REG_P8_STAT_EN_MAX                           (0x00000001U)

/* PTYPE_REG */

#define CSL_XGE_CPSW_PTYPE_REG_ESC_PRI_LD_VAL_MASK                             (0x0000001FU)
#define CSL_XGE_CPSW_PTYPE_REG_ESC_PRI_LD_VAL_SHIFT                            (0x00000000U)
#define CSL_XGE_CPSW_PTYPE_REG_ESC_PRI_LD_VAL_MAX                              (0x0000001FU)

#define CSL_XGE_CPSW_PTYPE_REG_P0_PTYPE_ESC_MASK                               (0x00000100U)
#define CSL_XGE_CPSW_PTYPE_REG_P0_PTYPE_ESC_SHIFT                              (0x00000008U)
#define CSL_XGE_CPSW_PTYPE_REG_P0_PTYPE_ESC_MAX                                (0x00000001U)

#define CSL_XGE_CPSW_PTYPE_REG_P1_PTYPE_ESC_MASK                               (0x00000200U)
#define CSL_XGE_CPSW_PTYPE_REG_P1_PTYPE_ESC_SHIFT                              (0x00000009U)
#define CSL_XGE_CPSW_PTYPE_REG_P1_PTYPE_ESC_MAX                                (0x00000001U)

#define CSL_XGE_CPSW_PTYPE_REG_P2_PTYPE_ESC_MASK                               (0x00000400U)
#define CSL_XGE_CPSW_PTYPE_REG_P2_PTYPE_ESC_SHIFT                              (0x0000000AU)
#define CSL_XGE_CPSW_PTYPE_REG_P2_PTYPE_ESC_MAX                                (0x00000001U)

#define CSL_XGE_CPSW_PTYPE_REG_P3_PTYPE_ESC_MASK                               (0x00000800U)
#define CSL_XGE_CPSW_PTYPE_REG_P3_PTYPE_ESC_SHIFT                              (0x0000000BU)
#define CSL_XGE_CPSW_PTYPE_REG_P3_PTYPE_ESC_MAX                                (0x00000001U)

#define CSL_XGE_CPSW_PTYPE_REG_P4_PTYPE_ESC_MASK                               (0x00001000U)
#define CSL_XGE_CPSW_PTYPE_REG_P4_PTYPE_ESC_SHIFT                              (0x0000000CU)
#define CSL_XGE_CPSW_PTYPE_REG_P4_PTYPE_ESC_MAX                                (0x00000001U)

#define CSL_XGE_CPSW_PTYPE_REG_P5_PTYPE_ESC_MASK                               (0x00002000U)
#define CSL_XGE_CPSW_PTYPE_REG_P5_PTYPE_ESC_SHIFT                              (0x0000000DU)
#define CSL_XGE_CPSW_PTYPE_REG_P5_PTYPE_ESC_MAX                                (0x00000001U)

#define CSL_XGE_CPSW_PTYPE_REG_P6_PTYPE_ESC_MASK                               (0x00004000U)
#define CSL_XGE_CPSW_PTYPE_REG_P6_PTYPE_ESC_SHIFT                              (0x0000000EU)
#define CSL_XGE_CPSW_PTYPE_REG_P6_PTYPE_ESC_MAX                                (0x00000001U)

#define CSL_XGE_CPSW_PTYPE_REG_P7_PTYPE_ESC_MASK                               (0x00008000U)
#define CSL_XGE_CPSW_PTYPE_REG_P7_PTYPE_ESC_SHIFT                              (0x0000000FU)
#define CSL_XGE_CPSW_PTYPE_REG_P7_PTYPE_ESC_MAX                                (0x00000001U)

#define CSL_XGE_CPSW_PTYPE_REG_P8_PTYPE_ESC_MASK                               (0x00010000U)
#define CSL_XGE_CPSW_PTYPE_REG_P8_PTYPE_ESC_SHIFT                              (0x00000010U)
#define CSL_XGE_CPSW_PTYPE_REG_P8_PTYPE_ESC_MAX                                (0x00000001U)

/* SOFT_IDLE_REG */

#define CSL_XGE_CPSW_SOFT_IDLE_REG_SOFT_IDLE_MASK                              (0x00000001U)
#define CSL_XGE_CPSW_SOFT_IDLE_REG_SOFT_IDLE_SHIFT                             (0x00000000U)
#define CSL_XGE_CPSW_SOFT_IDLE_REG_SOFT_IDLE_MAX                               (0x00000001U)

/* THRU_RATE_REG */

#define CSL_XGE_CPSW_THRU_RATE_REG_P0_RX_THRU_RATE_MASK                        (0x0000000FU)
#define CSL_XGE_CPSW_THRU_RATE_REG_P0_RX_THRU_RATE_SHIFT                       (0x00000000U)
#define CSL_XGE_CPSW_THRU_RATE_REG_P0_RX_THRU_RATE_MAX                         (0x0000000FU)

#define CSL_XGE_CPSW_THRU_RATE_REG_SL_RX_THRU_RATE_MASK                        (0x0000F000U)
#define CSL_XGE_CPSW_THRU_RATE_REG_SL_RX_THRU_RATE_SHIFT                       (0x0000000CU)
#define CSL_XGE_CPSW_THRU_RATE_REG_SL_RX_THRU_RATE_MAX                         (0x0000000FU)

/* GAP_THRESH_REG */

#define CSL_XGE_CPSW_GAP_THRESH_REG_GAP_THRESH_MASK                            (0x0000001FU)
#define CSL_XGE_CPSW_GAP_THRESH_REG_GAP_THRESH_SHIFT                           (0x00000000U)
#define CSL_XGE_CPSW_GAP_THRESH_REG_GAP_THRESH_MAX                             (0x0000001FU)

/* TX_START_WDS_REG */

#define CSL_XGE_CPSW_TX_START_WDS_REG_TX_START_WDS_MASK                        (0x000007FFU)
#define CSL_XGE_CPSW_TX_START_WDS_REG_TX_START_WDS_SHIFT                       (0x00000000U)
#define CSL_XGE_CPSW_TX_START_WDS_REG_TX_START_WDS_MAX                         (0x000007FFU)

/* EEE_PRESCALE_REG */

#define CSL_XGE_CPSW_EEE_PRESCALE_REG_EEE_PRESCALE_MASK                        (0x00000FFFU)
#define CSL_XGE_CPSW_EEE_PRESCALE_REG_EEE_PRESCALE_SHIFT                       (0x00000000U)
#define CSL_XGE_CPSW_EEE_PRESCALE_REG_EEE_PRESCALE_MAX                         (0x00000FFFU)

/* TX_G_OFLOW_THRESH_SET_REG */

#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI0_MASK                       (0x0000000FU)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI0_SHIFT                      (0x00000000U)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI0_MAX                        (0x0000000FU)

#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI1_MASK                       (0x000000F0U)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI1_SHIFT                      (0x00000004U)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI1_MAX                        (0x0000000FU)

#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI2_MASK                       (0x00000F00U)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI2_SHIFT                      (0x00000008U)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI2_MAX                        (0x0000000FU)

#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI3_MASK                       (0x0000F000U)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI3_SHIFT                      (0x0000000CU)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI3_MAX                        (0x0000000FU)

#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI4_MASK                       (0x000F0000U)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI4_SHIFT                      (0x00000010U)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI4_MAX                        (0x0000000FU)

#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI5_MASK                       (0x00F00000U)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI5_SHIFT                      (0x00000014U)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI5_MAX                        (0x0000000FU)

#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI6_MASK                       (0x0F000000U)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI6_SHIFT                      (0x00000018U)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI6_MAX                        (0x0000000FU)

#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI7_MASK                       (0xF0000000U)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI7_SHIFT                      (0x0000001CU)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_SET_REG_PRI7_MAX                        (0x0000000FU)

/* TX_G_OFLOW_THRESH_CLR_REG */

#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI0_MASK                       (0x0000000FU)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI0_SHIFT                      (0x00000000U)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI0_MAX                        (0x0000000FU)

#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI1_MASK                       (0x000000F0U)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI1_SHIFT                      (0x00000004U)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI1_MAX                        (0x0000000FU)

#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI2_MASK                       (0x00000F00U)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI2_SHIFT                      (0x00000008U)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI2_MAX                        (0x0000000FU)

#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI3_MASK                       (0x0000F000U)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI3_SHIFT                      (0x0000000CU)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI3_MAX                        (0x0000000FU)

#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI4_MASK                       (0x000F0000U)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI4_SHIFT                      (0x00000010U)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI4_MAX                        (0x0000000FU)

#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI5_MASK                       (0x00F00000U)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI5_SHIFT                      (0x00000014U)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI5_MAX                        (0x0000000FU)

#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI6_MASK                       (0x0F000000U)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI6_SHIFT                      (0x00000018U)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI6_MAX                        (0x0000000FU)

#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI7_MASK                       (0xF0000000U)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI7_SHIFT                      (0x0000001CU)
#define CSL_XGE_CPSW_TX_G_OFLOW_THRESH_CLR_REG_PRI7_MAX                        (0x0000000FU)

/* TX_G_BUF_THRESH_SET_L_REG */

#define CSL_XGE_CPSW_TX_G_BUF_THRESH_SET_L_REG_PRI0_MASK                       (0x000000FFU)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_SET_L_REG_PRI0_SHIFT                      (0x00000000U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_SET_L_REG_PRI0_MAX                        (0x000000FFU)

#define CSL_XGE_CPSW_TX_G_BUF_THRESH_SET_L_REG_PRI1_MASK                       (0x0000FF00U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_SET_L_REG_PRI1_SHIFT                      (0x00000008U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_SET_L_REG_PRI1_MAX                        (0x000000FFU)

#define CSL_XGE_CPSW_TX_G_BUF_THRESH_SET_L_REG_PRI2_MASK                       (0x00FF0000U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_SET_L_REG_PRI2_SHIFT                      (0x00000010U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_SET_L_REG_PRI2_MAX                        (0x000000FFU)

#define CSL_XGE_CPSW_TX_G_BUF_THRESH_SET_L_REG_PRI3_MASK                       (0xFF000000U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_SET_L_REG_PRI3_SHIFT                      (0x00000018U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_SET_L_REG_PRI3_MAX                        (0x000000FFU)

/* TX_G_BUF_THRESH_SET_H_REG */

#define CSL_XGE_CPSW_TX_G_BUF_THRESH_SET_H_REG_PRI4_MASK                       (0x000000FFU)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_SET_H_REG_PRI4_SHIFT                      (0x00000000U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_SET_H_REG_PRI4_MAX                        (0x000000FFU)

#define CSL_XGE_CPSW_TX_G_BUF_THRESH_SET_H_REG_PRI5_MASK                       (0x0000FF00U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_SET_H_REG_PRI5_SHIFT                      (0x00000008U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_SET_H_REG_PRI5_MAX                        (0x000000FFU)

#define CSL_XGE_CPSW_TX_G_BUF_THRESH_SET_H_REG_PRI6_MASK                       (0x00FF0000U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_SET_H_REG_PRI6_SHIFT                      (0x00000010U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_SET_H_REG_PRI6_MAX                        (0x000000FFU)

#define CSL_XGE_CPSW_TX_G_BUF_THRESH_SET_H_REG_PRI7_MASK                       (0xFF000000U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_SET_H_REG_PRI7_SHIFT                      (0x00000018U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_SET_H_REG_PRI7_MAX                        (0x000000FFU)

/* TX_G_BUF_THRESH_CLR_L_REG */

#define CSL_XGE_CPSW_TX_G_BUF_THRESH_CLR_L_REG_PRI0_MASK                       (0x000000FFU)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_CLR_L_REG_PRI0_SHIFT                      (0x00000000U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_CLR_L_REG_PRI0_MAX                        (0x000000FFU)

#define CSL_XGE_CPSW_TX_G_BUF_THRESH_CLR_L_REG_PRI1_MASK                       (0x0000FF00U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_CLR_L_REG_PRI1_SHIFT                      (0x00000008U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_CLR_L_REG_PRI1_MAX                        (0x000000FFU)

#define CSL_XGE_CPSW_TX_G_BUF_THRESH_CLR_L_REG_PRI2_MASK                       (0x00FF0000U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_CLR_L_REG_PRI2_SHIFT                      (0x00000010U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_CLR_L_REG_PRI2_MAX                        (0x000000FFU)

#define CSL_XGE_CPSW_TX_G_BUF_THRESH_CLR_L_REG_PRI3_MASK                       (0xFF000000U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_CLR_L_REG_PRI3_SHIFT                      (0x00000018U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_CLR_L_REG_PRI3_MAX                        (0x000000FFU)

/* TX_G_BUF_THRESH_CLR_H_REG */

#define CSL_XGE_CPSW_TX_G_BUF_THRESH_CLR_H_REG_PRI4_MASK                       (0x000000FFU)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_CLR_H_REG_PRI4_SHIFT                      (0x00000000U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_CLR_H_REG_PRI4_MAX                        (0x000000FFU)

#define CSL_XGE_CPSW_TX_G_BUF_THRESH_CLR_H_REG_PRI5_MASK                       (0x0000FF00U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_CLR_H_REG_PRI5_SHIFT                      (0x00000008U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_CLR_H_REG_PRI5_MAX                        (0x000000FFU)

#define CSL_XGE_CPSW_TX_G_BUF_THRESH_CLR_H_REG_PRI6_MASK                       (0x00FF0000U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_CLR_H_REG_PRI6_SHIFT                      (0x00000010U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_CLR_H_REG_PRI6_MAX                        (0x000000FFU)

#define CSL_XGE_CPSW_TX_G_BUF_THRESH_CLR_H_REG_PRI7_MASK                       (0xFF000000U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_CLR_H_REG_PRI7_SHIFT                      (0x00000018U)
#define CSL_XGE_CPSW_TX_G_BUF_THRESH_CLR_H_REG_PRI7_MAX                        (0x000000FFU)

/* VLAN_LTYPE_REG */

#define CSL_XGE_CPSW_VLAN_LTYPE_REG_VLAN_LTYPE_INNER_MASK                      (0x0000FFFFU)
#define CSL_XGE_CPSW_VLAN_LTYPE_REG_VLAN_LTYPE_INNER_SHIFT                     (0x00000000U)
#define CSL_XGE_CPSW_VLAN_LTYPE_REG_VLAN_LTYPE_INNER_MAX                       (0x0000FFFFU)

#define CSL_XGE_CPSW_VLAN_LTYPE_REG_VLAN_LTYPE_OUTER_MASK                      (0xFFFF0000U)
#define CSL_XGE_CPSW_VLAN_LTYPE_REG_VLAN_LTYPE_OUTER_SHIFT                     (0x00000010U)
#define CSL_XGE_CPSW_VLAN_LTYPE_REG_VLAN_LTYPE_OUTER_MAX                       (0x0000FFFFU)

/* EST_TS_DOMAIN_REG */

#define CSL_XGE_CPSW_EST_TS_DOMAIN_REG_EST_TS_DOMAIN_MASK                      (0x000000FFU)
#define CSL_XGE_CPSW_EST_TS_DOMAIN_REG_EST_TS_DOMAIN_SHIFT                     (0x00000000U)
#define CSL_XGE_CPSW_EST_TS_DOMAIN_REG_EST_TS_DOMAIN_MAX                       (0x000000FFU)

/* TX_PRI0_MAXLEN_REG */

#define CSL_XGE_CPSW_TX_PRI0_MAXLEN_REG_TX_PRI0_MAXLEN_MASK                    (0x00003FFFU)
#define CSL_XGE_CPSW_TX_PRI0_MAXLEN_REG_TX_PRI0_MAXLEN_SHIFT                   (0x00000000U)
#define CSL_XGE_CPSW_TX_PRI0_MAXLEN_REG_TX_PRI0_MAXLEN_MAX                     (0x00003FFFU)

/* TX_PRI1_MAXLEN_REG */

#define CSL_XGE_CPSW_TX_PRI1_MAXLEN_REG_TX_PRI1_MAXLEN_MASK                    (0x00003FFFU)
#define CSL_XGE_CPSW_TX_PRI1_MAXLEN_REG_TX_PRI1_MAXLEN_SHIFT                   (0x00000000U)
#define CSL_XGE_CPSW_TX_PRI1_MAXLEN_REG_TX_PRI1_MAXLEN_MAX                     (0x00003FFFU)

/* TX_PRI2_MAXLEN_REG */

#define CSL_XGE_CPSW_TX_PRI2_MAXLEN_REG_TX_PRI2_MAXLEN_MASK                    (0x00003FFFU)
#define CSL_XGE_CPSW_TX_PRI2_MAXLEN_REG_TX_PRI2_MAXLEN_SHIFT                   (0x00000000U)
#define CSL_XGE_CPSW_TX_PRI2_MAXLEN_REG_TX_PRI2_MAXLEN_MAX                     (0x00003FFFU)

/* TX_PRI3_MAXLEN_REG */

#define CSL_XGE_CPSW_TX_PRI3_MAXLEN_REG_TX_PRI3_MAXLEN_MASK                    (0x00003FFFU)
#define CSL_XGE_CPSW_TX_PRI3_MAXLEN_REG_TX_PRI3_MAXLEN_SHIFT                   (0x00000000U)
#define CSL_XGE_CPSW_TX_PRI3_MAXLEN_REG_TX_PRI3_MAXLEN_MAX                     (0x00003FFFU)

/* TX_PRI4_MAXLEN_REG */

#define CSL_XGE_CPSW_TX_PRI4_MAXLEN_REG_TX_PRI4_MAXLEN_MASK                    (0x00003FFFU)
#define CSL_XGE_CPSW_TX_PRI4_MAXLEN_REG_TX_PRI4_MAXLEN_SHIFT                   (0x00000000U)
#define CSL_XGE_CPSW_TX_PRI4_MAXLEN_REG_TX_PRI4_MAXLEN_MAX                     (0x00003FFFU)

/* TX_PRI5_MAXLEN_REG */

#define CSL_XGE_CPSW_TX_PRI5_MAXLEN_REG_TX_PRI5_MAXLEN_MASK                    (0x00003FFFU)
#define CSL_XGE_CPSW_TX_PRI5_MAXLEN_REG_TX_PRI5_MAXLEN_SHIFT                   (0x00000000U)
#define CSL_XGE_CPSW_TX_PRI5_MAXLEN_REG_TX_PRI5_MAXLEN_MAX                     (0x00003FFFU)

/* TX_PRI6_MAXLEN_REG */

#define CSL_XGE_CPSW_TX_PRI6_MAXLEN_REG_TX_PRI6_MAXLEN_MASK                    (0x00003FFFU)
#define CSL_XGE_CPSW_TX_PRI6_MAXLEN_REG_TX_PRI6_MAXLEN_SHIFT                   (0x00000000U)
#define CSL_XGE_CPSW_TX_PRI6_MAXLEN_REG_TX_PRI6_MAXLEN_MAX                     (0x00003FFFU)

/* TX_PRI7_MAXLEN_REG */

#define CSL_XGE_CPSW_TX_PRI7_MAXLEN_REG_TX_PRI7_MAXLEN_MASK                    (0x00003FFFU)
#define CSL_XGE_CPSW_TX_PRI7_MAXLEN_REG_TX_PRI7_MAXLEN_SHIFT                   (0x00000000U)
#define CSL_XGE_CPSW_TX_PRI7_MAXLEN_REG_TX_PRI7_MAXLEN_MAX                     (0x00003FFFU)

/* P0_CONTROL_REG */

#define CSL_XGE_CPSW_P0_CONTROL_REG_RX_CHECKSUM_EN_MASK                        (0x00000001U)
#define CSL_XGE_CPSW_P0_CONTROL_REG_RX_CHECKSUM_EN_SHIFT                       (0x00000000U)
#define CSL_XGE_CPSW_P0_CONTROL_REG_RX_CHECKSUM_EN_MAX                         (0x00000001U)

#define CSL_XGE_CPSW_P0_CONTROL_REG_DSCP_IPV4_EN_MASK                          (0x00000002U)
#define CSL_XGE_CPSW_P0_CONTROL_REG_DSCP_IPV4_EN_SHIFT                         (0x00000001U)
#define CSL_XGE_CPSW_P0_CONTROL_REG_DSCP_IPV4_EN_MAX                           (0x00000001U)

#define CSL_XGE_CPSW_P0_CONTROL_REG_DSCP_IPV6_EN_MASK                          (0x00000004U)
#define CSL_XGE_CPSW_P0_CONTROL_REG_DSCP_IPV6_EN_SHIFT                         (0x00000002U)
#define CSL_XGE_CPSW_P0_CONTROL_REG_DSCP_IPV6_EN_MAX                           (0x00000001U)

#define CSL_XGE_CPSW_P0_CONTROL_REG_TX_ECC_ERR_EN_MASK                         (0x00004000U)
#define CSL_XGE_CPSW_P0_CONTROL_REG_TX_ECC_ERR_EN_SHIFT                        (0x0000000EU)
#define CSL_XGE_CPSW_P0_CONTROL_REG_TX_ECC_ERR_EN_MAX                          (0x00000001U)

#define CSL_XGE_CPSW_P0_CONTROL_REG_RX_ECC_ERR_EN_MASK                         (0x00008000U)
#define CSL_XGE_CPSW_P0_CONTROL_REG_RX_ECC_ERR_EN_SHIFT                        (0x0000000FU)
#define CSL_XGE_CPSW_P0_CONTROL_REG_RX_ECC_ERR_EN_MAX                          (0x00000001U)

#define CSL_XGE_CPSW_P0_CONTROL_REG_RX_REMAP_VLAN_MASK                         (0x00010000U)
#define CSL_XGE_CPSW_P0_CONTROL_REG_RX_REMAP_VLAN_SHIFT                        (0x00000010U)
#define CSL_XGE_CPSW_P0_CONTROL_REG_RX_REMAP_VLAN_MAX                          (0x00000001U)

#define CSL_XGE_CPSW_P0_CONTROL_REG_RX_REMAP_DSCP_V4_MASK                      (0x00020000U)
#define CSL_XGE_CPSW_P0_CONTROL_REG_RX_REMAP_DSCP_V4_SHIFT                     (0x00000011U)
#define CSL_XGE_CPSW_P0_CONTROL_REG_RX_REMAP_DSCP_V4_MAX                       (0x00000001U)

#define CSL_XGE_CPSW_P0_CONTROL_REG_RX_REMAP_DSCP_V6_MASK                      (0x00040000U)
#define CSL_XGE_CPSW_P0_CONTROL_REG_RX_REMAP_DSCP_V6_SHIFT                     (0x00000012U)
#define CSL_XGE_CPSW_P0_CONTROL_REG_RX_REMAP_DSCP_V6_MAX                       (0x00000001U)

/* P0_FLOW_ID_OFFSET_REG */

#define CSL_XGE_CPSW_P0_FLOW_ID_OFFSET_REG_VALUE_MASK                          (0x00003FFFU)
#define CSL_XGE_CPSW_P0_FLOW_ID_OFFSET_REG_VALUE_SHIFT                         (0x00000000U)
#define CSL_XGE_CPSW_P0_FLOW_ID_OFFSET_REG_VALUE_MAX                           (0x00003FFFU)

/* P0_BLK_CNT_REG */

#define CSL_XGE_CPSW_P0_BLK_CNT_REG_RX_BLK_CNT_MASK                            (0x0000003FU)
#define CSL_XGE_CPSW_P0_BLK_CNT_REG_RX_BLK_CNT_SHIFT                           (0x00000000U)
#define CSL_XGE_CPSW_P0_BLK_CNT_REG_RX_BLK_CNT_MAX                             (0x0000003FU)

#define CSL_XGE_CPSW_P0_BLK_CNT_REG_TX_BLK_CNT_MASK                            (0x00001F00U)
#define CSL_XGE_CPSW_P0_BLK_CNT_REG_TX_BLK_CNT_SHIFT                           (0x00000008U)
#define CSL_XGE_CPSW_P0_BLK_CNT_REG_TX_BLK_CNT_MAX                             (0x0000001FU)

/* P0_PORT_VLAN_REG */

#define CSL_XGE_CPSW_P0_PORT_VLAN_REG_PORT_VID_MASK                            (0x00000FFFU)
#define CSL_XGE_CPSW_P0_PORT_VLAN_REG_PORT_VID_SHIFT                           (0x00000000U)
#define CSL_XGE_CPSW_P0_PORT_VLAN_REG_PORT_VID_MAX                             (0x00000FFFU)

#define CSL_XGE_CPSW_P0_PORT_VLAN_REG_PORT_CFI_MASK                            (0x00001000U)
#define CSL_XGE_CPSW_P0_PORT_VLAN_REG_PORT_CFI_SHIFT                           (0x0000000CU)
#define CSL_XGE_CPSW_P0_PORT_VLAN_REG_PORT_CFI_MAX                             (0x00000001U)

#define CSL_XGE_CPSW_P0_PORT_VLAN_REG_PORT_PRI_MASK                            (0x0000E000U)
#define CSL_XGE_CPSW_P0_PORT_VLAN_REG_PORT_PRI_SHIFT                           (0x0000000DU)
#define CSL_XGE_CPSW_P0_PORT_VLAN_REG_PORT_PRI_MAX                             (0x00000007U)

/* P0_TX_PRI_MAP_REG */

#define CSL_XGE_CPSW_P0_TX_PRI_MAP_REG_PRI0_MASK                               (0x00000007U)
#define CSL_XGE_CPSW_P0_TX_PRI_MAP_REG_PRI0_SHIFT                              (0x00000000U)
#define CSL_XGE_CPSW_P0_TX_PRI_MAP_REG_PRI0_MAX                                (0x00000007U)

#define CSL_XGE_CPSW_P0_TX_PRI_MAP_REG_PRI1_MASK                               (0x00000070U)
#define CSL_XGE_CPSW_P0_TX_PRI_MAP_REG_PRI1_SHIFT                              (0x00000004U)
#define CSL_XGE_CPSW_P0_TX_PRI_MAP_REG_PRI1_MAX                                (0x00000007U)

#define CSL_XGE_CPSW_P0_TX_PRI_MAP_REG_PRI2_MASK                               (0x00000700U)
#define CSL_XGE_CPSW_P0_TX_PRI_MAP_REG_PRI2_SHIFT                              (0x00000008U)
#define CSL_XGE_CPSW_P0_TX_PRI_MAP_REG_PRI2_MAX                                (0x00000007U)

#define CSL_XGE_CPSW_P0_TX_PRI_MAP_REG_PRI3_MASK                               (0x00007000U)
#define CSL_XGE_CPSW_P0_TX_PRI_MAP_REG_PRI3_SHIFT                              (0x0000000CU)
#define CSL_XGE_CPSW_P0_TX_PRI_MAP_REG_PRI3_MAX                                (0x00000007U)

#define CSL_XGE_CPSW_P0_TX_PRI_MAP_REG_PRI4_MASK                               (0x00070000U)
#define CSL_XGE_CPSW_P0_TX_PRI_MAP_REG_PRI4_SHIFT                              (0x00000010U)
#define CSL_XGE_CPSW_P0_TX_PRI_MAP_REG_PRI4_MAX                                (0x00000007U)

#define CSL_XGE_CPSW_P0_TX_PRI_MAP_REG_PRI5_MASK                               (0x00700000U)
#define CSL_XGE_CPSW_P0_TX_PRI_MAP_REG_PRI5_SHIFT                              (0x00000014U)
#define CSL_XGE_CPSW_P0_TX_PRI_MAP_REG_PRI5_MAX                                (0x00000007U)

#define CSL_XGE_CPSW_P0_TX_PRI_MAP_REG_PRI6_MASK                               (0x07000000U)
#define CSL_XGE_CPSW_P0_TX_PRI_MAP_REG_PRI6_SHIFT                              (0x00000018U)
#define CSL_XGE_CPSW_P0_TX_PRI_MAP_REG_PRI6_MAX                                (0x00000007U)

#define CSL_XGE_CPSW_P0_TX_PRI_MAP_REG_PRI7_MASK                               (0x70000000U)
#define CSL_XGE_CPSW_P0_TX_PRI_MAP_REG_PRI7_SHIFT                              (0x0000001CU)
#define CSL_XGE_CPSW_P0_TX_PRI_MAP_REG_PRI7_MAX                                (0x00000007U)

/* P0_PRI_CTL_REG */

#define CSL_XGE_CPSW_P0_PRI_CTL_REG_RX_PTYPE_MASK                              (0x00000100U)
#define CSL_XGE_CPSW_P0_PRI_CTL_REG_RX_PTYPE_SHIFT                             (0x00000008U)
#define CSL_XGE_CPSW_P0_PRI_CTL_REG_RX_PTYPE_MAX                               (0x00000001U)

#define CSL_XGE_CPSW_P0_PRI_CTL_REG_RX_FLOW_PRI_MASK                           (0x00FF0000U)
#define CSL_XGE_CPSW_P0_PRI_CTL_REG_RX_FLOW_PRI_SHIFT                          (0x00000010U)
#define CSL_XGE_CPSW_P0_PRI_CTL_REG_RX_FLOW_PRI_MAX                            (0x000000FFU)

/* P0_RX_PRI_MAP_REG */

#define CSL_XGE_CPSW_P0_RX_PRI_MAP_REG_PRI0_MASK                               (0x00000007U)
#define CSL_XGE_CPSW_P0_RX_PRI_MAP_REG_PRI0_SHIFT                              (0x00000000U)
#define CSL_XGE_CPSW_P0_RX_PRI_MAP_REG_PRI0_MAX                                (0x00000007U)

#define CSL_XGE_CPSW_P0_RX_PRI_MAP_REG_PRI1_MASK                               (0x00000070U)
#define CSL_XGE_CPSW_P0_RX_PRI_MAP_REG_PRI1_SHIFT                              (0x00000004U)
#define CSL_XGE_CPSW_P0_RX_PRI_MAP_REG_PRI1_MAX                                (0x00000007U)

#define CSL_XGE_CPSW_P0_RX_PRI_MAP_REG_PRI2_MASK                               (0x00000700U)
#define CSL_XGE_CPSW_P0_RX_PRI_MAP_REG_PRI2_SHIFT                              (0x00000008U)
#define CSL_XGE_CPSW_P0_RX_PRI_MAP_REG_PRI2_MAX                                (0x00000007U)

#define CSL_XGE_CPSW_P0_RX_PRI_MAP_REG_PRI3_MASK                               (0x00007000U)
#define CSL_XGE_CPSW_P0_RX_PRI_MAP_REG_PRI3_SHIFT                              (0x0000000CU)
#define CSL_XGE_CPSW_P0_RX_PRI_MAP_REG_PRI3_MAX                                (0x00000007U)

#define CSL_XGE_CPSW_P0_RX_PRI_MAP_REG_PRI4_MASK                               (0x00070000U)
#define CSL_XGE_CPSW_P0_RX_PRI_MAP_REG_PRI4_SHIFT                              (0x00000010U)
#define CSL_XGE_CPSW_P0_RX_PRI_MAP_REG_PRI4_MAX                                (0x00000007U)

#define CSL_XGE_CPSW_P0_RX_PRI_MAP_REG_PRI5_MASK                               (0x00700000U)
#define CSL_XGE_CPSW_P0_RX_PRI_MAP_REG_PRI5_SHIFT                              (0x00000014U)
#define CSL_XGE_CPSW_P0_RX_PRI_MAP_REG_PRI5_MAX                                (0x00000007U)

#define CSL_XGE_CPSW_P0_RX_PRI_MAP_REG_PRI6_MASK                               (0x07000000U)
#define CSL_XGE_CPSW_P0_RX_PRI_MAP_REG_PRI6_SHIFT                              (0x00000018U)
#define CSL_XGE_CPSW_P0_RX_PRI_MAP_REG_PRI6_MAX                                (0x00000007U)

#define CSL_XGE_CPSW_P0_RX_PRI_MAP_REG_PRI7_MASK                               (0x70000000U)
#define CSL_XGE_CPSW_P0_RX_PRI_MAP_REG_PRI7_SHIFT                              (0x0000001CU)
#define CSL_XGE_CPSW_P0_RX_PRI_MAP_REG_PRI7_MAX                                (0x00000007U)

/* P0_RX_MAXLEN_REG */

#define CSL_XGE_CPSW_P0_RX_MAXLEN_REG_RX_MAXLEN_MASK                           (0x00003FFFU)
#define CSL_XGE_CPSW_P0_RX_MAXLEN_REG_RX_MAXLEN_SHIFT                          (0x00000000U)
#define CSL_XGE_CPSW_P0_RX_MAXLEN_REG_RX_MAXLEN_MAX                            (0x00003FFFU)

/* P0_TX_BLKS_PRI_REG */

#define CSL_XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI0_MASK                              (0x0000000FU)
#define CSL_XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI0_SHIFT                             (0x00000000U)
#define CSL_XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI0_MAX                               (0x0000000FU)

#define CSL_XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI1_MASK                              (0x000000F0U)
#define CSL_XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI1_SHIFT                             (0x00000004U)
#define CSL_XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI1_MAX                               (0x0000000FU)

#define CSL_XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI2_MASK                              (0x00000F00U)
#define CSL_XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI2_SHIFT                             (0x00000008U)
#define CSL_XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI2_MAX                               (0x0000000FU)

#define CSL_XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI3_MASK                              (0x0000F000U)
#define CSL_XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI3_SHIFT                             (0x0000000CU)
#define CSL_XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI3_MAX                               (0x0000000FU)

#define CSL_XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI4_MASK                              (0x000F0000U)
#define CSL_XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI4_SHIFT                             (0x00000010U)
#define CSL_XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI4_MAX                               (0x0000000FU)

#define CSL_XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI5_MASK                              (0x00F00000U)
#define CSL_XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI5_SHIFT                             (0x00000014U)
#define CSL_XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI5_MAX                               (0x0000000FU)

#define CSL_XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI6_MASK                              (0x0F000000U)
#define CSL_XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI6_SHIFT                             (0x00000018U)
#define CSL_XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI6_MAX                               (0x0000000FU)

#define CSL_XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI7_MASK                              (0xF0000000U)
#define CSL_XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI7_SHIFT                             (0x0000001CU)
#define CSL_XGE_CPSW_P0_TX_BLKS_PRI_REG_PRI7_MAX                               (0x0000000FU)

/* P0_IDLE2LPI_REG */

#define CSL_XGE_CPSW_P0_IDLE2LPI_REG_COUNT_MASK                                (0x00FFFFFFU)
#define CSL_XGE_CPSW_P0_IDLE2LPI_REG_COUNT_SHIFT                               (0x00000000U)
#define CSL_XGE_CPSW_P0_IDLE2LPI_REG_COUNT_MAX                                 (0x00FFFFFFU)

/* P0_LPI2WAKE_REG */

#define CSL_XGE_CPSW_P0_LPI2WAKE_REG_COUNT_MASK                                (0x00FFFFFFU)
#define CSL_XGE_CPSW_P0_LPI2WAKE_REG_COUNT_SHIFT                               (0x00000000U)
#define CSL_XGE_CPSW_P0_LPI2WAKE_REG_COUNT_MAX                                 (0x00FFFFFFU)

/* P0_EEE_STATUS_REG */

#define CSL_XGE_CPSW_P0_EEE_STATUS_REG_WAIT_IDLE2LPI_MASK                      (0x00000001U)
#define CSL_XGE_CPSW_P0_EEE_STATUS_REG_WAIT_IDLE2LPI_SHIFT                     (0x00000000U)
#define CSL_XGE_CPSW_P0_EEE_STATUS_REG_WAIT_IDLE2LPI_MAX                       (0x00000001U)

#define CSL_XGE_CPSW_P0_EEE_STATUS_REG_RX_LPI_MASK                             (0x00000002U)
#define CSL_XGE_CPSW_P0_EEE_STATUS_REG_RX_LPI_SHIFT                            (0x00000001U)
#define CSL_XGE_CPSW_P0_EEE_STATUS_REG_RX_LPI_MAX                              (0x00000001U)

#define CSL_XGE_CPSW_P0_EEE_STATUS_REG_TX_LPI_MASK                             (0x00000004U)
#define CSL_XGE_CPSW_P0_EEE_STATUS_REG_TX_LPI_SHIFT                            (0x00000002U)
#define CSL_XGE_CPSW_P0_EEE_STATUS_REG_TX_LPI_MAX                              (0x00000001U)

#define CSL_XGE_CPSW_P0_EEE_STATUS_REG_TX_WAKE_MASK                            (0x00000008U)
#define CSL_XGE_CPSW_P0_EEE_STATUS_REG_TX_WAKE_SHIFT                           (0x00000003U)
#define CSL_XGE_CPSW_P0_EEE_STATUS_REG_TX_WAKE_MAX                             (0x00000001U)

#define CSL_XGE_CPSW_P0_EEE_STATUS_REG_TX_FIFO_HOLD_MASK                       (0x00000010U)
#define CSL_XGE_CPSW_P0_EEE_STATUS_REG_TX_FIFO_HOLD_SHIFT                      (0x00000004U)
#define CSL_XGE_CPSW_P0_EEE_STATUS_REG_TX_FIFO_HOLD_MAX                        (0x00000001U)

#define CSL_XGE_CPSW_P0_EEE_STATUS_REG_RX_FIFO_EMPTY_MASK                      (0x00000020U)
#define CSL_XGE_CPSW_P0_EEE_STATUS_REG_RX_FIFO_EMPTY_SHIFT                     (0x00000005U)
#define CSL_XGE_CPSW_P0_EEE_STATUS_REG_RX_FIFO_EMPTY_MAX                       (0x00000001U)

#define CSL_XGE_CPSW_P0_EEE_STATUS_REG_TX_FIFO_EMPTY_MASK                      (0x00000040U)
#define CSL_XGE_CPSW_P0_EEE_STATUS_REG_TX_FIFO_EMPTY_SHIFT                     (0x00000006U)
#define CSL_XGE_CPSW_P0_EEE_STATUS_REG_TX_FIFO_EMPTY_MAX                       (0x00000001U)

/* P0_RX_PKTS_PRI_REG */

#define CSL_XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI0_MASK                              (0x0000000FU)
#define CSL_XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI0_SHIFT                             (0x00000000U)
#define CSL_XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI0_MAX                               (0x0000000FU)

#define CSL_XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI1_MASK                              (0x000000F0U)
#define CSL_XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI1_SHIFT                             (0x00000004U)
#define CSL_XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI1_MAX                               (0x0000000FU)

#define CSL_XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI2_MASK                              (0x00000F00U)
#define CSL_XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI2_SHIFT                             (0x00000008U)
#define CSL_XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI2_MAX                               (0x0000000FU)

#define CSL_XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI3_MASK                              (0x0000F000U)
#define CSL_XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI3_SHIFT                             (0x0000000CU)
#define CSL_XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI3_MAX                               (0x0000000FU)

#define CSL_XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI4_MASK                              (0x000F0000U)
#define CSL_XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI4_SHIFT                             (0x00000010U)
#define CSL_XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI4_MAX                               (0x0000000FU)

#define CSL_XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI5_MASK                              (0x00F00000U)
#define CSL_XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI5_SHIFT                             (0x00000014U)
#define CSL_XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI5_MAX                               (0x0000000FU)

#define CSL_XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI6_MASK                              (0x0F000000U)
#define CSL_XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI6_SHIFT                             (0x00000018U)
#define CSL_XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI6_MAX                               (0x0000000FU)

#define CSL_XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI7_MASK                              (0xF0000000U)
#define CSL_XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI7_SHIFT                             (0x0000001CU)
#define CSL_XGE_CPSW_P0_RX_PKTS_PRI_REG_PRI7_MAX                               (0x0000000FU)

/* P0_RX_GAP_REG */

#define CSL_XGE_CPSW_P0_RX_GAP_REG_RX_GAP_EN_MASK                              (0x000000FFU)
#define CSL_XGE_CPSW_P0_RX_GAP_REG_RX_GAP_EN_SHIFT                             (0x00000000U)
#define CSL_XGE_CPSW_P0_RX_GAP_REG_RX_GAP_EN_MAX                               (0x000000FFU)

#define CSL_XGE_CPSW_P0_RX_GAP_REG_RX_GAP_CNT_MASK                             (0x03FF0000U)
#define CSL_XGE_CPSW_P0_RX_GAP_REG_RX_GAP_CNT_SHIFT                            (0x00000010U)
#define CSL_XGE_CPSW_P0_RX_GAP_REG_RX_GAP_CNT_MAX                              (0x000003FFU)

/* P0_FIFO_STATUS_REG */

#define CSL_XGE_CPSW_P0_FIFO_STATUS_REG_TX_PRI_ACTIVE_MASK                     (0x000000FFU)
#define CSL_XGE_CPSW_P0_FIFO_STATUS_REG_TX_PRI_ACTIVE_SHIFT                    (0x00000000U)
#define CSL_XGE_CPSW_P0_FIFO_STATUS_REG_TX_PRI_ACTIVE_MAX                      (0x000000FFU)

/* P0_RX_DSCP_MAP_REG */

#define CSL_XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI0_MASK                              (0x00000007U)
#define CSL_XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI0_SHIFT                             (0x00000000U)
#define CSL_XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI0_MAX                               (0x00000007U)

#define CSL_XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI1_MASK                              (0x00000070U)
#define CSL_XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI1_SHIFT                             (0x00000004U)
#define CSL_XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI1_MAX                               (0x00000007U)

#define CSL_XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI2_MASK                              (0x00000700U)
#define CSL_XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI2_SHIFT                             (0x00000008U)
#define CSL_XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI2_MAX                               (0x00000007U)

#define CSL_XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI3_MASK                              (0x00007000U)
#define CSL_XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI3_SHIFT                             (0x0000000CU)
#define CSL_XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI3_MAX                               (0x00000007U)

#define CSL_XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI4_MASK                              (0x00070000U)
#define CSL_XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI4_SHIFT                             (0x00000010U)
#define CSL_XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI4_MAX                               (0x00000007U)

#define CSL_XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI5_MASK                              (0x00700000U)
#define CSL_XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI5_SHIFT                             (0x00000014U)
#define CSL_XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI5_MAX                               (0x00000007U)

#define CSL_XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI6_MASK                              (0x07000000U)
#define CSL_XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI6_SHIFT                             (0x00000018U)
#define CSL_XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI6_MAX                               (0x00000007U)

#define CSL_XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI7_MASK                              (0x70000000U)
#define CSL_XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI7_SHIFT                             (0x0000001CU)
#define CSL_XGE_CPSW_P0_RX_DSCP_MAP_REG_PRI7_MAX                               (0x00000007U)

/* P0_PRI_CIR_REG */

#define CSL_XGE_CPSW_P0_PRI_CIR_REG_PRI_CIR_MASK                               (0x0FFFFFFFU)
#define CSL_XGE_CPSW_P0_PRI_CIR_REG_PRI_CIR_SHIFT                              (0x00000000U)
#define CSL_XGE_CPSW_P0_PRI_CIR_REG_PRI_CIR_MAX                                (0x0FFFFFFFU)

/* P0_PRI_EIR_REG */

#define CSL_XGE_CPSW_P0_PRI_EIR_REG_PRI_EIR_MASK                               (0x0FFFFFFFU)
#define CSL_XGE_CPSW_P0_PRI_EIR_REG_PRI_EIR_SHIFT                              (0x00000000U)
#define CSL_XGE_CPSW_P0_PRI_EIR_REG_PRI_EIR_MAX                                (0x0FFFFFFFU)

/* P0_TX_D_THRESH_SET_L_REG */

#define CSL_XGE_CPSW_P0_TX_D_THRESH_SET_L_REG_PRI0_MASK                        (0x0000001FU)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_SET_L_REG_PRI0_SHIFT                       (0x00000000U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_SET_L_REG_PRI0_MAX                         (0x0000001FU)

#define CSL_XGE_CPSW_P0_TX_D_THRESH_SET_L_REG_PRI1_MASK                        (0x00001F00U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_SET_L_REG_PRI1_SHIFT                       (0x00000008U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_SET_L_REG_PRI1_MAX                         (0x0000001FU)

#define CSL_XGE_CPSW_P0_TX_D_THRESH_SET_L_REG_PRI2_MASK                        (0x001F0000U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_SET_L_REG_PRI2_SHIFT                       (0x00000010U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_SET_L_REG_PRI2_MAX                         (0x0000001FU)

#define CSL_XGE_CPSW_P0_TX_D_THRESH_SET_L_REG_PRI3_MASK                        (0x1F000000U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_SET_L_REG_PRI3_SHIFT                       (0x00000018U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_SET_L_REG_PRI3_MAX                         (0x0000001FU)

/* P0_TX_D_THRESH_SET_H_REG */

#define CSL_XGE_CPSW_P0_TX_D_THRESH_SET_H_REG_PRI4_MASK                        (0x0000001FU)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_SET_H_REG_PRI4_SHIFT                       (0x00000000U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_SET_H_REG_PRI4_MAX                         (0x0000001FU)

#define CSL_XGE_CPSW_P0_TX_D_THRESH_SET_H_REG_PRI5_MASK                        (0x00001F00U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_SET_H_REG_PRI5_SHIFT                       (0x00000008U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_SET_H_REG_PRI5_MAX                         (0x0000001FU)

#define CSL_XGE_CPSW_P0_TX_D_THRESH_SET_H_REG_PRI6_MASK                        (0x001F0000U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_SET_H_REG_PRI6_SHIFT                       (0x00000010U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_SET_H_REG_PRI6_MAX                         (0x0000001FU)

#define CSL_XGE_CPSW_P0_TX_D_THRESH_SET_H_REG_PRI7_MASK                        (0x1F000000U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_SET_H_REG_PRI7_SHIFT                       (0x00000018U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_SET_H_REG_PRI7_MAX                         (0x0000001FU)

/* P0_TX_D_THRESH_CLR_L_REG */

#define CSL_XGE_CPSW_P0_TX_D_THRESH_CLR_L_REG_PRI0_MASK                        (0x0000001FU)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_CLR_L_REG_PRI0_SHIFT                       (0x00000000U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_CLR_L_REG_PRI0_MAX                         (0x0000001FU)

#define CSL_XGE_CPSW_P0_TX_D_THRESH_CLR_L_REG_PRI1_MASK                        (0x00001F00U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_CLR_L_REG_PRI1_SHIFT                       (0x00000008U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_CLR_L_REG_PRI1_MAX                         (0x0000001FU)

#define CSL_XGE_CPSW_P0_TX_D_THRESH_CLR_L_REG_PRI2_MASK                        (0x001F0000U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_CLR_L_REG_PRI2_SHIFT                       (0x00000010U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_CLR_L_REG_PRI2_MAX                         (0x0000001FU)

#define CSL_XGE_CPSW_P0_TX_D_THRESH_CLR_L_REG_PRI3_MASK                        (0x1F000000U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_CLR_L_REG_PRI3_SHIFT                       (0x00000018U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_CLR_L_REG_PRI3_MAX                         (0x0000001FU)

/* P0_TX_D_THRESH_CLR_H_REG */

#define CSL_XGE_CPSW_P0_TX_D_THRESH_CLR_H_REG_PRI4_MASK                        (0x0000001FU)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_CLR_H_REG_PRI4_SHIFT                       (0x00000000U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_CLR_H_REG_PRI4_MAX                         (0x0000001FU)

#define CSL_XGE_CPSW_P0_TX_D_THRESH_CLR_H_REG_PRI5_MASK                        (0x00001F00U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_CLR_H_REG_PRI5_SHIFT                       (0x00000008U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_CLR_H_REG_PRI5_MAX                         (0x0000001FU)

#define CSL_XGE_CPSW_P0_TX_D_THRESH_CLR_H_REG_PRI6_MASK                        (0x001F0000U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_CLR_H_REG_PRI6_SHIFT                       (0x00000010U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_CLR_H_REG_PRI6_MAX                         (0x0000001FU)

#define CSL_XGE_CPSW_P0_TX_D_THRESH_CLR_H_REG_PRI7_MASK                        (0x1F000000U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_CLR_H_REG_PRI7_SHIFT                       (0x00000018U)
#define CSL_XGE_CPSW_P0_TX_D_THRESH_CLR_H_REG_PRI7_MAX                         (0x0000001FU)

/* P0_TX_G_BUF_THRESH_SET_L_REG */

#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_SET_L_REG_PRI0_MASK                    (0x0000001FU)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_SET_L_REG_PRI0_SHIFT                   (0x00000000U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_SET_L_REG_PRI0_MAX                     (0x0000001FU)

#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_SET_L_REG_PRI1_MASK                    (0x00001F00U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_SET_L_REG_PRI1_SHIFT                   (0x00000008U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_SET_L_REG_PRI1_MAX                     (0x0000001FU)

#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_SET_L_REG_PRI2_MASK                    (0x001F0000U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_SET_L_REG_PRI2_SHIFT                   (0x00000010U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_SET_L_REG_PRI2_MAX                     (0x0000001FU)

#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_SET_L_REG_PRI3_MASK                    (0x1F000000U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_SET_L_REG_PRI3_SHIFT                   (0x00000018U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_SET_L_REG_PRI3_MAX                     (0x0000001FU)

/* P0_TX_G_BUF_THRESH_SET_H_REG */

#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_SET_H_REG_PRI4_MASK                    (0x0000001FU)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_SET_H_REG_PRI4_SHIFT                   (0x00000000U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_SET_H_REG_PRI4_MAX                     (0x0000001FU)

#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_SET_H_REG_PRI5_MASK                    (0x00001F00U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_SET_H_REG_PRI5_SHIFT                   (0x00000008U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_SET_H_REG_PRI5_MAX                     (0x0000001FU)

#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_SET_H_REG_PRI6_MASK                    (0x001F0000U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_SET_H_REG_PRI6_SHIFT                   (0x00000010U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_SET_H_REG_PRI6_MAX                     (0x0000001FU)

#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_SET_H_REG_PRI7_MASK                    (0x1F000000U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_SET_H_REG_PRI7_SHIFT                   (0x00000018U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_SET_H_REG_PRI7_MAX                     (0x0000001FU)

/* P0_TX_G_BUF_THRESH_CLR_L_REG */

#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI0_MASK                    (0x0000001FU)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI0_SHIFT                   (0x00000000U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI0_MAX                     (0x0000001FU)

#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI1_MASK                    (0x00001F00U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI1_SHIFT                   (0x00000008U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI1_MAX                     (0x0000001FU)

#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI2_MASK                    (0x001F0000U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI2_SHIFT                   (0x00000010U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI2_MAX                     (0x0000001FU)

#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI3_MASK                    (0x1F000000U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI3_SHIFT                   (0x00000018U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI3_MAX                     (0x0000001FU)

/* P0_TX_G_BUF_THRESH_CLR_H_REG */

#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI4_MASK                    (0x0000001FU)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI4_SHIFT                   (0x00000000U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI4_MAX                     (0x0000001FU)

#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI5_MASK                    (0x00001F00U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI5_SHIFT                   (0x00000008U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI5_MAX                     (0x0000001FU)

#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI6_MASK                    (0x001F0000U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI6_SHIFT                   (0x00000010U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI6_MAX                     (0x0000001FU)

#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI7_MASK                    (0x1F000000U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI7_SHIFT                   (0x00000018U)
#define CSL_XGE_CPSW_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI7_MAX                     (0x0000001FU)

/* P0_SRC_ID_A_REG */

#define CSL_XGE_CPSW_P0_SRC_ID_A_REG_PORT1_MASK                                (0x000000FFU)
#define CSL_XGE_CPSW_P0_SRC_ID_A_REG_PORT1_SHIFT                               (0x00000000U)
#define CSL_XGE_CPSW_P0_SRC_ID_A_REG_PORT1_MAX                                 (0x000000FFU)

#define CSL_XGE_CPSW_P0_SRC_ID_A_REG_PORT2_MASK                                (0x0000FF00U)
#define CSL_XGE_CPSW_P0_SRC_ID_A_REG_PORT2_SHIFT                               (0x00000008U)
#define CSL_XGE_CPSW_P0_SRC_ID_A_REG_PORT2_MAX                                 (0x000000FFU)

#define CSL_XGE_CPSW_P0_SRC_ID_A_REG_PORT3_MASK                                (0x00FF0000U)
#define CSL_XGE_CPSW_P0_SRC_ID_A_REG_PORT3_SHIFT                               (0x00000010U)
#define CSL_XGE_CPSW_P0_SRC_ID_A_REG_PORT3_MAX                                 (0x000000FFU)

#define CSL_XGE_CPSW_P0_SRC_ID_A_REG_PORT4_MASK                                (0xFF000000U)
#define CSL_XGE_CPSW_P0_SRC_ID_A_REG_PORT4_SHIFT                               (0x00000018U)
#define CSL_XGE_CPSW_P0_SRC_ID_A_REG_PORT4_MAX                                 (0x000000FFU)

/* P0_SRC_ID_B_REG */

#define CSL_XGE_CPSW_P0_SRC_ID_B_REG_PORT5_MASK                                (0x000000FFU)
#define CSL_XGE_CPSW_P0_SRC_ID_B_REG_PORT5_SHIFT                               (0x00000000U)
#define CSL_XGE_CPSW_P0_SRC_ID_B_REG_PORT5_MAX                                 (0x000000FFU)

#define CSL_XGE_CPSW_P0_SRC_ID_B_REG_PORT6_MASK                                (0x0000FF00U)
#define CSL_XGE_CPSW_P0_SRC_ID_B_REG_PORT6_SHIFT                               (0x00000008U)
#define CSL_XGE_CPSW_P0_SRC_ID_B_REG_PORT6_MAX                                 (0x000000FFU)

#define CSL_XGE_CPSW_P0_SRC_ID_B_REG_PORT7_MASK                                (0x00FF0000U)
#define CSL_XGE_CPSW_P0_SRC_ID_B_REG_PORT7_SHIFT                               (0x00000010U)
#define CSL_XGE_CPSW_P0_SRC_ID_B_REG_PORT7_MAX                                 (0x000000FFU)

#define CSL_XGE_CPSW_P0_SRC_ID_B_REG_PORT8_MASK                                (0xFF000000U)
#define CSL_XGE_CPSW_P0_SRC_ID_B_REG_PORT8_SHIFT                               (0x00000018U)
#define CSL_XGE_CPSW_P0_SRC_ID_B_REG_PORT8_MAX                                 (0x000000FFU)

/* P0_HOST_BLKS_PRI_REG */

#define CSL_XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI0_MASK                            (0x0000000FU)
#define CSL_XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI0_SHIFT                           (0x00000000U)
#define CSL_XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI0_MAX                             (0x0000000FU)

#define CSL_XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI1_MASK                            (0x000000F0U)
#define CSL_XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI1_SHIFT                           (0x00000004U)
#define CSL_XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI1_MAX                             (0x0000000FU)

#define CSL_XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI2_MASK                            (0x00000F00U)
#define CSL_XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI2_SHIFT                           (0x00000008U)
#define CSL_XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI2_MAX                             (0x0000000FU)

#define CSL_XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI3_MASK                            (0x0000F000U)
#define CSL_XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI3_SHIFT                           (0x0000000CU)
#define CSL_XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI3_MAX                             (0x0000000FU)

#define CSL_XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI4_MASK                            (0x000F0000U)
#define CSL_XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI4_SHIFT                           (0x00000010U)
#define CSL_XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI4_MAX                             (0x0000000FU)

#define CSL_XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI5_MASK                            (0x00F00000U)
#define CSL_XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI5_SHIFT                           (0x00000014U)
#define CSL_XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI5_MAX                             (0x0000000FU)

#define CSL_XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI6_MASK                            (0x0F000000U)
#define CSL_XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI6_SHIFT                           (0x00000018U)
#define CSL_XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI6_MAX                             (0x0000000FU)

#define CSL_XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI7_MASK                            (0xF0000000U)
#define CSL_XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI7_SHIFT                           (0x0000001CU)
#define CSL_XGE_CPSW_P0_HOST_BLKS_PRI_REG_PRI7_MAX                             (0x0000000FU)

#ifdef __cplusplus
}
#endif
#endif
