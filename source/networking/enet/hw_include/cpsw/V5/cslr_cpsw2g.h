/********************************************************************
 * Copyright (C) 2017-2019 Texas Instruments Incorporated.
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
 *  Name        : cslr_cpsw2g.h
*/
#ifndef CSLR_CPSW2G_H_
#define CSLR_CPSW2G_H_

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
    volatile uint32_t SGMII_IDVER_REG;           /* idver_reg */
    volatile uint32_t SOFT_RESET_REG;            /* soft_reset_reg */
    volatile uint8_t  Resv_16[8];
    volatile uint32_t CONTROL_REG;               /* control_reg */
    volatile uint32_t STATUS_REG;                /* status_reg */
    volatile uint32_t MR_ADV_ABILITY_REG;        /* mr_adv_ability_reg */
    volatile uint32_t MR_NP_TX_REG;              /* mr_np_tx_reg */
    volatile uint32_t MR_LP_ADV_ABILITY_REG;     /* mr_lp_adv_ability_reg */
    volatile uint32_t MR_LP_NP_RX_REG;           /* mr_lp_np_rx_reg */
    volatile uint8_t  Resv_64[24];
    volatile uint32_t DIAG_CLEAR_REG;            /* diag_clear_reg */
    volatile uint32_t DIAG_CONTROL_REG;          /* diag_control_reg */
    volatile uint32_t DIAG_STATUS_REG;           /* diag_status_reg */
    volatile uint8_t  Resv_256[180];
} CSL_cpsw2gRegs_CPSGMII;


typedef struct {
    volatile uint32_t USER_ACCESS_REG;           /* user_access_reg */
    volatile uint32_t USER_PHY_SEL_REG;          /* user_phy_sel_reg */
} CSL_cpsw2gRegs_MDIO_USER_GROUP;


typedef struct {
    volatile uint32_t MDIO_VERSION_REG;          /* version_reg */
    volatile uint32_t CONTROL_REG;               /* control_reg */
    volatile uint32_t ALIVE_REG;                 /* alive_reg */
    volatile uint32_t LINK_REG;                  /* link_reg */
    volatile uint32_t LINK_INT_RAW_REG;          /* link_int_raw_reg */
    volatile uint32_t LINK_INT_MASKED_REG;       /* link_int_masked_reg */
    volatile uint32_t LINK_INT_MASK_SET_REG;     /* link_int_mask_set_reg */
    volatile uint32_t LINK_INT_MASK_CLEAR_REG;   /* link_int_mask_clear_reg */
    volatile uint32_t USER_INT_RAW_REG;          /* user_int_raw_reg */
    volatile uint32_t USER_INT_MASKED_REG;       /* user_int_masked_reg */
    volatile uint32_t USER_INT_MASK_SET_REG;     /* user_int_mask_set_reg */
    volatile uint32_t USER_INT_MASK_CLEAR_REG;   /* user_int_mask_clear_reg */
    volatile uint32_t MANUAL_IF_REG;             /* manual_if_reg */
    volatile uint32_t POLL_REG;                  /* poll_reg */
    volatile uint32_t POLL_EN_REG;               /* poll_reg */
    volatile uint32_t CLAUS45_REG;               /* poll_reg */
    volatile uint32_t USER_ADDR0_REG;            /* poll_reg */
    volatile uint32_t USER_ADDR1_REG;            /* poll_reg */
    volatile uint8_t  Resv_128[56];
    CSL_cpsw2gRegs_MDIO_USER_GROUP USER_GROUP[2];
} CSL_cpsw2gRegs_MDIO;


typedef struct {
    volatile uint32_t REVISION;
    volatile uint32_t CONTROL;
    volatile uint8_t  Resv_16[8];
    volatile uint32_t EOI_REG;
    volatile uint32_t INTR_VECTOR_REG;
    volatile uint8_t  Resv_256[232];
    volatile uint32_t ENABLE_REG_EVNT_PULSE0_0;
    volatile uint32_t ENABLE_REG_STAT_PULSE0_0;
    volatile uint32_t ENABLE_REG_STAT_PULSE1_0;
    volatile uint8_t  Resv_768[500];
    volatile uint32_t ENABLE_CLR_REG_EVNT_PULSE0_0;
    volatile uint32_t ENABLE_CLR_REG_STAT_PULSE0_0;
    volatile uint32_t ENABLE_CLR_REG_STAT_PULSE1_0;
    volatile uint8_t  Resv_1280[500];
    volatile uint32_t STATUS_REG_EVNT_PULSE0_0;
    volatile uint32_t STATUS_REG_STAT_PULSE0_0;
    volatile uint32_t STATUS_REG_STAT_PULSE1_0;
    volatile uint8_t  Resv_1792[500];
    volatile uint32_t STATUS_CLR_REG_EVNT_PULSE0_0;
    volatile uint32_t STATUS_CLR_REG_STAT_PULSE0_0;
    volatile uint32_t STATUS_CLR_REG_STAT_PULSE1_0;
    volatile uint8_t  Resv_2688[884];
    volatile uint32_t INTR_VECTOR_REG_EVNT_PULSE0;
    volatile uint32_t INTR_VECTOR_REG_STAT_PULSE0;
    volatile uint32_t INTR_VECTOR_REG_STAT_PULSE1;
} CSL_cpsw2gRegs_INTD;


typedef struct {
    volatile uint8_t  Resv_4[4];
    volatile uint32_t P0_CONTROL_REG;            /* p0_control_reg */
    volatile uint32_t P0_FLOW_ID_OFFSET_REG;     /* p0_flow_id_offset_reg */
    volatile uint8_t  Resv_16[4];
    volatile uint32_t P0_BLK_CNT_REG;            /* p0_blk_cnt_reg */
    volatile uint32_t P0_PORT_VLAN_REG;          /* p0_port_vlan_reg */
    volatile uint32_t P0_TX_PRI_MAP_REG;         /* p0_tx_pri_map_reg */
    volatile uint32_t P0_PRI_CTL_REG;            /* p0_pri_ctl_reg */
    volatile uint32_t P0_RX_PRI_MAP_REG;         /* p0_rx_pri_map_reg */
    volatile uint32_t P0_RX_MAXLEN_REG;          /* p0_rx_maxlen_reg */
    volatile uint32_t P0_TX_BLKS_PRI_REG;        /* p0_tx_blks_pri_reg */
    volatile uint8_t  Resv_48[4];
    volatile uint32_t P0_IDLE2LPI_REG;           /* p0_idle2lpi_reg */
    volatile uint32_t P0_LPI2WAKE_REG;           /* p0_lpi2wake_reg */
    volatile uint32_t P0_EEE_STATUS_REG;         /* p0_eee_status_reg */
    volatile uint32_t P0_RX_PKTS_PRI_REG;        /* p0_rx_pkts_pri_reg */
    volatile uint8_t  Resv_76[12];
    volatile uint32_t P0_RX_GAP_REG;             /* p0_rx_gap_reg */
    volatile uint32_t P0_FIFO_STATUS_REG;        /* p0_fifo_status_reg */
    volatile uint8_t  Resv_288[204];
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
    volatile uint8_t  Resv_768[352];
    volatile uint32_t P0_SRC_ID_A_REG;           /* p0_src_id_a_reg */
    volatile uint32_t P0_SRC_ID_B_REG;           /* p0_src_id_b_reg */
    volatile uint8_t  Resv_800[24];
    volatile uint32_t P0_HOST_BLKS_PRI_REG;      /* p0_host_blks_pri_reg */
} CSL_cpsw2gRegs_CPSW_NU_CPSW_NU_CPPI;


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
    volatile uint8_t  Resv_80[20];
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
    volatile uint8_t  Resv_4096[3160];
} CSL_cpsw2gRegs_CPSW_NU_CPSW_NU_ETH;


typedef struct {
    volatile uint32_t FETCH_LOC[128];            /* RAM Location */
} CSL_cpsw2gRegs_CPSW_NU_CPSW_NU_EST;


typedef struct {
    volatile uint32_t RXGOODFRAMES;              /* RxGoodFrames */
    volatile uint32_t RXBROADCASTFRAMES;         /* RxBroadcastFrames */
    volatile uint32_t RXMULTICASTFRAMES;         /* RxMulticastFrames */
    volatile uint8_t  Resv_16[4];
    volatile uint32_t RXCRCERRORS;               /* RxCRCErrors */
    volatile uint8_t  Resv_24[4];
    volatile uint32_t RXOVERSIZEDFRAMES;         /* RxOversizedFrames */
    volatile uint8_t  Resv_32[4];
    volatile uint32_t RXUNDERSIZEDFRAMES;        /* RxUndersizedFrames */
    volatile uint32_t RXFRAGMENTS;               /* RxFragments */
    volatile uint32_t ALE_DROP;                  /* ALE_Drop */
    volatile uint32_t ALE_OVERRUN_DROP;          /* ALE_Overrun_Drop */
    volatile uint32_t RXOCTETS;                  /* RxOctets */
    volatile uint32_t TXGOODFRAMES;              /* TxGoodFrames */
    volatile uint32_t TXBROADCASTFRAMES;         /* TxBroadcastFrames */
    volatile uint32_t TXMULTICASTFRAMES;         /* TxMulticastFrames */
    volatile uint8_t  Resv_100[36];
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
    volatile uint8_t  Resv_380[176];
    volatile uint32_t TX_MEMORY_PROTECT_ERROR;   /* Tx_Memory_Protect_Error */
} CSL_cpsw2gRegs_CPSW_NU_CPSW_NU_STAT_0;


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
    volatile uint8_t  Resv_380[176];
    volatile uint32_t TX_MEMORY_PROTECT_ERROR;   /* Tx_Memory_Protect_Error */
    volatile uint32_t ENET_PN_TX_PRI_REG[8];     /* enet_pn_tx_pri */
    volatile uint32_t ENET_PN_TX_PRI_BCNT_REG[8];   /* enet_pn_tx_pri_bcnt */
    volatile uint32_t ENET_PN_TX_PRI_DROP_REG[8];   /* enet_pn_tx_pri_drop */
    volatile uint32_t ENET_PN_TX_PRI_DROP_BCNT_REG[8];   /* enet_pn_tx_pri_drop_bcnt */
} CSL_cpsw2gRegs_CPSW_NU_CPSW_NU_STAT_1;


typedef struct {
    volatile uint32_t COMP_LOW_REG;              /* comp_low_reg */
    volatile uint32_t COMP_HIGH_REG;             /* comp_high_reg */
    volatile uint32_t CONTROL_REG;               /* control_reg */
    volatile uint32_t LENGTH_REG;                /* length_reg */
    volatile uint32_t PPM_LOW_REG;               /* ppm_low_reg */
    volatile uint32_t PPM_HIGH_REG;              /* ppm_high_reg */
    volatile uint32_t NUDGE_REG;                 /* nudge_reg */
    volatile uint8_t  Resv_32[4];
} CSL_cpsw2gRegs_CPSW_NU_CPTS_TS_GENF;


typedef struct {
    volatile uint32_t COMP_LOW_REG;              /* comp_low_reg */
    volatile uint32_t COMP_HIGH_REG;             /* comp_high_reg */
    volatile uint32_t CONTROL_REG;               /* control_reg */
    volatile uint32_t LENGTH_REG;                /* length_reg */
    volatile uint32_t PPM_LOW_REG;               /* ppm_low_reg */
    volatile uint32_t PPM_HIGH_REG;              /* ppm_high_reg */
    volatile uint32_t NUDGE_REG;                 /* nudge_reg */
    volatile uint8_t  Resv_32[4];
} CSL_cpsw2gRegs_CPSW_NU_CPTS_TS_ESTF;


typedef struct {
    volatile uint32_t IDVER_REG;                 /* idver_reg */
    volatile uint32_t CONTROL_REG;               /* control_reg */
    volatile uint32_t RFTCLK_SEL_REG;            /* rftclk_sel_reg */
    volatile uint32_t TS_PUSH_REG;               /* ts_push_reg */
    volatile uint32_t TS_LOAD_VAL_REG;           /* ts_load_low_val_reg */
    volatile uint32_t TS_LOAD_EN_REG;            /* ts_load_en_reg */
    volatile uint32_t TS_COMP_VAL_REG;           /* ts_comp_low_val_reg */
    volatile uint32_t TS_COMP_LEN_REG;           /* ts_comp_len_reg */
    volatile uint32_t INTSTAT_RAW_REG;           /* intstat_raw_reg */
    volatile uint32_t INTSTAT_MASKED_REG;        /* intstat_masked_reg */
    volatile uint32_t INT_ENABLE_REG;            /* int_enable_reg */
    volatile uint32_t TS_COMP_NUDGE_REG;         /* ts_comp_nudge_reg */
    volatile uint32_t EVENT_POP_REG;             /* event_pop_reg */
    volatile uint32_t EVENT_0_REG;               /* event_0_reg */
    volatile uint32_t EVENT_1_REG;               /* event_1_reg */
    volatile uint32_t EVENT_2_REG;               /* event_2_reg */
    volatile uint32_t EVENT_3_REG;               /* event_3_reg */
    volatile uint32_t TS_LOAD_HIGH_VAL_REG;      /* ts_load_high_val_reg */
    volatile uint32_t TS_COMP_HIGH_VAL_REG;      /* ts_comp_high_val_reg */
    volatile uint32_t TS_ADD_VAL_REG;            /* ts_add_val */
    volatile uint32_t TS_PPM_LOW_VAL_REG;        /* ts_ppm_low_val_reg */
    volatile uint32_t TS_PPM_HIGH_VAL_REG;       /* ts_ppm_high_val_reg */
    volatile uint32_t TS_NUDGE_VAL_REG;          /* ts_nudge_val_reg */
    volatile uint8_t  Resv_224[132];
    CSL_cpsw2gRegs_CPSW_NU_CPTS_TS_GENF TS_GENF[2];
    volatile uint8_t  Resv_512[224];
    CSL_cpsw2gRegs_CPSW_NU_CPTS_TS_ESTF TS_ESTF[1];
} CSL_cpsw2gRegs_CPSW_NU_CPTS;


typedef struct {
    volatile uint32_t IDVER_REG;                 /* idver_reg */
    volatile uint32_t STATUS_REG;                /* status_reg */
    volatile uint32_t CONTROL_REG;               /* control_reg */
    volatile uint32_t CONTROL2_REG;              /* control2_reg */
    volatile uint32_t PRESCALE_REG;              /* prescale_reg */
    volatile uint32_t AGING_TIMER_REG;           /* aging_timer_reg */
    volatile uint8_t  Resv_32[8];
    volatile uint32_t TABLE_CONTROL_REG;         /* table_control_reg */
    volatile uint8_t  Resv_52[16];
    volatile uint32_t TABLE_WORD2_REG;           /* table_word2_reg */
    volatile uint32_t TABLE_WORD1_REG;           /* table_word1_reg */
    volatile uint32_t TABLE_WORD0_REG;           /* table_word0_reg */
    volatile uint32_t PORT_CONTROL_REG[2];       /* port_control_reg */
    volatile uint8_t  Resv_144[72];
    volatile uint32_t UNKNOWN_VLAN_REG;          /* unknown_vlan_reg */
    volatile uint32_t UNKNOWN_MCAST_FLOOD_REG;   /* unknown_mcast_flood_reg */
    volatile uint32_t UNKNOWN_REG_MCAST_FLOOD_REG;   /* unknown_reg_mcast_flood_reg */
    volatile uint32_t FORCE_UNTAGGED_EGRESS_REG;   /* force_untagged_egress_reg */
    volatile uint8_t  Resv_192[32];
    volatile uint32_t VLAN_MASK_MUX0_REG;        /* vlan_mask_mux_reg */
    volatile uint32_t VLAN_MASK_MUX1_REG;        /* vlan_mask_mux_reg */
    volatile uint32_t VLAN_MASK_MUX2_REG;        /* vlan_mask_mux_reg */
    volatile uint32_t VLAN_MASK_MUX3_REG;        /* vlan_mask_mux_reg */
    volatile uint32_t VLAN_MASK_MUX4_REG;        /* vlan_mask_mux_reg */
    volatile uint32_t VLAN_MASK_MUX5_REG;        /* vlan_mask_mux_reg */
    volatile uint32_t VLAN_MASK_MUX6_REG;        /* vlan_mask_mux_reg */
    volatile uint32_t VLAN_MASK_MUX7_REG;        /* vlan_mask_mux_reg */
    volatile uint8_t  Resv_256[32];
    volatile uint32_t POLICER_PORT_OUI_REG;      /* policer_port_oui_reg */
    volatile uint32_t POLICER_DA_SA_REG;         /* policer_da_sa_reg */
    volatile uint32_t POLICER_VLAN_REG;          /* policer_vlan_reg */
    volatile uint32_t POLICER_ETHERTYPE_IPSA_REG;   /* policer_ethertype_ipsa_reg */
    volatile uint32_t POLICER_IPDA_REG;          /* policer_ipda_reg */
    volatile uint8_t  Resv_280[4];
    volatile uint32_t POLICER_PIR_REG;           /* policer_pir_reg */
    volatile uint32_t POLICER_CIR_REG;           /* policer_cir_reg */
    volatile uint32_t POLICER_TBL_CTL_REG;       /* policer_tbl_ctl_reg */
    volatile uint32_t POLICER_CTL_REG;           /* policer_ctl_reg */
    volatile uint32_t POLICER_TEST_CTL_REG;      /* policer_test_ctl_reg */
    volatile uint32_t POLICER_HIT_STATUS_REG;    /* policer_hit_status_reg */
    volatile uint8_t  Resv_308[4];
    volatile uint32_t THREAD_DEF_REG;            /* thread_def_reg */
    volatile uint32_t THREAD_CTL_REG;            /* thread_ctl_reg */
    volatile uint32_t THREAD_VAL_REG;            /* thread_val_reg */
} CSL_cpsw2gRegs_CPSW_NU_ALE;


typedef struct {
    volatile uint32_t CPSW_ID_VER_REG;           /* idver_reg */
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
    volatile uint8_t  Resv_4096[3808];
    CSL_cpsw2gRegs_CPSW_NU_CPSW_NU_CPPI CPSW_NU_CPPI;
    volatile uint8_t  Resv_8192[3292];
    CSL_cpsw2gRegs_CPSW_NU_CPSW_NU_ETH CPSW_NU_ETH;
    volatile uint8_t  Resv_73728[61440];
    CSL_cpsw2gRegs_CPSW_NU_CPSW_NU_EST CPSW_NU_EST;
    volatile uint8_t  Resv_106496[32256];
    CSL_cpsw2gRegs_CPSW_NU_CPSW_NU_STAT_0 CPSW_NU_STAT_0;
    volatile uint8_t  Resv_107008[128];
    CSL_cpsw2gRegs_CPSW_NU_CPSW_NU_STAT_1 CPSW_NU_STAT_1;
    volatile uint8_t  Resv_118784[11264];
    CSL_cpsw2gRegs_CPSW_NU_CPTS CPTS;
    volatile uint8_t  Resv_122880[3552];
    CSL_cpsw2gRegs_CPSW_NU_ALE ALE;
} CSL_cpsw2gRegs_CPSW_NU;


typedef struct {
    volatile uint32_t CPSW_NUSS_IDVER_REG;       /* ID Version Register */
    volatile uint32_t SYNCE_COUNT_REG;           /* SyncE Count Register */
    volatile uint32_t SYNCE_MUX_REG;             /* SyncE Mux Register */
    volatile uint32_t CONTROL_REG;               /* Control Register */
    volatile uint32_t SGMII_MODE_REG;            /* SyncE Mux Register */
    volatile uint8_t  Resv_24[4];
    volatile uint32_t RGMII_STATUS_REG;          /* RGMII Status Register */
    volatile uint32_t SUBSSYSTEM_STATUS_REG;     /* Subsystem Status Register */
    volatile uint8_t  Resv_256[224];
    CSL_cpsw2gRegs_CPSGMII CPSGMII;
    volatile uint8_t  Resv_3840[3328];
    CSL_cpsw2gRegs_MDIO MDIO;
    volatile uint8_t  Resv_4096[112];
    CSL_cpsw2gRegs_INTD INTD;
    volatile uint8_t  Resv_131072[124276];
    CSL_cpsw2gRegs_CPSW_NU CPSW_NU;
} CSL_cpsw2gRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_CPSW2G_CPSW_NUSS_IDVER_REG                                         (0x00000000U)
#define CSL_CPSW2G_SYNCE_COUNT_REG                                             (0x00000004U)
#define CSL_CPSW2G_SYNCE_MUX_REG                                               (0x00000008U)
#define CSL_CPSW2G_CONTROL_REG                                                 (0x0000000CU)
#define CSL_CPSW2G_SGMII_MODE_REG                                              (0x00000010U)
#define CSL_CPSW2G_RGMII_STATUS_REG                                            (0x00000018U)
#define CSL_CPSW2G_SUBSSYSTEM_STATUS_REG                                       (0x0000001CU)
#define CSL_CPSW2G_CPSGMII_SGMII_IDVER_REG                                     (0x00000100U)
#define CSL_CPSW2G_CPSGMII_SOFT_RESET_REG                                      (0x00000104U)
#define CSL_CPSW2G_CPSGMII_CONTROL_REG                                         (0x00000110U)
#define CSL_CPSW2G_CPSGMII_STATUS_REG                                          (0x00000114U)
#define CSL_CPSW2G_CPSGMII_MR_ADV_ABILITY_REG                                  (0x00000118U)
#define CSL_CPSW2G_CPSGMII_MR_NP_TX_REG                                        (0x0000011CU)
#define CSL_CPSW2G_CPSGMII_MR_LP_ADV_ABILITY_REG                               (0x00000120U)
#define CSL_CPSW2G_CPSGMII_MR_LP_NP_RX_REG                                     (0x00000124U)
#define CSL_CPSW2G_CPSGMII_DIAG_CLEAR_REG                                      (0x00000140U)
#define CSL_CPSW2G_CPSGMII_DIAG_CONTROL_REG                                    (0x00000144U)
#define CSL_CPSW2G_CPSGMII_DIAG_STATUS_REG                                     (0x00000148U)
#define CSL_CPSW2G_MDIO_MDIO_VERSION_REG                                       (0x00000F00U)
#define CSL_CPSW2G_MDIO_CONTROL_REG                                            (0x00000F04U)
#define CSL_CPSW2G_MDIO_ALIVE_REG                                              (0x00000F08U)
#define CSL_CPSW2G_MDIO_LINK_REG                                               (0x00000F0CU)
#define CSL_CPSW2G_MDIO_LINK_INT_RAW_REG                                       (0x00000F10U)
#define CSL_CPSW2G_MDIO_LINK_INT_MASKED_REG                                    (0x00000F14U)
#define CSL_CPSW2G_MDIO_LINK_INT_MASK_SET_REG                                  (0x00000F18U)
#define CSL_CPSW2G_MDIO_LINK_INT_MASK_CLEAR_REG                                (0x00000F1CU)
#define CSL_CPSW2G_MDIO_USER_INT_RAW_REG                                       (0x00000F20U)
#define CSL_CPSW2G_MDIO_USER_INT_MASKED_REG                                    (0x00000F24U)
#define CSL_CPSW2G_MDIO_USER_INT_MASK_SET_REG                                  (0x00000F28U)
#define CSL_CPSW2G_MDIO_USER_INT_MASK_CLEAR_REG                                (0x00000F2CU)
#define CSL_CPSW2G_MDIO_MANUAL_IF_REG                                          (0x00000F30U)
#define CSL_CPSW2G_MDIO_POLL_REG                                               (0x00000F34U)
#define CSL_CPSW2G_MDIO_POLL_EN_REG                                            (0x00000F38U)
#define CSL_CPSW2G_MDIO_CLAUS45_REG                                            (0x00000F3CU)
#define CSL_CPSW2G_MDIO_USER_ADDR0_REG                                         (0x00000F40U)
#define CSL_CPSW2G_MDIO_USER_ADDR1_REG                                         (0x00000F44U)
#define CSL_CPSW2G_MDIO_USER_GROUP_USER_ACCESS_REG(USER_GROUP)                 (0x00000F80U+((USER_GROUP)*0x8U))
#define CSL_CPSW2G_MDIO_USER_GROUP_USER_PHY_SEL_REG(USER_GROUP)                (0x00000F84U+((USER_GROUP)*0x8U))
#define CSL_CPSW2G_INTD_REVISION                                               (0x00001000U)
#define CSL_CPSW2G_INTD_CONTROL                                                (0x00001004U)
#define CSL_CPSW2G_INTD_EOI_REG                                                (0x00001010U)
#define CSL_CPSW2G_INTD_INTR_VECTOR_REG                                        (0x00001014U)
#define CSL_CPSW2G_INTD_ENABLE_REG_EVNT_PULSE0_0                               (0x00001100U)
#define CSL_CPSW2G_INTD_ENABLE_REG_STAT_PULSE0_0                               (0x00001104U)
#define CSL_CPSW2G_INTD_ENABLE_REG_STAT_PULSE1_0                               (0x00001108U)
#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_EVNT_PULSE0_0                           (0x00001300U)
#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_STAT_PULSE0_0                           (0x00001304U)
#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_STAT_PULSE1_0                           (0x00001308U)
#define CSL_CPSW2G_INTD_STATUS_REG_EVNT_PULSE0_0                               (0x00001500U)
#define CSL_CPSW2G_INTD_STATUS_REG_STAT_PULSE0_0                               (0x00001504U)
#define CSL_CPSW2G_INTD_STATUS_REG_STAT_PULSE1_0                               (0x00001508U)
#define CSL_CPSW2G_INTD_STATUS_CLR_REG_EVNT_PULSE0_0                           (0x00001700U)
#define CSL_CPSW2G_INTD_STATUS_CLR_REG_STAT_PULSE0_0                           (0x00001704U)
#define CSL_CPSW2G_INTD_STATUS_CLR_REG_STAT_PULSE1_0                           (0x00001708U)
#define CSL_CPSW2G_INTD_INTR_VECTOR_REG_EVNT_PULSE0                            (0x00001A80U)
#define CSL_CPSW2G_INTD_INTR_VECTOR_REG_STAT_PULSE0                            (0x00001A84U)
#define CSL_CPSW2G_INTD_INTR_VECTOR_REG_STAT_PULSE1                            (0x00001A88U)
#define CSL_CPSW2G_CPSW_NU_CPSW_ID_VER_REG                                     (0x00020000U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG                                         (0x00020004U)
#define CSL_CPSW2G_CPSW_NU_EM_CONTROL_REG                                      (0x00020010U)
#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG                                    (0x00020014U)
#define CSL_CPSW2G_CPSW_NU_PTYPE_REG                                           (0x00020018U)
#define CSL_CPSW2G_CPSW_NU_SOFT_IDLE_REG                                       (0x0002001CU)
#define CSL_CPSW2G_CPSW_NU_THRU_RATE_REG                                       (0x00020020U)
#define CSL_CPSW2G_CPSW_NU_GAP_THRESH_REG                                      (0x00020024U)
#define CSL_CPSW2G_CPSW_NU_TX_START_WDS_REG                                    (0x00020028U)
#define CSL_CPSW2G_CPSW_NU_EEE_PRESCALE_REG                                    (0x0002002CU)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_SET_REG                           (0x00020030U)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_CLR_REG                           (0x00020034U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_SET_L_REG                           (0x00020038U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_SET_H_REG                           (0x0002003CU)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_CLR_L_REG                           (0x00020040U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_CLR_H_REG                           (0x00020044U)
#define CSL_CPSW2G_CPSW_NU_VLAN_LTYPE_REG                                      (0x00020050U)
#define CSL_CPSW2G_CPSW_NU_EST_TS_DOMAIN_REG                                   (0x00020054U)
#define CSL_CPSW2G_CPSW_NU_TX_PRI0_MAXLEN_REG                                  (0x00020100U)
#define CSL_CPSW2G_CPSW_NU_TX_PRI1_MAXLEN_REG                                  (0x00020104U)
#define CSL_CPSW2G_CPSW_NU_TX_PRI2_MAXLEN_REG                                  (0x00020108U)
#define CSL_CPSW2G_CPSW_NU_TX_PRI3_MAXLEN_REG                                  (0x0002010CU)
#define CSL_CPSW2G_CPSW_NU_TX_PRI4_MAXLEN_REG                                  (0x00020110U)
#define CSL_CPSW2G_CPSW_NU_TX_PRI5_MAXLEN_REG                                  (0x00020114U)
#define CSL_CPSW2G_CPSW_NU_TX_PRI6_MAXLEN_REG                                  (0x00020118U)
#define CSL_CPSW2G_CPSW_NU_TX_PRI7_MAXLEN_REG                                  (0x0002011CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_CONTROL_REG                         (0x00021004U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_FLOW_ID_OFFSET_REG                  (0x00021008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_BLK_CNT_REG                         (0x00021010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_PORT_VLAN_REG                       (0x00021014U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_PRI_MAP_REG                      (0x00021018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_PRI_CTL_REG                         (0x0002101CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PRI_MAP_REG                      (0x00021020U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_MAXLEN_REG                       (0x00021024U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_BLKS_PRI_REG                     (0x00021028U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_IDLE2LPI_REG                        (0x00021030U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_LPI2WAKE_REG                        (0x00021034U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_EEE_STATUS_REG                      (0x00021038U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PKTS_PRI_REG                     (0x0002103CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_GAP_REG                          (0x0002104CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_FIFO_STATUS_REG                     (0x00021050U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_DSCP_MAP_REG(P0_RX_DSCP_MAP_REG) (0x00021120U+((P0_RX_DSCP_MAP_REG)*0x4U))
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_PRI_CIR_REG(P0_PRI_CIR_REG)         (0x00021140U+((P0_PRI_CIR_REG)*0x4U))
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_PRI_EIR_REG(P0_PRI_EIR_REG)         (0x00021160U+((P0_PRI_EIR_REG)*0x4U))
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_SET_L_REG               (0x00021180U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_SET_H_REG               (0x00021184U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_CLR_L_REG               (0x00021188U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_CLR_H_REG               (0x0002118CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_SET_L_REG           (0x00021190U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_SET_H_REG           (0x00021194U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_CLR_L_REG           (0x00021198U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_CLR_H_REG           (0x0002119CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_SRC_ID_A_REG                        (0x00021300U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_SRC_ID_B_REG                        (0x00021304U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_HOST_BLKS_PRI_REG                   (0x00021320U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RESERVED_REG                         (0x00022000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_CONTROL_REG                          (0x00022004U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAX_BLKS_REG                         (0x00022008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_BLK_CNT_REG                          (0x00022010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PORT_VLAN_REG                        (0x00022014U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_PRI_MAP_REG                       (0x00022018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PRI_CTL_REG                          (0x0002201CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_PRI_MAP_REG                       (0x00022020U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_MAXLEN_REG                        (0x00022024U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_BLKS_PRI_REG                      (0x00022028U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_FLOW_THRESH_REG                   (0x0002202CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_IDLE2LPI_REG                         (0x00022030U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_LPI2WAKE_REG                         (0x00022034U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EEE_STATUS_REG                       (0x00022038U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_FIFO_STATUS_REG                      (0x00022050U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EST_CONTROL_REG                      (0x00022060U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_DSCP_MAP_REG(PN_RX_DSCP_MAP_REG)  (0x00022120U+((PN_RX_DSCP_MAP_REG)*0x4U))
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PRI_CIR_REG(PN_PRI_CIR_REG)          (0x00022140U+((PN_PRI_CIR_REG)*0x4U))
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PRI_EIR_REG(PN_PRI_EIR_REG)          (0x00022160U+((PN_PRI_EIR_REG)*0x4U))
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_SET_L_REG                (0x00022180U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_SET_H_REG                (0x00022184U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_CLR_L_REG                (0x00022188U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_CLR_H_REG                (0x0002218CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_SET_L_REG            (0x00022190U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_SET_H_REG            (0x00022194U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_CLR_L_REG            (0x00022198U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_CLR_H_REG            (0x0002219CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_OFLOW_ADDVAL_L_REG              (0x00022300U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_OFLOW_ADDVAL_H_REG              (0x00022304U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_SA_L_REG                             (0x00022308U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_SA_H_REG                             (0x0002230CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG                           (0x00022310U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_SEQ_LTYPE_REG                     (0x00022314U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_VLAN_LTYPE_REG                    (0x00022318U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG                    (0x0002231CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL2_REG                          (0x00022320U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG                      (0x00022330U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG                       (0x00022334U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_SOFT_RESET_REG                   (0x00022338U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_BOFFTEST_REG                     (0x0002233CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_RX_PAUSETIMER_REG                (0x00022340U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_RXN_PAUSETIMER_REG(PN_MAC_RXN_PAUSETIMER_REG) (0x00022350U+((PN_MAC_RXN_PAUSETIMER_REG)*0x4U))
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_TX_PAUSETIMER_REG                (0x00022370U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_TXN_PAUSETIMER_REG(PN_MAC_TXN_PAUSETIMER_REG) (0x00022380U+((PN_MAC_TXN_PAUSETIMER_REG)*0x4U))
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_EMCONTROL_REG                    (0x000223A0U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_TX_GAP_REG                       (0x000223A4U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_EST_FETCH_LOC(FETCH_LOC)                    (0x00032000U+((FETCH_LOC)*0x4U))
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXGOODFRAMES                         (0x0003A000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXBROADCASTFRAMES                    (0x0003A004U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXMULTICASTFRAMES                    (0x0003A008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXCRCERRORS                          (0x0003A010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXOVERSIZEDFRAMES                    (0x0003A018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXUNDERSIZEDFRAMES                   (0x0003A020U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXFRAGMENTS                          (0x0003A024U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_DROP                             (0x0003A028U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_OVERRUN_DROP                     (0x0003A02CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXOCTETS                             (0x0003A030U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_TXGOODFRAMES                         (0x0003A034U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_TXBROADCASTFRAMES                    (0x0003A038U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_TXMULTICASTFRAMES                    (0x0003A03CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_TXOCTETS                             (0x0003A064U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_OCTETFRAMES64                        (0x0003A068U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_OCTETFRAMES65T127                    (0x0003A06CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_OCTETFRAMES128T255                   (0x0003A070U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_OCTETFRAMES256T511                   (0x0003A074U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_OCTETFRAMES512T1023                  (0x0003A078U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_OCTETFRAMES1024TUP                   (0x0003A07CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_NETOCTETS                            (0x0003A080U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RX_BOTTOM_OF_FIFO_DROP               (0x0003A084U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_PORTMASK_DROP                        (0x0003A088U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RX_TOP_OF_FIFO_DROP                  (0x0003A08CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_RATE_LIMIT_DROP                  (0x0003A090U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_VID_INGRESS_DROP                 (0x0003A094U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_DA_EQ_SA_DROP                    (0x0003A098U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_BLOCK_DROP                       (0x0003A09CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_SECURE_DROP                      (0x0003A0A0U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_AUTH_DROP                        (0x0003A0A4U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_UNKN_UNI                         (0x0003A0A8U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_UNKN_UNI_BCNT                    (0x0003A0ACU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_UNKN_MLT                         (0x0003A0B0U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_UNKN_MLT_BCNT                    (0x0003A0B4U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_UNKN_BRD                         (0x0003A0B8U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_UNKN_BRD_BCNT                    (0x0003A0BCU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_POL_MATCH                        (0x0003A0C0U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_POL_MATCH_RED                    (0x0003A0C4U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_POL_MATCH_YELLOW                 (0x0003A0C8U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_TX_MEMORY_PROTECT_ERROR              (0x0003A17CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXGOODFRAMES                         (0x0003A200U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXBROADCASTFRAMES                    (0x0003A204U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXMULTICASTFRAMES                    (0x0003A208U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXPAUSEFRAMES                        (0x0003A20CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXCRCERRORS                          (0x0003A210U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXALIGNCODEERRORS                    (0x0003A214U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXOVERSIZEDFRAMES                    (0x0003A218U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXJABBERFRAMES                       (0x0003A21CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXUNDERSIZEDFRAMES                   (0x0003A220U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXFRAGMENTS                          (0x0003A224U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_DROP                             (0x0003A228U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_OVERRUN_DROP                     (0x0003A22CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXOCTETS                             (0x0003A230U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXGOODFRAMES                         (0x0003A234U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXBROADCASTFRAMES                    (0x0003A238U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXMULTICASTFRAMES                    (0x0003A23CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXPAUSEFRAMES                        (0x0003A240U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXDEFERREDFRAMES                     (0x0003A244U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXCOLLISIONFRAMES                    (0x0003A248U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXSINGLECOLLFRAMES                   (0x0003A24CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXMULTCOLLFRAMES                     (0x0003A250U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXEXCESSIVECOLLISIONS                (0x0003A254U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXLATECOLLISIONS                     (0x0003A258U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXIPGERROR                           (0x0003A25CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXCARRIERSENSEERRORS                 (0x0003A260U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXOCTETS                             (0x0003A264U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_OCTETFRAMES64                        (0x0003A268U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_OCTETFRAMES65T127                    (0x0003A26CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_OCTETFRAMES128T255                   (0x0003A270U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_OCTETFRAMES256T511                   (0x0003A274U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_OCTETFRAMES512T1023                  (0x0003A278U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_OCTETFRAMES1024TUP                   (0x0003A27CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_NETOCTETS                            (0x0003A280U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RX_BOTTOM_OF_FIFO_DROP               (0x0003A284U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_PORTMASK_DROP                        (0x0003A288U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RX_TOP_OF_FIFO_DROP                  (0x0003A28CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_RATE_LIMIT_DROP                  (0x0003A290U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_VID_INGRESS_DROP                 (0x0003A294U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_DA_EQ_SA_DROP                    (0x0003A298U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_BLOCK_DROP                       (0x0003A29CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_SECURE_DROP                      (0x0003A2A0U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_AUTH_DROP                        (0x0003A2A4U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_UNKN_UNI                         (0x0003A2A8U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_UNKN_UNI_BCNT                    (0x0003A2ACU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_UNKN_MLT                         (0x0003A2B0U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_UNKN_MLT_BCNT                    (0x0003A2B4U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_UNKN_BRD                         (0x0003A2B8U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_UNKN_BRD_BCNT                    (0x0003A2BCU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_POL_MATCH                        (0x0003A2C0U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_POL_MATCH_RED                    (0x0003A2C4U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_POL_MATCH_YELLOW                 (0x0003A2C8U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TX_MEMORY_PROTECT_ERROR              (0x0003A37CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ENET_PN_TX_PRI_REG(ENET_PN_TX_PRI_REG) (0x0003A380U+((ENET_PN_TX_PRI_REG)*0x4U))
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ENET_PN_TX_PRI_BCNT_REG(ENET_PN_TX_PRI_BCNT_REG) (0x0003A3A0U+((ENET_PN_TX_PRI_BCNT_REG)*0x4U))
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ENET_PN_TX_PRI_DROP_REG(ENET_PN_TX_PRI_DROP_REG) (0x0003A3C0U+((ENET_PN_TX_PRI_DROP_REG)*0x4U))
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ENET_PN_TX_PRI_DROP_BCNT_REG(ENET_PN_TX_PRI_DROP_BCNT_REG) (0x0003A3E0U+((ENET_PN_TX_PRI_DROP_BCNT_REG)*0x4U))
#define CSL_CPSW2G_CPSW_NU_CPTS_IDVER_REG                                      (0x0003D000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG                                    (0x0003D004U)
#define CSL_CPSW2G_CPSW_NU_CPTS_RFTCLK_SEL_REG                                 (0x0003D008U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_PUSH_REG                                    (0x0003D00CU)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_LOAD_VAL_REG                                (0x0003D010U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_LOAD_EN_REG                                 (0x0003D014U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_COMP_VAL_REG                                (0x0003D018U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_COMP_LEN_REG                                (0x0003D01CU)
#define CSL_CPSW2G_CPSW_NU_CPTS_INTSTAT_RAW_REG                                (0x0003D020U)
#define CSL_CPSW2G_CPSW_NU_CPTS_INTSTAT_MASKED_REG                             (0x0003D024U)
#define CSL_CPSW2G_CPSW_NU_CPTS_INT_ENABLE_REG                                 (0x0003D028U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_COMP_NUDGE_REG                              (0x0003D02CU)
#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_POP_REG                                  (0x0003D030U)
#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_0_REG                                    (0x0003D034U)
#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_1_REG                                    (0x0003D038U)
#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_2_REG                                    (0x0003D03CU)
#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_3_REG                                    (0x0003D040U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_LOAD_HIGH_VAL_REG                           (0x0003D044U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_COMP_HIGH_VAL_REG                           (0x0003D048U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ADD_VAL_REG                                 (0x0003D04CU)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_PPM_LOW_VAL_REG                             (0x0003D050U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_PPM_HIGH_VAL_REG                            (0x0003D054U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_NUDGE_VAL_REG                               (0x0003D058U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_COMP_LOW_REG(TS_GENF)                  (0x0003D0E0U+((TS_GENF)*0x20U))
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_COMP_HIGH_REG(TS_GENF)                 (0x0003D0E4U+((TS_GENF)*0x20U))
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_CONTROL_REG(TS_GENF)                   (0x0003D0E8U+((TS_GENF)*0x20U))
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_LENGTH_REG(TS_GENF)                    (0x0003D0ECU+((TS_GENF)*0x20U))
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_PPM_LOW_REG(TS_GENF)                   (0x0003D0F0U+((TS_GENF)*0x20U))
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_PPM_HIGH_REG(TS_GENF)                  (0x0003D0F4U+((TS_GENF)*0x20U))
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_NUDGE_REG(TS_GENF)                     (0x0003D0F8U+((TS_GENF)*0x20U))
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_COMP_LOW_REG                           (0x0003D200U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_COMP_HIGH_REG                          (0x0003D204U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_CONTROL_REG                            (0x0003D208U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_LENGTH_REG                             (0x0003D20CU)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_PPM_LOW_REG                            (0x0003D210U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_PPM_HIGH_REG                           (0x0003D214U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_NUDGE_REG                              (0x0003D218U)
#define CSL_CPSW2G_CPSW_NU_ALE_IDVER_REG                                       (0x0003E000U)
#define CSL_CPSW2G_CPSW_NU_ALE_STATUS_REG                                      (0x0003E004U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG                                     (0x0003E008U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL2_REG                                    (0x0003E00CU)
#define CSL_CPSW2G_CPSW_NU_ALE_PRESCALE_REG                                    (0x0003E010U)
#define CSL_CPSW2G_CPSW_NU_ALE_AGING_TIMER_REG                                 (0x0003E014U)
#define CSL_CPSW2G_CPSW_NU_ALE_TABLE_CONTROL_REG                               (0x0003E020U)
#define CSL_CPSW2G_CPSW_NU_ALE_TABLE_WORD2_REG                                 (0x0003E034U)
#define CSL_CPSW2G_CPSW_NU_ALE_TABLE_WORD1_REG                                 (0x0003E038U)
#define CSL_CPSW2G_CPSW_NU_ALE_TABLE_WORD0_REG                                 (0x0003E03CU)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG(PORT_CONTROL_REG)              (0x0003E040U+((PORT_CONTROL_REG)*0x4U))
#define CSL_CPSW2G_CPSW_NU_ALE_UNKNOWN_VLAN_REG                                (0x0003E090U)
#define CSL_CPSW2G_CPSW_NU_ALE_UNKNOWN_MCAST_FLOOD_REG                         (0x0003E094U)
#define CSL_CPSW2G_CPSW_NU_ALE_UNKNOWN_REG_MCAST_FLOOD_REG                     (0x0003E098U)
#define CSL_CPSW2G_CPSW_NU_ALE_FORCE_UNTAGGED_EGRESS_REG                       (0x0003E09CU)
#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX0_REG                              (0x0003E0C0U)
#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX1_REG                              (0x0003E0C4U)
#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX2_REG                              (0x0003E0C8U)
#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX3_REG                              (0x0003E0CCU)
#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX4_REG                              (0x0003E0D0U)
#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX5_REG                              (0x0003E0D4U)
#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX6_REG                              (0x0003E0D8U)
#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX7_REG                              (0x0003E0DCU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_PORT_OUI_REG                            (0x0003E100U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_DA_SA_REG                               (0x0003E104U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_VLAN_REG                                (0x0003E108U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_ETHERTYPE_IPSA_REG                      (0x0003E10CU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_IPDA_REG                                (0x0003E110U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_PIR_REG                                 (0x0003E118U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_CIR_REG                                 (0x0003E11CU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_TBL_CTL_REG                             (0x0003E120U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_CTL_REG                                 (0x0003E124U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_TEST_CTL_REG                            (0x0003E128U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_HIT_STATUS_REG                          (0x0003E12CU)
#define CSL_CPSW2G_CPSW_NU_ALE_THREAD_DEF_REG                                  (0x0003E134U)
#define CSL_CPSW2G_CPSW_NU_ALE_THREAD_CTL_REG                                  (0x0003E138U)
#define CSL_CPSW2G_CPSW_NU_ALE_THREAD_VAL_REG                                  (0x0003E13CU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* SGMII_IDVER_REG */

#define CSL_CPSW2G_CPSGMII_SGMII_IDVER_REG_MINOR_VER_MASK                      (0x000000FFU)
#define CSL_CPSW2G_CPSGMII_SGMII_IDVER_REG_MINOR_VER_SHIFT                     (0x00000000U)
#define CSL_CPSW2G_CPSGMII_SGMII_IDVER_REG_MINOR_VER_MAX                       (0x000000FFU)

#define CSL_CPSW2G_CPSGMII_SGMII_IDVER_REG_MAJOR_VER_MASK                      (0x00000700U)
#define CSL_CPSW2G_CPSGMII_SGMII_IDVER_REG_MAJOR_VER_SHIFT                     (0x00000008U)
#define CSL_CPSW2G_CPSGMII_SGMII_IDVER_REG_MAJOR_VER_MAX                       (0x00000007U)

#define CSL_CPSW2G_CPSGMII_SGMII_IDVER_REG_RTL_VER_MASK                        (0x0000F800U)
#define CSL_CPSW2G_CPSGMII_SGMII_IDVER_REG_RTL_VER_SHIFT                       (0x0000000BU)
#define CSL_CPSW2G_CPSGMII_SGMII_IDVER_REG_RTL_VER_MAX                         (0x0000001FU)

#define CSL_CPSW2G_CPSGMII_SGMII_IDVER_REG_TX_IDENT_MASK                       (0xFFFF0000U)
#define CSL_CPSW2G_CPSGMII_SGMII_IDVER_REG_TX_IDENT_SHIFT                      (0x00000010U)
#define CSL_CPSW2G_CPSGMII_SGMII_IDVER_REG_TX_IDENT_MAX                        (0x0000FFFFU)

/* SOFT_RESET_REG */

#define CSL_CPSW2G_CPSGMII_SOFT_RESET_REG_SOFT_RESET_MASK                      (0x00000001U)
#define CSL_CPSW2G_CPSGMII_SOFT_RESET_REG_SOFT_RESET_SHIFT                     (0x00000000U)
#define CSL_CPSW2G_CPSGMII_SOFT_RESET_REG_SOFT_RESET_MAX                       (0x00000001U)

#define CSL_CPSW2G_CPSGMII_SOFT_RESET_REG_RT_SOFT_RESET_MASK                   (0x00000002U)
#define CSL_CPSW2G_CPSGMII_SOFT_RESET_REG_RT_SOFT_RESET_SHIFT                  (0x00000001U)
#define CSL_CPSW2G_CPSGMII_SOFT_RESET_REG_RT_SOFT_RESET_MAX                    (0x00000001U)

/* CONTROL_REG */

#define CSL_CPSW2G_CPSGMII_CONTROL_REG_MR_AN_ENABLE_MASK                       (0x00000001U)
#define CSL_CPSW2G_CPSGMII_CONTROL_REG_MR_AN_ENABLE_SHIFT                      (0x00000000U)
#define CSL_CPSW2G_CPSGMII_CONTROL_REG_MR_AN_ENABLE_MAX                        (0x00000001U)

#define CSL_CPSW2G_CPSGMII_CONTROL_REG_MR_AN_RESTART_MASK                      (0x00000002U)
#define CSL_CPSW2G_CPSGMII_CONTROL_REG_MR_AN_RESTART_SHIFT                     (0x00000001U)
#define CSL_CPSW2G_CPSGMII_CONTROL_REG_MR_AN_RESTART_MAX                       (0x00000001U)

#define CSL_CPSW2G_CPSGMII_CONTROL_REG_FAST_LINK_TIMER_MASK                    (0x00000004U)
#define CSL_CPSW2G_CPSGMII_CONTROL_REG_FAST_LINK_TIMER_SHIFT                   (0x00000002U)
#define CSL_CPSW2G_CPSGMII_CONTROL_REG_FAST_LINK_TIMER_MAX                     (0x00000001U)

#define CSL_CPSW2G_CPSGMII_CONTROL_REG_MR_NP_LOADED_MASK                       (0x00000008U)
#define CSL_CPSW2G_CPSGMII_CONTROL_REG_MR_NP_LOADED_SHIFT                      (0x00000003U)
#define CSL_CPSW2G_CPSGMII_CONTROL_REG_MR_NP_LOADED_MAX                        (0x00000001U)

#define CSL_CPSW2G_CPSGMII_CONTROL_REG_LOOPBACK_MASK                           (0x00000010U)
#define CSL_CPSW2G_CPSGMII_CONTROL_REG_LOOPBACK_SHIFT                          (0x00000004U)
#define CSL_CPSW2G_CPSGMII_CONTROL_REG_LOOPBACK_MAX                            (0x00000001U)

#define CSL_CPSW2G_CPSGMII_CONTROL_REG_MASTER_MASK                             (0x00000020U)
#define CSL_CPSW2G_CPSGMII_CONTROL_REG_MASTER_SHIFT                            (0x00000005U)
#define CSL_CPSW2G_CPSGMII_CONTROL_REG_MASTER_MAX                              (0x00000001U)

#define CSL_CPSW2G_CPSGMII_CONTROL_REG_TEST_PATTERN_EN_MASK                    (0x00000040U)
#define CSL_CPSW2G_CPSGMII_CONTROL_REG_TEST_PATTERN_EN_SHIFT                   (0x00000006U)
#define CSL_CPSW2G_CPSGMII_CONTROL_REG_TEST_PATTERN_EN_MAX                     (0x00000001U)

/* STATUS_REG */

#define CSL_CPSW2G_CPSGMII_STATUS_REG_LINK_MASK                                (0x00000001U)
#define CSL_CPSW2G_CPSGMII_STATUS_REG_LINK_SHIFT                               (0x00000000U)
#define CSL_CPSW2G_CPSGMII_STATUS_REG_LINK_MAX                                 (0x00000001U)

#define CSL_CPSW2G_CPSGMII_STATUS_REG_AN_ERROR_MASK                            (0x00000002U)
#define CSL_CPSW2G_CPSGMII_STATUS_REG_AN_ERROR_SHIFT                           (0x00000001U)
#define CSL_CPSW2G_CPSGMII_STATUS_REG_AN_ERROR_MAX                             (0x00000001U)

#define CSL_CPSW2G_CPSGMII_STATUS_REG_MR_AN_COMPLETE_MASK                      (0x00000004U)
#define CSL_CPSW2G_CPSGMII_STATUS_REG_MR_AN_COMPLETE_SHIFT                     (0x00000002U)
#define CSL_CPSW2G_CPSGMII_STATUS_REG_MR_AN_COMPLETE_MAX                       (0x00000001U)

#define CSL_CPSW2G_CPSGMII_STATUS_REG_MR_PAGE_RX_MASK                          (0x00000008U)
#define CSL_CPSW2G_CPSGMII_STATUS_REG_MR_PAGE_RX_SHIFT                         (0x00000003U)
#define CSL_CPSW2G_CPSGMII_STATUS_REG_MR_PAGE_RX_MAX                           (0x00000001U)

#define CSL_CPSW2G_CPSGMII_STATUS_REG_LOCK_MASK                                (0x00000010U)
#define CSL_CPSW2G_CPSGMII_STATUS_REG_LOCK_SHIFT                               (0x00000004U)
#define CSL_CPSW2G_CPSGMII_STATUS_REG_LOCK_MAX                                 (0x00000001U)

#define CSL_CPSW2G_CPSGMII_STATUS_REG_FIB_SIG_DETECT_MASK                      (0x00000020U)
#define CSL_CPSW2G_CPSGMII_STATUS_REG_FIB_SIG_DETECT_SHIFT                     (0x00000005U)
#define CSL_CPSW2G_CPSGMII_STATUS_REG_FIB_SIG_DETECT_MAX                       (0x00000001U)

/* MR_ADV_ABILITY_REG */

#define CSL_CPSW2G_CPSGMII_MR_ADV_ABILITY_REG_MR_ADV_ABILITY_MASK              (0x0000FFFFU)
#define CSL_CPSW2G_CPSGMII_MR_ADV_ABILITY_REG_MR_ADV_ABILITY_SHIFT             (0x00000000U)
#define CSL_CPSW2G_CPSGMII_MR_ADV_ABILITY_REG_MR_ADV_ABILITY_MAX               (0x0000FFFFU)

/* MR_NP_TX_REG */

#define CSL_CPSW2G_CPSGMII_MR_NP_TX_REG_MR_NP_TX_MASK                          (0x0000FFFFU)
#define CSL_CPSW2G_CPSGMII_MR_NP_TX_REG_MR_NP_TX_SHIFT                         (0x00000000U)
#define CSL_CPSW2G_CPSGMII_MR_NP_TX_REG_MR_NP_TX_MAX                           (0x0000FFFFU)

/* MR_LP_ADV_ABILITY_REG */

#define CSL_CPSW2G_CPSGMII_MR_LP_ADV_ABILITY_REG_MR_LP_ADV_ABILITY_MASK        (0x0000FFFFU)
#define CSL_CPSW2G_CPSGMII_MR_LP_ADV_ABILITY_REG_MR_LP_ADV_ABILITY_SHIFT       (0x00000000U)
#define CSL_CPSW2G_CPSGMII_MR_LP_ADV_ABILITY_REG_MR_LP_ADV_ABILITY_MAX         (0x0000FFFFU)

/* MR_LP_NP_RX_REG */

#define CSL_CPSW2G_CPSGMII_MR_LP_NP_RX_REG_MR_LP_NP_RX_MASK                    (0x0000FFFFU)
#define CSL_CPSW2G_CPSGMII_MR_LP_NP_RX_REG_MR_LP_NP_RX_SHIFT                   (0x00000000U)
#define CSL_CPSW2G_CPSGMII_MR_LP_NP_RX_REG_MR_LP_NP_RX_MAX                     (0x0000FFFFU)

/* DIAG_CLEAR_REG */

#define CSL_CPSW2G_CPSGMII_DIAG_CLEAR_REG_DIAG_CLEAR_MASK                      (0x00000001U)
#define CSL_CPSW2G_CPSGMII_DIAG_CLEAR_REG_DIAG_CLEAR_SHIFT                     (0x00000000U)
#define CSL_CPSW2G_CPSGMII_DIAG_CLEAR_REG_DIAG_CLEAR_MAX                       (0x00000001U)

/* DIAG_CONTROL_REG */

#define CSL_CPSW2G_CPSGMII_DIAG_CONTROL_REG_DIAG_EDGE_SEL_MASK                 (0x00000003U)
#define CSL_CPSW2G_CPSGMII_DIAG_CONTROL_REG_DIAG_EDGE_SEL_SHIFT                (0x00000000U)
#define CSL_CPSW2G_CPSGMII_DIAG_CONTROL_REG_DIAG_EDGE_SEL_MAX                  (0x00000003U)

#define CSL_CPSW2G_CPSGMII_DIAG_CONTROL_REG_DIAG_SM_SEL_MASK                   (0x00000070U)
#define CSL_CPSW2G_CPSGMII_DIAG_CONTROL_REG_DIAG_SM_SEL_SHIFT                  (0x00000004U)
#define CSL_CPSW2G_CPSGMII_DIAG_CONTROL_REG_DIAG_SM_SEL_MAX                    (0x00000007U)

/* DIAG_STATUS_REG */

#define CSL_CPSW2G_CPSGMII_DIAG_STATUS_REG_DIAG_STATUS_MASK                    (0x0000FFFFU)
#define CSL_CPSW2G_CPSGMII_DIAG_STATUS_REG_DIAG_STATUS_SHIFT                   (0x00000000U)
#define CSL_CPSW2G_CPSGMII_DIAG_STATUS_REG_DIAG_STATUS_MAX                     (0x0000FFFFU)

/* USER_ACCESS_REG */

#define CSL_CPSW2G_MDIO_USER_GROUP_USER_ACCESS_REG_DATA_MASK                   (0x0000FFFFU)
#define CSL_CPSW2G_MDIO_USER_GROUP_USER_ACCESS_REG_DATA_SHIFT                  (0x00000000U)
#define CSL_CPSW2G_MDIO_USER_GROUP_USER_ACCESS_REG_DATA_MAX                    (0x0000FFFFU)

#define CSL_CPSW2G_MDIO_USER_GROUP_USER_ACCESS_REG_PHYADR_MASK                 (0x001F0000U)
#define CSL_CPSW2G_MDIO_USER_GROUP_USER_ACCESS_REG_PHYADR_SHIFT                (0x00000010U)
#define CSL_CPSW2G_MDIO_USER_GROUP_USER_ACCESS_REG_PHYADR_MAX                  (0x0000001FU)

#define CSL_CPSW2G_MDIO_USER_GROUP_USER_ACCESS_REG_REGADR_MASK                 (0x03E00000U)
#define CSL_CPSW2G_MDIO_USER_GROUP_USER_ACCESS_REG_REGADR_SHIFT                (0x00000015U)
#define CSL_CPSW2G_MDIO_USER_GROUP_USER_ACCESS_REG_REGADR_MAX                  (0x0000001FU)

#define CSL_CPSW2G_MDIO_USER_GROUP_USER_ACCESS_REG_ACK_MASK                    (0x20000000U)
#define CSL_CPSW2G_MDIO_USER_GROUP_USER_ACCESS_REG_ACK_SHIFT                   (0x0000001DU)
#define CSL_CPSW2G_MDIO_USER_GROUP_USER_ACCESS_REG_ACK_MAX                     (0x00000001U)

#define CSL_CPSW2G_MDIO_USER_GROUP_USER_ACCESS_REG_WRITE_MASK                  (0x40000000U)
#define CSL_CPSW2G_MDIO_USER_GROUP_USER_ACCESS_REG_WRITE_SHIFT                 (0x0000001EU)
#define CSL_CPSW2G_MDIO_USER_GROUP_USER_ACCESS_REG_WRITE_MAX                   (0x00000001U)

#define CSL_CPSW2G_MDIO_USER_GROUP_USER_ACCESS_REG_GO_MASK                     (0x80000000U)
#define CSL_CPSW2G_MDIO_USER_GROUP_USER_ACCESS_REG_GO_SHIFT                    (0x0000001FU)
#define CSL_CPSW2G_MDIO_USER_GROUP_USER_ACCESS_REG_GO_MAX                      (0x00000001U)

/* USER_PHY_SEL_REG */

#define CSL_CPSW2G_MDIO_USER_GROUP_USER_PHY_SEL_REG_PHYADR_MON_MASK            (0x0000001FU)
#define CSL_CPSW2G_MDIO_USER_GROUP_USER_PHY_SEL_REG_PHYADR_MON_SHIFT           (0x00000000U)
#define CSL_CPSW2G_MDIO_USER_GROUP_USER_PHY_SEL_REG_PHYADR_MON_MAX             (0x0000001FU)

#define CSL_CPSW2G_MDIO_USER_GROUP_USER_PHY_SEL_REG_LINKINT_ENABLE_MASK        (0x00000040U)
#define CSL_CPSW2G_MDIO_USER_GROUP_USER_PHY_SEL_REG_LINKINT_ENABLE_SHIFT       (0x00000006U)
#define CSL_CPSW2G_MDIO_USER_GROUP_USER_PHY_SEL_REG_LINKINT_ENABLE_MAX         (0x00000001U)

#define CSL_CPSW2G_MDIO_USER_GROUP_USER_PHY_SEL_REG_LINKSEL_MASK               (0x00000080U)
#define CSL_CPSW2G_MDIO_USER_GROUP_USER_PHY_SEL_REG_LINKSEL_SHIFT              (0x00000007U)
#define CSL_CPSW2G_MDIO_USER_GROUP_USER_PHY_SEL_REG_LINKSEL_MAX                (0x00000001U)

/* MDIO_VERSION_REG */

#define CSL_CPSW2G_MDIO_MDIO_VERSION_REG_SCHEME_MASK                           (0xC0000000U)
#define CSL_CPSW2G_MDIO_MDIO_VERSION_REG_SCHEME_SHIFT                          (0x0000001EU)
#define CSL_CPSW2G_MDIO_MDIO_VERSION_REG_SCHEME_MAX                            (0x00000003U)

#define CSL_CPSW2G_MDIO_MDIO_VERSION_REG_BU_MASK                               (0x30000000U)
#define CSL_CPSW2G_MDIO_MDIO_VERSION_REG_BU_SHIFT                              (0x0000001CU)
#define CSL_CPSW2G_MDIO_MDIO_VERSION_REG_BU_MAX                                (0x00000003U)

#define CSL_CPSW2G_MDIO_MDIO_VERSION_REG_MODULE_ID_MASK                        (0x0FFF0000U)
#define CSL_CPSW2G_MDIO_MDIO_VERSION_REG_MODULE_ID_SHIFT                       (0x00000010U)
#define CSL_CPSW2G_MDIO_MDIO_VERSION_REG_MODULE_ID_MAX                         (0x00000FFFU)

#define CSL_CPSW2G_MDIO_MDIO_VERSION_REG_REVRTL_MASK                           (0x0000F800U)
#define CSL_CPSW2G_MDIO_MDIO_VERSION_REG_REVRTL_SHIFT                          (0x0000000BU)
#define CSL_CPSW2G_MDIO_MDIO_VERSION_REG_REVRTL_MAX                            (0x0000001FU)

#define CSL_CPSW2G_MDIO_MDIO_VERSION_REG_REVMAJ_MASK                           (0x00000700U)
#define CSL_CPSW2G_MDIO_MDIO_VERSION_REG_REVMAJ_SHIFT                          (0x00000008U)
#define CSL_CPSW2G_MDIO_MDIO_VERSION_REG_REVMAJ_MAX                            (0x00000007U)

#define CSL_CPSW2G_MDIO_MDIO_VERSION_REG_CUSTOM_MASK                           (0x000000C0U)
#define CSL_CPSW2G_MDIO_MDIO_VERSION_REG_CUSTOM_SHIFT                          (0x00000006U)
#define CSL_CPSW2G_MDIO_MDIO_VERSION_REG_CUSTOM_MAX                            (0x00000003U)

#define CSL_CPSW2G_MDIO_MDIO_VERSION_REG_REVMIN_MASK                           (0x0000003FU)
#define CSL_CPSW2G_MDIO_MDIO_VERSION_REG_REVMIN_SHIFT                          (0x00000000U)
#define CSL_CPSW2G_MDIO_MDIO_VERSION_REG_REVMIN_MAX                            (0x0000003FU)

/* CONTROL_REG */

#define CSL_CPSW2G_MDIO_CONTROL_REG_CLKDIV_MASK                                (0x0000FFFFU)
#define CSL_CPSW2G_MDIO_CONTROL_REG_CLKDIV_SHIFT                               (0x00000000U)
#define CSL_CPSW2G_MDIO_CONTROL_REG_CLKDIV_MAX                                 (0x0000FFFFU)

#define CSL_CPSW2G_MDIO_CONTROL_REG_INT_TEST_ENABLE_MASK                       (0x00020000U)
#define CSL_CPSW2G_MDIO_CONTROL_REG_INT_TEST_ENABLE_SHIFT                      (0x00000011U)
#define CSL_CPSW2G_MDIO_CONTROL_REG_INT_TEST_ENABLE_MAX                        (0x00000001U)

#define CSL_CPSW2G_MDIO_CONTROL_REG_FAULT_DETECT_ENABLE_MASK                   (0x00040000U)
#define CSL_CPSW2G_MDIO_CONTROL_REG_FAULT_DETECT_ENABLE_SHIFT                  (0x00000012U)
#define CSL_CPSW2G_MDIO_CONTROL_REG_FAULT_DETECT_ENABLE_MAX                    (0x00000001U)

#define CSL_CPSW2G_MDIO_CONTROL_REG_FAULT_MASK                                 (0x00080000U)
#define CSL_CPSW2G_MDIO_CONTROL_REG_FAULT_SHIFT                                (0x00000013U)
#define CSL_CPSW2G_MDIO_CONTROL_REG_FAULT_MAX                                  (0x00000001U)

#define CSL_CPSW2G_MDIO_CONTROL_REG_PREAMBLE_MASK                              (0x00100000U)
#define CSL_CPSW2G_MDIO_CONTROL_REG_PREAMBLE_SHIFT                             (0x00000014U)
#define CSL_CPSW2G_MDIO_CONTROL_REG_PREAMBLE_MAX                               (0x00000001U)

#define CSL_CPSW2G_MDIO_CONTROL_REG_HIGHEST_USER_CHANNEL_MASK                  (0x1F000000U)
#define CSL_CPSW2G_MDIO_CONTROL_REG_HIGHEST_USER_CHANNEL_SHIFT                 (0x00000018U)
#define CSL_CPSW2G_MDIO_CONTROL_REG_HIGHEST_USER_CHANNEL_MAX                   (0x0000001FU)

#define CSL_CPSW2G_MDIO_CONTROL_REG_ENABLE_MASK                                (0x40000000U)
#define CSL_CPSW2G_MDIO_CONTROL_REG_ENABLE_SHIFT                               (0x0000001EU)
#define CSL_CPSW2G_MDIO_CONTROL_REG_ENABLE_MAX                                 (0x00000001U)

#define CSL_CPSW2G_MDIO_CONTROL_REG_IDLE_MASK                                  (0x80000000U)
#define CSL_CPSW2G_MDIO_CONTROL_REG_IDLE_SHIFT                                 (0x0000001FU)
#define CSL_CPSW2G_MDIO_CONTROL_REG_IDLE_MAX                                   (0x00000001U)

/* ALIVE_REG */

#define CSL_CPSW2G_MDIO_ALIVE_REG_ALIVE_MASK                                   (0xFFFFFFFFU)
#define CSL_CPSW2G_MDIO_ALIVE_REG_ALIVE_SHIFT                                  (0x00000000U)
#define CSL_CPSW2G_MDIO_ALIVE_REG_ALIVE_MAX                                    (0xFFFFFFFFU)

/* LINK_REG */

#define CSL_CPSW2G_MDIO_LINK_REG_LINK_MASK                                     (0xFFFFFFFFU)
#define CSL_CPSW2G_MDIO_LINK_REG_LINK_SHIFT                                    (0x00000000U)
#define CSL_CPSW2G_MDIO_LINK_REG_LINK_MAX                                      (0xFFFFFFFFU)

/* LINK_INT_RAW_REG */

#define CSL_CPSW2G_MDIO_LINK_INT_RAW_REG_LINKINTRAW_MASK                       (0x00000003U)
#define CSL_CPSW2G_MDIO_LINK_INT_RAW_REG_LINKINTRAW_SHIFT                      (0x00000000U)
#define CSL_CPSW2G_MDIO_LINK_INT_RAW_REG_LINKINTRAW_MAX                        (0x00000003U)

/* LINK_INT_MASKED_REG */

#define CSL_CPSW2G_MDIO_LINK_INT_MASKED_REG_LINKINTMASKED_MASK                 (0x00000003U)
#define CSL_CPSW2G_MDIO_LINK_INT_MASKED_REG_LINKINTMASKED_SHIFT                (0x00000000U)
#define CSL_CPSW2G_MDIO_LINK_INT_MASKED_REG_LINKINTMASKED_MAX                  (0x00000003U)

/* LINK_INT_MASK_SET_REG */

#define CSL_CPSW2G_MDIO_LINK_INT_MASK_SET_REG_LINKINTMASKSET_MASK              (0x00000001U)
#define CSL_CPSW2G_MDIO_LINK_INT_MASK_SET_REG_LINKINTMASKSET_SHIFT             (0x00000000U)
#define CSL_CPSW2G_MDIO_LINK_INT_MASK_SET_REG_LINKINTMASKSET_MAX               (0x00000001U)

/* LINK_INT_MASK_CLEAR_REG */

#define CSL_CPSW2G_MDIO_LINK_INT_MASK_CLEAR_REG_LINKINTMASKCLR_MASK            (0x00000001U)
#define CSL_CPSW2G_MDIO_LINK_INT_MASK_CLEAR_REG_LINKINTMASKCLR_SHIFT           (0x00000000U)
#define CSL_CPSW2G_MDIO_LINK_INT_MASK_CLEAR_REG_LINKINTMASKCLR_MAX             (0x00000001U)

/* USER_INT_RAW_REG */

#define CSL_CPSW2G_MDIO_USER_INT_RAW_REG_USERINTRAW_MASK                       (0x00000003U)
#define CSL_CPSW2G_MDIO_USER_INT_RAW_REG_USERINTRAW_SHIFT                      (0x00000000U)
#define CSL_CPSW2G_MDIO_USER_INT_RAW_REG_USERINTRAW_MAX                        (0x00000003U)

/* USER_INT_MASKED_REG */

#define CSL_CPSW2G_MDIO_USER_INT_MASKED_REG_USERINTMASKED_MASK                 (0x00000003U)
#define CSL_CPSW2G_MDIO_USER_INT_MASKED_REG_USERINTMASKED_SHIFT                (0x00000000U)
#define CSL_CPSW2G_MDIO_USER_INT_MASKED_REG_USERINTMASKED_MAX                  (0x00000003U)

/* USER_INT_MASK_SET_REG */

#define CSL_CPSW2G_MDIO_USER_INT_MASK_SET_REG_USERINTMASKSET_MASK              (0x00000003U)
#define CSL_CPSW2G_MDIO_USER_INT_MASK_SET_REG_USERINTMASKSET_SHIFT             (0x00000000U)
#define CSL_CPSW2G_MDIO_USER_INT_MASK_SET_REG_USERINTMASKSET_MAX               (0x00000003U)

/* USER_INT_MASK_CLEAR_REG */

#define CSL_CPSW2G_MDIO_USER_INT_MASK_CLEAR_REG_USERINTMASKCLR_MASK            (0x00000003U)
#define CSL_CPSW2G_MDIO_USER_INT_MASK_CLEAR_REG_USERINTMASKCLR_SHIFT           (0x00000000U)
#define CSL_CPSW2G_MDIO_USER_INT_MASK_CLEAR_REG_USERINTMASKCLR_MAX             (0x00000003U)

/* MANUAL_IF_REG */

#define CSL_CPSW2G_MDIO_MANUAL_IF_REG_MDIO_PIN_MASK                            (0x00000001U)
#define CSL_CPSW2G_MDIO_MANUAL_IF_REG_MDIO_PIN_SHIFT                           (0x00000000U)
#define CSL_CPSW2G_MDIO_MANUAL_IF_REG_MDIO_PIN_MAX                             (0x00000001U)

#define CSL_CPSW2G_MDIO_MANUAL_IF_REG_MDIO_OE_MASK                             (0x00000002U)
#define CSL_CPSW2G_MDIO_MANUAL_IF_REG_MDIO_OE_SHIFT                            (0x00000001U)
#define CSL_CPSW2G_MDIO_MANUAL_IF_REG_MDIO_OE_MAX                              (0x00000001U)

#define CSL_CPSW2G_MDIO_MANUAL_IF_REG_MDIO_MDCLK_O_MASK                        (0x00000004U)
#define CSL_CPSW2G_MDIO_MANUAL_IF_REG_MDIO_MDCLK_O_SHIFT                       (0x00000002U)
#define CSL_CPSW2G_MDIO_MANUAL_IF_REG_MDIO_MDCLK_O_MAX                         (0x00000001U)

/* POLL_REG */

#define CSL_CPSW2G_MDIO_POLL_REG_IPG_MASK                                      (0x000000FFU)
#define CSL_CPSW2G_MDIO_POLL_REG_IPG_SHIFT                                     (0x00000000U)
#define CSL_CPSW2G_MDIO_POLL_REG_IPG_MAX                                       (0x000000FFU)

#define CSL_CPSW2G_MDIO_POLL_REG_STATECHANGEMODE_MASK                          (0x40000000U)
#define CSL_CPSW2G_MDIO_POLL_REG_STATECHANGEMODE_SHIFT                         (0x0000001EU)
#define CSL_CPSW2G_MDIO_POLL_REG_STATECHANGEMODE_MAX                           (0x00000001U)

#define CSL_CPSW2G_MDIO_POLL_REG_MANUALMODE_MASK                               (0x80000000U)
#define CSL_CPSW2G_MDIO_POLL_REG_MANUALMODE_SHIFT                              (0x0000001FU)
#define CSL_CPSW2G_MDIO_POLL_REG_MANUALMODE_MAX                                (0x00000001U)

/* POLL_EN_REG */

#define CSL_CPSW2G_MDIO_POLL_EN_REG_POLL_EN_MASK                               (0xFFFFFFFFU)
#define CSL_CPSW2G_MDIO_POLL_EN_REG_POLL_EN_SHIFT                              (0x00000000U)
#define CSL_CPSW2G_MDIO_POLL_EN_REG_POLL_EN_MAX                                (0xFFFFFFFFU)

/* CLAUS45_REG */

#define CSL_CPSW2G_MDIO_CLAUS45_REG_CLAUSE45_MASK                              (0xFFFFFFFFU)
#define CSL_CPSW2G_MDIO_CLAUS45_REG_CLAUSE45_SHIFT                             (0x00000000U)
#define CSL_CPSW2G_MDIO_CLAUS45_REG_CLAUSE45_MAX                               (0xFFFFFFFFU)

/* USER_ADDR0_REG */

#define CSL_CPSW2G_MDIO_USER_ADDR0_REG_USER_ADDR0_MASK                         (0x0000FFFFU)
#define CSL_CPSW2G_MDIO_USER_ADDR0_REG_USER_ADDR0_SHIFT                        (0x00000000U)
#define CSL_CPSW2G_MDIO_USER_ADDR0_REG_USER_ADDR0_MAX                          (0x0000FFFFU)

/* USER_ADDR1_REG */

#define CSL_CPSW2G_MDIO_USER_ADDR1_REG_USER_ADDR1_MASK                         (0x0000FFFFU)
#define CSL_CPSW2G_MDIO_USER_ADDR1_REG_USER_ADDR1_SHIFT                        (0x00000000U)
#define CSL_CPSW2G_MDIO_USER_ADDR1_REG_USER_ADDR1_MAX                          (0x0000FFFFU)

/* REVISION */

#define CSL_CPSW2G_INTD_REVISION_SCHEME_MASK                                   (0xC0000000U)
#define CSL_CPSW2G_INTD_REVISION_SCHEME_SHIFT                                  (0x0000001EU)
#define CSL_CPSW2G_INTD_REVISION_SCHEME_MAX                                    (0x00000003U)

#define CSL_CPSW2G_INTD_REVISION_BU_MASK                                       (0x30000000U)
#define CSL_CPSW2G_INTD_REVISION_BU_SHIFT                                      (0x0000001CU)
#define CSL_CPSW2G_INTD_REVISION_BU_MAX                                        (0x00000003U)

#define CSL_CPSW2G_INTD_REVISION_FUNCTION_MASK                                 (0x0FFF0000U)
#define CSL_CPSW2G_INTD_REVISION_FUNCTION_SHIFT                                (0x00000010U)
#define CSL_CPSW2G_INTD_REVISION_FUNCTION_MAX                                  (0x00000FFFU)

#define CSL_CPSW2G_INTD_REVISION_RTLVER_MASK                                   (0x0000F800U)
#define CSL_CPSW2G_INTD_REVISION_RTLVER_SHIFT                                  (0x0000000BU)
#define CSL_CPSW2G_INTD_REVISION_RTLVER_MAX                                    (0x0000001FU)

#define CSL_CPSW2G_INTD_REVISION_MAJREV_MASK                                   (0x00000700U)
#define CSL_CPSW2G_INTD_REVISION_MAJREV_SHIFT                                  (0x00000008U)
#define CSL_CPSW2G_INTD_REVISION_MAJREV_MAX                                    (0x00000007U)

#define CSL_CPSW2G_INTD_REVISION_CUSTOM_MASK                                   (0x000000C0U)
#define CSL_CPSW2G_INTD_REVISION_CUSTOM_SHIFT                                  (0x00000006U)
#define CSL_CPSW2G_INTD_REVISION_CUSTOM_MAX                                    (0x00000003U)

#define CSL_CPSW2G_INTD_REVISION_MINREV_MASK                                   (0x0000003FU)
#define CSL_CPSW2G_INTD_REVISION_MINREV_SHIFT                                  (0x00000000U)
#define CSL_CPSW2G_INTD_REVISION_MINREV_MAX                                    (0x0000003FU)

/* CONTROL */

/* EOI_REG */

#define CSL_CPSW2G_INTD_EOI_REG_EOI_VECTOR_MASK                                (0x000000FFU)
#define CSL_CPSW2G_INTD_EOI_REG_EOI_VECTOR_SHIFT                               (0x00000000U)
#define CSL_CPSW2G_INTD_EOI_REG_EOI_VECTOR_MAX                                 (0x000000FFU)

/* INTR_VECTOR_REG */

#define CSL_CPSW2G_INTD_INTR_VECTOR_REG_INTR_VECTOR_MASK                       (0xFFFFFFFFU)
#define CSL_CPSW2G_INTD_INTR_VECTOR_REG_INTR_VECTOR_SHIFT                      (0x00000000U)
#define CSL_CPSW2G_INTD_INTR_VECTOR_REG_INTR_VECTOR_MAX                        (0xFFFFFFFFU)

/* ENABLE_REG_EVNT_PULSE0_0 */

#define CSL_CPSW2G_INTD_ENABLE_REG_EVNT_PULSE0_0_ENABLE_EVNT_PULSE0_EN_EVNT_LEVEL0_MASK (0x00000001U)
#define CSL_CPSW2G_INTD_ENABLE_REG_EVNT_PULSE0_0_ENABLE_EVNT_PULSE0_EN_EVNT_LEVEL0_SHIFT (0x00000000U)
#define CSL_CPSW2G_INTD_ENABLE_REG_EVNT_PULSE0_0_ENABLE_EVNT_PULSE0_EN_EVNT_LEVEL0_MAX (0x00000001U)

#define CSL_CPSW2G_INTD_ENABLE_REG_EVNT_PULSE0_0_ENABLE_EVNT_PULSE0_EN_STAT_LEVEL0_MASK (0x00000002U)
#define CSL_CPSW2G_INTD_ENABLE_REG_EVNT_PULSE0_0_ENABLE_EVNT_PULSE0_EN_STAT_LEVEL0_SHIFT (0x00000001U)
#define CSL_CPSW2G_INTD_ENABLE_REG_EVNT_PULSE0_0_ENABLE_EVNT_PULSE0_EN_STAT_LEVEL0_MAX (0x00000001U)

#define CSL_CPSW2G_INTD_ENABLE_REG_EVNT_PULSE0_0_ENABLE_EVNT_PULSE0_EN_STAT_LEVEL1_MASK (0x00000004U)
#define CSL_CPSW2G_INTD_ENABLE_REG_EVNT_PULSE0_0_ENABLE_EVNT_PULSE0_EN_STAT_LEVEL1_SHIFT (0x00000002U)
#define CSL_CPSW2G_INTD_ENABLE_REG_EVNT_PULSE0_0_ENABLE_EVNT_PULSE0_EN_STAT_LEVEL1_MAX (0x00000001U)

/* ENABLE_REG_STAT_PULSE0_0 */

#define CSL_CPSW2G_INTD_ENABLE_REG_STAT_PULSE0_0_ENABLE_STAT_PULSE0_EN_EVNT_LEVEL0_MASK (0x00000001U)
#define CSL_CPSW2G_INTD_ENABLE_REG_STAT_PULSE0_0_ENABLE_STAT_PULSE0_EN_EVNT_LEVEL0_SHIFT (0x00000000U)
#define CSL_CPSW2G_INTD_ENABLE_REG_STAT_PULSE0_0_ENABLE_STAT_PULSE0_EN_EVNT_LEVEL0_MAX (0x00000001U)

#define CSL_CPSW2G_INTD_ENABLE_REG_STAT_PULSE0_0_ENABLE_STAT_PULSE0_EN_STAT_LEVEL0_MASK (0x00000002U)
#define CSL_CPSW2G_INTD_ENABLE_REG_STAT_PULSE0_0_ENABLE_STAT_PULSE0_EN_STAT_LEVEL0_SHIFT (0x00000001U)
#define CSL_CPSW2G_INTD_ENABLE_REG_STAT_PULSE0_0_ENABLE_STAT_PULSE0_EN_STAT_LEVEL0_MAX (0x00000001U)

#define CSL_CPSW2G_INTD_ENABLE_REG_STAT_PULSE0_0_ENABLE_STAT_PULSE0_EN_STAT_LEVEL1_MASK (0x00000004U)
#define CSL_CPSW2G_INTD_ENABLE_REG_STAT_PULSE0_0_ENABLE_STAT_PULSE0_EN_STAT_LEVEL1_SHIFT (0x00000002U)
#define CSL_CPSW2G_INTD_ENABLE_REG_STAT_PULSE0_0_ENABLE_STAT_PULSE0_EN_STAT_LEVEL1_MAX (0x00000001U)

/* ENABLE_REG_STAT_PULSE1_0 */

#define CSL_CPSW2G_INTD_ENABLE_REG_STAT_PULSE1_0_ENABLE_STAT_PULSE1_EN_EVNT_LEVEL0_MASK (0x00000001U)
#define CSL_CPSW2G_INTD_ENABLE_REG_STAT_PULSE1_0_ENABLE_STAT_PULSE1_EN_EVNT_LEVEL0_SHIFT (0x00000000U)
#define CSL_CPSW2G_INTD_ENABLE_REG_STAT_PULSE1_0_ENABLE_STAT_PULSE1_EN_EVNT_LEVEL0_MAX (0x00000001U)

#define CSL_CPSW2G_INTD_ENABLE_REG_STAT_PULSE1_0_ENABLE_STAT_PULSE1_EN_STAT_LEVEL0_MASK (0x00000002U)
#define CSL_CPSW2G_INTD_ENABLE_REG_STAT_PULSE1_0_ENABLE_STAT_PULSE1_EN_STAT_LEVEL0_SHIFT (0x00000001U)
#define CSL_CPSW2G_INTD_ENABLE_REG_STAT_PULSE1_0_ENABLE_STAT_PULSE1_EN_STAT_LEVEL0_MAX (0x00000001U)

#define CSL_CPSW2G_INTD_ENABLE_REG_STAT_PULSE1_0_ENABLE_STAT_PULSE1_EN_STAT_LEVEL1_MASK (0x00000004U)
#define CSL_CPSW2G_INTD_ENABLE_REG_STAT_PULSE1_0_ENABLE_STAT_PULSE1_EN_STAT_LEVEL1_SHIFT (0x00000002U)
#define CSL_CPSW2G_INTD_ENABLE_REG_STAT_PULSE1_0_ENABLE_STAT_PULSE1_EN_STAT_LEVEL1_MAX (0x00000001U)

/* ENABLE_CLR_REG_EVNT_PULSE0_0 */

#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_EVNT_PULSE0_0_ENABLE_EVNT_PULSE0_EN_EVNT_LEVEL0_CLR_MASK (0x00000001U)
#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_EVNT_PULSE0_0_ENABLE_EVNT_PULSE0_EN_EVNT_LEVEL0_CLR_SHIFT (0x00000000U)
#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_EVNT_PULSE0_0_ENABLE_EVNT_PULSE0_EN_EVNT_LEVEL0_CLR_MAX (0x00000001U)

#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_EVNT_PULSE0_0_ENABLE_EVNT_PULSE0_EN_STAT_LEVEL0_CLR_MASK (0x00000002U)
#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_EVNT_PULSE0_0_ENABLE_EVNT_PULSE0_EN_STAT_LEVEL0_CLR_SHIFT (0x00000001U)
#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_EVNT_PULSE0_0_ENABLE_EVNT_PULSE0_EN_STAT_LEVEL0_CLR_MAX (0x00000001U)

#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_EVNT_PULSE0_0_ENABLE_EVNT_PULSE0_EN_STAT_LEVEL1_CLR_MASK (0x00000004U)
#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_EVNT_PULSE0_0_ENABLE_EVNT_PULSE0_EN_STAT_LEVEL1_CLR_SHIFT (0x00000002U)
#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_EVNT_PULSE0_0_ENABLE_EVNT_PULSE0_EN_STAT_LEVEL1_CLR_MAX (0x00000001U)

/* ENABLE_CLR_REG_STAT_PULSE0_0 */

#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_STAT_PULSE0_0_ENABLE_STAT_PULSE0_EN_EVNT_LEVEL0_CLR_MASK (0x00000001U)
#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_STAT_PULSE0_0_ENABLE_STAT_PULSE0_EN_EVNT_LEVEL0_CLR_SHIFT (0x00000000U)
#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_STAT_PULSE0_0_ENABLE_STAT_PULSE0_EN_EVNT_LEVEL0_CLR_MAX (0x00000001U)

#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_STAT_PULSE0_0_ENABLE_STAT_PULSE0_EN_STAT_LEVEL0_CLR_MASK (0x00000002U)
#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_STAT_PULSE0_0_ENABLE_STAT_PULSE0_EN_STAT_LEVEL0_CLR_SHIFT (0x00000001U)
#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_STAT_PULSE0_0_ENABLE_STAT_PULSE0_EN_STAT_LEVEL0_CLR_MAX (0x00000001U)

#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_STAT_PULSE0_0_ENABLE_STAT_PULSE0_EN_STAT_LEVEL1_CLR_MASK (0x00000004U)
#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_STAT_PULSE0_0_ENABLE_STAT_PULSE0_EN_STAT_LEVEL1_CLR_SHIFT (0x00000002U)
#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_STAT_PULSE0_0_ENABLE_STAT_PULSE0_EN_STAT_LEVEL1_CLR_MAX (0x00000001U)

/* ENABLE_CLR_REG_STAT_PULSE1_0 */

#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_STAT_PULSE1_0_ENABLE_STAT_PULSE1_EN_EVNT_LEVEL0_CLR_MASK (0x00000001U)
#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_STAT_PULSE1_0_ENABLE_STAT_PULSE1_EN_EVNT_LEVEL0_CLR_SHIFT (0x00000000U)
#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_STAT_PULSE1_0_ENABLE_STAT_PULSE1_EN_EVNT_LEVEL0_CLR_MAX (0x00000001U)

#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_STAT_PULSE1_0_ENABLE_STAT_PULSE1_EN_STAT_LEVEL0_CLR_MASK (0x00000002U)
#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_STAT_PULSE1_0_ENABLE_STAT_PULSE1_EN_STAT_LEVEL0_CLR_SHIFT (0x00000001U)
#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_STAT_PULSE1_0_ENABLE_STAT_PULSE1_EN_STAT_LEVEL0_CLR_MAX (0x00000001U)

#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_STAT_PULSE1_0_ENABLE_STAT_PULSE1_EN_STAT_LEVEL1_CLR_MASK (0x00000004U)
#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_STAT_PULSE1_0_ENABLE_STAT_PULSE1_EN_STAT_LEVEL1_CLR_SHIFT (0x00000002U)
#define CSL_CPSW2G_INTD_ENABLE_CLR_REG_STAT_PULSE1_0_ENABLE_STAT_PULSE1_EN_STAT_LEVEL1_CLR_MAX (0x00000001U)

/* STATUS_REG_EVNT_PULSE0_0 */

#define CSL_CPSW2G_INTD_STATUS_REG_EVNT_PULSE0_0_STATUS_EVNT_PULSE0_EVNT_LEVEL0_MASK (0x00000001U)
#define CSL_CPSW2G_INTD_STATUS_REG_EVNT_PULSE0_0_STATUS_EVNT_PULSE0_EVNT_LEVEL0_SHIFT (0x00000000U)
#define CSL_CPSW2G_INTD_STATUS_REG_EVNT_PULSE0_0_STATUS_EVNT_PULSE0_EVNT_LEVEL0_MAX (0x00000001U)

#define CSL_CPSW2G_INTD_STATUS_REG_EVNT_PULSE0_0_STATUS_EVNT_PULSE0_STAT_LEVEL0_MASK (0x00000002U)
#define CSL_CPSW2G_INTD_STATUS_REG_EVNT_PULSE0_0_STATUS_EVNT_PULSE0_STAT_LEVEL0_SHIFT (0x00000001U)
#define CSL_CPSW2G_INTD_STATUS_REG_EVNT_PULSE0_0_STATUS_EVNT_PULSE0_STAT_LEVEL0_MAX (0x00000001U)

#define CSL_CPSW2G_INTD_STATUS_REG_EVNT_PULSE0_0_STATUS_EVNT_PULSE0_STAT_LEVEL1_MASK (0x00000004U)
#define CSL_CPSW2G_INTD_STATUS_REG_EVNT_PULSE0_0_STATUS_EVNT_PULSE0_STAT_LEVEL1_SHIFT (0x00000002U)
#define CSL_CPSW2G_INTD_STATUS_REG_EVNT_PULSE0_0_STATUS_EVNT_PULSE0_STAT_LEVEL1_MAX (0x00000001U)

/* STATUS_REG_STAT_PULSE0_0 */

#define CSL_CPSW2G_INTD_STATUS_REG_STAT_PULSE0_0_STATUS_STAT_PULSE0_EVNT_LEVEL0_MASK (0x00000001U)
#define CSL_CPSW2G_INTD_STATUS_REG_STAT_PULSE0_0_STATUS_STAT_PULSE0_EVNT_LEVEL0_SHIFT (0x00000000U)
#define CSL_CPSW2G_INTD_STATUS_REG_STAT_PULSE0_0_STATUS_STAT_PULSE0_EVNT_LEVEL0_MAX (0x00000001U)

#define CSL_CPSW2G_INTD_STATUS_REG_STAT_PULSE0_0_STATUS_STAT_PULSE0_STAT_LEVEL0_MASK (0x00000002U)
#define CSL_CPSW2G_INTD_STATUS_REG_STAT_PULSE0_0_STATUS_STAT_PULSE0_STAT_LEVEL0_SHIFT (0x00000001U)
#define CSL_CPSW2G_INTD_STATUS_REG_STAT_PULSE0_0_STATUS_STAT_PULSE0_STAT_LEVEL0_MAX (0x00000001U)

#define CSL_CPSW2G_INTD_STATUS_REG_STAT_PULSE0_0_STATUS_STAT_PULSE0_STAT_LEVEL1_MASK (0x00000004U)
#define CSL_CPSW2G_INTD_STATUS_REG_STAT_PULSE0_0_STATUS_STAT_PULSE0_STAT_LEVEL1_SHIFT (0x00000002U)
#define CSL_CPSW2G_INTD_STATUS_REG_STAT_PULSE0_0_STATUS_STAT_PULSE0_STAT_LEVEL1_MAX (0x00000001U)

/* STATUS_REG_STAT_PULSE1_0 */

#define CSL_CPSW2G_INTD_STATUS_REG_STAT_PULSE1_0_STATUS_STAT_PULSE1_EVNT_LEVEL0_MASK (0x00000001U)
#define CSL_CPSW2G_INTD_STATUS_REG_STAT_PULSE1_0_STATUS_STAT_PULSE1_EVNT_LEVEL0_SHIFT (0x00000000U)
#define CSL_CPSW2G_INTD_STATUS_REG_STAT_PULSE1_0_STATUS_STAT_PULSE1_EVNT_LEVEL0_MAX (0x00000001U)

#define CSL_CPSW2G_INTD_STATUS_REG_STAT_PULSE1_0_STATUS_STAT_PULSE1_STAT_LEVEL0_MASK (0x00000002U)
#define CSL_CPSW2G_INTD_STATUS_REG_STAT_PULSE1_0_STATUS_STAT_PULSE1_STAT_LEVEL0_SHIFT (0x00000001U)
#define CSL_CPSW2G_INTD_STATUS_REG_STAT_PULSE1_0_STATUS_STAT_PULSE1_STAT_LEVEL0_MAX (0x00000001U)

#define CSL_CPSW2G_INTD_STATUS_REG_STAT_PULSE1_0_STATUS_STAT_PULSE1_STAT_LEVEL1_MASK (0x00000004U)
#define CSL_CPSW2G_INTD_STATUS_REG_STAT_PULSE1_0_STATUS_STAT_PULSE1_STAT_LEVEL1_SHIFT (0x00000002U)
#define CSL_CPSW2G_INTD_STATUS_REG_STAT_PULSE1_0_STATUS_STAT_PULSE1_STAT_LEVEL1_MAX (0x00000001U)

/* STATUS_CLR_REG_EVNT_PULSE0_0 */

/* STATUS_CLR_REG_STAT_PULSE0_0 */

/* STATUS_CLR_REG_STAT_PULSE1_0 */

/* INTR_VECTOR_REG_EVNT_PULSE0 */

#define CSL_CPSW2G_INTD_INTR_VECTOR_REG_EVNT_PULSE0_INTR_VECTOR_EVNT_PULSE0_MASK (0xFFFFFFFFU)
#define CSL_CPSW2G_INTD_INTR_VECTOR_REG_EVNT_PULSE0_INTR_VECTOR_EVNT_PULSE0_SHIFT (0x00000000U)
#define CSL_CPSW2G_INTD_INTR_VECTOR_REG_EVNT_PULSE0_INTR_VECTOR_EVNT_PULSE0_MAX (0xFFFFFFFFU)

/* INTR_VECTOR_REG_STAT_PULSE0 */

#define CSL_CPSW2G_INTD_INTR_VECTOR_REG_STAT_PULSE0_INTR_VECTOR_STAT_PULSE0_MASK (0xFFFFFFFFU)
#define CSL_CPSW2G_INTD_INTR_VECTOR_REG_STAT_PULSE0_INTR_VECTOR_STAT_PULSE0_SHIFT (0x00000000U)
#define CSL_CPSW2G_INTD_INTR_VECTOR_REG_STAT_PULSE0_INTR_VECTOR_STAT_PULSE0_MAX (0xFFFFFFFFU)

/* INTR_VECTOR_REG_STAT_PULSE1 */

#define CSL_CPSW2G_INTD_INTR_VECTOR_REG_STAT_PULSE1_INTR_VECTOR_STAT_PULSE1_MASK (0xFFFFFFFFU)
#define CSL_CPSW2G_INTD_INTR_VECTOR_REG_STAT_PULSE1_INTR_VECTOR_STAT_PULSE1_SHIFT (0x00000000U)
#define CSL_CPSW2G_INTD_INTR_VECTOR_REG_STAT_PULSE1_INTR_VECTOR_STAT_PULSE1_MAX (0xFFFFFFFFU)

/* P0_CONTROL_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_CONTROL_REG_RX_CHECKSUM_EN_MASK     (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_CONTROL_REG_RX_CHECKSUM_EN_SHIFT    (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_CONTROL_REG_RX_CHECKSUM_EN_MAX      (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_CONTROL_REG_DSCP_IPV4_EN_MASK       (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_CONTROL_REG_DSCP_IPV4_EN_SHIFT      (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_CONTROL_REG_DSCP_IPV4_EN_MAX        (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_CONTROL_REG_DSCP_IPV6_EN_MASK       (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_CONTROL_REG_DSCP_IPV6_EN_SHIFT      (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_CONTROL_REG_DSCP_IPV6_EN_MAX        (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_CONTROL_REG_TX_ECC_ERR_EN_MASK      (0x00004000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_CONTROL_REG_TX_ECC_ERR_EN_SHIFT     (0x0000000EU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_CONTROL_REG_TX_ECC_ERR_EN_MAX       (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_CONTROL_REG_RX_ECC_ERR_EN_MASK      (0x00008000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_CONTROL_REG_RX_ECC_ERR_EN_SHIFT     (0x0000000FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_CONTROL_REG_RX_ECC_ERR_EN_MAX       (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_CONTROL_REG_RX_REMAP_VLAN_MASK      (0x00010000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_CONTROL_REG_RX_REMAP_VLAN_SHIFT     (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_CONTROL_REG_RX_REMAP_VLAN_MAX       (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_CONTROL_REG_RX_REMAP_DSCP_V4_MASK   (0x00020000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_CONTROL_REG_RX_REMAP_DSCP_V4_SHIFT  (0x00000011U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_CONTROL_REG_RX_REMAP_DSCP_V4_MAX    (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_CONTROL_REG_RX_REMAP_DSCP_V6_MASK   (0x00040000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_CONTROL_REG_RX_REMAP_DSCP_V6_SHIFT  (0x00000012U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_CONTROL_REG_RX_REMAP_DSCP_V6_MAX    (0x00000001U)

/* P0_FLOW_ID_OFFSET_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_FLOW_ID_OFFSET_REG_VALUE_MASK       (0x00003FFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_FLOW_ID_OFFSET_REG_VALUE_SHIFT      (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_FLOW_ID_OFFSET_REG_VALUE_MAX        (0x00003FFFU)

/* P0_BLK_CNT_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_BLK_CNT_REG_RX_BLK_CNT_MASK         (0x0000003FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_BLK_CNT_REG_RX_BLK_CNT_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_BLK_CNT_REG_RX_BLK_CNT_MAX          (0x0000003FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_BLK_CNT_REG_TX_BLK_CNT_MASK         (0x00001F00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_BLK_CNT_REG_TX_BLK_CNT_SHIFT        (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_BLK_CNT_REG_TX_BLK_CNT_MAX          (0x0000001FU)

/* P0_PORT_VLAN_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_PORT_VLAN_REG_PORT_VID_MASK         (0x00000FFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_PORT_VLAN_REG_PORT_VID_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_PORT_VLAN_REG_PORT_VID_MAX          (0x00000FFFU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_PORT_VLAN_REG_PORT_CFI_MASK         (0x00001000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_PORT_VLAN_REG_PORT_CFI_SHIFT        (0x0000000CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_PORT_VLAN_REG_PORT_CFI_MAX          (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_PORT_VLAN_REG_PORT_PRI_MASK         (0x0000E000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_PORT_VLAN_REG_PORT_PRI_SHIFT        (0x0000000DU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_PORT_VLAN_REG_PORT_PRI_MAX          (0x00000007U)

/* P0_TX_PRI_MAP_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_PRI_MAP_REG_PRI0_MASK            (0x00000007U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_PRI_MAP_REG_PRI0_SHIFT           (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_PRI_MAP_REG_PRI0_MAX             (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_PRI_MAP_REG_PRI1_MASK            (0x00000070U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_PRI_MAP_REG_PRI1_SHIFT           (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_PRI_MAP_REG_PRI1_MAX             (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_PRI_MAP_REG_PRI2_MASK            (0x00000700U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_PRI_MAP_REG_PRI2_SHIFT           (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_PRI_MAP_REG_PRI2_MAX             (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_PRI_MAP_REG_PRI3_MASK            (0x00007000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_PRI_MAP_REG_PRI3_SHIFT           (0x0000000CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_PRI_MAP_REG_PRI3_MAX             (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_PRI_MAP_REG_PRI4_MASK            (0x00070000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_PRI_MAP_REG_PRI4_SHIFT           (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_PRI_MAP_REG_PRI4_MAX             (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_PRI_MAP_REG_PRI5_MASK            (0x00700000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_PRI_MAP_REG_PRI5_SHIFT           (0x00000014U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_PRI_MAP_REG_PRI5_MAX             (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_PRI_MAP_REG_PRI6_MASK            (0x07000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_PRI_MAP_REG_PRI6_SHIFT           (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_PRI_MAP_REG_PRI6_MAX             (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_PRI_MAP_REG_PRI7_MASK            (0x70000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_PRI_MAP_REG_PRI7_SHIFT           (0x0000001CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_PRI_MAP_REG_PRI7_MAX             (0x00000007U)

/* P0_PRI_CTL_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_PRI_CTL_REG_RX_PTYPE_MASK           (0x00000100U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_PRI_CTL_REG_RX_PTYPE_SHIFT          (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_PRI_CTL_REG_RX_PTYPE_MAX            (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_PRI_CTL_REG_RX_FLOW_PRI_MASK        (0x00FF0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_PRI_CTL_REG_RX_FLOW_PRI_SHIFT       (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_PRI_CTL_REG_RX_FLOW_PRI_MAX         (0x000000FFU)

/* P0_RX_PRI_MAP_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PRI_MAP_REG_PRI0_MASK            (0x00000007U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PRI_MAP_REG_PRI0_SHIFT           (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PRI_MAP_REG_PRI0_MAX             (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PRI_MAP_REG_PRI1_MASK            (0x00000070U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PRI_MAP_REG_PRI1_SHIFT           (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PRI_MAP_REG_PRI1_MAX             (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PRI_MAP_REG_PRI2_MASK            (0x00000700U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PRI_MAP_REG_PRI2_SHIFT           (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PRI_MAP_REG_PRI2_MAX             (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PRI_MAP_REG_PRI3_MASK            (0x00007000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PRI_MAP_REG_PRI3_SHIFT           (0x0000000CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PRI_MAP_REG_PRI3_MAX             (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PRI_MAP_REG_PRI4_MASK            (0x00070000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PRI_MAP_REG_PRI4_SHIFT           (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PRI_MAP_REG_PRI4_MAX             (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PRI_MAP_REG_PRI5_MASK            (0x00700000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PRI_MAP_REG_PRI5_SHIFT           (0x00000014U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PRI_MAP_REG_PRI5_MAX             (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PRI_MAP_REG_PRI6_MASK            (0x07000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PRI_MAP_REG_PRI6_SHIFT           (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PRI_MAP_REG_PRI6_MAX             (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PRI_MAP_REG_PRI7_MASK            (0x70000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PRI_MAP_REG_PRI7_SHIFT           (0x0000001CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PRI_MAP_REG_PRI7_MAX             (0x00000007U)

/* P0_RX_MAXLEN_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_MAXLEN_REG_RX_MAXLEN_MASK        (0x00003FFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_MAXLEN_REG_RX_MAXLEN_SHIFT       (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_MAXLEN_REG_RX_MAXLEN_MAX         (0x00003FFFU)

/* P0_TX_BLKS_PRI_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_BLKS_PRI_REG_PRI0_MASK           (0x0000000FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_BLKS_PRI_REG_PRI0_SHIFT          (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_BLKS_PRI_REG_PRI0_MAX            (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_BLKS_PRI_REG_PRI1_MASK           (0x000000F0U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_BLKS_PRI_REG_PRI1_SHIFT          (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_BLKS_PRI_REG_PRI1_MAX            (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_BLKS_PRI_REG_PRI2_MASK           (0x00000F00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_BLKS_PRI_REG_PRI2_SHIFT          (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_BLKS_PRI_REG_PRI2_MAX            (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_BLKS_PRI_REG_PRI3_MASK           (0x0000F000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_BLKS_PRI_REG_PRI3_SHIFT          (0x0000000CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_BLKS_PRI_REG_PRI3_MAX            (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_BLKS_PRI_REG_PRI4_MASK           (0x000F0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_BLKS_PRI_REG_PRI4_SHIFT          (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_BLKS_PRI_REG_PRI4_MAX            (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_BLKS_PRI_REG_PRI5_MASK           (0x00F00000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_BLKS_PRI_REG_PRI5_SHIFT          (0x00000014U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_BLKS_PRI_REG_PRI5_MAX            (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_BLKS_PRI_REG_PRI6_MASK           (0x0F000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_BLKS_PRI_REG_PRI6_SHIFT          (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_BLKS_PRI_REG_PRI6_MAX            (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_BLKS_PRI_REG_PRI7_MASK           (0xF0000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_BLKS_PRI_REG_PRI7_SHIFT          (0x0000001CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_BLKS_PRI_REG_PRI7_MAX            (0x0000000FU)

/* P0_IDLE2LPI_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_IDLE2LPI_REG_COUNT_MASK             (0x00FFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_IDLE2LPI_REG_COUNT_SHIFT            (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_IDLE2LPI_REG_COUNT_MAX              (0x00FFFFFFU)

/* P0_LPI2WAKE_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_LPI2WAKE_REG_COUNT_MASK             (0x00FFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_LPI2WAKE_REG_COUNT_SHIFT            (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_LPI2WAKE_REG_COUNT_MAX              (0x00FFFFFFU)

/* P0_EEE_STATUS_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_EEE_STATUS_REG_WAIT_IDLE2LPI_MASK   (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_EEE_STATUS_REG_WAIT_IDLE2LPI_SHIFT  (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_EEE_STATUS_REG_WAIT_IDLE2LPI_MAX    (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_EEE_STATUS_REG_RX_LPI_MASK          (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_EEE_STATUS_REG_RX_LPI_SHIFT         (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_EEE_STATUS_REG_RX_LPI_MAX           (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_EEE_STATUS_REG_TX_LPI_MASK          (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_EEE_STATUS_REG_TX_LPI_SHIFT         (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_EEE_STATUS_REG_TX_LPI_MAX           (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_EEE_STATUS_REG_TX_WAKE_MASK         (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_EEE_STATUS_REG_TX_WAKE_SHIFT        (0x00000003U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_EEE_STATUS_REG_TX_WAKE_MAX          (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_EEE_STATUS_REG_TX_FIFO_HOLD_MASK    (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_EEE_STATUS_REG_TX_FIFO_HOLD_SHIFT   (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_EEE_STATUS_REG_TX_FIFO_HOLD_MAX     (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_EEE_STATUS_REG_RX_FIFO_EMPTY_MASK   (0x00000020U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_EEE_STATUS_REG_RX_FIFO_EMPTY_SHIFT  (0x00000005U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_EEE_STATUS_REG_RX_FIFO_EMPTY_MAX    (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_EEE_STATUS_REG_TX_FIFO_EMPTY_MASK   (0x00000040U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_EEE_STATUS_REG_TX_FIFO_EMPTY_SHIFT  (0x00000006U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_EEE_STATUS_REG_TX_FIFO_EMPTY_MAX    (0x00000001U)

/* P0_RX_PKTS_PRI_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PKTS_PRI_REG_PRI0_MASK           (0x0000000FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PKTS_PRI_REG_PRI0_SHIFT          (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PKTS_PRI_REG_PRI0_MAX            (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PKTS_PRI_REG_PRI1_MASK           (0x000000F0U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PKTS_PRI_REG_PRI1_SHIFT          (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PKTS_PRI_REG_PRI1_MAX            (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PKTS_PRI_REG_PRI2_MASK           (0x00000F00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PKTS_PRI_REG_PRI2_SHIFT          (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PKTS_PRI_REG_PRI2_MAX            (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PKTS_PRI_REG_PRI3_MASK           (0x0000F000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PKTS_PRI_REG_PRI3_SHIFT          (0x0000000CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PKTS_PRI_REG_PRI3_MAX            (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PKTS_PRI_REG_PRI4_MASK           (0x000F0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PKTS_PRI_REG_PRI4_SHIFT          (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PKTS_PRI_REG_PRI4_MAX            (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PKTS_PRI_REG_PRI5_MASK           (0x00F00000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PKTS_PRI_REG_PRI5_SHIFT          (0x00000014U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PKTS_PRI_REG_PRI5_MAX            (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PKTS_PRI_REG_PRI6_MASK           (0x0F000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PKTS_PRI_REG_PRI6_SHIFT          (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PKTS_PRI_REG_PRI6_MAX            (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PKTS_PRI_REG_PRI7_MASK           (0xF0000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PKTS_PRI_REG_PRI7_SHIFT          (0x0000001CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_PKTS_PRI_REG_PRI7_MAX            (0x0000000FU)

/* P0_RX_GAP_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_GAP_REG_RX_GAP_EN_MASK           (0x000000FFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_GAP_REG_RX_GAP_EN_SHIFT          (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_GAP_REG_RX_GAP_EN_MAX            (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_GAP_REG_RX_GAP_CNT_MASK          (0x03FF0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_GAP_REG_RX_GAP_CNT_SHIFT         (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_GAP_REG_RX_GAP_CNT_MAX           (0x000003FFU)

/* P0_FIFO_STATUS_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_FIFO_STATUS_REG_TX_PRI_ACTIVE_MASK  (0x000000FFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_FIFO_STATUS_REG_TX_PRI_ACTIVE_SHIFT (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_FIFO_STATUS_REG_TX_PRI_ACTIVE_MAX   (0x000000FFU)

/* P0_RX_DSCP_MAP_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_DSCP_MAP_REG_PRI0_MASK           (0x00000007U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_DSCP_MAP_REG_PRI0_SHIFT          (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_DSCP_MAP_REG_PRI0_MAX            (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_DSCP_MAP_REG_PRI1_MASK           (0x00000070U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_DSCP_MAP_REG_PRI1_SHIFT          (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_DSCP_MAP_REG_PRI1_MAX            (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_DSCP_MAP_REG_PRI2_MASK           (0x00000700U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_DSCP_MAP_REG_PRI2_SHIFT          (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_DSCP_MAP_REG_PRI2_MAX            (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_DSCP_MAP_REG_PRI3_MASK           (0x00007000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_DSCP_MAP_REG_PRI3_SHIFT          (0x0000000CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_DSCP_MAP_REG_PRI3_MAX            (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_DSCP_MAP_REG_PRI4_MASK           (0x00070000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_DSCP_MAP_REG_PRI4_SHIFT          (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_DSCP_MAP_REG_PRI4_MAX            (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_DSCP_MAP_REG_PRI5_MASK           (0x00700000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_DSCP_MAP_REG_PRI5_SHIFT          (0x00000014U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_DSCP_MAP_REG_PRI5_MAX            (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_DSCP_MAP_REG_PRI6_MASK           (0x07000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_DSCP_MAP_REG_PRI6_SHIFT          (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_DSCP_MAP_REG_PRI6_MAX            (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_DSCP_MAP_REG_PRI7_MASK           (0x70000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_DSCP_MAP_REG_PRI7_SHIFT          (0x0000001CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_RX_DSCP_MAP_REG_PRI7_MAX            (0x00000007U)

/* P0_PRI_CIR_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_PRI_CIR_REG_PRI_CIR_MASK            (0x0FFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_PRI_CIR_REG_PRI_CIR_SHIFT           (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_PRI_CIR_REG_PRI_CIR_MAX             (0x0FFFFFFFU)

/* P0_PRI_EIR_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_PRI_EIR_REG_PRI_EIR_MASK            (0x0FFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_PRI_EIR_REG_PRI_EIR_SHIFT           (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_PRI_EIR_REG_PRI_EIR_MAX             (0x0FFFFFFFU)

/* P0_TX_D_THRESH_SET_L_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_SET_L_REG_PRI0_MASK     (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_SET_L_REG_PRI0_SHIFT    (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_SET_L_REG_PRI0_MAX      (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_SET_L_REG_PRI1_MASK     (0x00001F00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_SET_L_REG_PRI1_SHIFT    (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_SET_L_REG_PRI1_MAX      (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_SET_L_REG_PRI2_MASK     (0x001F0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_SET_L_REG_PRI2_SHIFT    (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_SET_L_REG_PRI2_MAX      (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_SET_L_REG_PRI3_MASK     (0x1F000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_SET_L_REG_PRI3_SHIFT    (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_SET_L_REG_PRI3_MAX      (0x0000001FU)

/* P0_TX_D_THRESH_SET_H_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_SET_H_REG_PRI4_MASK     (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_SET_H_REG_PRI4_SHIFT    (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_SET_H_REG_PRI4_MAX      (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_SET_H_REG_PRI5_MASK     (0x00001F00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_SET_H_REG_PRI5_SHIFT    (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_SET_H_REG_PRI5_MAX      (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_SET_H_REG_PRI6_MASK     (0x001F0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_SET_H_REG_PRI6_SHIFT    (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_SET_H_REG_PRI6_MAX      (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_SET_H_REG_PRI7_MASK     (0x1F000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_SET_H_REG_PRI7_SHIFT    (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_SET_H_REG_PRI7_MAX      (0x0000001FU)

/* P0_TX_D_THRESH_CLR_L_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_CLR_L_REG_PRI0_MASK     (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_CLR_L_REG_PRI0_SHIFT    (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_CLR_L_REG_PRI0_MAX      (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_CLR_L_REG_PRI1_MASK     (0x00001F00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_CLR_L_REG_PRI1_SHIFT    (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_CLR_L_REG_PRI1_MAX      (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_CLR_L_REG_PRI2_MASK     (0x001F0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_CLR_L_REG_PRI2_SHIFT    (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_CLR_L_REG_PRI2_MAX      (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_CLR_L_REG_PRI3_MASK     (0x1F000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_CLR_L_REG_PRI3_SHIFT    (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_CLR_L_REG_PRI3_MAX      (0x0000001FU)

/* P0_TX_D_THRESH_CLR_H_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_CLR_H_REG_PRI4_MASK     (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_CLR_H_REG_PRI4_SHIFT    (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_CLR_H_REG_PRI4_MAX      (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_CLR_H_REG_PRI5_MASK     (0x00001F00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_CLR_H_REG_PRI5_SHIFT    (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_CLR_H_REG_PRI5_MAX      (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_CLR_H_REG_PRI6_MASK     (0x001F0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_CLR_H_REG_PRI6_SHIFT    (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_CLR_H_REG_PRI6_MAX      (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_CLR_H_REG_PRI7_MASK     (0x1F000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_CLR_H_REG_PRI7_SHIFT    (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_D_THRESH_CLR_H_REG_PRI7_MAX      (0x0000001FU)

/* P0_TX_G_BUF_THRESH_SET_L_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_SET_L_REG_PRI0_MASK (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_SET_L_REG_PRI0_SHIFT (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_SET_L_REG_PRI0_MAX  (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_SET_L_REG_PRI1_MASK (0x00001F00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_SET_L_REG_PRI1_SHIFT (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_SET_L_REG_PRI1_MAX  (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_SET_L_REG_PRI2_MASK (0x001F0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_SET_L_REG_PRI2_SHIFT (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_SET_L_REG_PRI2_MAX  (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_SET_L_REG_PRI3_MASK (0x1F000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_SET_L_REG_PRI3_SHIFT (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_SET_L_REG_PRI3_MAX  (0x0000001FU)

/* P0_TX_G_BUF_THRESH_SET_H_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_SET_H_REG_PRI4_MASK (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_SET_H_REG_PRI4_SHIFT (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_SET_H_REG_PRI4_MAX  (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_SET_H_REG_PRI5_MASK (0x00001F00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_SET_H_REG_PRI5_SHIFT (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_SET_H_REG_PRI5_MAX  (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_SET_H_REG_PRI6_MASK (0x001F0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_SET_H_REG_PRI6_SHIFT (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_SET_H_REG_PRI6_MAX  (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_SET_H_REG_PRI7_MASK (0x1F000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_SET_H_REG_PRI7_SHIFT (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_SET_H_REG_PRI7_MAX  (0x0000001FU)

/* P0_TX_G_BUF_THRESH_CLR_L_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI0_MASK (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI0_SHIFT (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI0_MAX  (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI1_MASK (0x00001F00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI1_SHIFT (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI1_MAX  (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI2_MASK (0x001F0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI2_SHIFT (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI2_MAX  (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI3_MASK (0x1F000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI3_SHIFT (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_CLR_L_REG_PRI3_MAX  (0x0000001FU)

/* P0_TX_G_BUF_THRESH_CLR_H_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI4_MASK (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI4_SHIFT (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI4_MAX  (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI5_MASK (0x00001F00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI5_SHIFT (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI5_MAX  (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI6_MASK (0x001F0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI6_SHIFT (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI6_MAX  (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI7_MASK (0x1F000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI7_SHIFT (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_TX_G_BUF_THRESH_CLR_H_REG_PRI7_MAX  (0x0000001FU)

/* P0_SRC_ID_A_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_SRC_ID_A_REG_PORT1_MASK             (0x000000FFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_SRC_ID_A_REG_PORT1_SHIFT            (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_SRC_ID_A_REG_PORT1_MAX              (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_SRC_ID_A_REG_PORT2_MASK             (0x0000FF00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_SRC_ID_A_REG_PORT2_SHIFT            (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_SRC_ID_A_REG_PORT2_MAX              (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_SRC_ID_A_REG_PORT3_MASK             (0x00FF0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_SRC_ID_A_REG_PORT3_SHIFT            (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_SRC_ID_A_REG_PORT3_MAX              (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_SRC_ID_A_REG_PORT4_MASK             (0xFF000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_SRC_ID_A_REG_PORT4_SHIFT            (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_SRC_ID_A_REG_PORT4_MAX              (0x000000FFU)

/* P0_SRC_ID_B_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_SRC_ID_B_REG_PORT5_MASK             (0x000000FFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_SRC_ID_B_REG_PORT5_SHIFT            (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_SRC_ID_B_REG_PORT5_MAX              (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_SRC_ID_B_REG_PORT6_MASK             (0x0000FF00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_SRC_ID_B_REG_PORT6_SHIFT            (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_SRC_ID_B_REG_PORT6_MAX              (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_SRC_ID_B_REG_PORT7_MASK             (0x00FF0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_SRC_ID_B_REG_PORT7_SHIFT            (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_SRC_ID_B_REG_PORT7_MAX              (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_SRC_ID_B_REG_PORT8_MASK             (0xFF000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_SRC_ID_B_REG_PORT8_SHIFT            (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_SRC_ID_B_REG_PORT8_MAX              (0x000000FFU)

/* P0_HOST_BLKS_PRI_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_HOST_BLKS_PRI_REG_PRI0_MASK         (0x0000000FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_HOST_BLKS_PRI_REG_PRI0_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_HOST_BLKS_PRI_REG_PRI0_MAX          (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_HOST_BLKS_PRI_REG_PRI1_MASK         (0x000000F0U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_HOST_BLKS_PRI_REG_PRI1_SHIFT        (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_HOST_BLKS_PRI_REG_PRI1_MAX          (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_HOST_BLKS_PRI_REG_PRI2_MASK         (0x00000F00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_HOST_BLKS_PRI_REG_PRI2_SHIFT        (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_HOST_BLKS_PRI_REG_PRI2_MAX          (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_HOST_BLKS_PRI_REG_PRI3_MASK         (0x0000F000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_HOST_BLKS_PRI_REG_PRI3_SHIFT        (0x0000000CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_HOST_BLKS_PRI_REG_PRI3_MAX          (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_HOST_BLKS_PRI_REG_PRI4_MASK         (0x000F0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_HOST_BLKS_PRI_REG_PRI4_SHIFT        (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_HOST_BLKS_PRI_REG_PRI4_MAX          (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_HOST_BLKS_PRI_REG_PRI5_MASK         (0x00F00000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_HOST_BLKS_PRI_REG_PRI5_SHIFT        (0x00000014U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_HOST_BLKS_PRI_REG_PRI5_MAX          (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_HOST_BLKS_PRI_REG_PRI6_MASK         (0x0F000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_HOST_BLKS_PRI_REG_PRI6_SHIFT        (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_HOST_BLKS_PRI_REG_PRI6_MAX          (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_HOST_BLKS_PRI_REG_PRI7_MASK         (0xF0000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_HOST_BLKS_PRI_REG_PRI7_SHIFT        (0x0000001CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_CPPI_P0_HOST_BLKS_PRI_REG_PRI7_MAX          (0x0000000FU)

/* PN_RESERVED_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RESERVED_REG_RESERVED_MASK           (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RESERVED_REG_RESERVED_SHIFT          (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RESERVED_REG_RESERVED_MAX            (0xFFFFFFFFU)

/* PN_CONTROL_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_CONTROL_REG_DSCP_IPV4_EN_MASK        (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_CONTROL_REG_DSCP_IPV4_EN_SHIFT       (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_CONTROL_REG_DSCP_IPV4_EN_MAX         (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_CONTROL_REG_DSCP_IPV6_EN_MASK        (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_CONTROL_REG_DSCP_IPV6_EN_SHIFT       (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_CONTROL_REG_DSCP_IPV6_EN_MAX         (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_CONTROL_REG_TX_LPI_CLKSTOP_EN_MASK   (0x00001000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_CONTROL_REG_TX_LPI_CLKSTOP_EN_SHIFT  (0x0000000CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_CONTROL_REG_TX_LPI_CLKSTOP_EN_MAX    (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_CONTROL_REG_TX_ECC_ERR_EN_MASK       (0x00004000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_CONTROL_REG_TX_ECC_ERR_EN_SHIFT      (0x0000000EU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_CONTROL_REG_TX_ECC_ERR_EN_MAX        (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_CONTROL_REG_RX_ECC_ERR_EN_MASK       (0x00008000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_CONTROL_REG_RX_ECC_ERR_EN_SHIFT      (0x0000000FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_CONTROL_REG_RX_ECC_ERR_EN_MAX        (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_CONTROL_REG_IET_PORT_EN_MASK         (0x00010000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_CONTROL_REG_IET_PORT_EN_SHIFT        (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_CONTROL_REG_IET_PORT_EN_MAX          (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_CONTROL_REG_EST_PORT_EN_MASK         (0x00020000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_CONTROL_REG_EST_PORT_EN_SHIFT        (0x00000011U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_CONTROL_REG_EST_PORT_EN_MAX          (0x00000001U)

/* PN_MAX_BLKS_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAX_BLKS_REG_RX_MAX_BLKS_MASK        (0x000000FFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAX_BLKS_REG_RX_MAX_BLKS_SHIFT       (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAX_BLKS_REG_RX_MAX_BLKS_MAX         (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAX_BLKS_REG_TX_MAX_BLKS_MASK        (0x0000FF00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAX_BLKS_REG_TX_MAX_BLKS_SHIFT       (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAX_BLKS_REG_TX_MAX_BLKS_MAX         (0x000000FFU)

/* PN_BLK_CNT_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_BLK_CNT_REG_RX_BLK_CNT_E_MASK        (0x0000003FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_BLK_CNT_REG_RX_BLK_CNT_E_SHIFT       (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_BLK_CNT_REG_RX_BLK_CNT_E_MAX         (0x0000003FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_BLK_CNT_REG_TX_BLK_CNT_MASK          (0x00001F00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_BLK_CNT_REG_TX_BLK_CNT_SHIFT         (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_BLK_CNT_REG_TX_BLK_CNT_MAX           (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_BLK_CNT_REG_RX_BLK_CNT_P_MASK        (0x003F0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_BLK_CNT_REG_RX_BLK_CNT_P_SHIFT       (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_BLK_CNT_REG_RX_BLK_CNT_P_MAX         (0x0000003FU)

/* PN_PORT_VLAN_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PORT_VLAN_REG_PORT_VID_MASK          (0x00000FFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PORT_VLAN_REG_PORT_VID_SHIFT         (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PORT_VLAN_REG_PORT_VID_MAX           (0x00000FFFU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PORT_VLAN_REG_PORT_CFI_MASK          (0x00001000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PORT_VLAN_REG_PORT_CFI_SHIFT         (0x0000000CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PORT_VLAN_REG_PORT_CFI_MAX           (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PORT_VLAN_REG_PORT_PRI_MASK          (0x0000E000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PORT_VLAN_REG_PORT_PRI_SHIFT         (0x0000000DU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PORT_VLAN_REG_PORT_PRI_MAX           (0x00000007U)

/* PN_TX_PRI_MAP_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_PRI_MAP_REG_PRI0_MASK             (0x00000007U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_PRI_MAP_REG_PRI0_SHIFT            (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_PRI_MAP_REG_PRI0_MAX              (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_PRI_MAP_REG_PRI1_MASK             (0x00000070U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_PRI_MAP_REG_PRI1_SHIFT            (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_PRI_MAP_REG_PRI1_MAX              (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_PRI_MAP_REG_PRI2_MASK             (0x00000700U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_PRI_MAP_REG_PRI2_SHIFT            (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_PRI_MAP_REG_PRI2_MAX              (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_PRI_MAP_REG_PRI3_MASK             (0x00007000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_PRI_MAP_REG_PRI3_SHIFT            (0x0000000CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_PRI_MAP_REG_PRI3_MAX              (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_PRI_MAP_REG_PRI4_MASK             (0x00070000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_PRI_MAP_REG_PRI4_SHIFT            (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_PRI_MAP_REG_PRI4_MAX              (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_PRI_MAP_REG_PRI5_MASK             (0x00700000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_PRI_MAP_REG_PRI5_SHIFT            (0x00000014U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_PRI_MAP_REG_PRI5_MAX              (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_PRI_MAP_REG_PRI6_MASK             (0x07000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_PRI_MAP_REG_PRI6_SHIFT            (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_PRI_MAP_REG_PRI6_MAX              (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_PRI_MAP_REG_PRI7_MASK             (0x70000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_PRI_MAP_REG_PRI7_SHIFT            (0x0000001CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_PRI_MAP_REG_PRI7_MAX              (0x00000007U)

/* PN_PRI_CTL_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PRI_CTL_REG_TX_HOST_BLKS_REM_MASK    (0x0000F000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PRI_CTL_REG_TX_HOST_BLKS_REM_SHIFT   (0x0000000CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PRI_CTL_REG_TX_HOST_BLKS_REM_MAX     (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PRI_CTL_REG_RX_FLOW_PRI_MASK         (0x00FF0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PRI_CTL_REG_RX_FLOW_PRI_SHIFT        (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PRI_CTL_REG_RX_FLOW_PRI_MAX          (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PRI_CTL_REG_TX_FLOW_PRI_MASK         (0xFF000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PRI_CTL_REG_TX_FLOW_PRI_SHIFT        (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PRI_CTL_REG_TX_FLOW_PRI_MAX          (0x000000FFU)

/* PN_RX_PRI_MAP_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_PRI_MAP_REG_PRI0_MASK             (0x00000007U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_PRI_MAP_REG_PRI0_SHIFT            (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_PRI_MAP_REG_PRI0_MAX              (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_PRI_MAP_REG_PRI1_MASK             (0x00000070U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_PRI_MAP_REG_PRI1_SHIFT            (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_PRI_MAP_REG_PRI1_MAX              (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_PRI_MAP_REG_PRI2_MASK             (0x00000700U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_PRI_MAP_REG_PRI2_SHIFT            (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_PRI_MAP_REG_PRI2_MAX              (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_PRI_MAP_REG_PRI3_MASK             (0x00007000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_PRI_MAP_REG_PRI3_SHIFT            (0x0000000CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_PRI_MAP_REG_PRI3_MAX              (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_PRI_MAP_REG_PRI4_MASK             (0x00070000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_PRI_MAP_REG_PRI4_SHIFT            (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_PRI_MAP_REG_PRI4_MAX              (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_PRI_MAP_REG_PRI5_MASK             (0x00700000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_PRI_MAP_REG_PRI5_SHIFT            (0x00000014U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_PRI_MAP_REG_PRI5_MAX              (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_PRI_MAP_REG_PRI6_MASK             (0x07000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_PRI_MAP_REG_PRI6_SHIFT            (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_PRI_MAP_REG_PRI6_MAX              (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_PRI_MAP_REG_PRI7_MASK             (0x70000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_PRI_MAP_REG_PRI7_SHIFT            (0x0000001CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_PRI_MAP_REG_PRI7_MAX              (0x00000007U)

/* PN_RX_MAXLEN_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_MAXLEN_REG_RX_MAXLEN_MASK         (0x00003FFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_MAXLEN_REG_RX_MAXLEN_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_MAXLEN_REG_RX_MAXLEN_MAX          (0x00003FFFU)

/* PN_TX_BLKS_PRI_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_BLKS_PRI_REG_PRI0_MASK            (0x0000000FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_BLKS_PRI_REG_PRI0_SHIFT           (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_BLKS_PRI_REG_PRI0_MAX             (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_BLKS_PRI_REG_PRI1_MASK            (0x000000F0U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_BLKS_PRI_REG_PRI1_SHIFT           (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_BLKS_PRI_REG_PRI1_MAX             (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_BLKS_PRI_REG_PRI2_MASK            (0x00000F00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_BLKS_PRI_REG_PRI2_SHIFT           (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_BLKS_PRI_REG_PRI2_MAX             (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_BLKS_PRI_REG_PRI3_MASK            (0x0000F000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_BLKS_PRI_REG_PRI3_SHIFT           (0x0000000CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_BLKS_PRI_REG_PRI3_MAX             (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_BLKS_PRI_REG_PRI4_MASK            (0x000F0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_BLKS_PRI_REG_PRI4_SHIFT           (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_BLKS_PRI_REG_PRI4_MAX             (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_BLKS_PRI_REG_PRI5_MASK            (0x00F00000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_BLKS_PRI_REG_PRI5_SHIFT           (0x00000014U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_BLKS_PRI_REG_PRI5_MAX             (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_BLKS_PRI_REG_PRI6_MASK            (0x0F000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_BLKS_PRI_REG_PRI6_SHIFT           (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_BLKS_PRI_REG_PRI6_MAX             (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_BLKS_PRI_REG_PRI7_MASK            (0xF0000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_BLKS_PRI_REG_PRI7_SHIFT           (0x0000001CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_BLKS_PRI_REG_PRI7_MAX             (0x0000000FU)

/* PN_RX_FLOW_THRESH_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_FLOW_THRESH_REG_COUNT_MASK        (0x000001FFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_FLOW_THRESH_REG_COUNT_SHIFT       (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_FLOW_THRESH_REG_COUNT_MAX         (0x000001FFU)

/* PN_IDLE2LPI_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_IDLE2LPI_REG_COUNT_MASK              (0x00FFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_IDLE2LPI_REG_COUNT_SHIFT             (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_IDLE2LPI_REG_COUNT_MAX               (0x00FFFFFFU)

/* PN_LPI2WAKE_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_LPI2WAKE_REG_COUNT_MASK              (0x00FFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_LPI2WAKE_REG_COUNT_SHIFT             (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_LPI2WAKE_REG_COUNT_MAX               (0x00FFFFFFU)

/* PN_EEE_STATUS_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EEE_STATUS_REG_WAIT_IDLE2LPI_MASK    (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EEE_STATUS_REG_WAIT_IDLE2LPI_SHIFT   (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EEE_STATUS_REG_WAIT_IDLE2LPI_MAX     (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EEE_STATUS_REG_RX_LPI_MASK           (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EEE_STATUS_REG_RX_LPI_SHIFT          (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EEE_STATUS_REG_RX_LPI_MAX            (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EEE_STATUS_REG_TX_LPI_MASK           (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EEE_STATUS_REG_TX_LPI_SHIFT          (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EEE_STATUS_REG_TX_LPI_MAX            (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EEE_STATUS_REG_TX_WAKE_MASK          (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EEE_STATUS_REG_TX_WAKE_SHIFT         (0x00000003U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EEE_STATUS_REG_TX_WAKE_MAX           (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EEE_STATUS_REG_TX_FIFO_HOLD_MASK     (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EEE_STATUS_REG_TX_FIFO_HOLD_SHIFT    (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EEE_STATUS_REG_TX_FIFO_HOLD_MAX      (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EEE_STATUS_REG_RX_FIFO_EMPTY_MASK    (0x00000020U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EEE_STATUS_REG_RX_FIFO_EMPTY_SHIFT   (0x00000005U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EEE_STATUS_REG_RX_FIFO_EMPTY_MAX     (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EEE_STATUS_REG_TX_FIFO_EMPTY_MASK    (0x00000040U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EEE_STATUS_REG_TX_FIFO_EMPTY_SHIFT   (0x00000006U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EEE_STATUS_REG_TX_FIFO_EMPTY_MAX     (0x00000001U)

/* PN_FIFO_STATUS_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_FIFO_STATUS_REG_TX_PRI_ACTIVE_MASK   (0x000000FFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_FIFO_STATUS_REG_TX_PRI_ACTIVE_SHIFT  (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_FIFO_STATUS_REG_TX_PRI_ACTIVE_MAX    (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_FIFO_STATUS_REG_TX_E_MAC_ALLOW_MASK  (0x0000FF00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_FIFO_STATUS_REG_TX_E_MAC_ALLOW_SHIFT (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_FIFO_STATUS_REG_TX_E_MAC_ALLOW_MAX   (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_FIFO_STATUS_REG_EST_CNT_ERR_MASK     (0x00010000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_FIFO_STATUS_REG_EST_CNT_ERR_SHIFT    (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_FIFO_STATUS_REG_EST_CNT_ERR_MAX      (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_FIFO_STATUS_REG_EST_ADD_ERR_MASK     (0x00020000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_FIFO_STATUS_REG_EST_ADD_ERR_SHIFT    (0x00000011U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_FIFO_STATUS_REG_EST_ADD_ERR_MAX      (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_FIFO_STATUS_REG_EST_BUFACT_MASK      (0x00040000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_FIFO_STATUS_REG_EST_BUFACT_SHIFT     (0x00000012U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_FIFO_STATUS_REG_EST_BUFACT_MAX       (0x00000001U)

/* PN_EST_CONTROL_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EST_CONTROL_REG_EST_ONEBUF_MASK      (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EST_CONTROL_REG_EST_ONEBUF_SHIFT     (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EST_CONTROL_REG_EST_ONEBUF_MAX       (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EST_CONTROL_REG_EST_BUFSEL_MASK      (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EST_CONTROL_REG_EST_BUFSEL_SHIFT     (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EST_CONTROL_REG_EST_BUFSEL_MAX       (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EST_CONTROL_REG_EST_TS_EN_MASK       (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EST_CONTROL_REG_EST_TS_EN_SHIFT      (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EST_CONTROL_REG_EST_TS_EN_MAX        (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EST_CONTROL_REG_EST_TS_FIRST_MASK    (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EST_CONTROL_REG_EST_TS_FIRST_SHIFT   (0x00000003U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EST_CONTROL_REG_EST_TS_FIRST_MAX     (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EST_CONTROL_REG_EST_TS_ONEPRI_MASK   (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EST_CONTROL_REG_EST_TS_ONEPRI_SHIFT  (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EST_CONTROL_REG_EST_TS_ONEPRI_MAX    (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EST_CONTROL_REG_EST_TS_PRI_MASK      (0x000000E0U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EST_CONTROL_REG_EST_TS_PRI_SHIFT     (0x00000005U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EST_CONTROL_REG_EST_TS_PRI_MAX       (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EST_CONTROL_REG_EST_FILL_EN_MASK     (0x00000100U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EST_CONTROL_REG_EST_FILL_EN_SHIFT    (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EST_CONTROL_REG_EST_FILL_EN_MAX      (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EST_CONTROL_REG_EST_FILL_MARGIN_MASK (0x03FF0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EST_CONTROL_REG_EST_FILL_MARGIN_SHIFT (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_EST_CONTROL_REG_EST_FILL_MARGIN_MAX  (0x000003FFU)

/* PN_RX_DSCP_MAP_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_DSCP_MAP_REG_PRI0_MASK            (0x00000007U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_DSCP_MAP_REG_PRI0_SHIFT           (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_DSCP_MAP_REG_PRI0_MAX             (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_DSCP_MAP_REG_PRI1_MASK            (0x00000070U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_DSCP_MAP_REG_PRI1_SHIFT           (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_DSCP_MAP_REG_PRI1_MAX             (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_DSCP_MAP_REG_PRI2_MASK            (0x00000700U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_DSCP_MAP_REG_PRI2_SHIFT           (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_DSCP_MAP_REG_PRI2_MAX             (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_DSCP_MAP_REG_PRI3_MASK            (0x00007000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_DSCP_MAP_REG_PRI3_SHIFT           (0x0000000CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_DSCP_MAP_REG_PRI3_MAX             (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_DSCP_MAP_REG_PRI4_MASK            (0x00070000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_DSCP_MAP_REG_PRI4_SHIFT           (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_DSCP_MAP_REG_PRI4_MAX             (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_DSCP_MAP_REG_PRI5_MASK            (0x00700000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_DSCP_MAP_REG_PRI5_SHIFT           (0x00000014U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_DSCP_MAP_REG_PRI5_MAX             (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_DSCP_MAP_REG_PRI6_MASK            (0x07000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_DSCP_MAP_REG_PRI6_SHIFT           (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_DSCP_MAP_REG_PRI6_MAX             (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_DSCP_MAP_REG_PRI7_MASK            (0x70000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_DSCP_MAP_REG_PRI7_SHIFT           (0x0000001CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_RX_DSCP_MAP_REG_PRI7_MAX             (0x00000007U)

/* PN_PRI_CIR_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PRI_CIR_REG_PRI_CIR_MASK             (0x0FFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PRI_CIR_REG_PRI_CIR_SHIFT            (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PRI_CIR_REG_PRI_CIR_MAX              (0x0FFFFFFFU)

/* PN_PRI_EIR_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PRI_EIR_REG_PRI_EIR_MASK             (0x0FFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PRI_EIR_REG_PRI_EIR_SHIFT            (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_PRI_EIR_REG_PRI_EIR_MAX              (0x0FFFFFFFU)

/* PN_TX_D_THRESH_SET_L_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_SET_L_REG_PRI0_MASK      (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_SET_L_REG_PRI0_SHIFT     (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_SET_L_REG_PRI0_MAX       (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_SET_L_REG_PRI1_MASK      (0x00001F00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_SET_L_REG_PRI1_SHIFT     (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_SET_L_REG_PRI1_MAX       (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_SET_L_REG_PRI2_MASK      (0x001F0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_SET_L_REG_PRI2_SHIFT     (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_SET_L_REG_PRI2_MAX       (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_SET_L_REG_PRI3_MASK      (0x1F000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_SET_L_REG_PRI3_SHIFT     (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_SET_L_REG_PRI3_MAX       (0x0000001FU)

/* PN_TX_D_THRESH_SET_H_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_SET_H_REG_PRI4_MASK      (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_SET_H_REG_PRI4_SHIFT     (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_SET_H_REG_PRI4_MAX       (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_SET_H_REG_PRI5_MASK      (0x00001F00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_SET_H_REG_PRI5_SHIFT     (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_SET_H_REG_PRI5_MAX       (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_SET_H_REG_PRI6_MASK      (0x001F0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_SET_H_REG_PRI6_SHIFT     (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_SET_H_REG_PRI6_MAX       (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_SET_H_REG_PRI7_MASK      (0x1F000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_SET_H_REG_PRI7_SHIFT     (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_SET_H_REG_PRI7_MAX       (0x0000001FU)

/* PN_TX_D_THRESH_CLR_L_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_CLR_L_REG_PRI0_MASK      (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_CLR_L_REG_PRI0_SHIFT     (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_CLR_L_REG_PRI0_MAX       (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_CLR_L_REG_PRI1_MASK      (0x00001F00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_CLR_L_REG_PRI1_SHIFT     (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_CLR_L_REG_PRI1_MAX       (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_CLR_L_REG_PRI2_MASK      (0x001F0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_CLR_L_REG_PRI2_SHIFT     (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_CLR_L_REG_PRI2_MAX       (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_CLR_L_REG_PRI3_MASK      (0x1F000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_CLR_L_REG_PRI3_SHIFT     (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_CLR_L_REG_PRI3_MAX       (0x0000001FU)

/* PN_TX_D_THRESH_CLR_H_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_CLR_H_REG_PRI4_MASK      (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_CLR_H_REG_PRI4_SHIFT     (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_CLR_H_REG_PRI4_MAX       (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_CLR_H_REG_PRI5_MASK      (0x00001F00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_CLR_H_REG_PRI5_SHIFT     (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_CLR_H_REG_PRI5_MAX       (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_CLR_H_REG_PRI6_MASK      (0x001F0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_CLR_H_REG_PRI6_SHIFT     (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_CLR_H_REG_PRI6_MAX       (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_CLR_H_REG_PRI7_MASK      (0x1F000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_CLR_H_REG_PRI7_SHIFT     (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_THRESH_CLR_H_REG_PRI7_MAX       (0x0000001FU)

/* PN_TX_G_BUF_THRESH_SET_L_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_SET_L_REG_PRI0_MASK  (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_SET_L_REG_PRI0_SHIFT (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_SET_L_REG_PRI0_MAX   (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_SET_L_REG_PRI1_MASK  (0x00001F00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_SET_L_REG_PRI1_SHIFT (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_SET_L_REG_PRI1_MAX   (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_SET_L_REG_PRI2_MASK  (0x001F0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_SET_L_REG_PRI2_SHIFT (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_SET_L_REG_PRI2_MAX   (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_SET_L_REG_PRI3_MASK  (0x1F000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_SET_L_REG_PRI3_SHIFT (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_SET_L_REG_PRI3_MAX   (0x0000001FU)

/* PN_TX_G_BUF_THRESH_SET_H_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_SET_H_REG_PRI4_MASK  (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_SET_H_REG_PRI4_SHIFT (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_SET_H_REG_PRI4_MAX   (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_SET_H_REG_PRI5_MASK  (0x00001F00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_SET_H_REG_PRI5_SHIFT (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_SET_H_REG_PRI5_MAX   (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_SET_H_REG_PRI6_MASK  (0x001F0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_SET_H_REG_PRI6_SHIFT (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_SET_H_REG_PRI6_MAX   (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_SET_H_REG_PRI7_MASK  (0x1F000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_SET_H_REG_PRI7_SHIFT (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_SET_H_REG_PRI7_MAX   (0x0000001FU)

/* PN_TX_G_BUF_THRESH_CLR_L_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI0_MASK  (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI0_SHIFT (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI0_MAX   (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI1_MASK  (0x00001F00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI1_SHIFT (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI1_MAX   (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI2_MASK  (0x001F0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI2_SHIFT (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI2_MAX   (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI3_MASK  (0x1F000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI3_SHIFT (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_CLR_L_REG_PRI3_MAX   (0x0000001FU)

/* PN_TX_G_BUF_THRESH_CLR_H_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI4_MASK  (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI4_SHIFT (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI4_MAX   (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI5_MASK  (0x00001F00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI5_SHIFT (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI5_MAX   (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI6_MASK  (0x001F0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI6_SHIFT (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI6_MAX   (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI7_MASK  (0x1F000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI7_SHIFT (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_G_BUF_THRESH_CLR_H_REG_PRI7_MAX   (0x0000001FU)

/* PN_TX_D_OFLOW_ADDVAL_L_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI0_MASK    (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI0_SHIFT   (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI0_MAX     (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI1_MASK    (0x00001F00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI1_SHIFT   (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI1_MAX     (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI2_MASK    (0x001F0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI2_SHIFT   (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI2_MAX     (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI3_MASK    (0x1F000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI3_SHIFT   (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_OFLOW_ADDVAL_L_REG_PRI3_MAX     (0x0000001FU)

/* PN_TX_D_OFLOW_ADDVAL_H_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI4_MASK    (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI4_SHIFT   (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI4_MAX     (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI5_MASK    (0x00001F00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI5_SHIFT   (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI5_MAX     (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI6_MASK    (0x001F0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI6_SHIFT   (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI6_MAX     (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI7_MASK    (0x1F000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI7_SHIFT   (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TX_D_OFLOW_ADDVAL_H_REG_PRI7_MAX     (0x0000001FU)

/* PN_SA_L_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_SA_L_REG_MACSRCADDR_15_8_MASK        (0x000000FFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_SA_L_REG_MACSRCADDR_15_8_SHIFT       (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_SA_L_REG_MACSRCADDR_15_8_MAX         (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_SA_L_REG_MACSRCADDR_7_0_MASK         (0x0000FF00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_SA_L_REG_MACSRCADDR_7_0_SHIFT        (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_SA_L_REG_MACSRCADDR_7_0_MAX          (0x000000FFU)

/* PN_SA_H_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_SA_H_REG_MACSRCADDR_47_40_MASK       (0x000000FFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_SA_H_REG_MACSRCADDR_47_40_SHIFT      (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_SA_H_REG_MACSRCADDR_47_40_MAX        (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_SA_H_REG_MACSRCADDR_39_32_MASK       (0x0000FF00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_SA_H_REG_MACSRCADDR_39_32_SHIFT      (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_SA_H_REG_MACSRCADDR_39_32_MAX        (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_SA_H_REG_MACSRCADDR_31_24_MASK       (0x00FF0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_SA_H_REG_MACSRCADDR_31_24_SHIFT      (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_SA_H_REG_MACSRCADDR_31_24_MAX        (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_SA_H_REG_MACSRCADDR_23_16_MASK       (0xFF000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_SA_H_REG_MACSRCADDR_23_16_SHIFT      (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_SA_H_REG_MACSRCADDR_23_16_MAX        (0x000000FFU)

/* PN_TS_CTL_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_RX_ANNEX_F_EN_MASK     (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_RX_ANNEX_F_EN_SHIFT    (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_RX_ANNEX_F_EN_MAX      (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE1_EN_MASK (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE1_EN_SHIFT (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE1_EN_MAX  (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE2_EN_MASK (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE2_EN_SHIFT (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_RX_VLAN_LTYPE2_EN_MAX  (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_RX_ANNEX_D_EN_MASK     (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_RX_ANNEX_D_EN_SHIFT    (0x00000003U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_RX_ANNEX_D_EN_MAX      (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_TX_ANNEX_F_EN_MASK     (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_TX_ANNEX_F_EN_SHIFT    (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_TX_ANNEX_F_EN_MAX      (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE1_EN_MASK (0x00000020U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE1_EN_SHIFT (0x00000005U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE1_EN_MAX  (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE2_EN_MASK (0x00000040U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE2_EN_SHIFT (0x00000006U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_TX_VLAN_LTYPE2_EN_MAX  (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_TX_ANNEX_D_EN_MASK     (0x00000080U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_TX_ANNEX_D_EN_SHIFT    (0x00000007U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_TX_ANNEX_D_EN_MAX      (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_LTYPE2_EN_MASK         (0x00000100U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_LTYPE2_EN_SHIFT        (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_LTYPE2_EN_MAX          (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_RX_ANNEX_E_EN_MASK     (0x00000200U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_RX_ANNEX_E_EN_SHIFT    (0x00000009U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_RX_ANNEX_E_EN_MAX      (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_TX_ANNEX_E_EN_MASK     (0x00000400U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_TX_ANNEX_E_EN_SHIFT    (0x0000000AU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_TX_ANNEX_E_EN_MAX      (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_TX_HOST_TS_EN_MASK     (0x00000800U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_TX_HOST_TS_EN_SHIFT    (0x0000000BU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_TX_HOST_TS_EN_MAX      (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_MSG_TYPE_EN_MASK       (0xFFFF0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_MSG_TYPE_EN_SHIFT      (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_REG_TS_MSG_TYPE_EN_MAX        (0x0000FFFFU)

/* PN_TS_SEQ_LTYPE_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_SEQ_LTYPE_REG_TS_LTYPE1_MASK      (0x0000FFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_SEQ_LTYPE_REG_TS_LTYPE1_SHIFT     (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_SEQ_LTYPE_REG_TS_LTYPE1_MAX       (0x0000FFFFU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_SEQ_LTYPE_REG_TS_SEQ_ID_OFFSET_MASK (0x003F0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_SEQ_LTYPE_REG_TS_SEQ_ID_OFFSET_SHIFT (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_SEQ_LTYPE_REG_TS_SEQ_ID_OFFSET_MAX (0x0000003FU)

/* PN_TS_VLAN_LTYPE_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE1_MASK (0x0000FFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE1_SHIFT (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE1_MAX (0x0000FFFFU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE2_MASK (0xFFFF0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE2_SHIFT (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_VLAN_LTYPE_REG_TS_VLAN_LTYPE2_MAX (0x0000FFFFU)

/* PN_TS_CTL_LTYPE2_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_LTYPE2_MASK     (0x0000FFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_LTYPE2_SHIFT    (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_LTYPE2_MAX      (0x0000FFFFU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_107_MASK        (0x00010000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_107_SHIFT       (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_107_MAX         (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_129_MASK        (0x00020000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_129_SHIFT       (0x00000011U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_129_MAX         (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_130_MASK        (0x00040000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_130_SHIFT       (0x00000012U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_130_MAX         (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_131_MASK        (0x00080000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_131_SHIFT       (0x00000013U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_131_MAX         (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_132_MASK        (0x00100000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_132_SHIFT       (0x00000014U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_132_MAX         (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_319_MASK        (0x00200000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_319_SHIFT       (0x00000015U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_319_MAX         (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_320_MASK        (0x00400000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_320_SHIFT       (0x00000016U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_320_MAX         (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_TTL_NONZERO_MASK (0x00800000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_TTL_NONZERO_SHIFT (0x00000017U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_TTL_NONZERO_MAX (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_UNI_EN_MASK     (0x01000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_UNI_EN_SHIFT    (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL_LTYPE2_REG_TS_UNI_EN_MAX      (0x00000001U)

/* PN_TS_CTL2_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL2_REG_TS_MCAST_TYPE_EN_MASK    (0x0000FFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL2_REG_TS_MCAST_TYPE_EN_SHIFT   (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL2_REG_TS_MCAST_TYPE_EN_MAX     (0x0000FFFFU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL2_REG_TS_DOMAIN_OFFSET_MASK    (0x003F0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL2_REG_TS_DOMAIN_OFFSET_SHIFT   (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_TS_CTL2_REG_TS_DOMAIN_OFFSET_MAX     (0x0000003FU)

/* PN_MAC_CONTROL_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_FULLDUPLEX_MASK      (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_FULLDUPLEX_SHIFT     (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_FULLDUPLEX_MAX       (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_LOOPBACK_MASK        (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_LOOPBACK_SHIFT       (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_LOOPBACK_MAX         (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_MTEST_MASK           (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_MTEST_SHIFT          (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_MTEST_MAX            (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_RX_FLOW_EN_MASK      (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_RX_FLOW_EN_SHIFT     (0x00000003U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_RX_FLOW_EN_MAX       (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_TX_FLOW_EN_MASK      (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_TX_FLOW_EN_SHIFT     (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_TX_FLOW_EN_MAX       (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_GMII_EN_MASK         (0x00000020U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_GMII_EN_SHIFT        (0x00000005U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_GMII_EN_MAX          (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_TX_PACE_MASK         (0x00000040U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_TX_PACE_SHIFT        (0x00000006U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_TX_PACE_MAX          (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_GIG_MASK             (0x00000080U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_GIG_SHIFT            (0x00000007U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_GIG_MAX              (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_TX_SHORT_GAP_ENABLE_MASK (0x00000400U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_TX_SHORT_GAP_ENABLE_SHIFT (0x0000000AU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_TX_SHORT_GAP_ENABLE_MAX (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_CMD_IDLE_MASK        (0x00000800U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_CMD_IDLE_SHIFT       (0x0000000BU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_CMD_IDLE_MAX         (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_CRC_TYPE_MASK        (0x00001000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_CRC_TYPE_SHIFT       (0x0000000CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_CRC_TYPE_MAX         (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_IFCTL_A_MASK         (0x00008000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_IFCTL_A_SHIFT        (0x0000000FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_IFCTL_A_MAX          (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_IFCTL_B_MASK         (0x00010000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_IFCTL_B_SHIFT        (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_IFCTL_B_MAX          (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_GIG_FORCE_MASK       (0x00020000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_GIG_FORCE_SHIFT      (0x00000011U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_GIG_FORCE_MAX        (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_CTL_EN_MASK          (0x00040000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_CTL_EN_SHIFT         (0x00000012U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_CTL_EN_MAX           (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_EXT_RX_FLOW_EN_MASK  (0x00080000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_EXT_RX_FLOW_EN_SHIFT (0x00000013U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_EXT_RX_FLOW_EN_MAX   (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_EXT_TX_FLOW_EN_MASK  (0x00100000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_EXT_TX_FLOW_EN_SHIFT (0x00000014U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_EXT_TX_FLOW_EN_MAX   (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_TX_SHORT_GAP_LIM_EN_MASK (0x00200000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_TX_SHORT_GAP_LIM_EN_SHIFT (0x00000015U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_TX_SHORT_GAP_LIM_EN_MAX (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_RX_CEF_EN_MASK       (0x00400000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_RX_CEF_EN_SHIFT      (0x00000016U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_RX_CEF_EN_MAX        (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_RX_CSF_EN_MASK       (0x00800000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_RX_CSF_EN_SHIFT      (0x00000017U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_RX_CSF_EN_MAX        (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_RX_CMF_EN_MASK       (0x01000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_RX_CMF_EN_SHIFT      (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_CONTROL_REG_RX_CMF_EN_MAX        (0x00000001U)

/* PN_MAC_STATUS_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_TX_FLOW_ACT_MASK      (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_TX_FLOW_ACT_SHIFT     (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_TX_FLOW_ACT_MAX       (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_RX_FLOW_ACT_MASK      (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_RX_FLOW_ACT_SHIFT     (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_RX_FLOW_ACT_MAX       (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_EXT_FULLDUPLEX_MASK   (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_EXT_FULLDUPLEX_SHIFT  (0x00000003U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_EXT_FULLDUPLEX_MAX    (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_EXT_GIG_MASK          (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_EXT_GIG_SHIFT         (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_EXT_GIG_MAX           (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_EXT_TX_FLOW_EN_MASK   (0x00000020U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_EXT_TX_FLOW_EN_SHIFT  (0x00000005U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_EXT_TX_FLOW_EN_MAX    (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_EXT_RX_FLOW_EN_MASK   (0x00000040U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_EXT_RX_FLOW_EN_SHIFT  (0x00000006U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_EXT_RX_FLOW_EN_MAX    (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_RX_PFC_FLOW_ACT_MASK  (0x0000FF00U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_RX_PFC_FLOW_ACT_SHIFT (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_RX_PFC_FLOW_ACT_MAX   (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_TX_PFC_FLOW_ACT_MASK  (0x00FF0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_TX_PFC_FLOW_ACT_SHIFT (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_TX_PFC_FLOW_ACT_MAX   (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_TORF_PRI_MASK         (0x07000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_TORF_PRI_SHIFT        (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_TORF_PRI_MAX          (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_TORF_MASK             (0x08000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_TORF_SHIFT            (0x0000001BU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_TORF_MAX              (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_TX_IDLE_MASK          (0x10000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_TX_IDLE_SHIFT         (0x0000001CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_TX_IDLE_MAX           (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_P_IDLE_MASK           (0x20000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_P_IDLE_SHIFT          (0x0000001DU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_P_IDLE_MAX            (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_E_IDLE_MASK           (0x40000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_E_IDLE_SHIFT          (0x0000001EU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_E_IDLE_MAX            (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_IDLE_MASK             (0x80000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_IDLE_SHIFT            (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_STATUS_REG_IDLE_MAX              (0x00000001U)

/* PN_MAC_SOFT_RESET_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_SOFT_RESET_REG_SOFT_RESET_MASK   (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_SOFT_RESET_REG_SOFT_RESET_SHIFT  (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_SOFT_RESET_REG_SOFT_RESET_MAX    (0x00000001U)

/* PN_MAC_BOFFTEST_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_BOFFTEST_REG_TX_BACKOFF_MASK     (0x000003FFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_BOFFTEST_REG_TX_BACKOFF_SHIFT    (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_BOFFTEST_REG_TX_BACKOFF_MAX      (0x000003FFU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_BOFFTEST_REG_COLL_COUNT_MASK     (0x0000F000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_BOFFTEST_REG_COLL_COUNT_SHIFT    (0x0000000CU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_BOFFTEST_REG_COLL_COUNT_MAX      (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_BOFFTEST_REG_RNDNUM_MASK         (0x03FF0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_BOFFTEST_REG_RNDNUM_SHIFT        (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_BOFFTEST_REG_RNDNUM_MAX          (0x000003FFU)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_BOFFTEST_REG_PACEVAL_MASK        (0x7C000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_BOFFTEST_REG_PACEVAL_SHIFT       (0x0000001AU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_BOFFTEST_REG_PACEVAL_MAX         (0x0000001FU)

/* PN_MAC_RX_PAUSETIMER_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_RX_PAUSETIMER_REG_RX_PAUSETIMER_MASK (0x0000FFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_RX_PAUSETIMER_REG_RX_PAUSETIMER_SHIFT (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_RX_PAUSETIMER_REG_RX_PAUSETIMER_MAX (0x0000FFFFU)

/* PN_MAC_RXN_PAUSETIMER_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_RXN_PAUSETIMER_REG_RX_PAUSETIMER_MASK (0x0000FFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_RXN_PAUSETIMER_REG_RX_PAUSETIMER_SHIFT (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_RXN_PAUSETIMER_REG_RX_PAUSETIMER_MAX (0x0000FFFFU)

/* PN_MAC_TX_PAUSETIMER_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_TX_PAUSETIMER_REG_TX_PAUSETIMER_MASK (0x0000FFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_TX_PAUSETIMER_REG_TX_PAUSETIMER_SHIFT (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_TX_PAUSETIMER_REG_TX_PAUSETIMER_MAX (0x0000FFFFU)

/* PN_MAC_TXN_PAUSETIMER_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_TXN_PAUSETIMER_REG_TX_PAUSETIMER_MASK (0x0000FFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_TXN_PAUSETIMER_REG_TX_PAUSETIMER_SHIFT (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_TXN_PAUSETIMER_REG_TX_PAUSETIMER_MAX (0x0000FFFFU)

/* PN_MAC_EMCONTROL_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_EMCONTROL_REG_FREE_MASK          (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_EMCONTROL_REG_FREE_SHIFT         (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_EMCONTROL_REG_FREE_MAX           (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_EMCONTROL_REG_SOFT_MASK          (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_EMCONTROL_REG_SOFT_SHIFT         (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_EMCONTROL_REG_SOFT_MAX           (0x00000001U)

/* PN_MAC_TX_GAP_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_TX_GAP_REG_TX_GAP_MASK           (0x0000FFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_TX_GAP_REG_TX_GAP_SHIFT          (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_ETH_PN_MAC_TX_GAP_REG_TX_GAP_MAX            (0x0000FFFFU)

/* FETCH_LOC */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_EST_FETCH_LOC_LOC_MASK                      (0x003FFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_EST_FETCH_LOC_LOC_SHIFT                     (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_EST_FETCH_LOC_LOC_MAX                       (0x003FFFFFU)

/* RXGOODFRAMES */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXGOODFRAMES_COUNT_MASK              (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXGOODFRAMES_COUNT_SHIFT             (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXGOODFRAMES_COUNT_MAX               (0xFFFFFFFFU)

/* RXBROADCASTFRAMES */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXBROADCASTFRAMES_COUNT_MASK         (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXBROADCASTFRAMES_COUNT_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXBROADCASTFRAMES_COUNT_MAX          (0xFFFFFFFFU)

/* RXMULTICASTFRAMES */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXMULTICASTFRAMES_COUNT_MASK         (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXMULTICASTFRAMES_COUNT_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXMULTICASTFRAMES_COUNT_MAX          (0xFFFFFFFFU)

/* RXCRCERRORS */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXCRCERRORS_COUNT_MASK               (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXCRCERRORS_COUNT_SHIFT              (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXCRCERRORS_COUNT_MAX                (0xFFFFFFFFU)

/* RXOVERSIZEDFRAMES */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXOVERSIZEDFRAMES_COUNT_MASK         (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXOVERSIZEDFRAMES_COUNT_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXOVERSIZEDFRAMES_COUNT_MAX          (0xFFFFFFFFU)

/* RXUNDERSIZEDFRAMES */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXUNDERSIZEDFRAMES_COUNT_MASK        (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXUNDERSIZEDFRAMES_COUNT_SHIFT       (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXUNDERSIZEDFRAMES_COUNT_MAX         (0xFFFFFFFFU)

/* RXFRAGMENTS */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXFRAGMENTS_COUNT_MASK               (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXFRAGMENTS_COUNT_SHIFT              (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXFRAGMENTS_COUNT_MAX                (0xFFFFFFFFU)

/* ALE_DROP */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_DROP_COUNT_MASK                  (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_DROP_COUNT_SHIFT                 (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_DROP_COUNT_MAX                   (0xFFFFFFFFU)

/* ALE_OVERRUN_DROP */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_OVERRUN_DROP_COUNT_MASK          (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_OVERRUN_DROP_COUNT_SHIFT         (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_OVERRUN_DROP_COUNT_MAX           (0xFFFFFFFFU)

/* RXOCTETS */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXOCTETS_COUNT_MASK                  (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXOCTETS_COUNT_SHIFT                 (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RXOCTETS_COUNT_MAX                   (0xFFFFFFFFU)

/* TXGOODFRAMES */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_TXGOODFRAMES_COUNT_MASK              (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_TXGOODFRAMES_COUNT_SHIFT             (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_TXGOODFRAMES_COUNT_MAX               (0xFFFFFFFFU)

/* TXBROADCASTFRAMES */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_TXBROADCASTFRAMES_COUNT_MASK         (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_TXBROADCASTFRAMES_COUNT_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_TXBROADCASTFRAMES_COUNT_MAX          (0xFFFFFFFFU)

/* TXMULTICASTFRAMES */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_TXMULTICASTFRAMES_COUNT_MASK         (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_TXMULTICASTFRAMES_COUNT_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_TXMULTICASTFRAMES_COUNT_MAX          (0xFFFFFFFFU)

/* TXOCTETS */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_TXOCTETS_COUNT_MASK                  (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_TXOCTETS_COUNT_SHIFT                 (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_TXOCTETS_COUNT_MAX                   (0xFFFFFFFFU)

/* OCTETFRAMES64 */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_OCTETFRAMES64_COUNT_MASK             (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_OCTETFRAMES64_COUNT_SHIFT            (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_OCTETFRAMES64_COUNT_MAX              (0xFFFFFFFFU)

/* OCTETFRAMES65T127 */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_OCTETFRAMES65T127_COUNT_MASK         (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_OCTETFRAMES65T127_COUNT_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_OCTETFRAMES65T127_COUNT_MAX          (0xFFFFFFFFU)

/* OCTETFRAMES128T255 */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_OCTETFRAMES128T255_COUNT_MASK        (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_OCTETFRAMES128T255_COUNT_SHIFT       (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_OCTETFRAMES128T255_COUNT_MAX         (0xFFFFFFFFU)

/* OCTETFRAMES256T511 */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_OCTETFRAMES256T511_COUNT_MASK        (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_OCTETFRAMES256T511_COUNT_SHIFT       (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_OCTETFRAMES256T511_COUNT_MAX         (0xFFFFFFFFU)

/* OCTETFRAMES512T1023 */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_OCTETFRAMES512T1023_COUNT_MASK       (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_OCTETFRAMES512T1023_COUNT_SHIFT      (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_OCTETFRAMES512T1023_COUNT_MAX        (0xFFFFFFFFU)

/* OCTETFRAMES1024TUP */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_OCTETFRAMES1024TUP_COUNT_MASK        (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_OCTETFRAMES1024TUP_COUNT_SHIFT       (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_OCTETFRAMES1024TUP_COUNT_MAX         (0xFFFFFFFFU)

/* NETOCTETS */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_NETOCTETS_COUNT_MASK                 (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_NETOCTETS_COUNT_SHIFT                (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_NETOCTETS_COUNT_MAX                  (0xFFFFFFFFU)

/* RX_BOTTOM_OF_FIFO_DROP */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RX_BOTTOM_OF_FIFO_DROP_COUNT_MASK    (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RX_BOTTOM_OF_FIFO_DROP_COUNT_SHIFT   (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RX_BOTTOM_OF_FIFO_DROP_COUNT_MAX     (0xFFFFFFFFU)

/* PORTMASK_DROP */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_PORTMASK_DROP_COUNT_MASK             (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_PORTMASK_DROP_COUNT_SHIFT            (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_PORTMASK_DROP_COUNT_MAX              (0xFFFFFFFFU)

/* RX_TOP_OF_FIFO_DROP */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RX_TOP_OF_FIFO_DROP_COUNT_MASK       (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RX_TOP_OF_FIFO_DROP_COUNT_SHIFT      (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_RX_TOP_OF_FIFO_DROP_COUNT_MAX        (0xFFFFFFFFU)

/* ALE_RATE_LIMIT_DROP */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_RATE_LIMIT_DROP_COUNT_MASK       (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_RATE_LIMIT_DROP_COUNT_SHIFT      (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_RATE_LIMIT_DROP_COUNT_MAX        (0xFFFFFFFFU)

/* ALE_VID_INGRESS_DROP */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_VID_INGRESS_DROP_COUNT_MASK      (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_VID_INGRESS_DROP_COUNT_SHIFT     (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_VID_INGRESS_DROP_COUNT_MAX       (0xFFFFFFFFU)

/* ALE_DA_EQ_SA_DROP */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_DA_EQ_SA_DROP_COUNT_MASK         (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_DA_EQ_SA_DROP_COUNT_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_DA_EQ_SA_DROP_COUNT_MAX          (0xFFFFFFFFU)

/* ALE_BLOCK_DROP */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_BLOCK_DROP_COUNT_MASK            (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_BLOCK_DROP_COUNT_SHIFT           (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_BLOCK_DROP_COUNT_MAX             (0xFFFFFFFFU)

/* ALE_SECURE_DROP */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_SECURE_DROP_COUNT_MASK           (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_SECURE_DROP_COUNT_SHIFT          (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_SECURE_DROP_COUNT_MAX            (0xFFFFFFFFU)

/* ALE_AUTH_DROP */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_AUTH_DROP_COUNT_MASK             (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_AUTH_DROP_COUNT_SHIFT            (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_AUTH_DROP_COUNT_MAX              (0xFFFFFFFFU)

/* ALE_UNKN_UNI */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_UNKN_UNI_COUNT_MASK              (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_UNKN_UNI_COUNT_SHIFT             (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_UNKN_UNI_COUNT_MAX               (0xFFFFFFFFU)

/* ALE_UNKN_UNI_BCNT */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_UNKN_UNI_BCNT_COUNT_MASK         (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_UNKN_UNI_BCNT_COUNT_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_UNKN_UNI_BCNT_COUNT_MAX          (0xFFFFFFFFU)

/* ALE_UNKN_MLT */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_UNKN_MLT_COUNT_MASK              (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_UNKN_MLT_COUNT_SHIFT             (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_UNKN_MLT_COUNT_MAX               (0xFFFFFFFFU)

/* ALE_UNKN_MLT_BCNT */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_UNKN_MLT_BCNT_COUNT_MASK         (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_UNKN_MLT_BCNT_COUNT_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_UNKN_MLT_BCNT_COUNT_MAX          (0xFFFFFFFFU)

/* ALE_UNKN_BRD */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_UNKN_BRD_COUNT_MASK              (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_UNKN_BRD_COUNT_SHIFT             (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_UNKN_BRD_COUNT_MAX               (0xFFFFFFFFU)

/* ALE_UNKN_BRD_BCNT */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_UNKN_BRD_BCNT_COUNT_MASK         (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_UNKN_BRD_BCNT_COUNT_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_UNKN_BRD_BCNT_COUNT_MAX          (0xFFFFFFFFU)

/* ALE_POL_MATCH */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_POL_MATCH_COUNT_MASK             (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_POL_MATCH_COUNT_SHIFT            (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_POL_MATCH_COUNT_MAX              (0xFFFFFFFFU)

/* ALE_POL_MATCH_RED */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_POL_MATCH_RED_COUNT_MASK         (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_POL_MATCH_RED_COUNT_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_POL_MATCH_RED_COUNT_MAX          (0xFFFFFFFFU)

/* ALE_POL_MATCH_YELLOW */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_POL_MATCH_YELLOW_COUNT_MASK      (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_POL_MATCH_YELLOW_COUNT_SHIFT     (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_ALE_POL_MATCH_YELLOW_COUNT_MAX       (0xFFFFFFFFU)

/* TX_MEMORY_PROTECT_ERROR */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_TX_MEMORY_PROTECT_ERROR_COUNT_MASK   (0x000000FFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_TX_MEMORY_PROTECT_ERROR_COUNT_SHIFT  (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_0_TX_MEMORY_PROTECT_ERROR_COUNT_MAX    (0x000000FFU)

/* RXGOODFRAMES */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXGOODFRAMES_COUNT_MASK              (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXGOODFRAMES_COUNT_SHIFT             (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXGOODFRAMES_COUNT_MAX               (0xFFFFFFFFU)

/* RXBROADCASTFRAMES */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXBROADCASTFRAMES_COUNT_MASK         (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXBROADCASTFRAMES_COUNT_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXBROADCASTFRAMES_COUNT_MAX          (0xFFFFFFFFU)

/* RXMULTICASTFRAMES */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXMULTICASTFRAMES_COUNT_MASK         (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXMULTICASTFRAMES_COUNT_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXMULTICASTFRAMES_COUNT_MAX          (0xFFFFFFFFU)

/* RXPAUSEFRAMES */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXPAUSEFRAMES_COUNT_MASK             (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXPAUSEFRAMES_COUNT_SHIFT            (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXPAUSEFRAMES_COUNT_MAX              (0xFFFFFFFFU)

/* RXCRCERRORS */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXCRCERRORS_COUNT_MASK               (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXCRCERRORS_COUNT_SHIFT              (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXCRCERRORS_COUNT_MAX                (0xFFFFFFFFU)

/* RXALIGNCODEERRORS */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXALIGNCODEERRORS_COUNT_MASK         (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXALIGNCODEERRORS_COUNT_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXALIGNCODEERRORS_COUNT_MAX          (0xFFFFFFFFU)

/* RXOVERSIZEDFRAMES */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXOVERSIZEDFRAMES_COUNT_MASK         (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXOVERSIZEDFRAMES_COUNT_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXOVERSIZEDFRAMES_COUNT_MAX          (0xFFFFFFFFU)

/* RXJABBERFRAMES */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXJABBERFRAMES_COUNT_MASK            (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXJABBERFRAMES_COUNT_SHIFT           (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXJABBERFRAMES_COUNT_MAX             (0xFFFFFFFFU)

/* RXUNDERSIZEDFRAMES */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXUNDERSIZEDFRAMES_COUNT_MASK        (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXUNDERSIZEDFRAMES_COUNT_SHIFT       (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXUNDERSIZEDFRAMES_COUNT_MAX         (0xFFFFFFFFU)

/* RXFRAGMENTS */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXFRAGMENTS_COUNT_MASK               (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXFRAGMENTS_COUNT_SHIFT              (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXFRAGMENTS_COUNT_MAX                (0xFFFFFFFFU)

/* ALE_DROP */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_DROP_COUNT_MASK                  (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_DROP_COUNT_SHIFT                 (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_DROP_COUNT_MAX                   (0xFFFFFFFFU)

/* ALE_OVERRUN_DROP */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_OVERRUN_DROP_COUNT_MASK          (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_OVERRUN_DROP_COUNT_SHIFT         (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_OVERRUN_DROP_COUNT_MAX           (0xFFFFFFFFU)

/* RXOCTETS */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXOCTETS_COUNT_MASK                  (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXOCTETS_COUNT_SHIFT                 (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXOCTETS_COUNT_MAX                   (0xFFFFFFFFU)

/* TXGOODFRAMES */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXGOODFRAMES_COUNT_MASK              (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXGOODFRAMES_COUNT_SHIFT             (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXGOODFRAMES_COUNT_MAX               (0xFFFFFFFFU)

/* TXBROADCASTFRAMES */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXBROADCASTFRAMES_COUNT_MASK         (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXBROADCASTFRAMES_COUNT_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXBROADCASTFRAMES_COUNT_MAX          (0xFFFFFFFFU)

/* TXMULTICASTFRAMES */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXMULTICASTFRAMES_COUNT_MASK         (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXMULTICASTFRAMES_COUNT_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXMULTICASTFRAMES_COUNT_MAX          (0xFFFFFFFFU)

/* TXPAUSEFRAMES */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXPAUSEFRAMES_COUNT_MASK             (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXPAUSEFRAMES_COUNT_SHIFT            (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXPAUSEFRAMES_COUNT_MAX              (0xFFFFFFFFU)

/* TXDEFERREDFRAMES */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXDEFERREDFRAMES_COUNT_MASK          (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXDEFERREDFRAMES_COUNT_SHIFT         (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXDEFERREDFRAMES_COUNT_MAX           (0xFFFFFFFFU)

/* TXCOLLISIONFRAMES */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXCOLLISIONFRAMES_COUNT_MASK         (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXCOLLISIONFRAMES_COUNT_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXCOLLISIONFRAMES_COUNT_MAX          (0xFFFFFFFFU)

/* TXSINGLECOLLFRAMES */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXSINGLECOLLFRAMES_COUNT_MASK        (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXSINGLECOLLFRAMES_COUNT_SHIFT       (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXSINGLECOLLFRAMES_COUNT_MAX         (0xFFFFFFFFU)

/* TXMULTCOLLFRAMES */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXMULTCOLLFRAMES_COUNT_MASK          (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXMULTCOLLFRAMES_COUNT_SHIFT         (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXMULTCOLLFRAMES_COUNT_MAX           (0xFFFFFFFFU)

/* TXEXCESSIVECOLLISIONS */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXEXCESSIVECOLLISIONS_COUNT_MASK     (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXEXCESSIVECOLLISIONS_COUNT_SHIFT    (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXEXCESSIVECOLLISIONS_COUNT_MAX      (0xFFFFFFFFU)

/* TXLATECOLLISIONS */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXLATECOLLISIONS_COUNT_MASK          (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXLATECOLLISIONS_COUNT_SHIFT         (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXLATECOLLISIONS_COUNT_MAX           (0xFFFFFFFFU)

/* RXIPGERROR */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXIPGERROR_COUNT_MASK                (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXIPGERROR_COUNT_SHIFT               (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RXIPGERROR_COUNT_MAX                 (0xFFFFFFFFU)

/* TXCARRIERSENSEERRORS */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXCARRIERSENSEERRORS_COUNT_MASK      (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXCARRIERSENSEERRORS_COUNT_SHIFT     (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXCARRIERSENSEERRORS_COUNT_MAX       (0xFFFFFFFFU)

/* TXOCTETS */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXOCTETS_COUNT_MASK                  (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXOCTETS_COUNT_SHIFT                 (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TXOCTETS_COUNT_MAX                   (0xFFFFFFFFU)

/* OCTETFRAMES64 */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_OCTETFRAMES64_COUNT_MASK             (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_OCTETFRAMES64_COUNT_SHIFT            (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_OCTETFRAMES64_COUNT_MAX              (0xFFFFFFFFU)

/* OCTETFRAMES65T127 */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_OCTETFRAMES65T127_COUNT_MASK         (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_OCTETFRAMES65T127_COUNT_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_OCTETFRAMES65T127_COUNT_MAX          (0xFFFFFFFFU)

/* OCTETFRAMES128T255 */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_OCTETFRAMES128T255_COUNT_MASK        (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_OCTETFRAMES128T255_COUNT_SHIFT       (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_OCTETFRAMES128T255_COUNT_MAX         (0xFFFFFFFFU)

/* OCTETFRAMES256T511 */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_OCTETFRAMES256T511_COUNT_MASK        (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_OCTETFRAMES256T511_COUNT_SHIFT       (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_OCTETFRAMES256T511_COUNT_MAX         (0xFFFFFFFFU)

/* OCTETFRAMES512T1023 */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_OCTETFRAMES512T1023_COUNT_MASK       (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_OCTETFRAMES512T1023_COUNT_SHIFT      (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_OCTETFRAMES512T1023_COUNT_MAX        (0xFFFFFFFFU)

/* OCTETFRAMES1024TUP */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_OCTETFRAMES1024TUP_COUNT_MASK        (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_OCTETFRAMES1024TUP_COUNT_SHIFT       (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_OCTETFRAMES1024TUP_COUNT_MAX         (0xFFFFFFFFU)

/* NETOCTETS */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_NETOCTETS_COUNT_MASK                 (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_NETOCTETS_COUNT_SHIFT                (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_NETOCTETS_COUNT_MAX                  (0xFFFFFFFFU)

/* RX_BOTTOM_OF_FIFO_DROP */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RX_BOTTOM_OF_FIFO_DROP_COUNT_MASK    (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RX_BOTTOM_OF_FIFO_DROP_COUNT_SHIFT   (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RX_BOTTOM_OF_FIFO_DROP_COUNT_MAX     (0xFFFFFFFFU)

/* PORTMASK_DROP */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_PORTMASK_DROP_COUNT_MASK             (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_PORTMASK_DROP_COUNT_SHIFT            (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_PORTMASK_DROP_COUNT_MAX              (0xFFFFFFFFU)

/* RX_TOP_OF_FIFO_DROP */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RX_TOP_OF_FIFO_DROP_COUNT_MASK       (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RX_TOP_OF_FIFO_DROP_COUNT_SHIFT      (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_RX_TOP_OF_FIFO_DROP_COUNT_MAX        (0xFFFFFFFFU)

/* ALE_RATE_LIMIT_DROP */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_RATE_LIMIT_DROP_COUNT_MASK       (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_RATE_LIMIT_DROP_COUNT_SHIFT      (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_RATE_LIMIT_DROP_COUNT_MAX        (0xFFFFFFFFU)

/* ALE_VID_INGRESS_DROP */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_VID_INGRESS_DROP_COUNT_MASK      (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_VID_INGRESS_DROP_COUNT_SHIFT     (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_VID_INGRESS_DROP_COUNT_MAX       (0xFFFFFFFFU)

/* ALE_DA_EQ_SA_DROP */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_DA_EQ_SA_DROP_COUNT_MASK         (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_DA_EQ_SA_DROP_COUNT_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_DA_EQ_SA_DROP_COUNT_MAX          (0xFFFFFFFFU)

/* ALE_BLOCK_DROP */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_BLOCK_DROP_COUNT_MASK            (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_BLOCK_DROP_COUNT_SHIFT           (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_BLOCK_DROP_COUNT_MAX             (0xFFFFFFFFU)

/* ALE_SECURE_DROP */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_SECURE_DROP_COUNT_MASK           (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_SECURE_DROP_COUNT_SHIFT          (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_SECURE_DROP_COUNT_MAX            (0xFFFFFFFFU)

/* ALE_AUTH_DROP */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_AUTH_DROP_COUNT_MASK             (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_AUTH_DROP_COUNT_SHIFT            (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_AUTH_DROP_COUNT_MAX              (0xFFFFFFFFU)

/* ALE_UNKN_UNI */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_UNKN_UNI_COUNT_MASK              (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_UNKN_UNI_COUNT_SHIFT             (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_UNKN_UNI_COUNT_MAX               (0xFFFFFFFFU)

/* ALE_UNKN_UNI_BCNT */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_UNKN_UNI_BCNT_COUNT_MASK         (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_UNKN_UNI_BCNT_COUNT_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_UNKN_UNI_BCNT_COUNT_MAX          (0xFFFFFFFFU)

/* ALE_UNKN_MLT */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_UNKN_MLT_COUNT_MASK              (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_UNKN_MLT_COUNT_SHIFT             (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_UNKN_MLT_COUNT_MAX               (0xFFFFFFFFU)

/* ALE_UNKN_MLT_BCNT */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_UNKN_MLT_BCNT_COUNT_MASK         (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_UNKN_MLT_BCNT_COUNT_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_UNKN_MLT_BCNT_COUNT_MAX          (0xFFFFFFFFU)

/* ALE_UNKN_BRD */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_UNKN_BRD_COUNT_MASK              (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_UNKN_BRD_COUNT_SHIFT             (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_UNKN_BRD_COUNT_MAX               (0xFFFFFFFFU)

/* ALE_UNKN_BRD_BCNT */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_UNKN_BRD_BCNT_COUNT_MASK         (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_UNKN_BRD_BCNT_COUNT_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_UNKN_BRD_BCNT_COUNT_MAX          (0xFFFFFFFFU)

/* ALE_POL_MATCH */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_POL_MATCH_COUNT_MASK             (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_POL_MATCH_COUNT_SHIFT            (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_POL_MATCH_COUNT_MAX              (0xFFFFFFFFU)

/* ALE_POL_MATCH_RED */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_POL_MATCH_RED_COUNT_MASK         (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_POL_MATCH_RED_COUNT_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_POL_MATCH_RED_COUNT_MAX          (0xFFFFFFFFU)

/* ALE_POL_MATCH_YELLOW */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_POL_MATCH_YELLOW_COUNT_MASK      (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_POL_MATCH_YELLOW_COUNT_SHIFT     (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ALE_POL_MATCH_YELLOW_COUNT_MAX       (0xFFFFFFFFU)

/* TX_MEMORY_PROTECT_ERROR */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TX_MEMORY_PROTECT_ERROR_COUNT_MASK   (0x000000FFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TX_MEMORY_PROTECT_ERROR_COUNT_SHIFT  (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_TX_MEMORY_PROTECT_ERROR_COUNT_MAX    (0x000000FFU)

/* ENET_PN_TX_PRI_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ENET_PN_TX_PRI_REG_PN_TX_PRIN_MASK   (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ENET_PN_TX_PRI_REG_PN_TX_PRIN_SHIFT  (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ENET_PN_TX_PRI_REG_PN_TX_PRIN_MAX    (0xFFFFFFFFU)

/* ENET_PN_TX_PRI_BCNT_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ENET_PN_TX_PRI_BCNT_REG_PN_TX_PRIN_BCNT_MASK (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ENET_PN_TX_PRI_BCNT_REG_PN_TX_PRIN_BCNT_SHIFT (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ENET_PN_TX_PRI_BCNT_REG_PN_TX_PRIN_BCNT_MAX (0xFFFFFFFFU)

/* ENET_PN_TX_PRI_DROP_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ENET_PN_TX_PRI_DROP_REG_PN_TX_PRIN_DROP_MASK (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ENET_PN_TX_PRI_DROP_REG_PN_TX_PRIN_DROP_SHIFT (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ENET_PN_TX_PRI_DROP_REG_PN_TX_PRIN_DROP_MAX (0xFFFFFFFFU)

/* ENET_PN_TX_PRI_DROP_BCNT_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ENET_PN_TX_PRI_DROP_BCNT_REG_PN_TX_PRIN_DROP_BCNT_MASK (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ENET_PN_TX_PRI_DROP_BCNT_REG_PN_TX_PRIN_DROP_BCNT_SHIFT (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_NU_STAT_1_ENET_PN_TX_PRI_DROP_BCNT_REG_PN_TX_PRIN_DROP_BCNT_MAX (0xFFFFFFFFU)

/* COMP_LOW_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_COMP_LOW_REG_COMP_LOW_MASK             (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_COMP_LOW_REG_COMP_LOW_SHIFT            (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_COMP_LOW_REG_COMP_LOW_MAX              (0xFFFFFFFFU)

/* COMP_HIGH_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_COMP_HIGH_REG_COMP_HIGH_MASK           (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_COMP_HIGH_REG_COMP_HIGH_SHIFT          (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_COMP_HIGH_REG_COMP_HIGH_MAX            (0xFFFFFFFFU)

/* CONTROL_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_CONTROL_REG_PPM_DIR_MASK               (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_CONTROL_REG_PPM_DIR_SHIFT              (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_CONTROL_REG_PPM_DIR_MAX                (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_CONTROL_REG_POLARITY_INV_MASK          (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_CONTROL_REG_POLARITY_INV_SHIFT         (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_CONTROL_REG_POLARITY_INV_MAX           (0x00000001U)

/* LENGTH_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_LENGTH_REG_LENGTH_MASK                 (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_LENGTH_REG_LENGTH_SHIFT                (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_LENGTH_REG_LENGTH_MAX                  (0xFFFFFFFFU)

/* PPM_LOW_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_PPM_LOW_REG_PPM_LOW_MASK               (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_PPM_LOW_REG_PPM_LOW_SHIFT              (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_PPM_LOW_REG_PPM_LOW_MAX                (0xFFFFFFFFU)

/* PPM_HIGH_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_PPM_HIGH_REG_PPM_HIGH_MASK             (0x000003FFU)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_PPM_HIGH_REG_PPM_HIGH_SHIFT            (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_PPM_HIGH_REG_PPM_HIGH_MAX              (0x000003FFU)

/* NUDGE_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_NUDGE_REG_NUDGE_MASK                   (0x000000FFU)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_NUDGE_REG_NUDGE_SHIFT                  (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_GENF_NUDGE_REG_NUDGE_MAX                    (0x000000FFU)

/* COMP_LOW_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_COMP_LOW_REG_COMP_LOW_MASK             (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_COMP_LOW_REG_COMP_LOW_SHIFT            (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_COMP_LOW_REG_COMP_LOW_MAX              (0xFFFFFFFFU)

/* COMP_HIGH_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_COMP_HIGH_REG_COMP_HIGH_MASK           (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_COMP_HIGH_REG_COMP_HIGH_SHIFT          (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_COMP_HIGH_REG_COMP_HIGH_MAX            (0xFFFFFFFFU)

/* CONTROL_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_CONTROL_REG_PPM_DIR_MASK               (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_CONTROL_REG_PPM_DIR_SHIFT              (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_CONTROL_REG_PPM_DIR_MAX                (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_CONTROL_REG_POLARITY_INV_MASK          (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_CONTROL_REG_POLARITY_INV_SHIFT         (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_CONTROL_REG_POLARITY_INV_MAX           (0x00000001U)

/* LENGTH_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_LENGTH_REG_LENGTH_MASK                 (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_LENGTH_REG_LENGTH_SHIFT                (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_LENGTH_REG_LENGTH_MAX                  (0xFFFFFFFFU)

/* PPM_LOW_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_PPM_LOW_REG_PPM_LOW_MASK               (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_PPM_LOW_REG_PPM_LOW_SHIFT              (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_PPM_LOW_REG_PPM_LOW_MAX                (0xFFFFFFFFU)

/* PPM_HIGH_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_PPM_HIGH_REG_PPM_HIGH_MASK             (0x000003FFU)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_PPM_HIGH_REG_PPM_HIGH_SHIFT            (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_PPM_HIGH_REG_PPM_HIGH_MAX              (0x000003FFU)

/* NUDGE_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_NUDGE_REG_NUDGE_MASK                   (0x000000FFU)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_NUDGE_REG_NUDGE_SHIFT                  (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ESTF_NUDGE_REG_NUDGE_MAX                    (0x000000FFU)

/* IDVER_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_IDVER_REG_MINOR_VER_MASK                       (0x000000FFU)
#define CSL_CPSW2G_CPSW_NU_CPTS_IDVER_REG_MINOR_VER_SHIFT                      (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_IDVER_REG_MINOR_VER_MAX                        (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_CPTS_IDVER_REG_MAJOR_VER_MASK                       (0x00000700U)
#define CSL_CPSW2G_CPSW_NU_CPTS_IDVER_REG_MAJOR_VER_SHIFT                      (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPTS_IDVER_REG_MAJOR_VER_MAX                        (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPTS_IDVER_REG_RTL_VER_MASK                         (0x0000F800U)
#define CSL_CPSW2G_CPSW_NU_CPTS_IDVER_REG_RTL_VER_SHIFT                        (0x0000000BU)
#define CSL_CPSW2G_CPSW_NU_CPTS_IDVER_REG_RTL_VER_MAX                          (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPTS_IDVER_REG_TX_IDENT_MASK                        (0xFFFF0000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_IDVER_REG_TX_IDENT_SHIFT                       (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPTS_IDVER_REG_TX_IDENT_MAX                         (0x0000FFFFU)

/* CONTROL_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_CPTS_EN_MASK                       (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_CPTS_EN_SHIFT                      (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_CPTS_EN_MAX                        (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_INT_TEST_MASK                      (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_INT_TEST_SHIFT                     (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_INT_TEST_MAX                       (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_TS_COMP_POLARITY_MASK              (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_TS_COMP_POLARITY_SHIFT             (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_TS_COMP_POLARITY_MAX               (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_TSTAMP_EN_MASK                     (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_TSTAMP_EN_SHIFT                    (0x00000003U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_TSTAMP_EN_MAX                      (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_SEQUENCE_EN_MASK                   (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_SEQUENCE_EN_SHIFT                  (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_SEQUENCE_EN_MAX                    (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_MODE_MASK                          (0x00000020U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_MODE_SHIFT                         (0x00000005U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_MODE_MAX                           (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_TS_COMP_TOG_MASK                   (0x00000040U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_TS_COMP_TOG_SHIFT                  (0x00000006U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_TS_COMP_TOG_MAX                    (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_TS_PPM_DIR_MASK                    (0x00000080U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_TS_PPM_DIR_SHIFT                   (0x00000007U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_TS_PPM_DIR_MAX                     (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_HW1_TS_PUSH_EN_MASK                (0x00000100U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_HW1_TS_PUSH_EN_SHIFT               (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_HW1_TS_PUSH_EN_MAX                 (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_HW2_TS_PUSH_EN_MASK                (0x00000200U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_HW2_TS_PUSH_EN_SHIFT               (0x00000009U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_HW2_TS_PUSH_EN_MAX                 (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_HW3_TS_PUSH_EN_MASK                (0x00000400U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_HW3_TS_PUSH_EN_SHIFT               (0x0000000AU)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_HW3_TS_PUSH_EN_MAX                 (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_HW4_TS_PUSH_EN_MASK                (0x00000800U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_HW4_TS_PUSH_EN_SHIFT               (0x0000000BU)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_HW4_TS_PUSH_EN_MAX                 (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_HW5_TS_PUSH_EN_MASK                (0x00001000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_HW5_TS_PUSH_EN_SHIFT               (0x0000000CU)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_HW5_TS_PUSH_EN_MAX                 (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_HW6_TS_PUSH_EN_MASK                (0x00002000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_HW6_TS_PUSH_EN_SHIFT               (0x0000000DU)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_HW6_TS_PUSH_EN_MAX                 (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_HW7_TS_PUSH_EN_MASK                (0x00004000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_HW7_TS_PUSH_EN_SHIFT               (0x0000000EU)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_HW7_TS_PUSH_EN_MAX                 (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_HW8_TS_PUSH_EN_MASK                (0x00008000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_HW8_TS_PUSH_EN_SHIFT               (0x0000000FU)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_HW8_TS_PUSH_EN_MAX                 (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_TS_SYNC_SEL_MASK                   (0xF0000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_TS_SYNC_SEL_SHIFT                  (0x0000001CU)
#define CSL_CPSW2G_CPSW_NU_CPTS_CONTROL_REG_TS_SYNC_SEL_MAX                    (0x0000000FU)

/* RFTCLK_SEL_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_RFTCLK_SEL_REG_RFTCLK_SEL_MASK                 (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_CPTS_RFTCLK_SEL_REG_RFTCLK_SEL_SHIFT                (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_RFTCLK_SEL_REG_RFTCLK_SEL_MAX                  (0x0000001FU)

/* TS_PUSH_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_PUSH_REG_TS_PUSH_MASK                       (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_PUSH_REG_TS_PUSH_SHIFT                      (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_PUSH_REG_TS_PUSH_MAX                        (0x00000001U)

/* TS_LOAD_VAL_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_LOAD_VAL_REG_TS_LOAD_VAL_MASK               (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_LOAD_VAL_REG_TS_LOAD_VAL_SHIFT              (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_LOAD_VAL_REG_TS_LOAD_VAL_MAX                (0xFFFFFFFFU)

/* TS_LOAD_EN_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_LOAD_EN_REG_TS_LOAD_EN_MASK                 (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_LOAD_EN_REG_TS_LOAD_EN_SHIFT                (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_LOAD_EN_REG_TS_LOAD_EN_MAX                  (0x00000001U)

/* TS_COMP_VAL_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_COMP_VAL_REG_TS_COMP_VAL_MASK               (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_COMP_VAL_REG_TS_COMP_VAL_SHIFT              (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_COMP_VAL_REG_TS_COMP_VAL_MAX                (0xFFFFFFFFU)

/* TS_COMP_LEN_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_COMP_LEN_REG_TS_COMP_LENGTH_MASK            (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_COMP_LEN_REG_TS_COMP_LENGTH_SHIFT           (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_COMP_LEN_REG_TS_COMP_LENGTH_MAX             (0xFFFFFFFFU)

/* INTSTAT_RAW_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_INTSTAT_RAW_REG_TS_PEND_RAW_MASK               (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPTS_INTSTAT_RAW_REG_TS_PEND_RAW_SHIFT              (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_INTSTAT_RAW_REG_TS_PEND_RAW_MAX                (0x00000001U)

/* INTSTAT_MASKED_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_INTSTAT_MASKED_REG_TS_PEND_MASK                (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPTS_INTSTAT_MASKED_REG_TS_PEND_SHIFT               (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_INTSTAT_MASKED_REG_TS_PEND_MAX                 (0x00000001U)

/* INT_ENABLE_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_INT_ENABLE_REG_TS_PEND_EN_MASK                 (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPTS_INT_ENABLE_REG_TS_PEND_EN_SHIFT                (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_INT_ENABLE_REG_TS_PEND_EN_MAX                  (0x00000001U)

/* TS_COMP_NUDGE_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_COMP_NUDGE_REG_NUDGE_MASK                   (0x000000FFU)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_COMP_NUDGE_REG_NUDGE_SHIFT                  (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_COMP_NUDGE_REG_NUDGE_MAX                    (0x000000FFU)

/* EVENT_POP_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_POP_REG_EVENT_POP_MASK                   (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_POP_REG_EVENT_POP_SHIFT                  (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_POP_REG_EVENT_POP_MAX                    (0x00000001U)

/* EVENT_0_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_0_REG_TIME_STAMP_MASK                    (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_0_REG_TIME_STAMP_SHIFT                   (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_0_REG_TIME_STAMP_MAX                     (0xFFFFFFFFU)

/* EVENT_1_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_1_REG_SEQUENCE_ID_MASK                   (0x0000FFFFU)
#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_1_REG_SEQUENCE_ID_SHIFT                  (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_1_REG_SEQUENCE_ID_MAX                    (0x0000FFFFU)

#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_1_REG_MESSAGE_TYPE_MASK                  (0x000F0000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_1_REG_MESSAGE_TYPE_SHIFT                 (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_1_REG_MESSAGE_TYPE_MAX                   (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_1_REG_EVENT_TYPE_MASK                    (0x00F00000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_1_REG_EVENT_TYPE_SHIFT                   (0x00000014U)
#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_1_REG_EVENT_TYPE_MAX                     (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_1_REG_PORT_NUMBER_MASK                   (0x1F000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_1_REG_PORT_NUMBER_SHIFT                  (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_1_REG_PORT_NUMBER_MAX                    (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_1_REG_PREMPT_QUEUE_MASK                  (0x20000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_1_REG_PREMPT_QUEUE_SHIFT                 (0x0000001DU)
#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_1_REG_PREMPT_QUEUE_MAX                   (0x00000001U)

/* EVENT_2_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_2_REG_DOMAIN_MASK                        (0x000000FFU)
#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_2_REG_DOMAIN_SHIFT                       (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_2_REG_DOMAIN_MAX                         (0x000000FFU)

/* EVENT_3_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_3_REG_TIME_STAMP_MASK                    (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_3_REG_TIME_STAMP_SHIFT                   (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_EVENT_3_REG_TIME_STAMP_MAX                     (0xFFFFFFFFU)

/* TS_LOAD_HIGH_VAL_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_LOAD_HIGH_VAL_REG_TS_LOAD_VAL_MASK          (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_LOAD_HIGH_VAL_REG_TS_LOAD_VAL_SHIFT         (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_LOAD_HIGH_VAL_REG_TS_LOAD_VAL_MAX           (0xFFFFFFFFU)

/* TS_COMP_HIGH_VAL_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_COMP_HIGH_VAL_REG_TS_COMP_HIGH_VAL_MASK     (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_COMP_HIGH_VAL_REG_TS_COMP_HIGH_VAL_SHIFT    (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_COMP_HIGH_VAL_REG_TS_COMP_HIGH_VAL_MAX      (0xFFFFFFFFU)

/* TS_ADD_VAL_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ADD_VAL_REG_ADD_VAL_MASK                    (0x00000007U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ADD_VAL_REG_ADD_VAL_SHIFT                   (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_ADD_VAL_REG_ADD_VAL_MAX                     (0x00000007U)

/* TS_PPM_LOW_VAL_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_PPM_LOW_VAL_REG_TS_PPM_LOW_VAL_MASK         (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_PPM_LOW_VAL_REG_TS_PPM_LOW_VAL_SHIFT        (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_PPM_LOW_VAL_REG_TS_PPM_LOW_VAL_MAX          (0xFFFFFFFFU)

/* TS_PPM_HIGH_VAL_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_PPM_HIGH_VAL_REG_TS_PPM_HIGH_VAL_MASK       (0x000003FFU)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_PPM_HIGH_VAL_REG_TS_PPM_HIGH_VAL_SHIFT      (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_PPM_HIGH_VAL_REG_TS_PPM_HIGH_VAL_MAX        (0x000003FFU)

/* TS_NUDGE_VAL_REG */

#define CSL_CPSW2G_CPSW_NU_CPTS_TS_NUDGE_VAL_REG_TS_NUDGE_VAL_MASK             (0x000000FFU)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_NUDGE_VAL_REG_TS_NUDGE_VAL_SHIFT            (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPTS_TS_NUDGE_VAL_REG_TS_NUDGE_VAL_MAX              (0x000000FFU)

/* IDVER_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_IDVER_REG_MINOR_VER_MASK                        (0x000000FFU)
#define CSL_CPSW2G_CPSW_NU_ALE_IDVER_REG_MINOR_VER_SHIFT                       (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_IDVER_REG_MINOR_VER_MAX                         (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_ALE_IDVER_REG_MAJOR_VER_MASK                        (0x00000700U)
#define CSL_CPSW2G_CPSW_NU_ALE_IDVER_REG_MAJOR_VER_SHIFT                       (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_ALE_IDVER_REG_MAJOR_VER_MAX                         (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_ALE_IDVER_REG_RTL_VER_MASK                          (0x0000F800U)
#define CSL_CPSW2G_CPSW_NU_ALE_IDVER_REG_RTL_VER_SHIFT                         (0x0000000BU)
#define CSL_CPSW2G_CPSW_NU_ALE_IDVER_REG_RTL_VER_MAX                           (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_ALE_IDVER_REG_IDENT_MASK                            (0xFFFF0000U)
#define CSL_CPSW2G_CPSW_NU_ALE_IDVER_REG_IDENT_SHIFT                           (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_ALE_IDVER_REG_IDENT_MAX                             (0x0000FFFFU)

/* STATUS_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_STATUS_REG_ENTRIES_DIV_1024_MASK                (0x000000FFU)
#define CSL_CPSW2G_CPSW_NU_ALE_STATUS_REG_ENTRIES_DIV_1024_SHIFT               (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_STATUS_REG_ENTRIES_DIV_1024_MAX                 (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_ALE_STATUS_REG_POLICERS_DIV_8_MASK                  (0x0000FF00U)
#define CSL_CPSW2G_CPSW_NU_ALE_STATUS_REG_POLICERS_DIV_8_SHIFT                 (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_ALE_STATUS_REG_POLICERS_DIV_8_MAX                   (0x000000FFU)

/* CONTROL_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_ENABLE_RATE_LIMIT_MASK              (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_ENABLE_RATE_LIMIT_SHIFT             (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_ENABLE_RATE_LIMIT_MAX               (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_ENABLE_AUTH_MODE_MASK               (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_ENABLE_AUTH_MODE_SHIFT              (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_ENABLE_AUTH_MODE_MAX                (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_VLAN_AWARE_MASK                     (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_VLAN_AWARE_SHIFT                    (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_VLAN_AWARE_MAX                      (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_RATE_LIMIT_TX_MASK                  (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_RATE_LIMIT_TX_SHIFT                 (0x00000003U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_RATE_LIMIT_TX_MAX                   (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_BYPASS_MASK                         (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_BYPASS_SHIFT                        (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_BYPASS_MAX                          (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_ENABLE_OUI_DENY_MASK                (0x00000020U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_ENABLE_OUI_DENY_SHIFT               (0x00000005U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_ENABLE_OUI_DENY_MAX                 (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_EN_VID0_MODE_MASK                   (0x00000040U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_EN_VID0_MODE_SHIFT                  (0x00000006U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_EN_VID0_MODE_MAX                    (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_LEARN_NO_VID_MASK                   (0x00000080U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_LEARN_NO_VID_SHIFT                  (0x00000007U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_LEARN_NO_VID_MAX                    (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_UNI_FLOOD_TO_HOST_MASK              (0x00000100U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_UNI_FLOOD_TO_HOST_SHIFT             (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_UNI_FLOOD_TO_HOST_MAX               (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_MIRROR_SEN_MASK                     (0x00000400U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_MIRROR_SEN_SHIFT                    (0x0000000AU)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_MIRROR_SEN_MAX                      (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_MIRROR_DEN_MASK                     (0x00000800U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_MIRROR_DEN_SHIFT                    (0x0000000BU)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_MIRROR_DEN_MAX                      (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_MIRROR_MEN_MASK                     (0x00001000U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_MIRROR_MEN_SHIFT                    (0x0000000CU)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_MIRROR_MEN_MAX                      (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_UVLAN_NO_LEARN_MASK                 (0x00002000U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_UVLAN_NO_LEARN_SHIFT                (0x0000000DU)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_UVLAN_NO_LEARN_MAX                  (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_REG_UPD_STATIC_MASK                 (0x00008000U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_REG_UPD_STATIC_SHIFT                (0x0000000FU)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_REG_UPD_STATIC_MAX                  (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_MIRROR_TOP_MASK                     (0x00010000U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_MIRROR_TOP_SHIFT                    (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_MIRROR_TOP_MAX                      (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_UPD_BW_CTL_MASK                     (0x00E00000U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_UPD_BW_CTL_SHIFT                    (0x00000015U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_UPD_BW_CTL_MAX                      (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_MIRROR_DP_MASK                      (0x01000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_MIRROR_DP_SHIFT                     (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_MIRROR_DP_MAX                       (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_AGE_OUT_NOW_MASK                    (0x20000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_AGE_OUT_NOW_SHIFT                   (0x0000001DU)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_AGE_OUT_NOW_MAX                     (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_CLEAR_TABLE_MASK                    (0x40000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_CLEAR_TABLE_SHIFT                   (0x0000001EU)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_CLEAR_TABLE_MAX                     (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_ENABLE_MASK                         (0x80000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_ENABLE_SHIFT                        (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL_REG_ENABLE_MAX                          (0x00000001U)

/* CONTROL2_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL2_REG_TRK_EN_DST_MASK                    (0x80000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL2_REG_TRK_EN_DST_SHIFT                   (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL2_REG_TRK_EN_DST_MAX                     (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL2_REG_TRK_EN_SRC_MASK                    (0x40000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL2_REG_TRK_EN_SRC_SHIFT                   (0x0000001EU)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL2_REG_TRK_EN_SRC_MAX                     (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL2_REG_TRK_EN_PRI_MASK                    (0x20000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL2_REG_TRK_EN_PRI_SHIFT                   (0x0000001DU)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL2_REG_TRK_EN_PRI_MAX                     (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL2_REG_TRK_EN_VLAN_MASK                   (0x08000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL2_REG_TRK_EN_VLAN_SHIFT                  (0x0000001BU)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL2_REG_TRK_EN_VLAN_MAX                    (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL2_REG_TRK_EN_SIP_MASK                    (0x02000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL2_REG_TRK_EN_SIP_SHIFT                   (0x00000019U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL2_REG_TRK_EN_SIP_MAX                     (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL2_REG_TRK_EN_DIP_MASK                    (0x01000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL2_REG_TRK_EN_DIP_SHIFT                   (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL2_REG_TRK_EN_DIP_MAX                     (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL2_REG_TRK_BASE_MASK                      (0x00070000U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL2_REG_TRK_BASE_SHIFT                     (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL2_REG_TRK_BASE_MAX                       (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL2_REG_MIRROR_MIDX_MASK                   (0x0000003FU)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL2_REG_MIRROR_MIDX_SHIFT                  (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_CONTROL2_REG_MIRROR_MIDX_MAX                    (0x0000003FU)

/* PRESCALE_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_PRESCALE_REG_PRESCALE_MASK                      (0x000FFFFFU)
#define CSL_CPSW2G_CPSW_NU_ALE_PRESCALE_REG_PRESCALE_SHIFT                     (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_PRESCALE_REG_PRESCALE_MAX                       (0x000FFFFFU)

/* AGING_TIMER_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_AGING_TIMER_REG_AGING_TIMER_MASK                (0x00FFFFFFU)
#define CSL_CPSW2G_CPSW_NU_ALE_AGING_TIMER_REG_AGING_TIMER_SHIFT               (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_AGING_TIMER_REG_AGING_TIMER_MAX                 (0x00FFFFFFU)

#define CSL_CPSW2G_CPSW_NU_ALE_AGING_TIMER_REG_PRESCALE_2_DISABLE_MASK         (0x40000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_AGING_TIMER_REG_PRESCALE_2_DISABLE_SHIFT        (0x0000001EU)
#define CSL_CPSW2G_CPSW_NU_ALE_AGING_TIMER_REG_PRESCALE_2_DISABLE_MAX          (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_AGING_TIMER_REG_PRESCALE_1_DISABLE_MASK         (0x80000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_AGING_TIMER_REG_PRESCALE_1_DISABLE_SHIFT        (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_ALE_AGING_TIMER_REG_PRESCALE_1_DISABLE_MAX          (0x00000001U)

/* TABLE_CONTROL_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_TABLE_CONTROL_REG_ENTRY_POINTER_MASK            (0x0000003FU)
#define CSL_CPSW2G_CPSW_NU_ALE_TABLE_CONTROL_REG_ENTRY_POINTER_SHIFT           (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_TABLE_CONTROL_REG_ENTRY_POINTER_MAX             (0x0000003FU)

#define CSL_CPSW2G_CPSW_NU_ALE_TABLE_CONTROL_REG_WRITE_RDZ_MASK                (0x80000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_TABLE_CONTROL_REG_WRITE_RDZ_SHIFT               (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_ALE_TABLE_CONTROL_REG_WRITE_RDZ_MAX                 (0x00000001U)

/* TABLE_WORD2_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_TABLE_WORD2_REG_ENTRY_71_64_MASK                (0x0000007FU)
#define CSL_CPSW2G_CPSW_NU_ALE_TABLE_WORD2_REG_ENTRY_71_64_SHIFT               (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_TABLE_WORD2_REG_ENTRY_71_64_MAX                 (0x0000007FU)

/* TABLE_WORD1_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_TABLE_WORD1_REG_ENTRY_63_32_MASK                (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_ALE_TABLE_WORD1_REG_ENTRY_63_32_SHIFT               (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_TABLE_WORD1_REG_ENTRY_63_32_MAX                 (0xFFFFFFFFU)

/* TABLE_WORD0_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_TABLE_WORD0_REG_ENTRY_31_0_MASK                 (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_ALE_TABLE_WORD0_REG_ENTRY_31_0_SHIFT                (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_TABLE_WORD0_REG_ENTRY_31_0_MAX                  (0xFFFFFFFFU)

/* PORT_CONTROL_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_PORT_STATE_MASK                (0x00000003U)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_PORT_STATE_SHIFT               (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_PORT_STATE_MAX                 (0x00000003U)

#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_DROP_UNTAGGED_MASK             (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_DROP_UNTAGGED_SHIFT            (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_DROP_UNTAGGED_MAX              (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_VID_INGRESS_CHECK_MASK         (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_VID_INGRESS_CHECK_SHIFT        (0x00000003U)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_VID_INGRESS_CHECK_MAX          (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_NO_LEARN_MASK                  (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_NO_LEARN_SHIFT                 (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_NO_LEARN_MAX                   (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_NO_SA_UPDATE_MASK              (0x00000020U)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_NO_SA_UPDATE_SHIFT             (0x00000005U)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_NO_SA_UPDATE_MAX               (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_MIRROR_SP_MASK                 (0x00000080U)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_MIRROR_SP_SHIFT                (0x00000007U)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_MIRROR_SP_MAX                  (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_TRUNK_NUMBER_MASK              (0x00000300U)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_TRUNK_NUMBER_SHIFT             (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_TRUNK_NUMBER_MAX               (0x00000003U)

#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_TRUNK_EN_MASK                  (0x00000400U)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_TRUNK_EN_SHIFT                 (0x0000000AU)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_TRUNK_EN_MAX                   (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_MAC_ONLY_MASK                  (0x00000800U)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_MAC_ONLY_SHIFT                 (0x0000000BU)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_MAC_ONLY_MAX                   (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_DISABLE_AUTH_MODE_MASK         (0x00001000U)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_DISABLE_AUTH_MODE_SHIFT        (0x0000000CU)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_DISABLE_AUTH_MODE_MAX          (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_MAC_ONLY_CAF_MASK              (0x00002000U)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_MAC_ONLY_CAF_SHIFT             (0x0000000DU)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_MAC_ONLY_CAF_MAX               (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_DROP_DUAL_VLAN_MASK            (0x00004000U)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_DROP_DUAL_VLAN_SHIFT           (0x0000000EU)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_DROP_DUAL_VLAN_MAX             (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_DROP_DBL_VLAN_MASK             (0x00008000U)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_DROP_DBL_VLAN_SHIFT            (0x0000000FU)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_DROP_DBL_VLAN_MAX              (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_MCAST_LIMIT_MASK               (0x00FF0000U)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_MCAST_LIMIT_SHIFT              (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_MCAST_LIMIT_MAX                (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_BCAST_LIMIT_MASK               (0xFF000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_BCAST_LIMIT_SHIFT              (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_ALE_PORT_CONTROL_REG_BCAST_LIMIT_MAX                (0x000000FFU)

/* UNKNOWN_VLAN_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_UNKNOWN_VLAN_REG_LIST_MASK                      (0x00000003U)
#define CSL_CPSW2G_CPSW_NU_ALE_UNKNOWN_VLAN_REG_LIST_SHIFT                     (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_UNKNOWN_VLAN_REG_LIST_MAX                       (0x00000003U)

/* UNKNOWN_MCAST_FLOOD_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_UNKNOWN_MCAST_FLOOD_REG_MASK_MASK               (0x00000003U)
#define CSL_CPSW2G_CPSW_NU_ALE_UNKNOWN_MCAST_FLOOD_REG_MASK_SHIFT              (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_UNKNOWN_MCAST_FLOOD_REG_MASK_MAX                (0x00000003U)

/* UNKNOWN_REG_MCAST_FLOOD_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_UNKNOWN_REG_MCAST_FLOOD_REG_MASK_MASK           (0x00000003U)
#define CSL_CPSW2G_CPSW_NU_ALE_UNKNOWN_REG_MCAST_FLOOD_REG_MASK_SHIFT          (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_UNKNOWN_REG_MCAST_FLOOD_REG_MASK_MAX            (0x00000003U)

/* FORCE_UNTAGGED_EGRESS_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_FORCE_UNTAGGED_EGRESS_REG_MASK_MASK             (0x00000003U)
#define CSL_CPSW2G_CPSW_NU_ALE_FORCE_UNTAGGED_EGRESS_REG_MASK_SHIFT            (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_FORCE_UNTAGGED_EGRESS_REG_MASK_MAX              (0x00000003U)

/* VLAN_MASK_MUX0_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX0_REG_MASK_MASK                    (0x00000003U)
#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX0_REG_MASK_SHIFT                   (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX0_REG_MASK_MAX                     (0x00000003U)

/* VLAN_MASK_MUX1_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX1_REG_MASK_MASK                    (0x00000003U)
#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX1_REG_MASK_SHIFT                   (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX1_REG_MASK_MAX                     (0x00000003U)

/* VLAN_MASK_MUX2_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX2_REG_MASK_MASK                    (0x00000003U)
#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX2_REG_MASK_SHIFT                   (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX2_REG_MASK_MAX                     (0x00000003U)

/* VLAN_MASK_MUX3_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX3_REG_MASK_MASK                    (0x00000003U)
#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX3_REG_MASK_SHIFT                   (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX3_REG_MASK_MAX                     (0x00000003U)

/* VLAN_MASK_MUX4_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX4_REG_MASK_MASK                    (0x00000003U)
#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX4_REG_MASK_SHIFT                   (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX4_REG_MASK_MAX                     (0x00000003U)

/* VLAN_MASK_MUX5_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX5_REG_MASK_MASK                    (0x00000003U)
#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX5_REG_MASK_SHIFT                   (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX5_REG_MASK_MAX                     (0x00000003U)

/* VLAN_MASK_MUX6_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX6_REG_MASK_MASK                    (0x00000003U)
#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX6_REG_MASK_SHIFT                   (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX6_REG_MASK_MAX                     (0x00000003U)

/* VLAN_MASK_MUX7_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX7_REG_MASK_MASK                    (0x00000003U)
#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX7_REG_MASK_SHIFT                   (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_VLAN_MASK_MUX7_REG_MASK_MAX                     (0x00000003U)

/* POLICER_PORT_OUI_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_PORT_OUI_REG_PORT_MEN_MASK              (0x80000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_PORT_OUI_REG_PORT_MEN_SHIFT             (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_PORT_OUI_REG_PORT_MEN_MAX               (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_PORT_OUI_REG_TRUNK_ID_MASK              (0x40000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_PORT_OUI_REG_TRUNK_ID_SHIFT             (0x0000001EU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_PORT_OUI_REG_TRUNK_ID_MAX               (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_PORT_OUI_REG_PORT_NUM_MASK              (0x02000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_PORT_OUI_REG_PORT_NUM_SHIFT             (0x00000019U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_PORT_OUI_REG_PORT_NUM_MAX               (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_PORT_OUI_REG_PRI_MEN_MASK               (0x00080000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_PORT_OUI_REG_PRI_MEN_SHIFT              (0x00000013U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_PORT_OUI_REG_PRI_MEN_MAX                (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_PORT_OUI_REG_PRI_VAL_MASK               (0x00070000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_PORT_OUI_REG_PRI_VAL_SHIFT              (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_PORT_OUI_REG_PRI_VAL_MAX                (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_PORT_OUI_REG_OUI_MEN_MASK               (0x00008000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_PORT_OUI_REG_OUI_MEN_SHIFT              (0x0000000FU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_PORT_OUI_REG_OUI_MEN_MAX                (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_PORT_OUI_REG_OUI_INDEX_MASK             (0x0000003FU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_PORT_OUI_REG_OUI_INDEX_SHIFT            (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_PORT_OUI_REG_OUI_INDEX_MAX              (0x0000003FU)

/* POLICER_DA_SA_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_DA_SA_REG_DST_MEN_MASK                  (0x80000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_DA_SA_REG_DST_MEN_SHIFT                 (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_DA_SA_REG_DST_MEN_MAX                   (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_DA_SA_REG_DST_INDEX_MASK                (0x003F0000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_DA_SA_REG_DST_INDEX_SHIFT               (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_DA_SA_REG_DST_INDEX_MAX                 (0x0000003FU)

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_DA_SA_REG_SRC_MEN_MASK                  (0x00008000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_DA_SA_REG_SRC_MEN_SHIFT                 (0x0000000FU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_DA_SA_REG_SRC_MEN_MAX                   (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_DA_SA_REG_SRC_INDEX_MASK                (0x0000003FU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_DA_SA_REG_SRC_INDEX_SHIFT               (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_DA_SA_REG_SRC_INDEX_MAX                 (0x0000003FU)

/* POLICER_VLAN_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_VLAN_REG_OVLAN_MEN_MASK                 (0x80000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_VLAN_REG_OVLAN_MEN_SHIFT                (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_VLAN_REG_OVLAN_MEN_MAX                  (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_VLAN_REG_OVLAN_INDEX_MASK               (0x003F0000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_VLAN_REG_OVLAN_INDEX_SHIFT              (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_VLAN_REG_OVLAN_INDEX_MAX                (0x0000003FU)

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_VLAN_REG_IVLAN_MEN_MASK                 (0x00008000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_VLAN_REG_IVLAN_MEN_SHIFT                (0x0000000FU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_VLAN_REG_IVLAN_MEN_MAX                  (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_VLAN_REG_IVLAN_INDEX_MASK               (0x0000003FU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_VLAN_REG_IVLAN_INDEX_SHIFT              (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_VLAN_REG_IVLAN_INDEX_MAX                (0x0000003FU)

/* POLICER_ETHERTYPE_IPSA_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_ETHERTYPE_IPSA_REG_ETHERTYPE_MEN_MASK   (0x80000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_ETHERTYPE_IPSA_REG_ETHERTYPE_MEN_SHIFT  (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_ETHERTYPE_IPSA_REG_ETHERTYPE_MEN_MAX    (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_ETHERTYPE_IPSA_REG_ETHERTYPE_INDEX_MASK (0x003F0000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_ETHERTYPE_IPSA_REG_ETHERTYPE_INDEX_SHIFT (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_ETHERTYPE_IPSA_REG_ETHERTYPE_INDEX_MAX  (0x0000003FU)

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_ETHERTYPE_IPSA_REG_IPSRC_MEN_MASK       (0x00008000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_ETHERTYPE_IPSA_REG_IPSRC_MEN_SHIFT      (0x0000000FU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_ETHERTYPE_IPSA_REG_IPSRC_MEN_MAX        (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_ETHERTYPE_IPSA_REG_IPSRC_INDEX_MASK     (0x0000003FU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_ETHERTYPE_IPSA_REG_IPSRC_INDEX_SHIFT    (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_ETHERTYPE_IPSA_REG_IPSRC_INDEX_MAX      (0x0000003FU)

/* POLICER_IPDA_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_IPDA_REG_IPDST_MEN_MASK                 (0x80000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_IPDA_REG_IPDST_MEN_SHIFT                (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_IPDA_REG_IPDST_MEN_MAX                  (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_IPDA_REG_IPDST_INDEX_MASK               (0x003F0000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_IPDA_REG_IPDST_INDEX_SHIFT              (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_IPDA_REG_IPDST_INDEX_MAX                (0x0000003FU)

/* POLICER_PIR_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_PIR_REG_PRI_IDLE_INC_VAL_MASK           (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_PIR_REG_PRI_IDLE_INC_VAL_SHIFT          (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_PIR_REG_PRI_IDLE_INC_VAL_MAX            (0xFFFFFFFFU)

/* POLICER_CIR_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_CIR_REG_CIR_IDLE_INC_VAL_MASK           (0xFFFFFFFFU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_CIR_REG_CIR_IDLE_INC_VAL_SHIFT          (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_CIR_REG_CIR_IDLE_INC_VAL_MAX            (0xFFFFFFFFU)

/* POLICER_TBL_CTL_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_TBL_CTL_REG_WRITE_ENABLE_MASK           (0x80000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_TBL_CTL_REG_WRITE_ENABLE_SHIFT          (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_TBL_CTL_REG_WRITE_ENABLE_MAX            (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_TBL_CTL_REG_POL_TBL_INDEX_MASK          (0x00000007U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_TBL_CTL_REG_POL_TBL_INDEX_SHIFT         (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_TBL_CTL_REG_POL_TBL_INDEX_MAX           (0x00000007U)

/* POLICER_CTL_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_CTL_REG_POL_EN_MASK                     (0x80000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_CTL_REG_POL_EN_SHIFT                    (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_CTL_REG_POL_EN_MAX                      (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_CTL_REG_RED_DROP_EN_MASK                (0x20000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_CTL_REG_RED_DROP_EN_SHIFT               (0x0000001DU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_CTL_REG_RED_DROP_EN_MAX                 (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_CTL_REG_YELLOW_DROP_EN_MASK             (0x10000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_CTL_REG_YELLOW_DROP_EN_SHIFT            (0x0000001CU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_CTL_REG_YELLOW_DROP_EN_MAX              (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_CTL_REG_YELLOW_THRESH_MASK              (0x07000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_CTL_REG_YELLOW_THRESH_SHIFT             (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_CTL_REG_YELLOW_THRESH_MAX               (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_CTL_REG_POL_MATCH_MODE_MASK             (0x00C00000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_CTL_REG_POL_MATCH_MODE_SHIFT            (0x00000016U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_CTL_REG_POL_MATCH_MODE_MAX              (0x00000003U)

/* POLICER_TEST_CTL_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_TEST_CTL_REG_POL_CLR_MASK               (0x80000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_TEST_CTL_REG_POL_CLR_SHIFT              (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_TEST_CTL_REG_POL_CLR_MAX                (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_TEST_CTL_REG_POL_CLR_RED_MASK           (0x40000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_TEST_CTL_REG_POL_CLR_RED_SHIFT          (0x0000001EU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_TEST_CTL_REG_POL_CLR_RED_MAX            (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_TEST_CTL_REG_POL_CLR_YELLOW_MASK        (0x20000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_TEST_CTL_REG_POL_CLR_YELLOW_SHIFT       (0x0000001DU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_TEST_CTL_REG_POL_CLR_YELLOW_MAX         (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_TEST_CTL_REG_POL_CLR_SELECTED_MASK      (0x10000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_TEST_CTL_REG_POL_CLR_SELECTED_SHIFT     (0x0000001CU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_TEST_CTL_REG_POL_CLR_SELECTED_MAX       (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_TEST_CTL_REG_POL_TEST_ENTRY_MASK        (0x00000007U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_TEST_CTL_REG_POL_TEST_ENTRY_SHIFT       (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_TEST_CTL_REG_POL_TEST_ENTRY_MAX         (0x00000007U)

/* POLICER_HIT_STATUS_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_HIT_STATUS_REG_POL_HIT_MASK             (0x80000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_HIT_STATUS_REG_POL_HIT_SHIFT            (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_HIT_STATUS_REG_POL_HIT_MAX              (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_HIT_STATUS_REG_POL_HIT_RED_MASK         (0x40000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_HIT_STATUS_REG_POL_HIT_RED_SHIFT        (0x0000001EU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_HIT_STATUS_REG_POL_HIT_RED_MAX          (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_HIT_STATUS_REG_POL_HIT_YELLOW_MASK      (0x20000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_HIT_STATUS_REG_POL_HIT_YELLOW_SHIFT     (0x0000001DU)
#define CSL_CPSW2G_CPSW_NU_ALE_POLICER_HIT_STATUS_REG_POL_HIT_YELLOW_MAX       (0x00000001U)

/* THREAD_DEF_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_THREAD_DEF_REG_ENABLE_MASK                      (0x00008000U)
#define CSL_CPSW2G_CPSW_NU_ALE_THREAD_DEF_REG_ENABLE_SHIFT                     (0x0000000FU)
#define CSL_CPSW2G_CPSW_NU_ALE_THREAD_DEF_REG_ENABLE_MAX                       (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_THREAD_DEF_REG_VALUE_MASK                       (0x0000003FU)
#define CSL_CPSW2G_CPSW_NU_ALE_THREAD_DEF_REG_VALUE_SHIFT                      (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_THREAD_DEF_REG_VALUE_MAX                        (0x0000003FU)

/* THREAD_CTL_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_THREAD_CTL_REG_ENTRY_PTR_MASK                   (0x00000007U)
#define CSL_CPSW2G_CPSW_NU_ALE_THREAD_CTL_REG_ENTRY_PTR_SHIFT                  (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_THREAD_CTL_REG_ENTRY_PTR_MAX                    (0x00000007U)

/* THREAD_VAL_REG */

#define CSL_CPSW2G_CPSW_NU_ALE_THREAD_VAL_REG_ENABLE_MASK                      (0x00008000U)
#define CSL_CPSW2G_CPSW_NU_ALE_THREAD_VAL_REG_ENABLE_SHIFT                     (0x0000000FU)
#define CSL_CPSW2G_CPSW_NU_ALE_THREAD_VAL_REG_ENABLE_MAX                       (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_ALE_THREAD_VAL_REG_VALUE_MASK                       (0x0000003FU)
#define CSL_CPSW2G_CPSW_NU_ALE_THREAD_VAL_REG_VALUE_SHIFT                      (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_ALE_THREAD_VAL_REG_VALUE_MAX                        (0x0000003FU)

/* CPSW_ID_VER_REG */

#define CSL_CPSW2G_CPSW_NU_CPSW_ID_VER_REG_MINOR_VER_MASK                      (0x0000003FU)
#define CSL_CPSW2G_CPSW_NU_CPSW_ID_VER_REG_MINOR_VER_SHIFT                     (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_ID_VER_REG_MINOR_VER_MAX                       (0x0000003FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_ID_VER_REG_CUSTOM_VER_MASK                     (0x000000C0U)
#define CSL_CPSW2G_CPSW_NU_CPSW_ID_VER_REG_CUSTOM_VER_SHIFT                    (0x00000006U)
#define CSL_CPSW2G_CPSW_NU_CPSW_ID_VER_REG_CUSTOM_VER_MAX                      (0x00000003U)

#define CSL_CPSW2G_CPSW_NU_CPSW_ID_VER_REG_MAJOR_VER_MASK                      (0x00000700U)
#define CSL_CPSW2G_CPSW_NU_CPSW_ID_VER_REG_MAJOR_VER_SHIFT                     (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CPSW_ID_VER_REG_MAJOR_VER_MAX                       (0x00000007U)

#define CSL_CPSW2G_CPSW_NU_CPSW_ID_VER_REG_RTL_VER_MASK                        (0x0000F800U)
#define CSL_CPSW2G_CPSW_NU_CPSW_ID_VER_REG_RTL_VER_SHIFT                       (0x0000000BU)
#define CSL_CPSW2G_CPSW_NU_CPSW_ID_VER_REG_RTL_VER_MAX                         (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_CPSW_ID_VER_REG_IDENT_MASK                          (0xFFFF0000U)
#define CSL_CPSW2G_CPSW_NU_CPSW_ID_VER_REG_IDENT_SHIFT                         (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CPSW_ID_VER_REG_IDENT_MAX                           (0x0000FFFFU)

/* CONTROL_REG */

#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_S_CN_SWITCH_MASK                        (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_S_CN_SWITCH_SHIFT                       (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_S_CN_SWITCH_MAX                         (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_VLAN_AWARE_MASK                         (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_VLAN_AWARE_SHIFT                        (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_VLAN_AWARE_MAX                          (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P0_ENABLE_MASK                          (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P0_ENABLE_SHIFT                         (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P0_ENABLE_MAX                           (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P0_PASS_PRI_TAGGED_MASK                 (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P0_PASS_PRI_TAGGED_SHIFT                (0x00000003U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P0_PASS_PRI_TAGGED_MAX                  (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P1_PASS_PRI_TAGGED_MASK                 (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P1_PASS_PRI_TAGGED_SHIFT                (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P1_PASS_PRI_TAGGED_MAX                  (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P2_PASS_PRI_TAGGED_MASK                 (0x00000020U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P2_PASS_PRI_TAGGED_SHIFT                (0x00000005U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P2_PASS_PRI_TAGGED_MAX                  (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P3_PASS_PRI_TAGGED_MASK                 (0x00000040U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P3_PASS_PRI_TAGGED_SHIFT                (0x00000006U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P3_PASS_PRI_TAGGED_MAX                  (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P4_PASS_PRI_TAGGED_MASK                 (0x00000080U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P4_PASS_PRI_TAGGED_SHIFT                (0x00000007U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P4_PASS_PRI_TAGGED_MAX                  (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P5_PASS_PRI_TAGGED_MASK                 (0x00000100U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P5_PASS_PRI_TAGGED_SHIFT                (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P5_PASS_PRI_TAGGED_MAX                  (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P6_PASS_PRI_TAGGED_MASK                 (0x00000200U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P6_PASS_PRI_TAGGED_SHIFT                (0x00000009U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P6_PASS_PRI_TAGGED_MAX                  (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P7_PASS_PRI_TAGGED_MASK                 (0x00000400U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P7_PASS_PRI_TAGGED_SHIFT                (0x0000000AU)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P7_PASS_PRI_TAGGED_MAX                  (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P8_PASS_PRI_TAGGED_MASK                 (0x00000800U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P8_PASS_PRI_TAGGED_SHIFT                (0x0000000BU)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P8_PASS_PRI_TAGGED_MAX                  (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P0_TX_CRC_REMOVE_MASK                   (0x00002000U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P0_TX_CRC_REMOVE_SHIFT                  (0x0000000DU)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P0_TX_CRC_REMOVE_MAX                    (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P0_RX_PAD_MASK                          (0x00004000U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P0_RX_PAD_SHIFT                         (0x0000000EU)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P0_RX_PAD_MAX                           (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P0_RX_PASS_CRC_ERR_MASK                 (0x00008000U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P0_RX_PASS_CRC_ERR_SHIFT                (0x0000000FU)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_P0_RX_PASS_CRC_ERR_MAX                  (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_EEE_ENABLE_MASK                         (0x00010000U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_EEE_ENABLE_SHIFT                        (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_EEE_ENABLE_MAX                          (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_IET_ENABLE_MASK                         (0x00020000U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_IET_ENABLE_SHIFT                        (0x00000011U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_IET_ENABLE_MAX                          (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_EST_ENABLE_MASK                         (0x00040000U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_EST_ENABLE_SHIFT                        (0x00000012U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_EST_ENABLE_MAX                          (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_ECC_CRC_MODE_MASK                       (0x80000000U)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_ECC_CRC_MODE_SHIFT                      (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_CONTROL_REG_ECC_CRC_MODE_MAX                        (0x00000001U)

/* EM_CONTROL_REG */

#define CSL_CPSW2G_CPSW_NU_EM_CONTROL_REG_FREE_MASK                            (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_EM_CONTROL_REG_FREE_SHIFT                           (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_EM_CONTROL_REG_FREE_MAX                             (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_EM_CONTROL_REG_SOFT_MASK                            (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_EM_CONTROL_REG_SOFT_SHIFT                           (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_EM_CONTROL_REG_SOFT_MAX                             (0x00000001U)

/* STAT_PORT_EN_REG */

#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG_P0_STAT_EN_MASK                    (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG_P0_STAT_EN_SHIFT                   (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG_P0_STAT_EN_MAX                     (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG_P1_STAT_EN_MASK                    (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG_P1_STAT_EN_SHIFT                   (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG_P1_STAT_EN_MAX                     (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG_P2_STAT_EN_MASK                    (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG_P2_STAT_EN_SHIFT                   (0x00000002U)
#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG_P2_STAT_EN_MAX                     (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG_P3_STAT_EN_MASK                    (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG_P3_STAT_EN_SHIFT                   (0x00000003U)
#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG_P3_STAT_EN_MAX                     (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG_P4_STAT_EN_MASK                    (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG_P4_STAT_EN_SHIFT                   (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG_P4_STAT_EN_MAX                     (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG_P5_STAT_EN_MASK                    (0x00000020U)
#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG_P5_STAT_EN_SHIFT                   (0x00000005U)
#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG_P5_STAT_EN_MAX                     (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG_P6_STAT_EN_MASK                    (0x00000040U)
#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG_P6_STAT_EN_SHIFT                   (0x00000006U)
#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG_P6_STAT_EN_MAX                     (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG_P7_STAT_EN_MASK                    (0x00000080U)
#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG_P7_STAT_EN_SHIFT                   (0x00000007U)
#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG_P7_STAT_EN_MAX                     (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG_P8_STAT_EN_MASK                    (0x00000100U)
#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG_P8_STAT_EN_SHIFT                   (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_STAT_PORT_EN_REG_P8_STAT_EN_MAX                     (0x00000001U)

/* PTYPE_REG */

#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_ESC_PRI_LD_VAL_MASK                       (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_ESC_PRI_LD_VAL_SHIFT                      (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_ESC_PRI_LD_VAL_MAX                        (0x0000001FU)

#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_P0_PTYPE_ESC_MASK                         (0x00000100U)
#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_P0_PTYPE_ESC_SHIFT                        (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_P0_PTYPE_ESC_MAX                          (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_P1_PTYPE_ESC_MASK                         (0x00000200U)
#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_P1_PTYPE_ESC_SHIFT                        (0x00000009U)
#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_P1_PTYPE_ESC_MAX                          (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_P2_PTYPE_ESC_MASK                         (0x00000400U)
#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_P2_PTYPE_ESC_SHIFT                        (0x0000000AU)
#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_P2_PTYPE_ESC_MAX                          (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_P3_PTYPE_ESC_MASK                         (0x00000800U)
#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_P3_PTYPE_ESC_SHIFT                        (0x0000000BU)
#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_P3_PTYPE_ESC_MAX                          (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_P4_PTYPE_ESC_MASK                         (0x00001000U)
#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_P4_PTYPE_ESC_SHIFT                        (0x0000000CU)
#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_P4_PTYPE_ESC_MAX                          (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_P5_PTYPE_ESC_MASK                         (0x00002000U)
#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_P5_PTYPE_ESC_SHIFT                        (0x0000000DU)
#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_P5_PTYPE_ESC_MAX                          (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_P6_PTYPE_ESC_MASK                         (0x00004000U)
#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_P6_PTYPE_ESC_SHIFT                        (0x0000000EU)
#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_P6_PTYPE_ESC_MAX                          (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_P7_PTYPE_ESC_MASK                         (0x00008000U)
#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_P7_PTYPE_ESC_SHIFT                        (0x0000000FU)
#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_P7_PTYPE_ESC_MAX                          (0x00000001U)

#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_P8_PTYPE_ESC_MASK                         (0x00010000U)
#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_P8_PTYPE_ESC_SHIFT                        (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_PTYPE_REG_P8_PTYPE_ESC_MAX                          (0x00000001U)

/* SOFT_IDLE_REG */

#define CSL_CPSW2G_CPSW_NU_SOFT_IDLE_REG_SOFT_IDLE_MASK                        (0x00000001U)
#define CSL_CPSW2G_CPSW_NU_SOFT_IDLE_REG_SOFT_IDLE_SHIFT                       (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_SOFT_IDLE_REG_SOFT_IDLE_MAX                         (0x00000001U)

/* THRU_RATE_REG */

#define CSL_CPSW2G_CPSW_NU_THRU_RATE_REG_P0_RX_THRU_RATE_MASK                  (0x0000000FU)
#define CSL_CPSW2G_CPSW_NU_THRU_RATE_REG_P0_RX_THRU_RATE_SHIFT                 (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_THRU_RATE_REG_P0_RX_THRU_RATE_MAX                   (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_THRU_RATE_REG_SL_RX_THRU_RATE_MASK                  (0x0000F000U)
#define CSL_CPSW2G_CPSW_NU_THRU_RATE_REG_SL_RX_THRU_RATE_SHIFT                 (0x0000000CU)
#define CSL_CPSW2G_CPSW_NU_THRU_RATE_REG_SL_RX_THRU_RATE_MAX                   (0x0000000FU)

/* GAP_THRESH_REG */

#define CSL_CPSW2G_CPSW_NU_GAP_THRESH_REG_GAP_THRESH_MASK                      (0x0000001FU)
#define CSL_CPSW2G_CPSW_NU_GAP_THRESH_REG_GAP_THRESH_SHIFT                     (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_GAP_THRESH_REG_GAP_THRESH_MAX                       (0x0000001FU)

/* TX_START_WDS_REG */

#define CSL_CPSW2G_CPSW_NU_TX_START_WDS_REG_TX_START_WDS_MASK                  (0x000007FFU)
#define CSL_CPSW2G_CPSW_NU_TX_START_WDS_REG_TX_START_WDS_SHIFT                 (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_TX_START_WDS_REG_TX_START_WDS_MAX                   (0x000007FFU)

/* EEE_PRESCALE_REG */

#define CSL_CPSW2G_CPSW_NU_EEE_PRESCALE_REG_EEE_PRESCALE_MASK                  (0x00000FFFU)
#define CSL_CPSW2G_CPSW_NU_EEE_PRESCALE_REG_EEE_PRESCALE_SHIFT                 (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_EEE_PRESCALE_REG_EEE_PRESCALE_MAX                   (0x00000FFFU)

/* TX_G_OFLOW_THRESH_SET_REG */

#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_SET_REG_PRI0_MASK                 (0x0000000FU)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_SET_REG_PRI0_SHIFT                (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_SET_REG_PRI0_MAX                  (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_SET_REG_PRI1_MASK                 (0x000000F0U)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_SET_REG_PRI1_SHIFT                (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_SET_REG_PRI1_MAX                  (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_SET_REG_PRI2_MASK                 (0x00000F00U)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_SET_REG_PRI2_SHIFT                (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_SET_REG_PRI2_MAX                  (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_SET_REG_PRI3_MASK                 (0x0000F000U)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_SET_REG_PRI3_SHIFT                (0x0000000CU)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_SET_REG_PRI3_MAX                  (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_SET_REG_PRI4_MASK                 (0x000F0000U)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_SET_REG_PRI4_SHIFT                (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_SET_REG_PRI4_MAX                  (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_SET_REG_PRI5_MASK                 (0x00F00000U)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_SET_REG_PRI5_SHIFT                (0x00000014U)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_SET_REG_PRI5_MAX                  (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_SET_REG_PRI6_MASK                 (0x0F000000U)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_SET_REG_PRI6_SHIFT                (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_SET_REG_PRI6_MAX                  (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_SET_REG_PRI7_MASK                 (0xF0000000U)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_SET_REG_PRI7_SHIFT                (0x0000001CU)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_SET_REG_PRI7_MAX                  (0x0000000FU)

/* TX_G_OFLOW_THRESH_CLR_REG */

#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_CLR_REG_PRI0_MASK                 (0x0000000FU)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_CLR_REG_PRI0_SHIFT                (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_CLR_REG_PRI0_MAX                  (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_CLR_REG_PRI1_MASK                 (0x000000F0U)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_CLR_REG_PRI1_SHIFT                (0x00000004U)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_CLR_REG_PRI1_MAX                  (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_CLR_REG_PRI2_MASK                 (0x00000F00U)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_CLR_REG_PRI2_SHIFT                (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_CLR_REG_PRI2_MAX                  (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_CLR_REG_PRI3_MASK                 (0x0000F000U)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_CLR_REG_PRI3_SHIFT                (0x0000000CU)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_CLR_REG_PRI3_MAX                  (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_CLR_REG_PRI4_MASK                 (0x000F0000U)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_CLR_REG_PRI4_SHIFT                (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_CLR_REG_PRI4_MAX                  (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_CLR_REG_PRI5_MASK                 (0x00F00000U)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_CLR_REG_PRI5_SHIFT                (0x00000014U)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_CLR_REG_PRI5_MAX                  (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_CLR_REG_PRI6_MASK                 (0x0F000000U)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_CLR_REG_PRI6_SHIFT                (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_CLR_REG_PRI6_MAX                  (0x0000000FU)

#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_CLR_REG_PRI7_MASK                 (0xF0000000U)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_CLR_REG_PRI7_SHIFT                (0x0000001CU)
#define CSL_CPSW2G_CPSW_NU_TX_G_OFLOW_THRESH_CLR_REG_PRI7_MAX                  (0x0000000FU)

/* TX_G_BUF_THRESH_SET_L_REG */

#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_SET_L_REG_PRI0_MASK                 (0x000000FFU)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_SET_L_REG_PRI0_SHIFT                (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_SET_L_REG_PRI0_MAX                  (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_SET_L_REG_PRI1_MASK                 (0x0000FF00U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_SET_L_REG_PRI1_SHIFT                (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_SET_L_REG_PRI1_MAX                  (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_SET_L_REG_PRI2_MASK                 (0x00FF0000U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_SET_L_REG_PRI2_SHIFT                (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_SET_L_REG_PRI2_MAX                  (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_SET_L_REG_PRI3_MASK                 (0xFF000000U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_SET_L_REG_PRI3_SHIFT                (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_SET_L_REG_PRI3_MAX                  (0x000000FFU)

/* TX_G_BUF_THRESH_SET_H_REG */

#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_SET_H_REG_PRI4_MASK                 (0x000000FFU)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_SET_H_REG_PRI4_SHIFT                (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_SET_H_REG_PRI4_MAX                  (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_SET_H_REG_PRI5_MASK                 (0x0000FF00U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_SET_H_REG_PRI5_SHIFT                (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_SET_H_REG_PRI5_MAX                  (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_SET_H_REG_PRI6_MASK                 (0x00FF0000U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_SET_H_REG_PRI6_SHIFT                (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_SET_H_REG_PRI6_MAX                  (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_SET_H_REG_PRI7_MASK                 (0xFF000000U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_SET_H_REG_PRI7_SHIFT                (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_SET_H_REG_PRI7_MAX                  (0x000000FFU)

/* TX_G_BUF_THRESH_CLR_L_REG */

#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_CLR_L_REG_PRI0_MASK                 (0x000000FFU)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_CLR_L_REG_PRI0_SHIFT                (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_CLR_L_REG_PRI0_MAX                  (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_CLR_L_REG_PRI1_MASK                 (0x0000FF00U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_CLR_L_REG_PRI1_SHIFT                (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_CLR_L_REG_PRI1_MAX                  (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_CLR_L_REG_PRI2_MASK                 (0x00FF0000U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_CLR_L_REG_PRI2_SHIFT                (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_CLR_L_REG_PRI2_MAX                  (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_CLR_L_REG_PRI3_MASK                 (0xFF000000U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_CLR_L_REG_PRI3_SHIFT                (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_CLR_L_REG_PRI3_MAX                  (0x000000FFU)

/* TX_G_BUF_THRESH_CLR_H_REG */

#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_CLR_H_REG_PRI4_MASK                 (0x000000FFU)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_CLR_H_REG_PRI4_SHIFT                (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_CLR_H_REG_PRI4_MAX                  (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_CLR_H_REG_PRI5_MASK                 (0x0000FF00U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_CLR_H_REG_PRI5_SHIFT                (0x00000008U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_CLR_H_REG_PRI5_MAX                  (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_CLR_H_REG_PRI6_MASK                 (0x00FF0000U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_CLR_H_REG_PRI6_SHIFT                (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_CLR_H_REG_PRI6_MAX                  (0x000000FFU)

#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_CLR_H_REG_PRI7_MASK                 (0xFF000000U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_CLR_H_REG_PRI7_SHIFT                (0x00000018U)
#define CSL_CPSW2G_CPSW_NU_TX_G_BUF_THRESH_CLR_H_REG_PRI7_MAX                  (0x000000FFU)

/* VLAN_LTYPE_REG */

#define CSL_CPSW2G_CPSW_NU_VLAN_LTYPE_REG_VLAN_LTYPE_INNER_MASK                (0x0000FFFFU)
#define CSL_CPSW2G_CPSW_NU_VLAN_LTYPE_REG_VLAN_LTYPE_INNER_SHIFT               (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_VLAN_LTYPE_REG_VLAN_LTYPE_INNER_MAX                 (0x0000FFFFU)

#define CSL_CPSW2G_CPSW_NU_VLAN_LTYPE_REG_VLAN_LTYPE_OUTER_MASK                (0xFFFF0000U)
#define CSL_CPSW2G_CPSW_NU_VLAN_LTYPE_REG_VLAN_LTYPE_OUTER_SHIFT               (0x00000010U)
#define CSL_CPSW2G_CPSW_NU_VLAN_LTYPE_REG_VLAN_LTYPE_OUTER_MAX                 (0x0000FFFFU)

/* EST_TS_DOMAIN_REG */

#define CSL_CPSW2G_CPSW_NU_EST_TS_DOMAIN_REG_EST_TS_DOMAIN_MASK                (0x000000FFU)
#define CSL_CPSW2G_CPSW_NU_EST_TS_DOMAIN_REG_EST_TS_DOMAIN_SHIFT               (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_EST_TS_DOMAIN_REG_EST_TS_DOMAIN_MAX                 (0x000000FFU)

/* TX_PRI0_MAXLEN_REG */

#define CSL_CPSW2G_CPSW_NU_TX_PRI0_MAXLEN_REG_TX_PRI0_MAXLEN_MASK              (0x00003FFFU)
#define CSL_CPSW2G_CPSW_NU_TX_PRI0_MAXLEN_REG_TX_PRI0_MAXLEN_SHIFT             (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_TX_PRI0_MAXLEN_REG_TX_PRI0_MAXLEN_MAX               (0x00003FFFU)

/* TX_PRI1_MAXLEN_REG */

#define CSL_CPSW2G_CPSW_NU_TX_PRI1_MAXLEN_REG_TX_PRI1_MAXLEN_MASK              (0x00003FFFU)
#define CSL_CPSW2G_CPSW_NU_TX_PRI1_MAXLEN_REG_TX_PRI1_MAXLEN_SHIFT             (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_TX_PRI1_MAXLEN_REG_TX_PRI1_MAXLEN_MAX               (0x00003FFFU)

/* TX_PRI2_MAXLEN_REG */

#define CSL_CPSW2G_CPSW_NU_TX_PRI2_MAXLEN_REG_TX_PRI2_MAXLEN_MASK              (0x00003FFFU)
#define CSL_CPSW2G_CPSW_NU_TX_PRI2_MAXLEN_REG_TX_PRI2_MAXLEN_SHIFT             (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_TX_PRI2_MAXLEN_REG_TX_PRI2_MAXLEN_MAX               (0x00003FFFU)

/* TX_PRI3_MAXLEN_REG */

#define CSL_CPSW2G_CPSW_NU_TX_PRI3_MAXLEN_REG_TX_PRI3_MAXLEN_MASK              (0x00003FFFU)
#define CSL_CPSW2G_CPSW_NU_TX_PRI3_MAXLEN_REG_TX_PRI3_MAXLEN_SHIFT             (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_TX_PRI3_MAXLEN_REG_TX_PRI3_MAXLEN_MAX               (0x00003FFFU)

/* TX_PRI4_MAXLEN_REG */

#define CSL_CPSW2G_CPSW_NU_TX_PRI4_MAXLEN_REG_TX_PRI4_MAXLEN_MASK              (0x00003FFFU)
#define CSL_CPSW2G_CPSW_NU_TX_PRI4_MAXLEN_REG_TX_PRI4_MAXLEN_SHIFT             (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_TX_PRI4_MAXLEN_REG_TX_PRI4_MAXLEN_MAX               (0x00003FFFU)

/* TX_PRI5_MAXLEN_REG */

#define CSL_CPSW2G_CPSW_NU_TX_PRI5_MAXLEN_REG_TX_PRI5_MAXLEN_MASK              (0x00003FFFU)
#define CSL_CPSW2G_CPSW_NU_TX_PRI5_MAXLEN_REG_TX_PRI5_MAXLEN_SHIFT             (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_TX_PRI5_MAXLEN_REG_TX_PRI5_MAXLEN_MAX               (0x00003FFFU)

/* TX_PRI6_MAXLEN_REG */

#define CSL_CPSW2G_CPSW_NU_TX_PRI6_MAXLEN_REG_TX_PRI6_MAXLEN_MASK              (0x00003FFFU)
#define CSL_CPSW2G_CPSW_NU_TX_PRI6_MAXLEN_REG_TX_PRI6_MAXLEN_SHIFT             (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_TX_PRI6_MAXLEN_REG_TX_PRI6_MAXLEN_MAX               (0x00003FFFU)

/* TX_PRI7_MAXLEN_REG */

#define CSL_CPSW2G_CPSW_NU_TX_PRI7_MAXLEN_REG_TX_PRI7_MAXLEN_MASK              (0x00003FFFU)
#define CSL_CPSW2G_CPSW_NU_TX_PRI7_MAXLEN_REG_TX_PRI7_MAXLEN_SHIFT             (0x00000000U)
#define CSL_CPSW2G_CPSW_NU_TX_PRI7_MAXLEN_REG_TX_PRI7_MAXLEN_MAX               (0x00003FFFU)

/* CPSW_NUSS_IDVER_REG */

#define CSL_CPSW2G_CPSW_NUSS_IDVER_REG_MINOR_VER_MASK                          (0x000000FFU)
#define CSL_CPSW2G_CPSW_NUSS_IDVER_REG_MINOR_VER_SHIFT                         (0x00000000U)
#define CSL_CPSW2G_CPSW_NUSS_IDVER_REG_MINOR_VER_MAX                           (0x000000FFU)

#define CSL_CPSW2G_CPSW_NUSS_IDVER_REG_MAJOR_VER_MASK                          (0x00000700U)
#define CSL_CPSW2G_CPSW_NUSS_IDVER_REG_MAJOR_VER_SHIFT                         (0x00000008U)
#define CSL_CPSW2G_CPSW_NUSS_IDVER_REG_MAJOR_VER_MAX                           (0x00000007U)

#define CSL_CPSW2G_CPSW_NUSS_IDVER_REG_RTL_VER_MASK                            (0x0000F800U)
#define CSL_CPSW2G_CPSW_NUSS_IDVER_REG_RTL_VER_SHIFT                           (0x0000000BU)
#define CSL_CPSW2G_CPSW_NUSS_IDVER_REG_RTL_VER_MAX                             (0x0000001FU)

#define CSL_CPSW2G_CPSW_NUSS_IDVER_REG_IDENT_MASK                              (0xFFFF0000U)
#define CSL_CPSW2G_CPSW_NUSS_IDVER_REG_IDENT_SHIFT                             (0x00000010U)
#define CSL_CPSW2G_CPSW_NUSS_IDVER_REG_IDENT_MAX                               (0x0000FFFFU)

/* SYNCE_COUNT_REG */

#define CSL_CPSW2G_SYNCE_COUNT_REG_SYNCE_CNT_MASK                              (0xFFFFFFFFU)
#define CSL_CPSW2G_SYNCE_COUNT_REG_SYNCE_CNT_SHIFT                             (0x00000000U)
#define CSL_CPSW2G_SYNCE_COUNT_REG_SYNCE_CNT_MAX                               (0xFFFFFFFFU)

/* SYNCE_MUX_REG */

#define CSL_CPSW2G_SYNCE_MUX_REG_SYNCE_SEL_MASK                                (0x0000003FU)
#define CSL_CPSW2G_SYNCE_MUX_REG_SYNCE_SEL_SHIFT                               (0x00000000U)
#define CSL_CPSW2G_SYNCE_MUX_REG_SYNCE_SEL_MAX                                 (0x0000003FU)

/* CONTROL_REG */

#define CSL_CPSW2G_CONTROL_REG_EEE_EN_MASK                                     (0x00000001U)
#define CSL_CPSW2G_CONTROL_REG_EEE_EN_SHIFT                                    (0x00000000U)
#define CSL_CPSW2G_CONTROL_REG_EEE_EN_MAX                                      (0x00000001U)

#define CSL_CPSW2G_CONTROL_REG_EEE_PHY_ONLY_MASK                               (0x00000002U)
#define CSL_CPSW2G_CONTROL_REG_EEE_PHY_ONLY_SHIFT                              (0x00000001U)
#define CSL_CPSW2G_CONTROL_REG_EEE_PHY_ONLY_MAX                                (0x00000001U)

/* SGMII_MODE_REG */

#define CSL_CPSW2G_SGMII_MODE_REG_SYNCE_SEL_MASK                               (0x00000001U)
#define CSL_CPSW2G_SGMII_MODE_REG_SYNCE_SEL_SHIFT                              (0x00000000U)
#define CSL_CPSW2G_SGMII_MODE_REG_SYNCE_SEL_MAX                                (0x00000001U)

/* RGMII_STATUS_REG */

#define CSL_CPSW2G_RGMII_STATUS_REG_LINK_MASK                                  (0x00000001U)
#define CSL_CPSW2G_RGMII_STATUS_REG_LINK_SHIFT                                 (0x00000000U)
#define CSL_CPSW2G_RGMII_STATUS_REG_LINK_MAX                                   (0x00000001U)

#define CSL_CPSW2G_RGMII_STATUS_REG_SPEED_MASK                                 (0x00000006U)
#define CSL_CPSW2G_RGMII_STATUS_REG_SPEED_SHIFT                                (0x00000001U)
#define CSL_CPSW2G_RGMII_STATUS_REG_SPEED_MAX                                  (0x00000003U)

#define CSL_CPSW2G_RGMII_STATUS_REG_FULLDUPLEX_MASK                            (0x00000008U)
#define CSL_CPSW2G_RGMII_STATUS_REG_FULLDUPLEX_SHIFT                           (0x00000003U)
#define CSL_CPSW2G_RGMII_STATUS_REG_FULLDUPLEX_MAX                             (0x00000001U)

/* SUBSSYSTEM_STATUS_REG */

#define CSL_CPSW2G_SUBSSYSTEM_STATUS_REG_EEE_CLKSTOP_ACK_MASK                  (0x00000001U)
#define CSL_CPSW2G_SUBSSYSTEM_STATUS_REG_EEE_CLKSTOP_ACK_SHIFT                 (0x00000000U)
#define CSL_CPSW2G_SUBSSYSTEM_STATUS_REG_EEE_CLKSTOP_ACK_MAX                   (0x00000001U)

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t REV;                       /* Aggregator Revision Register */
    volatile uint8_t  Resv_8[4];
    volatile uint32_t VECTOR;                    /* ECC Vector Register */
    volatile uint32_t STAT;                      /* Misc Status */
    volatile uint8_t  Resv_60[44];
    volatile uint32_t SEC_EOI_REG;               /* EOI Register */
    volatile uint32_t SEC_STATUS_REG0;           /* Interrupt Status Register 0 */
    volatile uint8_t  Resv_128[60];
    volatile uint32_t SEC_ENABLE_SET_REG0;       /* Interrupt Enable Set Register 0 */
    volatile uint8_t  Resv_192[60];
    volatile uint32_t SEC_ENABLE_CLR_REG0;       /* Interrupt Enable Clear Register 0 */
    volatile uint8_t  Resv_316[120];
    volatile uint32_t DED_EOI_REG;               /* EOI Register */
    volatile uint32_t DED_STATUS_REG0;           /* Interrupt Status Register 0 */
    volatile uint8_t  Resv_384[60];
    volatile uint32_t DED_ENABLE_SET_REG0;       /* Interrupt Enable Set Register 0 */
    volatile uint8_t  Resv_448[60];
    volatile uint32_t DED_ENABLE_CLR_REG0;       /* Interrupt Enable Clear Register 0 */
    volatile uint8_t  Resv_512[60];
    volatile uint32_t AGGR_ENABLE_SET;           /* AGGR interrupt enable set Register */
    volatile uint32_t AGGR_ENABLE_CLR;           /* AGGR interrupt enable clear Register */
    volatile uint32_t AGGR_STATUS_SET;           /* AGGR interrupt status set Register */
    volatile uint32_t AGGR_STATUS_CLR;           /* AGGR interrupt status clear Register */
} CSL_cpsw2g_eccRegs_ECC;


typedef struct {
    CSL_cpsw2g_eccRegs_ECC ECC;
} CSL_cpsw2g_eccRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_CPSW2G_ECC_ECC_REV                                                 (0x00000000U)
#define CSL_CPSW2G_ECC_ECC_VECTOR                                              (0x00000008U)
#define CSL_CPSW2G_ECC_ECC_STAT                                                (0x0000000CU)
#define CSL_CPSW2G_ECC_ECC_SEC_EOI_REG                                         (0x0000003CU)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0                                     (0x00000040U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0                                 (0x00000080U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0                                 (0x000000C0U)
#define CSL_CPSW2G_ECC_ECC_DED_EOI_REG                                         (0x0000013CU)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0                                     (0x00000140U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0                                 (0x00000180U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0                                 (0x000001C0U)
#define CSL_CPSW2G_ECC_ECC_AGGR_ENABLE_SET                                     (0x00000200U)
#define CSL_CPSW2G_ECC_ECC_AGGR_ENABLE_CLR                                     (0x00000204U)
#define CSL_CPSW2G_ECC_ECC_AGGR_STATUS_SET                                     (0x00000208U)
#define CSL_CPSW2G_ECC_ECC_AGGR_STATUS_CLR                                     (0x0000020CU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* REV */

#define CSL_CPSW2G_ECC_ECC_REV_SCHEME_MASK                                     (0xC0000000U)
#define CSL_CPSW2G_ECC_ECC_REV_SCHEME_SHIFT                                    (0x0000001EU)
#define CSL_CPSW2G_ECC_ECC_REV_SCHEME_MAX                                      (0x00000003U)

#define CSL_CPSW2G_ECC_ECC_REV_BU_MASK                                         (0x30000000U)
#define CSL_CPSW2G_ECC_ECC_REV_BU_SHIFT                                        (0x0000001CU)
#define CSL_CPSW2G_ECC_ECC_REV_BU_MAX                                          (0x00000003U)

#define CSL_CPSW2G_ECC_ECC_REV_MODULE_ID_MASK                                  (0x0FFF0000U)
#define CSL_CPSW2G_ECC_ECC_REV_MODULE_ID_SHIFT                                 (0x00000010U)
#define CSL_CPSW2G_ECC_ECC_REV_MODULE_ID_MAX                                   (0x00000FFFU)

#define CSL_CPSW2G_ECC_ECC_REV_REVRTL_MASK                                     (0x0000F800U)
#define CSL_CPSW2G_ECC_ECC_REV_REVRTL_SHIFT                                    (0x0000000BU)
#define CSL_CPSW2G_ECC_ECC_REV_REVRTL_MAX                                      (0x0000001FU)

#define CSL_CPSW2G_ECC_ECC_REV_REVMAJ_MASK                                     (0x00000700U)
#define CSL_CPSW2G_ECC_ECC_REV_REVMAJ_SHIFT                                    (0x00000008U)
#define CSL_CPSW2G_ECC_ECC_REV_REVMAJ_MAX                                      (0x00000007U)

#define CSL_CPSW2G_ECC_ECC_REV_CUSTOM_MASK                                     (0x000000C0U)
#define CSL_CPSW2G_ECC_ECC_REV_CUSTOM_SHIFT                                    (0x00000006U)
#define CSL_CPSW2G_ECC_ECC_REV_CUSTOM_MAX                                      (0x00000003U)

#define CSL_CPSW2G_ECC_ECC_REV_REVMIN_MASK                                     (0x0000003FU)
#define CSL_CPSW2G_ECC_ECC_REV_REVMIN_SHIFT                                    (0x00000000U)
#define CSL_CPSW2G_ECC_ECC_REV_REVMIN_MAX                                      (0x0000003FU)

/* VECTOR */

#define CSL_CPSW2G_ECC_ECC_VECTOR_ECC_VECTOR_MASK                              (0x000007FFU)
#define CSL_CPSW2G_ECC_ECC_VECTOR_ECC_VECTOR_SHIFT                             (0x00000000U)
#define CSL_CPSW2G_ECC_ECC_VECTOR_ECC_VECTOR_MAX                               (0x000007FFU)

#define CSL_CPSW2G_ECC_ECC_VECTOR_RD_SVBUS_MASK                                (0x00008000U)
#define CSL_CPSW2G_ECC_ECC_VECTOR_RD_SVBUS_SHIFT                               (0x0000000FU)
#define CSL_CPSW2G_ECC_ECC_VECTOR_RD_SVBUS_MAX                                 (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_VECTOR_RD_SVBUS_ADDRESS_MASK                        (0x00FF0000U)
#define CSL_CPSW2G_ECC_ECC_VECTOR_RD_SVBUS_ADDRESS_SHIFT                       (0x00000010U)
#define CSL_CPSW2G_ECC_ECC_VECTOR_RD_SVBUS_ADDRESS_MAX                         (0x000000FFU)

#define CSL_CPSW2G_ECC_ECC_VECTOR_RD_SVBUS_DONE_MASK                           (0x01000000U)
#define CSL_CPSW2G_ECC_ECC_VECTOR_RD_SVBUS_DONE_SHIFT                          (0x00000018U)
#define CSL_CPSW2G_ECC_ECC_VECTOR_RD_SVBUS_DONE_MAX                            (0x00000001U)

/* STAT */

#define CSL_CPSW2G_ECC_ECC_STAT_NUM_RAMS_MASK                                  (0x000007FFU)
#define CSL_CPSW2G_ECC_ECC_STAT_NUM_RAMS_SHIFT                                 (0x00000000U)
#define CSL_CPSW2G_ECC_ECC_STAT_NUM_RAMS_MAX                                   (0x000007FFU)

/* SEC_EOI_REG */

#define CSL_CPSW2G_ECC_ECC_SEC_EOI_REG_EOI_WR_MASK                             (0x00000001U)
#define CSL_CPSW2G_ECC_ECC_SEC_EOI_REG_EOI_WR_SHIFT                            (0x00000000U)
#define CSL_CPSW2G_ECC_ECC_SEC_EOI_REG_EOI_WR_MAX                              (0x00000001U)

/* SEC_STATUS_REG0 */

#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC0_PEND_MASK                   (0x00000001U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC0_PEND_SHIFT                  (0x00000000U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC0_PEND_MAX                    (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC1_PEND_MASK                   (0x00000002U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC1_PEND_SHIFT                  (0x00000001U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC1_PEND_MAX                    (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC2_PEND_MASK                   (0x00000004U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC2_PEND_SHIFT                  (0x00000002U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC2_PEND_MAX                    (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC3_PEND_MASK                   (0x00000008U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC3_PEND_SHIFT                  (0x00000003U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC3_PEND_MAX                    (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC4_PEND_MASK                   (0x00000010U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC4_PEND_SHIFT                  (0x00000004U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC4_PEND_MAX                    (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC5_PEND_MASK                   (0x00000020U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC5_PEND_SHIFT                  (0x00000005U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC5_PEND_MAX                    (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC6_PEND_MASK                   (0x00000040U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC6_PEND_SHIFT                  (0x00000006U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC6_PEND_MAX                    (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC7_PEND_MASK                   (0x00000080U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC7_PEND_SHIFT                  (0x00000007U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC7_PEND_MAX                    (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC8_PEND_MASK                   (0x00000100U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC8_PEND_SHIFT                  (0x00000008U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC8_PEND_MAX                    (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC9_PEND_MASK                   (0x00000200U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC9_PEND_SHIFT                  (0x00000009U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC9_PEND_MAX                    (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC10_PEND_MASK                  (0x00000400U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC10_PEND_SHIFT                 (0x0000000AU)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC10_PEND_MAX                   (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC11_PEND_MASK                  (0x00000800U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC11_PEND_SHIFT                 (0x0000000BU)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC11_PEND_MAX                   (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC12_PEND_MASK                  (0x00001000U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC12_PEND_SHIFT                 (0x0000000CU)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC12_PEND_MAX                   (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC13_PEND_MASK                  (0x00002000U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC13_PEND_SHIFT                 (0x0000000DU)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC13_PEND_MAX                   (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC14_PEND_MASK                  (0x00004000U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC14_PEND_SHIFT                 (0x0000000EU)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC14_PEND_MAX                   (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC15_PEND_MASK                  (0x00008000U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC15_PEND_SHIFT                 (0x0000000FU)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC15_PEND_MAX                   (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC16_PEND_MASK                  (0x00010000U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC16_PEND_SHIFT                 (0x00000010U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC16_PEND_MAX                   (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC17_PEND_MASK                  (0x00020000U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC17_PEND_SHIFT                 (0x00000011U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC17_PEND_MAX                   (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC18_PEND_MASK                  (0x00040000U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC18_PEND_SHIFT                 (0x00000012U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC18_PEND_MAX                   (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC19_PEND_MASK                  (0x00080000U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC19_PEND_SHIFT                 (0x00000013U)
#define CSL_CPSW2G_ECC_ECC_SEC_STATUS_REG0_RAMECC19_PEND_MAX                   (0x00000001U)

/* SEC_ENABLE_SET_REG0 */

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC0_ENABLE_SET_MASK         (0x00000001U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC0_ENABLE_SET_SHIFT        (0x00000000U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC0_ENABLE_SET_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC1_ENABLE_SET_MASK         (0x00000002U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC1_ENABLE_SET_SHIFT        (0x00000001U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC1_ENABLE_SET_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC2_ENABLE_SET_MASK         (0x00000004U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC2_ENABLE_SET_SHIFT        (0x00000002U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC2_ENABLE_SET_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC3_ENABLE_SET_MASK         (0x00000008U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC3_ENABLE_SET_SHIFT        (0x00000003U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC3_ENABLE_SET_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC4_ENABLE_SET_MASK         (0x00000010U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC4_ENABLE_SET_SHIFT        (0x00000004U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC4_ENABLE_SET_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC5_ENABLE_SET_MASK         (0x00000020U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC5_ENABLE_SET_SHIFT        (0x00000005U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC5_ENABLE_SET_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC6_ENABLE_SET_MASK         (0x00000040U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC6_ENABLE_SET_SHIFT        (0x00000006U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC6_ENABLE_SET_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC7_ENABLE_SET_MASK         (0x00000080U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC7_ENABLE_SET_SHIFT        (0x00000007U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC7_ENABLE_SET_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC8_ENABLE_SET_MASK         (0x00000100U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC8_ENABLE_SET_SHIFT        (0x00000008U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC8_ENABLE_SET_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC9_ENABLE_SET_MASK         (0x00000200U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC9_ENABLE_SET_SHIFT        (0x00000009U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC9_ENABLE_SET_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC10_ENABLE_SET_MASK        (0x00000400U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC10_ENABLE_SET_SHIFT       (0x0000000AU)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC10_ENABLE_SET_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC11_ENABLE_SET_MASK        (0x00000800U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC11_ENABLE_SET_SHIFT       (0x0000000BU)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC11_ENABLE_SET_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC12_ENABLE_SET_MASK        (0x00001000U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC12_ENABLE_SET_SHIFT       (0x0000000CU)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC12_ENABLE_SET_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC13_ENABLE_SET_MASK        (0x00002000U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC13_ENABLE_SET_SHIFT       (0x0000000DU)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC13_ENABLE_SET_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC14_ENABLE_SET_MASK        (0x00004000U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC14_ENABLE_SET_SHIFT       (0x0000000EU)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC14_ENABLE_SET_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC15_ENABLE_SET_MASK        (0x00008000U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC15_ENABLE_SET_SHIFT       (0x0000000FU)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC15_ENABLE_SET_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC16_ENABLE_SET_MASK        (0x00010000U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC16_ENABLE_SET_SHIFT       (0x00000010U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC16_ENABLE_SET_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC17_ENABLE_SET_MASK        (0x00020000U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC17_ENABLE_SET_SHIFT       (0x00000011U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC17_ENABLE_SET_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC18_ENABLE_SET_MASK        (0x00040000U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC18_ENABLE_SET_SHIFT       (0x00000012U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC18_ENABLE_SET_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC19_ENABLE_SET_MASK        (0x00080000U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC19_ENABLE_SET_SHIFT       (0x00000013U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_SET_REG0_RAMECC19_ENABLE_SET_MAX         (0x00000001U)

/* SEC_ENABLE_CLR_REG0 */

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC0_ENABLE_CLR_MASK         (0x00000001U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC0_ENABLE_CLR_SHIFT        (0x00000000U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC0_ENABLE_CLR_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC1_ENABLE_CLR_MASK         (0x00000002U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC1_ENABLE_CLR_SHIFT        (0x00000001U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC1_ENABLE_CLR_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC2_ENABLE_CLR_MASK         (0x00000004U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC2_ENABLE_CLR_SHIFT        (0x00000002U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC2_ENABLE_CLR_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC3_ENABLE_CLR_MASK         (0x00000008U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC3_ENABLE_CLR_SHIFT        (0x00000003U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC3_ENABLE_CLR_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC4_ENABLE_CLR_MASK         (0x00000010U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC4_ENABLE_CLR_SHIFT        (0x00000004U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC4_ENABLE_CLR_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC5_ENABLE_CLR_MASK         (0x00000020U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC5_ENABLE_CLR_SHIFT        (0x00000005U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC5_ENABLE_CLR_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC6_ENABLE_CLR_MASK         (0x00000040U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC6_ENABLE_CLR_SHIFT        (0x00000006U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC6_ENABLE_CLR_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC7_ENABLE_CLR_MASK         (0x00000080U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC7_ENABLE_CLR_SHIFT        (0x00000007U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC7_ENABLE_CLR_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC8_ENABLE_CLR_MASK         (0x00000100U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC8_ENABLE_CLR_SHIFT        (0x00000008U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC8_ENABLE_CLR_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC9_ENABLE_CLR_MASK         (0x00000200U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC9_ENABLE_CLR_SHIFT        (0x00000009U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC9_ENABLE_CLR_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC10_ENABLE_CLR_MASK        (0x00000400U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC10_ENABLE_CLR_SHIFT       (0x0000000AU)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC10_ENABLE_CLR_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC11_ENABLE_CLR_MASK        (0x00000800U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC11_ENABLE_CLR_SHIFT       (0x0000000BU)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC11_ENABLE_CLR_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC12_ENABLE_CLR_MASK        (0x00001000U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC12_ENABLE_CLR_SHIFT       (0x0000000CU)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC12_ENABLE_CLR_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC13_ENABLE_CLR_MASK        (0x00002000U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC13_ENABLE_CLR_SHIFT       (0x0000000DU)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC13_ENABLE_CLR_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC14_ENABLE_CLR_MASK        (0x00004000U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC14_ENABLE_CLR_SHIFT       (0x0000000EU)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC14_ENABLE_CLR_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC15_ENABLE_CLR_MASK        (0x00008000U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC15_ENABLE_CLR_SHIFT       (0x0000000FU)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC15_ENABLE_CLR_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC16_ENABLE_CLR_MASK        (0x00010000U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC16_ENABLE_CLR_SHIFT       (0x00000010U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC16_ENABLE_CLR_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC17_ENABLE_CLR_MASK        (0x00020000U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC17_ENABLE_CLR_SHIFT       (0x00000011U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC17_ENABLE_CLR_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC18_ENABLE_CLR_MASK        (0x00040000U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC18_ENABLE_CLR_SHIFT       (0x00000012U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC18_ENABLE_CLR_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC19_ENABLE_CLR_MASK        (0x00080000U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC19_ENABLE_CLR_SHIFT       (0x00000013U)
#define CSL_CPSW2G_ECC_ECC_SEC_ENABLE_CLR_REG0_RAMECC19_ENABLE_CLR_MAX         (0x00000001U)

/* DED_EOI_REG */

#define CSL_CPSW2G_ECC_ECC_DED_EOI_REG_EOI_WR_MASK                             (0x00000001U)
#define CSL_CPSW2G_ECC_ECC_DED_EOI_REG_EOI_WR_SHIFT                            (0x00000000U)
#define CSL_CPSW2G_ECC_ECC_DED_EOI_REG_EOI_WR_MAX                              (0x00000001U)

/* DED_STATUS_REG0 */

#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC0_PEND_MASK                   (0x00000001U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC0_PEND_SHIFT                  (0x00000000U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC0_PEND_MAX                    (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC1_PEND_MASK                   (0x00000002U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC1_PEND_SHIFT                  (0x00000001U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC1_PEND_MAX                    (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC2_PEND_MASK                   (0x00000004U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC2_PEND_SHIFT                  (0x00000002U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC2_PEND_MAX                    (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC3_PEND_MASK                   (0x00000008U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC3_PEND_SHIFT                  (0x00000003U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC3_PEND_MAX                    (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC4_PEND_MASK                   (0x00000010U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC4_PEND_SHIFT                  (0x00000004U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC4_PEND_MAX                    (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC5_PEND_MASK                   (0x00000020U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC5_PEND_SHIFT                  (0x00000005U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC5_PEND_MAX                    (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC6_PEND_MASK                   (0x00000040U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC6_PEND_SHIFT                  (0x00000006U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC6_PEND_MAX                    (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC7_PEND_MASK                   (0x00000080U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC7_PEND_SHIFT                  (0x00000007U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC7_PEND_MAX                    (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC8_PEND_MASK                   (0x00000100U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC8_PEND_SHIFT                  (0x00000008U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC8_PEND_MAX                    (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC9_PEND_MASK                   (0x00000200U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC9_PEND_SHIFT                  (0x00000009U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC9_PEND_MAX                    (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC10_PEND_MASK                  (0x00000400U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC10_PEND_SHIFT                 (0x0000000AU)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC10_PEND_MAX                   (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC11_PEND_MASK                  (0x00000800U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC11_PEND_SHIFT                 (0x0000000BU)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC11_PEND_MAX                   (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC12_PEND_MASK                  (0x00001000U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC12_PEND_SHIFT                 (0x0000000CU)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC12_PEND_MAX                   (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC13_PEND_MASK                  (0x00002000U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC13_PEND_SHIFT                 (0x0000000DU)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC13_PEND_MAX                   (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC14_PEND_MASK                  (0x00004000U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC14_PEND_SHIFT                 (0x0000000EU)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC14_PEND_MAX                   (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC15_PEND_MASK                  (0x00008000U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC15_PEND_SHIFT                 (0x0000000FU)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC15_PEND_MAX                   (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC16_PEND_MASK                  (0x00010000U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC16_PEND_SHIFT                 (0x00000010U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC16_PEND_MAX                   (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC17_PEND_MASK                  (0x00020000U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC17_PEND_SHIFT                 (0x00000011U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC17_PEND_MAX                   (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC18_PEND_MASK                  (0x00040000U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC18_PEND_SHIFT                 (0x00000012U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC18_PEND_MAX                   (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC19_PEND_MASK                  (0x00080000U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC19_PEND_SHIFT                 (0x00000013U)
#define CSL_CPSW2G_ECC_ECC_DED_STATUS_REG0_RAMECC19_PEND_MAX                   (0x00000001U)

/* DED_ENABLE_SET_REG0 */

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC0_ENABLE_SET_MASK         (0x00000001U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC0_ENABLE_SET_SHIFT        (0x00000000U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC0_ENABLE_SET_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC1_ENABLE_SET_MASK         (0x00000002U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC1_ENABLE_SET_SHIFT        (0x00000001U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC1_ENABLE_SET_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC2_ENABLE_SET_MASK         (0x00000004U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC2_ENABLE_SET_SHIFT        (0x00000002U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC2_ENABLE_SET_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC3_ENABLE_SET_MASK         (0x00000008U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC3_ENABLE_SET_SHIFT        (0x00000003U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC3_ENABLE_SET_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC4_ENABLE_SET_MASK         (0x00000010U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC4_ENABLE_SET_SHIFT        (0x00000004U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC4_ENABLE_SET_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC5_ENABLE_SET_MASK         (0x00000020U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC5_ENABLE_SET_SHIFT        (0x00000005U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC5_ENABLE_SET_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC6_ENABLE_SET_MASK         (0x00000040U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC6_ENABLE_SET_SHIFT        (0x00000006U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC6_ENABLE_SET_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC7_ENABLE_SET_MASK         (0x00000080U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC7_ENABLE_SET_SHIFT        (0x00000007U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC7_ENABLE_SET_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC8_ENABLE_SET_MASK         (0x00000100U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC8_ENABLE_SET_SHIFT        (0x00000008U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC8_ENABLE_SET_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC9_ENABLE_SET_MASK         (0x00000200U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC9_ENABLE_SET_SHIFT        (0x00000009U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC9_ENABLE_SET_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC10_ENABLE_SET_MASK        (0x00000400U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC10_ENABLE_SET_SHIFT       (0x0000000AU)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC10_ENABLE_SET_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC11_ENABLE_SET_MASK        (0x00000800U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC11_ENABLE_SET_SHIFT       (0x0000000BU)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC11_ENABLE_SET_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC12_ENABLE_SET_MASK        (0x00001000U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC12_ENABLE_SET_SHIFT       (0x0000000CU)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC12_ENABLE_SET_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC13_ENABLE_SET_MASK        (0x00002000U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC13_ENABLE_SET_SHIFT       (0x0000000DU)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC13_ENABLE_SET_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC14_ENABLE_SET_MASK        (0x00004000U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC14_ENABLE_SET_SHIFT       (0x0000000EU)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC14_ENABLE_SET_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC15_ENABLE_SET_MASK        (0x00008000U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC15_ENABLE_SET_SHIFT       (0x0000000FU)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC15_ENABLE_SET_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC16_ENABLE_SET_MASK        (0x00010000U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC16_ENABLE_SET_SHIFT       (0x00000010U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC16_ENABLE_SET_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC17_ENABLE_SET_MASK        (0x00020000U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC17_ENABLE_SET_SHIFT       (0x00000011U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC17_ENABLE_SET_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC18_ENABLE_SET_MASK        (0x00040000U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC18_ENABLE_SET_SHIFT       (0x00000012U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC18_ENABLE_SET_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC19_ENABLE_SET_MASK        (0x00080000U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC19_ENABLE_SET_SHIFT       (0x00000013U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_SET_REG0_RAMECC19_ENABLE_SET_MAX         (0x00000001U)

/* DED_ENABLE_CLR_REG0 */

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC0_ENABLE_CLR_MASK         (0x00000001U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC0_ENABLE_CLR_SHIFT        (0x00000000U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC0_ENABLE_CLR_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC1_ENABLE_CLR_MASK         (0x00000002U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC1_ENABLE_CLR_SHIFT        (0x00000001U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC1_ENABLE_CLR_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC2_ENABLE_CLR_MASK         (0x00000004U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC2_ENABLE_CLR_SHIFT        (0x00000002U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC2_ENABLE_CLR_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC3_ENABLE_CLR_MASK         (0x00000008U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC3_ENABLE_CLR_SHIFT        (0x00000003U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC3_ENABLE_CLR_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC4_ENABLE_CLR_MASK         (0x00000010U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC4_ENABLE_CLR_SHIFT        (0x00000004U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC4_ENABLE_CLR_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC5_ENABLE_CLR_MASK         (0x00000020U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC5_ENABLE_CLR_SHIFT        (0x00000005U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC5_ENABLE_CLR_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC6_ENABLE_CLR_MASK         (0x00000040U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC6_ENABLE_CLR_SHIFT        (0x00000006U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC6_ENABLE_CLR_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC7_ENABLE_CLR_MASK         (0x00000080U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC7_ENABLE_CLR_SHIFT        (0x00000007U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC7_ENABLE_CLR_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC8_ENABLE_CLR_MASK         (0x00000100U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC8_ENABLE_CLR_SHIFT        (0x00000008U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC8_ENABLE_CLR_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC9_ENABLE_CLR_MASK         (0x00000200U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC9_ENABLE_CLR_SHIFT        (0x00000009U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC9_ENABLE_CLR_MAX          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC10_ENABLE_CLR_MASK        (0x00000400U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC10_ENABLE_CLR_SHIFT       (0x0000000AU)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC10_ENABLE_CLR_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC11_ENABLE_CLR_MASK        (0x00000800U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC11_ENABLE_CLR_SHIFT       (0x0000000BU)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC11_ENABLE_CLR_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC12_ENABLE_CLR_MASK        (0x00001000U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC12_ENABLE_CLR_SHIFT       (0x0000000CU)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC12_ENABLE_CLR_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC13_ENABLE_CLR_MASK        (0x00002000U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC13_ENABLE_CLR_SHIFT       (0x0000000DU)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC13_ENABLE_CLR_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC14_ENABLE_CLR_MASK        (0x00004000U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC14_ENABLE_CLR_SHIFT       (0x0000000EU)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC14_ENABLE_CLR_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC15_ENABLE_CLR_MASK        (0x00008000U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC15_ENABLE_CLR_SHIFT       (0x0000000FU)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC15_ENABLE_CLR_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC16_ENABLE_CLR_MASK        (0x00010000U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC16_ENABLE_CLR_SHIFT       (0x00000010U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC16_ENABLE_CLR_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC17_ENABLE_CLR_MASK        (0x00020000U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC17_ENABLE_CLR_SHIFT       (0x00000011U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC17_ENABLE_CLR_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC18_ENABLE_CLR_MASK        (0x00040000U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC18_ENABLE_CLR_SHIFT       (0x00000012U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC18_ENABLE_CLR_MAX         (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC19_ENABLE_CLR_MASK        (0x00080000U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC19_ENABLE_CLR_SHIFT       (0x00000013U)
#define CSL_CPSW2G_ECC_ECC_DED_ENABLE_CLR_REG0_RAMECC19_ENABLE_CLR_MAX         (0x00000001U)

/* AGGR_ENABLE_SET */

#define CSL_CPSW2G_ECC_ECC_AGGR_ENABLE_SET_PARITY_MASK                         (0x00000001U)
#define CSL_CPSW2G_ECC_ECC_AGGR_ENABLE_SET_PARITY_SHIFT                        (0x00000000U)
#define CSL_CPSW2G_ECC_ECC_AGGR_ENABLE_SET_PARITY_MAX                          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_AGGR_ENABLE_SET_TIMEOUT_MASK                        (0x00000002U)
#define CSL_CPSW2G_ECC_ECC_AGGR_ENABLE_SET_TIMEOUT_SHIFT                       (0x00000001U)
#define CSL_CPSW2G_ECC_ECC_AGGR_ENABLE_SET_TIMEOUT_MAX                         (0x00000001U)

/* AGGR_ENABLE_CLR */

#define CSL_CPSW2G_ECC_ECC_AGGR_ENABLE_CLR_PARITY_MASK                         (0x00000001U)
#define CSL_CPSW2G_ECC_ECC_AGGR_ENABLE_CLR_PARITY_SHIFT                        (0x00000000U)
#define CSL_CPSW2G_ECC_ECC_AGGR_ENABLE_CLR_PARITY_MAX                          (0x00000001U)

#define CSL_CPSW2G_ECC_ECC_AGGR_ENABLE_CLR_TIMEOUT_MASK                        (0x00000002U)
#define CSL_CPSW2G_ECC_ECC_AGGR_ENABLE_CLR_TIMEOUT_SHIFT                       (0x00000001U)
#define CSL_CPSW2G_ECC_ECC_AGGR_ENABLE_CLR_TIMEOUT_MAX                         (0x00000001U)

/* AGGR_STATUS_SET */

#define CSL_CPSW2G_ECC_ECC_AGGR_STATUS_SET_PARITY_MASK                         (0x00000003U)
#define CSL_CPSW2G_ECC_ECC_AGGR_STATUS_SET_PARITY_SHIFT                        (0x00000000U)
#define CSL_CPSW2G_ECC_ECC_AGGR_STATUS_SET_PARITY_MAX                          (0x00000003U)

#define CSL_CPSW2G_ECC_ECC_AGGR_STATUS_SET_TIMEOUT_MASK                        (0x0000000CU)
#define CSL_CPSW2G_ECC_ECC_AGGR_STATUS_SET_TIMEOUT_SHIFT                       (0x00000002U)
#define CSL_CPSW2G_ECC_ECC_AGGR_STATUS_SET_TIMEOUT_MAX                         (0x00000003U)

/* AGGR_STATUS_CLR */

#define CSL_CPSW2G_ECC_ECC_AGGR_STATUS_CLR_PARITY_MASK                         (0x00000003U)
#define CSL_CPSW2G_ECC_ECC_AGGR_STATUS_CLR_PARITY_SHIFT                        (0x00000000U)
#define CSL_CPSW2G_ECC_ECC_AGGR_STATUS_CLR_PARITY_MAX                          (0x00000003U)

#define CSL_CPSW2G_ECC_ECC_AGGR_STATUS_CLR_TIMEOUT_MASK                        (0x0000000CU)
#define CSL_CPSW2G_ECC_ECC_AGGR_STATUS_CLR_TIMEOUT_SHIFT                       (0x00000002U)
#define CSL_CPSW2G_ECC_ECC_AGGR_STATUS_CLR_TIMEOUT_MAX                         (0x00000003U)

#ifdef __cplusplus
}
#endif
#endif
