/* ==========================================================================
 * $File: //dwh/usb_iip/dev/software/DWC_usb3/driver/hw.h $
 * $Revision: #60 $
 * $Date: 2013/12/17 $
 * $Change: 2392669 $
 *
 * Synopsys SS USB3 Linux Software Driver and documentation (hereinafter,
 * "Software") is an Unsupported proprietary work of Synopsys, Inc. unless
 * otherwise expressly agreed to in writing between Synopsys and you.
 *
 * The Software IS NOT an item of Licensed Software or Licensed Product under
 * any End User Software License Agreement or Agreement for Licensed Product
 * with Synopsys or any supplement thereto. You are permitted to use and
 * redistribute this Software in source and binary forms, with or without
 * modification, provided that redistributions of source code must retain this
 * notice. You may not view, use, disclose, copy or distribute this file or
 * any information contained herein except pursuant to this license grant from
 * Synopsys. If you do not agree with this notice, including the disclaimer
 * below, then you are not authorized to use the Software.
 *
 * THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS" BASIS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 * ========================================================================== */

#ifndef _DWC_USB3_REGS_H_
#define _DWC_USB3_REGS_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file
 *
 * This file contains the data structures for accessing the DWC_usb3 core
 * registers and DMA descriptor fields.
 *
 * The application interfaces with the HS OTG core by reading from and
 * writing to the Control and Status Register (CSR) space through the
 * AHB Slave interface. These registers are 32 bits wide, and the
 * addresses are 32-bit-block aligned.
 * CSRs are classified as follows:
 * - Core Global Registers
 * - Device Global Registers
 * - Device Endpoint Specific Registers
 */


/****************************************************************************/
/* Core Global Registers */

/**
 * This enum represents the bit fields of the Core SoC Bus Configuration 0
 * Register (GSBUSCFG0).
 */
	/** Bus Burst Len			<i>Access: R_W</i>.
	 * - 0:   single
	 * - 1:   incr
	 * - 3:   incr4
	 * - 7:   incr8
	 * - 15:  incr16
	 * - 31:  incr32  (non-AHB mode only)
	 * - 63:  incr64  (non-AHB mode only)
	 * - 127: incr128 (non-AHB mode only)
	 * - 255: incr256 (non-AHB mode only)
	 */
#define DWC_SBUSCFG0_HBURSTLEN_BITS			0x000000ffU
#define DWC_SBUSCFG0_HBURSTLEN_SHIFT		0U

#define DWC_SBUSCFG0_INT_DMA_BURST_SINGLE	0U
#define DWC_SBUSCFG0_INT_DMA_BURST_INCR		1U
#define DWC_SBUSCFG0_INT_DMA_BURST_INCR4	3U
#define DWC_SBUSCFG0_INT_DMA_BURST_INCR8	7U
#define DWC_SBUSCFG0_INT_DMA_BURST_INCR16	15U
#define DWC_SBUSCFG0_INT_DMA_BURST_INCR32	31U
#define DWC_SBUSCFG0_INT_DMA_BURST_INCR64	63U
#define DWC_SBUSCFG0_INT_DMA_BURST_INCR128	127U
#define DWC_SBUSCFG0_INT_DMA_BURST_INCR256	255U

	/** Descriptor Write is Posted		<i>Access: R_W</i> */
#define DWC_SBUSCFG0_DES_WR_POST_BIT			0x00000100U
#define DWC_SBUSCFG0_DES_WR_POST_SHIFT			8U

	/** Data Write is Posted		<i>Access: R_W</i> */
#define DWC_SBUSCFG0_DAT_WR_POST_BIT			0x00000200U
#define DWC_SBUSCFG0_DAT_WR_POST_SHIFT			9U

	/** Descriptor Access is Big-Endian	<i>Access: R_W</i> */
#define DWC_SBUSCFG0_DES_BIG_END_BIT			0x00000400U
#define DWC_SBUSCFG0_DES_BIG_END_SHIFT			10U

	/** Data Access is Big-Endian		<i>Access: R_W</i> */
#define DWC_SBUSCFG0_DAT_BIG_END_BIT			0x00000800U
#define DWC_SBUSCFG0_DAT_BIG_END_SHIFT			11U

	/** Store and Forward Mode		<i>Access: R_W</i> */
#define DWC_SBUSCFG0_STORE_AND_FORWARD_BIT		0x00001000U
#define DWC_SBUSCFG0_STORE_AND_FORWARD_SHIFT	12U

	/** Force Single Request		<i>Access: R_W</i> */
#define DWC_SBUSCFG0_SING_REQ_BIT				0x00004000U
#define DWC_SBUSCFG0_SING_REQ_SHIFT				14U

	/** Descriptor Readback Enable		<i>Access: R_W</i> */
#define DWC_SBUSCFG0_READ_AFTER_WRITE_BIT		0x00008000U
#define DWC_SBUSCFG0_READ_AFTER_WRITE_SHIFT		15U

	/** Descriptor Write Request Info	<i>Access: R_W</i> */
#define DWC_SBUSCFG0_DES_WR_REQ_INFO_BITS		0x000f0000U
#define DWC_SBUSCFG0_DES_WR_REQ_INFO_SHIFT		16U

	/** Data Write Request Info		<i>Access: R_W</i> */
#define DWC_SBUSCFG0_DAT_WR_REQ_INFO_BITS		0x00f00000U
#define DWC_SBUSCFG0_DAT_WR_REQ_INFO_SHIFT		20U

	/** Descriptor Read Request Info	<i>Access: R_W</i> */
#define DWC_SBUSCFG0_DES_RD_REQ_INFO_BITS		0x0f000000U
#define DWC_SBUSCFG0_DES_RD_REQ_INFO_SHIFT		24U

	/** Data Read Request Info		<i>Access: R_W</i> */
#define DWC_SBUSCFG0_DAT_RD_REQ_INFO_BITS		0xf0000000U
#define DWC_SBUSCFG0_DAT_RD_REQ_INFO_SHIFT		28U


/**
 * This enum represents the bit fields of the Core SoC Bus Configuration 1
 * Register (GSBUSCFG1).
 */
	/** OCP Address Space For Descriptor	<i>Access: R_W</i> */
#define DWC_SBUSCFG1_DES_ADDR_SPC_BITS			0x0000000fU
#define DWC_SBUSCFG1_DES_ADDR_SPC_SHIFT			0U

	/** OCP Address Space For Data		<i>Access: R_W</i> */
#define DWC_SBUSCFG1_DAT_ADDR_SPC_BITS			0x000000f0U
#define DWC_SBUSCFG1_DAT_ADDR_SPC_SHIFT			4U

/**
 * This enum represents the bit fields of the Core Tx Threshold Control
 * Register (GTXTHRCFG).
 */
	/** Maximum Tx Burst Size		<i>Access: R_W</i> */
#define DWC_TXTHRCTL_USB_MAX_TX_BURST_SIZE_BITS		0x00ff0000U
#define DWC_TXTHRCTL_USB_MAX_TX_BURST_SIZE_SHIFT	16U

	/** Tx Multi-Packet Threshold Count	<i>Access: R_W</i> */
#define DWC_TXTHRCTL_USB_TX_PKT_CNT_BITS			0x1f000000U
#define DWC_TXTHRCTL_USB_TX_PKT_CNT_SHIFT			24U

	/** Tx Multi-Packet Threshold Enable	<i>Access: R_W</i> */
#define DWC_TXTHRCTL_USB_TX_PKT_CNT_EN_BIT			0x20000000U
#define DWC_TXTHRCTL_USB_TX_PKT_CNT_EN_SHIFT		29U

/**
 * This enum represents the bit fields of the Core Rx Threshold Control
 * Register (GRXTHRCFG).
 */
	/** Maximum Rx Burst Size		<i>Access: R_W</i> */
#define DWC_RXTHRCTL_USB_MAX_RX_BURST_SIZE_BITS		0x00f80000U
#define DWC_RXTHRCTL_USB_MAX_RX_BURST_SIZE_SHIFT	19U

	/** Rx Multi-Packet Threshold Count	<i>Access: R_W</i> */
#define DWC_RXTHRCTL_USB_RX_PKT_CNT_BITS			0x0f000000U
#define DWC_RXTHRCTL_USB_RX_PKT_CNT_SHIFT			24U

	/** Rx Multi-Packet Threshold Enable	<i>Access: R_W</i> */
#define DWC_RXTHRCTL_USB_RX_PKT_CNT_EN_BIT			0x20000000U
#define DWC_RXTHRCTL_USB_RX_PKT_CNT_EN_SHIFT		29U


/**
 * This enum represents the bit fields of the Core Control
 * Register (GCTL).
 */
	/** Disable Clock Gating		<i>Access: R_W</i> */
#define DWC_GCTL_DSBL_CLCK_GTNG_BIT			0x00000001U
#define DWC_GCTL_DSBL_CLCK_GTNG_SHIFT		0U

	/** Global Hibernation Enable		<i>Access: R_W</i> */
#define DWC_GCTL_GBL_HIBER_EN_BIT			0x00000002U
#define DWC_GCTL_GBL_HIBER_EN_SHIFT			1U

	/** Disable Scrambling			<i>Access: R_W</i> */
#define DWC_GCTL_DIS_SCRAMBLE_BIT			0x00000008U
#define DWC_GCTL_DIS_SCRAMBLE_SHIFT			3U

	/** Scale-down Mode			<i>Access: R_W</i> */
#define DWC_GCTL_SCALE_DOWN_BITS			0x00000030U
#define DWC_GCTL_SCALE_DOWN_SHIFT			4U

	/** RAM Clock Select			<i>Access: R_W</i> */
#define DWC_GCTL_RAM_CLK_SEL_BITS			0x000000c0U
#define DWC_GCTL_RAM_CLK_SEL_SHIFT			6U

	/** Debug Attach			<i>Access: R_W</i> */
#define DWC_GCTL_DEBUG_ATTACH_BIT			0x00000100U
#define DWC_GCTL_DEBUG_ATTACH_SHIFT			8U

	/** Disable U1/U2 Timer Scaledown	<i>Access: R_W</i> */
#define DWC_GCTL_U1U2_TIMER_SCALE_BIT		0x00000200U
#define DWC_GCTL_U1U2_TIMER_SCALE_SHIFT		9U

	/** SOF ITP SYNC			<i>Access: R_W</i> */
#define DWC_GCTL_SOFITPSYNC_BIT				0x00000400U
#define DWC_GCTL_SOFITPSYNC_SHIFT			10U

	/** Core Soft Reset			<i>Access: R_W</i> */
#define DWC_GCTL_CORE_SOFT_RST_BIT			0x00000800U
#define DWC_GCTL_CORE_SOFT_RST_SHIFT		11U

	/** Port Capability Direction		<i>Access: R_W</i> */
#define DWC_GCTL_PRT_CAP_DIR_BITS			0x00003000U
#define DWC_GCTL_PRT_CAP_DIR_SHIFT			12U

	/** Port Capability Values */
#define DWC_GCTL_PRT_CAP_HOST			1U
#define DWC_GCTL_PRT_CAP_DEVICE			2U
#define DWC_GCTL_PRT_CAP_OTG			3U

	/** Frame Scale Down			<i>Access: R_W</i> */
#define DWC_GCTL_FRMSCLDWN_BITS				0x0000c000U
#define DWC_GCTL_FRMSCLDWN_SHIFT			14U

	/** U2 Reset ECN			<i>Access: R_W</i> */
#define DWC_GCTL_U2RSTECN_BIT				0x00010000U
#define DWC_GCTL_U2RSTECN_SHIFT				16U

	/** Bypass SetAddress			<i>Access: R_W</i> */
#define DWC_GCTL_BYPSSETADDR_BIT			0x00020000U
#define DWC_GCTL_BYPSSETADDR_SHIFT			17U

	/** Master Filter Bypass		<i>Access: R_W</i> */
#define DWC_GCTL_MASTERFILTBYPASS_BIT		0x00040000U
#define DWC_GCTL_MASTERFILTBYPASS_SHIFT		18U

	/** Power Down Scale			<i>Access: R_W</i> */
#define DWC_GCTL_PWR_DN_SCALE_BITS			0xfff80000U
#define DWC_GCTL_PWR_DN_SCALE_SHIFT			19U

/**
 * This enum represents the bit fields of the Core Interrupt Mask
 * Register (GEVTEN).
 */
	/** ULPI Carkit Event Enable		<i>Access: R_W</i> */
#define DWC_GEVTEN_ULPI_CK_EVT_EN_BIT		0x00000001U
#define DWC_GEVTEN_ULPI_CK_EVT_SHIFT		0U

	/** I2C Event Enable			<i>Access: R_W</i> */
#define DWC_GEVTEN_I2C_EVT_EN_BIT			0x00000002U
#define DWC_GEVTEN_I2C_EVT_EN_SHIFT			1U

/**
 * This enum represents the bit fields of the Core Status
 * Register (GSTS).
 */
	/** Current Mode			<i>Access: RO</i>.
	 * - 0: Device Mode
	 * - 1: Host Mode
	 * - 2: DRD Mode
	 */
#define DWC_GSTS_CURMODE_BITS		0x00000003U
#define DWC_GSTS_CURMODE_SHIFT		0U

#define DWC_GSTS_DEVICE_MODE		0U
#define DWC_GSTS_HOST_MODE			1U
#define DWC_GSTS_DRD_MODE			2U

	/** Bus Error Address Valid		<i>Access: RO</i> */
#define DWC_GSTS_BUS_ERR_ADDR_VLD_BIT		0x00000010U
#define DWC_GSTS_BUS_ERR_ADDR_VLD_SHIFT		4U

	/** CSR Timeout */
#define DWC_GSTS_CSR_TIMEOUT_BIT			0x00000020U
#define DWC_GSTS_CSR_TIMEOUT_SHIFT			5U

	/** Device Interrupt Pending */
#define DWC_GSTS_DEV_EVT_PENDING_BIT		0x00000040U
#define DWC_GSTS_DEV_EVT_PENDING_SHIFT		6U

	/** Host Interrupt Pending */
#define DWC_GSTS_HOST_EVT_PENDING_BIT		0x00000080U
#define DWC_GSTS_HOST_EVT_PENDING_SHIFT		7U

	/** ADP Interrupt Pending */
#define DWC_GSTS_ADP_EVT_PENDING_BIT		0x00000100U
#define DWC_GSTS_ADP_EVT_PENDING_SHIFT		8U

	/** BC Interrupt Pending */
#define DWC_GSTS_BC_EVT_PENDING_BIT			0x00000200U
#define DWC_GSTS_BC_EVT_PENDING_SHIFT		9U

	/** OTG Interrupt Pending */
#define DWC_GSTS_OTG_EVT_PENDING_BIT		0x00000400U
#define DWC_GSTS_OTG_EVT_PENDING_SHIFT		10U

	/** SSIC Interrupt Pending */
#define DWC_GSTS_SSIC_IP_BIT		0x00000800U
#define DWC_GSTS_SSIC_IP_SHIFT		11U

	/** Current BELT Value			<i>Access: RO</i> */
#define DWC_GSTS_CBELT_BITS			0xfff00000U
#define DWC_GSTS_CBELT_SHIFT		20U

/**
 * This enum represents the bit fields of the Hardware Parameters 0
 * Register (GHWPARAMS0).
 */
#define DWC_HWP0_MODE_BITS			0x00000007U
#define DWC_HWP0_MODE_SHIFT			0U

#define DWC_HWP0_MBUS_TYPE_BITS		0x00000038U
#define DWC_HWP0_MBUS_TYPE_SHIFT	3U

#define DWC_HWP0_SBUS_TYPE_BITS		0x000000c0U
#define DWC_HWP0_SBUS_TYPE_SHIFT	6U

#define DWC_HWP0_MDWIDTH_BITS		0x0000ff00U
#define DWC_HWP0_MDWIDTH_SHIFT		8U

#define DWC_HWP0_SDWIDTH_BITS		0x00ff0000U
#define DWC_HWP0_SDWIDTH_SHIFT		16U

#define DWC_HWP0_AWIDTH_BITS		0x3f000000U
#define DWC_HWP0_AWIDTH_SHIFT		24U

/**
 * This enum represents the bit fields of the Hardware Parameters 1
 * Register (GHWPARAMS1).
 */
#define DWC_HWP1_IDWIDTH_M1_BITS		0x00000007U
#define DWC_HWP1_IDWIDTH_M1_SHIFT		0U

#define DWC_HWP1_BURSTWIDTH_M1_BITS		0x00000038U
#define DWC_HWP1_BURSTWIDTH_M1_SHIFT	3U

#define DWC_HWP1_DATAINFOWIDTH_BITS		0x000001c0U
#define DWC_HWP1_DATAINFOWIDTH_SHIFT	6U

#define DWC_HWP1_REQINFOWIDTH_BITS		0x00000e00U
#define DWC_HWP1_REQINFOWIDTH_SHIFT		9U

#define DWC_HWP1_ASPACEWIDTH_BITS		0x00007000U
#define DWC_HWP1_ASPACEWIDTH_SHIFT		12U

#define DWC_HWP1_DEV_NUM_INT_BITS		0x001f8000U
#define DWC_HWP1_DEV_NUM_INT_SHIFT		15U

#define DWC_HWP1_NUM_RAMS_BITS			0x00600000U
#define DWC_HWP1_NUM_RAMS_SHIFT			21U

#define DWC_HWP1_SPRAM_TYP_BIT			0x00800000U
#define DWC_HWP1_SPRAM_TYP_SHIFT		23U

#define DWC_HWP1_EN_PWROPT_BITS			0x03000000U
#define DWC_HWP1_EN_PWROPT_SHIFT		24U

#define DWC_EN_PWROPT_NONE				0U
#define DWC_EN_PWROPT_CLK_GATING_ONLY	1U
#define DWC_EN_PWROPT_HIBERNATION		2U

#define DWC_HWP1_MAC_PHY_CLKS_SYNC_BIT		0x04000000U
#define DWC_HWP1_MAC_PHY_CLKS_SYNC_SHIFT	26U

#define DWC_HWP1_MAC_RAM_CLKS_SYNC_BIT		0x08000000U
#define DWC_HWP1_MAC_RAM_CLKS_SYNC_SHIFT	27U

#define DWC_HWP1_RAM_BUS_CLKS_SYNC_BIT		0x10000000U
#define DWC_HWP1_RAM_BUS_CLKS_SYNC_SHIFT	28U

#define DWC_HWP1_RM_OPT_FEATURES_BIT		0x40000000U
#define DWC_HWP1_RM_OPT_FEATURES_SHIFT		30U

/**
 * This enum represents the bit fields of the Hardware Parameters 2
 * Register (GHWPARAMS2).
 */
#define DWC_HWP2_USERID_BITS	0xffffffffU
#define DWC_HWP2_USERID_SHIFT	0U

/**
 * This enum represents the bit fields of the Hardware Parameters 3
 * Register (GHWPARAMS3).
 */
#define DWC_HWP3_SSPHY_IFC_BITS			0x00000003U
#define DWC_HWP3_SSPHY_IFC_SHIFT		0U

#define DWC_HWP3_HSPHY_IFC_BITS			0x0000000cU
#define DWC_HWP3_HSPHY_IFC_SHIFT		2U

#define DWC_HWP3_FSPHY_IFC_BITS			0x00000030U
#define DWC_HWP3_FSPHY_IFC_SHIFT		4U

#define DWC_HWP3_HSPHY_DWIDTH_BITS		0x000000c0U
#define DWC_HWP3_HSPHY_DWIDTH_SHIFT		6U

#define DWC_HWP3_VEND_CTL_IFC_BIT		0x00000400U
#define DWC_HWP3_VEND_CTL_IFC_SHIFT		10U

#define DWC_HWP3_ULPI_CARKIT_BIT		0x00000800U
#define DWC_HWP3_ULPI_CARKIT_SHIFT		11U

#define DWC_HWP3_NUM_EPS_BITS			0x0003f000U
#define DWC_HWP3_NUM_EPS_SHIFT			12U

#define DWC_HWP3_NUM_IN_EPS_BITS		0x007c0000U
#define DWC_HWP3_NUM_IN_EPS_SHIFT		18U

#define DWC_HWP3_TOT_XFR_RSRC_BITS		0x7f800000U
#define DWC_HWP3_TOT_XFR_RSRC_SHIFT		23U

/**
 * This enum represents the bit fields of the Hardware Parameters 4
 * Register (GHWPARAMS4).
 */

#define DWC_HWP4_TRBS_PER_XFER_BITS			0x0000003fU
#define DWC_HWP4_TRBS_PER_XFER_SHIFT		0U

#define DWC_HWP4_HIBER_SPAD_BITS			0x0001e000U
#define DWC_HWP4_HIBER_SPAD_SHIFT			13U

#define DWC_HWP4_NUM_SS_USB_INST_BITS		0x001e0000U
#define DWC_HWP4_NUM_SS_USB_INST_SHIFT		17U

#define DWC_HWP4_EN_ISOC_SUPT_BIT			0x00800000U
#define DWC_HWP4_EN_ISOC_SUPT_SHIFT			23U

#define DWC_HWP4_BMU_PTL_DEPTH_BITS			0x0f000000U
#define DWC_HWP4_BMU_PTL_DEPTH_SHIFT		24U

#define DWC_HWP4_BMU_LSP_DEPTH_BITS			0xf0000000U
#define DWC_HWP4_BMU_LSP_DEPTH_SHIFT		28U

/**
 * This enum represents the bit fields of the Hardware Parameters 5
 * Register (GHWPARAMS5).
 */
#define DWC_HWP5_BMU_BUSGM_DEPTH_BITS		0x0000000fU
#define DWC_HWP5_BMU_BUSGM_DEPTH_SHIFT		0U

#define DWC_HWP5_RXQ_FIFO_DEPTH_BITS		0x000003f0U
#define DWC_HWP5_RXQ_FIFO_DEPTH_SHIFT		4U

#define DWC_HWP5_TXQ_FIFO_DEPTH_BITS		0x0000fc00U
#define DWC_HWP5_TXQ_FIFO_DEPTH_SHIFT		10U

#define DWC_HWP5_DWQ_FIFO_DEPTH_BITS		0x003f0000U
#define DWC_HWP5_DWQ_FIFO_DEPTH_SHIFT		16U

#define DWC_HWP5_DFQ_FIFO_DEPTH_BITS		0x0fc00000U
#define DWC_HWP5_DFQ_FIFO_DEPTH_SHIFT		22U

/**
 * This enum represents the bit fields of the Hardware Parameters 6
 * Register (GHWPARAMS6).
 */
#define DWC_HWP6_PSQ_FIFO_DEPTH_BITS	0x0000003fU
#define DWC_HWP6_PSQ_FIFO_DEPTH_SHIFT	0U

#define DWC_HWP6_EN_DBG_PORTS_BIT		0x00000040U
#define DWC_HWP6_EN_DBG_PORTS_SHIFT		6U

#define DWC_HWP6_EN_FPGA_BIT			0x00000080U
#define DWC_HWP6_EN_FPGA_SHIFT			7U

#define DWC_HWP6_EN_SRP_BIT				0x00000400U
#define DWC_HWP6_EN_SRP_SHIFT			10U

#define DWC_HWP6_EN_HNP_BIT				0x00000800U
#define DWC_HWP6_EN_HNP_SHIFT			11U

#define DWC_HWP6_EN_ADP_BIT				0x00001000U
#define DWC_HWP6_EN_ADP_SHIFT			12U

#define DWC_HWP6_EN_OTG_BIT				0x00002000U
#define DWC_HWP6_EN_OTG_SHIFT			13U

#define DWC_HWP6_EN_BC_BIT				0x00004000U
#define DWC_HWP6_EN_BC_SHIFT			14U

#define DWC_HWP6_EN_BUS_FILTERS_BIT		0x00008000U
#define DWC_HWP6_EN_BUS_FILTERS_SHIFT	15U

#define DWC_HWP6_RAM0_DEPTH_BITS		0xffff0000U
#define DWC_HWP6_RAM0_DEPTH_SHIFT		16U

/**
 * This enum represents the bit fields of the Hardware Parameters 7
 * Register (GHWPARAMS7).
 */
#define DWC_HWP7_RAM1_DEPTH_BITS		0x0000ffffU
#define DWC_HWP7_RAM1_DEPTH_SHIFT		0U

#define DWC_HWP7_RAM2_DEPTH_BITS		0xffff0000U
#define DWC_HWP7_RAM2_DEPTH_SHIFT		16U

/**
 * This enum represents the bit fields of the Hardware Parameters 8
 * Register (GHWPARAMS8).
 */
#define DWC_HWP8_DCACHE_DEPTH_BITS		0xffffffffU
#define DWC_HWP8_DCACHE_DEPTH_SHIFT		0U

/**
 * This enum represents the bit fields of the Debug Queue/FIFO Space
 * Register (GDBGFIFOSPACE).
 */
	/** FIFO/Queue Select			<i>Access: R_W</i> */
#define DWC_DBGFIFOSPACE_FIFO_QUEUE_SEL_BITS	0x000000ffU
#define DWC_DBGFIFOSPACE_FIFO_QUEUE_SEL_SHIFT	0U

	/*   0 - 31  TxFIFO Number   */
	/*  32 - 63  RxFIFO Number   */
	/*  64 - 95  TxReqQ Number   */
	/*  96 - 127 RxReqQ Number   */
	/* 128 - 159 RxInfoQ Number  */
	/* 160       DescFetchQ      */
	/* 161       EventQ          */
	/* 162       ProtocolStatusQ */

	/** Space Available			<i>Access: R</i> */
#define DWC_DBGFIFOSPACE_SPACE_AVAIL_BITS	0xffff0000U
#define DWC_DBGFIFOSPACE_SPACE_AVAIL_SHIFT	16U

/**
 * This enum represents the bit fields of the Debug LTSSM
 * Register (GDBGLTSSM).
 */
	/** Pipe Status				<i>Access: R</i> */
#define DWC_DBGLTSSM_PIPE_STATUS_BITS		0x0003ffffU
#define DWC_DBGLTSSM_PIPE_STATUS_SHIFT		0U

	/** LTDB SubState			<i>Access: R</i> */
#define DWC_DBGLTSSM_LTDB_SUB_STATE_BITS	0x003c0000U
#define DWC_DBGLTSSM_LTDB_SUB_STATE_SHIFT	18U

	/** LTDB State				<i>Access: R</i> */
#define DWC_DBGLTSSM_LTDB_STATE_BITS		0x03c00000U
#define DWC_DBGLTSSM_LTDB_STATE_SHIFT		22U

	/** LTDB Timeout			<i>Access: R</i> */
#define DWC_DBGLTSSM_LTDB_TIMEOUT_BIT		0x04000000U
#define DWC_DBGLTSSM_LTDB_TIMEOUT_SHIFT		26U

/**
 * This enum represents the bit fields of the Core RMMI PHY Control
 * Registers (GUSB3RMMICTLn).
 */

#define DWC_RMMICTL_MPHY_STATE_BITS		0x0e000000U
#define DWC_RMMICTL_MPHY_STATE_SHIFT	25U

#define DWC_MPHY_STATE_DISABLED		0U
#define DWC_MPHY_STATE_HIBERN8		1U
#define DWC_MPHY_STATE_SLEEP		2U
#define DWC_MPHY_STATE_STALL		3U
#define DWC_MPHY_STATE_PWM_BURST	4U
#define DWC_MPHY_STATE_HS_BURST		5U
#define DWC_MPHY_STATE_LINE_CFG		6U
#define DWC_MPHY_STATE_LINE_RESET	7U

#define DWC_RMMICTL_AUTO_EXIT_RRAP_BIT		0x10000000U
#define DWC_RMMICTL_AUTO_EXIT_RRAP_SHIFT	28U

#define DWC_RMMICTL_AUTO_ROM_RRAP_BIT		0x20000000U
#define DWC_RMMICTL_AUTO_ROM_RRAP_SHIFT		29U

#define DWC_RMMICTL_AUTO_EXIT_H8_BIT		0x40000000U
#define DWC_RMMICTL_AUTO_EXIT_H8_SHIFT		30U

#define DWC_RMMICTL_AUTO_ROM_H8_BIT			0x80000000U
#define DWC_RMMICTL_AUTO_ROM_H8_SHIFT		31U

/**
 * This enum represents the bit fields of the Core USB2 PHY Configuration
 * Registers (GUSB2PHYCFGn).
 */
	/** HS/FS Timeout Calibration			<i>Access: R_W</i> */
#define DWC_USB2PHYCFG_TOUT_CAL_BITS		0x00000007U
#define DWC_USB2PHYCFG_TOUT_CAL_SHIFT		0U

	/** UTMI+ PHY Intf Width (8-bit/16-bit) SelecT	<i>Access: R_W</i> */
#define DWC_USB2PHYCFG_16B_PHY_IF_BIT		0x00000008U
#define DWC_USB2PHYCFG_16B_PHY_IF_SHIFT		3U
	/*--------*/
	/** ULPI DDR Select				<i>Access: R_W</i> */
#define DWC_USB2PHYCFG_DDR_SEL_BIT			0x00000008U
#define DWC_USB2PHYCFG_DDR_SEL_SHIFT		3U

	/** UTMI+ / ULPI Select				<i>Access: R_W</i> */
#define DWC_USB2PHYCFG_UTMI_ULPI_BIT		0x00000010U
#define DWC_USB2PHYCFG_UTMI_ULPI_SHIFT		4U

	/** Full-speed Serial Interface Select		<i>Access: R_W</i> */
#define DWC_USB2PHYCFG_FSINTF_BIT			0x00000020U
#define DWC_USB2PHYCFG_FSINTF_SHIFT			5U

	/** Suspend USB2 Phy				<i>Access: R_W</i> */
#define DWC_USB2PHYCFG_SUS_PHY_BIT			0x00000040U
#define DWC_USB2PHYCFG_SUS_PHY_SHIFT		6U

	/** USB2.0 HS PHY/USB1.1 FS Serial Xcvr Select	<i>Access: R_W</i> */
#define DWC_USB2PHYCFG_PHY_SEL_BIT			0x00000080U
#define DWC_USB2PHYCFG_PHY_SEL_SHIFT		7U

	/** Enable UTMI Sleep				<i>Access: R_W</i> */
#define DWC_USB2PHYCFG_ENBL_SLP_M_BIT		0x00000100U
#define DWC_USB2PHYCFG_ENBL_SLP_M_SHIFT		8U

	/** USB2.0 Turnaround Time			<i>Access: R_W</i> */
#define DWC_USB2PHYCFG_USB_TRD_TIM_BITS		0x00003c00U
#define DWC_USB2PHYCFG_USB_TRD_TIM_SHIFT	10U

	/** PHY Low-power Clock Select			<i>Access: R_W</i> */
#define DWC_USB2PHYCFG_PHY_LPWR_CLK_SEL_BIT		0x00004000U
#define DWC_USB2PHYCFG_PHY_LPWR_CLK_SEL_SHIFT	14U

	/** ULPI Auto Resume				<i>Access: R_W</i> */
#define DWC_USB2PHYCFG_ULPI_AUTO_RES_BIT		0x0000800U
#define DWC_USB2PHYCFG_ULPI_AUTO_RES_SHIFT		15U

	/** ULPI Clock SuspendM				<i>Access: R_W</i> */
#define DWC_USB2PHYCFG_ULPI_CLK_SUS_M_BIT		0x00010000U
#define DWC_USB2PHYCFG_ULPI_CLK_SUS_M_SHIFT		16U

	/** ULPI External Vbus Drive			<i>Access: R_W</i> */
#define DWC_USB2PHYCFG_ULPI_EXT_VBUS_DRV_BIT	 0x00020000U
#define DWC_USB2PHYCFG_ULPI_EXT_VBUS_DRV_SHIFT	 17U

	/** ULPI External Vbus Indicator		<i>Access: R_W</i> */
#define DWC_USB2PHYCFG_ULPI_EXT_VBUS_IND_BIT	 0x00040000U
#define DWC_USB2PHYCFG_ULPI_EXT_VBUS_IND_SHIFT	 18U

	/** PHY Interrupt Number			<i>Access: R_W</i> */
#define DWC_USB2PHYCFG_PHY_INTR_NUM_BITS	 0x01f80000U
#define DWC_USB2PHYCFG_PHY_INTR_NUM_SHIFT	 19U

	/** OTG Interrupt Number			<i>Access: R_W</i> */
#define DWC_USB2PHYCFG_OTG_INTR_NUM_BITS	 0x7e000000U
#define DWC_USB2PHYCFG_OTG_INTR_NUM_SHIFT	 25U

	/** PHY Soft Reset				<i>Access: R_W</i> */
#define DWC_USB2PHYCFG_PHY_SOFT_RST_BIT		 0x80000000U
#define DWC_USB2PHYCFG_PHY_SOFT_RST_SHIFT	 31U

/**
 * This enum represents the bit fields in the USB2 I2C Control
 * Registers (GUSB2I2CCTLn).
 */
	/** All bits are reserved */
#define DWC_USB2I2C_RSVD_BITS		0xffffffffU
#define DWC_USB2I2C_RSVD_SHIFT		0U

/**
 * This enum represents the bit fields in the USB2 Phy Vendor Control
 * Registers (GUSB2PHYACCn).
 */
	/** Register Data			<i>Access: R_W</i> */
#define DWC_USB2PHY_REGDATA_BITS			0x000000ffU
#define DWC_USB2PHY_REGDATA_SHIFT			0U

	/** UTMI+ Vendor Ctrl Register Address	<i>Access: R_W</i> */
#define DWC_USB2PHY_VCTRL_BITS				0x0000ff00U
#define DWC_USB2PHY_VCTRL_SHIFT				8U
	/*--------*/
	/** ULPI Extended Register Address	<i>Access: R_W</i> */
#define DWC_USB2PHY_EXTREGADDR_BITS			0x00003f00U
#define DWC_USB2PHY_EXTREGADDR_SHIFT		8U

	/** Register Address			<i>Access: R_W</i> */
#define DWC_USB2PHY_REGADDR_BITS			0x003f0000U
#define DWC_USB2PHY_REGADDR_SHIFT			16U

	/** Register Write			<i>Access: R_W</i> */
#define DWC_USB2PHY_REGWR_BIT				0x00400000U
#define DWC_USB2PHY_REGWR_SHIFT				22U

	/** VStatus Busy			<i>Access: RO</i> */
#define DWC_USB2PHY_VSTSBSY_BIT				0x00800000U
#define DWC_USB2PHY_VSTSBSY_SHIFT			23U

	/** VStatus Done			<i>Access: R_SS_SC</i> */
#define DWC_USB2PHY_VSTSDONE_BIT			0x01000000U
#define DWC_USB2PHY_VSTSDONE_SHIFT			24U

	/** New Register Request		<i>Access: R_WS_SC</i> */
#define DWC_USB2PHY_NEWREGREQ_BIT			0x02000000U
#define DWC_USB2PHY_NEWREGREQ_SHIFT			25U

	/** Disable ULPI Drivers		<i>Access: R_WS_SC</i> */
#define DWC_USB2PHY_DIS_ULPI_DRVR_BIT		0x04000000U
#define DWC_USB2PHY_DIS_ULPI_DRVR_SHIFT		26U

/**
 * This enum represents the bit fields of the USB3 Pipe Control
 * Registers (GUSB3PIPECTLn).
 */
	/** Elastic Buffer Mode			<i>Access: R_W</i> */
#define DWC_PIPECTL_ELAS_BUF_MODE_BIT		0x00000001U
#define DWC_PIPECTL_ELAS_BUF_MODE_SHIFT		0U

	/** Tx De-Emphasis			<i>Access: R_W</i> */
#define DWC_PIPECTL_TX_DEMPH_BITS			0x00000006U
#define DWC_PIPECTL_TX_DEMPH_SHIFT			1U

	/** Tx Margin				<i>Access: R_W</i> */
#define DWC_PIPECTL_TX_MARGIN_BITS			0x00000038U
#define DWC_PIPECTL_TX_MARGIN_SHIFT			3U

	/** Tx Swing				<i>Access: R_W</i> */
#define DWC_PIPECTL_TX_SWING_BIT			0x00000040U
#define DWC_PIPECTL_TX_SWING_SHIFT			6U

	/** USB3 SSIC Enable			<i>Access: R_W</i> */
#define DWC_PIPECTL_SSIC_EN_BIT				0x00000080U
#define DWC_PIPECTL_SSIC_EN_SHIFT			7U

	/** LFPS Filter				<i>Access: R_W</i> */
#define DWC_PIPECTL_LFPS_FILTER_BIT			0x00000200U
#define DWC_PIPECTL_LFPS_FILTER_SHIFT		9U

	/** P3 Exit Signal In P2		<i>Access: R_W</i> */
#define DWC_PIPECTL_P3_EX_SIG_P2_BIT		0x00000400U
#define DWC_PIPECTL_P3_EX_SIG_P2_SHIFT		10U

	/** P3-P2 Transitions OK		<i>Access: R_W</i> */
#define DWC_PIPECTL_P3_P2_TRAN_OK_BIT		0x00000800U
#define DWC_PIPECTL_P3_P2_TRAN_OK_SHIFT		11U

	/** LFPS P0 Align			<i>Access: R_W</i> */
#define DWC_PIPECTL_LFPS_P0_ALGN_BIT		0x00001000U
#define DWC_PIPECTL_LFPS_P0_ALGN_SHIFT		12U

	/** Pipe Data Width			<i>Access: R_W</i> */
#define DWC_PIPECTL_DATA_WIDTH_BITS			0x00018000U
#define DWC_PIPECTL_DATA_WIDTH_SHIFT		15U

	/** Suspend USB3 Phy			<i>Access: R_W</i> */
#define DWC_PIPECTL_SUS_PHY_BIT				0x00020000U
#define DWC_PIPECTL_SUS_PHY_SHIFT			17U

	/** PHY Soft Reset			<i>Access: R_W</i> */
#define DWC_PIPECTL_PHY_SOFT_RST_BIT		0x80000000U
#define DWC_PIPECTL_PHY_SOFT_RST_SHIFT		31U

/**
 * This enum represents the bit fields in the FIFO Size Registers.
 */
	/** Depth				<i>Access: R_W</i> */
#define DWC_FIFOSZ_DEPTH_BITS			0x0000ffffU
#define DWC_FIFOSZ_DEPTH_SHIFT			0U

	/** Starting Address			<i>Access: RO or R_W</i> */
#define DWC_FIFOSZ_STARTADDR_BITS		0xffff0000U
#define DWC_FIFOSZ_STARTADDR_SHIFT		16U

/**
 * This enum represents the bit fields of the Event Buffer Size
 * Registers (GEVENTSIZn).
 */
	/** Event Buffer Size			<i>Access: R_W</i> */
#define DWC_EVENTSIZ_SIZ_BITS			0x0000ffffU
#define DWC_EVENTSIZ_SIZ_SHIFT			0U

	/** Event Interrupt Mask (1 == disable)	<i>Access: R_W</i> */
#define DWC_EVENTSIZ_INT_MSK_BIT		0x80000000U
#define DWC_EVENTSIZ_INT_MSK_SHIFT		31U

/**
 * This enum represents the bit fields of the Event Buffer Count
 * Registers (GEVENTCNTn).
 */
	/** Event Count				<i>Access: R_W</i> */
#define DWC_EVENTCNT_CNT_BITS			0x0000ffffU
#define DWC_EVENTCNT_CNT_SHIFT			0U

/**
 * This enum represents the bit fields of a generic Event Buffer entry.
 */
	/** Non-Endpoint Specific Event flag */
#define DWC_EVENT_NON_EP_BIT			0x01U
#define DWC_EVENT_NON_EP_SHIFT			0U

	/** Non-Endpoint Specific Event Type */
#define DWC_EVENT_INTTYPE_BITS			0xfeU
#define DWC_EVENT_INTTYPE_SHIFT			1U

	/** Non-Endpoint Specific Event Type values */
#define DWC_EVENT_DEV_INT		0U	/** @< */
#define DWC_EVENT_OTG_INT		1U	/** @< */
#define DWC_EVENT_CARKIT_INT	3U	/** @< */
#define DWC_EVENT_I2C_INT		4U

/**
 * This enum represents the non-generic bit fields of an Event Buffer entry
 * for Device Specific events (DEVT).
 */
	/** Device Specific Event Type */
#define DWC_DEVT_BITS		0x00000f00U
#define DWC_DEVT_SHIFT		8U

	/** Device Specific Event Type values */
#define DWC_DEVT_DISCONN				0U	/** @< */
#define DWC_DEVT_USBRESET				1U	/** @< */
#define DWC_DEVT_CONNDONE				2U	/** @< */
#define DWC_DEVT_ULST_CHNG				3U	/** @< */
#define DWC_DEVT_WKUP					4U	/** @< */
#define DWC_DEVT_HIBER_REQ				5U	/** @< */
#define DWC_DEVT_U3_L2L1_SUSP			6U	/** @< */
#define DWC_DEVT_SOF					7U	/** @< */
#define DWC_DEVT_ERRATICERR				9U	/** @< */
#define DWC_DEVT_CMD_CMPL				10U	/** @< */
#define DWC_DEVT_OVERFLOW				11U	/** @< */
#define DWC_DEVT_VNDR_DEV_TST_RCVD		12U	/** @< */
#define DWC_DEVT_INACT_TIMEOUT_RCVD		13U

	/** Event Information */
#define DWC_DEVT_EVT_INFO_BITS			0xffff0000U
#define DWC_DEVT_EVT_INFO_SHIFT			16U

	/** USB/Link State */
#define DWC_DEVT_ULST_STATE_BITS		0x000f0000U
#define DWC_DEVT_ULST_STATE_SHIFT		16U

	/** USB/Link State values in SS */
#define DWC_LINK_STATE_U0			0U	/** @< */
#define DWC_LINK_STATE_U1			1U	/** @< */
#define DWC_LINK_STATE_U2			2U	/** @< */
#define DWC_LINK_STATE_U3			3U	/** @< */
#define DWC_LINK_STATE_SS_DIS		4U	/** @< */
#define DWC_LINK_STATE_RX_DET		5U	/** @< */
#define DWC_LINK_STATE_SS_INACT		6U	/** @< */
#define DWC_LINK_STATE_POLL			7U	/** @< */
#define DWC_LINK_STATE_RECOV		8U	/** @< */
#define DWC_LINK_STATE_HRESET		9U	/** @< */
#define DWC_LINK_STATE_CMPLY		10U	/** @< */
#define DWC_LINK_STATE_LPBK			11U	/** @< */
#define DWC_LINK_STATE_RESET		14U	/** @< */
#define DWC_LINK_STATE_RESUME		15U

	/** USB/Link State values in HS/FS/LS */
#define DWC_LINK_STATE_ON				0U	/** @< */
#define DWC_LINK_STATE_SLEEP			2U	/** @< */
#define DWC_LINK_STATE_SUSPEND			3U	/** @< */
#define DWC_LINK_STATE_EARLY_SUSPEND	5U

#define DWC_DEVT_ULST_SS_BIT			0x00100000U
#define DWC_DEVT_ULST_SS_SHIFT			20U

#define DWC_DEVT_HIBER_STATE_BITS		DWC_DEVT_ULST_STATE_BITS
#define DWC_DEVT_HIBER_STATE_SHIFT		DWC_DEVT_ULST_STATE_SHIFT

#define DWC_DEVT_HIBER_SS_BIT			DWC_DEVT_ULST_SS_BIT
#define DWC_DEVT_HIBER_SS_SHIFT			DWC_DEVT_ULST_SS_SHIFT

#define DWC_DEVT_HIBER_HIRD_BITS		0x0f000000U
#define DWC_DEVT_HIBER_HIRD_SHIFT		24U

/**
 * This enum represents the bit fields of an Event Buffer entry for
 * Endpoint Specific events (DEPEVT).
 */
	/** Endpoint Number */
#define DWC_DEPEVT_EPNUM_BITS			0x0000003eU
#define DWC_DEPEVT_EPNUM_SHIFT			1U

	/** Endpoint Event Type */
#define DWC_DEPEVT_INTTYPE_BITS			0x000003c0U
#define DWC_DEPEVT_INTTYPE_SHIFT		6U

	/** Endpoint Event Type values */
#define DWC_DEPEVT_XFER_CMPL		1U	/** @< */
#define DWC_DEPEVT_XFER_IN_PROG		2U	/** @< */
#define DWC_DEPEVT_XFER_NRDY		3U	/** @< */
#define DWC_DEPEVT_FIFOXRUN			4U	/** @< */
#define DWC_DEPEVT_STRM_EVT			6U	/** @< */
#define DWC_DEPEVT_EPCMD_CMPL		7U

	/** Event Status for Start Xfer Command */
#define DWC_DEPEVT_NO_MORE_RSCS_BIT			0x00001000U
#define DWC_DEPEVT_NO_MORE_RSCS_SHIFT		12U
#define DWC_DEPEVT_ISOC_TIME_PASSED_BIT		0x00002000U
#define DWC_DEPEVT_ISOC_TIME_PASSED_SHIFT	13U

	/** Event Status for Stream Event */
#define DWC_DEPEVT_STRM_EVT_BITS			0x0000f000U
#define DWC_DEPEVT_STRM_EVT_SHIFT			12U

	/** Stream Event Status values */
#define DWC_DEPEVT_STRM_FOUND		1U	/** @< */
#define DWC_DEPEVT_STRM_NOT_FOUND	2U

	/** Event Status for Xfer Complete or Xfer In Progress Event */
#define DWC_DEPEVT_BUS_ERR_BIT			0x00001000U
#define DWC_DEPEVT_BUS_ERR_SHIFT		12U
#define DWC_DEPEVT_SHORT_PKT_BIT		0x00002000U
#define DWC_DEPEVT_SHORT_PKT_SHIFT		13U
#define DWC_DEPEVT_IOC_BIT				0x00004000U
#define DWC_DEPEVT_IOC_SHIFT			14U
#define DWC_DEPEVT_LST_BIT				0x00008000U
#define DWC_DEPEVT_LST_SHIFT			15U
#define DWC_DEPEVT_MISSED_ISOC_BIT		DWC_DEPEVT_LST_BIT
#define DWC_DEPEVT_MISSED_ISOC_SHIFT	DWC_DEPEVT_LST_SHIFT

	/** Event Status for Xfer Not Ready Event */
#define DWC_DEPEVT_CTRL_BITS			0x00003000U
#define DWC_DEPEVT_CTRL_SHIFT			12U
#define DWC_DEPEVT_XFER_ACTIVE_BIT		0x00008000U
#define DWC_DEPEVT_XFER_ACTIVE_SHIFT	15U

	/** Xfer Not Ready Event Status values */
#define DWC_DEPEVT_CTRL_SETUP		0U	/** @< */
#define DWC_DEPEVT_CTRL_DATA		1U	/** @< */
#define DWC_DEPEVT_CTRL_STATUS		2U

	/** Stream ID */
#define DWC_DEPEVT_STRM_ID_BITS				0xffff0000U
#define DWC_DEPEVT_STRM_ID_SHIFT			16U

	/** Isoc uFrame Number (for Xfer Not Ready on Isoc EP) */
#define DWC_DEPEVT_ISOC_UFRAME_NUM_BITS		0xffff0000U
#define DWC_DEPEVT_ISOC_UFRAME_NUM_SHIFT	16U

	/** Xfer Resource Index (for Start Xfer Command) */
#define DWC_DEPEVT_XFER_RSC_IDX_BITS		0x007f0000U
#define DWC_DEPEVT_XFER_RSC_IDX_SHIFT		16U

	/** Current Data Sequence Number (for Get Endpoint State Command) */
#define DWC_DEPEVT_CUR_DAT_SEQ_NUM_BITS		0x001f0000U
#define DWC_DEPEVT_CUR_DAT_SEQ_NUM_SHIFT	16U

	/** Flow Control State (for Get Endpoint State Command) */
#define DWC_DEPEVT_FLOW_CTRL_BIT			0x00200000U
#define DWC_DEPEVT_FLOW_CTRL_SHIFT			21U

/**
 * This enum represents the non-generic bit fields of an Event Buffer entry
 * for other Core events (GEVT).
 */
	/** PHY Port Number */
#define DWC_GINT_PHY_PORT_BITS		0xf00U
#define DWC_GINT_PHY_PORT_SHIFT		8U

/**
 * This struct represents the 32-bit register fields of the Event Buffer
 * Registers (GEVENTBUFn).
 */
typedef struct geventbuf_data {
	/** Event Buffer Address Register Low Word */
	volatile u32 geventadr_lo;

	/** Event Buffer Address Register High Word */
	volatile u32 geventadr_hi;

	/** Event Buffer Size Register.
	 * Fields defined in enum @ref geventsiz_data. */
	volatile u32 geventsiz;

	/** Event Buffer Count Register.
	 * Fields defined in enum @ref geventcnt_data. */
	volatile u32 geventcnt;
} geventbuf_data_t;

/**
 * Core Global Registers	<i>Offsets 100h-5FCh</i>.
 *
 * The dwc_usb3_core_global_regs structure defines the size
 * and relative field offsets for the Core Global Registers.
 */
typedef struct dwc_usb3_core_global_regs {

#define DWC_CORE_GLOBAL_REG_OFFSET	0x100U

	/** Core BIU Configuration 0 Register	<i>Offset: 100h</i>.
	 * Fields defined in enum @ref gsbuscfg0_data. */
	volatile u32 gsbuscfg0;

	/** Core BIU Configuration 1 Register	<i>Offset: 104h</i>.
	 * Fields defined in enum @ref gsbuscfg1_data. */
	volatile u32 gsbuscfg1;

	/** Core Tx Threshold Control Register	<i>Offset: 108h</i>.
	 * Fields defined in enum @ref gtxthrcfg_data. */
	volatile u32 gtxthrcfg;

	/** Core Threshold Control Register	<i>Offset: 10Ch</i>.
	 * Fields defined in enum @ref grxthrcfg_data. */
	volatile u32 grxthrcfg;

	/** Core Control Register		<i>Offset: 110h</i>.
	 * Fields defined in enum @ref gctl_data. */
	volatile u32 gctl;

	/** Core Interrupt Mask Register	<i>Offset: 114h</i>.
	 * Fields defined in enum @ref gevten_data. */
	volatile u32 gevten;

	/** Core Status Register		<i>Offset: 118h</i>.
	 * Fields defined in enum @ref gsts_data. */
	volatile u32 gsts;

	/** reserved				<i>Offset: 11Ch</i> */
	volatile u32 reserved0;

	/** Synopsys ID Register		<i>Offset: 120h</i> */
	volatile u32 gsnpsid;

	/** General Purpose I/O Register	<i>Offset: 124h</i> */
	volatile u32 ggpio;

	/** User ID Register			<i>Offset: 128h</i> */
	volatile u32 guid;

	/** reserved				<i>Offset: 12Ch</i> */
	volatile u32 reserved1;

	/** Bus Error Address Register		<i>Offset: 130h</i> */
	volatile u32 gbuserraddrlo;

	/** Bus Error Address Register		<i>Offset: 134h</i> */
	volatile u32 gbuserraddrhi;

	/** SS Port to Bus Instance Mapping Register	<i>Offset: 138h</i> */
	volatile u32 gprtbimap_lo;

	/** SS Port to Bus Instance Mapping Register	<i>Offset: 13Ch</i> */
	volatile u32 gprtbimap_hi;

	/** Hardware Parameter 0 Register	<i>Offset: 140h</i>.
	 * Fields defined in enum @ref ghwparams0_data. */
	volatile u32 ghwparams0;

	/** Hardware Parameter 1 Register	<i>Offset: 144h</i>.
	 * Fields defined in enum @ref ghwparams1_data. */
	volatile u32 ghwparams1;

	/** Hardware Parameter 2 Register	<i>Offset: 148h</i>.
	 * Fields defined in enum @ref ghwparams2_data. */
	volatile u32 ghwparams2;

	/** Hardware Parameter 3 Register	<i>Offset: 14Ch</i>.
	 * Fields defined in enum @ref ghwparams3_data. */
	volatile u32 ghwparams3;

	/** Hardware Parameter 4 Register	<i>Offset: 150h</i>.
	 * Fields defined in enum @ref ghwparams4_data. */
	volatile u32 ghwparams4;

	/** Hardware Parameter 5 Register	<i>Offset: 154h</i>.
	 * Fields defined in enum @ref ghwparams5_data. */
	volatile u32 ghwparams5;

	/** Hardware Parameter 6 Register	<i>Offset: 158h</i>.
	 * Fields defined in enum @ref ghwparams6_data. */
	volatile u32 ghwparams6;

	/** Hardware Parameter 7 Register	<i>Offset: 15Ch</i>.
	 * Fields defined in enum @ref ghwparams7_data. */
	volatile u32 ghwparams7;

	/** Debug Queue/FIFO Space Register	<i>Offset: 160h</i>.
	 * Fields defined in enum @ref gdbgfifospace_data. */
	volatile u32 gdbgfifospace;

	/** Debug LTSSM Register		<i>Offset: 164h</i>.
	 * Fields defined in enum @ref gdbgltssm_data */
	volatile u32 gdbgltssm;

	/** Debug LNMCC Register		<i>Offset: 168h</i> */
	volatile u32 gdbglnmcc;

	/** Debug BMU Register			<i>Offset: 16Ch</i> */
	volatile u32 gdbgbmu;

	/** Debug LSP Mux Register		<i>Offset: 170h</i> */
	volatile u32 gdbglspmux;

	/** Debug LSP Register			<i>Offset: 174h</i> */
	volatile u32 gdbglsp;

	/** Debug EP Info 0 Register		<i>Offset: 178h</i> */
	volatile u32 gdbgepinfo0;

	/** Debug EP Info 1 Register		<i>Offset: 17Ch</i> */
	volatile u32 gdbgepinfo1;

	/** HS Port to Bus Instance Mapping Register	<i>Offset: 180h</i> */
	volatile u32 gprtbimap_hs_lo;

	/** HS Port to Bus Instance Mapping Register	<i>Offset: 184h</i> */
	volatile u32 gprtbimap_hs_hi;

	/** FS Port to Bus Instance Mapping Register	<i>Offset: 188h</i> */
	volatile u32 gprtbimap_fs_lo;

	/** FS Port to Bus Instance Mapping Register	<i>Offset: 18Ch</i> */
	volatile u32 gprtbimap_fs_hi;

	/** reserved				<i>Offset: 190h-1BCh</i> */
	volatile u32 reserved4[12];

	/** Global RMMI PHY Control Register	<i>Offset: 1C0h-200h</i> */
	volatile u32 gusb3rmmictl[16];

	/** USB2 Configuration Registers	<i>Offset: 200h-23Ch</i>.
	 * Fields defined in enum @ref gusb2phycfg_data. */
	volatile u32 gusb2phycfg[16];

	/** USB2 I2C Access Registers		<i>Offset: 240h-27Ch</i>.
	 * Fields defined in enum @ref gusb2i2cctl_data. */
	volatile u32 gusb2i2cctl[16];

	/** USB2 PHY Vendor Control Registers	<i>Offset: 280h-2BCh</i>.
	 * Fields defined in enum @ref gusb2phyacc_data. */
	volatile u32 gusb2phyacc[16];

	/** USB3 Pipe Control Registers		<i>Offset: 2C0h-2FCh</i>.
	 * Fields defined in enum @ref gusb3pipectl_data. */
	volatile u32 gusb3pipectl[16];

	/** Transmit FIFO Size Registers	<i>Offset: 300h-37Ch</i>.
	 * Fields defined in enum @ref gfifosize_data. */
	volatile u32 gtxfifosiz[32];

	/** Receive FIFO Size Registers		<i>Offset: 380h-3FC0h</i>.
	 * Fields defined in enum @ref gfifosize_data. */
	volatile u32 grxfifosiz[32];

	/** Event Buffer Registers		<i>Offset: 400h-5FCh</i>.
	 * Fields defined in struct @ref geventbuf_data. */
	struct geventbuf_data geventbuf[32];

	/** Hardware Parameter 8 Register	<i>Offset: 600h</i>.
	 * Fields defined in enum @ref ghwparams8_data. */
	volatile u32 ghwparams8;
} dwc_usb3_core_global_regs_t;


/****************************************************************************/
/* Device Global Registers */

/**
 * This enum represents the bit fields in the Device Configuration
 * Register (DCFG).
 */
	/** Device Speed			<i>Access: R_W</i> */
#define DWC_DCFG_DEVSPD_BITS		0x000007U
#define DWC_DCFG_DEVSPD_SHIFT		0U

	/** Device Speed values */
#define DWC_SPEED_HS_PHY_30MHZ_OR_60MHZ		0U	/** @< */
#define DWC_SPEED_FS_PHY_30MHZ_OR_60MHZ		1U	/** @< */
#define DWC_SPEED_LS_PHY_6MHZ				2U	/** @< */
#define DWC_SPEED_FS_PHY_48MHZ				3U	/** @< */
#define DWC_SPEED_SS_PHY_125MHZ_OR_250MHZ	4U

	/** Device Address			<i>Access: R_W</i> */
#define DWC_DCFG_DEVADDR_BITS		0x0003f8U
#define DWC_DCFG_DEVADDR_SHIFT		3U

	/** Periodic Frame Interval		<i>Access: R_W</i> */
#define DWC_DCFG_PER_FR_INTVL_BITS		0x000c00U
#define DWC_DCFG_PER_FR_INTVL_SHIFT		10U

	/** Periodic Frame Interval values */
#define DWC_DCFG_PER_FR_INTVL_80		0U	/** @< */
#define DWC_DCFG_PER_FR_INTVL_85		1U	/** @< */
#define DWC_DCFG_PER_FR_INTVL_90		2U	/** @< */
#define DWC_DCFG_PER_FR_INTVL_95		3U

	/** Device Interrupt Number		<i>Access: R_W</i> */
#define DWC_DCFG_DEV_INTR_NUM_BITS		0x01f000U
#define DWC_DCFG_DEV_INTR_NUM_SHIFT		12U

	/** Number of Receive Buffers		<i>Access: R_W</i> */
#define DWC_DCFG_NUM_RCV_BUF_BITS		0x3e0000U
#define DWC_DCFG_NUM_RCV_BUF_SHIFT		17U

	/** LPM Capable				<i>Access: R_W</i> */
#define DWC_DCFG_LPM_CAP_BIT			0x400000U
#define DWC_DCFG_LPM_CAP_SHIFT			22U

/**
 * This enum represents the bit fields in the Device Control
 * Register (DCTL).
 */
	/** Soft Disconnect			<i>Access: R_W</i> */
#define DWC_DCTL_SFT_DISCONN_BIT		0x00000001U
#define DWC_DCTL_SFT_DISCONN_SHIFT		0U

	/** Test Control			<i>Access: R_W</i> */
#define DWC_DCTL_TSTCTL_BITS			0x0000001eU
#define DWC_DCTL_TSTCTL_SHIFT			1U

	/** USB/Link State Change Request	<i>Access: R_W</i> */
#define DWC_DCTL_ULST_CHNG_REQ_BITS			0x000001e0U
#define DWC_DCTL_ULST_CHNG_REQ_SHIFT		5U

	/** Requested Link State Transition/Action In SS Mode */
#define DWC_LINK_STATE_REQ_NO_ACTION		0U	/** @< */
#define DWC_LINK_STATE_REQ_SS_DISABLED		4U	/** @< */
#define DWC_LINK_STATE_REQ_RX_DETECT		5U	/** @< */
#define DWC_LINK_STATE_REQ_INACTIVE			6U	/** @< */
#define DWC_LINK_STATE_REQ_RECOVERY			8U	/** @< */
#define DWC_LINK_STATE_REQ_COMPLIANCE		10U	/** @< */
#define DWC_LINK_STATE_REQ_LOOPBACK			11U	/** @< */
#define DWC_LINK_STATE_REQ_HOST_MODE_ONLY	15U

	/** Requested Link State Transition/Action In HS/FS/LS Mode */
#define DWC_LINK_STATE_REQ_REMOTE_WAKEUP	8U

	/** U1/U2 control			<i>Access: R_W</i> */
#define DWC_DCTL_ACCEPT_U1_EN_BIT		0x00000200U
#define DWC_DCTL_ACCEPT_U1_EN_SHIFT		9U
#define DWC_DCTL_INIT_U1_EN_BIT			0x00000400U
#define DWC_DCTL_INIT_U1_EN_SHIFT		10U
#define DWC_DCTL_ACCEPT_U2_EN_BIT		0x00000800U
#define DWC_DCTL_ACCEPT_U2_EN_SHIFT		11U
#define DWC_DCTL_INIT_U2_EN_BIT			0x00001000U
#define DWC_DCTL_INIT_U2_EN_SHIFT		12U

	/** Controller Save State		<i>Access: R_W</i> */
#define DWC_DCTL_CSS_BIT					0x00010000U
#define DWC_DCTL_CSS_SHIFT					16U

	/** Controller Restore State		<i>Access: R_W</i> */
#define DWC_DCTL_CRS_BIT					0x00020000U
#define DWC_DCTL_CRS_SHIFT					17U

	/** L1 Hibernation Enable		<i>Access: R_W</i> */
#define DWC_DCTL_L1_HIBER_EN_BIT			0x00040000U
#define DWC_DCTL_L1_HIBER_EN_RES_SHIFT		18U

	/** Keep Connect (for hibernation)	<i>Access: R_W</i> */
#define DWC_DCTL_KEEP_CONNECT_BIT			0x00080000U
#define DWC_DCTL_KEEP_CONNECT_SHIFT			19U

	/** LPM NYET Response Threshold		<i>Access: R_W</i> */
#define DWC_DCTL_LPM_NYET_THRESH_BITS		0x00f00000U
#define DWC_DCTL_LPM_NYET_THRESH_SHIFT		20U

	/** LPM Response			<i>Access: R_W</i> */
#define DWC_DCTL_APP_L1_RES_BIT				0x00800000U
#define DWC_DCTL_APP_L1_RES_SHIFT			23U

	/* HIRD Threshold			<i>Access: R_W</i> */
#define DWC_DCTL_HIRD_THR_BITS				0x1f000000U
#define DWC_DCTL_HIRD_THR_SHIFT				24U

	/** Light Soft Reset			<i>Access: R_W</i> */
#define DWC_DCTL_LSFT_RST_BIT				0x20000000U
#define DWC_DCTL_LSFT_RST_SHIFT				29U

	/** Core Soft Reset			<i>Access: R_W</i> */
#define DWC_DCTL_CSFT_RST_BIT				0x40000000U
#define DWC_DCTL_CSFT_RST_SHIFT				30U

	/** Run/Stop				<i>Access: R_W</i> */
#define DWC_DCTL_RUN_STOP_BIT				0x80000000U
#define DWC_DCTL_RUN_STOP_SHIFT				31U

/**
 * This enum represents the bit fields of the Device Event Enable
 * Register (DEVTEN).
 */
	/** Disconnect Detected Event Enable	<i>Access: R_W</i> */
#define DWC_DEVTEN_DISCONN_BIT				0x0001U
#define DWC_DEVTEN_DISCONN_SHIFT			0U

	/** USB Reset Enable			<i>Access: R_W</i> */
#define DWC_DEVTEN_USBRESET_BIT				0x0002U
#define DWC_DEVTEN_USBRESET_SHIFT			1U

	/** Connect Done Enable			<i>Access: R_W</i> */
#define DWC_DEVTEN_CONNDONE_BIT				0x0004U
#define DWC_DEVTEN_CONNDONE_SHIFT			2U

	/** USB/Link State Change Event Enable	<i>Access: R_W</i> */
#define DWC_DEVTEN_ULST_CHNG_BIT			0x0008U
#define DWC_DEVTEN_ULST_CHNG_SHIFT			3U

	/** Resume/Remote-Wakeup Event Enable	<i>Access: R_W</i> */
#define DWC_DEVTEN_WKUP_BIT					0x0010U
#define DWC_DEVTEN_WKUP_SHIFT				4U

	/** Hibernation Request Event Enable	<i>Access: R_W</i> */
#define DWC_DEVTEN_HIBER_REQ_BIT			0x0020U
#define DWC_DEVTEN_HIBER_REQ_SHIFT			5U

	/** End of Periodic Frame Event Enable	<i>Access: R_W</i> */
#define DWC_DEVTEN_U3_L2L1_SUSP_BIT			0x0040U
#define DWC_DEVTEN_U3_L2L1_SUSP_SHIFT		6U

	/** Start of (Micro)Frame Enable	<i>Access: R_W</i> */
#define DWC_DEVTEN_SOF_BIT					0x0080U
#define DWC_DEVTEN_SOF_SHIFT				7U

	/** Erratic Error Event Enable		<i>Access: R_W</i> */
#define DWC_DEVTEN_ERRATICERR_BIT			0x0200U
#define DWC_DEVTEN_ERRATICERR_SHIFT			9U

	/** U2 Inactivity Timeout Enable	<i>Access: R_W</i> */
#define DWC_DEVTEN_INACT_TIMEOUT_BIT		0x2000U
#define DWC_DEVTEN_INACT_TIMEOUT_SHIFT		13U

/**
 * This enum represents the bit fields in the Device Status
 * Register (DSTS).
 */
	/** Connected Speed			<i>Access: RO</i>.
	 * (see enum @ref dcfg_data for values) */
#define DWC_DSTS_CONNSPD_BITS			0x00000007U
#define DWC_DSTS_CONNSPD_SHIFT			0U

	/** (Micro)Frame Number of Received SOF	<i>Access: RO</i> */
#define DWC_DSTS_SOF_FN_BITS			0x0001fff8U
#define DWC_DSTS_SOF_FN_SHIFT			3U

	/** RX Fifo Empty			<i>Access: RO</i> */
#define DWC_DSTS_RXFIFO_EMPTY_BIT		0x00020000U
#define DWC_DSTS_RXFIFO_EMPTY_SHIFT		17U

	/** USB/Link State			<i>Access: RO</i> */
#define DWC_DSTS_USBLNK_STATE_BITS		0x003c0000U
#define DWC_DSTS_USBLNK_STATE_SHIFT		18U

	/** USB/Link State values same as for devt_data_t */

	/** Device Controller Halted		<i>Access: RO</i> */
#define DWC_DSTS_DEV_CTRL_HLT_BIT		0x00400000U
#define DWC_DSTS_DEV_CTRL_HLT_SHIFT		22U

	/** Core Idle				<i>Access: RO</i> */
#define DWC_DSTS_CORE_IDLE_BIT			0x00800000U
#define DWC_DSTS_CORE_IDLE_SHIFT		23U

	/** Save State Status			<i>Access: RO</i> */
#define DWC_DSTS_SSS_BIT				0x01000000U
#define DWC_DSTS_SSS_SHIFT				24U

	/** Restore State Status		<i>Access: RO</i> */
#define DWC_DSTS_RSS_BIT				0x02000000U
#define DWC_DSTS_RSS_SHIFT				25U

	/** Save/Restore Error			<i>Access: RO</i> */
#define DWC_DSTS_SRE_BIT				0x10000000U
#define DWC_DSTS_SRE_SHIFT				28U

	/** Link-state Not Ready		<i>Access: RO</i> */
#define DWC_DSTS_LNR_BIT				0x20000000U
#define DWC_DSTS_LNR_SHIFT				29U

/**
 * This enum represents the bit fields in the Device Generic Command Parameter
 * Register (DGCMDPARn) for the various commands.
 */
	/** Periodic Parameters - for DWC_DGCMD_SET_PERIODIC_PARAMS command */
#define DWC_DGCMD_PER_PARAM_SEL_BITS			0x000003ffU
#define DWC_DGCMD_PER_PARAM_SEL_SHIFT			0U

	/** Host Role Request - for DWC_DGCMD_XMIT_HOST_ROLE_REQUEST command */
#define DWC_DGCMDPAR_HOST_ROLE_REQ_BITS			0x00000003U
#define DWC_DGCMDPAR_HOST_ROLE_REQ_SHIFT		0U

	/** RSP Phase values - for DWC_DGCMD_XMIT_HOST_ROLE_REQUEST (older
	 * cores) or DWC_DGCMD_HOST_ROLE_REQ_DEV_NOTIF (newer cores) */
#define DWC_DGCMDPAR_HOST_ROLE_REQ_INITIATE		1U	/** @< */
#define DWC_DGCMDPAR_HOST_ROLE_REQ_CONFIRM		2U

	/** Notification Type - for DWC_DGCMD_XMIT_DEV_NOTIF command */
#define DWC_DGCMDPAR_DEV_NOTIF_TYPE_BITS		0x0000000fU
#define DWC_DGCMDPAR_DEV_NOTIF_TYPE_SHIFT		0U

	/** Notification Type values - for DWC_DGCMD_XMIT_DEV_NOTIF command */
#define DWC_DGCMD_FUNCTION_WAKE_DEV_NOTIF		1U		/** @< */
#define DWC_DGCMD_LATENCY_TOL_DEV_NOTIF			2U		/** @< */
#define DWC_DGCMD_BUS_INTVL_ADJ_DEV_NOTIF		3U		/** @< */
#define DWC_DGCMD_HOST_ROLE_REQ_DEV_NOTIF		4U

	/** Notification Parameters - for DWC_DGCMD_XMIT_DEV_NOTIF command */
#define DWC_DGCMDPAR_DEV_NOTIF_PARAM_BITS		0xfffffff0U
#define DWC_DGCMDPAR_DEV_NOTIF_PARAM_SHIFT		4U

	/** Best Effort Latency Tolerance Value - for
	 * DWC_DGCMD_LATENCY_TOL_DEV_NOTIF command type */
#define DWC_DGCMDPAR_BELT_VALUE_BITS			0x000003ffU
#define DWC_DGCMDPAR_BELT_VALUE_SHIFT			0U

	/** Best Effort Latency Tolerance Scale - for
	 * DWC_DGCMD_LATENCY_TOL_DEV_NOTIF command type */
#define DWC_DGCMDPAR_BELT_SCALE_BITS			0x00000c00U
#define DWC_DGCMDPAR_BELT_SCALE_SHIFT			10U

	/** Latency Scale values (ns) */
#define DWC_LATENCY_VALUE_MULT_1024			1U	/** @< */
#define DWC_LATENCY_VALUE_MULT_32768		2U	/** @< */
#define DWC_LATENCY_VALUE_MULT_1048576		3U

/**
 * This enum represents the bit fields in the Device Generic Command
 * Register (DGCMDn).
 */
	/** Command Type			<i>Access: R_W</i> */
#define DWC_DGCMD_TYP_BITS			0x0ffU
#define DWC_DGCMD_TYP_SHIFT			0U

	/** Command Type values */
#define DWC_DGCMD_SET_PERIODIC_PARAMS				2U	/** @< */
#define DWC_DGCMD_XMIT_FUNC_WAKE_DEV_NOTIF			3U	/** @< */
#define DWC_DGCMD_SET_SCRATCHPAD_ARRAY_ADR_LO		4U	/** @< */
#define DWC_DGCMD_SET_SCRATCHPAD_ARRAY_ADR_HI		5U	/** @< */
#define DWC_DGCMD_XMIT_HOST_ROLE_REQUEST			6U	/** @< */
#define DWC_DGCMD_XMIT_DEV_NOTIF					7U	/** @< */
#define DWC_DGCMD_SELECTED_FIFO_FLUSH				9U	/** @< */
#define DWC_DGCMD_ALL_FIFO_FLUSH					10U	/** @< */
#define DWC_DGCMD_SET_EP_NRDY						12U	/** @< */
#define DWC_DGCMD_RUN_SOC_BUS_LOOPBK_TST			16U

	/** Command Interrupt on Complete	<i>Access: R_W</i> */
#define DWC_DGCMD_IOC_BIT			0x100U
#define DWC_DGCMD_IOC_SHIFT			8U

	/** Command Active			<i>Access: R_W</i> */
#define DWC_DGCMD_ACT_BIT			0x400U
#define DWC_DGCMD_ACT_SHIFT			10U

	/** Command Status			<i>Access: R_W</i> */
#define DWC_DGCMD_STS_BITS			0xf000U
#define DWC_DGCMD_STS_SHIFT			12U

	/** Command Status values */
#define DWC_DGCMD_STS_ERROR			15U

/**
 * This enum represents the bit fields in the Device Endpoint Mapping
 * Registers (DEPMAPn).
 */
	/** Resource Number			<i>Access: R_W / RO</i> */
#define DWC_EPMAP_RES_NUM_BITS		0x1fU
#define DWC_EPMAP_RES_NUM_SHIFT		0U

/**
 * Device Global Registers	<i>Offsets 700h-7FCh</i>.
 *
 * The following structures define the size and relative field offsets
 * for the Device Mode Global Registers.
 */
typedef struct dwc_usb3_dev_global_regs {

#define DWC_DEV_GLOBAL_REG_OFFSET	0x700U

	/** Device Configuration Register		<i>Offset: 700h</i>.
	 * Fields defined in enum @ref dcfg_data. */
	volatile u32 dcfg;

	/** Device Control Register			<i>Offset: 704h</i>.
	 * Fields defined in enum @ref dctl_data. */
	volatile u32 dctl;

	/** Device All Endpoints Interrupt Mask Register <i>Offset: 708h</i>.
	 * Fields defined in enum @ref devten_data. */
	volatile u32 devten;

	/** Device Status Register			<i>Offset: 70Ch</i>.
	 * Fields defined in enum @ref dsts_data. */
	volatile u32 dsts;

	/** Device Generic Command Parameter Register	<i>Offset: 710h</i>.
	 * Fields defined in enum @ref dgcmdpar_data. */
	volatile u32 dgcmdpar;

	/** Device Generic Command Register		<i>Offset: 714h</i>.
	 * Fields defined in enum @ref dgcmd_data. */
	volatile u32 dgcmd;

	/** reserved				<i>Offset: 718h-71Ch</i> */
	volatile u32 reserved[2];

	/** Device Active Logical Endpoint Enable Register <i>Offset: 720h</i>.
	 * One bit per logical endpoint, bit0=EP0 ... bit31=EP31. */
	volatile u32 dalepena;
} dwc_usb3_dev_global_regs_t;


/****************************************************************************/
/* Device Endpoint Specific Registers */

/**
 * This enum represents the bit fields in the Device Endpoint Command
 * Parameter 1 Register (DEPCMDPAR1n) for the Set Endpoint Configuration
 * (DEPCFG) command.
 */
	/** Interrupt number */
#define DWC_EPCFG1_INTRNUM_BITS			0x0000003fU
#define DWC_EPCFG1_INTRNUM_SHIFT		0U

	/** Stream Completed */
#define DWC_EPCFG1_XFER_CMPL_BIT		0x00000100U
#define DWC_EPCFG1_XFER_CMPL_SHIFT		8U

	/** Stream In Progress */
#define DWC_EPCFG1_XFER_IN_PROG_BIT		0x00000200U
#define DWC_EPCFG1_XFER_IN_PROG_SHIFT	9U

	/** Stream Not Ready */
#define DWC_EPCFG1_XFER_NRDY_BIT		0x00000400U
#define DWC_EPCFG1_XFER_NRDY_SHIFT		10U

	/** Rx FIFO Underrun / Tx FIFO Overrun */
#define DWC_EPCFG1_FIFOXRUN_BIT			0x00000800U
#define DWC_EPCFG1_FIFOXRUN_SHIFT		11U

	/** Back-to-Back Setup Packets Received	 */
#define DWC_EPCFG1_SETUP_PNDG_BIT		0x00001000U
#define DWC_EPCFG1_SETUP_PNDG_SHIFT		12U

	/** Endpoint Command Complete */
#define DWC_EPCFG1_EPCMD_CMPL_BIT		0x00002000U
#define DWC_EPCFG1_EPCMD_CMPL_SHIFT		13U

	/** Endpoint bInterval */
#define DWC_EPCFG1_BINTERVAL_BITS		0x00ff0000U
#define DWC_EPCFG1_BINTERVAL_SHIFT		16U

	/** Endpoint Stream Capability */
#define DWC_EPCFG1_STRM_CAP_BIT			0x01000000U
#define DWC_EPCFG1_STRM_CAP_SHIFT		24U

	/** Endpoint Direction */
#define DWC_EPCFG1_EP_DIR_BIT			0x02000000U
#define DWC_EPCFG1_EP_DIR_SHIFT			25U

	/** Endpoint Number */
#define DWC_EPCFG1_EP_NUM_BITS			0x3c000000U
#define DWC_EPCFG1_EP_NUM_SHIFT			26

/**
 * This enum represents the bit fields in the Device Endpoint Command
 * Parameter 0 Register (DEPCMDPAR0n) for the Set Endpoint Configuration
 * (DWC_EPCMD_SET_EP_CFG) command.
 */
	/** Endpoint Type			<i>Access: R_W</i> */
#define DWC_EPCFG0_EPTYPE_BITS			0x00000006U
#define DWC_EPCFG0_EPTYPE_SHIFT			1U

	/** Endpoint Type values */
#define DWC_USB3_EP_TYPE_CONTROL	0U	/** @< */
#define DWC_USB3_EP_TYPE_ISOC		1U	/** @< */
#define DWC_USB3_EP_TYPE_BULK		2U	/** @< */
#define DWC_USB3_EP_TYPE_INTR		3U

	/** Maximum Packet Size			<i>Access: R_W</i> */
#define DWC_EPCFG0_MPS_BITS					0x00003ff8U
#define DWC_EPCFG0_MPS_SHIFT				3U

	/** Flow Control State			<i>Access: R_W</i> */
#define DWC_EPCFG0_FLOW_CTRL_STATE_BIT		0x00010000U
#define DWC_EPCFG0_FLOW_CTRL_STATE_SHIFT	16U

	/** Tx Fifo Number (IN endpoints only)	<i>Access: R_W</i> */
#define DWC_EPCFG0_TXFNUM_BITS				0x003e0000U
#define DWC_EPCFG0_TXFNUM_SHIFT				17U

	/** Burst Size				<i>Access: R_W</i> */
#define DWC_EPCFG0_BRSTSIZ_BITS				0x03c00000U
#define DWC_EPCFG0_BRSTSIZ_SHIFT			22U

	/** Data Sequence Num (old)		<i>Access: R_W</i> */
#define DWC_EPCFG0_DSNUM_BITS				0x7c000000U
#define DWC_EPCFG0_DSNUM_SHIFT				26U

	/** Ignore Data Sequence Num (old)	<i>Access: R_W</i> */
#define DWC_EPCFG0_IGN_DSNUM_BIT			0x80000000U
#define DWC_EPCFG0_IGN_DSNUM_SHIFT			31U

	/** Config Action (new)			<i>Access: R_W</i> */
#define DWC_EPCFG0_CFG_ACTION_BITS			0xc0000000U
#define DWC_EPCFG0_CFG_ACTION_SHIFT			30U

	/** Config Action values (new) */
#define DWC_CFG_ACTION_INIT			0U	/** @< */
#define DWC_CFG_ACTION_RESTORE		1U	/** @< */
#define DWC_CFG_ACTION_MODIFY		2U

/**
 * This enum represents the bit fields in the Device Endpoint Command
 * Register (DEPCMDn).
 */
	/** Command Type			<i>Access: R_W</i> */
#define DWC_EPCMD_TYP_BITS			0x0ffU
#define DWC_EPCMD_TYP_SHIFT			0U

	/** Command Type values */
#define DWC_EPCMD_SET_EP_CFG		1U	/** @< */
#define DWC_EPCMD_SET_XFER_CFG		2U	/** @< */
#define DWC_EPCMD_GET_EP_STATE		3U	/** @< */
#define DWC_EPCMD_SET_STALL			4U	/** @< */
#define DWC_EPCMD_CLR_STALL			5U	/** @< */
#define DWC_EPCMD_START_XFER		6U	/** @< */
#define DWC_EPCMD_UPDATE_XFER		7U	/** @< */
#define DWC_EPCMD_END_XFER			8U	/** @< */
#define DWC_EPCMD_START_NEW_CFG		9U

	/** Command Interrupt on Complete	<i>Access: R_W</i> */
#define DWC_EPCMD_IOC_BIT		 			0x100U
#define DWC_EPCMD_IOC_SHIFT					8U

	/** Command Active			<i>Access: R_W</i> */
#define DWC_EPCMD_ACT_BIT					0x400U
#define DWC_EPCMD_ACT_SHIFT					10U

	/** High Priority / Force RM Bit	<i>Access: R_W</i> */
#define DWC_EPCMD_HP_FRM_BIT				0x800U
#define DWC_EPCMD_HP_FRM_SHIFT				11U

	/** Command Completion Status		<i>Access: R_W</i> */
#define DWC_EPCMD_CMPL_STS_BITS				0xf000U
#define DWC_EPCMD_CMPL_STS_SHIFT			12U

	/** Stream Number or uFrame (input)	<i>Access: R_W</i> */
#define DWC_EPCMD_STR_NUM_OR_UF_BITS		0xffff0000U
#define DWC_EPCMD_STR_NUM_OR_UF_SHIFT		16U

	/** Transfer Resource Index (output)	<i>Access: R_W</i> */
#define DWC_EPCMD_XFER_RSRC_IDX_BITS		0x007f0000U
#define DWC_EPCMD_XFER_RSRC_IDX_SHIFT		16U

/**
 * Device Endpoint Specific Registers <i>Offsets 800h-9ECh for OUT,
 *						 810h-9FCh for IN</i>.
 * There will be one set of endpoint registers per logical endpoint
 * implemented.
 */
typedef struct dwc_usb3_dev_ep_regs {

#define DWC_DEV_OUT_EP_REG_OFFSET	0x800U
#define DWC_DEV_IN_EP_REG_OFFSET	0x810U
#define DWC_EP_REG_OFFSET			0x20U

	/** Device Endpoint Command Parameter 2 Register <i>Offset: 800h/810h +
	 *					(ep_num * 20h) + 00h</i> */
	volatile u32 depcmdpar2;

	/** Device Endpoint Command Parameter 1 Register <i>Offset: 800h/810h +
	 *					(ep_num * 20h) + 04h</i> */
	volatile u32 depcmdpar1;

	/** Device Endpoint Command Parameter 0 Register <i>Offset: 800h/810h +
	 *					(ep_num * 20h) + 08h</i> */
	volatile u32 depcmdpar0;

	/** Device Endpoint Command Register	<i>Offset: 800h/810h +
	 *					(ep_num * 20h) + 0Ch</i>.
	 * Fields defined in enum @ref depcmd_data. */
	volatile u32 depcmd;

	/** reserved				<i>Offset: 800h/810h +
	 *					(ep_num * 20h) + 10h-1Ch</i> */
	volatile u32 reserved[4];
} dwc_usb3_dev_ep_regs_t;


/****************************************************************************/
/* DMA Descriptor Specific Structures */

/**
 * This enum represents the bit fields in the DMA Descriptor
 * Status quadlet.
 */
	/** Transfer Count */
#define DWC_DSCSTS_XFRCNT_BITS		0x00ffffffU
#define DWC_DSCSTS_XFRCNT_SHIFT		0U

	/** Packet Count Minus 1 (for HS IN transfers) */
#define DWC_DSCSTS_PCM1_BITS		0x03000000U
#define DWC_DSCSTS_PCM1_SHIFT		24U

	/** Transfer Request Block Response */
#define DWC_DSCSTS_TRBRSP_BITS		0xf0000000U
#define DWC_DSCSTS_TRBRSP_SHIFT		28U

	/** Response values */
#define DWC_TRBRSP_MISSED_ISOC_IN	1U	/** @< */
#define DWC_TRBRSP_SETUP_PEND		2U	/** @< */
#define DWC_TRBRSP_XFER_IN_PROG		4U

/**
 * This enum represents the bit fields in the DMA Descriptor
 * Control quadlet.
 */
	/** Hardware-Owned bit */
#define DWC_DSCCTL_HWO_BIT			0x00000001U
#define DWC_DSCCTL_HWO_SHIFT		0U

	/** Last Descriptor bit */
#define DWC_DSCCTL_LST_BIT			0x00000002U
#define DWC_DSCCTL_LST_SHIFT		1U

	/** Chain Buffer bit */
#define DWC_DSCCTL_CHN_BIT			0x00000004U
#define DWC_DSCCTL_CHN_SHIFT		2U

	/** Continue on Short Packet bit */
#define DWC_DSCCTL_CSP_BIT			0x00000008U
#define DWC_DSCCTL_CSP_SHIFT		3U

	/** Transfer Request Block Control field */
#define DWC_DSCCTL_TRBCTL_BITS		0x000003f0U
#define DWC_DSCCTL_TRBCTL_SHIFT		4U

	/** Transfer Request Block Control types */
#define DWC_DSCCTL_TRBCTL_NORMAL		1	/** @< */
#define DWC_DSCCTL_TRBCTL_SETUP			2	/** @< */
#define DWC_DSCCTL_TRBCTL_STATUS_2		3	/** @< */
#define DWC_DSCCTL_TRBCTL_STATUS_3		4	/** @< */
#define DWC_DSCCTL_TRBCTL_CTLDATA_1ST	5	/** @< */
#define DWC_DSCCTL_TRBCTL_ISOC_1ST		6	/** @< */
#define DWC_DSCCTL_TRBCTL_ISOC			7	/** @< */
#define DWC_DSCCTL_TRBCTL_LINK			8

	/** Interrupt on Short Packet bit */
#define DWC_DSCCTL_ISP_BIT				0x00000400U
#define DWC_DSCCTL_ISP_SHIFT			10U
#define DWC_DSCCTL_IMI_BIT				DWC_DSCCTL_ISP_BIT
#define DWC_DSCCTL_IMI_SHIFT			DWC_DSCCTL_ISP_SHIFT

	/** Interrupt on Completion bit */
#define DWC_DSCCTL_IOC_BIT				0x00000800U
#define DWC_DSCCTL_IOC_SHIFT			11U

	/** Stream ID / SOF Number */
#define DWC_DSCCTL_STRMID_SOFN_BITS		0x3fffc000U
#define DWC_DSCCTL_STRMID_SOFN_SHIFT	14U

/**
 * DMA Descriptor structure
 *
 * DMA Descriptor structure contains 4 quadlets:
 * Buffer Pointer Low address, Buffer Pointer High address, Status, and Control.
 */
typedef struct dwc_usb3_dma_desc {
	/** Buffer Pointer - Low address quadlet */
	u32	bptl;

	/** Buffer Pointer - High address quadlet */
	u32	bpth;

	/** Status quadlet. Fields defined in enum @ref desc_sts_data. */
	u32	status;

	/** Control quadlet. Fields defined in enum @ref desc_ctl_data. */
	u32	control;
} dwc_usb3_dma_desc_t;

#ifdef SSIC

/* SSIC Registers */
typedef struct dwc_usb3_ssic_regs {

#define DWC_SSIC_REG_OFFSET	0xc40

	volatile u32 sctl[16];
	volatile u32 sevt[16];
	volatile u32 sevten[16];
	volatile u32 gscfg;
	volatile u32 gsser;
	volatile u32 gsdbg;
} dwc_usb3_ssic_regs_t;

#define DWC_SEVTEN_ROM_INIT_CMPLT_EN_BIT	0x00000001U
#define DWC_SEVTEN_ROM_INIT_CMPLT_EN_SHIFT	0U

#define DWC_SEVTEN_LACC_CMPLT_EN_BIT		0x00000002U
#define DWC_SEVTEN_LACC_CMPLT_EN_SHIFT		1U

#define DWC_SEVTEN_RCMD_RES_RCVD_EN_BIT		0x00000004U
#define DWC_SEVTEN_RCMD_RES_RCVD_EN_SHIFT	2U

#define DWC_SEVTEN_RCMD_RES_SENT_EN_BIT		0x00000008U
#define DWC_SEVTEN_RCMD_RES_SENT_EN_SHIFT	3U

#define DWC_SEVTEN_MPHY_ST_CHNGD_EN_BIT		0x00000010U
#define DWC_SEVTEN_MPHY_ST_CHNGD_EN_SHIFT	4U

#define DWC_SEVTEN_OK_STRT_RRAP_EN_BIT		0x00000020U
#define DWC_SEVTEN_OK_STRT_RRAP_EN_SHIFT	5U

#define DWC_SEVTEN_RRAP_ERROR_EN_BIT		0x00000040U
#define DWC_SEVTEN_RRAP_ERROR_EN_SHIFT		6U

#define DWC_SEVT_ROM_INIT_CMPLT_BIT			0x00000001U
#define DWC_SEVT_ROM_INIT_CMPLT_SHIFT		0U

#define DWC_SEVT_LACC_CMPLT_BIT				0x00000002U
#define DWC_SEVT_LACC_CMPLT_SHIFT			1U

#define DWC_SEVT_RCMD_RES_RCVD_BIT			0x00000004U
#define DWC_SEVT_RCMD_RES_RCVD_SHIFT		2U

#define DWC_SEVT_RCMD_RES_SENT_BIT			0x00000008U
#define DWC_SEVT_RCMD_RES_SENT_SHIFT		3U

#define DWC_SEVT_MPHY_ST_CHNG_BIT			0x00000010U
#define DWC_SEVT_MPHY_ST_CHNG_SHIFT			4U

#define DWC_SEVT_OK_STRT_RRAP_BIT			0x00000020U
#define DWC_SEVT_OK_STRT_RRAP_SHIFT			5U

#define DWC_SEVT_RRAP_ERROR_BIT				0x00000040U
#define DWC_SEVT_RRAP_ERROR_SHIFT			6U

#define DWC_SEVT_RACC_RESULT_BITS			0x00000300U
#define DWC_SEVT_RACC_RESULT_SHIFT			8U

#define DWC_SEVT_LACC_RESULT_BIT			0x00000400U
#define DWC_SEVT_LACC_RESULT_SHIFT			10U

#define DWC_SEVT_READ_RCVD_BIT				0x00000800U
#define DWC_SEVT_READ_RCVD_SHIFT			11U

#define DWC_SEVT_RUADDR_RCVD_BITS			0x0000f000U
#define DWC_SEVT_RUADDR_RCVD_SHIFT			12U

#define DWC_SEVT_RLADDR_RCVD_BITS			0x00ff0000U
#define DWC_SEVT_RLADDR_RCVD_SHIFT			16U

#define DWC_SEVT_RDATA_RCVD_BITS			0xff000000U
#define DWC_SEVT_RDATA_RCVD_SHIFT			24U

#define DWC_SCTL_GO_ACC_BIT					0x00000001U
#define DWC_SCTL_GO_ACC_SHIFT				0U

#define DWC_SCTL_RACC_BIT					0x00000002U
#define DWC_SCTL_RACC_SHIFT					1U

#define DWC_SCTL_RD_WR_N_BIT				0x00000004U
#define DWC_SCTL_RD_WR_N_SHIFT				2U

#define DWC_SCTL_BCW_BIT					0x00000008U
#define DWC_SCTL_BCW_SHIFT					3U

#define DWC_SCTL_IN_LN_CFG_BIT				0x00000010U
#define DWC_SCTL_IN_LN_CFG_SHIFT			4U

#define DWC_SCTL_CFG_DONE_BIT				0x00000020U
#define DWC_SCTL_CFG_DEON_SHIFT				5U

#define DWC_SCTL_CBS_ACC_BIT				0x00000040U
#define DWC_SCTL_CBS_ACC_SHIFT				6U

#define DWC_SCTL_RXS_ACC_BIT				0x00000080U
#define DWC_SCTL_RXS_ACC_SHIFT				7U

#define DWC_SCTL_EAID_BITS					0x0000f000U
#define DWC_SCTL_EAID_SHIFT					12U

#define DWC_SCTL_AID_BITS					0x00ff0000U
#define DWC_SCTL_AID_SHIFT					16U

#define DWC_SCTL_ADATA_BITS					0xff000000U
#define DWC_SCTL_ADATA_SHIFT				24U

#endif /* SSIC */

#ifdef __cplusplus
}
#endif

#endif /* _DWC_USB3_REGS_H_ */
