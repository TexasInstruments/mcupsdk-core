/********************************************************************
 * Copyright (C) 2019 Texas Instruments Incorporated.
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
 *  Name        : cslr_usb3p0ss.h
*/
#ifndef CSLR_USB3P0SS_H_
#define CSLR_USB3P0SS_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  : Global Control Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PID;                       /* Revision Register */
    volatile uint32_t USB3P0SS_W1;               /* Wrapper Register 1 */
    volatile uint32_t STATIC_CONFIG;             /* Static Configuration Register */
    volatile uint32_t PHY_TEST;                  /* USB2 PHY TEST Control and Status */
    volatile uint32_t USB3P0SS_DEBUG_CTRL;       /* USB debug control */
    volatile uint32_t USB3P0SS_DEBUG_INFO;       /* USB debug information */
    volatile uint32_t USB3P0SS_DEBUG_LINK_STATE;   /* USB debug link state */
    volatile uint32_t USB3P0SS_DEVICE_CTRL;      /* Register for device control */
} CSL_usb3p0ss_cmnRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_USB3P0SS_CMN_PID                                             (0x00000000U)
#define CSL_USB3P0SS_CMN_USB3P0SS_W1                                     (0x00000004U)
#define CSL_USB3P0SS_CMN_STATIC_CONFIG                                   (0x00000008U)
#define CSL_USB3P0SS_CMN_PHY_TEST                                        (0x0000000CU)
#define CSL_USB3P0SS_CMN_USB3P0SS_DEBUG_CTRL                             (0x00000010U)
#define CSL_USB3P0SS_CMN_USB3P0SS_DEBUG_INFO                             (0x00000014U)
#define CSL_USB3P0SS_CMN_USB3P0SS_DEBUG_LINK_STATE                       (0x00000018U)
#define CSL_USB3P0SS_CMN_USB3P0SS_DEVICE_CTRL                            (0x0000001CU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_USB3P0SS_CMN_PID_MINOR_MASK                                  (0x0000003FU)
#define CSL_USB3P0SS_CMN_PID_MINOR_SHIFT                                 (0x00000000U)
#define CSL_USB3P0SS_CMN_PID_MINOR_MAX                                   (0x0000003FU)

#define CSL_USB3P0SS_CMN_PID_CUSTOM_MASK                                 (0x000000C0U)
#define CSL_USB3P0SS_CMN_PID_CUSTOM_SHIFT                                (0x00000006U)
#define CSL_USB3P0SS_CMN_PID_CUSTOM_MAX                                  (0x00000003U)

#define CSL_USB3P0SS_CMN_PID_MAJOR_MASK                                  (0x00000700U)
#define CSL_USB3P0SS_CMN_PID_MAJOR_SHIFT                                 (0x00000008U)
#define CSL_USB3P0SS_CMN_PID_MAJOR_MAX                                   (0x00000007U)

#define CSL_USB3P0SS_CMN_PID_RTL_MASK                                    (0x0000F800U)
#define CSL_USB3P0SS_CMN_PID_RTL_SHIFT                                   (0x0000000BU)
#define CSL_USB3P0SS_CMN_PID_RTL_MAX                                     (0x0000001FU)

#define CSL_USB3P0SS_CMN_PID_MODULE_ID_MASK                              (0x0FFF0000U)
#define CSL_USB3P0SS_CMN_PID_MODULE_ID_SHIFT                             (0x00000010U)
#define CSL_USB3P0SS_CMN_PID_MODULE_ID_MAX                               (0x00000FFFU)

#define CSL_USB3P0SS_CMN_PID_BU_MASK                                     (0x30000000U)
#define CSL_USB3P0SS_CMN_PID_BU_SHIFT                                    (0x0000001CU)
#define CSL_USB3P0SS_CMN_PID_BU_MAX                                      (0x00000003U)

#define CSL_USB3P0SS_CMN_PID_SCHEME_MASK                                 (0xC0000000U)
#define CSL_USB3P0SS_CMN_PID_SCHEME_SHIFT                                (0x0000001EU)
#define CSL_USB3P0SS_CMN_PID_SCHEME_MAX                                  (0x00000003U)

/* USB3P0SS_W1 */

#define CSL_USB3P0SS_CMN_USB3P0SS_W1_PWRUP_RST_N_MASK                    (0x00000001U)
#define CSL_USB3P0SS_CMN_USB3P0SS_W1_PWRUP_RST_N_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_CMN_USB3P0SS_W1_PWRUP_RST_N_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CMN_USB3P0SS_W1_RSVD1_MASK                          (0x000000FEU)
#define CSL_USB3P0SS_CMN_USB3P0SS_W1_RSVD1_SHIFT                         (0x00000001U)
#define CSL_USB3P0SS_CMN_USB3P0SS_W1_RSVD1_MAX                           (0x0000007FU)

#define CSL_USB3P0SS_CMN_USB3P0SS_W1_OVERCURRENT_SEL_MASK                (0x00000100U)
#define CSL_USB3P0SS_CMN_USB3P0SS_W1_OVERCURRENT_SEL_SHIFT               (0x00000008U)
#define CSL_USB3P0SS_CMN_USB3P0SS_W1_OVERCURRENT_SEL_MAX                 (0x00000001U)

#define CSL_USB3P0SS_CMN_USB3P0SS_W1_MODESTRAP_SEL_MASK                  (0x00000200U)
#define CSL_USB3P0SS_CMN_USB3P0SS_W1_MODESTRAP_SEL_SHIFT                 (0x00000009U)
#define CSL_USB3P0SS_CMN_USB3P0SS_W1_MODESTRAP_SEL_MAX                   (0x00000001U)

#define CSL_USB3P0SS_CMN_USB3P0SS_W1_RSVD2_MASK                          (0x0000FC00U)
#define CSL_USB3P0SS_CMN_USB3P0SS_W1_RSVD2_SHIFT                         (0x0000000AU)
#define CSL_USB3P0SS_CMN_USB3P0SS_W1_RSVD2_MAX                           (0x0000003FU)

#define CSL_USB3P0SS_CMN_USB3P0SS_W1_OVERCURRENT_N_MASK                  (0x00010000U)
#define CSL_USB3P0SS_CMN_USB3P0SS_W1_OVERCURRENT_N_SHIFT                 (0x00000010U)
#define CSL_USB3P0SS_CMN_USB3P0SS_W1_OVERCURRENT_N_MAX                   (0x00000001U)

#define CSL_USB3P0SS_CMN_USB3P0SS_W1_MODESTRAP_MASK                      (0x00060000U)
#define CSL_USB3P0SS_CMN_USB3P0SS_W1_MODESTRAP_SHIFT                     (0x00000011U)
#define CSL_USB3P0SS_CMN_USB3P0SS_W1_MODESTRAP_MAX                       (0x00000003U)

#define CSL_USB3P0SS_CMN_USB3P0SS_W1_USB2_ONLY_MODE_MASK                 (0x00080000U)
#define CSL_USB3P0SS_CMN_USB3P0SS_W1_USB2_ONLY_MODE_SHIFT                (0x00000013U)
#define CSL_USB3P0SS_CMN_USB3P0SS_W1_USB2_ONLY_MODE_MAX                  (0x00000001U)

#define CSL_USB3P0SS_CMN_USB3P0SS_W1_RSVD3_MASK                          (0xFFF00000U)
#define CSL_USB3P0SS_CMN_USB3P0SS_W1_RSVD3_SHIFT                         (0x00000014U)
#define CSL_USB3P0SS_CMN_USB3P0SS_W1_RSVD3_MAX                           (0x00000FFFU)

/* STATIC_CONFIG */

#define CSL_USB3P0SS_CMN_STATIC_CONFIG_LANE_REVERSE_MASK                 (0x00000001U)
#define CSL_USB3P0SS_CMN_STATIC_CONFIG_LANE_REVERSE_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_CMN_STATIC_CONFIG_LANE_REVERSE_MAX                  (0x00000001U)

#define CSL_USB3P0SS_CMN_STATIC_CONFIG_VBUS_SEL_MASK                     (0x00000006U)
#define CSL_USB3P0SS_CMN_STATIC_CONFIG_VBUS_SEL_SHIFT                    (0x00000001U)
#define CSL_USB3P0SS_CMN_STATIC_CONFIG_VBUS_SEL_MAX                      (0x00000003U)

#define CSL_USB3P0SS_CMN_STATIC_CONFIG_LOOPBACK_MODE_MASK                (0x00000018U)
#define CSL_USB3P0SS_CMN_STATIC_CONFIG_LOOPBACK_MODE_SHIFT               (0x00000003U)
#define CSL_USB3P0SS_CMN_STATIC_CONFIG_LOOPBACK_MODE_MAX                 (0x00000003U)

#define CSL_USB3P0SS_CMN_STATIC_CONFIG_PLL_REF_SEL_MASK                  (0x000001E0U)
#define CSL_USB3P0SS_CMN_STATIC_CONFIG_PLL_REF_SEL_SHIFT                 (0x00000005U)
#define CSL_USB3P0SS_CMN_STATIC_CONFIG_PLL_REF_SEL_MAX                   (0x0000000FU)

#define CSL_USB3P0SS_CMN_STATIC_CONFIG_RSVD_MASK                         (0xFFFFFE00U)
#define CSL_USB3P0SS_CMN_STATIC_CONFIG_RSVD_SHIFT                        (0x00000009U)
#define CSL_USB3P0SS_CMN_STATIC_CONFIG_RSVD_MAX                          (0x007FFFFFU)

/* PHY_TEST */

#define CSL_USB3P0SS_CMN_PHY_TEST_PLL_BYPASS_MODE_MASK                   (0x00000001U)
#define CSL_USB3P0SS_CMN_PHY_TEST_PLL_BYPASS_MODE_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_CMN_PHY_TEST_PLL_BYPASS_MODE_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CMN_PHY_TEST_BIST_MODE_SEL_MASK                     (0x0000001EU)
#define CSL_USB3P0SS_CMN_PHY_TEST_BIST_MODE_SEL_SHIFT                    (0x00000001U)
#define CSL_USB3P0SS_CMN_PHY_TEST_BIST_MODE_SEL_MAX                      (0x0000000FU)

#define CSL_USB3P0SS_CMN_PHY_TEST_BIST_MODE_EN_MASK                      (0x00000020U)
#define CSL_USB3P0SS_CMN_PHY_TEST_BIST_MODE_EN_SHIFT                     (0x00000005U)
#define CSL_USB3P0SS_CMN_PHY_TEST_BIST_MODE_EN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CMN_PHY_TEST_BIST_ON_MASK                           (0x00000040U)
#define CSL_USB3P0SS_CMN_PHY_TEST_BIST_ON_SHIFT                          (0x00000006U)
#define CSL_USB3P0SS_CMN_PHY_TEST_BIST_ON_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CMN_PHY_TEST_BIST_COMPLETE_MASK                     (0x00000080U)
#define CSL_USB3P0SS_CMN_PHY_TEST_BIST_COMPLETE_SHIFT                    (0x00000007U)
#define CSL_USB3P0SS_CMN_PHY_TEST_BIST_COMPLETE_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CMN_PHY_TEST_BIST_ERROR_MASK                        (0x00000100U)
#define CSL_USB3P0SS_CMN_PHY_TEST_BIST_ERROR_SHIFT                       (0x00000008U)
#define CSL_USB3P0SS_CMN_PHY_TEST_BIST_ERROR_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CMN_PHY_TEST_BIST_ERROR_COUNT_MASK                  (0x0001FE00U)
#define CSL_USB3P0SS_CMN_PHY_TEST_BIST_ERROR_COUNT_SHIFT                 (0x00000009U)
#define CSL_USB3P0SS_CMN_PHY_TEST_BIST_ERROR_COUNT_MAX                   (0x000000FFU)

#define CSL_USB3P0SS_CMN_PHY_TEST_BIST_MODE_MASK                         (0x00020000U)
#define CSL_USB3P0SS_CMN_PHY_TEST_BIST_MODE_SHIFT                        (0x00000011U)
#define CSL_USB3P0SS_CMN_PHY_TEST_BIST_MODE_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CMN_PHY_TEST_RSVD_MASK                              (0xFFFC0000U)
#define CSL_USB3P0SS_CMN_PHY_TEST_RSVD_SHIFT                             (0x00000012U)
#define CSL_USB3P0SS_CMN_PHY_TEST_RSVD_MAX                               (0x00003FFFU)

/* USB3P0SS_DEBUG_CTRL */

#define CSL_USB3P0SS_CMN_USB3P0SS_DEBUG_CTRL_DEBUG_SEL_MASK              (0x0000001FU)
#define CSL_USB3P0SS_CMN_USB3P0SS_DEBUG_CTRL_DEBUG_SEL_SHIFT             (0x00000000U)
#define CSL_USB3P0SS_CMN_USB3P0SS_DEBUG_CTRL_DEBUG_SEL_MAX               (0x0000001FU)

#define CSL_USB3P0SS_CMN_USB3P0SS_DEBUG_CTRL_RSVD_MASK                   (0xFFFFFFE0U)
#define CSL_USB3P0SS_CMN_USB3P0SS_DEBUG_CTRL_RSVD_SHIFT                  (0x00000005U)
#define CSL_USB3P0SS_CMN_USB3P0SS_DEBUG_CTRL_RSVD_MAX                    (0x07FFFFFFU)

/* USB3P0SS_DEBUG_INFO */

#define CSL_USB3P0SS_CMN_USB3P0SS_DEBUG_INFO_DEBUG_INFO_MASK             (0xFFFFFFFFU)
#define CSL_USB3P0SS_CMN_USB3P0SS_DEBUG_INFO_DEBUG_INFO_SHIFT            (0x00000000U)
#define CSL_USB3P0SS_CMN_USB3P0SS_DEBUG_INFO_DEBUG_INFO_MAX              (0xFFFFFFFFU)

/* USB3P0SS_DEBUG_LINK_STATE */

#define CSL_USB3P0SS_CMN_USB3P0SS_DEBUG_LINK_STATE_DEBUG_LINK_STATE_MASK (0x7FFFFFFFU)
#define CSL_USB3P0SS_CMN_USB3P0SS_DEBUG_LINK_STATE_DEBUG_LINK_STATE_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CMN_USB3P0SS_DEBUG_LINK_STATE_DEBUG_LINK_STATE_MAX  (0x7FFFFFFFU)

#define CSL_USB3P0SS_CMN_USB3P0SS_DEBUG_LINK_STATE_RSVD_MASK             (0x80000000U)
#define CSL_USB3P0SS_CMN_USB3P0SS_DEBUG_LINK_STATE_RSVD_SHIFT            (0x0000001FU)
#define CSL_USB3P0SS_CMN_USB3P0SS_DEBUG_LINK_STATE_RSVD_MAX              (0x00000001U)

/* USB3P0SS_DEVICE_CTRL */

#define CSL_USB3P0SS_CMN_USB3P0SS_DEVICE_CTRL_DEV_WAKEUP_MASK            (0x00000001U)
#define CSL_USB3P0SS_CMN_USB3P0SS_DEVICE_CTRL_DEV_WAKEUP_SHIFT           (0x00000000U)
#define CSL_USB3P0SS_CMN_USB3P0SS_DEVICE_CTRL_DEV_WAKEUP_MAX             (0x00000001U)

#define CSL_USB3P0SS_CMN_USB3P0SS_DEVICE_CTRL_RSVD_MASK                  (0xFFFFFFFEU)
#define CSL_USB3P0SS_CMN_USB3P0SS_DEVICE_CTRL_RSVD_SHIFT                 (0x00000001U)
#define CSL_USB3P0SS_CMN_USB3P0SS_DEVICE_CTRL_RSVD_MAX                   (0x7FFFFFFFU)

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t CDNS_DID;
    volatile uint32_t CDNS_RID;
    volatile uint32_t OTGCAPABILITY;
    volatile uint8_t  Resv_16[4];
    volatile uint32_t OTGCMD;
    volatile uint32_t OTGSTS;
    volatile uint32_t OTGSTATE;
    volatile uint8_t  Resv_32[4];
    volatile uint32_t OTGIEN;
    volatile uint32_t OTGIVECT;
    volatile uint32_t CLK_FREQ;
    volatile uint32_t OTGTMR;
    volatile uint8_t  Resv_64[16];
    volatile uint32_t OTGSIMULATE;
    volatile uint32_t OVERRIDE;
    volatile uint32_t SUSP_CTRL;
    volatile uint32_t PHYRST_CFG;
    volatile uint32_t OTGANASTS;
    volatile uint32_t ADP_RAMP_TIME;
    volatile uint32_t OTGCTRL1;
    volatile uint32_t OTGCTRL2;
    volatile uint32_t VBUSVALID_DBNC_CFG;
    volatile uint32_t SESSVALID_DBNC_CFG;
    volatile uint32_t IDDIG_DBNC_CFG;
    volatile uint8_t  Resv_140[32];
    volatile uint32_t DB_ASF_MEM_MASK;
} CSL_usb3p0ss_ctrlRegs_drd;


typedef struct {
    volatile uint32_t ASF_INT_STATUS;
    volatile uint32_t ASF_INT_RAW_STATUS;
    volatile uint32_t ASF_INT_MASK;
    volatile uint32_t ASF_INT_TEST;
    volatile uint32_t ASF_FATAL_NONFATAL_SELECT;
    volatile uint32_t RSVD_6_ASF;
    volatile uint32_t RSVD_7_ASF;
    volatile uint32_t RSVD_8_ASF;
    volatile uint32_t ASF_SRAM_CORR_FAULT_STATUS;
    volatile uint32_t ASF_SRAM_UNCORR_FAULT_STATUS;
    volatile uint32_t ASF_SRAM_FAULT_STATS;
    volatile uint32_t RSVD_12_ASF;
    volatile uint32_t ASF_TRANS_TO_CTRL;
    volatile uint32_t ASF_TRANS_TO_FAULT_MASK;
    volatile uint32_t ASF_TRANS_TO_FAULT_STATUS;
    volatile uint32_t RSVD_16_ASF;
    volatile uint32_t RSVD_17_ASF;
    volatile uint32_t RSVD_18_ASF;
} CSL_usb3p0ss_ctrlRegs_asf;


typedef struct {
    volatile uint32_t HCIVERSION_CAPLENGTH;
    volatile uint32_t HCSPARAMS1;
    volatile uint32_t HCSPARAMS2;
    volatile uint32_t HCSPARAMS3;
    volatile uint32_t HCCPARAMS;
    volatile uint32_t DBOFF;
    volatile uint32_t RTSOFF;
    volatile uint32_t HCCPARAMS2;
    volatile uint32_t RSVD_CR[24];
    volatile uint32_t USBCMD;
    volatile uint32_t USBSTS;
    volatile uint32_t PAGESIZE;
    volatile uint32_t RSVDZ8C;
    volatile uint32_t RSVDZ90;
    volatile uint32_t DNCTRL;
    volatile uint32_t CRCR_LO;
    volatile uint32_t CRCR_HI;
    volatile uint32_t RSVDZA0;
    volatile uint32_t RSVDZA4;
    volatile uint32_t RSVDZA8;
    volatile uint32_t RSVDZAC;
    volatile uint32_t DCBAAP_LO;
    volatile uint32_t DCBAAP_HI;
    volatile uint32_t CONFIG;
    volatile uint32_t RSVDZBC;
    volatile uint32_t RSVDZ_OP[240];
    volatile uint32_t PORTSC1USB2;
    volatile uint32_t PORTPMSC1USB2;
    volatile uint32_t RSVDP1USB2;
    volatile uint32_t PORT1HLPMC;
    volatile uint32_t PORTSC1USB3;
    volatile uint32_t PORTPMSC1USB3;
    volatile uint32_t PORTLI1;
    volatile uint32_t RSVDP1USB3;
    volatile uint8_t  Resv_8192[7008];
    volatile uint32_t MFINDEX;
    volatile uint32_t RSVDZ_RT[7];
    volatile uint32_t IMAN0;
    volatile uint32_t IMOD0;
    volatile uint32_t ERSTSZ0;
    volatile uint32_t RESERVED0;
    volatile uint32_t ERSTBA0_LO;
    volatile uint32_t ERSTBA0_HI;
    volatile uint32_t ERDP0_LO;
    volatile uint32_t ERDP0_HI;
    volatile uint32_t IMAN1;
    volatile uint32_t IMOD1;
    volatile uint32_t ERSTSZ1;
    volatile uint32_t RESERVED1;
    volatile uint32_t ERSTBA1_LO;
    volatile uint32_t ERSTBA1_HI;
    volatile uint32_t ERDP1_LO;
    volatile uint32_t ERDP1_HI;
    volatile uint32_t IMAN2;
    volatile uint32_t IMOD2;
    volatile uint32_t ERSTSZ2;
    volatile uint32_t RESERVED2;
    volatile uint32_t ERSTBA2_LO;
    volatile uint32_t ERSTBA2_HI;
    volatile uint32_t ERDP2_LO;
    volatile uint32_t ERDP2_HI;
    volatile uint32_t IMAN3;
    volatile uint32_t IMOD3;
    volatile uint32_t ERSTSZ3;
    volatile uint32_t RESERVED3;
    volatile uint32_t ERSTBA3_LO;
    volatile uint32_t ERSTBA3_HI;
    volatile uint32_t ERDP3_LO;
    volatile uint32_t ERDP3_HI;
    volatile uint32_t IMAN4;
    volatile uint32_t IMOD4;
    volatile uint32_t ERSTSZ4;
    volatile uint32_t RESERVED4;
    volatile uint32_t ERSTBA4_LO;
    volatile uint32_t ERSTBA4_HI;
    volatile uint32_t ERDP4_LO;
    volatile uint32_t ERDP4_HI;
    volatile uint32_t IMAN5;
    volatile uint32_t IMOD5;
    volatile uint32_t ERSTSZ5;
    volatile uint32_t RESERVED5;
    volatile uint32_t ERSTBA5_LO;
    volatile uint32_t ERSTBA5_HI;
    volatile uint32_t ERDP5_LO;
    volatile uint32_t ERDP5_HI;
    volatile uint32_t IMAN6;
    volatile uint32_t IMOD6;
    volatile uint32_t ERSTSZ6;
    volatile uint32_t RESERVED6;
    volatile uint32_t ERSTBA6_LO;
    volatile uint32_t ERSTBA6_HI;
    volatile uint32_t ERDP6_LO;
    volatile uint32_t ERDP6_HI;
    volatile uint32_t IMAN7;
    volatile uint32_t IMOD7;
    volatile uint32_t ERSTSZ7;
    volatile uint32_t RESERVED7;
    volatile uint32_t ERSTBA7_LO;
    volatile uint32_t ERSTBA7_HI;
    volatile uint32_t ERDP7_LO;
    volatile uint32_t ERDP7_HI;
    volatile uint8_t  Resv_12288[3808];
    volatile uint32_t DB0;
    volatile uint32_t DB[64];
    volatile uint8_t  Resv_32768[20220];
    volatile uint32_t XECP_PORT_CAP_REG;
    volatile uint32_t XECP_PORT_1_REG;
    volatile uint32_t XECP_CDNS_DEBUG_BUS_CAP;
    volatile uint32_t XECP_CDNS_DEBUG_BUS_CTRL;
    volatile uint32_t XECP_CDNS_DEBUG_BUS_STATUS;
    volatile uint32_t XECP_PM_CAP;
    volatile uint32_t XECP_PM_PMCSR;
    volatile uint32_t XECP_MSI_CAP;
    volatile uint32_t XECP_MSI_ADDR_L;
    volatile uint32_t XECP_MSI_ADDR_H;
    volatile uint32_t XECP_MSI_DATA;
    volatile uint32_t XECP_AXI_CAP;
    volatile uint32_t XECP_AXI_CFG0;
    volatile uint32_t XECP_AXI_CTRL0;
    volatile uint32_t XECP_AXI_CTRL1;
    volatile uint32_t XECP_AXI_CTRL2;
    volatile uint32_t XECP_SUPP_USB2_CAP0;
    volatile uint32_t XECP_SUPP_USB2_CAP1;
    volatile uint32_t XECP_SUPP_USB2_CAP2;
    volatile uint32_t XECP_SUPP_USB2_PROTOCOL_SLOT_TYPE;
    volatile uint32_t XECP_PSI_FULL_SPEED;
    volatile uint32_t XECP_PSI_LOW_SPEED;
    volatile uint32_t XECP_PSI_HIGH_SPEED;
    volatile uint8_t  Resv_32864[4];
    volatile uint32_t XECP_SUPP_USB3_CAP0;
    volatile uint32_t XECP_SUPP_USB3_CAP1;
    volatile uint32_t XECP_SUPP_USB3_CAP2;
    volatile uint32_t XECP_SUPP_USB3_PROTOCOL_SLOT_TYPE;
    volatile uint32_t PSI_SUPER_SPEED;
    volatile uint8_t  Resv_32896[12];
    volatile uint32_t XECP_CMDM_STS0;
    volatile uint32_t XECP_CMDM_RESERVED_1;
    volatile uint32_t XECP_CMDM_RESERVED_2;
    volatile uint32_t XECP_CMDM_RESERVED_3;
    volatile uint32_t XECP_CMDM_RESERVED_4;
    volatile uint32_t XECP_CMDM_RESERVED_5;
    volatile uint32_t XECP_CMDM_CTRL_REG1;
    volatile uint32_t XECP_CMDM_CTRL_REG2;
    volatile uint32_t XECP_CMDM_CTRL_REG3;
    volatile uint8_t  Resv_32944[12];
    volatile uint32_t XECP_HOST_CTRL_CAP;
    volatile uint32_t XECP_HOST_CTRL_RSVD;
    volatile uint32_t XECP_HOST_CLR_MASK_REG;
    volatile uint32_t XECP_HOST_CLR_IN_EP_VALID_REG;
    volatile uint32_t XECP_HOST_CLR_PMASK_REG;
    volatile uint32_t XECP_HOST_CTRL_OCRD_REG;
    volatile uint32_t XECP_HOST_CTRL_TEST_BUS_LO;
    volatile uint32_t XECP_HOST_CTRL_TEST_BUS_HI;
    volatile uint32_t XECP_HOST_CTRL_TRM_REG1;
    volatile uint32_t XECP_HOST_CTRL_SCH_REG1;
    volatile uint32_t XECP_HOST_CTRL_ODMA_REG;
    volatile uint32_t XECP_HOST_CTRL_IDMA_REG;
    volatile uint32_t XECP_HOST_CTRL_PORT_CTRL;
    volatile uint8_t  Resv_33024[28];
    volatile uint32_t XECP_AUX_CTRL_REG;
    volatile uint32_t XECP_HOST_BW_OV_SS_REG;
    volatile uint32_t XECP_HOST_BW_OV_HS_REG;
    volatile uint32_t XECP_HOST_BW_OV_FS_LS_REG;
    volatile uint32_t XECP_HOST_BW_OV_SYS_REG;
    volatile uint32_t XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG;
    volatile uint32_t XECP_UPORTS_PON_RST_REG;
    volatile uint32_t XECP_HOST_CTRL_TRM_REG3;
    volatile uint32_t XECP_AUX_CTRL_REG1;
    volatile uint8_t  Resv_33064[4];
    volatile uint32_t XECP_HOST_CTRL_WATERMARK_REG;
    volatile uint32_t XECP_HOST_CTRL_PORT_LINK_REG;
    volatile uint32_t XECP_USB2_LINK_MGR_CTRL_REG1;
    volatile uint32_t XECP_USB2_LINK_MGR_CTRL_REG2;
    volatile uint32_t XECP_USB2_LINK_MGR_CTRL_REG3;
    volatile uint32_t XECP_USB2_LINK_MGR_CTRL_REG4;
    volatile uint32_t XECP_HOST_CTRL_BW_CTRL_REG;
    volatile uint32_t XECP_FPGA_REVISION_REG;
    volatile uint32_t XECP_HOST_INTF_CTRL_REG;
    volatile uint32_t XECP_BW_OV_SS_BURST_REG;
    volatile uint32_t XECP_HOST_CTRL_TRM_REG2;
    volatile uint8_t  Resv_33128[20];
    volatile uint32_t XECP_HOST_CTRL_BW_MAX1_REG;
    volatile uint32_t XECP_HOST_CTRL_BW_MAX2_REG;
    volatile uint32_t XECP_USB2_LINESTATE_REG;
    volatile uint32_t XECP_HOST_PROTO_GAP_TIMER1_REG;
    volatile uint32_t XECP_HOST_PROTO_GAP_TIMER2_REG;
    volatile uint32_t XECP_HOST_PROTO_BTO_TIMER_REG;
    volatile uint32_t XECP_HOST_CTRL_PSCH_REG;
    volatile uint32_t XECP_HOST_CTRL_PSCH1_REG;
    volatile uint8_t  Resv_33168[8];
    volatile uint32_t XECP_HOST_CTRL_LTM_REG;
    volatile uint32_t XECP_AUX_CTRL_REG2;
    volatile uint32_t XECP_AUX_CTRL_REG3;
    volatile uint32_t XECP_DEBUG_CTRL_REG;
    volatile uint32_t XECP_HOST_CTRL_SCH_REG2;
    volatile uint32_t XECP_AUX_DEBUG_READ_ONLY;
    volatile uint32_t XECP_AUX_CTRL_PORTNUM_REG;
    volatile uint32_t XECP_AUX_CTRL_DEV_REMOVE_REG;
    volatile uint8_t  Resv_33248[48];
    volatile uint32_t XECP_HOST_CTRL_TTE_REG1;
    volatile uint32_t XECP_HOST_CTRL_LTM_REG1;
    volatile uint32_t XECP_HOST_CTRL_LTM_REG2;
    volatile uint8_t  Resv_33280[20];
    volatile uint32_t XECP_AUX_SCRATCHPAD_0;
    volatile uint32_t XECP_AUX_SCRATCHPAD_1;
    volatile uint8_t  Resv_33296[8];
    volatile uint32_t XECP_BATTERY_CHARGE_REG;
    volatile uint32_t XECP_BATTERY_CHARGE_REG1;
    volatile uint32_t XECP_BATTERY_CHARGE_REG2;
    volatile uint32_t XECP_BATTERY_CHARGE_REG3;
    volatile uint32_t XECP_HOST_CTRL_PORT_LINK_REG1;
    volatile uint8_t  Resv_33648[332];
    volatile uint32_t XECP_USBLEGSUP;
    volatile uint32_t XECP_USBLEGCTLSTS;
    volatile uint8_t  Resv_34816[1160];
    volatile uint32_t XECP_USB3_TEST_PORT0_REG;
} CSL_usb3p0ss_ctrlRegs_xhci;


typedef struct {
    volatile uint32_t USB_CONF;
    volatile uint32_t USB_STS;
    volatile uint32_t USB_CMD;
    volatile uint32_t USB_IPTN;
    volatile uint32_t USB_LPM;
    volatile uint32_t USB_IEN;
    volatile uint32_t USB_ISTS;
    volatile uint32_t EP_SEL;
    volatile uint32_t EP_TRADDR;
    volatile uint32_t EP_CFG;
    volatile uint32_t EP_CMD;
    volatile uint32_t EP_STS;
    volatile uint32_t EP_STS_SID;
    volatile uint32_t EP_STS_EN;
    volatile uint32_t DRBL;
    volatile uint32_t EP_IEN;
    volatile uint32_t EP_ISTS;
    volatile uint32_t USB_PWR;
    volatile uint32_t USB_CONF2;
    volatile uint32_t USB_CAP1;
    volatile uint32_t USB_CAP2;
    volatile uint32_t USB_CAP3;
    volatile uint32_t USB_CAP4;
    volatile uint32_t USB_CAP5;
    volatile uint32_t USB_CAP6;
    volatile uint32_t USB_CPKT1;
    volatile uint32_t USB_CPKT2;
    volatile uint32_t USB_CPKT3;
    volatile uint32_t EP_DMA_EXT_ADDR;
    volatile uint32_t BUF_ADDR;
    volatile uint32_t BUF_DATA;
    volatile uint32_t BUF_CTRL;
    volatile uint32_t DTRANS;
    volatile uint32_t TDL_FROM_TRB;
    volatile uint32_t TDL_BEH;
    volatile uint32_t EP_TDL;
    volatile uint32_t TDL_BEH2;
    volatile uint32_t DMA_ADV_TD;
    volatile uint8_t  Resv_256[104];
    volatile uint32_t CFG_REG1;
    volatile uint32_t DBG_LINK1;
    volatile uint32_t DBG_LINK2;
    volatile uint32_t CFG_REG4;
    volatile uint32_t CFG_REG5;
    volatile uint32_t CFG_REG6;
    volatile uint32_t CFG_REG7;
    volatile uint32_t CFG_REG8;
    volatile uint32_t CFG_REG9;
    volatile uint32_t CFG_REG10;
    volatile uint32_t CFG_REG11;
    volatile uint32_t CFG_REG12;
    volatile uint32_t CFG_REG13;
    volatile uint32_t CFG_REG14;
    volatile uint32_t CFG_REG15;
    volatile uint32_t CFG_REG16;
    volatile uint32_t CFG_REG17;
    volatile uint32_t CFG_REG18;
    volatile uint32_t CFG_REG19;
    volatile uint32_t CFG_REG20;
    volatile uint32_t CFG_REG21;
    volatile uint32_t CFG_REG22;
    volatile uint32_t CFG_REG23;
    volatile uint32_t CFG_REG24;
    volatile uint32_t CFG_REG25;
    volatile uint32_t CFG_REG26;
    volatile uint32_t CFG_REG27;
    volatile uint32_t CFG_REG28;
    volatile uint32_t CFG_REG29;
    volatile uint32_t CFG_REG30;
    volatile uint32_t CFG_REG31;
    volatile uint32_t CFG_REG32;
    volatile uint32_t CFG_REG33;
    volatile uint32_t CFG_REG34;
    volatile uint32_t CFG_REG35;
    volatile uint8_t  Resv_428[32];
    volatile uint32_t CFG_REG36;
    volatile uint32_t CFG_REG37;
    volatile uint32_t CFG_REG38;
    volatile uint32_t CFG_REG39;
    volatile uint32_t CFG_REG40;
    volatile uint32_t CFG_REG41;
    volatile uint32_t CFG_REG42;
    volatile uint32_t CFG_REG43;
    volatile uint32_t CFG_REG44;
    volatile uint32_t CFG_REG45;
    volatile uint32_t CFG_REG46;
    volatile uint32_t CFG_REG47;
    volatile uint32_t CFG_REG48;
    volatile uint32_t CFG_REG49;
    volatile uint32_t CFG_REG50;
    volatile uint32_t CFG_REG51;
    volatile uint32_t CFG_REG52;
    volatile uint32_t CFG_REG53;
    volatile uint32_t CFG_REG54;
    volatile uint32_t CFG_REG55;
    volatile uint32_t CFG_REG56;
    volatile uint32_t CFG_REG57;
    volatile uint32_t CFG_REG58;
    volatile uint32_t CFG_REG59;
    volatile uint32_t CFG_REG60;
    volatile uint32_t CFG_REG61;
    volatile uint32_t CFG_REG62;
    volatile uint32_t CFG_REG63;
    volatile uint8_t  Resv_544[4];
    volatile uint32_t CFG_REG64;
    volatile uint32_t CFG_REG65;
    volatile uint32_t CFG_REG66;
    volatile uint8_t  Resv_768[212];
    volatile uint32_t DMA_AXI_CTRL;
    volatile uint32_t DMA_AXI_ID;
    volatile uint32_t DMA_AXI_CAP;
    volatile uint32_t DMA_AXI_CTRL0;
    volatile uint32_t DMA_AXI_CTRL1;
    volatile uint32_t DMA_AXI_CTRL2;
} CSL_usb3p0ss_ctrlRegs_dev;


typedef struct {
    CSL_usb3p0ss_ctrlRegs_drd DRD;
    volatile uint8_t  Resv_256[112];
    CSL_usb3p0ss_ctrlRegs_asf ASF;
    volatile uint8_t  Resv_65536[65208];
    CSL_usb3p0ss_ctrlRegs_xhci XHCI;
    volatile uint8_t  Resv_131072[30716];
    CSL_usb3p0ss_ctrlRegs_dev DEV;
} CSL_usb3p0ss_ctrlRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_USB3P0SS_CTRL_DRD_CDNS_DID                                   (0x00000000U)
#define CSL_USB3P0SS_CTRL_DRD_CDNS_RID                                   (0x00000004U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY                              (0x00000008U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD                                     (0x00000010U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS                                     (0x00000014U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE                                   (0x00000018U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN                                     (0x00000020U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT                                   (0x00000024U)
#define CSL_USB3P0SS_CTRL_DRD_CLK_FREQ                                   (0x00000028U)
#define CSL_USB3P0SS_CTRL_DRD_OTGTMR                                     (0x0000002CU)
#define CSL_USB3P0SS_CTRL_DRD_OTGSIMULATE                                (0x00000040U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE                                   (0x00000044U)
#define CSL_USB3P0SS_CTRL_DRD_SUSP_CTRL                                  (0x00000048U)
#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG                                 (0x0000004CU)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS                                  (0x00000050U)
#define CSL_USB3P0SS_CTRL_DRD_ADP_RAMP_TIME                              (0x00000054U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1                                   (0x00000058U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL2                                   (0x0000005CU)
#define CSL_USB3P0SS_CTRL_DRD_VBUSVALID_DBNC_CFG                         (0x00000060U)
#define CSL_USB3P0SS_CTRL_DRD_SESSVALID_DBNC_CFG                         (0x00000064U)
#define CSL_USB3P0SS_CTRL_DRD_IDDIG_DBNC_CFG                             (0x00000068U)
#define CSL_USB3P0SS_CTRL_DRD_DB_ASF_MEM_MASK                            (0x0000008CU)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_STATUS                             (0x00000100U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_RAW_STATUS                         (0x00000104U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_MASK                               (0x00000108U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_TEST                               (0x0000010CU)
#define CSL_USB3P0SS_CTRL_ASF_ASF_FATAL_NONFATAL_SELECT                  (0x00000110U)
#define CSL_USB3P0SS_CTRL_ASF_RSVD_6_ASF                                 (0x00000114U)
#define CSL_USB3P0SS_CTRL_ASF_RSVD_7_ASF                                 (0x00000118U)
#define CSL_USB3P0SS_CTRL_ASF_RSVD_8_ASF                                 (0x0000011CU)
#define CSL_USB3P0SS_CTRL_ASF_ASF_SRAM_CORR_FAULT_STATUS                 (0x00000120U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_SRAM_UNCORR_FAULT_STATUS               (0x00000124U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_SRAM_FAULT_STATS                       (0x00000128U)
#define CSL_USB3P0SS_CTRL_ASF_RSVD_12_ASF                                (0x0000012CU)
#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_CTRL                          (0x00000130U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_FAULT_MASK                    (0x00000134U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_FAULT_STATUS                  (0x00000138U)
#define CSL_USB3P0SS_CTRL_ASF_RSVD_16_ASF                                (0x0000013CU)
#define CSL_USB3P0SS_CTRL_ASF_RSVD_17_ASF                                (0x00000140U)
#define CSL_USB3P0SS_CTRL_ASF_RSVD_18_ASF                                (0x00000144U)
#define CSL_USB3P0SS_CTRL_XHCI_HCIVERSION_CAPLENGTH                      (0x00010000U)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS1                                (0x00010004U)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS2                                (0x00010008U)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS3                                (0x0001000CU)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS                                 (0x00010010U)
#define CSL_USB3P0SS_CTRL_XHCI_DBOFF                                     (0x00010014U)
#define CSL_USB3P0SS_CTRL_XHCI_RTSOFF                                    (0x00010018U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS2                                (0x0001001CU)
#define CSL_USB3P0SS_CTRL_XHCI_RSVD_CR(RSVD_CR)                          (0x00010020U+((RSVD_CR)*0x4U))
#define CSL_USB3P0SS_CTRL_XHCI_USBCMD                                    (0x00010080U)
#define CSL_USB3P0SS_CTRL_XHCI_USBSTS                                    (0x00010084U)
#define CSL_USB3P0SS_CTRL_XHCI_PAGESIZE                                  (0x00010088U)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDZ8C                                   (0x0001008CU)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDZ90                                   (0x00010090U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL                                    (0x00010094U)
#define CSL_USB3P0SS_CTRL_XHCI_CRCR_LO                                   (0x00010098U)
#define CSL_USB3P0SS_CTRL_XHCI_CRCR_HI                                   (0x0001009CU)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDZA0                                   (0x000100A0U)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDZA4                                   (0x000100A4U)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDZA8                                   (0x000100A8U)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDZAC                                   (0x000100ACU)
#define CSL_USB3P0SS_CTRL_XHCI_DCBAAP_LO                                 (0x000100B0U)
#define CSL_USB3P0SS_CTRL_XHCI_DCBAAP_HI                                 (0x000100B4U)
#define CSL_USB3P0SS_CTRL_XHCI_CONFIG                                    (0x000100B8U)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDZBC                                   (0x000100BCU)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDZ_OP(RSVDZ_OP)                        (0x000100C0U+((RSVDZ_OP)*0x4U))
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2                               (0x00010480U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB2                             (0x00010484U)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDP1USB2                                (0x00010488U)
#define CSL_USB3P0SS_CTRL_XHCI_PORT1HLPMC                                (0x0001048CU)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3                               (0x00010490U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB3                             (0x00010494U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTLI1                                   (0x00010498U)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDP1USB3                                (0x0001049CU)
#define CSL_USB3P0SS_CTRL_XHCI_MFINDEX                                   (0x00012000U)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDZ_RT(RSVDZ_RT)                        (0x00012004U+((RSVDZ_RT)*0x4U))
#define CSL_USB3P0SS_CTRL_XHCI_IMAN0                                     (0x00012020U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD0                                     (0x00012024U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ0                                   (0x00012028U)
#define CSL_USB3P0SS_CTRL_XHCI_RESERVED0                                 (0x0001202CU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA0_LO                                (0x00012030U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA0_HI                                (0x00012034U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP0_LO                                  (0x00012038U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP0_HI                                  (0x0001203CU)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN1                                     (0x00012040U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD1                                     (0x00012044U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ1                                   (0x00012048U)
#define CSL_USB3P0SS_CTRL_XHCI_RESERVED1                                 (0x0001204CU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA1_LO                                (0x00012050U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA1_HI                                (0x00012054U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP1_LO                                  (0x00012058U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP1_HI                                  (0x0001205CU)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN2                                     (0x00012060U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD2                                     (0x00012064U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ2                                   (0x00012068U)
#define CSL_USB3P0SS_CTRL_XHCI_RESERVED2                                 (0x0001206CU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA2_LO                                (0x00012070U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA2_HI                                (0x00012074U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP2_LO                                  (0x00012078U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP2_HI                                  (0x0001207CU)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN3                                     (0x00012080U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD3                                     (0x00012084U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ3                                   (0x00012088U)
#define CSL_USB3P0SS_CTRL_XHCI_RESERVED3                                 (0x0001208CU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA3_LO                                (0x00012090U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA3_HI                                (0x00012094U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP3_LO                                  (0x00012098U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP3_HI                                  (0x0001209CU)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN4                                     (0x000120A0U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD4                                     (0x000120A4U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ4                                   (0x000120A8U)
#define CSL_USB3P0SS_CTRL_XHCI_RESERVED4                                 (0x000120ACU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA4_LO                                (0x000120B0U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA4_HI                                (0x000120B4U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP4_LO                                  (0x000120B8U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP4_HI                                  (0x000120BCU)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN5                                     (0x000120C0U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD5                                     (0x000120C4U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ5                                   (0x000120C8U)
#define CSL_USB3P0SS_CTRL_XHCI_RESERVED5                                 (0x000120CCU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA5_LO                                (0x000120D0U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA5_HI                                (0x000120D4U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP5_LO                                  (0x000120D8U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP5_HI                                  (0x000120DCU)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN6                                     (0x000120E0U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD6                                     (0x000120E4U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ6                                   (0x000120E8U)
#define CSL_USB3P0SS_CTRL_XHCI_RESERVED6                                 (0x000120ECU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA6_LO                                (0x000120F0U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA6_HI                                (0x000120F4U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP6_LO                                  (0x000120F8U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP6_HI                                  (0x000120FCU)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN7                                     (0x00012100U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD7                                     (0x00012104U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ7                                   (0x00012108U)
#define CSL_USB3P0SS_CTRL_XHCI_RESERVED7                                 (0x0001210CU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA7_LO                                (0x00012110U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA7_HI                                (0x00012114U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP7_LO                                  (0x00012118U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP7_HI                                  (0x0001211CU)
#define CSL_USB3P0SS_CTRL_XHCI_DB0                                       (0x00013000U)
#define CSL_USB3P0SS_CTRL_XHCI_DB(DB)                                    (0x00013004U+((DB)*0x4U))
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_CAP_REG                         (0x00018000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG                           (0x00018004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CDNS_DEBUG_BUS_CAP                   (0x00018008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CDNS_DEBUG_BUS_CTRL                  (0x0001800CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CDNS_DEBUG_BUS_STATUS                (0x00018010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP                               (0x00018014U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR                             (0x00018018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_CAP                              (0x0001801CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_ADDR_L                           (0x00018020U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_ADDR_H                           (0x00018024U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_DATA                             (0x00018028U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP                              (0x0001802CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CFG0                             (0x00018030U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CTRL0                            (0x00018034U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CTRL1                            (0x00018038U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CTRL2                            (0x0001803CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP0                       (0x00018040U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP1                       (0x00018044U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2                       (0x00018048U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_PROTOCOL_SLOT_TYPE         (0x0001804CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_FULL_SPEED                       (0x00018050U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_LOW_SPEED                        (0x00018054U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_HIGH_SPEED                       (0x00018058U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP0                       (0x00018060U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP1                       (0x00018064U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP2                       (0x00018068U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_PROTOCOL_SLOT_TYPE         (0x0001806CU)
#define CSL_USB3P0SS_CTRL_XHCI_PSI_SUPER_SPEED                           (0x00018070U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0                            (0x00018080U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_RESERVED_1                      (0x00018084U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_RESERVED_2                      (0x00018088U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_RESERVED_3                      (0x0001808CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_RESERVED_4                      (0x00018090U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_RESERVED_5                      (0x00018094U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1                       (0x00018098U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2                       (0x0001809CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3                       (0x000180A0U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_CAP                        (0x000180B0U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_RSVD                       (0x000180B4U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_MASK_REG                    (0x000180B8U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_IN_EP_VALID_REG             (0x000180BCU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_PMASK_REG                   (0x000180C0U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_OCRD_REG                   (0x000180C4U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TEST_BUS_LO                (0x000180C8U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TEST_BUS_HI                (0x000180CCU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1                   (0x000180D0U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1                   (0x000180D4U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG                   (0x000180D8U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG                   (0x000180DCU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL                  (0x000180E0U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG                         (0x00018100U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_SS_REG                    (0x00018104U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_HS_REG                    (0x00018108U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_FS_LS_REG                 (0x0001810CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_SYS_REG                   (0x00018110U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG        (0x00018114U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_UPORTS_PON_RST_REG                   (0x00018118U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3                   (0x0001811CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1                        (0x00018120U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_WATERMARK_REG              (0x00018128U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG              (0x0001812CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1              (0x00018130U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG2              (0x00018134U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG3              (0x00018138U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG4              (0x0001813CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_CTRL_REG                (0x00018140U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_FPGA_REVISION_REG                    (0x00018144U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_INTF_CTRL_REG                   (0x00018148U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BW_OV_SS_BURST_REG                   (0x0001814CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2                   (0x00018150U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_MAX1_REG                (0x00018168U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_MAX2_REG                (0x0001816CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINESTATE_REG                   (0x00018170U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_GAP_TIMER1_REG            (0x00018174U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_GAP_TIMER2_REG            (0x00018178U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_BTO_TIMER_REG             (0x0001817CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PSCH_REG                   (0x00018180U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PSCH1_REG                  (0x00018184U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG                    (0x00018190U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2                        (0x00018194U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3                        (0x00018198U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG                       (0x0001819CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG2                   (0x000181A0U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_DEBUG_READ_ONLY                  (0x000181A4U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_PORTNUM_REG                 (0x000181A8U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_DEV_REMOVE_REG              (0x000181ACU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TTE_REG1                   (0x000181E0U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1                   (0x000181E4U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG2                   (0x000181E8U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_SCRATCHPAD_0                     (0x00018200U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_SCRATCHPAD_1                     (0x00018204U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG                   (0x00018210U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG1                  (0x00018214U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG2                  (0x00018218U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG3                  (0x0001821CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG1             (0x00018220U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGSUP                            (0x00018370U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS                         (0x00018374U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB3_TEST_PORT0_REG                  (0x00018800U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF                                   (0x00020000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS                                    (0x00020004U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CMD                                    (0x00020008U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IPTN                                   (0x0002000CU)
#define CSL_USB3P0SS_CTRL_DEV_USB_LPM                                    (0x00020010U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN                                    (0x00020014U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS                                   (0x00020018U)
#define CSL_USB3P0SS_CTRL_DEV_EP_SEL                                     (0x0002001CU)
#define CSL_USB3P0SS_CTRL_DEV_EP_TRADDR                                  (0x00020020U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CFG                                     (0x00020024U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CMD                                     (0x00020028U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS                                     (0x0002002CU)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_SID                                 (0x00020030U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN                                  (0x00020034U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL                                       (0x00020038U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN                                     (0x0002003CU)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS                                    (0x00020040U)
#define CSL_USB3P0SS_CTRL_DEV_USB_PWR                                    (0x00020044U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF2                                  (0x00020048U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1                                   (0x0002004CU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP2                                   (0x00020050U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP3                                   (0x00020054U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP4                                   (0x00020058U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP5                                   (0x0002005CU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP6                                   (0x00020060U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CPKT1                                  (0x00020064U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CPKT2                                  (0x00020068U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CPKT3                                  (0x0002006CU)
#define CSL_USB3P0SS_CTRL_DEV_EP_DMA_EXT_ADDR                            (0x00020070U)
#define CSL_USB3P0SS_CTRL_DEV_BUF_ADDR                                   (0x00020074U)
#define CSL_USB3P0SS_CTRL_DEV_BUF_DATA                                   (0x00020078U)
#define CSL_USB3P0SS_CTRL_DEV_BUF_CTRL                                   (0x0002007CU)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS                                     (0x00020080U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB                               (0x00020084U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH                                    (0x00020088U)
#define CSL_USB3P0SS_CTRL_DEV_EP_TDL                                     (0x0002008CU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2                                   (0x00020090U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD                                 (0x00020094U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG1                                   (0x00020100U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1                                  (0x00020104U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2                                  (0x00020108U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG4                                   (0x0002010CU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG5                                   (0x00020110U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG6                                   (0x00020114U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG7                                   (0x00020118U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG8                                   (0x0002011CU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG9                                   (0x00020120U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG10                                  (0x00020124U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG11                                  (0x00020128U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG12                                  (0x0002012CU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG13                                  (0x00020130U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG14                                  (0x00020134U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG15                                  (0x00020138U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG16                                  (0x0002013CU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG17                                  (0x00020140U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG18                                  (0x00020144U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG19                                  (0x00020148U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG20                                  (0x0002014CU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG21                                  (0x00020150U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG22                                  (0x00020154U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG23                                  (0x00020158U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG24                                  (0x0002015CU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG25                                  (0x00020160U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG26                                  (0x00020164U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG27                                  (0x00020168U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG28                                  (0x0002016CU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG29                                  (0x00020170U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG30                                  (0x00020174U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG31                                  (0x00020178U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG32                                  (0x0002017CU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG33                                  (0x00020180U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG34                                  (0x00020184U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG35                                  (0x00020188U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG36                                  (0x000201ACU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG37                                  (0x000201B0U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG38                                  (0x000201B4U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG39                                  (0x000201B8U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG40                                  (0x000201BCU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG41                                  (0x000201C0U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG42                                  (0x000201C4U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG43                                  (0x000201C8U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG44                                  (0x000201CCU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG45                                  (0x000201D0U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG46                                  (0x000201D4U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG47                                  (0x000201D8U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG48                                  (0x000201DCU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG49                                  (0x000201E0U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG50                                  (0x000201E4U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG51                                  (0x000201E8U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG52                                  (0x000201ECU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG53                                  (0x000201F0U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG54                                  (0x000201F4U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG55                                  (0x000201F8U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG56                                  (0x000201FCU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG57                                  (0x00020200U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG58                                  (0x00020204U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG59                                  (0x00020208U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG60                                  (0x0002020CU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG61                                  (0x00020210U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG62                                  (0x00020214U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG63                                  (0x00020218U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG64                                  (0x00020220U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG65                                  (0x00020224U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG66                                  (0x00020228U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL                               (0x00020300U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_ID                                 (0x00020304U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CAP                                (0x00020308U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL0                              (0x0002030CU)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL1                              (0x00020310U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL2                              (0x00020314U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* CDNS_DID */

#define CSL_USB3P0SS_CTRL_DRD_CDNS_DID_DID_MASK                          (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_DRD_CDNS_DID_DID_SHIFT                         (0x00000000U)
#define CSL_USB3P0SS_CTRL_DRD_CDNS_DID_DID_MAX                           (0xFFFFFFFFU)

/* CDNS_RID */

#define CSL_USB3P0SS_CTRL_DRD_CDNS_RID_CDNS_RID_MASK                     (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_DRD_CDNS_RID_CDNS_RID_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_CTRL_DRD_CDNS_RID_CDNS_RID_MAX                      (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_DRD_CDNS_RID_RSVD1_MASK                        (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_DRD_CDNS_RID_RSVD1_SHIFT                       (0x00000010U)
#define CSL_USB3P0SS_CTRL_DRD_CDNS_RID_RSVD1_MAX                         (0x0000FFFFU)

/* OTGCAPABILITY */

#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY_SRP_SUPPORT_MASK             (0x00000001U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY_SRP_SUPPORT_SHIFT            (0x00000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY_SRP_SUPPORT_MAX              (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY_HNP_SUPPORT_MASK             (0x00000002U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY_HNP_SUPPORT_SHIFT            (0x00000001U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY_HNP_SUPPORT_MAX              (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY_ADP_SUPPORT_MASK             (0x00000004U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY_ADP_SUPPORT_SHIFT            (0x00000002U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY_ADP_SUPPORT_MAX              (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY_BC_SUPPORT_MASK              (0x00000008U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY_BC_SUPPORT_SHIFT             (0x00000003U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY_BC_SUPPORT_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY_RSP_SUPPORT_MASK             (0x00000010U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY_RSP_SUPPORT_SHIFT            (0x00000004U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY_RSP_SUPPORT_MAX              (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY_REFCLK_DISABLE_MASK          (0x00000020U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY_REFCLK_DISABLE_SHIFT         (0x00000005U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY_REFCLK_DISABLE_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY_RSVD1_MASK                   (0x000000C0U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY_RSVD1_SHIFT                  (0x00000006U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY_RSVD1_MAX                    (0x00000003U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY_OTG2REVISION_MASK            (0x000FFF00U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY_OTG2REVISION_SHIFT           (0x00000008U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY_OTG2REVISION_MAX             (0x00000FFFU)

#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY_OTG3REVISION_MASK            (0xFFF00000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY_OTG3REVISION_SHIFT           (0x00000014U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCAPABILITY_OTG3REVISION_MAX             (0x00000FFFU)

/* OTGCMD */

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DEV_BUS_REQ_MASK                    (0x00000001U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DEV_BUS_REQ_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DEV_BUS_REQ_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_HOST_BUS_REQ_MASK                   (0x00000002U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_HOST_BUS_REQ_SHIFT                  (0x00000001U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_HOST_BUS_REQ_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_OTG_EN_MASK                         (0x00000004U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_OTG_EN_SHIFT                        (0x00000002U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_OTG_EN_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_OTG_DIS_MASK                        (0x00000008U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_OTG_DIS_SHIFT                       (0x00000003U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_OTG_DIS_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_A_DEV_EN_MASK                       (0x00000010U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_A_DEV_EN_SHIFT                      (0x00000004U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_A_DEV_EN_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_A_DEV_DIS_MASK                      (0x00000020U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_A_DEV_DIS_SHIFT                     (0x00000005U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_A_DEV_DIS_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DEV_SESS_VLD_USE_SET_MASK           (0x00000040U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DEV_SESS_VLD_USE_SET_SHIFT          (0x00000006U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DEV_SESS_VLD_USE_SET_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DEV_SESS_VLD_USE_CLR_MASK           (0x00000080U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DEV_SESS_VLD_USE_CLR_SHIFT          (0x00000007U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DEV_SESS_VLD_USE_CLR_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DEV_BUS_DROP_MASK                   (0x00000100U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DEV_BUS_DROP_SHIFT                  (0x00000008U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DEV_BUS_DROP_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_HOST_BUS_DROP_MASK                  (0x00000200U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_HOST_BUS_DROP_SHIFT                 (0x00000009U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_HOST_BUS_DROP_MAX                   (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DIS_VBUS_DROP_MASK                  (0x00000400U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DIS_VBUS_DROP_SHIFT                 (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DIS_VBUS_DROP_MAX                   (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DEV_POWER_OFF_MASK                  (0x00000800U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DEV_POWER_OFF_SHIFT                 (0x0000000BU)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DEV_POWER_OFF_MAX                   (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_HOST_POWER_OFF_MASK                 (0x00001000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_HOST_POWER_OFF_SHIFT                (0x0000000CU)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_HOST_POWER_OFF_MAX                  (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DEV_DEVEN_FORCE_SET_MASK            (0x00002000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DEV_DEVEN_FORCE_SET_SHIFT           (0x0000000DU)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DEV_DEVEN_FORCE_SET_MAX             (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DEV_DEVEN_FORCE_CLR_MASK            (0x00004000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DEV_DEVEN_FORCE_CLR_SHIFT           (0x0000000EU)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DEV_DEVEN_FORCE_CLR_MAX             (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_H_WRST_FOR_SWAP_SET_MASK            (0x00008000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_H_WRST_FOR_SWAP_SET_SHIFT           (0x0000000FU)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_H_WRST_FOR_SWAP_SET_MAX             (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_H_WRST_FOR_SWAP_CLR_MASK            (0x00010000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_H_WRST_FOR_SWAP_CLR_SHIFT           (0x00000010U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_H_WRST_FOR_SWAP_CLR_MAX             (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_D_WRST_FOR_SWAP_SET_MASK            (0x00020000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_D_WRST_FOR_SWAP_SET_SHIFT           (0x00000011U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_D_WRST_FOR_SWAP_SET_MAX             (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_D_WRST_FOR_SWAP_CLR_MASK            (0x00040000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_D_WRST_FOR_SWAP_CLR_SHIFT           (0x00000012U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_D_WRST_FOR_SWAP_CLR_MAX             (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_SS_HOST_DISABLED_SET_MASK           (0x00080000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_SS_HOST_DISABLED_SET_SHIFT          (0x00000013U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_SS_HOST_DISABLED_SET_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_SS_HOST_DISABLED_CLR_MASK           (0x00100000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_SS_HOST_DISABLED_CLR_SHIFT          (0x00000014U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_SS_HOST_DISABLED_CLR_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_SS_PERIPH_DISABLED_SET_MASK         (0x00200000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_SS_PERIPH_DISABLED_SET_SHIFT        (0x00000015U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_SS_PERIPH_DISABLED_SET_MAX          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_SS_PERIPH_DISABLED_CLR_MASK         (0x00400000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_SS_PERIPH_DISABLED_CLR_SHIFT        (0x00000016U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_SS_PERIPH_DISABLED_CLR_MAX          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_A_SET_B_HNP_EN_SET_MASK             (0x00800000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_A_SET_B_HNP_EN_SET_SHIFT            (0x00000017U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_A_SET_B_HNP_EN_SET_MAX              (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_A_SET_B_HNP_EN_CLR_MASK             (0x01000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_A_SET_B_HNP_EN_CLR_SHIFT            (0x00000018U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_A_SET_B_HNP_EN_CLR_MAX              (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_B_HNP_EN_SET_MASK                   (0x02000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_B_HNP_EN_SET_SHIFT                  (0x00000019U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_B_HNP_EN_SET_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_B_HNP_EN_CLR_MASK                   (0x04000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_B_HNP_EN_CLR_SHIFT                  (0x0000001AU)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_B_HNP_EN_CLR_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_OTG2_SWITCH_TO_PERIPH_MASK          (0x08000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_OTG2_SWITCH_TO_PERIPH_SHIFT         (0x0000001BU)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_OTG2_SWITCH_TO_PERIPH_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_INIT_SRP_MASK                       (0x10000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_INIT_SRP_SHIFT                      (0x0000001CU)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_INIT_SRP_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DEV_VBUS_DEB_SHORT_SET_MASK         (0x20000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DEV_VBUS_DEB_SHORT_SET_SHIFT        (0x0000001DU)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DEV_VBUS_DEB_SHORT_SET_MAX          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DEV_VBUS_DEB_SHORT_CLR_MASK         (0x40000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DEV_VBUS_DEB_SHORT_CLR_SHIFT        (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_DEV_VBUS_DEB_SHORT_CLR_MAX          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_RSVD2_MASK                          (0x80000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_RSVD2_SHIFT                         (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DRD_OTGCMD_RSVD2_MAX                           (0x00000001U)

/* OTGSTS */

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_ID_VALUE_MASK                       (0x00000001U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_ID_VALUE_SHIFT                      (0x00000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_ID_VALUE_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_VBUS_VALID_MASK                     (0x00000002U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_VBUS_VALID_SHIFT                    (0x00000001U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_VBUS_VALID_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_SESSION_VALID_MASK                  (0x00000004U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_SESSION_VALID_SHIFT                 (0x00000002U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_SESSION_VALID_MAX                   (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_DEV_ACTIVE_MASK                     (0x00000008U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_DEV_ACTIVE_SHIFT                    (0x00000003U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_DEV_ACTIVE_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_HOST_ACTIVE_MASK                    (0x00000010U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_HOST_ACTIVE_SHIFT                   (0x00000004U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_HOST_ACTIVE_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_OTG_IS_ENABLED_MASK                 (0x00000020U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_OTG_IS_ENABLED_SHIFT                (0x00000005U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_OTG_IS_ENABLED_MAX                  (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_OTG_MODE_MASK                       (0x00000040U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_OTG_MODE_SHIFT                      (0x00000006U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_OTG_MODE_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_SS_HOST_DISABLED_MASK               (0x00000080U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_SS_HOST_DISABLED_SHIFT              (0x00000007U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_SS_HOST_DISABLED_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_SS_PERIPH_DISABLED_MASK             (0x00000100U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_SS_PERIPH_DISABLED_SHIFT            (0x00000008U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_SS_PERIPH_DISABLED_MAX              (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_DEV_VBUS_DEB_SHORT_MASK             (0x00000200U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_DEV_VBUS_DEB_SHORT_SHIFT            (0x00000009U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_DEV_VBUS_DEB_SHORT_MAX              (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_DEV_SESS_VLD_USE_MASK               (0x00000400U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_DEV_SESS_VLD_USE_SHIFT              (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_DEV_SESS_VLD_USE_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_OTG_NRDY_MASK                       (0x00000800U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_OTG_NRDY_SHIFT                      (0x0000000BU)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_OTG_NRDY_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_STRAP_MASK                          (0x00007000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_STRAP_SHIFT                         (0x0000000CU)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_STRAP_MAX                           (0x00000007U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_H_WRST_FOR_SWAP_MASK                (0x00008000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_H_WRST_FOR_SWAP_SHIFT               (0x0000000FU)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_H_WRST_FOR_SWAP_MAX                 (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_DEV_DEVEN_FORCE_MASK                (0x00010000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_DEV_DEVEN_FORCE_SHIFT               (0x00000010U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_DEV_DEVEN_FORCE_MAX                 (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_D_WRST_FOR_SWAP_MASK                (0x00020000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_D_WRST_FOR_SWAP_SHIFT               (0x00000011U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_D_WRST_FOR_SWAP_MAX                 (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_SRP_INITIAL_CONDITION_MET_MASK      (0x00040000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_SRP_INITIAL_CONDITION_MET_SHIFT     (0x00000012U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_SRP_INITIAL_CONDITION_MET_MAX       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_SRP_DET_NOT_COMPLIANT_DEV_MASK      (0x00080000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_SRP_DET_NOT_COMPLIANT_DEV_SHIFT     (0x00000013U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_SRP_DET_NOT_COMPLIANT_DEV_MAX       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_LINESTATE_MASK                      (0x00300000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_LINESTATE_SHIFT                     (0x00000014U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_LINESTATE_MAX                       (0x00000003U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_RSVD3_MASK                          (0x00400000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_RSVD3_SHIFT                         (0x00000016U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_RSVD3_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_A_SET_B_HNP_EN_MASK                 (0x00800000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_A_SET_B_HNP_EN_SHIFT                (0x00000017U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_A_SET_B_HNP_EN_MAX                  (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_RSVD4_MASK                          (0x01000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_RSVD4_SHIFT                         (0x00000018U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_RSVD4_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_B_HNP_EN_MASK                       (0x02000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_B_HNP_EN_SHIFT                      (0x00000019U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_B_HNP_EN_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_XHC_READY_MASK                      (0x04000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_XHC_READY_SHIFT                     (0x0000001AU)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_XHC_READY_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_DEV_READY_MASK                      (0x08000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_DEV_READY_SHIFT                     (0x0000001BU)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_DEV_READY_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_IDDIG_PHY_MASK                      (0x10000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_IDDIG_PHY_SHIFT                     (0x0000001CU)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_IDDIG_PHY_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_HOST_DISCONNECT_PHY_MASK            (0x20000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_HOST_DISCONNECT_PHY_SHIFT           (0x0000001DU)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_HOST_DISCONNECT_PHY_MAX             (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_SESS_VALID_PHY_MASK                 (0x40000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_SESS_VALID_PHY_SHIFT                (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_SESS_VALID_PHY_MAX                  (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_VBUS_VALID_PHY_MASK                 (0x80000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_VBUS_VALID_PHY_SHIFT                (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTS_VBUS_VALID_PHY_MAX                  (0x00000001U)

/* OTGSTATE */

#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_DEV_OTG_STATE_MASK                (0x00000007U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_DEV_OTG_STATE_SHIFT               (0x00000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_DEV_OTG_STATE_MAX                 (0x00000007U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_HOST_OTG_STATE_MASK               (0x00000038U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_HOST_OTG_STATE_SHIFT              (0x00000003U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_HOST_OTG_STATE_MAX                (0x00000007U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_RSVD1_MASK                        (0x000000C0U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_RSVD1_SHIFT                       (0x00000006U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_RSVD1_MAX                         (0x00000003U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_APB_AXI_CTRL_MASK                 (0x00000300U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_APB_AXI_CTRL_SHIFT                (0x00000008U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_APB_AXI_CTRL_MAX                  (0x00000003U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_PIPE_CTRL_MASK                    (0x00000C00U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_PIPE_CTRL_SHIFT                   (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_PIPE_CTRL_MAX                     (0x00000003U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_UTMI_CTRL_MASK                    (0x00003000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_UTMI_CTRL_SHIFT                   (0x0000000CU)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_UTMI_CTRL_MAX                     (0x00000003U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_RSVD2_MASK                        (0x0000C000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_RSVD2_SHIFT                       (0x0000000EU)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_RSVD2_MAX                         (0x00000003U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_DEV_POWER_STATE_MASK              (0x00070000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_DEV_POWER_STATE_SHIFT             (0x00000010U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_DEV_POWER_STATE_MAX               (0x00000007U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_HOST_POWER_STATE_MASK             (0x00380000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_HOST_POWER_STATE_SHIFT            (0x00000013U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_HOST_POWER_STATE_MAX              (0x00000007U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_RSVD3_MASK                        (0xFFC00000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_RSVD3_SHIFT                       (0x00000016U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSTATE_RSVD3_MAX                         (0x000003FFU)

/* OTGIEN */

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_ID_CHANGE_INT_EN_MASK               (0x00000001U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_ID_CHANGE_INT_EN_SHIFT              (0x00000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_ID_CHANGE_INT_EN_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_VBUS_ON_FAILED_INT_EN_MASK          (0x00000002U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_VBUS_ON_FAILED_INT_EN_SHIFT         (0x00000001U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_VBUS_ON_FAILED_INT_EN_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_OTGSESSVALID_RISE_INT_EN_MASK       (0x00000004U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_OTGSESSVALID_RISE_INT_EN_SHIFT      (0x00000002U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_OTGSESSVALID_RISE_INT_EN_MAX        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_OTGSESSVALID_FALL_INT_EN_MASK       (0x00000008U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_OTGSESSVALID_FALL_INT_EN_SHIFT      (0x00000003U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_OTGSESSVALID_FALL_INT_EN_MAX        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_VBUSVALID_RISE_INT_EN_MASK          (0x00000010U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_VBUSVALID_RISE_INT_EN_SHIFT         (0x00000004U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_VBUSVALID_RISE_INT_EN_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_VBUSVALID_FALL_INT_EN_MASK          (0x00000020U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_VBUSVALID_FALL_INT_EN_SHIFT         (0x00000005U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_VBUSVALID_FALL_INT_EN_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_SENSE_RISE_INT_EN_MASK              (0x00000040U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_SENSE_RISE_INT_EN_SHIFT             (0x00000006U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_SENSE_RISE_INT_EN_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_PROBE_RISE_INT_EN_MASK              (0x00000080U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_PROBE_RISE_INT_EN_SHIFT             (0x00000007U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_PROBE_RISE_INT_EN_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_ADP_PROBE_COMPLETED_INT_EN_MASK     (0x00000100U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_ADP_PROBE_COMPLETED_INT_EN_SHIFT    (0x00000008U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_ADP_PROBE_COMPLETED_INT_EN_MAX      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_TA_AIDL_BDIS_TMOUT_INT_EN_MASK      (0x00000200U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_TA_AIDL_BDIS_TMOUT_INT_EN_SHIFT     (0x00000009U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_TA_AIDL_BDIS_TMOUT_INT_EN_MAX       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_TA_BIDL_ADIS_TMOUT_INT_EN_MASK      (0x00000400U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_TA_BIDL_ADIS_TMOUT_INT_EN_SHIFT     (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_TA_BIDL_ADIS_TMOUT_INT_EN_MAX       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_SRP_DET_INT_EN_MASK                 (0x00000800U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_SRP_DET_INT_EN_SHIFT                (0x0000000BU)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_SRP_DET_INT_EN_MAX                  (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_SRP_NOT_COMP_DEV_REMOVED_INT_EN_MASK (0x00001000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_SRP_NOT_COMP_DEV_REMOVED_INT_EN_SHIFT (0x0000000CU)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_SRP_NOT_COMP_DEV_REMOVED_INT_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_OVERCURRENT_INT_EN_MASK             (0x00002000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_OVERCURRENT_INT_EN_SHIFT            (0x0000000DU)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_OVERCURRENT_INT_EN_MAX              (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_SRP_FAIL_INT_EN_MASK                (0x00004000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_SRP_FAIL_INT_EN_SHIFT               (0x0000000EU)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_SRP_FAIL_INT_EN_MAX                 (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_SRP_CMPL_INT_EN_MASK                (0x00008000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_SRP_CMPL_INT_EN_SHIFT               (0x0000000FU)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_SRP_CMPL_INT_EN_MAX                 (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_TB_ASE0_BRST_TMOUT_INT_EN_MASK      (0x00010000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_TB_ASE0_BRST_TMOUT_INT_EN_SHIFT     (0x00000010U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_TB_ASE0_BRST_TMOUT_INT_EN_MAX       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_TB_AIDL_BDIS_MIN_TMOUT_INT_EN_MASK  (0x00020000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_TB_AIDL_BDIS_MIN_TMOUT_INT_EN_SHIFT (0x00000011U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_TB_AIDL_BDIS_MIN_TMOUT_INT_EN_MAX   (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_TIMER_TMOUT_INT_EN_MASK             (0x00040000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_TIMER_TMOUT_INT_EN_SHIFT            (0x00000012U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_TIMER_TMOUT_INT_EN_MAX              (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_H_POLL_ENTRY_INT_EN_MASK            (0x00080000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_H_POLL_ENTRY_INT_EN_SHIFT           (0x00000013U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_H_POLL_ENTRY_INT_EN_MAX             (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_H_WRST_GEN_CMPL_INT_EN_MASK         (0x00100000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_H_WRST_GEN_CMPL_INT_EN_SHIFT        (0x00000014U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_H_WRST_GEN_CMPL_INT_EN_MAX          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_RID_FLOAT_FALL_INT_EN_MASK          (0x00200000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_RID_FLOAT_FALL_INT_EN_SHIFT         (0x00000015U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_RID_FLOAT_FALL_INT_EN_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_RID_FLOAT_RISE_INT_EN_MASK          (0x00400000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_RID_FLOAT_RISE_INT_EN_SHIFT         (0x00000016U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_RID_FLOAT_RISE_INT_EN_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_RID_GND_RISE_INT_EN_MASK            (0x00800000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_RID_GND_RISE_INT_EN_SHIFT           (0x00000017U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_RID_GND_RISE_INT_EN_MAX             (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_RID_C_RISE_INT_EN_MASK              (0x01000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_RID_C_RISE_INT_EN_SHIFT             (0x00000018U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_RID_C_RISE_INT_EN_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_RID_B_RISE_INT_EN_MASK              (0x02000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_RID_B_RISE_INT_EN_SHIFT             (0x00000019U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_RID_B_RISE_INT_EN_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_RID_A_RISE_INT_EN_MASK              (0x04000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_RID_A_RISE_INT_EN_SHIFT             (0x0000001AU)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_RID_A_RISE_INT_EN_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_DM_VDAT_REF_RISE_INT_EN_MASK        (0x08000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_DM_VDAT_REF_RISE_INT_EN_SHIFT       (0x0000001BU)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_DM_VDAT_REF_RISE_INT_EN_MAX         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_DP_VDAT_REF_RISE_INT_EN_MASK        (0x10000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_DP_VDAT_REF_RISE_INT_EN_SHIFT       (0x0000001CU)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_DP_VDAT_REF_RISE_INT_EN_MAX         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_DCD_COMP_RISE_INT_EN_MASK           (0x20000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_DCD_COMP_RISE_INT_EN_SHIFT          (0x0000001DU)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_DCD_COMP_RISE_INT_EN_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_DCD_COMP_FALL_INT_EN_MASK           (0x40000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_DCD_COMP_FALL_INT_EN_SHIFT          (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_DCD_COMP_FALL_INT_EN_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_DM_VLGC_COMP_RISE_INT_EN_MASK       (0x80000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_DM_VLGC_COMP_RISE_INT_EN_SHIFT      (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DRD_OTGIEN_DM_VLGC_COMP_RISE_INT_EN_MAX        (0x00000001U)

/* OTGIVECT */

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_ID_CHANGE_INT_MASK                (0x00000001U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_ID_CHANGE_INT_SHIFT               (0x00000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_ID_CHANGE_INT_MAX                 (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_VBUS_ON_FAILED_INT_MASK           (0x00000002U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_VBUS_ON_FAILED_INT_SHIFT          (0x00000001U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_VBUS_ON_FAILED_INT_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_OTGSESSVALID_RISE_INT_MASK        (0x00000004U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_OTGSESSVALID_RISE_INT_SHIFT       (0x00000002U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_OTGSESSVALID_RISE_INT_MAX         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_OTGSESSVALID_FALL_INT_MASK        (0x00000008U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_OTGSESSVALID_FALL_INT_SHIFT       (0x00000003U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_OTGSESSVALID_FALL_INT_MAX         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_VBUSVALID_RISE_INT_MASK           (0x00000010U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_VBUSVALID_RISE_INT_SHIFT          (0x00000004U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_VBUSVALID_RISE_INT_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_VBUSVALID_FALL_INT_MASK           (0x00000020U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_VBUSVALID_FALL_INT_SHIFT          (0x00000005U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_VBUSVALID_FALL_INT_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_SENSE_RISE_INT_MASK               (0x00000040U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_SENSE_RISE_INT_SHIFT              (0x00000006U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_SENSE_RISE_INT_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_PROBE_RISE_INT_MASK               (0x00000080U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_PROBE_RISE_INT_SHIFT              (0x00000007U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_PROBE_RISE_INT_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_ADP_PROBE_COMPLETED_INT_MASK      (0x00000100U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_ADP_PROBE_COMPLETED_INT_SHIFT     (0x00000008U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_ADP_PROBE_COMPLETED_INT_MAX       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_TA_AIDL_BDIS_TMOUT_INT_MASK       (0x00000200U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_TA_AIDL_BDIS_TMOUT_INT_SHIFT      (0x00000009U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_TA_AIDL_BDIS_TMOUT_INT_MAX        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_TA_BIDL_ADIS_TMOUT_INT_MASK       (0x00000400U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_TA_BIDL_ADIS_TMOUT_INT_SHIFT      (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_TA_BIDL_ADIS_TMOUT_INT_MAX        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_SRP_DET_INT_MASK                  (0x00000800U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_SRP_DET_INT_SHIFT                 (0x0000000BU)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_SRP_DET_INT_MAX                   (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_SRP_NOT_COMP_DEV_REMOVED_INT_MASK (0x00001000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_SRP_NOT_COMP_DEV_REMOVED_INT_SHIFT (0x0000000CU)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_SRP_NOT_COMP_DEV_REMOVED_INT_MAX  (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_OVERCURRENT_INT_MASK              (0x00002000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_OVERCURRENT_INT_SHIFT             (0x0000000DU)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_OVERCURRENT_INT_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_SRP_FAIL_INT_MASK                 (0x00004000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_SRP_FAIL_INT_SHIFT                (0x0000000EU)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_SRP_FAIL_INT_MAX                  (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_SRP_CMPL_INT_MASK                 (0x00008000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_SRP_CMPL_INT_SHIFT                (0x0000000FU)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_SRP_CMPL_INT_MAX                  (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_TB_ASE0_BRST_TMOUT_INT_MASK       (0x00010000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_TB_ASE0_BRST_TMOUT_INT_SHIFT      (0x00000010U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_TB_ASE0_BRST_TMOUT_INT_MAX        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_TB_AIDL_BDIS_MIN_TMOUT_INT_MASK   (0x00020000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_TB_AIDL_BDIS_MIN_TMOUT_INT_SHIFT  (0x00000011U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_TB_AIDL_BDIS_MIN_TMOUT_INT_MAX    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_TIMER_TMOUT_INT_MASK              (0x00040000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_TIMER_TMOUT_INT_SHIFT             (0x00000012U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_TIMER_TMOUT_INT_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_H_POLLTRY_INT_MASK                (0x00080000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_H_POLLTRY_INT_SHIFT               (0x00000013U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_H_POLLTRY_INT_MAX                 (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_H_WRST_GEN_CMPL_INT_MASK          (0x00100000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_H_WRST_GEN_CMPL_INT_SHIFT         (0x00000014U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_H_WRST_GEN_CMPL_INT_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_RID_FLOAT_FALL_INT_MASK           (0x00200000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_RID_FLOAT_FALL_INT_SHIFT          (0x00000015U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_RID_FLOAT_FALL_INT_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_RID_FLOAT_RISE_INT_MASK           (0x00400000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_RID_FLOAT_RISE_INT_SHIFT          (0x00000016U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_RID_FLOAT_RISE_INT_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_RID_GND_RISE_INT_MASK             (0x00800000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_RID_GND_RISE_INT_SHIFT            (0x00000017U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_RID_GND_RISE_INT_MAX              (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_RID_C_RISE_INT_MASK               (0x01000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_RID_C_RISE_INT_SHIFT              (0x00000018U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_RID_C_RISE_INT_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_RID_B_RISE_INT_MASK               (0x02000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_RID_B_RISE_INT_SHIFT              (0x00000019U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_RID_B_RISE_INT_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_RID_A_RISE_INT_MASK               (0x04000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_RID_A_RISE_INT_SHIFT              (0x0000001AU)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_RID_A_RISE_INT_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_DM_VDAT_REF_RISE_INT_MASK         (0x08000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_DM_VDAT_REF_RISE_INT_SHIFT        (0x0000001BU)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_DM_VDAT_REF_RISE_INT_MAX          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_DP_VDAT_REF_RISE_INT_MASK         (0x10000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_DP_VDAT_REF_RISE_INT_SHIFT        (0x0000001CU)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_DP_VDAT_REF_RISE_INT_MAX          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_DCD_COMP_RISE_INT_MASK            (0x20000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_DCD_COMP_RISE_INT_SHIFT           (0x0000001DU)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_DCD_COMP_RISE_INT_MAX             (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_DCD_COMP_FALL_INT_MASK            (0x40000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_DCD_COMP_FALL_INT_SHIFT           (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_DCD_COMP_FALL_INT_MAX             (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_DM_VLGC_COMP_RISE_INT_MASK        (0x80000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_DM_VLGC_COMP_RISE_INT_SHIFT       (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DRD_OTGIVECT_DM_VLGC_COMP_RISE_INT_MAX         (0x00000001U)

/* CLK_FREQ */

#define CSL_USB3P0SS_CTRL_DRD_CLK_FREQ_RSVD1_MASK                        (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_DRD_CLK_FREQ_RSVD1_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_CTRL_DRD_CLK_FREQ_RSVD1_MAX                         (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_DRD_CLK_FREQ_CLK_FREQ_KHZ_MASK                 (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_DRD_CLK_FREQ_CLK_FREQ_KHZ_SHIFT                (0x00000010U)
#define CSL_USB3P0SS_CTRL_DRD_CLK_FREQ_CLK_FREQ_KHZ_MAX                  (0x0000FFFFU)

/* OTGTMR */

#define CSL_USB3P0SS_CTRL_DRD_OTGTMR_TIMEOUT_VALUE_MASK                  (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_DRD_OTGTMR_TIMEOUT_VALUE_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGTMR_TIMEOUT_VALUE_MAX                   (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_DRD_OTGTMR_TIMEOUT_UNITS_MASK                  (0x00030000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGTMR_TIMEOUT_UNITS_SHIFT                 (0x00000010U)
#define CSL_USB3P0SS_CTRL_DRD_OTGTMR_TIMEOUT_UNITS_MAX                   (0x00000003U)

#define CSL_USB3P0SS_CTRL_DRD_OTGTMR_TIMER_WRITE_MASK                    (0x00040000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGTMR_TIMER_WRITE_SHIFT                   (0x00000012U)
#define CSL_USB3P0SS_CTRL_DRD_OTGTMR_TIMER_WRITE_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGTMR_TIMER_START_MASK                    (0x00080000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGTMR_TIMER_START_SHIFT                   (0x00000013U)
#define CSL_USB3P0SS_CTRL_DRD_OTGTMR_TIMER_START_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGTMR_TIMER_STOP_MASK                     (0x00100000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGTMR_TIMER_STOP_SHIFT                    (0x00000014U)
#define CSL_USB3P0SS_CTRL_DRD_OTGTMR_TIMER_STOP_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGTMR_RSVD1_MASK                          (0x7FE00000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGTMR_RSVD1_SHIFT                         (0x00000015U)
#define CSL_USB3P0SS_CTRL_DRD_OTGTMR_RSVD1_MAX                           (0x000003FFU)

#define CSL_USB3P0SS_CTRL_DRD_OTGTMR_ITC_EN_MASK                         (0x80000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGTMR_ITC_EN_SHIFT                        (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DRD_OTGTMR_ITC_EN_MAX                          (0x00000001U)

/* OTGSIMULATE */

#define CSL_USB3P0SS_CTRL_DRD_OTGSIMULATE_OTG_CFG_FAST_SIMS_MASK         (0x00000001U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSIMULATE_OTG_CFG_FAST_SIMS_SHIFT        (0x00000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSIMULATE_OTG_CFG_FAST_SIMS_MAX          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGSIMULATE_RSVD1_MASK                     (0xFFFFFFFEU)
#define CSL_USB3P0SS_CTRL_DRD_OTGSIMULATE_RSVD1_SHIFT                    (0x00000001U)
#define CSL_USB3P0SS_CTRL_DRD_OTGSIMULATE_RSVD1_MAX                      (0x7FFFFFFFU)

/* OVERRIDE */

#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_IDPULLUP_MASK                     (0x00000001U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_IDPULLUP_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_IDPULLUP_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_BC_PULLDOWNCTRL_MASK              (0x00000002U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_BC_PULLDOWNCTRL_SHIFT             (0x00000001U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_BC_PULLDOWNCTRL_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_BC_DPPULLDOWN_MASK                (0x00000004U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_BC_DPPULLDOWN_SHIFT               (0x00000002U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_BC_DPPULLDOWN_MAX                 (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_BC_DMPULLDOWN_MASK                (0x00000008U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_BC_DMPULLDOWN_SHIFT               (0x00000003U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_BC_DMPULLDOWN_MAX                 (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_RSVD1_MASK                        (0x00000010U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_RSVD1_SHIFT                       (0x00000004U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_RSVD1_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_DRIVE_VBUS_SEL_MASK               (0x00000020U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_DRIVE_VBUS_SEL_SHIFT              (0x00000005U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_DRIVE_VBUS_SEL_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_DRIVE_VBUS_SFR_MASK               (0x00000040U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_DRIVE_VBUS_SFR_SHIFT              (0x00000006U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_DRIVE_VBUS_SFR_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_RSVD2_MASK                        (0x00000080U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_RSVD2_SHIFT                       (0x00000007U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_RSVD2_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_IDDIG_SEL_MASK                    (0x00000100U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_IDDIG_SEL_SHIFT                   (0x00000008U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_IDDIG_SEL_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_IDDIG_SFR_MASK                    (0x00000200U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_IDDIG_SFR_SHIFT                   (0x00000009U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_IDDIG_SFR_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_SESS_VLD_SEL_MASK                 (0x00000400U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_SESS_VLD_SEL_SHIFT                (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_SESS_VLD_SEL_MAX                  (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_SESS_VLD_SFR_MASK                 (0x00000800U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_SESS_VLD_SFR_SHIFT                (0x0000000BU)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_SESS_VLD_SFR_MAX                  (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERCURRENT_SEL_MASK              (0x00001000U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERCURRENT_SEL_SHIFT             (0x0000000CU)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERCURRENT_SEL_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERCURRENT_SFR_MASK              (0x00002000U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERCURRENT_SFR_SHIFT             (0x0000000DU)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERCURRENT_SFR_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_TERMSEL_SEL_MASK         (0x00004000U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_TERMSEL_SEL_SHIFT        (0x0000000EU)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_TERMSEL_SEL_MAX          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_TERMSEL_SFR_MASK         (0x00008000U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_TERMSEL_SFR_SHIFT        (0x0000000FU)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_TERMSEL_SFR_MAX          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_OPMODE_SEL_MASK          (0x00010000U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_OPMODE_SEL_SHIFT         (0x00000010U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_OPMODE_SEL_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_OPMODE_SFR_MASK          (0x00060000U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_OPMODE_SFR_SHIFT         (0x00000011U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_OPMODE_SFR_MAX           (0x00000003U)

#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_TXBITSTUFF_SEL_MASK      (0x00080000U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_TXBITSTUFF_SEL_SHIFT     (0x00000013U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_TXBITSTUFF_SEL_MAX       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_TXBITSTUFF_SFR_MASK      (0x00100000U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_TXBITSTUFF_SFR_SHIFT     (0x00000014U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_TXBITSTUFF_SFR_MAX       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_XCVRSEL_SEL_MASK         (0x00200000U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_XCVRSEL_SEL_SHIFT        (0x00000015U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_XCVRSEL_SEL_MAX          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_XCVRSEL_SFR_MASK         (0x00C00000U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_XCVRSEL_SFR_SHIFT        (0x00000016U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_XCVRSEL_SFR_MAX          (0x00000003U)

#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_SUSPENDM_SEL_MASK        (0x01000000U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_SUSPENDM_SEL_SHIFT       (0x00000018U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_SUSPENDM_SEL_MAX         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_SUSPENDM_SFR_MASK        (0x02000000U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_SUSPENDM_SFR_SHIFT       (0x00000019U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_SUSPENDM_SFR_MAX         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_SLEEPM_SEL_MASK          (0x04000000U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_SLEEPM_SEL_SHIFT         (0x0000001AU)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_SLEEPM_SEL_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_SLEEPM_SFR_MASK          (0x08000000U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_SLEEPM_SFR_SHIFT         (0x0000001BU)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_OVERRIDE_SLEEPM_SFR_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_RSVD3_MASK                        (0xF0000000U)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_RSVD3_SHIFT                       (0x0000001CU)
#define CSL_USB3P0SS_CTRL_DRD_OVERRIDE_RSVD3_MAX                         (0x0000000FU)

/* SUSP_CTRL */

#define CSL_USB3P0SS_CTRL_DRD_SUSP_CTRL_SUSPEND_DELAY_MASK               (0x000000FFU)
#define CSL_USB3P0SS_CTRL_DRD_SUSP_CTRL_SUSPEND_DELAY_SHIFT              (0x00000000U)
#define CSL_USB3P0SS_CTRL_DRD_SUSP_CTRL_SUSPEND_DELAY_MAX                (0x000000FFU)

#define CSL_USB3P0SS_CTRL_DRD_SUSP_CTRL_SUSPEND_RESIDENCY_MASK           (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_DRD_SUSP_CTRL_SUSPEND_RESIDENCY_SHIFT          (0x00000008U)
#define CSL_USB3P0SS_CTRL_DRD_SUSP_CTRL_SUSPEND_RESIDENCY_MAX            (0x000000FFU)

#define CSL_USB3P0SS_CTRL_DRD_SUSP_CTRL_SUSPEND_DELAY_ENABLE_MASK        (0x00010000U)
#define CSL_USB3P0SS_CTRL_DRD_SUSP_CTRL_SUSPEND_DELAY_ENABLE_SHIFT       (0x00000010U)
#define CSL_USB3P0SS_CTRL_DRD_SUSP_CTRL_SUSPEND_DELAY_ENABLE_MAX         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_SUSP_CTRL_SUSPEND_RESIDENCY_ENABLE_MASK    (0x00020000U)
#define CSL_USB3P0SS_CTRL_DRD_SUSP_CTRL_SUSPEND_RESIDENCY_ENABLE_SHIFT   (0x00000011U)
#define CSL_USB3P0SS_CTRL_DRD_SUSP_CTRL_SUSPEND_RESIDENCY_ENABLE_MAX     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_SUSP_CTRL_SLEEP_RESIDENCY_ENABLE_MASK      (0x00040000U)
#define CSL_USB3P0SS_CTRL_DRD_SUSP_CTRL_SLEEP_RESIDENCY_ENABLE_SHIFT     (0x00000012U)
#define CSL_USB3P0SS_CTRL_DRD_SUSP_CTRL_SLEEP_RESIDENCY_ENABLE_MAX       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_SUSP_CTRL_SUSPEND_DELAY_CAPABILITY_MASK    (0x00080000U)
#define CSL_USB3P0SS_CTRL_DRD_SUSP_CTRL_SUSPEND_DELAY_CAPABILITY_SHIFT   (0x00000013U)
#define CSL_USB3P0SS_CTRL_DRD_SUSP_CTRL_SUSPEND_DELAY_CAPABILITY_MAX     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_SUSP_CTRL_RSVD1_MASK                       (0xFFF00000U)
#define CSL_USB3P0SS_CTRL_DRD_SUSP_CTRL_RSVD1_SHIFT                      (0x00000014U)
#define CSL_USB3P0SS_CTRL_DRD_SUSP_CTRL_RSVD1_MAX                        (0x00000FFFU)

/* PHYRST_CFG */

#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG_PHYRST_A_ENABLE_MASK            (0x00000001U)
#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG_PHYRST_A_ENABLE_SHIFT           (0x00000000U)
#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG_PHYRST_A_ENABLE_MAX             (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG_PHYRST_B_ENABLE_MASK            (0x00000002U)
#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG_PHYRST_B_ENABLE_SHIFT           (0x00000001U)
#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG_PHYRST_B_ENABLE_MAX             (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG_PHYRST_C_ENABLE_MASK            (0x00000004U)
#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG_PHYRST_C_ENABLE_SHIFT           (0x00000002U)
#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG_PHYRST_C_ENABLE_MAX             (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG_PHYRST_D_ENABLE_MASK            (0x00000008U)
#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG_PHYRST_D_ENABLE_SHIFT           (0x00000003U)
#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG_PHYRST_D_ENABLE_MAX             (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG_RSVD1_MASK                      (0x000000F0U)
#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG_RSVD1_SHIFT                     (0x00000004U)
#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG_RSVD1_MAX                       (0x0000000FU)

#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG_PHYRST_A_VALUE_MASK             (0x00001F00U)
#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG_PHYRST_A_VALUE_SHIFT            (0x00000008U)
#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG_PHYRST_A_VALUE_MAX              (0x0000001FU)

#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG_RSVD2_MASK                      (0x00FFE000U)
#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG_RSVD2_SHIFT                     (0x0000000DU)
#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG_RSVD2_MAX                       (0x000007FFU)

#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG_PHYRST_SFR_RST_MASK             (0x01000000U)
#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG_PHYRST_SFR_RST_SHIFT            (0x00000018U)
#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG_PHYRST_SFR_RST_MAX              (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG_RSVD3_MASK                      (0xFE000000U)
#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG_RSVD3_SHIFT                     (0x00000019U)
#define CSL_USB3P0SS_CTRL_DRD_PHYRST_CFG_RSVD3_MAX                       (0x0000007FU)

/* OTGANASTS */

#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_DP_VDAT_REF_COMP_STS_MASK        (0x00000001U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_DP_VDAT_REF_COMP_STS_SHIFT       (0x00000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_DP_VDAT_REF_COMP_STS_MAX         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_DM_VDAT_REF_COMP_STS_MASK        (0x00000002U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_DM_VDAT_REF_COMP_STS_SHIFT       (0x00000001U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_DM_VDAT_REF_COMP_STS_MAX         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_DM_VLGC_COMP_STS_MASK            (0x00000004U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_DM_VLGC_COMP_STS_SHIFT           (0x00000002U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_DM_VLGC_COMP_STS_MAX             (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_DCD_COMP_STS_MASK                (0x00000008U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_DCD_COMP_STS_SHIFT               (0x00000003U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_DCD_COMP_STS_MAX                 (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_OTGSESSVALID_MASK                (0x00000010U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_OTGSESSVALID_SHIFT               (0x00000004U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_OTGSESSVALID_MAX                 (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_ADP_PROBE_ANA_MASK               (0x00000020U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_ADP_PROBE_ANA_SHIFT              (0x00000005U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_ADP_PROBE_ANA_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_ADP_SENSE_ANA_MASK               (0x00000040U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_ADP_SENSE_ANA_SHIFT              (0x00000006U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_ADP_SENSE_ANA_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_SESSEND_MASK                     (0x00000080U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_SESSEND_SHIFT                    (0x00000007U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_SESSEND_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_FLOAT_COMP_STS_MASK          (0x00000100U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_FLOAT_COMP_STS_SHIFT         (0x00000008U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_FLOAT_COMP_STS_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_GND_COMP_STS_MASK            (0x00000200U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_GND_COMP_STS_SHIFT           (0x00000009U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_GND_COMP_STS_MAX             (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_C_COMP_STS_MASK              (0x00000400U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_C_COMP_STS_SHIFT             (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_C_COMP_STS_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_B_COMP_STS_MASK              (0x00000800U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_B_COMP_STS_SHIFT             (0x0000000BU)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_B_COMP_STS_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_A_COMP_STS_MASK              (0x00001000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_A_COMP_STS_SHIFT             (0x0000000CU)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_A_COMP_STS_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_IDDIG_MASK                       (0x00002000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_IDDIG_SHIFT                      (0x0000000DU)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_IDDIG_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_LINESTATE_MASK                   (0x0000C000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_LINESTATE_SHIFT                  (0x0000000EU)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_LINESTATE_MAX                    (0x00000003U)

#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_FLOAT_MASK                   (0x00010000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_FLOAT_SHIFT                  (0x00000010U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_FLOAT_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_GND_MASK                     (0x00020000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_GND_SHIFT                    (0x00000011U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_GND_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_C_MASK                       (0x00040000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_C_SHIFT                      (0x00000012U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_C_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_B_MASK                       (0x00080000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_B_SHIFT                      (0x00000013U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_B_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_A_MASK                       (0x00100000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_A_SHIFT                      (0x00000014U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RID_A_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RSVD1_MASK                       (0x00E00000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RSVD1_SHIFT                      (0x00000015U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RSVD1_MAX                        (0x00000007U)

#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_ADP_CHRG_TMOUT_DET_MASK          (0x01000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_ADP_CHRG_TMOUT_DET_SHIFT         (0x00000018U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_ADP_CHRG_TMOUT_DET_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RSVD2_MASK                       (0xFE000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RSVD2_SHIFT                      (0x00000019U)
#define CSL_USB3P0SS_CTRL_DRD_OTGANASTS_RSVD2_MAX                        (0x0000007FU)

/* ADP_RAMP_TIME */

#define CSL_USB3P0SS_CTRL_DRD_ADP_RAMP_TIME_ADP_RAMP_TIME_MASK           (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_DRD_ADP_RAMP_TIME_ADP_RAMP_TIME_SHIFT          (0x00000000U)
#define CSL_USB3P0SS_CTRL_DRD_ADP_RAMP_TIME_ADP_RAMP_TIME_MAX            (0xFFFFFFFFU)

/* OTGCTRL1 */

#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_ADP_EN_MASK                       (0x00000001U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_ADP_EN_SHIFT                      (0x00000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_ADP_EN_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_ADP_PROBE_EN_MASK                 (0x00000002U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_ADP_PROBE_EN_SHIFT                (0x00000001U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_ADP_PROBE_EN_MAX                  (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_ADP_SENSE_EN_MASK                 (0x00000004U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_ADP_SENSE_EN_SHIFT                (0x00000002U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_ADP_SENSE_EN_MAX                  (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_ADP_SINK_CURRENT_EN_MASK          (0x00000008U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_ADP_SINK_CURRENT_EN_SHIFT         (0x00000003U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_ADP_SINK_CURRENT_EN_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_ADP_SOURCE_CURRENT_EN_MASK        (0x00000010U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_ADP_SOURCE_CURRENT_EN_SHIFT       (0x00000004U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_ADP_SOURCE_CURRENT_EN_MAX         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_DO_ADP_PRB_MASK                   (0x00000020U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_DO_ADP_PRB_SHIFT                  (0x00000005U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_DO_ADP_PRB_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_DO_ADP_SNS_MASK                   (0x00000040U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_DO_ADP_SNS_SHIFT                  (0x00000006U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_DO_ADP_SNS_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_ADP_AUTO_MASK                     (0x00000080U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_ADP_AUTO_SHIFT                    (0x00000007U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_ADP_AUTO_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_BC_EN_MASK                        (0x00000100U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_BC_EN_SHIFT                       (0x00000008U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_BC_EN_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_IDM_SINK_EN_MASK                  (0x00000200U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_IDM_SINK_EN_SHIFT                 (0x00000009U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_IDM_SINK_EN_MAX                   (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_IDP_SINK_EN_MASK                  (0x00000400U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_IDP_SINK_EN_SHIFT                 (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_IDP_SINK_EN_MAX                   (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_IDP_SRC_EN_MASK                   (0x00000800U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_IDP_SRC_EN_SHIFT                  (0x0000000BU)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_IDP_SRC_EN_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_VDM_SRC_EN_MASK                   (0x00001000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_VDM_SRC_EN_SHIFT                  (0x0000000CU)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_VDM_SRC_EN_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_VDP_SRC_EN_MASK                   (0x00002000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_VDP_SRC_EN_SHIFT                  (0x0000000DU)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_VDP_SRC_EN_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_RSVD1_MASK                        (0x0000C000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_RSVD1_SHIFT                       (0x0000000EU)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_RSVD1_MAX                         (0x00000003U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_DM_VDAT_REF_COMP_EN_MASK          (0x00010000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_DM_VDAT_REF_COMP_EN_SHIFT         (0x00000010U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_DM_VDAT_REF_COMP_EN_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_DM_VLGC_COMP_EN_MASK              (0x00020000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_DM_VLGC_COMP_EN_SHIFT             (0x00000011U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_DM_VLGC_COMP_EN_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_DP_VDAT_REF_COMP_EN_MASK          (0x00040000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_DP_VDAT_REF_COMP_EN_SHIFT         (0x00000012U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_DP_VDAT_REF_COMP_EN_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_RID_FLOAT_COMP_EN_MASK            (0x00080000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_RID_FLOAT_COMP_EN_SHIFT           (0x00000013U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_RID_FLOAT_COMP_EN_MAX             (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_RID_NONFLOAT_COMP_EN_MASK         (0x00100000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_RID_NONFLOAT_COMP_EN_SHIFT        (0x00000014U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_RID_NONFLOAT_COMP_EN_MAX          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_RSVD2_MASK                        (0xFFE00000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_RSVD2_SHIFT                       (0x00000015U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL1_RSVD2_MAX                         (0x000007FFU)

/* OTGCTRL2 */

#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL2_TA_ADP_PRB_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL2_TA_ADP_PRB_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL2_TA_ADP_PRB_MAX                    (0x000000FFU)

#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL2_TB_ADP_PRB_MASK                   (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL2_TB_ADP_PRB_SHIFT                  (0x00000008U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL2_TB_ADP_PRB_MAX                    (0x000000FFU)

#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL2_ADP_CHRG_TMOUT_MASK               (0x00FF0000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL2_ADP_CHRG_TMOUT_SHIFT              (0x00000010U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL2_ADP_CHRG_TMOUT_MAX                (0x000000FFU)

#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL2_T_ADP_DSCHG_MASK                  (0xFF000000U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL2_T_ADP_DSCHG_SHIFT                 (0x00000018U)
#define CSL_USB3P0SS_CTRL_DRD_OTGCTRL2_T_ADP_DSCHG_MAX                   (0x000000FFU)

/* VBUSVALID_DBNC_CFG */

#define CSL_USB3P0SS_CTRL_DRD_VBUSVALID_DBNC_CFG_VBUSVALID_RISE_DBNC_VAL_MASK (0x000003FFU)
#define CSL_USB3P0SS_CTRL_DRD_VBUSVALID_DBNC_CFG_VBUSVALID_RISE_DBNC_VAL_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_DRD_VBUSVALID_DBNC_CFG_VBUSVALID_RISE_DBNC_VAL_MAX (0x000003FFU)

#define CSL_USB3P0SS_CTRL_DRD_VBUSVALID_DBNC_CFG_RSVD1_MASK              (0x0000FC00U)
#define CSL_USB3P0SS_CTRL_DRD_VBUSVALID_DBNC_CFG_RSVD1_SHIFT             (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DRD_VBUSVALID_DBNC_CFG_RSVD1_MAX               (0x0000003FU)

#define CSL_USB3P0SS_CTRL_DRD_VBUSVALID_DBNC_CFG_VBUSVALID_FALL_DBNC_VAL_MASK (0x03FF0000U)
#define CSL_USB3P0SS_CTRL_DRD_VBUSVALID_DBNC_CFG_VBUSVALID_FALL_DBNC_VAL_SHIFT (0x00000010U)
#define CSL_USB3P0SS_CTRL_DRD_VBUSVALID_DBNC_CFG_VBUSVALID_FALL_DBNC_VAL_MAX (0x000003FFU)

#define CSL_USB3P0SS_CTRL_DRD_VBUSVALID_DBNC_CFG_RSVD2_MASK              (0x7C000000U)
#define CSL_USB3P0SS_CTRL_DRD_VBUSVALID_DBNC_CFG_RSVD2_SHIFT             (0x0000001AU)
#define CSL_USB3P0SS_CTRL_DRD_VBUSVALID_DBNC_CFG_RSVD2_MAX               (0x0000001FU)

#define CSL_USB3P0SS_CTRL_DRD_VBUSVALID_DBNC_CFG_VBUSVALID_DBNC_DIS_MASK (0x80000000U)
#define CSL_USB3P0SS_CTRL_DRD_VBUSVALID_DBNC_CFG_VBUSVALID_DBNC_DIS_SHIFT (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DRD_VBUSVALID_DBNC_CFG_VBUSVALID_DBNC_DIS_MAX  (0x00000001U)

/* SESSVALID_DBNC_CFG */

#define CSL_USB3P0SS_CTRL_DRD_SESSVALID_DBNC_CFG_SESSVALID_ON_DBNC_VAL_MASK (0x000003FFU)
#define CSL_USB3P0SS_CTRL_DRD_SESSVALID_DBNC_CFG_SESSVALID_ON_DBNC_VAL_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_DRD_SESSVALID_DBNC_CFG_SESSVALID_ON_DBNC_VAL_MAX (0x000003FFU)

#define CSL_USB3P0SS_CTRL_DRD_SESSVALID_DBNC_CFG_RSVD1_MASK              (0x0000FC00U)
#define CSL_USB3P0SS_CTRL_DRD_SESSVALID_DBNC_CFG_RSVD1_SHIFT             (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DRD_SESSVALID_DBNC_CFG_RSVD1_MAX               (0x0000003FU)

#define CSL_USB3P0SS_CTRL_DRD_SESSVALID_DBNC_CFG_SESSVALID_OFF_DBNC_VAL_MASK (0x03FF0000U)
#define CSL_USB3P0SS_CTRL_DRD_SESSVALID_DBNC_CFG_SESSVALID_OFF_DBNC_VAL_SHIFT (0x00000010U)
#define CSL_USB3P0SS_CTRL_DRD_SESSVALID_DBNC_CFG_SESSVALID_OFF_DBNC_VAL_MAX (0x000003FFU)

#define CSL_USB3P0SS_CTRL_DRD_SESSVALID_DBNC_CFG_RSVD2_MASK              (0x7C000000U)
#define CSL_USB3P0SS_CTRL_DRD_SESSVALID_DBNC_CFG_RSVD2_SHIFT             (0x0000001AU)
#define CSL_USB3P0SS_CTRL_DRD_SESSVALID_DBNC_CFG_RSVD2_MAX               (0x0000001FU)

#define CSL_USB3P0SS_CTRL_DRD_SESSVALID_DBNC_CFG_SESSVALID_DBNC_DIS_MASK (0x80000000U)
#define CSL_USB3P0SS_CTRL_DRD_SESSVALID_DBNC_CFG_SESSVALID_DBNC_DIS_SHIFT (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DRD_SESSVALID_DBNC_CFG_SESSVALID_DBNC_DIS_MAX  (0x00000001U)

/* IDDIG_DBNC_CFG */

#define CSL_USB3P0SS_CTRL_DRD_IDDIG_DBNC_CFG_IDDIG_ON_DBNC_VAL_MASK      (0x000003FFU)
#define CSL_USB3P0SS_CTRL_DRD_IDDIG_DBNC_CFG_IDDIG_ON_DBNC_VAL_SHIFT     (0x00000000U)
#define CSL_USB3P0SS_CTRL_DRD_IDDIG_DBNC_CFG_IDDIG_ON_DBNC_VAL_MAX       (0x000003FFU)

#define CSL_USB3P0SS_CTRL_DRD_IDDIG_DBNC_CFG_RSVD1_MASK                  (0x0000FC00U)
#define CSL_USB3P0SS_CTRL_DRD_IDDIG_DBNC_CFG_RSVD1_SHIFT                 (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DRD_IDDIG_DBNC_CFG_RSVD1_MAX                   (0x0000003FU)

#define CSL_USB3P0SS_CTRL_DRD_IDDIG_DBNC_CFG_IDDIG_OFF_DBNC_VAL_MASK     (0x03FF0000U)
#define CSL_USB3P0SS_CTRL_DRD_IDDIG_DBNC_CFG_IDDIG_OFF_DBNC_VAL_SHIFT    (0x00000010U)
#define CSL_USB3P0SS_CTRL_DRD_IDDIG_DBNC_CFG_IDDIG_OFF_DBNC_VAL_MAX      (0x000003FFU)

#define CSL_USB3P0SS_CTRL_DRD_IDDIG_DBNC_CFG_RSVD2_MASK                  (0x7C000000U)
#define CSL_USB3P0SS_CTRL_DRD_IDDIG_DBNC_CFG_RSVD2_SHIFT                 (0x0000001AU)
#define CSL_USB3P0SS_CTRL_DRD_IDDIG_DBNC_CFG_RSVD2_MAX                   (0x0000001FU)

#define CSL_USB3P0SS_CTRL_DRD_IDDIG_DBNC_CFG_IDDIG_DBNC_DIS_MASK         (0x80000000U)
#define CSL_USB3P0SS_CTRL_DRD_IDDIG_DBNC_CFG_IDDIG_DBNC_DIS_SHIFT        (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DRD_IDDIG_DBNC_CFG_IDDIG_DBNC_DIS_MAX          (0x00000001U)

/* DB_ASF_MEM_MASK */

#define CSL_USB3P0SS_CTRL_DRD_DB_ASF_MEM_MASK_DB_ASF_MEM_MASK_MASK       (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_DRD_DB_ASF_MEM_MASK_DB_ASF_MEM_MASK_SHIFT      (0x00000000U)
#define CSL_USB3P0SS_CTRL_DRD_DB_ASF_MEM_MASK_DB_ASF_MEM_MASK_MAX        (0xFFFFFFFFU)

/* ASF_INT_STATUS */

#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_STATUS_ASF_SRAM_CORR_ERR_MASK      (0x00000001U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_STATUS_ASF_SRAM_CORR_ERR_SHIFT     (0x00000000U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_STATUS_ASF_SRAM_CORR_ERR_MAX       (0x00000001U)

#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_STATUS_ASF_SRAM_UNCORR_ERR_MASK    (0x00000002U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_STATUS_ASF_SRAM_UNCORR_ERR_SHIFT   (0x00000001U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_STATUS_ASF_SRAM_UNCORR_ERR_MAX     (0x00000001U)

#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_STATUS_RSVD1_MASK                  (0x0000000CU)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_STATUS_RSVD1_SHIFT                 (0x00000002U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_STATUS_RSVD1_MAX                   (0x00000003U)

#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_STATUS_ASF_TRANS_TO_ERR_MASK       (0x00000010U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_STATUS_ASF_TRANS_TO_ERR_SHIFT      (0x00000004U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_STATUS_ASF_TRANS_TO_ERR_MAX        (0x00000001U)

#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_STATUS_RSVD2_MASK                  (0xFFFFFFE0U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_STATUS_RSVD2_SHIFT                 (0x00000005U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_STATUS_RSVD2_MAX                   (0x07FFFFFFU)

/* ASF_INT_RAW_STATUS */

#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_RAW_STATUS_ASF_SRAM_CORR_ERR_MASK  (0x00000001U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_RAW_STATUS_ASF_SRAM_CORR_ERR_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_RAW_STATUS_ASF_SRAM_CORR_ERR_MAX   (0x00000001U)

#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_RAW_STATUS_ASF_SRAM_UNCORR_ERR_MASK (0x00000002U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_RAW_STATUS_ASF_SRAM_UNCORR_ERR_SHIFT (0x00000001U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_RAW_STATUS_ASF_SRAM_UNCORR_ERR_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_RAW_STATUS_RSVD1_MASK              (0x0000000CU)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_RAW_STATUS_RSVD1_SHIFT             (0x00000002U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_RAW_STATUS_RSVD1_MAX               (0x00000003U)

#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_RAW_STATUS_ASF_TRANS_TO_ERR_MASK   (0x00000010U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_RAW_STATUS_ASF_TRANS_TO_ERR_SHIFT  (0x00000004U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_RAW_STATUS_ASF_TRANS_TO_ERR_MAX    (0x00000001U)

#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_RAW_STATUS_RSVD2_MASK              (0xFFFFFFE0U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_RAW_STATUS_RSVD2_SHIFT             (0x00000005U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_RAW_STATUS_RSVD2_MAX               (0x07FFFFFFU)

/* ASF_INT_MASK */

#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_MASK_ASF_SRAM_CORR_ERR_MASK_MASK   (0x00000001U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_MASK_ASF_SRAM_CORR_ERR_MASK_SHIFT  (0x00000000U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_MASK_ASF_SRAM_CORR_ERR_MASK_MAX    (0x00000001U)

#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_MASK_ASF_SRAM_UNCORR_ERR_MASK_MASK (0x00000002U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_MASK_ASF_SRAM_UNCORR_ERR_MASK_SHIFT (0x00000001U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_MASK_ASF_SRAM_UNCORR_ERR_MASK_MAX  (0x00000001U)

#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_MASK_RSVD1_MASK                    (0x0000000CU)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_MASK_RSVD1_SHIFT                   (0x00000002U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_MASK_RSVD1_MAX                     (0x00000003U)

#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_MASK_ASF_TRANS_TO_ERR_MASK_MASK    (0x00000010U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_MASK_ASF_TRANS_TO_ERR_MASK_SHIFT   (0x00000004U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_MASK_ASF_TRANS_TO_ERR_MASK_MAX     (0x00000001U)

#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_MASK_RSVD2_MASK                    (0xFFFFFFE0U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_MASK_RSVD2_SHIFT                   (0x00000005U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_MASK_RSVD2_MAX                     (0x07FFFFFFU)

/* ASF_INT_TEST */

#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_TEST_ASF_SRAM_CORR_ERR_TEST_MASK   (0x00000001U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_TEST_ASF_SRAM_CORR_ERR_TEST_SHIFT  (0x00000000U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_TEST_ASF_SRAM_CORR_ERR_TEST_MAX    (0x00000001U)

#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_TEST_ASF_SRAM_UNCORR_ERR_TEST_MASK (0x00000002U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_TEST_ASF_SRAM_UNCORR_ERR_TEST_SHIFT (0x00000001U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_TEST_ASF_SRAM_UNCORR_ERR_TEST_MAX  (0x00000001U)

#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_TEST_RSVD1_MASK                    (0x0000000CU)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_TEST_RSVD1_SHIFT                   (0x00000002U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_TEST_RSVD1_MAX                     (0x00000003U)

#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_TEST_ASF_TRANS_TO_ERR_TEST_MASK    (0x00000010U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_TEST_ASF_TRANS_TO_ERR_TEST_SHIFT   (0x00000004U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_TEST_ASF_TRANS_TO_ERR_TEST_MAX     (0x00000001U)

#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_TEST_RSVD2_MASK                    (0xFFFFFFE0U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_TEST_RSVD2_SHIFT                   (0x00000005U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_INT_TEST_RSVD2_MAX                     (0x07FFFFFFU)

/* ASF_FATAL_NONFATAL_SELECT */

#define CSL_USB3P0SS_CTRL_ASF_ASF_FATAL_NONFATAL_SELECT_ASF_SRAM_CORR_ERR_MASK (0x00000001U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_FATAL_NONFATAL_SELECT_ASF_SRAM_CORR_ERR_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_FATAL_NONFATAL_SELECT_ASF_SRAM_CORR_ERR_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_ASF_ASF_FATAL_NONFATAL_SELECT_ASF_SRAM_UNCORR_ERR_MASK (0x00000002U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_FATAL_NONFATAL_SELECT_ASF_SRAM_UNCORR_ERR_SHIFT (0x00000001U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_FATAL_NONFATAL_SELECT_ASF_SRAM_UNCORR_ERR_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_ASF_ASF_FATAL_NONFATAL_SELECT_RSVD1_MASK       (0x0000000CU)
#define CSL_USB3P0SS_CTRL_ASF_ASF_FATAL_NONFATAL_SELECT_RSVD1_SHIFT      (0x00000002U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_FATAL_NONFATAL_SELECT_RSVD1_MAX        (0x00000003U)

#define CSL_USB3P0SS_CTRL_ASF_ASF_FATAL_NONFATAL_SELECT_ASF_TRANS_TO_ERR_MASK (0x00000010U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_FATAL_NONFATAL_SELECT_ASF_TRANS_TO_ERR_SHIFT (0x00000004U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_FATAL_NONFATAL_SELECT_ASF_TRANS_TO_ERR_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_ASF_ASF_FATAL_NONFATAL_SELECT_RSVD2_MASK       (0xFFFFFFE0U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_FATAL_NONFATAL_SELECT_RSVD2_SHIFT      (0x00000005U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_FATAL_NONFATAL_SELECT_RSVD2_MAX        (0x07FFFFFFU)

/* RSVD_6_ASF */

#define CSL_USB3P0SS_CTRL_ASF_RSVD_6_ASF_RSVD_1_MASK                     (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_ASF_RSVD_6_ASF_RSVD_1_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_CTRL_ASF_RSVD_6_ASF_RSVD_1_MAX                      (0xFFFFFFFFU)

/* RSVD_7_ASF */

#define CSL_USB3P0SS_CTRL_ASF_RSVD_7_ASF_RSVD_1_MASK                     (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_ASF_RSVD_7_ASF_RSVD_1_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_CTRL_ASF_RSVD_7_ASF_RSVD_1_MAX                      (0xFFFFFFFFU)

/* RSVD_8_ASF */

#define CSL_USB3P0SS_CTRL_ASF_RSVD_8_ASF_RSVD_1_MASK                     (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_ASF_RSVD_8_ASF_RSVD_1_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_CTRL_ASF_RSVD_8_ASF_RSVD_1_MAX                      (0xFFFFFFFFU)

/* ASF_SRAM_CORR_FAULT_STATUS */

#define CSL_USB3P0SS_CTRL_ASF_ASF_SRAM_CORR_FAULT_STATUS_ASF_SRAM_CORR_FAULT_ADDR_MASK (0x00FFFFFFU)
#define CSL_USB3P0SS_CTRL_ASF_ASF_SRAM_CORR_FAULT_STATUS_ASF_SRAM_CORR_FAULT_ADDR_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_SRAM_CORR_FAULT_STATUS_ASF_SRAM_CORR_FAULT_ADDR_MAX (0x00FFFFFFU)

#define CSL_USB3P0SS_CTRL_ASF_ASF_SRAM_CORR_FAULT_STATUS_ASF_SRAM_CORR_FAULT_INST_MASK (0xFF000000U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_SRAM_CORR_FAULT_STATUS_ASF_SRAM_CORR_FAULT_INST_SHIFT (0x00000018U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_SRAM_CORR_FAULT_STATUS_ASF_SRAM_CORR_FAULT_INST_MAX (0x000000FFU)

/* ASF_SRAM_UNCORR_FAULT_STATUS */

#define CSL_USB3P0SS_CTRL_ASF_ASF_SRAM_UNCORR_FAULT_STATUS_ASF_SRAM_UNCORR_FAULT_ADDR_MASK (0x00FFFFFFU)
#define CSL_USB3P0SS_CTRL_ASF_ASF_SRAM_UNCORR_FAULT_STATUS_ASF_SRAM_UNCORR_FAULT_ADDR_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_SRAM_UNCORR_FAULT_STATUS_ASF_SRAM_UNCORR_FAULT_ADDR_MAX (0x00FFFFFFU)

#define CSL_USB3P0SS_CTRL_ASF_ASF_SRAM_UNCORR_FAULT_STATUS_ASF_SRAM_UNCORR_FAULT_INST_MASK (0xFF000000U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_SRAM_UNCORR_FAULT_STATUS_ASF_SRAM_UNCORR_FAULT_INST_SHIFT (0x00000018U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_SRAM_UNCORR_FAULT_STATUS_ASF_SRAM_UNCORR_FAULT_INST_MAX (0x000000FFU)

/* ASF_SRAM_FAULT_STATS */

#define CSL_USB3P0SS_CTRL_ASF_ASF_SRAM_FAULT_STATS_ASF_SRAM_FAULT_CORR_STATS_MASK (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_ASF_ASF_SRAM_FAULT_STATS_ASF_SRAM_FAULT_CORR_STATS_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_SRAM_FAULT_STATS_ASF_SRAM_FAULT_CORR_STATS_MAX (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_ASF_ASF_SRAM_FAULT_STATS_ASF_SRAM_FAULT_UNCORR_STATS_MASK (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_SRAM_FAULT_STATS_ASF_SRAM_FAULT_UNCORR_STATS_SHIFT (0x00000010U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_SRAM_FAULT_STATS_ASF_SRAM_FAULT_UNCORR_STATS_MAX (0x0000FFFFU)

/* RSVD_12_ASF */

#define CSL_USB3P0SS_CTRL_ASF_RSVD_12_ASF_RSVD_1_MASK                    (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_ASF_RSVD_12_ASF_RSVD_1_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_CTRL_ASF_RSVD_12_ASF_RSVD_1_MAX                     (0xFFFFFFFFU)

/* ASF_TRANS_TO_CTRL */

#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_CTRL_ASF_TRANS_TO_CTRL_MASK   (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_CTRL_ASF_TRANS_TO_CTRL_SHIFT  (0x00000000U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_CTRL_ASF_TRANS_TO_CTRL_MAX    (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_CTRL_RSVD1_MASK               (0x7FFF0000U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_CTRL_RSVD1_SHIFT              (0x00000010U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_CTRL_RSVD1_MAX                (0x00007FFFU)

#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_CTRL_ASF_TRANS_TO_EN_MASK     (0x80000000U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_CTRL_ASF_TRANS_TO_EN_SHIFT    (0x0000001FU)
#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_CTRL_ASF_TRANS_TO_EN_MAX      (0x00000001U)

/* ASF_TRANS_TO_FAULT_MASK */

#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_FAULT_MASK_ASF_TRANS_TO_FAULT_MASK_MASK (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_FAULT_MASK_ASF_TRANS_TO_FAULT_MASK_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_FAULT_MASK_ASF_TRANS_TO_FAULT_MASK_MAX (0xFFFFFFFFU)

/* ASF_TRANS_TO_FAULT_STATUS */

#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_FAULT_STATUS_ASF_TRANS_TIMEOUT_APB_MASK (0x00000001U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_FAULT_STATUS_ASF_TRANS_TIMEOUT_APB_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_FAULT_STATUS_ASF_TRANS_TIMEOUT_APB_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_FAULT_STATUS_ASF_TRANS_TIMEOUT_AWAXI_MASK (0x00000002U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_FAULT_STATUS_ASF_TRANS_TIMEOUT_AWAXI_SHIFT (0x00000001U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_FAULT_STATUS_ASF_TRANS_TIMEOUT_AWAXI_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_FAULT_STATUS_ASF_TRANS_TIMEOUT_WAXI_MASK (0x00000004U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_FAULT_STATUS_ASF_TRANS_TIMEOUT_WAXI_SHIFT (0x00000002U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_FAULT_STATUS_ASF_TRANS_TIMEOUT_WAXI_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_FAULT_STATUS_ASF_TRANS_TIMEOUT_ARAXI_MASK (0x00000008U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_FAULT_STATUS_ASF_TRANS_TIMEOUT_ARAXI_SHIFT (0x00000003U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_FAULT_STATUS_ASF_TRANS_TIMEOUT_ARAXI_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_FAULT_STATUS_ASF_TRANS_TIMEOUT_RAXI_MASK (0x00000010U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_FAULT_STATUS_ASF_TRANS_TIMEOUT_RAXI_SHIFT (0x00000004U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_FAULT_STATUS_ASF_TRANS_TIMEOUT_RAXI_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_FAULT_STATUS_ASF_TRANS_TIMEOUT_BAXI_MASK (0x00000020U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_FAULT_STATUS_ASF_TRANS_TIMEOUT_BAXI_SHIFT (0x00000005U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_FAULT_STATUS_ASF_TRANS_TIMEOUT_BAXI_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_FAULT_STATUS_RSVD1_MASK       (0xFFFFFFC0U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_FAULT_STATUS_RSVD1_SHIFT      (0x00000006U)
#define CSL_USB3P0SS_CTRL_ASF_ASF_TRANS_TO_FAULT_STATUS_RSVD1_MAX        (0x03FFFFFFU)

/* RSVD_16_ASF */

#define CSL_USB3P0SS_CTRL_ASF_RSVD_16_ASF_RSVD_1_MASK                    (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_ASF_RSVD_16_ASF_RSVD_1_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_CTRL_ASF_RSVD_16_ASF_RSVD_1_MAX                     (0xFFFFFFFFU)

/* RSVD_17_ASF */

#define CSL_USB3P0SS_CTRL_ASF_RSVD_17_ASF_RSVD_1_MASK                    (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_ASF_RSVD_17_ASF_RSVD_1_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_CTRL_ASF_RSVD_17_ASF_RSVD_1_MAX                     (0xFFFFFFFFU)

/* RSVD_18_ASF */

#define CSL_USB3P0SS_CTRL_ASF_RSVD_18_ASF_RSVD_1_MASK                    (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_ASF_RSVD_18_ASF_RSVD_1_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_CTRL_ASF_RSVD_18_ASF_RSVD_1_MAX                     (0xFFFFFFFFU)

/* HCIVERSION_CAPLENGTH */

#define CSL_USB3P0SS_CTRL_XHCI_HCIVERSION_CAPLENGTH_CAPLENGTH_MASK       (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_HCIVERSION_CAPLENGTH_CAPLENGTH_SHIFT      (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_HCIVERSION_CAPLENGTH_CAPLENGTH_MAX        (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_HCIVERSION_CAPLENGTH_RSVD1_MASK           (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_XHCI_HCIVERSION_CAPLENGTH_RSVD1_SHIFT          (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_HCIVERSION_CAPLENGTH_RSVD1_MAX            (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_HCIVERSION_CAPLENGTH_HCIVERSION_MASK      (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_HCIVERSION_CAPLENGTH_HCIVERSION_SHIFT     (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_HCIVERSION_CAPLENGTH_HCIVERSION_MAX       (0x0000FFFFU)

/* HCSPARAMS1 */

#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS1_MAXSLOTS_MASK                  (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS1_MAXSLOTS_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS1_MAXSLOTS_MAX                   (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS1_MAXINTRS_MASK                  (0x0007FF00U)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS1_MAXINTRS_SHIFT                 (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS1_MAXINTRS_MAX                   (0x000007FFU)

#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS1_RSVD1_MASK                     (0x00F80000U)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS1_RSVD1_SHIFT                    (0x00000013U)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS1_RSVD1_MAX                      (0x0000001FU)

#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS1_MAXPORTS_MASK                  (0xFF000000U)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS1_MAXPORTS_SHIFT                 (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS1_MAXPORTS_MAX                   (0x000000FFU)

/* HCSPARAMS2 */

#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS2_IST_MASK                       (0x0000000FU)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS2_IST_SHIFT                      (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS2_IST_MAX                        (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS2_ERSTMAX_MASK                   (0x000000F0U)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS2_ERSTMAX_SHIFT                  (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS2_ERSTMAX_MAX                    (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS2_RSVD1_MASK                     (0x001FFF00U)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS2_RSVD1_SHIFT                    (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS2_RSVD1_MAX                      (0x00001FFFU)

#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS2_MAXSPBUFHI_MASK                (0x03E00000U)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS2_MAXSPBUFHI_SHIFT               (0x00000015U)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS2_MAXSPBUFHI_MAX                 (0x0000001FU)

#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS2_SPR_MASK                       (0x04000000U)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS2_SPR_SHIFT                      (0x0000001AU)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS2_SPR_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS2_MAXSPBUFLO_MASK                (0xF8000000U)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS2_MAXSPBUFLO_SHIFT               (0x0000001BU)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS2_MAXSPBUFLO_MAX                 (0x0000001FU)

/* HCSPARAMS3 */

#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS3_U1DEVEXITLAT_MASK              (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS3_U1DEVEXITLAT_SHIFT             (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS3_U1DEVEXITLAT_MAX               (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS3_RSVD1_MASK                     (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS3_RSVD1_SHIFT                    (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS3_RSVD1_MAX                      (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS3_U2DEVEXITLAT_MASK              (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS3_U2DEVEXITLAT_SHIFT             (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_HCSPARAMS3_U2DEVEXITLAT_MAX               (0x0000FFFFU)

/* HCCPARAMS */

#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_AC64_MASK                       (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_AC64_SHIFT                      (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_AC64_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_BNC_MASK                        (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_BNC_SHIFT                       (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_BNC_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_CSZ_MASK                        (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_CSZ_SHIFT                       (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_CSZ_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_PPC_MASK                        (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_PPC_SHIFT                       (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_PPC_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_PIND_MASK                       (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_PIND_SHIFT                      (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_PIND_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_LHRC_MASK                       (0x00000020U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_LHRC_SHIFT                      (0x00000005U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_LHRC_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_LTC_MASK                        (0x00000040U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_LTC_SHIFT                       (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_LTC_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_NSS_MASK                        (0x00000080U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_NSS_SHIFT                       (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_NSS_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_PAE_MASK                        (0x00000100U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_PAE_SHIFT                       (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_PAE_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_SPC_MASK                        (0x00000200U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_SPC_SHIFT                       (0x00000009U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_SPC_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_RSVD1_MASK                      (0x00000C00U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_RSVD1_SHIFT                     (0x0000000AU)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_RSVD1_MAX                       (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_MAXPSASIZE_MASK                 (0x0000F000U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_MAXPSASIZE_SHIFT                (0x0000000CU)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_MAXPSASIZE_MAX                  (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_XECP_MASK                       (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_XECP_SHIFT                      (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS_XECP_MAX                        (0x0000FFFFU)

/* DBOFF */

#define CSL_USB3P0SS_CTRL_XHCI_DBOFF_RSVD1_MASK                          (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_DBOFF_RSVD1_SHIFT                         (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_DBOFF_RSVD1_MAX                           (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_DBOFF_DAO_MASK                            (0xFFFFFFFCU)
#define CSL_USB3P0SS_CTRL_XHCI_DBOFF_DAO_SHIFT                           (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_DBOFF_DAO_MAX                             (0x3FFFFFFFU)

/* RTSOFF */

#define CSL_USB3P0SS_CTRL_XHCI_RTSOFF_RSVD1_MASK                         (0x0000001FU)
#define CSL_USB3P0SS_CTRL_XHCI_RTSOFF_RSVD1_SHIFT                        (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_RTSOFF_RSVD1_MAX                          (0x0000001FU)

#define CSL_USB3P0SS_CTRL_XHCI_RTSOFF_RRSO_MASK                          (0xFFFFFFE0U)
#define CSL_USB3P0SS_CTRL_XHCI_RTSOFF_RRSO_SHIFT                         (0x00000005U)
#define CSL_USB3P0SS_CTRL_XHCI_RTSOFF_RRSO_MAX                           (0x07FFFFFFU)

/* HCCPARAMS2 */

#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS2_RSVD1_MASK                     (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS2_RSVD1_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS2_RSVD1_MAX                      (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS2_CTC_MASK                       (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS2_CTC_SHIFT                      (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS2_CTC_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS2_RSVD2_MASK                     (0xFFFFFFF0U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS2_RSVD2_SHIFT                    (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_HCCPARAMS2_RSVD2_MAX                      (0x0FFFFFFFU)

/* RSVD_CR */

#define CSL_USB3P0SS_CTRL_XHCI_RSVD_CR_RSVD_1_MASK                       (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_RSVD_CR_RSVD_1_SHIFT                      (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_RSVD_CR_RSVD_1_MAX                        (0xFFFFFFFFU)

/* USBCMD */

#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_R_S_MASK                           (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_R_S_SHIFT                          (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_R_S_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_HCRST_MASK                         (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_HCRST_SHIFT                        (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_HCRST_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_INTE_MASK                          (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_INTE_SHIFT                         (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_INTE_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_HSEE_MASK                          (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_HSEE_SHIFT                         (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_HSEE_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_RSVDP1_MASK                        (0x00000070U)
#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_RSVDP1_SHIFT                       (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_RSVDP1_MAX                         (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_LHCRST_MASK                        (0x00000080U)
#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_LHCRST_SHIFT                       (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_LHCRST_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_CSS_MASK                           (0x00000100U)
#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_CSS_SHIFT                          (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_CSS_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_CRS_MASK                           (0x00000200U)
#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_CRS_SHIFT                          (0x00000009U)
#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_CRS_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_EWE_MASK                           (0x00000400U)
#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_EWE_SHIFT                          (0x0000000AU)
#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_EWE_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_EU3S_MASK                          (0x00000800U)
#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_EU3S_SHIFT                         (0x0000000BU)
#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_EU3S_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_RSVDP2_MASK                        (0xFFFFF000U)
#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_RSVDP2_SHIFT                       (0x0000000CU)
#define CSL_USB3P0SS_CTRL_XHCI_USBCMD_RSVDP2_MAX                         (0x000FFFFFU)

/* USBSTS */

#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_HCH_MASK                           (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_HCH_SHIFT                          (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_HCH_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_RSVDZ1_MASK                        (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_RSVDZ1_SHIFT                       (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_RSVDZ1_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_HSE_MASK                           (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_HSE_SHIFT                          (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_HSE_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_EINT_MASK                          (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_EINT_SHIFT                         (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_EINT_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_PCD_MASK                           (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_PCD_SHIFT                          (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_PCD_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_RSVDZ2_MASK                        (0x000000E0U)
#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_RSVDZ2_SHIFT                       (0x00000005U)
#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_RSVDZ2_MAX                         (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_SSS_MASK                           (0x00000100U)
#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_SSS_SHIFT                          (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_SSS_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_RSS_MASK                           (0x00000200U)
#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_RSS_SHIFT                          (0x00000009U)
#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_RSS_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_SRE_MASK                           (0x00000400U)
#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_SRE_SHIFT                          (0x0000000AU)
#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_SRE_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_CNR_MASK                           (0x00000800U)
#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_CNR_SHIFT                          (0x0000000BU)
#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_CNR_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_HCE_MASK                           (0x00001000U)
#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_HCE_SHIFT                          (0x0000000CU)
#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_HCE_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_RSVDP_MASK                         (0xFFFFE000U)
#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_RSVDP_SHIFT                        (0x0000000DU)
#define CSL_USB3P0SS_CTRL_XHCI_USBSTS_RSVDP_MAX                          (0x0007FFFFU)

/* PAGESIZE */

#define CSL_USB3P0SS_CTRL_XHCI_PAGESIZE_PAGESIZE_MASK                    (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_PAGESIZE_PAGESIZE_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_PAGESIZE_PAGESIZE_MAX                     (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_PAGESIZE_RSVD1_MASK                       (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_PAGESIZE_RSVD1_SHIFT                      (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_PAGESIZE_RSVD1_MAX                        (0x0000FFFFU)

/* RSVDZ8C */

#define CSL_USB3P0SS_CTRL_XHCI_RSVDZ8C_RSVDZ_1_MASK                      (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDZ8C_RSVDZ_1_SHIFT                     (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDZ8C_RSVDZ_1_MAX                       (0xFFFFFFFFU)

/* RSVDZ90 */

#define CSL_USB3P0SS_CTRL_XHCI_RSVDZ90_RSVDZ_1_MASK                      (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDZ90_RSVDZ_1_SHIFT                     (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDZ90_RSVDZ_1_MAX                       (0xFFFFFFFFU)

/* DNCTRL */

#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N0_MASK                            (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N0_SHIFT                           (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N0_MAX                             (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N1_MASK                            (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N1_SHIFT                           (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N1_MAX                             (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N2_MASK                            (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N2_SHIFT                           (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N2_MAX                             (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N3_MASK                            (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N3_SHIFT                           (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N3_MAX                             (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N4_MASK                            (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N4_SHIFT                           (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N4_MAX                             (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N5_MASK                            (0x00000020U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N5_SHIFT                           (0x00000005U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N5_MAX                             (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N6_MASK                            (0x00000040U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N6_SHIFT                           (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N6_MAX                             (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N7_MASK                            (0x00000080U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N7_SHIFT                           (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N7_MAX                             (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N8_MASK                            (0x00000100U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N8_SHIFT                           (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N8_MAX                             (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N9_MASK                            (0x00000200U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N9_SHIFT                           (0x00000009U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N9_MAX                             (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N10_MASK                           (0x00000400U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N10_SHIFT                          (0x0000000AU)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N10_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N11_MASK                           (0x00000800U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N11_SHIFT                          (0x0000000BU)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N11_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N12_MASK                           (0x00001000U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N12_SHIFT                          (0x0000000CU)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N12_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N13_MASK                           (0x00002000U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N13_SHIFT                          (0x0000000DU)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N13_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N14_MASK                           (0x00004000U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N14_SHIFT                          (0x0000000EU)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N14_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N15_MASK                           (0x00008000U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N15_SHIFT                          (0x0000000FU)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_N15_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_RSVDP1_MASK                        (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_RSVDP1_SHIFT                       (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_DNCTRL_RSVDP1_MAX                         (0x0000FFFFU)

/* CRCR_LO */

#define CSL_USB3P0SS_CTRL_XHCI_CRCR_LO_RCS_MASK                          (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_CRCR_LO_RCS_SHIFT                         (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_CRCR_LO_RCS_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_CRCR_LO_CS_MASK                           (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_CRCR_LO_CS_SHIFT                          (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_CRCR_LO_CS_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_CRCR_LO_CA_MASK                           (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_CRCR_LO_CA_SHIFT                          (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_CRCR_LO_CA_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_CRCR_LO_CRR_MASK                          (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_CRCR_LO_CRR_SHIFT                         (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_CRCR_LO_CRR_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_CRCR_LO_RSVD1_MASK                        (0x00000030U)
#define CSL_USB3P0SS_CTRL_XHCI_CRCR_LO_RSVD1_SHIFT                       (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_CRCR_LO_RSVD1_MAX                         (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_CRCR_LO_CRPTR_L_MASK                      (0xFFFFFFC0U)
#define CSL_USB3P0SS_CTRL_XHCI_CRCR_LO_CRPTR_L_SHIFT                     (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_CRCR_LO_CRPTR_L_MAX                       (0x03FFFFFFU)

/* CRCR_HI */

#define CSL_USB3P0SS_CTRL_XHCI_CRCR_HI_CRPTR_H_MASK                      (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_CRCR_HI_CRPTR_H_SHIFT                     (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_CRCR_HI_CRPTR_H_MAX                       (0xFFFFFFFFU)

/* RSVDZA0 */

#define CSL_USB3P0SS_CTRL_XHCI_RSVDZA0_RSVDZ_1_MASK                      (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDZA0_RSVDZ_1_SHIFT                     (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDZA0_RSVDZ_1_MAX                       (0xFFFFFFFFU)

/* RSVDZA4 */

#define CSL_USB3P0SS_CTRL_XHCI_RSVDZA4_RSVDZ_1_MASK                      (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDZA4_RSVDZ_1_SHIFT                     (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDZA4_RSVDZ_1_MAX                       (0xFFFFFFFFU)

/* RSVDZA8 */

#define CSL_USB3P0SS_CTRL_XHCI_RSVDZA8_RSVDZ_1_MASK                      (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDZA8_RSVDZ_1_SHIFT                     (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDZA8_RSVDZ_1_MAX                       (0xFFFFFFFFU)

/* RSVDZAC */

#define CSL_USB3P0SS_CTRL_XHCI_RSVDZAC_RSVDZ_1_MASK                      (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDZAC_RSVDZ_1_SHIFT                     (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDZAC_RSVDZ_1_MAX                       (0xFFFFFFFFU)

/* DCBAAP_LO */

#define CSL_USB3P0SS_CTRL_XHCI_DCBAAP_LO_RSVDZ1_MASK                     (0x0000003FU)
#define CSL_USB3P0SS_CTRL_XHCI_DCBAAP_LO_RSVDZ1_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_DCBAAP_LO_RSVDZ1_MAX                      (0x0000003FU)

#define CSL_USB3P0SS_CTRL_XHCI_DCBAAP_LO_DCBAAPTR_L_MASK                 (0xFFFFFFC0U)
#define CSL_USB3P0SS_CTRL_XHCI_DCBAAP_LO_DCBAAPTR_L_SHIFT                (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_DCBAAP_LO_DCBAAPTR_L_MAX                  (0x03FFFFFFU)

/* DCBAAP_HI */

#define CSL_USB3P0SS_CTRL_XHCI_DCBAAP_HI_DCBAAPTR_H_MASK                 (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_DCBAAP_HI_DCBAAPTR_H_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_DCBAAP_HI_DCBAAPTR_H_MAX                  (0xFFFFFFFFU)

/* CONFIG */

#define CSL_USB3P0SS_CTRL_XHCI_CONFIG_MAXSLOTSEN_MASK                    (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_CONFIG_MAXSLOTSEN_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_CONFIG_MAXSLOTSEN_MAX                     (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_CONFIG_RSVDP1_MASK                        (0xFFFFFF00U)
#define CSL_USB3P0SS_CTRL_XHCI_CONFIG_RSVDP1_SHIFT                       (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_CONFIG_RSVDP1_MAX                         (0x00FFFFFFU)

/* RSVDZBC */

#define CSL_USB3P0SS_CTRL_XHCI_RSVDZBC_RSVDZ_1_MASK                      (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDZBC_RSVDZ_1_SHIFT                     (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDZBC_RSVDZ_1_MAX                       (0xFFFFFFFFU)

/* RSVDZ_OP */

#define CSL_USB3P0SS_CTRL_XHCI_RSVDZ_OP_RSVDZ_1_MASK                     (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDZ_OP_RSVDZ_1_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDZ_OP_RSVDZ_1_MAX                      (0xFFFFFFFFU)

/* PORTSC1USB2 */

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_CCS_MASK                      (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_CCS_SHIFT                     (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_CCS_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_PED_MASK                      (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_PED_SHIFT                     (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_PED_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_RSVDZ_1_MASK                  (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_RSVDZ_1_SHIFT                 (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_RSVDZ_1_MAX                   (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_OCA_MASK                      (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_OCA_SHIFT                     (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_OCA_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_PR_MASK                       (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_PR_SHIFT                      (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_PR_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_PLS_MASK                      (0x000001E0U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_PLS_SHIFT                     (0x00000005U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_PLS_MAX                       (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_PP_MASK                       (0x00000200U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_PP_SHIFT                      (0x00000009U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_PP_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_PORTSPEED_MASK                (0x00003C00U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_PORTSPEED_SHIFT               (0x0000000AU)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_PORTSPEED_MAX                 (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_PIC_MASK                      (0x0000C000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_PIC_SHIFT                     (0x0000000EU)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_PIC_MAX                       (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_LWS_MASK                      (0x00010000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_LWS_SHIFT                     (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_LWS_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_CSC_MASK                      (0x00020000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_CSC_SHIFT                     (0x00000011U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_CSC_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_PEC_MASK                      (0x00040000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_PEC_SHIFT                     (0x00000012U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_PEC_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_WRC_MASK                      (0x00080000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_WRC_SHIFT                     (0x00000013U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_WRC_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_OCC_MASK                      (0x00100000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_OCC_SHIFT                     (0x00000014U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_OCC_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_PRC_MASK                      (0x00200000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_PRC_SHIFT                     (0x00000015U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_PRC_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_PLC_MASK                      (0x00400000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_PLC_SHIFT                     (0x00000016U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_PLC_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_RSVDZ_2_MASK                  (0x00800000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_RSVDZ_2_SHIFT                 (0x00000017U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_RSVDZ_2_MAX                   (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_CAS_MASK                      (0x01000000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_CAS_SHIFT                     (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_CAS_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_WCE_MASK                      (0x02000000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_WCE_SHIFT                     (0x00000019U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_WCE_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_WDE_MASK                      (0x04000000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_WDE_SHIFT                     (0x0000001AU)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_WDE_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_WOE_MASK                      (0x08000000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_WOE_SHIFT                     (0x0000001BU)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_WOE_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_RSVDZ_3_MASK                  (0x30000000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_RSVDZ_3_SHIFT                 (0x0000001CU)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_RSVDZ_3_MAX                   (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_DR_MASK                       (0x40000000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_DR_SHIFT                      (0x0000001EU)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_DR_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_RSVDZ_4_MASK                  (0x80000000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_RSVDZ_4_SHIFT                 (0x0000001FU)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB2_RSVDZ_4_MAX                   (0x00000001U)

/* PORTPMSC1USB2 */

#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB2_L1S_MASK                    (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB2_L1S_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB2_L1S_MAX                     (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB2_RWE_MASK                    (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB2_RWE_SHIFT                   (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB2_RWE_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB2_BESL_MASK                   (0x000000F0U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB2_BESL_SHIFT                  (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB2_BESL_MAX                    (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB2_L1DS_MASK                   (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB2_L1DS_SHIFT                  (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB2_L1DS_MAX                    (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB2_HLE_MASK                    (0x00010000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB2_HLE_SHIFT                   (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB2_HLE_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB2_RSVDP_MASK                  (0x0FFE0000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB2_RSVDP_SHIFT                 (0x00000011U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB2_RSVDP_MAX                   (0x000007FFU)

#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB2_PTC_MASK                    (0xF0000000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB2_PTC_SHIFT                   (0x0000001CU)
#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB2_PTC_MAX                     (0x0000000FU)

/* RSVDP1USB2 */

#define CSL_USB3P0SS_CTRL_XHCI_RSVDP1USB2_RSVDP_1_MASK                   (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDP1USB2_RSVDP_1_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDP1USB2_RSVDP_1_MAX                    (0xFFFFFFFFU)

/* PORT1HLPMC */

#define CSL_USB3P0SS_CTRL_XHCI_PORT1HLPMC_HIRDM_MASK                     (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_PORT1HLPMC_HIRDM_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORT1HLPMC_HIRDM_MAX                      (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_PORT1HLPMC_L1_TIMEOUT_MASK                (0x000003FCU)
#define CSL_USB3P0SS_CTRL_XHCI_PORT1HLPMC_L1_TIMEOUT_SHIFT               (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_PORT1HLPMC_L1_TIMEOUT_MAX                 (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_PORT1HLPMC_BESLD_MASK                     (0x00003C00U)
#define CSL_USB3P0SS_CTRL_XHCI_PORT1HLPMC_BESLD_SHIFT                    (0x0000000AU)
#define CSL_USB3P0SS_CTRL_XHCI_PORT1HLPMC_BESLD_MAX                      (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_PORT1HLPMC_RSVDP_MASK                     (0xFFFFC000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORT1HLPMC_RSVDP_SHIFT                    (0x0000000EU)
#define CSL_USB3P0SS_CTRL_XHCI_PORT1HLPMC_RSVDP_MAX                      (0x0003FFFFU)

/* PORTSC1USB3 */

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_CCS_MASK                      (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_CCS_SHIFT                     (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_CCS_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_PED_MASK                      (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_PED_SHIFT                     (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_PED_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_RSVDZ_1_MASK                  (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_RSVDZ_1_SHIFT                 (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_RSVDZ_1_MAX                   (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_OCA_MASK                      (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_OCA_SHIFT                     (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_OCA_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_PR_MASK                       (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_PR_SHIFT                      (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_PR_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_PLS_MASK                      (0x000001E0U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_PLS_SHIFT                     (0x00000005U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_PLS_MAX                       (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_PP_MASK                       (0x00000200U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_PP_SHIFT                      (0x00000009U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_PP_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_PORTSPEED_MASK                (0x00003C00U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_PORTSPEED_SHIFT               (0x0000000AU)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_PORTSPEED_MAX                 (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_PIC_MASK                      (0x0000C000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_PIC_SHIFT                     (0x0000000EU)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_PIC_MAX                       (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_LWS_MASK                      (0x00010000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_LWS_SHIFT                     (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_LWS_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_CSC_MASK                      (0x00020000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_CSC_SHIFT                     (0x00000011U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_CSC_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_PEC_MASK                      (0x00040000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_PEC_SHIFT                     (0x00000012U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_PEC_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_WRC_MASK                      (0x00080000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_WRC_SHIFT                     (0x00000013U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_WRC_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_OCC_MASK                      (0x00100000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_OCC_SHIFT                     (0x00000014U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_OCC_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_PRC_MASK                      (0x00200000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_PRC_SHIFT                     (0x00000015U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_PRC_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_PLC_MASK                      (0x00400000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_PLC_SHIFT                     (0x00000016U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_PLC_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_CEC_MASK                      (0x00800000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_CEC_SHIFT                     (0x00000017U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_CEC_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_CAS_MASK                      (0x01000000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_CAS_SHIFT                     (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_CAS_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_WCE_MASK                      (0x02000000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_WCE_SHIFT                     (0x00000019U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_WCE_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_WDE_MASK                      (0x04000000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_WDE_SHIFT                     (0x0000001AU)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_WDE_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_WOE_MASK                      (0x08000000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_WOE_SHIFT                     (0x0000001BU)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_WOE_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_RSVDZ_2_MASK                  (0x30000000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_RSVDZ_2_SHIFT                 (0x0000001CU)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_RSVDZ_2_MAX                   (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_DR_MASK                       (0x40000000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_DR_SHIFT                      (0x0000001EU)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_DR_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_WPR_MASK                      (0x80000000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_WPR_SHIFT                     (0x0000001FU)
#define CSL_USB3P0SS_CTRL_XHCI_PORTSC1USB3_WPR_MAX                       (0x00000001U)

/* PORTPMSC1USB3 */

#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB3_U1_TIMEOUT_MASK             (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB3_U1_TIMEOUT_SHIFT            (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB3_U1_TIMEOUT_MAX              (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB3_U2_TIMEOUT_MASK             (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB3_U2_TIMEOUT_SHIFT            (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB3_U2_TIMEOUT_MAX              (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB3_FLA_MASK                    (0x00010000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB3_FLA_SHIFT                   (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB3_FLA_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB3_RSVDP_MASK                  (0xFFFE0000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB3_RSVDP_SHIFT                 (0x00000011U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTPMSC1USB3_RSVDP_MAX                   (0x00007FFFU)

/* PORTLI1 */

#define CSL_USB3P0SS_CTRL_XHCI_PORTLI1_LEC_MASK                          (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_PORTLI1_LEC_SHIFT                         (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTLI1_LEC_MAX                           (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_PORTLI1_RSVDP_MASK                        (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTLI1_RSVDP_SHIFT                       (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_PORTLI1_RSVDP_MAX                         (0x0000FFFFU)

/* RSVDP1USB3 */

#define CSL_USB3P0SS_CTRL_XHCI_RSVDP1USB3_RSVDP_1_MASK                   (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDP1USB3_RSVDP_1_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDP1USB3_RSVDP_1_MAX                    (0xFFFFFFFFU)

/* MFINDEX */

#define CSL_USB3P0SS_CTRL_XHCI_MFINDEX_MFINDEX_MASK                      (0x00003FFFU)
#define CSL_USB3P0SS_CTRL_XHCI_MFINDEX_MFINDEX_SHIFT                     (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_MFINDEX_MFINDEX_MAX                       (0x00003FFFU)

#define CSL_USB3P0SS_CTRL_XHCI_MFINDEX_RSVDZ1_MASK                       (0xFFFFC000U)
#define CSL_USB3P0SS_CTRL_XHCI_MFINDEX_RSVDZ1_SHIFT                      (0x0000000EU)
#define CSL_USB3P0SS_CTRL_XHCI_MFINDEX_RSVDZ1_MAX                        (0x0003FFFFU)

/* RSVDZ_RT */

#define CSL_USB3P0SS_CTRL_XHCI_RSVDZ_RT_RSVDZ_1_MASK                     (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDZ_RT_RSVDZ_1_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_RSVDZ_RT_RSVDZ_1_MAX                      (0xFFFFFFFFU)

/* IMAN0 */

#define CSL_USB3P0SS_CTRL_XHCI_IMAN0_IP_MASK                             (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN0_IP_SHIFT                            (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN0_IP_MAX                              (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_IMAN0_IE_MASK                             (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN0_IE_SHIFT                            (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN0_IE_MAX                              (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_IMAN0_RSVDP1_MASK                         (0xFFFFFFFCU)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN0_RSVDP1_SHIFT                        (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN0_RSVDP1_MAX                          (0x3FFFFFFFU)

/* IMOD0 */

#define CSL_USB3P0SS_CTRL_XHCI_IMOD0_IMODI_MASK                          (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD0_IMODI_SHIFT                         (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD0_IMODI_MAX                           (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_IMOD0_IMODC_MASK                          (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD0_IMODC_SHIFT                         (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD0_IMODC_MAX                           (0x0000FFFFU)

/* ERSTSZ0 */

#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ0_ERSTS_MASK                        (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ0_ERSTS_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ0_ERSTS_MAX                         (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ0_RSVDP1_MASK                       (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ0_RSVDP1_SHIFT                      (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ0_RSVDP1_MAX                        (0x0000FFFFU)

/* RESERVED0 */

#define CSL_USB3P0SS_CTRL_XHCI_RESERVED0_RSVDP1_MASK                     (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_RESERVED0_RSVDP1_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_RESERVED0_RSVDP1_MAX                      (0xFFFFFFFFU)

/* ERSTBA0_LO */

#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA0_LO_RSVDP1_MASK                    (0x0000003FU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA0_LO_RSVDP1_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA0_LO_RSVDP1_MAX                     (0x0000003FU)

#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA0_LO_ERSTBADDR_LO_MASK              (0xFFFFFFC0U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA0_LO_ERSTBADDR_LO_SHIFT             (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA0_LO_ERSTBADDR_LO_MAX               (0x03FFFFFFU)

/* ERSTBA0_HI */

#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA0_HI_ERSTBADDR_HI_MASK              (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA0_HI_ERSTBADDR_HI_SHIFT             (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA0_HI_ERSTBADDR_HI_MAX               (0xFFFFFFFFU)

/* ERDP0_LO */

#define CSL_USB3P0SS_CTRL_XHCI_ERDP0_LO_DESI_MASK                        (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP0_LO_DESI_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP0_LO_DESI_MAX                         (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_ERDP0_LO_EHB_MASK                         (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP0_LO_EHB_SHIFT                        (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP0_LO_EHB_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_ERDP0_LO_ERDPTR_MASK                      (0xFFFFFFF0U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP0_LO_ERDPTR_SHIFT                     (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP0_LO_ERDPTR_MAX                       (0x0FFFFFFFU)

/* ERDP0_HI */

#define CSL_USB3P0SS_CTRL_XHCI_ERDP0_HI_ERDPTR_HI_MASK                   (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP0_HI_ERDPTR_HI_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP0_HI_ERDPTR_HI_MAX                    (0xFFFFFFFFU)

/* IMAN1 */

#define CSL_USB3P0SS_CTRL_XHCI_IMAN1_IP_MASK                             (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN1_IP_SHIFT                            (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN1_IP_MAX                              (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_IMAN1_IE_MASK                             (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN1_IE_SHIFT                            (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN1_IE_MAX                              (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_IMAN1_RSVDP1_MASK                         (0xFFFFFFFCU)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN1_RSVDP1_SHIFT                        (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN1_RSVDP1_MAX                          (0x3FFFFFFFU)

/* IMOD1 */

#define CSL_USB3P0SS_CTRL_XHCI_IMOD1_IMODI_MASK                          (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD1_IMODI_SHIFT                         (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD1_IMODI_MAX                           (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_IMOD1_IMODC_MASK                          (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD1_IMODC_SHIFT                         (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD1_IMODC_MAX                           (0x0000FFFFU)

/* ERSTSZ1 */

#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ1_ERSTS_MASK                        (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ1_ERSTS_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ1_ERSTS_MAX                         (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ1_RSVDP1_MASK                       (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ1_RSVDP1_SHIFT                      (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ1_RSVDP1_MAX                        (0x0000FFFFU)

/* RESERVED1 */

#define CSL_USB3P0SS_CTRL_XHCI_RESERVED1_RSVDP1_MASK                     (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_RESERVED1_RSVDP1_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_RESERVED1_RSVDP1_MAX                      (0xFFFFFFFFU)

/* ERSTBA1_LO */

#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA1_LO_RSVDP1_MASK                    (0x0000003FU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA1_LO_RSVDP1_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA1_LO_RSVDP1_MAX                     (0x0000003FU)

#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA1_LO_ERSTBADDR_LO_MASK              (0xFFFFFFC0U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA1_LO_ERSTBADDR_LO_SHIFT             (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA1_LO_ERSTBADDR_LO_MAX               (0x03FFFFFFU)

/* ERSTBA1_HI */

#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA1_HI_ERSTBADDR_HI_MASK              (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA1_HI_ERSTBADDR_HI_SHIFT             (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA1_HI_ERSTBADDR_HI_MAX               (0xFFFFFFFFU)

/* ERDP1_LO */

#define CSL_USB3P0SS_CTRL_XHCI_ERDP1_LO_DESI_MASK                        (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP1_LO_DESI_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP1_LO_DESI_MAX                         (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_ERDP1_LO_EHB_MASK                         (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP1_LO_EHB_SHIFT                        (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP1_LO_EHB_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_ERDP1_LO_ERDPTR_MASK                      (0xFFFFFFF0U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP1_LO_ERDPTR_SHIFT                     (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP1_LO_ERDPTR_MAX                       (0x0FFFFFFFU)

/* ERDP1_HI */

#define CSL_USB3P0SS_CTRL_XHCI_ERDP1_HI_ERDPTR_HI_MASK                   (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP1_HI_ERDPTR_HI_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP1_HI_ERDPTR_HI_MAX                    (0xFFFFFFFFU)

/* IMAN2 */

#define CSL_USB3P0SS_CTRL_XHCI_IMAN2_IP_MASK                             (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN2_IP_SHIFT                            (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN2_IP_MAX                              (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_IMAN2_IE_MASK                             (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN2_IE_SHIFT                            (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN2_IE_MAX                              (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_IMAN2_RSVDP1_MASK                         (0xFFFFFFFCU)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN2_RSVDP1_SHIFT                        (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN2_RSVDP1_MAX                          (0x3FFFFFFFU)

/* IMOD2 */

#define CSL_USB3P0SS_CTRL_XHCI_IMOD2_IMODI_MASK                          (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD2_IMODI_SHIFT                         (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD2_IMODI_MAX                           (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_IMOD2_IMODC_MASK                          (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD2_IMODC_SHIFT                         (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD2_IMODC_MAX                           (0x0000FFFFU)

/* ERSTSZ2 */

#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ2_ERSTS_MASK                        (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ2_ERSTS_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ2_ERSTS_MAX                         (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ2_RSVDP1_MASK                       (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ2_RSVDP1_SHIFT                      (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ2_RSVDP1_MAX                        (0x0000FFFFU)

/* RESERVED2 */

#define CSL_USB3P0SS_CTRL_XHCI_RESERVED2_RSVDP1_MASK                     (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_RESERVED2_RSVDP1_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_RESERVED2_RSVDP1_MAX                      (0xFFFFFFFFU)

/* ERSTBA2_LO */

#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA2_LO_RSVDP1_MASK                    (0x0000003FU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA2_LO_RSVDP1_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA2_LO_RSVDP1_MAX                     (0x0000003FU)

#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA2_LO_ERSTBADDR_LO_MASK              (0xFFFFFFC0U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA2_LO_ERSTBADDR_LO_SHIFT             (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA2_LO_ERSTBADDR_LO_MAX               (0x03FFFFFFU)

/* ERSTBA2_HI */

#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA2_HI_ERSTBADDR_HI_MASK              (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA2_HI_ERSTBADDR_HI_SHIFT             (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA2_HI_ERSTBADDR_HI_MAX               (0xFFFFFFFFU)

/* ERDP2_LO */

#define CSL_USB3P0SS_CTRL_XHCI_ERDP2_LO_DESI_MASK                        (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP2_LO_DESI_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP2_LO_DESI_MAX                         (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_ERDP2_LO_EHB_MASK                         (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP2_LO_EHB_SHIFT                        (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP2_LO_EHB_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_ERDP2_LO_ERDPTR_MASK                      (0xFFFFFFF0U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP2_LO_ERDPTR_SHIFT                     (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP2_LO_ERDPTR_MAX                       (0x0FFFFFFFU)

/* ERDP2_HI */

#define CSL_USB3P0SS_CTRL_XHCI_ERDP2_HI_ERDPTR_HI_MASK                   (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP2_HI_ERDPTR_HI_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP2_HI_ERDPTR_HI_MAX                    (0xFFFFFFFFU)

/* IMAN3 */

#define CSL_USB3P0SS_CTRL_XHCI_IMAN3_IP_MASK                             (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN3_IP_SHIFT                            (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN3_IP_MAX                              (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_IMAN3_IE_MASK                             (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN3_IE_SHIFT                            (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN3_IE_MAX                              (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_IMAN3_RSVDP1_MASK                         (0xFFFFFFFCU)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN3_RSVDP1_SHIFT                        (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN3_RSVDP1_MAX                          (0x3FFFFFFFU)

/* IMOD3 */

#define CSL_USB3P0SS_CTRL_XHCI_IMOD3_IMODI_MASK                          (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD3_IMODI_SHIFT                         (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD3_IMODI_MAX                           (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_IMOD3_IMODC_MASK                          (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD3_IMODC_SHIFT                         (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD3_IMODC_MAX                           (0x0000FFFFU)

/* ERSTSZ3 */

#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ3_ERSTS_MASK                        (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ3_ERSTS_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ3_ERSTS_MAX                         (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ3_RSVDP1_MASK                       (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ3_RSVDP1_SHIFT                      (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ3_RSVDP1_MAX                        (0x0000FFFFU)

/* RESERVED3 */

#define CSL_USB3P0SS_CTRL_XHCI_RESERVED3_RSVDP1_MASK                     (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_RESERVED3_RSVDP1_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_RESERVED3_RSVDP1_MAX                      (0xFFFFFFFFU)

/* ERSTBA3_LO */

#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA3_LO_RSVDP1_MASK                    (0x0000003FU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA3_LO_RSVDP1_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA3_LO_RSVDP1_MAX                     (0x0000003FU)

#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA3_LO_ERSTBADDR_LO_MASK              (0xFFFFFFC0U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA3_LO_ERSTBADDR_LO_SHIFT             (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA3_LO_ERSTBADDR_LO_MAX               (0x03FFFFFFU)

/* ERSTBA3_HI */

#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA3_HI_ERSTBADDR_HI_MASK              (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA3_HI_ERSTBADDR_HI_SHIFT             (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA3_HI_ERSTBADDR_HI_MAX               (0xFFFFFFFFU)

/* ERDP3_LO */

#define CSL_USB3P0SS_CTRL_XHCI_ERDP3_LO_DESI_MASK                        (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP3_LO_DESI_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP3_LO_DESI_MAX                         (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_ERDP3_LO_EHB_MASK                         (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP3_LO_EHB_SHIFT                        (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP3_LO_EHB_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_ERDP3_LO_ERDPTR_MASK                      (0xFFFFFFF0U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP3_LO_ERDPTR_SHIFT                     (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP3_LO_ERDPTR_MAX                       (0x0FFFFFFFU)

/* ERDP3_HI */

#define CSL_USB3P0SS_CTRL_XHCI_ERDP3_HI_ERDPTR_HI_MASK                   (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP3_HI_ERDPTR_HI_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP3_HI_ERDPTR_HI_MAX                    (0xFFFFFFFFU)

/* IMAN4 */

#define CSL_USB3P0SS_CTRL_XHCI_IMAN4_IP_MASK                             (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN4_IP_SHIFT                            (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN4_IP_MAX                              (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_IMAN4_IE_MASK                             (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN4_IE_SHIFT                            (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN4_IE_MAX                              (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_IMAN4_RSVDP1_MASK                         (0xFFFFFFFCU)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN4_RSVDP1_SHIFT                        (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN4_RSVDP1_MAX                          (0x3FFFFFFFU)

/* IMOD4 */

#define CSL_USB3P0SS_CTRL_XHCI_IMOD4_IMODI_MASK                          (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD4_IMODI_SHIFT                         (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD4_IMODI_MAX                           (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_IMOD4_IMODC_MASK                          (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD4_IMODC_SHIFT                         (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD4_IMODC_MAX                           (0x0000FFFFU)

/* ERSTSZ4 */

#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ4_ERSTS_MASK                        (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ4_ERSTS_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ4_ERSTS_MAX                         (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ4_RSVDP1_MASK                       (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ4_RSVDP1_SHIFT                      (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ4_RSVDP1_MAX                        (0x0000FFFFU)

/* RESERVED4 */

#define CSL_USB3P0SS_CTRL_XHCI_RESERVED4_RSVDP1_MASK                     (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_RESERVED4_RSVDP1_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_RESERVED4_RSVDP1_MAX                      (0xFFFFFFFFU)

/* ERSTBA4_LO */

#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA4_LO_RSVDP1_MASK                    (0x0000003FU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA4_LO_RSVDP1_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA4_LO_RSVDP1_MAX                     (0x0000003FU)

#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA4_LO_ERSTBADDR_LO_MASK              (0xFFFFFFC0U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA4_LO_ERSTBADDR_LO_SHIFT             (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA4_LO_ERSTBADDR_LO_MAX               (0x03FFFFFFU)

/* ERSTBA4_HI */

#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA4_HI_ERSTBADDR_HI_MASK              (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA4_HI_ERSTBADDR_HI_SHIFT             (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA4_HI_ERSTBADDR_HI_MAX               (0xFFFFFFFFU)

/* ERDP4_LO */

#define CSL_USB3P0SS_CTRL_XHCI_ERDP4_LO_DESI_MASK                        (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP4_LO_DESI_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP4_LO_DESI_MAX                         (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_ERDP4_LO_EHB_MASK                         (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP4_LO_EHB_SHIFT                        (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP4_LO_EHB_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_ERDP4_LO_ERDPTR_MASK                      (0xFFFFFFF0U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP4_LO_ERDPTR_SHIFT                     (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP4_LO_ERDPTR_MAX                       (0x0FFFFFFFU)

/* ERDP4_HI */

#define CSL_USB3P0SS_CTRL_XHCI_ERDP4_HI_ERDPTR_HI_MASK                   (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP4_HI_ERDPTR_HI_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP4_HI_ERDPTR_HI_MAX                    (0xFFFFFFFFU)

/* IMAN5 */

#define CSL_USB3P0SS_CTRL_XHCI_IMAN5_IP_MASK                             (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN5_IP_SHIFT                            (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN5_IP_MAX                              (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_IMAN5_IE_MASK                             (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN5_IE_SHIFT                            (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN5_IE_MAX                              (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_IMAN5_RSVDP1_MASK                         (0xFFFFFFFCU)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN5_RSVDP1_SHIFT                        (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN5_RSVDP1_MAX                          (0x3FFFFFFFU)

/* IMOD5 */

#define CSL_USB3P0SS_CTRL_XHCI_IMOD5_IMODI_MASK                          (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD5_IMODI_SHIFT                         (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD5_IMODI_MAX                           (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_IMOD5_IMODC_MASK                          (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD5_IMODC_SHIFT                         (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD5_IMODC_MAX                           (0x0000FFFFU)

/* ERSTSZ5 */

#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ5_ERSTS_MASK                        (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ5_ERSTS_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ5_ERSTS_MAX                         (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ5_RSVDP1_MASK                       (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ5_RSVDP1_SHIFT                      (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ5_RSVDP1_MAX                        (0x0000FFFFU)

/* RESERVED5 */

#define CSL_USB3P0SS_CTRL_XHCI_RESERVED5_RSVDP1_MASK                     (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_RESERVED5_RSVDP1_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_RESERVED5_RSVDP1_MAX                      (0xFFFFFFFFU)

/* ERSTBA5_LO */

#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA5_LO_RSVDP1_MASK                    (0x0000003FU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA5_LO_RSVDP1_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA5_LO_RSVDP1_MAX                     (0x0000003FU)

#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA5_LO_ERSTBADDR_LO_MASK              (0xFFFFFFC0U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA5_LO_ERSTBADDR_LO_SHIFT             (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA5_LO_ERSTBADDR_LO_MAX               (0x03FFFFFFU)

/* ERSTBA5_HI */

#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA5_HI_ERSTBADDR_HI_MASK              (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA5_HI_ERSTBADDR_HI_SHIFT             (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA5_HI_ERSTBADDR_HI_MAX               (0xFFFFFFFFU)

/* ERDP5_LO */

#define CSL_USB3P0SS_CTRL_XHCI_ERDP5_LO_DESI_MASK                        (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP5_LO_DESI_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP5_LO_DESI_MAX                         (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_ERDP5_LO_EHB_MASK                         (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP5_LO_EHB_SHIFT                        (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP5_LO_EHB_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_ERDP5_LO_ERDPTR_MASK                      (0xFFFFFFF0U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP5_LO_ERDPTR_SHIFT                     (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP5_LO_ERDPTR_MAX                       (0x0FFFFFFFU)

/* ERDP5_HI */

#define CSL_USB3P0SS_CTRL_XHCI_ERDP5_HI_ERDPTR_HI_MASK                   (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP5_HI_ERDPTR_HI_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP5_HI_ERDPTR_HI_MAX                    (0xFFFFFFFFU)

/* IMAN6 */

#define CSL_USB3P0SS_CTRL_XHCI_IMAN6_IP_MASK                             (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN6_IP_SHIFT                            (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN6_IP_MAX                              (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_IMAN6_IE_MASK                             (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN6_IE_SHIFT                            (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN6_IE_MAX                              (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_IMAN6_RSVDP1_MASK                         (0xFFFFFFFCU)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN6_RSVDP1_SHIFT                        (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN6_RSVDP1_MAX                          (0x3FFFFFFFU)

/* IMOD6 */

#define CSL_USB3P0SS_CTRL_XHCI_IMOD6_IMODI_MASK                          (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD6_IMODI_SHIFT                         (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD6_IMODI_MAX                           (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_IMOD6_IMODC_MASK                          (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD6_IMODC_SHIFT                         (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD6_IMODC_MAX                           (0x0000FFFFU)

/* ERSTSZ6 */

#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ6_ERSTS_MASK                        (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ6_ERSTS_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ6_ERSTS_MAX                         (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ6_RSVDP1_MASK                       (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ6_RSVDP1_SHIFT                      (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ6_RSVDP1_MAX                        (0x0000FFFFU)

/* RESERVED6 */

#define CSL_USB3P0SS_CTRL_XHCI_RESERVED6_RSVDP1_MASK                     (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_RESERVED6_RSVDP1_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_RESERVED6_RSVDP1_MAX                      (0xFFFFFFFFU)

/* ERSTBA6_LO */

#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA6_LO_RSVDP1_MASK                    (0x0000003FU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA6_LO_RSVDP1_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA6_LO_RSVDP1_MAX                     (0x0000003FU)

#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA6_LO_ERSTBADDR_LO_MASK              (0xFFFFFFC0U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA6_LO_ERSTBADDR_LO_SHIFT             (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA6_LO_ERSTBADDR_LO_MAX               (0x03FFFFFFU)

/* ERSTBA6_HI */

#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA6_HI_ERSTBADDR_HI_MASK              (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA6_HI_ERSTBADDR_HI_SHIFT             (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA6_HI_ERSTBADDR_HI_MAX               (0xFFFFFFFFU)

/* ERDP6_LO */

#define CSL_USB3P0SS_CTRL_XHCI_ERDP6_LO_DESI_MASK                        (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP6_LO_DESI_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP6_LO_DESI_MAX                         (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_ERDP6_LO_EHB_MASK                         (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP6_LO_EHB_SHIFT                        (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP6_LO_EHB_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_ERDP6_LO_ERDPTR_MASK                      (0xFFFFFFF0U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP6_LO_ERDPTR_SHIFT                     (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP6_LO_ERDPTR_MAX                       (0x0FFFFFFFU)

/* ERDP6_HI */

#define CSL_USB3P0SS_CTRL_XHCI_ERDP6_HI_ERDPTR_HI_MASK                   (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP6_HI_ERDPTR_HI_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP6_HI_ERDPTR_HI_MAX                    (0xFFFFFFFFU)

/* IMAN7 */

#define CSL_USB3P0SS_CTRL_XHCI_IMAN7_IP_MASK                             (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN7_IP_SHIFT                            (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN7_IP_MAX                              (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_IMAN7_IE_MASK                             (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN7_IE_SHIFT                            (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN7_IE_MAX                              (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_IMAN7_RSVDP1_MASK                         (0xFFFFFFFCU)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN7_RSVDP1_SHIFT                        (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_IMAN7_RSVDP1_MAX                          (0x3FFFFFFFU)

/* IMOD7 */

#define CSL_USB3P0SS_CTRL_XHCI_IMOD7_IMODI_MASK                          (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD7_IMODI_SHIFT                         (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD7_IMODI_MAX                           (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_IMOD7_IMODC_MASK                          (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD7_IMODC_SHIFT                         (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_IMOD7_IMODC_MAX                           (0x0000FFFFU)

/* ERSTSZ7 */

#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ7_ERSTS_MASK                        (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ7_ERSTS_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ7_ERSTS_MAX                         (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ7_RSVDP1_MASK                       (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ7_RSVDP1_SHIFT                      (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTSZ7_RSVDP1_MAX                        (0x0000FFFFU)

/* RESERVED7 */

#define CSL_USB3P0SS_CTRL_XHCI_RESERVED7_RSVDP1_MASK                     (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_RESERVED7_RSVDP1_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_RESERVED7_RSVDP1_MAX                      (0xFFFFFFFFU)

/* ERSTBA7_LO */

#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA7_LO_RSVDP1_MASK                    (0x0000003FU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA7_LO_RSVDP1_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA7_LO_RSVDP1_MAX                     (0x0000003FU)

#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA7_LO_ERSTBADDR_LO_MASK              (0xFFFFFFC0U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA7_LO_ERSTBADDR_LO_SHIFT             (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA7_LO_ERSTBADDR_LO_MAX               (0x03FFFFFFU)

/* ERSTBA7_HI */

#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA7_HI_ERSTBADDR_HI_MASK              (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA7_HI_ERSTBADDR_HI_SHIFT             (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERSTBA7_HI_ERSTBADDR_HI_MAX               (0xFFFFFFFFU)

/* ERDP7_LO */

#define CSL_USB3P0SS_CTRL_XHCI_ERDP7_LO_DESI_MASK                        (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP7_LO_DESI_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP7_LO_DESI_MAX                         (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_ERDP7_LO_EHB_MASK                         (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP7_LO_EHB_SHIFT                        (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP7_LO_EHB_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_ERDP7_LO_ERDPTR_MASK                      (0xFFFFFFF0U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP7_LO_ERDPTR_SHIFT                     (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP7_LO_ERDPTR_MAX                       (0x0FFFFFFFU)

/* ERDP7_HI */

#define CSL_USB3P0SS_CTRL_XHCI_ERDP7_HI_ERDPTR_HI_MASK                   (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP7_HI_ERDPTR_HI_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_ERDP7_HI_ERDPTR_HI_MAX                    (0xFFFFFFFFU)

/* DB0 */

#define CSL_USB3P0SS_CTRL_XHCI_DB0_DB_TARGET_MASK                        (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_DB0_DB_TARGET_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_DB0_DB_TARGET_MAX                         (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_DB0_RSVDZ1_MASK                           (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_XHCI_DB0_RSVDZ1_SHIFT                          (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_DB0_RSVDZ1_MAX                            (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_DB0_DB_STREAM_ID_MASK                     (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_DB0_DB_STREAM_ID_SHIFT                    (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_DB0_DB_STREAM_ID_MAX                      (0x0000FFFFU)

/* DB */

#define CSL_USB3P0SS_CTRL_XHCI_DB_DB_TARGET_MASK                         (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_DB_DB_TARGET_SHIFT                        (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_DB_DB_TARGET_MAX                          (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_DB_RSVDZ1_MASK                            (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_XHCI_DB_RSVDZ1_SHIFT                           (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_DB_RSVDZ1_MAX                             (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_DB_DB_STREAM_ID_MASK                      (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_DB_DB_STREAM_ID_SHIFT                     (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_DB_DB_STREAM_ID_MAX                       (0x0000FFFFU)

/* XECP_PORT_CAP_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_CAP_REG_XHCI_PORT_CAP_ID_MASK   (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_CAP_REG_XHCI_PORT_CAP_ID_SHIFT  (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_CAP_REG_XHCI_PORT_CAP_ID_MAX    (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_CAP_REG_XHCI_PORT_CAPABILITY_DW_MASK (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_CAP_REG_XHCI_PORT_CAPABILITY_DW_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_CAP_REG_XHCI_PORT_CAPABILITY_DW_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_CAP_REG_XHCI_PORT_CAP_REV_MASK  (0x00FF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_CAP_REG_XHCI_PORT_CAP_REV_SHIFT (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_CAP_REG_XHCI_PORT_CAP_REV_MAX   (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_CAP_REG_LPM_2_STB_SWITCH_CAPABLE_MASK (0x01000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_CAP_REG_LPM_2_STB_SWITCH_CAPABLE_SHIFT (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_CAP_REG_LPM_2_STB_SWITCH_CAPABLE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_CAP_REG_LPM_2_STB_SWITCH_EN_MASK (0x02000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_CAP_REG_LPM_2_STB_SWITCH_EN_SHIFT (0x00000019U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_CAP_REG_LPM_2_STB_SWITCH_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_CAP_REG_RESERVED1_MASK          (0xFC000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_CAP_REG_RESERVED1_SHIFT         (0x0000001AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_CAP_REG_RESERVED1_MAX           (0x0000003FU)

/* XECP_PORT_1_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_TRAINING_FAIL_MASK        (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_TRAINING_FAIL_SHIFT       (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_TRAINING_FAIL_MAX         (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_TERM_DEB_MAX_MASK         (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_TERM_DEB_MAX_SHIFT        (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_TERM_DEB_MAX_MAX          (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_U3_SPUR_LFPS_FIX_MASK     (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_U3_SPUR_LFPS_FIX_SHIFT    (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_U3_SPUR_LFPS_FIX_MAX      (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_SKP_OS_FIX_MASK           (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_SKP_OS_FIX_SHIFT          (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_SKP_OS_FIX_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_TTIME_FOR_RESET_EN_MASK   (0x00000020U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_TTIME_FOR_RESET_EN_SHIFT  (0x00000005U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_TTIME_FOR_RESET_EN_MAX    (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_CPOLLINGTIMEOUT_EN_MASK   (0x00000040U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_CPOLLINGTIMEOUT_EN_SHIFT  (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_CPOLLINGTIMEOUT_EN_MAX    (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_RESERVED5_MASK            (0x00000080U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_RESERVED5_SHIFT           (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_RESERVED5_MAX             (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_U1_LFPS_MINGEN_TIME_MASK  (0x00007F00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_U1_LFPS_MINGEN_TIME_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_U1_LFPS_MINGEN_TIME_MAX   (0x0000007FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_U1_LFPS_TIME_WR_STROBE_MASK (0x00008000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_U1_LFPS_TIME_WR_STROBE_SHIFT (0x0000000FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_U1_LFPS_TIME_WR_STROBE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_RESERVED_MASK             (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_RESERVED_SHIFT            (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PORT_1_REG_RESERVED_MAX              (0x0000FFFFU)

/* XECP_CDNS_DEBUG_BUS_CAP */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CDNS_DEBUG_BUS_CAP_XHCI_DEBUG_BUS_CAP_ID_MASK (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CDNS_DEBUG_BUS_CAP_XHCI_DEBUG_BUS_CAP_ID_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CDNS_DEBUG_BUS_CAP_XHCI_DEBUG_BUS_CAP_ID_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CDNS_DEBUG_BUS_CAP_XHCI_DEBUG_BUS_DW_MASK (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CDNS_DEBUG_BUS_CAP_XHCI_DEBUG_BUS_DW_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CDNS_DEBUG_BUS_CAP_XHCI_DEBUG_BUS_DW_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CDNS_DEBUG_BUS_CAP_RSVDP1_MASK       (0x7FFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CDNS_DEBUG_BUS_CAP_RSVDP1_SHIFT      (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CDNS_DEBUG_BUS_CAP_RSVDP1_MAX        (0x00007FFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CDNS_DEBUG_BUS_CAP_CPU_DEBUG_EN_MASK (0x80000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CDNS_DEBUG_BUS_CAP_CPU_DEBUG_EN_SHIFT (0x0000001FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CDNS_DEBUG_BUS_CAP_CPU_DEBUG_EN_MAX  (0x00000001U)

/* XECP_CDNS_DEBUG_BUS_CTRL */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CDNS_DEBUG_BUS_CTRL_CPU_DEBUG_BUS_SEL_MASK (0x0000001FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CDNS_DEBUG_BUS_CTRL_CPU_DEBUG_BUS_SEL_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CDNS_DEBUG_BUS_CTRL_CPU_DEBUG_BUS_SEL_MAX (0x0000001FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CDNS_DEBUG_BUS_CTRL_RSVDP1_MASK      (0xFFFFFFE0U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CDNS_DEBUG_BUS_CTRL_RSVDP1_SHIFT     (0x00000005U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CDNS_DEBUG_BUS_CTRL_RSVDP1_MAX       (0x07FFFFFFU)

/* XECP_CDNS_DEBUG_BUS_STATUS */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CDNS_DEBUG_BUS_STATUS_XHCI_DEBUG_BUS_MASK (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CDNS_DEBUG_BUS_STATUS_XHCI_DEBUG_BUS_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CDNS_DEBUG_BUS_STATUS_XHCI_DEBUG_BUS_MAX (0xFFFFFFFFU)

/* XECP_PM_CAP */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_XHCI_PM_CAP_ID_MASK           (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_XHCI_PM_CAP_ID_SHIFT          (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_XHCI_PM_CAP_ID_MAX            (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_XHCI_PM_CAPABILITY_DW_MASK    (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_XHCI_PM_CAPABILITY_DW_SHIFT   (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_XHCI_PM_CAPABILITY_DW_MAX     (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_VERSION_MASK                  (0x00070000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_VERSION_SHIFT                 (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_VERSION_MAX                   (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_PME_CLOCK_MASK                (0x00080000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_PME_CLOCK_SHIFT               (0x00000013U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_PME_CLOCK_MAX                 (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_RESERVED_MASK                 (0x00100000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_RESERVED_SHIFT                (0x00000014U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_RESERVED_MAX                  (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_DSI_MASK                      (0x00200000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_DSI_SHIFT                     (0x00000015U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_DSI_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_AUX_CURRENT_MASK              (0x01C00000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_AUX_CURRENT_SHIFT             (0x00000016U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_AUX_CURRENT_MAX               (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_D1_SUPPORT_MASK               (0x02000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_D1_SUPPORT_SHIFT              (0x00000019U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_D1_SUPPORT_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_D2_SUPPORT_MASK               (0x04000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_D2_SUPPORT_SHIFT              (0x0000001AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_D2_SUPPORT_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_PME_SUPPORT_MASK              (0xF8000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_PME_SUPPORT_SHIFT             (0x0000001BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_CAP_PME_SUPPORT_MAX               (0x0000001FU)

/* XECP_PM_PMCSR */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_POWERSTATE_MASK             (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_POWERSTATE_SHIFT            (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_POWERSTATE_MAX              (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_RESERVED1_MASK              (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_RESERVED1_SHIFT             (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_RESERVED1_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_NO_SOFT_RESET_MASK          (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_NO_SOFT_RESET_SHIFT         (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_NO_SOFT_RESET_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_RESERVED2_MASK              (0x000000F0U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_RESERVED2_SHIFT             (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_RESERVED2_MAX               (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_PME_EN_MASK                 (0x00000100U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_PME_EN_SHIFT                (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_PME_EN_MAX                  (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_DATA_SELECT_MASK            (0x00001E00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_DATA_SELECT_SHIFT           (0x00000009U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_DATA_SELECT_MAX             (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_DATA_SCALE_MASK             (0x00006000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_DATA_SCALE_SHIFT            (0x0000000DU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_DATA_SCALE_MAX              (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_PME_STATUS_MASK             (0x00008000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_PME_STATUS_SHIFT            (0x0000000FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_PME_STATUS_MAX              (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_RESERVED3_MASK              (0x003F0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_RESERVED3_SHIFT             (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_RESERVED3_MAX               (0x0000003FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_B2_B3_MASK                  (0x00400000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_B2_B3_SHIFT                 (0x00000016U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_B2_B3_MAX                   (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_BPCC_EN_MASK                (0x00800000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_BPCC_EN_SHIFT               (0x00000017U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_BPCC_EN_MAX                 (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_DATA_REGISTER_MASK          (0xFF000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_DATA_REGISTER_SHIFT         (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PM_PMCSR_DATA_REGISTER_MAX           (0x000000FFU)

/* XECP_MSI_CAP */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_CAP_MSI_ID_MASK                  (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_CAP_MSI_ID_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_CAP_MSI_ID_MAX                   (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_CAP_XECP_MSI_CAP_OFFSET_MASK     (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_CAP_XECP_MSI_CAP_OFFSET_SHIFT    (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_CAP_XECP_MSI_CAP_OFFSET_MAX      (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_CAP_MSI_EN_MASK                  (0x00010000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_CAP_MSI_EN_SHIFT                 (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_CAP_MSI_EN_MAX                   (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_CAP_MSI_MMC_MASK                 (0x000E0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_CAP_MSI_MMC_SHIFT                (0x00000011U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_CAP_MSI_MMC_MAX                  (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_CAP_MSI_MME_MASK                 (0x00700000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_CAP_MSI_MME_SHIFT                (0x00000014U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_CAP_MSI_MME_MAX                  (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_CAP_AC64_MASK                    (0x00800000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_CAP_AC64_SHIFT                   (0x00000017U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_CAP_AC64_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_CAP_PER_VECTOR_MASKING_MASK      (0x01000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_CAP_PER_VECTOR_MASKING_SHIFT     (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_CAP_PER_VECTOR_MASKING_MAX       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_CAP_RESERVED_MASK                (0xFE000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_CAP_RESERVED_SHIFT               (0x00000019U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_CAP_RESERVED_MAX                 (0x0000007FU)

/* XECP_MSI_ADDR_L */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_ADDR_L_RESERVED_MASK             (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_ADDR_L_RESERVED_SHIFT            (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_ADDR_L_RESERVED_MAX              (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_ADDR_L_MSI_ADDR_LOW_MASK         (0xFFFFFFFCU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_ADDR_L_MSI_ADDR_LOW_SHIFT        (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_ADDR_L_MSI_ADDR_LOW_MAX          (0x3FFFFFFFU)

/* XECP_MSI_ADDR_H */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_ADDR_H_MSI_ADDR_HI_MASK          (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_ADDR_H_MSI_ADDR_HI_SHIFT         (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_ADDR_H_MSI_ADDR_HI_MAX           (0xFFFFFFFFU)

/* XECP_MSI_DATA */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_DATA_MSI_DATA_MASK               (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_DATA_MSI_DATA_SHIFT              (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_DATA_MSI_DATA_MAX                (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_DATA_RSVD_MASK                   (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_DATA_RSVD_SHIFT                  (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_MSI_DATA_RSVD_MAX                    (0x0000FFFFU)

/* XECP_AXI_CAP */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_AXI_CAP_ID_MASK              (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_AXI_CAP_ID_SHIFT             (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_AXI_CAP_ID_MAX               (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_XECP_AXI_CAP_OFFSET_MASK     (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_XECP_AXI_CAP_OFFSET_SHIFT    (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_XECP_AXI_CAP_OFFSET_MAX      (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_AXI_ADDRESS_WIDTH_64_MASK    (0x00010000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_AXI_ADDRESS_WIDTH_64_SHIFT   (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_AXI_ADDRESS_WIDTH_64_MAX     (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_RSVDP1_MASK                  (0x003E0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_RSVDP1_SHIFT                 (0x00000011U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_RSVDP1_MAX                   (0x0000001FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_AXI_MASTER_WRAPPER_SPLIT_BYTE_BURSTS_MASK (0x00400000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_AXI_MASTER_WRAPPER_SPLIT_BYTE_BURSTS_SHIFT (0x00000016U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_AXI_MASTER_WRAPPER_SPLIT_BYTE_BURSTS_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_AXI_DISABLE_OOO_MASK         (0x00800000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_AXI_DISABLE_OOO_SHIFT        (0x00000017U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_AXI_DISABLE_OOO_MAX          (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_AXI_DATA_BUS_SIZE_MASK       (0x07000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_AXI_DATA_BUS_SIZE_SHIFT      (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_AXI_DATA_BUS_SIZE_MAX        (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_RSVDP2_MASK                  (0x18000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_RSVDP2_SHIFT                 (0x0000001BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_RSVDP2_MAX                   (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_AXI_ERROR_MASK               (0x20000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_AXI_ERROR_SHIFT              (0x0000001DU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_AXI_ERROR_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_AXI_IDLE_MASK                (0x40000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_AXI_IDLE_SHIFT               (0x0000001EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_AXI_IDLE_MAX                 (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_AXI_HALT_MASK                (0x80000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_AXI_HALT_SHIFT               (0x0000001FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CAP_AXI_HALT_MAX                 (0x00000001U)

/* XECP_AXI_CFG0 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CFG0_AXI_MAX_WR_OT_MASK          (0x0000003FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CFG0_AXI_MAX_WR_OT_SHIFT         (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CFG0_AXI_MAX_WR_OT_MAX           (0x0000003FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CFG0_RSVDZ1_MASK                 (0x000000C0U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CFG0_RSVDZ1_SHIFT                (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CFG0_RSVDZ1_MAX                  (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CFG0_AXI_WR_DEPTH_MASK           (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CFG0_AXI_WR_DEPTH_SHIFT          (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CFG0_AXI_WR_DEPTH_MAX            (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CFG0_AXI_MAX_RD_OT_MASK          (0x003F0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CFG0_AXI_MAX_RD_OT_SHIFT         (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CFG0_AXI_MAX_RD_OT_MAX           (0x0000003FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CFG0_RSVDZ2_MASK                 (0x00C00000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CFG0_RSVDZ2_SHIFT                (0x00000016U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CFG0_RSVDZ2_MAX                  (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CFG0_AXI_RD_DEPTH_MASK           (0xFF000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CFG0_AXI_RD_DEPTH_SHIFT          (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CFG0_AXI_RD_DEPTH_MAX            (0x000000FFU)

/* XECP_AXI_CTRL0 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CTRL0_AXI_BMAX_MASK              (0x0000000FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CTRL0_AXI_BMAX_SHIFT             (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CTRL0_AXI_BMAX_MAX               (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CTRL0_RSVDP_MASK                 (0xFFFFFFF0U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CTRL0_RSVDP_SHIFT                (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CTRL0_RSVDP_MAX                  (0x0FFFFFFFU)

/* XECP_AXI_CTRL1 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CTRL1_AXI_WOT_MASK               (0x0000003FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CTRL1_AXI_WOT_SHIFT              (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CTRL1_AXI_WOT_MAX                (0x0000003FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CTRL1_RSVDP1_MASK                (0x0000FFC0U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CTRL1_RSVDP1_SHIFT               (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CTRL1_RSVDP1_MAX                 (0x000003FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CTRL1_AXI_ROT_MASK               (0x003F0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CTRL1_AXI_ROT_SHIFT              (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CTRL1_AXI_ROT_MAX                (0x0000003FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CTRL1_RSVDP2_MASK                (0xFFC00000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CTRL1_RSVDP2_SHIFT               (0x00000016U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CTRL1_RSVDP2_MAX                 (0x000003FFU)

/* XECP_AXI_CTRL2 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CTRL2_AXI_WTHRES_MASK            (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CTRL2_AXI_WTHRES_SHIFT           (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CTRL2_AXI_WTHRES_MAX             (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CTRL2_RSVDP_MASK                 (0xFFFFFF00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CTRL2_RSVDP_SHIFT                (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AXI_CTRL2_RSVDP_MAX                  (0x00FFFFFFU)

/* XECP_SUPP_USB2_CAP0 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP0_PID_MASK              (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP0_PID_SHIFT             (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP0_PID_MAX               (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP0_NEXTCAPID_MASK        (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP0_NEXTCAPID_SHIFT       (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP0_NEXTCAPID_MAX         (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP0_MINOR_REV_MASK        (0x00FF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP0_MINOR_REV_SHIFT       (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP0_MINOR_REV_MAX         (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP0_MAJOR_REV_MASK        (0xFF000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP0_MAJOR_REV_SHIFT       (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP0_MAJOR_REV_MAX         (0x000000FFU)

/* XECP_SUPP_USB2_CAP1 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP1_USB_STRING_MASK       (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP1_USB_STRING_SHIFT      (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP1_USB_STRING_MAX        (0xFFFFFFFFU)

/* XECP_SUPP_USB2_CAP2 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2_COMPATIBLE_PORT_OFFSET_MASK (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2_COMPATIBLE_PORT_OFFSET_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2_COMPATIBLE_PORT_OFFSET_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2_COMPATIBLE_PORT_COUNT_MASK (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2_COMPATIBLE_PORT_COUNT_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2_COMPATIBLE_PORT_COUNT_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2_L1C_MASK              (0x00010000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2_L1C_SHIFT             (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2_L1C_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2_HSO_MASK              (0x00020000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2_HSO_SHIFT             (0x00000011U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2_HSO_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2_IHI_MASK              (0x00040000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2_IHI_SHIFT             (0x00000012U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2_IHI_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2_HLC_MASK              (0x00080000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2_HLC_SHIFT             (0x00000013U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2_HLC_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2_HLC_BESL_MASK         (0x00100000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2_HLC_BESL_SHIFT        (0x00000014U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2_HLC_BESL_MAX          (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2_RSVDP_2_MASK          (0x0FE00000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2_RSVDP_2_SHIFT         (0x00000015U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2_RSVDP_2_MAX           (0x0000007FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2_PSIC_MASK             (0xF0000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2_PSIC_SHIFT            (0x0000001CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_CAP2_PSIC_MAX              (0x0000000FU)

/* XECP_SUPP_USB2_PROTOCOL_SLOT_TYPE */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_PROTOCOL_SLOT_TYPE_PST_MASK (0x0000001FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_PROTOCOL_SLOT_TYPE_PST_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_PROTOCOL_SLOT_TYPE_PST_MAX (0x0000001FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_PROTOCOL_SLOT_TYPE_RSVDP1_MASK (0xFFFFFFE0U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_PROTOCOL_SLOT_TYPE_RSVDP1_SHIFT (0x00000005U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB2_PROTOCOL_SLOT_TYPE_RSVDP1_MAX (0x07FFFFFFU)

/* XECP_PSI_FULL_SPEED */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_FULL_SPEED_PSIV_MASK             (0x0000000FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_FULL_SPEED_PSIV_SHIFT            (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_FULL_SPEED_PSIV_MAX              (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_FULL_SPEED_PSIE_MASK             (0x00000030U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_FULL_SPEED_PSIE_SHIFT            (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_FULL_SPEED_PSIE_MAX              (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_FULL_SPEED_PLT_MASK              (0x000000C0U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_FULL_SPEED_PLT_SHIFT             (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_FULL_SPEED_PLT_MAX               (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_FULL_SPEED_PFD_MASK              (0x00000100U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_FULL_SPEED_PFD_SHIFT             (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_FULL_SPEED_PFD_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_FULL_SPEED_RSVDP1_MASK           (0x0000FE00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_FULL_SPEED_RSVDP1_SHIFT          (0x00000009U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_FULL_SPEED_RSVDP1_MAX            (0x0000007FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_FULL_SPEED_PSIM_MASK             (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_FULL_SPEED_PSIM_SHIFT            (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_FULL_SPEED_PSIM_MAX              (0x0000FFFFU)

/* XECP_PSI_LOW_SPEED */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_LOW_SPEED_PSIV_MASK              (0x0000000FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_LOW_SPEED_PSIV_SHIFT             (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_LOW_SPEED_PSIV_MAX               (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_LOW_SPEED_PSIE_MASK              (0x00000030U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_LOW_SPEED_PSIE_SHIFT             (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_LOW_SPEED_PSIE_MAX               (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_LOW_SPEED_PLT_MASK               (0x000000C0U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_LOW_SPEED_PLT_SHIFT              (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_LOW_SPEED_PLT_MAX                (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_LOW_SPEED_PFD_MASK               (0x00000100U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_LOW_SPEED_PFD_SHIFT              (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_LOW_SPEED_PFD_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_LOW_SPEED_RSVDP1_MASK            (0x0000FE00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_LOW_SPEED_RSVDP1_SHIFT           (0x00000009U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_LOW_SPEED_RSVDP1_MAX             (0x0000007FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_LOW_SPEED_PSIM_MASK              (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_LOW_SPEED_PSIM_SHIFT             (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_LOW_SPEED_PSIM_MAX               (0x0000FFFFU)

/* XECP_PSI_HIGH_SPEED */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_HIGH_SPEED_PSIV_MASK             (0x0000000FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_HIGH_SPEED_PSIV_SHIFT            (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_HIGH_SPEED_PSIV_MAX              (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_HIGH_SPEED_PSIE_MASK             (0x00000030U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_HIGH_SPEED_PSIE_SHIFT            (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_HIGH_SPEED_PSIE_MAX              (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_HIGH_SPEED_PLT_MASK              (0x000000C0U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_HIGH_SPEED_PLT_SHIFT             (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_HIGH_SPEED_PLT_MAX               (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_HIGH_SPEED_PFD_MASK              (0x00000100U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_HIGH_SPEED_PFD_SHIFT             (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_HIGH_SPEED_PFD_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_HIGH_SPEED_RSVDP1_MASK           (0x0000FE00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_HIGH_SPEED_RSVDP1_SHIFT          (0x00000009U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_HIGH_SPEED_RSVDP1_MAX            (0x0000007FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_HIGH_SPEED_PSIM_MASK             (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_HIGH_SPEED_PSIM_SHIFT            (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_PSI_HIGH_SPEED_PSIM_MAX              (0x0000FFFFU)

/* XECP_SUPP_USB3_CAP0 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP0_PID_MASK              (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP0_PID_SHIFT             (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP0_PID_MAX               (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP0_NEXTCAPID_MASK        (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP0_NEXTCAPID_SHIFT       (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP0_NEXTCAPID_MAX         (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP0_MINOR_REV_MASK        (0x00FF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP0_MINOR_REV_SHIFT       (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP0_MINOR_REV_MAX         (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP0_MAJOR_REV_MASK        (0xFF000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP0_MAJOR_REV_SHIFT       (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP0_MAJOR_REV_MAX         (0x000000FFU)

/* XECP_SUPP_USB3_CAP1 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP1_USB_STRING_MASK       (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP1_USB_STRING_SHIFT      (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP1_USB_STRING_MAX        (0xFFFFFFFFU)

/* XECP_SUPP_USB3_CAP2 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP2_COMPATIBLE_PORT_OFFSET_MASK (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP2_COMPATIBLE_PORT_OFFSET_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP2_COMPATIBLE_PORT_OFFSET_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP2_COMPATIBLE_PORT_COUNT_MASK (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP2_COMPATIBLE_PORT_COUNT_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP2_COMPATIBLE_PORT_COUNT_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP2_RSVDP1_MASK           (0x0FFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP2_RSVDP1_SHIFT          (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP2_RSVDP1_MAX            (0x00000FFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP2_PSIC_MASK             (0xF0000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP2_PSIC_SHIFT            (0x0000001CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_CAP2_PSIC_MAX              (0x0000000FU)

/* XECP_SUPP_USB3_PROTOCOL_SLOT_TYPE */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_PROTOCOL_SLOT_TYPE_PST_MASK (0x0000001FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_PROTOCOL_SLOT_TYPE_PST_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_PROTOCOL_SLOT_TYPE_PST_MAX (0x0000001FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_PROTOCOL_SLOT_TYPE_RSVDP1_MASK (0xFFFFFFE0U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_PROTOCOL_SLOT_TYPE_RSVDP1_SHIFT (0x00000005U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_SUPP_USB3_PROTOCOL_SLOT_TYPE_RSVDP1_MAX (0x07FFFFFFU)

/* PSI_SUPER_SPEED */

#define CSL_USB3P0SS_CTRL_XHCI_PSI_SUPER_SPEED_PSIV_MASK                 (0x0000000FU)
#define CSL_USB3P0SS_CTRL_XHCI_PSI_SUPER_SPEED_PSIV_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_PSI_SUPER_SPEED_PSIV_MAX                  (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_PSI_SUPER_SPEED_PSIE_MASK                 (0x00000030U)
#define CSL_USB3P0SS_CTRL_XHCI_PSI_SUPER_SPEED_PSIE_SHIFT                (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_PSI_SUPER_SPEED_PSIE_MAX                  (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_PSI_SUPER_SPEED_PLT_MASK                  (0x000000C0U)
#define CSL_USB3P0SS_CTRL_XHCI_PSI_SUPER_SPEED_PLT_SHIFT                 (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_PSI_SUPER_SPEED_PLT_MAX                   (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_PSI_SUPER_SPEED_PFD_MASK                  (0x00000100U)
#define CSL_USB3P0SS_CTRL_XHCI_PSI_SUPER_SPEED_PFD_SHIFT                 (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_PSI_SUPER_SPEED_PFD_MAX                   (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_PSI_SUPER_SPEED_RSVDP1_MASK               (0x0000FE00U)
#define CSL_USB3P0SS_CTRL_XHCI_PSI_SUPER_SPEED_RSVDP1_SHIFT              (0x00000009U)
#define CSL_USB3P0SS_CTRL_XHCI_PSI_SUPER_SPEED_RSVDP1_MAX                (0x0000007FU)

#define CSL_USB3P0SS_CTRL_XHCI_PSI_SUPER_SPEED_PSIM_MASK                 (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_PSI_SUPER_SPEED_PSIM_SHIFT                (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_PSI_SUPER_SPEED_PSIM_MAX                  (0x0000FFFFU)

/* XECP_CMDM_STS0 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_VEND_DEF_CMDM_CAP_ID_193_MASK (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_VEND_DEF_CMDM_CAP_ID_193_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_VEND_DEF_CMDM_CAP_ID_193_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_XECP_CMDM_NEXT_CAP_OFFSET_MASK (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_XECP_CMDM_NEXT_CAP_OFFSET_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_XECP_CMDM_NEXT_CAP_OFFSET_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_CMD_RUNNING_MASK           (0x00010000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_CMD_RUNNING_SHIFT          (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_CMD_RUNNING_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_HOST_CMD_DB_RANG_STICKY_MASK (0x00020000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_HOST_CMD_DB_RANG_STICKY_SHIFT (0x00000011U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_HOST_CMD_DB_RANG_STICKY_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_STOPPING_CMD_RING_MASK     (0x00040000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_STOPPING_CMD_RING_SHIFT    (0x00000012U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_STOPPING_CMD_RING_MAX      (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_RESERVED_R_MASK            (0x00080000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_RESERVED_R_SHIFT           (0x00000013U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_RESERVED_R_MAX             (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_TRM_STALL_REQ_MASK         (0x00100000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_TRM_STALL_REQ_SHIFT        (0x00000014U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_TRM_STALL_REQ_MAX          (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_TRM_EPERR_UPD_REQ_MASK     (0x00200000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_TRM_EPERR_UPD_REQ_SHIFT    (0x00000015U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_TRM_EPERR_UPD_REQ_MAX      (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_DBM_EP_UPD_REQ_MASK        (0x00400000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_DBM_EP_UPD_REQ_SHIFT       (0x00000016U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_DBM_EP_UPD_REQ_MAX         (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_UPDATE_ENDPT_ACTIVE_MASK   (0x00800000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_UPDATE_ENDPT_ACTIVE_SHIFT  (0x00000017U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_UPDATE_ENDPT_ACTIVE_MAX    (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_ODMA_ADDRESS_DEV_PENDING_MASK (0x01000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_ODMA_ADDRESS_DEV_PENDING_SHIFT (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_ODMA_ADDRESS_DEV_PENDING_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_ODMA_ADDRESS_DEV_DONE_MASK (0x02000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_ODMA_ADDRESS_DEV_DONE_SHIFT (0x00000019U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_ODMA_ADDRESS_DEV_DONE_MAX  (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_CMDM_CLR_DB_REQ_MASK       (0x04000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_CMDM_CLR_DB_REQ_SHIFT      (0x0000001AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_CMDM_CLR_DB_REQ_MAX        (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_CMDM_STOP_REQ_MASK         (0x08000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_CMDM_STOP_REQ_SHIFT        (0x0000001BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_CMDM_STOP_REQ_MAX          (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_CMDM_CNTX_LOCK_REQ_MASK    (0x10000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_CMDM_CNTX_LOCK_REQ_SHIFT   (0x0000001CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_CMDM_CNTX_LOCK_REQ_MAX     (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_TRM_CNTX_IN_USE_MASK       (0x20000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_TRM_CNTX_IN_USE_SHIFT      (0x0000001DU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_TRM_CNTX_IN_USE_MAX        (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_ODMA_CNTX_IN_USE_MASK      (0x40000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_ODMA_CNTX_IN_USE_SHIFT     (0x0000001EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_ODMA_CNTX_IN_USE_MAX       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_IDMA_CNTX_IN_USE_MASK      (0x80000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_IDMA_CNTX_IN_USE_SHIFT     (0x0000001FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_STS0_IDMA_CNTX_IN_USE_MAX       (0x00000001U)

/* XECP_CMDM_RESERVED_1 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_RESERVED_1_RESERVED_R_MASK      (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_RESERVED_1_RESERVED_R_SHIFT     (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_RESERVED_1_RESERVED_R_MAX       (0xFFFFFFFFU)

/* XECP_CMDM_RESERVED_2 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_RESERVED_2_RESERVED_R_MASK      (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_RESERVED_2_RESERVED_R_SHIFT     (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_RESERVED_2_RESERVED_R_MAX       (0xFFFFFFFFU)

/* XECP_CMDM_RESERVED_3 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_RESERVED_3_RESERVED_R_MASK      (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_RESERVED_3_RESERVED_R_SHIFT     (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_RESERVED_3_RESERVED_R_MAX       (0xFFFFFFFFU)

/* XECP_CMDM_RESERVED_4 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_RESERVED_4_RESERVED_R_MASK      (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_RESERVED_4_RESERVED_R_SHIFT     (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_RESERVED_4_RESERVED_R_MAX       (0xFFFFFFFFU)

/* XECP_CMDM_RESERVED_5 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_RESERVED_5_RESERVED_R_MASK      (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_RESERVED_5_RESERVED_R_SHIFT     (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_RESERVED_5_RESERVED_R_MAX       (0xFFFFFFFFU)

/* XECP_CMDM_CTRL_REG1 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_UPDATE_ENDPT_EVENT_ENABLE_MASK (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_UPDATE_ENDPT_EVENT_ENABLE_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_UPDATE_ENDPT_EVENT_ENABLE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_FORCE_BANDWIDTH_FAIL_MASK (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_FORCE_BANDWIDTH_FAIL_SHIFT (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_FORCE_BANDWIDTH_FAIL_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_CNTX_4SETDQPTR_MASK (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_CNTX_4SETDQPTR_SHIFT (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_CNTX_4SETDQPTR_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_ADDR_DEV_SLST_BSR_CHECK_EN_MASK (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_ADDR_DEV_SLST_BSR_CHECK_EN_SHIFT (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_ADDR_DEV_SLST_BSR_CHECK_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_CNTX_4RSTDEV_MASK (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_CNTX_4RSTDEV_SHIFT (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_CNTX_4RSTDEV_MAX  (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_CNTX_4ENSLOT_REG_MASK (0x00000020U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_CNTX_4ENSLOT_REG_SHIFT (0x00000005U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_CNTX_4ENSLOT_REG_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_CNTX_4ENADDR_MASK (0x00000040U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_CNTX_4ENADDR_SHIFT (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_CNTX_4ENADDR_MAX  (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_CNTX_4ENCFGENDPT_MASK (0x00000080U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_CNTX_4ENCFGENDPT_SHIFT (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_CNTX_4ENCFGENDPT_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_EP_CNTX_4EN_SLOT_MASK (0x00000100U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_EP_CNTX_4EN_SLOT_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_EP_CNTX_4EN_SLOT_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_EP_CNTX_4RST_ENDPT_MASK (0x00000200U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_EP_CNTX_4RST_ENDPT_SHIFT (0x00000009U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_EP_CNTX_4RST_ENDPT_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_EP_CNTX_4RST_DEVICE_MASK (0x00000400U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_EP_CNTX_4RST_DEVICE_SHIFT (0x0000000AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_EP_CNTX_4RST_DEVICE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_EP_CNTX_4CFG_ENDPT_MASK (0x00000800U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_EP_CNTX_4CFG_ENDPT_SHIFT (0x0000000BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_EP_CNTX_4CFG_ENDPT_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_GLOB_TSP_EN_MASK      (0x00001000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_GLOB_TSP_EN_SHIFT     (0x0000000CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_GLOB_TSP_EN_MAX       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_INIT_RETRY_MASK       (0x00002000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_INIT_RETRY_SHIFT      (0x0000000DU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_INIT_RETRY_MAX        (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_EVAL_EPST_CHECK_EN_MASK (0x00004000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_EVAL_EPST_CHECK_EN_SHIFT (0x0000000EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_EVAL_EPST_CHECK_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_EP_CNTX_4DIS_SLOT_MASK (0x00008000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_EP_CNTX_4DIS_SLOT_SHIFT (0x0000000FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_EP_CNTX_4DIS_SLOT_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_SPLIT_STATE_WITH_TSPSET_MASK (0x00010000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_SPLIT_STATE_WITH_TSPSET_SHIFT (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CLR_SPLIT_STATE_WITH_TSPSET_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_FORCE_BANDWIDTH_PASS_MASK (0x00020000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_FORCE_BANDWIDTH_PASS_SHIFT (0x00000011U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_FORCE_BANDWIDTH_PASS_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_FORCE_BANDWIDTH_SYS_PASS_MASK (0x00040000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_FORCE_BANDWIDTH_SYS_PASS_SHIFT (0x00000012U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_FORCE_BANDWIDTH_SYS_PASS_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_REPORT_BANDWIDTH_SKIP_SCAN_MASK (0x00080000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_REPORT_BANDWIDTH_SKIP_SCAN_SHIFT (0x00000013U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_REPORT_BANDWIDTH_SKIP_SCAN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_ENABLE_MAX_EP_CACHE_MASK (0x00100000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_ENABLE_MAX_EP_CACHE_SHIFT (0x00000014U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_ENABLE_MAX_EP_CACHE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CFG_ENDPT_CNTX_LOCK_DIS_MASK (0x00200000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CFG_ENDPT_CNTX_LOCK_DIS_SHIFT (0x00000015U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_CFG_ENDPT_CNTX_LOCK_DIS_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_FEATURE_RETRY_EN_MASK (0x00400000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_FEATURE_RETRY_EN_SHIFT (0x00000016U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_FEATURE_RETRY_EN_MAX  (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_EVAL_CNTX_BW_SCAN_EN_MASK (0x00800000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_EVAL_CNTX_BW_SCAN_EN_SHIFT (0x00000017U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_EVAL_CNTX_BW_SCAN_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_DEFAULT_ISOCH_EP_BANDWIDTH_MASK (0x0F000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_DEFAULT_ISOCH_EP_BANDWIDTH_SHIFT (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_DEFAULT_ISOCH_EP_BANDWIDTH_MAX (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_DEFAULT_INTR_EP_BANDWIDTH_MASK (0xF0000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_DEFAULT_INTR_EP_BANDWIDTH_SHIFT (0x0000001CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG1_DEFAULT_INTR_EP_BANDWIDTH_MAX (0x0000000FU)

/* XECP_CMDM_CTRL_REG2 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_CLR_ST_MASK           (0x00003FFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_CLR_ST_SHIFT          (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_CLR_ST_MAX            (0x00003FFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_BURST_SIZE_DEFAULT_EN_MASK (0x00004000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_BURST_SIZE_DEFAULT_EN_SHIFT (0x0000000EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_BURST_SIZE_DEFAULT_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_DISABLE_STALL_CLR_EP_MASK (0x00008000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_DISABLE_STALL_CLR_EP_SHIFT (0x0000000FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_DISABLE_STALL_CLR_EP_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_INCREASE_UPDATE_EP_PRIORITY_EN_MASK (0x00010000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_INCREASE_UPDATE_EP_PRIORITY_EN_SHIFT (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_INCREASE_UPDATE_EP_PRIORITY_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_TRM_BREAK_LOOP_EN_MASK (0x00020000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_TRM_BREAK_LOOP_EN_SHIFT (0x00000011U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_TRM_BREAK_LOOP_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_TSP_4SET_DQPTR_REG_MASK (0x00040000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_TSP_4SET_DQPTR_REG_SHIFT (0x00000012U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_TSP_4SET_DQPTR_REG_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_FORCE_RESET_ENDPT_REG_MASK (0x00080000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_FORCE_RESET_ENDPT_REG_SHIFT (0x00000013U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_FORCE_RESET_ENDPT_REG_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_SET_DQPTR_CLR_EP_ARYS_MASK (0x00100000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_SET_DQPTR_CLR_EP_ARYS_SHIFT (0x00000014U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_SET_DQPTR_CLR_EP_ARYS_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_CMD_ST_DIS_REG_MASK   (0x00200000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_CMD_ST_DIS_REG_SHIFT  (0x00000015U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_CMD_ST_DIS_REG_MAX    (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_ENABLE_BW_CAL_MASK    (0x00400000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_ENABLE_BW_CAL_SHIFT   (0x00000016U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_ENABLE_BW_CAL_MAX     (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_CLR_CNTX_4RST_ENDPT_REG_MASK (0x00800000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_CLR_CNTX_4RST_ENDPT_REG_SHIFT (0x00000017U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_CLR_CNTX_4RST_ENDPT_REG_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_MOVE_XFER_DQPTR_2CPL_DQPTR_EN_MASK (0x01000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_MOVE_XFER_DQPTR_2CPL_DQPTR_EN_SHIFT (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_MOVE_XFER_DQPTR_2CPL_DQPTR_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_CLR_EP_CNTX_4STALL_UPD_MASK (0x02000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_CLR_EP_CNTX_4STALL_UPD_SHIFT (0x00000019U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_CLR_EP_CNTX_4STALL_UPD_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_CLR_EP_CNTX_4STOP_ENDPT_MASK (0x04000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_CLR_EP_CNTX_4STOP_ENDPT_SHIFT (0x0000001AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_CLR_EP_CNTX_4STOP_ENDPT_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_ALL_CLK_GATE_DIS_MASK (0x08000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_ALL_CLK_GATE_DIS_SHIFT (0x0000001BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_ALL_CLK_GATE_DIS_MAX  (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_SRE_MASK              (0x10000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_SRE_SHIFT             (0x0000001CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_SRE_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_SLOT_ID_OVERIDE_EN_MASK (0x20000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_SLOT_ID_OVERIDE_EN_SHIFT (0x0000001DU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_SLOT_ID_OVERIDE_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_STOP_ENDPT_2MS_TIMEOUT_EN_MASK (0x40000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_STOP_ENDPT_2MS_TIMEOUT_EN_SHIFT (0x0000001EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_STOP_ENDPT_2MS_TIMEOUT_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_DOING_2DW_OCNTX_WR_EN_MASK (0x80000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_DOING_2DW_OCNTX_WR_EN_SHIFT (0x0000001FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG2_DOING_2DW_OCNTX_WR_EN_MAX (0x00000001U)

/* XECP_CMDM_CTRL_REG3 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_DEFAULT_PORT_BANDWD_AVAIL_MASK (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_DEFAULT_PORT_BANDWD_AVAIL_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_DEFAULT_PORT_BANDWD_AVAIL_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_DEFAULT_HS_BANDWD_AVAIL_MASK (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_DEFAULT_HS_BANDWD_AVAIL_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_DEFAULT_HS_BANDWD_AVAIL_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_DISABLE_SLOT_TIMER_SELECT_MASK (0x00030000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_DISABLE_SLOT_TIMER_SELECT_SHIFT (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_DISABLE_SLOT_TIMER_SELECT_MAX (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_CLR_TRM_DMA_CNTX_EN_MASK (0x00040000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_CLR_TRM_DMA_CNTX_EN_SHIFT (0x00000012U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_CLR_TRM_DMA_CNTX_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_STOP_2TIMEOUT_EN_MASK (0x00080000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_STOP_2TIMEOUT_EN_SHIFT (0x00000013U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_STOP_2TIMEOUT_EN_MAX  (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_BREAK_CNTX_LOCK_EN_MASK (0x00100000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_BREAK_CNTX_LOCK_EN_SHIFT (0x00000014U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_BREAK_CNTX_LOCK_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_STOP_EP_CLR_STREAM_ST_EN_MASK (0x00200000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_STOP_EP_CLR_STREAM_ST_EN_SHIFT (0x00000015U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_STOP_EP_CLR_STREAM_ST_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_ALLOW_CLR_4STOP_MASK  (0x00400000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_ALLOW_CLR_4STOP_SHIFT (0x00000016U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_ALLOW_CLR_4STOP_MAX   (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_STREAM_ALWAYS_UPDATE_CNTX_MASK (0x00800000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_STREAM_ALWAYS_UPDATE_CNTX_SHIFT (0x00000017U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_STREAM_ALWAYS_UPDATE_CNTX_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_SET_BURST_SIZE_4PRDC_DIS_MASK (0x01000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_SET_BURST_SIZE_4PRDC_DIS_SHIFT (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_SET_BURST_SIZE_4PRDC_DIS_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_STOP_EP_CLR_LCSTREAM_ID_EN_MASK (0x02000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_STOP_EP_CLR_LCSTREAM_ID_EN_SHIFT (0x00000019U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_STOP_EP_CLR_LCSTREAM_ID_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_UPDATE_CNTX_WHEN_STOPPED_MASK (0x04000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_UPDATE_CNTX_WHEN_STOPPED_SHIFT (0x0000001AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_UPDATE_CNTX_WHEN_STOPPED_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_DISABLE_SETDQPTR_CLR_STREAM_ST_MASK (0x08000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_DISABLE_SETDQPTR_CLR_STREAM_ST_SHIFT (0x0000001BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_DISABLE_SETDQPTR_CLR_STREAM_ST_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_DISABLE_NON0EP_CNTX_CLR_MASK (0x10000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_DISABLE_NON0EP_CNTX_CLR_SHIFT (0x0000001CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_DISABLE_NON0EP_CNTX_CLR_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_EXTRA_DB_RM_4STOP_EN_MASK (0x20000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_EXTRA_DB_RM_4STOP_EN_SHIFT (0x0000001DU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_EXTRA_DB_RM_4STOP_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_IGNORE_HI_ATOMIC_EN_MASK (0x40000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_IGNORE_HI_ATOMIC_EN_SHIFT (0x0000001EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_IGNORE_HI_ATOMIC_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_FRINDEX_WR_EN_MASK    (0x80000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_FRINDEX_WR_EN_SHIFT   (0x0000001FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_CMDM_CTRL_REG3_FRINDEX_WR_EN_MAX     (0x00000001U)

/* XECP_HOST_CTRL_CAP */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_CAP_VEND_DEF_HOST_CAP_ID_192_MASK (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_CAP_VEND_DEF_HOST_CAP_ID_192_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_CAP_VEND_DEF_HOST_CAP_ID_192_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_CAP_XECP_HOST_NEXT_CAP_OFFSET_MASK (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_CAP_XECP_HOST_NEXT_CAP_OFFSET_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_CAP_XECP_HOST_NEXT_CAP_OFFSET_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_CAP_RESERVED_R_MASK        (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_CAP_RESERVED_R_SHIFT       (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_CAP_RESERVED_R_MAX         (0x0000FFFFU)

/* XECP_HOST_CTRL_RSVD */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_RSVD_RESERVED_MASK         (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_RSVD_RESERVED_SHIFT        (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_RSVD_RESERVED_MAX          (0xFFFFFFFFU)

/* XECP_HOST_CLR_MASK_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_MASK_REG_EP_DIR_MASK        (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_MASK_REG_EP_DIR_SHIFT       (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_MASK_REG_EP_DIR_MAX         (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_MASK_REG_EP_NUM_MASK        (0x0000001EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_MASK_REG_EP_NUM_SHIFT       (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_MASK_REG_EP_NUM_MAX         (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_MASK_REG_SLOT_NUM_MASK      (0x000003E0U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_MASK_REG_SLOT_NUM_SHIFT     (0x00000005U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_MASK_REG_SLOT_NUM_MAX       (0x0000001FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_MASK_REG_RESERVED_R_MASK    (0xFFFFFC00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_MASK_REG_RESERVED_R_SHIFT   (0x0000000AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_MASK_REG_RESERVED_R_MAX     (0x003FFFFFU)

/* XECP_HOST_CLR_IN_EP_VALID_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_IN_EP_VALID_REG_PORT_NUM_MASK (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_IN_EP_VALID_REG_PORT_NUM_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_IN_EP_VALID_REG_PORT_NUM_MAX (0xFFFFFFFFU)

/* XECP_HOST_CLR_PMASK_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_PMASK_REG_EP_DIR_MASK       (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_PMASK_REG_EP_DIR_SHIFT      (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_PMASK_REG_EP_DIR_MAX        (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_PMASK_REG_EP_NUM_MASK       (0x0000001EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_PMASK_REG_EP_NUM_SHIFT      (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_PMASK_REG_EP_NUM_MAX        (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_PMASK_REG_SLOT_NUM_MASK     (0x000003E0U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_PMASK_REG_SLOT_NUM_SHIFT    (0x00000005U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_PMASK_REG_SLOT_NUM_MAX      (0x0000001FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_PMASK_REG_RESERVED_R_MASK   (0xFFFFFC00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_PMASK_REG_RESERVED_R_SHIFT  (0x0000000AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CLR_PMASK_REG_RESERVED_R_MAX    (0x003FFFFFU)

/* XECP_HOST_CTRL_OCRD_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_OCRD_REG_PORT_NUM_MASK     (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_OCRD_REG_PORT_NUM_SHIFT    (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_OCRD_REG_PORT_NUM_MAX      (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_OCRD_REG_RESERVED_R_MASK   (0x03FFFF00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_OCRD_REG_RESERVED_R_SHIFT  (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_OCRD_REG_RESERVED_R_MAX    (0x0003FFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_OCRD_REG_ST_UPD_REG_MASK   (0x04000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_OCRD_REG_ST_UPD_REG_SHIFT  (0x0000001AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_OCRD_REG_ST_UPD_REG_MAX    (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_OCRD_REG_MINUS_4RFIFO_MASK (0x08000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_OCRD_REG_MINUS_4RFIFO_SHIFT (0x0000001BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_OCRD_REG_MINUS_4RFIFO_MAX  (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_OCRD_REG_MINUS_OCRD_MASK   (0x10000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_OCRD_REG_MINUS_OCRD_SHIFT  (0x0000001CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_OCRD_REG_MINUS_OCRD_MAX    (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_OCRD_REG_PLUS_OCRD_MASK    (0x20000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_OCRD_REG_PLUS_OCRD_SHIFT   (0x0000001DU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_OCRD_REG_PLUS_OCRD_MAX     (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_OCRD_REG_CLR_CPL_ST_MASK   (0x40000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_OCRD_REG_CLR_CPL_ST_SHIFT  (0x0000001EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_OCRD_REG_CLR_CPL_ST_MAX    (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_OCRD_REG_CLR_XFER_ST_MASK  (0x80000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_OCRD_REG_CLR_XFER_ST_SHIFT (0x0000001FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_OCRD_REG_CLR_XFER_ST_MAX   (0x00000001U)

/* XECP_HOST_CTRL_TEST_BUS_LO */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TEST_BUS_LO_TEST_BUS_LO_MASK (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TEST_BUS_LO_TEST_BUS_LO_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TEST_BUS_LO_TEST_BUS_LO_MAX (0xFFFFFFFFU)

/* XECP_HOST_CTRL_TEST_BUS_HI */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TEST_BUS_HI_TEST_BUS_HI_MASK (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TEST_BUS_HI_TEST_BUS_HI_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TEST_BUS_HI_TEST_BUS_HI_MAX (0xFFFFFFFFU)

/* XECP_HOST_CTRL_TRM_REG1 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_IN_TD_PACE_ENABLE_MASK (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_IN_TD_PACE_ENABLE_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_IN_TD_PACE_ENABLE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_DISABLE_FC_4INRDY_MASK (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_DISABLE_FC_4INRDY_SHIFT (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_DISABLE_FC_4INRDY_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_LINK_NOP_SUCESS_EN_MASK (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_LINK_NOP_SUCESS_EN_SHIFT (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_LINK_NOP_SUCESS_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_DISABLE_STALL_MASK (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_DISABLE_STALL_SHIFT (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_DISABLE_STALL_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_XPORT_CRD_DISABLE_MASK (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_XPORT_CRD_DISABLE_SHIFT (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_XPORT_CRD_DISABLE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_CPL_PKT_CLR_MASK_EN_MASK (0x00000020U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_CPL_PKT_CLR_MASK_EN_SHIFT (0x00000005U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_CPL_PKT_CLR_MASK_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_EN_BB_PORT_DISABLE_MASK (0x00000040U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_EN_BB_PORT_DISABLE_SHIFT (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_EN_BB_PORT_DISABLE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_NPKT0_FC_DISABLE_MASK (0x00000080U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_NPKT0_FC_DISABLE_SHIFT (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_NPKT0_FC_DISABLE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_TRB_CACHE_INVALIDE_EN_MASK (0x00000100U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_TRB_CACHE_INVALIDE_EN_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_TRB_CACHE_INVALIDE_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_IN_NPKT_PACE_DISABLE_MASK (0x00000200U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_IN_NPKT_PACE_DISABLE_SHIFT (0x00000009U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_IN_NPKT_PACE_DISABLE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_FLUSH_2CLR_VALID_EN_MASK (0x00000400U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_FLUSH_2CLR_VALID_EN_SHIFT (0x0000000AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_FLUSH_2CLR_VALID_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_CTRL_REG_CLR_BNDRY_MASK (0x00000800U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_CTRL_REG_CLR_BNDRY_SHIFT (0x0000000BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_CTRL_REG_CLR_BNDRY_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_ENT_EN_MASK       (0x00001000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_ENT_EN_SHIFT      (0x0000000CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_ENT_EN_MAX        (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_SINGLE_BURST_EN_MASK (0x00002000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_SINGLE_BURST_EN_SHIFT (0x0000000DU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_SINGLE_BURST_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_XFER_BLOCK_EN_MASK (0x00004000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_XFER_BLOCK_EN_SHIFT (0x0000000EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_XFER_BLOCK_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_ISO_0LEN_LPF_EN_MASK (0x00008000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_ISO_0LEN_LPF_EN_SHIFT (0x0000000FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_ISO_0LEN_LPF_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_SET_ADDR_ERR_EN_MASK (0x00010000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_SET_ADDR_ERR_EN_SHIFT (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_SET_ADDR_ERR_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_PHASE1_IMD_EN_MASK (0x00020000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_PHASE1_IMD_EN_SHIFT (0x00000011U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_PHASE1_IMD_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_NO_OP_AS_TD_MASK  (0x00040000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_NO_OP_AS_TD_SHIFT (0x00000012U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_NO_OP_AS_TD_MAX   (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_SHORT_ERR_4MSI_EN_MASK (0x00080000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_SHORT_ERR_4MSI_EN_SHIFT (0x00000013U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_SHORT_ERR_4MSI_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_DEADLOCK_DETECT_EN_MASK (0x00100000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_DEADLOCK_DETECT_EN_SHIFT (0x00000014U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_DEADLOCK_DETECT_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_DISABLE_ERDY_DROP_MASK (0x00200000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_DISABLE_ERDY_DROP_SHIFT (0x00000015U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_DISABLE_ERDY_DROP_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_CPL_DB_RANG_EN_MASK (0x00400000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_CPL_DB_RANG_EN_SHIFT (0x00000016U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_CPL_DB_RANG_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_WRITE_ERDP_LO_MASK (0x00800000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_WRITE_ERDP_LO_SHIFT (0x00000017U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_WRITE_ERDP_LO_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_DISABLE_IMD_4NODMA_MASK (0x01000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_DISABLE_IMD_4NODMA_SHIFT (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_DISABLE_IMD_4NODMA_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_TRB_ERR_RM_DB_EN_MASK (0x02000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_TRB_ERR_RM_DB_EN_SHIFT (0x00000019U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_TRB_ERR_RM_DB_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_EP_HALT_2RETRY_EN_MASK (0x04000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_EP_HALT_2RETRY_EN_SHIFT (0x0000001AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_EP_HALT_2RETRY_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_ENABLE_NOOP_UPD_MASK (0x08000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_ENABLE_NOOP_UPD_SHIFT (0x0000001BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_ENABLE_NOOP_UPD_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_DISABLE_CPL_SST_PPIPE_ERR_MASK (0x10000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_DISABLE_CPL_SST_PPIPE_ERR_SHIFT (0x0000001CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_DISABLE_CPL_SST_PPIPE_ERR_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_DISABLE_CPL_SST_MDATA_ERR_MASK (0x20000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_DISABLE_CPL_SST_MDATA_ERR_SHIFT (0x0000001DU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_DISABLE_CPL_SST_MDATA_ERR_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_USB2_NAK_AUTO_DETECT_REG_EN_MASK (0x40000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_USB2_NAK_AUTO_DETECT_REG_EN_SHIFT (0x0000001EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_USB2_NAK_AUTO_DETECT_REG_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_TRB_PACE_EN_MASK  (0x80000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_TRB_PACE_EN_SHIFT (0x0000001FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG1_TRB_PACE_EN_MAX   (0x00000001U)

/* XECP_HOST_CTRL_SCH_REG1 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_POLL_DELAY_DIS_MASK (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_POLL_DELAY_DIS_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_POLL_DELAY_DIS_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_TRM_ACTIVE_IN_EP_VALID_MASK (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_TRM_ACTIVE_IN_EP_VALID_SHIFT (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_TRM_ACTIVE_IN_EP_VALID_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_2_MASK        (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_2_SHIFT       (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_2_MAX         (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_TTE_ENABLE_INTROUT_OVERLAP_STOP_MASK (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_TTE_ENABLE_INTROUT_OVERLAP_STOP_SHIFT (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_TTE_ENABLE_INTROUT_OVERLAP_STOP_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_SORT_PATTERN_MASK (0x00000030U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_SORT_PATTERN_SHIFT (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_SORT_PATTERN_MAX (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_ASYNC_OUT_MAX_PERF_MASK (0x00000040U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_ASYNC_OUT_MAX_PERF_SHIFT (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_ASYNC_OUT_MAX_PERF_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_ASYNC_1PKT_PERF_MASK (0x00000080U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_ASYNC_1PKT_PERF_SHIFT (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_ASYNC_1PKT_PERF_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCRATCH_PAD_EN_MASK (0x00000100U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCRATCH_PAD_EN_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCRATCH_PAD_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_MAXEP_MASK        (0x00000600U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_MAXEP_SHIFT       (0x00000009U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_MAXEP_MAX         (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_CACHE_SIZE_CTRL_MASK (0x00001800U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_CACHE_SIZE_CTRL_SHIFT (0x0000000BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_CACHE_SIZE_CTRL_MAX (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_TTE_0_MASK        (0x00002000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_TTE_0_SHIFT       (0x0000000DU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_TTE_0_MAX         (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_TTE_1_MASK        (0x00004000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_TTE_1_SHIFT       (0x0000000EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_TTE_1_MAX         (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_TTE_2_MASK        (0x00008000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_TTE_2_SHIFT       (0x0000000FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_TTE_2_MAX         (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_TTE_3_MASK        (0x00030000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_TTE_3_SHIFT       (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_TTE_3_MAX         (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_DISABLE_GL_HUB_ISO_FIX_MASK (0x00040000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_DISABLE_GL_HUB_ISO_FIX_SHIFT (0x00000012U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_DISABLE_GL_HUB_ISO_FIX_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_DISABLE_GL_HUB_INT_FIX_MASK (0x00080000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_DISABLE_GL_HUB_INT_FIX_SHIFT (0x00000013U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_DISABLE_GL_HUB_INT_FIX_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_TTE_4_MASK        (0x00100000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_TTE_4_SHIFT       (0x00000014U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_TTE_4_MAX         (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_STOP_SERVE_NC_MASK (0x00200000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_STOP_SERVE_NC_SHIFT (0x00000015U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_STOP_SERVE_NC_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_CCLK_PRDC_DONE_CHECK_MASK (0x00C00000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_CCLK_PRDC_DONE_CHECK_SHIFT (0x00000016U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_CCLK_PRDC_DONE_CHECK_MAX (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_ASYNC_PRDC_CC_DIS_MASK (0x01000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_ASYNC_PRDC_CC_DIS_SHIFT (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_ASYNC_PRDC_CC_DIS_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_TT_OVERLAP_ALL_INS_MASK (0x02000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_TT_OVERLAP_ALL_INS_SHIFT (0x00000019U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_TT_OVERLAP_ALL_INS_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_ASYNC_1PKT_SPLIT_PREF_MASK (0x04000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_ASYNC_1PKT_SPLIT_PREF_SHIFT (0x0000001AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_ASYNC_1PKT_SPLIT_PREF_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_BLOCK_PENDING_EN_MASK (0x08000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_BLOCK_PENDING_EN_SHIFT (0x0000001BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_BLOCK_PENDING_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_LIMIT_PRDC_MASK (0xF0000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_LIMIT_PRDC_SHIFT (0x0000001CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG1_SCH_LIMIT_PRDC_MAX (0x0000000FU)

/* XECP_HOST_CTRL_ODMA_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_EP_TRANS_TIMEOUT_EN_MASK (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_EP_TRANS_TIMEOUT_EN_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_EP_TRANS_TIMEOUT_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_EP_TRANS_TIMEOUT_LEN_MASK (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_EP_TRANS_TIMEOUT_LEN_SHIFT (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_EP_TRANS_TIMEOUT_LEN_MAX (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_ODMA_RD_TO_IDLE_MASK (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_ODMA_RD_TO_IDLE_SHIFT (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_ODMA_RD_TO_IDLE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_ODMA_RESP_TO_IDLE_MASK (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_ODMA_RESP_TO_IDLE_SHIFT (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_ODMA_RESP_TO_IDLE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_ODMA_COMPLETION_TO_IDLE_MASK (0x00000020U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_ODMA_COMPLETION_TO_IDLE_SHIFT (0x00000005U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_ODMA_COMPLETION_TO_IDLE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_ODMA_SET_ADDR_TO_IDLE_MASK (0x00000040U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_ODMA_SET_ADDR_TO_IDLE_SHIFT (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_ODMA_SET_ADDR_TO_IDLE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_ODMA_7_MASK       (0x00000080U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_ODMA_7_SHIFT      (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_ODMA_7_MAX        (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_CLEAR_CNTX_LOCKS_MASK (0x00000100U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_CLEAR_CNTX_LOCKS_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_CLEAR_CNTX_LOCKS_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_ODMA_9_MASK       (0x00000200U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_ODMA_9_SHIFT      (0x00000009U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_ODMA_9_MAX        (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_EP_TIMER_TICK_MASK (0x00000400U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_EP_TIMER_TICK_SHIFT (0x0000000AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_EP_TIMER_TICK_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_ODMA_11_MASK      (0x00000800U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_ODMA_11_SHIFT     (0x0000000BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_ODMA_11_MAX       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_EP_BASE_TIMER_MASK (0x00001000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_EP_BASE_TIMER_SHIFT (0x0000000CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_EP_BASE_TIMER_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_ACK_CRD_CHECK_EN_MASK (0x00002000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_ACK_CRD_CHECK_EN_SHIFT (0x0000000DU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_ACK_CRD_CHECK_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_SPEED_UP_TIMEOUT_MASK (0x00004000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_SPEED_UP_TIMEOUT_SHIFT (0x0000000EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_SPEED_UP_TIMEOUT_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_RESERVED_RW_MASK  (0xFFFF8000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_RESERVED_RW_SHIFT (0x0000000FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_ODMA_REG_RESERVED_RW_MAX   (0x0001FFFFU)

/* XECP_HOST_CTRL_IDMA_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_EP_TRANS_TIMEOUT_EN_MASK (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_EP_TRANS_TIMEOUT_EN_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_EP_TRANS_TIMEOUT_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_EP_TIMER_TICK_MASK (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_EP_TIMER_TICK_SHIFT (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_EP_TIMER_TICK_MAX (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_PTR_BUF_ROOM_RESTORE_PULSE_MASK (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_PTR_BUF_ROOM_RESTORE_PULSE_SHIFT (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_PTR_BUF_ROOM_RESTORE_PULSE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_RESTORE_RDP_CREDITS_PULSE_MASK (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_RESTORE_RDP_CREDITS_PULSE_SHIFT (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_RESTORE_RDP_CREDITS_PULSE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_ACK_PST_CLR_PULSE_MASK (0x00000020U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_ACK_PST_CLR_PULSE_SHIFT (0x00000005U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_ACK_PST_CLR_PULSE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_DM_PST_CLR_PULSE_MASK (0x00000040U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_DM_PST_CLR_PULSE_SHIFT (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_DM_PST_CLR_PULSE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_CLEAR_CNTX_LOCKS_MASK (0x00000080U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_CLEAR_CNTX_LOCKS_SHIFT (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_CLEAR_CNTX_LOCKS_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_COMPLIANCE_ISO_ENABLE_MASK (0x00000100U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_COMPLIANCE_ISO_ENABLE_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_COMPLIANCE_ISO_ENABLE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_9_MASK       (0x00000200U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_9_SHIFT      (0x00000009U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_9_MAX        (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_TIMER_TICK0_MASK  (0x00000400U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_TIMER_TICK0_SHIFT (0x0000000AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_TIMER_TICK0_MAX   (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_SEQ_NUM_ADJ_ON_NRDY_MASK (0x00000800U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_SEQ_NUM_ADJ_ON_NRDY_SHIFT (0x0000000BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_SEQ_NUM_ADJ_ON_NRDY_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_SPEED_UP_TIMEOUT_MASK (0x00001000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_SPEED_UP_TIMEOUT_SHIFT (0x0000000CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_SPEED_UP_TIMEOUT_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_13_MASK      (0x00002000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_13_SHIFT     (0x0000000DU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_13_MAX       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_14_MASK      (0x00004000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_14_SHIFT     (0x0000000EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_14_MAX       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_15_MASK      (0x00008000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_15_SHIFT     (0x0000000FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_15_MAX       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_PTR_BUF_ROOM_SET_MASK (0x00010000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_PTR_BUF_ROOM_SET_SHIFT (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_PTR_BUF_ROOM_SET_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_17_MASK      (0x00020000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_17_SHIFT     (0x00000011U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_17_MAX       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_ADDR_FIFO_FLUSH_BIT_MASK (0x00040000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_ADDR_FIFO_FLUSH_BIT_SHIFT (0x00000012U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_ADDR_FIFO_FLUSH_BIT_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_23_19_MASK   (0x00F80000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_23_19_SHIFT  (0x00000013U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_23_19_MAX    (0x0000001FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_24_MASK      (0x01000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_24_SHIFT     (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_24_MAX       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_25_MASK      (0x02000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_25_SHIFT     (0x00000019U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_25_MAX       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_TIMER_TICK1_MASK  (0x04000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_TIMER_TICK1_SHIFT (0x0000001AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_TIMER_TICK1_MAX   (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_27_MASK      (0x08000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_27_SHIFT     (0x0000001BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_27_MAX       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_28_MASK      (0x10000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_28_SHIFT     (0x0000001CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_IDMA_28_MAX       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_DB_EVENT_GEN_EN_MASK (0x20000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_DB_EVENT_GEN_EN_SHIFT (0x0000001DU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_DB_EVENT_GEN_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_EVENT_PRIORITY_MASK (0x40000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_EVENT_PRIORITY_SHIFT (0x0000001EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_EVENT_PRIORITY_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_EVENT_FIFO_DIS_MASK (0x80000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_EVENT_FIFO_DIS_SHIFT (0x0000001FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_IDMA_REG_EVENT_FIFO_DIS_MAX (0x00000001U)

/* XECP_HOST_CTRL_PORT_CTRL */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_RES1_MASK        (0x0000000FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_RES1_SHIFT       (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_RES1_MAX         (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_TEST_BUS_SEL_CTRL_BITS_MASK (0x000001F0U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_TEST_BUS_SEL_CTRL_BITS_SHIFT (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_TEST_BUS_SEL_CTRL_BITS_MAX (0x0000001FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_RESERVED_RW_MASK (0x00000600U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_RESERVED_RW_SHIFT (0x00000009U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_RESERVED_RW_MAX  (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_PCIE_GASKET_MASK (0x00000800U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_PCIE_GASKET_SHIFT (0x0000000BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_PCIE_GASKET_MAX  (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_ENABLE_ITP_XMT_MASK (0x00001000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_ENABLE_ITP_XMT_SHIFT (0x0000000CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_ENABLE_ITP_XMT_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_HBUF_WATER_MARK_REG_CCLK_MASK (0x0001E000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_HBUF_WATER_MARK_REG_CCLK_SHIFT (0x0000000DU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_HBUF_WATER_MARK_REG_CCLK_MAX (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_OVERFLOW_SYS_ERR_EN_MASK (0x00020000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_OVERFLOW_SYS_ERR_EN_SHIFT (0x00000011U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_OVERFLOW_SYS_ERR_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_RD_WR_ADDR_CONFLICT_EN_MASK (0x00040000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_RD_WR_ADDR_CONFLICT_EN_SHIFT (0x00000012U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_RD_WR_ADDR_CONFLICT_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_LOCK_HEADER_DATA_EN_MASK (0x00080000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_LOCK_HEADER_DATA_EN_SHIFT (0x00000013U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_LOCK_HEADER_DATA_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_RESERVED_R_MASK  (0xFFF00000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_RESERVED_R_SHIFT (0x00000014U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_CTRL_RESERVED_R_MAX   (0x00000FFFU)

/* XECP_AUX_CTRL_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_FORCE_FD_RST_MASK       (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_FORCE_FD_RST_SHIFT      (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_FORCE_FD_RST_MAX        (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_PERST_4FD_RST_MASK (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_PERST_4FD_RST_SHIFT (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_PERST_4FD_RST_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_PERST_4MAIN_PWRUP_MASK (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_PERST_4MAIN_PWRUP_SHIFT (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_PERST_4MAIN_PWRUP_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_PM_CTRL_MAIN_RST_EN_MASK (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_PM_CTRL_MAIN_RST_EN_SHIFT (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_PM_CTRL_MAIN_RST_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_MAIN_PWRUP_RST_MASK (0x00000020U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_MAIN_PWRUP_RST_SHIFT (0x00000005U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_MAIN_PWRUP_RST_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_WARM_RST_2USB_PHY_MASK (0x00000040U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_WARM_RST_2USB_PHY_SHIFT (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_WARM_RST_2USB_PHY_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_HC_WARM_RST_2USB_PHY_MASK (0x00000080U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_HC_WARM_RST_2USB_PHY_SHIFT (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_HC_WARM_RST_2USB_PHY_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_MAIN_PWRUP_2PCIE_PHY_MASK (0x00000100U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_MAIN_PWRUP_2PCIE_PHY_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_MAIN_PWRUP_2PCIE_PHY_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_MAIN_PWRUP_2PCORE_MASK (0x00000200U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_MAIN_PWRUP_2PCORE_SHIFT (0x00000009U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_MAIN_PWRUP_2PCORE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_MAIN_PWRUP_4U2PORT_MASK (0x00000400U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_MAIN_PWRUP_4U2PORT_SHIFT (0x0000000AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_MAIN_PWRUP_4U2PORT_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_MAIN_PWRUP_4U3PORT_MASK (0x00000800U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_MAIN_PWRUP_4U3PORT_SHIFT (0x0000000BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_MAIN_PWRUP_4U3PORT_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_WARM_RST_4U3PORT_MASK (0x00001000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_WARM_RST_4U3PORT_SHIFT (0x0000000CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_WARM_RST_4U3PORT_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_HOT_RST_4U3PORT_MASK (0x00002000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_HOT_RST_4U3PORT_SHIFT (0x0000000DU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_HOT_RST_4U3PORT_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_PCIE_LINKDOWN_RST_EN_MASK (0x00004000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_PCIE_LINKDOWN_RST_EN_SHIFT (0x0000000EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_PCIE_LINKDOWN_RST_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_WARM_RST_4UPHY_PON_MASK (0x00008000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_WARM_RST_4UPHY_PON_SHIFT (0x0000000FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_WARM_RST_4UPHY_PON_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_MAC_PHY_PIPE_RST_MASK (0x00010000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_MAC_PHY_PIPE_RST_SHIFT (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_MAC_PHY_PIPE_RST_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_HC_RST_2PCIE_PHY_MASK (0x00020000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_HC_RST_2PCIE_PHY_SHIFT (0x00000011U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_HC_RST_2PCIE_PHY_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_EEPROM_LOAD_ON_MAIN_MASK (0x00040000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_EEPROM_LOAD_ON_MAIN_SHIFT (0x00000012U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_EEPROM_LOAD_ON_MAIN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_MAIN_PWRUP_HC_2PCORE_MASK (0x00080000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_MAIN_PWRUP_HC_2PCORE_SHIFT (0x00000013U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_MAIN_PWRUP_HC_2PCORE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_HC_WARM_RST_4UPHY_PON_MASK (0x00100000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_HC_WARM_RST_4UPHY_PON_SHIFT (0x00000014U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_HC_WARM_RST_4UPHY_PON_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_HCRESET_4USB2_MASK (0x00200000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_HCRESET_4USB2_SHIFT (0x00000015U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_HCRESET_4USB2_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_COLD_RST_N_PULSE_MASK   (0x00400000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_COLD_RST_N_PULSE_SHIFT  (0x00000016U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_COLD_RST_N_PULSE_MAX    (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_MAIN_PWRUP_2USB_PHY_MASK (0x00800000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_MAIN_PWRUP_2USB_PHY_SHIFT (0x00000017U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_MAIN_PWRUP_2USB_PHY_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_LINKDOWN_RST_4UPORT_MASK (0x01000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_LINKDOWN_RST_4UPORT_SHIFT (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_LINKDOWN_RST_4UPORT_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_FAST_SIM_RST_MASK       (0x02000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_FAST_SIM_RST_SHIFT      (0x00000019U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_FAST_SIM_RST_MAX        (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_PERST_EN_MASK    (0x04000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_PERST_EN_SHIFT   (0x0000001AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_IGNORE_PERST_EN_MAX     (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_PERST_4MAIN_EN_MASK     (0x08000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_PERST_4MAIN_EN_SHIFT    (0x0000001BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_PERST_4MAIN_EN_MAX      (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_PERST_2PWDOWN_EN_MASK   (0x10000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_PERST_2PWDOWN_EN_SHIFT  (0x0000001CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_PERST_2PWDOWN_EN_MAX    (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_PCIE_PHY_RST_SEL_MASK   (0x20000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_PCIE_PHY_RST_SEL_SHIFT  (0x0000001DU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_PCIE_PHY_RST_SEL_MAX    (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_PERST_FILTER_DIS_MASK   (0x40000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_PERST_FILTER_DIS_SHIFT  (0x0000001EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_PERST_FILTER_DIS_MAX    (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_RESERVED_R_MASK         (0x80000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_RESERVED_R_SHIFT        (0x0000001FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG_RESERVED_R_MAX          (0x00000001U)

/* XECP_HOST_BW_OV_SS_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_SS_REG_SS_BW_CALC_MASK    (0x00000FFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_SS_REG_SS_BW_CALC_SHIFT   (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_SS_REG_SS_BW_CALC_MAX     (0x00000FFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_SS_REG_MAX_TT_BW_MASK     (0x00FFF000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_SS_REG_MAX_TT_BW_SHIFT    (0x0000000CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_SS_REG_MAX_TT_BW_MAX      (0x00000FFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_SS_REG_RESERVED_R_MASK    (0xFF000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_SS_REG_RESERVED_R_SHIFT   (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_SS_REG_RESERVED_R_MAX     (0x000000FFU)

/* XECP_HOST_BW_OV_HS_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_HS_REG_BW_OV_HS_TT_MASK   (0x00000FFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_HS_REG_BW_OV_HS_TT_SHIFT  (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_HS_REG_BW_OV_HS_TT_MAX    (0x00000FFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_HS_REG_BW_OV_HS_MASK      (0x00FFF000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_HS_REG_BW_OV_HS_SHIFT     (0x0000000CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_HS_REG_BW_OV_HS_MAX       (0x00000FFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_HS_REG_RESERVED_R_MASK    (0xFF000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_HS_REG_RESERVED_R_SHIFT   (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_HS_REG_RESERVED_R_MAX     (0x000000FFU)

/* XECP_HOST_BW_OV_FS_LS_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_FS_LS_REG_LS_BW_CALC_MASK (0x00000FFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_FS_LS_REG_LS_BW_CALC_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_FS_LS_REG_LS_BW_CALC_MAX  (0x00000FFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_FS_LS_REG_FS_BW_CALC_MASK (0x00FFF000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_FS_LS_REG_FS_BW_CALC_SHIFT (0x0000000CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_FS_LS_REG_FS_BW_CALC_MAX  (0x00000FFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_FS_LS_REG_RESERVED_R_MASK (0xFF000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_FS_LS_REG_RESERVED_R_SHIFT (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_FS_LS_REG_RESERVED_R_MAX  (0x000000FFU)

/* XECP_HOST_BW_OV_SYS_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_SYS_REG_SYS_BW_CALC_MASK  (0x00000FFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_SYS_REG_SYS_BW_CALC_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_SYS_REG_SYS_BW_CALC_MAX   (0x00000FFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_SYS_REG_BW_OV_SYS_TT_MASK (0x00FFF000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_SYS_REG_BW_OV_SYS_TT_SHIFT (0x0000000CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_SYS_REG_BW_OV_SYS_TT_MAX  (0x00000FFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_SYS_REG_RESERVED_R_MASK   (0xFF000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_SYS_REG_RESERVED_R_SHIFT  (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_BW_OV_SYS_REG_RESERVED_R_MAX    (0x000000FFU)

/* XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_LS_CTRL_DELAY_DEF_MASK (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_LS_CTRL_DELAY_DEF_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_LS_CTRL_DELAY_DEF_MAX (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_LS_CTRL_DELAY_EN_MASK (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_LS_CTRL_DELAY_EN_SHIFT (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_LS_CTRL_DELAY_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_FS_CTRL_DELAY_DEF_MASK (0x00000070U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_FS_CTRL_DELAY_DEF_SHIFT (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_FS_CTRL_DELAY_DEF_MAX (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_FS_CTRL_DELAY_EN_MASK (0x00000080U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_FS_CTRL_DELAY_EN_SHIFT (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_FS_CTRL_DELAY_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_HS_CTRL_DELAY_DEF_MASK (0x00000700U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_HS_CTRL_DELAY_DEF_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_HS_CTRL_DELAY_DEF_MAX (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_HS_CTRL_DELAY_EN_MASK (0x00000800U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_HS_CTRL_DELAY_EN_SHIFT (0x0000000BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_HS_CTRL_DELAY_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_FS_BULK_DELAY_DEF_MASK (0x00007000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_FS_BULK_DELAY_DEF_SHIFT (0x0000000CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_FS_BULK_DELAY_DEF_MAX (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_FS_BULK_DELAY_EN_MASK (0x00008000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_FS_BULK_DELAY_EN_SHIFT (0x0000000FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_FS_BULK_DELAY_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_HS_BULK_DELAY_DEF_MASK (0x00070000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_HS_BULK_DELAY_DEF_SHIFT (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_HS_BULK_DELAY_DEF_MAX (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_HS_BULK_DELAY_EN_MASK (0x00080000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_HS_BULK_DELAY_EN_SHIFT (0x00000013U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_HS_BULK_DELAY_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_RESERVED_R_MASK (0xFFF00000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_RESERVED_R_SHIFT (0x00000014U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_ASYNC_DELAY_REG_RESERVED_R_MAX (0x00000FFFU)

/* XECP_UPORTS_PON_RST_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_UPORTS_PON_RST_REG_USB_PHY_PORT_NUM_MASK (0x0000000FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_UPORTS_PON_RST_REG_USB_PHY_PORT_NUM_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_UPORTS_PON_RST_REG_USB_PHY_PORT_NUM_MAX (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_UPORTS_PON_RST_REG_RESERVED_R_MASK   (0xFFFFFFF0U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_UPORTS_PON_RST_REG_RESERVED_R_SHIFT  (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_UPORTS_PON_RST_REG_RESERVED_R_MAX    (0x0FFFFFFFU)

/* XECP_HOST_CTRL_TRM_REG3 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_CFG_EN_CACHE_MASK (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_CFG_EN_CACHE_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_CFG_EN_CACHE_MAX  (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_CFG_EN_LOOKAHEAD_MASK (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_CFG_EN_LOOKAHEAD_SHIFT (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_CFG_EN_LOOKAHEAD_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_CFG_EN_HIT_INVALID_MASK (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_CFG_EN_HIT_INVALID_SHIFT (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_CFG_EN_HIT_INVALID_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_CFG_CACHE_DEBUG_MASK (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_CFG_CACHE_DEBUG_SHIFT (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_CFG_CACHE_DEBUG_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_CFG_EN_LOOK_POS_MASK (0x00000030U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_CFG_EN_LOOK_POS_SHIFT (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_CFG_EN_LOOK_POS_MAX (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_CFG_EN_DEFER_BC_MASK (0x00000040U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_CFG_EN_DEFER_BC_SHIFT (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_CFG_EN_DEFER_BC_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_CFG_EN_MISS_DOUBLE_MASK (0x00000080U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_CFG_EN_MISS_DOUBLE_SHIFT (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_CFG_EN_MISS_DOUBLE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_CPL_EXTRA_DB_RANG_EN_MASK (0x00000100U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_CPL_EXTRA_DB_RANG_EN_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_CPL_EXTRA_DB_RANG_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_RESERVED_RW_MASK  (0x0000FE00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_RESERVED_RW_SHIFT (0x00000009U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_RESERVED_RW_MAX   (0x0000007FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_RESERVED_R_MASK   (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_RESERVED_R_SHIFT  (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG3_RESERVED_R_MAX    (0x0000FFFFU)

/* XECP_AUX_CTRL_REG1 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_FORCE_PM_STATE_MASK    (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_FORCE_PM_STATE_SHIFT   (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_FORCE_PM_STATE_MAX     (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_PM_STATE_MASK          (0x0000001EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_PM_STATE_SHIFT         (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_PM_STATE_MAX           (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_ENABLE_P2_ENTER_MASK   (0x00000020U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_ENABLE_P2_ENTER_SHIFT  (0x00000005U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_ENABLE_P2_ENTER_MAX    (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_P2_OVERWRITE_P1_EN_MASK (0x00000040U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_P2_OVERWRITE_P1_EN_SHIFT (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_P2_OVERWRITE_P1_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_IGNORE_AUX_PME_EN_MASK (0x00000080U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_IGNORE_AUX_PME_EN_SHIFT (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_IGNORE_AUX_PME_EN_MAX  (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_PHYSTATUS_FALL_TIMEOUT_EN_MASK (0x00000100U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_PHYSTATUS_FALL_TIMEOUT_EN_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_PHYSTATUS_FALL_TIMEOUT_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_CCLK_GATE_DISABLE_MASK (0x00000200U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_CCLK_GATE_DISABLE_SHIFT (0x00000009U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_CCLK_GATE_DISABLE_MAX  (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_NEW_OW_EN_MASK         (0x00000400U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_NEW_OW_EN_SHIFT        (0x0000000AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_NEW_OW_EN_MAX          (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_ISOLATION_EN_MASK      (0x00000800U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_ISOLATION_EN_SHIFT     (0x0000000BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_ISOLATION_EN_MAX       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_PME_STATUS_EN_MASK     (0x00001000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_PME_STATUS_EN_SHIFT    (0x0000000CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_PME_STATUS_EN_MAX      (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_ELECIDLE_MASK_EN_MASK  (0x00002000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_ELECIDLE_MASK_EN_SHIFT (0x0000000DU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_ELECIDLE_MASK_EN_MAX   (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_CFG_PIPE_RST_EN_MASK   (0x00004000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_CFG_PIPE_RST_EN_SHIFT  (0x0000000EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_CFG_PIPE_RST_EN_MAX    (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_CFG_RXDET_P3_EN_MASK   (0x00008000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_CFG_RXDET_P3_EN_SHIFT  (0x0000000FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_CFG_RXDET_P3_EN_MAX    (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_CFG_CLK_GATE_DIS_MASK  (0x00010000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_CFG_CLK_GATE_DIS_SHIFT (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_CFG_CLK_GATE_DIS_MAX   (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_CFG_USB_P2_EN_MASK     (0x00020000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_CFG_USB_P2_EN_SHIFT    (0x00000011U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_CFG_USB_P2_EN_MAX      (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_CFG_IOB_DRIVESTRENGTH_MASK (0x000C0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_CFG_IOB_DRIVESTRENGTH_SHIFT (0x00000012U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_CFG_IOB_DRIVESTRENGTH_MAX (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_CFG_PCIE_TXREG_PD_MASK (0x00100000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_CFG_PCIE_TXREG_PD_SHIFT (0x00000014U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_CFG_PCIE_TXREG_PD_MAX  (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_CLR_SAVE_FLAG_MASK     (0x00200000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_CLR_SAVE_FLAG_SHIFT    (0x00000015U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_CLR_SAVE_FLAG_MAX      (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_RESERVEDRW_MASK        (0x00400000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_RESERVEDRW_SHIFT       (0x00000016U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_RESERVEDRW_MAX         (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_SR_CMD_SAVE_EN_MASK    (0x00800000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_SR_CMD_SAVE_EN_SHIFT   (0x00000017U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_SR_CMD_SAVE_EN_MAX     (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_CLR_SSV_EN_MASK        (0x01000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_CLR_SSV_EN_SHIFT       (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_CLR_SSV_EN_MAX         (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_SET_SSV_EN_MASK        (0x02000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_SET_SSV_EN_SHIFT       (0x00000019U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_SET_SSV_EN_MAX         (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_POWERDOWN_P1_EN_MASK   (0x04000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_POWERDOWN_P1_EN_SHIFT  (0x0000001AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_POWERDOWN_P1_EN_MAX    (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_USE_PERST_4FD_RST_MASK (0x08000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_USE_PERST_4FD_RST_SHIFT (0x0000001BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_USE_PERST_4FD_RST_MAX  (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_DIRECT_RATE_PASS_EN_MASK (0x10000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_DIRECT_RATE_PASS_EN_SHIFT (0x0000001CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_DIRECT_RATE_PASS_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_EXTEND_PHYSTATUS_EN_MASK (0x20000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_EXTEND_PHYSTATUS_EN_SHIFT (0x0000001DU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_EXTEND_PHYSTATUS_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_LOW_PWR_CCLK_GATE_EN_MASK (0x40000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_LOW_PWR_CCLK_GATE_EN_SHIFT (0x0000001EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_LOW_PWR_CCLK_GATE_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_D3_HOT_PME_EN_MASK     (0x80000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_D3_HOT_PME_EN_SHIFT    (0x0000001FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG1_D3_HOT_PME_EN_MAX      (0x00000001U)

/* XECP_HOST_CTRL_WATERMARK_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_WATERMARK_REG_XBUF_WATER_MARK_MASK (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_WATERMARK_REG_XBUF_WATER_MARK_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_WATERMARK_REG_XBUF_WATER_MARK_MAX (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_WATERMARK_REG_RBUF_WATER_MARK_MASK (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_WATERMARK_REG_RBUF_WATER_MARK_SHIFT (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_WATERMARK_REG_RBUF_WATER_MARK_MAX (0x0000FFFFU)

/* XECP_HOST_CTRL_PORT_LINK_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_DIS_COMP_MASK (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_DIS_COMP_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_DIS_COMP_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_LPBK_MODE_MASK (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_LPBK_MODE_SHIFT (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_LPBK_MODE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_U1_ENABLE_MASK (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_U1_ENABLE_SHIFT (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_U1_ENABLE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_U2_ENABLE_MASK (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_U2_ENABLE_SHIFT (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_U2_ENABLE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_SYMBOL_ERR_EN_MASK (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_SYMBOL_ERR_EN_SHIFT (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_SYMBOL_ERR_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_DIS_SCRMB_MASK (0x00000020U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_DIS_SCRMB_SHIFT (0x00000005U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_DIS_SCRMB_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_FAST_TRAINING_MASK (0x00000040U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_FAST_TRAINING_SHIFT (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_FAST_TRAINING_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_RECOVERY_MASK (0x00000080U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_RECOVERY_SHIFT (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_RECOVERY_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_FORCE_PM_ACCEPT_MASK (0x00000100U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_FORCE_PM_ACCEPT_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_FORCE_PM_ACCEPT_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_U3_RECOV_VAL_MASK (0x00000E00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_U3_RECOV_VAL_SHIFT (0x00000009U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_U3_RECOV_VAL_MAX (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_NORM_RECOV_VAL_MASK (0x00007000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_NORM_RECOV_VAL_SHIFT (0x0000000CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_NORM_RECOV_VAL_MAX (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_LOWPOWER_LATENCY_MASK (0x00018000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_LOWPOWER_LATENCY_SHIFT (0x0000000FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_CFG_LOWPOWER_LATENCY_MAX (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_DBG_MODE_SEL_MASK (0x000E0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_DBG_MODE_SEL_SHIFT (0x00000011U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_DBG_MODE_SEL_MAX (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_LINK_ERR_CNT_SLV_EN_MASK (0x00100000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_LINK_ERR_CNT_SLV_EN_SHIFT (0x00000014U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_LINK_ERR_CNT_SLV_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_FORCE_COMP_PATTERN_MASK (0x01E00000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_FORCE_COMP_PATTERN_SHIFT (0x00000015U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_FORCE_COMP_PATTERN_MAX (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_FORCE_LTSSM_U0_MASK (0x02000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_FORCE_LTSSM_U0_SHIFT (0x00000019U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_FORCE_LTSSM_U0_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_FORCE_LTSSM_MASK (0x04000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_FORCE_LTSSM_SHIFT (0x0000001AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_FORCE_LTSSM_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_FORCE_LTSSM_STATE_MASK (0xF8000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_FORCE_LTSSM_STATE_SHIFT (0x0000001BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG_FORCE_LTSSM_STATE_MAX (0x0000001FU)

/* XECP_USB2_LINK_MGR_CTRL_REG1 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_QUICK_SIM_MASK (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_QUICK_SIM_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_QUICK_SIM_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_PHY_RST_MASK (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_PHY_RST_SHIFT (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_PHY_RST_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_PHY_RSTDISCON_MASK (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_PHY_RSTDISCON_SHIFT (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_PHY_RSTDISCON_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_PHY_CLKGATEDIS_MASK (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_PHY_CLKGATEDIS_SHIFT (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_PHY_CLKGATEDIS_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_PHY_SUSDISALL_MASK (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_PHY_SUSDISALL_SHIFT (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_PHY_SUSDISALL_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_AUTOPING_MASK (0x00000020U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_AUTOPING_SHIFT (0x00000005U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_AUTOPING_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_FORCEPING_MASK (0x00000040U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_FORCEPING_SHIFT (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_FORCEPING_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_DROPPING_MASK (0x00000080U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_DROPPING_SHIFT (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_DROPPING_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_DIRECT_RESUME_MASK (0x00000100U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_DIRECT_RESUME_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_DIRECT_RESUME_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_DIS_ISO_PEEK_MASK (0x00000200U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_DIS_ISO_PEEK_SHIFT (0x00000009U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_DIS_ISO_PEEK_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_DIS_PORT_ERR_MASK (0x00000400U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_DIS_PORT_ERR_SHIFT (0x0000000AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_DIS_PORT_ERR_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_ENABLE_DISC_WIN_MASK (0x00000800U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_ENABLE_DISC_WIN_SHIFT (0x0000000BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_ENABLE_DISC_WIN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_UTMIRST1_MASK (0x00001000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_UTMIRST1_SHIFT (0x0000000CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_UTMIRST1_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_UTMIRST2_MASK (0x00002000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_UTMIRST2_SHIFT (0x0000000DU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_UTMIRST2_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_FS_LS_EXT_DISCON_MASK (0x00004000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_FS_LS_EXT_DISCON_SHIFT (0x0000000EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_FS_LS_EXT_DISCON_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_SPLIT_192_LIMITDIS_MASK (0x00008000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_SPLIT_192_LIMITDIS_SHIFT (0x0000000FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_SPLIT_192_LIMITDIS_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_FORCE_FULL_SPEED_MASK (0x00010000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_FORCE_FULL_SPEED_SHIFT (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_FORCE_FULL_SPEED_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_EOP_DETECT_MASK (0x00020000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_EOP_DETECT_SHIFT (0x00000011U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_EOP_DETECT_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_ENABLE_FLUSH_TO_MASK (0x00040000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_ENABLE_FLUSH_TO_SHIFT (0x00000012U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_ENABLE_FLUSH_TO_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_HW_LPM_ERRATA1_MASK (0x00080000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_HW_LPM_ERRATA1_SHIFT (0x00000013U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_HW_LPM_ERRATA1_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_HW_LPM_ERRATA_MASK (0x00100000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_HW_LPM_ERRATA_SHIFT (0x00000014U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_HW_LPM_ERRATA_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_RESUME_DEB_DIS_MASK (0x00200000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_RESUME_DEB_DIS_SHIFT (0x00000015U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_RESUME_DEB_DIS_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_LATENCY_TOL_MSG_MASK (0x00400000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_LATENCY_TOL_MSG_SHIFT (0x00000016U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_LATENCY_TOL_MSG_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_MASK (0x00800000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_SHIFT (0x00000017U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_USB2_PM_DEBUG_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_TIMER_DISCONNECT_DETECT_LO_MASK (0xFF000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_TIMER_DISCONNECT_DETECT_LO_SHIFT (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG1_TIMER_DISCONNECT_DETECT_LO_MAX (0x000000FFU)

/* XECP_USB2_LINK_MGR_CTRL_REG2 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG2_TIMER_DISCONNECT_DETECT_HI_MASK (0x0000001FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG2_TIMER_DISCONNECT_DETECT_HI_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG2_TIMER_DISCONNECT_DETECT_HI_MAX (0x0000001FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG2_TIMER_CONNECT_DETECT_MASK (0x0003FFE0U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG2_TIMER_CONNECT_DETECT_SHIFT (0x00000005U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG2_TIMER_CONNECT_DETECT_MAX (0x00001FFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG2_TIMER_CHIRP_K_DETECT_MASK (0x7FFC0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG2_TIMER_CHIRP_K_DETECT_SHIFT (0x00000012U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG2_TIMER_CHIRP_K_DETECT_MAX (0x00001FFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG2_TIMER_RESET_0_MASK (0x80000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG2_TIMER_RESET_0_SHIFT (0x0000001FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG2_TIMER_RESET_0_MAX (0x00000001U)

/* XECP_USB2_LINK_MGR_CTRL_REG3 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG3_TIMER_RESET_MASK (0x00007FFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG3_TIMER_RESET_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG3_TIMER_RESET_MAX (0x00007FFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG3_TIMER_U3_SETTLE_MASK (0x0FFF8000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG3_TIMER_U3_SETTLE_SHIFT (0x0000000FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG3_TIMER_U3_SETTLE_MAX (0x00001FFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG3_TIMER_U2_SETTLE_MASK (0xF0000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG3_TIMER_U2_SETTLE_SHIFT (0x0000001CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG3_TIMER_U2_SETTLE_MAX (0x0000000FU)

/* XECP_USB2_LINK_MGR_CTRL_REG4 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG4_TIMER_U2_SETTLE_MASK (0x000001FFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG4_TIMER_U2_SETTLE_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG4_TIMER_U2_SETTLE_MAX (0x000001FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG4_TIMER_RESUME_U2_REFLECT_MASK (0x01FFFE00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG4_TIMER_RESUME_U2_REFLECT_SHIFT (0x00000009U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG4_TIMER_RESUME_U2_REFLECT_MAX (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG4_RESERVED_R_MASK (0xFE000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG4_RESERVED_R_SHIFT (0x00000019U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINK_MGR_CTRL_REG4_RESERVED_R_MAX (0x0000007FU)

/* XECP_HOST_CTRL_BW_CTRL_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_CTRL_REG_BW_CTRL_7_0_MASK (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_CTRL_REG_BW_CTRL_7_0_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_CTRL_REG_BW_CTRL_7_0_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_CTRL_REG_BW_SYS_FACTOR_MASK (0x00000F00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_CTRL_REG_BW_SYS_FACTOR_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_CTRL_REG_BW_SYS_FACTOR_MAX (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_CTRL_REG_BW_CTRL_15_12_MASK (0x0000F000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_CTRL_REG_BW_CTRL_15_12_SHIFT (0x0000000CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_CTRL_REG_BW_CTRL_15_12_MAX (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_CTRL_REG_RESERVED_MASK  (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_CTRL_REG_RESERVED_SHIFT (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_CTRL_REG_RESERVED_MAX   (0x0000FFFFU)

/* XECP_FPGA_REVISION_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_FPGA_REVISION_REG_FPGA_REVISION_REG_DEFAULT_MASK (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_FPGA_REVISION_REG_FPGA_REVISION_REG_DEFAULT_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_FPGA_REVISION_REG_FPGA_REVISION_REG_DEFAULT_MAX (0xFFFFFFFFU)

/* XECP_HOST_INTF_CTRL_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_INTF_CTRL_REG_HOST_ERR_MASK_MASK (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_INTF_CTRL_REG_HOST_ERR_MASK_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_INTF_CTRL_REG_HOST_ERR_MASK_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_INTF_CTRL_REG_HC_HALT_TIMEOUT_EN_MASK (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_INTF_CTRL_REG_HC_HALT_TIMEOUT_EN_SHIFT (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_INTF_CTRL_REG_HC_HALT_TIMEOUT_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_INTF_CTRL_REG_HOST_INTF_CTRL_MASK (0x0000003CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_INTF_CTRL_REG_HOST_INTF_CTRL_SHIFT (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_INTF_CTRL_REG_HOST_INTF_CTRL_MAX (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_INTF_CTRL_REG_CFG_MAX_NUM_OF_RD_MASK (0x000000C0U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_INTF_CTRL_REG_CFG_MAX_NUM_OF_RD_SHIFT (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_INTF_CTRL_REG_CFG_MAX_NUM_OF_RD_MAX (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_INTF_CTRL_REG_PROT_HDR_RBUF_OVERFLOW_CCLK_MASK (0x00000100U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_INTF_CTRL_REG_PROT_HDR_RBUF_OVERFLOW_CCLK_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_INTF_CTRL_REG_PROT_HDR_RBUF_OVERFLOW_CCLK_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_INTF_CTRL_REG_RESERVED_R_MASK   (0xFFFFFE00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_INTF_CTRL_REG_RESERVED_R_SHIFT  (0x00000009U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_INTF_CTRL_REG_RESERVED_R_MAX    (0x007FFFFFU)

/* XECP_BW_OV_SS_BURST_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_BW_OV_SS_BURST_REG_BW_OV_SS_BURST_MASK (0x00000FFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BW_OV_SS_BURST_REG_BW_OV_SS_BURST_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BW_OV_SS_BURST_REG_BW_OV_SS_BURST_MAX (0x00000FFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_BW_OV_SS_BURST_REG_BW_OV_SYS_BURST_MASK (0x00FFF000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BW_OV_SS_BURST_REG_BW_OV_SYS_BURST_SHIFT (0x0000000CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BW_OV_SS_BURST_REG_BW_OV_SYS_BURST_MAX (0x00000FFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_BW_OV_SS_BURST_REG_RESERVED_MASK     (0xFF000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BW_OV_SS_BURST_REG_RESERVED_SHIFT    (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BW_OV_SS_BURST_REG_RESERVED_MAX      (0x000000FFU)

/* XECP_HOST_CTRL_TRM_REG2 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_RESERVE_TRB_EN_MASK (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_RESERVE_TRB_EN_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_RESERVE_TRB_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_ISO_CNT_2NODMA_EN_MASK (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_ISO_CNT_2NODMA_EN_SHIFT (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_ISO_CNT_2NODMA_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_UPORT_CRD_UPD_EN_MASK (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_UPORT_CRD_UPD_EN_SHIFT (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_UPORT_CRD_UPD_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_CNTX_1ST_TD_EN_MASK (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_CNTX_1ST_TD_EN_SHIFT (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_CNTX_1ST_TD_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_USE_EMPTY_4TTE_OVERLAP_MASK (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_USE_EMPTY_4TTE_OVERLAP_SHIFT (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_USE_EMPTY_4TTE_OVERLAP_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_SETADDR_OVERRIDE_MASK (0x00000020U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_SETADDR_OVERRIDE_SHIFT (0x00000005U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_SETADDR_OVERRIDE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_FC_ON_2INCOMPLET_EN_MASK (0x00000040U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_FC_ON_2INCOMPLET_EN_SHIFT (0x00000006U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_FC_ON_2INCOMPLET_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_PKT_BNDRY_2IGNORE_NTRB_EN_MASK (0x00000080U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_PKT_BNDRY_2IGNORE_NTRB_EN_SHIFT (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_PKT_BNDRY_2IGNORE_NTRB_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_SUPPORT_0LEN_TTE_EN_MASK (0x00000100U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_SUPPORT_0LEN_TTE_EN_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_SUPPORT_0LEN_TTE_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_TRM_ADV_DETECT_EN_MASK (0x00000200U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_TRM_ADV_DETECT_EN_SHIFT (0x00000009U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_TRM_ADV_DETECT_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_STREAM_ID_MATCH_EN_MASK (0x00000400U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_STREAM_ID_MATCH_EN_SHIFT (0x0000000AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_STREAM_ID_MATCH_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_STREAM_IDLE2PRIME_EN_MASK (0x00000800U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_STREAM_IDLE2PRIME_EN_SHIFT (0x0000000BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_STREAM_IDLE2PRIME_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_DISABLE_CPL_SST_IDEQ0_ERR_MASK (0x00001000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_DISABLE_CPL_SST_IDEQ0_ERR_SHIFT (0x0000000CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_DISABLE_CPL_SST_IDEQ0_ERR_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_RPORT_CRD_CHECK_DIS_MASK (0x00002000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_RPORT_CRD_CHECK_DIS_SHIFT (0x0000000DU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_RPORT_CRD_CHECK_DIS_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_ODMA_CRD_CAL_EN_MASK (0x00004000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_ODMA_CRD_CAL_EN_SHIFT (0x0000000EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_ODMA_CRD_CAL_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_SKIP_INTR_4RESP_EN_MASK (0x00008000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_SKIP_INTR_4RESP_EN_SHIFT (0x0000000FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_SKIP_INTR_4RESP_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_MSI_CNT_EN_MASK   (0x00010000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_MSI_CNT_EN_SHIFT  (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_MSI_CNT_EN_MAX    (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_ENT_AT_END_OF_TD_EN_MASK (0x00020000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_ENT_AT_END_OF_TD_EN_SHIFT (0x00000011U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_ENT_AT_END_OF_TD_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_MULTI_NON0_CTRL_EP_EN_MASK (0x00040000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_MULTI_NON0_CTRL_EP_EN_SHIFT (0x00000012U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_MULTI_NON0_CTRL_EP_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_DEADLOCK_TRB_ERR_EN_MASK (0x00080000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_DEADLOCK_TRB_ERR_EN_SHIFT (0x00000013U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_DEADLOCK_TRB_ERR_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_DEV_MBS_CAP_EN_MASK (0x00100000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_DEV_MBS_CAP_EN_SHIFT (0x00000014U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_DEV_MBS_CAP_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_SECOND_EVENT_FOR_ISP_EN_MASK (0x00200000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_SECOND_EVENT_FOR_ISP_EN_SHIFT (0x00000015U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_SECOND_EVENT_FOR_ISP_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_STOP_2MS_4TTE_EN_MASK (0x00400000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_STOP_2MS_4TTE_EN_SHIFT (0x00000016U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_STOP_2MS_4TTE_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_DOUBLE_CLR_MASK_DIS_MASK (0x00800000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_DOUBLE_CLR_MASK_DIS_SHIFT (0x00000017U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_DOUBLE_CLR_MASK_DIS_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_IGNORE_NPKT_4PRDC_MASK (0x01000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_IGNORE_NPKT_4PRDC_SHIFT (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_IGNORE_NPKT_4PRDC_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_TTE_IN_EP_BLOCK_EN_MASK (0x02000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_TTE_IN_EP_BLOCK_EN_SHIFT (0x00000019U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_TTE_IN_EP_BLOCK_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_TTE_PKT_BNDRY_EXIT_MASK (0x04000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_TTE_PKT_BNDRY_EXIT_SHIFT (0x0000001AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_TTE_PKT_BNDRY_EXIT_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_TRM_ODMA_FIFO_DISABLE_MASK (0x08000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_TRM_ODMA_FIFO_DISABLE_SHIFT (0x0000001BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_TRM_ODMA_FIFO_DISABLE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_ERR_CPL_CODE_STORE_EN_MASK (0x10000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_ERR_CPL_CODE_STORE_EN_SHIFT (0x0000001CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_ERR_CPL_CODE_STORE_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_NON_DMA_IMD_EN_MASK (0x20000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_NON_DMA_IMD_EN_SHIFT (0x0000001DU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_NON_DMA_IMD_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_PKT_BNDRY_BLOCK_HALT_EN_MASK (0x40000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_PKT_BNDRY_BLOCK_HALT_EN_SHIFT (0x0000001EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_PKT_BNDRY_BLOCK_HALT_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_INSERT_CPL_IDMA_WAIT_EN_MASK (0x80000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_INSERT_CPL_IDMA_WAIT_EN_SHIFT (0x0000001FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TRM_REG2_INSERT_CPL_IDMA_WAIT_EN_MAX (0x00000001U)

/* XECP_HOST_CTRL_BW_MAX1_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_MAX1_REG_FSLS_MAX_BW_MASK (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_MAX1_REG_FSLS_MAX_BW_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_MAX1_REG_FSLS_MAX_BW_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_MAX1_REG_HS_MAX_BW_MASK (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_MAX1_REG_HS_MAX_BW_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_MAX1_REG_HS_MAX_BW_MAX  (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_MAX1_REG_SS_MAX_BW_MASK (0x00FF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_MAX1_REG_SS_MAX_BW_SHIFT (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_MAX1_REG_SS_MAX_BW_MAX  (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_MAX1_REG_FSLS_BHUB_MAX_BW_MASK (0xFF000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_MAX1_REG_FSLS_BHUB_MAX_BW_SHIFT (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_MAX1_REG_FSLS_BHUB_MAX_BW_MAX (0x000000FFU)

/* XECP_HOST_CTRL_BW_MAX2_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_MAX2_REG_BW_MAX_REG59_32_MASK (0x0FFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_MAX2_REG_BW_MAX_REG59_32_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_MAX2_REG_BW_MAX_REG59_32_MAX (0x0FFFFFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_MAX2_REG_RESERVED_R_MASK (0xF0000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_MAX2_REG_RESERVED_R_SHIFT (0x0000001CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_BW_MAX2_REG_RESERVED_R_MAX (0x0000000FU)

/* XECP_USB2_LINESTATE_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINESTATE_REG_UTMI_LINESTATE_MASK (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINESTATE_REG_UTMI_LINESTATE_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINESTATE_REG_UTMI_LINESTATE_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINESTATE_REG_PDOWN_STATUS_MASK (0x00FFFF00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINESTATE_REG_PDOWN_STATUS_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINESTATE_REG_PDOWN_STATUS_MAX  (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINESTATE_REG_RESERVED_R_MASK   (0xFF000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINESTATE_REG_RESERVED_R_SHIFT  (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB2_LINESTATE_REG_RESERVED_R_MAX    (0x000000FFU)

/* XECP_HOST_PROTO_GAP_TIMER1_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_GAP_TIMER1_REG_USB2_PROTO_PKT_GAP_HS_MASK (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_GAP_TIMER1_REG_USB2_PROTO_PKT_GAP_HS_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_GAP_TIMER1_REG_USB2_PROTO_PKT_GAP_HS_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_GAP_TIMER1_REG_USB2_PROTO_PKT_GAP_HS_SOF_MASK (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_GAP_TIMER1_REG_USB2_PROTO_PKT_GAP_HS_SOF_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_GAP_TIMER1_REG_USB2_PROTO_PKT_GAP_HS_SOF_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_GAP_TIMER1_REG_USB2_PROTO_PKT_GAP_HS_RX_MASK (0x00FF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_GAP_TIMER1_REG_USB2_PROTO_PKT_GAP_HS_RX_SHIFT (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_GAP_TIMER1_REG_USB2_PROTO_PKT_GAP_HS_RX_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_GAP_TIMER1_REG_USB2_PROTO_PKT_GAP_FS_MASK (0xFF000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_GAP_TIMER1_REG_USB2_PROTO_PKT_GAP_FS_SHIFT (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_GAP_TIMER1_REG_USB2_PROTO_PKT_GAP_FS_MAX (0x000000FFU)

/* XECP_HOST_PROTO_GAP_TIMER2_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_GAP_TIMER2_REG_USB2_PROTO_PKT_GAP_LS_MASK (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_GAP_TIMER2_REG_USB2_PROTO_PKT_GAP_LS_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_GAP_TIMER2_REG_USB2_PROTO_PKT_GAP_LS_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_GAP_TIMER2_REG_USB2_PROTO_PKT_GAP_LS_HRX_MASK (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_GAP_TIMER2_REG_USB2_PROTO_PKT_GAP_LS_HRX_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_GAP_TIMER2_REG_USB2_PROTO_PKT_GAP_LS_HRX_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_GAP_TIMER2_REG_USB2_PROTO_PKT_GAP_LS_HTX_MASK (0x00FF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_GAP_TIMER2_REG_USB2_PROTO_PKT_GAP_LS_HTX_SHIFT (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_GAP_TIMER2_REG_USB2_PROTO_PKT_GAP_LS_HTX_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_GAP_TIMER2_REG_RESERVED_R_MASK (0xFF000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_GAP_TIMER2_REG_RESERVED_R_SHIFT (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_GAP_TIMER2_REG_RESERVED_R_MAX (0x000000FFU)

/* XECP_HOST_PROTO_BTO_TIMER_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_BTO_TIMER_REG_USB2_PROTO_BTO_HS_MASK (0x000003FFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_BTO_TIMER_REG_USB2_PROTO_BTO_HS_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_BTO_TIMER_REG_USB2_PROTO_BTO_HS_MAX (0x000003FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_BTO_TIMER_REG_USB2_PROTO_BTO_FS_MASK (0x001FFC00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_BTO_TIMER_REG_USB2_PROTO_BTO_FS_SHIFT (0x0000000AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_BTO_TIMER_REG_USB2_PROTO_BTO_FS_MAX (0x000007FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_BTO_TIMER_REG_USB2_PROTO_BTO_LS_MASK (0xFFE00000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_BTO_TIMER_REG_USB2_PROTO_BTO_LS_SHIFT (0x00000015U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_PROTO_BTO_TIMER_REG_USB2_PROTO_BTO_LS_MAX (0x000007FFU)

/* XECP_HOST_CTRL_PSCH_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PSCH_REG_RESERVED_RW_MASK  (0x00FFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PSCH_REG_RESERVED_RW_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PSCH_REG_RESERVED_RW_MAX   (0x00FFFFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PSCH_REG_PSCH_HOST_CTRL_REG_I_31_24_MASK (0xFF000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PSCH_REG_PSCH_HOST_CTRL_REG_I_31_24_SHIFT (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PSCH_REG_PSCH_HOST_CTRL_REG_I_31_24_MAX (0x000000FFU)

/* XECP_HOST_CTRL_PSCH1_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PSCH1_REG_RESERVED_RW1_MASK (0x000003FFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PSCH1_REG_RESERVED_RW1_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PSCH1_REG_RESERVED_RW1_MAX (0x000003FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PSCH1_REG_IDLE_SCALE_MASK  (0x00000C00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PSCH1_REG_IDLE_SCALE_SHIFT (0x0000000AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PSCH1_REG_IDLE_SCALE_MAX   (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PSCH1_REG_RESERVED_RW2_MASK (0x0000F000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PSCH1_REG_RESERVED_RW2_SHIFT (0x0000000CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PSCH1_REG_RESERVED_RW2_MAX (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PSCH1_REG_RESERVED_R_MASK  (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PSCH1_REG_RESERVED_R_SHIFT (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PSCH1_REG_RESERVED_R_MAX   (0x0000FFFFU)

/* XECP_HOST_CTRL_LTM_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG_BELT_SELECTED_MASK (0x00000FFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG_BELT_SELECTED_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG_BELT_SELECTED_MAX  (0x00000FFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG_RESERVED_R_MASK    (0xFFFFF000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG_RESERVED_R_SHIFT   (0x0000000CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG_RESERVED_R_MAX     (0x000FFFFFU)

/* XECP_AUX_CTRL_REG2 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_AUTO_P2_OW_EN_REG_MASK (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_AUTO_P2_OW_EN_REG_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_AUTO_P2_OW_EN_REG_MAX  (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_FAST_SIMS_MASK     (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_FAST_SIMS_SHIFT    (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_FAST_SIMS_MAX      (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_P0_DRIVE_DIS_MASK      (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_P0_DRIVE_DIS_SHIFT     (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_P0_DRIVE_DIS_MAX       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_IDLE_WAKEUP_EN_MASK    (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_IDLE_WAKEUP_EN_SHIFT   (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_IDLE_WAKEUP_EN_MAX     (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_RESERVED_RW1_MASK      (0x000001F0U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_RESERVED_RW1_SHIFT     (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_RESERVED_RW1_MAX       (0x0000001FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_PSCEG_SEL_MASK     (0x00000200U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_PSCEG_SEL_SHIFT    (0x00000009U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_PSCEG_SEL_MAX      (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_PRDC_PREVENT_L1_EN_MASK (0x00000400U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_PRDC_PREVENT_L1_EN_SHIFT (0x0000000AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_PRDC_PREVENT_L1_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_RESERVED_RW2_MASK      (0x00001800U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_RESERVED_RW2_SHIFT     (0x0000000BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_RESERVED_RW2_MAX       (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_U2_P3_EN_MASK      (0x00002000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_U2_P3_EN_SHIFT     (0x0000000DU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_U2_P3_EN_MAX       (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_U2_P3_TIMEOUT_MASK (0x0000C000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_U2_P3_TIMEOUT_SHIFT (0x0000000EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_U2_P3_TIMEOUT_MAX  (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_WAKE_EXIT_USE_LVL_EN_MASK (0x00010000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_WAKE_EXIT_USE_LVL_EN_SHIFT (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_WAKE_EXIT_USE_LVL_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_U2_P3_LFPS_TIME_MASK (0x00020000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_U2_P3_LFPS_TIME_SHIFT (0x00000011U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_U2_P3_LFPS_TIME_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_U0_WAKE_TIMEOUT_EN_MASK (0x00040000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_U0_WAKE_TIMEOUT_EN_SHIFT (0x00000012U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_U0_WAKE_TIMEOUT_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_PDOWN_2LINK_RST_EN_REG_MASK (0x00080000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_PDOWN_2LINK_RST_EN_REG_SHIFT (0x00000013U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_PDOWN_2LINK_RST_EN_REG_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_U3_AUTO_MASK       (0x00100000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_U3_AUTO_SHIFT      (0x00000014U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_U3_AUTO_MAX        (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_P2_OVERWRITE_WHENL1_EN_MASK (0x00200000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_P2_OVERWRITE_WHENL1_EN_SHIFT (0x00000015U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_P2_OVERWRITE_WHENL1_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_LTSSM_IDLE2TS2_MASK (0x00400000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_LTSSM_IDLE2TS2_SHIFT (0x00000016U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_LTSSM_IDLE2TS2_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_PSCEG_DIS_SEL_MASK (0x00800000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_PSCEG_DIS_SEL_SHIFT (0x00000017U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_PSCEG_DIS_SEL_MAX  (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_AUTO_PM_EXIT_L1_EN_MASK (0x01000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_AUTO_PM_EXIT_L1_EN_SHIFT (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_AUTO_PM_EXIT_L1_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_DBGP_DIS_AUTO_MASK (0x02000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_DBGP_DIS_AUTO_SHIFT (0x00000019U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_DBGP_DIS_AUTO_MAX  (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_DEBOUNCE_EN_MASK   (0x04000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_DEBOUNCE_EN_SHIFT  (0x0000001AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_CFG_DEBOUNCE_EN_MAX    (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_BATT_CHARGE_D3_EN_MASK (0x08000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_BATT_CHARGE_D3_EN_SHIFT (0x0000001BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_BATT_CHARGE_D3_EN_MAX  (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_SHADOW_DECODE_DIS_MASK (0x10000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_SHADOW_DECODE_DIS_SHIFT (0x0000001CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_SHADOW_DECODE_DIS_MAX  (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_SNPS_PHYSTATUS_DONE_L1_DIS_MASK (0x20000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_SNPS_PHYSTATUS_DONE_L1_DIS_SHIFT (0x0000001DU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_SNPS_PHYSTATUS_DONE_L1_DIS_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_SNPS_PHYSTATUS_DONE_L23_DIS_MASK (0x40000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_SNPS_PHYSTATUS_DONE_L23_DIS_SHIFT (0x0000001EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_SNPS_PHYSTATUS_DONE_L23_DIS_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_UPORTS_CHANGE_DETECT_EN_MASK (0x80000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_UPORTS_CHANGE_DETECT_EN_SHIFT (0x0000001FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG2_UPORTS_CHANGE_DETECT_EN_MAX (0x00000001U)

/* XECP_AUX_CTRL_REG3 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3_PHY_MISC_CTRL_REG_1_0_MASK (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3_PHY_MISC_CTRL_REG_1_0_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3_PHY_MISC_CTRL_REG_1_0_MAX (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3_PHY_MISC_CTRL_REG_4_2_MASK (0x0000001CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3_PHY_MISC_CTRL_REG_4_2_SHIFT (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3_PHY_MISC_CTRL_REG_4_2_MAX (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3_STA_MASK               (0x000000E0U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3_STA_SHIFT              (0x00000005U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3_STA_MAX                (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3_TRANS_HS_CROSS_ADJ_MASK (0x00000300U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3_TRANS_HS_CROSS_ADJ_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3_TRANS_HS_CROSS_ADJ_MAX (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3_HS_TRANS_RS_TIME_ADJ_MASK (0x00000400U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3_HS_TRANS_RS_TIME_ADJ_SHIFT (0x0000000AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3_HS_TRANS_RS_TIME_ADJ_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3_HS_TRANS_PRE_EMPH_EN_MASK (0x00000800U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3_HS_TRANS_PRE_EMPH_EN_SHIFT (0x0000000BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3_HS_TRANS_PRE_EMPH_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3_FSLS_SRC_IMPD_ADJ_MASK (0x0000F000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3_FSLS_SRC_IMPD_ADJ_SHIFT (0x0000000CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3_FSLS_SRC_IMPD_ADJ_MAX  (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3_HS_DC_V_LVL_ADJ_MASK   (0x000F0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3_HS_DC_V_LVL_ADJ_SHIFT  (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3_HS_DC_V_LVL_ADJ_MAX    (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3_RESERVED_R_MASK        (0xFFF00000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3_RESERVED_R_SHIFT       (0x00000014U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_REG3_RESERVED_R_MAX         (0x00000FFFU)

/* XECP_DEBUG_CTRL_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_DEBUG_MODE_SEL_MASK   (0x0000001FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_DEBUG_MODE_SEL_SHIFT  (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_DEBUG_MODE_SEL_MAX    (0x0000001FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_DEBUG_FINE_MODE_SEL_MASK (0x00000060U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_DEBUG_FINE_MODE_SEL_SHIFT (0x00000005U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_DEBUG_FINE_MODE_SEL_MAX (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_DEBUG_EN_TOGGLE_MASK  (0x00000080U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_DEBUG_EN_TOGGLE_SHIFT (0x00000007U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_DEBUG_EN_TOGGLE_MAX   (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_PORT_PWR_CTRL_TOGGLE_MASK (0x00000100U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_PORT_PWR_CTRL_TOGGLE_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_PORT_PWR_CTRL_TOGGLE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_SW_EEPROM_EN_MASK     (0x00000200U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_SW_EEPROM_EN_SHIFT    (0x00000009U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_SW_EEPROM_EN_MAX      (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_RESERVED_RW_MASK      (0x00003C00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_RESERVED_RW_SHIFT     (0x0000000AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_RESERVED_RW_MAX       (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_CFG_RESUME_TIMER_EN_MASK (0x00004000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_CFG_RESUME_TIMER_EN_SHIFT (0x0000000EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_CFG_RESUME_TIMER_EN_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_CFG_RESUME_WAKE_DIS_MASK (0x00008000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_CFG_RESUME_WAKE_DIS_SHIFT (0x0000000FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_CFG_RESUME_WAKE_DIS_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_CFG_PM_DEBUG_CTRL_MASK (0x000F0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_CFG_PM_DEBUG_CTRL_SHIFT (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_CFG_PM_DEBUG_CTRL_MAX (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_NEW_NB_CTRL_REG_MASK  (0x01F00000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_NEW_NB_CTRL_REG_SHIFT (0x00000014U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_NEW_NB_CTRL_REG_MAX   (0x0000001FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_CFG_L1_L0S_CTRL_MASK  (0x1E000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_CFG_L1_L0S_CTRL_SHIFT (0x00000019U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_CFG_L1_L0S_CTRL_MAX   (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_CFG_PCIE_GASKET_CTRL_MASK (0xE0000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_CFG_PCIE_GASKET_CTRL_SHIFT (0x0000001DU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_DEBUG_CTRL_REG_CFG_PCIE_GASKET_CTRL_MAX (0x00000007U)

/* XECP_HOST_CTRL_SCH_REG2 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG2_SCH_PRDC_RETRY_USB2_DIS_MASK (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG2_SCH_PRDC_RETRY_USB2_DIS_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG2_SCH_PRDC_RETRY_USB2_DIS_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG2_SCH_POLL_RESERVATION_DIS_MASK (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG2_SCH_POLL_RESERVATION_DIS_SHIFT (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG2_SCH_POLL_RESERVATION_DIS_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG2_SCH_ALWAYS_RESERVE_DIS_MASK (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG2_SCH_ALWAYS_RESERVE_DIS_SHIFT (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG2_SCH_ALWAYS_RESERVE_DIS_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG2_SCH_POLL_DBRANG_DIS_MASK (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG2_SCH_POLL_DBRANG_DIS_SHIFT (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG2_SCH_POLL_DBRANG_DIS_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG2_RESERVED_RW0_MASK (0x000000F0U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG2_RESERVED_RW0_SHIFT (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG2_RESERVED_RW0_MAX  (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG2_SERVICE_TIME_WATERMARK_MASK (0x00007F00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG2_SERVICE_TIME_WATERMARK_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG2_SERVICE_TIME_WATERMARK_MAX (0x0000007FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG2_RESERVED_RW1_MASK (0x00FF8000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG2_RESERVED_RW1_SHIFT (0x0000000FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG2_RESERVED_RW1_MAX  (0x000001FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG2_RESERVED_R_MASK   (0xFF000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG2_RESERVED_R_SHIFT  (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_SCH_REG2_RESERVED_R_MAX    (0x000000FFU)

/* XECP_AUX_DEBUG_READ_ONLY */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_DEBUG_READ_ONLY_ALL_UPORTS_IN_U3NC_MASK (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_DEBUG_READ_ONLY_ALL_UPORTS_IN_U3NC_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_DEBUG_READ_ONLY_ALL_UPORTS_IN_U3NC_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_DEBUG_READ_ONLY_P2_OVERWRITE_ENTER_MASK (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_DEBUG_READ_ONLY_P2_OVERWRITE_ENTER_SHIFT (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_DEBUG_READ_ONLY_P2_OVERWRITE_ENTER_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_DEBUG_READ_ONLY_RESERVED_R_MASK  (0xFFFFFFFCU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_DEBUG_READ_ONLY_RESERVED_R_SHIFT (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_DEBUG_READ_ONLY_RESERVED_R_MAX   (0x3FFFFFFFU)

/* XECP_AUX_CTRL_PORTNUM_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_PORTNUM_REG_LIMIT_NPORTS_USB2_MASK (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_PORTNUM_REG_LIMIT_NPORTS_USB2_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_PORTNUM_REG_LIMIT_NPORTS_USB2_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_PORTNUM_REG_LIMIT_NPORTS_USB3_MASK (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_PORTNUM_REG_LIMIT_NPORTS_USB3_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_PORTNUM_REG_LIMIT_NPORTS_USB3_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_PORTNUM_REG_LIMIT_MAX_NPORTS_MASK (0x00FF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_PORTNUM_REG_LIMIT_MAX_NPORTS_SHIFT (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_PORTNUM_REG_LIMIT_MAX_NPORTS_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_PORTNUM_REG_LIMIT_MAX_SLOTS_MASK (0xFF000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_PORTNUM_REG_LIMIT_MAX_SLOTS_SHIFT (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_PORTNUM_REG_LIMIT_MAX_SLOTS_MAX (0x000000FFU)

/* XECP_AUX_CTRL_DEV_REMOVE_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_DEV_REMOVE_REG_DEVICE_REMOVABLE_USB2_MASK (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_DEV_REMOVE_REG_DEVICE_REMOVABLE_USB2_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_DEV_REMOVE_REG_DEVICE_REMOVABLE_USB2_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_DEV_REMOVE_REG_DEVICE_REMOVABLE_USB3_MASK (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_DEV_REMOVE_REG_DEVICE_REMOVABLE_USB3_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_DEV_REMOVE_REG_DEVICE_REMOVABLE_USB3_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_DEV_REMOVE_REG_RESERVED_R_MASK (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_DEV_REMOVE_REG_RESERVED_R_SHIFT (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_CTRL_DEV_REMOVE_REG_RESERVED_R_MAX (0x0000FFFFU)

/* XECP_HOST_CTRL_TTE_REG1 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TTE_REG1_EOB_ISO_DIS_MASK  (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TTE_REG1_EOB_ISO_DIS_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TTE_REG1_EOB_ISO_DIS_MAX   (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TTE_REG1_EOB_INT_DIS_MASK  (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TTE_REG1_EOB_INT_DIS_SHIFT (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TTE_REG1_EOB_INT_DIS_MAX   (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TTE_REG1_RESERVED_RW_MASK  (0x000000FCU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TTE_REG1_RESERVED_RW_SHIFT (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TTE_REG1_RESERVED_RW_MAX   (0x0000003FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TTE_REG1_RESERVED_R_MASK   (0xFFFFFF00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TTE_REG1_RESERVED_R_SHIFT  (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_TTE_REG1_RESERVED_R_MAX    (0x00FFFFFFU)

/* XECP_HOST_CTRL_LTM_REG1 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1_BELT_SLOT_SELECT_MASK (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1_BELT_SLOT_SELECT_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1_BELT_SLOT_SELECT_MAX (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1_BELT_PORT_SELECT_MASK (0x00FF0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1_BELT_PORT_SELECT_SHIFT (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1_BELT_PORT_SELECT_MAX (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1_BELT_FORCE_DISABLESLOT_MASK (0x01000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1_BELT_FORCE_DISABLESLOT_SHIFT (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1_BELT_FORCE_DISABLESLOT_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1_BELT_FORCE_DISABLEALL_MASK (0x02000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1_BELT_FORCE_DISABLEALL_SHIFT (0x00000019U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1_BELT_FORCE_DISABLEALL_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1_BELT_FORCE_RECOMPUTE_MASK (0x04000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1_BELT_FORCE_RECOMPUTE_SHIFT (0x0000001AU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1_BELT_FORCE_RECOMPUTE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1_BELT_PCIE_EN_MASK (0x08000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1_BELT_PCIE_EN_SHIFT (0x0000001BU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1_BELT_PCIE_EN_MAX  (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1_BELT_USB3_EN_MASK (0x10000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1_BELT_USB3_EN_SHIFT (0x0000001CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1_BELT_USB3_EN_MAX  (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1_BELT_USB2_EN_MASK (0x20000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1_BELT_USB2_EN_SHIFT (0x0000001DU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1_BELT_USB2_EN_MAX  (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1_BELT_SELECT_MASK  (0xC0000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1_BELT_SELECT_SHIFT (0x0000001EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG1_BELT_SELECT_MAX   (0x00000003U)

/* XECP_HOST_CTRL_LTM_REG2 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG2_DEFAULT_PCIE_LTM_MASK (0x00000FFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG2_DEFAULT_PCIE_LTM_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG2_DEFAULT_PCIE_LTM_MAX (0x00000FFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG2_RESERVED_R_MASK   (0xFFFFF000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG2_RESERVED_R_SHIFT  (0x0000000CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_LTM_REG2_RESERVED_R_MAX    (0x000FFFFFU)

/* XECP_AUX_SCRATCHPAD_0 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_SCRATCHPAD_0_RESERVED_RW_MASK    (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_SCRATCHPAD_0_RESERVED_RW_SHIFT   (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_SCRATCHPAD_0_RESERVED_RW_MAX     (0xFFFFFFFFU)

/* XECP_AUX_SCRATCHPAD_1 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_SCRATCHPAD_1_RESERVED_RW_MASK    (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_SCRATCHPAD_1_RESERVED_RW_SHIFT   (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_AUX_SCRATCHPAD_1_RESERVED_RW_MAX     (0xFFFFFFFFU)

/* XECP_BATTERY_CHARGE_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG_BATTERY_CHARGE_MODE_REG_MASK (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG_BATTERY_CHARGE_MODE_REG_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG_BATTERY_CHARGE_MODE_REG_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG_RESERVEDR_MASK    (0x01FFFFFEU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG_RESERVEDR_SHIFT   (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG_RESERVEDR_MAX     (0x00FFFFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG_BATTERY_CHARGE_MODE_EN_REG_MASK (0xFE000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG_BATTERY_CHARGE_MODE_EN_REG_SHIFT (0x00000019U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG_BATTERY_CHARGE_MODE_EN_REG_MAX (0x0000007FU)

/* XECP_BATTERY_CHARGE_REG1 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG1_BATTERY_CHARGE_CTRL_REG1_DEFAULT_MASK (0x7FFFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG1_BATTERY_CHARGE_CTRL_REG1_DEFAULT_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG1_BATTERY_CHARGE_CTRL_REG1_DEFAULT_MAX (0x7FFFFFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG1_RESERVED_R_MASK  (0x80000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG1_RESERVED_R_SHIFT (0x0000001FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG1_RESERVED_R_MAX   (0x00000001U)

/* XECP_BATTERY_CHARGE_REG2 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG2_BATTERY_CHARGE_CTRL_REG2_DEFAULT_MASK (0x00FFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG2_BATTERY_CHARGE_CTRL_REG2_DEFAULT_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG2_BATTERY_CHARGE_CTRL_REG2_DEFAULT_MAX (0x00FFFFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG2_RESERVED_R_MASK  (0xFF000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG2_RESERVED_R_SHIFT (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG2_RESERVED_R_MAX   (0x000000FFU)

/* XECP_BATTERY_CHARGE_REG3 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG3_BATTERY_CHARGE_DEBUG_MASK (0x00FFFFFFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG3_BATTERY_CHARGE_DEBUG_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG3_BATTERY_CHARGE_DEBUG_MAX (0x00FFFFFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG3_RESERVED_R_MASK  (0xFF000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG3_RESERVED_R_SHIFT (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_BATTERY_CHARGE_REG3_RESERVED_R_MAX   (0x000000FFU)

/* XECP_HOST_CTRL_PORT_LINK_REG1 */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG1_LINK_8B_DEBUG_CTRL_MASK (0x00000003U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG1_LINK_8B_DEBUG_CTRL_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG1_LINK_8B_DEBUG_CTRL_MAX (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG1_LINK_LTSSM_DEBUG_CTRL_MASK (0x000000FCU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG1_LINK_LTSSM_DEBUG_CTRL_SHIFT (0x00000002U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG1_LINK_LTSSM_DEBUG_CTRL_MAX (0x0000003FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG1_CFG_TIEBREAK_VAL_MASK (0x00000F00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG1_CFG_TIEBREAK_VAL_SHIFT (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG1_CFG_TIEBREAK_VAL_MAX (0x0000000FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG1_CFG_TIEBREAK_MODE_MASK (0x00003000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG1_CFG_TIEBREAK_MODE_SHIFT (0x0000000CU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG1_CFG_TIEBREAK_MODE_MAX (0x00000003U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG1_RESERVED_RW_MASK (0xFFFFC000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG1_RESERVED_RW_SHIFT (0x0000000EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_HOST_CTRL_PORT_LINK_REG1_RESERVED_RW_MAX (0x0003FFFFU)

/* XECP_USBLEGSUP */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGSUP_CID_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGSUP_CID_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGSUP_CID_MAX                    (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGSUP_NEXTCP_MASK                (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGSUP_NEXTCP_SHIFT               (0x00000008U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGSUP_NEXTCP_MAX                 (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGSUP_HCBIOSOS_MASK              (0x00010000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGSUP_HCBIOSOS_SHIFT             (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGSUP_HCBIOSOS_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGSUP_RSVD1_MASK                 (0x00FE0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGSUP_RSVD1_SHIFT                (0x00000011U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGSUP_RSVD1_MAX                  (0x0000007FU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGSUP_HCOSOS_MASK                (0x01000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGSUP_HCOSOS_SHIFT               (0x00000018U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGSUP_HCOSOS_MAX                 (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGSUP_RSVD2_MASK                 (0xFE000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGSUP_RSVD2_SHIFT                (0x00000019U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGSUP_RSVD2_MAX                  (0x0000007FU)

/* XECP_USBLEGCTLSTS */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_USBSMIE_MASK            (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_USBSMIE_SHIFT           (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_USBSMIE_MAX             (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_RSVD1_MASK              (0x0000000EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_RSVD1_SHIFT             (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_RSVD1_MAX               (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_SMIHSEE_MASK            (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_SMIHSEE_SHIFT           (0x00000004U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_SMIHSEE_MAX             (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_RSVD2_MASK              (0x00001FE0U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_RSVD2_SHIFT             (0x00000005U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_RSVD2_MAX               (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_SMIOSOE_MASK            (0x00002000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_SMIOSOE_SHIFT           (0x0000000DU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_SMIOSOE_MAX             (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_SMIPCICE_MASK           (0x00004000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_SMIPCICE_SHIFT          (0x0000000EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_SMIPCICE_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_SMIBARE_MASK            (0x00008000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_SMIBARE_SHIFT           (0x0000000FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_SMIBARE_MAX             (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_SMIEI_MASK              (0x00010000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_SMIEI_SHIFT             (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_SMIEI_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_RSVD3_MASK              (0x000E0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_RSVD3_SHIFT             (0x00000011U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_RSVD3_MAX               (0x00000007U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_SMIHSE_MASK             (0x00100000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_SMIHSE_SHIFT            (0x00000014U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_SMIHSE_MAX              (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_RSVD4_MASK              (0x1FE00000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_RSVD4_SHIFT             (0x00000015U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_RSVD4_MAX               (0x000000FFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_SMIOSOC_MASK            (0x20000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_SMIOSOC_SHIFT           (0x0000001DU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_SMIOSOC_MAX             (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_SMIPCIC_MASK            (0x40000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_SMIPCIC_SHIFT           (0x0000001EU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_SMIPCIC_MAX             (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_SMIBAR_MASK             (0x80000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_SMIBAR_SHIFT            (0x0000001FU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USBLEGCTLSTS_SMIBAR_MAX              (0x00000001U)

/* XECP_USB3_TEST_PORT0_REG */

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB3_TEST_PORT0_REG_USB3_TEST_CTRL_MASK (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB3_TEST_PORT0_REG_USB3_TEST_CTRL_SHIFT (0x00000000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB3_TEST_PORT0_REG_USB3_TEST_CTRL_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB3_TEST_PORT0_REG_USB3_TEST_LOOP_NUM_MASK (0x0000FFFEU)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB3_TEST_PORT0_REG_USB3_TEST_LOOP_NUM_SHIFT (0x00000001U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB3_TEST_PORT0_REG_USB3_TEST_LOOP_NUM_MAX (0x00007FFFU)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB3_TEST_PORT0_REG_LINK_TEST_DONE_MASK (0x00010000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB3_TEST_PORT0_REG_LINK_TEST_DONE_SHIFT (0x00000010U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB3_TEST_PORT0_REG_LINK_TEST_DONE_MAX (0x00000001U)

#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB3_TEST_PORT0_REG_LINK_TEST_LOOP_PASS_MASK (0xFFFE0000U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB3_TEST_PORT0_REG_LINK_TEST_LOOP_PASS_SHIFT (0x00000011U)
#define CSL_USB3P0SS_CTRL_XHCI_XECP_USB3_TEST_PORT0_REG_LINK_TEST_LOOP_PASS_MAX (0x00007FFFU)

/* USB_CONF */

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_CFGRST_MASK                       (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_CFGRST_SHIFT                      (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_CFGRST_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_CFGSET_MASK                       (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_CFGSET_SHIFT                      (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_CFGSET_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_RESERVED0_MASK                    (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_RESERVED0_SHIFT                   (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_RESERVED0_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_USB3DIS_MASK                      (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_USB3DIS_SHIFT                     (0x00000003U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_USB3DIS_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_USB2DIS_MASK                      (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_USB2DIS_SHIFT                     (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_USB2DIS_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_LENDIAN_MASK                      (0x00000020U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_LENDIAN_SHIFT                     (0x00000005U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_LENDIAN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_BENDIAN_MASK                      (0x00000040U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_BENDIAN_SHIFT                     (0x00000006U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_BENDIAN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_SWRST_MASK                        (0x00000080U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_SWRST_SHIFT                       (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_SWRST_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_RESERVED1_MASK                    (0x00000100U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_RESERVED1_SHIFT                   (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_RESERVED1_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_RESERVED2_MASK                    (0x00000200U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_RESERVED2_SHIFT                   (0x00000009U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_RESERVED2_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_DMAOFFEN_MASK                     (0x00000400U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_DMAOFFEN_SHIFT                    (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_DMAOFFEN_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_DMAOFFDS_MASK                     (0x00000800U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_DMAOFFDS_SHIFT                    (0x0000000BU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_DMAOFFDS_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_CFORCE_FS_MASK                    (0x00001000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_CFORCE_FS_SHIFT                   (0x0000000CU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_CFORCE_FS_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_SFORCE_FS_MASK                    (0x00002000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_SFORCE_FS_SHIFT                   (0x0000000DU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_SFORCE_FS_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_DEVEN_MASK                        (0x00004000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_DEVEN_SHIFT                       (0x0000000EU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_DEVEN_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_DEVDS_MASK                        (0x00008000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_DEVDS_SHIFT                       (0x0000000FU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_DEVDS_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_L1EN_MASK                         (0x00010000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_L1EN_SHIFT                        (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_L1EN_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_L1DS_MASK                         (0x00020000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_L1DS_SHIFT                        (0x00000011U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_L1DS_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_CLK2OFFEN_MASK                    (0x00040000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_CLK2OFFEN_SHIFT                   (0x00000012U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_CLK2OFFEN_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_CLK2OFFDS_MASK                    (0x00080000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_CLK2OFFDS_SHIFT                   (0x00000013U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_CLK2OFFDS_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_LGO_L0_MASK                       (0x00100000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_LGO_L0_SHIFT                      (0x00000014U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_LGO_L0_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_CLK3OFFEN_MASK                    (0x00200000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_CLK3OFFEN_SHIFT                   (0x00000015U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_CLK3OFFEN_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_CLK3OFFDS_MASK                    (0x00400000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_CLK3OFFDS_SHIFT                   (0x00000016U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_CLK3OFFDS_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_RESERVED3_MASK                    (0x00800000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_RESERVED3_SHIFT                   (0x00000017U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_RESERVED3_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_U1EN_MASK                         (0x01000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_U1EN_SHIFT                        (0x00000018U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_U1EN_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_U1DS_MASK                         (0x02000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_U1DS_SHIFT                        (0x00000019U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_U1DS_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_U2EN_MASK                         (0x04000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_U2EN_SHIFT                        (0x0000001AU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_U2EN_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_U2DS_MASK                         (0x08000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_U2DS_SHIFT                        (0x0000001BU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_U2DS_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_LGO_U0_MASK                       (0x10000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_LGO_U0_SHIFT                      (0x0000001CU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_LGO_U0_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_LGO_U1_MASK                       (0x20000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_LGO_U1_SHIFT                      (0x0000001DU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_LGO_U1_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_LGO_U2_MASK                       (0x40000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_LGO_U2_SHIFT                      (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_LGO_U2_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_LGO_SSINACT_MASK                  (0x80000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_LGO_SSINACT_SHIFT                 (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF_LGO_SSINACT_MAX                   (0x00000001U)

/* USB_STS */

#define CSL_USB3P0SS_CTRL_DEV_USB_STS_CFGSTS_MASK                        (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_CFGSTS_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_CFGSTS_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_STS_MEM_OV_MASK                        (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_MEM_OV_SHIFT                       (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_MEM_OV_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_STS_USB3CONS_MASK                      (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_USB3CONS_SHIFT                     (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_USB3CONS_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_STS_RESERVED0_MASK                     (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_RESERVED0_SHIFT                    (0x00000003U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_RESERVED0_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_STS_USBSPEED_MASK                      (0x00000070U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_USBSPEED_SHIFT                     (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_USBSPEED_MAX                       (0x00000007U)

#define CSL_USB3P0SS_CTRL_DEV_USB_STS_ENDIAN_MIRROR_MASK                 (0x00000080U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_ENDIAN_MIRROR_SHIFT                (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_ENDIAN_MIRROR_MAX                  (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_STS_CLK2OFF_MASK                       (0x00000100U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_CLK2OFF_SHIFT                      (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_CLK2OFF_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_STS_CLK3OFF_MASK                       (0x00000200U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_CLK3OFF_SHIFT                      (0x00000009U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_CLK3OFF_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_STS_IN_RST_MASK                        (0x00000400U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_IN_RST_SHIFT                       (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_IN_RST_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_STS_RESERVED1_MASK                     (0x00003800U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_RESERVED1_SHIFT                    (0x0000000BU)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_RESERVED1_MAX                      (0x00000007U)

#define CSL_USB3P0SS_CTRL_DEV_USB_STS_DEVS_MASK                          (0x00004000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_DEVS_SHIFT                         (0x0000000EU)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_DEVS_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_STS_ADDRESSED_MASK                     (0x00008000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_ADDRESSED_SHIFT                    (0x0000000FU)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_ADDRESSED_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_STS_L1ENS_MASK                         (0x00010000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_L1ENS_SHIFT                        (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_L1ENS_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_STS_VBUSS_MASK                         (0x00020000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_VBUSS_SHIFT                        (0x00000011U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_VBUSS_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_STS_LPMST_MASK                         (0x000C0000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_LPMST_SHIFT                        (0x00000012U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_LPMST_MAX                          (0x00000003U)

#define CSL_USB3P0SS_CTRL_DEV_USB_STS_USB2CONS_MASK                      (0x00100000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_USB2CONS_SHIFT                     (0x00000014U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_USB2CONS_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_STS_DISABLE_HS_MASK                    (0x00200000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_DISABLE_HS_SHIFT                   (0x00000015U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_DISABLE_HS_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_STS_RESERVED2_MASK                     (0x00C00000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_RESERVED2_SHIFT                    (0x00000016U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_RESERVED2_MAX                      (0x00000003U)

#define CSL_USB3P0SS_CTRL_DEV_USB_STS_U1ENS_MASK                         (0x01000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_U1ENS_SHIFT                        (0x00000018U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_U1ENS_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_STS_U2ENS_MASK                         (0x02000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_U2ENS_SHIFT                        (0x00000019U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_U2ENS_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_STS_LST_MASK                           (0x3C000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_LST_SHIFT                          (0x0000001AU)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_LST_MAX                            (0x0000000FU)

#define CSL_USB3P0SS_CTRL_DEV_USB_STS_DMAOFF_MASK                        (0x40000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_DMAOFF_SHIFT                       (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_DMAOFF_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_STS_ENDIAN_MASK                        (0x80000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_ENDIAN_SHIFT                       (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DEV_USB_STS_ENDIAN_MAX                         (0x00000001U)

/* USB_CMD */

#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_SET_ADDR_MASK                      (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_SET_ADDR_SHIFT                     (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_SET_ADDR_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_FADDR_MASK                         (0x000000FEU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_FADDR_SHIFT                        (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_FADDR_MAX                          (0x0000007FU)

#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_SDNFW_MASK                         (0x00000100U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_SDNFW_SHIFT                        (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_SDNFW_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_STMODE_MASK                        (0x00000200U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_STMODE_SHIFT                       (0x00000009U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_STMODE_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_TMODE_SEL_MASK                     (0x00000C00U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_TMODE_SEL_SHIFT                    (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_TMODE_SEL_MAX                      (0x00000003U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_SDNLTM_MASK                        (0x00001000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_SDNLTM_SHIFT                       (0x0000000CU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_SDNLTM_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_SPKT_MASK                          (0x00002000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_SPKT_SHIFT                         (0x0000000DU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_SPKT_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_RESERVED0_MASK                     (0x0000C000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_RESERVED0_SHIFT                    (0x0000000EU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_RESERVED0_MAX                      (0x00000003U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_DNLTM_BELT_7_0_MASK                (0x00FF0000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_DNLTM_BELT_7_0_SHIFT               (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_DNLTM_BELT_7_0_MAX                 (0x000000FFU)

#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_DNLTM_BELT_11_8_MASK               (0x0F000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_DNLTM_BELT_11_8_SHIFT              (0x00000018U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_DNLTM_BELT_11_8_MAX                (0x0000000FU)

#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_RESERVED1_MASK                     (0xF0000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_RESERVED1_SHIFT                    (0x0000001CU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CMD_RESERVED1_MAX                      (0x0000000FU)

/* USB_IPTN */

#define CSL_USB3P0SS_CTRL_DEV_USB_IPTN_ITPN_MASK                         (0x00003FFFU)
#define CSL_USB3P0SS_CTRL_DEV_USB_IPTN_ITPN_SHIFT                        (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IPTN_ITPN_MAX                          (0x00003FFFU)

#define CSL_USB3P0SS_CTRL_DEV_USB_IPTN_RESERVED_MASK                     (0xFFFFC000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IPTN_RESERVED_SHIFT                    (0x0000000EU)
#define CSL_USB3P0SS_CTRL_DEV_USB_IPTN_RESERVED_MAX                      (0x0003FFFFU)

/* USB_LPM */

#define CSL_USB3P0SS_CTRL_DEV_USB_LPM_HIRD_MASK                          (0x0000000FU)
#define CSL_USB3P0SS_CTRL_DEV_USB_LPM_HIRD_SHIFT                         (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_LPM_HIRD_MAX                           (0x0000000FU)

#define CSL_USB3P0SS_CTRL_DEV_USB_LPM_BRW_MASK                           (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_USB_LPM_BRW_SHIFT                          (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_USB_LPM_BRW_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_LPM_RESERVED_MASK                      (0xFFFFFFE0U)
#define CSL_USB3P0SS_CTRL_DEV_USB_LPM_RESERVED_SHIFT                     (0x00000005U)
#define CSL_USB3P0SS_CTRL_DEV_USB_LPM_RESERVED_MAX                       (0x07FFFFFFU)

/* USB_IEN */

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_CONIEN_MASK                        (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_CONIEN_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_CONIEN_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_DISIEN_MASK                        (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_DISIEN_SHIFT                       (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_DISIEN_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_UWRESIEN_MASK                      (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_UWRESIEN_SHIFT                     (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_UWRESIEN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_UHRESIEN_MASK                      (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_UHRESIEN_SHIFT                     (0x00000003U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_UHRESIEN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_U3ENTIEN_MASK                      (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_U3ENTIEN_SHIFT                     (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_U3ENTIEN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_U3EXTIEN_MASK                      (0x00000020U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_U3EXTIEN_SHIFT                     (0x00000005U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_U3EXTIEN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_U2ENTIEN_MASK                      (0x00000040U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_U2ENTIEN_SHIFT                     (0x00000006U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_U2ENTIEN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_U2EXTIEN_MASK                      (0x00000080U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_U2EXTIEN_SHIFT                     (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_U2EXTIEN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_U1ENTIEN_MASK                      (0x00000100U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_U1ENTIEN_SHIFT                     (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_U1ENTIEN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_U1EXTIEN_MASK                      (0x00000200U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_U1EXTIEN_SHIFT                     (0x00000009U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_U1EXTIEN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_ITPIEN_MASK                        (0x00000400U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_ITPIEN_SHIFT                       (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_ITPIEN_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_WAKEIEN_MASK                       (0x00000800U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_WAKEIEN_SHIFT                      (0x0000000BU)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_WAKEIEN_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_SPKTIEN_MASK                       (0x00001000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_SPKTIEN_SHIFT                      (0x0000000CU)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_SPKTIEN_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_RESERVED0_MASK                     (0x0000E000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_RESERVED0_SHIFT                    (0x0000000DU)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_RESERVED0_MAX                      (0x00000007U)

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_CON2IEN_MASK                       (0x00010000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_CON2IEN_SHIFT                      (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_CON2IEN_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_DIS2IEN_MASK                       (0x00020000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_DIS2IEN_SHIFT                      (0x00000011U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_DIS2IEN_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_U2RESIEN_MASK                      (0x00040000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_U2RESIEN_SHIFT                     (0x00000012U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_U2RESIEN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_RESERVED1_MASK                     (0x00080000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_RESERVED1_SHIFT                    (0x00000013U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_RESERVED1_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_L2ENTIEN_MASK                      (0x00100000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_L2ENTIEN_SHIFT                     (0x00000014U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_L2ENTIEN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_L2EXTIEN_MASK                      (0x00200000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_L2EXTIEN_SHIFT                     (0x00000015U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_L2EXTIEN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_RESERVED2_MASK                     (0x00C00000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_RESERVED2_SHIFT                    (0x00000016U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_RESERVED2_MAX                      (0x00000003U)

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_L1ENTIEN_MASK                      (0x01000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_L1ENTIEN_SHIFT                     (0x00000018U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_L1ENTIEN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_L1EXTIEN_MASK                      (0x02000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_L1EXTIEN_SHIFT                     (0x00000019U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_L1EXTIEN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_CFGRESIEN_MASK                     (0x04000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_CFGRESIEN_SHIFT                    (0x0000001AU)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_CFGRESIEN_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_RESERVED3_MASK                     (0x08000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_RESERVED3_SHIFT                    (0x0000001BU)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_RESERVED3_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_UWRESSIEN_MASK                     (0x10000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_UWRESSIEN_SHIFT                    (0x0000001CU)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_UWRESSIEN_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_UWRESEIEN_MASK                     (0x20000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_UWRESEIEN_SHIFT                    (0x0000001DU)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_UWRESEIEN_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_RESERVED4_MASK                     (0xC0000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_RESERVED4_SHIFT                    (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_USB_IEN_RESERVED4_MAX                      (0x00000003U)

/* USB_ISTS */

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_CONI_MASK                         (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_CONI_SHIFT                        (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_CONI_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_DISI_MASK                         (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_DISI_SHIFT                        (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_DISI_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_UWRESI_MASK                       (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_UWRESI_SHIFT                      (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_UWRESI_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_UHRESI_MASK                       (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_UHRESI_SHIFT                      (0x00000003U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_UHRESI_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_U3ENTI_MASK                       (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_U3ENTI_SHIFT                      (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_U3ENTI_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_U3EXTI_MASK                       (0x00000020U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_U3EXTI_SHIFT                      (0x00000005U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_U3EXTI_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_U2ENTI_MASK                       (0x00000040U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_U2ENTI_SHIFT                      (0x00000006U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_U2ENTI_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_U2EXTI_MASK                       (0x00000080U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_U2EXTI_SHIFT                      (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_U2EXTI_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_U1ENTI_MASK                       (0x00000100U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_U1ENTI_SHIFT                      (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_U1ENTI_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_U1EXTI_MASK                       (0x00000200U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_U1EXTI_SHIFT                      (0x00000009U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_U1EXTI_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_ITPI_MASK                         (0x00000400U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_ITPI_SHIFT                        (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_ITPI_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_WAKEI_MASK                        (0x00000800U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_WAKEI_SHIFT                       (0x0000000BU)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_WAKEI_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_SPKTI_MASK                        (0x00001000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_SPKTI_SHIFT                       (0x0000000CU)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_SPKTI_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_RESERVED0_MASK                    (0x0000E000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_RESERVED0_SHIFT                   (0x0000000DU)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_RESERVED0_MAX                     (0x00000007U)

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_CON2I_MASK                        (0x00010000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_CON2I_SHIFT                       (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_CON2I_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_DIS2I_MASK                        (0x00020000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_DIS2I_SHIFT                       (0x00000011U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_DIS2I_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_U2RESI_MASK                       (0x00040000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_U2RESI_SHIFT                      (0x00000012U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_U2RESI_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_RESERVED1_MASK                    (0x00080000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_RESERVED1_SHIFT                   (0x00000013U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_RESERVED1_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_L2ENTI_MASK                       (0x00100000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_L2ENTI_SHIFT                      (0x00000014U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_L2ENTI_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_L2EXTI_MASK                       (0x00200000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_L2EXTI_SHIFT                      (0x00000015U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_L2EXTI_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_RESERVED2_MASK                    (0x00C00000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_RESERVED2_SHIFT                   (0x00000016U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_RESERVED2_MAX                     (0x00000003U)

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_L1ENTI_MASK                       (0x01000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_L1ENTI_SHIFT                      (0x00000018U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_L1ENTI_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_L1EXTI_MASK                       (0x02000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_L1EXTI_SHIFT                      (0x00000019U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_L1EXTI_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_CFGRESI_MASK                      (0x04000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_CFGRESI_SHIFT                     (0x0000001AU)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_CFGRESI_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_RESERVED3_MASK                    (0x08000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_RESERVED3_SHIFT                   (0x0000001BU)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_RESERVED3_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_UWRESSI_MASK                      (0x10000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_UWRESSI_SHIFT                     (0x0000001CU)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_UWRESSI_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_UWRESEI_MASK                      (0x20000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_UWRESEI_SHIFT                     (0x0000001DU)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_UWRESEI_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_RESERVED4_MASK                    (0xC0000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_RESERVED4_SHIFT                   (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_USB_ISTS_RESERVED4_MAX                     (0x00000003U)

/* EP_SEL */

#define CSL_USB3P0SS_CTRL_DEV_EP_SEL_EPNO_MASK                           (0x0000000FU)
#define CSL_USB3P0SS_CTRL_DEV_EP_SEL_EPNO_SHIFT                          (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_SEL_EPNO_MAX                            (0x0000000FU)

#define CSL_USB3P0SS_CTRL_DEV_EP_SEL_RESERVED0_MASK                      (0x00000070U)
#define CSL_USB3P0SS_CTRL_DEV_EP_SEL_RESERVED0_SHIFT                     (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_EP_SEL_RESERVED0_MAX                       (0x00000007U)

#define CSL_USB3P0SS_CTRL_DEV_EP_SEL_DIR_MASK                            (0x00000080U)
#define CSL_USB3P0SS_CTRL_DEV_EP_SEL_DIR_SHIFT                           (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_EP_SEL_DIR_MAX                             (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_SEL_RESERVED1_MASK                      (0xFFFFFF00U)
#define CSL_USB3P0SS_CTRL_DEV_EP_SEL_RESERVED1_SHIFT                     (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_EP_SEL_RESERVED1_MAX                       (0x00FFFFFFU)

/* EP_TRADDR */

#define CSL_USB3P0SS_CTRL_DEV_EP_TRADDR_TRADDR_MASK                      (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_DEV_EP_TRADDR_TRADDR_SHIFT                     (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_TRADDR_TRADDR_MAX                       (0xFFFFFFFFU)

/* EP_CFG */

#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_ENABLE_MASK                         (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_ENABLE_SHIFT                        (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_ENABLE_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_EPTYPE_MASK                         (0x00000006U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_EPTYPE_SHIFT                        (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_EPTYPE_MAX                          (0x00000003U)

#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_STREAM_EN_MASK                      (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_STREAM_EN_SHIFT                     (0x00000003U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_STREAM_EN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_TDL_CHK_MASK                        (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_TDL_CHK_SHIFT                       (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_TDL_CHK_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_SID_CHK_MASK                        (0x00000020U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_SID_CHK_SHIFT                       (0x00000005U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_SID_CHK_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_RESERVED0_MASK                      (0x00000040U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_RESERVED0_SHIFT                     (0x00000006U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_RESERVED0_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_EPENDIAN_MASK                       (0x00000080U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_EPENDIAN_SHIFT                      (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_EPENDIAN_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_MAXBURST_MASK                       (0x00000F00U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_MAXBURST_SHIFT                      (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_MAXBURST_MAX                        (0x0000000FU)

#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_RESERVED1_MASK                      (0x00003000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_RESERVED1_SHIFT                     (0x0000000CU)
#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_RESERVED1_MAX                       (0x00000003U)

#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_MULT_MASK                           (0x0000C000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_MULT_SHIFT                          (0x0000000EU)
#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_MULT_MAX                            (0x00000003U)

#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_MAXPKTSIZE_MASK                     (0x07FF0000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_MAXPKTSIZE_SHIFT                    (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_MAXPKTSIZE_MAX                      (0x000007FFU)

#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_BUFFERING_MASK                      (0xF8000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_BUFFERING_SHIFT                     (0x0000001BU)
#define CSL_USB3P0SS_CTRL_DEV_EP_CFG_BUFFERING_MAX                       (0x0000001FU)

/* EP_CMD */

#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_EPRST_MASK                          (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_EPRST_SHIFT                         (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_EPRST_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_SSTALL_MASK                         (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_SSTALL_SHIFT                        (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_SSTALL_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_CSTALL_MASK                         (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_CSTALL_SHIFT                        (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_CSTALL_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_ERDY_MASK                           (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_ERDY_SHIFT                          (0x00000003U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_ERDY_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_RESERVED0_MASK                      (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_RESERVED0_SHIFT                     (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_RESERVED0_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_REQ_CMPL_MASK                       (0x00000020U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_REQ_CMPL_SHIFT                      (0x00000005U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_REQ_CMPL_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_DRDY_MASK                           (0x00000040U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_DRDY_SHIFT                          (0x00000006U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_DRDY_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_DFLUSH_MASK                         (0x00000080U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_DFLUSH_SHIFT                        (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_DFLUSH_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_RESERVED1_MASK                      (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_RESERVED1_SHIFT                     (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_RESERVED1_MAX                       (0x000000FFU)

#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_ERDY_SID_MASK                       (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_ERDY_SID_SHIFT                      (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_EP_CMD_ERDY_SID_MAX                        (0x0000FFFFU)

/* EP_STS */

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_SETUP_MASK                          (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_SETUP_SHIFT                         (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_SETUP_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_STALL_MASK                          (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_STALL_SHIFT                         (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_STALL_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_IOC_MASK                            (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_IOC_SHIFT                           (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_IOC_MAX                             (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_ISP_MASK                            (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_ISP_SHIFT                           (0x00000003U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_ISP_MAX                             (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_DESCMIS_MASK                        (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_DESCMIS_SHIFT                       (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_DESCMIS_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_STREAMR_MASK                        (0x00000020U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_STREAMR_SHIFT                       (0x00000005U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_STREAMR_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_MD_EXIT_MASK                        (0x00000040U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_MD_EXIT_SHIFT                       (0x00000006U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_MD_EXIT_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_TRBERR_MASK                         (0x00000080U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_TRBERR_SHIFT                        (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_TRBERR_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_NRDY_MASK                           (0x00000100U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_NRDY_SHIFT                          (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_NRDY_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_DBUSY_MASK                          (0x00000200U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_DBUSY_SHIFT                         (0x00000009U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_DBUSY_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_BUFFEMPTY_MASK                      (0x00000400U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_BUFFEMPTY_SHIFT                     (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_BUFFEMPTY_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_CCS_MASK                            (0x00000800U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_CCS_SHIFT                           (0x0000000BU)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_CCS_MAX                             (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_PRIME_MASK                          (0x00001000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_PRIME_SHIFT                         (0x0000000CU)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_PRIME_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_SIDERR_MASK                         (0x00002000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_SIDERR_SHIFT                        (0x0000000DU)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_SIDERR_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_OUTSMM_MASK                         (0x00004000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_OUTSMM_SHIFT                        (0x0000000EU)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_OUTSMM_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_ISOERR_MASK                         (0x00008000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_ISOERR_SHIFT                        (0x0000000FU)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_ISOERR_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_HOSTPP_MASK                         (0x00010000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_HOSTPP_SHIFT                        (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_HOSTPP_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_SPSMST_MASK                         (0x00060000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_SPSMST_SHIFT                        (0x00000011U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_SPSMST_MAX                          (0x00000003U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_IOT_MASK                            (0x00080000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_IOT_SHIFT                           (0x00000013U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_IOT_MAX                             (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_RESERVED0_MASK                      (0x00F00000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_RESERVED0_SHIFT                     (0x00000014U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_RESERVED0_MAX                       (0x0000000FU)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_OUTQ_NO_MASK                        (0x0F000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_OUTQ_NO_SHIFT                       (0x00000018U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_OUTQ_NO_MAX                         (0x0000000FU)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_OUTQ_VAL_MASK                       (0x10000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_OUTQ_VAL_SHIFT                      (0x0000001CU)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_OUTQ_VAL_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_RESERVED1_MASK                      (0x60000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_RESERVED1_SHIFT                     (0x0000001DU)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_RESERVED1_MAX                       (0x00000003U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_STPWAIT_MASK                        (0x80000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_STPWAIT_SHIFT                       (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_STPWAIT_MAX                         (0x00000001U)

/* EP_STS_SID */

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_SID_SID_MASK                        (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_SID_SID_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_SID_SID_MAX                         (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_SID_RESERVED_MASK                   (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_SID_RESERVED_SHIFT                  (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_SID_RESERVED_MAX                    (0x0000FFFFU)

/* EP_STS_EN */

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_SETUPEN_MASK                     (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_SETUPEN_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_SETUPEN_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_RESERVED0_MASK                   (0x0000000EU)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_RESERVED0_SHIFT                  (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_RESERVED0_MAX                    (0x00000007U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_DESCMISEN_MASK                   (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_DESCMISEN_SHIFT                  (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_DESCMISEN_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_STREAMREN_MASK                   (0x00000020U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_STREAMREN_SHIFT                  (0x00000005U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_STREAMREN_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_MD_EXITEN_MASK                   (0x00000040U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_MD_EXITEN_SHIFT                  (0x00000006U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_MD_EXITEN_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_TRBERREN_MASK                    (0x00000080U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_TRBERREN_SHIFT                   (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_TRBERREN_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_NRDYEN_MASK                      (0x00000100U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_NRDYEN_SHIFT                     (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_NRDYEN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_RESERVED1_MASK                   (0x00000E00U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_RESERVED1_SHIFT                  (0x00000009U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_RESERVED1_MAX                    (0x00000007U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_PRIMEEN_MASK                     (0x00001000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_PRIMEEN_SHIFT                    (0x0000000CU)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_PRIMEEN_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_SIDERREN_MASK                    (0x00002000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_SIDERREN_SHIFT                   (0x0000000DU)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_SIDERREN_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_OUTSMMEN_MASK                    (0x00004000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_OUTSMMEN_SHIFT                   (0x0000000EU)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_OUTSMMEN_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_ISOERREN_MASK                    (0x00008000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_ISOERREN_SHIFT                   (0x0000000FU)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_ISOERREN_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_RESERVED2_MASK                   (0x00070000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_RESERVED2_SHIFT                  (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_RESERVED2_MAX                    (0x00000007U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_IOTEN_MASK                       (0x00080000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_IOTEN_SHIFT                      (0x00000013U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_IOTEN_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_RESERVED3_MASK                   (0x7FF00000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_RESERVED3_SHIFT                  (0x00000014U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_RESERVED3_MAX                    (0x000007FFU)

#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_STPWAITEN_MASK                   (0x80000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_STPWAITEN_SHIFT                  (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DEV_EP_STS_EN_STPWAITEN_MAX                    (0x00000001U)

/* DRBL */

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL0O_MASK                           (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL0O_SHIFT                          (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL0O_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL1O_MASK                           (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL1O_SHIFT                          (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL1O_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL2O_MASK                           (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL2O_SHIFT                          (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL2O_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL3O_MASK                           (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL3O_SHIFT                          (0x00000003U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL3O_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL4O_MASK                           (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL4O_SHIFT                          (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL4O_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL5O_MASK                           (0x00000020U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL5O_SHIFT                          (0x00000005U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL5O_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL6O_MASK                           (0x00000040U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL6O_SHIFT                          (0x00000006U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL6O_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL7O_MASK                           (0x00000080U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL7O_SHIFT                          (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL7O_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL8O_MASK                           (0x00000100U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL8O_SHIFT                          (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL8O_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL9O_MASK                           (0x00000200U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL9O_SHIFT                          (0x00000009U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL9O_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL10O_MASK                          (0x00000400U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL10O_SHIFT                         (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL10O_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL11O_MASK                          (0x00000800U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL11O_SHIFT                         (0x0000000BU)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL11O_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL12O_MASK                          (0x00001000U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL12O_SHIFT                         (0x0000000CU)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL12O_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL13O_MASK                          (0x00002000U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL13O_SHIFT                         (0x0000000DU)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL13O_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL14O_MASK                          (0x00004000U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL14O_SHIFT                         (0x0000000EU)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL14O_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL15O_MASK                          (0x00008000U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL15O_SHIFT                         (0x0000000FU)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL15O_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL0I_MASK                           (0x00010000U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL0I_SHIFT                          (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL0I_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL1I_MASK                           (0x00020000U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL1I_SHIFT                          (0x00000011U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL1I_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL2I_MASK                           (0x00040000U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL2I_SHIFT                          (0x00000012U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL2I_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL3I_MASK                           (0x00080000U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL3I_SHIFT                          (0x00000013U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL3I_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL4I_MASK                           (0x00100000U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL4I_SHIFT                          (0x00000014U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL4I_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL5I_MASK                           (0x00200000U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL5I_SHIFT                          (0x00000015U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL5I_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL6I_MASK                           (0x00400000U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL6I_SHIFT                          (0x00000016U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL6I_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL7I_MASK                           (0x00800000U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL7I_SHIFT                          (0x00000017U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL7I_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL8I_MASK                           (0x01000000U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL8I_SHIFT                          (0x00000018U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL8I_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL9I_MASK                           (0x02000000U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL9I_SHIFT                          (0x00000019U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL9I_MAX                            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL10I_MASK                          (0x04000000U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL10I_SHIFT                         (0x0000001AU)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL10I_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL11I_MASK                          (0x08000000U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL11I_SHIFT                         (0x0000001BU)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL11I_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL12I_MASK                          (0x10000000U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL12I_SHIFT                         (0x0000001CU)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL12I_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL13I_MASK                          (0x20000000U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL13I_SHIFT                         (0x0000001DU)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL13I_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL14I_MASK                          (0x40000000U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL14I_SHIFT                         (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL14I_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL15I_MASK                          (0x80000000U)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL15I_SHIFT                         (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DEV_DRBL_DRBL15I_MAX                           (0x00000001U)

/* EP_IEN */

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN0_MASK                        (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN0_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN0_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN1_MASK                        (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN1_SHIFT                       (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN1_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN2_MASK                        (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN2_SHIFT                       (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN2_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN3_MASK                        (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN3_SHIFT                       (0x00000003U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN3_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN4_MASK                        (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN4_SHIFT                       (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN4_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN5_MASK                        (0x00000020U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN5_SHIFT                       (0x00000005U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN5_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN6_MASK                        (0x00000040U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN6_SHIFT                       (0x00000006U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN6_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN7_MASK                        (0x00000080U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN7_SHIFT                       (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN7_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN8_MASK                        (0x00000100U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN8_SHIFT                       (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN8_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN9_MASK                        (0x00000200U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN9_SHIFT                       (0x00000009U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN9_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN10_MASK                       (0x00000400U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN10_SHIFT                      (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN10_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN11_MASK                       (0x00000800U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN11_SHIFT                      (0x0000000BU)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN11_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN12_MASK                       (0x00001000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN12_SHIFT                      (0x0000000CU)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN12_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN13_MASK                       (0x00002000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN13_SHIFT                      (0x0000000DU)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN13_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN14_MASK                       (0x00004000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN14_SHIFT                      (0x0000000EU)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN14_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN15_MASK                       (0x00008000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN15_SHIFT                      (0x0000000FU)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EOUTEN15_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN0_MASK                         (0x00010000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN0_SHIFT                        (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN0_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN1_MASK                         (0x00020000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN1_SHIFT                        (0x00000011U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN1_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN2_MASK                         (0x00040000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN2_SHIFT                        (0x00000012U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN2_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN3_MASK                         (0x00080000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN3_SHIFT                        (0x00000013U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN3_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN4_MASK                         (0x00100000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN4_SHIFT                        (0x00000014U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN4_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN5_MASK                         (0x00200000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN5_SHIFT                        (0x00000015U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN5_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN6_MASK                         (0x00400000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN6_SHIFT                        (0x00000016U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN6_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN7_MASK                         (0x00800000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN7_SHIFT                        (0x00000017U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN7_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN8_MASK                         (0x01000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN8_SHIFT                        (0x00000018U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN8_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN9_MASK                         (0x02000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN9_SHIFT                        (0x00000019U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN9_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN10_MASK                        (0x04000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN10_SHIFT                       (0x0000001AU)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN10_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN11_MASK                        (0x08000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN11_SHIFT                       (0x0000001BU)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN11_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN12_MASK                        (0x10000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN12_SHIFT                       (0x0000001CU)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN12_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN13_MASK                        (0x20000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN13_SHIFT                       (0x0000001DU)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN13_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN14_MASK                        (0x40000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN14_SHIFT                       (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN14_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN15_MASK                        (0x80000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN15_SHIFT                       (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DEV_EP_IEN_EINEN15_MAX                         (0x00000001U)

/* EP_ISTS */

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT0_MASK                         (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT0_SHIFT                        (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT0_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT1_MASK                         (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT1_SHIFT                        (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT1_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT2_MASK                         (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT2_SHIFT                        (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT2_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT3_MASK                         (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT3_SHIFT                        (0x00000003U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT3_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT4_MASK                         (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT4_SHIFT                        (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT4_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT5_MASK                         (0x00000020U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT5_SHIFT                        (0x00000005U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT5_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT6_MASK                         (0x00000040U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT6_SHIFT                        (0x00000006U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT6_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT7_MASK                         (0x00000080U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT7_SHIFT                        (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT7_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT8_MASK                         (0x00000100U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT8_SHIFT                        (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT8_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT9_MASK                         (0x00000200U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT9_SHIFT                        (0x00000009U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT9_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT10_MASK                        (0x00000400U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT10_SHIFT                       (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT10_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT11_MASK                        (0x00000800U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT11_SHIFT                       (0x0000000BU)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT11_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT12_MASK                        (0x00001000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT12_SHIFT                       (0x0000000CU)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT12_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT13_MASK                        (0x00002000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT13_SHIFT                       (0x0000000DU)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT13_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT14_MASK                        (0x00004000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT14_SHIFT                       (0x0000000EU)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT14_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT15_MASK                        (0x00008000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT15_SHIFT                       (0x0000000FU)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EOUT15_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN0_MASK                          (0x00010000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN0_SHIFT                         (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN0_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN1_MASK                          (0x00020000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN1_SHIFT                         (0x00000011U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN1_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN2_MASK                          (0x00040000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN2_SHIFT                         (0x00000012U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN2_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN3_MASK                          (0x00080000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN3_SHIFT                         (0x00000013U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN3_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN4_MASK                          (0x00100000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN4_SHIFT                         (0x00000014U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN4_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN5_MASK                          (0x00200000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN5_SHIFT                         (0x00000015U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN5_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN6_MASK                          (0x00400000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN6_SHIFT                         (0x00000016U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN6_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN7_MASK                          (0x00800000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN7_SHIFT                         (0x00000017U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN7_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN8_MASK                          (0x01000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN8_SHIFT                         (0x00000018U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN8_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN9_MASK                          (0x02000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN9_SHIFT                         (0x00000019U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN9_MAX                           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN10_MASK                         (0x04000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN10_SHIFT                        (0x0000001AU)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN10_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN11_MASK                         (0x08000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN11_SHIFT                        (0x0000001BU)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN11_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN12_MASK                         (0x10000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN12_SHIFT                        (0x0000001CU)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN12_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN13_MASK                         (0x20000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN13_SHIFT                        (0x0000001DU)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN13_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN14_MASK                         (0x40000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN14_SHIFT                        (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN14_MAX                          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN15_MASK                         (0x80000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN15_SHIFT                        (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DEV_EP_ISTS_EIN15_MAX                          (0x00000001U)

/* USB_PWR */

#define CSL_USB3P0SS_CTRL_DEV_USB_PWR_PSO_EN_MASK                        (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_USB_PWR_PSO_EN_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_PWR_PSO_EN_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_PWR_PSO_DS_MASK                        (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_USB_PWR_PSO_DS_SHIFT                       (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_USB_PWR_PSO_DS_MAX                         (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_PWR_RESERVED0_MASK                     (0x000000FCU)
#define CSL_USB3P0SS_CTRL_DEV_USB_PWR_RESERVED0_SHIFT                    (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_USB_PWR_RESERVED0_MAX                      (0x0000003FU)

#define CSL_USB3P0SS_CTRL_DEV_USB_PWR_STB_CLK_SWITCH_EN_MASK             (0x00000100U)
#define CSL_USB3P0SS_CTRL_DEV_USB_PWR_STB_CLK_SWITCH_EN_SHIFT            (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_USB_PWR_STB_CLK_SWITCH_EN_MAX              (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_PWR_STB_CLK_SWITCH_DONE_MASK           (0x00000200U)
#define CSL_USB3P0SS_CTRL_DEV_USB_PWR_STB_CLK_SWITCH_DONE_SHIFT          (0x00000009U)
#define CSL_USB3P0SS_CTRL_DEV_USB_PWR_STB_CLK_SWITCH_DONE_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_PWR_RESERVED1_MASK                     (0x3FFFFC00U)
#define CSL_USB3P0SS_CTRL_DEV_USB_PWR_RESERVED1_SHIFT                    (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DEV_USB_PWR_RESERVED1_MAX                      (0x000FFFFFU)

#define CSL_USB3P0SS_CTRL_DEV_USB_PWR_FAST_REG_ACCESS_STAT_MASK          (0x40000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_PWR_FAST_REG_ACCESS_STAT_SHIFT         (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_USB_PWR_FAST_REG_ACCESS_STAT_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_PWR_FAST_REG_ACCESS_MASK               (0x80000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_PWR_FAST_REG_ACCESS_SHIFT              (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DEV_USB_PWR_FAST_REG_ACCESS_MAX                (0x00000001U)

/* USB_CONF2 */

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF2_AHB_RETRY_EN_MASK                (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF2_AHB_RETRY_EN_SHIFT               (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF2_AHB_RETRY_EN_MAX                 (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CONF2_RESERVED_MASK                    (0xFFFFFFFEU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF2_RESERVED_SHIFT                   (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CONF2_RESERVED_MAX                     (0x7FFFFFFFU)

/* USB_CAP1 */

#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_SFR_TYPE_MASK                     (0x0000000FU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_SFR_TYPE_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_SFR_TYPE_MAX                      (0x0000000FU)

#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_SFR_WIDTH_MASK                    (0x000000F0U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_SFR_WIDTH_SHIFT                   (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_SFR_WIDTH_MAX                     (0x0000000FU)

#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_DMA_TYPE_MASK                     (0x00000F00U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_DMA_TYPE_SHIFT                    (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_DMA_TYPE_MAX                      (0x0000000FU)

#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_DMA_WIDTH_MASK                    (0x0000F000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_DMA_WIDTH_SHIFT                   (0x0000000CU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_DMA_WIDTH_MAX                     (0x0000000FU)

#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_U3PHY_TYPE_MASK                   (0x000F0000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_U3PHY_TYPE_SHIFT                  (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_U3PHY_TYPE_MAX                    (0x0000000FU)

#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_U3PHY_WIDTH_MASK                  (0x00F00000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_U3PHY_WIDTH_SHIFT                 (0x00000014U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_U3PHY_WIDTH_MAX                   (0x0000000FU)

#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_U2PHY_EN_MASK                     (0x01000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_U2PHY_EN_SHIFT                    (0x00000018U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_U2PHY_EN_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_U2PHY_TYPE_MASK                   (0x02000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_U2PHY_TYPE_SHIFT                  (0x00000019U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_U2PHY_TYPE_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_U2PHY_WIDTH_MASK                  (0x04000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_U2PHY_WIDTH_SHIFT                 (0x0000001AU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_U2PHY_WIDTH_MAX                   (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_OTG_READY_MASK                    (0x08000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_OTG_READY_SHIFT                   (0x0000001BU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_OTG_READY_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_TDL_FROM_TRB_MASK                 (0x10000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_TDL_FROM_TRB_SHIFT                (0x0000001CU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_TDL_FROM_TRB_MAX                  (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_RESERVED_MASK                     (0xE0000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_RESERVED_SHIFT                    (0x0000001DU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP1_RESERVED_MAX                      (0x00000007U)

/* USB_CAP2 */

#define CSL_USB3P0SS_CTRL_DEV_USB_CAP2_ACTUAL_MEM_SIZE_MASK              (0x000000FFU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP2_ACTUAL_MEM_SIZE_SHIFT             (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP2_ACTUAL_MEM_SIZE_MAX               (0x000000FFU)

#define CSL_USB3P0SS_CTRL_DEV_USB_CAP2_MAX_MEM_SIZE_MASK                 (0x00001F00U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP2_MAX_MEM_SIZE_SHIFT                (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP2_MAX_MEM_SIZE_MAX                  (0x0000001FU)

#define CSL_USB3P0SS_CTRL_DEV_USB_CAP2_RESERVED_MASK                     (0xFFFFE000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP2_RESERVED_SHIFT                    (0x0000000DU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP2_RESERVED_MAX                      (0x0007FFFFU)

/* USB_CAP3 */

#define CSL_USB3P0SS_CTRL_DEV_USB_CAP3_EPOUT_N_MASK                      (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP3_EPOUT_N_SHIFT                     (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP3_EPOUT_N_MAX                       (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_DEV_USB_CAP3_EPIN_N_MASK                       (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP3_EPIN_N_SHIFT                      (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP3_EPIN_N_MAX                        (0x0000FFFFU)

/* USB_CAP4 */

#define CSL_USB3P0SS_CTRL_DEV_USB_CAP4_EPOUTI_N_MASK                     (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP4_EPOUTI_N_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP4_EPOUTI_N_MAX                      (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_DEV_USB_CAP4_EPINI_N_MASK                      (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP4_EPINI_N_SHIFT                     (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP4_EPINI_N_MAX                       (0x0000FFFFU)

/* USB_CAP5 */

#define CSL_USB3P0SS_CTRL_DEV_USB_CAP5_EPOUTI_N_MASK                     (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP5_EPOUTI_N_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP5_EPOUTI_N_MAX                      (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_DEV_USB_CAP5_EPINI_N_MASK                      (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP5_EPINI_N_SHIFT                     (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP5_EPINI_N_MAX                       (0x0000FFFFU)

/* USB_CAP6 */

#define CSL_USB3P0SS_CTRL_DEV_USB_CAP6_VERSION_MASK                      (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP6_VERSION_SHIFT                     (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CAP6_VERSION_MAX                       (0xFFFFFFFFU)

/* USB_CPKT1 */

#define CSL_USB3P0SS_CTRL_DEV_USB_CPKT1_CPKT1_MASK                       (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CPKT1_CPKT1_SHIFT                      (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CPKT1_CPKT1_MAX                        (0xFFFFFFFFU)

/* USB_CPKT2 */

#define CSL_USB3P0SS_CTRL_DEV_USB_CPKT2_CPKT2_MASK                       (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CPKT2_CPKT2_SHIFT                      (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CPKT2_CPKT2_MAX                        (0xFFFFFFFFU)

/* USB_CPKT3 */

#define CSL_USB3P0SS_CTRL_DEV_USB_CPKT3_CPKT3_MASK                       (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_DEV_USB_CPKT3_CPKT3_SHIFT                      (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_USB_CPKT3_CPKT3_MAX                        (0xFFFFFFFFU)

/* EP_DMA_EXT_ADDR */

#define CSL_USB3P0SS_CTRL_DEV_EP_DMA_EXT_ADDR_EP_DMA_ADDR_H_MASK         (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_DEV_EP_DMA_EXT_ADDR_EP_DMA_ADDR_H_SHIFT        (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_DMA_EXT_ADDR_EP_DMA_ADDR_H_MAX          (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_DEV_EP_DMA_EXT_ADDR_RESERVED0_MASK             (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_DMA_EXT_ADDR_RESERVED0_SHIFT            (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_EP_DMA_EXT_ADDR_RESERVED0_MAX              (0x0000FFFFU)

/* BUF_ADDR */

#define CSL_USB3P0SS_CTRL_DEV_BUF_ADDR_BUF_ADDR_MASK                     (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_DEV_BUF_ADDR_BUF_ADDR_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_BUF_ADDR_BUF_ADDR_MAX                      (0xFFFFFFFFU)

/* BUF_DATA */

#define CSL_USB3P0SS_CTRL_DEV_BUF_DATA_BUF_DATA_MASK                     (0xFFFFFFFFU)
#define CSL_USB3P0SS_CTRL_DEV_BUF_DATA_BUF_DATA_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_BUF_DATA_BUF_DATA_MAX                      (0xFFFFFFFFU)

/* BUF_CTRL */

#define CSL_USB3P0SS_CTRL_DEV_BUF_CTRL_BUF_CMD_SET_MASK                  (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_BUF_CTRL_BUF_CMD_SET_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_BUF_CTRL_BUF_CMD_SET_MAX                   (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_BUF_CTRL_BUF_CMD_WR_MASK                   (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_BUF_CTRL_BUF_CMD_WR_SHIFT                  (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_BUF_CTRL_BUF_CMD_WR_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_BUF_CTRL_BUF_CMD_RD_MASK                   (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_BUF_CTRL_BUF_CMD_RD_SHIFT                  (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_BUF_CTRL_BUF_CMD_RD_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_BUF_CTRL_BUF_CMD_WR4_MASK                  (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_BUF_CTRL_BUF_CMD_WR4_SHIFT                 (0x00000003U)
#define CSL_USB3P0SS_CTRL_DEV_BUF_CTRL_BUF_CMD_WR4_MAX                   (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_BUF_CTRL_BUF_CMD_IWU_MASK                  (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_BUF_CTRL_BUF_CMD_IWU_SHIFT                 (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_BUF_CTRL_BUF_CMD_IWU_MAX                   (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_BUF_CTRL_BUF_CMD_IWD_MASK                  (0x00000020U)
#define CSL_USB3P0SS_CTRL_DEV_BUF_CTRL_BUF_CMD_IWD_SHIFT                 (0x00000005U)
#define CSL_USB3P0SS_CTRL_DEV_BUF_CTRL_BUF_CMD_IWD_MAX                   (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_BUF_CTRL_RESERVED0_MASK                    (0x7FFFFFC0U)
#define CSL_USB3P0SS_CTRL_DEV_BUF_CTRL_RESERVED0_SHIFT                   (0x00000006U)
#define CSL_USB3P0SS_CTRL_DEV_BUF_CTRL_RESERVED0_MAX                     (0x01FFFFFFU)

#define CSL_USB3P0SS_CTRL_DEV_BUF_CTRL_BUF_CMD_STS_MASK                  (0x80000000U)
#define CSL_USB3P0SS_CTRL_DEV_BUF_CTRL_BUF_CMD_STS_SHIFT                 (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DEV_BUF_CTRL_BUF_CMD_STS_MAX                   (0x00000001U)

/* DTRANS */

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS0O_MASK                       (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS0O_SHIFT                      (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS0O_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS1O_MASK                       (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS1O_SHIFT                      (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS1O_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS2O_MASK                       (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS2O_SHIFT                      (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS2O_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS3O_MASK                       (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS3O_SHIFT                      (0x00000003U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS3O_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS4O_MASK                       (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS4O_SHIFT                      (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS4O_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS5O_MASK                       (0x00000020U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS5O_SHIFT                      (0x00000005U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS5O_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS6O_MASK                       (0x00000040U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS6O_SHIFT                      (0x00000006U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS6O_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS7O_MASK                       (0x00000080U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS7O_SHIFT                      (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS7O_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS8O_MASK                       (0x00000100U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS8O_SHIFT                      (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS8O_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS9O_MASK                       (0x00000200U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS9O_SHIFT                      (0x00000009U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS9O_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS10O_MASK                      (0x00000400U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS10O_SHIFT                     (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS10O_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS11O_MASK                      (0x00000800U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS11O_SHIFT                     (0x0000000BU)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS11O_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS12O_MASK                      (0x00001000U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS12O_SHIFT                     (0x0000000CU)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS12O_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS13O_MASK                      (0x00002000U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS13O_SHIFT                     (0x0000000DU)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS13O_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS14O_MASK                      (0x00004000U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS14O_SHIFT                     (0x0000000EU)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS14O_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS15O_MASK                      (0x00008000U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS15O_SHIFT                     (0x0000000FU)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS15O_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS0I_MASK                       (0x00010000U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS0I_SHIFT                      (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS0I_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS1I_MASK                       (0x00020000U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS1I_SHIFT                      (0x00000011U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS1I_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS2I_MASK                       (0x00040000U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS2I_SHIFT                      (0x00000012U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS2I_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS3I_MASK                       (0x00080000U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS3I_SHIFT                      (0x00000013U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS3I_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS4I_MASK                       (0x00100000U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS4I_SHIFT                      (0x00000014U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS4I_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS5I_MASK                       (0x00200000U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS5I_SHIFT                      (0x00000015U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS5I_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS6I_MASK                       (0x00400000U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS6I_SHIFT                      (0x00000016U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS6I_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS7I_MASK                       (0x00800000U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS7I_SHIFT                      (0x00000017U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS7I_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS8I_MASK                       (0x01000000U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS8I_SHIFT                      (0x00000018U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS8I_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS9I_MASK                       (0x02000000U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS9I_SHIFT                      (0x00000019U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS9I_MAX                        (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS10I_MASK                      (0x04000000U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS10I_SHIFT                     (0x0000001AU)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS10I_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS11I_MASK                      (0x08000000U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS11I_SHIFT                     (0x0000001BU)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS11I_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS12I_MASK                      (0x10000000U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS12I_SHIFT                     (0x0000001CU)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS12I_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS13I_MASK                      (0x20000000U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS13I_SHIFT                     (0x0000001DU)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS13I_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS14I_MASK                      (0x40000000U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS14I_SHIFT                     (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS14I_MAX                       (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS15I_MASK                      (0x80000000U)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS15I_SHIFT                     (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DEV_DTRANS_DTRANS15I_MAX                       (0x00000001U)

/* TDL_FROM_TRB */

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB0O_MASK           (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB0O_SHIFT          (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB0O_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB1O_MASK           (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB1O_SHIFT          (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB1O_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB2O_MASK           (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB2O_SHIFT          (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB2O_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB3O_MASK           (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB3O_SHIFT          (0x00000003U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB3O_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB4O_MASK           (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB4O_SHIFT          (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB4O_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB5O_MASK           (0x00000020U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB5O_SHIFT          (0x00000005U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB5O_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB6O_MASK           (0x00000040U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB6O_SHIFT          (0x00000006U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB6O_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB7O_MASK           (0x00000080U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB7O_SHIFT          (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB7O_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB8O_MASK           (0x00000100U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB8O_SHIFT          (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB8O_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB9O_MASK           (0x00000200U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB9O_SHIFT          (0x00000009U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB9O_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB10O_MASK          (0x00000400U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB10O_SHIFT         (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB10O_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB11O_MASK          (0x00000800U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB11O_SHIFT         (0x0000000BU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB11O_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB12O_MASK          (0x00001000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB12O_SHIFT         (0x0000000CU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB12O_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB13O_MASK          (0x00002000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB13O_SHIFT         (0x0000000DU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB13O_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB14O_MASK          (0x00004000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB14O_SHIFT         (0x0000000EU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB14O_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB15O_MASK          (0x00008000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB15O_SHIFT         (0x0000000FU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB15O_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB0I_MASK           (0x00010000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB0I_SHIFT          (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB0I_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB1I_MASK           (0x00020000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB1I_SHIFT          (0x00000011U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB1I_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB2I_MASK           (0x00040000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB2I_SHIFT          (0x00000012U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB2I_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB3I_MASK           (0x00080000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB3I_SHIFT          (0x00000013U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB3I_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB4I_MASK           (0x00100000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB4I_SHIFT          (0x00000014U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB4I_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB5I_MASK           (0x00200000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB5I_SHIFT          (0x00000015U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB5I_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB6I_MASK           (0x00400000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB6I_SHIFT          (0x00000016U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB6I_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB7I_MASK           (0x00800000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB7I_SHIFT          (0x00000017U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB7I_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB8I_MASK           (0x01000000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB8I_SHIFT          (0x00000018U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB8I_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB9I_MASK           (0x02000000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB9I_SHIFT          (0x00000019U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB9I_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB10I_MASK          (0x04000000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB10I_SHIFT         (0x0000001AU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB10I_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB11I_MASK          (0x08000000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB11I_SHIFT         (0x0000001BU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB11I_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB12I_MASK          (0x10000000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB12I_SHIFT         (0x0000001CU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB12I_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB13I_MASK          (0x20000000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB13I_SHIFT         (0x0000001DU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB13I_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB14I_MASK          (0x40000000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB14I_SHIFT         (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB14I_MAX           (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB15I_MASK          (0x80000000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB15I_SHIFT         (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_FROM_TRB_TDL_FROM_TRB15I_MAX           (0x00000001U)

/* TDL_BEH */

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH0O_MASK                     (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH0O_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH0O_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH1O_MASK                     (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH1O_SHIFT                    (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH1O_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH2O_MASK                     (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH2O_SHIFT                    (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH2O_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH3O_MASK                     (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH3O_SHIFT                    (0x00000003U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH3O_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH4O_MASK                     (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH4O_SHIFT                    (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH4O_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH5O_MASK                     (0x00000020U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH5O_SHIFT                    (0x00000005U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH5O_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH6O_MASK                     (0x00000040U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH6O_SHIFT                    (0x00000006U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH6O_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH7O_MASK                     (0x00000080U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH7O_SHIFT                    (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH7O_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH8O_MASK                     (0x00000100U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH8O_SHIFT                    (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH8O_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH9O_MASK                     (0x00000200U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH9O_SHIFT                    (0x00000009U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH9O_MAX                      (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH10O_MASK                    (0x00000400U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH10O_SHIFT                   (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH10O_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH11O_MASK                    (0x00000800U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH11O_SHIFT                   (0x0000000BU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH11O_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH12O_MASK                    (0x00001000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH12O_SHIFT                   (0x0000000CU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH12O_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH13O_MASK                    (0x00002000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH13O_SHIFT                   (0x0000000DU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH13O_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH14O_MASK                    (0x00004000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH14O_SHIFT                   (0x0000000EU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH14O_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH15O_MASK                    (0x00008000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH15O_SHIFT                   (0x0000000FU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_TDL_BEH15O_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED16_MASK                    (0x00010000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED16_SHIFT                   (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED16_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED17_MASK                    (0x00020000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED17_SHIFT                   (0x00000011U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED17_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED18_MASK                    (0x00040000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED18_SHIFT                   (0x00000012U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED18_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED19_MASK                    (0x00080000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED19_SHIFT                   (0x00000013U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED19_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED20_MASK                    (0x00100000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED20_SHIFT                   (0x00000014U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED20_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED21_MASK                    (0x00200000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED21_SHIFT                   (0x00000015U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED21_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED22_MASK                    (0x00400000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED22_SHIFT                   (0x00000016U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED22_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED23_MASK                    (0x00800000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED23_SHIFT                   (0x00000017U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED23_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED24_MASK                    (0x01000000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED24_SHIFT                   (0x00000018U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED24_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED25_MASK                    (0x02000000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED25_SHIFT                   (0x00000019U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED25_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED26_MASK                    (0x04000000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED26_SHIFT                   (0x0000001AU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED26_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED27_MASK                    (0x08000000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED27_SHIFT                   (0x0000001BU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED27_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED28_MASK                    (0x10000000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED28_SHIFT                   (0x0000001CU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED28_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED29_MASK                    (0x20000000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED29_SHIFT                   (0x0000001DU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED29_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED30_MASK                    (0x40000000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED30_SHIFT                   (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED30_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED31_MASK                    (0x80000000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED31_SHIFT                   (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH_RESERVED31_MAX                     (0x00000001U)

/* EP_TDL */

#define CSL_USB3P0SS_CTRL_DEV_EP_TDL_TDL_MASK                            (0x000007FFU)
#define CSL_USB3P0SS_CTRL_DEV_EP_TDL_TDL_SHIFT                           (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_EP_TDL_TDL_MAX                             (0x000007FFU)

#define CSL_USB3P0SS_CTRL_DEV_EP_TDL_RESERVED0_MASK                      (0xFFFFF800U)
#define CSL_USB3P0SS_CTRL_DEV_EP_TDL_RESERVED0_SHIFT                     (0x0000000BU)
#define CSL_USB3P0SS_CTRL_DEV_EP_TDL_RESERVED0_MAX                       (0x001FFFFFU)

/* TDL_BEH2 */

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH20O_MASK                   (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH20O_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH20O_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH21O_MASK                   (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH21O_SHIFT                  (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH21O_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH22O_MASK                   (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH22O_SHIFT                  (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH22O_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH23O_MASK                   (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH23O_SHIFT                  (0x00000003U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH23O_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH24O_MASK                   (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH24O_SHIFT                  (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH24O_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH25O_MASK                   (0x00000020U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH25O_SHIFT                  (0x00000005U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH25O_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH26O_MASK                   (0x00000040U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH26O_SHIFT                  (0x00000006U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH26O_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH27O_MASK                   (0x00000080U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH27O_SHIFT                  (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH27O_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH28O_MASK                   (0x00000100U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH28O_SHIFT                  (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH28O_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH29O_MASK                   (0x00000200U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH29O_SHIFT                  (0x00000009U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH29O_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH210O_MASK                  (0x00000400U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH210O_SHIFT                 (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH210O_MAX                   (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH211O_MASK                  (0x00000800U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH211O_SHIFT                 (0x0000000BU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH211O_MAX                   (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH212O_MASK                  (0x00001000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH212O_SHIFT                 (0x0000000CU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH212O_MAX                   (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH213O_MASK                  (0x00002000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH213O_SHIFT                 (0x0000000DU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH213O_MAX                   (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH214O_MASK                  (0x00004000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH214O_SHIFT                 (0x0000000EU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH214O_MAX                   (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH215O_MASK                  (0x00008000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH215O_SHIFT                 (0x0000000FU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_TDL_BEH215O_MAX                   (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED16_MASK                   (0x00010000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED16_SHIFT                  (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED16_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED17_MASK                   (0x00020000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED17_SHIFT                  (0x00000011U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED17_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED18_MASK                   (0x00040000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED18_SHIFT                  (0x00000012U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED18_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED19_MASK                   (0x00080000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED19_SHIFT                  (0x00000013U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED19_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED20_MASK                   (0x00100000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED20_SHIFT                  (0x00000014U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED20_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED21_MASK                   (0x00200000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED21_SHIFT                  (0x00000015U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED21_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED22_MASK                   (0x00400000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED22_SHIFT                  (0x00000016U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED22_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED23_MASK                   (0x00800000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED23_SHIFT                  (0x00000017U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED23_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED24_MASK                   (0x01000000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED24_SHIFT                  (0x00000018U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED24_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED25_MASK                   (0x02000000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED25_SHIFT                  (0x00000019U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED25_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED26_MASK                   (0x04000000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED26_SHIFT                  (0x0000001AU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED26_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED27_MASK                   (0x08000000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED27_SHIFT                  (0x0000001BU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED27_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED28_MASK                   (0x10000000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED28_SHIFT                  (0x0000001CU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED28_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED29_MASK                   (0x20000000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED29_SHIFT                  (0x0000001DU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED29_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED30_MASK                   (0x40000000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED30_SHIFT                  (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED30_MAX                    (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED31_MASK                   (0x80000000U)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED31_SHIFT                  (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DEV_TDL_BEH2_RESERVED31_MAX                    (0x00000001U)

/* DMA_ADV_TD */

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD0O_MASK               (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD0O_SHIFT              (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD0O_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD1O_MASK               (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD1O_SHIFT              (0x00000001U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD1O_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD2O_MASK               (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD2O_SHIFT              (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD2O_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD3O_MASK               (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD3O_SHIFT              (0x00000003U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD3O_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD4O_MASK               (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD4O_SHIFT              (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD4O_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD5O_MASK               (0x00000020U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD5O_SHIFT              (0x00000005U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD5O_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD6O_MASK               (0x00000040U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD6O_SHIFT              (0x00000006U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD6O_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD7O_MASK               (0x00000080U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD7O_SHIFT              (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD7O_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD8O_MASK               (0x00000100U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD8O_SHIFT              (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD8O_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD9O_MASK               (0x00000200U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD9O_SHIFT              (0x00000009U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD9O_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD10O_MASK              (0x00000400U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD10O_SHIFT             (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD10O_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD11O_MASK              (0x00000800U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD11O_SHIFT             (0x0000000BU)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD11O_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD12O_MASK              (0x00001000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD12O_SHIFT             (0x0000000CU)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD12O_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD13O_MASK              (0x00002000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD13O_SHIFT             (0x0000000DU)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD13O_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD14O_MASK              (0x00004000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD14O_SHIFT             (0x0000000EU)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD14O_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD15O_MASK              (0x00008000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD15O_SHIFT             (0x0000000FU)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD15O_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD0I_MASK               (0x00010000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD0I_SHIFT              (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD0I_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD1I_MASK               (0x00020000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD1I_SHIFT              (0x00000011U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD1I_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD2I_MASK               (0x00040000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD2I_SHIFT              (0x00000012U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD2I_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD3I_MASK               (0x00080000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD3I_SHIFT              (0x00000013U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD3I_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD4I_MASK               (0x00100000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD4I_SHIFT              (0x00000014U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD4I_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD5I_MASK               (0x00200000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD5I_SHIFT              (0x00000015U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD5I_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD6I_MASK               (0x00400000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD6I_SHIFT              (0x00000016U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD6I_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD7I_MASK               (0x00800000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD7I_SHIFT              (0x00000017U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD7I_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD8I_MASK               (0x01000000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD8I_SHIFT              (0x00000018U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD8I_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD9I_MASK               (0x02000000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD9I_SHIFT              (0x00000019U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD9I_MAX                (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD10I_MASK              (0x04000000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD10I_SHIFT             (0x0000001AU)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD10I_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD11I_MASK              (0x08000000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD11I_SHIFT             (0x0000001BU)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD11I_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD12I_MASK              (0x10000000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD12I_SHIFT             (0x0000001CU)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD12I_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD13I_MASK              (0x20000000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD13I_SHIFT             (0x0000001DU)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD13I_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD14I_MASK              (0x40000000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD14I_SHIFT             (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD14I_MAX               (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD15I_MASK              (0x80000000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD15I_SHIFT             (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DEV_DMA_ADV_TD_DMA_ADV_TD15I_MAX               (0x00000001U)

/* CFG_REG1 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG1_DEBOUNCER_CNT_MASK                (0x0003FFFFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG1_DEBOUNCER_CNT_SHIFT               (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG1_DEBOUNCER_CNT_MAX                 (0x0003FFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG1_RESERVED_MASK                     (0xFFFC0000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG1_RESERVED_SHIFT                    (0x00000012U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG1_RESERVED_MAX                      (0x00003FFFU)

/* DBG_LINK1 */

#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_LFPS_MIN_DET_U1_EXIT_MASK        (0x000000FFU)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_LFPS_MIN_DET_U1_EXIT_SHIFT       (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_LFPS_MIN_DET_U1_EXIT_MAX         (0x000000FFU)

#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_LFPS_MIN_GEN_U1_EXIT_MASK        (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_LFPS_MIN_GEN_U1_EXIT_SHIFT       (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_LFPS_MIN_GEN_U1_EXIT_MAX         (0x000000FFU)

#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_RXDET_BREAK_DIS_MASK             (0x00010000U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_RXDET_BREAK_DIS_SHIFT            (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_RXDET_BREAK_DIS_MAX              (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_LFPS_GEN_PING_MASK               (0x003E0000U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_LFPS_GEN_PING_SHIFT              (0x00000011U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_LFPS_GEN_PING_MAX                (0x0000001FU)

#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_RESERVED0_MASK                   (0x00C00000U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_RESERVED0_SHIFT                  (0x00000016U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_RESERVED0_MAX                    (0x00000003U)

#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_LFPS_MIN_DET_U1_EXIT_SET_MASK    (0x01000000U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_LFPS_MIN_DET_U1_EXIT_SET_SHIFT   (0x00000018U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_LFPS_MIN_DET_U1_EXIT_SET_MAX     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_LFPS_MIN_GEN_U1_EXIT_SET_MASK    (0x02000000U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_LFPS_MIN_GEN_U1_EXIT_SET_SHIFT   (0x00000019U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_LFPS_MIN_GEN_U1_EXIT_SET_MAX     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_RXDET_BREAK_DIS_SET_MASK         (0x04000000U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_RXDET_BREAK_DIS_SET_SHIFT        (0x0000001AU)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_RXDET_BREAK_DIS_SET_MAX          (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_LFPS_GEN_PING_SET_MASK           (0x08000000U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_LFPS_GEN_PING_SET_SHIFT          (0x0000001BU)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_LFPS_GEN_PING_SET_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_RESERVED1_MASK                   (0xF0000000U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_RESERVED1_SHIFT                  (0x0000001CU)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK1_RESERVED1_MAX                    (0x0000000FU)

/* DBG_LINK2 */

#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2_RXEQTR_AVAL_MASK                 (0x000000FFU)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2_RXEQTR_AVAL_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2_RXEQTR_AVAL_MAX                  (0x000000FFU)

#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2_RXEQTR_DVAL_MASK                 (0x0000FF00U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2_RXEQTR_DVAL_SHIFT                (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2_RXEQTR_DVAL_MAX                  (0x000000FFU)

#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2_PHYRXVAL_DVAL_MASK               (0x00FF0000U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2_PHYRXVAL_DVAL_SHIFT              (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2_PHYRXVAL_DVAL_MAX                (0x000000FFU)

#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2_TXDET_DVAL_MASK                  (0x07000000U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2_TXDET_DVAL_SHIFT                 (0x00000018U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2_TXDET_DVAL_MAX                   (0x00000007U)

#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2_RESERVED_MASK                    (0x08000000U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2_RESERVED_SHIFT                   (0x0000001BU)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2_RESERVED_MAX                     (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2_RXEQTR_AVAL_SET_MASK             (0x10000000U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2_RXEQTR_AVAL_SET_SHIFT            (0x0000001CU)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2_RXEQTR_AVAL_SET_MAX              (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2_RXEQTR_DVAL_SET_MASK             (0x20000000U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2_RXEQTR_DVAL_SET_SHIFT            (0x0000001DU)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2_RXEQTR_DVAL_SET_MAX              (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2_PHYRXVAL_DVAL_SET_MASK           (0x40000000U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2_PHYRXVAL_DVAL_SET_SHIFT          (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2_PHYRXVAL_DVAL_SET_MAX            (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2_TXDET_DVAL_SET_MASK              (0x80000000U)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2_TXDET_DVAL_SET_SHIFT             (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DEV_DBG_LINK2_TXDET_DVAL_SET_MAX               (0x00000001U)

/* CFG_REG4 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG4_RXDETECT_QUIET_TIMEOUT_MASK       (0x000000FFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG4_RXDETECT_QUIET_TIMEOUT_SHIFT      (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG4_RXDETECT_QUIET_TIMEOUT_MAX        (0x000000FFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG4_RESERVED_MASK                     (0x3FFFFF00U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG4_RESERVED_SHIFT                    (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG4_RESERVED_MAX                      (0x003FFFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG4_RXDETECT_QUIET_TIMEOUT_PRESCALE_MASK (0xC0000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG4_RXDETECT_QUIET_TIMEOUT_PRESCALE_SHIFT (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG4_RXDETECT_QUIET_TIMEOUT_PRESCALE_MAX (0x00000003U)

/* CFG_REG5 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG5_U3_HDSK_FAIL_TIMEOUT_MASK         (0x000007FFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG5_U3_HDSK_FAIL_TIMEOUT_SHIFT        (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG5_U3_HDSK_FAIL_TIMEOUT_MAX          (0x000007FFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG5_RESERVED_MASK                     (0x3FFFF800U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG5_RESERVED_SHIFT                    (0x0000000BU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG5_RESERVED_MAX                      (0x0007FFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG5_U3_HDSK_FAIL_TIMEOUT_PRESCALE_MASK (0xC0000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG5_U3_HDSK_FAIL_TIMEOUT_PRESCALE_SHIFT (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG5_U3_HDSK_FAIL_TIMEOUT_PRESCALE_MAX (0x00000003U)

/* CFG_REG6 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG6_SSINACTIVE_QUIET_TIMEOUT_MASK     (0x000000FFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG6_SSINACTIVE_QUIET_TIMEOUT_SHIFT    (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG6_SSINACTIVE_QUIET_TIMEOUT_MAX      (0x000000FFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG6_RESERVED_MASK                     (0x3FFFFF00U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG6_RESERVED_SHIFT                    (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG6_RESERVED_MAX                      (0x003FFFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG6_SSINACTIVE_QUIET_TIMEOUT_PRESCALE_MASK (0xC0000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG6_SSINACTIVE_QUIET_TIMEOUT_PRESCALE_SHIFT (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG6_SSINACTIVE_QUIET_TIMEOUT_PRESCALE_MAX (0x00000003U)

/* CFG_REG7 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG7_POLLING_LFPS_TIMEOUT_MASK         (0x00001FFFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG7_POLLING_LFPS_TIMEOUT_SHIFT        (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG7_POLLING_LFPS_TIMEOUT_MAX          (0x00001FFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG7_RESERVED_MASK                     (0x3FFFE000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG7_RESERVED_SHIFT                    (0x0000000DU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG7_RESERVED_MAX                      (0x0001FFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG7_POLLING_LFPS_TIMEOUT_PRESCALE_MASK (0xC0000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG7_POLLING_LFPS_TIMEOUT_PRESCALE_SHIFT (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG7_POLLING_LFPS_TIMEOUT_PRESCALE_MAX (0x00000003U)

/* CFG_REG8 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG8_POLLING_ACTIVE_TIMEOUT_MASK       (0x000003FFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG8_POLLING_ACTIVE_TIMEOUT_SHIFT      (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG8_POLLING_ACTIVE_TIMEOUT_MAX        (0x000003FFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG8_RESERVED_MASK                     (0x3FFFFC00U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG8_RESERVED_SHIFT                    (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG8_RESERVED_MAX                      (0x000FFFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG8_POLLING_ACTIVE_TIMEOUT_PRESCALE_MASK (0xC0000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG8_POLLING_ACTIVE_TIMEOUT_PRESCALE_SHIFT (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG8_POLLING_ACTIVE_TIMEOUT_PRESCALE_MAX (0x00000003U)

/* CFG_REG9 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG9_POLLING_IDLE_TIMEOUT_MASK         (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG9_POLLING_IDLE_TIMEOUT_SHIFT        (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG9_POLLING_IDLE_TIMEOUT_MAX          (0x0000001FU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG9_RESERVED_MASK                     (0x3FFFFFE0U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG9_RESERVED_SHIFT                    (0x00000005U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG9_RESERVED_MAX                      (0x01FFFFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG9_POLLING_IDLE_TIMEOUT_PRESCALE_MASK (0xC0000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG9_POLLING_IDLE_TIMEOUT_PRESCALE_SHIFT (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG9_POLLING_IDLE_TIMEOUT_PRESCALE_MAX (0x00000003U)

/* CFG_REG10 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG10_POLLING_CONF_TIMEOUT_MASK        (0x000000FFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG10_POLLING_CONF_TIMEOUT_SHIFT       (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG10_POLLING_CONF_TIMEOUT_MAX         (0x000000FFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG10_RESERVED_MASK                    (0x3FFFFF00U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG10_RESERVED_SHIFT                   (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG10_RESERVED_MAX                     (0x003FFFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG10_POLLING_CONF_TIMEOUT_PRESCALE_MASK (0xC0000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG10_POLLING_CONF_TIMEOUT_PRESCALE_SHIFT (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG10_POLLING_CONF_TIMEOUT_PRESCALE_MAX (0x00000003U)

/* CFG_REG11 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG11_RECOVERY_ACTIVE_TIMEOUT_MASK     (0x000000FFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG11_RECOVERY_ACTIVE_TIMEOUT_SHIFT    (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG11_RECOVERY_ACTIVE_TIMEOUT_MAX      (0x000000FFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG11_RESERVED_MASK                    (0x3FFFFF00U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG11_RESERVED_SHIFT                   (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG11_RESERVED_MAX                     (0x003FFFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG11_RECOVERY_ACTIVE_TIMEOUT_PRESCALE_MASK (0xC0000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG11_RECOVERY_ACTIVE_TIMEOUT_PRESCALE_SHIFT (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG11_RECOVERY_ACTIVE_TIMEOUT_PRESCALE_MAX (0x00000003U)

/* CFG_REG12 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG12_RECOVERY_CONF_TIMEOUT_MASK       (0x000000FFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG12_RECOVERY_CONF_TIMEOUT_SHIFT      (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG12_RECOVERY_CONF_TIMEOUT_MAX        (0x000000FFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG12_RESERVED_MASK                    (0x3FFFFF00U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG12_RESERVED_SHIFT                   (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG12_RESERVED_MAX                     (0x003FFFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG12_RECOVERY_CONF_TIMEOUT_PRESCALE_MASK (0xC0000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG12_RECOVERY_CONF_TIMEOUT_PRESCALE_SHIFT (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG12_RECOVERY_CONF_TIMEOUT_PRESCALE_MAX (0x00000003U)

/* CFG_REG13 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG13_RECOVERY_IDLE_TIMEOUT_MASK       (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG13_RECOVERY_IDLE_TIMEOUT_SHIFT      (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG13_RECOVERY_IDLE_TIMEOUT_MAX        (0x0000001FU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG13_RESERVED_MASK                    (0x0FFFFFE0U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG13_RESERVED_SHIFT                   (0x00000005U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG13_RESERVED_MAX                     (0x007FFFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG13_RECOVERY_IDLE_TIMEOUT_PRESCALE_MASK (0xC0000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG13_RECOVERY_IDLE_TIMEOUT_PRESCALE_SHIFT (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG13_RECOVERY_IDLE_TIMEOUT_PRESCALE_MAX (0x00000003U)

/* CFG_REG14 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG14_HOTRESET_ACTIVE_TIMEOUT_MASK     (0x000000FFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG14_HOTRESET_ACTIVE_TIMEOUT_SHIFT    (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG14_HOTRESET_ACTIVE_TIMEOUT_MAX      (0x000000FFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG14_RESERVED_MASK                    (0x3FFFFF00U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG14_RESERVED_SHIFT                   (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG14_RESERVED_MAX                     (0x003FFFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG14_HOTRESET_ACTIVE_TIMEOUT_PRESCALE_MASK (0xC0000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG14_HOTRESET_ACTIVE_TIMEOUT_PRESCALE_SHIFT (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG14_HOTRESET_ACTIVE_TIMEOUT_PRESCALE_MAX (0x00000003U)

/* CFG_REG15 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG15_HOTRESET_EXIT_TIMEOUT_MASK       (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG15_HOTRESET_EXIT_TIMEOUT_SHIFT      (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG15_HOTRESET_EXIT_TIMEOUT_MAX        (0x0000001FU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG15_RESERVED_MASK                    (0x3FFFFFE0U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG15_RESERVED_SHIFT                   (0x00000005U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG15_RESERVED_MAX                     (0x01FFFFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG15_HOTRESET_EXIT_TIMEOUT_PRESCALE_MASK (0xC0000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG15_HOTRESET_EXIT_TIMEOUT_PRESCALE_SHIFT (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG15_HOTRESET_EXIT_TIMEOUT_PRESCALE_MAX (0x00000003U)

/* CFG_REG16 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG16_LFPS_PING_REPEAT_MASK            (0x00000FFFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG16_LFPS_PING_REPEAT_SHIFT           (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG16_LFPS_PING_REPEAT_MAX             (0x00000FFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG16_RESERVED_MASK                    (0x3FFFF000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG16_RESERVED_SHIFT                   (0x0000000CU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG16_RESERVED_MAX                     (0x0003FFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG16_LFPS_PING_REPEAT_PRESCALE_MASK   (0xC0000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG16_LFPS_PING_REPEAT_PRESCALE_SHIFT  (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG16_LFPS_PING_REPEAT_PRESCALE_MAX    (0x00000003U)

/* CFG_REG17 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG17_PENDING_HP_TIMEOUT_MASK          (0x000003FFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG17_PENDING_HP_TIMEOUT_SHIFT         (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG17_PENDING_HP_TIMEOUT_MAX           (0x000003FFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG17_RESERVED_MASK                    (0x3FFFFC00U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG17_RESERVED_SHIFT                   (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG17_RESERVED_MAX                     (0x000FFFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG17_PENDING_HP_TIMEOUT_PRESCALE_MASK (0xC0000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG17_PENDING_HP_TIMEOUT_PRESCALE_SHIFT (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG17_PENDING_HP_TIMEOUT_PRESCALE_MAX  (0x00000003U)

/* CFG_REG18 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG18_CREDIT_HP_TIMEOUT_MASK           (0x0000007FU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG18_CREDIT_HP_TIMEOUT_SHIFT          (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG18_CREDIT_HP_TIMEOUT_MAX            (0x0000007FU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG18_RESERVED_MASK                    (0x3FFFFF80U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG18_RESERVED_SHIFT                   (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG18_RESERVED_MAX                     (0x007FFFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG18_CREDIT_HP_TIMEOUT_PRESCALE_MASK  (0xC0000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG18_CREDIT_HP_TIMEOUT_PRESCALE_SHIFT (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG18_CREDIT_HP_TIMEOUT_PRESCALE_MAX   (0x00000003U)

/* CFG_REG19 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG19_LUP_TIMEOUT_MASK                 (0x000003FFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG19_LUP_TIMEOUT_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG19_LUP_TIMEOUT_MAX                  (0x000003FFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG19_RESERVED_MASK                    (0x3FFFFC00U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG19_RESERVED_SHIFT                   (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG19_RESERVED_MAX                     (0x000FFFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG19_LUP_TIMEOUT_PRESCALE_MASK        (0xC0000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG19_LUP_TIMEOUT_PRESCALE_SHIFT       (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG19_LUP_TIMEOUT_PRESCALE_MAX         (0x00000003U)

/* CFG_REG20 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG20_LDN_TIMEOUT_MASK                 (0x000000FFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG20_LDN_TIMEOUT_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG20_LDN_TIMEOUT_MAX                  (0x000000FFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG20_RESERVED_MASK                    (0x3FFFFF00U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG20_RESERVED_SHIFT                   (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG20_RESERVED_MAX                     (0x003FFFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG20_LDN_TIMEOUT_PRESCALE_MASK        (0xC0000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG20_LDN_TIMEOUT_PRESCALE_SHIFT       (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG20_LDN_TIMEOUT_PRESCALE_MAX         (0x00000003U)

/* CFG_REG21 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG21_PM_LC_TIMEOUT_MASK               (0x000003FFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG21_PM_LC_TIMEOUT_SHIFT              (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG21_PM_LC_TIMEOUT_MAX                (0x000003FFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG21_RESERVED_MASK                    (0x3FFFFC00U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG21_RESERVED_SHIFT                   (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG21_RESERVED_MAX                     (0x000FFFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG21_PM_LC_TIMEOUT_PRESCALE_MASK      (0xC0000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG21_PM_LC_TIMEOUT_PRESCALE_SHIFT     (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG21_PM_LC_TIMEOUT_PRESCALE_MAX       (0x00000003U)

/* CFG_REG22 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG22_PM_ENTRY_TIMEOUT_MASK            (0x000007FFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG22_PM_ENTRY_TIMEOUT_SHIFT           (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG22_PM_ENTRY_TIMEOUT_MAX             (0x000007FFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG22_RESERVED_MASK                    (0x3FFFF800U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG22_RESERVED_SHIFT                   (0x0000000BU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG22_RESERVED_MAX                     (0x0007FFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG22_PM_ENTRY_TIMEOUT_PRESCALE_MASK   (0xC0000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG22_PM_ENTRY_TIMEOUT_PRESCALE_SHIFT  (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG22_PM_ENTRY_TIMEOUT_PRESCALE_MAX    (0x00000003U)

/* CFG_REG23 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG23_UX_EXIT_TIMEOUT_MASK             (0x0000007FU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG23_UX_EXIT_TIMEOUT_SHIFT            (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG23_UX_EXIT_TIMEOUT_MAX              (0x0000007FU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG23_RESERVED_MASK                    (0x3FFFFF80U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG23_RESERVED_SHIFT                   (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG23_RESERVED_MAX                     (0x007FFFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG23_UX_EXIT_TIMEOUT_PRESCALE_MASK    (0xC0000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG23_UX_EXIT_TIMEOUT_PRESCALE_SHIFT   (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG23_UX_EXIT_TIMEOUT_PRESCALE_MAX     (0x00000003U)

/* CFG_REG24 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG24_LFPS_DET_RESET_MIN_MASK          (0x007FFFFFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG24_LFPS_DET_RESET_MIN_SHIFT         (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG24_LFPS_DET_RESET_MIN_MAX           (0x007FFFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG24_RESERVED_MASK                    (0xFF800000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG24_RESERVED_SHIFT                   (0x00000017U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG24_RESERVED_MAX                     (0x000001FFU)

/* CFG_REG25 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG25_LFPS_DET_RESET_MAX_MASK          (0x00FFFFFFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG25_LFPS_DET_RESET_MAX_SHIFT         (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG25_LFPS_DET_RESET_MAX_MAX           (0x00FFFFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG25_RESERVED_MASK                    (0xFF000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG25_RESERVED_SHIFT                   (0x00000018U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG25_RESERVED_MAX                     (0x000000FFU)

/* CFG_REG26 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG26_LFPS_DET_POLLING_MIN_MASK        (0x0000007FU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG26_LFPS_DET_POLLING_MIN_SHIFT       (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG26_LFPS_DET_POLLING_MIN_MAX         (0x0000007FU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG26_RESERVED_MASK                    (0xFFFFFF80U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG26_RESERVED_SHIFT                   (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG26_RESERVED_MAX                     (0x01FFFFFFU)

/* CFG_REG27 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG27_LFPS_DET_POLLING_MAX_MASK        (0x000000FFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG27_LFPS_DET_POLLING_MAX_SHIFT       (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG27_LFPS_DET_POLLING_MAX_MAX         (0x000000FFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG27_RESERVED_MASK                    (0xFFFFFF00U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG27_RESERVED_SHIFT                   (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG27_RESERVED_MAX                     (0x00FFFFFFU)

/* CFG_REG28 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG28_LFPS_DET_PING_MIN_MASK           (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG28_LFPS_DET_PING_MIN_SHIFT          (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG28_LFPS_DET_PING_MIN_MAX            (0x00000007U)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG28_RESERVED_MASK                    (0xFFFFFFF8U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG28_RESERVED_SHIFT                   (0x00000003U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG28_RESERVED_MAX                     (0x1FFFFFFFU)

/* CFG_REG29 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG29_LFPS_DET_PING_MAX_MASK           (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG29_LFPS_DET_PING_MAX_SHIFT          (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG29_LFPS_DET_PING_MAX_MAX            (0x0000001FU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG29_RESERVED_MASK                    (0xFFFFFFE0U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG29_RESERVED_SHIFT                   (0x00000005U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG29_RESERVED_MAX                     (0x07FFFFFFU)

/* CFG_REG30 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG30_LFPS_DET_U1EXIT_MIN_MASK         (0x0000003FU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG30_LFPS_DET_U1EXIT_MIN_SHIFT        (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG30_LFPS_DET_U1EXIT_MIN_MAX          (0x0000003FU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG30_RESERVED_MASK                    (0xFFFFFFC0U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG30_RESERVED_SHIFT                   (0x00000006U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG30_RESERVED_MAX                     (0x03FFFFFFU)

/* CFG_REG31 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG31_LFPS_DET_U1EXIT_MAX_MASK         (0x0000007FU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG31_LFPS_DET_U1EXIT_MAX_SHIFT        (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG31_LFPS_DET_U1EXIT_MAX_MAX          (0x0000007FU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG31_RESERVED_MASK                    (0xFFFFFF80U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG31_RESERVED_SHIFT                   (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG31_RESERVED_MAX                     (0x01FFFFFFU)

/* CFG_REG32 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG32_LFPS_DET_U2EXIT_MIN_MASK         (0x0000003FU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG32_LFPS_DET_U2EXIT_MIN_SHIFT        (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG32_LFPS_DET_U2EXIT_MIN_MAX          (0x0000003FU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG32_RESERVED_MASK                    (0xFFFFFFC0U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG32_RESERVED_SHIFT                   (0x00000006U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG32_RESERVED_MAX                     (0x03FFFFFFU)

/* CFG_REG33 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG33_LFPS_DET_U2EXIT_MAX_MASK         (0x0003FFFFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG33_LFPS_DET_U2EXIT_MAX_SHIFT        (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG33_LFPS_DET_U2EXIT_MAX_MAX          (0x0003FFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG33_RESERVED_MASK                    (0xFFFC0000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG33_RESERVED_SHIFT                   (0x00000012U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG33_RESERVED_MAX                     (0x00003FFFU)

/* CFG_REG34 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG34_LFPS_DET_U3EXIT_MIN_MASK         (0x0000003FU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG34_LFPS_DET_U3EXIT_MIN_SHIFT        (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG34_LFPS_DET_U3EXIT_MIN_MAX          (0x0000003FU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG34_RESERVED_MASK                    (0xFFFFFFC0U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG34_RESERVED_SHIFT                   (0x00000006U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG34_RESERVED_MAX                     (0x03FFFFFFU)

/* CFG_REG35 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG35_LFPS_DET_U3EXIT_MAX_MASK         (0x001FFFFFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG35_LFPS_DET_U3EXIT_MAX_SHIFT        (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG35_LFPS_DET_U3EXIT_MAX_MAX          (0x001FFFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG35_RESERVED_MASK                    (0xFFE00000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG35_RESERVED_SHIFT                   (0x00000015U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG35_RESERVED_MAX                     (0x000007FFU)

/* CFG_REG36 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG36_LFPS_GEN_PING_MASK               (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG36_LFPS_GEN_PING_SHIFT              (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG36_LFPS_GEN_PING_MAX                (0x0000001FU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG36_RESERVED_MASK                    (0xFFFFFFE0U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG36_RESERVED_SHIFT                   (0x00000005U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG36_RESERVED_MAX                     (0x07FFFFFFU)

/* CFG_REG37 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG37_LFPS_GEN_POLLING_MASK            (0x000000FFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG37_LFPS_GEN_POLLING_SHIFT           (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG37_LFPS_GEN_POLLING_MAX             (0x000000FFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG37_RESERVED_MASK                    (0xFFFFFF00U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG37_RESERVED_SHIFT                   (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG37_RESERVED_MAX                     (0x00FFFFFFU)

/* CFG_REG38 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG38_LFPS_GEN_U1EXIT_MASK             (0x0003FFFFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG38_LFPS_GEN_U1EXIT_SHIFT            (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG38_LFPS_GEN_U1EXIT_MAX              (0x0003FFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG38_RESERVED_MASK                    (0xFFFC0000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG38_RESERVED_SHIFT                   (0x00000012U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG38_RESERVED_MAX                     (0x00003FFFU)

/* CFG_REG39 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG39_LFPS_GEN_U3EXIT_MASK             (0x001FFFFFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG39_LFPS_GEN_U3EXIT_SHIFT            (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG39_LFPS_GEN_U3EXIT_MAX              (0x001FFFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG39_RESERVED_MASK                    (0xFFE00000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG39_RESERVED_SHIFT                   (0x00000015U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG39_RESERVED_MAX                     (0x000007FFU)

/* CFG_REG40 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG40_LFPS_MIN_GEN_U1EXIT_MASK         (0x0000007FU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG40_LFPS_MIN_GEN_U1EXIT_SHIFT        (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG40_LFPS_MIN_GEN_U1EXIT_MAX          (0x0000007FU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG40_RESERVED_MASK                    (0xFFFFFF80U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG40_RESERVED_SHIFT                   (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG40_RESERVED_MAX                     (0x01FFFFFFU)

/* CFG_REG41 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG41_LFPS_MIN_GEN_U2EXIT_MASK         (0x00007FFFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG41_LFPS_MIN_GEN_U2EXIT_SHIFT        (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG41_LFPS_MIN_GEN_U2EXIT_MAX          (0x00007FFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG41_RESERVED_MASK                    (0xFFFF8000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG41_RESERVED_SHIFT                   (0x0000000FU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG41_RESERVED_MAX                     (0x0001FFFFU)

/* CFG_REG42 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG42_LFPS_POLLING_REPEAT_MASK         (0x000007FFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG42_LFPS_POLLING_REPEAT_SHIFT        (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG42_LFPS_POLLING_REPEAT_MAX          (0x000007FFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG42_RESERVED_MASK                    (0xFFFFF800U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG42_RESERVED_SHIFT                   (0x0000000BU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG42_RESERVED_MAX                     (0x001FFFFFU)

/* CFG_REG43 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG43_LFPS_POLLING_MAX_TREPEAT_MASK    (0x000007FFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG43_LFPS_POLLING_MAX_TREPEAT_SHIFT   (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG43_LFPS_POLLING_MAX_TREPEAT_MAX     (0x000007FFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG43_RESERVED_MASK                    (0xFFFFF800U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG43_RESERVED_SHIFT                   (0x0000000BU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG43_RESERVED_MAX                     (0x001FFFFFU)

/* CFG_REG44 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG44_LFPS_POLLING_MIN_TREPEAT_MASK    (0x000007FFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG44_LFPS_POLLING_MIN_TREPEAT_SHIFT   (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG44_LFPS_POLLING_MIN_TREPEAT_MAX     (0x000007FFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG44_RESERVED_MASK                    (0xFFFFF800U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG44_RESERVED_SHIFT                   (0x0000000BU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG44_RESERVED_MAX                     (0x001FFFFFU)

/* CFG_REG45 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG45_ITP_WAKEUP_TIMEOUT_MASK          (0x0000007FU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG45_ITP_WAKEUP_TIMEOUT_SHIFT         (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG45_ITP_WAKEUP_TIMEOUT_MAX           (0x0000007FU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG45_RESERVED_MASK                    (0x3FFFFF80U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG45_RESERVED_SHIFT                   (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG45_RESERVED_MAX                     (0x007FFFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG45_ITP_WAKEUP_TIMEOUT_PRESCALE_MASK (0xC0000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG45_ITP_WAKEUP_TIMEOUT_PRESCALE_SHIFT (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG45_ITP_WAKEUP_TIMEOUT_PRESCALE_MAX  (0x00000003U)

/* CFG_REG46 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG46_TSEQ_QUANTITY_MASK               (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG46_TSEQ_QUANTITY_SHIFT              (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG46_TSEQ_QUANTITY_MAX                (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG46_RESERVED_MASK                    (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG46_RESERVED_SHIFT                   (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG46_RESERVED_MAX                     (0x0000FFFFU)

/* CFG_REG47 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG47_ERDY_TIMEOUT_CNT_MASK            (0x000FFFFFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG47_ERDY_TIMEOUT_CNT_SHIFT           (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG47_ERDY_TIMEOUT_CNT_MAX             (0x000FFFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG47_RESERVED_MASK                    (0xFFF00000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG47_RESERVED_SHIFT                   (0x00000014U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG47_RESERVED_MAX                     (0x00000FFFU)

/* CFG_REG48 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG48_TWTRSTFS_J_CNT_MASK              (0x0003FFFFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG48_TWTRSTFS_J_CNT_SHIFT             (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG48_TWTRSTFS_J_CNT_MAX               (0x0003FFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG48_RESERVED_MASK                    (0xFFFC0000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG48_RESERVED_SHIFT                   (0x00000012U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG48_RESERVED_MAX                     (0x00003FFFU)

/* CFG_REG49 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG49_TUCH_CNT_MASK                    (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG49_TUCH_CNT_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG49_TUCH_CNT_MAX                     (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG49_RESERVED_MASK                    (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG49_RESERVED_SHIFT                   (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG49_RESERVED_MAX                     (0x0000FFFFU)

/* CFG_REG50 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG50_TWAITCHK_CNT_MASK                (0x00000FFFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG50_TWAITCHK_CNT_SHIFT               (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG50_TWAITCHK_CNT_MAX                 (0x00000FFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG50_RESERVED_MASK                    (0xFFFFF000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG50_RESERVED_SHIFT                   (0x0000000CU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG50_RESERVED_MAX                     (0x000FFFFFU)

/* CFG_REG51 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG51_TWTFS_CNT_MASK                   (0x0001FFFFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG51_TWTFS_CNT_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG51_TWTFS_CNT_MAX                    (0x0001FFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG51_RESERVED_MASK                    (0xFFFE0000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG51_RESERVED_SHIFT                   (0x00000011U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG51_RESERVED_MAX                     (0x00007FFFU)

/* CFG_REG52 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG52_TWTREV_CNT_MASK                  (0x0001FFFFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG52_TWTREV_CNT_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG52_TWTREV_CNT_MAX                   (0x0001FFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG52_RESERVED_MASK                    (0xFFFE0000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG52_RESERVED_SHIFT                   (0x00000011U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG52_RESERVED_MAX                     (0x00007FFFU)

/* CFG_REG53 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG53_TWTRSTHS_CNT_MASK                (0x00007FFFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG53_TWTRSTHS_CNT_SHIFT               (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG53_TWTRSTHS_CNT_MAX                 (0x00007FFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG53_RESERVED_MASK                    (0xFFFF8000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG53_RESERVED_SHIFT                   (0x0000000FU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG53_RESERVED_MAX                     (0x0001FFFFU)

/* CFG_REG54 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG54_TWTRSM_CNT_MASK                  (0x0003FFFFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG54_TWTRSM_CNT_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG54_TWTRSM_CNT_MAX                   (0x0003FFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG54_RESERVED_MASK                    (0xFFFC0000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG54_RESERVED_SHIFT                   (0x00000012U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG54_RESERVED_MAX                     (0x00003FFFU)

/* CFG_REG55 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG55_TDRSMUP_CNT_MASK                 (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG55_TDRSMUP_CNT_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG55_TDRSMUP_CNT_MAX                  (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG55_RESERVED_MASK                    (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG55_RESERVED_SHIFT                   (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG55_RESERVED_MAX                     (0x0000FFFFU)

/* CFG_REG56 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG56_TOUTHS_CNT_MASK                  (0x0000003FU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG56_TOUTHS_CNT_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG56_TOUTHS_CNT_MAX                   (0x0000003FU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG56_RESERVED_MASK                    (0xFFFFFFC0U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG56_RESERVED_SHIFT                   (0x00000006U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG56_RESERVED_MAX                     (0x03FFFFFFU)

/* CFG_REG57 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG57_LFPS_DEB_WIDTH_MASK              (0x00000003U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG57_LFPS_DEB_WIDTH_SHIFT             (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG57_LFPS_DEB_WIDTH_MAX               (0x00000003U)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG57_RESERVED_MASK                    (0xFFFFFFFCU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG57_RESERVED_SHIFT                   (0x00000002U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG57_RESERVED_MAX                     (0x3FFFFFFFU)

/* CFG_REG58 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG58_LFPS_GEN_U2EXIT_MASK             (0x0003FFFFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG58_LFPS_GEN_U2EXIT_SHIFT            (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG58_LFPS_GEN_U2EXIT_MAX              (0x0003FFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG58_RESERVED_MASK                    (0xFFFC0000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG58_RESERVED_SHIFT                   (0x00000012U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG58_RESERVED_MAX                     (0x00003FFFU)

/* CFG_REG59 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG59_LFPS_MIN_GEN_U3EXIT_MASK         (0x0000FFFFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG59_LFPS_MIN_GEN_U3EXIT_SHIFT        (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG59_LFPS_MIN_GEN_U3EXIT_MAX          (0x0000FFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG59_RESERVED_MASK                    (0xFFFF0000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG59_RESERVED_SHIFT                   (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG59_RESERVED_MAX                     (0x0000FFFFU)

/* CFG_REG60 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG60_PORT_CONFIG_TIMEOUT_MASK         (0x0000007FU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG60_PORT_CONFIG_TIMEOUT_SHIFT        (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG60_PORT_CONFIG_TIMEOUT_MAX          (0x0000007FU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG60_RESERVED_MASK                    (0xFFFFFF80U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG60_RESERVED_SHIFT                   (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG60_RESERVED_MAX                     (0x01FFFFFFU)

/* CFG_REG61 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG61_LFPS_POL_LFPS_TO_RXEQ_MASK       (0x000007FFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG61_LFPS_POL_LFPS_TO_RXEQ_SHIFT      (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG61_LFPS_POL_LFPS_TO_RXEQ_MAX        (0x000007FFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG61_RESERVED_MASK                    (0xFFFFF800U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG61_RESERVED_SHIFT                   (0x0000000BU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG61_RESERVED_MAX                     (0x001FFFFFU)

/* CFG_REG62 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG62_PHY_TX_LATENCY_MASK              (0x0000003FU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG62_PHY_TX_LATENCY_SHIFT             (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG62_PHY_TX_LATENCY_MAX               (0x0000003FU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG62_RESERVED_MASK                    (0x3FFFFFC0U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG62_RESERVED_SHIFT                   (0x00000006U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG62_RESERVED_MAX                     (0x00FFFFFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG62_PHY_TX_LATENCY_PRESCALE_MASK     (0xC0000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG62_PHY_TX_LATENCY_PRESCALE_SHIFT    (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG62_PHY_TX_LATENCY_PRESCALE_MAX      (0x00000003U)

/* CFG_REG63 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG63_U2_INACTIVITY_TMOUT_MASK         (0x00007FFFU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG63_U2_INACTIVITY_TMOUT_SHIFT        (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG63_U2_INACTIVITY_TMOUT_MAX          (0x00007FFFU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG63_RESERVED_MASK                    (0xFFFF8000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG63_RESERVED_SHIFT                   (0x0000000FU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG63_RESERVED_MAX                     (0x0001FFFFU)

/* CFG_REG64 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG64_TFILTSE0_MASK                    (0x0000007FU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG64_TFILTSE0_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG64_TFILTSE0_MAX                     (0x0000007FU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG64_RESERVED_MASK                    (0xFFFFFF80U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG64_RESERVED_SHIFT                   (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG64_RESERVED_MAX                     (0x01FFFFFFU)

/* CFG_REG65 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG65_TFILT_MASK                       (0x0000007FU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG65_TFILT_SHIFT                      (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG65_TFILT_MAX                        (0x0000007FU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG65_RESERVED_MASK                    (0xFFFF8000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG65_RESERVED_SHIFT                   (0x0000000FU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG65_RESERVED_MAX                     (0x0001FFFFU)

/* CFG_REG66 */

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG66_TWTRSTFS_SE0_MASK                (0x0000007FU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG66_TWTRSTFS_SE0_SHIFT               (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG66_TWTRSTFS_SE0_MAX                 (0x0000007FU)

#define CSL_USB3P0SS_CTRL_DEV_CFG_REG66_RESERVED_MASK                    (0xFFFF8000U)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG66_RESERVED_SHIFT                   (0x0000000FU)
#define CSL_USB3P0SS_CTRL_DEV_CFG_REG66_RESERVED_MAX                     (0x0001FFFFU)

/* DMA_AXI_CTRL */

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_MARPROT_MASK                  (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_MARPROT_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_MARPROT_MAX                   (0x00000007U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_RESERVED0_MASK                (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_RESERVED0_SHIFT               (0x00000003U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_RESERVED0_MAX                 (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_MARCACHE_MASK                 (0x000000F0U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_MARCACHE_SHIFT                (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_MARCACHE_MAX                  (0x0000000FU)

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_MARLOCK_MASK                  (0x00000300U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_MARLOCK_SHIFT                 (0x00000008U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_MARLOCK_MAX                   (0x00000003U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_RESERVED1_MASK                (0x0000FC00U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_RESERVED1_SHIFT               (0x0000000AU)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_RESERVED1_MAX                 (0x0000003FU)

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_MAWPROT_MASK                  (0x00070000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_MAWPROT_SHIFT                 (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_MAWPROT_MAX                   (0x00000007U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_RESERVED2_MASK                (0x00080000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_RESERVED2_SHIFT               (0x00000013U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_RESERVED2_MAX                 (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_MAWCACHE_MASK                 (0x00F00000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_MAWCACHE_SHIFT                (0x00000014U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_MAWCACHE_MAX                  (0x0000000FU)

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_MAWLOCK_MASK                  (0x03000000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_MAWLOCK_SHIFT                 (0x00000018U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_MAWLOCK_MAX                   (0x00000003U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_RESERVED3_MASK                (0xFC000000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_RESERVED3_SHIFT               (0x0000001AU)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL_RESERVED3_MAX                 (0x0000003FU)

/* DMA_AXI_ID */

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_ID_MAW_ID_MASK                     (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_ID_MAW_ID_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_ID_MAW_ID_MAX                      (0x0000001FU)

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_ID_RESERVED0_MASK                  (0x0000FFE0U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_ID_RESERVED0_SHIFT                 (0x00000005U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_ID_RESERVED0_MAX                   (0x000007FFU)

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_ID_MAR_ID_MASK                     (0x001F0000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_ID_MAR_ID_SHIFT                    (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_ID_MAR_ID_MAX                      (0x0000001FU)

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_ID_RESERVED1_MASK                  (0xFFE00000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_ID_RESERVED1_SHIFT                 (0x00000015U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_ID_RESERVED1_MAX                   (0x000007FFU)

/* DMA_AXI_CAP */

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CAP_RESERVED0_MASK                 (0x000FFFFFU)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CAP_RESERVED0_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CAP_RESERVED0_MAX                  (0x000FFFFFU)

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CAP_AXI_DECERR_EN_MASK             (0x00100000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CAP_AXI_DECERR_EN_SHIFT            (0x00000014U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CAP_AXI_DECERR_EN_MAX              (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CAP_AXI_SLVERR_EN_MASK             (0x00200000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CAP_AXI_SLVERR_EN_SHIFT            (0x00000015U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CAP_AXI_SLVERR_EN_MAX              (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CAP_RESERVED1_MASK                 (0x0FC00000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CAP_RESERVED1_SHIFT                (0x00000016U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CAP_RESERVED1_MAX                  (0x0000003FU)

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CAP_AXI_DECERR_MASK                (0x10000000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CAP_AXI_DECERR_SHIFT               (0x0000001CU)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CAP_AXI_DECERR_MAX                 (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CAP_AXI_SLVERR_MASK                (0x20000000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CAP_AXI_SLVERR_SHIFT               (0x0000001DU)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CAP_AXI_SLVERR_MAX                 (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CAP_AXI_IDLE_MASK                  (0x40000000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CAP_AXI_IDLE_SHIFT                 (0x0000001EU)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CAP_AXI_IDLE_MAX                   (0x00000001U)

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CAP_RESERVED2_MASK                 (0x80000000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CAP_RESERVED2_SHIFT                (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CAP_RESERVED2_MAX                  (0x00000001U)

/* DMA_AXI_CTRL0 */

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL0_B_MAX_MASK                   (0x0000000FU)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL0_B_MAX_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL0_B_MAX_MAX                    (0x0000000FU)

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL0_RESERVED_MASK                (0xFFFFFFF0U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL0_RESERVED_SHIFT               (0x00000004U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL0_RESERVED_MAX                 (0x0FFFFFFFU)

/* DMA_AXI_CTRL1 */

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL1_ROT_MASK                     (0x0000001FU)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL1_ROT_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL1_ROT_MAX                      (0x0000001FU)

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL1_RESERVED0_MASK               (0x0000FFE0U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL1_RESERVED0_SHIFT              (0x00000005U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL1_RESERVED0_MAX                (0x000007FFU)

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL1_WOT_MASK                     (0x001F0000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL1_WOT_SHIFT                    (0x00000010U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL1_WOT_MAX                      (0x0000001FU)

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL1_RESERVED1_MASK               (0xFFE00000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL1_RESERVED1_SHIFT              (0x00000015U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL1_RESERVED1_MAX                (0x000007FFU)

/* DMA_AXI_CTRL2 */

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL2_AXI_WTHRES_MASK              (0x0000007FU)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL2_AXI_WTHRES_SHIFT             (0x00000000U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL2_AXI_WTHRES_MAX               (0x0000007FU)

#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL2_RESERVED0_MASK               (0xFFFFFF80U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL2_RESERVED0_SHIFT              (0x00000007U)
#define CSL_USB3P0SS_CTRL_DEV_DMA_AXI_CTRL2_RESERVED0_MAX                (0x01FFFFFFU)

/**************************************************************************
* Hardware Region  : USB2 PHY registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t AFE_TX_REG0;
    volatile uint32_t AFE_TX_REG1;
    volatile uint32_t AFE_TX_REG2;
    volatile uint32_t AFE_TX_REG3;
    volatile uint32_t AFE_TX_REG4;
    volatile uint32_t AFE_TX_REG5;
    volatile uint32_t AFE_TX_REG6;
    volatile uint32_t AFE_TX_REG7;
    volatile uint32_t AFE_TX_REG8;
    volatile uint32_t AFE_TX_REG9;
    volatile uint32_t AFE_TX_REG10;
    volatile uint32_t AFE_TX_REG11;
    volatile uint32_t AFE_TX_REG12;
    volatile uint32_t AFE_RX_REG0;
    volatile uint32_t AFE_RX_REG1;
    volatile uint32_t AFE_RX_REG2;
    volatile uint32_t AFE_RX_REG3;
    volatile uint32_t AFE_RX_REG4;
    volatile uint32_t AFE_RX_REG5;
    volatile uint32_t AFE_RX_REG6;
    volatile uint32_t AFE_TX_REG13;
    volatile uint32_t AFE_TX_REG14;
    volatile uint32_t AFE_RX_REG7;
    volatile uint32_t AFE_RX_REG8;
    volatile uint32_t AFE_UNUSED_REG0;
    volatile uint32_t AFE_UNUSED_REG1;
    volatile uint8_t  Resv_128[24];
    volatile uint32_t AFE_BG_REG0;
    volatile uint32_t AFE_BG_REG1;
    volatile uint32_t AFE_BG_REG2;
    volatile uint32_t AFE_BG_REG3;
    volatile uint32_t AFE_CALIB_REG0;
    volatile uint32_t AFE_BC_REG0;
    volatile uint32_t AFE_BC_REG1;
    volatile uint32_t AFE_BC_REG2;
    volatile uint32_t AFE_BC_REG3;
    volatile uint32_t AFE_BC_REG4;
    volatile uint32_t AFE_BC_REG5;
    volatile uint32_t AFE_BC_REG6;
    volatile uint32_t AFE_PLL_REG0;
    volatile uint32_t AFE_PLL_REG1;
    volatile uint32_t AFE_PLL_REG2;
    volatile uint32_t AFE_PLL_REG3;
    volatile uint32_t AFE_PLL_REG4;
    volatile uint32_t AFE_PLL_REG5;
    volatile uint32_t AFE_BG_REG4;
    volatile uint32_t AFE_CALIB_REG1;
    volatile uint32_t AFE_BC_REG7;
    volatile uint32_t AFE_PLL_REG6;
    volatile uint32_t AFE_UNUSED_REG2;
    volatile uint32_t AFE_UNUSED_REG3;
    volatile uint8_t  Resv_256[32];
    volatile uint32_t PLL_REG0;
    volatile uint32_t PLL_REG1;
    volatile uint32_t PLL_REG2;
    volatile uint32_t PLL_REG3;
    volatile uint32_t PLL_REG4;
    volatile uint32_t PLL_REG5;
    volatile uint32_t PLL_REG6;
    volatile uint32_t PLL_REG7;
    volatile uint32_t PLL_REG8;
    volatile uint32_t PLL_REG9;
    volatile uint32_t PLL_REG10;
    volatile uint32_t PLL_REG11;
    volatile uint32_t PLL_REG12;
    volatile uint32_t PLL_REG13;
    volatile uint32_t PLL_REG14;
    volatile uint32_t PLL_UNUSED_REG0;
    volatile uint32_t PLL_UNUSED_REG1;
    volatile uint32_t PLL_REG15;
    volatile uint32_t PLL_REG16;
    volatile uint32_t PLL_UNUSED_REG2;
    volatile uint8_t  Resv_384[48];
    volatile uint32_t CALIB_REG0;
    volatile uint32_t CALIB_REG1;
    volatile uint32_t BC_REG0;
    volatile uint32_t BC_REG1;
    volatile uint32_t BC_REG2;
    volatile uint32_t BC_REG3;
    volatile uint32_t BC_REG4;
    volatile uint32_t BC_REG5;
    volatile uint32_t BC_REG6;
    volatile uint32_t BC_REG7;
    volatile uint32_t TED_REG0;
    volatile uint32_t TED_REG1;
    volatile uint32_t TED_REG2;
    volatile uint32_t CALIB_REG2;
    volatile uint32_t CALIB_REG3;
    volatile uint32_t BC_REG8;
    volatile uint32_t BC_REG9;
    volatile uint32_t BC_REG10;
    volatile uint32_t BC_REG11;
    volatile uint32_t BC_REG12;
    volatile uint32_t TED_REG3;
    volatile uint32_t TED_REG4;
    volatile uint32_t DIG_UNUSED_REG0;
    volatile uint32_t DIG_UNUSED_REG1;
    volatile uint32_t DIG_UNUSED_REG2;
    volatile uint32_t DIG_UNUSED_REG3;
    volatile uint32_t INTERRUPT_REG1;
    volatile uint32_t INTERRUPT_REG2;
    volatile uint8_t  Resv_512[16];
    volatile uint32_t RX_REG0;
    volatile uint32_t RX_REG1;
    volatile uint32_t TX_REG0;
    volatile uint32_t TX_REG1;
    volatile uint32_t CDR_REG0;
    volatile uint32_t CDR_REG1;
    volatile uint32_t CDR_REG2;
    volatile uint32_t CDR_REG3;
    volatile uint32_t CDR_REG4;
    volatile uint32_t CDR_REG5;
    volatile uint32_t CDR_REG6;
    volatile uint32_t CDR_REG7;
    volatile uint32_t CDR_REG8;
    volatile uint32_t RX_REG2;
    volatile uint32_t RX_REG3;
    volatile uint32_t RX_REG4;
    volatile uint32_t RX_REG5;
    volatile uint32_t RX_REG6;
    volatile uint32_t RX_REG7;
    volatile uint32_t TX_REG2;
    volatile uint32_t TX_REG3;
    volatile uint32_t TX_REG4;
    volatile uint32_t CDR_REG9;
    volatile uint32_t CDR_REG10;
    volatile uint32_t CDR_REG11;
    volatile uint32_t CDR_RE12;
    volatile uint32_t DIG_TXRX_UNUSED_REG0;
    volatile uint32_t DIG_TXRX_UNUSED_REG1;
    volatile uint32_t DIG_TXRX_UNUSED_REG2;
    volatile uint32_t DIG_TXRX_UNUSED_REG3;
    volatile uint8_t  Resv_640[8];
    volatile uint32_t UTMI_REG0;
    volatile uint32_t UTMI_REG1;
    volatile uint32_t UTMI_REG2;
    volatile uint32_t UTMI_REG3;
    volatile uint32_t UTMI_REG4;
    volatile uint32_t UTMI_REG5;
    volatile uint32_t UTMI_REG6;
    volatile uint32_t UTMI_REG7;
    volatile uint32_t UTMI_REG8;
    volatile uint32_t UTMI_REG9;
    volatile uint32_t UTMI_REG10;
    volatile uint32_t UTMI_REG11;
    volatile uint32_t UTMI_REG12;
    volatile uint32_t UTMI_REG13;
    volatile uint32_t UTMI_REG14;
    volatile uint32_t UTMI_REG15;
    volatile uint32_t UTMI_REG16;
    volatile uint32_t UTMI_REG17;
    volatile uint32_t UTMI_REG18;
    volatile uint32_t UTMI_REG19;
    volatile uint32_t UTMI_REG20;
    volatile uint32_t UTMI_REG21;
    volatile uint32_t UTMI_REG22;
    volatile uint32_t UTMI_REG23;
    volatile uint32_t UTMI_REG24;
    volatile uint32_t UTMI_REG25;
    volatile uint32_t UTMI_REG26;
    volatile uint32_t UTMI_REG27;
    volatile uint32_t UTMI_REG28;
    volatile uint32_t UTMI_REG29;
    volatile uint32_t UTMI_REG30;
    volatile uint32_t UTMI_UNUSED_REG0;
    volatile uint32_t UTMI_UNUSED_REG1;
    volatile uint32_t UTMI_UNUSED_REG2;
    volatile uint32_t UTMI_UNUSED_REG3;
    volatile uint32_t UTMI_REG31;
    volatile uint32_t UTMI_REG32;
    volatile uint32_t UTMI_REG33;
    volatile uint32_t UTMI_REG34;
    volatile uint32_t UTMI_REG35;
    volatile uint32_t UTMI_REG36;
    volatile uint32_t UTMI_REG37;
    volatile uint32_t UTMI_REG38;
    volatile uint32_t UTMI_REG39;
    volatile uint32_t UTMI_REG40;
    volatile uint32_t UTMI_REG41;
    volatile uint32_t UTMI_REG42;
    volatile uint32_t UTMI_REG43;
    volatile uint32_t UTMI_REG44;
    volatile uint32_t UTMI_REG45;
    volatile uint32_t UTMI_REG46;
    volatile uint32_t UTMI_REG47;
    volatile uint32_t UTMI_REG48;
    volatile uint32_t UTMI_REG49;
    volatile uint32_t UTMI_REG50;
    volatile uint32_t UTMI_REG51;
    volatile uint32_t UTMI_REG52;
    volatile uint32_t UTMI_REG53;
    volatile uint32_t UTMI_REG54;
    volatile uint32_t UTMI_REG55;
    volatile uint32_t UTMI_REG56;
    volatile uint32_t UTMI_REG57;
    volatile uint32_t UTMI_REG58;
    volatile uint32_t UTMI_REG59;
    volatile uint32_t UTMI_UNUSED_REG6;
    volatile uint32_t UTMI_UNUSED_REG7;
} CSL_usb3p0ss_phy2Regs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_USB3P0SS_PHY2_AFE_TX_REG0                                    (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG1                                    (0x00000004U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG2                                    (0x00000008U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG3                                    (0x0000000CU)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG4                                    (0x00000010U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG5                                    (0x00000014U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG6                                    (0x00000018U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG7                                    (0x0000001CU)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG8                                    (0x00000020U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG9                                    (0x00000024U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG10                                   (0x00000028U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG11                                   (0x0000002CU)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG12                                   (0x00000030U)
#define CSL_USB3P0SS_PHY2_AFE_RX_REG0                                    (0x00000034U)
#define CSL_USB3P0SS_PHY2_AFE_RX_REG1                                    (0x00000038U)
#define CSL_USB3P0SS_PHY2_AFE_RX_REG2                                    (0x0000003CU)
#define CSL_USB3P0SS_PHY2_AFE_RX_REG3                                    (0x00000040U)
#define CSL_USB3P0SS_PHY2_AFE_RX_REG4                                    (0x00000044U)
#define CSL_USB3P0SS_PHY2_AFE_RX_REG5                                    (0x00000048U)
#define CSL_USB3P0SS_PHY2_AFE_RX_REG6                                    (0x0000004CU)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG13                                   (0x00000050U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG14                                   (0x00000054U)
#define CSL_USB3P0SS_PHY2_AFE_RX_REG7                                    (0x00000058U)
#define CSL_USB3P0SS_PHY2_AFE_RX_REG8                                    (0x0000005CU)
#define CSL_USB3P0SS_PHY2_AFE_UNUSED_REG0                                (0x00000060U)
#define CSL_USB3P0SS_PHY2_AFE_UNUSED_REG1                                (0x00000064U)
#define CSL_USB3P0SS_PHY2_AFE_BG_REG0                                    (0x00000080U)
#define CSL_USB3P0SS_PHY2_AFE_BG_REG1                                    (0x00000084U)
#define CSL_USB3P0SS_PHY2_AFE_BG_REG2                                    (0x00000088U)
#define CSL_USB3P0SS_PHY2_AFE_BG_REG3                                    (0x0000008CU)
#define CSL_USB3P0SS_PHY2_AFE_CALIB_REG0                                 (0x00000090U)
#define CSL_USB3P0SS_PHY2_AFE_BC_REG0                                    (0x00000094U)
#define CSL_USB3P0SS_PHY2_AFE_BC_REG1                                    (0x00000098U)
#define CSL_USB3P0SS_PHY2_AFE_BC_REG2                                    (0x0000009CU)
#define CSL_USB3P0SS_PHY2_AFE_BC_REG3                                    (0x000000A0U)
#define CSL_USB3P0SS_PHY2_AFE_BC_REG4                                    (0x000000A4U)
#define CSL_USB3P0SS_PHY2_AFE_BC_REG5                                    (0x000000A8U)
#define CSL_USB3P0SS_PHY2_AFE_BC_REG6                                    (0x000000ACU)
#define CSL_USB3P0SS_PHY2_AFE_PLL_REG0                                   (0x000000B0U)
#define CSL_USB3P0SS_PHY2_AFE_PLL_REG1                                   (0x000000B4U)
#define CSL_USB3P0SS_PHY2_AFE_PLL_REG2                                   (0x000000B8U)
#define CSL_USB3P0SS_PHY2_AFE_PLL_REG3                                   (0x000000BCU)
#define CSL_USB3P0SS_PHY2_AFE_PLL_REG4                                   (0x000000C0U)
#define CSL_USB3P0SS_PHY2_AFE_PLL_REG5                                   (0x000000C4U)
#define CSL_USB3P0SS_PHY2_AFE_BG_REG4                                    (0x000000C8U)
#define CSL_USB3P0SS_PHY2_AFE_CALIB_REG1                                 (0x000000CCU)
#define CSL_USB3P0SS_PHY2_AFE_BC_REG7                                    (0x000000D0U)
#define CSL_USB3P0SS_PHY2_AFE_PLL_REG6                                   (0x000000D4U)
#define CSL_USB3P0SS_PHY2_AFE_UNUSED_REG2                                (0x000000D8U)
#define CSL_USB3P0SS_PHY2_AFE_UNUSED_REG3                                (0x000000DCU)
#define CSL_USB3P0SS_PHY2_PLL_REG0                                       (0x00000100U)
#define CSL_USB3P0SS_PHY2_PLL_REG1                                       (0x00000104U)
#define CSL_USB3P0SS_PHY2_PLL_REG2                                       (0x00000108U)
#define CSL_USB3P0SS_PHY2_PLL_REG3                                       (0x0000010CU)
#define CSL_USB3P0SS_PHY2_PLL_REG4                                       (0x00000110U)
#define CSL_USB3P0SS_PHY2_PLL_REG5                                       (0x00000114U)
#define CSL_USB3P0SS_PHY2_PLL_REG6                                       (0x00000118U)
#define CSL_USB3P0SS_PHY2_PLL_REG7                                       (0x0000011CU)
#define CSL_USB3P0SS_PHY2_PLL_REG8                                       (0x00000120U)
#define CSL_USB3P0SS_PHY2_PLL_REG9                                       (0x00000124U)
#define CSL_USB3P0SS_PHY2_PLL_REG10                                      (0x00000128U)
#define CSL_USB3P0SS_PHY2_PLL_REG11                                      (0x0000012CU)
#define CSL_USB3P0SS_PHY2_PLL_REG12                                      (0x00000130U)
#define CSL_USB3P0SS_PHY2_PLL_REG13                                      (0x00000134U)
#define CSL_USB3P0SS_PHY2_PLL_REG14                                      (0x00000138U)
#define CSL_USB3P0SS_PHY2_PLL_UNUSED_REG0                                (0x0000013CU)
#define CSL_USB3P0SS_PHY2_PLL_UNUSED_REG1                                (0x00000140U)
#define CSL_USB3P0SS_PHY2_PLL_REG15                                      (0x00000144U)
#define CSL_USB3P0SS_PHY2_PLL_REG16                                      (0x00000148U)
#define CSL_USB3P0SS_PHY2_PLL_UNUSED_REG2                                (0x0000014CU)
#define CSL_USB3P0SS_PHY2_CALIB_REG0                                     (0x00000180U)
#define CSL_USB3P0SS_PHY2_CALIB_REG1                                     (0x00000184U)
#define CSL_USB3P0SS_PHY2_BC_REG0                                        (0x00000188U)
#define CSL_USB3P0SS_PHY2_BC_REG1                                        (0x0000018CU)
#define CSL_USB3P0SS_PHY2_BC_REG2                                        (0x00000190U)
#define CSL_USB3P0SS_PHY2_BC_REG3                                        (0x00000194U)
#define CSL_USB3P0SS_PHY2_BC_REG4                                        (0x00000198U)
#define CSL_USB3P0SS_PHY2_BC_REG5                                        (0x0000019CU)
#define CSL_USB3P0SS_PHY2_BC_REG6                                        (0x000001A0U)
#define CSL_USB3P0SS_PHY2_BC_REG7                                        (0x000001A4U)
#define CSL_USB3P0SS_PHY2_TED_REG0                                       (0x000001A8U)
#define CSL_USB3P0SS_PHY2_TED_REG1                                       (0x000001ACU)
#define CSL_USB3P0SS_PHY2_TED_REG2                                       (0x000001B0U)
#define CSL_USB3P0SS_PHY2_CALIB_REG2                                     (0x000001B4U)
#define CSL_USB3P0SS_PHY2_CALIB_REG3                                     (0x000001B8U)
#define CSL_USB3P0SS_PHY2_BC_REG8                                        (0x000001BCU)
#define CSL_USB3P0SS_PHY2_BC_REG9                                        (0x000001C0U)
#define CSL_USB3P0SS_PHY2_BC_REG10                                       (0x000001C4U)
#define CSL_USB3P0SS_PHY2_BC_REG11                                       (0x000001C8U)
#define CSL_USB3P0SS_PHY2_BC_REG12                                       (0x000001CCU)
#define CSL_USB3P0SS_PHY2_TED_REG3                                       (0x000001D0U)
#define CSL_USB3P0SS_PHY2_TED_REG4                                       (0x000001D4U)
#define CSL_USB3P0SS_PHY2_DIG_UNUSED_REG0                                (0x000001D8U)
#define CSL_USB3P0SS_PHY2_DIG_UNUSED_REG1                                (0x000001DCU)
#define CSL_USB3P0SS_PHY2_DIG_UNUSED_REG2                                (0x000001E0U)
#define CSL_USB3P0SS_PHY2_DIG_UNUSED_REG3                                (0x000001E4U)
#define CSL_USB3P0SS_PHY2_INTERRUPT_REG1                                 (0x000001E8U)
#define CSL_USB3P0SS_PHY2_INTERRUPT_REG2                                 (0x000001ECU)
#define CSL_USB3P0SS_PHY2_RX_REG0                                        (0x00000200U)
#define CSL_USB3P0SS_PHY2_RX_REG1                                        (0x00000204U)
#define CSL_USB3P0SS_PHY2_TX_REG0                                        (0x00000208U)
#define CSL_USB3P0SS_PHY2_TX_REG1                                        (0x0000020CU)
#define CSL_USB3P0SS_PHY2_CDR_REG0                                       (0x00000210U)
#define CSL_USB3P0SS_PHY2_CDR_REG1                                       (0x00000214U)
#define CSL_USB3P0SS_PHY2_CDR_REG2                                       (0x00000218U)
#define CSL_USB3P0SS_PHY2_CDR_REG3                                       (0x0000021CU)
#define CSL_USB3P0SS_PHY2_CDR_REG4                                       (0x00000220U)
#define CSL_USB3P0SS_PHY2_CDR_REG5                                       (0x00000224U)
#define CSL_USB3P0SS_PHY2_CDR_REG6                                       (0x00000228U)
#define CSL_USB3P0SS_PHY2_CDR_REG7                                       (0x0000022CU)
#define CSL_USB3P0SS_PHY2_CDR_REG8                                       (0x00000230U)
#define CSL_USB3P0SS_PHY2_RX_REG2                                        (0x00000234U)
#define CSL_USB3P0SS_PHY2_RX_REG3                                        (0x00000238U)
#define CSL_USB3P0SS_PHY2_RX_REG4                                        (0x0000023CU)
#define CSL_USB3P0SS_PHY2_RX_REG5                                        (0x00000240U)
#define CSL_USB3P0SS_PHY2_RX_REG6                                        (0x00000244U)
#define CSL_USB3P0SS_PHY2_RX_REG7                                        (0x00000248U)
#define CSL_USB3P0SS_PHY2_TX_REG2                                        (0x0000024CU)
#define CSL_USB3P0SS_PHY2_TX_REG3                                        (0x00000250U)
#define CSL_USB3P0SS_PHY2_TX_REG4                                        (0x00000254U)
#define CSL_USB3P0SS_PHY2_CDR_REG9                                       (0x00000258U)
#define CSL_USB3P0SS_PHY2_CDR_REG10                                      (0x0000025CU)
#define CSL_USB3P0SS_PHY2_CDR_REG11                                      (0x00000260U)
#define CSL_USB3P0SS_PHY2_CDR_RE12                                       (0x00000264U)
#define CSL_USB3P0SS_PHY2_DIG_TXRX_UNUSED_REG0                           (0x00000268U)
#define CSL_USB3P0SS_PHY2_DIG_TXRX_UNUSED_REG1                           (0x0000026CU)
#define CSL_USB3P0SS_PHY2_DIG_TXRX_UNUSED_REG2                           (0x00000270U)
#define CSL_USB3P0SS_PHY2_DIG_TXRX_UNUSED_REG3                           (0x00000274U)
#define CSL_USB3P0SS_PHY2_UTMI_REG0                                      (0x00000280U)
#define CSL_USB3P0SS_PHY2_UTMI_REG1                                      (0x00000284U)
#define CSL_USB3P0SS_PHY2_UTMI_REG2                                      (0x00000288U)
#define CSL_USB3P0SS_PHY2_UTMI_REG3                                      (0x0000028CU)
#define CSL_USB3P0SS_PHY2_UTMI_REG4                                      (0x00000290U)
#define CSL_USB3P0SS_PHY2_UTMI_REG5                                      (0x00000294U)
#define CSL_USB3P0SS_PHY2_UTMI_REG6                                      (0x00000298U)
#define CSL_USB3P0SS_PHY2_UTMI_REG7                                      (0x0000029CU)
#define CSL_USB3P0SS_PHY2_UTMI_REG8                                      (0x000002A0U)
#define CSL_USB3P0SS_PHY2_UTMI_REG9                                      (0x000002A4U)
#define CSL_USB3P0SS_PHY2_UTMI_REG10                                     (0x000002A8U)
#define CSL_USB3P0SS_PHY2_UTMI_REG11                                     (0x000002ACU)
#define CSL_USB3P0SS_PHY2_UTMI_REG12                                     (0x000002B0U)
#define CSL_USB3P0SS_PHY2_UTMI_REG13                                     (0x000002B4U)
#define CSL_USB3P0SS_PHY2_UTMI_REG14                                     (0x000002B8U)
#define CSL_USB3P0SS_PHY2_UTMI_REG15                                     (0x000002BCU)
#define CSL_USB3P0SS_PHY2_UTMI_REG16                                     (0x000002C0U)
#define CSL_USB3P0SS_PHY2_UTMI_REG17                                     (0x000002C4U)
#define CSL_USB3P0SS_PHY2_UTMI_REG18                                     (0x000002C8U)
#define CSL_USB3P0SS_PHY2_UTMI_REG19                                     (0x000002CCU)
#define CSL_USB3P0SS_PHY2_UTMI_REG20                                     (0x000002D0U)
#define CSL_USB3P0SS_PHY2_UTMI_REG21                                     (0x000002D4U)
#define CSL_USB3P0SS_PHY2_UTMI_REG22                                     (0x000002D8U)
#define CSL_USB3P0SS_PHY2_UTMI_REG23                                     (0x000002DCU)
#define CSL_USB3P0SS_PHY2_UTMI_REG24                                     (0x000002E0U)
#define CSL_USB3P0SS_PHY2_UTMI_REG25                                     (0x000002E4U)
#define CSL_USB3P0SS_PHY2_UTMI_REG26                                     (0x000002E8U)
#define CSL_USB3P0SS_PHY2_UTMI_REG27                                     (0x000002ECU)
#define CSL_USB3P0SS_PHY2_UTMI_REG28                                     (0x000002F0U)
#define CSL_USB3P0SS_PHY2_UTMI_REG29                                     (0x000002F4U)
#define CSL_USB3P0SS_PHY2_UTMI_REG30                                     (0x000002F8U)
#define CSL_USB3P0SS_PHY2_UTMI_UNUSED_REG0                               (0x000002FCU)
#define CSL_USB3P0SS_PHY2_UTMI_UNUSED_REG1                               (0x00000300U)
#define CSL_USB3P0SS_PHY2_UTMI_UNUSED_REG2                               (0x00000304U)
#define CSL_USB3P0SS_PHY2_UTMI_UNUSED_REG3                               (0x00000308U)
#define CSL_USB3P0SS_PHY2_UTMI_REG31                                     (0x0000030CU)
#define CSL_USB3P0SS_PHY2_UTMI_REG32                                     (0x00000310U)
#define CSL_USB3P0SS_PHY2_UTMI_REG33                                     (0x00000314U)
#define CSL_USB3P0SS_PHY2_UTMI_REG34                                     (0x00000318U)
#define CSL_USB3P0SS_PHY2_UTMI_REG35                                     (0x0000031CU)
#define CSL_USB3P0SS_PHY2_UTMI_REG36                                     (0x00000320U)
#define CSL_USB3P0SS_PHY2_UTMI_REG37                                     (0x00000324U)
#define CSL_USB3P0SS_PHY2_UTMI_REG38                                     (0x00000328U)
#define CSL_USB3P0SS_PHY2_UTMI_REG39                                     (0x0000032CU)
#define CSL_USB3P0SS_PHY2_UTMI_REG40                                     (0x00000330U)
#define CSL_USB3P0SS_PHY2_UTMI_REG41                                     (0x00000334U)
#define CSL_USB3P0SS_PHY2_UTMI_REG42                                     (0x00000338U)
#define CSL_USB3P0SS_PHY2_UTMI_REG43                                     (0x0000033CU)
#define CSL_USB3P0SS_PHY2_UTMI_REG44                                     (0x00000340U)
#define CSL_USB3P0SS_PHY2_UTMI_REG45                                     (0x00000344U)
#define CSL_USB3P0SS_PHY2_UTMI_REG46                                     (0x00000348U)
#define CSL_USB3P0SS_PHY2_UTMI_REG47                                     (0x0000034CU)
#define CSL_USB3P0SS_PHY2_UTMI_REG48                                     (0x00000350U)
#define CSL_USB3P0SS_PHY2_UTMI_REG49                                     (0x00000354U)
#define CSL_USB3P0SS_PHY2_UTMI_REG50                                     (0x00000358U)
#define CSL_USB3P0SS_PHY2_UTMI_REG51                                     (0x0000035CU)
#define CSL_USB3P0SS_PHY2_UTMI_REG52                                     (0x00000360U)
#define CSL_USB3P0SS_PHY2_UTMI_REG53                                     (0x00000364U)
#define CSL_USB3P0SS_PHY2_UTMI_REG54                                     (0x00000368U)
#define CSL_USB3P0SS_PHY2_UTMI_REG55                                     (0x0000036CU)
#define CSL_USB3P0SS_PHY2_UTMI_REG56                                     (0x00000370U)
#define CSL_USB3P0SS_PHY2_UTMI_REG57                                     (0x00000374U)
#define CSL_USB3P0SS_PHY2_UTMI_REG58                                     (0x00000378U)
#define CSL_USB3P0SS_PHY2_UTMI_REG59                                     (0x0000037CU)
#define CSL_USB3P0SS_PHY2_UTMI_UNUSED_REG6                               (0x00000380U)
#define CSL_USB3P0SS_PHY2_UTMI_UNUSED_REG7                               (0x00000384U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* AFE_TX_REG0 */

#define CSL_USB3P0SS_PHY2_AFE_TX_REG0_BF_0_MASK                          (0x00000001U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG0_BF_0_SHIFT                         (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG0_BF_0_MAX                           (0x00000001U)

#define CSL_USB3P0SS_PHY2_AFE_TX_REG0_BF_1_MASK                          (0x00000002U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG0_BF_1_SHIFT                         (0x00000001U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG0_BF_1_MAX                           (0x00000001U)

#define CSL_USB3P0SS_PHY2_AFE_TX_REG0_TX_ANA_REG0_MASK                   (0x00000080U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG0_TX_ANA_REG0_SHIFT                  (0x00000007U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG0_TX_ANA_REG0_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_AFE_TX_REG0_BF_6_2_MASK                        (0x0000007CU)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG0_BF_6_2_SHIFT                       (0x00000002U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG0_BF_6_2_MAX                         (0x0000001FU)

/* AFE_TX_REG1 */

#define CSL_USB3P0SS_PHY2_AFE_TX_REG1_BF_0_MASK                          (0x00000001U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG1_BF_0_SHIFT                         (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG1_BF_0_MAX                           (0x00000001U)

#define CSL_USB3P0SS_PHY2_AFE_TX_REG1_BF_6_1_MASK                        (0x0000007EU)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG1_BF_6_1_SHIFT                       (0x00000001U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG1_BF_6_1_MAX                         (0x0000003FU)

#define CSL_USB3P0SS_PHY2_AFE_TX_REG1_TX_ANA_REG1_MASK                   (0x00000080U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG1_TX_ANA_REG1_SHIFT                  (0x00000007U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG1_TX_ANA_REG1_MAX                    (0x00000001U)

/* AFE_TX_REG2 */

#define CSL_USB3P0SS_PHY2_AFE_TX_REG2_BF_0_MASK                          (0x00000001U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG2_BF_0_SHIFT                         (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG2_BF_0_MAX                           (0x00000001U)

#define CSL_USB3P0SS_PHY2_AFE_TX_REG2_BF_6_MASK                          (0x00000040U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG2_BF_6_SHIFT                         (0x00000006U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG2_BF_6_MAX                           (0x00000001U)

#define CSL_USB3P0SS_PHY2_AFE_TX_REG2_BF_5_1_MASK                        (0x0000003EU)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG2_BF_5_1_SHIFT                       (0x00000001U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG2_BF_5_1_MAX                         (0x0000001FU)

#define CSL_USB3P0SS_PHY2_AFE_TX_REG2_TX_ANA_REG2_MASK                   (0x00000080U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG2_TX_ANA_REG2_SHIFT                  (0x00000007U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG2_TX_ANA_REG2_MAX                    (0x00000001U)

/* AFE_TX_REG3 */

#define CSL_USB3P0SS_PHY2_AFE_TX_REG3_TX_ANA_REG3_MASK                   (0x000000E0U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG3_TX_ANA_REG3_SHIFT                  (0x00000005U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG3_TX_ANA_REG3_MAX                    (0x00000007U)

#define CSL_USB3P0SS_PHY2_AFE_TX_REG3_BF_0_MASK                          (0x00000001U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG3_BF_0_SHIFT                         (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG3_BF_0_MAX                           (0x00000001U)

#define CSL_USB3P0SS_PHY2_AFE_TX_REG3_BF_4_1_MASK                        (0x0000001EU)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG3_BF_4_1_SHIFT                       (0x00000001U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG3_BF_4_1_MAX                         (0x0000000FU)

/* AFE_TX_REG4 */

#define CSL_USB3P0SS_PHY2_AFE_TX_REG4_BF_0_MASK                          (0x00000001U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG4_BF_0_SHIFT                         (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG4_BF_0_MAX                           (0x00000001U)

#define CSL_USB3P0SS_PHY2_AFE_TX_REG4_BF_4_1_MASK                        (0x0000001EU)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG4_BF_4_1_SHIFT                       (0x00000001U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG4_BF_4_1_MAX                         (0x0000000FU)

#define CSL_USB3P0SS_PHY2_AFE_TX_REG4_BF_6_MASK                          (0x00000040U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG4_BF_6_SHIFT                         (0x00000006U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG4_BF_6_MAX                           (0x00000001U)

#define CSL_USB3P0SS_PHY2_AFE_TX_REG4_BF_5_MASK                          (0x00000020U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG4_BF_5_SHIFT                         (0x00000005U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG4_BF_5_MAX                           (0x00000001U)

#define CSL_USB3P0SS_PHY2_AFE_TX_REG4_TX_ANA_REG4_MASK                   (0x00000080U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG4_TX_ANA_REG4_SHIFT                  (0x00000007U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG4_TX_ANA_REG4_MAX                    (0x00000001U)

/* AFE_TX_REG5 */

#define CSL_USB3P0SS_PHY2_AFE_TX_REG5_BF_0_MASK                          (0x00000001U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG5_BF_0_SHIFT                         (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG5_BF_0_MAX                           (0x00000001U)

#define CSL_USB3P0SS_PHY2_AFE_TX_REG5_AFE_TX_REG5_MASK                   (0x00000080U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG5_AFE_TX_REG5_SHIFT                  (0x00000007U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG5_AFE_TX_REG5_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_AFE_TX_REG5_BF_6_1_MASK                        (0x0000007EU)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG5_BF_6_1_SHIFT                       (0x00000001U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG5_BF_6_1_MAX                         (0x0000003FU)

/* AFE_TX_REG6 */

#define CSL_USB3P0SS_PHY2_AFE_TX_REG6_TX_ANA_REG6_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG6_TX_ANA_REG6_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG6_TX_ANA_REG6_MAX                    (0x000000FFU)

/* AFE_TX_REG7 */

#define CSL_USB3P0SS_PHY2_AFE_TX_REG7_TX_ANA_REG7_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG7_TX_ANA_REG7_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG7_TX_ANA_REG7_MAX                    (0x000000FFU)

/* AFE_TX_REG8 */

#define CSL_USB3P0SS_PHY2_AFE_TX_REG8_TX_ANA_REG8_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG8_TX_ANA_REG8_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG8_TX_ANA_REG8_MAX                    (0x000000FFU)

/* AFE_TX_REG9 */

#define CSL_USB3P0SS_PHY2_AFE_TX_REG9_TX_ANA_REG9_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG9_TX_ANA_REG9_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG9_TX_ANA_REG9_MAX                    (0x000000FFU)

/* AFE_TX_REG10 */

#define CSL_USB3P0SS_PHY2_AFE_TX_REG10_TX_ANA_REG10_MASK                 (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG10_TX_ANA_REG10_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG10_TX_ANA_REG10_MAX                  (0x000000FFU)

/* AFE_TX_REG11 */

#define CSL_USB3P0SS_PHY2_AFE_TX_REG11_TX_ANA_REG11_MASK                 (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG11_TX_ANA_REG11_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG11_TX_ANA_REG11_MAX                  (0x000000FFU)

/* AFE_TX_REG12 */

#define CSL_USB3P0SS_PHY2_AFE_TX_REG12_BF_2_MASK                         (0x00000004U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG12_BF_2_SHIFT                        (0x00000002U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG12_BF_2_MAX                          (0x00000001U)

#define CSL_USB3P0SS_PHY2_AFE_TX_REG12_BF_3_MASK                         (0x00000008U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG12_BF_3_SHIFT                        (0x00000003U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG12_BF_3_MAX                          (0x00000001U)

#define CSL_USB3P0SS_PHY2_AFE_TX_REG12_BF_6_MASK                         (0x00000040U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG12_BF_6_SHIFT                        (0x00000006U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG12_BF_6_MAX                          (0x00000001U)

#define CSL_USB3P0SS_PHY2_AFE_TX_REG12_TX_ANA_REG12_MASK                 (0x00000080U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG12_TX_ANA_REG12_SHIFT                (0x00000007U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG12_TX_ANA_REG12_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_AFE_TX_REG12_BF_5_MASK                         (0x00000020U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG12_BF_5_SHIFT                        (0x00000005U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG12_BF_5_MAX                          (0x00000001U)

#define CSL_USB3P0SS_PHY2_AFE_TX_REG12_BF_1_0_MASK                       (0x00000003U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG12_BF_1_0_SHIFT                      (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG12_BF_1_0_MAX                        (0x00000003U)

#define CSL_USB3P0SS_PHY2_AFE_TX_REG12_BF_4_MASK                         (0x00000010U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG12_BF_4_SHIFT                        (0x00000004U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG12_BF_4_MAX                          (0x00000001U)

/* AFE_RX_REG0 */

#define CSL_USB3P0SS_PHY2_AFE_RX_REG0_RX_ANA_REG0_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_RX_REG0_RX_ANA_REG0_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_RX_REG0_RX_ANA_REG0_MAX                    (0x000000FFU)

/* AFE_RX_REG1 */

#define CSL_USB3P0SS_PHY2_AFE_RX_REG1_RX_ANA_REG1_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_RX_REG1_RX_ANA_REG1_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_RX_REG1_RX_ANA_REG1_MAX                    (0x000000FFU)

/* AFE_RX_REG2 */

#define CSL_USB3P0SS_PHY2_AFE_RX_REG2_RX_ANA_REG2_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_RX_REG2_RX_ANA_REG2_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_RX_REG2_RX_ANA_REG2_MAX                    (0x000000FFU)

/* AFE_RX_REG3 */

#define CSL_USB3P0SS_PHY2_AFE_RX_REG3_RX_ANA_REG3_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_RX_REG3_RX_ANA_REG3_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_RX_REG3_RX_ANA_REG3_MAX                    (0x000000FFU)

/* AFE_RX_REG4 */

#define CSL_USB3P0SS_PHY2_AFE_RX_REG4_RX_ANA_REG4_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_RX_REG4_RX_ANA_REG4_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_RX_REG4_RX_ANA_REG4_MAX                    (0x000000FFU)

/* AFE_RX_REG5 */

#define CSL_USB3P0SS_PHY2_AFE_RX_REG5_RX_ANA_REG5_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_RX_REG5_RX_ANA_REG5_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_RX_REG5_RX_ANA_REG5_MAX                    (0x000000FFU)

/* AFE_RX_REG6 */

#define CSL_USB3P0SS_PHY2_AFE_RX_REG6_RX_ANA_REG6_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_RX_REG6_RX_ANA_REG6_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_RX_REG6_RX_ANA_REG6_MAX                    (0x000000FFU)

/* AFE_TX_REG13 */

#define CSL_USB3P0SS_PHY2_AFE_TX_REG13_TX_ANA_REG13_MASK                 (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG13_TX_ANA_REG13_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG13_TX_ANA_REG13_MAX                  (0x000000FFU)

/* AFE_TX_REG14 */

#define CSL_USB3P0SS_PHY2_AFE_TX_REG14_TX_ANA_REG14_MASK                 (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG14_TX_ANA_REG14_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_TX_REG14_TX_ANA_REG14_MAX                  (0x000000FFU)

/* AFE_RX_REG7 */

#define CSL_USB3P0SS_PHY2_AFE_RX_REG7_RX_ANA_REG7_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_RX_REG7_RX_ANA_REG7_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_RX_REG7_RX_ANA_REG7_MAX                    (0x000000FFU)

/* AFE_RX_REG8 */

#define CSL_USB3P0SS_PHY2_AFE_RX_REG8_RX_ANA_REG8_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_RX_REG8_RX_ANA_REG8_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_RX_REG8_RX_ANA_REG8_MAX                    (0x000000FFU)

/* AFE_UNUSED_REG0 */

#define CSL_USB3P0SS_PHY2_AFE_UNUSED_REG0_AFE_UNUSED_REG0_MASK           (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_UNUSED_REG0_AFE_UNUSED_REG0_SHIFT          (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_UNUSED_REG0_AFE_UNUSED_REG0_MAX            (0x000000FFU)

/* AFE_UNUSED_REG1 */

#define CSL_USB3P0SS_PHY2_AFE_UNUSED_REG1_AFE_UNUSED_REG1_MASK           (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_UNUSED_REG1_AFE_UNUSED_REG1_SHIFT          (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_UNUSED_REG1_AFE_UNUSED_REG1_MAX            (0x000000FFU)

/* AFE_BG_REG0 */

#define CSL_USB3P0SS_PHY2_AFE_BG_REG0_BG_ANA_REG0_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_BG_REG0_BG_ANA_REG0_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_BG_REG0_BG_ANA_REG0_MAX                    (0x000000FFU)

/* AFE_BG_REG1 */

#define CSL_USB3P0SS_PHY2_AFE_BG_REG1_BG_ANA_REG1_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_BG_REG1_BG_ANA_REG1_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_BG_REG1_BG_ANA_REG1_MAX                    (0x000000FFU)

/* AFE_BG_REG2 */

#define CSL_USB3P0SS_PHY2_AFE_BG_REG2_BG_ANA_REG2_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_BG_REG2_BG_ANA_REG2_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_BG_REG2_BG_ANA_REG2_MAX                    (0x000000FFU)

/* AFE_BG_REG3 */

#define CSL_USB3P0SS_PHY2_AFE_BG_REG3_BG_ANA_REG3_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_BG_REG3_BG_ANA_REG3_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_BG_REG3_BG_ANA_REG3_MAX                    (0x000000FFU)

/* AFE_CALIB_REG0 */

#define CSL_USB3P0SS_PHY2_AFE_CALIB_REG0_CALIB_ANA_REG0_MASK             (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_CALIB_REG0_CALIB_ANA_REG0_SHIFT            (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_CALIB_REG0_CALIB_ANA_REG0_MAX              (0x000000FFU)

/* AFE_BC_REG0 */

#define CSL_USB3P0SS_PHY2_AFE_BC_REG0_BC_ANA_REG0_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_BC_REG0_BC_ANA_REG0_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_BC_REG0_BC_ANA_REG0_MAX                    (0x000000FFU)

/* AFE_BC_REG1 */

#define CSL_USB3P0SS_PHY2_AFE_BC_REG1_BC_ANA_REG1_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_BC_REG1_BC_ANA_REG1_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_BC_REG1_BC_ANA_REG1_MAX                    (0x000000FFU)

/* AFE_BC_REG2 */

#define CSL_USB3P0SS_PHY2_AFE_BC_REG2_BC_ANA_REG2_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_BC_REG2_BC_ANA_REG2_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_BC_REG2_BC_ANA_REG2_MAX                    (0x000000FFU)

/* AFE_BC_REG3 */

#define CSL_USB3P0SS_PHY2_AFE_BC_REG3_BC_ANA_REG3_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_BC_REG3_BC_ANA_REG3_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_BC_REG3_BC_ANA_REG3_MAX                    (0x000000FFU)

/* AFE_BC_REG4 */

#define CSL_USB3P0SS_PHY2_AFE_BC_REG4_BC_ANA_REG4_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_BC_REG4_BC_ANA_REG4_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_BC_REG4_BC_ANA_REG4_MAX                    (0x000000FFU)

/* AFE_BC_REG5 */

#define CSL_USB3P0SS_PHY2_AFE_BC_REG5_BC_ANA_REG5_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_BC_REG5_BC_ANA_REG5_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_BC_REG5_BC_ANA_REG5_MAX                    (0x000000FFU)

/* AFE_BC_REG6 */

#define CSL_USB3P0SS_PHY2_AFE_BC_REG6_BC_ANA_REG6_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_BC_REG6_BC_ANA_REG6_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_BC_REG6_BC_ANA_REG6_MAX                    (0x000000FFU)

/* AFE_PLL_REG0 */

#define CSL_USB3P0SS_PHY2_AFE_PLL_REG0_AFE_PLL_REG0_MASK                 (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_PLL_REG0_AFE_PLL_REG0_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_PLL_REG0_AFE_PLL_REG0_MAX                  (0x000000FFU)

/* AFE_PLL_REG1 */

#define CSL_USB3P0SS_PHY2_AFE_PLL_REG1_AFE_PLL_REG1_MASK                 (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_PLL_REG1_AFE_PLL_REG1_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_PLL_REG1_AFE_PLL_REG1_MAX                  (0x000000FFU)

/* AFE_PLL_REG2 */

#define CSL_USB3P0SS_PHY2_AFE_PLL_REG2_AFE_PLL_REG2_MASK                 (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_PLL_REG2_AFE_PLL_REG2_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_PLL_REG2_AFE_PLL_REG2_MAX                  (0x000000FFU)

/* AFE_PLL_REG3 */

#define CSL_USB3P0SS_PHY2_AFE_PLL_REG3_AFE_PLL_REG3_MASK                 (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_PLL_REG3_AFE_PLL_REG3_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_PLL_REG3_AFE_PLL_REG3_MAX                  (0x000000FFU)

/* AFE_PLL_REG4 */

#define CSL_USB3P0SS_PHY2_AFE_PLL_REG4_AFE_PLL_REG4_MASK                 (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_PLL_REG4_AFE_PLL_REG4_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_PLL_REG4_AFE_PLL_REG4_MAX                  (0x000000FFU)

/* AFE_PLL_REG5 */

#define CSL_USB3P0SS_PHY2_AFE_PLL_REG5_AFE_PLL_REG5_MASK                 (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_PLL_REG5_AFE_PLL_REG5_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_PLL_REG5_AFE_PLL_REG5_MAX                  (0x000000FFU)

/* AFE_BG_REG4 */

#define CSL_USB3P0SS_PHY2_AFE_BG_REG4_BG_ANA_REG4_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_BG_REG4_BG_ANA_REG4_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_BG_REG4_BG_ANA_REG4_MAX                    (0x000000FFU)

/* AFE_CALIB_REG1 */

#define CSL_USB3P0SS_PHY2_AFE_CALIB_REG1_CALIB_ANA_REG1_MASK             (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_CALIB_REG1_CALIB_ANA_REG1_SHIFT            (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_CALIB_REG1_CALIB_ANA_REG1_MAX              (0x000000FFU)

/* AFE_BC_REG7 */

#define CSL_USB3P0SS_PHY2_AFE_BC_REG7_BC_ANA_REG7_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_BC_REG7_BC_ANA_REG7_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_BC_REG7_BC_ANA_REG7_MAX                    (0x000000FFU)

/* AFE_PLL_REG6 */

#define CSL_USB3P0SS_PHY2_AFE_PLL_REG6_PLL_ANA_REG6_MASK                 (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_PLL_REG6_PLL_ANA_REG6_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_PLL_REG6_PLL_ANA_REG6_MAX                  (0x000000FFU)

/* AFE_UNUSED_REG2 */

#define CSL_USB3P0SS_PHY2_AFE_UNUSED_REG2_UNUSED_MASK                    (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_UNUSED_REG2_UNUSED_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_UNUSED_REG2_UNUSED_MAX                     (0x000000FFU)

/* AFE_UNUSED_REG3 */

#define CSL_USB3P0SS_PHY2_AFE_UNUSED_REG3_UNUSED_MASK                    (0x000000FFU)
#define CSL_USB3P0SS_PHY2_AFE_UNUSED_REG3_UNUSED_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_PHY2_AFE_UNUSED_REG3_UNUSED_MAX                     (0x000000FFU)

/* PLL_REG0 */

#define CSL_USB3P0SS_PHY2_PLL_REG0_INITIAL_WAIT_TIME_MASK                (0x000000FFU)
#define CSL_USB3P0SS_PHY2_PLL_REG0_INITIAL_WAIT_TIME_SHIFT               (0x00000000U)
#define CSL_USB3P0SS_PHY2_PLL_REG0_INITIAL_WAIT_TIME_MAX                 (0x000000FFU)

/* PLL_REG1 */

#define CSL_USB3P0SS_PHY2_PLL_REG1_FBDIV_EN_MASK                         (0x00000002U)
#define CSL_USB3P0SS_PHY2_PLL_REG1_FBDIV_EN_SHIFT                        (0x00000001U)
#define CSL_USB3P0SS_PHY2_PLL_REG1_FBDIV_EN_MAX                          (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG1_RST_FDBK_DIV_DELAY_VALUE_MASK         (0x000000F8U)
#define CSL_USB3P0SS_PHY2_PLL_REG1_RST_FDBK_DIV_DELAY_VALUE_SHIFT        (0x00000003U)
#define CSL_USB3P0SS_PHY2_PLL_REG1_RST_FDBK_DIV_DELAY_VALUE_MAX          (0x0000001FU)

#define CSL_USB3P0SS_PHY2_PLL_REG1_INITIAL_WAIT_TIME_EN_MASK             (0x00000001U)
#define CSL_USB3P0SS_PHY2_PLL_REG1_INITIAL_WAIT_TIME_EN_SHIFT            (0x00000000U)
#define CSL_USB3P0SS_PHY2_PLL_REG1_INITIAL_WAIT_TIME_EN_MAX              (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG1_RST_FDBK_DIV_DELAY_EN_MASK            (0x00000004U)
#define CSL_USB3P0SS_PHY2_PLL_REG1_RST_FDBK_DIV_DELAY_EN_SHIFT           (0x00000002U)
#define CSL_USB3P0SS_PHY2_PLL_REG1_RST_FDBK_DIV_DELAY_EN_MAX             (0x00000001U)

/* PLL_REG2 */

#define CSL_USB3P0SS_PHY2_PLL_REG2_VCO_SETTLING_TIME_MASK                (0x0000003FU)
#define CSL_USB3P0SS_PHY2_PLL_REG2_VCO_SETTLING_TIME_SHIFT               (0x00000000U)
#define CSL_USB3P0SS_PHY2_PLL_REG2_VCO_SETTLING_TIME_MAX                 (0x0000003FU)

#define CSL_USB3P0SS_PHY2_PLL_REG2_VCO_SETTLING_TIME_EN_MASK             (0x00000040U)
#define CSL_USB3P0SS_PHY2_PLL_REG2_VCO_SETTLING_TIME_EN_SHIFT            (0x00000006U)
#define CSL_USB3P0SS_PHY2_PLL_REG2_VCO_SETTLING_TIME_EN_MAX              (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG2_UNUSED_MASK                           (0x00000080U)
#define CSL_USB3P0SS_PHY2_PLL_REG2_UNUSED_SHIFT                          (0x00000007U)
#define CSL_USB3P0SS_PHY2_PLL_REG2_UNUSED_MAX                            (0x00000001U)

/* PLL_REG3 */

#define CSL_USB3P0SS_PHY2_PLL_REG3_FBDIV_VALUE_MASK                      (0x000000FFU)
#define CSL_USB3P0SS_PHY2_PLL_REG3_FBDIV_VALUE_SHIFT                     (0x00000000U)
#define CSL_USB3P0SS_PHY2_PLL_REG3_FBDIV_VALUE_MAX                       (0x000000FFU)

/* PLL_REG4 */

#define CSL_USB3P0SS_PHY2_PLL_REG4_PD_PFD_VALUE_MASK                     (0x00000020U)
#define CSL_USB3P0SS_PHY2_PLL_REG4_PD_PFD_VALUE_SHIFT                    (0x00000005U)
#define CSL_USB3P0SS_PHY2_PLL_REG4_PD_PFD_VALUE_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG4_COARSEDONE_VALUE_MASK                 (0x00000001U)
#define CSL_USB3P0SS_PHY2_PLL_REG4_COARSEDONE_VALUE_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_PHY2_PLL_REG4_COARSEDONE_VALUE_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG4_PLL_LOCK_VALUE_MASK                   (0x00000004U)
#define CSL_USB3P0SS_PHY2_PLL_REG4_PLL_LOCK_VALUE_SHIFT                  (0x00000002U)
#define CSL_USB3P0SS_PHY2_PLL_REG4_PLL_LOCK_VALUE_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG4_PLL_LOCK_TIME_15_MASK                 (0x00000040U)
#define CSL_USB3P0SS_PHY2_PLL_REG4_PLL_LOCK_TIME_15_SHIFT                (0x00000006U)
#define CSL_USB3P0SS_PHY2_PLL_REG4_PLL_LOCK_TIME_15_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG4_PLL_LOCK_EN_MASK                      (0x00000008U)
#define CSL_USB3P0SS_PHY2_PLL_REG4_PLL_LOCK_EN_SHIFT                     (0x00000003U)
#define CSL_USB3P0SS_PHY2_PLL_REG4_PLL_LOCK_EN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG4_PD_PFD_EN_MASK                        (0x00000010U)
#define CSL_USB3P0SS_PHY2_PLL_REG4_PD_PFD_EN_SHIFT                       (0x00000004U)
#define CSL_USB3P0SS_PHY2_PLL_REG4_PD_PFD_EN_MAX                         (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG4_UNUSED_MASK                           (0x00000080U)
#define CSL_USB3P0SS_PHY2_PLL_REG4_UNUSED_SHIFT                          (0x00000007U)
#define CSL_USB3P0SS_PHY2_PLL_REG4_UNUSED_MAX                            (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG4_COARSEDONE_EN_MASK                    (0x00000002U)
#define CSL_USB3P0SS_PHY2_PLL_REG4_COARSEDONE_EN_SHIFT                   (0x00000001U)
#define CSL_USB3P0SS_PHY2_PLL_REG4_COARSEDONE_EN_MAX                     (0x00000001U)

/* PLL_REG5 */

#define CSL_USB3P0SS_PHY2_PLL_REG5_STARTLOOP_EN_4_0_MASK                 (0x00000080U)
#define CSL_USB3P0SS_PHY2_PLL_REG5_STARTLOOP_EN_4_0_SHIFT                (0x00000007U)
#define CSL_USB3P0SS_PHY2_PLL_REG5_STARTLOOP_EN_4_0_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG5_STARTLOOP_5_MASK                      (0x00000020U)
#define CSL_USB3P0SS_PHY2_PLL_REG5_STARTLOOP_5_SHIFT                     (0x00000005U)
#define CSL_USB3P0SS_PHY2_PLL_REG5_STARTLOOP_5_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG5_STARTLOOP_4_0_MASK                    (0x0000001FU)
#define CSL_USB3P0SS_PHY2_PLL_REG5_STARTLOOP_4_0_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_PHY2_PLL_REG5_STARTLOOP_4_0_MAX                     (0x0000001FU)

#define CSL_USB3P0SS_PHY2_PLL_REG5_STARTLOOP_EN_5_MASK                   (0x00000040U)
#define CSL_USB3P0SS_PHY2_PLL_REG5_STARTLOOP_EN_5_SHIFT                  (0x00000006U)
#define CSL_USB3P0SS_PHY2_PLL_REG5_STARTLOOP_EN_5_MAX                    (0x00000001U)

/* PLL_REG6 */

#define CSL_USB3P0SS_PHY2_PLL_REG6_RST_FDBK_DIV_VALUE_MASK               (0x00000002U)
#define CSL_USB3P0SS_PHY2_PLL_REG6_RST_FDBK_DIV_VALUE_SHIFT              (0x00000001U)
#define CSL_USB3P0SS_PHY2_PLL_REG6_RST_FDBK_DIV_VALUE_MAX                (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG6_VCO_CNT_WINDOW_EN_MASK                (0x00000004U)
#define CSL_USB3P0SS_PHY2_PLL_REG6_VCO_CNT_WINDOW_EN_SHIFT               (0x00000002U)
#define CSL_USB3P0SS_PHY2_PLL_REG6_VCO_CNT_WINDOW_EN_MAX                 (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG6_VCO_CNT_WINDOW_VALUE_MASK             (0x00000008U)
#define CSL_USB3P0SS_PHY2_PLL_REG6_VCO_CNT_WINDOW_VALUE_SHIFT            (0x00000003U)
#define CSL_USB3P0SS_PHY2_PLL_REG6_VCO_CNT_WINDOW_VALUE_MAX              (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG6_BIG_JUMP_EN_MASK                      (0x00000010U)
#define CSL_USB3P0SS_PHY2_PLL_REG6_BIG_JUMP_EN_SHIFT                     (0x00000004U)
#define CSL_USB3P0SS_PHY2_PLL_REG6_BIG_JUMP_EN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG6_LSB_ERROR_0P5_MASK                    (0x00000020U)
#define CSL_USB3P0SS_PHY2_PLL_REG6_LSB_ERROR_0P5_SHIFT                   (0x00000005U)
#define CSL_USB3P0SS_PHY2_PLL_REG6_LSB_ERROR_0P5_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG6_COARSE_CODE_SEL_MASK                  (0x00000040U)
#define CSL_USB3P0SS_PHY2_PLL_REG6_COARSE_CODE_SEL_SHIFT                 (0x00000006U)
#define CSL_USB3P0SS_PHY2_PLL_REG6_COARSE_CODE_SEL_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG6_RST_FDBK_DIV_EN_MASK                  (0x00000001U)
#define CSL_USB3P0SS_PHY2_PLL_REG6_RST_FDBK_DIV_EN_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_PHY2_PLL_REG6_RST_FDBK_DIV_EN_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG6_UNUSED_MASK                           (0x00000080U)
#define CSL_USB3P0SS_PHY2_PLL_REG6_UNUSED_SHIFT                          (0x00000007U)
#define CSL_USB3P0SS_PHY2_PLL_REG6_UNUSED_MAX                            (0x00000001U)

/* PLL_REG7 */

#define CSL_USB3P0SS_PHY2_PLL_REG7_REFCLK_SEL_EN_MASK                    (0x00000001U)
#define CSL_USB3P0SS_PHY2_PLL_REG7_REFCLK_SEL_EN_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_PHY2_PLL_REG7_REFCLK_SEL_EN_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG7_REFCLK_SEL_MASK                       (0x0000001EU)
#define CSL_USB3P0SS_PHY2_PLL_REG7_REFCLK_SEL_SHIFT                      (0x00000001U)
#define CSL_USB3P0SS_PHY2_PLL_REG7_REFCLK_SEL_MAX                        (0x0000000FU)

#define CSL_USB3P0SS_PHY2_PLL_REG7_UNUSED_MASK                           (0x000000E0U)
#define CSL_USB3P0SS_PHY2_PLL_REG7_UNUSED_SHIFT                          (0x00000005U)
#define CSL_USB3P0SS_PHY2_PLL_REG7_UNUSED_MAX                            (0x00000007U)

/* PLL_REG8 */

#define CSL_USB3P0SS_PHY2_PLL_REG8_COARSE_CODE_MASK                      (0x000000FFU)
#define CSL_USB3P0SS_PHY2_PLL_REG8_COARSE_CODE_SHIFT                     (0x00000000U)
#define CSL_USB3P0SS_PHY2_PLL_REG8_COARSE_CODE_MAX                       (0x000000FFU)

/* PLL_REG9 */

#define CSL_USB3P0SS_PHY2_PLL_REG9_V2I_CODE_MASK                         (0x0000003FU)
#define CSL_USB3P0SS_PHY2_PLL_REG9_V2I_CODE_SHIFT                        (0x00000000U)
#define CSL_USB3P0SS_PHY2_PLL_REG9_V2I_CODE_MAX                          (0x0000003FU)

#define CSL_USB3P0SS_PHY2_PLL_REG9_V2I_CODE_EN_MASK                      (0x00000040U)
#define CSL_USB3P0SS_PHY2_PLL_REG9_V2I_CODE_EN_SHIFT                     (0x00000006U)
#define CSL_USB3P0SS_PHY2_PLL_REG9_V2I_CODE_EN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG9_COARSE_CODE_MASK                      (0x00000080U)
#define CSL_USB3P0SS_PHY2_PLL_REG9_COARSE_CODE_SHIFT                     (0x00000007U)
#define CSL_USB3P0SS_PHY2_PLL_REG9_COARSE_CODE_MAX                       (0x00000001U)

/* PLL_REG10 */

#define CSL_USB3P0SS_PHY2_PLL_REG10_IPDIV_VALUE_MASK                     (0x0000007CU)
#define CSL_USB3P0SS_PHY2_PLL_REG10_IPDIV_VALUE_SHIFT                    (0x00000002U)
#define CSL_USB3P0SS_PHY2_PLL_REG10_IPDIV_VALUE_MAX                      (0x0000001FU)

#define CSL_USB3P0SS_PHY2_PLL_REG10_IPDIV_EN_MASK                        (0x00000002U)
#define CSL_USB3P0SS_PHY2_PLL_REG10_IPDIV_EN_SHIFT                       (0x00000001U)
#define CSL_USB3P0SS_PHY2_PLL_REG10_IPDIV_EN_MAX                         (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG10_UNUSED_MASK                          (0x00000080U)
#define CSL_USB3P0SS_PHY2_PLL_REG10_UNUSED_SHIFT                         (0x00000007U)
#define CSL_USB3P0SS_PHY2_PLL_REG10_UNUSED_MAX                           (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG10_COARSE_CODE_EN_MASK                  (0x00000001U)
#define CSL_USB3P0SS_PHY2_PLL_REG10_COARSE_CODE_EN_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_PHY2_PLL_REG10_COARSE_CODE_EN_MAX                   (0x00000001U)

/* PLL_REG11 */

#define CSL_USB3P0SS_PHY2_PLL_REG11_PLL_STANDBY_MASK                     (0x00000080U)
#define CSL_USB3P0SS_PHY2_PLL_REG11_PLL_STANDBY_SHIFT                    (0x00000007U)
#define CSL_USB3P0SS_PHY2_PLL_REG11_PLL_STANDBY_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG11_PLL_PSO_DEL_MASK                     (0x00000008U)
#define CSL_USB3P0SS_PHY2_PLL_REG11_PLL_PSO_DEL_SHIFT                    (0x00000003U)
#define CSL_USB3P0SS_PHY2_PLL_REG11_PLL_PSO_DEL_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG11_PLL_PSO_MASK                         (0x00000002U)
#define CSL_USB3P0SS_PHY2_PLL_REG11_PLL_PSO_SHIFT                        (0x00000001U)
#define CSL_USB3P0SS_PHY2_PLL_REG11_PLL_PSO_MAX                          (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG11_PLL_PSO_DEL_EN_MASK                  (0x00000004U)
#define CSL_USB3P0SS_PHY2_PLL_REG11_PLL_PSO_DEL_EN_SHIFT                 (0x00000002U)
#define CSL_USB3P0SS_PHY2_PLL_REG11_PLL_PSO_DEL_EN_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG11_PLL_STANDBY_EN_MASK                  (0x00000040U)
#define CSL_USB3P0SS_PHY2_PLL_REG11_PLL_STANDBY_EN_SHIFT                 (0x00000006U)
#define CSL_USB3P0SS_PHY2_PLL_REG11_PLL_STANDBY_EN_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG11_PLL_PSO_EN_MASK                      (0x00000001U)
#define CSL_USB3P0SS_PHY2_PLL_REG11_PLL_PSO_EN_SHIFT                     (0x00000000U)
#define CSL_USB3P0SS_PHY2_PLL_REG11_PLL_PSO_EN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG11_PLL_PD_EN_MASK                       (0x00000010U)
#define CSL_USB3P0SS_PHY2_PLL_REG11_PLL_PD_EN_SHIFT                      (0x00000004U)
#define CSL_USB3P0SS_PHY2_PLL_REG11_PLL_PD_EN_MAX                        (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG11_PLL_PD_MASK                          (0x00000020U)
#define CSL_USB3P0SS_PHY2_PLL_REG11_PLL_PD_SHIFT                         (0x00000005U)
#define CSL_USB3P0SS_PHY2_PLL_REG11_PLL_PD_MAX                           (0x00000001U)

/* PLL_REG12 */

#define CSL_USB3P0SS_PHY2_PLL_REG12_PLL_LDO_CORE_EN_MASK                 (0x00000008U)
#define CSL_USB3P0SS_PHY2_PLL_REG12_PLL_LDO_CORE_EN_SHIFT                (0x00000003U)
#define CSL_USB3P0SS_PHY2_PLL_REG12_PLL_LDO_CORE_EN_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG12_PLL_PD_ANA_EN_MASK                   (0x00000001U)
#define CSL_USB3P0SS_PHY2_PLL_REG12_PLL_PD_ANA_EN_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_PLL_REG12_PLL_PD_ANA_EN_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG12_PLL_PD_ANA_MASK                      (0x00000002U)
#define CSL_USB3P0SS_PHY2_PLL_REG12_PLL_PD_ANA_SHIFT                     (0x00000001U)
#define CSL_USB3P0SS_PHY2_PLL_REG12_PLL_PD_ANA_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG12_UNUSED_MASK                          (0x000000C0U)
#define CSL_USB3P0SS_PHY2_PLL_REG12_UNUSED_SHIFT                         (0x00000006U)
#define CSL_USB3P0SS_PHY2_PLL_REG12_UNUSED_MAX                           (0x00000003U)

#define CSL_USB3P0SS_PHY2_PLL_REG12_PLL_LDO_CORE_EN_EN_MASK              (0x00000004U)
#define CSL_USB3P0SS_PHY2_PLL_REG12_PLL_LDO_CORE_EN_EN_SHIFT             (0x00000002U)
#define CSL_USB3P0SS_PHY2_PLL_REG12_PLL_LDO_CORE_EN_EN_MAX               (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG12_PLL_LDO_REF_EN_MASK                  (0x00000020U)
#define CSL_USB3P0SS_PHY2_PLL_REG12_PLL_LDO_REF_EN_SHIFT                 (0x00000005U)
#define CSL_USB3P0SS_PHY2_PLL_REG12_PLL_LDO_REF_EN_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG12_PLL_LDO_REF_EN_EN_MASK               (0x00000010U)
#define CSL_USB3P0SS_PHY2_PLL_REG12_PLL_LDO_REF_EN_EN_SHIFT              (0x00000004U)
#define CSL_USB3P0SS_PHY2_PLL_REG12_PLL_LDO_REF_EN_EN_MAX                (0x00000001U)

/* PLL_REG13 */

#define CSL_USB3P0SS_PHY2_PLL_REG13_PLL_CLKON_MASK                       (0x00000080U)
#define CSL_USB3P0SS_PHY2_PLL_REG13_PLL_CLKON_SHIFT                      (0x00000007U)
#define CSL_USB3P0SS_PHY2_PLL_REG13_PLL_CLKON_MAX                        (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG13_PLL_LDO_REF_CORE_EN_MASK             (0x00000001U)
#define CSL_USB3P0SS_PHY2_PLL_REG13_PLL_LDO_REF_CORE_EN_SHIFT            (0x00000000U)
#define CSL_USB3P0SS_PHY2_PLL_REG13_PLL_LDO_REF_CORE_EN_MAX              (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG13_PLL_LDO_REF_CORE_MASK                (0x0000007EU)
#define CSL_USB3P0SS_PHY2_PLL_REG13_PLL_LDO_REF_CORE_SHIFT               (0x00000001U)
#define CSL_USB3P0SS_PHY2_PLL_REG13_PLL_LDO_REF_CORE_MAX                 (0x0000003FU)

/* PLL_REG14 */

#define CSL_USB3P0SS_PHY2_PLL_REG14_PLL_LDO_ISO_CNT_THRESHOLD_EN_MASK    (0x00000001U)
#define CSL_USB3P0SS_PHY2_PLL_REG14_PLL_LDO_ISO_CNT_THRESHOLD_EN_SHIFT   (0x00000000U)
#define CSL_USB3P0SS_PHY2_PLL_REG14_PLL_LDO_ISO_CNT_THRESHOLD_EN_MAX     (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG14_PLL_LDO_CNT_THRESHOLD_MASK           (0x000000E0U)
#define CSL_USB3P0SS_PHY2_PLL_REG14_PLL_LDO_CNT_THRESHOLD_SHIFT          (0x00000005U)
#define CSL_USB3P0SS_PHY2_PLL_REG14_PLL_LDO_CNT_THRESHOLD_MAX            (0x00000007U)

#define CSL_USB3P0SS_PHY2_PLL_REG14_PLL_LDO_ISO_CNT_THRESHOLD_MASK       (0x0000000EU)
#define CSL_USB3P0SS_PHY2_PLL_REG14_PLL_LDO_ISO_CNT_THRESHOLD_SHIFT      (0x00000001U)
#define CSL_USB3P0SS_PHY2_PLL_REG14_PLL_LDO_ISO_CNT_THRESHOLD_MAX        (0x00000007U)

#define CSL_USB3P0SS_PHY2_PLL_REG14_PLL_LDO_CNT_THRESHOLD_EN_MASK        (0x00000010U)
#define CSL_USB3P0SS_PHY2_PLL_REG14_PLL_LDO_CNT_THRESHOLD_EN_SHIFT       (0x00000004U)
#define CSL_USB3P0SS_PHY2_PLL_REG14_PLL_LDO_CNT_THRESHOLD_EN_MAX         (0x00000001U)

/* PLL_UNUSED_REG0 */

#define CSL_USB3P0SS_PHY2_PLL_UNUSED_REG0_UNUSED_MASK                    (0x000000FFU)
#define CSL_USB3P0SS_PHY2_PLL_UNUSED_REG0_UNUSED_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_PHY2_PLL_UNUSED_REG0_UNUSED_MAX                     (0x000000FFU)

/* PLL_UNUSED_REG1 */

#define CSL_USB3P0SS_PHY2_PLL_UNUSED_REG1_UNUSED_MASK                    (0x000000FFU)
#define CSL_USB3P0SS_PHY2_PLL_UNUSED_REG1_UNUSED_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_PHY2_PLL_UNUSED_REG1_UNUSED_MAX                     (0x000000FFU)

/* PLL_REG15 */

#define CSL_USB3P0SS_PHY2_PLL_REG15_PD_PFD_MASK                          (0x00000004U)
#define CSL_USB3P0SS_PHY2_PLL_REG15_PD_PFD_SHIFT                         (0x00000002U)
#define CSL_USB3P0SS_PHY2_PLL_REG15_PD_PFD_MAX                           (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG15_RST_FDBK_DIV_MASK                    (0x00000010U)
#define CSL_USB3P0SS_PHY2_PLL_REG15_RST_FDBK_DIV_SHIFT                   (0x00000004U)
#define CSL_USB3P0SS_PHY2_PLL_REG15_RST_FDBK_DIV_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG15_VCO_CNT_WIN_MASK                     (0x00000020U)
#define CSL_USB3P0SS_PHY2_PLL_REG15_VCO_CNT_WIN_SHIFT                    (0x00000005U)
#define CSL_USB3P0SS_PHY2_PLL_REG15_VCO_CNT_WIN_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG15_COARSEDONE_MASK                      (0x00000040U)
#define CSL_USB3P0SS_PHY2_PLL_REG15_COARSEDONE_SHIFT                     (0x00000006U)
#define CSL_USB3P0SS_PHY2_PLL_REG15_COARSEDONE_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG15_STARTLOOP_MASK                       (0x00000002U)
#define CSL_USB3P0SS_PHY2_PLL_REG15_STARTLOOP_SHIFT                      (0x00000001U)
#define CSL_USB3P0SS_PHY2_PLL_REG15_STARTLOOP_MAX                        (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG15_COARSE_CODE_8_MASK                   (0x00000001U)
#define CSL_USB3P0SS_PHY2_PLL_REG15_COARSE_CODE_8_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_PLL_REG15_COARSE_CODE_8_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG15_UNUSED_MASK                          (0x00000008U)
#define CSL_USB3P0SS_PHY2_PLL_REG15_UNUSED_SHIFT                         (0x00000003U)
#define CSL_USB3P0SS_PHY2_PLL_REG15_UNUSED_MAX                           (0x00000001U)

#define CSL_USB3P0SS_PHY2_PLL_REG15_PLL_LOCK_MASK                        (0x00000080U)
#define CSL_USB3P0SS_PHY2_PLL_REG15_PLL_LOCK_SHIFT                       (0x00000007U)
#define CSL_USB3P0SS_PHY2_PLL_REG15_PLL_LOCK_MAX                         (0x00000001U)

/* PLL_REG16 */

#define CSL_USB3P0SS_PHY2_PLL_REG16_COARSE_CODE_MASK                     (0x000000FFU)
#define CSL_USB3P0SS_PHY2_PLL_REG16_COARSE_CODE_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_PHY2_PLL_REG16_COARSE_CODE_MAX                      (0x000000FFU)

/* PLL_UNUSED_REG2 */

#define CSL_USB3P0SS_PHY2_PLL_UNUSED_REG2_UNUSED_MASK                    (0x000000FFU)
#define CSL_USB3P0SS_PHY2_PLL_UNUSED_REG2_UNUSED_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_PHY2_PLL_UNUSED_REG2_UNUSED_MAX                     (0x000000FFU)

/* CALIB_REG0 */

#define CSL_USB3P0SS_PHY2_CALIB_REG0_COMP_OUT_MASK                       (0x00000020U)
#define CSL_USB3P0SS_PHY2_CALIB_REG0_COMP_OUT_SHIFT                      (0x00000005U)
#define CSL_USB3P0SS_PHY2_CALIB_REG0_COMP_OUT_MAX                        (0x00000001U)

#define CSL_USB3P0SS_PHY2_CALIB_REG0_CALIB_CLK_MASK                      (0x00000080U)
#define CSL_USB3P0SS_PHY2_CALIB_REG0_CALIB_CLK_SHIFT                     (0x00000007U)
#define CSL_USB3P0SS_PHY2_CALIB_REG0_CALIB_CLK_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_CALIB_REG0_CALIB_CLK_EN_MASK                   (0x00000040U)
#define CSL_USB3P0SS_PHY2_CALIB_REG0_CALIB_CLK_EN_SHIFT                  (0x00000006U)
#define CSL_USB3P0SS_PHY2_CALIB_REG0_CALIB_CLK_EN_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_CALIB_REG0_INIT_WAIT_OVR_EN_MASK               (0x00000001U)
#define CSL_USB3P0SS_PHY2_CALIB_REG0_INIT_WAIT_OVR_EN_SHIFT              (0x00000000U)
#define CSL_USB3P0SS_PHY2_CALIB_REG0_INIT_WAIT_OVR_EN_MAX                (0x00000001U)

#define CSL_USB3P0SS_PHY2_CALIB_REG0_INIT_WAIT_OVR_MASK                  (0x0000001EU)
#define CSL_USB3P0SS_PHY2_CALIB_REG0_INIT_WAIT_OVR_SHIFT                 (0x00000001U)
#define CSL_USB3P0SS_PHY2_CALIB_REG0_INIT_WAIT_OVR_MAX                   (0x0000000FU)

/* CALIB_REG1 */

#define CSL_USB3P0SS_PHY2_CALIB_REG1_CALIB_CODE_EN_MASK                  (0x00000001U)
#define CSL_USB3P0SS_PHY2_CALIB_REG1_CALIB_CODE_EN_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_PHY2_CALIB_REG1_CALIB_CODE_EN_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_CALIB_REG1_UNUSED_MASK                         (0x00000080U)
#define CSL_USB3P0SS_PHY2_CALIB_REG1_UNUSED_SHIFT                        (0x00000007U)
#define CSL_USB3P0SS_PHY2_CALIB_REG1_UNUSED_MAX                          (0x00000001U)

#define CSL_USB3P0SS_PHY2_CALIB_REG1_CALIB_CODE_MASK                     (0x0000007EU)
#define CSL_USB3P0SS_PHY2_CALIB_REG1_CALIB_CODE_SHIFT                    (0x00000001U)
#define CSL_USB3P0SS_PHY2_CALIB_REG1_CALIB_CODE_MAX                      (0x0000003FU)

/* BC_REG0 */

#define CSL_USB3P0SS_PHY2_BC_REG0_ID_PULLUP_EN_MASK                      (0x00000001U)
#define CSL_USB3P0SS_PHY2_BC_REG0_ID_PULLUP_EN_SHIFT                     (0x00000000U)
#define CSL_USB3P0SS_PHY2_BC_REG0_ID_PULLUP_EN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG0_ADP_EN_MASK                            (0x00000008U)
#define CSL_USB3P0SS_PHY2_BC_REG0_ADP_EN_SHIFT                           (0x00000003U)
#define CSL_USB3P0SS_PHY2_BC_REG0_ADP_EN_MAX                             (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG0_ADP_EN_EN_MASK                         (0x00000004U)
#define CSL_USB3P0SS_PHY2_BC_REG0_ADP_EN_EN_SHIFT                        (0x00000002U)
#define CSL_USB3P0SS_PHY2_BC_REG0_ADP_EN_EN_MAX                          (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG0_UNUSED_MASK                            (0x000000F0U)
#define CSL_USB3P0SS_PHY2_BC_REG0_UNUSED_SHIFT                           (0x00000004U)
#define CSL_USB3P0SS_PHY2_BC_REG0_UNUSED_MAX                             (0x0000000FU)

#define CSL_USB3P0SS_PHY2_BC_REG0_ID_PULLUP_MASK                         (0x00000002U)
#define CSL_USB3P0SS_PHY2_BC_REG0_ID_PULLUP_SHIFT                        (0x00000001U)
#define CSL_USB3P0SS_PHY2_BC_REG0_ID_PULLUP_MAX                          (0x00000001U)

/* BC_REG1 */

#define CSL_USB3P0SS_PHY2_BC_REG1_ADP_PROBE_EN_MASK                      (0x00000002U)
#define CSL_USB3P0SS_PHY2_BC_REG1_ADP_PROBE_EN_SHIFT                     (0x00000001U)
#define CSL_USB3P0SS_PHY2_BC_REG1_ADP_PROBE_EN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG1_ADP_SINK_I_EN_MASK                     (0x00000020U)
#define CSL_USB3P0SS_PHY2_BC_REG1_ADP_SINK_I_EN_SHIFT                    (0x00000005U)
#define CSL_USB3P0SS_PHY2_BC_REG1_ADP_SINK_I_EN_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG1_ADP_SENSE_EN_CTRL_MASK                 (0x00000004U)
#define CSL_USB3P0SS_PHY2_BC_REG1_ADP_SENSE_EN_CTRL_SHIFT                (0x00000002U)
#define CSL_USB3P0SS_PHY2_BC_REG1_ADP_SENSE_EN_CTRL_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG1_ADP_SENSE_EN_MASK                      (0x00000008U)
#define CSL_USB3P0SS_PHY2_BC_REG1_ADP_SENSE_EN_SHIFT                     (0x00000003U)
#define CSL_USB3P0SS_PHY2_BC_REG1_ADP_SENSE_EN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG1_ADP_SOURCE_I_EN_CTRL_MASK              (0x00000040U)
#define CSL_USB3P0SS_PHY2_BC_REG1_ADP_SOURCE_I_EN_CTRL_SHIFT             (0x00000006U)
#define CSL_USB3P0SS_PHY2_BC_REG1_ADP_SOURCE_I_EN_CTRL_MAX               (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG1_ADP_SOURCE_I_EN_MASK                   (0x00000080U)
#define CSL_USB3P0SS_PHY2_BC_REG1_ADP_SOURCE_I_EN_SHIFT                  (0x00000007U)
#define CSL_USB3P0SS_PHY2_BC_REG1_ADP_SOURCE_I_EN_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG1_ADP_SINK_I_EN_CTRL_MASK                (0x00000010U)
#define CSL_USB3P0SS_PHY2_BC_REG1_ADP_SINK_I_EN_CTRL_SHIFT               (0x00000004U)
#define CSL_USB3P0SS_PHY2_BC_REG1_ADP_SINK_I_EN_CTRL_MAX                 (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG1_ADP_PROBE_EN_CTRL_MASK                 (0x00000001U)
#define CSL_USB3P0SS_PHY2_BC_REG1_ADP_PROBE_EN_CTRL_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_PHY2_BC_REG1_ADP_PROBE_EN_CTRL_MAX                  (0x00000001U)

/* BC_REG2 */

#define CSL_USB3P0SS_PHY2_BC_REG2_IDP_SINK_EN_VALUE_MASK                 (0x00000020U)
#define CSL_USB3P0SS_PHY2_BC_REG2_IDP_SINK_EN_VALUE_SHIFT                (0x00000005U)
#define CSL_USB3P0SS_PHY2_BC_REG2_IDP_SINK_EN_VALUE_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG2_IDM_SINK_EN_VALUE_MASK                 (0x00000080U)
#define CSL_USB3P0SS_PHY2_BC_REG2_IDM_SINK_EN_VALUE_SHIFT                (0x00000007U)
#define CSL_USB3P0SS_PHY2_BC_REG2_IDM_SINK_EN_VALUE_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG2_IDP_SRC_EN_VALUE_MASK                  (0x00000008U)
#define CSL_USB3P0SS_PHY2_BC_REG2_IDP_SRC_EN_VALUE_SHIFT                 (0x00000003U)
#define CSL_USB3P0SS_PHY2_BC_REG2_IDP_SRC_EN_VALUE_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG2_IDP_SRC_EN_CNTRL_MASK                  (0x00000004U)
#define CSL_USB3P0SS_PHY2_BC_REG2_IDP_SRC_EN_CNTRL_SHIFT                 (0x00000002U)
#define CSL_USB3P0SS_PHY2_BC_REG2_IDP_SRC_EN_CNTRL_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG2_IDM_SINK_EN_CNTRL_MASK                 (0x00000040U)
#define CSL_USB3P0SS_PHY2_BC_REG2_IDM_SINK_EN_CNTRL_SHIFT                (0x00000006U)
#define CSL_USB3P0SS_PHY2_BC_REG2_IDM_SINK_EN_CNTRL_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG2_BC_EN_CNTRL_MASK                       (0x00000001U)
#define CSL_USB3P0SS_PHY2_BC_REG2_BC_EN_CNTRL_SHIFT                      (0x00000000U)
#define CSL_USB3P0SS_PHY2_BC_REG2_BC_EN_CNTRL_MAX                        (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG2_IDP_SINK_EN_CNTRL_MASK                 (0x00000010U)
#define CSL_USB3P0SS_PHY2_BC_REG2_IDP_SINK_EN_CNTRL_SHIFT                (0x00000004U)
#define CSL_USB3P0SS_PHY2_BC_REG2_IDP_SINK_EN_CNTRL_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG2_BC_EN_VALUE_MASK                       (0x00000002U)
#define CSL_USB3P0SS_PHY2_BC_REG2_BC_EN_VALUE_SHIFT                      (0x00000001U)
#define CSL_USB3P0SS_PHY2_BC_REG2_BC_EN_VALUE_MAX                        (0x00000001U)

/* BC_REG3 */

#define CSL_USB3P0SS_PHY2_BC_REG3_VDM_SRC_EN_VALUE_MASK                  (0x00000002U)
#define CSL_USB3P0SS_PHY2_BC_REG3_VDM_SRC_EN_VALUE_SHIFT                 (0x00000001U)
#define CSL_USB3P0SS_PHY2_BC_REG3_VDM_SRC_EN_VALUE_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG3_VDM_SRC_EN_CNTRL_MASK                  (0x00000001U)
#define CSL_USB3P0SS_PHY2_BC_REG3_VDM_SRC_EN_CNTRL_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_PHY2_BC_REG3_VDM_SRC_EN_CNTRL_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG3_VDP_SRC_EN_VALUE_MASK                  (0x00000008U)
#define CSL_USB3P0SS_PHY2_BC_REG3_VDP_SRC_EN_VALUE_SHIFT                 (0x00000003U)
#define CSL_USB3P0SS_PHY2_BC_REG3_VDP_SRC_EN_VALUE_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG3_DM_VDAT_REF_COMP_EN_VALUE_MASK         (0x00000080U)
#define CSL_USB3P0SS_PHY2_BC_REG3_DM_VDAT_REF_COMP_EN_VALUE_SHIFT        (0x00000007U)
#define CSL_USB3P0SS_PHY2_BC_REG3_DM_VDAT_REF_COMP_EN_VALUE_MAX          (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG3_DM_VDAT_REF_COMP_EN_CNTRL_MASK         (0x00000040U)
#define CSL_USB3P0SS_PHY2_BC_REG3_DM_VDAT_REF_COMP_EN_CNTRL_SHIFT        (0x00000006U)
#define CSL_USB3P0SS_PHY2_BC_REG3_DM_VDAT_REF_COMP_EN_CNTRL_MAX          (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG3_VDP_SRC_EN_CNTRL_MASK                  (0x00000004U)
#define CSL_USB3P0SS_PHY2_BC_REG3_VDP_SRC_EN_CNTRL_SHIFT                 (0x00000002U)
#define CSL_USB3P0SS_PHY2_BC_REG3_VDP_SRC_EN_CNTRL_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG3_DP_VDAT_REF_COMP_EN_CNTRL_MASK         (0x00000010U)
#define CSL_USB3P0SS_PHY2_BC_REG3_DP_VDAT_REF_COMP_EN_CNTRL_SHIFT        (0x00000004U)
#define CSL_USB3P0SS_PHY2_BC_REG3_DP_VDAT_REF_COMP_EN_CNTRL_MAX          (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG3_DP_VDAT_REF_COMP_EN_VALUE_MASK         (0x00000020U)
#define CSL_USB3P0SS_PHY2_BC_REG3_DP_VDAT_REF_COMP_EN_VALUE_SHIFT        (0x00000005U)
#define CSL_USB3P0SS_PHY2_BC_REG3_DP_VDAT_REF_COMP_EN_VALUE_MAX          (0x00000001U)

/* BC_REG4 */

#define CSL_USB3P0SS_PHY2_BC_REG4_RID_A_REF_EN_CNTRL_MASK                (0x00000040U)
#define CSL_USB3P0SS_PHY2_BC_REG4_RID_A_REF_EN_CNTRL_SHIFT               (0x00000006U)
#define CSL_USB3P0SS_PHY2_BC_REG4_RID_A_REF_EN_CNTRL_MAX                 (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG4_RID_FLOAT_REF_EN_CNTRL_MASK            (0x00000010U)
#define CSL_USB3P0SS_PHY2_BC_REG4_RID_FLOAT_REF_EN_CNTRL_SHIFT           (0x00000004U)
#define CSL_USB3P0SS_PHY2_BC_REG4_RID_FLOAT_REF_EN_CNTRL_MAX             (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG4_RID_A_REF_EN_VALUE_MASK                (0x00000080U)
#define CSL_USB3P0SS_PHY2_BC_REG4_RID_A_REF_EN_VALUE_SHIFT               (0x00000007U)
#define CSL_USB3P0SS_PHY2_BC_REG4_RID_A_REF_EN_VALUE_MAX                 (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG4_RID_FLOAT_COMP_EN_VALUE_MASK           (0x00000002U)
#define CSL_USB3P0SS_PHY2_BC_REG4_RID_FLOAT_COMP_EN_VALUE_SHIFT          (0x00000001U)
#define CSL_USB3P0SS_PHY2_BC_REG4_RID_FLOAT_COMP_EN_VALUE_MAX            (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG4_RID_FLOAT_REF_EN_VALUE_MASK            (0x00000020U)
#define CSL_USB3P0SS_PHY2_BC_REG4_RID_FLOAT_REF_EN_VALUE_SHIFT           (0x00000005U)
#define CSL_USB3P0SS_PHY2_BC_REG4_RID_FLOAT_REF_EN_VALUE_MAX             (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG4_RID_FLOAT_COMP_EN_CNTRL_MASK           (0x00000001U)
#define CSL_USB3P0SS_PHY2_BC_REG4_RID_FLOAT_COMP_EN_CNTRL_SHIFT          (0x00000000U)
#define CSL_USB3P0SS_PHY2_BC_REG4_RID_FLOAT_COMP_EN_CNTRL_MAX            (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG4_RID_NONFLOAT_COMP_EN_CNTRL_MASK        (0x00000004U)
#define CSL_USB3P0SS_PHY2_BC_REG4_RID_NONFLOAT_COMP_EN_CNTRL_SHIFT       (0x00000002U)
#define CSL_USB3P0SS_PHY2_BC_REG4_RID_NONFLOAT_COMP_EN_CNTRL_MAX         (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG4_RID_NONFLOAT_COMP_EN_VALUE_MASK        (0x00000008U)
#define CSL_USB3P0SS_PHY2_BC_REG4_RID_NONFLOAT_COMP_EN_VALUE_SHIFT       (0x00000003U)
#define CSL_USB3P0SS_PHY2_BC_REG4_RID_NONFLOAT_COMP_EN_VALUE_MAX         (0x00000001U)

/* BC_REG5 */

#define CSL_USB3P0SS_PHY2_BC_REG5_RID_B_C_COMP_EN_VALUE_MASK             (0x00000080U)
#define CSL_USB3P0SS_PHY2_BC_REG5_RID_B_C_COMP_EN_VALUE_SHIFT            (0x00000007U)
#define CSL_USB3P0SS_PHY2_BC_REG5_RID_B_C_COMP_EN_VALUE_MAX              (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG5_RID_B_C_COMP_EN_CNTRL_MASK             (0x00000040U)
#define CSL_USB3P0SS_PHY2_BC_REG5_RID_B_C_COMP_EN_CNTRL_SHIFT            (0x00000006U)
#define CSL_USB3P0SS_PHY2_BC_REG5_RID_B_C_COMP_EN_CNTRL_MAX              (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG5_RID_A_COMP_EN_CNTRL_MASK               (0x00000010U)
#define CSL_USB3P0SS_PHY2_BC_REG5_RID_A_COMP_EN_CNTRL_SHIFT              (0x00000004U)
#define CSL_USB3P0SS_PHY2_BC_REG5_RID_A_COMP_EN_CNTRL_MAX                (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG5_RID_B_REF_EN_CNTRL_MASK                (0x00000001U)
#define CSL_USB3P0SS_PHY2_BC_REG5_RID_B_REF_EN_CNTRL_SHIFT               (0x00000000U)
#define CSL_USB3P0SS_PHY2_BC_REG5_RID_B_REF_EN_CNTRL_MAX                 (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG5_RID_A_COMP_EN_VALUE_MASK               (0x00000020U)
#define CSL_USB3P0SS_PHY2_BC_REG5_RID_A_COMP_EN_VALUE_SHIFT              (0x00000005U)
#define CSL_USB3P0SS_PHY2_BC_REG5_RID_A_COMP_EN_VALUE_MAX                (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG5_RID_C_REF_EN_CNTRL_MASK                (0x00000004U)
#define CSL_USB3P0SS_PHY2_BC_REG5_RID_C_REF_EN_CNTRL_SHIFT               (0x00000002U)
#define CSL_USB3P0SS_PHY2_BC_REG5_RID_C_REF_EN_CNTRL_MAX                 (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG5_RID_B_REF_EN_VALUE_MASK                (0x00000002U)
#define CSL_USB3P0SS_PHY2_BC_REG5_RID_B_REF_EN_VALUE_SHIFT               (0x00000001U)
#define CSL_USB3P0SS_PHY2_BC_REG5_RID_B_REF_EN_VALUE_MAX                 (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG5_RID_C_REF_EN_VALUE_MASK                (0x00000008U)
#define CSL_USB3P0SS_PHY2_BC_REG5_RID_C_REF_EN_VALUE_SHIFT               (0x00000003U)
#define CSL_USB3P0SS_PHY2_BC_REG5_RID_C_REF_EN_VALUE_MAX                 (0x00000001U)

/* BC_REG6 */

#define CSL_USB3P0SS_PHY2_BC_REG6_DM_VLGC_COMP_EN_VALUE_MASK             (0x00000002U)
#define CSL_USB3P0SS_PHY2_BC_REG6_DM_VLGC_COMP_EN_VALUE_SHIFT            (0x00000001U)
#define CSL_USB3P0SS_PHY2_BC_REG6_DM_VLGC_COMP_EN_VALUE_MAX              (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG6_BC_DELAY_EN_MASK                       (0x00000004U)
#define CSL_USB3P0SS_PHY2_BC_REG6_BC_DELAY_EN_SHIFT                      (0x00000002U)
#define CSL_USB3P0SS_PHY2_BC_REG6_BC_DELAY_EN_MAX                        (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG6_BC_DELAY_VALUE_MASK                    (0x000000F8U)
#define CSL_USB3P0SS_PHY2_BC_REG6_BC_DELAY_VALUE_SHIFT                   (0x00000003U)
#define CSL_USB3P0SS_PHY2_BC_REG6_BC_DELAY_VALUE_MAX                     (0x0000001FU)

#define CSL_USB3P0SS_PHY2_BC_REG6_DM_VLGC_COMP_EN_CNTRL_MASK             (0x00000001U)
#define CSL_USB3P0SS_PHY2_BC_REG6_DM_VLGC_COMP_EN_CNTRL_SHIFT            (0x00000000U)
#define CSL_USB3P0SS_PHY2_BC_REG6_DM_VLGC_COMP_EN_CNTRL_MAX              (0x00000001U)

/* BC_REG7 */

#define CSL_USB3P0SS_PHY2_BC_REG7_DM_CURRENT_SRC_EN_VALUE_MASK           (0x00000004U)
#define CSL_USB3P0SS_PHY2_BC_REG7_DM_CURRENT_SRC_EN_VALUE_SHIFT          (0x00000002U)
#define CSL_USB3P0SS_PHY2_BC_REG7_DM_CURRENT_SRC_EN_VALUE_MAX            (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG7_RESET_CNTRL_MASK                       (0x00000008U)
#define CSL_USB3P0SS_PHY2_BC_REG7_RESET_CNTRL_SHIFT                      (0x00000003U)
#define CSL_USB3P0SS_PHY2_BC_REG7_RESET_CNTRL_MAX                        (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG7_RID_FLOAT_SRC_EN_VALUE_MASK            (0x00000020U)
#define CSL_USB3P0SS_PHY2_BC_REG7_RID_FLOAT_SRC_EN_VALUE_SHIFT           (0x00000005U)
#define CSL_USB3P0SS_PHY2_BC_REG7_RID_FLOAT_SRC_EN_VALUE_MAX             (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG7_RID_NONFLOAT_SRC_EN_VALUE_MASK         (0x00000080U)
#define CSL_USB3P0SS_PHY2_BC_REG7_RID_NONFLOAT_SRC_EN_VALUE_SHIFT        (0x00000007U)
#define CSL_USB3P0SS_PHY2_BC_REG7_RID_NONFLOAT_SRC_EN_VALUE_MAX          (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG7_RID_FLOAT_SRC_EN_CNTRL_MASK            (0x00000010U)
#define CSL_USB3P0SS_PHY2_BC_REG7_RID_FLOAT_SRC_EN_CNTRL_SHIFT           (0x00000004U)
#define CSL_USB3P0SS_PHY2_BC_REG7_RID_FLOAT_SRC_EN_CNTRL_MAX             (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG7_DM_CURRENT_SRC_EN_CNTRL_MASK           (0x00000002U)
#define CSL_USB3P0SS_PHY2_BC_REG7_DM_CURRENT_SRC_EN_CNTRL_SHIFT          (0x00000001U)
#define CSL_USB3P0SS_PHY2_BC_REG7_DM_CURRENT_SRC_EN_CNTRL_MAX            (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG7_UNUSED_MASK                            (0x00000001U)
#define CSL_USB3P0SS_PHY2_BC_REG7_UNUSED_SHIFT                           (0x00000000U)
#define CSL_USB3P0SS_PHY2_BC_REG7_UNUSED_MAX                             (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG7_RID_NONFLOAT_SRC_EN_CNTRL_MASK         (0x00000040U)
#define CSL_USB3P0SS_PHY2_BC_REG7_RID_NONFLOAT_SRC_EN_CNTRL_SHIFT        (0x00000006U)
#define CSL_USB3P0SS_PHY2_BC_REG7_RID_NONFLOAT_SRC_EN_CNTRL_MAX          (0x00000001U)

/* TED_REG0 */

#define CSL_USB3P0SS_PHY2_TED_REG0_CALIB_CODE_UP_EN_MASK                 (0x00000080U)
#define CSL_USB3P0SS_PHY2_TED_REG0_CALIB_CODE_UP_EN_SHIFT                (0x00000007U)
#define CSL_USB3P0SS_PHY2_TED_REG0_CALIB_CODE_UP_EN_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_TED_REG0_COMP_OUT_UP_INV_MASK                  (0x00000001U)
#define CSL_USB3P0SS_PHY2_TED_REG0_COMP_OUT_UP_INV_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_PHY2_TED_REG0_COMP_OUT_UP_INV_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_TED_REG0_DELAY_EN_MASK                         (0x00000010U)
#define CSL_USB3P0SS_PHY2_TED_REG0_DELAY_EN_SHIFT                        (0x00000004U)
#define CSL_USB3P0SS_PHY2_TED_REG0_DELAY_EN_MAX                          (0x00000001U)

#define CSL_USB3P0SS_PHY2_TED_REG0_CALIB_DONE_MASK                       (0x00000008U)
#define CSL_USB3P0SS_PHY2_TED_REG0_CALIB_DONE_SHIFT                      (0x00000003U)
#define CSL_USB3P0SS_PHY2_TED_REG0_CALIB_DONE_MAX                        (0x00000001U)

#define CSL_USB3P0SS_PHY2_TED_REG0_COMP_OUT_DOWN_INV_MASK                (0x00000002U)
#define CSL_USB3P0SS_PHY2_TED_REG0_COMP_OUT_DOWN_INV_SHIFT               (0x00000001U)
#define CSL_USB3P0SS_PHY2_TED_REG0_COMP_OUT_DOWN_INV_MAX                 (0x00000001U)

#define CSL_USB3P0SS_PHY2_TED_REG0_CALIIB_DONE_EN_MASK                   (0x00000004U)
#define CSL_USB3P0SS_PHY2_TED_REG0_CALIIB_DONE_EN_SHIFT                  (0x00000002U)
#define CSL_USB3P0SS_PHY2_TED_REG0_CALIIB_DONE_EN_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_TED_REG0_DELAY_VALUE_MASK                      (0x00000060U)
#define CSL_USB3P0SS_PHY2_TED_REG0_DELAY_VALUE_SHIFT                     (0x00000005U)
#define CSL_USB3P0SS_PHY2_TED_REG0_DELAY_VALUE_MAX                       (0x00000003U)

/* TED_REG1 */

#define CSL_USB3P0SS_PHY2_TED_REG1_CALIB_CODE_UP_MASK                    (0x0000000FU)
#define CSL_USB3P0SS_PHY2_TED_REG1_CALIB_CODE_UP_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_PHY2_TED_REG1_CALIB_CODE_UP_MAX                     (0x0000000FU)

#define CSL_USB3P0SS_PHY2_TED_REG1_CALIB_CODE_DOWN_MASK                  (0x000000F0U)
#define CSL_USB3P0SS_PHY2_TED_REG1_CALIB_CODE_DOWN_SHIFT                 (0x00000004U)
#define CSL_USB3P0SS_PHY2_TED_REG1_CALIB_CODE_DOWN_MAX                   (0x0000000FU)

/* TED_REG2 */

#define CSL_USB3P0SS_PHY2_TED_REG2_CALIB_MODE_DN_MASK                    (0x00000010U)
#define CSL_USB3P0SS_PHY2_TED_REG2_CALIB_MODE_DN_SHIFT                   (0x00000004U)
#define CSL_USB3P0SS_PHY2_TED_REG2_CALIB_MODE_DN_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_TED_REG2_CALIB_CODE_DN_EN_MASK                 (0x00000001U)
#define CSL_USB3P0SS_PHY2_TED_REG2_CALIB_CODE_DN_EN_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_PHY2_TED_REG2_CALIB_CODE_DN_EN_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_TED_REG2_CALIB_MODE_UP_MASK                    (0x00000004U)
#define CSL_USB3P0SS_PHY2_TED_REG2_CALIB_MODE_UP_SHIFT                   (0x00000002U)
#define CSL_USB3P0SS_PHY2_TED_REG2_CALIB_MODE_UP_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_TED_REG2_UNUSED_MASK                           (0x000000E0U)
#define CSL_USB3P0SS_PHY2_TED_REG2_UNUSED_SHIFT                          (0x00000005U)
#define CSL_USB3P0SS_PHY2_TED_REG2_UNUSED_MAX                            (0x00000007U)

#define CSL_USB3P0SS_PHY2_TED_REG2_CALIB_MODE_UP_EN_MASK                 (0x00000002U)
#define CSL_USB3P0SS_PHY2_TED_REG2_CALIB_MODE_UP_EN_SHIFT                (0x00000001U)
#define CSL_USB3P0SS_PHY2_TED_REG2_CALIB_MODE_UP_EN_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_TED_REG2_CALIB_MODE_DN_EN_MASK                 (0x00000008U)
#define CSL_USB3P0SS_PHY2_TED_REG2_CALIB_MODE_DN_EN_SHIFT                (0x00000003U)
#define CSL_USB3P0SS_PHY2_TED_REG2_CALIB_MODE_DN_EN_MAX                  (0x00000001U)

/* CALIB_REG2 */

#define CSL_USB3P0SS_PHY2_CALIB_REG2_CALIB_CMP_MASK                      (0x00000008U)
#define CSL_USB3P0SS_PHY2_CALIB_REG2_CALIB_CMP_SHIFT                     (0x00000003U)
#define CSL_USB3P0SS_PHY2_CALIB_REG2_CALIB_CMP_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_CALIB_REG2_CALIB_DONE_MASK                     (0x00000001U)
#define CSL_USB3P0SS_PHY2_CALIB_REG2_CALIB_DONE_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_PHY2_CALIB_REG2_CALIB_DONE_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_CALIB_REG2_CALIB_CLOCK_MASK                    (0x00000002U)
#define CSL_USB3P0SS_PHY2_CALIB_REG2_CALIB_CLOCK_SHIFT                   (0x00000001U)
#define CSL_USB3P0SS_PHY2_CALIB_REG2_CALIB_CLOCK_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_CALIB_REG2_CALIB_PD_MASK                       (0x00000004U)
#define CSL_USB3P0SS_PHY2_CALIB_REG2_CALIB_PD_SHIFT                      (0x00000002U)
#define CSL_USB3P0SS_PHY2_CALIB_REG2_CALIB_PD_MAX                        (0x00000001U)

#define CSL_USB3P0SS_PHY2_CALIB_REG2_UNUSED_MASK                         (0x000000F0U)
#define CSL_USB3P0SS_PHY2_CALIB_REG2_UNUSED_SHIFT                        (0x00000004U)
#define CSL_USB3P0SS_PHY2_CALIB_REG2_UNUSED_MAX                          (0x0000000FU)

/* CALIB_REG3 */

#define CSL_USB3P0SS_PHY2_CALIB_REG3_BG_UNIT_RES_CALIB_MASK              (0x0000001FU)
#define CSL_USB3P0SS_PHY2_CALIB_REG3_BG_UNIT_RES_CALIB_SHIFT             (0x00000000U)
#define CSL_USB3P0SS_PHY2_CALIB_REG3_BG_UNIT_RES_CALIB_MAX               (0x0000001FU)

#define CSL_USB3P0SS_PHY2_CALIB_REG3_UNUSED_MASK                         (0x000000E0U)
#define CSL_USB3P0SS_PHY2_CALIB_REG3_UNUSED_SHIFT                        (0x00000005U)
#define CSL_USB3P0SS_PHY2_CALIB_REG3_UNUSED_MAX                          (0x00000007U)

/* BC_REG8 */

#define CSL_USB3P0SS_PHY2_BC_REG8_DCD_COMP_MASK                          (0x00000080U)
#define CSL_USB3P0SS_PHY2_BC_REG8_DCD_COMP_SHIFT                         (0x00000007U)
#define CSL_USB3P0SS_PHY2_BC_REG8_DCD_COMP_MAX                           (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG8_ADP_SENSE_MASK                         (0x00000040U)
#define CSL_USB3P0SS_PHY2_BC_REG8_ADP_SENSE_SHIFT                        (0x00000006U)
#define CSL_USB3P0SS_PHY2_BC_REG8_ADP_SENSE_MAX                          (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG8_UNUSED_MASK                            (0x00000003U)
#define CSL_USB3P0SS_PHY2_BC_REG8_UNUSED_SHIFT                           (0x00000000U)
#define CSL_USB3P0SS_PHY2_BC_REG8_UNUSED_MAX                             (0x00000003U)

#define CSL_USB3P0SS_PHY2_BC_REG8_VBUSVALID_MASK                         (0x00000008U)
#define CSL_USB3P0SS_PHY2_BC_REG8_VBUSVALID_SHIFT                        (0x00000003U)
#define CSL_USB3P0SS_PHY2_BC_REG8_VBUSVALID_MAX                          (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG8_ADP_PROBE_MASK                         (0x00000020U)
#define CSL_USB3P0SS_PHY2_BC_REG8_ADP_PROBE_SHIFT                        (0x00000005U)
#define CSL_USB3P0SS_PHY2_BC_REG8_ADP_PROBE_MAX                          (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG8_IDDIG_MASK                             (0x00000004U)
#define CSL_USB3P0SS_PHY2_BC_REG8_IDDIG_SHIFT                            (0x00000002U)
#define CSL_USB3P0SS_PHY2_BC_REG8_IDDIG_MAX                              (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG8_BVALID_MASK                            (0x00000010U)
#define CSL_USB3P0SS_PHY2_BC_REG8_BVALID_SHIFT                           (0x00000004U)
#define CSL_USB3P0SS_PHY2_BC_REG8_BVALID_MAX                             (0x00000001U)

/* BC_REG9 */

#define CSL_USB3P0SS_PHY2_BC_REG9_O_IDP_SRC_EN_MASK                      (0x00000002U)
#define CSL_USB3P0SS_PHY2_BC_REG9_O_IDP_SRC_EN_SHIFT                     (0x00000001U)
#define CSL_USB3P0SS_PHY2_BC_REG9_O_IDP_SRC_EN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG9_O_BC_EN_MASK                           (0x00000001U)
#define CSL_USB3P0SS_PHY2_BC_REG9_O_BC_EN_SHIFT                          (0x00000000U)
#define CSL_USB3P0SS_PHY2_BC_REG9_O_BC_EN_MAX                            (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG9_O_DM_VDAT_REF_COMP_EN_MASK             (0x00000080U)
#define CSL_USB3P0SS_PHY2_BC_REG9_O_DM_VDAT_REF_COMP_EN_SHIFT            (0x00000007U)
#define CSL_USB3P0SS_PHY2_BC_REG9_O_DM_VDAT_REF_COMP_EN_MAX              (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG9_O_VDM_SRC_EN_MASK                      (0x00000020U)
#define CSL_USB3P0SS_PHY2_BC_REG9_O_VDM_SRC_EN_SHIFT                     (0x00000005U)
#define CSL_USB3P0SS_PHY2_BC_REG9_O_VDM_SRC_EN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG9_O_DP_VDAT_REF_COMP_EN_MASK             (0x00000040U)
#define CSL_USB3P0SS_PHY2_BC_REG9_O_DP_VDAT_REF_COMP_EN_SHIFT            (0x00000006U)
#define CSL_USB3P0SS_PHY2_BC_REG9_O_DP_VDAT_REF_COMP_EN_MAX              (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG9_O_IDM_SINK_EN_MASK                     (0x00000008U)
#define CSL_USB3P0SS_PHY2_BC_REG9_O_IDM_SINK_EN_SHIFT                    (0x00000003U)
#define CSL_USB3P0SS_PHY2_BC_REG9_O_IDM_SINK_EN_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG9_O_IDP_SINK_EN_MASK                     (0x00000004U)
#define CSL_USB3P0SS_PHY2_BC_REG9_O_IDP_SINK_EN_SHIFT                    (0x00000002U)
#define CSL_USB3P0SS_PHY2_BC_REG9_O_IDP_SINK_EN_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG9_O_VDP_SRC_EN_MASK                      (0x00000010U)
#define CSL_USB3P0SS_PHY2_BC_REG9_O_VDP_SRC_EN_SHIFT                     (0x00000004U)
#define CSL_USB3P0SS_PHY2_BC_REG9_O_VDP_SRC_EN_MAX                       (0x00000001U)

/* BC_REG10 */

#define CSL_USB3P0SS_PHY2_BC_REG10_O_RID_B_C_COMP_EN_MASK                (0x00000080U)
#define CSL_USB3P0SS_PHY2_BC_REG10_O_RID_B_C_COMP_EN_SHIFT               (0x00000007U)
#define CSL_USB3P0SS_PHY2_BC_REG10_O_RID_B_C_COMP_EN_MAX                 (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG10_O_RID_A_COMP_EN_MASK                  (0x00000040U)
#define CSL_USB3P0SS_PHY2_BC_REG10_O_RID_A_COMP_EN_SHIFT                 (0x00000006U)
#define CSL_USB3P0SS_PHY2_BC_REG10_O_RID_A_COMP_EN_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG10_O_RID_A_REF_EN_MASK                   (0x00000008U)
#define CSL_USB3P0SS_PHY2_BC_REG10_O_RID_A_REF_EN_SHIFT                  (0x00000003U)
#define CSL_USB3P0SS_PHY2_BC_REG10_O_RID_A_REF_EN_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG10_O_RID_NONFLOAT_SRC_EN_MASK            (0x00000002U)
#define CSL_USB3P0SS_PHY2_BC_REG10_O_RID_NONFLOAT_SRC_EN_SHIFT           (0x00000001U)
#define CSL_USB3P0SS_PHY2_BC_REG10_O_RID_NONFLOAT_SRC_EN_MAX             (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG10_O_RID_FLOAT_SRC_EN_MASK               (0x00000001U)
#define CSL_USB3P0SS_PHY2_BC_REG10_O_RID_FLOAT_SRC_EN_SHIFT              (0x00000000U)
#define CSL_USB3P0SS_PHY2_BC_REG10_O_RID_FLOAT_SRC_EN_MAX                (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG10_O_RID_C_REF_EN_MASK                   (0x00000020U)
#define CSL_USB3P0SS_PHY2_BC_REG10_O_RID_C_REF_EN_SHIFT                  (0x00000005U)
#define CSL_USB3P0SS_PHY2_BC_REG10_O_RID_C_REF_EN_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG10_O_RID_B_REF_EN_MASK                   (0x00000010U)
#define CSL_USB3P0SS_PHY2_BC_REG10_O_RID_B_REF_EN_SHIFT                  (0x00000004U)
#define CSL_USB3P0SS_PHY2_BC_REG10_O_RID_B_REF_EN_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG10_O_RID_FLOAT_REF_EN_MASK               (0x00000004U)
#define CSL_USB3P0SS_PHY2_BC_REG10_O_RID_FLOAT_REF_EN_SHIFT              (0x00000002U)
#define CSL_USB3P0SS_PHY2_BC_REG10_O_RID_FLOAT_REF_EN_MAX                (0x00000001U)

/* BC_REG11 */

#define CSL_USB3P0SS_PHY2_BC_REG11_O_DM_VLGC_COMP_EN_MASK                (0x00000001U)
#define CSL_USB3P0SS_PHY2_BC_REG11_O_DM_VLGC_COMP_EN_SHIFT               (0x00000000U)
#define CSL_USB3P0SS_PHY2_BC_REG11_O_DM_VLGC_COMP_EN_MAX                 (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG11_I_RID_A_COMP_STS_MASK                 (0x00000008U)
#define CSL_USB3P0SS_PHY2_BC_REG11_I_RID_A_COMP_STS_SHIFT                (0x00000003U)
#define CSL_USB3P0SS_PHY2_BC_REG11_I_RID_A_COMP_STS_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG11_I_AFE_RXDP_ANA_MASK                   (0x00000040U)
#define CSL_USB3P0SS_PHY2_BC_REG11_I_AFE_RXDP_ANA_SHIFT                  (0x00000006U)
#define CSL_USB3P0SS_PHY2_BC_REG11_I_AFE_RXDP_ANA_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG11_I_AFE_RXDM_ANA_MASK                   (0x00000020U)
#define CSL_USB3P0SS_PHY2_BC_REG11_I_AFE_RXDM_ANA_SHIFT                  (0x00000005U)
#define CSL_USB3P0SS_PHY2_BC_REG11_I_AFE_RXDM_ANA_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG11_I_DP_VDAT_REF_COMP_STS_MASK           (0x00000002U)
#define CSL_USB3P0SS_PHY2_BC_REG11_I_DP_VDAT_REF_COMP_STS_SHIFT          (0x00000001U)
#define CSL_USB3P0SS_PHY2_BC_REG11_I_DP_VDAT_REF_COMP_STS_MAX            (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG11_O_IDM_SRC_EN_MASK                     (0x00000080U)
#define CSL_USB3P0SS_PHY2_BC_REG11_O_IDM_SRC_EN_SHIFT                    (0x00000007U)
#define CSL_USB3P0SS_PHY2_BC_REG11_O_IDM_SRC_EN_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG11_I_DM_VDAT_REF_COMP_STS_MASK           (0x00000004U)
#define CSL_USB3P0SS_PHY2_BC_REG11_I_DM_VDAT_REF_COMP_STS_SHIFT          (0x00000002U)
#define CSL_USB3P0SS_PHY2_BC_REG11_I_DM_VDAT_REF_COMP_STS_MAX            (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG11_I_RID_B_C_COMP_STS_MASK               (0x00000010U)
#define CSL_USB3P0SS_PHY2_BC_REG11_I_RID_B_C_COMP_STS_SHIFT              (0x00000004U)
#define CSL_USB3P0SS_PHY2_BC_REG11_I_RID_B_C_COMP_STS_MAX                (0x00000001U)

/* BC_REG12 */

#define CSL_USB3P0SS_PHY2_BC_REG12_RID_B_COMP_STS_MASK                   (0x00000010U)
#define CSL_USB3P0SS_PHY2_BC_REG12_RID_B_COMP_STS_SHIFT                  (0x00000004U)
#define CSL_USB3P0SS_PHY2_BC_REG12_RID_B_COMP_STS_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG12_RID_FLOAT_COMP_STS_MASK               (0x00000040U)
#define CSL_USB3P0SS_PHY2_BC_REG12_RID_FLOAT_COMP_STS_SHIFT              (0x00000006U)
#define CSL_USB3P0SS_PHY2_BC_REG12_RID_FLOAT_COMP_STS_MAX                (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG12_RID_GND_COMP_STS_MASK                 (0x00000080U)
#define CSL_USB3P0SS_PHY2_BC_REG12_RID_GND_COMP_STS_SHIFT                (0x00000007U)
#define CSL_USB3P0SS_PHY2_BC_REG12_RID_GND_COMP_STS_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG12_DP_VDAT_REF_COMP_STS_MASK             (0x00000001U)
#define CSL_USB3P0SS_PHY2_BC_REG12_DP_VDAT_REF_COMP_STS_SHIFT            (0x00000000U)
#define CSL_USB3P0SS_PHY2_BC_REG12_DP_VDAT_REF_COMP_STS_MAX              (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG12_RID_C_COMP_STS_MASK                   (0x00000020U)
#define CSL_USB3P0SS_PHY2_BC_REG12_RID_C_COMP_STS_SHIFT                  (0x00000005U)
#define CSL_USB3P0SS_PHY2_BC_REG12_RID_C_COMP_STS_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG12_DM_VDAT_REF_COMP_STS_MASK             (0x00000002U)
#define CSL_USB3P0SS_PHY2_BC_REG12_DM_VDAT_REF_COMP_STS_SHIFT            (0x00000001U)
#define CSL_USB3P0SS_PHY2_BC_REG12_DM_VDAT_REF_COMP_STS_MAX              (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG12_RID_A_COMP_STS_MASK                   (0x00000008U)
#define CSL_USB3P0SS_PHY2_BC_REG12_RID_A_COMP_STS_SHIFT                  (0x00000003U)
#define CSL_USB3P0SS_PHY2_BC_REG12_RID_A_COMP_STS_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_BC_REG12_DM_VLGC_COMP_STS_MASK                 (0x00000004U)
#define CSL_USB3P0SS_PHY2_BC_REG12_DM_VLGC_COMP_STS_SHIFT                (0x00000002U)
#define CSL_USB3P0SS_PHY2_BC_REG12_DM_VLGC_COMP_STS_MAX                  (0x00000001U)

/* TED_REG3 */

#define CSL_USB3P0SS_PHY2_TED_REG3_UNUSED_MASK                           (0x00000003U)
#define CSL_USB3P0SS_PHY2_TED_REG3_UNUSED_SHIFT                          (0x00000000U)
#define CSL_USB3P0SS_PHY2_TED_REG3_UNUSED_MAX                            (0x00000003U)

#define CSL_USB3P0SS_PHY2_TED_REG3_CALIB_CODE_DOWN_MASK                  (0x000000F0U)
#define CSL_USB3P0SS_PHY2_TED_REG3_CALIB_CODE_DOWN_SHIFT                 (0x00000004U)
#define CSL_USB3P0SS_PHY2_TED_REG3_CALIB_CODE_DOWN_MAX                   (0x0000000FU)

#define CSL_USB3P0SS_PHY2_TED_REG3_CALIB_DONE_DOWN_MASK                  (0x00000004U)
#define CSL_USB3P0SS_PHY2_TED_REG3_CALIB_DONE_DOWN_SHIFT                 (0x00000002U)
#define CSL_USB3P0SS_PHY2_TED_REG3_CALIB_DONE_DOWN_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_TED_REG3_COMPARATOR_DOWN_MASK                  (0x00000008U)
#define CSL_USB3P0SS_PHY2_TED_REG3_COMPARATOR_DOWN_SHIFT                 (0x00000003U)
#define CSL_USB3P0SS_PHY2_TED_REG3_COMPARATOR_DOWN_MAX                   (0x00000001U)

/* TED_REG4 */

#define CSL_USB3P0SS_PHY2_TED_REG4_UNUSED_MASK                           (0x00000003U)
#define CSL_USB3P0SS_PHY2_TED_REG4_UNUSED_SHIFT                          (0x00000000U)
#define CSL_USB3P0SS_PHY2_TED_REG4_UNUSED_MAX                            (0x00000003U)

#define CSL_USB3P0SS_PHY2_TED_REG4_COMPARATOR_UP_MASK                    (0x00000008U)
#define CSL_USB3P0SS_PHY2_TED_REG4_COMPARATOR_UP_SHIFT                   (0x00000003U)
#define CSL_USB3P0SS_PHY2_TED_REG4_COMPARATOR_UP_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_TED_REG4_CALIB_CODE_UP_MASK                    (0x000000F0U)
#define CSL_USB3P0SS_PHY2_TED_REG4_CALIB_CODE_UP_SHIFT                   (0x00000004U)
#define CSL_USB3P0SS_PHY2_TED_REG4_CALIB_CODE_UP_MAX                     (0x0000000FU)

#define CSL_USB3P0SS_PHY2_TED_REG4_CALIB_DONE_UP_MASK                    (0x00000004U)
#define CSL_USB3P0SS_PHY2_TED_REG4_CALIB_DONE_UP_SHIFT                   (0x00000002U)
#define CSL_USB3P0SS_PHY2_TED_REG4_CALIB_DONE_UP_MAX                     (0x00000001U)

/* DIG_UNUSED_REG0 */

#define CSL_USB3P0SS_PHY2_DIG_UNUSED_REG0_GLITCH_FILTER_EN_MASK          (0x00000080U)
#define CSL_USB3P0SS_PHY2_DIG_UNUSED_REG0_GLITCH_FILTER_EN_SHIFT         (0x00000007U)
#define CSL_USB3P0SS_PHY2_DIG_UNUSED_REG0_GLITCH_FILTER_EN_MAX           (0x00000001U)

#define CSL_USB3P0SS_PHY2_DIG_UNUSED_REG0_UNUSED_MASK                    (0x0000007FU)
#define CSL_USB3P0SS_PHY2_DIG_UNUSED_REG0_UNUSED_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_PHY2_DIG_UNUSED_REG0_UNUSED_MAX                     (0x0000007FU)

/* DIG_UNUSED_REG1 */

#define CSL_USB3P0SS_PHY2_DIG_UNUSED_REG1_THRESHOLD_OVR_EN_MASK          (0x00000001U)
#define CSL_USB3P0SS_PHY2_DIG_UNUSED_REG1_THRESHOLD_OVR_EN_SHIFT         (0x00000000U)
#define CSL_USB3P0SS_PHY2_DIG_UNUSED_REG1_THRESHOLD_OVR_EN_MAX           (0x00000001U)

#define CSL_USB3P0SS_PHY2_DIG_UNUSED_REG1_THRESHOLD_OVR_VALUE_MSB_MASK   (0x00000006U)
#define CSL_USB3P0SS_PHY2_DIG_UNUSED_REG1_THRESHOLD_OVR_VALUE_MSB_SHIFT  (0x00000001U)
#define CSL_USB3P0SS_PHY2_DIG_UNUSED_REG1_THRESHOLD_OVR_VALUE_MSB_MAX    (0x00000003U)

#define CSL_USB3P0SS_PHY2_DIG_UNUSED_REG1_UNUSED_MASK                    (0x000000F8U)
#define CSL_USB3P0SS_PHY2_DIG_UNUSED_REG1_UNUSED_SHIFT                   (0x00000003U)
#define CSL_USB3P0SS_PHY2_DIG_UNUSED_REG1_UNUSED_MAX                     (0x0000001FU)

/* DIG_UNUSED_REG2 */

#define CSL_USB3P0SS_PHY2_DIG_UNUSED_REG2_THRESHOLD_OVR_VALUE_LSB_MASK   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_DIG_UNUSED_REG2_THRESHOLD_OVR_VALUE_LSB_SHIFT  (0x00000000U)
#define CSL_USB3P0SS_PHY2_DIG_UNUSED_REG2_THRESHOLD_OVR_VALUE_LSB_MAX    (0x000000FFU)

/* DIG_UNUSED_REG3 */

#define CSL_USB3P0SS_PHY2_DIG_UNUSED_REG3_UNUSED_MASK                    (0x000000FFU)
#define CSL_USB3P0SS_PHY2_DIG_UNUSED_REG3_UNUSED_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_PHY2_DIG_UNUSED_REG3_UNUSED_MAX                     (0x000000FFU)

/* INTERRUPT_REG1 */

#define CSL_USB3P0SS_PHY2_INTERRUPT_REG1_ISR_MASK                        (0x00000040U)
#define CSL_USB3P0SS_PHY2_INTERRUPT_REG1_ISR_SHIFT                       (0x00000006U)
#define CSL_USB3P0SS_PHY2_INTERRUPT_REG1_ISR_MAX                         (0x00000001U)

#define CSL_USB3P0SS_PHY2_INTERRUPT_REG1_UNUSED_MASK                     (0x0000003FU)
#define CSL_USB3P0SS_PHY2_INTERRUPT_REG1_UNUSED_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_PHY2_INTERRUPT_REG1_UNUSED_MAX                      (0x0000003FU)

#define CSL_USB3P0SS_PHY2_INTERRUPT_REG1_IRSR_MASK                       (0x00000080U)
#define CSL_USB3P0SS_PHY2_INTERRUPT_REG1_IRSR_SHIFT                      (0x00000007U)
#define CSL_USB3P0SS_PHY2_INTERRUPT_REG1_IRSR_MAX                        (0x00000001U)

/* INTERRUPT_REG2 */

#define CSL_USB3P0SS_PHY2_INTERRUPT_REG2_IMR_MASK                        (0x00000001U)
#define CSL_USB3P0SS_PHY2_INTERRUPT_REG2_IMR_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_PHY2_INTERRUPT_REG2_IMR_MAX                         (0x00000001U)

#define CSL_USB3P0SS_PHY2_INTERRUPT_REG2_UNUSED_MASK                     (0x000000FEU)
#define CSL_USB3P0SS_PHY2_INTERRUPT_REG2_UNUSED_SHIFT                    (0x00000001U)
#define CSL_USB3P0SS_PHY2_INTERRUPT_REG2_UNUSED_MAX                      (0x0000007FU)

/* RX_REG0 */

#define CSL_USB3P0SS_PHY2_RX_REG0_HS_SYNC_DET_BITS_MASK                  (0x00000008U)
#define CSL_USB3P0SS_PHY2_RX_REG0_HS_SYNC_DET_BITS_SHIFT                 (0x00000003U)
#define CSL_USB3P0SS_PHY2_RX_REG0_HS_SYNC_DET_BITS_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_RX_REG0_FSLS_TIMEOUT_EN_MASK                   (0x00000010U)
#define CSL_USB3P0SS_PHY2_RX_REG0_FSLS_TIMEOUT_EN_SHIFT                  (0x00000004U)
#define CSL_USB3P0SS_PHY2_RX_REG0_FSLS_TIMEOUT_EN_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_RX_REG0_FS_EOP_SE0_EN_MASK                     (0x00000001U)
#define CSL_USB3P0SS_PHY2_RX_REG0_FS_EOP_SE0_EN_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_PHY2_RX_REG0_FS_EOP_SE0_EN_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_RX_REG0_FSLS_NO_EOP_TIMEOUT_MASK               (0x000000E0U)
#define CSL_USB3P0SS_PHY2_RX_REG0_FSLS_NO_EOP_TIMEOUT_SHIFT              (0x00000005U)
#define CSL_USB3P0SS_PHY2_RX_REG0_FSLS_NO_EOP_TIMEOUT_MAX                (0x00000007U)

#define CSL_USB3P0SS_PHY2_RX_REG0_FS_EOP_SE0_THRESHOLD_MASK              (0x00000006U)
#define CSL_USB3P0SS_PHY2_RX_REG0_FS_EOP_SE0_THRESHOLD_SHIFT             (0x00000001U)
#define CSL_USB3P0SS_PHY2_RX_REG0_FS_EOP_SE0_THRESHOLD_MAX               (0x00000003U)

/* RX_REG1 */

#define CSL_USB3P0SS_PHY2_RX_REG1_LS_EOP_SE0_THRESHOLD_MASK              (0x000000FCU)
#define CSL_USB3P0SS_PHY2_RX_REG1_LS_EOP_SE0_THRESHOLD_SHIFT             (0x00000002U)
#define CSL_USB3P0SS_PHY2_RX_REG1_LS_EOP_SE0_THRESHOLD_MAX               (0x0000003FU)

#define CSL_USB3P0SS_PHY2_RX_REG1_FS_NO_EOP_TIMEOUT_EN_MASK              (0x00000001U)
#define CSL_USB3P0SS_PHY2_RX_REG1_FS_NO_EOP_TIMEOUT_EN_SHIFT             (0x00000000U)
#define CSL_USB3P0SS_PHY2_RX_REG1_FS_NO_EOP_TIMEOUT_EN_MAX               (0x00000001U)

#define CSL_USB3P0SS_PHY2_RX_REG1_LS_EOP_SE0_EN_MASK                     (0x00000002U)
#define CSL_USB3P0SS_PHY2_RX_REG1_LS_EOP_SE0_EN_SHIFT                    (0x00000001U)
#define CSL_USB3P0SS_PHY2_RX_REG1_LS_EOP_SE0_EN_MAX                      (0x00000001U)

/* TX_REG0 */

#define CSL_USB3P0SS_PHY2_TX_REG0_SOF_EXTENSION_EN_MASK                  (0x00000001U)
#define CSL_USB3P0SS_PHY2_TX_REG0_SOF_EXTENSION_EN_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_PHY2_TX_REG0_SOF_EXTENSION_EN_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_TX_REG0_UNUSED_MASK                            (0x000000F0U)
#define CSL_USB3P0SS_PHY2_TX_REG0_UNUSED_SHIFT                           (0x00000004U)
#define CSL_USB3P0SS_PHY2_TX_REG0_UNUSED_MAX                             (0x0000000FU)

#define CSL_USB3P0SS_PHY2_TX_REG0_SOF_EXTENSION_MASK                     (0x00000006U)
#define CSL_USB3P0SS_PHY2_TX_REG0_SOF_EXTENSION_SHIFT                    (0x00000001U)
#define CSL_USB3P0SS_PHY2_TX_REG0_SOF_EXTENSION_MAX                      (0x00000003U)

#define CSL_USB3P0SS_PHY2_TX_REG0_FS_PREAMBLE_EN_MASK                    (0x00000008U)
#define CSL_USB3P0SS_PHY2_TX_REG0_FS_PREAMBLE_EN_SHIFT                   (0x00000003U)
#define CSL_USB3P0SS_PHY2_TX_REG0_FS_PREAMBLE_EN_MAX                     (0x00000001U)

/* TX_REG1 */

#define CSL_USB3P0SS_PHY2_TX_REG1_PREAMBLE_VALUE_MASK                    (0x000000FFU)
#define CSL_USB3P0SS_PHY2_TX_REG1_PREAMBLE_VALUE_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_PHY2_TX_REG1_PREAMBLE_VALUE_MAX                     (0x000000FFU)

/* CDR_REG0 */

#define CSL_USB3P0SS_PHY2_CDR_REG0_SQUELCH_DELAY_MASK                    (0x00000006U)
#define CSL_USB3P0SS_PHY2_CDR_REG0_SQUELCH_DELAY_SHIFT                   (0x00000001U)
#define CSL_USB3P0SS_PHY2_CDR_REG0_SQUELCH_DELAY_MAX                     (0x00000003U)

#define CSL_USB3P0SS_PHY2_CDR_REG0_PLL_CLKDIV_EN_MASK                    (0x00000008U)
#define CSL_USB3P0SS_PHY2_CDR_REG0_PLL_CLKDIV_EN_SHIFT                   (0x00000003U)
#define CSL_USB3P0SS_PHY2_CDR_REG0_PLL_CLKDIV_EN_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_CDR_REG0_PLL_CLKDIV_MASK                       (0x00000030U)
#define CSL_USB3P0SS_PHY2_CDR_REG0_PLL_CLKDIV_SHIFT                      (0x00000004U)
#define CSL_USB3P0SS_PHY2_CDR_REG0_PLL_CLKDIV_MAX                        (0x00000003U)

#define CSL_USB3P0SS_PHY2_CDR_REG0_UNUSED_MASK                           (0x000000C0U)
#define CSL_USB3P0SS_PHY2_CDR_REG0_UNUSED_SHIFT                          (0x00000006U)
#define CSL_USB3P0SS_PHY2_CDR_REG0_UNUSED_MAX                            (0x00000003U)

#define CSL_USB3P0SS_PHY2_CDR_REG0_SQUELCH_DELAY_EN_MASK                 (0x00000001U)
#define CSL_USB3P0SS_PHY2_CDR_REG0_SQUELCH_DELAY_EN_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_PHY2_CDR_REG0_SQUELCH_DELAY_EN_MAX                  (0x00000001U)

/* CDR_REG1 */

#define CSL_USB3P0SS_PHY2_CDR_REG1_CALIB_COMP_OUT_MASK                   (0x00000040U)
#define CSL_USB3P0SS_PHY2_CDR_REG1_CALIB_COMP_OUT_SHIFT                  (0x00000006U)
#define CSL_USB3P0SS_PHY2_CDR_REG1_CALIB_COMP_OUT_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_CDR_REG1_CALIB_SPC_THRESHOLD_MASK              (0x00000038U)
#define CSL_USB3P0SS_PHY2_CDR_REG1_CALIB_SPC_THRESHOLD_SHIFT             (0x00000003U)
#define CSL_USB3P0SS_PHY2_CDR_REG1_CALIB_SPC_THRESHOLD_MAX               (0x00000007U)

#define CSL_USB3P0SS_PHY2_CDR_REG1_CALIB_SPC_THRESHOLD_EN_MASK           (0x00000004U)
#define CSL_USB3P0SS_PHY2_CDR_REG1_CALIB_SPC_THRESHOLD_EN_SHIFT          (0x00000002U)
#define CSL_USB3P0SS_PHY2_CDR_REG1_CALIB_SPC_THRESHOLD_EN_MAX            (0x00000001U)

#define CSL_USB3P0SS_PHY2_CDR_REG1_CALIB_ITERATION_MASK                  (0x00000002U)
#define CSL_USB3P0SS_PHY2_CDR_REG1_CALIB_ITERATION_SHIFT                 (0x00000001U)
#define CSL_USB3P0SS_PHY2_CDR_REG1_CALIB_ITERATION_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_CDR_REG1_DYNAMIC_CALIB_EN_MASK                 (0x00000001U)
#define CSL_USB3P0SS_PHY2_CDR_REG1_DYNAMIC_CALIB_EN_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_PHY2_CDR_REG1_DYNAMIC_CALIB_EN_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_CDR_REG1_UNUSED_MASK                           (0x00000080U)
#define CSL_USB3P0SS_PHY2_CDR_REG1_UNUSED_SHIFT                          (0x00000007U)
#define CSL_USB3P0SS_PHY2_CDR_REG1_UNUSED_MAX                            (0x00000001U)

/* CDR_REG2 */

#define CSL_USB3P0SS_PHY2_CDR_REG2_CALIB_CLOCK_EN_MASK                   (0x00000002U)
#define CSL_USB3P0SS_PHY2_CDR_REG2_CALIB_CLOCK_EN_SHIFT                  (0x00000001U)
#define CSL_USB3P0SS_PHY2_CDR_REG2_CALIB_CLOCK_EN_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_CDR_REG2_HSRX_EN_DEL_MASK                      (0x00000040U)
#define CSL_USB3P0SS_PHY2_CDR_REG2_HSRX_EN_DEL_SHIFT                     (0x00000006U)
#define CSL_USB3P0SS_PHY2_CDR_REG2_HSRX_EN_DEL_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_CDR_REG2_CALIB_CLOCK_MASK                      (0x00000004U)
#define CSL_USB3P0SS_PHY2_CDR_REG2_CALIB_CLOCK_SHIFT                     (0x00000002U)
#define CSL_USB3P0SS_PHY2_CDR_REG2_CALIB_CLOCK_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_CDR_REG2_HSRX_EN_MASK                          (0x00000010U)
#define CSL_USB3P0SS_PHY2_CDR_REG2_HSRX_EN_SHIFT                         (0x00000004U)
#define CSL_USB3P0SS_PHY2_CDR_REG2_HSRX_EN_MAX                           (0x00000001U)

#define CSL_USB3P0SS_PHY2_CDR_REG2_CALIB_OUT_EN_MASK                     (0x00000001U)
#define CSL_USB3P0SS_PHY2_CDR_REG2_CALIB_OUT_EN_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_PHY2_CDR_REG2_CALIB_OUT_EN_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_CDR_REG2_HSRX_EN_DEL_EN_MASK                   (0x00000020U)
#define CSL_USB3P0SS_PHY2_CDR_REG2_HSRX_EN_DEL_EN_SHIFT                  (0x00000005U)
#define CSL_USB3P0SS_PHY2_CDR_REG2_HSRX_EN_DEL_EN_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_CDR_REG2_UNUSED_MASK                           (0x00000080U)
#define CSL_USB3P0SS_PHY2_CDR_REG2_UNUSED_SHIFT                          (0x00000007U)
#define CSL_USB3P0SS_PHY2_CDR_REG2_UNUSED_MAX                            (0x00000001U)

#define CSL_USB3P0SS_PHY2_CDR_REG2_HSRX_EN_EN_MASK                       (0x00000008U)
#define CSL_USB3P0SS_PHY2_CDR_REG2_HSRX_EN_EN_SHIFT                      (0x00000003U)
#define CSL_USB3P0SS_PHY2_CDR_REG2_HSRX_EN_EN_MAX                        (0x00000001U)

/* CDR_REG3 */

#define CSL_USB3P0SS_PHY2_CDR_REG3_CALIB_ACTIVE_MASK                     (0x00000080U)
#define CSL_USB3P0SS_PHY2_CDR_REG3_CALIB_ACTIVE_SHIFT                    (0x00000007U)
#define CSL_USB3P0SS_PHY2_CDR_REG3_CALIB_ACTIVE_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_CDR_REG3_CALIB_DONE_MASK                       (0x00000040U)
#define CSL_USB3P0SS_PHY2_CDR_REG3_CALIB_DONE_SHIFT                      (0x00000006U)
#define CSL_USB3P0SS_PHY2_CDR_REG3_CALIB_DONE_MAX                        (0x00000001U)

#define CSL_USB3P0SS_PHY2_CDR_REG3_CALIB_CODE_MASK                       (0x0000003FU)
#define CSL_USB3P0SS_PHY2_CDR_REG3_CALIB_CODE_SHIFT                      (0x00000000U)
#define CSL_USB3P0SS_PHY2_CDR_REG3_CALIB_CODE_MAX                        (0x0000003FU)

/* CDR_REG4 */

#define CSL_USB3P0SS_PHY2_CDR_REG4_LATENCY_THRESHOLD_MASK                (0x00000018U)
#define CSL_USB3P0SS_PHY2_CDR_REG4_LATENCY_THRESHOLD_SHIFT               (0x00000003U)
#define CSL_USB3P0SS_PHY2_CDR_REG4_LATENCY_THRESHOLD_MAX                 (0x00000003U)

#define CSL_USB3P0SS_PHY2_CDR_REG4_CLK_GATE_SQ_MASK_MASK                 (0x00000020U)
#define CSL_USB3P0SS_PHY2_CDR_REG4_CLK_GATE_SQ_MASK_SHIFT                (0x00000005U)
#define CSL_USB3P0SS_PHY2_CDR_REG4_CLK_GATE_SQ_MASK_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_CDR_REG4_CLK_GATE_EN_MASK                      (0x00000040U)
#define CSL_USB3P0SS_PHY2_CDR_REG4_CLK_GATE_EN_SHIFT                     (0x00000006U)
#define CSL_USB3P0SS_PHY2_CDR_REG4_CLK_GATE_EN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_CDR_REG4_FILTER_EN_MASK                        (0x00000001U)
#define CSL_USB3P0SS_PHY2_CDR_REG4_FILTER_EN_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_PHY2_CDR_REG4_FILTER_EN_MAX                         (0x00000001U)

#define CSL_USB3P0SS_PHY2_CDR_REG4_DECISION_ERROR_EN_MASK                (0x00000002U)
#define CSL_USB3P0SS_PHY2_CDR_REG4_DECISION_ERROR_EN_SHIFT               (0x00000001U)
#define CSL_USB3P0SS_PHY2_CDR_REG4_DECISION_ERROR_EN_MAX                 (0x00000001U)

#define CSL_USB3P0SS_PHY2_CDR_REG4_CLK_GATE_VALUE_MASK                   (0x00000080U)
#define CSL_USB3P0SS_PHY2_CDR_REG4_CLK_GATE_VALUE_SHIFT                  (0x00000007U)
#define CSL_USB3P0SS_PHY2_CDR_REG4_CLK_GATE_VALUE_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_CDR_REG4_LATENCY_THRESHOLD_EN_MASK             (0x00000004U)
#define CSL_USB3P0SS_PHY2_CDR_REG4_LATENCY_THRESHOLD_EN_SHIFT            (0x00000002U)
#define CSL_USB3P0SS_PHY2_CDR_REG4_LATENCY_THRESHOLD_EN_MAX              (0x00000001U)

/* CDR_REG5 */

#define CSL_USB3P0SS_PHY2_CDR_REG5_SAMPLE_5X_EN_MASK                     (0x00000004U)
#define CSL_USB3P0SS_PHY2_CDR_REG5_SAMPLE_5X_EN_SHIFT                    (0x00000002U)
#define CSL_USB3P0SS_PHY2_CDR_REG5_SAMPLE_5X_EN_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_CDR_REG5_SMALL_PULSE_MASK                      (0x00000002U)
#define CSL_USB3P0SS_PHY2_CDR_REG5_SMALL_PULSE_SHIFT                     (0x00000001U)
#define CSL_USB3P0SS_PHY2_CDR_REG5_SMALL_PULSE_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_CDR_REG5_UNUSED_MASK                           (0x000000F8U)
#define CSL_USB3P0SS_PHY2_CDR_REG5_UNUSED_SHIFT                          (0x00000003U)
#define CSL_USB3P0SS_PHY2_CDR_REG5_UNUSED_MAX                            (0x0000001FU)

#define CSL_USB3P0SS_PHY2_CDR_REG5_SMALL_PULSE_EN_MASK                   (0x00000001U)
#define CSL_USB3P0SS_PHY2_CDR_REG5_SMALL_PULSE_EN_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_CDR_REG5_SMALL_PULSE_EN_MAX                    (0x00000001U)

/* CDR_REG6 */

#define CSL_USB3P0SS_PHY2_CDR_REG6_UNUSED_MASK                           (0x000000FFU)
#define CSL_USB3P0SS_PHY2_CDR_REG6_UNUSED_SHIFT                          (0x00000000U)
#define CSL_USB3P0SS_PHY2_CDR_REG6_UNUSED_MAX                            (0x000000FFU)

/* CDR_REG7 */

#define CSL_USB3P0SS_PHY2_CDR_REG7_UNUSED_MASK                           (0x000000FFU)
#define CSL_USB3P0SS_PHY2_CDR_REG7_UNUSED_SHIFT                          (0x00000000U)
#define CSL_USB3P0SS_PHY2_CDR_REG7_UNUSED_MAX                            (0x000000FFU)

/* CDR_REG8 */

#define CSL_USB3P0SS_PHY2_CDR_REG8_UNUSED_MASK                           (0x000000FFU)
#define CSL_USB3P0SS_PHY2_CDR_REG8_UNUSED_SHIFT                          (0x00000000U)
#define CSL_USB3P0SS_PHY2_CDR_REG8_UNUSED_MAX                            (0x000000FFU)

/* RX_REG2 */

#define CSL_USB3P0SS_PHY2_RX_REG2_CDR_ERROR_MASK                         (0x00000040U)
#define CSL_USB3P0SS_PHY2_RX_REG2_CDR_ERROR_SHIFT                        (0x00000006U)
#define CSL_USB3P0SS_PHY2_RX_REG2_CDR_ERROR_MAX                          (0x00000001U)

#define CSL_USB3P0SS_PHY2_RX_REG2_EB_ERROR_MASK                          (0x00000080U)
#define CSL_USB3P0SS_PHY2_RX_REG2_EB_ERROR_SHIFT                         (0x00000007U)
#define CSL_USB3P0SS_PHY2_RX_REG2_EB_ERROR_MAX                           (0x00000001U)

#define CSL_USB3P0SS_PHY2_RX_REG2_EOP_DETECTED_MASK                      (0x00000010U)
#define CSL_USB3P0SS_PHY2_RX_REG2_EOP_DETECTED_SHIFT                     (0x00000004U)
#define CSL_USB3P0SS_PHY2_RX_REG2_EOP_DETECTED_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_RX_REG2_ALIGNMENT_ERROR_MASK                   (0x00000002U)
#define CSL_USB3P0SS_PHY2_RX_REG2_ALIGNMENT_ERROR_SHIFT                  (0x00000001U)
#define CSL_USB3P0SS_PHY2_RX_REG2_ALIGNMENT_ERROR_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_RX_REG2_NO_EOP_MASK                            (0x00000001U)
#define CSL_USB3P0SS_PHY2_RX_REG2_NO_EOP_SHIFT                           (0x00000000U)
#define CSL_USB3P0SS_PHY2_RX_REG2_NO_EOP_MAX                             (0x00000001U)

#define CSL_USB3P0SS_PHY2_RX_REG2_SYNC_DETECTED_MASK                     (0x00000020U)
#define CSL_USB3P0SS_PHY2_RX_REG2_SYNC_DETECTED_SHIFT                    (0x00000005U)
#define CSL_USB3P0SS_PHY2_RX_REG2_SYNC_DETECTED_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_RX_REG2_NORMAL_EOP_MASK                        (0x00000004U)
#define CSL_USB3P0SS_PHY2_RX_REG2_NORMAL_EOP_SHIFT                       (0x00000002U)
#define CSL_USB3P0SS_PHY2_RX_REG2_NORMAL_EOP_MAX                         (0x00000001U)

#define CSL_USB3P0SS_PHY2_RX_REG2_HS_EOP_CONDITION_MASK                  (0x00000008U)
#define CSL_USB3P0SS_PHY2_RX_REG2_HS_EOP_CONDITION_SHIFT                 (0x00000003U)
#define CSL_USB3P0SS_PHY2_RX_REG2_HS_EOP_CONDITION_MAX                   (0x00000001U)

/* RX_REG3 */

#define CSL_USB3P0SS_PHY2_RX_REG3_START_FLAG_MASK                        (0x00000001U)
#define CSL_USB3P0SS_PHY2_RX_REG3_START_FLAG_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_PHY2_RX_REG3_START_FLAG_MAX                         (0x00000001U)

#define CSL_USB3P0SS_PHY2_RX_REG3_RX_STATE_BITUNSTUFF_MASK               (0x0000000EU)
#define CSL_USB3P0SS_PHY2_RX_REG3_RX_STATE_BITUNSTUFF_SHIFT              (0x00000001U)
#define CSL_USB3P0SS_PHY2_RX_REG3_RX_STATE_BITUNSTUFF_MAX                (0x00000007U)

#define CSL_USB3P0SS_PHY2_RX_REG3_SE0_VALIDATED_MASK                     (0x00000040U)
#define CSL_USB3P0SS_PHY2_RX_REG3_SE0_VALIDATED_SHIFT                    (0x00000006U)
#define CSL_USB3P0SS_PHY2_RX_REG3_SE0_VALIDATED_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_RX_REG3_LSFS_EOP_DETECTED_MASK                 (0x00000020U)
#define CSL_USB3P0SS_PHY2_RX_REG3_LSFS_EOP_DETECTED_SHIFT                (0x00000005U)
#define CSL_USB3P0SS_PHY2_RX_REG3_LSFS_EOP_DETECTED_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_RX_REG3_BIT_UNSTUFF_ERROR_MASK                 (0x00000010U)
#define CSL_USB3P0SS_PHY2_RX_REG3_BIT_UNSTUFF_ERROR_SHIFT                (0x00000004U)
#define CSL_USB3P0SS_PHY2_RX_REG3_BIT_UNSTUFF_ERROR_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_RX_REG3_HS_EOP_DETECTED_MASK                   (0x00000080U)
#define CSL_USB3P0SS_PHY2_RX_REG3_HS_EOP_DETECTED_SHIFT                  (0x00000007U)
#define CSL_USB3P0SS_PHY2_RX_REG3_HS_EOP_DETECTED_MAX                    (0x00000001U)

/* RX_REG4 */

#define CSL_USB3P0SS_PHY2_RX_REG4_DEASSERT_RXACTIVE_REG_MASK             (0x00000040U)
#define CSL_USB3P0SS_PHY2_RX_REG4_DEASSERT_RXACTIVE_REG_SHIFT            (0x00000006U)
#define CSL_USB3P0SS_PHY2_RX_REG4_DEASSERT_RXACTIVE_REG_MAX              (0x00000001U)

#define CSL_USB3P0SS_PHY2_RX_REG4_RXACTIVE_REG_MASK                      (0x00000080U)
#define CSL_USB3P0SS_PHY2_RX_REG4_RXACTIVE_REG_SHIFT                     (0x00000007U)
#define CSL_USB3P0SS_PHY2_RX_REG4_RXACTIVE_REG_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_RX_REG4_UNUSED_MASK                            (0x0000003FU)
#define CSL_USB3P0SS_PHY2_RX_REG4_UNUSED_SHIFT                           (0x00000000U)
#define CSL_USB3P0SS_PHY2_RX_REG4_UNUSED_MAX                             (0x0000003FU)

/* RX_REG5 */

#define CSL_USB3P0SS_PHY2_RX_REG5_SIE_CNT_UPPER_MASK                     (0x000000FFU)
#define CSL_USB3P0SS_PHY2_RX_REG5_SIE_CNT_UPPER_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_PHY2_RX_REG5_SIE_CNT_UPPER_MAX                      (0x000000FFU)

/* RX_REG6 */

#define CSL_USB3P0SS_PHY2_RX_REG6_PHY_CNT_UPPER_MASK                     (0x000000FFU)
#define CSL_USB3P0SS_PHY2_RX_REG6_PHY_CNT_UPPER_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_PHY2_RX_REG6_PHY_CNT_UPPER_MAX                      (0x000000FFU)

/* RX_REG7 */

#define CSL_USB3P0SS_PHY2_RX_REG7_PHY_CNT_LOWER_MASK                     (0x000000F0U)
#define CSL_USB3P0SS_PHY2_RX_REG7_PHY_CNT_LOWER_SHIFT                    (0x00000004U)
#define CSL_USB3P0SS_PHY2_RX_REG7_PHY_CNT_LOWER_MAX                      (0x0000000FU)

#define CSL_USB3P0SS_PHY2_RX_REG7_SIE_CNT_LOWER_MASK                     (0x0000000FU)
#define CSL_USB3P0SS_PHY2_RX_REG7_SIE_CNT_LOWER_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_PHY2_RX_REG7_SIE_CNT_LOWER_MAX                      (0x0000000FU)

/* TX_REG2 */

#define CSL_USB3P0SS_PHY2_TX_REG2_EOP_TRANSMITTED_MASK                   (0x00000008U)
#define CSL_USB3P0SS_PHY2_TX_REG2_EOP_TRANSMITTED_SHIFT                  (0x00000003U)
#define CSL_USB3P0SS_PHY2_TX_REG2_EOP_TRANSMITTED_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_TX_REG2_REMOTE_WAKEUP_MASK                     (0x00000001U)
#define CSL_USB3P0SS_PHY2_TX_REG2_REMOTE_WAKEUP_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_PHY2_TX_REG2_REMOTE_WAKEUP_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_TX_REG2_HS_BITSTUFF_EN_MASK                    (0x00000004U)
#define CSL_USB3P0SS_PHY2_TX_REG2_HS_BITSTUFF_EN_SHIFT                   (0x00000002U)
#define CSL_USB3P0SS_PHY2_TX_REG2_HS_BITSTUFF_EN_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_TX_REG2_TX_HS_STATE_MASK                       (0x000000F0U)
#define CSL_USB3P0SS_PHY2_TX_REG2_TX_HS_STATE_SHIFT                      (0x00000004U)
#define CSL_USB3P0SS_PHY2_TX_REG2_TX_HS_STATE_MAX                        (0x0000000FU)

#define CSL_USB3P0SS_PHY2_TX_REG2_RESUME_EOP_MASK                        (0x00000002U)
#define CSL_USB3P0SS_PHY2_TX_REG2_RESUME_EOP_SHIFT                       (0x00000001U)
#define CSL_USB3P0SS_PHY2_TX_REG2_RESUME_EOP_MAX                         (0x00000001U)

/* TX_REG3 */

#define CSL_USB3P0SS_PHY2_TX_REG3_PREAMBLE_SENT_MASK                     (0x00000001U)
#define CSL_USB3P0SS_PHY2_TX_REG3_PREAMBLE_SENT_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_PHY2_TX_REG3_PREAMBLE_SENT_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_TX_REG3_PD_STATE_MASK                          (0x0000000EU)
#define CSL_USB3P0SS_PHY2_TX_REG3_PD_STATE_SHIFT                         (0x00000001U)
#define CSL_USB3P0SS_PHY2_TX_REG3_PD_STATE_MAX                           (0x00000007U)

#define CSL_USB3P0SS_PHY2_TX_REG3_TX_LSFS_STATE_MASK                     (0x000000F0U)
#define CSL_USB3P0SS_PHY2_TX_REG3_TX_LSFS_STATE_SHIFT                    (0x00000004U)
#define CSL_USB3P0SS_PHY2_TX_REG3_TX_LSFS_STATE_MAX                      (0x0000000FU)

/* TX_REG4 */

#define CSL_USB3P0SS_PHY2_TX_REG4_LS_KEEP_ALIVE_MASK                     (0x00000001U)
#define CSL_USB3P0SS_PHY2_TX_REG4_LS_KEEP_ALIVE_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_PHY2_TX_REG4_LS_KEEP_ALIVE_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_TX_REG4_LSFS_BITSTUFF_EN_MASK                  (0x00000002U)
#define CSL_USB3P0SS_PHY2_TX_REG4_LSFS_BITSTUFF_EN_SHIFT                 (0x00000001U)
#define CSL_USB3P0SS_PHY2_TX_REG4_LSFS_BITSTUFF_EN_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_TX_REG4_UNUSED_MASK                            (0x000000FCU)
#define CSL_USB3P0SS_PHY2_TX_REG4_UNUSED_SHIFT                           (0x00000002U)
#define CSL_USB3P0SS_PHY2_TX_REG4_UNUSED_MAX                             (0x0000003FU)

/* CDR_REG9 */

#define CSL_USB3P0SS_PHY2_CDR_REG9_I_ANA_COMP_OUT_MASK                   (0x00000004U)
#define CSL_USB3P0SS_PHY2_CDR_REG9_I_ANA_COMP_OUT_SHIFT                  (0x00000002U)
#define CSL_USB3P0SS_PHY2_CDR_REG9_I_ANA_COMP_OUT_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_CDR_REG9_ANA_CALIB_ACTIVE_MASK                 (0x00000001U)
#define CSL_USB3P0SS_PHY2_CDR_REG9_ANA_CALIB_ACTIVE_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_PHY2_CDR_REG9_ANA_CALIB_ACTIVE_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_CDR_REG9_SAMPLER_CALIB_DONE_MASK               (0x00000002U)
#define CSL_USB3P0SS_PHY2_CDR_REG9_SAMPLER_CALIB_DONE_SHIFT              (0x00000001U)
#define CSL_USB3P0SS_PHY2_CDR_REG9_SAMPLER_CALIB_DONE_MAX                (0x00000001U)

#define CSL_USB3P0SS_PHY2_CDR_REG9_UNUSED_MASK                           (0x000000F8U)
#define CSL_USB3P0SS_PHY2_CDR_REG9_UNUSED_SHIFT                          (0x00000003U)
#define CSL_USB3P0SS_PHY2_CDR_REG9_UNUSED_MAX                            (0x0000001FU)

/* CDR_REG10 */

#define CSL_USB3P0SS_PHY2_CDR_REG10_UNUSED_MASK                          (0x000000C0U)
#define CSL_USB3P0SS_PHY2_CDR_REG10_UNUSED_SHIFT                         (0x00000006U)
#define CSL_USB3P0SS_PHY2_CDR_REG10_UNUSED_MAX                           (0x00000003U)

#define CSL_USB3P0SS_PHY2_CDR_REG10_CALIB_CODE_MASK                      (0x0000003FU)
#define CSL_USB3P0SS_PHY2_CDR_REG10_CALIB_CODE_SHIFT                     (0x00000000U)
#define CSL_USB3P0SS_PHY2_CDR_REG10_CALIB_CODE_MAX                       (0x0000003FU)

/* CDR_REG11 */

#define CSL_USB3P0SS_PHY2_CDR_REG11_O_HSRX_REC_DICISION_ERROR_MASK       (0x00000008U)
#define CSL_USB3P0SS_PHY2_CDR_REG11_O_HSRX_REC_DICISION_ERROR_SHIFT      (0x00000003U)
#define CSL_USB3P0SS_PHY2_CDR_REG11_O_HSRX_REC_DICISION_ERROR_MAX        (0x00000001U)

#define CSL_USB3P0SS_PHY2_CDR_REG11_RECEIVE_START_MASK                   (0x00000002U)
#define CSL_USB3P0SS_PHY2_CDR_REG11_RECEIVE_START_SHIFT                  (0x00000001U)
#define CSL_USB3P0SS_PHY2_CDR_REG11_RECEIVE_START_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_CDR_REG11_SMALL_PULSE_MASK                     (0x000000F0U)
#define CSL_USB3P0SS_PHY2_CDR_REG11_SMALL_PULSE_SHIFT                    (0x00000004U)
#define CSL_USB3P0SS_PHY2_CDR_REG11_SMALL_PULSE_MAX                      (0x0000000FU)

#define CSL_USB3P0SS_PHY2_CDR_REG11_O_ANA_CLK_GATE_MASK                  (0x00000004U)
#define CSL_USB3P0SS_PHY2_CDR_REG11_O_ANA_CLK_GATE_SHIFT                 (0x00000002U)
#define CSL_USB3P0SS_PHY2_CDR_REG11_O_ANA_CLK_GATE_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_CDR_REG11_I_ANA_TED_SQUELCH_MASK               (0x00000001U)
#define CSL_USB3P0SS_PHY2_CDR_REG11_I_ANA_TED_SQUELCH_SHIFT              (0x00000000U)
#define CSL_USB3P0SS_PHY2_CDR_REG11_I_ANA_TED_SQUELCH_MAX                (0x00000001U)

/* CDR_RE12 */

#define CSL_USB3P0SS_PHY2_CDR_RE12_UNUSED_MASK                           (0x000000FFU)
#define CSL_USB3P0SS_PHY2_CDR_RE12_UNUSED_SHIFT                          (0x00000000U)
#define CSL_USB3P0SS_PHY2_CDR_RE12_UNUSED_MAX                            (0x000000FFU)

/* DIG_TXRX_UNUSED_REG0 */

#define CSL_USB3P0SS_PHY2_DIG_TXRX_UNUSED_REG0_UNUSED_MASK               (0x000000FFU)
#define CSL_USB3P0SS_PHY2_DIG_TXRX_UNUSED_REG0_UNUSED_SHIFT              (0x00000000U)
#define CSL_USB3P0SS_PHY2_DIG_TXRX_UNUSED_REG0_UNUSED_MAX                (0x000000FFU)

/* DIG_TXRX_UNUSED_REG1 */

#define CSL_USB3P0SS_PHY2_DIG_TXRX_UNUSED_REG1_UNUSED_MASK               (0x000000FFU)
#define CSL_USB3P0SS_PHY2_DIG_TXRX_UNUSED_REG1_UNUSED_SHIFT              (0x00000000U)
#define CSL_USB3P0SS_PHY2_DIG_TXRX_UNUSED_REG1_UNUSED_MAX                (0x000000FFU)

/* DIG_TXRX_UNUSED_REG2 */

#define CSL_USB3P0SS_PHY2_DIG_TXRX_UNUSED_REG2_UNUSED_MASK               (0x000000FFU)
#define CSL_USB3P0SS_PHY2_DIG_TXRX_UNUSED_REG2_UNUSED_SHIFT              (0x00000000U)
#define CSL_USB3P0SS_PHY2_DIG_TXRX_UNUSED_REG2_UNUSED_MAX                (0x000000FFU)

/* DIG_TXRX_UNUSED_REG3 */

#define CSL_USB3P0SS_PHY2_DIG_TXRX_UNUSED_REG3_UNUSED_MASK               (0x000000FFU)
#define CSL_USB3P0SS_PHY2_DIG_TXRX_UNUSED_REG3_UNUSED_SHIFT              (0x00000000U)
#define CSL_USB3P0SS_PHY2_DIG_TXRX_UNUSED_REG3_UNUSED_MAX                (0x000000FFU)

/* UTMI_REG0 */

#define CSL_USB3P0SS_PHY2_UTMI_REG0_BIST_MODE_SEL_MASK                   (0x0000001EU)
#define CSL_USB3P0SS_PHY2_UTMI_REG0_BIST_MODE_SEL_SHIFT                  (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG0_BIST_MODE_SEL_MAX                    (0x0000000FU)

#define CSL_USB3P0SS_PHY2_UTMI_REG0_LOOPBACK_SEL_MASK                    (0x000000C0U)
#define CSL_USB3P0SS_PHY2_UTMI_REG0_LOOPBACK_SEL_SHIFT                   (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG0_LOOPBACK_SEL_MAX                     (0x00000003U)

#define CSL_USB3P0SS_PHY2_UTMI_REG0_BIST_EN_MASK                         (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG0_BIST_EN_SHIFT                        (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG0_BIST_EN_MAX                          (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG0_LOOPBACK_EN_MASK                     (0x00000020U)
#define CSL_USB3P0SS_PHY2_UTMI_REG0_LOOPBACK_EN_SHIFT                    (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG0_LOOPBACK_EN_MAX                      (0x00000001U)

/* UTMI_REG1 */

#define CSL_USB3P0SS_PHY2_UTMI_REG1_PHY_SOFT_RST_MASK                    (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG1_PHY_SOFT_RST_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG1_PHY_SOFT_RST_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG1_BIST_ERR_MASK                        (0x000000C0U)
#define CSL_USB3P0SS_PHY2_UTMI_REG1_BIST_ERR_SHIFT                       (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG1_BIST_ERR_MAX                         (0x00000003U)

#define CSL_USB3P0SS_PHY2_UTMI_REG1_BIST_SOFT_RST_MASK                   (0x00000020U)
#define CSL_USB3P0SS_PHY2_UTMI_REG1_BIST_SOFT_RST_SHIFT                  (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG1_BIST_SOFT_RST_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG1_CLKDIV_SOFT_RST_MASK                 (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG1_CLKDIV_SOFT_RST_SHIFT                (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG1_CLKDIV_SOFT_RST_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG1_TX_HS_SOFT_RST_MASK                  (0x00000008U)
#define CSL_USB3P0SS_PHY2_UTMI_REG1_TX_HS_SOFT_RST_SHIFT                 (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG1_TX_HS_SOFT_RST_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG1_CALIB_SOFT_RST_MASK                  (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG1_CALIB_SOFT_RST_SHIFT                 (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG1_CALIB_SOFT_RST_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG1_TX_LSFS_SOFT_RST_MASK                (0x00000010U)
#define CSL_USB3P0SS_PHY2_UTMI_REG1_TX_LSFS_SOFT_RST_SHIFT               (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG1_TX_LSFS_SOFT_RST_MAX                 (0x00000001U)

/* UTMI_REG2 */

#define CSL_USB3P0SS_PHY2_UTMI_REG2_BITUNSTUFF_SOFT_RST_MASK             (0x00000020U)
#define CSL_USB3P0SS_PHY2_UTMI_REG2_BITUNSTUFF_SOFT_RST_SHIFT            (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG2_BITUNSTUFF_SOFT_RST_MAX              (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG2_LSFS_DLL_SOFT_RST_MASK               (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG2_LSFS_DLL_SOFT_RST_SHIFT              (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG2_LSFS_DLL_SOFT_RST_MAX                (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG2_SHIFT_REG_SOFT_RST_MASK              (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG2_SHIFT_REG_SOFT_RST_SHIFT             (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG2_SHIFT_REG_SOFT_RST_MAX               (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG2_RX_HS_SOFT_RST_MASK                  (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG2_RX_HS_SOFT_RST_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG2_RX_HS_SOFT_RST_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG2_RX_CNTRL_SOFT_RST_MASK               (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG2_RX_CNTRL_SOFT_RST_SHIFT              (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG2_RX_CNTRL_SOFT_RST_MAX                (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG2_NRZI_DEC_SOFT_RST_MASK               (0x00000010U)
#define CSL_USB3P0SS_PHY2_UTMI_REG2_NRZI_DEC_SOFT_RST_SHIFT              (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG2_NRZI_DEC_SOFT_RST_MAX                (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG2_EOP_DET_SOFT_RST_MASK                (0x00000008U)
#define CSL_USB3P0SS_PHY2_UTMI_REG2_EOP_DET_SOFT_RST_SHIFT               (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG2_EOP_DET_SOFT_RST_MAX                 (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG2_SYNC_DET_SOFT_RST_MASK               (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG2_SYNC_DET_SOFT_RST_SHIFT              (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG2_SYNC_DET_SOFT_RST_MAX                (0x00000001U)

/* UTMI_REG3 */

#define CSL_USB3P0SS_PHY2_UTMI_REG3_HS_RX_ERR_MASK                       (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG3_HS_RX_ERR_SHIFT                      (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG3_HS_RX_ERR_MAX                        (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG3_FS_LINESTATE_FIL_EN_MASK             (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG3_FS_LINESTATE_FIL_EN_SHIFT            (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG3_FS_LINESTATE_FIL_EN_MAX              (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG3_FS_LINESTATE_FIL_CNT_MASK            (0x0000003EU)
#define CSL_USB3P0SS_PHY2_UTMI_REG3_FS_LINESTATE_FIL_CNT_SHIFT           (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG3_FS_LINESTATE_FIL_CNT_MAX             (0x0000001FU)

#define CSL_USB3P0SS_PHY2_UTMI_REG3_LS_LINESTATE_FIL_EN_MASK             (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG3_LS_LINESTATE_FIL_EN_SHIFT            (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG3_LS_LINESTATE_FIL_EN_MAX              (0x00000001U)

/* UTMI_REG4 */

#define CSL_USB3P0SS_PHY2_UTMI_REG4_LS_LINESTATE_FIL_CNT_MASK            (0x000000FFU)
#define CSL_USB3P0SS_PHY2_UTMI_REG4_LS_LINESTATE_FIL_CNT_SHIFT           (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG4_LS_LINESTATE_FIL_CNT_MAX             (0x000000FFU)

/* UTMI_REG5 */

#define CSL_USB3P0SS_PHY2_UTMI_REG5_HSRX_EN_MASK                         (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG5_HSRX_EN_SHIFT                        (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG5_HSRX_EN_MAX                          (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG5_HSRX_MASK                            (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG5_HSRX_SHIFT                           (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG5_HSRX_MAX                             (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG5_HS_SAMP_EN_MASK                      (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG5_HS_SAMP_EN_SHIFT                     (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG5_HS_SAMP_EN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG5_HSTX_BOOST_DEAMP_OFF_MASK            (0x00000020U)
#define CSL_USB3P0SS_PHY2_UTMI_REG5_HSTX_BOOST_DEAMP_OFF_SHIFT           (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG5_HSTX_BOOST_DEAMP_OFF_MAX             (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG5_BIST_MODE_EN_MASK                    (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG5_BIST_MODE_EN_SHIFT                   (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG5_BIST_MODE_EN_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG5_HS_SAMP_MASK                         (0x00000008U)
#define CSL_USB3P0SS_PHY2_UTMI_REG5_HS_SAMP_SHIFT                        (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG5_HS_SAMP_MAX                          (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG5_HSTX_BOOST_MASK                      (0x00000010U)
#define CSL_USB3P0SS_PHY2_UTMI_REG5_HSTX_BOOST_SHIFT                     (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG5_HSTX_BOOST_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG5_BIST_ON_MASK                         (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG5_BIST_ON_SHIFT                        (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG5_BIST_ON_MAX                          (0x00000001U)

/* UTMI_REG6 */

#define CSL_USB3P0SS_PHY2_UTMI_REG6_HS_DRVEN_THRESHOLD_MASK              (0x0000003EU)
#define CSL_USB3P0SS_PHY2_UTMI_REG6_HS_DRVEN_THRESHOLD_SHIFT             (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG6_HS_DRVEN_THRESHOLD_MAX               (0x0000001FU)

#define CSL_USB3P0SS_PHY2_UTMI_REG6_HS_DRVEN_TH_EN_MASK                  (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG6_HS_DRVEN_TH_EN_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG6_HS_DRVEN_TH_EN_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG6_VBUSVALID_CNTRL_MASK                 (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG6_VBUSVALID_CNTRL_SHIFT                (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG6_VBUSVALID_CNTRL_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG6_VBUSVALID_L3_DEV_EN_MASK             (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG6_VBUSVALID_L3_DEV_EN_SHIFT            (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG6_VBUSVALID_L3_DEV_EN_MAX              (0x00000001U)

/* UTMI_REG7 */

#define CSL_USB3P0SS_PHY2_UTMI_REG7_HSTX_MASK                            (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG7_HSTX_SHIFT                           (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG7_HSTX_MAX                             (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG7_HSTX_EN_MASK                         (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG7_HSTX_EN_SHIFT                        (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG7_HSTX_EN_MAX                          (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG7_HSTX_CHIRP_EN_MASK                   (0x00000010U)
#define CSL_USB3P0SS_PHY2_UTMI_REG7_HSTX_CHIRP_EN_SHIFT                  (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG7_HSTX_CHIRP_EN_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG7_HSTX_EN_DEL_EN_MASK                  (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG7_HSTX_EN_DEL_EN_SHIFT                 (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG7_HSTX_EN_DEL_EN_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG7_HSTX_BC_MODE_MASK                    (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG7_HSTX_BC_MODE_SHIFT                   (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG7_HSTX_BC_MODE_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG7_HSTX_BC_EN_MASK                      (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG7_HSTX_BC_EN_SHIFT                     (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG7_HSTX_BC_EN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG7_HSTX_EN_DEL_MASK                     (0x00000008U)
#define CSL_USB3P0SS_PHY2_UTMI_REG7_HSTX_EN_DEL_SHIFT                    (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG7_HSTX_EN_DEL_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG7_HSTX_CHIRP_MODE_MASK                 (0x00000020U)
#define CSL_USB3P0SS_PHY2_UTMI_REG7_HSTX_CHIRP_MODE_SHIFT                (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG7_HSTX_CHIRP_MODE_MAX                  (0x00000001U)

/* UTMI_REG8 */

#define CSL_USB3P0SS_PHY2_UTMI_REG8_HSTX_DATA_MASK                       (0x00000020U)
#define CSL_USB3P0SS_PHY2_UTMI_REG8_HSTX_DATA_SHIFT                      (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG8_HSTX_DATA_MAX                        (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG8_HS_TERM_MASK                         (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG8_HS_TERM_SHIFT                        (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG8_HS_TERM_MAX                          (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG8_HSTX_PREDRV_EN_MASK                  (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG8_HSTX_PREDRV_EN_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG8_HSTX_PREDRV_EN_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG8_HSTX_DRV_MASK                        (0x00000008U)
#define CSL_USB3P0SS_PHY2_UTMI_REG8_HSTX_DRV_SHIFT                       (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG8_HSTX_DRV_MAX                         (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG8_HS_TERM_EN_MASK                      (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG8_HS_TERM_EN_SHIFT                     (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG8_HS_TERM_EN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG8_HSTX_DRV_EN_MASK                     (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG8_HSTX_DRV_EN_SHIFT                    (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG8_HSTX_DRV_EN_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG8_HSTX_PREDRV_MASK                     (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG8_HSTX_PREDRV_SHIFT                    (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG8_HSTX_PREDRV_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG8_HSTX_DATA_EN_MASK                    (0x00000010U)
#define CSL_USB3P0SS_PHY2_UTMI_REG8_HSTX_DATA_EN_SHIFT                   (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG8_HSTX_DATA_EN_MAX                     (0x00000001U)

/* UTMI_REG9 */

#define CSL_USB3P0SS_PHY2_UTMI_REG9_HSTX_EN_DEL_TH_MASK                  (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG9_HSTX_EN_DEL_TH_SHIFT                 (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG9_HSTX_EN_DEL_TH_MAX                   (0x00000003U)

#define CSL_USB3P0SS_PHY2_UTMI_REG9_CLKOFF_EN_MASK                       (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG9_CLKOFF_EN_SHIFT                      (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG9_CLKOFF_EN_MAX                        (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG9_SDC_SPACE_MASK                       (0x00000070U)
#define CSL_USB3P0SS_PHY2_UTMI_REG9_SDC_SPACE_SHIFT                      (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG9_SDC_SPACE_MAX                        (0x00000007U)

#define CSL_USB3P0SS_PHY2_UTMI_REG9_SDC_SPACE_EN_MASK                    (0x00000008U)
#define CSL_USB3P0SS_PHY2_UTMI_REG9_SDC_SPACE_EN_SHIFT                   (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG9_SDC_SPACE_EN_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG9_HSTX_EN_DEL_TH_EN_MASK               (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG9_HSTX_EN_DEL_TH_EN_SHIFT              (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG9_HSTX_EN_DEL_TH_EN_MAX                (0x00000001U)

/* UTMI_REG10 */

#define CSL_USB3P0SS_PHY2_UTMI_REG10_PLL_CLKON_EN_MASK                   (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG10_PLL_CLKON_EN_SHIFT                  (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG10_PLL_CLKON_EN_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG10_LSFS_SERX_EN_MASK                   (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG10_LSFS_SERX_EN_SHIFT                  (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG10_LSFS_SERX_EN_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG10_BG_PD_BG_OK_MASK                    (0x00000020U)
#define CSL_USB3P0SS_PHY2_UTMI_REG10_BG_PD_BG_OK_SHIFT                   (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG10_BG_PD_BG_OK_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG10_PLL_CLKON_MASK                      (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG10_PLL_CLKON_SHIFT                     (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG10_PLL_CLKON_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG10_BG_PD_BG_OK_EN_MASK                 (0x00000010U)
#define CSL_USB3P0SS_PHY2_UTMI_REG10_BG_PD_BG_OK_EN_SHIFT                (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG10_BG_PD_BG_OK_EN_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG10_LSFS_RX_MASK                        (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG10_LSFS_RX_SHIFT                       (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG10_LSFS_RX_MAX                         (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG10_LSFS_SERX_MASK                      (0x00000008U)
#define CSL_USB3P0SS_PHY2_UTMI_REG10_LSFS_SERX_SHIFT                     (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG10_LSFS_SERX_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG10_LSFS_RX_EN_MASK                     (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG10_LSFS_RX_EN_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG10_LSFS_RX_EN_MAX                      (0x00000001U)

/* UTMI_REG11 */

#define CSL_USB3P0SS_PHY2_UTMI_REG11_SERX_MASK_THRESHOLD_MASK            (0x00000030U)
#define CSL_USB3P0SS_PHY2_UTMI_REG11_SERX_MASK_THRESHOLD_SHIFT           (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG11_SERX_MASK_THRESHOLD_MAX             (0x00000003U)

#define CSL_USB3P0SS_PHY2_UTMI_REG11_SERX_MASK_EN_MASK                   (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG11_SERX_MASK_EN_SHIFT                  (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG11_SERX_MASK_EN_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG11_LSFS_TX_EN_MASK                     (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG11_LSFS_TX_EN_SHIFT                    (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG11_LSFS_TX_EN_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG11_LSFS_TX_MASK                        (0x00000008U)
#define CSL_USB3P0SS_PHY2_UTMI_REG11_LSFS_TX_SHIFT                       (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG11_LSFS_TX_MAX                         (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG11_FSLS_EDGESEL_EN_MASK                (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG11_FSLS_EDGESEL_EN_SHIFT               (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG11_FSLS_EDGESEL_EN_MAX                 (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG11_FSLS_EDGESEL_MASK                   (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG11_FSLS_EDGESEL_SHIFT                  (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG11_FSLS_EDGESEL_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG11_CLEAN_LINESTATE_SERX_MASK_EN_MASK   (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG11_CLEAN_LINESTATE_SERX_MASK_EN_SHIFT  (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG11_CLEAN_LINESTATE_SERX_MASK_EN_MAX    (0x00000001U)

/* UTMI_REG12 */

#define CSL_USB3P0SS_PHY2_UTMI_REG12_FSLS_TX_DATA_EN_MASK                (0x00000010U)
#define CSL_USB3P0SS_PHY2_UTMI_REG12_FSLS_TX_DATA_EN_SHIFT               (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG12_FSLS_TX_DATA_EN_MAX                 (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG12_FSLS_TX_SE0_EN_MASK                 (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG12_FSLS_TX_SE0_EN_SHIFT                (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG12_FSLS_TX_SE0_EN_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG12_FSLS_TX_DATA_MASK                   (0x00000020U)
#define CSL_USB3P0SS_PHY2_UTMI_REG12_FSLS_TX_DATA_SHIFT                  (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG12_FSLS_TX_DATA_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG12_FSLS_TX_DRV_EN_MASK                 (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG12_FSLS_TX_DRV_EN_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG12_FSLS_TX_DRV_EN_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG12_FSLS_TX_DRV_MASK                    (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG12_FSLS_TX_DRV_SHIFT                   (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG12_FSLS_TX_DRV_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG12_SERX_BIAS_EN_MASK                   (0x000000C0U)
#define CSL_USB3P0SS_PHY2_UTMI_REG12_SERX_BIAS_EN_SHIFT                  (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG12_SERX_BIAS_EN_MAX                    (0x00000003U)

#define CSL_USB3P0SS_PHY2_UTMI_REG12_FSLS_TX_SE0_MASK                    (0x00000008U)
#define CSL_USB3P0SS_PHY2_UTMI_REG12_FSLS_TX_SE0_SHIFT                   (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG12_FSLS_TX_SE0_MAX                     (0x00000001U)

/* UTMI_REG13 */

#define CSL_USB3P0SS_PHY2_UTMI_REG13_FSLS_SERIALMODE_PULLUP2_EN_MASK     (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG13_FSLS_SERIALMODE_PULLUP2_EN_SHIFT    (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG13_FSLS_SERIALMODE_PULLUP2_EN_MAX      (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG13_LANE_REVERSE_MASK                   (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG13_LANE_REVERSE_SHIFT                  (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG13_LANE_REVERSE_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG13_FSLS_SERIALMODE_PULLUP2_MASK        (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG13_FSLS_SERIALMODE_PULLUP2_SHIFT       (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG13_FSLS_SERIALMODE_PULLUP2_MAX         (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG13_LANE_REVERSE_EN_MASK                (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG13_LANE_REVERSE_EN_SHIFT               (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG13_LANE_REVERSE_EN_MAX                 (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG13_DM_PULLDOWN_MASK                    (0x00000020U)
#define CSL_USB3P0SS_PHY2_UTMI_REG13_DM_PULLDOWN_SHIFT                   (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG13_DM_PULLDOWN_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG13_DM_PULLDOWN_EN_MASK                 (0x00000010U)
#define CSL_USB3P0SS_PHY2_UTMI_REG13_DM_PULLDOWN_EN_SHIFT                (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG13_DM_PULLDOWN_EN_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG13_DP_PULLDOWN_MASK                    (0x00000008U)
#define CSL_USB3P0SS_PHY2_UTMI_REG13_DP_PULLDOWN_SHIFT                   (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG13_DP_PULLDOWN_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG13_DP_PULLDOWN_EN_MASK                 (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG13_DP_PULLDOWN_EN_SHIFT                (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG13_DP_PULLDOWN_EN_MAX                  (0x00000001U)

/* UTMI_REG14 */

#define CSL_USB3P0SS_PHY2_UTMI_REG14_DM_PULLUP1_MASK                     (0x00000008U)
#define CSL_USB3P0SS_PHY2_UTMI_REG14_DM_PULLUP1_SHIFT                    (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG14_DM_PULLUP1_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG14_DP_PULLUP2_MASK                     (0x00000020U)
#define CSL_USB3P0SS_PHY2_UTMI_REG14_DP_PULLUP2_SHIFT                    (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG14_DP_PULLUP2_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG14_DP_PULLUP1_MASK                     (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG14_DP_PULLUP1_SHIFT                    (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG14_DP_PULLUP1_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG14_DM_PULLUP2_MASK                     (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG14_DM_PULLUP2_SHIFT                    (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG14_DM_PULLUP2_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG14_DM_PULLUP1_EN_MASK                  (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG14_DM_PULLUP1_EN_SHIFT                 (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG14_DM_PULLUP1_EN_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG14_DP_PULLUP2_EN_MASK                  (0x00000010U)
#define CSL_USB3P0SS_PHY2_UTMI_REG14_DP_PULLUP2_EN_SHIFT                 (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG14_DP_PULLUP2_EN_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG14_DM_PULLUP2_EN_MASK                  (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG14_DM_PULLUP2_EN_SHIFT                 (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG14_DM_PULLUP2_EN_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG14_DP_PULLUP1_EN_MASK                  (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG14_DP_PULLUP1_EN_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG14_DP_PULLUP1_EN_MAX                   (0x00000001U)

/* UTMI_REG15 */

#define CSL_USB3P0SS_PHY2_UTMI_REG15_TED_EN_CNT_MASK                     (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG15_TED_EN_CNT_SHIFT                    (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG15_TED_EN_CNT_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG15_TED_EN_VALUE_MASK                   (0x00000008U)
#define CSL_USB3P0SS_PHY2_UTMI_REG15_TED_EN_VALUE_SHIFT                  (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG15_TED_EN_VALUE_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG15_ED_EN_CNT_MASK                      (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG15_ED_EN_CNT_SHIFT                     (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG15_ED_EN_CNT_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG15_TXVALID_GATE_THRESHOLD_HS_MASK      (0x00000030U)
#define CSL_USB3P0SS_PHY2_UTMI_REG15_TXVALID_GATE_THRESHOLD_HS_SHIFT     (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG15_TXVALID_GATE_THRESHOLD_HS_MAX       (0x00000003U)

#define CSL_USB3P0SS_PHY2_UTMI_REG15_TXVALID_GATE_THRESHOLD_FS_MASK      (0x000000C0U)
#define CSL_USB3P0SS_PHY2_UTMI_REG15_TXVALID_GATE_THRESHOLD_FS_SHIFT     (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG15_TXVALID_GATE_THRESHOLD_FS_MAX       (0x00000003U)

#define CSL_USB3P0SS_PHY2_UTMI_REG15_ED_EN_VALUE_MASK                    (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG15_ED_EN_VALUE_SHIFT                   (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG15_ED_EN_VALUE_MAX                     (0x00000001U)

/* UTMI_REG16 */

#define CSL_USB3P0SS_PHY2_UTMI_REG16_UNUSED_MASK                         (0x000000FFU)
#define CSL_USB3P0SS_PHY2_UTMI_REG16_UNUSED_SHIFT                        (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG16_UNUSED_MAX                          (0x000000FFU)

/* UTMI_REG17 */

#define CSL_USB3P0SS_PHY2_UTMI_REG17_SQUELCH_COUNT_IDLE_EN_MASK          (0x00000020U)
#define CSL_USB3P0SS_PHY2_UTMI_REG17_SQUELCH_COUNT_IDLE_EN_SHIFT         (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG17_SQUELCH_COUNT_IDLE_EN_MAX           (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG17_TX_SQ_CNT_EN_MASK                   (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG17_TX_SQ_CNT_EN_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG17_TX_SQ_CNT_EN_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG17_TX_SQ_CNT_MASK                      (0x0000001EU)
#define CSL_USB3P0SS_PHY2_UTMI_REG17_TX_SQ_CNT_SHIFT                     (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG17_TX_SQ_CNT_MAX                       (0x0000000FU)

#define CSL_USB3P0SS_PHY2_UTMI_REG17_SQUELCH_COUNT_IDLE_MASK             (0x000000C0U)
#define CSL_USB3P0SS_PHY2_UTMI_REG17_SQUELCH_COUNT_IDLE_SHIFT            (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG17_SQUELCH_COUNT_IDLE_MAX              (0x00000003U)

/* UTMI_REG18 */

#define CSL_USB3P0SS_PHY2_UTMI_REG18_CLIPPER_EN_MASK                     (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG18_CLIPPER_EN_SHIFT                    (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG18_CLIPPER_EN_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG18_SLEEP_EN_MASK                       (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG18_SLEEP_EN_SHIFT                      (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG18_SLEEP_EN_MAX                        (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG18_BIST_POWERUP_MASK                   (0x00000020U)
#define CSL_USB3P0SS_PHY2_UTMI_REG18_BIST_POWERUP_SHIFT                  (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG18_BIST_POWERUP_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG18_SLEEP_VALUE_MASK                    (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG18_SLEEP_VALUE_SHIFT                   (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG18_SLEEP_VALUE_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG18_CLIPPER_EN_EN_MASK                  (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG18_CLIPPER_EN_EN_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG18_CLIPPER_EN_EN_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG18_BIST_POWERUP_EN_MASK                (0x00000010U)
#define CSL_USB3P0SS_PHY2_UTMI_REG18_BIST_POWERUP_EN_SHIFT               (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG18_BIST_POWERUP_EN_MAX                 (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG18_POWERUP_EN_MASK                     (0x00000008U)
#define CSL_USB3P0SS_PHY2_UTMI_REG18_POWERUP_EN_SHIFT                    (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG18_POWERUP_EN_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG18_UNUSED_MASK                         (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG18_UNUSED_SHIFT                        (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG18_UNUSED_MAX                          (0x00000001U)

/* UTMI_REG19 */

#define CSL_USB3P0SS_PHY2_UTMI_REG19_TED_SW_EN_MASK                      (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG19_TED_SW_EN_SHIFT                     (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG19_TED_SW_EN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG19_UNUSED_MASK                         (0x000000FEU)
#define CSL_USB3P0SS_PHY2_UTMI_REG19_UNUSED_SHIFT                        (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG19_UNUSED_MAX                          (0x0000007FU)

/* UTMI_REG20 */

#define CSL_USB3P0SS_PHY2_UTMI_REG20_CALIB_RST_DT_EN_MASK                (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG20_CALIB_RST_DT_EN_SHIFT               (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG20_CALIB_RST_DT_EN_MAX                 (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG20_CALIB_RST_DT_MASK                   (0x0000003EU)
#define CSL_USB3P0SS_PHY2_UTMI_REG20_CALIB_RST_DT_SHIFT                  (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG20_CALIB_RST_DT_MAX                    (0x0000001FU)

#define CSL_USB3P0SS_PHY2_UTMI_REG20_HOSTDISCON_RST_REG_EN_MASK          (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG20_HOSTDISCON_RST_REG_EN_SHIFT         (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG20_HOSTDISCON_RST_REG_EN_MAX           (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG20_HOSTDISCON_RST_REG_MASK             (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG20_HOSTDISCON_RST_REG_SHIFT            (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG20_HOSTDISCON_RST_REG_MAX              (0x00000001U)

/* UTMI_REG21 */

#define CSL_USB3P0SS_PHY2_UTMI_REG21_CALIB_TRIGER_POSEDGE_MASK           (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG21_CALIB_TRIGER_POSEDGE_SHIFT          (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG21_CALIB_TRIGER_POSEDGE_MAX            (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG21_VBUSVALID_EN_MASK                   (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG21_VBUSVALID_EN_SHIFT                  (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG21_VBUSVALID_EN_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG21_AUTO_CAL_ENABLE_MASK                (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG21_AUTO_CAL_ENABLE_SHIFT               (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG21_AUTO_CAL_ENABLE_MAX                 (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG21_VBUSVALID_MASK                      (0x00000008U)
#define CSL_USB3P0SS_PHY2_UTMI_REG21_VBUSVALID_SHIFT                     (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG21_VBUSVALID_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG21_SUSPENDM_EN_MASK                    (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG21_SUSPENDM_EN_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG21_SUSPENDM_EN_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG21_SUSPENDM_MASK                       (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG21_SUSPENDM_SHIFT                      (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG21_SUSPENDM_MAX                        (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG21_ABSVALID_EN_MASK                    (0x00000010U)
#define CSL_USB3P0SS_PHY2_UTMI_REG21_ABSVALID_EN_SHIFT                   (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG21_ABSVALID_EN_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG21_ABSVALID_MASK                       (0x00000020U)
#define CSL_USB3P0SS_PHY2_UTMI_REG21_ABSVALID_SHIFT                      (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG21_ABSVALID_MAX                        (0x00000001U)

/* UTMI_REG22 */

#define CSL_USB3P0SS_PHY2_UTMI_REG22_BCCALIB_OFFSET_MASK                 (0x000000FFU)
#define CSL_USB3P0SS_PHY2_UTMI_REG22_BCCALIB_OFFSET_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG22_BCCALIB_OFFSET_MAX                  (0x000000FFU)

/* UTMI_REG23 */

#define CSL_USB3P0SS_PHY2_UTMI_REG23_HSCALIB_OFFSET_MASK                 (0x000000FFU)
#define CSL_USB3P0SS_PHY2_UTMI_REG23_HSCALIB_OFFSET_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG23_HSCALIB_OFFSET_MAX                  (0x000000FFU)

/* UTMI_REG24 */

#define CSL_USB3P0SS_PHY2_UTMI_REG24_FSCALIB_OFFSET_MASK                 (0x000000FFU)
#define CSL_USB3P0SS_PHY2_UTMI_REG24_FSCALIB_OFFSET_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG24_FSCALIB_OFFSET_MAX                  (0x000000FFU)

/* UTMI_REG25 */

#define CSL_USB3P0SS_PHY2_UTMI_REG25_UNUSED_MASK                         (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG25_UNUSED_SHIFT                        (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG25_UNUSED_MAX                          (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG25_HSCALIB_MASK                        (0x0000007FU)
#define CSL_USB3P0SS_PHY2_UTMI_REG25_HSCALIB_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG25_HSCALIB_MAX                         (0x0000007FU)

/* UTMI_REG26 */

#define CSL_USB3P0SS_PHY2_UTMI_REG26_FSCALIB_MASK                        (0x0000007FU)
#define CSL_USB3P0SS_PHY2_UTMI_REG26_FSCALIB_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG26_FSCALIB_MAX                         (0x0000007FU)

#define CSL_USB3P0SS_PHY2_UTMI_REG26_UNUSED_MASK                         (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG26_UNUSED_SHIFT                        (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG26_UNUSED_MAX                          (0x00000001U)

/* UTMI_REG27 */

#define CSL_USB3P0SS_PHY2_UTMI_REG27_BCCALIB_MASK                        (0x0000007FU)
#define CSL_USB3P0SS_PHY2_UTMI_REG27_BCCALIB_SHIFT                       (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG27_BCCALIB_MAX                         (0x0000007FU)

#define CSL_USB3P0SS_PHY2_UTMI_REG27_UNUSED_MASK                         (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG27_UNUSED_SHIFT                        (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG27_UNUSED_MAX                          (0x00000001U)

/* UTMI_REG28 */

#define CSL_USB3P0SS_PHY2_UTMI_REG28_CDR_EB_WR_RESET_MASK                (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG28_CDR_EB_WR_RESET_SHIFT               (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG28_CDR_EB_WR_RESET_MAX                 (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG28_SERX_EN_CNTRL_OPMODE01_MASK         (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG28_SERX_EN_CNTRL_OPMODE01_SHIFT        (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG28_SERX_EN_CNTRL_OPMODE01_MAX          (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG28_UNUSED_MASK                         (0x0000007EU)
#define CSL_USB3P0SS_PHY2_UTMI_REG28_UNUSED_SHIFT                        (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG28_UNUSED_MAX                          (0x0000003FU)

/* UTMI_REG29 */

#define CSL_USB3P0SS_PHY2_UTMI_REG29_PLL_STANDALONE_EN_MASK              (0x00000010U)
#define CSL_USB3P0SS_PHY2_UTMI_REG29_PLL_STANDALONE_EN_SHIFT             (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG29_PLL_STANDALONE_EN_MAX               (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG29_PLL_STANDALONE_MASK                 (0x00000020U)
#define CSL_USB3P0SS_PHY2_UTMI_REG29_PLL_STANDALONE_SHIFT                (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG29_PLL_STANDALONE_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG29_SPARE_OUT_MASK                      (0x0000000FU)
#define CSL_USB3P0SS_PHY2_UTMI_REG29_SPARE_OUT_SHIFT                     (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG29_SPARE_OUT_MAX                       (0x0000000FU)

#define CSL_USB3P0SS_PHY2_UTMI_REG29_UNUSED_MASK                         (0x000000C0U)
#define CSL_USB3P0SS_PHY2_UTMI_REG29_UNUSED_SHIFT                        (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG29_UNUSED_MAX                          (0x00000003U)

/* UTMI_REG30 */

#define CSL_USB3P0SS_PHY2_UTMI_REG30_DIG_DIV_REFCLOCK_GATE_OVR_MASK      (0x00000008U)
#define CSL_USB3P0SS_PHY2_UTMI_REG30_DIG_DIV_REFCLOCK_GATE_OVR_SHIFT     (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG30_DIG_DIV_REFCLOCK_GATE_OVR_MAX       (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG30_FB_CLOCK_GATE_OVR_MASK              (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG30_FB_CLOCK_GATE_OVR_SHIFT             (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG30_FB_CLOCK_GATE_OVR_MAX               (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG30_PLL_480_CLOCK_GATE_OVR_MASK         (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG30_PLL_480_CLOCK_GATE_OVR_SHIFT        (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG30_PLL_480_CLOCK_GATE_OVR_MAX          (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG30_VCO_PLL_CLOCK_GATE_OVR_MASK         (0x00000010U)
#define CSL_USB3P0SS_PHY2_UTMI_REG30_VCO_PLL_CLOCK_GATE_OVR_SHIFT        (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG30_VCO_PLL_CLOCK_GATE_OVR_MAX          (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG30_SCAN_ATS_HS_CLOCK_GATE_OVR_MASK     (0x00000020U)
#define CSL_USB3P0SS_PHY2_UTMI_REG30_SCAN_ATS_HS_CLOCK_GATE_OVR_SHIFT    (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG30_SCAN_ATS_HS_CLOCK_GATE_OVR_MAX      (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG30_HS_CLOCK_GATE_OVR_MASK              (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG30_HS_CLOCK_GATE_OVR_SHIFT             (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG30_HS_CLOCK_GATE_OVR_MAX               (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG30_UNUSED_MASK                         (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG30_UNUSED_SHIFT                        (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG30_UNUSED_MAX                          (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG30_ANA_DIV_REFCLOCK_GATE_OVR_MASK      (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG30_ANA_DIV_REFCLOCK_GATE_OVR_SHIFT     (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG30_ANA_DIV_REFCLOCK_GATE_OVR_MAX       (0x00000001U)

/* UTMI_UNUSED_REG0 */

#define CSL_USB3P0SS_PHY2_UTMI_UNUSED_REG0_UNUSED_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_UTMI_UNUSED_REG0_UNUSED_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_UNUSED_REG0_UNUSED_MAX                    (0x000000FFU)

/* UTMI_UNUSED_REG1 */

#define CSL_USB3P0SS_PHY2_UTMI_UNUSED_REG1_UNUSED_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_UTMI_UNUSED_REG1_UNUSED_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_UNUSED_REG1_UNUSED_MAX                    (0x000000FFU)

/* UTMI_UNUSED_REG2 */

#define CSL_USB3P0SS_PHY2_UTMI_UNUSED_REG2_UNUSED_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_UTMI_UNUSED_REG2_UNUSED_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_UNUSED_REG2_UNUSED_MAX                    (0x000000FFU)

/* UTMI_UNUSED_REG3 */

#define CSL_USB3P0SS_PHY2_UTMI_UNUSED_REG3_UNUSED_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_UTMI_UNUSED_REG3_UNUSED_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_UNUSED_REG3_UNUSED_MAX                    (0x000000FFU)

/* UTMI_REG31 */

#define CSL_USB3P0SS_PHY2_UTMI_REG31_BIST_COMPLETE_MASK                  (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG31_BIST_COMPLETE_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG31_BIST_COMPLETE_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG31_UNUSED_MASK                         (0x000000FCU)
#define CSL_USB3P0SS_PHY2_UTMI_REG31_UNUSED_SHIFT                        (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG31_UNUSED_MAX                          (0x0000003FU)

#define CSL_USB3P0SS_PHY2_UTMI_REG31_BIST_ERROR_MASK                     (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG31_BIST_ERROR_SHIFT                    (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG31_BIST_ERROR_MAX                      (0x00000001U)

/* UTMI_REG32 */

#define CSL_USB3P0SS_PHY2_UTMI_REG32_BIST_ERR_COUNT_MASK                 (0x000000FFU)
#define CSL_USB3P0SS_PHY2_UTMI_REG32_BIST_ERR_COUNT_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG32_BIST_ERR_COUNT_MAX                  (0x000000FFU)

/* UTMI_REG33 */

#define CSL_USB3P0SS_PHY2_UTMI_REG33_CHIRP_MODE_EN_MASK                  (0x00000008U)
#define CSL_USB3P0SS_PHY2_UTMI_REG33_CHIRP_MODE_EN_SHIFT                 (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG33_CHIRP_MODE_EN_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG33_HSRX_EN_MASK                        (0x00000020U)
#define CSL_USB3P0SS_PHY2_UTMI_REG33_HSRX_EN_SHIFT                       (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG33_HSRX_EN_MAX                         (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG33_HSTX_EN_DELAYED_MASK                (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG33_HSTX_EN_DELAYED_SHIFT               (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG33_HSTX_EN_DELAYED_MAX                 (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG33_AFE_HSRX_DIFF_DATA_MASK             (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG33_AFE_HSRX_DIFF_DATA_SHIFT            (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG33_AFE_HSRX_DIFF_DATA_MAX              (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG33_HSTX_BOOST_DEAMP_OFF_MASK           (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG33_HSTX_BOOST_DEAMP_OFF_SHIFT          (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG33_HSTX_BOOST_DEAMP_OFF_MAX            (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG33_HSTX_EN_MASK                        (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG33_HSTX_EN_SHIFT                       (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG33_HSTX_EN_MAX                         (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG33_BG_POWERGOOD_MASK                   (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG33_BG_POWERGOOD_SHIFT                  (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG33_BG_POWERGOOD_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG33_HSRX_SAMPLER_ENABLE_MASK            (0x00000010U)
#define CSL_USB3P0SS_PHY2_UTMI_REG33_HSRX_SAMPLER_ENABLE_SHIFT           (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG33_HSRX_SAMPLER_ENABLE_MAX             (0x00000001U)

/* UTMI_REG34 */

#define CSL_USB3P0SS_PHY2_UTMI_REG34_O_DPRPU2_EN_MASK                    (0x00000020U)
#define CSL_USB3P0SS_PHY2_UTMI_REG34_O_DPRPU2_EN_SHIFT                   (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG34_O_DPRPU2_EN_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG34_O_DMRPD_EN_MASK                     (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG34_O_DMRPD_EN_SHIFT                    (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG34_O_DMRPD_EN_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG34_O_OTGC_ID_PULLUP_EN_MASK            (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG34_O_OTGC_ID_PULLUP_EN_SHIFT           (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG34_O_OTGC_ID_PULLUP_EN_MAX             (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG34_O_DPRPD_EN_MASK                     (0x00000008U)
#define CSL_USB3P0SS_PHY2_UTMI_REG34_O_DPRPD_EN_SHIFT                    (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG34_O_DPRPD_EN_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG34_O_DMRPU2_EN_MASK                    (0x00000010U)
#define CSL_USB3P0SS_PHY2_UTMI_REG34_O_DMRPU2_EN_SHIFT                   (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG34_O_DMRPU2_EN_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG34_O_DMRPU1_EN_MASK                    (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG34_O_DMRPU1_EN_SHIFT                   (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG34_O_DMRPU1_EN_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG34_O_DPRPU1_EN_MASK                    (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG34_O_DPRPU1_EN_SHIFT                   (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG34_O_DPRPU1_EN_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG34_O_FS_EDGE_SEL_MASK                  (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG34_O_FS_EDGE_SEL_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG34_O_FS_EDGE_SEL_MAX                   (0x00000001U)

/* UTMI_REG35 */

#define CSL_USB3P0SS_PHY2_UTMI_REG35_O_LSFS_DDI_MASK                     (0x00000010U)
#define CSL_USB3P0SS_PHY2_UTMI_REG35_O_LSFS_DDI_SHIFT                    (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG35_O_LSFS_DDI_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG35_O_LSFSDRV_EN_MASK                   (0x00000020U)
#define CSL_USB3P0SS_PHY2_UTMI_REG35_O_LSFSDRV_EN_SHIFT                  (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG35_O_LSFSDRV_EN_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG35_O_SERX_BIAS_EN_MASK                 (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG35_O_SERX_BIAS_EN_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG35_O_SERX_BIAS_EN_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG35_O_LSFSRX_EN_MASK                    (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG35_O_LSFSRX_EN_SHIFT                   (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG35_O_LSFSRX_EN_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG35_O_LSFSTX_EN_MASK                    (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG35_O_LSFSTX_EN_SHIFT                   (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG35_O_LSFSTX_EN_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG35_O_SERX_EN_MASK                      (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG35_O_SERX_EN_SHIFT                     (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG35_O_SERX_EN_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG35_O_ASSERT_SEZERO_MASK                (0x00000008U)
#define CSL_USB3P0SS_PHY2_UTMI_REG35_O_ASSERT_SEZERO_SHIFT               (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG35_O_ASSERT_SEZERO_MAX                 (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG35_I_AFE_LSFSRX_ANA_MASK               (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG35_I_AFE_LSFSRX_ANA_SHIFT              (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG35_I_AFE_LSFSRX_ANA_MAX                (0x00000001U)

/* UTMI_REG36 */

#define CSL_USB3P0SS_PHY2_UTMI_REG36_O_PLL_PSO_DELAY_MASK                (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG36_O_PLL_PSO_DELAY_SHIFT               (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG36_O_PLL_PSO_DELAY_MAX                 (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG36_O_PLL_PSO_MASK                      (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG36_O_PLL_PSO_SHIFT                     (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG36_O_PLL_PSO_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG36_O_PLL_IPDIV_MASK                    (0x0000001FU)
#define CSL_USB3P0SS_PHY2_UTMI_REG36_O_PLL_IPDIV_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG36_O_PLL_IPDIV_MAX                     (0x0000001FU)

#define CSL_USB3P0SS_PHY2_UTMI_REG36_O_PLL_PD_MASK                       (0x00000020U)
#define CSL_USB3P0SS_PHY2_UTMI_REG36_O_PLL_PD_SHIFT                      (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG36_O_PLL_PD_MAX                        (0x00000001U)

/* UTMI_REG37 */

#define CSL_USB3P0SS_PHY2_UTMI_REG37_O_PLL_FBDIV_VALUE_MASK              (0x000000FFU)
#define CSL_USB3P0SS_PHY2_UTMI_REG37_O_PLL_FBDIV_VALUE_SHIFT             (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG37_O_PLL_FBDIV_VALUE_MAX               (0x000000FFU)

/* UTMI_REG38 */

#define CSL_USB3P0SS_PHY2_UTMI_REG38_O_OTGC_VBUSVALID_EN_MASK            (0x00000008U)
#define CSL_USB3P0SS_PHY2_UTMI_REG38_O_OTGC_VBUSVALID_EN_SHIFT           (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG38_O_OTGC_VBUSVALID_EN_MAX             (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG38_O_PLL_STANDBY_MASK                  (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG38_O_PLL_STANDBY_SHIFT                 (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG38_O_PLL_STANDBY_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG38_O_AFE_SUSPENDM_MASK                 (0x00000010U)
#define CSL_USB3P0SS_PHY2_UTMI_REG38_O_AFE_SUSPENDM_SHIFT                (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG38_O_AFE_SUSPENDM_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG38_O_PLL_LDO_REF_EN_MASK               (0x00000020U)
#define CSL_USB3P0SS_PHY2_UTMI_REG38_O_PLL_LDO_REF_EN_SHIFT              (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG38_O_PLL_LDO_REF_EN_MAX                (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG38_O_AFE_CLIPPER_EN_MASK               (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG38_O_AFE_CLIPPER_EN_SHIFT              (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG38_O_AFE_CLIPPER_EN_MAX                (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG38_O_PLL_LDO_ISOLATION_CNTRL_MASK      (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG38_O_PLL_LDO_ISOLATION_CNTRL_SHIFT     (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG38_O_PLL_LDO_ISOLATION_CNTRL_MAX       (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG38_O_PLL_LDO_CORE_EN_MASK              (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG38_O_PLL_LDO_CORE_EN_SHIFT             (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG38_O_PLL_LDO_CORE_EN_MAX               (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG38_O_OTGC_ABSVALID_EN_MASK             (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG38_O_OTGC_ABSVALID_EN_SHIFT            (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG38_O_OTGC_ABSVALID_EN_MAX              (0x00000001U)

/* UTMI_REG39 */

#define CSL_USB3P0SS_PHY2_UTMI_REG39_UNUSED_MASK                         (0x000000FFU)
#define CSL_USB3P0SS_PHY2_UTMI_REG39_UNUSED_SHIFT                        (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG39_UNUSED_MAX                          (0x000000FFU)

/* UTMI_REG40 */

#define CSL_USB3P0SS_PHY2_UTMI_REG40_UNUSED_MASK                         (0x000000FFU)
#define CSL_USB3P0SS_PHY2_UTMI_REG40_UNUSED_SHIFT                        (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG40_UNUSED_MAX                          (0x000000FFU)

/* UTMI_REG41 */

#define CSL_USB3P0SS_PHY2_UTMI_REG41_I_TED_SQUELCH_ANA_MASK              (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG41_I_TED_SQUELCH_ANA_SHIFT             (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG41_I_TED_SQUELCH_ANA_MAX               (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG41_HS_CALIB_CODE_MASK                  (0x0000003FU)
#define CSL_USB3P0SS_PHY2_UTMI_REG41_HS_CALIB_CODE_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG41_HS_CALIB_CODE_MAX                   (0x0000003FU)

#define CSL_USB3P0SS_PHY2_UTMI_REG41_I_USB2_RESCAL_CALIB_DONE_MASK       (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG41_I_USB2_RESCAL_CALIB_DONE_SHIFT      (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG41_I_USB2_RESCAL_CALIB_DONE_MAX        (0x00000001U)

/* UTMI_REG42 */

#define CSL_USB3P0SS_PHY2_UTMI_REG42_HS_SOF_MASK                         (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG42_HS_SOF_SHIFT                        (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG42_HS_SOF_MAX                          (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG42_ALL_CALIB_DONE_MASK                 (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG42_ALL_CALIB_DONE_SHIFT                (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG42_ALL_CALIB_DONE_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG42_FS_CALIB_CODE_MASK                  (0x0000003FU)
#define CSL_USB3P0SS_PHY2_UTMI_REG42_FS_CALIB_CODE_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG42_FS_CALIB_CODE_MAX                   (0x0000003FU)

/* UTMI_REG43 */

#define CSL_USB3P0SS_PHY2_UTMI_REG43_BC_CALIB_CODE_MASK                  (0x0000003FU)
#define CSL_USB3P0SS_PHY2_UTMI_REG43_BC_CALIB_CODE_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG43_BC_CALIB_CODE_MAX                   (0x0000003FU)

#define CSL_USB3P0SS_PHY2_UTMI_REG43_FS_MODE_PRE_MASK                    (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG43_FS_MODE_PRE_SHIFT                   (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG43_FS_MODE_PRE_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG43_LS_MODE_MASK                        (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG43_LS_MODE_SHIFT                       (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG43_LS_MODE_MAX                         (0x00000001U)

/* UTMI_REG44 */

#define CSL_USB3P0SS_PHY2_UTMI_REG44_RSTN_REFCLOCK_MASK                  (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG44_RSTN_REFCLOCK_SHIFT                 (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG44_RSTN_REFCLOCK_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG44_RSTN_CLKDIV_MASK                    (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG44_RSTN_CLKDIV_SHIFT                   (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG44_RSTN_CLKDIV_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG44_RSTN_BYTE_CLOCK_MASK                (0x00000010U)
#define CSL_USB3P0SS_PHY2_UTMI_REG44_RSTN_BYTE_CLOCK_SHIFT               (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG44_RSTN_BYTE_CLOCK_MAX                 (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG44_RSTN_HS_CLOCK_MASK                  (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG44_RSTN_HS_CLOCK_SHIFT                 (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG44_RSTN_HS_CLOCK_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG44_RSTN_CALIB_CLKDIV_MASK              (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG44_RSTN_CALIB_CLKDIV_SHIFT             (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG44_RSTN_CALIB_CLKDIV_MAX               (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG44_UDC_RSTN_CDR_ASYNC_MASK             (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG44_UDC_RSTN_CDR_ASYNC_SHIFT            (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG44_UDC_RSTN_CDR_ASYNC_MAX              (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG44_RSTN_SIECLOCK_MASK                  (0x00000008U)
#define CSL_USB3P0SS_PHY2_UTMI_REG44_RSTN_SIECLOCK_SHIFT                 (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG44_RSTN_SIECLOCK_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG44_RSTN_HS_TX_CLOCK_MASK               (0x00000020U)
#define CSL_USB3P0SS_PHY2_UTMI_REG44_RSTN_HS_TX_CLOCK_SHIFT              (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG44_RSTN_HS_TX_CLOCK_MAX                (0x00000001U)

/* UTMI_REG45 */

#define CSL_USB3P0SS_PHY2_UTMI_REG45_BIST_MODE_RSTN_MASK                 (0x00000008U)
#define CSL_USB3P0SS_PHY2_UTMI_REG45_BIST_MODE_RSTN_SHIFT                (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG45_BIST_MODE_RSTN_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG45_UDC_APB_RSTN_MASK                   (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG45_UDC_APB_RSTN_SHIFT                  (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG45_UDC_APB_RSTN_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG45_GLOBAL_RESETN_MASK                  (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG45_GLOBAL_RESETN_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG45_GLOBAL_RESETN_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG45_UDC_CALIB_RSTN_MASK                 (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG45_UDC_CALIB_RSTN_SHIFT                (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG45_UDC_CALIB_RSTN_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG45_UDC_BC_CALIB_RSTN_MASK              (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG45_UDC_BC_CALIB_RSTN_SHIFT             (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG45_UDC_BC_CALIB_RSTN_MAX               (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG45_O_RSTN_CDR_ASYNC_MASK               (0x00000020U)
#define CSL_USB3P0SS_PHY2_UTMI_REG45_O_RSTN_CDR_ASYNC_SHIFT              (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG45_O_RSTN_CDR_ASYNC_MAX                (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG45_O_USB2_CALIB_RSTN_MASK              (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG45_O_USB2_CALIB_RSTN_SHIFT             (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG45_O_USB2_CALIB_RSTN_MAX               (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG45_O_PLL_CALIB_RSTN_MASK               (0x00000010U)
#define CSL_USB3P0SS_PHY2_UTMI_REG45_O_PLL_CALIB_RSTN_SHIFT              (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG45_O_PLL_CALIB_RSTN_MAX                (0x00000001U)

/* UTMI_REG46 */

#define CSL_USB3P0SS_PHY2_UTMI_REG46_RECOVERY_CNT_EN_MASK                (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG46_RECOVERY_CNT_EN_SHIFT               (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG46_RECOVERY_CNT_EN_MAX                 (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG46_CLEAN_LINESTATE_MASK                (0x00000030U)
#define CSL_USB3P0SS_PHY2_UTMI_REG46_CLEAN_LINESTATE_SHIFT               (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG46_CLEAN_LINESTATE_MAX                 (0x00000003U)

#define CSL_USB3P0SS_PHY2_UTMI_REG46_BC_STATE_MACHINE_STATUS_MASK        (0x0000000FU)
#define CSL_USB3P0SS_PHY2_UTMI_REG46_BC_STATE_MACHINE_STATUS_SHIFT       (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG46_BC_STATE_MACHINE_STATUS_MAX         (0x0000000FU)

#define CSL_USB3P0SS_PHY2_UTMI_REG46_UNUSED_MASK                         (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG46_UNUSED_SHIFT                        (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG46_UNUSED_MAX                          (0x00000001U)

/* UTMI_REG47 */

#define CSL_USB3P0SS_PHY2_UTMI_REG47_DEV_OPMODE_MASK                     (0x00000018U)
#define CSL_USB3P0SS_PHY2_UTMI_REG47_DEV_OPMODE_SHIFT                    (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG47_DEV_OPMODE_MAX                      (0x00000003U)

#define CSL_USB3P0SS_PHY2_UTMI_REG47_LSFS_HOSTDISCONNECT_MASK            (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG47_LSFS_HOSTDISCONNECT_SHIFT           (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG47_LSFS_HOSTDISCONNECT_MAX             (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG47_HS_HOSTDISCONNECT_MASK              (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG47_HS_HOSTDISCONNECT_SHIFT             (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG47_HS_HOSTDISCONNECT_MAX               (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG47_FILTER_CNT_EN_MASK                  (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG47_FILTER_CNT_EN_SHIFT                 (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG47_FILTER_CNT_EN_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG47_I_DED_ANA_MASK                      (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG47_I_DED_ANA_SHIFT                     (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG47_I_DED_ANA_MAX                       (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG47_HOST_OPMODE_MASK                    (0x00000060U)
#define CSL_USB3P0SS_PHY2_UTMI_REG47_HOST_OPMODE_SHIFT                   (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG47_HOST_OPMODE_MAX                     (0x00000003U)

/* UTMI_REG48 */

#define CSL_USB3P0SS_PHY2_UTMI_REG48_BIST_TX_STATE_MASK                  (0x000000C0U)
#define CSL_USB3P0SS_PHY2_UTMI_REG48_BIST_TX_STATE_SHIFT                 (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG48_BIST_TX_STATE_MAX                   (0x00000003U)

#define CSL_USB3P0SS_PHY2_UTMI_REG48_DATA_CNT_TX_MASK                    (0x0000003FU)
#define CSL_USB3P0SS_PHY2_UTMI_REG48_DATA_CNT_TX_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG48_DATA_CNT_TX_MAX                     (0x0000003FU)

/* UTMI_REG49 */

#define CSL_USB3P0SS_PHY2_UTMI_REG49_DATA_CNT_RX_MASK                    (0x0000003FU)
#define CSL_USB3P0SS_PHY2_UTMI_REG49_DATA_CNT_RX_SHIFT                   (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG49_DATA_CNT_RX_MAX                     (0x0000003FU)

#define CSL_USB3P0SS_PHY2_UTMI_REG49_BIST_RX_STATE_MASK                  (0x000000C0U)
#define CSL_USB3P0SS_PHY2_UTMI_REG49_BIST_RX_STATE_SHIFT                 (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG49_BIST_RX_STATE_MAX                   (0x00000003U)

/* UTMI_REG50 */

#define CSL_USB3P0SS_PHY2_UTMI_REG50_O_BG_PD_BG_OK_MASK                  (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG50_O_BG_PD_BG_OK_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG50_O_BG_PD_BG_OK_MAX                   (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG50_INC_DATA_CNT_TX_MASK                (0x00000008U)
#define CSL_USB3P0SS_PHY2_UTMI_REG50_INC_DATA_CNT_TX_SHIFT               (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG50_INC_DATA_CNT_TX_MAX                 (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG50_O_BG_PD_MASK                        (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG50_O_BG_PD_SHIFT                       (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG50_O_BG_PD_MAX                         (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG50_INC_DATA_CNT_RX_MASK                (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG50_INC_DATA_CNT_RX_SHIFT               (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG50_INC_DATA_CNT_RX_MAX                 (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG50_BIST_TOP_STATE_MASK                 (0x000000F0U)
#define CSL_USB3P0SS_PHY2_UTMI_REG50_BIST_TOP_STATE_SHIFT                (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG50_BIST_TOP_STATE_MAX                  (0x0000000FU)

/* UTMI_REG51 */

#define CSL_USB3P0SS_PHY2_UTMI_REG51_DPPULLDOWN_MASK                     (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG51_DPPULLDOWN_SHIFT                    (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG51_DPPULLDOWN_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG51_TERMSELECT_MASK                     (0x00000008U)
#define CSL_USB3P0SS_PHY2_UTMI_REG51_TERMSELECT_SHIFT                    (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG51_TERMSELECT_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG51_RESET_MASK                          (0x00000020U)
#define CSL_USB3P0SS_PHY2_UTMI_REG51_RESET_SHIFT                         (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG51_RESET_MAX                           (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG51_POWERDOWN_MASK                      (0x000000C0U)
#define CSL_USB3P0SS_PHY2_UTMI_REG51_POWERDOWN_SHIFT                     (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG51_POWERDOWN_MAX                       (0x00000003U)

#define CSL_USB3P0SS_PHY2_UTMI_REG51_DATABUS16_8_MASK                    (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG51_DATABUS16_8_SHIFT                   (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG51_DATABUS16_8_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG51_DMPULLDOWN_MASK                     (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG51_DMPULLDOWN_SHIFT                    (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG51_DMPULLDOWN_MAX                      (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG51_SUSPENDM_MASK                       (0x00000010U)
#define CSL_USB3P0SS_PHY2_UTMI_REG51_SUSPENDM_SHIFT                      (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG51_SUSPENDM_MAX                        (0x00000001U)

/* UTMI_REG52 */

#define CSL_USB3P0SS_PHY2_UTMI_REG52_TXBITSTUFFENABLE_MASK               (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG52_TXBITSTUFFENABLE_SHIFT              (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG52_TXBITSTUFFENABLE_MAX                (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG52_LINESTATE_MASK                      (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG52_LINESTATE_SHIFT                     (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG52_LINESTATE_MAX                       (0x00000003U)

#define CSL_USB3P0SS_PHY2_UTMI_REG52_HOSTDISCONNECT_MASK                 (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG52_HOSTDISCONNECT_SHIFT                (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG52_HOSTDISCONNECT_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG52_LANE_REVERSE_MASK                   (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG52_LANE_REVERSE_SHIFT                  (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG52_LANE_REVERSE_MAX                    (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG52_TXBITSTUFFENABLEH_MASK              (0x00000020U)
#define CSL_USB3P0SS_PHY2_UTMI_REG52_TXBITSTUFFENABLEH_SHIFT             (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG52_TXBITSTUFFENABLEH_MAX               (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG52_XCVRSELECT_MASK                     (0x00000018U)
#define CSL_USB3P0SS_PHY2_UTMI_REG52_XCVRSELECT_SHIFT                    (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG52_XCVRSELECT_MAX                      (0x00000003U)

/* UTMI_REG53 */

#define CSL_USB3P0SS_PHY2_UTMI_REG53_FSLSSERIALMODE_MASK                 (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG53_FSLSSERIALMODE_SHIFT                (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG53_FSLSSERIALMODE_MAX                  (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG53_TX_SE0_MASK                         (0x00000010U)
#define CSL_USB3P0SS_PHY2_UTMI_REG53_TX_SE0_SHIFT                        (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG53_TX_SE0_MAX                          (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG53_TX_ENABLE_N_MASK                    (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG53_TX_ENABLE_N_SHIFT                   (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG53_TX_ENABLE_N_MAX                     (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG53_TX_DAT_MASK                         (0x00000020U)
#define CSL_USB3P0SS_PHY2_UTMI_REG53_TX_DAT_SHIFT                        (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG53_TX_DAT_MAX                          (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG53_UNUSED_MASK                         (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG53_UNUSED_SHIFT                        (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG53_UNUSED_MAX                          (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG53_SLEEPM_MASK                         (0x00000008U)
#define CSL_USB3P0SS_PHY2_UTMI_REG53_SLEEPM_SHIFT                        (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG53_SLEEPM_MAX                          (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG53_OPMODE_MASK                         (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG53_OPMODE_SHIFT                        (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG53_OPMODE_MAX                          (0x00000003U)

/* UTMI_REG54 */

#define CSL_USB3P0SS_PHY2_UTMI_REG54_RX_DP_MASK                          (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG54_RX_DP_SHIFT                         (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG54_RX_DP_MAX                           (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG54_RX_RCV_MASK                         (0x00000020U)
#define CSL_USB3P0SS_PHY2_UTMI_REG54_RX_RCV_SHIFT                        (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG54_RX_RCV_MAX                          (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG54_RX_DM_MASK                          (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG54_RX_DM_SHIFT                         (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG54_RX_DM_MAX                           (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG54_UNUSED_MASK                         (0x0000001FU)
#define CSL_USB3P0SS_PHY2_UTMI_REG54_UNUSED_SHIFT                        (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG54_UNUSED_MAX                          (0x0000001FU)

/* UTMI_REG55 */

#define CSL_USB3P0SS_PHY2_UTMI_REG55_RXVALID_MASK                        (0x00000008U)
#define CSL_USB3P0SS_PHY2_UTMI_REG55_RXVALID_SHIFT                       (0x00000003U)
#define CSL_USB3P0SS_PHY2_UTMI_REG55_RXVALID_MAX                         (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG55_RXACTIVE_MASK                       (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG55_RXACTIVE_SHIFT                      (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG55_RXACTIVE_MAX                        (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG55_TXREADY_MASK                        (0x00000020U)
#define CSL_USB3P0SS_PHY2_UTMI_REG55_TXREADY_SHIFT                       (0x00000005U)
#define CSL_USB3P0SS_PHY2_UTMI_REG55_TXREADY_MAX                         (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG55_TXVALIDH_MASK                       (0x00000080U)
#define CSL_USB3P0SS_PHY2_UTMI_REG55_TXVALIDH_SHIFT                      (0x00000007U)
#define CSL_USB3P0SS_PHY2_UTMI_REG55_TXVALIDH_MAX                        (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG55_UNUSED_MASK                         (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG55_UNUSED_SHIFT                        (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG55_UNUSED_MAX                          (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG55_RXVALIDH_MASK                       (0x00000010U)
#define CSL_USB3P0SS_PHY2_UTMI_REG55_RXVALIDH_SHIFT                      (0x00000004U)
#define CSL_USB3P0SS_PHY2_UTMI_REG55_RXVALIDH_MAX                        (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG55_TXVALID_MASK                        (0x00000040U)
#define CSL_USB3P0SS_PHY2_UTMI_REG55_TXVALID_SHIFT                       (0x00000006U)
#define CSL_USB3P0SS_PHY2_UTMI_REG55_TXVALID_MAX                         (0x00000001U)

#define CSL_USB3P0SS_PHY2_UTMI_REG55_RXERROR_MASK                        (0x00000002U)
#define CSL_USB3P0SS_PHY2_UTMI_REG55_RXERROR_SHIFT                       (0x00000001U)
#define CSL_USB3P0SS_PHY2_UTMI_REG55_RXERROR_MAX                         (0x00000001U)

/* UTMI_REG56 */

#define CSL_USB3P0SS_PHY2_UTMI_REG56_DATAIN_UPPER_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_UTMI_REG56_DATAIN_UPPER_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG56_DATAIN_UPPER_MAX                    (0x000000FFU)

/* UTMI_REG57 */

#define CSL_USB3P0SS_PHY2_UTMI_REG57_DATAIN_LOWER_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_UTMI_REG57_DATAIN_LOWER_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG57_DATAIN_LOWER_MAX                    (0x000000FFU)

/* UTMI_REG58 */

#define CSL_USB3P0SS_PHY2_UTMI_REG58_DATAOUT_UPPER_MASK                  (0x000000FFU)
#define CSL_USB3P0SS_PHY2_UTMI_REG58_DATAOUT_UPPER_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG58_DATAOUT_UPPER_MAX                   (0x000000FFU)

/* UTMI_REG59 */

#define CSL_USB3P0SS_PHY2_UTMI_REG59_DATAOUT_LOWER_MASK                  (0x000000FFU)
#define CSL_USB3P0SS_PHY2_UTMI_REG59_DATAOUT_LOWER_SHIFT                 (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_REG59_DATAOUT_LOWER_MAX                   (0x000000FFU)

/* UTMI_UNUSED_REG6 */

#define CSL_USB3P0SS_PHY2_UTMI_UNUSED_REG6_UNUSED_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_UTMI_UNUSED_REG6_UNUSED_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_UNUSED_REG6_UNUSED_MAX                    (0x000000FFU)

/* UTMI_UNUSED_REG7 */

#define CSL_USB3P0SS_PHY2_UTMI_UNUSED_REG7_UNUSED_MASK                   (0x000000FFU)
#define CSL_USB3P0SS_PHY2_UTMI_UNUSED_REG7_UNUSED_SHIFT                  (0x00000000U)
#define CSL_USB3P0SS_PHY2_UTMI_UNUSED_REG7_UNUSED_MAX                    (0x000000FFU)

#ifdef __cplusplus
}
#endif
#endif
