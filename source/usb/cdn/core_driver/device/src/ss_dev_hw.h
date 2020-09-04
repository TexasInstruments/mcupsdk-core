/******************************************************************************
*
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
*
******************************************************************************
* ss_dev_hw.h
* Hardware header file
******************************************************************************/

#ifndef SS_DEV_HW_H
#define SS_DEV_HW_H

#ifdef __cplusplus
extern "C"
{
#endif


typedef struct {
    uint32_t USBR_CONF;
    uint32_t USBR_STS;
    uint32_t USBR_CMD;
    uint32_t USBR_ITPN;
    uint32_t USBR_LPM;
    uint32_t USBR_IEN;
    uint32_t USBR_ISTS;
    uint32_t USBR_EP_SEL;
    uint32_t USBR_EP_TRADDR;
    uint32_t USBR_EP_CFG;
    uint32_t USBR_EP_CMD;
    uint32_t USBR_EP_STS;
    uint32_t USBR_EP_STS_SID;
    uint32_t USBR_EP_STS_EN;
    uint32_t USBR_DRBL;
    uint32_t USBR_EP_IEN;
    uint32_t USBR_EP_ISTS;
    uint32_t USBR_PWR;
    uint32_t USBR_CONF2;    /* Global Configuration Register 2 */
    uint32_t USBR_CAP1;     /* Capability Register 1 */
    uint8_t reserved[16];
    uint32_t USBR_CAP6;
    uint8_t reserved_1[12];
    uint32_t EP_DMA_EXT_ADDR;
    uint32_t USBR_BUF_ADDR;
    uint32_t USBR_BUF_DATA;
    uint32_t USBR_BUF_CTRL;
    uint32_t DTRANS;
    uint32_t TDL_FROM_TRB;
    uint32_t TDL_BEH;
    uint32_t EP_TDL;
    uint32_t TDL_BEH2;
    uint32_t DMA_ADV_TD;
    uint8_t reserved_3[104];
    uint32_t CFG_REG1;
    uint32_t USBR_DBG_LINK1;
    uint8_t reserved_4[504];
    uint32_t DMA_AXI_CTRL;
    uint32_t DMA_AXI_ID;
    uint32_t DMA_AXI_CAP;
    uint32_t DMA_AXI_CTRL0;
    uint32_t DMA_AXI_CTRL1;
    uint32_t DMA_AXI_CTRL2;

} __attribute__((packed)) ssReg_t;

/**
 * Macro returns endpoint number from given endpoint address
 * @param epAddr endpoint address
 * @return endpoint number
 */
static inline uint8_t USBD_EPNUM_FROM_EPADDR(uint8_t epAddr) {
    return (epAddr & 0x0FU);
}

#define EPDIR_IN_EPADDR_OFFSET 7U
#define EPDIR_IN_EPADDR_POS 1U

/**
 * Function returns endpoint direction from given endpoint address
 * @param epAddr endpoint address
 * @return endpoint direction
 */
static inline uint8_t USBD_EPDIR_FROM_EPADDR(uint8_t epAddr) {
    return ((epAddr >> EPDIR_IN_EPADDR_OFFSET) & EPDIR_IN_EPADDR_POS);
}

/* EP_CFG */

#define USBRV_EP_DISABLED      0U
#define USBRV_EP_ENABLED       1U

#define USBRV_EP_CONTROL       0U
#define USBRV_EP_ISOCHRONOUS   1U
#define USBRV_EP_BULK          2U
#define USBRV_EP_INTERRUPT     3U

/**
 * Clear enable endpoint bit
 * @param reg register in which bit is to be cleared
 */
static inline void SET_EP_CONF_DISABLE(uint32_t * reg) {
    (*reg) &= ~((uint32_t) 0x1U);
}

/**
 * set enable endpoint bit
 * @param reg register in which bit is to be set
 */
static inline void SET_EP_CONF_ENABLE(uint32_t * reg) {
    (*reg) |= (uint32_t) 0x1U;
}

/**
 * set endpoint type bits in register
 * @param reg register
 * @param eptype endpoint type
 */
static inline void SET_EP_CONF_EPTYPE(uint32_t * reg, uint8_t eptype) {
    (*reg) |= ((uint32_t) (eptype) & 0x03U) << 1;
}

/**
 * set endpoint endian bits in register
 * @param reg register
 * @param epEndian endpoint endianness
 */
static inline void SET_EP_CONF_EPENDIAN(uint32_t * reg, uint8_t epEndian) {
    (*reg) |= ((uint32_t) (epEndian) & 0x01U) << 7;
}

/**
 * set endpoint max packet size field
 * @param reg register
 * @param maxpcksize max packet size value
 */
static inline void SET_EP_CONF_MAXPKTSIZE(uint32_t * reg, uint16_t maxpcksize) {
    (*reg) |= (((uint32_t) (maxpcksize) & (uint32_t) 0x00FFU) << 16) | (((uint32_t) (maxpcksize) & (uint32_t) 0x1F00U) << 16);
}

/**
 * set buffering value field
 * @param reg register
 * @param buffering buffering value
 */
static inline void SET_EP_CONF_BUFFERING(uint32_t * reg, uint8_t buffering) {
    (*reg) |= ((uint32_t) (buffering) & 0x1FU) << 27;
}

/**
 * set mult value of endpoint object
 * @param reg register
 * @param mult mult value: 0,1 or 2
 */
static inline void SET_EP_CONF_MULT(uint32_t * reg, uint8_t mult) {
    (*reg) |= ((uint32_t) (mult) & 0x03U) << 14;
}

/**
 * set maxburst value of endpoint object in SS mode
 * @param reg register pointer
 * @param maxBurst
 */
static inline void SET_EP_CONF_MAXBURST(uint32_t * reg, uint8_t maxBurst) {
    (*reg) |= ((uint32_t) (maxBurst) & 0x0FU) << 8;
}

/**
 * set stream enable bits
 * @param reg register pointer
 */
static inline void SET_EP_CONF_STREAM_EN(uint32_t * reg) {
    (*reg) |= (uint32_t) 0x38;
}

/* EP_STS */
#define EP_STS_STPWAIT          0x80000000U
#define EP_STS_OUTQ_VAL         0x10000000U
#define EP_STS_OUTQ_NO          0x0F000000U
#define EP_STS_IOT              0x00080000U
#define EP_STS_SPSMST           0x00060000U
#define EP_STS_HOSTPP           0x00010000U
#define EP_STS_ISOERR           0x00008000U
#define EP_STS_OUTSMM           0x00004000U
#define EP_STS_SIDERR           0x00002000U
#define EP_STS_PRIME            0x00001000U
#define EP_STS_CCS              0x00000800U
#define EP_STS_BUFFEMPTY        0x00000400U
#define EP_STS_DBUSY            0x00000200U
#define EP_STS_NRDY             0x00000100U
#define EP_STS_TRBERR           0x00000080U
#define EP_STS_MD_EXIT          0x00000040U
#define EP_STS_STREAMR          0x00000020U
#define EP_STS_DESCMIS          0x00000010U
#define EP_STS_ISP              0x00000008U
#define EP_STS_IOC              0x00000004U
#define EP_STS_STALL            0x00000002U
#define EP_STS_SETUP            0x00000001U

/* EP_CMD */
#define EP_CMD_ERDY_SID

/**
 * set ERDY and SID field in endpoint command register
 * @param reg register
 * @param sid stream ID
 */
static inline void SET_EP_CMD_ERDY_SID(uint32_t * reg, uint8_t sid) {
    (*reg) |= ((uint32_t) (sid) & 0xFFFFU) << 16;
}

/**
 * set TDL value of endpoint command register
 * @param reg register
 * @param tdl TDL value
 */
static inline void SET_EP_CMD_TDL(uint32_t * reg, uint8_t tdl) {
    (*reg) |= ((uint32_t) (tdl) & 0x7FU) << 9;
}

#define EP_CMD_STDL             0x00000100U
#define EP_CMD_DFLUSH           0x00000080U
#define EP_CMD_DRDY             0x00000040U
#define EP_CMD_REQ_CMPL         0x00000020U
#define EP_CMD_ERDY             0x00000008U
#define EP_CMD_CSTALL           0x00000004U
#define EP_CMD_SSTALL           0x00000002U
#define EP_CMD_EPRST            0x00000001U

/* USB CONF */
#define USB_CONF_LGO_SSINACT    0x80000000U
#define USB_CONF_LGO_U2         0x40000000U
#define USB_CONF_LGO_U1         0x20000000U
#define USB_CONF_LGO_U0         0x10000000U
#define USB_CONF_U2DS           0x08000000U
#define USB_CONF_U2EN           0x04000000U
#define USB_CONF_U1DS           0x02000000U
#define USB_CONF_U1EN           0x01000000U
#define USB_CONF_CLK3OFFDS      0x00400000U
#define USB_CONF_CLK3OFFEN      0x00200000U
#define USB_CONF_LGO_L0         0x00100000U
#define USB_CONF_CLK2OFFDS      0x00080000U
#define USB_CONF_CLK2OFFEN      0x00040000U
#define USB_CONF_L1DS           0x00020000U
#define USB_CONF_L1EN           0x00010000U
#define USB_CONF_DEVDS          0x00008000U
#define USB_CONF_DEVEN          0x00004000U
#define USB_CONF_CFORCE_FS      0x00002000U
#define USB_CONF_SFORCE_FS      0x00001000U
#define USB_CONF_DMAOFFDS       0x00000800U
#define USB_CONF_DMAOFFEN       0x00000400U
#define USB_CONF_DMULT          0x00000200U
#define USB_CONF_DSING          0x00000100U
#define USB_CONF_SWRST          0x00000080U
#define USB_CONF_BENDIAN        0x00000040U
#define USB_CONF_LENDIAN        0x00000020U
#define USB_CONF_USB2DIS        0x00000010U
#define USB_CONF_USB3DIS        0x00000008U
#define USB_CONF_CFGSET         0x00000002U
#define USB_CONF_CFGRST         0x00000001U

/* USB_STS */

/**
 * return actual speed value
 * @param reg register to read
 * @return return speed value
 */
static inline uint8_t GET_USB_STS_SPEED(uint32_t reg) {

    return (((uint8_t) (reg) >> 4) & 0x07U);
}

/**
 * return memory overflow bit
 * @param reg register
 * @return memory overflow bit
 */
static inline uint8_t GET_USB_STS_MEM_OV(uint32_t reg) {

    return (((uint8_t) (reg) >> 1) & 0x01U);
}

#define USB_STS_MEM_OV          0x00000002U
#define USB_STS_IN_RST          0x00000400U

/* USB_CMD */

#define USBRV_TM_TEST_J      0x00U
#define USBRV_TM_TEST_K      0x01U
#define USBRV_TM_SE0_NAK     0x02U
#define USBRV_TM_TEST_PACKET 0x03U

/**
 * set testing mode
 * @param reg register
 * @param tmodesel test selector
 */
static inline void SET_USB_CMD_TMODE_SEL(uint32_t * reg, uint8_t tmodesel) {
    (*reg) |= ((uint32_t) (tmodesel) & 0x03U) << 10;
}

/* EP_STS_EN */
#define EP_STS_EN_STPWAITEN     0x80000000U
#define EP_STS_EN_IOTEN         0x00080000U
#define EP_STS_EN_ISOERREN      0x00008000U
#define EP_STS_EN_OUTSMMEN      0x00004000U
#define EP_STS_EN_SIDERREN      0x00002000U
#define EP_STS_EN_PRIMEEN       0x00001000U
#define EP_STS_EN_NRDYEN        0x00000100U
#define EP_STS_EN_TRBERREN      0x00000080U
#define EP_STS_EN_MD_EXITEN     0x00000040U
#define EP_STS_EN_STREAMREN     0x00000020U
#define EP_STS_EN_DESCMISEN     0x00000010U
#define EP_STS_EN_SETUPEN       0x00000001U

/* USB_IEN */
#define USB_IEN_UWRESEIEN       0x20000000U
#define USB_IEN_UWRESSIEN       0x10000000U
#define USB_IEN_CFGRESIEN       0x04000000U
#define USB_IEN_L1EXTIEN        0x02000000U
#define USB_IEN_L1ENTIEN        0x01000000U
#define USB_IEN_L2EXTIEN        0x00200000U
#define USB_IEN_L2ENTIEN        0x00100000U
#define USB_IEN_U2RESIEN        0x00040000U
#define USB_IEN_DIS2I           0x00020000U
#define USB_IEN_CON2I           0x00010000U
#define USB_IEN_SPKTIEN         0x00001000U
#define USB_IEN_WAKEIEN         0x00000800U
#define USB_IEN_ITPIEN          0x00000400U
#define USB_IEN_U1EXTIEN        0x00000200U
#define USB_IEN_U1ENTIEN        0x00000100U
#define USB_IEN_U2EXTIEN        0x00000080U
#define USB_IEN_U2ENTIEN        0x00000040U
#define USB_IEN_U3EXTIEN        0x00000020U
#define USB_IEN_U3ENTIEN        0x00000010U
#define USB_IEN_UHRESIEN        0x00000008U
#define USB_IEN_UWRESIEN        0x00000004U
#define USB_IEN_DISIEN          0x00000002U
#define USB_IEN_CONIEN          0x00000001U

/* USB_ISTS */
#define USB_ISTS_UWRESEI        0x20000000U
#define USB_ISTS_UWRESSI        0x10000000U
#define USB_ISTS_CFGRESI        0x04000000U
#define USB_ISTS_L1EXTI         0x02000000U
#define USB_ISTS_L1ENTI         0x01000000U
#define USB_ISTS_L2EXTI         0x00200000U
#define USB_ISTS_L2ENTI         0x00100000U
#define USB_ISTS_U2RESI         0x00040000U
#define USB_ISTS_DIS2I          0x00020000U
#define USB_ISTS_CON2I          0x00010000U
#define USB_ISTS_SPKTI          0x00001000U
#define USB_ISTS_WAKEI          0x00000800U
#define USB_ISTS_ITPI           0x00000400U
#define USB_ISTS_U1EXTI         0x00000200U
#define USB_ISTS_U1ENTI         0x00000100U
#define USB_ISTS_U2EXTI         0x00000080U
#define USB_ISTS_U2ENTI         0x00000040U
#define USB_ISTS_U3EXTI         0x00000020U
#define USB_ISTS_U3ENTI         0x00000010U
#define USB_ISTS_UHRESI         0x00000008U
#define USB_ISTS_UWRESI         0x00000004U
#define USB_ISTS_DISI           0x00000002U
#define USB_ISTS_CONI           0x00000001U

/* USB_PWR */
#define USB_PWR_FAST_REG_ACCESS      0x80000000U
#define USB_PWR_FAST_REG_ACCESS_STAT 0x40000000U

/* USBR_CAP6 */
#define DEV_VER_V1              0x00024509U
#define DEV_VER_V3              0x0002450dU

/**
 * Get the base version of the device
 * @param reg: USBR_CAP6 register
 * @return bit[23:0] corresponding to base version
 */
static inline uint32_t GET_USBR_CAP6_DEV_BASE_VER(uint32_t reg) {
    return ((uint32_t) (reg & 0xFFFFFFU));
}

/* BUF_CTRL */
#define BUF_CTRL_BUF_CMD_IWD    0x00000020U
#define BUF_CTRL_BUF_CMD_SET    0x00000001U
#define BUF_CTRL_BUF_CMD_STS    0x80000000U

/* DBG_LINK1 */
#define LINK1_LFPS_MIN_GEN_U1_EXIT_SET  0x02000000U
#define LINK1_LFPS_MIN_GEN_U1_EXIT      0x0000FF00U

/* DMA_AXI_CTRL */
#define DMA_AXI_CTRL_MARPROT_OFFST      0x00U
#define DMA_AXI_CTRL_MARPROT_MASK       ((uint32_t) 0x00000007U)
#define DMA_AXI_CTRL_MAWPROT_OFFST      0x10U
#define DMA_AXI_CTRL_MAWPROT_MASK       ((uint32_t) 0x00070000U)
#define DMA_AXI_CTRL_NON_SECURE         0x02U

#ifdef CUSBDSS_DMA_ACCESS_NON_SECURE
/**
 * Set MARPROT bits in DMA_AXI_CTRL register value
 * @param reg: Old value of DMA_AXI_CTRL reg
 * @param marprot: MARPROT value
 * @return: New value of DMA_AXI_CTRL reg
 */
static inline uint32_t SET_DMA_AXI_CTRL_MARPROT(uint32_t reg, uint32_t marprot) {
    uint32_t regval = reg & (~DMA_AXI_CTRL_MARPROT_MASK);
    regval |= ((marprot << DMA_AXI_CTRL_MARPROT_OFFST) & DMA_AXI_CTRL_MARPROT_MASK);
    return regval;
}

/**
 * Set MAWPROT bits in DMA_AXI_CTRL register value
 * @param reg: Old value of DMA_AXI_CTRL reg
 * @param mawprot: MAWPROT value
 * @return: New value of DMA_AXI_CTRL reg
 */
static inline uint32_t SET_DMA_AXI_CTRL_MAWPROT(uint32_t reg, uint32_t mawprot) {
    uint32_t regval = reg & (~DMA_AXI_CTRL_MAWPROT_MASK);
    regval |= ((mawprot << DMA_AXI_CTRL_MAWPROT_OFFST) & DMA_AXI_CTRL_MAWPROT_MASK);
    return regval;
}
#endif


#ifdef __cplusplus
}
#endif

#endif /* SS_DEV_HW_H */

