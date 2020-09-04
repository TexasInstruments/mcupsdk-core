/********************************************************************
 * Copyright (C) 2020 Texas Instruments Incorporated.
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
 *  Name        : cslr_hsm_soc_ctrl.h
*/
#ifndef CSLR_HSM_SOC_CTRL_H_
#define CSLR_HSM_SOC_CTRL_H_

#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef volatile struct CSL_HSMSocCtrlRegs_t
{
    uint32_t PID ;                                                         /**< @brief   Offset: 0x0000 */
    uint32_t HW_REG0 ;                                                     /**< @brief   Offset: 0x0004 */
    uint32_t HW_REG1 ;                                                     /**< @brief   Offset: 0x0008 */
    uint32_t HW_REG2 ;                                                     /**< @brief   Offset: 0x000c */
    uint32_t HW_REG3 ;                                                     /**< @brief   Offset: 0x0010 */
    uint32_t SECAP_TX_DATA ;                                               /**< @brief   Offset: 0x0014 */
    uint32_t SECAP_TX_CONTROL ;                                            /**< @brief   Offset: 0x0018 */
    uint32_t SECAP_RX_DATA ;                                               /**< @brief   Offset: 0x001c */
    uint32_t SECAP_RX_CONTROL ;                                            /**< @brief   Offset: 0x0020 */
    uint32_t HSM_MBOX_MEMINIT_START ;                                      /**< @brief   Offset: 0x0024 */
    uint32_t HSM_MBOX_MEMINIT_STATUS ;                                     /**< @brief   Offset: 0x0028 */
    uint32_t HSM_MBOX_MEMINIT_DONE ;                                       /**< @brief   Offset: 0x002c */
    uint32_t HSM_MBOX_WRITE_DONE ;                                         /**< @brief   Offset: 0x0030 */
    uint32_t HSM_MBOX_READ_REQ ;                                           /**< @brief   Offset: 0x0034 */
    uint32_t HSM_MBOX_READ_DONE ;                                          /**< @brief   Offset: 0x0038 */
    uint32_t HSM_CAPEVNT_SEL0 ;                                            /**< @brief   Offset: 0x003c */
    uint32_t HSM_DMA_REQ ;                                                 /**< @brief   Offset: 0x0040 */
    uint32_t HSM_IRQ_REQ ;                                                 /**< @brief   Offset: 0x0044 */
    uint32_t HSM_ESM_REQ ;                                                 /**< @brief   Offset: 0x0048 */
    uint32_t HSM_ESM_GRP2_MASK0 ;                                          /**< @brief   Offset: 0x004c */
    uint32_t HSM_ESM_GRP2_MASK1 ;                                          /**< @brief   Offset: 0x0050 */
    uint32_t HSM_ESM_GRP2_MASK2 ;                                          /**< @brief   Offset: 0x0054 */
    uint32_t HSM_ESM_GRP2_MASK3 ;                                          /**< @brief   Offset: 0x0058 */
    uint32_t HSM_ESM_GRP3_MASK0 ;                                          /**< @brief   Offset: 0x005c */
    uint32_t HSM_ESM_GRP3_MASK1 ;                                          /**< @brief   Offset: 0x0060 */
    uint32_t HSM_ESM_GRP3_MASK2 ;                                          /**< @brief   Offset: 0x0064 */
    uint32_t HSM_ESM_GRP3_MASK3 ;                                          /**< @brief   Offset: 0x0068 */
    uint32_t HSM_MSS_RST_CAUSE ;                                           /**< @brief   Offset: 0x006c */
    uint32_t HSM_MSS_RST_CAUSE_CLR ;                                       /**< @brief   Offset: 0x0070 */
    uint32_t HSM_SECURE_BOOT_INFO_REG0 ;                                   /**< @brief   Offset: 0x0074 */
    uint32_t HSM_SECURE_BOOT_INFO_REG1 ;                                   /**< @brief   Offset: 0x0078 */
    uint32_t HSM_SECURE_BOOT_INFO_REG2 ;                                   /**< @brief   Offset: 0x007c */
    uint32_t HSM_SECURE_BOOT_INFO_REG3 ;                                   /**< @brief   Offset: 0x0080 */
    uint32_t HSM_SECURE_BOOT_INFO_REG4 ;                                   /**< @brief   Offset: 0x0084 */
    uint32_t HSM_SECURE_BOOT_INFO_REG5 ;                                   /**< @brief   Offset: 0x0088 */
    uint32_t HSM_SECURE_BOOT_INFO_REG6 ;                                   /**< @brief   Offset: 0x008c */
    uint32_t HSM_SECURE_BOOT_INFO_REG7 ;                                   /**< @brief   Offset: 0x0090 */
    uint32_t HSM_CR5SS_RST_INTAGG_MASK ;                                   /**< @brief   Offset: 0x0094 */
    uint32_t HSM_CR5SS_RST_INTAGG_STATUS ;                                 /**< @brief   Offset: 0x0098 */
    uint32_t HSM_CR5SS_RST_INTAGG_STATUS_RAW ;                             /**< @brief   Offset: 0x009c */
    uint32_t HSM_TO_MSS_INT_MASK ;                                         /**< @brief   Offset: 0x00a0 */
    uint32_t HSM_WDT_RST_CONFIG ;                                          /**< @brief   Offset: 0x00a4 */
    uint32_t HSM_PERIPH_ERRAGG_MASK0 ;                                     /**< @brief   Offset: 0x00a8 */
    uint32_t HSM_PERIPH_ERRAGG_STATUS0 ;                                   /**< @brief   Offset: 0x00ac */
    uint32_t HSM_PERIPH_ERRAGG_STATUS_RAW0 ;                               /**< @brief   Offset: 0x00b0 */
    uint32_t HSM_PERIPH_ERRAGG_MASK1 ;                                     /**< @brief   Offset: 0x00b4 */
    uint32_t HSM_PERIPH_ERRAGG_STATUS1 ;                                   /**< @brief   Offset: 0x00b8 */
    uint32_t HSM_PERIPH_ERRAGG_STATUS_RAW1 ;                               /**< @brief   Offset: 0x00bc */
    uint32_t HSM_CM4_AHB_ERRAGG_MASK ;                                     /**< @brief   Offset: 0x00c0 */
    uint32_t HSM_CM4_AHB_ERRAGG_STATUS ;                                   /**< @brief   Offset: 0x00c4 */
    uint32_t HSM_CM4_AHB_ERRAGG_STATUS_RAW ;                               /**< @brief   Offset: 0x00c8 */
    uint32_t HSM_MPU_ERRAGG_MASK0 ;                                        /**< @brief   Offset: 0x00cc */
    uint32_t HSM_MPU_ERRAGG_STATUS0 ;                                      /**< @brief   Offset: 0x00d0 */
    uint32_t HSM_MPU_ERRAGG_STATUS_RAW0 ;                                  /**< @brief   Offset: 0x00d4 */
    uint32_t HSM_MPU_ERRAGG_MASK1 ;                                        /**< @brief   Offset: 0x00d8 */
    uint32_t HSM_MPU_ERRAGG_STATUS1 ;                                      /**< @brief   Offset: 0x00dc */
    uint32_t HSM_MPU_ERRAGG_STATUS_RAW1 ;                                  /**< @brief   Offset: 0x00e0 */
    uint32_t RESERVED0[199] ;                                              /**< @brief   Offset: 0x00e4 */
    uint32_t ISC_CTRL_REG_HSM_CM4 ;                                        /**< @brief   Offset: 0x0400 */
    uint32_t ISC_CTRL_REG_HSM_TPTC_A0 ;                                    /**< @brief   Offset: 0x0404 */
    uint32_t ISC_CTRL_REG_HSM_TPTC_A1 ;                                    /**< @brief   Offset: 0x0408 */
    uint32_t HSM_PBIST_SELFTEST ;                                          /**< @brief   Offset: 0x040c */
    uint32_t RESERVED1[252] ;                                              /**< @brief   Offset: 0x0410 */
    uint32_t ISC_CTRL_REG_MSS_R5FA_AXI ;                                   /**< @brief   Offset: 0x0800 */
    uint32_t ISC_CTRL_REG_MSS_R5FB_AXI ;                                   /**< @brief   Offset: 0x0804 */
    uint32_t ISC_CTRL_REG_MSS_TPTC_A0 ;                                    /**< @brief   Offset: 0x0808 */
    uint32_t ISC_CTRL_REG_MSS_TPTC_A1 ;                                    /**< @brief   Offset: 0x080c */
    uint32_t ISC_CTRL_REG_MSS_TPTC_B0 ;                                    /**< @brief   Offset: 0x0810 */
    uint32_t ISC_CTRL_REG_MSS_ETHERNET_DMA ;                               /**< @brief   Offset: 0x0814 */
    uint32_t ISC_CTRL_REG_MSS_R5FA_I_AXI ;                                 /**< @brief   Offset: 0x0818 */
    uint32_t ISC_CTRL_REG_MSS_R5FB_I_AXI ;                                 /**< @brief   Offset: 0x081c */
    uint32_t ISC_CTRL_REG_DBG_JTAG ;                                       /**< @brief   Offset: 0x0820 */
    uint32_t ISC_CTRL_REG_DBG_RS232 ;                                      /**< @brief   Offset: 0x0824 */
    uint32_t ISC_CTRL_REG_DMM ;                                            /**< @brief   Offset: 0x0828 */
    uint32_t ISC_CTRL_REG_DSS_MDMA ;                                       /**< @brief   Offset: 0x082c */
    uint32_t ISC_CTRL_REG_DSS_TPTC_A0 ;                                    /**< @brief   Offset: 0x0830 */
    uint32_t ISC_CTRL_REG_DSS_TPTC_A1 ;                                    /**< @brief   Offset: 0x0834 */
    uint32_t ISC_CTRL_REG_DSS_TPTC_B0 ;                                    /**< @brief   Offset: 0x0838 */
    uint32_t ISC_CTRL_REG_DSS_TPTC_B1 ;                                    /**< @brief   Offset: 0x083c */
    uint32_t ISC_CTRL_REG_DSS_TPTC_C0 ;                                    /**< @brief   Offset: 0x0840 */
    uint32_t ISC_CTRL_REG_DSS_TPTC_C1 ;                                    /**< @brief   Offset: 0x0844 */
    uint32_t ISC_CTRL_REG_DSS_TPTC_C2 ;                                    /**< @brief   Offset: 0x0848 */
    uint32_t ISC_CTRL_REG_DSS_TPTC_C3 ;                                    /**< @brief   Offset: 0x084c */
    uint32_t ISC_CTRL_REG_DSS_TPTC_C4 ;                                    /**< @brief   Offset: 0x0850 */
    uint32_t ISC_CTRL_REG_DSS_TPTC_C5 ;                                    /**< @brief   Offset: 0x0854 */
    uint32_t ISC_CTRL_REG_DSS_HWA_CM4 ;                                    /**< @brief   Offset: 0x0858 */
    uint32_t ISC_CTRL_REG_RCSS_CSIA ;                                      /**< @brief   Offset: 0x085c */
    uint32_t ISC_CTRL_REG_RCSS_CSIB ;                                      /**< @brief   Offset: 0x0860 */
    uint32_t ISC_CTRL_REG_RCSS_TPTC_A0 ;                                   /**< @brief   Offset: 0x0864 */
    uint32_t ISC_CTRL_REG_RCSS_TPTC_A1 ;                                   /**< @brief   Offset: 0x0868 */
    uint32_t ISC_CTRL_REG_RCSS_ICSS_A ;                                    /**< @brief   Offset: 0x086c */
    uint32_t ISC_CTRL_REG_RCSS_ICSS_B ;                                    /**< @brief   Offset: 0x0870 */
    uint32_t DBG_MASTERS_PRIVID_OVERRIDE ;                                 /**< @brief   Offset: 0x0874 */
    uint32_t HSM_RTC_CFG_CTRL ;                                            /**< @brief   Offset: 0x0878 */
    uint32_t HSM_RTC_CFG_STATUS ;                                          /**< @brief   Offset: 0x087c */
    uint32_t HSM_RTC_MSB_COUNTER ;                                         /**< @brief   Offset: 0x0880 */
    uint32_t HSM_RTC_LSB_COUNTER ;                                         /**< @brief   Offset: 0x0884 */
    uint32_t HSM_TO_MSS_DMA_MASK ;                                         /**< @brief   Offset: 0x0888 */
    uint32_t EFUSE_STATUS ;                                                /**< @brief   Offset: 0x088c */
    uint32_t RESERVED2[464] ;                                              /**< @brief   Offset: 0x0890 */
    uint32_t HW_SPARE_RW0 ;                                                /**< @brief   Offset: 0x0fd0 */
    uint32_t HW_SPARE_RW1 ;                                                /**< @brief   Offset: 0x0fd4 */
    uint32_t HW_SPARE_RW2 ;                                                /**< @brief   Offset: 0x0fd8 */
    uint32_t HW_SPARE_RW3 ;                                                /**< @brief   Offset: 0x0fdc */
    uint32_t HW_SPARE_RO0 ;                                                /**< @brief   Offset: 0x0fe0 */
    uint32_t HW_SPARE_RO1 ;                                                /**< @brief   Offset: 0x0fe4 */
    uint32_t HW_SPARE_RO2 ;                                                /**< @brief   Offset: 0x0fe8 */
    uint32_t HW_SPARE_RO3 ;                                                /**< @brief   Offset: 0x0fec */
    uint32_t HSM_MBOX_READ_DONE_ACK ;                                      /**< @brief   Offset: 0x0ff0 */
    uint32_t HW_SPARE_REC ;                                                /**< @brief   Offset: 0x0ff4 */
    uint32_t RESERVED3[4] ;                                                /**< @brief   Offset: 0x0ff8 */
    uint32_t LOCK0_KICK0 ;                                                 /**< @brief   Offset: 0x1008 */
    uint32_t LOCK0_KICK1 ;                                                 /**< @brief   Offset: 0x100c */
    uint32_t INTR_RAW_STATUS ;                                             /**< @brief   Offset: 0x1010 */
    uint32_t INTR_ENABLED_STATUS_CLEAR ;                                   /**< @brief   Offset: 0x1014 */
    uint32_t INTR_ENABLE ;                                                 /**< @brief   Offset: 0x1018 */
    uint32_t INTR_ENABLE_CLEAR ;                                           /**< @brief   Offset: 0x101c */
    uint32_t EOI ;                                                         /**< @brief   Offset: 0x1020 */
    uint32_t FAULT_ADDRESS ;                                               /**< @brief   Offset: 0x1024 */
    uint32_t FAULT_TYPE_STATUS ;                                           /**< @brief   Offset: 0x1028 */
    uint32_t FAULT_ATTR_STATUS ;                                           /**< @brief   Offset: 0x102c */
    uint32_t FAULT_CLEAR ;                                                 /**< @brief   Offset: 0x1030 */
} CSL_hsm_soc_ctrlRegs;

#ifdef __cplusplus
}
#endif
#endif
