/*
 *  Copyright (C) 2021 Texas Instruments Incorporated.
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
 */

#ifndef CSLR_FSI_RX_H_
#define CSLR_FSI_RX_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/**************************************************************************
* Module Base Offset Values
**************************************************************************/

#define CSL_FSI_RX_CFG_REGS_BASE                                               (0x00000000U)


/**************************************************************************
* Hardware Region  : FSI Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint16_t RX_MASTER_CTRL_ALTB_;
    volatile uint16_t RESERVED_1;
    volatile uint16_t RESERVED_2;
    volatile uint16_t RESERVED_3;
    volatile uint16_t RX_OPER_CTRL;
    volatile uint16_t RESERVED_4;
    volatile uint16_t RX_FRAME_INFO;
    volatile uint16_t RX_FRAME_TAG_UDATA;
    volatile uint16_t RX_DMA_CTRL;
    volatile uint16_t RESERVED_5;
    volatile uint16_t RX_EVT_STS_ALT1_;
    volatile uint16_t RX_CRC_INFO;
    volatile uint16_t RX_EVT_CLR_ALT1_;
    volatile uint16_t RX_EVT_FRC_ALT1_;
    volatile uint16_t RX_BUF_PTR_LOAD;
    volatile uint16_t RX_BUF_PTR_STS;
    volatile uint16_t RX_FRAME_WD_CTRL;
    volatile uint16_t RESERVED_6;
    volatile uint32_t RX_FRAME_WD_REF;
    volatile uint32_t RX_FRAME_WD_CNT;
    volatile uint16_t RX_PING_WD_CTRL;
    volatile uint16_t RX_PING_TAG;
    volatile uint32_t RX_PING_WD_REF;
    volatile uint32_t RX_PING_WD_CNT;
    volatile uint16_t RX_INT1_CTRL_ALT1_;
    volatile uint16_t RX_INT2_CTRL_ALT1_;
    volatile uint16_t RX_LOCK_CTRL;
    volatile uint16_t RESERVED_7;
    volatile uint32_t RX_ECC_DATA;
    volatile uint16_t RX_ECC_VAL;
    volatile uint16_t RESERVED_8;
    volatile uint32_t RX_ECC_SEC_DATA;
    volatile uint16_t RX_ECC_LOG;
    volatile uint16_t RESERVED_9;
    volatile uint16_t RX_FRAME_TAG_CMP;
    volatile uint16_t RX_PING_TAG_CMP;
    volatile uint16_t RESERVED_10[2];
    volatile uint32_t RX_TRIG_CTRL_0;
    volatile uint32_t RX_TRIG_WIDTH_0;
    volatile uint16_t RX_DLYLINE_CTRL;
    volatile uint16_t RESERVED_11[1];
    volatile uint32_t RX_TRIG_CTRL_1;
    volatile uint32_t RX_TRIG_CTRL_2;
    volatile uint32_t RX_TRIG_CTRL_3;
    volatile uint32_t RX_VIS_1;
    volatile uint16_t RX_UDATA_FILTER;
    volatile uint16_t RESERVED_12[5];
    volatile uint16_t RX_BUF_BASE[16];
} CSL_fsi_rx_cfgRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB_                                    (0x00000000U)
#define CSL_FSI_RX_CFG_RESERVED_1                                              (0x00000002U)
#define CSL_FSI_RX_CFG_RESERVED_2                                              (0x00000004U)
#define CSL_FSI_RX_CFG_RESERVED_3                                              (0x00000006U)
#define CSL_FSI_RX_CFG_RX_OPER_CTRL                                            (0x00000008U)
#define CSL_FSI_RX_CFG_RESERVED_4                                              (0x0000000AU)
#define CSL_FSI_RX_CFG_RX_FRAME_INFO                                           (0x0000000CU)
#define CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA                                      (0x0000000EU)
#define CSL_FSI_RX_CFG_RX_DMA_CTRL                                             (0x00000010U)
#define CSL_FSI_RX_CFG_RESERVED_5                                              (0x00000012U)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1_                                        (0x00000014U)
#define CSL_FSI_RX_CFG_RX_CRC_INFO                                             (0x00000016U)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1_                                        (0x00000018U)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1_                                        (0x0000001AU)
#define CSL_FSI_RX_CFG_RX_BUF_PTR_LOAD                                         (0x0000001CU)
#define CSL_FSI_RX_CFG_RX_BUF_PTR_STS                                          (0x0000001EU)
#define CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL                                        (0x00000020U)
#define CSL_FSI_RX_CFG_RESERVED_6                                              (0x00000022U)
#define CSL_FSI_RX_CFG_RX_FRAME_WD_REF                                         (0x00000024U)
#define CSL_FSI_RX_CFG_RX_FRAME_WD_CNT                                         (0x00000028U)
#define CSL_FSI_RX_CFG_RX_PING_WD_CTRL                                         (0x0000002CU)
#define CSL_FSI_RX_CFG_RX_PING_TAG                                             (0x0000002EU)
#define CSL_FSI_RX_CFG_RX_PING_WD_REF                                          (0x00000030U)
#define CSL_FSI_RX_CFG_RX_PING_WD_CNT                                          (0x00000034U)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1_                                      (0x00000038U)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1_                                      (0x0000003AU)
#define CSL_FSI_RX_CFG_RX_LOCK_CTRL                                            (0x0000003CU)
#define CSL_FSI_RX_CFG_RESERVED_7                                              (0x0000003EU)
#define CSL_FSI_RX_CFG_RX_ECC_DATA                                             (0x00000040U)
#define CSL_FSI_RX_CFG_RX_ECC_VAL                                              (0x00000044U)
#define CSL_FSI_RX_CFG_RESERVED_8                                              (0x00000046U)
#define CSL_FSI_RX_CFG_RX_ECC_SEC_DATA                                         (0x00000048U)
#define CSL_FSI_RX_CFG_RX_ECC_LOG                                              (0x0000004CU)
#define CSL_FSI_RX_CFG_RESERVED_9                                              (0x0000004EU)
#define CSL_FSI_RX_CFG_RX_FRAME_TAG_CMP                                        (0x00000050U)
#define CSL_FSI_RX_CFG_RX_PING_TAG_CMP                                         (0x00000052U)
#define CSL_FSI_RX_CFG_RESERVED_10(RESERVED_10)                                (0x00000054U+((RESERVED_10)*0x2U))
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL_0                                          (0x00000058U)
#define CSL_FSI_RX_CFG_RX_TRIG_WIDTH_0                                         (0x0000005CU)
#define CSL_FSI_RX_CFG_RX_DLYLINE_CTRL                                         (0x00000060U)
#define CSL_FSI_RX_CFG_RESERVED_11(RESERVED_11)                                (0x00000062U+((RESERVED_11)*0x2U))
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL_1                                          (0x00000064U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL_2                                          (0x00000068U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL_3                                          (0x0000006CU)
#define CSL_FSI_RX_CFG_RX_VIS_1                                                (0x00000070U)
#define CSL_FSI_RX_CFG_RX_UDATA_FILTER                                         (0x00000074U)
#define CSL_FSI_RX_CFG_RESERVED_12(RESERVED_12)                                (0x00000076U+((RESERVED_12)*0x2U))
#define CSL_FSI_RX_CFG_RX_BUF_BASE(RX_BUF_BASE)                                (0x00000080U+((RX_BUF_BASE)*0x2U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* RX_MASTER_CTRL_ALTB_ */

#define CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__CORE_RST_MASK                      (0x0001U)
#define CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__CORE_RST_SHIFT                     (0x0000U)
#define CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__CORE_RST_MAX                       (0x0001U)

#define CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__INT_LOOPBACK_MASK                  (0x0002U)
#define CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__INT_LOOPBACK_SHIFT                 (0x0001U)
#define CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__INT_LOOPBACK_MAX                   (0x0001U)

#define CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__SPI_PAIRING_MASK                   (0x0004U)
#define CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__SPI_PAIRING_SHIFT                  (0x0002U)
#define CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__SPI_PAIRING_MAX                    (0x0001U)

#define CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__INPUT_ISOLATE_MASK                 (0x0008U)
#define CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__INPUT_ISOLATE_SHIFT                (0x0003U)
#define CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__INPUT_ISOLATE_MAX                  (0x0001U)

#define CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__DATA_FILTER_EN_MASK                (0x0010U)
#define CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__DATA_FILTER_EN_SHIFT               (0x0004U)
#define CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__DATA_FILTER_EN_MAX                 (0x0001U)

#define CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__RESERVED_1_MASK                    (0x00E0U)
#define CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__RESERVED_1_SHIFT                   (0x0005U)
#define CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__RESERVED_1_MAX                     (0x0007U)

#define CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__KEY_MASK                           (0xFF00U)
#define CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__KEY_SHIFT                          (0x0008U)
#define CSL_FSI_RX_CFG_RX_MASTER_CTRL_ALTB__KEY_MAX                            (0x00FFU)

/* RESERVED_1 */

/* RESERVED_2 */

/* RESERVED_3 */

/* RX_OPER_CTRL */

#define CSL_FSI_RX_CFG_RX_OPER_CTRL_DATA_WIDTH_MASK                            (0x0003U)
#define CSL_FSI_RX_CFG_RX_OPER_CTRL_DATA_WIDTH_SHIFT                           (0x0000U)
#define CSL_FSI_RX_CFG_RX_OPER_CTRL_DATA_WIDTH_MAX                             (0x0003U)

#define CSL_FSI_RX_CFG_RX_OPER_CTRL_SPI_MODE_MASK                              (0x0004U)
#define CSL_FSI_RX_CFG_RX_OPER_CTRL_SPI_MODE_SHIFT                             (0x0002U)
#define CSL_FSI_RX_CFG_RX_OPER_CTRL_SPI_MODE_MAX                               (0x0001U)

#define CSL_FSI_RX_CFG_RX_OPER_CTRL_N_WORDS_MASK                               (0x0078U)
#define CSL_FSI_RX_CFG_RX_OPER_CTRL_N_WORDS_SHIFT                              (0x0003U)
#define CSL_FSI_RX_CFG_RX_OPER_CTRL_N_WORDS_MAX                                (0x000FU)

#define CSL_FSI_RX_CFG_RX_OPER_CTRL_ECC_SEL_MASK                               (0x0080U)
#define CSL_FSI_RX_CFG_RX_OPER_CTRL_ECC_SEL_SHIFT                              (0x0007U)
#define CSL_FSI_RX_CFG_RX_OPER_CTRL_ECC_SEL_MAX                                (0x0001U)

#define CSL_FSI_RX_CFG_RX_OPER_CTRL_PING_WD_RST_MODE_MASK                      (0x0100U)
#define CSL_FSI_RX_CFG_RX_OPER_CTRL_PING_WD_RST_MODE_SHIFT                     (0x0008U)
#define CSL_FSI_RX_CFG_RX_OPER_CTRL_PING_WD_RST_MODE_MAX                       (0x0001U)

#define CSL_FSI_RX_CFG_RX_OPER_CTRL_RESERVED_1_MASK                            (0xFE00U)
#define CSL_FSI_RX_CFG_RX_OPER_CTRL_RESERVED_1_SHIFT                           (0x0009U)
#define CSL_FSI_RX_CFG_RX_OPER_CTRL_RESERVED_1_MAX                             (0x007FU)

/* RESERVED_4 */

/* RX_FRAME_INFO */

#define CSL_FSI_RX_CFG_RX_FRAME_INFO_FRAME_TYPE_MASK                           (0x000FU)
#define CSL_FSI_RX_CFG_RX_FRAME_INFO_FRAME_TYPE_SHIFT                          (0x0000U)
#define CSL_FSI_RX_CFG_RX_FRAME_INFO_FRAME_TYPE_MAX                            (0x000FU)

#define CSL_FSI_RX_CFG_RX_FRAME_INFO_RESERVED_1_MASK                           (0xFFF0U)
#define CSL_FSI_RX_CFG_RX_FRAME_INFO_RESERVED_1_SHIFT                          (0x0004U)
#define CSL_FSI_RX_CFG_RX_FRAME_INFO_RESERVED_1_MAX                            (0x0FFFU)

/* RX_FRAME_TAG_UDATA */

#define CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA_ZERO_MASK                            (0x0001U)
#define CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA_ZERO_SHIFT                           (0x0000U)
#define CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA_ZERO_MAX                             (0x0001U)

#define CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA_FRAME_TAG_MASK                       (0x001EU)
#define CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA_FRAME_TAG_SHIFT                      (0x0001U)
#define CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA_FRAME_TAG_MAX                        (0x000FU)

#define CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA_RESERVED_1_MASK                      (0x00E0U)
#define CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA_RESERVED_1_SHIFT                     (0x0005U)
#define CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA_RESERVED_1_MAX                       (0x0007U)

#define CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA_USER_DATA_MASK                       (0xFF00U)
#define CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA_USER_DATA_SHIFT                      (0x0008U)
#define CSL_FSI_RX_CFG_RX_FRAME_TAG_UDATA_USER_DATA_MAX                        (0x00FFU)

/* RX_DMA_CTRL */

#define CSL_FSI_RX_CFG_RX_DMA_CTRL_DMA_EVT_EN_MASK                             (0x0001U)
#define CSL_FSI_RX_CFG_RX_DMA_CTRL_DMA_EVT_EN_SHIFT                            (0x0000U)
#define CSL_FSI_RX_CFG_RX_DMA_CTRL_DMA_EVT_EN_MAX                              (0x0001U)

#define CSL_FSI_RX_CFG_RX_DMA_CTRL_RESERVED_1_MASK                             (0xFFFEU)
#define CSL_FSI_RX_CFG_RX_DMA_CTRL_RESERVED_1_SHIFT                            (0x0001U)
#define CSL_FSI_RX_CFG_RX_DMA_CTRL_RESERVED_1_MAX                              (0x7FFFU)

/* RESERVED_5 */

/* RX_EVT_STS_ALT1_ */

#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__PING_WD_TO_MASK                        (0x0001U)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__PING_WD_TO_SHIFT                       (0x0000U)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__PING_WD_TO_MAX                         (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__FRAME_WD_TO_MASK                       (0x0002U)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__FRAME_WD_TO_SHIFT                      (0x0001U)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__FRAME_WD_TO_MAX                        (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__CRC_ERR_MASK                           (0x0004U)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__CRC_ERR_SHIFT                          (0x0002U)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__CRC_ERR_MAX                            (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__TYPE_ERR_MASK                          (0x0008U)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__TYPE_ERR_SHIFT                         (0x0003U)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__TYPE_ERR_MAX                           (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__EOF_ERR_MASK                           (0x0010U)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__EOF_ERR_SHIFT                          (0x0004U)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__EOF_ERR_MAX                            (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__BUF_OVERRUN_MASK                       (0x0020U)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__BUF_OVERRUN_SHIFT                      (0x0005U)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__BUF_OVERRUN_MAX                        (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__FRAME_DONE_MASK                        (0x0040U)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__FRAME_DONE_SHIFT                       (0x0006U)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__FRAME_DONE_MAX                         (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__BUF_UNDERRUN_MASK                      (0x0080U)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__BUF_UNDERRUN_SHIFT                     (0x0007U)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__BUF_UNDERRUN_MAX                       (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__ERR_FRAME_MASK                         (0x0100U)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__ERR_FRAME_SHIFT                        (0x0008U)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__ERR_FRAME_MAX                          (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__PING_FRAME_MASK                        (0x0200U)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__PING_FRAME_SHIFT                       (0x0009U)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__PING_FRAME_MAX                         (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__FRAME_OVERRUN_MASK                     (0x0400U)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__FRAME_OVERRUN_SHIFT                    (0x000AU)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__FRAME_OVERRUN_MAX                      (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__DATA_FRAME_MASK                        (0x0800U)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__DATA_FRAME_SHIFT                       (0x000BU)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__DATA_FRAME_MAX                         (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__PING_TAG_MATCH_MASK                    (0x1000U)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__PING_TAG_MATCH_SHIFT                   (0x000CU)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__PING_TAG_MATCH_MAX                     (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__DATA_TAG_MATCH_MASK                    (0x2000U)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__DATA_TAG_MATCH_SHIFT                   (0x000DU)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__DATA_TAG_MATCH_MAX                     (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__ERROR_TAG_MATCH_MASK                   (0x4000U)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__ERROR_TAG_MATCH_SHIFT                  (0x000EU)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__ERROR_TAG_MATCH_MAX                    (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__RESERVED_1_MASK                        (0x8000U)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__RESERVED_1_SHIFT                       (0x000FU)
#define CSL_FSI_RX_CFG_RX_EVT_STS_ALT1__RESERVED_1_MAX                         (0x0001U)

/* RX_CRC_INFO */

#define CSL_FSI_RX_CFG_RX_CRC_INFO_RX_CRC_MASK                                 (0x00FFU)
#define CSL_FSI_RX_CFG_RX_CRC_INFO_RX_CRC_SHIFT                                (0x0000U)
#define CSL_FSI_RX_CFG_RX_CRC_INFO_RX_CRC_MAX                                  (0x00FFU)

#define CSL_FSI_RX_CFG_RX_CRC_INFO_CALC_CRC_MASK                               (0xFF00U)
#define CSL_FSI_RX_CFG_RX_CRC_INFO_CALC_CRC_SHIFT                              (0x0008U)
#define CSL_FSI_RX_CFG_RX_CRC_INFO_CALC_CRC_MAX                                (0x00FFU)

/* RX_EVT_CLR_ALT1_ */

#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__PING_WD_TO_MASK                        (0x0001U)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__PING_WD_TO_SHIFT                       (0x0000U)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__PING_WD_TO_MAX                         (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__FRAME_WD_TO_MASK                       (0x0002U)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__FRAME_WD_TO_SHIFT                      (0x0001U)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__FRAME_WD_TO_MAX                        (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__CRC_ERR_MASK                           (0x0004U)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__CRC_ERR_SHIFT                          (0x0002U)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__CRC_ERR_MAX                            (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__TYPE_ERR_MASK                          (0x0008U)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__TYPE_ERR_SHIFT                         (0x0003U)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__TYPE_ERR_MAX                           (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__EOF_ERR_MASK                           (0x0010U)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__EOF_ERR_SHIFT                          (0x0004U)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__EOF_ERR_MAX                            (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__BUF_OVERRUN_MASK                       (0x0020U)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__BUF_OVERRUN_SHIFT                      (0x0005U)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__BUF_OVERRUN_MAX                        (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__FRAME_DONE_MASK                        (0x0040U)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__FRAME_DONE_SHIFT                       (0x0006U)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__FRAME_DONE_MAX                         (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__BUF_UNDERRUN_MASK                      (0x0080U)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__BUF_UNDERRUN_SHIFT                     (0x0007U)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__BUF_UNDERRUN_MAX                       (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__ERR_FRAME_MASK                         (0x0100U)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__ERR_FRAME_SHIFT                        (0x0008U)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__ERR_FRAME_MAX                          (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__PING_FRAME_MASK                        (0x0200U)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__PING_FRAME_SHIFT                       (0x0009U)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__PING_FRAME_MAX                         (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__FRAME_OVERRUN_MASK                     (0x0400U)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__FRAME_OVERRUN_SHIFT                    (0x000AU)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__FRAME_OVERRUN_MAX                      (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__DATA_FRAME_MASK                        (0x0800U)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__DATA_FRAME_SHIFT                       (0x000BU)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__DATA_FRAME_MAX                         (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__PING_TAG_MATCH_MASK                    (0x1000U)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__PING_TAG_MATCH_SHIFT                   (0x000CU)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__PING_TAG_MATCH_MAX                     (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__DATA_TAG_MATCH_MASK                    (0x2000U)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__DATA_TAG_MATCH_SHIFT                   (0x000DU)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__DATA_TAG_MATCH_MAX                     (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__ERROR_TAG_MATCH_MASK                   (0x4000U)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__ERROR_TAG_MATCH_SHIFT                  (0x000EU)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__ERROR_TAG_MATCH_MAX                    (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__RESERVED_1_MASK                        (0x8000U)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__RESERVED_1_SHIFT                       (0x000FU)
#define CSL_FSI_RX_CFG_RX_EVT_CLR_ALT1__RESERVED_1_MAX                         (0x0001U)

/* RX_EVT_FRC_ALT1_ */

#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__PING_WD_TO_MASK                        (0x0001U)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__PING_WD_TO_SHIFT                       (0x0000U)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__PING_WD_TO_MAX                         (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__FRAME_WD_TO_MASK                       (0x0002U)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__FRAME_WD_TO_SHIFT                      (0x0001U)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__FRAME_WD_TO_MAX                        (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__CRC_ERR_MASK                           (0x0004U)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__CRC_ERR_SHIFT                          (0x0002U)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__CRC_ERR_MAX                            (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__TYPE_ERR_MASK                          (0x0008U)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__TYPE_ERR_SHIFT                         (0x0003U)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__TYPE_ERR_MAX                           (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__EOF_ERR_MASK                           (0x0010U)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__EOF_ERR_SHIFT                          (0x0004U)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__EOF_ERR_MAX                            (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__BUF_OVERRUN_MASK                       (0x0020U)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__BUF_OVERRUN_SHIFT                      (0x0005U)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__BUF_OVERRUN_MAX                        (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__FRAME_DONE_MASK                        (0x0040U)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__FRAME_DONE_SHIFT                       (0x0006U)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__FRAME_DONE_MAX                         (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__BUF_UNDERRUN_MASK                      (0x0080U)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__BUF_UNDERRUN_SHIFT                     (0x0007U)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__BUF_UNDERRUN_MAX                       (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__ERR_FRAME_MASK                         (0x0100U)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__ERR_FRAME_SHIFT                        (0x0008U)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__ERR_FRAME_MAX                          (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__PING_FRAME_MASK                        (0x0200U)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__PING_FRAME_SHIFT                       (0x0009U)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__PING_FRAME_MAX                         (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__FRAME_OVERRUN_MASK                     (0x0400U)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__FRAME_OVERRUN_SHIFT                    (0x000AU)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__FRAME_OVERRUN_MAX                      (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__DATA_FRAME_MASK                        (0x0800U)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__DATA_FRAME_SHIFT                       (0x000BU)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__DATA_FRAME_MAX                         (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__PING_TAG_MATCH_MASK                    (0x1000U)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__PING_TAG_MATCH_SHIFT                   (0x000CU)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__PING_TAG_MATCH_MAX                     (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__DATA_TAG_MATCH_MASK                    (0x2000U)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__DATA_TAG_MATCH_SHIFT                   (0x000DU)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__DATA_TAG_MATCH_MAX                     (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__ERROR_TAG_MATCH_MASK                   (0x4000U)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__ERROR_TAG_MATCH_SHIFT                  (0x000EU)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__ERROR_TAG_MATCH_MAX                    (0x0001U)

#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__RESERVED_1_MASK                        (0x8000U)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__RESERVED_1_SHIFT                       (0x000FU)
#define CSL_FSI_RX_CFG_RX_EVT_FRC_ALT1__RESERVED_1_MAX                         (0x0001U)

/* RX_BUF_PTR_LOAD */

#define CSL_FSI_RX_CFG_RX_BUF_PTR_LOAD_BUF_PTR_LOAD_MASK                       (0x000FU)
#define CSL_FSI_RX_CFG_RX_BUF_PTR_LOAD_BUF_PTR_LOAD_SHIFT                      (0x0000U)
#define CSL_FSI_RX_CFG_RX_BUF_PTR_LOAD_BUF_PTR_LOAD_MAX                        (0x000FU)

#define CSL_FSI_RX_CFG_RX_BUF_PTR_LOAD_RESERVED_1_MASK                         (0xFFF0U)
#define CSL_FSI_RX_CFG_RX_BUF_PTR_LOAD_RESERVED_1_SHIFT                        (0x0004U)
#define CSL_FSI_RX_CFG_RX_BUF_PTR_LOAD_RESERVED_1_MAX                          (0x0FFFU)

/* RX_BUF_PTR_STS */

#define CSL_FSI_RX_CFG_RX_BUF_PTR_STS_CURR_BUF_PTR_MASK                        (0x000FU)
#define CSL_FSI_RX_CFG_RX_BUF_PTR_STS_CURR_BUF_PTR_SHIFT                       (0x0000U)
#define CSL_FSI_RX_CFG_RX_BUF_PTR_STS_CURR_BUF_PTR_MAX                         (0x000FU)

#define CSL_FSI_RX_CFG_RX_BUF_PTR_STS_RESERVED_1_MASK                          (0x00F0U)
#define CSL_FSI_RX_CFG_RX_BUF_PTR_STS_RESERVED_1_SHIFT                         (0x0004U)
#define CSL_FSI_RX_CFG_RX_BUF_PTR_STS_RESERVED_1_MAX                           (0x000FU)

#define CSL_FSI_RX_CFG_RX_BUF_PTR_STS_CURR_WORD_CNT_MASK                       (0x1F00U)
#define CSL_FSI_RX_CFG_RX_BUF_PTR_STS_CURR_WORD_CNT_SHIFT                      (0x0008U)
#define CSL_FSI_RX_CFG_RX_BUF_PTR_STS_CURR_WORD_CNT_MAX                        (0x001FU)

#define CSL_FSI_RX_CFG_RX_BUF_PTR_STS_RESERVED_2_MASK                          (0xE000U)
#define CSL_FSI_RX_CFG_RX_BUF_PTR_STS_RESERVED_2_SHIFT                         (0x000DU)
#define CSL_FSI_RX_CFG_RX_BUF_PTR_STS_RESERVED_2_MAX                           (0x0007U)

/* RX_FRAME_WD_CTRL */

#define CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL_FRAME_WD_CNT_RST_MASK                  (0x0001U)
#define CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL_FRAME_WD_CNT_RST_SHIFT                 (0x0000U)
#define CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL_FRAME_WD_CNT_RST_MAX                   (0x0001U)

#define CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL_FRAME_WD_EN_MASK                       (0x0002U)
#define CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL_FRAME_WD_EN_SHIFT                      (0x0001U)
#define CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL_FRAME_WD_EN_MAX                        (0x0001U)

#define CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL_RESERVED_1_MASK                        (0xFFFCU)
#define CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL_RESERVED_1_SHIFT                       (0x0002U)
#define CSL_FSI_RX_CFG_RX_FRAME_WD_CTRL_RESERVED_1_MAX                         (0x3FFFU)

/* RESERVED_6 */

/* RX_FRAME_WD_REF */

#define CSL_FSI_RX_CFG_RX_FRAME_WD_REF_FRAME_WD_REF_MASK                       (0xFFFFFFFFU)
#define CSL_FSI_RX_CFG_RX_FRAME_WD_REF_FRAME_WD_REF_SHIFT                      (0x00000000U)
#define CSL_FSI_RX_CFG_RX_FRAME_WD_REF_FRAME_WD_REF_MAX                        (0xFFFFFFFFU)

/* RX_FRAME_WD_CNT */

#define CSL_FSI_RX_CFG_RX_FRAME_WD_CNT_FRAME_WD_CNT_MASK                       (0xFFFFFFFFU)
#define CSL_FSI_RX_CFG_RX_FRAME_WD_CNT_FRAME_WD_CNT_SHIFT                      (0x00000000U)
#define CSL_FSI_RX_CFG_RX_FRAME_WD_CNT_FRAME_WD_CNT_MAX                        (0xFFFFFFFFU)

/* RX_PING_WD_CTRL */

#define CSL_FSI_RX_CFG_RX_PING_WD_CTRL_PING_WD_RST_MASK                        (0x0001U)
#define CSL_FSI_RX_CFG_RX_PING_WD_CTRL_PING_WD_RST_SHIFT                       (0x0000U)
#define CSL_FSI_RX_CFG_RX_PING_WD_CTRL_PING_WD_RST_MAX                         (0x0001U)

#define CSL_FSI_RX_CFG_RX_PING_WD_CTRL_PING_WD_EN_MASK                         (0x0002U)
#define CSL_FSI_RX_CFG_RX_PING_WD_CTRL_PING_WD_EN_SHIFT                        (0x0001U)
#define CSL_FSI_RX_CFG_RX_PING_WD_CTRL_PING_WD_EN_MAX                          (0x0001U)

#define CSL_FSI_RX_CFG_RX_PING_WD_CTRL_RESERVED_1_MASK                         (0xFFFCU)
#define CSL_FSI_RX_CFG_RX_PING_WD_CTRL_RESERVED_1_SHIFT                        (0x0002U)
#define CSL_FSI_RX_CFG_RX_PING_WD_CTRL_RESERVED_1_MAX                          (0x3FFFU)

/* RX_PING_TAG */

#define CSL_FSI_RX_CFG_RX_PING_TAG_ZERO_MASK                                   (0x0001U)
#define CSL_FSI_RX_CFG_RX_PING_TAG_ZERO_SHIFT                                  (0x0000U)
#define CSL_FSI_RX_CFG_RX_PING_TAG_ZERO_MAX                                    (0x0001U)

#define CSL_FSI_RX_CFG_RX_PING_TAG_PING_TAG_MASK                               (0x001EU)
#define CSL_FSI_RX_CFG_RX_PING_TAG_PING_TAG_SHIFT                              (0x0001U)
#define CSL_FSI_RX_CFG_RX_PING_TAG_PING_TAG_MAX                                (0x000FU)

#define CSL_FSI_RX_CFG_RX_PING_TAG_RESERVED_1_MASK                             (0xFFE0U)
#define CSL_FSI_RX_CFG_RX_PING_TAG_RESERVED_1_SHIFT                            (0x0005U)
#define CSL_FSI_RX_CFG_RX_PING_TAG_RESERVED_1_MAX                              (0x07FFU)

/* RX_PING_WD_REF */

#define CSL_FSI_RX_CFG_RX_PING_WD_REF_PING_WD_REF_MASK                         (0xFFFFFFFFU)
#define CSL_FSI_RX_CFG_RX_PING_WD_REF_PING_WD_REF_SHIFT                        (0x00000000U)
#define CSL_FSI_RX_CFG_RX_PING_WD_REF_PING_WD_REF_MAX                          (0xFFFFFFFFU)

/* RX_PING_WD_CNT */

#define CSL_FSI_RX_CFG_RX_PING_WD_CNT_PING_WD_CNT_MASK                         (0xFFFFFFFFU)
#define CSL_FSI_RX_CFG_RX_PING_WD_CNT_PING_WD_CNT_SHIFT                        (0x00000000U)
#define CSL_FSI_RX_CFG_RX_PING_WD_CNT_PING_WD_CNT_MAX                          (0xFFFFFFFFU)

/* RX_INT1_CTRL_ALT1_ */

#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_PING_WD_TO_MASK              (0x0001U)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_PING_WD_TO_SHIFT             (0x0000U)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_PING_WD_TO_MAX               (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_FRAME_WD_TO_MASK             (0x0002U)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_FRAME_WD_TO_SHIFT            (0x0001U)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_FRAME_WD_TO_MAX              (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_CRC_ERR_MASK                 (0x0004U)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_CRC_ERR_SHIFT                (0x0002U)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_CRC_ERR_MAX                  (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_TYPE_ERR_MASK                (0x0008U)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_TYPE_ERR_SHIFT               (0x0003U)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_TYPE_ERR_MAX                 (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_EOF_ERR_MASK                 (0x0010U)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_EOF_ERR_SHIFT                (0x0004U)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_EOF_ERR_MAX                  (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_OVERRUN_MASK                 (0x0020U)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_OVERRUN_SHIFT                (0x0005U)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_OVERRUN_MAX                  (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_FRAME_DONE_MASK              (0x0040U)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_FRAME_DONE_SHIFT             (0x0006U)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_FRAME_DONE_MAX               (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_UNDERRUN_MASK                (0x0080U)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_UNDERRUN_SHIFT               (0x0007U)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_UNDERRUN_MAX                 (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_ERR_FRAME_MASK               (0x0100U)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_ERR_FRAME_SHIFT              (0x0008U)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_ERR_FRAME_MAX                (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_PING_FRAME_MASK              (0x0200U)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_PING_FRAME_SHIFT             (0x0009U)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_PING_FRAME_MAX               (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_FRAME_OVERRUN_MASK           (0x0400U)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_FRAME_OVERRUN_SHIFT          (0x000AU)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_FRAME_OVERRUN_MAX            (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_DATA_FRAME_MASK              (0x0800U)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_DATA_FRAME_SHIFT             (0x000BU)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_DATA_FRAME_MAX               (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_PING_TAG_MATCH_MASK          (0x1000U)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_PING_TAG_MATCH_SHIFT         (0x000CU)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_PING_TAG_MATCH_MAX           (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_DATA_TAG_MATCH_MASK          (0x2000U)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_DATA_TAG_MATCH_SHIFT         (0x000DU)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_DATA_TAG_MATCH_MAX           (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_ERROR_TAG_MATCH_MASK         (0x4000U)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_ERROR_TAG_MATCH_SHIFT        (0x000EU)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__INT1_EN_ERROR_TAG_MATCH_MAX          (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__RESERVED_1_MASK                      (0x8000U)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__RESERVED_1_SHIFT                     (0x000FU)
#define CSL_FSI_RX_CFG_RX_INT1_CTRL_ALT1__RESERVED_1_MAX                       (0x0001U)

/* RX_INT2_CTRL_ALT1_ */

#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_PING_WD_TO_MASK              (0x0001U)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_PING_WD_TO_SHIFT             (0x0000U)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_PING_WD_TO_MAX               (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_FRAME_WD_TO_MASK             (0x0002U)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_FRAME_WD_TO_SHIFT            (0x0001U)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_FRAME_WD_TO_MAX              (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_CRC_ERR_MASK                 (0x0004U)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_CRC_ERR_SHIFT                (0x0002U)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_CRC_ERR_MAX                  (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_TYPE_ERR_MASK                (0x0008U)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_TYPE_ERR_SHIFT               (0x0003U)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_TYPE_ERR_MAX                 (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_EOF_ERR_MASK                 (0x0010U)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_EOF_ERR_SHIFT                (0x0004U)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_EOF_ERR_MAX                  (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_OVERRUN_MASK                 (0x0020U)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_OVERRUN_SHIFT                (0x0005U)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_OVERRUN_MAX                  (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_FRAME_DONE_MASK              (0x0040U)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_FRAME_DONE_SHIFT             (0x0006U)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_FRAME_DONE_MAX               (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_UNDERRUN_MASK                (0x0080U)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_UNDERRUN_SHIFT               (0x0007U)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_UNDERRUN_MAX                 (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_ERR_FRAME_MASK               (0x0100U)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_ERR_FRAME_SHIFT              (0x0008U)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_ERR_FRAME_MAX                (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_PING_FRAME_MASK              (0x0200U)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_PING_FRAME_SHIFT             (0x0009U)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_PING_FRAME_MAX               (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_FRAME_OVERRUN_MASK           (0x0400U)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_FRAME_OVERRUN_SHIFT          (0x000AU)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_FRAME_OVERRUN_MAX            (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_DATA_FRAME_MASK              (0x0800U)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_DATA_FRAME_SHIFT             (0x000BU)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_DATA_FRAME_MAX               (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_PING_TAG_MATCH_MASK          (0x1000U)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_PING_TAG_MATCH_SHIFT         (0x000CU)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_PING_TAG_MATCH_MAX           (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_DATA_TAG_MATCH_MASK          (0x2000U)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_DATA_TAG_MATCH_SHIFT         (0x000DU)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_DATA_TAG_MATCH_MAX           (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_ERROR_TAG_MATCH_MASK         (0x4000U)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_ERROR_TAG_MATCH_SHIFT        (0x000EU)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__INT2_EN_ERROR_TAG_MATCH_MAX          (0x0001U)

#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__RESERVED_1_MASK                      (0x8000U)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__RESERVED_1_SHIFT                     (0x000FU)
#define CSL_FSI_RX_CFG_RX_INT2_CTRL_ALT1__RESERVED_1_MAX                       (0x0001U)

/* RX_LOCK_CTRL */

#define CSL_FSI_RX_CFG_RX_LOCK_CTRL_LOCK_MASK                                  (0x0001U)
#define CSL_FSI_RX_CFG_RX_LOCK_CTRL_LOCK_SHIFT                                 (0x0000U)
#define CSL_FSI_RX_CFG_RX_LOCK_CTRL_LOCK_MAX                                   (0x0001U)

#define CSL_FSI_RX_CFG_RX_LOCK_CTRL_RESERVED_1_MASK                            (0x00FEU)
#define CSL_FSI_RX_CFG_RX_LOCK_CTRL_RESERVED_1_SHIFT                           (0x0001U)
#define CSL_FSI_RX_CFG_RX_LOCK_CTRL_RESERVED_1_MAX                             (0x007FU)

#define CSL_FSI_RX_CFG_RX_LOCK_CTRL_KEY_MASK                                   (0xFF00U)
#define CSL_FSI_RX_CFG_RX_LOCK_CTRL_KEY_SHIFT                                  (0x0008U)
#define CSL_FSI_RX_CFG_RX_LOCK_CTRL_KEY_MAX                                    (0x00FFU)

/* RESERVED_7 */

/* RX_ECC_DATA */

#define CSL_FSI_RX_CFG_RX_ECC_DATA_DATA_LOW_MASK                               (0x0000FFFFU)
#define CSL_FSI_RX_CFG_RX_ECC_DATA_DATA_LOW_SHIFT                              (0x00000000U)
#define CSL_FSI_RX_CFG_RX_ECC_DATA_DATA_LOW_MAX                                (0x0000FFFFU)

#define CSL_FSI_RX_CFG_RX_ECC_DATA_DATA_HIGH_MASK                              (0xFFFF0000U)
#define CSL_FSI_RX_CFG_RX_ECC_DATA_DATA_HIGH_SHIFT                             (0x00000010U)
#define CSL_FSI_RX_CFG_RX_ECC_DATA_DATA_HIGH_MAX                               (0x0000FFFFU)

/* RX_ECC_VAL */

#define CSL_FSI_RX_CFG_RX_ECC_VAL_ECC_VAL_MASK                                 (0x007FU)
#define CSL_FSI_RX_CFG_RX_ECC_VAL_ECC_VAL_SHIFT                                (0x0000U)
#define CSL_FSI_RX_CFG_RX_ECC_VAL_ECC_VAL_MAX                                  (0x007FU)

#define CSL_FSI_RX_CFG_RX_ECC_VAL_RESERVED_1_MASK                              (0xFF80U)
#define CSL_FSI_RX_CFG_RX_ECC_VAL_RESERVED_1_SHIFT                             (0x0007U)
#define CSL_FSI_RX_CFG_RX_ECC_VAL_RESERVED_1_MAX                               (0x01FFU)

/* RESERVED_8 */

/* RX_ECC_SEC_DATA */

#define CSL_FSI_RX_CFG_RX_ECC_SEC_DATA_SEC_DATA_MASK                           (0xFFFFFFFFU)
#define CSL_FSI_RX_CFG_RX_ECC_SEC_DATA_SEC_DATA_SHIFT                          (0x00000000U)
#define CSL_FSI_RX_CFG_RX_ECC_SEC_DATA_SEC_DATA_MAX                            (0xFFFFFFFFU)

/* RX_ECC_LOG */

#define CSL_FSI_RX_CFG_RX_ECC_LOG_SBE_MASK                                     (0x0001U)
#define CSL_FSI_RX_CFG_RX_ECC_LOG_SBE_SHIFT                                    (0x0000U)
#define CSL_FSI_RX_CFG_RX_ECC_LOG_SBE_MAX                                      (0x0001U)

#define CSL_FSI_RX_CFG_RX_ECC_LOG_MBE_MASK                                     (0x0002U)
#define CSL_FSI_RX_CFG_RX_ECC_LOG_MBE_SHIFT                                    (0x0001U)
#define CSL_FSI_RX_CFG_RX_ECC_LOG_MBE_MAX                                      (0x0001U)

#define CSL_FSI_RX_CFG_RX_ECC_LOG_RESERVED_1_MASK                              (0xFFFCU)
#define CSL_FSI_RX_CFG_RX_ECC_LOG_RESERVED_1_SHIFT                             (0x0002U)
#define CSL_FSI_RX_CFG_RX_ECC_LOG_RESERVED_1_MAX                               (0x3FFFU)

/* RESERVED_9 */

/* RX_FRAME_TAG_CMP */

#define CSL_FSI_RX_CFG_RX_FRAME_TAG_CMP_TAG_REF_MASK                           (0x000FU)
#define CSL_FSI_RX_CFG_RX_FRAME_TAG_CMP_TAG_REF_SHIFT                          (0x0000U)
#define CSL_FSI_RX_CFG_RX_FRAME_TAG_CMP_TAG_REF_MAX                            (0x000FU)

#define CSL_FSI_RX_CFG_RX_FRAME_TAG_CMP_TAG_MASK_MASK                          (0x00F0U)
#define CSL_FSI_RX_CFG_RX_FRAME_TAG_CMP_TAG_MASK_SHIFT                         (0x0004U)
#define CSL_FSI_RX_CFG_RX_FRAME_TAG_CMP_TAG_MASK_MAX                           (0x000FU)

#define CSL_FSI_RX_CFG_RX_FRAME_TAG_CMP_CMP_EN_MASK                            (0x0100U)
#define CSL_FSI_RX_CFG_RX_FRAME_TAG_CMP_CMP_EN_SHIFT                           (0x0008U)
#define CSL_FSI_RX_CFG_RX_FRAME_TAG_CMP_CMP_EN_MAX                             (0x0001U)

#define CSL_FSI_RX_CFG_RX_FRAME_TAG_CMP_BROADCAST_EN_MASK                      (0x0200U)
#define CSL_FSI_RX_CFG_RX_FRAME_TAG_CMP_BROADCAST_EN_SHIFT                     (0x0009U)
#define CSL_FSI_RX_CFG_RX_FRAME_TAG_CMP_BROADCAST_EN_MAX                       (0x0001U)

#define CSL_FSI_RX_CFG_RX_FRAME_TAG_CMP_RESERVED_1_MASK                        (0xFC00U)
#define CSL_FSI_RX_CFG_RX_FRAME_TAG_CMP_RESERVED_1_SHIFT                       (0x000AU)
#define CSL_FSI_RX_CFG_RX_FRAME_TAG_CMP_RESERVED_1_MAX                         (0x003FU)

/* RX_PING_TAG_CMP */

#define CSL_FSI_RX_CFG_RX_PING_TAG_CMP_TAG_REF_MASK                            (0x000FU)
#define CSL_FSI_RX_CFG_RX_PING_TAG_CMP_TAG_REF_SHIFT                           (0x0000U)
#define CSL_FSI_RX_CFG_RX_PING_TAG_CMP_TAG_REF_MAX                             (0x000FU)

#define CSL_FSI_RX_CFG_RX_PING_TAG_CMP_TAG_MASK_MASK                           (0x00F0U)
#define CSL_FSI_RX_CFG_RX_PING_TAG_CMP_TAG_MASK_SHIFT                          (0x0004U)
#define CSL_FSI_RX_CFG_RX_PING_TAG_CMP_TAG_MASK_MAX                            (0x000FU)

#define CSL_FSI_RX_CFG_RX_PING_TAG_CMP_CMP_EN_MASK                             (0x0100U)
#define CSL_FSI_RX_CFG_RX_PING_TAG_CMP_CMP_EN_SHIFT                            (0x0008U)
#define CSL_FSI_RX_CFG_RX_PING_TAG_CMP_CMP_EN_MAX                              (0x0001U)

#define CSL_FSI_RX_CFG_RX_PING_TAG_CMP_BROADCAST_EN_MASK                       (0x0200U)
#define CSL_FSI_RX_CFG_RX_PING_TAG_CMP_BROADCAST_EN_SHIFT                      (0x0009U)
#define CSL_FSI_RX_CFG_RX_PING_TAG_CMP_BROADCAST_EN_MAX                        (0x0001U)

#define CSL_FSI_RX_CFG_RX_PING_TAG_CMP_RESERVED_1_MASK                         (0xFC00U)
#define CSL_FSI_RX_CFG_RX_PING_TAG_CMP_RESERVED_1_SHIFT                        (0x000AU)
#define CSL_FSI_RX_CFG_RX_PING_TAG_CMP_RESERVED_1_MAX                          (0x003FU)

/* RESERVED_10 */

/* RX_TRIG_CTRL0 */

#define CSL_FSI_RX_CFG_RX_TRIG_CTRL0_TRIG_EN_MASK                              (0x00000001U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL0_TRIG_EN_SHIFT                             (0x00000000U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL0_TRIG_EN_MAX                               (0x00000001U)

#define CSL_FSI_RX_CFG_RX_TRIG_CTRL0_TRIG_SEL_MASK                             (0x0000001EU)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL0_TRIG_SEL_SHIFT                            (0x00000001U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL0_TRIG_SEL_MAX                              (0x0000000FU)

#define CSL_FSI_RX_CFG_RX_TRIG_CTRL0_RESERVED_1_MASK                           (0x000000E0U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL0_RESERVED_1_SHIFT                          (0x00000005U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL0_RESERVED_1_MAX                            (0x00000007U)

#define CSL_FSI_RX_CFG_RX_TRIG_CTRL0_RX_TRIG_DLY_MASK                          (0xFFFFFF00U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL0_RX_TRIG_DLY_SHIFT                         (0x00000008U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL0_RX_TRIG_DLY_MAX                           (0x00FFFFFFU)

/* RX_TRIG_WIDTH_0 */

#define CSL_FSI_RX_CFG_RX_TRIG_WIDTH_0_RX_TRIG_WIDTH_MASK                      (0x0000FFFFU)
#define CSL_FSI_RX_CFG_RX_TRIG_WIDTH_0_RX_TRIG_WIDTH_SHIFT                     (0x00000000U)
#define CSL_FSI_RX_CFG_RX_TRIG_WIDTH_0_RX_TRIG_WIDTH_MAX                       (0x0000FFFFU)

#define CSL_FSI_RX_CFG_RX_TRIG_WIDTH_0_RESERVED_1_MASK                         (0xFFFF0000U)
#define CSL_FSI_RX_CFG_RX_TRIG_WIDTH_0_RESERVED_1_SHIFT                        (0x00000010U)
#define CSL_FSI_RX_CFG_RX_TRIG_WIDTH_0_RESERVED_1_MAX                          (0x0000FFFFU)

/* RX_DLYLINE_CTRL */

#define CSL_FSI_RX_CFG_RX_DLYLINE_CTRL_RXCLK_DLY_MASK                          (0x001FU)
#define CSL_FSI_RX_CFG_RX_DLYLINE_CTRL_RXCLK_DLY_SHIFT                         (0x0000U)
#define CSL_FSI_RX_CFG_RX_DLYLINE_CTRL_RXCLK_DLY_MAX                           (0x001FU)

#define CSL_FSI_RX_CFG_RX_DLYLINE_CTRL_RXD0_DLY_MASK                           (0x03E0U)
#define CSL_FSI_RX_CFG_RX_DLYLINE_CTRL_RXD0_DLY_SHIFT                          (0x0005U)
#define CSL_FSI_RX_CFG_RX_DLYLINE_CTRL_RXD0_DLY_MAX                            (0x001FU)

#define CSL_FSI_RX_CFG_RX_DLYLINE_CTRL_RXD1_DLY_MASK                           (0x7C00U)
#define CSL_FSI_RX_CFG_RX_DLYLINE_CTRL_RXD1_DLY_SHIFT                          (0x000AU)
#define CSL_FSI_RX_CFG_RX_DLYLINE_CTRL_RXD1_DLY_MAX                            (0x001FU)

#define CSL_FSI_RX_CFG_RX_DLYLINE_CTRL_RESERVED_1_MASK                         (0x8000U)
#define CSL_FSI_RX_CFG_RX_DLYLINE_CTRL_RESERVED_1_SHIFT                        (0x000FU)
#define CSL_FSI_RX_CFG_RX_DLYLINE_CTRL_RESERVED_1_MAX                          (0x0001U)

/* RX_TRIG_CTRL1 */

#define CSL_FSI_RX_CFG_RX_TRIG_CTRL1_TRIG_EN_MASK                              (0x00000001U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL1_TRIG_EN_SHIFT                             (0x00000000U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL1_TRIG_EN_MAX                               (0x00000001U)

#define CSL_FSI_RX_CFG_RX_TRIG_CTRL1_TRIG_SEL_MASK                             (0x0000001EU)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL1_TRIG_SEL_SHIFT                            (0x00000001U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL1_TRIG_SEL_MAX                              (0x0000000FU)

#define CSL_FSI_RX_CFG_RX_TRIG_CTRL1_RESERVED_1_MASK                           (0x000000E0U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL1_RESERVED_1_SHIFT                          (0x00000005U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL1_RESERVED_1_MAX                            (0x00000007U)

#define CSL_FSI_RX_CFG_RX_TRIG_CTRL1_RX_TRIG_DLY_MASK                          (0xFFFFFF00U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL1_RX_TRIG_DLY_SHIFT                         (0x00000008U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL1_RX_TRIG_DLY_MAX                           (0x00FFFFFFU)

/* RX_TRIG_CTRL2 */

#define CSL_FSI_RX_CFG_RX_TRIG_CTRL2_TRIG_EN_MASK                              (0x00000001U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL2_TRIG_EN_SHIFT                             (0x00000000U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL2_TRIG_EN_MAX                               (0x00000001U)

#define CSL_FSI_RX_CFG_RX_TRIG_CTRL2_TRIG_SEL_MASK                             (0x0000001EU)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL2_TRIG_SEL_SHIFT                            (0x00000001U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL2_TRIG_SEL_MAX                              (0x0000000FU)

#define CSL_FSI_RX_CFG_RX_TRIG_CTRL2_RESERVED_1_MASK                           (0x000000E0U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL2_RESERVED_1_SHIFT                          (0x00000005U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL2_RESERVED_1_MAX                            (0x00000007U)

#define CSL_FSI_RX_CFG_RX_TRIG_CTRL2_RX_TRIG_DLY_MASK                          (0xFFFFFF00U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL2_RX_TRIG_DLY_SHIFT                         (0x00000008U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL2_RX_TRIG_DLY_MAX                           (0x00FFFFFFU)

/* RX_TRIG_CTRL3 */

#define CSL_FSI_RX_CFG_RX_TRIG_CTRL3_TRIG_EN_MASK                              (0x00000001U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL3_TRIG_EN_SHIFT                             (0x00000000U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL3_TRIG_EN_MAX                               (0x00000001U)

#define CSL_FSI_RX_CFG_RX_TRIG_CTRL3_TRIG_SEL_MASK                             (0x0000001EU)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL3_TRIG_SEL_SHIFT                            (0x00000001U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL3_TRIG_SEL_MAX                              (0x0000000FU)

#define CSL_FSI_RX_CFG_RX_TRIG_CTRL3_RESERVED_1_MASK                           (0x000000E0U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL3_RESERVED_1_SHIFT                          (0x00000005U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL3_RESERVED_1_MAX                            (0x00000007U)

#define CSL_FSI_RX_CFG_RX_TRIG_CTRL3_RX_TRIG_DLY_MASK                          (0xFFFFFF00U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL3_RX_TRIG_DLY_SHIFT                         (0x00000008U)
#define CSL_FSI_RX_CFG_RX_TRIG_CTRL3_RX_TRIG_DLY_MAX                           (0x00FFFFFFU)

/* RESERVED_11 */

/* RX_VIS_1 */

#define CSL_FSI_RX_CFG_RX_VIS_1_RESERVED_1_MASK                                (0x00000007U)
#define CSL_FSI_RX_CFG_RX_VIS_1_RESERVED_1_SHIFT                               (0x00000000U)
#define CSL_FSI_RX_CFG_RX_VIS_1_RESERVED_1_MAX                                 (0x00000007U)

#define CSL_FSI_RX_CFG_RX_VIS_1_RX_CORE_STS_MASK                               (0x00000008U)
#define CSL_FSI_RX_CFG_RX_VIS_1_RX_CORE_STS_SHIFT                              (0x00000003U)
#define CSL_FSI_RX_CFG_RX_VIS_1_RX_CORE_STS_MAX                                (0x00000001U)

#define CSL_FSI_RX_CFG_RX_VIS_1_RESERVED_2_MASK                                (0xFFFFFFF0U)
#define CSL_FSI_RX_CFG_RX_VIS_1_RESERVED_2_SHIFT                               (0x00000004U)
#define CSL_FSI_RX_CFG_RX_VIS_1_RESERVED_2_MAX                                 (0x0FFFFFFFU)

/* RX_USER_DATA */

#define CSL_FSI_RX_CFG_RX_USER_DATA_USER_REG_MASK                              (0x00FFU)
#define CSL_FSI_RX_CFG_RX_USER_DATA_USER_REG_SHIFT                             (0x0000U)
#define CSL_FSI_RX_CFG_RX_USER_DATA_USER_REG_MAX                               (0x00FFU)

#define CSL_FSI_RX_CFG_RX_USER_DATA_USER_MASK_MASK                             (0xFF00U)
#define CSL_FSI_RX_CFG_RX_USER_DATA_USER_MASK_SHIFT                            (0x0008U)
#define CSL_FSI_RX_CFG_RX_USER_DATA_USER_MASK_MAX                              (0x00FFU)


/* RESERVED_12 */

/* RX_BUF_BASE */

#define CSL_FSI_RX_CFG_RX_BUF_BASE_BASE_ADDRESS_MASK                           (0xFFFFU)
#define CSL_FSI_RX_CFG_RX_BUF_BASE_BASE_ADDRESS_SHIFT                          (0x0000U)
#define CSL_FSI_RX_CFG_RX_BUF_BASE_BASE_ADDRESS_MAX                            (0xFFFFU)

#ifdef __cplusplus
}
#endif
#endif
