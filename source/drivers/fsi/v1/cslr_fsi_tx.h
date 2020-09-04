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

#ifndef CSLR_FSI_TX_H_
#define CSLR_FSI_TX_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/**************************************************************************
* Module Base Offset Values
**************************************************************************/

#define CSL_FSI_TX_CFG_REGS_BASE                                               (0x00000000U)


/**************************************************************************
* Hardware Region  : FSI Registers
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint16_t TX_MASTER_CTRL;
    volatile uint16_t RESERVED_1;
    volatile uint16_t TX_CLK_CTRL;
    volatile uint16_t RESERVED_2;
    volatile uint16_t TX_OPER_CTRL_LO_ALT1_;
    volatile uint16_t TX_OPER_CTRL_HI_ALT1_;
    volatile uint16_t TX_FRAME_CTRL;
    volatile uint16_t TX_FRAME_TAG_UDATA;
    volatile uint16_t TX_BUF_PTR_LOAD;
    volatile uint16_t TX_BUF_PTR_STS;
    volatile uint16_t TX_PING_CTRL_ALT1_;
    volatile uint16_t TX_PING_TAG;
    volatile uint32_t TX_PING_TO_REF;
    volatile uint32_t TX_PING_TO_CNT;
    volatile uint16_t TX_INT_CTRL;
    volatile uint16_t TX_DMA_CTRL;
    volatile uint16_t TX_LOCK_CTRL;
    volatile uint16_t RESERVED_3;
    volatile uint16_t TX_EVT_STS;
    volatile uint16_t RESERVED_4;
    volatile uint16_t TX_EVT_CLR;
    volatile uint16_t TX_EVT_FRC;
    volatile uint16_t TX_USER_CRC;
    volatile uint16_t RESERVED_5;
    volatile uint32_t RESERVED_6;
    volatile uint32_t RESERVED_7;
    volatile uint32_t RESERVED_8;
    volatile uint32_t TX_ECC_DATA;
    volatile uint16_t TX_ECC_VAL;
    volatile uint16_t RESERVED_9;
    volatile uint16_t TX_DLYLINE_CTRL;
    volatile uint16_t RESERVED_10[24];
    volatile uint16_t TX_BUF_BASE[16];
} CSL_fsi_tx_cfgRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_FSI_TX_CFG_TX_MASTER_CTRL                                          (0x00000000U)
#define CSL_FSI_TX_CFG_RESERVED_1                                              (0x00000002U)
#define CSL_FSI_TX_CFG_TX_CLK_CTRL                                             (0x00000004U)
#define CSL_FSI_TX_CFG_RESERVED_2                                              (0x00000006U)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1_                                   (0x00000008U)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1_                                   (0x0000000AU)
#define CSL_FSI_TX_CFG_TX_FRAME_CTRL                                           (0x0000000CU)
#define CSL_FSI_TX_CFG_TX_FRAME_TAG_UDATA                                      (0x0000000EU)
#define CSL_FSI_TX_CFG_TX_BUF_PTR_LOAD                                         (0x00000010U)
#define CSL_FSI_TX_CFG_TX_BUF_PTR_STS                                          (0x00000012U)
#define CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1_                                      (0x00000014U)
#define CSL_FSI_TX_CFG_TX_PING_TAG                                             (0x00000016U)
#define CSL_FSI_TX_CFG_TX_PING_TO_REF                                          (0x00000018U)
#define CSL_FSI_TX_CFG_TX_PING_TO_CNT                                          (0x0000001CU)
#define CSL_FSI_TX_CFG_TX_INT_CTRL                                             (0x00000020U)
#define CSL_FSI_TX_CFG_TX_DMA_CTRL                                             (0x00000022U)
#define CSL_FSI_TX_CFG_TX_LOCK_CTRL                                            (0x00000024U)
#define CSL_FSI_TX_CFG_RESERVED_3                                              (0x00000026U)
#define CSL_FSI_TX_CFG_TX_EVT_STS                                              (0x00000028U)
#define CSL_FSI_TX_CFG_RESERVED_4                                              (0x0000002AU)
#define CSL_FSI_TX_CFG_TX_EVT_CLR                                              (0x0000002CU)
#define CSL_FSI_TX_CFG_TX_EVT_FRC                                              (0x0000002EU)
#define CSL_FSI_TX_CFG_TX_USER_CRC                                             (0x00000030U)
#define CSL_FSI_TX_CFG_RESERVED_5                                              (0x00000032U)
#define CSL_FSI_TX_CFG_RESERVED_6                                              (0x00000034U)
#define CSL_FSI_TX_CFG_RESERVED_7                                              (0x00000038U)
#define CSL_FSI_TX_CFG_RESERVED_8                                              (0x0000003CU)
#define CSL_FSI_TX_CFG_TX_ECC_DATA                                             (0x00000040U)
#define CSL_FSI_TX_CFG_TX_ECC_VAL                                              (0x00000044U)
#define CSL_FSI_TX_CFG_RESERVED_9                                              (0x00000046U)
#define CSL_FSI_TX_CFG_TX_DLYLINE_CTRL                                         (0x00000048U)
#define CSL_FSI_TX_CFG_RESERVED_10(RESERVED_10)                                (0x00000050U+((RESERVED_10)*0x2U))
#define CSL_FSI_TX_CFG_TX_BUF_BASE(TX_BUF_BASE)                                (0x00000080U+((TX_BUF_BASE)*0x2U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* TX_MASTER_CTRL */

#define CSL_FSI_TX_CFG_TX_MASTER_CTRL_CORE_RST_MASK                            (0x0001U)
#define CSL_FSI_TX_CFG_TX_MASTER_CTRL_CORE_RST_SHIFT                           (0x0000U)
#define CSL_FSI_TX_CFG_TX_MASTER_CTRL_CORE_RST_MAX                             (0x0001U)

#define CSL_FSI_TX_CFG_TX_MASTER_CTRL_FLUSH_MASK                               (0x0002U)
#define CSL_FSI_TX_CFG_TX_MASTER_CTRL_FLUSH_SHIFT                              (0x0001U)
#define CSL_FSI_TX_CFG_TX_MASTER_CTRL_FLUSH_MAX                                (0x0001U)

#define CSL_FSI_TX_CFG_TX_MASTER_CTRL_RESERVED_1_MASK                          (0x00FCU)
#define CSL_FSI_TX_CFG_TX_MASTER_CTRL_RESERVED_1_SHIFT                         (0x0002U)
#define CSL_FSI_TX_CFG_TX_MASTER_CTRL_RESERVED_1_MAX                           (0x003FU)

#define CSL_FSI_TX_CFG_TX_MASTER_CTRL_KEY_MASK                                 (0xFF00U)
#define CSL_FSI_TX_CFG_TX_MASTER_CTRL_KEY_SHIFT                                (0x0008U)
#define CSL_FSI_TX_CFG_TX_MASTER_CTRL_KEY_MAX                                  (0x00FFU)

/* RESERVED_1 */

/* TX_CLK_CTRL */

#define CSL_FSI_TX_CFG_TX_CLK_CTRL_CLK_RST_MASK                                (0x0001U)
#define CSL_FSI_TX_CFG_TX_CLK_CTRL_CLK_RST_SHIFT                               (0x0000U)
#define CSL_FSI_TX_CFG_TX_CLK_CTRL_CLK_RST_MAX                                 (0x0001U)

#define CSL_FSI_TX_CFG_TX_CLK_CTRL_CLK_EN_MASK                                 (0x0002U)
#define CSL_FSI_TX_CFG_TX_CLK_CTRL_CLK_EN_SHIFT                                (0x0001U)
#define CSL_FSI_TX_CFG_TX_CLK_CTRL_CLK_EN_MAX                                  (0x0001U)

#define CSL_FSI_TX_CFG_TX_CLK_CTRL_PRESCALE_VAL_MASK                           (0x03FCU)
#define CSL_FSI_TX_CFG_TX_CLK_CTRL_PRESCALE_VAL_SHIFT                          (0x0002U)
#define CSL_FSI_TX_CFG_TX_CLK_CTRL_PRESCALE_VAL_MAX                            (0x00FFU)

#define CSL_FSI_TX_CFG_TX_CLK_CTRL_RESERVED_1_MASK                             (0xFC00U)
#define CSL_FSI_TX_CFG_TX_CLK_CTRL_RESERVED_1_SHIFT                            (0x000AU)
#define CSL_FSI_TX_CFG_TX_CLK_CTRL_RESERVED_1_MAX                              (0x003FU)

/* RESERVED_2 */

/* TX_OPER_CTRL_LO_ALT1_ */

#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__DATA_WIDTH_MASK                   (0x0003U)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__DATA_WIDTH_SHIFT                  (0x0000U)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__DATA_WIDTH_MAX                    (0x0003U)

#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__SPI_MODE_MASK                     (0x0004U)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__SPI_MODE_SHIFT                    (0x0002U)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__SPI_MODE_MAX                      (0x0001U)

#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__START_MODE_MASK                   (0x0038U)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__START_MODE_SHIFT                  (0x0003U)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__START_MODE_MAX                    (0x0007U)

#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__SW_CRC_MASK                       (0x0040U)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__SW_CRC_SHIFT                      (0x0006U)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__SW_CRC_MAX                        (0x0001U)

#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__PING_TO_MODE_MASK                 (0x0080U)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__PING_TO_MODE_SHIFT                (0x0007U)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__PING_TO_MODE_MAX                  (0x0001U)

#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__SEL_PLLCLK_MASK                   (0x0100U)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__SEL_PLLCLK_SHIFT                  (0x0008U)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__SEL_PLLCLK_MAX                    (0x0001U)

#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__TDM_ENABLE_MASK                   (0x0200U)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__TDM_ENABLE_SHIFT                  (0x0009U)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__TDM_ENABLE_MAX                    (0x0001U)

#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__SEL_TDM_IN_MASK                   (0x0400U)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__SEL_TDM_IN_SHIFT                  (0x000AU)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__SEL_TDM_IN_MAX                    (0x0001U)

#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__RESERVED_1_MASK                   (0xF800U)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__RESERVED_1_SHIFT                  (0x000BU)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_LO_ALT1__RESERVED_1_MAX                    (0x001FU)

/* TX_OPER_CTRL_HI_ALT1_ */

#define CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1__RESERVED_1_MASK                   (0x001FU)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1__RESERVED_1_SHIFT                  (0x0000U)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1__RESERVED_1_MAX                    (0x001FU)

#define CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1__FORCE_ERR_MASK                    (0x0020U)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1__FORCE_ERR_SHIFT                   (0x0005U)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1__FORCE_ERR_MAX                     (0x0001U)

#define CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1__ECC_SEL_MASK                      (0x0040U)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1__ECC_SEL_SHIFT                     (0x0006U)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1__ECC_SEL_MAX                       (0x0001U)

#define CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1__EXT_TRIG_SEL_MASK                 (0x1F80U)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1__EXT_TRIG_SEL_SHIFT                (0x0007U)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1__EXT_TRIG_SEL_MAX                  (0x003FU)

#define CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1__RESERVED_2_MASK                   (0xE000U)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1__RESERVED_2_SHIFT                  (0x000DU)
#define CSL_FSI_TX_CFG_TX_OPER_CTRL_HI_ALT1__RESERVED_2_MAX                    (0x0007U)

/* TX_FRAME_CTRL */

#define CSL_FSI_TX_CFG_TX_FRAME_CTRL_FRAME_TYPE_MASK                           (0x000FU)
#define CSL_FSI_TX_CFG_TX_FRAME_CTRL_FRAME_TYPE_SHIFT                          (0x0000U)
#define CSL_FSI_TX_CFG_TX_FRAME_CTRL_FRAME_TYPE_MAX                            (0x000FU)

#define CSL_FSI_TX_CFG_TX_FRAME_CTRL_N_WORDS_MASK                              (0x00F0U)
#define CSL_FSI_TX_CFG_TX_FRAME_CTRL_N_WORDS_SHIFT                             (0x0004U)
#define CSL_FSI_TX_CFG_TX_FRAME_CTRL_N_WORDS_MAX                               (0x000FU)

#define CSL_FSI_TX_CFG_TX_FRAME_CTRL_RESERVED_1_MASK                           (0x7F00U)
#define CSL_FSI_TX_CFG_TX_FRAME_CTRL_RESERVED_1_SHIFT                          (0x0008U)
#define CSL_FSI_TX_CFG_TX_FRAME_CTRL_RESERVED_1_MAX                            (0x007FU)

#define CSL_FSI_TX_CFG_TX_FRAME_CTRL_START_MASK                                (0x8000U)
#define CSL_FSI_TX_CFG_TX_FRAME_CTRL_START_SHIFT                               (0x000FU)
#define CSL_FSI_TX_CFG_TX_FRAME_CTRL_START_MAX                                 (0x0001U)

/* TX_FRAME_TAG_UDATA */

#define CSL_FSI_TX_CFG_TX_FRAME_TAG_UDATA_FRAME_TAG_MASK                       (0x000FU)
#define CSL_FSI_TX_CFG_TX_FRAME_TAG_UDATA_FRAME_TAG_SHIFT                      (0x0000U)
#define CSL_FSI_TX_CFG_TX_FRAME_TAG_UDATA_FRAME_TAG_MAX                        (0x000FU)

#define CSL_FSI_TX_CFG_TX_FRAME_TAG_UDATA_RESERVED_1_MASK                      (0x00F0U)
#define CSL_FSI_TX_CFG_TX_FRAME_TAG_UDATA_RESERVED_1_SHIFT                     (0x0004U)
#define CSL_FSI_TX_CFG_TX_FRAME_TAG_UDATA_RESERVED_1_MAX                       (0x000FU)

#define CSL_FSI_TX_CFG_TX_FRAME_TAG_UDATA_USER_DATA_MASK                       (0xFF00U)
#define CSL_FSI_TX_CFG_TX_FRAME_TAG_UDATA_USER_DATA_SHIFT                      (0x0008U)
#define CSL_FSI_TX_CFG_TX_FRAME_TAG_UDATA_USER_DATA_MAX                        (0x00FFU)

/* TX_BUF_PTR_LOAD */

#define CSL_FSI_TX_CFG_TX_BUF_PTR_LOAD_BUF_PTR_LOAD_MASK                       (0x000FU)
#define CSL_FSI_TX_CFG_TX_BUF_PTR_LOAD_BUF_PTR_LOAD_SHIFT                      (0x0000U)
#define CSL_FSI_TX_CFG_TX_BUF_PTR_LOAD_BUF_PTR_LOAD_MAX                        (0x000FU)

#define CSL_FSI_TX_CFG_TX_BUF_PTR_LOAD_RESERVED_1_MASK                         (0xFFF0U)
#define CSL_FSI_TX_CFG_TX_BUF_PTR_LOAD_RESERVED_1_SHIFT                        (0x0004U)
#define CSL_FSI_TX_CFG_TX_BUF_PTR_LOAD_RESERVED_1_MAX                          (0x0FFFU)

/* TX_BUF_PTR_STS */

#define CSL_FSI_TX_CFG_TX_BUF_PTR_STS_CURR_BUF_PTR_MASK                        (0x000FU)
#define CSL_FSI_TX_CFG_TX_BUF_PTR_STS_CURR_BUF_PTR_SHIFT                       (0x0000U)
#define CSL_FSI_TX_CFG_TX_BUF_PTR_STS_CURR_BUF_PTR_MAX                         (0x000FU)

#define CSL_FSI_TX_CFG_TX_BUF_PTR_STS_RESERVED_1_MASK                          (0x00F0U)
#define CSL_FSI_TX_CFG_TX_BUF_PTR_STS_RESERVED_1_SHIFT                         (0x0004U)
#define CSL_FSI_TX_CFG_TX_BUF_PTR_STS_RESERVED_1_MAX                           (0x000FU)

#define CSL_FSI_TX_CFG_TX_BUF_PTR_STS_CURR_WORD_CNT_MASK                       (0x1F00U)
#define CSL_FSI_TX_CFG_TX_BUF_PTR_STS_CURR_WORD_CNT_SHIFT                      (0x0008U)
#define CSL_FSI_TX_CFG_TX_BUF_PTR_STS_CURR_WORD_CNT_MAX                        (0x001FU)

#define CSL_FSI_TX_CFG_TX_BUF_PTR_STS_RESERVED_2_MASK                          (0xE000U)
#define CSL_FSI_TX_CFG_TX_BUF_PTR_STS_RESERVED_2_SHIFT                         (0x000DU)
#define CSL_FSI_TX_CFG_TX_BUF_PTR_STS_RESERVED_2_MAX                           (0x0007U)

/* TX_PING_CTRL_ALT1_ */

#define CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1__CNT_RST_MASK                         (0x0001U)
#define CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1__CNT_RST_SHIFT                        (0x0000U)
#define CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1__CNT_RST_MAX                          (0x0001U)

#define CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1__TIMER_EN_MASK                        (0x0002U)
#define CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1__TIMER_EN_SHIFT                       (0x0001U)
#define CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1__TIMER_EN_MAX                         (0x0001U)

#define CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1__EXT_TRIG_EN_MASK                     (0x0004U)
#define CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1__EXT_TRIG_EN_SHIFT                    (0x0002U)
#define CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1__EXT_TRIG_EN_MAX                      (0x0001U)

#define CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1__EXT_TRIG_SEL_MASK                    (0x01F8U)
#define CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1__EXT_TRIG_SEL_SHIFT                   (0x0003U)
#define CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1__EXT_TRIG_SEL_MAX                     (0x003FU)

#define CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1__RESERVED_1_MASK                      (0xFE00U)
#define CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1__RESERVED_1_SHIFT                     (0x0009U)
#define CSL_FSI_TX_CFG_TX_PING_CTRL_ALT1__RESERVED_1_MAX                       (0x007FU)

/* TX_PING_TAG */

#define CSL_FSI_TX_CFG_TX_PING_TAG_TAG_MASK                                    (0x000FU)
#define CSL_FSI_TX_CFG_TX_PING_TAG_TAG_SHIFT                                   (0x0000U)
#define CSL_FSI_TX_CFG_TX_PING_TAG_TAG_MAX                                     (0x000FU)

#define CSL_FSI_TX_CFG_TX_PING_TAG_RESERVED_1_MASK                             (0xFFF0U)
#define CSL_FSI_TX_CFG_TX_PING_TAG_RESERVED_1_SHIFT                            (0x0004U)
#define CSL_FSI_TX_CFG_TX_PING_TAG_RESERVED_1_MAX                              (0x0FFFU)

/* TX_PING_TO_REF */

#define CSL_FSI_TX_CFG_TX_PING_TO_REF_TO_REF_MASK                              (0xFFFFFFFFU)
#define CSL_FSI_TX_CFG_TX_PING_TO_REF_TO_REF_SHIFT                             (0x00000000U)
#define CSL_FSI_TX_CFG_TX_PING_TO_REF_TO_REF_MAX                               (0xFFFFFFFFU)

/* TX_PING_TO_CNT */

#define CSL_FSI_TX_CFG_TX_PING_TO_CNT_TO_CNT_MASK                              (0xFFFFFFFFU)
#define CSL_FSI_TX_CFG_TX_PING_TO_CNT_TO_CNT_SHIFT                             (0x00000000U)
#define CSL_FSI_TX_CFG_TX_PING_TO_CNT_TO_CNT_MAX                               (0xFFFFFFFFU)

/* TX_INT_CTRL */

#define CSL_FSI_TX_CFG_TX_INT_CTRL_INT1_EN_FRAME_DONE_MASK                     (0x0001U)
#define CSL_FSI_TX_CFG_TX_INT_CTRL_INT1_EN_FRAME_DONE_SHIFT                    (0x0000U)
#define CSL_FSI_TX_CFG_TX_INT_CTRL_INT1_EN_FRAME_DONE_MAX                      (0x0001U)

#define CSL_FSI_TX_CFG_TX_INT_CTRL_INT1_EN_BUF_UNDERRUN_MASK                   (0x0002U)
#define CSL_FSI_TX_CFG_TX_INT_CTRL_INT1_EN_BUF_UNDERRUN_SHIFT                  (0x0001U)
#define CSL_FSI_TX_CFG_TX_INT_CTRL_INT1_EN_BUF_UNDERRUN_MAX                    (0x0001U)

#define CSL_FSI_TX_CFG_TX_INT_CTRL_INT1_EN_BUF_OVERRUN_MASK                    (0x0004U)
#define CSL_FSI_TX_CFG_TX_INT_CTRL_INT1_EN_BUF_OVERRUN_SHIFT                   (0x0002U)
#define CSL_FSI_TX_CFG_TX_INT_CTRL_INT1_EN_BUF_OVERRUN_MAX                     (0x0001U)

#define CSL_FSI_TX_CFG_TX_INT_CTRL_INT1_EN_PING_TO_MASK                        (0x0008U)
#define CSL_FSI_TX_CFG_TX_INT_CTRL_INT1_EN_PING_TO_SHIFT                       (0x0003U)
#define CSL_FSI_TX_CFG_TX_INT_CTRL_INT1_EN_PING_TO_MAX                         (0x0001U)

#define CSL_FSI_TX_CFG_TX_INT_CTRL_RESERVED_1_MASK                             (0x00F0U)
#define CSL_FSI_TX_CFG_TX_INT_CTRL_RESERVED_1_SHIFT                            (0x0004U)
#define CSL_FSI_TX_CFG_TX_INT_CTRL_RESERVED_1_MAX                              (0x000FU)

#define CSL_FSI_TX_CFG_TX_INT_CTRL_INT2_EN_FRAME_DONE_MASK                     (0x0100U)
#define CSL_FSI_TX_CFG_TX_INT_CTRL_INT2_EN_FRAME_DONE_SHIFT                    (0x0008U)
#define CSL_FSI_TX_CFG_TX_INT_CTRL_INT2_EN_FRAME_DONE_MAX                      (0x0001U)

#define CSL_FSI_TX_CFG_TX_INT_CTRL_INT2_EN_BUF_UNDERRUN_MASK                   (0x0200U)
#define CSL_FSI_TX_CFG_TX_INT_CTRL_INT2_EN_BUF_UNDERRUN_SHIFT                  (0x0009U)
#define CSL_FSI_TX_CFG_TX_INT_CTRL_INT2_EN_BUF_UNDERRUN_MAX                    (0x0001U)

#define CSL_FSI_TX_CFG_TX_INT_CTRL_INT2_EN_BUF_OVERRUN_MASK                    (0x0400U)
#define CSL_FSI_TX_CFG_TX_INT_CTRL_INT2_EN_BUF_OVERRUN_SHIFT                   (0x000AU)
#define CSL_FSI_TX_CFG_TX_INT_CTRL_INT2_EN_BUF_OVERRUN_MAX                     (0x0001U)

#define CSL_FSI_TX_CFG_TX_INT_CTRL_INT2_EN_PING_TO_MASK                        (0x0800U)
#define CSL_FSI_TX_CFG_TX_INT_CTRL_INT2_EN_PING_TO_SHIFT                       (0x000BU)
#define CSL_FSI_TX_CFG_TX_INT_CTRL_INT2_EN_PING_TO_MAX                         (0x0001U)

#define CSL_FSI_TX_CFG_TX_INT_CTRL_RESERVED_2_MASK                             (0xF000U)
#define CSL_FSI_TX_CFG_TX_INT_CTRL_RESERVED_2_SHIFT                            (0x000CU)
#define CSL_FSI_TX_CFG_TX_INT_CTRL_RESERVED_2_MAX                              (0x000FU)

/* TX_DMA_CTRL */

#define CSL_FSI_TX_CFG_TX_DMA_CTRL_DMA_EVT_EN_MASK                             (0x0001U)
#define CSL_FSI_TX_CFG_TX_DMA_CTRL_DMA_EVT_EN_SHIFT                            (0x0000U)
#define CSL_FSI_TX_CFG_TX_DMA_CTRL_DMA_EVT_EN_MAX                              (0x0001U)

#define CSL_FSI_TX_CFG_TX_DMA_CTRL_RESERVED_1_MASK                             (0xFFFEU)
#define CSL_FSI_TX_CFG_TX_DMA_CTRL_RESERVED_1_SHIFT                            (0x0001U)
#define CSL_FSI_TX_CFG_TX_DMA_CTRL_RESERVED_1_MAX                              (0x7FFFU)

/* TX_LOCK_CTRL */

#define CSL_FSI_TX_CFG_TX_LOCK_CTRL_LOCK_MASK                                  (0x0001U)
#define CSL_FSI_TX_CFG_TX_LOCK_CTRL_LOCK_SHIFT                                 (0x0000U)
#define CSL_FSI_TX_CFG_TX_LOCK_CTRL_LOCK_MAX                                   (0x0001U)

#define CSL_FSI_TX_CFG_TX_LOCK_CTRL_RESERVED_1_MASK                            (0x00FEU)
#define CSL_FSI_TX_CFG_TX_LOCK_CTRL_RESERVED_1_SHIFT                           (0x0001U)
#define CSL_FSI_TX_CFG_TX_LOCK_CTRL_RESERVED_1_MAX                             (0x007FU)

#define CSL_FSI_TX_CFG_TX_LOCK_CTRL_KEY_MASK                                   (0xFF00U)
#define CSL_FSI_TX_CFG_TX_LOCK_CTRL_KEY_SHIFT                                  (0x0008U)
#define CSL_FSI_TX_CFG_TX_LOCK_CTRL_KEY_MAX                                    (0x00FFU)

/* RESERVED_3 */

/* TX_EVT_STS */

#define CSL_FSI_TX_CFG_TX_EVT_STS_FRAME_DONE_MASK                              (0x0001U)
#define CSL_FSI_TX_CFG_TX_EVT_STS_FRAME_DONE_SHIFT                             (0x0000U)
#define CSL_FSI_TX_CFG_TX_EVT_STS_FRAME_DONE_MAX                               (0x0001U)

#define CSL_FSI_TX_CFG_TX_EVT_STS_BUF_UNDERRUN_MASK                            (0x0002U)
#define CSL_FSI_TX_CFG_TX_EVT_STS_BUF_UNDERRUN_SHIFT                           (0x0001U)
#define CSL_FSI_TX_CFG_TX_EVT_STS_BUF_UNDERRUN_MAX                             (0x0001U)

#define CSL_FSI_TX_CFG_TX_EVT_STS_BUF_OVERRUN_MASK                             (0x0004U)
#define CSL_FSI_TX_CFG_TX_EVT_STS_BUF_OVERRUN_SHIFT                            (0x0002U)
#define CSL_FSI_TX_CFG_TX_EVT_STS_BUF_OVERRUN_MAX                              (0x0001U)

#define CSL_FSI_TX_CFG_TX_EVT_STS_PING_TRIGGERED_MASK                          (0x0008U)
#define CSL_FSI_TX_CFG_TX_EVT_STS_PING_TRIGGERED_SHIFT                         (0x0003U)
#define CSL_FSI_TX_CFG_TX_EVT_STS_PING_TRIGGERED_MAX                           (0x0001U)

#define CSL_FSI_TX_CFG_TX_EVT_STS_RESERVED_1_MASK                              (0xFFF0U)
#define CSL_FSI_TX_CFG_TX_EVT_STS_RESERVED_1_SHIFT                             (0x0004U)
#define CSL_FSI_TX_CFG_TX_EVT_STS_RESERVED_1_MAX                               (0x0FFFU)

/* RESERVED_4 */

/* TX_EVT_CLR */

#define CSL_FSI_TX_CFG_TX_EVT_CLR_FRAME_DONE_MASK                              (0x0001U)
#define CSL_FSI_TX_CFG_TX_EVT_CLR_FRAME_DONE_SHIFT                             (0x0000U)
#define CSL_FSI_TX_CFG_TX_EVT_CLR_FRAME_DONE_MAX                               (0x0001U)

#define CSL_FSI_TX_CFG_TX_EVT_CLR_BUF_UNDERRUN_MASK                            (0x0002U)
#define CSL_FSI_TX_CFG_TX_EVT_CLR_BUF_UNDERRUN_SHIFT                           (0x0001U)
#define CSL_FSI_TX_CFG_TX_EVT_CLR_BUF_UNDERRUN_MAX                             (0x0001U)

#define CSL_FSI_TX_CFG_TX_EVT_CLR_BUF_OVERRUN_MASK                             (0x0004U)
#define CSL_FSI_TX_CFG_TX_EVT_CLR_BUF_OVERRUN_SHIFT                            (0x0002U)
#define CSL_FSI_TX_CFG_TX_EVT_CLR_BUF_OVERRUN_MAX                              (0x0001U)

#define CSL_FSI_TX_CFG_TX_EVT_CLR_PING_TRIGGERED_MASK                          (0x0008U)
#define CSL_FSI_TX_CFG_TX_EVT_CLR_PING_TRIGGERED_SHIFT                         (0x0003U)
#define CSL_FSI_TX_CFG_TX_EVT_CLR_PING_TRIGGERED_MAX                           (0x0001U)

#define CSL_FSI_TX_CFG_TX_EVT_CLR_RESERVED_1_MASK                              (0xFFF0U)
#define CSL_FSI_TX_CFG_TX_EVT_CLR_RESERVED_1_SHIFT                             (0x0004U)
#define CSL_FSI_TX_CFG_TX_EVT_CLR_RESERVED_1_MAX                               (0x0FFFU)

/* TX_EVT_FRC */

#define CSL_FSI_TX_CFG_TX_EVT_FRC_FRAME_DONE_MASK                              (0x0001U)
#define CSL_FSI_TX_CFG_TX_EVT_FRC_FRAME_DONE_SHIFT                             (0x0000U)
#define CSL_FSI_TX_CFG_TX_EVT_FRC_FRAME_DONE_MAX                               (0x0001U)

#define CSL_FSI_TX_CFG_TX_EVT_FRC_BUF_UNDERRUN_MASK                            (0x0002U)
#define CSL_FSI_TX_CFG_TX_EVT_FRC_BUF_UNDERRUN_SHIFT                           (0x0001U)
#define CSL_FSI_TX_CFG_TX_EVT_FRC_BUF_UNDERRUN_MAX                             (0x0001U)

#define CSL_FSI_TX_CFG_TX_EVT_FRC_BUF_OVERRUN_MASK                             (0x0004U)
#define CSL_FSI_TX_CFG_TX_EVT_FRC_BUF_OVERRUN_SHIFT                            (0x0002U)
#define CSL_FSI_TX_CFG_TX_EVT_FRC_BUF_OVERRUN_MAX                              (0x0001U)

#define CSL_FSI_TX_CFG_TX_EVT_FRC_PING_TRIGGERED_MASK                          (0x0008U)
#define CSL_FSI_TX_CFG_TX_EVT_FRC_PING_TRIGGERED_SHIFT                         (0x0003U)
#define CSL_FSI_TX_CFG_TX_EVT_FRC_PING_TRIGGERED_MAX                           (0x0001U)

#define CSL_FSI_TX_CFG_TX_EVT_FRC_RESERVED_1_MASK                              (0xFFF0U)
#define CSL_FSI_TX_CFG_TX_EVT_FRC_RESERVED_1_SHIFT                             (0x0004U)
#define CSL_FSI_TX_CFG_TX_EVT_FRC_RESERVED_1_MAX                               (0x0FFFU)

/* TX_USER_CRC */

#define CSL_FSI_TX_CFG_TX_USER_CRC_USER_CRC_MASK                               (0x00FFU)
#define CSL_FSI_TX_CFG_TX_USER_CRC_USER_CRC_SHIFT                              (0x0000U)
#define CSL_FSI_TX_CFG_TX_USER_CRC_USER_CRC_MAX                                (0x00FFU)

#define CSL_FSI_TX_CFG_TX_USER_CRC_RESERVED_1_MASK                             (0xFF00U)
#define CSL_FSI_TX_CFG_TX_USER_CRC_RESERVED_1_SHIFT                            (0x0008U)
#define CSL_FSI_TX_CFG_TX_USER_CRC_RESERVED_1_MAX                              (0x00FFU)

/* RESERVED_5 */

/* RESERVED_6 */

/* RESERVED_7 */

/* RESERVED_8 */

/* TX_ECC_DATA */

#define CSL_FSI_TX_CFG_TX_ECC_DATA_DATA_LOW_MASK                               (0x0000FFFFU)
#define CSL_FSI_TX_CFG_TX_ECC_DATA_DATA_LOW_SHIFT                              (0x00000000U)
#define CSL_FSI_TX_CFG_TX_ECC_DATA_DATA_LOW_MAX                                (0x0000FFFFU)

#define CSL_FSI_TX_CFG_TX_ECC_DATA_DATA_HIGH_MASK                              (0xFFFF0000U)
#define CSL_FSI_TX_CFG_TX_ECC_DATA_DATA_HIGH_SHIFT                             (0x00000010U)
#define CSL_FSI_TX_CFG_TX_ECC_DATA_DATA_HIGH_MAX                               (0x0000FFFFU)

/* TX_ECC_VAL */

#define CSL_FSI_TX_CFG_TX_ECC_VAL_ECC_VAL_MASK                                 (0x007FU)
#define CSL_FSI_TX_CFG_TX_ECC_VAL_ECC_VAL_SHIFT                                (0x0000U)
#define CSL_FSI_TX_CFG_TX_ECC_VAL_ECC_VAL_MAX                                  (0x007FU)

#define CSL_FSI_TX_CFG_TX_ECC_VAL_RESERVED_1_MASK                              (0xFF80U)
#define CSL_FSI_TX_CFG_TX_ECC_VAL_RESERVED_1_SHIFT                             (0x0007U)
#define CSL_FSI_TX_CFG_TX_ECC_VAL_RESERVED_1_MAX                               (0x01FFU)

/* RESERVED_9 */

/* TX_DLYLINE_CTRL */

#define CSL_FSI_TX_CFG_TX_DLYLINE_CTRL_TXCLK_DLY_MASK                          (0x001FU)
#define CSL_FSI_TX_CFG_TX_DLYLINE_CTRL_TXCLK_DLY_SHIFT                         (0x0000U)
#define CSL_FSI_TX_CFG_TX_DLYLINE_CTRL_TXCLK_DLY_MAX                           (0x001FU)

#define CSL_FSI_TX_CFG_TX_DLYLINE_CTRL_TXD0_DLY_MASK                           (0x03E0U)
#define CSL_FSI_TX_CFG_TX_DLYLINE_CTRL_TXD0_DLY_SHIFT                          (0x0005U)
#define CSL_FSI_TX_CFG_TX_DLYLINE_CTRL_TXD0_DLY_MAX                            (0x001FU)

#define CSL_FSI_TX_CFG_TX_DLYLINE_CTRL_TXD1_DLY_MASK                           (0x7C00U)
#define CSL_FSI_TX_CFG_TX_DLYLINE_CTRL_TXD1_DLY_SHIFT                          (0x000AU)
#define CSL_FSI_TX_CFG_TX_DLYLINE_CTRL_TXD1_DLY_MAX                            (0x001FU)

#define CSL_FSI_TX_CFG_TX_DLYLINE_CTRL_RESERVED_1_MASK                         (0x8000U)
#define CSL_FSI_TX_CFG_TX_DLYLINE_CTRL_RESERVED_1_SHIFT                        (0x000FU)
#define CSL_FSI_TX_CFG_TX_DLYLINE_CTRL_RESERVED_1_MAX                          (0x0001U)

/* RESERVED_10 */

/* TX_BUF_BASE */

#define CSL_FSI_TX_CFG_TX_BUF_BASE_BASE_ADDRESS_MASK                           (0xFFFFU)
#define CSL_FSI_TX_CFG_TX_BUF_BASE_BASE_ADDRESS_SHIFT                          (0x0000U)
#define CSL_FSI_TX_CFG_TX_BUF_BASE_BASE_ADDRESS_MAX                            (0xFFFFU)

#ifdef __cplusplus
}
#endif
#endif
