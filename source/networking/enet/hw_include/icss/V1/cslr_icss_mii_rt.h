/********************************************************************
 * Copyright (C) 2013-2014 Texas Instruments Incorporated.
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
#ifndef CSLR_ICSSMIIRT_H
#define CSLR_ICSSMIIRT_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>


/**************************************************************************
* Register Overlay Structure for __ALL__
**************************************************************************/
#ifndef CSL_MODIFICATION
typedef struct {
    volatile Uint32 RXCFG0;
    volatile Uint32 RXCFG1;
    volatile Uint8  RSVD0[8];
    volatile Uint32 TXCFG0;
    volatile Uint32 TXCFG1;
    volatile Uint8  RSVD1[8];
    volatile Uint32 TX_CRC0;
    volatile Uint32 TX_CRC1;
    volatile Uint8  RSVD2[8];
    volatile Uint32 TX_IPG0;
    volatile Uint32 TX_IPG1;
    volatile Uint32 PRS0;
    volatile Uint32 PRS1;
    volatile Uint32 RX_FRMS0;
    volatile Uint32 RX_FRMS1;
    volatile Uint32 RX_PCNT0;
    volatile Uint32 RX_PCNT1;
    volatile Uint32 RX_ERR0;
    volatile Uint32 RX_ERR1;
    volatile Uint8  RSVD3[252];
} CSL_IcssMiiRtRegs;
#else
typedef struct {
    volatile Uint32 RXCFG0;
    volatile Uint32 RXCFG1;
    volatile Uint8  RSVD0[8];
    volatile Uint32 TXCFG0;
    volatile Uint32 TXCFG1;
    volatile Uint8  RSVD1[8];
    volatile Uint32 TX_CRC0;
    volatile Uint32 TX_CRC1;
    volatile Uint8  RSVD2[8];
    volatile Uint32 TX_IPG0;
    volatile Uint32 TX_IPG1;
    volatile Uint32 PRS0;
    volatile Uint32 PRS1;
    volatile Uint32 RX_FRMS0;
    volatile Uint32 RX_FRMS1;
    volatile Uint32 RX_PCNT0;
    volatile Uint32 RX_PCNT1;
    volatile Uint32 RX_ERR0;
    volatile Uint32 RX_ERR1;
    volatile Uint8  RSVD3[8];
    volatile Uint32 RX_FIFO_LEVEL0;
    volatile Uint32 RX_FIFO_LEVEL1;
    volatile Uint32 TX_FIFO_LEVEL0;
    volatile Uint32 TX_FIFO_LEVEL1;
    volatile Uint8  RSVD4[228];
} CSL_icssmiirtRegs;
#endif

/**************************************************************************
* Register Macros
**************************************************************************/

/* MII RXCFG 0 REGISTER */
#define CSL_ICSSMIIRT_RXCFG0                                    (0x0U)

/* MII RXCFG 1 REGISTER */
#define CSL_ICSSMIIRT_RXCFG1                                    (0x4U)

/* MII TXCFG 0 REGISTER */
#define CSL_ICSSMIIRT_TXCFG0                                    (0x10U)

/* MII TXCFG 1 REGISTER */
#define CSL_ICSSMIIRT_TXCFG1                                    (0x14U)

/* MII TXCRC 0 REGISTER */
#define CSL_ICSSMIIRT_TX_CRC0                                   (0x20U)

/* MII TXCRC 1 REGISTER */
#define CSL_ICSSMIIRT_TX_CRC1                                   (0x24U)

/* MII TXIPG 0 REGISTER */
#define CSL_ICSSMIIRT_TX_IPG0                                   (0x30U)

/* MII TXIPG 1 REGISTER */
#define CSL_ICSSMIIRT_TX_IPG1                                   (0x34U)

/* MII PORT STATUS 0 REGISTER */
#define CSL_ICSSMIIRT_PRS0                                      (0x38U)

/* MII PORT STATUS 1 REGISTER */
#define CSL_ICSSMIIRT_PRS1                                      (0x3CU)

/* MII RXFRMS 0 REGISTER */
#define CSL_ICSSMIIRT_RX_FRMS0                                  (0x40U)

/* MII RXFRMS 1 REGISTER */
#define CSL_ICSSMIIRT_RX_FRMS1                                  (0x44U)

/* MII RXPCNT 0 REGISTER */
#define CSL_ICSSMIIRT_RX_PCNT0                                  (0x48U)

/* MII RXPCNT 1 REGISTER */
#define CSL_ICSSMIIRT_RX_PCNT1                                  (0x4CU)

/* MII RXERR 0 REGISTER */
#define CSL_ICSSMIIRT_RX_ERR0                                   (0x50U)

/* MII RXERR 1 REGISTER */
#define CSL_ICSSMIIRT_RX_ERR1                                   (0x54U)
#define CSL_ICSSMIIRT_RX_FIFO_LEVEL0                            (0x60U)
#define CSL_ICSSMIIRT_RX_FIFO_LEVEL1                            (0x64U)
#define CSL_ICSSMIIRT_TX_FIFO_LEVEL0                            (0x68U)
#define CSL_ICSSMIIRT_TX_FIFO_LEVEL1                            (0x6CU)


/**************************************************************************
* Field Definition Macros
**************************************************************************/

/* RXCFG0 */
#ifndef CSL_MODIFICATION
#define CSL_ICSSMIIRT_RXCFG0_RX_ENABLE_MASK                     (0x00000001U)
#define CSL_ICSSMIIRT_RXCFG0_RX_ENABLE_SHIFT                    (0U)
#define CSL_ICSSMIIRT_RXCFG0_RX_ENABLE_RESETVAL                 (0x00000000U)
#define CSL_ICSSMIIRT_RXCFG0_RX_ENABLE_MAX                      (0x00000001U)
#else
#define CSL_ICSSMIIRT_RXCFG0_RX_EN_MASK                         (0x00000001U)
#define CSL_ICSSMIIRT_RXCFG0_RX_EN_SHIFT                        (0U)
#define CSL_ICSSMIIRT_RXCFG0_RX_EN_RESETVAL                     (0x00000000U)
#define CSL_ICSSMIIRT_RXCFG0_RX_EN_MAX                          (0x00000001U)
#endif
#define CSL_ICSSMIIRT_RXCFG0_RX_CUT_PREAMBLE_MASK               (0x00000004U)
#define CSL_ICSSMIIRT_RXCFG0_RX_CUT_PREAMBLE_SHIFT              (2U)
#define CSL_ICSSMIIRT_RXCFG0_RX_CUT_PREAMBLE_RESETVAL           (0x00000000U)
#define CSL_ICSSMIIRT_RXCFG0_RX_CUT_PREAMBLE_MAX                (0x00000001U)

#define CSL_ICSSMIIRT_RXCFG0_RX_MUX_SEL_MASK                    (0x00000008U)
#define CSL_ICSSMIIRT_RXCFG0_RX_MUX_SEL_SHIFT                   (3U)
#define CSL_ICSSMIIRT_RXCFG0_RX_MUX_SEL_RESETVAL                (0x00000000U)
#define CSL_ICSSMIIRT_RXCFG0_RX_MUX_SEL_MAX                     (0x00000001U)

#define CSL_ICSSMIIRT_RXCFG0_RX_L2_EN_MASK                      (0x00000010U)
#define CSL_ICSSMIIRT_RXCFG0_RX_L2_EN_SHIFT                     (4U)
#define CSL_ICSSMIIRT_RXCFG0_RX_L2_EN_RESETVAL                  (0x00000000U)
#define CSL_ICSSMIIRT_RXCFG0_RX_L2_EN_MAX                       (0x00000001U)

#define CSL_ICSSMIIRT_RXCFG0_RX_BYTE_SWAP_MASK                  (0x00000020U)
#define CSL_ICSSMIIRT_RXCFG0_RX_BYTE_SWAP_SHIFT                 (5U)
#define CSL_ICSSMIIRT_RXCFG0_RX_BYTE_SWAP_RESETVAL              (0x00000000U)
#define CSL_ICSSMIIRT_RXCFG0_RX_BYTE_SWAP_MAX                   (0x00000001U)

#define CSL_ICSSMIIRT_RXCFG0_RX_AUTO_FWD_PRE_MASK               (0x00000040U)
#define CSL_ICSSMIIRT_RXCFG0_RX_AUTO_FWD_PRE_SHIFT              (6U)
#define CSL_ICSSMIIRT_RXCFG0_RX_AUTO_FWD_PRE_RESETVAL           (0x00000000U)
#define CSL_ICSSMIIRT_RXCFG0_RX_AUTO_FWD_PRE_MAX                (0x00000001U)

#define CSL_ICSSMIIRT_RXCFG0_RX_SFD_RAW0_MASK                   (0x00000080U)
#define CSL_ICSSMIIRT_RXCFG0_RX_SFD_RAW0_SHIFT                  (7U)
#define CSL_ICSSMIIRT_RXCFG0_RX_SFD_RAW0_RESETVAL               (0x00000000U)
#define CSL_ICSSMIIRT_RXCFG0_RX_SFD_RAW0_MAX                    (0x00000001U)

#define CSL_ICSSMIIRT_RXCFG0_RX_ERR_RAW0_MASK                   (0x00000100U)
#define CSL_ICSSMIIRT_RXCFG0_RX_ERR_RAW0_SHIFT                  (8U)
#define CSL_ICSSMIIRT_RXCFG0_RX_ERR_RAW0_RESETVAL               (0x00000000U)
#define CSL_ICSSMIIRT_RXCFG0_RX_ERR_RAW0_MAX                    (0x00000001U)

#define CSL_ICSSMIIRT_RXCFG0_RX_EOF_SCLR_DIS0_MASK              (0x00000200U)
#define CSL_ICSSMIIRT_RXCFG0_RX_EOF_SCLR_DIS0_SHIFT             (9U)
#define CSL_ICSSMIIRT_RXCFG0_RX_EOF_SCLR_DIS0_RESETVAL          (0x00000000U)
#define CSL_ICSSMIIRT_RXCFG0_RX_EOF_SCLR_DIS0_MAX               (0x00000001U)

#define CSL_ICSSMIIRT_RXCFG0_RESETVAL                           (0x00000000U)

#ifndef CSL_MODIFICATION
#define CSL_ICSSMIIRT_RXCFG0_RX_DATA_RDY_MODE_DIS_SHIFT   (1U)
#define CSL_ICSSMIIRT_RXCFG0_RX_DATA_RDY_MODE_DIS_MASK    (0x00000002U)
#define CSL_ICSSMIIRT_RXCFG0_RX_L2_EOF_SCLR_DIS_SHIFT     (9U)
#define CSL_ICSSMIIRT_RXCFG0_RX_L2_EOF_SCLR_DIS_MASK      (0x00000200U)
#endif
/* RXCFG1 */
#ifndef CSL_MODIFICATION
#define CSL_ICSSMIIRT_RXCFG1_RX_ENABLE_MASK                     (0x00000001U)
#define CSL_ICSSMIIRT_RXCFG1_RX_ENABLE_SHIFT                    (0U)
#define CSL_ICSSMIIRT_RXCFG1_RX_ENABLE_RESETVAL                 (0x00000000U)
#define CSL_ICSSMIIRT_RXCFG1_RX_ENABLE_MAX                      (0x00000001U)
#else
#define CSL_ICSSMIIRT_RXCFG1_RX_EN_MASK                         (0x00000001U)
#define CSL_ICSSMIIRT_RXCFG1_RX_EN_SHIFT                        (0U)
#define CSL_ICSSMIIRT_RXCFG1_RX_EN_RESETVAL                     (0x00000000U)
#define CSL_ICSSMIIRT_RXCFG1_RX_EN_MAX                          (0x00000001U)
#endif

#define CSL_ICSSMIIRT_RXCFG1_RX_CUT_PREAMBLE_MASK               (0x00000004U)
#define CSL_ICSSMIIRT_RXCFG1_RX_CUT_PREAMBLE_SHIFT              (2U)
#define CSL_ICSSMIIRT_RXCFG1_RX_CUT_PREAMBLE_RESETVAL           (0x00000000U)
#define CSL_ICSSMIIRT_RXCFG1_RX_CUT_PREAMBLE_MAX                (0x00000001U)

#define CSL_ICSSMIIRT_RXCFG1_RX_MUX_SEL_MASK                    (0x00000008U)
#define CSL_ICSSMIIRT_RXCFG1_RX_MUX_SEL_SHIFT                   (3U)
#define CSL_ICSSMIIRT_RXCFG1_RX_MUX_SEL_RESETVAL                (0x00000001U)
#define CSL_ICSSMIIRT_RXCFG1_RX_MUX_SEL_MAX                     (0x00000001U)

#define CSL_ICSSMIIRT_RXCFG1_RX_L2_EN_MASK                      (0x00000010U)
#define CSL_ICSSMIIRT_RXCFG1_RX_L2_EN_SHIFT                     (4U)
#define CSL_ICSSMIIRT_RXCFG1_RX_L2_EN_RESETVAL                  (0x00000000U)
#define CSL_ICSSMIIRT_RXCFG1_RX_L2_EN_MAX                       (0x00000001U)

#define CSL_ICSSMIIRT_RXCFG1_RX_BYTE_SWAP_MASK                  (0x00000020U)
#define CSL_ICSSMIIRT_RXCFG1_RX_BYTE_SWAP_SHIFT                 (5U)
#define CSL_ICSSMIIRT_RXCFG1_RX_BYTE_SWAP_RESETVAL              (0x00000000U)
#define CSL_ICSSMIIRT_RXCFG1_RX_BYTE_SWAP_MAX                   (0x00000001U)

#define CSL_ICSSMIIRT_RXCFG1_RX_AUTO_FWD_PRE_MASK               (0x00000040U)
#define CSL_ICSSMIIRT_RXCFG1_RX_AUTO_FWD_PRE_SHIFT              (6U)
#define CSL_ICSSMIIRT_RXCFG1_RX_AUTO_FWD_PRE_RESETVAL           (0x00000000U)
#define CSL_ICSSMIIRT_RXCFG1_RX_AUTO_FWD_PRE_MAX                (0x00000001U)

#define CSL_ICSSMIIRT_RXCFG1_RX_SFD_RAW0_MASK                   (0x00000080U)
#define CSL_ICSSMIIRT_RXCFG1_RX_SFD_RAW0_SHIFT                  (7U)
#define CSL_ICSSMIIRT_RXCFG1_RX_SFD_RAW0_RESETVAL               (0x00000000U)
#define CSL_ICSSMIIRT_RXCFG1_RX_SFD_RAW0_MAX                    (0x00000001U)

#define CSL_ICSSMIIRT_RXCFG1_RX_ERR_RAW0_MASK                   (0x00000100U)
#define CSL_ICSSMIIRT_RXCFG1_RX_ERR_RAW0_SHIFT                  (8U)
#define CSL_ICSSMIIRT_RXCFG1_RX_ERR_RAW0_RESETVAL               (0x00000000U)
#define CSL_ICSSMIIRT_RXCFG1_RX_ERR_RAW0_MAX                    (0x00000001U)

#define CSL_ICSSMIIRT_RXCFG1_RX_EOF_SCLR_DIS0_MASK              (0x00000200U)
#define CSL_ICSSMIIRT_RXCFG1_RX_EOF_SCLR_DIS0_SHIFT             (9U)
#define CSL_ICSSMIIRT_RXCFG1_RX_EOF_SCLR_DIS0_RESETVAL          (0x00000000U)
#define CSL_ICSSMIIRT_RXCFG1_RX_EOF_SCLR_DIS0_MAX               (0x00000001U)

#define CSL_ICSSMIIRT_RXCFG1_RESETVAL                           (0x00000008U)

#ifndef CSL_MODIFICATION
#define CSL_ICSSMIIRT_RXCFG1_RX_DATA_RDY_MODE_DIS_SHIFT   (1U)
#define CSL_ICSSMIIRT_RXCFG1_RX_DATA_RDY_MODE_DIS_MASK    (0x00000002U)
#define CSL_ICSSMIIRT_RXCFG1_RX_L2_EOF_SCLR_DIS_SHIFT     (9U)
#define CSL_ICSSMIIRT_RXCFG1_RX_L2_EOF_SCLR_DIS_MASK      (0x00000200U)
#endif
/* TXCFG0 */

#ifndef CSL_MODIFICATION
#define CSL_ICSSMIIRT_TXCFG0_TX_ENABLE_MASK                     (0x00000001U)
#define CSL_ICSSMIIRT_TXCFG0_TX_ENABLE_SHIFT                    (0U)
#define CSL_ICSSMIIRT_TXCFG0_TX_ENABLE_RESETVAL                 (0x00000000U)
#define CSL_ICSSMIIRT_TXCFG0_TX_ENABLE_MAX                      (0x00000001U)
#else
#define CSL_ICSSMIIRT_TXCFG0_TX_EN_MASK                         (0x00000001U)
#define CSL_ICSSMIIRT_TXCFG0_TX_EN_SHIFT                        (0U)
#define CSL_ICSSMIIRT_TXCFG0_TX_EN_RESETVAL                     (0x00000000U)
#define CSL_ICSSMIIRT_TXCFG0_TX_EN_MAX                          (0x00000001U)
#endif

#define CSL_ICSSMIIRT_TXCFG0_TX_AUTO_PREAMBLE_MASK              (0x00000002U)
#define CSL_ICSSMIIRT_TXCFG0_TX_AUTO_PREAMBLE_SHIFT             (1U)
#define CSL_ICSSMIIRT_TXCFG0_TX_AUTO_PREAMBLE_RESETVAL          (0x00000000U)
#define CSL_ICSSMIIRT_TXCFG0_TX_AUTO_PREAMBLE_MAX               (0x00000001U)

#define CSL_ICSSMIIRT_TXCFG0_TX_EN_MODE_MASK                    (0x00000004U)
#define CSL_ICSSMIIRT_TXCFG0_TX_EN_MODE_SHIFT                   (2U)
#define CSL_ICSSMIIRT_TXCFG0_TX_EN_MODE_RESETVAL                (0x00000000U)
#define CSL_ICSSMIIRT_TXCFG0_TX_EN_MODE_MAX                     (0x00000001U)

#define CSL_ICSSMIIRT_TXCFG0_TX_BYTE_SWAP_MASK                  (0x00000008U)
#define CSL_ICSSMIIRT_TXCFG0_TX_BYTE_SWAP_SHIFT                 (3U)
#define CSL_ICSSMIIRT_TXCFG0_TX_BYTE_SWAP_RESETVAL              (0x00000000U)
#define CSL_ICSSMIIRT_TXCFG0_TX_BYTE_SWAP_MAX                   (0x00000001U)

#define CSL_ICSSMIIRT_TXCFG0_TX_MUX_SEL_MASK                    (0x00000100U)
#define CSL_ICSSMIIRT_TXCFG0_TX_MUX_SEL_SHIFT                   (8U)
#define CSL_ICSSMIIRT_TXCFG0_TX_MUX_SEL_RESETVAL                (0x00000001U)
#define CSL_ICSSMIIRT_TXCFG0_TX_MUX_SEL_MAX                     (0x00000001U)

#define CSL_ICSSMIIRT_TXCFG0_PRE_TX_AUTO_SEQUENCE_MASK          (0x00000200U)
#define CSL_ICSSMIIRT_TXCFG0_PRE_TX_AUTO_SEQUENCE_SHIFT         (9U)
#define CSL_ICSSMIIRT_TXCFG0_PRE_TX_AUTO_SEQUENCE_RESETVAL      (0x00000000U)
#define CSL_ICSSMIIRT_TXCFG0_PRE_TX_AUTO_SEQUENCE_MAX           (0x00000001U)

#define CSL_ICSSMIIRT_TXCFG0_PRE_TX_AUTO_ESC_ERR_MASK           (0x00000400U)
#define CSL_ICSSMIIRT_TXCFG0_PRE_TX_AUTO_ESC_ERR_SHIFT          (10U)
#define CSL_ICSSMIIRT_TXCFG0_PRE_TX_AUTO_ESC_ERR_RESETVAL       (0x00000000U)
#define CSL_ICSSMIIRT_TXCFG0_PRE_TX_AUTO_ESC_ERR_MAX            (0x00000001U)

#define CSL_ICSSMIIRT_TXCFG0_TX_32_MODE_EN_MASK                 (0x00000800U)
#define CSL_ICSSMIIRT_TXCFG0_TX_32_MODE_EN_SHIFT                (11U)
#define CSL_ICSSMIIRT_TXCFG0_TX_32_MODE_EN_RESETVAL             (0x00000000U)
#define CSL_ICSSMIIRT_TXCFG0_TX_32_MODE_EN_MAX                  (0x00000001U)

#define CSL_ICSSMIIRT_TXCFG0_TX_START_DELAY_MASK                (0x03FF0000U)
#define CSL_ICSSMIIRT_TXCFG0_TX_START_DELAY_SHIFT               (16U)
#define CSL_ICSSMIIRT_TXCFG0_TX_START_DELAY_RESETVAL            (0x00000040U)
#define CSL_ICSSMIIRT_TXCFG0_TX_START_DELAY_MAX                 (0x000003ffU)

#define CSL_ICSSMIIRT_TXCFG0_TX_CLK_DELAY_MASK                  (0x70000000U)
#define CSL_ICSSMIIRT_TXCFG0_TX_CLK_DELAY_SHIFT                 (28U)
#define CSL_ICSSMIIRT_TXCFG0_TX_CLK_DELAY_RESETVAL              (0x00000000U)
#define CSL_ICSSMIIRT_TXCFG0_TX_CLK_DELAY_MAX                   (0x00000007U)

#define CSL_ICSSMIIRT_TXCFG0_RESETVAL                           (0x00400100U)

/* TXCFG1 */
#ifndef CSL_MODIFICATION
#define CSL_ICSSMIIRT_TXCFG1_TX_ENABLE_MASK                     (0x00000001U)
#define CSL_ICSSMIIRT_TXCFG1_TX_ENABLE_SHIFT                    (0U)
#define CSL_ICSSMIIRT_TXCFG1_TX_ENABLE_RESETVAL                 (0x00000000U)
#define CSL_ICSSMIIRT_TXCFG1_TX_ENABLE_MAX                      (0x00000001U)
#else
#define CSL_ICSSMIIRT_TXCFG1_TX_EN_MASK                         (0x00000001U)
#define CSL_ICSSMIIRT_TXCFG1_TX_EN_SHIFT                        (0U)
#define CSL_ICSSMIIRT_TXCFG1_TX_EN_RESETVAL                     (0x00000000U)
#define CSL_ICSSMIIRT_TXCFG1_TX_EN_MAX                          (0x00000001U)
#endif

#define CSL_ICSSMIIRT_TXCFG1_TX_AUTO_PREAMBLE_MASK              (0x00000002U)
#define CSL_ICSSMIIRT_TXCFG1_TX_AUTO_PREAMBLE_SHIFT             (1U)
#define CSL_ICSSMIIRT_TXCFG1_TX_AUTO_PREAMBLE_RESETVAL          (0x00000000U)
#define CSL_ICSSMIIRT_TXCFG1_TX_AUTO_PREAMBLE_MAX               (0x00000001U)

#define CSL_ICSSMIIRT_TXCFG1_TX_EN_MODE_MASK                    (0x00000004U)
#define CSL_ICSSMIIRT_TXCFG1_TX_EN_MODE_SHIFT                   (2U)
#define CSL_ICSSMIIRT_TXCFG1_TX_EN_MODE_RESETVAL                (0x00000000U)
#define CSL_ICSSMIIRT_TXCFG1_TX_EN_MODE_MAX                     (0x00000001U)

#define CSL_ICSSMIIRT_TXCFG1_TX_BYTE_SWAP_MASK                  (0x00000008U)
#define CSL_ICSSMIIRT_TXCFG1_TX_BYTE_SWAP_SHIFT                 (3U)
#define CSL_ICSSMIIRT_TXCFG1_TX_BYTE_SWAP_RESETVAL              (0x00000000U)
#define CSL_ICSSMIIRT_TXCFG1_TX_BYTE_SWAP_MAX                   (0x00000001U)

#define CSL_ICSSMIIRT_TXCFG1_TX_MUX_SEL_MASK                    (0x00000100U)
#define CSL_ICSSMIIRT_TXCFG1_TX_MUX_SEL_SHIFT                   (8U)
#define CSL_ICSSMIIRT_TXCFG1_TX_MUX_SEL_RESETVAL                (0x00000000U)
#define CSL_ICSSMIIRT_TXCFG1_TX_MUX_SEL_MAX                     (0x00000001U)

#define CSL_ICSSMIIRT_TXCFG1_PRE_TX_AUTO_SEQUENCE_MASK          (0x00000200U)
#define CSL_ICSSMIIRT_TXCFG1_PRE_TX_AUTO_SEQUENCE_SHIFT         (9U)
#define CSL_ICSSMIIRT_TXCFG1_PRE_TX_AUTO_SEQUENCE_RESETVAL      (0x00000000U)
#define CSL_ICSSMIIRT_TXCFG1_PRE_TX_AUTO_SEQUENCE_MAX           (0x00000001U)

#define CSL_ICSSMIIRT_TXCFG1_PRE_TX_AUTO_ESC_ERR_MASK           (0x00000400U)
#define CSL_ICSSMIIRT_TXCFG1_PRE_TX_AUTO_ESC_ERR_SHIFT          (10U)
#define CSL_ICSSMIIRT_TXCFG1_PRE_TX_AUTO_ESC_ERR_RESETVAL       (0x00000000U)
#define CSL_ICSSMIIRT_TXCFG1_PRE_TX_AUTO_ESC_ERR_MAX            (0x00000001U)

#define CSL_ICSSMIIRT_TXCFG1_TX_START_DELAY_MASK                (0x03FF0000U)
#define CSL_ICSSMIIRT_TXCFG1_TX_START_DELAY_SHIFT               (16U)
#define CSL_ICSSMIIRT_TXCFG1_TX_START_DELAY_RESETVAL            (0x00000040U)
#define CSL_ICSSMIIRT_TXCFG1_TX_START_DELAY_MAX                 (0x000003ffU)

#define CSL_ICSSMIIRT_TXCFG1_TX_32_MODE_EN_MASK                 (0x00000800U)
#define CSL_ICSSMIIRT_TXCFG1_TX_32_MODE_EN_SHIFT                (11U)
#define CSL_ICSSMIIRT_TXCFG1_TX_32_MODE_EN_RESETVAL             (0x00000000U)
#define CSL_ICSSMIIRT_TXCFG1_TX_32_MODE_EN_MAX                  (0x00000001U)

#define CSL_ICSSMIIRT_TXCFG1_TX_CLK_DELAY_MASK                  (0x70000000U)
#define CSL_ICSSMIIRT_TXCFG1_TX_CLK_DELAY_SHIFT                 (28U)
#define CSL_ICSSMIIRT_TXCFG1_TX_CLK_DELAY_RESETVAL              (0x00000000U)
#define CSL_ICSSMIIRT_TXCFG1_TX_CLK_DELAY_MAX                   (0x00000007U)

#define CSL_ICSSMIIRT_TXCFG1_RESETVAL                           (0x00400000U)

/* TX_CRC0 */

#define CSL_ICSSMIIRT_TX_CRC0_TX_CRC_MASK                       (0xFFFFFFFFU)
#define CSL_ICSSMIIRT_TX_CRC0_TX_CRC_SHIFT                      (0U)
#define CSL_ICSSMIIRT_TX_CRC0_TX_CRC_RESETVAL                   (0x00000000U)
#define CSL_ICSSMIIRT_TX_CRC0_TX_CRC_MAX                        (0xffffffffU)

#define CSL_ICSSMIIRT_TX_CRC0_RESETVAL                          (0x00000000U)

/* TX_CRC1 */

#define CSL_ICSSMIIRT_TX_CRC1_TX_CRC_MASK                       (0xFFFFFFFFU)
#define CSL_ICSSMIIRT_TX_CRC1_TX_CRC_SHIFT                      (0U)
#define CSL_ICSSMIIRT_TX_CRC1_TX_CRC_RESETVAL                   (0x00000000U)
#define CSL_ICSSMIIRT_TX_CRC1_TX_CRC_MAX                        (0xffffffffU)

#define CSL_ICSSMIIRT_TX_CRC1_RESETVAL                          (0x00000000U)

/* TX_IPG0 */

#define CSL_ICSSMIIRT_TX_IPG0_TX_IPG_MASK                       (0x000003FFU)
#define CSL_ICSSMIIRT_TX_IPG0_TX_IPG_SHIFT                      (0U)
#define CSL_ICSSMIIRT_TX_IPG0_TX_IPG_RESETVAL                   (0x00000028U)
#define CSL_ICSSMIIRT_TX_IPG0_TX_IPG_MAX                        (0x000003ffU)

#define CSL_ICSSMIIRT_TX_IPG0_RESETVAL                          (0x00000028U)

/* TX_IPG1 */

#define CSL_ICSSMIIRT_TX_IPG1_TX_IPG_MASK                       (0x000003FFU)
#define CSL_ICSSMIIRT_TX_IPG1_TX_IPG_SHIFT                      (0U)
#define CSL_ICSSMIIRT_TX_IPG1_TX_IPG_RESETVAL                   (0x00000028U)
#define CSL_ICSSMIIRT_TX_IPG1_TX_IPG_MAX                        (0x000003ffU)

#define CSL_ICSSMIIRT_TX_IPG1_RESETVAL                          (0x00000028U)

/* PRS0 */

#define CSL_ICSSMIIRT_PRS0_MII_COL_MASK                         (0x00000001U)
#define CSL_ICSSMIIRT_PRS0_MII_COL_SHIFT                        (0U)
#define CSL_ICSSMIIRT_PRS0_MII_COL_RESETVAL                     (0x00000000U)
#define CSL_ICSSMIIRT_PRS0_MII_COL_MAX                          (0x00000001U)

#define CSL_ICSSMIIRT_PRS0_MII_CRS_MASK                         (0x00000002U)
#define CSL_ICSSMIIRT_PRS0_MII_CRS_SHIFT                        (1U)
#define CSL_ICSSMIIRT_PRS0_MII_CRS_RESETVAL                     (0x00000000U)
#define CSL_ICSSMIIRT_PRS0_MII_CRS_MAX                          (0x00000001U)

#define CSL_ICSSMIIRT_PRS0_RESETVAL                             (0x00000000U)

/* PRS1 */

#define CSL_ICSSMIIRT_PRS1_MII_COL_MASK                         (0x00000001U)
#define CSL_ICSSMIIRT_PRS1_MII_COL_SHIFT                        (0U)
#define CSL_ICSSMIIRT_PRS1_MII_COL_RESETVAL                     (0x00000000U)
#define CSL_ICSSMIIRT_PRS1_MII_COL_MAX                          (0x00000001U)

#define CSL_ICSSMIIRT_PRS1_MII_CRS_MASK                         (0x00000002U)
#define CSL_ICSSMIIRT_PRS1_MII_CRS_SHIFT                        (1U)
#define CSL_ICSSMIIRT_PRS1_MII_CRS_RESETVAL                     (0x00000000U)
#define CSL_ICSSMIIRT_PRS1_MII_CRS_MAX                          (0x00000001U)

#define CSL_ICSSMIIRT_PRS1_RESETVAL                             (0x00000000U)

/* RX_FRMS0 */

#define CSL_ICSSMIIRT_RX_FRMS0_RX_MIN_FRM_MASK                  (0x0000FFFFU)
#define CSL_ICSSMIIRT_RX_FRMS0_RX_MIN_FRM_SHIFT                 (0U)
#define CSL_ICSSMIIRT_RX_FRMS0_RX_MIN_FRM_RESETVAL              (0x0000003fU)
#define CSL_ICSSMIIRT_RX_FRMS0_RX_MIN_FRM_MAX                   (0x0000ffffU)

#define CSL_ICSSMIIRT_RX_FRMS0_RX_MAX_FRM_MASK                  (0xFFFF0000U)
#define CSL_ICSSMIIRT_RX_FRMS0_RX_MAX_FRM_SHIFT                 (16U)
#define CSL_ICSSMIIRT_RX_FRMS0_RX_MAX_FRM_RESETVAL              (0x000005f1U)
#define CSL_ICSSMIIRT_RX_FRMS0_RX_MAX_FRM_MAX                   (0x0000ffffU)

#define CSL_ICSSMIIRT_RX_FRMS0_RESETVAL                         (0x05f1003fU)

/* RX_FRMS1 */

#define CSL_ICSSMIIRT_RX_FRMS1_RX_MIN_FRM_MASK                  (0x0000FFFFU)
#define CSL_ICSSMIIRT_RX_FRMS1_RX_MIN_FRM_SHIFT                 (0U)
#define CSL_ICSSMIIRT_RX_FRMS1_RX_MIN_FRM_RESETVAL              (0x0000003fU)
#define CSL_ICSSMIIRT_RX_FRMS1_RX_MIN_FRM_MAX                   (0x0000ffffU)

#define CSL_ICSSMIIRT_RX_FRMS1_RX_MAX_FRM_MASK                  (0xFFFF0000U)
#define CSL_ICSSMIIRT_RX_FRMS1_RX_MAX_FRM_SHIFT                 (16U)
#define CSL_ICSSMIIRT_RX_FRMS1_RX_MAX_FRM_RESETVAL              (0x000005f1U)
#define CSL_ICSSMIIRT_RX_FRMS1_RX_MAX_FRM_MAX                   (0x0000ffffU)

#define CSL_ICSSMIIRT_RX_FRMS1_RESETVAL                         (0x05f1003fU)

/* RX_PCNT0 */

#define CSL_ICSSMIIRT_RX_PCNT0_RX_MIN_PCNT_MASK                 (0x0000000FU)
#define CSL_ICSSMIIRT_RX_PCNT0_RX_MIN_PCNT_SHIFT                (0U)
#define CSL_ICSSMIIRT_RX_PCNT0_RX_MIN_PCNT_RESETVAL             (0x00000001U)
#define CSL_ICSSMIIRT_RX_PCNT0_RX_MIN_PCNT_MAX                  (0x0000000fU)

#define CSL_ICSSMIIRT_RX_PCNT0_RX_MAX_PCNT_MASK                 (0x000000F0U)
#define CSL_ICSSMIIRT_RX_PCNT0_RX_MAX_PCNT_SHIFT                (4U)
#define CSL_ICSSMIIRT_RX_PCNT0_RX_MAX_PCNT_RESETVAL             (0x0000000eU)
#define CSL_ICSSMIIRT_RX_PCNT0_RX_MAX_PCNT_MAX                  (0x0000000fU)

#define CSL_ICSSMIIRT_RX_PCNT0_RESETVAL                         (0x000000e1U)

/* RX_PCNT1 */

#define CSL_ICSSMIIRT_RX_PCNT1_RX_MIN_PCNT_MASK                 (0x0000000FU)
#define CSL_ICSSMIIRT_RX_PCNT1_RX_MIN_PCNT_SHIFT                (0U)
#define CSL_ICSSMIIRT_RX_PCNT1_RX_MIN_PCNT_RESETVAL             (0x00000001U)
#define CSL_ICSSMIIRT_RX_PCNT1_RX_MIN_PCNT_MAX                  (0x0000000fU)

#define CSL_ICSSMIIRT_RX_PCNT1_RX_MAX_PCNT_MASK                 (0x000000F0U)
#define CSL_ICSSMIIRT_RX_PCNT1_RX_MAX_PCNT_SHIFT                (4U)
#define CSL_ICSSMIIRT_RX_PCNT1_RX_MAX_PCNT_RESETVAL             (0x0000000eU)
#define CSL_ICSSMIIRT_RX_PCNT1_RX_MAX_PCNT_MAX                  (0x0000000fU)

#define CSL_ICSSMIIRT_RX_PCNT1_RESETVAL                         (0x000000e1U)

/* RX_ERR0 */

#define CSL_ICSSMIIRT_RX_ERR0_RX_MIN_PCNT_ERR_MASK              (0x00000001U)
#define CSL_ICSSMIIRT_RX_ERR0_RX_MIN_PCNT_ERR_SHIFT             (0U)
#define CSL_ICSSMIIRT_RX_ERR0_RX_MIN_PCNT_ERR_RESETVAL          (0x00000000U)
#define CSL_ICSSMIIRT_RX_ERR0_RX_MIN_PCNT_ERR_MAX               (0x00000001U)

#define CSL_ICSSMIIRT_RX_ERR0_RX_MAX_PCNT_ERR_MASK              (0x00000002U)
#define CSL_ICSSMIIRT_RX_ERR0_RX_MAX_PCNT_ERR_SHIFT             (1U)
#define CSL_ICSSMIIRT_RX_ERR0_RX_MAX_PCNT_ERR_RESETVAL          (0x00000000U)
#define CSL_ICSSMIIRT_RX_ERR0_RX_MAX_PCNT_ERR_MAX               (0x00000001U)

#define CSL_ICSSMIIRT_RX_ERR0_RX_MIN_FRM_ERR_MASK               (0x00000004U)
#define CSL_ICSSMIIRT_RX_ERR0_RX_MIN_FRM_ERR_SHIFT              (2U)
#define CSL_ICSSMIIRT_RX_ERR0_RX_MIN_FRM_ERR_RESETVAL           (0x00000000U)
#define CSL_ICSSMIIRT_RX_ERR0_RX_MIN_FRM_ERR_MAX                (0x00000001U)

#define CSL_ICSSMIIRT_RX_ERR0_RX_MAX_FRM_ERR_MASK               (0x00000008U)
#define CSL_ICSSMIIRT_RX_ERR0_RX_MAX_FRM_ERR_SHIFT              (3U)
#define CSL_ICSSMIIRT_RX_ERR0_RX_MAX_FRM_ERR_RESETVAL           (0x00000000U)
#define CSL_ICSSMIIRT_RX_ERR0_RX_MAX_FRM_ERR_MAX                (0x00000001U)

#define CSL_ICSSMIIRT_RX_ERR0_RESETVAL                          (0x00000000U)

/* RX_ERR1 */

#define CSL_ICSSMIIRT_RX_ERR1_RX_MIN_PCNT_ERR_MASK              (0x00000001U)
#define CSL_ICSSMIIRT_RX_ERR1_RX_MIN_PCNT_ERR_SHIFT             (0U)
#define CSL_ICSSMIIRT_RX_ERR1_RX_MIN_PCNT_ERR_RESETVAL          (0x00000000U)
#define CSL_ICSSMIIRT_RX_ERR1_RX_MIN_PCNT_ERR_MAX               (0x00000001U)

#define CSL_ICSSMIIRT_RX_ERR1_RX_MAX_PCNT_ERR_MASK              (0x00000002U)
#define CSL_ICSSMIIRT_RX_ERR1_RX_MAX_PCNT_ERR_SHIFT             (1U)
#define CSL_ICSSMIIRT_RX_ERR1_RX_MAX_PCNT_ERR_RESETVAL          (0x00000000U)
#define CSL_ICSSMIIRT_RX_ERR1_RX_MAX_PCNT_ERR_MAX               (0x00000001U)

#define CSL_ICSSMIIRT_RX_ERR1_RX_MIN_FRM_ERR_MASK               (0x00000004U)
#define CSL_ICSSMIIRT_RX_ERR1_RX_MIN_FRM_ERR_SHIFT              (2U)
#define CSL_ICSSMIIRT_RX_ERR1_RX_MIN_FRM_ERR_RESETVAL           (0x00000000U)
#define CSL_ICSSMIIRT_RX_ERR1_RX_MIN_FRM_ERR_MAX                (0x00000001U)

#define CSL_ICSSMIIRT_RX_ERR1_RX_MAX_FRM_ERR_MASK               (0x00000008U)
#define CSL_ICSSMIIRT_RX_ERR1_RX_MAX_FRM_ERR_SHIFT              (3U)
#define CSL_ICSSMIIRT_RX_ERR1_RX_MAX_FRM_ERR_RESETVAL           (0x00000000U)
#define CSL_ICSSMIIRT_RX_ERR1_RX_MAX_FRM_ERR_MAX                (0x00000001U)

#define CSL_ICSSMIIRT_RX_ERR1_RESETVAL                          (0x00000000U)

/* RX_FIFO_LEVEL0 */

#define CSL_ICSSMIIRT_RX_FIFO_LEVEL0_RX_FIFO_LEVEL0_MASK        (0x000000FFU)
#define CSL_ICSSMIIRT_RX_FIFO_LEVEL0_RX_FIFO_LEVEL0_SHIFT       (0U)
#define CSL_ICSSMIIRT_RX_FIFO_LEVEL0_RX_FIFO_LEVEL0_RESETVAL    (0x00000000U)
#define CSL_ICSSMIIRT_RX_FIFO_LEVEL0_RX_FIFO_LEVEL0_MAX         (0x000000ffU)

#define CSL_ICSSMIIRT_RX_FIFO_LEVEL0_RESETVAL                   (0x00000000U)

/* RX_FIFO_LEVEL1 */

#define CSL_ICSSMIIRT_RX_FIFO_LEVEL1_RX_FIFO_LEVEL1_MASK        (0x000000FFU)
#define CSL_ICSSMIIRT_RX_FIFO_LEVEL1_RX_FIFO_LEVEL1_SHIFT       (0U)
#define CSL_ICSSMIIRT_RX_FIFO_LEVEL1_RX_FIFO_LEVEL1_RESETVAL    (0x00000000U)
#define CSL_ICSSMIIRT_RX_FIFO_LEVEL1_RX_FIFO_LEVEL1_MAX         (0x000000ffU)

#define CSL_ICSSMIIRT_RX_FIFO_LEVEL1_RESETVAL                   (0x00000000U)

/* TX_FIFO_LEVEL0 */

#define CSL_ICSSMIIRT_TX_FIFO_LEVEL0_TX_FIFO_LEVEL0_MASK        (0x000000FFU)
#define CSL_ICSSMIIRT_TX_FIFO_LEVEL0_TX_FIFO_LEVEL0_SHIFT       (0U)
#define CSL_ICSSMIIRT_TX_FIFO_LEVEL0_TX_FIFO_LEVEL0_RESETVAL    (0x00000000U)
#define CSL_ICSSMIIRT_TX_FIFO_LEVEL0_TX_FIFO_LEVEL0_MAX         (0x000000ffU)

#define CSL_ICSSMIIRT_TX_FIFO_LEVEL0_RESETVAL                   (0x00000000U)

/* TX_FIFO_LEVEL1 */

#define CSL_ICSSMIIRT_TX_FIFO_LEVEL1_TX_FIFO_LEVEL1_MASK        (0x000000FFU)
#define CSL_ICSSMIIRT_TX_FIFO_LEVEL1_TX_FIFO_LEVEL1_SHIFT       (0U)
#define CSL_ICSSMIIRT_TX_FIFO_LEVEL1_TX_FIFO_LEVEL1_RESETVAL    (0x00000000U)
#define CSL_ICSSMIIRT_TX_FIFO_LEVEL1_TX_FIFO_LEVEL1_MAX         (0x000000ffU)

#define CSL_ICSSMIIRT_TX_FIFO_LEVEL1_RESETVAL                   (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
