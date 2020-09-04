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
 *  Name        : cslr_sci.h
*/
#ifndef CSLR_SCI_H_
#define CSLR_SCI_H_

#include <stdint.h>

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

typedef struct {
    volatile uint32_t SCIGCR0;
    volatile uint32_t SCIGCR1;
    volatile uint32_t RESERVED1;
    volatile uint32_t SCISETINT;
    volatile uint32_t SCICLEARINT;
    volatile uint32_t SCISETINTLVL;
    volatile uint32_t SCICLEARINTLVL;
    volatile uint32_t SCIFLR;
    volatile uint32_t SCIINTVECT0;
    volatile uint32_t SCIINTVECT1;
    volatile uint32_t SCICHAR;
    volatile uint32_t SCIBAUD;
    volatile uint32_t SCIED;
    volatile uint32_t SCIRD;
    volatile uint32_t SCITD;
    volatile uint32_t SCIPIO0;
    volatile uint32_t SCIPIO1;
    volatile uint32_t SCIPIO2;
    volatile uint32_t SCIPIO3;
    volatile uint32_t SCIPIO4;
    volatile uint32_t SCIPIO5;
    volatile uint32_t SCIPIO6;
    volatile uint32_t SCIPIO7;
    volatile uint32_t SCIPIO8;
    volatile uint32_t RESERVED2;
    volatile uint32_t RESERVED3;
    volatile uint32_t RESERVED4;
    volatile uint32_t RESERVED5;
    volatile uint32_t RESERVED6;
    volatile uint32_t RESERVED7;
    volatile uint32_t RESERVED8;
    volatile uint32_t RESERVED9;
    volatile uint32_t SCIPIO9;
    volatile uint8_t  Resv_144[12];
    volatile uint32_t SCIIODCTRL;
} CSL_sciRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_SCI_SCIGCR0                                                    (0x00000000U)
#define CSL_SCI_SCIGCR1                                                    (0x00000004U)
#define CSL_SCI_RESERVED1                                                  (0x00000008U)
#define CSL_SCI_SCISETINT                                                  (0x0000000CU)
#define CSL_SCI_SCICLEARINT                                                (0x00000010U)
#define CSL_SCI_SCISETINTLVL                                               (0x00000014U)
#define CSL_SCI_SCICLEARINTLVL                                             (0x00000018U)
#define CSL_SCI_SCIFLR                                                     (0x0000001CU)
#define CSL_SCI_SCIINTVECT0                                                (0x00000020U)
#define CSL_SCI_SCIINTVECT1                                                (0x00000024U)
#define CSL_SCI_SCICHAR                                                    (0x00000028U)
#define CSL_SCI_SCIBAUD                                                    (0x0000002CU)
#define CSL_SCI_SCIED                                                      (0x00000030U)
#define CSL_SCI_SCIRD                                                      (0x00000034U)
#define CSL_SCI_SCITD                                                      (0x00000038U)
#define CSL_SCI_SCIPIO0                                                    (0x0000003CU)
#define CSL_SCI_SCIPIO1                                                    (0x00000040U)
#define CSL_SCI_SCIPIO2                                                    (0x00000044U)
#define CSL_SCI_SCIPIO3                                                    (0x00000048U)
#define CSL_SCI_SCIPIO4                                                    (0x0000004CU)
#define CSL_SCI_SCIPIO5                                                    (0x00000050U)
#define CSL_SCI_SCIPIO6                                                    (0x00000054U)
#define CSL_SCI_SCIPIO7                                                    (0x00000058U)
#define CSL_SCI_SCIPIO8                                                    (0x0000005CU)
#define CSL_SCI_RESERVED2                                                  (0x00000060U)
#define CSL_SCI_RESERVED3                                                  (0x00000064U)
#define CSL_SCI_RESERVED4                                                  (0x00000068U)
#define CSL_SCI_RESERVED5                                                  (0x0000006CU)
#define CSL_SCI_RESERVED6                                                  (0x00000070U)
#define CSL_SCI_RESERVED7                                                  (0x00000074U)
#define CSL_SCI_RESERVED8                                                  (0x00000078U)
#define CSL_SCI_RESERVED9                                                  (0x0000007CU)
#define CSL_SCI_SCIPIO9                                                    (0x00000080U)
#define CSL_SCI_SCIIODCTRL                                                 (0x00000090U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* SCIGCR0 */

#define CSL_SCI_SCIGCR0_RESET_MASK                                         (0x00000001U)
#define CSL_SCI_SCIGCR0_RESET_SHIFT                                        (0x00000000U)
#define CSL_SCI_SCIGCR0_RESET_RESETVAL                                     (0x00000000U)
#define CSL_SCI_SCIGCR0_RESET_MAX                                          (0x00000001U)

#define CSL_SCI_SCIGCR0_RESERVED_MASK                                      (0xFFFFFFFEU)
#define CSL_SCI_SCIGCR0_RESERVED_SHIFT                                     (0x00000001U)
#define CSL_SCI_SCIGCR0_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_SCI_SCIGCR0_RESERVED_MAX                                       (0x7FFFFFFFU)

#define CSL_SCI_SCIGCR0_RESETVAL                                           (0x00000000U)

/* SCIGCR1 */

#define CSL_SCI_SCIGCR1_COMM_MODE_MASK                                     (0x00000001U)
#define CSL_SCI_SCIGCR1_COMM_MODE_SHIFT                                    (0x00000000U)
#define CSL_SCI_SCIGCR1_COMM_MODE_RESETVAL                                 (0x00000000U)
#define CSL_SCI_SCIGCR1_COMM_MODE_MAX                                      (0x00000001U)

#define CSL_SCI_SCIGCR1_TIMING_MODE_MASK                                   (0x00000002U)
#define CSL_SCI_SCIGCR1_TIMING_MODE_SHIFT                                  (0x00000001U)
#define CSL_SCI_SCIGCR1_TIMING_MODE_RESETVAL                               (0x00000000U)
#define CSL_SCI_SCIGCR1_TIMING_MODE_MAX                                    (0x00000001U)

#define CSL_SCI_SCIGCR1_PARITY_ENA_MASK                                    (0x00000004U)
#define CSL_SCI_SCIGCR1_PARITY_ENA_SHIFT                                   (0x00000002U)
#define CSL_SCI_SCIGCR1_PARITY_ENA_RESETVAL                                (0x00000000U)
#define CSL_SCI_SCIGCR1_PARITY_ENA_MAX                                     (0x00000001U)

#define CSL_SCI_SCIGCR1_PARITY_MASK                                        (0x00000008U)
#define CSL_SCI_SCIGCR1_PARITY_SHIFT                                       (0x00000003U)
#define CSL_SCI_SCIGCR1_PARITY_RESETVAL                                    (0x00000000U)
#define CSL_SCI_SCIGCR1_PARITY_MAX                                         (0x00000001U)

#define CSL_SCI_SCIGCR1_STOP_MASK                                          (0x00000010U)
#define CSL_SCI_SCIGCR1_STOP_SHIFT                                         (0x00000004U)
#define CSL_SCI_SCIGCR1_STOP_RESETVAL                                      (0x00000000U)
#define CSL_SCI_SCIGCR1_STOP_MAX                                           (0x00000001U)

#define CSL_SCI_SCIGCR1_CLOCK_MASK                                         (0x00000020U)
#define CSL_SCI_SCIGCR1_CLOCK_SHIFT                                        (0x00000005U)
#define CSL_SCI_SCIGCR1_CLOCK_RESETVAL                                     (0x00000000U)
#define CSL_SCI_SCIGCR1_CLOCK_MAX                                          (0x00000001U)

#define CSL_SCI_SCIGCR1_RESERVED1_MASK                                     (0x00000040U)
#define CSL_SCI_SCIGCR1_RESERVED1_SHIFT                                    (0x00000006U)
#define CSL_SCI_SCIGCR1_RESERVED1_RESETVAL                                 (0x00000000U)
#define CSL_SCI_SCIGCR1_RESERVED1_MAX                                      (0x00000001U)

#define CSL_SCI_SCIGCR1_SW_NRESET_MASK                                     (0x00000080U)
#define CSL_SCI_SCIGCR1_SW_NRESET_SHIFT                                    (0x00000007U)
#define CSL_SCI_SCIGCR1_SW_NRESET_RESETVAL                                 (0x00000000U)
#define CSL_SCI_SCIGCR1_SW_NRESET_MAX                                      (0x00000001U)

#define CSL_SCI_SCIGCR1_SLEEP_MASK                                         (0x00000100U)
#define CSL_SCI_SCIGCR1_SLEEP_SHIFT                                        (0x00000008U)
#define CSL_SCI_SCIGCR1_SLEEP_RESETVAL                                     (0x00000000U)
#define CSL_SCI_SCIGCR1_SLEEP_MAX                                          (0x00000001U)

#define CSL_SCI_SCIGCR1_POWERDOWN_MASK                                     (0x00000200U)
#define CSL_SCI_SCIGCR1_POWERDOWN_SHIFT                                    (0x00000009U)
#define CSL_SCI_SCIGCR1_POWERDOWN_RESETVAL                                 (0x00000000U)
#define CSL_SCI_SCIGCR1_POWERDOWN_MAX                                      (0x00000001U)

#define CSL_SCI_SCIGCR1_RESERVED2_MASK                                     (0x0000FC00U)
#define CSL_SCI_SCIGCR1_RESERVED2_SHIFT                                    (0x0000000AU)
#define CSL_SCI_SCIGCR1_RESERVED2_RESETVAL                                 (0x00000000U)
#define CSL_SCI_SCIGCR1_RESERVED2_MAX                                      (0x0000003FU)

#define CSL_SCI_SCIGCR1_LOOP_BACK_MASK                                     (0x00010000U)
#define CSL_SCI_SCIGCR1_LOOP_BACK_SHIFT                                    (0x00000010U)
#define CSL_SCI_SCIGCR1_LOOP_BACK_RESETVAL                                 (0x00000000U)
#define CSL_SCI_SCIGCR1_LOOP_BACK_MAX                                      (0x00000001U)

#define CSL_SCI_SCIGCR1_CONT_MASK                                          (0x00020000U)
#define CSL_SCI_SCIGCR1_CONT_SHIFT                                         (0x00000011U)
#define CSL_SCI_SCIGCR1_CONT_RESETVAL                                      (0x00000000U)
#define CSL_SCI_SCIGCR1_CONT_MAX                                           (0x00000001U)

#define CSL_SCI_SCIGCR1_RESERVED3_MASK                                     (0x00FC0000U)
#define CSL_SCI_SCIGCR1_RESERVED3_SHIFT                                    (0x00000012U)
#define CSL_SCI_SCIGCR1_RESERVED3_RESETVAL                                 (0x00000000U)
#define CSL_SCI_SCIGCR1_RESERVED3_MAX                                      (0x0000003FU)

#define CSL_SCI_SCIGCR1_RXENA_MASK                                         (0x01000000U)
#define CSL_SCI_SCIGCR1_RXENA_SHIFT                                        (0x00000018U)
#define CSL_SCI_SCIGCR1_RXENA_RESETVAL                                     (0x00000000U)
#define CSL_SCI_SCIGCR1_RXENA_MAX                                          (0x00000001U)

#define CSL_SCI_SCIGCR1_TXENA_MASK                                         (0x02000000U)
#define CSL_SCI_SCIGCR1_TXENA_SHIFT                                        (0x00000019U)
#define CSL_SCI_SCIGCR1_TXENA_RESETVAL                                     (0x00000000U)
#define CSL_SCI_SCIGCR1_TXENA_MAX                                          (0x00000001U)

#define CSL_SCI_SCIGCR1_RESERVED4_MASK                                     (0xFC000000U)
#define CSL_SCI_SCIGCR1_RESERVED4_SHIFT                                    (0x0000001AU)
#define CSL_SCI_SCIGCR1_RESERVED4_RESETVAL                                 (0x00000000U)
#define CSL_SCI_SCIGCR1_RESERVED4_MAX                                      (0x0000003FU)

#define CSL_SCI_SCIGCR1_RESETVAL                                           (0x00000000U)

/* RESERVED1 */

#define CSL_SCI_RESERVED1_RESERVED_MASK                                    (0xFFFFFFFFU)
#define CSL_SCI_RESERVED1_RESERVED_SHIFT                                   (0x00000000U)
#define CSL_SCI_RESERVED1_RESERVED_RESETVAL                                (0x00000000U)
#define CSL_SCI_RESERVED1_RESERVED_MAX                                     (0xFFFFFFFFU)

#define CSL_SCI_RESERVED1_RESETVAL                                         (0x00000000U)

/* SCISETINT */

#define CSL_SCI_SCISETINT_SET_BRKDT_INT_MASK                               (0x00000001U)
#define CSL_SCI_SCISETINT_SET_BRKDT_INT_SHIFT                              (0x00000000U)
#define CSL_SCI_SCISETINT_SET_BRKDT_INT_RESETVAL                           (0x00000000U)
#define CSL_SCI_SCISETINT_SET_BRKDT_INT_MAX                                (0x00000001U)

#define CSL_SCI_SCISETINT_SET_WAKEUP_INT_MASK                              (0x00000002U)
#define CSL_SCI_SCISETINT_SET_WAKEUP_INT_SHIFT                             (0x00000001U)
#define CSL_SCI_SCISETINT_SET_WAKEUP_INT_RESETVAL                          (0x00000000U)
#define CSL_SCI_SCISETINT_SET_WAKEUP_INT_MAX                               (0x00000001U)

#define CSL_SCI_SCISETINT_RESERVED1_MASK                                   (0x000000FCU)
#define CSL_SCI_SCISETINT_RESERVED1_SHIFT                                  (0x00000002U)
#define CSL_SCI_SCISETINT_RESERVED1_RESETVAL                               (0x00000000U)
#define CSL_SCI_SCISETINT_RESERVED1_MAX                                    (0x0000003FU)

#define CSL_SCI_SCISETINT_SET_TX_INT_MASK                                  (0x00000100U)
#define CSL_SCI_SCISETINT_SET_TX_INT_SHIFT                                 (0x00000008U)
#define CSL_SCI_SCISETINT_SET_TX_INT_RESETVAL                              (0x00000000U)
#define CSL_SCI_SCISETINT_SET_TX_INT_MAX                                   (0x00000001U)

#define CSL_SCI_SCISETINT_SET_RX_INT_MASK                                  (0x00000200U)
#define CSL_SCI_SCISETINT_SET_RX_INT_SHIFT                                 (0x00000009U)
#define CSL_SCI_SCISETINT_SET_RX_INT_RESETVAL                              (0x00000000U)
#define CSL_SCI_SCISETINT_SET_RX_INT_MAX                                   (0x00000001U)

#define CSL_SCI_SCISETINT_RESERVED2_MASK                                   (0x0000FC00U)
#define CSL_SCI_SCISETINT_RESERVED2_SHIFT                                  (0x0000000AU)
#define CSL_SCI_SCISETINT_RESERVED2_RESETVAL                               (0x00000000U)
#define CSL_SCI_SCISETINT_RESERVED2_MAX                                    (0x0000003FU)

#define CSL_SCI_SCISETINT_SET_TX_DMA_MASK                                  (0x00010000U)
#define CSL_SCI_SCISETINT_SET_TX_DMA_SHIFT                                 (0x00000010U)
#define CSL_SCI_SCISETINT_SET_TX_DMA_RESETVAL                              (0x00000000U)
#define CSL_SCI_SCISETINT_SET_TX_DMA_MAX                                   (0x00000001U)

#define CSL_SCI_SCISETINT_SET_RX_DMA_MASK                                  (0x00020000U)
#define CSL_SCI_SCISETINT_SET_RX_DMA_SHIFT                                 (0x00000011U)
#define CSL_SCI_SCISETINT_SET_RX_DMA_RESETVAL                              (0x00000000U)
#define CSL_SCI_SCISETINT_SET_RX_DMA_MAX                                   (0x00000001U)

#define CSL_SCI_SCISETINT_SET_RX_DMA_ALL_MASK                              (0x00040000U)
#define CSL_SCI_SCISETINT_SET_RX_DMA_ALL_SHIFT                             (0x00000012U)
#define CSL_SCI_SCISETINT_SET_RX_DMA_ALL_RESETVAL                          (0x00000000U)
#define CSL_SCI_SCISETINT_SET_RX_DMA_ALL_MAX                               (0x00000001U)

#define CSL_SCI_SCISETINT_RESERVED3_MASK                                   (0x00F80000U)
#define CSL_SCI_SCISETINT_RESERVED3_SHIFT                                  (0x00000013U)
#define CSL_SCI_SCISETINT_RESERVED3_RESETVAL                               (0x00000000U)
#define CSL_SCI_SCISETINT_RESERVED3_MAX                                    (0x0000001FU)

#define CSL_SCI_SCISETINT_SET_PE_INT_MASK                                  (0x01000000U)
#define CSL_SCI_SCISETINT_SET_PE_INT_SHIFT                                 (0x00000018U)
#define CSL_SCI_SCISETINT_SET_PE_INT_RESETVAL                              (0x00000000U)
#define CSL_SCI_SCISETINT_SET_PE_INT_MAX                                   (0x00000001U)

#define CSL_SCI_SCISETINT_SET_OE_INT_MASK                                  (0x02000000U)
#define CSL_SCI_SCISETINT_SET_OE_INT_SHIFT                                 (0x00000019U)
#define CSL_SCI_SCISETINT_SET_OE_INT_RESETVAL                              (0x00000000U)
#define CSL_SCI_SCISETINT_SET_OE_INT_MAX                                   (0x00000001U)

#define CSL_SCI_SCISETINT_SET_FE_INT_MASK                                  (0x04000000U)
#define CSL_SCI_SCISETINT_SET_FE_INT_SHIFT                                 (0x0000001AU)
#define CSL_SCI_SCISETINT_SET_FE_INT_RESETVAL                              (0x00000000U)
#define CSL_SCI_SCISETINT_SET_FE_INT_MAX                                   (0x00000001U)

#define CSL_SCI_SCISETINT_RESERVED4_MASK                                   (0xF8000000U)
#define CSL_SCI_SCISETINT_RESERVED4_SHIFT                                  (0x0000001BU)
#define CSL_SCI_SCISETINT_RESERVED4_RESETVAL                               (0x00000000U)
#define CSL_SCI_SCISETINT_RESERVED4_MAX                                    (0x0000001FU)

#define CSL_SCI_SCISETINT_RESETVAL                                         (0x00000000U)

/* SCICLEARINT */

#define CSL_SCI_SCICLEARINT_CLR_BRKDT_INT_MASK                             (0x00000001U)
#define CSL_SCI_SCICLEARINT_CLR_BRKDT_INT_SHIFT                            (0x00000000U)
#define CSL_SCI_SCICLEARINT_CLR_BRKDT_INT_RESETVAL                         (0x00000000U)
#define CSL_SCI_SCICLEARINT_CLR_BRKDT_INT_MAX                              (0x00000001U)

#define CSL_SCI_SCICLEARINT_CLR_WAKEUP_INT_MASK                            (0x00000002U)
#define CSL_SCI_SCICLEARINT_CLR_WAKEUP_INT_SHIFT                           (0x00000001U)
#define CSL_SCI_SCICLEARINT_CLR_WAKEUP_INT_RESETVAL                        (0x00000000U)
#define CSL_SCI_SCICLEARINT_CLR_WAKEUP_INT_MAX                             (0x00000001U)

#define CSL_SCI_SCICLEARINT_RESERVED1_MASK                                 (0x000000FCU)
#define CSL_SCI_SCICLEARINT_RESERVED1_SHIFT                                (0x00000002U)
#define CSL_SCI_SCICLEARINT_RESERVED1_RESETVAL                             (0x00000000U)
#define CSL_SCI_SCICLEARINT_RESERVED1_MAX                                  (0x0000003FU)

#define CSL_SCI_SCICLEARINT_CLR_TX_INT_MASK                                (0x00000100U)
#define CSL_SCI_SCICLEARINT_CLR_TX_INT_SHIFT                               (0x00000008U)
#define CSL_SCI_SCICLEARINT_CLR_TX_INT_RESETVAL                            (0x00000000U)
#define CSL_SCI_SCICLEARINT_CLR_TX_INT_MAX                                 (0x00000001U)

#define CSL_SCI_SCICLEARINT_CLR_RX_INT_MASK                                (0x00000200U)
#define CSL_SCI_SCICLEARINT_CLR_RX_INT_SHIFT                               (0x00000009U)
#define CSL_SCI_SCICLEARINT_CLR_RX_INT_RESETVAL                            (0x00000000U)
#define CSL_SCI_SCICLEARINT_CLR_RX_INT_MAX                                 (0x00000001U)

#define CSL_SCI_SCICLEARINT_RESERVED2_MASK                                 (0x0000FC00U)
#define CSL_SCI_SCICLEARINT_RESERVED2_SHIFT                                (0x0000000AU)
#define CSL_SCI_SCICLEARINT_RESERVED2_RESETVAL                             (0x00000000U)
#define CSL_SCI_SCICLEARINT_RESERVED2_MAX                                  (0x0000003FU)

#define CSL_SCI_SCICLEARINT_CLR_TX_DMA_MASK                                (0x00010000U)
#define CSL_SCI_SCICLEARINT_CLR_TX_DMA_SHIFT                               (0x00000010U)
#define CSL_SCI_SCICLEARINT_CLR_TX_DMA_RESETVAL                            (0x00000000U)
#define CSL_SCI_SCICLEARINT_CLR_TX_DMA_MAX                                 (0x00000001U)

#define CSL_SCI_SCICLEARINT_CLR_RX_DMA_MASK                                (0x00020000U)
#define CSL_SCI_SCICLEARINT_CLR_RX_DMA_SHIFT                               (0x00000011U)
#define CSL_SCI_SCICLEARINT_CLR_RX_DMA_RESETVAL                            (0x00000000U)
#define CSL_SCI_SCICLEARINT_CLR_RX_DMA_MAX                                 (0x00000001U)

#define CSL_SCI_SCICLEARINT_CLR_RX_DMA_ALL_MASK                            (0x00040000U)
#define CSL_SCI_SCICLEARINT_CLR_RX_DMA_ALL_SHIFT                           (0x00000012U)
#define CSL_SCI_SCICLEARINT_CLR_RX_DMA_ALL_RESETVAL                        (0x00000000U)
#define CSL_SCI_SCICLEARINT_CLR_RX_DMA_ALL_MAX                             (0x00000001U)

#define CSL_SCI_SCICLEARINT_RESERVED3_MASK                                 (0x00F80000U)
#define CSL_SCI_SCICLEARINT_RESERVED3_SHIFT                                (0x00000013U)
#define CSL_SCI_SCICLEARINT_RESERVED3_RESETVAL                             (0x00000000U)
#define CSL_SCI_SCICLEARINT_RESERVED3_MAX                                  (0x0000001FU)

#define CSL_SCI_SCICLEARINT_CLR_PE_INT_MASK                                (0x01000000U)
#define CSL_SCI_SCICLEARINT_CLR_PE_INT_SHIFT                               (0x00000018U)
#define CSL_SCI_SCICLEARINT_CLR_PE_INT_RESETVAL                            (0x00000000U)
#define CSL_SCI_SCICLEARINT_CLR_PE_INT_MAX                                 (0x00000001U)

#define CSL_SCI_SCICLEARINT_CLR_OE_INT_MASK                                (0x02000000U)
#define CSL_SCI_SCICLEARINT_CLR_OE_INT_SHIFT                               (0x00000019U)
#define CSL_SCI_SCICLEARINT_CLR_OE_INT_RESETVAL                            (0x00000000U)
#define CSL_SCI_SCICLEARINT_CLR_OE_INT_MAX                                 (0x00000001U)

#define CSL_SCI_SCICLEARINT_CLR_FE_INT_MASK                                (0x04000000U)
#define CSL_SCI_SCICLEARINT_CLR_FE_INT_SHIFT                               (0x0000001AU)
#define CSL_SCI_SCICLEARINT_CLR_FE_INT_RESETVAL                            (0x00000000U)
#define CSL_SCI_SCICLEARINT_CLR_FE_INT_MAX                                 (0x00000001U)

#define CSL_SCI_SCICLEARINT_RESERVED4_MASK                                 (0xF8000000U)
#define CSL_SCI_SCICLEARINT_RESERVED4_SHIFT                                (0x0000001BU)
#define CSL_SCI_SCICLEARINT_RESERVED4_RESETVAL                             (0x00000000U)
#define CSL_SCI_SCICLEARINT_RESERVED4_MAX                                  (0x0000001FU)

#define CSL_SCI_SCICLEARINT_RESETVAL                                       (0x00000000U)

/* SCISETINTLVL */

#define CSL_SCI_SCISETINTLVL_SET_BRKDT_INT_LVL_MASK                        (0x00000001U)
#define CSL_SCI_SCISETINTLVL_SET_BRKDT_INT_LVL_SHIFT                       (0x00000000U)
#define CSL_SCI_SCISETINTLVL_SET_BRKDT_INT_LVL_RESETVAL                    (0x00000000U)
#define CSL_SCI_SCISETINTLVL_SET_BRKDT_INT_LVL_MAX                         (0x00000001U)

#define CSL_SCI_SCISETINTLVL_SET_WAKEUP_INT_LVL_MASK                       (0x00000002U)
#define CSL_SCI_SCISETINTLVL_SET_WAKEUP_INT_LVL_SHIFT                      (0x00000001U)
#define CSL_SCI_SCISETINTLVL_SET_WAKEUP_INT_LVL_RESETVAL                   (0x00000000U)
#define CSL_SCI_SCISETINTLVL_SET_WAKEUP_INT_LVL_MAX                        (0x00000001U)

#define CSL_SCI_SCISETINTLVL_RESERVED1_MASK                                (0x000000FCU)
#define CSL_SCI_SCISETINTLVL_RESERVED1_SHIFT                               (0x00000002U)
#define CSL_SCI_SCISETINTLVL_RESERVED1_RESETVAL                            (0x00000000U)
#define CSL_SCI_SCISETINTLVL_RESERVED1_MAX                                 (0x0000003FU)

#define CSL_SCI_SCISETINTLVL_SET_TX_INT_LVL_MASK                           (0x00000100U)
#define CSL_SCI_SCISETINTLVL_SET_TX_INT_LVL_SHIFT                          (0x00000008U)
#define CSL_SCI_SCISETINTLVL_SET_TX_INT_LVL_RESETVAL                       (0x00000000U)
#define CSL_SCI_SCISETINTLVL_SET_TX_INT_LVL_MAX                            (0x00000001U)

#define CSL_SCI_SCISETINTLVL_SET_RX_INT_LVL_MASK                           (0x00000200U)
#define CSL_SCI_SCISETINTLVL_SET_RX_INT_LVL_SHIFT                          (0x00000009U)
#define CSL_SCI_SCISETINTLVL_SET_RX_INT_LVL_RESETVAL                       (0x00000000U)
#define CSL_SCI_SCISETINTLVL_SET_RX_INT_LVL_MAX                            (0x00000001U)

#define CSL_SCI_SCISETINTLVL_RESERVED2_MASK                                (0x00007C00U)
#define CSL_SCI_SCISETINTLVL_RESERVED2_SHIFT                               (0x0000000AU)
#define CSL_SCI_SCISETINTLVL_RESERVED2_RESETVAL                            (0x00000000U)
#define CSL_SCI_SCISETINTLVL_RESERVED2_MAX                                 (0x0000001FU)

#define CSL_SCI_SCISETINTLVL_SET_INC_BR_INT_LVL_MASK                       (0x00008000U)
#define CSL_SCI_SCISETINTLVL_SET_INC_BR_INT_LVL_SHIFT                      (0x0000000FU)
#define CSL_SCI_SCISETINTLVL_SET_INC_BR_INT_LVL_RESETVAL                   (0x00000000U)
#define CSL_SCI_SCISETINTLVL_SET_INC_BR_INT_LVL_MAX                        (0x00000001U)

#define CSL_SCI_SCISETINTLVL_RESERVED3_MASK                                (0x00030000U)
#define CSL_SCI_SCISETINTLVL_RESERVED3_SHIFT                               (0x00000010U)
#define CSL_SCI_SCISETINTLVL_RESERVED3_RESETVAL                            (0x00000000U)
#define CSL_SCI_SCISETINTLVL_RESERVED3_MAX                                 (0x00000003U)

#define CSL_SCI_SCISETINTLVL_SET_RX_DMA_ALL_INT_LVL_MASK                   (0x00040000U)
#define CSL_SCI_SCISETINTLVL_SET_RX_DMA_ALL_INT_LVL_SHIFT                  (0x00000012U)
#define CSL_SCI_SCISETINTLVL_SET_RX_DMA_ALL_INT_LVL_RESETVAL               (0x00000000U)
#define CSL_SCI_SCISETINTLVL_SET_RX_DMA_ALL_INT_LVL_MAX                    (0x00000001U)

#define CSL_SCI_SCISETINTLVL_RESERVED4_MASK                                (0x00F80000U)
#define CSL_SCI_SCISETINTLVL_RESERVED4_SHIFT                               (0x00000013U)
#define CSL_SCI_SCISETINTLVL_RESERVED4_RESETVAL                            (0x00000000U)
#define CSL_SCI_SCISETINTLVL_RESERVED4_MAX                                 (0x0000001FU)

#define CSL_SCI_SCISETINTLVL_SET_PE_INT_LVL_MASK                           (0x01000000U)
#define CSL_SCI_SCISETINTLVL_SET_PE_INT_LVL_SHIFT                          (0x00000018U)
#define CSL_SCI_SCISETINTLVL_SET_PE_INT_LVL_RESETVAL                       (0x00000000U)
#define CSL_SCI_SCISETINTLVL_SET_PE_INT_LVL_MAX                            (0x00000001U)

#define CSL_SCI_SCISETINTLVL_SET_OE_INT_LVL_MASK                           (0x02000000U)
#define CSL_SCI_SCISETINTLVL_SET_OE_INT_LVL_SHIFT                          (0x00000019U)
#define CSL_SCI_SCISETINTLVL_SET_OE_INT_LVL_RESETVAL                       (0x00000000U)
#define CSL_SCI_SCISETINTLVL_SET_OE_INT_LVL_MAX                            (0x00000001U)

#define CSL_SCI_SCISETINTLVL_SET_FE_INT_LVL_MASK                           (0x04000000U)
#define CSL_SCI_SCISETINTLVL_SET_FE_INT_LVL_SHIFT                          (0x0000001AU)
#define CSL_SCI_SCISETINTLVL_SET_FE_INT_LVL_RESETVAL                       (0x00000000U)
#define CSL_SCI_SCISETINTLVL_SET_FE_INT_LVL_MAX                            (0x00000001U)

#define CSL_SCI_SCISETINTLVL_RESERVED5_MASK                                (0xF8000000U)
#define CSL_SCI_SCISETINTLVL_RESERVED5_SHIFT                               (0x0000001BU)
#define CSL_SCI_SCISETINTLVL_RESERVED5_RESETVAL                            (0x00000000U)
#define CSL_SCI_SCISETINTLVL_RESERVED5_MAX                                 (0x0000001FU)

#define CSL_SCI_SCISETINTLVL_RESETVAL                                      (0x00000000U)

/* SCICLEARINTLVL */

#define CSL_SCI_SCICLEARINTLVL_CLR_BRKDT_INT_LVL_MASK                      (0x00000001U)
#define CSL_SCI_SCICLEARINTLVL_CLR_BRKDT_INT_LVL_SHIFT                     (0x00000000U)
#define CSL_SCI_SCICLEARINTLVL_CLR_BRKDT_INT_LVL_RESETVAL                  (0x00000000U)
#define CSL_SCI_SCICLEARINTLVL_CLR_BRKDT_INT_LVL_MAX                       (0x00000001U)

#define CSL_SCI_SCICLEARINTLVL_CLR_WAKEUP_INT_LVL_MASK                     (0x00000002U)
#define CSL_SCI_SCICLEARINTLVL_CLR_WAKEUP_INT_LVL_SHIFT                    (0x00000001U)
#define CSL_SCI_SCICLEARINTLVL_CLR_WAKEUP_INT_LVL_RESETVAL                 (0x00000000U)
#define CSL_SCI_SCICLEARINTLVL_CLR_WAKEUP_INT_LVL_MAX                      (0x00000001U)

#define CSL_SCI_SCICLEARINTLVL_RESERVED1_MASK                              (0x000000FCU)
#define CSL_SCI_SCICLEARINTLVL_RESERVED1_SHIFT                             (0x00000002U)
#define CSL_SCI_SCICLEARINTLVL_RESERVED1_RESETVAL                          (0x00000000U)
#define CSL_SCI_SCICLEARINTLVL_RESERVED1_MAX                               (0x0000003FU)

#define CSL_SCI_SCICLEARINTLVL_CLR_TX_INT_LVL_MASK                         (0x00000100U)
#define CSL_SCI_SCICLEARINTLVL_CLR_TX_INT_LVL_SHIFT                        (0x00000008U)
#define CSL_SCI_SCICLEARINTLVL_CLR_TX_INT_LVL_RESETVAL                     (0x00000000U)
#define CSL_SCI_SCICLEARINTLVL_CLR_TX_INT_LVL_MAX                          (0x00000001U)

#define CSL_SCI_SCICLEARINTLVL_CLR_RX_INT_LVL_MASK                         (0x00000200U)
#define CSL_SCI_SCICLEARINTLVL_CLR_RX_INT_LVL_SHIFT                        (0x00000009U)
#define CSL_SCI_SCICLEARINTLVL_CLR_RX_INT_LVL_RESETVAL                     (0x00000000U)
#define CSL_SCI_SCICLEARINTLVL_CLR_RX_INT_LVL_MAX                          (0x00000001U)

#define CSL_SCI_SCICLEARINTLVL_RESERVED2_MASK                              (0x00007C00U)
#define CSL_SCI_SCICLEARINTLVL_RESERVED2_SHIFT                             (0x0000000AU)
#define CSL_SCI_SCICLEARINTLVL_RESERVED2_RESETVAL                          (0x00000000U)
#define CSL_SCI_SCICLEARINTLVL_RESERVED2_MAX                               (0x0000001FU)

#define CSL_SCI_SCICLEARINTLVL_CLR_INC_BR_INT_LVL_MASK                     (0x00008000U)
#define CSL_SCI_SCICLEARINTLVL_CLR_INC_BR_INT_LVL_SHIFT                    (0x0000000FU)
#define CSL_SCI_SCICLEARINTLVL_CLR_INC_BR_INT_LVL_RESETVAL                 (0x00000000U)
#define CSL_SCI_SCICLEARINTLVL_CLR_INC_BR_INT_LVL_MAX                      (0x00000001U)

#define CSL_SCI_SCICLEARINTLVL_RESERVED3_MASK                              (0x00030000U)
#define CSL_SCI_SCICLEARINTLVL_RESERVED3_SHIFT                             (0x00000010U)
#define CSL_SCI_SCICLEARINTLVL_RESERVED3_RESETVAL                          (0x00000000U)
#define CSL_SCI_SCICLEARINTLVL_RESERVED3_MAX                               (0x00000003U)

#define CSL_SCI_SCICLEARINTLVL_CLR_RX_DMA_ALL_INT_LVL_MASK                 (0x00040000U)
#define CSL_SCI_SCICLEARINTLVL_CLR_RX_DMA_ALL_INT_LVL_SHIFT                (0x00000012U)
#define CSL_SCI_SCICLEARINTLVL_CLR_RX_DMA_ALL_INT_LVL_RESETVAL             (0x00000000U)
#define CSL_SCI_SCICLEARINTLVL_CLR_RX_DMA_ALL_INT_LVL_MAX                  (0x00000001U)

#define CSL_SCI_SCICLEARINTLVL_RESERVED4_MASK                              (0x00F80000U)
#define CSL_SCI_SCICLEARINTLVL_RESERVED4_SHIFT                             (0x00000013U)
#define CSL_SCI_SCICLEARINTLVL_RESERVED4_RESETVAL                          (0x00000000U)
#define CSL_SCI_SCICLEARINTLVL_RESERVED4_MAX                               (0x0000001FU)

#define CSL_SCI_SCICLEARINTLVL_CLR_PE_INT_LVL_MASK                         (0x01000000U)
#define CSL_SCI_SCICLEARINTLVL_CLR_PE_INT_LVL_SHIFT                        (0x00000018U)
#define CSL_SCI_SCICLEARINTLVL_CLR_PE_INT_LVL_RESETVAL                     (0x00000000U)
#define CSL_SCI_SCICLEARINTLVL_CLR_PE_INT_LVL_MAX                          (0x00000001U)

#define CSL_SCI_SCICLEARINTLVL_CLR_OE_INT_LVL_MASK                         (0x02000000U)
#define CSL_SCI_SCICLEARINTLVL_CLR_OE_INT_LVL_SHIFT                        (0x00000019U)
#define CSL_SCI_SCICLEARINTLVL_CLR_OE_INT_LVL_RESETVAL                     (0x00000000U)
#define CSL_SCI_SCICLEARINTLVL_CLR_OE_INT_LVL_MAX                          (0x00000001U)

#define CSL_SCI_SCICLEARINTLVL_CLR_FE_INT_LVL_MASK                         (0x04000000U)
#define CSL_SCI_SCICLEARINTLVL_CLR_FE_INT_LVL_SHIFT                        (0x0000001AU)
#define CSL_SCI_SCICLEARINTLVL_CLR_FE_INT_LVL_RESETVAL                     (0x00000000U)
#define CSL_SCI_SCICLEARINTLVL_CLR_FE_INT_LVL_MAX                          (0x00000001U)

#define CSL_SCI_SCICLEARINTLVL_RESERVED5_MASK                              (0xF8000000U)
#define CSL_SCI_SCICLEARINTLVL_RESERVED5_SHIFT                             (0x0000001BU)
#define CSL_SCI_SCICLEARINTLVL_RESERVED5_RESETVAL                          (0x00000000U)
#define CSL_SCI_SCICLEARINTLVL_RESERVED5_MAX                               (0x0000001FU)

#define CSL_SCI_SCICLEARINTLVL_RESETVAL                                    (0x00000000U)

/* SCIFLR */

#define CSL_SCI_SCIFLR_BRKDT_MASK                                          (0x00000001U)
#define CSL_SCI_SCIFLR_BRKDT_SHIFT                                         (0x00000000U)
#define CSL_SCI_SCIFLR_BRKDT_RESETVAL                                      (0x00000000U)
#define CSL_SCI_SCIFLR_BRKDT_MAX                                           (0x00000001U)

#define CSL_SCI_SCIFLR_WAKEUP_MASK                                         (0x00000002U)
#define CSL_SCI_SCIFLR_WAKEUP_SHIFT                                        (0x00000001U)
#define CSL_SCI_SCIFLR_WAKEUP_RESETVAL                                     (0x00000000U)
#define CSL_SCI_SCIFLR_WAKEUP_MAX                                          (0x00000001U)

#define CSL_SCI_SCIFLR_IDLE_MASK                                           (0x00000004U)
#define CSL_SCI_SCIFLR_IDLE_SHIFT                                          (0x00000002U)
#define CSL_SCI_SCIFLR_IDLE_RESETVAL                                       (0x00000001U)
#define CSL_SCI_SCIFLR_IDLE_MAX                                            (0x00000001U)

#define CSL_SCI_SCIFLR_BUS_BUSY_FLAG_MASK                                  (0x00000008U)
#define CSL_SCI_SCIFLR_BUS_BUSY_FLAG_SHIFT                                 (0x00000003U)
#define CSL_SCI_SCIFLR_BUS_BUSY_FLAG_RESETVAL                              (0x00000000U)
#define CSL_SCI_SCIFLR_BUS_BUSY_FLAG_MAX                                   (0x00000001U)

#define CSL_SCI_SCIFLR_RESERVED1_MASK                                      (0x000000F0U)
#define CSL_SCI_SCIFLR_RESERVED1_SHIFT                                     (0x00000004U)
#define CSL_SCI_SCIFLR_RESERVED1_RESETVAL                                  (0x00000000U)
#define CSL_SCI_SCIFLR_RESERVED1_MAX                                       (0x0000000FU)

#define CSL_SCI_SCIFLR_TXRDY_MASK                                          (0x00000100U)
#define CSL_SCI_SCIFLR_TXRDY_SHIFT                                         (0x00000008U)
#define CSL_SCI_SCIFLR_TXRDY_RESETVAL                                      (0x00000001U)
#define CSL_SCI_SCIFLR_TXRDY_MAX                                           (0x00000001U)

#define CSL_SCI_SCIFLR_RXRDY_MASK                                          (0x00000200U)
#define CSL_SCI_SCIFLR_RXRDY_SHIFT                                         (0x00000009U)
#define CSL_SCI_SCIFLR_RXRDY_RESETVAL                                      (0x00000000U)
#define CSL_SCI_SCIFLR_RXRDY_MAX                                           (0x00000001U)

#define CSL_SCI_SCIFLR_TXWAKE_MASK                                         (0x00000400U)
#define CSL_SCI_SCIFLR_TXWAKE_SHIFT                                        (0x0000000AU)
#define CSL_SCI_SCIFLR_TXWAKE_RESETVAL                                     (0x00000000U)
#define CSL_SCI_SCIFLR_TXWAKE_MAX                                          (0x00000001U)

#define CSL_SCI_SCIFLR_TX_EMPTY_MASK                                       (0x00000800U)
#define CSL_SCI_SCIFLR_TX_EMPTY_SHIFT                                      (0x0000000BU)
#define CSL_SCI_SCIFLR_TX_EMPTY_RESETVAL                                   (0x00000001U)
#define CSL_SCI_SCIFLR_TX_EMPTY_MAX                                        (0x00000001U)

#define CSL_SCI_SCIFLR_RXWAKE_MASK                                         (0x00001000U)
#define CSL_SCI_SCIFLR_RXWAKE_SHIFT                                        (0x0000000CU)
#define CSL_SCI_SCIFLR_RXWAKE_RESETVAL                                     (0x00000000U)
#define CSL_SCI_SCIFLR_RXWAKE_MAX                                          (0x00000001U)

#define CSL_SCI_SCIFLR_RESERVED2_MASK                                      (0x00FFE000U)
#define CSL_SCI_SCIFLR_RESERVED2_SHIFT                                     (0x0000000DU)
#define CSL_SCI_SCIFLR_RESERVED2_RESETVAL                                  (0x00000000U)
#define CSL_SCI_SCIFLR_RESERVED2_MAX                                       (0x000007FFU)

#define CSL_SCI_SCIFLR_PE_MASK                                             (0x01000000U)
#define CSL_SCI_SCIFLR_PE_SHIFT                                            (0x00000018U)
#define CSL_SCI_SCIFLR_PE_RESETVAL                                         (0x00000000U)
#define CSL_SCI_SCIFLR_PE_MAX                                              (0x00000001U)

#define CSL_SCI_SCIFLR_OE_MASK                                             (0x02000000U)
#define CSL_SCI_SCIFLR_OE_SHIFT                                            (0x00000019U)
#define CSL_SCI_SCIFLR_OE_RESETVAL                                         (0x00000000U)
#define CSL_SCI_SCIFLR_OE_MAX                                              (0x00000001U)

#define CSL_SCI_SCIFLR_FE_MASK                                             (0x04000000U)
#define CSL_SCI_SCIFLR_FE_SHIFT                                            (0x0000001AU)
#define CSL_SCI_SCIFLR_FE_RESETVAL                                         (0x00000000U)
#define CSL_SCI_SCIFLR_FE_MAX                                              (0x00000001U)

#define CSL_SCI_SCIFLR_RESERVED3_MASK                                      (0xF8000000U)
#define CSL_SCI_SCIFLR_RESERVED3_SHIFT                                     (0x0000001BU)
#define CSL_SCI_SCIFLR_RESERVED3_RESETVAL                                  (0x00000000U)
#define CSL_SCI_SCIFLR_RESERVED3_MAX                                       (0x0000001FU)

#define CSL_SCI_SCIFLR_RESETVAL                                            (0x00000904U)

/* SCIINTVECT0 */

#define CSL_SCI_SCIINTVECT0_INTVECT0_MASK                                  (0x0000000FU)
#define CSL_SCI_SCIINTVECT0_INTVECT0_SHIFT                                 (0x00000000U)
#define CSL_SCI_SCIINTVECT0_INTVECT0_RESETVAL                              (0x00000000U)
#define CSL_SCI_SCIINTVECT0_INTVECT0_MAX                                   (0x0000000FU)

#define CSL_SCI_SCIINTVECT0_RESERVED_MASK                                  (0xFFFFFFF0U)
#define CSL_SCI_SCIINTVECT0_RESERVED_SHIFT                                 (0x00000004U)
#define CSL_SCI_SCIINTVECT0_RESERVED_RESETVAL                              (0x00000000U)
#define CSL_SCI_SCIINTVECT0_RESERVED_MAX                                   (0x0FFFFFFFU)

#define CSL_SCI_SCIINTVECT0_RESETVAL                                       (0x00000000U)

/* SCIINTVECT1 */

#define CSL_SCI_SCIINTVECT1_INTVECT1_MASK                                  (0x0000000FU)
#define CSL_SCI_SCIINTVECT1_INTVECT1_SHIFT                                 (0x00000000U)
#define CSL_SCI_SCIINTVECT1_INTVECT1_RESETVAL                              (0x00000000U)
#define CSL_SCI_SCIINTVECT1_INTVECT1_MAX                                   (0x0000000FU)

#define CSL_SCI_SCIINTVECT1_RESERVED_MASK                                  (0xFFFFFFF0U)
#define CSL_SCI_SCIINTVECT1_RESERVED_SHIFT                                 (0x00000004U)
#define CSL_SCI_SCIINTVECT1_RESERVED_RESETVAL                              (0x00000000U)
#define CSL_SCI_SCIINTVECT1_RESERVED_MAX                                   (0x0FFFFFFFU)

#define CSL_SCI_SCIINTVECT1_RESETVAL                                       (0x00000000U)

/* SCICHAR */

#define CSL_SCI_SCICHAR_CHAR_MASK                                          (0x00000007U)
#define CSL_SCI_SCICHAR_CHAR_SHIFT                                         (0x00000000U)
#define CSL_SCI_SCICHAR_CHAR_RESETVAL                                      (0x00000000U)
#define CSL_SCI_SCICHAR_CHAR_MAX                                           (0x00000007U)

#define CSL_SCI_SCICHAR_RESERVED_MASK                                      (0xFFFFFFF8U)
#define CSL_SCI_SCICHAR_RESERVED_SHIFT                                     (0x00000003U)
#define CSL_SCI_SCICHAR_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_SCI_SCICHAR_RESERVED_MAX                                       (0x1FFFFFFFU)

#define CSL_SCI_SCICHAR_RESETVAL                                           (0x00000000U)

/* SCIBAUD */

#define CSL_SCI_SCIBAUD_BAUD_MASK                                          (0x00FFFFFFU)
#define CSL_SCI_SCIBAUD_BAUD_SHIFT                                         (0x00000000U)
#define CSL_SCI_SCIBAUD_BAUD_RESETVAL                                      (0x00000000U)
#define CSL_SCI_SCIBAUD_BAUD_MAX                                           (0x00FFFFFFU)

#define CSL_SCI_SCIBAUD_RESERVED_MASK                                      (0xFF000000U)
#define CSL_SCI_SCIBAUD_RESERVED_SHIFT                                     (0x00000018U)
#define CSL_SCI_SCIBAUD_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_SCI_SCIBAUD_RESERVED_MAX                                       (0x000000FFU)

#define CSL_SCI_SCIBAUD_RESETVAL                                           (0x00000000U)

/* SCIED */

#define CSL_SCI_SCIED_ED_MASK                                              (0x000000FFU)
#define CSL_SCI_SCIED_ED_SHIFT                                             (0x00000000U)
#define CSL_SCI_SCIED_ED_RESETVAL                                          (0x00000000U)
#define CSL_SCI_SCIED_ED_MAX                                               (0x000000FFU)

#define CSL_SCI_SCIED_RESERVED_MASK                                        (0xFFFFFF00U)
#define CSL_SCI_SCIED_RESERVED_SHIFT                                       (0x00000008U)
#define CSL_SCI_SCIED_RESERVED_RESETVAL                                    (0x00000000U)
#define CSL_SCI_SCIED_RESERVED_MAX                                         (0x00FFFFFFU)

#define CSL_SCI_SCIED_RESETVAL                                             (0x00000000U)

/* SCIRD */

#define CSL_SCI_SCIRD_RD_MASK                                              (0x000000FFU)
#define CSL_SCI_SCIRD_RD_SHIFT                                             (0x00000000U)
#define CSL_SCI_SCIRD_RD_RESETVAL                                          (0x00000000U)
#define CSL_SCI_SCIRD_RD_MAX                                               (0x000000FFU)

#define CSL_SCI_SCIRD_RESERVED_MASK                                        (0xFFFFFF00U)
#define CSL_SCI_SCIRD_RESERVED_SHIFT                                       (0x00000008U)
#define CSL_SCI_SCIRD_RESERVED_RESETVAL                                    (0x00000000U)
#define CSL_SCI_SCIRD_RESERVED_MAX                                         (0x00FFFFFFU)

#define CSL_SCI_SCIRD_RESETVAL                                             (0x00000000U)

/* SCITD */

#define CSL_SCI_SCITD_TD_MASK                                              (0x000000FFU)
#define CSL_SCI_SCITD_TD_SHIFT                                             (0x00000000U)
#define CSL_SCI_SCITD_TD_RESETVAL                                          (0x00000000U)
#define CSL_SCI_SCITD_TD_MAX                                               (0x000000FFU)

#define CSL_SCI_SCITD_RESERVED_MASK                                        (0xFFFFFF00U)
#define CSL_SCI_SCITD_RESERVED_SHIFT                                       (0x00000008U)
#define CSL_SCI_SCITD_RESERVED_RESETVAL                                    (0x00000000U)
#define CSL_SCI_SCITD_RESERVED_MAX                                         (0x00FFFFFFU)

#define CSL_SCI_SCITD_RESETVAL                                             (0x00000000U)

/* SCIPIO0 */

#define CSL_SCI_SCIPIO0_CLK_FUNC_MASK                                      (0x00000001U)
#define CSL_SCI_SCIPIO0_CLK_FUNC_SHIFT                                     (0x00000000U)
#define CSL_SCI_SCIPIO0_CLK_FUNC_RESETVAL                                  (0x00000000U)
#define CSL_SCI_SCIPIO0_CLK_FUNC_MAX                                       (0x00000001U)

#define CSL_SCI_SCIPIO0_RX_FUNC_MASK                                       (0x00000002U)
#define CSL_SCI_SCIPIO0_RX_FUNC_SHIFT                                      (0x00000001U)
#define CSL_SCI_SCIPIO0_RX_FUNC_RESETVAL                                   (0x00000000U)
#define CSL_SCI_SCIPIO0_RX_FUNC_MAX                                        (0x00000001U)

#define CSL_SCI_SCIPIO0_TX_FUNC_MASK                                       (0x00000004U)
#define CSL_SCI_SCIPIO0_TX_FUNC_SHIFT                                      (0x00000002U)
#define CSL_SCI_SCIPIO0_TX_FUNC_RESETVAL                                   (0x00000000U)
#define CSL_SCI_SCIPIO0_TX_FUNC_MAX                                        (0x00000001U)

#define CSL_SCI_SCIPIO0_RESERVED_MASK                                      (0xFFFFFFF8U)
#define CSL_SCI_SCIPIO0_RESERVED_SHIFT                                     (0x00000003U)
#define CSL_SCI_SCIPIO0_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_SCI_SCIPIO0_RESERVED_MAX                                       (0x1FFFFFFFU)

#define CSL_SCI_SCIPIO0_RESETVAL                                           (0x00000000U)

/* SCIPIO1 */

#define CSL_SCI_SCIPIO1_CLK_DIR_MASK                                       (0x00000001U)
#define CSL_SCI_SCIPIO1_CLK_DIR_SHIFT                                      (0x00000000U)
#define CSL_SCI_SCIPIO1_CLK_DIR_RESETVAL                                   (0x00000000U)
#define CSL_SCI_SCIPIO1_CLK_DIR_MAX                                        (0x00000001U)

#define CSL_SCI_SCIPIO1_RX_DIR_MASK                                        (0x00000002U)
#define CSL_SCI_SCIPIO1_RX_DIR_SHIFT                                       (0x00000001U)
#define CSL_SCI_SCIPIO1_RX_DIR_RESETVAL                                    (0x00000000U)
#define CSL_SCI_SCIPIO1_RX_DIR_MAX                                         (0x00000001U)

#define CSL_SCI_SCIPIO1_TX_DIR_MASK                                        (0x00000004U)
#define CSL_SCI_SCIPIO1_TX_DIR_SHIFT                                       (0x00000002U)
#define CSL_SCI_SCIPIO1_TX_DIR_RESETVAL                                    (0x00000000U)
#define CSL_SCI_SCIPIO1_TX_DIR_MAX                                         (0x00000001U)

#define CSL_SCI_SCIPIO1_RESERVED_MASK                                      (0xFFFFFFF8U)
#define CSL_SCI_SCIPIO1_RESERVED_SHIFT                                     (0x00000003U)
#define CSL_SCI_SCIPIO1_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_SCI_SCIPIO1_RESERVED_MAX                                       (0x1FFFFFFFU)

#define CSL_SCI_SCIPIO1_RESETVAL                                           (0x00000000U)

/* SCIPIO2 */

#define CSL_SCI_SCIPIO2_CLK_DATA_IN_MASK                                   (0x00000001U)
#define CSL_SCI_SCIPIO2_CLK_DATA_IN_SHIFT                                  (0x00000000U)
#define CSL_SCI_SCIPIO2_CLK_DATA_IN_RESETVAL                               (0x00000000U)
#define CSL_SCI_SCIPIO2_CLK_DATA_IN_MAX                                    (0x00000001U)

#define CSL_SCI_SCIPIO2_RX_DATA_IN_MASK                                    (0x00000002U)
#define CSL_SCI_SCIPIO2_RX_DATA_IN_SHIFT                                   (0x00000001U)
#define CSL_SCI_SCIPIO2_RX_DATA_IN_RESETVAL                                (0x00000000U)
#define CSL_SCI_SCIPIO2_RX_DATA_IN_MAX                                     (0x00000001U)

#define CSL_SCI_SCIPIO2_TX_DATA_IN_MASK                                    (0x00000004U)
#define CSL_SCI_SCIPIO2_TX_DATA_IN_SHIFT                                   (0x00000002U)
#define CSL_SCI_SCIPIO2_TX_DATA_IN_RESETVAL                                (0x00000000U)
#define CSL_SCI_SCIPIO2_TX_DATA_IN_MAX                                     (0x00000001U)

#define CSL_SCI_SCIPIO2_RESERVED_MASK                                      (0xFFFFFFF8U)
#define CSL_SCI_SCIPIO2_RESERVED_SHIFT                                     (0x00000003U)
#define CSL_SCI_SCIPIO2_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_SCI_SCIPIO2_RESERVED_MAX                                       (0x1FFFFFFFU)

#define CSL_SCI_SCIPIO2_RESETVAL                                           (0x00000000U)

/* SCIPIO3 */

#define CSL_SCI_SCIPIO3_CLK_DATA_OUT_MASK                                  (0x00000001U)
#define CSL_SCI_SCIPIO3_CLK_DATA_OUT_SHIFT                                 (0x00000000U)
#define CSL_SCI_SCIPIO3_CLK_DATA_OUT_RESETVAL                              (0x00000000U)
#define CSL_SCI_SCIPIO3_CLK_DATA_OUT_MAX                                   (0x00000001U)

#define CSL_SCI_SCIPIO3_RX_DATA_OUT_MASK                                   (0x00000002U)
#define CSL_SCI_SCIPIO3_RX_DATA_OUT_SHIFT                                  (0x00000001U)
#define CSL_SCI_SCIPIO3_RX_DATA_OUT_RESETVAL                               (0x00000000U)
#define CSL_SCI_SCIPIO3_RX_DATA_OUT_MAX                                    (0x00000001U)

#define CSL_SCI_SCIPIO3_TX_DATA_OUT_MASK                                   (0x00000004U)
#define CSL_SCI_SCIPIO3_TX_DATA_OUT_SHIFT                                  (0x00000002U)
#define CSL_SCI_SCIPIO3_TX_DATA_OUT_RESETVAL                               (0x00000000U)
#define CSL_SCI_SCIPIO3_TX_DATA_OUT_MAX                                    (0x00000001U)

#define CSL_SCI_SCIPIO3_RESERVED_MASK                                      (0xFFFFFFF8U)
#define CSL_SCI_SCIPIO3_RESERVED_SHIFT                                     (0x00000003U)
#define CSL_SCI_SCIPIO3_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_SCI_SCIPIO3_RESERVED_MAX                                       (0x1FFFFFFFU)

#define CSL_SCI_SCIPIO3_RESETVAL                                           (0x00000000U)

/* SCIPIO4 */

#define CSL_SCI_SCIPIO4_CLK_DATA_SET_MASK                                  (0x00000001U)
#define CSL_SCI_SCIPIO4_CLK_DATA_SET_SHIFT                                 (0x00000000U)
#define CSL_SCI_SCIPIO4_CLK_DATA_SET_RESETVAL                              (0x00000000U)
#define CSL_SCI_SCIPIO4_CLK_DATA_SET_MAX                                   (0x00000001U)

#define CSL_SCI_SCIPIO4_RX_DATA_SET_MASK                                   (0x00000002U)
#define CSL_SCI_SCIPIO4_RX_DATA_SET_SHIFT                                  (0x00000001U)
#define CSL_SCI_SCIPIO4_RX_DATA_SET_RESETVAL                               (0x00000000U)
#define CSL_SCI_SCIPIO4_RX_DATA_SET_MAX                                    (0x00000001U)

#define CSL_SCI_SCIPIO4_TX_DATA_SET_MASK                                   (0x00000004U)
#define CSL_SCI_SCIPIO4_TX_DATA_SET_SHIFT                                  (0x00000002U)
#define CSL_SCI_SCIPIO4_TX_DATA_SET_RESETVAL                               (0x00000000U)
#define CSL_SCI_SCIPIO4_TX_DATA_SET_MAX                                    (0x00000001U)

#define CSL_SCI_SCIPIO4_RESERVED_MASK                                      (0xFFFFFFF8U)
#define CSL_SCI_SCIPIO4_RESERVED_SHIFT                                     (0x00000003U)
#define CSL_SCI_SCIPIO4_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_SCI_SCIPIO4_RESERVED_MAX                                       (0x1FFFFFFFU)

#define CSL_SCI_SCIPIO4_RESETVAL                                           (0x00000000U)

/* SCIPIO5 */

#define CSL_SCI_SCIPIO5_CLK_DATA_CLR_MASK                                  (0x00000001U)
#define CSL_SCI_SCIPIO5_CLK_DATA_CLR_SHIFT                                 (0x00000000U)
#define CSL_SCI_SCIPIO5_CLK_DATA_CLR_RESETVAL                              (0x00000000U)
#define CSL_SCI_SCIPIO5_CLK_DATA_CLR_MAX                                   (0x00000001U)

#define CSL_SCI_SCIPIO5_RX_DATA_CLR_MASK                                   (0x00000002U)
#define CSL_SCI_SCIPIO5_RX_DATA_CLR_SHIFT                                  (0x00000001U)
#define CSL_SCI_SCIPIO5_RX_DATA_CLR_RESETVAL                               (0x00000000U)
#define CSL_SCI_SCIPIO5_RX_DATA_CLR_MAX                                    (0x00000001U)

#define CSL_SCI_SCIPIO5_TX_DATA_CLR_MASK                                   (0x00000004U)
#define CSL_SCI_SCIPIO5_TX_DATA_CLR_SHIFT                                  (0x00000002U)
#define CSL_SCI_SCIPIO5_TX_DATA_CLR_RESETVAL                               (0x00000000U)
#define CSL_SCI_SCIPIO5_TX_DATA_CLR_MAX                                    (0x00000001U)

#define CSL_SCI_SCIPIO5_RESERVED_MASK                                      (0xFFFFFFF8U)
#define CSL_SCI_SCIPIO5_RESERVED_SHIFT                                     (0x00000003U)
#define CSL_SCI_SCIPIO5_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_SCI_SCIPIO5_RESERVED_MAX                                       (0x1FFFFFFFU)

#define CSL_SCI_SCIPIO5_RESETVAL                                           (0x00000000U)

/* SCIPIO6 */

#define CSL_SCI_SCIPIO6_CLK_PDR_MASK                                       (0x00000001U)
#define CSL_SCI_SCIPIO6_CLK_PDR_SHIFT                                      (0x00000000U)
#define CSL_SCI_SCIPIO6_CLK_PDR_RESETVAL                                   (0x00000000U)
#define CSL_SCI_SCIPIO6_CLK_PDR_MAX                                        (0x00000001U)

#define CSL_SCI_SCIPIO6_RX_PDR_MASK                                        (0x00000002U)
#define CSL_SCI_SCIPIO6_RX_PDR_SHIFT                                       (0x00000001U)
#define CSL_SCI_SCIPIO6_RX_PDR_RESETVAL                                    (0x00000000U)
#define CSL_SCI_SCIPIO6_RX_PDR_MAX                                         (0x00000001U)

#define CSL_SCI_SCIPIO6_TX_PDR_MASK                                        (0x00000004U)
#define CSL_SCI_SCIPIO6_TX_PDR_SHIFT                                       (0x00000002U)
#define CSL_SCI_SCIPIO6_TX_PDR_RESETVAL                                    (0x00000000U)
#define CSL_SCI_SCIPIO6_TX_PDR_MAX                                         (0x00000001U)

#define CSL_SCI_SCIPIO6_RESERVED_MASK                                      (0xFFFFFFF8U)
#define CSL_SCI_SCIPIO6_RESERVED_SHIFT                                     (0x00000003U)
#define CSL_SCI_SCIPIO6_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_SCI_SCIPIO6_RESERVED_MAX                                       (0x1FFFFFFFU)

#define CSL_SCI_SCIPIO6_RESETVAL                                           (0x00000000U)

/* SCIPIO7 */

#define CSL_SCI_SCIPIO7_CLK_PD_MASK                                        (0x00000001U)
#define CSL_SCI_SCIPIO7_CLK_PD_SHIFT                                       (0x00000000U)
#define CSL_SCI_SCIPIO7_CLK_PD_RESETVAL                                    (0x00000000U)
#define CSL_SCI_SCIPIO7_CLK_PD_MAX                                         (0x00000001U)

#define CSL_SCI_SCIPIO7_RX_PD_MASK                                         (0x00000002U)
#define CSL_SCI_SCIPIO7_RX_PD_SHIFT                                        (0x00000001U)
#define CSL_SCI_SCIPIO7_RX_PD_RESETVAL                                     (0x00000000U)
#define CSL_SCI_SCIPIO7_RX_PD_MAX                                          (0x00000001U)

#define CSL_SCI_SCIPIO7_TX_PD_MASK                                         (0x00000004U)
#define CSL_SCI_SCIPIO7_TX_PD_SHIFT                                        (0x00000002U)
#define CSL_SCI_SCIPIO7_TX_PD_RESETVAL                                     (0x00000000U)
#define CSL_SCI_SCIPIO7_TX_PD_MAX                                          (0x00000001U)

#define CSL_SCI_SCIPIO7_RESERVED_MASK                                      (0xFFFFFFF8U)
#define CSL_SCI_SCIPIO7_RESERVED_SHIFT                                     (0x00000003U)
#define CSL_SCI_SCIPIO7_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_SCI_SCIPIO7_RESERVED_MAX                                       (0x1FFFFFFFU)

#define CSL_SCI_SCIPIO7_RESETVAL                                           (0x00000000U)

/* SCIPIO8 */

#define CSL_SCI_SCIPIO8_CLK_PSL_MASK                                       (0x00000001U)
#define CSL_SCI_SCIPIO8_CLK_PSL_SHIFT                                      (0x00000000U)
#define CSL_SCI_SCIPIO8_CLK_PSL_RESETVAL                                   (0x00000000U)
#define CSL_SCI_SCIPIO8_CLK_PSL_MAX                                        (0x00000001U)

#define CSL_SCI_SCIPIO8_RX_PSL_MASK                                        (0x00000002U)
#define CSL_SCI_SCIPIO8_RX_PSL_SHIFT                                       (0x00000001U)
#define CSL_SCI_SCIPIO8_RX_PSL_RESETVAL                                    (0x00000000U)
#define CSL_SCI_SCIPIO8_RX_PSL_MAX                                         (0x00000001U)

#define CSL_SCI_SCIPIO8_TX_PSL_MASK                                        (0x00000004U)
#define CSL_SCI_SCIPIO8_TX_PSL_SHIFT                                       (0x00000002U)
#define CSL_SCI_SCIPIO8_TX_PSL_RESETVAL                                    (0x00000000U)
#define CSL_SCI_SCIPIO8_TX_PSL_MAX                                         (0x00000001U)

#define CSL_SCI_SCIPIO8_RESERVED_MASK                                      (0xFFFFFFF8U)
#define CSL_SCI_SCIPIO8_RESERVED_SHIFT                                     (0x00000003U)
#define CSL_SCI_SCIPIO8_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_SCI_SCIPIO8_RESERVED_MAX                                       (0x1FFFFFFFU)

#define CSL_SCI_SCIPIO8_RESETVAL                                           (0x00000000U)

/* RESERVED2 */

#define CSL_SCI_RESERVED2_RESERVED_MASK                                    (0xFFFFFFFFU)
#define CSL_SCI_RESERVED2_RESERVED_SHIFT                                   (0x00000000U)
#define CSL_SCI_RESERVED2_RESERVED_RESETVAL                                (0x00000000U)
#define CSL_SCI_RESERVED2_RESERVED_MAX                                     (0xFFFFFFFFU)

#define CSL_SCI_RESERVED2_RESETVAL                                         (0x00000000U)

/* RESERVED3 */

#define CSL_SCI_RESERVED3_RESERVED_MASK                                    (0xFFFFFFFFU)
#define CSL_SCI_RESERVED3_RESERVED_SHIFT                                   (0x00000000U)
#define CSL_SCI_RESERVED3_RESERVED_RESETVAL                                (0x00000000U)
#define CSL_SCI_RESERVED3_RESERVED_MAX                                     (0xFFFFFFFFU)

#define CSL_SCI_RESERVED3_RESETVAL                                         (0x00000000U)

/* RESERVED4 */

#define CSL_SCI_RESERVED4_RESERVED_MASK                                    (0xFFFFFFFFU)
#define CSL_SCI_RESERVED4_RESERVED_SHIFT                                   (0x00000000U)
#define CSL_SCI_RESERVED4_RESERVED_RESETVAL                                (0x00000000U)
#define CSL_SCI_RESERVED4_RESERVED_MAX                                     (0xFFFFFFFFU)

#define CSL_SCI_RESERVED4_RESETVAL                                         (0x00000000U)

/* RESERVED5 */

#define CSL_SCI_RESERVED5_RESERVED_MASK                                    (0xFFFFFFFFU)
#define CSL_SCI_RESERVED5_RESERVED_SHIFT                                   (0x00000000U)
#define CSL_SCI_RESERVED5_RESERVED_RESETVAL                                (0x00000000U)
#define CSL_SCI_RESERVED5_RESERVED_MAX                                     (0xFFFFFFFFU)

#define CSL_SCI_RESERVED5_RESETVAL                                         (0x00000000U)

/* RESERVED6 */

#define CSL_SCI_RESERVED6_RESERVED_MASK                                    (0xFFFFFFFFU)
#define CSL_SCI_RESERVED6_RESERVED_SHIFT                                   (0x00000000U)
#define CSL_SCI_RESERVED6_RESERVED_RESETVAL                                (0x00000000U)
#define CSL_SCI_RESERVED6_RESERVED_MAX                                     (0xFFFFFFFFU)

#define CSL_SCI_RESERVED6_RESETVAL                                         (0x00000000U)

/* RESERVED7 */

#define CSL_SCI_RESERVED7_RESERVED_MASK                                    (0xFFFFFFFFU)
#define CSL_SCI_RESERVED7_RESERVED_SHIFT                                   (0x00000000U)
#define CSL_SCI_RESERVED7_RESERVED_RESETVAL                                (0x00000000U)
#define CSL_SCI_RESERVED7_RESERVED_MAX                                     (0xFFFFFFFFU)

#define CSL_SCI_RESERVED7_RESETVAL                                         (0x00000000U)

/* RESERVED8 */

#define CSL_SCI_RESERVED8_RESERVED_MASK                                    (0xFFFFFFFFU)
#define CSL_SCI_RESERVED8_RESERVED_SHIFT                                   (0x00000000U)
#define CSL_SCI_RESERVED8_RESERVED_RESETVAL                                (0x00000000U)
#define CSL_SCI_RESERVED8_RESERVED_MAX                                     (0xFFFFFFFFU)

#define CSL_SCI_RESERVED8_RESETVAL                                         (0x00000000U)

/* RESERVED9 */

#define CSL_SCI_RESERVED9_RESERVED_MASK                                    (0xFFFFFFFFU)
#define CSL_SCI_RESERVED9_RESERVED_SHIFT                                   (0x00000000U)
#define CSL_SCI_RESERVED9_RESERVED_RESETVAL                                (0x00000000U)
#define CSL_SCI_RESERVED9_RESERVED_MAX                                     (0xFFFFFFFFU)

#define CSL_SCI_RESERVED9_RESETVAL                                         (0x00000000U)

/* SCIPIO9 */

#define CSL_SCI_SCIPIO9_CLK_SL_MASK                                        (0x00000001U)
#define CSL_SCI_SCIPIO9_CLK_SL_SHIFT                                       (0x00000000U)
#define CSL_SCI_SCIPIO9_CLK_SL_RESETVAL                                    (0x00000000U)
#define CSL_SCI_SCIPIO9_CLK_SL_MAX                                         (0x00000001U)

#define CSL_SCI_SCIPIO9_RX_SL_MASK                                         (0x00000002U)
#define CSL_SCI_SCIPIO9_RX_SL_SHIFT                                        (0x00000001U)
#define CSL_SCI_SCIPIO9_RX_SL_RESETVAL                                     (0x00000000U)
#define CSL_SCI_SCIPIO9_RX_SL_MAX                                          (0x00000001U)

#define CSL_SCI_SCIPIO9_TX_SL_MASK                                         (0x00000004U)
#define CSL_SCI_SCIPIO9_TX_SL_SHIFT                                        (0x00000002U)
#define CSL_SCI_SCIPIO9_TX_SL_RESETVAL                                     (0x00000000U)
#define CSL_SCI_SCIPIO9_TX_SL_MAX                                          (0x00000001U)

#define CSL_SCI_SCIPIO9_RESERVED_MASK                                      (0xFFFFFFF8U)
#define CSL_SCI_SCIPIO9_RESERVED_SHIFT                                     (0x00000003U)
#define CSL_SCI_SCIPIO9_RESERVED_RESETVAL                                  (0x00000000U)
#define CSL_SCI_SCIPIO9_RESERVED_MAX                                       (0x1FFFFFFFU)

#define CSL_SCI_SCIPIO9_RESETVAL                                           (0x00000000U)

/* SCIIODCTRL */

#define CSL_SCI_SCIIODCTRL_RXP_ENA_MASK                                    (0x00000001U)
#define CSL_SCI_SCIIODCTRL_RXP_ENA_SHIFT                                   (0x00000000U)
#define CSL_SCI_SCIIODCTRL_RXP_ENA_RESETVAL                                (0x00000000U)
#define CSL_SCI_SCIIODCTRL_RXP_ENA_MAX                                     (0x00000001U)

#define CSL_SCI_SCIIODCTRL_LBP_ENA_MASK                                    (0x00000002U)
#define CSL_SCI_SCIIODCTRL_LBP_ENA_SHIFT                                   (0x00000001U)
#define CSL_SCI_SCIIODCTRL_LBP_ENA_RESETVAL                                (0x00000000U)
#define CSL_SCI_SCIIODCTRL_LBP_ENA_MAX                                     (0x00000001U)

#define CSL_SCI_SCIIODCTRL_RESERVED1_MASK                                  (0x000000FCU)
#define CSL_SCI_SCIIODCTRL_RESERVED1_SHIFT                                 (0x00000002U)
#define CSL_SCI_SCIIODCTRL_RESERVED1_RESETVAL                              (0x00000000U)
#define CSL_SCI_SCIIODCTRL_RESERVED1_MAX                                   (0x0000003FU)

#define CSL_SCI_SCIIODCTRL_IODFTENA_MASK                                   (0x00000F00U)
#define CSL_SCI_SCIIODCTRL_IODFTENA_SHIFT                                  (0x00000008U)
#define CSL_SCI_SCIIODCTRL_IODFTENA_RESETVAL                               (0x00000000U)
#define CSL_SCI_SCIIODCTRL_IODFTENA_MAX                                    (0x0000000FU)

#define CSL_SCI_SCIIODCTRL_RESERVED2_MASK                                  (0x0000F000U)
#define CSL_SCI_SCIIODCTRL_RESERVED2_SHIFT                                 (0x0000000CU)
#define CSL_SCI_SCIIODCTRL_RESERVED2_RESETVAL                              (0x00000000U)
#define CSL_SCI_SCIIODCTRL_RESERVED2_MAX                                   (0x0000000FU)

#define CSL_SCI_SCIIODCTRL_TX_SHIFT_MASK                                   (0x00070000U)
#define CSL_SCI_SCIIODCTRL_TX_SHIFT_SHIFT                                  (0x00000010U)
#define CSL_SCI_SCIIODCTRL_TX_SHIFT_RESETVAL                               (0x00000000U)
#define CSL_SCI_SCIIODCTRL_TX_SHIFT_MAX                                    (0x00000007U)

#define CSL_SCI_SCIIODCTRL_PIN_SAMPLE_MASK_MASK                            (0x00180000U)
#define CSL_SCI_SCIIODCTRL_PIN_SAMPLE_MASK_SHIFT                           (0x00000013U)
#define CSL_SCI_SCIIODCTRL_PIN_SAMPLE_MASK_RESETVAL                        (0x00000000U)
#define CSL_SCI_SCIIODCTRL_PIN_SAMPLE_MASK_MAX                             (0x00000003U)

#define CSL_SCI_SCIIODCTRL_RESERVED3_MASK                                  (0x00E00000U)
#define CSL_SCI_SCIIODCTRL_RESERVED3_SHIFT                                 (0x00000015U)
#define CSL_SCI_SCIIODCTRL_RESERVED3_RESETVAL                              (0x00000000U)
#define CSL_SCI_SCIIODCTRL_RESERVED3_MAX                                   (0x00000007U)

#define CSL_SCI_SCIIODCTRL_BRKDT_ENA_MASK                                  (0x01000000U)
#define CSL_SCI_SCIIODCTRL_BRKDT_ENA_SHIFT                                 (0x00000018U)
#define CSL_SCI_SCIIODCTRL_BRKDT_ENA_RESETVAL                              (0x00000000U)
#define CSL_SCI_SCIIODCTRL_BRKDT_ENA_MAX                                   (0x00000001U)

#define CSL_SCI_SCIIODCTRL_PEN_MASK                                        (0x02000000U)
#define CSL_SCI_SCIIODCTRL_PEN_SHIFT                                       (0x00000019U)
#define CSL_SCI_SCIIODCTRL_PEN_RESETVAL                                    (0x00000000U)
#define CSL_SCI_SCIIODCTRL_PEN_MAX                                         (0x00000001U)

#define CSL_SCI_SCIIODCTRL_FEN_MASK                                        (0x04000000U)
#define CSL_SCI_SCIIODCTRL_FEN_SHIFT                                       (0x0000001AU)
#define CSL_SCI_SCIIODCTRL_FEN_RESETVAL                                    (0x00000000U)
#define CSL_SCI_SCIIODCTRL_FEN_MAX                                         (0x00000001U)

#define CSL_SCI_SCIIODCTRL_RESERVED4_MASK                                  (0xF8000000U)
#define CSL_SCI_SCIIODCTRL_RESERVED4_SHIFT                                 (0x0000001BU)
#define CSL_SCI_SCIIODCTRL_RESERVED4_RESETVAL                              (0x00000000U)
#define CSL_SCI_SCIIODCTRL_RESERVED4_MAX                                   (0x0000001FU)

#define CSL_SCI_SCIIODCTRL_RESETVAL                                        (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
