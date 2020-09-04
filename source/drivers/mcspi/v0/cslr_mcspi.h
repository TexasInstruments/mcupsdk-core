/*
 *  Copyright (C) 2014-2021 Texas Instruments Incorporated
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

/**
*
*  \file   cslr_mcspi.h
*
*  \brief  register-level header file for MCSPI
*
**/

#ifndef CSLR_MCSPI_H_
#define CSLR_MCSPI_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************
* Register Overlay Structure for __ALL__
**************************************************************************/
typedef struct {
    volatile uint32_t HL_REV;
    volatile uint32_t HL_HWINFO;
    volatile uint8_t  RSVD0[8];
    volatile uint32_t HL_SYSCONFIG;
    volatile uint8_t  RSVD1[236];
    volatile uint32_t REVISION;
    volatile uint8_t  RSVD2[12];
    volatile uint32_t SYSCONFIG;
    volatile uint32_t SYSSTATUS;
    volatile uint32_t IRQSTATUS;
    volatile uint32_t IRQENABLE;
    volatile uint32_t WAKEUPENABLE;
    volatile uint32_t SYST;
    volatile uint32_t MODULCTRL;
    volatile uint32_t CH0CONF;
    volatile uint32_t CH0STAT;
    volatile uint32_t CH0CTRL;
    volatile uint32_t TX0;
    volatile uint32_t RX0;
    volatile uint32_t CH1CONF;
    volatile uint32_t CH1STAT;
    volatile uint32_t CH1CTRL;
    volatile uint32_t TX1;
    volatile uint32_t RX1;
    volatile uint32_t CH2CONF;
    volatile uint32_t CH2STAT;
    volatile uint32_t CH2CTRL;
    volatile uint32_t TX2;
    volatile uint32_t RX2;
    volatile uint32_t CH3CONF;
    volatile uint32_t CH3STAT;
    volatile uint32_t CH3CTRL;
    volatile uint32_t TX3;
    volatile uint32_t RX3;
    volatile uint32_t XFERLEVEL;
    volatile uint32_t DAFTX;
    volatile uint8_t  RSVD3[28];
    volatile uint32_t DAFRX;
} CSL_McspiRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

/* IP Revision Identifier (X.Y.R) Used by software to track features, bugs,
 * and compatibility */
#define CSL_MCSPI_HL_REV                                        (0x0U)

/* Information about the IP module's hardware configuration, i.e. typically
 * the module's HDL generics (if any). Actual field format and encoding is up
 * to the module's designer to decide. */
#define CSL_MCSPI_HL_HWINFO                                     (0x4U)

/* Clock management configuration */
#define CSL_MCSPI_HL_SYSCONFIG                                  (0x10U)

/* This register contains the hard coded RTL revision number. */
#define CSL_MCSPI_REVISION                                      (0x100U)

/* This register allows controlling various parameters of the OCP interface. */
#define CSL_MCSPI_SYSCONFIG                                     (0x110U)

/* This register provides status information about the module excluding the
 * interrupt status information */
#define CSL_MCSPI_SYSSTATUS                                     (0x114U)

/* The interrupt status regroups all the status of the module internal events
 * that can generate an interrupt */
#define CSL_MCSPI_IRQSTATUS                                     (0x118U)

/* This register allows to enable/disable the module internal sources of
 * interrupt, on an event-by-event basis. */
#define CSL_MCSPI_IRQENABLE                                     (0x11CU)

/* The wakeup enable register allows to enable/disable the module internal
 * sources of wakeup on event-by-event basis. */
#define CSL_MCSPI_WAKEUPENABLE                                  (0x120U)

/* This register is used to check the correctness of the system interconnect
 * either internally to peripheral bus, or externally to device IO pads, when
 * the module is configured in system test (SYSTEST) mode. */
#define CSL_MCSPI_SYST                                          (0x124U)

/* This register is dedicated to the configuration of the serial port
 * interface. */
#define CSL_MCSPI_MODULCTRL                                     (0x128U)

/* This register is dedicated to the configuration of the channel 0 */
#define CSL_MCSPI_CH0CONF                                       (0x12CU)

/* This register provides status information about transmitter and receiver
 * registers of channel 0 */
#define CSL_MCSPI_CH0STAT                                       (0x130U)

/* This register is dedicated to enable the channel 0 */
#define CSL_MCSPI_CH0CTRL                                       (0x134U)

/* This register contains a single SPI word to transmit on the serial link,
 * what ever SPI word length is. */
#define CSL_MCSPI_TX0                                           (0x138U)

/* This register contains a single SPI word received through the serial link,
 * what ever SPI word length is. */
#define CSL_MCSPI_RX0                                           (0x13CU)

/* This register is dedicated to the configuration of the channel. */
#define CSL_MCSPI_CH1CONF                                       (0x140U)

/* This register provides status information about transmitter and receiver
 * registers of channel 1 */
#define CSL_MCSPI_CH1STAT                                       (0x144U)

/* This register is dedicated to enable the channel 1 */
#define CSL_MCSPI_CH1CTRL                                       (0x148U)

/* This register contains a single SPI word to transmit on the serial link,
 * what ever SPI word length is. */
#define CSL_MCSPI_TX1                                           (0x14CU)

/* This register contains a single SPI word received through the serial link,
 * what ever SPI word length is. */
#define CSL_MCSPI_RX1                                           (0x150U)

/* This register is dedicated to the configuration of the channel 2 */
#define CSL_MCSPI_CH2CONF                                       (0x154U)

/* This register provides status information about transmitter and receiver
 * registers of channel 2 */
#define CSL_MCSPI_CH2STAT                                       (0x158U)

/* This register is dedicated to enable the channel 2 */
#define CSL_MCSPI_CH2CTRL                                       (0x15CU)

/* This register contains a single SPI word to transmit on the serial link,
 * what ever SPI word length is. */
#define CSL_MCSPI_TX2                                           (0x160U)

/* This register contains a single SPI word received through the serial link,
 * what ever SPI word length is. */
#define CSL_MCSPI_RX2                                           (0x164U)

/* This register is dedicated to the configuration of the channel 3 */
#define CSL_MCSPI_CH3CONF                                       (0x168U)

/* This register provides status information about transmitter and receiver
 * registers of channel 3 */
#define CSL_MCSPI_CH3STAT                                       (0x16CU)

/* This register is dedicated to enable the channel 3 */
#define CSL_MCSPI_CH3CTRL                                       (0x170U)

/* This register contains a single SPI word to transmit on the serial link,
 * what ever SPI word length is. */
#define CSL_MCSPI_TX3                                           (0x174U)

/* This register contains a single SPI word received through the serial link,
 * what ever SPI word length is. */
#define CSL_MCSPI_RX3                                           (0x178U)

/* This register provides transfer levels needed while using FIFO buffer
 * during transfer. */
#define CSL_MCSPI_XFERLEVEL                                     (0x17CU)

/* This register contains the SPI words to transmit on the serial link when
 * FIFO used and DMA address is aligned on 256 bit.This register is an image
 * of one of MCSPI_TX(i) register corresponding to the channel which have its
 * FIFO enabled. */
#define CSL_MCSPI_DAFTX                                         (0x180U)

/* This register contains the SPI words to received on the serial link when
 * FIFO used and DMA address is aligned on 256 bit.This register is an image
 * of one of MCSPI_RX(i) register corresponding to the channel which have its
 * FIFO enabled. */
#define CSL_MCSPI_DAFRX                                         (0x1A0U)


/**************************************************************************
* Field Definition Macros
**************************************************************************/

/* HL_REV */

#define CSL_MCSPI_HL_REV_FUNC_MASK                              (0x0FFF0000U)
#define CSL_MCSPI_HL_REV_FUNC_SHIFT                             (16U)
#define CSL_MCSPI_HL_REV_FUNC_RESETVAL                          (0x00000030U)
#define CSL_MCSPI_HL_REV_FUNC_MAX                               (0x00000fffU)

#define CSL_MCSPI_HL_REV_SCHEME_MASK                            (0xC0000000U)
#define CSL_MCSPI_HL_REV_SCHEME_SHIFT                           (30U)
#define CSL_MCSPI_HL_REV_SCHEME_RESETVAL                        (0x00000002U)
#define CSL_MCSPI_HL_REV_SCHEME_HIGHLANDER                      (0x00000001U)
#define CSL_MCSPI_HL_REV_SCHEME_LEGACY                          (0x00000000U)

#define CSL_MCSPI_HL_REV_CUSTOM_MASK                            (0x000000C0U)
#define CSL_MCSPI_HL_REV_CUSTOM_SHIFT                           (6U)
#define CSL_MCSPI_HL_REV_CUSTOM_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_HL_REV_CUSTOM_READ0                           (0x00000000U)

#define CSL_MCSPI_HL_REV_X_MAJOR_MASK                           (0x00000700U)
#define CSL_MCSPI_HL_REV_X_MAJOR_SHIFT                          (8U)
#define CSL_MCSPI_HL_REV_X_MAJOR_RESETVAL                       (0x00000000U)
#define CSL_MCSPI_HL_REV_X_MAJOR_MAX                            (0x00000007U)

#define CSL_MCSPI_HL_REV_Y_MINOR_MASK                           (0x0000003FU)
#define CSL_MCSPI_HL_REV_Y_MINOR_SHIFT                          (0U)
#define CSL_MCSPI_HL_REV_Y_MINOR_RESETVAL                       (0x00000000U)
#define CSL_MCSPI_HL_REV_Y_MINOR_MAX                            (0x0000003fU)

#define CSL_MCSPI_HL_REV_R_RTL_MASK                             (0x0000F800U)
#define CSL_MCSPI_HL_REV_R_RTL_SHIFT                            (11U)
#define CSL_MCSPI_HL_REV_R_RTL_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_HL_REV_R_RTL_MAX                              (0x0000001fU)

#define CSL_MCSPI_HL_REV_RSVD_MASK                              (0x30000000U)
#define CSL_MCSPI_HL_REV_RSVD_SHIFT                             (28U)
#define CSL_MCSPI_HL_REV_RSVD_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_HL_REV_RSVD_MAX                               (0x00000003U)

#define CSL_MCSPI_HL_REV_RESETVAL                               (0x80300000U)

/* HL_HWINFO */

#define CSL_MCSPI_HL_HWINFO_USEFIFO_MASK                        (0x00000001U)
#define CSL_MCSPI_HL_HWINFO_USEFIFO_SHIFT                       (0U)
#define CSL_MCSPI_HL_HWINFO_USEFIFO_RESETVAL                    (0x00000001U)
#define CSL_MCSPI_HL_HWINFO_USEFIFO_FIFOEN                      (0x00000001U)
#define CSL_MCSPI_HL_HWINFO_USEFIFO_NOFIFO                      (0x00000000U)

#define CSL_MCSPI_HL_HWINFO_FFNBYTE_MASK                        (0x0000003EU)
#define CSL_MCSPI_HL_HWINFO_FFNBYTE_SHIFT                       (1U)
#define CSL_MCSPI_HL_HWINFO_FFNBYTE_RESETVAL                    (0x00000004U)
#define CSL_MCSPI_HL_HWINFO_FFNBYTE_MAX                         (0x0000001fU)

#define CSL_MCSPI_HL_HWINFO_RSVD_MASK                           (0xFFFFFF80U)
#define CSL_MCSPI_HL_HWINFO_RSVD_SHIFT                          (7U)
#define CSL_MCSPI_HL_HWINFO_RSVD_RESETVAL                       (0x00000000U)
#define CSL_MCSPI_HL_HWINFO_RSVD_MAX                            (0x01ffffffU)

#define CSL_MCSPI_HL_HWINFO_RETMODE_MASK                        (0x00000040U)
#define CSL_MCSPI_HL_HWINFO_RETMODE_SHIFT                       (6U)
#define CSL_MCSPI_HL_HWINFO_RETMODE_RESETVAL                    (0x00000000U)
#define CSL_MCSPI_HL_HWINFO_RETMODE_NORETMODE                   (0x00000000U)
#define CSL_MCSPI_HL_HWINFO_RETMODE_RETMODEEN                   (0x00000001U)

#define CSL_MCSPI_HL_HWINFO_RESETVAL                            (0x00000009U)

/* HL_SYSCONFIG */

#define CSL_MCSPI_HL_SYSCONFIG_IDLEMODE_MASK                    (0x0000000CU)
#define CSL_MCSPI_HL_SYSCONFIG_IDLEMODE_SHIFT                   (2U)
#define CSL_MCSPI_HL_SYSCONFIG_IDLEMODE_RESETVAL                (0x00000002U)
#define CSL_MCSPI_HL_SYSCONFIG_IDLEMODE_SMARTIDLEWAKEUP         (0x00000003U)
#define CSL_MCSPI_HL_SYSCONFIG_IDLEMODE_FORCEIDLE               (0x00000000U)
#define CSL_MCSPI_HL_SYSCONFIG_IDLEMODE_NOIDLE                  (0x00000001U)
#define CSL_MCSPI_HL_SYSCONFIG_IDLEMODE_SMARTIDLE               (0x00000002U)

#define CSL_MCSPI_HL_SYSCONFIG_RSVD_MASK                        (0xFFFFFFF0U)
#define CSL_MCSPI_HL_SYSCONFIG_RSVD_SHIFT                       (4U)
#define CSL_MCSPI_HL_SYSCONFIG_RSVD_RESETVAL                    (0x00000000U)
#define CSL_MCSPI_HL_SYSCONFIG_RSVD_MAX                         (0x0fffffffU)

#define CSL_MCSPI_HL_SYSCONFIG_SOFTRESET_MASK                   (0x00000001U)
#define CSL_MCSPI_HL_SYSCONFIG_SOFTRESET_SHIFT                  (0U)
#define CSL_MCSPI_HL_SYSCONFIG_SOFTRESET_RESETVAL               (0x00000000U)
#define CSL_MCSPI_HL_SYSCONFIG_SOFTRESET_RESETDONE              (0x00000000U)
#define CSL_MCSPI_HL_SYSCONFIG_SOFTRESET_NOACTION               (0x00000000U)
#define CSL_MCSPI_HL_SYSCONFIG_SOFTRESET_SOFTRESET              (0x00000001U)
#define CSL_MCSPI_HL_SYSCONFIG_SOFTRESET_RESETONGOING           (0x00000001U)

#define CSL_MCSPI_HL_SYSCONFIG_FREEEMU_MASK                     (0x00000002U)
#define CSL_MCSPI_HL_SYSCONFIG_FREEEMU_SHIFT                    (1U)
#define CSL_MCSPI_HL_SYSCONFIG_FREEEMU_RESETVAL                 (0x00000000U)
#define CSL_MCSPI_HL_SYSCONFIG_FREEEMU_EMUDIS                   (0x00000001U)
#define CSL_MCSPI_HL_SYSCONFIG_FREEEMU_EMUEN                    (0x00000000U)

#define CSL_MCSPI_HL_SYSCONFIG_RESETVAL                         (0x00000008U)

/* REVISION */

#define CSL_MCSPI_REVISION_REV_MASK                             (0x000000FFU)
#define CSL_MCSPI_REVISION_REV_SHIFT                            (0U)
#define CSL_MCSPI_REVISION_REV_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_REVISION_REV_MAX                              (0x000000ffU)

#define CSL_MCSPI_REVISION_RESETVAL                             (0x00000000U)

/* SYSCONFIG */

#define CSL_MCSPI_SYSCONFIG_SIDLEMODE_MASK                      (0x00000018U)
#define CSL_MCSPI_SYSCONFIG_SIDLEMODE_SHIFT                     (3U)
#define CSL_MCSPI_SYSCONFIG_SIDLEMODE_RESETVAL                  (0x00000002U)
#define CSL_MCSPI_SYSCONFIG_SIDLEMODE_FORCE                     (0x00000000U)
#define CSL_MCSPI_SYSCONFIG_SIDLEMODE_NO                        (0x00000001U)
#define CSL_MCSPI_SYSCONFIG_SIDLEMODE_SMART                     (0x00000002U)
#define CSL_MCSPI_SYSCONFIG_SIDLEMODE_SMART_IDLE_WAKEUP         (0x00000003U)

#define CSL_MCSPI_SYSCONFIG_SOFTRESET_MASK                      (0x00000002U)
#define CSL_MCSPI_SYSCONFIG_SOFTRESET_SHIFT                     (1U)
#define CSL_MCSPI_SYSCONFIG_SOFTRESET_RESETVAL                  (0x00000000U)
#define CSL_MCSPI_SYSCONFIG_SOFTRESET_OFF                       (0x00000000U)
#define CSL_MCSPI_SYSCONFIG_SOFTRESET_ON                        (0x00000001U)

#define CSL_MCSPI_SYSCONFIG_CLOCKACTIVITY_MASK                  (0x00000300U)
#define CSL_MCSPI_SYSCONFIG_CLOCKACTIVITY_SHIFT                 (8U)
#define CSL_MCSPI_SYSCONFIG_CLOCKACTIVITY_RESETVAL              (0x00000000U)
#define CSL_MCSPI_SYSCONFIG_CLOCKACTIVITY_NONE                  (0x00000000U)
#define CSL_MCSPI_SYSCONFIG_CLOCKACTIVITY_OCP                   (0x00000001U)
#define CSL_MCSPI_SYSCONFIG_CLOCKACTIVITY_FUNC                  (0x00000002U)
#define CSL_MCSPI_SYSCONFIG_CLOCKACTIVITY_BOTH                  (0x00000003U)

#define CSL_MCSPI_SYSCONFIG_AUTOIDLE_MASK                       (0x00000001U)
#define CSL_MCSPI_SYSCONFIG_AUTOIDLE_SHIFT                      (0U)
#define CSL_MCSPI_SYSCONFIG_AUTOIDLE_RESETVAL                   (0x00000001U)
#define CSL_MCSPI_SYSCONFIG_AUTOIDLE_OFF                        (0x00000000U)
#define CSL_MCSPI_SYSCONFIG_AUTOIDLE_ON                         (0x00000001U)

#define CSL_MCSPI_SYSCONFIG_ENAWAKEUP_MASK                      (0x00000004U)
#define CSL_MCSPI_SYSCONFIG_ENAWAKEUP_SHIFT                     (2U)
#define CSL_MCSPI_SYSCONFIG_ENAWAKEUP_RESETVAL                  (0x00000001U)
#define CSL_MCSPI_SYSCONFIG_ENAWAKEUP_NOWAKEUP                  (0x00000000U)
#define CSL_MCSPI_SYSCONFIG_ENAWAKEUP_ON                        (0x00000001U)

#define CSL_MCSPI_SYSCONFIG_RESETVAL                            (0x00000015U)

/* SYSSTATUS */

#define CSL_MCSPI_SYSSTATUS_RESETDONE_MASK                      (0x00000001U)
#define CSL_MCSPI_SYSSTATUS_RESETDONE_SHIFT                     (0U)
#define CSL_MCSPI_SYSSTATUS_RESETDONE_RESETVAL                  (0x00000000U)
#define CSL_MCSPI_SYSSTATUS_RESETDONE_INPROGRESS                (0x00000000U)
#define CSL_MCSPI_SYSSTATUS_RESETDONE_COMPLETED                 (0x00000001U)

#define CSL_MCSPI_SYSSTATUS_RESETVAL                            (0x00000000U)

/* IRQSTATUS */

#define CSL_MCSPI_IRQSTATUS_RX3_FULL_MASK                       (0x00004000U)
#define CSL_MCSPI_IRQSTATUS_RX3_FULL_SHIFT                      (14U)
#define CSL_MCSPI_IRQSTATUS_RX3_FULL_RESETVAL                   (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_RX3_FULL_NOEVNT_R                   (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_RX3_FULL_EVNT_R                     (0x00000001U)
#define CSL_MCSPI_IRQSTATUS_RX3_FULL_NOEFFECT_W                 (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_RX3_FULL_CLEARSRC_W                 (0x00000001U)

#define CSL_MCSPI_IRQSTATUS_WKS_MASK                            (0x00010000U)
#define CSL_MCSPI_IRQSTATUS_WKS_SHIFT                           (16U)
#define CSL_MCSPI_IRQSTATUS_WKS_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_WKS_NOEVNT_R                        (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_WKS_EVNT_R                          (0x00000001U)
#define CSL_MCSPI_IRQSTATUS_WKS_NOEFFECT_W                      (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_WKS_CLEARSRC_W                      (0x00000001U)

#define CSL_MCSPI_IRQSTATUS_RX2_FULL_MASK                       (0x00000400U)
#define CSL_MCSPI_IRQSTATUS_RX2_FULL_SHIFT                      (10U)
#define CSL_MCSPI_IRQSTATUS_RX2_FULL_RESETVAL                   (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_RX2_FULL_NOEVNT_R                   (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_RX2_FULL_EVNT_R                     (0x00000001U)
#define CSL_MCSPI_IRQSTATUS_RX2_FULL_NOEFFECT_W                 (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_RX2_FULL_CLEARSRC_W                 (0x00000001U)

#define CSL_MCSPI_IRQSTATUS_TX3_EMPTY_MASK                      (0x00001000U)
#define CSL_MCSPI_IRQSTATUS_TX3_EMPTY_SHIFT                     (12U)
#define CSL_MCSPI_IRQSTATUS_TX3_EMPTY_RESETVAL                  (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_TX3_EMPTY_NOEVNT_R                  (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_TX3_EMPTY_EVNT_R                    (0x00000001U)
#define CSL_MCSPI_IRQSTATUS_TX3_EMPTY_NOEFFECT_W                (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_TX3_EMPTY_CLEARSRC_W                (0x00000001U)

#define CSL_MCSPI_IRQSTATUS_RX1_FULL_MASK                       (0x00000040U)
#define CSL_MCSPI_IRQSTATUS_RX1_FULL_SHIFT                      (6U)
#define CSL_MCSPI_IRQSTATUS_RX1_FULL_RESETVAL                   (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_RX1_FULL_NOEVNT_R                   (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_RX1_FULL_EVNT_R                     (0x00000001U)
#define CSL_MCSPI_IRQSTATUS_RX1_FULL_NOEFFECT_W                 (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_RX1_FULL_CLEARSRC_W                 (0x00000001U)

#define CSL_MCSPI_IRQSTATUS_TX3_UNDERFLOW_MASK                  (0x00002000U)
#define CSL_MCSPI_IRQSTATUS_TX3_UNDERFLOW_SHIFT                 (13U)
#define CSL_MCSPI_IRQSTATUS_TX3_UNDERFLOW_RESETVAL              (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_TX3_UNDERFLOW_NOEVNT_R              (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_TX3_UNDERFLOW_EVNT_R                (0x00000001U)
#define CSL_MCSPI_IRQSTATUS_TX3_UNDERFLOW_NOEFFECT_W            (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_TX3_UNDERFLOW_CLEARSRC_W            (0x00000001U)

#define CSL_MCSPI_IRQSTATUS_TX2_EMPTY_MASK                      (0x00000100U)
#define CSL_MCSPI_IRQSTATUS_TX2_EMPTY_SHIFT                     (8U)
#define CSL_MCSPI_IRQSTATUS_TX2_EMPTY_RESETVAL                  (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_TX2_EMPTY_NOEVNT_R                  (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_TX2_EMPTY_EVNT_R                    (0x00000001U)
#define CSL_MCSPI_IRQSTATUS_TX2_EMPTY_NOEFFECT_W                (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_TX2_EMPTY_CLEARSRC_W                (0x00000001U)

#define CSL_MCSPI_IRQSTATUS_TX2_UNDERFLOW_MASK                  (0x00000200U)
#define CSL_MCSPI_IRQSTATUS_TX2_UNDERFLOW_SHIFT                 (9U)
#define CSL_MCSPI_IRQSTATUS_TX2_UNDERFLOW_RESETVAL              (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_TX2_UNDERFLOW_NOEVNT_R              (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_TX2_UNDERFLOW_EVNT_R                (0x00000001U)
#define CSL_MCSPI_IRQSTATUS_TX2_UNDERFLOW_NOEFFECT_W            (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_TX2_UNDERFLOW_CLEARSRC_W            (0x00000001U)

#define CSL_MCSPI_IRQSTATUS_TX1_EMPTY_MASK                      (0x00000010U)
#define CSL_MCSPI_IRQSTATUS_TX1_EMPTY_SHIFT                     (4U)
#define CSL_MCSPI_IRQSTATUS_TX1_EMPTY_RESETVAL                  (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_TX1_EMPTY_NOEVNT_R                  (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_TX1_EMPTY_EVNT_R                    (0x00000001U)
#define CSL_MCSPI_IRQSTATUS_TX1_EMPTY_NOEFFECT_W                (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_TX1_EMPTY_CLEARSRC_W                (0x00000001U)

#define CSL_MCSPI_IRQSTATUS_TX0_UNDERFLOW_MASK                  (0x00000002U)
#define CSL_MCSPI_IRQSTATUS_TX0_UNDERFLOW_SHIFT                 (1U)
#define CSL_MCSPI_IRQSTATUS_TX0_UNDERFLOW_RESETVAL              (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_TX0_UNDERFLOW_NOEVNT_R              (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_TX0_UNDERFLOW_EVNT_R                (0x00000001U)
#define CSL_MCSPI_IRQSTATUS_TX0_UNDERFLOW_NOEFFECT_W            (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_TX0_UNDERFLOW_CLEARSRC_W            (0x00000001U)

#define CSL_MCSPI_IRQSTATUS_TX1_UNDERFLOW_MASK                  (0x00000020U)
#define CSL_MCSPI_IRQSTATUS_TX1_UNDERFLOW_SHIFT                 (5U)
#define CSL_MCSPI_IRQSTATUS_TX1_UNDERFLOW_RESETVAL              (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_TX1_UNDERFLOW_NOEVNT_R              (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_TX1_UNDERFLOW_EVNT_R                (0x00000001U)
#define CSL_MCSPI_IRQSTATUS_TX1_UNDERFLOW_NOEFFECT_W            (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_TX1_UNDERFLOW_CLEARSRC_W            (0x00000001U)

#define CSL_MCSPI_IRQSTATUS_RX0_FULL_MASK                       (0x00000004U)
#define CSL_MCSPI_IRQSTATUS_RX0_FULL_SHIFT                      (2U)
#define CSL_MCSPI_IRQSTATUS_RX0_FULL_RESETVAL                   (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_RX0_FULL_NOEVNT_R                   (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_RX0_FULL_EVNT_R                     (0x00000001U)
#define CSL_MCSPI_IRQSTATUS_RX0_FULL_NOEFFECT_W                 (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_RX0_FULL_CLEARSRC_W                 (0x00000001U)

#define CSL_MCSPI_IRQSTATUS_RX0_OVERFLOW_MASK                   (0x00000008U)
#define CSL_MCSPI_IRQSTATUS_RX0_OVERFLOW_SHIFT                  (3U)
#define CSL_MCSPI_IRQSTATUS_RX0_OVERFLOW_RESETVAL               (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_RX0_OVERFLOW_NOEVNT_R               (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_RX0_OVERFLOW_EVNT_R                 (0x00000001U)
#define CSL_MCSPI_IRQSTATUS_RX0_OVERFLOW_NOEFFECT_W             (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_RX0_OVERFLOW_CLEARSRC_W             (0x00000001U)

#define CSL_MCSPI_IRQSTATUS_TX0_EMPTY_MASK                      (0x00000001U)
#define CSL_MCSPI_IRQSTATUS_TX0_EMPTY_SHIFT                     (0U)
#define CSL_MCSPI_IRQSTATUS_TX0_EMPTY_RESETVAL                  (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_TX0_EMPTY_NOEVNT_R                  (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_TX0_EMPTY_EVNT_R                    (0x00000001U)
#define CSL_MCSPI_IRQSTATUS_TX0_EMPTY_NOEFFECT_W                (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_TX0_EMPTY_CLEARSRC_W                (0x00000001U)

#define CSL_MCSPI_IRQSTATUS_EOW_MASK                            (0x00020000U)
#define CSL_MCSPI_IRQSTATUS_EOW_SHIFT                           (17U)
#define CSL_MCSPI_IRQSTATUS_EOW_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_EOW_NOEVNT_R                        (0x00000000U)
#define CSL_MCSPI_IRQSTATUS_EOW_EVNT_R                          (0x00000001U)
#define CSL_MCSPI_IRQSTATUS_EOW_CLEARSRC_W                      (0x00000001U)
#define CSL_MCSPI_IRQSTATUS_EOW_NOEFFECT_W                      (0x00000000U)

#define CSL_MCSPI_IRQSTATUS_RESETVAL                            (0x00000000U)

/* IRQENABLE */

#define CSL_MCSPI_IRQENABLE_TX2_UNDERFLOW_ENABLE_MASK           (0x00000200U)
#define CSL_MCSPI_IRQENABLE_TX2_UNDERFLOW_ENABLE_SHIFT          (9U)
#define CSL_MCSPI_IRQENABLE_TX2_UNDERFLOW_ENABLE_RESETVAL       (0x00000000U)
#define CSL_MCSPI_IRQENABLE_TX2_UNDERFLOW_ENABLE_IRQDISABLED    (0x00000000U)
#define CSL_MCSPI_IRQENABLE_TX2_UNDERFLOW_ENABLE_IRQENABLED     (0x00000001U)

#define CSL_MCSPI_IRQENABLE_TX0_EMPTY_ENABLE_MASK               (0x00000001U)
#define CSL_MCSPI_IRQENABLE_TX0_EMPTY_ENABLE_SHIFT              (0U)
#define CSL_MCSPI_IRQENABLE_TX0_EMPTY_ENABLE_RESETVAL           (0x00000000U)
#define CSL_MCSPI_IRQENABLE_TX0_EMPTY_ENABLE_IRQDISABLED        (0x00000000U)
#define CSL_MCSPI_IRQENABLE_TX0_EMPTY_ENABLE_IRQENABLED         (0x00000001U)

#define CSL_MCSPI_IRQENABLE_RX0_FULL_ENABLE_MASK                (0x00000004U)
#define CSL_MCSPI_IRQENABLE_RX0_FULL_ENABLE_SHIFT               (2U)
#define CSL_MCSPI_IRQENABLE_RX0_FULL_ENABLE_RESETVAL            (0x00000000U)
#define CSL_MCSPI_IRQENABLE_RX0_FULL_ENABLE_IRQDISABLED         (0x00000000U)
#define CSL_MCSPI_IRQENABLE_RX0_FULL_ENABLE_IRQENABLED          (0x00000001U)

#define CSL_MCSPI_IRQENABLE_TX1_EMPTY_ENABLE_MASK               (0x00000010U)
#define CSL_MCSPI_IRQENABLE_TX1_EMPTY_ENABLE_SHIFT              (4U)
#define CSL_MCSPI_IRQENABLE_TX1_EMPTY_ENABLE_RESETVAL           (0x00000000U)
#define CSL_MCSPI_IRQENABLE_TX1_EMPTY_ENABLE_IRQDISABLED        (0x00000000U)
#define CSL_MCSPI_IRQENABLE_TX1_EMPTY_ENABLE_IRQENABLED         (0x00000001U)

#define CSL_MCSPI_IRQENABLE_RX1_FULL_ENABLE_MASK                (0x00000040U)
#define CSL_MCSPI_IRQENABLE_RX1_FULL_ENABLE_SHIFT               (6U)
#define CSL_MCSPI_IRQENABLE_RX1_FULL_ENABLE_RESETVAL            (0x00000000U)
#define CSL_MCSPI_IRQENABLE_RX1_FULL_ENABLE_IRQDISABLED         (0x00000000U)
#define CSL_MCSPI_IRQENABLE_RX1_FULL_ENABLE_IRQENABLED          (0x00000001U)

#define CSL_MCSPI_IRQENABLE_TX3_EMPTY_ENABLE_MASK               (0x00001000U)
#define CSL_MCSPI_IRQENABLE_TX3_EMPTY_ENABLE_SHIFT              (12U)
#define CSL_MCSPI_IRQENABLE_TX3_EMPTY_ENABLE_RESETVAL           (0x00000000U)
#define CSL_MCSPI_IRQENABLE_TX3_EMPTY_ENABLE_IRQDISABLED        (0x00000000U)
#define CSL_MCSPI_IRQENABLE_TX3_EMPTY_ENABLE_IRQENABLED         (0x00000001U)

#define CSL_MCSPI_IRQENABLE_TX0_UNDERFLOW_ENABLE_MASK           (0x00000002U)
#define CSL_MCSPI_IRQENABLE_TX0_UNDERFLOW_ENABLE_SHIFT          (1U)
#define CSL_MCSPI_IRQENABLE_TX0_UNDERFLOW_ENABLE_RESETVAL       (0x00000000U)
#define CSL_MCSPI_IRQENABLE_TX0_UNDERFLOW_ENABLE_IRQDISABLED    (0x00000000U)
#define CSL_MCSPI_IRQENABLE_TX0_UNDERFLOW_ENABLE_IRQENABLED     (0x00000001U)

#define CSL_MCSPI_IRQENABLE_RX0_OVERFLOW_ENABLE_MASK            (0x00000008U)
#define CSL_MCSPI_IRQENABLE_RX0_OVERFLOW_ENABLE_SHIFT           (3U)
#define CSL_MCSPI_IRQENABLE_RX0_OVERFLOW_ENABLE_RESETVAL        (0x00000000U)
#define CSL_MCSPI_IRQENABLE_RX0_OVERFLOW_ENABLE_IRQDISABLED     (0x00000000U)
#define CSL_MCSPI_IRQENABLE_RX0_OVERFLOW_ENABLE_IRQENABLED      (0x00000001U)

#define CSL_MCSPI_IRQENABLE_RX3_FULL_ENABLE_MASK                (0x00004000U)
#define CSL_MCSPI_IRQENABLE_RX3_FULL_ENABLE_SHIFT               (14U)
#define CSL_MCSPI_IRQENABLE_RX3_FULL_ENABLE_RESETVAL            (0x00000000U)
#define CSL_MCSPI_IRQENABLE_RX3_FULL_ENABLE_IRQDISABLED         (0x00000000U)
#define CSL_MCSPI_IRQENABLE_RX3_FULL_ENABLE_IRQENABLED          (0x00000001U)

#define CSL_MCSPI_IRQENABLE_WKE_MASK                            (0x00010000U)
#define CSL_MCSPI_IRQENABLE_WKE_SHIFT                           (16U)
#define CSL_MCSPI_IRQENABLE_WKE_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_IRQENABLE_WKE_IRQDISABLED                     (0x00000000U)
#define CSL_MCSPI_IRQENABLE_WKE_IRQENABLED                      (0x00000001U)

#define CSL_MCSPI_IRQENABLE_TX2_EMPTY_ENABLE_MASK               (0x00000100U)
#define CSL_MCSPI_IRQENABLE_TX2_EMPTY_ENABLE_SHIFT              (8U)
#define CSL_MCSPI_IRQENABLE_TX2_EMPTY_ENABLE_RESETVAL           (0x00000000U)
#define CSL_MCSPI_IRQENABLE_TX2_EMPTY_ENABLE_IRQDISABLED        (0x00000000U)
#define CSL_MCSPI_IRQENABLE_TX2_EMPTY_ENABLE_IRQENABLED         (0x00000001U)

#define CSL_MCSPI_IRQENABLE_RX2_FULL_ENABLE_MASK                (0x00000400U)
#define CSL_MCSPI_IRQENABLE_RX2_FULL_ENABLE_SHIFT               (10U)
#define CSL_MCSPI_IRQENABLE_RX2_FULL_ENABLE_RESETVAL            (0x00000000U)
#define CSL_MCSPI_IRQENABLE_RX2_FULL_ENABLE_IRQDISABLED         (0x00000000U)
#define CSL_MCSPI_IRQENABLE_RX2_FULL_ENABLE_IRQENABLED          (0x00000001U)

#define CSL_MCSPI_IRQENABLE_TX3_UNDERFLOW_ENABLE_MASK           (0x00002000U)
#define CSL_MCSPI_IRQENABLE_TX3_UNDERFLOW_ENABLE_SHIFT          (13U)
#define CSL_MCSPI_IRQENABLE_TX3_UNDERFLOW_ENABLE_RESETVAL       (0x00000000U)
#define CSL_MCSPI_IRQENABLE_TX3_UNDERFLOW_ENABLE_IRQDISABLED    (0x00000000U)
#define CSL_MCSPI_IRQENABLE_TX3_UNDERFLOW_ENABLE_IRQENABLED     (0x00000001U)

#define CSL_MCSPI_IRQENABLE_TX1_UNDERFLOW_ENABLE_MASK           (0x00000020U)
#define CSL_MCSPI_IRQENABLE_TX1_UNDERFLOW_ENABLE_SHIFT          (5U)
#define CSL_MCSPI_IRQENABLE_TX1_UNDERFLOW_ENABLE_RESETVAL       (0x00000000U)
#define CSL_MCSPI_IRQENABLE_TX1_UNDERFLOW_ENABLE_IRQDISABLED    (0x00000000U)
#define CSL_MCSPI_IRQENABLE_TX1_UNDERFLOW_ENABLE_IRQENABLED     (0x00000001U)

#define CSL_MCSPI_IRQENABLE_EOW_ENABLE_MASK                     (0x00020000U)
#define CSL_MCSPI_IRQENABLE_EOW_ENABLE_SHIFT                    (17U)
#define CSL_MCSPI_IRQENABLE_EOW_ENABLE_RESETVAL                 (0x00000000U)
#define CSL_MCSPI_IRQENABLE_EOW_ENABLE_IRQENABLED               (0x00000001U)
#define CSL_MCSPI_IRQENABLE_EOW_ENABLE_IRQDISABLED              (0x00000000U)

#define CSL_MCSPI_IRQENABLE_RESETVAL                            (0x00000000U)

/* WAKEUPENABLE */

#define CSL_MCSPI_WAKEUPENABLE_WKEN_MASK                        (0x00000001U)
#define CSL_MCSPI_WAKEUPENABLE_WKEN_SHIFT                       (0U)
#define CSL_MCSPI_WAKEUPENABLE_WKEN_RESETVAL                    (0x00000000U)
#define CSL_MCSPI_WAKEUPENABLE_WKEN_NOWAKEUP                    (0x00000000U)
#define CSL_MCSPI_WAKEUPENABLE_WKEN_WAKEUP                      (0x00000001U)

#define CSL_MCSPI_WAKEUPENABLE_RESETVAL                         (0x00000000U)

/* SYST */

#define CSL_MCSPI_SYST_SPIEN_2_MASK                             (0x00000004U)
#define CSL_MCSPI_SYST_SPIEN_2_SHIFT                            (2U)
#define CSL_MCSPI_SYST_SPIEN_2_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_SYST_SPIEN_2_MAX                              (0x00000001U)

#define CSL_MCSPI_SYST_WAKD_MASK                                (0x00000080U)
#define CSL_MCSPI_SYST_WAKD_SHIFT                               (7U)
#define CSL_MCSPI_SYST_WAKD_RESETVAL                            (0x00000000U)
#define CSL_MCSPI_SYST_WAKD_DRIVENLOW                           (0x00000000U)
#define CSL_MCSPI_SYST_WAKD_DRIVENHIGH                          (0x00000001U)

#define CSL_MCSPI_SYST_SPIDAT_0_MASK                            (0x00000010U)
#define CSL_MCSPI_SYST_SPIDAT_0_SHIFT                           (4U)
#define CSL_MCSPI_SYST_SPIDAT_0_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_SYST_SPIDAT_0_MAX                             (0x00000001U)

#define CSL_MCSPI_SYST_SPIEN_1_MASK                             (0x00000002U)
#define CSL_MCSPI_SYST_SPIEN_1_SHIFT                            (1U)
#define CSL_MCSPI_SYST_SPIEN_1_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_SYST_SPIEN_1_MAX                              (0x00000001U)

#define CSL_MCSPI_SYST_SPIDATDIR1_MASK                          (0x00000200U)
#define CSL_MCSPI_SYST_SPIDATDIR1_SHIFT                         (9U)
#define CSL_MCSPI_SYST_SPIDATDIR1_RESETVAL                      (0x00000000U)
#define CSL_MCSPI_SYST_SPIDATDIR1_OUT                           (0x00000000U)
#define CSL_MCSPI_SYST_SPIDATDIR1_IN                            (0x00000001U)

#define CSL_MCSPI_SYST_SSB_MASK                                 (0x00000800U)
#define CSL_MCSPI_SYST_SSB_SHIFT                                (11U)
#define CSL_MCSPI_SYST_SSB_RESETVAL                             (0x00000000U)
#define CSL_MCSPI_SYST_SSB_OFF                                  (0x00000000U)
#define CSL_MCSPI_SYST_SSB_SETTHEMALL                           (0x00000001U)

#define CSL_MCSPI_SYST_SPICLK_MASK                              (0x00000040U)
#define CSL_MCSPI_SYST_SPICLK_SHIFT                             (6U)
#define CSL_MCSPI_SYST_SPICLK_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_SYST_SPICLK_MAX                               (0x00000001U)

#define CSL_MCSPI_SYST_SPIDATDIR0_MASK                          (0x00000100U)
#define CSL_MCSPI_SYST_SPIDATDIR0_SHIFT                         (8U)
#define CSL_MCSPI_SYST_SPIDATDIR0_RESETVAL                      (0x00000000U)
#define CSL_MCSPI_SYST_SPIDATDIR0_OUT                           (0x00000000U)
#define CSL_MCSPI_SYST_SPIDATDIR0_IN                            (0x00000001U)

#define CSL_MCSPI_SYST_SPIENDIR_MASK                            (0x00000400U)
#define CSL_MCSPI_SYST_SPIENDIR_SHIFT                           (10U)
#define CSL_MCSPI_SYST_SPIENDIR_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_SYST_SPIENDIR_OUT                             (0x00000000U)
#define CSL_MCSPI_SYST_SPIENDIR_IN                              (0x00000001U)

#define CSL_MCSPI_SYST_SPIEN_3_MASK                             (0x00000008U)
#define CSL_MCSPI_SYST_SPIEN_3_SHIFT                            (3U)
#define CSL_MCSPI_SYST_SPIEN_3_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_SYST_SPIEN_3_MAX                              (0x00000001U)

#define CSL_MCSPI_SYST_SPIEN_0_MASK                             (0x00000001U)
#define CSL_MCSPI_SYST_SPIEN_0_SHIFT                            (0U)
#define CSL_MCSPI_SYST_SPIEN_0_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_SYST_SPIEN_0_MAX                              (0x00000001U)

#define CSL_MCSPI_SYST_SPIDAT_1_MASK                            (0x00000020U)
#define CSL_MCSPI_SYST_SPIDAT_1_SHIFT                           (5U)
#define CSL_MCSPI_SYST_SPIDAT_1_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_SYST_SPIDAT_1_MAX                             (0x00000001U)

#define CSL_MCSPI_SYST_RESETVAL                                 (0x00000000U)

/* MODULCTRL */

#define CSL_MCSPI_MODULCTRL_MS_MASK                             (0x00000004U)
#define CSL_MCSPI_MODULCTRL_MS_SHIFT                            (2U)
#define CSL_MCSPI_MODULCTRL_MS_RESETVAL                         (0x00000001U)
#define CSL_MCSPI_MODULCTRL_MS_MASTER                           (0x00000000U)
#define CSL_MCSPI_MODULCTRL_MS_SLAVE                            (0x00000001U)

#define CSL_MCSPI_MODULCTRL_SYSTEM_TEST_MASK                    (0x00000008U)
#define CSL_MCSPI_MODULCTRL_SYSTEM_TEST_SHIFT                   (3U)
#define CSL_MCSPI_MODULCTRL_SYSTEM_TEST_RESETVAL                (0x00000000U)
#define CSL_MCSPI_MODULCTRL_SYSTEM_TEST_OFF                     (0x00000000U)
#define CSL_MCSPI_MODULCTRL_SYSTEM_TEST_ON                      (0x00000001U)

#define CSL_MCSPI_MODULCTRL_SINGLE_MASK                         (0x00000001U)
#define CSL_MCSPI_MODULCTRL_SINGLE_SHIFT                        (0U)
#define CSL_MCSPI_MODULCTRL_SINGLE_RESETVAL                     (0x00000000U)
#define CSL_MCSPI_MODULCTRL_SINGLE_MULTI                        (0x00000000U)
#define CSL_MCSPI_MODULCTRL_SINGLE_SINGLE                       (0x00000001U)

#define CSL_MCSPI_MODULCTRL_PIN34_MASK                          (0x00000002U)
#define CSL_MCSPI_MODULCTRL_PIN34_SHIFT                         (1U)
#define CSL_MCSPI_MODULCTRL_PIN34_RESETVAL                      (0x00000000U)
#define CSL_MCSPI_MODULCTRL_PIN34_3PINMODE                      (0x00000001U)
#define CSL_MCSPI_MODULCTRL_PIN34_4PINMODE                      (0x00000000U)

#define CSL_MCSPI_MODULCTRL_INITDLY_MASK                        (0x00000070U)
#define CSL_MCSPI_MODULCTRL_INITDLY_SHIFT                       (4U)
#define CSL_MCSPI_MODULCTRL_INITDLY_RESETVAL                    (0x00000000U)
#define CSL_MCSPI_MODULCTRL_INITDLY_4CLKDLY                     (0x00000001U)
#define CSL_MCSPI_MODULCTRL_INITDLY_NODELAY                     (0x00000000U)
#define CSL_MCSPI_MODULCTRL_INITDLY_8CLKDLY                     (0x00000002U)
#define CSL_MCSPI_MODULCTRL_INITDLY_16CLKDLY                    (0x00000003U)
#define CSL_MCSPI_MODULCTRL_INITDLY_32CLKDLY                    (0x00000004U)

#define CSL_MCSPI_MODULCTRL_MOA_MASK                            (0x00000080U)
#define CSL_MCSPI_MODULCTRL_MOA_SHIFT                           (7U)
#define CSL_MCSPI_MODULCTRL_MOA_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_MODULCTRL_MOA_MULTIACCES                      (0x00000001U)
#define CSL_MCSPI_MODULCTRL_MOA_NOMULTIACCESS                   (0x00000000U)

#define CSL_MCSPI_MODULCTRL_FDAA_MASK                           (0x00000100U)
#define CSL_MCSPI_MODULCTRL_FDAA_SHIFT                          (8U)
#define CSL_MCSPI_MODULCTRL_FDAA_RESETVAL                       (0x00000000U)
#define CSL_MCSPI_MODULCTRL_FDAA_SHADOWREGEN                    (0x00000001U)
#define CSL_MCSPI_MODULCTRL_FDAA_NOSHADOWREG                    (0x00000000U)

#define CSL_MCSPI_MODULCTRL_RESETVAL                            (0x00000004U)

/* CH0CONF */

#define CSL_MCSPI_CH0CONF_CLKD_MASK                             (0x0000003CU)
#define CSL_MCSPI_CH0CONF_CLKD_SHIFT                            (2U)
#define CSL_MCSPI_CH0CONF_CLKD_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH0CONF_CLKD_MAX                              (0x0000000fU)

#define CSL_MCSPI_CH0CONF_PHA_MASK                              (0x00000001U)
#define CSL_MCSPI_CH0CONF_PHA_SHIFT                             (0U)
#define CSL_MCSPI_CH0CONF_PHA_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH0CONF_PHA_ODD                               (0x00000000U)
#define CSL_MCSPI_CH0CONF_PHA_EVEN                              (0x00000001U)

#define CSL_MCSPI_CH0CONF_TURBO_MASK                            (0x00080000U)
#define CSL_MCSPI_CH0CONF_TURBO_SHIFT                           (19U)
#define CSL_MCSPI_CH0CONF_TURBO_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH0CONF_TURBO_TURBO                           (0x00000001U)
#define CSL_MCSPI_CH0CONF_TURBO_OFF                             (0x00000000U)

#define CSL_MCSPI_CH0CONF_SPIENSLV_MASK                         (0x00600000U)
#define CSL_MCSPI_CH0CONF_SPIENSLV_SHIFT                        (21U)
#define CSL_MCSPI_CH0CONF_SPIENSLV_RESETVAL                     (0x00000000U)
#define CSL_MCSPI_CH0CONF_SPIENSLV_SPIEN2                       (0x00000002U)
#define CSL_MCSPI_CH0CONF_SPIENSLV_SPIEN3                       (0x00000003U)
#define CSL_MCSPI_CH0CONF_SPIENSLV_SPIEN0                       (0x00000000U)
#define CSL_MCSPI_CH0CONF_SPIENSLV_SPIEN1                       (0x00000001U)

#define CSL_MCSPI_CH0CONF_DPE0_MASK                             (0x00010000U)
#define CSL_MCSPI_CH0CONF_DPE0_SHIFT                            (16U)
#define CSL_MCSPI_CH0CONF_DPE0_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH0CONF_DPE0_DISABLED                         (0x00000001U)
#define CSL_MCSPI_CH0CONF_DPE0_ENABLED                          (0x00000000U)

#define CSL_MCSPI_CH0CONF_IS_MASK                               (0x00040000U)
#define CSL_MCSPI_CH0CONF_IS_SHIFT                              (18U)
#define CSL_MCSPI_CH0CONF_IS_RESETVAL                           (0x00000001U)
#define CSL_MCSPI_CH0CONF_IS_LINE0                              (0x00000000U)
#define CSL_MCSPI_CH0CONF_IS_LINE1                              (0x00000001U)

#define CSL_MCSPI_CH0CONF_DMAR_MASK                             (0x00008000U)
#define CSL_MCSPI_CH0CONF_DMAR_SHIFT                            (15U)
#define CSL_MCSPI_CH0CONF_DMAR_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH0CONF_DMAR_DISABLED                         (0x00000000U)
#define CSL_MCSPI_CH0CONF_DMAR_ENABLED                          (0x00000001U)

#define CSL_MCSPI_CH0CONF_FORCE_MASK                            (0x00100000U)
#define CSL_MCSPI_CH0CONF_FORCE_SHIFT                           (20U)
#define CSL_MCSPI_CH0CONF_FORCE_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH0CONF_FORCE_DEASSERT                        (0x00000000U)
#define CSL_MCSPI_CH0CONF_FORCE_ASSERT                          (0x00000001U)

#define CSL_MCSPI_CH0CONF_WL_MASK                               (0x00000F80U)
#define CSL_MCSPI_CH0CONF_WL_SHIFT                              (7U)
#define CSL_MCSPI_CH0CONF_WL_RESETVAL                           (0x00000000U)
#define CSL_MCSPI_CH0CONF_WL_MAX                                (0x0000001fU)

#define CSL_MCSPI_CH0CONF_DPE1_MASK                             (0x00020000U)
#define CSL_MCSPI_CH0CONF_DPE1_SHIFT                            (17U)
#define CSL_MCSPI_CH0CONF_DPE1_RESETVAL                         (0x00000001U)
#define CSL_MCSPI_CH0CONF_DPE1_ENABLED                          (0x00000000U)
#define CSL_MCSPI_CH0CONF_DPE1_DISABLED                         (0x00000001U)

#define CSL_MCSPI_CH0CONF_EPOL_MASK                             (0x00000040U)
#define CSL_MCSPI_CH0CONF_EPOL_SHIFT                            (6U)
#define CSL_MCSPI_CH0CONF_EPOL_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH0CONF_EPOL_ACTIVELOW                        (0x00000001U)
#define CSL_MCSPI_CH0CONF_EPOL_ACTIVEHIGH                       (0x00000000U)

#define CSL_MCSPI_CH0CONF_DMAW_MASK                             (0x00004000U)
#define CSL_MCSPI_CH0CONF_DMAW_SHIFT                            (14U)
#define CSL_MCSPI_CH0CONF_DMAW_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH0CONF_DMAW_ENABLED                          (0x00000001U)
#define CSL_MCSPI_CH0CONF_DMAW_DISABLED                         (0x00000000U)

#define CSL_MCSPI_CH0CONF_TRM_MASK                              (0x00003000U)
#define CSL_MCSPI_CH0CONF_TRM_SHIFT                             (12U)
#define CSL_MCSPI_CH0CONF_TRM_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH0CONF_TRM_TRANSONLY                         (0x00000002U)
#define CSL_MCSPI_CH0CONF_TRM_RSVD                              (0x00000003U)
#define CSL_MCSPI_CH0CONF_TRM_TRANSRECEI                        (0x00000000U)
#define CSL_MCSPI_CH0CONF_TRM_RECEIVONLY                        (0x00000001U)

#define CSL_MCSPI_CH0CONF_POL_MASK                              (0x00000002U)
#define CSL_MCSPI_CH0CONF_POL_SHIFT                             (1U)
#define CSL_MCSPI_CH0CONF_POL_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH0CONF_POL_ACTIVELOW                         (0x00000001U)
#define CSL_MCSPI_CH0CONF_POL_ACTIVEHIGH                        (0x00000000U)

#define CSL_MCSPI_CH0CONF_SBE_MASK                              (0x00800000U)
#define CSL_MCSPI_CH0CONF_SBE_SHIFT                             (23U)
#define CSL_MCSPI_CH0CONF_SBE_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH0CONF_SBE_DISABLED                          (0x00000000U)
#define CSL_MCSPI_CH0CONF_SBE_ENABLED                           (0x00000001U)

#define CSL_MCSPI_CH0CONF_SBPOL_MASK                            (0x01000000U)
#define CSL_MCSPI_CH0CONF_SBPOL_SHIFT                           (24U)
#define CSL_MCSPI_CH0CONF_SBPOL_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH0CONF_SBPOL_LOWLEVEL                        (0x00000000U)
#define CSL_MCSPI_CH0CONF_SBPOL_HIGHLEVEL                       (0x00000001U)

#define CSL_MCSPI_CH0CONF_TCS0_MASK                             (0x06000000U)
#define CSL_MCSPI_CH0CONF_TCS0_SHIFT                            (25U)
#define CSL_MCSPI_CH0CONF_TCS0_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH0CONF_TCS0_THREECYCLEDLY                    (0x00000003U)
#define CSL_MCSPI_CH0CONF_TCS0_TWOCYCLEDLY                      (0x00000002U)
#define CSL_MCSPI_CH0CONF_TCS0_ONECYCLEDLY                      (0x00000001U)
#define CSL_MCSPI_CH0CONF_TCS0_ZEROCYCLEDLY                     (0x00000000U)

#define CSL_MCSPI_CH0CONF_FFER_MASK                             (0x10000000U)
#define CSL_MCSPI_CH0CONF_FFER_SHIFT                            (28U)
#define CSL_MCSPI_CH0CONF_FFER_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH0CONF_FFER_FFENABLED                        (0x00000001U)
#define CSL_MCSPI_CH0CONF_FFER_FFDISABLED                       (0x00000000U)

#define CSL_MCSPI_CH0CONF_FFEW_MASK                             (0x08000000U)
#define CSL_MCSPI_CH0CONF_FFEW_SHIFT                            (27U)
#define CSL_MCSPI_CH0CONF_FFEW_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH0CONF_FFEW_FFENABLED                        (0x00000001U)
#define CSL_MCSPI_CH0CONF_FFEW_FFDISABLED                       (0x00000000U)

#define CSL_MCSPI_CH0CONF_CLKG_MASK                             (0x20000000U)
#define CSL_MCSPI_CH0CONF_CLKG_SHIFT                            (29U)
#define CSL_MCSPI_CH0CONF_CLKG_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH0CONF_CLKG_ONECYCLE                         (0x00000001U)
#define CSL_MCSPI_CH0CONF_CLKG_POWERTWO                         (0x00000000U)

#define CSL_MCSPI_CH0CONF_RESETVAL                              (0x00060000U)

/* CH0STAT */

#define CSL_MCSPI_CH0STAT_RXS_MASK                              (0x00000001U)
#define CSL_MCSPI_CH0STAT_RXS_SHIFT                             (0U)
#define CSL_MCSPI_CH0STAT_RXS_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH0STAT_RXS_EMPTY                             (0x00000000U)
#define CSL_MCSPI_CH0STAT_RXS_FULL                              (0x00000001U)

#define CSL_MCSPI_CH0STAT_EOT_MASK                              (0x00000004U)
#define CSL_MCSPI_CH0STAT_EOT_SHIFT                             (2U)
#define CSL_MCSPI_CH0STAT_EOT_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH0STAT_EOT_COMPLETED                         (0x00000001U)
#define CSL_MCSPI_CH0STAT_EOT_INPROGRESS                        (0x00000000U)

#define CSL_MCSPI_CH0STAT_TXS_MASK                              (0x00000002U)
#define CSL_MCSPI_CH0STAT_TXS_SHIFT                             (1U)
#define CSL_MCSPI_CH0STAT_TXS_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH0STAT_TXS_EMPTY                             (0x00000001U)
#define CSL_MCSPI_CH0STAT_TXS_FULL                              (0x00000000U)

#define CSL_MCSPI_CH0STAT_RXFFF_MASK                            (0x00000040U)
#define CSL_MCSPI_CH0STAT_RXFFF_SHIFT                           (6U)
#define CSL_MCSPI_CH0STAT_RXFFF_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH0STAT_RXFFF_FULL                            (0x00000001U)
#define CSL_MCSPI_CH0STAT_RXFFF_NOTFULL                         (0x00000000U)

#define CSL_MCSPI_CH0STAT_TXFFF_MASK                            (0x00000010U)
#define CSL_MCSPI_CH0STAT_TXFFF_SHIFT                           (4U)
#define CSL_MCSPI_CH0STAT_TXFFF_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH0STAT_TXFFF_FULL                            (0x00000001U)
#define CSL_MCSPI_CH0STAT_TXFFF_NOTFULL                         (0x00000000U)

#define CSL_MCSPI_CH0STAT_RXFFE_MASK                            (0x00000020U)
#define CSL_MCSPI_CH0STAT_RXFFE_SHIFT                           (5U)
#define CSL_MCSPI_CH0STAT_RXFFE_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH0STAT_RXFFE_EMPTY                           (0x00000001U)
#define CSL_MCSPI_CH0STAT_RXFFE_NOTEMPTY                        (0x00000000U)

#define CSL_MCSPI_CH0STAT_TXFFE_MASK                            (0x00000008U)
#define CSL_MCSPI_CH0STAT_TXFFE_SHIFT                           (3U)
#define CSL_MCSPI_CH0STAT_TXFFE_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH0STAT_TXFFE_EMPTY                           (0x00000001U)
#define CSL_MCSPI_CH0STAT_TXFFE_NOTEMPTY                        (0x00000000U)

#define CSL_MCSPI_CH0STAT_RESETVAL                              (0x00000000U)

/* CH0CTRL */

#define CSL_MCSPI_CH0CTRL_EN_MASK                               (0x00000001U)
#define CSL_MCSPI_CH0CTRL_EN_SHIFT                              (0U)
#define CSL_MCSPI_CH0CTRL_EN_RESETVAL                           (0x00000000U)
#define CSL_MCSPI_CH0CTRL_EN_ACT                                (0x00000001U)
#define CSL_MCSPI_CH0CTRL_EN_NACT                               (0x00000000U)

#define CSL_MCSPI_CH0CTRL_EXTCLK_MASK                           (0x0000FF00U)
#define CSL_MCSPI_CH0CTRL_EXTCLK_SHIFT                          (8U)
#define CSL_MCSPI_CH0CTRL_EXTCLK_RESETVAL                       (0x00000000U)
#define CSL_MCSPI_CH0CTRL_EXTCLK_MAX                            (0x000000ffU)

#define CSL_MCSPI_CH0CTRL_RESETVAL                              (0x00000000U)

/* TX0 */

#define CSL_MCSPI_TX0_TDATA_MASK                                (0xFFFFFFFFU)
#define CSL_MCSPI_TX0_TDATA_SHIFT                               (0U)
#define CSL_MCSPI_TX0_TDATA_RESETVAL                            (0x00000000U)
#define CSL_MCSPI_TX0_TDATA_MAX                                 (0xffffffffU)

#define CSL_MCSPI_TX0_RESETVAL                                  (0x00000000U)

/* RX0 */

#define CSL_MCSPI_RX0_RDATA_MASK                                (0xFFFFFFFFU)
#define CSL_MCSPI_RX0_RDATA_SHIFT                               (0U)
#define CSL_MCSPI_RX0_RDATA_RESETVAL                            (0x00000000U)
#define CSL_MCSPI_RX0_RDATA_MAX                                 (0xffffffffU)

#define CSL_MCSPI_RX0_RESETVAL                                  (0x00000000U)

/* CH1CONF */

#define CSL_MCSPI_CH1CONF_CLKD_MASK                             (0x0000003CU)
#define CSL_MCSPI_CH1CONF_CLKD_SHIFT                            (2U)
#define CSL_MCSPI_CH1CONF_CLKD_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH1CONF_CLKD_MAX                              (0x0000000fU)

#define CSL_MCSPI_CH1CONF_PHA_MASK                              (0x00000001U)
#define CSL_MCSPI_CH1CONF_PHA_SHIFT                             (0U)
#define CSL_MCSPI_CH1CONF_PHA_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH1CONF_PHA_ODD                               (0x00000000U)
#define CSL_MCSPI_CH1CONF_PHA_EVEN                              (0x00000001U)

#define CSL_MCSPI_CH1CONF_TURBO_MASK                            (0x00080000U)
#define CSL_MCSPI_CH1CONF_TURBO_SHIFT                           (19U)
#define CSL_MCSPI_CH1CONF_TURBO_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH1CONF_TURBO_TURBO                           (0x00000001U)
#define CSL_MCSPI_CH1CONF_TURBO_OFF                             (0x00000000U)

#define CSL_MCSPI_CH1CONF_DPE0_MASK                             (0x00010000U)
#define CSL_MCSPI_CH1CONF_DPE0_SHIFT                            (16U)
#define CSL_MCSPI_CH1CONF_DPE0_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH1CONF_DPE0_DISABLED                         (0x00000001U)
#define CSL_MCSPI_CH1CONF_DPE0_ENABLED                          (0x00000000U)

#define CSL_MCSPI_CH1CONF_IS_MASK                               (0x00040000U)
#define CSL_MCSPI_CH1CONF_IS_SHIFT                              (18U)
#define CSL_MCSPI_CH1CONF_IS_RESETVAL                           (0x00000001U)
#define CSL_MCSPI_CH1CONF_IS_LINE0                              (0x00000000U)
#define CSL_MCSPI_CH1CONF_IS_LINE1                              (0x00000001U)

#define CSL_MCSPI_CH1CONF_DMAR_MASK                             (0x00008000U)
#define CSL_MCSPI_CH1CONF_DMAR_SHIFT                            (15U)
#define CSL_MCSPI_CH1CONF_DMAR_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH1CONF_DMAR_DISABLED                         (0x00000000U)
#define CSL_MCSPI_CH1CONF_DMAR_ENABLED                          (0x00000001U)

#define CSL_MCSPI_CH1CONF_FORCE_MASK                            (0x00100000U)
#define CSL_MCSPI_CH1CONF_FORCE_SHIFT                           (20U)
#define CSL_MCSPI_CH1CONF_FORCE_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH1CONF_FORCE_DEASSERT                        (0x00000000U)
#define CSL_MCSPI_CH1CONF_FORCE_ASSERT                          (0x00000001U)

#define CSL_MCSPI_CH1CONF_WL_MASK                               (0x00000F80U)
#define CSL_MCSPI_CH1CONF_WL_SHIFT                              (7U)
#define CSL_MCSPI_CH1CONF_WL_RESETVAL                           (0x00000000U)
#define CSL_MCSPI_CH1CONF_WL_MAX                                (0x0000001fU)

#define CSL_MCSPI_CH1CONF_DPE1_MASK                             (0x00020000U)
#define CSL_MCSPI_CH1CONF_DPE1_SHIFT                            (17U)
#define CSL_MCSPI_CH1CONF_DPE1_RESETVAL                         (0x00000001U)
#define CSL_MCSPI_CH1CONF_DPE1_ENABLED                          (0x00000000U)
#define CSL_MCSPI_CH1CONF_DPE1_DISABLED                         (0x00000001U)

#define CSL_MCSPI_CH1CONF_EPOL_MASK                             (0x00000040U)
#define CSL_MCSPI_CH1CONF_EPOL_SHIFT                            (6U)
#define CSL_MCSPI_CH1CONF_EPOL_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH1CONF_EPOL_ACTIVELOW                        (0x00000001U)
#define CSL_MCSPI_CH1CONF_EPOL_ACTIVEHIGH                       (0x00000000U)

#define CSL_MCSPI_CH1CONF_DMAW_MASK                             (0x00004000U)
#define CSL_MCSPI_CH1CONF_DMAW_SHIFT                            (14U)
#define CSL_MCSPI_CH1CONF_DMAW_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH1CONF_DMAW_ENABLED                          (0x00000001U)
#define CSL_MCSPI_CH1CONF_DMAW_DISABLED                         (0x00000000U)

#define CSL_MCSPI_CH1CONF_TRM_MASK                              (0x00003000U)
#define CSL_MCSPI_CH1CONF_TRM_SHIFT                             (12U)
#define CSL_MCSPI_CH1CONF_TRM_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH1CONF_TRM_TRANSONLY                         (0x00000002U)
#define CSL_MCSPI_CH1CONF_TRM_RSVD                              (0x00000003U)
#define CSL_MCSPI_CH1CONF_TRM_TRANSRECEI                        (0x00000000U)
#define CSL_MCSPI_CH1CONF_TRM_RECEIVONLY                        (0x00000001U)

#define CSL_MCSPI_CH1CONF_POL_MASK                              (0x00000002U)
#define CSL_MCSPI_CH1CONF_POL_SHIFT                             (1U)
#define CSL_MCSPI_CH1CONF_POL_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH1CONF_POL_ACTIVELOW                         (0x00000001U)
#define CSL_MCSPI_CH1CONF_POL_ACTIVEHIGH                        (0x00000000U)

#define CSL_MCSPI_CH1CONF_SBE_MASK                              (0x00800000U)
#define CSL_MCSPI_CH1CONF_SBE_SHIFT                             (23U)
#define CSL_MCSPI_CH1CONF_SBE_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH1CONF_SBE_DISABLED                          (0x00000000U)
#define CSL_MCSPI_CH1CONF_SBE_ENABLED                           (0x00000001U)

#define CSL_MCSPI_CH1CONF_SBPOL_MASK                            (0x01000000U)
#define CSL_MCSPI_CH1CONF_SBPOL_SHIFT                           (24U)
#define CSL_MCSPI_CH1CONF_SBPOL_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH1CONF_SBPOL_LOWLEVEL                        (0x00000000U)
#define CSL_MCSPI_CH1CONF_SBPOL_HIGHLEVEL                       (0x00000001U)

#define CSL_MCSPI_CH1CONF_TCS1_MASK                             (0x06000000U)
#define CSL_MCSPI_CH1CONF_TCS1_SHIFT                            (25U)
#define CSL_MCSPI_CH1CONF_TCS1_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH1CONF_TCS1_THREECYCLEDLY                    (0x00000003U)
#define CSL_MCSPI_CH1CONF_TCS1_TWOCYCLEDLY                      (0x00000002U)
#define CSL_MCSPI_CH1CONF_TCS1_ONECYCLEDLY                      (0x00000001U)
#define CSL_MCSPI_CH1CONF_TCS1_ZEROCYCLEDLY                     (0x00000000U)

#define CSL_MCSPI_CH1CONF_FFER_MASK                             (0x10000000U)
#define CSL_MCSPI_CH1CONF_FFER_SHIFT                            (28U)
#define CSL_MCSPI_CH1CONF_FFER_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH1CONF_FFER_FFENABLED                        (0x00000001U)
#define CSL_MCSPI_CH1CONF_FFER_FFDISABLED                       (0x00000000U)

#define CSL_MCSPI_CH1CONF_FFEW_MASK                             (0x08000000U)
#define CSL_MCSPI_CH1CONF_FFEW_SHIFT                            (27U)
#define CSL_MCSPI_CH1CONF_FFEW_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH1CONF_FFEW_FFENABLED                        (0x00000001U)
#define CSL_MCSPI_CH1CONF_FFEW_FFDISABLED                       (0x00000000U)

#define CSL_MCSPI_CH1CONF_CLKG_MASK                             (0x20000000U)
#define CSL_MCSPI_CH1CONF_CLKG_SHIFT                            (29U)
#define CSL_MCSPI_CH1CONF_CLKG_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH1CONF_CLKG_ONECYCLE                         (0x00000001U)
#define CSL_MCSPI_CH1CONF_CLKG_POWERTWO                         (0x00000000U)

#define CSL_MCSPI_CH1CONF_RESETVAL                              (0x00060000U)

/* CH1STAT */

#define CSL_MCSPI_CH1STAT_RXS_MASK                              (0x00000001U)
#define CSL_MCSPI_CH1STAT_RXS_SHIFT                             (0U)
#define CSL_MCSPI_CH1STAT_RXS_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH1STAT_RXS_EMPTY                             (0x00000000U)
#define CSL_MCSPI_CH1STAT_RXS_FULL                              (0x00000001U)

#define CSL_MCSPI_CH1STAT_EOT_MASK                              (0x00000004U)
#define CSL_MCSPI_CH1STAT_EOT_SHIFT                             (2U)
#define CSL_MCSPI_CH1STAT_EOT_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH1STAT_EOT_COMPLETED                         (0x00000001U)
#define CSL_MCSPI_CH1STAT_EOT_INPROGRESS                        (0x00000000U)

#define CSL_MCSPI_CH1STAT_TXS_MASK                              (0x00000002U)
#define CSL_MCSPI_CH1STAT_TXS_SHIFT                             (1U)
#define CSL_MCSPI_CH1STAT_TXS_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH1STAT_TXS_EMPTY                             (0x00000001U)
#define CSL_MCSPI_CH1STAT_TXS_FULL                              (0x00000000U)

#define CSL_MCSPI_CH1STAT_RXFFE_MASK                            (0x00000020U)
#define CSL_MCSPI_CH1STAT_RXFFE_SHIFT                           (5U)
#define CSL_MCSPI_CH1STAT_RXFFE_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH1STAT_RXFFE_EMPTY                           (0x00000001U)
#define CSL_MCSPI_CH1STAT_RXFFE_NOTEMPTY                        (0x00000000U)

#define CSL_MCSPI_CH1STAT_RXFFF_MASK                            (0x00000040U)
#define CSL_MCSPI_CH1STAT_RXFFF_SHIFT                           (6U)
#define CSL_MCSPI_CH1STAT_RXFFF_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH1STAT_RXFFF_FULL                            (0x00000001U)
#define CSL_MCSPI_CH1STAT_RXFFF_NOTFULL                         (0x00000000U)

#define CSL_MCSPI_CH1STAT_TXFFE_MASK                            (0x00000008U)
#define CSL_MCSPI_CH1STAT_TXFFE_SHIFT                           (3U)
#define CSL_MCSPI_CH1STAT_TXFFE_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH1STAT_TXFFE_EMPTY                           (0x00000001U)
#define CSL_MCSPI_CH1STAT_TXFFE_NOTEMPTY                        (0x00000000U)

#define CSL_MCSPI_CH1STAT_TXFFF_MASK                            (0x00000010U)
#define CSL_MCSPI_CH1STAT_TXFFF_SHIFT                           (4U)
#define CSL_MCSPI_CH1STAT_TXFFF_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH1STAT_TXFFF_FULL                            (0x00000001U)
#define CSL_MCSPI_CH1STAT_TXFFF_NOTFULL                         (0x00000000U)

#define CSL_MCSPI_CH1STAT_RESETVAL                              (0x00000000U)

/* CH1CTRL */

#define CSL_MCSPI_CH1CTRL_EN_MASK                               (0x00000001U)
#define CSL_MCSPI_CH1CTRL_EN_SHIFT                              (0U)
#define CSL_MCSPI_CH1CTRL_EN_RESETVAL                           (0x00000000U)
#define CSL_MCSPI_CH1CTRL_EN_ACT                                (0x00000001U)
#define CSL_MCSPI_CH1CTRL_EN_NACT                               (0x00000000U)

#define CSL_MCSPI_CH1CTRL_EXTCLK_MASK                           (0x0000FF00U)
#define CSL_MCSPI_CH1CTRL_EXTCLK_SHIFT                          (8U)
#define CSL_MCSPI_CH1CTRL_EXTCLK_RESETVAL                       (0x00000000U)
#define CSL_MCSPI_CH1CTRL_EXTCLK_MAX                            (0x000000ffU)

#define CSL_MCSPI_CH1CTRL_RESETVAL                              (0x00000000U)

/* TX1 */

#define CSL_MCSPI_TX1_TDATA_MASK                                (0xFFFFFFFFU)
#define CSL_MCSPI_TX1_TDATA_SHIFT                               (0U)
#define CSL_MCSPI_TX1_TDATA_RESETVAL                            (0x00000000U)
#define CSL_MCSPI_TX1_TDATA_MAX                                 (0xffffffffU)

#define CSL_MCSPI_TX1_RESETVAL                                  (0x00000000U)

/* RX1 */

#define CSL_MCSPI_RX1_RDATA_MASK                                (0xFFFFFFFFU)
#define CSL_MCSPI_RX1_RDATA_SHIFT                               (0U)
#define CSL_MCSPI_RX1_RDATA_RESETVAL                            (0x00000000U)
#define CSL_MCSPI_RX1_RDATA_MAX                                 (0xffffffffU)

#define CSL_MCSPI_RX1_RESETVAL                                  (0x00000000U)

/* CH2CONF */

#define CSL_MCSPI_CH2CONF_CLKD_MASK                             (0x0000003CU)
#define CSL_MCSPI_CH2CONF_CLKD_SHIFT                            (2U)
#define CSL_MCSPI_CH2CONF_CLKD_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH2CONF_CLKD_MAX                              (0x0000000fU)

#define CSL_MCSPI_CH2CONF_PHA_MASK                              (0x00000001U)
#define CSL_MCSPI_CH2CONF_PHA_SHIFT                             (0U)
#define CSL_MCSPI_CH2CONF_PHA_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH2CONF_PHA_ODD                               (0x00000000U)
#define CSL_MCSPI_CH2CONF_PHA_EVEN                              (0x00000001U)

#define CSL_MCSPI_CH2CONF_TURBO_MASK                            (0x00080000U)
#define CSL_MCSPI_CH2CONF_TURBO_SHIFT                           (19U)
#define CSL_MCSPI_CH2CONF_TURBO_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH2CONF_TURBO_TURBO                           (0x00000001U)
#define CSL_MCSPI_CH2CONF_TURBO_OFF                             (0x00000000U)

#define CSL_MCSPI_CH2CONF_DPE0_MASK                             (0x00010000U)
#define CSL_MCSPI_CH2CONF_DPE0_SHIFT                            (16U)
#define CSL_MCSPI_CH2CONF_DPE0_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH2CONF_DPE0_DISABLED                         (0x00000001U)
#define CSL_MCSPI_CH2CONF_DPE0_ENABLED                          (0x00000000U)

#define CSL_MCSPI_CH2CONF_IS_MASK                               (0x00040000U)
#define CSL_MCSPI_CH2CONF_IS_SHIFT                              (18U)
#define CSL_MCSPI_CH2CONF_IS_RESETVAL                           (0x00000001U)
#define CSL_MCSPI_CH2CONF_IS_LINE0                              (0x00000000U)
#define CSL_MCSPI_CH2CONF_IS_LINE1                              (0x00000001U)

#define CSL_MCSPI_CH2CONF_DMAR_MASK                             (0x00008000U)
#define CSL_MCSPI_CH2CONF_DMAR_SHIFT                            (15U)
#define CSL_MCSPI_CH2CONF_DMAR_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH2CONF_DMAR_DISABLED                         (0x00000000U)
#define CSL_MCSPI_CH2CONF_DMAR_ENABLED                          (0x00000001U)

#define CSL_MCSPI_CH2CONF_FORCE_MASK                            (0x00100000U)
#define CSL_MCSPI_CH2CONF_FORCE_SHIFT                           (20U)
#define CSL_MCSPI_CH2CONF_FORCE_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH2CONF_FORCE_DEASSERT                        (0x00000000U)
#define CSL_MCSPI_CH2CONF_FORCE_ASSERT                          (0x00000001U)

#define CSL_MCSPI_CH2CONF_WL_MASK                               (0x00000F80U)
#define CSL_MCSPI_CH2CONF_WL_SHIFT                              (7U)
#define CSL_MCSPI_CH2CONF_WL_RESETVAL                           (0x00000000U)
#define CSL_MCSPI_CH2CONF_WL_MAX                                (0x0000001fU)

#define CSL_MCSPI_CH2CONF_DPE1_MASK                             (0x00020000U)
#define CSL_MCSPI_CH2CONF_DPE1_SHIFT                            (17U)
#define CSL_MCSPI_CH2CONF_DPE1_RESETVAL                         (0x00000001U)
#define CSL_MCSPI_CH2CONF_DPE1_ENABLED                          (0x00000000U)
#define CSL_MCSPI_CH2CONF_DPE1_DISABLED                         (0x00000001U)

#define CSL_MCSPI_CH2CONF_EPOL_MASK                             (0x00000040U)
#define CSL_MCSPI_CH2CONF_EPOL_SHIFT                            (6U)
#define CSL_MCSPI_CH2CONF_EPOL_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH2CONF_EPOL_ACTIVELOW                        (0x00000001U)
#define CSL_MCSPI_CH2CONF_EPOL_ACTIVEHIGH                       (0x00000000U)

#define CSL_MCSPI_CH2CONF_DMAW_MASK                             (0x00004000U)
#define CSL_MCSPI_CH2CONF_DMAW_SHIFT                            (14U)
#define CSL_MCSPI_CH2CONF_DMAW_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH2CONF_DMAW_ENABLED                          (0x00000001U)
#define CSL_MCSPI_CH2CONF_DMAW_DISABLED                         (0x00000000U)

#define CSL_MCSPI_CH2CONF_TRM_MASK                              (0x00003000U)
#define CSL_MCSPI_CH2CONF_TRM_SHIFT                             (12U)
#define CSL_MCSPI_CH2CONF_TRM_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH2CONF_TRM_TRANSONLY                         (0x00000002U)
#define CSL_MCSPI_CH2CONF_TRM_RSVD                              (0x00000003U)
#define CSL_MCSPI_CH2CONF_TRM_TRANSRECEI                        (0x00000000U)
#define CSL_MCSPI_CH2CONF_TRM_RECEIVONLY                        (0x00000001U)

#define CSL_MCSPI_CH2CONF_POL_MASK                              (0x00000002U)
#define CSL_MCSPI_CH2CONF_POL_SHIFT                             (1U)
#define CSL_MCSPI_CH2CONF_POL_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH2CONF_POL_ACTIVELOW                         (0x00000001U)
#define CSL_MCSPI_CH2CONF_POL_ACTIVEHIGH                        (0x00000000U)

#define CSL_MCSPI_CH2CONF_SBE_MASK                              (0x00800000U)
#define CSL_MCSPI_CH2CONF_SBE_SHIFT                             (23U)
#define CSL_MCSPI_CH2CONF_SBE_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH2CONF_SBE_DISABLED                          (0x00000000U)
#define CSL_MCSPI_CH2CONF_SBE_ENABLED                           (0x00000001U)

#define CSL_MCSPI_CH2CONF_SBPOL_MASK                            (0x01000000U)
#define CSL_MCSPI_CH2CONF_SBPOL_SHIFT                           (24U)
#define CSL_MCSPI_CH2CONF_SBPOL_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH2CONF_SBPOL_LOWLEVEL                        (0x00000000U)
#define CSL_MCSPI_CH2CONF_SBPOL_HIGHLEVEL                       (0x00000001U)

#define CSL_MCSPI_CH2CONF_TCS2_MASK                             (0x06000000U)
#define CSL_MCSPI_CH2CONF_TCS2_SHIFT                            (25U)
#define CSL_MCSPI_CH2CONF_TCS2_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH2CONF_TCS2_THREECYCLEDLY                    (0x00000003U)
#define CSL_MCSPI_CH2CONF_TCS2_TWOCYCLEDLY                      (0x00000002U)
#define CSL_MCSPI_CH2CONF_TCS2_ONECYCLEDLY                      (0x00000001U)
#define CSL_MCSPI_CH2CONF_TCS2_ZEROCYCLEDLY                     (0x00000000U)

#define CSL_MCSPI_CH2CONF_FFER_MASK                             (0x10000000U)
#define CSL_MCSPI_CH2CONF_FFER_SHIFT                            (28U)
#define CSL_MCSPI_CH2CONF_FFER_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH2CONF_FFER_FFENABLED                        (0x00000001U)
#define CSL_MCSPI_CH2CONF_FFER_FFDISABLED                       (0x00000000U)

#define CSL_MCSPI_CH2CONF_FFEW_MASK                             (0x08000000U)
#define CSL_MCSPI_CH2CONF_FFEW_SHIFT                            (27U)
#define CSL_MCSPI_CH2CONF_FFEW_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH2CONF_FFEW_FFENABLED                        (0x00000001U)
#define CSL_MCSPI_CH2CONF_FFEW_FFDISABLED                       (0x00000000U)

#define CSL_MCSPI_CH2CONF_CLKG_MASK                             (0x20000000U)
#define CSL_MCSPI_CH2CONF_CLKG_SHIFT                            (29U)
#define CSL_MCSPI_CH2CONF_CLKG_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH2CONF_CLKG_ONECYCLE                         (0x00000001U)
#define CSL_MCSPI_CH2CONF_CLKG_POWERTWO                         (0x00000000U)

#define CSL_MCSPI_CH2CONF_RESETVAL                              (0x00060000U)

/* CH2STAT */

#define CSL_MCSPI_CH2STAT_RXS_MASK                              (0x00000001U)
#define CSL_MCSPI_CH2STAT_RXS_SHIFT                             (0U)
#define CSL_MCSPI_CH2STAT_RXS_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH2STAT_RXS_EMPTY                             (0x00000000U)
#define CSL_MCSPI_CH2STAT_RXS_FULL                              (0x00000001U)

#define CSL_MCSPI_CH2STAT_EOT_MASK                              (0x00000004U)
#define CSL_MCSPI_CH2STAT_EOT_SHIFT                             (2U)
#define CSL_MCSPI_CH2STAT_EOT_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH2STAT_EOT_COMPLETED                         (0x00000001U)
#define CSL_MCSPI_CH2STAT_EOT_INPROGRESS                        (0x00000000U)

#define CSL_MCSPI_CH2STAT_TXS_MASK                              (0x00000002U)
#define CSL_MCSPI_CH2STAT_TXS_SHIFT                             (1U)
#define CSL_MCSPI_CH2STAT_TXS_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH2STAT_TXS_EMPTY                             (0x00000001U)
#define CSL_MCSPI_CH2STAT_TXS_FULL                              (0x00000000U)

#define CSL_MCSPI_CH2STAT_RXFFE_MASK                            (0x00000020U)
#define CSL_MCSPI_CH2STAT_RXFFE_SHIFT                           (5U)
#define CSL_MCSPI_CH2STAT_RXFFE_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH2STAT_RXFFE_EMPTY                           (0x00000001U)
#define CSL_MCSPI_CH2STAT_RXFFE_NOTEMPTY                        (0x00000000U)

#define CSL_MCSPI_CH2STAT_RXFFF_MASK                            (0x00000040U)
#define CSL_MCSPI_CH2STAT_RXFFF_SHIFT                           (6U)
#define CSL_MCSPI_CH2STAT_RXFFF_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH2STAT_RXFFF_FULL                            (0x00000001U)
#define CSL_MCSPI_CH2STAT_RXFFF_NOTFULL                         (0x00000000U)

#define CSL_MCSPI_CH2STAT_TXFFE_MASK                            (0x00000008U)
#define CSL_MCSPI_CH2STAT_TXFFE_SHIFT                           (3U)
#define CSL_MCSPI_CH2STAT_TXFFE_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH2STAT_TXFFE_EMPTY                           (0x00000001U)
#define CSL_MCSPI_CH2STAT_TXFFE_NOTEMPTY                        (0x00000000U)

#define CSL_MCSPI_CH2STAT_TXFFF_MASK                            (0x00000010U)
#define CSL_MCSPI_CH2STAT_TXFFF_SHIFT                           (4U)
#define CSL_MCSPI_CH2STAT_TXFFF_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH2STAT_TXFFF_FULL                            (0x00000001U)
#define CSL_MCSPI_CH2STAT_TXFFF_NOTFULL                         (0x00000000U)

#define CSL_MCSPI_CH2STAT_RESETVAL                              (0x00000000U)

/* CH2CTRL */

#define CSL_MCSPI_CH2CTRL_EN_MASK                               (0x00000001U)
#define CSL_MCSPI_CH2CTRL_EN_SHIFT                              (0U)
#define CSL_MCSPI_CH2CTRL_EN_RESETVAL                           (0x00000000U)
#define CSL_MCSPI_CH2CTRL_EN_ACT                                (0x00000001U)
#define CSL_MCSPI_CH2CTRL_EN_NACT                               (0x00000000U)

#define CSL_MCSPI_CH2CTRL_EXTCLK_MASK                           (0x0000FF00U)
#define CSL_MCSPI_CH2CTRL_EXTCLK_SHIFT                          (8U)
#define CSL_MCSPI_CH2CTRL_EXTCLK_RESETVAL                       (0x00000000U)
#define CSL_MCSPI_CH2CTRL_EXTCLK_MAX                            (0x000000ffU)

#define CSL_MCSPI_CH2CTRL_RESETVAL                              (0x00000000U)

/* TX2 */

#define CSL_MCSPI_TX2_TDATA_MASK                                (0xFFFFFFFFU)
#define CSL_MCSPI_TX2_TDATA_SHIFT                               (0U)
#define CSL_MCSPI_TX2_TDATA_RESETVAL                            (0x00000000U)
#define CSL_MCSPI_TX2_TDATA_MAX                                 (0xffffffffU)

#define CSL_MCSPI_TX2_RESETVAL                                  (0x00000000U)

/* RX2 */

#define CSL_MCSPI_RX2_RDATA_MASK                                (0xFFFFFFFFU)
#define CSL_MCSPI_RX2_RDATA_SHIFT                               (0U)
#define CSL_MCSPI_RX2_RDATA_RESETVAL                            (0x00000000U)
#define CSL_MCSPI_RX2_RDATA_MAX                                 (0xffffffffU)

#define CSL_MCSPI_RX2_RESETVAL                                  (0x00000000U)

/* CH3CONF */

#define CSL_MCSPI_CH3CONF_CLKD_MASK                             (0x0000003CU)
#define CSL_MCSPI_CH3CONF_CLKD_SHIFT                            (2U)
#define CSL_MCSPI_CH3CONF_CLKD_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH3CONF_CLKD_MAX                              (0x0000000fU)

#define CSL_MCSPI_CH3CONF_PHA_MASK                              (0x00000001U)
#define CSL_MCSPI_CH3CONF_PHA_SHIFT                             (0U)
#define CSL_MCSPI_CH3CONF_PHA_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH3CONF_PHA_ODD                               (0x00000000U)
#define CSL_MCSPI_CH3CONF_PHA_EVEN                              (0x00000001U)

#define CSL_MCSPI_CH3CONF_TURBO_MASK                            (0x00080000U)
#define CSL_MCSPI_CH3CONF_TURBO_SHIFT                           (19U)
#define CSL_MCSPI_CH3CONF_TURBO_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH3CONF_TURBO_TURBO                           (0x00000001U)
#define CSL_MCSPI_CH3CONF_TURBO_OFF                             (0x00000000U)

#define CSL_MCSPI_CH3CONF_DPE0_MASK                             (0x00010000U)
#define CSL_MCSPI_CH3CONF_DPE0_SHIFT                            (16U)
#define CSL_MCSPI_CH3CONF_DPE0_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH3CONF_DPE0_DISABLED                         (0x00000001U)
#define CSL_MCSPI_CH3CONF_DPE0_ENABLED                          (0x00000000U)

#define CSL_MCSPI_CH3CONF_IS_MASK                               (0x00040000U)
#define CSL_MCSPI_CH3CONF_IS_SHIFT                              (18U)
#define CSL_MCSPI_CH3CONF_IS_RESETVAL                           (0x00000001U)
#define CSL_MCSPI_CH3CONF_IS_LINE0                              (0x00000000U)
#define CSL_MCSPI_CH3CONF_IS_LINE1                              (0x00000001U)

#define CSL_MCSPI_CH3CONF_DMAR_MASK                             (0x00008000U)
#define CSL_MCSPI_CH3CONF_DMAR_SHIFT                            (15U)
#define CSL_MCSPI_CH3CONF_DMAR_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH3CONF_DMAR_DISABLED                         (0x00000000U)
#define CSL_MCSPI_CH3CONF_DMAR_ENABLED                          (0x00000001U)

#define CSL_MCSPI_CH3CONF_FORCE_MASK                            (0x00100000U)
#define CSL_MCSPI_CH3CONF_FORCE_SHIFT                           (20U)
#define CSL_MCSPI_CH3CONF_FORCE_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH3CONF_FORCE_DEASSERT                        (0x00000000U)
#define CSL_MCSPI_CH3CONF_FORCE_ASSERT                          (0x00000001U)

#define CSL_MCSPI_CH3CONF_WL_MASK                               (0x00000F80U)
#define CSL_MCSPI_CH3CONF_WL_SHIFT                              (7U)
#define CSL_MCSPI_CH3CONF_WL_RESETVAL                           (0x00000000U)
#define CSL_MCSPI_CH3CONF_WL_MAX                                (0x0000001fU)

#define CSL_MCSPI_CH3CONF_DPE1_MASK                             (0x00020000U)
#define CSL_MCSPI_CH3CONF_DPE1_SHIFT                            (17U)
#define CSL_MCSPI_CH3CONF_DPE1_RESETVAL                         (0x00000001U)
#define CSL_MCSPI_CH3CONF_DPE1_ENABLED                          (0x00000000U)
#define CSL_MCSPI_CH3CONF_DPE1_DISABLED                         (0x00000001U)

#define CSL_MCSPI_CH3CONF_EPOL_MASK                             (0x00000040U)
#define CSL_MCSPI_CH3CONF_EPOL_SHIFT                            (6U)
#define CSL_MCSPI_CH3CONF_EPOL_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH3CONF_EPOL_ACTIVELOW                        (0x00000001U)
#define CSL_MCSPI_CH3CONF_EPOL_ACTIVEHIGH                       (0x00000000U)

#define CSL_MCSPI_CH3CONF_DMAW_MASK                             (0x00004000U)
#define CSL_MCSPI_CH3CONF_DMAW_SHIFT                            (14U)
#define CSL_MCSPI_CH3CONF_DMAW_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH3CONF_DMAW_ENABLED                          (0x00000001U)
#define CSL_MCSPI_CH3CONF_DMAW_DISABLED                         (0x00000000U)

#define CSL_MCSPI_CH3CONF_TRM_MASK                              (0x00003000U)
#define CSL_MCSPI_CH3CONF_TRM_SHIFT                             (12U)
#define CSL_MCSPI_CH3CONF_TRM_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH3CONF_TRM_TRANSONLY                         (0x00000002U)
#define CSL_MCSPI_CH3CONF_TRM_RSVD                              (0x00000003U)
#define CSL_MCSPI_CH3CONF_TRM_TRANSRECEI                        (0x00000000U)
#define CSL_MCSPI_CH3CONF_TRM_RECEIVONLY                        (0x00000001U)

#define CSL_MCSPI_CH3CONF_POL_MASK                              (0x00000002U)
#define CSL_MCSPI_CH3CONF_POL_SHIFT                             (1U)
#define CSL_MCSPI_CH3CONF_POL_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH3CONF_POL_ACTIVELOW                         (0x00000001U)
#define CSL_MCSPI_CH3CONF_POL_ACTIVEHIGH                        (0x00000000U)

#define CSL_MCSPI_CH3CONF_SBE_MASK                              (0x00800000U)
#define CSL_MCSPI_CH3CONF_SBE_SHIFT                             (23U)
#define CSL_MCSPI_CH3CONF_SBE_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH3CONF_SBE_DISABLED                          (0x00000000U)
#define CSL_MCSPI_CH3CONF_SBE_ENABLED                           (0x00000001U)

#define CSL_MCSPI_CH3CONF_SBPOL_MASK                            (0x01000000U)
#define CSL_MCSPI_CH3CONF_SBPOL_SHIFT                           (24U)
#define CSL_MCSPI_CH3CONF_SBPOL_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH3CONF_SBPOL_LOWLEVEL                        (0x00000000U)
#define CSL_MCSPI_CH3CONF_SBPOL_HIGHLEVEL                       (0x00000001U)

#define CSL_MCSPI_CH3CONF_TCS3_MASK                             (0x06000000U)
#define CSL_MCSPI_CH3CONF_TCS3_SHIFT                            (25U)
#define CSL_MCSPI_CH3CONF_TCS3_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH3CONF_TCS3_THREECYCLEDLY                    (0x00000003U)
#define CSL_MCSPI_CH3CONF_TCS3_TWOCYCLEDLY                      (0x00000002U)
#define CSL_MCSPI_CH3CONF_TCS3_ONECYCLEDLY                      (0x00000001U)
#define CSL_MCSPI_CH3CONF_TCS3_ZEROCYCLEDLY                     (0x00000000U)

#define CSL_MCSPI_CH3CONF_FFER_MASK                             (0x10000000U)
#define CSL_MCSPI_CH3CONF_FFER_SHIFT                            (28U)
#define CSL_MCSPI_CH3CONF_FFER_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH3CONF_FFER_FFENABLED                        (0x00000001U)
#define CSL_MCSPI_CH3CONF_FFER_FFDISABLED                       (0x00000000U)

#define CSL_MCSPI_CH3CONF_FFEW_MASK                             (0x08000000U)
#define CSL_MCSPI_CH3CONF_FFEW_SHIFT                            (27U)
#define CSL_MCSPI_CH3CONF_FFEW_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH3CONF_FFEW_FFENABLED                        (0x00000001U)
#define CSL_MCSPI_CH3CONF_FFEW_FFDISABLED                       (0x00000000U)

#define CSL_MCSPI_CH3CONF_CLKG_MASK                             (0x20000000U)
#define CSL_MCSPI_CH3CONF_CLKG_SHIFT                            (29U)
#define CSL_MCSPI_CH3CONF_CLKG_RESETVAL                         (0x00000000U)
#define CSL_MCSPI_CH3CONF_CLKG_ONECYCLE                         (0x00000001U)
#define CSL_MCSPI_CH3CONF_CLKG_POWERTWO                         (0x00000000U)

#define CSL_MCSPI_CH3CONF_RESETVAL                              (0x00060000U)

/* CH3STAT */

#define CSL_MCSPI_CH3STAT_RXS_MASK                              (0x00000001U)
#define CSL_MCSPI_CH3STAT_RXS_SHIFT                             (0U)
#define CSL_MCSPI_CH3STAT_RXS_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH3STAT_RXS_EMPTY                             (0x00000000U)
#define CSL_MCSPI_CH3STAT_RXS_FULL                              (0x00000001U)

#define CSL_MCSPI_CH3STAT_EOT_MASK                              (0x00000004U)
#define CSL_MCSPI_CH3STAT_EOT_SHIFT                             (2U)
#define CSL_MCSPI_CH3STAT_EOT_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH3STAT_EOT_COMPLETED                         (0x00000001U)
#define CSL_MCSPI_CH3STAT_EOT_INPROGRESS                        (0x00000000U)

#define CSL_MCSPI_CH3STAT_TXS_MASK                              (0x00000002U)
#define CSL_MCSPI_CH3STAT_TXS_SHIFT                             (1U)
#define CSL_MCSPI_CH3STAT_TXS_RESETVAL                          (0x00000000U)
#define CSL_MCSPI_CH3STAT_TXS_EMPTY                             (0x00000001U)
#define CSL_MCSPI_CH3STAT_TXS_FULL                              (0x00000000U)

#define CSL_MCSPI_CH3STAT_RXFFE_MASK                            (0x00000020U)
#define CSL_MCSPI_CH3STAT_RXFFE_SHIFT                           (5U)
#define CSL_MCSPI_CH3STAT_RXFFE_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH3STAT_RXFFE_EMPTY                           (0x00000001U)
#define CSL_MCSPI_CH3STAT_RXFFE_NOTEMPTY                        (0x00000000U)

#define CSL_MCSPI_CH3STAT_RXFFF_MASK                            (0x00000040U)
#define CSL_MCSPI_CH3STAT_RXFFF_SHIFT                           (6U)
#define CSL_MCSPI_CH3STAT_RXFFF_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH3STAT_RXFFF_FULL                            (0x00000001U)
#define CSL_MCSPI_CH3STAT_RXFFF_NOTFULL                         (0x00000000U)

#define CSL_MCSPI_CH3STAT_TXFFE_MASK                            (0x00000008U)
#define CSL_MCSPI_CH3STAT_TXFFE_SHIFT                           (3U)
#define CSL_MCSPI_CH3STAT_TXFFE_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH3STAT_TXFFE_EMPTY                           (0x00000001U)
#define CSL_MCSPI_CH3STAT_TXFFE_NOTEMPTY                        (0x00000000U)

#define CSL_MCSPI_CH3STAT_TXFFF_MASK                            (0x00000010U)
#define CSL_MCSPI_CH3STAT_TXFFF_SHIFT                           (4U)
#define CSL_MCSPI_CH3STAT_TXFFF_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_CH3STAT_TXFFF_FULL                            (0x00000001U)
#define CSL_MCSPI_CH3STAT_TXFFF_NOTFULL                         (0x00000000U)

#define CSL_MCSPI_CH3STAT_RESETVAL                              (0x00000000U)

/* CH3CTRL */

#define CSL_MCSPI_CH3CTRL_EN_MASK                               (0x00000001U)
#define CSL_MCSPI_CH3CTRL_EN_SHIFT                              (0U)
#define CSL_MCSPI_CH3CTRL_EN_RESETVAL                           (0x00000000U)
#define CSL_MCSPI_CH3CTRL_EN_ACT                                (0x00000001U)
#define CSL_MCSPI_CH3CTRL_EN_NACT                               (0x00000000U)

#define CSL_MCSPI_CH3CTRL_EXTCLK_MASK                           (0x0000FF00U)
#define CSL_MCSPI_CH3CTRL_EXTCLK_SHIFT                          (8U)
#define CSL_MCSPI_CH3CTRL_EXTCLK_RESETVAL                       (0x00000000U)
#define CSL_MCSPI_CH3CTRL_EXTCLK_MAX                            (0x000000ffU)

#define CSL_MCSPI_CH3CTRL_RESETVAL                              (0x00000000U)

/* TX3 */

#define CSL_MCSPI_TX3_TDATA_MASK                                (0xFFFFFFFFU)
#define CSL_MCSPI_TX3_TDATA_SHIFT                               (0U)
#define CSL_MCSPI_TX3_TDATA_RESETVAL                            (0x00000000U)
#define CSL_MCSPI_TX3_TDATA_MAX                                 (0xffffffffU)

#define CSL_MCSPI_TX3_RESETVAL                                  (0x00000000U)

/* RX3 */

#define CSL_MCSPI_RX3_RDATA_MASK                                (0xFFFFFFFFU)
#define CSL_MCSPI_RX3_RDATA_SHIFT                               (0U)
#define CSL_MCSPI_RX3_RDATA_RESETVAL                            (0x00000000U)
#define CSL_MCSPI_RX3_RDATA_MAX                                 (0xffffffffU)

#define CSL_MCSPI_RX3_RESETVAL                                  (0x00000000U)

/* XFERLEVEL */

#define CSL_MCSPI_XFERLEVEL_AEL_MASK                            (0x000000FFU)
#define CSL_MCSPI_XFERLEVEL_AEL_SHIFT                           (0U)
#define CSL_MCSPI_XFERLEVEL_AEL_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_XFERLEVEL_AEL_MAX                             (0x000000ffU)

#define CSL_MCSPI_XFERLEVEL_WCNT_MASK                           (0xFFFF0000U)
#define CSL_MCSPI_XFERLEVEL_WCNT_SHIFT                          (16U)
#define CSL_MCSPI_XFERLEVEL_WCNT_RESETVAL                       (0x00000000U)
#define CSL_MCSPI_XFERLEVEL_WCNT_MAX                            (0x0000ffffU)

#define CSL_MCSPI_XFERLEVEL_AFL_MASK                            (0x0000FF00U)
#define CSL_MCSPI_XFERLEVEL_AFL_SHIFT                           (8U)
#define CSL_MCSPI_XFERLEVEL_AFL_RESETVAL                        (0x00000000U)
#define CSL_MCSPI_XFERLEVEL_AFL_MAX                             (0x000000ffU)

#define CSL_MCSPI_XFERLEVEL_RESETVAL                            (0x00000000U)

/* DAFTX */

#define CSL_MCSPI_DAFTX_DAFTDATA_MASK                           (0xFFFFFFFFU)
#define CSL_MCSPI_DAFTX_DAFTDATA_SHIFT                          (0U)
#define CSL_MCSPI_DAFTX_DAFTDATA_RESETVAL                       (0x00000000U)
#define CSL_MCSPI_DAFTX_DAFTDATA_MAX                            (0xffffffffU)

#define CSL_MCSPI_DAFTX_RESETVAL                                (0x00000000U)

/* DAFRX */

#define CSL_MCSPI_DAFRX_DAFRDATA_MASK                           (0xFFFFFFFFU)
#define CSL_MCSPI_DAFRX_DAFRDATA_SHIFT                          (0U)
#define CSL_MCSPI_DAFRX_DAFRDATA_RESETVAL                       (0x00000000U)
#define CSL_MCSPI_DAFRX_DAFRDATA_MAX                            (0xffffffffU)

#define CSL_MCSPI_DAFRX_RESETVAL                                (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
