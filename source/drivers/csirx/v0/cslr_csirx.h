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
 *  Name        : cslr_csi2.h
*/
#ifndef CSLR_CSI2_H_
#define CSLR_CSI2_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <drivers/hw_include/cslr.h>

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {

    volatile uint32_t CTX_CTRL1;
    volatile uint32_t CTX_CTRL2;
    volatile uint32_t CTX_DAT_OFST;
    volatile uint32_t CTX_DAT_PING_ADDR;
    volatile uint32_t CTX_DAT_PONG_ADDR;
    volatile uint32_t CTX_IRQENABLE;
    volatile uint32_t CTX_IRQSTATUS;
    volatile uint32_t CTX_CTRL3;

} CSL_csirxContextRegs;

typedef struct {

    volatile uint32_t CTX_TRANSCODEH;
    volatile uint32_t CTX_TRANSCODEV;

} CSL_csirxContextTranscodeRegs;

typedef struct {
    volatile uint32_t REVISION;
    volatile uint8_t  Resv_16[12];
    volatile uint32_t SYSCONFIG;
    volatile uint32_t SYSSTATUS;
    volatile uint32_t IRQSTATUS;
    volatile uint32_t IRQENABLE;
    volatile uint8_t  Resv_64[32];
    volatile uint32_t CTRL;
    volatile uint32_t DBG_H;
    volatile uint32_t GNQ;
    volatile uint32_t COMPLEXIO_CFG2;
    volatile uint32_t COMPLEXIO_CFG1;
    volatile uint32_t COMPLEXIO1_IRQSTATUS;
    volatile uint32_t COMPLEXIO2_IRQSTATUS;
    volatile uint32_t SHORT_PACKET;
    volatile uint32_t COMPLEXIO1_IRQENABLE;
    volatile uint32_t COMPLEXIO2_IRQENABLE;
    volatile uint32_t DBG_P;
    volatile uint32_t TIMING;
    CSL_csirxContextRegs ctx[8];
    volatile uint32_t PHY_CFG_REG0;
    volatile uint32_t PHY_CFG_REG1;
    volatile uint32_t PHY_CFG_REG2;
    volatile uint32_t PHY_CFG_REG3;
    volatile uint32_t PHY_CFG_REG4;
    volatile uint32_t PHY_CFG_REG5;
    volatile uint32_t PHY_CFG_REG6;
    volatile uint8_t  Resv_448[52];
    CSL_csirxContextTranscodeRegs ctxTranscode[8];
} CSL_csirxRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_CSI2_REVISION                                            (0x00000000U)
#define CSL_CSI2_SYSCONFIG                                           (0x00000010U)
#define CSL_CSI2_SYSSTATUS                                           (0x00000014U)
#define CSL_CSI2_IRQSTATUS                                           (0x00000018U)
#define CSL_CSI2_IRQENABLE                                           (0x0000001CU)
#define CSL_CSI2_CTRL                                                (0x00000040U)
#define CSL_CSI2_DBG_H                                               (0x00000044U)
#define CSL_CSI2_GNQ                                                 (0x00000048U)
#define CSL_CSI2_COMPLEXIO_CFG2                                      (0x0000004CU)
#define CSL_CSI2_COMPLEXIO_CFG1                                      (0x00000050U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS                                (0x00000054U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS                                (0x00000058U)
#define CSL_CSI2_SHORT_PACKET                                        (0x0000005CU)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE                                (0x00000060U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE                                (0x00000064U)
#define CSL_CSI2_DBG_P                                               (0x00000068U)
#define CSL_CSI2_TIMING                                              (0x0000006CU)
#define CSL_CSI2_CTX0_CTRL1                                          (0x00000070U)
#define CSL_CSI2_CTX0_CTRL2                                          (0x00000074U)
#define CSL_CSI2_CTX0_DAT_OFST                                       (0x00000078U)
#define CSL_CSI2_CTX0_DAT_PING_ADDR                                  (0x0000007CU)
#define CSL_CSI2_CTX0_DAT_PONG_ADDR                                  (0x00000080U)
#define CSL_CSI2_CTX0_IRQENABLE                                      (0x00000084U)
#define CSL_CSI2_CTX0_IRQSTATUS                                      (0x00000088U)
#define CSL_CSI2_CTX0_CTRL3                                          (0x0000008CU)
#define CSL_CSI2_CTX1_CTRL1                                          (0x00000090U)
#define CSL_CSI2_CTX1_CTRL2                                          (0x00000094U)
#define CSL_CSI2_CTX1_DAT_OFST                                       (0x00000098U)
#define CSL_CSI2_CTX1_DAT_PING_ADDR                                  (0x0000009CU)
#define CSL_CSI2_CTX1_DAT_PONG_ADDR                                  (0x000000A0U)
#define CSL_CSI2_CTX1_IRQENABLE                                      (0x000000A4U)
#define CSL_CSI2_CTX1_IRQSTATUS                                      (0x000000A8U)
#define CSL_CSI2_CTX1_CTRL3                                          (0x000000ACU)
#define CSL_CSI2_CTX2_CTRL1                                          (0x000000B0U)
#define CSL_CSI2_CTX2_CTRL2                                          (0x000000B4U)
#define CSL_CSI2_CTX2_DAT_OFST                                       (0x000000B8U)
#define CSL_CSI2_CTX2_DAT_PING_ADDR                                  (0x000000BCU)
#define CSL_CSI2_CTX2_DAT_PONG_ADDR                                  (0x000000C0U)
#define CSL_CSI2_CTX2_IRQENABLE                                      (0x000000C4U)
#define CSL_CSI2_CTX2_IRQSTATUS                                      (0x000000C8U)
#define CSL_CSI2_CTX2_CTRL3                                          (0x000000CCU)
#define CSL_CSI2_CTX3_CTRL1                                          (0x000000D0U)
#define CSL_CSI2_CTX3_CTRL2                                          (0x000000D4U)
#define CSL_CSI2_CTX3_DAT_OFST                                       (0x000000D8U)
#define CSL_CSI2_CTX3_DAT_PING_ADDR                                  (0x000000DCU)
#define CSL_CSI2_CTX3_DAT_PONG_ADDR                                  (0x000000E0U)
#define CSL_CSI2_CTX3_IRQENABLE                                      (0x000000E4U)
#define CSL_CSI2_CTX3_IRQSTATUS                                      (0x000000E8U)
#define CSL_CSI2_CTX3_CTRL3                                          (0x000000ECU)
#define CSL_CSI2_CTX4_CTRL1                                          (0x000000F0U)
#define CSL_CSI2_CTX4_CTRL2                                          (0x000000F4U)
#define CSL_CSI2_CTX4_DAT_OFST                                       (0x000000F8U)
#define CSL_CSI2_CTX4_DAT_PING_ADDR                                  (0x000000FCU)
#define CSL_CSI2_CTX4_DAT_PONG_ADDR                                  (0x00000100U)
#define CSL_CSI2_CTX4_IRQENABLE                                      (0x00000104U)
#define CSL_CSI2_CTX4_IRQSTATUS                                      (0x00000108U)
#define CSL_CSI2_CTX4_CTRL3                                          (0x0000010CU)
#define CSL_CSI2_CTX5_CTRL1                                          (0x00000110U)
#define CSL_CSI2_CTX5_CTRL2                                          (0x00000114U)
#define CSL_CSI2_CTX5_DAT_OFST                                       (0x00000118U)
#define CSL_CSI2_CTX5_DAT_PING_ADDR                                  (0x0000011CU)
#define CSL_CSI2_CTX5_DAT_PONG_ADDR                                  (0x00000120U)
#define CSL_CSI2_CTX5_IRQENABLE                                      (0x00000124U)
#define CSL_CSI2_CTX5_IRQSTATUS                                      (0x00000128U)
#define CSL_CSI2_CTX5_CTRL3                                          (0x0000012CU)
#define CSL_CSI2_CTX6_CTRL1                                          (0x00000130U)
#define CSL_CSI2_CTX6_CTRL2                                          (0x00000134U)
#define CSL_CSI2_CTX6_DAT_OFST                                       (0x00000138U)
#define CSL_CSI2_CTX6_DAT_PING_ADDR                                  (0x0000013CU)
#define CSL_CSI2_CTX6_DAT_PONG_ADDR                                  (0x00000140U)
#define CSL_CSI2_CTX6_IRQENABLE                                      (0x00000144U)
#define CSL_CSI2_CTX6_IRQSTATUS                                      (0x00000148U)
#define CSL_CSI2_CTX6_CTRL3                                          (0x0000014CU)
#define CSL_CSI2_CTX7_CTRL1                                          (0x00000150U)
#define CSL_CSI2_CTX7_CTRL2                                          (0x00000154U)
#define CSL_CSI2_CTX7_DAT_OFST                                       (0x00000158U)
#define CSL_CSI2_CTX7_DAT_PING_ADDR                                  (0x0000015CU)
#define CSL_CSI2_CTX7_DAT_PONG_ADDR                                  (0x00000160U)
#define CSL_CSI2_CTX7_IRQENABLE                                      (0x00000164U)
#define CSL_CSI2_CTX7_IRQSTATUS                                      (0x00000168U)
#define CSL_CSI2_CTX7_CTRL3                                          (0x0000016CU)
#define CSL_CSI2_PHY_CFG_REG0                                        (0x00000170U)
#define CSL_CSI2_PHY_CFG_REG1                                        (0x00000174U)
#define CSL_CSI2_PHY_CFG_REG2                                        (0x00000178U)
#define CSL_CSI2_PHY_CFG_REG3                                        (0x0000017CU)
#define CSL_CSI2_PHY_CFG_REG4                                        (0x00000180U)
#define CSL_CSI2_PHY_CFG_REG5                                        (0x00000184U)
#define CSL_CSI2_PHY_CFG_REG6                                        (0x00000188U)
#define CSL_CSI2_CTX0_TRANSCODEH                                     (0x000001C0U)
#define CSL_CSI2_CTX0_TRANSCODEV                                     (0x000001C4U)
#define CSL_CSI2_CTX1_TRANSCODEH                                     (0x000001C8U)
#define CSL_CSI2_CTX1_TRANSCODEV                                     (0x000001CCU)
#define CSL_CSI2_CTX2_TRANSCODEH                                     (0x000001D0U)
#define CSL_CSI2_CTX2_TRANSCODEV                                     (0x000001D4U)
#define CSL_CSI2_CTX3_TRANSCODEH                                     (0x000001D8U)
#define CSL_CSI2_CTX3_TRANSCODEV                                     (0x000001DCU)
#define CSL_CSI2_CTX4_TRANSCODEH                                     (0x000001E0U)
#define CSL_CSI2_CTX4_TRANSCODEV                                     (0x000001E4U)
#define CSL_CSI2_CTX5_TRANSCODEH                                     (0x000001E8U)
#define CSL_CSI2_CTX5_TRANSCODEV                                     (0x000001ECU)
#define CSL_CSI2_CTX6_TRANSCODEH                                     (0x000001F0U)
#define CSL_CSI2_CTX6_TRANSCODEV                                     (0x000001F4U)
#define CSL_CSI2_CTX7_TRANSCODEH                                     (0x000001F8U)
#define CSL_CSI2_CTX7_TRANSCODEV                                     (0x000001FCU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* CSI2_REVISION */

#define CSL_CSI2_REVISION_REV_MASK                                   (0x000000FFU)
#define CSL_CSI2_REVISION_REV_SHIFT                                  (0x00000000U)
#define CSL_CSI2_REVISION_REV_RESETVAL                               (0x00000030U)
#define CSL_CSI2_REVISION_REV_MAX                                    (0x000000FFU)

#define CSL_CSI2_REVISION_RES1_MASK                                  (0xFFFFFF00U)
#define CSL_CSI2_REVISION_RES1_SHIFT                                 (0x00000008U)
#define CSL_CSI2_REVISION_RES1_RESETVAL                              (0x00000000U)
#define CSL_CSI2_REVISION_RES1_MAX                                   (0x00FFFFFFU)

#define CSL_CSI2_REVISION_RESETVAL                                   (0x00000030U)

/* CSI2_SYSCONFIG */

#define CSL_CSI2_SYSCONFIG_AUTO_IDLE_MASK                            (0x00000001U)
#define CSL_CSI2_SYSCONFIG_AUTO_IDLE_SHIFT                           (0x00000000U)
#define CSL_CSI2_SYSCONFIG_AUTO_IDLE_RESETVAL                        (0x00000001U)
#define CSL_CSI2_SYSCONFIG_AUTO_IDLE_MAX                             (0x00000001U)

#define CSL_CSI2_SYSCONFIG_SOFT_RESET_MASK                           (0x00000002U)
#define CSL_CSI2_SYSCONFIG_SOFT_RESET_SHIFT                          (0x00000001U)
#define CSL_CSI2_SYSCONFIG_SOFT_RESET_RESETVAL                       (0x00000000U)
#define CSL_CSI2_SYSCONFIG_SOFT_RESET_MAX                            (0x00000001U)

#define CSL_CSI2_SYSCONFIG_RES3_MASK                                 (0x00000FFCU)
#define CSL_CSI2_SYSCONFIG_RES3_SHIFT                                (0x00000002U)
#define CSL_CSI2_SYSCONFIG_RES3_RESETVAL                             (0x00000000U)
#define CSL_CSI2_SYSCONFIG_RES3_MAX                                  (0x000003FFU)

#define CSL_CSI2_SYSCONFIG_MSTANDBY_MODE_MASK                        (0x00003000U)
#define CSL_CSI2_SYSCONFIG_MSTANDBY_MODE_SHIFT                       (0x0000000CU)
#define CSL_CSI2_SYSCONFIG_MSTANDBY_MODE_RESETVAL                    (0x00000000U)
#define CSL_CSI2_SYSCONFIG_MSTANDBY_MODE_MAX                         (0x00000003U)

#define CSL_CSI2_SYSCONFIG_RES2_MASK                                 (0xFFFFC000U)
#define CSL_CSI2_SYSCONFIG_RES2_SHIFT                                (0x0000000EU)
#define CSL_CSI2_SYSCONFIG_RES2_RESETVAL                             (0x00000000U)
#define CSL_CSI2_SYSCONFIG_RES2_MAX                                  (0x0003FFFFU)

#define CSL_CSI2_SYSCONFIG_RESETVAL                                  (0x00000001U)

/* CSI2_SYSSTATUS */

#define CSL_CSI2_SYSSTATUS_RESET_DONE_MASK                           (0x00000001U)
#define CSL_CSI2_SYSSTATUS_RESET_DONE_SHIFT                          (0x00000000U)
#define CSL_CSI2_SYSSTATUS_RESET_DONE_RESETVAL                       (0x00000001U)
#define CSL_CSI2_SYSSTATUS_RESET_DONE_MAX                            (0x00000001U)

#define CSL_CSI2_SYSSTATUS_RES4_MASK                                 (0xFFFFFFFEU)
#define CSL_CSI2_SYSSTATUS_RES4_SHIFT                                (0x00000001U)
#define CSL_CSI2_SYSSTATUS_RES4_RESETVAL                             (0x00000000U)
#define CSL_CSI2_SYSSTATUS_RES4_MAX                                  (0x7FFFFFFFU)

#define CSL_CSI2_SYSSTATUS_RESETVAL                                  (0x00000001U)

/* CSI2_IRQSTATUS */

#define CSL_CSI2_IRQSTATUS_CONTEXT0_MASK                             (0x00000001U)
#define CSL_CSI2_IRQSTATUS_CONTEXT0_SHIFT                            (0x00000000U)
#define CSL_CSI2_IRQSTATUS_CONTEXT0_RESETVAL                         (0x00000000U)
#define CSL_CSI2_IRQSTATUS_CONTEXT0_MAX                              (0x00000001U)

#define CSL_CSI2_IRQSTATUS_CONTEXT1_MASK                             (0x00000002U)
#define CSL_CSI2_IRQSTATUS_CONTEXT1_SHIFT                            (0x00000001U)
#define CSL_CSI2_IRQSTATUS_CONTEXT1_RESETVAL                         (0x00000000U)
#define CSL_CSI2_IRQSTATUS_CONTEXT1_MAX                              (0x00000001U)

#define CSL_CSI2_IRQSTATUS_CONTEXT2_MASK                             (0x00000004U)
#define CSL_CSI2_IRQSTATUS_CONTEXT2_SHIFT                            (0x00000002U)
#define CSL_CSI2_IRQSTATUS_CONTEXT2_RESETVAL                         (0x00000000U)
#define CSL_CSI2_IRQSTATUS_CONTEXT2_MAX                              (0x00000001U)

#define CSL_CSI2_IRQSTATUS_CONTEXT3_MASK                             (0x00000008U)
#define CSL_CSI2_IRQSTATUS_CONTEXT3_SHIFT                            (0x00000003U)
#define CSL_CSI2_IRQSTATUS_CONTEXT3_RESETVAL                         (0x00000000U)
#define CSL_CSI2_IRQSTATUS_CONTEXT3_MAX                              (0x00000001U)

#define CSL_CSI2_IRQSTATUS_CONTEXT4_MASK                             (0x00000010U)
#define CSL_CSI2_IRQSTATUS_CONTEXT4_SHIFT                            (0x00000004U)
#define CSL_CSI2_IRQSTATUS_CONTEXT4_RESETVAL                         (0x00000000U)
#define CSL_CSI2_IRQSTATUS_CONTEXT4_MAX                              (0x00000001U)

#define CSL_CSI2_IRQSTATUS_CONTEXT5_MASK                             (0x00000020U)
#define CSL_CSI2_IRQSTATUS_CONTEXT5_SHIFT                            (0x00000005U)
#define CSL_CSI2_IRQSTATUS_CONTEXT5_RESETVAL                         (0x00000000U)
#define CSL_CSI2_IRQSTATUS_CONTEXT5_MAX                              (0x00000001U)

#define CSL_CSI2_IRQSTATUS_CONTEXT6_MASK                             (0x00000040U)
#define CSL_CSI2_IRQSTATUS_CONTEXT6_SHIFT                            (0x00000006U)
#define CSL_CSI2_IRQSTATUS_CONTEXT6_RESETVAL                         (0x00000000U)
#define CSL_CSI2_IRQSTATUS_CONTEXT6_MAX                              (0x00000001U)

#define CSL_CSI2_IRQSTATUS_CONTEXT7_MASK                             (0x00000080U)
#define CSL_CSI2_IRQSTATUS_CONTEXT7_SHIFT                            (0x00000007U)
#define CSL_CSI2_IRQSTATUS_CONTEXT7_RESETVAL                         (0x00000000U)
#define CSL_CSI2_IRQSTATUS_CONTEXT7_MAX                              (0x00000001U)

#define CSL_CSI2_IRQSTATUS_FIFO_OVF_IRQ_MASK                         (0x00000100U)
#define CSL_CSI2_IRQSTATUS_FIFO_OVF_IRQ_SHIFT                        (0x00000008U)
#define CSL_CSI2_IRQSTATUS_FIFO_OVF_IRQ_RESETVAL                     (0x00000000U)
#define CSL_CSI2_IRQSTATUS_FIFO_OVF_IRQ_MAX                          (0x00000001U)

#define CSL_CSI2_IRQSTATUS_COMPLEXIO1_ERR_IRQ_MASK                   (0x00000200U)
#define CSL_CSI2_IRQSTATUS_COMPLEXIO1_ERR_IRQ_SHIFT                  (0x00000009U)
#define CSL_CSI2_IRQSTATUS_COMPLEXIO1_ERR_IRQ_RESETVAL               (0x00000000U)
#define CSL_CSI2_IRQSTATUS_COMPLEXIO1_ERR_IRQ_MAX                    (0x00000001U)

#define CSL_CSI2_IRQSTATUS_COMPLEXIO2_ERR_IRQ_MASK                   (0x00000400U)
#define CSL_CSI2_IRQSTATUS_COMPLEXIO2_ERR_IRQ_SHIFT                  (0x0000000AU)
#define CSL_CSI2_IRQSTATUS_COMPLEXIO2_ERR_IRQ_RESETVAL               (0x00000000U)
#define CSL_CSI2_IRQSTATUS_COMPLEXIO2_ERR_IRQ_MAX                    (0x00000001U)

#define CSL_CSI2_IRQSTATUS_ECC_NO_CORRECTION_IRQ_MASK                (0x00000800U)
#define CSL_CSI2_IRQSTATUS_ECC_NO_CORRECTION_IRQ_SHIFT               (0x0000000BU)
#define CSL_CSI2_IRQSTATUS_ECC_NO_CORRECTION_IRQ_RESETVAL            (0x00000000U)
#define CSL_CSI2_IRQSTATUS_ECC_NO_CORRECTION_IRQ_MAX                 (0x00000001U)

#define CSL_CSI2_IRQSTATUS_ECC_CORRECTION_IRQ_MASK                   (0x00001000U)
#define CSL_CSI2_IRQSTATUS_ECC_CORRECTION_IRQ_SHIFT                  (0x0000000CU)
#define CSL_CSI2_IRQSTATUS_ECC_CORRECTION_IRQ_RESETVAL               (0x00000000U)
#define CSL_CSI2_IRQSTATUS_ECC_CORRECTION_IRQ_MAX                    (0x00000001U)

#define CSL_CSI2_IRQSTATUS_SHORT_PACKET_IRQ_MASK                     (0x00002000U)
#define CSL_CSI2_IRQSTATUS_SHORT_PACKET_IRQ_SHIFT                    (0x0000000DU)
#define CSL_CSI2_IRQSTATUS_SHORT_PACKET_IRQ_RESETVAL                 (0x00000000U)
#define CSL_CSI2_IRQSTATUS_SHORT_PACKET_IRQ_MAX                      (0x00000001U)

#define CSL_CSI2_IRQSTATUS_OCP_ERR_IRQ_MASK                          (0x00004000U)
#define CSL_CSI2_IRQSTATUS_OCP_ERR_IRQ_SHIFT                         (0x0000000EU)
#define CSL_CSI2_IRQSTATUS_OCP_ERR_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_IRQSTATUS_OCP_ERR_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_IRQSTATUS_RES5_MASK                                 (0xFFFF8000U)
#define CSL_CSI2_IRQSTATUS_RES5_SHIFT                                (0x0000000FU)
#define CSL_CSI2_IRQSTATUS_RES5_RESETVAL                             (0x00000000U)
#define CSL_CSI2_IRQSTATUS_RES5_MAX                                  (0x0001FFFFU)

#define CSL_CSI2_IRQSTATUS_RESETVAL                                  (0x00000000U)

/* CSI2_IRQENABLE */

#define CSL_CSI2_IRQENABLE_CONTEXT0_MASK                             (0x00000001U)
#define CSL_CSI2_IRQENABLE_CONTEXT0_SHIFT                            (0x00000000U)
#define CSL_CSI2_IRQENABLE_CONTEXT0_RESETVAL                         (0x00000000U)
#define CSL_CSI2_IRQENABLE_CONTEXT0_MAX                              (0x00000001U)

#define CSL_CSI2_IRQENABLE_CONTEXT1_MASK                             (0x00000002U)
#define CSL_CSI2_IRQENABLE_CONTEXT1_SHIFT                            (0x00000001U)
#define CSL_CSI2_IRQENABLE_CONTEXT1_RESETVAL                         (0x00000000U)
#define CSL_CSI2_IRQENABLE_CONTEXT1_MAX                              (0x00000001U)

#define CSL_CSI2_IRQENABLE_CONTEXT2_MASK                             (0x00000004U)
#define CSL_CSI2_IRQENABLE_CONTEXT2_SHIFT                            (0x00000002U)
#define CSL_CSI2_IRQENABLE_CONTEXT2_RESETVAL                         (0x00000000U)
#define CSL_CSI2_IRQENABLE_CONTEXT2_MAX                              (0x00000001U)

#define CSL_CSI2_IRQENABLE_CONTEXT3_MASK                             (0x00000008U)
#define CSL_CSI2_IRQENABLE_CONTEXT3_SHIFT                            (0x00000003U)
#define CSL_CSI2_IRQENABLE_CONTEXT3_RESETVAL                         (0x00000000U)
#define CSL_CSI2_IRQENABLE_CONTEXT3_MAX                              (0x00000001U)

#define CSL_CSI2_IRQENABLE_CONTEXT4_MASK                             (0x00000010U)
#define CSL_CSI2_IRQENABLE_CONTEXT4_SHIFT                            (0x00000004U)
#define CSL_CSI2_IRQENABLE_CONTEXT4_RESETVAL                         (0x00000000U)
#define CSL_CSI2_IRQENABLE_CONTEXT4_MAX                              (0x00000001U)

#define CSL_CSI2_IRQENABLE_CONTEXT5_MASK                             (0x00000020U)
#define CSL_CSI2_IRQENABLE_CONTEXT5_SHIFT                            (0x00000005U)
#define CSL_CSI2_IRQENABLE_CONTEXT5_RESETVAL                         (0x00000000U)
#define CSL_CSI2_IRQENABLE_CONTEXT5_MAX                              (0x00000001U)

#define CSL_CSI2_IRQENABLE_CONTEXT6_MASK                             (0x00000040U)
#define CSL_CSI2_IRQENABLE_CONTEXT6_SHIFT                            (0x00000006U)
#define CSL_CSI2_IRQENABLE_CONTEXT6_RESETVAL                         (0x00000000U)
#define CSL_CSI2_IRQENABLE_CONTEXT6_MAX                              (0x00000001U)

#define CSL_CSI2_IRQENABLE_CONTEXT7_MASK                             (0x00000080U)
#define CSL_CSI2_IRQENABLE_CONTEXT7_SHIFT                            (0x00000007U)
#define CSL_CSI2_IRQENABLE_CONTEXT7_RESETVAL                         (0x00000000U)
#define CSL_CSI2_IRQENABLE_CONTEXT7_MAX                              (0x00000001U)

#define CSL_CSI2_IRQENABLE_FIFO_OVF_IRQ_MASK                         (0x00000100U)
#define CSL_CSI2_IRQENABLE_FIFO_OVF_IRQ_SHIFT                        (0x00000008U)
#define CSL_CSI2_IRQENABLE_FIFO_OVF_IRQ_RESETVAL                     (0x00000000U)
#define CSL_CSI2_IRQENABLE_FIFO_OVF_IRQ_MAX                          (0x00000001U)

#define CSL_CSI2_IRQENABLE_COMPLEXIO1_ERR_IRQ_MASK                   (0x00000200U)
#define CSL_CSI2_IRQENABLE_COMPLEXIO1_ERR_IRQ_SHIFT                  (0x00000009U)
#define CSL_CSI2_IRQENABLE_COMPLEXIO1_ERR_IRQ_RESETVAL               (0x00000000U)
#define CSL_CSI2_IRQENABLE_COMPLEXIO1_ERR_IRQ_MAX                    (0x00000001U)

#define CSL_CSI2_IRQENABLE_COMPLEXIO2_ERR_IRQ_MASK                   (0x00000400U)
#define CSL_CSI2_IRQENABLE_COMPLEXIO2_ERR_IRQ_SHIFT                  (0x0000000AU)
#define CSL_CSI2_IRQENABLE_COMPLEXIO2_ERR_IRQ_RESETVAL               (0x00000000U)
#define CSL_CSI2_IRQENABLE_COMPLEXIO2_ERR_IRQ_MAX                    (0x00000001U)

#define CSL_CSI2_IRQENABLE_ECC_NO_CORRECTION_IRQ_MASK                (0x00000800U)
#define CSL_CSI2_IRQENABLE_ECC_NO_CORRECTION_IRQ_SHIFT               (0x0000000BU)
#define CSL_CSI2_IRQENABLE_ECC_NO_CORRECTION_IRQ_RESETVAL            (0x00000000U)
#define CSL_CSI2_IRQENABLE_ECC_NO_CORRECTION_IRQ_MAX                 (0x00000001U)

#define CSL_CSI2_IRQENABLE_ECC_CORRECTION_IRQ_MASK                   (0x00001000U)
#define CSL_CSI2_IRQENABLE_ECC_CORRECTION_IRQ_SHIFT                  (0x0000000CU)
#define CSL_CSI2_IRQENABLE_ECC_CORRECTION_IRQ_RESETVAL               (0x00000000U)
#define CSL_CSI2_IRQENABLE_ECC_CORRECTION_IRQ_MAX                    (0x00000001U)

#define CSL_CSI2_IRQENABLE_SHORT_PACKET_IRQ_MASK                     (0x00002000U)
#define CSL_CSI2_IRQENABLE_SHORT_PACKET_IRQ_SHIFT                    (0x0000000DU)
#define CSL_CSI2_IRQENABLE_SHORT_PACKET_IRQ_RESETVAL                 (0x00000000U)
#define CSL_CSI2_IRQENABLE_SHORT_PACKET_IRQ_MAX                      (0x00000001U)

#define CSL_CSI2_IRQENABLE_OCP_ERR_IRQ_MASK                          (0x00004000U)
#define CSL_CSI2_IRQENABLE_OCP_ERR_IRQ_SHIFT                         (0x0000000EU)
#define CSL_CSI2_IRQENABLE_OCP_ERR_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_IRQENABLE_OCP_ERR_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_IRQENABLE_RES6_MASK                                 (0xFFFF8000U)
#define CSL_CSI2_IRQENABLE_RES6_SHIFT                                (0x0000000FU)
#define CSL_CSI2_IRQENABLE_RES6_RESETVAL                             (0x00000000U)
#define CSL_CSI2_IRQENABLE_RES6_MAX                                  (0x0001FFFFU)

#define CSL_CSI2_IRQENABLE_RESETVAL                                  (0x00000000U)

/* CSI2_CTRL */

#define CSL_CSI2_CTRL_IF_EN_MASK                                     (0x00000001U)
#define CSL_CSI2_CTRL_IF_EN_SHIFT                                    (0x00000000U)
#define CSL_CSI2_CTRL_IF_EN_RESETVAL                                 (0x00000000U)
#define CSL_CSI2_CTRL_IF_EN_MAX                                      (0x00000001U)

#define CSL_CSI2_CTRL_SECURE_MASK                                    (0x00000002U)
#define CSL_CSI2_CTRL_SECURE_SHIFT                                   (0x00000001U)
#define CSL_CSI2_CTRL_SECURE_RESETVAL                                (0x00000000U)
#define CSL_CSI2_CTRL_SECURE_MAX                                     (0x00000001U)

#define CSL_CSI2_CTRL_ECC_EN_MASK                                    (0x00000004U)
#define CSL_CSI2_CTRL_ECC_EN_SHIFT                                   (0x00000002U)
#define CSL_CSI2_CTRL_ECC_EN_RESETVAL                                (0x00000000U)
#define CSL_CSI2_CTRL_ECC_EN_MAX                                     (0x00000001U)

#define CSL_CSI2_CTRL_FRAME_MASK                                     (0x00000008U)
#define CSL_CSI2_CTRL_FRAME_SHIFT                                    (0x00000003U)
#define CSL_CSI2_CTRL_FRAME_RESETVAL                                 (0x00000000U)
#define CSL_CSI2_CTRL_FRAME_MAX                                      (0x00000001U)

#define CSL_CSI2_CTRL_ENDIANNESS_MASK                                (0x00000010U)
#define CSL_CSI2_CTRL_ENDIANNESS_SHIFT                               (0x00000004U)
#define CSL_CSI2_CTRL_ENDIANNESS_RESETVAL                            (0x00000000U)
#define CSL_CSI2_CTRL_ENDIANNESS_MAX                                 (0x00000001U)

#define CSL_CSI2_CTRL_BURST_SIZE_MASK                                (0x00000060U)
#define CSL_CSI2_CTRL_BURST_SIZE_SHIFT                               (0x00000005U)
#define CSL_CSI2_CTRL_BURST_SIZE_RESETVAL                            (0x00000000U)
#define CSL_CSI2_CTRL_BURST_SIZE_MAX                                 (0x00000003U)

#define CSL_CSI2_CTRL_DBG_EN_MASK                                    (0x00000080U)
#define CSL_CSI2_CTRL_DBG_EN_SHIFT                                   (0x00000007U)
#define CSL_CSI2_CTRL_DBG_EN_RESETVAL                                (0x00000000U)
#define CSL_CSI2_CTRL_DBG_EN_MAX                                     (0x00000001U)

#define CSL_CSI2_CTRL_VP_OUT_CTRL_MASK                               (0x00000300U)
#define CSL_CSI2_CTRL_VP_OUT_CTRL_SHIFT                              (0x00000008U)
#define CSL_CSI2_CTRL_VP_OUT_CTRL_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTRL_VP_OUT_CTRL_MAX                                (0x00000003U)

#define CSL_CSI2_CTRL_STREAMING_32_BIT_MASK                          (0x00000400U)
#define CSL_CSI2_CTRL_STREAMING_32_BIT_SHIFT                         (0x0000000AU)
#define CSL_CSI2_CTRL_STREAMING_32_BIT_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTRL_STREAMING_32_BIT_MAX                           (0x00000001U)

#define CSL_CSI2_CTRL_VP_ONLY_EN_MASK                                (0x00000800U)
#define CSL_CSI2_CTRL_VP_ONLY_EN_SHIFT                               (0x0000000BU)
#define CSL_CSI2_CTRL_VP_ONLY_EN_RESETVAL                            (0x00000000U)
#define CSL_CSI2_CTRL_VP_ONLY_EN_MAX                                 (0x00000001U)

#define CSL_CSI2_CTRL_RES8_MASK                                      (0x00001000U)
#define CSL_CSI2_CTRL_RES8_SHIFT                                     (0x0000000CU)
#define CSL_CSI2_CTRL_RES8_RESETVAL                                  (0x00000000U)
#define CSL_CSI2_CTRL_RES8_MAX                                       (0x00000001U)

#define CSL_CSI2_CTRL_NON_POSTED_WRITE_MASK                          (0x00002000U)
#define CSL_CSI2_CTRL_NON_POSTED_WRITE_SHIFT                         (0x0000000DU)
#define CSL_CSI2_CTRL_NON_POSTED_WRITE_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTRL_NON_POSTED_WRITE_MAX                           (0x00000001U)

#define CSL_CSI2_CTRL_STREAMING_MASK                                 (0x00004000U)
#define CSL_CSI2_CTRL_STREAMING_SHIFT                                (0x0000000EU)
#define CSL_CSI2_CTRL_STREAMING_RESETVAL                             (0x00000000U)
#define CSL_CSI2_CTRL_STREAMING_MAX                                  (0x00000001U)

#define CSL_CSI2_CTRL_VP_CLK_EN_MASK                                 (0x00008000U)
#define CSL_CSI2_CTRL_VP_CLK_EN_SHIFT                                (0x0000000FU)
#define CSL_CSI2_CTRL_VP_CLK_EN_RESETVAL                             (0x00000000U)
#define CSL_CSI2_CTRL_VP_CLK_EN_MAX                                  (0x00000001U)

#define CSL_CSI2_CTRL_BURST_SIZE_EXPAND_MASK                         (0x00010000U)
#define CSL_CSI2_CTRL_BURST_SIZE_EXPAND_SHIFT                        (0x00000010U)
#define CSL_CSI2_CTRL_BURST_SIZE_EXPAND_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTRL_BURST_SIZE_EXPAND_MAX                          (0x00000001U)

#define CSL_CSI2_CTRL_MFLAG_LEVL_MASK                                (0x000E0000U)
#define CSL_CSI2_CTRL_MFLAG_LEVL_SHIFT                               (0x00000011U)
#define CSL_CSI2_CTRL_MFLAG_LEVL_RESETVAL                            (0x00000000U)
#define CSL_CSI2_CTRL_MFLAG_LEVL_MAX                                 (0x00000007U)

#define CSL_CSI2_CTRL_MFLAG_LEVH_MASK                                (0x00700000U)
#define CSL_CSI2_CTRL_MFLAG_LEVH_SHIFT                               (0x00000014U)
#define CSL_CSI2_CTRL_MFLAG_LEVH_RESETVAL                            (0x00000000U)
#define CSL_CSI2_CTRL_MFLAG_LEVH_MAX                                 (0x00000007U)

#define CSL_CSI2_CTRL_RES7_MASK                                      (0xFF800000U)
#define CSL_CSI2_CTRL_RES7_SHIFT                                     (0x00000017U)
#define CSL_CSI2_CTRL_RES7_RESETVAL                                  (0x00000000U)
#define CSL_CSI2_CTRL_RES7_MAX                                       (0x000001FFU)

#define CSL_CSI2_CTRL_RESETVAL                                       (0x00000000U)

/* CSI2_DBG_H */

#define CSL_CSI2_DBG_H_DBG_MASK                                      (0xFFFFFFFFU)
#define CSL_CSI2_DBG_H_DBG_SHIFT                                     (0x00000000U)
#define CSL_CSI2_DBG_H_DBG_RESETVAL                                  (0x00000000U)
#define CSL_CSI2_DBG_H_DBG_MAX                                       (0xFFFFFFFFU)

#define CSL_CSI2_DBG_H_RESETVAL                                      (0x00000000U)

/* CSI2_GNQ */

#define CSL_CSI2_GNQ_NBCONTEXTS_MASK                                 (0x00000003U)
#define CSL_CSI2_GNQ_NBCONTEXTS_SHIFT                                (0x00000000U)
#define CSL_CSI2_GNQ_NBCONTEXTS_RESETVAL                             (0x00000003U)
#define CSL_CSI2_GNQ_NBCONTEXTS_MAX                                  (0x00000003U)

#define CSL_CSI2_GNQ_FIFODEPTH_MASK                                  (0x0000003CU)
#define CSL_CSI2_GNQ_FIFODEPTH_SHIFT                                 (0x00000002U)
#define CSL_CSI2_GNQ_FIFODEPTH_RESETVAL                              (0x00000006U)
#define CSL_CSI2_GNQ_FIFODEPTH_MAX                                   (0x0000000FU)

#define CSL_CSI2_GNQ_RES9_MASK                                       (0xFFFFFFC0U)
#define CSL_CSI2_GNQ_RES9_SHIFT                                      (0x00000006U)
#define CSL_CSI2_GNQ_RES9_RESETVAL                                   (0x00000000U)
#define CSL_CSI2_GNQ_RES9_MAX                                        (0x03FFFFFFU)

#define CSL_CSI2_GNQ_RESETVAL                                        (0x0000001BU)

/* CSI2_COMPLEXIO_CFG2 */

#define CSL_CSI2_COMPLEXIO_CFG2_CLOCK_POSITION_MASK                  (0x00000007U)
#define CSL_CSI2_COMPLEXIO_CFG2_CLOCK_POSITION_SHIFT                 (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG2_CLOCK_POSITION_RESETVAL              (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG2_CLOCK_POSITION_MAX                   (0x00000007U)

#define CSL_CSI2_COMPLEXIO_CFG2_CLOCK_POL_MASK                       (0x00000008U)
#define CSL_CSI2_COMPLEXIO_CFG2_CLOCK_POL_SHIFT                      (0x00000003U)
#define CSL_CSI2_COMPLEXIO_CFG2_CLOCK_POL_RESETVAL                   (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG2_CLOCK_POL_MAX                        (0x00000001U)

#define CSL_CSI2_COMPLEXIO_CFG2_DATA1_POSITION_MASK                  (0x00000070U)
#define CSL_CSI2_COMPLEXIO_CFG2_DATA1_POSITION_SHIFT                 (0x00000004U)
#define CSL_CSI2_COMPLEXIO_CFG2_DATA1_POSITION_RESETVAL              (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG2_DATA1_POSITION_MAX                   (0x00000007U)

#define CSL_CSI2_COMPLEXIO_CFG2_DATA1_POL_MASK                       (0x00000080U)
#define CSL_CSI2_COMPLEXIO_CFG2_DATA1_POL_SHIFT                      (0x00000007U)
#define CSL_CSI2_COMPLEXIO_CFG2_DATA1_POL_RESETVAL                   (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG2_DATA1_POL_MAX                        (0x00000001U)

#define CSL_CSI2_COMPLEXIO_CFG2_DATA2_POSITION_MASK                  (0x00000700U)
#define CSL_CSI2_COMPLEXIO_CFG2_DATA2_POSITION_SHIFT                 (0x00000008U)
#define CSL_CSI2_COMPLEXIO_CFG2_DATA2_POSITION_RESETVAL              (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG2_DATA2_POSITION_MAX                   (0x00000007U)

#define CSL_CSI2_COMPLEXIO_CFG2_DATA2_POL_MASK                       (0x00000800U)
#define CSL_CSI2_COMPLEXIO_CFG2_DATA2_POL_SHIFT                      (0x0000000BU)
#define CSL_CSI2_COMPLEXIO_CFG2_DATA2_POL_RESETVAL                   (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG2_DATA2_POL_MAX                        (0x00000001U)

#define CSL_CSI2_COMPLEXIO_CFG2_DATA3_POSITION_MASK                  (0x00007000U)
#define CSL_CSI2_COMPLEXIO_CFG2_DATA3_POSITION_SHIFT                 (0x0000000CU)
#define CSL_CSI2_COMPLEXIO_CFG2_DATA3_POSITION_RESETVAL              (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG2_DATA3_POSITION_MAX                   (0x00000007U)

#define CSL_CSI2_COMPLEXIO_CFG2_DATA3_POL_MASK                       (0x00008000U)
#define CSL_CSI2_COMPLEXIO_CFG2_DATA3_POL_SHIFT                      (0x0000000FU)
#define CSL_CSI2_COMPLEXIO_CFG2_DATA3_POL_RESETVAL                   (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG2_DATA3_POL_MAX                        (0x00000001U)

#define CSL_CSI2_COMPLEXIO_CFG2_DATA4_POSITION_MASK                  (0x00070000U)
#define CSL_CSI2_COMPLEXIO_CFG2_DATA4_POSITION_SHIFT                 (0x00000010U)
#define CSL_CSI2_COMPLEXIO_CFG2_DATA4_POSITION_RESETVAL              (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG2_DATA4_POSITION_MAX                   (0x00000007U)

#define CSL_CSI2_COMPLEXIO_CFG2_DATA4_POL_MASK                       (0x00080000U)
#define CSL_CSI2_COMPLEXIO_CFG2_DATA4_POL_SHIFT                      (0x00000013U)
#define CSL_CSI2_COMPLEXIO_CFG2_DATA4_POL_RESETVAL                   (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG2_DATA4_POL_MAX                        (0x00000001U)

#define CSL_CSI2_COMPLEXIO_CFG2_RES11_MASK                           (0x00F00000U)
#define CSL_CSI2_COMPLEXIO_CFG2_RES11_SHIFT                          (0x00000014U)
#define CSL_CSI2_COMPLEXIO_CFG2_RES11_RESETVAL                       (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG2_RES11_MAX                            (0x0000000FU)

#define CSL_CSI2_COMPLEXIO_CFG2_PWR_AUTO_MASK                        (0x01000000U)
#define CSL_CSI2_COMPLEXIO_CFG2_PWR_AUTO_SHIFT                       (0x00000018U)
#define CSL_CSI2_COMPLEXIO_CFG2_PWR_AUTO_RESETVAL                    (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG2_PWR_AUTO_MAX                         (0x00000001U)

#define CSL_CSI2_COMPLEXIO_CFG2_PWR_STATUS_MASK                      (0x06000000U)
#define CSL_CSI2_COMPLEXIO_CFG2_PWR_STATUS_SHIFT                     (0x00000019U)
#define CSL_CSI2_COMPLEXIO_CFG2_PWR_STATUS_RESETVAL                  (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG2_PWR_STATUS_MAX                       (0x00000003U)

#define CSL_CSI2_COMPLEXIO_CFG2_PWR_CMD_MASK                         (0x18000000U)
#define CSL_CSI2_COMPLEXIO_CFG2_PWR_CMD_SHIFT                        (0x0000001BU)
#define CSL_CSI2_COMPLEXIO_CFG2_PWR_CMD_RESETVAL                     (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG2_PWR_CMD_MAX                          (0x00000003U)

#define CSL_CSI2_COMPLEXIO_CFG2_RESET_DONE_MASK                      (0x20000000U)
#define CSL_CSI2_COMPLEXIO_CFG2_RESET_DONE_SHIFT                     (0x0000001DU)
#define CSL_CSI2_COMPLEXIO_CFG2_RESET_DONE_RESETVAL                  (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG2_RESET_DONE_MAX                       (0x00000001U)

#define CSL_CSI2_COMPLEXIO_CFG2_RESET_CTRL_MASK                      (0x40000000U)
#define CSL_CSI2_COMPLEXIO_CFG2_RESET_CTRL_SHIFT                     (0x0000001EU)
#define CSL_CSI2_COMPLEXIO_CFG2_RESET_CTRL_RESETVAL                  (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG2_RESET_CTRL_MAX                       (0x00000001U)

#define CSL_CSI2_COMPLEXIO_CFG2_RES10_MASK                           (0x80000000U)
#define CSL_CSI2_COMPLEXIO_CFG2_RES10_SHIFT                          (0x0000001FU)
#define CSL_CSI2_COMPLEXIO_CFG2_RES10_RESETVAL                       (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG2_RES10_MAX                            (0x00000001U)

#define CSL_CSI2_COMPLEXIO_CFG2_RESETVAL                             (0x00000000U)

/* CSI2_COMPLEXIO_CFG1 */

#define CSL_CSI2_COMPLEXIO_CFG1_CLOCK_POSITION_MASK                  (0x00000007U)
#define CSL_CSI2_COMPLEXIO_CFG1_CLOCK_POSITION_SHIFT                 (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG1_CLOCK_POSITION_RESETVAL              (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG1_CLOCK_POSITION_MAX                   (0x00000007U)

#define CSL_CSI2_COMPLEXIO_CFG1_CLOCK_POL_MASK                       (0x00000008U)
#define CSL_CSI2_COMPLEXIO_CFG1_CLOCK_POL_SHIFT                      (0x00000003U)
#define CSL_CSI2_COMPLEXIO_CFG1_CLOCK_POL_RESETVAL                   (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG1_CLOCK_POL_MAX                        (0x00000001U)

#define CSL_CSI2_COMPLEXIO_CFG1_DATA1_POSITION_MASK                  (0x00000070U)
#define CSL_CSI2_COMPLEXIO_CFG1_DATA1_POSITION_SHIFT                 (0x00000004U)
#define CSL_CSI2_COMPLEXIO_CFG1_DATA1_POSITION_RESETVAL              (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG1_DATA1_POSITION_MAX                   (0x00000007U)

#define CSL_CSI2_COMPLEXIO_CFG1_DATA1_POL_MASK                       (0x00000080U)
#define CSL_CSI2_COMPLEXIO_CFG1_DATA1_POL_SHIFT                      (0x00000007U)
#define CSL_CSI2_COMPLEXIO_CFG1_DATA1_POL_RESETVAL                   (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG1_DATA1_POL_MAX                        (0x00000001U)

#define CSL_CSI2_COMPLEXIO_CFG1_DATA2_POSITION_MASK                  (0x00000700U)
#define CSL_CSI2_COMPLEXIO_CFG1_DATA2_POSITION_SHIFT                 (0x00000008U)
#define CSL_CSI2_COMPLEXIO_CFG1_DATA2_POSITION_RESETVAL              (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG1_DATA2_POSITION_MAX                   (0x00000007U)

#define CSL_CSI2_COMPLEXIO_CFG1_DATA2_POL_MASK                       (0x00000800U)
#define CSL_CSI2_COMPLEXIO_CFG1_DATA2_POL_SHIFT                      (0x0000000BU)
#define CSL_CSI2_COMPLEXIO_CFG1_DATA2_POL_RESETVAL                   (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG1_DATA2_POL_MAX                        (0x00000001U)

#define CSL_CSI2_COMPLEXIO_CFG1_DATA3_POSITION_MASK                  (0x00007000U)
#define CSL_CSI2_COMPLEXIO_CFG1_DATA3_POSITION_SHIFT                 (0x0000000CU)
#define CSL_CSI2_COMPLEXIO_CFG1_DATA3_POSITION_RESETVAL              (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG1_DATA3_POSITION_MAX                   (0x00000007U)

#define CSL_CSI2_COMPLEXIO_CFG1_DATA3_POL_MASK                       (0x00008000U)
#define CSL_CSI2_COMPLEXIO_CFG1_DATA3_POL_SHIFT                      (0x0000000FU)
#define CSL_CSI2_COMPLEXIO_CFG1_DATA3_POL_RESETVAL                   (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG1_DATA3_POL_MAX                        (0x00000001U)

#define CSL_CSI2_COMPLEXIO_CFG1_DATA4_POSITION_MASK                  (0x00070000U)
#define CSL_CSI2_COMPLEXIO_CFG1_DATA4_POSITION_SHIFT                 (0x00000010U)
#define CSL_CSI2_COMPLEXIO_CFG1_DATA4_POSITION_RESETVAL              (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG1_DATA4_POSITION_MAX                   (0x00000007U)

#define CSL_CSI2_COMPLEXIO_CFG1_DATA4_POL_MASK                       (0x00080000U)
#define CSL_CSI2_COMPLEXIO_CFG1_DATA4_POL_SHIFT                      (0x00000013U)
#define CSL_CSI2_COMPLEXIO_CFG1_DATA4_POL_RESETVAL                   (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG1_DATA4_POL_MAX                        (0x00000001U)

#define CSL_CSI2_COMPLEXIO_CFG1_RES13_MASK                           (0x00F00000U)
#define CSL_CSI2_COMPLEXIO_CFG1_RES13_SHIFT                          (0x00000014U)
#define CSL_CSI2_COMPLEXIO_CFG1_RES13_RESETVAL                       (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG1_RES13_MAX                            (0x0000000FU)

#define CSL_CSI2_COMPLEXIO_CFG1_PWR_AUTO_MASK                        (0x01000000U)
#define CSL_CSI2_COMPLEXIO_CFG1_PWR_AUTO_SHIFT                       (0x00000018U)
#define CSL_CSI2_COMPLEXIO_CFG1_PWR_AUTO_RESETVAL                    (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG1_PWR_AUTO_MAX                         (0x00000001U)

#define CSL_CSI2_COMPLEXIO_CFG1_PWR_STATUS_MASK                      (0x06000000U)
#define CSL_CSI2_COMPLEXIO_CFG1_PWR_STATUS_SHIFT                     (0x00000019U)
#define CSL_CSI2_COMPLEXIO_CFG1_PWR_STATUS_RESETVAL                  (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG1_PWR_STATUS_MAX                       (0x00000003U)

#define CSL_CSI2_COMPLEXIO_CFG1_PWR_CMD_MASK                         (0x18000000U)
#define CSL_CSI2_COMPLEXIO_CFG1_PWR_CMD_SHIFT                        (0x0000001BU)
#define CSL_CSI2_COMPLEXIO_CFG1_PWR_CMD_RESETVAL                     (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG1_PWR_CMD_MAX                          (0x00000003U)

#define CSL_CSI2_COMPLEXIO_CFG1_RESET_DONE_MASK                      (0x20000000U)
#define CSL_CSI2_COMPLEXIO_CFG1_RESET_DONE_SHIFT                     (0x0000001DU)
#define CSL_CSI2_COMPLEXIO_CFG1_RESET_DONE_RESETVAL                  (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG1_RESET_DONE_MAX                       (0x00000001U)

#define CSL_CSI2_COMPLEXIO_CFG1_RESET_CTRL_MASK                      (0x40000000U)
#define CSL_CSI2_COMPLEXIO_CFG1_RESET_CTRL_SHIFT                     (0x0000001EU)
#define CSL_CSI2_COMPLEXIO_CFG1_RESET_CTRL_RESETVAL                  (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG1_RESET_CTRL_MAX                       (0x00000001U)

#define CSL_CSI2_COMPLEXIO_CFG1_RES12_MASK                           (0x80000000U)
#define CSL_CSI2_COMPLEXIO_CFG1_RES12_SHIFT                          (0x0000001FU)
#define CSL_CSI2_COMPLEXIO_CFG1_RES12_RESETVAL                       (0x00000000U)
#define CSL_CSI2_COMPLEXIO_CFG1_RES12_MAX                            (0x00000001U)

#define CSL_CSI2_COMPLEXIO_CFG1_RESETVAL                             (0x00000000U)

/* CSI2_COMPLEXIO1_IRQSTATUS */

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS1_MASK                 (0x00000001U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS1_SHIFT                (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS1_RESETVAL             (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS1_MAX                  (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS2_MASK                 (0x00000002U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS2_SHIFT                (0x00000001U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS2_RESETVAL             (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS2_MAX                  (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS3_MASK                 (0x00000004U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS3_SHIFT                (0x00000002U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS3_RESETVAL             (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS3_MAX                  (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS4_MASK                 (0x00000008U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS4_SHIFT                (0x00000003U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS4_RESETVAL             (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS4_MAX                  (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS5_MASK                 (0x00000010U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS5_SHIFT                (0x00000004U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS5_RESETVAL             (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS5_MAX                  (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS1_MASK             (0x00000020U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS1_SHIFT            (0x00000005U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS1_RESETVAL         (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS1_MAX              (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS2_MASK             (0x00000040U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS2_SHIFT            (0x00000006U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS2_RESETVAL         (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS2_MAX              (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS3_MASK             (0x00000080U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS3_SHIFT            (0x00000007U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS3_RESETVAL         (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS3_MAX              (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS4_MASK             (0x00000100U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS4_SHIFT            (0x00000008U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS4_RESETVAL         (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS4_MAX              (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS5_MASK             (0x00000200U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS5_SHIFT            (0x00000009U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS5_RESETVAL         (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS5_MAX              (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC1_MASK                   (0x00000400U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC1_SHIFT                  (0x0000000AU)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC1_RESETVAL               (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC1_MAX                    (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC2_MASK                   (0x00000800U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC2_SHIFT                  (0x0000000BU)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC2_RESETVAL               (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC2_MAX                    (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC3_MASK                   (0x00001000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC3_SHIFT                  (0x0000000CU)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC3_RESETVAL               (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC3_MAX                    (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC4_MASK                   (0x00002000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC4_SHIFT                  (0x0000000DU)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC4_RESETVAL               (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC4_MAX                    (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC5_MASK                   (0x00004000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC5_SHIFT                  (0x0000000EU)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC5_RESETVAL               (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC5_MAX                    (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL1_MASK               (0x00008000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL1_SHIFT              (0x0000000FU)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL1_RESETVAL           (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL1_MAX                (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL2_MASK               (0x00010000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL2_SHIFT              (0x00000010U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL2_RESETVAL           (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL2_MAX                (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL3_MASK               (0x00020000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL3_SHIFT              (0x00000011U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL3_RESETVAL           (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL3_MAX                (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL4_MASK               (0x00040000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL4_SHIFT              (0x00000012U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL4_RESETVAL           (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL4_MAX                (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL5_MASK               (0x00080000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL5_SHIFT              (0x00000013U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL5_RESETVAL           (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL5_MAX                (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM1_MASK                (0x00100000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM1_SHIFT               (0x00000014U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM1_RESETVAL            (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM1_MAX                 (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM2_MASK                (0x00200000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM2_SHIFT               (0x00000015U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM2_RESETVAL            (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM2_MAX                 (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM3_MASK                (0x00400000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM3_SHIFT               (0x00000016U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM3_RESETVAL            (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM3_MAX                 (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM4_MASK                (0x00800000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM4_SHIFT               (0x00000017U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM4_RESETVAL            (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM4_MAX                 (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM5_MASK                (0x01000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM5_SHIFT               (0x00000018U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM5_RESETVAL            (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM5_MAX                 (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEALLULPMENTER_MASK         (0x02000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEALLULPMENTER_SHIFT        (0x00000019U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEALLULPMENTER_RESETVAL     (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEALLULPMENTER_MAX          (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEALLULPMEXIT_MASK          (0x04000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEALLULPMEXIT_SHIFT         (0x0000001AU)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEALLULPMEXIT_RESETVAL      (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEALLULPMEXIT_MAX           (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_RES14_MASK                     (0xF8000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_RES14_SHIFT                    (0x0000001BU)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_RES14_RESETVAL                 (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_RES14_MAX                      (0x0000001FU)

#define CSL_CSI2_COMPLEXIO1_IRQSTATUS_RESETVAL                       (0x00000000U)

/* CSI2_COMPLEXIO2_IRQSTATUS */

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTHS1_MASK                 (0x00000001U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTHS1_SHIFT                (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTHS1_RESETVAL             (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTHS1_MAX                  (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTHS2_MASK                 (0x00000002U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTHS2_SHIFT                (0x00000001U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTHS2_RESETVAL             (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTHS2_MAX                  (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTHS3_MASK                 (0x00000004U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTHS3_SHIFT                (0x00000002U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTHS3_RESETVAL             (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTHS3_MAX                  (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTHS4_MASK                 (0x00000008U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTHS4_SHIFT                (0x00000003U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTHS4_RESETVAL             (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTHS4_MAX                  (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTHS5_MASK                 (0x00000010U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTHS5_SHIFT                (0x00000004U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTHS5_RESETVAL             (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTHS5_MAX                  (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTSYNCHS1_MASK             (0x00000020U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTSYNCHS1_SHIFT            (0x00000005U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTSYNCHS1_RESETVAL         (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTSYNCHS1_MAX              (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTSYNCHS2_MASK             (0x00000040U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTSYNCHS2_SHIFT            (0x00000006U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTSYNCHS2_RESETVAL         (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTSYNCHS2_MAX              (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTSYNCHS3_MASK             (0x00000080U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTSYNCHS3_SHIFT            (0x00000007U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTSYNCHS3_RESETVAL         (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTSYNCHS3_MAX              (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTSYNCHS4_MASK             (0x00000100U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTSYNCHS4_SHIFT            (0x00000008U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTSYNCHS4_RESETVAL         (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTSYNCHS4_MAX              (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTSYNCHS5_MASK             (0x00000200U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTSYNCHS5_SHIFT            (0x00000009U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTSYNCHS5_RESETVAL         (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRSOTSYNCHS5_MAX              (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRESC1_MASK                   (0x00000400U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRESC1_SHIFT                  (0x0000000AU)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRESC1_RESETVAL               (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRESC1_MAX                    (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRESC2_MASK                   (0x00000800U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRESC2_SHIFT                  (0x0000000BU)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRESC2_RESETVAL               (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRESC2_MAX                    (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRESC3_MASK                   (0x00001000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRESC3_SHIFT                  (0x0000000CU)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRESC3_RESETVAL               (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRESC3_MAX                    (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRESC4_MASK                   (0x00002000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRESC4_SHIFT                  (0x0000000DU)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRESC4_RESETVAL               (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRESC4_MAX                    (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRESC5_MASK                   (0x00004000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRESC5_SHIFT                  (0x0000000EU)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRESC5_RESETVAL               (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRESC5_MAX                    (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRCONTROL1_MASK               (0x00008000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRCONTROL1_SHIFT              (0x0000000FU)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRCONTROL1_RESETVAL           (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRCONTROL1_MAX                (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRCONTROL2_MASK               (0x00010000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRCONTROL2_SHIFT              (0x00000010U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRCONTROL2_RESETVAL           (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRCONTROL2_MAX                (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRCONTROL3_MASK               (0x00020000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRCONTROL3_SHIFT              (0x00000011U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRCONTROL3_RESETVAL           (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRCONTROL3_MAX                (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRCONTROL4_MASK               (0x00040000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRCONTROL4_SHIFT              (0x00000012U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRCONTROL4_RESETVAL           (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRCONTROL4_MAX                (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRCONTROL5_MASK               (0x00080000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRCONTROL5_SHIFT              (0x00000013U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRCONTROL5_RESETVAL           (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_ERRCONTROL5_MAX                (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEULPM1_MASK                (0x00100000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEULPM1_SHIFT               (0x00000014U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEULPM1_RESETVAL            (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEULPM1_MAX                 (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEULPM2_MASK                (0x00200000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEULPM2_SHIFT               (0x00000015U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEULPM2_RESETVAL            (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEULPM2_MAX                 (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEULPM3_MASK                (0x00400000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEULPM3_SHIFT               (0x00000016U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEULPM3_RESETVAL            (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEULPM3_MAX                 (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEULPM4_MASK                (0x00800000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEULPM4_SHIFT               (0x00000017U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEULPM4_RESETVAL            (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEULPM4_MAX                 (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEULPM5_MASK                (0x01000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEULPM5_SHIFT               (0x00000018U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEULPM5_RESETVAL            (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEULPM5_MAX                 (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEALLULPMENTER_MASK         (0x02000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEALLULPMENTER_SHIFT        (0x00000019U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEALLULPMENTER_RESETVAL     (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEALLULPMENTER_MAX          (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEALLULPMEXIT_MASK          (0x04000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEALLULPMEXIT_SHIFT         (0x0000001AU)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEALLULPMEXIT_RESETVAL      (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_STATEALLULPMEXIT_MAX           (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_RES15_MASK                     (0xF8000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_RES15_SHIFT                    (0x0000001BU)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_RES15_RESETVAL                 (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_RES15_MAX                      (0x0000001FU)

#define CSL_CSI2_COMPLEXIO2_IRQSTATUS_RESETVAL                       (0x00000000U)

/* CSI2_SHORT_PACKET */

#define CSL_CSI2_SHORT_PACKET_SHORT_PACKET_MASK                      (0x00FFFFFFU)
#define CSL_CSI2_SHORT_PACKET_SHORT_PACKET_SHIFT                     (0x00000000U)
#define CSL_CSI2_SHORT_PACKET_SHORT_PACKET_RESETVAL                  (0x00000000U)
#define CSL_CSI2_SHORT_PACKET_SHORT_PACKET_MAX                       (0x00FFFFFFU)

#define CSL_CSI2_SHORT_PACKET_RES16_MASK                             (0xFF000000U)
#define CSL_CSI2_SHORT_PACKET_RES16_SHIFT                            (0x00000018U)
#define CSL_CSI2_SHORT_PACKET_RES16_RESETVAL                         (0x00000000U)
#define CSL_CSI2_SHORT_PACKET_RES16_MAX                              (0x000000FFU)

#define CSL_CSI2_SHORT_PACKET_RESETVAL                               (0x00000000U)

/* CSI2_COMPLEXIO1_IRQENABLE */

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS1_MASK                 (0x00000001U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS1_SHIFT                (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS1_RESETVAL             (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS1_MAX                  (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS2_MASK                 (0x00000002U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS2_SHIFT                (0x00000001U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS2_RESETVAL             (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS2_MAX                  (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS3_MASK                 (0x00000004U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS3_SHIFT                (0x00000002U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS3_RESETVAL             (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS3_MAX                  (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS4_MASK                 (0x00000008U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS4_SHIFT                (0x00000003U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS4_RESETVAL             (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS4_MAX                  (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS5_MASK                 (0x00000010U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS5_SHIFT                (0x00000004U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS5_RESETVAL             (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS5_MAX                  (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS1_MASK             (0x00000020U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS1_SHIFT            (0x00000005U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS1_RESETVAL         (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS1_MAX              (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS2_MASK             (0x00000040U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS2_SHIFT            (0x00000006U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS2_RESETVAL         (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS2_MAX              (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS3_MASK             (0x00000080U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS3_SHIFT            (0x00000007U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS3_RESETVAL         (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS3_MAX              (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS4_MASK             (0x00000100U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS4_SHIFT            (0x00000008U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS4_RESETVAL         (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS4_MAX              (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS5_MASK             (0x00000200U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS5_SHIFT            (0x00000009U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS5_RESETVAL         (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS5_MAX              (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC1_MASK                   (0x00000400U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC1_SHIFT                  (0x0000000AU)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC1_RESETVAL               (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC1_MAX                    (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC2_MASK                   (0x00000800U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC2_SHIFT                  (0x0000000BU)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC2_RESETVAL               (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC2_MAX                    (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC3_MASK                   (0x00001000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC3_SHIFT                  (0x0000000CU)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC3_RESETVAL               (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC3_MAX                    (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC4_MASK                   (0x00002000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC4_SHIFT                  (0x0000000DU)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC4_RESETVAL               (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC4_MAX                    (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC5_MASK                   (0x00004000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC5_SHIFT                  (0x0000000EU)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC5_RESETVAL               (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC5_MAX                    (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL1_MASK               (0x00008000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL1_SHIFT              (0x0000000FU)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL1_RESETVAL           (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL1_MAX                (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL2_MASK               (0x00010000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL2_SHIFT              (0x00000010U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL2_RESETVAL           (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL2_MAX                (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL3_MASK               (0x00020000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL3_SHIFT              (0x00000011U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL3_RESETVAL           (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL3_MAX                (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL4_MASK               (0x00040000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL4_SHIFT              (0x00000012U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL4_RESETVAL           (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL4_MAX                (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL5_MASK               (0x00080000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL5_SHIFT              (0x00000013U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL5_RESETVAL           (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL5_MAX                (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM1_MASK                (0x00100000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM1_SHIFT               (0x00000014U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM1_RESETVAL            (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM1_MAX                 (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM2_MASK                (0x00200000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM2_SHIFT               (0x00000015U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM2_RESETVAL            (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM2_MAX                 (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM3_MASK                (0x00400000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM3_SHIFT               (0x00000016U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM3_RESETVAL            (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM3_MAX                 (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM4_MASK                (0x00800000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM4_SHIFT               (0x00000017U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM4_RESETVAL            (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM4_MAX                 (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM5_MASK                (0x01000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM5_SHIFT               (0x00000018U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM5_RESETVAL            (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM5_MAX                 (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEALLULPMENTER_MASK         (0x02000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEALLULPMENTER_SHIFT        (0x00000019U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEALLULPMENTER_RESETVAL     (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEALLULPMENTER_MAX          (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEALLULPMEXIT_MASK          (0x04000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEALLULPMEXIT_SHIFT         (0x0000001AU)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEALLULPMEXIT_RESETVAL      (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEALLULPMEXIT_MAX           (0x00000001U)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_RES17_MASK                     (0xF8000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_RES17_SHIFT                    (0x0000001BU)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_RES17_RESETVAL                 (0x00000000U)
#define CSL_CSI2_COMPLEXIO1_IRQENABLE_RES17_MAX                      (0x0000001FU)

#define CSL_CSI2_COMPLEXIO1_IRQENABLE_RESETVAL                       (0x00000000U)

/* CSI2_COMPLEXIO2_IRQENABLE */

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTHS1_MASK                 (0x00000001U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTHS1_SHIFT                (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTHS1_RESETVAL             (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTHS1_MAX                  (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTHS2_MASK                 (0x00000002U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTHS2_SHIFT                (0x00000001U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTHS2_RESETVAL             (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTHS2_MAX                  (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTHS3_MASK                 (0x00000004U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTHS3_SHIFT                (0x00000002U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTHS3_RESETVAL             (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTHS3_MAX                  (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTHS4_MASK                 (0x00000008U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTHS4_SHIFT                (0x00000003U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTHS4_RESETVAL             (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTHS4_MAX                  (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTHS5_MASK                 (0x00000010U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTHS5_SHIFT                (0x00000004U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTHS5_RESETVAL             (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTHS5_MAX                  (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTSYNCHS1_MASK             (0x00000020U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTSYNCHS1_SHIFT            (0x00000005U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTSYNCHS1_RESETVAL         (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTSYNCHS1_MAX              (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTSYNCHS2_MASK             (0x00000040U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTSYNCHS2_SHIFT            (0x00000006U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTSYNCHS2_RESETVAL         (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTSYNCHS2_MAX              (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTSYNCHS3_MASK             (0x00000080U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTSYNCHS3_SHIFT            (0x00000007U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTSYNCHS3_RESETVAL         (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTSYNCHS3_MAX              (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTSYNCHS4_MASK             (0x00000100U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTSYNCHS4_SHIFT            (0x00000008U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTSYNCHS4_RESETVAL         (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTSYNCHS4_MAX              (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTSYNCHS5_MASK             (0x00000200U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTSYNCHS5_SHIFT            (0x00000009U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTSYNCHS5_RESETVAL         (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRSOTSYNCHS5_MAX              (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRESC1_MASK                   (0x00000400U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRESC1_SHIFT                  (0x0000000AU)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRESC1_RESETVAL               (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRESC1_MAX                    (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRESC2_MASK                   (0x00000800U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRESC2_SHIFT                  (0x0000000BU)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRESC2_RESETVAL               (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRESC2_MAX                    (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRESC3_MASK                   (0x00001000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRESC3_SHIFT                  (0x0000000CU)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRESC3_RESETVAL               (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRESC3_MAX                    (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRESC4_MASK                   (0x00002000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRESC4_SHIFT                  (0x0000000DU)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRESC4_RESETVAL               (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRESC4_MAX                    (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRESC5_MASK                   (0x00004000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRESC5_SHIFT                  (0x0000000EU)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRESC5_RESETVAL               (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRESC5_MAX                    (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRCONTROL1_MASK               (0x00008000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRCONTROL1_SHIFT              (0x0000000FU)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRCONTROL1_RESETVAL           (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRCONTROL1_MAX                (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRCONTROL2_MASK               (0x00010000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRCONTROL2_SHIFT              (0x00000010U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRCONTROL2_RESETVAL           (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRCONTROL2_MAX                (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRCONTROL3_MASK               (0x00020000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRCONTROL3_SHIFT              (0x00000011U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRCONTROL3_RESETVAL           (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRCONTROL3_MAX                (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRCONTROL4_MASK               (0x00040000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRCONTROL4_SHIFT              (0x00000012U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRCONTROL4_RESETVAL           (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRCONTROL4_MAX                (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRCONTROL5_MASK               (0x00080000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRCONTROL5_SHIFT              (0x00000013U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRCONTROL5_RESETVAL           (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_ERRCONTROL5_MAX                (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEULPM1_MASK                (0x00100000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEULPM1_SHIFT               (0x00000014U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEULPM1_RESETVAL            (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEULPM1_MAX                 (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEULPM2_MASK                (0x00200000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEULPM2_SHIFT               (0x00000015U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEULPM2_RESETVAL            (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEULPM2_MAX                 (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEULPM3_MASK                (0x00400000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEULPM3_SHIFT               (0x00000016U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEULPM3_RESETVAL            (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEULPM3_MAX                 (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEULPM4_MASK                (0x00800000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEULPM4_SHIFT               (0x00000017U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEULPM4_RESETVAL            (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEULPM4_MAX                 (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEULPM5_MASK                (0x01000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEULPM5_SHIFT               (0x00000018U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEULPM5_RESETVAL            (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEULPM5_MAX                 (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEALLULPMENTER_MASK         (0x02000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEALLULPMENTER_SHIFT        (0x00000019U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEALLULPMENTER_RESETVAL     (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEALLULPMENTER_MAX          (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEALLULPMEXIT_MASK          (0x04000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEALLULPMEXIT_SHIFT         (0x0000001AU)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEALLULPMEXIT_RESETVAL      (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_STATEALLULPMEXIT_MAX           (0x00000001U)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_RES18_MASK                     (0xF8000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_RES18_SHIFT                    (0x0000001BU)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_RES18_RESETVAL                 (0x00000000U)
#define CSL_CSI2_COMPLEXIO2_IRQENABLE_RES18_MAX                      (0x0000001FU)

#define CSL_CSI2_COMPLEXIO2_IRQENABLE_RESETVAL                       (0x00000000U)

/* CSI2_DBG_P */

#define CSL_CSI2_DBG_P_DBG_MASK                                      (0xFFFFFFFFU)
#define CSL_CSI2_DBG_P_DBG_SHIFT                                     (0x00000000U)
#define CSL_CSI2_DBG_P_DBG_RESETVAL                                  (0x00000000U)
#define CSL_CSI2_DBG_P_DBG_MAX                                       (0xFFFFFFFFU)

#define CSL_CSI2_DBG_P_RESETVAL                                      (0x00000000U)

/* CSI2_TIMING */

#define CSL_CSI2_TIMING_STOP_STATE_COUNTER_IO1_MASK                  (0x00001FFFU)
#define CSL_CSI2_TIMING_STOP_STATE_COUNTER_IO1_SHIFT                 (0x00000000U)
#define CSL_CSI2_TIMING_STOP_STATE_COUNTER_IO1_RESETVAL              (0x00001FFFU)
#define CSL_CSI2_TIMING_STOP_STATE_COUNTER_IO1_MAX                   (0x00001FFFU)

#define CSL_CSI2_TIMING_STOP_STATE_X4_IO1_MASK                       (0x00002000U)
#define CSL_CSI2_TIMING_STOP_STATE_X4_IO1_SHIFT                      (0x0000000DU)
#define CSL_CSI2_TIMING_STOP_STATE_X4_IO1_RESETVAL                   (0x00000001U)
#define CSL_CSI2_TIMING_STOP_STATE_X4_IO1_MAX                        (0x00000001U)

#define CSL_CSI2_TIMING_STOP_STATE_X16_IO1_MASK                      (0x00004000U)
#define CSL_CSI2_TIMING_STOP_STATE_X16_IO1_SHIFT                     (0x0000000EU)
#define CSL_CSI2_TIMING_STOP_STATE_X16_IO1_RESETVAL                  (0x00000001U)
#define CSL_CSI2_TIMING_STOP_STATE_X16_IO1_MAX                       (0x00000001U)

#define CSL_CSI2_TIMING_FORCE_RX_MODE_IO1_MASK                       (0x00008000U)
#define CSL_CSI2_TIMING_FORCE_RX_MODE_IO1_SHIFT                      (0x0000000FU)
#define CSL_CSI2_TIMING_FORCE_RX_MODE_IO1_RESETVAL                   (0x00000000U)
#define CSL_CSI2_TIMING_FORCE_RX_MODE_IO1_MAX                        (0x00000001U)

#define CSL_CSI2_TIMING_STOP_STATE_COUNTER_IO2_MASK                  (0x1FFF0000U)
#define CSL_CSI2_TIMING_STOP_STATE_COUNTER_IO2_SHIFT                 (0x00000010U)
#define CSL_CSI2_TIMING_STOP_STATE_COUNTER_IO2_RESETVAL              (0x00001FFFU)
#define CSL_CSI2_TIMING_STOP_STATE_COUNTER_IO2_MAX                   (0x00001FFFU)

#define CSL_CSI2_TIMING_STOP_STATE_X4_IO2_MASK                       (0x20000000U)
#define CSL_CSI2_TIMING_STOP_STATE_X4_IO2_SHIFT                      (0x0000001DU)
#define CSL_CSI2_TIMING_STOP_STATE_X4_IO2_RESETVAL                   (0x00000001U)
#define CSL_CSI2_TIMING_STOP_STATE_X4_IO2_MAX                        (0x00000001U)

#define CSL_CSI2_TIMING_STOP_STATE_X16_IO2_MASK                      (0x40000000U)
#define CSL_CSI2_TIMING_STOP_STATE_X16_IO2_SHIFT                     (0x0000001EU)
#define CSL_CSI2_TIMING_STOP_STATE_X16_IO2_RESETVAL                  (0x00000001U)
#define CSL_CSI2_TIMING_STOP_STATE_X16_IO2_MAX                       (0x00000001U)

#define CSL_CSI2_TIMING_FORCE_RX_MODE_IO2_MASK                       (0x80000000U)
#define CSL_CSI2_TIMING_FORCE_RX_MODE_IO2_SHIFT                      (0x0000001FU)
#define CSL_CSI2_TIMING_FORCE_RX_MODE_IO2_RESETVAL                   (0x00000000U)
#define CSL_CSI2_TIMING_FORCE_RX_MODE_IO2_MAX                        (0x00000001U)

#define CSL_CSI2_TIMING_RESETVAL                                     (0x7FFF7FFFU)

/* CSI2_CTX0_CTRL1 */

#define CSL_CSI2_CTX0_CTRL1_CTX_EN_MASK                              (0x00000001U)
#define CSL_CSI2_CTX0_CTRL1_CTX_EN_SHIFT                             (0x00000000U)
#define CSL_CSI2_CTX0_CTRL1_CTX_EN_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX0_CTRL1_CTX_EN_MAX                               (0x00000001U)

#define CSL_CSI2_CTX0_CTRL1_LINE_MODULO_MASK                         (0x00000002U)
#define CSL_CSI2_CTX0_CTRL1_LINE_MODULO_SHIFT                        (0x00000001U)
#define CSL_CSI2_CTX0_CTRL1_LINE_MODULO_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX0_CTRL1_LINE_MODULO_MAX                          (0x00000001U)

#define CSL_CSI2_CTX0_CTRL1_VP_FORCE_MASK                            (0x00000004U)
#define CSL_CSI2_CTX0_CTRL1_VP_FORCE_SHIFT                           (0x00000002U)
#define CSL_CSI2_CTX0_CTRL1_VP_FORCE_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX0_CTRL1_VP_FORCE_MAX                             (0x00000001U)

#define CSL_CSI2_CTX0_CTRL1_PING_PONG_MASK                           (0x00000008U)
#define CSL_CSI2_CTX0_CTRL1_PING_PONG_SHIFT                          (0x00000003U)
#define CSL_CSI2_CTX0_CTRL1_PING_PONG_RESETVAL                       (0x00000001U)
#define CSL_CSI2_CTX0_CTRL1_PING_PONG_MAX                            (0x00000001U)

#define CSL_CSI2_CTX0_CTRL1_COUNT_UNLOCK_MASK                        (0x00000010U)
#define CSL_CSI2_CTX0_CTRL1_COUNT_UNLOCK_SHIFT                       (0x00000004U)
#define CSL_CSI2_CTX0_CTRL1_COUNT_UNLOCK_RESETVAL                    (0x00000000U)
#define CSL_CSI2_CTX0_CTRL1_COUNT_UNLOCK_MAX                         (0x00000001U)

#define CSL_CSI2_CTX0_CTRL1_CS_EN_MASK                               (0x00000020U)
#define CSL_CSI2_CTX0_CTRL1_CS_EN_SHIFT                              (0x00000005U)
#define CSL_CSI2_CTX0_CTRL1_CS_EN_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX0_CTRL1_CS_EN_MAX                                (0x00000001U)

#define CSL_CSI2_CTX0_CTRL1_EOL_EN_MASK                              (0x00000040U)
#define CSL_CSI2_CTX0_CTRL1_EOL_EN_SHIFT                             (0x00000006U)
#define CSL_CSI2_CTX0_CTRL1_EOL_EN_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX0_CTRL1_EOL_EN_MAX                               (0x00000001U)

#define CSL_CSI2_CTX0_CTRL1_EOF_EN_MASK                              (0x00000080U)
#define CSL_CSI2_CTX0_CTRL1_EOF_EN_SHIFT                             (0x00000007U)
#define CSL_CSI2_CTX0_CTRL1_EOF_EN_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX0_CTRL1_EOF_EN_MAX                               (0x00000001U)

#define CSL_CSI2_CTX0_CTRL1_COUNT_MASK                               (0x0000FF00U)
#define CSL_CSI2_CTX0_CTRL1_COUNT_SHIFT                              (0x00000008U)
#define CSL_CSI2_CTX0_CTRL1_COUNT_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX0_CTRL1_COUNT_MAX                                (0x000000FFU)

#define CSL_CSI2_CTX0_CTRL1_FEC_NUMBER_MASK                          (0x00FF0000U)
#define CSL_CSI2_CTX0_CTRL1_FEC_NUMBER_SHIFT                         (0x00000010U)
#define CSL_CSI2_CTX0_CTRL1_FEC_NUMBER_RESETVAL                      (0x00000001U)
#define CSL_CSI2_CTX0_CTRL1_FEC_NUMBER_MAX                           (0x000000FFU)

#define CSL_CSI2_CTX0_CTRL1_TRANSCODE_MASK                           (0x0F000000U)
#define CSL_CSI2_CTX0_CTRL1_TRANSCODE_SHIFT                          (0x00000018U)
#define CSL_CSI2_CTX0_CTRL1_TRANSCODE_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX0_CTRL1_TRANSCODE_MAX                            (0x0000000FU)

#define CSL_CSI2_CTX0_CTRL1_HSCALE_MASK                              (0x10000000U)
#define CSL_CSI2_CTX0_CTRL1_HSCALE_SHIFT                             (0x0000001CU)
#define CSL_CSI2_CTX0_CTRL1_HSCALE_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX0_CTRL1_HSCALE_MAX                               (0x00000001U)

#define CSL_CSI2_CTX0_CTRL1_RES19_MASK                               (0x20000000U)
#define CSL_CSI2_CTX0_CTRL1_RES19_SHIFT                              (0x0000001DU)
#define CSL_CSI2_CTX0_CTRL1_RES19_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX0_CTRL1_RES19_MAX                                (0x00000001U)

#define CSL_CSI2_CTX0_CTRL1_GENERIC_MASK                             (0x40000000U)
#define CSL_CSI2_CTX0_CTRL1_GENERIC_SHIFT                            (0x0000001EU)
#define CSL_CSI2_CTX0_CTRL1_GENERIC_RESETVAL                         (0x00000000U)
#define CSL_CSI2_CTX0_CTRL1_GENERIC_MAX                              (0x00000001U)

#define CSL_CSI2_CTX0_CTRL1_BYTESWAP_MASK                            (0x80000000U)
#define CSL_CSI2_CTX0_CTRL1_BYTESWAP_SHIFT                           (0x0000001FU)
#define CSL_CSI2_CTX0_CTRL1_BYTESWAP_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX0_CTRL1_BYTESWAP_MAX                             (0x00000001U)

#define CSL_CSI2_CTX0_CTRL1_RESETVAL                                 (0x00010008U)

/* CSI2_CTX0_CTRL2 */

#define CSL_CSI2_CTX0_CTRL2_FORMAT_MASK                              (0x000003FFU)
#define CSL_CSI2_CTX0_CTRL2_FORMAT_SHIFT                             (0x00000000U)
#define CSL_CSI2_CTX0_CTRL2_FORMAT_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX0_CTRL2_FORMAT_MAX                               (0x000003FFU)

#define CSL_CSI2_CTX0_CTRL2_DPCM_PRED_MASK                           (0x00000400U)
#define CSL_CSI2_CTX0_CTRL2_DPCM_PRED_SHIFT                          (0x0000000AU)
#define CSL_CSI2_CTX0_CTRL2_DPCM_PRED_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX0_CTRL2_DPCM_PRED_MAX                            (0x00000001U)

#define CSL_CSI2_CTX0_CTRL2_VIRTUAL_ID_MASK                          (0x00001800U)
#define CSL_CSI2_CTX0_CTRL2_VIRTUAL_ID_SHIFT                         (0x0000000BU)
#define CSL_CSI2_CTX0_CTRL2_VIRTUAL_ID_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX0_CTRL2_VIRTUAL_ID_MAX                           (0x00000003U)

#define CSL_CSI2_CTX0_CTRL2_USER_DEF_MAPPING_MASK                    (0x00006000U)
#define CSL_CSI2_CTX0_CTRL2_USER_DEF_MAPPING_SHIFT                   (0x0000000DU)
#define CSL_CSI2_CTX0_CTRL2_USER_DEF_MAPPING_RESETVAL                (0x00000000U)
#define CSL_CSI2_CTX0_CTRL2_USER_DEF_MAPPING_MAX                     (0x00000003U)

#define CSL_CSI2_CTX0_CTRL2_RES20_MASK                               (0x00008000U)
#define CSL_CSI2_CTX0_CTRL2_RES20_SHIFT                              (0x0000000FU)
#define CSL_CSI2_CTX0_CTRL2_RES20_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX0_CTRL2_RES20_MAX                                (0x00000001U)

#define CSL_CSI2_CTX0_CTRL2_FRAME_MASK                               (0xFFFF0000U)
#define CSL_CSI2_CTX0_CTRL2_FRAME_SHIFT                              (0x00000010U)
#define CSL_CSI2_CTX0_CTRL2_FRAME_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX0_CTRL2_FRAME_MAX                                (0x0000FFFFU)

#define CSL_CSI2_CTX0_CTRL2_RESETVAL                                 (0x00000000U)

/* CSI2_CTX0_DAT_OFST */

#define CSL_CSI2_CTX0_DAT_OFST_RES_MASK                              (0x0000001FU)
#define CSL_CSI2_CTX0_DAT_OFST_RES_SHIFT                             (0x00000000U)
#define CSL_CSI2_CTX0_DAT_OFST_RES_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX0_DAT_OFST_RES_MAX                               (0x0000001FU)

#define CSL_CSI2_CTX0_DAT_OFST_OFST_MASK                             (0x0001FFE0U)
#define CSL_CSI2_CTX0_DAT_OFST_OFST_SHIFT                            (0x00000005U)
#define CSL_CSI2_CTX0_DAT_OFST_OFST_RESETVAL                         (0x00000000U)
#define CSL_CSI2_CTX0_DAT_OFST_OFST_MAX                              (0x00000FFFU)

#define CSL_CSI2_CTX0_DAT_OFST_RES21_MASK                            (0xFFFE0000U)
#define CSL_CSI2_CTX0_DAT_OFST_RES21_SHIFT                           (0x00000011U)
#define CSL_CSI2_CTX0_DAT_OFST_RES21_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX0_DAT_OFST_RES21_MAX                             (0x00007FFFU)

#define CSL_CSI2_CTX0_DAT_OFST_RESETVAL                              (0x00000000U)

/* CSI2_CTX0_DAT_PING_ADDR */

#define CSL_CSI2_CTX0_DAT_PING_ADDR_RES_MASK                         (0x0000001FU)
#define CSL_CSI2_CTX0_DAT_PING_ADDR_RES_SHIFT                        (0x00000000U)
#define CSL_CSI2_CTX0_DAT_PING_ADDR_RES_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX0_DAT_PING_ADDR_RES_MAX                          (0x0000001FU)

#define CSL_CSI2_CTX0_DAT_PING_ADDR_ADDR_MASK                        (0xFFFFFFE0U)
#define CSL_CSI2_CTX0_DAT_PING_ADDR_ADDR_SHIFT                       (0x00000005U)
#define CSL_CSI2_CTX0_DAT_PING_ADDR_ADDR_RESETVAL                    (0x00000000U)
#define CSL_CSI2_CTX0_DAT_PING_ADDR_ADDR_MAX                         (0x07FFFFFFU)

#define CSL_CSI2_CTX0_DAT_PING_ADDR_RESETVAL                         (0x00000000U)

/* CSI2_CTX0_DAT_PONG_ADDR */

#define CSL_CSI2_CTX0_DAT_PONG_ADDR_RES_MASK                         (0x0000001FU)
#define CSL_CSI2_CTX0_DAT_PONG_ADDR_RES_SHIFT                        (0x00000000U)
#define CSL_CSI2_CTX0_DAT_PONG_ADDR_RES_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX0_DAT_PONG_ADDR_RES_MAX                          (0x0000001FU)

#define CSL_CSI2_CTX0_DAT_PONG_ADDR_ADDR_MASK                        (0xFFFFFFE0U)
#define CSL_CSI2_CTX0_DAT_PONG_ADDR_ADDR_SHIFT                       (0x00000005U)
#define CSL_CSI2_CTX0_DAT_PONG_ADDR_ADDR_RESETVAL                    (0x00000000U)
#define CSL_CSI2_CTX0_DAT_PONG_ADDR_ADDR_MAX                         (0x07FFFFFFU)

#define CSL_CSI2_CTX0_DAT_PONG_ADDR_RESETVAL                         (0x00000000U)

/* CSI2_CTX0_IRQENABLE */

#define CSL_CSI2_CTX0_IRQENABLE_FS_IRQ_MASK                          (0x00000001U)
#define CSL_CSI2_CTX0_IRQENABLE_FS_IRQ_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX0_IRQENABLE_FS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX0_IRQENABLE_FS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX0_IRQENABLE_FE_IRQ_MASK                          (0x00000002U)
#define CSL_CSI2_CTX0_IRQENABLE_FE_IRQ_SHIFT                         (0x00000001U)
#define CSL_CSI2_CTX0_IRQENABLE_FE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX0_IRQENABLE_FE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX0_IRQENABLE_LS_IRQ_MASK                          (0x00000004U)
#define CSL_CSI2_CTX0_IRQENABLE_LS_IRQ_SHIFT                         (0x00000002U)
#define CSL_CSI2_CTX0_IRQENABLE_LS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX0_IRQENABLE_LS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX0_IRQENABLE_LE_IRQ_MASK                          (0x00000008U)
#define CSL_CSI2_CTX0_IRQENABLE_LE_IRQ_SHIFT                         (0x00000003U)
#define CSL_CSI2_CTX0_IRQENABLE_LE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX0_IRQENABLE_LE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX0_IRQENABLE_RES23_MASK                           (0x00000010U)
#define CSL_CSI2_CTX0_IRQENABLE_RES23_SHIFT                          (0x00000004U)
#define CSL_CSI2_CTX0_IRQENABLE_RES23_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX0_IRQENABLE_RES23_MAX                            (0x00000001U)

#define CSL_CSI2_CTX0_IRQENABLE_CS_IRQ_MASK                          (0x00000020U)
#define CSL_CSI2_CTX0_IRQENABLE_CS_IRQ_SHIFT                         (0x00000005U)
#define CSL_CSI2_CTX0_IRQENABLE_CS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX0_IRQENABLE_CS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX0_IRQENABLE_FRAME_NUMBER_IRQ_MASK                (0x00000040U)
#define CSL_CSI2_CTX0_IRQENABLE_FRAME_NUMBER_IRQ_SHIFT               (0x00000006U)
#define CSL_CSI2_CTX0_IRQENABLE_FRAME_NUMBER_IRQ_RESETVAL            (0x00000000U)
#define CSL_CSI2_CTX0_IRQENABLE_FRAME_NUMBER_IRQ_MAX                 (0x00000001U)

#define CSL_CSI2_CTX0_IRQENABLE_LINE_NUMBER_IRQ_MASK                 (0x00000080U)
#define CSL_CSI2_CTX0_IRQENABLE_LINE_NUMBER_IRQ_SHIFT                (0x00000007U)
#define CSL_CSI2_CTX0_IRQENABLE_LINE_NUMBER_IRQ_RESETVAL             (0x00000000U)
#define CSL_CSI2_CTX0_IRQENABLE_LINE_NUMBER_IRQ_MAX                  (0x00000001U)

#define CSL_CSI2_CTX0_IRQENABLE_ECC_CORRECTION_IRQ_MASK              (0x00000100U)
#define CSL_CSI2_CTX0_IRQENABLE_ECC_CORRECTION_IRQ_SHIFT             (0x00000008U)
#define CSL_CSI2_CTX0_IRQENABLE_ECC_CORRECTION_IRQ_RESETVAL          (0x00000000U)
#define CSL_CSI2_CTX0_IRQENABLE_ECC_CORRECTION_IRQ_MAX               (0x00000001U)

#define CSL_CSI2_CTX0_IRQENABLE_RES22_MASK                           (0xFFFFFE00U)
#define CSL_CSI2_CTX0_IRQENABLE_RES22_SHIFT                          (0x00000009U)
#define CSL_CSI2_CTX0_IRQENABLE_RES22_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX0_IRQENABLE_RES22_MAX                            (0x007FFFFFU)

#define CSL_CSI2_CTX0_IRQENABLE_RESETVAL                             (0x00000000U)

/* CSI2_CTX0_IRQSTATUS */

#define CSL_CSI2_CTX0_IRQSTATUS_FS_IRQ_MASK                          (0x00000001U)
#define CSL_CSI2_CTX0_IRQSTATUS_FS_IRQ_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX0_IRQSTATUS_FS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX0_IRQSTATUS_FS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX0_IRQSTATUS_FE_IRQ_MASK                          (0x00000002U)
#define CSL_CSI2_CTX0_IRQSTATUS_FE_IRQ_SHIFT                         (0x00000001U)
#define CSL_CSI2_CTX0_IRQSTATUS_FE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX0_IRQSTATUS_FE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX0_IRQSTATUS_LS_IRQ_MASK                          (0x00000004U)
#define CSL_CSI2_CTX0_IRQSTATUS_LS_IRQ_SHIFT                         (0x00000002U)
#define CSL_CSI2_CTX0_IRQSTATUS_LS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX0_IRQSTATUS_LS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX0_IRQSTATUS_LE_IRQ_MASK                          (0x00000008U)
#define CSL_CSI2_CTX0_IRQSTATUS_LE_IRQ_SHIFT                         (0x00000003U)
#define CSL_CSI2_CTX0_IRQSTATUS_LE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX0_IRQSTATUS_LE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX0_IRQSTATUS_RES25_MASK                           (0x00000010U)
#define CSL_CSI2_CTX0_IRQSTATUS_RES25_SHIFT                          (0x00000004U)
#define CSL_CSI2_CTX0_IRQSTATUS_RES25_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX0_IRQSTATUS_RES25_MAX                            (0x00000001U)

#define CSL_CSI2_CTX0_IRQSTATUS_CS_IRQ_MASK                          (0x00000020U)
#define CSL_CSI2_CTX0_IRQSTATUS_CS_IRQ_SHIFT                         (0x00000005U)
#define CSL_CSI2_CTX0_IRQSTATUS_CS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX0_IRQSTATUS_CS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX0_IRQSTATUS_FRAME_NUMBER_IRQ_MASK                (0x00000040U)
#define CSL_CSI2_CTX0_IRQSTATUS_FRAME_NUMBER_IRQ_SHIFT               (0x00000006U)
#define CSL_CSI2_CTX0_IRQSTATUS_FRAME_NUMBER_IRQ_RESETVAL            (0x00000000U)
#define CSL_CSI2_CTX0_IRQSTATUS_FRAME_NUMBER_IRQ_MAX                 (0x00000001U)

#define CSL_CSI2_CTX0_IRQSTATUS_LINE_NUMBER_IRQ_MASK                 (0x00000080U)
#define CSL_CSI2_CTX0_IRQSTATUS_LINE_NUMBER_IRQ_SHIFT                (0x00000007U)
#define CSL_CSI2_CTX0_IRQSTATUS_LINE_NUMBER_IRQ_RESETVAL             (0x00000000U)
#define CSL_CSI2_CTX0_IRQSTATUS_LINE_NUMBER_IRQ_MAX                  (0x00000001U)

#define CSL_CSI2_CTX0_IRQSTATUS_ECC_CORRECTION_IRQ_MASK              (0x00000100U)
#define CSL_CSI2_CTX0_IRQSTATUS_ECC_CORRECTION_IRQ_SHIFT             (0x00000008U)
#define CSL_CSI2_CTX0_IRQSTATUS_ECC_CORRECTION_IRQ_RESETVAL          (0x00000000U)
#define CSL_CSI2_CTX0_IRQSTATUS_ECC_CORRECTION_IRQ_MAX               (0x00000001U)

#define CSL_CSI2_CTX0_IRQSTATUS_RES24_MASK                           (0xFFFFFE00U)
#define CSL_CSI2_CTX0_IRQSTATUS_RES24_SHIFT                          (0x00000009U)
#define CSL_CSI2_CTX0_IRQSTATUS_RES24_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX0_IRQSTATUS_RES24_MAX                            (0x007FFFFFU)

#define CSL_CSI2_CTX0_IRQSTATUS_RESETVAL                             (0x00000000U)

/* CSI2_CTX0_CTRL3 */

#define CSL_CSI2_CTX0_CTRL3_LINE_NUMBER_MASK                         (0x0000FFFFU)
#define CSL_CSI2_CTX0_CTRL3_LINE_NUMBER_SHIFT                        (0x00000000U)
#define CSL_CSI2_CTX0_CTRL3_LINE_NUMBER_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX0_CTRL3_LINE_NUMBER_MAX                          (0x0000FFFFU)

#define CSL_CSI2_CTX0_CTRL3_ALPHA_MASK                               (0x3FFF0000U)
#define CSL_CSI2_CTX0_CTRL3_ALPHA_SHIFT                              (0x00000010U)
#define CSL_CSI2_CTX0_CTRL3_ALPHA_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX0_CTRL3_ALPHA_MAX                                (0x00003FFFU)

#define CSL_CSI2_CTX0_CTRL3_RESERVED_MASK                            (0xC0000000U)
#define CSL_CSI2_CTX0_CTRL3_RESERVED_SHIFT                           (0x0000001EU)
#define CSL_CSI2_CTX0_CTRL3_RESERVED_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX0_CTRL3_RESERVED_MAX                             (0x00000003U)

#define CSL_CSI2_CTX0_CTRL3_RESETVAL                                 (0x00000000U)

/* CSI2_CTX1_CTRL1 */

#define CSL_CSI2_CTX1_CTRL1_CTX_EN_MASK                              (0x00000001U)
#define CSL_CSI2_CTX1_CTRL1_CTX_EN_SHIFT                             (0x00000000U)
#define CSL_CSI2_CTX1_CTRL1_CTX_EN_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX1_CTRL1_CTX_EN_MAX                               (0x00000001U)

#define CSL_CSI2_CTX1_CTRL1_LINE_MODULO_MASK                         (0x00000002U)
#define CSL_CSI2_CTX1_CTRL1_LINE_MODULO_SHIFT                        (0x00000001U)
#define CSL_CSI2_CTX1_CTRL1_LINE_MODULO_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX1_CTRL1_LINE_MODULO_MAX                          (0x00000001U)

#define CSL_CSI2_CTX1_CTRL1_VP_FORCE_MASK                            (0x00000004U)
#define CSL_CSI2_CTX1_CTRL1_VP_FORCE_SHIFT                           (0x00000002U)
#define CSL_CSI2_CTX1_CTRL1_VP_FORCE_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX1_CTRL1_VP_FORCE_MAX                             (0x00000001U)

#define CSL_CSI2_CTX1_CTRL1_PING_PONG_MASK                           (0x00000008U)
#define CSL_CSI2_CTX1_CTRL1_PING_PONG_SHIFT                          (0x00000003U)
#define CSL_CSI2_CTX1_CTRL1_PING_PONG_RESETVAL                       (0x00000001U)
#define CSL_CSI2_CTX1_CTRL1_PING_PONG_MAX                            (0x00000001U)

#define CSL_CSI2_CTX1_CTRL1_COUNT_UNLOCK_MASK                        (0x00000010U)
#define CSL_CSI2_CTX1_CTRL1_COUNT_UNLOCK_SHIFT                       (0x00000004U)
#define CSL_CSI2_CTX1_CTRL1_COUNT_UNLOCK_RESETVAL                    (0x00000000U)
#define CSL_CSI2_CTX1_CTRL1_COUNT_UNLOCK_MAX                         (0x00000001U)

#define CSL_CSI2_CTX1_CTRL1_CS_EN_MASK                               (0x00000020U)
#define CSL_CSI2_CTX1_CTRL1_CS_EN_SHIFT                              (0x00000005U)
#define CSL_CSI2_CTX1_CTRL1_CS_EN_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX1_CTRL1_CS_EN_MAX                                (0x00000001U)

#define CSL_CSI2_CTX1_CTRL1_EOL_EN_MASK                              (0x00000040U)
#define CSL_CSI2_CTX1_CTRL1_EOL_EN_SHIFT                             (0x00000006U)
#define CSL_CSI2_CTX1_CTRL1_EOL_EN_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX1_CTRL1_EOL_EN_MAX                               (0x00000001U)

#define CSL_CSI2_CTX1_CTRL1_EOF_EN_MASK                              (0x00000080U)
#define CSL_CSI2_CTX1_CTRL1_EOF_EN_SHIFT                             (0x00000007U)
#define CSL_CSI2_CTX1_CTRL1_EOF_EN_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX1_CTRL1_EOF_EN_MAX                               (0x00000001U)

#define CSL_CSI2_CTX1_CTRL1_COUNT_MASK                               (0x0000FF00U)
#define CSL_CSI2_CTX1_CTRL1_COUNT_SHIFT                              (0x00000008U)
#define CSL_CSI2_CTX1_CTRL1_COUNT_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX1_CTRL1_COUNT_MAX                                (0x000000FFU)

#define CSL_CSI2_CTX1_CTRL1_FEC_NUMBER_MASK                          (0x00FF0000U)
#define CSL_CSI2_CTX1_CTRL1_FEC_NUMBER_SHIFT                         (0x00000010U)
#define CSL_CSI2_CTX1_CTRL1_FEC_NUMBER_RESETVAL                      (0x00000001U)
#define CSL_CSI2_CTX1_CTRL1_FEC_NUMBER_MAX                           (0x000000FFU)

#define CSL_CSI2_CTX1_CTRL1_TRANSCODE_MASK                           (0x0F000000U)
#define CSL_CSI2_CTX1_CTRL1_TRANSCODE_SHIFT                          (0x00000018U)
#define CSL_CSI2_CTX1_CTRL1_TRANSCODE_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX1_CTRL1_TRANSCODE_MAX                            (0x0000000FU)

#define CSL_CSI2_CTX1_CTRL1_HSCALE_MASK                              (0x10000000U)
#define CSL_CSI2_CTX1_CTRL1_HSCALE_SHIFT                             (0x0000001CU)
#define CSL_CSI2_CTX1_CTRL1_HSCALE_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX1_CTRL1_HSCALE_MAX                               (0x00000001U)

#define CSL_CSI2_CTX1_CTRL1_RES19_MASK                               (0x20000000U)
#define CSL_CSI2_CTX1_CTRL1_RES19_SHIFT                              (0x0000001DU)
#define CSL_CSI2_CTX1_CTRL1_RES19_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX1_CTRL1_RES19_MAX                                (0x00000001U)

#define CSL_CSI2_CTX1_CTRL1_GENERIC_MASK                             (0x40000000U)
#define CSL_CSI2_CTX1_CTRL1_GENERIC_SHIFT                            (0x0000001EU)
#define CSL_CSI2_CTX1_CTRL1_GENERIC_RESETVAL                         (0x00000000U)
#define CSL_CSI2_CTX1_CTRL1_GENERIC_MAX                              (0x00000001U)

#define CSL_CSI2_CTX1_CTRL1_BYTESWAP_MASK                            (0x80000000U)
#define CSL_CSI2_CTX1_CTRL1_BYTESWAP_SHIFT                           (0x0000001FU)
#define CSL_CSI2_CTX1_CTRL1_BYTESWAP_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX1_CTRL1_BYTESWAP_MAX                             (0x00000001U)

#define CSL_CSI2_CTX1_CTRL1_RESETVAL                                 (0x00010008U)

/* CSI2_CTX1_CTRL2 */

#define CSL_CSI2_CTX1_CTRL2_FORMAT_MASK                              (0x000003FFU)
#define CSL_CSI2_CTX1_CTRL2_FORMAT_SHIFT                             (0x00000000U)
#define CSL_CSI2_CTX1_CTRL2_FORMAT_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX1_CTRL2_FORMAT_MAX                               (0x000003FFU)

#define CSL_CSI2_CTX1_CTRL2_DPCM_PRED_MASK                           (0x00000400U)
#define CSL_CSI2_CTX1_CTRL2_DPCM_PRED_SHIFT                          (0x0000000AU)
#define CSL_CSI2_CTX1_CTRL2_DPCM_PRED_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX1_CTRL2_DPCM_PRED_MAX                            (0x00000001U)

#define CSL_CSI2_CTX1_CTRL2_VIRTUAL_ID_MASK                          (0x00001800U)
#define CSL_CSI2_CTX1_CTRL2_VIRTUAL_ID_SHIFT                         (0x0000000BU)
#define CSL_CSI2_CTX1_CTRL2_VIRTUAL_ID_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX1_CTRL2_VIRTUAL_ID_MAX                           (0x00000003U)

#define CSL_CSI2_CTX1_CTRL2_USER_DEF_MAPPING_MASK                    (0x00006000U)
#define CSL_CSI2_CTX1_CTRL2_USER_DEF_MAPPING_SHIFT                   (0x0000000DU)
#define CSL_CSI2_CTX1_CTRL2_USER_DEF_MAPPING_RESETVAL                (0x00000000U)
#define CSL_CSI2_CTX1_CTRL2_USER_DEF_MAPPING_MAX                     (0x00000003U)

#define CSL_CSI2_CTX1_CTRL2_RES20_MASK                               (0x00008000U)
#define CSL_CSI2_CTX1_CTRL2_RES20_SHIFT                              (0x0000000FU)
#define CSL_CSI2_CTX1_CTRL2_RES20_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX1_CTRL2_RES20_MAX                                (0x00000001U)

#define CSL_CSI2_CTX1_CTRL2_FRAME_MASK                               (0xFFFF0000U)
#define CSL_CSI2_CTX1_CTRL2_FRAME_SHIFT                              (0x00000010U)
#define CSL_CSI2_CTX1_CTRL2_FRAME_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX1_CTRL2_FRAME_MAX                                (0x0000FFFFU)

#define CSL_CSI2_CTX1_CTRL2_RESETVAL                                 (0x00000000U)

/* CSI2_CTX1_DAT_OFST */

#define CSL_CSI2_CTX1_DAT_OFST_OFST_MASK                             (0x0001FFE0U)
#define CSL_CSI2_CTX1_DAT_OFST_OFST_SHIFT                            (0x00000005U)
#define CSL_CSI2_CTX1_DAT_OFST_OFST_RESETVAL                         (0x00000000U)
#define CSL_CSI2_CTX1_DAT_OFST_OFST_MAX                              (0x00000FFFU)

#define CSL_CSI2_CTX1_DAT_OFST_RES21_MASK                            (0xFFFE0000U)
#define CSL_CSI2_CTX1_DAT_OFST_RES21_SHIFT                           (0x00000011U)
#define CSL_CSI2_CTX1_DAT_OFST_RES21_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX1_DAT_OFST_RES21_MAX                             (0x00007FFFU)

#define CSL_CSI2_CTX1_DAT_OFST_RESETVAL                              (0x00000000U)

/* CSI2_CTX1_DAT_PING_ADDR */

#define CSL_CSI2_CTX1_DAT_PING_ADDR_RES_MASK                         (0x0000001FU)
#define CSL_CSI2_CTX1_DAT_PING_ADDR_RES_SHIFT                        (0x00000000U)
#define CSL_CSI2_CTX1_DAT_PING_ADDR_RES_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX1_DAT_PING_ADDR_RES_MAX                          (0x0000001FU)

#define CSL_CSI2_CTX1_DAT_PING_ADDR_ADDR_MASK                        (0xFFFFFFE0U)
#define CSL_CSI2_CTX1_DAT_PING_ADDR_ADDR_SHIFT                       (0x00000005U)
#define CSL_CSI2_CTX1_DAT_PING_ADDR_ADDR_RESETVAL                    (0x00000000U)
#define CSL_CSI2_CTX1_DAT_PING_ADDR_ADDR_MAX                         (0x07FFFFFFU)

#define CSL_CSI2_CTX1_DAT_PING_ADDR_RESETVAL                         (0x00000000U)

/* CSI2_CTX1_DAT_PONG_ADDR */

#define CSL_CSI2_CTX1_DAT_PONG_ADDR_RES_MASK                         (0x0000001FU)
#define CSL_CSI2_CTX1_DAT_PONG_ADDR_RES_SHIFT                        (0x00000000U)
#define CSL_CSI2_CTX1_DAT_PONG_ADDR_RES_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX1_DAT_PONG_ADDR_RES_MAX                          (0x0000001FU)

#define CSL_CSI2_CTX1_DAT_PONG_ADDR_ADDR_MASK                        (0xFFFFFFE0U)
#define CSL_CSI2_CTX1_DAT_PONG_ADDR_ADDR_SHIFT                       (0x00000005U)
#define CSL_CSI2_CTX1_DAT_PONG_ADDR_ADDR_RESETVAL                    (0x00000000U)
#define CSL_CSI2_CTX1_DAT_PONG_ADDR_ADDR_MAX                         (0x07FFFFFFU)

#define CSL_CSI2_CTX1_DAT_PONG_ADDR_RESETVAL                         (0x00000000U)

/* CSI2_CTX1_IRQENABLE */

#define CSL_CSI2_CTX1_IRQENABLE_FS_IRQ_MASK                          (0x00000001U)
#define CSL_CSI2_CTX1_IRQENABLE_FS_IRQ_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX1_IRQENABLE_FS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX1_IRQENABLE_FS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX1_IRQENABLE_FE_IRQ_MASK                          (0x00000002U)
#define CSL_CSI2_CTX1_IRQENABLE_FE_IRQ_SHIFT                         (0x00000001U)
#define CSL_CSI2_CTX1_IRQENABLE_FE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX1_IRQENABLE_FE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX1_IRQENABLE_LS_IRQ_MASK                          (0x00000004U)
#define CSL_CSI2_CTX1_IRQENABLE_LS_IRQ_SHIFT                         (0x00000002U)
#define CSL_CSI2_CTX1_IRQENABLE_LS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX1_IRQENABLE_LS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX1_IRQENABLE_LE_IRQ_MASK                          (0x00000008U)
#define CSL_CSI2_CTX1_IRQENABLE_LE_IRQ_SHIFT                         (0x00000003U)
#define CSL_CSI2_CTX1_IRQENABLE_LE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX1_IRQENABLE_LE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX1_IRQENABLE_RES23_MASK                           (0x00000010U)
#define CSL_CSI2_CTX1_IRQENABLE_RES23_SHIFT                          (0x00000004U)
#define CSL_CSI2_CTX1_IRQENABLE_RES23_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX1_IRQENABLE_RES23_MAX                            (0x00000001U)

#define CSL_CSI2_CTX1_IRQENABLE_CS_IRQ_MASK                          (0x00000020U)
#define CSL_CSI2_CTX1_IRQENABLE_CS_IRQ_SHIFT                         (0x00000005U)
#define CSL_CSI2_CTX1_IRQENABLE_CS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX1_IRQENABLE_CS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX1_IRQENABLE_FRAME_NUMBER_IRQ_MASK                (0x00000040U)
#define CSL_CSI2_CTX1_IRQENABLE_FRAME_NUMBER_IRQ_SHIFT               (0x00000006U)
#define CSL_CSI2_CTX1_IRQENABLE_FRAME_NUMBER_IRQ_RESETVAL            (0x00000000U)
#define CSL_CSI2_CTX1_IRQENABLE_FRAME_NUMBER_IRQ_MAX                 (0x00000001U)

#define CSL_CSI2_CTX1_IRQENABLE_LINE_NUMBER_IRQ_MASK                 (0x00000080U)
#define CSL_CSI2_CTX1_IRQENABLE_LINE_NUMBER_IRQ_SHIFT                (0x00000007U)
#define CSL_CSI2_CTX1_IRQENABLE_LINE_NUMBER_IRQ_RESETVAL             (0x00000000U)
#define CSL_CSI2_CTX1_IRQENABLE_LINE_NUMBER_IRQ_MAX                  (0x00000001U)

#define CSL_CSI2_CTX1_IRQENABLE_ECC_CORRECTION_IRQ_MASK              (0x00000100U)
#define CSL_CSI2_CTX1_IRQENABLE_ECC_CORRECTION_IRQ_SHIFT             (0x00000008U)
#define CSL_CSI2_CTX1_IRQENABLE_ECC_CORRECTION_IRQ_RESETVAL          (0x00000000U)
#define CSL_CSI2_CTX1_IRQENABLE_ECC_CORRECTION_IRQ_MAX               (0x00000001U)

#define CSL_CSI2_CTX1_IRQENABLE_RES22_MASK                           (0xFFFFFE00U)
#define CSL_CSI2_CTX1_IRQENABLE_RES22_SHIFT                          (0x00000009U)
#define CSL_CSI2_CTX1_IRQENABLE_RES22_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX1_IRQENABLE_RES22_MAX                            (0x007FFFFFU)

#define CSL_CSI2_CTX1_IRQENABLE_RESETVAL                             (0x00000000U)

/* CSI2_CTX1_IRQSTATUS */

#define CSL_CSI2_CTX1_IRQSTATUS_FS_IRQ_MASK                          (0x00000001U)
#define CSL_CSI2_CTX1_IRQSTATUS_FS_IRQ_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX1_IRQSTATUS_FS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX1_IRQSTATUS_FS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX1_IRQSTATUS_FE_IRQ_MASK                          (0x00000002U)
#define CSL_CSI2_CTX1_IRQSTATUS_FE_IRQ_SHIFT                         (0x00000001U)
#define CSL_CSI2_CTX1_IRQSTATUS_FE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX1_IRQSTATUS_FE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX1_IRQSTATUS_LS_IRQ_MASK                          (0x00000004U)
#define CSL_CSI2_CTX1_IRQSTATUS_LS_IRQ_SHIFT                         (0x00000002U)
#define CSL_CSI2_CTX1_IRQSTATUS_LS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX1_IRQSTATUS_LS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX1_IRQSTATUS_LE_IRQ_MASK                          (0x00000008U)
#define CSL_CSI2_CTX1_IRQSTATUS_LE_IRQ_SHIFT                         (0x00000003U)
#define CSL_CSI2_CTX1_IRQSTATUS_LE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX1_IRQSTATUS_LE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX1_IRQSTATUS_RES25_MASK                           (0x00000010U)
#define CSL_CSI2_CTX1_IRQSTATUS_RES25_SHIFT                          (0x00000004U)
#define CSL_CSI2_CTX1_IRQSTATUS_RES25_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX1_IRQSTATUS_RES25_MAX                            (0x00000001U)

#define CSL_CSI2_CTX1_IRQSTATUS_CS_IRQ_MASK                          (0x00000020U)
#define CSL_CSI2_CTX1_IRQSTATUS_CS_IRQ_SHIFT                         (0x00000005U)
#define CSL_CSI2_CTX1_IRQSTATUS_CS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX1_IRQSTATUS_CS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX1_IRQSTATUS_FRAME_NUMBER_IRQ_MASK                (0x00000040U)
#define CSL_CSI2_CTX1_IRQSTATUS_FRAME_NUMBER_IRQ_SHIFT               (0x00000006U)
#define CSL_CSI2_CTX1_IRQSTATUS_FRAME_NUMBER_IRQ_RESETVAL            (0x00000000U)
#define CSL_CSI2_CTX1_IRQSTATUS_FRAME_NUMBER_IRQ_MAX                 (0x00000001U)

#define CSL_CSI2_CTX1_IRQSTATUS_LINE_NUMBER_IRQ_MASK                 (0x00000080U)
#define CSL_CSI2_CTX1_IRQSTATUS_LINE_NUMBER_IRQ_SHIFT                (0x00000007U)
#define CSL_CSI2_CTX1_IRQSTATUS_LINE_NUMBER_IRQ_RESETVAL             (0x00000000U)
#define CSL_CSI2_CTX1_IRQSTATUS_LINE_NUMBER_IRQ_MAX                  (0x00000001U)

#define CSL_CSI2_CTX1_IRQSTATUS_ECC_CORRECTION_IRQ_MASK              (0x00000100U)
#define CSL_CSI2_CTX1_IRQSTATUS_ECC_CORRECTION_IRQ_SHIFT             (0x00000008U)
#define CSL_CSI2_CTX1_IRQSTATUS_ECC_CORRECTION_IRQ_RESETVAL          (0x00000000U)
#define CSL_CSI2_CTX1_IRQSTATUS_ECC_CORRECTION_IRQ_MAX               (0x00000001U)

#define CSL_CSI2_CTX1_IRQSTATUS_RES24_MASK                           (0xFFFFFE00U)
#define CSL_CSI2_CTX1_IRQSTATUS_RES24_SHIFT                          (0x00000009U)
#define CSL_CSI2_CTX1_IRQSTATUS_RES24_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX1_IRQSTATUS_RES24_MAX                            (0x007FFFFFU)

#define CSL_CSI2_CTX1_IRQSTATUS_RESETVAL                             (0x00000000U)

/* CSI2_CTX1_CTRL3 */

#define CSL_CSI2_CTX1_CTRL3_LINE_NUMBER_MASK                         (0x0000FFFFU)
#define CSL_CSI2_CTX1_CTRL3_LINE_NUMBER_SHIFT                        (0x00000000U)
#define CSL_CSI2_CTX1_CTRL3_LINE_NUMBER_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX1_CTRL3_LINE_NUMBER_MAX                          (0x0000FFFFU)

#define CSL_CSI2_CTX1_CTRL3_ALPHA_MASK                               (0x3FFF0000U)
#define CSL_CSI2_CTX1_CTRL3_ALPHA_SHIFT                              (0x00000010U)
#define CSL_CSI2_CTX1_CTRL3_ALPHA_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX1_CTRL3_ALPHA_MAX                                (0x00003FFFU)

#define CSL_CSI2_CTX1_CTRL3_RESERVED_MASK                            (0xC0000000U)
#define CSL_CSI2_CTX1_CTRL3_RESERVED_SHIFT                           (0x0000001EU)
#define CSL_CSI2_CTX1_CTRL3_RESERVED_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX1_CTRL3_RESERVED_MAX                             (0x00000003U)

#define CSL_CSI2_CTX1_CTRL3_RESETVAL                                 (0x00000000U)

/* CSI2_CTX2_CTRL1 */

#define CSL_CSI2_CTX2_CTRL1_CTX_EN_MASK                              (0x00000001U)
#define CSL_CSI2_CTX2_CTRL1_CTX_EN_SHIFT                             (0x00000000U)
#define CSL_CSI2_CTX2_CTRL1_CTX_EN_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX2_CTRL1_CTX_EN_MAX                               (0x00000001U)

#define CSL_CSI2_CTX2_CTRL1_LINE_MODULO_MASK                         (0x00000002U)
#define CSL_CSI2_CTX2_CTRL1_LINE_MODULO_SHIFT                        (0x00000001U)
#define CSL_CSI2_CTX2_CTRL1_LINE_MODULO_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX2_CTRL1_LINE_MODULO_MAX                          (0x00000001U)

#define CSL_CSI2_CTX2_CTRL1_VP_FORCE_MASK                            (0x00000004U)
#define CSL_CSI2_CTX2_CTRL1_VP_FORCE_SHIFT                           (0x00000002U)
#define CSL_CSI2_CTX2_CTRL1_VP_FORCE_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX2_CTRL1_VP_FORCE_MAX                             (0x00000001U)

#define CSL_CSI2_CTX2_CTRL1_PING_PONG_MASK                           (0x00000008U)
#define CSL_CSI2_CTX2_CTRL1_PING_PONG_SHIFT                          (0x00000003U)
#define CSL_CSI2_CTX2_CTRL1_PING_PONG_RESETVAL                       (0x00000001U)
#define CSL_CSI2_CTX2_CTRL1_PING_PONG_MAX                            (0x00000001U)

#define CSL_CSI2_CTX2_CTRL1_COUNT_UNLOCK_MASK                        (0x00000010U)
#define CSL_CSI2_CTX2_CTRL1_COUNT_UNLOCK_SHIFT                       (0x00000004U)
#define CSL_CSI2_CTX2_CTRL1_COUNT_UNLOCK_RESETVAL                    (0x00000000U)
#define CSL_CSI2_CTX2_CTRL1_COUNT_UNLOCK_MAX                         (0x00000001U)

#define CSL_CSI2_CTX2_CTRL1_CS_EN_MASK                               (0x00000020U)
#define CSL_CSI2_CTX2_CTRL1_CS_EN_SHIFT                              (0x00000005U)
#define CSL_CSI2_CTX2_CTRL1_CS_EN_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX2_CTRL1_CS_EN_MAX                                (0x00000001U)

#define CSL_CSI2_CTX2_CTRL1_EOL_EN_MASK                              (0x00000040U)
#define CSL_CSI2_CTX2_CTRL1_EOL_EN_SHIFT                             (0x00000006U)
#define CSL_CSI2_CTX2_CTRL1_EOL_EN_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX2_CTRL1_EOL_EN_MAX                               (0x00000001U)

#define CSL_CSI2_CTX2_CTRL1_EOF_EN_MASK                              (0x00000080U)
#define CSL_CSI2_CTX2_CTRL1_EOF_EN_SHIFT                             (0x00000007U)
#define CSL_CSI2_CTX2_CTRL1_EOF_EN_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX2_CTRL1_EOF_EN_MAX                               (0x00000001U)

#define CSL_CSI2_CTX2_CTRL1_COUNT_MASK                               (0x0000FF00U)
#define CSL_CSI2_CTX2_CTRL1_COUNT_SHIFT                              (0x00000008U)
#define CSL_CSI2_CTX2_CTRL1_COUNT_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX2_CTRL1_COUNT_MAX                                (0x000000FFU)

#define CSL_CSI2_CTX2_CTRL1_FEC_NUMBER_MASK                          (0x00FF0000U)
#define CSL_CSI2_CTX2_CTRL1_FEC_NUMBER_SHIFT                         (0x00000010U)
#define CSL_CSI2_CTX2_CTRL1_FEC_NUMBER_RESETVAL                      (0x00000001U)
#define CSL_CSI2_CTX2_CTRL1_FEC_NUMBER_MAX                           (0x000000FFU)

#define CSL_CSI2_CTX2_CTRL1_TRANSCODE_MASK                           (0x0F000000U)
#define CSL_CSI2_CTX2_CTRL1_TRANSCODE_SHIFT                          (0x00000018U)
#define CSL_CSI2_CTX2_CTRL1_TRANSCODE_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX2_CTRL1_TRANSCODE_MAX                            (0x0000000FU)

#define CSL_CSI2_CTX2_CTRL1_HSCALE_MASK                              (0x10000000U)
#define CSL_CSI2_CTX2_CTRL1_HSCALE_SHIFT                             (0x0000001CU)
#define CSL_CSI2_CTX2_CTRL1_HSCALE_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX2_CTRL1_HSCALE_MAX                               (0x00000001U)

#define CSL_CSI2_CTX2_CTRL1_RES19_MASK                               (0x20000000U)
#define CSL_CSI2_CTX2_CTRL1_RES19_SHIFT                              (0x0000001DU)
#define CSL_CSI2_CTX2_CTRL1_RES19_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX2_CTRL1_RES19_MAX                                (0x00000001U)

#define CSL_CSI2_CTX2_CTRL1_GENERIC_MASK                             (0x40000000U)
#define CSL_CSI2_CTX2_CTRL1_GENERIC_SHIFT                            (0x0000001EU)
#define CSL_CSI2_CTX2_CTRL1_GENERIC_RESETVAL                         (0x00000000U)
#define CSL_CSI2_CTX2_CTRL1_GENERIC_MAX                              (0x00000001U)

#define CSL_CSI2_CTX2_CTRL1_BYTESWAP_MASK                            (0x80000000U)
#define CSL_CSI2_CTX2_CTRL1_BYTESWAP_SHIFT                           (0x0000001FU)
#define CSL_CSI2_CTX2_CTRL1_BYTESWAP_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX2_CTRL1_BYTESWAP_MAX                             (0x00000001U)

#define CSL_CSI2_CTX2_CTRL1_RESETVAL                                 (0x00010008U)

/* CSI2_CTX2_CTRL2 */

#define CSL_CSI2_CTX2_CTRL2_FORMAT_MASK                              (0x000003FFU)
#define CSL_CSI2_CTX2_CTRL2_FORMAT_SHIFT                             (0x00000000U)
#define CSL_CSI2_CTX2_CTRL2_FORMAT_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX2_CTRL2_FORMAT_MAX                               (0x000003FFU)

#define CSL_CSI2_CTX2_CTRL2_DPCM_PRED_MASK                           (0x00000400U)
#define CSL_CSI2_CTX2_CTRL2_DPCM_PRED_SHIFT                          (0x0000000AU)
#define CSL_CSI2_CTX2_CTRL2_DPCM_PRED_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX2_CTRL2_DPCM_PRED_MAX                            (0x00000001U)

#define CSL_CSI2_CTX2_CTRL2_VIRTUAL_ID_MASK                          (0x00001800U)
#define CSL_CSI2_CTX2_CTRL2_VIRTUAL_ID_SHIFT                         (0x0000000BU)
#define CSL_CSI2_CTX2_CTRL2_VIRTUAL_ID_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX2_CTRL2_VIRTUAL_ID_MAX                           (0x00000003U)

#define CSL_CSI2_CTX2_CTRL2_USER_DEF_MAPPING_MASK                    (0x00006000U)
#define CSL_CSI2_CTX2_CTRL2_USER_DEF_MAPPING_SHIFT                   (0x0000000DU)
#define CSL_CSI2_CTX2_CTRL2_USER_DEF_MAPPING_RESETVAL                (0x00000000U)
#define CSL_CSI2_CTX2_CTRL2_USER_DEF_MAPPING_MAX                     (0x00000003U)

#define CSL_CSI2_CTX2_CTRL2_RES20_MASK                               (0x00008000U)
#define CSL_CSI2_CTX2_CTRL2_RES20_SHIFT                              (0x0000000FU)
#define CSL_CSI2_CTX2_CTRL2_RES20_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX2_CTRL2_RES20_MAX                                (0x00000001U)

#define CSL_CSI2_CTX2_CTRL2_FRAME_MASK                               (0xFFFF0000U)
#define CSL_CSI2_CTX2_CTRL2_FRAME_SHIFT                              (0x00000010U)
#define CSL_CSI2_CTX2_CTRL2_FRAME_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX2_CTRL2_FRAME_MAX                                (0x0000FFFFU)

#define CSL_CSI2_CTX2_CTRL2_RESETVAL                                 (0x00000000U)

/* CSI2_CTX2_DAT_OFST */

#define CSL_CSI2_CTX2_DAT_OFST_OFST_MASK                             (0x0001FFE0U)
#define CSL_CSI2_CTX2_DAT_OFST_OFST_SHIFT                            (0x00000005U)
#define CSL_CSI2_CTX2_DAT_OFST_OFST_RESETVAL                         (0x00000000U)
#define CSL_CSI2_CTX2_DAT_OFST_OFST_MAX                              (0x00000FFFU)

#define CSL_CSI2_CTX2_DAT_OFST_RES21_MASK                            (0xFFFE0000U)
#define CSL_CSI2_CTX2_DAT_OFST_RES21_SHIFT                           (0x00000011U)
#define CSL_CSI2_CTX2_DAT_OFST_RES21_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX2_DAT_OFST_RES21_MAX                             (0x00007FFFU)

#define CSL_CSI2_CTX2_DAT_OFST_RESETVAL                              (0x00000000U)

/* CSI2_CTX2_DAT_PING_ADDR */

#define CSL_CSI2_CTX2_DAT_PING_ADDR_RES_MASK                         (0x0000001FU)
#define CSL_CSI2_CTX2_DAT_PING_ADDR_RES_SHIFT                        (0x00000000U)
#define CSL_CSI2_CTX2_DAT_PING_ADDR_RES_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX2_DAT_PING_ADDR_RES_MAX                          (0x0000001FU)

#define CSL_CSI2_CTX2_DAT_PING_ADDR_ADDR_MASK                        (0xFFFFFFE0U)
#define CSL_CSI2_CTX2_DAT_PING_ADDR_ADDR_SHIFT                       (0x00000005U)
#define CSL_CSI2_CTX2_DAT_PING_ADDR_ADDR_RESETVAL                    (0x00000000U)
#define CSL_CSI2_CTX2_DAT_PING_ADDR_ADDR_MAX                         (0x07FFFFFFU)

#define CSL_CSI2_CTX2_DAT_PING_ADDR_RESETVAL                         (0x00000000U)

/* CSI2_CTX2_DAT_PONG_ADDR */

#define CSL_CSI2_CTX2_DAT_PONG_ADDR_RES_MASK                         (0x0000001FU)
#define CSL_CSI2_CTX2_DAT_PONG_ADDR_RES_SHIFT                        (0x00000000U)
#define CSL_CSI2_CTX2_DAT_PONG_ADDR_RES_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX2_DAT_PONG_ADDR_RES_MAX                          (0x0000001FU)

#define CSL_CSI2_CTX2_DAT_PONG_ADDR_ADDR_MASK                        (0xFFFFFFE0U)
#define CSL_CSI2_CTX2_DAT_PONG_ADDR_ADDR_SHIFT                       (0x00000005U)
#define CSL_CSI2_CTX2_DAT_PONG_ADDR_ADDR_RESETVAL                    (0x00000000U)
#define CSL_CSI2_CTX2_DAT_PONG_ADDR_ADDR_MAX                         (0x07FFFFFFU)

#define CSL_CSI2_CTX2_DAT_PONG_ADDR_RESETVAL                         (0x00000000U)

/* CSI2_CTX2_IRQENABLE */

#define CSL_CSI2_CTX2_IRQENABLE_FS_IRQ_MASK                          (0x00000001U)
#define CSL_CSI2_CTX2_IRQENABLE_FS_IRQ_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX2_IRQENABLE_FS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX2_IRQENABLE_FS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX2_IRQENABLE_FE_IRQ_MASK                          (0x00000002U)
#define CSL_CSI2_CTX2_IRQENABLE_FE_IRQ_SHIFT                         (0x00000001U)
#define CSL_CSI2_CTX2_IRQENABLE_FE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX2_IRQENABLE_FE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX2_IRQENABLE_LS_IRQ_MASK                          (0x00000004U)
#define CSL_CSI2_CTX2_IRQENABLE_LS_IRQ_SHIFT                         (0x00000002U)
#define CSL_CSI2_CTX2_IRQENABLE_LS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX2_IRQENABLE_LS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX2_IRQENABLE_LE_IRQ_MASK                          (0x00000008U)
#define CSL_CSI2_CTX2_IRQENABLE_LE_IRQ_SHIFT                         (0x00000003U)
#define CSL_CSI2_CTX2_IRQENABLE_LE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX2_IRQENABLE_LE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX2_IRQENABLE_RES23_MASK                           (0x00000010U)
#define CSL_CSI2_CTX2_IRQENABLE_RES23_SHIFT                          (0x00000004U)
#define CSL_CSI2_CTX2_IRQENABLE_RES23_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX2_IRQENABLE_RES23_MAX                            (0x00000001U)

#define CSL_CSI2_CTX2_IRQENABLE_CS_IRQ_MASK                          (0x00000020U)
#define CSL_CSI2_CTX2_IRQENABLE_CS_IRQ_SHIFT                         (0x00000005U)
#define CSL_CSI2_CTX2_IRQENABLE_CS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX2_IRQENABLE_CS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX2_IRQENABLE_FRAME_NUMBER_IRQ_MASK                (0x00000040U)
#define CSL_CSI2_CTX2_IRQENABLE_FRAME_NUMBER_IRQ_SHIFT               (0x00000006U)
#define CSL_CSI2_CTX2_IRQENABLE_FRAME_NUMBER_IRQ_RESETVAL            (0x00000000U)
#define CSL_CSI2_CTX2_IRQENABLE_FRAME_NUMBER_IRQ_MAX                 (0x00000001U)

#define CSL_CSI2_CTX2_IRQENABLE_LINE_NUMBER_IRQ_MASK                 (0x00000080U)
#define CSL_CSI2_CTX2_IRQENABLE_LINE_NUMBER_IRQ_SHIFT                (0x00000007U)
#define CSL_CSI2_CTX2_IRQENABLE_LINE_NUMBER_IRQ_RESETVAL             (0x00000000U)
#define CSL_CSI2_CTX2_IRQENABLE_LINE_NUMBER_IRQ_MAX                  (0x00000001U)

#define CSL_CSI2_CTX2_IRQENABLE_ECC_CORRECTION_IRQ_MASK              (0x00000100U)
#define CSL_CSI2_CTX2_IRQENABLE_ECC_CORRECTION_IRQ_SHIFT             (0x00000008U)
#define CSL_CSI2_CTX2_IRQENABLE_ECC_CORRECTION_IRQ_RESETVAL          (0x00000000U)
#define CSL_CSI2_CTX2_IRQENABLE_ECC_CORRECTION_IRQ_MAX               (0x00000001U)

#define CSL_CSI2_CTX2_IRQENABLE_RES22_MASK                           (0xFFFFFE00U)
#define CSL_CSI2_CTX2_IRQENABLE_RES22_SHIFT                          (0x00000009U)
#define CSL_CSI2_CTX2_IRQENABLE_RES22_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX2_IRQENABLE_RES22_MAX                            (0x007FFFFFU)

#define CSL_CSI2_CTX2_IRQENABLE_RESETVAL                             (0x00000000U)

/* CSI2_CTX2_IRQSTATUS */

#define CSL_CSI2_CTX2_IRQSTATUS_FS_IRQ_MASK                          (0x00000001U)
#define CSL_CSI2_CTX2_IRQSTATUS_FS_IRQ_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX2_IRQSTATUS_FS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX2_IRQSTATUS_FS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX2_IRQSTATUS_FE_IRQ_MASK                          (0x00000002U)
#define CSL_CSI2_CTX2_IRQSTATUS_FE_IRQ_SHIFT                         (0x00000001U)
#define CSL_CSI2_CTX2_IRQSTATUS_FE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX2_IRQSTATUS_FE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX2_IRQSTATUS_LS_IRQ_MASK                          (0x00000004U)
#define CSL_CSI2_CTX2_IRQSTATUS_LS_IRQ_SHIFT                         (0x00000002U)
#define CSL_CSI2_CTX2_IRQSTATUS_LS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX2_IRQSTATUS_LS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX2_IRQSTATUS_LE_IRQ_MASK                          (0x00000008U)
#define CSL_CSI2_CTX2_IRQSTATUS_LE_IRQ_SHIFT                         (0x00000003U)
#define CSL_CSI2_CTX2_IRQSTATUS_LE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX2_IRQSTATUS_LE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX2_IRQSTATUS_RES25_MASK                           (0x00000010U)
#define CSL_CSI2_CTX2_IRQSTATUS_RES25_SHIFT                          (0x00000004U)
#define CSL_CSI2_CTX2_IRQSTATUS_RES25_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX2_IRQSTATUS_RES25_MAX                            (0x00000001U)

#define CSL_CSI2_CTX2_IRQSTATUS_CS_IRQ_MASK                          (0x00000020U)
#define CSL_CSI2_CTX2_IRQSTATUS_CS_IRQ_SHIFT                         (0x00000005U)
#define CSL_CSI2_CTX2_IRQSTATUS_CS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX2_IRQSTATUS_CS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX2_IRQSTATUS_FRAME_NUMBER_IRQ_MASK                (0x00000040U)
#define CSL_CSI2_CTX2_IRQSTATUS_FRAME_NUMBER_IRQ_SHIFT               (0x00000006U)
#define CSL_CSI2_CTX2_IRQSTATUS_FRAME_NUMBER_IRQ_RESETVAL            (0x00000000U)
#define CSL_CSI2_CTX2_IRQSTATUS_FRAME_NUMBER_IRQ_MAX                 (0x00000001U)

#define CSL_CSI2_CTX2_IRQSTATUS_LINE_NUMBER_IRQ_MASK                 (0x00000080U)
#define CSL_CSI2_CTX2_IRQSTATUS_LINE_NUMBER_IRQ_SHIFT                (0x00000007U)
#define CSL_CSI2_CTX2_IRQSTATUS_LINE_NUMBER_IRQ_RESETVAL             (0x00000000U)
#define CSL_CSI2_CTX2_IRQSTATUS_LINE_NUMBER_IRQ_MAX                  (0x00000001U)

#define CSL_CSI2_CTX2_IRQSTATUS_ECC_CORRECTION_IRQ_MASK              (0x00000100U)
#define CSL_CSI2_CTX2_IRQSTATUS_ECC_CORRECTION_IRQ_SHIFT             (0x00000008U)
#define CSL_CSI2_CTX2_IRQSTATUS_ECC_CORRECTION_IRQ_RESETVAL          (0x00000000U)
#define CSL_CSI2_CTX2_IRQSTATUS_ECC_CORRECTION_IRQ_MAX               (0x00000001U)

#define CSL_CSI2_CTX2_IRQSTATUS_RES24_MASK                           (0xFFFFFE00U)
#define CSL_CSI2_CTX2_IRQSTATUS_RES24_SHIFT                          (0x00000009U)
#define CSL_CSI2_CTX2_IRQSTATUS_RES24_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX2_IRQSTATUS_RES24_MAX                            (0x007FFFFFU)

#define CSL_CSI2_CTX2_IRQSTATUS_RESETVAL                             (0x00000000U)

/* CSI2_CTX2_CTRL3 */

#define CSL_CSI2_CTX2_CTRL3_LINE_NUMBER_MASK                         (0x0000FFFFU)
#define CSL_CSI2_CTX2_CTRL3_LINE_NUMBER_SHIFT                        (0x00000000U)
#define CSL_CSI2_CTX2_CTRL3_LINE_NUMBER_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX2_CTRL3_LINE_NUMBER_MAX                          (0x0000FFFFU)

#define CSL_CSI2_CTX2_CTRL3_ALPHA_MASK                               (0x3FFF0000U)
#define CSL_CSI2_CTX2_CTRL3_ALPHA_SHIFT                              (0x00000010U)
#define CSL_CSI2_CTX2_CTRL3_ALPHA_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX2_CTRL3_ALPHA_MAX                                (0x00003FFFU)

#define CSL_CSI2_CTX2_CTRL3_RESERVED_MASK                            (0xC0000000U)
#define CSL_CSI2_CTX2_CTRL3_RESERVED_SHIFT                           (0x0000001EU)
#define CSL_CSI2_CTX2_CTRL3_RESERVED_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX2_CTRL3_RESERVED_MAX                             (0x00000003U)

#define CSL_CSI2_CTX2_CTRL3_RESETVAL                                 (0x00000000U)

/* CSI2_CTX3_CTRL1 */

#define CSL_CSI2_CTX3_CTRL1_CTX_EN_MASK                              (0x00000001U)
#define CSL_CSI2_CTX3_CTRL1_CTX_EN_SHIFT                             (0x00000000U)
#define CSL_CSI2_CTX3_CTRL1_CTX_EN_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX3_CTRL1_CTX_EN_MAX                               (0x00000001U)

#define CSL_CSI2_CTX3_CTRL1_LINE_MODULO_MASK                         (0x00000002U)
#define CSL_CSI2_CTX3_CTRL1_LINE_MODULO_SHIFT                        (0x00000001U)
#define CSL_CSI2_CTX3_CTRL1_LINE_MODULO_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX3_CTRL1_LINE_MODULO_MAX                          (0x00000001U)

#define CSL_CSI2_CTX3_CTRL1_VP_FORCE_MASK                            (0x00000004U)
#define CSL_CSI2_CTX3_CTRL1_VP_FORCE_SHIFT                           (0x00000002U)
#define CSL_CSI2_CTX3_CTRL1_VP_FORCE_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX3_CTRL1_VP_FORCE_MAX                             (0x00000001U)

#define CSL_CSI2_CTX3_CTRL1_PING_PONG_MASK                           (0x00000008U)
#define CSL_CSI2_CTX3_CTRL1_PING_PONG_SHIFT                          (0x00000003U)
#define CSL_CSI2_CTX3_CTRL1_PING_PONG_RESETVAL                       (0x00000001U)
#define CSL_CSI2_CTX3_CTRL1_PING_PONG_MAX                            (0x00000001U)

#define CSL_CSI2_CTX3_CTRL1_COUNT_UNLOCK_MASK                        (0x00000010U)
#define CSL_CSI2_CTX3_CTRL1_COUNT_UNLOCK_SHIFT                       (0x00000004U)
#define CSL_CSI2_CTX3_CTRL1_COUNT_UNLOCK_RESETVAL                    (0x00000000U)
#define CSL_CSI2_CTX3_CTRL1_COUNT_UNLOCK_MAX                         (0x00000001U)

#define CSL_CSI2_CTX3_CTRL1_CS_EN_MASK                               (0x00000020U)
#define CSL_CSI2_CTX3_CTRL1_CS_EN_SHIFT                              (0x00000005U)
#define CSL_CSI2_CTX3_CTRL1_CS_EN_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX3_CTRL1_CS_EN_MAX                                (0x00000001U)

#define CSL_CSI2_CTX3_CTRL1_EOL_EN_MASK                              (0x00000040U)
#define CSL_CSI2_CTX3_CTRL1_EOL_EN_SHIFT                             (0x00000006U)
#define CSL_CSI2_CTX3_CTRL1_EOL_EN_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX3_CTRL1_EOL_EN_MAX                               (0x00000001U)

#define CSL_CSI2_CTX3_CTRL1_EOF_EN_MASK                              (0x00000080U)
#define CSL_CSI2_CTX3_CTRL1_EOF_EN_SHIFT                             (0x00000007U)
#define CSL_CSI2_CTX3_CTRL1_EOF_EN_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX3_CTRL1_EOF_EN_MAX                               (0x00000001U)

#define CSL_CSI2_CTX3_CTRL1_COUNT_MASK                               (0x0000FF00U)
#define CSL_CSI2_CTX3_CTRL1_COUNT_SHIFT                              (0x00000008U)
#define CSL_CSI2_CTX3_CTRL1_COUNT_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX3_CTRL1_COUNT_MAX                                (0x000000FFU)

#define CSL_CSI2_CTX3_CTRL1_FEC_NUMBER_MASK                          (0x00FF0000U)
#define CSL_CSI2_CTX3_CTRL1_FEC_NUMBER_SHIFT                         (0x00000010U)
#define CSL_CSI2_CTX3_CTRL1_FEC_NUMBER_RESETVAL                      (0x00000001U)
#define CSL_CSI2_CTX3_CTRL1_FEC_NUMBER_MAX                           (0x000000FFU)

#define CSL_CSI2_CTX3_CTRL1_TRANSCODE_MASK                           (0x0F000000U)
#define CSL_CSI2_CTX3_CTRL1_TRANSCODE_SHIFT                          (0x00000018U)
#define CSL_CSI2_CTX3_CTRL1_TRANSCODE_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX3_CTRL1_TRANSCODE_MAX                            (0x0000000FU)

#define CSL_CSI2_CTX3_CTRL1_HSCALE_MASK                              (0x10000000U)
#define CSL_CSI2_CTX3_CTRL1_HSCALE_SHIFT                             (0x0000001CU)
#define CSL_CSI2_CTX3_CTRL1_HSCALE_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX3_CTRL1_HSCALE_MAX                               (0x00000001U)

#define CSL_CSI2_CTX3_CTRL1_RES19_MASK                               (0x20000000U)
#define CSL_CSI2_CTX3_CTRL1_RES19_SHIFT                              (0x0000001DU)
#define CSL_CSI2_CTX3_CTRL1_RES19_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX3_CTRL1_RES19_MAX                                (0x00000001U)

#define CSL_CSI2_CTX3_CTRL1_GENERIC_MASK                             (0x40000000U)
#define CSL_CSI2_CTX3_CTRL1_GENERIC_SHIFT                            (0x0000001EU)
#define CSL_CSI2_CTX3_CTRL1_GENERIC_RESETVAL                         (0x00000000U)
#define CSL_CSI2_CTX3_CTRL1_GENERIC_MAX                              (0x00000001U)

#define CSL_CSI2_CTX3_CTRL1_BYTESWAP_MASK                            (0x80000000U)
#define CSL_CSI2_CTX3_CTRL1_BYTESWAP_SHIFT                           (0x0000001FU)
#define CSL_CSI2_CTX3_CTRL1_BYTESWAP_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX3_CTRL1_BYTESWAP_MAX                             (0x00000001U)

#define CSL_CSI2_CTX3_CTRL1_RESETVAL                                 (0x00010008U)

/* CSI2_CTX3_CTRL2 */

#define CSL_CSI2_CTX3_CTRL2_FORMAT_MASK                              (0x000003FFU)
#define CSL_CSI2_CTX3_CTRL2_FORMAT_SHIFT                             (0x00000000U)
#define CSL_CSI2_CTX3_CTRL2_FORMAT_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX3_CTRL2_FORMAT_MAX                               (0x000003FFU)

#define CSL_CSI2_CTX3_CTRL2_DPCM_PRED_MASK                           (0x00000400U)
#define CSL_CSI2_CTX3_CTRL2_DPCM_PRED_SHIFT                          (0x0000000AU)
#define CSL_CSI2_CTX3_CTRL2_DPCM_PRED_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX3_CTRL2_DPCM_PRED_MAX                            (0x00000001U)

#define CSL_CSI2_CTX3_CTRL2_VIRTUAL_ID_MASK                          (0x00001800U)
#define CSL_CSI2_CTX3_CTRL2_VIRTUAL_ID_SHIFT                         (0x0000000BU)
#define CSL_CSI2_CTX3_CTRL2_VIRTUAL_ID_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX3_CTRL2_VIRTUAL_ID_MAX                           (0x00000003U)

#define CSL_CSI2_CTX3_CTRL2_USER_DEF_MAPPING_MASK                    (0x00006000U)
#define CSL_CSI2_CTX3_CTRL2_USER_DEF_MAPPING_SHIFT                   (0x0000000DU)
#define CSL_CSI2_CTX3_CTRL2_USER_DEF_MAPPING_RESETVAL                (0x00000000U)
#define CSL_CSI2_CTX3_CTRL2_USER_DEF_MAPPING_MAX                     (0x00000003U)

#define CSL_CSI2_CTX3_CTRL2_RES20_MASK                               (0x00008000U)
#define CSL_CSI2_CTX3_CTRL2_RES20_SHIFT                              (0x0000000FU)
#define CSL_CSI2_CTX3_CTRL2_RES20_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX3_CTRL2_RES20_MAX                                (0x00000001U)

#define CSL_CSI2_CTX3_CTRL2_FRAME_MASK                               (0xFFFF0000U)
#define CSL_CSI2_CTX3_CTRL2_FRAME_SHIFT                              (0x00000010U)
#define CSL_CSI2_CTX3_CTRL2_FRAME_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX3_CTRL2_FRAME_MAX                                (0x0000FFFFU)

#define CSL_CSI2_CTX3_CTRL2_RESETVAL                                 (0x00000000U)

/* CSI2_CTX3_DAT_OFST */

#define CSL_CSI2_CTX3_DAT_OFST_OFST_MASK                             (0x0001FFE0U)
#define CSL_CSI2_CTX3_DAT_OFST_OFST_SHIFT                            (0x00000005U)
#define CSL_CSI2_CTX3_DAT_OFST_OFST_RESETVAL                         (0x00000000U)
#define CSL_CSI2_CTX3_DAT_OFST_OFST_MAX                              (0x00000FFFU)

#define CSL_CSI2_CTX3_DAT_OFST_RES21_MASK                            (0xFFFE0000U)
#define CSL_CSI2_CTX3_DAT_OFST_RES21_SHIFT                           (0x00000011U)
#define CSL_CSI2_CTX3_DAT_OFST_RES21_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX3_DAT_OFST_RES21_MAX                             (0x00007FFFU)

#define CSL_CSI2_CTX3_DAT_OFST_RESETVAL                              (0x00000000U)

/* CSI2_CTX3_DAT_PING_ADDR */

#define CSL_CSI2_CTX3_DAT_PING_ADDR_RES_MASK                         (0x0000001FU)
#define CSL_CSI2_CTX3_DAT_PING_ADDR_RES_SHIFT                        (0x00000000U)
#define CSL_CSI2_CTX3_DAT_PING_ADDR_RES_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX3_DAT_PING_ADDR_RES_MAX                          (0x0000001FU)

#define CSL_CSI2_CTX3_DAT_PING_ADDR_ADDR_MASK                        (0xFFFFFFE0U)
#define CSL_CSI2_CTX3_DAT_PING_ADDR_ADDR_SHIFT                       (0x00000005U)
#define CSL_CSI2_CTX3_DAT_PING_ADDR_ADDR_RESETVAL                    (0x00000000U)
#define CSL_CSI2_CTX3_DAT_PING_ADDR_ADDR_MAX                         (0x07FFFFFFU)

#define CSL_CSI2_CTX3_DAT_PING_ADDR_RESETVAL                         (0x00000000U)

/* CSI2_CTX3_DAT_PONG_ADDR */

#define CSL_CSI2_CTX3_DAT_PONG_ADDR_RES_MASK                         (0x0000001FU)
#define CSL_CSI2_CTX3_DAT_PONG_ADDR_RES_SHIFT                        (0x00000000U)
#define CSL_CSI2_CTX3_DAT_PONG_ADDR_RES_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX3_DAT_PONG_ADDR_RES_MAX                          (0x0000001FU)

#define CSL_CSI2_CTX3_DAT_PONG_ADDR_ADDR_MASK                        (0xFFFFFFE0U)
#define CSL_CSI2_CTX3_DAT_PONG_ADDR_ADDR_SHIFT                       (0x00000005U)
#define CSL_CSI2_CTX3_DAT_PONG_ADDR_ADDR_RESETVAL                    (0x00000000U)
#define CSL_CSI2_CTX3_DAT_PONG_ADDR_ADDR_MAX                         (0x07FFFFFFU)

#define CSL_CSI2_CTX3_DAT_PONG_ADDR_RESETVAL                         (0x00000000U)

/* CSI2_CTX3_IRQENABLE */

#define CSL_CSI2_CTX3_IRQENABLE_FS_IRQ_MASK                          (0x00000001U)
#define CSL_CSI2_CTX3_IRQENABLE_FS_IRQ_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX3_IRQENABLE_FS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX3_IRQENABLE_FS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX3_IRQENABLE_FE_IRQ_MASK                          (0x00000002U)
#define CSL_CSI2_CTX3_IRQENABLE_FE_IRQ_SHIFT                         (0x00000001U)
#define CSL_CSI2_CTX3_IRQENABLE_FE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX3_IRQENABLE_FE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX3_IRQENABLE_LS_IRQ_MASK                          (0x00000004U)
#define CSL_CSI2_CTX3_IRQENABLE_LS_IRQ_SHIFT                         (0x00000002U)
#define CSL_CSI2_CTX3_IRQENABLE_LS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX3_IRQENABLE_LS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX3_IRQENABLE_LE_IRQ_MASK                          (0x00000008U)
#define CSL_CSI2_CTX3_IRQENABLE_LE_IRQ_SHIFT                         (0x00000003U)
#define CSL_CSI2_CTX3_IRQENABLE_LE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX3_IRQENABLE_LE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX3_IRQENABLE_RES23_MASK                           (0x00000010U)
#define CSL_CSI2_CTX3_IRQENABLE_RES23_SHIFT                          (0x00000004U)
#define CSL_CSI2_CTX3_IRQENABLE_RES23_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX3_IRQENABLE_RES23_MAX                            (0x00000001U)

#define CSL_CSI2_CTX3_IRQENABLE_CS_IRQ_MASK                          (0x00000020U)
#define CSL_CSI2_CTX3_IRQENABLE_CS_IRQ_SHIFT                         (0x00000005U)
#define CSL_CSI2_CTX3_IRQENABLE_CS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX3_IRQENABLE_CS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX3_IRQENABLE_FRAME_NUMBER_IRQ_MASK                (0x00000040U)
#define CSL_CSI2_CTX3_IRQENABLE_FRAME_NUMBER_IRQ_SHIFT               (0x00000006U)
#define CSL_CSI2_CTX3_IRQENABLE_FRAME_NUMBER_IRQ_RESETVAL            (0x00000000U)
#define CSL_CSI2_CTX3_IRQENABLE_FRAME_NUMBER_IRQ_MAX                 (0x00000001U)

#define CSL_CSI2_CTX3_IRQENABLE_LINE_NUMBER_IRQ_MASK                 (0x00000080U)
#define CSL_CSI2_CTX3_IRQENABLE_LINE_NUMBER_IRQ_SHIFT                (0x00000007U)
#define CSL_CSI2_CTX3_IRQENABLE_LINE_NUMBER_IRQ_RESETVAL             (0x00000000U)
#define CSL_CSI2_CTX3_IRQENABLE_LINE_NUMBER_IRQ_MAX                  (0x00000001U)

#define CSL_CSI2_CTX3_IRQENABLE_ECC_CORRECTION_IRQ_MASK              (0x00000100U)
#define CSL_CSI2_CTX3_IRQENABLE_ECC_CORRECTION_IRQ_SHIFT             (0x00000008U)
#define CSL_CSI2_CTX3_IRQENABLE_ECC_CORRECTION_IRQ_RESETVAL          (0x00000000U)
#define CSL_CSI2_CTX3_IRQENABLE_ECC_CORRECTION_IRQ_MAX               (0x00000001U)

#define CSL_CSI2_CTX3_IRQENABLE_RES22_MASK                           (0xFFFFFE00U)
#define CSL_CSI2_CTX3_IRQENABLE_RES22_SHIFT                          (0x00000009U)
#define CSL_CSI2_CTX3_IRQENABLE_RES22_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX3_IRQENABLE_RES22_MAX                            (0x007FFFFFU)

#define CSL_CSI2_CTX3_IRQENABLE_RESETVAL                             (0x00000000U)

/* CSI2_CTX3_IRQSTATUS */

#define CSL_CSI2_CTX3_IRQSTATUS_FS_IRQ_MASK                          (0x00000001U)
#define CSL_CSI2_CTX3_IRQSTATUS_FS_IRQ_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX3_IRQSTATUS_FS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX3_IRQSTATUS_FS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX3_IRQSTATUS_FE_IRQ_MASK                          (0x00000002U)
#define CSL_CSI2_CTX3_IRQSTATUS_FE_IRQ_SHIFT                         (0x00000001U)
#define CSL_CSI2_CTX3_IRQSTATUS_FE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX3_IRQSTATUS_FE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX3_IRQSTATUS_LS_IRQ_MASK                          (0x00000004U)
#define CSL_CSI2_CTX3_IRQSTATUS_LS_IRQ_SHIFT                         (0x00000002U)
#define CSL_CSI2_CTX3_IRQSTATUS_LS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX3_IRQSTATUS_LS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX3_IRQSTATUS_LE_IRQ_MASK                          (0x00000008U)
#define CSL_CSI2_CTX3_IRQSTATUS_LE_IRQ_SHIFT                         (0x00000003U)
#define CSL_CSI2_CTX3_IRQSTATUS_LE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX3_IRQSTATUS_LE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX3_IRQSTATUS_RES25_MASK                           (0x00000010U)
#define CSL_CSI2_CTX3_IRQSTATUS_RES25_SHIFT                          (0x00000004U)
#define CSL_CSI2_CTX3_IRQSTATUS_RES25_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX3_IRQSTATUS_RES25_MAX                            (0x00000001U)

#define CSL_CSI2_CTX3_IRQSTATUS_CS_IRQ_MASK                          (0x00000020U)
#define CSL_CSI2_CTX3_IRQSTATUS_CS_IRQ_SHIFT                         (0x00000005U)
#define CSL_CSI2_CTX3_IRQSTATUS_CS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX3_IRQSTATUS_CS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX3_IRQSTATUS_FRAME_NUMBER_IRQ_MASK                (0x00000040U)
#define CSL_CSI2_CTX3_IRQSTATUS_FRAME_NUMBER_IRQ_SHIFT               (0x00000006U)
#define CSL_CSI2_CTX3_IRQSTATUS_FRAME_NUMBER_IRQ_RESETVAL            (0x00000000U)
#define CSL_CSI2_CTX3_IRQSTATUS_FRAME_NUMBER_IRQ_MAX                 (0x00000001U)

#define CSL_CSI2_CTX3_IRQSTATUS_LINE_NUMBER_IRQ_MASK                 (0x00000080U)
#define CSL_CSI2_CTX3_IRQSTATUS_LINE_NUMBER_IRQ_SHIFT                (0x00000007U)
#define CSL_CSI2_CTX3_IRQSTATUS_LINE_NUMBER_IRQ_RESETVAL             (0x00000000U)
#define CSL_CSI2_CTX3_IRQSTATUS_LINE_NUMBER_IRQ_MAX                  (0x00000001U)

#define CSL_CSI2_CTX3_IRQSTATUS_ECC_CORRECTION_IRQ_MASK              (0x00000100U)
#define CSL_CSI2_CTX3_IRQSTATUS_ECC_CORRECTION_IRQ_SHIFT             (0x00000008U)
#define CSL_CSI2_CTX3_IRQSTATUS_ECC_CORRECTION_IRQ_RESETVAL          (0x00000000U)
#define CSL_CSI2_CTX3_IRQSTATUS_ECC_CORRECTION_IRQ_MAX               (0x00000001U)

#define CSL_CSI2_CTX3_IRQSTATUS_RES24_MASK                           (0xFFFFFE00U)
#define CSL_CSI2_CTX3_IRQSTATUS_RES24_SHIFT                          (0x00000009U)
#define CSL_CSI2_CTX3_IRQSTATUS_RES24_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX3_IRQSTATUS_RES24_MAX                            (0x007FFFFFU)

#define CSL_CSI2_CTX3_IRQSTATUS_RESETVAL                             (0x00000000U)

/* CSI2_CTX3_CTRL3 */

#define CSL_CSI2_CTX3_CTRL3_LINE_NUMBER_MASK                         (0x0000FFFFU)
#define CSL_CSI2_CTX3_CTRL3_LINE_NUMBER_SHIFT                        (0x00000000U)
#define CSL_CSI2_CTX3_CTRL3_LINE_NUMBER_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX3_CTRL3_LINE_NUMBER_MAX                          (0x0000FFFFU)

#define CSL_CSI2_CTX3_CTRL3_ALPHA_MASK                               (0x3FFF0000U)
#define CSL_CSI2_CTX3_CTRL3_ALPHA_SHIFT                              (0x00000010U)
#define CSL_CSI2_CTX3_CTRL3_ALPHA_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX3_CTRL3_ALPHA_MAX                                (0x00003FFFU)

#define CSL_CSI2_CTX3_CTRL3_RESERVED_MASK                            (0xC0000000U)
#define CSL_CSI2_CTX3_CTRL3_RESERVED_SHIFT                           (0x0000001EU)
#define CSL_CSI2_CTX3_CTRL3_RESERVED_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX3_CTRL3_RESERVED_MAX                             (0x00000003U)

#define CSL_CSI2_CTX3_CTRL3_RESETVAL                                 (0x00000000U)

/* CSI2_CTX4_CTRL1 */

#define CSL_CSI2_CTX4_CTRL1_CTX_EN_MASK                              (0x00000001U)
#define CSL_CSI2_CTX4_CTRL1_CTX_EN_SHIFT                             (0x00000000U)
#define CSL_CSI2_CTX4_CTRL1_CTX_EN_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX4_CTRL1_CTX_EN_MAX                               (0x00000001U)

#define CSL_CSI2_CTX4_CTRL1_LINE_MODULO_MASK                         (0x00000002U)
#define CSL_CSI2_CTX4_CTRL1_LINE_MODULO_SHIFT                        (0x00000001U)
#define CSL_CSI2_CTX4_CTRL1_LINE_MODULO_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX4_CTRL1_LINE_MODULO_MAX                          (0x00000001U)

#define CSL_CSI2_CTX4_CTRL1_VP_FORCE_MASK                            (0x00000004U)
#define CSL_CSI2_CTX4_CTRL1_VP_FORCE_SHIFT                           (0x00000002U)
#define CSL_CSI2_CTX4_CTRL1_VP_FORCE_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX4_CTRL1_VP_FORCE_MAX                             (0x00000001U)

#define CSL_CSI2_CTX4_CTRL1_PING_PONG_MASK                           (0x00000008U)
#define CSL_CSI2_CTX4_CTRL1_PING_PONG_SHIFT                          (0x00000003U)
#define CSL_CSI2_CTX4_CTRL1_PING_PONG_RESETVAL                       (0x00000001U)
#define CSL_CSI2_CTX4_CTRL1_PING_PONG_MAX                            (0x00000001U)

#define CSL_CSI2_CTX4_CTRL1_COUNT_UNLOCK_MASK                        (0x00000010U)
#define CSL_CSI2_CTX4_CTRL1_COUNT_UNLOCK_SHIFT                       (0x00000004U)
#define CSL_CSI2_CTX4_CTRL1_COUNT_UNLOCK_RESETVAL                    (0x00000000U)
#define CSL_CSI2_CTX4_CTRL1_COUNT_UNLOCK_MAX                         (0x00000001U)

#define CSL_CSI2_CTX4_CTRL1_CS_EN_MASK                               (0x00000020U)
#define CSL_CSI2_CTX4_CTRL1_CS_EN_SHIFT                              (0x00000005U)
#define CSL_CSI2_CTX4_CTRL1_CS_EN_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX4_CTRL1_CS_EN_MAX                                (0x00000001U)

#define CSL_CSI2_CTX4_CTRL1_EOL_EN_MASK                              (0x00000040U)
#define CSL_CSI2_CTX4_CTRL1_EOL_EN_SHIFT                             (0x00000006U)
#define CSL_CSI2_CTX4_CTRL1_EOL_EN_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX4_CTRL1_EOL_EN_MAX                               (0x00000001U)

#define CSL_CSI2_CTX4_CTRL1_EOF_EN_MASK                              (0x00000080U)
#define CSL_CSI2_CTX4_CTRL1_EOF_EN_SHIFT                             (0x00000007U)
#define CSL_CSI2_CTX4_CTRL1_EOF_EN_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX4_CTRL1_EOF_EN_MAX                               (0x00000001U)

#define CSL_CSI2_CTX4_CTRL1_COUNT_MASK                               (0x0000FF00U)
#define CSL_CSI2_CTX4_CTRL1_COUNT_SHIFT                              (0x00000008U)
#define CSL_CSI2_CTX4_CTRL1_COUNT_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX4_CTRL1_COUNT_MAX                                (0x000000FFU)

#define CSL_CSI2_CTX4_CTRL1_FEC_NUMBER_MASK                          (0x00FF0000U)
#define CSL_CSI2_CTX4_CTRL1_FEC_NUMBER_SHIFT                         (0x00000010U)
#define CSL_CSI2_CTX4_CTRL1_FEC_NUMBER_RESETVAL                      (0x00000001U)
#define CSL_CSI2_CTX4_CTRL1_FEC_NUMBER_MAX                           (0x000000FFU)

#define CSL_CSI2_CTX4_CTRL1_TRANSCODE_MASK                           (0x0F000000U)
#define CSL_CSI2_CTX4_CTRL1_TRANSCODE_SHIFT                          (0x00000018U)
#define CSL_CSI2_CTX4_CTRL1_TRANSCODE_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX4_CTRL1_TRANSCODE_MAX                            (0x0000000FU)

#define CSL_CSI2_CTX4_CTRL1_HSCALE_MASK                              (0x10000000U)
#define CSL_CSI2_CTX4_CTRL1_HSCALE_SHIFT                             (0x0000001CU)
#define CSL_CSI2_CTX4_CTRL1_HSCALE_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX4_CTRL1_HSCALE_MAX                               (0x00000001U)

#define CSL_CSI2_CTX4_CTRL1_RES19_MASK                               (0x20000000U)
#define CSL_CSI2_CTX4_CTRL1_RES19_SHIFT                              (0x0000001DU)
#define CSL_CSI2_CTX4_CTRL1_RES19_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX4_CTRL1_RES19_MAX                                (0x00000001U)

#define CSL_CSI2_CTX4_CTRL1_GENERIC_MASK                             (0x40000000U)
#define CSL_CSI2_CTX4_CTRL1_GENERIC_SHIFT                            (0x0000001EU)
#define CSL_CSI2_CTX4_CTRL1_GENERIC_RESETVAL                         (0x00000000U)
#define CSL_CSI2_CTX4_CTRL1_GENERIC_MAX                              (0x00000001U)

#define CSL_CSI2_CTX4_CTRL1_BYTESWAP_MASK                            (0x80000000U)
#define CSL_CSI2_CTX4_CTRL1_BYTESWAP_SHIFT                           (0x0000001FU)
#define CSL_CSI2_CTX4_CTRL1_BYTESWAP_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX4_CTRL1_BYTESWAP_MAX                             (0x00000001U)

#define CSL_CSI2_CTX4_CTRL1_RESETVAL                                 (0x00010008U)

/* CSI2_CTX4_CTRL2 */

#define CSL_CSI2_CTX4_CTRL2_FORMAT_MASK                              (0x000003FFU)
#define CSL_CSI2_CTX4_CTRL2_FORMAT_SHIFT                             (0x00000000U)
#define CSL_CSI2_CTX4_CTRL2_FORMAT_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX4_CTRL2_FORMAT_MAX                               (0x000003FFU)

#define CSL_CSI2_CTX4_CTRL2_DPCM_PRED_MASK                           (0x00000400U)
#define CSL_CSI2_CTX4_CTRL2_DPCM_PRED_SHIFT                          (0x0000000AU)
#define CSL_CSI2_CTX4_CTRL2_DPCM_PRED_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX4_CTRL2_DPCM_PRED_MAX                            (0x00000001U)

#define CSL_CSI2_CTX4_CTRL2_VIRTUAL_ID_MASK                          (0x00001800U)
#define CSL_CSI2_CTX4_CTRL2_VIRTUAL_ID_SHIFT                         (0x0000000BU)
#define CSL_CSI2_CTX4_CTRL2_VIRTUAL_ID_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX4_CTRL2_VIRTUAL_ID_MAX                           (0x00000003U)

#define CSL_CSI2_CTX4_CTRL2_USER_DEF_MAPPING_MASK                    (0x00006000U)
#define CSL_CSI2_CTX4_CTRL2_USER_DEF_MAPPING_SHIFT                   (0x0000000DU)
#define CSL_CSI2_CTX4_CTRL2_USER_DEF_MAPPING_RESETVAL                (0x00000000U)
#define CSL_CSI2_CTX4_CTRL2_USER_DEF_MAPPING_MAX                     (0x00000003U)

#define CSL_CSI2_CTX4_CTRL2_RES20_MASK                               (0x00008000U)
#define CSL_CSI2_CTX4_CTRL2_RES20_SHIFT                              (0x0000000FU)
#define CSL_CSI2_CTX4_CTRL2_RES20_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX4_CTRL2_RES20_MAX                                (0x00000001U)

#define CSL_CSI2_CTX4_CTRL2_FRAME_MASK                               (0xFFFF0000U)
#define CSL_CSI2_CTX4_CTRL2_FRAME_SHIFT                              (0x00000010U)
#define CSL_CSI2_CTX4_CTRL2_FRAME_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX4_CTRL2_FRAME_MAX                                (0x0000FFFFU)

#define CSL_CSI2_CTX4_CTRL2_RESETVAL                                 (0x00000000U)

/* CSI2_CTX4_DAT_OFST */

#define CSL_CSI2_CTX4_DAT_OFST_OFST_MASK                             (0x0001FFE0U)
#define CSL_CSI2_CTX4_DAT_OFST_OFST_SHIFT                            (0x00000005U)
#define CSL_CSI2_CTX4_DAT_OFST_OFST_RESETVAL                         (0x00000000U)
#define CSL_CSI2_CTX4_DAT_OFST_OFST_MAX                              (0x00000FFFU)

#define CSL_CSI2_CTX4_DAT_OFST_RES21_MASK                            (0xFFFE0000U)
#define CSL_CSI2_CTX4_DAT_OFST_RES21_SHIFT                           (0x00000011U)
#define CSL_CSI2_CTX4_DAT_OFST_RES21_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX4_DAT_OFST_RES21_MAX                             (0x00007FFFU)

#define CSL_CSI2_CTX4_DAT_OFST_RESETVAL                              (0x00000000U)

/* CSI2_CTX4_DAT_PING_ADDR */

#define CSL_CSI2_CTX4_DAT_PING_ADDR_RES_MASK                         (0x0000001FU)
#define CSL_CSI2_CTX4_DAT_PING_ADDR_RES_SHIFT                        (0x00000000U)
#define CSL_CSI2_CTX4_DAT_PING_ADDR_RES_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX4_DAT_PING_ADDR_RES_MAX                          (0x0000001FU)

#define CSL_CSI2_CTX4_DAT_PING_ADDR_ADDR_MASK                        (0xFFFFFFE0U)
#define CSL_CSI2_CTX4_DAT_PING_ADDR_ADDR_SHIFT                       (0x00000005U)
#define CSL_CSI2_CTX4_DAT_PING_ADDR_ADDR_RESETVAL                    (0x00000000U)
#define CSL_CSI2_CTX4_DAT_PING_ADDR_ADDR_MAX                         (0x07FFFFFFU)

#define CSL_CSI2_CTX4_DAT_PING_ADDR_RESETVAL                         (0x00000000U)

/* CSI2_CTX4_DAT_PONG_ADDR */

#define CSL_CSI2_CTX4_DAT_PONG_ADDR_RES_MASK                         (0x0000001FU)
#define CSL_CSI2_CTX4_DAT_PONG_ADDR_RES_SHIFT                        (0x00000000U)
#define CSL_CSI2_CTX4_DAT_PONG_ADDR_RES_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX4_DAT_PONG_ADDR_RES_MAX                          (0x0000001FU)

#define CSL_CSI2_CTX4_DAT_PONG_ADDR_ADDR_MASK                        (0xFFFFFFE0U)
#define CSL_CSI2_CTX4_DAT_PONG_ADDR_ADDR_SHIFT                       (0x00000005U)
#define CSL_CSI2_CTX4_DAT_PONG_ADDR_ADDR_RESETVAL                    (0x00000000U)
#define CSL_CSI2_CTX4_DAT_PONG_ADDR_ADDR_MAX                         (0x07FFFFFFU)

#define CSL_CSI2_CTX4_DAT_PONG_ADDR_RESETVAL                         (0x00000000U)

/* CSI2_CTX4_IRQENABLE */

#define CSL_CSI2_CTX4_IRQENABLE_FS_IRQ_MASK                          (0x00000001U)
#define CSL_CSI2_CTX4_IRQENABLE_FS_IRQ_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX4_IRQENABLE_FS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX4_IRQENABLE_FS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX4_IRQENABLE_FE_IRQ_MASK                          (0x00000002U)
#define CSL_CSI2_CTX4_IRQENABLE_FE_IRQ_SHIFT                         (0x00000001U)
#define CSL_CSI2_CTX4_IRQENABLE_FE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX4_IRQENABLE_FE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX4_IRQENABLE_LS_IRQ_MASK                          (0x00000004U)
#define CSL_CSI2_CTX4_IRQENABLE_LS_IRQ_SHIFT                         (0x00000002U)
#define CSL_CSI2_CTX4_IRQENABLE_LS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX4_IRQENABLE_LS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX4_IRQENABLE_LE_IRQ_MASK                          (0x00000008U)
#define CSL_CSI2_CTX4_IRQENABLE_LE_IRQ_SHIFT                         (0x00000003U)
#define CSL_CSI2_CTX4_IRQENABLE_LE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX4_IRQENABLE_LE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX4_IRQENABLE_RES23_MASK                           (0x00000010U)
#define CSL_CSI2_CTX4_IRQENABLE_RES23_SHIFT                          (0x00000004U)
#define CSL_CSI2_CTX4_IRQENABLE_RES23_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX4_IRQENABLE_RES23_MAX                            (0x00000001U)

#define CSL_CSI2_CTX4_IRQENABLE_CS_IRQ_MASK                          (0x00000020U)
#define CSL_CSI2_CTX4_IRQENABLE_CS_IRQ_SHIFT                         (0x00000005U)
#define CSL_CSI2_CTX4_IRQENABLE_CS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX4_IRQENABLE_CS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX4_IRQENABLE_FRAME_NUMBER_IRQ_MASK                (0x00000040U)
#define CSL_CSI2_CTX4_IRQENABLE_FRAME_NUMBER_IRQ_SHIFT               (0x00000006U)
#define CSL_CSI2_CTX4_IRQENABLE_FRAME_NUMBER_IRQ_RESETVAL            (0x00000000U)
#define CSL_CSI2_CTX4_IRQENABLE_FRAME_NUMBER_IRQ_MAX                 (0x00000001U)

#define CSL_CSI2_CTX4_IRQENABLE_LINE_NUMBER_IRQ_MASK                 (0x00000080U)
#define CSL_CSI2_CTX4_IRQENABLE_LINE_NUMBER_IRQ_SHIFT                (0x00000007U)
#define CSL_CSI2_CTX4_IRQENABLE_LINE_NUMBER_IRQ_RESETVAL             (0x00000000U)
#define CSL_CSI2_CTX4_IRQENABLE_LINE_NUMBER_IRQ_MAX                  (0x00000001U)

#define CSL_CSI2_CTX4_IRQENABLE_ECC_CORRECTION_IRQ_MASK              (0x00000100U)
#define CSL_CSI2_CTX4_IRQENABLE_ECC_CORRECTION_IRQ_SHIFT             (0x00000008U)
#define CSL_CSI2_CTX4_IRQENABLE_ECC_CORRECTION_IRQ_RESETVAL          (0x00000000U)
#define CSL_CSI2_CTX4_IRQENABLE_ECC_CORRECTION_IRQ_MAX               (0x00000001U)

#define CSL_CSI2_CTX4_IRQENABLE_RES22_MASK                           (0xFFFFFE00U)
#define CSL_CSI2_CTX4_IRQENABLE_RES22_SHIFT                          (0x00000009U)
#define CSL_CSI2_CTX4_IRQENABLE_RES22_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX4_IRQENABLE_RES22_MAX                            (0x007FFFFFU)

#define CSL_CSI2_CTX4_IRQENABLE_RESETVAL                             (0x00000000U)

/* CSI2_CTX4_IRQSTATUS */

#define CSL_CSI2_CTX4_IRQSTATUS_FS_IRQ_MASK                          (0x00000001U)
#define CSL_CSI2_CTX4_IRQSTATUS_FS_IRQ_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX4_IRQSTATUS_FS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX4_IRQSTATUS_FS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX4_IRQSTATUS_FE_IRQ_MASK                          (0x00000002U)
#define CSL_CSI2_CTX4_IRQSTATUS_FE_IRQ_SHIFT                         (0x00000001U)
#define CSL_CSI2_CTX4_IRQSTATUS_FE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX4_IRQSTATUS_FE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX4_IRQSTATUS_LS_IRQ_MASK                          (0x00000004U)
#define CSL_CSI2_CTX4_IRQSTATUS_LS_IRQ_SHIFT                         (0x00000002U)
#define CSL_CSI2_CTX4_IRQSTATUS_LS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX4_IRQSTATUS_LS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX4_IRQSTATUS_LE_IRQ_MASK                          (0x00000008U)
#define CSL_CSI2_CTX4_IRQSTATUS_LE_IRQ_SHIFT                         (0x00000003U)
#define CSL_CSI2_CTX4_IRQSTATUS_LE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX4_IRQSTATUS_LE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX4_IRQSTATUS_RES25_MASK                           (0x00000010U)
#define CSL_CSI2_CTX4_IRQSTATUS_RES25_SHIFT                          (0x00000004U)
#define CSL_CSI2_CTX4_IRQSTATUS_RES25_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX4_IRQSTATUS_RES25_MAX                            (0x00000001U)

#define CSL_CSI2_CTX4_IRQSTATUS_CS_IRQ_MASK                          (0x00000020U)
#define CSL_CSI2_CTX4_IRQSTATUS_CS_IRQ_SHIFT                         (0x00000005U)
#define CSL_CSI2_CTX4_IRQSTATUS_CS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX4_IRQSTATUS_CS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX4_IRQSTATUS_FRAME_NUMBER_IRQ_MASK                (0x00000040U)
#define CSL_CSI2_CTX4_IRQSTATUS_FRAME_NUMBER_IRQ_SHIFT               (0x00000006U)
#define CSL_CSI2_CTX4_IRQSTATUS_FRAME_NUMBER_IRQ_RESETVAL            (0x00000000U)
#define CSL_CSI2_CTX4_IRQSTATUS_FRAME_NUMBER_IRQ_MAX                 (0x00000001U)

#define CSL_CSI2_CTX4_IRQSTATUS_LINE_NUMBER_IRQ_MASK                 (0x00000080U)
#define CSL_CSI2_CTX4_IRQSTATUS_LINE_NUMBER_IRQ_SHIFT                (0x00000007U)
#define CSL_CSI2_CTX4_IRQSTATUS_LINE_NUMBER_IRQ_RESETVAL             (0x00000000U)
#define CSL_CSI2_CTX4_IRQSTATUS_LINE_NUMBER_IRQ_MAX                  (0x00000001U)

#define CSL_CSI2_CTX4_IRQSTATUS_ECC_CORRECTION_IRQ_MASK              (0x00000100U)
#define CSL_CSI2_CTX4_IRQSTATUS_ECC_CORRECTION_IRQ_SHIFT             (0x00000008U)
#define CSL_CSI2_CTX4_IRQSTATUS_ECC_CORRECTION_IRQ_RESETVAL          (0x00000000U)
#define CSL_CSI2_CTX4_IRQSTATUS_ECC_CORRECTION_IRQ_MAX               (0x00000001U)

#define CSL_CSI2_CTX4_IRQSTATUS_RES24_MASK                           (0xFFFFFE00U)
#define CSL_CSI2_CTX4_IRQSTATUS_RES24_SHIFT                          (0x00000009U)
#define CSL_CSI2_CTX4_IRQSTATUS_RES24_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX4_IRQSTATUS_RES24_MAX                            (0x007FFFFFU)

#define CSL_CSI2_CTX4_IRQSTATUS_RESETVAL                             (0x00000000U)

/* CSI2_CTX4_CTRL3 */

#define CSL_CSI2_CTX4_CTRL3_LINE_NUMBER_MASK                         (0x0000FFFFU)
#define CSL_CSI2_CTX4_CTRL3_LINE_NUMBER_SHIFT                        (0x00000000U)
#define CSL_CSI2_CTX4_CTRL3_LINE_NUMBER_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX4_CTRL3_LINE_NUMBER_MAX                          (0x0000FFFFU)

#define CSL_CSI2_CTX4_CTRL3_ALPHA_MASK                               (0x3FFF0000U)
#define CSL_CSI2_CTX4_CTRL3_ALPHA_SHIFT                              (0x00000010U)
#define CSL_CSI2_CTX4_CTRL3_ALPHA_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX4_CTRL3_ALPHA_MAX                                (0x00003FFFU)

#define CSL_CSI2_CTX4_CTRL3_RESERVED_MASK                            (0xC0000000U)
#define CSL_CSI2_CTX4_CTRL3_RESERVED_SHIFT                           (0x0000001EU)
#define CSL_CSI2_CTX4_CTRL3_RESERVED_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX4_CTRL3_RESERVED_MAX                             (0x00000003U)

#define CSL_CSI2_CTX4_CTRL3_RESETVAL                                 (0x00000000U)

/* CSI2_CTX5_CTRL1 */

#define CSL_CSI2_CTX5_CTRL1_CTX_EN_MASK                              (0x00000001U)
#define CSL_CSI2_CTX5_CTRL1_CTX_EN_SHIFT                             (0x00000000U)
#define CSL_CSI2_CTX5_CTRL1_CTX_EN_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX5_CTRL1_CTX_EN_MAX                               (0x00000001U)

#define CSL_CSI2_CTX5_CTRL1_LINE_MODULO_MASK                         (0x00000002U)
#define CSL_CSI2_CTX5_CTRL1_LINE_MODULO_SHIFT                        (0x00000001U)
#define CSL_CSI2_CTX5_CTRL1_LINE_MODULO_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX5_CTRL1_LINE_MODULO_MAX                          (0x00000001U)

#define CSL_CSI2_CTX5_CTRL1_VP_FORCE_MASK                            (0x00000004U)
#define CSL_CSI2_CTX5_CTRL1_VP_FORCE_SHIFT                           (0x00000002U)
#define CSL_CSI2_CTX5_CTRL1_VP_FORCE_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX5_CTRL1_VP_FORCE_MAX                             (0x00000001U)

#define CSL_CSI2_CTX5_CTRL1_PING_PONG_MASK                           (0x00000008U)
#define CSL_CSI2_CTX5_CTRL1_PING_PONG_SHIFT                          (0x00000003U)
#define CSL_CSI2_CTX5_CTRL1_PING_PONG_RESETVAL                       (0x00000001U)
#define CSL_CSI2_CTX5_CTRL1_PING_PONG_MAX                            (0x00000001U)

#define CSL_CSI2_CTX5_CTRL1_COUNT_UNLOCK_MASK                        (0x00000010U)
#define CSL_CSI2_CTX5_CTRL1_COUNT_UNLOCK_SHIFT                       (0x00000004U)
#define CSL_CSI2_CTX5_CTRL1_COUNT_UNLOCK_RESETVAL                    (0x00000000U)
#define CSL_CSI2_CTX5_CTRL1_COUNT_UNLOCK_MAX                         (0x00000001U)

#define CSL_CSI2_CTX5_CTRL1_CS_EN_MASK                               (0x00000020U)
#define CSL_CSI2_CTX5_CTRL1_CS_EN_SHIFT                              (0x00000005U)
#define CSL_CSI2_CTX5_CTRL1_CS_EN_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX5_CTRL1_CS_EN_MAX                                (0x00000001U)

#define CSL_CSI2_CTX5_CTRL1_EOL_EN_MASK                              (0x00000040U)
#define CSL_CSI2_CTX5_CTRL1_EOL_EN_SHIFT                             (0x00000006U)
#define CSL_CSI2_CTX5_CTRL1_EOL_EN_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX5_CTRL1_EOL_EN_MAX                               (0x00000001U)

#define CSL_CSI2_CTX5_CTRL1_EOF_EN_MASK                              (0x00000080U)
#define CSL_CSI2_CTX5_CTRL1_EOF_EN_SHIFT                             (0x00000007U)
#define CSL_CSI2_CTX5_CTRL1_EOF_EN_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX5_CTRL1_EOF_EN_MAX                               (0x00000001U)

#define CSL_CSI2_CTX5_CTRL1_COUNT_MASK                               (0x0000FF00U)
#define CSL_CSI2_CTX5_CTRL1_COUNT_SHIFT                              (0x00000008U)
#define CSL_CSI2_CTX5_CTRL1_COUNT_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX5_CTRL1_COUNT_MAX                                (0x000000FFU)

#define CSL_CSI2_CTX5_CTRL1_FEC_NUMBER_MASK                          (0x00FF0000U)
#define CSL_CSI2_CTX5_CTRL1_FEC_NUMBER_SHIFT                         (0x00000010U)
#define CSL_CSI2_CTX5_CTRL1_FEC_NUMBER_RESETVAL                      (0x00000001U)
#define CSL_CSI2_CTX5_CTRL1_FEC_NUMBER_MAX                           (0x000000FFU)

#define CSL_CSI2_CTX5_CTRL1_TRANSCODE_MASK                           (0x0F000000U)
#define CSL_CSI2_CTX5_CTRL1_TRANSCODE_SHIFT                          (0x00000018U)
#define CSL_CSI2_CTX5_CTRL1_TRANSCODE_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX5_CTRL1_TRANSCODE_MAX                            (0x0000000FU)

#define CSL_CSI2_CTX5_CTRL1_HSCALE_MASK                              (0x10000000U)
#define CSL_CSI2_CTX5_CTRL1_HSCALE_SHIFT                             (0x0000001CU)
#define CSL_CSI2_CTX5_CTRL1_HSCALE_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX5_CTRL1_HSCALE_MAX                               (0x00000001U)

#define CSL_CSI2_CTX5_CTRL1_RES19_MASK                               (0x20000000U)
#define CSL_CSI2_CTX5_CTRL1_RES19_SHIFT                              (0x0000001DU)
#define CSL_CSI2_CTX5_CTRL1_RES19_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX5_CTRL1_RES19_MAX                                (0x00000001U)

#define CSL_CSI2_CTX5_CTRL1_GENERIC_MASK                             (0x40000000U)
#define CSL_CSI2_CTX5_CTRL1_GENERIC_SHIFT                            (0x0000001EU)
#define CSL_CSI2_CTX5_CTRL1_GENERIC_RESETVAL                         (0x00000000U)
#define CSL_CSI2_CTX5_CTRL1_GENERIC_MAX                              (0x00000001U)

#define CSL_CSI2_CTX5_CTRL1_BYTESWAP_MASK                            (0x80000000U)
#define CSL_CSI2_CTX5_CTRL1_BYTESWAP_SHIFT                           (0x0000001FU)
#define CSL_CSI2_CTX5_CTRL1_BYTESWAP_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX5_CTRL1_BYTESWAP_MAX                             (0x00000001U)

#define CSL_CSI2_CTX5_CTRL1_RESETVAL                                 (0x00010008U)

/* CSI2_CTX5_CTRL2 */

#define CSL_CSI2_CTX5_CTRL2_FORMAT_MASK                              (0x000003FFU)
#define CSL_CSI2_CTX5_CTRL2_FORMAT_SHIFT                             (0x00000000U)
#define CSL_CSI2_CTX5_CTRL2_FORMAT_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX5_CTRL2_FORMAT_MAX                               (0x000003FFU)

#define CSL_CSI2_CTX5_CTRL2_DPCM_PRED_MASK                           (0x00000400U)
#define CSL_CSI2_CTX5_CTRL2_DPCM_PRED_SHIFT                          (0x0000000AU)
#define CSL_CSI2_CTX5_CTRL2_DPCM_PRED_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX5_CTRL2_DPCM_PRED_MAX                            (0x00000001U)

#define CSL_CSI2_CTX5_CTRL2_VIRTUAL_ID_MASK                          (0x00001800U)
#define CSL_CSI2_CTX5_CTRL2_VIRTUAL_ID_SHIFT                         (0x0000000BU)
#define CSL_CSI2_CTX5_CTRL2_VIRTUAL_ID_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX5_CTRL2_VIRTUAL_ID_MAX                           (0x00000003U)

#define CSL_CSI2_CTX5_CTRL2_USER_DEF_MAPPING_MASK                    (0x00006000U)
#define CSL_CSI2_CTX5_CTRL2_USER_DEF_MAPPING_SHIFT                   (0x0000000DU)
#define CSL_CSI2_CTX5_CTRL2_USER_DEF_MAPPING_RESETVAL                (0x00000000U)
#define CSL_CSI2_CTX5_CTRL2_USER_DEF_MAPPING_MAX                     (0x00000003U)

#define CSL_CSI2_CTX5_CTRL2_RES20_MASK                               (0x00008000U)
#define CSL_CSI2_CTX5_CTRL2_RES20_SHIFT                              (0x0000000FU)
#define CSL_CSI2_CTX5_CTRL2_RES20_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX5_CTRL2_RES20_MAX                                (0x00000001U)

#define CSL_CSI2_CTX5_CTRL2_FRAME_MASK                               (0xFFFF0000U)
#define CSL_CSI2_CTX5_CTRL2_FRAME_SHIFT                              (0x00000010U)
#define CSL_CSI2_CTX5_CTRL2_FRAME_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX5_CTRL2_FRAME_MAX                                (0x0000FFFFU)

#define CSL_CSI2_CTX5_CTRL2_RESETVAL                                 (0x00000000U)

/* CSI2_CTX5_DAT_OFST */

#define CSL_CSI2_CTX5_DAT_OFST_OFST_MASK                             (0x0001FFE0U)
#define CSL_CSI2_CTX5_DAT_OFST_OFST_SHIFT                            (0x00000005U)
#define CSL_CSI2_CTX5_DAT_OFST_OFST_RESETVAL                         (0x00000000U)
#define CSL_CSI2_CTX5_DAT_OFST_OFST_MAX                              (0x00000FFFU)

#define CSL_CSI2_CTX5_DAT_OFST_RES21_MASK                            (0xFFFE0000U)
#define CSL_CSI2_CTX5_DAT_OFST_RES21_SHIFT                           (0x00000011U)
#define CSL_CSI2_CTX5_DAT_OFST_RES21_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX5_DAT_OFST_RES21_MAX                             (0x00007FFFU)

#define CSL_CSI2_CTX5_DAT_OFST_RESETVAL                              (0x00000000U)

/* CSI2_CTX5_DAT_PING_ADDR */

#define CSL_CSI2_CTX5_DAT_PING_ADDR_RES_MASK                         (0x0000001FU)
#define CSL_CSI2_CTX5_DAT_PING_ADDR_RES_SHIFT                        (0x00000000U)
#define CSL_CSI2_CTX5_DAT_PING_ADDR_RES_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX5_DAT_PING_ADDR_RES_MAX                          (0x0000001FU)

#define CSL_CSI2_CTX5_DAT_PING_ADDR_ADDR_MASK                        (0xFFFFFFE0U)
#define CSL_CSI2_CTX5_DAT_PING_ADDR_ADDR_SHIFT                       (0x00000005U)
#define CSL_CSI2_CTX5_DAT_PING_ADDR_ADDR_RESETVAL                    (0x00000000U)
#define CSL_CSI2_CTX5_DAT_PING_ADDR_ADDR_MAX                         (0x07FFFFFFU)

#define CSL_CSI2_CTX5_DAT_PING_ADDR_RESETVAL                         (0x00000000U)

/* CSI2_CTX5_DAT_PONG_ADDR */

#define CSL_CSI2_CTX5_DAT_PONG_ADDR_RES_MASK                         (0x0000001FU)
#define CSL_CSI2_CTX5_DAT_PONG_ADDR_RES_SHIFT                        (0x00000000U)
#define CSL_CSI2_CTX5_DAT_PONG_ADDR_RES_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX5_DAT_PONG_ADDR_RES_MAX                          (0x0000001FU)

#define CSL_CSI2_CTX5_DAT_PONG_ADDR_ADDR_MASK                        (0xFFFFFFE0U)
#define CSL_CSI2_CTX5_DAT_PONG_ADDR_ADDR_SHIFT                       (0x00000005U)
#define CSL_CSI2_CTX5_DAT_PONG_ADDR_ADDR_RESETVAL                    (0x00000000U)
#define CSL_CSI2_CTX5_DAT_PONG_ADDR_ADDR_MAX                         (0x07FFFFFFU)

#define CSL_CSI2_CTX5_DAT_PONG_ADDR_RESETVAL                         (0x00000000U)

/* CSI2_CTX5_IRQENABLE */

#define CSL_CSI2_CTX5_IRQENABLE_FS_IRQ_MASK                          (0x00000001U)
#define CSL_CSI2_CTX5_IRQENABLE_FS_IRQ_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX5_IRQENABLE_FS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX5_IRQENABLE_FS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX5_IRQENABLE_FE_IRQ_MASK                          (0x00000002U)
#define CSL_CSI2_CTX5_IRQENABLE_FE_IRQ_SHIFT                         (0x00000001U)
#define CSL_CSI2_CTX5_IRQENABLE_FE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX5_IRQENABLE_FE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX5_IRQENABLE_LS_IRQ_MASK                          (0x00000004U)
#define CSL_CSI2_CTX5_IRQENABLE_LS_IRQ_SHIFT                         (0x00000002U)
#define CSL_CSI2_CTX5_IRQENABLE_LS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX5_IRQENABLE_LS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX5_IRQENABLE_LE_IRQ_MASK                          (0x00000008U)
#define CSL_CSI2_CTX5_IRQENABLE_LE_IRQ_SHIFT                         (0x00000003U)
#define CSL_CSI2_CTX5_IRQENABLE_LE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX5_IRQENABLE_LE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX5_IRQENABLE_RES23_MASK                           (0x00000010U)
#define CSL_CSI2_CTX5_IRQENABLE_RES23_SHIFT                          (0x00000004U)
#define CSL_CSI2_CTX5_IRQENABLE_RES23_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX5_IRQENABLE_RES23_MAX                            (0x00000001U)

#define CSL_CSI2_CTX5_IRQENABLE_CS_IRQ_MASK                          (0x00000020U)
#define CSL_CSI2_CTX5_IRQENABLE_CS_IRQ_SHIFT                         (0x00000005U)
#define CSL_CSI2_CTX5_IRQENABLE_CS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX5_IRQENABLE_CS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX5_IRQENABLE_FRAME_NUMBER_IRQ_MASK                (0x00000040U)
#define CSL_CSI2_CTX5_IRQENABLE_FRAME_NUMBER_IRQ_SHIFT               (0x00000006U)
#define CSL_CSI2_CTX5_IRQENABLE_FRAME_NUMBER_IRQ_RESETVAL            (0x00000000U)
#define CSL_CSI2_CTX5_IRQENABLE_FRAME_NUMBER_IRQ_MAX                 (0x00000001U)

#define CSL_CSI2_CTX5_IRQENABLE_LINE_NUMBER_IRQ_MASK                 (0x00000080U)
#define CSL_CSI2_CTX5_IRQENABLE_LINE_NUMBER_IRQ_SHIFT                (0x00000007U)
#define CSL_CSI2_CTX5_IRQENABLE_LINE_NUMBER_IRQ_RESETVAL             (0x00000000U)
#define CSL_CSI2_CTX5_IRQENABLE_LINE_NUMBER_IRQ_MAX                  (0x00000001U)

#define CSL_CSI2_CTX5_IRQENABLE_ECC_CORRECTION_IRQ_MASK              (0x00000100U)
#define CSL_CSI2_CTX5_IRQENABLE_ECC_CORRECTION_IRQ_SHIFT             (0x00000008U)
#define CSL_CSI2_CTX5_IRQENABLE_ECC_CORRECTION_IRQ_RESETVAL          (0x00000000U)
#define CSL_CSI2_CTX5_IRQENABLE_ECC_CORRECTION_IRQ_MAX               (0x00000001U)

#define CSL_CSI2_CTX5_IRQENABLE_RES22_MASK                           (0xFFFFFE00U)
#define CSL_CSI2_CTX5_IRQENABLE_RES22_SHIFT                          (0x00000009U)
#define CSL_CSI2_CTX5_IRQENABLE_RES22_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX5_IRQENABLE_RES22_MAX                            (0x007FFFFFU)

#define CSL_CSI2_CTX5_IRQENABLE_RESETVAL                             (0x00000000U)

/* CSI2_CTX5_IRQSTATUS */

#define CSL_CSI2_CTX5_IRQSTATUS_FS_IRQ_MASK                          (0x00000001U)
#define CSL_CSI2_CTX5_IRQSTATUS_FS_IRQ_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX5_IRQSTATUS_FS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX5_IRQSTATUS_FS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX5_IRQSTATUS_FE_IRQ_MASK                          (0x00000002U)
#define CSL_CSI2_CTX5_IRQSTATUS_FE_IRQ_SHIFT                         (0x00000001U)
#define CSL_CSI2_CTX5_IRQSTATUS_FE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX5_IRQSTATUS_FE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX5_IRQSTATUS_LS_IRQ_MASK                          (0x00000004U)
#define CSL_CSI2_CTX5_IRQSTATUS_LS_IRQ_SHIFT                         (0x00000002U)
#define CSL_CSI2_CTX5_IRQSTATUS_LS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX5_IRQSTATUS_LS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX5_IRQSTATUS_LE_IRQ_MASK                          (0x00000008U)
#define CSL_CSI2_CTX5_IRQSTATUS_LE_IRQ_SHIFT                         (0x00000003U)
#define CSL_CSI2_CTX5_IRQSTATUS_LE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX5_IRQSTATUS_LE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX5_IRQSTATUS_RES25_MASK                           (0x00000010U)
#define CSL_CSI2_CTX5_IRQSTATUS_RES25_SHIFT                          (0x00000004U)
#define CSL_CSI2_CTX5_IRQSTATUS_RES25_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX5_IRQSTATUS_RES25_MAX                            (0x00000001U)

#define CSL_CSI2_CTX5_IRQSTATUS_CS_IRQ_MASK                          (0x00000020U)
#define CSL_CSI2_CTX5_IRQSTATUS_CS_IRQ_SHIFT                         (0x00000005U)
#define CSL_CSI2_CTX5_IRQSTATUS_CS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX5_IRQSTATUS_CS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX5_IRQSTATUS_FRAME_NUMBER_IRQ_MASK                (0x00000040U)
#define CSL_CSI2_CTX5_IRQSTATUS_FRAME_NUMBER_IRQ_SHIFT               (0x00000006U)
#define CSL_CSI2_CTX5_IRQSTATUS_FRAME_NUMBER_IRQ_RESETVAL            (0x00000000U)
#define CSL_CSI2_CTX5_IRQSTATUS_FRAME_NUMBER_IRQ_MAX                 (0x00000001U)

#define CSL_CSI2_CTX5_IRQSTATUS_LINE_NUMBER_IRQ_MASK                 (0x00000080U)
#define CSL_CSI2_CTX5_IRQSTATUS_LINE_NUMBER_IRQ_SHIFT                (0x00000007U)
#define CSL_CSI2_CTX5_IRQSTATUS_LINE_NUMBER_IRQ_RESETVAL             (0x00000000U)
#define CSL_CSI2_CTX5_IRQSTATUS_LINE_NUMBER_IRQ_MAX                  (0x00000001U)

#define CSL_CSI2_CTX5_IRQSTATUS_ECC_CORRECTION_IRQ_MASK              (0x00000100U)
#define CSL_CSI2_CTX5_IRQSTATUS_ECC_CORRECTION_IRQ_SHIFT             (0x00000008U)
#define CSL_CSI2_CTX5_IRQSTATUS_ECC_CORRECTION_IRQ_RESETVAL          (0x00000000U)
#define CSL_CSI2_CTX5_IRQSTATUS_ECC_CORRECTION_IRQ_MAX               (0x00000001U)

#define CSL_CSI2_CTX5_IRQSTATUS_RES24_MASK                           (0xFFFFFE00U)
#define CSL_CSI2_CTX5_IRQSTATUS_RES24_SHIFT                          (0x00000009U)
#define CSL_CSI2_CTX5_IRQSTATUS_RES24_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX5_IRQSTATUS_RES24_MAX                            (0x007FFFFFU)

#define CSL_CSI2_CTX5_IRQSTATUS_RESETVAL                             (0x00000000U)

/* CSI2_CTX5_CTRL3 */

#define CSL_CSI2_CTX5_CTRL3_LINE_NUMBER_MASK                         (0x0000FFFFU)
#define CSL_CSI2_CTX5_CTRL3_LINE_NUMBER_SHIFT                        (0x00000000U)
#define CSL_CSI2_CTX5_CTRL3_LINE_NUMBER_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX5_CTRL3_LINE_NUMBER_MAX                          (0x0000FFFFU)

#define CSL_CSI2_CTX5_CTRL3_ALPHA_MASK                               (0x3FFF0000U)
#define CSL_CSI2_CTX5_CTRL3_ALPHA_SHIFT                              (0x00000010U)
#define CSL_CSI2_CTX5_CTRL3_ALPHA_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX5_CTRL3_ALPHA_MAX                                (0x00003FFFU)

#define CSL_CSI2_CTX5_CTRL3_RESERVED_MASK                            (0xC0000000U)
#define CSL_CSI2_CTX5_CTRL3_RESERVED_SHIFT                           (0x0000001EU)
#define CSL_CSI2_CTX5_CTRL3_RESERVED_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX5_CTRL3_RESERVED_MAX                             (0x00000003U)

#define CSL_CSI2_CTX5_CTRL3_RESETVAL                                 (0x00000000U)

/* CSI2_CTX6_CTRL1 */

#define CSL_CSI2_CTX6_CTRL1_CTX_EN_MASK                              (0x00000001U)
#define CSL_CSI2_CTX6_CTRL1_CTX_EN_SHIFT                             (0x00000000U)
#define CSL_CSI2_CTX6_CTRL1_CTX_EN_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX6_CTRL1_CTX_EN_MAX                               (0x00000001U)

#define CSL_CSI2_CTX6_CTRL1_LINE_MODULO_MASK                         (0x00000002U)
#define CSL_CSI2_CTX6_CTRL1_LINE_MODULO_SHIFT                        (0x00000001U)
#define CSL_CSI2_CTX6_CTRL1_LINE_MODULO_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX6_CTRL1_LINE_MODULO_MAX                          (0x00000001U)

#define CSL_CSI2_CTX6_CTRL1_VP_FORCE_MASK                            (0x00000004U)
#define CSL_CSI2_CTX6_CTRL1_VP_FORCE_SHIFT                           (0x00000002U)
#define CSL_CSI2_CTX6_CTRL1_VP_FORCE_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX6_CTRL1_VP_FORCE_MAX                             (0x00000001U)

#define CSL_CSI2_CTX6_CTRL1_PING_PONG_MASK                           (0x00000008U)
#define CSL_CSI2_CTX6_CTRL1_PING_PONG_SHIFT                          (0x00000003U)
#define CSL_CSI2_CTX6_CTRL1_PING_PONG_RESETVAL                       (0x00000001U)
#define CSL_CSI2_CTX6_CTRL1_PING_PONG_MAX                            (0x00000001U)

#define CSL_CSI2_CTX6_CTRL1_COUNT_UNLOCK_MASK                        (0x00000010U)
#define CSL_CSI2_CTX6_CTRL1_COUNT_UNLOCK_SHIFT                       (0x00000004U)
#define CSL_CSI2_CTX6_CTRL1_COUNT_UNLOCK_RESETVAL                    (0x00000000U)
#define CSL_CSI2_CTX6_CTRL1_COUNT_UNLOCK_MAX                         (0x00000001U)

#define CSL_CSI2_CTX6_CTRL1_CS_EN_MASK                               (0x00000020U)
#define CSL_CSI2_CTX6_CTRL1_CS_EN_SHIFT                              (0x00000005U)
#define CSL_CSI2_CTX6_CTRL1_CS_EN_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX6_CTRL1_CS_EN_MAX                                (0x00000001U)

#define CSL_CSI2_CTX6_CTRL1_EOL_EN_MASK                              (0x00000040U)
#define CSL_CSI2_CTX6_CTRL1_EOL_EN_SHIFT                             (0x00000006U)
#define CSL_CSI2_CTX6_CTRL1_EOL_EN_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX6_CTRL1_EOL_EN_MAX                               (0x00000001U)

#define CSL_CSI2_CTX6_CTRL1_EOF_EN_MASK                              (0x00000080U)
#define CSL_CSI2_CTX6_CTRL1_EOF_EN_SHIFT                             (0x00000007U)
#define CSL_CSI2_CTX6_CTRL1_EOF_EN_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX6_CTRL1_EOF_EN_MAX                               (0x00000001U)

#define CSL_CSI2_CTX6_CTRL1_COUNT_MASK                               (0x0000FF00U)
#define CSL_CSI2_CTX6_CTRL1_COUNT_SHIFT                              (0x00000008U)
#define CSL_CSI2_CTX6_CTRL1_COUNT_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX6_CTRL1_COUNT_MAX                                (0x000000FFU)

#define CSL_CSI2_CTX6_CTRL1_FEC_NUMBER_MASK                          (0x00FF0000U)
#define CSL_CSI2_CTX6_CTRL1_FEC_NUMBER_SHIFT                         (0x00000010U)
#define CSL_CSI2_CTX6_CTRL1_FEC_NUMBER_RESETVAL                      (0x00000001U)
#define CSL_CSI2_CTX6_CTRL1_FEC_NUMBER_MAX                           (0x000000FFU)

#define CSL_CSI2_CTX6_CTRL1_TRANSCODE_MASK                           (0x0F000000U)
#define CSL_CSI2_CTX6_CTRL1_TRANSCODE_SHIFT                          (0x00000018U)
#define CSL_CSI2_CTX6_CTRL1_TRANSCODE_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX6_CTRL1_TRANSCODE_MAX                            (0x0000000FU)

#define CSL_CSI2_CTX6_CTRL1_HSCALE_MASK                              (0x10000000U)
#define CSL_CSI2_CTX6_CTRL1_HSCALE_SHIFT                             (0x0000001CU)
#define CSL_CSI2_CTX6_CTRL1_HSCALE_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX6_CTRL1_HSCALE_MAX                               (0x00000001U)

#define CSL_CSI2_CTX6_CTRL1_RES19_MASK                               (0x20000000U)
#define CSL_CSI2_CTX6_CTRL1_RES19_SHIFT                              (0x0000001DU)
#define CSL_CSI2_CTX6_CTRL1_RES19_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX6_CTRL1_RES19_MAX                                (0x00000001U)

#define CSL_CSI2_CTX6_CTRL1_GENERIC_MASK                             (0x40000000U)
#define CSL_CSI2_CTX6_CTRL1_GENERIC_SHIFT                            (0x0000001EU)
#define CSL_CSI2_CTX6_CTRL1_GENERIC_RESETVAL                         (0x00000000U)
#define CSL_CSI2_CTX6_CTRL1_GENERIC_MAX                              (0x00000001U)

#define CSL_CSI2_CTX6_CTRL1_BYTESWAP_MASK                            (0x80000000U)
#define CSL_CSI2_CTX6_CTRL1_BYTESWAP_SHIFT                           (0x0000001FU)
#define CSL_CSI2_CTX6_CTRL1_BYTESWAP_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX6_CTRL1_BYTESWAP_MAX                             (0x00000001U)

#define CSL_CSI2_CTX6_CTRL1_RESETVAL                                 (0x00010008U)

/* CSI2_CTX6_CTRL2 */

#define CSL_CSI2_CTX6_CTRL2_FORMAT_MASK                              (0x000003FFU)
#define CSL_CSI2_CTX6_CTRL2_FORMAT_SHIFT                             (0x00000000U)
#define CSL_CSI2_CTX6_CTRL2_FORMAT_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX6_CTRL2_FORMAT_MAX                               (0x000003FFU)

#define CSL_CSI2_CTX6_CTRL2_DPCM_PRED_MASK                           (0x00000400U)
#define CSL_CSI2_CTX6_CTRL2_DPCM_PRED_SHIFT                          (0x0000000AU)
#define CSL_CSI2_CTX6_CTRL2_DPCM_PRED_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX6_CTRL2_DPCM_PRED_MAX                            (0x00000001U)

#define CSL_CSI2_CTX6_CTRL2_VIRTUAL_ID_MASK                          (0x00001800U)
#define CSL_CSI2_CTX6_CTRL2_VIRTUAL_ID_SHIFT                         (0x0000000BU)
#define CSL_CSI2_CTX6_CTRL2_VIRTUAL_ID_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX6_CTRL2_VIRTUAL_ID_MAX                           (0x00000003U)

#define CSL_CSI2_CTX6_CTRL2_USER_DEF_MAPPING_MASK                    (0x00006000U)
#define CSL_CSI2_CTX6_CTRL2_USER_DEF_MAPPING_SHIFT                   (0x0000000DU)
#define CSL_CSI2_CTX6_CTRL2_USER_DEF_MAPPING_RESETVAL                (0x00000000U)
#define CSL_CSI2_CTX6_CTRL2_USER_DEF_MAPPING_MAX                     (0x00000003U)

#define CSL_CSI2_CTX6_CTRL2_RES20_MASK                               (0x00008000U)
#define CSL_CSI2_CTX6_CTRL2_RES20_SHIFT                              (0x0000000FU)
#define CSL_CSI2_CTX6_CTRL2_RES20_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX6_CTRL2_RES20_MAX                                (0x00000001U)

#define CSL_CSI2_CTX6_CTRL2_FRAME_MASK                               (0xFFFF0000U)
#define CSL_CSI2_CTX6_CTRL2_FRAME_SHIFT                              (0x00000010U)
#define CSL_CSI2_CTX6_CTRL2_FRAME_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX6_CTRL2_FRAME_MAX                                (0x0000FFFFU)

#define CSL_CSI2_CTX6_CTRL2_RESETVAL                                 (0x00000000U)

/* CSI2_CTX6_DAT_OFST */

#define CSL_CSI2_CTX6_DAT_OFST_OFST_MASK                             (0x0001FFE0U)
#define CSL_CSI2_CTX6_DAT_OFST_OFST_SHIFT                            (0x00000005U)
#define CSL_CSI2_CTX6_DAT_OFST_OFST_RESETVAL                         (0x00000000U)
#define CSL_CSI2_CTX6_DAT_OFST_OFST_MAX                              (0x00000FFFU)

#define CSL_CSI2_CTX6_DAT_OFST_RES21_MASK                            (0xFFFE0000U)
#define CSL_CSI2_CTX6_DAT_OFST_RES21_SHIFT                           (0x00000011U)
#define CSL_CSI2_CTX6_DAT_OFST_RES21_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX6_DAT_OFST_RES21_MAX                             (0x00007FFFU)

#define CSL_CSI2_CTX6_DAT_OFST_RESETVAL                              (0x00000000U)

/* CSI2_CTX6_DAT_PING_ADDR */

#define CSL_CSI2_CTX6_DAT_PING_ADDR_RES_MASK                         (0x0000001FU)
#define CSL_CSI2_CTX6_DAT_PING_ADDR_RES_SHIFT                        (0x00000000U)
#define CSL_CSI2_CTX6_DAT_PING_ADDR_RES_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX6_DAT_PING_ADDR_RES_MAX                          (0x0000001FU)

#define CSL_CSI2_CTX6_DAT_PING_ADDR_ADDR_MASK                        (0xFFFFFFE0U)
#define CSL_CSI2_CTX6_DAT_PING_ADDR_ADDR_SHIFT                       (0x00000005U)
#define CSL_CSI2_CTX6_DAT_PING_ADDR_ADDR_RESETVAL                    (0x00000000U)
#define CSL_CSI2_CTX6_DAT_PING_ADDR_ADDR_MAX                         (0x07FFFFFFU)

#define CSL_CSI2_CTX6_DAT_PING_ADDR_RESETVAL                         (0x00000000U)

/* CSI2_CTX6_DAT_PONG_ADDR */

#define CSL_CSI2_CTX6_DAT_PONG_ADDR_RES_MASK                         (0x0000001FU)
#define CSL_CSI2_CTX6_DAT_PONG_ADDR_RES_SHIFT                        (0x00000000U)
#define CSL_CSI2_CTX6_DAT_PONG_ADDR_RES_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX6_DAT_PONG_ADDR_RES_MAX                          (0x0000001FU)

#define CSL_CSI2_CTX6_DAT_PONG_ADDR_ADDR_MASK                        (0xFFFFFFE0U)
#define CSL_CSI2_CTX6_DAT_PONG_ADDR_ADDR_SHIFT                       (0x00000005U)
#define CSL_CSI2_CTX6_DAT_PONG_ADDR_ADDR_RESETVAL                    (0x00000000U)
#define CSL_CSI2_CTX6_DAT_PONG_ADDR_ADDR_MAX                         (0x07FFFFFFU)

#define CSL_CSI2_CTX6_DAT_PONG_ADDR_RESETVAL                         (0x00000000U)

/* CSI2_CTX6_IRQENABLE */

#define CSL_CSI2_CTX6_IRQENABLE_FS_IRQ_MASK                          (0x00000001U)
#define CSL_CSI2_CTX6_IRQENABLE_FS_IRQ_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX6_IRQENABLE_FS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX6_IRQENABLE_FS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX6_IRQENABLE_FE_IRQ_MASK                          (0x00000002U)
#define CSL_CSI2_CTX6_IRQENABLE_FE_IRQ_SHIFT                         (0x00000001U)
#define CSL_CSI2_CTX6_IRQENABLE_FE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX6_IRQENABLE_FE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX6_IRQENABLE_LS_IRQ_MASK                          (0x00000004U)
#define CSL_CSI2_CTX6_IRQENABLE_LS_IRQ_SHIFT                         (0x00000002U)
#define CSL_CSI2_CTX6_IRQENABLE_LS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX6_IRQENABLE_LS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX6_IRQENABLE_LE_IRQ_MASK                          (0x00000008U)
#define CSL_CSI2_CTX6_IRQENABLE_LE_IRQ_SHIFT                         (0x00000003U)
#define CSL_CSI2_CTX6_IRQENABLE_LE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX6_IRQENABLE_LE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX6_IRQENABLE_RES23_MASK                           (0x00000010U)
#define CSL_CSI2_CTX6_IRQENABLE_RES23_SHIFT                          (0x00000004U)
#define CSL_CSI2_CTX6_IRQENABLE_RES23_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX6_IRQENABLE_RES23_MAX                            (0x00000001U)

#define CSL_CSI2_CTX6_IRQENABLE_CS_IRQ_MASK                          (0x00000020U)
#define CSL_CSI2_CTX6_IRQENABLE_CS_IRQ_SHIFT                         (0x00000005U)
#define CSL_CSI2_CTX6_IRQENABLE_CS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX6_IRQENABLE_CS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX6_IRQENABLE_FRAME_NUMBER_IRQ_MASK                (0x00000040U)
#define CSL_CSI2_CTX6_IRQENABLE_FRAME_NUMBER_IRQ_SHIFT               (0x00000006U)
#define CSL_CSI2_CTX6_IRQENABLE_FRAME_NUMBER_IRQ_RESETVAL            (0x00000000U)
#define CSL_CSI2_CTX6_IRQENABLE_FRAME_NUMBER_IRQ_MAX                 (0x00000001U)

#define CSL_CSI2_CTX6_IRQENABLE_LINE_NUMBER_IRQ_MASK                 (0x00000080U)
#define CSL_CSI2_CTX6_IRQENABLE_LINE_NUMBER_IRQ_SHIFT                (0x00000007U)
#define CSL_CSI2_CTX6_IRQENABLE_LINE_NUMBER_IRQ_RESETVAL             (0x00000000U)
#define CSL_CSI2_CTX6_IRQENABLE_LINE_NUMBER_IRQ_MAX                  (0x00000001U)

#define CSL_CSI2_CTX6_IRQENABLE_ECC_CORRECTION_IRQ_MASK              (0x00000100U)
#define CSL_CSI2_CTX6_IRQENABLE_ECC_CORRECTION_IRQ_SHIFT             (0x00000008U)
#define CSL_CSI2_CTX6_IRQENABLE_ECC_CORRECTION_IRQ_RESETVAL          (0x00000000U)
#define CSL_CSI2_CTX6_IRQENABLE_ECC_CORRECTION_IRQ_MAX               (0x00000001U)

#define CSL_CSI2_CTX6_IRQENABLE_RES22_MASK                           (0xFFFFFE00U)
#define CSL_CSI2_CTX6_IRQENABLE_RES22_SHIFT                          (0x00000009U)
#define CSL_CSI2_CTX6_IRQENABLE_RES22_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX6_IRQENABLE_RES22_MAX                            (0x007FFFFFU)

#define CSL_CSI2_CTX6_IRQENABLE_RESETVAL                             (0x00000000U)

/* CSI2_CTX6_IRQSTATUS */

#define CSL_CSI2_CTX6_IRQSTATUS_FS_IRQ_MASK                          (0x00000001U)
#define CSL_CSI2_CTX6_IRQSTATUS_FS_IRQ_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX6_IRQSTATUS_FS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX6_IRQSTATUS_FS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX6_IRQSTATUS_FE_IRQ_MASK                          (0x00000002U)
#define CSL_CSI2_CTX6_IRQSTATUS_FE_IRQ_SHIFT                         (0x00000001U)
#define CSL_CSI2_CTX6_IRQSTATUS_FE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX6_IRQSTATUS_FE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX6_IRQSTATUS_LS_IRQ_MASK                          (0x00000004U)
#define CSL_CSI2_CTX6_IRQSTATUS_LS_IRQ_SHIFT                         (0x00000002U)
#define CSL_CSI2_CTX6_IRQSTATUS_LS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX6_IRQSTATUS_LS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX6_IRQSTATUS_LE_IRQ_MASK                          (0x00000008U)
#define CSL_CSI2_CTX6_IRQSTATUS_LE_IRQ_SHIFT                         (0x00000003U)
#define CSL_CSI2_CTX6_IRQSTATUS_LE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX6_IRQSTATUS_LE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX6_IRQSTATUS_RES25_MASK                           (0x00000010U)
#define CSL_CSI2_CTX6_IRQSTATUS_RES25_SHIFT                          (0x00000004U)
#define CSL_CSI2_CTX6_IRQSTATUS_RES25_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX6_IRQSTATUS_RES25_MAX                            (0x00000001U)

#define CSL_CSI2_CTX6_IRQSTATUS_CS_IRQ_MASK                          (0x00000020U)
#define CSL_CSI2_CTX6_IRQSTATUS_CS_IRQ_SHIFT                         (0x00000005U)
#define CSL_CSI2_CTX6_IRQSTATUS_CS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX6_IRQSTATUS_CS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX6_IRQSTATUS_FRAME_NUMBER_IRQ_MASK                (0x00000040U)
#define CSL_CSI2_CTX6_IRQSTATUS_FRAME_NUMBER_IRQ_SHIFT               (0x00000006U)
#define CSL_CSI2_CTX6_IRQSTATUS_FRAME_NUMBER_IRQ_RESETVAL            (0x00000000U)
#define CSL_CSI2_CTX6_IRQSTATUS_FRAME_NUMBER_IRQ_MAX                 (0x00000001U)

#define CSL_CSI2_CTX6_IRQSTATUS_LINE_NUMBER_IRQ_MASK                 (0x00000080U)
#define CSL_CSI2_CTX6_IRQSTATUS_LINE_NUMBER_IRQ_SHIFT                (0x00000007U)
#define CSL_CSI2_CTX6_IRQSTATUS_LINE_NUMBER_IRQ_RESETVAL             (0x00000000U)
#define CSL_CSI2_CTX6_IRQSTATUS_LINE_NUMBER_IRQ_MAX                  (0x00000001U)

#define CSL_CSI2_CTX6_IRQSTATUS_ECC_CORRECTION_IRQ_MASK              (0x00000100U)
#define CSL_CSI2_CTX6_IRQSTATUS_ECC_CORRECTION_IRQ_SHIFT             (0x00000008U)
#define CSL_CSI2_CTX6_IRQSTATUS_ECC_CORRECTION_IRQ_RESETVAL          (0x00000000U)
#define CSL_CSI2_CTX6_IRQSTATUS_ECC_CORRECTION_IRQ_MAX               (0x00000001U)

#define CSL_CSI2_CTX6_IRQSTATUS_RES24_MASK                           (0xFFFFFE00U)
#define CSL_CSI2_CTX6_IRQSTATUS_RES24_SHIFT                          (0x00000009U)
#define CSL_CSI2_CTX6_IRQSTATUS_RES24_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX6_IRQSTATUS_RES24_MAX                            (0x007FFFFFU)

#define CSL_CSI2_CTX6_IRQSTATUS_RESETVAL                             (0x00000000U)

/* CSI2_CTX6_CTRL3 */

#define CSL_CSI2_CTX6_CTRL3_LINE_NUMBER_MASK                         (0x0000FFFFU)
#define CSL_CSI2_CTX6_CTRL3_LINE_NUMBER_SHIFT                        (0x00000000U)
#define CSL_CSI2_CTX6_CTRL3_LINE_NUMBER_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX6_CTRL3_LINE_NUMBER_MAX                          (0x0000FFFFU)

#define CSL_CSI2_CTX6_CTRL3_ALPHA_MASK                               (0x3FFF0000U)
#define CSL_CSI2_CTX6_CTRL3_ALPHA_SHIFT                              (0x00000010U)
#define CSL_CSI2_CTX6_CTRL3_ALPHA_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX6_CTRL3_ALPHA_MAX                                (0x00003FFFU)

#define CSL_CSI2_CTX6_CTRL3_RESERVED_MASK                            (0xC0000000U)
#define CSL_CSI2_CTX6_CTRL3_RESERVED_SHIFT                           (0x0000001EU)
#define CSL_CSI2_CTX6_CTRL3_RESERVED_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX6_CTRL3_RESERVED_MAX                             (0x00000003U)

#define CSL_CSI2_CTX6_CTRL3_RESETVAL                                 (0x00000000U)

/* CSI2_CTX7_CTRL1 */

#define CSL_CSI2_CTX7_CTRL1_CTX_EN_MASK                              (0x00000001U)
#define CSL_CSI2_CTX7_CTRL1_CTX_EN_SHIFT                             (0x00000000U)
#define CSL_CSI2_CTX7_CTRL1_CTX_EN_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX7_CTRL1_CTX_EN_MAX                               (0x00000001U)

#define CSL_CSI2_CTX7_CTRL1_LINE_MODULO_MASK                         (0x00000002U)
#define CSL_CSI2_CTX7_CTRL1_LINE_MODULO_SHIFT                        (0x00000001U)
#define CSL_CSI2_CTX7_CTRL1_LINE_MODULO_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX7_CTRL1_LINE_MODULO_MAX                          (0x00000001U)

#define CSL_CSI2_CTX7_CTRL1_VP_FORCE_MASK                            (0x00000004U)
#define CSL_CSI2_CTX7_CTRL1_VP_FORCE_SHIFT                           (0x00000002U)
#define CSL_CSI2_CTX7_CTRL1_VP_FORCE_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX7_CTRL1_VP_FORCE_MAX                             (0x00000001U)

#define CSL_CSI2_CTX7_CTRL1_PING_PONG_MASK                           (0x00000008U)
#define CSL_CSI2_CTX7_CTRL1_PING_PONG_SHIFT                          (0x00000003U)
#define CSL_CSI2_CTX7_CTRL1_PING_PONG_RESETVAL                       (0x00000001U)
#define CSL_CSI2_CTX7_CTRL1_PING_PONG_MAX                            (0x00000001U)

#define CSL_CSI2_CTX7_CTRL1_COUNT_UNLOCK_MASK                        (0x00000010U)
#define CSL_CSI2_CTX7_CTRL1_COUNT_UNLOCK_SHIFT                       (0x00000004U)
#define CSL_CSI2_CTX7_CTRL1_COUNT_UNLOCK_RESETVAL                    (0x00000000U)
#define CSL_CSI2_CTX7_CTRL1_COUNT_UNLOCK_MAX                         (0x00000001U)

#define CSL_CSI2_CTX7_CTRL1_CS_EN_MASK                               (0x00000020U)
#define CSL_CSI2_CTX7_CTRL1_CS_EN_SHIFT                              (0x00000005U)
#define CSL_CSI2_CTX7_CTRL1_CS_EN_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX7_CTRL1_CS_EN_MAX                                (0x00000001U)

#define CSL_CSI2_CTX7_CTRL1_EOL_EN_MASK                              (0x00000040U)
#define CSL_CSI2_CTX7_CTRL1_EOL_EN_SHIFT                             (0x00000006U)
#define CSL_CSI2_CTX7_CTRL1_EOL_EN_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX7_CTRL1_EOL_EN_MAX                               (0x00000001U)

#define CSL_CSI2_CTX7_CTRL1_EOF_EN_MASK                              (0x00000080U)
#define CSL_CSI2_CTX7_CTRL1_EOF_EN_SHIFT                             (0x00000007U)
#define CSL_CSI2_CTX7_CTRL1_EOF_EN_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX7_CTRL1_EOF_EN_MAX                               (0x00000001U)

#define CSL_CSI2_CTX7_CTRL1_COUNT_MASK                               (0x0000FF00U)
#define CSL_CSI2_CTX7_CTRL1_COUNT_SHIFT                              (0x00000008U)
#define CSL_CSI2_CTX7_CTRL1_COUNT_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX7_CTRL1_COUNT_MAX                                (0x000000FFU)

#define CSL_CSI2_CTX7_CTRL1_FEC_NUMBER_MASK                          (0x00FF0000U)
#define CSL_CSI2_CTX7_CTRL1_FEC_NUMBER_SHIFT                         (0x00000010U)
#define CSL_CSI2_CTX7_CTRL1_FEC_NUMBER_RESETVAL                      (0x00000001U)
#define CSL_CSI2_CTX7_CTRL1_FEC_NUMBER_MAX                           (0x000000FFU)

#define CSL_CSI2_CTX7_CTRL1_TRANSCODE_MASK                           (0x0F000000U)
#define CSL_CSI2_CTX7_CTRL1_TRANSCODE_SHIFT                          (0x00000018U)
#define CSL_CSI2_CTX7_CTRL1_TRANSCODE_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX7_CTRL1_TRANSCODE_MAX                            (0x0000000FU)

#define CSL_CSI2_CTX7_CTRL1_HSCALE_MASK                              (0x10000000U)
#define CSL_CSI2_CTX7_CTRL1_HSCALE_SHIFT                             (0x0000001CU)
#define CSL_CSI2_CTX7_CTRL1_HSCALE_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX7_CTRL1_HSCALE_MAX                               (0x00000001U)

#define CSL_CSI2_CTX7_CTRL1_RES19_MASK                               (0x20000000U)
#define CSL_CSI2_CTX7_CTRL1_RES19_SHIFT                              (0x0000001DU)
#define CSL_CSI2_CTX7_CTRL1_RES19_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX7_CTRL1_RES19_MAX                                (0x00000001U)

#define CSL_CSI2_CTX7_CTRL1_GENERIC_MASK                             (0x40000000U)
#define CSL_CSI2_CTX7_CTRL1_GENERIC_SHIFT                            (0x0000001EU)
#define CSL_CSI2_CTX7_CTRL1_GENERIC_RESETVAL                         (0x00000000U)
#define CSL_CSI2_CTX7_CTRL1_GENERIC_MAX                              (0x00000001U)

#define CSL_CSI2_CTX7_CTRL1_BYTESWAP_MASK                            (0x80000000U)
#define CSL_CSI2_CTX7_CTRL1_BYTESWAP_SHIFT                           (0x0000001FU)
#define CSL_CSI2_CTX7_CTRL1_BYTESWAP_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX7_CTRL1_BYTESWAP_MAX                             (0x00000001U)

#define CSL_CSI2_CTX7_CTRL1_RESETVAL                                 (0x00010008U)

/* CSI2_CTX7_CTRL2 */

#define CSL_CSI2_CTX7_CTRL2_FORMAT_MASK                              (0x000003FFU)
#define CSL_CSI2_CTX7_CTRL2_FORMAT_SHIFT                             (0x00000000U)
#define CSL_CSI2_CTX7_CTRL2_FORMAT_RESETVAL                          (0x00000000U)
#define CSL_CSI2_CTX7_CTRL2_FORMAT_MAX                               (0x000003FFU)

#define CSL_CSI2_CTX7_CTRL2_DPCM_PRED_MASK                           (0x00000400U)
#define CSL_CSI2_CTX7_CTRL2_DPCM_PRED_SHIFT                          (0x0000000AU)
#define CSL_CSI2_CTX7_CTRL2_DPCM_PRED_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX7_CTRL2_DPCM_PRED_MAX                            (0x00000001U)

#define CSL_CSI2_CTX7_CTRL2_VIRTUAL_ID_MASK                          (0x00001800U)
#define CSL_CSI2_CTX7_CTRL2_VIRTUAL_ID_SHIFT                         (0x0000000BU)
#define CSL_CSI2_CTX7_CTRL2_VIRTUAL_ID_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX7_CTRL2_VIRTUAL_ID_MAX                           (0x00000003U)

#define CSL_CSI2_CTX7_CTRL2_USER_DEF_MAPPING_MASK                    (0x00006000U)
#define CSL_CSI2_CTX7_CTRL2_USER_DEF_MAPPING_SHIFT                   (0x0000000DU)
#define CSL_CSI2_CTX7_CTRL2_USER_DEF_MAPPING_RESETVAL                (0x00000000U)
#define CSL_CSI2_CTX7_CTRL2_USER_DEF_MAPPING_MAX                     (0x00000003U)

#define CSL_CSI2_CTX7_CTRL2_RES20_MASK                               (0x00008000U)
#define CSL_CSI2_CTX7_CTRL2_RES20_SHIFT                              (0x0000000FU)
#define CSL_CSI2_CTX7_CTRL2_RES20_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX7_CTRL2_RES20_MAX                                (0x00000001U)

#define CSL_CSI2_CTX7_CTRL2_FRAME_MASK                               (0xFFFF0000U)
#define CSL_CSI2_CTX7_CTRL2_FRAME_SHIFT                              (0x00000010U)
#define CSL_CSI2_CTX7_CTRL2_FRAME_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX7_CTRL2_FRAME_MAX                                (0x0000FFFFU)

#define CSL_CSI2_CTX7_CTRL2_RESETVAL                                 (0x00000000U)

/* CSI2_CTX7_DAT_OFST */

#define CSL_CSI2_CTX7_DAT_OFST_OFST_MASK                             (0x0001FFE0U)
#define CSL_CSI2_CTX7_DAT_OFST_OFST_SHIFT                            (0x00000005U)
#define CSL_CSI2_CTX7_DAT_OFST_OFST_RESETVAL                         (0x00000000U)
#define CSL_CSI2_CTX7_DAT_OFST_OFST_MAX                              (0x00000FFFU)

#define CSL_CSI2_CTX7_DAT_OFST_RES21_MASK                            (0xFFFE0000U)
#define CSL_CSI2_CTX7_DAT_OFST_RES21_SHIFT                           (0x00000011U)
#define CSL_CSI2_CTX7_DAT_OFST_RES21_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX7_DAT_OFST_RES21_MAX                             (0x00007FFFU)

#define CSL_CSI2_CTX7_DAT_OFST_RESETVAL                              (0x00000000U)

/* CSI2_CTX7_DAT_PING_ADDR */

#define CSL_CSI2_CTX7_DAT_PING_ADDR_RES_MASK                         (0x0000001FU)
#define CSL_CSI2_CTX7_DAT_PING_ADDR_RES_SHIFT                        (0x00000000U)
#define CSL_CSI2_CTX7_DAT_PING_ADDR_RES_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX7_DAT_PING_ADDR_RES_MAX                          (0x0000001FU)

#define CSL_CSI2_CTX7_DAT_PING_ADDR_ADDR_MASK                        (0xFFFFFFE0U)
#define CSL_CSI2_CTX7_DAT_PING_ADDR_ADDR_SHIFT                       (0x00000005U)
#define CSL_CSI2_CTX7_DAT_PING_ADDR_ADDR_RESETVAL                    (0x00000000U)
#define CSL_CSI2_CTX7_DAT_PING_ADDR_ADDR_MAX                         (0x07FFFFFFU)

#define CSL_CSI2_CTX7_DAT_PING_ADDR_RESETVAL                         (0x00000000U)

/* CSI2_CTX7_DAT_PONG_ADDR */

#define CSL_CSI2_CTX7_DAT_PONG_ADDR_RES_MASK                         (0x0000001FU)
#define CSL_CSI2_CTX7_DAT_PONG_ADDR_RES_SHIFT                        (0x00000000U)
#define CSL_CSI2_CTX7_DAT_PONG_ADDR_RES_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX7_DAT_PONG_ADDR_RES_MAX                          (0x0000001FU)

#define CSL_CSI2_CTX7_DAT_PONG_ADDR_ADDR_MASK                        (0xFFFFFFE0U)
#define CSL_CSI2_CTX7_DAT_PONG_ADDR_ADDR_SHIFT                       (0x00000005U)
#define CSL_CSI2_CTX7_DAT_PONG_ADDR_ADDR_RESETVAL                    (0x00000000U)
#define CSL_CSI2_CTX7_DAT_PONG_ADDR_ADDR_MAX                         (0x07FFFFFFU)

#define CSL_CSI2_CTX7_DAT_PONG_ADDR_RESETVAL                         (0x00000000U)

/* CSI2_CTX7_IRQENABLE */

#define CSL_CSI2_CTX7_IRQENABLE_FS_IRQ_MASK                          (0x00000001U)
#define CSL_CSI2_CTX7_IRQENABLE_FS_IRQ_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX7_IRQENABLE_FS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX7_IRQENABLE_FS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX7_IRQENABLE_FE_IRQ_MASK                          (0x00000002U)
#define CSL_CSI2_CTX7_IRQENABLE_FE_IRQ_SHIFT                         (0x00000001U)
#define CSL_CSI2_CTX7_IRQENABLE_FE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX7_IRQENABLE_FE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX7_IRQENABLE_LS_IRQ_MASK                          (0x00000004U)
#define CSL_CSI2_CTX7_IRQENABLE_LS_IRQ_SHIFT                         (0x00000002U)
#define CSL_CSI2_CTX7_IRQENABLE_LS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX7_IRQENABLE_LS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX7_IRQENABLE_LE_IRQ_MASK                          (0x00000008U)
#define CSL_CSI2_CTX7_IRQENABLE_LE_IRQ_SHIFT                         (0x00000003U)
#define CSL_CSI2_CTX7_IRQENABLE_LE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX7_IRQENABLE_LE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX7_IRQENABLE_RES23_MASK                           (0x00000010U)
#define CSL_CSI2_CTX7_IRQENABLE_RES23_SHIFT                          (0x00000004U)
#define CSL_CSI2_CTX7_IRQENABLE_RES23_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX7_IRQENABLE_RES23_MAX                            (0x00000001U)

#define CSL_CSI2_CTX7_IRQENABLE_CS_IRQ_MASK                          (0x00000020U)
#define CSL_CSI2_CTX7_IRQENABLE_CS_IRQ_SHIFT                         (0x00000005U)
#define CSL_CSI2_CTX7_IRQENABLE_CS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX7_IRQENABLE_CS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX7_IRQENABLE_FRAME_NUMBER_IRQ_MASK                (0x00000040U)
#define CSL_CSI2_CTX7_IRQENABLE_FRAME_NUMBER_IRQ_SHIFT               (0x00000006U)
#define CSL_CSI2_CTX7_IRQENABLE_FRAME_NUMBER_IRQ_RESETVAL            (0x00000000U)
#define CSL_CSI2_CTX7_IRQENABLE_FRAME_NUMBER_IRQ_MAX                 (0x00000001U)

#define CSL_CSI2_CTX7_IRQENABLE_LINE_NUMBER_IRQ_MASK                 (0x00000080U)
#define CSL_CSI2_CTX7_IRQENABLE_LINE_NUMBER_IRQ_SHIFT                (0x00000007U)
#define CSL_CSI2_CTX7_IRQENABLE_LINE_NUMBER_IRQ_RESETVAL             (0x00000000U)
#define CSL_CSI2_CTX7_IRQENABLE_LINE_NUMBER_IRQ_MAX                  (0x00000001U)

#define CSL_CSI2_CTX7_IRQENABLE_ECC_CORRECTION_IRQ_MASK              (0x00000100U)
#define CSL_CSI2_CTX7_IRQENABLE_ECC_CORRECTION_IRQ_SHIFT             (0x00000008U)
#define CSL_CSI2_CTX7_IRQENABLE_ECC_CORRECTION_IRQ_RESETVAL          (0x00000000U)
#define CSL_CSI2_CTX7_IRQENABLE_ECC_CORRECTION_IRQ_MAX               (0x00000001U)

#define CSL_CSI2_CTX7_IRQENABLE_RES22_MASK                           (0xFFFFFE00U)
#define CSL_CSI2_CTX7_IRQENABLE_RES22_SHIFT                          (0x00000009U)
#define CSL_CSI2_CTX7_IRQENABLE_RES22_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX7_IRQENABLE_RES22_MAX                            (0x007FFFFFU)

#define CSL_CSI2_CTX7_IRQENABLE_RESETVAL                             (0x00000000U)

/* CSI2_CTX7_IRQSTATUS */

#define CSL_CSI2_CTX7_IRQSTATUS_FS_IRQ_MASK                          (0x00000001U)
#define CSL_CSI2_CTX7_IRQSTATUS_FS_IRQ_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX7_IRQSTATUS_FS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX7_IRQSTATUS_FS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX7_IRQSTATUS_FE_IRQ_MASK                          (0x00000002U)
#define CSL_CSI2_CTX7_IRQSTATUS_FE_IRQ_SHIFT                         (0x00000001U)
#define CSL_CSI2_CTX7_IRQSTATUS_FE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX7_IRQSTATUS_FE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX7_IRQSTATUS_LS_IRQ_MASK                          (0x00000004U)
#define CSL_CSI2_CTX7_IRQSTATUS_LS_IRQ_SHIFT                         (0x00000002U)
#define CSL_CSI2_CTX7_IRQSTATUS_LS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX7_IRQSTATUS_LS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX7_IRQSTATUS_LE_IRQ_MASK                          (0x00000008U)
#define CSL_CSI2_CTX7_IRQSTATUS_LE_IRQ_SHIFT                         (0x00000003U)
#define CSL_CSI2_CTX7_IRQSTATUS_LE_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX7_IRQSTATUS_LE_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX7_IRQSTATUS_RES25_MASK                           (0x00000010U)
#define CSL_CSI2_CTX7_IRQSTATUS_RES25_SHIFT                          (0x00000004U)
#define CSL_CSI2_CTX7_IRQSTATUS_RES25_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX7_IRQSTATUS_RES25_MAX                            (0x00000001U)

#define CSL_CSI2_CTX7_IRQSTATUS_CS_IRQ_MASK                          (0x00000020U)
#define CSL_CSI2_CTX7_IRQSTATUS_CS_IRQ_SHIFT                         (0x00000005U)
#define CSL_CSI2_CTX7_IRQSTATUS_CS_IRQ_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX7_IRQSTATUS_CS_IRQ_MAX                           (0x00000001U)

#define CSL_CSI2_CTX7_IRQSTATUS_FRAME_NUMBER_IRQ_MASK                (0x00000040U)
#define CSL_CSI2_CTX7_IRQSTATUS_FRAME_NUMBER_IRQ_SHIFT               (0x00000006U)
#define CSL_CSI2_CTX7_IRQSTATUS_FRAME_NUMBER_IRQ_RESETVAL            (0x00000000U)
#define CSL_CSI2_CTX7_IRQSTATUS_FRAME_NUMBER_IRQ_MAX                 (0x00000001U)

#define CSL_CSI2_CTX7_IRQSTATUS_LINE_NUMBER_IRQ_MASK                 (0x00000080U)
#define CSL_CSI2_CTX7_IRQSTATUS_LINE_NUMBER_IRQ_SHIFT                (0x00000007U)
#define CSL_CSI2_CTX7_IRQSTATUS_LINE_NUMBER_IRQ_RESETVAL             (0x00000000U)
#define CSL_CSI2_CTX7_IRQSTATUS_LINE_NUMBER_IRQ_MAX                  (0x00000001U)

#define CSL_CSI2_CTX7_IRQSTATUS_ECC_CORRECTION_IRQ_MASK              (0x00000100U)
#define CSL_CSI2_CTX7_IRQSTATUS_ECC_CORRECTION_IRQ_SHIFT             (0x00000008U)
#define CSL_CSI2_CTX7_IRQSTATUS_ECC_CORRECTION_IRQ_RESETVAL          (0x00000000U)
#define CSL_CSI2_CTX7_IRQSTATUS_ECC_CORRECTION_IRQ_MAX               (0x00000001U)

#define CSL_CSI2_CTX7_IRQSTATUS_RES24_MASK                           (0xFFFFFE00U)
#define CSL_CSI2_CTX7_IRQSTATUS_RES24_SHIFT                          (0x00000009U)
#define CSL_CSI2_CTX7_IRQSTATUS_RES24_RESETVAL                       (0x00000000U)
#define CSL_CSI2_CTX7_IRQSTATUS_RES24_MAX                            (0x007FFFFFU)

#define CSL_CSI2_CTX7_IRQSTATUS_RESETVAL                             (0x00000000U)

/* CSI2_CTX7_CTRL3 */

#define CSL_CSI2_CTX7_CTRL3_LINE_NUMBER_MASK                         (0x0000FFFFU)
#define CSL_CSI2_CTX7_CTRL3_LINE_NUMBER_SHIFT                        (0x00000000U)
#define CSL_CSI2_CTX7_CTRL3_LINE_NUMBER_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX7_CTRL3_LINE_NUMBER_MAX                          (0x0000FFFFU)

#define CSL_CSI2_CTX7_CTRL3_ALPHA_MASK                               (0x3FFF0000U)
#define CSL_CSI2_CTX7_CTRL3_ALPHA_SHIFT                              (0x00000010U)
#define CSL_CSI2_CTX7_CTRL3_ALPHA_RESETVAL                           (0x00000000U)
#define CSL_CSI2_CTX7_CTRL3_ALPHA_MAX                                (0x00003FFFU)

#define CSL_CSI2_CTX7_CTRL3_RESERVED_MASK                            (0xC0000000U)
#define CSL_CSI2_CTX7_CTRL3_RESERVED_SHIFT                           (0x0000001EU)
#define CSL_CSI2_CTX7_CTRL3_RESERVED_RESETVAL                        (0x00000000U)
#define CSL_CSI2_CTX7_CTRL3_RESERVED_MAX                             (0x00000003U)

#define CSL_CSI2_CTX7_CTRL3_RESETVAL                                 (0x00000000U)

/* CSI2_PHY_CFG_REG0 */

#define CSL_CSI2_PHY_CFG_REG0_THS_SETTLE_MASK                        (0x000000FFU)
#define CSL_CSI2_PHY_CFG_REG0_THS_SETTLE_SHIFT                       (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG0_THS_SETTLE_RESETVAL                    (0x00000027U)
#define CSL_CSI2_PHY_CFG_REG0_THS_SETTLE_MAX                         (0x000000FFU)

#define CSL_CSI2_PHY_CFG_REG0_THS_TERM_MASK                          (0x0000FF00U)
#define CSL_CSI2_PHY_CFG_REG0_THS_TERM_SHIFT                         (0x00000008U)
#define CSL_CSI2_PHY_CFG_REG0_THS_TERM_RESETVAL                      (0x00000004U)
#define CSL_CSI2_PHY_CFG_REG0_THS_TERM_MAX                           (0x000000FFU)

#define CSL_CSI2_PHY_CFG_REG0_RESERVED_MASK                          (0x00FF0000U)
#define CSL_CSI2_PHY_CFG_REG0_RESERVED_SHIFT                         (0x00000010U)
#define CSL_CSI2_PHY_CFG_REG0_RESERVED_RESETVAL                      (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG0_RESERVED_MAX                           (0x000000FFU)

#define CSL_CSI2_PHY_CFG_REG0_HS_CLK_CONFIG_MASK                     (0xFF000000U)
#define CSL_CSI2_PHY_CFG_REG0_HS_CLK_CONFIG_SHIFT                    (0x00000018U)
#define CSL_CSI2_PHY_CFG_REG0_HS_CLK_CONFIG_RESETVAL                 (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG0_HS_CLK_CONFIG_MAX                      (0x000000FFU)

#define CSL_CSI2_PHY_CFG_REG0_RESETVAL                               (0x00000427U)

/* CSI2_PHY_CFG_REG1 */

#define CSL_CSI2_PHY_CFG_REG1_TCLK_SETTLE_MASK                       (0x000000FFU)
#define CSL_CSI2_PHY_CFG_REG1_TCLK_SETTLE_SHIFT                      (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG1_TCLK_SETTLE_RESETVAL                   (0x0000000EU)
#define CSL_CSI2_PHY_CFG_REG1_TCLK_SETTLE_MAX                        (0x000000FFU)

#define CSL_CSI2_PHY_CFG_REG1_CTRLCLK_DIV_FACT_MASK                  (0x00000300U)
#define CSL_CSI2_PHY_CFG_REG1_CTRLCLK_DIV_FACT_SHIFT                 (0x00000008U)
#define CSL_CSI2_PHY_CFG_REG1_CTRLCLK_DIV_FACT_RESETVAL              (0x00000001U)
#define CSL_CSI2_PHY_CFG_REG1_CTRLCLK_DIV_FACT_MAX                   (0x00000003U)

#define CSL_CSI2_PHY_CFG_REG1_D_PHY_HS_SYNC_PAT_MASK                 (0x0003FC00U)
#define CSL_CSI2_PHY_CFG_REG1_D_PHY_HS_SYNC_PAT_SHIFT                (0x0000000AU)
#define CSL_CSI2_PHY_CFG_REG1_D_PHY_HS_SYNC_PAT_RESETVAL             (0x000000B8U)
#define CSL_CSI2_PHY_CFG_REG1_D_PHY_HS_SYNC_PAT_MAX                  (0x000000FFU)

#define CSL_CSI2_PHY_CFG_REG1_TCLK_TERM_MASK                         (0x01FC0000U)
#define CSL_CSI2_PHY_CFG_REG1_TCLK_TERM_SHIFT                        (0x00000012U)
#define CSL_CSI2_PHY_CFG_REG1_TCLK_TERM_RESETVAL                     (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG1_TCLK_TERM_MAX                          (0x0000007FU)

#define CSL_CSI2_PHY_CFG_REG1_CLK_MISS_DET_MASK                      (0x02000000U)
#define CSL_CSI2_PHY_CFG_REG1_CLK_MISS_DET_SHIFT                     (0x00000019U)
#define CSL_CSI2_PHY_CFG_REG1_CLK_MISS_DET_RESETVAL                  (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG1_CLK_MISS_DET_MAX                       (0x00000001U)

#define CSL_CSI2_PHY_CFG_REG1_RSVD1_MASK                             (0x0C000000U)
#define CSL_CSI2_PHY_CFG_REG1_RSVD1_SHIFT                            (0x0000001AU)
#define CSL_CSI2_PHY_CFG_REG1_RSVD1_RESETVAL                         (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG1_RSVD1_MAX                              (0x00000003U)

#define CSL_CSI2_PHY_CFG_REG1_RESETDONERXBYTECLK_MASK                (0x10000000U)
#define CSL_CSI2_PHY_CFG_REG1_RESETDONERXBYTECLK_SHIFT               (0x0000001CU)
#define CSL_CSI2_PHY_CFG_REG1_RESETDONERXBYTECLK_RESETVAL            (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG1_RESETDONERXBYTECLK_MAX                 (0x00000001U)

#define CSL_CSI2_PHY_CFG_REG1_RESETDONECTRLCLK_MASK                  (0x20000000U)
#define CSL_CSI2_PHY_CFG_REG1_RESETDONECTRLCLK_SHIFT                 (0x0000001DU)
#define CSL_CSI2_PHY_CFG_REG1_RESETDONECTRLCLK_RESETVAL              (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG1_RESETDONECTRLCLK_MAX                   (0x00000001U)

#define CSL_CSI2_PHY_CFG_REG1_RSVD2_MASK                             (0xC0000000U)
#define CSL_CSI2_PHY_CFG_REG1_RSVD2_SHIFT                            (0x0000001EU)
#define CSL_CSI2_PHY_CFG_REG1_RSVD2_RESETVAL                         (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG1_RSVD2_MAX                              (0x00000003U)

#define CSL_CSI2_PHY_CFG_REG1_RESETVAL                               (0x0002E10EU)

/* CSI2_PHY_CFG_REG2 */

#define CSL_CSI2_PHY_CFG_REG2_CCP2_SYNC_PAT_MASK                     (0x00FFFFFFU)
#define CSL_CSI2_PHY_CFG_REG2_CCP2_SYNC_PAT_SHIFT                    (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG2_CCP2_SYNC_PAT_RESETVAL                 (0x000000FFU)
#define CSL_CSI2_PHY_CFG_REG2_CCP2_SYNC_PAT_MAX                      (0x00FFFFFFU)

#define CSL_CSI2_PHY_CFG_REG2_RXTRIGGERESC3_MASK                     (0x03000000U)
#define CSL_CSI2_PHY_CFG_REG2_RXTRIGGERESC3_SHIFT                    (0x00000018U)
#define CSL_CSI2_PHY_CFG_REG2_RXTRIGGERESC3_RESETVAL                 (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG2_RXTRIGGERESC3_MAX                      (0x00000003U)

#define CSL_CSI2_PHY_CFG_REG2_RXTRIGGERESC2_MASK                     (0x0C000000U)
#define CSL_CSI2_PHY_CFG_REG2_RXTRIGGERESC2_SHIFT                    (0x0000001AU)
#define CSL_CSI2_PHY_CFG_REG2_RXTRIGGERESC2_RESETVAL                 (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG2_RXTRIGGERESC2_MAX                      (0x00000003U)

#define CSL_CSI2_PHY_CFG_REG2_RXTRIGGERESC1_MASK                     (0x30000000U)
#define CSL_CSI2_PHY_CFG_REG2_RXTRIGGERESC1_SHIFT                    (0x0000001CU)
#define CSL_CSI2_PHY_CFG_REG2_RXTRIGGERESC1_RESETVAL                 (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG2_RXTRIGGERESC1_MAX                      (0x00000003U)

#define CSL_CSI2_PHY_CFG_REG2_RXTRIGGERESC0_MASK                     (0xC0000000U)
#define CSL_CSI2_PHY_CFG_REG2_RXTRIGGERESC0_SHIFT                    (0x0000001EU)
#define CSL_CSI2_PHY_CFG_REG2_RXTRIGGERESC0_RESETVAL                 (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG2_RXTRIGGERESC0_MAX                      (0x00000003U)

#define CSL_CSI2_PHY_CFG_REG2_RESETVAL                               (0x000000FFU)

/* CSI2_PHY_CFG_REG3 */

#define CSL_CSI2_PHY_CFG_REG3_RECAL_BIAS_MASK                        (0x00000001U)
#define CSL_CSI2_PHY_CFG_REG3_RECAL_BIAS_SHIFT                       (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG3_RECAL_BIAS_RESETVAL                    (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG3_RECAL_BIAS_MAX                         (0x00000001U)

#define CSL_CSI2_PHY_CFG_REG3_RECAL_HS_RX_MASK                       (0x00000002U)
#define CSL_CSI2_PHY_CFG_REG3_RECAL_HS_RX_SHIFT                      (0x00000001U)
#define CSL_CSI2_PHY_CFG_REG3_RECAL_HS_RX_RESETVAL                   (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG3_RECAL_HS_RX_MAX                        (0x00000001U)

#define CSL_CSI2_PHY_CFG_REG3_RSVD1_MASK                             (0x00000004U)
#define CSL_CSI2_PHY_CFG_REG3_RSVD1_SHIFT                            (0x00000002U)
#define CSL_CSI2_PHY_CFG_REG3_RSVD1_RESETVAL                         (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG3_RSVD1_MAX                              (0x00000001U)

#define CSL_CSI2_PHY_CFG_REG3_OVR_ENCCP_TO_HSRX_MASK                 (0x00000008U)
#define CSL_CSI2_PHY_CFG_REG3_OVR_ENCCP_TO_HSRX_SHIFT                (0x00000003U)
#define CSL_CSI2_PHY_CFG_REG3_OVR_ENCCP_TO_HSRX_RESETVAL             (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG3_OVR_ENCCP_TO_HSRX_MAX                  (0x00000001U)

#define CSL_CSI2_PHY_CFG_REG3_OVR_ENCCP_TO_ANAT_MASK                 (0x00000010U)
#define CSL_CSI2_PHY_CFG_REG3_OVR_ENCCP_TO_ANAT_SHIFT                (0x00000004U)
#define CSL_CSI2_PHY_CFG_REG3_OVR_ENCCP_TO_ANAT_RESETVAL             (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG3_OVR_ENCCP_TO_ANAT_MAX                  (0x00000001U)

#define CSL_CSI2_PHY_CFG_REG3_ENBIAS_MASK                            (0x00000020U)
#define CSL_CSI2_PHY_CFG_REG3_ENBIAS_SHIFT                           (0x00000005U)
#define CSL_CSI2_PHY_CFG_REG3_ENBIAS_RESETVAL                        (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG3_ENBIAS_MAX                             (0x00000001U)

#define CSL_CSI2_PHY_CFG_REG3_OVR_ENBIAS_MASK                        (0x00000040U)
#define CSL_CSI2_PHY_CFG_REG3_OVR_ENBIAS_SHIFT                       (0x00000006U)
#define CSL_CSI2_PHY_CFG_REG3_OVR_ENBIAS_RESETVAL                    (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG3_OVR_ENBIAS_MAX                         (0x00000001U)

#define CSL_CSI2_PHY_CFG_REG3_ENLDO_MASK                             (0x00000080U)
#define CSL_CSI2_PHY_CFG_REG3_ENLDO_SHIFT                            (0x00000007U)
#define CSL_CSI2_PHY_CFG_REG3_ENLDO_RESETVAL                         (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG3_ENLDO_MAX                              (0x00000001U)

#define CSL_CSI2_PHY_CFG_REG3_OVR_ENLDO_MASK                         (0x00000100U)
#define CSL_CSI2_PHY_CFG_REG3_OVR_ENLDO_SHIFT                        (0x00000008U)
#define CSL_CSI2_PHY_CFG_REG3_OVR_ENLDO_RESETVAL                     (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG3_OVR_ENLDO_MAX                          (0x00000001U)

#define CSL_CSI2_PHY_CFG_REG3_ENULPRX_MASK                           (0x00003E00U)
#define CSL_CSI2_PHY_CFG_REG3_ENULPRX_SHIFT                          (0x00000009U)
#define CSL_CSI2_PHY_CFG_REG3_ENULPRX_RESETVAL                       (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG3_ENULPRX_MAX                            (0x0000001FU)

#define CSL_CSI2_PHY_CFG_REG3_ENLPRX_MASK                            (0x0007C000U)
#define CSL_CSI2_PHY_CFG_REG3_ENLPRX_SHIFT                           (0x0000000EU)
#define CSL_CSI2_PHY_CFG_REG3_ENLPRX_RESETVAL                        (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG3_ENLPRX_MAX                             (0x0000001FU)

#define CSL_CSI2_PHY_CFG_REG3_OVR_ENLPRX_MASK                        (0x00080000U)
#define CSL_CSI2_PHY_CFG_REG3_OVR_ENLPRX_SHIFT                       (0x00000013U)
#define CSL_CSI2_PHY_CFG_REG3_OVR_ENLPRX_RESETVAL                    (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG3_OVR_ENLPRX_MAX                         (0x00000001U)

#define CSL_CSI2_PHY_CFG_REG3_ENRXTERM_MASK                          (0x01F00000U)
#define CSL_CSI2_PHY_CFG_REG3_ENRXTERM_SHIFT                         (0x00000014U)
#define CSL_CSI2_PHY_CFG_REG3_ENRXTERM_RESETVAL                      (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG3_ENRXTERM_MAX                           (0x0000001FU)

#define CSL_CSI2_PHY_CFG_REG3_OVR_ENRXTERM_MASK                      (0x02000000U)
#define CSL_CSI2_PHY_CFG_REG3_OVR_ENRXTERM_SHIFT                     (0x00000019U)
#define CSL_CSI2_PHY_CFG_REG3_OVR_ENRXTERM_RESETVAL                  (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG3_OVR_ENRXTERM_MAX                       (0x00000001U)

#define CSL_CSI2_PHY_CFG_REG3_ENHSRX_MASK                            (0x7C000000U)
#define CSL_CSI2_PHY_CFG_REG3_ENHSRX_SHIFT                           (0x0000001AU)
#define CSL_CSI2_PHY_CFG_REG3_ENHSRX_RESETVAL                        (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG3_ENHSRX_MAX                             (0x0000001FU)

#define CSL_CSI2_PHY_CFG_REG3_OVR_ENHSRX_MASK                        (0x80000000U)
#define CSL_CSI2_PHY_CFG_REG3_OVR_ENHSRX_SHIFT                       (0x0000001FU)
#define CSL_CSI2_PHY_CFG_REG3_OVR_ENHSRX_RESETVAL                    (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG3_OVR_ENHSRX_MAX                         (0x00000001U)

#define CSL_CSI2_PHY_CFG_REG3_RESETVAL                               (0x00000000U)

/* CSI2_PHY_CFG_REG4 */

#define CSL_CSI2_PHY_CFG_REG4_RSVD1_MASK                             (0x00000001U)
#define CSL_CSI2_PHY_CFG_REG4_RSVD1_SHIFT                            (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG4_RSVD1_RESETVAL                         (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG4_RSVD1_MAX                              (0x00000001U)

#define CSL_CSI2_PHY_CFG_REG4_BYPASS_EFUSE_MASK                      (0x00000002U)
#define CSL_CSI2_PHY_CFG_REG4_BYPASS_EFUSE_SHIFT                     (0x00000001U)
#define CSL_CSI2_PHY_CFG_REG4_BYPASS_EFUSE_RESETVAL                  (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG4_BYPASS_EFUSE_MAX                       (0x00000001U)

#define CSL_CSI2_PHY_CFG_REG4_TRIM_TERM_LANE0_MASK                   (0x0000007CU)
#define CSL_CSI2_PHY_CFG_REG4_TRIM_TERM_LANE0_SHIFT                  (0x00000002U)
#define CSL_CSI2_PHY_CFG_REG4_TRIM_TERM_LANE0_RESETVAL               (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG4_TRIM_TERM_LANE0_MAX                    (0x0000001FU)

#define CSL_CSI2_PHY_CFG_REG4_TRIM_TERM_LANE1_MASK                   (0x00000F80U)
#define CSL_CSI2_PHY_CFG_REG4_TRIM_TERM_LANE1_SHIFT                  (0x00000007U)
#define CSL_CSI2_PHY_CFG_REG4_TRIM_TERM_LANE1_RESETVAL               (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG4_TRIM_TERM_LANE1_MAX                    (0x0000001FU)

#define CSL_CSI2_PHY_CFG_REG4_TRIM_TERM_LANE2_MASK                   (0x0001F000U)
#define CSL_CSI2_PHY_CFG_REG4_TRIM_TERM_LANE2_SHIFT                  (0x0000000CU)
#define CSL_CSI2_PHY_CFG_REG4_TRIM_TERM_LANE2_RESETVAL               (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG4_TRIM_TERM_LANE2_MAX                    (0x0000001FU)

#define CSL_CSI2_PHY_CFG_REG4_TRIM_TERM_LANE3_MASK                   (0x003E0000U)
#define CSL_CSI2_PHY_CFG_REG4_TRIM_TERM_LANE3_SHIFT                  (0x00000011U)
#define CSL_CSI2_PHY_CFG_REG4_TRIM_TERM_LANE3_RESETVAL               (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG4_TRIM_TERM_LANE3_MAX                    (0x0000001FU)

#define CSL_CSI2_PHY_CFG_REG4_TRIM_TERM_LANE4_MASK                   (0x07C00000U)
#define CSL_CSI2_PHY_CFG_REG4_TRIM_TERM_LANE4_SHIFT                  (0x00000016U)
#define CSL_CSI2_PHY_CFG_REG4_TRIM_TERM_LANE4_RESETVAL               (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG4_TRIM_TERM_LANE4_MAX                    (0x0000001FU)

#define CSL_CSI2_PHY_CFG_REG4_TRIM_BIAS_GEN_MASK                     (0xF8000000U)
#define CSL_CSI2_PHY_CFG_REG4_TRIM_BIAS_GEN_SHIFT                    (0x0000001BU)
#define CSL_CSI2_PHY_CFG_REG4_TRIM_BIAS_GEN_RESETVAL                 (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG4_TRIM_BIAS_GEN_MAX                      (0x0000001FU)

#define CSL_CSI2_PHY_CFG_REG4_RESETVAL                               (0x00000000U)

/* CSI2_PHY_CFG_REG5 */

#define CSL_CSI2_PHY_CFG_REG5_RSVD1_MASK                             (0x00000001U)
#define CSL_CSI2_PHY_CFG_REG5_RSVD1_SHIFT                            (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG5_RSVD1_RESETVAL                         (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG5_RSVD1_MAX                              (0x00000001U)

#define CSL_CSI2_PHY_CFG_REG5_BYPASS_CALIB_OFFSET_MASK               (0x00000002U)
#define CSL_CSI2_PHY_CFG_REG5_BYPASS_CALIB_OFFSET_SHIFT              (0x00000001U)
#define CSL_CSI2_PHY_CFG_REG5_BYPASS_CALIB_OFFSET_RESETVAL           (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG5_BYPASS_CALIB_OFFSET_MAX                (0x00000001U)

#define CSL_CSI2_PHY_CFG_REG5_TRIM_OFFSET_LANE0_HS_RX_MASK           (0x000000FCU)
#define CSL_CSI2_PHY_CFG_REG5_TRIM_OFFSET_LANE0_HS_RX_SHIFT          (0x00000002U)
#define CSL_CSI2_PHY_CFG_REG5_TRIM_OFFSET_LANE0_HS_RX_RESETVAL       (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG5_TRIM_OFFSET_LANE0_HS_RX_MAX            (0x0000003FU)

#define CSL_CSI2_PHY_CFG_REG5_TRIM_OFFSET_LANE1_HS_RX_MASK           (0x00003F00U)
#define CSL_CSI2_PHY_CFG_REG5_TRIM_OFFSET_LANE1_HS_RX_SHIFT          (0x00000008U)
#define CSL_CSI2_PHY_CFG_REG5_TRIM_OFFSET_LANE1_HS_RX_RESETVAL       (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG5_TRIM_OFFSET_LANE1_HS_RX_MAX            (0x0000003FU)

#define CSL_CSI2_PHY_CFG_REG5_TRIM_OFFSET_LANE2_HS_RX_MASK           (0x000FC000U)
#define CSL_CSI2_PHY_CFG_REG5_TRIM_OFFSET_LANE2_HS_RX_SHIFT          (0x0000000EU)
#define CSL_CSI2_PHY_CFG_REG5_TRIM_OFFSET_LANE2_HS_RX_RESETVAL       (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG5_TRIM_OFFSET_LANE2_HS_RX_MAX            (0x0000003FU)

#define CSL_CSI2_PHY_CFG_REG5_TRIM_OFFSET_LANE3_HS_RX_MASK           (0x03F00000U)
#define CSL_CSI2_PHY_CFG_REG5_TRIM_OFFSET_LANE3_HS_RX_SHIFT          (0x00000014U)
#define CSL_CSI2_PHY_CFG_REG5_TRIM_OFFSET_LANE3_HS_RX_RESETVAL       (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG5_TRIM_OFFSET_LANE3_HS_RX_MAX            (0x0000003FU)

#define CSL_CSI2_PHY_CFG_REG5_TRIM_OFFSET_LANE4_HS_RX_MASK           (0xFC000000U)
#define CSL_CSI2_PHY_CFG_REG5_TRIM_OFFSET_LANE4_HS_RX_SHIFT          (0x0000001AU)
#define CSL_CSI2_PHY_CFG_REG5_TRIM_OFFSET_LANE4_HS_RX_RESETVAL       (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG5_TRIM_OFFSET_LANE4_HS_RX_MAX            (0x0000003FU)

#define CSL_CSI2_PHY_CFG_REG5_RESETVAL                               (0x00000000U)

/* CSI2_PHY_CFG_REG6 */

#define CSL_CSI2_PHY_CFG_REG6_BIASGEN_CAL_OVR_VAL_MASK               (0x0000001FU)
#define CSL_CSI2_PHY_CFG_REG6_BIASGEN_CAL_OVR_VAL_SHIFT              (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG6_BIASGEN_CAL_OVR_VAL_RESETVAL           (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG6_BIASGEN_CAL_OVR_VAL_MAX                (0x0000001FU)

#define CSL_CSI2_PHY_CFG_REG6_BIASGEN_CAL_OVR_MASK                   (0x00000020U)
#define CSL_CSI2_PHY_CFG_REG6_BIASGEN_CAL_OVR_SHIFT                  (0x00000005U)
#define CSL_CSI2_PHY_CFG_REG6_BIASGEN_CAL_OVR_RESETVAL               (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG6_BIASGEN_CAL_OVR_MAX                    (0x00000001U)

#define CSL_CSI2_PHY_CFG_REG6_RSVD1_MASK                             (0x00000040U)
#define CSL_CSI2_PHY_CFG_REG6_RSVD1_SHIFT                            (0x00000006U)
#define CSL_CSI2_PHY_CFG_REG6_RSVD1_RESETVAL                         (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG6_RSVD1_MAX                              (0x00000001U)

#define CSL_CSI2_PHY_CFG_REG6_OBSV_BIAS_CURR_DXA_MASK                (0x00000080U)
#define CSL_CSI2_PHY_CFG_REG6_OBSV_BIAS_CURR_DXA_SHIFT               (0x00000007U)
#define CSL_CSI2_PHY_CFG_REG6_OBSV_BIAS_CURR_DXA_RESETVAL            (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG6_OBSV_BIAS_CURR_DXA_MAX                 (0x00000001U)

#define CSL_CSI2_PHY_CFG_REG6_OBSV_LDO_VOLT_DYA_MASK                 (0x00000100U)
#define CSL_CSI2_PHY_CFG_REG6_OBSV_LDO_VOLT_DYA_SHIFT                (0x00000008U)
#define CSL_CSI2_PHY_CFG_REG6_OBSV_LDO_VOLT_DYA_RESETVAL             (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG6_OBSV_LDO_VOLT_DYA_MAX                  (0x00000001U)

#define CSL_CSI2_PHY_CFG_REG6_BYPASS_LDO_REG_MASK                    (0x00000200U)
#define CSL_CSI2_PHY_CFG_REG6_BYPASS_LDO_REG_SHIFT                   (0x00000009U)
#define CSL_CSI2_PHY_CFG_REG6_BYPASS_LDO_REG_RESETVAL                (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG6_BYPASS_LDO_REG_MAX                     (0x00000001U)

#define CSL_CSI2_PHY_CFG_REG6_HSCOMOOUT_MASK                         (0x00000400U)
#define CSL_CSI2_PHY_CFG_REG6_HSCOMOOUT_SHIFT                        (0x0000000AU)
#define CSL_CSI2_PHY_CFG_REG6_HSCOMOOUT_RESETVAL                     (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG6_HSCOMOOUT_MAX                          (0x00000001U)

#define CSL_CSI2_PHY_CFG_REG6_AFE_LANE_POL_MASK                      (0x00000800U)
#define CSL_CSI2_PHY_CFG_REG6_AFE_LANE_POL_SHIFT                     (0x0000000BU)
#define CSL_CSI2_PHY_CFG_REG6_AFE_LANE_POL_RESETVAL                  (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG6_AFE_LANE_POL_MAX                       (0x00000001U)

#define CSL_CSI2_PHY_CFG_REG6_AFE_LANE_SEL_MASK                      (0x000FF000U)
#define CSL_CSI2_PHY_CFG_REG6_AFE_LANE_SEL_SHIFT                     (0x0000000CU)
#define CSL_CSI2_PHY_CFG_REG6_AFE_LANE_SEL_RESETVAL                  (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG6_AFE_LANE_SEL_MAX                       (0x000000FFU)

#define CSL_CSI2_PHY_CFG_REG6_OVR_AFE_LANE_ADR_POL_MASK              (0x00100000U)
#define CSL_CSI2_PHY_CFG_REG6_OVR_AFE_LANE_ADR_POL_SHIFT             (0x00000014U)
#define CSL_CSI2_PHY_CFG_REG6_OVR_AFE_LANE_ADR_POL_RESETVAL          (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG6_OVR_AFE_LANE_ADR_POL_MAX               (0x00000001U)

#define CSL_CSI2_PHY_CFG_REG6_RSVD2_MASK                             (0xFFE00000U)
#define CSL_CSI2_PHY_CFG_REG6_RSVD2_SHIFT                            (0x00000015U)
#define CSL_CSI2_PHY_CFG_REG6_RSVD2_RESETVAL                         (0x00000000U)
#define CSL_CSI2_PHY_CFG_REG6_RSVD2_MAX                              (0x000007FFU)

#define CSL_CSI2_PHY_CFG_REG6_RESETVAL                               (0x00000000U)

/* CSI2_CTX0_TRANSCODEH */

#define CSL_CSI2_CTX0_TRANSCODEH_HSKIP_MASK                          (0x00001FFFU)
#define CSL_CSI2_CTX0_TRANSCODEH_HSKIP_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX0_TRANSCODEH_HSKIP_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX0_TRANSCODEH_HSKIP_MAX                           (0x00001FFFU)

#define CSL_CSI2_CTX0_TRANSCODEH_RSVD1_MASK                          (0x0000E000U)
#define CSL_CSI2_CTX0_TRANSCODEH_RSVD1_SHIFT                         (0x0000000DU)
#define CSL_CSI2_CTX0_TRANSCODEH_RSVD1_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX0_TRANSCODEH_RSVD1_MAX                           (0x00000007U)

#define CSL_CSI2_CTX0_TRANSCODEH_HCOUNT_MASK                         (0x1FFF0000U)
#define CSL_CSI2_CTX0_TRANSCODEH_HCOUNT_SHIFT                        (0x00000010U)
#define CSL_CSI2_CTX0_TRANSCODEH_HCOUNT_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX0_TRANSCODEH_HCOUNT_MAX                          (0x00001FFFU)

#define CSL_CSI2_CTX0_TRANSCODEH_RSVD2_MASK                          (0xE0000000U)
#define CSL_CSI2_CTX0_TRANSCODEH_RSVD2_SHIFT                         (0x0000001DU)
#define CSL_CSI2_CTX0_TRANSCODEH_RSVD2_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX0_TRANSCODEH_RSVD2_MAX                           (0x00000007U)

#define CSL_CSI2_CTX0_TRANSCODEH_RESETVAL                            (0x00000000U)

/* CSI2_CTX0_TRANSCODEV */

#define CSL_CSI2_CTX0_TRANSCODEV_VSKIP_MASK                          (0x00001FFFU)
#define CSL_CSI2_CTX0_TRANSCODEV_VSKIP_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX0_TRANSCODEV_VSKIP_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX0_TRANSCODEV_VSKIP_MAX                           (0x00001FFFU)

#define CSL_CSI2_CTX0_TRANSCODEV_RSVD1_MASK                          (0x0000E000U)
#define CSL_CSI2_CTX0_TRANSCODEV_RSVD1_SHIFT                         (0x0000000DU)
#define CSL_CSI2_CTX0_TRANSCODEV_RSVD1_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX0_TRANSCODEV_RSVD1_MAX                           (0x00000007U)

#define CSL_CSI2_CTX0_TRANSCODEV_VCOUNT_MASK                         (0x1FFF0000U)
#define CSL_CSI2_CTX0_TRANSCODEV_VCOUNT_SHIFT                        (0x00000010U)
#define CSL_CSI2_CTX0_TRANSCODEV_VCOUNT_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX0_TRANSCODEV_VCOUNT_MAX                          (0x00001FFFU)

#define CSL_CSI2_CTX0_TRANSCODEV_RSVD2_MASK                          (0xE0000000U)
#define CSL_CSI2_CTX0_TRANSCODEV_RSVD2_SHIFT                         (0x0000001DU)
#define CSL_CSI2_CTX0_TRANSCODEV_RSVD2_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX0_TRANSCODEV_RSVD2_MAX                           (0x00000007U)

#define CSL_CSI2_CTX0_TRANSCODEV_RESETVAL                            (0x00000000U)

/* CSI2_CTX1_TRANSCODEH */

#define CSL_CSI2_CTX1_TRANSCODEH_HSKIP_MASK                          (0x00001FFFU)
#define CSL_CSI2_CTX1_TRANSCODEH_HSKIP_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX1_TRANSCODEH_HSKIP_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX1_TRANSCODEH_HSKIP_MAX                           (0x00001FFFU)

#define CSL_CSI2_CTX1_TRANSCODEH_RSVD1_MASK                          (0x0000E000U)
#define CSL_CSI2_CTX1_TRANSCODEH_RSVD1_SHIFT                         (0x0000000DU)
#define CSL_CSI2_CTX1_TRANSCODEH_RSVD1_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX1_TRANSCODEH_RSVD1_MAX                           (0x00000007U)

#define CSL_CSI2_CTX1_TRANSCODEH_HCOUNT_MASK                         (0x1FFF0000U)
#define CSL_CSI2_CTX1_TRANSCODEH_HCOUNT_SHIFT                        (0x00000010U)
#define CSL_CSI2_CTX1_TRANSCODEH_HCOUNT_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX1_TRANSCODEH_HCOUNT_MAX                          (0x00001FFFU)

#define CSL_CSI2_CTX1_TRANSCODEH_RSVD2_MASK                          (0xE0000000U)
#define CSL_CSI2_CTX1_TRANSCODEH_RSVD2_SHIFT                         (0x0000001DU)
#define CSL_CSI2_CTX1_TRANSCODEH_RSVD2_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX1_TRANSCODEH_RSVD2_MAX                           (0x00000007U)

#define CSL_CSI2_CTX1_TRANSCODEH_RESETVAL                            (0x00000000U)

/* CSI2_CTX1_TRANSCODEV */

#define CSL_CSI2_CTX1_TRANSCODEV_VSKIP_MASK                          (0x00001FFFU)
#define CSL_CSI2_CTX1_TRANSCODEV_VSKIP_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX1_TRANSCODEV_VSKIP_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX1_TRANSCODEV_VSKIP_MAX                           (0x00001FFFU)

#define CSL_CSI2_CTX1_TRANSCODEV_RSVD1_MASK                          (0x0000E000U)
#define CSL_CSI2_CTX1_TRANSCODEV_RSVD1_SHIFT                         (0x0000000DU)
#define CSL_CSI2_CTX1_TRANSCODEV_RSVD1_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX1_TRANSCODEV_RSVD1_MAX                           (0x00000007U)

#define CSL_CSI2_CTX1_TRANSCODEV_VCOUNT_MASK                         (0x1FFF0000U)
#define CSL_CSI2_CTX1_TRANSCODEV_VCOUNT_SHIFT                        (0x00000010U)
#define CSL_CSI2_CTX1_TRANSCODEV_VCOUNT_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX1_TRANSCODEV_VCOUNT_MAX                          (0x00001FFFU)

#define CSL_CSI2_CTX1_TRANSCODEV_RSVD2_MASK                          (0xE0000000U)
#define CSL_CSI2_CTX1_TRANSCODEV_RSVD2_SHIFT                         (0x0000001DU)
#define CSL_CSI2_CTX1_TRANSCODEV_RSVD2_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX1_TRANSCODEV_RSVD2_MAX                           (0x00000007U)

#define CSL_CSI2_CTX1_TRANSCODEV_RESETVAL                            (0x00000000U)

/* CSI2_CTX2_TRANSCODEH */

#define CSL_CSI2_CTX2_TRANSCODEH_HSKIP_MASK                          (0x00001FFFU)
#define CSL_CSI2_CTX2_TRANSCODEH_HSKIP_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX2_TRANSCODEH_HSKIP_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX2_TRANSCODEH_HSKIP_MAX                           (0x00001FFFU)

#define CSL_CSI2_CTX2_TRANSCODEH_RSVD1_MASK                          (0x0000E000U)
#define CSL_CSI2_CTX2_TRANSCODEH_RSVD1_SHIFT                         (0x0000000DU)
#define CSL_CSI2_CTX2_TRANSCODEH_RSVD1_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX2_TRANSCODEH_RSVD1_MAX                           (0x00000007U)

#define CSL_CSI2_CTX2_TRANSCODEH_HCOUNT_MASK                         (0x1FFF0000U)
#define CSL_CSI2_CTX2_TRANSCODEH_HCOUNT_SHIFT                        (0x00000010U)
#define CSL_CSI2_CTX2_TRANSCODEH_HCOUNT_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX2_TRANSCODEH_HCOUNT_MAX                          (0x00001FFFU)

#define CSL_CSI2_CTX2_TRANSCODEH_RSVD2_MASK                          (0xE0000000U)
#define CSL_CSI2_CTX2_TRANSCODEH_RSVD2_SHIFT                         (0x0000001DU)
#define CSL_CSI2_CTX2_TRANSCODEH_RSVD2_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX2_TRANSCODEH_RSVD2_MAX                           (0x00000007U)

#define CSL_CSI2_CTX2_TRANSCODEH_RESETVAL                            (0x00000000U)

/* CSI2_CTX2_TRANSCODEV */

#define CSL_CSI2_CTX2_TRANSCODEV_VSKIP_MASK                          (0x00001FFFU)
#define CSL_CSI2_CTX2_TRANSCODEV_VSKIP_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX2_TRANSCODEV_VSKIP_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX2_TRANSCODEV_VSKIP_MAX                           (0x00001FFFU)

#define CSL_CSI2_CTX2_TRANSCODEV_RSVD1_MASK                          (0x0000E000U)
#define CSL_CSI2_CTX2_TRANSCODEV_RSVD1_SHIFT                         (0x0000000DU)
#define CSL_CSI2_CTX2_TRANSCODEV_RSVD1_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX2_TRANSCODEV_RSVD1_MAX                           (0x00000007U)

#define CSL_CSI2_CTX2_TRANSCODEV_VCOUNT_MASK                         (0x1FFF0000U)
#define CSL_CSI2_CTX2_TRANSCODEV_VCOUNT_SHIFT                        (0x00000010U)
#define CSL_CSI2_CTX2_TRANSCODEV_VCOUNT_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX2_TRANSCODEV_VCOUNT_MAX                          (0x00001FFFU)

#define CSL_CSI2_CTX2_TRANSCODEV_RSVD2_MASK                          (0xE0000000U)
#define CSL_CSI2_CTX2_TRANSCODEV_RSVD2_SHIFT                         (0x0000001DU)
#define CSL_CSI2_CTX2_TRANSCODEV_RSVD2_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX2_TRANSCODEV_RSVD2_MAX                           (0x00000007U)

#define CSL_CSI2_CTX2_TRANSCODEV_RESETVAL                            (0x00000000U)

/* CSI2_CTX3_TRANSCODEH */

#define CSL_CSI2_CTX3_TRANSCODEH_HSKIP_MASK                          (0x00001FFFU)
#define CSL_CSI2_CTX3_TRANSCODEH_HSKIP_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX3_TRANSCODEH_HSKIP_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX3_TRANSCODEH_HSKIP_MAX                           (0x00001FFFU)

#define CSL_CSI2_CTX3_TRANSCODEH_RSVD1_MASK                          (0x0000E000U)
#define CSL_CSI2_CTX3_TRANSCODEH_RSVD1_SHIFT                         (0x0000000DU)
#define CSL_CSI2_CTX3_TRANSCODEH_RSVD1_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX3_TRANSCODEH_RSVD1_MAX                           (0x00000007U)

#define CSL_CSI2_CTX3_TRANSCODEH_HCOUNT_MASK                         (0x1FFF0000U)
#define CSL_CSI2_CTX3_TRANSCODEH_HCOUNT_SHIFT                        (0x00000010U)
#define CSL_CSI2_CTX3_TRANSCODEH_HCOUNT_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX3_TRANSCODEH_HCOUNT_MAX                          (0x00001FFFU)

#define CSL_CSI2_CTX3_TRANSCODEH_RSVD2_MASK                          (0xE0000000U)
#define CSL_CSI2_CTX3_TRANSCODEH_RSVD2_SHIFT                         (0x0000001DU)
#define CSL_CSI2_CTX3_TRANSCODEH_RSVD2_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX3_TRANSCODEH_RSVD2_MAX                           (0x00000007U)

#define CSL_CSI2_CTX3_TRANSCODEH_RESETVAL                            (0x00000000U)

/* CSI2_CTX3_TRANSCODEV */

#define CSL_CSI2_CTX3_TRANSCODEV_VSKIP_MASK                          (0x00001FFFU)
#define CSL_CSI2_CTX3_TRANSCODEV_VSKIP_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX3_TRANSCODEV_VSKIP_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX3_TRANSCODEV_VSKIP_MAX                           (0x00001FFFU)

#define CSL_CSI2_CTX3_TRANSCODEV_RSVD1_MASK                          (0x0000E000U)
#define CSL_CSI2_CTX3_TRANSCODEV_RSVD1_SHIFT                         (0x0000000DU)
#define CSL_CSI2_CTX3_TRANSCODEV_RSVD1_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX3_TRANSCODEV_RSVD1_MAX                           (0x00000007U)

#define CSL_CSI2_CTX3_TRANSCODEV_VCOUNT_MASK                         (0x1FFF0000U)
#define CSL_CSI2_CTX3_TRANSCODEV_VCOUNT_SHIFT                        (0x00000010U)
#define CSL_CSI2_CTX3_TRANSCODEV_VCOUNT_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX3_TRANSCODEV_VCOUNT_MAX                          (0x00001FFFU)

#define CSL_CSI2_CTX3_TRANSCODEV_RSVD2_MASK                          (0xE0000000U)
#define CSL_CSI2_CTX3_TRANSCODEV_RSVD2_SHIFT                         (0x0000001DU)
#define CSL_CSI2_CTX3_TRANSCODEV_RSVD2_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX3_TRANSCODEV_RSVD2_MAX                           (0x00000007U)

#define CSL_CSI2_CTX3_TRANSCODEV_RESETVAL                            (0x00000000U)

/* CSI2_CTX4_TRANSCODEH */

#define CSL_CSI2_CTX4_TRANSCODEH_HSKIP_MASK                          (0x00001FFFU)
#define CSL_CSI2_CTX4_TRANSCODEH_HSKIP_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX4_TRANSCODEH_HSKIP_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX4_TRANSCODEH_HSKIP_MAX                           (0x00001FFFU)

#define CSL_CSI2_CTX4_TRANSCODEH_RSVD1_MASK                          (0x0000E000U)
#define CSL_CSI2_CTX4_TRANSCODEH_RSVD1_SHIFT                         (0x0000000DU)
#define CSL_CSI2_CTX4_TRANSCODEH_RSVD1_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX4_TRANSCODEH_RSVD1_MAX                           (0x00000007U)

#define CSL_CSI2_CTX4_TRANSCODEH_HCOUNT_MASK                         (0x1FFF0000U)
#define CSL_CSI2_CTX4_TRANSCODEH_HCOUNT_SHIFT                        (0x00000010U)
#define CSL_CSI2_CTX4_TRANSCODEH_HCOUNT_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX4_TRANSCODEH_HCOUNT_MAX                          (0x00001FFFU)

#define CSL_CSI2_CTX4_TRANSCODEH_RSVD2_MASK                          (0xE0000000U)
#define CSL_CSI2_CTX4_TRANSCODEH_RSVD2_SHIFT                         (0x0000001DU)
#define CSL_CSI2_CTX4_TRANSCODEH_RSVD2_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX4_TRANSCODEH_RSVD2_MAX                           (0x00000007U)

#define CSL_CSI2_CTX4_TRANSCODEH_RESETVAL                            (0x00000000U)

/* CSI2_CTX4_TRANSCODEV */

#define CSL_CSI2_CTX4_TRANSCODEV_VSKIP_MASK                          (0x00001FFFU)
#define CSL_CSI2_CTX4_TRANSCODEV_VSKIP_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX4_TRANSCODEV_VSKIP_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX4_TRANSCODEV_VSKIP_MAX                           (0x00001FFFU)

#define CSL_CSI2_CTX4_TRANSCODEV_RSVD1_MASK                          (0x0000E000U)
#define CSL_CSI2_CTX4_TRANSCODEV_RSVD1_SHIFT                         (0x0000000DU)
#define CSL_CSI2_CTX4_TRANSCODEV_RSVD1_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX4_TRANSCODEV_RSVD1_MAX                           (0x00000007U)

#define CSL_CSI2_CTX4_TRANSCODEV_VCOUNT_MASK                         (0x1FFF0000U)
#define CSL_CSI2_CTX4_TRANSCODEV_VCOUNT_SHIFT                        (0x00000010U)
#define CSL_CSI2_CTX4_TRANSCODEV_VCOUNT_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX4_TRANSCODEV_VCOUNT_MAX                          (0x00001FFFU)

#define CSL_CSI2_CTX4_TRANSCODEV_RSVD2_MASK                          (0xE0000000U)
#define CSL_CSI2_CTX4_TRANSCODEV_RSVD2_SHIFT                         (0x0000001DU)
#define CSL_CSI2_CTX4_TRANSCODEV_RSVD2_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX4_TRANSCODEV_RSVD2_MAX                           (0x00000007U)

#define CSL_CSI2_CTX4_TRANSCODEV_RESETVAL                            (0x00000000U)

/* CSI2_CTX5_TRANSCODEH */

#define CSL_CSI2_CTX5_TRANSCODEH_HSKIP_MASK                          (0x00001FFFU)
#define CSL_CSI2_CTX5_TRANSCODEH_HSKIP_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX5_TRANSCODEH_HSKIP_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX5_TRANSCODEH_HSKIP_MAX                           (0x00001FFFU)

#define CSL_CSI2_CTX5_TRANSCODEH_RSVD1_MASK                          (0x0000E000U)
#define CSL_CSI2_CTX5_TRANSCODEH_RSVD1_SHIFT                         (0x0000000DU)
#define CSL_CSI2_CTX5_TRANSCODEH_RSVD1_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX5_TRANSCODEH_RSVD1_MAX                           (0x00000007U)

#define CSL_CSI2_CTX5_TRANSCODEH_HCOUNT_MASK                         (0x1FFF0000U)
#define CSL_CSI2_CTX5_TRANSCODEH_HCOUNT_SHIFT                        (0x00000010U)
#define CSL_CSI2_CTX5_TRANSCODEH_HCOUNT_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX5_TRANSCODEH_HCOUNT_MAX                          (0x00001FFFU)

#define CSL_CSI2_CTX5_TRANSCODEH_RSVD2_MASK                          (0xE0000000U)
#define CSL_CSI2_CTX5_TRANSCODEH_RSVD2_SHIFT                         (0x0000001DU)
#define CSL_CSI2_CTX5_TRANSCODEH_RSVD2_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX5_TRANSCODEH_RSVD2_MAX                           (0x00000007U)

#define CSL_CSI2_CTX5_TRANSCODEH_RESETVAL                            (0x00000000U)

/* CSI2_CTX5_TRANSCODEV */

#define CSL_CSI2_CTX5_TRANSCODEV_VSKIP_MASK                          (0x00001FFFU)
#define CSL_CSI2_CTX5_TRANSCODEV_VSKIP_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX5_TRANSCODEV_VSKIP_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX5_TRANSCODEV_VSKIP_MAX                           (0x00001FFFU)

#define CSL_CSI2_CTX5_TRANSCODEV_RSVD1_MASK                          (0x0000E000U)
#define CSL_CSI2_CTX5_TRANSCODEV_RSVD1_SHIFT                         (0x0000000DU)
#define CSL_CSI2_CTX5_TRANSCODEV_RSVD1_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX5_TRANSCODEV_RSVD1_MAX                           (0x00000007U)

#define CSL_CSI2_CTX5_TRANSCODEV_VCOUNT_MASK                         (0x1FFF0000U)
#define CSL_CSI2_CTX5_TRANSCODEV_VCOUNT_SHIFT                        (0x00000010U)
#define CSL_CSI2_CTX5_TRANSCODEV_VCOUNT_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX5_TRANSCODEV_VCOUNT_MAX                          (0x00001FFFU)

#define CSL_CSI2_CTX5_TRANSCODEV_RSVD2_MASK                          (0xE0000000U)
#define CSL_CSI2_CTX5_TRANSCODEV_RSVD2_SHIFT                         (0x0000001DU)
#define CSL_CSI2_CTX5_TRANSCODEV_RSVD2_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX5_TRANSCODEV_RSVD2_MAX                           (0x00000007U)

#define CSL_CSI2_CTX5_TRANSCODEV_RESETVAL                            (0x00000000U)

/* CSI2_CTX6_TRANSCODEH */

#define CSL_CSI2_CTX6_TRANSCODEH_HSKIP_MASK                          (0x00001FFFU)
#define CSL_CSI2_CTX6_TRANSCODEH_HSKIP_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX6_TRANSCODEH_HSKIP_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX6_TRANSCODEH_HSKIP_MAX                           (0x00001FFFU)

#define CSL_CSI2_CTX6_TRANSCODEH_RSVD1_MASK                          (0x0000E000U)
#define CSL_CSI2_CTX6_TRANSCODEH_RSVD1_SHIFT                         (0x0000000DU)
#define CSL_CSI2_CTX6_TRANSCODEH_RSVD1_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX6_TRANSCODEH_RSVD1_MAX                           (0x00000007U)

#define CSL_CSI2_CTX6_TRANSCODEH_HCOUNT_MASK                         (0x1FFF0000U)
#define CSL_CSI2_CTX6_TRANSCODEH_HCOUNT_SHIFT                        (0x00000010U)
#define CSL_CSI2_CTX6_TRANSCODEH_HCOUNT_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX6_TRANSCODEH_HCOUNT_MAX                          (0x00001FFFU)

#define CSL_CSI2_CTX6_TRANSCODEH_RSVD2_MASK                          (0xE0000000U)
#define CSL_CSI2_CTX6_TRANSCODEH_RSVD2_SHIFT                         (0x0000001DU)
#define CSL_CSI2_CTX6_TRANSCODEH_RSVD2_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX6_TRANSCODEH_RSVD2_MAX                           (0x00000007U)

#define CSL_CSI2_CTX6_TRANSCODEH_RESETVAL                            (0x00000000U)

/* CSI2_CTX6_TRANSCODEV */

#define CSL_CSI2_CTX6_TRANSCODEV_VSKIP_MASK                          (0x00001FFFU)
#define CSL_CSI2_CTX6_TRANSCODEV_VSKIP_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX6_TRANSCODEV_VSKIP_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX6_TRANSCODEV_VSKIP_MAX                           (0x00001FFFU)

#define CSL_CSI2_CTX6_TRANSCODEV_RSVD1_MASK                          (0x0000E000U)
#define CSL_CSI2_CTX6_TRANSCODEV_RSVD1_SHIFT                         (0x0000000DU)
#define CSL_CSI2_CTX6_TRANSCODEV_RSVD1_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX6_TRANSCODEV_RSVD1_MAX                           (0x00000007U)

#define CSL_CSI2_CTX6_TRANSCODEV_VCOUNT_MASK                         (0x1FFF0000U)
#define CSL_CSI2_CTX6_TRANSCODEV_VCOUNT_SHIFT                        (0x00000010U)
#define CSL_CSI2_CTX6_TRANSCODEV_VCOUNT_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX6_TRANSCODEV_VCOUNT_MAX                          (0x00001FFFU)

#define CSL_CSI2_CTX6_TRANSCODEV_RSVD2_MASK                          (0xE0000000U)
#define CSL_CSI2_CTX6_TRANSCODEV_RSVD2_SHIFT                         (0x0000001DU)
#define CSL_CSI2_CTX6_TRANSCODEV_RSVD2_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX6_TRANSCODEV_RSVD2_MAX                           (0x00000007U)

#define CSL_CSI2_CTX6_TRANSCODEV_RESETVAL                            (0x00000000U)

/* CSI2_CTX7_TRANSCODEH */

#define CSL_CSI2_CTX7_TRANSCODEH_HSKIP_MASK                          (0x00001FFFU)
#define CSL_CSI2_CTX7_TRANSCODEH_HSKIP_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX7_TRANSCODEH_HSKIP_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX7_TRANSCODEH_HSKIP_MAX                           (0x00001FFFU)

#define CSL_CSI2_CTX7_TRANSCODEH_RSVD1_MASK                          (0x0000E000U)
#define CSL_CSI2_CTX7_TRANSCODEH_RSVD1_SHIFT                         (0x0000000DU)
#define CSL_CSI2_CTX7_TRANSCODEH_RSVD1_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX7_TRANSCODEH_RSVD1_MAX                           (0x00000007U)

#define CSL_CSI2_CTX7_TRANSCODEH_HCOUNT_MASK                         (0x1FFF0000U)
#define CSL_CSI2_CTX7_TRANSCODEH_HCOUNT_SHIFT                        (0x00000010U)
#define CSL_CSI2_CTX7_TRANSCODEH_HCOUNT_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX7_TRANSCODEH_HCOUNT_MAX                          (0x00001FFFU)

#define CSL_CSI2_CTX7_TRANSCODEH_RSVD2_MASK                          (0xE0000000U)
#define CSL_CSI2_CTX7_TRANSCODEH_RSVD2_SHIFT                         (0x0000001DU)
#define CSL_CSI2_CTX7_TRANSCODEH_RSVD2_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX7_TRANSCODEH_RSVD2_MAX                           (0x00000007U)

#define CSL_CSI2_CTX7_TRANSCODEH_RESETVAL                            (0x00000000U)

/* CSI2_CTX7_TRANSCODEV */

#define CSL_CSI2_CTX7_TRANSCODEV_VSKIP_MASK                          (0x00001FFFU)
#define CSL_CSI2_CTX7_TRANSCODEV_VSKIP_SHIFT                         (0x00000000U)
#define CSL_CSI2_CTX7_TRANSCODEV_VSKIP_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX7_TRANSCODEV_VSKIP_MAX                           (0x00001FFFU)

#define CSL_CSI2_CTX7_TRANSCODEV_RSVD1_MASK                          (0x0000E000U)
#define CSL_CSI2_CTX7_TRANSCODEV_RSVD1_SHIFT                         (0x0000000DU)
#define CSL_CSI2_CTX7_TRANSCODEV_RSVD1_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX7_TRANSCODEV_RSVD1_MAX                           (0x00000007U)

#define CSL_CSI2_CTX7_TRANSCODEV_VCOUNT_MASK                         (0x1FFF0000U)
#define CSL_CSI2_CTX7_TRANSCODEV_VCOUNT_SHIFT                        (0x00000010U)
#define CSL_CSI2_CTX7_TRANSCODEV_VCOUNT_RESETVAL                     (0x00000000U)
#define CSL_CSI2_CTX7_TRANSCODEV_VCOUNT_MAX                          (0x00001FFFU)

#define CSL_CSI2_CTX7_TRANSCODEV_RSVD2_MASK                          (0xE0000000U)
#define CSL_CSI2_CTX7_TRANSCODEV_RSVD2_SHIFT                         (0x0000001DU)
#define CSL_CSI2_CTX7_TRANSCODEV_RSVD2_RESETVAL                      (0x00000000U)
#define CSL_CSI2_CTX7_TRANSCODEV_RSVD2_MAX                           (0x00000007U)

#define CSL_CSI2_CTX7_TRANSCODEV_RESETVAL                            (0x00000000U)

/*! \brief define for efficient programming of complex IO IRQENABLE register
       bit positions across lanes */
#define CSL_CSI2_COMPLEXIO_IRQENABLE_INTER_LANE_SHIFT_DISTANCE \
            (CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL2_SHIFT - \
             CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL1_SHIFT)

/*! \brief check assumption of constancy of \
       \ref CSL_CSI2_COMPLEXIO_IRQENABLE_INTER_LANE_SHIFT_DISTANCE */
#if ( (CSL_CSI2_COMPLEXIO_IRQENABLE_INTER_LANE_SHIFT_DISTANCE != \
        (CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL3_SHIFT -    \
         CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL2_SHIFT)) || \
      (CSL_CSI2_COMPLEXIO_IRQENABLE_INTER_LANE_SHIFT_DISTANCE != \
        (CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL4_SHIFT -    \
         CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL3_SHIFT)) || \
      (CSL_CSI2_COMPLEXIO_IRQENABLE_INTER_LANE_SHIFT_DISTANCE != \
        (CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL5_SHIFT -    \
         CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRCONTROL4_SHIFT)) || \
      (CSL_CSI2_COMPLEXIO_IRQENABLE_INTER_LANE_SHIFT_DISTANCE != \
        (CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC2_SHIFT -        \
         CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC1_SHIFT)) ||     \
      (CSL_CSI2_COMPLEXIO_IRQENABLE_INTER_LANE_SHIFT_DISTANCE != \
        (CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC3_SHIFT -        \
         CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC2_SHIFT)) ||     \
      (CSL_CSI2_COMPLEXIO_IRQENABLE_INTER_LANE_SHIFT_DISTANCE != \
        (CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC4_SHIFT -        \
         CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC3_SHIFT)) ||     \
      (CSL_CSI2_COMPLEXIO_IRQENABLE_INTER_LANE_SHIFT_DISTANCE != \
        (CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC5_SHIFT -        \
         CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRESC4_SHIFT)) ||     \
      (CSL_CSI2_COMPLEXIO_IRQENABLE_INTER_LANE_SHIFT_DISTANCE != \
        (CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS2_SHIFT -      \
         CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS1_SHIFT)) ||   \
      (CSL_CSI2_COMPLEXIO_IRQENABLE_INTER_LANE_SHIFT_DISTANCE != \
        (CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS3_SHIFT -      \
         CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS2_SHIFT)) ||   \
      (CSL_CSI2_COMPLEXIO_IRQENABLE_INTER_LANE_SHIFT_DISTANCE != \
        (CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS4_SHIFT -      \
         CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS3_SHIFT)) ||   \
      (CSL_CSI2_COMPLEXIO_IRQENABLE_INTER_LANE_SHIFT_DISTANCE != \
        (CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS5_SHIFT -      \
         CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTHS4_SHIFT)) ||   \
      (CSL_CSI2_COMPLEXIO_IRQENABLE_INTER_LANE_SHIFT_DISTANCE != \
        (CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS2_SHIFT -  \
         CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS1_SHIFT)) || \
      (CSL_CSI2_COMPLEXIO_IRQENABLE_INTER_LANE_SHIFT_DISTANCE !=   \
        (CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS3_SHIFT -    \
         CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS2_SHIFT)) || \
      (CSL_CSI2_COMPLEXIO_IRQENABLE_INTER_LANE_SHIFT_DISTANCE !=   \
        (CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS4_SHIFT -    \
         CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS3_SHIFT)) || \
      (CSL_CSI2_COMPLEXIO_IRQENABLE_INTER_LANE_SHIFT_DISTANCE !=   \
        (CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS5_SHIFT -    \
         CSL_CSI2_COMPLEXIO1_IRQENABLE_ERRSOTSYNCHS4_SHIFT)) || \
      (CSL_CSI2_COMPLEXIO_IRQENABLE_INTER_LANE_SHIFT_DISTANCE !=   \
        (CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM2_SHIFT -       \
         CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM1_SHIFT)) ||    \
      (CSL_CSI2_COMPLEXIO_IRQENABLE_INTER_LANE_SHIFT_DISTANCE !=   \
        (CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM2_SHIFT -       \
         CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM1_SHIFT)) ||    \
      (CSL_CSI2_COMPLEXIO_IRQENABLE_INTER_LANE_SHIFT_DISTANCE !=   \
        (CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM3_SHIFT -       \
         CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM2_SHIFT)) ||    \
      (CSL_CSI2_COMPLEXIO_IRQENABLE_INTER_LANE_SHIFT_DISTANCE !=   \
        (CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM4_SHIFT -       \
         CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM3_SHIFT)) ||    \
      (CSL_CSI2_COMPLEXIO_IRQENABLE_INTER_LANE_SHIFT_DISTANCE !=   \
        (CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM5_SHIFT -       \
         CSL_CSI2_COMPLEXIO1_IRQENABLE_STATEULPM4_SHIFT))       \
     )
#error CSL_CSI2_COMPLEXIO_IRQENABLE_INTER_LANE_SHIFT_DISTANCE assumption violated
#endif

/*! \brief define for efficient programming of complex IO IRQSTATUS register
       bit positions across lanes */
#define CSL_CSI2_COMPLEXIO_IRQSTATUS_INTER_LANE_SHIFT_DISTANCE \
            (CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL2_SHIFT - \
             CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL1_SHIFT)

/*! \brief check assumption of constancy of \
       \ref CSL_CSI2_COMPLEXIO_IRQSTATUS_INTER_LANE_SHIFT_DISTANCE */
#if ( (CSL_CSI2_COMPLEXIO_IRQSTATUS_INTER_LANE_SHIFT_DISTANCE != \
        (CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL3_SHIFT -    \
         CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL2_SHIFT)) || \
      (CSL_CSI2_COMPLEXIO_IRQSTATUS_INTER_LANE_SHIFT_DISTANCE != \
        (CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL4_SHIFT -    \
         CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL3_SHIFT)) || \
      (CSL_CSI2_COMPLEXIO_IRQSTATUS_INTER_LANE_SHIFT_DISTANCE != \
        (CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL5_SHIFT -    \
         CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRCONTROL4_SHIFT)) || \
      (CSL_CSI2_COMPLEXIO_IRQSTATUS_INTER_LANE_SHIFT_DISTANCE != \
        (CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC2_SHIFT -        \
         CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC1_SHIFT)) ||     \
      (CSL_CSI2_COMPLEXIO_IRQSTATUS_INTER_LANE_SHIFT_DISTANCE != \
        (CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC3_SHIFT -        \
         CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC2_SHIFT)) ||     \
      (CSL_CSI2_COMPLEXIO_IRQSTATUS_INTER_LANE_SHIFT_DISTANCE != \
        (CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC4_SHIFT -        \
         CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC3_SHIFT)) ||     \
      (CSL_CSI2_COMPLEXIO_IRQSTATUS_INTER_LANE_SHIFT_DISTANCE != \
        (CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC5_SHIFT -        \
         CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRESC4_SHIFT)) ||     \
      (CSL_CSI2_COMPLEXIO_IRQSTATUS_INTER_LANE_SHIFT_DISTANCE != \
        (CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS2_SHIFT -      \
         CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS1_SHIFT)) ||   \
      (CSL_CSI2_COMPLEXIO_IRQSTATUS_INTER_LANE_SHIFT_DISTANCE != \
        (CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS3_SHIFT -      \
         CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS2_SHIFT)) ||   \
      (CSL_CSI2_COMPLEXIO_IRQSTATUS_INTER_LANE_SHIFT_DISTANCE != \
        (CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS4_SHIFT -      \
         CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS3_SHIFT)) ||   \
      (CSL_CSI2_COMPLEXIO_IRQSTATUS_INTER_LANE_SHIFT_DISTANCE != \
        (CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS5_SHIFT -      \
         CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTHS4_SHIFT)) ||   \
      (CSL_CSI2_COMPLEXIO_IRQSTATUS_INTER_LANE_SHIFT_DISTANCE != \
        (CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS2_SHIFT -  \
         CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS1_SHIFT)) || \
      (CSL_CSI2_COMPLEXIO_IRQSTATUS_INTER_LANE_SHIFT_DISTANCE !=   \
        (CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS3_SHIFT -    \
         CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS2_SHIFT)) || \
      (CSL_CSI2_COMPLEXIO_IRQSTATUS_INTER_LANE_SHIFT_DISTANCE !=   \
        (CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS4_SHIFT -    \
         CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS3_SHIFT)) || \
      (CSL_CSI2_COMPLEXIO_IRQSTATUS_INTER_LANE_SHIFT_DISTANCE !=   \
        (CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS5_SHIFT -    \
         CSL_CSI2_COMPLEXIO1_IRQSTATUS_ERRSOTSYNCHS4_SHIFT)) || \
      (CSL_CSI2_COMPLEXIO_IRQSTATUS_INTER_LANE_SHIFT_DISTANCE !=   \
        (CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM2_SHIFT -       \
         CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM1_SHIFT)) ||    \
      (CSL_CSI2_COMPLEXIO_IRQSTATUS_INTER_LANE_SHIFT_DISTANCE !=   \
        (CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM2_SHIFT -       \
         CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM1_SHIFT)) ||    \
      (CSL_CSI2_COMPLEXIO_IRQSTATUS_INTER_LANE_SHIFT_DISTANCE !=   \
        (CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM3_SHIFT -       \
         CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM2_SHIFT)) ||    \
      (CSL_CSI2_COMPLEXIO_IRQSTATUS_INTER_LANE_SHIFT_DISTANCE !=   \
        (CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM4_SHIFT -       \
         CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM3_SHIFT)) ||    \
      (CSL_CSI2_COMPLEXIO_IRQSTATUS_INTER_LANE_SHIFT_DISTANCE !=   \
        (CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM5_SHIFT -       \
         CSL_CSI2_COMPLEXIO1_IRQSTATUS_STATEULPM4_SHIFT))       \
     )
#error CSL_CSI2_COMPLEXIO_IRQSTATUS_INTER_LANE_SHIFT_DISTANCE assumption violated
#endif

/*! \brief define for efficient programming of common IRQENABLE register's
       context IRQ bit positions across contexts */
#define CSL_CSI2_INTER_CONTEXT_IRQENABLE_SHIFT_DISTANCE  (CSL_CSI2_IRQENABLE_CONTEXT1_SHIFT - \
                                                          CSL_CSI2_IRQENABLE_CONTEXT0_SHIFT)

/*! \brief check assumption of constancy of
       \ref CSL_CSI2_INTER_CONTEXT_IRQENABLE_SHIFT_DISTANCE */
#if ( (CSL_CSI2_INTER_CONTEXT_IRQENABLE_SHIFT_DISTANCE != \
        (CSL_CSI2_IRQENABLE_CONTEXT2_SHIFT  - CSL_CSI2_IRQENABLE_CONTEXT1_SHIFT)) || \
      (CSL_CSI2_INTER_CONTEXT_IRQENABLE_SHIFT_DISTANCE != \
        (CSL_CSI2_IRQENABLE_CONTEXT3_SHIFT  - CSL_CSI2_IRQENABLE_CONTEXT2_SHIFT)) || \
      (CSL_CSI2_INTER_CONTEXT_IRQENABLE_SHIFT_DISTANCE != \
        (CSL_CSI2_IRQENABLE_CONTEXT4_SHIFT  - CSL_CSI2_IRQENABLE_CONTEXT3_SHIFT)) || \
      (CSL_CSI2_INTER_CONTEXT_IRQENABLE_SHIFT_DISTANCE != \
        (CSL_CSI2_IRQENABLE_CONTEXT5_SHIFT  - CSL_CSI2_IRQENABLE_CONTEXT4_SHIFT)) || \
      (CSL_CSI2_INTER_CONTEXT_IRQENABLE_SHIFT_DISTANCE != \
        (CSL_CSI2_IRQENABLE_CONTEXT6_SHIFT  - CSL_CSI2_IRQENABLE_CONTEXT5_SHIFT)) || \
      (CSL_CSI2_INTER_CONTEXT_IRQENABLE_SHIFT_DISTANCE != \
        (CSL_CSI2_IRQENABLE_CONTEXT7_SHIFT  - CSL_CSI2_IRQENABLE_CONTEXT6_SHIFT) \
    ) \
)
#error CSL_CSI2_INTER_CONTEXT_IRQENABLE_SHIFT_DISTANCE assumption violated
#endif

/*! \brief define for efficient programming of common IRQSTATUS register's
       context IRQ bit positions across contexts */
#define CSL_CSI2_INTER_CONTEXT_IRQSTATUS_SHIFT_DISTANCE  (CSL_CSI2_IRQSTATUS_CONTEXT1_SHIFT - \
                                                          CSL_CSI2_IRQSTATUS_CONTEXT0_SHIFT)

/*! \brief check assumption of constancy of
       \ref CSL_CSI2_INTER_CONTEXT_IRQSTATUS_SHIFT_DISTANCE */
#if ( (CSL_CSI2_INTER_CONTEXT_IRQSTATUS_SHIFT_DISTANCE != \
        (CSL_CSI2_IRQSTATUS_CONTEXT2_SHIFT  - CSL_CSI2_IRQSTATUS_CONTEXT1_SHIFT)) || \
      (CSL_CSI2_INTER_CONTEXT_IRQSTATUS_SHIFT_DISTANCE != \
        (CSL_CSI2_IRQSTATUS_CONTEXT3_SHIFT  - CSL_CSI2_IRQSTATUS_CONTEXT2_SHIFT)) || \
      (CSL_CSI2_INTER_CONTEXT_IRQSTATUS_SHIFT_DISTANCE != \
        (CSL_CSI2_IRQSTATUS_CONTEXT4_SHIFT  - CSL_CSI2_IRQSTATUS_CONTEXT3_SHIFT)) || \
      (CSL_CSI2_INTER_CONTEXT_IRQSTATUS_SHIFT_DISTANCE != \
        (CSL_CSI2_IRQSTATUS_CONTEXT5_SHIFT  - CSL_CSI2_IRQSTATUS_CONTEXT4_SHIFT)) || \
      (CSL_CSI2_INTER_CONTEXT_IRQSTATUS_SHIFT_DISTANCE != \
        (CSL_CSI2_IRQSTATUS_CONTEXT6_SHIFT  - CSL_CSI2_IRQSTATUS_CONTEXT5_SHIFT)) || \
      (CSL_CSI2_INTER_CONTEXT_IRQSTATUS_SHIFT_DISTANCE != \
        (CSL_CSI2_IRQSTATUS_CONTEXT7_SHIFT  - CSL_CSI2_IRQSTATUS_CONTEXT6_SHIFT) \
    ) \
)
#error CSL_CSI2_INTER_CONTEXT_IRQSTATUS_SHIFT_DISTANCE assumption violated
#endif

/*! \brief define for efficient programming of several context registers
       -- CTRL1, CTRL2, CTRL3, DAT_OFST, DAT_PING_ADDR, DAT_PONG_ADDR, IRQENABLE,
        IRQSTATUS -- across contexts */
#define CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE   (CSL_CSI2_CTX1_CTRL1 -    \
                                                CSL_CSI2_CTX0_CTRL1)

/*! \brief check assumption of constancy of
       \ref CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE */
#if ( (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX2_CTRL1 -    \
                                                CSL_CSI2_CTX1_CTRL1)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX3_CTRL1 -    \
                                                CSL_CSI2_CTX2_CTRL1)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX4_CTRL1 -    \
                                                CSL_CSI2_CTX3_CTRL1)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX5_CTRL1 -    \
                                                CSL_CSI2_CTX4_CTRL1)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX6_CTRL1 -    \
                                                CSL_CSI2_CTX5_CTRL1)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX7_CTRL1 -    \
                                                CSL_CSI2_CTX6_CTRL1)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX1_CTRL2 -    \
                                                CSL_CSI2_CTX0_CTRL2)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX2_CTRL2 -    \
                                                CSL_CSI2_CTX1_CTRL2)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX3_CTRL2 -    \
                                                CSL_CSI2_CTX2_CTRL2)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX4_CTRL2 -    \
                                                CSL_CSI2_CTX3_CTRL2)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX5_CTRL2 -    \
                                                CSL_CSI2_CTX4_CTRL2)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX6_CTRL2 -    \
                                                CSL_CSI2_CTX5_CTRL2)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX7_CTRL2 -    \
                                                CSL_CSI2_CTX6_CTRL2)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX1_CTRL3 -    \
                                                CSL_CSI2_CTX0_CTRL3)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX2_CTRL3 -    \
                                                CSL_CSI2_CTX1_CTRL3)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX3_CTRL3 -    \
                                                CSL_CSI2_CTX2_CTRL3)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX4_CTRL3 -    \
                                                CSL_CSI2_CTX3_CTRL3)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX5_CTRL3 -    \
                                                CSL_CSI2_CTX4_CTRL3)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX6_CTRL3 -    \
                                                CSL_CSI2_CTX5_CTRL3)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX7_CTRL3 -    \
                                                CSL_CSI2_CTX6_CTRL3)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX1_DAT_OFST -    \
                                                CSL_CSI2_CTX0_DAT_OFST)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX2_DAT_OFST -    \
                                                CSL_CSI2_CTX1_DAT_OFST)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX3_DAT_OFST -    \
                                                CSL_CSI2_CTX2_DAT_OFST)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX4_DAT_OFST -    \
                                                CSL_CSI2_CTX3_DAT_OFST)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX5_DAT_OFST -    \
                                                CSL_CSI2_CTX4_DAT_OFST)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX6_DAT_OFST -    \
                                                CSL_CSI2_CTX5_DAT_OFST)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX7_DAT_OFST -    \
                                                CSL_CSI2_CTX6_DAT_OFST)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX1_DAT_PING_ADDR -    \
                                                CSL_CSI2_CTX0_DAT_PING_ADDR)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX2_DAT_PING_ADDR -    \
                                                CSL_CSI2_CTX1_DAT_PING_ADDR)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX3_DAT_PING_ADDR -    \
                                                CSL_CSI2_CTX2_DAT_PING_ADDR)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX4_DAT_PING_ADDR -    \
                                                CSL_CSI2_CTX3_DAT_PING_ADDR)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX5_DAT_PING_ADDR -    \
                                                CSL_CSI2_CTX4_DAT_PING_ADDR)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX6_DAT_PING_ADDR -    \
                                                CSL_CSI2_CTX5_DAT_PING_ADDR)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX7_DAT_PING_ADDR -    \
                                                CSL_CSI2_CTX6_DAT_PING_ADDR)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX1_DAT_PONG_ADDR -    \
                                                CSL_CSI2_CTX0_DAT_PONG_ADDR)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX2_DAT_PONG_ADDR -    \
                                                CSL_CSI2_CTX1_DAT_PONG_ADDR)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX3_DAT_PONG_ADDR -    \
                                                CSL_CSI2_CTX2_DAT_PONG_ADDR)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX4_DAT_PONG_ADDR -    \
                                                CSL_CSI2_CTX3_DAT_PONG_ADDR)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX5_DAT_PONG_ADDR -    \
                                                CSL_CSI2_CTX4_DAT_PONG_ADDR)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX6_DAT_PONG_ADDR -    \
                                                CSL_CSI2_CTX5_DAT_PONG_ADDR)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX7_DAT_PONG_ADDR -    \
                                                CSL_CSI2_CTX6_DAT_PONG_ADDR)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX1_IRQENABLE -    \
                                               CSL_CSI2_CTX0_IRQENABLE)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX2_IRQENABLE -    \
                                                CSL_CSI2_CTX1_IRQENABLE)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX3_IRQENABLE -    \
                                                CSL_CSI2_CTX2_IRQENABLE)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX4_IRQENABLE -    \
                                                CSL_CSI2_CTX3_IRQENABLE)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX5_IRQENABLE -    \
                                                CSL_CSI2_CTX4_IRQENABLE)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX6_IRQENABLE -    \
                                                CSL_CSI2_CTX5_IRQENABLE)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX7_IRQENABLE -    \
                                                CSL_CSI2_CTX6_IRQENABLE)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX1_IRQSTATUS -    \
                                                CSL_CSI2_CTX0_IRQSTATUS)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX2_IRQSTATUS -    \
                                                CSL_CSI2_CTX1_IRQSTATUS)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX3_IRQSTATUS -    \
                                                CSL_CSI2_CTX2_IRQSTATUS)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX4_IRQSTATUS -    \
                                                CSL_CSI2_CTX3_IRQSTATUS)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX5_IRQSTATUS -    \
                                                CSL_CSI2_CTX4_IRQSTATUS)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX6_IRQSTATUS -    \
                                                CSL_CSI2_CTX5_IRQSTATUS)) || \
      (CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE != (CSL_CSI2_CTX7_IRQSTATUS -    \
                                                CSL_CSI2_CTX6_IRQSTATUS)) \
    )
#error CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE assumption violated
#endif

/*! \brief define for efficient programming of TRANSCODE(H/V) context registers
       across contexts. These don't follow the same address distance as other
       context registers mentioned in \ref CSL_CSI2_INTER_CONTEXT_ADDR_DISTANCE */
#define CSL_CSI2_INTER_CONTEXT_TRANSCODE_ADDR_DISTANCE   (CSL_CSI2_CTX1_TRANSCODEH  - \
                                                          CSL_CSI2_CTX0_TRANSCODEH)
#if ( (CSL_CSI2_INTER_CONTEXT_TRANSCODE_ADDR_DISTANCE != (CSL_CSI2_CTX2_TRANSCODEH -    \
                                                          CSL_CSI2_CTX1_TRANSCODEH)) || \
      (CSL_CSI2_INTER_CONTEXT_TRANSCODE_ADDR_DISTANCE != (CSL_CSI2_CTX3_TRANSCODEH -    \
                                                          CSL_CSI2_CTX2_TRANSCODEH)) || \
      (CSL_CSI2_INTER_CONTEXT_TRANSCODE_ADDR_DISTANCE != (CSL_CSI2_CTX4_TRANSCODEH -    \
                                                          CSL_CSI2_CTX3_TRANSCODEH)) || \
      (CSL_CSI2_INTER_CONTEXT_TRANSCODE_ADDR_DISTANCE != (CSL_CSI2_CTX5_TRANSCODEH -    \
                                                          CSL_CSI2_CTX4_TRANSCODEH)) || \
      (CSL_CSI2_INTER_CONTEXT_TRANSCODE_ADDR_DISTANCE != (CSL_CSI2_CTX6_TRANSCODEH -    \
                                                          CSL_CSI2_CTX5_TRANSCODEH)) || \
      (CSL_CSI2_INTER_CONTEXT_TRANSCODE_ADDR_DISTANCE != (CSL_CSI2_CTX7_TRANSCODEH -    \
                                                          CSL_CSI2_CTX6_TRANSCODEH)) || \
      (CSL_CSI2_INTER_CONTEXT_TRANSCODE_ADDR_DISTANCE != (CSL_CSI2_CTX1_TRANSCODEV -    \
                                                          CSL_CSI2_CTX0_TRANSCODEV)) || \
      (CSL_CSI2_INTER_CONTEXT_TRANSCODE_ADDR_DISTANCE != (CSL_CSI2_CTX2_TRANSCODEV -    \
                                                          CSL_CSI2_CTX1_TRANSCODEV)) || \
      (CSL_CSI2_INTER_CONTEXT_TRANSCODE_ADDR_DISTANCE != (CSL_CSI2_CTX3_TRANSCODEV -    \
                                                          CSL_CSI2_CTX2_TRANSCODEV)) || \
      (CSL_CSI2_INTER_CONTEXT_TRANSCODE_ADDR_DISTANCE != (CSL_CSI2_CTX4_TRANSCODEV -    \
                                                          CSL_CSI2_CTX3_TRANSCODEV)) || \
      (CSL_CSI2_INTER_CONTEXT_TRANSCODE_ADDR_DISTANCE != (CSL_CSI2_CTX5_TRANSCODEV -    \
                                                          CSL_CSI2_CTX4_TRANSCODEV)) || \
      (CSL_CSI2_INTER_CONTEXT_TRANSCODE_ADDR_DISTANCE != (CSL_CSI2_CTX6_TRANSCODEV -    \
                                                          CSL_CSI2_CTX5_TRANSCODEV)) || \
      (CSL_CSI2_INTER_CONTEXT_TRANSCODE_ADDR_DISTANCE != (CSL_CSI2_CTX7_TRANSCODEV -     \
                                                          CSL_CSI2_CTX6_TRANSCODEV)) \
    )
#error CSL_CSI2_INTER_CONTEXT_TRANSCODE_ADDR_DISTANCE assumption violated
#endif
#ifdef __cplusplus
}
#endif
#endif
