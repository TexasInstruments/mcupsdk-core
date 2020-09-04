/*
 *   Copyright (c) Texas Instruments Incorporated 2022
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

#ifndef SDL_ECC_BUS_SAFETY_HW_H_
#define SDL_ECC_BUS_SAFETY_HW_H_

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************************************
* Register Definitions
****************************************************************************************************/
/* MSS */
#if defined (SOC_AM273X) || defined (SOC_AWR294X)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_CTRL                                       (0x000001A0U)

#elif defined (SOC_AM263X)
#define SDL_MSS_CTRL_MSS_BUS_SAFETY_CTRL                                       (0x00018200U)
#endif

/* DSS */
#define SDL_DSS_CTRL_DSS_BUS_SAFETY_CTRL                                       (0x00000800U)

/* RSS */
#if defined (SOC_AWR294X)
#define SDL_RSS_CTRL_RSS_BUS_SAFETY_CTRL                                       (0x00000090U)
#endif
/* DSS_MCRC */
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_CTRL                                  (0x00000BC8U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_FI                                    (0x00000BCCU)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR                                   (0x00000BD0U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_CMD                          (0x00000BD8U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITE                        (0x00000BDCU)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_READ                         (0x00000BE0U)
#define SDL_DSS_CTRL_DSS_MCRC_BUS_SAFETY_ERR_STAT_WRITERESP                    (0x00000BE4U)

/* DSS_CBUFF_FIFO */
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_CTRL                            (0x00000B08U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_FI                              (0x00000B0CU)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR                             (0x00000B10U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_CMD                    (0x00000B18U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000B1CU)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_READ                   (0x00000B20U)
#define SDL_DSS_CTRL_DSS_CBUFF_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000B24U)

/* DSS_CM4_S */
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_CTRL                                 (0x00000C68U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_FI                                   (0x00000C6CU)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR                                  (0x00000C70U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_CMD                         (0x00000C78U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_WRITE                       (0x00000C7CU)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_READ                        (0x00000C80U)
#define SDL_DSS_CTRL_DSS_CM4_S_BUS_SAFETY_ERR_STAT_WRITERESP                   (0x00000C84U)

/* DSS MDO FIFO */
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_CTRL                              (0x00000AE8U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_FI                                (0x00000AECU)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR                               (0x00000AF0U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_CMD                      (0x00000AF8U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITE                    (0x00000AFCU)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_READ                     (0x00000B00U)
#define SDL_DSS_CTRL_DSS_MDO_FIFO_BUS_SAFETY_ERR_STAT_WRITERESP                (0x00000B04U)

/* DSS_TPTC_A0_RD */
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_CTRL                            (0x000008E0U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_FI                              (0x000008E4U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR                             (0x000008E8U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x000008F0U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ                   (0x000008F4U)

/* DSS_TPTC_A1_RD */
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_CTRL                            (0x000008F8U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_FI                              (0x000008FCU)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR                             (0x00000900U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x00000908U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ                   (0x0000090CU)

/* DSS_TPTC_B0_RD */
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_CTRL                            (0x00000910U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_FI                              (0x00000914U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR                             (0x00000918U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x00000920U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ                   (0x00000924U)

/* DSS_TPTC_B1_RD */
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_CTRL                            (0x00000928U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_FI                              (0x0000092CU)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR                             (0x00000930U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x00000938U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_RD_BUS_SAFETY_ERR_STAT_READ                   (0x0000093CU)

/* DSS_TPTC_C0_RD */
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_CTRL                            (0x00000940U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_FI                              (0x00000944U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR                             (0x00000948U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x00000950U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_RD_BUS_SAFETY_ERR_STAT_READ                   (0x00000954U)

/* DSS_TPTC_C1_RD */
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_CTRL                            (0x00000958U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_FI                              (0x0000095CU)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR                             (0x00000960U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x00000968U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_RD_BUS_SAFETY_ERR_STAT_READ                   (0x0000096CU)

/* DSS_TPTC_C2_RD */
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_CTRL                            (0x00000970U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_FI                              (0x00000974U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR                             (0x00000978U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x00000980U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_RD_BUS_SAFETY_ERR_STAT_READ                   (0x00000984U)

/* DSS_TPTC_C3_RD */
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_CTRL                            (0x00000988U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_FI                              (0x0000098CU)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR                             (0x00000990U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x00000998U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_RD_BUS_SAFETY_ERR_STAT_READ                   (0x0000099CU)

/* DSS_TPTC_C4_RD */
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_CTRL                            (0x000009A0U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_FI                              (0x000009A4U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR                             (0x000009A8U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x000009B0U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_RD_BUS_SAFETY_ERR_STAT_READ                   (0x000009B4U)

/* DSS_TPTC_C5_RD */
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_CTRL                            (0x000009B8U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_FI                              (0x000009BCU)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR                             (0x000009C0U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x000009C8U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_RD_BUS_SAFETY_ERR_STAT_READ                   (0x000009CCU)

/* DSS_TPTC_A0_WR */
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_CTRL                            (0x000009D0U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_FI                              (0x000009D4U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR                             (0x000009D8U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x000009E0U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x000009E4U)
#define SDL_DSS_CTRL_DSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x000009E8U)

/* DSS_TPTC_A1_WR */
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_CTRL                            (0x000009ECU)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_FI                              (0x000009F0U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR                             (0x000009F4U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x000009FCU)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000A00U)
#define SDL_DSS_CTRL_DSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000A04U)

/* DSS_TPTC_B0_WR */
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_CTRL                            (0x00000A08U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_FI                              (0x00000A0CU)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR                             (0x00000A10U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x00000A18U)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000A1CU)
#define SDL_DSS_CTRL_DSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000A20U)

/* DSS_TPTC_B1_WR */
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_CTRL                            (0x00000A24U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_FI                              (0x00000A28U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR                             (0x00000A2CU)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000A30U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x00000A34U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000A38U)
#define SDL_DSS_CTRL_DSS_TPTC_B1_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000A3CU)

/* DSS_TPTC_C0_WR */
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_CTRL                            (0x00000A40U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_FI                              (0x00000A44U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR                             (0x00000A48U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x00000A50U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000A54U)
#define SDL_DSS_CTRL_DSS_TPTC_C0_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000A58U)

/* DSS_TPTC_C1_WR */
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_CTRL                            (0x00000A5CU)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_FI                              (0x00000A60U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR                             (0x00000A64U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x00000A6CU)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000A70U)
#define SDL_DSS_CTRL_DSS_TPTC_C1_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000A74U)

/* DSS_TPTC_C2_WR */
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_CTRL                            (0x00000A78U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_FI                              (0x00000A7CU)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR                             (0x00000A80U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x00000A88U)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000A8CU)
#define SDL_DSS_CTRL_DSS_TPTC_C2_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000A90U)

/* DSS_TPTC_C3_WR */
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_CTRL                            (0x00000A94U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_FI                              (0x00000A98U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR                             (0x00000A9CU)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x00000AA4U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000AA8U)
#define SDL_DSS_CTRL_DSS_TPTC_C3_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000AACU)

/* DSS_TPTC_C5_WR */
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_CTRL                            (0x00000AB0U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_FI                              (0x00000AB4U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR                             (0x00000AB8U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x00000AC0U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000AC4U)
#define SDL_DSS_CTRL_DSS_TPTC_C4_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000AC8U)

/* DSS_TPTC_C5_WR */
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_CTRL                            (0x00000ACCU)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_FI                              (0x00000AD0U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR                             (0x00000AD4U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x00000ADCU)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000AE0U)
#define SDL_DSS_CTRL_DSS_TPTC_C5_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000AE4U)

/* DSS_L3_BANKA */

#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_CTRL                              (0x00000830U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_FI                                (0x00000834U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR                               (0x00000838U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_CMD                      (0x00000844U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITE                    (0x00000848U)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_READ                     (0x0000084CU)
#define SDL_DSS_CTRL_DSS_L3_BANKA_BUS_SAFETY_ERR_STAT_WRITERESP                (0x00000850U)

/* DSS_L3_BANKB */
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_CTRL                              (0x00000854U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_FI                                (0x00000858U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR                               (0x0000085CU)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_CMD                      (0x00000868U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITE                    (0x0000086CU)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_READ                     (0x00000870U)
#define SDL_DSS_CTRL_DSS_L3_BANKB_BUS_SAFETY_ERR_STAT_WRITERESP                (0x00000874U)

/* DSS_L3_BANKC */
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_CTRL                              (0x00000878U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_FI                                (0x0000087CU)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR                               (0x00000880U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_CMD                      (0x0000088CU)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITE                    (0x00000890U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_READ                     (0x00000894U)
#define SDL_DSS_CTRL_DSS_L3_BANKC_BUS_SAFETY_ERR_STAT_WRITERESP                (0x00000898U)

/* DSS_L3_BANKD */
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_CTRL                              (0x0000089CU)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_FI                                (0x000008A0U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR                               (0x000008A4U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_CMD                      (0x000008B0U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITE                    (0x000008B4U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_READ                     (0x000008B8U)
#define SDL_DSS_CTRL_DSS_L3_BANKD_BUS_SAFETY_ERR_STAT_WRITERESP                (0x000008BCU)

/* DSS_MBOX */
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_CTRL                                  (0x00000C88U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_FI                                    (0x00000C8CU)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR                                   (0x00000C90U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_CMD                          (0x00000C98U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE                        (0x00000C9CU)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_READ                         (0x00000CA0U)
#define SDL_DSS_CTRL_DSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP                    (0x00000CA4U)

/* RSS_MBOX */
#if defined (SOC_AWR294X)
#define SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_CTRL                                  (0x000001F4U)
#define SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_FI                                    (0x000001F8U)
#define SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR                                   (0x000001FCU)
#define SDL_RSS_CTRL_RSS_BUS_SAFETY_SEC_ERR_STAT0                              (0x00000094U)
#define SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_CMD                          (0x00000204U)
#define SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE                        (0x00000208U)
#define SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_READ                         (0x0000020CU)
#define SDL_RSS_CTRL_RSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP                    (0x00000210U)
#endif

/* DSS_HWA_DMA0 */
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_CTRL                              (0x00000C08U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_FI                                (0x00000C0CU)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR                               (0x00000C10U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_DATA0                    (0x00000C14U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_CMD                      (0x00000C18U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITE                    (0x00000C1CU)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_READ                     (0x00000C20U)
#define SDL_DSS_CTRL_DSS_HWA_DMA0_BUS_SAFETY_ERR_STAT_WRITERESP                (0x00000C24U)

/*DSS_HWA_DMA1*/
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_CTRL                              (0x00000C28U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_FI                                (0x00000C2CU)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR                               (0x00000C30U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_DATA0                    (0x00000C34U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_CMD                      (0x00000C38U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITE                    (0x00000C3CU)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_READ                     (0x00000C40U)
#define SDL_DSS_CTRL_DSS_HWA_DMA1_BUS_SAFETY_ERR_STAT_WRITERESP                (0x00000C44U)

/*DSS_DSP_MDMA*/
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_CTRL                              (0x0000080CU)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_FI                                (0x00000810U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR                               (0x00000814U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA0                    (0x00000818U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_DATA1                    (0x0000081CU)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_CMD                      (0x00000820U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITE                    (0x00000824U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_READ                     (0x00000828U)
#define SDL_DSS_CTRL_DSS_DSP_MDMA_BUS_SAFETY_ERR_STAT_WRITERESP                (0x0000082CU)

/*DSS_DSP_SDMA*/
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_CTRL                              (0x000008C0U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_FI                                (0x000008C4U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR                               (0x000008C8U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_DATA0                    (0x000008CCU)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_CMD                      (0x000008D0U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITE                    (0x000008D4U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_READ                     (0x000008D8U)
#define SDL_DSS_CTRL_DSS_DSP_SDMA_BUS_SAFETY_ERR_STAT_WRITERESP                (0x000008DCU)

/*DSS_PCR*/
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_CTRL                                   (0x00000BE8U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_FI                                     (0x00000BECU)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR                                    (0x00000BF0U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_CMD                           (0x00000BF8U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_WRITE                         (0x00000BFCU)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_READ                          (0x00000C00U)
#define SDL_DSS_CTRL_DSS_PCR_BUS_SAFETY_ERR_STAT_WRITERESP                     (0x00000C04U)

/*RSS_ADCBUF_RD*/
#define SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_CTRL                             (0x0000010CU)
#define SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_FI                               (0x00000110U)
#define SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR                              (0x00000114U)
#define SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_CMD                     (0x0000011CU)
#define SDL_RSS_CTRL_RSS_ADCBUF_RD_BUS_SAFETY_ERR_STAT_READ                    (0x00000120U)

/*RSS_ADCBUF_WR*/
#define SDL_RSS_CTRL_DMMSWINT1                                                 (0x000002D0U)
#define SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_CTRL                             (0x00000124U)
#define SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_FI                               (0x00000128U)
#define SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR                              (0x0000012CU)
#define SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_CMD                     (0x00000134U)
#define SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITE                   (0x00000138U)
#define SDL_RSS_CTRL_RSS_ADCBUF_WR_BUS_SAFETY_ERR_STAT_WRITERESP               (0x0000013CU)

#if defined (SOC_AM263X)
/* MSS_TPTC_A0_RD */
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL                            (0x000183A0U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI                              (0x000183A4U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR                             (0x000183A8U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x000183B0U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ                   (0x000183B4U)

/* MSS_TPTC_A1_RD */
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL                            (0x000183C0U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI                              (0x000183C4U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR                             (0x000183C8U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x000183D0U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ                   (0x000183D4U)

/* MSS_TPTC_A0_WR */
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL                            (0x000183E0U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI                              (0x000183E4U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR                             (0x000183E8U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0                  (0x000183ECU)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x000183F0U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x000183F4U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x000183F8U)

/* MSS_TPTC_A1_WR */
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL                            (0x00018400U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI                              (0x00018404U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR                             (0x00018408U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0                  (0x0001840CU)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x00018410U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x00018414U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00018418U)

/* MSS_CR5A_AHB */
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL                              (0x00018740U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI                                (0x00018744U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR                               (0x00018748U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_DATA0                    (0x0001874CU)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_CMD                      (0x00018750U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITE                    (0x00018754U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_READ                     (0x00018758U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITERESP                (0x0001875CU)

/* MSS_CR5B_AHB */
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL                              (0x00018760U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI                                (0x00018764U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR                               (0x00018768U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_DATA0                    (0x0001876CU)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_CMD                      (0x00018770U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITE                    (0x00018774U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_READ                     (0x00018778U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITERESP                (0x0001877CU)

/* MSS_CR5C_AHB */
#define SDL_MSS_CTRL_MSS_CR5C_AHB_BUS_SAFETY_CTRL                              (0x00018780U)
#define SDL_MSS_CTRL_MSS_CR5C_AHB_BUS_SAFETY_FI                                (0x00018784U)
#define SDL_MSS_CTRL_MSS_CR5C_AHB_BUS_SAFETY_ERR                               (0x00018788U)
#define SDL_MSS_CTRL_MSS_CR5C_AHB_BUS_SAFETY_ERR_STAT_DATA0                    (0x0001878CU)
#define SDL_MSS_CTRL_MSS_CR5C_AHB_BUS_SAFETY_ERR_STAT_CMD                      (0x00018790U)
#define SDL_MSS_CTRL_MSS_CR5C_AHB_BUS_SAFETY_ERR_STAT_WRITE                    (0x00018794U)
#define SDL_MSS_CTRL_MSS_CR5C_AHB_BUS_SAFETY_ERR_STAT_READ                     (0x00018798U)
#define SDL_MSS_CTRL_MSS_CR5C_AHB_BUS_SAFETY_ERR_STAT_WRITERESP                (0x0001879CU)

/* MSS_CR5D_AHB */
#define SDL_MSS_CTRL_MSS_CR5D_AHB_BUS_SAFETY_CTRL                              (0x000187A0U)
#define SDL_MSS_CTRL_MSS_CR5D_AHB_BUS_SAFETY_FI                                (0x000187A4U)
#define SDL_MSS_CTRL_MSS_CR5D_AHB_BUS_SAFETY_ERR                               (0x000187A8U)
#define SDL_MSS_CTRL_MSS_CR5D_AHB_BUS_SAFETY_ERR_STAT_DATA0                    (0x000187ACU)
#define SDL_MSS_CTRL_MSS_CR5D_AHB_BUS_SAFETY_ERR_STAT_CMD                      (0x000187B0U)
#define SDL_MSS_CTRL_MSS_CR5D_AHB_BUS_SAFETY_ERR_STAT_WRITE                    (0x000187B4U)
#define SDL_MSS_CTRL_MSS_CR5D_AHB_BUS_SAFETY_ERR_STAT_READ                     (0x000187B8U)
#define SDL_MSS_CTRL_MSS_CR5D_AHB_BUS_SAFETY_ERR_STAT_WRITERESP                (0x000187BCU)

/* MSS_CR5A_AXI_RD */
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL                           (0x00018220U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI                             (0x00018224U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR                            (0x00018228U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0                 (0x0001822CU)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_CMD                   (0x00018230U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_READ                  (0x00018234U)

/* MSS_CR5B_AXI_RD */
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL                           (0x00018240U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI                             (0x00018244U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR                            (0x00018248U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0                 (0x0001824CU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_CMD                   (0x00018250U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_READ                  (0x00018254U)

/* MSS_CR5C_AXI_RD */
#define SDL_MSS_CTRL_MSS_CR5C_AXI_RD_BUS_SAFETY_CTRL                           (0x00018260U)
#define SDL_MSS_CTRL_MSS_CR5C_AXI_RD_BUS_SAFETY_FI                             (0x00018264U)
#define SDL_MSS_CTRL_MSS_CR5C_AXI_RD_BUS_SAFETY_ERR                            (0x00018268U)
#define SDL_MSS_CTRL_MSS_CR5C_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0                 (0x0001826CU)
#define SDL_MSS_CTRL_MSS_CR5C_AXI_RD_BUS_SAFETY_ERR_STAT_CMD                   (0x00018270U)
#define SDL_MSS_CTRL_MSS_CR5C_AXI_RD_BUS_SAFETY_ERR_STAT_READ                  (0x00018274U)

/* MSS_CR5D_AXI_RD */
#define SDL_MSS_CTRL_MSS_CR5D_AXI_RD_BUS_SAFETY_CTRL                           (0x00018280U)
#define SDL_MSS_CTRL_MSS_CR5D_AXI_RD_BUS_SAFETY_FI                             (0x00018284U)
#define SDL_MSS_CTRL_MSS_CR5D_AXI_RD_BUS_SAFETY_ERR                            (0x00018288U)
#define SDL_MSS_CTRL_MSS_CR5D_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0                 (0x0001828CU)
#define SDL_MSS_CTRL_MSS_CR5D_AXI_RD_BUS_SAFETY_ERR_STAT_CMD                   (0x00018290U)
#define SDL_MSS_CTRL_MSS_CR5D_AXI_RD_BUS_SAFETY_ERR_STAT_READ                  (0x00018294U)

/* MSS_CR5A_AXI_WR */
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL                           (0x000182A0U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI                             (0x000182A4U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR                            (0x000182A8U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0                 (0x000182ACU)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_CMD                   (0x000182B0U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE                 (0x000182B4U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP             (0x000182B8U)

/* MSS_CR5B_AXI_WR */
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL                           (0x000182C0U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI                             (0x000182C4U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR                            (0x000182C8U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0                 (0x000182CCU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_CMD                   (0x000182D0U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE                 (0x000182D4U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP             (0x000182D8U)

/* MSS_CR5C_AXI_WR */
#define SDL_MSS_CTRL_MSS_CR5C_AXI_WR_BUS_SAFETY_CTRL                           (0x000182E0U)
#define SDL_MSS_CTRL_MSS_CR5C_AXI_WR_BUS_SAFETY_FI                             (0x000182E4U)
#define SDL_MSS_CTRL_MSS_CR5C_AXI_WR_BUS_SAFETY_ERR                            (0x000182E8U)
#define SDL_MSS_CTRL_MSS_CR5C_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0                 (0x000182ECU)
#define SDL_MSS_CTRL_MSS_CR5C_AXI_WR_BUS_SAFETY_ERR_STAT_CMD                   (0x000182F0U)
#define SDL_MSS_CTRL_MSS_CR5C_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE                 (0x000182F4U)
#define SDL_MSS_CTRL_MSS_CR5C_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP             (0x000182F8U)

/* MSS_CR5D_AXI_WR */
#define SDL_MSS_CTRL_MSS_CR5D_AXI_WR_BUS_SAFETY_CTRL                           (0x00018300U)
#define SDL_MSS_CTRL_MSS_CR5D_AXI_WR_BUS_SAFETY_FI                             (0x00018304U)
#define SDL_MSS_CTRL_MSS_CR5D_AXI_WR_BUS_SAFETY_ERR                            (0x00018308U)
#define SDL_MSS_CTRL_MSS_CR5D_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0                 (0x0001830CU)
#define SDL_MSS_CTRL_MSS_CR5D_AXI_WR_BUS_SAFETY_ERR_STAT_CMD                   (0x00018310U)
#define SDL_MSS_CTRL_MSS_CR5D_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE                 (0x00018314U)
#define SDL_MSS_CTRL_MSS_CR5D_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP             (0x00018318U)

/* MSS_CR5A_AXI_S */
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL                            (0x00018320U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI                              (0x00018324U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR                             (0x00018328U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_DATA0                  (0x0001832CU)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_CMD                    (0x00018330U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITE                  (0x00018334U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_READ                   (0x00018338U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP              (0x0001833CU)

/* MSS_CR5B_AXI_S */
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL                            (0x00018340U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI                              (0x00018344U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR                             (0x00018348U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_DATA0                  (0x0001834CU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_CMD                    (0x00018350U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITE                  (0x00018354U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_READ                   (0x00018358U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP              (0x0001835CU)

/* MSS_CR5C_AXI_S */
#define SDL_MSS_CTRL_MSS_CR5C_AXI_S_BUS_SAFETY_CTRL                            (0x00018360U)
#define SDL_MSS_CTRL_MSS_CR5C_AXI_S_BUS_SAFETY_FI                              (0x00018364U)
#define SDL_MSS_CTRL_MSS_CR5C_AXI_S_BUS_SAFETY_ERR                             (0x00018368U)
#define SDL_MSS_CTRL_MSS_CR5C_AXI_S_BUS_SAFETY_ERR_STAT_DATA0                  (0x0001836CU)
#define SDL_MSS_CTRL_MSS_CR5C_AXI_S_BUS_SAFETY_ERR_STAT_CMD                    (0x00018370U)
#define SDL_MSS_CTRL_MSS_CR5C_AXI_S_BUS_SAFETY_ERR_STAT_WRITE                  (0x00018374U)
#define SDL_MSS_CTRL_MSS_CR5C_AXI_S_BUS_SAFETY_ERR_STAT_READ                   (0x00018378U)
#define SDL_MSS_CTRL_MSS_CR5C_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP              (0x0001837CU)

/* MSS_CR5D_AXI_S */
#define SDL_MSS_CTRL_MSS_CR5D_AXI_S_BUS_SAFETY_CTRL                            (0x00018380U)
#define SDL_MSS_CTRL_MSS_CR5D_AXI_S_BUS_SAFETY_FI                              (0x00018384U)
#define SDL_MSS_CTRL_MSS_CR5D_AXI_S_BUS_SAFETY_ERR                             (0x00018388U)
#define SDL_MSS_CTRL_MSS_CR5D_AXI_S_BUS_SAFETY_ERR_STAT_DATA0                  (0x0001838CU)
#define SDL_MSS_CTRL_MSS_CR5D_AXI_S_BUS_SAFETY_ERR_STAT_CMD                    (0x00018390U)
#define SDL_MSS_CTRL_MSS_CR5D_AXI_S_BUS_SAFETY_ERR_STAT_WRITE                  (0x00018394U)
#define SDL_MSS_CTRL_MSS_CR5D_AXI_S_BUS_SAFETY_ERR_STAT_READ                   (0x00018398U)
#define SDL_MSS_CTRL_MSS_CR5D_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP              (0x0001839CU)
#endif

#if defined (SOC_AM273X) || defined (SOC_AWR294X)
/* MSS_TPTC_A0_RD */
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_CTRL                            (0x0000024CU)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_FI                              (0x00000250U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR                             (0x00000254U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x0000025CU)
#define SDL_MSS_CTRL_MSS_TPTC_A0_RD_BUS_SAFETY_ERR_STAT_READ                   (0x00000260U)

/* MSS_TPTC_A1_RD */
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_CTRL                            (0x00000264U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_FI                              (0x00000268U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR                             (0x0000026CU)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x00000274U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_RD_BUS_SAFETY_ERR_STAT_READ                   (0x00000278U)

/* MSS_TPTC_B0_RD */
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_CTRL                            (0x0000027CU)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_FI                              (0x00000280U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR                             (0x00000284U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_CMD                    (0x0000028CU)
#define SDL_MSS_CTRL_MSS_TPTC_B0_RD_BUS_SAFETY_ERR_STAT_READ                   (0x00000290U)

/* MSS_TPTC_A0_WR */
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_CTRL                            (0x00000294U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_FI                              (0x00000298U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR                             (0x0000029CU)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_DATA0                  (0x000002A0U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x000002A4U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x000002A8U)
#define SDL_MSS_CTRL_MSS_TPTC_A0_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x000002ACU)

/* MSS_TPTC_A1_WR */
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_CTRL                            (0x000002B0U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_FI                              (0x000002B4U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR                             (0x000002B8U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_DATA0                  (0x000002BCU)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x000002C0U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x000002C4U)
#define SDL_MSS_CTRL_MSS_TPTC_A1_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x000002C8U)

/* MSS_TPTC_B0_WR */
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_CTRL                            (0x000002CCU)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_FI                              (0x000002D0U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR                             (0x000002D4U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_DATA0                  (0x000002D8U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_CMD                    (0x000002DCU)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITE                  (0x000002E0U)
#define SDL_MSS_CTRL_MSS_TPTC_B0_WR_BUS_SAFETY_ERR_STAT_WRITERESP              (0x000002E4U)

/* MSS_CR5A_AHB */
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_CTRL                              (0x000005B8U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_FI                                (0x000005BCU)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR                               (0x000005C0U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_DATA0                    (0x000005C4U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_CMD                      (0x000005C8U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITE                    (0x000005CCU)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_READ                     (0x000005D0U)
#define SDL_MSS_CTRL_MSS_CR5A_AHB_BUS_SAFETY_ERR_STAT_WRITERESP                (0x000005D4U)

/* MSS_CR5B_AHB */
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_CTRL                              (0x000005D8U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_FI                                (0x000005DCU)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR                               (0x000005E0U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_DATA0                    (0x000005E4U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_CMD                      (0x000005E8U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITE                    (0x000005ECU)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_READ                     (0x000005F0U)
#define SDL_MSS_CTRL_MSS_CR5B_AHB_BUS_SAFETY_ERR_STAT_WRITERESP                (0x000005F4U)

/* MSS_CR5A_AXI_RD */
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_CTRL                           (0x000001A4U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_FI                             (0x000001A8U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR                            (0x000001ACU)
// #define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0                 (0x000001B0U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_CMD                   (0x000001B4U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_RD_BUS_SAFETY_ERR_STAT_READ                  (0x000001B8U)

/* MSS_CR5B_AXI_RD */
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_CTRL                           (0x000001BCU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_FI                             (0x000001C0U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR                            (0x000001C4U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_DATA0                 (0x000001C8U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_CMD                   (0x000001CCU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_RD_BUS_SAFETY_ERR_STAT_READ                  (0x000001D0U)

/* MSS_CR5A_AXI_WR */
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_CTRL                           (0x000001D4U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_FI                             (0x000001D8U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR                            (0x000001DCU)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0                 (0x000001E0U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_CMD                   (0x000001E4U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE                 (0x000001E8U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP             (0x000001ECU)

/* MSS_CR5B_AXI_WR */
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_CTRL                           (0x000001F0U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_FI                             (0x000001F4U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR                            (0x000001F8U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_DATA0                 (0x000001FCU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_CMD                   (0x00000200U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITE                 (0x00000204U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_WR_BUS_SAFETY_ERR_STAT_WRITERESP             (0x00000208U)

/* MSS_CR5A_AXI_S */
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_CTRL                            (0x0000020CU)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_FI                              (0x00000210U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR                             (0x00000214U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000218U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_CMD                    (0x0000021CU)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000220U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_READ                   (0x00000224U)
#define SDL_MSS_CTRL_MSS_CR5A_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000228U)

/* MSS_CR5B_AXI_S */
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_CTRL                            (0x0000022CU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_FI                              (0x00000230U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR                             (0x00000234U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_DATA0                  (0x00000238U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_CMD                    (0x0000023CU)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITE                  (0x00000240U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_READ                   (0x00000244U)
#define SDL_MSS_CTRL_MSS_CR5B_AXI_S_BUS_SAFETY_ERR_STAT_WRITERESP              (0x00000248U)

/* MSS_MBOX */
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_CTRL                                  (0x000004B0U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_FI                                    (0x000004B4U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR                                   (0x000004B8U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_DATA0                        (0x000004BCU)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_CMD                          (0x000004C0U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITE                        (0x000004C4U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_READ                         (0x000004C8U)
#define SDL_MSS_CTRL_MSS_MBOX_BUS_SAFETY_ERR_STAT_WRITERESP                    (0x000004CCU)

/* MSS_MBOX AM263X*/
#define SDL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_CTRL                                 (0x000186A0U)
#define SDL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_FI                                   (0x000186A4U)
#define SDL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR                                  (0x000186A8U)
#define SDL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_DATA0                       (0x000186ACU)
#define SDL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_CMD                         (0x000186B0U)
#define SDL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_WRITE                       (0x000186B4U)
#define SDL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_READ                        (0x000186B8U)
#define SDL_MSS_CTRL_MBOX_SRAM_BUS_SAFETY_ERR_STAT_WRITERESP                   (0x000186BCU)

#endif


/****************************************************************************************************
* Field Definition Macros
****************************************************************************************************/

/* BUS_SAFETY_CTRL */

#define SDL_CTRL_BUS_SAFETY_CTRL_ENABLE_MASK       (0x00000007U)
#define SDL_CTRL_BUS_SAFETY_CTRL_ENABLE_SHIFT      (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_CTRL_ENABLE_RESETVAL   (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_CTRL_ENABLE_MAX        (0x00000007U)

/* BUS_SAFETY_CTRL */

#define SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ENABLE_MASK (0x00000007U)
#define SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ENABLE_SHIFT (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ENABLE_RESETVAL (0x00000007U)
#define SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ENABLE_MAX (0x00000007U)

#define SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ERR_CLEAR_MASK (0x00000100U)
#define SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ERR_CLEAR_SHIFT (0x00000008U)
#define SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ERR_CLEAR_RESETVAL (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_CTRL_BUS_SAFETY_CTRL_ERR_CLEAR_MAX (0x00000001U)

/* BUS_SAFETY_FI */

#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_GLOBAL_MAIN_MASK (0x00000001U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_GLOBAL_MAIN_SHIFT (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_GLOBAL_MAIN_RESETVAL (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_GLOBAL_MAIN_MAX (0x00000001U)

#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_GLOBAL_SAFE_MASK (0x00000002U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_GLOBAL_SAFE_SHIFT (0x00000001U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_GLOBAL_SAFE_RESETVAL (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_GLOBAL_SAFE_MAX (0x00000001U)

#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_SEC_MASK    (0x00000010U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_SEC_SHIFT   (0x00000004U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_SEC_RESETVAL (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_SEC_MAX     (0x00000001U)

#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DED_MASK    (0x00000020U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DED_SHIFT   (0x00000005U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DED_RESETVAL (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DED_MAX     (0x00000001U)

#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DATA_MASK   (0x0000FF00U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DATA_SHIFT  (0x00000008U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DATA_RESETVAL (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_DATA_MAX    (0x000000FFU)

#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_MAIN_MASK   (0x00FF0000U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_MAIN_SHIFT  (0x00000010U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_MAIN_RESETVAL (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_MAIN_MAX    (0x000000FFU)

#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_SAFE_MASK   (0xFF000000U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_SAFE_SHIFT  (0x00000018U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_SAFE_RESETVAL (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_FI_BUS_SAFETY_FI_SAFE_MAX    (0x000000FFU)

/* BUS_SAFETY_ERR */

#define SDL_CTRL_BUS_SAFETY_ERR_BUS_SAFETY_ERR_SEC_MASK  (0x00FF0000U)
#define SDL_CTRL_BUS_SAFETY_ERR_BUS_SAFETY_ERR_SEC_SHIFT (0x00000010U)
#define SDL_CTRL_BUS_SAFETY_ERR_BUS_SAFETY_ERR_SEC_RESETVAL (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_ERR_BUS_SAFETY_ERR_SEC_MAX   (0x000000FFU)

#define SDL_CTRL_BUS_SAFETY_ERR_BUS_SAFETY_ERR_DED_MASK  (0xFF000000U)
#define SDL_CTRL_BUS_SAFETY_ERR_BUS_SAFETY_ERR_DED_SHIFT (0x00000018U)
#define SDL_CTRL_BUS_SAFETY_ERR_BUS_SAFETY_ERR_DED_RESETVAL (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_ERR_BUS_SAFETY_ERR_DED_MAX   (0x000000FFU)

/* BUS_SAFETY_ERR_STAT_CMD */

#define SDL_CTRL_BUS_SAFETY_ERR_STAT_CMD_BUS_SAFETY_ERR_STAT_CMD_STAT_MASK (0xFFFFFFFFU)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_CMD_BUS_SAFETY_ERR_STAT_CMD_STAT_SHIFT (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_CMD_BUS_SAFETY_ERR_STAT_CMD_STAT_RESETVAL (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_CMD_BUS_SAFETY_ERR_STAT_CMD_STAT_MAX (0xFFFFFFFFU)

/* BUS_SAFETY_ERR_STAT_WRITE */

#define SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITE_BUS_SAFETY_ERR_STAT_WRITE_STAT_MASK (0xFFFFFFFFU)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITE_BUS_SAFETY_ERR_STAT_WRITE_STAT_SHIFT (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITE_BUS_SAFETY_ERR_STAT_WRITE_STAT_RESETVAL (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITE_BUS_SAFETY_ERR_STAT_WRITE_STAT_MAX (0xFFFFFFFFU)

/* BUS_SAFETY_ERR_STAT_READ */

#define SDL_CTRL_BUS_SAFETY_ERR_STAT_READ_BUS_SAFETY_ERR_STAT_READ_STAT_MASK (0xFFFFFFFFU)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_READ_BUS_SAFETY_ERR_STAT_READ_STAT_SHIFT (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_READ_BUS_SAFETY_ERR_STAT_READ_STAT_RESETVAL (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_READ_BUS_SAFETY_ERR_STAT_READ_STAT_MAX (0xFFFFFFFFU)

/* BUS_SAFETY_ERR_STAT_WRITERESP */

#define SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITERESP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MASK (0xFFFFFFFFU)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITERESP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_SHIFT (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITERESP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_RESETVAL (0x00000000U)
#define SDL_CTRL_BUS_SAFETY_ERR_STAT_WRITERESP_BUS_SAFETY_ERR_STAT_WRITERESP_STAT_MAX (0xFFFFFFFFU)


/* DMMSWINT1 */
#define SDL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMADCBUFWREN_MASK                    (0x00020000U)
#define SDL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMADCBUFWREN_SHIFT                   (0x00000011U)
#define SDL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMADCBUFWREN_RESETVAL                (0x00000000U)
#define SDL_RSS_CTRL_DMMSWINT1_DMMSWINT1_DMMADCBUFWREN_MAX                     (0x00000001U)

#ifdef __cplusplus
}
#endif
#endif
