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

#ifndef CSLR_GPIO_H_
#define CSLR_GPIO_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************
* Register Macros
**************************************************************************/
#define CSL_GPIO_GIODIR(n)                            (0x34U + ((n) * 0x20U))
#define CSL_GPIO_GIODIN(n)                            (0x38U + ((n) * 0x20U))
#define CSL_GPIO_GIODOUT(n)                           (0x3CU + ((n) * 0x20U))
#define CSL_GPIO_GIOSET(n)                            (0x40U + ((n) * 0x20U))
#define CSL_GPIO_GIOCLR(n)                            (0x44U + ((n) * 0x20U))
#define CSL_GPIO_GIOPDR(n)                            (0x48U + ((n) * 0x20U))
#define CSL_GPIO_GIOPULDIS(n)                         (0x4CU + ((n) * 0x20U))
#define CSL_GPIO_GIOPSL(n)                            (0x50U + ((n) * 0x20U))
#define CSL_GPIO_GIOGCR                               (0x00000000U)
#define CSL_GPIO_GIOPWDN                              (0x00000004U)
#define CSL_GPIO_GIOINTDET                            (0x00000008U)
#define CSL_GPIO_GIOPOL                               (0x0000000CU)
#define CSL_GPIO_GIOENASET                            (0x00000010U)
#define CSL_GPIO_GIOENACLR                            (0x00000014U)
#define CSL_GPIO_GIOLVLSET                            (0x00000018U)
#define CSL_GPIO_GIOLVLCLR                            (0x0000001CU)
#define CSL_GPIO_GIOFLG                               (0x00000020U)
#define CSL_GPIO_GIOOFFA                              (0x00000024U)
#define CSL_GPIO_GIOOFFB                              (0x00000028U)
#define CSL_GPIO_GIOEMUA                              (0x0000002CU)
#define CSL_GPIO_GIOEMUB                              (0x00000030U)
#define CSL_GPIO_GIOSRC(n)                            (0x134U + ((n)) * 0x4U))


/**************************************************************************
* Field Definition Macros
**************************************************************************/

/* GIOGCR */

#define CSL_GPIO_GIOGCR_RESET_MASK                                          (0x00000001U)
#define CSL_GPIO_GIOGCR_RESET_SHIFT                                         (0x00000000U)
#define CSL_GPIO_GIOGCR_RESET_RESETVAL                                      (0x00000000U)
#define CSL_GPIO_GIOGCR_RESET_MAX                                           (0x00000001U)

#define CSL_GPIO_GIOGCR_NU0_MASK                                            (0xFFFFFFFEU)
#define CSL_GPIO_GIOGCR_NU0_SHIFT                                           (0x00000001U)
#define CSL_GPIO_GIOGCR_NU0_RESETVAL                                        (0x00000000U)
#define CSL_GPIO_GIOGCR_NU0_MAX                                             (0x7FFFFFFFU)

#define CSL_GPIO_GIOGCR_RESETVAL                                            (0x00000000U)

/* GIOPWDN */

#define CSL_GPIO_GIOPWDN_GIOPWDN_MASK                                       (0x00000001U)
#define CSL_GPIO_GIOPWDN_GIOPWDN_SHIFT                                      (0x00000000U)
#define CSL_GPIO_GIOPWDN_GIOPWDN_RESETVAL                                   (0x00000000U)
#define CSL_GPIO_GIOPWDN_GIOPWDN_MAX                                        (0x00000001U)

#define CSL_GPIO_GIOPWDN_RESETVAL                                           (0x00000000U)

/* GIOINTDET */

#define CSL_GPIO_GIOINTDET_GIOINTDET_0_MASK                                 (0x000000FFU)
#define CSL_GPIO_GIOINTDET_GIOINTDET_0_SHIFT                                (0x00000000U)
#define CSL_GPIO_GIOINTDET_GIOINTDET_0_RESETVAL                             (0x00000000U)
#define CSL_GPIO_GIOINTDET_GIOINTDET_0_MAX                                  (0x000000FFU)

#define CSL_GPIO_GIOINTDET_GIOINTDET_1_MASK                                 (0x0000FF00U)
#define CSL_GPIO_GIOINTDET_GIOINTDET_1_SHIFT                                (0x00000008U)
#define CSL_GPIO_GIOINTDET_GIOINTDET_1_RESETVAL                             (0x00000000U)
#define CSL_GPIO_GIOINTDET_GIOINTDET_1_MAX                                  (0x000000FFU)

#define CSL_GPIO_GIOINTDET_GIOINTDET_2_MASK                                 (0x00FF0000U)
#define CSL_GPIO_GIOINTDET_GIOINTDET_2_SHIFT                                (0x00000010U)
#define CSL_GPIO_GIOINTDET_GIOINTDET_2_RESETVAL                             (0x00000000U)
#define CSL_GPIO_GIOINTDET_GIOINTDET_2_MAX                                  (0x000000FFU)

#define CSL_GPIO_GIOINTDET_GIOINTDET_3_MASK                                 (0xFF000000U)
#define CSL_GPIO_GIOINTDET_GIOINTDET_3_SHIFT                                (0x00000018U)
#define CSL_GPIO_GIOINTDET_GIOINTDET_3_RESETVAL                             (0x00000000U)
#define CSL_GPIO_GIOINTDET_GIOINTDET_3_MAX                                  (0x000000FFU)

#define CSL_GPIO_GIOINTDET_RESETVAL                                         (0x00000000U)

/* GIOPOL */

#define CSL_GPIO_GIOPOL_GIOPOL_0_MASK                                       (0x000000FFU)
#define CSL_GPIO_GIOPOL_GIOPOL_0_SHIFT                                      (0x00000000U)
#define CSL_GPIO_GIOPOL_GIOPOL_0_RESETVAL                                   (0x00000000U)
#define CSL_GPIO_GIOPOL_GIOPOL_0_MAX                                        (0x000000FFU)

#define CSL_GPIO_GIOPOL_GIOPOL_1_MASK                                       (0x0000FF00U)
#define CSL_GPIO_GIOPOL_GIOPOL_1_SHIFT                                      (0x00000008U)
#define CSL_GPIO_GIOPOL_GIOPOL_1_RESETVAL                                   (0x00000000U)
#define CSL_GPIO_GIOPOL_GIOPOL_1_MAX                                        (0x000000FFU)

#define CSL_GPIO_GIOPOL_GIOPOL_2_MASK                                       (0x00FF0000U)
#define CSL_GPIO_GIOPOL_GIOPOL_2_SHIFT                                      (0x00000010U)
#define CSL_GPIO_GIOPOL_GIOPOL_2_RESETVAL                                   (0x00000000U)
#define CSL_GPIO_GIOPOL_GIOPOL_2_MAX                                        (0x000000FFU)

#define CSL_GPIO_GIOPOL_GIOPOL_3_MASK                                       (0xFF000000U)
#define CSL_GPIO_GIOPOL_GIOPOL_3_SHIFT                                      (0x00000018U)
#define CSL_GPIO_GIOPOL_GIOPOL_3_RESETVAL                                   (0x00000000U)
#define CSL_GPIO_GIOPOL_GIOPOL_3_MAX                                        (0x000000FFU)

#define CSL_GPIO_GIOPOL_RESETVAL                                            (0x00000000U)

/* GIOENASET */

#define CSL_GPIO_GIOENASET_GIOENASET_0_MASK                                 (0x000000FFU)
#define CSL_GPIO_GIOENASET_GIOENASET_0_SHIFT                                (0x00000000U)
#define CSL_GPIO_GIOENASET_GIOENASET_0_RESETVAL                             (0x00000000U)
#define CSL_GPIO_GIOENASET_GIOENASET_0_MAX                                  (0x000000FFU)

#define CSL_GPIO_GIOENASET_GIOENASET_1_MASK                                 (0x0000FF00U)
#define CSL_GPIO_GIOENASET_GIOENASET_1_SHIFT                                (0x00000008U)
#define CSL_GPIO_GIOENASET_GIOENASET_1_RESETVAL                             (0x00000000U)
#define CSL_GPIO_GIOENASET_GIOENASET_1_MAX                                  (0x000000FFU)

#define CSL_GPIO_GIOENASET_GIOENASET_2_MASK                                 (0x00FF0000U)
#define CSL_GPIO_GIOENASET_GIOENASET_2_SHIFT                                (0x00000010U)
#define CSL_GPIO_GIOENASET_GIOENASET_2_RESETVAL                             (0x00000000U)
#define CSL_GPIO_GIOENASET_GIOENASET_2_MAX                                  (0x000000FFU)

#define CSL_GPIO_GIOENASET_GIOENASET_3_MASK                                 (0xFF000000U)
#define CSL_GPIO_GIOENASET_GIOENASET_3_SHIFT                                (0x00000018U)
#define CSL_GPIO_GIOENASET_GIOENASET_3_RESETVAL                             (0x00000000U)
#define CSL_GPIO_GIOENASET_GIOENASET_3_MAX                                  (0x000000FFU)

#define CSL_GPIO_GIOENASET_RESETVAL                                         (0x00000000U)

/* GIOENACLR */

#define CSL_GPIO_GIOENACLR_GIOENACLR_0_MASK                                 (0x000000FFU)
#define CSL_GPIO_GIOENACLR_GIOENACLR_0_SHIFT                                (0x00000000U)
#define CSL_GPIO_GIOENACLR_GIOENACLR_0_RESETVAL                             (0x00000000U)
#define CSL_GPIO_GIOENACLR_GIOENACLR_0_MAX                                  (0x000000FFU)

#define CSL_GPIO_GIOENACLR_GIOENACLR_1_MASK                                 (0x0000FF00U)
#define CSL_GPIO_GIOENACLR_GIOENACLR_1_SHIFT                                (0x00000008U)
#define CSL_GPIO_GIOENACLR_GIOENACLR_1_RESETVAL                             (0x00000000U)
#define CSL_GPIO_GIOENACLR_GIOENACLR_1_MAX                                  (0x000000FFU)

#define CSL_GPIO_GIOENACLR_GIOENACLR_2_MASK                                 (0x00FF0000U)
#define CSL_GPIO_GIOENACLR_GIOENACLR_2_SHIFT                                (0x00000010U)
#define CSL_GPIO_GIOENACLR_GIOENACLR_2_RESETVAL                             (0x00000000U)
#define CSL_GPIO_GIOENACLR_GIOENACLR_2_MAX                                  (0x000000FFU)

#define CSL_GPIO_GIOENACLR_GIOENACLR_3_MASK                                 (0xFF000000U)
#define CSL_GPIO_GIOENACLR_GIOENACLR_3_SHIFT                                (0x00000018U)
#define CSL_GPIO_GIOENACLR_GIOENACLR_3_RESETVAL                             (0x00000000U)
#define CSL_GPIO_GIOENACLR_GIOENACLR_3_MAX                                  (0x000000FFU)

#define CSL_GPIO_GIOENACLR_RESETVAL                                         (0x00000000U)

/* GIOLVLSET */

#define CSL_GPIO_GIOLVLSET_GIOLVLSET_0_MASK                                 (0x000000FFU)
#define CSL_GPIO_GIOLVLSET_GIOLVLSET_0_SHIFT                                (0x00000000U)
#define CSL_GPIO_GIOLVLSET_GIOLVLSET_0_RESETVAL                             (0x00000000U)
#define CSL_GPIO_GIOLVLSET_GIOLVLSET_0_MAX                                  (0x000000FFU)

#define CSL_GPIO_GIOLVLSET_GIOLVLSET_1_MASK                                 (0x0000FF00U)
#define CSL_GPIO_GIOLVLSET_GIOLVLSET_1_SHIFT                                (0x00000008U)
#define CSL_GPIO_GIOLVLSET_GIOLVLSET_1_RESETVAL                             (0x00000000U)
#define CSL_GPIO_GIOLVLSET_GIOLVLSET_1_MAX                                  (0x000000FFU)

#define CSL_GPIO_GIOLVLSET_GIOLVLSET_2_MASK                                 (0x00FF0000U)
#define CSL_GPIO_GIOLVLSET_GIOLVLSET_2_SHIFT                                (0x00000010U)
#define CSL_GPIO_GIOLVLSET_GIOLVLSET_2_RESETVAL                             (0x00000000U)
#define CSL_GPIO_GIOLVLSET_GIOLVLSET_2_MAX                                  (0x000000FFU)

#define CSL_GPIO_GIOLVLSET_GIOLVLSET_3_MASK                                 (0xFF000000U)
#define CSL_GPIO_GIOLVLSET_GIOLVLSET_3_SHIFT                                (0x00000018U)
#define CSL_GPIO_GIOLVLSET_GIOLVLSET_3_RESETVAL                             (0x00000000U)
#define CSL_GPIO_GIOLVLSET_GIOLVLSET_3_MAX                                  (0x000000FFU)

#define CSL_GPIO_GIOLVLSET_RESETVAL                                         (0x00000000U)

/* GIOLVLCLR */

#define CSL_GPIO_GIOLVLCLR_GIOLVLCLR_0_MASK                                 (0x000000FFU)
#define CSL_GPIO_GIOLVLCLR_GIOLVLCLR_0_SHIFT                                (0x00000000U)
#define CSL_GPIO_GIOLVLCLR_GIOLVLCLR_0_RESETVAL                             (0x00000000U)
#define CSL_GPIO_GIOLVLCLR_GIOLVLCLR_0_MAX                                  (0x000000FFU)

#define CSL_GPIO_GIOLVLCLR_GIOLVLCLR_1_MASK                                 (0x0000FF00U)
#define CSL_GPIO_GIOLVLCLR_GIOLVLCLR_1_SHIFT                                (0x00000008U)
#define CSL_GPIO_GIOLVLCLR_GIOLVLCLR_1_RESETVAL                             (0x00000000U)
#define CSL_GPIO_GIOLVLCLR_GIOLVLCLR_1_MAX                                  (0x000000FFU)

#define CSL_GPIO_GIOLVLCLR_GIOLVLCLR_2_MASK                                 (0x00FF0000U)
#define CSL_GPIO_GIOLVLCLR_GIOLVLCLR_2_SHIFT                                (0x00000010U)
#define CSL_GPIO_GIOLVLCLR_GIOLVLCLR_2_RESETVAL                             (0x00000000U)
#define CSL_GPIO_GIOLVLCLR_GIOLVLCLR_2_MAX                                  (0x000000FFU)

#define CSL_GPIO_GIOLVLCLR_GIOLVLCLR_3_MASK                                 (0xFF000000U)
#define CSL_GPIO_GIOLVLCLR_GIOLVLCLR_3_SHIFT                                (0x00000018U)
#define CSL_GPIO_GIOLVLCLR_GIOLVLCLR_3_RESETVAL                             (0x00000000U)
#define CSL_GPIO_GIOLVLCLR_GIOLVLCLR_3_MAX                                  (0x000000FFU)

#define CSL_GPIO_GIOLVLCLR_RESETVAL                                         (0x00000000U)

/* GIOFLG */

#define CSL_GPIO_GIOFLG_GIOFLG_0_MASK                                       (0x000000FFU)
#define CSL_GPIO_GIOFLG_GIOFLG_0_SHIFT                                      (0x00000000U)
#define CSL_GPIO_GIOFLG_GIOFLG_0_RESETVAL                                   (0x00000000U)
#define CSL_GPIO_GIOFLG_GIOFLG_0_MAX                                        (0x000000FFU)

#define CSL_GPIO_GIOFLG_GIOFLG_1_MASK                                       (0x0000FF00U)
#define CSL_GPIO_GIOFLG_GIOFLG_1_SHIFT                                      (0x00000008U)
#define CSL_GPIO_GIOFLG_GIOFLG_1_RESETVAL                                   (0x00000000U)
#define CSL_GPIO_GIOFLG_GIOFLG_1_MAX                                        (0x000000FFU)

#define CSL_GPIO_GIOFLG_GIOFLG_2_MASK                                       (0x00FF0000U)
#define CSL_GPIO_GIOFLG_GIOFLG_2_SHIFT                                      (0x00000010U)
#define CSL_GPIO_GIOFLG_GIOFLG_2_RESETVAL                                   (0x00000000U)
#define CSL_GPIO_GIOFLG_GIOFLG_2_MAX                                        (0x000000FFU)

#define CSL_GPIO_GIOFLG_GIOFLG_3_MASK                                       (0xFF000000U)
#define CSL_GPIO_GIOFLG_GIOFLG_3_SHIFT                                      (0x00000018U)
#define CSL_GPIO_GIOFLG_GIOFLG_3_RESETVAL                                   (0x00000000U)
#define CSL_GPIO_GIOFLG_GIOFLG_3_MAX                                        (0x000000FFU)

#define CSL_GPIO_GIOFLG_RESETVAL                                            (0x00000000U)

/* GIOOFFA */

#define CSL_GPIO_GIOOFFA_GIOOFFA_MASK                                       (0x0000003FU)
#define CSL_GPIO_GIOOFFA_GIOOFFA_SHIFT                                      (0x00000000U)
#define CSL_GPIO_GIOOFFA_GIOOFFA_RESETVAL                                   (0x00000000U)
#define CSL_GPIO_GIOOFFA_GIOOFFA_MAX                                        (0x0000003FU)

#define CSL_GPIO_GIOOFFA_NU1_MASK                                           (0xFFFFFFC0U)
#define CSL_GPIO_GIOOFFA_NU1_SHIFT                                          (0x00000006U)
#define CSL_GPIO_GIOOFFA_NU1_RESETVAL                                       (0x00000000U)
#define CSL_GPIO_GIOOFFA_NU1_MAX                                            (0x03FFFFFFU)

#define CSL_GPIO_GIOOFFA_RESETVAL                                           (0x00000000U)

/* GIOOFFB */

#define CSL_GPIO_GIOOFFB_GIOOFFB_MASK                                       (0x0000003FU)
#define CSL_GPIO_GIOOFFB_GIOOFFB_SHIFT                                      (0x00000000U)
#define CSL_GPIO_GIOOFFB_GIOOFFB_RESETVAL                                   (0x00000000U)
#define CSL_GPIO_GIOOFFB_GIOOFFB_MAX                                        (0x0000003FU)

#define CSL_GPIO_GIOOFFB_NU2_MASK                                           (0xFFFFFFC0U)
#define CSL_GPIO_GIOOFFB_NU2_SHIFT                                          (0x00000006U)
#define CSL_GPIO_GIOOFFB_NU2_RESETVAL                                       (0x00000000U)
#define CSL_GPIO_GIOOFFB_NU2_MAX                                            (0x03FFFFFFU)

#define CSL_GPIO_GIOOFFB_RESETVAL                                           (0x00000000U)

/* GIOEMUA */

#define CSL_GPIO_GIOEMUA_GIOEMUA_MASK                                       (0x0000003FU)
#define CSL_GPIO_GIOEMUA_GIOEMUA_SHIFT                                      (0x00000000U)
#define CSL_GPIO_GIOEMUA_GIOEMUA_RESETVAL                                   (0x00000000U)
#define CSL_GPIO_GIOEMUA_GIOEMUA_MAX                                        (0x0000003FU)

#define CSL_GPIO_GIOEMUA_NU3_MASK                                           (0xFFFFFFC0U)
#define CSL_GPIO_GIOEMUA_NU3_SHIFT                                          (0x00000006U)
#define CSL_GPIO_GIOEMUA_NU3_RESETVAL                                       (0x00000000U)
#define CSL_GPIO_GIOEMUA_NU3_MAX                                            (0x03FFFFFFU)

#define CSL_GPIO_GIOEMUA_RESETVAL                                           (0x00000000U)

/* GIOEMUB */

#define CSL_GPIO_GIOEMUB_GIOEMUB_MASK                                       (0x0000003FU)
#define CSL_GPIO_GIOEMUB_GIOEMUB_SHIFT                                      (0x00000000U)
#define CSL_GPIO_GIOEMUB_GIOEMUB_RESETVAL                                   (0x00000000U)
#define CSL_GPIO_GIOEMUB_GIOEMUB_MAX                                        (0x0000003FU)

#define CSL_GPIO_GIOEMUB_NU4_MASK                                           (0xFFFFFFC0U)
#define CSL_GPIO_GIOEMUB_NU4_SHIFT                                          (0x00000006U)
#define CSL_GPIO_GIOEMUB_NU4_RESETVAL                                       (0x00000000U)
#define CSL_GPIO_GIOEMUB_NU4_MAX                                            (0x03FFFFFFU)

#define CSL_GPIO_GIOEMUB_RESETVAL                                           (0x00000000U)

/* GIODIRA */

#define CSL_GPIO_GIODIRA_GIODIRA_MASK                                       (0x000000FFU)
#define CSL_GPIO_GIODIRA_GIODIRA_SHIFT                                      (0x00000000U)
#define CSL_GPIO_GIODIRA_GIODIRA_RESETVAL                                   (0x00000000U)
#define CSL_GPIO_GIODIRA_GIODIRA_MAX                                        (0x000000FFU)

#define CSL_GPIO_GIODIRA_NU5_MASK                                           (0xFFFFFF00U)
#define CSL_GPIO_GIODIRA_NU5_SHIFT                                          (0x00000008U)
#define CSL_GPIO_GIODIRA_NU5_RESETVAL                                       (0x00000000U)
#define CSL_GPIO_GIODIRA_NU5_MAX                                            (0x00FFFFFFU)

#define CSL_GPIO_GIODIRA_RESETVAL                                           (0x00000000U)

/* GIODIN */

#define CSL_GPIO_GIODIN_GIODIN_MASK                                         (0x000000FFU)
#define CSL_GPIO_GIODIN_GIODIN_SHIFT                                        (0x00000000U)
#define CSL_GPIO_GIODIN_GIODIN_RESETVAL                                     (0x00000000U)
#define CSL_GPIO_GIODIN_GIODIN_MAX                                          (0x000000FFU)

#define CSL_GPIO_GIODIN_RESETVAL                                            (0x00000000U)

/* GIODOUT */

#define CSL_GPIO_GIODOUT_GIODOUT_MASK                                       (0x000000FFU)
#define CSL_GPIO_GIODOUT_GIODOUT_SHIFT                                      (0x00000000U)
#define CSL_GPIO_GIODOUT_GIODOUT_RESETVAL                                   (0x00000000U)
#define CSL_GPIO_GIODOUT_GIODOUT_MAX                                        (0x000000FFU)

#define CSL_GPIO_GIODOUT_RESETVAL                                           (0x00000000U)

/* GIOSET */

#define CSL_GPIO_GIOSET_GIODSET_MASK                                        (0x000000FFU)
#define CSL_GPIO_GIOSET_GIODSET_SHIFT                                       (0x00000000U)
#define CSL_GPIO_GIOSET_GIODSET_RESETVAL                                    (0x00000000U)
#define CSL_GPIO_GIOSET_GIODSET_MAX                                         (0x000000FFU)

#define CSL_GPIO_GIOSET_RESETVAL                                            (0x00000000U)

/* GIOCLR */

#define CSL_GPIO_GIOCLR_GIODCLR_MASK                                        (0x000000FFU)
#define CSL_GPIO_GIOCLR_GIODCLR_SHIFT                                       (0x00000000U)
#define CSL_GPIO_GIOCLR_GIODCLR_RESETVAL                                    (0x00000000U)
#define CSL_GPIO_GIOCLR_GIODCLR_MAX                                         (0x000000FFU)

#define CSL_GPIO_GIOCLR_RESETVAL                                            (0x00000000U)

/* GIOPDR */

#define CSL_GPIO_GIOPDR_GIOPDR_MASK                                         (0x000000FFU)
#define CSL_GPIO_GIOPDR_GIOPDR_SHIFT                                        (0x00000000U)
#define CSL_GPIO_GIOPDR_GIOPDR_RESETVAL                                     (0x00000000U)
#define CSL_GPIO_GIOPDR_GIOPDR_MAX                                          (0x000000FFU)

#define CSL_GPIO_GIOPDR_RESETVAL                                            (0x00000000U)

/* GIOPULDIS */

#define CSL_GPIO_GIOPULDIS_GIOPULDIS_MASK                                   (0x000000FFU)
#define CSL_GPIO_GIOPULDIS_GIOPULDIS_SHIFT                                  (0x00000000U)
#define CSL_GPIO_GIOPULDIS_GIOPULDIS_RESETVAL                               (0x00000000U)
#define CSL_GPIO_GIOPULDIS_GIOPULDIS_MAX                                    (0x000000FFU)

#define CSL_GPIO_GIOPULDIS_RESETVAL                                         (0x00000000U)

/* GIOPSL */

#define CSL_GPIO_GIOPSL_GIOPSL_MASK                                         (0x000000FFU)
#define CSL_GPIO_GIOPSL_GIOPSL_SHIFT                                        (0x00000000U)
#define CSL_GPIO_GIOPSL_GIOPSL_RESETVAL                                     (0x00000000U)
#define CSL_GPIO_GIOPSL_GIOPSL_MAX                                          (0x000000FFU)

#define CSL_GPIO_GIOPSL_RESETVAL                                            (0x00000000U)

/* GIOSRC */

#define CSL_GPIO_GIOSRC_GIOSRC_MASK                                         (0x000000FFU)
#define CSL_GPIO_GIOSRC_GIOSRC_SHIFT                                        (0x00000000U)
#define CSL_GPIO_GIOSRC_GIOSRC_RESETVAL                                     (0x00000000U)
#define CSL_GPIO_GIOSRC_GIOSRC_MAX                                          (0x000000FFU)

#define CSL_GPIO_GIOSRC_RESETVAL                                            (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
