/*
 *   Copyright (c) 2022-23 Texas Instruments Incorporated
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

#ifndef SDL_HWA_HW_H_
#define SDL_HWA_HW_H_

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************************************
* Register Definitions
****************************************************************************************************/

#define SDL_HWA_ENABLE                          (0x014U)
#define SDL_HWA_MEM_INIT_START                  (0X600U)
#define SDL_HWA_MEM_INIT_DONE                   (0X604U)
#define SDL_HWA_MEM_INIT_STATUS                 (0X608U)
#define SDL_HWA_SAFETY_ERR_MASK                 (0X618U)
#define SDL_HWA_SAFETY_EN                       (0X614U)
#define SDL_HWA_SAFETY_ERR_STATUS               (0X61CU)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW           (0X620U)
#define SDL_HWA_SAFETY_DMEM0_ERR_ADDR           (0X624U)
#define SDL_HWA_SAFETY_DMEM1_ERR_ADDR           (0X628U)
#define SDL_HWA_SAFETY_DMEM2_ERR_ADDR           (0X62CU)
#define SDL_HWA_SAFETY_DMEM3_ERR_ADDR           (0X630U)
#define SDL_HWA_SAFETY_DMEM4_ERR_ADDR           (0X634U)
#define SDL_HWA_SAFETY_DMEM5_ERR_ADDR           (0X638U)
#define SDL_HWA_SAFETY_DMEM6_ERR_ADDR           (0X63CU)
#define SDL_HWA_SAFETY_DMEM7_ERR_ADDR           (0X640U)
#define SDL_HWA_SAFETY_WINDOW_RAM_ERR_ADDR      (0X644U)

/****************************************************************************************************
* Field Definition Macros
****************************************************************************************************/
/* HWA_ENABLE */
#define SDL_HWA_ENABLE_HWA_EN_SHIFT                             (0U)
#define SDL_HWA_ENABLE_HWA_EN_MASK                              (0X00000007U)
#define SDL_HWA_ENABLE_HWA_EN_ENABLE                            (7U)
#define SDL_HWA_ENABLE_HWA_EN_DISABLE                           (0U)
#define SDL_HWA_ENABLE_HWA_CLK_EN_SHIFT                         (8U)
#define SDL_HWA_ENABLE_HWA_CLK_EN_MASK                          (0X00000700U)
#define SDL_HWA_ENABLE_HWA_CLK_EN_ENABLE                        (7U)
#define SDL_HWA_ENABLE_HWA_CLK_EN_DISABLE                       (0U)
/* MEM_INIT_START */
#define SDL_HWA_MEM_INIT_START_WINDOW_RAM_ENABLE                (1U)
#define SDL_HWA_MEM_INIT_START_WINDOW_RAM_SHIFT                 (9U)
#define SDL_HWA_MEM_INIT_START_WINDOW_RAM_MASK                  (0X00000200U)
#define SDL_HWA_MEM_INIT_START_DMEM7_ENABLE                     (1U)
#define SDL_HWA_MEM_INIT_START_DMEM7_SHIFT                      (7U)
#define SDL_HWA_MEM_INIT_START_DMEM7_MASK                       (0X00000080U)
#define SDL_HWA_MEM_INIT_START_DMEM6_ENABLE                     (1U)
#define SDL_HWA_MEM_INIT_START_DMEM6_SHIFT                      (6U)
#define SDL_HWA_MEM_INIT_START_DMEM6_MASK                       (0X00000040U)
#define SDL_HWA_MEM_INIT_START_DMEM5_ENABLE                     (1U)
#define SDL_HWA_MEM_INIT_START_DMEM5_SHIFT                      (5U)
#define SDL_HWA_MEM_INIT_START_DMEM5_MASK                       (0X00000020U)
#define SDL_HWA_MEM_INIT_START_DMEM4_ENABLE                     (1U)
#define SDL_HWA_MEM_INIT_START_DMEM4_SHIFT                      (4U)
#define SDL_HWA_MEM_INIT_START_DMEM4_MASK                       (0X00000010U)
#define SDL_HWA_MEM_INIT_START_DMEM3_ENABLE                     (1U)
#define SDL_HWA_MEM_INIT_START_DMEM3_SHIFT                      (3U)
#define SDL_HWA_MEM_INIT_START_DMEM3_MASK                       (0X00000008U)
#define SDL_HWA_MEM_INIT_START_DMEM2_ENABLE                     (1U)
#define SDL_HWA_MEM_INIT_START_DMEM2_SHIFT                      (2U)
#define SDL_HWA_MEM_INIT_START_DMEM2_MASK                       (0X00000004U)
#define SDL_HWA_MEM_INIT_START_DMEM1_ENABLE                     (1U)
#define SDL_HWA_MEM_INIT_START_DMEM1_SHIFT                      (1U)
#define SDL_HWA_MEM_INIT_START_DMEM1_MASK                       (0X00000002U)
#define SDL_HWA_MEM_INIT_START_DMEM0_ENABLE                     (1U)
#define SDL_HWA_MEM_INIT_START_DMEM0_SHIFT                      (0U)
#define SDL_HWA_MEM_INIT_START_DMEM0_MASK                       (0X00000001U)
#define SDL_HWA_MEM_INIT_START_DMEM_ALL_ENABLE                  (0XFFU)
#define SDL_HWA_MEM_INIT_START_DMEM_ALL_SHIFT                   (0U)
#define SDL_HWA_MEM_INIT_START_DMEM_ALL_MASK                    (0X000000FFU)
/* MEM_INIT_DONE */
#define SDL_HWA_MEM_INIT_DONE_WINDOW_RAM_ENABLE                 (1U)
#define SDL_HWA_MEM_INIT_DONE_WINDOW_RAM_SHIFT                  (9U)
#define SDL_HWA_MEM_INIT_DONE_WINDOW_RAM_MASK                   (0X00000200U)
#define SDL_HWA_MEM_INIT_DONE_DMEM7_ENABLE                      (1U)
#define SDL_HWA_MEM_INIT_DONE_DMEM7_SHIFT                       (7U)
#define SDL_HWA_MEM_INIT_DONE_DMEM7_MASK                        (0X00000080U)
#define SDL_HWA_MEM_INIT_DONE_DMEM6_ENABLE                      (1U)
#define SDL_HWA_MEM_INIT_DONE_DMEM6_SHIFT                       (6U)
#define SDL_HWA_MEM_INIT_DONE_DMEM6_MASK                        (0X00000040U)
#define SDL_HWA_MEM_INIT_DONE_DMEM5_ENABLE                      (1U)
#define SDL_HWA_MEM_INIT_DONE_DMEM5_SHIFT                       (5U)
#define SDL_HWA_MEM_INIT_DONE_DMEM5_MASK                        (0X00000020U)
#define SDL_HWA_MEM_INIT_DONE_DMEM4_ENABLE                      (1U)
#define SDL_HWA_MEM_INIT_DONE_DMEM4_SHIFT                       (4U)
#define SDL_HWA_MEM_INIT_DONE_DMEM4_MASK                        (0X00000010U)
#define SDL_HWA_MEM_INIT_DONE_DMEM3_ENABLE                      (1U)
#define SDL_HWA_MEM_INIT_DONE_DMEM3_SHIFT                       (3U)
#define SDL_HWA_MEM_INIT_DONE_DMEM3_MASK                        (0X00000008U)
#define SDL_HWA_MEM_INIT_DONE_DMEM2_ENABLE                      (1U)
#define SDL_HWA_MEM_INIT_DONE_DMEM2_SHIFT                       (2U)
#define SDL_HWA_MEM_INIT_DONE_DMEM2_MASK                        (0X00000004U)
#define SDL_HWA_MEM_INIT_DONE_DMEM1_ENABLE                      (1U)
#define SDL_HWA_MEM_INIT_DONE_DMEM1_SHIFT                       (1U)
#define SDL_HWA_MEM_INIT_DONE_DMEM1_MASK                        (0X00000002U)
#define SDL_HWA_MEM_INIT_DONE_DMEM0_ENABLE                      (1U)
#define SDL_HWA_MEM_INIT_DONE_DMEM0_SHIFT                       (0U)
#define SDL_HWA_MEM_INIT_DONE_DMEM0_MASK                        (0X00000001U)
#define SDL_HWA_MEM_INIT_DONE_DMEM_ALL_ENABLE                   (0XFFU)
#define SDL_HWA_MEM_INIT_DONE_DMEM_ALL_SHIFT                    (0U)
#define SDL_HWA_MEM_INIT_DONE_DMEM_ALL_MASK                     (0X000000FFU)
/* HWA_SAFETY_ERR_STATUS_RAW */
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_FSM_LOCKSTEP_ENABLE       (1U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_FSM_LOCKSTEP_SHIFT        (9U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_FSM_LOCKSTEP_MASK         (0X00000200U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_WINDOW_RAM_ENABLE         (1U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_WINDOW_RAM_SHIFT          (8U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_WINDOW_RAM_MASK           (0X00000100U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_DMEM7_ENABLE              (1U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_DMEM7_SHIFT               (7U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_DMEM7_MASK                (0X00000080U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_DMEM6_ENABLE              (1U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_DMEM6_SHIFT               (6U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_DMEM6_MASK                (0X00000040U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_DMEM5_ENABLE              (1U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_DMEM5_SHIFT               (5U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_DMEM5_MASK                (0X00000020U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_DMEM4_ENABLE              (1U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_DMEM4_SHIFT               (4U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_DMEM4_MASK                (0X00000010U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_DMEM3_ENABLE              (1U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_DMEM3_SHIFT               (3U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_DMEM3_MASK                (0X00000008U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_DMEM2_ENABLE              (1U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_DMEM2_SHIFT               (2U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_DMEM2_MASK                (0X00000004U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_DMEM1_ENABLE              (1U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_DMEM1_SHIFT               (1U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_DMEM1_MASK                (0X00000002U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_DMEM0_ENABLE              (1U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_DMEM0_SHIFT               (0U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_DMEM0_MASK                (0X00000001U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_DMEM_ALL_ENABLE           (0XFFU)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_DMEM_ALL_SHIFT            (0U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_DMEM_ALL_MASK             (0X000000FFU)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_ALL_ENABLE                (0X3FF1U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_ALL_SHIFT                 (0U)
#define SDL_HWA_SAFETY_ERR_STATUS_RAW_ALL_MASK                  (0X000003FFU)
/* HWA_SAFETY_ERR_STATUS */
#define SDL_HWA_SAFETY_ERR_STATUS_FSM_LOCKSTEP_ENABLE           (1U)
#define SDL_HWA_SAFETY_ERR_STATUS_FSM_LOCKSTEP_DISABLE          (0U)
#define SDL_HWA_SAFETY_ERR_STATUS_FSM_LOCKSTEP_SHIFT            (9U)
#define SDL_HWA_SAFETY_ERR_STATUS_FSM_LOCKSTEP_MASK             (0X00000200U)
#define SDL_HWA_SAFETY_ERR_STATUS_WINDOW_RAM_ENABLE             (1U)
#define SDL_HWA_SAFETY_ERR_STATUS_WINDOW_RAM_DISABLE            (0U)
#define SDL_HWA_SAFETY_ERR_STATUS_WINDOW_RAM_SHIFT              (8U)
#define SDL_HWA_SAFETY_ERR_STATUS_WINDOW_RAM_MASK               (0X00000100U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM7_ENABLE                  (1U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM7_DISABLE                 (0U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM7_SHIFT                   (7U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM7_MASK                    (0X00000080U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM6_ENABLE                  (1U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM6_DISABLE                 (0U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM6_SHIFT                   (6U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM6_MASK                    (0X00000040U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM5_ENABLE                  (1U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM5_DISABLE                 (0U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM5_SHIFT                   (5U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM5_MASK                    (0X00000020U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM4_ENABLE                  (1U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM4_DISABLE                 (0U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM4_SHIFT                   (4U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM4_MASK                    (0X00000010U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM3_ENABLE                  (1U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM3_DISABLE                 (0U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM3_SHIFT                   (3U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM3_MASK                    (0X00000008U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM2_ENABLE                  (1U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM2_DISABLE                 (0U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM2_SHIFT                   (2U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM2_MASK                    (0X00000004U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM1_ENABLE                  (1U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM1_DISABLE                 (0U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM1_SHIFT                   (1U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM1_MASK                    (0X00000002U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM0_ENABLE                  (1U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM0_DISABLE                 (0U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM0_SHIFT                   (0U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM0_MASK                    (0X00000001U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM_ALL_ENABLE               (0XFFU)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM_ALL_SHIFT                (0U)
#define SDL_HWA_SAFETY_ERR_STATUS_DMEM_ALL_MASK                 (0X000000FFU)
#define SDL_HWA_SAFETY_ERR_STATUS_ALL_ENABLE                    (0X3FFU)
#define SDL_HWA_SAFETY_ERR_STATUS_ALL_DISABLE                   (0X000U)
#define SDL_HWA_SAFETY_ERR_STATUS_ALL_SHIFT                     (0U)
#define SDL_HWA_SAFETY_ERR_STATUS_ALL_MASK                      (0X000003FFU)
/* HWA_SAFETY_ERR_MASK */
#define SDL_HWA_SAFETY_ERR_MASK_FSM_LOCKSTEP_ENABLE             (1U)
#define SDL_HWA_SAFETY_ERR_MASK_FSM_LOCKSTEP_DISABLE             (0U)
#define SDL_HWA_SAFETY_ERR_MASK_FSM_LOCKSTEP_SHIFT              (9U)
#define SDL_HWA_SAFETY_ERR_MASK_FSM_LOCKSTEP_MASK               (0X00000200U)
#define SDL_HWA_SAFETY_ERR_MASK_WINDOW_RAM_ENABLE               (1U)
#define SDL_HWA_SAFETY_ERR_MASK_WINDOW_RAM_DISABLE              (0U)
#define SDL_HWA_SAFETY_ERR_MASK_WINDOW_RAM_SHIFT                (8U)
#define SDL_HWA_SAFETY_ERR_MASK_WINDOW_RAM_MASK                 (0X00000100U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM7_ENABLE                    (1U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM7_DISABLE                   (0U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM7_SHIFT                     (7U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM7_MASK                      (0X00000080U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM6_ENABLE                    (1U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM6_DISABLE                   (0U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM6_SHIFT                     (6U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM6_MASK                      (0X00000040U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM5_ENABLE                    (1U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM5_DISABLE                   (0U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM5_SHIFT                     (5U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM5_MASK                      (0X00000020U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM4_ENABLE                    (1U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM4_DISABLE                   (0U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM4_SHIFT                     (4U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM4_MASK                      (0X00000010U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM3_ENABLE                    (1U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM3_DISABLE                   (0U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM3_SHIFT                     (3U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM3_MASK                      (0X00000008U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM2_ENABLE                   (1U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM2_DISABLE                  (0U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM2_SHIFT                    (2U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM2_MASK                     (0X00000004U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM1_ENABLE                   (1U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM1_DISABLE                  (0U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM1_SHIFT                    (1U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM1_MASK                     (0X00000002U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM0_ENABLE                   (1U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM0_DISABLE                  (0U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM0_SHIFT                    (0U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM0_MASK                     (0X00000001U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM_ALL_ENABLE                (0XFFU)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM_ALL_SHIFT                 (0U)
#define SDL_HWA_SAFETY_ERR_MASK_DMEM_ALL_MASK                  (0X000000FFU)
/* HWA_SAFETY_DMEM0_ERR_ADDR */
#define SDL_HWA_SAFETY_DMEMX_ERR_ADDR_SHIFT                    (0U)
#define SDL_HWA_SAFETY_DMEMX_ERR_ADDR_MASK                     (0X000003FFU)
/* HWA_SAFETY_WINDOW_RAM_ERR_ADDR */
#define SDL_HWA_SAFETY_WINDOW_RAM_ERR_ADDR_SHIFT               (0U)
#define SDL_HWA_SAFETY_WINDOW_RAM_ERR_ADDR_MASK                (0X000007FFU)
/* HWA_SAFETY_EN */
#define SDL_HWA_SAFETY_EN_CFG_DMEM_PARITY_EN_SHIFT             (3U)
#define SDL_HWA_SAFETY_EN_CFG_DMEM_PARITY_EN_MASK              (0X00000008U)
#define SDL_HWA_SAFETY_EN_CFG_DMEM_PARITY_EN_ENABLE            (1U)
#define SDL_HWA_SAFETY_EN_CFG_DMEM_PARITY_EN_DISABLE           (0U)
#define SDL_HWA_SAFETY_EN_CFG_WINDOW_RAM_PARITY_EN_SHIFT       (2U)
#define SDL_HWA_SAFETY_EN_CFG_WINDOW_RAM_PARITY_EN_MASK        (0X00000004U)
#define SDL_HWA_SAFETY_EN_CFG_WINDOW_RAM_PARITY_EN_ENABLE      (1U)
#define SDL_HWA_SAFETY_EN_CFG_WINDOW_RAM_PARITY_EN_DISABLE     (0U)
#define SDL_HWA_SAFETY_EN_CFG_FSM_LOCKSTEP_INV_EN_SHIFT        (1U)
#define SDL_HWA_SAFETY_EN_CFG_FSM_LOCKSTEP_INV_EN_MASK         (0X00000002U)
#define SDL_HWA_SAFETY_EN_CFG_FSM_LOCKSTEP_INV_EN_ENABLE       (1U)
#define SDL_HWA_SAFETY_EN_CFG_FSM_LOCKSTEP_INV_EN_DISABLE      (0U)
#define SDL_HWA_SAFETY_EN_CFG_FSM_LOCKSTEP_EN_SHIFT            (0U)
#define SDL_HWA_SAFETY_EN_CFG_FSM_LOCKSTEP_EN_MASK             (0X00000001U)
#define SDL_HWA_SAFETY_EN_CFG_FSM_LOCKSTEP_EN_ENABLE           (1U)
#define SDL_HWA_SAFETY_EN_CFG_FSM_LOCKSTEP_EN_DISABLE          (0U)
/* MEM_INIT_STATUS */
#define SDL_HWA_MEM_INIT_STATUS_WINDOW_RAM_ENABLE              (1U)
#define SDL_HWA_MEM_INIT_STATUS_WINDOW_RAM_SHIFT               (9U)
#define SDL_HWA_MEM_INIT_STATUS_WINDOW_RAM_MASK                (0X00000200U)
#define SDL_HWA_MEM_INIT_STATUS_DMEM7_ENABLE                   (1U)
#define SDL_HWA_MEM_INIT_STATUS_DMEM7_SHIFT                    (7U)
#define SDL_HWA_MEM_INIT_STATUS_DMEM7_MASK                     (0X00000080U)
#define SDL_HWA_MEM_INIT_STATUS_DMEM6_ENABLE                   (1U)
#define SDL_HWA_MEM_INIT_STATUS_DMEM6_SHIFT                    (6U)
#define SDL_HWA_MEM_INIT_STATUS_DMEM6_MASK                     (0X00000040U)
#define SDL_HWA_MEM_INIT_STATUS_DMEM5_ENABLE                   (1U)
#define SDL_HWA_MEM_INIT_STATUS_DMEM5_SHIFT                    (5U)
#define SDL_HWA_MEM_INIT_STATUS_DMEM5_MASK                     (0X00000020U)
#define SDL_HWA_MEM_INIT_STATUS_DMEM4_ENABLE                   (1U)
#define SDL_HWA_MEM_INIT_STATUS_DMEM4_SHIFT                    (4U)
#define SDL_HWA_MEM_INIT_STATUS_DMEM4_MASK                     (0X00000010U)
#define SDL_HWA_MEM_INIT_STATUS_DMEM3_ENABLE                   (1U)
#define SDL_HWA_MEM_INIT_STATUS_DMEM3_SHIFT                    (3U)
#define SDL_HWA_MEM_INIT_STATUS_DMEM3_MASK                     (0X00000008U)
#define SDL_HWA_MEM_INIT_STATUS_DMEM2_ENABLE                   (1U)
#define SDL_HWA_MEM_INIT_STATUS_DMEM2_SHIFT                    (2U)
#define SDL_HWA_MEM_INIT_STATUS_DMEM2_MASK                     (0X00000004U)
#define SDL_HWA_MEM_INIT_STATUS_DMEM1_ENABLE                   (1U)
#define SDL_HWA_MEM_INIT_STATUS_DMEM1_SHIFT                    (1U)
#define SDL_HWA_MEM_INIT_STATUS_DMEM1_MASK                     (0X00000002U)
#define SDL_HWA_MEM_INIT_STATUS_DMEM0_ENABLE                   (1U)
#define SDL_HWA_MEM_INIT_STATUS_DMEM0_SHIFT                    (0U)
#define SDL_HWA_MEM_INIT_STATUS_DMEM0_MASK                     (0X00000001U)
/* HWA_SAFETY_DMEM_ERR_ADDR */
#define SDL_HWA_SAFETY_DMEM_ERR_ADDR_SHIFT                     (0U)
#define SDL_HWA_SAFETY_DMEM_ERR_ADDR_MASK                      (0X000003FFU)
/* HWA_SAFETY_DMEM_ERR_ADDR */
#define SDL_HWA_SAFETY_WINDOW_RAM_ERR_ADDR_SHIFT               (0U)
#define SDL_HWA_SAFETY_WINDOW_RAM_ERR_ADDR_MASK                (0X000007FFU)

#ifdef __cplusplus
}
#endif
#endif  /* SDL_HWA_HW_H_ */

