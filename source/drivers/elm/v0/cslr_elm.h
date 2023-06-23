/********************************************************************
 * Copyright (C) 2023 Texas Instruments Incorporated.
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
 *  Name        : cslr_elm.h
*/
#ifndef CSLR_ELM_H
#define CSLR_ELM_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>


/**************************************************************************
* Register Overlay Structure for Header
**************************************************************************/
typedef struct {
    volatile uint32_t REVISION;
    volatile uint8_t  RSVD0[12];
    volatile uint32_t SYSCONFIG;
    volatile uint32_t SYSSTS;
    volatile uint32_t IRQSTS;
    volatile uint32_t IRQEN;
    volatile uint32_t LOCATION_CONFIG;
    volatile uint8_t  RSVD1[92];
    volatile uint32_t PAGE_CTRL;
} CSL_ElmHeaderRegs;


/**************************************************************************
* Register Overlay Structure for ELM_SYNDROMES
**************************************************************************/
typedef struct {
    volatile uint32_t SYNDROME_FRAGMENT_0;
    volatile uint32_t SYNDROME_FRAGMENT_1;
    volatile uint32_t SYNDROME_FRAGMENT_2;
    volatile uint32_t SYNDROME_FRAGMENT_3;
    volatile uint32_t SYNDROME_FRAGMENT_4;
    volatile uint32_t SYNDROME_FRAGMENT_5;
    volatile uint32_t SYNDROME_FRAGMENT_6;
    volatile uint8_t  RSVD0[36];
} CSL_ElmSyndromesRegs;


/**************************************************************************
* Register Overlay Structure for ELM_ERROR_LOCATIONS
**************************************************************************/
typedef struct {
    volatile uint32_t LOCATION_STS;
    volatile uint8_t  RSVD0[124];
    volatile uint32_t ERROR_LOCATION_0;
    volatile uint32_t ERROR_LOCATION_1;
    volatile uint32_t ERROR_LOCATION_2;
    volatile uint32_t ERROR_LOCATION_3;
    volatile uint32_t ERROR_LOCATION_4;
    volatile uint32_t ERROR_LOCATION_5;
    volatile uint32_t ERROR_LOCATION_6;
    volatile uint32_t ERROR_LOCATION_7;
    volatile uint32_t ERROR_LOCATION_8;
    volatile uint32_t ERROR_LOCATION_9;
    volatile uint32_t ERROR_LOCATION_10;
    volatile uint32_t ERROR_LOCATION_11;
    volatile uint32_t ERROR_LOCATION_12;
    volatile uint32_t ERROR_LOCATION_13;
    volatile uint32_t ERROR_LOCATION_14;
    volatile uint32_t ERROR_LOCATION_15;
    volatile uint8_t  RSVD1[64];
} CSL_ElmError_locationsRegs;


/**************************************************************************
* Register Overlay Structure
**************************************************************************/
typedef struct {
    CSL_ElmHeaderRegs	HEADER;
    volatile uint8_t  RSVD2[892];
    CSL_ElmSyndromesRegs	SYNDROMES[8];
    volatile uint8_t  RSVD3[512];
    CSL_ElmError_locationsRegs	ERROR_LOCATIONS[8];
} CSL_elmRegs;




/**************************************************************************
* Register Macros
**************************************************************************/

/* Register Macros for OCP_target */

/* This register contains the IP revision code. (A write to this register has
 * no effect, the same as the reset) */
#define CSL_ELM_REVISION                                        (0x0U)

/* This register allows controlling various parameters of the OCP interface */
#define CSL_ELM_SYSCONFIG                                       (0x10U)

/* Internal Reset monitoring (OCP domain) Undefined since: on HW perspective
 * reset state is 0 on SW user perspective when module is accessible is 1 */
#define CSL_ELM_SYSSTS                                          (0x14U)

/* Interrupt status. This register doubles as a status register for the error
 * location processes. */
#define CSL_ELM_IRQSTS                                          (0x18U)

/* Interrupt enable */
#define CSL_ELM_IRQEN                                           (0x1CU)

/* ECC algorithm parameters */
#define CSL_ELM_LOCATION_CONFIG                                 (0x20U)

/* Page definition */
#define CSL_ELM_PAGE_CTRL                                       (0x80U)

/* Input syndrome polynomial bits 32 to 63. */
#define CSL_ELM_SYNDROME_FRAGMENT_1(n)                          (0x404U + ((n) * (0x40U)))

/* Input syndrome polynomial bits 64 to 95. */
#define CSL_ELM_SYNDROME_FRAGMENT_2(n)                          (0x408U + ((n) * (0x40U)))

/* Input syndrome polynomial bits 192 to 207. */
#define CSL_ELM_SYNDROME_FRAGMENT_6(n)                          (0x418U + ((n) * (0x40U)))

/* Input syndrome polynomial bits 0 to 31. */
#define CSL_ELM_SYNDROME_FRAGMENT_0(n)                          (0x400U + ((n) * (0x40U)))

/* Input syndrome polynomial bits 96 to 127 */
#define CSL_ELM_SYNDROME_FRAGMENT_3(n)                          (0x40CU + ((n) * (0x40U)))

/* Input syndrome polynomial bits 128 to 159. */
#define CSL_ELM_SYNDROME_FRAGMENT_4(n)                          (0x410U + ((n) * (0x40U)))

/* Input syndrome polynomial bits 160 to 191. */
#define CSL_ELM_SYNDROME_FRAGMENT_5(n)                          (0x414U + ((n) * (0x40U)))

/* Error location register */
#define CSL_ELM_ERROR_LOCATION_7(n)                             (0x89CU + ((n) * (0x100U)))

/* Error location register */
#define CSL_ELM_ERROR_LOCATION_6(n)                             (0x898U + ((n) * (0x100U)))

/* Error location register */
#define CSL_ELM_ERROR_LOCATION_5(n)                             (0x894U + ((n) * (0x100U)))

/* Error location register */
#define CSL_ELM_ERROR_LOCATION_4(n)                             (0x890U + ((n) * (0x100U)))

/* Error location register */
#define CSL_ELM_ERROR_LOCATION_3(n)                             (0x88CU + ((n) * (0x100U)))

/* Error location register */
#define CSL_ELM_ERROR_LOCATION_2(n)                             (0x888U + ((n) * (0x100U)))

/* Error location register */
#define CSL_ELM_ERROR_LOCATION_1(n)                             (0x884U + ((n) * (0x100U)))

/* Error location register */
#define CSL_ELM_ERROR_LOCATION_0(n)                             (0x880U + ((n) * (0x100U)))

/* Exit status for the syndrome polynomial processing */
#define CSL_ELM_LOCATION_STS(n)                                 (0x800U + ((n) * (0x100U)))

/* Error location register */
#define CSL_ELM_ERROR_LOCATION_11(n)                            (0x8ACU + ((n) * (0x100U)))

/* Error location register */
#define CSL_ELM_ERROR_LOCATION_13(n)                            (0x8B4U + ((n) * (0x100U)))

/* Error location register */
#define CSL_ELM_ERROR_LOCATION_15(n)                            (0x8BCU + ((n) * (0x100U)))

/* Error location register */
#define CSL_ELM_ERROR_LOCATION_10(n)                            (0x8A8U + ((n) * (0x100U)))

/* Error location register */
#define CSL_ELM_ERROR_LOCATION_8(n)                             (0x8A0U + ((n) * (0x100U)))

/* Error location register */
#define CSL_ELM_ERROR_LOCATION_9(n)                             (0x8A4U + ((n) * (0x100U)))

/* Error location register */
#define CSL_ELM_ERROR_LOCATION_14(n)                            (0x8B8U + ((n) * (0x100U)))

/* Error location register */
#define CSL_ELM_ERROR_LOCATION_12(n)                            (0x8B0U + ((n) * (0x100U)))


/**************************************************************************
* Field Definition Macros
**************************************************************************/

/* REVISION */

#define CSL_ELM_REVISION_REV_NUMBER_SHIFT                       (0U)
#define CSL_ELM_REVISION_REV_NUMBER_MASK                        (0x000000FFU)
#define CSL_ELM_REVISION_REV_NUMBER_RESETVAL                    (0x00000020U)
#define CSL_ELM_REVISION_REV_NUMBER_MAX                         (0x000000ffU)

#define CSL_ELM_REVISION_RESETVAL                               (0x00000020U)

/* SYSCONFIG */

#define CSL_ELM_SYSCONFIG_AUTOGATING_SHIFT                      (0U)
#define CSL_ELM_SYSCONFIG_AUTOGATING_MASK                       (0x00000001U)
#define CSL_ELM_SYSCONFIG_AUTOGATING_RESETVAL                   (0x00000001U)
#define CSL_ELM_SYSCONFIG_AUTOGATING_OCP_FREE                   (0x00000000U)
#define CSL_ELM_SYSCONFIG_AUTOGATING_OCP_GATING                 (0x00000001U)

#define CSL_ELM_SYSCONFIG_SOFTRESET_SHIFT                       (1U)
#define CSL_ELM_SYSCONFIG_SOFTRESET_MASK                        (0x00000002U)
#define CSL_ELM_SYSCONFIG_SOFTRESET_RESETVAL                    (0x00000000U)
#define CSL_ELM_SYSCONFIG_SOFTRESET_RESET                       (0x00000002U)
#define CSL_ELM_SYSCONFIG_SOFTRESET_NORMAL                      (0x00000000U)

#define CSL_ELM_SYSCONFIG_CLOCKACTIVITYOCP_SHIFT                (8U)
#define CSL_ELM_SYSCONFIG_CLOCKACTIVITYOCP_MASK                 (0x00000100U)
#define CSL_ELM_SYSCONFIG_CLOCKACTIVITYOCP_RESETVAL             (0x00000000U)
#define CSL_ELM_SYSCONFIG_CLOCKACTIVITYOCP_OCP_OFF              (0x00000000U)
#define CSL_ELM_SYSCONFIG_CLOCKACTIVITYOCP_OCP_ON               (0x00000100U)

#define CSL_ELM_SYSCONFIG_SIDLEMODE_SHIFT                       (3U)
#define CSL_ELM_SYSCONFIG_SIDLEMODE_MASK                        (0x00000018U)
#define CSL_ELM_SYSCONFIG_SIDLEMODE_RESETVAL                    (0x00000002U)
#define CSL_ELM_SYSCONFIG_SIDLEMODE_NO_IDLE                     (0x00000008U)
#define CSL_ELM_SYSCONFIG_SIDLEMODE_RESERVED                    (0x00000018U)
#define CSL_ELM_SYSCONFIG_SIDLEMODE_FORCE_IDLE                  (0x00000000U)
#define CSL_ELM_SYSCONFIG_SIDLEMODE_SMART_IDLE                  (0x00000010U)

#define CSL_ELM_SYSCONFIG_RESETVAL                              (0x00000011U)

/* SYSSTS */

#define CSL_ELM_SYSSTS_RESETDONE_SHIFT                          (0U)
#define CSL_ELM_SYSSTS_RESETDONE_MASK                           (0x00000001U)
#define CSL_ELM_SYSSTS_RESETDONE_RESETVAL                       (0x00000001U)
#define CSL_ELM_SYSSTS_RESETDONE_RST_DONE                       (0x00000001U)
#define CSL_ELM_SYSSTS_RESETDONE_RST_ONGOING                    (0x00000000U)

#define CSL_ELM_SYSSTS_RESETVAL                                 (0x00000001U)

/* IRQSTS */

#define CSL_ELM_IRQSTS_LOC_VALID_0_SHIFT                        (0U)
#define CSL_ELM_IRQSTS_LOC_VALID_0_MASK                         (0x00000001U)
#define CSL_ELM_IRQSTS_LOC_VALID_0_RESETVAL                     (0x00000000U)
#define CSL_ELM_IRQSTS_LOC_VALID_0_MAX                          (0x00000001U)

#define CSL_ELM_IRQSTS_LOC_VALID_1_SHIFT                        (1U)
#define CSL_ELM_IRQSTS_LOC_VALID_1_MASK                         (0x00000002U)
#define CSL_ELM_IRQSTS_LOC_VALID_1_RESETVAL                     (0x00000000U)
#define CSL_ELM_IRQSTS_LOC_VALID_1_MAX                          (0x00000001U)

#define CSL_ELM_IRQSTS_LOC_VALID_2_SHIFT                        (2U)
#define CSL_ELM_IRQSTS_LOC_VALID_2_MASK                         (0x00000004U)
#define CSL_ELM_IRQSTS_LOC_VALID_2_RESETVAL                     (0x00000000U)
#define CSL_ELM_IRQSTS_LOC_VALID_2_MAX                          (0x00000001U)

#define CSL_ELM_IRQSTS_LOC_VALID_3_SHIFT                        (3U)
#define CSL_ELM_IRQSTS_LOC_VALID_3_MASK                         (0x00000008U)
#define CSL_ELM_IRQSTS_LOC_VALID_3_RESETVAL                     (0x00000000U)
#define CSL_ELM_IRQSTS_LOC_VALID_3_MAX                          (0x00000001U)

#define CSL_ELM_IRQSTS_LOC_VALID_4_SHIFT                        (4U)
#define CSL_ELM_IRQSTS_LOC_VALID_4_MASK                         (0x00000010U)
#define CSL_ELM_IRQSTS_LOC_VALID_4_RESETVAL                     (0x00000000U)
#define CSL_ELM_IRQSTS_LOC_VALID_4_MAX                          (0x00000001U)

#define CSL_ELM_IRQSTS_LOC_VALID_5_SHIFT                        (5U)
#define CSL_ELM_IRQSTS_LOC_VALID_5_MASK                         (0x00000020U)
#define CSL_ELM_IRQSTS_LOC_VALID_5_RESETVAL                     (0x00000000U)
#define CSL_ELM_IRQSTS_LOC_VALID_5_MAX                          (0x00000001U)

#define CSL_ELM_IRQSTS_LOC_VALID_6_SHIFT                        (6U)
#define CSL_ELM_IRQSTS_LOC_VALID_6_MASK                         (0x00000040U)
#define CSL_ELM_IRQSTS_LOC_VALID_6_RESETVAL                     (0x00000000U)
#define CSL_ELM_IRQSTS_LOC_VALID_6_MAX                          (0x00000001U)

#define CSL_ELM_IRQSTS_LOC_VALID_7_SHIFT                        (7U)
#define CSL_ELM_IRQSTS_LOC_VALID_7_MASK                         (0x00000080U)
#define CSL_ELM_IRQSTS_LOC_VALID_7_RESETVAL                     (0x00000000U)
#define CSL_ELM_IRQSTS_LOC_VALID_7_MAX                          (0x00000001U)

#define CSL_ELM_IRQSTS_PAGE_VALID_SHIFT                         (8U)
#define CSL_ELM_IRQSTS_PAGE_VALID_MASK                          (0x00000100U)
#define CSL_ELM_IRQSTS_PAGE_VALID_RESETVAL                      (0x00000000U)
#define CSL_ELM_IRQSTS_PAGE_VALID_MAX                           (0x00000001U)

#define CSL_ELM_IRQSTS_RESETVAL                                 (0x00000000U)

/* IRQEN */

#define CSL_ELM_IRQEN_LOCATION_MASK_0_SHIFT                     (0U)
#define CSL_ELM_IRQEN_LOCATION_MASK_0_MASK                      (0x00000001U)
#define CSL_ELM_IRQEN_LOCATION_MASK_0_RESETVAL                  (0x00000000U)
#define CSL_ELM_IRQEN_LOCATION_MASK_0_MAX                       (0x00000001U)

#define CSL_ELM_IRQEN_LOCATION_MASK_1_SHIFT                     (1U)
#define CSL_ELM_IRQEN_LOCATION_MASK_1_MASK                      (0x00000002U)
#define CSL_ELM_IRQEN_LOCATION_MASK_1_RESETVAL                  (0x00000000U)
#define CSL_ELM_IRQEN_LOCATION_MASK_1_MAX                       (0x00000001U)

#define CSL_ELM_IRQEN_LOCATION_MASK_2_SHIFT                     (2U)
#define CSL_ELM_IRQEN_LOCATION_MASK_2_MASK                      (0x00000004U)
#define CSL_ELM_IRQEN_LOCATION_MASK_2_RESETVAL                  (0x00000000U)
#define CSL_ELM_IRQEN_LOCATION_MASK_2_MAX                       (0x00000001U)

#define CSL_ELM_IRQEN_LOCATION_MASK_3_SHIFT                     (3U)
#define CSL_ELM_IRQEN_LOCATION_MASK_3_MASK                      (0x00000008U)
#define CSL_ELM_IRQEN_LOCATION_MASK_3_RESETVAL                  (0x00000000U)
#define CSL_ELM_IRQEN_LOCATION_MASK_3_MAX                       (0x00000001U)

#define CSL_ELM_IRQEN_LOCATION_MASK_4_SHIFT                     (4U)
#define CSL_ELM_IRQEN_LOCATION_MASK_4_MASK                      (0x00000010U)
#define CSL_ELM_IRQEN_LOCATION_MASK_4_RESETVAL                  (0x00000000U)
#define CSL_ELM_IRQEN_LOCATION_MASK_4_MAX                       (0x00000001U)

#define CSL_ELM_IRQEN_LOCATION_MASK_5_SHIFT                     (5U)
#define CSL_ELM_IRQEN_LOCATION_MASK_5_MASK                      (0x00000020U)
#define CSL_ELM_IRQEN_LOCATION_MASK_5_RESETVAL                  (0x00000000U)
#define CSL_ELM_IRQEN_LOCATION_MASK_5_MAX                       (0x00000001U)

#define CSL_ELM_IRQEN_LOCATION_MASK_6_SHIFT                     (6U)
#define CSL_ELM_IRQEN_LOCATION_MASK_6_MASK                      (0x00000040U)
#define CSL_ELM_IRQEN_LOCATION_MASK_6_RESETVAL                  (0x00000000U)
#define CSL_ELM_IRQEN_LOCATION_MASK_6_MAX                       (0x00000001U)

#define CSL_ELM_IRQEN_LOCATION_MASK_7_SHIFT                     (7U)
#define CSL_ELM_IRQEN_LOCATION_MASK_7_MASK                      (0x00000080U)
#define CSL_ELM_IRQEN_LOCATION_MASK_7_RESETVAL                  (0x00000000U)
#define CSL_ELM_IRQEN_LOCATION_MASK_7_MAX                       (0x00000001U)

#define CSL_ELM_IRQEN_PAGE_MASK_SHIFT                           (8U)
#define CSL_ELM_IRQEN_PAGE_MASK_MASK                            (0x00000100U)
#define CSL_ELM_IRQEN_PAGE_MASK_RESETVAL                        (0x00000000U)
#define CSL_ELM_IRQEN_PAGE_MASK_MAX                             (0x00000001U)

#define CSL_ELM_IRQEN_RESETVAL                                  (0x00000000U)

/* LOCATION_CONFIG */

#define CSL_ELM_LOCATION_CONFIG_ECC_BCH_LEVEL_SHIFT             (0U)
#define CSL_ELM_LOCATION_CONFIG_ECC_BCH_LEVEL_MASK              (0x00000003U)
#define CSL_ELM_LOCATION_CONFIG_ECC_BCH_LEVEL_RESETVAL          (0x00000000U)
#define CSL_ELM_LOCATION_CONFIG_ECC_BCH_LEVEL_MAX               (0x00000003U)

#define CSL_ELM_LOCATION_CONFIG_ECC_SIZE_SHIFT                  (16U)
#define CSL_ELM_LOCATION_CONFIG_ECC_SIZE_MASK                   (0x07FF0000U)
#define CSL_ELM_LOCATION_CONFIG_ECC_SIZE_RESETVAL               (0x00000000U)
#define CSL_ELM_LOCATION_CONFIG_ECC_SIZE_MAX                    (0x000007ffU)

#define CSL_ELM_LOCATION_CONFIG_RESETVAL                        (0x00000000U)

/* PAGE_CTRL */

#define CSL_ELM_PAGE_CTRL_SECTOR_0_SHIFT                        (0U)
#define CSL_ELM_PAGE_CTRL_SECTOR_0_MASK                         (0x00000001U)
#define CSL_ELM_PAGE_CTRL_SECTOR_0_RESETVAL                     (0x00000000U)
#define CSL_ELM_PAGE_CTRL_SECTOR_0_MAX                          (0x00000001U)

#define CSL_ELM_PAGE_CTRL_SECTOR_1_SHIFT                        (1U)
#define CSL_ELM_PAGE_CTRL_SECTOR_1_MASK                         (0x00000002U)
#define CSL_ELM_PAGE_CTRL_SECTOR_1_RESETVAL                     (0x00000000U)
#define CSL_ELM_PAGE_CTRL_SECTOR_1_MAX                          (0x00000001U)

#define CSL_ELM_PAGE_CTRL_SECTOR_2_SHIFT                        (2U)
#define CSL_ELM_PAGE_CTRL_SECTOR_2_MASK                         (0x00000004U)
#define CSL_ELM_PAGE_CTRL_SECTOR_2_RESETVAL                     (0x00000000U)
#define CSL_ELM_PAGE_CTRL_SECTOR_2_MAX                          (0x00000001U)

#define CSL_ELM_PAGE_CTRL_SECTOR_3_SHIFT                        (3U)
#define CSL_ELM_PAGE_CTRL_SECTOR_3_MASK                         (0x00000008U)
#define CSL_ELM_PAGE_CTRL_SECTOR_3_RESETVAL                     (0x00000000U)
#define CSL_ELM_PAGE_CTRL_SECTOR_3_MAX                          (0x00000001U)

#define CSL_ELM_PAGE_CTRL_SECTOR_4_SHIFT                        (4U)
#define CSL_ELM_PAGE_CTRL_SECTOR_4_MASK                         (0x00000010U)
#define CSL_ELM_PAGE_CTRL_SECTOR_4_RESETVAL                     (0x00000000U)
#define CSL_ELM_PAGE_CTRL_SECTOR_4_MAX                          (0x00000001U)

#define CSL_ELM_PAGE_CTRL_SECTOR_5_SHIFT                        (5U)
#define CSL_ELM_PAGE_CTRL_SECTOR_5_MASK                         (0x00000020U)
#define CSL_ELM_PAGE_CTRL_SECTOR_5_RESETVAL                     (0x00000000U)
#define CSL_ELM_PAGE_CTRL_SECTOR_5_MAX                          (0x00000001U)

#define CSL_ELM_PAGE_CTRL_SECTOR_6_SHIFT                        (6U)
#define CSL_ELM_PAGE_CTRL_SECTOR_6_MASK                         (0x00000040U)
#define CSL_ELM_PAGE_CTRL_SECTOR_6_RESETVAL                     (0x00000000U)
#define CSL_ELM_PAGE_CTRL_SECTOR_6_MAX                          (0x00000001U)

#define CSL_ELM_PAGE_CTRL_SECTOR_7_SHIFT                        (7U)
#define CSL_ELM_PAGE_CTRL_SECTOR_7_MASK                         (0x00000080U)
#define CSL_ELM_PAGE_CTRL_SECTOR_7_RESETVAL                     (0x00000000U)
#define CSL_ELM_PAGE_CTRL_SECTOR_7_MAX                          (0x00000001U)

#define CSL_ELM_PAGE_CTRL_RESETVAL                              (0x00000000U)

/* SYNDROME_FRAGMENT_1 */

#define CSL_ELM_SYNDROME_FRAGMENT_1_SYNDROME_1_SHIFT            (0U)
#define CSL_ELM_SYNDROME_FRAGMENT_1_SYNDROME_1_MASK             (0xFFFFFFFFU)
#define CSL_ELM_SYNDROME_FRAGMENT_1_SYNDROME_1_RESETVAL         (0x00000000U)
#define CSL_ELM_SYNDROME_FRAGMENT_1_SYNDROME_1_MAX              (0xffffffffU)

#define CSL_ELM_SYNDROME_FRAGMENT_1_RESETVAL                    (0x00000000U)

/* SYNDROME_FRAGMENT_2 */

#define CSL_ELM_SYNDROME_FRAGMENT_2_SYNDROME_2_SHIFT            (0U)
#define CSL_ELM_SYNDROME_FRAGMENT_2_SYNDROME_2_MASK             (0xFFFFFFFFU)
#define CSL_ELM_SYNDROME_FRAGMENT_2_SYNDROME_2_RESETVAL         (0x00000000U)
#define CSL_ELM_SYNDROME_FRAGMENT_2_SYNDROME_2_MAX              (0xffffffffU)

#define CSL_ELM_SYNDROME_FRAGMENT_2_RESETVAL                    (0x00000000U)

/* SYNDROME_FRAGMENT_6 */

#define CSL_ELM_SYNDROME_FRAGMENT_6_SYNDROME_6_SHIFT            (0U)
#define CSL_ELM_SYNDROME_FRAGMENT_6_SYNDROME_6_MASK             (0x0000FFFFU)
#define CSL_ELM_SYNDROME_FRAGMENT_6_SYNDROME_6_RESETVAL         (0x00000000U)
#define CSL_ELM_SYNDROME_FRAGMENT_6_SYNDROME_6_MAX              (0x0000ffffU)

#define CSL_ELM_SYNDROME_FRAGMENT_6_SYNDROME_VALID_SHIFT        (16U)
#define CSL_ELM_SYNDROME_FRAGMENT_6_SYNDROME_VALID_MASK         (0x00010000U)
#define CSL_ELM_SYNDROME_FRAGMENT_6_SYNDROME_VALID_RESETVAL     (0x00000000U)
#define CSL_ELM_SYNDROME_FRAGMENT_6_SYNDROME_VALID_MAX          (0x00000001U)

#define CSL_ELM_SYNDROME_FRAGMENT_6_RESETVAL                    (0x00000000U)

/* SYNDROME_FRAGMENT_0 */

#define CSL_ELM_SYNDROME_FRAGMENT_0_SYNDROME_0_SHIFT            (0U)
#define CSL_ELM_SYNDROME_FRAGMENT_0_SYNDROME_0_MASK             (0xFFFFFFFFU)
#define CSL_ELM_SYNDROME_FRAGMENT_0_SYNDROME_0_RESETVAL         (0x00000000U)
#define CSL_ELM_SYNDROME_FRAGMENT_0_SYNDROME_0_MAX              (0xffffffffU)

#define CSL_ELM_SYNDROME_FRAGMENT_0_RESETVAL                    (0x00000000U)

/* SYNDROME_FRAGMENT_3 */

#define CSL_ELM_SYNDROME_FRAGMENT_3_SYNDROME_3_SHIFT            (0U)
#define CSL_ELM_SYNDROME_FRAGMENT_3_SYNDROME_3_MASK             (0xFFFFFFFFU)
#define CSL_ELM_SYNDROME_FRAGMENT_3_SYNDROME_3_RESETVAL         (0x00000000U)
#define CSL_ELM_SYNDROME_FRAGMENT_3_SYNDROME_3_MAX              (0xffffffffU)

#define CSL_ELM_SYNDROME_FRAGMENT_3_RESETVAL                    (0x00000000U)

/* SYNDROME_FRAGMENT_4 */

#define CSL_ELM_SYNDROME_FRAGMENT_4_SYNDROME_4_SHIFT            (0U)
#define CSL_ELM_SYNDROME_FRAGMENT_4_SYNDROME_4_MASK             (0xFFFFFFFFU)
#define CSL_ELM_SYNDROME_FRAGMENT_4_SYNDROME_4_RESETVAL         (0x00000000U)
#define CSL_ELM_SYNDROME_FRAGMENT_4_SYNDROME_4_MAX              (0xffffffffU)

#define CSL_ELM_SYNDROME_FRAGMENT_4_RESETVAL                    (0x00000000U)

/* SYNDROME_FRAGMENT_5 */

#define CSL_ELM_SYNDROME_FRAGMENT_5_SYNDROME_5_SHIFT            (0U)
#define CSL_ELM_SYNDROME_FRAGMENT_5_SYNDROME_5_MASK             (0xFFFFFFFFU)
#define CSL_ELM_SYNDROME_FRAGMENT_5_SYNDROME_5_RESETVAL         (0x00000000U)
#define CSL_ELM_SYNDROME_FRAGMENT_5_SYNDROME_5_MAX              (0xffffffffU)

#define CSL_ELM_SYNDROME_FRAGMENT_5_RESETVAL                    (0x00000000U)

/* ERROR_LOCATION_7 */

#define CSL_ELM_ERROR_LOCATION_7_ECC_ERROR_LOCATION_SHIFT       (0U)
#define CSL_ELM_ERROR_LOCATION_7_ECC_ERROR_LOCATION_MASK        (0x00001FFFU)
#define CSL_ELM_ERROR_LOCATION_7_ECC_ERROR_LOCATION_RESETVAL    (0x00000000U)
#define CSL_ELM_ERROR_LOCATION_7_ECC_ERROR_LOCATION_MAX         (0x00001fffU)

#define CSL_ELM_ERROR_LOCATION_7_RESETVAL                       (0x00000000U)

/* ERROR_LOCATION_6 */

#define CSL_ELM_ERROR_LOCATION_6_ECC_ERROR_LOCATION_SHIFT       (0U)
#define CSL_ELM_ERROR_LOCATION_6_ECC_ERROR_LOCATION_MASK        (0x00001FFFU)
#define CSL_ELM_ERROR_LOCATION_6_ECC_ERROR_LOCATION_RESETVAL    (0x00000000U)
#define CSL_ELM_ERROR_LOCATION_6_ECC_ERROR_LOCATION_MAX         (0x00001fffU)

#define CSL_ELM_ERROR_LOCATION_6_RESETVAL                       (0x00000000U)

/* ERROR_LOCATION_5 */

#define CSL_ELM_ERROR_LOCATION_5_ECC_ERROR_LOCATION_SHIFT       (0U)
#define CSL_ELM_ERROR_LOCATION_5_ECC_ERROR_LOCATION_MASK        (0x00001FFFU)
#define CSL_ELM_ERROR_LOCATION_5_ECC_ERROR_LOCATION_RESETVAL    (0x00000000U)
#define CSL_ELM_ERROR_LOCATION_5_ECC_ERROR_LOCATION_MAX         (0x00001fffU)

#define CSL_ELM_ERROR_LOCATION_5_RESETVAL                       (0x00000000U)

/* ERROR_LOCATION_4 */

#define CSL_ELM_ERROR_LOCATION_4_ECC_ERROR_LOCATION_SHIFT       (0U)
#define CSL_ELM_ERROR_LOCATION_4_ECC_ERROR_LOCATION_MASK        (0x00001FFFU)
#define CSL_ELM_ERROR_LOCATION_4_ECC_ERROR_LOCATION_RESETVAL    (0x00000000U)
#define CSL_ELM_ERROR_LOCATION_4_ECC_ERROR_LOCATION_MAX         (0x00001fffU)

#define CSL_ELM_ERROR_LOCATION_4_RESETVAL                       (0x00000000U)

/* ERROR_LOCATION_3 */

#define CSL_ELM_ERROR_LOCATION_3_ECC_ERROR_LOCATION_SHIFT       (0U)
#define CSL_ELM_ERROR_LOCATION_3_ECC_ERROR_LOCATION_MASK        (0x00001FFFU)
#define CSL_ELM_ERROR_LOCATION_3_ECC_ERROR_LOCATION_RESETVAL    (0x00000000U)
#define CSL_ELM_ERROR_LOCATION_3_ECC_ERROR_LOCATION_MAX         (0x00001fffU)

#define CSL_ELM_ERROR_LOCATION_3_RESETVAL                       (0x00000000U)

/* ERROR_LOCATION_2 */

#define CSL_ELM_ERROR_LOCATION_2_ECC_ERROR_LOCATION_SHIFT       (0U)
#define CSL_ELM_ERROR_LOCATION_2_ECC_ERROR_LOCATION_MASK        (0x00001FFFU)
#define CSL_ELM_ERROR_LOCATION_2_ECC_ERROR_LOCATION_RESETVAL    (0x00000000U)
#define CSL_ELM_ERROR_LOCATION_2_ECC_ERROR_LOCATION_MAX         (0x00001fffU)

#define CSL_ELM_ERROR_LOCATION_2_RESETVAL                       (0x00000000U)

/* ERROR_LOCATION_1 */

#define CSL_ELM_ERROR_LOCATION_1_ECC_ERROR_LOCATION_SHIFT       (0U)
#define CSL_ELM_ERROR_LOCATION_1_ECC_ERROR_LOCATION_MASK        (0x00001FFFU)
#define CSL_ELM_ERROR_LOCATION_1_ECC_ERROR_LOCATION_RESETVAL    (0x00000000U)
#define CSL_ELM_ERROR_LOCATION_1_ECC_ERROR_LOCATION_MAX         (0x00001fffU)

#define CSL_ELM_ERROR_LOCATION_1_RESETVAL                       (0x00000000U)

/* ERROR_LOCATION_0 */

#define CSL_ELM_ERROR_LOCATION_0_ECC_ERROR_LOCATION_SHIFT       (0U)
#define CSL_ELM_ERROR_LOCATION_0_ECC_ERROR_LOCATION_MASK        (0x00001FFFU)
#define CSL_ELM_ERROR_LOCATION_0_ECC_ERROR_LOCATION_RESETVAL    (0x00000000U)
#define CSL_ELM_ERROR_LOCATION_0_ECC_ERROR_LOCATION_MAX         (0x00001fffU)

#define CSL_ELM_ERROR_LOCATION_0_RESETVAL                       (0x00000000U)

/* LOCATION_STS */

#define CSL_ELM_LOCATION_STS_ECC_NB_ERRORS_SHIFT                (0U)
#define CSL_ELM_LOCATION_STS_ECC_NB_ERRORS_MASK                 (0x0000001FU)
#define CSL_ELM_LOCATION_STS_ECC_NB_ERRORS_RESETVAL             (0x00000000U)
#define CSL_ELM_LOCATION_STS_ECC_NB_ERRORS_MAX                  (0x0000001fU)

#define CSL_ELM_LOCATION_STS_ECC_CORRECTBL_SHIFT                (8U)
#define CSL_ELM_LOCATION_STS_ECC_CORRECTBL_MASK                 (0x00000100U)
#define CSL_ELM_LOCATION_STS_ECC_CORRECTBL_RESETVAL             (0x00000000U)
#define CSL_ELM_LOCATION_STS_ECC_CORRECTBL_MAX                  (0x00000001U)

#define CSL_ELM_LOCATION_STS_RESETVAL                           (0x00000000U)

/* ERROR_LOCATION_11 */

#define CSL_ELM_ERROR_LOCATION_11_ECC_ERROR_LOCATION_SHIFT      (0U)
#define CSL_ELM_ERROR_LOCATION_11_ECC_ERROR_LOCATION_MASK       (0x00001FFFU)
#define CSL_ELM_ERROR_LOCATION_11_ECC_ERROR_LOCATION_RESETVAL   (0x00000000U)
#define CSL_ELM_ERROR_LOCATION_11_ECC_ERROR_LOCATION_MAX        (0x00001fffU)

#define CSL_ELM_ERROR_LOCATION_11_RESETVAL                      (0x00000000U)

/* ERROR_LOCATION_13 */

#define CSL_ELM_ERROR_LOCATION_13_ECC_ERROR_LOCATION_SHIFT      (0U)
#define CSL_ELM_ERROR_LOCATION_13_ECC_ERROR_LOCATION_MASK       (0x00001FFFU)
#define CSL_ELM_ERROR_LOCATION_13_ECC_ERROR_LOCATION_RESETVAL   (0x00000000U)
#define CSL_ELM_ERROR_LOCATION_13_ECC_ERROR_LOCATION_MAX        (0x00001fffU)

#define CSL_ELM_ERROR_LOCATION_13_RESETVAL                      (0x00000000U)

/* ERROR_LOCATION_15 */

#define CSL_ELM_ERROR_LOCATION_15_ECC_ERROR_LOCATION_SHIFT      (0U)
#define CSL_ELM_ERROR_LOCATION_15_ECC_ERROR_LOCATION_MASK       (0x00001FFFU)
#define CSL_ELM_ERROR_LOCATION_15_ECC_ERROR_LOCATION_RESETVAL   (0x00000000U)
#define CSL_ELM_ERROR_LOCATION_15_ECC_ERROR_LOCATION_MAX        (0x00001fffU)

#define CSL_ELM_ERROR_LOCATION_15_RESETVAL                      (0x00000000U)

/* ERROR_LOCATION_10 */

#define CSL_ELM_ERROR_LOCATION_10_ECC_ERROR_LOCATION_SHIFT      (0U)
#define CSL_ELM_ERROR_LOCATION_10_ECC_ERROR_LOCATION_MASK       (0x00001FFFU)
#define CSL_ELM_ERROR_LOCATION_10_ECC_ERROR_LOCATION_RESETVAL   (0x00000000U)
#define CSL_ELM_ERROR_LOCATION_10_ECC_ERROR_LOCATION_MAX        (0x00001fffU)

#define CSL_ELM_ERROR_LOCATION_10_RESETVAL                      (0x00000000U)

/* ERROR_LOCATION_8 */

#define CSL_ELM_ERROR_LOCATION_8_ECC_ERROR_LOCATION_SHIFT       (0U)
#define CSL_ELM_ERROR_LOCATION_8_ECC_ERROR_LOCATION_MASK        (0x00001FFFU)
#define CSL_ELM_ERROR_LOCATION_8_ECC_ERROR_LOCATION_RESETVAL    (0x00000000U)
#define CSL_ELM_ERROR_LOCATION_8_ECC_ERROR_LOCATION_MAX         (0x00001fffU)

#define CSL_ELM_ERROR_LOCATION_8_RESETVAL                       (0x00000000U)

/* ERROR_LOCATION_9 */

#define CSL_ELM_ERROR_LOCATION_9_ECC_ERROR_LOCATION_SHIFT       (0U)
#define CSL_ELM_ERROR_LOCATION_9_ECC_ERROR_LOCATION_MASK        (0x00001FFFU)
#define CSL_ELM_ERROR_LOCATION_9_ECC_ERROR_LOCATION_RESETVAL    (0x00000000U)
#define CSL_ELM_ERROR_LOCATION_9_ECC_ERROR_LOCATION_MAX         (0x00001fffU)

#define CSL_ELM_ERROR_LOCATION_9_RESETVAL                       (0x00000000U)

/* ERROR_LOCATION_14 */

#define CSL_ELM_ERROR_LOCATION_14_ECC_ERROR_LOCATION_SHIFT      (0U)
#define CSL_ELM_ERROR_LOCATION_14_ECC_ERROR_LOCATION_MASK       (0x00001FFFU)
#define CSL_ELM_ERROR_LOCATION_14_ECC_ERROR_LOCATION_RESETVAL   (0x00000000U)
#define CSL_ELM_ERROR_LOCATION_14_ECC_ERROR_LOCATION_MAX        (0x00001fffU)

#define CSL_ELM_ERROR_LOCATION_14_RESETVAL                      (0x00000000U)

/* ERROR_LOCATION_12 */

#define CSL_ELM_ERROR_LOCATION_12_ECC_ERROR_LOCATION_SHIFT      (0U)
#define CSL_ELM_ERROR_LOCATION_12_ECC_ERROR_LOCATION_MASK       (0x00001FFFU)
#define CSL_ELM_ERROR_LOCATION_12_ECC_ERROR_LOCATION_RESETVAL   (0x00000000U)
#define CSL_ELM_ERROR_LOCATION_12_ECC_ERROR_LOCATION_MAX        (0x00001fffU)

#define CSL_ELM_ERROR_LOCATION_12_RESETVAL                      (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
