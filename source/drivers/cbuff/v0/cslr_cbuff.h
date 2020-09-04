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
 *  Name        : cslr_cbuff.h
*/
#ifndef CSLR_CBUFF_H_
#define CSLR_CBUFF_H_

#ifdef __cplusplus
extern "C"
{
#endif
/*#include <ti/csl/cslr.h>
#include <stdint.h>*/

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t CFG_DATA_LL;
    volatile uint32_t CFG_DATA_LL_LPHDR_VAL;
    volatile uint32_t CFG_DATA_LL_THRESHOLD;
} CSL_cbuffRegs_LL;


typedef struct {
    volatile uint32_t CONFIG_REG_0;
    volatile uint32_t CFG_SPHDR_ADDRESS;
    volatile uint32_t CFG_CMD_HSVAL;
    volatile uint32_t CFG_CMD_HEVAL;
    volatile uint32_t CFG_CMD_VSVAL;
    volatile uint32_t CFG_CMD_VEVAL;
    volatile uint32_t CFG_LPHDR_ADDRESS;
    volatile uint8_t  Resv_32[4];
    volatile uint32_t CFG_CHIRPS_PER_FRAME;
    volatile uint32_t CFG_FIFO_FREE_THRESHOLD;
    volatile uint32_t CFG_LPPYLD_ADDRESS;
    volatile uint32_t CFG_DELAY_CONFIG;
    CSL_cbuffRegs_LL  LL_ENTRY[32];
    /* SOFTWARE Modification: Making an array would make the driver easier to
     * code and save on memory. */
    #if 0
    volatile uint32_t CFG_LVDS_MAPPING_LANE0_FMT_0;
    volatile uint32_t CFG_LVDS_MAPPING_LANE1_FMT_0;
    volatile uint32_t CFG_LVDS_MAPPING_LANE2_FMT_0;
    volatile uint32_t CFG_LVDS_MAPPING_LANE3_FMT_0;
    volatile uint32_t CFG_LVDS_MAPPING_LANE0_FMT_1;
    volatile uint32_t CFG_LVDS_MAPPING_LANE1_FMT_1;
    volatile uint32_t CFG_LVDS_MAPPING_LANE2_FMT_1;
    volatile uint32_t CFG_LVDS_MAPPING_LANE3_FMT_1;
    #else
    volatile uint32_t CFG_LVDS_MAPPING_LANE_FMT[8];
    #endif
    volatile uint32_t CFG_LVDS_GEN_0;
    volatile uint32_t CFG_LVDS_GEN_1;
    volatile uint32_t CFG_LVDS_GEN_2;
    volatile uint32_t CFG_MASK_REG0;
    volatile uint32_t CFG_MASK_REG1;
    volatile uint32_t CFG_MASK_REG2;
    volatile uint32_t CFG_MASK_REG3;
    volatile uint32_t STAT_CBUFF_REG0;
    volatile uint32_t STAT_CBUFF_REG1;
    volatile uint32_t STAT_CBUFF_REG2;
    volatile uint32_t STAT_CBUFF_REG3;
    volatile uint32_t STAT_LVDS_REG0;
    volatile uint32_t STAT_LVDS_REG1;
    volatile uint32_t STAT_LVDS_REG2;
    volatile uint32_t STAT_LVDS_REG3;
    volatile uint32_t CLR_CBUFF_REG0;
    volatile uint32_t CLR_CBUFF_REG1;
    volatile uint32_t CLR_LVDS_REG0;
    volatile uint32_t CLR_LVDS_REG1;
    volatile uint32_t STAT_CBUFF_ECC_REG;
    volatile uint32_t MASK_CBUFF_ECC_REG;
    volatile uint32_t CLR_CBUFF_ECC_REG;
    volatile uint32_t STAT_SAFETY;
    volatile uint32_t MASK_SAFETY;
    volatile uint32_t CLR_SAFETY;
} CSL_CbuffRegs;

/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_CBUFF_CONFIG_REG_0                                             (0x00000000U)
#define CSL_CBUFF_CFG_SPHDR_ADDRESS                                        (0x00000004U)
#define CSL_CBUFF_CFG_CMD_HSVAL                                            (0x00000008U)
#define CSL_CBUFF_CFG_CMD_HEVAL                                            (0x0000000CU)
#define CSL_CBUFF_CFG_CMD_VSVAL                                            (0x00000010U)
#define CSL_CBUFF_CFG_CMD_VEVAL                                            (0x00000014U)
#define CSL_CBUFF_CFG_LPHDR_ADDRESS                                        (0x00000018U)
#define CSL_CBUFF_CFG_CHIRPS_PER_FRAME                                     (0x00000020U)
#define CSL_CBUFF_CFG_FIFO_FREE_THRESHOLD                                  (0x00000024U)
#define CSL_CBUFF_CFG_LPPYLD_ADDRESS                                       (0x00000028U)
#define CSL_CBUFF_CFG_DELAY_CONFIG                                         (0x0000002CU)
#define CSL_CBUFF_CFG_DATA_LL(n)                                           (0x30U + (n)*0xC)
#define CSL_CBUFF_CFG_DATA_LL_LPHDR_VAL(n)                                 (0x34U + (n)*0xC)
#define CSL_CBUFF_CFG_DATA_LL_THRESHOLD(n)                                 (0x38U + (n)*0xC)
#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT(n)                             (0x01B0U + (n)*0x4)
#define CSL_CBUFF_CFG_LVDS_GEN_0                                           (0x000001D0U)
#define CSL_CBUFF_CFG_LVDS_GEN_1                                           (0x000001D4U)
#define CSL_CBUFF_CFG_LVDS_GEN_2                                           (0x000001D8U)
#define CSL_CBUFF_CFG_MASK_REG0                                            (0x000001DCU)
#define CSL_CBUFF_CFG_MASK_REG1                                            (0x000001E0U)
#define CSL_CBUFF_CFG_MASK_REG2                                            (0x000001E4U)
#define CSL_CBUFF_CFG_MASK_REG3                                            (0x000001E8U)
#define CSL_CBUFF_STAT_CBUFF_REG0                                          (0x000001ECU)
#define CSL_CBUFF_STAT_CBUFF_REG1                                          (0x000001F0U)
#define CSL_CBUFF_STAT_CBUFF_REG2                                          (0x000001F4U)
#define CSL_CBUFF_STAT_CBUFF_REG3                                          (0x000001F8U)
#define CSL_CBUFF_STAT_LVDS_REG0                                           (0x000001FCU)
#define CSL_CBUFF_STAT_LVDS_REG1                                           (0x00000200U)
#define CSL_CBUFF_STAT_LVDS_REG2                                           (0x00000204U)
#define CSL_CBUFF_STAT_LVDS_REG3                                           (0x00000208U)
#define CSL_CBUFF_CLR_CBUFF_REG0                                           (0x0000020CU)
#define CSL_CBUFF_CLR_CBUFF_REG1                                           (0x00000210U)
#define CSL_CBUFF_CLR_LVDS_REG0                                            (0x00000214U)
#define CSL_CBUFF_CLR_LVDS_REG1                                            (0x00000218U)
#define CSL_CBUFF_STAT_CBUFF_ECC_REG                                       (0x0000021CU)
#define CSL_CBUFF_MASK_CBUFF_ECC_REG                                       (0x00000220U)
#define CSL_CBUFF_CLR_CBUFF_ECC_REG                                        (0x00000224U)
#define CSL_CBUFF_STAT_SAFETY                                              (0x00000228U)
#define CSL_CBUFF_MASK_SAFETY                                              (0x0000022CU)
#define CSL_CBUFF_CLR_SAFETY                                               (0x00000230U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* CONFIG_REG_0 */

#define CSL_CBUFF_CONFIG_REG_0_CFG_1LVDS_0CSI_MASK                         (0x00000001U)
#define CSL_CBUFF_CONFIG_REG_0_CFG_1LVDS_0CSI_SHIFT                        (0x00000000U)
#define CSL_CBUFF_CONFIG_REG_0_CFG_1LVDS_0CSI_RESETVAL                     (0x00000000U)
#define CSL_CBUFF_CONFIG_REG_0_CFG_1LVDS_0CSI_MAX                          (0x00000001U)

#define CSL_CBUFF_CONFIG_REG_0_CFG_ECC_EN_MASK                             (0x00000002U)
#define CSL_CBUFF_CONFIG_REG_0_CFG_ECC_EN_SHIFT                            (0x00000001U)
#define CSL_CBUFF_CONFIG_REG_0_CFG_ECC_EN_RESETVAL                         (0x00000000U)
#define CSL_CBUFF_CONFIG_REG_0_CFG_ECC_EN_MAX                              (0x00000001U)

#define CSL_CBUFF_CONFIG_REG_0_CFTRIGEN_MASK                               (0x00000004U)
#define CSL_CBUFF_CONFIG_REG_0_CFTRIGEN_SHIFT                              (0x00000002U)
#define CSL_CBUFF_CONFIG_REG_0_CFTRIGEN_RESETVAL                           (0x00000000U)
#define CSL_CBUFF_CONFIG_REG_0_CFTRIGEN_MAX                                (0x00000001U)

#define CSL_CBUFF_CONFIG_REG_0_CFG_SW_TRIG_EN_MASK                         (0x00000008U)
#define CSL_CBUFF_CONFIG_REG_0_CFG_SW_TRIG_EN_SHIFT                        (0x00000003U)
#define CSL_CBUFF_CONFIG_REG_0_CFG_SW_TRIG_EN_RESETVAL                     (0x00000000U)
#define CSL_CBUFF_CONFIG_REG_0_CFG_SW_TRIG_EN_MAX                          (0x00000001U)

#define CSL_CBUFF_CONFIG_REG_0_NU1_MASK                                    (0x000000F0U)
#define CSL_CBUFF_CONFIG_REG_0_NU1_SHIFT                                   (0x00000004U)
#define CSL_CBUFF_CONFIG_REG_0_NU1_RESETVAL                                (0x00000000U)
#define CSL_CBUFF_CONFIG_REG_0_NU1_MAX                                     (0x0000000FU)

#define CSL_CBUFF_CONFIG_REG_0_CCFWLEN_MASK                                (0x00000100U)
#define CSL_CBUFF_CONFIG_REG_0_CCFWLEN_SHIFT                               (0x00000008U)
#define CSL_CBUFF_CONFIG_REG_0_CCFWLEN_RESETVAL                            (0x00000000U)
#define CSL_CBUFF_CONFIG_REG_0_CCFWLEN_MAX                                 (0x00000001U)

#define CSL_CBUFF_CONFIG_REG_0_CRDTHSEL_MASK                               (0x00000200U)
#define CSL_CBUFF_CONFIG_REG_0_CRDTHSEL_SHIFT                              (0x00000009U)
#define CSL_CBUFF_CONFIG_REG_0_CRDTHSEL_RESETVAL                           (0x00000000U)
#define CSL_CBUFF_CONFIG_REG_0_CRDTHSEL_MAX                                (0x00000001U)

#define CSL_CBUFF_CONFIG_REG_0_CVC0EN_MASK                                 (0x00000C00U)
#define CSL_CBUFF_CONFIG_REG_0_CVC0EN_SHIFT                                (0x0000000AU)
#define CSL_CBUFF_CONFIG_REG_0_CVC0EN_RESETVAL                             (0x00000000U)
#define CSL_CBUFF_CONFIG_REG_0_CVC0EN_MAX                                  (0x00000003U)

#define CSL_CBUFF_CONFIG_REG_0_CVC1EN_MASK                                 (0x00003000U)
#define CSL_CBUFF_CONFIG_REG_0_CVC1EN_SHIFT                                (0x0000000CU)
#define CSL_CBUFF_CONFIG_REG_0_CVC1EN_RESETVAL                             (0x00000000U)
#define CSL_CBUFF_CONFIG_REG_0_CVC1EN_MAX                                  (0x00000003U)

#define CSL_CBUFF_CONFIG_REG_0_CVC2EN_MASK                                 (0x0000C000U)
#define CSL_CBUFF_CONFIG_REG_0_CVC2EN_SHIFT                                (0x0000000EU)
#define CSL_CBUFF_CONFIG_REG_0_CVC2EN_RESETVAL                             (0x00000000U)
#define CSL_CBUFF_CONFIG_REG_0_CVC2EN_MAX                                  (0x00000003U)

#define CSL_CBUFF_CONFIG_REG_0_CVC3EN_MASK                                 (0x00030000U)
#define CSL_CBUFF_CONFIG_REG_0_CVC3EN_SHIFT                                (0x00000010U)
#define CSL_CBUFF_CONFIG_REG_0_CVC3EN_RESETVAL                             (0x00000000U)
#define CSL_CBUFF_CONFIG_REG_0_CVC3EN_MAX                                  (0x00000003U)

#define CSL_CBUFF_CONFIG_REG_0_CCFWPEN_MASK                                (0x00040000U)
#define CSL_CBUFF_CONFIG_REG_0_CCFWPEN_SHIFT                               (0x00000012U)
#define CSL_CBUFF_CONFIG_REG_0_CCFWPEN_RESETVAL                            (0x00000000U)
#define CSL_CBUFF_CONFIG_REG_0_CCFWPEN_MAX                                 (0x00000001U)

#define CSL_CBUFF_CONFIG_REG_0_DBUSEN_MASK                                 (0x00080000U)
#define CSL_CBUFF_CONFIG_REG_0_DBUSEN_SHIFT                                (0x00000013U)
#define CSL_CBUFF_CONFIG_REG_0_DBUSEN_RESETVAL                             (0x00000000U)
#define CSL_CBUFF_CONFIG_REG_0_DBUSEN_MAX                                  (0x00000001U)

#define CSL_CBUFF_CONFIG_REG_0_CFG_VBUSP_BURST_EN_MASK                     (0x00F00000U)
#define CSL_CBUFF_CONFIG_REG_0_CFG_VBUSP_BURST_EN_SHIFT                    (0x00000014U)
#define CSL_CBUFF_CONFIG_REG_0_CFG_VBUSP_BURST_EN_RESETVAL                 (0x00000000U)
#define CSL_CBUFF_CONFIG_REG_0_CFG_VBUSP_BURST_EN_MAX                      (0x0000000FU)

#define CSL_CBUFF_CONFIG_REG_0_CFG_CHIRP_AVAIL_TRIG_MASK                   (0x01000000U)
#define CSL_CBUFF_CONFIG_REG_0_CFG_CHIRP_AVAIL_TRIG_SHIFT                  (0x00000018U)
#define CSL_CBUFF_CONFIG_REG_0_CFG_CHIRP_AVAIL_TRIG_RESETVAL               (0x00000000U)
#define CSL_CBUFF_CONFIG_REG_0_CFG_CHIRP_AVAIL_TRIG_MAX                    (0x00000001U)

#define CSL_CBUFF_CONFIG_REG_0_CFG_FRAME_START_TRIG_MASK                   (0x02000000U)
#define CSL_CBUFF_CONFIG_REG_0_CFG_FRAME_START_TRIG_SHIFT                  (0x00000019U)
#define CSL_CBUFF_CONFIG_REG_0_CFG_FRAME_START_TRIG_RESETVAL               (0x00000000U)
#define CSL_CBUFF_CONFIG_REG_0_CFG_FRAME_START_TRIG_MAX                    (0x00000001U)

#define CSL_CBUFF_CONFIG_REG_0_CSWLRST_MASK                                (0x04000000U)
#define CSL_CBUFF_CONFIG_REG_0_CSWLRST_SHIFT                               (0x0000001AU)
#define CSL_CBUFF_CONFIG_REG_0_CSWLRST_RESETVAL                            (0x00000000U)
#define CSL_CBUFF_CONFIG_REG_0_CSWLRST_MAX                                 (0x00000001U)

#define CSL_CBUFF_CONFIG_REG_0_CSWCRST_MASK                                (0x08000000U)
#define CSL_CBUFF_CONFIG_REG_0_CSWCRST_SHIFT                               (0x0000001BU)
#define CSL_CBUFF_CONFIG_REG_0_CSWCRST_RESETVAL                            (0x00000000U)
#define CSL_CBUFF_CONFIG_REG_0_CSWCRST_MAX                                 (0x00000001U)

#define CSL_CBUFF_CONFIG_REG_0_DBUSSEL_MASK                                (0xF0000000U)
#define CSL_CBUFF_CONFIG_REG_0_DBUSSEL_SHIFT                               (0x0000001CU)
#define CSL_CBUFF_CONFIG_REG_0_DBUSSEL_RESETVAL                            (0x00000000U)
#define CSL_CBUFF_CONFIG_REG_0_DBUSSEL_MAX                                 (0x0000000FU)

#define CSL_CBUFF_CONFIG_REG_0_RESETVAL                                    (0x00000000U)

/* CFG_SPHDR_ADDRESS */

#define CSL_CBUFF_CFG_SPHDR_ADDRESS_CFG_SPHDR_ADDRESS_MASK                 (0xFFFFFFFFU)
#define CSL_CBUFF_CFG_SPHDR_ADDRESS_CFG_SPHDR_ADDRESS_SHIFT                (0x00000000U)
#define CSL_CBUFF_CFG_SPHDR_ADDRESS_CFG_SPHDR_ADDRESS_RESETVAL             (0x00000000U)
#define CSL_CBUFF_CFG_SPHDR_ADDRESS_CFG_SPHDR_ADDRESS_MAX                  (0xFFFFFFFFU)

#define CSL_CBUFF_CFG_SPHDR_ADDRESS_RESETVAL                               (0x00000000U)

/* CFG_CMD_HSVAL */

#define CSL_CBUFF_CFG_CMD_HSVAL_CFG_CMD_HSVAL_MASK                         (0xFFFFFFFFU)
#define CSL_CBUFF_CFG_CMD_HSVAL_CFG_CMD_HSVAL_SHIFT                        (0x00000000U)
#define CSL_CBUFF_CFG_CMD_HSVAL_CFG_CMD_HSVAL_RESETVAL                     (0x00000000U)
#define CSL_CBUFF_CFG_CMD_HSVAL_CFG_CMD_HSVAL_MAX                          (0xFFFFFFFFU)

#define CSL_CBUFF_CFG_CMD_HSVAL_RESETVAL                                   (0x00000000U)

/* CFG_CMD_HEVAL */

#define CSL_CBUFF_CFG_CMD_HEVAL_CFG_CMD_HEVAL_MASK                         (0xFFFFFFFFU)
#define CSL_CBUFF_CFG_CMD_HEVAL_CFG_CMD_HEVAL_SHIFT                        (0x00000000U)
#define CSL_CBUFF_CFG_CMD_HEVAL_CFG_CMD_HEVAL_RESETVAL                     (0x00000000U)
#define CSL_CBUFF_CFG_CMD_HEVAL_CFG_CMD_HEVAL_MAX                          (0xFFFFFFFFU)

#define CSL_CBUFF_CFG_CMD_HEVAL_RESETVAL                                   (0x00000000U)

/* CFG_CMD_VSVAL */

#define CSL_CBUFF_CFG_CMD_VSVAL_CFG_CMD_VSVAL_MASK                         (0xFFFFFFFFU)
#define CSL_CBUFF_CFG_CMD_VSVAL_CFG_CMD_VSVAL_SHIFT                        (0x00000000U)
#define CSL_CBUFF_CFG_CMD_VSVAL_CFG_CMD_VSVAL_RESETVAL                     (0x00000000U)
#define CSL_CBUFF_CFG_CMD_VSVAL_CFG_CMD_VSVAL_MAX                          (0xFFFFFFFFU)

#define CSL_CBUFF_CFG_CMD_VSVAL_RESETVAL                                   (0x00000000U)

/* CFG_CMD_VEVAL */

#define CSL_CBUFF_CFG_CMD_VEVAL_CFG_CMD_VEVAL_MASK                         (0xFFFFFFFFU)
#define CSL_CBUFF_CFG_CMD_VEVAL_CFG_CMD_VEVAL_SHIFT                        (0x00000000U)
#define CSL_CBUFF_CFG_CMD_VEVAL_CFG_CMD_VEVAL_RESETVAL                     (0x00000000U)
#define CSL_CBUFF_CFG_CMD_VEVAL_CFG_CMD_VEVAL_MAX                          (0xFFFFFFFFU)

#define CSL_CBUFF_CFG_CMD_VEVAL_RESETVAL                                   (0x00000000U)

/* CFG_LPHDR_ADDRESS */

#define CSL_CBUFF_CFG_LPHDR_ADDRESS_CFG_LPHDR_ADDRESS_MASK                 (0xFFFFFFFFU)
#define CSL_CBUFF_CFG_LPHDR_ADDRESS_CFG_LPHDR_ADDRESS_SHIFT                (0x00000000U)
#define CSL_CBUFF_CFG_LPHDR_ADDRESS_CFG_LPHDR_ADDRESS_RESETVAL             (0x00000000U)
#define CSL_CBUFF_CFG_LPHDR_ADDRESS_CFG_LPHDR_ADDRESS_MAX                  (0xFFFFFFFFU)

#define CSL_CBUFF_CFG_LPHDR_ADDRESS_RESETVAL                               (0x00000000U)

/* CFG_CHIRPS_PER_FRAME */

#define CSL_CBUFF_CFG_CHIRPS_PER_FRAME_CFG_CHIRPS_PER_FRAME_MASK           (0xFFFFFFFFU)
#define CSL_CBUFF_CFG_CHIRPS_PER_FRAME_CFG_CHIRPS_PER_FRAME_SHIFT          (0x00000000U)
#define CSL_CBUFF_CFG_CHIRPS_PER_FRAME_CFG_CHIRPS_PER_FRAME_RESETVAL       (0x00000000U)
#define CSL_CBUFF_CFG_CHIRPS_PER_FRAME_CFG_CHIRPS_PER_FRAME_MAX            (0xFFFFFFFFU)

#define CSL_CBUFF_CFG_CHIRPS_PER_FRAME_RESETVAL                            (0x00000000U)

/* CFG_FIFO_FREE_THRESHOLD */

#define CSL_CBUFF_CFG_FIFO_FREE_THRESHOLD_CFG_FIFO_FREE_THRESHOLD0_MASK    (0x000000FFU)
#define CSL_CBUFF_CFG_FIFO_FREE_THRESHOLD_CFG_FIFO_FREE_THRESHOLD0_SHIFT   (0x00000000U)
#define CSL_CBUFF_CFG_FIFO_FREE_THRESHOLD_CFG_FIFO_FREE_THRESHOLD0_RESETVAL (0x00000001U)
#define CSL_CBUFF_CFG_FIFO_FREE_THRESHOLD_CFG_FIFO_FREE_THRESHOLD0_MAX     (0x000000FFU)

#define CSL_CBUFF_CFG_FIFO_FREE_THRESHOLD_CFG_FIFO_FREE_THRESHOLD1_MASK    (0x0000FF00U)
#define CSL_CBUFF_CFG_FIFO_FREE_THRESHOLD_CFG_FIFO_FREE_THRESHOLD1_SHIFT   (0x00000008U)
#define CSL_CBUFF_CFG_FIFO_FREE_THRESHOLD_CFG_FIFO_FREE_THRESHOLD1_RESETVAL (0x00000001U)
#define CSL_CBUFF_CFG_FIFO_FREE_THRESHOLD_CFG_FIFO_FREE_THRESHOLD1_MAX     (0x000000FFU)

#define CSL_CBUFF_CFG_FIFO_FREE_THRESHOLD_CFG_FIFO_FREE_THRESHOLD2_MASK    (0x00FF0000U)
#define CSL_CBUFF_CFG_FIFO_FREE_THRESHOLD_CFG_FIFO_FREE_THRESHOLD2_SHIFT   (0x00000010U)
#define CSL_CBUFF_CFG_FIFO_FREE_THRESHOLD_CFG_FIFO_FREE_THRESHOLD2_RESETVAL (0x00000001U)
#define CSL_CBUFF_CFG_FIFO_FREE_THRESHOLD_CFG_FIFO_FREE_THRESHOLD2_MAX     (0x000000FFU)

#define CSL_CBUFF_CFG_FIFO_FREE_THRESHOLD_CFG_FIFO_FREE_THRESHOLD3_MASK    (0xFF000000U)
#define CSL_CBUFF_CFG_FIFO_FREE_THRESHOLD_CFG_FIFO_FREE_THRESHOLD3_SHIFT   (0x00000018U)
#define CSL_CBUFF_CFG_FIFO_FREE_THRESHOLD_CFG_FIFO_FREE_THRESHOLD3_RESETVAL (0x00000001U)
#define CSL_CBUFF_CFG_FIFO_FREE_THRESHOLD_CFG_FIFO_FREE_THRESHOLD3_MAX     (0x000000FFU)

#define CSL_CBUFF_CFG_FIFO_FREE_THRESHOLD_RESETVAL                         (0x01010101U)

/* CFG_LPPYLD_ADDRESS */

#define CSL_CBUFF_CFG_LPPYLD_ADDRESS_CFG_LPPYLD_ADDRESS_MASK               (0xFFFFFFFFU)
#define CSL_CBUFF_CFG_LPPYLD_ADDRESS_CFG_LPPYLD_ADDRESS_SHIFT              (0x00000000U)
#define CSL_CBUFF_CFG_LPPYLD_ADDRESS_CFG_LPPYLD_ADDRESS_RESETVAL           (0x00000000U)
#define CSL_CBUFF_CFG_LPPYLD_ADDRESS_CFG_LPPYLD_ADDRESS_MAX                (0xFFFFFFFFU)

#define CSL_CBUFF_CFG_LPPYLD_ADDRESS_RESETVAL                              (0x00000000U)

/* CFG_DELAY_CONFIG */

#define CSL_CBUFF_CFG_DELAY_CONFIG_CFG_SPHDR_DELAY_MASK                    (0x000000FFU)
#define CSL_CBUFF_CFG_DELAY_CONFIG_CFG_SPHDR_DELAY_SHIFT                   (0x00000000U)
#define CSL_CBUFF_CFG_DELAY_CONFIG_CFG_SPHDR_DELAY_RESETVAL                (0x00000000U)
#define CSL_CBUFF_CFG_DELAY_CONFIG_CFG_SPHDR_DELAY_MAX                     (0x000000FFU)

#define CSL_CBUFF_CFG_DELAY_CONFIG_CFG_LPHDR_DELAY_MASK                    (0x0000FF00U)
#define CSL_CBUFF_CFG_DELAY_CONFIG_CFG_LPHDR_DELAY_SHIFT                   (0x00000008U)
#define CSL_CBUFF_CFG_DELAY_CONFIG_CFG_LPHDR_DELAY_RESETVAL                (0x00000000U)
#define CSL_CBUFF_CFG_DELAY_CONFIG_CFG_LPHDR_DELAY_MAX                     (0x000000FFU)

#define CSL_CBUFF_CFG_DELAY_CONFIG_CFG_DATA_WR_DELAY_MASK                  (0x00FF0000U)
#define CSL_CBUFF_CFG_DELAY_CONFIG_CFG_DATA_WR_DELAY_SHIFT                 (0x00000010U)
#define CSL_CBUFF_CFG_DELAY_CONFIG_CFG_DATA_WR_DELAY_RESETVAL              (0x00000000U)
#define CSL_CBUFF_CFG_DELAY_CONFIG_CFG_DATA_WR_DELAY_MAX                   (0x000000FFU)

#define CSL_CBUFF_CFG_DELAY_CONFIG_NU_MASK                                 (0xFF000000U)
#define CSL_CBUFF_CFG_DELAY_CONFIG_NU_SHIFT                                (0x00000018U)
#define CSL_CBUFF_CFG_DELAY_CONFIG_NU_RESETVAL                             (0x00000000U)
#define CSL_CBUFF_CFG_DELAY_CONFIG_NU_MAX                                  (0x000000FFU)

#define CSL_CBUFF_CFG_DELAY_CONFIG_RESETVAL                                (0x00000000U)

/* CFG_DATA_LL */

#define CSL_CBUFF_CFG_DATA_LL_LL_VALID_MASK                                (0x00000001U)
#define CSL_CBUFF_CFG_DATA_LL_LL_VALID_SHIFT                               (0x00000000U)
#define CSL_CBUFF_CFG_DATA_LL_LL_VALID_RESETVAL                            (0x00000000U)
#define CSL_CBUFF_CFG_DATA_LL_LL_VALID_MAX                                 (0x00000001U)

#define CSL_CBUFF_CFG_DATA_LL_LL_HE_MASK                                   (0x00000002U)
#define CSL_CBUFF_CFG_DATA_LL_LL_HE_SHIFT                                  (0x00000001U)
#define CSL_CBUFF_CFG_DATA_LL_LL_HE_RESETVAL                               (0x00000000U)
#define CSL_CBUFF_CFG_DATA_LL_LL_HE_MAX                                    (0x00000001U)

#define CSL_CBUFF_CFG_DATA_LL_LL_HS_MASK                                   (0x00000004U)
#define CSL_CBUFF_CFG_DATA_LL_LL_HS_SHIFT                                  (0x00000002U)
#define CSL_CBUFF_CFG_DATA_LL_LL_HS_RESETVAL                               (0x00000000U)
#define CSL_CBUFF_CFG_DATA_LL_LL_HS_MAX                                    (0x00000001U)

#define CSL_CBUFF_CFG_DATA_LL_LL_VCNUM_MASK                                (0x00000018U)
#define CSL_CBUFF_CFG_DATA_LL_LL_VCNUM_SHIFT                               (0x00000003U)
#define CSL_CBUFF_CFG_DATA_LL_LL_VCNUM_RESETVAL                            (0x00000000U)
#define CSL_CBUFF_CFG_DATA_LL_LL_VCNUM_MAX                                 (0x00000003U)

#define CSL_CBUFF_CFG_DATA_LL_LL_FMT_MASK                                  (0x00000060U)
#define CSL_CBUFF_CFG_DATA_LL_LL_FMT_SHIFT                                 (0x00000005U)
#define CSL_CBUFF_CFG_DATA_LL_LL_FMT_RESETVAL                              (0x00000000U)
#define CSL_CBUFF_CFG_DATA_LL_LL_FMT_MAX                                   (0x00000003U)

#define CSL_CBUFF_CFG_DATA_LL_LL_FMT_MAP_MASK                              (0x00000080U)
#define CSL_CBUFF_CFG_DATA_LL_LL_FMT_MAP_SHIFT                             (0x00000007U)
#define CSL_CBUFF_CFG_DATA_LL_LL_FMT_MAP_RESETVAL                          (0x00000000U)
#define CSL_CBUFF_CFG_DATA_LL_LL_FMT_MAP_MAX                               (0x00000001U)

#define CSL_CBUFF_CFG_DATA_LL_LL_FMT_IN_MASK                               (0x00000100U)
#define CSL_CBUFF_CFG_DATA_LL_LL_FMT_IN_SHIFT                              (0x00000008U)
#define CSL_CBUFF_CFG_DATA_LL_LL_FMT_IN_RESETVAL                           (0x00000000U)
#define CSL_CBUFF_CFG_DATA_LL_LL_FMT_IN_MAX                                (0x00000001U)

#define CSL_CBUFF_CFG_DATA_LL_LL_SIZE_MASK                                 (0x007FFE00U)
#define CSL_CBUFF_CFG_DATA_LL_LL_SIZE_SHIFT                                (0x00000009U)
#define CSL_CBUFF_CFG_DATA_LL_LL_SIZE_RESETVAL                             (0x00000000U)
#define CSL_CBUFF_CFG_DATA_LL_LL_SIZE_MAX                                  (0x00003FFFU)

#define CSL_CBUFF_CFG_DATA_LL_LL_BITPOS_SEL_MASK                           (0x03800000U)
#define CSL_CBUFF_CFG_DATA_LL_LL_BITPOS_SEL_SHIFT                          (0x00000017U)
#define CSL_CBUFF_CFG_DATA_LL_LL_BITPOS_SEL_RESETVAL                       (0x00000000U)
#define CSL_CBUFF_CFG_DATA_LL_LL_BITPOS_SEL_MAX                            (0x00000007U)

#define CSL_CBUFF_CFG_DATA_LL_LL_WAITFOR_PKTSENT_MASK                      (0x04000000U)
#define CSL_CBUFF_CFG_DATA_LL_LL_WAITFOR_PKTSENT_SHIFT                     (0x0000001AU)
#define CSL_CBUFF_CFG_DATA_LL_LL_WAITFOR_PKTSENT_RESETVAL                  (0x00000000U)
#define CSL_CBUFF_CFG_DATA_LL_LL_WAITFOR_PKTSENT_MAX                       (0x00000001U)

#define CSL_CBUFF_CFG_DATA_LL_LL_LPHDR_EN_MASK                             (0x08000000U)
#define CSL_CBUFF_CFG_DATA_LL_LL_LPHDR_EN_SHIFT                            (0x0000001BU)
#define CSL_CBUFF_CFG_DATA_LL_LL_LPHDR_EN_RESETVAL                         (0x00000000U)
#define CSL_CBUFF_CFG_DATA_LL_LL_LPHDR_EN_MAX                              (0x00000001U)

#define CSL_CBUFF_CFG_DATA_LL_LL_CRC_EN_MASK                               (0x10000000U)
#define CSL_CBUFF_CFG_DATA_LL_LL_CRC_EN_SHIFT                              (0x0000001CU)
#define CSL_CBUFF_CFG_DATA_LL_LL_CRC_EN_RESETVAL                           (0x00000000U)
#define CSL_CBUFF_CFG_DATA_LL_LL_CRC_EN_MAX                                (0x00000001U)

#define CSL_CBUFF_CFG_DATA_LL_LL_SHORT_PKT_DELAY_EN_MASK                   (0x20000000U)
#define CSL_CBUFF_CFG_DATA_LL_LL_SHORT_PKT_DELAY_EN_SHIFT                  (0x0000001DU)
#define CSL_CBUFF_CFG_DATA_LL_LL_SHORT_PKT_DELAY_EN_RESETVAL               (0x00000000U)
#define CSL_CBUFF_CFG_DATA_LL_LL_SHORT_PKT_DELAY_EN_MAX                    (0x00000001U)

#define CSL_CBUFF_CFG_DATA_LL_LL_LONG_PKT_DELAY_EN_MASK                    (0x40000000U)
#define CSL_CBUFF_CFG_DATA_LL_LL_LONG_PKT_DELAY_EN_SHIFT                   (0x0000001EU)
#define CSL_CBUFF_CFG_DATA_LL_LL_LONG_PKT_DELAY_EN_RESETVAL                (0x00000000U)
#define CSL_CBUFF_CFG_DATA_LL_LL_LONG_PKT_DELAY_EN_MAX                     (0x00000001U)

#define CSL_CBUFF_CFG_DATA_LL_LL_DATA_WR_DELAY_EN_MASK                     (0x80000000U)
#define CSL_CBUFF_CFG_DATA_LL_LL_DATA_WR_DELAY_EN_SHIFT                    (0x0000001FU)
#define CSL_CBUFF_CFG_DATA_LL_LL_DATA_WR_DELAY_EN_RESETVAL                 (0x00000000U)
#define CSL_CBUFF_CFG_DATA_LL_LL_DATA_WR_DELAY_EN_MAX                      (0x00000001U)

#define CSL_CBUFF_CFG_DATA_LL_RESETVAL                                     (0x00000000U)

/* CFG_DATA_LL_LPHDR_VAL */

#define CSL_CBUFF_CFG_DATA_LL_LPHDR_VAL_LL_LPHDR_VAL_MASK                  (0xFFFFFFFFU)
#define CSL_CBUFF_CFG_DATA_LL_LPHDR_VAL_LL_LPHDR_VAL_SHIFT                 (0x00000000U)
#define CSL_CBUFF_CFG_DATA_LL_LPHDR_VAL_LL_LPHDR_VAL_RESETVAL              (0x00000000U)
#define CSL_CBUFF_CFG_DATA_LL_LPHDR_VAL_LL_LPHDR_VAL_MAX                   (0xFFFFFFFFU)

#define CSL_CBUFF_CFG_DATA_LL_LPHDR_VAL_RESETVAL                           (0x00000000U)

/* CFG_DATA_LL_THRESHOLD */

#define CSL_CBUFF_CFG_DATA_LL_THRESHOLD_LL_RD_THRESHOLD_MASK               (0x0000007FU)
#define CSL_CBUFF_CFG_DATA_LL_THRESHOLD_LL_RD_THRESHOLD_SHIFT              (0x00000000U)
#define CSL_CBUFF_CFG_DATA_LL_THRESHOLD_LL_RD_THRESHOLD_RESETVAL           (0x00000000U)
#define CSL_CBUFF_CFG_DATA_LL_THRESHOLD_LL_RD_THRESHOLD_MAX                (0x0000007FU)

#define CSL_CBUFF_CFG_DATA_LL_THRESHOLD_LL_WR_THRESHOLD_MASK               (0x00007F00U)
#define CSL_CBUFF_CFG_DATA_LL_THRESHOLD_LL_WR_THRESHOLD_SHIFT              (0x00000008U)
#define CSL_CBUFF_CFG_DATA_LL_THRESHOLD_LL_WR_THRESHOLD_RESETVAL           (0x0000003FU)
#define CSL_CBUFF_CFG_DATA_LL_THRESHOLD_LL_WR_THRESHOLD_MAX                (0x0000007FU)

#define CSL_CBUFF_CFG_DATA_LL_THRESHOLD_LLDMAN_MASK                        (0x00070000U)
#define CSL_CBUFF_CFG_DATA_LL_THRESHOLD_LLDMAN_SHIFT                       (0x00000010U)
#define CSL_CBUFF_CFG_DATA_LL_THRESHOLD_LLDMAN_RESETVAL                    (0x00000000U)
#define CSL_CBUFF_CFG_DATA_LL_THRESHOLD_LLDMAN_MAX                         (0x00000007U)

#define CSL_CBUFF_CFG_DATA_LL_THRESHOLD_RESETVAL                           (0x00003F00U)

/* CFG_LVDS_MAPPING_LANE_FMT */

#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_A_MASK                         (0x0000000FU)
#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_A_SHIFT                        (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_A_RESETVAL                     (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_A_MAX                          (0x0000000FU)

#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_B_MASK                         (0x000000F0U)
#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_B_SHIFT                        (0x00000004U)
#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_B_RESETVAL                     (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_B_MAX                          (0x0000000FU)

#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_C_MASK                         (0x00000F00U)
#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_C_SHIFT                        (0x00000008U)
#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_C_RESETVAL                     (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_C_MAX                          (0x0000000FU)

#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_D_MASK                         (0x0000F000U)
#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_D_SHIFT                        (0x0000000CU)
#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_D_RESETVAL                     (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_D_MAX                          (0x0000000FU)

#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_E_MASK                         (0x000F0000U)
#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_E_SHIFT                        (0x00000010U)
#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_E_RESETVAL                     (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_E_MAX                          (0x0000000FU)

#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_F_MASK                         (0x00F00000U)
#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_F_SHIFT                        (0x00000014U)
#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_F_RESETVAL                     (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_F_MAX                          (0x0000000FU)

#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_G_MASK                         (0x0F000000U)
#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_G_SHIFT                        (0x00000018U)
#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_G_RESETVAL                     (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_G_MAX                          (0x0000000FU)

#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_H_MASK                         (0xF0000000U)
#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_H_SHIFT                        (0x0000001CU)
#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_H_RESETVAL                     (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE_FMT_H_MAX                          (0x0000000FU)

#define CSL_CBUFF_CFG_LVDS_MAPPING_LANE0_FMT_0_RESETVAL                    (0x00000000U)

/* CFG_LVDS_GEN_0 */

#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_LVDS_LANE0_EN_MASK                    (0x00000001U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_LVDS_LANE0_EN_SHIFT                   (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_LVDS_LANE0_EN_RESETVAL                (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_LVDS_LANE0_EN_MAX                     (0x00000001U)

#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_LVDS_LANE1_EN_MASK                    (0x00000002U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_LVDS_LANE1_EN_SHIFT                   (0x00000001U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_LVDS_LANE1_EN_RESETVAL                (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_LVDS_LANE1_EN_MAX                     (0x00000001U)

#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_LVDS_LANE2_EN_MASK                    (0x00000004U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_LVDS_LANE2_EN_SHIFT                   (0x00000002U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_LVDS_LANE2_EN_RESETVAL                (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_LVDS_LANE2_EN_MAX                     (0x00000001U)

#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_LVDS_LANE3_EN_MASK                    (0x00000008U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_LVDS_LANE3_EN_SHIFT                   (0x00000003U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_LVDS_LANE3_EN_RESETVAL                (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_LVDS_LANE3_EN_MAX                     (0x00000001U)

#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_8B10B_EN_MASK                         (0x00000010U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_8B10B_EN_SHIFT                        (0x00000004U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_8B10B_EN_RESETVAL                     (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_8B10B_EN_MAX                          (0x00000001U)

#define CSL_CBUFF_CFG_LVDS_GEN_0_CTC2EN_MASK                               (0x00000020U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CTC2EN_SHIFT                              (0x00000005U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CTC2EN_RESETVAL                           (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CTC2EN_MAX                                (0x00000001U)

#define CSL_CBUFF_CFG_LVDS_GEN_0_CACDSEL_MASK                              (0x00000040U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CACDSEL_SHIFT                             (0x00000006U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CACDSEL_RESETVAL                          (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CACDSEL_MAX                               (0x00000001U)

#define CSL_CBUFF_CFG_LVDS_GEN_0_CPKFMT_MASK                               (0x00000080U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CPKFMT_SHIFT                              (0x00000007U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CPKFMT_RESETVAL                           (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CPKFMT_MAX                                (0x00000001U)

#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_LINE_MODE_MASK                        (0x00000300U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_LINE_MODE_SHIFT                       (0x00000008U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_LINE_MODE_RESETVAL                    (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_LINE_MODE_MAX                         (0x00000003U)

#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_BIT_CLK_MODE_MASK                     (0x00000400U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_BIT_CLK_MODE_SHIFT                    (0x0000000AU)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_BIT_CLK_MODE_RESETVAL                 (0x00000001U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CFG_BIT_CLK_MODE_MAX                      (0x00000001U)

#define CSL_CBUFF_CFG_LVDS_GEN_0_CCSMEN_MASK                               (0x00000800U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CCSMEN_SHIFT                              (0x0000000BU)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CCSMEN_RESETVAL                           (0x00000001U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CCSMEN_MAX                                (0x00000001U)

#define CSL_CBUFF_CFG_LVDS_GEN_0_CKCHAR_MASK                               (0x00003000U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CKCHAR_SHIFT                              (0x0000000CU)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CKCHAR_RESETVAL                           (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CKCHAR_MAX                                (0x00000003U)

#define CSL_CBUFF_CFG_LVDS_GEN_0_CCLKSEL_MASK                              (0x00004000U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CCLKSEL_SHIFT                             (0x0000000EU)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CCLKSEL_RESETVAL                          (0x00000001U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CCLKSEL_MAX                               (0x00000001U)

#define CSL_CBUFF_CFG_LVDS_GEN_0_CCLKSEL1_MASK                             (0x00008000U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CCLKSEL1_SHIFT                            (0x0000000FU)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CCLKSEL1_RESETVAL                         (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CCLKSEL1_MAX                              (0x00000001U)

#define CSL_CBUFF_CFG_LVDS_GEN_0_CCKDIV_MASK                               (0x003F0000U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CCKDIV_SHIFT                              (0x00000010U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CCKDIV_RESETVAL                           (0x00000002U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CCKDIV_MAX                                (0x0000003FU)

#define CSL_CBUFF_CFG_LVDS_GEN_0_CPOSSEL_MASK                              (0x00400000U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CPOSSEL_SHIFT                             (0x00000016U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CPOSSEL_RESETVAL                          (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CPOSSEL_MAX                               (0x00000001U)

#define CSL_CBUFF_CFG_LVDS_GEN_0_CMSBF_MASK                                (0x00800000U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CMSBF_SHIFT                               (0x00000017U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CMSBF_RESETVAL                            (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CMSBF_MAX                                 (0x00000001U)

#define CSL_CBUFF_CFG_LVDS_GEN_0_CFDLY_MASK                                (0x0F000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CFDLY_SHIFT                               (0x00000018U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CFDLY_RESETVAL                            (0x00000004U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CFDLY_MAX                                 (0x0000000FU)

#define CSL_CBUFF_CFG_LVDS_GEN_0_CBCRCEN_MASK                              (0x10000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CBCRCEN_SHIFT                             (0x0000001CU)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CBCRCEN_RESETVAL                          (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CBCRCEN_MAX                               (0x00000001U)

#define CSL_CBUFF_CFG_LVDS_GEN_0_CBLPEN_MASK                               (0x20000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CBLPEN_SHIFT                              (0x0000001DU)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CBLPEN_RESETVAL                           (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CBLPEN_MAX                                (0x00000001U)

#define CSL_CBUFF_CFG_LVDS_GEN_0_CPZ_MASK                                  (0xC0000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CPZ_SHIFT                                 (0x0000001EU)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CPZ_RESETVAL                              (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_0_CPZ_MAX                                   (0x00000003U)

#define CSL_CBUFF_CFG_LVDS_GEN_0_RESETVAL                                  (0x04024C00U)

/* CFG_LVDS_GEN_1 */

#define CSL_CBUFF_CFG_LVDS_GEN_1_CTPEN_MASK                                (0x00000001U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CTPEN_SHIFT                               (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CTPEN_RESETVAL                            (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CTPEN_MAX                                 (0x00000001U)

#define CSL_CBUFF_CFG_LVDS_GEN_1_CSDRINV_MASK                              (0x00000002U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CSDRINV_SHIFT                             (0x00000001U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CSDRINV_RESETVAL                          (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CSDRINV_MAX                               (0x00000001U)

#define CSL_CBUFF_CFG_LVDS_GEN_1_C3C3L_MASK                                (0x00000004U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_C3C3L_SHIFT                               (0x00000002U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_C3C3L_RESETVAL                            (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_C3C3L_MAX                                 (0x00000001U)

#define CSL_CBUFF_CFG_LVDS_GEN_1_NU3_MASK                                  (0x00000008U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_NU3_SHIFT                                 (0x00000003U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_NU3_RESETVAL                              (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_NU3_MAX                                   (0x00000001U)

#define CSL_CBUFF_CFG_LVDS_GEN_1_CTIDDLY_MASK                              (0x00000070U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CTIDDLY_SHIFT                             (0x00000004U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CTIDDLY_RESETVAL                          (0x00000005U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CTIDDLY_MAX                               (0x00000007U)

#define CSL_CBUFF_CFG_LVDS_GEN_1_NU1_MASK                                  (0x00000080U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_NU1_SHIFT                                 (0x00000007U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_NU1_RESETVAL                              (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_NU1_MAX                                   (0x00000001U)

#define CSL_CBUFF_CFG_LVDS_GEN_1_CTPSEL0_MASK                              (0x00000300U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CTPSEL0_SHIFT                             (0x00000008U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CTPSEL0_RESETVAL                          (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CTPSEL0_MAX                               (0x00000003U)

#define CSL_CBUFF_CFG_LVDS_GEN_1_CTPSEL1_MASK                              (0x00000C00U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CTPSEL1_SHIFT                             (0x0000000AU)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CTPSEL1_RESETVAL                          (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CTPSEL1_MAX                               (0x00000003U)

#define CSL_CBUFF_CFG_LVDS_GEN_1_CTPSEL2_MASK                              (0x00003000U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CTPSEL2_SHIFT                             (0x0000000CU)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CTPSEL2_RESETVAL                          (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CTPSEL2_MAX                               (0x00000003U)

#define CSL_CBUFF_CFG_LVDS_GEN_1_CTPSEL3_MASK                              (0x0000C000U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CTPSEL3_SHIFT                             (0x0000000EU)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CTPSEL3_RESETVAL                          (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CTPSEL3_MAX                               (0x00000003U)

#define CSL_CBUFF_CFG_LVDS_GEN_1_CLFVEN_MASK                               (0x00010000U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CLFVEN_SHIFT                              (0x00000010U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CLFVEN_RESETVAL                           (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CLFVEN_MAX                                (0x00000001U)

#define CSL_CBUFF_CFG_LVDS_GEN_1_CFCPOL_MASK                               (0x00020000U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CFCPOL_SHIFT                              (0x00000011U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CFCPOL_RESETVAL                           (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CFCPOL_MAX                                (0x00000001U)

#define CSL_CBUFF_CFG_LVDS_GEN_1_CGBCEN_MASK                               (0x00040000U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CGBCEN_SHIFT                              (0x00000012U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CGBCEN_RESETVAL                           (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_CGBCEN_MAX                                (0x00000001U)

#define CSL_CBUFF_CFG_LVDS_GEN_1_NU2_MASK                                  (0xFFF80000U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_NU2_SHIFT                                 (0x00000013U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_NU2_RESETVAL                              (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_1_NU2_MAX                                   (0x00001FFFU)

#define CSL_CBUFF_CFG_LVDS_GEN_1_RESETVAL                                  (0x00000050U)

/* CFG_LVDS_GEN_2 */

#define CSL_CBUFF_CFG_LVDS_GEN_2_CFG_LVDS_GEN_2_MASK                       (0xFFFFFFFFU)
#define CSL_CBUFF_CFG_LVDS_GEN_2_CFG_LVDS_GEN_2_SHIFT                      (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_2_CFG_LVDS_GEN_2_RESETVAL                   (0x00000000U)
#define CSL_CBUFF_CFG_LVDS_GEN_2_CFG_LVDS_GEN_2_MAX                        (0xFFFFFFFFU)

#define CSL_CBUFF_CFG_LVDS_GEN_2_RESETVAL                                  (0x00000000U)

/* CFG_MASK_REG0 */

#define CSL_CBUFF_CFG_MASK_REG0_CFG_MASK_REG0_MASK                         (0xFFFFFFFFU)
#define CSL_CBUFF_CFG_MASK_REG0_CFG_MASK_REG0_SHIFT                        (0x00000000U)
#define CSL_CBUFF_CFG_MASK_REG0_CFG_MASK_REG0_RESETVAL                     (0xFFFFFFFFU)
#define CSL_CBUFF_CFG_MASK_REG0_CFG_MASK_REG0_MAX                          (0xFFFFFFFFU)

#define CSL_CBUFF_CFG_MASK_REG0_RESETVAL                                   (0xFFFFFFFFU)

/* CFG_MASK_REG1 */

#define CSL_CBUFF_CFG_MASK_REG1_CFG_MASK_REG1_MASK                         (0xFFFFFFFFU)
#define CSL_CBUFF_CFG_MASK_REG1_CFG_MASK_REG1_SHIFT                        (0x00000000U)
#define CSL_CBUFF_CFG_MASK_REG1_CFG_MASK_REG1_RESETVAL                     (0xFFFFFFFFU)
#define CSL_CBUFF_CFG_MASK_REG1_CFG_MASK_REG1_MAX                          (0xFFFFFFFFU)

#define CSL_CBUFF_CFG_MASK_REG1_RESETVAL                                   (0xFFFFFFFFU)

/* CFG_MASK_REG2 */

#define CSL_CBUFF_CFG_MASK_REG2_CFG_MASK_REG2_MASK                         (0xFFFFFFFFU)
#define CSL_CBUFF_CFG_MASK_REG2_CFG_MASK_REG2_SHIFT                        (0x00000000U)
#define CSL_CBUFF_CFG_MASK_REG2_CFG_MASK_REG2_RESETVAL                     (0xFFFFFFFFU)
#define CSL_CBUFF_CFG_MASK_REG2_CFG_MASK_REG2_MAX                          (0xFFFFFFFFU)

#define CSL_CBUFF_CFG_MASK_REG2_RESETVAL                                   (0xFFFFFFFFU)

/* CFG_MASK_REG3 */

#define CSL_CBUFF_CFG_MASK_REG3_CFG_MASK_REG3_MASK                         (0xFFFFFFFFU)
#define CSL_CBUFF_CFG_MASK_REG3_CFG_MASK_REG3_SHIFT                        (0x00000000U)
#define CSL_CBUFF_CFG_MASK_REG3_CFG_MASK_REG3_RESETVAL                     (0xFFFFFFFFU)
#define CSL_CBUFF_CFG_MASK_REG3_CFG_MASK_REG3_MAX                          (0xFFFFFFFFU)

#define CSL_CBUFF_CFG_MASK_REG3_RESETVAL                                   (0xFFFFFFFFU)

/* STAT_CBUFF_REG0 */

#define CSL_CBUFF_STAT_CBUFF_REG0_S_CSI_PKT_RCVD_MASK                      (0x00000001U)
#define CSL_CBUFF_STAT_CBUFF_REG0_S_CSI_PKT_RCVD_SHIFT                     (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG0_S_CSI_PKT_RCVD_RESETVAL                  (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG0_S_CSI_PKT_RCVD_MAX                       (0x00000001U)

#define CSL_CBUFF_STAT_CBUFF_REG0_S_CSI_PKT_VS_RCVD_STATE_MASK             (0x00000002U)
#define CSL_CBUFF_STAT_CBUFF_REG0_S_CSI_PKT_VS_RCVD_STATE_SHIFT            (0x00000001U)
#define CSL_CBUFF_STAT_CBUFF_REG0_S_CSI_PKT_VS_RCVD_STATE_RESETVAL         (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG0_S_CSI_PKT_VS_RCVD_STATE_MAX              (0x00000001U)

#define CSL_CBUFF_STAT_CBUFF_REG0_S_CSI_PKT_VE_RCVD_STATE_MASK             (0x00000004U)
#define CSL_CBUFF_STAT_CBUFF_REG0_S_CSI_PKT_VE_RCVD_STATE_SHIFT            (0x00000002U)
#define CSL_CBUFF_STAT_CBUFF_REG0_S_CSI_PKT_VE_RCVD_STATE_RESETVAL         (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG0_S_CSI_PKT_VE_RCVD_STATE_MAX              (0x00000001U)

#define CSL_CBUFF_STAT_CBUFF_REG0_S_CSI_PKT_HS_RCVD_STATE_MASK             (0x00000008U)
#define CSL_CBUFF_STAT_CBUFF_REG0_S_CSI_PKT_HS_RCVD_STATE_SHIFT            (0x00000003U)
#define CSL_CBUFF_STAT_CBUFF_REG0_S_CSI_PKT_HS_RCVD_STATE_RESETVAL         (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG0_S_CSI_PKT_HS_RCVD_STATE_MAX              (0x00000001U)

#define CSL_CBUFF_STAT_CBUFF_REG0_S_CSI_PKT_HE_RCVD_STATE_MASK             (0x00000010U)
#define CSL_CBUFF_STAT_CBUFF_REG0_S_CSI_PKT_HE_RCVD_STATE_SHIFT            (0x00000004U)
#define CSL_CBUFF_STAT_CBUFF_REG0_S_CSI_PKT_HE_RCVD_STATE_RESETVAL         (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG0_S_CSI_PKT_HE_RCVD_STATE_MAX              (0x00000001U)

#define CSL_CBUFF_STAT_CBUFF_REG0_S_CSI_PKT_LP_RCVD_STATE_MASK             (0x00000020U)
#define CSL_CBUFF_STAT_CBUFF_REG0_S_CSI_PKT_LP_RCVD_STATE_SHIFT            (0x00000005U)
#define CSL_CBUFF_STAT_CBUFF_REG0_S_CSI_PKT_LP_RCVD_STATE_RESETVAL         (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG0_S_CSI_PKT_LP_RCVD_STATE_MAX              (0x00000001U)

#define CSL_CBUFF_STAT_CBUFF_REG0_S_LL_INDEX_MASK                          (0x000007C0U)
#define CSL_CBUFF_STAT_CBUFF_REG0_S_LL_INDEX_SHIFT                         (0x00000006U)
#define CSL_CBUFF_STAT_CBUFF_REG0_S_LL_INDEX_RESETVAL                      (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG0_S_LL_INDEX_MAX                           (0x0000001FU)

#define CSL_CBUFF_STAT_CBUFF_REG0_S_CHIRP_DONE_MASK                        (0x00000800U)
#define CSL_CBUFF_STAT_CBUFF_REG0_S_CHIRP_DONE_SHIFT                       (0x0000000BU)
#define CSL_CBUFF_STAT_CBUFF_REG0_S_CHIRP_DONE_RESETVAL                    (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG0_S_CHIRP_DONE_MAX                         (0x00000001U)

#define CSL_CBUFF_STAT_CBUFF_REG0_S_FRAME_DONE_MASK                        (0x00001000U)
#define CSL_CBUFF_STAT_CBUFF_REG0_S_FRAME_DONE_SHIFT                       (0x0000000CU)
#define CSL_CBUFF_STAT_CBUFF_REG0_S_FRAME_DONE_RESETVAL                    (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG0_S_FRAME_DONE_MAX                         (0x00000001U)

#define CSL_CBUFF_STAT_CBUFF_REG0_STAT_CBUFF_REG0_OTHERS_MASK              (0xFFFFE000U)
#define CSL_CBUFF_STAT_CBUFF_REG0_STAT_CBUFF_REG0_OTHERS_SHIFT             (0x0000000DU)
#define CSL_CBUFF_STAT_CBUFF_REG0_STAT_CBUFF_REG0_OTHERS_RESETVAL          (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG0_STAT_CBUFF_REG0_OTHERS_MAX               (0x0007FFFFU)

#define CSL_CBUFF_STAT_CBUFF_REG0_RESETVAL                                 (0x00000000U)

/* STAT_CBUFF_REG1 */

#define CSL_CBUFF_STAT_CBUFF_REG1_S_LCLFSM_ERR_MASK                        (0x00000001U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_LCLFSM_ERR_SHIFT                       (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_LCLFSM_ERR_RESETVAL                    (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_LCLFSM_ERR_MAX                         (0x00000001U)

#define CSL_CBUFF_STAT_CBUFF_REG1_S_LCLPOP_ERR_MASK                        (0x00000002U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_LCLPOP_ERR_SHIFT                       (0x00000001U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_LCLPOP_ERR_RESETVAL                    (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_LCLPOP_ERR_MAX                         (0x00000001U)

#define CSL_CBUFF_STAT_CBUFF_REG1_S_LCLPUSH_ERR_MASK                       (0x00000004U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_LCLPUSH_ERR_SHIFT                      (0x00000002U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_LCLPUSH_ERR_RESETVAL                   (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_LCLPUSH_ERR_MAX                        (0x00000001U)

#define CSL_CBUFF_STAT_CBUFF_REG1_S1_UNUSED1_MASK                          (0x000000F8U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S1_UNUSED1_SHIFT                         (0x00000003U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S1_UNUSED1_RESETVAL                      (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S1_UNUSED1_MAX                           (0x0000001FU)

#define CSL_CBUFF_STAT_CBUFF_REG1_S_CBPOP_ERR_MASK                         (0x00000100U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_CBPOP_ERR_SHIFT                        (0x00000008U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_CBPOP_ERR_RESETVAL                     (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_CBPOP_ERR_MAX                          (0x00000001U)

#define CSL_CBUFF_STAT_CBUFF_REG1_S_CBPUSH_ERR_MASK                        (0x00000200U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_CBPUSH_ERR_SHIFT                       (0x00000009U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_CBPUSH_ERR_RESETVAL                    (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_CBPUSH_ERR_MAX                         (0x00000001U)

#define CSL_CBUFF_STAT_CBUFF_REG1_S_CBFIFO_FULL_MASK                       (0x00000400U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_CBFIFO_FULL_SHIFT                      (0x0000000AU)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_CBFIFO_FULL_RESETVAL                   (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_CBFIFO_FULL_MAX                        (0x00000001U)

#define CSL_CBUFF_STAT_CBUFF_REG1_S_CBFIFO_EMPTY_MASK                      (0x00000800U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_CBFIFO_EMPTY_SHIFT                     (0x0000000BU)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_CBFIFO_EMPTY_RESETVAL                  (0x00000001U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_CBFIFO_EMPTY_MAX                       (0x00000001U)

#define CSL_CBUFF_STAT_CBUFF_REG1_S1_UNUSED2_MASK                          (0x0000F000U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S1_UNUSED2_SHIFT                         (0x0000000CU)
#define CSL_CBUFF_STAT_CBUFF_REG1_S1_UNUSED2_RESETVAL                      (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S1_UNUSED2_MAX                           (0x0000000FU)

#define CSL_CBUFF_STAT_CBUFF_REG1_S_CHIRP_ERR_MASK                         (0x00010000U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_CHIRP_ERR_SHIFT                        (0x00000010U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_CHIRP_ERR_RESETVAL                     (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_CHIRP_ERR_MAX                          (0x00000001U)

#define CSL_CBUFF_STAT_CBUFF_REG1_S_FRAME_ERR_MASK                         (0x00020000U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_FRAME_ERR_SHIFT                        (0x00000011U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_FRAME_ERR_RESETVAL                     (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_FRAME_ERR_MAX                          (0x00000001U)

#define CSL_CBUFF_STAT_CBUFF_REG1_S_PKTRCV_ERR_MASK                        (0x00040000U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_PKTRCV_ERR_SHIFT                       (0x00000012U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_PKTRCV_ERR_RESETVAL                    (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_PKTRCV_ERR_MAX                         (0x00000001U)

#define CSL_CBUFF_STAT_CBUFF_REG1_S_CBFIFO_EMPTY_IN_FSM_MASK               (0x00080000U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_CBFIFO_EMPTY_IN_FSM_SHIFT              (0x00000013U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_CBFIFO_EMPTY_IN_FSM_RESETVAL           (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_CBFIFO_EMPTY_IN_FSM_MAX                (0x00000001U)

#define CSL_CBUFF_STAT_CBUFF_REG1_S_CBFIFO_READY_IN_FSM_MASK               (0x00100000U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_CBFIFO_READY_IN_FSM_SHIFT              (0x00000014U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_CBFIFO_READY_IN_FSM_RESETVAL           (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S_CBFIFO_READY_IN_FSM_MAX                (0x00000001U)

#define CSL_CBUFF_STAT_CBUFF_REG1_S1_UNUSED3_MASK                          (0xFFE00000U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S1_UNUSED3_SHIFT                         (0x00000015U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S1_UNUSED3_RESETVAL                      (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG1_S1_UNUSED3_MAX                           (0x000007FFU)

#define CSL_CBUFF_STAT_CBUFF_REG1_RESETVAL                                 (0x00000800U)

/* STAT_CBUFF_REG2 */

#define CSL_CBUFF_STAT_CBUFF_REG2_STAT_CBUFF_REG2_MASK                     (0xFFFFFFFFU)
#define CSL_CBUFF_STAT_CBUFF_REG2_STAT_CBUFF_REG2_SHIFT                    (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG2_STAT_CBUFF_REG2_RESETVAL                 (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG2_STAT_CBUFF_REG2_MAX                      (0xFFFFFFFFU)

#define CSL_CBUFF_STAT_CBUFF_REG2_RESETVAL                                 (0x00000000U)

/* STAT_CBUFF_REG3 */

#define CSL_CBUFF_STAT_CBUFF_REG3_STAT_CBUFF_REG3_MASK                     (0xFFFFFFFFU)
#define CSL_CBUFF_STAT_CBUFF_REG3_STAT_CBUFF_REG3_SHIFT                    (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG3_STAT_CBUFF_REG3_RESETVAL                 (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_REG3_STAT_CBUFF_REG3_MAX                      (0xFFFFFFFFU)

#define CSL_CBUFF_STAT_CBUFF_REG3_RESETVAL                                 (0x00000000U)

/* STAT_LVDS_REG0 */

#define CSL_CBUFF_STAT_LVDS_REG0_STAT_LVDS_REG0_MASK                       (0xFFFFFFFFU)
#define CSL_CBUFF_STAT_LVDS_REG0_STAT_LVDS_REG0_SHIFT                      (0x00000000U)
#define CSL_CBUFF_STAT_LVDS_REG0_STAT_LVDS_REG0_RESETVAL                   (0x44446666U)
#define CSL_CBUFF_STAT_LVDS_REG0_STAT_LVDS_REG0_MAX                        (0xFFFFFFFFU)

#define CSL_CBUFF_STAT_LVDS_REG0_RESETVAL                                  (0x44446666U)

/* STAT_LVDS_REG1 */

#define CSL_CBUFF_STAT_LVDS_REG1_STAT_LVDS_REG1_MASK                       (0xFFFFFFFFU)
#define CSL_CBUFF_STAT_LVDS_REG1_STAT_LVDS_REG1_SHIFT                      (0x00000000U)
#define CSL_CBUFF_STAT_LVDS_REG1_STAT_LVDS_REG1_RESETVAL                   (0x00000000U)
#define CSL_CBUFF_STAT_LVDS_REG1_STAT_LVDS_REG1_MAX                        (0xFFFFFFFFU)

#define CSL_CBUFF_STAT_LVDS_REG1_RESETVAL                                  (0x00000000U)

/* STAT_LVDS_REG2 */

#define CSL_CBUFF_STAT_LVDS_REG2_STAT_LVDS_REG2_MASK                       (0xFFFFFFFFU)
#define CSL_CBUFF_STAT_LVDS_REG2_STAT_LVDS_REG2_SHIFT                      (0x00000000U)
#define CSL_CBUFF_STAT_LVDS_REG2_STAT_LVDS_REG2_RESETVAL                   (0x00000000U)
#define CSL_CBUFF_STAT_LVDS_REG2_STAT_LVDS_REG2_MAX                        (0xFFFFFFFFU)

#define CSL_CBUFF_STAT_LVDS_REG2_RESETVAL                                  (0x00000000U)

/* STAT_LVDS_REG3 */

#define CSL_CBUFF_STAT_LVDS_REG3_STAT_LVDS_REG3_MASK                       (0xFFFFFFFFU)
#define CSL_CBUFF_STAT_LVDS_REG3_STAT_LVDS_REG3_SHIFT                      (0x00000000U)
#define CSL_CBUFF_STAT_LVDS_REG3_STAT_LVDS_REG3_RESETVAL                   (0x00000000U)
#define CSL_CBUFF_STAT_LVDS_REG3_STAT_LVDS_REG3_MAX                        (0xFFFFFFFFU)

#define CSL_CBUFF_STAT_LVDS_REG3_RESETVAL                                  (0x00000000U)

/* CLR_CBUFF_REG0 */

#define CSL_CBUFF_CLR_CBUFF_REG0_C_CSI_PKT_RCVD_MASK                       (0x00000001U)
#define CSL_CBUFF_CLR_CBUFF_REG0_C_CSI_PKT_RCVD_SHIFT                      (0x00000000U)
#define CSL_CBUFF_CLR_CBUFF_REG0_C_CSI_PKT_RCVD_RESETVAL                   (0x00000000U)
#define CSL_CBUFF_CLR_CBUFF_REG0_C_CSI_PKT_RCVD_MAX                        (0x00000001U)

#define CSL_CBUFF_CLR_CBUFF_REG0_C_CSI_PKT_VS_RCVD_STATE_MASK              (0x00000002U)
#define CSL_CBUFF_CLR_CBUFF_REG0_C_CSI_PKT_VS_RCVD_STATE_SHIFT             (0x00000001U)
#define CSL_CBUFF_CLR_CBUFF_REG0_C_CSI_PKT_VS_RCVD_STATE_RESETVAL          (0x00000000U)
#define CSL_CBUFF_CLR_CBUFF_REG0_C_CSI_PKT_VS_RCVD_STATE_MAX               (0x00000001U)

#define CSL_CBUFF_CLR_CBUFF_REG0_C_CSI_PKT_VE_RCVD_STATE_MASK              (0x00000004U)
#define CSL_CBUFF_CLR_CBUFF_REG0_C_CSI_PKT_VE_RCVD_STATE_SHIFT             (0x00000002U)
#define CSL_CBUFF_CLR_CBUFF_REG0_C_CSI_PKT_VE_RCVD_STATE_RESETVAL          (0x00000000U)
#define CSL_CBUFF_CLR_CBUFF_REG0_C_CSI_PKT_VE_RCVD_STATE_MAX               (0x00000001U)

#define CSL_CBUFF_CLR_CBUFF_REG0_C_CSI_PKT_HS_RCVD_STATE_MASK              (0x00000008U)
#define CSL_CBUFF_CLR_CBUFF_REG0_C_CSI_PKT_HS_RCVD_STATE_SHIFT             (0x00000003U)
#define CSL_CBUFF_CLR_CBUFF_REG0_C_CSI_PKT_HS_RCVD_STATE_RESETVAL          (0x00000000U)
#define CSL_CBUFF_CLR_CBUFF_REG0_C_CSI_PKT_HS_RCVD_STATE_MAX               (0x00000001U)

#define CSL_CBUFF_CLR_CBUFF_REG0_C_CSI_PKT_HE_RCVD_STATE_MASK              (0x00000010U)
#define CSL_CBUFF_CLR_CBUFF_REG0_C_CSI_PKT_HE_RCVD_STATE_SHIFT             (0x00000004U)
#define CSL_CBUFF_CLR_CBUFF_REG0_C_CSI_PKT_HE_RCVD_STATE_RESETVAL          (0x00000000U)
#define CSL_CBUFF_CLR_CBUFF_REG0_C_CSI_PKT_HE_RCVD_STATE_MAX               (0x00000001U)

#define CSL_CBUFF_CLR_CBUFF_REG0_C_CSI_PKT_LP_RCVD_STATE_MASK              (0x00000020U)
#define CSL_CBUFF_CLR_CBUFF_REG0_C_CSI_PKT_LP_RCVD_STATE_SHIFT             (0x00000005U)
#define CSL_CBUFF_CLR_CBUFF_REG0_C_CSI_PKT_LP_RCVD_STATE_RESETVAL          (0x00000000U)
#define CSL_CBUFF_CLR_CBUFF_REG0_C_CSI_PKT_LP_RCVD_STATE_MAX               (0x00000001U)

#define CSL_CBUFF_CLR_CBUFF_REG0_C_LL_INDEX_MASK                           (0x000007C0U)
#define CSL_CBUFF_CLR_CBUFF_REG0_C_LL_INDEX_SHIFT                          (0x00000006U)
#define CSL_CBUFF_CLR_CBUFF_REG0_C_LL_INDEX_RESETVAL                       (0x00000000U)
#define CSL_CBUFF_CLR_CBUFF_REG0_C_LL_INDEX_MAX                            (0x0000001FU)

#define CSL_CBUFF_CLR_CBUFF_REG0_C_CHIRP_DONE_MASK                         (0x00000800U)
#define CSL_CBUFF_CLR_CBUFF_REG0_C_CHIRP_DONE_SHIFT                        (0x0000000BU)
#define CSL_CBUFF_CLR_CBUFF_REG0_C_CHIRP_DONE_RESETVAL                     (0x00000000U)
#define CSL_CBUFF_CLR_CBUFF_REG0_C_CHIRP_DONE_MAX                          (0x00000001U)

#define CSL_CBUFF_CLR_CBUFF_REG0_C_FRAME_DONE_MASK                         (0x00001000U)
#define CSL_CBUFF_CLR_CBUFF_REG0_C_FRAME_DONE_SHIFT                        (0x0000000CU)
#define CSL_CBUFF_CLR_CBUFF_REG0_C_FRAME_DONE_RESETVAL                     (0x00000000U)
#define CSL_CBUFF_CLR_CBUFF_REG0_C_FRAME_DONE_MAX                          (0x00000001U)

#define CSL_CBUFF_CLR_CBUFF_REG0_CLR_CBUFF_REG0_OTHERS_MASK                (0xFFFFE000U)
#define CSL_CBUFF_CLR_CBUFF_REG0_CLR_CBUFF_REG0_OTHERS_SHIFT               (0x0000000DU)
#define CSL_CBUFF_CLR_CBUFF_REG0_CLR_CBUFF_REG0_OTHERS_RESETVAL            (0x00000000U)
#define CSL_CBUFF_CLR_CBUFF_REG0_CLR_CBUFF_REG0_OTHERS_MAX                 (0x0007FFFFU)

#define CSL_CBUFF_CLR_CBUFF_REG0_RESETVAL                                  (0x00000000U)

/* CLR_CBUFF_REG1 */

#define CSL_CBUFF_CLR_CBUFF_REG1_CLR_CBUFF_REG1_MASK                       (0xFFFFFFFFU)
#define CSL_CBUFF_CLR_CBUFF_REG1_CLR_CBUFF_REG1_SHIFT                      (0x00000000U)
#define CSL_CBUFF_CLR_CBUFF_REG1_CLR_CBUFF_REG1_RESETVAL                   (0x00000000U)
#define CSL_CBUFF_CLR_CBUFF_REG1_CLR_CBUFF_REG1_MAX                        (0xFFFFFFFFU)

#define CSL_CBUFF_CLR_CBUFF_REG1_RESETVAL                                  (0x00000000U)

/* CLR_LVDS_REG0 */

#define CSL_CBUFF_CLR_LVDS_REG0_CLR_LVDS_REG0_MASK                         (0xFFFFFFFFU)
#define CSL_CBUFF_CLR_LVDS_REG0_CLR_LVDS_REG0_SHIFT                        (0x00000000U)
#define CSL_CBUFF_CLR_LVDS_REG0_CLR_LVDS_REG0_RESETVAL                     (0x00000000U)
#define CSL_CBUFF_CLR_LVDS_REG0_CLR_LVDS_REG0_MAX                          (0xFFFFFFFFU)

#define CSL_CBUFF_CLR_LVDS_REG0_RESETVAL                                   (0x00000000U)

/* CLR_LVDS_REG1 */

#define CSL_CBUFF_CLR_LVDS_REG1_CLR_LVDS_REG1_MASK                         (0xFFFFFFFFU)
#define CSL_CBUFF_CLR_LVDS_REG1_CLR_LVDS_REG1_SHIFT                        (0x00000000U)
#define CSL_CBUFF_CLR_LVDS_REG1_CLR_LVDS_REG1_RESETVAL                     (0x00000000U)
#define CSL_CBUFF_CLR_LVDS_REG1_CLR_LVDS_REG1_MAX                          (0xFFFFFFFFU)

#define CSL_CBUFF_CLR_LVDS_REG1_RESETVAL                                   (0x00000000U)

/* STAT_CBUFF_ECC_REG */

#define CSL_CBUFF_STAT_CBUFF_ECC_REG_SECCADD_MASK                          (0x0000003FU)
#define CSL_CBUFF_STAT_CBUFF_ECC_REG_SECCADD_SHIFT                         (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_ECC_REG_SECCADD_RESETVAL                      (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_ECC_REG_SECCADD_MAX                           (0x0000003FU)

#define CSL_CBUFF_STAT_CBUFF_ECC_REG_NU1_MASK                              (0x000000C0U)
#define CSL_CBUFF_STAT_CBUFF_ECC_REG_NU1_SHIFT                             (0x00000006U)
#define CSL_CBUFF_STAT_CBUFF_ECC_REG_NU1_RESETVAL                          (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_ECC_REG_NU1_MAX                               (0x00000003U)

#define CSL_CBUFF_STAT_CBUFF_ECC_REG_SECCSBE_MASK                          (0x00000100U)
#define CSL_CBUFF_STAT_CBUFF_ECC_REG_SECCSBE_SHIFT                         (0x00000008U)
#define CSL_CBUFF_STAT_CBUFF_ECC_REG_SECCSBE_RESETVAL                      (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_ECC_REG_SECCSBE_MAX                           (0x00000001U)

#define CSL_CBUFF_STAT_CBUFF_ECC_REG_SECCDBE_MASK                          (0x00000200U)
#define CSL_CBUFF_STAT_CBUFF_ECC_REG_SECCDBE_SHIFT                         (0x00000009U)
#define CSL_CBUFF_STAT_CBUFF_ECC_REG_SECCDBE_RESETVAL                      (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_ECC_REG_SECCDBE_MAX                           (0x00000001U)

#define CSL_CBUFF_STAT_CBUFF_ECC_REG_NU2_MASK                              (0xFFFFFC00U)
#define CSL_CBUFF_STAT_CBUFF_ECC_REG_NU2_SHIFT                             (0x0000000AU)
#define CSL_CBUFF_STAT_CBUFF_ECC_REG_NU2_RESETVAL                          (0x00000000U)
#define CSL_CBUFF_STAT_CBUFF_ECC_REG_NU2_MAX                               (0x003FFFFFU)

#define CSL_CBUFF_STAT_CBUFF_ECC_REG_RESETVAL                              (0x00000000U)

/* MASK_CBUFF_ECC_REG */

#define CSL_CBUFF_MASK_CBUFF_ECC_REG_NU1_MASK                              (0x000000FFU)
#define CSL_CBUFF_MASK_CBUFF_ECC_REG_NU1_SHIFT                             (0x00000000U)
#define CSL_CBUFF_MASK_CBUFF_ECC_REG_NU1_RESETVAL                          (0x00000000U)
#define CSL_CBUFF_MASK_CBUFF_ECC_REG_NU1_MAX                               (0x000000FFU)

#define CSL_CBUFF_MASK_CBUFF_ECC_REG_MECCSBE_MASK                          (0x00000100U)
#define CSL_CBUFF_MASK_CBUFF_ECC_REG_MECCSBE_SHIFT                         (0x00000008U)
#define CSL_CBUFF_MASK_CBUFF_ECC_REG_MECCSBE_RESETVAL                      (0x00000001U)
#define CSL_CBUFF_MASK_CBUFF_ECC_REG_MECCSBE_MAX                           (0x00000001U)

#define CSL_CBUFF_MASK_CBUFF_ECC_REG_MECCDBE_MASK                          (0x00000200U)
#define CSL_CBUFF_MASK_CBUFF_ECC_REG_MECCDBE_SHIFT                         (0x00000009U)
#define CSL_CBUFF_MASK_CBUFF_ECC_REG_MECCDBE_RESETVAL                      (0x00000001U)
#define CSL_CBUFF_MASK_CBUFF_ECC_REG_MECCDBE_MAX                           (0x00000001U)

#define CSL_CBUFF_MASK_CBUFF_ECC_REG_NU2_MASK                              (0xFFFFFC00U)
#define CSL_CBUFF_MASK_CBUFF_ECC_REG_NU2_SHIFT                             (0x0000000AU)
#define CSL_CBUFF_MASK_CBUFF_ECC_REG_NU2_RESETVAL                          (0x00000000U)
#define CSL_CBUFF_MASK_CBUFF_ECC_REG_NU2_MAX                               (0x003FFFFFU)

#define CSL_CBUFF_MASK_CBUFF_ECC_REG_RESETVAL                              (0x00000300U)

/* CLR_CBUFF_ECC_REG */

#define CSL_CBUFF_CLR_CBUFF_ECC_REG_CECCADD_MASK                           (0x00000001U)
#define CSL_CBUFF_CLR_CBUFF_ECC_REG_CECCADD_SHIFT                          (0x00000000U)
#define CSL_CBUFF_CLR_CBUFF_ECC_REG_CECCADD_RESETVAL                       (0x00000000U)
#define CSL_CBUFF_CLR_CBUFF_ECC_REG_CECCADD_MAX                            (0x00000001U)

#define CSL_CBUFF_CLR_CBUFF_ECC_REG_NU1_MASK                               (0x000000FEU)
#define CSL_CBUFF_CLR_CBUFF_ECC_REG_NU1_SHIFT                              (0x00000001U)
#define CSL_CBUFF_CLR_CBUFF_ECC_REG_NU1_RESETVAL                           (0x00000000U)
#define CSL_CBUFF_CLR_CBUFF_ECC_REG_NU1_MAX                                (0x0000007FU)

#define CSL_CBUFF_CLR_CBUFF_ECC_REG_CECCSBE_MASK                           (0x00000100U)
#define CSL_CBUFF_CLR_CBUFF_ECC_REG_CECCSBE_SHIFT                          (0x00000008U)
#define CSL_CBUFF_CLR_CBUFF_ECC_REG_CECCSBE_RESETVAL                       (0x00000000U)
#define CSL_CBUFF_CLR_CBUFF_ECC_REG_CECCSBE_MAX                            (0x00000001U)

#define CSL_CBUFF_CLR_CBUFF_ECC_REG_CECCDBE_MASK                           (0x00000200U)
#define CSL_CBUFF_CLR_CBUFF_ECC_REG_CECCDBE_SHIFT                          (0x00000009U)
#define CSL_CBUFF_CLR_CBUFF_ECC_REG_CECCDBE_RESETVAL                       (0x00000000U)
#define CSL_CBUFF_CLR_CBUFF_ECC_REG_CECCDBE_MAX                            (0x00000001U)

#define CSL_CBUFF_CLR_CBUFF_ECC_REG_NU2_MASK                               (0xFFFFFC00U)
#define CSL_CBUFF_CLR_CBUFF_ECC_REG_NU2_SHIFT                              (0x0000000AU)
#define CSL_CBUFF_CLR_CBUFF_ECC_REG_NU2_RESETVAL                           (0x00000000U)
#define CSL_CBUFF_CLR_CBUFF_ECC_REG_NU2_MAX                                (0x003FFFFFU)

#define CSL_CBUFF_CLR_CBUFF_ECC_REG_RESETVAL                               (0x00000000U)

/* STAT_SAFETY */

#define CSL_CBUFF_STAT_SAFETY_SAF_CRC_MASK                                 (0x000000FFU)
#define CSL_CBUFF_STAT_SAFETY_SAF_CRC_SHIFT                                (0x00000000U)
#define CSL_CBUFF_STAT_SAFETY_SAF_CRC_RESETVAL                             (0x00000000U)
#define CSL_CBUFF_STAT_SAFETY_SAF_CRC_MAX                                  (0x000000FFU)

#define CSL_CBUFF_STAT_SAFETY_SAF_CHIRP_ERR_MASK                           (0x00000100U)
#define CSL_CBUFF_STAT_SAFETY_SAF_CHIRP_ERR_SHIFT                          (0x00000008U)
#define CSL_CBUFF_STAT_SAFETY_SAF_CHIRP_ERR_RESETVAL                       (0x00000000U)
#define CSL_CBUFF_STAT_SAFETY_SAF_CHIRP_ERR_MAX                            (0x00000001U)

#define CSL_CBUFF_STAT_SAFETY_SAF_UNUSED1_MASK                             (0xFFFFFE00U)
#define CSL_CBUFF_STAT_SAFETY_SAF_UNUSED1_SHIFT                            (0x00000009U)
#define CSL_CBUFF_STAT_SAFETY_SAF_UNUSED1_RESETVAL                         (0x00000000U)
#define CSL_CBUFF_STAT_SAFETY_SAF_UNUSED1_MAX                              (0x007FFFFFU)

#define CSL_CBUFF_STAT_SAFETY_RESETVAL                                     (0x00000000U)

/* MASK_SAFETY */

#define CSL_CBUFF_MASK_SAFETY_MASK_SAFETY_MASK                             (0xFFFFFFFFU)
#define CSL_CBUFF_MASK_SAFETY_MASK_SAFETY_SHIFT                            (0x00000000U)
#define CSL_CBUFF_MASK_SAFETY_MASK_SAFETY_RESETVAL                         (0xFFFFFFFFU)
#define CSL_CBUFF_MASK_SAFETY_MASK_SAFETY_MAX                              (0xFFFFFFFFU)

#define CSL_CBUFF_MASK_SAFETY_RESETVAL                                     (0xFFFFFFFFU)

/* CLR_SAFETY */

#define CSL_CBUFF_CLR_SAFETY_CLR_SAFETY_MASK                               (0xFFFFFFFFU)
#define CSL_CBUFF_CLR_SAFETY_CLR_SAFETY_SHIFT                              (0x00000000U)
#define CSL_CBUFF_CLR_SAFETY_CLR_SAFETY_RESETVAL                           (0x00000000U)
#define CSL_CBUFF_CLR_SAFETY_CLR_SAFETY_MAX                                (0xFFFFFFFFU)

#define CSL_CBUFF_CLR_SAFETY_RESETVAL                                      (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
