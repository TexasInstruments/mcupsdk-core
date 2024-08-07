/*
 *  Copyright (C) 2020 Texas Instruments Incorporated
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

#ifndef CSLR_SOC_DEFINES_H_
#define CSLR_SOC_DEFINES_H_

#include <stdint.h>


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * \anchor CSL_CoreID
 * \name Core ID's of core or CPUs present on this SOC
 *
 * @{
 */
#define CSL_CORE_ID_R5FSS0_0         (0U)
#define CSL_CORE_ID_R5FSS0_1         (1U)
#define CSL_CORE_ID_C66SS0           (2U)
#define CSL_CORE_ID_MAX              (3U)
/** @} */
/**
 * \anchor PrivID
 * \name Priv ID's of core or CPUs present on this SOC
 *
 * @{
 */
#define PRIV_ID_HSMM4            (1U)
#define PRIV_ID_R5FSS            (2U)
#define PRIV_ID_R5FSSI           (3U)
#define PRIV_ID_DSSTPTC          (5U)

/** @} */


/** @brief Number of ePWM instances */
#define CSL_EPWM_PER_CNT (3U)

/** @brief Number of UART instances */
#define CSL_MSS_UART_PER_CNT                (2U)
#define CSL_DSS_UART_PER_CNT                (1U)
#define CSL_RCSS_UART_PER_CNT               (1U)

/** @brief Number of MIBSPI instances */
#define CSL_MSS_MIBSPI_PER_CNT              (2U)
#define CSL_RCSS_MIBSPI_PER_CNT             (2U)

/** @brief Number of I2C instances */
#define CSL_MSS_I2C_CNT                     (1U)
#define CSL_RCSS_I2C_CNT                    (2U)
#define CSL_MSS_I2C_PER_CNT                 (CSL_MSS_I2C_CNT + CSL_RCSS_I2C_CNT)
#define CSL_DSS_I2C_PER_CNT                 (CSL_RCSS_I2C_CNT)

#define SOC_DSP_L1P_BASE               (CSL_DSP_L1P_U_BASE)
#define SOC_DSP_L1D_BASE               (CSL_DSP_L1D_U_BASE)
#define SOC_DSP_L2_BASE                (CSL_DSP_L2_U_BASE)
#define SOC_DSP_ICFG_BASE              (CSL_DSP_ICFG_U_BASE - 0x800000U)

/*
 * This represents the maximum supported in a SOC across all instances of EDMA
 */
/** @brief Number of DMA Channels */
#define SOC_EDMA_NUM_DMACH                 (64U)
/** @brief Number of QDMA Channels */
#define SOC_EDMA_NUM_QDMACH                (8U)
/** @brief Number of PaRAM Sets available */
#define SOC_EDMA_NUM_PARAMSETS             (256U)
/** @brief Number of Event Queues available */
#define SOC_EDMA_NUM_EVQUE                 (2U)
/** @brief Support for Channel to PaRAM Set mapping */
#define SOC_EDMA_CHMAPEXIST                (1U)
/** @brief Number of EDMA Regions */
#define SOC_EDMA_NUM_REGIONS               (8U)
/** @brief Support for Memory Protection */
#define SOC_EDMA_MEMPROTECT                (1U)

#define MCAN_MSG_RAM_MAX_WORD_COUNT     (4352U)

/* todo! Need to cross verify later */
/*! @brief  Maximum number of Rx Dma buffers. */
#define MCAN_MAX_RX_DMA_BUFFERS             (7U)
/*! @brief  Maximum number of Tx Dma buffers. */
#define MCAN_MAX_TX_DMA_BUFFERS             (4U)

/* ESM number of groups */
#define ESM_NUM_GROUP_MAX                    (3U)
#define ESM_NUM_INTR_PER_GROUP               (128U)

/** @brief DSP TPCC A EVENT MAP */
#define EDMA_DSS_TPCC_A_EVT_RTIA_DMA_REQ0    0
#define EDMA_DSS_TPCC_A_EVT_RTIA_DMA_REQ1    1
#define EDMA_DSS_TPCC_A_EVT_RTIA_DMA_REQ2    2
#define EDMA_DSS_TPCC_A_EVT_RTIA_DMA_REQ3    3
#define EDMA_DSS_TPCC_A_EVT_RTIB_DMA_REQ0    4
#define EDMA_DSS_TPCC_A_EVT_RTIB_DMA_REQ1    5
#define EDMA_DSS_TPCC_A_EVT_RTIB_DMA_REQ2    6
#define EDMA_DSS_TPCC_A_EVT_RTIB_DMA_REQ3    7
#define EDMA_DSS_TPCC_A_EVT_WDT_DMA_REQ0     8
#define EDMA_DSS_TPCC_A_EVT_WDT_DMA_REQ1     9
#define EDMA_DSS_TPCC_A_EVT_WDT_DMA_REQ2     10
#define EDMA_DSS_TPCC_A_EVT_WDT_DMA_REQ3     11
#define EDMA_DSS_TPCC_A_EVT_MCRC_DMA_REQ0    12
#define EDMA_DSS_TPCC_A_EVT_MCRC_DMA_REQ1    13
#define EDMA_DSS_TPCC_A_EVT_SCIA_RX_DMA_REQ  14
#define EDMA_DSS_TPCC_A_EVT_SCIA_TX_DMA_REQ  15
#define EDMA_DSS_TPCC_A_EVT_FREE_0           16
#define EDMA_DSS_TPCC_A_EVT_FREE_1           17
#define EDMA_DSS_TPCC_A_EVT_CBUFF_DMA_REQ0   18
#define EDMA_DSS_TPCC_A_EVT_CBUFF_DMA_REQ1   19
#define EDMA_DSS_TPCC_A_EVT_CBUFF_DMA_REQ2   20
#define EDMA_DSS_TPCC_A_EVT_CBUFF_DMA_REQ3   21
#define EDMA_DSS_TPCC_A_EVT_CBUFF_DMA_REQ4   22
#define EDMA_DSS_TPCC_A_EVT_CBUFF_DMA_REQ5   23
#define EDMA_DSS_TPCC_A_EVT_CBUFF_DMA_REQ6   24
#define EDMA_DSS_TPCC_A_EVT_FREE_2           25
#define EDMA_DSS_TPCC_A_EVT_FREE_3           26
#define EDMA_DSS_TPCC_A_EVT_FREE_4           27
#define EDMA_DSS_TPCC_A_EVT_FREE_5           28
#define EDMA_DSS_TPCC_A_EVT_FREE_6           29
#define EDMA_DSS_TPCC_A_EVT_FREE_7           30
#define EDMA_DSS_TPCC_A_EVT_FREE_8           31
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ0     32
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ1     33
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ2     34
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ3     35
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ4     36
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ5     37
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ6     38
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ7     39
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ8     40
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ9     41
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ10    42
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ11    43
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ12    44
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ13    45
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ14    46
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ15    47
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ16    48
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ17    49
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ18    50
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ19    51
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ20    52
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ21    53
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ22    54
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ23    55
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ24    56
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ25    57
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ26    58
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ27    59
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ28    60
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ29    61
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ30    62
#define EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ31    63

/** @brief DSP TPCC B EVENT MAP */
#define EDMA_DSS_TPCC_B_EVT_RTIA_DMA_REQ0    0
#define EDMA_DSS_TPCC_B_EVT_RTIA_DMA_REQ1    1
#define EDMA_DSS_TPCC_B_EVT_RTIA_DMA_REQ2    2
#define EDMA_DSS_TPCC_B_EVT_RTIA_DMA_REQ3    3
#define EDMA_DSS_TPCC_B_EVT_RTIB_DMA_REQ0    4
#define EDMA_DSS_TPCC_B_EVT_RTIB_DMA_REQ1    5
#define EDMA_DSS_TPCC_B_EVT_RTIB_DMA_REQ2    6
#define EDMA_DSS_TPCC_B_EVT_RTIB_DMA_REQ3    7
#define EDMA_DSS_TPCC_B_EVT_WDT_DMA_REQ0     8
#define EDMA_DSS_TPCC_B_EVT_WDT_DMA_REQ1     9
#define EDMA_DSS_TPCC_B_EVT_WDT_DMA_REQ2     10
#define EDMA_DSS_TPCC_B_EVT_WDT_DMA_REQ3     11
#define EDMA_DSS_TPCC_B_EVT_MCRC_DMA_REQ0    12
#define EDMA_DSS_TPCC_B_EVT_MCRC_DMA_REQ1    13
#define EDMA_DSS_TPCC_B_EVT_SCIA_RX_DMA_REQ  14
#define EDMA_DSS_TPCC_B_EVT_SCIA_TX_DMA_REQ  15
#define EDMA_DSS_TPCC_B_EVT_CSI2A_EOF_INT    16
#define EDMA_DSS_TPCC_B_EVT_CSI2A_EOL_INT    17
#define EDMA_DSS_TPCC_B_EVT_CBUFF_DMA_REQ0   18
#define EDMA_DSS_TPCC_B_EVT_CBUFF_DMA_REQ1   19
#define EDMA_DSS_TPCC_B_EVT_CBUFF_DMA_REQ2   20
#define EDMA_DSS_TPCC_B_EVT_CBUFF_DMA_REQ3   21
#define EDMA_DSS_TPCC_B_EVT_CBUFF_DMA_REQ4   22
#define EDMA_DSS_TPCC_B_EVT_CBUFF_DMA_REQ5   23
#define EDMA_DSS_TPCC_B_EVT_CBUFF_DMA_REQ6   24
#define EDMA_DSS_TPCC_B_EVT_CSI2A_SOF_INT0   25
#define EDMA_DSS_TPCC_B_EVT_CSI2A_SOF_INT1   26
#define EDMA_DSS_TPCC_B_EVT_CSI2A_EOL_CNTX0  27
#define EDMA_DSS_TPCC_B_EVT_CSI2A_EOL_CNTX1  28
#define EDMA_DSS_TPCC_B_EVT_CSI2A_EOL_CNTX2  29
#define EDMA_DSS_TPCC_B_EVT_CSI2A_EOL_CNTX3  30
#define EDMA_DSS_TPCC_B_EVT_FREE_0           31
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ0     32
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ1     33
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ2     34
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ3     35
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ4     36
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ5     37
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ6     38
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ7     39
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ8     40
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ9     41
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ10    42
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ11    43
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ12    44
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ13    45
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ14    46
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ15    47
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ16    48
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ17    49
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ18    50
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ19    51
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ20    52
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ21    53
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ22    54
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ23    55
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ24    56
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ25    57
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ26    58
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ27    59
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ28    60
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ29    61
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ30    62
#define EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ31    63

/** @brief DSP TPCC C EVENT MAP */
#define EDMA_DSS_TPCC_C_EVT_RTIA_DMA_REQ0    0
#define EDMA_DSS_TPCC_C_EVT_RTIA_DMA_REQ1    1
#define EDMA_DSS_TPCC_C_EVT_RTIA_DMA_REQ2    2
#define EDMA_DSS_TPCC_C_EVT_RTIA_DMA_REQ3    3
#define EDMA_DSS_TPCC_C_EVT_RTIB_DMA_REQ0    4
#define EDMA_DSS_TPCC_C_EVT_RTIB_DMA_REQ1    5
#define EDMA_DSS_TPCC_C_EVT_RTIB_DMA_REQ2    6
#define EDMA_DSS_TPCC_C_EVT_RTIB_DMA_REQ3    7
#define EDMA_DSS_TPCC_C_EVT_WDT_DMA_REQ0     8
#define EDMA_DSS_TPCC_C_EVT_WDT_DMA_REQ1     9
#define EDMA_DSS_TPCC_C_EVT_WDT_DMA_REQ2     10
#define EDMA_DSS_TPCC_C_EVT_WDT_DMA_REQ3     11
#define EDMA_DSS_TPCC_C_EVT_MCRC_DMA_REQ0    12
#define EDMA_DSS_TPCC_C_EVT_MCRC_DMA_REQ1    13
#define EDMA_DSS_TPCC_C_EVT_SCIA_RX_DMA_REQ  14
#define EDMA_DSS_TPCC_C_EVT_SCIA_TX_DMA_REQ  15
#define EDMA_DSS_TPCC_C_EVT_CSI2B_EOF_INT    16
#define EDMA_DSS_TPCC_C_EVT_CSI2B_EOL_INT    17
#define EDMA_DSS_TPCC_C_EVT_CBUFF_DMA_REQ0   18
#define EDMA_DSS_TPCC_C_EVT_CBUFF_DMA_REQ1   19
#define EDMA_DSS_TPCC_C_EVT_CBUFF_DMA_REQ2   20
#define EDMA_DSS_TPCC_C_EVT_CBUFF_DMA_REQ3   21
#define EDMA_DSS_TPCC_C_EVT_CBUFF_DMA_REQ4   22
#define EDMA_DSS_TPCC_C_EVT_CBUFF_DMA_REQ5   23
#define EDMA_DSS_TPCC_C_EVT_CBUFF_DMA_REQ6   24
#define EDMA_DSS_TPCC_C_EVT_CSI2B_SOF_INT0   25
#define EDMA_DSS_TPCC_C_EVT_CSI2B_SOF_INT1   26
#define EDMA_DSS_TPCC_C_EVT_CSI2B_EOL_CNTX0  27
#define EDMA_DSS_TPCC_C_EVT_CSI2B_EOL_CNTX1  28
#define EDMA_DSS_TPCC_C_EVT_CSI2B_EOL_CNTX2  29
#define EDMA_DSS_TPCC_C_EVT_CSI2B_EOL_CNTX3  30
#define EDMA_DSS_TPCC_C_EVT_FREE_0           31
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0     32
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ1     33
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ2     34
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ3     35
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ4     36
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ5     37
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ6     38
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ7     39
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ8     40
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ9     41
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ10    42
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ11    43
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ12    44
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ13    45
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ14    46
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ15    47
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ16    48
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ17    49
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ18    50
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ19    51
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ20    52
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ21    53
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ22    54
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ23    55
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ24    56
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ25    57
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ26    58
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ27    59
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ28    60
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ29    61
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ30    62
#define EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ31    63


/** @brief MSS TPCC A EVENT MAP */
#define EDMA_MSS_TPCC_A_EVT_SPIA_DMA_REQ0    0
#define EDMA_MSS_TPCC_A_EVT_SPIA_DMA_REQ1    1
#define EDMA_MSS_TPCC_A_EVT_SPIA_DMA_REQ2    2
#define EDMA_MSS_TPCC_A_EVT_SPIA_DMA_REQ3    3
#define EDMA_MSS_TPCC_A_EVT_SPIA_DMA_REQ4    4
#define EDMA_MSS_TPCC_A_EVT_SPIA_DMA_REQ5    5
#define EDMA_MSS_TPCC_A_EVT_SPIB_DMA_REQ0    6
#define EDMA_MSS_TPCC_A_EVT_SPIB_DMA_REQ1    7
#define EDMA_MSS_TPCC_A_EVT_SPIB_DMA_REQ2    8
#define EDMA_MSS_TPCC_A_EVT_SPIB_DMA_REQ3    9
#define EDMA_MSS_TPCC_A_EVT_SPIB_DMA_REQ4    10
#define EDMA_MSS_TPCC_A_EVT_SPIB_DMA_REQ5    11
#define EDMA_MSS_TPCC_A_EVT_QSPI_DMA_REQ0    12
#define EDMA_MSS_TPCC_A_EVT_MCRC_DMA_REQ0    13
#define EDMA_MSS_TPCC_A_EVT_MCRC_DMA_REQ1    14
#define EDMA_MSS_TPCC_A_EVT_RTIA_DMA_REQ0    15
#define EDMA_MSS_TPCC_A_EVT_RTIA_DMA_REQ1    16
#define EDMA_MSS_TPCC_A_EVT_RTIA_DMA_REQ2    17
#define EDMA_MSS_TPCC_A_EVT_RTIA_DMA_REQ3    18
#define EDMA_MSS_TPCC_A_EVT_RTIB_DMA_REQ0    19
#define EDMA_MSS_TPCC_A_EVT_RTIB_DMA_REQ1    20
#define EDMA_MSS_TPCC_A_EVT_RTIC_DMA_REQ0    21
#define EDMA_MSS_TPCC_A_EVT_RTIC_DMA_REQ1    22
#define EDMA_MSS_TPCC_A_EVT_WDT_DMA_REQ0     23
#define EDMA_MSS_TPCC_A_EVT_WDT_DMA_REQ1     24
#define EDMA_MSS_TPCC_A_EVT_WDT_DMA_REQ2     25
#define EDMA_MSS_TPCC_A_EVT_WDT_DMA_REQ3     26
#define EDMA_MSS_TPCC_A_EVT_ETPWMA_DMA_REQ0  27
#define EDMA_MSS_TPCC_A_EVT_ETPWMA_DMA_REQ1  28
#define EDMA_MSS_TPCC_A_EVT_ETPWMB_DMA_REQ0  29
#define EDMA_MSS_TPCC_A_EVT_ETPWMB_DMA_REQ1  30
#define EDMA_MSS_TPCC_A_EVT_ETPWMC_DMA_REQ0  31
#define EDMA_MSS_TPCC_A_EVT_ETPWMC_DMA_REQ1  32
#define EDMA_MSS_TPCC_A_EVT_MCANA_DMA_REQ0   33
#define EDMA_MSS_TPCC_A_EVT_MCANA_DMA_REQ1   34
#define EDMA_MSS_TPCC_A_EVT_MCANA_FE_INT1    35
#define EDMA_MSS_TPCC_A_EVT_MCANA_FE_INT2    36
#define EDMA_MSS_TPCC_A_EVT_MCANA_FE_INT4    37
#define EDMA_MSS_TPCC_A_EVT_MCANB_DMA_REQ0   38
#define EDMA_MSS_TPCC_A_EVT_MCANB_DMA_REQ1   39
#define EDMA_MSS_TPCC_A_EVT_MCANB_FE_INT1    40
#define EDMA_MSS_TPCC_A_EVT_MCANB_FE_INT2    41
#define EDMA_MSS_TPCC_A_EVT_MCANB_FE_INT4    42
#define EDMA_MSS_TPCC_A_EVT_RTIB_DMA_REQ2    43
#define EDMA_MSS_TPCC_A_EVT_RTIB_DMA_REQ3    44
#define EDMA_MSS_TPCC_A_EVT_RTIC_DMA_REQ2    45
#define EDMA_MSS_TPCC_A_EVT_RTIC_DMA_REQ3    46
#define EDMA_MSS_TPCC_A_EVT_FREE_0           47
#define EDMA_MSS_TPCC_A_EVT_FREE_1           48
#define EDMA_MSS_TPCC_A_EVT_GIO_PAD_INT0     49
#define EDMA_MSS_TPCC_A_EVT_GIO_PAD_INT1     50
#define EDMA_MSS_TPCC_A_EVT_GIO_PAD_INT2     51
#define EDMA_MSS_TPCC_A_EVT_GIO_PAD_INT3     52
#define EDMA_MSS_TPCC_A_EVT_GIO_PAD_INT4     53
#define EDMA_MSS_TPCC_A_EVT_FREE_2           54
#define EDMA_MSS_TPCC_A_EVT_I2C_DMA_REQ0     55
#define EDMA_MSS_TPCC_A_EVT_I2C_DMA_REQ1     56
#define EDMA_MSS_TPCC_A_EVT_SCIA_RX_DMA_REQ  57
#define EDMA_MSS_TPCC_A_EVT_SCIA_TX_DMA_REQ  58
#define EDMA_MSS_TPCC_A_EVT_SCIB_RX_DMA_REQ  59
#define EDMA_MSS_TPCC_A_EVT_SCIB_TX_DMA_REQ  60
#define EDMA_MSS_TPCC_A_EVT_FREE_3           61
#define EDMA_MSS_TPCC_A_EVT_FREE_4           62
#define EDMA_MSS_TPCC_A_EVT_CBUFF_DMA_REQ    63

/** @brief MSS TPCC B EVENT MAP */
#define EDMA_MSS_TPCC_B_EVT_MCRC_DMA_REQ0       0
#define EDMA_MSS_TPCC_B_EVT_MCRC_DMA_REQ1       1
#define EDMA_MSS_TPCC_B_EVT_ETPWMA_DMA_REQ0     2
#define EDMA_MSS_TPCC_B_EVT_ETPWMA_DMA_REQ1     3
#define EDMA_MSS_TPCC_B_EVT_ETPWMB_DMA_REQ0     4
#define EDMA_MSS_TPCC_B_EVT_ETPWMB_DMA_REQ1     5
#define EDMA_MSS_TPCC_B_EVT_ETPWMC_DMA_REQ0     6
#define EDMA_MSS_TPCC_B_EVT_ETPWMC_DMA_REQ1     7
#define EDMA_MSS_TPCC_B_EVT_MCANA_DMA_REQ0      8
#define EDMA_MSS_TPCC_B_EVT_MCANA_DMA_REQ1      9
#define EDMA_MSS_TPCC_B_EVT_MCANB_DMA_REQ0      10
#define EDMA_MSS_TPCC_B_EVT_MCANB_DMA_REQ1      11
#define EDMA_MSS_TPCC_B_EVT_FREE_0              12
#define EDMA_MSS_TPCC_B_EVT_FREE_1              13
#define EDMA_MSS_TPCC_B_EVT_FREE_2              14
#define EDMA_MSS_TPCC_B_EVT_FREE_3              15
#define EDMA_MSS_TPCC_B_EVT_GIO_PAD_INT0        16
#define EDMA_MSS_TPCC_B_EVT_GIO_PAD_INT1        17
#define EDMA_MSS_TPCC_B_EVT_GIO_PAD_INT2        18
#define EDMA_MSS_TPCC_B_EVT_GIO_PAD_INT3        19
#define EDMA_MSS_TPCC_B_EVT_GIO_PAD_INT4        20
#define EDMA_MSS_TPCC_B_EVT_FREE_4              21
#define EDMA_MSS_TPCC_B_EVT_FREE_5              22
#define EDMA_MSS_TPCC_B_EVT_FREE_6              23
#define EDMA_MSS_TPCC_B_EVT_FREE_7              24
#define EDMA_MSS_TPCC_B_EVT_FREE_8              25
#define EDMA_MSS_TPCC_B_EVT_DTHE_SHA_DMA_REQ0   26
#define EDMA_MSS_TPCC_B_EVT_DTHE_SHA_DMA_REQ1   27
#define EDMA_MSS_TPCC_B_EVT_DTHE_SHA_DMA_REQ2   28
#define EDMA_MSS_TPCC_B_EVT_DTHE_SHA_DMA_REQ3   29
#define EDMA_MSS_TPCC_B_EVT_DTHE_SHA_DMA_REQ4   30
#define EDMA_MSS_TPCC_B_EVT_DTHE_SHA_DMA_REQ5   31
#define EDMA_MSS_TPCC_B_EVT_DTHE_AES_DMA_REQ0   32
#define EDMA_MSS_TPCC_B_EVT_DTHE_AES_DMA_REQ1   33
#define EDMA_MSS_TPCC_B_EVT_DTHE_AES_DMA_REQ2   34
#define EDMA_MSS_TPCC_B_EVT_DTHE_AES_DMA_REQ3   35
#define EDMA_MSS_TPCC_B_EVT_DTHE_AES_DMA_REQ4   36
#define EDMA_MSS_TPCC_B_EVT_DTHE_AES_DMA_REQ5   37
#define EDMA_MSS_TPCC_B_EVT_DTHE_AES_DMA_REQ6   38
#define EDMA_MSS_TPCC_B_EVT_DTHE_AES_DMA_REQ7   39
#define EDMA_MSS_TPCC_B_EVT_FREE_9              40
#define EDMA_MSS_TPCC_B_EVT_FREE_10             41
#define EDMA_MSS_TPCC_B_EVT_FREE_11             42
#define EDMA_MSS_TPCC_B_EVT_MCANA_FE_INT1       43
#define EDMA_MSS_TPCC_B_EVT_MCANA_FE_INT2       44
#define EDMA_MSS_TPCC_B_EVT_MCANA_FE_INT3       45
#define EDMA_MSS_TPCC_B_EVT_MCANA_FE_INT4       46
#define EDMA_MSS_TPCC_B_EVT_MCANA_FE_INT5       47
#define EDMA_MSS_TPCC_B_EVT_MCANA_FE_INT6       48
#define EDMA_MSS_TPCC_B_EVT_MCANA_FE_INT7       49
#define EDMA_MSS_TPCC_B_EVT_MCANB_FE_INT1       50
#define EDMA_MSS_TPCC_B_EVT_MCANB_FE_INT2       51
#define EDMA_MSS_TPCC_B_EVT_MCANB_FE_INT3       52
#define EDMA_MSS_TPCC_B_EVT_MCANB_FE_INT4       53
#define EDMA_MSS_TPCC_B_EVT_MCANB_FE_INT5       54
#define EDMA_MSS_TPCC_B_EVT_MCANB_FE_INT6       55
#define EDMA_MSS_TPCC_B_EVT_MCANB_FE_INT7       56
#define EDMA_MSS_TPCC_B_EVT_FREE_12             57
#define EDMA_MSS_TPCC_B_EVT_FREE_13             58
#define EDMA_MSS_TPCC_B_EVT_FREE_14             59
#define EDMA_MSS_TPCC_B_EVT_FREE_15             60
#define EDMA_MSS_TPCC_B_EVT_FREE_16             61
#define EDMA_MSS_TPCC_B_EVT_FREE_17             62
#define EDMA_MSS_TPCC_B_EVT_FREE_18             63

/** @brief RCSS TPCC A EVENT MAP */
#define EDMA_RCSS_TPCC_A_EVT_SPIA_DMA_REQ0          0
#define EDMA_RCSS_TPCC_A_EVT_SPIA_DMA_REQ1          1
#define EDMA_RCSS_TPCC_A_EVT_SPIA_DMA_REQ2          2
#define EDMA_RCSS_TPCC_A_EVT_SPIA_DMA_REQ3          3
#define EDMA_RCSS_TPCC_A_EVT_SPIA_DMA_REQ4          4
#define EDMA_RCSS_TPCC_A_EVT_SPIA_DMA_REQ5          5
#define EDMA_RCSS_TPCC_A_EVT_SPIB_DMA_REQ0          6
#define EDMA_RCSS_TPCC_A_EVT_SPIB_DMA_REQ1          7
#define EDMA_RCSS_TPCC_A_EVT_SPIB_DMA_REQ2          8
#define EDMA_RCSS_TPCC_A_EVT_SPIB_DMA_REQ3          9
#define EDMA_RCSS_TPCC_A_EVT_SPIB_DMA_REQ4          10
#define EDMA_RCSS_TPCC_A_EVT_SPIB_DMA_REQ5          11
#define EDMA_RCSS_TPCC_A_EVT_ECAP_DMA_REQ           12
#define EDMA_RCSS_TPCC_A_EVT_FREE_0                 13
#define EDMA_RCSS_TPCC_A_EVT_FREE_1                 14
#define EDMA_RCSS_TPCC_A_EVT_FREE_2                 15
#define EDMA_RCSS_TPCC_A_EVT_CSI2A_EOF_INT          16
#define EDMA_RCSS_TPCC_A_EVT_CSI2A_EOL_INT          17
#define EDMA_RCSS_TPCC_A_EVT_CSI2A_EOL_CNTX0_INT    18
#define EDMA_RCSS_TPCC_A_EVT_CSI2A_EOL_CNTX1_INT    19
#define EDMA_RCSS_TPCC_A_EVT_CSI2A_EOL_CNTX2_INT    20
#define EDMA_RCSS_TPCC_A_EVT_CSI2A_EOL_CNTX3_INT    21
#define EDMA_RCSS_TPCC_A_EVT_CSI2A_EOL_CNTX4_INT    22
#define EDMA_RCSS_TPCC_A_EVT_CSI2A_EOL_CNTX5_INT    23
#define EDMA_RCSS_TPCC_A_EVT_CSI2A_EOL_CNTX6_INT    24
#define EDMA_RCSS_TPCC_A_EVT_CSI2A_EOL_CNTX7_INT    25
#define EDMA_RCSS_TPCC_A_EVT_CSI2A_SOF_TRIG0_INT    26
#define EDMA_RCSS_TPCC_A_EVT_CSI2A_SOF_TRIG1_INT    27
#define EDMA_RCSS_TPCC_A_EVT_FREE_3                 28
#define EDMA_RCSS_TPCC_A_EVT_FREE_4                 29
#define EDMA_RCSS_TPCC_A_EVT_FREE_5                 30
#define EDMA_RCSS_TPCC_A_EVT_FREE_6                 31
#define EDMA_RCSS_TPCC_A_EVT_CSI2B_EOF_INT          32
#define EDMA_RCSS_TPCC_A_EVT_CSI2B_EOL_INT          33
#define EDMA_RCSS_TPCC_A_EVT_CSI2B_EOL_CNTX0_INT    34
#define EDMA_RCSS_TPCC_A_EVT_CSI2B_EOL_CNTX1_INT    35
#define EDMA_RCSS_TPCC_A_EVT_CSI2B_EOL_CNTX2_INT    36
#define EDMA_RCSS_TPCC_A_EVT_CSI2B_EOL_CNTX3_INT    37
#define EDMA_RCSS_TPCC_A_EVT_CSI2B_EOL_CNTX4_INT    38
#define EDMA_RCSS_TPCC_A_EVT_CSI2B_EOL_CNTX5_INT    39
#define EDMA_RCSS_TPCC_A_EVT_CSI2B_EOL_CNTX6_INT    40
#define EDMA_RCSS_TPCC_A_EVT_CSI2B_EOL_CNTX7_INT    41
#define EDMA_RCSS_TPCC_A_EVT_CSI2B_SOF_TRIG0_INT    42
#define EDMA_RCSS_TPCC_A_EVT_CSI2B_SOF_TRIG1_INT    43
#define EDMA_RCSS_TPCC_A_EVT_SCIA_TX_SINGLE_REQ     44
#define EDMA_RCSS_TPCC_A_EVT_SCIA_TX_BURST_REQ      45
#define EDMA_RCSS_TPCC_A_EVT_FREE_7                 46
#define EDMA_RCSS_TPCC_A_EVT_FREE_8                 47
#define EDMA_RCSS_TPCC_A_EVT_MCASPA_TX_REQ          48
#define EDMA_RCSS_TPCC_A_EVT_MCASPB_TX_REQ          49
#define EDMA_RCSS_TPCC_A_EVT_MCASPC_TX_REQ          50
#define EDMA_RCSS_TPCC_A_EVT_MCASPA_RX_REQ          51
#define EDMA_RCSS_TPCC_A_EVT_MCASPB_RX_REQ          52
#define EDMA_RCSS_TPCC_A_EVT_MCASPC_RX_REQ          53
#define EDMA_RCSS_TPCC_A_EVT_I2CA_TX_DMA_REQ        54
#define EDMA_RCSS_TPCC_A_EVT_I2CA_RX_DMA_REQ        55
#define EDMA_RCSS_TPCC_A_EVT_I2CB_TX_DMA_REQ        56
#define EDMA_RCSS_TPCC_A_EVT_I2CB_RX_DMA_REQ        57
#define EDMA_RCSS_TPCC_A_EVT_SCIA_RX_SINGLE_REQ     58
#define EDMA_RCSS_TPCC_A_EVT_SCIA_RX_BURST_REQ      59
#define EDMA_RCSS_TPCC_A_EVT_FREE_9                 60
#define EDMA_RCSS_TPCC_A_EVT_FREE_10                61
#define EDMA_RCSS_TPCC_A_EVT_FREE_11                62
#define EDMA_RCSS_TPCC_A_EVT_FREE_12                63


#define EDMA_DSS_TPCC_A_NUM_PARAM_SETS  (128U)
#define EDMA_DSS_TPCC_A_NUM_DMA_CHANS   (64U)
#define EDMA_DSS_TPCC_A_NUM_TC          (2U)

#define EDMA_DSS_TPCC_B_NUM_PARAM_SETS  (128U)
#define EDMA_DSS_TPCC_B_NUM_DMA_CHANS   (64U)
#define EDMA_DSS_TPCC_B_NUM_TC          (2U)

#define EDMA_DSS_TPCC_C_NUM_PARAM_SETS  (256U)
#define EDMA_DSS_TPCC_C_NUM_DMA_CHANS   (64U)

/*! Note even though EDMA's CCCFG register indicates 6 TCs for CC2, only first 2
 *  are verified for radar processing flows, hence we limit to 2 TCs. */
#define EDMA_DSS_TPCC_C_NUM_TC          (2U)

#define EDMA_RCSS_TPCC_A_NUM_PARAM_SETS  (128U)
#define EDMA_RDSS_TPCC_A_NUM_DMA_CHANS   (64U)
#define EDMA_RCSS_TPCC_A_NUM_TC          (2U)

#define EDMA_MSS_TPCC_A_NUM_PARAM_SETS  (128U)
#define EDMA_MSS_TPCC_A_NUM_DMA_CHANS   (64U)
#define EDMA_MSS_TPCC_A_NUM_TC          (2U)

#define EDMA_MSS_TPCC_B_NUM_PARAM_SETS  (128U)
#define EDMA_MSS_TPCC_B_NUM_DMA_CHANS   (64U)
#define EDMA_MSS_TPCC_B_NUM_TC          (1U)

#define EDMA_HSM_TPCC_A_NUM_PARAM_SETS  (128U)
#define EDMA_HSM_TPCC_A_NUM_TC          (2U)

#define EDMA_TPCC_ERRAGG_TPCC_EERINT__POS   (0U)
#define EDMA_TPCC_INTAGG_TPCC_INTG__POS     (0U)
#define EDMA_TPCC_ERRAGG_TPTC_MIN_ERR__POS  (2U) /* position of the lowest TC Id, others are higher */

#define EDMA_DSS_NUM_CC        4

#define EDMA_DSS_MAX_NUM_TC     CSL_MAX(EDMA_DSS_TPCC_A_NUM_TC, \
                                CSL_MAX(EDMA_DSS_TPCC_B_NUM_TC, \
                                CSL_MAX(EDMA_DSS_TPCC_C_NUM_TC, \
                                EDMA_RCSS_TPCC_A_NUM_TC)))

#define EDMA_MSS_NUM_CC        6

#define EDMA_MSS_MAX_NUM_TC     CSL_MAX(EDMA_MSS_TPCC_A_NUM_TC, \
                                CSL_MAX(EDMA_MSS_TPCC_B_NUM_TC, \
                                CSL_MAX(EDMA_DSS_TPCC_A_NUM_TC, \
                                CSL_MAX(EDMA_DSS_TPCC_B_NUM_TC, \
                                CSL_MAX(EDMA_DSS_TPCC_C_NUM_TC, \
                                EDMA_RCSS_TPCC_A_NUM_TC)))))

/***********************************************************************
 * Peripheral number of instance definition
 ***********************************************************************/
#define HWA_NUM_INSTANCES               (1U)

/*! \brief number of HWA memory banks */
#define SOC_HWA_NUM_MEM_BANKS           (8U)
/*! \brief number of HWA parameter sets */
#define SOC_HWA_NUM_PARAM_SETS          (64U)
/*! \brief number of HWA MDA channels */
#define SOC_HWA_NUM_DMA_CHANNEL         (32U)
/*! \brief number of csirx IRQs*/
#define SOC_HWA_NUM_CSIRX_IRQS          (20U)
/*! \brief number of HWA memory size in bytes */
#define SOC_HWA_MEM_SIZE                (CSL_DSS_HWA_BANK_SIZE * SOC_HWA_NUM_MEM_BANKS)

/***********************************************************************
 * HWA Hardware trigger source definitions
 ***********************************************************************/
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2B_CONTEXT_0_LINE_END  (0U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2B_CONTEXT_1_LINE_END  (1U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2B_CONTEXT_2_LINE_END  (2U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2B_CONTEXT_3_LINE_END  (3U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2B_CONTEXT_4_LINE_END  (4U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2B_CONTEXT_5_LINE_END  (5U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2B_CONTEXT_6_LINE_END  (6U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2B_CONTEXT_7_LINE_END  (7U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2B_FRAME_START_0       (8U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2B_FRAME_START_1       (9U)

#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2A_CONTEXT_0_LINE_END  (10U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2A_CONTEXT_1_LINE_END  (11U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2A_CONTEXT_2_LINE_END  (12U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2A_CONTEXT_3_LINE_END  (13U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2A_CONTEXT_4_LINE_END  (14U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2A_CONTEXT_5_LINE_END  (15U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2A_CONTEXT_6_LINE_END  (16U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2A_CONTEXT_7_LINE_END  (17U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2A_FRAME_START_0       (18U)
#define HWA_HARDWARE_TRIGGER_SOURCE_CSI2A_FRAME_START_1       (19U)


/***********************************************************************
 * MSS - CLOCK settings
 ***********************************************************************/
 /* Sys_vclk : 200MHz */
#define MSS_SYS_VCLK                  200000000U
#define R5F_CLOCK_MHZ                 400U

/**
 *  \anchor CSL_ArmR5ClusterGroupID
 *  \name R5 Cluster Group IDs
 *
 *  @{
 */
/** \brief R5 Cluster Group ID0 */
#define CSL_ARM_R5_CLUSTER_GROUP_ID_0                 ((uint32_t) 0x00U)
/** @} */

/**
 *  \anchor CSL_ArmR5CPUID
 *  \name R5 Core IDs
 *
 *  @{
 */
/** \brief R5 Core ID0 */
#define CSL_ARM_R5_CPU_ID_0                          ((uint32_t) 0x00U)
/** \brief R5 Core ID1 */
#define CSL_ARM_R5_CPU_ID_1                          ((uint32_t) 0x01U)
/** @} */

/***********************************************************************
 * Cache line size definitions
 ***********************************************************************/
#if (__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R') /* R5F */
#define CSL_CACHE_L1P_LINESIZE     (32U)
#define CSL_CACHE_L1D_LINESIZE     (32U)
#elif defined(_TMS320C6X) /* C66 */
#define CSL_CACHE_L1P_LINESIZE     (32U)
#define CSL_CACHE_L1D_LINESIZE     (64U)
#define CSL_CACHE_L2_LINESIZE      (128U)
#endif

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*! @brief R5F to Hardware Accelerator address translation macro. */
//#define ADDR_TRANSLATE_CPU_TO_HWA(x)  (uint16_t)(((uint32_t)(x) - SOC_XWR18XX_MSS_HWA_MEM0_BASE_ADDRESS) & 0x0000FFFFU)


/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif  /* CSLR_SOC_DEFINES_H_ */
