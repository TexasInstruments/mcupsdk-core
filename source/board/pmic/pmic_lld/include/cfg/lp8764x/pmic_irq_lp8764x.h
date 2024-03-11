/******************************************************************************
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
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
 *****************************************************************************/
/**
 *  \addtogroup DRV_PMIC_IRQ_MODULE
 *
 *  @{
 */

/**
 * \file   pmic_irq_lp8764x.h
 *
 * \brief  LP8764x Hera PMIC IRQ Driver API/interface file.
 *
 */

#ifndef PMIC_IRQ_LP8764X_H_
#define PMIC_IRQ_LP8764X_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 *  \anchor Pmic_lp8764x_IrqNum
 *  \name PMIC IRQ Interrupt values for LP8764x HERA PMIC Device.
 *
 *  @{
 */
/*! PMIC WDG RESET Interrupt */
#define PMIC_LP8764X_WD_RST_INT                     (0U)
/*! PMIC WDG FAIL Interrupt */
#define PMIC_LP8764X_WD_FAIL_INT                    (1U)
/*! PMIC WDG LONG WINDOW TIMEOUT Interrupt */
#define PMIC_LP8764X_WD_LONGWIN_TIMEOUT_INT         (2U)

/*! PMIC ESM MCU RESET Interrupt */
#define PMIC_LP8764X_ESM_MCU_RST_INT                (3U)
/*! PMIC ESM MCU FAIL Interrupt */
#define PMIC_LP8764X_ESM_MCU_FAIL_INT               (4U)
/*! PMIC ESM MCU PIN Interrupt */
#define PMIC_LP8764X_ESM_MCU_PIN_INT                (5U)

/*! PMIC NRSTOUT SOC READBACK Interrupt */
#define PMIC_LP8764X_NRSTOUT_SOC_READBACK_INT       (6U)
/*! PMIC EN DRV READBACK Interrupt */
#define PMIC_LP8764X_EN_DRV_READBACK_INT            (7U)

/*! PMIC I2C2 ADDRESS ERROR Interrupt */
#define PMIC_LP8764X_I2C2_ADR_ERR_INT               (8U)
/*! PMIC I2C2 CRC ERROR Interrupt */
#define PMIC_LP8764X_I2C2_CRC_ERR_INT               (9U)
/*! PMIC I2C1/SPI COMM ADDRESS ERROR Interrupt */
#define PMIC_LP8764X_COMM_ADR_ERR_INT               (10U)
/*! PMIC I2C1/SPI COMM CRC ERROR Interrupt */
#define PMIC_LP8764X_COMM_CRC_ERR_INT               (11U)
/*! PMIC SPI COMM FRAME ERROR Interrupt */
#define PMIC_LP8764X_COMM_FRM_ERR_INT               (12U)

/*! PMIC SOC POWER ERR Interrupt */
#define PMIC_LP8764X_SOC_PWR_ERR_INT                (13U)
/*! PMIC MCU POWER ERR Interrupt */
#define PMIC_LP8764X_MCU_PWR_ERR_INT                (14U)
/*! PMIC ORDERLY SHUTDOWN Interrupt */
#define PMIC_LP8764X_ORD_SHUTDOWN_INT               (15U)
/*! PMIC IMMEDIATE SHUTDOWN Interrupt */
#define PMIC_LP8764X_IMM_SHUTOWN_INT                (16U)

/*! PMIC PFSM ERROR Interrupt */
#define PMIC_LP8764X_PFSM_ERR_INT                   (17U)
/*! PMIC VCCA Over-Voltage Interrupt */
#define PMIC_LP8764X_VCCA_OVP_INT                   (18U)
/*! PMIC Thermal Threshold Immediate Shutdown Interrupt */
#define PMIC_LP8764X_TSD_IMM_INT                    (19U)

/*! PMIC NRSTOUT Readback Error Interrupt */
#define PMIC_LP8764X_NRSTOUT_READBACK_INT           (20U)
/*! PMIC NINT Readback Error Interrupt */
#define PMIC_LP8764X_NINT_READBACK_INT              (21U)
/*! PMIC SPMI Interface Error Interrupt */
#define PMIC_LP8764X_SPMI_ERR_INT                   (22U)
/*! PMIC RECOV_CNT Threshold Interrupt */
#define PMIC_LP8764X_RECOV_CNT_INT                  (23U)
/*! PMIC Register CRC Error Interrupt */
#define PMIC_LP8764X_REG_CRC_ERR_INT                (24U)
/*! PMIC LBIST/ABIST Error Interrupt */
#define PMIC_LP8764X_BIST_FAIL_INT                  (25U)
/*! PMIC Thermal Shutdown Orderly Interrupt */
#define PMIC_LP8764X_TSD_ORD_INT                    (26U)

/*! PMIC Thermal Warning Interrupt */
#define PMIC_LP8764X_TWARN_INT                      (27U)
/*! PMIC External Clock Interrupt */
#define PMIC_LP8764X_EXT_CLK_INT                    (28U)
/*! PMIC BIST PASS Interrupt */
#define PMIC_LP8764X_BIST_PASS_INT                  (29U)

/*! PMIC First Supply Detection Interrupt */
#define PMIC_LP8764X_FSD_INT                        (30U)
/*! PMIC ENABLE Interrupt */
#define PMIC_LP8764X_ENABLE_INT                     (31U)

/*! PMIC GPIO PIN 8 Interrupt */
#define PMIC_LP8764X_GPIO8_INT                      (32U)
/*! PMIC GPIO PIN 7 Interrupt */
#define PMIC_LP8764X_GPIO7_INT                      (33U)
/*! PMIC GPIO PIN 6 Interrupt */
#define PMIC_LP8764X_GPIO6_INT                      (34U)
/*! PMIC GPIO PIN 5 Interrupt */
#define PMIC_LP8764X_GPIO5_INT                      (35U)
/*! PMIC GPIO PIN 4 Interrupt */
#define PMIC_LP8764X_GPIO4_INT                      (36U)
/*! PMIC GPIO PIN 3 Interrupt */
#define PMIC_LP8764X_GPIO3_INT                      (37U)
/*! PMIC GPIO PIN 2 Interrupt */
#define PMIC_LP8764X_GPIO2_INT                      (38U)
/*! PMIC GPIO PIN 1 Interrupt */
#define PMIC_LP8764X_GPIO1_INT                      (39U)
/*! PMIC GPIO PIN 10 Interrupt */
#define PMIC_LP8764X_GPIO10_INT                     (40U)
/*! PMIC GPIO PIN 9 Interrupt */
#define PMIC_LP8764X_GPIO9_INT                      (41U)

/*! PMIC VMON2 Residual Voltage Threshold Interrupt */
#define PMIC_LP8764X_VMON2_RV_INT                   (42U)
/*! PMIC VMON2 Under-Voltage Interrupt */
#define PMIC_LP8764X_VMON2_UV_INT                   (43U)
/*! PMIC VMON2 Over-Voltage Interrupt */
#define PMIC_LP8764X_VMON2_OV_INT                   (44U)
/*! PMIC VMON1 Residual Voltage Threshold Interrupt */
#define PMIC_LP8764X_VMON1_RV_INT                   (45U)
/*! PMIC VMON1 Under-Voltage Interrupt */
#define PMIC_LP8764X_VMON1_UV_INT                   (46U)
/*! PMIC VMON1 Over-Voltage Interrupt */
#define PMIC_LP8764X_VMON1_OV_INT                   (47U)
/*! PMIC VCCA Under-Voltage Interrupt */
#define PMIC_LP8764X_VCCA_UV_INT                    (48U)
/*! PMIC VCCA Over-Voltage Interrupt */
#define PMIC_LP8764X_VCCA_OV_INT                    (49U)

/*! PMIC BUCK4 Current Limit Interrupt */
#define PMIC_LP8764X_BUCK4_ILIM_INT                 (50U)
/*! PMIC BUCK4 SC Interrupt */
#define PMIC_LP8764X_BUCK4_SC_INT                   (51U)
/*! PMIC BUCK4 Under-Voltage Interrupt */
#define PMIC_LP8764X_BUCK4_UV_INT                   (52U)
/*! PMIC BUCK4 Over-Voltage Interrupt */
#define PMIC_LP8764X_BUCK4_OV_INT                   (53U)

/*! PMIC BUCK3 Current Limit Interrupt */
#define PMIC_LP8764X_BUCK3_ILIM_INT                 (54U)
/*! PMIC BUCK3 SC Interrupt */
#define PMIC_LP8764X_BUCK3_SC_INT                   (55U)
/*! PMIC BUCK3 Under-Voltage Interrupt */
#define PMIC_LP8764X_BUCK3_UV_INT                   (56U)
/*! PMIC BUCK4 Over-Voltage Interrupt */
#define PMIC_LP8764X_BUCK3_OV_INT                   (57U)

/*! PMIC BUCK2 Current Limit Interrupt */
#define PMIC_LP8764X_BUCK2_ILIM_INT                 (58U)
/*! PMIC BUCK2 SC Interrupt */
#define PMIC_LP8764X_BUCK2_SC_INT                   (59U)
/*! PMIC BUCK2 Under-Voltage Interrupt */
#define PMIC_LP8764X_BUCK2_UV_INT                   (60U)
/*! PMIC BUCK2 Over-Voltage Interrupt */
#define PMIC_LP8764X_BUCK2_OV_INT                   (61U)

/*! PMIC BUCK1 Current Limit Interrupt */
#define PMIC_LP8764X_BUCK1_ILIM_INT                 (62U)
/*! PMIC BUCK1 SC Interrupt */
#define PMIC_LP8764X_BUCK1_SC_INT                   (63U)
/*! PMIC BUCK1 Under-Voltage Interrupt */
#define PMIC_LP8764X_BUCK1_UV_INT                   (64U)
/*! PMIC BUCK1 Over-Voltage Interrupt */
#define PMIC_LP8764X_BUCK1_OV_INT                   (65U)

/*! PMIC SOFT REBOOT Startup Interrupt */
#define PMIC_LP8764X_SOFT_REBOOT_INT                (66U)

/*! PMIC Max Interrupt Number on PG1.0 */
#define PMIC_LP8764X_IRQ_MAX_NUM_PG_1_0             (66U)
/*! PMIC Max Interrupt Number on PG2.0*/
#define PMIC_LP8764X_IRQ_MAX_NUM_PG_2_0             (67U)

/* @} */

/**
 *  \anchor Pmic_lp8764x_IrqGpioNum
 *  \name PMIC GPIO Interrupt Mask values for tps6594x.
 *
 *  @{
 */
#define PMIC_LP8764X_IRQ_GPIO_1_INT_MASK_NUM     (0U)
#define PMIC_LP8764X_IRQ_GPIO_2_INT_MASK_NUM     (1U)
#define PMIC_LP8764X_IRQ_GPIO_3_INT_MASK_NUM     (2U)
#define PMIC_LP8764X_IRQ_GPIO_4_INT_MASK_NUM     (3U)
#define PMIC_LP8764X_IRQ_GPIO_5_INT_MASK_NUM     (4U)
#define PMIC_LP8764X_IRQ_GPIO_6_INT_MASK_NUM     (5U)
#define PMIC_LP8764X_IRQ_GPIO_7_INT_MASK_NUM     (6U)
#define PMIC_LP8764X_IRQ_GPIO_8_INT_MASK_NUM     (7U)
#define PMIC_LP8764X_IRQ_GPIO_9_INT_MASK_NUM     (8U)
#define PMIC_LP8764X_IRQ_GPIO_10_INT_MASK_NUM    (9U)

/* @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif/* PMIC_IRQ_LP8764X_H_ */

/* @} */
