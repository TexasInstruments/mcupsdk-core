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
 * \file   pmic_irq_tps6594x.h
 *
 * \brief  TPS6594x LEO PMIC IRQ Driver API/interface file.
 *
 */

#ifndef PMIC_IRQ_TPS6594X_H_
#define PMIC_IRQ_TPS6594X_H_

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
 *  \anchor Pmic_tps6594x_IrqNum
 *  \name PMIC IRQ Interrupt values for tps6594x LEO PMIC device
 *
 *  @{
 */

/*! PMIC WDG RESET Interrupt */
#define PMIC_TPS6594X_WD_RST_INT                     (0U)
/*! PMIC WDG FAIL Interrupt */
#define PMIC_TPS6594X_WD_FAIL_INT                    (1U)
/*! PMIC WDG LONG WINDOW TIMEOUT Interrupt */
#define PMIC_TPS6594X_WD_LONGWIN_TIMEOUT_INT         (2U)

/*! PMIC ESM MCU RESET Interrupt */
#define PMIC_TPS6594X_ESM_MCU_RST_INT                (3U)
/*! PMIC ESM MCU FAIL Interrupt */
#define PMIC_TPS6594X_ESM_MCU_FAIL_INT               (4U)
/*! PMIC ESM MCU PIN Interrupt */
#define PMIC_TPS6594X_ESM_MCU_PIN_INT                (5U)
/*! PMIC ESM SOC RESET Interrupt */
#define PMIC_TPS6594X_ESM_SOC_RST_INT                (6U)
/*! PMIC ESM SOC FAIL Interrupt */
#define PMIC_TPS6594X_ESM_SOC_FAIL_INT               (7U)
/*! PMIC ESM SOC PIN Interrupt */
#define PMIC_TPS6594X_ESM_SOC_PIN_INT                (8U)

/*! PMIC NRSTOUT SOC READBACK Interrupt */
#define PMIC_TPS6594X_NRSTOUT_SOC_READBACK_INT       (9U)
/*! PMIC EN DRV READBACK Interrupt */
#define PMIC_TPS6594X_EN_DRV_READBACK_INT            (10U)

/*! PMIC I2C2 ADDRESS ERROR Interrupt */
#define PMIC_TPS6594X_I2C2_ADR_ERR_INT               (11U)
/*! PMIC I2C2 CRC ERROR Interrupt */
#define PMIC_TPS6594X_I2C2_CRC_ERR_INT               (12U)
/*! PMIC I2C1/SPI COMM ADDRESS ERROR Interrupt */
#define PMIC_TPS6594X_COMM_ADR_ERR_INT               (13U)
/*! PMIC I2C1/SPI COMM CRC ERROR Interrupt */
#define PMIC_TPS6594X_COMM_CRC_ERR_INT               (14U)
/*! PMIC SPI COMM FRAME ERROR Interrupt */
#define PMIC_TPS6594X_COMM_FRM_ERR_INT               (15U)

/*! PMIC SOC POWER ERR Interrupt */
#define PMIC_TPS6594X_SOC_PWR_ERR_INT                (16U)
/*! PMIC MCU POWER ERR Interrupt */
#define PMIC_TPS6594X_MCU_PWR_ERR_INT                (17U)
/*! PMIC ORDERLY SHUTDOWN Interrupt */
#define PMIC_TPS6594X_ORD_SHUTDOWN_INT               (18U)
/*! PMIC IMMEDIATE SHUTDOWN Interrupt */
#define PMIC_TPS6594X_IMM_SHUTOWN_INT                (19U)

/*! PMIC PFSM ERROR Interrupt */
#define PMIC_TPS6594X_PFSM_ERR_INT                   (20U)
/*! PMIC VCCA Over-Voltage Interrupt */
#define PMIC_TPS6594X_VCCA_OVP_INT                   (21U)
/*! PMIC Thermal Threshold Immediate Shutdown Interrupt */
#define PMIC_TPS6594X_TSD_IMM_INT                    (22U)

/*! PMIC NRSTOUT Readback Error Interrupt */
#define PMIC_TPS6594X_NRSTOUT_READBACK_INT           (23U)
/*! PMIC NINT Readback Error Interrupt */
#define PMIC_TPS6594X_NINT_READBACK_INT              (24U)
/*! PMIC NPWRON Long Press Error Interrupt */
#define PMIC_TPS6594X_NPWRON_LONG_INT                (25U)
/*! PMIC SPMI Interface Error Interrupt */
#define PMIC_TPS6594X_SPMI_ERR_INT                   (26U)
/*! PMIC RECOV_CNT Threshold Interrupt */
#define PMIC_TPS6594X_RECOV_CNT_INT                  (27U)
/*! PMIC Register CRC Error Interrupt */
#define PMIC_TPS6594X_REG_CRC_ERR_INT                (28U)
/*! PMIC LBIST/ABIST Error Interrupt */
#define PMIC_TPS6594X_BIST_FAIL_INT                  (29U)
/*! PMIC Thermal Shutdown Orderly Interrupt */
#define PMIC_TPS6594X_TSD_ORD_INT                    (30U)

/*! PMIC Thermal Warning Interrupt */
#define PMIC_TPS6594X_TWARN_INT                      (31U)
/*! PMIC External Clock Interrupt */
#define PMIC_TPS6594X_EXT_CLK_INT                    (32U)
/*! PMIC BIST PASS Interrupt */
#define PMIC_TPS6594X_BIST_PASS_INT                  (33U)

/*! PMIC First Supply Detection Interrupt */
#define PMIC_TPS6594X_FSD_INT                        (34U)
/*! PMIC RTC ALARM Interrupt */
#define PMIC_TPS6594X_RTC_ALARM_INT                  (35U)
/*! PMIC RTC TIMER Interrupt */
#define PMIC_TPS6594X_RTC_TIMER_INT                  (36U)
/*! PMIC ENABLE Interrupt */
#define PMIC_TPS6594X_ENABLE_INT                     (37U)
/*! PMIC NPWRON Startup Interrupt */
#define PMIC_TPS6594X_NPWRON_START_INT               (38U)

/*! PMIC GPIO PIN 8 Interrupt */
#define PMIC_TPS6594X_GPIO8_INT                      (39U)
/*! PMIC GPIO PIN 7 Interrupt */
#define PMIC_TPS6594X_GPIO7_INT                      (40U)
/*! PMIC GPIO PIN 6 Interrupt */
#define PMIC_TPS6594X_GPIO6_INT                      (41U)
/*! PMIC GPIO PIN 5 Interrupt */
#define PMIC_TPS6594X_GPIO5_INT                      (42U)
/*! PMIC GPIO PIN 4 Interrupt */
#define PMIC_TPS6594X_GPIO4_INT                      (43U)
/*! PMIC GPIO PIN 3 Interrupt */
#define PMIC_TPS6594X_GPIO3_INT                      (44U)
/*! PMIC GPIO PIN 2 Interrupt */
#define PMIC_TPS6594X_GPIO2_INT                      (45U)
/*! PMIC GPIO PIN 1 Interrupt */
#define PMIC_TPS6594X_GPIO1_INT                      (46U)
/*! PMIC GPIO PIN 11 Interrupt */
#define PMIC_TPS6594X_GPIO11_INT                     (47U)
/*! PMIC GPIO PIN 10 Interrupt */
#define PMIC_TPS6594X_GPIO10_INT                     (48U)
/*! PMIC GPIO PIN 9 Interrupt */
#define PMIC_TPS6594X_GPIO9_INT                      (49U)

/*! PMIC VCCA Under-Voltage Interrupt */
#define PMIC_TPS6594X_VCCA_UV_INT                    (50U)
/*! PMIC VCCA Over-Voltage Interrupt */
#define PMIC_TPS6594X_VCCA_OV_INT                    (51U)

/*! PMIC LDO4 Current Limit Interrupt */
#define PMIC_TPS6594X_LDO4_ILIM_INT                  (52U)
/*! PMIC LDO4 SC Interrupt */
#define PMIC_TPS6594X_LDO4_SC_INT                    (53U)
/*! PMIC LDO4 Under-Voltage Interrupt */
#define PMIC_TPS6594X_LDO4_UV_INT                    (54U)
/*! PMIC LDO4 Over-Voltage Interrupt */
#define PMIC_TPS6594X_LDO4_OV_INT                    (55U)

/*! PMIC LDO3 Current Limit Interrupt */
#define PMIC_TPS6594X_LDO3_ILIM_INT                  (56U)
/*! PMIC LDO3 SC Interrupt */
#define PMIC_TPS6594X_LDO3_SC_INT                    (57U)
/*! PMIC LDO3 Under-Voltage Interrupt */
#define PMIC_TPS6594X_LDO3_UV_INT                    (58U)
/*! PMIC LDO3 Over-Voltage Interrupt */
#define PMIC_TPS6594X_LDO3_OV_INT                    (59U)

/*! PMIC LDO2 Current Limit Interrupt */
#define PMIC_TPS6594X_LDO2_ILIM_INT                  (60U)
/*! PMIC LDO2 SC Interrupt */
#define PMIC_TPS6594X_LDO2_SC_INT                    (61U)
/*! PMIC LDO2 Under-Voltage Interrupt */
#define PMIC_TPS6594X_LDO2_UV_INT                    (62U)
/*! PMIC LDO2 Over-Voltage Interrupt */
#define PMIC_TPS6594X_LDO2_OV_INT                    (63U)

/*! PMIC LDO1 Current Limit Interrupt */
#define PMIC_TPS6594X_LDO1_ILIM_INT                  (64U)
/*! PMIC LDO1 SC Interrupt */
#define PMIC_TPS6594X_LDO1_SC_INT                    (65U)
/*! PMIC LDO1 Under-Voltage Interrupt */
#define PMIC_TPS6594X_LDO1_UV_INT                    (66U)
/*! PMIC LDO1 Over-Voltage Interrupt */
#define PMIC_TPS6594X_LDO1_OV_INT                    (67U)

/*! PMIC BUCK5 Current Limit Interrupt */
#define PMIC_TPS6594X_BUCK5_ILIM_INT                 (68U)
/*! PMIC BUCK5 SC Interrupt */
#define PMIC_TPS6594X_BUCK5_SC_INT                   (69U)
/*! PMIC BUCK5 Under-Voltage Interrupt */
#define PMIC_TPS6594X_BUCK5_UV_INT                   (70U)
/*! PMIC BUCK5 Over-Voltage Interrupt */
#define PMIC_TPS6594X_BUCK5_OV_INT                   (71U)

/*! PMIC BUCK4 Current Limit Interrupt */
#define PMIC_TPS6594X_BUCK4_ILIM_INT                 (72U)
/*! PMIC BUCK4 SC Interrupt */
#define PMIC_TPS6594X_BUCK4_SC_INT                   (73U)
/*! PMIC BUCK4 Under-Voltage Interrupt */
#define PMIC_TPS6594X_BUCK4_UV_INT                   (74U)
/*! PMIC BUCK4 Over-Voltage Interrupt */
#define PMIC_TPS6594X_BUCK4_OV_INT                   (75U)

/*! PMIC BUCK3 Current Limit Interrupt */
#define PMIC_TPS6594X_BUCK3_ILIM_INT                 (76U)
/*! PMIC BUCK3 SC Interrupt */
#define PMIC_TPS6594X_BUCK3_SC_INT                   (77U)
/*! PMIC BUCK3 Under-Voltage Interrupt */
#define PMIC_TPS6594X_BUCK3_UV_INT                   (78U)
/*! PMIC BUCK3 Over-Voltage Interrupt */
#define PMIC_TPS6594X_BUCK3_OV_INT                   (79U)

/*! PMIC BUCK2 Current Limit Interrupt */
#define PMIC_TPS6594X_BUCK2_ILIM_INT                 (80U)
/*! PMIC BUCK2 SC Interrupt */
#define PMIC_TPS6594X_BUCK2_SC_INT                   (81U)
/*! PMIC BUCK2 Under-Voltage Interrupt */
#define PMIC_TPS6594X_BUCK2_UV_INT                   (82U)
/*! PMIC BUCK2 Over-Voltage Interrupt */
#define PMIC_TPS6594X_BUCK2_OV_INT                   (83U)

/*! PMIC BUCK1 Current Limit Interrupt */
#define PMIC_TPS6594X_BUCK1_ILIM_INT                 (84U)
/*! PMIC BUCK1 SC Interrupt */
#define PMIC_TPS6594X_BUCK1_SC_INT                   (85U)
/*! PMIC BUCK1 Under-Voltage Interrupt */
#define PMIC_TPS6594X_BUCK1_UV_INT                   (86U)
/*! PMIC BUCK1 Over-Voltage Interrupt */
#define PMIC_TPS6594X_BUCK1_OV_INT                   (87U)

/*! PMIC SOFT REBOOT Startup Interrupt */
#define PMIC_TPS6594X_SOFT_REBOOT_INT                (88U)

/*! PMIC Max Interrupt Number on PG1.0 */
#define PMIC_TPS6594X_IRQ_MAX_NUM_PG_1_0             (88U)
/*! PMIC Max Interrupt Number on PG2.0 */
#define PMIC_TPS6594X_IRQ_MAX_NUM_PG_2_0             (89U)

/* @} */

/**
 *  \anchor Pmic_tps6594x_IrqGpioNum
 *  \name PMIC GPIO Interrupt Mask values for tps6594x
 *
 *  @{
 */
#define PMIC_TPS6594X_IRQ_GPIO_1_INT_MASK_NUM     (0U)
#define PMIC_TPS6594X_IRQ_GPIO_2_INT_MASK_NUM     (1U)
#define PMIC_TPS6594X_IRQ_GPIO_3_INT_MASK_NUM     (2U)
#define PMIC_TPS6594X_IRQ_GPIO_4_INT_MASK_NUM     (3U)
#define PMIC_TPS6594X_IRQ_GPIO_5_INT_MASK_NUM     (4U)
#define PMIC_TPS6594X_IRQ_GPIO_6_INT_MASK_NUM     (5U)
#define PMIC_TPS6594X_IRQ_GPIO_7_INT_MASK_NUM     (6U)
#define PMIC_TPS6594X_IRQ_GPIO_8_INT_MASK_NUM     (7U)
#define PMIC_TPS6594X_IRQ_GPIO_9_INT_MASK_NUM     (8U)
#define PMIC_TPS6594X_IRQ_GPIO_10_INT_MASK_NUM    (9U)
#define PMIC_TPS6594X_IRQ_GPIO_11_INT_MASK_NUM    (10U)

/* @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif/* PMIC_IRQ_TPS6594X_H_ */

/* @} */
