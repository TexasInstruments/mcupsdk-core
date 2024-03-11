/******************************************************************************
 * Copyright (c) 2024 Texas Instruments Incorporated - http://www.ti.com
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
 * @file   pmic_irq.h
 *
 * @brief  PMIC IRQ Driver API/interface file.
 */

#ifndef PMIC_IRQ_H_
#define PMIC_IRQ_H_

/* ==========================================================================*/
/*                             Include Files                                 */
/* ==========================================================================*/

#include "pmic_core.h"
#include "pmic_core_priv.h"
#include "pmic_irq_priv.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup Pmic_IRQ PMIC Interrupt Request
 * @{
 * @brief Contains definitions related to PMIC IRQ functionality.
 */

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @defgroup Pmic_IRQMacros PMIC Interrupt Request Macros
 * @{
 * @ingroup Pmic_IRQ
 * @brief Contains macros used in the IRQ module of PMIC driver.
 */

/**
 * @brief Number of interrupt masks available for general purpose outputs (GPOs) in the PMIC.
 * This constant defines the total number of interrupt masks allocated for GPOs in the PMIC
 * interrupt configuration.
 * @ingroup Pmic_IRQMacros
 */
#define PMIC_IRQ_GPO_ALL_INT_MASK_NUM   (5U)

/**
 * @brief Mask value indicating all interrupts.
 * This constant represents the mask value used to enable or disable all interrupts in the PMIC.
 * @ingroup Pmic_IRQMacros
 */
#define PMIC_IRQ_ALL                    (0xFFU)

/**
 * @brief Value indicating no interrupts to clear.
 * This constant indicates that no interrupts are to be cleared when performing a clear operation.
 * @ingroup Pmic_IRQMacros
 */
#define PMIC_IRQ_CLEAR_NONE             (0U)

/**
 * @brief Value indicating interrupts to clear.
 * This constant indicates that interrupts are to be cleared when performing a clear operation.
 * @ingroup Pmic_IRQMacros
 */
#define PMIC_IRQ_CLEAR                  (1U)

/**
 * @brief Value indicating unmasking of interrupts.
 * This constant represents the value used to unmask interrupts in the PMIC interrupt configuration.
 * @ingroup Pmic_IRQMacros
 */
#define PMIC_IRQ_UNMASK                 ((bool) -1)

/**
 * @brief Value indicating masking of interrupts.
 * This constant represents the value used to mask interrupts in the PMIC interrupt configuration.
 * @ingroup Pmic_IRQMacros
 */
#define PMIC_IRQ_MASK                   ((bool) 0)

/**
 * @brief Type of interrupt for GPIOs.
 * This constant defines the type of interrupt associated with GPIOs in the PMIC interrupt configuration.
 * @ingroup Pmic_IRQMacros
 */
#define PMIC_IRQ_GPIO_INT_TYPE          (0x0U)

/**
 * @}
 */
/* End of Pmic_IRQMacros */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/**
 * @defgroup Pmic_IRQStructures PMIC IRQ Structures
 * @{
 * @ingroup Pmic_IRQ
 * @brief Contains structures used in the IRQ module of PMIC driver.
 */

/**
 * @brief Structure for storing PMIC interrupt status.
 *
 * @param intStatus Array to store interrupt status.
 *
 * @ingroup Pmic_IRQStructures
 */
typedef struct Pmic_IrqStatus_s {
    uint32_t intStatus[4];
}
Pmic_IrqStatus_t;

/**
 * @}
 */
/* End of Pmic_IRQStructures */

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/**
 * @defgroup Pmic_IRQFunctions PMIC Interrupt Request Functions
 * @{
 * @ingroup Pmic_IRQ
 * @brief Contains functions used in the IRQ module of PMIC driver.
 */


/**
 * @brief Set a specific bit in the interrupt status.
 * This function sets a specific bit in the interrupt status based on the
 * position provided.
 *
 * @param pErrStat Pointer to the interrupt status.
 * @param pos Position of the bit to set.
 * @return void
 *
 * @ingroup Pmic_IRQFunctions
 */
void Pmic_intrBitSet(Pmic_IrqStatus_t * pErrStat, uint32_t pos);

/**
 * @brief Clear a specific bit in the interrupt status.
 * This function clears a specific bit in the interrupt status based on the IRQ
 * number provided.
 *
 * @param pErrStat Pointer to the interrupt status.
 * @param pIrqNum Pointer to the IRQ number.
 * @return void
 *
 * @ingroup Pmic_IRQFunctions
 */
static void Pmic_intrBitClear(Pmic_IrqStatus_t * pErrStat,
                              const uint8_t * pIrqNum);

/**
 * @brief Extract the IRQ number from the interrupt status.
 * This function extracts the IRQ number from the interrupt status up to a
 * maximum value.
 *
 * @param pErrStat Pointer to the interrupt status.
 * @param maxVal Maximum value for the IRQ number.
 * @return irqNum Extracted IRQ number.
 *
 * @ingroup Pmic_IRQFunctions
 */
static uint8_t Pmic_intrBitExtract(const Pmic_IrqStatus_t * pErrStat,
                                   uint8_t maxVal);

/**
 * @brief Validate the IRQ number.
 * This function validates the IRQ number based on the PMIC device type.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param irqNum IRQ number to validate.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the IRQ number is valid;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_IRQFunctions
 */
static int32_t Pmic_irqValidateIrqNum(const Pmic_CoreHandle_t * pPmicCoreHandle,
                                      const uint8_t irqNum);

/**
 * @brief Validate the IRQ number for getting the mask interrupt status.
 * This function validates the IRQ number for getting the mask interrupt status
 * based on the PMIC device type.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param irqNum IRQ number to validate.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the IRQ number is valid;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_IRQFunctions
 */
static int32_t Pmic_irqValidateIrqNumGetMaskIntrStatus(
        const Pmic_CoreHandle_t * pPmicCoreHandle,
        const uint8_t irqNum);

/**
 * @brief Get the maximum value for IRQ.
 * This function retrieves the maximum value for IRQ based on the PMIC device
 * type.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param maxVal Pointer to store the maximum value for IRQ.
 * @return void
 *
 * @ingroup Pmic_IRQFunctions
 */
void Pmic_getMaxVal(uint8_t * maxVal);

/**
 * @brief Get the interrupt configuration.
 * This function retrieves the interrupt configuration based on the PMIC device
 * type.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pIntrCfg Pointer to store the interrupt configuration.
 * @return void
 *
 * @ingroup Pmic_IRQFunctions
 */
static void Pmic_get_intrCfg(const Pmic_CoreHandle_t * pPmicCoreHandle,
                             Pmic_IntrCfg_t ** pIntrCfg);

/**
 * @brief Get the GPIO interrupt configuration.
 * This function retrieves the GPIO interrupt configuration based on the PMIC
 * device type.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pGpioIntrCfg Pointer to store the GPIO interrupt configuration.
 * @return void
 *
 * @ingroup Pmic_IRQFunctions
 */
static void Pmic_get_gpioIntrCfg(Pmic_GpioIntrTypeCfg_t ** pGpioIntrCfg);

/**
 * @brief Mask or unmask the GPIO interrupt.
 * This function masks or unmasks the GPIO interrupt based on the provided
 * parameters.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param irqGpioNum IRQ number for GPIO interrupt.
 * @param mask Boolean value indicating whether to mask or unmask the interrupt.
 * @param gpioIntrType GPIO interrupt type.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_IRQFunctions
 */
static int32_t Pmic_irqGpioMask(Pmic_CoreHandle_t * pPmicCoreHandle,
                                const uint8_t irqGpioNum,
                                const bool mask,
                                const uint8_t gpioIntrType);

/**
 * @brief Mask or unmask all GPIO interrupts.
 * This function masks or unmasks all GPIO interrupts based on the provided
 * parameters.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param irqGpioNum IRQ number for GPIO interrupt.
 * @param mask Boolean value indicating whether to mask or unmask the
 * interrupts.
 * @param gpioIntrType GPIO interrupt type.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_IRQFunctions
 */
int32_t Pmic_maskGpioIntr(Pmic_CoreHandle_t * pPmicCoreHandle,
                                 const uint8_t irqGpioNum,
                                 const bool mask,
                                 const uint8_t gpioIntrType);

/**
 * @brief Clear the interrupt status.
 * This function clears the interrupt status based on the provided IRQ number.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param irqNum IRQ number to clear.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_IRQFunctions
 */
static int32_t Pmic_irqClear(Pmic_CoreHandle_t * pPmicCoreHandle,
                             const uint8_t irqNum);

/**
 * @brief Clear all interrupt statuses.
 * This function clears all interrupt statuses based on the provided IRQ number.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param irqNum IRQ number to clear.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_IRQFunctions
 */
static int32_t Pmic_irqClearStatus(Pmic_CoreHandle_t * pPmicCoreHandle,
                                   const uint8_t irqNum);

/**
 * @brief Mask or unmask a specific IRQ.
 * This function masks or unmasks a specific IRQ based on the provided
 * parameters.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param irqNum IRQ number to mask or unmask.
 * @param mask Boolean value indicating whether to mask or unmask the IRQ.
 * @param pIntrCfg Pointer to the interrupt configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_IRQFunctions
 */
static int32_t Pmic_irqMask(Pmic_CoreHandle_t * pPmicCoreHandle,
                            const bool mask,
                            const Pmic_IntrCfg_t * pIntrCfg);

/**
 * @brief Mask or unmask interrupts.
 * This function masks or unmasks interrupts based on the provided parameters.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param irqNum IRQ number to mask or unmask.
 * @param mask Boolean value indicating whether to mask or unmask the
 * interrupts.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_IRQFunctions
 */
static int32_t Pmic_maskIntr(Pmic_CoreHandle_t * pPmicCoreHandle,
                             const uint8_t irqNum,
                             const bool mask);

/**
 * @brief Get the L1 register address for IRQ.
 * This function retrieves the L1 register address for IRQ based on the count.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param l1RegAddr Pointer to store the L1 register address.
 * @param count Count to determine the L1 register address.
 * @return void
 *
 * @ingroup Pmic_IRQFunctions
 */
static void Pmic_irqGetL1Reg(uint16_t * l1RegAddr, uint8_t count);

/**
 * @brief Get the L2 error for the specified L1 register address.
 * This function retrieves the L2 error for the specified L1 register address.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param l1RegAddr L1 register address.
 * @param pErrStat Pointer to store the error status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_IRQFunctions
 */
static int32_t Pmic_irqGetL2Error(Pmic_CoreHandle_t * pPmicCoreHandle,
                                  uint16_t l1RegAddr,
                                  Pmic_IrqStatus_t * pErrStat);

/**
 * @brief Extract the error status from the PMIC.
 * This function extracts the error status from the PMIC and clears the error
 * bit position.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pErrStat Pointer to the error status structure.
 * @param pIrqNum Pointer to store the IRQ number.
 * @return void
 *
 * @ingroup Pmic_IRQFunctions
 */
static void Pmic_extractErrStatus(Pmic_IrqStatus_t * pErrStat,
                                  uint8_t * pIrqNum);

/**
 * @brief Get the error status from the PMIC.
 * This function retrieves the error status from the PMIC and optionally clears
 * the IRQ.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pErrStat Pointer to store the error status.
 * @param clearIRQ Boolean value indicating whether to clear the IRQ.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_IRQFunctions
 */
int32_t Pmic_irqGetErrStatus(Pmic_CoreHandle_t * pPmicCoreHandle,
                             Pmic_IrqStatus_t * pErrStat,
                             const bool clearIRQ);

/**
 * @brief Clear the error status of the specified IRQ.
 * This function clears the error status of the specified IRQ.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param irqNum IRQ number to clear the error status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_IRQFunctions
 */
int32_t Pmic_irqClrErrStatus(Pmic_CoreHandle_t * pPmicCoreHandle,
                             const uint8_t irqNum);

/**
 * @brief Mask or unmask a specific interrupt.
 * This function masks or unmasks a specific interrupt based on the provided
 * parameters.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param irqNum IRQ number to mask or unmask.
 * @param mask Boolean value indicating whether to mask or unmask the interrupt.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_IRQFunctions
 */
int32_t Pmic_irqMaskIntr(Pmic_CoreHandle_t * pPmicCoreHandle,
                         const uint8_t irqNum,
                         const bool mask);

/**
 * @brief Get the next error status from the PMIC.
 * This function retrieves the next error status from the PMIC.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pErrStat Pointer to store the error status.
 * @param pIrqNum Pointer to store the IRQ number associated with the error.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_IRQFunctions
 */
int32_t Pmic_getNextErrorStatus(const Pmic_CoreHandle_t * pPmicCoreHandle,
                                Pmic_IrqStatus_t * pErrStat, uint8_t * pIrqNum);

/**
 * @brief Mask or unmask a specific GPIO interrupt.
 * This function masks or unmasks a specific GPIO interrupt based on the
 * provided parameters.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param irqGpioNum IRQ GPIO number to mask or unmask.
 * @param mask Boolean value indicating whether to mask or unmask the interrupt.
 * @param gpioIntrType Type of GPIO interrupt.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_IRQFunctions
 */
int32_t Pmic_irqGpioMaskIntr(Pmic_CoreHandle_t * pPmicCoreHandle,
                             const uint8_t irqGpioNum,
                             const bool mask,
                             const uint8_t gpioIntrType);

/**
 * @brief Get the mask status of a specific IRQ.
 * This function retrieves the mask status of a specific IRQ based on the
 * provided parameters.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param irqNum IRQ number to get the mask status.
 * @param pMaskStatus Pointer to store the mask status.
 * @param pIntrCfg Pointer to the interrupt configuration.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_IRQFunctions
 */
static int32_t Pmic_getIrqMaskStatus(Pmic_CoreHandle_t * pPmicCoreHandle,
                                     bool * pMaskStatus,
                                     const Pmic_IntrCfg_t * pIntrCfg);

/**
 * @brief Get the mask status of a specific interrupt.
 * This function retrieves the mask status of a specific interrupt based on the
 * provided parameters.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param irqNum IRQ number to get the mask status.
 * @param pMaskStatus Pointer to store the mask status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_IRQFunctions
 */
static int32_t Pmic_getMaskIntrStatus(Pmic_CoreHandle_t * pPmicCoreHandle,
                                      bool * pMaskStatus);

/**
 * @brief Get the mask status of a specific IRQ.
 * This function retrieves the mask status of a specific IRQ based on the
 * provided parameters.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param irqNum IRQ number to get the mask status.
 * @param pMaskStatus Pointer to store the mask status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_IRQFunctions
 */
int32_t Pmic_irqGetMaskIntrStatus(Pmic_CoreHandle_t * pPmicCoreHandle,
                                  const uint8_t irqNum, bool * pMaskStatus);

/**
 * @brief Get the mask status of a specific GPIO interrupt.
 * This function retrieves the mask status of a specific GPIO interrupt based on
 * the provided parameters.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param irqGpioNum IRQ GPIO number to get the mask status.
 * @param gpioIntrType Type of GPIO interrupt.
 * @param pIntrMaskStat Pointer to store the mask status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_IRQFunctions
 */
static int32_t Pmic_getIrqGpioMaskStatus(Pmic_CoreHandle_t * pPmicCoreHandle,
                                         const uint8_t irqGpioNum,
                                         const uint8_t gpioIntrType,
                                         bool * pIntrMaskStat);

/**
 * @brief Get the mask status of a specific IRQ.
 * This function retrieves the mask status of a specific IRQ.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param irqNum IRQ number to get the mask status.
 * @param pMaskStatus Pointer to store the mask status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_IRQFunctions
 */
static int32_t Pmic_getMaskGpioIntrStatus(Pmic_CoreHandle_t * pPmicCoreHandle,
                                          const uint8_t irqGpioNum,
                                          const uint8_t gpioIntrType,
                                          bool * pIntrMaskStat);

/**
 * @brief Get the mask status of a specific GPIO interrupt.
 * This function retrieves the mask status of a specific GPIO interrupt.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param irqGpioNum IRQ GPIO number to get the mask status.
 * @param gpioIntrType Type of GPIO interrupt.
 * @param pIntrMaskStat Pointer to store the mask status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_IRQFunctions
 */
int32_t Pmic_irqGetGpioMaskIntr(Pmic_CoreHandle_t * pPmicCoreHandle,
                                const uint8_t irqGpioNum,
                                const uint8_t gpioIntrType,
                                bool * pIntrMaskStat);


/**
 * @}
 */
/* End of Pmic_IRQFunctions */

/**
 * @}
 */
/* End of Pmic_IRQ */

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* PMIC_IRQ_H_ */

