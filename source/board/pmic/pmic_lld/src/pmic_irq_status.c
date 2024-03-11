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
 * @file   pmic_irq_status.c
 *
 * @brief  This file contains APIs definitions for PMIC Interrupt IRQ Status
 *         Handler.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pmic_irq.h"
#include "pmic_irq_tps65386x.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 * @brief Set a specific bit in the interrupt status.
 * This function sets a specific bit in the interrupt status based on the
 * position provided.
 *
 * @param pErrStat Pointer to the interrupt status.
 * @param pos Position of the bit to set.
 * @return void
 */
void Pmic_intrBitSet(Pmic_IrqStatus_t * pErrStat, uint32_t pos) {
    uint32_t intStatSize = 0U;

    /* Size of intStatus in bits */
    intStatSize = sizeof(pErrStat -> intStatus[0U]) << 3U;

    pErrStat -> intStatus[pos / intStatSize] |= (uint32_t) 1U << (pos % intStatSize);
}

/**
 * @brief Clear a specific bit in the interrupt status.
 * This function clears a specific bit in the interrupt status based on the IRQ
 * number provided.
 *
 * @param pErrStat Pointer to the interrupt status.
 * @param pIrqNum Pointer to the IRQ number.
 * @return void
 */
static void Pmic_intrBitClear(Pmic_IrqStatus_t * pErrStat,
                              const uint8_t * pIrqNum) {
    uint32_t intStatSize = 0U;

    /* Size of intStatus in bits */
    intStatSize = sizeof(pErrStat -> intStatus[0U]) << 3U;

    pErrStat -> intStatus[ * pIrqNum / intStatSize] &=
        ~((uint32_t) 1U << ( * pIrqNum % intStatSize));
}

/**
 * @brief Extract the IRQ number from the interrupt status.
 * This function extracts the IRQ number from the interrupt status up to a
 * maximum value.
 *
 * @param pErrStat Pointer to the interrupt status.
 * @param maxVal Maximum value for the IRQ number.
 * @return irqNum Extracted IRQ number.
 */
static uint8_t Pmic_intrBitExtract(const Pmic_IrqStatus_t * pErrStat,
                                   uint8_t maxVal) {
    uint8_t irqNum = 0U;
    uint32_t intStatSize = 0U;

    /* Size of intStatus in bits */
    intStatSize = sizeof(pErrStat -> intStatus[0U]) << 3U;
    for (irqNum = 0U; irqNum < maxVal; irqNum++) {
        if (((pErrStat -> intStatus[irqNum / intStatSize]) &
                ((uint32_t) 1U << (irqNum % intStatSize))) != 0U) {
            break;
        }
    }

    return irqNum;
}

/**
 * @brief Validate the IRQ number.
 * This function validates the IRQ number based on the PMIC device type.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param irqNum IRQ number to validate.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the IRQ number is valid;
 * otherwise, returns an error code.
 */
static int32_t Pmic_irqValidateIrqNum(const Pmic_CoreHandle_t * pPmicCoreHandle,
                                      const uint8_t irqNum) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t maxVal;

    if ((pPmicCoreHandle -> pmicDeviceType) == PMIC_DEV_BB_TPS65386X) {
        maxVal = PMIC_BB_IRQ_MAX_NUM;
        if ((irqNum > maxVal) && (irqNum != PMIC_IRQ_ALL)) {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
        }
    }

    return pmicStatus;
}

/**
 * @brief Validate the IRQ number for getting the mask interrupt status.
 * This function validates the IRQ number for getting the mask interrupt status
 * based on the PMIC device type.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param irqNum IRQ number to validate.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the IRQ number is valid;
 * otherwise, returns an error code.
 */
static int32_t Pmic_irqValidateIrqNumGetMaskIntrStatus(
        const Pmic_CoreHandle_t * pPmicCoreHandle,
        const uint8_t irqNum) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t maxVal;

    if ((pPmicCoreHandle -> pmicDeviceType) == PMIC_DEV_BB_TPS65386X) {
        maxVal = PMIC_BB_IRQ_MAX_NUM;
        if (irqNum > maxVal) {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
        }
    }

    return pmicStatus;
}

/**
 * @brief Get the maximum value for IRQ.
 * This function retrieves the maximum value for IRQ based on the PMIC device
 * type.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param maxVal Pointer to store the maximum value for IRQ.
 * @return void
 */
void Pmic_getMaxVal(uint8_t * maxVal) {
    * maxVal = PMIC_BB_IRQ_MAX_NUM;
}

/**
 * @brief Get the interrupt configuration.
 * This function retrieves the interrupt configuration based on the PMIC device
 * type.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pIntrCfg Pointer to store the interrupt configuration.
 * @return void
 */
static void Pmic_get_intrCfg(const Pmic_CoreHandle_t * pPmicCoreHandle,
                             Pmic_IntrCfg_t ** pIntrCfg) {
    if ((pPmicCoreHandle -> pmicDeviceType) == PMIC_DEV_BB_TPS65386X) {
        pmic_get_bb_intrCfg(pIntrCfg);
    }
}

/**
 * @brief Get the GPIO interrupt configuration.
 * This function retrieves the GPIO interrupt configuration based on the PMIC
 * device type.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pGpioIntrCfg Pointer to store the GPIO interrupt configuration.
 * @return void
 */
static void Pmic_get_gpioIntrCfg(Pmic_GpioIntrTypeCfg_t ** pGpioIntrCfg) {
    pmic_get_bb_intrGpioCfg(pGpioIntrCfg);
}

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
 */
static int32_t Pmic_irqGpioMask(Pmic_CoreHandle_t * pPmicCoreHandle,
                                const uint8_t irqGpioNum,
                                const bool mask,
                                const uint8_t gpioIntrType) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    Pmic_GpioIntrTypeCfg_t * pGpioIntrCfg[PMIC_IRQ_GPO_ALL_INT_MASK_NUM];
    uint8_t bitMask = 0U;
    uint8_t maskVal = 0U;

    Pmic_get_gpioIntrCfg(pGpioIntrCfg);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    if (PMIC_IRQ_GPIO_INT_TYPE == gpioIntrType) {
        pmicStatus = Pmic_commIntf_recvByte(
            pPmicCoreHandle,
            (uint16_t) pGpioIntrCfg[irqGpioNum] -> gpioIntrMaskRegAddr, & regData);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            if (true == mask) {
                maskVal = 1U;
            }
            bitMask = (uint8_t)(PMIC_IRQ_MASK_CLR_BITFIELD <<
                pGpioIntrCfg[irqGpioNum] -> gpioMaskBitPos);
            Pmic_setBitField( & regData, pGpioIntrCfg[irqGpioNum] -> gpioMaskBitPos,
                bitMask, maskVal);
            pmicStatus = Pmic_commIntf_sendByte(
                pPmicCoreHandle,
                (uint16_t) pGpioIntrCfg[irqGpioNum] -> gpioIntrMaskRegAddr, regData);
        }
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

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
 */
int32_t Pmic_maskGpioIntr(Pmic_CoreHandle_t * pPmicCoreHandle,
                                 const uint8_t irqGpioNum,
                                 const bool mask,
                                 const uint8_t gpioIntrType) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t irqGpioId = 0U;

    if (PMIC_IRQ_GPO_ALL_INT_MASK_NUM != irqGpioNum) {
        pmicStatus =
            Pmic_irqGpioMask(pPmicCoreHandle, irqGpioNum, mask, gpioIntrType);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        (PMIC_IRQ_GPO_ALL_INT_MASK_NUM == irqGpioNum)) {
        for (irqGpioId = 0U; irqGpioId < (PMIC_IRQ_GPO_ALL_INT_MASK_NUM - 1U); irqGpioId++) {
            pmicStatus =
                Pmic_irqGpioMask(pPmicCoreHandle, irqGpioId, mask, gpioIntrType);
        }
    }

    return pmicStatus;
}

/**
 * @brief Clear the interrupt status.
 * This function clears the interrupt status based on the provided IRQ number.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param irqNum IRQ number to clear.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_irqClear(Pmic_CoreHandle_t * pPmicCoreHandle,
                             const uint8_t irqNum) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    Pmic_IntrCfg_t * pIntrCfg[PMIC_BB_IRQ_MAX_NUM];
    uint8_t bitMask = 0U;

    Pmic_get_intrCfg(pPmicCoreHandle, pIntrCfg);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(
        pPmicCoreHandle, pIntrCfg[irqNum] -> intrClrRegAddr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        bitMask = (uint8_t)(PMIC_IRQ_MASK_CLR_BITFIELD <<
            pIntrCfg[irqNum] -> intrClrBitPos);
        Pmic_setBitField( & regData, pIntrCfg[irqNum] -> intrClrBitPos, bitMask,
            PMIC_IRQ_CLEAR);
        pmicStatus = Pmic_commIntf_sendByte(
            pPmicCoreHandle, pIntrCfg[irqNum] -> intrClrRegAddr, regData);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/**
 * @brief Clear all interrupt statuses.
 * This function clears all interrupt statuses based on the provided IRQ number.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param irqNum IRQ number to clear.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_irqClearStatus(Pmic_CoreHandle_t * pPmicCoreHandle,
                                   const uint8_t irqNum) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t irqId = 0U;
    uint8_t maxVal = 0U;

    if (PMIC_IRQ_ALL != irqNum) {
        pmicStatus = Pmic_irqClear(pPmicCoreHandle, irqNum);
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (PMIC_IRQ_ALL == irqNum)) {
        Pmic_getMaxVal( & maxVal);

        for (irqId = 0U; irqId < maxVal; irqId++) {
            pmicStatus = Pmic_irqClear(pPmicCoreHandle, irqId);
        }
    }

    return pmicStatus;
}

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
 */
static int32_t Pmic_irqMask(Pmic_CoreHandle_t * pPmicCoreHandle,
                            const bool mask,
                            const Pmic_IntrCfg_t * pIntrCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t bitMask = 0U;
    uint8_t maskVal = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(
            pPmicCoreHandle, (uint16_t) pIntrCfg->intrMaskRegAddr, & regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        if (true == mask) {
            maskVal = 1U;
        }
        bitMask = (uint8_t)(PMIC_IRQ_MASK_CLR_BITFIELD <<
            pIntrCfg->intrMaskBitPos);
        Pmic_setBitField( & regData, pIntrCfg->intrMaskBitPos, bitMask,
            maskVal);
        pmicStatus = Pmic_commIntf_sendByte(
            pPmicCoreHandle, (uint16_t) pIntrCfg->intrMaskRegAddr, regData);
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

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
 */
static int32_t Pmic_maskIntr(Pmic_CoreHandle_t * pPmicCoreHandle,
                             const uint8_t irqNum,
                             const bool mask) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t irqId = 0U;
    uint8_t maxVal = 0U;
    /* Pmic_IntrCfg_t * pIntrCfg[PMIC_BB_IRQ_MAX_NUM]; */

    Pmic_IntrCfg_t *pIntrCfg = NULL;

    Pmic_get_intrCfg(pPmicCoreHandle, &pIntrCfg);

    if (PMIC_IRQ_ALL != irqNum) {
        if (PMIC_IRQ_INVALID_REGADDR == pIntrCfg->intrMaskRegAddr) {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }

        if (PMIC_ST_SUCCESS == pmicStatus) {
            pmicStatus = Pmic_irqMask(pPmicCoreHandle, mask, pIntrCfg);
        }
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (PMIC_IRQ_ALL == irqNum)) {
        Pmic_getMaxVal( & maxVal);
        for (irqId = 0U; irqId < maxVal; irqId++) {
            if (PMIC_IRQ_INVALID_REGADDR == pIntrCfg->intrMaskRegAddr) {
                pmicStatus = (int32_t) PMIC_IRQ_INVALID_REGADDR;
            }

            pmicStatus = Pmic_irqMask(pPmicCoreHandle, mask, pIntrCfg);
        }
    }

    return pmicStatus;
}

/**
 * @brief Get the L1 register address for IRQ.
 * This function retrieves the L1 register address for IRQ based on the count.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param l1RegAddr Pointer to store the L1 register address.
 * @param count Count to determine the L1 register address.
 * @return void
 */
static void Pmic_irqGetL1Reg(uint16_t * l1RegAddr, uint8_t count) {
    * l1RegAddr = PMIC_INT_UNUSED_REGADDR;

    switch ((uint8_t)(1U << count)) {
    case PMIC_SAFETY_OFF_STATE_CFG_MASK:
        *
        l1RegAddr = PMIC_SAFETY_CFG_REGADDR;
        break;

    case PMIC_RDBK_INT_CFG_MASK:
        *
        l1RegAddr = PMIC_RDBK_INT_CFG1_REGADDR;
        break;

    case PMIC_OV_INT_CFG_MASK:
        *
        l1RegAddr = PMIC_OV_INT_CFG1_REGADDR;
        break;

    case PMIC_UV_INT_CFG_MASK:
        *
        l1RegAddr = PMIC_UV_INT_CFG1_REGADDR;
        break;

    case PMIC_WD_ERR_STAT_MASK:
        *
        l1RegAddr = PMIC_WDG_INT_CFG_REGADDR;
        break;

    case PMIC_ESM_INT_CFG_MASK:
        *
        l1RegAddr = PMIC_ESM_INT_CFG_REGADDR;
        break;

    case PMIC_CM_COMP_INT_MASK_CFG_MASK:
        *
        l1RegAddr = PMIC_CM_COMP_INT_MSKCFG_REGADDR;
        break;

    case PMIC_CM_VMON_INT_CFG_MASK:
        *
        l1RegAddr = PMIC_CM_VMON_INT_CFG_REGADDR;
        break;

    default:
        break;
    }

    if (PMIC_INT_UNUSED_REGADDR == * l1RegAddr) {
        * l1RegAddr = 0U;
    }
}

/**
 * @brief Get the L2 error for the specified L1 register address.
 * This function retrieves the L2 error for the specified L1 register address.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param l1RegAddr L1 register address.
 * @param pErrStat Pointer to store the error status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_irqGetL2Error(Pmic_CoreHandle_t * pPmicCoreHandle,
                                  uint16_t l1RegAddr,
                                  Pmic_IrqStatus_t * pErrStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    /* Default case is valid only for TPS65386x BB PMIC */
    pmicStatus = Pmic_BB_irqGetL2Error(pPmicCoreHandle, l1RegAddr, pErrStat);

    return pmicStatus;
}

/**
 * @brief Extract the error status from the PMIC.
 * This function extracts the error status from the PMIC and clears the error
 * bit position.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pErrStat Pointer to the error status structure.
 * @param pIrqNum Pointer to store the IRQ number.
 * @return void
 */
static void Pmic_extractErrStatus(Pmic_IrqStatus_t * pErrStat,
                                  uint8_t * pIrqNum) {
    uint8_t maxVal = 0U;

    Pmic_getMaxVal( & maxVal);

    * pIrqNum = Pmic_intrBitExtract(pErrStat, maxVal);
    /* To clear the Error Bit position after extracting */
    Pmic_intrBitClear(pErrStat, pIrqNum);
}

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
 */
int32_t Pmic_irqGetErrStatus(Pmic_CoreHandle_t * pPmicCoreHandle,
                             Pmic_IrqStatus_t * pErrStat,
                             const bool clearIRQ) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint16_t l1RegAddr = 0U;
    uint8_t count = 0U;
    uint8_t clearIRQStat = 0U;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pErrStat)) {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        /* Clearing all Error Status structure members */
        pErrStat -> intStatus[0U] = 0U;
        pErrStat -> intStatus[1U] = 0U;
        pErrStat -> intStatus[2U] = 0U;
        pErrStat -> intStatus[3U] = 0U;

        if (PMIC_ST_SUCCESS == pmicStatus) {
            for (count = 7U;; count--) {
                l1RegAddr = 0U;
                Pmic_irqGetL1Reg( & l1RegAddr, count);
                if (0U != l1RegAddr) {
                    pmicStatus = Pmic_irqGetL2Error(pPmicCoreHandle, l1RegAddr, pErrStat);
                }
                if ((PMIC_ST_SUCCESS != pmicStatus) || (count == 0U)) {
                    break;
                }
            }
            if (true == clearIRQ) {
                clearIRQStat = 1U;
            }

            if (PMIC_IRQ_CLEAR == clearIRQStat) {
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle, PMIC_IRQ_ALL);
            }
        }
    }

    return pmicStatus;
}

/**
 * @brief Clear the error status of the specified IRQ.
 * This function clears the error status of the specified IRQ.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param irqNum IRQ number to clear the error status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_irqClrErrStatus(Pmic_CoreHandle_t * pPmicCoreHandle,
                             const uint8_t irqNum) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pmicStatus = Pmic_irqValidateIrqNum(pPmicCoreHandle, irqNum);
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pmicStatus = Pmic_irqClearStatus(pPmicCoreHandle, irqNum);
    }

    return pmicStatus;
}

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
 */
int32_t Pmic_irqMaskIntr(Pmic_CoreHandle_t * pPmicCoreHandle,
                         const uint8_t irqNum,
                         const bool mask) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pmicStatus = Pmic_irqValidateIrqNum(pPmicCoreHandle, irqNum);
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pmicStatus = Pmic_maskIntr(pPmicCoreHandle, irqNum, mask);
    }

    return pmicStatus;
}

/**
 * @brief Get the next error status from the PMIC.
 * This function retrieves the next error status from the PMIC.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pErrStat Pointer to store the error status.
 * @param pIrqNum Pointer to store the IRQ number associated with the error.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
int32_t Pmic_getNextErrorStatus(const Pmic_CoreHandle_t * pPmicCoreHandle,
                                Pmic_IrqStatus_t * pErrStat, uint8_t * pIrqNum) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pErrStat)) {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pIrqNum)) {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((pErrStat -> intStatus[0U] == 0U) && (pErrStat -> intStatus[1U] == 0U) &&
            (pErrStat -> intStatus[2U] == 0U) && (pErrStat -> intStatus[3U] == 0U))) {
        pmicStatus = PMIC_ST_ERR_INV_INT;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        Pmic_extractErrStatus(pErrStat, pIrqNum);
    }

    return pmicStatus;
}

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
 */
int32_t Pmic_irqGpioMaskIntr(Pmic_CoreHandle_t * pPmicCoreHandle,
                             const uint8_t irqGpioNum,
                             const bool mask,
                             const uint8_t gpioIntrType) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) &&
        ((irqGpioNum == PMIC_BB_IRQ_GPO_3_INT_MASK_NUM) &&
            (PMIC_DEV_BB_TPS65386X == pPmicCoreHandle -> pmicDeviceType))) {
        pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pmicStatus =
            Pmic_maskGpioIntr(pPmicCoreHandle, irqGpioNum, mask, gpioIntrType);
    }

    return pmicStatus;
}

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
 */
static int32_t Pmic_getIrqMaskStatus(Pmic_CoreHandle_t * pPmicCoreHandle,
                                     bool * pMaskStatus,
                                     const Pmic_IntrCfg_t * pIntrCfg) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t bitMask = 0U;

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(
        pPmicCoreHandle, (uint16_t) pIntrCfg->intrMaskRegAddr, & regData);

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        bitMask = (uint8_t)(PMIC_IRQ_MASK_CLR_BITFIELD <<
            pIntrCfg->intrMaskBitPos);
        * pMaskStatus = PMIC_IRQ_UNMASK;

        if ((Pmic_getBitField(regData, pIntrCfg->intrMaskBitPos,
                bitMask)) == PMIC_IRQ_MASK_VAL_1) {
            * pMaskStatus = PMIC_IRQ_MASK;
        }
    }
    return pmicStatus;
}

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
 */
static int32_t Pmic_getMaskIntrStatus(Pmic_CoreHandle_t * pPmicCoreHandle,
                                      bool * pMaskStatus) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    /* Pmic_IntrCfg_t * pIntrCfg[PMIC_BB_IRQ_MAX_NUM]; */

    Pmic_IntrCfg_t *pIntrCfg = NULL;

    Pmic_get_intrCfg(pPmicCoreHandle, &pIntrCfg);

    if (PMIC_IRQ_INVALID_REGADDR == pIntrCfg->intrMaskRegAddr) {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pmicStatus =
            Pmic_getIrqMaskStatus(pPmicCoreHandle, pMaskStatus, pIntrCfg);
    }

    return pmicStatus;
}

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
 */
int32_t Pmic_irqGetMaskIntrStatus(Pmic_CoreHandle_t * pPmicCoreHandle,
                                  const uint8_t irqNum, bool * pMaskStatus) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pMaskStatus)) {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pmicStatus =
            Pmic_irqValidateIrqNumGetMaskIntrStatus(pPmicCoreHandle, irqNum);
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pmicStatus = Pmic_getMaskIntrStatus(pPmicCoreHandle, pMaskStatus);
    }

    return pmicStatus;
}

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
 */
static int32_t Pmic_getIrqGpioMaskStatus(Pmic_CoreHandle_t * pPmicCoreHandle,
                                         const uint8_t irqGpioNum,
                                         const uint8_t gpioIntrType,
                                         bool * pIntrMaskStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    Pmic_GpioIntrTypeCfg_t * pGpioIntrCfg[PMIC_IRQ_GPO_ALL_INT_MASK_NUM];
    uint8_t bitMask = 0U;

    Pmic_get_gpioIntrCfg(pGpioIntrCfg);

    /* Start Critical Section */
    Pmic_criticalSectionStart(pPmicCoreHandle);

    if (PMIC_IRQ_GPIO_INT_TYPE == gpioIntrType) {
        pmicStatus = Pmic_commIntf_recvByte(
            pPmicCoreHandle,
            (uint16_t) pGpioIntrCfg[irqGpioNum] -> gpioIntrMaskRegAddr, & regData);

        if (PMIC_ST_SUCCESS == pmicStatus) {
            bitMask = (uint8_t)(PMIC_IRQ_MASK_CLR_BITFIELD <<
                pGpioIntrCfg[irqGpioNum] -> gpioMaskBitPos);
            * pIntrMaskStat = PMIC_IRQ_UNMASK;

            if ((Pmic_getBitField(regData, pGpioIntrCfg[irqGpioNum] -> gpioMaskBitPos,
                    bitMask)) == PMIC_IRQ_MASK_VAL_1) {
                * pIntrMaskStat = PMIC_IRQ_MASK;
            }
        }
    }

    /* Stop Critical Section */
    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/**
 * @brief Get the mask status of a specific IRQ.
 * This function retrieves the mask status of a specific IRQ.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param irqNum IRQ number to get the mask status.
 * @param pMaskStatus Pointer to store the mask status.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_getMaskGpioIntrStatus(Pmic_CoreHandle_t * pPmicCoreHandle,
                                          const uint8_t irqGpioNum,
                                          const uint8_t gpioIntrType,
                                          bool * pIntrMaskStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    pmicStatus = Pmic_getIrqGpioMaskStatus(pPmicCoreHandle, irqGpioNum,
        gpioIntrType, pIntrMaskStat);

    return pmicStatus;
}

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
 */
int32_t Pmic_irqGetGpioMaskIntr(Pmic_CoreHandle_t * pPmicCoreHandle,
                                const uint8_t irqGpioNum,
                                const uint8_t gpioIntrType,
                                bool * pIntrMaskStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    if (NULL == pPmicCoreHandle) {
        pmicStatus = PMIC_ST_ERR_INV_HANDLE;
    }

    if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pIntrMaskStat)) {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pmicStatus = Pmic_getMaskGpioIntrStatus(pPmicCoreHandle, irqGpioNum,
            gpioIntrType, pIntrMaskStat);
    }

    return pmicStatus;
}
