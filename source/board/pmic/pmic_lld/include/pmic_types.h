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
 *  @file pmic_types.h
 *
 *  @brief PMIC Driver Common data types file.
 */

#ifndef PMIC_TYPES_H_
#define PMIC_TYPES_H_

/* ========================================================================= */
/*                             Include Files                                 */
/* ========================================================================= */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif
/* ========================================================================= */
/*                             Macros & Typedefs                             */
/* ========================================================================= */

/**
 *  \anchor Pmic_CrcEnableCfg
 *  \name PMIC CRC Enable/Disable Configuration
 *
 *  @{
 */
#define PMIC_CRC_DISABLE    (0U)
#define PMIC_CRC_ENABLE     (1U)
/* @} */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/
/**
 * @brief PMIC Subsystems
 *
 *  @param   gpioEnable    PMIC GPIO SubSystem
 *  @param   rtcEnable     PMIC RTC SubSystem
 *  @param   wdgEnable     PMIC Watchdog SubSystem
 *  @param   buckEnable    PMIC BUCK SubSystem
 *  @param   ldoEnable     PMIC LDO SubSystem
 *  @param   esmEnable     PMIC ESM SubSystem
 */
typedef struct Pmic_DevSubSysInfo_s {
    bool gpioEnable;
    bool rtcEnable;
    bool wdgEnable;
    bool buckEnable;
    bool ldoEnable;
    bool esmEnable;
} Pmic_DevSubSysInfo_t;

/**
 * @brief  PMIC Interface Handle.
 *         Contains various PMIC driver instance specific information. like,
 *         the PMIC device type, PMIC interface mode, Slave addresses,
 *         various application defined API function pointers for
 *         LLD and Critical sections.
 *
 *         Note: Pmic_init() configures the Pmic Handle information and provide
 *               pmic core handle to user application. User should ensure that
 *               application shall not modify the PMIC Handle information.
 *
 *  @param   pPmic_SubSysInfo             PMIC driver subsystem information
 *  @param   drvInitStatus                PMIC Driver initialization status.
 *                                        Valid driver status:
 *                                        For Main instance:
 *                                           DRV_INIT_STATUS | PMIC_MAIN_INST.
 *                                        For QA instance:
 *                                           DRV_INIT_STATUS | PMIC_QA_INST.
 *  @param   pmicDeviceType               PMIC device type
 *  @param   pmicDevRev                   PMIC device revision ID
 *  @param   pmicDevSiliconRev            PMIC device silicon revision ID
 *  @param   commMode                     Interface mode - Single I2C, Dual
 *                                        I2C or SPI.
 *  @param   slaveAddr                    Main Interface Slave Address
 *  @param   qaSlaveAddr                  WDOG QA Interface Slave Address
 *  @param   nvmSlaveAddr                 NVM Slave Address which provides only
 *                                        read access to CRC status of Page-1
 *                                        Application shall use this slave
 *                                        address to read only CRC status.
 *                                        Application shall not do
 *                                        any write operations using this slave
 *                                        address
 *  @param   i2c1Speed                    I2C1 Speed when commMode is Single or
 *                                        Dual I2C
 *  @param   i2c2Speed                    I2C2 Speed when commMode is Dual I2C
 *  @param   crcEnable                    Parameter to enable/disable CRC
 *                                        For Valid Values:
 *                                                       \ref Pmic_CrcEnableCfg
 *  @param   pFnPmicCommIoRead            Pointer to I2C/SPI Comm LLD Read
 *                                        Function
 *  @param   pFnPmicCommIoWrite           Pointer to I2C/SPI Comm LLD Write
 *                                        Function
 *  @param   pCommHandle                  Pointer to Handle for I2C1/SPI
 *                                        Main Interface
 *  @param   pQACommHandle                Pointer to Handle for I2C2-QA
 *                                        Interface
 *  @param   pFnPmicCritSecStart          Pointer to Pmic Critical-Section
 *                                        Start Function
 *  @param   pFnPmicCritSecStop           Pointer to Pmic Critical-Section
 *                                        Stop Function
 */
typedef struct Pmic_CoreHandle_s {
    const Pmic_DevSubSysInfo_t *pPmic_SubSysInfo;
    uint32_t drvInitStatus;
    uint8_t pmicDeviceType;
    uint8_t pmicDevRev;
    uint8_t pmicDevSiliconRev;
    uint8_t commMode;
    uint8_t slaveAddr;
    uint8_t qaSlaveAddr;
    uint8_t nvmSlaveAddr;
    uint8_t i2c1Speed;
    uint8_t i2c2Speed;
    bool crcEnable;
    void *pCommHandle;
    void *pQACommHandle;
    int32_t (*pFnPmicCommIoRead)(struct Pmic_CoreHandle_s *pmicCorehandle,
                                 uint8_t instType, uint16_t regAddr,
                                 uint8_t *pRxBuf, uint8_t bufLen);
    int32_t (*pFnPmicCommIoWrite)(struct Pmic_CoreHandle_s *pmicCorehandle,
                                  uint8_t instType, uint16_t regAddr,
                                  uint8_t *pTxBuf, uint8_t bufLen);
    void (*pFnPmicCritSecStart)(void);
    void (*pFnPmicCritSecStop)(void);
} Pmic_CoreHandle_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* PMIC_TYPES_H_ */
