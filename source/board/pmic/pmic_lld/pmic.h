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
 *  \defgroup DRV_PMIC_MODULE PMIC Driver
 *
 *  This is an API guide for PMIC(Power Management Integrated Circuit) Driver.
 *  PMIC Driver is designed to power up different components on the embedded
 *  boards or provide supply to MCU(Micro Controller Unit) or
 *  SoC(System on chip) using APIs provided in the guide.
 *
 *  The PMIC Driver supports below mentioned PMIC devices and their
 *  features or Modules.
 *
 *  Supported PMIC Devices are:
 *    1. TPS6594x (Leo PMIC Device)
 *    2. LP8764x  (Hera PMIC Device)
 *
 *  Above PMICs have multiple functionalities and configurable features. Like,
 *  Real Time Clock (RTC) which provides Time, Calendar, Alarm and timer,
 *  Configurable GPIO pins to support wakeup, nSLEEP, PGOOD, nRESET for
 *  SOC/MCU, GPIOs with configurable PU/PD to enable other chips,
 *  Have number of BUCK(with different phases) and LDO regulators to provide
 *  supply to other modules on the board, Have the Voltage Monitor feature
 *  to monitor and notify for OV, UV, SC and Over Heat(Thermal Monitor).
 *  Have interrupt feature to notify severe, moderate, fsm Errors and provide
 *  asynchronous events for all supported features including GPIO External
 *  Interrupts, Have the WatchDog feature to monitor correct operation of the
 *  MCU using WDG trigger mode or using WDOG QA mode, Supports I2C(single and
 *  dual mode) and SPI communication protocols to access the registers for
 *  Read/Write operations with or without CRC.
 *
 *  All above PMICs features can be accessed or configured by using PMIC Driver
 *  APIs and illustrated in the guide.
 *
 *  @{
 */
/* @} */

/**
 *  \ingroup DRV_PMIC_MODULE
 *  \defgroup DRV_PMIC_API_MODULE PMIC Driver Initialization API
 *  This is PMIC driver core handle initialization APIs common for all supported
 *  PMIC devices. Like, Pmic init, and deinit APIs
 *
 *  @{
 */

/**
 *  \file pmic.h
 *
 *  \brief PMIC Driver initialization API/interface file.
 */

#ifndef PMIC_H_
#define PMIC_H_

/* ========================================================================= */
/*                             Include Files                                 */
/* ========================================================================= */
#include <pmic_types.h>
#include <pmic_core.h>
#include <pmic_gpio.h>
#include <pmic_rtc.h>
#include <pmic_irq.h>
#include <pmic_power.h>
#include <pmic_wdg.h>
#include <pmic_esm.h>
#include <pmic_fsm.h>

#ifdef __cplusplus
extern "C" {
#endif
/* ========================================================================= */
/*                             Macros & Typedefs                             */
/* ========================================================================= */

/**
 *  \anchor Pmic_ErrorCodes
 *  \name PMIC Error Codes
 *
 *  Error codes returned by PMIC APIs
 *
 *  @{
 */
/** \brief Error Code for SUCCESS */
#define PMIC_ST_SUCCESS                                 (0)
/** \brief Error Code for Invalid input Handle */
#define PMIC_ST_ERR_INV_HANDLE                          (-((int32_t)1))
/** \brief Error Code when input Param is NULL */
#define PMIC_ST_ERR_NULL_PARAM                          (-((int32_t)2))
/** \brief Error Code for Invalid input Param */
#define PMIC_ST_ERR_INV_PARAM                           (-((int32_t)3))
/** \brief Error Code for Invalid PMIC Device */
#define PMIC_ST_ERR_INV_DEVICE                          (-((int32_t)4))
/** \brief Error Code when input Function pointer is NULL */
#define PMIC_ST_ERR_NULL_FPTR                           (-((int32_t)5))
/** \brief Error Code for Invalid PMIC Subsystem */
#define PMIC_ST_ERR_INV_SUBSYSTEM                       (-((int32_t)6))
/** \brief Error Code for Insufficient input configuration params for PMIC
 *         Device Initialization */
#define PMIC_ST_ERR_INSUFFICIENT_CFG                    (-((int32_t)7))
/** \brief Error Code for I2C Communication Fail */
#define PMIC_ST_ERR_I2C_COMM_FAIL                       (-((int32_t)8))
/** \brief Error Code for SPI Communication Fail */
#define PMIC_ST_ERR_SPI_COMM_FAIL                       (-((int32_t)9))
/** \brief Error Code for IO data failure when CRC is enabled */
#define PMIC_ST_ERR_DATA_IO_CRC                         (-((int32_t)10))
/** \brief Error Code for Interface Selected is not working properly */
#define PMIC_ST_ERR_INTF_SETUP_FAILED                   (-((int32_t)11))
/** \brief Error Code for Interface Init Fail */
#define PMIC_ST_ERR_COMM_INTF_INIT_FAIL                 (-((int32_t)12))
/** \brief Error Code if Pmic_init() function called before PMIC driver is
 *         initialized */
#define PMIC_ST_ERR_UNINIT                              (-((int32_t)13))
/** \brief Error Code for Invalid Voltage Value */
#define PMIC_ST_ERR_INV_VOLTAGE                         (-((int32_t)14))
/** \brief Error Code for Invalid Power resource Value */
#define PMIC_ST_ERR_INV_REGULATOR                       (-((int32_t)15))
/** \brief Error Code for Invalid Power Good Threshold Value */
#define PMIC_ST_ERR_INV_PGOOD_LEVEL                     (-((int32_t)16))
/** \brief Error Code for Invalid Temperature Threshold value */
#define PMIC_ST_ERR_INV_TEMP_THRESHOLD                  (-((int32_t)17))
/** \brief Error Code for Invalid input GPIO PIN */
#define PMIC_ST_ERR_INV_GPIO                            (-((int32_t)18))
/** \brief Error Code for Invalid GPIO Functionality */
#define PMIC_ST_ERR_INV_GPIO_FUNC                       (-((int32_t)19))
/** \brief Error Code for input GPIO handle is invalid */
#define PMIC_ST_ERR_INV_GPIO_LINE_PARAMS                (-((int32_t)20))
/** \brief Error Code for Invalid Pin for GPIO Configuration */
#define PMIC_ST_ERR_PIN_NOT_GPIO                        (-((int32_t)21))
/** \brief Error Code for Invalid Watchdog longwindow or Window-1
 *         or Window-2 duration */
#define PMIC_ST_ERR_INV_WDG_WINDOW                      (-((int32_t)22))
/** \brief Error Code for Invalid Watchdog Answer */
#define PMIC_ST_ERR_INV_WDG_ANSWER                      (-((int32_t)23))
/** \brief Error Code for Watchdog early Answer */
#define PMIC_ST_ERR_WDG_EARLY_ANSWER                    (-((int32_t)24))
/** \brief Error Code for input ESM TargetID is invalid */
#define PMIC_ST_ERR_INV_ESM_TARGET                      (-((int32_t)25))
/** \brief Error Code for ESM Operation Mode is invalid */
#define PMIC_ST_ERR_INV_ESM_MODE                        (-((int32_t)26))
/** \brief Error Code for Invalid Interrupt */
#define PMIC_ST_ERR_INV_INT                             (-((int32_t)27))
/** \brief Error Code when Interrupts Clear is failed */
#define PMIC_ST_ERR_CLEAR_INT_FAILED                    (-((int32_t)28))
/** \brief Error Code for Invalid Time */
#define PMIC_ST_ERR_INV_TIME                            (-((int32_t)29))
/** \brief Error Code for Invalid Date */
#define PMIC_ST_ERR_INV_DATE                            (-((int32_t)30))
/** \brief Error Code for RTC Stop command failure */
#define PMIC_ST_ERR_RTC_STOP_FAIL                       (-((int32_t)31))
/** \brief Error Code for any other failures */
#define PMIC_ST_ERR_FAIL                                (-((int32_t)32))
/** \brief Error Code for ESM in Start State */
#define PMIC_ST_ERR_ESM_STARTED                         (-((int32_t)33))
/** \brief Error Code for Invalid ESM delay1, delay2, HMAX, HMIN, LMAX and
 *         LMIN values */
#define PMIC_ST_ERR_INV_ESM_VAL                         (-((int32_t)34))
/** \brief warning Code for Device ID mismatch warning */
#define PMIC_ST_WARN_INV_DEVICE_ID                      (-((int32_t)35))
/** \brief Error Code for EN_DRV Pin configuration when FORCE_EN_DRV_LOW is set
 *         to '1'  */
#define PMIC_ST_ERR_INV_EN_DRV_PIN_CFG                  (-((int32_t)36))
/** \brief Error Code for I2C Speed configuration when commMode is set
 *         to PMIC_INTF_SPI  */
#define PMIC_ST_ERR_INV_COMM_MODE                       (-((int32_t)37))
/** \brief Error Code for CRC Status Failure */
#define PMIC_ST_ERR_CRC_STATUS_FAIL                     (-((int32_t)38))
/** \brief Error Code for Register Write Failure when register is locked */
#define PMIC_ST_ERR_REG_LOCKED_WR_FAIL                  (-((int32_t)39))
/** \brief Error Code for when a feature is not supported on PMIC device type
 *         or PMIC silicon revision*/
#define PMIC_ST_ERR_NOT_SUPPORTED                       (-((int32_t)40))
/** \brief Error Code for Invalid PMIC Device Silicon Revision*/
#define PMIC_ST_ERR_INV_SILICON_REVISION                (-((int32_t)41))
/* @} */

/**
 *  \anchor Pmic_DeviceType
 *  \name PMIC Device type
 *
 *  @{
 */
#define PMIC_DEV_LEO_TPS6594X  (0U)
#define PMIC_DEV_HERA_LP8764X  (1U)
/* @} */

/**
 *  \anchor Pmic_CommMode
 *  \name PMIC Communication Mode
 *
 *  @{
 */
#define PMIC_INTF_SINGLE_I2C   (0U)
#define PMIC_INTF_DUAL_I2C     (1U)
#define PMIC_INTF_SPI          (2U)
/* @} */

/**
 *  \anchor Pmic_I2CSpeedSel
 *  \name PMIC Select I2C Speed
 *          Note: I2C Master before switching the I2C speed to HS/Standard Mode,
 *          I2C Master has to set/reset I2C1_HS/I2C2_HS bit field accordingly
 *          then only I2C Master can communicate with PMIC in HS/Standard Mode
 *
 *  @{
 */
 /** \brief  Standard or Fast or Fast+ Mode  */
#define PMIC_I2C_STANDARD_MODE   (0U)
/** \brief  High-Speed Mode */
#define PMIC_I2C_FORCED_HS_MODE  (1U)
/* @} */

/**
 *  \anchor Pmic_InstType
 *  \name PMIC Instance Type
 *
 *  @{
 */
#define PMIC_MAIN_INST   (1U << 0U)
#define PMIC_QA_INST     (1U << 1U)
/** \brief  Valid only to read CRC status from Page-1 using NVM Slave Address
 *          Valid only while calling the pFnPmicCommIoRead API
 */
#define PMIC_NVM_INST    (1U << 2U)
/* @} */

/**
 *  \anchor Pmic_ValidParamCfg
 *  \name  PMIC Config Structure Param Bits
 *
 *  @{
 */
  /** \brief validParams value used to set PMIC device type of PMIC Driver
   *         Handle */
#define PMIC_CFG_DEVICE_TYPE_VALID      (0U)
  /** \brief validParams value used to set Interface mode of PMIC Driver
   *         Handle */
#define PMIC_CFG_COMM_MODE_VALID        (1U)
  /** \brief validParams value used to set Main Interface Slave Address of PMIC
   *         Driver Handle */
#define PMIC_CFG_SLAVEADDR_VALID        (2U)
  /** \brief validParams value used to set WDOG QA Interface Slave Address of
   *         PMIC Driver Handle */
#define PMIC_CFG_QASLAVEADDR_VALID      (3U)
  /** \brief validParams value used to set NVM Slave Address of PMIC Driver
   *         Handle */
#define PMIC_CFG_NVMSLAVEADDR_VALID     (4U)
  /** \brief validParams value used to set Pointer to Handle for I2C1/SPI
   *         Main Interface of PMIC Driver Handle */
#define PMIC_CFG_COMM_HANDLE_VALID      (5U)
  /** \brief validParams value used to set Pointer to Handle for I2C2-QA
   *         Interface of PMIC Driver Handle */
#define PMIC_CFG_QACOMM_HANDLE_VALID    (6U)
  /** \brief validParams value used to set Pointer to I2C/SPI Comm LLD Read
   *         Function of PMIC Driver Handle */
#define PMIC_CFG_COMM_IO_RD_VALID       (7U)
  /** \brief validParams value used to set Pointer to I2C/SPI Comm LLD Write
   *         Function of PMIC Driver Handle */
#define PMIC_CFG_COMM_IO_WR_VALID       (8U)
  /** \brief validParams value used to set Pointer to Pmic Critical-Section
   *         Start Function of PMIC Driver Handle */
#define PMIC_CFG_CRITSEC_START_VALID    (9U)
  /** \brief validParams value used to set Pointer to Pmic Critical-Section
   *         Stop Function of PMIC Driver Handle */
#define PMIC_CFG_CRITSEC_STOP_VALID     (10U)
  /** \brief validParams value used to set I2C1 Speed of PMIC Driver
   *         Handle */
#define PMIC_CFG_I2C1_SPEED_VALID       (11U)
  /** \brief validParams value used to set I2C2 Speed of PMIC Driver
   *         Handle */
#define PMIC_CFG_I2C2_SPEED_VALID       (12U)
/* @} */

/**
 *  \anchor Pmic_CfgStructPrmBitShiftVal
 *  \name PMIC Config Structure Param Bit shift values
 *
 *  Application can use below shifted values to set the validParam
 *  struct member defined in Pmic_CoreCfg_t structure
 *
 *  @{
 */
#define PMIC_CFG_DEVICE_TYPE_VALID_SHIFT   (1U << PMIC_CFG_DEVICE_TYPE_VALID)
#define PMIC_CFG_COMM_MODE_VALID_SHIFT     (1U << PMIC_CFG_COMM_MODE_VALID)
#define PMIC_CFG_SLAVEADDR_VALID_SHIFT     (1U << PMIC_CFG_SLAVEADDR_VALID)
#define PMIC_CFG_QASLAVEADDR_VALID_SHIFT   (1U << PMIC_CFG_QASLAVEADDR_VALID)
#define PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT  (1U << PMIC_CFG_NVMSLAVEADDR_VALID)
#define PMIC_CFG_COMM_HANDLE_VALID_SHIFT   (1U << PMIC_CFG_COMM_HANDLE_VALID)
#define PMIC_CFG_QACOMM_HANDLE_VALID_SHIFT (1U << PMIC_CFG_QACOMM_HANDLE_VALID)
#define PMIC_CFG_COMM_IO_RD_VALID_SHIFT    (1U << PMIC_CFG_COMM_IO_RD_VALID)
#define PMIC_CFG_COMM_IO_WR_VALID_SHIFT    (1U << PMIC_CFG_COMM_IO_WR_VALID)
#define PMIC_CFG_CRITSEC_START_VALID_SHIFT (1U << PMIC_CFG_CRITSEC_START_VALID)
#define PMIC_CFG_CRITSEC_STOP_VALID_SHIFT  (1U << PMIC_CFG_CRITSEC_STOP_VALID)
#define PMIC_CFG_I2C1_SPEED_VALID_SHIFT    (1U << PMIC_CFG_I2C1_SPEED_VALID)
#define PMIC_CFG_I2C2_SPEED_VALID_SHIFT    (1U << PMIC_CFG_I2C2_SPEED_VALID)
/* @} */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/
/*!
 * \brief  PMIC configuration structure.
 *         Contains various parameters which are needed to prepare
 *         PMIC driver handle using Valid params.
 *         like, PMIC device type, PMIC interface mode, Slave address,
 *         various application defined API function pointers for
 *         LLD and Critical sections.
 *         Application has to set the corresponding bit in validParams
 *         structure member to update the driver with Pmic_CoreCfg_t
 *         structure fields.
 *         For Example, If the Application needs to configure the PMIC driver
 *         pmicDeviceType member of the structure then application has
 *         to set PMIC_CFG_DEVICE_TYPE_VALID bit of validParams struct
 *         and then call pmic_init().
 *
 *  \param   validParams                  Validate params Bits.
 *                                        Selection of structure parameters to
 *                                        be set, from the combination of
 *                                        \ref Pmic_ValidParamCfg and the
 *                                        corresponding member value must be
 *                                        updated.
 *  \param   instType                     Instance type.
 *                                        For Valid Values: \ref Pmic_InstType.
 *  \param   pmicDeviceType               PMIC device type.
 *                                        For Valid Values: \ref Pmic_DeviceType
 *                                        Valid only when
 *                                        PMIC_CFG_DEVICE_TYPE_VALID bit of
 *                                        validParams is set.
 *  \param   commMode                     Interface mode - Single I2C, Dual
 *                                        I2C or SPI.
 *                                        For Valid Values: \ref Pmic_CommMode.
 *                                        Valid only when
 *                                        PMIC_CFG_COMM_MODE_VALID bit of
 *                                        validParams is set.
 *  \param   slaveAddr                    Main Interface Slave Address.
 *                                        Valid only when
 *                                        PMIC_CFG_SLAVEADDR_VALID bit of
 *                                        validParams is set.
 *  \param   qaSlaveAddr                  WDOG QA Interface Slave Address.
 *                                        Valid only when
 *                                        PMIC_CFG_QASLAVEADDR_VALID bit
 *                                        of validParams is set.
 *  \param   nvmSlaveAddr                 NVM Slave Address which provides only
 *                                        read access to CRC status of Page-1
 *                                        Application shall use this slave
 *                                        address to read only CRC status.
 *                                        Application shall not do
 *                                        any write operations using this slave
 *                                        address
 *                                        Valid only when
 *                                        PMIC_CFG_NVMSLAVEADDR_VALID bit
 *                                        of validParams is set.
 *  \param   i2c1Speed                    Configures I2C1 Speed when commMode is
 *                                        Single or Dual I2C
 *                                        For Valid Values:
 *                                                  \ref Pmic_I2CSpeedSel
 *                                        Valid only when
 *                                        PMIC_CFG_I2C1_SPEED_VALID bit is set
 *  \param   i2c2Speed                    Configures I2C2 Speed when commMode is
 *                                        Dual I2C
 *                                        For Valid Values:
 *                                                  \ref Pmic_I2CSpeedSel
 *                                        Valid only when
 *                                        PMIC_CFG_I2C2_SPEED_VALID bit is set
 *  \param   pFnPmicCommIoRead            Pointer to I2C/SPI Comm LLD Read
 *                                        Function. Valid only when
 *                                        PMIC_CFG_COMM_IO_RD_VALID bit
 *                                        of validParams is set.
 *  \param   pFnPmicCommIoWrite           Pointer to I2C/SPI Comm LLD Write
 *                                        Function. Valid only when
 *                                        PMIC_CFG_COMM_IO_WR_VALID bit
 *                                        of validParams is set.
 *  \param   pCommHandle                  Pointer to Handle for I2C1/SPI
 *                                        Main Interface. Valid only when
 *                                        PMIC_CFG_COMM_HANDLE_VALID bit
 *                                        of validParams is set.
 *  \param   pQACommHandle                Pointer to Handle for I2C2-QA
 *                                        Interface. Valid only when
 *                                        PMIC_CFG_QACOMM_HANDLE_VALID bit
 *                                        of validParams is set.
 *  \param   pFnPmicCritSecStart          Pointer to Pmic Critical-Section
 *                                        Start Function. Valid only when
 *                                        PMIC_CFG_CRITSEC_START_VALID bit
 *                                        of validParams is set.
 *  \param   pFnPmicCritSecStop           Pointer to Pmic Critical-Section
 *                                        Stop Function. Valid only when
 *                                        PMIC_CFG_CRITSECSTOP_VALID bit of
 *                                        validParams is set.
 */
typedef struct Pmic_CoreCfg_s {
    uint32_t     validParams;
    uint32_t     instType;
    uint8_t      pmicDeviceType;
    uint8_t      commMode;
    uint8_t      slaveAddr;
    uint8_t      qaSlaveAddr;
    uint8_t      nvmSlaveAddr;
    uint8_t      i2c1Speed;
    uint8_t      i2c2Speed;
    void        *pCommHandle;
    void        *pQACommHandle;
    int32_t (*pFnPmicCommIoRead)(struct Pmic_CoreHandle_s  *pmicCorehandle,
                                 uint8_t                    instType,
                                 uint16_t                   regAddr,
                                 uint8_t                   *pRxBuf,
                                 uint8_t                    bufLen);
    int32_t (*pFnPmicCommIoWrite)(struct Pmic_CoreHandle_s *pmicCorehandle,
                                  uint8_t                   instType,
                                  uint16_t                  regAddr,
                                  uint8_t                  *pTxBuf,
                                  uint8_t                   bufLen);
    void (*pFnPmicCritSecStart)(void);
    void (*pFnPmicCritSecStop)(void);
} Pmic_CoreCfg_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/
/*!
 * \brief  API to Initialize pmic core handle for PMIC LLD.
 *
 * Requirement: REQ_TAG(PDK-5814), REQ_TAG(PDK-5810), REQ_TAG(PDK-5813),
 *              REQ_TAG(PDK-5843), REQ_TAG(PDK-5811), REQ_TAG(PDK-5853),
 *              REQ_TAG(PDK-9129), REQ_TAG(PDK-9329), REQ_TAG(PDK-9159),
 *              REQ_TAG(PDK-5816), REQ_TAG(PDK-5817), REQ_TAG(PDK-5818),
 *              REQ_TAG(PDK-5819), REQ_TAG(PDK-5820), REQ_TAG(PDK-5821),
 *              REQ_TAG(PDK-5822), REQ_TAG(PDK-5823), REQ_TAG(PDK-5824),
 *              REQ_TAG(PDK-5825), REQ_TAG(PDK-5826), REQ_TAG(PDK-5827),
 *              REQ_TAG(PDK-5856), REQ_TAG(PDK-5857), REQ_TAG(PDK-5858),
 *              REQ_TAG(PDK-5859), REQ_TAG(PDK-5860)
 * Design: did_pmic_comm_intf_cfg, did_pmic_comm_single_i2c_cfg,
 *         did_pmic_comm_dual_i2c_cfg, did_pmic_comm_spi_cfg,
 *         did_pmic_tps6594x_j721e_support, did_pmic_lp8764x_j7200_support,
 *         did_pmic_validation_feature_support, did_pmic_performance_support,
 *         did_pmic_generic_feature_support, did_pmic_safety_feature_support,
 *         did_pmic_pre_emption_support, did_pmic_stateless_reentrant_support,
 *         did_pmic_dynamic_alloc_mem_not_supported, did_pmic_build_infra_cfg,
 *         did_pmic_debug_release_profile_support, did_pmic_standalone_support,
 *         did_pmic_multiple_pmic_support, did_pmic_baremetal_support
 * Architecture: aid_pmic_tps6594x_lp8764x_support, aid_pmic_standalone_support,
 *               aid_pmic_multiple_pmic_support, aid_pmic_pre_emption_support,
 *               aid_pmic_stateless_reentrant_support, aid_pmic_generic_support,
 *               aid_pmic_baremetal_support, aid_pmic_comm_intf_i2c_spi_cfg,
 *               aid_pmic_dynamic_alloc_mem_not_supported,
 *               aid_pmic_build_infra_cfg,
 *               aid_pmic_debug_release_profile_support,
 *               aid_pmic_performance_support, aid_pmic_test_support
 *
 *         This function gets device configuration from pPmicConfigData and
 *         initializes device specific information in pPmicCoreHandle after
 *         validation of given params depends on validParams bit fields
 *         and does some basic validation on PMIC interface I2C/SPI,
 *         confirming that PMIC is accessible for PMIC configuration and
 *         monitor features.
 *
 *  \param   pPmicConfigData [IN]   PMIC Configuration data
 *  \param   pPmicCoreHandle [OUT]  PMIC Interface Handle.
 *
 *  \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *           For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_init(const Pmic_CoreCfg_t *pPmicConfigData,
                  Pmic_CoreHandle_t    *pPmicCoreHandle);

/*!
 * \brief  API to DeInitilizes an existing PMIC Instance.
 *
 * Requirement: REQ_TAG(PDK-5814)
 * Design: did_pmic_comm_intf_cfg
 * Architecture: aid_pmic_tps6594x_lp8764x_support
 *
 *         This function takes an existing Instance pPmicCoreHandle and
 *         closes the LLD being used for this Instance. It should be called
 *         only once per valid pPmicCoreHandle. Should not be called
 *         if Pmic_init() is not called
 *
 *  \param   pPmicCoreHandle  [IN] PMIC Interface Handle.
 *
 *  \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *           For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_deinit(Pmic_CoreHandle_t  *pPmicCoreHandle);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* PMIC_H_ */

/* @} */
