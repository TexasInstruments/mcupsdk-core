/*
 *  Copyright (c) Texas Instruments Incorporated 2020
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
 */

/*!
 * \file  enetextphy_priv.h
 *
 * \brief This file contains internal type definitions and helper macros for the
 *        Ethernet PHY interface.
 */

#ifndef ENETEXTPHY_PRIV_H_
#define ENETEXTPHY_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include "enetextphy.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Macro to perform round-up division. */
#define ENETEXTPHY_DIV_ROUNDUP(val, div)         (((val) + (div) - 1) / (div))

#define ENETEXTPHY_PHY_MAX                  (2)

#define ENETEXTPHYTRACE_ERR_IF(cond, format, ...) ((cond) ? (_DebugP_logZone(DebugP_LOG_ZONE_ERROR, "ERROR: %s:%d: " format, __FUNCTION__, __LINE__, ##__VA_ARGS__)) : 0U)
#define ENETEXTPHYTRACE_WARN_IF(cond, format, ...) ((cond) ? (_DebugP_logZone(DebugP_LOG_ZONE_WARN, "WARNING: %s:%d: " format, __FUNCTION__, __LINE__, ##__VA_ARGS__)) : 0U)
#define ENETEXTPHYTRACE_DBG_IF(cond, format, ...) ((cond) ? (_DebugP_logZone(DebugP_LOG_ZONE_INFO, "INFO: %s:%d: " format, __FUNCTION__, __LINE__, ##__VA_ARGS__)) : 0U)
#define ENETEXTPHYTRACE_VERBOSE_IF(cond, format, ...)           ((cond) ? (_DebugP_logZone(DebugP_LOG_ZONE_INFO, "INFO: %s:%d: " format, __FUNCTION__, __LINE__, ##__VA_ARGS__)) : 0U)
#define ENETEXTPHYTRACE_INFO_IF(cond, format, ...) ((cond) ? (_DebugP_logZone(DebugP_LOG_ZONE_INFO, "INFO: %s:%d: " format, __FUNCTION__, __LINE__, ##__VA_ARGS__)) : 0U)

/*!
 * \brief Variable declaration helper macro to avoid unused variable error
 *        (-Werror=unused-variable) when variable is used in TRACE and when
 *        corresponding trace level is not enabled.
 */
#define ENETEXTPHYTRACE_VAR(var)                    ((var) = (var))

#define ENETEXTPHYTRACE_ERR(format, ...)  (_DebugP_logZone(DebugP_LOG_ZONE_ERROR, "ERROR: %s:%d: " format, __FUNCTION__, __LINE__, ##__VA_ARGS__))
#define ENETEXTPHYTRACE_WARN(format, ...) (_DebugP_logZone(DebugP_LOG_ZONE_WARN, "WARNING: %s:%d: " format, __FUNCTION__, __LINE__, ##__VA_ARGS__))
#define ENETEXTPHYTRACE_DBG(format, ...) (_DebugP_logZone(DebugP_LOG_ZONE_INFO, "INFO: %s:%d: " format, __FUNCTION__, __LINE__, ##__VA_ARGS__))
#define ENETEXTPHYTRACE_VERBOSE(format, ...) (_DebugP_logZone(DebugP_LOG_ZONE_INFO, "INFO: %s:%d: " format, __FUNCTION__, __LINE__, ##__VA_ARGS__))


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief PHY specific driver.
 *
 * PHY specific driver implementation. This driver code must not modify registers
 * and/or bitfields that are being modified by the main PHY driver. Otherwise,
 * PHY driver state machine may not work as the underlying state of the PHY would
 * be inconsistent.
 */
typedef struct EnetExtPhy_Drv_s
{
    /*!
     * \brief Driver name.
     *
     * Name of the PHY-specific driver.
     */
    const char *name;

    /*!
     * \brief Check if driver supports a PHY model identified by its version.
     *
     * PHY-specific function that drivers must implement for upper check if the
     * PHY driver supports a PHY model identified by its version from ID1 and
     * ID2 registers.
     *
     * Note that a given PHY driver can support multiple PHY models.
     *
     * \param hPhy     PHY device handle
     * \param version  PHY version from ID registers
     *
     * \return Whether PHY model is supported or not
     */

    bool (*isPhyDevSupported)(EnetExtPhy_Handle hPhy,
                              const EnetExtPhy_Version *version);

    /*!
     * \brief Check if driver supports a MII interface type.
     *
     * PHY-specific function that drivers must implement for upper layer to check
     * whether a MAC mode is supported by the PHY driver or not.
     *
     * \param hPhy     PHY device handle
     * \param mii      MII interface
     *
     * \return Whether MII interface type is supported or not
     */
    bool (*isMacModeSupported)(EnetExtPhy_Handle hPhy,
                               EnetExtPhy_Mii mii);

    /*!
     * \brief PHY specific configuration.
     *
     * PHY-specific function that drivers must implement to configure the PHY
     * device.  The configuration can be composed of generic and PHY-specific
     * parameter (via extended config).
     *
     * \param hPhy     PHY device handle
     * \param cfg      PHY configuration parameter
     * \param mii      MII interface
     *
     * \return \ref EnetExtPhy_ErrorCodes
     */
    int32_t (*config)(EnetExtPhy_Handle hPhy,
                      const EnetExtPhy_Cfg *cfg,
                      EnetExtPhy_Mii mii);

    /*!
     * \brief PHY specific soft reset.
     *
     * PHY-specific function that drivers must implement to start a soft-reset
     * operation.
     *
     * \param hPhy     PHY device handle
     */
    void (*reset)(EnetExtPhy_Handle hPhy);

    /*!
     * \brief PHY specific soft reset status.
     *
     * PHY-specific function that drivers must implement to check if soft-reset
     * operation is complete.
     *
     * \param hPhy     PHY device handle
     *
     * \return Whether soft-reset is complete or not.
     */
    bool (*isResetComplete)(EnetExtPhy_Handle hPhy);

    /*!
     * \brief Read PHY extended register.
     *
     * PHY-specific function that drivers must implement to read extended
     * registers.
     *
     * \param hPhy     PHY device handle
     * \param reg      Register number
     * \param val      Pointer to the read value
     *
     * \return \ref EnetExtPhy_ErrorCodes
     */
    int32_t (*readExtReg)(EnetExtPhy_Handle hPhy,
                          uint32_t reg,
                          uint16_t *val);

    /*!
     * \brief Write PHY register.
     *
     * PHY-specific function that drivers must implement to write extended
     * registers.
     *
     * \param hPhy     PHY device handle
     * \param reg      Register number
     * \param val      Value to be written
     *
     * \return \ref EnetExtPhy_ErrorCodes
     */
    int32_t (*writeExtReg)(EnetExtPhy_Handle hPhy,
                           uint32_t reg,
                           uint16_t val);

    /*!
     * \brief Read-modify-write PHY extended register.
     *
     * PHY-specific function that drivers must implement to read-write-modify
     * extended registers.
     *
     * \param group      User group (use 0 if single group is supported)
     * \param phyAddr    PHY device address
     * \param reg        Register address
     * \param val        Value read from register
     */
    int32_t (*rmwExtReg)(EnetExtPhy_Handle hPhy,
                         uint32_t group,
                         uint32_t reg,
                         uint16_t mask,
                         uint16_t val);

    /*! Print PHY registers */
    void (*printRegs)(EnetExtPhy_Handle hPhy);
} EnetExtPhy_Drv;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                        Deprecated Function Declarations                    */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* ENETEXTPHY_PRIV_H_ */
