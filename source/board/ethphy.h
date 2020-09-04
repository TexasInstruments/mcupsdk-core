/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

/**
 *  \defgroup BOARD_ETHPHY_MODULE APIs for Ethernet PHY
 *  \ingroup BOARD_MODULE
 *
 *  This module contains APIs to program and use the Ethernet PHY present on
 *  the board.
 *  See \ref BOARD_ETHPHY_PAGE for more details.
 *
 *  @{
 */

#ifndef ETHPHY_H_
#define ETHPHY_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <kernel/dpl/SystemP.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  \anchor ETHPHY_Commands
 *  \name ETHPHY Commands
 *
 *  Various commands for the Eternet PHY
 *
 *  @{
 */
/** \brief Command to configure the PHY in MII mode */
#define ETHPHY_CMD_ENABLE_MII                           (0U)
/** \brief Command for Soft Restart. It restarts the PHY without affecting
 *         registers.
 * */
#define ETHPHY_CMD_SOFT_RESTART                         (1U)
/** \brief Command to enable Auto MDI-X (Automatic Crossover) */
#define ETHPHY_CMD_ENABLE_AUTO_MDIX                     (2U)
/** \brief Command to verify the PHY Identifier Register. It checks if the
 *         PHYIDR1 register has the expected value.
 */
#define ETHPHY_CMD_VERIFY_IDENTIFIER_REGISTER           (3U)
/** \brief Command to disable 1000M ability advertisement  */
#define ETHPHY_CMD_DISABLE_1000M_ADVERTISEMENT          (4U)
/** \brief Command to enable Fast Link Down Detection. "data" argument should
 *         be passed while making \ref ETHPHY_command API call with this
 *         command. Refer to the PHY-specific .h file in "source/board/ethphy"
 *         for the data structure needed to be passed.
 */
#define ETHPHY_CMD_ENABLE_FAST_LINK_DOWN_DETECTION      (5U)
/** \brief Command to configure the source of PHY LEDs. "data" argument should
 *         be passed while making \ref ETHPHY_command API call with this
 *         command. Refer to the PHY-specific .h file in "source/board/ethphy"
 *         for the data structure needed to be passed.
 */
#define ETHPHY_CMD_CONFIGURE_LED_SOURCE                 (6U)
/** \brief Command to configure the blink rate of PHY LEDs. "data" argument should
 *         be passed while making \ref ETHPHY_command API call with this
 *         command. Refer to the PHY-specific .h file in "source/board/ethphy"
 *         for the data structure needed to be passed.
 */
#define ETHPHY_CMD_CONFIGURE_LED_BLINK_RATE             (7U)
/** \brief Command to enable the extended full duplex ability */
#define ETHPHY_CMD_ENABLE_EXTENDED_FD_ABILITY           (8U)
/** \brief Command to enable odd nibble detection */
#define ETHPHY_CMD_ENABLE_ODD_NIBBLE_DETECTION          (9U)
/** \brief Command to enable enhanced IPG detection */
#define ETHPHY_CMD_ENABLE_ENHANCED_IPG_DETECTION        (10U)
/** \brief Command to get Link Status for the PHY */
#define ETHPHY_CMD_GET_LINK_STATUS                      (11U)
/** \brief Command to get Speed and Duplex configuration for the PHY. "data"
 *         argument should be passed while making \ref ETHPHY_command API call
 *         with this command. "data" should be of type \ref ETHPHY_SpeedDuplexConfig
 */
#define ETHPHY_CMD_GET_SPEED_AND_DUPLEX_CONFIG          (12U)
/** \brief Command to set Speed and Duplex configuration for the PHY. "data"
 *         argument should be passed while making \ref ETHPHY_command API call
 *         with this command. "data" should be of type \ref ETHPHY_SpeedDuplexConfig
 */
#define ETHPHY_CMD_SET_SPEED_AND_DUPLEX_CONFIG          (13U)
/** \brief Command to enable low latency for 10M/100M operation in RGMII mode*/
#define ETHPHY_CMD_ENABLE_LOW_LATENCY_10M_100M_RGMII    (14U)
/** \brief Command to set RX half full threshold in RGMII mode. "data"
 *         argument should be passed while making \ref ETHPHY_command API call
 *         with this command. "data" should be a pointer to "uint8_t" and value
 *         should be present in the lowest 3 bits.
 */
#define ETHPHY_CMD_SET_RX_HALF_FULL_THRESHOLD_RGMII     (15U)
/** \brief Command to set TX half full threshold in RGMII mode. "data"
 *         argument should be passed while making \ref ETHPHY_command API call
 *         with this command. "data" should be a pointer to "uint8_t" and value
 *         should be present in the lowest 3 bits.
 */
#define ETHPHY_CMD_SET_TX_HALF_FULL_THRESHOLD_RGMII     (16U)
/** \brief Command to get auto-negotiation completion status. "data"
 *         argument should be passed while making \ref ETHPHY_command API call
 *         with this command. "data" should be a pointer to "uint8_t" and value
 *         returned will be 0 if auto-negotiation is not complete, and 1 if it is
 *         complete.
 */
#define ETHPHY_CMD_GET_AUTONEG_COMPLETE_STATUS          (17U)
/** \brief Command to get link partner's auto-negotiation ability. "data"
 *         argument should be passed while making \ref ETHPHY_command API call
 *         with this command. "data" should be a pointer to "uint8_t" and value
 *         returned will be 0 if partner is not auto-negotiation able, and 1 if
 *         it is in auto-neogtiation able.
 */
#define ETHPHY_CMD_GET_LINK_PARTNER_AUTONEG_ABILITY     (18U)
/** \brief Command to enable the IEEE Power Down mode*/
#define ETHPHY_CMD_ENABLE_IEEE_POWER_DOWN               (19U)
/** \brief Command to disable the IEEE Power Down mode*/
#define ETHPHY_CMD_DISABLE_IEEE_POWER_DOWN              (20U)
/** @} */

/**
 *  \anchor ETHPHY_SPEED_DUPLEX_CONFIGURATIONS
 *  \name ETHPHY SPEED AND DUPLEX CONFIGURATIONS
 *
 *  Different modes for speed and duplex configuration in ETHPHY
 *
 *  @{
 */
#define ETHPHY_SPEED_DUPLEX_CONFIG_AUTONEG       (0U)
#define ETHPHY_SPEED_DUPLEX_CONFIG_10FD          (1U)
#define ETHPHY_SPEED_DUPLEX_CONFIG_100FD         (2U)
#define ETHPHY_SPEED_DUPLEX_CONFIG_1000FD        (3U)
#define ETHPHY_SPEED_DUPLEX_CONFIG_10HD          (4U)
#define ETHPHY_SPEED_DUPLEX_CONFIG_100HD         (5U)
#define ETHPHY_SPEED_DUPLEX_CONFIG_1000HD        (6U)
#define ETHPHY_SPEED_DUPLEX_CONFIG_INVALID       (7U)
/** @} */

/** \brief Handle to the ETHPHY driver returned by #ETHPHY_open() */
typedef void *ETHPHY_Handle;
/** \brief Forward declaration of \ref ETHPHY_Config */
typedef struct ETHPHY_Config_s ETHPHY_Config;
/** \brief Forward declaration of \ref ETHPHY_Params */
typedef struct ETHPHY_Params_s ETHPHY_Params;

/**
 * \name ETHPHY driver implementation callbacks
 *
 * @{
 */

/**
 *  \brief Driver implementation to open a specific ETHPHY driver
 *
 *  Typically this callback is hidden from the end application and is implemented
 *  when a new type of ETHPHY device needs to be implemented.
 *
 *  \param config [IN] ETHPHY configuration for the specific ETHPHY device
 *  \param params [IN] User controllable parameters when opening the ETHPHY device
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*ETHPHY_OpenFxn)(ETHPHY_Config *config, const ETHPHY_Params *params);

/**
 *  \brief Driver implementation to close a specific ETHPHY driver
 *
 *  Typically this callback is hidden from the end application and is implemented
 *  when a new type of ETHPHY device needs to be implemented.
 *
 *  \param config [IN] ETHPHY configuration for the specific ETHPHY device
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
typedef void (*ETHPHY_CloseFxn)(ETHPHY_Config *config);

/**
 *  \brief Driver implementation to send command to the ETHPHY using specific
 *         ETHPHY driver
 *
 *  Typically this callback is hidden from the end application and is implemented
 *  when a new type of ETHPHY device needs to be implemented.
 *
 *  \param config   [IN] ETHPHY configuration for the specific ETHPHY device
 *  \param command  [IN] Command from \ref ETHPHY_Commands
 *  \param data     [IN] Pointer to structure which has the data to write/read
 *  \param dataSize [IN] Size of the structure pointed by data
 *
 *  \return SystemP_SUCCESS on success, else failure
 */
typedef int32_t (*ETHPHY_CommandFxn)(ETHPHY_Config  *config,
                                     uint32_t       command,
                                     void           *data,
                                     uint32_t       dataSize);

/** @} */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 * \brief Parameters passed during ETHPHY_open()
 */
typedef struct ETHPHY_Params_s
{
    uint32_t reserved; /**< reserved for future use */
} ETHPHY_Params;

/**
 * \brief ETHPHY Driver implementation callbacks
 */
typedef struct ETHPHY_Fxns_s
{
    ETHPHY_OpenFxn     openFxn;    /**< ETHPHY driver implementation specific callback */
    ETHPHY_CloseFxn    closeFxn;   /**< ETHPHY driver implementation specific callback */
    ETHPHY_CommandFxn  commandFxn; /**< ETHPHY driver implementation specific callback */
} ETHPHY_Fxns;

/**
 * \brief ETHPHY device attributes. These are filled by SysCfg based on the PHY
 *        device that is selected.
 */
typedef struct ETHPHY_Attrs_s
{
    uint32_t mdioBaseAddress; /**< Base Address of the MDIO module connected to ETHPHY */
    uint32_t phyAddress;      /**< ETHPHY Address corresponding to MDIO module */
} ETHPHY_Attrs;

/**
 * \brief ETHPHY driver configuration. These are filled by SysCfg based on the PHY
 *        device that is selected.
 */
typedef struct ETHPHY_Config_s
{
    ETHPHY_Attrs *attrs;     /**< ETHPHY device attributes */
    ETHPHY_Fxns  *fxns;      /**< ETHPHY device implementation functions */
    void         *object;    /**< ETHPHY driver object pointer */
} ETHPHY_Config;

/**
 * \brief Data structure to be passed/returned when calling \ref ETHPHY_command
 *        with \ref ETHPHY_CMD_GET_SPEED_AND_DUPLEX_CONFIG or
 *        \ref ETHPHY_CMD_SET_SPEED_AND_DUPLEX_CONFIG
 */
typedef struct ETHPHY_SpeedDuplexConfig_s
{
    uint32_t config; /**< Speed and Duplex Configuration. Allowed values are
                        \ref ETHPHY_SPEED_DUPLEX_CONFIGURATIONS */
} ETHPHY_SpeedDuplexConfig;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief Set default parameters in the \ref ETHPHY_Params structure
 *
 *  Call this API to set defaults and then override the fields as needed
 *  before calling \ref ETHPHY_open.
 *
 *  \param params   [OUT] Initialized parameters
 */
void ETHPHY_Params_init(ETHPHY_Params *params);

/**
 *  \brief Open ETHPHY driver
 *
 *  Global variables `ETHPHY_Config gETHPHY_Config[]` and
 *  `uint32_t gETHPHY_ConfigNum` is instantiated by SysCfg
 *  to describe the ETHPHY configuration based on user selection in SysCfg.
 *
 *  \param instanceId   [IN] Index within `ETHPHY_Config gETHPHY_Config[]`
 *                      denoting the ETHPHY driver to open
 *  \param params       [IN] Open parameters
 *
 *  \return Handle to ETHPHY driver which should be used in subsequent API call
 *          Else returns NULL in case of failure
 */
ETHPHY_Handle ETHPHY_open(uint32_t instanceId, const ETHPHY_Params *params);

/**
 * \brief Close ETHPHY driver
 *
 * \param handle    [in] ETHPHY driver handle from \ref ETHPHY_open
 */
void ETHPHY_close(ETHPHY_Handle handle);

/**
 * \brief Send a command to the ETHPHY
 *
 * \param handle    [in] ETHPHY driver handle from \ref ETHPHY_open
 * \param command   [in] Command from \ref ETHPHY_Commands
 *  \param data     [IN] Pointer to structure which has the data to write/read
 *  \param dataSize [IN] Size of the structure pointed by data
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t ETHPHY_command(ETHPHY_Handle handle,
                       uint32_t command,
                       void *data,
                       uint32_t dataSize);

/**
 * \brief Return ETHPHY attributes
 *
 * \param instanceId    [IN] ETHPHY instance ID
 *
 * \return \ref ETHPHY_Attrs, else NULL if instanceId is invalid
 */
const ETHPHY_Attrs *ETHPHY_getAttrs(uint32_t instanceId);

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef ETHPHY_H_ */

/** @} */
