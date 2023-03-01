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
 * \file  enetextphy.h
 *
 * \brief This file contains the type definitions and helper macros for the
 *        Ethernet PHY interface.
 */


#ifndef ENETEXTPHY_H_
#define ENETEXTPHY_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>
#include <kernel/dpl/SystemP.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/*! \brief Check if PHY address is valid (0 - 31). */
#define ENETEXTPHY_IS_ADDR_VALID(addr)           ((addr) <= 31U)

/*! \brief Macro to set bit at given bit position. */
#define ENETEXTPHY_BIT(n)                        (1U << (n))

/*! \brief Macro to check if bit at given bit position is set. */
#define ENETEXTPHY_IS_BIT_SET(val, n)            (((val) & ENETEXTPHY_BIT(n)) != 0U)

/*! \brief Macro to get the size of an array. */
#define ENETEXTPHY_ARRAYSIZE(x)                  (sizeof(x) / sizeof(x[0]))

#define ENETEXTPHY_TIMEOUT_MS (1000U * 5U)

/*!
 * \anchor EnetExtPhy_ErrorCodes
 * \name   Ethernet PHY driver error codes
 *
 * Error codes returned by the Ethernet PHY driver APIs.
 *
 * @{
 */

/* Ethernet PHY driver error codes are same as CSL's to maintain consistency */

/*! \brief Success. */
#define ENETEXTPHY_SOK                           (SystemP_SUCCESS)

/*! \brief Generic failure error condition (typically caused by hardware). */
#define ENETEXTPHY_EFAIL                         (SystemP_FAILURE)

/*! \brief Time out while waiting for a given condition to happen. */
#define ENETEXTPHY_ETIMEOUT                      (SystemP_TIMEOUT)

/*! \brief Bad arguments (i.e. NULL pointer). */
#define ENETEXTPHY_EBADARGS                      (SystemP_TIMEOUT - 1)

/*! \brief Invalid parameters (i.e. value out-of-range). */
#define ENETEXTPHY_EINVALIDPARAMS                (SystemP_TIMEOUT - 2)

/*! \brief Allocation failure. */
#define ENETEXTPHY_EALLOC                        (SystemP_TIMEOUT - 3)

/*! \brief Operation not permitted. */
#define ENETEXTPHY_EPERM                         (SystemP_TIMEOUT - 4)

/*! \brief Operation not supported. */
#define ENETEXTPHY_ENOTSUPPORTED                 (SystemP_TIMEOUT - 5)

/*! @} */

/*!
 * \anchor EnetExtPhy_LinkCaps
 * \name   Ethernet PHY link capability masks
 *
 * Error codes returned by the Ethernet PHY driver APIs.
 *
 * @{
 */

/*! \brief 10-Mbps, half-duplex capability mask. */
#define ENETEXTPHY_LINK_CAP_HD10                 ENETEXTPHY_BIT(1)

/*! \brief 10-Mbps, full-duplex capability mask. */
#define ENETEXTPHY_LINK_CAP_FD10                 ENETEXTPHY_BIT(2)

/*! \brief 100-Mbps, half-duplex capability mask. */
#define ENETEXTPHY_LINK_CAP_HD100                ENETEXTPHY_BIT(3)

/*! \brief 100-Mbps, full-duplex capability mask. */
#define ENETEXTPHY_LINK_CAP_FD100                ENETEXTPHY_BIT(4)

/*! \brief 1-Gbps, half-duplex capability mask. */
#define ENETEXTPHY_LINK_CAP_HD1000               ENETEXTPHY_BIT(5)

/*! \brief 1-Gbps, full-duplex capability mask. */
#define ENETEXTPHY_LINK_CAP_FD1000               ENETEXTPHY_BIT(6)

/*! \brief 10-Mbps, full and half-duplex capability mask. */
#define ENETEXTPHY_LINK_CAP_10                   (ENETEXTPHY_LINK_CAP_HD10 | \
                                               ENETEXTPHY_LINK_CAP_FD10)

/*! \brief 100-Mbps, full and half-duplex capability mask. */
#define ENETEXTPHY_LINK_CAP_100                  (ENETEXTPHY_LINK_CAP_HD100 | \
                                               ENETEXTPHY_LINK_CAP_FD100)

/*! \brief 1-Gbps, full and half-duplex capability mask. */
#define ENETEXTPHY_LINK_CAP_1000                 (ENETEXTPHY_LINK_CAP_HD1000 | \
                                               ENETEXTPHY_LINK_CAP_FD1000)

/*! \brief Auto-negotiation mask with all duplexity and speed values set. */
#define ENETEXTPHY_LINK_CAP_ALL                  (ENETEXTPHY_LINK_CAP_HD10 |   \
                                               ENETEXTPHY_LINK_CAP_FD10 |   \
                                               ENETEXTPHY_LINK_CAP_HD100 |  \
                                               ENETEXTPHY_LINK_CAP_FD100 |  \
                                               ENETEXTPHY_LINK_CAP_HD1000 | \
                                               ENETEXTPHY_LINK_CAP_FD1000)

/*! @} */

/*! \brief Max extended configuration size, arbitrarily chosen. */
#define ENETEXTPHY_EXTENDED_CFG_SIZE_MAX         (128U)

/*! \brief Invalid PHY address indicator. */
#define ENETEXTPHY_INVALID_PHYADDR               (~0U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/*!
 * \brief EnetExtPhy driver magic value, used to indicate if driver is open or not.
 */
typedef enum EnetExtPhy_Magic_e
{
    /*! Magic number used to identify when driver has been opened. */
    ENETEXTPHY_MAGIC = 0xCADACADAU,

    /*! Magic number used to identify when driver is closed. */
    ENETEXTPHY_NO_MAGIC = 0x0U,
} EnetExtPhy_Magic;

/*!
 * \brief MAC Media-Independent Interface (MII).
 */
typedef enum EnetExtPhy_Mii_e
{
    /*! \brief MII interface */
    ENETEXTPHY_MAC_MII_MII = 0U,

    /*! \brief RMII interface */
    ENETEXTPHY_MAC_MII_RMII,

    /*! \brief GMII interface */
    ENETEXTPHY_MAC_MII_GMII,

    /*! \brief RGMII interface */
    ENETEXTPHY_MAC_MII_RGMII,

    /*! \brief SGMII interface */
    ENETEXTPHY_MAC_MII_SGMII,

    /*! \brief QSGMII interface */
    ENETEXTPHY_MAC_MII_QSGMII,
} EnetExtPhy_Mii;

/*!
 * \brief MAC interface speed.
 */
typedef enum EnetExtPhy_Speed_e
{
    /*! 10 Mbps */
    ENETEXTPHY_SPEED_10MBIT = 0U,

    /*! 100 Mbps */
    ENETEXTPHY_SPEED_100MBIT,

    /*! 1 Gbps */
    ENETEXTPHY_SPEED_1GBIT,

    /*! Speed determined automatically */
    ENETEXTPHY_SPEED_AUTO,
} EnetExtPhy_Speed;

/*!
 * \brief MAC interface duplexity.
 */
typedef enum EnetExtPhy_Duplexity_e
{
    /*! Half duplex */
    ENETEXTPHY_DUPLEX_HALF = 0U,

    /*! Full duplex */
    ENETEXTPHY_DUPLEX_FULL,

    /*! Duplexity determined automatically */
    ENETEXTPHY_DUPLEX_AUTO,
} EnetExtPhy_Duplexity;

/*!
 * \brief PHY version (ID).
 */
typedef struct EnetExtPhy_Version_s
{
    /*! Organizationally Unique Identifier (OUI) */
    uint32_t oui;

    /*! Manufacturer's model number */
    uint32_t model;

    /*! Revision number */
    uint32_t revision;
} EnetExtPhy_Version;

/*!
 * \brief PHY link status.
 */
typedef enum EnetExtPhy_LinkStatus_e
{
    /*! PHY got link up */
    ENETEXTPHY_GOT_LINK = 0U,

    /*! PHY link is still up */
    ENETEXTPHY_LINK_UP,

    /*! PHY lost link */
    ENETEXTPHY_LOST_LINK,

    /*! PHY link is still down */
    ENETEXTPHY_LINK_DOWN,
} EnetExtPhy_LinkStatus;

/*!
 * \brief Link speed and duplexity configuration.
 */
typedef struct EnetExtPhy_LinkCfg_s
{
    /*! Link speed */
    EnetExtPhy_Speed speed;

    /*! Duplexity */
    EnetExtPhy_Duplexity duplexity;
} EnetExtPhy_LinkCfg;


/*!
 * \brief PHY configuration parameters.
 */
typedef struct EnetExtPhy_Cfg_s
{
    /*! MDIO group */
    uint32_t phyGroup;

    /*! PHY device address */
    uint32_t phyAddr;

    /*! Auto-negotiation advertise capabilities */
    uint32_t nwayCaps;

    /*! MDIX enable */
    bool mdixEn;

    /*! Enable master mode */
    bool masterMode;

    /*! Enable external clock source */
    bool extClkSource;

    /*! Skip PHY-specific extended configuration */
    bool skipExtendedCfg;

    /*! Extended PHY-specific configuration */
    uint8_t extendedCfg[ENETEXTPHY_EXTENDED_CFG_SIZE_MAX];

    /*! Size of the extended configuration */
    uint32_t extendedCfgSize;
} EnetExtPhy_Cfg;

/*!
 * \brief MDIO driver.
 */
typedef struct EnetExtPhy_Mdio_s
{
    /*!
     * \brief Check if PHY is alive.
     *
     * Checks if PHY is alive, either using an explicit register read or any other
     * mechanism supported by the MDIO peripheral (i.e. background BMSR reads).
     *
     * \param phyAddr    PHY device address
     * \param isAlive    Whether PHY is alive or not
     * \param args       Caller's arguments passed to the PHY driver at open time
     *
     * \return \ref EnetExtPhy_ErrorCodes
     */
    int32_t (*isAlive)(uint32_t phyAddr,
                       bool *isAlive,
                       void *arg);

    /*!
     * \brief Check if PHY is linked.
     *
     * Checks if PHY is linked, either using an explicit register read or any other
     * mechanism supported by the MDIO peripheral.
     *
     * \param phyAddr    PHY device address
     * \param isLinked   Whether PHY is linked or not
     * \param args       Caller's arguments passed to the PHY driver at open time
     *
     * \return \ref EnetExtPhy_ErrorCodes
     */
    int32_t (*isLinked)(uint32_t phyAddr,
                        bool *isLinked,
                        void *arg);

    /*!
     * \brief Read PHY register using Clause-22 frame.
     *
     * Reads a PHY register using a Clause-22 frame.
     *
     * \param group      User group (use 0 if single group is supported)
     * \param phyAddr    PHY device address
     * \param reg        Register address
     * \param val        Value read from register
     * \param args       Caller's arguments passed to the PHY driver at open time
     *
     * \return \ref EnetExtPhy_ErrorCodes
     */
    int32_t (*readC22)(uint32_t group,
                       uint32_t phyAddr,
                       uint32_t reg,
                       uint16_t *val,
                       void *arg);

    /*!
     * \brief Write PHY register using Clause-22 frame.
     *
     * Writes a PHY register using a Clause-22 frame.
     *
     * \param group      User group (use 0 if single group is supported)
     * \param phyAddr    PHY device address
     * \param reg        Register address
     * \param val        Value to be written
     * \param args       Caller's arguments passed to the PHY driver at open time
     *
     * \return \ref EnetExtPhy_ErrorCodes
     */
    int32_t (*writeC22)(uint32_t group,
                        uint32_t phyAddr,
                        uint32_t reg,
                        uint16_t val,
                        void *arg);

    /*!
     * \brief Read PHY register using Clause-45 frame.
     *
     * Reads a PHY register using a Clause-45 frame.  Returns #ENETEXTPHY_ENOTSUPPORTED
     * if MDIO doesn't support Clause-45 frames.
     *
     * \param group      User group (use 0 if single group is supported)
     * \param phyAddr    PHY device address
     * \param reg        Register address
     * \param val        Value read from register
     * \param args       Caller's arguments passed to the PHY driver at open time
     *
     * \return \ref EnetExtPhy_ErrorCodes
     */
    int32_t (*readC45)(uint32_t group,
                       uint32_t phyAddr,
                       uint8_t mmd,
                       uint16_t reg,
                       uint16_t *val,
                       void *arg);

    /*!
     * \brief Write PHY register using Clause-45 frame.
     *
     * Writes a PHY register using a Clause-45 frame. Returns #ENETEXTPHY_ENOTSUPPORTED
     * if MDIO doesn't support Clause-45 frames.
     *
     * \param group      User group (use 0 if single group is supported)
     * \param phyAddr    PHY device address
     * \param reg        Register address
     * \param val        Value to be written
     * \param args       Caller's arguments passed to the PHY driver at open time
     *
     * \return \ref EnetExtPhy_ErrorCodes
     */
    int32_t (*writeC45)(uint32_t group,
                        uint32_t phyAddr,
                        uint8_t mmd,
                        uint16_t reg,
                        uint16_t val,
                        void *arg);
} EnetExtPhy_Mdio;

/*!
 * \brief MDIO driver handle.
 */
typedef EnetExtPhy_Mdio *EnetExtPhy_MdioHandle;

/*!
 * \brief PHY specific driver handle.
 */
typedef struct EnetExtPhy_Drv_s *EnetExtPhyDrv_Handle;


/*!
 * \brief PHY driver FSM state.
 */
typedef struct EnetExtPhy_State_s
{
    /*! PHY speed (auto-negotiated or manually set) */
    EnetExtPhy_Speed speed;

    /*! PHY duplexity (auto-negotiated or manually set) */
    EnetExtPhy_Duplexity duplexity;

    /*! Timeout (ticks) */
    uint32_t timeout;

    /*! Residence time (ticks) */
    uint32_t residenceTime;

    /*! Whether PHY is auto-negotiation capable */
    bool isNwayCapable;

    /*! Whether auto-negotiation is to be used or not */
    bool enableNway;

    /*! Whether manual mode needs to be configured */
    bool needsManualCfg;

    /*! Whether auto-negotiation advertisement needs to be configured */
    bool needsNwayCfg;

    /*! Refined link capability mask (app, SoC, PHY) */
    uint32_t linkCaps;

    /*! PHY link capability mask */
    uint32_t phyLinkCaps;

    /*! Whether PHY loopback is enabled or not */
    bool loopbackEn;

    /*! Whether MDIX switch is needed or not */
    bool needsMdixSwitch;

    /*! Whether MDIX is enabled or not (MDI) */
    bool enableMdix;
} EnetExtPhy_State;

/*
 * \brief PHY driver object.
 */
typedef struct EnetExtPhy_Obj_s
{
    /*! MDIO handle used to access PHY registers */
    EnetExtPhy_MdioHandle hMdio;

    /*! PHY configuration params */
    EnetExtPhy_Cfg phyCfg;

    /*! MII interface type */
    EnetExtPhy_Mii mii;

    /*! MAC port supported capabilities */
    uint32_t macCaps;

    /*! Port Link configuration (speed, duplexity) */
    EnetExtPhy_LinkCfg linkCfg;


    /*! State-machine state */
    EnetExtPhy_State state;
    /*! PHY group */
    uint32_t group;

    /*! PHY device address */
    uint32_t addr;

    /*! Requested link capability mask */
    uint32_t reqLinkCaps;

    /*! PHY driver */
    EnetExtPhyDrv_Handle hDrv;

    /*! Magic number indicating that this object is in use */
    EnetExtPhy_Magic magic;

    /*! Caller-provided arguments to be used in MDIO driver calls */
    void *mdioArgs;
} EnetExtPhy_Obj;

/*!
 * \brief PHY driver object handle.
 *
 * PHY driver opaque handle used to call any PHY related APIs.
 */
typedef struct EnetExtPhy_Obj_s *EnetExtPhy_Handle;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief Initialize PHY config params.
 *
 * Initializes PHY driver configuration parameters.
 *
 * \param phyCfg   PHY configuration params
 */
void EnetExtPhy_initCfg(EnetExtPhy_Cfg *phyCfg);

/*!
 * \brief Set PHY extended parameters.
 *
 * Sets the PHY-specific extended parameters to the PHY config structure.
 *
 * \param phyCfg            Pointer to the PHY config
 * \param extendedCfg       Pointer to the PHY extended config
 * \param extendedCfgSize   Size of the PHY extended config
 */
void EnetExtPhy_setExtendedCfg(EnetExtPhy_Cfg *phyCfg,
                            const void *extendedCfg,
                            uint32_t extendedCfgSize);

/*!
 * \brief Open the PHY driver.
 *
 * Open the Ethernet PHY driver for the given MAC port number. The PHY driver
 * takes PHY specific configuration parameters, the MAC port type connection
 * and the desired link configuration (auto or manual).
 *
 * \param phyCfg      PHY configuration params
 * \param mii         PHY MII interface type
 * \param linkCfg     Link configuration (speed and duplexity)
 * \param macPortCaps MAC port speed/duplex capabilities. It's a bit mask of
 *                    \ref EnetExtPhy_LinkCaps
 * \param hMdio       MDIO driver to be used for PHY register read/write
 * \param mdioArgs    Private data passed to the MDIO driver functions
 *
 * \return PHY device handle if successful, NULL otherwise.
 */
EnetExtPhy_Handle EnetExtPhy_open(const EnetExtPhy_Cfg *phyCfg,
                            EnetExtPhy_Mii mii,
                            const EnetExtPhy_LinkCfg *linkCfg,
                            uint32_t macPortCaps,
                            EnetExtPhy_MdioHandle hMdio,
                            void *mdioArgs);

/*!
 * \brief Close the PHY driver.
 *
 * Closes the Ethernet PHY driver.
 *
 * \param hPhy     PHY device handle
 */
void EnetExtPhy_close(EnetExtPhy_Handle hPhy);

/*!
 * \brief Get PHY id.
 *
 * Gets the device ID of a PHY, read from IDR1 and IDR2 registers.
 *
 * \param hPhy     PHY device handle
 * \param version  Pointer to PHY version.
 *
 * \return \ref EnetExtPhy_ErrorCodes
 */
int32_t EnetExtPhy_getId(EnetExtPhy_Handle hPhy,
                      EnetExtPhy_Version *version);

/*!
 * \brief Get PHY alive status.
 *
 * Gets the PHY alive status. Whether PHY is responding to read accesses.
 *
 * \param hPhy     PHY device handle
 *
 * \return true if PHY is alive, false otherwise
 */
bool EnetExtPhy_isAlive(EnetExtPhy_Handle hPhy);

/*!
 * \brief Get link status.
 *
 * Gets the link status: linked or not, based on driver's state machine.
 * The PHY driver state machine can take a little longer to detect link up
 * because it runs on tick period intervals and need to traverse few states
 * to reach link up FSM state.
 *
 * \param hPhy     PHY device handle
 *
 * \return true if PHY is linked, false otherwise
 */
bool EnetExtPhy_isLinked(EnetExtPhy_Handle hPhy);

/*!
 * \brief Get link configuration.
 *
 * Gets the link configuration, that is, the configuration that the PHY has
 * negotiated with the link partner or the manual link configuration it was
 * set to.
 *
 * \param hPhy     PHY device handle
 * \param linkCfg  Link configuration
 *
 * \return \ref EnetExtPhy_ErrorCodes
 */
int32_t EnetExtPhy_getLinkCfg(EnetExtPhy_Handle hPhy,
                           EnetExtPhy_LinkCfg *linkCfg);

/*!
 * \brief Read PHY register.
 *
 * Reads a PHY register. It's not meant for extended registers.
 *
 * \param hPhy     PHY device handle
 * \param reg      Register number
 * \param val      Pointer to the read value
 *
 * \return \ref EnetExtPhy_ErrorCodes
 */
int32_t EnetExtPhy_readReg(EnetExtPhy_Handle hPhy,
                        uint32_t reg,
                        uint16_t *val);

/*!
 * \brief Write PHY register.
 *
 * Writes a PHY register. It's not meant for extended registers.
 *
 * \param hPhy     PHY device handle
 * \param reg      Register number
 * \param val      Value to be written
 *
 * \return \ref EnetExtPhy_ErrorCodes
 */
int32_t EnetExtPhy_writeReg(EnetExtPhy_Handle hPhy,
                         uint32_t reg,
                         uint16_t val);

/*!
 * \brief Read-modify-write PHY register.
 *
 * Read-modify-write a PHY register. It's not meant for extended registers.
 *
 * \param hPhy     PHY device handle
 * \param reg      Register number
 * \param mask     Bitmask to be applied on read value and value to be written
 * \param val      Value to be written
 *
 * \return \ref EnetExtPhy_ErrorCodes
 */
int32_t EnetExtPhy_rmwReg(EnetExtPhy_Handle hPhy,
                       uint32_t reg,
                       uint16_t mask,
                       uint16_t val);

/*!
 * \brief Read PHY extended register.
 *
 * Reads a PHY extended register.
 *
 * \param hPhy     PHY device handle
 * \param reg      Register number
 * \param val      Pointer to the read value
 *
 * \return \ref EnetExtPhy_ErrorCodes
 */
int32_t EnetExtPhy_readExtReg(EnetExtPhy_Handle hPhy,
                           uint32_t reg,
                           uint16_t *val);

/*!
 * \brief Write PHY extended register.
 *
 * Writes a PHY extended register.
 *
 * \param hPhy     PHY device handle
 * \param reg      Register number
 * \param val      Value to be written
 *
 * \return \ref EnetExtPhy_ErrorCodes
 */
int32_t EnetExtPhy_writeExtReg(EnetExtPhy_Handle hPhy,
                            uint32_t reg,
                            uint16_t val);

/*!
 * \brief Read-modify-write PHY extended register.
 *
 * Read-modify-write a PHY extended register.
 *
 * \param hPhy     PHY device handle
 * \param reg      Register number
 * \param mask     Bitmask to be applied on read value and value to be written
 * \param val      Value to be written
 *
 * \return \ref EnetExtPhy_ErrorCodes
 */
int32_t EnetExtPhy_rmwExtReg(EnetExtPhy_Handle hPhy,
                          uint32_t reg,
                          uint16_t mask,
                          uint16_t val);

/*!
 * \brief Read PHY register using Clause-45 frame.
 *
 * Reads a PHY register using Clause-45 frame.
 *
 * \param hPhy     PHY device handle
 * \param mmd      MMD
 * \param reg      Register number
 * \param val      Pointer to the read value
 *
 * \return \ref EnetExtPhy_ErrorCodes
 */
int32_t EnetExtPhy_readC45Reg(EnetExtPhy_Handle hPhy,
                           uint8_t mmd,
                           uint32_t reg,
                           uint16_t *val);

/*!
 * \brief Write PHY register using Clause-45 frame.
 *
 * Writes a PHY register using Clause-45 frame.
 *
 * \param hPhy     PHY device handle
 * \param mmd      MMD
 * \param reg      Register number
 * \param val      Value to be written
 *
 * \return \ref EnetExtPhy_ErrorCodes
 */
int32_t EnetExtPhy_writeC45Reg(EnetExtPhy_Handle hPhy,
                            uint8_t mmd,
                            uint32_t reg,
                            uint16_t val);

/*!
 * \brief Read-modify-write PHY register using Clause-45 frame.
 *
 * Read-modify-write a PHY register using Clause-45 frame.
 *
 * \param hPhy     PHY device handle
 * \param mmd      MMD
 * \param reg      Register number
 * \param mask     Bitmask to be applied on read value and value to be written
 * \param val      Value to be written
 *
 * \return \ref EnetExtPhy_ErrorCodes
 */
int32_t EnetExtPhy_rmwC45Reg(EnetExtPhy_Handle hPhy,
                          uint8_t mmd,
                          uint32_t reg,
                          uint16_t mask,
                          uint16_t val);

/*!
 * \brief Print all PHY registers.
 *
 * Prints all registers of a PHY.
 *
 * \param hPhy     PHY device handle
 */
void EnetExtPhy_printRegs(EnetExtPhy_Handle hPhy);

bool EnetExtPhy_WaitForLinkUp(EnetExtPhy_Handle hPhy, uint32_t timeoutMs);

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

#endif /* ENETEXTPHY_H_ */

/*! @} */
