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
 *  \defgroup DRV_MDIO_MODULE APIs for MDIO
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the MDIO module.
 *
 *  @{
 */

#ifndef MDIO_H_
#define MDIO_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <drivers/hw_include/cslr_mdio.h>
#include <kernel/dpl/SystemP.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  \anchor MDIO_LinkSelModes
 *  \name MDIO Link Status Determination Select
 *
 *  @{
 */
/**Macro to select MDIO mode of Link detection*/
#define MDIO_LINKSEL_MDIO_MODE      (0)
/**Macro to select MLINK mode of Link detection*/
#define MDIO_LINKSEL_MLINK_MODE     (1)
/** @} */

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
 *  \brief  This function initializes the MDIO clock
 *
 * \param   baseAddr Base Address of the MDIO module
 *
 * \return  #SystemP_SUCCESS in case of success, #SystemP_FAILURE otherwise
 */
int32_t MDIO_initClock(uint32_t baseAddr);

/**
 * \brief   This API reads an IEEE defined PHY register using MDIO. It should be
 *          called only after successful execution of #MDIO_initClock().
 *
 * \param   baseAddr    Base Address of the MDIO module
 * \param   pUserGroup  Pointer to uint32_t storing user group to be used while
 *                      accessing PHY registers. Pass NULL for using default value
 * \param   phyAddr     PHY Address
 * \param   regNum      Register Number to be read.
 * \param   pData       Pointer where the read value shall be written
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t MDIO_phyRegRead(uint32_t baseAddr,
                        void     *pUserGroup,
                        uint32_t phyAddr,
                        uint32_t regNum,
                        uint16_t *pData);

/**
 * \brief   This API writes an IEEE defined PHY register using MDIO. It should be
 *          called only after successful execution of #MDIO_initClock().
 *
 * \param   baseAddr    Base Address of the MDIO module
 * \param   pUserGroup  Pointer to uint32_t storing user group to be used while
 *                      accessing PHY registers. Pass NULL for using default value
 * \param   phyAddr     PHY Address
 * \param   regNum      Register Number to be written
 * \param   wrVal       Value to be written
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t MDIO_phyRegWrite(uint32_t baseAddr,
                         void     *pUserGroup,
                         uint32_t phyAddr,
                         uint32_t regNum,
                         uint16_t wrVal);
/**
 * \brief   This API reads an extended PHY register using MDIO. It should be
 *          called only after successful execution of #MDIO_initClock().
 *
 * \param   baseAddr    Base Address of the MDIO module
 * \param   pUserGroup  Pointer to uint32_t storing user group to be used while
 *                      accessing PHY registers. Pass NULL for using default value
 * \param   phyAddr     PHY Address
 * \param   regNum      Register Number to be read.
 * \param   pData       Pointer where the read value shall be written
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t MDIO_phyExtRegRead(uint32_t baseAddr,
                           void     *pUserGroup,
                           uint32_t phyAddr,
                           uint32_t regNum,
                           uint16_t *pData);

/**
 * \brief   This API writes an extended PHY register using MDIO. It should be
 *          called only after successful execution of #MDIO_initClock().
 *
 * \param   baseAddr    Base Address of the MDIO module
 * \param   pUserGroup  Pointer to uint32_t storing user group to be used while
 *                      accessing PHY registers. Pass NULL for using default value
 * \param   phyAddr     PHY Address
 * \param   regNum      Register Number to be written
 * \param   wrVal       Value to be written
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t MDIO_phyExtRegWrite(uint32_t baseAddr,
                            void     *pUserGroup,
                            uint32_t phyAddr,
                            uint32_t regNum,
                            uint16_t wrVal);

/**
 * \brief   This API reads the link status of all PHY connected to this MDIO.
 *          The bit corresponding to the PHY address will be set if the PHY
 *          link is active.
 *
 * \param   baseAddr    Base Address of the MDIO module
 * \param   phyAddr     PHY Address
 *
 * \return SystemP_SUCCESS if link is up, else failure
 */
int32_t MDIO_phyLinkStatus(uint32_t baseAddr, uint32_t phyAddr);

/**
* \brief Function to enable the MDIO Link change interrupt.
*
*
* \param    mdioBaseAddress    Base Address of the MDIO module
* \param    regInst            0/1 to select the MDIO User PHY Select Register
* \param    phyAddr            PHY Address
* \param    linkSel            Flag to select to use MDIO mode or MLINK mode.
*                              Allowed values are \ref MDIO_LinkSelModes
*
 * \return SystemP_SUCCESS if link is up, else failure
 */
int32_t MDIO_enableLinkInterrupt(uint32_t mdioBaseAddress,
                                 uint32_t regInst,
                                 uint32_t phyAddr,
                                 uint8_t linkSel);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* #ifndef MDIO_H_ */
