/*!
* \file IOLM_Port_Sitara_soc.h
*
* \brief
* SOC specific IO Link functions
*
* \author
* KUNBUS GmbH
*
* \date
* 2021-05-19
*
* \copyright
* Copyright (c) 2021, KUNBUS GmbH<br /><br />
* All rights reserved.<br />
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:<br />
* <ol>
* <li>Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.</li>
* <li>Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.</li>
* <li>Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.</li>
* </ol>
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef IO_LINK_IOLINK_SOC_H_
#define IO_LINK_IOLINK_SOC_H_

#include <stdint.h>
#include <stdbool.h>
#include <osal.h>
#include <pru.h>
#include <IOLM_Phy.h>

#include "ti_drivers_config.h"
#include <drivers/gpio.h>
#include <drivers/pinmux.h>
#include "pru_IOLink.h"
#include "IOLM_Port_spi.h"

/** \brief register address offset between GPIO_SET_DATA01 and GPIO_SET_DATA23 */
#define SOC_GPIO_REG_BANK_OFFSET            (0x28)
/** \brief Number of pins per bank - shift value - used instead of divide operator */
#define SOC_GPIO_PINS_PER_BANK_SHIFT        (5U)
/** \brief Returns the bank index based on pin number */
#define SOC_GPIO_GET_BANK_INDEX(pinNum)     (((uint32_t) pinNum) >> SOC_GPIO_PINS_PER_BANK_SHIFT)
/** \brief Returns the base address offset based on pin number */
#define SOC_GPIO_GET_BASE_ADDR_OFS(pinNum)  (SOC_GPIO_GET_BANK_INDEX((uint32_t) pinNum) * SOC_GPIO_REG_BANK_OFFSET)
/** \brief Returns the bit mask within a bank based on pin number */
#define SOC_GPIO_GET_PIN_MASK(pinNum)       (((uint32_t) 1U) << (pinNum % 32))

// Modes to change pinmux
#define IOL_MUX_PRU_RX          (1U)
#define IOL_MUX_PRU_TX          (0U)

#define IOL_MUX_GPIO_RX         (7U)
#define IOL_MUX_GPIO_TX         (7U)

#define IOL_PRU_INSTANCE_MAX    CONFIG_PRUICSS_NUM_INSTANCES

#define IOLM_PHY_INTERFACE_SITARAPRU \
    .pfuInit                = NULL, \
    .pfuWakeup              = NULL, \
    .pfuSwitchPortPower     = NULL, \
    .pfuSetMode             = NULL, \
    .pfuSetIQMode           = NULL, \
    .pfuTransferPrepare     = NULL, \
    .pfuTransferTrig        = NULL, \
    .pfuSetDO               = NULL, \
    .pfuGetDI               = NULL, \
    .pfuSetIQ               = NULL, \
    .pfuGetIQ               = NULL, \
    .pfuSetFHCfg            = NULL, \
    .pfuSetPhyCycleTimer    = NULL

/**
\brief This enumeration indicates the requested CQ mode of the port.
*/
typedef enum IOLM_SOC_ECqMode
{
    /** \brief CQ disabled. == SIO disabled */
    IOLM_SOC_eCqMode_INACTIVE = 0,
    /** \brief CQ is in NPN mode. */
    IOLM_SOC_eCqMode_NPN,
    /** \brief CQ is in PNP mode. */
    IOLM_SOC_eCqMode_PNP,
    /** \brief CQ is in Push Pull mode. */
    IOLM_SOC_eCqMode_PUSHPULL,
    IOLM_SOC_eCqMode_FORCE32BIT = 0xffffffff
} IOLM_SOC_ECqMode_t;

/**
\brief Enumeration for binary power states
*/
typedef enum IOLM_SOC_EPowerState
{
    /** \brief Power off */
    IOLM_SOC_ePowerState_OFF        = 0,
    /** \brief Power on */
    IOLM_SOC_ePowerState_ON         = 1,
    IOLM_SOC_ePowerState_FORCE32BIT = 0xffffffff
} IOLM_SOC_EPowerState_t;

extern const IOLM_SPhyGeneric IOLM_SOC_phyPortCfgPru_g[IOLM_PORT_COUNT];

void IOLM_SOC_init();
void IOLM_SOC_setMode(uint8_t instance_p, uint8_t portNum_p, IOLM_PL_ePortMode_t mode_p);
const IOLM_PL_sPortConfig_t *IOLM_SOC_getPortCfg(uint8_t instance_p, uint8_t portNum_p);
const IOLM_PL_sPruIccsCfg_t *IOLM_SOC_getPruCfg(uint8_t instance_p);
void IOLM_SOC_setDO(uint8_t instance_p, uint8_t portNum_p, bool boPortTargetState_p);
bool IOLM_SOC_getDI(uint8_t instance_p, uint8_t portNum_p);
void IOLM_SOC_setPower(uint8_t instance_p, uint8_t portNum_p, bool powerState_p);
void IOLM_SOC_SetIQ(uint8_t instance_p, uint8_t portNum_p, bool boOutValue_p);
bool IOLM_SOC_GetIQ(uint8_t instance_p, uint8_t portNum_p);
void IOLM_SOC_SetIQMode(uint8_t instance_p, uint8_t portNum_p, IOL_EIQMode eIQMode_p);
OSAL_EError_t IOLM_SOC_checkInstPortValid(uint8_t instance_p, uint8_t portNum_p);
void IOLM_SOC_SetCqMode(INT8U port_p, IOLM_SOC_ECqMode_t eCqMode_p);
IOLM_SOC_ECqMode_t IOLM_SOC_GetCqMode(INT8U port_p);
#endif /* IO_LINK_IOLINK_SOC_H_ */
