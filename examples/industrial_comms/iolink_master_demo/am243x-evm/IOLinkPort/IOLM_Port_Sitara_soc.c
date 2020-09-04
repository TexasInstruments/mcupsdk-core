/*!
* \file IOLM_Port_Sitara_soc.c
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


#include "IOLM_Port_Sitara_soc.h"


#define IOLM_SOC_PRU_COUNT (sizeof(iolPruSetup_g)/sizeof(iolPruSetup_g[0]))

const IOLM_PL_sPruIccsCfg_t iolPruSetup_g[] =
{
    {
        .pruIcssInst = CONFIG_PRU_ICSS0,
        {
            {.pruIrqNum = 120, .pruIrqPrio = 1},
            {.pruIrqNum = 121, .pruIrqPrio = 1},
            {.pruIrqNum = 0, .pruIrqPrio = 0},
            {.pruIrqNum = 0, .pruIrqPrio = 0},
            {.pruIrqNum = 0, .pruIrqPrio = 0},
            {.pruIrqNum = 0, .pruIrqPrio = 0},
            {.pruIrqNum = 0, .pruIrqPrio = 0},
            {.pruIrqNum = 0, .pruIrqPrio = 0},
        }
    }
};

#define IOLM_SOC_PORT_COUNT (uint8_t)(sizeof(iolPinSetup_g)/sizeof(iolPinSetup_g[0]))

IOLM_SOC_ECqMode_t iolEqMode_g[IOLM_PORT_COUNT];

const IOLM_PL_sPortConfig_t iolPinSetup_g[] =
{
 { // Port 0
    .rx = {
        .ctlRegOffset = PIN_PRG0_PRU0_GPO0,
        .gpioBase = CONFIG_IOL_RX1_BASE_ADDR,
        .gpioPin = CONFIG_IOL_RX1_PIN
    },
    .tx = {
        .gpio.ctlRegOffset = PIN_PRG0_PRU0_GPO8,
        .gpio.gpioBase = CONFIG_IOL_TX1_BASE_ADDR,
        .gpio.gpioPin = CONFIG_IOL_TX1_PIN,
        .pruPin = 8,
    },
    .txEn = {
        .gpio.ctlRegOffset = PIN_PRG0_PRU1_GPO6,
        .gpio.gpioBase = CONFIG_IOL_TXEN1_BASE_ADDR,
        .gpio.gpioPin = CONFIG_IOL_TXEN1_PIN,
        .gpioPhysAddr = CONFIG_IOL_TXEN1_BASE_ADDR + SOC_GPIO_GET_BASE_ADDR_OFS(CONFIG_IOL_TXEN1_PIN),
        .gpioPinMask = SOC_GPIO_GET_PIN_MASK(CONFIG_IOL_TXEN1_PIN),
    },
    .pwrEn = {
        .ctlRegOffset = PIN_PRG0_PRU0_GPO18,
        .gpioBase = CONFIG_IOL_ENL1_BASE_ADDR,
        .gpioPin = CONFIG_IOL_ENL1_PIN
    }
 },
 { // Port 1
    .rx = {
        .ctlRegOffset = PIN_PRG0_PRU0_GPO1,
        .gpioBase = CONFIG_IOL_RX2_BASE_ADDR,
        .gpioPin = CONFIG_IOL_RX2_PIN
    },
    .tx = {
        .gpio.ctlRegOffset = PIN_PRG0_PRU0_GPO11,
        .gpio.gpioBase = CONFIG_IOL_TX2_BASE_ADDR,
        .gpio.gpioPin = CONFIG_IOL_TX2_PIN,
        .pruPin = 11,
    },
    .txEn = {
        .gpio.ctlRegOffset = PIN_PRG0_PRU1_GPO8,
        .gpio.gpioBase = CONFIG_IOL_TXEN2_BASE_ADDR,
        .gpio.gpioPin = CONFIG_IOL_TXEN2_PIN,
        .gpioPhysAddr = CONFIG_IOL_TXEN2_BASE_ADDR + SOC_GPIO_GET_BASE_ADDR_OFS(CONFIG_IOL_TXEN2_PIN),
        .gpioPinMask = SOC_GPIO_GET_PIN_MASK(CONFIG_IOL_TXEN2_PIN),
    },
    .pwrEn = {
        .ctlRegOffset = PIN_PRG0_PRU0_GPO19,
        .gpioBase = CONFIG_IOL_ENL2_BASE_ADDR,
        .gpioPin = CONFIG_IOL_ENL2_PIN
    }
 },
 { // Port 2
    .rx = {
        .ctlRegOffset = PIN_PRG0_PRU0_GPO2,
        .gpioBase = CONFIG_IOL_RX3_BASE_ADDR,
        .gpioPin = CONFIG_IOL_RX3_PIN
    },
    .tx = {
        .gpio.ctlRegOffset = PIN_PRG0_PRU0_GPO12,
        .gpio.gpioBase = CONFIG_IOL_TX3_BASE_ADDR,
        .gpio.gpioPin = CONFIG_IOL_TX3_PIN,
        .pruPin = 12,
    },
    .txEn = {
        .gpio.ctlRegOffset = PIN_PRG0_PRU1_GPO11,
        .gpio.gpioBase = CONFIG_IOL_TXEN3_BASE_ADDR,
        .gpio.gpioPin = CONFIG_IOL_TXEN3_PIN,
        .gpioPhysAddr = CONFIG_IOL_TXEN3_BASE_ADDR + SOC_GPIO_GET_BASE_ADDR_OFS(CONFIG_IOL_TXEN3_PIN),
        .gpioPinMask = SOC_GPIO_GET_PIN_MASK(CONFIG_IOL_TXEN3_PIN),
    },
  . pwrEn = {
        .ctlRegOffset = PIN_PRG0_PRU1_GPO0,
        .gpioBase = CONFIG_IOL_ENL3_BASE_ADDR,
        .gpioPin = CONFIG_IOL_ENL3_PIN
    }
 },
 { // Port 3
    .rx = {
        .ctlRegOffset = PIN_PRG0_PRU0_GPO3,
        .gpioBase = CONFIG_IOL_RX4_BASE_ADDR,
        .gpioPin = CONFIG_IOL_RX4_PIN
     },
    .tx = {
        .gpio.ctlRegOffset = PIN_PRG0_PRU0_GPO13,
        .gpio.gpioBase = CONFIG_IOL_TX4_BASE_ADDR,
        .gpio.gpioPin = CONFIG_IOL_TX4_PIN,
        .pruPin = 13,
    },
    .txEn = {
        .gpio.ctlRegOffset = PIN_PRG0_PRU1_GPO12,
        .gpio.gpioBase = CONFIG_IOL_TXEN4_BASE_ADDR,
        .gpio.gpioPin = CONFIG_IOL_TXEN4_PIN,
        .gpioPhysAddr = CONFIG_IOL_TXEN4_BASE_ADDR + SOC_GPIO_GET_BASE_ADDR_OFS(CONFIG_IOL_TXEN4_PIN),
        .gpioPinMask = SOC_GPIO_GET_PIN_MASK(CONFIG_IOL_TXEN4_PIN),
    },
    .pwrEn = {
        .ctlRegOffset = PIN_PRG0_PRU1_GPO1,
        .gpioBase = CONFIG_IOL_ENL4_BASE_ADDR,
        .gpioPin = CONFIG_IOL_ENL4_PIN
    }
 },
 { // Port 4
    .rx = {
        .ctlRegOffset = PIN_PRG0_PRU0_GPO4,
        .gpioBase = CONFIG_IOL_RX5_BASE_ADDR,
        .gpioPin = CONFIG_IOL_RX5_PIN
    },
    .tx = {
        .gpio.ctlRegOffset = PIN_PRG0_PRU0_GPO14,
        .gpio.gpioBase = CONFIG_IOL_TX5_BASE_ADDR,
        .gpio.gpioPin = CONFIG_IOL_TX5_PIN,
        .pruPin = 14,
    },
    .txEn = {
        .gpio.ctlRegOffset = PIN_PRG0_PRU1_GPO13,
        .gpio.gpioBase = CONFIG_IOL_TXEN5_BASE_ADDR,
        .gpio.gpioPin = CONFIG_IOL_TXEN5_PIN,
        .gpioPhysAddr = CONFIG_IOL_TXEN5_BASE_ADDR + SOC_GPIO_GET_BASE_ADDR_OFS(CONFIG_IOL_TXEN5_PIN),
        .gpioPinMask = SOC_GPIO_GET_PIN_MASK(CONFIG_IOL_TXEN5_PIN),
    },
    .pwrEn = {
        .ctlRegOffset = PIN_PRG0_PRU1_GPO2,
        .gpioBase = CONFIG_IOL_ENL5_BASE_ADDR,
        .gpioPin = CONFIG_IOL_ENL5_PIN
    }
 },
 { // Port 5
    .rx = {
        .ctlRegOffset = PIN_PRG0_PRU0_GPO5,
        .gpioBase = CONFIG_IOL_RX6_BASE_ADDR,
        .gpioPin = CONFIG_IOL_RX6_PIN
    },
    .tx = {
        .gpio.ctlRegOffset = PIN_PRG0_PRU0_GPO15,
        .gpio.gpioBase = CONFIG_IOL_TX6_BASE_ADDR,
        .gpio.gpioPin = CONFIG_IOL_TX6_PIN,
        .pruPin = 15,
    },
    .txEn = {
        .gpio.ctlRegOffset = PIN_PRG0_PRU1_GPO14,
        .gpio.gpioBase = CONFIG_IOL_TXEN6_BASE_ADDR,
        .gpio.gpioPin = CONFIG_IOL_TXEN6_PIN,
        .gpioPhysAddr = CONFIG_IOL_TXEN6_BASE_ADDR + SOC_GPIO_GET_BASE_ADDR_OFS(CONFIG_IOL_TXEN6_PIN),
        .gpioPinMask = SOC_GPIO_GET_PIN_MASK(CONFIG_IOL_TXEN6_PIN),
    },
    .pwrEn = {
        .ctlRegOffset = PIN_PRG0_PRU1_GPO3,
        .gpioBase = CONFIG_IOL_ENL6_BASE_ADDR,
        .gpioPin = CONFIG_IOL_ENL6_PIN
    }
 },
 { // Port 6
    .rx = {
        .ctlRegOffset = PIN_PRG0_PRU0_GPO6,
        .gpioBase = CONFIG_IOL_RX7_BASE_ADDR,
        .gpioPin = CONFIG_IOL_RX7_PIN
    },
    .tx = {
        .gpio.ctlRegOffset = PIN_PRG0_PRU0_GPO16,
        .gpio.gpioBase = CONFIG_IOL_TX7_BASE_ADDR,
        .gpio.gpioPin = CONFIG_IOL_TX7_PIN,
        .pruPin = 16,
    },
    .txEn = {
        .gpio.ctlRegOffset = PIN_PRG0_PRU1_GPO15,
        .gpio.gpioBase = CONFIG_IOL_TXEN7_BASE_ADDR,
        .gpio.gpioPin = CONFIG_IOL_TXEN7_PIN,
        .gpioPhysAddr = CONFIG_IOL_TXEN7_BASE_ADDR + SOC_GPIO_GET_BASE_ADDR_OFS(CONFIG_IOL_TXEN7_PIN),
        .gpioPinMask = SOC_GPIO_GET_PIN_MASK(CONFIG_IOL_TXEN7_PIN),
    },
    .pwrEn = {
        .ctlRegOffset = PIN_PRG0_PRU1_GPO4,
        .gpioBase = CONFIG_IOL_ENL7_BASE_ADDR,
        .gpioPin = CONFIG_IOL_ENL7_PIN
    }
 },
 { // Port 7
    .rx = {
        .ctlRegOffset = PIN_PRG0_PRU0_GPO7,
        .gpioBase = CONFIG_IOL_RX8_BASE_ADDR,
        .gpioPin = CONFIG_IOL_RX8_PIN
    },
    .tx = {
        .gpio.ctlRegOffset = PIN_PRG0_PRU0_GPO17,
        .gpio.gpioBase = CONFIG_IOL_TX8_BASE_ADDR,
        .gpio.gpioPin = CONFIG_IOL_TX8_PIN,
        .pruPin = 17,
    },
    .txEn = {
        .gpio.ctlRegOffset = PIN_PRG0_PRU1_GPO16,
        .gpio.gpioBase = CONFIG_IOL_TXEN8_BASE_ADDR,
        .gpio.gpioPin = CONFIG_IOL_TXEN8_PIN,
        .gpioPhysAddr = CONFIG_IOL_TXEN8_BASE_ADDR + SOC_GPIO_GET_BASE_ADDR_OFS(CONFIG_IOL_TXEN8_PIN),
        .gpioPinMask = SOC_GPIO_GET_PIN_MASK(CONFIG_IOL_TXEN8_PIN),
    },
    .pwrEn = {
        .ctlRegOffset = PIN_PRG0_PRU1_GPO5,
        .gpioBase = CONFIG_IOL_ENL8_BASE_ADDR,
        .gpioPin = CONFIG_IOL_ENL8_PIN
    }
 }
};

const IOLM_SPhyGeneric IOLM_SOC_phyPortCfgPru_g[IOLM_PORT_COUNT] =
{
    {
        .u8Port = 0,
        .eType = IOLM_Phy_eType_Pru,
        IOLM_PHY_INTERFACE_SITARAPRU
    },
    {
        .u8Port = 1,
        .eType = IOLM_Phy_eType_Pru,
        IOLM_PHY_INTERFACE_SITARAPRU
    },
    {
        .u8Port = 2,
        .eType = IOLM_Phy_eType_Pru,
        IOLM_PHY_INTERFACE_SITARAPRU
    },
    {
        .u8Port = 3,
        .eType = IOLM_Phy_eType_Pru,
        IOLM_PHY_INTERFACE_SITARAPRU
    },
    {
        .u8Port = 4,
        .eType = IOLM_Phy_eType_Pru,
        IOLM_PHY_INTERFACE_SITARAPRU
    },
    {
        .u8Port = 5,
        .eType = IOLM_Phy_eType_Pru,
        IOLM_PHY_INTERFACE_SITARAPRU
    },
    {
        .u8Port = 6,
        .eType = IOLM_Phy_eType_Pru,
        IOLM_PHY_INTERFACE_SITARAPRU
    },
    {
        .u8Port = 7,
        .eType = IOLM_Phy_eType_Pru,
        IOLM_PHY_INTERFACE_SITARAPRU
    },
};

void Board_initPruss(uint32_t pruSelect)
{
    (void)pruSelect;
}

void IOLM_SOC_SetPinMux(int16_t regOffset_p, uint32_t mode_p, bool isRx_p)
{
    Pinmux_PerCfg_t pinMuxCfg[2] = {
        {PINMUX_END, PINMUX_END},
        {PINMUX_END, PINMUX_END}
    };

    pinMuxCfg[0].offset = regOffset_p;
    pinMuxCfg[0].settings = PIN_MODE(mode_p) | PIN_PULL_DISABLE;

    if (isRx_p)
    {
        pinMuxCfg[0].settings |= PIN_INPUT_ENABLE; // rx active
    }

    Pinmux_config(pinMuxCfg, PINMUX_DOMAIN_ID_MAIN);
}

void IOLM_SOC_GPIOWrite(const IOLM_PL_sGpioConfig_t* pGpio_p, bool value_p)
{
    if (value_p)
    {
        GPIO_pinWriteHigh(pGpio_p->gpioBase, pGpio_p->gpioPin);
    }
    else
    {
        GPIO_pinWriteLow(pGpio_p->gpioBase, pGpio_p->gpioPin);
    }
}

bool IOLM_SOC_GPIORead(const IOLM_PL_sGpioConfig_t* pGpio_p)
{
    bool value;

    value = GPIO_pinRead(pGpio_p->gpioBase, pGpio_p->gpioPin);

    return value;
}

/**
 * \brief  Initialize SoC related hardware
 *
 */
void IOLM_SOC_init()
{
    uint8_t portNum;

    OSAL_MEMORY_memset(iolEqMode_g, 0, sizeof(iolEqMode_g));

    PRU_IOLM_registerSetModeCallback(IOLM_SOC_setMode);
    PRU_IOLM_registerGetPortCfgCallback(IOLM_SOC_getPortCfg);
    PRU_IOLM_registerGetPruCfgCallback(IOLM_SOC_getPruCfg);
    PRU_IOLM_registerSetDoCallback(IOLM_SOC_setDO);
    PRU_IOLM_registerGetDiCallback(IOLM_SOC_getDI);
    PRU_IOLM_registerSetPowerCallback(IOLM_SOC_setPower);
    PRU_IOLM_registerSetIQCallback(IOLM_SOC_SetIQ);
    PRU_IOLM_registerGetIQCallback(IOLM_SOC_GetIQ);
    PRU_IOLM_registerSetIQModeCallback(IOLM_SOC_SetIQMode);

    Board_initPruss(iolPruSetup_g[0].pruIcssInst);

    for (portNum = 0; portNum < IOLM_SOC_PORT_COUNT; portNum++)
    {
        IOLM_SOC_SetPinMux(iolPinSetup_g[portNum].rx.ctlRegOffset, IOL_MUX_PRU_RX, true);
        IOLM_SOC_SetPinMux(iolPinSetup_g[portNum].tx.gpio.ctlRegOffset, IOL_MUX_PRU_TX, false);
        IOLM_SOC_SetPinMux(iolPinSetup_g[portNum].txEn.gpio.ctlRegOffset, IOL_MUX_GPIO_TX, false);
        IOLM_SOC_SetPinMux(iolPinSetup_g[portNum].pwrEn.ctlRegOffset, IOL_MUX_GPIO_TX, false);

        // setup the GPIO direction
        GPIO_setDirMode(iolPinSetup_g[portNum].rx.gpioBase, iolPinSetup_g[portNum].rx.gpioPin, GPIO_DIRECTION_INPUT);
        GPIO_setDirMode(iolPinSetup_g[portNum].txEn.gpio.gpioBase, iolPinSetup_g[portNum].txEn.gpio.gpioPin, GPIO_DIRECTION_OUTPUT);
        GPIO_setDirMode(iolPinSetup_g[portNum].pwrEn.gpioBase, iolPinSetup_g[portNum].pwrEn.gpioPin, GPIO_DIRECTION_OUTPUT);
        GPIO_setDirMode(iolPinSetup_g[portNum].tx.gpio.gpioBase, iolPinSetup_g[portNum].tx.gpio.gpioPin, GPIO_DIRECTION_OUTPUT);

        IOLM_SOC_GPIOWrite(&iolPinSetup_g[portNum].tx.gpio, 0);
        IOLM_SOC_GPIOWrite(&iolPinSetup_g[portNum].txEn.gpio, 0);
        IOLM_SOC_GPIOWrite(&iolPinSetup_g[portNum].pwrEn, 0);
        IOLM_SOC_GPIOWrite(&iolPinSetup_g[portNum].pwrEn, 1);
    }
}

void IOLM_SOC_setMode(uint8_t instance_p, uint8_t portNum_p, IOLM_PL_ePortMode_t mode_p)
{
    IOLM_SOC_ECqMode_t cqMode = IOLM_SOC_eCqMode_PUSHPULL;
    // tx enable

    if(IOLM_SOC_checkInstPortValid(instance_p, portNum_p) != OSAL_eERR_NOERROR)
    {
        goto laExit;
    }

    IOLM_SOC_GPIOWrite(&iolPinSetup_g[portNum_p].txEn.gpio, 0);
    IOLM_SOC_SetPinMux(iolPinSetup_g[portNum_p].txEn.gpio.ctlRegOffset, IOL_MUX_GPIO_TX, false);

    switch (mode_p)
    {
    case IOLM_PL_eModeSioInactive:
    case IOLM_PL_eModeSioDI:
        IOLM_SOC_SetPinMux(iolPinSetup_g[portNum_p].rx.ctlRegOffset, IOL_MUX_GPIO_RX, true);
        IOLM_SOC_SetPinMux(iolPinSetup_g[portNum_p].tx.gpio.ctlRegOffset, IOL_MUX_GPIO_TX, false);
        break;
    case IOLM_PL_eModeSioDO:
        IOLM_SOC_SetCqMode(instance_p * 8 + portNum_p, cqMode);
        IOLM_SOC_SetPinMux(iolPinSetup_g[portNum_p].rx.ctlRegOffset, IOL_MUX_GPIO_RX, true);
        IOLM_SOC_SetPinMux(iolPinSetup_g[portNum_p].tx.gpio.ctlRegOffset, IOL_MUX_GPIO_TX, false);
        IOLM_SOC_GPIOWrite(&iolPinSetup_g[portNum_p].txEn.gpio, 1);
        break;
    case IOLM_PL_eModeSdci:
        IOLM_SOC_GPIOWrite(&iolPinSetup_g[portNum_p].txEn.gpio, 0);
        IOLM_SOC_SetPinMux(iolPinSetup_g[portNum_p].rx.ctlRegOffset, IOL_MUX_PRU_RX, true);
        IOLM_SOC_SetPinMux(iolPinSetup_g[portNum_p].tx.gpio.ctlRegOffset, IOL_MUX_PRU_TX, false);
        break;
    default:
        break;
    }
laExit:
    return;
}

const IOLM_PL_sPortConfig_t* IOLM_SOC_getPortCfg(uint8_t instance_p, uint8_t portNum_p)
{
    if (portNum_p < IOLM_SOC_PORT_COUNT)
    {
        return &iolPinSetup_g[portNum_p];
    }
    else
    {
        return NULL;
    }
}

const IOLM_PL_sPruIccsCfg_t* IOLM_SOC_getPruCfg(uint8_t instance_p)
{
    if (instance_p < IOLM_SOC_PRU_COUNT)
    {
        return &iolPruSetup_g[instance_p];
    }
    else
    {
        return NULL;
    }
}

/**
 * \brief  Set the output value of specific port in SIO-mode
 *
 * \param  portNum_p            Output port which should be changed.
 *
 * \param  boPortTargetState_p  Output state (0=>LOW, 1=>HIGH).
 *
 */
void IOLM_SOC_setDO(uint8_t instance_p, uint8_t portNum_p, bool boPortTargetState_p)
{
    if(IOLM_SOC_checkInstPortValid(instance_p, portNum_p) != OSAL_eERR_NOERROR)
    {
        goto laExit;
    }

    IOLM_SOC_ECqMode_t cqMode = IOLM_SOC_GetCqMode(instance_p * 8 + portNum_p);
    switch (cqMode) {
        case IOLM_SOC_eCqMode_NPN:
            if (boPortTargetState_p == true)
            {
                IOLM_SOC_GPIOWrite(&iolPinSetup_g[portNum_p].txEn.gpio, 1);
            }
            else
            {
                IOLM_SOC_GPIOWrite(&iolPinSetup_g[portNum_p].txEn.gpio, 0);
            }
        IOLM_SOC_GPIOWrite(&iolPinSetup_g[portNum_p].tx.gpio, 1);
        break;
        case IOLM_SOC_eCqMode_PNP:
            if (boPortTargetState_p == true)
            {
                IOLM_SOC_GPIOWrite(&iolPinSetup_g[portNum_p].txEn.gpio, 1);
            }
            else
            {
                IOLM_SOC_GPIOWrite(&iolPinSetup_g[portNum_p].txEn.gpio, 0);
            }
        IOLM_SOC_GPIOWrite(&iolPinSetup_g[portNum_p].tx.gpio, 0);
        break;
        case IOLM_SOC_eCqMode_PUSHPULL:
            if (boPortTargetState_p == true)
            {
                IOLM_SOC_GPIOWrite(&iolPinSetup_g[portNum_p].tx.gpio, 0);
            }
            else
            {
                IOLM_SOC_GPIOWrite(&iolPinSetup_g[portNum_p].tx.gpio, 1);
            }
        IOLM_SOC_GPIOWrite(&iolPinSetup_g[portNum_p].txEn.gpio, 1);
        break;
    default:
        OSAL_printf("SIO Mode defined but no CQ Mode defined");
        break;
    }
laExit:
    return;
}

/**
 * \brief  Read the input value of specific port in SIO-mode
 *
 * \param  instance_p    pru instance
 * \param  portNum_p     Input port which should be read.
 *
 * \return 0=>LOW, 1=>HIGH.
 *
 */
bool IOLM_SOC_getDI(uint8_t instance_p, uint8_t portNum_p)
{
    bool boDInputValue = true;

    if(IOLM_SOC_checkInstPortValid(instance_p, portNum_p) == OSAL_eERR_NOERROR)
    {
        IOLM_SOC_GPIOWrite(&iolPinSetup_g[portNum_p].txEn.gpio, 0);
        boDInputValue = IOLM_SOC_GPIORead(&iolPinSetup_g[portNum_p].rx);
    }

    return !(boDInputValue);
}

/**
 * \brief  Turn the power of a specific port on or off
 *
 * \param  instance_p    pru instance
 * \param  portNum_p    IO-Link port which should be turned on/off.
 *
 * \param  powerState_p      target state
 *
 */
void IOLM_SOC_setPower(uint8_t instance_p, uint8_t portNum_p, bool powerState_p)
{
    uint32_t powerState = (powerState_p ? 1 : 0);

    if(IOLM_SOC_checkInstPortValid(instance_p, portNum_p) == OSAL_eERR_NOERROR)
    {
        IOLM_SOC_GPIOWrite(&iolPinSetup_g[portNum_p].pwrEn, powerState);
    }
}


/**
 * \brief  Set the output value of specific ports IQ signal
 *
 * \param  instance_p    pru instance
 * \param  portNum_p     port number of the pru
 *
 * \param  boOutValue_p  Output state (0=>LOW, 1=>HIGH).
 *
 */
void IOLM_SOC_SetIQ(uint8_t instance_p, uint8_t portNum_p, bool boOutValue_p)
{
    /* IQ Output is not supported on AM437x, AM64x and AM65x hardware. */
    (void)instance_p;
    (void)portNum_p;
    (void)boOutValue_p;
}

/**
 * \brief  Read the input value of specific ports IQ signal
 *
 * \param  instance_p    pru instance
 * \param  portNum_p     port number of the pru
 *
 * \return 0=>LOW, 1=>HIGH.
 *
 */
bool IOLM_SOC_GetIQ(uint8_t instance_p, uint8_t portNum_p)
{
    bool iqState = false;

    if(IOLM_SOC_checkInstPortValid(instance_p, portNum_p) == OSAL_eERR_NOERROR)
    {
        iqState = IOLM_SPI_getIq(portNum_p);
    }
    return iqState;
}


/**
 * \brief  Set the mode of specific ports IQ behaviour
 *
 * \param  instance_p    pru instance
 * \param  portNum_p     port number of the pru
 *
 * \param  eIQMode_p     Desired target mode.
 *
 */
void IOLM_SOC_SetIQMode(uint8_t instance_p, uint8_t portNum_p, IOL_EIQMode eIQMode_p)
{
    /* No hardware configuration steps necessary for mode switch */
    (void)instance_p;
    (void)portNum_p;
    (void)eIQMode_p;
}

/**
 * \brief  Set the mode of specific ports CQ behaviour
 *
 * \param  port_p       port number
 * \param  eCqMode_p    desired target mode
 *
 * \return  void
 *
 */
void IOLM_SOC_SetCqMode(INT8U port_p, IOLM_SOC_ECqMode_t eCqMode_p)
{
    iolEqMode_g[port_p] = eCqMode_p;
}

/**
 * \brief  Set the mode of specific ports CQ behaviour
 *
 * \param  port_p       port number
 * 
 * \return   target mode.
 *
 */
IOLM_SOC_ECqMode_t IOLM_SOC_GetCqMode(INT8U port_p)
{
    IOLM_SOC_ECqMode_t eEqMode = iolEqMode_g[port_p];

    return eEqMode;
}

/**
 * \brief  Set the mode of specific ports IQ behaviour
 *
 * \param  instance_p    pru instance
 * \param  portNum_p     port number of the pru
 *
 * \return  eIQMode_p     Desired target mode.
 *
 */
OSAL_EError_t IOLM_SOC_checkInstPortValid(uint8_t instance_p, uint8_t portNum_p)
{
    if (instance_p >= IOL_PRU_INSTANCE_MAX)
    {
        return OSAL_eERR_EINVAL;
    }

    if (portNum_p >= IOLM_PORT_COUNT)
    {
        return OSAL_eERR_EINVAL;
    }

    if (instance_p != iolPruSetup_g->pruIcssInst)
    {
        return OSAL_eERR_EINVAL;
    }

    return OSAL_eERR_NOERROR;
}
