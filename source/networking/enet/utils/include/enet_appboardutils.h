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
 * \file enet_appboardutils.h
 *
 * \brief Board utilities header file.
 */

#ifndef ENET_APPBOARDUTILS_H_
#define ENET_APPBOARDUTILS_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*!
 * \brief EthFw board init function which does limited configuration.
 */
void EnetBoard_initEthFw(void);

void EnetBoard_deinit(void);

uint32_t EnetBoard_getPhyAddr(Enet_Type enetType,
                                      Enet_MacPort portNum);

void EnetBoard_setPhyConfig(Enet_Type enetType,
                                    Enet_MacPort portNum,
                                    CpswMacPort_Cfg *macCfg,
                                    EnetMacPort_Interface *interface,
                                    EnetPhy_Cfg *phyCfg);

void EnetBoard_setPhyConfigSgmii(Enet_MacPort portNum,
                                         CpswMacPort_Cfg *macCfg,
                                         EnetMacPort_Interface *interface,
                                         EnetPhy_Cfg *phyCfg);

void EnetBoard_setPhyConfigQsgmii(Enet_Type enetType,
                                          Enet_MacPort portNum,
                                          CpswMacPort_Cfg *macCfg,
                                          EnetMacPort_Interface *interface,
                                          EnetPhy_Cfg *phyCfg);

void EnetBoard_setPhyConfigRmii(Enet_MacPort portNum,
                                        CpswMacPort_Cfg *macCfg,
                                        EnetMacPort_Interface *interface,
                                        EnetPhy_Cfg *phyCfg);

void EnetBoard_setPhyConfigRgmii(Enet_Type enetType,
                                         Enet_MacPort portNum,
                                         CpswMacPort_Cfg *macCfg,
                                         EnetMacPort_Interface *interface,
                                         EnetPhy_Cfg *phyCfg);

/**
 * \brief Set link, MAC and PHY configuration for loopback.
 *
 * Sets configuration of port link and PHY for PHY loopback.  The port link
 * is set to fixed speed set according to the max speed of the given MAC
 * interface: 100Mbps for MII layer type, 1Gpbs for GMII layer type.
 */
void EnetBoard_setLpbkCfg(bool phyLpbk,
                                  Enet_MacPort portNum,
                                  const EnetMacPort_Interface *interface,
                                  CpswMacPort_Cfg *macCfg,
                                  EnetPhy_Cfg *phyCfg,
                                  EnetMacPort_LinkCfg *linkCfg);

void EnetBoard_getMacAddrList(uint8_t macAddr[][ENET_MAC_ADDR_LEN],
                                      uint32_t maxMacEntries,
                                      uint32_t *pAvailMacEntries);

/*!
 * \brief Enables pinmux for the board interfaces required for EthFw.
 */
int32_t EnetBoard_configEthFwPinmux(void);

/**
 *  \brief Utils API to set mode_sel field of ENET_CTRL register
 */
void EnetBoard_setEnetControl(Enet_Type enetType,
                                      uint32_t instId,
                                      Enet_MacPort portNum,
                                      uint32_t modeSel);

#ifdef __cplusplus
}
#endif

#endif /* ENET_APPBOARDUTILS_H_ */
