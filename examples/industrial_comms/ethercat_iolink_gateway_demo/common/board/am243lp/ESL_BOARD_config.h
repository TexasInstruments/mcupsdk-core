/*!
 *  \file ESL_BOARD_config.h
 *
 *  \brief
 *  Board support for AM243 Launchpad.
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2022-02-11
 *
 *  \copyright
 *  Copyright (c) 2021, KUNBUS GmbH<br /><br />
 *  All rights reserved.<br />
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:<br />
 *  <ol>
 *  <li>Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.</li>
 *  <li>Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.</li>
 *  <li>Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.</li>
 *  </ol>
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#if !(defined __ESL_BOARD_CONFIG_H__)
#define __ESL_BOARD_CONFIG_H__		1

#include <ESL_OS_os.h>
#include <ESL_BOARD_OS_config.h>

#define ECAT_PHYADDR_IN                     (3u)
#define ECAT_PHYADDR_OUT                    (15u)
#define ECAT_PHYPOLINVERT_IN                (true)
#define ECAT_PHYPOLINVERT_OUT               (true)
#define ECAT_PHYUSERXLINK_IN                (true)
#define ECAT_PHYUSERXLINK_OUT               (true)

#define TIESC_I2CDEVICE                     "/dev/i2c-0"

#if ((defined TI_EC_VENDOR) && (TI_EC_VENDOR==1))
 #define ECAT_PRODUCTCODE_CTT               0x54490023      // set product code
 #define ECAT_PRODUCTCODE_CIA402            0x54490024      // set product code
 #define ECAT_PRODUCTCODE_SIMPLE            0x54490025      // set product code
 #define ECAT_PRODUCTNAME_CTT               "TI EtherCAT Toolkit Conformance for AM243X.R5F"
 #define ECAT_PRODUCTNAME_CIA402            "TI EtherCAT Toolkit CiA402 for AM243X.R5F"
 #define ECAT_PRODUCTNAME_SIMPLE            "TI EtherCAT Toolkit for AM243X.R5F"
 #define ECAT_REVISION_NO                   0x00010000
#else
 #define ECAT_PRODUCTCODE_CTT               0x000187FD      // set product code
 #define ECAT_PRODUCTCODE_CIA402            0x000187FD      // set product code
 #define ECAT_PRODUCTCODE_SIMPLE            0x000187FD      // set product code
 #define ECAT_PRODUCTNAME_CTT               "KUNBUS EtherCAT Toolkit Conformance for AM243X.R5F"
 #define ECAT_PRODUCTNAME_CIA402            "KUNBUS EtherCAT Toolkit CiA402 for AM243X.R5F"
 #define ECAT_PRODUCTNAME_SIMPLE            "KUNBUS EtherCAT Toolkit for AM243X.R5F"
 #define ECAT_REVISION_NO                   0x00010000
#endif

#if (defined __cplusplus)
extern "C" {
#endif

extern void ESL_BOARD_OS_initPruss      (uint32_t   pruSelect_p
                                        ,int32_t*   pBaseIrqOffset_p);
extern void ESL_BOARD_OS_initStatusLED  (void*      pGpioHandle_p
                                        ,uint32_t   selectedPru_p);
extern void ESL_BOARD_OS_statusLED      (void*      pGpioHandle_p
                                        ,uint32_t   selectedPru_p
                                        ,bool       runLed_p
                                        ,bool       errLed_p);

extern void ESL_BOARD_OS_configureResets(void*      pGpioHandle_p
                                        ,uint32_t   selectedPru_p);
extern void ESL_BOARD_OS_phyReset       (void*      pGpioHandle_p
                                        ,uint32_t   selectedPru_p
                                        ,uint8_t    phyIdx_p
                                        ,bool       reset_p);
extern void ESL_BOARD_OS_registerPhys   (uint32_t   selectedPru_p);

#if (defined __cplusplus)
}
#endif

#endif /* __ESL_BOARD_CONFIG_H__ */
