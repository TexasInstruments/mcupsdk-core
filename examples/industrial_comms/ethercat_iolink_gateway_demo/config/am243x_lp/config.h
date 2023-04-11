/*!
 *  \file config.h
 *
 *  \brief
 *  Configuration for the board AM243x LP
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2022-08-08
 *
 *  \copyright
 *  Copyright (c) 2022, KUNBUS GmbH<br /><br />
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

#if !(defined PROTECT_CONFIG_H)
#define PROTECT_CONFIG_H        1

// sample data for the optional API functions
#define IOLM_VENDORID              0xABCDU
#define IOLM_MASTERID              0x12345678
#define IOLM_USEDPORTS             8U
#if (defined FBTL_EC_SLV_SUPPORT)
// 2 CPU
#define IOLM_PRUINSTANCE           0
#define ECAT_PRUINSTANCE           0xFFU
#else
// 1 CPU
#define IOLM_PRUINSTANCE           0
#define ECAT_PRUINSTANCE           1U
#endif
// \todo add constraint IOLM_PRUINSTANCE must be 0

#define ECAT_REVISION              0x00010000
#define ECAT_SERIALNR              0x76543210
#define ECAT_HWVERSION             "1.0 E3"
#define ECAT_SWVERSION             VERSION
#if ((defined TI_EC_VENDOR) && (TI_EC_VENDOR==1))
#define ECAT_VENDORID              0x0000059DU      // primary vendor id(TI), secondary vendor ID (1389 + 0xE0000000)
#define ECAT_PNAME                 "TI EC-IOL-Gateway for AM243X.R5F"
#define ECAT_PCODE                 0x54491001
#else
#define ECAT_VENDORID              0x00000569U      // primary vendor id(KB) (0x0569 == 1385), secondary vendor ID (1385 + 0xE0000000)
#define ECAT_PNAME                 "KUNBUS EC-IOL-Gateway for AM243X.R5F"
#define ECAT_PCODE                 0x00018801U
#endif /*  (defined TI_EC_VENDOR) ... */

#endif /* PROTECT_CONFIG_H */