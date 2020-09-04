/*!
 *  \file hwal_ipprotection_types.h
 *
 *  \brief
 *  HWAL IP protection datatypes.
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2021-11-01
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

#include <stdint.h>

#if !(defined PROTECT_HWAL_IPPROTECTION_TYPES_H)
#define PROTECT_HWAL_IPPROTECTION_TYPES_H		1

#define SHA512_SUM_SIZE         (512U/8U)
#define SHA512_SUM_LONGSIZE     (SHA512_SUM_SIZE/sizeof(uint64_t))

#define RSA_KEY_SIZE            (1024U/8U)
#define RSA_KEY_LONGSIZE        (RSA_KEY_SIZE/sizeof(uint64_t))

/*! <!-- Description: -->
 *
 *  \brief
 *  IP protection License types
 * */
typedef enum HWAL_IPP_eLicenseProfile
{
    HWAL_eLP_noPRUTypeC         = 0x0010,       /*!< Chip Type C */
    HWAL_eLP_noPRUTypeD         = 0x0020,       /*!< Chip Type D */
    HWAL_eLP_restrictedTypeE    = 0x4000,       /*!< Chip Type E */
    HWAL_eLP_unrestrictedTypeF  = 0x8000,       /*!< Chip Type F */
    HWAL_eLP_MAX                = 0xffffffff,   /*!< 32Bit ENUM */
} HWAL_IPP_eLicenseProfile_t;

/*! <!-- Description: -->
 *
 *  \brief
 *  IP protection data
 * */
typedef struct HWAL_IPP_sProtectionDesc
{
    uint32_t                        licenseHWaddress;       /*!< address where to read license information from */
    uint32_t                        licenseMask;            /*!< license mask to apply */
    char                            mcuName[0x10];          /*!< MCU name */
    HWAL_IPP_eLicenseProfile_t      licenseProfileType;     /*!< license type to be applied */
    uint64_t                        sha512LicenseValue[SHA512_SUM_LONGSIZE];
                                                            /*!< license register content hash */
} HWAL_IPP_sProtectionDesc_t;

/*! <!-- Description: -->
 *
 *  \brief
 *  IP protection encapsulated data
 * */
typedef struct HWAL_IPP_sProtectionRawDesc {
    uint32_t                        licenseRegister;        /*!< address where to read license information from */
    uint32_t                        licenseRegisterMask;    /*!< license register value mask */
    uint32_t                        licenseRegisterValue;   /*!< license register value */
    HWAL_IPP_eLicenseProfile_t      licenseProfile;         /*!< license to be applied */
    char                            gitHash[0x30];          /*!< git hash sum */
} HWAL_IPP_sProtectionRawDesc_t;

#if (defined __cplusplus)
extern "C" {
#endif

#if (defined __cplusplus)
}
#endif

#endif /* PROTECT_HWAL_IPPROTECTION_TYPES_H */
