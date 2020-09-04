/*!
* \file FBTL_PHYS_ram.h
*
* \brief
* FBTL physical layer for RAM like underlay.
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

#if !(defined __FBTL_PHYS_RAM_H__)
#define __FBTL_PHYS_RAM_H__		1

#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <physical/FBTL_phys.h>
#include <system/FBTL_sys.h>

#define FBTL_RAM_VERSION        ((uint32_t)(0x02000000u)) /* 1= DPR, 2= FBTL v1 */
#define FBTL_READY_FLAG         ((uint32_t)(0xAA5555AAu))

#define FBTL_IRQMANINGUARD      ((uint32_t)(0x55aacc33u))
#define FBTL_IRQMANOUTGUARD     ((uint32_t)(0x33ccaa55u))

/*!
    \brief RAM like Header structure

    \ingroup FBTL_PHYS
*/
typedef struct FBTL_PHYS_RAM_SHeader
{
    uint32_t    version;                ///!< Version of RAM structure
    uint32_t    appReady;               ///!< Application ready signal
    uint32_t    size;                   ///!< Size of RAM / can be changed by app cpu
    uint32_t    chanListOffset;         ///!< Offset to a list of \ref DPR_SChannelDef
} FBTL_STRUCT_PACKED FBTL_PHYS_RAM_SHeader_t;


/*!
    \brief Control/Status buffer control structure

    \ingroup FBTL_PHYS
*/
typedef struct FBTL_PHYS_RAM_SIrqManager
{
    volatile    uint32_t                    inGuard;            ///!< IRQ guard

    volatile    uint32_t                    latest;             ///!< Simpson latest
    volatile    uint32_t                    reading;            ///!< Simpson reading
    volatile    uint32_t                    aSlot[2];           ///!< Simpson buffer matrix

    volatile    FBTL_PHYS_SCtrlStatus_t     aCtrlStatus[2][2];  ///!< Control status buffers

    volatile    uint32_t                    outGuard;           ///!< IRQ guard
} FBTL_STRUCT_PACKED FBTL_PHYS_RAM_SIrqManager_t;

#if (defined __cplusplus)
extern "C" {
#endif

#if (defined __cplusplus)
}
#endif

#endif /* __FBTL_PHYS_RAM_H__ */
