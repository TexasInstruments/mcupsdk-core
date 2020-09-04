/*!
 *  \file ESL_eeprom.c
 *
 *  \brief
 *  EEPROM load and store function.
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2021-05-18
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

#define EEPROM_FILEPATH "eeprom.bin"

#include <ecSlvApiDef.h>
#include <ecSlvApi.h>
#include <ESL_eeprom.h>
#include <osal.h>

#include <drivers/i2c.h>
#include <board/eeprom.h>

#include "ti_board_open_close.h"

#define APP_OSPI_FLASH_OFFSET_BASE  (0x200000U)
typedef struct SPI_SEepromHeader
{
    uint32_t    startMagic;
    uint32_t    dataSize;
} SPI_SEepromHeader_t;

#if !(defined FBTLPROVIDER) || (FBTLPROVIDER==0)
#if (defined CONFIG_FLASH0)
static Flash_Attrs*     flashAttribute_s    = NULL;
#endif
#endif

/*! <!-- Description: -->
 *
 *  \brief
 *  Initialize EEProm instance
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p  application context
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <ESL_eeprom.h>
 *
 *  // required variables
 *  void* pHandle;
 *
 *  // the Call
 *  EC_SLV_APP_initFlash(pHandle);
 *  \endcode
 *
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
void EC_SLV_APP_EEP_initFlash(void* pContext_p)
{
    OSALUNREF_PARM(pContext_p);

#if !(defined FBTLPROVIDER) || (FBTLPROVIDER==0)
#if (defined CONFIG_FLASH0)
    flashAttribute_s = Flash_getAttrs(CONFIG_FLASH0);
#endif
#endif
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Write EEPROM to memory.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p      Application context
 *  \param[in]  pEeprom_p		Pointer to eeprom memory address.
 *  \param[in]  length_p    Eeprom length.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <ecSlvApi.h>
 *
 *  // required variables
 *  void*       handle;
 *  uint16_t    pEeprom[0x400];
 *  uint32_t    length = 0x400;
 *
 *  // the Call
 *  EC_SLV_APP_writeEeprom(handle, pEeprom, length);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVAPI
 *
 * */
void EC_SLV_APP_EEP_writeEeprom(void *pContext_p, void* pEeprom_p, uint32_t length_p)
{
    uint32_t                offset      = APP_OSPI_FLASH_OFFSET_BASE;

    uint32_t                pageCount   = 0;
    uint32_t                idx         = 0;
    uint32_t                blk,page;
    uint8_t*                offloader   = NULL;
    uint32_t                flashMagic  = (uint32_t)pContext_p;
    SPI_SEepromHeader_t*    pageHead    = NULL;

#if !(defined FBTLPROVIDER) || (FBTLPROVIDER==0)
#if (defined CONFIG_FLASH0)
    if (NULL == flashAttribute_s)
    {
        return;
    }

    pageCount = (length_p + flashAttribute_s->blockSize - 1) / flashAttribute_s->blockSize;
#endif
#endif

    pageHead = (SPI_SEepromHeader_t*)OSAL_MEMORY_calloc(sizeof(SPI_SEepromHeader_t)+length_p, sizeof(uint8_t));
    OSAL_MEMORY_memcpy((void*)&pageHead[1], pEeprom_p, length_p);
    pageHead[0].startMagic  = flashMagic;
    pageHead[0].dataSize    = length_p;

    offloader = (uint8_t*)pageHead;

#if !(defined FBTLPROVIDER) || (FBTLPROVIDER==0)
#if (defined CONFIG_FLASH0)
    for (idx = 0; idx < pageCount; ++idx)
    {
        Flash_offsetToBlkPage   (gFlashHandle[CONFIG_FLASH0],
                                 offset+(idx*flashAttribute_s->blockSize),
                                 &blk, &page);

        Flash_eraseBlk          (gFlashHandle[CONFIG_FLASH0], blk);
    }

    Flash_write(gFlashHandle[CONFIG_FLASH0], offset, (uint8_t*)pageHead, sizeof(SPI_SEepromHeader_t)+length_p);
#endif
#endif
    OSAL_MEMORY_free(pageHead);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Load EEPROM from memory.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p      application context
 *  \param[out] pEeprom_p       Pointer to eeprom memory address.
 *  \param[out] pLength_p       Eeprom length.
 *
 *  \return     bool            eeprom loaded correctly or not.
 *
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <ecSlvApi.h>
 *
 *  // required variables
 *  uint8_t* pEeprom = NULL;;
 *  uint32_t length;
 *
 *  // the Call
 *  EC_SLV_APP_loadEeprom(pEeprom, &length);
 *  pEeprom = (uint8_t*)OSAL_MEMORY_calloc(length, sizeof(uint8_t));
 *  EC_SLV_APP_loadEeprom(pEeprom, &length);
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup EC_SLV_APP
 *
 * */
bool EC_SLV_APP_EEP_loadEeprom(void* pContext_p, void* pEeprom_p, uint32_t* pLength_p)
{
    bool                    bRet        = false;
    uint32_t                offset      = APP_OSPI_FLASH_OFFSET_BASE;
    SPI_SEepromHeader_t     pageProto   = {0};
    int32_t                 status      = SystemP_SUCCESS;
#if !(defined FBTLPROVIDER) || (FBTLPROVIDER==0)
#if (defined CONFIG_FLASH0)
    uint32_t                flashMagic  = (uint32_t)pContext_p;

    status = Flash_read(gFlashHandle[CONFIG_FLASH0], offset, (uint8_t*)&pageProto, sizeof(SPI_SEepromHeader_t));
    if (SystemP_SUCCESS !=status)
    {
        goto Exit;
    }

    if (pageProto.startMagic != flashMagic)
    {
        goto Exit;
    }

    status = Flash_read(gFlashHandle[CONFIG_FLASH0], offset+sizeof(SPI_SEepromHeader_t), pEeprom_p, pageProto.dataSize);
    if (SystemP_SUCCESS !=status)
    {
        goto Exit;
    }

    if (pLength_p)
    {
        pLength_p[0] = pageProto.dataSize;
    }
#else
    goto Exit;
#endif
#else
    goto Exit;
#endif

    bRet = true;
Exit:
    return bRet;
}

//*************************************************************************************************
