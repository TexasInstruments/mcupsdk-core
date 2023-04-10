/*!
* \file IOLM_Port_spi.h
*
* \brief
* Interface for SPI Communication on IOLink Board
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

#ifndef IO_LINK_IOLINK_SPI_H_
#define IO_LINK_IOLINK_SPI_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdbool.h>
#include "osal.h"
#include "IOL_Port_Types.h"
#include "IOLM_Port_LEDTask.h"
#include <drivers/mcspi.h>
#include <kernel/dpl/CacheP.h>
#include "ti_drivers_config.h"

/* ========================================================================== */
/*                                Defines                                     */
/* ========================================================================== */

#define IOLM_SPI_LED_WIDTH_BIT_MASK (16U)
#define IOLM_SPI_LED_NUM_BYTES      (IOLM_SPI_LED_WIDTH_BIT_MASK / 8)

#define IOLM_SPI_LED_INSTANCE       CONFIG_MCSPI_IOL
#define IOLM_SPI_LED_CHANNEL        (1U)
#define IOLM_SPI_LED_DATA_SIZE      (16U)

#define IOLM_SPI_IQ_INSTANCE        CONFIG_MCSPI_IOL
#define IOLM_SPI_IQ_CHANNEL         (0U)
#define IOLM_SPI_IQ_DATA_SIZE       (8U)

/* ========================================================================== */
/*                             typedefs                                       */
/* ========================================================================== */

typedef struct IOLM_SPI_mapping
{
    uint32_t    spiInstance;
    uint32_t    spiChannel;
    uint32_t    spiDataSize;
} IOLM_SPI_mapping_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * \brief  Initialize the first SPI instance
 *
 *
 * \return void
 *
 */
void IOLM_SPI_init(void);

/**
 * \brief  Close the first SPI instance
 *
 *
 * \return void
 *
 */
void IOLM_SPI_close(void);

/**
 * \brief  Transfer and Reveive data to a specific MCSPI instance/channel
 *
 * \param[in]   mcspiInstance_p     MCSPI instance as defined in SysConfig
 * \param[in]   mcspiChannel_p      MCSPI channel as defined in SysConfig
 * \param[in]   mcspiDataSize_p     bit width of data to send (4 to 32 bits)
 * \param[in]   pTxData_p           pointer to transmit buffer
 * \param[in]   pRxData_p           pointer to receive buffer
 * \param[in]   lengthInBytes_p     length of transaction in bits
 *
 * \return      error               as int32_t
 * \retval      #SystemP_SUCCESS    on Success
 * \retval      #SystemP_FAILURE    on Failure
 *
 */
int32_t IOLM_SPI_mcspiTransfer(uint32_t mcspiInstance_p, uint32_t mcspiChannel_p, uint32_t mcspiDataSize_p,
                               uint8_t *pTxData_p, uint8_t *pRxData_p, uint32_t lengthInBytes_p);

/**
 * \brief  Transfer and Receive data to a specific MCSPI instance/channel
 *
 * WARNING: DEPRECATED!
 * This function is only used in Linux ports.
 * If this is touched for any reason, please use IOLM_SPI_mcspiTransfer instead!
 *
 * \param[in]   spiInstance_p       MCSPI instance as defined in SysConfig
 * \param[in]   mcspiChannel_p      MCSPI channel as defined in SysConfig
 * \param[in]   tx_p                32 bit data to transmit
 *
 * \return      rxBuf               as uint32_t
 *
 */
uint32_t IOLM_SPI_transfer(uint32_t spiInstance_p, uint32_t mcspiChannel_p, uint32_t tx_p);

/**
 * \brief  Set the Status-LEDs on the IO-Link adapter board
 *
 * \param[in]   bitMask_p           bitmask of the target LED states, red/green alternating, 16bits, staring at port 0
 *
 * \return      error               as int32_t
 * \retval      #SystemP_SUCCESS    on Success
 * \retval      #SystemP_FAILURE    on Failure
 *
 */
int32_t IOLM_SPI_setIolLeds(uint16_t bitMask_p);

/**
 * \brief  Get state of IQ pin on specified IO-Link port through SPI
 *
 * \param[in]   bitMask_p           bitmask of the target LED states, red/green alternating, 16bits, staring at port 0
 *
 * \return      error               as int32_t
 * \retval      #SystemP_SUCCESS    on Success
 * \retval      #SystemP_FAILURE    on Failure
 *
 */
bool IOLM_SPI_getIq(uint8_t portNum_p);

/**
 * \brief  Set state of LED on base board
 *
 * \param[in]   ledNumber_p         number of LED
 * \param[in]   ledState_p          target state
 *
 * \return      error               as int32_t
 * \retval      #SystemP_SUCCESS    on Success
 * \retval      #SystemP_FAILURE    on Failure
 *
 */
int32_t IOLM_SPI_baseBoardLED(uint32_t ledNumber_p, bool ledState_p);

#endif /* IO_LINK_IOLINK_SPI_H_ */
