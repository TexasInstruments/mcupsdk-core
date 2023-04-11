/*!
 *  \file SMIdirect_UART.h
 *
 *  \brief
 *  interface to a IOLink SMI GUI connected with UART
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2022-08-05
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

#if !(defined PROTECT_SMIDIRECT_UART_H)
#define PROTECT_SMIDIRECT_UART_H     1

#include <stdint.h>

#include <ti_drivers_open_close.h>
#include <IOL_Serial.h>
#include <gw_api_interface.h>  /* interface to Gateway */

#define SMIDIRECT_UART_NUM_TX_BUFFER          36
#define SMIDIRECT_UART_SIZE_TX_BUFFER         64
#define SMIDIRECT_UART_SIZE_RX_BUFFER         1024

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  typedef struct of expected port configuration
 *
 * */
typedef struct GWL_SPortConfig
{
    /**! \brief handle for IOL_Serial utility */
    IOL_Serial_SHandle iolSerial;
    /**! \brief driver handle for used UART */
    UART_Handle uartHandle;
    /**! \brief signal whether a UART transmit needs to be processed */
    volatile bool isTxPending;
    /**! \brief transmit transaction, used for UART driver */
    UART_Transaction txTransaction;
    uint8_t aSerialDataTx[SMIDIRECT_UART_NUM_TX_BUFFER * SMIDIRECT_UART_SIZE_TX_BUFFER];
    uint32_t aSerialLengthTx[SMIDIRECT_UART_NUM_TX_BUFFER];
    /**! \brief signal whether a UART receive needs to be processed */
    volatile bool isRxComplete;
    /**! \brief state of receive:  true=header receiving, false=data receiving */
    bool headerActive;
    /**! \brief current receive count */
    uint32_t currentRxCount;
    /**! \brief expected receive count */
    uint32_t expectRxCount;
    /**! \brief used for timeout check of time between response header and response */
    int32_t lastRXCancelTime;
    /**! \brief receive transaction, used for UART driver */
    UART_Transaction rxTransaction;
    /**! \brief buffer for rx data */
    uint8_t aUartDataRx[SMIDIRECT_UART_SIZE_RX_BUFFER];
    /**! \brief collecting buffer for SMI command */
    uint8_t aSerialDataRx[IOL_SERIAL_RX_DEFAULT_LENGTH];
} SMIdirect_UART_transfer_t;

void vUartRxCallback(UART_Handle handle, UART_Transaction *transaction);
void vUartTxCallback(UART_Handle handle, UART_Transaction* transaction);
void vUARTrun(void * const pArg_p);

void SMIdirect_UART_RecvCB(IOLM_SMI_SHeader* pGenericHeader_p, uint8_t* pGenericData_p);
void SMIdirect_UART_SendCB(IOLM_SMI_SHeader* pHeader_p, INT8U* pArgBlock_p);
GW_API_EErrorcode_t SMIdirect_UART_stop(void);
GW_API_EErrorcode_t SMIdirect_UART_start(void);

#endif /* PROTECT_SMIDIRECT_UART_H */
