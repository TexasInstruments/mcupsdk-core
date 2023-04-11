/*!
 *  \file SMIdirect_UART.c
 *
 *  \brief
 *  interface to a IOLink SMI GUI connected with UART
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2022-10-17
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

#include <stdint.h>
#include <osal.h>

#include <FreeRTOS.h>
#include <task.h>
#include <ti_drivers_open_close.h>

#include <IOL_Serial.h>
#include <gw_api_interface.h>  

#include "SMIdirect_UART.h"

// mutex to safe receive processing work
OSAL_SCHED_SMutexHandle_t* SMIdirect_UART_pMutexHandle_g;
// event to signal receive or transmit work
OSAL_SCHED_SEventHandle_t* SMIdirect_UART_pEventHandle_g;

// clientId assigned by the gateway
uint8_t  SMIdirect_UART_clientId_g = 0;
uint8_t  SMIdirect_UART_saveCommandClientId_g = 0;

// variables to cummunicate via UART
SMIdirect_UART_transfer_t SMIdirect_UART_sTransData_g;

// Task values
static void* SMIdirect_UART_pTaskHandle_s;
#define SMIDIRECT_UART_TASK_STACKSIZE       (4096U / sizeof(configSTACK_DEPTH_TYPE))
static StackType_t smiDirectUartTaskStack_s[SMIDIRECT_UART_TASK_STACKSIZE] __attribute__((aligned(32), section(".threadstack"))) = {0};

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  UART Rx Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  handle          Handle for UART instance (not used)
 *  \param[in]  transaction     Pointer to UART transaction 
 *
 * */
void vUartRxCallback(UART_Handle handle, UART_Transaction *transaction)
{
    OSALUNREF_PARM(handle);

    // mark receive ready for UART task
    SMIdirect_UART_sTransData_g.currentRxCount = transaction->count;
    SMIdirect_UART_sTransData_g.isRxComplete = true;

    // wakeup UART task
    OSAL_EVT_set(SMIdirect_UART_pEventHandle_g);
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  UART Tx Callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  handle          Handle for UART instance (not used)
 *  \param[in]  transaction     Pointer to UART transaction (not used)
 *
 * */
void vUartTxCallback(UART_Handle handle, UART_Transaction* transaction)
{
    OSALUNREF_PARM(handle);
    OSALUNREF_PARM(transaction);

    // signal IOL_Serial the finished transfer
    IOL_Serial_vReleaseTxBuffer(&SMIdirect_UART_sTransData_g.iolSerial);

    // TX is empty and ready for a new transmit
    SMIdirect_UART_sTransData_g.isTxPending = false;

    // wakeup UART task
    OSAL_EVT_set(SMIdirect_UART_pEventHandle_g);
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  UARTtask handles the communication with UART driver
 *  task will be triggered if
 *       * a block received by UART (header or command data)
 *       * a response is transferred to UART (not really needed)
 *       * a response is for transfer available
 *       * timeout, if command data expected, but not received
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]       pArg_p        not used
 *
 * */
void OSAL_FUNC_NORETURN SMIdirect_UARTtask(void* pArg_p)
{
    OSALUNREF_PARM(pArg_p);

    // loop forever
    while (true)
    {
        uint32_t evtTimeout = OSAL_WAIT_INFINITE;

        if ((!SMIdirect_UART_sTransData_g.headerActive) && ((OSAL_getMsTick() - SMIdirect_UART_sTransData_g.lastRXCancelTime) >
            (IOL_SERIAL_TIMEOUT_MS / IOLM_SYSTICK_INTERVAL_MS)))
        {
            // command data was expected but not received, kill transaction
            UART_readCancel(SMIdirect_UART_sTransData_g.uartHandle, &SMIdirect_UART_sTransData_g.rxTransaction);

            // signal received command data with length 0, task will prepare a new receive for header
            SMIdirect_UART_sTransData_g.isRxComplete = true;
            SMIdirect_UART_sTransData_g.currentRxCount = 0;
        }

        if (!SMIdirect_UART_sTransData_g.isTxPending)
        {
            // transmitter is empty, check for new data
            uint32_t txCount;
            uint8_t* pTxData;
            pTxData = IOL_Serial_pu8GetTxBuffer(&SMIdirect_UART_sTransData_g.iolSerial, &txCount);
            if (NULL != pTxData)
            {
                // new data to send available
                UART_Transaction_init(&SMIdirect_UART_sTransData_g.txTransaction);
                SMIdirect_UART_sTransData_g.txTransaction.count = txCount;
                SMIdirect_UART_sTransData_g.txTransaction.buf = (void*)pTxData;
                SMIdirect_UART_sTransData_g.txTransaction.args = NULL;
                // signal a transmitting UART
                SMIdirect_UART_sTransData_g.isTxPending = true;
                UART_write(SMIdirect_UART_sTransData_g.uartHandle, &SMIdirect_UART_sTransData_g.txTransaction);
            }
        }

        if (SMIdirect_UART_sTransData_g.isRxComplete)
        {
            // response data received, store data in IOL_Serial
            IOL_Serial_vReceive(&SMIdirect_UART_sTransData_g.iolSerial, SMIdirect_UART_sTransData_g.aSerialDataRx, SMIdirect_UART_sTransData_g.currentRxCount);

            // prepare receiving next message (next header or command data expected)
            if (SMIdirect_UART_sTransData_g.headerActive && (SMIdirect_UART_sTransData_g.currentRxCount >= sizeof(IOLM_SMI_SHeader)))
            {
                // a header is completely received ==> next receive are the command data 
                IOLM_SMI_SHeader* pHead = (IOLM_SMI_SHeader*)SMIdirect_UART_sTransData_g.aSerialDataRx;
                SMIdirect_UART_sTransData_g.headerActive = false;
                SMIdirect_UART_sTransData_g.expectRxCount = pHead->u16ArgBlockLength;
                SMIdirect_UART_sTransData_g.lastRXCancelTime = OSAL_getMsTick();
                evtTimeout = (IOL_SERIAL_TIMEOUT_MS+10) / IOLM_SYSTICK_INTERVAL_MS;
            }
            else
            {
                // a new command sequence shall started ==> wait for new header
                SMIdirect_UART_sTransData_g.headerActive = true;
                SMIdirect_UART_sTransData_g.expectRxCount = sizeof(IOLM_SMI_SHeader);
            }

            // start a new UART RX transaction
            SMIdirect_UART_sTransData_g.currentRxCount = 0U;
            SMIdirect_UART_sTransData_g.isRxComplete = false;
            UART_Transaction_init(&SMIdirect_UART_sTransData_g.rxTransaction);
            SMIdirect_UART_sTransData_g.rxTransaction.count = SMIdirect_UART_sTransData_g.expectRxCount;
            SMIdirect_UART_sTransData_g.rxTransaction.buf = (void*)SMIdirect_UART_sTransData_g.aSerialDataRx;
            SMIdirect_UART_sTransData_g.rxTransaction.args = NULL;
            UART_read(SMIdirect_UART_sTransData_g.uartHandle, &SMIdirect_UART_sTransData_g.rxTransaction);

            // trigger IOL_Serial to process received data
            OSAL_MTX_get(SMIdirect_UART_pMutexHandle_g, OSAL_WAIT_INFINITE, NULL);
            IOL_Serial_vRxProcessing(&SMIdirect_UART_sTransData_g.iolSerial);
            OSAL_MTX_release(SMIdirect_UART_pMutexHandle_g);
        }

        // wait for a new receive or transmit signal, or a timeout
        OSAL_EVT_wait(SMIdirect_UART_pEventHandle_g, evtTimeout, NULL);
    }
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  RecvCB is called from gateway after a response from IOLink Master was received
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pHeader_p     Header data
 *  \param[in]  pArgBlock_p   Argblock
 *
 * */
void SMIdirect_UART_RecvCB(IOLM_SMI_SHeader* pGenericHeader_p, uint8_t* pGenericData_p)
{
    // change clientId from assigned one to command expected clientId 
    pGenericHeader_p->u8ClientId = SMIdirect_UART_saveCommandClientId_g;

    // a generic SMI resonse from IOLink Master is received
    OSAL_MTX_get(SMIdirect_UART_pMutexHandle_g, OSAL_WAIT_INFINITE, NULL);
    IOL_Serial_vSendSmi(&SMIdirect_UART_sTransData_g.iolSerial, pGenericHeader_p, pGenericData_p);
    OSAL_MTX_release(SMIdirect_UART_pMutexHandle_g);

    // inform UART task
    OSAL_EVT_set(SMIdirect_UART_pEventHandle_g);
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  SendCB is called from IOL_Serial after a completely command was received
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pHeader_p     Header data
 *  \param[in]  pArgBlock_p   Argblock
 *
 * */
void SMIdirect_UART_SendCB(IOLM_SMI_SHeader* pHeader_p, INT8U* pArgBlock_p)
{
    // change clientId from command to assigned one
    SMIdirect_UART_saveCommandClientId_g = pHeader_p->u8ClientId;
    pHeader_p->u8ClientId = SMIdirect_UART_clientId_g;

    // send a generic SMI command to IOLink Master
    GW_API_smiGenericCommand(SMIdirect_UART_clientId_g, pHeader_p, pArgBlock_p);
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Set up neccessary configurations for UART and start SMIdirect_UARTtask
 *
 *  <!-- Parameters and return values: -->
 *
 *  \return  OSAL_MTXCTRLBLK_alloc     mutex create error
 *           GW_API_eSMIDIRECT_NOEVT   event create error
 *           GW_API_eSMIDIRECT_NOUART  no UART driver found   
 *           GW_API_eSMIDIRECT_TASK    task create error   
 *           GW_API_eNOCLIENTID        no clientId assigned        
 *
 * */
GW_API_EErrorcode_t SMIdirect_UART_stop(void)
{
    uint32_t errorCode = GW_API_eSUCCESS;

    // TODO implement stop function (at the moment the stop functions like OSAL_SCHED_killTask() don't work yet)

    return errorCode;
}

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Set up neccessary configurations for UART and start SMIdirect_UARTtask
 *
 *  <!-- Parameters and return values: -->
 *
 *  \return  OSAL_MTXCTRLBLK_alloc     mutex create error
 *           GW_API_eSMIDIRECT_NOEVT   event create error
 *           GW_API_eSMIDIRECT_NOUART  no UART driver found   
 *           GW_API_eSMIDIRECT_TASK    task create error   
 *           GW_API_eNOCLIENTID        no clientId assigned        
 *
 * */
GW_API_EErrorcode_t SMIdirect_UART_start(void)
{
    GW_API_EErrorcode_t errorCode = GW_API_eSUCCESS;

    SMIdirect_UART_sTransData_g.iolSerial.u32BufferCount   = SMIDIRECT_UART_NUM_TX_BUFFER;
    SMIdirect_UART_sTransData_g.iolSerial.u32BufferLen     = SMIDIRECT_UART_SIZE_TX_BUFFER;
    SMIdirect_UART_sTransData_g.iolSerial.pu8Data          = SMIdirect_UART_sTransData_g.aSerialDataTx;
    SMIdirect_UART_sTransData_g.iolSerial.pu32Length       = SMIdirect_UART_sTransData_g.aSerialLengthTx;
    SMIdirect_UART_sTransData_g.iolSerial.pu8RxBuffer      = SMIdirect_UART_sTransData_g.aUartDataRx;
    SMIdirect_UART_sTransData_g.iolSerial.u32RxBufferLen   = sizeof(SMIdirect_UART_sTransData_g.aUartDataRx);
    SMIdirect_UART_sTransData_g.iolSerial.cbRxCallback     = SMIdirect_UART_SendCB;
    IOL_Serial_vInit(&SMIdirect_UART_sTransData_g.iolSerial);

    SMIdirect_UART_pMutexHandle_g  = OSAL_MTXCTRLBLK_alloc();
    if (NULL == SMIdirect_UART_pMutexHandle_g)
    {
        errorCode = GW_API_eSMIDIRECT_NOMBX;
        // @cppcheck_justify{misra-c2012-15.1} use goto Exit for single point of return
        //cppcheck-suppress misra-c2012-15.1
        goto laError;
    }
    OSAL_MTX_init(SMIdirect_UART_pMutexHandle_g);
    SMIdirect_UART_pEventHandle_g  = OSAL_EVTCTRLBLK_alloc();
    if (NULL == SMIdirect_UART_pMutexHandle_g)
    {
        errorCode = GW_API_eSMIDIRECT_NOEVT;
        // @cppcheck_justify{misra-c2012-15.1} use goto Exit for single point of return
        //cppcheck-suppress misra-c2012-15.1
        goto laError;
    }
    OSAL_EVT_init(SMIdirect_UART_pEventHandle_g);

    // store driver handle for used UART (defined in sysconfig)
    SMIdirect_UART_sTransData_g.uartHandle = gUartHandle[CONFIG_UART_SMI];
    if (NULL == SMIdirect_UART_sTransData_g.uartHandle) 
    {
        errorCode = GW_API_eSMIDIRECT_NOUART;
        // @cppcheck_justify{misra-c2012-15.1} use goto Exit for single point of return
        //cppcheck-suppress misra-c2012-15.1
        goto laError;
    }

    // dummy receive to trigger correct receive
    SMIdirect_UART_sTransData_g.isRxComplete = true;
    SMIdirect_UART_sTransData_g.isTxPending = false;
    SMIdirect_UART_sTransData_g.headerActive = false;
    SMIdirect_UART_sTransData_g.currentRxCount = 0;
    SMIdirect_UART_sTransData_g.expectRxCount = 0;

    SMIdirect_UART_pTaskHandle_s = OSAL_SCHED_startTask((OSAL_SCHED_CBTask_t)SMIdirect_UARTtask,
                                                  NULL,
                                                  OSAL_TASK_ePRIO_20,
                                                  (uint8_t*)smiDirectUartTaskStack_s,
                                                  sizeof(smiDirectUartTaskStack_s),
                                                  OSAL_OS_START_TASK_FLG_NONE,
                                                  "SMIdirect UART Task");

    if (NULL == SMIdirect_UART_pTaskHandle_s)
    {
        errorCode = GW_API_eSMIDIRECT_TASK;
        // @cppcheck_justify{misra-c2012-15.1} use goto Exit for single point of return
        //cppcheck-suppress misra-c2012-15.1
        goto laError;
    }

    errorCode = GW_API_registerSMIClient(0, 0, &SMIdirect_UART_clientId_g, SMIdirect_UART_RecvCB);
    if (GW_API_eSUCCESS != errorCode)
    {
        errorCode = GW_API_eNOCLIENTID;
        // @cppcheck_justify{misra-c2012-15.1} use goto Exit for single point of return
        //cppcheck-suppress misra-c2012-15.1
        goto laError;
    }

laError:
    return errorCode;
}

