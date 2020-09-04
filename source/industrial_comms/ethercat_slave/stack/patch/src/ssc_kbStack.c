/*!
* \file ssc_kbStack.c
*
* \brief
* Beckhoff EC SSC Integration: Application callback Implementation.
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

#include <ssc_kbStack.h>

#include "ssc_backend.h"
#include <pru_syncManager.h>
#include <ecSlvApi_Eeprom.h>
#if EXPLICITWARN
#warning this object dictionary here has to be filled with life by API !!!!!
#endif
#if !KUNBUS_SSC_EVAL
    #define KUNBUS_SSC_LICENSE "VALIDETGMEMBER"
#endif

/* Beckhoff variable from SSC 5.12 */
TOBJECT OBJMEM *ApplicationObjDic = NULL;
/* /Beckhoff */

/*! <!-- Description: -->
 *
 *  \brief
 *  Input Mapping callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pData		Process data pointer.
 *
 *  <!-- Example: -->
 *
 *  <!-- References: -->
 *
 *  \sa APPL_OutputMapping, APPL_GenerateMapping, APPL_Application, APPL_AckErrorInd, APPL_StopOutputHandler, APPL_StartOutputHandler,
 *      APPL_StopInputHandler, APPL_StartInputHandler, APPL_StopMailboxHandler, APPL_StartMailboxHandler, APPL_ResetOutputs
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void APPL_InputMapping(uint16_t *pData)
{
    SSC_inputMapping(pData);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Input Mapping callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pData		Process data pointer.
 *
 *  <!-- Example: -->
 *
 *  <!-- References: -->
 *
 *  \sa APPL_InputMapping, APPL_GenerateMapping, APPL_Application, APPL_AckErrorInd, APPL_StopOutputHandler, APPL_StartOutputHandler,
 *      APPL_StopInputHandler, APPL_StartInputHandler, APPL_StopMailboxHandler, APPL_StartMailboxHandler, APPL_ResetOutputs
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void APPL_OutputMapping(uint16_t *pData)
{
    SSC_outputMapping(pData);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Application callback
 *
 *  <!-- References: -->
 *
 *  \sa APPL_InputMapping, APPL_GenerateMapping, APPL_OutputMapping, APPL_AckErrorInd, APPL_StopOutputHandler, APPL_StartOutputHandler,
 *      APPL_StopInputHandler, APPL_StartInputHandler, APPL_StopMailboxHandler, APPL_StartMailboxHandler, APPL_ResetOutputs
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void APPL_Application(void)
{
    SSC_application();
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Generate Mapping callback
 *
 *  <!-- Parameters and return values: -->
 *  \param[out] pi16uInputSize  Input Mapping Size
 *  \param[out] pi16OutputSize  Output Mapping Size
 *  \return     uint16_t        SSC error code
 *
 *  <!-- References: -->
 *
 *  \sa APPL_InputMapping, APPL_Application, APPL_OutputMapping, APPL_AckErrorInd, APPL_StopOutputHandler, APPL_StartOutputHandler,
 *      APPL_StopInputHandler, APPL_StartInputHandler, APPL_StopMailboxHandler, APPL_StartMailboxHandler, APPL_ResetOutputs
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t APPL_GenerateMapping(uint16_t *pi16uInputSize, uint16_t *pi16OutputSize)
{
    return SSC_generateMapping(pi16uInputSize, pi16OutputSize);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Start Mailbox Handler callback
 *
 *  <!-- Parameters and return values: -->
 *  \return     uint16_t        SSC error code
 *
 *  \sa APPL_InputMapping, APPL_Application, APPL_OutputMapping, APPL_AckErrorInd, APPL_StopOutputHandler, APPL_StartOutputHandler,
 *      APPL_StopInputHandler, APPL_StartInputHandler, APPL_StopMailboxHandler, APPL_GenerateMapping, APPL_ResetOutputs
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t APPL_StartMailboxHandler(void)
{
#if !KUNBUS_SSC_EVAL
    EC_API_SLV_SSC_setLicense(KUNBUS_SSC_LICENSE, strlen(KUNBUS_SSC_LICENSE));
#endif
    return SSC_startMailboxHandler();
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Stop Mailbox Handler callback
 *
 *  <!-- Parameters and return values: -->
 *  \return     uint16_t        SSC error code
 *
 *  <!-- References: -->
 *
 *  \sa APPL_InputMapping, APPL_Application, APPL_OutputMapping, APPL_AckErrorInd, APPL_StopOutputHandler, APPL_StartOutputHandler,
 *      APPL_StopInputHandler, APPL_StartInputHandler, APPL_StartMailboxHandler, APPL_GenerateMapping, APPL_ResetOutputs
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t APPL_StopMailboxHandler(void)
{
    return SSC_stopMailboxHandler();
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Start Input Handler callback
 *
 *  <!-- Parameters and return values: -->
 *  \parm[in]   pIntMask        Interrupt Mask
 *  \return     uint16_t        SSC error code
 *
 *  <!-- References: -->
 *
 *  \sa APPL_InputMapping, APPL_Application, APPL_OutputMapping, APPL_AckErrorInd, APPL_StopOutputHandler, APPL_StartOutputHandler,
 *      APPL_StopInputHandler, APPL_StopMailboxHandler, APPL_StartMailboxHandler, APPL_GenerateMapping, APPL_ResetOutputs
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t APPL_StartInputHandler(uint16_t *pIntMask)
{
    return SSC_startInputHandler(pIntMask);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Stop Input Handler callback
 *
 *  <!-- Parameters and return values: -->
 *  \return     uint16_t        SSC error code
 *
 *  <!-- Example: -->
 *
 *  <!-- References: -->
 *
 *  \sa APPL_InputMapping, APPL_Application, APPL_OutputMapping, APPL_AckErrorInd, APPL_StopOutputHandler, APPL_StartOutputHandler,
 *      APPL_StartInputHandler, APPL_StopMailboxHandler, APPL_StartMailboxHandler, APPL_GenerateMapping, APPL_ResetOutputs
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t APPL_StopInputHandler(void)
{
    return SSC_stopInputHandler();
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Start Output Handler callback
 *
 *  <!-- Parameters and return values: -->
 *  \return     uint16_t        SSC error code
 *
 *  <!-- References: -->
 *
 *  \sa APPL_InputMapping, APPL_Application, APPL_OutputMapping, APPL_AckErrorInd, APPL_StopOutputHandler, APPL_StopInputHandler,
 *      APPL_StartInputHandler, APPL_StopMailboxHandler, APPL_StartMailboxHandler, APPL_GenerateMapping, APPL_ResetOutputs
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t APPL_StartOutputHandler(void)
{
    return SSC_startOutputHandler();
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Start Output Handler callback
 *
 *  <!-- Parameters and return values: -->
 *  \return     uint16_t        SSC error code
 *
 *  <!-- Example: -->
 *
 *  <!-- References: -->
 *
 *  \sa APPL_InputMapping, APPL_Application, APPL_OutputMapping, APPL_AckErrorInd, APPL_StartOutputHandler, APPL_StopInputHandler,
 *      APPL_StartInputHandler, APPL_StopMailboxHandler, APPL_StartMailboxHandler, APPL_GenerateMapping, APPL_ResetOutputs
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t APPL_StopOutputHandler(void)
{
    return SSC_stopOutputHandler();
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Start Output Handler callback
 *
 *  <!-- Parameters and return values: -->
 *  \param[in]  stateTrans  State transission of error
 *
 *  <!-- References: -->
 *
 *  \sa APPL_InputMapping, APPL_Application, APPL_OutputMapping, APPL_StopOutputHandler, APPL_StartOutputHandler, APPL_StopInputHandler,
 *      APPL_StartInputHandler, APPL_StopMailboxHandler, APPL_StartMailboxHandler, APPL_GenerateMapping, APPL_ResetOutputs
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void APPL_AckErrorInd(uint16_t stateTrans)
{
    SSC_ackErrorInd(stateTrans);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  SoE continue indicator
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  parameter		Parameter description.
 *  \return     ErrorCode p Closer description of ErrorCode, if required.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <ssc.h>
 *
 *  // required variables
 *  uint32_t retVal = 0;
 *  void* pvVariable = NULL;
 *
 *  // the Call
 *  retVal = SOE_ContinueInd(pvVariable);
 *  \endcode
 *
 *  <!-- References: -->
 *
 *  \sa SOE_NotificationReq, SOE_ServiceInd
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVBACKEND
 *
 * */
void SOE_ContinueInd(TMBX MBXMEM * pMbx)
{
    SSC_SOE_send(pMbx);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  This function is called when a ServoDriveProfile over EtherCAT service is received from
 *  the master.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pReceiveMbx		Pointer to the received mailbox data from the master.
 *  \return     ErrorCode p Closer description of ErrorCode, if required.
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <ssc.h>
 *
 *  // required variables
 *  uint32_t retVal = 0;
 *  void* pvVariable = NULL;
 *
 *  // the Call
 *  retVal = SOE_NotificationReq(pvVariable);
 *  \endcode
 *
 *  <!-- References: -->
 *
 *  \sa SOE_NotificationReq, SOE_ContinueInd
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVBACKEND
 *
 * */
UINT8 SOE_ServiceInd(TMBX MBXMEM *pReceiveMbx)
{
   return SSC_SOE_recv(pReceiveMbx);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Bootloader start callback
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  State         Current state.
 *
 *  <!-- References: -->
 *
 *  \sa BL_Stop, BL_Finish
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVBACKEND
 *
 * */
void BL_Start(uint8_t State)
{
    SSC_BL_start(State);
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Bootloader stop callback
 *
 *  <!-- References: -->
 *
 *  \sa BL_Start, BL_Finish
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVBACKEND
 *
 * */
void BL_Stop(void)
{
    SSC_BL_stop();
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Bootloader finish callback
 *
 *  <!-- References: -->
 *
 *  \sa BL_Start, BL_Stop
 *
 *  <!-- Group: -->
 *
 *  \ingroup SLVBACKEND
 *
 * */
void BL_Finish(void)
{
    SSC_BL_finish();
}

/*! <!-- Description: -->
 *
 *  \brief
 *  ESC Write in ISR context
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pData_p     Data
 *  \param[in]  address_p	address to write
 *  \param[in]  length_p    data len
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_ESC_writeIsr(uint8_t *pData_p, uint16_t address_p, uint16_t length_p)
{
    if (SSC_callbacks_g.cbEscWrite)
    {
        SSC_callbacks_g.cbEscWrite(SSC_callbacks_g.pEscWriteCtxt, 1, pData_p, address_p, length_p);
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Write array to ESC through PRU
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pData_p		data.
 *  \param[in]  address_p   Address
 *  \param[in]  length_p    Length of data
 *
 *  <!-- Group: -->
 *
 *  \ingroup group
 *
 * */
void SSC_ESC_write(uint8_t *pData_p, uint16_t address_p, uint16_t length_p)
{
    if (SSC_callbacks_g.cbEscWrite)
    {
        SSC_callbacks_g.cbEscWrite(SSC_callbacks_g.pEscWriteCtxt, 0, pData_p, address_p, length_p);
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Write word to ESC through PRU
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  wordValue_p     data.
 *  \param[in]  address_p       Address
 *
 *  <!-- Group: -->
 *
 *  \ingroup group
 *
 * */
void SSC_ESC_writeWord(uint16_t wordValue_p, uint16_t address_p)
{
    if (SSC_callbacks_g.cbEscWriteWord)
    {
        SSC_callbacks_g.cbEscWriteWord(SSC_callbacks_g.pEscWriteWordCtxt, wordValue_p, address_p);
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Write word to ESC through PRU
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  byteValue_p     data.
 *  \param[in]  address_p       Address
 *
 *  <!-- Group: -->
 *
 *  \ingroup group
 *
 * */
void SSC_ESC_writeByte(uint8_t byteValue_p, uint16_t address_p)
{
    if (SSC_callbacks_g.cbEscWriteByte)
    {
        SSC_callbacks_g.cbEscWriteByte(SSC_callbacks_g.pEscWriteByteCtxt, byteValue_p, address_p);
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Write to mailbox.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pData_p		Data
 *  \param[in]  address_p   address
 *  \param[in]  length_p    Length of Data
 *
 *  <!-- Group: -->
 *  \ingroup ssc
 *
 * */
void SSC_ESC_writeMbxMem(uint8_t *pData_p, uint16_t address_p, uint16_t length_p)
{
    if (SSC_callbacks_g.cbEscWriteMbxMem)
    {
        SSC_callbacks_g.cbEscWriteMbxMem(SSC_callbacks_g.pEscWriteMbxMemCtxt, pData_p, address_p, length_p);
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  ESC Read in ISR context
 *
 *  \details
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[out] pData_p     Data
 *  \param[in]  address_p	address to read
 *  \param[in]  length_p    data len
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_ESC_readIsr(uint8_t *pData_p, uint16_t address_p, uint16_t length_p)
{
    if (SSC_callbacks_g.cbEscRead)
    {
        SSC_callbacks_g.cbEscRead(SSC_callbacks_g.pEscReadCtxt, 1, pData_p, address_p, length_p);
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  ESC Read
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[out] pdata_p     Data
 *  \param[in]  address_p	address to read
 *  \param[in]  length_p    data len
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_ESC_read(uint8_t *pdata_p, uint16_t address_p, uint16_t length_p)
{
    if (SSC_callbacks_g.cbEscRead)
    {
        SSC_callbacks_g.cbEscRead(SSC_callbacks_g.pEscReadCtxt, 0, pdata_p, address_p, length_p);
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  ESC Read Byte in ISR context
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  address_p   address to read
 *  \return     uint8_t     Register value
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint8_t SSC_ESC_readByteIsr(uint16_t address_p)
{
    uint8_t retVal = ~0;

    if (SSC_callbacks_g.cbEscReadByte)
    {
        retVal = SSC_callbacks_g.cbEscReadByte(SSC_callbacks_g.pEscReadByteCtxt, 1, address_p);
    }

    return retVal;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  ESC Read Byte
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  address_p   address to read
 *  \return     uint8_t     Register value
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint8_t SSC_ESC_readByte(uint16_t address_p)
{
    uint8_t retVal = ~0;

    if (SSC_callbacks_g.cbEscReadByte)
    {
        retVal = SSC_callbacks_g.cbEscReadByte(SSC_callbacks_g.pEscReadByteCtxt, 0, address_p);
    }
    return retVal;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  ESC Read Word in ISR context
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  address_p   address to read
 *  \return     uint16_t    Register value
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t SSC_ESC_readWordIsr(uint16_t address_p)
{
    uint16_t retVal = ~0;

    if (SSC_callbacks_g.cbEscReadWord)
    {
        retVal = SSC_callbacks_g.cbEscReadWord(SSC_callbacks_g.pEscReadWordCtxt, 1, address_p);
    }

    return retVal;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  ESC Read Word
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  address_p   address to read
 *  \return     uint16_t    Register value
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint16_t SSC_ESC_readWord(uint16_t address_p)
{
    uint16_t retVal = ~0;

    if (SSC_callbacks_g.cbEscReadWord)
    {
        retVal = SSC_callbacks_g.cbEscReadWord(SSC_callbacks_g.pEscReadWordCtxt, 0, address_p);
    }
    return retVal;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  ESC Read Dword from ISR context
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  address_p   address to read
 *  \return     uint32_t    Value
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint32_t SSC_ESC_readDwordIsr(uint16_t address_p)
{
    uint32_t retVal = ~0;

    if (SSC_callbacks_g.cbEscReadDword)
    {
        retVal = SSC_callbacks_g.cbEscReadDword(SSC_callbacks_g.pEscReadDwordCtxt, 1, address_p);
    }

    return retVal;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  ESC Read Dword
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  address_p   address to read
 *  \return     uint32_t    Value
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
uint32_t SSC_ESC_readDword(uint16_t address_p)
{
    uint32_t retVal = ~0;

    if (SSC_callbacks_g.cbEscReadDword)
    {
        retVal = SSC_callbacks_g.cbEscReadDword(SSC_callbacks_g.pEscReadDwordCtxt, 0, address_p);
    }

    return retVal;
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Read from mailbox.
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[out] pData_p		Data
 *  \param[in]  address_p   address
 *  \param[in]  length_p    Length of Data
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
void SSC_ESC_readMbxMem(uint8_t *pData_p, uint16_t address_p, uint16_t length_p)
{
    if (SSC_callbacks_g.cbEscReadMbxMem)
    {
        SSC_callbacks_g.cbEscReadMbxMem(SSC_callbacks_g.pEscReadMbxMemCtxt, pData_p, address_p, length_p);
    }
}

/*! <!-- Description: -->
 *
 *  \brief
 *  Trigger EEPROM reload
 *
 *  <!-- Parameters and return values: -->
 *  \return int32_t error code
 *
 *  <!-- Example: -->
 *
 *  <!-- References: -->
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
int32_t SSC_EEP_ESC_reload(void)
{
    int32_t retVal = ~0;

    if (SSC_callbacks_g.cbEscEepromLdReg)
    {
        retVal = SSC_callbacks_g.cbEscEepromLdReg(SSC_callbacks_g.pEscEepromLdRegCtxt, 1);
    }

    return retVal;
}

//*************************************************************************************************

