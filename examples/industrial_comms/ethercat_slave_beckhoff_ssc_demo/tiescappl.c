/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <ecat_def.h>

#if TIESC_APPLICATION

#if COE_SUPPORTED
#include <objdef.h>
#endif
#include <ecatappl.h>

#include <tiescutils.h>

#define _TIESC_HW_
#include <ecatslv.h>
#include <tiescappl.h>
#undef _TIESC_HW_

#include <tiescsoc.h>

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern uint32_t appState;

/* ========================================================================== */
/*                       Function Definitions                                 */
/* ========================================================================== */

/*-----------------------------------------------------------------------------------------
------
------    application specific functions
------
-----------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    generic functions
------
-----------------------------------------------------------------------------------------*/
/////////////////////////////////////////////////////////////////////////////////////////
/**

\brief This function resets the outputs
*////////////////////////////////////////////////////////////////////////////////////////

void PDO_ResetOutputs(void)
{
    sDOOutputs.LEDs = 0x00;
    sDO1Outputs.Cmd = 0x00;
    sDO1Outputs.Count = 0x00;
    sDO1Outputs.MotorData = 0x1000;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    The function is called when an error state was acknowledged by the master

*////////////////////////////////////////////////////////////////////////////////////////

void    APPL_AckErrorInd(uint16_t stateTrans)
{

}
/*ECATCHANGE_START(V4.42.1) ECAT2*/
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from INIT to PREOP when
             all general settings were checked to start the mailbox handler. This function
             informs the application about the state transition, the application can refuse
             the state transition when returning an AL Status error code.
            The return code NOERROR_INWORK can be used, if the application cannot confirm
            the state transition immediately, in that case this function will be called cyclically
            until a value unequal NOERROR_INWORK is returned

*////////////////////////////////////////////////////////////////////////////////////////

uint16_t APPL_StartMailboxHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from PREEOP to INIT
             to stop the mailbox handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*////////////////////////////////////////////////////////////////////////////////////////

uint16_t APPL_StopMailboxHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param    pIntMask    pointer to the AL Event Mask which will be written to the AL event Mask
                        register (0x204) when this function is succeeded. The event mask can be adapted
                        in this function
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from PREOP to SAFEOP when
             all general settings were checked to start the input handler. This function
             informs the application about the state transition, the application can refuse
             the state transition when returning an AL Status error code.
             When returning ALSTATUSCODE_NOERROR, the inputs has to be updated once before return.
            The return code NOERROR_INWORK can be used, if the application cannot confirm
            the state transition immediately, in that case the application need to be complete
            the transition by calling ECAT_StateChange
*////////////////////////////////////////////////////////////////////////////////////////

uint16_t APPL_StartInputHandler(uint16_t *pIntMask)
{
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from SAFEOP to PREEOP
             to stop the input handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*////////////////////////////////////////////////////////////////////////////////////////

uint16_t APPL_StopInputHandler(void)
{
#if ESC_EEPROM_EMULATION

    /*
        Since both SPI Flash and HVS (input) are on the same SPI instance, application will have to make sure that
        access to both does not happen concurrently. At this context, it is ensured that input(HVS) read wont happen.
    */
    if(bsp_get_eeprom_update_status())
    {
        bsp_set_eeprom_update_status(0);
        bsp_eeprom_emulation_flush();
    }

#endif
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief    The function is called in the state transition from SAFEOP to OP when
             all general settings were checked to start the output handler. This function
             informs the application about the state transition, the application can refuse
             the state transition when returning an AL Status error code.
            The return code NOERROR_INWORK can be used, if the application cannot confirm
            the state transition immediately, in that case this function will be called cyclically
            until a value unequal NOERROR_INWORK is returned
*////////////////////////////////////////////////////////////////////////////////////////

uint16_t APPL_StartOutputHandler(void)
{
    /*always return NOERROR_INWORK the state transition will be finished by calling AL_ControlRes*/
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return     0, NOERROR_INWORK

 \brief    The function is called in the state transition from OP to SAFEOP
             to stop the output handler. This functions informs the application
             about the state transition, the application cannot refuse
             the state transition.

*////////////////////////////////////////////////////////////////////////////////////////

uint16_t APPL_StopOutputHandler(void)
{
    PDO_ResetOutputs();
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\return     0(ALSTATUSCODE_NOERROR), NOERROR_INWORK
\param      pInputSize  pointer to save the input process data length
\param      pOutputSize  pointer to save the output process data length

\brief    This function calculates the process data sizes from the actual SM-PDO-Assign
            and PDO mapping
*////////////////////////////////////////////////////////////////////////////////////////
uint16_t APPL_GenerateMapping(uint16_t *pInputSize, uint16_t *pOutputSize)
{
    uint16_t result = 0;
    uint16_t PDOAssignEntryCnt = 0;
    OBJCONST TOBJECT OBJMEM *pPDO = NULL;
    uint16_t PDOSubindex0 = 0;
    uint32_t *pPDOEntry = NULL;
    uint16_t PDOEntryCnt = 0;
    uint16_t InputSize = 0;
    uint16_t OutputSize = 0;

    /*Scan object 0x1C12 RXPDO assign*/
    for(PDOAssignEntryCnt = 0; PDOAssignEntryCnt < sRxPDOassign.u16SubIndex0;
            PDOAssignEntryCnt++)
    {
        pPDO = OBJ_GetObjectHandle(sRxPDOassign.aEntries[PDOAssignEntryCnt]);

        if(pPDO != NULL)
        {
            PDOSubindex0 = *((uint16_t *)pPDO->pVarPtr);

            for(PDOEntryCnt = 0; PDOEntryCnt < PDOSubindex0; PDOEntryCnt++)
            {
                pPDOEntry = (uint32_t *)((uint8_t *)pPDO->pVarPtr + (OBJ_GetEntryOffset((
                                             PDOEntryCnt + 1), pPDO) >> 3));     //goto PDO entry
                // we increment the expected output size depending on the mapped Entry
                OutputSize += (uint16_t)((*pPDOEntry) & 0xFF);
            }
        }
    }

    OutputSize = (OutputSize + 7) >> 3;

    /*Scan Object 0x1C13 TXPDO assign*/
    for(PDOAssignEntryCnt = 0; PDOAssignEntryCnt < sTxPDOassign.u16SubIndex0;
            PDOAssignEntryCnt++)
    {
        pPDO = OBJ_GetObjectHandle(sTxPDOassign.aEntries[PDOAssignEntryCnt]);

        if(pPDO != NULL)
        {
            PDOSubindex0 = *((uint16_t *)pPDO->pVarPtr);

            for(PDOEntryCnt = 0; PDOEntryCnt < PDOSubindex0; PDOEntryCnt++)
            {
                pPDOEntry = (uint32_t *)((uint8_t *)pPDO->pVarPtr + (OBJ_GetEntryOffset((
                                             PDOEntryCnt + 1), pPDO) >> 3));     //goto PDO entry
                // we increment the expected output size depending on the mapped Entry
                InputSize += (uint16_t)((*pPDOEntry) & 0xFF);
            }
        }
    }

    InputSize = (InputSize + 7) >> 3;

    *pInputSize = InputSize;
    *pOutputSize = OutputSize;
    return result;
}


/////////////////////////////////////////////////////////////////////////////////////////
/**
\param      pData  pointer to input process data
\brief      This function will copies the inputs from the local memory to the ESC memory
            to the hardware
*////////////////////////////////////////////////////////////////////////////////////////
void APPL_InputMapping(uint16_t *pData)
{
    uint16_t j = 0;
    uint8_t *pTmpData = (uint8_t *)pData;


    for(j = 0; j < sTxPDOassign.u16SubIndex0; j++)
    {
        switch(sTxPDOassign.aEntries[j])
        {
            /* TxPDO 1 */
            case 0x1A00:
                *pTmpData++ = sDIInputs.switchs;
                break;

            case 0x1A03: // attention: maybe a non-aligned byte access...
                *pTmpData++ = sAI1Inputs.info1 & 0xFF;
                *pTmpData++ = (sAI1Inputs.info1 & 0xFF00) >> 8;
                *pTmpData++ = (sAI1Inputs.info1 & 0xFF0000) >> 16;
                *pTmpData++ = (sAI1Inputs.info1 & 0xFF000000) >> 24;
                *pTmpData++ = sAI1Inputs.info2 & 0xFF;
                *pTmpData++ = (sAI1Inputs.info2 & 0xFF00) >> 8;
                break;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\param      pData  pointer to output process data

\brief    This function will copies the outputs from the ESC memory to the local memory
            to the hardware
*////////////////////////////////////////////////////////////////////////////////////////
void APPL_OutputMapping(uint16_t *pData)
{
    uint16_t j = 0;
    uint8_t *pTmpData = (uint8_t *)pData;   // allow byte processing

    for(j = 0; j < sRxPDOassign.u16SubIndex0; j++)
    {
        switch(sRxPDOassign.aEntries[j])
        {
            /* RxPDO 2 */
            case 0x1601:
                sDOOutputs.LEDs = *pTmpData++;
                break;

            case 0x1602:
                sDO1Outputs.Count = *pTmpData++;
                sDO1Outputs.Cmd   = *pTmpData++;
                sDO1Outputs.MotorData = *pTmpData++;
                sDO1Outputs.MotorData |= (*pTmpData++ << 8);
                break;
        }
    }
}

void APPL_Application(void)
{
    uint8_t LED; // initial test data
    static uint8_t prevState = 55;

    LED = sDOOutputs.LEDs;

#ifdef ENABLE_PDI_TASK
    if(LED != prevState)
    {
        tiesc_setOutputLed(LED);
    }
#endif

    prevState = LED;
    appState = sDO1Outputs.Cmd;// set the application state

    /*TODO: Enable this when APIs are available*/
//#ifdef ENABLE_PDI_TASK
//    Board_getDigInput(&sDIInputs.switchs);
//#endif

    if(appState == 0)
    {
        appState =
            sDIInputs.switchs;    //special mode to control app state by input switchs!
    }

    sAI1Inputs.info1 = 0x12345600 | LED;
    HW_EscReadWord(sAI1Inputs.info2, 0x10);
}

#if EXPLICIT_DEVICE_ID
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    The Explicit Device ID of the EtherCAT slave

 \brief     Calculate the Explicit Device ID
*////////////////////////////////////////////////////////////////////////////////////////
uint16_t APPL_GetDeviceID()
{
    return 0xBABA;
}
#endif

/////////////////////////////////////////////////////////////////////////////////////////
/**

 \brief    This is the main function

*////////////////////////////////////////////////////////////////////////////////////////
int ethercat_slave_beckhoff_ssc_demo_main()
{
    common_main();
    return 0;
}
/*ECATCHANGE_END(V4.42.1) ECAT2*/
#endif //#if TIESC_APPLICATION
