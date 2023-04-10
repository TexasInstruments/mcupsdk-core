/*!
* \file IOLM_Port_smiExample.c
*
* \brief
* Example application to show how the IO-Link Master Stack can be used via the SMI.
*
* \author
* KUNBUS GmbH
*
* \date
* 2021-10-06
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

#include <stdint.h>
#include "IOLM_Port_version.h"
#include "IOLM_Port_smiExample.h"
#include "IOLinkPort/IOLM_Port_SMI.h"

IOLM_SMI_SCallbacks IOLM_EXMPL_SSmiCallbacks_g =
{
    /* Generic channel for SMI over UART */
    .cbChipInfo                 =       IOLM_EXMPL_cbChipInfo,
    .cbGenericCnf               =       IOLM_SMI_cbGenericCnf,
    /* Confirmation for port status request */
    .cbPortStatusCnf            =       IOLM_EXMPL_cbPortStatusCnf,
#if (IOLM_EXMPL_ENABLE_STATE_MACHINE == 1)
    /* Master Identification */
    .cbMasterIdentificationCnf  =       IOLM_EXMPL_cbMasterIdentificationCnf,
    /* Event handling(acyclic) */
    .cbPortEventInd             =       IOLM_EXMPL_PortEventInd,
    .cbDeviceEventInd           =       IOLM_EXMPL_DeviceEventInd,
    /* Communication establishment */
    .cbPortConfigurationCnf     =       IOLM_EXMPL_cbPortConfigurationCnf,
    /* Acyclic communication */
    .cbDeviceWriteCnf           =       IOLM_SMI_cbDeviceWriteCnf,
    .cbDeviceReadCnf            =       IOLM_EXMPL_cbDeviceReadCnf,
    /* Cyclic communication */
    .cbPDInCnf                  =       IOLM_EXMPL_cbPDInCnf,
    .cbPDOutCnf                 =       IOLM_EXMPL_cbPDOutCnf,
#endif
    /* Mainloop Request - called by stack to indicate SMI mainloop run is required */
    .cbMainLoopRequest          =       IOLM_MAIN_cbMainLoopRequest,
};

IOLM_EXMPL_SPortDataValues_t port[IOLM_PORT_COUNT + 1];

/*!
 * \brief
 * Update LEDs of specific port.
 *
 * This function sends the desired target state for the red and green LED
 * on the IO-Link add-on board to the LED handler.
 * In the handler call it is possible to turn on/off LEDs or set LEDs to a slow or fast blinking state.
 * The current status of the port is retrieved from the global status sctructure port[]
 * and therefore needs to be updated to the current status beforehand.
 *
 * \param[in]  portNumber_p            Number of the target port.
 *
 * \return void
 *
 */
void IOLM_EXMPL_updateLEDs(uint8_t portNumber_p)
{
    static IOLM_SMI_EPortStatus previousPortStatus_s[IOLM_PORT_COUNT + 1] =
        { IOLM_SMI_ePortStatus_NOT_AVAILABLE, };

    IOLM_SMI_EPortStatus portStatus = port[portNumber_p].currentStackPortStatus;

    if (portStatus != previousPortStatus_s[portNumber_p])
    {
        switch (portStatus)
        {
        case IOLM_SMI_ePortStatus_PORT_DIAG:
            IOLM_LED_setLedColorState(portNumber_p, IOLM_eLEDColor_green, IOLM_eLEDState_off);
            IOLM_LED_setLedColorState(portNumber_p, IOLM_eLEDColor_red, IOLM_eLEDState_slow);
            break;
        case IOLM_SMI_ePortStatus_NO_DEVICE:
            IOLM_LED_setLedColorState(portNumber_p, IOLM_eLEDColor_green, IOLM_eLEDState_slow);
            IOLM_LED_setLedColorState(portNumber_p, IOLM_eLEDColor_red, IOLM_eLEDState_off);
            break;
        case IOLM_SMI_ePortStatus_DEACTIVATED:
        case IOLM_SMI_ePortStatus_NOT_AVAILABLE:
            IOLM_LED_setLedColorState(portNumber_p, IOLM_eLEDColor_green, IOLM_eLEDState_off);
            IOLM_LED_setLedColorState(portNumber_p, IOLM_eLEDColor_red, IOLM_eLEDState_off);
            break;
        case IOLM_SMI_ePortStatus_PORT_POWER_OFF:
            IOLM_LED_setLedColorState(portNumber_p, IOLM_eLEDColor_green, IOLM_eLEDState_off);
            IOLM_LED_setLedColorState(portNumber_p, IOLM_eLEDColor_red, IOLM_eLEDState_off);
            break;
        case IOLM_SMI_ePortStatus_PREOPERATE:
            IOLM_LED_setLedColorState(portNumber_p, IOLM_eLEDColor_green, IOLM_eLEDState_fast);
            IOLM_LED_setLedColorState(portNumber_p, IOLM_eLEDColor_red, IOLM_eLEDState_off);
            break;
        case IOLM_SMI_ePortStatus_OPERATE:
        case IOLM_SMI_ePortStatus_DI_CQ:
        case IOLM_SMI_ePortStatus_DO_CQ:
            IOLM_LED_setLedColorState(portNumber_p, IOLM_eLEDColor_green, IOLM_eLEDState_on);
            IOLM_LED_setLedColorState(portNumber_p, IOLM_eLEDColor_red, IOLM_eLEDState_off);
            break;
        default:
            break;
        }
        previousPortStatus_s[portNumber_p] = portStatus;
    }
}

/*!
 *  \brief
 *  Initialization of example application
 *
 *  \return     void
 *
 */
void IOLM_EXMPL_init(void)
{
    uint8_t portNumber;
    uint8_t dataValueCounter;
    /* SMI Example init */
    /* Initialize state machine status array */
    /* In SMI context portNumber numbers start with 1 -> portNumber 0 will not be used */

    port[0].exampleState = IOLM_eExampleState_NotUsed;
    
    /* Set all active ports to init state */
    for (portNumber = IOLM_EXMPL_SMI_PORTS_NUMBER_START; portNumber <= IOLM_PORT_COUNT; portNumber++)
    {
        port[portNumber].exampleState = IOLM_eExampleState_Init;
    }

    /* Set all active ports to init port */
    for (portNumber = IOLM_EXMPL_SMI_PORTS_NUMBER_START; portNumber <= IOLM_PORT_COUNT; portNumber++)
     {
        port[portNumber].currentStackPortStatus = IOLM_SMI_ePortStatus_NO_DEVICE;
        for (dataValueCounter = 0; dataValueCounter < PD_INPUT_LENGTH; dataValueCounter++)
        {
            port[portNumber].aPDInCnfData[dataValueCounter] = 0;
        }
    }
    /* Initialize external SMI channel */
    IOLM_SMI_portInit();
    /* IO-Link Master stack init Example init */
    IOLM_SMI_vInit(&IOLM_EXMPL_SSmiCallbacks_g);
}

/*!
 *  \brief
 *  Initialization of example main loop
 *
 *  \return     void
 *
 */
void OSAL_FUNC_NORETURN IOLM_EXMPL_mainLoop(void)
{
    uint8_t portNumber;

    IOLM_EXMPL_printf("---------------START EXAMPLE APPLICATION------------------\n");
    IOLM_SMI_vMasterIdentificationReq(IOLM_SMI_CLIENT_APP);
    OSAL_SCHED_sleep(5); /*Wait for answer*/
    IOLM_EXMPL_printf("\n");
    while (1)
    {
        /* get port status periodically for setting the LEDs */
        for (portNumber = IOLM_EXMPL_SMI_PORTS_NUMBER_START; portNumber <= IOLM_PORT_COUNT; portNumber++)
        {
            IOLM_SMI_vPortStatusReq(IOLM_SMI_CLIENT_APP, portNumber);
        }
#if (IOLM_EXMPL_ENABLE_STATE_MACHINE == 1)
        IOLM_EXMPL_stateMachine();
#endif
        OSAL_SCHED_sleep(IOLM_EXMPL_DELAY_IN_MILISEC);
    }
}

/*!
 *  \brief
 *  Set port to desired port mode.
 *
 *  \param[in]  portNumber_p                port number as uint8_t.
 *  \param[in]  targetPortMode_p            target port mode as IOLM_SMI_EPortMode.
 */
void IOLM_EXMPL_setPortToPortMode(uint8_t portNumber_p, IOLM_SMI_EPortMode targetPortMode_p)
{
    IOLM_SMI_SPortConfigList portConfig;

    OSAL_MEMORY_memset(&portConfig, 0, sizeof(portConfig));
    portConfig.u8PortMode = targetPortMode_p;
    portConfig.u16ArgBlockID = IOLM_SMI_ENDIAN_16(IOLM_SMI_eArgBlockID_PortConfigList);
    IOLM_SMI_vPortConfigurationReq(IOLM_SMI_CLIENT_APP,
                                   portNumber_p,
                                   sizeof(portConfig),
                                   (INT8U*)&portConfig);
}

/*!
 *  \brief
 *  Initialization of example state machine
 *
 *  \return     void
 *
 */
void IOLM_EXMPL_stateMachine(void)
{
    uint8_t portNumber;

    for (portNumber = IOLM_EXMPL_SMI_PORTS_NUMBER_START; portNumber <= IOLM_EXMPL_PORTS_USED_IN_STATE_MACHINE; portNumber++)
    {
        /* STATE MACHINE ARCHITECTURE:  
            --------------------Establishment of the Communication--------------------
                   Application                                              Stack
                   states:                                                  act as:
             1.)   IOLM_eExampleState_Init                   
             2.)   IOLM_eExampleState_Config                    ->          Receiver
             3.)   IOLM_eExampleState_ConfigWait                <-          Transmitter 
             4.)   IOLM_eExampleState_PortStatusRequest         ->          Receiver
             5.)   IOLM_eExampleState_PortStatusWait            <-          Transmitter
             ----------------------------Communication--------------------------------
             6.)   IOLM_eExampleState_ReadVendorName            ->          Receiver
             7.)   IOLM_eExampleState_ReadVendorNameWait        <-          Transmitter
             8.)   IOLM_eExampleState_ReadProductName           ->          Receiver
             9.)   IOLM_eExampleState_ReadProductNameWait       <-          Transmitter
             10.)  IOLM_eExampleState_ReadSerialnumber          ->          Receiver
             11.)  IOLM_eExampleState_ReadSerialnumberWait      <-          Transmitter
             12.)  IOLM_eExampleState_WriteProcessDataValue     ->          Receiver
             13.)  IOLM_eExampleState_WriteProcessDataValueWait <-          Transmitter
             -------------------  Read process data Communication---------------------
             14.)  IOLM_eExampleState_PortStatusRequestPD       ->          Receiver
             15.)  IOLM_eExampleState_PortStatusWaitPD          <-          Transmitter
             16.)  IOLM_eExampleState_ReadProcessDataValue      ->          Receiver
             17.)  IOLM_eExampleState_ReadProcessDataValueWait  <-          Transmitter
             18.)  IOLM_eExampleState_CheckForOperate           ->          Receiver
             --.)  default:IOLM_eExampleState_PortStatusWait    <-          Transmitter

            Note: The state machine is not strictly executed in this specific sequence,
            see for example IOLM_EXMPL_PortErrorHandler.
        */
        switch (port[portNumber].exampleState)
        {
            case IOLM_eExampleState_Init:                                                                         /*  wait for port status response */
                IOLM_EXMPL_printf("IO-Link port %u: Port is waiting for port status request.\n", portNumber);
                port[portNumber].exampleState = IOLM_eExampleState_Config;
                break;
            case IOLM_eExampleState_Config:
            {
                IOLM_SMI_SPortConfigList portConfig;
                memset(&portConfig, 0, sizeof(portConfig));
                portConfig.u8PortMode = IOLM_SMI_ePortMode_IOL_AUTOSTART;
                portConfig.u16ArgBlockID = IOLM_SMI_ENDIAN_16(IOLM_SMI_eArgBlockID_PortConfigList);
                IOLM_EXMPL_printf("IO-Link port %u: Port is now in \"Config\" mode\n", portNumber);
                IOLM_SMI_vPortConfigurationReq(IOLM_SMI_CLIENT_APP, portNumber, sizeof(portConfig), (INT8U*)&portConfig);
                port[portNumber].exampleState = IOLM_eExampleState_ConfigWait;
            }
                break;
            case IOLM_eExampleState_ConfigWait: /* wait for read response */
            {
                 IOLM_EXMPL_printf("IO-Link port %u: Port is now in \"ConfigWait\" mode\n", portNumber);
            }
                 break;
             case IOLM_eExampleState_PortStatusRequest: /* start check port status */
             {
             IOLM_SMI_vPortStatusReq(IOLM_SMI_CLIENT_APP, portNumber);
             IOLM_EXMPL_printf("IO-Link port %u: Port is now in \"Port request\" mode\n", portNumber);
             port[portNumber].exampleState = IOLM_eExampleState_PortStatusWait;
             }
                break;
             case IOLM_eExampleState_PortStatusWait: /*  wait for port status response */
             {
                IOLM_EXMPL_printf("IO-Link port %u: Port is now in \"PortStatusWait\" mode\n", portNumber);
             }
                 break;
            case IOLM_eExampleState_ReadVendor:                                                                             /* start reading vendor name */
            {
                /* Allocate ArgBlock Memory */
                uint8_t aMem[IOLM_SMI_ARGBLOCK_ONREQ_LEN(0)];
                IOLM_SMI_SOnRequestData* pReq = (IOLM_SMI_SOnRequestData*)aMem;

                /* Fill Request */
                pReq->u16ArgBlockID = IOLM_SMI_ENDIAN_16(IOLM_SMI_eArgBlockID_OnRequestDataRead);
                pReq->u16Index = IOLM_SMI_ENDIAN_16(IOL_eISDUIndex_VendorName);                                             /* Index 16 = VendorName     */
                pReq->u8Subindex = 0;

                /* Send Request */
                IOLM_EXMPL_printf("IO-Link port %u: Read Vendor Name\n", portNumber);
                IOLM_SMI_vDeviceReadReq(IOLM_SMI_CLIENT_APP, portNumber, IOLM_SMI_ARGBLOCK_ONREQ_LEN(0), (INT8U*)aMem);     /* Data */
                port[portNumber].exampleState = IOLM_eExampleState_ReadVendorWait;
            }
                break;

            case IOLM_eExampleState_ReadVendorWait:                                                                         /* wait for read response */
                IOLM_EXMPL_printf("IO-Link port %u: Port is now in \"ReadVendorWait\" mode\n", portNumber);
                break;
            case IOLM_eExampleState_ReadProductName:                                                                        /* start reading ProductName */
            {
                /* Allocate ArgBlock Memory */
                uint8_t aMem[IOLM_SMI_ARGBLOCK_ONREQ_LEN(0)];
                IOLM_SMI_SOnRequestData* pReq = (IOLM_SMI_SOnRequestData*)aMem;

                /* Fill Request */
                pReq->u16ArgBlockID = IOLM_SMI_ENDIAN_16(IOLM_SMI_eArgBlockID_OnRequestDataRead);

                pReq->u16Index = IOLM_SMI_ENDIAN_16(IOL_eISDUIndex_ProductName);                                            /* Index 18 = ProductName     */
                pReq->u8Subindex = 0;

                /* Send Request */
                IOLM_EXMPL_printf("IO-Link port %u: Read Product Name\n", portNumber);
                IOLM_SMI_vDeviceReadReq(IOLM_SMI_CLIENT_APP, portNumber, IOLM_SMI_ARGBLOCK_ONREQ_LEN(0), (INT8U*)aMem);     /* Data */
                port[portNumber].exampleState = IOLM_eExampleState_ReadProductNameWait;
            }
                break;
            case IOLM_eExampleState_ReadProductNameWait:                                                                    /* wait for read response */
                IOLM_EXMPL_printf("IO-Link port %u: Port is now in \"ReadProductNameWait\" mode\n", portNumber);
                break;
            case IOLM_eExampleState_ReadSerialnumber:                                                                       /* start reading Serialnumber */
            {
                /* Allocate ArgBlock Memory */
                uint8_t aMem[IOLM_SMI_ARGBLOCK_ONREQ_LEN(0)];
                IOLM_SMI_SOnRequestData* pReq = (IOLM_SMI_SOnRequestData*)aMem;

                /* Fill Request */
                pReq->u16ArgBlockID = IOLM_SMI_ENDIAN_16(IOLM_SMI_eArgBlockID_OnRequestDataRead);
                pReq->u16Index = IOLM_SMI_ENDIAN_16(IOL_eISDUIndex_SerialNumber);                                           /* Index 21 = Serialnumber */
                pReq->u8Subindex = 0;

                /* Send Request */
                IOLM_EXMPL_printf("IO-Link port %u: Read Serialnumber\n", portNumber);
                IOLM_SMI_vDeviceReadReq(IOLM_SMI_CLIENT_APP, portNumber, IOLM_SMI_ARGBLOCK_ONREQ_LEN(0), (INT8U*)aMem);     /* Data */
                port[portNumber].exampleState = IOLM_eExampleState_ReadSerialnumberWait;

            }
                break;

            case IOLM_eExampleState_ReadSerialnumberWait:                                                                   /* wait for read response */
                IOLM_EXMPL_printf("IO-Link port %u: Port is now in \"ReadSerialnumberWait\" mode\n", portNumber);
                break;
            /*
            SPECIFIC DEVICE PROCESS DATA WRITE CALL:
            For different devices there a different process data write commands. For example: one will turn on a LED, an other will change 
            a specific temperature measurement mode and so on, so this following comments are just for inspiration, 
            you need to check your device manual first before write process data.
            */
            /*
            case IOLM_eExampleState_WriteProcessDataValue:
            {
                //Allocate ArgBlock Memory 
                INT8U aMem[IOLM_SMI_ARGBLOCK_ONREQ_LEN(2)];                                                               
                IOLM_SMI_SPDOut* pReq = (IOLM_SMI_SPDOut*)aMem;

                //Fill Request
                pReq->u16ArgBlockID = IOLM_SMI_ENDIAN_16(IOLM_SMI_eArgBlockID_PDOut);
                pReq->u8OE = 0x01;
                pReq->u8OutputDataLength = 0x01;                                                                            
                pReq->au8Data[0]= 0x02;

                //Send PD Request
                IOLM_SMI_vPDOutReq(IOLM_SMI_CLIENT_APP, portNumber, IOLM_SMI_ARGBLOCK_ONREQ_LEN(2), (INT8U*)aMem);
                IOLM_EXMPL_printf("IO-Link port %u: Port is now in \"Write Process Data\" mode\n", portNumber);
                IOLM_EXMPL_printf("IO-Link port %u: write 0x02\n", portNumber);
                port[portNumber].exampleState = IOLM_eExampleState_WriteProcessDataValueWait;
            }
                break;

            case IOLM_eExampleState_WriteProcessDataValueWait:
            {
                IOLM_EXMPL_printf("IO-Link port %u: Port is now in \"Wait Write Process Data\" mode\n", portNumber);
            }
                break;
            */
            case IOLM_eExampleState_PortStatusRequestPD:                                                                      /* start check port status */
                IOLM_SMI_vPortStatusReq(IOLM_SMI_CLIENT_APP, portNumber);
                IOLM_EXMPL_printf("IO-Link port %u: Port is now in \"Process Data Exchange \" mode\n", portNumber);
                port[portNumber].exampleState = IOLM_eExampleState_PortStatusWaitPD;
                break;
            case IOLM_eExampleState_PortStatusWaitPD:                                                                         /*  wait for port status response */
                IOLM_EXMPL_printf("IO-Link port %u: Port is now in \"Process Data Exchange wait \" mode\n", portNumber);
                break;
            case IOLM_eExampleState_ReadProcessDataValue: /* start reading process data */
            {
                IOLM_SMI_vPDInReq(IOLM_SMI_CLIENT_APP, portNumber);
                IOLM_EXMPL_printf("IO-Link port %u: Port is now in \"Port Read Process Data\" mode\n", portNumber);
                port[portNumber].exampleState = IOLM_eExampleState_ReadProcessDataValueWait;
            }
                break;
            case IOLM_eExampleState_ReadProcessDataValueWait:
            {
                IOLM_EXMPL_printf("IO-Link port %u: Port is now in \"Wait Read Process Data\" mode\n", portNumber);
            }
                break;
            default:
                break;
        }
    }
}

/*!
 * \brief
 * Get Master identification confirmation callback.  
 *
 * This function is called if a Master identification read request has finished.
 *
 * \param[in]  clientID_p              Client ID.
 * \param[in]  error_p                 Error message as #IOL_EErrorType.
 * \param[in]  argBlockLength_p        Length of ArgBlock.
 * \param[in]  pArgBlock_p             Data pointer which points to the master identification (#IOLM_SMI_SMasterident).
 *
 * \return void
 *
 */
void IOLM_EXMPL_cbMasterIdentificationCnf(uint8_t clientID_p, uint16_t error_p, uint16_t argBlockLength_p, uint8_t* pArgBlock_p)
{
    IOLM_SMI_SMasterident* psuMasterIdent = (IOLM_SMI_SMasterident*)pArgBlock_p;
    psuMasterIdent->u16ArgBlockID = IOLM_SMI_ENDIAN_16(IOLM_SMI_eArgBlockID_MasterIdent);

    (void)clientID_p;
    (void)argBlockLength_p;

    if (error_p != IOL_eErrorType_NONE)
    {
        IOLM_EXMPL_printf("NO Master Identification Possible.\n");
    }
    else
    {
        IOLM_EXMPL_printf("Master Identification: \nMasterID:  %u \nMax ports: %u \n ",
            IOLM_SMI_ENDIAN_32(psuMasterIdent->u32MasterID),
            psuMasterIdent->u8MaxNumberOfPorts);
    }
}

/*!
 * \brief
 * Port event indication
 *
 * This function is called if a port event indication occurs. 
 *
 * \param[in]  port_p                  Port ID.
 * \param[in]  argBlockLength_p        Length of ArgBlock.
 * \param[in]  pArgBlock_p             Data pointer which points to the port event data (#IOLM_SMI_SPortEvent).
 *
 * \return void
 *
 */
void IOLM_EXMPL_PortEventInd(uint8_t port_p, uint16_t argBlockLength_p, uint8_t* pArgBlock_p)
{
    IOLM_SMI_SPortEvent* pEvent = (IOLM_SMI_SPortEvent*)pArgBlock_p;

    (void)port_p;
    (void)argBlockLength_p;
    (void)pEvent;

    /* Example Output*/
    IOLM_EXMPL_printf("IO-Link port %u: Port event - qualifier: 0x%02x - code: 0x%04x\n",
        port_p, pEvent->u8EventQualifier,
        pEvent->u16EventCode); 
}

/*!
 * \brief
 * Device event indication
 *
 * This function is called if a device event indication occurs.
 *
 * \param[in]  port_p                  Port ID.
 * \param[in]  argBlockLength_p        Length of ArgBlock.
 * \param[in]  pArgBlock_p             Data pointer which points to the device event data (#IOLM_SMI_SDeviceEvent).
 *
 * \return void
 *
 */
void IOLM_EXMPL_DeviceEventInd(uint8_t port_p, uint16_t argBlockLength_p, uint8_t* pArgBlock_p)
{
    IOLM_SMI_SDeviceEvent* pEvent = (IOLM_SMI_SDeviceEvent*)pArgBlock_p;

    (void)port_p;
    (void)argBlockLength_p;
    (void)pEvent;

    /* Example Output */
    IOLM_EXMPL_printf("IO-Link port %u: Device event - qualifier: 0x%02x - code: 0x%04x\n",
        port_p,
        pEvent->u8EventQualifier,
        pEvent->u16EventCode);
}

/*!
* \brief
* Port configuration confirmation callback.
*
* This function is called if a device configuration request has finished.
*
* \param[in] clientID_p Client ID.
* \param[in] port_p Port ID.
* \param[in] error_p Error message as #IOL_EErrorType.
*
* \return void
*
*/
void IOLM_EXMPL_cbPortConfigurationCnf(uint8_t clientID_p, uint8_t port_p, uint16_t error_p)
{
     if (error_p != IOL_eErrorType_NONE)
     {
         IOLM_EXMPL_printf("IO-Link port %u: Port is now in \"IOL_eErrorType_NONE\" mode\n", port_p);
         port[port_p].exampleState = IOLM_EXMPL_PortErrorHandler(port_p, error_p);
     }
     else
     {
         port[port_p].exampleState = IOLM_eExampleState_ReadVendor;
     }
}

/*!
 * \brief
 * Port status confirmation callback.
 *
 * This function is called if a device port status request has finished.
 *
 * \param[in]  clientID_p              Client ID.
 * \param[in]  port_p                  Port ID.
 * \param[in]  error_p                 Error message as #IOL_EErrorType.
 * \param[in]  argBlockLength_p        Length of ArgBlock.
 * \param[in]  pArgBlock_p             Data pointer which points to the port status list (#IOLM_SMI_SPortStatusList).
 *
 * \return void
 *
 */
void IOLM_EXMPL_cbPortStatusCnf(uint8_t clientID_p, uint8_t port_p, uint16_t error_p,uint16_t argBlockLength_p, uint8_t* pArgBlock_p)
{
    IOLM_SMI_SPortStatusList* psuPortStatus = (IOLM_SMI_SPortStatusList*)pArgBlock_p;

    (void)argBlockLength_p;
    
    port[port_p].currentStackPortStatus = psuPortStatus->u8PortStatusInfo;

    IOLM_EXMPL_updateLEDs(port_p);

#if (IOLM_EXMPL_ENABLE_STATE_MACHINE == 1)
    if (error_p != IOL_eErrorType_NONE) 
    {
        IOLM_EXMPL_PortErrorHandler(port_p, error_p);
    }
    else if (port[port_p].currentStackPortStatus == IOLM_SMI_ePortStatus_OPERATE)
    {
        if (port[port_p].exampleState == IOLM_eExampleState_PortStatusWait)
        {
            port[port_p].exampleState = IOLM_eExampleState_ReadVendor;
        }
        else if (port[port_p].exampleState == IOLM_eExampleState_PortStatusRequestPD
            || port[port_p].exampleState == IOLM_eExampleState_PortStatusWaitPD)
        {
            port[port_p].exampleState = IOLM_eExampleState_ReadProcessDataValue;
        }
    }
#endif
}

/*!
 * \brief
 * Device write confirmation callback.
 *
 * This function is called if a device write request has finished.
 *
 * \param[in]  clientID_p              Client ID.
 * \param[in]  port_p                  Port ID.
 * \param[in]  error_p                 Error message as #IOL_EErrorType.
 *
 * \return void
 *
 */
void IOLM_SMI_cbDeviceWriteCnf(uint8_t clientID_p, uint8_t port_p, uint16_t error_p)
{
    IOLM_EXMPL_printf("---------------Write Confirmation------------------\n");
    if (error_p != IOL_eErrorType_NONE)
    {
        port[port_p].exampleState = IOLM_EXMPL_PortErrorHandler(port_p, error_p);
    }
    else
    {
        port[port_p].exampleState = IOLM_eExampleState_PortStatusRequestPD;
    }
}

/*!
 * \brief
 * Device read confirmation callback.
 *
 * This function is called if a device read request has finished. 
 * This is the answer to an on-request data. 
 *
 * \param[in]  clientID_p              Client ID.
 * \param[in]  port_p                  Port ID.
 * \param[in]  error_p                 Error message as #IOL_EErrorType.
 * \param[in]  argBlockLength_p        Length of ArgBlock.
 * \param[in]  pArgBlock_p             Data pointer which points to the device on-request data (#IOLM_SMI_SOnRequestData).
 *
 * \return void
 *
 */
void IOLM_EXMPL_cbDeviceReadCnf(uint8_t clientID_p, uint8_t port_p, uint16_t error_p, uint16_t argBlockLength_p, uint8_t* pArgBlock_p)
{
#if (IOLM_EXMPL_ENABLE_PRINTF == 1)
    IOLM_SMI_SOnRequestData* pReq = (IOLM_SMI_SOnRequestData*)pArgBlock_p;
    uint8_t payloadLength = argBlockLength_p - IOLM_SMI_ARGBLOCK_ONREQ_LEN(0);
    uint8_t aDeviceData[DEVICE_READ_DATA_LENGTH] = {""};

    OSAL_MEMORY_memcpy(aDeviceData, pReq->au8Data, payloadLength);
    aDeviceData[payloadLength] = 0;

#endif
    (void)clientID_p;
        
    if (error_p != IOL_eErrorType_NONE)
    {
        port[port_p].exampleState = IOLM_EXMPL_PortErrorHandler(port_p, error_p);
    }
    else if (port[port_p].exampleState == IOLM_eExampleState_ReadVendorWait)
    {
        IOLM_EXMPL_printf("IO-Link port %u: Device vendor is \"%s\"\n", port_p, aDeviceData);
        port[port_p].exampleState = IOLM_eExampleState_ReadProductName;
    }
    else if (port[port_p].exampleState == IOLM_eExampleState_ReadProductNameWait)
    {
        IOLM_EXMPL_printf("IO-Link port %u: Device vendor is \"%s\"\n", port_p, aDeviceData);
        port[port_p].exampleState = IOLM_eExampleState_ReadSerialnumber;
    }
    else if (port[port_p].exampleState == IOLM_eExampleState_ReadSerialnumberWait)
    {
        IOLM_EXMPL_printf("IO-Link port %u: Serialnumber: \"%s\"\n", port_p, aDeviceData);
        port[port_p].exampleState = IOLM_eExampleState_PortStatusRequestPD;
    }
}

/*!
 * \brief
 *  Set output data confirmation callback.
 *
 * This function is called if a process data write request from the master to the device has finished.
 *
 * \param[in]  clientID_p              Client ID.
 * \param[in]  port_p                  Port ID.
 * \param[in]  error_p                 Error message as #IOL_EErrorType.
 *
 * \return void
 *
 */
void IOLM_EXMPL_cbPDOutCnf(uint8_t clientID_p, uint8_t port_p, uint16_t error_p)
{
    (void)clientID_p;

    if (error_p != IOL_eErrorType_NONE)
    {
        port[port_p].exampleState = IOLM_EXMPL_PortErrorHandler(port_p, error_p);
    }
    else
    {
        IOLM_EXMPL_printf("---------------Write Process Data Success-----------------\n");
        port[port_p].exampleState = IOLM_eExampleState_ReadProcessDataValue;
    }
}

/*!
 * \brief
 * Get input data request and confirmation.
 *
 * This function is called if a process data read request from the master to the device has finished.
 *
 * \param[in]  clientID_p              Client ID.
 * \param[in]  port_p                  Port ID.
 * \param[in]  error_p                 Error message as #IOL_EErrorType.
 * \param[in]  argBlockLength_p        Length of ArgBlock.
 * \param[in]  pArgBlock_p             Data pointer which points to the PDIn data (#IOLM_SMI_SPDIn).
 *
 * \return void
 *
 */
void IOLM_EXMPL_cbPDInCnf(uint8_t clientID_p, uint8_t port_p, uint16_t error_p, uint16_t argBlockLength_p, uint8_t* pArgBlock_p)
{
    IOLM_SMI_SPDIn* psuPDIn = (IOLM_SMI_SPDIn*)pArgBlock_p;
    uint8_t au8CurrentData[PD_INPUT_LENGTH] = { 0, };
    uint8_t memoryCompareResult = 0;
    uint8_t dataElement;

    (void)clientID_p;
    (void)argBlockLength_p;

    if (error_p != IOL_eErrorType_NONE)
    {
        port[port_p].exampleState = IOLM_EXMPL_PortErrorHandler(port_p, error_p);
    }
    else
    {
        IOLM_EXMPL_printf("---------------Read Process Data Success-------------------\n");
        OSAL_MEMORY_memcpy(au8CurrentData, psuPDIn->au8Data, psuPDIn->u8InputDataLength);
        memoryCompareResult = OSAL_MEMORY_memcmp(au8CurrentData, port[port_p].aPDInCnfData, sizeof(au8CurrentData));
        if (memoryCompareResult != 0)
        {
            OSAL_MEMORY_memcpy(port[port_p].aPDInCnfData, au8CurrentData, sizeof(au8CurrentData));
            for (dataElement = 0; dataElement <= psuPDIn->u8InputDataLength; dataElement++)
            {
                if (au8CurrentData[dataElement] != 0)
                {
                    IOLM_EXMPL_printf("IO-Link port %u: Process data: 0x%02x \n", port_p, au8CurrentData[dataElement]);
                }
            }
        }
        port[port_p].exampleState = IOLM_eExampleState_PortStatusRequestPD;
    }
}

/*!
 * \brief
 * Get the versioning from IOL8M-Sitara-Example-Project.
 *
 * This function is used to get information about the version of the master.
 *
 * \param[in]  u8Instance_p              Client ID.
 * \param[in]  pu8Data_p                 Data(unused).
 * \param[in]  u16Length_p               Data length(unused).
 *
 * \return     void
 */
void IOLM_EXMPL_cbChipInfo(INT8U u8Instance_p, INT8U *pu8Data_p, INT16U u16Length_p)
{
    if (u8Instance_p == 0x00)
    {
        IOLM_SMI_SStackInformation suInfo;
        suInfo.u16ArgBlockID = IOLM_SMI_ENDIAN_16(IOLM_SMI_eArgBlockID_StackInformation);
        suInfo.u8Version0 = IOL8M_SITARA_VERSION_VERSION_MAJOR;
        suInfo.u8Version1 = IOL8M_SITARA_VERSION_VERSION_MINOR;
        suInfo.u8Version2 = IOL8M_SITARA_VERSION_VERSION_PATCH;
        suInfo.u8Version3 = 0;
        suInfo.u16TagVersion = IOL8M_SITARA_VERSION_COMMIT_U16;
        suInfo.u16HWRev = 0;
        suInfo.u32Reserved = 0;
        IOLM_SMI_vExtRsp(IOL_eErrorType_NONE, (INT8U *)&suInfo, sizeof(suInfo));
    }
    else
    {
        IOLM_SMI_vExtRsp(IOL_eErrorType_IDX_NOTAVAIL, NULL, 0);
    }
}

/*!
 * \brief
 * Call port error handler.
 *
 * This function is called from the callback, if an error occurs.
 *
 * \param[in]  port_p                  Port ID.
 * \param[in]  error_p                 Error message as #IOL_EErrorType.
 *
 * \return  portState_g[port_p] as uint8_t.
 *
 */
IOLM_EXMPL_EExampleState_t IOLM_EXMPL_PortErrorHandler(uint8_t port_p, uint16_t error_p)
{
    IOLM_EXMPL_printf("IO-Link port %u: Read failed with error 0x%04x", port_p, error_p);
    switch (error_p)
    {
    case IOL_eErrorType_ARGBLOCK_NOT_SUPPORTED:
        IOLM_EXMPL_printf(" - \"ARGBLOCK_NOT_SUPPORTED\"");
        if (port[port_p].exampleState == IOLM_eExampleState_PortStatusWait)
        {
            port[port_p].exampleState = IOLM_eExampleState_ReadVendor;
        }
        else if (port[port_p].exampleState == IOLM_eExampleState_PortStatusRequestPD
            || port[port_p].exampleState == IOLM_eExampleState_PortStatusWaitPD)
        {
            port[port_p].exampleState = IOLM_eExampleState_ReadProcessDataValue;
        }
        else
        {
            port[port_p].exampleState = IOLM_eExampleState_Init;
        }
        break;
    case IOL_eErrorType_ARGBLOCK_ID_NOT_SUPPORTED:
        IOLM_EXMPL_printf(" - \"ARGBLOCK_ID_NOT_SUPPORTED\"");
        if (port[port_p].exampleState == IOLM_eExampleState_PortStatusWait)
        {
            port[port_p].exampleState = IOLM_eExampleState_ReadVendor;
        }
        else if (port[port_p].exampleState == IOLM_eExampleState_PortStatusRequestPD
            || port[port_p].exampleState == IOLM_eExampleState_PortStatusWaitPD)
        {
            port[port_p].exampleState = IOLM_eExampleState_ReadProcessDataValue;
        }
        else
        {
            port[port_p].exampleState = IOLM_eExampleState_Init;
        }
        break;
    case IOL_eErrorType_SERVICE_NOT_SUPPORTED:
        IOLM_EXMPL_printf(" - \"SERVICE_NOT_SUPPORTED\"");
        if (port[port_p].exampleState == IOLM_eExampleState_PortStatusWait)
        {
            port[port_p].exampleState = IOLM_eExampleState_ReadVendor;
        }
        else if (port[port_p].exampleState == IOLM_eExampleState_PortStatusRequestPD
            || port[port_p].exampleState == IOLM_eExampleState_PortStatusWaitPD)
        {
            port[port_p].exampleState = IOLM_eExampleState_ReadProcessDataValue;
        }
        break;
    case IOL_eErrorType_DEVICE_NOT_ACCESSIBLE:
        IOLM_EXMPL_printf(" - \"DEVICE_NOT_ACCESSIBLE\"");
        port[port_p].exampleState = IOLM_eExampleState_PortStatusWait;
        break;
    case IOL_eErrorType_PORT_NUM_INVALID:
        IOLM_EXMPL_printf(" - \"PORT_NUM_INVALID\"");
        port[port_p].exampleState = IOLM_eExampleState_PortStatusWait;
        break;
    case IOL_eErrorType_SERVICE_TEMP_UNAVAILABLE:
        IOLM_EXMPL_printf(" - \"SERVICE_TEMP_UNAVAILABLE\"");
        port[port_p].exampleState = IOLM_eExampleState_PortStatusWait;
        break;
    case IOL_eErrorType_PORT_CONFIG_INCONSISTENT:
        IOLM_EXMPL_printf(" - \"PORT_CONFIG_INCONSISTENT\"");
        port[port_p].exampleState = IOLM_eExampleState_Init;
        break;
    default:
        port[port_p].exampleState = IOLM_eExampleState_Init;
        break;
    }
    IOLM_EXMPL_printf("\n");
    return port[port_p].exampleState;
}
