/*
 *  Copyright (c) 2021, KUNBUS GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
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

#ifndef INC_PROT__IOLM_PORT_FUNCTIONS_H__
#define INC_PROT__IOLM_PORT_FUNCTIONS_H__

#include "IOLM_Types.h"
#include "IOL_Port_Functions.h"

#ifdef __cplusplus
extern "C" {
#endif



#ifndef IOLM_WIRELESS
/**
\brief Set physical layer mode.

The PL-SetMode service is used to setup the electrical characteristics
and configurations of the physical layer.

\param[in] u8Port_p         The port number of the affected port.
\param[in] eTargetMode_p    Indicates the requested operational mode of the port.

\pre The port number must be smaller than #IOLM_PORT_COUNT.
\post None.

\par IRQ-safe
<b>Yes</b> Function can be called in interrupt context.

\ingroup grp_api_port

*/
IOL_FUNC_DECL void IOLM_Port_PL_vSetMode(INT8U u8Port_p, IOL_ETargetMode eTargetMode_p);
#endif

#ifndef IOLM_WIRELESS
/**
\brief Send wake-up pulse.

The PL-WakeUp service initiates a specific sequence which
prepares the physical layer to send and receive communication requests
(see 5.3.3.3). The function returns the wake-up mode of the Phy.

If #IOLM_eWakeupMode_Done is returned, the wake-up pulse will be generated
without further activity of the stack.

#IOLM_eWakeupMode_Phy indicates a phy with frame handler. The stack needs
only to trigger retries if communication can't be established or is lost.

#IOLM_eWakeupMode_Stack is used, if the stack should generate the
wake-up pulse using the timer and SIO interface.

#IOLM_eWakeupMode_Ack is used when wake-up is done by application and
later acknowledged by #IOLM_API_PL_u8WakeAck.

\param[in] u8Port_p The port number of the affected port.

\return #IOLM_EWakeupMode Wake-up mode.

\pre The port number must be smaller than #IOLM_PORT_COUNT.
\post None.

\par IRQ-safe
<b>No</b> Function will be called from Mainloop context.

\ingroup grp_api_port

*/
IOL_FUNC_DECL IOLM_EWakeupMode IOLM_Port_PL_eWakeUp(INT8U u8Port_p);
#endif

#ifndef IOLM_WIRELESS
/**
\brief Prepare data exchange.

The PL-transfer service is used to exchange the SDCI data between data
link layer and physical layer. The prepare function setups Tx/Rx (maybe with DMA).
The transfer start follows later.

\param[in] u8Port_p             The port number of the affected port.
\param[in] pu8Data_p            Contains the data value which is transferred
                                over the SDCI interface.
\param[in] u8DataLength_p       The size of the data in byte.
\param[in] pu8ResponseData_p    Memory for the response data.
\param[in] u8ResponseLength_p   The size of the expected response data in byte.
\pre
- The port number must be smaller than #IOLM_PORT_COUNT.
- The given pointer(s) must be valid.
\post None.

\par IRQ-safe
    If #IOLM_IRQ_PROCESSING is set, the functions
    is called from IRQ context. If not, the function is called from Mainloop.

\ingroup grp_api_port

*/
IOL_FUNC_DECL void IOLM_Port_PL_vTransferPrepare(INT8U u8Port_p,
    const INT8U* pu8Data_p, INT8U u8DataLength_p, 
    INT8U* pu8ResponseData_p, INT8U u8ResponseLength_p
    );
#endif

#ifndef IOLM_WIRELESS
/**
\brief Start frame transmission.

The PL-transfer service triggers the transmission of the prepared frame.
If all expected bytes are received or an error happens, the function
#IOLM_API_PL_vTransferInd has to be called.

\param[in] u8Port_p The port number of the affected port.

\pre The port number must be smaller than #IOLM_PORT_COUNT.
\post None.

\par IRQ-safe
<b>Yes</b> Function can be called in interrupt context.

\ingroup grp_api_port

*/
IOL_FUNC_DECL void IOLM_Port_PL_vTransferTrig(INT8U u8Port_p);
#endif

#ifndef IOLM_WIRELESS
/**
\brief Set output pin value.

The PL set DO function is used if the port is in DO Mode.
It sets the output pin to the corresponding value.

\param[in] u8Port_p         The port number of the affected port.
\param[in] boOutValue_p     Value of the output pin.

\pre
- The port number must be smaller than #IOLM_PORT_COUNT.
\post None.

\par IRQ-safe
<b>Yes</b> Function can be called in interrupt context.

\ingroup grp_api_port

*/
IOL_FUNC_DECL void IOLM_Port_PL_vSetDO(INT8U u8Port_p, TBOOL boOutValue_p);
#endif

#ifndef IOLM_WIRELESS
/**
\brief Get input pin value.

The PL get DI function is used if the port is in DI mode.
It reads the input value from the corresponding port.

\param[in] u8Port_p The port number of the affected port.

\return Value of the corresponding pin
\pre
- The port number must be smaller than #IOLM_PORT_COUNT.
\post None.

\par IRQ-safe
<b>Yes</b> Function can be called in interrupt context.

\ingroup grp_api_port

*/

IOL_FUNC_DECL TBOOL IOLM_Port_PL_boGetDI(INT8U u8Port_p);
#endif

#ifndef IOLM_WIRELESS
/**
\brief Set IQ output pin value.

The PL set IQ function sets the IQ output pin (e.g. M12 Pin 2)
to the corresponding value.

\param[in] u8Port_p         The port number of the affected port.
\param[in] boOutValue_p     Value of the output pin.

\pre
- The port number must be smaller than #IOLM_PORT_COUNT.
\post None.

\par IRQ-safe
<b>Yes</b> Function can be called in interrupt context.

\ingroup grp_api_port

*/
IOL_FUNC_DECL void IOLM_Port_PL_vSetIQ(INT8U u8Port_p, TBOOL boOutValue_p);
#endif

#ifndef IOLM_WIRELESS
/**
\brief Get IQ input pin value.

The PL get IQ function reads the input value from
IQ Pin (e.g. M12 Pin 2) of the corresponding port.

\param[in] u8Port_p The port number of the affected port.

\return Value of the corresponding pin
\pre
- The port number must be smaller than #IOLM_PORT_COUNT.
\post None.

\par IRQ-safe
<b>Yes</b> Function can be called in interrupt context.

\ingroup grp_api_port

*/

IOL_FUNC_DECL TBOOL IOLM_Port_PL_boGetIQ(INT8U u8Port_p);
#endif

#ifndef IOLM_WIRELESS
/**
\brief Set IQ mode.

The PL set IQ mode function sets the mode for IQ pin (e.g. M12 Pin 2).

\param[in] u8Port_p         The port number of the affected port.
\param[in] eIQMode_p        Value of IQ mode.

\pre
- The port number must be smaller than #IOLM_PORT_COUNT.
\post None.

\par IRQ-safe
<b>Yes</b> Function can be called in interrupt context.

\ingroup grp_api_port

*/
IOL_FUNC_DECL void IOLM_Port_PL_vSetIQMode(INT8U u8Port_p, IOL_EIQMode eIQMode_p);
#endif

#if (IOLM_PHY_FRAMEHANDLER == 1) || defined(IOL_DOCU)
/**
\brief Frame handler setup.

This function indicates a frame handler configuration change. The information
can be used to configure a external Phy with this setup.

\param[in] u8Port_p     The port number of the affected port.
\param[in] u8PdOut_p    Number of output bytes.
\param[in] u8PdIn_p     Number of input bytes.
\param[in] u8ODLen_p    OD length.
\param[in] u8IOLCycle_p Cycle time in IO-Link notation.

\return #IOL_EBaudrate current baud rate
\pre
- The port number must be smaller than #IOLM_PORT_COUNT.
\post None.

\par IRQ-safe
<b>Yes</b> Function can be called in interrupt context.

\ingroup grp_api_port

*/
IOL_FUNC_DECL IOL_EBaudrate IOLM_Port_PL_eSetFHCfg(INT8U u8Port_p, INT8U u8PdOut_p, INT8U u8PdIn_p,
                INT8U u8ODLen_p, INT8U u8IOLCycle_p);
#endif

#if (defined(IOLM_PHY_CYCLETIMER) && (IOLM_PHY_CYCLETIMER == 1)) && (!defined(IOLM_PHY_FRAMEHANDLER) || (IOLM_PHY_FRAMEHANDLER == 0))
/**
\brief Cycle timer setup.

This function indicates a cycle timer configuration change. The information
can be used to configure an external Phy cycle timer.

\param[in] u8Port_p     The port number of the affected port.
\param[in] u32TimeUs_p  Cycle time in microseconds.

\pre
- The port number must be smaller than #IOLM_PORT_COUNT.
\post None.

\par IRQ-safe
<b>Yes</b> Function can be called in interrupt context.

\ingroup grp_api_port

*/
IOL_FUNC_DECL void IOLM_Port_PL_vSetPhyCycleTimer(INT8U u8Port_p, INT32U u32TimeUs_p);
#endif


#ifndef IOLM_WIRELESS
/**
\brief Start port hardware timer.

The start timer function is used to start a hardware timer for
the specified port. After expiration the #IOLM_API_vTimerExpired
callback has to be called. If there is no phy cycle timer present this handles
all hardware timers.

\param[in] u8Port_p     The port number of the affected port.
\param[in] u32TimeUs_p  Timeout in microseconds.

\pre
- The port number must be smaller than #IOLM_PORT_COUNT.
\post None.

\par IRQ-safe
<b>Yes</b> Function can be called in interrupt context.

\ingroup grp_api_port

*/
IOL_FUNC_DECL void IOLM_Port_vStartTimer(INT8U u8Port_p, INT32U u32TimeUs_p);
#endif


/**
\brief Get SysTick value.

This function is used to read the SysTick counter value.
The tick rate of this counter should be #IOLM_SYSTICK_INTERVAL_MS
milliseconds.

\return Tick value

\pre The port number must be smaller than #IOLM_PORT_COUNT.
\post None.

\par IRQ-safe
<b>No</b> Function will be called from Mainloop context.

\ingroup grp_api_port

*/
IOL_FUNC_DECL INT32S IOLM_Port_s32SysTickVal(void);



/**
\brief Enter critical section

This function is called before call of critical sections and need to disable interrupts

\ingroup grp_api_port

*/
IOL_FUNC_DECL INT32U IOLM_Port_vCriticalStart(void);


/**
\brief Enter critical section

This function is called before call of critical sections and need to disable interrupts

\ingroup grp_api_port

*/
IOL_FUNC_DECL void IOLM_Port_vCriticalEnd(INT32U u32Level_p);


#ifndef IOLM_Port_pu8MemAlloc
/**
\brief Allocate memory callback.

This callback is called by the stack to allocate memory resources.

\param[in]  u32Length_p     Length of ArgBlock.

\return New pointer to allocated memory.

\ingroup grp_api_port

*/
IOL_FUNC_DECL INT8U *IOLM_Port_pu8MemAlloc(INT32U u32Length_p);
#endif

#ifndef IOLM_Port_vMemFree
/**
\brief Free memory callback.

This callback is called by the stack to free the resources for an ArgBlock.
Since this is hardware specific, it has to be implemented in the application code.

\param[in]  pu8Mem_p   Data pointer to the memory location.

\ingroup grp_api_port

*/
IOL_FUNC_DECL void IOLM_Port_vMemFree(INT8U *pu8Mem_p);
#endif

#ifdef IOLM_SMI_MONITOR
/**
\brief data logging function

Sends a log message for physical layer data logging

\param[in]  u8Instance_p   port/track number
\param[in]  u8Channel_p    type of data log
\param[in]  u16Timestamp_p 16 bit timestamp
\param[in]  pu8Data_p      log data
\param[in]  u8Length_p     log data length

\ingroup grp_api_port

*/
void IOLM_Port_DL_vLog(INT8U u8Instance_p, INT8U u8Channel_p, INT16U u16Timestamp_p,
    INT8U* pu8Data_p, INT8U u8Length_p);
#endif 


/**
\brief Event indication.

This function is called if events for a specific port are available. 
If the events are handled, the function #IOLM_API_AL_vEventRsp has to be called.
There is no further #IOL_eESource_DEVICE event indicated until the last is confirmed.
Events which are only needed for internal handling (DataStorage, ISDU, etc) are handled
by the stack and are not indicated.

\param[in] u8Port_p         The port number of the affected port.
\param[in] eEInstance_p     Event instance.
\param[in] eEType_p         Event type.
\param[in] eOrigin_p        Event source.
\param[in] eEMode_p         Event mode.
\param[in] u16ECode_p       EventCode.

\par IRQ-safe
<b>No</b> Function will be called from Mainloop context.

\ingroup grp_api_evt

*/
IOL_FUNC_DECL void IOLM_Port_AL_vEventInd(INT8U u8Port_p, IOL_EEInstance eEInstance_p,
    IOL_EEType eEType_p, IOL_EESource eOrigin_p, IOL_EEMode eEMode_p, INT16U u16ECode_p);



/**
\brief Read confirmation.

This function is called if a read request is handled. If the error info
is #IOL_eErrorType_NONE, the data pointer is valid.

\param[in] u8Port_p         The port number of the affected port.
\param[in] u16ErrorInfo_p   Error information (see #IOL_EErrorType).
\param[in] pu8Data_p        Response data.
\param[in] u8Length_p       Response data Length.

\pre The port number must be smaller than #IOLM_PORT_COUNT.
\post None.

\par IRQ-safe
<b>No</b> Function will be called from Mainloop context.

\ingroup grp_api_isdu

*/
IOL_FUNC_DECL void IOLM_Port_AL_vReadCnf(INT8U u8Port_p, INT16U u16ErrorInfo_p, const INT8U *pu8Data_p, INT8U u8Length_p);



/**
\brief Write confirmation.

This function is called if a write request is handled. If the error info
is #IOL_eErrorType_NONE, the write operation was successful.

\param[in] u8Port_p         The port number of the affected port.
\param[in] u16ErrorInfo_p   Error information (see #IOL_EErrorType).

\pre The port number must be smaller than #IOLM_PORT_COUNT.
\post None.

\par IRQ-safe
<b>No</b> Function will be called from Mainloop context.

\ingroup grp_api_isdu

*/
IOL_FUNC_DECL void IOLM_Port_AL_vWriteCnf(INT8U u8Port_p, INT16U u16ErrorInfo_p);


/**
\brief New input indication.

This function is called if new input data are available. They can be
read by call of #IOLM_API_AL_eGetInput().

\param[in] u8Port_p The port number of the affected port.

\pre The port number must be smaller than #IOLM_PORT_COUNT.
\post None.

\par IRQ-safe
<b>No</b> Function will be called from Mainloop context.

\ingroup grp_api_pd

*/
IOL_FUNC_DECL void IOLM_Port_AL_vNewInput(INT8U u8Port_p);

/**
\brief Process Data cycle.

This function is called if the Process Data cycle is finished.
The application can provide new Process Data using #IOLM_API_AL_eSetOutput().

\param[in] u8Port_p The port number of the affected port.

\pre The port number must be smaller than #IOLM_PORT_COUNT.
\post None.


\par IRQ-safe
<b>No</b> Function will be called from Mainloop context.

\ingroup grp_api_pd

*/
IOL_FUNC_DECL void IOLM_Port_AL_vPDCycle(INT8U u8Port_p);

/**
\brief Control indication.

This function is called if the Process Data status has changed.
The control code indicates the new Process Data state.

\param[in] u8Port_p         The port number of the affected port.
\param[in] eControlCode_p   Input data status.


\par IRQ-safe
<b>Yes</b> Function can be called in interrupt context.

\ingroup grp_api_pd

*/
IOL_FUNC_DECL void IOLM_Port_AL_vControlInd(INT8U u8Port_p, IOL_EPDInStatus eControlCode_p);



/**
\brief  Port mode indication.

This function is called if the State for this port has changed. 

If a Port is in State #IOL_ePortMode_READY_TO_OPERATE, the Device application
can change the state of all ready ports to operate by call of
#IOLM_API_SM_eOperate().

\param[in] u8Port_p     The port number of the affected port.
\param[in] ePortMode_p  New port State.

\pre The port number must be smaller than #IOLM_PORT_COUNT.
\post None.

\par IRQ-safe
<b>No</b> Function will be called from Mainloop context.

\ingroup grp_api_sm

*/
IOL_FUNC_DECL void IOLM_Port_SM_vPortMode(INT8U u8Port_p, IOL_EPortMode ePortMode_p);



/**
\brief Store DS content.

This function save the Data Storage content of a specific port in a
non volatile memory. This function should not block for longer time (cyclic
communication must be handled). After completion #IOLM_API_DS_vStoreComplete
has to be called.

\param[in] u8Port_p     The port number of the affected port.
\param[in] pu8Data_p    Data to be stored.
\param[in] u16Length_p  Length of data.

\pre The port number must be smaller than #IOLM_PORT_COUNT.
\post None.

\par IRQ-safe
<b>No</b> Function will be called from Mainloop context.

\ingroup grp_api_port

*/
IOL_FUNC_DECL void IOLM_Port_DS_vStoreData(INT8U u8Port_p, INT8U *pu8Data_p, INT16U u16Length_p);

/**
\brief Load DS content.

This function loads the Data Storage content of a specific port from a
non volatile memory. This is a blocking function and only called during initialization.
The return value is the content length. This should only be >0 if the data is
valid.

\param[in] u8Port_p     The port number of the affected port.
\param[in] pu8Data_p    Data area to load.
\param[in] u16Length_p  Length of data pointer.

\return valid data length loaded

\pre The port number must be smaller than #IOLM_PORT_COUNT.
\post None.

\par IRQ-safe
<b>No</b> Function will be called from Mainloop context.

\ingroup grp_api_port

*/
IOL_FUNC_DECL INT16U IOLM_Port_DS_u16LoadData(INT8U u8Port_p, INT8U *pu8Data_p, INT16U u16Length_p);


#ifdef IOLM_DEVICE_TESTER
/**
\brief Device tester port.

This function is used for Device tester events. The function is called if
a event in the stack events which could be used to trigger the Master tester
or change internal states. This function should return 0 in normal cases.
A value >0 is used to generate protocol faults. This API is not documented
in detail.

\param[in] eMTestEvent_p Test event.

\return 0 if disabled

\par IRQ-safe
<b>Yes</b>  Function can be called in interrupt context.

\ingroup grp_api_diag

*/
IOL_FUNC_DECL INT32U IOLM_Port_u32DeviceTester(INT8U u8Port_p, IOLM_EMTestEvent eMTestEvent_p);
#endif

#ifdef IOLM_WIRELESS

/**
\brief Set Master configuration confirmation.

This function is called from the Stack if the Master configuration request has finished. 

\param[in] eError_p Error information.

\par IRQ-safe
<b>No</b> Function needs to be called in Mainloop context.

\ingroup grp_api_sm

*/
IOL_FUNC_DECL void IOLM_Port_SM_vSetMasterConfigCnf(IOL_EErrorInfo eError_p);


/**
\brief Set track configuration request.

This function is called from the stack if the track configuration has to change
on the physical layer.

\param[in] u8Track_p    Track number.
\param[in] psuParam_p   Parameter set.

\par IRQ-safe
<b>No</b> Function needs to be called in Mainloop context.

\ingroup grp_api_bsp

*/
IOL_FUNC_DECL IOL_EErrorInfo IOLM_Port_PL_eSetTrackConfigReq(INT8U u8Track_p, const IOLM_STrackparameterList *psuParam_p);



/**
\brief Set master Cycle configuration request.

This function is called from the stack if the track configuration has to change
on the physical layer.

\param[in] u8Track_p    Track number.
\param[in] u8Slot_p     Slot number set.
\param[in] u8MasterCycleTimePDIn    Coded Master cycle for PDIn data.
\param[in] u8MasterCycleTimePDOut   Coded Master cycle for PDOut data.

\return void

\par IRQ-safe
<b>No</b> Function needs to be called in Mainloop context.

\ingroup grp_api_bsp

*/
IOL_FUNC_DECL IOL_EErrorInfo IOLM_DL_eSetMasterCycleTime(INT8U u8Track_p, INT8U u8Slot_p,
                                                         INT8U u8MasterCycleTimePDIn,
                                                         INT8U u8MasterCycleTimePDOut);

/**
\brief Set mode request.

This function is called from the stack if the mode has to change on the
physical layer.

\param[in] u8Track_p    Track number.
\param[in] eTrackMode_p Target track mode.
\param[in] u8TxPower_p  Transmission power.

\par IRQ-safe
<b>No</b> Function needs to be called in Mainloop context.

\ingroup grp_api_bsp

*/
IOL_FUNC_DECL IOL_EErrorInfo IOLM_Port_PL_eSetModeReq(INT8U u8Track_p,
    IOLM_ETrackMode eTrackMode_p, INT8U u8TxPower_p);

/**
\brief Set antenna gain request.

This function is called from the stack if the antenna gain has to change on the
physical layer.

\param[in] i8Gain_p    Antenna gain

\par IRQ-safe
<b>No</b> Function needs to be called in Mainloop context.

\ingroup grp_api_bsp

*/
IOL_FUNC_DECL void IOLM_Port_PL_eSetAntennaGainReq(INT8U u8Track_p, INT8S s8Gain_p);

/**
\brief Track mode confirmation.

This function is called from the stack if the #IOLM_API_SM_eSetTrackModeReq has finished.

\param[in] u8Track_p    Track number.
\param[in] eError_p     Error information.

\par IRQ-safe
<b>No</b> Function needs to be called in Mainloop context.

\ingroup grp_api_sm

*/
IOL_FUNC_DECL void IOLM_Port_SM_vSetTrackModeCnf(INT8U u8Track_p, IOL_EErrorInfo eError_p);

/**
\brief Port pairing confirmation.

This function is called from the stack if the #IOLM_API_SM_ePortPairingReq has finished.

\param[in] u8Port_p Port number.
\param[in] eError_p Error information.

\par IRQ-safe
<b>No</b> Function needs to be called in Mainloop context.

\ingroup grp_api_sm

*/
IOL_FUNC_DECL void IOLM_Port_SM_ePortPairingCnf(INT8U u8Port_p, IOL_EErrorInfo eError_p);

/**
\brief Set port configuration confirmation.

This function is called from the stack if the #IOLM_API_SM_eSetPortConfigReq has finished.

\param[in] u8Port_p Port number.
\param[in] eError_p Error information.

\par IRQ-safe
<b>No</b> Function needs to be called in Mainloop context.

\ingroup grp_api_sm

*/
IOL_FUNC_DECL void IOLM_Port_SM_eSetPortConfigCnf(INT8U u8Port_p, IOL_EErrorInfo eError_p);

/**
\brief Scan result indication.

This function is called from the stack if a scan result is received in scan mode.

\param[in] u8Track_p    Track number.
\param[in] psuParam_p   Scan result information.
\param[in] boScanEnd_p  True if scan has finished.

\par IRQ-safe
<b>No</b> Function needs to be called in Mainloop context.

\ingroup grp_api_sm

*/
IOL_FUNC_DECL void IOLM_Port_SM_vTrackScanResultInd(INT8U u8Track_p,
    IOLM_SScanParam *psuParam_p, TBOOL boScanEnd_p);


/**
\brief PL pairing request.

This function is called to pair a Device on the physical layer.

\param[in] u8Track_p        Track number.
\param[in] u8TargetTrack_p  Track which the Device should assigned to.
\param[in] u8Slot_p         Scan slot number.
\param[in] psuParam_p       Pairing parameter.

\return Error information

\ingroup grp_api_bsp

*/
IOL_FUNC_DECL IOL_EErrorInfo IOLM_Port_PL_ePairingReq(INT8U u8Track_p, INT8U u8TargetTrack_p, 
    INT8U u8Slot_p, IOLM_SPairingParam *psuParam_p);

/**
\brief PL set slot configuration.

This function is called to set the slot configuration in physical layer.

\param[in] u8Track_p        Track number.
\param[in] u8Slot_p Scan    Slot number.
\param[in] psuSlotConfig_p  Slot parameter.

\return Error information

\ingroup grp_api_bsp

*/
IOL_FUNC_DECL IOL_EErrorInfo IOLM_Port_PL_eSetSlotConfig(INT8U u8Track_p, INT8U u8Slot_p,
    IOLM_SPlSlotConfig *psuSlotConfig_p);

/**
\brief PL transfer request.

This function is used to set the next downlink message.

\param[in] u8Track_p        Track number.
\param[in] pu8PreDlData_p   2 bytes pre downlink data.
\param[in] pu8Data_p        Downlink data.
\param[in] u8DataLength_p   Downlink length.

\return Error information

\ingroup grp_api_bsp

*/
IOL_FUNC_DECL IOL_EErrorInfo IOLM_Port_PL_eTransferReq(INT8U u8Track_p, INT8U *pu8PreDlData_p,
    INT8U *pu8Data_p, INT8U u8DataLength_p);

/**
\brief PL quality request.

This function is used to request the signal quality of the specified slot.

\param[in] u8Track_p    Track number.
\param[in] u8Slot_p     Slot number.

\return Error information

\ingroup grp_api_bsp

*/
IOL_FUNC_DECL IOL_EErrorInfo IOLM_Port_PL_eQualityServiceReq(INT8U u8Track_p, INT8U u8Slot_p);

/**
\brief PL command trigger request.

This function is used to trigger a command in the physical Layer.

\param[in] u8Track_p    Track number.
\param[in] eCommand_p   Command.

\return Error information

*/
IOL_FUNC_DECL IOL_EErrorInfo IOLM_Port_PL_eCmdTrigReq(INT8U u8Track_p, IOL_EPLCommand eCommand_p);

/**
\brief PL settings request.

This function is used to set/get some non specific settings.

\param[in] u8Track_p    Track number.
\param[in] pu8Data_p    Data.
\param[in] u16Length_p  Data length.

\return Error information

\ingroup grp_api_bsp

*/
IOL_FUNC_DECL void IOLM_Port_PL_vSettingsReq(INT8U u8Track_p, INT8U *pu8Data_p, INT16U u16Length_p);

/**
\brief PL settings response.

This function is used to set/get some non specific settings.

\param[in] u8Track_p    Track number.
\param[in] pu8Data_p    Data.
\param[in] u16Length_p  Data length.

\return Error information

\ingroup grp_api_bsp

*/
IOL_FUNC_DECL void IOLM_Port_PL_vSettingsRsp(INT8U u8Track_p, INT8U *pu8Data_p, INT16U u16Length_p);

/**
\brief PL firmware update response.

This function is used to transfer firmware binaries to the specific controllers.

\param[in] u8Track_p    Track number.
\param[in] u8MMMS_p     MM = 1, MS = 2.
\param[in] eResponse_p  ErrorCode.

\return Error information

\ingroup grp_api_bsp

*/
IOL_FUNC_DECL void IOLM_Port_PL_vFirmwareUpdateRsp(INT8U u8Track_p, INT8U u8MMMS_p, IOL_EErrorType eResponse_p);

/**
\brief PL firmware update request.

This function is used to transfer firmware binaries to the specific controllers.

\param[in] u8Track_p    Track number.
\param[in] u8MMMS_p     MM = 1, MS = 2.
\param[in] pu8Data_p    Data.
\param[in] u16Length_p  Data length.

\ingroup grp_api_bsp

*/
IOL_FUNC_DECL void IOLM_Port_PL_vFirmwareUpdateReq(INT8U u8Track_p, INT8U u8MMMS_p, INT8U *pu8Data_p, INT16U u16Length_p);

/**
\brief PL chip info request.

This function is used to get Information about the specific controllers.

\param[in] u8Track_p    Track number
\param[in] u8MMMS_p     MM = 1, MS = 2
\param[in] eResponse_p  ErrorCode

\return Error Information

\ingroup grp_api_bsp

*/
IOL_FUNC_DECL void IOLM_Port_PL_vChipInfoRsp(INT8U u8Track_p, INT8U *pu8Data_p, INT16U u16Length_p);

/**
\brief PL chip info response.

This function is used to get Information about the specific controllers.

\param[in] u8Track_p        Track number.
\param[in] u8Instance_p     MM = 1, MS = 2.
\param[in] pu8Data_p        Data.
\param[in] u16Length_p      Data length.

\ingroup grp_api_bsp

*/
IOL_FUNC_DECL void IOLM_Port_PL_vChipInfoReq(INT8U u8Track_p, INT8U *pu8Data_p, INT16U u16Length_p);

/**
\brief Port quality confirmation.

The function is used to request the port quality.

\param[in] u8Port_p     Port number.
\param[in] u8QualityM_p Link quality indication (LQI_M) determined by master
                        values: 0% to 100 % (0x00 to 0x64) or INVALID (0xFF)
\param[in] u8RssiM_p    Received signal strength indication measured by master (RSSI_M)
                        values: (-128 to 20) for the estimated RF input power
                        in dBm or +127 for INVALID if the RSSI is not available
                        or not supported
\param[in] u8QualityD_p Link quality indication (LQI_D) determined by device
                        values: 0% to 100 % (0x00 to 0x64) or INVALID (0xFF)
\param[in] u8RssiD_p    Received signal strength indication measured by device (RSSI_D)
                        values: (-128 to 20) for the estimated RF input power
                        in dBm or +127 for INVALID if the RSSI is not available
                        or not supported
\param[in] eError_p     ErrorCode.

\ingroup grp_api_sm

*/
IOL_FUNC_DECL void IOLM_Port_SM_vGetPortQualityCnf(INT8U u8Port_p, INT8U u8QualityM_p, INT8S s8RssiM_p, INT8U u8QualityD_p, INT8S s8RssiD_p, IOL_EErrorInfo eError_p);


/**
\brief This service is used to get the hopping table from the PL

\param[in] u8Track_p        Track number (track that shall process this request)
\param[in] u8TargetTrack_p  Target track number (track whose hopping table shall is required)

\ingroup grp_api_bsp
*/
IOL_FUNC_DECL IOL_EErrorInfo IOLM_Port_PL_eGetHopTableReq(INT8U u8Track_p, INT8U u8TargetTrack_p);

/**
\brief This service is used to set the hopping table to the PL

\param[in] u8Track_p            Track number (track that shall process this request)
\param[in] u8TargetTrack_p      Target track number (track whose hopping table shall be set)
\param[in] eUpdateType_p        This parameter contains the type of update, for 
                                the usage of UpdateType.
\param[in] u8Index_p            This parameter contains the index of the changed
                                cell in the hopping table or the table length if 
                                update type is full table. 
                                Permitted values: 1-78
\param[in] pu8Data_p            Pointer to the full table or the cell value to replace/add.
\ingroup grp_api_bsp
*/
IOL_FUNC_DECL IOL_EErrorInfo IOLM_Port_PL_eSetHopTableReq(INT8U u8Track_p, INT8U u8TargetTrack_p, IOL_EHopUpdateType eUpdateType_p, INT8U u8Index_p, INT8U *pu8Data_p);

/**
\brief This service is used to to set a countdown in the PL

\param[in] u8Track_p            Track number.
\param[in] u32WakeUpTime_p      This parameter contains the WakeUpTime in W-Sub-cycles to set in PL. 
                                Permitted values: 0 to 16777215 (3 Octets)

\ingroup grp_api_bsp
*/
IOL_FUNC_DECL IOL_EErrorInfo IOLM_Port_PL_eSetWakeUpTimeReq(INT8U u8Track_p, INT32U u32WakeUpTime_p);


/**
\brief Response to PL_WakeUpTime indication 

\param[in] u8Track_p    Track number.
\param[in] eError_p     ErrorCode.

\ingroup grp_api_bsp
*/
IOL_FUNC_DECL void IOLM_Port_PL_vWakeUpTimeRsp(INT8U u8Track_p, IOL_EErrorInfo eError_p);

/**
\brief Response to PL_AHTStatus indication 

\param[in] u8Track_p    Track number.
\param[in] eError_p     ErrorCode.

\ingroup grp_api_bsp
*/
IOL_FUNC_DECL void IOLM_Port_PL_vAHTStatusRsp(INT8U u8Track_p, IOL_EErrorInfo eError_p);

#endif

#ifdef IOLM_OS_AVAILABLE
/**
\brief Request Mainloop processing.

This function is called if the stack needs Mainloop processing due to pending events.

\ingroup grp_api_general

*/
IOL_FUNC_DECL void IOLM_Port_SYS_vMainLoopRequest(void);
#endif

#ifdef IOL_SAFETY
/**
\brief Activate ready pulse detection.

This function is called at the begin of the ready pulse detection e.g. to activate the interrupt handler.

\param[in] u8Port_p Port number.

\pre The port number must be smaller than #IOLM_PORT_COUNT.
\post None.

\ingroup grp_api_port

*/
IOL_FUNC_DECL void IOLM_Port_PL_vStartReadyPulseScan(INT8U u8Port_p);

/**
\brief Deactivate ready pulse detection.

This function is called at the end of the ready pulse detection e.g. to deactivate the interrupt handler.

\param[in] u8Port_p Port number.

\pre The port number must be smaller than #IOLM_PORT_COUNT.
\post None.

\ingroup grp_api_port

*/
IOL_FUNC_DECL void IOLM_Port_PL_vStopReadyPulseScan(INT8U u8Port_p);

#endif

#ifndef IOLM_WIRELESS
/**
\brief Switch port power on or off.

This function is called to switch on/off L+.

\param[in] u8Port_p     Port number.
\param[in] boOn_p       TRUE = switch on, FALSE = switch off.

\pre The port number must be smaller than #IOLM_PORT_COUNT.
\post None.

\ingroup grp_api_port

*/
IOL_FUNC_DECL void IOLM_Port_PL_vSwitchPortPower(INT8U u8Port_p, TBOOL boOn_p);
#endif

#ifdef __cplusplus
}
#endif


#endif
