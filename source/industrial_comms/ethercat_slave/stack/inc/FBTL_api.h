/*!
* \file FBTL_api.h
*
* \brief
* Fieldbus transport layer API interface.
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

#if !(defined __FBTL_API_H__)
#define __FBTL_API_H__		1

#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>

#if defined _WIN32 || defined __CYGWIN__
    #define FBTL_DLL_IMPORT __declspec(dllimport)
    #define FBTL_DLL_EXPORT __declspec(dllexport)
    #define FBTL_DLL_LOCAL
#else
    #if __GNUC__ >= 4
        #define FBTL_DLL_IMPORT __attribute__ ((visibility ("default")))
        #define FBTL_DLL_EXPORT __attribute__ ((visibility ("default")))
        #define FBTL_DLL_LOCAL  __attribute__ ((visibility ("hidden")))
    #else
        #define FBTL_DLL_IMPORT
        #define FBTL_DLL_EXPORT
        #define FBTL_DLL_LOCAL
    #endif
#endif

typedef void(*FBTL_SVC_CBdataReceive_t) (void*       pFbtlHandle_p
                                        ,void*       pFbtlQueueElement_p
                                        ,uint8_t*    pData_p
                                        ,uint32_t    length_p);

typedef void(*FBTL_SVC_CBledChange_t)   (void*      pFbtlHandle_p
                                        ,bool       errLed_p
                                        ,bool       runLed_p);

typedef void(*FBTL_SVC_CBpdChange_t)    (void*      pFbtlHandle_p
                                        ,void*      pContext_p);

typedef void(*FBTL_SVC_CBsyncFunc_t)    (void*      pFbtlHandle_p
                                        ,void*      pContext_p);

typedef bool(*FBTL_SVC_CBisSlowFunc_t)  (void*      pFbtlHandle_p
                                        ,uint16_t   serviceCode_p);

/**
\brief Process data configuration
*
*
* \ingroup grp_gen_service
* \details
* The Process Data Buffers can be configured in different ways. It is compulsory to select one mode.
* If EC_Stack is selected then only EtherCAT Services can be used to read and write PDOs.
* If PD_Gen types are selected then only Generic Services can be used to read and write PDOs.
*/
typedef enum FBTL_API_EPDMode
{
    FBTL_API_ePD_none               = 0x00,     /*!< If the PD Mode is not configured then the system wont run*/
    FBTL_API_ePD_bus_Specific       = 0x01,     /*!< Uses the default buffers of the EtherCAT Stack. EtherCAT Services must be used to manipulate the Process Data. */
    FBTL_API_ePD_single_Buffer      = 0x02,     /*!< Uses a single buffer for the process data. Generic Services must to access the process data memory.
                                                 *   No EtherCAT helper functions available to manipulate  the PD*/
    FBTL_API_ePD_double_Buffer      = 0x03,     /*!< Uses a double buffer for the process data. Generic Services must to access the process data memory.
                                                 *   No EtherCAT helper functions available to manipulate  the PD*/
    FBTL_API_ePD_triple_Buffer      = 0x04,     /*!< Uses a triple buffer for the process data. Generic Services must to access the process data memory.
                                                 *   No EtherCAT helper functions available to manipulate  the PD*/
    FBTL_API_ePD_quad_Buffer        = 0x05      /*!< Uses a quad buffer for the process data. Generic Services must to access the process data memory.
                                                 *   No EtherCAT helper functions available to manipulate  the PD*/
} FBTL_API_EPDMode_t;

/*!
    \brief Interrupt status register

\ingroup FBTL_API
*/
typedef enum FBTL_API_EInterruptStatus
{
    /// @cond internal
    FBTL_API_eIRQ_noIrq     = 0x00000000,       ///!< No interrupt */
    /// @endcond

    FBTL_API_eIRQ_status    = 0x00000001,       ///!< Stack status word did change
    FBTL_API_eIRQ_acyclic   = 0x00000002,       ///!< Acyclic channel data updated
    FBTL_API_eIRQ_cyclic    = 0x00000004,       ///!< Cyclic channel data changed
    FBTL_API_eIRQ_sync      = 0x00000008,       ///!< Bus timing synchronization (DC, IRT DLR)

    FBTL_API_eIRQ_spec0L    = 0x00000100,       ///!< Bus Specific DWORD 0 Lower Half
    FBTL_API_eIRQ_spec0H    = 0x00000200,       ///!< Bus Specific DWORD 0 Upper Half
    FBTL_API_eIRQ_spec1L    = 0x00000400,       ///!< Bus Specific DWORD 1 Lower Half
    FBTL_API_eIRQ_spec1H    = 0x00000800,       ///!< Bus Specific DWORD 1 Upper Half
    FBTL_API_eIRQ_spec2L    = 0x00001000,       ///!< Bus Specific DWORD 2 Lower Half
    FBTL_API_eIRQ_spec2H    = 0x00002000,       ///!< Bus Specific DWORD 2 Upper Half
    FBTL_API_eIRQ_spec3L    = 0x00004000,       ///!< Bus Specific DWORD 3 Lower Half
    FBTL_API_eIRQ_spec3H    = 0x00008000,       ///!< Bus Specific DWORD 3 Upper Half
    FBTL_API_eIRQ_spec4L    = 0x00010000,       ///!< Bus Specific DWORD 4 Lower Half
    FBTL_API_eIRQ_spec4H    = 0x00020000,       ///!< Bus Specific DWORD 4 Upper Half
    FBTL_API_eIRQ_spec5L    = 0x00040000,       ///!< Bus Specific DWORD 5 Lower Half
    FBTL_API_eIRQ_spec5H    = 0x00080000,       ///!< Bus Specific DWORD 5 Upper Half
    FBTL_API_eIRQ_spec6L    = 0x00100000,       ///!< Bus Specific DWORD 6 Lower Half
    FBTL_API_eIRQ_spec6H    = 0x00200000,       ///!< Bus Specific DWORD 6 Upper Half
    FBTL_API_eIRQ_spec7L    = 0x00400000,       ///!< Bus Specific DWORD 7 Lower Half
    FBTL_API_eIRQ_spec7H    = 0x00800000,       ///!< Bus Specific DWORD 7 Upper Half

    /// @cond internal
    FBTL_API_eIRQ_F32       = 0xFFFFFFFF
    /// @endcond
} FBTL_API_EInterruptStatus_t;

/**
\brief Service Type


\ingroup grp_gen_service
*/
typedef enum FBTL_EServiceType
{
    FBTL_eSRV_General   = 0,    ///<!   General service
#if (defined FBTL_EC_SLV_SUPPORT) && (FBTL_EC_SLV_SUPPORT==1)
    FBTL_eSRV_EtherCAT,         ///<!   EtherCAT service
#endif
#if (defined FBTL_PNIO_SUPPORT) && (FBTL_PNIO_SUPPORT==1)
    FBTL_eSRV_Profinet,         ///<!   ProfiNET service
#endif
#if (defined FBTL_ENIP_SUPPORT) && (FBTL_ENIP_SUPPORT==1)
    FBTL_eSRV_EthernetIp,       ///<!   Ethernet/OP
#endif
#if (defined FBTL_IOLM_SUPPORT) && (FBTL_IOLM_SUPPORT==1)
    FBTL_eSRV_IoLinkMaster,     ///<!   IO-Link master
#endif

    FBTL_eSRV_Max,
    /// @cond INTERNAL
    FBTL_eSRV_Reserved = 0xFF
    /// @endcond
} FBTL_EServiceType_t;

/*!
    \brief Error codes

    \ingroup FBTL_API
*/
/* keep synchronuous to OSAL */
typedef enum FBTL_API_eError
{
    FBTL_eERR_NOERROR                   = 0,    /*!< no error */
    FBTL_eERR_ENOENT                    = 2,    /*!< no such entity */
    FBTL_eERR_AGAIN                     = 11,   /*!< repeatable (temporary) error */
    FBTL_eERR_NOMEMORY                  = 12,   /*!< out of memory */
    FBTL_eERR_EINVAL                    = 22,   /*!< invalid parameter */
    FBTL_eERR_NOTIMPLEMENTED            = 34,   /*!< not implemented */
    FBTL_eERR_TIMEDOUT                  = 110,  /*!< timed out */
} FBTL_API_eError_t;

typedef bool (*FBTL_API_CBInterruptHandler_t)(void*                         pFbtlHandle_p
                                             ,void*                         pContext_p
                                             ,FBTL_API_EInterruptStatus_t   irq_p
                                             ,volatile uint32_t*            pIntClearWord_p);

#if (defined __cplusplus)
extern "C" {
#endif

extern FBTL_DLL_EXPORT uint32_t FBTL_API_CFG_prepare    (void*                     		pSysInterface_p
                                                        ,void**                    		pFbtlConfig_p);
extern FBTL_DLL_EXPORT uint32_t FBTL_API_CFG_release    (void**                    		pFbtlConfig_p);
extern FBTL_DLL_EXPORT uint32_t FBTL_API_CFG_acyclic    (void*                     		pFbtlConfig_p
                                                        ,uint32_t                  		toBusSize_p
                                                        ,uint32_t                  		fmBusSize_p);
extern FBTL_DLL_EXPORT uint32_t FBTL_API_CFG_procData   (void*                     		pFbtlConfig_p
                                                        ,uint32_t                  		toBusSize_p
                                                        ,uint32_t                  		fmBusSize_p
                                                        ,FBTL_API_EPDMode_t        		procDataMode_p);

extern FBTL_DLL_EXPORT uint32_t FBTL_API_init           (void*                     		pSysInterface_p
                                                        ,void*                     		pFbtlConfig_p
                                                        ,void**                    		pFbtlHandle_p);
extern FBTL_DLL_EXPORT uint32_t FBTL_API_deinit         (void**                    		pFbtlHandle_p);
extern FBTL_DLL_EXPORT FBTL_API_EPDMode_t
                                FBTL_API_procDataMode   (void*                      	pFbtlHandle_p);

extern FBTL_DLL_EXPORT uint32_t FBTL_API_run            (void*                     		pFbtlHandle_p);

extern FBTL_DLL_EXPORT void     FBTL_API_triggerRun     (void*                          pFbtlHandle_p);

extern FBTL_DLL_EXPORT uint32_t FBTL_API_sendAcycWait   (void*                          pFbtlHandle_p);
extern FBTL_DLL_EXPORT void     FBTL_API_sendAcycRun    (void*                          pFbtlHandle_p);

extern FBTL_DLL_EXPORT uint32_t FBTL_API_syncSignalWait (void*                          pFbtlHandle_p);
extern FBTL_DLL_EXPORT uint32_t FBTL_API_syncSignalRun  (void*                          pFbtlHandle_p);

extern FBTL_DLL_EXPORT uint32_t FBTL_API_receiveWait    (void*                          pFbtlHandle_p);

extern FBTL_DLL_EXPORT void     FBTL_API_receiverRun    (void*                     		pFbtlHandle_p
                                                        ,bool                           responseCheck_p);
extern FBTL_DLL_EXPORT void     FBTL_API_timerRun       (void*                     		pFbtlHandle_p);

extern FBTL_DLL_EXPORT uint32_t FBTL_API_serviceWait    (void*                          pFbtlHandle_p);
extern FBTL_DLL_EXPORT uint32_t FBTL_API_serviceRun     (void*                          pFbtlHandle_p);
extern FBTL_DLL_EXPORT uint32_t FBTL_API_slowServiceWait(void*                          pFbtlHandle_p);
extern FBTL_DLL_EXPORT uint32_t FBTL_API_slowServiceRun (void*                          pFbtlHandle_p);

extern FBTL_DLL_EXPORT void     FBTL_API_regStateVar    (void*                     		pFbtlHandle_p
                                                        ,uint32_t*                 		pStateOffload_p);
extern FBTL_DLL_EXPORT void     FBTL_API_regLedCB       (void*                     		pFbtlHandle_p
                                                        ,FBTL_SVC_CBledChange_t    		cbLed_p);

extern FBTL_DLL_EXPORT void     FBTL_API_registerRxCb   (void*                     		pFbtlHandle_p
                                                        ,uint32_t                  		service_p
                                                        ,FBTL_SVC_CBdataReceive_t  		cbFunc_p);

extern FBTL_DLL_EXPORT void     FBTL_API_registerPDChgCb(void*                    		pFbtlHandle_p
                                                        ,void*                    		pContext_p
                                                        ,FBTL_SVC_CBpdChange_t    		cbFunc_p);
extern FBTL_DLL_EXPORT void     FBTL_API_registerSyncCb (void*                    		pFbtlHandle_p
                                                        ,void*                    		pContext_p
                                                        ,FBTL_SVC_CBsyncFunc_t    		cbFunc_p);
extern FBTL_DLL_EXPORT void     FBTL_API_registerSlowCb (void*                          pFbtlHandle_p
                                                        ,void*                          pContext_p
                                                        ,FBTL_SVC_CBisSlowFunc_t        cbFunc_p);

extern FBTL_DLL_EXPORT void     FBTL_API_service_send   (void*                     		pFbtlHandle_p
                                                        ,uint8_t*                  		pData_p
                                                        ,uint32_t                  		length_p);
extern FBTL_DLL_EXPORT void     FBTL_API_service_sendTimeout
                                                        (void*                     		pFbtlHandle_p
                                                        ,uint8_t*                  		pData_p
                                                        ,uint32_t                  		length_p
                                                        ,uint32_t                  		timeoutMs_p);
extern FBTL_DLL_EXPORT uint32_t FBTL_API_service_sendReceive
                                                        (void*                     		pFbtlHandle_p
                                                        ,uint8_t*                  		pTxData_p
                                                        ,uint32_t                  		txLength_p
                                                        ,uint8_t**                 		pRxData_p
                                                        ,uint32_t*                 		pRxLength_p
                                                        ,uint32_t                  		timeoutMs_p);
extern FBTL_DLL_EXPORT uint32_t FBTL_API_service_sendACK(void*                     		pFbtlHandle_p
                                                        ,uint8_t*                  		pData_p
                                                        ,uint32_t                  		length_p
                                                        ,uint32_t                  		timeoutMs_p);
extern FBTL_DLL_EXPORT uint32_t FBTL_API_service_receive(void*                     		pFbtlHandle_p
                                                        ,uint32_t                  		service_p
                                                        ,uint8_t**                      pRxData_p
                                                        ,uint32_t*                      pRxLength_p
                                                        ,uint32_t                       timeoutMs_p);

extern FBTL_DLL_EXPORT uint32_t FBTL_API_IRQ_enterHandler(
                                                         void*                          pFbtlHandle_p
                                                        ,FBTL_API_EInterruptStatus_t    workIrq_p);
extern FBTL_DLL_EXPORT uint32_t FBTL_API_IRQ_exitHandler(void*                          pFbtlHandle_p
                                                        ,FBTL_API_EInterruptStatus_t    workIrq_p);
extern FBTL_DLL_EXPORT uint32_t FBTL_API_IRQ_irqHandler (void*                          pFbtlHandle_p
                                                        ,FBTL_API_EInterruptStatus_t    workIrq_p
                                                        ,uint32_t                       waitTimeout_p);


extern FBTL_DLL_EXPORT void     FBTL_API_workInterrupt  (void*                          pFbtlHandle_p);

#if (defined __cplusplus)
}
#endif

#endif /* __FBTL_API_H__ */
