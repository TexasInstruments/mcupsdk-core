/*!
* \file FBTL_service.h
*
* \brief
* FBTL Service generic interface.
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

#if !(defined __FBTL_SERVICE_H__)
#define __FBTL_SERVICE_H__		1

#include <FBTL_api.h>
#include <FBTL_queue.h>
#include <FBTL_serviceTypes.h>

#if (defined FBTL_EC_SLV_SUPPORT) && (FBTL_EC_SLV_SUPPORT == 1)
#include "fieldbus/FBTL_ECSLV_serviceTypes.h"
#endif
#if (defined FBTL_PNIO_SUPPORT) && (FBTL_PNIO_SUPPORT == 1)
#include "fieldbus/FBTL_PNIO_serviceTypes.h"
#endif
#if (defined FBTL_ENIP_SUPPORT) && (FBTL_ENIP_SUPPORT == 1)
#include "fieldbus/FBTL_ENIP_serviceTypes.h"
#endif
#if (defined FBTL_IOLM_SUPPORT) && (FBTL_IOLM_SUPPORT == 1)
#include "fieldbus/FBTL_IOLM_serviceTypes.h"
#endif

/**
\brief General Service Header

Header is used for all services.

\ingroup FBTL_GEN_SERVICE
*/
typedef struct FBTL_SVC_SHeader
{
    uint16_t                            service;            ///!<   Service according \ref FBTL_SVC_EServiceType
    uint16_t                            length;             ///!<   Number of Data bytes following after this structure
    uint32_t                            serviceIdent;       ///!<   Service identification number
    uint32_t                            framesToFollow;     ///!<   Number of frames to follow
    uint32_t                            errorCode;          ///!<   Error Code
} FBTL_SVC_SHeader_t;

/**
\brief Message with Uint8 Value

Used for simple commands which only needs 8 bit values

\ingroup grp_gen_service
*/
typedef struct FBTL_SVC_SUint8Header
{
    FBTL_SVC_SHeader_t                  header;             ///!<   administrative header
    uint8_t                             data;               ///!<   Payload
} FBTL_SVC_SUint8Header_t;

/**
\brief Message with Uint16 Value

Used for simple commands which only needs 16 bit values

\ingroup grp_gen_service
*/
typedef struct FBTL_SVC_SUint16Header
{
    FBTL_SVC_SHeader_t                  header;             ///!<   administrative header
    uint16_t                            data;               ///!< Payload
} FBTL_SVC_SUint16Header_t;

/**
\brief Message with Uint32 Value

Used for simple commands which only needs 32 bit values

\ingroup grp_gen_service
*/
typedef struct FBTL_SVC_SUint32Header
{
    FBTL_SVC_SHeader_t                  header;             ///!<   administrative header
    uint32_t                            data;               ///!<   Payload
} FBTL_SVC_SUint32Header_t;

/**
\brief Message with double Uint8 Value

Used for simple commands which only needs 8 bit values

\ingroup grp_gen_service
*/
typedef struct FBTL_SVC_S2Uint8Header
{
    FBTL_SVC_SHeader_t                  header;             ///!<   administrative header
    uint8_t                             data1;              ///!<   Payload
    uint8_t                             data2;              ///!<   Payload
} FBTL_SVC_S2Uint8Header_t;

/**
\brief Message with double Uint16 Value

Used for simple commands which only needs 16 bit values

\ingroup grp_gen_service
*/
typedef struct FBTL_SVC_S2Uint16Header
{
    FBTL_SVC_SHeader_t                  header;             ///!<   administrative header
    uint16_t                            data1;              ///!<   Payload
    uint16_t                            data2;              ///!<   Payload
} FBTL_SVC_S2Uint16Header_t;

/**
\brief Message with Double Uint32 Value

Used for simple commands which only needs 2x32 bit values

\ingroup grp_gen_service
*/
typedef struct FBTL_SVC_S2Uint32Header
{
    FBTL_SVC_SHeader_t                  header;             ///!<   administrative header
    uint32_t                            data1;              ///!<   Payload
    uint32_t                            data2;              ///!<   Payload
} FBTL_SVC_S2Uint32Header_t;

/**
\brief Message with string Value

Used for simple commands which only needs string or data array values

\ingroup grp_gen_service
*/
typedef struct FBTL_SVC_SStringHeader
{
    FBTL_SVC_SHeader_t                  header;             ///!<   administrative header
    char                                aData[128];         ///!<   data, actual length can be reduced
} FBTL_SVC_SStringHeader_t;

/**
\brief Process data configuration

\ingroup grp_gen_service
*/
typedef struct FBTL_SVC_SMsgPdConfigHeader
{
    FBTL_SVC_SHeader_t  header;
    uint8_t             operationMode; ///!< Buffer Configuration
} FBTL_SVC_SMsgPdConfigHeader_t;

typedef struct FBTL_SVC_SElement
{
    struct FBTL_SVC_SServiceMessage
    {
        uint32_t                            size;           ///!<   Size of service message
        FBTL_SVC_SHeader_t*                 pMessage;       ///!<   Service data
        FBTL_SVC_CBdataReceive_t            cbCallback;     ///!<   Service response callback
    } serviceMessage;
    void*                                   pSignal;        ///!<   Finish signal

    struct FBTL_SVC_SSegmentation
    {
        uint32_t                            transferred;    ///!<   Already transferred bytes (segmentation)
        uint8_t*                            pSegmentPtr;    ///!<   pointer (segmentation)
    } segmentation;

    struct FBTL_SVC_STimeout
    {
        uint32_t                            startMs;        ///!<   Service Queue entrance timestamp
        uint32_t                            timeoutMs;      ///!<   Service Queue timeout timestamp
    } timeout;

    FBTL_QUEUE_SHook_t*                     pWorkflow;      ///!<   Workflow hook
    FBTL_QUEUE_SHook_t*                     pTimeout;       ///!<   Timeout hook
} FBTL_SVC_SElement_t;


#if (defined __cplusplus)
extern "C" {
#endif

extern char*    FBTL_SVC_enumString         (uint32_t   enumVal);

extern void     FBTL_SVC_register           (void*      pFbtlHandle_p);

extern uint32_t FBTL_SVC_configurePDBuffer  (void*      pFbtlHandle_p);
extern uint32_t FBTL_SVC_getPDInBuffer      (void*      pFbtlHandle_p
                                            ,uint32_t   payloadLength_p
                                            ,void**     ppPDBuffer_p);
extern uint32_t FBTL_SVC_returnPDInBuffer   (void*      pFbtlHandle_p
                                            ,void*      pBuffer_p
                                            ,uint32_t   payloadLength_p);
extern uint32_t FBTL_SVC_getPDOutBuffer     (void*      pFbtlHandle_p
                                            ,uint32_t   payloadLength_p
                                            ,void**     ppPDBuffer_p);
extern uint32_t FBTL_SVC_returnPDOutBuffer  (void*      pFbtlHandle_p
                                            ,void*      pBuffer_p
                                            ,uint32_t   payloadLength_p);

extern uint8_t* FBTL_SVC_getReqBuffer       (void*      pFbtlHandle_p
                                            ,uint32_t   service_p
                                            ,uint32_t   payloadLength_p);
extern uint8_t* FBTL_SVC_getResBuffer       (void*      pFbtlHandle_p
                                            ,uint32_t   service_p
                                            ,uint32_t   ident_p
                                            ,uint32_t   payloadLength_p);
extern void     FBTL_SVC_releaseBuffer      (void*      pFbtlHandle_p
                                            ,uint8_t*   pBuffer_p);
extern bool     FBTL_SVC_isFastServiceType  (uint32_t   service_p);

extern void     FBTL_SVC_cbBlocking         (void*      pFbtlHandle_p
                                            ,void*      pFbtlQueueElement_p
                                            ,uint8_t*   pData_p
                                            ,uint32_t   length_p);
#if (defined __cplusplus)
}
#endif

#endif /* __FBTL_SERVICE_H__ */
