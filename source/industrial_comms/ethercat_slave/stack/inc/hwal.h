/*!
* \file hwal.h
*
* \brief
* Hardware abstraction layer interface.
*
* \author
* KUNBUS GmbH
*
* \date
* 2021-05-20
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

#if !(defined __HWAL_H__)
#define __HWAL_H__		1

#include <osal.h>

#if ((defined ECATSLAVE_SO) && (ECATSLAVE_SO==1) || ((defined ETHERNETIP_SO) && (ETHERNETIP_SO==1)) || ((defined PROFINETIO_SO) && (PROFINETIO_SO==1))) // defined if ECATSLV is compiled as a DLL
#if (defined HWALAPI_EXPORTS) || (defined ECSLVAPI_EXPORTS) // defined if we are building the ECATSLAVE DLL (instead of using it)
#define HWAL_API OSAL_DLL_EXPORT
#else
#define HWAL_API OSAL_DLL_IMPORT
#endif // ECSLVAPI_EXPORTS
#define HWAL_LOC OSAL_DLL_LOCAL
#else // ECATSLAVE_SO is not defined: this means ECATSLAVE is a static lib.
#define HWAL_API
#define HWAL_LOC
#endif // ECATSLAVE_SO

/* The PRUSS drivers before version 1.0.0.16 have a typo in macro names*/
#if (defined PRUICSS_DRV_VERSION_ID) && (PRUICSS_DRV_VERSION_ID < 0x01000010)
#define PRUICSS_INSTANCE_ONE   (PRUICCSS_INSTANCE_ONE)
#define PRUICSS_INSTANCE_TWO   (PRUICCSS_INSTANCE_TWO)
#define PRUICSS_INSTANCE_THREE (PRUICCSS_INSTANCE_THREE)
#define PRUICSS_INSTANCE_MAX   (PRUICCSS_INSTANCE_MAX)
#endif

/*!
 *  \brief
 *  Each ICSSG has two TX PRU's
 */
#define MAX_ICSSG_TX_PRU        2U
#define IDX_ICSSG_TX_PRU0       0U
#define IDX_ICSSG_TX_PRU1       1U

/*!
 *  \brief
 *  Performance counter precision settings.
  */
typedef enum HWAL_PRFCNT_EPrecision
{
    HWAL_PRFCNT_EPRECISION_LOW,                     // divider of performance counter is set to 64, which decreases precision of counter, but increases maximal measured value
    HWAL_PRFCNT_EPRECISION_HIGH,                    // divider of performance counter is set to 1, which increases precision of counter, but decreases maximal measured value
    HWAL_PRFCNT_EPRECISION_FORCE32BIT = 0xffffffff
} HWAL_PRFCNT_EPrecision_t;

/*!
 *  \brief
 *  Performance counter time units definition.
 */
typedef enum HWAL_PRFCNT_ETimeUnit
{
    HWAL_PRFCNT_eTIMEUNIT_MILLISECONDS = 1000,          // number of mili-seconds in one second
    HWAL_PRFCNT_eTIMEUNIT_MICROSECONDS = 1000000,       // number of micro-seconds in one second
    HWAL_PRFCNT_eTIMEUNIT_NANOSECONDS  = 1000000000,    // number of nano-seconds in one second
    HWAL_PRFCNT_eTIMEUNIT_FORCE32BIT   = 0xffffffff
}HWAL_PRFCNT_ETimeUnit_t;

typedef enum HWAL_eMEM_type {
    HWAL_UMT_dram0,
    HWAL_UMT_dram1,
    HWAL_UMT_shrdram,
    HWAL_UMT_intc,
    HWAL_UMT_cfg,
    HWAL_UMT_uart0,
    HWAL_UMT_iep,
    HWAL_UMT_ecap0,
    HWAL_UMT_mii_rt,
    HWAL_UMT_mdio,
    HWAL_UMT_ocmc,
    HWAL_UMT_control0,
    HWAL_UMT_control1,
    HWAL_UMT_debug0,
    HWAL_UMT_debug1,
    HWAL_UMT_iram0,
    HWAL_UMT_iram1,
    HWAL_UMT_TxPru0_CtlReg,
    HWAL_UMT_TxPru1_CtlReg,
    HWAL_UMT_TxPru0_iram,
    HWAL_UMT_TxPru1_iram,
} HWAL_eMEM_type_t;

typedef struct HWAL_MEM_SBarDescriptor
{
    uint32_t                    logicAddress;
    uint32_t                    baseAddress;
    uint32_t                    mapOffset;
    uint32_t                    mapSize;
} HWAL_MEM_SBarDescriptor_t;

typedef struct HWAL_PRU_SPhysical
{
    HWAL_MEM_SBarDescriptor_t   dram[2];
    HWAL_MEM_SBarDescriptor_t   shrDram;
    HWAL_MEM_SBarDescriptor_t   intc;
    HWAL_MEM_SBarDescriptor_t   cfg;
    HWAL_MEM_SBarDescriptor_t   uart0;
    HWAL_MEM_SBarDescriptor_t   iep;
    HWAL_MEM_SBarDescriptor_t   ecap0;
    HWAL_MEM_SBarDescriptor_t   mii_rt;
    HWAL_MEM_SBarDescriptor_t   mii_mdio;
    HWAL_MEM_SBarDescriptor_t   ocmc;
    HWAL_MEM_SBarDescriptor_t   control[2];
    HWAL_MEM_SBarDescriptor_t   debug[2];
    HWAL_MEM_SBarDescriptor_t   iram[2];
    HWAL_MEM_SBarDescriptor_t   txPruIram[MAX_ICSSG_TX_PRU];
    HWAL_MEM_SBarDescriptor_t   txPruCtlReg[MAX_ICSSG_TX_PRU];
} HWAL_PRU_SPhysical_t;

typedef uint32_t (*HWAL_PRFCNT_CBPrfCntOverflowHandler_t) (void);

#if (defined __cplusplus)
extern "C" {
#endif

extern HWAL_API uint32_t    HWAL_init(void);
extern          void        HWAL_deinit(void);

extern HWAL_API uint32_t    HWAL_getVersion                 (void);
extern HWAL_API uint32_t    HWAL_getVersionStr              (uint32_t                       bufLen_p
                                                            ,char*                          pBuffer_p
                                                            ,uint32_t*                      pUsedLen_p);
extern HWAL_API uint32_t    HWAL_getVersionId               (uint32_t                       bufLen_p
                                                            ,const char*                    pBuffer_p
                                                            ,uint32_t*                      pUsedLen_p);

extern const    char*       HWAL_memoryName                 (const HWAL_eMEM_type_t         memoryType_p);

extern          uint32_t    HWAL_pruInstByAddr              (uint32_t                       baseAddress_p
                                                            ,uint8_t*                       pPruIndex_p);
extern          uint32_t    HWAL_getMemory                  (uint8_t                        pruInstance_p
                                                            ,const HWAL_eMEM_type_t         memoryType_p
                                                            ,void**                         pLogicalMem_p
                                                            ,uint32_t*                      pSize_p);
extern          uint32_t    HWAL_freeMemory                 (void*                          pLogicalMem_p
                                                            ,uint32_t                       size_p);

extern          uint32_t    HWAL_getEventDevice             (uint8_t                        pruInstance_p
                                                            ,uint32_t                       irqNum_p
                                                            ,uint32_t                       bufSize_p
                                                            ,char*                          pEventDevice_p);
extern          uint32_t    HWAL_getPruPhysInfo             (uint8_t                        pruInstance_p
                                                            ,HWAL_PRU_SPhysical_t*          pInfo_p);
extern HWAL_API uint32_t    HWAL_cyclicTrigger              (void);
extern HWAL_API uint32_t    HWAL_leaveOp                    (void);

extern HWAL_API uint32_t    HWAL_PRFCNT_init                (HWAL_PRFCNT_EPrecision_t       precision_p);

extern HWAL_API uint32_t    HWAL_PRFCNT_getMpuCycles        (uint32_t*                      pOverflows_p
                                                            ,uint32_t*                      pMpuClockCycles_p);

extern HWAL_API uint32_t    HWAL_PRFCNT_getTimeFromCycles   (uint32_t                       overflows_p
                                                            ,uint32_t                       cyclesNum_p
                                                            ,HWAL_PRFCNT_ETimeUnit_t        timeUnit_p
                                                            ,uint32_t*                      pHiTime_p
                                                            ,uint32_t*                      pLoTime_p);

extern HWAL_API uint32_t    HWAL_PRFCNT_getDeltaTime        (uint32_t                       hiStartCycles_p
                                                            ,uint32_t                       loStartCycles_p
                                                            ,uint32_t                       hiEndCycles_p
                                                            ,uint32_t                       loEndCycles_p
                                                            ,HWAL_PRFCNT_ETimeUnit_t        timeUnit_p
                                                            ,uint32_t*                      pHiDeltaTime_p
                                                            ,uint32_t*                      pLoDeltaTime_p);

#if (defined __cplusplus)
}
#endif

#endif /* __HWAL_H__ */
