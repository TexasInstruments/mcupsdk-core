/*!
* \file osal.h
*
* \brief
* OS abstraction layer (generic) interface.
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

#if !(defined PROTECT_OSAL_H)
#define PROTECT_OSAL_H  1

/* >= C99 required */
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>

#include <osal_error.h>

#if defined _WIN32 || defined __CYGWIN__
#define OSAL_DLL_IMPORT __declspec(dllimport)
#define OSAL_DLL_EXPORT __declspec(dllexport)
#define OSAL_DLL_LOCAL
#else
#if __GNUC__ >= 4
#define OSAL_DLL_IMPORT __attribute__ ((visibility ("default")))
#define OSAL_DLL_EXPORT __attribute__ ((visibility ("default")))
#define OSAL_DLL_LOCAL  __attribute__ ((visibility ("hidden")))
#else
#define OSAL_DLL_IMPORT
#define OSAL_DLL_EXPORT
#define OSAL_DLL_LOCAL
#endif
#endif

#if ((defined ECATSLAVE_SO) && (ECATSLAVE_SO==1) || ((defined ETHERNETIP_SO) && (ETHERNETIP_SO==1)) || ((defined PROFINETIO_SO) && (PROFINETIO_SO==1))) // defined if ECATSLV is compiled as a DLL
#if (defined OSALAPI_EXPORTS) || (defined ECSLVAPI_EXPORTS) // defined if we are building the ECATSLAVE DLL (instead of using it)
#define OSAL_API OSAL_DLL_EXPORT
#else
#define OSAL_API OSAL_DLL_IMPORT
#endif // ECSLVAPI_EXPORTS
#define OSAL_LOC OSAL_DLL_LOCAL
#else // ECATSLAVE_SO is not defined: this means ECATSLAVE is a static lib.
#define OSAL_API
#define OSAL_LOC
#endif // ECATSLAVE_SO


#if (defined __GNUC__) || (defined __TI_ARM__)
#define OSAL_STRUCT_PACKED  __attribute__((__packed__))
#define OSAL_FUNC_NORETURN  __attribute__((noreturn))
#define OSAL_FUNC_UNUSED    __attribute__((unused))
#define OSAL_FUNC_NOINLINE  __attribute__((noinline))

#elif defined (MSVCC)
#else
#ifndef __attribute__
    /* @cppcheck_justify{misra-c2012-21.1} some compilers do not provide the attribute macro */
    //cppcheck-suppress misra-c2012-21.1
    #define __attribute__(x)
#endif
#define OSAL_STRUCT_PACKED
#define OSAL_FUNC_NORETURN  __attribute__((noreturn))
#define OSAL_FUNC_UNUSED    __attribute__((unused))
#define OSAL_FUNC_NOINLINE  __attribute__((noinline))
#endif

#define OSAL_WAIT_INFINITE              (UINT32_MAX)

#define OSALUNREF_PARM(x) \
    (void)(x)

#if (defined __GNUC__) || (defined __TI_ARM__)
#define OSAL_ASSERT_DMB()               __asm("    dmb")
#define OSAL_ASSERT_DSB()               __asm("    dsb")
#else
#define OSAL_ASSERT_DMB()
#define OSAL_ASSERT_DSB()
#endif


#define OSAL_TICKS_IN_MILLI_SEC         1               /* 1000us tick */

#define UINT32REGISTERGET(base, offset) \
    (((volatile uint32_t*)&(((volatile uint8_t*)(base))[(offset)]))[0]);

#define UINT32REGISTERSET(base, offset, value) \
    ((volatile uint32_t*)&(((volatile uint8_t*)(base))[(offset)]))[0] = (value);

#define OSAL_ERROR_CLASS(x)     (0x39000000|((x)&0xffff))
#define OSAL_OS_ERROR_CLASS(x)  (0x39010000|((x)&0xffff))

/*!
 *  \brief
 *  OSAL error codes.
 *  \ingroup OSALAPI
 */
typedef enum OSAL_EError
{
    OSAL_eERR_NOERROR               = 0,                        /*!< No error, everything is fine. */
    OSAL_eERR_ENOENT                = OSAL_ERROR_CLASS(2),      /*!< No such file or directory. */
    OSAL_eERR_ENXIO                 = OSAL_ERROR_CLASS(6),      /*!< No such device or address. */
    OSAL_eERR_NOMEMORY              = OSAL_ERROR_CLASS(12),     /*!< Not enough memory. */
    OSAL_eERR_NOACCESS              = OSAL_ERROR_CLASS(13),     /*!< No access, no hardware license */
    OSAL_eERR_INVALIDSTATE          = OSAL_ERROR_CLASS(14),     /*!< Invalid state. */
    OSAL_eERR_EINVAL                = OSAL_ERROR_CLASS(22),     /*!< Invalid argument. */
    OSAL_eERR_NOTIMPLEMENTED        = OSAL_ERROR_CLASS(34),     /*!< Not implemeented. */
    OSAL_eERR_TIMEDOUT              = OSAL_ERROR_CLASS(110),    /*!< Operation or connection timed out. */
    OSAL_eERR_MUTEXINCONSISTENT     = OSAL_ERROR_CLASS(205),    /*!< Mutex incionsistent */
} OSAL_EError_t;

typedef enum OSAL_TASK_EPriority
{
    OSAL_TASK_ePRIO_0                   = 0,    /* Lowest Priority = idle */
    OSAL_TASK_ePRIO_1,
    OSAL_TASK_ePRIO_2,
    OSAL_TASK_ePRIO_3,
    OSAL_TASK_ePRIO_4,
    OSAL_TASK_ePRIO_5,
    OSAL_TASK_ePRIO_6,
    OSAL_TASK_ePRIO_7,
    OSAL_TASK_ePRIO_8,
    OSAL_TASK_ePRIO_9,
    OSAL_TASK_ePRIO_10,
    OSAL_TASK_ePRIO_11,
    OSAL_TASK_ePRIO_12,
    OSAL_TASK_ePRIO_13,
    OSAL_TASK_ePRIO_14,
    OSAL_TASK_ePRIO_15,
    OSAL_TASK_ePRIO_16,
    OSAL_TASK_ePRIO_17,
    OSAL_TASK_ePRIO_18,
    OSAL_TASK_ePRIO_19,
    OSAL_TASK_ePRIO_20,
    OSAL_TASK_ePRIO_21,
    OSAL_TASK_ePRIO_22,
    OSAL_TASK_ePRIO_23,
    OSAL_TASK_ePRIO_24,
    OSAL_TASK_ePRIO_25,
    OSAL_TASK_ePRIO_26,
    OSAL_TASK_ePRIO_27,
    OSAL_TASK_ePRIO_28,
    OSAL_TASK_ePRIO_29,
    OSAL_TASK_ePRIO_30,
    OSAL_TASK_ePRIO_31,                 /* Highest (RT) priority */

    /// @cond INTERNAL
    OSAL_TASK_ePRIO_Force32Bit          = 0xffffffff  /* 32Bit */
    /// @endcond
} OSAL_TASK_EPriority_t;

/* Generic */                                                       /* Linux    TiRTOS  FreeRTOS    */
#define OSAL_TASK_ePRIO_Lowest          (OSAL_TASK_ePRIO_2)         /* 26       2       1           */
#define OSAL_TASK_ePRIO_Idle            (OSAL_TASK_ePRIO_Lowest)    /* 26       2       1           */
#define OSAL_TASK_ePRIO_Normal          (OSAL_TASK_ePRIO_8)         /* 38       8       4           */
#define OSAL_TASK_ePRIO_Highest         (OSAL_TASK_ePRIO_30)        /* 82       30      15          */
#define OSAL_TASK_ePRIO_InterruptSub1   (OSAL_TASK_ePRIO_29)        /* 81       28      14          */
#define OSAL_TASK_ePRIO_Interrupt       (OSAL_TASK_ePRIO_Highest)   /* 82       30      15          */
#define OSAL_TASK_ePRIO_OSALTimer       (OSAL_TASK_ePRIO_21)        /* 64       21      10          */

/* HWAL */
#define OSAL_TASK_ePRIO_HWAL_WATCHDOG 	(OSAL_TASK_ePRIO_Interrupt)
#define OSAL_TASK_ePRIO_HWAL_LICENSE 	(OSAL_TASK_ePRIO_Normal)

/* EtherCAT */
#define OSAL_TASK_ePRIO_ECEEPROM        (OSAL_TASK_ePRIO_4)         /* 30       4       2           */
#define OSAL_TASK_ePRIO_ECRestart       (OSAL_TASK_ePRIO_10)        /* 42       10      5           */
#define OSAL_TASK_ePRIO_ECReboot        (OSAL_TASK_ePRIO_11)        /* 44       11      5           */
#define OSAL_TASK_ePRIO_ECLED           (OSAL_TASK_ePRIO_8)         /* 38       8       4           */
#define OSAL_TASK_ePRIO_ECSync          (OSAL_TASK_ePRIO_16)        /* 54       16      8           */
#define OSAL_TASK_ePRIO_ECEoE           (OSAL_TASK_ePRIO_25)        /* 72       25      12          */
#define OSAL_TASK_ePRIO_ECPDI           (OSAL_TASK_ePRIO_16)        /* 54       16      8           */
#define OSAL_TASK_ePRIO_ECDemoThread    (OSAL_TASK_ePRIO_29)        /* 80       29      14          */

/* FBTL */
#define OSAL_TASK_ePRIO_FBTLSync        (OSAL_TASK_ePRIO_Interrupt)
#define OSAL_TASK_ePRIO_FBTLAcycIST     (OSAL_TASK_ePRIO_Interrupt)
#define OSAL_TASK_ePRIO_FBTLCyclic      (OSAL_TASK_ePRIO_InterruptSub1)
#define OSAL_TASK_ePRIO_FBTLSendAcyc    (OSAL_TASK_ePRIO_InterruptSub1)
#define OSAL_TASK_ePRIO_FBTLReceiver    (OSAL_TASK_ePRIO_InterruptSub1)
#define OSAL_TASK_ePRIO_FBTLService     (OSAL_TASK_ePRIO_Interrupt)
#define OSAL_TASK_ePRIO_FBTLSlowService (OSAL_TASK_ePRIO_Interrupt)
#define OSAL_TASK_ePRIO_FBTLOSISTCONT   (OSAL_TASK_ePRIO_InterruptSub1)

/* DPR */
#define OSAL_TASK_ePRIO_DPRIRQ          (OSAL_TASK_ePRIO_28)
#define OSAL_TASK_ePRIO_DPRRX           (OSAL_TASK_ePRIO_26)
#define OSAL_TASK_ePRIO_DPRTX           (OSAL_TASK_ePRIO_27)

/* IOL */
#define OSAL_TASK_ePRIO_IOL_LED         (OSAL_TASK_ePRIO_20)
#define OSAL_TASK_ePRIO_IOL_Power       (OSAL_TASK_ePRIO_13)
#define OSAL_TASK_ePRIO_IOL_SMI         (OSAL_TASK_ePRIO_22)
#define OSAL_TASK_ePRIO_IOL_Main        (OSAL_TASK_ePRIO_16)
#define OSAL_TASK_ePRIO_IOL_Port        (OSAL_TASK_ePRIO_29)

/* EtherNet/IP */
#define OSAL_TASK_ePRIO_EIP_MAIN          (OSAL_TASK_ePRIO_8)
#define OSAL_TASK_ePRIO_EIP_FLASH         (OSAL_TASK_ePRIO_8)
#define OSAL_TASK_ePRIO_EIP_EEPROM        (OSAL_TASK_ePRIO_8)
#define OSAL_TASK_ePRIO_EIP_CPULOAD       (OSAL_TASK_ePRIO_2)
#define OSAL_TASK_ePRIO_EIP_CYCLICIO      (OSAL_TASK_ePRIO_20)
#define OSAL_TASK_ePRIO_EIP_PACKET        (OSAL_TASK_ePRIO_20)
#define OSAL_TASK_ePRIO_EIP_STATISTIC     (OSAL_TASK_ePRIO_14)
#define OSAL_TASK_ePRIO_EIP_PHYMDIX       (OSAL_TASK_ePRIO_12)
#define OSAL_TASK_ePRIO_EIP_TIMESYNC_DEL  (OSAL_TASK_ePRIO_20)
#define OSAL_TASK_ePRIO_EIP_TIMESYNC_TS   (OSAL_TASK_ePRIO_20)
#define OSAL_TASK_ePRIO_EIP_TIMESYNC_NRT  (OSAL_TASK_ePRIO_16)
#define OSAL_TASK_ePRIO_EIP_TIMESYNC_BAC  (OSAL_TASK_ePRIO_14)
#define OSAL_TASK_ePRIO_EIP_LWIP_TCPIP    (OSAL_TASK_ePRIO_14)
#define OSAL_TASK_ePRIO_EIP_WEBSERVER     (OSAL_TASK_ePRIO_2)

/*!
 *  \brief
 *  OSAL memory management configuration.
 *  \ingroup OSALAPI
 */
typedef enum OSAL_EMemoryManager
{
    OSAL_eMEMMAN_default    = 0,        //!< default mem manager = dynamic
    OSAL_eMEMMAN_dynamic    = 1,        //!< memory management is done through libc calloc/free
    OSAL_eMEMMAN_prealloc   = 2,        //!< libc calloc is used to prealloc buffer, memory management is done in osal
    OSAL_eMEMMAN_static     = 3,        //!< memory management on given static heap buffer, memory management is done in osal
} OSAL_EMemoryManager_t;

typedef struct OSAL_MEMMAN_SConfigParameter
{
    size_t                              size_p;     //!< size of managed memory,  if OSAL_EMemoryManager_t != OSAL_eMEMMAN_default: size of memory available
    void*                               pMem_p;     //!< pointer to given memory, if OSAL_EMemoryManager_t == OSAL_eMEMMAN_static:  pointer to managed memory
} OSAL_MEMMAN_SConfigParameter;

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Interrupt Handler Callback function
 *
 *  \details
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  hwIrq_p     Hardware Interrupt Number
 *  \param[in]  pCtxt_p     Interrupt Context
 *  \return     statusCode: 0:Interrupt worked
 *
 *  <!-- Example: -->
 *
 *  <!-- References: -->
 *
 *  \sa OSAL_RegisterIrqHandler
 *
 *  <!-- Group: -->
 *
 *  \ingroup OSALAPI
 *
 */
typedef uint32_t(*OSAL_IRQ_CBHandler_t)(uint32_t hwIrq_p, void* pCtxt_p);

typedef void (*OSAL_ERR_CBHandler_t)(
     uint32_t                       errorCode_p        //!< [in] Error code
    ,bool                           fatal_p            //!< [in] Is Error fatal
    ,uint8_t                        paraCnt_p          //!< [in] parameter counter
    ,va_list                        argptr_p           //!< [in] Error arguments
    );

typedef void (*OSAL_SCHED_CBTask_t)(void* pArg_p);

typedef enum OSAL_ELinkState
{
    OSAL_eLINK_DOWN,
    OSAL_eLINK_UP,
    /// @cond INTERNAL
    OSAL_eLINK_FORCE_32BIT = 0xffffffff
    /// @endcond
} OSAL_ELinkState_t;

typedef enum OSAL_EMemoryError
{
    OSAL_eMEM_CALLOC,
    OSAL_eMEM_FREE,
    OSAL_eMEM_MEMSET,
    OSAL_eMEM_MEMCPY,
    OSAL_eMEM_MEMCMP,
    /// @cond INTERNAL
    OSAL_eLMEM_FORCE_32BIT = 0xffffffff
    /// @endcond
} OSAL_EMemoryError_t;

#define OSAL_OS_START_TASK_FLG_NONE          0x00000000
#define OSAL_OS_START_TASK_FLG_FPU           0x00000001          // Task uses floating point processor

typedef void (*OSAL_CMDQUEUE_CBEvent_t)(void* pA0, void* pA1);

typedef struct OSAL_CMDQUEUE_SCommand
{
    OSAL_CMDQUEUE_CBEvent_t         cbEvent;        //!< Function, called for processing the event
    void*                           pThis;          //!< Instance data for the Event
    void*                           pArg;           //!< Argument structure for the Event
} OSAL_CMDQUEUE_SCommand_t;

typedef struct OSAL_SCHED_SEventHandle
{
    uint16_t                        dummy;          //!< to prevent warnings/errors about an empty struct
} OSAL_SCHED_SEventHandle_t;

struct OSAL_CMDQUEUE_SHandle;

typedef struct OSAL_TIMER_SHandle
{
    OSAL_CMDQUEUE_SCommand_t        cmd;
    struct OSAL_CMDQUEUE_SHandle*   pCmdQueue;
    uint8_t                         singleShot;

    //------------- Additional fields for OSAL only usage --------------------------
    char                            aName[0x20];
} OSAL_TIMER_SHandle_t;

typedef struct OSAL_SCHED_SMutexHandle
{
    uint16_t                        dummy;           // to prevent warnings/errors about an empty struct
} OSAL_SCHED_SMutexHandle_t;

typedef struct OSAL_SCHED_SSignalHandle
{
    uint16_t                        dummy;           // to prevent warnings/errors about an empty struct
} OSAL_SCHED_SSignalHandle_t;

typedef struct OSAL_CMDQUEUE_SHandle
{
    uint16_t                        dummy;           // to prevent warnings/errors about an empty struct

    //------------- Additional fields for OSAL only usage --------------------------
    char                            aName[0x20];
} OSAL_CMDQUEUE_SHandle_t;

typedef long long OSAL_PJumpBuf_t[64] __attribute__((__aligned__ (8)));

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Callback to printout OSAL_printf occurencies
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p      call context
 *  \param[in]  pFormat_p       Format string.
 *  \param[in]  arg_p           Parameter list.
 *
 *  <!-- Group: -->
 *
 *  \ingroup OSALAPI
 *
 * */
typedef void (*OSAL_printfOut_t)(void*                          pContext_p
                                ,const char* __restrict         pFormat_p
                                 /* @cppcheck_justify{misra-c2012-17.1} dynamic printf is only possible with use of stdarg */
                                 //cppcheck-suppress misra-c2012-17.1
                                ,va_list                        arg_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Calloc trace, called after memory allocation to show memory usage
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p  Call context.
 *  \param[in]  nmemb_p     number of allocated elements
 *  \param[in]  size_p      size of single allocated element
 *  \param[in]  pPtr_p      pointer to allocated memory
 *
 *  \note can be provided by the application layer, and registered with \ref OSAL_MEMORY_traceCallocRegister
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <osal.h>
 *
 *  // required variables
 *  void* pvVariable = NULL;
 *
 *  // the call
 *  OSAL_MEMORY_traceCalloc(NULL, 123, 456, pvVariable);
 *  \endcode
 *
 *  <!-- References: -->
 *
 *  \sa OSAL_MEMORY_traceCallocRegister OSAL_MEMORY_traceFreeRegister OSAL_MEMORY_errorRegister
 *
 *  <!-- Group: -->
 *
 *  \ingroup OSALAPI_MEMORY
 *
 * */
typedef void (*OSAL_MEMORY_traceCallocCB_t)(
                                 void*                          pContext_p
                                ,size_t                         nmemb_p
                                ,size_t                         size_p
                                ,void*                          pPtr_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  free trace, called before memory decommission to show memory usage
 *
 *  \note can be provided by the application layer, and registered with \ref OSAL_MEMORY_traceFreeRegister
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p  Call context.
 *  \param[in]  pPtr_p      pointer to memory that is decommitted
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <osal.h>
 *
 *  // required variables
 *  void* pvVariable = NULL;
 *
 *  // the call
 *  OSAL_MEMORY_traceCalloc(NULL, 123, 456, pvVariable);
 *
 *  OSAL_MEMORY_traceFree(NULL, pvVariable);
 *
 *  \endcode
 *
 *  <!-- References: -->
 *
 *  \sa OSAL_MEMORY_traceCallocRegister OSAL_MEMORY_traceFreeRegister OSAL_MEMORY_errorRegister
 *
 *  <!-- Group: -->
 *
 *  \ingroup OSALAPI_MEMORY
 *
 * */
typedef void (*OSAL_MEMORY_traceFreeCB_t)(
                                 void*                          pContext_p
                                ,void*                          pPtr_p);

/*!
 *  <!-- Description: -->
 *
 *  \brief
 *  Hook to be called on memory operation error in OSAL
 *
 *  \details
 *  The error callback is called in different situations with different parameters. Not all parameters are
 *  necessarily valid at the same time.
 *
 *  \note can be provided by the application layer, and registered with \ref OSAL_MEMORY_errorRegister
 *
 *  <table>
 *  <tr><td><b>errType_p</b></td><td><b>Valid parameters</b></td><td><b>Description</b></td></tr>
 *  <tr><td>OSAL_eMEM_CALLOC</td><td>size_p - size of requested memory</td><td>Calloc did return NULL</td></tr>
 *  <tr><td>OSAL_eMEM_FREE</td><td>pPtr1_p - pointer to buffer to be freed</td><td>Free did hit an error</td></tr>
 *  <tr><td>OSAL_eMEM_MEMSET</td><td>pPtr1_p - destination pointer<br>data_p - value to be set<br>size_p - set width (in bytes)</td><td>memset did hit a error</td></tr>
 *  <tr><td>OSAL_eMEM_MEMCPY</td><td>pPtr1_p - destination pointer<br>pPtr2_p - source pointer<br>size_p - copy length</td><td>Memcpy did hit an error</td></tr>
 *  <tr><td>OSAL_eMEM_MEMCMP</td><td>pPtr1_p - reference pointer<br>pPtr2_p - comparison pointer<br>size_p - comparison length</td><td>Memcmp did hit an error</td></tr>
 *  </table>
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pContext_p  Call context.
 *  \param[in]  errType_p   Type of error
 *  \param[in]  pPtr1_p     pointer 1
 *  \param[in]  pPtr2_p     pointer 2
 *  \param[in]  data_p      fixed value
 *  \param[in]  size_p      size of working range
 *
 *  <!-- References: -->
 *
 *  \sa OSAL_MEMORY_traceCallocRegister OSAL_MEMORY_traceFreeRegister OSAL_MEMORY_errorRegister
 *
 *  <!-- Group: -->
 *
 *  \ingroup OSALAPI_MEMORY
 *
 * */
typedef void (*OSAL_MEMORY_errorCB_t)(
                                 void*                          pContext_p
                                ,OSAL_EMemoryError_t            errType_p
                                ,void*                          pPtr1_p
                                ,void*                          pPtr2_p
                                ,uint32_t                       data_p
                                ,size_t                         size_p);

#if (defined __cplusplus)
extern "C" {
#endif

/* OSAL */
extern OSAL_API uint32_t            OSAL_init                   (void);
extern OSAL_API void                OSAL_deinit                 (void);

extern OSAL_API uint32_t            OSAL_getVersion             (void);
extern OSAL_API uint32_t            OSAL_getVersionStr          (uint32_t                       bufLen_p
                                                                ,char*                          pBuffer_p
                                                                ,uint32_t*                      pUsedLen_p);
extern OSAL_API uint32_t            OSAL_getVersionId           (uint32_t                       bufLen_p
                                                                ,char*                          pBuffer_p
                                                                ,uint32_t*                      pUsedLen_p);

extern OSAL_API void                OSAL_setExceptionPoint      (OSAL_PJumpBuf_t*               pJumpBuf_p);
extern OSAL_API OSAL_PJumpBuf_t*    OSAL_getExceptionPoint      (void);
extern OSAL_API int32_t             OSAL_setJmp                 (OSAL_PJumpBuf_t*               pJumpBuf_p);
extern OSAL_API void                OSAL_longJmp                (OSAL_PJumpBuf_t*               pJumpBuf_p
                                                                ,int32_t                        value_p);

/* @cppcheck_justify{misra-c2012-2.7} va_list is false positive shown as unused */
//cppcheck-suppress misra-c2012-2.7
extern OSAL_API void                OSAL_error                  (const char*                    pFunction_p
                                                                ,uint32_t                       line_p
                                                                ,uint32_t                       errCode_p
                                                                ,bool                           fatal_p
                                                                ,uint32_t                        paraCnt_p
                                                                ,...);
extern OSAL_API void                OSAL_registerErrorHandler   (OSAL_ERR_CBHandler_t           cbErrHandler_p);

extern OSAL_API void                OSAL_closeHandles           (void);

extern OSAL_API void                OSAL_registerPrintOut       (void*                          pContext_p
                                                                ,OSAL_printfOut_t               cbFunc_p);

/* @cppcheck_justify{misra-c2012-17.1} va_list dynamic args is defined in stdarg.h */
//cppcheck-suppress misra-c2012-17.1
/* @cppcheck_justify{misra-c2012-2.7} va_list is false positive shown as unused */
//cppcheck-suppress misra-c2012-2.7
extern OSAL_API void                OSAL_printf                 (const char* __restrict         format_p
                                                                ,...);
extern OSAL_API void                OSAL_vprintf                (const char* __restrict         format_p
                                                                 /* @cppcheck_justify{misra-c2012-17.1} dynamic printf is only possible with use of stdarg */
                                                                 //cppcheck-suppress misra-c2012-17.1
                                                                ,va_list                        arg_p);
extern OSAL_API void                OSAL_printfSuppress         (bool suppress_p);

extern OSAL_API uint32_t            OSAL_getMsTick              (void);
extern OSAL_API void                OSAL_waitTimerUs            (uint32_t                       waitTime_p);
extern OSAL_API uint32_t            OSAL_randomEnable           (uint32_t                       seed_p);
extern OSAL_API uint32_t            OSAL_getRandomNumber        (void);

/* OSAL_SCHED */
extern OSAL_API void                OSAL_SCHED_sleep            (uint32_t                       duration_p);
extern OSAL_API void                OSAL_SCHED_sleepUs          (uint32_t                       durationUs_p);
extern OSAL_API void                OSAL_SCHED_yield            (void);
extern OSAL_API void                OSAL_SCHED_taskPrepare      (void);
extern OSAL_API void*               OSAL_SCHED_startTask        (OSAL_SCHED_CBTask_t            cbTask_p
                                                                ,void*                          pArg_p
                                                                ,OSAL_TASK_EPriority_t          prio_p
                                                                ,uint8_t*                       pStack_p
                                                                ,uint16_t                       stackSize_p
                                                                ,uint32_t                       flags_p
                                                                ,char*                          pTaskName_p);
extern OSAL_API uint32_t            OSAL_SCHED_joinTask         (void*                          pTask_p);
extern OSAL_API void                OSAL_SCHED_killTask         (void*                          pHandle_p);
extern OSAL_API void                OSAL_SCHED_exitTask         (void*                          pTask_p);
extern OSAL_API volatile void*      OSAL_SCHED_getOwnTcb        (void);
extern OSAL_API bool                OSAL_SCHED_isRunning        (void);

extern OSAL_API volatile OSAL_PJumpBuf_t*
                                    OSAL_getExceptPointFromTcb  (volatile void*                 pTcb_p);

extern OSAL_API void                OSAL_startOs                (void);

extern OSAL_API void                OSAL_EVT_init               (OSAL_SCHED_SEventHandle_t*     pEvent_p);
extern OSAL_API void                OSAL_EVT_set                (OSAL_SCHED_SEventHandle_t*     pEvent_p);
extern OSAL_API uint32_t            OSAL_EVT_wait               (OSAL_SCHED_SEventHandle_t*     pEvent_p
                                                                ,uint32_t                       timeOut_p
                                                                ,uint8_t*                       pIsTimeout_p);

extern OSAL_API void                OSAL_TIMER_init             (OSAL_TIMER_SHandle_t*          pTimer_p
                                                                ,char*                          pName_p);
extern OSAL_API void                OSAL_TIMER_set100usecTickSupport
                                                                (bool                           support_p);
extern OSAL_API void                OSAL_TIMER_start            (OSAL_TIMER_SHandle_t*          pTimer_p
                                                                ,uint32_t                       durationMs_p);
extern OSAL_API void                OSAL_TIMER_startUs          (OSAL_TIMER_SHandle_t*          pTimer_p
                                                                ,uint32_t                       durationUs_p);
extern OSAL_API void                OSAL_TIMER_stop             (OSAL_TIMER_SHandle_t*          pTimer_p);
extern OSAL_API void                OSAL_TIMER_expire           (OSAL_TIMER_SHandle_t*          pTimer_p);

extern OSAL_API void                OSAL_CMDQUEUE_init          (OSAL_CMDQUEUE_SHandle_t*       pQueue_p
                                                                ,char*                          pName_p);
extern OSAL_API uint32_t            OSAL_CMDQUEUE_put           (OSAL_CMDQUEUE_SHandle_t*       pQueue_p
                                                                ,OSAL_CMDQUEUE_CBEvent_t        cbEvent_p
                                                                ,void*                          pThis_p
                                                                ,void*                          pArg_p);
extern OSAL_API uint8_t             OSAL_CMDQUEUE_peek          (OSAL_CMDQUEUE_SHandle_t*       pQueue_p
                                                                ,OSAL_CMDQUEUE_CBEvent_t        cbEvent_p
                                                                ,void*                          pThis_p
                                                                ,void*                          pArg_p);
extern OSAL_API uint32_t            OSAL_CMDQUEUE_exec          (OSAL_CMDQUEUE_SHandle_t*       pQueue_p
                                                                ,uint32_t                       timeOut_p
                                                                ,uint8_t*                       pIsTimeout_p);
extern OSAL_API uint32_t            OSAL_CMDQUEUE_getFree       (OSAL_CMDQUEUE_SHandle_t*       pQueue_p);

extern OSAL_API void                OSAL_MTX_init               (OSAL_SCHED_SMutexHandle_t*     pMutex_p);
extern OSAL_API void                OSAL_MTX_deinit             (OSAL_SCHED_SMutexHandle_t*     pMutex_p);
extern OSAL_API void                OSAL_MTX_release            (OSAL_SCHED_SMutexHandle_t*     pMutex_p);
extern OSAL_API uint32_t            OSAL_MTX_get                (OSAL_SCHED_SMutexHandle_t*     pMutex_p
                                                                ,uint32_t                       timeOut_p
                                                                ,uint8_t*                       pIsTimeout_p);

extern OSAL_API OSAL_SCHED_SEventHandle_t*
                                    OSAL_EVTCTRLBLK_alloc       (void);
extern OSAL_API void                OSAL_EVTCTRLBLK_free        (OSAL_SCHED_SEventHandle_t*     pCtrlBlk_p);
extern OSAL_API OSAL_TIMER_SHandle_t*
                                    OSAL_TIMCTRLBLK_alloc       (void);
extern OSAL_API void                OSAL_TIMCTRLBLK_free        (OSAL_TIMER_SHandle_t*          pCtrlBlk_p);
extern OSAL_API OSAL_SCHED_SMutexHandle_t*
                                    OSAL_MTXCTRLBLK_alloc       (void);
extern OSAL_API void                OSAL_MTXCTRLBLK_free        (OSAL_SCHED_SMutexHandle_t*     pCtrlBlk_p);
extern OSAL_API OSAL_CMDQUEUE_SHandle_t*
                                    OSAL_CQUEUECTRLBLK_alloc    (void);
extern OSAL_API void                OSAL_CQUEUECTRLBLK_free     (OSAL_CMDQUEUE_SHandle_t*       pCtrlBlk_p);

extern OSAL_API uint32_t            OSAL_CRITSECT_enter         (void);
extern OSAL_API void                OSAL_CRITSECT_exit          (uint32_t                       prevState_p);

/* OSAL_IRQ */
extern OSAL_API uint32_t            OSAL_IRQ_registerHandler    (uint32_t                       hwIrq_p
                                                                ,void*                          pCtxt_p,
                                                                OSAL_IRQ_CBHandler_t            cbIrqHandler_p);
extern OSAL_API uint32_t            OSAL_IRQ_unRegisterHandler  (uint32_t                       hwIrq_p);
extern OSAL_API void                OSAL_IRQ_enable             (uint32_t                       hwIrq_p);
extern OSAL_API void                OSAL_IRQ_disable            (uint32_t                       hwIrq_p);
extern OSAL_API uint32_t            OSAL_IRQ_disableGlobalIrq   (void);
extern OSAL_API void                OSAL_IRQ_enableGlobalIrq    (void);
extern OSAL_API uint32_t            OSAL_IRQ_disableGlobalFiq   (void);
extern OSAL_API void                OSAL_IRQ_enableGlobalFiq    (void);
extern OSAL_API void                OSAL_IRQ_restoreCPSR        (uint32_t                       key_p);

/* OSAL Semaphore */
extern OSAL_API void*               OSAL_createNamedMutex       (const char*                    pName_p);
extern OSAL_API void*               OSAL_openNamedMutex         (const char*                    pName_p);
extern OSAL_API void*               OSAL_deleteNamedMutex       (void*                          pMutex_p);
extern OSAL_API int32_t             OSAL_lockNamedMutex         (void*                          pMutex_p
                                                                ,uint32_t                       msTimeout_p);
extern OSAL_API int32_t             OSAL_unLockNamedMutex       (void*                          pMutex_p);

extern OSAL_API void*               OSAL_CSMTX_create           (void);
extern OSAL_API void*               OSAL_CSMTX_delete           (void*                          pCritSect_p);
extern OSAL_API int32_t             OSAL_CSMTX_enter            (void*                          pCritSect_p
                                                                ,uint32_t                       msTimeout_p);
extern OSAL_API int32_t             OSAL_CSMTX_leave            (void*                          pCritsect_p);

extern OSAL_API void*               OSAL_createSignal           (const char*                    pName_p);
extern OSAL_API void*               OSAL_openSignal             (const char*                    pName_p);
extern OSAL_API void*               OSAL_deleteSignal           (void*                          pSignal_p);
extern OSAL_API int32_t             OSAL_waitSignal             (void*                          pSignal_p
                                                                ,uint32_t                       msTimeout_p);
extern OSAL_API int32_t             OSAL_postSignal             (void*                          pSignal_p);

extern OSAL_API void*               OSAL_createCountSignal      (uint32_t                       countStart_p
                                                                ,uint32_t                       countMax_p);
extern OSAL_API void*               OSAL_deleteCountSignal      (void*                          pSignal_p);
extern OSAL_API int32_t             OSAL_waitCountSignal        (void*                          pSignal_p
                                                                ,uint32_t                       msTimeout_p);
extern OSAL_API int32_t             OSAL_postCountSignal        (void*                          pSignal_p);

/* OSAL Memory */
extern OSAL_API void                OSAL_MEMORY_init            (void);
extern OSAL_API void                OSAL_MMAP_init              (void);
extern OSAL_API void*               OSAL_MMAP_create            (uint32_t                       physicalAddr_p
                                                                ,uint32_t                       size_p);
extern OSAL_API void                OSAL_MMAP_remove            (void*                          pVirtAddr_p
                                                                ,uint32_t                       size_p);
extern OSAL_API uint32_t            OSAL_MEMORY_config          (OSAL_EMemoryManager_t          memAlg_p
                                                                ,OSAL_MEMMAN_SConfigParameter*  pConfig_p);
extern OSAL_API void*               OSAL_MEMORY_calloc          (size_t                         nmemb_p
                                                                ,size_t                         size_p);
extern OSAL_API void                OSAL_MEMORY_free            (void*                          pPtr_p);
extern OSAL_API void                OSAL_MEMORY_memcpy          (void*                          pTarget_p
                                                                ,const void *                   pSource_p
                                                                ,size_t                         size_p);
extern OSAL_API void                OSAL_MEMORY_memset          (void*                          pTarget_p
                                                                ,int32_t                        value_p
                                                                ,size_t                         size_p);
extern OSAL_API int32_t             OSAL_MEMORY_memcmp          (void*                          pPtr1_p
                                                                ,void*                          pPtr2_p
                                                                ,size_t                         size_p);
extern OSAL_API void                OSAL_MEMORY_traceCallocRegister
                                                                (OSAL_MEMORY_traceCallocCB_t    cbFunc_p
                                                                ,void*                          pCbFuncContext_p);
extern OSAL_API void                OSAL_MEMORY_traceFreeRegister
                                                                (OSAL_MEMORY_traceFreeCB_t      cbFunc_p
                                                                ,void*                          pCbFuncContext_p);
extern OSAL_API void                OSAL_MEMORY_errorRegister   (OSAL_MEMORY_errorCB_t          cbFunc_p
                                                                ,void*                          pCbFuncContext_p);

extern OSAL_API uint32_t            OSAL_VNET_create            (char*                          pDev_p);
extern OSAL_API uint32_t            OSAL_VNET_remove            (char*                          pDev_p);
extern OSAL_API uint32_t            OSAL_VNET_open              (char*                          pDev_p
                                                                ,void**                         pHandle_p);
extern OSAL_API uint32_t            OSAL_VNET_configureMAC      (void*                          pHandle_p
                                                                ,const uint8_t                  aMac_p[6]);
extern OSAL_API uint32_t            OSAL_VNET_configureIPv4     (void*                          pHandle_p
                                                                ,const uint32_t                 ipAddress_p
                                                                ,const uint32_t                 mask_p);
extern OSAL_API uint32_t            OSAL_VNET_configureIPv4Route(void*                          pHandle_p
                                                                ,const uint32_t                 defGateway_p);
extern OSAL_API uint32_t            OSAL_VNET_close             (void*                          pHandle_p);
extern OSAL_API uint32_t            OSAL_VNET_rxWait            (void*                          pHandle_p
                                                                ,uint32_t                       mSecTimeout_p);
extern OSAL_API uint32_t            OSAL_VNET_readFrame         (void*                          pHandle_p
                                                                ,uint32_t                       bufSize_p
                                                                ,void*                          pBuffer_p
                                                                ,uint32_t*                      pUsed_p);
extern OSAL_API uint32_t            OSAL_VNET_writeFrame        (void*                          pHandle_p
                                                                ,uint32_t                       bufSize_p
                                                                ,void*                          pBuffer_p);
extern OSAL_API uint32_t            OSAL_VNET_setLinkState      (void*                          pHandle_p
                                                                ,OSAL_ELinkState_t              state_p);
extern OSAL_API uint32_t            OSAL_VNET_setIfFlags        (void*                          pHandle_p
                                                                ,int32_t                        flags_p);

extern OSAL_API uint32_t            OSAL_VNET_ntohl             (uint32_t                       net_p);
extern OSAL_API uint32_t            OSAL_VNET_htonl             (uint32_t                       host_p);

#if (defined __cplusplus)
}
#endif

#endif /* PROTECT_OSAL_H */
