/*!
* \file FBTL_sys.h
*
* \brief
* FBTL system access interface.
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

#if !(defined __FBTL_SYS_H__)
#define __FBTL_SYS_H__		1

#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define FBTLUNREF_PARM(x) \
    (void)(x);

#if (defined __GNUC__) || (defined __ti__)
#define FBTL_STRUCT_PACKED   __attribute__((__packed__))
#else
#define FBTL_STRUCT_PACKED
#endif

#define FBTL_WAIT_INFINITE      (UINT32_MAX)
#define FBTL_WAIT_IDLE          ((uint32_t)(100u))

/**
    \brief System interface communication underlay Enum

    This enumeration is used to identify the type of system underlay.

    \ingroup FBTL_SYS
*/
typedef enum FBTL_SYS_EUnderlayIf
{
    /**
    \brief uninitialized type

    Unknown underlay
    */
    SYS_eUIF_unknown            =  0,

    /**
    \brief RAM like underlay

    e.g. DPR, Shared RAM, GPMC
    */
    SYS_eUIF_ram                = 1,

    /**
    \brief Line bound underlay

    e.g. SPI, I2C, UART
    */
    SYS_eUIF_line               = 2,

    /**
    \brief Stream bound underlay

    Packet oriented underlay, e.g. EtherNET, CAN
    */
    SYS_eUIF_stream             = 3,

    /// @cond INTERNAL
    SYS_eUIF_32BIT              = 0xffffffff
    /// @endcond
} FBTL_SYS_EUnderlayIf_t;

/*!
    \brief System interface error code Enum

    This enumeration is used to identify errors within SYS underlay implementation.

    \ingroup FBTL_SYS
*/
typedef enum SYS_IF_EError
{
    /**
    \brief No error
    */
    FBTL_eSYS_noError       = 0x00000000,

    /**
    \brief Cannot open timer clock file read.
    */
    FBTL_eSYS_noRdClock     = 0x7E000004,

    /**
    \brief Cannot open timer clock file write.
    */
    FBTL_eSYS_noWrClock     = 0x7E000005,

    /**
    \brief Underlay not ready
    */
    FBTL_eSYS_notReady      = 0x7E00000B,

    /**
    \brief Out of memory
    */
    FBTL_eSYS_noMemory      = 0x7E00000C,

    /**
    \brief Invalid parameter
    */
    FBTL_eSYS_invalidParm   = 0x7E000016,

    /**
    \brief Out of Range
    */
    FBTL_eSYS_outOfRange    = 0x7E000022,

    /**
    \brief Given Underlay not supported.
    */
    FBTL_eSYS_ulayMismatch  = 0x7E000033,

    /**
    \brief Function not implemented
    */
    FBTL_eSYS_noImplement   = 0x7E000030,

    /**
    \brief No valid clock.
    */
    FBTL_eSYS_invClock      = 0x7E000055,

    /**
    \brief Not transferred yet.
    */
    FBTL_eSYS_notTransferred = 0x7E000056,

    /**
    \brief Timed out.
    */
    FBTL_eSYS_timedOut      = 0x7E00006E,

    /**
    \brief Unkown error value
    */
    FBTL_eSYS_unknownErr    = 0x7EFFFFFF,
} SYS_IF_EError_t;

/*!
    \brief System interrupt direction

    \ingroup FBTL_SYS
*/
typedef enum FBTL_SYSLIB_EIrq
{
    FBTL_SL_IRQ_cpu2app     = 0x1000,                           ///!<   IRQ provider to remote
    FBTL_SL_IRQ_app2cpu     = 0x2000,                           ///!<   IRQ remote to provider
} FBTL_SYSLIB_EIrq_t;

/* Underlay */
typedef uint32_t        (*FBTL_SYS_underLaySize_t)          (void*                  pContext_p);
typedef volatile
        uint8_t*        (*FBTL_SYS_getUnderLay_t)           (void*                  pContext_p
                                                            ,uint32_t               offset_p
                                                            ,uint32_t               size_p
                                                            ,bool*                  pIsDirectAccess_p);
typedef uint32_t        (*FBTL_SYS_setUnderLay_t)           (void*                  pContext_p
                                                            ,volatile uint8_t*      pBuffer_p
                                                            ,uint32_t               offset_p
                                                            ,uint32_t               size_p);

/* Memory */
typedef uint8_t*        (*FBTL_SYS_malloc_t)                (void*                  pContext_p
                                                            ,uint32_t               size_p);
typedef void            (*FBTL_SYS_free_t)                  (void*                  pContext_p
                                                            ,uint8_t*               pPtr_p);
typedef void            (*FBTL_SYS_memset_t)                (void*                  pContext_p
                                                            ,uint8_t*               pPtr_p
                                                            ,uint8_t                value_p
                                                            ,uint32_t               num_p);
typedef void            (*FBTL_SYS_memcpy_t)                (void*                  pContext_p
                                                            ,uint8_t*               pDestination_p
                                                            ,const uint8_t*         pSource_p
                                                            ,uint32_t               num_p);

/* Interrupts */
typedef void*           (*FBTL_SYS_IRQ_create_t)            (void*                  pContext_p
                                                            ,bool                   outNotIn_p
                                                            ,FBTL_SYSLIB_EIrq_t     irqType_p);
typedef void            (*FBTL_SYS_IRQ_cbHookIsr_t)         (void*                  pContext_p);
typedef void            (*FBTL_SYS_IRQ_registerISR_t)       (void*                  pIrq_p
                                                            ,void*                  pIsrContext_p
                                                            ,FBTL_SYS_IRQ_cbHookIsr_t
                                                                                    cbIsrCallback_p);
typedef void            (*FBTL_SYS_IRQ_delete_t)            (void*                  pContext_p
                                                            ,void*                  pIrq_p);
typedef uint32_t        (*FBTL_SYS_IRQ_wait_t)              (void*                  pContext_p
                                                            ,void*                  pIrq_p
                                                            ,uint32_t               timeout_p);
typedef void            (*FBTL_SYS_IRQ_post_t)              (void*                  pContext_p
                                                            ,void*                  pIrq_p);
typedef void            (*FBTL_SYS_IRQ_handshake_t)         (void*                  pContext_p);
typedef bool            (*FBTL_SYS_IRQ_inISR_t)             (void*                  pContext_p);

/* Signals */
typedef void*           (*FBTL_SYS_SIG_create_t)            (void*                  pContext_p
                                                            ,uint8_t                spawn_p
                                                            ,char*                  pName_p);
typedef void            (*FBTL_SYS_SIG_delete_t)            (void*                  pContext_p
                                                            ,void*                  pSignal_p);
typedef uint32_t        (*FBTL_SYS_SIG_wait_t)              (void*                  pContext_p
                                                            ,void*                  pSignal_p
                                                            ,uint32_t               timeout_p);
typedef void            (*FBTL_SYS_SIG_post_t)              (void*                  pContext_p
                                                            ,void*                  pSignal_p);
typedef void*           (*FBTL_SYS_CSIG_create_t)           (void*                  pContext_p
                                                            ,uint32_t               countStart_p
                                                            ,uint32_t               countMax_p);

/* Mutex */
typedef void*           (*FBTL_SYS_MTX_create_t)            (void*                  pContext_p
                                                            ,uint8_t                spawn_p
                                                            ,char*                  pName_p);
typedef void            (*FBTL_SYS_MTX_delete_t)            (void*                  pContext_p
                                                            ,void*                  pSignal_p);
typedef uint32_t        (*FBTL_SYS_MTX_lock_t)              (void*                  pContext_p
                                                            ,void*                  pSignal_p
                                                            ,uint32_t               timeout_p);
typedef void            (*FBTL_SYS_MTX_unLock_t)            (void*                  pContext_p
                                                            ,void*                  pSignal_p);

typedef void*           (*FBTL_SYS_CS_create_t)             (void*                  pContext_p);
typedef void            (*FBTL_SYS_CS_delete_t)             (void*                  pContext_p
                                                            ,void*                  pCritSect_p);
typedef uint32_t        (*FBTL_SYS_CS_enter_t)              (void*                  pContext_p
                                                            ,void*                  pCritSect_p
                                                            ,uint32_t               timeout_p);
typedef void            (*FBTL_SYS_CS_leave_t)              (void*                  pContext_p
                                                            ,void*                  pCritSect_p);

/* Scheduler */
typedef void            (*FBTL_SYS_SCHED_yield_t)           (void*                  pContext_p);
typedef void            (*FBTL_SYS_SCHED_sleep_t)           (void*                  pContext_p
                                                            ,uint32_t               msTimeout_p);
typedef uint32_t        (*FBTL_SYS_SCHED_getThreadId_t)     (void*                  pContext_p);
typedef bool            (*FBTL_SYS_SCHED_isIdentThrdId_t)   (void*                  pContext_p
                                                            ,uint32_t               threadId);
typedef uint32_t        (*FBTL_SYS_getSysTickMs_t)          (void*                  pContext_p);
typedef uint32_t        (*FBTL_SYS_setCpuSpeed_t)           (void*                  pContext_p
                                                            ,uint32_t               frequency_p);
typedef uint32_t        (*FBTL_SYS_getCpuSpeed_t)           (void*                  pContext_p
                                                            ,uint32_t*              pFrequency_p);

typedef void            (*FBTL_SYS_printf_t)                (void*                  pContext_p
                                                            ,const char* __restrict pFormat_p
                                                            ,va_list                vaarg_p);

/*!
    \brief Underlay interface

    \ingroup FBTL_SYS
*/
typedef struct FBTL_SYSLIB_SUnderlay
{
    FBTL_SYS_EUnderlayIf_t              underLayType;           ///!<   Type of underlay
    FBTL_SYS_underLaySize_t             getUnderlaySize;        ///!<   get total virtual underlay size
    FBTL_SYS_getUnderLay_t              getUnderlayBuffer;      ///!<   get buffer chunk from underlay
    FBTL_SYS_setUnderLay_t              setUnderlayBuffer;      ///!<   set buffer chunk from underlay
} FBTL_SYSLIB_SUnderlay_t;

/*!
    \brief Underlay RAM parameters

    \ingroup FBTL_SYS
*/
typedef struct FBTL_SYSLIB_SRamUnterlay
{
    /* portType == RAM */
    volatile uint8_t*                   pVirtualRamBase;        ///!<   base of shared RAM
    uint32_t                            ramSize;                ///!<   size of shared RAM
    void*                               shmFileHdl;             ///!<   shared memory access file
    /* /portType == RAM */
} FBTL_SYSLIB_SRamUnterlay_t;

/*!
    \brief Dynamic memory functions

    \ingroup FBTL_SYS
*/
typedef struct FBTL_SYSLIB_SMemory
{
    FBTL_SYS_malloc_t                   mallocHeap;             ///!<   Heap allocation
    FBTL_SYS_free_t                     freeHeap;               ///!<   decommit allocated heap memory
    FBTL_SYS_memset_t                   memsetHeap;             ///!<   initialize heap memory
    FBTL_SYS_memcpy_t                   memcpyHeap;             ///!<   copy heap memory
} FBTL_SYSLIB_SMemory_t;

/*!
    \brief Interrupt implementation functions

    \ingroup FBTL_SYS
*/
typedef struct FBTL_SYSLIB_SIrq
{
    FBTL_SYS_IRQ_create_t               create;                 ///!<   Create Interrupt
    FBTL_SYS_IRQ_delete_t               delete;                 ///!<   Destroy Interrupt
    FBTL_SYS_IRQ_wait_t                 wait;                   ///!<   Wait for Interrupt
    FBTL_SYS_IRQ_post_t                 post;                   ///!<   Signal Interrupt
    FBTL_SYS_IRQ_handshake_t            handshake;              ///!<   Interrupt ready handshake
    FBTL_SYS_IRQ_registerISR_t          registerISR;            ///!<   Register ISR function
    FBTL_SYS_IRQ_inISR_t                detectInISR;            ///!<   Detection of ISR context
} FBTL_SYSLIB_SIrq_t;

/*!
    \brief Semaphore signal implementation functions

    \ingroup FBTL_SYS
*/
typedef struct FBTL_SYSLIB_SSignal
{
    FBTL_SYS_SIG_create_t               create;                 ///!<   Create Signal
    FBTL_SYS_SIG_delete_t               delete;                 ///!<   Destroy Signal
    FBTL_SYS_SIG_wait_t                 wait;                   ///!<   Wait for Signal
    FBTL_SYS_SIG_post_t                 post;                   ///!<   Post Signal

    FBTL_SYS_CSIG_create_t              createCount;            ///!<   Create counting signal
    FBTL_SYS_SIG_delete_t               deleteCount;            ///!<   Destroy counting signal
    FBTL_SYS_SIG_wait_t                 waitCount;              ///!<   Wait/decrement counting signal
    FBTL_SYS_SIG_post_t                 postCount;              ///!<   Post/increment counting signal
} FBTL_SYSLIB_SSignal_t;

/*!
    \brief Semaphore mutex implementation functions

    \ingroup FBTL_SYS
*/
typedef struct FBTL_SYSLIB_SMutex
{
    FBTL_SYS_MTX_create_t               create;                 ///!<   Create mutex
    FBTL_SYS_MTX_delete_t               delete;                 ///!<   Destroy mutex
    FBTL_SYS_MTX_lock_t                 lock;                   ///!<   Lock mutex
    FBTL_SYS_MTX_unLock_t               unLock;                 ///!<   Unlock mutex
} FBTL_SYSLIB_SMutex_t;

/*!
    \brief Critical section implementation functions

    \ingroup FBTL_SYS
*/
typedef struct FBTL_SYSLIB_SCritSect
{
    FBTL_SYS_CS_create_t                create;                 ///!<   Create critical section
    FBTL_SYS_CS_delete_t                delete;                 ///!<   Destroy critical section
    FBTL_SYS_CS_enter_t                 enter;                  ///!<   enter critical section
    FBTL_SYS_CS_leave_t                 leave;                  ///!<   leave critical section
} FBTL_SYSLIB_SCritSect_t;

/*!
    \brief Scheduler related implementation functions

    \ingroup FBTL_SYS
*/
typedef struct FBTL_SYSLIB_SScheduler
{
    FBTL_SYS_SCHED_yield_t          yield;                  ///!<   Task yield
    FBTL_SYS_SCHED_sleep_t          sleep;                  ///!<   Task sleep
    FBTL_SYS_SCHED_getThreadId_t    getThreadId;            ///!<   Get Thread Id
    FBTL_SYS_SCHED_isIdentThrdId_t  isThreadIdMatch;        ///!<   Compare Thread Id to current
    FBTL_SYS_getSysTickMs_t         getSysTickMs;           ///!<   Get Sys Ticks / Time
    FBTL_SYS_setCpuSpeed_t          setCpuSpeed;            ///!<   Set CPU speed
    FBTL_SYS_getCpuSpeed_t          getCpuSpeed;            ///!<   Get CPU speed
} FBTL_SYSLIB_SScheduler_t;

/*!
    \brief Threading handling

    \ingroup FBTL_SYS
*/
typedef struct FBTL_SYSLIB_SThreading
{
    bool                            stopThread;             ///!<   on true, terminate threads
    void*                           pFbtlHandle;            ///!<   FBTL handle for threads


    void*                           fbtlRunThread;          ///!<   Cyclic thread of FBTL
    void*                           fbtlSyncThread;         ///!<   Sync execution thread of FBTL
    void*                           fbtlAcycISTThread;      ///!<   Acyclic IST thread
    void*                           fbtlRecvThread;         ///!<   Receiver thread of FBTL
    void*                           fbtlSendAcycThread;     ///!<   Acyc send thread of FBTL
    void*                           fbtlServiceThread;      ///!<   Service handler thread of FBTL
    void*                           fbtlSlowServiceThread;  ///!<   Slow Service handler thread of FBTL for async calls thtat block
    void*                           fbtlSyncIstThread;      ///!<   Sync thread for synchronous application call
    void*                           fbtlLedIstThread;       ///!<   Status LED thread
} FBTL_SYSLIB_SThreading_t;

/*!
    \brief System implementation interface handle

    \ingroup FBTL_SYS
*/
typedef struct FBTL_SYSLIB_SHandle
{
    void*                               pLibContext;            ///!<   Library context
    bool                                verbose;                ///!<   Trace output enable

    FBTL_SYSLIB_SUnderlay_t             underlay;               ///!<   Underlay depending functions and values
    FBTL_SYSLIB_SRamUnterlay_t          ramUnderLay;            ///!<   RAM depending functions and values
    FBTL_SYSLIB_SMemory_t               heap;                   ///!<   Heap control functions
    FBTL_SYSLIB_SIrq_t                  irqHandler;             ///!<   Interrupt handling functions
    FBTL_SYSLIB_SSignal_t               signal;                 ///!<   Signal handling functions
    FBTL_SYSLIB_SMutex_t                mutex;                  ///!<   Mutex handling functions
    FBTL_SYSLIB_SCritSect_t             critSect;               ///!<   critical section handling functions
    FBTL_SYSLIB_SScheduler_t            scheduler;              ///!<   scheduler functions
    FBTL_SYSLIB_SThreading_t            threading;              ///!<   FBTL required threads

    FBTL_SYS_printf_t                   outPrintf;              ///!<   Printf implementation
} FBTL_SYSLIB_SHandle_t;

#if (defined __cplusplus)
extern "C" {
#endif

extern uint32_t                 FBTL_SYS_checkLibrary   (void*                  pSysLibHandle_p);

extern void                     FBTL_SYS_printf         (void*                  pSysLibHandle_p
                                                        ,const char* __restrict pFormat_p
                                                        ,...);

/* Underlay */
extern FBTL_SYS_EUnderlayIf_t   FBTL_SYS_getULType      (void*                  pSysLibHandle_p);
extern uint32_t                 FBTL_SYS_getUnderLaySize(void*                  pSysLibHandle_p);
extern volatile uint8_t*        FBTL_SYS_getUnderLay    (void*                  pSysLibHandle_p
                                                        ,uint32_t               offset_p
                                                        ,uint32_t               size_p
                                                        ,bool*                  pIsDirectAccess_p);
extern uint32_t                 FBTL_SYS_setUnderLay    (void*                  pSysLibHandle_p
                                                        ,volatile uint8_t*      pBuffer_p
                                                        ,uint32_t               offset_p
                                                        ,uint32_t               size_p);

/* memory */
extern uint8_t*                 FBTL_SYS_malloc         (void*                  pSysLibHandle_p
                                                        ,uint32_t               length_p);
extern void                     FBTL_SYS_free           (void*                  pSysLibHandle_p
                                                        ,uint8_t*               pData_p);
extern void                     FBTL_SYS_memset         (void*                  pSysLibHandle_p
                                                        ,uint8_t*               pPtr_p
                                                        ,uint8_t                value_p
                                                        ,uint32_t               num_p);
extern void                     FBTL_SYS_memcpy         (void*                  pSysLibHandle_p
                                                        ,uint8_t*               pDestination_p
                                                        ,const uint8_t*         pSource_p
                                                        ,uint32_t               num_p);

/* Interrupts */
extern void*                    FBTL_SYS_IRQ_create     (void*                  pSysLibHandle_p
                                                        ,bool                   outNotIn_p
                                                        ,FBTL_SYSLIB_EIrq_t     irqType_p);
extern void                     FBTL_SYS_IRQ_delete     (void*                  pSysLibHandle_p
                                                        ,void*                  pIrq_p);
extern uint32_t                 FBTL_SYS_IRQ_wait       (void*                  pSysLibHandle_p
                                                        ,void*                  pIrq_p
                                                        ,uint32_t               timeout_p);
extern void                     FBTL_SYS_IRQ_post       (void*                  pSysLibHandle_p
                                                        ,void*                  pIrq_p);
extern void                     FBTL_SYS_IRQ_handshake  (void*                  pSysLibHandle_p);
extern bool                     FBTL_SYS_IRQ_inIsr      (void*                  pSysLibHandle_p);
extern void                     FBTL_SYS_IRQ_register   (void*                  pSysLibHandle_p
                                                        ,void*                  pIrq_p
                                                        ,void*                  pIsrContext_p
                                                        ,FBTL_SYS_IRQ_cbHookIsr_t
                                                                                cbIsr_p);



/* Signals */
extern void*                    FBTL_SYS_SIG_create     (void*                  pSysLibHandle_p
                                                        ,uint8_t                spawn_p
                                                        ,char*                  pName_p);
extern void                     FBTL_SYS_SIG_delete     (void*                  pSysLibHandle_p
                                                        ,void*                  pSignal_p);
extern uint32_t                 FBTL_SYS_SIG_wait       (void*                  pSysLibHandle_p
                                                        ,void*                  pSignal_p
                                                        ,uint32_t               timeout_p);
extern void                     FBTL_SYS_SIG_post       (void*                  pSysLibHandle_p
                                                        ,void*                  pSignal_p);

extern void*                    FBTL_SYS_CSIG_create    (void*                  pSysLibHandle_p
                                                        ,uint32_t               countStart_p
                                                        ,uint32_t               countMax_p);
extern void                     FBTL_SYS_CSIG_delete    (void*                  pSysLibHandle_p
                                                        ,void*                  pSignal_p);
extern uint32_t                 FBTL_SYS_CSIG_wait      (void*                  pSysLibHandle_p
                                                        ,void*                  pSignal_p
                                                        ,uint32_t               timeout_p);
extern void                     FBTL_SYS_CSIG_post      (void*                  pSysLibHandle_p
                                                        ,void*                  pSignal_p);

/* Mutex */
extern void*                    FBTL_SYS_MTX_create     (void*                  pSysLibHandle_p
                                                        ,uint8_t                spawn_p
                                                        ,char*                  pName_p);
extern void                     FBTL_SYS_MTX_delete     (void*                  pSysLibHandle_p
                                                        ,void*                  pMutex_p);
extern uint32_t                 FBTL_SYS_MTX_lock       (void*                  pSysLibHandle_p
                                                        ,void*                  pMutex_p
                                                        ,uint32_t               timeout_p);
extern void                     FBTL_SYS_MTX_unLock     (void*                  pSysLibHandle_p
                                                        ,void*                  pMutex_p);

/* Cirtical section */
extern void*                    FBTL_SYS_CS_create      (void*                  pSysLibHandle_p);
extern void                     FBTL_SYS_CS_delete      (void*                  pSysLibHandle_p
                                                        ,void*                  pCritSect_p);
extern uint32_t                 FBTL_SYS_CS_enter       (void*                  pSysLibHandle_p
                                                        ,void*                  pCritSect_p
                                                        ,uint32_t               timeout_p);
extern void                     FBTL_SYS_CS_leave       (void*                  pSysLibHandle_p
                                                        ,void*                  pCritSect_p);

/* Scheduler */
extern uint32_t                 FBTL_SYS_getMsTick      (void*                  pSysLibHandle_p);

extern uint32_t                 FBTL_SYS_setCpuSpeed    (void*                  pSysLibHandle_p
                                                        ,uint32_t               frequency_p);
extern uint32_t                 FBTL_SYS_getCpuSpeed    (void*                  pSysLibHandle_p
                                                        ,uint32_t*              pFrequency_p);
extern void                     FBTL_SYS_yield          (void*                  pSysLibHandle_p);
extern void                     FBTL_SYS_sleep          (void*                  pSysLibHandle_p
                                                        ,uint32_t               msTimeout_p);
extern uint32_t                 FBTL_SYS_getThreadId    (void*                  pSysLibHandle_p);
extern bool                     FBTL_SYS_isIdentThrdId  (void*                  pSysLibHandle_p
                                                        ,uint32_t               threadId_p);
extern bool                     FBTL_SYS_isVerbose      (void*                  pSysLibHandle_p);

#if (defined __cplusplus)
}
#endif

#endif /* __FBTL_SYS_H__ */
