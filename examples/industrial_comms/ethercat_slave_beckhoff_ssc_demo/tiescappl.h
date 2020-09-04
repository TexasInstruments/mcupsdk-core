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

#ifdef _TIESC_HW_
#define PROTO
#else
#define PROTO extern
#endif

#if TIESC_HW
#include <tieschw.h>

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/*-----------------------------------------------------------------------------------------
------
------    TI ESC application specific objects
------
-----------------------------------------------------------------------------------------*/
/******************************************************************************
*                   Object 0x1601: digital output RxPDO
******************************************************************************/
#ifdef _OBJD_
OBJCONST TSDOINFOENTRYDESC  OBJMEM asEntryDesc0x1601[] =
{
    {DEFTYPE_UNSIGNED8, 0x8, ACCESS_READ }, /* Subindex 000 */
    {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ}, /* SubIndex 001: SubIndex 001 */
};

OBJCONST UCHAR OBJMEM aName0x1601[] =
    "RxPDO-Map\000SubIndex 1\000\377";
#endif //#ifdef _OBJD_

typedef struct STRUCT_PACKED_START
{
    UINT16   u16SubIndex0;
    UINT32   aEntries[1];
} STRUCT_PACKED_END
TOBJ1601;

PROTO TOBJ1601 RxPDOMap
#ifdef _TIESC_HW_
    = {1, {0x70100108}}
#endif
      ;
/******************************************************************************
*                   Object 0x1602:  Motor output RxPDO
******************************************************************************/
#ifdef _OBJD_
OBJCONST TSDOINFOENTRYDESC  OBJMEM asEntryDesc0x1602[] =
{
    {DEFTYPE_UNSIGNED8, 0x8, ACCESS_READ }, /* Subindex 000 */
    {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ}, /* SubIndex 001: SubIndex 001 */
    {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ}, /* SubIndex 002: SubIndex 002 */
    {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ} /* SubIndex 003: SubIndex 003 */
};

OBJCONST UCHAR OBJMEM aName0x1602[] =
    "RxPDO1-Map\000SubIndex 1\000SubIndex 2\000SubIndex 3\000\377";
#endif //#ifdef _OBJD_

typedef struct STRUCT_PACKED_START
{
    UINT16   u16SubIndex0;
    UINT32   aEntries[3];
} STRUCT_PACKED_END
TOBJ1602;

PROTO TOBJ1602 RxPDO1Map
#ifdef _TIESC_HW_
    = {3, {0x70200108, 0x70200208, 0x70200310}}
#endif
      ;

/******************************************************************************
*                   Object 0x1A00: digital input TxPDO
******************************************************************************/
#ifdef _OBJD_
OBJCONST TSDOINFOENTRYDESC  OBJMEM asEntryDesc0x1A00[] =
{
    {DEFTYPE_UNSIGNED8, 0x8, ACCESS_READ }, /* Subindex 000 */
    {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ},
};

OBJCONST UCHAR OBJMEM aName0x1A00[] =
    "TxPDO-Map\000SubIndex 1\000\377";
#endif //#ifdef _OBJD_

typedef struct STRUCT_PACKED_START
{
    UINT16   u16SubIndex0;
    UINT32   aEntries[1];
} STRUCT_PACKED_END
TOBJ1A00;

PROTO TOBJ1A00 TxPDOMap
#ifdef _TIESC_HW_
    = {1, {0x60000108}}
#endif
      ;

/******************************************************************************
*                   Object 0x1A03: Motor analog input TxPDO
******************************************************************************/
#ifdef _OBJD_
OBJCONST TSDOINFOENTRYDESC  OBJMEM asEntryDesc0x1A03[] =
{
    {DEFTYPE_UNSIGNED8, 0x8, ACCESS_READ }, /* Subindex 000 */
    {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ}, /* SubIndex 001: SubIndex 001 */
    {DEFTYPE_UNSIGNED32, 0x20, ACCESS_READ} /* SubIndex 002: SubIndex 002 */
};

OBJCONST UCHAR OBJMEM aName0x1A03[] =
    "TxPDO1-Map\000SubIndex 1\000SubIndex 2\000\377";
#endif //#ifdef _OBJD_

typedef struct STRUCT_PACKED_START
{
    UINT16   u16SubIndex0;
    UINT32   aEntries[2];
} STRUCT_PACKED_END
TOBJ1A03;
PROTO TOBJ1A03 sAITxPDO1Map

#ifdef _TIESC_HW_
    = {2, {0x60300120, 0x60300210}}
#endif
      ;

/******************************************************************************
*                   Object 0x1C12: RxPDO assignment
******************************************************************************/
#ifdef _OBJD_
OBJCONST TSDOINFOENTRYDESC  OBJMEM asEntryDesc0x1C12[] =
{
    {DEFTYPE_UNSIGNED8, 0x08, ACCESS_READ | ACCESS_WRITE_PREOP},
    {DEFTYPE_UNSIGNED16, 0x10, ACCESS_READ | ACCESS_WRITE_PREOP},
    {DEFTYPE_UNSIGNED16, 0x10, ACCESS_READ | ACCESS_WRITE_PREOP}
};
OBJCONST UCHAR OBJMEM aName0x1C12[] = "RxPDO assign";
#endif //#ifdef _OBJD_

typedef struct STRUCT_PACKED_START
{
    UINT16   u16SubIndex0;
    UINT16   aEntries[2];
} STRUCT_PACKED_END
TOBJ1C12;

PROTO TOBJ1C12 sRxPDOassign
#ifdef _TIESC_HW_
    = {0x02, {0x1601, 0x1602}}
#endif
      ;

/******************************************************************************
*                   Object 0x1C13: TxPDO assignment
******************************************************************************/
#ifdef _OBJD_
OBJCONST TSDOINFOENTRYDESC  OBJMEM asEntryDesc0x1C13[] =
{
    {DEFTYPE_UNSIGNED8, 0x08, ACCESS_READ | ACCESS_WRITE_PREOP},
    {DEFTYPE_UNSIGNED16, 0x10, ACCESS_READ | ACCESS_WRITE_PREOP},
    {DEFTYPE_UNSIGNED16, 0x10, ACCESS_READ | ACCESS_WRITE_PREOP}
};
OBJCONST UCHAR OBJMEM aName0x1C13[] = "TxPDO assign";
#endif //#ifdef _OBJD_

typedef struct STRUCT_PACKED_START
{
    UINT16   u16SubIndex0;
    UINT16   aEntries[2];
} STRUCT_PACKED_END
TOBJ1C13;


PROTO TOBJ1C13 sTxPDOassign
#ifdef _TIESC_HW_
    = {0x02, {0x1A00, 0x1A03}}
#endif
      ;

/******************************************************************************
*                   Object 0x6000: digital input object
******************************************************************************/
#ifdef _OBJD_
OBJCONST TSDOINFOENTRYDESC  OBJMEM asEntryDesc0x6000[] =
{
    {DEFTYPE_UNSIGNED8, 0x8, ACCESS_READ }, /* Subindex 000 */
    {DEFTYPE_BITARR8, 0x08, ACCESS_READ | OBJACCESS_TXPDOMAPPING}, /* SubIndex 001: Switchs */
};


OBJCONST UCHAR OBJMEM aName0x6000[] =
    "DI Inputs\000Switch\000\377";
#endif //#ifdef _OBJD_

typedef struct STRUCT_PACKED_START
{
    UINT16   u16SubIndex0;
    UINT8    switchs;
} STRUCT_PACKED_END
TOBJ6000;

PROTO TOBJ6000 sDIInputs
#ifdef _TIESC_HW_
    = {1, 0x0}
#endif
      ;

/******************************************************************************
*                   Object 0x6030: analog input object
******************************************************************************/
#ifdef _OBJD_
OBJCONST TSDOINFOENTRYDESC  OBJMEM asEntryDesc0x6030[] =
{
    {DEFTYPE_UNSIGNED8, 0x8, ACCESS_READ }, /* Subindex 000 */
    {DEFTYPE_INTEGER32, 0x20, ACCESS_READ | OBJACCESS_TXPDOMAPPING}, /* SubIndex 001: Info 1 */
    {DEFTYPE_INTEGER16, 0x10, ACCESS_READ | OBJACCESS_TXPDOMAPPING}
};//, /* SubIndex 002: Info 2 */


OBJCONST UCHAR OBJMEM aName0x6030[] =
    "Motor AI Inputs\000Info 1\000Info 2\000\377";
#endif //#ifdef _OBJD_

typedef struct STRUCT_PACKED_START
{
    UINT16   u16SubIndex0;
    INT32   info1;
    INT16   info2;
} STRUCT_PACKED_END
TOBJ6030;

PROTO TOBJ6030 sAI1Inputs
#ifdef _TIESC_HW_
    = {2, 0x0000, 0x00}
#endif
      ;

/******************************************************************************
*                   Object 0x7010: digital output object
******************************************************************************/
#ifdef _OBJD_
OBJCONST TSDOINFOENTRYDESC  OBJMEM asEntryDesc0x7010 [] =
{
    {DEFTYPE_UNSIGNED8, 0x8, ACCESS_READ }, /* Subindex 000 */
    {DEFTYPE_BITARR8, 0x08, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING}, /* SubIndex 001: LED */
};
OBJCONST UCHAR OBJMEM aName0x7010[] =
    "DO Outputs\000LED\000\377";
#endif //#ifdef _OBJD_


typedef struct STRUCT_PACKED_START
{
    UINT16   u16SubIndex0;
    UINT8    LEDs;
} STRUCT_PACKED_END
TOBJ7010;

PROTO TOBJ7010 sDOOutputs
#ifdef _TIESC_HW_
    = {1, 0x0}
#endif
      ;
/******************************************************************************
*                   Object 0x7020: Motor output object
******************************************************************************/
#ifdef _OBJD_
OBJCONST TSDOINFOENTRYDESC  OBJMEM asEntryDesc0x7020 [] =
{
    {DEFTYPE_UNSIGNED8, 0x8, ACCESS_READ }, /* Subindex 000 */
    {DEFTYPE_UNSIGNED8, 0x08, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING}, /* SubIndex 001: motor count */
    {DEFTYPE_UNSIGNED8, 0x08, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING}, /* SubIndex 002: motor command */
    {DEFTYPE_INTEGER16, 0x10, ACCESS_READWRITE | OBJACCESS_RXPDOMAPPING}
}; /* SubIndex 003: motor data */
OBJCONST UCHAR OBJMEM aName0x7020[] =
    "Motor Outputs\000Count\000Command\000Data\000\377";
#endif //#ifdef _OBJD_


typedef struct STRUCT_PACKED_START
{
    UINT16   u16SubIndex0;
    UINT8    Count;
    UINT8    Cmd;
    INT16    MotorData;
} STRUCT_PACKED_END
TOBJ7020;

PROTO TOBJ7020 sDO1Outputs
#ifdef _TIESC_HW_
    = {3, 0x00, 0x00, 0x0000}
#endif
      ;
/************************************************************/

#ifdef _OBJD_
/*if _PIC18 is enabled the object dictionary is fixed defined in coeappl.c*/
TOBJECT OBJMEM ApplicationObjDic[] =
{
    /* Object 0x1601 */
    {NULL, NULL,  0x1601, {DEFTYPE_PDOMAPPING, 1 | (OBJCODE_REC << 8)}, asEntryDesc0x1601, aName0x1601, &RxPDOMap, NULL, NULL, 0x0000 },
    /* Object 0x1602 */
    {NULL, NULL,  0x1602, {DEFTYPE_PDOMAPPING, 3 | (OBJCODE_REC << 8)}, asEntryDesc0x1602, aName0x1602, &RxPDO1Map, NULL, NULL, 0x0000 },
    /* Object 0x1A00 */
    {NULL, NULL,   0x1A00, {DEFTYPE_PDOMAPPING, 1 | (OBJCODE_REC << 8)}, asEntryDesc0x1A00, aName0x1A00, &TxPDOMap, NULL, NULL, 0x0000 },
    /* Object 0x1A03 */
    {NULL, NULL,   0x1A03, {DEFTYPE_PDOMAPPING, 2 | (OBJCODE_REC << 8)}, asEntryDesc0x1A03, aName0x1A03, &sAITxPDO1Map, NULL, NULL, 0x0000 },
    /* Object 0x1C12 */
    {NULL, NULL,   0x1C12, {DEFTYPE_UNSIGNED16, 2 | (OBJCODE_ARR << 8)}, asEntryDesc0x1C12, aName0x1C12, &sRxPDOassign, NULL, NULL, 0x0000 },
    /* Object 0x1C13 */
    {NULL, NULL,   0x1C13, {DEFTYPE_UNSIGNED16, 2 | (OBJCODE_ARR << 8)}, asEntryDesc0x1C13, aName0x1C13, &sTxPDOassign, NULL, NULL, 0x0000 },
    /* Object 0x6000 */
    {NULL, NULL,   0x6000, {DEFTYPE_RECORD, 1 | (OBJCODE_REC << 8)}, asEntryDesc0x6000, aName0x6000, &sDIInputs, NULL, NULL, 0x0000 },
    /* Object 0x6020 */
    {NULL, NULL,   0x6030, {DEFTYPE_RECORD, 2 | (OBJCODE_REC << 8)}, asEntryDesc0x6030, aName0x6030, &sAI1Inputs, NULL, NULL, 0x0000 },
    /* Object 0x7010 */
    {NULL, NULL,   0x7010, {DEFTYPE_RECORD, 1 | (OBJCODE_REC << 8)}, asEntryDesc0x7010, aName0x7010, &sDOOutputs, NULL, NULL, 0x0000 },
    /* Object 0x7020 */
    {NULL, NULL,   0x7020, {DEFTYPE_RECORD, 3 | (OBJCODE_REC << 8)}, asEntryDesc0x7020, aName0x7020, &sDO1Outputs, NULL, NULL, 0x0000 },
    {NULL, NULL, 0xFFFF, {0, 0}, NULL, NULL, NULL, NULL, NULL, 0x0000}
};
#endif  //#ifdef _OBJD_

#if EXPLICIT_DEVICE_ID
PROTO UINT16 APPL_GetDeviceID();
#endif
/*ECATCHANGE_START(V4.42.1) ECAT2*/
PROTO void APPL_Application(void);

PROTO void   APPL_AckErrorInd(UINT16 stateTrans);
PROTO UINT16 APPL_StartMailboxHandler(void);
PROTO UINT16 APPL_StopMailboxHandler(void);
PROTO UINT16 APPL_StartInputHandler(UINT16 *pIntMask);
PROTO UINT16 APPL_StopInputHandler(void);
PROTO UINT16 APPL_StartOutputHandler(void);
PROTO UINT16 APPL_StopOutputHandler(void);

PROTO UINT16 APPL_GenerateMapping(UINT16 *pInputSize, UINT16 *pOutputSize);
PROTO void APPL_InputMapping(UINT16 *pData);
PROTO void APPL_OutputMapping(UINT16 *pData);

#endif //#if TIESC_HW

#undef PROTO

