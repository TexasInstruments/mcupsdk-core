/*
 *  Copyright (c) Texas Instruments Incorporated 2018
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

/**
 *  \file udma_test_misc.c
 *
 *  \brief UDMA other misc test case file.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <udma_test.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t udmaTestPsilMacroTc(UdmaTestTaskObj *taskObj)
{
    int32_t     retVal = UDMA_SOK;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: PSIL/PDMA Macro Verification Testcase ::\r\n", taskObj->taskId);

    retVal = udmaTestPrintPsilMacro(taskObj);
    retVal += udmaTestPrintPdmaMacro(taskObj);

    return (retVal);
}

int32_t udmaTestTrMakeTc(UdmaTestTaskObj *taskObj)
{
    int32_t         retVal = UDMA_SOK;
    uint32_t        trSize, trSizeEncoded;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: TR Make Utility Testcase ::\r\n", taskObj->taskId);

    trSize        = UdmaUtils_getTrSizeBytes(UDMA_TR_TYPE_0);
    trSizeEncoded = UdmaUtils_getTrSizeEncoded(UDMA_TR_TYPE_0);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " TR0  Size: %d Bytes (%db Encoded)\r\n", trSize, trSizeEncoded);
    trSize        = UdmaUtils_getTrSizeBytes(UDMA_TR_TYPE_1);
    trSizeEncoded = UdmaUtils_getTrSizeEncoded(UDMA_TR_TYPE_1);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " TR1  Size: %d Bytes (%db Encoded)\r\n", trSize, trSizeEncoded);
    trSize        = UdmaUtils_getTrSizeBytes(UDMA_TR_TYPE_2);
    trSizeEncoded = UdmaUtils_getTrSizeEncoded(UDMA_TR_TYPE_2);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " TR2  Size: %d Bytes (%db Encoded)\r\n", trSize, trSizeEncoded);
    trSize        = UdmaUtils_getTrSizeBytes(UDMA_TR_TYPE_3);
    trSizeEncoded = UdmaUtils_getTrSizeEncoded(UDMA_TR_TYPE_3);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " TR3  Size: %d Bytes (%db Encoded)\r\n", trSize, trSizeEncoded);
    trSize        = UdmaUtils_getTrSizeBytes(UDMA_TR_TYPE_4);
    trSizeEncoded = UdmaUtils_getTrSizeEncoded(UDMA_TR_TYPE_4);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " TR4  Size: %d Bytes (%db Encoded)\r\n", trSize, trSizeEncoded);
    trSize        = UdmaUtils_getTrSizeBytes(UDMA_TR_TYPE_5);
    trSizeEncoded = UdmaUtils_getTrSizeEncoded(UDMA_TR_TYPE_5);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " TR5  Size: %d Bytes (%db Encoded)\r\n", trSize, trSizeEncoded);
    trSize        = UdmaUtils_getTrSizeBytes(UDMA_TR_TYPE_8);
    trSizeEncoded = UdmaUtils_getTrSizeEncoded(UDMA_TR_TYPE_8);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " TR8  Size: %d Bytes (%db Encoded)\r\n", trSize, trSizeEncoded);
    trSize        = UdmaUtils_getTrSizeBytes(UDMA_TR_TYPE_9);
    trSizeEncoded = UdmaUtils_getTrSizeEncoded(UDMA_TR_TYPE_9);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " TR9  Size: %d Bytes (%db Encoded)\r\n", trSize, trSizeEncoded);
    trSize        = UdmaUtils_getTrSizeBytes(UDMA_TR_TYPE_10);
    trSizeEncoded = UdmaUtils_getTrSizeEncoded(UDMA_TR_TYPE_10);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " TR10 Size: %d Bytes (%db Encoded)\r\n", trSize, trSizeEncoded);
    trSize        = UdmaUtils_getTrSizeBytes(UDMA_TR_TYPE_11);
    trSizeEncoded = UdmaUtils_getTrSizeEncoded(UDMA_TR_TYPE_11);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " TR11 Size: %d Bytes (%db Encoded)\r\n", trSize, trSizeEncoded);
    trSize        = UdmaUtils_getTrSizeBytes(UDMA_TR_TYPE_15);
    trSizeEncoded = UdmaUtils_getTrSizeEncoded(UDMA_TR_TYPE_15);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " TR15 Size: %d Bytes (%db Encoded)\r\n", trSize, trSizeEncoded);

    return (retVal);
}

int32_t udmaTestStructSizeTc(UdmaTestTaskObj *taskObj)
{
    int32_t retVal = UDMA_SOK;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: UDMA Struct Size Print Testcase ::\r\n", taskObj->taskId);

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Driver Object Size              : %-5d Bytes\r\n", sizeof(Udma_DrvObject));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Channel Object Size             : %-5d Bytes\r\n", sizeof(Udma_ChObject));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Event Object Size               : %-5d Bytes\r\n", sizeof(Udma_EventObject));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Ring Object Size                : %-5d Bytes\r\n", sizeof(Udma_RingObject));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Flow Object Size                : %-5d Bytes\r\n", sizeof(Udma_FlowObject));

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Udma_InitPrms Size              : %-5d Bytes\r\n", sizeof(Udma_InitPrms));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Udma_RmInitPrms Size            : %-5d Bytes\r\n", sizeof(Udma_RmInitPrms));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Udma_ChPrms Size                : %-5d Bytes\r\n", sizeof(Udma_ChPrms));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Udma_ChTxPrms Size              : %-5d Bytes\r\n", sizeof(Udma_ChTxPrms));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Udma_ChRxPrms Size              : %-5d Bytes\r\n", sizeof(Udma_ChRxPrms));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Udma_ChPdmaPrms Size            : %-5d Bytes\r\n", sizeof(Udma_ChPdmaPrms));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Udma_EventPrms Size             : %-5d Bytes\r\n", sizeof(Udma_EventPrms));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Udma_EventRxFlowIdFwStatus Size : %-5d Bytes\r\n", sizeof(Udma_EventRxFlowIdFwStatus));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Udma_RingPrms Size              : %-5d Bytes\r\n", sizeof(Udma_RingPrms));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Udma_FlowPrms Size              : %-5d Bytes\r\n", sizeof(Udma_FlowPrms));

    return (retVal);
}
