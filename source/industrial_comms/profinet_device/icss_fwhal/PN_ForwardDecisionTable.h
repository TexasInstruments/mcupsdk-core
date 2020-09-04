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

#ifndef PN_FORWARD_DECISION_TABLE_H_
#define PN_FORWARD_DECISION_TABLE_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Each row in table represent 32 MAC Addresses which are listed on right side                                               */
/* There is one bit for each of 32 MAC Addresses in a row                                                                    */
/* If value of bit is 1 that means frame with corresponding MAC Address will be forwarded otherwise it is not forwarded      */
/* Following MAC Addresses are parsed separately by the PRU Firmware. For these MAC Addresses decision table is not referred */
/* 01-0E-CF-00-01-01 (RTC3 Frames), 01-0E-CF-00-04-20 (PTCP-RTSyncPDU Frames with Follow Up)                                 */
/* 01-0E-CF 00-04-40 (PTCP-FollowUpPDU Frames), 01-0E-CF 00-04-80 (PTCP-RTSyncPDU Frames)                                    */

const uint32_t PN_Forward_Decision_Table[] =
{
    0x00000003, /* 01-0E-CF-00-00-00  ..  01-0E-CF-00-00-1F */
    0x00000000, /* 01-0E-CF-00-00-20  ..  01-0E-CF-00-00-3F */
    0x00000000, /* 01-0E-CF-00-00-40  ..  01-0E-CF-00-00-5F */
    0x00000000, /* 01-0E-CF-00-00-60  ..  01-0E-CF-00-00-7F */
    0x00000000, /* 01-0E-CF-00-00-80  ..  01-0E-CF-00-00-9F */
    0x00000000, /* 01-0E-CF-00-00-A0  ..  01-0E-CF-00-00-BF */
    0x00000000, /* 01-0E-CF-00-00-C0  ..  01-0E-CF-00-00-DF */
    0x00000000, /* 01-0E-CF-00-00-E0  ..  01-0E-CF-00-00-FF */
    0x00000000, /* 01-0E-CF-00-01-00  ..  01-0E-CF-00-01-1F */
    0x00000000, /* 01-0E-CF-00-01-20  ..  01-0E-CF-00-01-3F */
    0x00000000, /* 01-0E-CF-00-01-40  ..  01-0E-CF-00-01-5F */
    0x00000000, /* 01-0E-CF-00-01-60  ..  01-0E-CF-00-01-7F */
    0x00000000, /* 01-0E-CF-00-01-80  ..  01-0E-CF-00-01-9F */
    0x00000000, /* 01-0E-CF-00-01-A0  ..  01-0E-CF-00-01-BF */
    0x00000000, /* 01-0E-CF-00-01-C0  ..  01-0E-CF-00-01-DF */
    0x00000000, /* 01-0E-CF-00-01-E0  ..  01-0E-CF-00-01-FF */
    0xffffffff, /* 01-0E-CF-00-02-00  ..  01-0E-CF-00-02-1F */
    0xffffffff, /* 01-0E-CF-00-02-20  ..  01-0E-CF-00-02-3F */
    0xffffffff, /* 01-0E-CF-00-02-40  ..  01-0E-CF-00-02-5F */
    0xffffffff, /* 01-0E-CF-00-02-60  ..  01-0E-CF-00-02-7F */
    0xffffffff, /* 01-0E-CF-00-02-80  ..  01-0E-CF-00-02-9F */
    0xffffffff, /* 01-0E-CF-00-02-A0  ..  01-0E-CF-00-02-BF */
    0xffffffff, /* 01-0E-CF-00-02-C0  ..  01-0E-CF-00-02-DF */
    0xffffffff, /* 01-0E-CF-00-02-E0  ..  01-0E-CF-00-02-FF */
    0x00000000, /* 01-0E-CF-00-03-00  ..  01-0E-CF-00-03-1F */
    0x00000000, /* 01-0E-CF-00-03-20  ..  01-0E-CF-00-03-3F */
    0x00000000, /* 01-0E-CF-00-03-40  ..  01-0E-CF-00-03-5F */
    0x00000000, /* 01-0E-CF-00-03-60  ..  01-0E-CF-00-03-7F */
    0x00000000, /* 01-0E-CF-00-03-80  ..  01-0E-CF-00-03-9F */
    0x00000000, /* 01-0E-CF-00-03-A0  ..  01-0E-CF-00-03-BF */
    0x00000000, /* 01-0E-CF-00-03-C0  ..  01-0E-CF-00-03-DF */
    0x00000000, /* 01-0E-CF-00-03-E0  ..  01-0E-CF-00-03-FF */
    0x00000001, /* 01-0E-CF-00-04-00  ..  01-0E-CF-00-04-1F */
    0x00000001, /* 01-0E-CF-00-04-20  ..  01-0E-CF-00-04-3F */
    0x00000001, /* 01-0E-CF-00-04-40  ..  01-0E-CF-00-04-5F */
    0x00000000, /* 01-0E-CF-00-04-60  ..  01-0E-CF-00-04-7F */
    0x00000001, /* 01-0E-CF-00-04-80  ..  01-0E-CF-00-04-9F */
    0x00000000, /* 01-0E-CF-00-04-A0  ..  01-0E-CF-00-04-BF */
    0x00000000, /* 01-0E-CF-00-04-C0  ..  01-0E-CF-00-04-DF */
    0x00000000, /* 01-0E-CF-00-04-E0  ..  01-0E-CF-00-04-FF */
    0x00000000, /* 01-0E-CF-00-05-00  ..  01-0E-CF-00-05-1F */
    0x00000000, /* 01-0E-CF-00-05-20  ..  01-0E-CF-00-05-3F */
    0x00000000, /* 01-0E-CF-00-05-40  ..  01-0E-CF-00-05-5F */
    0x00000000, /* 01-0E-CF-00-05-60  ..  01-0E-CF-00-05-7F */
    0x00000000, /* 01-0E-CF-00-05-80  ..  01-0E-CF-00-05-9F */
    0x00000000, /* 01-0E-CF-00-05-A0  ..  01-0E-CF-00-05-BF */
    0x00000000, /* 01-0E-CF-00-05-C0  ..  01-0E-CF-00-05-DF */
    0x00000000, /* 01-0E-CF-00-05-E0  ..  01-0E-CF-00-05-FF */
    0x00000006, /* 01-15-4E-00-00-00  ..  01-15-4E-00-00-1F */
    0x00000001  /* 01-80-C2-00-00-00  ..  01-80-C2-00-00-1F */
};


#ifdef __cplusplus
}
#endif

#endif /* PN_FORWARD_DECISION_TABLE_H_ */