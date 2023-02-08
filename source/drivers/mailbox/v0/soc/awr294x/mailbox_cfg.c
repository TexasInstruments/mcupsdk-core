/*
 *  Copyright (C) 2018-2023 Texas Instruments Incorporated
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

#include <drivers/mailbox/v0/mailbox_priv.h>

/* we use mailbox based communication only between pair of CPUs,
  - RSS R4 and R5FSS0-0
  - RSS R4 and C66SS0
 */

/* max size of read and write buffer */
#define MAX_BUFFER_SIZE             (512U)

/* memory used to send messages to RSS R4 */
#define R5FSS0_0_TO_RSS_R4_MEM  (CSL_RSS_CR4_MBOX_U_BASE + 0x0000U )
#define C66SS0_TO_RSS_R4_MEM    (CSL_RSS_CR4_MBOX_U_BASE + 0x0800U )

/* memory used to receive messages from RSS R4 */
#define RSS_R4_TO_R5FSS0_0_MEM  (CSL_MSS_MBOX_U_BASE    + 0x0000U)
#define RSS_R4_TO_C66SS0_0_MEM  (CSL_DSS_MAILBOX_U_BASE + 0x0000U)

/* mailbox registers */
#define R5FSS0_0_MBOX_WRITE_DONE    (CSL_MSS_CTRL_U_BASE + 0x5FCU)
#define R5FSS0_0_MBOX_READ_DONE     (CSL_MSS_CTRL_U_BASE + 0x604U)
#define R5FSS0_0_MBOX_READ_DONE_ACK (CSL_MSS_CTRL_U_BASE + 0xFF0U)

#define C66SS0_MBOX_WRITE_DONE      (CSL_DSS_CTRL_U_BASE + 0x56CU)
#define C66SS0_MBOX_READ_DONE       (CSL_DSS_CTRL_U_BASE + 0x574U)
#define C66SS0_MBOX_READ_DONE_ACK   (CSL_DSS_CTRL_U_BASE + 0xFF0U)

/* CPU bit positions within the mailbox registers */
#define RSS_R4_MBOX_PROC_BIT_POS    (12U)

#define RIGHT_SHIFT_BY_TWO    (2U)

Mailbox_RemoteCoreObj gMailboxRemoteCoreObj_r5ss0_0_to_rss_r4 =
{
    .maxBufferSize = MAX_BUFFER_SIZE,
    .writeShmBuffer = (uint8_t*)R5FSS0_0_TO_RSS_R4_MEM,
    .readShmBuffer = (uint8_t*)RSS_R4_TO_R5FSS0_0_MEM,
    .writeIntrRegAddr = R5FSS0_0_MBOX_WRITE_DONE,
    .writeAckIntrRegAddr = R5FSS0_0_MBOX_READ_DONE,
    .readAckIntrRegAddr = R5FSS0_0_MBOX_READ_DONE_ACK,
    .writeIntrBitPos = RSS_R4_MBOX_PROC_BIT_POS,
    .writeAckIntrBitPos = RSS_R4_MBOX_PROC_BIT_POS,
    .readAckIntrBitPos = RSS_R4_MBOX_PROC_BIT_POS >> RIGHT_SHIFT_BY_TWO, /* here we need to /4 to get to the bit pos vs other mailbox regs */
};

Mailbox_RemoteCoreObj gMailboxRemoteCoreObj_c66ss0_to_rss_r4 =
{
    .maxBufferSize = MAX_BUFFER_SIZE,
    .writeShmBuffer = (uint8_t*)C66SS0_TO_RSS_R4_MEM,
    .readShmBuffer = (uint8_t*)RSS_R4_TO_C66SS0_0_MEM,
    .writeIntrRegAddr = C66SS0_MBOX_WRITE_DONE,
    .writeAckIntrRegAddr = C66SS0_MBOX_READ_DONE,
    .readAckIntrRegAddr = C66SS0_MBOX_READ_DONE_ACK,
    .writeIntrBitPos = RSS_R4_MBOX_PROC_BIT_POS,
    .writeAckIntrBitPos = RSS_R4_MBOX_PROC_BIT_POS,
    .readAckIntrBitPos = RSS_R4_MBOX_PROC_BIT_POS >> RIGHT_SHIFT_BY_TWO, /* here we need to /4 to get to the bit pos vs other mailbox regs */
};

Mailbox_RemoteCoreObj *Mailbox_getRemoteCoreObj(uint32_t selfCoreId, uint32_t remoteCoreId)
{
    Mailbox_RemoteCoreObj *obj = NULL;

    if(remoteCoreId == CSL_CORE_ID_RSS_R4)
    {
        if(selfCoreId == CSL_CORE_ID_R5FSS0_0)
        {
            obj = &gMailboxRemoteCoreObj_r5ss0_0_to_rss_r4;
        }
        else
        if(selfCoreId == CSL_CORE_ID_C66SS0)
        {
            obj = &gMailboxRemoteCoreObj_c66ss0_to_rss_r4;
        }
        else
        {
            /*MISRAC*/
        }

    }
    else
    {
        /*MISRAC*/
    }
    return obj;
}
