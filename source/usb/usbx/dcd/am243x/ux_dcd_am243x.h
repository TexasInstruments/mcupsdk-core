/*
 *  Copyright (C) 2018-2026 Texas Instruments Incorporated
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

#ifndef UX_DCD_AM243X_H
#define UX_DCD_AM243X_H

#define UX_DCD_AM243X_DEBUG_LOG_EN (0U)

#if (UX_DCD_AM243X_DEBUG_LOG_EN == 1U)
#include <kernel/dpl/DebugP.h>

#ifndef UX_DCD_LOG
#define UX_DCD_LOG(...) DebugP_log(__VA_ARGS__)
#endif

#else

#ifndef UX_DCD_LOG
#define UX_DCD_LOG(...)
#endif

#endif

#define UX_DCD_AM243X_SLAVE_CONTROLLER (0x01U)

#define UX_DCD_AM243X_MSG_SETUP (0U)
#define UX_DCD_AM243X_MSG_EP0IN (1U)
#define UX_DCD_AM243X_MSG_EPIN (2U)
#define UX_DCD_AM243X_MSG_EPOUT (3U)


typedef struct _ux_am243x_dcd_msg {
    ULONG type;
    ULONG ep_addr;
    ULONG actual;
    ULONG req;
    ULONG status;
    uint8_t setup[8];
} _ux_am243x_dcd_msg_t;

extern TX_QUEUE _ux_dcd_am243x_queue;

UINT _ux_dcd_am243x_initialize(ULONG dcd_io);
UINT _ux_dcd_am243x_initialize_complete(VOID);
VOID _ux_dcd_am243x_interrupt_thread(ULONG dcd_pointer);
UINT _ux_dcd_am243x_function(UX_SLAVE_DCD *dcd, UINT function, VOID *parameter);



#endif // #ifndef UX_DCD_AM243X_H
