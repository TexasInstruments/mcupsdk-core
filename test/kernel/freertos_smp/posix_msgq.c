/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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

/* FreeRTOS includes. */
#include "FreeRTOS_POSIX.h"

/* System headers. */
#include <stdbool.h>
#include <string.h>
#include "kernel/dpl/ClockP.h"

/* FreeRTOS+POSIX. */
#include "FreeRTOS_POSIX/pthread.h"
#include "FreeRTOS_POSIX/mqueue.h"
#include "FreeRTOS_POSIX/time.h"
#include "FreeRTOS_POSIX/fcntl.h"
#include "FreeRTOS_POSIX/errno.h"


#define MSG_QNAME  "/msgQueue0"
#define MQUEUE_MAX_NUMBER_OF_MESSAGES       1
#define MQUEUE_MSG_SIZE                     sizeof( uint8_t ) /**< Message size. */

#define MQUEUE_NUM_MSGS                     ( 10 ) /* Number of messages to send from dispatcher */
#define MQUEUE_TIMEOUT_SECONDS              ( 1 ) /* Timeout for mqueue functions */

typedef enum ControlMessage
{
    eMSG_LOWER_INAVLID = 0x00,
    eMSG_CTRL_MSG_CONTINUE = 0x01,
    eMSG_CTRL_MSG_EXIT = 0x02,
    eMSG_UPPER_INVALID = 0xFF
} eControlMessage;

mqd_t mqInboxID;

uint32_t mq_receiveError = 0;

int32_t mq_init( void )
{
    int status = SystemP_SUCCESS;
    /* Initialize message queue */

    struct mq_attr xQueueAttributes =
    {
        .mq_flags   = 0,
        .mq_maxmsg  = MQUEUE_MAX_NUMBER_OF_MESSAGES,
        .mq_msgsize = MQUEUE_MSG_SIZE,
        .mq_curmsgs = 0
    };

    mqInboxID = mq_open( MSG_QNAME,
                            O_CREAT | O_RDWR,
                            ( mode_t ) 0,
                            &xQueueAttributes );

    if( mqInboxID == ( mqd_t ) -1 )
    {
        DebugP_log( "Invalid inbox (mqueue) \r\n");
        status = SystemP_FAILURE;
    }

    return status;

}

void mq_dispatcher( void )
{
    struct timespec xSendTimeout = { 0 };

    ssize_t xMessageSize = 0;
    char pcSendBuffer[ MQUEUE_MSG_SIZE ] = { 0 };

    pcSendBuffer[ 0 ] = ( char ) eMSG_CTRL_MSG_CONTINUE;

    for (int i = 0; i < MQUEUE_NUM_MSGS; i++)
    {
        clock_gettime( CLOCK_REALTIME, &xSendTimeout );
        xSendTimeout.tv_sec += MQUEUE_TIMEOUT_SECONDS;

        xMessageSize = mq_timedsend( mqInboxID,
                                     pcSendBuffer,
                                     MQUEUE_MSG_SIZE,
                                     0,
                                     &xSendTimeout );

    }

    ClockP_usleep(1000); /* sleep for few usecs */

    /* Tell receiving task transactions are completed */
    pcSendBuffer[ 0 ] = ( char ) eMSG_CTRL_MSG_EXIT;

    xMessageSize = mq_send( mqInboxID,
                                pcSendBuffer,
                                MQUEUE_MSG_SIZE,
                                0 );
}


void mq_receiver( void )
{
    struct timespec xReceiveTimeout = { 0 };

    ssize_t xMessageSize = 0;
    char pcReceiveBuffer[ MQUEUE_MSG_SIZE ] = { 0 };

    while (true)
    {
        clock_gettime( CLOCK_REALTIME, &xReceiveTimeout );
        xReceiveTimeout.tv_sec += MQUEUE_TIMEOUT_SECONDS;

        xMessageSize = mq_receive( mqInboxID,
                                   pcReceiveBuffer,
                                   MQUEUE_MSG_SIZE,
                                   0 );

        if( xMessageSize == MQUEUE_MSG_SIZE )
        {
            switch( ( int ) pcReceiveBuffer[ 0 ] )
            {
                case eMSG_CTRL_MSG_CONTINUE:
                    break;

                case eMSG_CTRL_MSG_EXIT:
                    return;

                default:
                    /* Received an invalid message */
                    mq_receiveError++;
                    break;
            }
        }
        else
        {
            /* Invalid message. */
            mq_receiveError++;
        }

    }
}