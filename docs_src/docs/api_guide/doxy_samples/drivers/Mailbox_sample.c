
#include <stdio.h>
#include <kernel/dpl/DebugP.h>
//! [include]
#include <drivers/mailbox.h>
//! [include]

void mailbox_sample()
{
    //! [init]
    /* OPTIONAL: Called during System_init() when IPC enabled via SysConfig */
    /* MAKE SURE IPC Notify is initialized before mailbox module is initialized,
       else the mailbox module will not work as expected
      */
    int32_t status;
    Mailbox_Params mailboxInitParams;

    Mailbox_Params_init(&mailboxInitParams);
    status = Mailbox_init(&mailboxInitParams);
    DebugP_assert(status == SystemP_SUCCESS);
    //! [init]

    //! [write]
    /* send a message to RSS R4, the API waits until a ACK is received from the destination CPU */
    {

        uint32_t buffer[12];

        /* fill the buffer with message to send */

        status = Mailbox_write(
                        CSL_CORE_ID_RSS_R4, /* destination CPU ID */
                        (uint8_t*)buffer, /* message contents */
                        sizeof(buffer), /* size of message contents */
                        SystemP_WAIT_FOREVER  /* time to wait for ACK */
                    );
        DebugP_assert(status==SystemP_SUCCESS); /* if message size exceeds limit or timeout error is returned */
    }
    //! [write]

    //! [read]
    /* wait for message and read the contents from source CPU */
    {
        uint32_t buffer[12];

        status = Mailbox_read(
                    CSL_CORE_ID_RSS_R4, /* source CPU ID */
                    (uint8_t*)buffer,  /* buffer to read the message into */
                    sizeof(buffer), /* amount of bytes to read, note, user needs to know the size of expected message */
                    SystemP_WAIT_FOREVER /* time to wait for message recevied interrupt to occur */
                );
        DebugP_assert(status==SystemP_SUCCESS); /* if message size exceeds limit or timeout error is returned */

        /* use the received message from buffer */

        /* mark current message read as "done", this sends a ACK to the source CPU and updates internal state
           to be ready for new message.

           THIS MUST be called for every received message once, else next message will bot be received
         */
        status = Mailbox_readDone(CSL_CORE_ID_RSS_R4);
        DebugP_assert(status==SystemP_SUCCESS);
    }
    //! [read]
}