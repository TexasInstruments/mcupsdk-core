# IPC Notify {#DRIVERS_IPC_NOTIFY_PAGE}

[TOC]

This module define's APIs for low latency IPC between different core's on a CPU.
These low latency IPC APIs are constrained in features but offer extermely fast
transfer of message values between two cores.

Underlying implementation will use HW mechanisms to interrupt the
receiving cores, it will also use HW FIFOs (when available)
or shared memory based SW FIFOs in fast internal RAM's
to transport the message values.

\cond SOC_AM64X || SOC_AM243X
@VAR_SOC_NAME uses HW mailbox based HW FIFOs to transport the message and interrupt the receiving core.
\endcond

## Features Supported

- Low latency message send and receive between any to any CPUs running no-RTOS or RTOS
- Low latency is achieved by
  - Accessing the HW in very few steps (due to this most error checking is left to the user)
  - Combining the message and client ID into a single 32b value that is send via HW/SW FIFO's
    (due to this there are constraints on max client ID and max message value)
  - Handling the message received within ISR itself and calling user callback within few steps of receiving the interrupt.
- Client ID field allows to send messages to different SW clients on the receving side
  - Typically each SW client will be indepedant SW entity, so client ID allows to do basic
    demultiplexing of messages and thus keep the SW entities indepedant of each other even when IPC is involved.
- Ability to register different user handlers for different client ID's
- Callback based mechnism to recieve messages
- Ability to block on message send for ever OR return with error, if the underlying IPC HW/SW FIFO is full.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Enable/Disable IPC Notify between different CPUs

## Features NOT Supported

- IPC Notify cannot be used to talk to clients running Linux OS

## Important Usage Guidelines

- To balance low latency performance vs flexiblility to end user, below contraints are introduced in the API
  - The maximum number of clients supported is constrained to \ref IPC_NOTIFY_CLIENT_ID_MAX.
  - The maximum message value that is exchanged is contrained to \ref IPC_NOTIFY_MSG_VALUE_MAX
- \ref IPC_NOTIFY_MSG_VALUE_MAX is < 32b, and hence one cannot pass pointers as messages.
  - However, passing pointers as messages is not a recommended SW design and instead
    one should pass offset from some known base address as values instead.
  - Offsets can easily fit within \ref IPC_NOTIFY_MSG_VALUE_MAX limit.
- Internally the implementation will combine client ID and message value as one 32b integer.

## Example Usage

Include the below files to access the APIs
\snippet IpcNotify_sample.c include

Initialize IPC Notify
\snippet IpcNotify_sample.c init

Register handler to receive messages
\snippet IpcNotify_sample.c register

Send message
\snippet IpcNotify_sample.c send

Receive message and handle it in a task
\snippet IpcNotify_sample.c recv

## API

\ref DRV_IPC_NOTIFY_MODULE
