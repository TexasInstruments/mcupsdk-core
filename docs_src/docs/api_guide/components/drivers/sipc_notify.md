# SIPC Notify {#DRIVERS_SIPC_NOTIFY_PAGE}

[TOC]

\note Secure IPC Notify driver enables IPC communication between R5F cores and
HSM M4 core. This driver cannot be used without encrypted HSMRt binary for 
HS-FS devices (will be available in 8.4) and TIFS-MCU for HS-SE devices 
(available on mySecureSW).

## Introduction

- The **secure IPC notify(SIPC Notify)** driver enables IPC communication between
  R5FSSx-x cores and HSM M4 core for **HS-SE** devices.

- SIPC provides a means of secure communication between R5FSSx and HSM M4 for
using HSM services.
  The underlying implementation uses **mailbox HW interrupts and HSM MBOX shared
  memory**(refer @ref Mailbox-mechanism-for-sipc)
   to pass messages from R5FSSx-x to HSM M4 and vice versa.

- The trusted R5FSSx-x which can post a request for services from HSM are termed
 as **secure hosts**. A non-secure host cannot
  communicate with HSM, moreover, the shared memory used in SIPC communication
   is not accessible to non-secure hosts.
  The cores that are going to be secure hosts are defined via **sysconfig** and
  this will be a static configuration at compile time.
  At a time there can only be atmost two secure hosts which can talk to HSM via
  SIPC, thus the name **secure IPC**.

@note Sysconfig initialization has to be handled by **HSM client**. For
Secure IPC, Sysconfig is not supported in this release. It will be available with
upcoming SDK releases
along with **HSM client**
## Secure IPC design description

- SIPC supports RTOS-based implementation on the R5F side. The concept of
**CLIENT IDs**
  is borrowed from @ref DRIVERS_IPC_NOTIFY_PAGE, where
  in the RTOS scenario each task has a callback registered of type @ref
  SIPC_FxnCallback.
  This function will be called when a message is received for a respective
   CLIENT ID. Refer to @ref IPC_GUIDE to know more about CLIENT IDs.

- HSM client is a wrapper on top of SIPC which asks for specific services from
**TIFS-MCU**. User can install TIFS package from **MySecureSw** portal.

\imageStyle{sipc_communication_model.png,width:35%}
\image html sipc_communication_model.png "SIPC communication model"

- The maximum number of client Ids available on R5F is
 @ref SIPC_CLIENT_ID_MAX = 5.
On the HSM M4 side the @ref SIPC_CLIENT_ID_MAX = 2 as it doesn't run
RTOS applications.

### Mailbox mechanism for SIPC {#Mailbox-mechanism-for-sipc}

-  SIPC uses **HSM_MBOX** (2KB) memory for secure communication with HSM. It uses
**MBOX_READ_DONE_ACK** interrupt to notify target core to read from the dedicated
message queue residing in **HSM_MBOX** memory.  @ref DRIVERS_IPC_NOTIFY_PAGE uses
**MSS_MBOX**(16KB) memory for R5<-->R5 communication.

\imageStyle{sipc_mailbox.png,width:30%}
\image html sipc_mailbox.png "SIPC shared memory configuration"

- The swQx is a struct holding pointer to actual queue location in HSM MBOX.
There can be atmost 2 secure hosts at a time, so the total number of
queues will be 4. If R5F wants to send a message to HSM then R5F writes into
 R5F -> HSM queue and sends an interrupt to HSM M4, once acknowledged HSM will
read the message from the same queue. Similarly, If HSM wants to send a message
 then HSM will write into HSM --> R5 queues and
sends an interrupt to R5F.

\note The entire HSM MBOX memory must be read and write protected for non-secure
hosts.
setting up this firewall configuration is the responsibility of the HSM server
running on M4.

### SIPC initialization

- Following parameters are user-configurable.

    1. **Queue Depth.** -: Number of elements per queue.
    2. **Number of secure hosts** -: Could be 1 or 2.
    3. **Secure hosts core Ids** -: which R5F cores will be secure hosts.

These parameters will be initialized via sysconfig.

\snippet sipc_setup_sample.c setup

- sysconfig will also generate code to allocate memory for the queues in
HSM MBOX based on Queue depth and number of secure hosts.
The pointer to this queue will be passed to @ref SIPC_init at init time.
For the initialization on R5F side,
 the @ref SIPC_Params::tx_SipcQueues will point to **gQueue_R5ToHsm**
  where R5F will write to HSM. Similarly,
@ref SIPC_Params::rx_SipcQueues will point to **gQueueHsmToR5** from where R5F will read the message sent by HSM M4.

- As the size of HSM MBOX is 2KB the max Queue Depth is limited.
    1. if **SIPC_NUM_R5_CORES** is 2 then the total number of queues will be
    4 and the max queue depth will be  492/ @ref SIPC_MSG_SIZE .
    2. if **SIPC_NUM_R5_CORES** is 1 then the total number of queues will be
     1 and the max queue depth will be 984/ @ref SIPC_MSG_SIZE .

- **SIPC Initialization from R5F secure host**

\snippet sipc_setup_sample.c r5conf

- **SIPC Initialization from HSM**

\snippet sipc_setup_sample.c hsmconf

- **linker command file**

- Following sections need to be defined as described below. The SIPC queues will
be initialized in then mentioned memory sections.
\snippet sipc_setup_sample.c linker

### Message format.

 Unlike @ref DRIVERS_IPC_NOTIFY_PAGE the message size of SIPC can be in the range
 of 3 to 100 bytes.
- The minimum message size is 3 bytes i.e
[ **Dest ClientId** ][ **Src ClientId** ][**data**].
 Users can change @ref SIPC_MSG_SIZE based on a protocol that is implemented on
 top of SIPC.

- Following is an example of a message structure used by **HSM client** to
communicate with HSM via SIPC. The size of a message is **13 bytes** refer to
**HSM_CLIENT**.

\imageStyle{sipc_message_ex.png,width:50%}
\image html sipc_message_ex.png "message format used by HSM client"


@note It is recommended to use 13 bytes message format as mentioned above for
optimal latency.

## Example usage.

- Consider a use case where R5F sends a message to HSM and HSM echoes back the
same message.
    1. Do @ref SIPC_init.
    2. Register a callback to handle the message from HSM @ref
    SIPC_registerClient.
    3. Sends a message using @ref SIPC_sendMsg and pend on a semaphore
    to Wait for a message.
    4. Once the message is received post the semaphore @ref SemaphoreP_post
    and exit.

\snippet sipc_setup_sample.c exusage

- Refer **Secure IPC example** for more info.

@note The secure IPC example project is a part of **TIFS** package for **HS-SE**
 devices.
To use this example user needs to sign an NDA(non disclosure agreement). **TIFS**
package can installed via mysecure software.
## API

@ref DRV_SIPC_NOTIFY_MODULE
