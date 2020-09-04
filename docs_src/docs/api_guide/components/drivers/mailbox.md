# MAILBOX {#DRIVERS_MAILBOX_PAGE}

[TOC]

\attention Mailbox driver is only applicable to AWR294X
\attention Mailbox driver is used to send and receive messages with RSS R4 running radar control firmware.
\attention For application level IPC one should use \ref DRIVERS_IPC_RPMESSAGE_PAGE and/or \ref DRIVERS_IPC_NOTIFY_PAGE

## Features Supported

- Ability to send and receive messages using the mailbox + shared memory protocol
- Timeout on message write ACK and read receive interrupt
- User callback on read message receive
- Interrupts used for waiting for read message
- Polling used for waiting for write ACK

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Enabled as part of IPC module in SysConfig

## Features not Supported

- Only specific SOC and CPUs can be used for this mailbox communication, use \ref DRIVERS_IPC_RPMESSAGE_PAGE and/or \ref DRIVERS_IPC_NOTIFY_PAGE for the rest

## Important Notes

- Make sure \ref DRIVERS_IPC_NOTIFY_PAGE is enabled and initialized before initializing mailbox module, else mailbox module
  will not work as expected
  - When IPC is enabled via SysConfig, mailbox module is also enabled along with IPC Notify.
\cond SOC_AWR294X
- Mailbox driver uses the mailbox RAM as defined below for reading and writing messages.

Source CPU | Destination CPU | Memory used | Memory Size | Remarks
-----------|-----------------|-------------|-------------|--------
RSS R4     | R5FSS0-0        | 0xC5000000  | 512 B       | MSS Mailbox memory
RSS R4     | C66SS0          | 0x83100000  | 512 B       | DSS Mailbox memory
R5FSS0-0   | RSS R4          | 0xA4030000  | 512 B       | RSS Mailbox memory
RSS R4     | R5FSS0-0        | 0xA4030800  | 512 B       | RSS Mailbox memory

- Make sure these memories are left unused by rest of applications
- Make sure for R5F and C6x, this region is marked as non-cached in their address space via MPU or CPU settings
\endcond

## Example Usage

Include the below file to access the APIs
\snippet Mailbox_sample.c include

Mailbox Init Example
\snippet Mailbox_sample.c init

Message Write Example
\snippet Mailbox_sample.c write

Message Read Example
\snippet Mailbox_sample.c read

## API

\ref DRV_MAILBOX_MODULE