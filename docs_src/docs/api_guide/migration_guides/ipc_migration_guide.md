# IPC and Mailbox Migration Guide {#IPC_MIGRATION_GUIDE}

This section describes the differences between IPC APIs of MCU+ SDK and Processor SDK RTOS (PDK).
This can be used as migration aid when moving from Processor SDK RTOS (PDK) to MCU+ SDK.

In MCU+ SDK, the IPC APIs are simplified and consolidated into below two APIs
- \ref DRIVERS_IPC_NOTIFY_PAGE to provide exterme low latency IPC API
  - This can be considered as new API that can be used when sub-micro second IPC latency is needed in some hard real-time applications
- \ref DRIVERS_IPC_RPMESSAGE_PAGE to provide higher level message passing IPC API
  - \ref DRIVERS_IPC_RPMESSAGE_PAGE replaces both IPC LLD and mailbox LLD from PDK.

## API changes

There are changes in functions names, structure names and macro names. The changes in function names are listed below.

<table>
    <tr>
        <th> PDK
        <th> MCU+ SDK
        <th> Change Description / Remarks
    </tr>
    <tr><td colspan="3" bgcolor=#F0F0F0>**IPC LLD**</td></tr>
    <tr>
        <td>IpcInitPrms_init
        <td>\ref RPMessage_Params_init, \ref IpcNotify_Params_init
        <td>In MCU+ SDK, both IPC notify and IPC rpmsg need to be initialized. SysConfig can be used to simplify the IPC module init.
    </tr>
    <tr>
        <td>Ipc_init, RPMessage_init
        <td>\ref RPMessage_init, \ref IpcNotify_init
        <td>In MCU+ SDK, both IPC notify and IPC rpmsg need to be initialized. SysConfig can be used to simplify the IPC module init.
    </tr>
    <tr>
        <td>Ipc_deinit, RPMessage_deInit
        <td>\ref RPMessage_deInit, \ref IpcNotify_deInit
        <td>In MCU+ SDK, both IPC notify and IPC rpmsg need to be de-initialized. SysConfig can be used to simplify the IPC module init.
    </tr>
    <tr>
        <td>RPMessage_lateInit
        <td>\ref RPMessage_waitForLinuxReady
        <td>API rename
    </tr>
    <tr>
        <td>RPMessageParams_init
        <td>\ref RPMessage_CreateParams_init
        <td>API rename
    </tr>
    <tr>
        <td>RPMessage_create
        <td>\ref RPMessage_construct
        <td>Static alloc friendly API, simplified API
    </tr>
    <tr>
        <td>RPMessage_setCallback
        <td>NONE
        <td>Set callback as part of RPMessage_construct parameters
    </tr>
    <tr>
        <td>RPMessage_recv, RPMessage_recvNb
        <td>\ref RPMessage_recv
        <td>Unified API for blocking as well as non-blocking mode.
    </tr>
    <tr>
        <td>RPMessage_send
        <td>\ref RPMessage_send
        <td>API signature to match RPMessage_recv parameters and also supports timeout
    </tr>
    <tr>
        <td>RPMessage_delete
        <td>\ref RPMessage_destruct
        <td>Static alloc friendly API
    </tr>
    <tr>
        <td>RPMessage_unblock
        <td>\ref RPMessage_unblock
        <td>NO CHANGE
    </tr>
    <tr>
        <td>RPMessage_getRemoteEndPt, RPMessage_getRemoteEndPtToken
        <td>\ref RPMessage_controlEndPtCallback
        <td>Callback based mechanism to listen to announce messages from remote CPUs, to enable NORTOS implementation.
    </tr>
    <tr>
        <td>RPMessage_announce
        <td>\ref RPMessage_announce
        <td>NO CHANGE, except that "ALL" cannot be used to send annoucement to all CPUs, specific CPU ID MUST be used.
    </tr>
    <tr>
        <td>RPMessage_getMessageBufferSize, RPMessage_getObjMemRequired, RPMessage_unblockGetRemoteEndPt, Ipc_newMessageIsr, Ipc_mailboxEnableNewMsgInt, Ipc_mailboxDisableNewMsgInt
        <td>NONE
        <td>NOT needed in MCU+ SDK.
    </tr>
    <tr>
        <td>NONE
        <td>\ref RPMessage_getLocalEndPt
        <td>New APIs to complete the functionality
    </tr>
    <tr><td colspan="3" bgcolor=#F0F0F0>**Mailbox LLD**</td></tr>
    <tr>
        <td>Mailbox_initParams_init
        <td>\ref RPMessage_Params_init, \ref IpcNotify_Params_init
        <td>In MCU+ SDK, both IPC notify and IPC rpmsg need to be initialized. SysConfig can be used to simplify the IPC module init.
    </tr>
    <tr>
        <td>Mailbox_init
        <td>\ref RPMessage_init, \ref IpcNotify_init
        <td>In MCU+ SDK, both IPC notify and IPC rpmsg need to be initialized. SysConfig can be used to simplify the IPC module init.
    </tr>
    <tr>
        <td>Mailbox_deinit
        <td>\ref RPMessage_deInit, \ref IpcNotify_deInit
        <td>In MCU+ SDK, both IPC notify and IPC rpmsg need to be de-initialized. SysConfig can be used to simplify the IPC module init.
    </tr>
    <tr>
        <td>Mailbox_openParams_init
        <td>\ref RPMessage_CreateParams_init
        <td>In MCU+ SDK, create a local end point to receive messages from any remote CPU at local CPU
    </tr>
    <tr>
        <td>Mailbox_open
        <td>\ref RPMessage_construct
        <td>In MCU+ SDK, create a local end point to receive messages from any remote CPU at local CPU
    </tr>
    <tr>
        <td>Mailbox_write
        <td>\ref RPMessage_send
        <td>In MCU+ SDK, use this API to send a message to a specific remote CPU and specific end point on that CPU
    </tr>
    <tr>
        <td>Mailbox_read
        <td>\ref RPMessage_recv
        <td>In MCU+ SDK, use this API to receive messages from remote CPUs to the create local end point. The remote CPU and remote CPU end point is returned when the API returns.
    </tr>
    <tr>
        <td>Mailbox_readFlush
        <td>\ref RPMessage_recv
        <td>In MCU+ SDK, explicit flush is not needed and is taken care of in \ref RPMessage_recv.
    </tr>
    <tr>
        <td>Mailbox_close
        <td>\ref RPMessage_destruct
        <td>In MCU+ SDK, use this to close a previously created local end point.
    </tr>
    <tr>
        <td>Mailbox_GetMessageCount, Mailbox_getStats, Mailbox_enableInterrupts, Mailbox_disableInterrupts
        <td>NONE
        <td>NOT needed in MCU+ SDK.
    </tr>
    <tr>
        <td>NONE
        <td>\ref RPMessage_waitForLinuxReady, \ref RPMessage_controlEndPtCallback, \ref RPMessage_announce,
        <td>This APIs can be ignored when migrating from mailbox LLD.
    </tr>
    <tr>
        <td>NONE
        <td>\ref RPMessage_unblock, \ref RPMessage_getLocalEndPt
        <td>These APIs can be useful when writing applications using IPC rpmsg API.
    </tr>
</table>

## Important Notes

### Migration from IPC LLD API
- In MCU+ SDK use the \ref DRIVERS_IPC_RPMESSAGE_PAGE to get equivalent features of IPC LLD.
- MCU+ SDK implements the same rpmsg protocol as IPC LLD. Only the APIs are lot more simplified and enable low memory footprint applications vs PDK.

### Migration from Mailbox LLD API
 - To get equivalent features of mailbox LLD, create one rpmsg end point (equivalent to channel in mailbox LLD) and then use rpmsg APIs to send and receive message packets.
 - Unlike mailbox LLD, when using \ref DRIVERS_IPC_RPMESSAGE_PAGE, one can send a new message even if previous message is not yet read. This is possible when number of message buffers is configured as `> 1`. In order to not allow sending of new message until previous message is read, set the number of message buffers to `1` during module initialization

## See Also

\ref DRIVERS_IPC_NOTIFY_PAGE, \ref DRIVERS_IPC_RPMESSAGE_PAGE, \ref IPC_GUIDE