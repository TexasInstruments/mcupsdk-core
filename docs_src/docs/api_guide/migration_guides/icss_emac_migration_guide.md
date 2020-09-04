# ICSS-EMAC Migration Guide {#ICSS_EMAC_MIGRATION_GUIDE}

This list describes the differences between ICSS-EMAC driver of MCU+ SDK and ICSS-EMAC driver from the PDK.

## API changes

There are changes in API names, structure names and macro names. All names begin with `ICSS_EMAC_` now. In PDK driver, user had to include multiple header files from the driver. In MCU+ SDK, only a single include file is needed, which is `${SDK_INSTALL_PATH}/source/networking/icss_emac/icss_emac.h`. From the user point of view, IOCTL, Rx and Tx related API usage is same as in PDK. The initialization code for icss-emac has changed, and is described in the points below. All the APIs from PDK driver are not included, only the ones used in the networking libraries are added in the MCU+ SDK's driver. The changes in API names are listed below.

<table>
    <tr>
        <th> PDK
        <th> MCU+ SDK
    </tr>
    <tr>
        <td>Not available
        <td>\ref ICSS_EMAC_init
    </tr>
    <tr>
        <td>Not available
        <td>\ref ICSS_EMAC_deinit
    </tr>
    <tr>
        <td>Not available
        <td>\ref ICSS_EMAC_Params_init
    </tr>
    <tr>
        <td>ICSS_EmacInit
        <td>\ref ICSS_EMAC_open
    </tr>
    <tr>
        <td>ICSS_EmacDeInit
        <td>\ref ICSS_EMAC_close
    </tr>
    <tr>
        <td>ICSS_EmacIoctl
        <td>\ref ICSS_EMAC_ioctl
    </tr>
    <tr>
        <td>ICSS_EmacRxPktGet
        <td>\ref ICSS_EMAC_rxPktGet
    </tr>
    <tr>
        <td>ICSS_EmacRxPktInfo
        <td>\ref ICSS_EMAC_rxPktInfo
    </tr>
    <tr>
        <td>ICSS_EmacTxPacket
        <td>\ref ICSS_EMAC_txPacket
    </tr>
</table>

## Important Notes

- In PDK, for initializing the handle for icss-emac, user had to manually perform dynamic allocation of memory needed for the icss-emac internal object. In MCU+ SDK, all allocation is static and user does not have to worry about memory allocation. \ref ICSS_EMAC_Params should be filled before making the \ref ICSS_EMAC_open function call to get the handle, which will be used for subsequent API calls.

- In PDK, there were APIs to register few callback functions, but now those are consolidated in the \ref ICSS_EMAC_Params and should be provided during the \ref ICSS_EMAC_open, if relevant.

- From PDK driver, the `ICSS_EMAC_Object` structure was used directly by user to get or set some information. Now the structure is opaque to user and can not be modified by the user.

- `ICSS_EmacRxPktInfo2` and `ICSS_EmacTxPacketEnqueue` APIs are not available for use. These APIs are used internally by \ref ICSS_EMAC_rxPktInfo and \ref ICSS_EMAC_txPacket. The recommendation is to use \ref ICSS_EMAC_rxPktInfo and \ref ICSS_EMAC_txPacket directly.

- There are differences in TI-RTOS OSAL of PDK, and \ref KERNEL_DPL_PAGE in MCU+ SDK. Changes were made to address this difference. Task, Semaphore and Hwi abstraction is similar to that in TI-RTOS osal. For task, the stack needs to be statically allocated. The stack size is 8KB for all tasks, and it will need tuning as needed in future. There is no impact on the use of APIs.

- HWI creation for Link, Rx and Tx is done inside the driver. In PDK, the interrupt creation duty was with the user, but now the user can just give the host interrupt number to be configured and the driver takes care of creating HWI and assigning the interrupt handler internally.

- Task creation for Link, Rx and Tx is done inside the driver. In PDK, the user had to create tasks with the functions provided by icss-emac. Now the tasks are created from the driver, and user can provide the task priority for each task as an input.

- icss-emac needs information about the memory map of firmware which will be used with the icss-emac. User needs to provide the same information in the form of \ref ICSS_EMAC_FwStaticMmap and \ref ICSS_EMAC_FwDynamicMmap while filling the \ref ICSS_EMAC_Params structure. There are some other macros related to VLAN filtering and multicast filtering as well needed by icss-emac, for which should new structures are added and they should be passed if IOCTL for VLAN and multicast filtering are going to be called.

- `ICSS_EmacBaseAddressHandle_T` is removed now. It was replicating the information present in \ref PRUICSS_HwAttrs, so that is used directly wherever needed.

- Macros for HW register access like `HWREG`, `HWREGH` and `HWREGB` are deprecated from CSL in MCU+ SDK. They are replaced with the recommended replacements from `${SDK_INSTALL_PATH}/source/drivers/hw_include/hw_types.h`.

- TTS (Time-triggered Send) related functionality is not available in MCU+ SDK driver.

- Firmware Learning mode is not available in MCU+ SDK driver.

- Time based Interrupt Pacing support is not available in MCU+ SDK driver.

## See Also

\ref NETWORKING_ICSS_EMAC_MODULE