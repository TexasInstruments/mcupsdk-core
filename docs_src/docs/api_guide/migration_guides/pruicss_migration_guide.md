# PRUICSS Driver Migration Guide {#PRUICSS_MIGRATION_GUIDE}

This list describes the differences between PRUICSS driver of MCU+ SDK and PRUSS driver from the PDK.

## API changes

There are changes in API names, structure names and macro names. All names begin with `PRUICSS_` now. Arguments to APIs are same as earlier. All the APIs from PDK driver are not included, only the ones used in the examples, Industrial Protocol FW HAL(Firmware and Hardware Abstraction Layer), ICSS-EMAC and ICSS Timesync an are added in the MCU+ SDK's driver. The changes in API names are listed below.

<table>
    <tr>
        <th> PDK
        <th> MCU+ SDK
    </tr>
    <tr>
        <td>Not available
        <td>\ref PRUICSS_init
    </tr>
    <tr>
        <td>Not available
        <td>\ref PRUICSS_deinit
    </tr>
    <tr>
        <td>PRUICSS_create
        <td>\ref PRUICSS_open
    </tr>
    <tr>
        <td>Not available
        <td>\ref PRUICSS_close
    </tr>
    <tr>
        <td>PRUICSS_pruIntcInit
        <td>\ref PRUICSS_intcInit
    </tr>
    <tr>
        <td>PRUICSS_registerIrqHandler
        <td>\ref PRUICSS_registerIrqHandler
    </tr>
    <tr>
        <td>PRUICSS_pruReset
        <td>\ref PRUICSS_resetCore
    </tr>
    <tr>
        <td>PRUICSS_pruDisable
        <td>\ref PRUICSS_disableCore
    </tr>
    <tr>
        <td>PRUICSS_pruEnable
        <td>\ref PRUICSS_enableCore
    </tr>
    <tr>
        <td>PRUICSS_pruInitMemory
        <td>\ref PRUICSS_initMemory
    </tr>
    <tr>
        <td>PRUICSS_pruWriteMemory
        <td>\ref PRUICSS_writeMemory
    </tr>
    <tr>
        <td>PRUICSS_pruReadMemory
        <td>\ref PRUICSS_readMemory
    </tr>
    <tr>
        <td>PRUICSS_pruSendEvent
        <td>\ref PRUICSS_sendEvent
    </tr>
    <tr>
        <td>PRUICSS_pruWaitEvent
        <td>\ref PRUICSS_waitEvent
    </tr>
    <tr>
        <td>PRUICSS_pruClearEvent
        <td>\ref PRUICSS_clearEvent
    </tr>
    <tr>
        <td>PRUICSS_pruSendWaitCearEvent
        <td>\ref PRUICSS_sendWaitClearEvent
    </tr>
    <tr>
        <td>PRUICSS_getICSSVersion
        <td>\ref PRUICSS_getVersion
    </tr>
    <tr>
        <td>PRUICSS_setConstantTblEntry
        <td>\ref PRUICSS_setConstantTblEntry
    </tr>
    <tr>
        <td>PRUICSS_setIepClkSrc
        <td>\ref PRUICSS_setIepClkSrc
    </tr>
</table>

## Important Notes

- The INTC module in ICSSG supports 160 events and 20 channels, but the PDK driver only supported 64 events and 10 channels. This support has been added in the MCU+ SDK driver for ICSSG. All changes are internal to the driver implementation. API usage for INTC initialization is same as earlier.

- Driver implementation uses Semaphore and Hwi. There are differences in TI-RTOS OSAL of PDK, and \ref KERNEL_DPL_PAGE in MCU+ SDK. Changes were made to address this difference. There is no impact on the use of APIs.

- CSL macros were used from `pdk/packages/ti/csl/src/ip/icss/V1` in PDK driver. In MCU+ SDK, the CSL for ICSSG has been taken from `pdk/packages/ti/csl/src/ip/icss/V3`, and the file name in MCU+ SDK is `${SDK_INSTALL_PATH}/source/drivers/pruicss/g_v0/cslr_icss_g.h`.

- Macros for HW register access like `HWREG`, `HWREGH` and `HWREGB` are deprecated from CSL in MCU+ SDK. They are replaced with the recommended replacements from `${SDK_INSTALL_PATH}/source/drivers/hw_include/hw_types.h`.

- SysConfig has been added to initialize PRUICSS module and initialize the Core clock. The config name used in SysConfig should be used for \ref PRUICSS_open call. Enum for PRUICSS instance number has been removed.

- The member names for \ref PRUICSS_HwAttrs have been changed. There are few additions and deletions of members.

- Few APIs(like \ref PRUICSS_setIcssCfgMiiMode, \ref PRUICSS_setGpiMode, etc) have been added to aid users in modifying some of the configuration registers.

- It is recommended to use \ref PRUICSS_getAttrs() for getting HwAttrs structure instead of using it directly from handle.

## See Also

\ref DRV_PRUICSS_MODULE