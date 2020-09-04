# UDMA Migration Guide {#UDMA_MIGRATION_GUIDE}

This section describes the differences between UDMA APIs of MCU+ SDK and Processor SDK RTOS (PDK).
This can be used as a migration aid when moving from Processor SDK RTOS (PDK) to MCU+ SDK.

## API changes

There are few changes in API names, structure names and macro names along with few behaviour changes with respect to SysConfig integration.
These are listed in below table.

<table>
    <tr>
        <th> PDK
        <th> MCU+ SDK
        <th> Remarks
    </tr>
    <tr>
        <td>UDMA_WAIT_FOREVER and UDMA_NO_WAIT
        <td>\ref SystemP_WAIT_FOREVER and \ref SystemP_NO_WAIT
        <td>Use SystemP macro for timeout parameter in all relevant APIs
    </tr>
    <tr>
        <td>OSAL Abstraction through function pointer using Udma_OsalPrms, Udma_OsalCachePrms and Udma_PrintFxn structures
        <td>NA
        <td>These are removed as abstraction is done through DPL layer directly
    </tr>
    <tr>
        <td>Udma_RmInitPrms
        <td>NA
        <td>Removed user override of resources. Instead the driver queries the DMSC board configuration and populate the default value during driver init
    </tr>
    <tr>
        <td>UdmaUtils_makeTrpd first param type changed from: void *
        <td>\ref UdmaUtils_makeTrpd to: uint8_t *
        <td>None
    </tr>
    <tr>
        <td>All UTC, DRU, Proxy and generic flow related interfaces
        <td>NA
        <td>These are removed as these modules are not present in AM64x/AM243x
    </tr>
    <tr>
        <td>Udma_chDequeueTdResponse
        <td>NA
        <td>This is removed as this feature is not applicable for AM64x/AM243x
    </tr>
    <tr>
        <td>Udma_flowAlloc
        <td>NA
        <td>This is removed as this feature is not applicable for AM64x/AM243x
    </tr>
    <tr>
        <td>struct Udma_DrvObj
        <td>\ref Udma_DrvObject
        <td>Renamed and is also made as an abstract structure for user
    </tr>
    <tr>
        <td>struct Udma_ChObj
        <td>\ref Udma_ChObject
        <td>Renamed and is also made as an abstract structure for user
    </tr>
    <tr>
        <td>struct Udma_RingObj
        <td>\ref Udma_RingObject
        <td>Renamed and is also made as an abstract structure for user
    </tr>
    <tr>
        <td>struct Udma_FlowObj
        <td>\ref Udma_FlowObject
        <td>Renamed and is also made as an abstract structure for user
    </tr>
    <tr>
        <td>struct Udma_EventObj
        <td>\ref Udma_EventObject
        <td>Renamed and is also made as an abstract structure for user
    </tr>
    <tr>
        <td>rmInitPrms, skipRmOverlapCheck, printFxn and osalPrms members in \ref Udma_InitPrms structure
        <td>Removed
        <td>These are removed for reasons stated in above rows
    </tr>
</table>

## Important Notes

- In MCU+ SDK, users are recommended to use SysConfig as this will greatly simplify the task of driver configuration.
    - UDMA driver init and deinit APIs are called through the SysConfig generated init code - user need not explicitly call them
    - In case of UDMA blockcopy channels,
        - User can select number of channels needed in the application through SysConfig
        - Channel open, configuration and close is automatically done by the SysConfig generated code
        - This also includes ring memory allocation and configuration
        - User can then use the created channel handle through gConfigUdma0BlkCopyChHandle like SysConfig generated globals as declared in "ti_drivers_open_close.h" and use them in other runtime APIs

## See Also

 - \ref DRIVERS_UDMA_PAGE
 - \ref EXAMPLES_DRIVERS_UDMA_MEMCPY_POLLING
 - \ref EXAMPLES_DRIVERS_UDMA_MEMCPY_INTERRUPT
 - \ref EXAMPLES_DRIVERS_UDMA_SW_TRIGGER
 - \ref EXAMPLES_DRIVERS_UDMA_CHAINING
 - \ref EXAMPLES_DRIVERS_UDMA_ADC_READ
