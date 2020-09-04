# GPIO Migration Guide {#GPIO_MIGRATION_GUIDE}

This section describes the differences between GPIO APIs of MCU+ SDK and Processor SDK RTOS (PDK).
This can be used as a migration aid when moving from Processor SDK RTOS (PDK) to MCU+ SDK.

In MCU+ SDK, the GPIO APIs are simplified. Most of the CSL and LLD are merged into single API. Refer \ref DRIVERS_GPIO_PAGE for more details.

## API changes

There are changes in functions names, structure names and macro names. The changes in function names are listed below.

<table>
    <tr>
        <th> PDK
        <th> MCU+ SDK
        <th> Change Description / Remarks
    </tr>
    <tr>
        <td>GPIO_setConfig
        <td>\ref GPIO_setDirMode
        <td>In MCU+SDK, this GPIO_setConfig is separated into sub APIs such as \ref GPIO_setTrigType, \ref GPIO_setDirMode.
    </tr>
    <tr>
        <td>GPIO_write
        <td>\ref GPIO_pinWriteHigh
        <td>API rename along with interface parameter changes.
    </tr>
    <tr>
        <td>GPIO_write
        <td>\ref GPIO_pinWriteLow
        <td>API rename along with interface parameter changes.
    </tr>
    <tr>
        <td>GPIO_read
        <td>\ref GPIO_pinRead
        <td>API rename along with interface parameter changes.
    </tr>
\cond SOC_AM64X || SOC_AM243X || SOC_AM263X
    <tr>
        <td>None
        <td>\ref GPIO_pinOutValueRead
        <td>In MCU+SDK, this API can be used to determine the output logic level(value) on a specified GPIO pin.
    </tr>
\endcond
    <tr>
        <td>GPIO_setConfig
        <td>\ref GPIO_setTrigType
        <td>In MCU+SDK, this GPIO_setConfig is separated into sub APIs such as \ref GPIO_setTrigType, \ref GPIO_setDirMode.
    </tr>
\cond SOC_AM64X || SOC_AM243X || SOC_AM263X
    <tr>
        <td>None
        <td>\ref GPIO_bankIntrEnable
        <td>In MCU+SDK, this API can be used to enable the bank interrupt.
    </tr>
    <tr>
        <td>None
        <td>\ref GPIO_bankIntrDisable
        <td>In MCU+SDK, this API can be used to disable the bank interrupt.
    </tr>
    <tr>
        <td>None
        <td>\ref GPIO_getIntrStatus
        <td>In MCU+SDK, this API can be used to determine the enabled interrupt status of a specified pin.
    </tr>
    <tr>
        <td>GPIO_clearInt
        <td>\ref GPIO_clearIntrStatus
        <td>In MCU+SDK, this API can be used to clear the enabled interrupt status of a specified pin.
    </tr>
    <tr>
        <td>None
        <td>\ref GPIO_getBankIntrStatus
        <td>In MCU+SDK, this API is used to return the interrupt status of the specified bank.
    </tr>
    <tr>
        <td>None
        <td>\ref GPIO_clearBankIntrStatus
        <td>In MCU+SDK, this API is used to clear the interrupt status of the specified bank.
    </tr>
\endcond
    <tr>
        <td>GPIO_disableInt
        <td>None
        <td>In MCU+SDK, Bank interrupt disable is available instead of specific pin.
    </tr>
    <tr>
        <td>GPIO_enableInt
        <td>None
        <td>In MCU+SDK, Bank interrupt enable is available instead of specific pin.
    </tr>
    <tr>
        <td>GPIO_init
        <td>None
        <td>Removed as abstraction is done with simplified APIs.
    </tr>
    <tr>
        <td>GPIO_setCallback
        <td>None
        <td>Not Applicable.
    </tr>
    <tr>
        <td>GPIO_toggle
        <td>None
        <td>None
    </tr>
</table>

## Important Notes

- In MCU+ SDK, users are recommended to use SysConfig as this will greatly simplify the task of driver configuration.

## See Also

 - \ref DRIVERS_GPIO_PAGE
 - \ref EXAMPLES_DRIVERS_GPIO_LED_BLINK
 - \ref EXAMPLES_DRIVERS_GPIO_INPUT_INTERRUPT
