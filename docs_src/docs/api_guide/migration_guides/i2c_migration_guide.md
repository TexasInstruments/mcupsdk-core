# I2C Migration Guide {#I2C_MIGRATION_GUIDE}

This section describes the differences between I2C APIs of MCU+ SDK and Processor SDK RTOS (PDK).
This can be used as a migration aid when moving from Processor SDK RTOS (PDK) to MCU+ SDK.

In MCU+ SDK, the I2C module provides higher level driver APIs.Refer \ref DRIVERS_I2C_PAGE for more details.

## API changes

There are changes in functions names, structure names and macro names. The changes in function names are listed below.

<table>
    <tr>
        <th> PDK
        <th> MCU+ SDK
        <th> Change Description / Remarks
    </tr>
    <tr>
        <td>I2C_init
        <td>\ref I2C_init
        <td>NO CHANGE
    </tr>
    <tr>
        <td>I2C_deinit
        <td>\ref I2C_deinit
        <td>NO CHANGE
    </tr>
    <tr>
        <td>I2C_Params_init
        <td>\ref I2C_Params_init
        <td>NO CHANGE
    </tr>
    <tr>
        <td>I2C_open
        <td>\ref I2C_open
        <td>NO CHANGE
    </tr>
    <tr>
        <td>I2C_transactionInit
        <td>\ref I2C_Transaction_init
        <td>API rename
    </tr>
    <tr>
        <td>I2C_transfer
        <td>\ref I2C_transfer
        <td>NO CHANGE
    </tr>
    <tr>
        <td>None
        <td>\ref I2C_probe
        <td>In MCU+SDK, this API can be used to probe I2C targets.
    </tr>
    <tr>
        <td>None
        <td>\ref I2C_setBusFrequency
        <td>In MCU+SDK, this API can be used to set the bus frequency.
    </tr>

\cond SOC_AM64X || SOC_AM243X
    <tr>
        <td>None
        <td>\ref I2C_recoverBus
        <td>In MCU+SDK, this API can be used to recover the bus in case of error.
    </tr>
\endcond
    <tr>
        <td>I2C_close
        <td>\ref I2C_close
        <td>NO CHANGE
    </tr>
    <tr>
        <td>None
        <td>\ref I2C_getHandle
        <td>In MCU+SDK, this API can be used to get the handle of an open I2C instance from the instance index.
    </tr>
    <tr>
        <td>I2C_control
        <td>None
        <td>In MCU+SDK, this API is separated into 3 sub APIs. I2C_probe, I2C_setBusFrequency and I2C_recoverBus.
    </tr>
</table>

## Important Notes

- In MCU+ SDK, Users are recommended to use SysConfig to configure driver parameters as this will greatly simplify the task of driver configuration.

## See Also

 - \ref DRIVERS_I2C_PAGE
 \cond SOC_AM64X || SOC_AM243X || SOC_AM263X || SOC_AM263PX || SOC_AM65X || SOC_AM261X
 - \ref EXAMPLES_DRIVERS_I2C_LED_BLINK
 \endcond

 \cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
 - \ref EXAMPLES_DRIVERS_I2C_READ_V1
 \endcond

 \cond SOC_AM64X || SOC_AM243X
 - \ref EXAMPLES_DRIVERS_I2C_READ
 \endcond

 - \ref EXAMPLES_DRIVERS_I2C_TEMPERATURE

