# UART Migration Guide {#UART_MIGRATION_GUIDE}

This section describes the differences between UART APIs of MCU+ SDK and Processor SDK RTOS (PDK).
This can be used as a migration aid when moving from Processor SDK RTOS (PDK) to MCU+ SDK.

In MCU+ SDK, the UART module provides higher level driver APIs.Refer \ref DRIVERS_UART_PAGE for more details.

## API changes

There are changes in functions names, structure names and macro names. The changes in function names are listed below.

<table>
    <tr>
        <th> PDK
        <th> MCU+ SDK
        <th> Change Description / Remarks
    </tr>
    <tr>
        <td>UART_init
        <td>\ref UART_init
        <td>NO CHANGE
    </tr>
    <tr>
        <td>None
        <td>\ref UART_deinit
        <td>In MCU+SDK, this API can be used to de-initialize the UART module.
    </tr>
    <tr>
        <td>UART_open
        <td>\ref UART_open
        <td>NO CHANGE
    </tr>
    <tr>
        <td>UART_close
        <td>\ref UART_close
        <td>NO CHANGE
    </tr>
    <tr>
        <td>UART_write
        <td>\ref UART_write
        <td>NO CHANGE
    </tr>
    <tr>
        <td>UART_read
        <td>\ref UART_read
        <td>NO CHANGE
    </tr>
    <tr>
        <td>UART_writeCancel
        <td>\ref UART_writeCancel
        <td>NO CHANGE
    </tr>
    <tr>
        <td>UART_readCancel
        <td>\ref UART_readCancel
        <td>NO CHANGE
    </tr>
    <tr>
        <td>None
        <td>\ref UART_getHandle
        <td>In MCU+SDK, this API can be used to get the handle of an open UART instance from the instance index.
    </tr>
    <tr>
        <td>None
        <td>\ref UART_flushTxFifo
        <td>In MCU+SDK, this API is used to flush a TX FIFO of peripheral specified by the UART handle
    </tr>
    <tr>
        <td>UART_Params_init
        <td>\ref UART_Params_init
        <td>NO CHANGE
    </tr>
    <tr>
        <td>UART_transactionInit
        <td>\ref UART_Transaction_init
        <td>API rename
    </tr>
    <tr>
        <td>UART_control
        <td>None
        <td>This is a dummy function in PDK also.
    </tr>
    <tr>
        <td>UART_writePolling
        <td>\ref UART_write
        <td>Configuring field \ref UART_Params::transferMode to Polled mode will do the equivalent of PDK UART API.
    </tr>
    <tr>
        <td>UART_readPolling
        <td>\ref UART_read
        <td>Configuring field \ref UART_Params::transferMode to Polled mode will do the equivalent of PDK UART API.
    </tr>
    <tr>
        <td>UART_read2
        <td>\ref UART_read
        <td>API rename
    </tr>
    <tr>
        <td>UART_write2
        <td>\ref UART_write
        <td>API rename
    </tr>
</table>

## Important Notes

- In MCU+ SDK, users are recommended to use SysConfig as this will greatly simplify the task of driver configuration.
\cond SOC_AM64X || SOC_AM243X || SOC_AM263X
- In MCU+ SDK, \ref UART_ReadReturnMode is introduced as compared to UART_ReturnMode in PDK.
\endcond
- In MCU+ SDK, UART_DataMode is not supported.

## See Also

 - \ref DRIVERS_UART_PAGE
 - \ref EXAMPLES_DRIVERS_UART_ECHO
 - \ref EXAMPLES_DRIVERS_UART_ECHO_CALLBACK
