# DDR Migration Guide {#DDR_MIGRATION_GUIDE}

This section describes the differences between DDR APIs of MCU+ SDK and Processor SDK RTOS (PDK).
This can be used as a migration aid when moving from Processor SDK RTOS (PDK) to MCU+ SDK.
In PDK, these APIs are part of the board module in header files board.h and board_ddr.h.

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
        <td>Board_init API with Board_initCfg set to BOARD_INIT_DDR
        <td>\ref DDR_init
        <td>In PDK this was abstracted using the Board_init API. There is no separate API in PDK to provide configure DDR.
    </tr>
</table>

## Important Notes

- In MCU+ SDK, Users are recommended to use SysConfig to configure driver parameters as this will greatly simplify the task of driver configuration.

## See Also

- \ref DRIVERS_DDR_PAGE
