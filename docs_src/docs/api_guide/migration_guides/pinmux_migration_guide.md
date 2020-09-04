# PINMUX Migration Guide {#PINMUX_MIGRATION_GUIDE}

This section describes the differences between PINMUX APIs of MCU+ SDK and Processor SDK RTOS (PDK).
This can be used as a migration aid when moving from Processor SDK RTOS (PDK) to MCU+ SDK.
In PDK, these APIs are part of the board module in header files board_pinmux.h and AM64x_pinmux.h.

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
        <td>Board_init API with Board_initCfg set to BOARD_INIT_PINMUX_CONFIG, BOARD_INIT_PINMUX_CONFIG_MAIN or BOARD_INIT_PINMUX_CONFIG_MCU
        <td>\ref Pinmux_config
        <td>In PDK this was abstracted using the Board_init API and the pinmux data was part of the library. There is no separate API in PDK to provide user pinmux data.
    </tr>
    <tr>
        <td>pinmuxPerCfg_t
        <td>\ref Pinmux_PerCfg_t
        <td>Renamed and structure members renamed to offset and settings
    </tr>
    <tr>
        <td>pinmuxModuleCfg_t and pinmuxBoardCfg_t
        <td>NA
        <td>Removed. User should use the granular \ref Pinmux_config API instead
    </tr>
</table>

## Important Notes

Pinmux configuration is already integrated with each driver SysConfig code generation. Typically user need not explicitly call this pinmux API.

## See Also

- \ref DRIVERS_PINMUX_PAGE
