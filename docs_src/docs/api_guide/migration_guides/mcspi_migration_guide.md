# MCSPI Migration Guide {#MCSPI_MIGRATION_GUIDE}

This section describes the differences between MCSPI APIs of MCU+ SDK and Processor SDK RTOS (PDK).
This can be used as a migration aid when moving from Processor SDK RTOS (PDK) to MCU+ SDK.

In MCU+ SDK, the MCSPI module provides both low latency and higher level driver APIs.Refer \ref DRIVERS_MCSPI_PAGE for more details.
From the user point of view, higher level API usage is same as in PDK.APIs name begins with `MCSPI_` instead of `SPI_`.
Low latency APIs are provided on top of these higher level abstracted APIs and the usage of these APIs are showcased in \ref EXAMPLES_DRIVERS_MCSPI_PERFORMANCE_8BIT example.

## API changes

There are changes in API names, structure names and macro names. Low latency API names also begin with `MCSPI_` now. The low latency available API names and the name changes are listed below.

<table>
    <tr>
        <th> PDK
        <th> MCU+ SDK
        <th> Remarks
    </tr>
    <tr>
        <td>McSPIClkConfig
        <td>\ref MCSPI_chConfig
        <td>Configuring field \ref MCSPI_ChConfig::bitRate in \ref MCSPI_ChConfig will do the equivalent of PDK MCSPI API.
    </tr>
    <tr>
        <td>McSPIWordLengthSet
        <td>\ref MCSPI_chConfig
        <td>Configuring field \ref MCSPI_Transaction::dataSize in \ref MCSPI_ChConfig will do the equivalent of PDK MCSPI API.
    </tr>
    <tr>
        <td>McSPICSEnable
        <td>\ref MCSPI_open
        <td>CS is enabled via SysConfig.
    </tr>
    <tr>
        <td>McSPICSDisable
        <td>\ref MCSPI_open
        <td>CS is disabled via SysConfig.
    </tr>
    <tr>
        <td>McSPICSPolarityConfig
        <td>\ref MCSPI_chConfig
        <td>Configuring field \ref MCSPI_ChConfig::csPolarity in \ref MCSPI_ChConfig will do the equivalent of PDK MCSPI API.
    </tr>
    <tr>
        <td>McSPICSTimeControlSet
        <td>\ref MCSPI_chConfig
        <td>Configuring field \ref MCSPI_ChConfig::csIdleTime in \ref MCSPI_ChConfig will do the equivalent of PDK MCSPI API.
    </tr>
    <tr>
        <td>McSPICSAssert
        <td>\ref MCSPI_writeChConfReg
        <td>In MCU+SDK CS is asserted by driving FORCE field using \ref MCSPI_writeChConfReg.
    </tr>
    <tr>
        <td>McSPICSDeAssert
        <td>\ref MCSPI_writeChConfReg
        <td>In MCU+SDK CS is asserted by driving FORCE field using \ref MCSPI_writeChConfReg.
    </tr>
    <tr>
        <td>McSPIStartBitEnable
        <td>\ref MCSPI_open
        <td>Configuring field \ref MCSPI_ChConfig::startBitEnable in \ref MCSPI_ChConfig will do the equivalent of PDK MCSPI API.
    </tr>
    <tr>
        <td>McSPIStartBitPolarityConfig
        <td>\ref MCSPI_open
        <td>Configuring field \ref MCSPI_ChConfig::startBitPolarity in \ref MCSPI_ChConfig will do the equivalent of PDK MCSPI API.
    </tr>
    <tr>
        <td>McSPIStartBitDisable
        <td>\ref MCSPI_close
        <td>
    </tr>
    <tr>
        <td>McSPIControllerModeEnable
        <td>\ref MCSPI_open
        <td>Configuring field \ref MCSPI_OpenParams::msMode in \ref MCSPI_OpenParams will do the equivalent of PDK MCSPI API.
    </tr>
    <tr>
        <td>McSPIPeripheralModeEnable
        <td>\ref MCSPI_open
        <td>Configuring field \ref MCSPI_OpenParams::msMode in \ref MCSPI_OpenParams will do the equivalent of PDK MCSPI API.
    </tr>
    <tr>
        <td>McSPIControllerModeConfig
        <td>\ref MCSPI_open  \ref MCSPI_chConfig
        <td>Configuring fields \ref MCSPI_Attrs::chMode,MCSPI_Attrs::pinMode in \ref MCSPI_Attrs, field \ref MCSPI_ChConfig::trMode in \ref MCSPI_ChConfig will do the equivalent of PDK MCSPI API.
    </tr>
    <tr>
        <td>McSPIChannelEnable
        <td>\ref MCSPI_writeChCtrlReg
        <td>In MCU+SDK Channel is enabled by setting EN field using \ref MCSPI_writeChCtrlReg.
    </tr>
    <tr>
        <td>McSPIChannelDisable
        <td>\ref MCSPI_writeChCtrlReg
        <td>In MCU+SDK Channel is enabled by clearing EN field using \ref MCSPI_writeChCtrlReg.
    </tr>
    <tr>
        <td>McSPIReset
        <td>\ref MCSPI_open
        <td>
    </tr>
    <tr>
        <td>McSPITurboModeEnable
        <td>None
        <td>In MCU+SDK API not supported
    </tr>
    <tr>
        <td>McSPITurboModeDisable
        <td>None
        <td>In MCU+SDK API not supported
    </tr>
    <tr>
        <td>McSPITxFIFOConfig
        <td>\ref MCSPI_enableTxFIFO
        <td>Only API name change
    </tr>
    <tr>
        <td>McSPIRxFIFOConfig
        <td>\ref MCSPI_enableRxFIFO
        <td>Only API name change
    </tr>
    <tr>
        <td>McSPIFIFOTrigLvlSet
        <td>M\ref MCSPI_open
        <td>In MCU+SDK, FIFO trigger level is set to fixed value based on transfer mode.
    </tr>
    <tr>
        <td>McSPIWordCountSet
        <td>\ref MCSPI_transfer
        <td>
    </tr>
    <tr>
        <td>McSPIDMAEnable
        <td>None
        <td>In MCU+SDK API not supported
    </tr>
    <tr>
        <td>McSPIDMADisable
        <td>None
        <td>In MCU+SDK API not supported
    </tr>
    <tr>
        <td>McSPIIntEnable
        <td>\ref MCSPI_transfer
        <td>
    </tr>
    <tr>
        <td>McSPIIntDisable
        <td>None
        <td>Interrupt mode is supported only in higher level driver APIs and driver manages ISR.
    </tr>
    <tr>
        <td>McSPIInitDelayConfig
        <td>\ref MCSPI_open
        <td>Configuring field \ref MCSPI_Attrs::initDelay in \ref MCSPI_Attrs will do the equivalent of PDK MCSPI API.
    </tr>
    <tr>
        <td>McSPITransmitData
        <td>\ref MCSPI_writeTxDataReg
        <td>Only API name change
    </tr>
    <tr>
        <td>McSPIReceiveData
        <td>\ref MCSPI_readRxDataReg
        <td>Only API name change
    </tr>
    <tr>
        <td>McSPIIntStatusGet
        <td>None
        <td>Interrupt mode is supported only in higher level driver APIs and driver manages ISR.
    </tr>
    <tr>
        <td>McSPIIntStatusClear
        <td>None
        <td>Interrupt mode is supported only in higher level driver APIs and driver manages ISR.
    </tr>
    <tr>
        <td>McSPIChannelStatusGet
        <td>\ref MCSPI_readChStatusReg
        <td>Only API name change
    </tr>
    <tr>
        <td>McSPIMultipleWordAccessConfig
        <td>None
        <td>In MCU+SDK API not supported
    </tr>
    <tr>
        <td>McSPIFIFODatManagementConfig
        <td>None
        <td>In MCU+SDK API not supported
    </tr>
    <tr>
        <td>MCSPISysConfigSetup
        <td>\ref MCSPI_open
        <td>
    </tr>
    <tr>
        <td>MCSPIPinDirSet
        <td>\ref MCSPI_open
        <td>Configuring field \ref MCSPI_ChConfig::inputSelect, MCSPI_ChConfig::dpe0 and MCSPI_ChConfig::dpe1 in \ref MCSPI_ChConfig will do the equivalent of PDK MCSPI API.
    </tr>
    <tr>
        <td>MCSPISingleChModeEnable
        <td>\ref MCSPI_open
        <td>Configuring field \ref MCSPI_Attrs::chMode in \ref MCSPI_Attrs will do the equivalent of PDK MCSPI API.
    </tr>
    <tr>
        <td>MCSPIMultiChModeEnable
        <td>\ref MCSPI_open
        <td>Configuring field \ref MCSPI_Attrs::chMode in \ref MCSPI_Attrs will do the equivalent of PDK MCSPI API.
    </tr>
    <tr>
        <td>McSPISetPeripheralChipSel
        <td>\ref MCSPI_open
        <td>Configuring field \ref MCSPI_ChConfig::slvCsSelect in \ref MCSPI_ChConfig will do the equivalent of PDK MCSPI API.
    </tr>
    <tr>
        <td>McSPIGetChannelCtrl
        <td>\ref MCSPI_readChCtrlReg
        <td>Only API name change
    </tr>
    <tr>
        <td>McSPISetChannelCtrl
        <td>\ref MCSPI_writeChCtrlReg
        <td>Only API name change
    </tr>
    <tr>
        <td>McSPIGetChannelConf
        <td>\ref MCSPI_readChConf
        <td>Only API name change
    </tr>
    <tr>
        <td>McSPISetChannelConf
        <td>\ref MCSPI_writeChConfReg
        <td>Only API name change
    </tr>
    <tr>
        <td>Not Available
        <td>\ref MCSPI_getBaseAddr
        <td>This API is only present in MCU+SDK
    </tr>
    <tr>
        <td>Not Available
        <td>\ref MCSPI_reConfigFifo
        <td>This API is only present in MCU+SDK
    </tr>
    <tr>
        <td>Not Available
        <td>\ref MCSPI_getBufWidthShift
        <td>This API is only present in MCU+SDK
    </tr>
</table>

## Important Notes

- In MCU+ SDK, Users are recommended to use SysConfig as this will greatly simplify the task of driver configuration. SysConfig can be used to configure below parameters for both low latency and higher level driver usage.
- MCSPI module configuration parameters like Transmit/Receive mode, MISO, MOSI pin selection, Clock config,Setting word length and others.
- MCSPI channel configurations like CS enable, CS Polarity, CS Time Control and others.
- In the advanced configurations option you can configure initial delay for first transfer, transfer mode and timeout.
- MCSPI instances and pin configurations.
- Interrupt enable/disable.
- To use low latency APIs user should follow below steps.
    - User should configure "Interrupt mode enable" option in SysConfig to "Polling"
    - User should get the base address of MCSPI module by calling \ref MCSPI_getBaseAddr API.
    - This base address should be used to call low latency APIs.
    - Please refer \ref EXAMPLES_DRIVERS_MCSPI_PERFORMANCE_8BIT example for the usage of low latency APIs.

## See Also

- \ref DRIVERS_MCSPI_PAGE
- \ref EXAMPLES_DRIVERS_MCSPI_PERFORMANCE_32BIT
- \ref EXAMPLES_DRIVERS_MCSPI_LOOPBACK
