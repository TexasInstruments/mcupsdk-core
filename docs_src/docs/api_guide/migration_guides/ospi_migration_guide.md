# OSPI Migration Guide {#OSPI_MIGRATION_GUIDE}

This section describes the differences between OSPI APIs of MCU+ SDK and Processor SDK RTOS (PDK).
This can be used as a migration aid when moving from Processor SDK RTOS (PDK) to MCU+ SDK.

In MCU+ SDK, the OSPI module provides higher level driver APIs.Refer \ref DRIVERS_OSPI_PAGE for more details.
From the user point of view, higher level API usage is same as in PDK.APIs name begins with `OSPI_` instead of `SPI_`.

## API changes

There are changes in API names, structure names and macro names. The changes in function names are listed below.

<table>
    <tr>
        <th> PDK
        <th> MCU+ SDK
        <th> Remarks
    </tr>
    <tr>
        <td>SPI_init
        <td>\ref OSPI_init
        <td>API rename
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_deinit
        <td>In MCU+SDK, this API can be used to de-initializes the OSPI module
    </tr>
    <tr>
        <td>SPI_open
        <td>\ref OSPI_open
        <td>API rename
    </tr>
    <tr>
        <td>SPI_Params_init
        <td>\ref OSPI_Params_init
        <td>API rename
    </tr>
    <tr>
        <td>SPI_close
        <td>\ref OSPI_close
        <td>API rename
    </tr>
    <tr>
        <td>SPI_close
        <td>\ref OSPI_close
        <td>API rename
    </tr>
    <tr>
        <td>SPI_serviceISR
        <td>None
        <td>In MCU+SDK, ISR is handled inside the driver.
    </tr>
    <tr>
        <td>SPI_transfer
        <td>None
        <td>In MCU+SDK, SPI_transfer is separated into sub APIs \ref OSPI_readDirect, \ref OSPI_writeDirect, \ref OSPI_readIndirect, \ref OSPI_writeIndirect
    </tr>
    <tr>
        <td>SPI_transferCancel
        <td>None
        <td>In MCU+SDK, API is not supported.
    </tr>
    <tr>
        <td>SPI_control
        <td>None
        <td>In MCU+SDK, SPI_control is separated into sub APIs \ref OSPI_readCmd, \ref OSPI_writeCmd, \ref OSPI_enableDdrRdCmds and so on. API are listed below.
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_getHandle
        <td>In MCU+SDK, this API can be used to return the handle of an open OSPI Instance from the instance index
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_readDirect
        <td>In MCU+SDK, this API can be used to perform direct reads from the flash using DAC controller
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_readIndirect
        <td>In MCU+SDK, this API can be used to perform indirect reads from the flash using INDAC controller
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_writeDirect
        <td>In MCU+SDK, this API can be used to perform direct writes to the flash using DAC controller
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_writeIndirect
        <td>In MCU+SDK, this API can be used to perform indirect writes to the flash using INDAC controller
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_readCmd
        <td>In MCU+SDK, this API can be used to send specific commands and receive related data from flash
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_writeCmd
        <td>In MCU+SDK, this API can be used to send specific commands and related data to flash
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_Transaction_init
        <td>In MCU+SDK, this API can be used to initialize the \ref OSPI_Transaction structure
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_ReadCmdParams_init
        <td>In MCU+SDK, this API can be used to initialize the \ref OSPI_ReadCmdParams structure
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_WriteCmdParams_init
        <td>In MCU+SDK, this API can be used to initialize the \ref OSPI_WriteCmdParams structure
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_isDacEnable
        <td>In MCU+SDK, this API can be used to check if the Direct Access Controller is enabled
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_isDmaEnable
        <td>In MCU+SDK, this API can be used to check if DMA is enabled for reads
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_isIntrEnable
        <td>In MCU+SDK, this API can be used to check if interrupts are enabled
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_isPhyEnable
        <td>In MCU+SDK, this API can be used to check if the OSPI PHY controller is enabled
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_isDtrEnable
        <td>In MCU+SDK, this API can be used to check if the Dual Transfer Rate is enabled
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_enableDDR
        <td>In MCU+SDK, this API can be used to enable the Dual Data Rate (DDR)
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_enableSDR
        <td>In MCU+SDK, this API can be used to enable the Single Data Rate (SDR)
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_enableDdrRdCmds
        <td>In MCU+SDK, this API can be used to set DDR bit in INSTR_RD register for RD commands
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_setRdDataCaptureDelay
        <td>In MCU+SDK, this API can be used to set read data capture cycles in the OSPI controller
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_setNumAddrBytes
        <td>In MCU+SDK, this API can be used to set the number of bytes used to send address while reading or writing to flash memory
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_setDeviceSize
        <td>In MCU+SDK, this API can be used to set the block size and page size of the flash to the device size register in OSPI
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_setCmdDummyCycles
        <td>In MCU+SDK, this API can be used to set appropriate dummy cycles to be used while sending STIG commands to flash
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_setReadDummyCycles
        <td>In MCU+SDK, this API can be used to set appropriate dummy cycles for flash read
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_setPhyEnableSuccess
        <td>In MCU+SDK, this API can be used to set the phyEnableSuccess field in \ref OSPI_Object. Has to be called from flash driver
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_getPhyEnableSuccess
        <td>In MCU+SDK, this API can be used to fetch the phyEnableSuccess field in \ref OSPI_Object.
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_cmdModeBitSet
        <td>In MCU+SDK, this API can be used to set command mode bit
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_rdModeBitSet
        <td>In MCU+SDK, this API can be used to set Read mode bit
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_setDualOpCodeMode
        <td>In MCU+SDK, this API can be used to set OSPI controller to use dual byte opcodes
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_setXferOpCodes
        <td>In MCU+SDK, this API can be used to set the opcodes for reading and page programming the flash
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_setCmdExtType
        <td>In MCU+SDK, this API can be used to set the type of command extension used in dual byte opcode mode
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_enableDacMode
        <td>In MCU+SDK, this API can be used to enable the Direct Access Mode
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_disableDacMode
        <td>In MCU+SDK, this API can be used to disable the Direct Access Mode
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_getFlashDataBaseAddr
        <td>In MCU+SDK, this API can be used to get the SOC mapped data base address of the flash
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_phyTuneDDR
        <td>In MCU+SDK, this API can be used to tune the OSPI PHY for DDR mode to set optimal PHY parameters
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_phyTuneGrapher
        <td>In MCU+SDK, this API takes a 4x128x128 array and fills it with TX RX DLL data for graphing purpose
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_phyGetTuningData
        <td>In MCU+SDK, this API can be used to return the address to the attack vector buf required for tuning the PHY
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_phyReadAttackVector
        <td>In MCU+SDK, this API can be used to check if the attack vector, or the data used for tuning the PHY is present at an offset in the flash
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_enablePhy
        <td>In MCU+SDK, this API can be used to enable the PHY
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_disablePhy
        <td>In MCU+SDK, this API can be used to disable the PHY
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_enablePhyPipeline
        <td>In MCU+SDK, this API can be used to enable the PHY Pipeline
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_disablePhyPipeline
        <td>In MCU+SDK, this API can be used to disable the PHY Pipeline
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_norFlashInit1s1s1s
        <td>In MCU+SDK, this API can be used to initialize the NOR flash to work in 1-1-1 mode
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_norFlashReadId
        <td>In MCU+SDK, this API can be used to read the JEDEC ID from the NOR flash connected to the OSPI peripheral
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_norFlashWrite
        <td>In MCU+SDK, this API can be used to write data to the flash at a specified offset
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_norFlashRead
        <td>In MCU+SDK, this API can be used to read data from the flash from a specified offset
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_norFlashReadSfdp
        <td>In MCU+SDK, this API can be used to read SFDP table from the flash from a specified offset
    </tr>
    <tr>
        <td>None
        <td>\ref OSPI_norFlashErase
        <td>In MCU+SDK, this API can be used to erase 1 block of data starting from a provided address
    </tr>
</table>

## Important Notes

- In MCU+ SDK, Users are recommended to use SysConfig as this will greatly simplify the task of driver configuration.

## See Also

- \ref DRIVERS_OSPI_PAGE
- \ref EXAMPLES_DRIVERS_OSPI_FLASH_DIAG
- \ref EXAMPLES_DRIVERS_OSPI_FLASH_DMA
- \ref EXAMPLES_DRIVERS_OSPI_FLASH_IO
