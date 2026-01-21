# Flash {#BOARD_FLASH_PAGE}

[TOC]

\cond SOC_AM243X || SOC_AM273X || SOC_AM64X || SOC_AWR294X || SOC_AM263PX || SOC_AM261X
The Flash driver provides API to read and write to xSPI based flash devices present in the board.
\endcond
\cond SOC_AM263X
The Flash driver provides API to read and write to QSPI based flash devices present in the board.
\endcond
\cond SOC_AM65X
The Flash driver provides API to read and write to OSPI based flash devices present in the board.
\endcond
The driver takes care of all sequencing necessary to perform writes across pages and
the application need not take care of the programming intricacies.

## Features Supported

- APIs to read and write to a flash offset
- Provides API to return flash attributes like block size, page size etc
- API for block erases
\cond SOC_AM263PX || SOC_AM261X
- Supports Nand Flash
\endcond
\cond SOC_AM64X || SOC_AM243X
- Fallback mechanism in case of PHY tuning failure is currently only supported for the 8d_8d_8d protocol configuration
\endcond
## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Option to select flash type based on board
\cond SOC_AM64X || SOC_AM243X
- Supported flash devices
    - S28HS512T
    - S25HL512T
    - MX25LM25645G
\endcond
\cond SOC_AM263X
- Supported flash devices
    - S25FL128SA
\endcond
\cond SOC_AM263PX || SOC_AM261X
- Supported flash devices
    - IS25LX256
    - W25N01GVZEJ
\endcond
\cond SOC_AM273X || SOC_AWR294X
- Supported flash devices
    - GD25B64C
\endcond
\cond SOC_AM65X
- Supported flash devices
    - MT35XU512A
\endcond

## Features NOT Supported

\cond !(SOC_AM263PX || SOC_AM261X)
NA
\endcond

\cond SOC_AM263PX || SOC_AM261X
- DMA for NAND FLASH
\endcond

\cond SOC_AM64X || SOC_AM243X
## Flash Quirks

The flash driver provides quirk functions to handle flash-specific configuration requirements and recovery mechanisms. These quirks ensure proper flash initialization and reliable operation across different flash devices and protocols.

These quirk APIs can be passed to the flash driver via SysConfig in the Flash section as board configurations. This allows board-specific implementations to be plugged into the flash initialization sequence.

### Boot Quirks

Boot quirks are invoked at the start of flash initialization. The primary function of boot quirks is to:
- Set the flash device into 1s-1s-1s (Single SDR) mode for reliable initial communication
- If the mode switch fails, attempt to recover the flash based on the recovery mechanism specified in the flash device datasheet
- Ensure the flash is in a known good state before proceeding with further configuration

This mechanism is critical during boot to recover from potential flash configuration issues that may have occurred during previous operations or power cycles.

For S28HS512T and S25HL512T flash devices, the SDK provides the following boot quirk APIs:
- `Flash_quirkOSPIEarlyFixup` - Boot quirk for OSPI interface
- `Flash_quirkQSPIEarlyFixup` - Boot quirk for QSPI interface

The implementation of these APIs is available in `source/board/flash/ospi/flash_nor_ospi.c`. These APIs can be configured in SysConfig as the boot quirk function for the respective flash devices.

#### Example Implementation

\code
int32_t Flash_myBootQuirk(Flash_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    Flash_Attrs *attrs = config->attrs;

    /* Your code for handling quirks goes here */
    /* Set the flash in basic configuration */
    /* Recovery mechanism as per flash datasheet */

    return status;
}
\endcode

### Configuration Quirks

Configuration quirks are invoked at the end of flash initialization. These quirks handle:
- Flash device-specific configuration requirements
- Protocol-specific settings and optimizations

Configuration quirks ensure that the flash is properly configured for the intended protocol and operational mode after the initial boot sequence completes.

#### Example Implementation

\code
int32_t Flash_myConfigQuirk(Flash_Config *config)
{
    int32_t status = SystemP_SUCCESS;
    Flash_Attrs *attrs = config->attrs;

    /* Your code for handling quirks goes here */
    /* For example: modify flash timing parameters, handle special initialization, etc. */

    return status;
}
\endcode

### Configuring Quirks in SysConfig

The boot quirk and configuration quirk function pointers can be configured in the SysConfig Flash module under board configurations. This allows vendor-specific quirk implementations to be registered during flash initialization without modifying the core flash driver code.

\imageStyle{flash_quirks.png, width:60%}
\image html drivers/flash_quirks.png "Flash Quirk Functions"

\endcond

## Important Usage Guidelines

- Typically before writing to an offset, erase the block which corresponds to the offset
- Flashes support multiple erase sizes, conditionally. We provide 2 APIs for this cause in case the flash supports 2
  different erase sizes:
   - Flash_eraseBlk()
   - Flash_eraseSector()
  The larger size can be thought of as a 'block' and the smaller size as a 'sector'. In a flash
  where both are supported, one can make use of both. In some flashes, the sector configuration has to be fixed before-hand.
  In such cases the way erase sizes supported will be subject to this configuration. Take care in the flash configuration
  and the application to make sure the erase APIs are called only with the right configuration.
- Flash writes can only be done to a page size aligned offset, otherwise the write API returns an error
\cond SOC_AM64X || SOC_AM243X || SOC_AM263PX || SOC_AM261X
- The `sbl_ospi` will be typically flashed at location 0x00000000 in the flash memory. So applications using flash should refrain from using this offset. Similarly, the starting of the last block of the flash is used for storing attackVector patterns later used for PHY tuning. So avoid using this offset as well.
\endcond
\cond SOC_AM263X
- The `sbl_qspi` will be typically flashed at location 0x00000000 in the flash memory. So applications using flash should refrain from using this offset.
\endcond

## Example Usage

Include the below file to access the APIs
\snippet Flash_sample.c include

Flash Read API
\snippet Flash_sample.c read

Flash Write API
\snippet Flash_sample.c write

Flash Erase API
\snippet Flash_sample.c erase


## API

\ref BOARD_FLASH_MODULE
\ref BOARD_FLASH_MODULE