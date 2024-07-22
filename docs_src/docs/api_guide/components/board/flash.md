# Flash {#BOARD_FLASH_PAGE}

[TOC]

\cond SOC_AM243X || SOC_AM273X || SOC_AM64X || SOC_AWR294X || SOC_AM263PX || SOC_AM261X
The Flash driver provides API to read and write to xSPI based flash devices present in the board.
\endcond
\cond SOC_AM263X
The Flash driver provides API to read and write to QSPI based flash devices present in the board.
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

## Features NOT Supported

\cond !(SOC_AM263PX || SOC_AM261X)
NA
\endcond

\cond SOC_AM263PX || SOC_AM261X
- DMA for NAND FLASH
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