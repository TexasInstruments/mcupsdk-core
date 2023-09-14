# OSPI Flash Diagnostic {#EXAMPLES_DRIVERS_OSPI_FLASH_DIAG}

[TOC]

# Introduction

This is a diagnostic example for NOR-OSPI Flashes. This example doesn't use the flash driver and should ideally pass for any NOR-OSPI flash. For this particular example, the options in SysCfg
like the number of transfer lines used, or the clock divider value are ignored. This example will always talk to the flash in the lowest settings possible. The flash device is reset, and is
expected to support 1s1s1s mode after reset. Then the OSPI controller is programmed to work in 1s1s1s mode with 3 byte addressing mode.

The test itself is simple, first it tries to read the JEDEC ID of the flash which consists of the flash manufacturer ID and the flash device ID. These are then printed onto the logging console. When
doing flash bring-ups in new boards, this example can be run first for sanity. Users can cross check the printed ID with the one in flash datasheet to confirm that basic communication with flash is working.

The test then tries to erase a flash memory block at an offset of 512 KB and then write some known data to that memory. This data is then read back and verified to confirm that reads and writes are working
in 1s1s1s mode.

# Supported Combinations {#EXAMPLES_DRIVERS_OSPI_FLASH_DIAG_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/ospi/ospi_flash_diag

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/ospi/ospi_flash_diag

\endcond

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/ospi/ospi_flash_diag

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_OSPI_PAGE

# Sample Output

\code
[OSPI Flash Diagnostic Test] Starting ...
[OSPI Flash Diagnostic Test] Flash Manufacturer ID : 0x34
[OSPI Flash Diagnostic Test] Flash Device ID       : 0x5B1A
[OSPI Flash Diagnostic Test] Executing Flash Erase on first block...
[OSPI Flash Diagnostic Test] Done !!!
[OSPI Flash Diagnostic Test] Performing Write-Read Test...
[OSPI Flash Diagnostic Test] Write-Read Test Passed!
[OSPI Flash Diagnostic Test] SFDP Information :
================================================
                      SFDP
================================================
SFDP Major Revision                       : 0x1
SFDP Minor Revision                       : 0x8
Number of Parameter Headers in this Table : 6

Types of Additional Parameter Tables in this flash
---------------------------------------------------
4 BYTE ADDRESSING MODE INSTRUCTIONS TABLE
XSPI PROFILE TABLE
STATUS CONTROL AND CONFIGURATION REGISTER MAP TABLE
OCTAL DDR MODE COMMAND SEQUENCE TABLE
SECTOR MAP TABLE

Parsing of OCTAL DDR MODE COMMAND SEQUENCE TABLE table not yet supported.

Flash_NorXspiDevDefines gFlashNorXspiDeviceDefines_<part-number> = {

    .XSPI_NOR_CMD_RSTEN = 0x66,
    .XSPI_NOR_CMD_RSTMEM = 0x99,
    .XSPI_NOR_CMD_WREN = 0x06,
    .XSPI_NOR_CMD_WRREG = 0x71,
    .XSPI_NOR_CMD_BULK_ERASE = 0xC7,
    .XSPI_NOR_CMD_SECTOR_ERASE_3B = 0x21,
    .XSPI_NOR_CMD_SECTOR_ERASE_4B = 0x21,
    .XSPI_NOR_CMD_BLOCK_ERASE_3B = 0xDC,
    .XSPI_NOR_CMD_BLOCK_ERASE_4B = 0xDC,
    .XSPI_NOR_CMD_PAGE_PROG_3B = 0x02,
    .XSPI_NOR_CMD_PAGE_PROG_4B = 0x12,
    .XSPI_NOR_CMD_RDSR = 0x05,
    .XSPI_NOR_CMD_RDREG = 0x65,
    .XSPI_NOR_CMD_RDID = 0x9F,
    .XSPI_NOR_CMD_READ = 0x03,
    .XSPI_NOR_CMD_888_SDR_READ = 0x00,
    .XSPI_NOR_CMD_888_DDR_READ = 0xEE,
    .XSPI_NOR_CMD_444_SDR_READ = 0x00,
    .XSPI_NOR_CMD_444_DDR_READ = 0x00,
    .XSPI_NOR_CMD_114_READ = 0x00,
    .XSPI_NOR_SR_WIP = 1,
    .XSPI_NOR_SR_WEL = 2,
    .XSPI_NOR_RDID_NUM_BYTES = 5,
    .XSPI_NOR_MANF_ID = 0x34,
    .XSPI_NOR_DEVICE_ID = 0x5B1A,
    .XSPI_NOR_114_READ_MODE_CLKS = 0,
    .XSPI_NOR_114_READ_DUMMY_CYCLES = 0,
    .XSPI_NOR_444_READ_MODE_CLKS = 0,
    .XSPI_NOR_444_READ_DUMMY_CYCLES = 0,
    .XSPI_NOR_444_READ_DUMMY_CYCLES_LC = 0xFF,
    .XSPI_NOR_QUAD_CMD_READ_DUMMY_CYCLES = 0x00,
    .XSPI_NOR_OCTAL_READ_DUMMY_CYCLE = 24,
    .XSPI_NOR_OCTAL_READ_DUMMY_CYCLE_LC = 0x0B,
    .XSPI_NOR_OCTAL_DDR_RDSR_DUMMY_CYCLE = 4,
    .XSPI_NOR_OCTAL_DDR_RDREG_ADDR_BYTES = 4,
    .XSPI_NOR_OCTAL_DDR_WRREG_ADDR_BYTES = 4,
    .XSPI_NOR_OCTAL_DDR_RDVREG_DUMMY_CYCLE = 4,
    .XSPI_NOR_OCTAL_DDR_RDNVREG_DUMMY_CYCLE = 8,
    .XSPI_NOR_OCTAL_RDSFDP_DUMMY_CYCLE = 8,
    .XSPI_NOR_OCTAL_RDSFDP_ADDR_TYPE = 0,
    .XSPI_NOR_WRR_WRITE_TIMEOUT = 5120,
    .XSPI_NOR_BULK_ERASE_TIMEOUT = 256000000,
    .XSPI_NOR_PAGE_PROG_TIMEOUT = 512,
    .XSPI_NOR_VREG_OFFSET = 0x800000,
    .XSPI_NOR_NVREG_OFFSET = 0x0,
    .XSPI_NOR_QUAD_MODE_CFG_ADDR = 0x0,
    .XSPI_NOR_QUAD_MODE_CFG_BIT_LOCATION = 0x0,
    .XSPI_NOR_DDR_OCTAL_MODE_CFG_ADDR = 0x6,
    .XSPI_NOR_DDR_OCTAL_MODE_CFG_BIT_LOCATION = 0x1,
    .XSPI_NOR_DUMMY_CYCLE_CFG_ADDR = 0x3,
    .XSPI_NOR_FLASH_SIZE = 67108864,
    .XSPI_NOR_PAGE_SIZE = 256,
    .XSPI_NOR_BLOCK_SIZE = 262144,
    .XSPI_NOR_SECTOR_SIZE = 4096,
    .addrByteSupport = 1,
    .dtrSupport = 1,
    .qeType = 0,
    .seq444Enable = { 0, 0, 0, 0, 0 },
    .seq444Disable = { 0, 0, 0, 0 },
    .oeType = 0,
    .cmdExtType = 0,
    .byteOrder = 0,
};

All tests have passed!!
\endcode