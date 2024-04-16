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
[Cortex_R5_0] [OSPI Flash Diagnostic Test] Starting ...
[OSPI Flash Diagnostic Test] Flash Manufacturer ID : 0x9D
[OSPI Flash Diagnostic Test] Flash Device ID       : 0x5A19
[OSPI Flash Diagnostic Test] Executing Flash Erase on first block...
[OSPI Flash Diagnostic Test] Done !!!
[OSPI Flash Diagnostic Test] Performing Write-Read Test...
[OSPI Flash Diagnostic Test] Write-Read Test Passed!
[QSPI Flash Diagnostic Test] SFDP Information :
================================================
                      SFDP
================================================
SFDP Major Revision                       : 0x1
SFDP Minor Revision                       : 0x9
Number of Parameter Headers in this Table : 4

Types of Additional Parameter Tables in this flash
---------------------------------------------------
4 BYTE ADDRESSING MODE INSTRUCTIONS TABLE
NOR SPI PROFILE TABLE
OCTAL DDR MODE COMMAND SEQUENCE TABLE

Parsing of OCTAL DDR MODE COMMAND SEQUENCE TABLE table not yet supported.
JSON Data for the flash :

{

    "flashSize": 33554432,
    "flashPageSize": 256,
    "flashManfId": "0x9D",
    "flashDeviceId": "0x5A19",
    "flashBlockSize": 131072,
    "flashSectorSize": 4096,
    "cmdBlockErase3B": "0xD8",
    "cmdBlockErase4B": "0xDC",
    "cmdSectorErase3B": "0x20",
    "cmdSectorErase4B": "0x21",
    "protos": {
        "p111": {
            "isDtr": false,
            "cmdRd": "0x03",
            "cmdWr": "0x02",
            "modeClksCmd": 0,
            "modeClksRd": 0,
            "dummyClksCmd": 0,
            "dummyClksRd": 0,
            "enableType": "0",
            "enableSeq": "0x00",
            "dummyCfg": null,
            "protoCfg": null,
            "strDtrCfg": null
        },
        "p112": null,
        "p114": null,
        "p118": {
            "isDtr": false,
            "cmdRd": "0x7C",
            "cmdWr": "0x84",
            "modeClksCmd": 0,
            "modeClksRd": 1,
            "dummyClksCmd": 0,
            "dummyClksRd": 7,
            "enableType": "0",
            "enableSeq": "0x00",
            "dummyCfg": null,
            "protoCfg": null,
            "strDtrCfg": null
        },
        "p444s": null,
        "p444d": null,
        "p888s": null,
        "p888d": {
            "isDtr": false,
            "cmdRd": "0x0B",
            "cmdWr": "0x12",
            "modeClksCmd": 0,
            "modeClksRd": 0,
            "dummyClksCmd": 8,
            "dummyClksRd": 14,
            "enableType": "0",
            "enableSeq": "0x00",
            "dummyCfg": {
                "isAddrReg": false,
                "cmdRegRd":"0x00",
                "cmdRegWr":"0x00",
                "cfgReg":"0x00000000",
                "shift":0,
                "mask":"0x00",
                "bitP":14
            },
            "protoCfg": {
                "isAddrReg": false,
                "cmdRegRd": "0x00",
                "cmdRegWr": "0x00",
                "cfgReg": "0x00000000",
                "shift": 0,
                "mask": "0x00",
                "bitP": 0
            },
            "strDtrCfg": {
                "isAddrReg": false,
                "cmdRegRd": "0x00",
                "cmdRegWr": "0x00",
                "cfgReg": "0x00000000",
                "shift": 0,
                "mask": "0x00",
                "bitP": 0
            }
        },
        "pCustom": {
            "fxn": null
        }
    },
    "addrByteSupport": "1",
    "fourByteAddrEnSeq": "0xA1",
    "cmdExtType": "REPEAT",
    "resetType": "0x30",
    "deviceBusyType": "0",
    "cmdWren": "0x06",
    "cmdRdsr": "0x05",
    "srWip":  0,
    "srWel":  1,
    "cmdChipErase": "0xC7",
    "rdIdSettings": {
        "cmd": "0x9F",
        "numBytes": 5,
        "dummy4": 0,
        "dummy8": 0
    },
    "xspiWipRdCmd": "0x00",
    "xspiWipReg": "0x00000000",
    "xspiWipBit": 0,
    "flashDeviceBusyTimeout": 72000000,
    "flashPageProgTimeout": 120
}

All tests have passed!!
\endcode