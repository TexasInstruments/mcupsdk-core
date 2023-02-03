# DTHE RNG {#EXAMPLES_DRIVERS_DTHE_RNG}
[TOC]

# Introduction
This example demonstrates the usage of Random Number Generator (RNG) from DTHE module. In this example RNG generates a 128 bit random number.

\cond SOC_AM263X

 Parameter             | Value
 ----------------------|-----------
 CPU + OS              | r5fss0-0 nortos
 Toolchain             | ti-arm-clang
 Boards                | @VAR_BOARD_NAME_LOWER
 Example folder        | examples/security/crypto/dthe_rng/
 Supported Device Type | HS-FS

\endcond

## Build the DTHE RNG example
\code
$make -s -C examples/security/crypto/dthe_rng/am263x-cc/r5fss0-0_nortos/ti-arm-clang all DEVICE=am263x
\endcode


# Steps to Run through ROM Boot flow

## Via SBL_uart bootloader

### Build sbl_uart
\code
make -s -C examples/drivers/boot/sbl_uart/am263x-cc/r5fss0-0_nortos/ti-arm-clang/ all DEVICE=am263x
\endcode

### Set board for UART boot

See [EVM setup](https://software-dl.ti.com/mcu-plus-sdk/esd/AM263X/latest/exports/docs/api_guide_am263x/EVM_SETUP_PAGE.html#autotoc_md29)

### Run UART_bootloader
\code
python uart_bootloader.py -p <COMxx> --bootloader=sbl_prebuilt/am263x-cc/sbl_uart.release.hs.tiimage --file=../../examples/security/crypto/dthe_rng/am263x-cc/r5fss0-0_nortos/ti-arm-clang/dthe_rng.release.appimage
\endcode

### Sample output
On successful boot, R5 log at uart terminal, will have the following output.
UART Console:
\code

THE Rng example started ...
[DTHE] DTHE Rng output word 1 -- 0xC9F8865C
[DTHE] DTHE Rng output word 2 -- 0x8855AAD8
[DTHE] DTHE Rng output word 3 -- 0x1B0234F8
[DTHE] DTHE Rng output word 4 -- 0xC3F2CA6E
[DTHE] DTHE Rng output word 1 -- 0x186D31B3
[DTHE] DTHE Rng output word 2 -- 0xDAE15AEE
[DTHE] DTHE Rng output word 3 -- 0x1095366B
[DTHE] DTHE Rng output word 4 -- 0x9B28C823
[DTHE] DTHE Rng output word 1 -- 0xE3B00722
[DTHE] DTHE Rng output word 2 -- 0x5B09F37A
[DTHE] DTHE Rng output word 3 -- 0x7013565C
[DTHE] DTHE Rng output word 4 -- 0xFA7354A1
[DTHE] DTHE Rng output word 1 -- 0x174A9A62
[DTHE] DTHE Rng output word 2 -- 0xA161BFF5
[DTHE] DTHE Rng output word 3 -- 0x12CBD44E
[DTHE] DTHE Rng output word 4 -- 0xB3FB05DF
[DTHE] DTHE Rng output word 1 -- 0xB188A57C
[DTHE] DTHE Rng output word 2 -- 0x169F4621
[DTHE] DTHE Rng output word 3 -- 0x337A657D
[DTHE] DTHE Rng output word 4 -- 0xC6804EB6
[DTHE] DTHE Rng example completed!!
All tests have passed!!


\endcode

## Via SBL_qspi bootloader

### Build sbl_qspi# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

\code
make -C examples/drivers/boot/sbl_qspi/am263x-cc/r5fss0-0_nortos/ti-arm-clang/ all DEVICE=am263x
\endcode

### Build sbl_uart_uniflash
\code
make -s -C examples/drivers/boot/sbl_uart_uniflash/am263x-cc/r5fss0-0_nortos/ti-arm-clang/ all DEVICE=am263x
\endcode


### Set UART boot mode
See [EVM setup](https://software-dl.ti.com/mcu-plus-sdk/esd/AM263X/latest/exports/docs/api_guide_am263x/EVM_SETUP_PAGE.html#autotoc_md29)

### Edit default_sbl_qspi.cfg to include the correct images

### Run UART_uniflash
\code
python uart_uniflash.py -p <COMxx> --cfg=sbl_prebuilt/am263x-cc/default_sbl_qspi.cfg
\endcode

### Set QSPI boot mode
See [EVM setup](https://software-dl.ti.com/mcu-plus-sdk/esd/AM263X/latest/exports/docs/api_guide_am263x/EVM_SETUP_PAGE.html#autotoc_md29)

### Sample output
On successful boot, R5 log at uart terminal, will have the following output.

# Sample Output

Shown below is a sample output when the application is run.

UART Console:
\code

THE Rng example started ...
[DTHE] DTHE Rng output word 1 -- 0xC9F8865C
[DTHE] DTHE Rng output word 2 -- 0x8855AAD8
[DTHE] DTHE Rng output word 3 -- 0x1B0234F8
[DTHE] DTHE Rng output word 4 -- 0xC3F2CA6E
[DTHE] DTHE Rng output word 1 -- 0x186D31B3
[DTHE] DTHE Rng output word 2 -- 0xDAE15AEE
[DTHE] DTHE Rng output word 3 -- 0x1095366B
[DTHE] DTHE Rng output word 4 -- 0x9B28C823
[DTHE] DTHE Rng output word 1 -- 0xE3B00722
[DTHE] DTHE Rng output word 2 -- 0x5B09F37A
[DTHE] DTHE Rng output word 3 -- 0x7013565C
[DTHE] DTHE Rng output word 4 -- 0xFA7354A1
[DTHE] DTHE Rng output word 1 -- 0x174A9A62
[DTHE] DTHE Rng output word 2 -- 0xA161BFF5
[DTHE] DTHE Rng output word 3 -- 0x12CBD44E
[DTHE] DTHE Rng output word 4 -- 0xB3FB05DF
[DTHE] DTHE Rng output word 1 -- 0xB188A57C
[DTHE] DTHE Rng output word 2 -- 0x169F4621
[DTHE] DTHE Rng output word 3 -- 0x337A657D
[DTHE] DTHE Rng output word 4 -- 0xC6804EB6
[DTHE] DTHE Rng example com

\endcode

# See Also

\ref DRIVERS_DTHE_PAGE