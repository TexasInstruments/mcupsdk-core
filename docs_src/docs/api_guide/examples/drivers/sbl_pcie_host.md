# SBL PCIE HOST {#EXAMPLES_DRIVERS_SBL_PCIE_HOST}

[TOC]

# Introduction

\if SOC_AM65X
\note This is **NOT** a bootloader but a helper application running on a host board,
that sends the bootloader, system firmware and application image to the target board.

This application can be run as we run any other applcation using any boot modes
preferred. This application works in conjunction with \ref EXAMPLES_DRIVERS_SBL_PCIE.
The application sends the images required for the target board to boot from PCIe boot
mode. The application first finds the SBL image by name `sbl_pcie.release.tiimage`
from the SD card and sends the image via PCIe for ROM to boot.

Then the appimage is send via PCIe for the SBL in target board to pick up and boot.
The system firmware and appimage must be present in the SD card by the name `sysfw.bin`
and `app` respectively. On succesfully sending the image, application sends a magic word
to mark the completion of transfer.
\else
\note This is **NOT** a bootloader but a helper application running on a host board,
that sends the bootloader and application image to the target board.

This application can be run as we run any other applcation using any boot modes
preferred. This application works in conjunction with \ref EXAMPLES_DRIVERS_SBL_PCIE.
The application sends the images required for the target board to boot from PCIe boot
mode. The application first finds the SBL image by name `sbl_pcie.release.hs_fs.tiimage`
from the SD card and sends the image via PCIe for ROM to boot.

Then the appimage is send via PCIe for the SBL in target board to pick up and boot.
The appimage must be present in the SD card by the name `app`. On succesfully
sending the image, application sends a magic word to mark the completion of transfer
and marks the relative offset  to which the image is sent to.
\endif

# Supported Combinations {#EXAMPLES_DRIVERS_SBL_PCIE_HOST_COMBOS}

\cond SOC_AM64X || SOC_AM243X || SOC_AM65X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_pcie_host

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref EXAMPLES_DRIVERS_SBL_PCIE

# Sample Output

\code

Starting Host EVM application for PCIe Boot
Sending SBL image - "/sd0/sbl_pcie.release.hs_fs.tiimage" of size 288829
Sending appimage - "/sd0/app" of size 232249
Images transferred successfully

\endcode
