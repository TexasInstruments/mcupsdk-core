# Running an Application {#RUNNING_APPLICATION}

Once application image is loaded into memory, it needs to be run on the respective cores.

Bootloaders are responsible for this. It is a software that runs as soon as an SoC is powered on. It performs SOC initializations, configures clocks, registers. It then communicates with an external media to receive the application.

In MCU+SDK, multistage bootloading is done where first the **ROM bootloader** boots a secondary bootloader, and the secondary bootloader, or **SBL** boots up the application.

The ROM bootloader is stored in RAM. It expects a x509 signed image of the SBL.

Then the SBL needs to be placed at 0x70020000 in RAM, and the application offset needs to be specified to the SBL.

The SBL then reads this application, parses it, loads to respective CPUs, releases core from reset. The application starts running now.

For detailed information on bootloaders and bootflow, refer \ref BOOTFLOW_GUIDE

## SBLs in SDK

<table>
<tr>
	<th>SBL
    <th>Reference
</tr>
<tr>
    <td>SBL NULL
    <td>\ref EXAMPLES_DRIVERS_SBL_NULL
</tr>
\cond !SOC_AM263X
<tr>
    <td>SBL OSPI
    <td>\ref EXAMPLES_DRIVERS_SBL_OSPI
</tr>
\endcond
\cond SOC_AM263X
<tr>
    <td>SBL QSPI
    <td>\ref EXAMPLES_DRIVERS_SBL_QSPI
</tr>
\endcond
<tr>
    <td>SBL UART
    <td>\ref EXAMPLES_DRIVERS_SBL_UART
</tr>
<tr>
    <td>SBL CAN
    <td>\ref EXAMPLES_DRIVERS_SBL_CAN
</tr>
<tr>
    <td>SBL SD
    <td>\ref EXAMPLES_DRIVERS_SBL_SD
</tr>
</table>

The SBL is like any other example of the SDK. They use the bootloader library APIs to carry out the bootloading process.

Depending on the boot media from which we load the application binary, we have multiple SBLs like `sbl_ospi`,`sbl_uart` etc.

A bare minimum SBL called the `sbl_null` is also included which aids the users to load their applications via CCS. Here are some details regarding those.

### SBL NULL

- The `sbl_null` is a secondary bootloader which doesn't load any application binary, but just does the SOC initialization and puts all the cores in WFI (Wait For Interrupt) mode.
- This is supposed to be a "development form" bootloader which should be used only during initial development.
- The other method is using NO-BOOT/DEV-BOOT boot modes of the devices and using GEL scripts to initialize the SoC via debugger. The application
  binaries can then be side-loaded. ROM is not involved in this case. The `sbl_null` is an alternative to this process.
- This is referred to as the SOC initialization binary, refer \ref EVM_FLASH_SOC_INIT for more on this.

\cond SOC_AM263X
### SBL QSPI

- The `sbl_qspi` is a secondary bootloader which reads and parses the application image from a location in the QSPI flash and then moves on to core initialization and other steps

- To boot an application using the `sbl_qspi`, the application image needs to be flashed at a particular location in the QSPI flash memory.
- To flash an application (or any file in fact) to a location in the QSPI flash memory, follow the steps mentioned in \ref BASIC_STEPS_TO_FLASH_FILES

\endcond

\cond SOC_AM263PX || SOC_AM261X
### SBL OSPI

- The `sbl_ospi` is a secondary bootloader which reads and parses the application image from a location in the OSPI flash and then moves on to core initialization and other steps

- To boot an application using the `sbl_ospi`, the application image needs to be flashed at a particular location in the OSPI flash memory.

- This location or offset is specified in the SysConfig of the `sbl_ospi` application. Currently this is 0x81000. This offset is chosen under the assumption that the `sbl_ospi`
  application takes at max 512 KB from the start of the flash. If a custom bootloader is used, make sure that this offset is chosen in such a way that it is greater than the
  size of the bootloader which is being flashed and also aligns with the block size of the flash.

- To flash an application (or any file in fact) to a location in the OSPI flash memory, follow the steps mentioned in \ref BASIC_STEPS_TO_FLASH_FILES

\endcond

### SBL UART

- The `sbl_uart` is a secondary bootloader which receives the multicore application via UART, stores it in memory and then does the parsing, core initialization etc.

- To boot an application using the `sbl_uart`, you can refer to \ref UART_BOOTLOADER_PYTHON_SCRIPT subsection. Detailed steps on the usage is mentioned in the same subsection.

### SBL CAN

- The `sbl_can` is a secondary bootloader which needs to be flashed in QSPI Flash.

- The `sbl_can` receives the multicore application via CAN, stores it in memory and then does the parsing, core initialization etc.

- To boot an application using the `sbl_can`, you can refer to \ref CAN_BOOTLOADER_PYTHON_SCRIPT subsection.

### SBL SD

- The `sbl_sd` is a secondary bootloader which reads the application image file from the SD card and then moves on to core initialization and other steps

- To boot an application using the `sbl_sd`, the application image needs to be copied to the SD card as a file named "app"

- Similarly you can copy any application file to the SD card and rename in the SD card as "app" so that the SBL can pick it up.

- Currently the `sbl_sd` reads the full application file into an MSRAM buffer and then parses the multicore application. Because of this reason **application images higher than ~512 KB in size can't be booted by `sbl_sd` as of now**.

- To boot an application using `sbl_sd`, you can refer to \ref EXAMPLES_DRIVERS_SBL_SD subsection.

## SBL Based bootflow on HSFS device

For non secure boot, the SBL just reads the application image from a boot media, parses it, and boots it.

  \imageStyle{bootflow_am263x_hsfs.png,width:70%}
  \image html bootflow_am263x_hsfs.png "HSFS Application Run"

## SBL Based bootflow on HSSE device

In secure boot, the SBL reads the **signed application image** from a boot media, authenticates it, decrypts it, and boots it.

For detailed information refer \ref SECURE_BOOT

## Debugging support

For information on various debugging tools and techniques, refer [Debugging Sitara AM2x Microcontrollers](https://www.ti.com/document-viewer/lit/html/sprad28)