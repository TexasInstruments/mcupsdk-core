# LittleFS File System {#FS_LITTLEFS}

[TOC]

## Introduction

A little fail-safe filesystem designed for microcontrollers.

**Power-loss resilience**: Designed to handle random power failures. All file operations have strong copy-on-write guarantees and if power is lost the filesystem will fall back to the last known good state.

**Dynamic wear leveling**: Designed with flash in mind, and provides wear leveling over dynamic blocks. Additionally, littlefs can detect bad blocks and work around them.

**Bounded RAM/ROM**: Designed to work with a small amount of memory. RAM usage is strictly bounded, which means RAM consumption does not change as the filesystem grows. The filesystem contains no unbounded recursion and dynamic memory is limited to configurable buffers that can be provided statically.

In this SDK, LittleFS is integrated to work with the Flash driver. The porting layer supports device such as OSPI NOR flash. There is SysConfig support for the LittleFS integrated module. So, to use the LittleFS in your application, just add the LittleFS module under `FILE SYSTEMS` in the SysConfig. It will automatically add a Flash instance. Sysconfig also exposes configuration inputs such as Flash Offset, Block Count, Block Cycles, Read Size, Program/Write Size, Cache Buffer Size, Lookahead Buffer Size, Maximum File Name Size, Maximum File Size to the user to customise LittleFS as per the requirements. Therefore user need not configure LittleFS from the application.

You can refer to these examples here for API usage. Most of the initialization is done by the Sysconfig itself.

- \ref EXAMPLES_DRIVERS_OSPI_FLASH_FILE_IO

For the full FreeRTOS+FAT API reference, please refer to the below table.

## Additional References {#FS_LITTLEFS_ADDITIONAL_REFERENCES}

<table>
<tr>
    <th>Website Link
    <th>Description
</tr>
<tr>
    <td>[LittleFS File System](https://github.com/littlefs-project/littlefs/blob/master/README.md)
    <td>Complete documentation of the LittleFS file system
</tr>
</table>
