# FileX {#FS_FILEX}

[TOC]

## Introduction

FileX is a complete FAT format media and file management system for deeply embedded applications.

FileX supports an unlimited number of media devices at the same time, including RAM disks, FLASH managers, and actual physical devices. It supports 12-, 16-, and 32-bit File Allocation Table (FAT) formats, and also supports contiguous file allocation, and is highly optimized for both size and performance. FileX also includes fault tolerant support, media open/ close, and file write callback functions.

Designed to meet the growing need for FLASH devices, FileX uses the same design and coding methods as ThreadX. Like all Eclipse Foundation products, FileX is distributed with full ANSI C source code, and it has no run-time royalties.

In this SDK, FileX is integrated to work with RAM disk driver and MMCSD driver. In addition, FileX can be combined with the LevelX Flash Translation Layer (FTL) and integrated to work with the SDK NOR and NAND flash drivers. There is SysConfig support for FileX. To use FileX in your application, just add the FileX module under `FILE SYSTEM` in the SysConfig panel. The media (RAM disk, MMCSD or LevelX) can be selected and a corresponding media instance will automatically added.

- \ref EXAMPLES_FS_FILEX_HELLO_WORLD

For the full FileX documentation, please refer to the below table.

## Additional References {#FS_FILEX_ADDITIONAL_REFERENCES}

<table>
<tr>
    <th>Website Link
    <th>Description
</tr>
<tr>
    <td>[FileX File System](https://github.com/eclipse-threadx/rtos-docs/blob/main/rtos-docs/filex/about-this-guide.md)
    <td>Complete documentation of the FileX file system and API references
</tr>
</table>
