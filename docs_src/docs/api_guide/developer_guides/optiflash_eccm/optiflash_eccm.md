# Enabling safety on external flash {#OPTIFLASH_ECCM}

[TOC]

# Introduction 

Unlike NAND flash, which has `spare area` for storing ECC of `main area`, NOR flash do not have such provision. NOR flash in AM26x devices are also being used to perform XIP as well. 

This Page goes over how safety can be enabled on external NOR flash in AM26x devices which also use NOR flash for XIP.

# ECCM Module 

The following diagram shows 

\imageStyle{am263px_safety_io_highlight.png,width:40%}
\image html am263px_safety_io_highlight.png "ECC Module"

As can be seen, it sits just before the `Flash controller` and operates between the data that is coming from the flash and then process it on the fly. 

ECCM module works on the bases on inlining of ECC Bytes of data. As per the datasheet of ECCM, it expects 4B of ECC data after every 32B of actual data (As per the below iamge)

\imageStyle{eccm_padding.png,width:40%}
\image html eccm_padding.png "ECC Padding To Data Byte"

Here the requestor, R5F with cache enabled for the external flash, sends request for the data from the flash. R5F sends a request to get 32B of data from the external flash and this request goes via the ECCM, which then ask `flash controller` to get 36B of data. When the `flash controller` sends back the request 36B of data, ECCM module, 
1. strips off the 4B of ECC data 
2. calculates the ECC of 1st 32B of data
3. compares the computed ECC with extracted ECC bytes   

If computed ECC and saved ECC matches, ECCM forwards the data to the requestor, otherwise, it sends 0 and puts up an interrupt to the requestor, indicating that data fault has been detected.

## Important considerations 

1. Flash region should be defined as `Normal & Cached` in the MPU.
    
2. As can be seen, for every 32B, there is 4B of ECC. Because of this, the available size of flash is decreased and as per datasheet, size of flash is decreaed by ~11.11%. During the architecture phase of the software, one has to consider this fact. If the software requirement is to enable ECC on external flash, then, the size of the external flash should be reduced in linker file, etc. For example if 8mb of external flash is being used, then sofware/user applicaiton should only use 7mb of flash. 

3. Writing to flash with ECCM enabled should be avoided at all cost. Harware does not supports it. The right way to do this is to pre-compute ECC for the data that is to be written to flash and write the computed data via the flsopskd controller (\ref FLSOPSKD_IP). (or write to external flash via `bypass region`). 

# How to enable ECCM module 

## Write a config file
The following is an example of a config file (conf.json)

\code 

{
    "regions": [
        {
            "start": 1611661312,
            "size": 4194304,
            "eccEnable": true
        }
    ]
}

\endcode 

There can be at most 4 different ECC regions that can be configured. The above shows 1 region. For example, in am263px, there are 4 different core and each core can have its own region configured. Make sure that each region is not overlaping each other.

In the above, region start address is 0x60100000 and size 4MB.

## Using the config file 

While compiling the application, pass in the argument `make oeconfig=conf.json`. 

## Seeing the change

To see if the processing has been done correctly, use the command 

`tiarmreadelf --headres app_name.mcelf_xip`

For the processed code, output will be the following:

\code 


Program Headers:
  Type           Offset   VirtAddr   PhysAddr   FileSiz MemSiz  Flg Align
  NOTE           0x0000b4 0x00000000 0x00000000 0x000f0 0x000f0     0x0
  LOAD           0x0001a4 0x60360000 0x60360000 0x120000 0x120000 RW  0x1000
  LOAD           0x1201a4 0x60510000 0x60510000 0x120000 0x120000 RW  0x1000
  LOAD           0x2401a4 0x606c0000 0x606c0000 0x120000 0x120000 RW  0x1000

\endcode 

Without processing, it looks like the following:

\code 

Program Headers:
  Type           Offset   VirtAddr   PhysAddr   FileSiz MemSiz  Flg Align
  NOTE           0x0000b4 0x00000000 0x00000000 0x00080 0x00080     0x0
  LOAD           0x000134 0x60300000 0x60300000 0x100000 0x100000 RW  0x1000
  LOAD           0x100134 0x60480000 0x60480000 0x100000 0x100000 RW  0x1000
  LOAD           0x200134 0x60600000 0x60600000 0x100000 0x100000 RW  0x1000

\endcode 

on comparing the original program, the segment's address (VirtAddr and PhysAddr) have changed. Earlier, they were 0x60300000, 0x60480000, 0x60600000. Because for every 32B, 4B of ECC has been added, each segment size has been increased from 1MB to 1.125MB and same has happen to address.  