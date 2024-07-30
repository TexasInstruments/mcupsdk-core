# How to A/B Swap? Working of bootseg IP  {#bootseg_ip_working}

[TOC]

## Introduction

One of the key component of Firmware Updated-Over-Air (FOTA) is to switch to a new verison of firmware. BootSeg hardware IP has been added in the SOC to alows this switching to the other image. Bootseg IP is part of FSAS area of SOC. The following image shows the location of bootseg hardware block as part of FSAS.

\imageStyle{bootsegIP.png,width:50%}
\image html bootsegIP.png "Bootseg Hardware block. The part that is highlighted in red is the BootSeg IP."

In the above image, all CPU transactions are coming on VBUSM interface and going to `64M to 128M` protocol converter hardware block. After that it goes to BootSeg hardware block which does the address translation.

On the same data path there is `OTFA+ECCM` hardware block which is safety and security block (see \ref OPTIFLASH). Then there is FOTA IP at the bottom. At the end, right side of the above image, is the flash controller (OSPI0).

\note The SDK driver of this IP is at "source/drivers/fss/v0".

The main idea is, at a time, the flash is divided into 2 region. Suppose that flash is 32MB then 1st region is from 0 to 16MB and then 2nd region is from 16MB to 32MB. A valid image can be in any of the region and it is required to boot either from region 1st or region 2nd without application being aware from which region it is being boot from. To achieve this, all address are required to be translated to the correct region.

The following image shows the same:
\imageStyle{abswap_mech_1.png,width:40%}
\image html abswap_mech_1.png "Bootseg IP working"

1. Initially, starting of the flash is considered as Boot Region A (or the active region) and the applcaition that is present (FW V1) in this region is then booted. 
2. During the runtime of FW V1, a new firmware is recieved (FW V2) which is stored in Region B.
3. On next reset, previously the region which was "Region B", is now being set to "Region A" and new firmware (FW V2) is now booted.

It is easier to talk in terms of absolute numbers. That is, there are 2 regions in the flash from which application can boot viz. 0-16MB and 16MB - 32MB (for a flash of 32MB in size) and at boot time, it is required to set one region as Active region and other one as backup region or Region A and Region B.  Bootseg IP, is then used to make either 
1. 0MB -16MB as Active reigon from which to boot and 16MB - 32MB as backup region.
2. 16MB - 32MB as Active reigon from which to boot and 0MB - 16MB as backup region.

Please note that here, we are assuming that size of flash is 32MB and each region size is half of size of flash here.

##  How to make 1st Half as Active Region

In the fss driver `source\drivers\fss\v0\fss.h` there is an API called `FSS_selectRegionA` which will select 1st half of flash as the boot region or Active region and applicaton need to make sure to write in region B of the flash.

##  How to make 2nd Half as Active Region

In the fss driver `source\drivers\fss\v0\fss.h` there is an API called `FSS_selectRegionB` which will select 2nd half of flash as the boot region or Active region and applicaton need to make sure to write in region B of the flash.
