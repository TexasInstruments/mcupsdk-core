# How to use Flash Operation Scheduler Hardware {#FLSOPSKD_IP}

[TOC]

## Introduction

This page goes over different aspect of flash operation scheduler (FLSOPSKD) hardware engine. 


### Why is FLSOPSKD Hardware Engine?

OptiFlash technology (\ref OPTIFLASH) enables XIP, in case, where an application is performing flash writes, would look like as follows:

The following image shows the same:
\imageStyle{general_flash_access_request_multicore.png,width:40%}
\image html general_flash_access_request_multicore.png "General Flash Access Usecase"

Here each CPU is requestion data from the external flash through `flash controller` for performing XIP or read from the external flash and sending erase, write or status read request. On top of this, each request is asynchronous in nature, meaning that any request can come at anytime.

External flash is connected to microcontroller over 8 data lines (OSPI) and on same data lines, both reads and writes would happen. And in any flash write scenerio, writes would be happening while XIP is going on. During writing, reading should be stopped which also mean that XIP needs to be stopped and this would have an impact on performance and increased complexity in system design.

To synchronize all the read and write request on same 8bit data bus and to minimize the XIP/Read downtime, FLSOPSKD IP has been added. This will help schedule different flash related transaction by prioritizing XIP/Reads. XIP/Reads are prioritized because writes and erase to a nor flash are anyway very slow (may take multiple seconds for a single operation, depending on flash).

### What is FLSOPSKD Hardware Engine?

To achieve this sort of scheduling, with Optiflash, FLSOPSKD Hardware Engine has been added on top of Flash controller. 

The following image shows the same:
\imageStyle{flsopskd_internal.png,width:40%}
\image html flsopskd_internal.png "Flash Operation Scheduler Hardware"

One key feature of this hardware is that there is a 8051MCU inside this which is also programable. It has its own program memory (2KB in case of AM263Px) and data memory of 256B.

This 8051 has a very close access to flash controller's configuration and some hooks to know the state of the flash controller (or OSPI controller in case of AM263Px).

R5F CPU or anyother CPU, when it is required to communicate to 8051 would program the MMR registers and signal the firmware that is running inside the 8051 to further process it.

\note  TI provide an 8051 firmware OOB as an example. 

## Example Firmware

TI Provides some recommendation on how to write custom implementation of the above described scheduling.

The following image shows the same:
\imageStyle{fw_example_implementation.png,width:40%}
\image html fw_example_implementation.png "Firmware Example Implementation"

On the left hand side, any core which is trying to communicate to 8051 fiirmware would follow the and on the right hand side are the steps that 8051 would follow after recieving signal from SOC core.

Based on the above steps, MCU+ SDK drivers come with prebuit firmware of 8051 and its corresponding R5F drivers. 

### Communicating with 8051 firmware.
\note 
This Firmware works only for IS25LX256-LHLE NOR flash memory and AM263Px-SIP package.

\note
Before running this driver, make sure that SOC is PORz

#### Any operation general format
Any operation that is perform by 8051 is generally in format:
1.	wait for go bit
2.	wait for OSPI IP to be IDLE 
3.	perform the required operation 
4.	Save status/error code in the corresponding registers.
5.	Send operation finish interrupt to R5F cores.

8051 FW supports the following operations 
1.	scheduled Write
2.	scheduled Erase
3.	scheduled General Command

#### Initializing FLSOPSKD IP 
FLSOPSKD IP should be initilized by following procedure:
1.	Put 8051 in reset state
2.	Enable and initilize 8051 memory  
    1. This step consist of enabling R5F access to 8051 code and data access.
    2. wait for memory to finish initilization
3.	Load 8051 firmware in the 8051 code/data memory and verify it.
    1. If cache is enabled for this memory then Cache write back should be performed to maintain cache coherency. 
4.	Disable R5F access to 8051 memories to avoid unintentional modification of 8051 firmware.
5.	Enable required interrupts in the IP and R5F.
6.	Lift 8051 from reset

#### Performing Scheduled Flash Erase 
To perform flash erase R5F should do the following
1.	set GP0 register to 1. This will indicate 8051 that R5F is requesting Erase operation. 
2.	set ADDR register to sector offset. Currently this FW supports 128KB erase. Hence address should be aligned to 128KB Otherwise there will be undefined behavior.   
3.	Clear the interrupts
4.	Set the GO bit in` FOTA_CTRL` register.
Once the GO bit is set, then R5F should wait for `FOTA DONE` interrupt and check for error codes or status code.
Current implementation only supports FOTA done bit in `IRQ_STATUS_RAW.fota_done`. It does not provide any error code and status code. It also assumes that flash is configured.

#### Performing Scheduled Flash Write  
Current 8051 implements 256B page writes.
Completion of this is notified by IRQ_STATUS_RAW.fota_done bit and does not writes any error code and status code. 
to write a page to flash perform the following:
1.	set GP0 = 0. This tell 8051 to perform write operation.
2.	set ADD register to flash offset to which data is to be written. This offset should be 256B aligned.
3.	set CNT register to 256, which is the page size.
4.	Copy the buffer to `CSL_FSS_WBUF_GENREGS_REGS_BASE` (`0x5380E000U` in case of am263px) and `CSL_FSS_WBUF_GENREGS_REGS_BASE` + 256B location.
1.	This is due to some hardware issue 
5.	Set the GO bit in `FOTA_CTRL` register.
Once the GO bit is set, then R5F should wait for FOTA DONE interrupt and check for error codes or status code

#### Configuring Scheduled General Command Send   
This operation internally uses OSPI STIG feature. 
for any command to send, it needs the following fields 
 S.No|	Field
------|--------
 1      |	Opcode
 2      |	Extended Opcode
 3      |	Number of address bytes to send
 4      | Address 
 5      |	Number of dummy bits to send 
 6      |	Number of bytes to read 
 7      | Number of bytes to write
 8      | write bytes


Current 8051 supports setting all the above fields. Other than that there are other operations, like setting up the tx buffer, reading rx buffer, etc. 

##### General format of setting fields for setting general command fields  
1.	Set GP0 to `XXXXXX02`. Where `X` can have any value.
2.	clear interrupt 
3.	set GO bit
4.	wait for operation to complete

##### Setting Opcode  
To do this send `XXXX0002` where `XXXX` is the opcode that is required to be programmed. For example, `0x7c` is the opcode that is to be programmed then GP0 should be set to `007C0002` in hex.

##### Setting Extended Opcode  
Set GP0 to `XXXX0102` where `XXXX` is the extended opcode that is to be set.

##### Setting Number of address bytes to send  
For this set GP0 to `XXXX0502` there `XXXX` can have the following values 
S.No	|Number of address bytes to send |Value of `XXXX`
--------|-------------------------------------|---------------- 
1	|0	|0h
2	|1	|8h
3	|2	|9h
4	|3	|Ah
5	|4	|Bh

##### Setting Number of bytes to write  
set GP0 to `XXXX0402` where `XXXX` can have the following values
Number of bytes to write	|Value of `XXXX` 
-----------------------------|----------------
0	|0x0
1	|0x8
2	|0x9
3	|0xA
4	|0xB
5	|0xC
6	|0xD
7	|0xE
8	|0xF

For example, if it is required to send 8 bytes to write then GP0 should be set to `000F0402` 

##### Setting number of bytes to read  
for thsi set GP0 tp `XXXX0302` there `XXXX` can have the following values:
Number of bytes to write	|Value of `XXXX` 
-----------------------------|----------------
0	|0x0
1	|0x8
2	|0x9
3	|0xA
4	|0xB
5	|0xC
6	|0xD
7	|0xE
8	|0xF

##### Setting Address  

Its values is to be directly written to `FOTA_ADDR` register.

##### Filling tx buffer  
If this general command is going to write then its needs some data to write. This write data can be setup by using the following procedure (assuming 8B data is to be written):
1.	Write lower 32bits to `FOTA_ADDR` register
2.	set `GP0` to `00000702` and execute the operation
3.	Write upper 32bits to `FOTA_ADDR` register
4.	set `GP0` to `00000802` and execute the operation 

##### Reading rx Buffer  
If the after sending the STIG command, it is expected some data (max 8B) then to read the received data, do the following process (assuming it is required to read 8B):
1.	set `GP0` to `0000_0A02` and execute the command. 
2.	read upper 32bits from GP1 register. 
3.	set `GP1` to `0000_0902` and execute the command. 
4.	read lower 32bits from GP1 register.

##### Setting Dummy bits  
set `GP0` to `XXXX0602` and execute the command, For example if 4 dummy bits is required to be sent then set `GP0` to `00040602`. maximum value of dummy bits is 32.

#### Sending Scheduled General Command Send   
To send this command perform the following process:
1.	set GP0 register to 3. This will indicate 8051 that R5F is requesting Erase operation. 
2.	set ADDR register to address value if required.
3.	Clear the interrupts
4.	Set the GO bit in `FOTA_CTRL` register.


