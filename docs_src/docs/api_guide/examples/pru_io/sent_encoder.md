# SENT Encoder Example {#EXAMPLES_SENT_ENCODER}

[TOC]

## Introduction

This example emulates SENT sensor using PRU ICSS.

**The SENT encoder Example is a reference for software based implementation of sent sensor. It is meant for only validating \ref EXAMPLES_SENT_DECODER .**

## SENT Signal

The sensor sends out a signal that is made up of a string of pulses with data encoded as falling to falling edge periods. It happens independently of any receiver module activity and takes place without the receiver module sending a synchronization signal. The modulated signal with a constant amplitude voltage and an evaluation of the time interval between two falling edges (a single edge) is delivered in units of 4 bits (1 nibble), which can represent values ranging from 0 to 15. According to the SAE J2716 specification, a transmitter-specific nominal clock period that is used to quantify the pulse period (tick time) can be between 3-90 ms. The sensor device can use low-cost RC oscillators because the maximum permitted clock deviation is 20% from the normal tick time.
The specified tick time, the sent data value, and the message time of a SENT message determine its duration.
The following pulses (all with nominal timings) make up the transmitting sequence:

1.  Synchronization/calibration pulse, first (56 clock ticks)
2.  A single pulse with a 4 bit Status and Serial Communication header (12 to 27 clock ticks)
3.  A series of one to six data nibble pulses, each reflecting the values of the signal(s) to be conveyed and lasting 12 to 27 clock ticks. Since the trailing falling edge of the SENT transmission CRC nibble also serves as the leading falling edge of the subsequent SENT transmission synchronization/calibration pulse, the number of nibbles will be fixed for each application of the encoding scheme (such as throttle position sensors, mass air flow, etc.).
4.  One 4-bit checksum(CRC) nibble pulse (12 to 27 clock ticks)
5.  One optional pause pulse

\imageStyle{sent_signal.png,width:60%}
\image html sent_signal.png "SENT Signal"
# Supported Combinations
\cond SOC_AM263X || SOC_AM263PX
| Parameter      | Value                                        |
| -------------- | ---------------------------------------------|
| CPU + OS       | r5fss0-0 freertos                            |
| Toolchain      | ti-arm-clang                                 |
| Board          | @VAR_BOARD_NAME_LOWER                        |
| Example folder | examples/pru_io/sent/encoder/example         |
\endcond
\cond SOC_AM261X
| Parameter      | Value                                        |
| -------------- | ---------------------------------------------|
| CPU + OS       | r5fss0-0 freertos                            |
| Toolchain      | ti-arm-clang                                 |
| Board          | am261x-lp                                    |
| Example folder | examples/pru_io/sent/encoder/firmware        |
\endcond

\cond SOC_AM263X || SOC_AM263PX
| Parameter      | Value                                        |
| -------------- | ---------------------------------------------|
| ICSSM          | ICSSM0 - PRU0                                |
| Toolchain      | pru-cgt                                      |
| Board          | @VAR_BOARD_NAME_LOWER                        |
| Example folder | examples/pru_io/sent/encoder/firmware        |
\endcond
\cond SOC_AM261X
| Parameter      | Value                                        |
| -------------- | ---------------------------------------------|
| ICSSM          | ICSSM1 - PRU0                                |
| Toolchain      | pru-cgt                                      |
| Board          | am261x-lp                                    |
| Example folder | examples/pru_io/sent/encoder/firmware        |
\endcond

# Steps to Run the Example {#EXAMPLES_SENT_ENCODER_STEPS_TO_RUN}

- **When using CCS projects to build**, import the CCS project from the above mentioned Example folder path for R5F and PRU, After this `main.asm`, `linker.cmd` files gets copied to ccs workspace of PRU project.

- Build the PRU project using the CCS project menu (This step is optional, used when the firmware is modified) (see \ref CCS_PROJECTS_PAGE).
     - Build Flow: Once you click on build in PRU project, firmware header file which is generated in release or debug folder of ccs workspace, is moved to  `<sdk-install-dir/examples/pru_io/sent/encoder/firmware/device/>`

- Build the R5F project using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
     - Firmware header file path is included in R5F project include options by default, Instructions in Firmware header file can be written into PRU IRAM memory using \ref PRUICSS_loadFirmware API call
     - Build Flow: Once you click on build in R5F project, SysConfig files are generated, Finally the R5F project will be generated using both the generated SysConfig and PRU project binaries.
     - Note: The PRU project won't run independently as it is dependent on SysConfig files generated by the R5F project to intialize pru.

\note
Prerequisite: [PRU-CGT-2-3](https://www.ti.com/tool/PRU-CGT) (ti-pru-cgt) should be installed at: `C:/ti/`

- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Open a UART terminal as per \ref CCS_UART_TERMINAL and use config menu to send SENT frames

### Sample Output

```
******************SENT ENCODER Application**********************

SENT PRU-ICSS firmware loaded and running



Test with one sent frame: Y or N
y

SENT Frame...transmitted

Number of Frames transmitted:    1
Test with one sent frame: Y or N
n

Exiting Application
```
\note
    - \note
 - See list supported firmware design and list of supported feature : \ref SENT
# Steps to modify SENT encoder example
## Modifying Tick Period
Modification supported: Change hardcoded tick period in fw. Current release supports only 500ns tick period support in firmware. In order to modify it:

- **When using CCS projects to build**, Import encoder_pru_fw project from the above mentioned Example folder path.
- In `main.asm` file modify TICK_PRD_100ns to desired value divided by 100. For example for 3000ns.
```
 TICK_PRD_100ns .set  30 ;Hard coding Tick period for now
```
- Refer the following section for running the example after altering the firmware [Steps to Run the Example](\ref EXAMPLES_SENT_ENCODER_STEPS_TO_RUN)

## Modifying the pins used
The user would need to change the R30 value (r30_value) in the defines.inc file located in examples\\pru_io\\sent\\encoder\\firmware according to the
value of the GPIO that they need to use.
For example, if GPIO0,1,2,3,4,5,6,8 are used then those bits need to be set high which would be 0x17F in hex.

Refer the following section for running the example after altering the firmware [Steps to Run the Example](\ref EXAMPLES_SENT_ENCODER_STEPS_TO_RUN)


