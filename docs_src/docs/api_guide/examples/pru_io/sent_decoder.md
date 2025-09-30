# SENT Decoder Example {#EXAMPLES_SENT_DECODER}

[TOC]

## Introduction

The Single Edge Nibble Transmission protocol is a unidirectional communications method that connects the sensor/transmitting device and the controller/receiving device without the use of a coordination signal from the controller/receiving device. It is intended for usage in situations where high-resolution data must be transferred from a sensor to an engine control unit (ECU). It serves as a simpler, less expensive alternative to CAN, LIN, or PSI5 as well as a replacement for lesser resolution techniques such as traditional sensors that provide analog output voltage and PWM. Automotive applications for SENT-compatible sensor devices include applications for pedal sensing, mass airflow sensing, pressure sensing, temperature sensing, humidity sensing, and many others.

\imageStyle{SENT_Intro.png,width:60%}
\image html SENT_Intro.png "SENT Signal"

\cond SOC_AM263X
**The SENT decoder Example is a reference for software based implementation of SAE J2716. PRU ICSS firmware enables SENT decoder interface on TI Sitara AM263x processor.**
\endcond
\cond SOC_AM263PX
**The SENT decoder Example is a reference for software based implementation of SAE J2716. PRU ICSS firmware enables SENT decoder interface on TI Sitara AM263Px processor.**
\endcond
\cond SOC_AM261X
**The SENT decoder Example is a reference for software based implementation of SAE J2716. PRU ICSS firmware enables SENT decoder interface on TI Sitara AM261x processor.**
\endcond
This example does following:
configures pin mux,
sets soc mux to enable gpio mode for ICSS,
initializes PRU, DMEM and PRU INTC module,
downloads crc look up table in PRU Data RAM, and
extracts sent decoded signal to UART terminal.

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
| Parameter      | Value                                         |
| -------------- | ----------------------------------------------|
| CPU + OS       | r5fss0-0 freertos                             |
| Toolchain      | ti-arm-clang                                  |
| Board          | @VAR_BOARD_NAME_LOWER                         |
| Example folder | examples/pru_io/sent/decoder/example          |
\endcond
\cond SOC_AM261X
| Parameter      | Value                                         |
| -------------- | ----------------------------------------------|
| CPU + OS       | r5fss0-0 freertos                             |
| Toolchain      | ti-arm-clang                                  |
| Board          | am261x-lp                                     |
| Example folder | examples/pru_io/sent/decoder/example          |
\endcond

\cond SOC_AM263X || SOC_AM263PX
| Parameter      | Value                                              |
| -------------- | ---------------------------------------------------|
| ICSSM          | ICSSM0 - PRU0                                      |
| Toolchain      | pru-cgt                                            |
| Board          | @VAR_BOARD_NAME_LOWER                              |
| Example folder | examples/pru_io/sent/decoder/firmware/pru(0 or 1)  |
\endcond
\cond SOC_AM261X
| Parameter      | Value                                              |
| -------------- | ---------------------------------------------------|
| ICSSM          | ICSSM1 - PRU0                                      |
| Toolchain      | pru-cgt                                            |
| Board          | am261x-lp                                          |
| Example folder | examples/pru_io/sent/decoder/firmware/pru(0 or 1)  |
\endcond

## HW Setup

\cond SOC_AM263PX || SOC_AM263X

\imageStyle{sent_hw_setup1.jpg,width:60%}
\image html sent_hw_setup1.jpg "Encoder setup "
\imageStyle{sent_hw_setup3.jpg,width:60%}
\image html sent_hw_setup3.jpg "Decoder setup"
\imageStyle{sent_hw_setup2.jpg,width:60%}
\image html sent_hw_setup2.jpg "Encoder+Decoder Setup"
\cond SOC_AM263PX
- Requirement: Two <a href="https://www.ti.com/tool/TMDSCNCD263P"> AM263px CC </a> with <a href="https://www.ti.com/tool/TMDSHSECDOCK"> HSECDOCK </a>
\endcond
\cond SOC_AM263X
- Requirement: Two <a href="https://www.ti.com/tool/TMDSCNCD263"> AM263x CC </a> with <a href="https://www.ti.com/tool/TMDSHSECDOCK"> HSECDOCK </a>
\endcond

\endcond

\cond SOC_AM261X

\imageStyle{am261x_reva_encoder_decoder_setup.jpg,width:60%}
\image html am261x_reva_encoder_decoder_setup.jpg "Encoder+Decoder Setup"
- Requirement: Two <a href="https://www.ti.com/tool/LP-AM261"> AM261x LP </a>

\endcond

\cond SOC_AM263X

Connect the following pins of HSECDOCK as shown in the below image
```
PR0_PRU0_GPIO0 -> PR0_PRU0_GPIO0 (Pin no. 125)
PR0_PRU0_GPIO1 -> PR0_PRU0_GPIO1 (Pin no. 126)
PR0_PRU0_GPIO2 -> PR0_PRU0_GPIO2 (Pin no. 127)
PR0_PRU0_GPIO3 -> PR0_PRU0_GPIO3 (Pin no. 128)
PR0_PRU0_GPIO4 -> PR0_PRU0_GPIO4 (Pin no. 124)
PR0_PRU0_GPIO5 -> PR0_PRU0_GPIO5 (Pin no. 108)
PR0_PRU0_GPIO6 -> PR0_PRU0_GPIO6 (Pin no. 123)
PR0_PRU0_GPIO8 -> PR0_PRU0_GPIO8 (Pin no. 122)
Ground Pin(Any) -> Ground Pin(Any) [To Make sure both boards are having common ground]
```
\imageStyle{sent_decoder_pins_config.png,width:60%}
\image html sent_decoder_pins_config.png "HW setup"
\endcond
\cond SOC_AM263PX

Connect the following pins of HSECDOCK as shown in the below image
```
PR0_PRU0_GPIO0 -> PR0_PRU0_GPIO0 (Pin no. 125)
PR0_PRU0_GPIO1 -> PR0_PRU0_GPIO1 (Pin no. 126)
PR0_PRU0_GPIO2 -> PR0_PRU0_GPIO2 (Pin no. 127)
PR0_PRU0_GPIO3 -> PR0_PRU0_GPIO3 (Pin no. 128)
PR0_PRU0_GPIO4 -> PR0_PRU0_GPIO4 (Pin no. 124)
PR0_PRU0_GPIO5 -> PR0_PRU0_GPIO5 (Pin no. 108)
PR0_PRU0_GPIO6 -> PR0_PRU0_GPIO6 (Pin no. 123)
PR0_PRU0_GPIO8 -> PR0_PRU0_GPIO8 (Pin no. 122)
Ground Pin(Any) -> Ground Pin(Any) [To Make sure both boards are having common ground]
```
\imageStyle{sent_decoder_pins_config.png,width:60%}
\image html sent_decoder_pins_config.png "HW setup"
\endcond

\cond SOC_AM261X
Connect the following pins of AM261x as shown in the below image
```
PR0_PRU0_GPIO0 -> PR0_PRU0_GPIO0 (Pin no. J2-11)
PR0_PRU0_GPIO1 -> PR0_PRU0_GPIO1 (Pin no. J7-67)
PR0_PRU0_GPIO3 -> PR0_PRU0_GPIO3 (Pin no. J7-40)
PR0_PRU0_GPIO6 -> PR0_PRU0_GPIO6 (Pin no. J7-69)
PR0_PRU0_GPIO7 -> PR0_PRU0_GPIO7 (Pin no. J7-64)
PR0_PRU0_GPIO8 -> PR0_PRU0_GPIO8 (Pin no. J7-65)
PR0_PRU0_GPIO11 -> PR0_PRU0_GPIO11 (Pin no. J7-37)
PR0_PRU0_GPIO13 -> PR0_PRU0_GPIO13 (Pin no. J8-35)
Ground Pin(Any) -> Ground Pin(Any) [To Make sure both boards are having common ground]
```
\endcond

# Steps to Run the Example {#EXAMPLES_SENT_DECODER_STEPS_TO_RUN}

- **When using CCS projects to build**, import the CCS project from the above mentioned Example folder path for R5F and PRU, After this `main.asm`, `linker.cmd` files gets copied to ccs workspace of PRU project.

- Build the PRU project using the CCS project menu (This step is optional, used when the firmware is modified) (see \ref CCS_PROJECTS_PAGE).
     - Build Flow: Once you click on build in PRU project, firmware header file which is generated in release or debug folder of ccs workspace, is moved to  `<sdk-install-dir/examples/pru_io/sent/decoder/firmware/pru0/device/>` or `<sdk-install-dir/examples/pru_io/sent/decoder/firmware/pru1/device/>` based on which firmware was
     used.

- Build the R5F project using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
     - Firmware header file path is included in R5F project include options by default, Instructions in Firmware header file can be written into PRU IRAM memory using \ref PRUICSS_loadFirmware API call
     - Build Flow: Once you click on build in R5F project, SysConfig files are generated, Finally the R5F project will be generated using both the generated SysConfig and PRU project binaries.
     - Note: The PRU project won't run independently as it is dependent on SysConfig files generated by the R5F project to intialize pru.

    \note
    Prerequisite: [PRU-CGT-2-3](https://www.ti.com/tool/PRU-CGT) (ti-pru-cgt) should be installed at: `C:/ti/`

- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

\note
Prerequisite: [PRU-CGT-2-3](https://www.ti.com/tool/PRU-CGT) (ti-pru-cgt) should be installed at: `C:/ti/`

 - Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
 - Open a UART terminal as per \ref CCS_UART_TERMINAL and view received SENT frames 

### Sample Output

```
******************SENT DECODER Application**********************


SENT PRU-ICSS firmware loaded and running




******************CHANNEL0 SENT DATA**********************

Number of Frames received:       9
Channel0 calculated tick period:         538 ns
channel0 StatusComm Data:        08
Channel0:  Data0 Data1 Data2 Data3 Data4 Data5 CRC
Value:    07     04      08      07      04      08      03
**********************************************************

******************CHANNEL1 SENT DATA**********************

Number of Frames received:       9
Channel0 calculated tick period:         538 ns
channel0 StatusComm Data:        08
Channel0:  Data0 Data1 Data2 Data3 Data4 Data5 CRC
Value:    07     04      08      07      04      08      03
**********************************************************

******************CHANNEL2 SENT DATA**********************

Number of Frames received:       9
Channel0 calculated tick period:         538 ns
channel0 StatusComm Data:        08
Channel0:  Data0 Data1 Data2 Data3 Data4 Data5 CRC
Value:    07     04      08      07      04      08      03
**********************************************************

******************CHANNEL3 SENT DATA**********************

Number of Frames received:       9
Channel0 calculated tick period:         538 ns
channel0 StatusComm Data:        08
Channel0:  Data0 Data1 Data2 Data3 Data4 Data5 CRC
Value:    07     04      08      07      04      08      03
**********************************************************

******************CHANNEL4 SENT DATA**********************

Number of Frames received:       9
Channel0 calculated tick period:         538 ns
channel0 StatusComm Data:        08
Channel0:  Data0 Data1 Data2 Data3 Data4 Data5 CRC
Value:    07     04      08      07      04      08      03
**********************************************************

******************CHANNEL5 SENT DATA**********************

Number of Frames received:       9
Channel0 calculated tick period:         538 ns
channel0 StatusComm Data:        08
Channel0:  Data0 Data1 Data2 Data3 Data4 Data5 CRC
Value:    07     04      08      07      04      08      03
**********************************************************

******************CHANNEL6 SENT DATA**********************

Number of Frames received:       9
Channel0 calculated tick period:         538 ns
channel0 StatusComm Data:        08
Channel0:  Data0 Data1 Data2 Data3 Data4 Data5 CRC
Value:    07     04      08      07      04      08      03
**********************************************************

******************CHANNEL7 SENT DATA**********************

Number of Frames received:       9
Channel0 calculated tick period:         538 ns
channel0 StatusComm Data:        08
Channel0:  Data0 Data1 Data2 Data3 Data4 Data5 CRC
Value:    07     04      08      07      04      08      03
**********************************************************

```
\note
 - See list supported firmware design and list of supported feature : \ref SENT

# Steps to modify SENT decoder PRU Firmware
## Modifying Tick Period
Modification supported: Change hardcoded tick period in firmware. Current release supports only 1us tick period support in firmware. In order to modify it:

- **When using CCS projects to build**, Import decoder_pru1_fw project from the above mentioned Example folder path.
- In `main.asm` file modify ch0_ticktime to desired value(1000 to 3000) and accordingly modify ch0_syncpulse_min_dur = (0.8*ch0_ticktime)*56 and ch0_syncpulse_max_dur = (1.2*ch0_ticktime)*56. For example for 500ns tick period.
- Refer the following section for running the example after altering the firmware [Steps to Run the Example](\ref EXAMPLES_SENT_DECODER_STEPS_TO_RUN)

```
ch0_ticktime                .set   500
ch0_syncpulse_min_dur       .set   400*56
ch0_syncpulse_max_dur       .set   600*56
```

## Modifying the pins used
1.The user needs to select the correct pins with proper pinmux configurations in sysconfig.

2.Update PRU0 Channel State Definitions
Modify the channel state values in the PRU0 firmware's main.asm file to match your desired pin configuration.
Update these channel state values according to your pin requirements.

```
channel0_state .set    0    -> Pin for Channel 0,

channel1_state .set    1    -> Pin for Channel 1,

channel2_state .set    3    -> Pin for Channel 2,

channel3_state .set    6    -> Pin for Channel 3,

...additional channels as needed
```
3.Adjust PRU1 Channel Mask Configuration
Update the channel mask definitions in the PRU1's header.inc file to align with your pin selection.
Modify channel masks to match your hardware configuration.
```
channel0_mask .set   (0x00)    -> Mask for Channel 0,

channel1_mask .set   (0x01)    -> Mask for Channel 1,

channel2_mask .set   (0x03)    -> Mask for Channel 2,

channel3_mask .set   (0x06)    -> Mask for Channel 3,

... additional channels as needed
```
4.Rebuild and Deploy

After making these modifications:

- Import the project into Code Composer Studio (CCS)
- Build the project to incorporate your changes
- Deploy the updated firmware to your target device
- Run the application to validate your custom pin configuration
- Refer the following section for running the example after altering the firmware [Steps to Run the Example](@ref EXAMPLES_SENT_DECODER_STEPS_TO_RUN)
