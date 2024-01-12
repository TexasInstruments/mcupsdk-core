# SENT Decoder Using PRUICSS IEP ECAP Example {#EXAMPLES_SENT_DECODER_PRUICSS_IEP_ECAP}

[TOC]

## Introduction

The Single Edge Nibble Transmission protocol is a unidirectional communications method that connects the sensor/transmitting device and the controller/receiving device without the use of a coordination signal from the controller/receiving device. It is intended for usage in situations where high-resolution data must be transferred from a sensor to an engine control unit (ECU). It serves as a simpler, less expensive alternative to CAN, LIN, or PSI5 as well as a replacement for lesser resolution techniques such as traditional sensors that provide analog output voltage and PWM. Automotive applications for SENT-compatible sensor devices include applications for pedal sensing, mass airflow sensing, pressure sensing, temperature sensing, humidity sensing, and many others.

\imageStyle{SENT_Intro.png,width:60%}
\image html SENT_Intro.png "SENT Signal"

**The SENT decoder Example is a reference for software based implementation of SAE J2716. PRU ICSS firmware enables SENT decoder interface on TI Sitara AM263x processor.**
This example does following:
configures pin mux,
sets soc mux to enable gpio mode for ICSS,
initializes PRU, DMEM and PRU INTC module,
downloads crc look up table in PRU Data RAM, and
extracts sent decoded signal to UART terminal.

# Supported Combinations

\cond SOC_AM263X || SOC_AM263PX

| Parameter      | Value                                                 |
| -------------- | ------------------------------------------------------|
| ICSSM          | ICSSM                                                 |
| Toolchain      | pru-cgt                                               | 
| Board          | @VAR_BOARD_NAME_LOWER                                 |
| Example folder | examples/pru_io/sent/decoder_pruicss_iep_ecap/example |

\endcond
## HW Setup
\imageStyle{sent_hw_setup1.jpg,width:60%}
\image html sent_hw_setup1.jpg "Encoder setup "
\imageStyle{sent_hw_setup3.jpg,width:60%}
\image html sent_hw_setup3.jpg "Decoder setup"
\imageStyle{sent_hw_setup2.jpg,width:60%}
\image html sent_hw_setup2.jpg "Encoder+Decoder Setup"
- Requirement: Two <a href="https://www.ti.com/tool/TMDSCNCD263"> AM263x CC </a> with <a href="https://www.ti.com/tool/TMDSHSECDOCK"> HSECDOCK </a>

Connect the folowing pins while using PRUICSS IEP ECAP Example
```
PR0_PRU0_GPIO0 (GPIO93) -> PR0_PRU0_GPIO0 (Pin no. 125)
PR0_PRU0_GPIO1 (GPIO94) -> PR0_PRU0_GPIO1 (Pin no. 126)
PR0_PRU0_GPIO2 (GPIO95) -> PR0_PRU0_GPIO2 (Pin no. 127)
PR0_PRU0_GPIO3 (GPIO96) -> PR0_PRU0_GPIO3 (Pin no. 128)
PR0_PRU0_GPIO4 (GPIO92) -> PR0_PRU0_GPIO4 (Pin no. 124)
PR0_PRU0_GPIO5 (GPIO87) -> PR0_PRU0_GPIO5 (Pin no. 108)
Ground Pin(Any) -> Ground Pin(Any) [To Make sure both boards are having common ground]
```
\imageStyle{sent_decoder_pins_config.png,width:60%}
\image html sent_decoder_pins_config.png "HW setup"

# Steps to Run the Example

- **When using CCS projects to build**, Import sent_decoder_r5f project from the above mentioned Example folder path.
- Build the  projects.
- Load R5F project, run it. Decoder setup is now ready.
- We will be validating decoder demo using SENT encoder example. Next step is to setup sent encoder follow the steps mentioned in \ref EXAMPLES_SENT_ENCODER.

\note
Prerequisite: [PRU-CGT-2-3](https://www.ti.com/tool/PRU-CGT) (ti-pru-cgt) should be installed at: `C:/ti/`

 - Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
 - Open a UART terminal and use config menu to view received SENT frames \ref CCS_UART_TERMINAL
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

```
\note
 - See list supported firmware design and list of supported feature : \ref SENT

# Steps to modify SENT decoder PRU Firmware

Modification supported: Change hardcoded tick period in firmware. Current release supports only 1us tick period support in firmware. In order to modify it:

- **When using CCS projects to build**, Import decoder_pru1_fw project from the above mentioned Example folder path.
- In `main.asm` file modify ch0_ticktime to desired value(1000 to 3000) and accordingly modify ch0_syncpulse_min_dur = (0.8*ch0_ticktime)*56 and ch0_syncpulse_max_dur = (1.2*ch0_ticktime)*56. For example for 500ns tick period.
- Rebuild examples\\pru_io\\sent\\decoder_pruicss_iep_ecap\\decoder_pru0_fw project
- Rebuild examples\\pru_io\\sent\\decoder_pruicss_iep_ecap\\decoder_example
- Load R5F updated binary, run it.

```
ch0_ticktime                .set   500
ch0_syncpulse_min_dur       .set   400*56
ch0_syncpulse_max_dur       .set   600*56
```