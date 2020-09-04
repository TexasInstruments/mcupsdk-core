# EVM Revision E2 support {#RELEASE_NOTES_08_03_00_EVM_REV_E2_SUPPORT_PAGE}

[TOC]

\note AM263x MCU+ SDK release supports ControlCard Revision E1 and LaunchPad Revision E2.\n
    Limited testing is done for ControlCard E2 revision with this release and not all example may work out of box from SDK on E2 revision ControlCard.\n
    This page has information about using SDK release with E2 revision of ControlCard and lists down SDK examples which require update to run on E2 revision ControlCard.\n


ControlCard
- https://www.ti.com/tool/TMDSCNCD263
- TMDSCNCD263
- AM263x Arm-based MCU general purpose controlCARD development kit

LaunchPad
- https://www.ti.com/tool/LP-AM263
- LP-AM263
- AM263x Arm®-based MCU general-purpose LaunchPad™ development kit


# Pins for DAC and ADC on CC E2 are as follows
DAC Instance    |HSEC PIN
----------------|-------------------------
DAC             |9      (ADC1_MUX_SEL signal LOW)


ADC Instance    |ADC channel    |HSEC PIN
----------------|---------------|---------------------------------
ADC0            |ADC0_AIN0		|9      (ADC1_MUX_SEL signal HIGH)
ADC0            |ADC0_AIN1		|11     (ADC1_MUX_SEL signal HIGH)
ADC0            |ADC0_AIN2		|15
ADC0            |ADC0_AIN3		|17
ADC0            |ADC0_AIN4		|21
ADC0            |ADC0_AIN5		|23
ADC1            |ADC1_AIN0		|12
ADC1            |ADC1_AIN1		|14
ADC1            |ADC1_AIN2		|18
ADC1            |ADC1_AIN3		|20
ADC1            |ADC1_AIN4		|24
ADC1            |ADC1_AIN5		|26
ADC2            |ADC2_AIN0		|31
ADC2            |ADC2_AIN1		|33
ADC2            |ADC2_AIN2		|37
ADC2            |ADC2_AIN3		|39
ADC2            |ADC2_AIN4		|-
ADC2            |ADC2_AIN5		|-
ADC3            |ADC3_AIN0		|28
ADC3            |ADC3_AIN1		|30
ADC3            |ADC3_AIN2		|34
ADC3            |ADC3_AIN3		|36
ADC3            |ADC3_AIN4		|40
ADC3            |ADC3_AIN5		|42
ADC4            |ADC4_AIN0		|25     (ADC2_MUX_SEL signal HIGH)
ADC4            |ADC4_AIN1		|27     (ADC2_MUX_SEL signal HIGH)
ADC4            |ADC4_AIN2		|-
ADC4            |ADC4_AIN3		|-
ADC4            |ADC4_AIN4		|-
ADC4            |ADC4_AIN5		|-

# CMPSS Asynchronous Trip
\ref EXAMPLES_DRIVERS_CMPSS_ASYNCHRONOUS_TRIP

### Changes required
- Change the CMPSS instance from CMPSS0 to say CMPSS1
- Change the neccessary trip signal routing

### Description

A CMPSS example that enables the CMPSS High comparator and feeds the
asynchronous output to GPIO and EPWM

This example enables the ~~CMPSSA0~~ CMPSSA1 COMPH comparator and feeds the asynchronous
CTRIPOUTH signal to the XBAROUT0 pin and CTRIPH to EPWM0B.

CMPSS is configured to generate trip signals to trip the EPWM signals.
CMPIN1P is used to give positive input and internal DAC is configured
to provide the negative input. Internal DAC is configured to provide a
signal at VDD/2. An EPWM signal is generated at EPWM0B and is configured
to be tripped by CTRIPOUTH.

When a low input(VSS) is provided to CMPIN1P,
    - Trip signal(XBAROUT0) output is low
    - EPWM0B gives a PWM signal

When a high input(higher than VDD/2) is provided to CMPIN1P,
    - Trip signal(XBAROUT0) output turns high
    - EPWM0B gets tripped and outputs as high

### External Connections
 - Give input on CMPIN1P (ControlCard HSEC Pin 15)
 - Outputs can be observed on
   - XBAROUT0 (SOC pin QSPI0_CSN1. USER_LED1 on ControlCard)
   - and EPWM0B (ControlCard HSEC pin 51) using an oscilloscope

### Images
\imageStyle{CMPSS_INSTANCE_CHANGE.png,width:40%}
\image html CMPSS_INSTANCE_CHANGE.png "CMPSS_INSTANCE_CHANGE"

\imageStyle{CMPSS_EPWM_XBAR.png,width:40%}
\image html CMPSS_EPWM_XBAR.png "CMPSS_EPWM_XBAR"

\imageStyle{CMPSS_OUTPUT_XBAR.png,width:40%}
\image html CMPSS_OUTPUT_XBAR.png "CMPSS_OUTPUT_XBAR"


# ADC Burst Mode Oversampling
\ref EXAMPLES_DRIVERS_ADC_BURST_MODE_OVERSAMPLING
### Changes required

- Change the ADC0 instance to a different ADC instance, say, ADC1.
- Change the necessary interrupt routing

### Description
This example sets up ePWM0 to periodically trigger a set of conversions
(SOC0,1,12,13,14,15) on ~~ADC0~~ ADC1 for conversion of inputs on ADC_AIN0, ADC_AIN1 and
burst mode conversion on ADC_AIN3.

This demonstrates a batch of conversion on ~~ADC0~~ ADC1 (inputs on ADC_AIN0 and ADC_AIN1)
and burst mode conversion on ~~ADC0~~ ADC1 (input ADC_AIN3)

~~ADC0~~ ADC1 Interrupt ISR is used to read results of ~~ADC0~~ ADC1 (i.e. digital representations
of inputs ADC_AIN0, ADC_AIN1 and average of oversampled ADC_AIN3)



The below watch variables can be used to view ADC conversion results.

Watch Variables
gAdc0Result0 - Digital representation of the voltage on pin ~~ADC0_AIN0~~ ADC1_AIN0
gAdc0result1 - Digital representation of the voltage on pin ~~ADC0_AIN1~~ ADC1_AIN1
gadc0Result2 - Digital representation of the voltage on pin ~~ADC0_AIN3~~ ADC1_AIN3

#### External Connections
~~ADC0_AIN0~~ ADC1_AIN0, ~~ADC0_AIN1~~ ADC1_AIN1, ~~ADC0_AIN3~~ ADC1_AIN3 pins
should be connected to signals to be converted.

Check example.syscfg


### Images
\imageStyle{adc_burst_mode_oversampling_ADC_instance_change.png,width:40%}
\image html adc_burst_mode_oversampling_ADC_instance_change.png "ADC_instance_change"

\imageStyle{adc_burst_mode_oversampling_ADC_INT_XBAR.png,width:40%}
\image html adc_burst_mode_oversampling_ADC_INT_XBAR.png "ADC_INT_XBAR change"


# ADC Differential Mode
\ref EXAMPLES_DRIVERS_ADC_DIFFERENTIAL_MODE

### Changes required

- Change the ADC0 instance to a different ADC instance, say, ADC1.
- Change the necessary interrupt routing

### Description

This example sets up ePWM0 to periodically trigger a set of conversions
(SOC0,1) on ~~ADC0~~ ADC1 for conversion of inputs on ADC_AIN0, ADC_AIN1 in differential modes
(ADC_AIN0 - ADC_AIN1) on SOC0 and (ADC_AIN1 - ADC_AIN0) on SOC1.
Note:  In differential mode, the outputs are symmetric across "2112 or 0x840"
i.e.,
if there is a 1v on differential input, expected output should be around
2752 or 0xAC0.
if there is a -1v on differential input, expected output should be around
1472 or 0x5C0.
Expect overflow if the readings are above 4224
~~ADC0~~ ADC1 Interrupt ISR is used to read results of ~~ADC0~~ ADC1 (i.e. digital representations
of inputs ADC_AIN0, ADC_AIN1 and average of oversampled ADC_AIN3)
The below watch variables can be used to view ADC conversion results.
Watch Variables
gAdc0Result0 - Digital representation of the differential voltage on pins ~~ADC0_AIN0 - ADC0_AIN1~~ ADC1_AIN0 - ADC1_AIN1
gAdc0result1 - Digital representation of the differential voltage on pins ~~ADC0_AIN1 - ADC0_AIN0~~ ADC1_AIN1 - ADC1_AIN0
External Connections
~~ADC0_AIN0~~ ADC1_AIN0, ~~ADC0_AIN1~~ ADC1_AIN1 pins
should be connected to signals to be converted.
Check example.syscfg

### Images
\imageStyle{adc_diiferential_mode_ADC_instance_change.png,width:40%}
\image html adc_diiferential_mode_ADC_instance_change.png "ADC_instance_change"

\imageStyle{adc_diiferential_mode_ADC_INT_XBAR.png,width:40%}
\image html adc_diiferential_mode_ADC_INT_XBAR.png "ADC_INT_XBAR change"




# ADC Multiple SOC EPWM
\ref EXAMPLES_DRIVERS_ADC_MULTIPLE_SOC_EPWM
### Changes required

- Change the ADC0 instance to a different ADC instance, say, ADC2.
- Change the necessary interrupt routing

### Description

This example sets up ePWM0 to periodically trigger a set of conversions
(SOC0,1,2) on ~~ADC0~~ ADC2 and ADC1. This demonstrates multiple ADCs working
together to process a batch of conversions using the available parallelism
across multiple ADCs.
~~ADC0~~ ADC2 Interrupt ISR is used to read results of both ~~ADC0~~ ADC2 and ADC1.
The below watch variables can be used to view ADC conversion results.
External Connections
~~ADC0_AIN0~~ ADC2_AIN0, ~~ADC0_AIN1~~ ADC2_AIN1, ~~ADC0_AIN2~~ ADC2_AIN2 and ADC1_AIN0, ADC1_AIN1, ADC1_AIN2 pins
should be connected to signals to be converted.
Watch Variables
gAdc0Result0 - Digital representation of the voltage on pin ~~ADC0_AIN0~~ ADC2_AIN0
gAdc0result1 - Digital representation of the voltage on pin ~~ADC0_AIN1~~ ADC2_AIN1
adc0Result2 - Digital representation of the voltage on pin ~~ADC0_AIN2~~ ADC2_AIN2
adc1Result0 - Digital representation of the voltage on pin ADC1_AIN0
adc1Result1 - Digital representation of the voltage on pin ADC1_AIN1
adc1Result2 - Digital representation of the voltage on pin ADC1_AIN2

### Images
\imageStyle{adc_multiple_soc_epwm_ADC_instance_change.png,width:40%}
\image html adc_multiple_soc_epwm_ADC_instance_change.png "ADC_instance_change"

\imageStyle{adc_multiple_soc_epwm_ADC_INT_XBAR.png,width:40%}
\image html adc_multiple_soc_epwm_ADC_INT_XBAR.png "ADC_INT_XBAR change"


# ADC PPB EPWM Trip
\ref EXAMPLES_DRIVERS_ADC_PPB_EPWM_TRIP
### Changes required

- Change the ADC0 instance to a different ADC instance, say, ADC1.
- Change the necessary interrupt routing

### Description

This example uses the ADC Post Processing Block to trip a ePWM in case of
out of bound input.

ADCINT1 is configured to periodically trigger the ADC. Initially ADC is
triggered through software.

Limit detection Post processing block is configured and if ADC results are
outside the defined range, the PPB will generate an ADCxEVTy event. This
signal is configured as trip source for EPWM0, 1 and 2.

This example showcases one-shot, cycle-by-cycle and direct tripping of PWMs.
- EPWM0 is configured for One Shot trip.
- EPWM1 is configured for Cycle by Cycle trip.
- EPWM2 is configured for Direct trip via Digital compare submodule.

External Connections
~~ADC0_AIN2~~ ADC1_AIN2 pin should be connected to DAC output pin.
EPWM0A, EPWM0B, EPWM1A, EPWM1B, EPWM2A, EPWM2B can be connected to an
oscilloscope to validate tripping.

Watch Variables
adc0Result2 - Digital representation of the voltage on pin ~~ADC0_AIN2~~ ADC1_AIN2

### Images

\imageStyle{adc_ppb_epwm_trip_ADC_instance_change.png,width:40%}
\image html adc_ppb_epwm_trip_ADC_instance_change.png "ADC_instance_change"

\imageStyle{adc_ppb_epwm_trip_ADC_INT_XBAR.png,width:40%}
\image html adc_ppb_epwm_trip_ADC_INT_XBAR.png "ADC_INT_XBAR change"

\imageStyle{adc_ppb_epwm_trip_ADC_EPWM_XBAR.png,width:40%}
\image html adc_ppb_epwm_trip_ADC_EPWM_XBAR.png "ADC_EPWM_XBAR change"


# ADC SOC Continuous DMA
\ref EXAMPLES_DRIVERS_ADC_SOC_CONTINUOUS_DMA

### Changes required

- Change the ADC0 instance to a different ADC instance, say, ADC2.
- Change the necessary interrupt routing

### Description

This example sets up two ADC channels to convert simultaneously. The
results will be transferred by DMA into a buffer in RAM.

It configures ePWM0 to trigger SOC0 on ~~ADC0~~ ADC2 and ADC1. EPWM is only used to
trigger the first ADC conversion. INT0 of ~~ADC0~~ ADC2 is configured to generate
interrupt after first conversion and will then disable EPWM SOC generation.
INT1 of both ADC's is configured to enable continuous conversion.

DMA channel 0 is triggered at EOC0 of ~~ADC0~~ ADC2 and will copy conversion result
to a buffer in RAM. DMA channel 1 is triggered at EOC0 of ADC1 and will copy
conversion result to another buffer in RAM. DMA will generate interrupt after
the buffer is filled and will stop conversion on both ADC's.

The below watch variables can be used to view ADC conversion results.

External Connections
~~ADC0_AIN0~~ ADC2_AIN0 and ADC1_AIN0 pins should be connected to signals to be converted.

Watch Variables
gAdc0DataBuffer - Buffers which stores conversion results from ADC0
gAdc1DataBuffer - Buffers which stores conversion results from ADC1

### Images
\imageStyle{adc_soc_continuos_dma_ADC_instance_change.png,width:40%}
\image html adc_soc_continuos_dma_ADC_instance_change.png "ADC_instance_change"

\imageStyle{adc_soc_continuous_dma_ADC_INT_XBAR.png,width:40%}
\image html adc_soc_continuous_dma_ADC_INT_XBAR.png "ADC_INT_XBAR change"

\imageStyle{adc_soc_continuous_dma_EDMA_CHANNEL_TRIGGER.png,width:40%}
\image html adc_soc_continuous_dma_EDMA_CHANNEL_TRIGGER.png "EDMA_CHANNEL_TRIGGER_SELECT change"



# ADC SOC software
\ref EXAMPLES_DRIVERS_ADC_SOC_SOFTWARE
### Changes required

- Change the ADC0 instance to a different ADC instance, say, ADC1.
- Change the necessary interrupt routing

### Description

This example uses the ADC module to perform an ADC SOC conversion
triggered by software.

In this example ~~ADC0~~ ADC1 is used to convert the SOC, the user can also
select a different one.

This example also showcases how to configure and use the ADC module.

### Images
\imageStyle{adc_soc_software_ADC_instance_change.png,width:40%}
\image html adc_soc_software_ADC_instance_change.png "ADC_instance_change"




