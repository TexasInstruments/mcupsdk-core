# ADC DAC Loopback {#EXAMPLES_DRIVERS_ADC_DAC_LOOPBACK}

[TOC]

# Introduction
This example demostrates the ADC-DAC loopback feature. In AM263Px the DAC can be connected internally to the ADC CAL1 Channels. each ADC has two CAL channels viz. CAL Channel 1 and CAL Channel 2. these CAL channels are common to all the ADCs. While using the ADC-DAC loopback, the following needs to be taken care of,
1. ADC cal channel should be of high impedence, i.e., no source driving it.
2. DAC output should be driven only by DAC and there should not be any other source driving it.
3. ADC sampling times must be increased in this mode. the example uses the 256 Sample and Hold window configuration.  
4. Sampling the DAC voltage with multiple ADCs at the same time will produce inconsistent results. Due to sampling kickback and charge injection. Hence, only one ADC may be used with the given CAL channel at any point of using the DAC-ADC loopback.

## Configurations
- DAC is configured using syscfg
- ADC SOC0 is configured CAL1 Channel, with sample and hold window of 256 and generate interrupt at EOC0

# External Connections
No external connections required

# Watch Variables
The below watch variables can be used to view ADC conversion results.
- gAdcResults[]        : Digital representation of the voltage sample on pin AIN2 of ADC0, triggered by RTI
- gDac_shadowValues[]  : DAC digital values corresponding to the ADC conversions. 

# Supported Combinations {#EXAMPLES_DRIVERS_ADC_DAC_LOOPBACK_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/adc/adc_dac_loopback/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Using watch variables, view the ADC conversion results.
- View the ADC conversion results in UART console logs

# See Also

\ref DRIVERS_ADC_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
ADC DAC Loopback Test Started ...
DAC code	|	ADC Result
	100	|	101
--->0.08057V	|	0.07891V
	200	|	202
--->0.16113V	|	0.15781V
	300	|	304
--->0.24170V	|	0.23750V
	400	|	406
--->0.32227V	|	0.31719V
	500	|	508
--->0.40283V	|	0.39687V
	600	|	612
--->0.48340V	|	0.47813V
	700	|	716
--->0.56396V	|	0.55937V
	800	|	819
--->0.64453V	|	0.63984V
	900	|	921
--->0.72510V	|	0.71953V
	1000	|	1024
--->0.80566V	|	0.80000V
	1100	|	1125
--->0.88623V	|	0.87891V
	1200	|	1227
--->0.96680V	|	0.95859V
	1300	|	1328
--->1.04736V	|	1.03750V
	1400	|	1430
--->1.12793V	|	1.11719V
	1500	|	1533
--->1.20850V	|	1.19766V
	1600	|	1636
--->1.28906V	|	1.27813V
	1700	|	1742
--->1.36963V	|	1.36094V
	1800	|	1847
--->1.45020V	|	1.44297V
	1900	|	1954
--->1.53076V	|	1.52656V
	2000	|	2060
--->1.61133V	|	1.60938V
	2100	|	2166
--->1.69189V	|	1.69219V
	2200	|	2269
--->1.77246V	|	1.77266V
	2300	|	2372
--->1.85303V	|	1.85312V
	2400	|	2474
--->1.93359V	|	1.93281V
	2500	|	2577
--->2.01416V	|	2.01328V
	2600	|	2681
--->2.09473V	|	2.09453V
	2700	|	2786
--->2.17529V	|	2.17656V
	2800	|	2890
--->2.25586V	|	2.25781V
	2900	|	2993
--->2.33643V	|	2.33828V
	3000	|	3098
--->2.41699V	|	2.42031V
	3100	|	3201
--->2.49756V	|	2.50078V
	3200	|	3301
--->2.57812V	|	2.57891V
	3300	|	3401
--->2.65869V	|	2.65703V
	3400	|	3502
--->2.73926V	|	2.73594V
	3500	|	3603
--->2.81982V	|	2.81484V
	3600	|	3705
--->2.90039V	|	2.89453V
	3700	|	3809
--->2.98096V	|	2.97578V
	3800	|	3915
--->3.06152V	|	3.05859V
	3900	|	4019
--->3.14209V	|	3.13984V
ADC DAC loopback Test Passed
All tests have passed!!
\endcode
