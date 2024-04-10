# ADC Trigger Repeater Oversampling {#EXAMPLES_DRIVERS_ADC_TRIGGER_REPEATER_OVERSAMPLING}

[TOC]

# Trigger repeater module
 The trigger repeater module allows to achieve an oversampling, undersampling or to apply a trigger delay. The repeater modules can select any of the regular ADC triggers that are selectable by ADCSOCxCTL.TRIGGER register and generate a number of repeated pulses as configured in repeater module. The repeater module can apply four types of trigger modifications:
 - Oversample mode - allows multiple back-to-back samples from a single trigger pulse.
 - Undersample mode - scale down the trigger frequency for one or more SOCs.
 - Phase delay - delay the initial trigger by a specified number of SYSCLK.
 - Re-trigger Spread - the additional time between two samples.

# Example Description
This example configures ADC for oversampling using trigger repeater block. The ePWM0 is configured to periodically trigger the ADC1_AIN0 and the trigger repeater module is configured to generate 4 repeated pulses. Post-processing block will take the repeated pulses, increment PPB conversion count and generates an oversampling interrupt (OSINT1) which triggers a CPU interrupt. The accumulated samples will be stored in PPB SUM register.

\imageStyle{adc_trigger_repeater_oversampling.png,width:50%}
\image html ADC_trigger_repeater_oversampling.png "ADC Tigger Repeater Oversampling"

## ADC Configurations
1. SOC0 is triggered by EPWMSOCA. Sample window is set at 16
2. SOC0 samples on channel 0 and trigger source is set as repeated trigger
3. Trigger repeater module 1 is configured for oversampling
4. Trigger count (N) is set to 3, so it will generate N+1 repeated pulses

## ISR Configuration
1. INTXbar0 is set for EPWM0INT1
2. App_epwmISR services this interrupt
    - reads ADC OSINT flag to check if oversampling is done

# External Connections
ADC1_AIN0, ADC1_AIN1 pins should be connected to signals to be converted.
 - on AM263Px CC E1, with HSEC Dock
     - Feed Analog input to ADC1_AIN0 - HSEC PIN 12
 - on AM263Px LP
     - Feed Analog input to ADC1_AIN0 - J1/3 Pin 24

# Supported Combinations {#EXAMPLES_DRIVERS_ADC_TRIGGER_REPEATER_OVERSAMPLING_COMBOS}

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/adc/adc_trigger_repeater_oversampling/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section.
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Using the watch variables, view the ADC conversion results.
- View the ADC conversion results in UART console logs

# See Also

\ref DRIVERS_ADC_PAGE

# Sample Output

Shown below is a sample output when the application is running,

\code
ADC Trigger Repeater Oversampling Test Started...
ADC Trigger Repeater Oversampling results:
Trigger Repeater Oversampling Test Passed!!
All tests have passed!!
\endcode
