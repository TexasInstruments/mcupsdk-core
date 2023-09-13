# RESOLVER {#DRIVERS_RESOLVER_PAGE}

[TOC]

The Resolver driver provides API to configure the Resolver to Digital Converter module.
Below are the high level features supported by the Resolver driver.

## Features Supported

- Excitation Signal configurations
    - Frequency of 5KHz, 10KHz, 20KHz
    - Amplitude
    - Phase
- Single and Differential Ended sampling from the Sin, Cos Channels from the ADC_Rx Channels (x = 0,1)
- Different Sequencer modes to collect samples on sin, cos channels from ADC_Rx Channels (x= 0,1)
- Burst modes of sample collection to be averaged for sequencer mode 0.
- Sync In signal from the Motor control PWM via PWM Xbar
- Sample Signal to SOC start delay
- 2 Resolver Core configurations
    - DC offset Auto Estimation and Manual Correction, Band Pass Filter for the input samples.
    - Phase Gain Auto Estimation and Auto/ Manual Correction on Sin, Cos samples
    - 16 bit singed angle outputs from Arctan and Track2 loops
    - 32 bit singed velocity outputs from Track2 loop.
    - Error flags on multiple safety / redundancy signals.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Input configurations
    - Signal mode
    - ADC SoC width
    - ADC Burst Count
    - Sequencer Mode configurations

- Excitation frequency Configurations
    - Frequncy, Amplitude and Phase
    - Sync In enable/disable
    - ADC SOC delay

- Tuning Parameters
- Each Resolver Core Configurations
    - DC offset / BPF enable
    - Ideal Sample configurations
    - Phase Gain correction block configurations
    - Track2 configurations
- Interrupt Configurations

## Features NOT Supported

Tuning Parameters from the external memory are not supported yet

## Important Usage Guidelines

- Resolver should be disabled while configuring the blocks.

## Example Usage

Include the below file to access the APIs
\snippet Resolver_sample.c include

## API

\ref DRV_RESOLVER_MODULE
