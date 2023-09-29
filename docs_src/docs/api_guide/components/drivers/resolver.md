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

- Tuning Parameters from the external memory are not supported yet.

## Important Usage Guidelines

- Resolver should be disabled while configuring the blocks. Post the configurations, the Resolver may be enabled.
- DC offset Correction is either from manual offset value or from the estimation loop. when BPF is enabled and DC offset correction (auto) is disabled, it is recommended to turn manual DC offset values to 0. Otherwise, the Signals post BPF will still be added with the manual DC offset values and this can lead to the unexpected / saturated results.
- Auto Ideal Sample detection, if ran, should be performed with DC offset correction off. once the Ideal sample is observed, then the DC offset correction may be turned on.
- In order to use the Phase Gain Estimation loops, the motor should be in rotation.
- Angle from ArcTan and Track2 loops is a signed 16 bit integer, mapping values in -180 degrees to 180 degrees.
- Velocity from the Track2 loop is a signed 32 bit integer, and can be considered angle change from the sampling rate of the Track2 loop, which is either the exictation frequency, if the bottom sampling is turned off, or twice the excitation frequency, if the bottom sampling is turned on.

## Software Programming Guide

The Resolver driver contains flat APIs for individual register configurations and a top level init function, `RDC_init()` to be used with a `RDC_configParams` params.

1. Before using the RDC, please ensure the configurations are valid and the important guidelines are followed.
2. Disable the Resolver before configurations, with `RDC_disableResolver()`. The resolver may be enabled post the configurations using the API, `RDC_enableResolver()`
3. Baseline Parameters are to be initiated using the API, `RDC_BaselineParametersInit()`, for the resolver to operated properly.
4. Configure the Global configurations, like the ADC mode of sampling, ADC soc width, Burst count, sequencer operational mode using the following APIs.
    1. `RDC_enableAdcSingleEndedMode()`,`RDC_disableAdcSingleEndedMode()`
    2. `RDC_setAdcSocWidth()`
    3. `RDC_setAdcSequencerOperationalMode()`
5. Following, The Excitation Frequency and the Sampling Configurations may be set appropriately using the following APIs
    1. `RDC_setExcitationSignalFrequencySelect()`
    2. `RDC_setExcitationSignalPhase()`
    3. `RDC_setExcitationSignalAmplitudeControl()`
    4. `RDC_enableExcitationSignalSyncIn()`, `RDC_disableExcitationSignalSyncIn()`
    5. `RDC_setExcitationSignalSocDelay()`
6. Now configure the each resolver core based on the configured sequencer mode.
7. Configure the Manual DC Offset values to 0 and disable the auto DC offset correction, if DC offset correction has to be disabled. Otherwise, configure the appropriate Offset Calibration Coefficients, Auto Correction enable/disable and Manual correction values via the following API for each core. It is recommended to find the estimated values and replace them into the Manual values and keep the estimation logic for redundancy check.
    1. `RDC_setDcOffsetCalCoef()`
    2. `RDC_disableDcOffsetAutoCorrection()`, `RDC_enableDcOffsetAutoCorrection()`
    3. `RDC_setDcOffsetManualCorrectionValue()`
8. BPF filter can be used for the excitation frequency of 20KHz. If BPF is used, it is recommended to disable the DC offset Correction, as this removes the offsets. The BPF may be enabled/ disabled by the following APIs
    1. `RDC_enableBPF()`
    2. `RDC_disableBPF()`
9. The Ideal Sample Time may be overriden with the Manual Value and the Ideal Sample Mode should be set to Manual. If Manual value is unknown, it is recommended to use the Auto Modes (Modes 0 through 2) for auto Ideal Sample Mode of operation. The following APIs may be used for the Ideal Sample Configurations.
Please Note that the bottom sampling may be used, but the computations now run twice the excitation frequency rate.
    1. `RDC_setIdealSampleMode()`
    2. `RDC_overrideIdealSampleTime()`
    3. `RDC_setIdealSampleDetectionThreshold()`
    4. `RDC_setIdealSampleBpfAdjust()`
    5. `RDC_enableIdealSampleBottomSampling()`, `RDC_disableIdealSampleBottomSampling()`
10. Phase Gain Estimation/ auto correction logic needs the Motor to be rotating. Otherwise, the Manual values may be used for Sine, Cosine Gain Values and the Cosine Phase value. the following APIs may be used for the Phase Gain correction logic. It is recommended to find the estimated values and replace them into the Manual values and keep the estimation logic for redundancy check.
    1. `RDC_setPhaseGainEstimationTrainLimit()`
    2. `RDC_enablePhaseGainEstimation()`, `RDC_disablePhaseGainEstimation()`
    3. `RDC_enablePhaseAutoCorrection()`,`RDC_disablePhaseAutoCorrection()`
    4. `RDC_enableGainAutoCorrection()`,`RDC_disableGainAutoCorrection()`
    5. `RDC_setGainBypassValue()`, `RDC_setCosPhaseBypass()`
11. Track2 Loop is a type 2 tracking loop for the Angle and also outputs Velocity (theta diff at the sampling rate) configure the Kvelfilt value for the velocity filter coefficient, in the stucture Track2Constants_t and use `RDC_setTrack2Constants()` for configuring the track2 constants.
12. Finally, configure the interrupts using the following APIs
    1. `RDC_enableSequencerInterrupt()`, `RDC_disableSequencerInterrupt()`, `RDC_clearSequencerInterrupt()`
    2. `RDC_enableCoreInterrupt()`, `RDC_disableCoreInterrupt()`, `RDC_getCoreEnabledInterruptSources()`, `RDC_clearCoreInterrupt()`
13. the interrupts may be read by the status APIs `RDC_getSequencerInterruptStatus()` and `RDC_getCoreInterruptStatus()`

## Tuning Parameters
Ideal Sample, Phase/Gain correction values are identified to be the Tuning Parameters. For each there are sample(glitch) threshold and train limit achieved flags. for tuning, the software has to configure these, enable the resolver and wait for the train limit done flags. post that, the Phase Gain estimation values may be read (note for the PG estimation to run, the motor has to be in spinning.) and can be used for Manual correction values. The Ideal Sample Mode-Auto will will fill the Peak-Histogram-Buckets until the train limit done. software needs to read these buckets and identify the one with more number, use it as the ideal sample override value with the ideal sample mode set to Manual
## Example Usage

Include the below file to access the APIs
\snippet Resolver_sample.c include

## API

\ref DRV_RESOLVER_MODULE
