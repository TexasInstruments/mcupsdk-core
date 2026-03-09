# AI Examples {#EXAMPLES_AI}

[TOC]

# Introduction

This section contains AI/Machine Learning example applications demonstrating edge AI capabilities on AM26x devices using TI's feature extraction library and TVM-compiled neural network models.

These examples cover various use cases including:
- **Classification**: Detecting and categorizing signals into discrete classes
- **Regression**: Predicting continuous values from sensor data
- **Forecasting**: Predicting future values based on historical time series
- **Anomaly Detection**: Identifying abnormal patterns in sensor data
- **Data Capture**: Collecting sensor data for model training with Edge AI Studio

## Classification Examples
-# \subpage EXAMPLES_AI_GENERIC_TIMESERIES_CLASSIFICATION
-# \subpage EXAMPLES_AI_MOTOR_FAULT
-# \subpage EXAMPLES_AI_ARC_FAULT
-# \subpage EXAMPLES_AI_BLOWER_IMBALANCE

## Regression Examples
-# \subpage EXAMPLES_AI_GENERIC_TIMESERIES_REGRESSION
-# \subpage EXAMPLES_AI_TORQUE_MEASUREMENT
-# \subpage EXAMPLES_AI_WASHING_MACHINE_LOAD_WEIGHING

## Forecasting Examples
-# \subpage EXAMPLES_AI_GENERIC_TIMESERIES_FORECASTING
-# \subpage EXAMPLES_AI_HVAC_INDOOR_TEMP_FORECAST
-# \subpage EXAMPLES_AI_FORECASTING_PMSM_ROTOR_TEMP

## Anomaly Detection Examples
-# \subpage EXAMPLES_AI_GENERIC_TIMESERIES_ANOMALYDETECTION
-# \subpage EXAMPLES_AI_FAN_BLADE_ANOMALYDETECTION

## Data Capture and Live Preview Examples
\cond SOC_AM263X || SOC_AM263PX
-# \subpage EXAMPLES_AI_IMU_SENSOR_DATA_CAPTURE
\endcond
-# \subpage EXAMPLES_AI_TEMP_SENSOR_DATA_CAPTURE
-# \subpage EXAMPLES_AI_DAP_DEFAULT
