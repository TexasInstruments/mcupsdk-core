# ESM {#DRIVERS_ESM_PAGE}

[TOC]

The Error Signaling Module (ESM) driver collects and reports the various error conditions on the microcontroller.
The error condition is categorized based on a severity level. Error response is then generated based on
the category of the error. Possible error responses include a low priority interrupt, high priority interrupt,
and an external pin action.

## Features Supported

- Configuration of error number.
- Registering ESM notifier.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Selection of ESM instances.
- Option to configure the error number, priority level, enable failure influence and call-back function.
-  ESM register notifier/ ESM de register notifier API integration with SYSCFG
- Based on above parameters, the SysConfig generated code does ESM instance parameter configuration and registers
  the notifier as part of Drivers_open and Drivers_close functions.

## Features NOT Supported

- NA

## Usage Overview

### API Sequence

To use the ESM driver to report various error conditions:

- #ESM_init(): Initialize the ESM driver.
- #ESM_Params_init():  Initialize a #ESM_OpenParams structure with default
   values. Then change the parameters from non-default values as needed.
- #ESM_open():  Initialize the driver and register the notifier.
- #ESM_close():  De-initialize the ESM instance and de register the notifier.
- #ESM_deinit(): De-Initialize the ESM driver.

## Example Usage

Include the below file to access the APIs
\snippet Esm_sample.c include

Instance Open Example
\snippet Esm_sample.c open

Instance Close Example
\snippet Esm_sample.c close

## API

\ref DRV_ESM_MODULE
