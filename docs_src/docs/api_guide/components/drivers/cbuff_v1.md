# CBUFF {#DRIVERS_CBUFF_PAGE}

[TOC]

The CBUFF (Common Buffer Controller) driver provides API
to configure the CBUFF Module. Below are the high level
features supported by the CBUFF driver.

## Features Supported

- 128-bit device write-only interface over the EDMA writes user data to be sent over the LVDS interface
- Packing of user data into RAW12/RAW14/RAW8 to send data formats DATA12/DATA14/DATA16, respectively.
- SDR and DDR clocking modes.
- Detection of erroneous incomplete transfer when the trigger for the next transfer is received.

## Features NOT Supported

NA

## Important Usage Guidelines

- It is the responsiblity of the application to guarantee that the transfers in the CBUFF sessions do not overlap.

## Example Usage

Include the below file to access the APIs


## API

\ref DRV_CBUFF_MODULE
