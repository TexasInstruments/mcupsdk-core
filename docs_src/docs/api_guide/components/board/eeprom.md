# EEPROM {#BOARD_EEPROM_PAGE}

[TOC]

The EEPROM driver provides API to read and write to I2C based EEPROM devices present in the board.
The driver takes care of all sequencing necessary to perform writes across pages and
the application need not take care of the programming intricacies.

## Features Supported

- Read and write API from any offset
- Provide API to return EEPROM attributes like size, page size etc

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Option to select EEPROM type based on board
- Supported EEPROMs
\cond SOC_AM64X || SOC_AM243X
        - AT24C
\endcond
\cond SOC_AM263X
        - CAT24M
\endcond
\cond SOC_AM273X || SOC_AWR294X
        - CAV24C
\endcond
- Option to set the I2C address of the EEPROM

## Features NOT Supported

NA

## Important Usage Guidelines

None

## Example Usage

Include the below file to access the APIs
\snippet Eeprom_sample.c include

EEPROM Read API
\snippet Eeprom_sample.c read

EEPROM Write API
\snippet Eeprom_sample.c write

## API

\ref BOARD_EEPROM_MODULE
