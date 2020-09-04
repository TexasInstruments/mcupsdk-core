# MCU Reset Isolation{#MCU_RESET_ISOLATION}

[TOC]

## MCU Domain Reset Isolation: Introduction

\note Reset isolation is applicable only for **warm reset** and not **cold reset**

In AM64x/AM243x devices the MCU domain can be reset isolated from the MAIN domain
in cases where the MCU domain M4 core runs a safety application. The MCU domain
can run a safety application when reset isolated to:

1. Monitor the MAIN domain and reset MAIN domain from MCU domain in case of unexpected errors
2. Run a safety critical firmware completely isolated accessing only MCU domain peripherals

When enabling reset isolation MCU domain can also be debug isolated, restricting
JTAG access to MCU domain.

## Reset Isolation: Developing a reset isolated safety application

### MCU Domain
If MCU domain is isolated from accessing MAIN domain when enabling reset isolation
(MCU domain running a safety critical firware accessing only MCU domain peripherals),
the MCU firmware should not access any MAIN domain peripherals.

\note Peripherals like Mailbox, SecureProxy is in the MAIN domain, so MCU firware should
not use IPC or Sciclient calls if MCU domain is isolated from accessing MAIN domain.

When the MCU domain is monitoring the MAIN domain the MCU domain is not isolated from
accessing MAIN domain peripherals and registers. In such a use case the MCU domain
can access MAIN domain.

\note In all reset isolation use case, on receiving reset request, the MCU domain before
propogating the reset to MAIN domain, will enable isolation of MAIN domain from MCU domain
and MCU domain from MAIN domain.

\note When running safety applications in MCU domain, the MCU domain will be isolated
from the MAIN domain

### MAIN domain
After reset isolation, the MAIN domain firmware should not access any MCU domain peripherals
or registers. Before accessing the MCU domain registers the MAIN domain firmware should
check the for the MCU MAGIC WORD in the mirrored MAIN domain register.

SBL running in MAIN domain also checks for the MAGIC WORD and only conditionally loads and runs
MCU application.
SBL also checks if reset isolation in not enabled and sends a TISCI message to change dev group
to ALL so that MCU domain is also initialized by system firmware.
SBL also turns on the MCU PLL if reset isolation is not enabled.

### System Firmware & Board config
The board config dev group is changed to MAIN Domain only so that System Firmware does not
access the MCU domain registers on bootup.
The dev group is incremented to dev group ALL by SBL on sending a TISCI message, so
that the system firmware iniializes the MCU domain

