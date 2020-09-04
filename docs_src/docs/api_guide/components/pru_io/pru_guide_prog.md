# Programming {#COMPONENT_PRU_GUIDE_PROG}

[TOC]

## PRU Firmware Development

### Supported Languages

PRU firmware may be written in C code, or in assembly code, or can be a combination of both in a single example.

Refer for in detail reference:
* Instructions available: [PRU Assembly Instruction User Guide](https://www.ti.com/lit/ug/spruij2/spruij2.pdf)
* Assembly Coding: [PRU Assembly Language Tools](https://www.ti.com/lit/ug/spruhv6c/spruhv6c.pdf)
* C Coding: [PRU Optimizing C/C++ Compiler](https://www.ti.com/lit/ug/spruhv7c/spruhv7c.pdf)

### Example Project

To get started, follow along the empty example project included in the SDK at `<sdk>/examples/pru_io/empty/`, documentation at: \ref EXAMPLES_PRU_EMPTY.

### Building Project

\note The PRU projects need [TI-PRU-CGT](https://www.ti.com/tool/PRU-CGT) to compile the sources into executable binary.

## PRU-ICSS Driver APIs (- R5F side)

The following APIs are available for R5F core to initialize, manage and control PRU core. Refer \ref DRIVERS_PRUICSS_PAGE for more details.
#### Overview of APIs

* Module-level Initialization and Deinitialization
    - APIs : `PRUICSS_init()`, `PRUICSS_deinit()`
    - Called automatically by SysConfig generated code if PRU-ICSS module is instantiated in SysConfig
    - `PRUICSS_init()` needs to be called before calling any other APIs

* Handle Initialization and Deinitialization
    - APIs : `PRUICSS_open()`, `PRUICSS_close()`
    - `PRUICSS_open()` function creates the handle for a PRUICSS instance. This handle is required for calling other APIs
    - `PRUICSS_close()` is used for deleting the handle

* Interrupt Controller Module Initialization
    - APIs : `PRUICSS_intcInit()`
    - This function does Interrupt-Channel-Host mapping
    - The interrupt controller (INTC) module maps interrupts coming from different parts of the device (mapped to PRUICSS instance) to a reduced set of interrupt channels.
    - More details
        - Section in MCU+ SDK Docs : https://software-dl.ti.com/mcu-plus-sdk/esd/AM64X/08_01_00_36/exports/docs/api_guide_am64x/DRIVERS_PRUICSS_PAGE.html#PRUICSS_INTC
        - TRM : Section _"6.4.7 PRU_ICSSG Local INTC"_ in Technical Reference Manual(TRM) of AM64x for more details on INTC module

* PRU Core Control – Enable, Disable, Reset
    - APIs : `PRUICSS_enableCore()`, `PRUICSS_disableCore()`, `PRUICSS_resetCore()`

* Memory Management APIs
    - APIs : `PRUICSS_initMemory()`, `PRUICSS_writeMemory()`, `PRUICSS_readMemory()`
    - Init API sets the memory to zero
    - APIs can be used for any of the following memories
        - Data RAM0/RAM1
        - IRAM for PRU0/PRU1/TX_PRU0/TX_PRU1/RTU_PRU0/RTU_PRU1
            - Note : This can not be updated if the respective core is enabled
        - Data RAM2 (Shared RAM)
    - One example usage is to load the IRAM using the protocol firmware headers provided in .h file format

* Event Management APIs
    - `PRUICSS_registerIrqHandler()` : This function registers an Interrupt Handler for an event
    - `PRUICSS_sendEvent()` : This function generates an INTC event
    - `PRUICSS_waitEvent()` : This function waits for a system event to happen
    - `PRUICSS_clearEvent()` : This function clears an INTC event
    - `PRUICSS_sendWaitClearEvent()` : This function generates an INTC event, waits for INTC event and clears an INTC event

* Information APIs
    - `PRUICSS_getVersion()` : Get PRUICSS version number from ICSSCFG_REVID register
    - `PRUICSS_readEfuse()` : Read the state of efuse bits from ICSSG_HWDIS_REG register
    - `PRUICSS_getAttrs()` : Return PRUICSS attributes like base addresses of different modules, their sizes, etc.

* Configuration APIs
    - IEP:
        - `PRUICSS_setIepClkSrc()` : Configure the source of the IEP clock to be either IEP CLK as the source or ICSSGn_CORE_CLK
        - `PRUICSS_controlIepCounter()` : Enable/Disable the counter in IEP module
        - `PRUICSS_setIepCounterIncrementValue()` : Set the default increment value for counter in IEP module

    - Constant Table:
        - `PRUICSS_setConstantTblEntry()` : Update the constant table for specified constant table entry which have write permissions

    - Others:
        - `PRUICSS_setGpMuxSelect()` : Set the GP Mux Select mode (GP/EnDat/MII/SD) for a specific PRU
        - `PRUICSS_setGpiMode()` : Sets the GPI mode for a specific PRU
        - `PRUICSS_setSaMuxMode()` : Set the G_MUX_EN mux field in ICSSG_SA_MX_REG register
        - `PRUICSS_configureCycleCounter()` : Enable/Disable the PRU Cycle Counter for a core
        - `PRUICSS_setIcssCfgMiiMode()` : Set the MII mode (MII/RGMII/SGMII) in MII_G_RT_ICSS_G_CFG register
        - `PRUICSS_setIcssCfgTxFifo()` : Configure the TX FIFO (L1/L2) to be used
