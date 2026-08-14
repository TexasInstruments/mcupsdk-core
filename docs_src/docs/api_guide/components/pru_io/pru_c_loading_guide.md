# Programmatically Loading PRU C Applications from R5F Core {#COMPONENT_PRU_C_LOADING_GUIDE}

[TOC]

## Introduction

This guide addresses runtime hangs in `__TI_decompress_lzss()` and managing memory section initialization when loading PRU C applications programmatically from R5F core.

### Background

When downloading a PRU firmware binary via JTAG/CCS, the debugger automatically loads all initialized sections across multiple memory pages (Instruction RAM / `PAGE 0`, Data RAM / `PAGE 1`, and Shared RAM / `PAGE 2`).

However, when loading programmatically from an ARM Cortex-R5F application using standard SDK calls like `PRUICSS_loadFirmware()`:

- `PRUICSS_loadFirmware()` typically writes only the Instruction RAM (`PAGE 0` / `PRU_IMEM`) and assumes execution begins at address `0x0000_0000`.
- **Uninitialized `.cinit` or `.data`:** In C projects built with `--rom_model`, the runtime initialization routine (`_c_int00`) calls `__TI_decompress_lzss()` or `__TI_auto_init()` to decompress tables stored in `.cinit` (which resides in Data RAM). If Data RAM (`PAGE 1`) was never written by the R5F loader, the decompression pointers point to garbage/all-zeros, throwing the PRU core into an infinite decompression loop.

## Choosing the Initialization Strategy

According to SPRUHV6 (Section *Linker Autoinitialization*), you have two primary linkage strategies:

| Parameter | RAM Model (`--ram_model` / `-cr`) [Recommended] | ROM Model (`--rom_model` / `-c`) |
|-----------|------------------------------------------------|----------------------------------|
| **Initialization Actor** | Host Core (R5F writes variables to DRAM directly at load time). | PRU Boot Code (`_c_int00` decompresses tables from `.cinit` at runtime). |
| **Data Sections** | `.data` is placed in DRAM and pre-populated by R5F. `.cinit` is empty. | `.cinit` contains compressed tables; `.data` is uninitialized in the file. |
| **PRU Runtime Overhead** | Zero autoinitialization overhead; boots straight to `main()`. | Consumes PRU cycles decompressing memory before `main()`. |

## Step-by-Step Implementation

\note For projects based on the `empty_c` reference example available at [https://github.com/TexasInstruments/open-pru/tree/main/examples/empty_c](https://github.com/TexasInstruments/open-pru/tree/main/examples/empty_c), **Step 1 and Step 2 may be omitted** as the template includes pre-configured linker settings and memory initialization structures. The `empty_c` template is the **recommended starting point** for PRU C application development and inherently addresses the initialization requirements described in this document.

### Step 1: PRU Linker Command File (linker.cmd)

Ensure entry symbol `_c_int00*` is explicitly pinned to `0x0000_0000` in Instruction RAM (PAGE 0) so execution begins at entry point reset:

\code
SECTIONS
{
    /* Force boot code to reset vector 0x0 */
    .text:_c_int00*   > PRU_IMEM, PAGE 0, origin = 0x00000000
    .text             > PRU_IMEM, PAGE 0
    .stack            > PRU0_DMEM_0, PAGE 1
    .bss              > PRU0_DMEM_0, PAGE 1
    .data             > PRU0_DMEM_0, PAGE 1
    .cinit            > PRU0_DMEM_0, PAGE 1
    .rodata           > PRU0_DMEM_0, PAGE 1
}
\endcode

### Step 2: Generate Multi-Page C-Arrays via Hex Utility (hexpru)

Right click on the project -> Go to Properties -> Select build option and add below code in post build steps of the project:

\code
$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=PRUFirmware -o ../pru0_load_bin.h ${BuildArtifactFileBaseName}.out
\endcode

This generates separate C header arrays:

- `PRUFirmware_0[]` → PAGE 0: PRU Instruction RAM (Code)
- `PRUFirmware_1[]` → PAGE 1: PRU Data RAM (`.data` or `.cinit`)
- `PRUFirmware_2[]` → PAGE 2: Shared RAM (if data placed in PAGE 2)

### Step 3: Programmatic Loading Routine in R5F Core

Instead of standard single-section `PRUICSS_loadFirmware()`, write all generated page arrays explicitly using `PRUICSS_writeMemory()` before releasing the core from reset.

\code
#include <drivers/pruicss.h>
#include "pru0_load_bin.h" // It has C header arrays related to PAGE 0 (IMEM), PAGE 1 (DMEM), PAGE 2 (SMEM)

void load_and_run_pru(PRUICSS_Handle pruicssHandle, uint8_t pruCoreNum)
{
    /* 1. Halt/Disable the PRU core before loading */
    PRUICSS_disableCore(pruicssHandle, pruCoreNum);

    /* 2. Clear Instruction and Data RAM if needed */
    PRUICSS_resetCore(pruicssHandle, pruCoreNum);

    /* 3. Write PAGE 0 - Instruction RAM (IRAM) */
    PRUICSS_writeMemory(
        pruicssHandle,
        PRUICSS_IRAM_PRU(pruCoreNum),
        0x00000000,
        (uint32_t *)PRUFirmware_0,
        sizeof(PRUFirmware_0)
    );

    /* 4. Write PAGE 1 - Data RAM (DRAM0 / .data / .cinit) */
    PRUICSS_writeMemory(
        pruicssHandle,
        PRUICSS_DATARAM(pruCoreNum),
        0x00000000,
        (uint32_t *)PRUFirmware_1,
        sizeof(PRUFirmware_1)
    );

    /* 5. Clear Core Reset and Enable/Start PRU Core */
    PRUICSS_resetCore(pruicssHandle, pruCoreNum);
    PRUICSS_enableCore(pruicssHandle, pruCoreNum);
}
\endcode

## Verification Checklist

1. Linker flag set to `--ram_model` (preferred) or ensure `PAGE 1 (.cinit)` is loaded.
2. Linker command file explicitly forces `.text:_c_int00*` to `0x00000000` on `PAGE 0`.
3. `hexpru` generated arrays for all active memory pages.
4. R5F firmware writes both `PRU_IRAM_PRU` and `PRU_DRAM_PRU` memory regions.
5. Core enabled only after all memory write transactions complete.
