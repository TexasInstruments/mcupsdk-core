# Manual Smart Placement {#MANUAL_SMART_PLACEMENT}

[TOC]

## Introduction

This page goes over how to use manual smart placement. Manual smart placement goes with the assumption that the information on function criticality is already known by the developer.

## Steps involved in applying smart placement.

In general, there are following steps involved in applying smart placement:
1. Identification of function criticality.
2. Priority Assignment.
3. Function Annotation.
4. Linker update.

### Identification of function criticality

Here, the developer needs to decide what all functions in the application are critical.

### Priority Assignment

Once critical functions are identified, now it is required to put a priority function. For Manually identified functions, it is known that they are critical functions. So, we can start with assigning highest priority.

\note
Lower number corresponds to higher priority. `Priority` and `Priority Number` are both different. A highest `Priority` function would have lowest `Priority Number`.

\note
Here, function with lower `priority number` will be placed by the linker in the faster memory first.


## Function Annotation

Compiler provides a feature where any function can be annotated which is essentially controlling its placement in memory.

**C/C++ Source-level function attributes:**

    __attribute__(({local,onchip,offchip}(priority)))

Here `local`, `onchip` and `offchip` are corresponding to TCMx, MSRAM/OCRAM, FLASH.

Example:

    void __attribute__((local(1))) func0(void) { .. } // Place in TCM with priority 1

    void __attribute__((local(2))) func1(void) { .. } // Place in TCM with priority 2

    void __attribute__((onchip)) func2(void) { .. } // Place in MSRAM/OCRAM with implied priority 1

The attributes can be added to a function definition or a function declaration (if that function is called/referenced in the same compilation unit).

**Assembly metainfo directives:**

Functions can also be annotated by adding an assembly metainfo directive in an assembly file that is compiled and linked with the project using the following format:

      .global <global function symbol>

      .sym_meta_info <global function symbol>, “of_placement”, {“local”,”onchip”,”offchip”}, <priority>

e.g.

    .global strcmp

    .sym_meta_info strcmp, “of_placement”, “local”, 1


Here in C, strcmp is the symbol name given to function strcmp.

This would allow users to avoid having to compile 3rd party source code.

## Linker update

Linker will aggregate all function input sections into designated output sections while sorting the placement of input sections based on the given priority. Following code can be added in the linker file to inorder to do that.



    SECTIONS

    {

        .TI.local   : {} >> R5F_TCMA | R5F_TCMB | MSRAM

        .TI.onchip  : {} >> MSRAM | FLASH

        .TI.offchip : {} > FLASH

    }


By default, we should use section splitting as shown above between memory regions to get the full effect of function prioritization.

The above linker lines basically channeling all the functions that are annotated to be in `local` memory into TCM memory and if total size of the functions that are marked `local` is more than the size of R5F_TCMA then all the functions that could be placed in R5F_TCMA will be placed in R5F_TCMA and rest of functions will be moved in R5F_TCMB and even if it still fills R5F_TCMB then remaining function will be moved to MSRAM.

Similar treatment is for all the functions that are marked `onchip`, however, they should never be placed in any TCM otherwise it will be logically wrong.

Also, all functions which are marked `offchip`, should be placed in external FLASH.

It should be noted that although annotating using assembler directive is convenient, one short-coming is that static functions cannot be annotated or if annotated, no effect of it is on its placement. To solve this, C/C++ based annotation has to be used in the definition/declaration of that static function.

## Basic Smart Placement Example

\ref EXAMPLES_BASIC_SMART_PLACEMENT

This example provides a basic overview of applying smart placement and compares run time of functions with smart placement and without smart placement.

The Aim of this example is to:
1. Showcase the process of smart placement in simple terms.
2. How Smart Placement improves code performance?
3. How Cache Miss ratio is improved?
