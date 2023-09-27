#  Basic Smart Placement {#EXAMPLES_BASIC_SMART_PLACEMENT}

[TOC]

# Introduction

This example provides a basic overview of applying smart placement and compares run time of functions with smart placement and without smart placement.

The Aim of this example is to:
1. Showcase the process of smart placement in simple terms
2. How Smart Placement improves code performance?
3. How Cache Miss ratio is also improved?

More on smart placement can be read at \ref SMART_PLACEMENT

# Supported Combinations {#EXAMPLES_BASIC_SMART_PLACEMENT_COMBOS}

\cond SOC_AM64X
\attention A53 NORTOS support is experimental and is NOT supported by TI. \n
\endcond

\cond SOC_AM64X

 Not Supported

\endcond


\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | >= ti-arm-clang 3.1.0 STS
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/kernel/nortos/basic_smart_placement

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | >= ti-arm-clang 3.1.0 STS
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/kernel/nortos/basic_smart_placement

\endcond

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | >= ti-arm-clang 3.1.0 STS
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/kernel/nortos/basic_smart_placement

\endcond

\cond SOC_AM62X

 Not Supported

\endcond


### Building benchmark application

To build this application, compiler ti-cgt-armllvm >= 3.1.0 is required. Please make sure that `CGT_TI_ARM_CLANG_PATH` variable in imports.mak points to correct path of the compiler. Once this basic path is setup then compiling this program can be achieved using make command.

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code

Profile Point: Without Smart Placement
Cycle Count: 9964
No. Of CPU instructions executed Count: 3660
ICache Miss Count: 227
ICache Access Count: 9973

Profile Point: With Smart Placement
Cycle Count: 3906
No. Of CPU instructions executed Count: 3663
ICache Miss Count: 1
ICache Access Count: 3905
\endcode

# Description

When the program runs, it first calls `basic_smart_placement_main` function which internally calls `function_f1` and `annotated_function_f1`.

PMU has been used for profiling execution of `function_f1` and `annotated_function_f1`.

## Process of smart placement

Following steps can be followed to apply smart placement:

### Critical Function Identification

Here, critical functions are manually found using inspection. Because this is manual process, criticality of a function is determined purely based on the knowledge of software/firmware/usecase.

### Priority Assignment

Following is critical function's priority table.

Function Name           | Priority
------------------------|------------
annotated_function_f1   | 1
annotated_function_f2   | 2
annotated_function_f3   | 2
annotated_function_f4   | 2

Here note that from the manual inspection it has been identified that `annotated_function_f1` is more critical than rest of the functions therefore, it has given more a `priority number` which is smaller than other function's `priority number`. Because rest of the critical functions are all relatively not as of same priority so they are clubbed together by given them same `priority number`.

What `priority number` to assign to which critical function is also a manual process which involves a good understanding of firmware.

### Annotating functions

Once the above table has been formed, each function is annotated as shown:

\code

void __attribute__((local(1))) annotated_function_f1(void);
void __attribute__((local(2))) annotated_function_f2(void);
void __attribute__((local(2))) annotated_function_f3(void);
void __attribute__((local(2))) annotated_function_f4(void);

\endcode

<b>Note:</b>

1. Attributes are added in front, and won't work if following is tried:
\code
void annotated_function_f1(void) __ attribute __((local(1)));
\endcode
2. by specifying local, all these functions are indicated to be placed in local memory.

### Linker change

Here it has to be made sure that following 3 lines are added/present in the SECTIONS of linker.cmd:

\code

    .TI.local   : {} >> R5F_TCMA | R5F_TCMB | MSRAM
    .TI.onchip  : {} >> MSRAM | FLASH
    .TI.offchip : {} > FLASH

\endcode

The above lines basically channeling all the functions that are annotated to be in `local` memory into TCM memory and if total size of the functions that are marked `local` is more than the size of R5F_TCMA then all the functions that could be placed in R5F_TCMA will be placed in R5F_TCMA and rest of functions will be moved in R5F_TCMB and even if it still fills R5F_TCMB then remaining function will be moved to MSRAM.

Similar treatment is for all the functions that are marked `onchip`, however, they should never be placed in any TCM otherwise it will be logically wrong.

Also all functions which are marked `offchip`, should be placed in external FLASH.

## How Smart Placement improves code performance?

From the linker, onchip marked functions are routed to TCMA/B and TCM are fastest memories. Therefore, by annotating the critical functions directly from the source, code is placed in faster memory and hence runtime performance has been improved.

This can be seen from the above sample output. For same number of instructions executed, CPU cycles taken to execute functions without smart placement is 3 times more than with smart placement.

## How Cache Miss ratio is also improved?

Again, TCMs are never cached. Therefore, for the critical functions which are not cached well, placing them in TCM will help in improving cache rate and ultimately CPI of function.


# Conclusion

This example shows how to manually apply smart placement and shows how it improves code performance with ease-of-use because of it allows placement of function at source level.

