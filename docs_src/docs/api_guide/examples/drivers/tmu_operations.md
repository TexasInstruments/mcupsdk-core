# TMU Operations and Benchmarking {#EXAMPLES_DRIVERS_TMU_OPERATIONS}

[TOC]

# Introduction

This example calls the mathlib operations for 500 different input values.

- Calls the math library operations with TMU mathlib APIs, SDK mathlib APIs and CLANG mathlib APIs
- Compares the performance between these three versions and prints the time taken and the error check in comparison to the CLANG mathlib APIs

The operations used with this examples include:

1. SIN - Computes the trigonometric sin value of input angle. The input angle is given in radians and is within the range of 0 to 2PI.
2. COS - Computes the trigonometric cosine value of the input angle. The input angle is given in radians and is within the range of 0 to 2PI.
3. ATAN - Computes the trigonometric atan value of the input angle. The input angle is provided in radians
4. IEXP - Computes the exponential value of the input. The input is in float and is within the range of negative infinity to positive infinity
5. LOG - computes the logarithmic value of the input. The input is in float and is within the range of negative infinity to positive infinity
6. ATAN2 - computes the trigonometric atan2 value of input. This function takes two inputs x and y

Additionally, apart from these 500 test cases the example also computes the output of these operations when dealt with corner edge cases and draws a
comparison between the results of TMU mathlib APIs, SDK mathlib APIs and CLANG mathlib APIs.


# Supported Combinations {#EXAMPLES_DRIVERS_TMU_OPERATIONS_COMBOS}

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/tmu/tmu_operations/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- View the console log for results


# Sample Output

Shown below is a sample output when the application is run,

\code
TMU operations example Started ...

SIN FUNCTION
Clang mathlib sin
sin function mathlib input (POS_INFINITY) inf : nan
sin function mathlib input (NEG_INFINITY) -inf : nan
sin function mathlib input (+NaN) nan : nan
sin function mathlib input (-NaN) nan : nan
MCU SDK SIN
sin function sdk input (POS_INFINITY) inf : nan
sin function sdk input (NEG_INFINITY) -inf : nan
sin function sdk input (+NaN) nan : nan
sin function sdk input (-NaN) nan : nan
TMU SIN
sin function tmu input (POS_INFINITY) inf : 0.000000
sin function tmu input (NEG_INFINITY) -inf : 0.000000
sin function tmu input (+NaN) nan : 0.000000
sin function tmu input (-NaN) nan : 0.000000

TMU LIBRARY
Error: 0.0000004320
Max Cycles: 72.000000
Avg cycles: 33.080002

MCU SDK Trig Math Library
Error: 0.0000007150
Max Cycles: 80.000000
Avg cycles: 54.051998

CLANG MATHLIB
Max Cycles: 263.000000
Avg cycles: 150.410004

COS FUNCTION
Clang mathlib cos
cos function mathlib input (POS_INFINITY) inf : nan
cos function mathlib input (NEG_INFINITY) -inf : nan
cos function mathlib input (POS_NaN) nan : nan
cos function mathlib input (NEG_NaN) nan : nan
MCU SDK COS
cos function sdk input (POS_INFINITY) inf : nan
cos function sdk input (NEG_INFINITY) -inf : nan
cos function sdk input (POS_NaN) nan : nan
cos function sdk input (NEG_NaN) nan : nan
TMU COS
cos function tmu input (POS_INFINITY) inf : 1.000000
cos function tmu input (NEG_INFINITY) -inf : 1.000000
cos function tmu input (POS_NaN) nan : 1.000000
cos function mathlib input (NEG_NaN) nan : 1.000000

TMU LIBRARY
Error: 0.0000003780
Max Cycles: 70.000000
Avg cycles: 33.077999

MCU SDK Trig Math Library
Error: 0.0000002870
Max Cycles: 83.000000
Avg cycles: 64.075996

CLANG MATHLIB
Max Cycles: 290.000000
Avg cycles: 150.673996

ATAN FUNCTION
Clang mathlib atan
atan function mathlib input (ZERO/ZERO) 0.000000 0.000000 : nan
atan function mathlib input (ZERO/INFINITY) inf 0.000000 : 0.000000
atan function mathlib input (INFINITY/ZERO) 0.000000 inf : 1.570796
atan function mathlib input (INFINITY/INFINTY) inf inf : nan
atan function mathlib input (INFINITY/NORMAL) 3.402823e+38 inf : 1.570796
atan function mathlib input (NORMAL/ZERO) 0.000000 3.402823e+38 : 1.570796
atan function mathlib input (NORMAL/INFINITY) inf 3.402823e+38 : 0.000000
MCU SDK ATAN
atan function sdk input (ZERO/ZERO) 0.000000 0.000000 : nan
atan function sdk input (ZERO/INFINITY) 0.000000 inf : 0.000000
atan function sdk input (INFINITY/ZERO) inf 0.000000 : 1.570796
atan function sdk input (INFINITY/INFINTY) inf inf : nan
atan function sdk input (INFINITY/NORMAL) inf 3.402823e+38 : 1.570796
atan function sdk input (NORMAL/ZERO) 3.402823e+38 0.000000 : 1.570796
atan function sdk input (NORMAL/INFINITY) 3.402823e+38 inf : 0.000000
TMU ATAN
atan function tmu input (ZERO/ZERO) 0.000000 0.000000 : 0.785398
atan function tmu input (ZERO/INFINITY) 0.000000 inf : 0.000000
atan function tmu input (INFINITY/ZERO) inf 0.000000 : 0.000000
atan function tmu input (INFINITY/INFINTY) inf inf : 0.785398
atan function tmu input (INFINITY/NORMAL) inf 3.402823e+38 : 0.785398
atan function tmu input (NORMAL/ZERO) 3.402823e+38 0.000000 : 0.785398
atan function tmu input (NORMAL/INFINITY) 3.402823e+38 inf : 0.000000

TMU LIBRARY
Error: 0.0000000600
Max Cycles: 102.000000
Avg cycles: 42.119999

MCU SDK Trig Math Library
Error: 0.0000004770
Max Cycles: 94.000000
Avg cycles: 93.951996

CLANG MATHLIB
Max Cycles: 185.000000
Avg cycles: 79.599998

EXPONENTIAL FUNCTION
Clang mathlib exp
exp function mathlib input (POS_INFINITY) inf : 0.000000
exp function mathlib input (NEG_INFINITY) -inf : 0.000000
exp function mathlib input (POS_NaN) nan : nan
exp function mathlib input (NEG_NaN) nan : nan
exp function mathlib input (INF/POS_NaN) inf : 0.000000
exp function mathlib input (-INF/POS_NaN) -inf : 0.000000
TMU EXP
exp function tmu input (POS_INFINITY) inf : 0.000000
exp function tmu input (NEG_INFINITY) -inf : 0.000000
exp function tmu input (POS_NaN) nan : 0.000000
exp function tmu input (NEG_NaN) nan : 0.000000
exp function tmu input (INF/POS_NaN) inf : 0.000000
exp function mathlib input (-INF/POS_NaN) -inf : 0.000000

TMU LIBRARY
Error: 0.0000000600
Max Cycles: 39.000000
Avg cycles: 29.028002

CLANG MATHLIB
Max Cycles: 327.000000
Avg cycles: 150.514008

LOG FUNCTION
Clang mathlib log
log function mathlib input (POS_INFINITY) inf : inf
log function mathlib input (NEG_INFINITY) -inf : nan
log function mathlib input (POS_NaN) nan : nan
log function mathlib input (NEG_NaN) nan : nan
log function mathlib input (POS_INF/POS_NAN) inf nan : nan
log function mathlib input (NEG_INF/POS_NAN) -inf nan : nan
TMU LOG
log function tmu input (POS_INFINITY) inf : inf
log function tmu input (NEG_INFINITY) -inf : -inf
log function tmu input (POS_NaN) nan : inf
log function tmu input (NEG_NaN) nan : -inf
log function tmu input (inf/nan) inf nan : inf
log function tmu input (-inf/nan) -inf nan : -inf

TMU LIBRARY
Error: 0.0000009540
Max Cycles: 31.000000
Avg cycles: 29.004002

CLANG MATHLIB
Max Cycles: 178.000000
Avg cycles: 99.178001

ATAN2 FUNCTION

TMU LIBRARY
Error: 0.0000002380
Max Cycles: 100.000000
Avg cycles: 61.071999

MCU SDK Trig Math Library
Error: 0.0000007150
Max Cycles: 120.000000
Avg cycles: 108.584000

CLANG MATHLIB
Max Cycles: 285.000000
Avg cycles: 139.928009
All tests have passed!!
\endcode

