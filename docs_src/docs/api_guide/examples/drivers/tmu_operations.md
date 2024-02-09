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
Max Cycles: 28.000000
Avg cycles: 26.000000

MCU SDK Trig Math Library
Error: 0.0000007150
Max Cycles: 74.000000
Avg cycles: 54.040001

CLANG MATHLIB
Max Cycles: 293.000000
Avg cycles: 150.628006

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
Max Cycles: 26.000000
Avg cycles: 25.998001

MCU SDK Trig Math Library
Error: 0.0000002870
Max Cycles: 64.000000
Avg cycles: 64.000000

CLANG MATHLIB
Max Cycles: 310.000000
Avg cycles: 150.481995

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
Max Cycles: 28.000000
Avg cycles: 26.995998

MCU SDK Trig Math Library
Error: 0.0000004770
Max Cycles: 104.000000
Avg cycles: 94.019997

CLANG MATHLIB
Max Cycles: 187.000000
Avg cycles: 79.524002

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
Max Cycles: 21.000000
Avg cycles: 19.002001

CLANG MATHLIB
Max Cycles: 309.000000
Avg cycles: 150.477997

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
Max Cycles: 20.000000
Avg cycles: 18.006001

CLANG MATHLIB
Max Cycles: 411.000000
Avg cycles: 99.814003

ATAN2 FUNCTION

TMU LIBRARY
Error: 0.0000002380
Max Cycles: 69.000000
Avg cycles: 58.020004

MCU SDK Trig Math Library
Error: 0.0000007150
Max Cycles: 129.000000
Avg cycles: 108.587997

CLANG MATHLIB
Max Cycles: 303.000000
Avg cycles: 143.010010
All tests have passed!!

SINCOS FUNCTION

TMU LIBRARY
Error: 0.0000002980
Max Cycles: 58.000000
Avg cycles: 34.054001

MCU SDK Trig Math Library
Error: 0.0000007150
Max Cycles: 133.000000
Avg cycles: 108.595993

CLANG MATHLIB
Max Cycles: 545.000000
Avg cycles: 116.989998
All tests have passed!!
\endcode

