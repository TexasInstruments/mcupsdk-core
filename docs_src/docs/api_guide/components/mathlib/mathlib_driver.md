# MATHLIB {#MATHLIB_DRIVER}

[TOC]

## Introduction

Trigonometric functions are commonly used in real-time control applications, particularly within the inner loops of control algorithms, where speed and accuracy is essential. The performance of trigonometric functions is a key careabout for designers of these systems as it can have a significant impact on the overall performance of the system. Until recently, trignometric functions based on lookup tables were considered faster than the polynomial-based methods; however, with the inclusion of floating-point units (FPUs) and faster clock speeds, polynomial-based approximations have gained favor. TI has developed C functions of the most commonly used trigonometric functions using these polynomial-based methods and has optimized them for TI's Arm®-based microcontrollers (MCUs) and microprocessors (MPUs).

## Features Supported

Supports Below trignometric functions (ti_arm_trig):

- Sine
- Cosine
- Sine + Cosine
- Arcsine
- Arccosine
- Arctangent
- Arctangent2

## Features Not Supported

NA

## Additional References {#MATHLIB_ADDITIONAL_REFERENCES}

<a href="https://www.ti.com/lit/pdf/sprad27">App Note: Optimized Trigonometric Functions on TI Arm Cores</a>

## API

\ref DRV_MATHLIB_MODULE

\cond SOC_AM263X

## FastRTS library

Addintionally, TI also offers a traditional Look-up Table based trigonometric functions for users needing the most optimized operation with the trade-off of increase data size (3344Bytes of LUT stored in .trigData). It also bypasses some limitations of the aforementioned TI Arm Trig libary such as the limited input range of 0 - 2PI for sine/cosine functions. Furthermore, the Fast Run-Time-Support (FastRTS) library also offers an optimized math operation of logs and exponents.

## Features Supported

Supports Below trignometric and math functions (fastrts):

- Sine
- Cosine
- Sine + Cosine
- Arcsine
- Arccosine
- Arctangent
- Arctangent2
- Log (base of e)
- Exponent (base of e)

## Features Not Supported

NA

## API

\ref DRV_FASTRTS_MODULE

\endcond

\cond SOC_AM263PX || SOC_AM261X

## FastRTS library

Addintionally, TI also offers a traditional Look-up Table based trigonometric functions for users needing the most optimized operation with the trade-off of increase data size (3344Bytes of LUT stored in .trigData). It also bypasses some limitations of the aforementioned TI Arm Trig libary such as the limited input range of 0 - 2PI for sine/cosine functions. Furthermore, the Fast Run-Time-Support (FastRTS) library also offers an optimized math operation of logs and exponents.

## Features Supported

Supports Below trignometric and math functions (fastrts):

- Sine
- Cosine
- Sine + Cosine
- Arcsine
- Arccosine
- Arctangent
- Arctangent2
- Log (base of e)
- Exponent (base of e)

## Features Not Supported

NA

## API

\ref DRV_FASTRTS_MODULE

\endcond

\cond SOC_AM243X

## FastRTS library

Addintionally, TI also offers a traditional Look-up Table based trigonometric functions for users needing the most optimized operation with the trade-off of increase data size (3344Bytes of LUT stored in .trigData). It also bypasses some limitations of the aforementioned TI Arm Trig libary such as the limited input range of 0 - 2PI for sine/cosine functions. Furthermore, the Fast Run-Time-Support (FastRTS) library also offers an optimized math operation of logs and exponents.

## Features Supported

Supports Below trignometric and math functions (fastrts):

- Sine
- Cosine
- Sine + Cosine
- Arcsine
- Arccosine
- Arctangent
- Arctangent2
- Log (base of e)
- Exponent (base of e)

## Features Not Supported

NA

## API

\ref DRV_FASTRTS_MODULE

\endcond

\cond SOC_AM263PX

## TMU library

MCU PLUS SDK integrates a software library to use the TMU (Trigonometric Math Unit) hardware in AM263Px. This TMU library provides efficient (assembly level functions) and MATHLIB compatible API interface for performing trigonometric calculations, a critical component in many real-time applications such as Motor Control and Digital Power.


### Features Supported:

TMU library supports below trignometric and math functions (ti_arm_tmu):
- sin, sin_pu
- cos, cos_pu
- atan, atan_pu
- log_pu (base of 2)
- log_e_pu (base of e)
- iexp_pu (base of 2)
- iexp_e_pu (base of e)
- atan2
- sincos
- powf using log and exp

Additionally, the TMU assmebly code integrated with the SDK makes use of one operation specific register and it's corresponding result register. To make use of a different
register then the one used by SDK, users can add API calls in the TMU source file pointing to another register. Or they can make a direct write to the Operation specific register
in their application code as follows:

HWREG(x) = input value
/add delay for computation to complete/
result = HWREG(y)
Here, x is the address of the TMU specific operation registers and y is the address of the result register


### Features Not Supported:

- div, sqrt and quad operations
- Underflow and Overflow interrupts to show mathematical operation errors

### API

\ref DRV_TMU_MODULE

\endcond
