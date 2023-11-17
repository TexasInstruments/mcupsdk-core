# MATHLIB {#MATHLIB_DRIVER}

[TOC]

## Introduction

Trigonometric functions are commonly used in real-time control applications, particularly within the inner loops of control algorithms, where speed and accuracy is essential. The performance of trigonometric functions is a key careabout for designers of these systems as it can have a significant impact on the overall performance of the system. Until recently, trignometric functions based on lookup tables were considered faster than the polynomial-based methods; however, with the inclusion of floating-point units (FPUs) and faster clock speeds, polynomial-based approximations have gained favor. TI has developed C functions of the most commonly used trigonometric functions using these polynomial-based methods and has optimized them for TI's Arm®-based microcontrollers (MCUs) and microprocessors (MPUs).

## Features Supported

Supports Below trignometric functions

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

\cond SOC_AM263PX

## TMU library

MCU PLUS SDK integrates a software library to use the TMU (Trigonometric Math Unit) hardware in AM263Px. This TMU library provides efficient (assembly level functions) and MATHLIB compatible API interface for performing trigonometric calculations, a critical component in many real-time applications such as Motor Control and Digital Power.


### Features Supported:

TMU library supports below trignometric and math functions
- sin, sin_pu
- cos, cos_pu
- atan, atan_pu
- log_pu (base of 2)
- iexp_pu (base of 2)
- atan2


### Features Not Supported:

- div, sqrt and quad operations
- Pipelined operation. 8 result registers for taking advantage of the pipelined architecture of TMU
- Re-entrancy. Context save and restore operation
- Underflow and Overflow interrupts to show mathematical operation errors

### API

\ref DRV_TMU_MODULE

\endcond
